"""Fixed opening action tests use virtual time and mock hands only."""
import sys
from enum import Enum
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from adapters.litchibot import open_hand
from adapters.litchibot.open_hand import Opening, SharpaOpenController, checked_feedback
from adapters.litchibot.retarget import JOINT_NAMES, JOINT_LIMITS

START = np.minimum(np.full(22, .4), JOINT_LIMITS[:, 1])


@pytest.fixture
def action(monkeypatch):
    clock = SimpleNamespace(now=10.0)
    monkeypatch.setattr(open_hand, 'time', SimpleNamespace(monotonic=lambda: clock.now))
    class Side(Enum):
        LEFT = 'left'
        RIGHT = 'right'
    class Message:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id='')
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', SimpleNamespace(JointState=Message))
    hands = {side: SimpleNamespace(position=START.copy(), reads=0) for side in Side}
    for hand in hands.values():
        def read(h=hand):
            h.reads += 1
            return SimpleNamespace(position=h.position.copy())
        hand.read_joint_state = read
    node = SimpleNamespace(_hands=hands, stop_validation=Mock(), prepare_open=Mock())
    def ros_now():
        ns = int(clock.now * 1e9)
        return SimpleNamespace(nanoseconds=ns, to_msg=lambda: SimpleNamespace(sec=ns//10**9, nanosec=ns%10**9))
    node.get_clock = lambda: SimpleNamespace(now=ros_now)
    runtime = SimpleNamespace(node=node, config=dict(source='manus', mode='fake', token='a'*64),
        latch=SimpleNamespace(phase='READY', reason='', last_gui=clock.now, feedback_limit_tolerance=.02,
                             requested={'left': True, 'right': True}),
        publishers={s: Mock() for s in ('left', 'right')}, stop_hardware_side=Mock())
    control = SharpaOpenController(runtime)
    runtime.opening = control
    def request(side, enabled):
        control.cancel(side)
        runtime.latch.requested[side] = enabled
        runtime.stop_hardware_side(side)
    runtime.request = request
    sent = []
    def forward(side, message):
        control.authorize(side.value, message)
        hands[side].position = np.array(message.position)
        sent.append((side.value, np.array(message.position)))
    node.forward_open = forward
    return control, runtime, hands, clock, sent


@pytest.mark.parametrize('side,selected', [('left', {'left'}), ('right', {'right'}), ('both', {'left','right'})])
def test_open_reaches_defaults_without_restoring_following(action, side, selected):
    control, runtime, hands, clock, sent = action
    control.start(side)
    assert {s for s,v in runtime.latch.requested.items() if not v} == selected
    assert set(control.operations) == selected
    for _ in range(70):
        runtime.latch.last_gui = clock.now
        control.tick()
        clock.now += .04
    assert not control.operations
    assert {s for s,q in sent} == selected
    for enum, hand in hands.items():
        if enum.value in selected:
            np.testing.assert_array_equal(hand.position, np.zeros(22))
            assert not runtime.latch.requested[enum.value]
            assert '已张开' in control.results[enum.value]
        else:
            np.testing.assert_array_equal(hand.position, START)
            assert runtime.latch.requested[enum.value]
    runtime.node.stop_validation.assert_not_called()
    for s in selected:
        q = [v for k,v in sent if k == s]
        assert all(np.max(np.abs(b-a)) <= .5*.04 + 1e-9 for a,b in zip(q,q[1:]))


@pytest.mark.parametrize('selection', ['left', 'right', 'both'])
@pytest.mark.parametrize('residual', [.174, .218, .25, .251])
def test_coarse_opening_settles_only_inside_tolerance(action, selection, residual):
    control, runtime, hands, clock, sent = action
    selected = {'left','right'} if selection == 'both' else {selection}
    original = runtime.node.forward_open
    joint = JOINT_NAMES.index('thumb_IP')
    def forward(side, message):
        original(side, message)
        # The SDK accepts the target, but measured position retains an offset.
        hands[side].position[joint] += residual
    runtime.node.forward_open = forward
    control.start(selection)
    for _ in range(160):
        clock.now += .04
        runtime.latch.last_gui = clock.now
        control.tick()
        if not control.operations:break
    assert not control.operations
    assert all(not runtime.latch.requested[s] for s in selected)
    assert {s for s, q in sent} == selected
    if residual <= .25:
        runtime.node.stop_validation.assert_not_called()
        assert all('已张开' in control.results[s] for s in selected)
        for side, hand in hands.items():
            if side.value in selected:assert hand.position[joint] == pytest.approx(residual)
    else:
        runtime.node.stop_validation.assert_called_once()
        reason = runtime.node.stop_validation.call_args.args[0]
        assert 'did not settle before timeout' in reason
        assert 'joint=thumb_IP' in reason and 'error=0.25100 rad' in reason
        assert 'tolerance=0.25000 rad' in reason


@pytest.mark.parametrize('failure', ['lease', 'fault', 'feedback', 'sender', 'settle'])
def test_open_failure_never_bypasses_existing_stop(action, failure):
    control, runtime, hands, clock, sent = action
    control.start('both')
    clock.now += .01
    runtime.latch.last_gui = clock.now
    control.tick()
    sent.clear()
    clock.now += .04
    if failure == 'lease':clock.now += .21
    elif failure == 'fault':runtime.latch.phase = 'FAULT';runtime.latch.reason='existing fault'
    elif failure == 'feedback':next(iter(hands.values())).position[0] = float('nan')
    elif failure == 'sender':runtime.node.forward_open = Mock(side_effect=RuntimeError('sender failed'))
    else:clock.now += 6;runtime.latch.last_gui = clock.now
    control.tick()
    assert not control.operations
    assert not any(runtime.latch.requested.values())
    runtime.node.stop_validation.assert_called_once()
    assert not sent


def test_cancel_only_selected_action_and_reject_forged_target(action):
    control, runtime, hands, clock, sent = action
    control.start('both')
    runtime.request('left', False)
    assert set(control.operations) == {'right'}
    job = control.operations['right'];job.prepared_at = None;job.desired = job.target(clock.now)
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=0),
        frame_id='supervisor_open:right:'+'a'*16), name=list(JOINT_NAMES), position=[.8]*22)
    with pytest.raises(ValueError, match='fixed action'):
        control.authorize('right', message)
    assert not sent


@pytest.mark.parametrize('side', ['invalid', None])
def test_invalid_side_does_not_change_permits(action, side):
    control, runtime, hands, clock, sent = action
    with pytest.raises(ValueError):control.start(side)
    assert all(runtime.latch.requested.values())
    assert not control.operations and not sent


def test_no_click_means_no_action_or_feedback_reads(action):
    control, runtime, hands, clock, sent = action
    control.tick()
    assert all(h.reads == 0 for h in hands.values())
    assert not sent
    runtime.stop_hardware_side.assert_not_called()


@pytest.mark.parametrize('selection', ['left', 'right', 'both'])
def test_slow_native_preparation_waits_for_new_lease_before_any_send(action, selection):
    control, runtime, hands, clock, sent = action
    selected = {'left', 'right'} if selection == 'both' else {selection}
    def enable(side):
        clock.now += .35  # Real SDK enable polling can exceed the 200 ms lease.
    runtime.node.prepare_open.side_effect = enable
    control.start(selection)
    assert not sent
    assert {c.args[0].value for c in runtime.node.prepare_open.call_args_list} == selected
    assert all(not runtime.latch.requested[s] for s in selected)
    control.tick()
    assert not sent and set(control.operations) == selected
    runtime.node.stop_validation.assert_not_called()
    # A delayed heartbeat from before preparation cannot authorize movement.
    clock.now += .04
    runtime.latch.last_gui = clock.now - .1
    control.tick()
    assert not sent
    # Re-sample the actual hand pose after hardware preparation.
    for side, hand in hands.items():
        if side.value in selected:hand.position = START * .5
    runtime.latch.last_gui = clock.now
    control.tick()
    assert {s for s, q in sent} == selected
    for s, q in sent:np.testing.assert_array_equal(q, START * .5)
    for _ in range(70):
        clock.now += .04
        runtime.latch.last_gui = clock.now
        control.tick()
    assert not control.operations
    assert all('已张开' in control.results[s] for s in selected)
    runtime.node.stop_validation.assert_not_called()


@pytest.mark.parametrize('selection', ['left', 'right', 'both'])
def test_missing_post_preparation_heartbeat_still_stops_without_motion(action, selection):
    control, runtime, hands, clock, sent = action
    runtime.node.prepare_open.side_effect = lambda side: setattr(clock, 'now', clock.now + .35)
    control.start(selection)
    clock.now += .201
    control.tick()
    assert not sent and not control.operations
    runtime.node.stop_validation.assert_called_once()


def test_prepare_failure_stops_both_without_any_command(action):
    control, runtime, hands, clock, sent = action
    runtime.node.prepare_open.side_effect = [None, RuntimeError('enable failed')]
    with pytest.raises(RuntimeError, match='enable failed'):
        control.start('both')
    assert not sent and not control.operations
    assert not any(runtime.latch.requested.values())
    assert runtime.stop_hardware_side.call_count == 4


@pytest.mark.parametrize('values', [[0.0]*21, [float('nan')]*22, [3.0]*22])
def test_invalid_feedback_cannot_define_opening(values):
    with pytest.raises(ValueError):checked_feedback(values)


def test_opening_target_is_bounded_and_never_exceeds_speed():
    start = np.full(22, 1.5)
    job = Opening(start, 0, 3)
    np.testing.assert_array_equal(job.target(-1), start)
    np.testing.assert_array_equal(job.target(4), np.zeros(22))
    assert np.max(np.abs(job.target(1.1)-job.target(1))) <= .05 + 1e-9
