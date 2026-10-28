"""Explicit MANUS recovery never restores motion and requires live input."""
import sys
from enum import Enum
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from adapters.manus import normal_hand_runtime as module
from adapters.manus.normal_hand_runtime import ManusHandRuntime, ManusForwardingAuthority
from adapters.litchibot.open_hand import SharpaOpenController
from adapters.litchibot.retarget import JOINT_NAMES


@pytest.fixture
def recovery(monkeypatch):
    clock = SimpleNamespace(now=10.)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock.now)
    class Side(Enum):
        LEFT = 'left'
        RIGHT = 'right'
    class Message:
        def __init__(self):self.header = SimpleNamespace(stamp=None, frame_id='')
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', SimpleNamespace(JointState=Message))
    runtime = ManusHandRuntime.__new__(ManusHandRuntime)
    runtime.config = dict(source='manus', mode='fake', token='a'*64)
    runtime.latch = ManusForwardingAuthority({}, clock.now)
    runtime.latch.last_gui = clock.now
    runtime.feedback = {};runtime.recovery_targets = {};runtime.warning_times = {}
    hands = {s: SimpleNamespace(stop=Mock(), read_joint_state=Mock(
        return_value=SimpleNamespace(position=np.zeros(22)))) for s in Side}
    runtime.node = SimpleNamespace(_hands=hands, _has_received_command={s:False for s in Side},
        _timeout_stopped={s:False for s in Side}, needs_enable=set(),
        check_manus_connections=Mock(), stop_applied=True, stopped_sides=set(Side),
        validation_send_count=0, get_logger=lambda:Mock(), forward_full=Mock(),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(
            nanoseconds=int(clock.now*1e9), to_msg=lambda:SimpleNamespace(sec=int(clock.now), nanosec=0))))
    runtime.node.stop_validation = runtime.latch.trip
    runtime.publishers = {s:Mock() for s in ('left','right')}
    runtime.opening = SharpaOpenController(runtime)
    runtime.latch.trip('Open hand failed: left Open hand did not settle before timeout')
    runtime.opening.results['left'] = '张开失败'
    def feed(side, *, age=0, positions=None, names=JOINT_NAMES):
        ns = int((clock.now-age)*1e9)
        message = SimpleNamespace(name=list(names), position=[0.]*22 if positions is None else positions,
            header=SimpleNamespace(stamp=SimpleNamespace(sec=ns//10**9, nanosec=ns%10**9)))
        runtime.manus(side, message)
    return SimpleNamespace(r=runtime, clock=clock, hands=hands, feed=feed)


def test_gloves_can_connect_after_start_but_fault_needs_explicit_recovery(recovery):
    t = recovery;r = t.r
    t.feed('left');t.feed('right')
    assert r.latch.phase == 'FAULT'
    r.node.forward_full.assert_not_called()
    status = r.dispatch({'command':'reconnect_hands'})
    assert status['phase'] == 'READY' and not status['reason']
    assert all(d['state']=='DISABLED' and not d['permit'] for d in status['hands'].values())
    assert not r.node.stop_applied and not r.node.stopped_sides
    assert r.node.needs_enable == {'left','right'}
    assert not r.opening.results
    for hand in t.hands.values():hand.stop.assert_called_once()
    t.feed('left');t.feed('right')
    r.node.forward_full.assert_not_called()
    r.request('right', True);t.feed('right')
    r.node.forward_full.assert_called_once()
    assert not r.latch.requested['left']


@pytest.mark.parametrize('failure', ['missing_left','missing_right','stale','invalid','old_stamp',
                                    'future_stamp','gui','graph','feedback','stop'])
def test_unhealthy_recovery_keeps_fault_and_never_sends(recovery, failure):
    t = recovery;r = t.r
    t.feed('left');t.feed('right')
    if failure.startswith('missing_'):r.recovery_targets.pop(failure.removeprefix('missing_'))
    elif failure == 'stale':t.clock.now += .201;r.latch.last_gui = t.clock.now
    elif failure == 'invalid':t.feed('right', positions=[float('nan')]*22)
    elif failure == 'old_stamp':t.feed('right', age=.201)
    elif failure == 'future_stamp':t.feed('right', age=-.1)
    elif failure == 'gui':r.latch.last_gui -= .201
    elif failure == 'graph':r.node.check_manus_connections.side_effect = RuntimeError('Duplicate owner')
    elif failure == 'feedback':next(iter(t.hands.values())).read_joint_state.return_value.position = [3.]*22
    elif failure == 'stop':next(iter(t.hands.values())).stop.side_effect = RuntimeError('stop failed')
    reason = r.latch.reason
    with pytest.raises((ValueError, RuntimeError)):r.dispatch({'command':'reconnect_hands'})
    assert r.latch.phase == 'FAULT' and r.latch.reason == reason
    assert not any(r.latch.requested.values())
    assert r.node.stop_applied and r.node.stopped_sides
    r.node.forward_full.assert_not_called()


@pytest.mark.parametrize('busy', ['following','opening','ready'])
def test_recovery_cannot_interrupt_working_hands(recovery, busy):
    r = recovery.r
    recovery.feed('left');recovery.feed('right')
    if busy == 'following':r.latch.requested['right'] = True
    elif busy == 'opening':r.opening.operations['left'] = object()
    else:r.latch.phase = 'READY'
    with pytest.raises(ValueError):r.reconnect_hands()
    for hand in recovery.hands.values():hand.stop.assert_not_called()


def test_recovery_checks_graph_again_after_stopping(recovery):
    r = recovery.r
    recovery.feed('left');recovery.feed('right')
    r.node.check_manus_connections.side_effect = [None, RuntimeError('Disconnected during recovery')]
    with pytest.raises(RuntimeError):r.reconnect_hands()
    assert r.latch.phase == 'FAULT' and r.node.stop_applied
    r.node.forward_full.assert_not_called()
