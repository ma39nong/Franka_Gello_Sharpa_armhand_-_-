"""MANUS normal-mode behavior through the original mock SDK sender."""
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

from adapters.litchibot.test_full_teleop import config_file, isolated_guard_paths
from adapters.litchibot.retarget import JOINT_NAMES


@pytest.mark.parametrize('mode', ['fake', 'normal'])
def test_project_manus_launch_uses_standalone_normal_parameters(tmp_path, mode):
    import importlib.util
    from pathlib import Path
    from adapters.litchibot.test_terminal2 import context_for, ROOT
    from launch.actions import ExecuteProcess
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    from launch.utilities import perform_substitutions
    path, config = config_file(tmp_path, 'manus', mode)
    spec = importlib.util.spec_from_file_location('project_manus_launch', ROOT/'adapters/manus/teleop.launch.py')
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = context_for(description, {'hand_source':'manus','full_teleop':'true',
        'full_config':str(path),'validation_driver_script':str(ROOT/'adapters/litchibot/validation_driver.py')})
    module.validate_source(context)
    selected = [e for e in description.entities if isinstance(e,ExecuteProcess) and (e.condition is None or e.condition.evaluate(context))]
    nodes = [e for e in selected if isinstance(e,Node)]
    assert not any(e.node_package=='sharpa_driver' for e in nodes)
    manus = next(e for e in nodes if e.node_package=='manus_driver')
    retarget = next(e for e in nodes if e.node_package=='manus_sharpa_teleop')
    assert evaluate_parameters(context,manus._Node__parameters)[0]['reliable_keypoints'] is False
    rt = evaluate_parameters(context,retarget._Node__parameters)[0]
    assert rt['max_output_age_s']==0 and rt['max_input_age_s']==.25
    guard = next(e for e in selected if not isinstance(e,Node) and 'validation_driver.py' in ' '.join(perform_substitutions(context,a) for a in e.cmd))
    command = ' '.join(perform_substitutions(context,a) for a in guard.cmd)
    assert 'stop_on_command_timeout_s:=0.0' in command


@pytest.mark.parametrize('mode', ['fake', 'normal'])
@pytest.mark.parametrize('scenario', ['idle', 'jump', 'invalid', 'gap', 'heartbeat', 'disconnect', 'sender', 'latest',
                                     'open_left', 'open_right', 'open_both',
                                     'open_offset_left', 'open_offset_right', 'open_offset_both',
                                     'reconnect', 'reconnect_duplicate', 'reconnect_device', 'reconnect_source'])
def test_normal_manus_original_sender(tmp_path, monkeypatch, isolated_guard_paths, mode, scenario):
    rclpy = pytest.importorskip('rclpy')
    from sensor_msgs.msg import JointState
    import sharpa_driver.node as original
    from litchi_hardware.hardware.sharpa.mock import MockSharpaHand
    from litchi_hardware.hardware.sharpa.sdk_driver import SharpaSdkHand
    from adapters.litchibot import validation_driver
    path, config = config_file(tmp_path, 'manus', mode)
    forbidden = Mock(side_effect=AssertionError('No physical SDK in tests'))
    monkeypatch.setattr(SharpaSdkHand, '_load_sdk', forbidden)
    monkeypatch.setattr(validation_driver, 'ensure_clear_graph', Mock())
    hands = []
    def create(configuration, **kwargs):
        assert kwargs['use_fake_hardware'] is (mode == 'fake')
        hand = MockSharpaHand(configuration)
        hand.send_action = Mock(wraps=hand.send_action)
        if mode == 'normal':
            hand._manager = SimpleNamespace(get_all_devices=lambda: [SimpleNamespace(sn=configuration.serial, hand_side=configuration.side)])
            hand._ensure_enabled = Mock(side_effect=lambda: setattr(hand, '_stopped', False))
        hands.append(hand)
        return hand
    monkeypatch.setattr(original, 'create_hand', create)
    def exercise(node):
        runtime = node.full
        assert runtime.standalone_manus and node._command_timeout_s == 0
        runtime.dispatch({'command': 'heartbeat', 'monotonic_ns': time.monotonic_ns()})
        def feed(value=.2, side='left', names=JOINT_NAMES, age=0):
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.stamp.sec -= age
            msg.name = list(names)
            msg.position = [0.] * 22
            msg.position[JOINT_NAMES.index('thumb_IP')] = value
            runtime.manus(side, msg)
        feed()
        assert node.validation_send_count == 0
        if scenario.startswith('reconnect'):
            node.stop_validation('Open hand failed: left Open hand did not settle before timeout')
            feed(side='left');feed(side='right')
            assert node.latch.phase == 'FAULT'
            monkeypatch.setattr(node, 'get_node_names_and_namespaces', lambda:[('sharpa_driver','/')])
            def publishers(topic):
                if topic.endswith('/target'):
                    return [] if scenario=='reconnect_source' else [SimpleNamespace(node_name='manus_sharpa_retarget')]
                return [SimpleNamespace(node_name='sharpa_driver')]
            monkeypatch.setattr(node, 'get_publishers_info_by_topic', publishers)
            if scenario=='reconnect_duplicate':
                monkeypatch.setattr(node, 'get_node_names_and_namespaces',
                                    lambda:[('sharpa_driver','/'),('sharpa_driver','/other')])
            if scenario=='reconnect_device':hands[0].disconnect()
            if scenario!='reconnect':
                with pytest.raises(RuntimeError):runtime.dispatch({'command':'reconnect_hands'})
                assert node.latch.phase == 'FAULT' and node.stop_applied
                assert node.validation_send_count == 0
                return
            result=runtime.dispatch({'command':'reconnect_hands'})
            assert result['phase']=='READY' and not result['reason']
            assert not node.stop_applied and not node.stopped_sides
            assert node.needs_enable == {'left','right'}
            assert all(d['state']=='DISABLED' and not d['permit'] for d in result['hands'].values())
            feed(side='left');feed(side='right')
            assert node.validation_send_count == 0
            runtime.request('left',True);feed(side='left')
            assert node.validation_send_count == 1
            assert not node.latch.requested['right']
            if mode=='normal':hands[0]._ensure_enabled.assert_called_once()
            return
        if scenario.startswith('open_'):
            has_offset = scenario.startswith('open_offset_')
            selection = scenario.removeprefix('open_offset_' if has_offset else 'open_')
            selected = {'left', 'right'} if selection == 'both' else {selection}
            for hand in hands:
                hand._position = tuple(np.zeros(22))
                # Start from a bent joint and model slow physical enable polling.
                pose = np.zeros(22);pose[JOINT_NAMES.index('thumb_IP')] = .4
                hand._position = tuple(pose)
                if has_offset:
                    original_send = hand.send_action
                    def send(action, h=hand, original=original_send):
                        result = original(action)
                        measured = list(h._position)
                        measured[JOINT_NAMES.index('thumb_IP')] += .218
                        h._position = tuple(measured)
                        return result
                    hand.send_action = Mock(side_effect=send)
                if mode == 'normal':
                    def enable(h=hand):
                        time.sleep(.25)
                        h._stopped = False
                    hand._ensure_enabled.side_effect = enable
            runtime.dispatch({'command':'open_hand', 'arguments':{'side':selection}})
            runtime.opening.tick()
            assert node.latch.phase != 'FAULT', node.latch.reason
            assert node.validation_send_count == 0
            assert set(runtime.opening.operations) == selected
            end = time.monotonic() + 4
            while runtime.opening.operations and time.monotonic() < end:
                runtime.dispatch({'command':'heartbeat', 'monotonic_ns':time.monotonic_ns()})
                runtime.tick()
                time.sleep(.02)
            assert not runtime.opening.operations
            assert node.latch.phase != 'FAULT', node.latch.reason
            assert not any(node.latch.requested.values())
            for hand in hands:
                if hand.config.side.value in selected:
                    assert hand.send_action.call_count > 20
                    expected = np.zeros(22)
                    if has_offset:expected[JOINT_NAMES.index('thumb_IP')] = .218
                    np.testing.assert_array_equal(hand.read_joint_state().position, expected)
                    if mode == 'normal':hand._ensure_enabled.assert_called_once()
                else:
                    hand.send_action.assert_not_called()
            # Completion must leave Start hand usable, without reconnecting or
            # automatically restoring following. Its next frame re-enables SDK.
            for side in selected:
                before = node.validation_send_count
                runtime.request(side, True)
                feed(side=side)
                assert node.validation_send_count == before + 1
                assert node.latch.requested[side]
                assert node.latch.side_state(side, time.monotonic()) == 'ACTIVE'
            assert node.latch.phase != 'FAULT'
            return
        if scenario == 'idle':
            with patch('adapters.manus.normal_hand_runtime.time.monotonic', return_value=time.monotonic()+600):
                runtime.tick()
            assert node.latch.phase == 'READY'
            assert not any(node.latch.requested.values())
            runtime.request('right', True)
            feed(.8, 'right')
            assert node.validation_send_count == 1
            return
        runtime.request('left', True)
        feed()
        assert node.validation_send_count == 1
        if scenario == 'jump':
            for q in (1.5, 0., 1.5, 0.): feed(q)
            assert node.validation_send_count == 5
        elif scenario == 'invalid':
            for q in (float('nan'), float('inf'), 2.): feed(q)
            feed(names=tuple(reversed(JOINT_NAMES)))
            assert node.validation_send_count == 1
            feed(.8)
            assert node.validation_send_count == 2
        elif scenario in ('gap', 'heartbeat'):
            node._last_command_at[next(s for s in node._hands if s.value=='left')] -= 30
            node._publish_state()
            runtime.latch.last_gui = time.monotonic()-30
            runtime.dispatch({'command':'heartbeat','monotonic_ns':time.monotonic_ns()-30_000_000_000})
            runtime.tick()
            assert runtime.latch.requested['left']
            feed(.8, age=1)  # Standalone has no additional output-age latch.
            assert node.validation_send_count == 2
        elif scenario == 'disconnect':
            runtime.disconnected()
            feed(.8)
            assert node.validation_send_count == 1 and not any(node.latch.requested.values())
            runtime.dispatch({'command':'heartbeat','monotonic_ns':time.monotonic_ns()})
            feed(.8)
            assert node.validation_send_count == 1
            runtime.request('left', True); feed(.8)
            assert node.validation_send_count == 2
        elif scenario == 'sender':
            hands[0].send_action = Mock(side_effect=RuntimeError('Native send failed'))
            feed(.8)
            assert node.latch.phase == 'FAULT' and not any(node.latch.requested.values())
            return
        elif scenario == 'latest':
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            source = Node('manus_sharpa_retarget')
            executor = SingleThreadedExecutor()
            executor.add_node(node);executor.add_node(source)
            pub = source.create_publisher(JointState, '/teleop/sharpa/left/target', 100)
            try:
                end = time.monotonic()+2
                while not pub.get_subscription_count() and time.monotonic()<end:
                    executor.spin_once(timeout_sec=.005)
                assert pub.get_subscription_count() == 1
                for index in range(100):
                    msg = JointState();msg.header.stamp = node.get_clock().now().to_msg()
                    msg.name = list(JOINT_NAMES);msg.position = [0.]*22
                    msg.position[JOINT_NAMES.index('thumb_IP')] = .8 if index==99 else .1
                    pub.publish(msg)
                time.sleep(.03)
                before = node.validation_send_count
                end = time.monotonic()+.1
                while time.monotonic()<end:executor.spin_once(timeout_sec=.001)
                assert node.validation_send_count == before+1
                assert hands[0].read_joint_state().position[JOINT_NAMES.index('thumb_IP')] == .8
            finally:
                executor.remove_node(node);executor.remove_node(source)
                source.destroy_node();executor.shutdown()
        assert node.latch.phase != 'FAULT' and node.latch.requested['left']
        np.testing.assert_array_equal(hands[1].read_joint_state().position, np.zeros(22))
    monkeypatch.setattr(rclpy, 'spin', exercise)
    validation_driver.main(['--full-config',str(path),'--ros-args',
        '-p',f'use_fake_hardware:={str(mode=="fake").lower()}',
        '-p','enable_left:=true','-p','enable_right:=true',
        '-p','left_serial:='+('fake-left' if mode=='fake' else 'testL'),
        '-p','right_serial:='+('fake-right' if mode=='fake' else 'testR'),
        '-p','stop_on_command_timeout_s:=0.0'])
    forbidden.assert_not_called()
