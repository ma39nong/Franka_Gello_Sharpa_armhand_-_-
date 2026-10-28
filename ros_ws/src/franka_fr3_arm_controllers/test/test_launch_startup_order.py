import os
import runpy
from pathlib import Path
from types import SimpleNamespace

from launch.actions import LogInfo, RegisterEventHandler, Shutdown
from launch_ros.actions import Node


os.environ.setdefault("ROS_LOG_DIR", "/tmp/franka_launch_test_ros_logs")
LAUNCH_FILE = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "franka_fr3_arm_controllers.launch.py"
)
ROBOT_CONTROL_LAUNCH_FILE = (
    Path(__file__).resolve().parents[1] / "launch" / "robot_control.launch.py"
)
FRANKA_LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "franka.launch.py"


def _launch_namespace():
    return runpy.run_path(str(LAUNCH_FILE))


def test_successful_prerequisite_starts_only_its_next_action():
    module = _launch_namespace()
    next_action = object()

    actions = module["continue_after_success"](
        SimpleNamespace(returncode=0),
        None,
        next_action,
        "collision setup",
        "left controller",
    )

    assert isinstance(actions[0], LogInfo)
    assert actions[1] is next_action
    assert not any(isinstance(action, Shutdown) for action in actions)


def test_failed_prerequisite_never_starts_its_next_action():
    module = _launch_namespace()
    next_action = object()

    actions = module["continue_after_success"](
        SimpleNamespace(returncode=1),
        None,
        next_action,
        "collision setup",
        "left controller",
    )

    assert next_action not in actions
    assert any(isinstance(action, Shutdown) for action in actions)


def test_failed_final_controller_stops_bringup():
    module = _launch_namespace()

    actions = module["shutdown_if_failed"](
        SimpleNamespace(returncode=1), None, "right controller activation"
    )

    assert any(isinstance(action, Shutdown) for action in actions)


def test_successful_final_controller_does_not_stop_bringup():
    module = _launch_namespace()

    actions = module["shutdown_if_failed"](
        SimpleNamespace(returncode=0), None, "right controller activation"
    )

    assert not any(isinstance(action, Shutdown) for action in actions)


def test_real_bringup_registers_barriers_before_starting_collision_setter():
    module = _launch_namespace()
    configs = {
        "LEFT": {"namespace": "left"},
        "RIGHT": {"namespace": "right"},
    }
    setter = Node(
        package="franka_fr3_arm_controllers",
        executable="set_bi_collision_behavior.py",
    )

    actions = module["controller_activation_sequence"](configs, setter)

    assert len(actions) == 3
    assert all(isinstance(action, RegisterEventHandler) for action in actions)
    assert not any(isinstance(action, Node) for action in actions)


def test_fake_bringup_starts_only_the_first_controller_initially():
    module = _launch_namespace()
    configs = {
        "LEFT": {"namespace": "left"},
        "RIGHT": {"namespace": "right"},
    }

    actions = module["controller_activation_sequence"](configs)

    assert len(actions) == 3
    assert all(isinstance(action, RegisterEventHandler) for action in actions[:2])
    assert isinstance(actions[2], Node)


def test_outer_launch_does_not_start_a_second_collision_setter():
    source = ROBOT_CONTROL_LAUNCH_FILE.read_text(encoding="utf-8")

    assert "set_bi_collision_behavior.py" not in source


def test_franka_launch_uses_ordered_shutdown_control_node():
    source = FRANKA_LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'package="franka_fr3_arm_controllers"' in source
    assert 'executable="franka_ros2_control_node"' in source
    assert 'executable="ros2_control_node"' not in source
