import os
import runpy
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


os.environ.setdefault("ROS_LOG_DIR", "/tmp/lychee_test_ros_logs")
LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "moveit.launch.py"


def _namespace():
    return runpy.run_path(str(LAUNCH_FILE))


def test_state_only_bringup_schedules_no_command_controller_spawner():
    actions = _namespace()["_sequenced_controller_spawners"](
        namespace="left",
        command_controllers=["left_arm_controller"],
        activate_command_controllers=False,
    )

    assert len(actions) == 1
    assert isinstance(actions[0], Node)


def test_command_controller_is_scheduled_after_state_spawner():
    actions = _namespace()["_sequenced_controller_spawners"](
        namespace="left",
        command_controllers=["left_arm_controller"],
        activate_command_controllers=True,
    )

    assert len(actions) == 2
    assert isinstance(actions[0], Node)
    assert isinstance(actions[1], RegisterEventHandler)


def test_default_robot_ips_follow_left_then_right_order():
    description = _namespace()["generate_launch_description"]()
    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert perform_substitutions(
        LaunchContext(),
        arguments["robot_ips"].default_value,
    ) == "['172.16.0.3','172.16.0.2']"


def test_target_fr3_description_expands_with_pinned_franka_description():
    description_path = (
        Path(get_package_share_directory("lychee_fr3_description"))
        / "robots"
        / "lychee_dual_fr3"
        / "lychee_dual_fr3.urdf.xacro"
    )

    document = xacro.process_file(
        str(description_path),
        mappings={
            "robot_types": "['fr3','fr3']",
            "robot_ips": "['172.16.0.3','172.16.0.2']",
            "ros2_control": "true",
            "use_fake_hardware": "true",
        },
    ).toxml()

    assert 'name="left_fr3_joint1"' in document
    assert 'name="right_fr3_joint7"' in document


def test_real_per_arm_controller_actions_are_namespaced():
    config = _namespace()["_namespaced_moveit_controller_config"](
        namespace="",
        arm_prefixes=["left", "right"],
        robot_types=["fr3v2", "fr3v2"],
    )

    assert config["controller_names"] == [
        "/left/left_arm_controller",
        "/right/right_arm_controller",
    ]
    assert config["/left/left_arm_controller"]["action_ns"] == (
        "follow_joint_trajectory"
    )
