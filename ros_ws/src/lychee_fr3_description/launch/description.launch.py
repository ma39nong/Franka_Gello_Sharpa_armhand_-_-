from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    share_dir = Path(get_package_share_directory("lychee_fr3_description"))
    xacro_file = share_dir / "robots" / "lychee_dual_fr3" / "lychee_dual_fr3.urdf.xacro"
    mappings = {
        "hand": LaunchConfiguration("hand").perform(context),
        "ros2_control": LaunchConfiguration("ros2_control").perform(context),
        "use_fake_hardware": LaunchConfiguration("use_fake_hardware").perform(context),
    }
    robot_description = xacro.process_file(str(xacro_file), mappings=mappings).toprettyxml(
        indent="  "
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="lychee_fr3_robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("hand", default_value="false"),
            DeclareLaunchArgument("ros2_control", default_value="false"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
