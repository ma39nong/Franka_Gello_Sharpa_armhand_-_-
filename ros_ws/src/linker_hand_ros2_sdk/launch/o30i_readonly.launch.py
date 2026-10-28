"""Open the right O30i transport for raw identity and feedback only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("transport", default_value="libcanbus"),
        DeclareLaunchArgument("can", default_value="can1"),
        DeclareLaunchArgument("canfd_device", default_value="0"),
        DeclareLaunchArgument("canfd_channel", default_value="0"),
        DeclareLaunchArgument("libcanbus_path", default_value=""),
    ]
    driver = Node(
        package="linker_hand_ros2_sdk",
        executable="linker_hand_o30i",
        name="linker_hand_right_o30i_readonly",
        output="screen",
        parameters=[
            {
                "hand_type": "right",
                "transport": LaunchConfiguration("transport"),
                "can": LaunchConfiguration("can"),
                "canfd_device": LaunchConfiguration("canfd_device"),
                "canfd_channel": LaunchConfiguration("canfd_channel"),
                "libcanbus_path": LaunchConfiguration("libcanbus_path"),
                "output_enabled": False,
                "calibration_verified": False,
            }
        ],
    )
    return LaunchDescription([*arguments, driver])
