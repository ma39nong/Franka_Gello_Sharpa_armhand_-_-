from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file"),
            DeclareLaunchArgument("output_dir"),
            Node(
                package="teleop_data_collector",
                executable="rosbag_data_collector",
                name="teleop_data_collector",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"output_dir": LaunchConfiguration("output_dir")},
                ],
            ),
        ]
    )

