from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bind_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("bind_port", default_value="5602"),
            DeclareLaunchArgument("max_staleness_ms", default_value="150.0"),
            Node(
                package="teleop_hand_telemetry",
                executable="hand_telemetry_receiver",
                name="teleop_hand_telemetry_receiver",
                output="screen",
                parameters=[
                    {
                        "bind_host": LaunchConfiguration("bind_host"),
                        "bind_port": LaunchConfiguration("bind_port"),
                        "max_staleness_ms": LaunchConfiguration(
                            "max_staleness_ms"
                        ),
                    }
                ],
            ),
        ]
    )

