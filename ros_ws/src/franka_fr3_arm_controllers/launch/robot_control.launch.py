import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    robot_config = LaunchConfiguration("robot_config")
    ld.add_action(DeclareLaunchArgument(
        "robot_config",
        description="Absolute dual-FR3 workcell configuration path.",
    ))

    # Core hardware controllers: per-arm ros2_control at 1 kHz.
    franka_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('franka_fr3_arm_controllers'),
                'launch',
                'franka_fr3_arm_controllers.launch.py'
            )
        ),
        launch_arguments={"robot_config_file": robot_config}.items(),
    )

    ld.add_action(franka_controllers_launch)
    ld.add_action(Node(
        package="franka_fr3_arm_controllers",
        executable="reset_to_initial_pose.py",
        name="reset_to_initial_pose",
        output="screen",
    ))
    # The included dual-arm launch applies collision thresholds and only then
    # activates the effort controllers.  Keep this reset server independent:
    # it registers services but never begins a reset at startup.
    return ld
