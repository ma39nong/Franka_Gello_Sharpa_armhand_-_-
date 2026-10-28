import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def _load_config_yaml(config_file):
    if os.path.isabs(config_file):
        absolute_file_path = config_file
    else:
        absolute_file_path = os.path.join(
            get_package_share_directory('lychee_fr3_moveit_config'),
            'config',
            config_file,
        )
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_robot_nodes(context):
    robot_types = LaunchConfiguration('robot_types').perform(context)
    robot_ips = LaunchConfiguration('robot_ips').perform(context)
    arm_prefixes = LaunchConfiguration('arm_prefixes').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    load_gripper = LaunchConfiguration('load_gripper').perform(context)
    move_group_log_level = LaunchConfiguration('move_group_log_level').perform(context)
    moveit_controllers_file = LaunchConfiguration('moveit_controllers_file').perform(context)
    joint_limits_file = LaunchConfiguration('joint_limits_file').perform(context)

    lychee_urdf_xacro_file = PathJoinSubstitution(
        [
            FindPackageShare('lychee_fr3_description'),
            'robots',
            'lychee_dual_fr3',
            'lychee_dual_fr3.urdf.xacro',
        ]
    ).perform(context)

    robot_description = {
        'robot_description': xacro.process_file(
            lychee_urdf_xacro_file,
            mappings={
                'ros2_control': 'false',
                'robot_types': robot_types,
                'robot_ips': robot_ips,
                'arm_prefixes': arm_prefixes,
                'hand': load_gripper,
            },
        ).toprettyxml(indent='  ')
    }

    lychee_semantic_xacro_file = PathJoinSubstitution(
        [
            FindPackageShare('lychee_fr3_description'),
            'robots',
            'lychee_dual_fr3',
            'lychee_dual_fr3.srdf.xacro',
        ]
    ).perform(context)

    robot_description_semantic = {
        'robot_description_semantic': xacro.process_file(
            lychee_semantic_xacro_file,
            mappings={
                'robot_types': robot_types,
                'arm_prefixes': arm_prefixes,
                'hand': load_gripper,
            },
        ).toprettyxml(indent='  ')
    }

    kinematics_config = {
        'robot_description_kinematics': load_yaml(
            'lychee_fr3_moveit_config', 'config/kinematics.yaml'
        )
    }

    robot_family = 'fr3v2' if 'fr3v2' in robot_types else 'fr3'
    if not joint_limits_file:
        joint_limits_file = f'lychee_{robot_family}_joint_limits.yaml'
    robot_description_planning = _load_config_yaml(joint_limits_file)
    robot_description_planning.update(
        load_yaml('lychee_fr3_moveit_config', 'config/pilz_cartesian_limits.yaml')
    )
    joint_limits_config = {'robot_description_planning': robot_description_planning}

    ompl_planning_pipeline_config = {
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ' '.join(
                [
                    'default_planner_request_adapters/AddTimeOptimalParameterization',
                    'default_planner_request_adapters/FixWorkspaceBounds',
                    'default_planner_request_adapters/FixStartStateBounds',
                    'default_planner_request_adapters/FixStartStateCollision',
                    'default_planner_request_adapters/FixStartStatePathConstraints',
                ]
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['ompl'].update(
        load_yaml('lychee_fr3_moveit_config', 'config/ompl_planning.yaml')
    )
    pilz_planning_pipeline_config = {
        'pilz_industrial_motion_planner': {
            'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner',
            'request_adapters': '',
            'start_state_max_bounds_error': 0.1,
        }
    }
    planning_pipelines_config = {
        'planning_pipelines': ['ompl', 'pilz_industrial_motion_planner'],
        'default_planning_pipeline': 'ompl',
    }

    if not moveit_controllers_file:
        moveit_controllers_file = f'lychee_{robot_family}_controllers.yaml'
    moveit_controllers = {
        'moveit_simple_controller_manager': _load_config_yaml(moveit_controllers_file),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        # Do not republish the robot TF: franka-control already owns the
        # per-arm trees, and a second publisher splits lychee_root from link8.
        'publish_transforms_updates': False,
    }

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        arguments=['--ros-args', '--log-level', move_group_log_level],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            planning_pipelines_config,
            ompl_planning_pipeline_config,
            pilz_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # This launch file is also used beside the UDP teleop stack as an IK-only
    # service. It observes the two existing hardware state topics and publishes
    # the combined model state/TF required by /compute_ik; it never starts a
    # controller manager or sends a hardware command.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='moveit_ik_joint_state_publisher',
        parameters=[
            robot_description,
            {
                'source_list': [
                    f'/{prefix}/franka/joint_states'
                    for prefix in yaml.safe_load(arm_prefixes)
                ],
                'rate': 100.0,
                'ignore_timestamp': True,
            },
        ],
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='moveit_ik_robot_state_publisher',
        parameters=[robot_description],
    )

    return [run_move_group_node, joint_state_publisher, robot_state_publisher]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_types',
                default_value="['fr3v2','fr3v2']",
                description='Types of the robot arms as a string list.',
            ),
            DeclareLaunchArgument(
                'robot_ips',
                default_value="['172.16.0.3','172.16.0.2']",
                description='IP addresses of the robot arms as a string list.',
            ),
            DeclareLaunchArgument(
                'arm_prefixes',
                default_value="['left','right']",
                description='Arm prefixes as a string list.',
            ),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Namespace for the robot.',
            ),
            DeclareLaunchArgument(
                'load_gripper',
                default_value='false',
                description='Whether to load grippers.',
            ),
            DeclareLaunchArgument(
                'moveit_controllers_file',
                default_value='',
                description=(
                    'MoveIt simple-controller YAML from the config directory, or an absolute path. '
                    'Empty auto-selects fr3/fr3v2 from robot_types.'
                ),
            ),
            DeclareLaunchArgument(
                'joint_limits_file',
                default_value='',
                description=(
                    'MoveIt joint-limit YAML from the config directory, or an absolute path. '
                    'Empty auto-selects fr3/fr3v2 from robot_types.'
                ),
            ),
            DeclareLaunchArgument(
                'move_group_log_level',
                default_value='info',
                description=(
                    'ROS log level passed to move_group. Use '
                    'moveit_ros.planning_scene_monitor.planning_scene_monitor:=error '
                    'to suppress disconnected camera-frame planning-scene warnings.'
                ),
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
