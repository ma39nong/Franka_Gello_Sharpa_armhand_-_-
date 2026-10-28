# pyright: reportMissingImports=false, reportMissingTypeStubs=false, reportUnknownVariableType=false, reportAttributeAccessIssue=false, reportUnknownParameterType=false, reportMissingParameterType=false, reportAny=false, reportUnknownArgumentType=false, reportOptionalMemberAccess=false, reportUnknownMemberType=false, reportImplicitStringConcatenation=false
import ast
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
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


def _parse_launch_list(value, *, argument_name):
    parsed = ast.literal_eval(value)
    if not isinstance(parsed, list):
        raise ValueError(f"{argument_name} must be a Python-style list string")
    return parsed


def _join_namespace(*parts):
    clean_parts = [str(part).strip('/') for part in parts if str(part).strip('/')]
    return '/'.join(clean_parts)


def _arm_controller_name(arm_prefix):
    return f'{arm_prefix}_arm_controller' if arm_prefix else 'arm_controller'


def _joint_names(arm_prefix, robot_type):
    joint_prefix = f'{arm_prefix}_{robot_type}' if arm_prefix else robot_type
    return [f'{joint_prefix}_joint{joint_index}' for joint_index in range(1, 8)]


def _sequenced_controller_spawners(
    *,
    namespace,
    command_controllers,
    activate_command_controllers,
):
    state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout',
            '60',
            '--service-call-timeout',
            '60',
        ],
        output='screen',
    )
    command_arguments = list(command_controllers)
    if len(command_arguments) > 1:
        command_arguments.append('--activate-as-group')
    command_arguments.extend(
        [
            '--controller-manager-timeout',
            '60',
            '--service-call-timeout',
            '60',
            '--switch-timeout',
            '60',
        ]
    )
    command_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=command_arguments,
        output='screen',
    )

    def on_state_spawner_exit(event, _context):
        if event.returncode == 0:
            return [command_spawner]
        display_namespace = f'/{namespace}' if namespace else '/'
        return [
            LogInfo(
                msg=(
                    'ERROR: joint_state_broadcaster failed in namespace '
                    f'{display_namespace}; '
                    'command controllers will not be started'
                )
            )
        ]

    actions = [state_spawner]
    if activate_command_controllers:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=state_spawner,
                    on_exit=on_state_spawner_exit,
                )
            )
        )
    return actions


def _namespaced_moveit_controller_config(*, namespace, arm_prefixes, robot_types):
    controller_names = []
    config = {}
    for arm_prefix, robot_type in zip(arm_prefixes, robot_types, strict=True):
        controller_name = _arm_controller_name(arm_prefix)
        controller_namespace = _join_namespace(namespace, arm_prefix)
        moveit_controller_name = f'/{controller_namespace}/{controller_name}'
        controller_names.append(moveit_controller_name)
        config[moveit_controller_name] = {
            'action_ns': 'follow_joint_trajectory',
            'type': 'FollowJointTrajectory',
            'default': True,
            'joints': _joint_names(arm_prefix, robot_type),
        }
    config['controller_names'] = controller_names
    return config


def generate_robot_nodes(context):
    robot_types = LaunchConfiguration('robot_types').perform(context)
    robot_ips = LaunchConfiguration('robot_ips').perform(context)
    arm_prefixes = LaunchConfiguration('arm_prefixes').perform(context)
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands').perform(context)
    use_async_hardware = LaunchConfiguration('use_async_hardware').perform(context)
    start_arm_controllers = (
        LaunchConfiguration('start_arm_controllers').perform(context).lower() == 'true'
    )
    separate_controller_managers = (
        LaunchConfiguration('separate_controller_managers').perform(context).lower() == 'true'
    )
    namespace = LaunchConfiguration('namespace').perform(context)
    load_gripper = LaunchConfiguration('load_gripper').perform(context)
    joint_state_rate = int(LaunchConfiguration('joint_state_rate').perform(context))
    thread_priority = LaunchConfiguration('thread_priority').perform(context)
    ros2_controllers_file = LaunchConfiguration('ros2_controllers_file').perform(context)
    moveit_controllers_file = LaunchConfiguration('moveit_controllers_file').perform(context)
    joint_limits_file = LaunchConfiguration('joint_limits_file').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    move_group_log_level = LaunchConfiguration('move_group_log_level').perform(context)
    configure_collision_behavior = (
        LaunchConfiguration('configure_collision_behavior').perform(context).lower() == 'true'
    )
    collision_behavior_lower_torque_thresholds_acceleration = LaunchConfiguration(
        'collision_behavior_lower_torque_thresholds_acceleration'
    ).perform(context)
    collision_behavior_upper_torque_thresholds_acceleration = LaunchConfiguration(
        'collision_behavior_upper_torque_thresholds_acceleration'
    ).perform(context)
    collision_behavior_lower_torque_thresholds_nominal = LaunchConfiguration(
        'collision_behavior_lower_torque_thresholds_nominal'
    ).perform(context)
    collision_behavior_upper_torque_thresholds_nominal = LaunchConfiguration(
        'collision_behavior_upper_torque_thresholds_nominal'
    ).perform(context)
    collision_behavior_lower_force_thresholds_acceleration = LaunchConfiguration(
        'collision_behavior_lower_force_thresholds_acceleration'
    ).perform(context)
    collision_behavior_upper_force_thresholds_acceleration = LaunchConfiguration(
        'collision_behavior_upper_force_thresholds_acceleration'
    ).perform(context)
    collision_behavior_lower_force_thresholds_nominal = LaunchConfiguration(
        'collision_behavior_lower_force_thresholds_nominal'
    ).perform(context)
    collision_behavior_upper_force_thresholds_nominal = LaunchConfiguration(
        'collision_behavior_upper_force_thresholds_nominal'
    ).perform(context)
    robot_types_list = _parse_launch_list(robot_types, argument_name='robot_types')
    robot_ips_list = _parse_launch_list(robot_ips, argument_name='robot_ips')
    arm_prefixes_list = _parse_launch_list(arm_prefixes, argument_name='arm_prefixes')
    use_fake_hardware_enabled = use_fake_hardware.lower() == 'true'
    use_per_arm_controller_managers = (
        separate_controller_managers and not use_fake_hardware_enabled
    )

    if not configure_collision_behavior:
        collision_behavior_lower_torque_thresholds_acceleration = ''
        collision_behavior_upper_torque_thresholds_acceleration = ''
        collision_behavior_lower_torque_thresholds_nominal = ''
        collision_behavior_upper_torque_thresholds_nominal = ''
        collision_behavior_lower_force_thresholds_acceleration = ''
        collision_behavior_upper_force_thresholds_acceleration = ''
        collision_behavior_lower_force_thresholds_nominal = ''
        collision_behavior_upper_force_thresholds_nominal = ''

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
                'robot_types': robot_types,
                'robot_ips': robot_ips,
                'arm_prefixes': arm_prefixes,
                'hand': load_gripper,
                'use_fake_hardware': use_fake_hardware,
                'fake_sensor_commands': fake_sensor_commands,
                'ros2_control': 'true',
                'is_async': use_async_hardware,
                'thread_priority': thread_priority,
                'collision_behavior_lower_torque_thresholds_acceleration': (
                    collision_behavior_lower_torque_thresholds_acceleration
                ),
                'collision_behavior_upper_torque_thresholds_acceleration': (
                    collision_behavior_upper_torque_thresholds_acceleration
                ),
                'collision_behavior_lower_torque_thresholds_nominal': (
                    collision_behavior_lower_torque_thresholds_nominal
                ),
                'collision_behavior_upper_torque_thresholds_nominal': (
                    collision_behavior_upper_torque_thresholds_nominal
                ),
                'collision_behavior_lower_force_thresholds_acceleration': (
                    collision_behavior_lower_force_thresholds_acceleration
                ),
                'collision_behavior_upper_force_thresholds_acceleration': (
                    collision_behavior_upper_force_thresholds_acceleration
                ),
                'collision_behavior_lower_force_thresholds_nominal': (
                    collision_behavior_lower_force_thresholds_nominal
                ),
                'collision_behavior_upper_force_thresholds_nominal': (
                    collision_behavior_upper_force_thresholds_nominal
                ),
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
    joint_limits_config = {
        'robot_description_planning': robot_description_planning
    }

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
    ompl_planning_yaml = load_yaml('lychee_fr3_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)
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

    # Trajectory Execution Functionality
    if use_per_arm_controller_managers and not moveit_controllers_file:
        moveit_simple_controllers_yaml = _namespaced_moveit_controller_config(
            namespace=namespace,
            arm_prefixes=arm_prefixes_list,
            robot_types=robot_types_list,
        )
    elif not moveit_controllers_file:
        moveit_controllers_file = f'lychee_{robot_family}_controllers.yaml'
        moveit_simple_controllers_yaml = _load_config_yaml(moveit_controllers_file)
    else:
        moveit_simple_controllers_yaml = _load_config_yaml(moveit_controllers_file)
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
        '/MoveItSimpleControllerManager',
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
        'publish_transforms_updates': True,
        # 'publish_robot_description': True,
        'publish_robot_description_semantic': True,
    }

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        output='screen',
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
            {
                'capabilities': (
                    'pilz_industrial_motion_planner/MoveGroupSequenceAction'
                )
            },
        ],
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory('lychee_fr3_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        remappings=[
            ('/planning_scene', '/rviz/planning_scene'),
            ('/planning_scene_world', '/rviz/planning_scene_world'),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipelines_config,
            ompl_planning_pipeline_config,
            pilz_planning_pipeline_config,
            kinematics_config,
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description, {'publish_robot_description': True}],
    )

    if not ros2_controllers_file:
        ros2_controllers_file = (
            f'lychee_{robot_family}_velocity_ros_controllers.yaml'
            if use_fake_hardware.lower() == 'true'
            else f'lychee_{robot_family}_ros_controllers.yaml'
        )

    if os.path.isabs(ros2_controllers_file):
        ros2_controllers_path = ros2_controllers_file
    else:
        ros2_controllers_path = os.path.join(
            get_package_share_directory('lychee_fr3_moveit_config'),
            'config',
            ros2_controllers_file,
        )

    controller_nodes = []
    joint_state_sources = []
    if use_per_arm_controller_managers:
        franka_arm_xacro_file = PathJoinSubstitution(
            [
                FindPackageShare('lychee_fr3_description'),
                'robots',
                'lychee_dual_fr3',
                'lychee_single_fr3_control.urdf.xacro',
            ]
        ).perform(context)

        for arm_prefix, robot_type, robot_ip in zip(
            arm_prefixes_list, robot_types_list, robot_ips_list, strict=True
        ):
            arm_namespace = _join_namespace(namespace, arm_prefix)
            controller_name = _arm_controller_name(arm_prefix)
            arm_robot_description = {
                'robot_description': xacro.process_file(
                    franka_arm_xacro_file,
                    mappings={
                        'robot_type': robot_type,
                        'arm_prefix': arm_prefix,
                        'robot_ip': robot_ip,
                        'hand': load_gripper,
                        'use_fake_hardware': use_fake_hardware,
                        'fake_sensor_commands': fake_sensor_commands,
                        'is_async': use_async_hardware,
                        'thread_priority': thread_priority,
                    },
                ).toprettyxml(indent='  ')
            }

            controller_nodes.append(
                Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    namespace=arm_namespace,
                    parameters=[ros2_controllers_path, arm_robot_description],
                    remappings=[('joint_states', 'franka/joint_states')],
                    output={'stdout': 'screen', 'stderr': 'screen'},
                    on_exit=Shutdown(),
                )
            )
            controller_nodes.extend(
                _sequenced_controller_spawners(
                    namespace=arm_namespace,
                    command_controllers=[controller_name],
                    activate_command_controllers=start_arm_controllers,
                )
            )
            joint_state_sources.append(f'/{arm_namespace}/franka/joint_states')
    else:
        controller_nodes.append(
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                namespace=namespace,
                parameters=[robot_description, ros2_controllers_path],
                remappings=[('joint_states', 'franka/joint_states')],
                output={'stdout': 'screen', 'stderr': 'screen'},
                on_exit=Shutdown(),
            )
        )
        controller_nodes.extend(
            _sequenced_controller_spawners(
                namespace=namespace,
                command_controllers=[
                    'left_arm_controller',
                    'right_arm_controller',
                ],
                activate_command_controllers=start_arm_controllers,
            )
        )
        joint_state_sources.append('franka/joint_states')

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[
            {
                'source_list': joint_state_sources,
                'rate': joint_state_rate,
                'use_robot_description': False,
            }
        ],
    )

    nodes = [
        robot_state_publisher,
        run_move_group_node,
        joint_state_publisher,
    ] + controller_nodes

    if use_rviz:
        nodes.insert(0, rviz_node)

    return nodes


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
                'load_gripper',
                default_value='false',
                description='Whether to load grippers.',
            ),
            DeclareLaunchArgument(
                'use_fake_hardware',
                default_value='true',
                description='Use fake hardware.',
            ),
            DeclareLaunchArgument(
                'fake_sensor_commands',
                default_value='true',
                description='Use fake sensor commands.',
            ),
            DeclareLaunchArgument(
                'use_async_hardware',
                default_value='false',
                description=(
                    'Run each ros2_control hardware component in an async thread. '
                    'Enable only on systems configured for realtime scheduling.'
                ),
            ),
            DeclareLaunchArgument(
                'start_arm_controllers',
                default_value='true',
                description=(
                    'Activate trajectory command controllers after joint-state '
                    'broadcasting is ready. Set false for state-only/RViz bringup.'
                ),
            ),
            DeclareLaunchArgument(
                'separate_controller_managers',
                default_value='true',
                description=(
                    'For real hardware, start one ros2_control_node/controller_manager per arm.'
                ),
            ),
            DeclareLaunchArgument(
                'ros2_controllers_file',
                default_value='',
                description=(
                    'ros2_control controller YAML from the config directory, or an absolute path. '
                    'Empty means auto: position+velocity controller for fake hardware, effort controller otherwise.'
                ),
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
                'namespace',
                default_value='',
                description='Namespace for the robot.',
            ),
            DeclareLaunchArgument(
                'joint_state_rate',
                default_value='100',
                description='Joint state publisher rate in Hz.',
            ),
            DeclareLaunchArgument(
                'thread_priority',
                default_value='50',
                description='Thread priority for the hardware interface.',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Launch RViz.',
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
            DeclareLaunchArgument(
                'configure_collision_behavior',
                default_value='false',
                description=(
                    'Apply collision/reflex thresholds to each Franka hardware '
                    'interface during startup.'
                ),
            ),
            DeclareLaunchArgument(
                'collision_behavior_lower_torque_thresholds_acceleration',
                default_value='25.0 25.0 22.0 20.0 19.0 17.0 14.0',
                description='Lower joint torque contact thresholds during acceleration.',
            ),
            DeclareLaunchArgument(
                'collision_behavior_upper_torque_thresholds_acceleration',
                default_value='35.0 35.0 32.0 30.0 29.0 27.0 24.0',
                description='Upper joint torque collision thresholds during acceleration.',
            ),
            DeclareLaunchArgument(
                'collision_behavior_lower_torque_thresholds_nominal',
                default_value='25.0 25.0 22.0 20.0 19.0 17.0 14.0',
                description='Lower joint torque contact thresholds during nominal motion.',
            ),
            DeclareLaunchArgument(
                'collision_behavior_upper_torque_thresholds_nominal',
                default_value='35.0 35.0 32.0 30.0 29.0 27.0 24.0',
                description='Upper joint torque collision thresholds during nominal motion.',
            ),
            DeclareLaunchArgument(
                'collision_behavior_lower_force_thresholds_acceleration',
                default_value='30.0 30.0 30.0 25.0 25.0 25.0',
                description='Lower Cartesian force contact thresholds during acceleration.',
            ),
            DeclareLaunchArgument(
                'collision_behavior_upper_force_thresholds_acceleration',
                default_value='40.0 40.0 40.0 35.0 35.0 35.0',
                description='Upper Cartesian force collision thresholds during acceleration.',
            ),
            DeclareLaunchArgument(
                'collision_behavior_lower_force_thresholds_nominal',
                default_value='30.0 30.0 30.0 25.0 25.0 25.0',
                description='Lower Cartesian force contact thresholds during nominal motion.',
            ),
            DeclareLaunchArgument(
                'collision_behavior_upper_force_thresholds_nominal',
                default_value='40.0 40.0 40.0 35.0 35.0 35.0',
                description='Upper Cartesian force collision thresholds during nominal motion.',
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
