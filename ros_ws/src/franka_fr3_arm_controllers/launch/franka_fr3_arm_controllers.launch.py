#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Opens the specified YAML file and loads its contents into a Python dictionary.


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


WORKCELL_SIDES = {"LEFT", "RIGHT"}
WORKCELL_FIELDS = {
    "arm_id",
    "arm_prefix",
    "controller_cpus",
    "fake_sensor_commands",
    "namespace",
    "robot_ip",
    "urdf_file",
    "use_fake_hardware",
}


def validate_workcell(configs, path):
    if not isinstance(configs, dict):
        raise ValueError(f"{path}: expected a mapping")
    sides = set(configs)
    if sides != WORKCELL_SIDES:
        missing = sorted(WORKCELL_SIDES - sides)
        unknown = sorted(sides - WORKCELL_SIDES)
        details = []
        if missing:
            details.append(f"missing arms: {', '.join(missing)}")
        if unknown:
            details.append(f"unknown arms: {', '.join(unknown)}")
        raise ValueError(f"{path}: {'; '.join(details)}")
    for side in sorted(WORKCELL_SIDES):
        config = configs[side]
        if not isinstance(config, dict):
            raise ValueError(f"{path}: {side} must be a mapping")
        fields = set(config)
        missing = sorted(WORKCELL_FIELDS - fields)
        unknown = sorted(fields - WORKCELL_FIELDS)
        if missing or unknown:
            details = []
            if missing:
                details.append(f"missing keys: {', '.join(missing)}")
            if unknown:
                details.append(f"unknown keys: {', '.join(unknown)}")
            raise ValueError(f"{path}: {side}: {'; '.join(details)}")
        for field in WORKCELL_FIELDS:
            if not isinstance(config[field], str) or not config[field].strip():
                raise ValueError(f"{path}: {side}.{field} must be a non-empty string")
        for field in ("use_fake_hardware", "fake_sensor_commands"):
            if config[field] not in {"true", "false"}:
                raise ValueError(f"{path}: {side}.{field} must be 'true' or 'false'")
    return configs


def joint_impedance_spawner(config):
    """Create the command-controller spawner for one already configured arm."""
    return Node(
        package="controller_manager",
        executable="spawner",
        namespace=config["namespace"],
        arguments=["joint_impedance_controller", "--controller-manager-timeout", "30"],
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("franka_fr3_arm_controllers"),
                    "config",
                    "controllers.yaml",
                ]
            )
        ],
        output="screen",
    )


def continue_after_success(event, _context, next_action, completed_step, next_step):
    """Start the next control step only when its prerequisite exited cleanly."""
    if event.returncode == 0:
        return [
            LogInfo(msg=f"{completed_step} completed; starting {next_step}."),
            next_action,
        ]
    reason = (
        f"{completed_step} exited with code {event.returncode}; "
        f"refusing to start {next_step}."
    )
    return [LogInfo(msg=reason), Shutdown(reason=reason)]


def shutdown_if_failed(event, _context, completed_step):
    """End bringup if the final command controller could not activate."""
    if event.returncode == 0:
        return [LogInfo(msg=f"{completed_step} completed.")]
    reason = f"{completed_step} exited with code {event.returncode}."
    return [LogInfo(msg=reason), Shutdown(reason=reason)]


def controller_activation_sequence(configs, collision_setter=None):
    """Build an ordered, fail-closed activation chain for the two arms.

    Collision settings are a non-realtime FCI operation.  The effort interface
    starts libfranka ActiveControl, so both collision services must complete
    before either joint-impedance controller can claim its effort interfaces.
    """
    left_spawner = joint_impedance_spawner(configs["LEFT"])
    right_spawner = joint_impedance_spawner(configs["RIGHT"])

    actions = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=left_spawner,
                on_exit=lambda event, context: continue_after_success(
                    event,
                    context,
                    right_spawner,
                    "Left joint-impedance controller activation",
                    "right joint-impedance controller",
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=right_spawner,
                on_exit=lambda event, context: shutdown_if_failed(
                    event,
                    context,
                    "Right joint-impedance controller activation",
                ),
            )
        ),
    ]
    if collision_setter is None:
        actions.append(left_spawner)
        return actions

    actions.insert(
        0,
        RegisterEventHandler(
            OnProcessExit(
                target_action=collision_setter,
                on_exit=lambda event, context: continue_after_success(
                    event,
                    context,
                    left_spawner,
                    "Collision threshold configuration",
                    "left joint-impedance controller",
                ),
            )
        ),
    )
    return actions


def generate_robot_nodes(context):
    config_file_name = LaunchConfiguration("robot_config_file").perform(context)
    if os.path.isabs(config_file_name):
        config_file = config_file_name
    else:
        package_config_dir = FindPackageShare(
            "franka_fr3_arm_controllers"
        ).perform(context)
        config_file = os.path.join(package_config_dir, "config", config_file_name)
    configs = validate_workcell(load_yaml(config_file), config_file)
    fake_modes = {config["use_fake_hardware"] for config in configs.values()}
    if len(fake_modes) != 1:
        raise ValueError(
            f"{config_file}: mixed real and fake arms are not supported by "
            "the shared collision-threshold bringup"
        )
    nodes = []
    for side in ("LEFT", "RIGHT"):
        config = configs[side]
        namespace = config["namespace"]
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("franka_fr3_arm_controllers"),
                            "launch",
                            "franka.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "arm_id": str(config["arm_id"]),
                    "arm_prefix": str(config["arm_prefix"]),
                    "controller_cpus": str(config["controller_cpus"]),
                    "namespace": str(namespace),
                    "urdf_file": str(config["urdf_file"]),
                    "robot_ip": str(config["robot_ip"]),
                    "use_fake_hardware": str(config["use_fake_hardware"]),
                    "fake_sensor_commands": str(config["fake_sensor_commands"]),
                }.items(),
            )
        )

    # The fake profile has no FCI collision service.  Keep its controller
    # bringup available while preserving the same serial activation behavior.
    if fake_modes == {"true"}:
        nodes.extend(controller_activation_sequence(configs))
        return nodes

    collision_setter = Node(
        package="franka_fr3_arm_controllers",
        executable="set_bi_collision_behavior.py",
        name="collision_behavior_setter",
        output="screen",
    )
    # Register the complete chain before starting the one-shot process: a
    # fast failure must still prevent command-controller activation.
    nodes.extend(controller_activation_sequence(configs, collision_setter))
    nodes.append(collision_setter)
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_config_file",
                description="Absolute path or package-relative robot configuration file.",
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
