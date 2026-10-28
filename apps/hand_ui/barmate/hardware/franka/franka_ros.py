"""
Franka robot driver via ROS implementing the core FrankaRobotInterface.

This module is intentionally a low-level hardware building block. It talks to
ROS2 topics/actions directly and does not share a runtime assumption with the
non-ROS Franka backends.

Supported ROS2 command surfaces:
    1. ros2_control joint trajectory controller:
       ``trajectory_msgs/JointTrajectory`` topic.
    2. MoveIt controller action:
        ``control_msgs/action/FollowJointTrajectory`` action.
    3. MoveIt execution action:
       ``moveit_msgs/action/ExecuteTrajectory`` action plus optional
       ``moveit_msgs/msg/DisplayTrajectory`` preview topic.
    4. MoveIt Servo topics:
        ``~/delta_twist_cmds``, ``~/delta_joint_cmds``, ``~/pose_target_cmds``.

Common setup::

    robot = FrankaRosRobot(
        joint_state_topic="/joint_states",
        joint_trajectory_topic="/joint_trajectory_controller/joint_trajectory",
        follow_joint_trajectory_action="/joint_trajectory_controller/follow_joint_trajectory",
        servo_delta_twist_topic="/servo_node/delta_twist_cmds",
        servo_delta_joint_topic="/servo_node/delta_joint_cmds",
        servo_pose_target_topic="/servo_node/pose_target_cmds",
    )
    robot.connect()

Reading state::

    joint_state = robot.read_joint_state()       # JointState(position, velocity, effort)
    tcp_pose = robot.read_tcp_pose()             # Pose | None
    franka_state = robot.read_franka_state()     # FrankaRobotState snapshot
    observation = robot.get_observation()        # obs_joint, obs_tcp_pose, q, dq, tau_J, ...

State reads return the latest callback snapshot without spinning. By default the
driver spins its owned ROS node in a background thread after ``connect()``; when
``spin_in_background=False`` or an externally-owned node is supplied, the caller's
executor must keep the node spinning.

Sending commands through the generic robot interface::

    robot.send_action({"action_joint": [0.0] * 7})
    robot.send_action({"action_tcp_pose": [0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]})
    robot.send_action({"action_delta_twist": [0.0, 0.0, 0.01, 0.0, 0.0, 0.0]})
    robot.send_action({"action_delta_joint": [0.001] * 7})

Sending commands through explicit ROS surfaces::

    robot.publish_joint_trajectory([[0.0] * 7])
    robot.follow_joint_trajectory([
        {"positions": [0.0] * 7, "time_from_start_sec": 1.0},
    ])
    robot.command_delta_twist([0.0, 0.0, 0.01, 0.0, 0.0, 0.0])
    robot.command_delta_joint([0.001] * 7)
    robot.command_tcp_pose([0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0])

Call ``disconnect()`` when the ROS node owned by this driver should be destroyed.
"""

from __future__ import annotations

import importlib
import math
import threading
import time
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Callable, ClassVar, SupportsFloat, SupportsIndex, cast

from barmate.core.robot import FrankaRobotInterface
from barmate.core.types import (
    FeatureSpec,
    FrankaRobotState,
    JointState,
    Pose,
    RobotAction,
    RobotObservation,
)


Floatable = str | bytes | SupportsFloat | SupportsIndex


DEFAULT_FRANKA_JOINT_NAMES = (
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
)

DUAL_ARM_DOF = 7
DEFAULT_DUAL_ARM_JOINT_PREFIXES = {
    "left": "left_fr3v2_joint",
    "right": "right_fr3v2_joint",
}
DEFAULT_DUAL_ARM_CONTROLLER_NAMES = {
    "left": "/left_arm_controller",
    "right": "/right_arm_controller",
}


@dataclass(slots=True, frozen=True)
class FrankaRosPlaybackConfig:
    """Naming slice needed to build replay-oriented Franka ROS adapters."""

    left_namespace: str = "left"
    right_namespace: str = "right"
    joint_state_topic_name: str = "/joint_states"
    left_joint_prefix: str = DEFAULT_DUAL_ARM_JOINT_PREFIXES["left"]
    right_joint_prefix: str = DEFAULT_DUAL_ARM_JOINT_PREFIXES["right"]
    controller_name: str = ""
    left_controller_name: str = DEFAULT_DUAL_ARM_CONTROLLER_NAMES["left"]
    right_controller_name: str = DEFAULT_DUAL_ARM_CONTROLLER_NAMES["right"]
    joint_trajectory_topic_name: str = "joint_trajectory"
    follow_action_name: str = "follow_joint_trajectory"
    execute_action_name: str = "/execute_trajectory"
    display_topic_name: str = "/display_planned_path"
    command_surface: str = "follow_joint_trajectory"
    state_timeout_sec: float = 5.0


def namespaced_name(namespace: str, leaf_name: str) -> str:
    if leaf_name.startswith("/"):
        return leaf_name
    stripped = namespace.strip().strip("/")
    if not stripped:
        return f"/{leaf_name.lstrip('/')}"
    return f"/{stripped}/{leaf_name.lstrip('/')}"


def dual_arm_joint_names(arm: str, config: FrankaRosPlaybackConfig) -> tuple[str, ...]:
    prefix = config.left_joint_prefix if arm == "left" else config.right_joint_prefix
    return tuple(f"{prefix}{index}" for index in range(1, DUAL_ARM_DOF + 1))


def combined_dual_arm_joint_names(config: FrankaRosPlaybackConfig) -> tuple[str, ...]:
    return dual_arm_joint_names("left", config) + dual_arm_joint_names("right", config)


def dual_arm_controller_name(arm: str, config: FrankaRosPlaybackConfig) -> str:
    if config.controller_name:
        return config.controller_name
    return config.left_controller_name if arm == "left" else config.right_controller_name


def execute_trajectory_controller_names(
    config: FrankaRosPlaybackConfig,
) -> tuple[str, ...]:
    return tuple(
        controller.strip("/")
        for controller in (
            dual_arm_controller_name("left", config),
            dual_arm_controller_name("right", config),
        )
        if controller.strip("/")
    )


def _as_float_tuple(value: object, *, width: int | None = None) -> tuple[float, ...]:
    if value is None:
        values: tuple[float, ...] = ()
    elif isinstance(value, Sequence) and not isinstance(value, str):
        values = tuple(float(cast(Floatable, item)) for item in value)
    elif isinstance(value, Iterable) and not isinstance(value, (str, bytes, Mapping)):
        values = tuple(float(cast(Floatable, item)) for item in value)
    else:
        values = (float(cast(Floatable, value)),)

    if width is None:
        return values
    if len(values) >= width:
        return values[:width]
    return values + (0.0,) * (width - len(values))


def _duration_seconds(duration_sec: float) -> tuple[int, int]:
    seconds = max(0.0, float(duration_sec))
    whole_seconds = int(seconds)
    nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
    if nanoseconds == 1_000_000_000:
        whole_seconds += 1
        nanoseconds = 0
    return whole_seconds, nanoseconds


def _matrix_to_pose(matrix_like: object) -> Pose | None:
    values = _as_float_tuple(matrix_like)
    if len(values) == 7:
        return Pose(
            position=(values[0], values[1], values[2]),
            orientation_xyzw=(values[3], values[4], values[5], values[6]),
        )
    if len(values) != 16:
        return None

    matrix = [tuple(values[column * 4 + row] for column in range(4)) for row in range(4)]
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2][1] - matrix[1][2]) / scale
        qy = (matrix[0][2] - matrix[2][0]) / scale
        qz = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        qw = (matrix[2][1] - matrix[1][2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0][1] + matrix[1][0]) / scale
        qz = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        qw = (matrix[0][2] - matrix[2][0]) / scale
        qx = (matrix[0][1] + matrix[1][0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        qw = (matrix[1][0] - matrix[0][1]) / scale
        qx = (matrix[0][2] + matrix[2][0]) / scale
        qy = (matrix[1][2] + matrix[2][1]) / scale
        qz = 0.25 * scale

    return Pose(
        position=(matrix[0][3], matrix[1][3], matrix[2][3]),
        orientation_xyzw=(qx, qy, qz, qw),
    )


class FrankaRosRobot(FrankaRobotInterface):
    """Low-level ROS2 Franka driver using ROS topics and actions directly."""

    _rclpy_lifecycle_lock: ClassVar[threading.Lock] = threading.Lock()
    _rclpy_initialized_by_driver: ClassVar[bool] = False
    _active_driver_nodes: ClassVar[int] = 0

    @classmethod
    def from_config(
        cls,
        config: FrankaRosPlaybackConfig,
        *,
        arm: str | None = None,
    ) -> "FrankaRosRobot":
        """Build a replay ROS adapter from explicit dual-arm naming config."""

        if arm is None:
            return cls(
                name="dual",
                joint_names=combined_dual_arm_joint_names(config),
                node_name="barmate_dual_execute_moveit_hand_example",
                joint_state_topic=namespaced_name("", config.joint_state_topic_name),
                execute_trajectory_action=namespaced_name("", config.execute_action_name),
                display_trajectory_topic=namespaced_name("", config.display_topic_name),
                execute_trajectory_controller_names=execute_trajectory_controller_names(
                    config
                ),
                joint_position_command="execute_trajectory",
                state_timeout_sec=config.state_timeout_sec,
            )

        if arm not in {"left", "right"}:
            raise ValueError("arm must be 'left', 'right', or None")

        namespace = config.left_namespace if arm == "left" else config.right_namespace
        controller_name = dual_arm_controller_name(arm, config)
        controller_prefix = f"{controller_name}/{config.joint_trajectory_topic_name}"
        follow_action = f"{controller_name}/{config.follow_action_name}"
        return cls(
            name=arm,
            joint_names=dual_arm_joint_names(arm, config),
            node_name=f"barmate_{arm}_franka_ros_example",
            joint_state_topic=namespaced_name(namespace, config.joint_state_topic_name),
            joint_trajectory_topic=namespaced_name(namespace, controller_prefix),
            follow_joint_trajectory_action=namespaced_name(namespace, follow_action),
            execute_trajectory_action=namespaced_name("", config.execute_action_name),
            display_trajectory_topic=namespaced_name("", config.display_topic_name),
            execute_trajectory_controller_names=(controller_name,),
            joint_position_command=config.command_surface,
            state_timeout_sec=config.state_timeout_sec,
        )

    def __init__(
        self,
        name: str = "franka",
        *,
        joint_names: Sequence[str] = DEFAULT_FRANKA_JOINT_NAMES,
        node_name: str = "barmate_franka_ros",
        joint_state_topic: str = "/joint_states",
        tcp_pose_topic: str | None = None,
        joint_trajectory_topic: str = "/joint_trajectory_controller/joint_trajectory",
        follow_joint_trajectory_action: str = "/joint_trajectory_controller/follow_joint_trajectory",
        execute_trajectory_action: str = "/execute_trajectory",
        display_trajectory_topic: str = "/display_planned_path",
        execute_trajectory_controller_names: Sequence[str] = (),
        servo_delta_twist_topic: str = "/servo_node/delta_twist_cmds",
        servo_delta_joint_topic: str = "/servo_node/delta_joint_cmds",
        servo_pose_target_topic: str = "/servo_node/pose_target_cmds",
        planning_frame: str = "base_link",
        joint_position_command: str = "trajectory_topic",
        default_trajectory_duration_sec: float = 0.1,
        state_timeout_sec: float = 1.0,
        node_factory: Callable[[], Any] | None = None,
        spin_in_background: bool | None = None,
    ) -> None:
        self.name = name
        self.joint_names = tuple(joint_names)
        self.node_name = node_name
        self.joint_state_topic = joint_state_topic
        self.tcp_pose_topic = tcp_pose_topic
        self.joint_trajectory_topic = joint_trajectory_topic
        self.follow_joint_trajectory_action = follow_joint_trajectory_action
        self.execute_trajectory_action = execute_trajectory_action
        self.display_trajectory_topic = display_trajectory_topic
        self.execute_trajectory_controller_names = tuple(execute_trajectory_controller_names)
        self.servo_delta_twist_topic = servo_delta_twist_topic
        self.servo_delta_joint_topic = servo_delta_joint_topic
        self.servo_pose_target_topic = servo_pose_target_topic
        self.planning_frame = planning_frame
        self.joint_position_command = joint_position_command
        self.default_trajectory_duration_sec = float(default_trajectory_duration_sec)
        self.state_timeout_sec = float(state_timeout_sec)
        self._node_factory = node_factory
        self._spin_in_background = spin_in_background

        self._connected = False
        self._owns_node = False
        self._uses_driver_rclpy = False
        self._rclpy: Any | None = None
        self._node: Any | None = None
        self._executor: Any | None = None
        self._shutdown_exception_types: tuple[type[BaseException], ...] = ()
        self._duration_type: Any | None = None
        self._joint_trajectory_type: Any | None = None
        self._joint_trajectory_point_type: Any | None = None
        self._execute_trajectory_type: Any | None = None
        self._display_trajectory_type: Any | None = None
        self._moveit_error_codes_type: Any | None = None
        self._robot_state_type: Any | None = None
        self._robot_trajectory_type: Any | None = None
        self._twist_stamped_type: Any | None = None
        self._pose_stamped_type: Any | None = None
        self._joint_jog_type: Any | None = None
        self._follow_joint_trajectory_type: Any | None = None
        self._action_client_type: Any | None = None

        self._joint_state_subscription: Any | None = None
        self._tcp_pose_subscription: Any | None = None
        self._joint_trajectory_publisher: Any | None = None
        self._display_trajectory_publisher: Any | None = None
        self._delta_twist_publisher: Any | None = None
        self._delta_joint_publisher: Any | None = None
        self._pose_target_publisher: Any | None = None
        self._follow_joint_trajectory_client: Any | None = None
        self._execute_trajectory_client: Any | None = None

        self._latest_joint_state: JointState | None = None
        self._latest_tcp_pose: Pose | None = None
        self._latest_franka_state: FrankaRobotState | None = None
        self._state_lock = threading.Lock()
        self._spin_stop_event = threading.Event()
        self._spin_thread: threading.Thread | None = None

    def connect(self) -> None:
        if self._connected:
            return

        rclpy = importlib.import_module("rclpy")
        action_module = importlib.import_module("rclpy.action")
        executor_module = importlib.import_module("rclpy.executors")
        rclpy_binding = self._optional_import("rclpy._rclpy_pybind11")
        qos_module = importlib.import_module("rclpy.qos")
        builtin_interfaces = importlib.import_module("builtin_interfaces.msg")
        control_actions = importlib.import_module("control_msgs.action")
        control_messages = importlib.import_module("control_msgs.msg")
        geometry_messages = importlib.import_module("geometry_msgs.msg")
        moveit_actions = importlib.import_module("moveit_msgs.action")
        moveit_messages = importlib.import_module("moveit_msgs.msg")
        sensor_messages = importlib.import_module("sensor_msgs.msg")
        trajectory_messages = importlib.import_module("trajectory_msgs.msg")

        Duration = builtin_interfaces.Duration
        ActionClient = action_module.ActionClient
        SingleThreadedExecutor = executor_module.SingleThreadedExecutor
        shutdown_exception_types: list[type[BaseException]] = [
            executor_module.ExternalShutdownException,
            executor_module.ShutdownException,
        ]
        if rclpy_binding is not None and hasattr(rclpy_binding, "RCLError"):
            shutdown_exception_types.append(rclpy_binding.RCLError)
        FollowJointTrajectory = control_actions.FollowJointTrajectory
        JointJog = control_messages.JointJog
        PoseStamped = geometry_messages.PoseStamped
        TwistStamped = geometry_messages.TwistStamped
        ExecuteTrajectory = moveit_actions.ExecuteTrajectory
        DisplayTrajectory = moveit_messages.DisplayTrajectory
        MoveItErrorCodes = moveit_messages.MoveItErrorCodes
        RobotState = moveit_messages.RobotState
        RobotTrajectory = moveit_messages.RobotTrajectory
        DurabilityPolicy = qos_module.DurabilityPolicy
        QoSProfile = qos_module.QoSProfile
        ReliabilityPolicy = qos_module.ReliabilityPolicy
        qos_profile_sensor_data = qos_module.qos_profile_sensor_data
        RosJointState = sensor_messages.JointState
        JointTrajectory = trajectory_messages.JointTrajectory
        JointTrajectoryPoint = trajectory_messages.JointTrajectoryPoint

        with self._rclpy_lifecycle_lock:
            if not rclpy.ok():
                rclpy.init()
                type(self)._rclpy_initialized_by_driver = True
            if type(self)._rclpy_initialized_by_driver:
                type(self)._active_driver_nodes += 1
                self._uses_driver_rclpy = True

        self._rclpy = rclpy
        self._shutdown_exception_types = tuple(shutdown_exception_types)
        self._node = (
            self._node_factory()
            if self._node_factory is not None
            else rclpy.create_node(self.node_name)
        )
        node = self._node
        if node is None:
            raise RuntimeError("Franka ROS node factory returned None")
        self._owns_node = self._node_factory is None
        self._duration_type = Duration
        self._joint_trajectory_type = JointTrajectory
        self._joint_trajectory_point_type = JointTrajectoryPoint
        self._execute_trajectory_type = ExecuteTrajectory
        self._display_trajectory_type = DisplayTrajectory
        self._moveit_error_codes_type = MoveItErrorCodes
        self._robot_state_type = RobotState
        self._robot_trajectory_type = RobotTrajectory
        self._twist_stamped_type = TwistStamped
        self._pose_stamped_type = PoseStamped
        self._joint_jog_type = JointJog
        self._follow_joint_trajectory_type = FollowJointTrajectory
        self._action_client_type = ActionClient

        self._joint_state_subscription = node.create_subscription(
            RosJointState,
            self.joint_state_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        if self.tcp_pose_topic is not None:
            self._tcp_pose_subscription = node.create_subscription(
                PoseStamped,
                self.tcp_pose_topic,
                self._on_tcp_pose,
                qos_profile_sensor_data,
            )
        self._joint_trajectory_publisher = node.create_publisher(
            JointTrajectory,
            self.joint_trajectory_topic,
            10,
        )
        self._display_trajectory_publisher = node.create_publisher(
            DisplayTrajectory,
            self.display_trajectory_topic,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._delta_twist_publisher = node.create_publisher(
            TwistStamped,
            self.servo_delta_twist_topic,
            10,
        )
        self._delta_joint_publisher = node.create_publisher(
            JointJog,
            self.servo_delta_joint_topic,
            10,
        )
        self._pose_target_publisher = node.create_publisher(
            PoseStamped,
            self.servo_pose_target_topic,
            10,
        )
        if self.joint_position_command == "follow_joint_trajectory":
            self._follow_joint_trajectory_client = ActionClient(
                node,
                FollowJointTrajectory,
                self.follow_joint_trajectory_action,
            )
        if self.joint_position_command == "execute_trajectory":
            self._execute_trajectory_client = ActionClient(
                node,
                ExecuteTrajectory,
                self.execute_trajectory_action,
            )
        if self._should_spin_in_background():
            node_context = getattr(node, "context", None)
            executor = (
                SingleThreadedExecutor(context=node_context)
                if node_context is not None
                else SingleThreadedExecutor()
            )
            executor.add_node(node)
            self._executor = executor
        self._connected = True
        if self._should_spin_in_background():
            self._start_spin_thread()

    def disconnect(self) -> None:
        if not self._connected:
            return
        self._stop_spin_thread()
        if self._ros_context_ok():
            self._safe_stop()
        if self._executor is not None:
            if self._node is not None:
                self._executor.remove_node(self._node)
            self._executor.shutdown()
            self._executor = None
        if self._node is not None and self._owns_node:
            self._node.destroy_node()
        if self._rclpy is not None and self._uses_driver_rclpy:
            self._release_driver_rclpy(self._rclpy)

        self._connected = False
        self._owns_node = False
        self._uses_driver_rclpy = False
        self._rclpy = None
        self._node = None
        self._shutdown_exception_types = ()
        self._joint_state_subscription = None
        self._tcp_pose_subscription = None
        self._joint_trajectory_publisher = None
        self._display_trajectory_publisher = None
        self._delta_twist_publisher = None
        self._delta_joint_publisher = None
        self._pose_target_publisher = None
        self._follow_joint_trajectory_client = None
        self._execute_trajectory_client = None
        self._action_client_type = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def observation_features(self) -> FeatureSpec:
        return {
            "obs_joint": (7,),
            "obs_joint_velocity": (7,),
            "obs_joint_effort": (7,),
            "obs_tcp_pose": (7,),
            "control_command_success_rate": float,
            "q": (7,),
            "dq": (7,),
            "tau_J": (7,),
            "O_T_EE": (7,),
        }

    @property
    def action_features(self) -> FeatureSpec:
        return {
            "action_joint": (7,),
            "action_tcp_pose": (7,),
            "action_joint_trajectory": object,
            "follow_joint_trajectory": object,
            "execute_trajectory": object,
            "action_delta_twist": (6,),
            "action_delta_joint": (7,),
        }

    def read_joint_state(self) -> JointState:
        self._require_connected()
        with self._state_lock:
            joint_state = self._latest_joint_state
        if joint_state is None:
            raise RuntimeError("Franka ROS joint state has not been received")
        return joint_state

    def wait_for_joint_state(self, *, timeout_sec: float | None = None) -> JointState:
        """Wait until a JointState callback has populated this driver's state cache."""

        timeout = self.state_timeout_sec if timeout_sec is None else float(timeout_sec)
        deadline = time.monotonic() + max(timeout, 0.0)
        last_error: RuntimeError | None = None
        while True:
            try:
                return self.read_joint_state()
            except RuntimeError as exc:
                last_error = exc
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f"Timed out waiting for JointState on {self.joint_state_topic} "
                        + f"with joints: {', '.join(self.joint_names)}"
                    ) from last_error
                time.sleep(0.01)

    def read_tcp_pose(self) -> Pose | None:
        self._require_connected()
        with self._state_lock:
            tcp_pose = self._latest_tcp_pose
            franka_state = (
                self._latest_franka_state.as_dict()
                if self._latest_franka_state is not None
                else None
            )
        if tcp_pose is not None:
            return tcp_pose
        if franka_state is not None:
            return _matrix_to_pose(franka_state.get("O_T_EE"))
        return None

    def read_franka_state(self) -> FrankaRobotState:
        self._require_connected()
        with self._state_lock:
            if self._latest_franka_state is None:
                state = FrankaRobotState.zero().as_dict()
            else:
                state = self._latest_franka_state.as_dict()
            joint_state = self._latest_joint_state
            tcp_pose = self._latest_tcp_pose

        if joint_state is not None:
            state["q"] = joint_state.position
            state["q_d"] = joint_state.position
            state["dq"] = joint_state.velocity
            state["dq_d"] = joint_state.velocity
            state["tau_J"] = joint_state.effort
        if tcp_pose is not None:
            state["O_T_EE"] = (
                *tcp_pose.position,
                *tcp_pose.orientation_xyzw,
            )
        state["control_command_success_rate"] = 1.0
        return FrankaRobotState(state)

    def get_observation(self) -> RobotObservation:
        joint_state = self.read_joint_state()
        franka_state = self.read_franka_state().as_dict()
        tcp_pose = self.read_tcp_pose()
        observation: RobotObservation = {
            "obs_joint": joint_state.position,
            "obs_joint_velocity": joint_state.velocity,
            "obs_joint_effort": joint_state.effort,
            "obs_tcp_pose": ()
            if tcp_pose is None
            else (*tcp_pose.position, *tcp_pose.orientation_xyzw),
            "control_command_success_rate": franka_state.get("control_command_success_rate", 0.0),
        }
        observation.update(franka_state)
        return observation

    def send_action(self, action: RobotAction) -> RobotAction:
        result: RobotAction = {}
        if "action_joint" in action:
            target = _as_float_tuple(action["action_joint"], width=7)
            self.command_joint_position(target)
            result["action_joint"] = target
        if "action_tcp_pose" in action:
            target = _as_float_tuple(action["action_tcp_pose"], width=7)
            self.command_tcp_pose(target)
            result["action_tcp_pose"] = target
        if "action_joint_trajectory" in action:
            self.publish_joint_trajectory(action["action_joint_trajectory"])
            result["action_joint_trajectory"] = action["action_joint_trajectory"]
        if "follow_joint_trajectory" in action:
            self.follow_joint_trajectory(action["follow_joint_trajectory"])
            result["follow_joint_trajectory"] = action["follow_joint_trajectory"]
        if "execute_trajectory" in action:
            self.execute_trajectory(action["execute_trajectory"])
            result["execute_trajectory"] = action["execute_trajectory"]
        if "action_delta_twist" in action:
            target = _as_float_tuple(action["action_delta_twist"], width=6)
            self.command_delta_twist(target)
            result["action_delta_twist"] = target
        if "action_delta_joint" in action:
            target = _as_float_tuple(action["action_delta_joint"], width=7)
            self.command_delta_joint(target)
            result["action_delta_joint"] = target
        if not result:
            raise ValueError(
                "Franka ROS action must contain action_joint, action_tcp_pose, "
                + "action_joint_trajectory, follow_joint_trajectory, execute_trajectory, "
                + "action_delta_twist, or action_delta_joint"
            )
        result["accepted"] = True
        return result

    def stop(self) -> None:
        if not self._connected:
            return
        self.command_delta_twist((0.0,) * 6)
        self.command_delta_joint((0.0,) * 7)

    def command_joint_position(self, joint_position: Sequence[float]) -> None:
        target = _as_float_tuple(joint_position, width=7)
        self.send_joint_trajectory([target])

    def send_joint_trajectory(
        self,
        points: object,
        *,
        command_surface: str | None = None,
        timeout_sec: float = 10.0,
        controller_names: Sequence[str] | None = None,
        publish_display: bool = True,
    ) -> object | None:
        """Send a joint trajectory through one named ROS command surface."""

        surface = self.joint_position_command if command_surface is None else command_surface
        if surface == "trajectory_topic":
            self.publish_joint_trajectory(points)
            return None
        if surface == "follow_joint_trajectory":
            return self.follow_joint_trajectory(points, timeout_sec=timeout_sec)
        if surface == "execute_trajectory":
            return self.execute_trajectory(
                points,
                controller_names=controller_names,
                publish_display=publish_display,
                timeout_sec=timeout_sec,
            )
        raise ValueError(
            "Franka ROS command surface must be trajectory_topic, "
            + "follow_joint_trajectory, or execute_trajectory"
        )

    def publish_joint_trajectory(self, points: object) -> None:
        self._require_connected()
        if self._joint_trajectory_publisher is None:
            raise RuntimeError("Franka ROS joint trajectory publisher is not available")
        trajectory = self._make_joint_trajectory(points)
        self._joint_trajectory_publisher.publish(trajectory)

    def follow_joint_trajectory(self, points: object, *, timeout_sec: float = 10.0) -> object:
        self._require_connected()
        action_client = self._follow_joint_trajectory_client
        if action_client is None:
            action_client = self._create_follow_joint_trajectory_client()
        if self._follow_joint_trajectory_type is None:
            raise RuntimeError("Franka ROS FollowJointTrajectory action type is not available")
        if not self.wait_for_follow_joint_trajectory_server(timeout_sec=timeout_sec):
            raise RuntimeError(
                f"Franka ROS FollowJointTrajectory server is not available: {self.follow_joint_trajectory_action}"
            )

        goal = self._follow_joint_trajectory_type.Goal()
        goal.trajectory = self._make_joint_trajectory(points)
        send_goal_future = action_client.send_goal_async(goal)
        self._spin_until_future_complete(send_goal_future, timeout_sec)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Franka ROS FollowJointTrajectory goal was rejected")
        result_future = goal_handle.get_result_async()
        self._spin_until_future_complete(result_future, timeout_sec)
        return result_future.result()

    def wait_for_follow_joint_trajectory_server(self, *, timeout_sec: float) -> bool:
        self._require_connected()
        if self._follow_joint_trajectory_client is None:
            _ = self._create_follow_joint_trajectory_client()
        if not self._ros_context_ok():
            return False
        return bool(self._follow_joint_trajectory_client.wait_for_server(timeout_sec=timeout_sec))

    def wait_for_joint_trajectory_subscribers(
        self,
        *,
        timeout_sec: float,
        minimum_count: int = 1,
    ) -> bool:
        """Wait until the direct trajectory topic has at least one subscriber."""

        self._require_connected()
        if self._node is None:
            raise RuntimeError("Franka ROS node is not available")
        if not hasattr(self._node, "count_subscribers"):
            raise RuntimeError("Franka ROS node cannot count topic subscribers")
        if not self._ros_context_ok():
            return False

        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        while True:
            count = int(self._node.count_subscribers(self.joint_trajectory_topic))
            if count >= minimum_count:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(min(0.05, max(deadline - time.monotonic(), 0.0)))

    def _create_follow_joint_trajectory_client(self) -> object:
        if self._node is None:
            raise RuntimeError("Franka ROS node is not available")
        if self._action_client_type is None:
            raise RuntimeError("Franka ROS ActionClient type is not available")
        if self._follow_joint_trajectory_type is None:
            raise RuntimeError("Franka ROS FollowJointTrajectory action type is not available")
        self._follow_joint_trajectory_client = self._action_client_type(
            self._node,
            self._follow_joint_trajectory_type,
            self.follow_joint_trajectory_action,
        )
        return self._follow_joint_trajectory_client

    def _create_execute_trajectory_client(self) -> object:
        if self._node is None:
            raise RuntimeError("Franka ROS node is not available")
        if self._action_client_type is None:
            raise RuntimeError("Franka ROS ActionClient type is not available")
        if self._execute_trajectory_type is None:
            raise RuntimeError("Franka ROS ExecuteTrajectory action type is not available")
        self._execute_trajectory_client = self._action_client_type(
            self._node,
            self._execute_trajectory_type,
            self.execute_trajectory_action,
        )
        return self._execute_trajectory_client

    def execute_trajectory(
        self,
        points: object,
        *,
        controller_names: Sequence[str] | None = None,
        publish_display: bool = True,
        timeout_sec: float = 10.0,
    ) -> object:
        self._require_connected()
        action_client = self._execute_trajectory_client
        if action_client is None:
            action_client = self._create_execute_trajectory_client()
        if self._execute_trajectory_type is None:
            raise RuntimeError("Franka ROS ExecuteTrajectory action type is not available")
        if not self.wait_for_execute_trajectory_server(timeout_sec=timeout_sec):
            raise RuntimeError(
                f"Franka ROS ExecuteTrajectory server is not available: {self.execute_trajectory_action}"
            )

        robot_trajectory = self._make_robot_trajectory(points)
        if publish_display:
            self.publish_display_trajectory(robot_trajectory)

        goal = self._execute_trajectory_type.Goal()
        goal.trajectory = robot_trajectory
        names = self.execute_trajectory_controller_names if controller_names is None else tuple(controller_names)
        if hasattr(goal, "controller_names"):
            goal.controller_names = [str(name).strip("/") for name in names if str(name).strip("/")]
        send_goal_future = action_client.send_goal_async(goal)
        self._spin_until_future_complete(send_goal_future, timeout_sec)
        goal_handle = send_goal_future.result()
        if goal_handle is None:
            raise RuntimeError("Franka ROS ExecuteTrajectory goal did not return a goal handle")
        if not goal_handle.accepted:
            raise RuntimeError("Franka ROS ExecuteTrajectory goal was rejected")
        result_future = goal_handle.get_result_async()
        self._spin_until_future_complete(result_future, timeout_sec)
        wrapped_result = result_future.result()
        self._raise_for_moveit_error(wrapped_result)
        return wrapped_result

    def publish_display_trajectory(self, trajectory: object) -> None:
        self._require_connected()
        if self._display_trajectory_publisher is None or self._display_trajectory_type is None:
            raise RuntimeError("Franka ROS DisplayTrajectory publisher is not available")
        display_message = self._display_trajectory_type()
        if self._robot_state_type is not None:
            display_message.trajectory_start = self._robot_state_type()
        display_message.trajectory = [self._coerce_robot_trajectory(trajectory)]
        self._display_trajectory_publisher.publish(display_message)

    def wait_for_execute_trajectory_server(self, *, timeout_sec: float) -> bool:
        self._require_connected()
        if self._execute_trajectory_client is None:
            _ = self._create_execute_trajectory_client()
        if not self._ros_context_ok():
            return False
        return bool(self._execute_trajectory_client.wait_for_server(timeout_sec=timeout_sec))

    def command_delta_twist(self, delta_twist: Sequence[float]) -> None:
        self._require_connected()
        if self._delta_twist_publisher is None or self._twist_stamped_type is None:
            raise RuntimeError("Franka ROS servo delta twist publisher is not available")
        values = _as_float_tuple(delta_twist, width=6)
        message = self._twist_stamped_type()
        self._fill_header(message.header)
        message.twist.linear.x = values[0]
        message.twist.linear.y = values[1]
        message.twist.linear.z = values[2]
        message.twist.angular.x = values[3]
        message.twist.angular.y = values[4]
        message.twist.angular.z = values[5]
        self._delta_twist_publisher.publish(message)

    def command_delta_joint(self, delta_joint: Sequence[float]) -> None:
        self._require_connected()
        if self._delta_joint_publisher is None or self._joint_jog_type is None:
            raise RuntimeError("Franka ROS servo delta joint publisher is not available")
        message = self._joint_jog_type()
        self._fill_header(message.header)
        message.joint_names = list(self.joint_names)
        message.displacements = list(_as_float_tuple(delta_joint, width=7))
        if hasattr(message, "velocities"):
            message.velocities = [0.0] * 7
        if hasattr(message, "duration"):
            # control_msgs/JointJog.duration is float64 in Humble and a
            # builtin_interfaces/Duration in newer ROS 2 releases.
            if isinstance(message.duration, float):
                message.duration = float(self.default_trajectory_duration_sec)
            else:
                message.duration = self._make_duration(
                    self.default_trajectory_duration_sec
                )
        self._delta_joint_publisher.publish(message)

    def command_tcp_pose(self, tcp_pose: Sequence[float]) -> None:
        self._require_connected()
        if self._pose_target_publisher is None or self._pose_stamped_type is None:
            raise RuntimeError("Franka ROS servo pose target publisher is not available")
        values = _as_float_tuple(tcp_pose, width=7)
        message = self._pose_stamped_type()
        self._fill_header(message.header)
        message.pose.position.x = values[0]
        message.pose.position.y = values[1]
        message.pose.position.z = values[2]
        message.pose.orientation.x = values[3]
        message.pose.orientation.y = values[4]
        message.pose.orientation.z = values[5]
        message.pose.orientation.w = values[6]
        self._pose_target_publisher.publish(message)

    def legacy_observation(self) -> RobotObservation:
        return self.filter_observation(
            self.get_observation(),
            ("obs_joint", "obs_joint_velocity", "control_command_success_rate"),
        )

    @staticmethod
    def filter_observation(
        observation: Mapping[str, object], feature_names: Iterable[str]
    ) -> RobotObservation:
        return {name: observation[name] for name in feature_names if name in observation}

    @staticmethod
    def filter_action(action: Mapping[str, object], feature_names: Iterable[str]) -> RobotAction:
        return {name: action[name] for name in feature_names if name in action}

    def _make_joint_trajectory(self, points: object) -> object:
        if self._joint_trajectory_type is None:
            raise RuntimeError("Franka ROS JointTrajectory message type is not available")
        trajectory = self._joint_trajectory_type()
        trajectory.joint_names = list(self.joint_names)
        if hasattr(trajectory, "header"):
            self._fill_header(trajectory.header)
        trajectory.points = [
            self._make_joint_trajectory_point(point, index)
            for index, point in enumerate(self._iter_trajectory_points(points))
        ]
        return trajectory

    def _make_joint_trajectory_point(self, point: object, index: int) -> object:
        if self._joint_trajectory_point_type is None:
            raise RuntimeError("Franka ROS JointTrajectoryPoint message type is not available")
        joint_count = len(self.joint_names)
        message = self._joint_trajectory_point_type()
        if isinstance(point, Mapping):
            point_mapping = cast(Mapping[str, object], point)
            message.positions = list(
                _as_float_tuple(point_mapping.get("positions"), width=joint_count)
            )
            if "velocities" in point_mapping:
                message.velocities = list(
                    _as_float_tuple(point_mapping["velocities"], width=joint_count)
                )
            if "accelerations" in point_mapping:
                message.accelerations = list(
                    _as_float_tuple(point_mapping["accelerations"], width=joint_count)
                )
            if "effort" in point_mapping:
                message.effort = list(
                    _as_float_tuple(point_mapping["effort"], width=joint_count)
                )
            time_from_start = float(
                cast(
                    Floatable,
                    point_mapping.get(
                        "time_from_start_sec",
                        self.default_trajectory_duration_sec * (index + 1),
                    ),
                )
            )
        else:
            message.positions = list(_as_float_tuple(point, width=joint_count))
            time_from_start = self.default_trajectory_duration_sec * (index + 1)
        message.time_from_start = self._make_duration(time_from_start)
        return message

    def _iter_trajectory_points(self, points: object) -> tuple[object, ...]:
        if isinstance(points, Mapping):
            points_mapping = cast(Mapping[str, object], points)
            if "points" in points_mapping:
                raw_points = points_mapping["points"]
                if not isinstance(raw_points, Iterable) or isinstance(raw_points, str):
                    raise ValueError("Franka ROS trajectory points must be iterable")
                return tuple(raw_points)
            if "positions" in points_mapping:
                return (points_mapping,)
        if not isinstance(points, Iterable) or isinstance(points, str):
            raise ValueError("Franka ROS trajectory points must be iterable")
        values = tuple(cast(Iterable[object], points))
        if len(values) == len(self.joint_names) and all(isinstance(value, int | float) for value in values):
            return (values,)
        return values

    def _make_robot_trajectory(self, points: object) -> object:
        if self._robot_trajectory_type is None:
            raise RuntimeError("Franka ROS RobotTrajectory message type is not available")
        trajectory = self._robot_trajectory_type()
        trajectory.joint_trajectory = self._make_joint_trajectory(points)
        return trajectory

    def _coerce_robot_trajectory(self, trajectory: object) -> object:
        if self._robot_trajectory_type is not None and isinstance(trajectory, self._robot_trajectory_type):
            return trajectory
        if hasattr(trajectory, "joint_trajectory"):
            return trajectory
        return self._make_robot_trajectory(trajectory)

    def _raise_for_moveit_error(self, wrapped_result: object) -> None:
        result = getattr(wrapped_result, "result", wrapped_result)
        error_code = getattr(result, "error_code", None)
        if error_code is None or self._moveit_error_codes_type is None:
            return
        if int(error_code.val) == int(self._moveit_error_codes_type.SUCCESS):
            return
        status = getattr(wrapped_result, "status", "")
        state = getattr(result, "state", "")
        raise RuntimeError(
            "MoveIt ExecuteTrajectory failed with "
            + f"status={status!r} error_code={int(error_code.val)} state={state!r}"
        )

    def _make_duration(self, duration_sec: float) -> object:
        if self._duration_type is None:
            raise RuntimeError("Franka ROS Duration message type is not available")
        seconds, nanoseconds = _duration_seconds(duration_sec)
        return self._duration_type(sec=seconds, nanosec=nanoseconds)

    def _on_joint_state(self, message: object) -> None:
        names = tuple(
            str(name)
            for name in cast(Iterable[object], getattr(message, "name", ()))
        )
        positions = _as_float_tuple(getattr(message, "position", ()))
        velocities = _as_float_tuple(getattr(message, "velocity", ()))
        efforts = _as_float_tuple(getattr(message, "effort", ()))

        if names:
            index_by_name = {joint_name: index for index, joint_name in enumerate(names)}
            if any(
                joint_name not in index_by_name or index_by_name[joint_name] >= len(positions)
                for joint_name in self.joint_names
            ):
                return
            ordered_positions = tuple(
                positions[index_by_name[joint_name]]
                for joint_name in self.joint_names
            )
            ordered_velocities = tuple(
                velocities[index_by_name[joint_name]]
                if joint_name in index_by_name and index_by_name[joint_name] < len(velocities)
                else 0.0
                for joint_name in self.joint_names
            )
            ordered_efforts = tuple(
                efforts[index_by_name[joint_name]]
                if joint_name in index_by_name and index_by_name[joint_name] < len(efforts)
                else 0.0
                for joint_name in self.joint_names
            )
        else:
            ordered_positions = _as_float_tuple(positions, width=7)
            ordered_velocities = _as_float_tuple(velocities, width=7)
            ordered_efforts = _as_float_tuple(efforts, width=7)

        joint_state = JointState(
            position=ordered_positions,
            velocity=ordered_velocities,
            effort=ordered_efforts,
        )
        with self._state_lock:
            self._latest_joint_state = joint_state

    def _on_tcp_pose(self, message: object) -> None:
        pose = cast(Any, getattr(message, "pose", message))
        position = pose.position
        orientation = pose.orientation
        tcp_pose = Pose(
            position=(float(position.x), float(position.y), float(position.z)),
            orientation_xyzw=(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
        )
        with self._state_lock:
            self._latest_tcp_pose = tcp_pose

    def _fill_header(self, header: object) -> None:
        header_message = cast(Any, header)
        if self._node is not None and hasattr(header_message, "stamp"):
            header_message.stamp = self._node.get_clock().now().to_msg()
        if hasattr(header_message, "frame_id"):
            header_message.frame_id = self.planning_frame

    def _should_spin_in_background(self) -> bool:
        if self._spin_in_background is not None:
            return self._spin_in_background
        return self._node_factory is None

    def _start_spin_thread(self) -> None:
        if self._spin_thread is not None and self._spin_thread.is_alive():
            return
        self._spin_stop_event.clear()
        self._spin_thread = threading.Thread(
            target=self._spin_forever,
            name=f"{self.node_name}-spin",
            daemon=True,
        )
        self._spin_thread.start()

    def _stop_spin_thread(self) -> None:
        self._spin_stop_event.set()
        spin_thread = self._spin_thread
        if spin_thread is not None and spin_thread is not threading.current_thread():
            spin_thread.join(timeout=1.0)
        self._spin_thread = None

    def _spin_forever(self) -> None:
        while not self._spin_stop_event.is_set():
            if self._executor is None:
                return
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as exc:
                if self._is_shutdown_exception(exc):
                    return
                raise

    @classmethod
    def _release_driver_rclpy(cls, rclpy: Any) -> None:
        with cls._rclpy_lifecycle_lock:
            if cls._active_driver_nodes > 0:
                cls._active_driver_nodes -= 1
            if cls._active_driver_nodes == 0 and cls._rclpy_initialized_by_driver:
                if rclpy.ok():
                    rclpy.shutdown()
                cls._rclpy_initialized_by_driver = False

    @staticmethod
    def _optional_import(module_name: str) -> Any | None:
        try:
            return importlib.import_module(module_name)
        except ImportError:
            return None

    def _safe_stop(self) -> None:
        try:
            self.stop()
        except Exception as exc:
            if self._ros_context_ok() and not self._is_shutdown_exception(exc):
                raise

    def _ros_context_ok(self) -> bool:
        node_context = getattr(self._node, "context", None)
        if node_context is not None and hasattr(node_context, "ok"):
            return bool(node_context.ok())
        if self._rclpy is None:
            return False
        return bool(self._rclpy.ok())

    def _is_shutdown_exception(self, exc: BaseException) -> bool:
        return any(isinstance(exc, exception_type) for exception_type in self._shutdown_exception_types)

    def _spin_until_future_complete(self, future: object, timeout_sec: float) -> None:
        if self._rclpy is None or self._node is None:
            raise RuntimeError("Franka ROS node is not connected")
        future_object = cast(Any, future)
        deadline = time.monotonic() + timeout_sec
        while not future_object.done() and time.monotonic() < deadline:
            if not self._connected or not self._ros_context_ok():
                raise RuntimeError("Franka ROS action wait stopped because the ROS context is no longer valid")
            time.sleep(0.01)
        if not future_object.done():
            raise RuntimeError("Franka ROS action did not complete before timeout")

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("Franka ROS robot is not connected")
