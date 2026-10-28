"""ROS 2 backend for the Linker Hand NiceGUI controller."""

from __future__ import annotations

import threading
import atexit
import json
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, ClassVar, SupportsFloat, SupportsIndex, cast

from barmate.core.types import FeatureSpec, JointState, RobotAction
from barmate.hardware.linker_hand.control.types import (
    TACTILE_FINGER_KEYS,
    TactileFrame,
    normalize_tactile_frame,
)
from barmate.hardware.linker_hand.linker_hand_sdk import (
    MODEL_DEFAULT_SPEED,
    MODEL_DEFAULT_TORQUE,
    MODEL_JOINT_COUNTS,
    TACTILE_MATRIX_SHAPE,
    LinkerHandConfig,
)

NumericLike = str | bytes | SupportsFloat | SupportsIndex

_L10_LEFT_MIN = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.26, -0.26, -0.52)
_L10_LEFT_MAX = (1.45, 1.43, 1.62, 1.62, 1.62, 1.62, 0.26, 0.0, 0.0, 1.01)
_L10_LEFT_DIRECTION = (-1, -1, -1, -1, -1, -1, 0, -1, -1, -1)
_L10_RIGHT_MIN = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.26, 0.0, 0.0, -0.52)
_L10_RIGHT_MAX = (0.75, 1.43, 1.62, 1.62, 1.62, 1.62, 0.0, 0.13, 0.26, 1.01)
_L10_RIGHT_DIRECTION = (-1, -1, -1, -1, -1, -1, -1, 0, 0, -1)
_L20_LEFT_MIN = (
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    -0.297,
    -0.26,
    -0.26,
    -0.26,
    -0.26,
    0.122,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
)
_L20_LEFT_MAX = (
    0.87,
    1.4,
    1.4,
    1.4,
    1.4,
    0.683,
    0.26,
    0.26,
    0.26,
    0.26,
    1.78,
    0.0,
    0.0,
    0.0,
    0.0,
    1.29,
    1.08,
    1.08,
    1.08,
    1.08,
)
_L20_LEFT_DIRECTION = (
    -1,
    -1,
    -1,
    -1,
    -1,
    -1,
    -1,
    -1,
    -1,
    -1,
    -1,
    0,
    0,
    0,
    0,
    -1,
    -1,
    -1,
    -1,
    -1,
)
_L20_RIGHT_MIN = (
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    -0.297,
    -0.26,
    -0.26,
    -0.26,
    -0.26,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
)
_L20_RIGHT_MAX = (
    0.87,
    1.4,
    1.4,
    1.4,
    1.4,
    0.683,
    0.26,
    0.26,
    0.26,
    0.26,
    1.78,
    0.0,
    0.0,
    0.0,
    0.0,
    1.29,
    1.08,
    1.08,
    1.08,
    1.08,
)
_L20_RIGHT_DIRECTION = (
    -1,
    -1,
    -1,
    -1,
    -1,
    -1,
    0,
    0,
    0,
    0,
    -1,
    0,
    0,
    0,
    0,
    -1,
    -1,
    -1,
    -1,
    -1,
)

_L10_JOINT_NAMES = (
    "thumb_cmc_pitch",
    "thumb_cmc_yaw",
    "index_mcp_pitch",
    "middle_mcp_pitch",
    "ring_mcp_pitch",
    "pinky_mcp_pitch",
    "index_mcp_roll",
    "ring_mcp_roll",
    "pinky_mcp_roll",
    "thumb_cmc_roll",
)
_TWENTY_JOINT_NAMES = (
    "thumb_cmc_pitch",
    "index_mcp_pitch",
    "middle_mcp_pitch",
    "ring_mcp_pitch",
    "pinky_mcp_pitch",
    "thumb_cmc_yaw",
    "index_mcp_roll",
    "middle_mcp_roll",
    "ring_mcp_roll",
    "pinky_mcp_roll",
    "thumb_cmc_roll",
    "thumb_mcp",
    "index_pip",
    "middle_pip",
    "ring_pip",
    "pinky_pip",
)
_O30I_JOINT_NAMES = (
    "thumb_cmc_roll", "thumb_cmc_yaw", "thumb_mcp", "thumb_ip",
    "index_mcp_roll", "index_mcp_pitch", "index_pip", "index_dip",
    "middle_mcp_roll", "middle_mcp_pitch", "middle_pip", "middle_dip",
    "ring_mcp_roll", "ring_mcp_pitch", "ring_pip", "ring_dip",
    "pinky_mcp_roll", "pinky_mcp_pitch", "pinky_pip", "pinky_dip",
)
_O30I_LOWER = (
    0.0, 0.0, 0.0, 0.0, -0.3741, 0.0, 0.0, 0.0,
    -0.4906, 0.0, 0.0, 0.0, -0.1371, 0.0, 0.0, 0.0,
    -0.1835, 0.0, 0.0, 0.0,
)
_O30I_UPPER = (
    0.5731, 1.9268, 1.5194, 1.5910, 0.03711, 1.5050, 1.5765, 1.5016,
    0.05418, 1.6159, 1.5783, 1.5207, 0.18823, 1.6596, 1.5009, 1.4986,
    0.2810, 1.6152, 1.4858, 1.5549,
)
_G20_COMMAND_NAMES = (
    "Thumb Base", "Index Finger Base", "Middle Finger Base",
    "Ring Finger Base", "Pinky Finger Base", "Thumb Abduction",
    "Index Finger Abduction", "Middle Finger Abduction",
    "Ring Finger Abduction", "Pinky Finger Abduction",
    "Thumb Horizontal Abduction", "Reserved 1", "Reserved 2",
    "Reserved 3", "Reserved 4", "Thumb Tip", "Index Finger Tip",
    "Middle Finger Tip", "Ring Finger Tip", "Pinky Finger Tip",
)


@dataclass(frozen=True, slots=True)
class _ModelMapping:
    min_values: tuple[float, ...]
    max_values: tuple[float, ...]
    directions: tuple[int, ...]
    command_joint_to_raw_index: tuple[int, ...]
    command_joint_names: tuple[str, ...]


def _mapping_for(model: str, side: str) -> _ModelMapping:
    model = model.upper()
    side = side.lower()
    if model == "L10":
        return _ModelMapping(
            min_values=_L10_LEFT_MIN if side == "left" else _L10_RIGHT_MIN,
            max_values=_L10_LEFT_MAX if side == "left" else _L10_RIGHT_MAX,
            directions=_L10_LEFT_DIRECTION if side == "left" else _L10_RIGHT_DIRECTION,
            command_joint_to_raw_index=tuple(range(10)),
            command_joint_names=_L10_JOINT_NAMES,
        )
    if model in {"L20", "G20"}:
        return _ModelMapping(
            min_values=_L20_LEFT_MIN if side == "left" else _L20_RIGHT_MIN,
            max_values=_L20_LEFT_MAX if side == "left" else _L20_RIGHT_MAX,
            directions=_L20_LEFT_DIRECTION if side == "left" else _L20_RIGHT_DIRECTION,
            command_joint_to_raw_index=(
                0,
                1,
                2,
                3,
                4,
                5,
                6,
                7,
                8,
                9,
                10,
                15,
                16,
                17,
                18,
                19,
            ),
            command_joint_names=_TWENTY_JOINT_NAMES,
        )
    if model == "O30I":
        return _ModelMapping(
            min_values=_O30I_LOWER,
            max_values=_O30I_UPPER,
            directions=(1,) * 20,
            command_joint_to_raw_index=tuple(range(20)),
            command_joint_names=_O30I_JOINT_NAMES,
        )
    raise ValueError(f"Unsupported ROS Linker Hand model: {model}")


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return min(maximum, max(minimum, value))


def _scale(
    value: float, from_min: float, from_max: float, to_min: float, to_max: float
) -> float:
    if abs(from_max - from_min) < 1e-12:
        return to_min
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min


def _raw_to_controller_position(
    raw_value: float, mapping: _ModelMapping, raw_index: int
) -> float:
    value = _clamp(raw_value, 0.0, 255.0)
    if mapping.directions[raw_index] == -1:
        return _scale(
            value,
            0.0,
            255.0,
            mapping.max_values[raw_index],
            mapping.min_values[raw_index],
        )
    return _scale(
        value, 0.0, 255.0, mapping.min_values[raw_index], mapping.max_values[raw_index]
    )


def _controller_position_to_raw(
    position: float, mapping: _ModelMapping, raw_index: int
) -> int:
    clamped = _clamp(
        position, mapping.min_values[raw_index], mapping.max_values[raw_index]
    )
    if mapping.directions[raw_index] == -1:
        mapped = _scale(
            clamped,
            mapping.min_values[raw_index],
            mapping.max_values[raw_index],
            255.0,
            0.0,
        )
    else:
        mapped = _scale(
            clamped,
            mapping.min_values[raw_index],
            mapping.max_values[raw_index],
            0.0,
            255.0,
        )
    return int(round(_clamp(mapped, 0.0, 255.0)))


def _normalize_raw(values: Sequence[object], width: int) -> tuple[int, ...]:
    raw = [
        int(round(_clamp(float(cast(NumericLike, value)), 0.0, 255.0)))
        for value in values
    ]
    if len(raw) < width:
        raw.extend([0] * (width - len(raw)))
    return tuple(raw[:width])


def _normalize_config_values(values: Sequence[int | float]) -> tuple[int, ...]:
    return tuple(int(round(_clamp(float(value), 0.0, 255.0))) for value in values)


class _SharedRosNode:
    _instance: ClassVar["_SharedRosNode | None"] = None
    _lock: ClassVar[threading.Lock] = threading.Lock()

    def __init__(self) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init(args=None)
        self.rclpy = rclpy
        self.node = rclpy.create_node("barmate_linker_hand_control")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        atexit.register(self.shutdown)

    @classmethod
    def get(cls) -> "_SharedRosNode":
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def shutdown(self) -> None:
        with self._lock:
            if _SharedRosNode._instance is not self:
                return
            _SharedRosNode._instance = None
        self.executor.shutdown()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.node.destroy_node()
        if self._owns_context and self.rclpy.ok():
            self.rclpy.shutdown()


class RosLinkerHandControlHand:
    """Linker Hand control object that talks to ROS 2 topics."""

    def __init__(
        self,
        *,
        side: str,
        model: str = "L20",
        namespace_root: str = "/linker_hand",
        topic_contract: str = "bridge",
        can: str = "ros",
    ) -> None:
        self.hand_type = side.lower()
        if self.hand_type not in {"left", "right"}:
            raise ValueError(f"side must be left or right, got {side!r}")
        self.hand_joint = model.upper()
        self.config = LinkerHandConfig(
            hand_type=self.hand_type, hand_joint=self.hand_joint, can=can
        )
        self.can = can
        self._topic_contract = topic_contract
        if self._topic_contract not in {"sdk", "ros2_control", "bridge"}:
            raise ValueError(
                "topic_contract must be 'sdk', 'ros2_control', or 'bridge', "
                f"got {topic_contract!r}"
            )
        self._mapping = _mapping_for(self.hand_joint, self.hand_type)
        self._namespace = f"{namespace_root.rstrip('/')}/{self.hand_type}"
        self._raw_joint_count = MODEL_JOINT_COUNTS[self.hand_joint]
        self._raw_names = (
            _O30I_JOINT_NAMES
            if self.hand_joint == "O30I"
            else tuple(
                f"{self.hand_joint.lower()}_raw_joint_{index}"
                for index in range(self._raw_joint_count)
            )
        )
        self._command_topic = (
            f"/cb_{self.hand_type}_hand_control_cmd"
            if self._topic_contract == "sdk"
            else (
                f"/linker_hand_bridge/{self.hand_type}/manual_command"
                if self._topic_contract == "bridge"
                else f"{self._namespace}/hand_controller/commands"
            )
        )
        self._setting_topic = "/cb_hand_setting_cmd"
        self._action_topic = f"{self._namespace}/action_state"
        self._obs_topic = (
            f"/cb_{self.hand_type}_hand_state"
            if self._topic_contract == "sdk"
            else f"{self._namespace}/obs_state"
        )
        self._joint_state_topic = (
            f"/cb_{self.hand_type}_hand_state"
            if self._topic_contract in {"sdk", "bridge"}
            else f"{self._namespace}/joint_states"
        )
        self._matrix_touch_topic = f"/cb_{self.hand_type}_hand_matrix_touch"
        self._speed = MODEL_DEFAULT_SPEED.get(self.hand_joint, ())
        self._torque = MODEL_DEFAULT_TORQUE.get(self.hand_joint, ())
        self._last_command = (0,) * self._raw_joint_count
        self._latest_observed: tuple[int, ...] | None = None
        self._latest_tactile: TactileFrame = {}
        self._observed_event = threading.Event()
        self._lock = threading.RLock()
        self._shared: _SharedRosNode | None = None
        self._action_publisher: Any = None
        self._command_publisher: Any = None
        self._setting_publisher: Any = None
        self._subscriptions: list[Any] = []
        self._command_timer: Any = None
        self._command_active = False

    def connect(self) -> None:
        if self._shared is not None:
            return

        from sensor_msgs.msg import JointState as RosJointState
        from std_msgs.msg import Float64MultiArray, String

        shared = _SharedRosNode.get()
        node = shared.node
        self._action_publisher = node.create_publisher(
            RosJointState, self._action_topic, 10
        )
        if self._topic_contract in {"sdk", "bridge"}:
            self._command_publisher = node.create_publisher(
                RosJointState, self._command_topic, 10
            )
            self._setting_publisher = (
                node.create_publisher(String, self._setting_topic, 10)
                if self._topic_contract == "sdk"
                else None
            )
            self._subscriptions = [
                node.create_subscription(
                    RosJointState,
                    self._joint_state_topic,
                    (
                        self._handle_bridge_state
                        if self._topic_contract == "bridge"
                        else self._handle_obs_state
                    ),
                    10,
                ),
                node.create_subscription(
                    String, self._matrix_touch_topic, self._handle_matrix_touch, 10
                ),
            ]
            if self._topic_contract == "bridge":
                self._command_timer = node.create_timer(
                    0.05, self._republish_manual_command
                )
        else:
            self._command_publisher = node.create_publisher(
                Float64MultiArray, self._command_topic, 10
            )
            self._setting_publisher = None
            self._subscriptions = [
                node.create_subscription(
                    RosJointState, self._obs_topic, self._handle_obs_state, 10
                ),
                node.create_subscription(
                    RosJointState, self._joint_state_topic, self._handle_joint_state, 10
                ),
                node.create_subscription(
                    String, self._matrix_touch_topic, self._handle_matrix_touch, 10
                ),
            ]
        self._shared = shared

    @property
    def is_connected(self) -> bool:
        return self._shared is not None

    @property
    def observation_features(self) -> FeatureSpec:
        features: FeatureSpec = {
            "obs_joint": (self._raw_joint_count,),
            "obs_joint_velocity": (self._raw_joint_count,),
        }
        if self.hand_joint == "G20":
            features.update({key: TACTILE_MATRIX_SHAPE for key in TACTILE_FINGER_KEYS})
        return features

    @property
    def action_features(self) -> FeatureSpec:
        return {"action_joint": (self._raw_joint_count,)}

    @property
    def speed(self) -> tuple[int, ...]:
        return self._speed

    @property
    def torque(self) -> tuple[int, ...]:
        return self._torque

    @property
    def supports_torque(self) -> bool:
        return self.config.supports_torque

    def read_joint_state(self) -> JointState:
        if self.is_connected and not self._observed_event.is_set():
            self._observed_event.wait(timeout=1.0)
        with self._lock:
            position = self._latest_observed or self._last_command
            return JointState(
                position=tuple(float(value) for value in position),
                velocity=(0.0,) * len(position),
            )

    def read_tactile(self) -> TactileFrame:
        if not self.is_connected:
            self.connect()
        with self._lock:
            return dict(self._latest_tactile)

    def send_action(self, action: RobotAction) -> RobotAction:
        joint_position = action.get("action_joint")
        if not isinstance(joint_position, Sequence) or isinstance(joint_position, str):
            return {"accepted": False, "reason": "missing action_joint"}
        if not self.is_connected:
            self.connect()

        raw = _normalize_raw(joint_position, self._raw_joint_count)
        self._publish_action_state(raw)
        self._publish_command(raw)
        with self._lock:
            self._last_command = raw
            self._command_active = True
        return {"accepted": True, "action_joint": tuple(float(value) for value in raw)}

    def set_speed(self, speed: Sequence[int | float]) -> tuple[int, ...]:
        self._speed = _normalize_config_values(speed)
        if not self.is_connected:
            self.connect()
        self._publish_setting_command("set_speed", "speed", self._speed)
        return self._speed

    def set_torque(self, torque: Sequence[int | float]) -> tuple[int, ...]:
        if not self.supports_torque:
            raise NotImplementedError(
                f"{self.hand_joint} does not support torque control"
            )
        self._torque = _normalize_config_values(torque)
        if not self.is_connected:
            self.connect()
        self._publish_setting_command("set_max_torque_limits", "torque", self._torque)
        return self._torque

    def _publish_setting_command(
        self, setting_cmd: str, parameter_name: str, values: tuple[int, ...]
    ) -> None:
        from std_msgs.msg import String

        if self._topic_contract != "sdk" or self._setting_publisher is None:
            return
        message = String()
        message.data = json.dumps(
            {
                "setting_cmd": setting_cmd,
                "params": {
                    "hand_type": self.hand_type,
                    parameter_name: list(values),
                },
            }
        )
        self._setting_publisher.publish(message)

    def _publish_action_state(self, raw: tuple[int, ...]) -> None:
        from sensor_msgs.msg import JointState as RosJointState

        if self._action_publisher is None or self._shared is None:
            return
        message = RosJointState()
        message.header.stamp = self._shared.node.get_clock().now().to_msg()
        message.name = list(self._raw_names)
        message.position = [float(value) for value in raw]
        self._action_publisher.publish(message)

    def _publish_command(self, raw: tuple[int, ...]) -> None:
        if self._topic_contract == "sdk":
            self._publish_sdk_command(raw)
        elif self._topic_contract == "bridge":
            self._publish_bridge_command(raw)
        else:
            self._publish_controller_command(raw)

    def _republish_manual_command(self) -> None:
        with self._lock:
            if not self._command_active:
                return
            raw = self._last_command
        self._publish_bridge_command(raw)

    def _publish_bridge_command(self, raw: tuple[int, ...]) -> None:
        from sensor_msgs.msg import JointState as RosJointState

        if self._command_publisher is None or self._shared is None:
            return
        message = RosJointState()
        message.header.stamp = self._shared.node.get_clock().now().to_msg()
        if self.hand_joint == "G20":
            message.name = list(_G20_COMMAND_NAMES)
            message.position = [float(value) for value in raw]
        elif self.hand_joint == "O30I":
            message.name = list(_O30I_JOINT_NAMES)
            message.position = [
                _raw_to_controller_position(float(raw[index]), self._mapping, index)
                for index in range(20)
            ]
        else:
            raise RuntimeError(
                "bridge UI supports the target workcell's G20 and O30I models only"
            )
        self._command_publisher.publish(message)

    def _publish_sdk_command(self, raw: tuple[int, ...]) -> None:
        from sensor_msgs.msg import JointState as RosJointState

        if self._command_publisher is None or self._shared is None:
            return
        message = RosJointState()
        message.header.stamp = self._shared.node.get_clock().now().to_msg()
        message.name = list(self._raw_names)
        message.position = [float(value) for value in raw]
        self._command_publisher.publish(message)

    def _publish_controller_command(self, raw: tuple[int, ...]) -> None:
        from std_msgs.msg import Float64MultiArray

        if self._command_publisher is None:
            return
        message = Float64MultiArray()
        message.data = [
            _raw_to_controller_position(float(raw[raw_index]), self._mapping, raw_index)
            for raw_index in self._mapping.command_joint_to_raw_index
        ]
        self._command_publisher.publish(message)

    def _handle_obs_state(self, message: Any) -> None:
        with self._lock:
            self._latest_observed = _normalize_raw(
                message.position, self._raw_joint_count
            )
        self._observed_event.set()

    def _handle_bridge_state(self, message: Any) -> None:
        if self.hand_joint == "G20":
            self._handle_obs_state(message)
        else:
            self._handle_joint_state(message)

    def _handle_joint_state(self, message: Any) -> None:
        raw = [0] * self._raw_joint_count
        name_to_raw_index = dict(
            zip(
                self._mapping.command_joint_names,
                self._mapping.command_joint_to_raw_index,
                strict=True,
            )
        )
        if message.name:
            for name, position in zip(message.name, message.position, strict=False):
                raw_index = name_to_raw_index.get(name)
                if raw_index is not None:
                    raw[raw_index] = _controller_position_to_raw(
                        float(position), self._mapping, raw_index
                    )
        else:
            for command_index, position in enumerate(
                message.position[: len(self._mapping.command_joint_to_raw_index)]
            ):
                raw_index = self._mapping.command_joint_to_raw_index[command_index]
                raw[raw_index] = _controller_position_to_raw(
                    float(position), self._mapping, raw_index
                )
        with self._lock:
            self._latest_observed = tuple(raw)
        self._observed_event.set()

    def _handle_matrix_touch(self, message: Any) -> None:
        try:
            payload = json.loads(str(message.data))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        frame = normalize_tactile_frame(payload)
        with self._lock:
            self._latest_tactile = frame
