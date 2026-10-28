"""Linker Hand SDK adapter and mock implementing the core RobotInterface."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Protocol, SupportsFloat, SupportsIndex, cast

from barmate.core.robot import RobotInterface
from barmate.core.types import (
    FeatureSpec,
    JointState,
    Pose,
    RobotAction,
    RobotObservation,
)

# Implements barmate.core.robot.RobotInterface via barmate.vendor.linkerhand_sdk.

MODEL_JOINT_COUNTS = {
    "L6": 6,
    "L7": 7,
    "L10": 10,
    "L20": 20,
    "L21": 21,
    "L25": 25,
    "G20": 20,
    "O6": 6,
    "O30I": 20,
}
MODEL_RESERVED_JOINT_INDICES = {
    "G20": frozenset({11, 12, 13, 14}),
}
RESERVED_JOINT_DEFAULT = 0
MODEL_DEFAULT_SPEED = {
    "L10": (180, 250, 250, 250, 250),
    "L20": (255, 255, 255, 255, 255),
    "G20": (150, 150, 150, 150, 150),
    "O30I": (255,),
}
MODEL_DEFAULT_TORQUE = {
    "L10": (200, 200, 200, 200, 200),
    "L20": (200, 200, 200, 200, 200),
    "G20": (245, 245, 245, 245, 245),
}
MODEL_TORQUE_UNSUPPORTED = frozenset({"O30I"})
NumericLike = str | bytes | SupportsFloat | SupportsIndex
TACTILE_MATRIX_SHAPE = (12, 6)
TACTILE_MATRIX_KEYS = (
    "thumb_matrix",
    "index_matrix",
    "middle_matrix",
    "ring_matrix",
    "little_matrix",
)


@dataclass(frozen=True, slots=True)
class LinkerHandConfig:
    """Small Linker Hand connection configuration for root-package drivers."""

    hand_type: str = "right"
    hand_joint: str = "L20"
    modbus: str = "None"
    can: str = "can0"
    speed: tuple[int, ...] | None = None
    torque: tuple[int, ...] | None = None

    @property
    def model(self) -> str:
        return self.hand_joint.upper()

    @property
    def joint_count(self) -> int:
        if self.model not in MODEL_JOINT_COUNTS:
            raise ValueError(f"Unsupported LinkerHand model: {self.model}")
        return MODEL_JOINT_COUNTS[self.model]

    @property
    def resolved_speed(self) -> tuple[int, ...]:
        return self.speed or MODEL_DEFAULT_SPEED.get(self.model, ())

    @property
    def resolved_torque(self) -> tuple[int, ...]:
        return self.torque or MODEL_DEFAULT_TORQUE.get(self.model, ())

    @property
    def supports_torque(self) -> bool:
        return self.model not in MODEL_TORQUE_UNSUPPORTED


@dataclass(frozen=True, slots=True)
class LinkerHandPairConfig:
    """Connection choices for a left/right Linker Hand pair."""

    left_hand_joint: str = "G20"
    right_hand_joint: str = "L10"
    left_can: str = "can1"
    right_can: str = "can0"
    mock: bool = False


class LinkerHandPairFactory:
    """Factory for real or in-process Linker Hand pairs."""

    def create_pair(
        self, config: LinkerHandPairConfig
    ) -> tuple["LinkerHandRobot", "LinkerHandRobot"]:
        return (
            self.create_hand(
                "left",
                hand_joint=config.left_hand_joint,
                can=config.left_can,
                mock=config.mock,
            ),
            self.create_hand(
                "right",
                hand_joint=config.right_hand_joint,
                can=config.right_can,
                mock=config.mock,
            ),
        )

    def create_hand(
        self,
        hand_type: str,
        *,
        hand_joint: str,
        can: str,
        mock: bool = False,
    ) -> "LinkerHandRobot":
        if mock:
            return LinkerHandMockRobot(hand_type=hand_type, hand_joint=hand_joint)
        return LinkerHandRobot(hand_type=hand_type, hand_joint=hand_joint, can=can)


class _LinkerHandApi(Protocol):
    """Vendored LinkerHand SDK methods used by the RobotInterface adapter."""

    def set_speed(self, speed: list[int]) -> object: ...

    def set_torque(self, torque: list[int]) -> object: ...

    def finger_move(self, pose: list[int]) -> None: ...

    def get_state(self) -> Sequence[object] | None: ...

    def close_can(self) -> None: ...

    def get_thumb_matrix_touch(self) -> object: ...

    def get_index_matrix_touch(self) -> object: ...

    def get_middle_matrix_touch(self) -> object: ...

    def get_ring_matrix_touch(self) -> object: ...

    def get_little_matrix_touch(self) -> object: ...


class LinkerHandRobot(RobotInterface):
    """RobotInterface adapter around the vendored official LinkerHand SDK."""

    def __init__(
        self,
        hand_type: str = "right",
        hand_joint: str = "L20",
        *,
        modbus: str = "None",
        can: str = "can0",
        speed: Sequence[int] | None = None,
        torque: Sequence[int] | None = None,
        api_factory: Callable[..., _LinkerHandApi] | None = None,
    ) -> None:
        self.config = LinkerHandConfig(
            hand_type=hand_type,
            hand_joint=hand_joint,
            modbus=modbus,
            can=can,
            speed=None if speed is None else tuple(int(value) for value in speed),
            torque=None if torque is None else tuple(int(value) for value in torque),
        )
        self.hand_type = self.config.hand_type
        self.hand_joint = self.config.model
        self.modbus = self.config.modbus
        self.can = self.config.can
        self._api_factory = api_factory
        self._api: _LinkerHandApi | None = None
        self._last_command: tuple[float, ...] | None = None
        self._speed = self.config.resolved_speed
        self._torque = self.config.resolved_torque

    def connect(self) -> None:
        if self._api is not None:
            return
        if self.hand_joint == "O30I":
            raise RuntimeError(
                "O30I uses the ROS 2 CAN-FD driver; select the 'ros' backend"
            )
        factory = self._api_factory
        if factory is None:
            from barmate.vendor.linkerhand_sdk.linker_hand_api import LinkerHandApi

            factory = LinkerHandApi
        self._api = factory(
            hand_type=self.hand_type,
            hand_joint=self.hand_joint,
            modbus=self.modbus,
            can=self.can,
        )
        self.configure()

    def configure(
        self,
        *,
        speed: Sequence[int | float] | None = None,
        torque: Sequence[int | float] | None = None,
    ) -> None:
        """Apply speed and torque configuration at connection time or runtime."""

        if speed is not None:
            self.set_speed(speed)
        elif self._speed:
            self.set_speed(self._speed)
        if torque is not None:
            self.set_torque(torque)
        elif self._torque:
            self.set_torque(self._torque)

    @property
    def speed(self) -> tuple[int, ...]:
        """Current configured finger speed values."""

        return self._speed

    @property
    def torque(self) -> tuple[int, ...]:
        """Current configured finger torque values."""

        return self._torque

    @property
    def supports_torque(self) -> bool:
        """Whether the selected hand model accepts runtime torque commands."""

        return self.config.supports_torque

    def set_speed(self, speed: Sequence[int | float]) -> tuple[int, ...]:
        """Configure finger speed on the connected Linker Hand SDK object."""

        values = _normalize_config_values(speed)
        api = self._require_api()
        api.set_speed(list(values))
        self._speed = values
        return values

    def set_torque(self, torque: Sequence[int | float]) -> tuple[int, ...]:
        """Configure finger torque on the connected Linker Hand SDK object."""

        if not self.supports_torque:
            raise NotImplementedError(
                f"{self.hand_joint} does not support torque control"
            )
        values = _normalize_config_values(torque)
        api = self._require_api()
        api.set_torque(list(values))
        self._torque = values
        return values

    def disconnect(self) -> None:
        api = self._api
        if api is not None:
            pass
            # api.close_can()
        self._api = None

    @property
    def is_connected(self) -> bool:
        return self._api is not None

    @property
    def observation_features(self) -> FeatureSpec:
        features: FeatureSpec = {
            "obs_joint": (self.joint_count,),
            "obs_joint_velocity": (self.joint_count,),
        }
        if self.hand_joint == "G20":
            features.update({key: TACTILE_MATRIX_SHAPE for key in TACTILE_MATRIX_KEYS})
        return features

    @property
    def action_features(self) -> FeatureSpec:
        return {"action_joint": (self.joint_count,)}

    @property
    def joint_count(self) -> int:
        return self.config.joint_count

    def read_position(self) -> tuple[int, ...]:
        api = self._require_api()
        return self._normalize_position(api.get_state())

    def command_position(self, position: Sequence[object]) -> None:
        api = self._require_api()
        target = self._normalize_command_position(position)
        api.finger_move(target)
        self._last_command = tuple(float(value) for value in target)

    def read_command_state(self) -> tuple[float, ...]:
        """Return the most recently commanded finger positions without hardware read.

        On the first call (before any command has been sent), reads the
        hardware state once and caches it, so the return value is always
        the actual position — never a sentinel like all-zeros.
        """
        if self._last_command is None:
            target = self._read_current_command_position()
            self._last_command = tuple(float(value) for value in target)
        return self._last_command

    def read_joint_state(self) -> JointState:
        position = tuple(float(value) for value in self.read_position())
        return JointState(position=position, velocity=(0.0,) * len(position))

    def read_tcp_pose(self) -> Pose | None:
        return None

    def get_observation(self) -> RobotObservation:
        joint_state = self.read_joint_state()
        observation: RobotObservation = {
            "obs_joint": joint_state.position,
            "obs_joint_velocity": joint_state.velocity,
        }
        observation.update(self.read_tactile())
        return observation

    def read_tactile(self) -> RobotObservation:
        if self.hand_joint != "G20" or self._api is None:
            return {}
        return {
            "thumb_matrix": self._api.get_thumb_matrix_touch(),
            "index_matrix": self._api.get_index_matrix_touch(),
            "middle_matrix": self._api.get_middle_matrix_touch(),
            "ring_matrix": self._api.get_ring_matrix_touch(),
            "little_matrix": self._api.get_little_matrix_touch(),
        }

    def send_action(self, action: RobotAction) -> RobotAction:
        joint_position = action.get("action_joint")
        if isinstance(joint_position, Sequence) and not isinstance(joint_position, str):
            self.command_position(joint_position)
            return {"accepted": True, "action_joint": self._last_command}
        return {"accepted": False, "reason": "missing action_joint"}

    def _normalize_position(self, state: Sequence[object] | None) -> tuple[int, ...]:
        if state is None:
            return (0,) * self.joint_count
        values = tuple(int(float(cast(NumericLike, value))) for value in state)
        if len(values) >= self.joint_count:
            values = values[: self.joint_count]
        else:
            values = values + (0,) * (self.joint_count - len(values))
        return self._normalize_reserved_joint_values(values)

    def _read_current_command_position(self) -> tuple[int, ...]:
        state = self._require_api().get_state()
        if state is None:
            raise RuntimeError(
                f"{self.hand_type} {self.hand_joint} current joint state is unavailable"
            )
        values = tuple(int(float(cast(NumericLike, value))) for value in state)
        if len(values) < self.joint_count:
            raise RuntimeError(
                f"{self.hand_type} {self.hand_joint} current joint state has "
                f"{len(values)} value(s), expected {self.joint_count}: {values}"
            )
        values = values[: self.joint_count]
        values = self._normalize_reserved_joint_values(values)
        invalid = tuple(value for value in values if value < 0 or value > 255)
        if invalid:
            raise RuntimeError(
                f"{self.hand_type} {self.hand_joint} current joint state contains "
                f"invalid value(s) outside 0..255: {values}"
            )
        return values

    def _normalize_reserved_joint_values(
        self, values: tuple[int, ...]
    ) -> tuple[int, ...]:
        reserved_indices = MODEL_RESERVED_JOINT_INDICES.get(self.config.model)
        if not reserved_indices:
            return values
        return tuple(
            RESERVED_JOINT_DEFAULT if index in reserved_indices and value < 0 else value
            for index, value in enumerate(values)
        )

    def _normalize_command_position(self, position: Sequence[object]) -> list[int]:
        target = [
            max(0, min(255, int(round(float(cast(NumericLike, value))))))
            for value in position
        ]
        if len(target) < self.joint_count:
            target.extend([0] * (self.joint_count - len(target)))
        return target[: self.joint_count]

    def _require_api(self) -> _LinkerHandApi:
        if self._api is None:
            raise RuntimeError("Linker Hand is not connected")
        return self._api


class LinkerHandMockRobot(LinkerHandRobot):
    """In-process Linker Hand mock implementing the core RobotInterface."""

    def __init__(
        self,
        hand_type: str = "right",
        hand_joint: str = "L20",
        *,
        joint_position: Sequence[float] | None = None,
    ) -> None:
        super().__init__(hand_type=hand_type, hand_joint=hand_joint)
        initial = (
            (0.0,) * self.joint_count if joint_position is None else joint_position
        )
        self._joint_position = tuple(
            float(value) for value in initial[: self.joint_count]
        )
        if len(self._joint_position) < self.joint_count:
            self._joint_position = self._joint_position + (0.0,) * (
                self.joint_count - len(self._joint_position)
            )
        self._joint_velocity = (0.0,) * self.joint_count
        self.speed_calls: list[tuple[int, ...]] = []
        self.torque_calls: list[tuple[int, ...]] = []

    def connect(self) -> None:
        self._api = self
        self.configure()

    def configure(
        self,
        *,
        speed: Sequence[int | float] | None = None,
        torque: Sequence[int | float] | None = None,
    ) -> None:
        if speed is not None:
            self.set_speed(speed)
        elif self._speed:
            self.set_speed(self._speed)
        if torque is not None:
            self.set_torque(torque)
        elif self._torque:
            self.set_torque(self._torque)

    def disconnect(self) -> None:
        self._api = None

    def close_can(self) -> None:
        return None

    def set_speed(self, speed: Sequence[int | float]) -> tuple[int, ...]:
        values = _normalize_config_values(speed)
        self._speed = values
        self.speed_calls.append(values)
        return values

    def set_torque(self, torque: Sequence[int | float]) -> tuple[int, ...]:
        if not self.supports_torque:
            raise NotImplementedError(
                f"{self.hand_joint} does not support torque control"
            )
        values = _normalize_config_values(torque)
        self._torque = values
        self.torque_calls.append(values)
        return values

    def finger_move(self, pose: list[int]) -> None:
        previous = self._joint_position
        target = tuple(float(value) for value in pose[: self.joint_count])
        if len(target) < self.joint_count:
            target = target + (0.0,) * (self.joint_count - len(target))
        self._joint_position = target
        self._joint_velocity = tuple(
            self._joint_position[index] - previous[index]
            for index in range(self.joint_count)
        )

    def get_state(self) -> tuple[int, ...]:
        return tuple(int(value) for value in self._joint_position)

    def read_joint_state(self) -> JointState:
        return JointState(position=self._joint_position, velocity=self._joint_velocity)

    def read_position(self) -> tuple[int, ...]:
        return self.get_state()

    def _tactile_matrix(self, offset: int) -> tuple[tuple[int, ...], ...]:
        return tuple(
            tuple(
                offset + row * TACTILE_MATRIX_SHAPE[1] + column
                for column in range(TACTILE_MATRIX_SHAPE[1])
            )
            for row in range(TACTILE_MATRIX_SHAPE[0])
        )

    def get_thumb_matrix_touch(self) -> tuple[tuple[int, ...], ...]:
        return self._tactile_matrix(0)

    def get_index_matrix_touch(self) -> tuple[tuple[int, ...], ...]:
        return self._tactile_matrix(100)

    def get_middle_matrix_touch(self) -> tuple[tuple[int, ...], ...]:
        return self._tactile_matrix(200)

    def get_ring_matrix_touch(self) -> tuple[tuple[int, ...], ...]:
        return self._tactile_matrix(300)

    def get_little_matrix_touch(self) -> tuple[tuple[int, ...], ...]:
        return self._tactile_matrix(400)


def _normalize_config_values(values: Sequence[int | float]) -> tuple[int, ...]:
    return tuple(max(0, min(255, int(round(float(value))))) for value in values)
