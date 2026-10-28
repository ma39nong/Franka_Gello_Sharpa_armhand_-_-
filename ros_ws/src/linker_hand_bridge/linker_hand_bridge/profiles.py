"""Model-specific hand command contracts used by the safety bridge.

The bridge owns liveness and slew safety, while profiles own hardware semantics:
joint order, value ranges, fixed slots, kinematic projection, home pose, and
startup settings. Adding a hand model should add a profile rather than branch on
the model throughout the bridge.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Protocol, Sequence

from .core import (
    COMMAND_SLOTS,
    G20_JOINT_NAMES,
    RESERVED_SLOTS,
    G20Mapper,
    QposPacket,
    validate_hand_state,
)

O30I_JOINT_NAMES = (
    "thumb_cmc_roll",
    "thumb_cmc_yaw",
    "thumb_mcp",
    "thumb_ip",
    "index_mcp_roll",
    "index_mcp_pitch",
    "index_pip",
    "index_dip",
    "middle_mcp_roll",
    "middle_mcp_pitch",
    "middle_pip",
    "middle_dip",
    "ring_mcp_roll",
    "ring_mcp_pitch",
    "ring_pip",
    "ring_dip",
    "pinky_mcp_roll",
    "pinky_mcp_pitch",
    "pinky_pip",
    "pinky_dip",
)
# 0803 vendor URDF limits (corrected from 0706; see o30i_contract.py)
O30I_RIGHT_LOWER = (
    0.0, 0.0, 0.0, 0.0,
    -0.3741, 0.0, 0.0, 0.0,
    -0.4906, 0.0, 0.0, 0.0,
    -0.1371, 0.0, 0.0, 0.0,
    -0.1835, 0.0, 0.0, 0.0,
)
O30I_RIGHT_UPPER = (
    0.5731, 1.9268, 1.5194, 1.5910,
    0.03711, 1.5050, 1.5765, 1.5016,
    0.05418, 1.6159, 1.5783, 1.5207,
    0.18823, 1.6596, 1.5009, 1.4986,
    0.2810, 1.6152, 1.4858, 1.5549,
)


@dataclass(frozen=True)
class StartupSetting:
    command: str
    values_key: str
    values: tuple[int, ...]


class HandCommandMapper(Protocol):
    def map_packet(self, packet: QposPacket) -> tuple[float, ...]:
        ...

    def home(self, side: str) -> tuple[float, ...]:
        ...


@dataclass(frozen=True)
class HandDeviceProfile:
    model: str
    command_joint_names: tuple[str, ...]
    lower_bounds: tuple[float, ...]
    upper_bounds: tuple[float, ...]
    fixed_values: dict[int, float]
    max_publish_rate: float
    max_slew_rate: float
    mapper: HandCommandMapper
    allow_untagged_packets: bool
    startup_settings_factory: Callable[
        [int, int, int], tuple[StartupSetting, ...]
    ]

    @property
    def command_slots(self) -> int:
        return len(self.command_joint_names)

    def map_packet(self, packet: QposPacket) -> tuple[float, ...]:
        if packet.model is None and not self.allow_untagged_packets:
            raise ValueError(
                f"{packet.side} packet is missing required model tag "
                f"{self.model!r}"
            )
        if packet.model is not None and packet.model != self.model:
            raise ValueError(
                f"{packet.side} packet model {packet.model!r} does not match "
                f"configured model {self.model!r}"
            )
        return self.mapper.map_packet(packet)

    def home(self, side: str) -> tuple[float, ...]:
        return self.mapper.home(side)

    def validate_state(
        self, positions: Sequence[float]
    ) -> tuple[float, ...] | None:
        return validate_hand_state(
            positions,
            command_slots=self.command_slots,
            lower_bounds=self.lower_bounds,
            upper_bounds=self.upper_bounds,
        )

    def startup_settings(
        self, speed: int, finger_torque: int, thumb_torque: int
    ) -> tuple[StartupSetting, ...]:
        return self.startup_settings_factory(speed, finger_torque, thumb_torque)


def _g20_startup_settings(
    speed: int, finger_torque: int, thumb_torque: int
) -> tuple[StartupSetting, ...]:
    settings = []
    if speed > 0:
        settings.append(StartupSetting("set_speed", "speed", (speed,) * 5))
    if finger_torque > 0:
        settings.append(
            StartupSetting(
                "set_max_torque_limits",
                "torque",
                (thumb_torque,) + (finger_torque,) * 4,
            )
        )
    return tuple(settings)


class O30IMapper:
    """Validate and reorder canonical O30i URDF radians."""

    def map_packet(self, packet: QposPacket) -> tuple[float, ...]:
        if len(packet.joint_names) != len(packet.qpos):
            raise ValueError("O30i joint name/qpos length mismatch")
        by_name = dict(zip(packet.joint_names, packet.qpos, strict=True))
        if set(by_name) != set(O30I_JOINT_NAMES):
            missing = sorted(set(O30I_JOINT_NAMES) - set(by_name))
            unknown = sorted(set(by_name) - set(O30I_JOINT_NAMES))
            raise ValueError(
                f"O30i qpos contract differs: missing={missing}, unknown={unknown}"
            )
        values = tuple(float(by_name[name]) for name in O30I_JOINT_NAMES)
        violations = [
            name
            for name, value, lower, upper in zip(
                O30I_JOINT_NAMES,
                values,
                O30I_RIGHT_LOWER,
                O30I_RIGHT_UPPER,
                strict=True,
            )
            if value < lower or value > upper
        ]
        if violations:
            raise ValueError("O30i URDF limits exceeded: " + ", ".join(violations))
        return values

    def home(self, side: str) -> tuple[float, ...]:
        if side not in ("left", "right"):
            raise ValueError(f"unknown side {side!r}")
        return (0.0,) * len(O30I_JOINT_NAMES)


def _no_startup_settings(
    _speed: int, _finger_torque: int, _thumb_torque: int
) -> tuple[StartupSetting, ...]:
    return ()


def create_hand_profile(
    model: str, *, side: str = "right", abduction_invert: bool = False
) -> HandDeviceProfile:
    normalized = str(model).strip().lower()
    if normalized not in {"g20", "o30i"}:
        raise ValueError(
            f"unsupported hand model {model!r}; registered models: ['g20', 'o30i']"
        )
    if normalized == "o30i":
        # Both sides share one contract: the vendor's left and right 0803
        # URDFs carry IDENTICAL joint names and limits (mirroring lives in
        # the link geometry), verified 2026-08-04 when the left mount became
        # an O30i.
        if side not in ("left", "right"):
            raise ValueError(f"unknown side {side!r}")
        return HandDeviceProfile(
            model="o30i",
            command_joint_names=O30I_JOINT_NAMES,
            lower_bounds=O30I_RIGHT_LOWER,
            upper_bounds=O30I_RIGHT_UPPER,
            fixed_values={},
            max_publish_rate=30.0,
            max_slew_rate=12.0,
            mapper=O30IMapper(),
            allow_untagged_packets=False,
            startup_settings_factory=_no_startup_settings,
        )
    fixed_values = {
        index: 0.0
        for index in range(RESERVED_SLOTS.start, RESERVED_SLOTS.stop)
    }
    return HandDeviceProfile(
        model="g20",
        command_joint_names=G20_JOINT_NAMES,
        lower_bounds=(0.0,) * COMMAND_SLOTS,
        upper_bounds=(255.0,) * COMMAND_SLOTS,
        fixed_values=fixed_values,
        max_publish_rate=30.0,
        max_slew_rate=1500.0,
        mapper=G20Mapper(abduction_invert=abduction_invert),
        allow_untagged_packets=True,
        startup_settings_factory=_g20_startup_settings,
    )
