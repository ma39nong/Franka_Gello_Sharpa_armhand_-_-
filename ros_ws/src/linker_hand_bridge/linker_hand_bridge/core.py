"""ROS-independent packet decoding, device projection, and slew limiting.

Ported from the sibling WiLoR repository's
`ros2/wilor_linkerhand_bridge/wilor_linkerhand_bridge/core.py`. The projection
tables and quantization are kept unchanged because they encode the vendor's
range directions; the sibling repository is a read-only reference, so this is a
copy rather than an import.

This module must not acquire a PICO dependency. It sees only named URDF joint
angles arriving over a socket.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Sequence

PROTOCOL_VERSION = 1
MAX_DATAGRAM_BYTES = 16_384
COMMAND_SLOTS = 20
RESERVED_SLOTS = slice(11, 15)

# Official G20 command order, as reported by the vendor driver's
# get_finger_order() and verified against live hardware state.
G20_JOINT_NAMES = (
    "Thumb Base",
    "Index Finger Base",
    "Middle Finger Base",
    "Ring Finger Base",
    "Pinky Finger Base",
    "Thumb Abduction",
    "Index Finger Abduction",
    "Middle Finger Abduction",
    "Ring Finger Abduction",
    "Pinky Finger Abduction",
    "Thumb Horizontal Abduction",
    "Reserved 1",
    "Reserved 2",
    "Reserved 3",
    "Reserved 4",
    "Thumb Tip",
    "Index Finger Tip",
    "Middle Finger Tip",
    "Ring Finger Tip",
    "Pinky Finger Tip",
)

FINGERS = ("index", "middle", "ring", "pinky")

# Radian endpoints must match the kinematic model that produced each packet.
# The installed left G20 is the officially supplied L20 V10.1 mechanism.  The
# right G20 fallback remains on the older model until that physical serial is
# verified; the deployed right hand is normally an O30i and uses another mapper.
FINGER_LIMITS = {
    "mcp_roll": (-0.17, 0.17),
    "mcp_pitch": (0.0, 1.4),
    "pip": (0.0, 1.57),
    "dip": (0.0, 1.4),
}
THUMB_LIMITS = {
    "thumb_cmc_yaw": (0.0, 1.4),
    "thumb_cmc_roll": (0.0, 1.22),
    "thumb_cmc_pitch": (0.0, 0.79),
    "thumb_mcp": (0.0, 1.05),
    "thumb_tip": (0.0, 1.22),
}
LEFT_V101_FINGER_LIMITS = {
    "mcp_roll": (-0.23, 0.23),
    "mcp_pitch": (0.0, 1.2217),
    "pip": (0.0, 1.7279),
    "dip": (0.0, 1.3614),
}
LEFT_V101_THUMB_LIMITS = {
    "thumb_cmc_yaw": (0.0, 1.5882),
    "thumb_cmc_roll": (0.0, 1.309),
    "thumb_cmc_pitch": (0.0, 0.7854),
    "thumb_mcp": (0.0, 1.2217),
    "thumb_tip": (0.0, 1.2392),
}


def _g20_limits(side: str):
    if side == "left":
        return LEFT_V101_FINGER_LIMITS, LEFT_V101_THUMB_LIMITS
    return FINGER_LIMITS, THUMB_LIMITS

# Whether a side's vendor abduction slot runs opposite to its selected URDF's
# mcp_roll coordinate. Hardware establishes that left slot 255 moves the index
# toward the thumb. In the verified left V10.1 model the roll axis is reversed
# from the former URDF, so its mapping must now be inverted. The unverified
# fallback right G20 remains paired with the former right model and polarity.
# `test_abduction_polarity_matches_urdf_geometry` re-derives the result from the
# selected model and fails if an asset changes without a matching migration.
ABDUCTION_INVERTED = {"left": True, "right": True}


@dataclass(frozen=True)
class QposPacket:
    stream_id: str
    sequence: int
    timestamp: float
    side: str
    model: str | None
    joint_names: tuple[str, ...]
    qpos: tuple[float, ...]


def decode_qpos_packet(payload: bytes) -> QposPacket:
    """Decode and validate a hand qpos datagram."""
    if len(payload) > MAX_DATAGRAM_BYTES:
        raise ValueError(f"packet is too large: {len(payload)} bytes")
    try:
        message = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("invalid qpos packet JSON") from error
    if not isinstance(message, dict):
        raise ValueError("qpos packet JSON must be an object")

    required = {
        "version",
        "stream_id",
        "sequence",
        "timestamp",
        "side",
        "joint_names",
        "qpos",
    }
    missing = required.difference(message)
    if missing:
        raise ValueError(f"qpos packet is missing fields: {sorted(missing)}")
    if message["version"] != PROTOCOL_VERSION:
        raise ValueError(f"unsupported protocol version: {message['version']}")
    if message["side"] not in {"left", "right"}:
        raise ValueError(f"invalid hand side: {message['side']!r}")

    names = tuple(str(name) for name in message["joint_names"])
    if len(names) != len(set(names)):
        raise ValueError("qpos packet contains duplicate joint names")
    try:
        qpos = tuple(float(value) for value in message["qpos"])
        sequence = int(message["sequence"])
        timestamp = float(message["timestamp"])
    except (TypeError, ValueError) as error:
        raise ValueError("qpos packet contains a non-numeric field") from error
    if len(qpos) != len(names):
        raise ValueError(f"qpos length {len(qpos)} does not match {len(names)} names")
    if (
        sequence < 0
        or not math.isfinite(timestamp)
        or not all(math.isfinite(value) for value in qpos)
    ):
        raise ValueError("qpos packet has an invalid sequence, timestamp, or position")
    return QposPacket(
        stream_id=str(message["stream_id"]),
        sequence=sequence,
        timestamp=timestamp,
        side=str(message["side"]),
        model=(
            None
            if message.get("model") is None
            else str(message["model"]).strip().lower()
        ),
        joint_names=names,
        qpos=qpos,
    )


def validate_hand_state(
    positions: Sequence[float],
    *,
    command_slots: int = COMMAND_SLOTS,
    lower_bounds: Sequence[float] | None = None,
    upper_bounds: Sequence[float] | None = None,
) -> tuple[float, ...] | None:
    """Return a usable 20-slot state, or None if the message must be ignored.

    The vendor driver's very first published state carries 10 values rather than
    20, because it publishes its `[-1] * 10` initializer before the first
    hardware poll fills it in. It also uses -1 as a no-data sentinel. Neither is
    a valid measurement and neither may be allowed to seed the slew limiter.
    """
    if lower_bounds is None:
        lower_bounds = [0.0] * command_slots
    if upper_bounds is None:
        upper_bounds = [255.0] * command_slots
    if (
        len(lower_bounds) != command_slots
        or len(upper_bounds) != command_slots
        or len(positions) != command_slots
    ):
        return None
    values = []
    for value, lower, upper in zip(positions, lower_bounds, upper_bounds):
        number = float(value)
        if not math.isfinite(number) or number < lower or number > upper:
            return None
        values.append(number)
    return tuple(values)


def _unit(value: float, limits: tuple[float, float]) -> float:
    lower, upper = limits
    return min(1.0, max(0.0, (value - lower) / (upper - lower)))


def _raw(unit_value: float, *, inverted: bool) -> float:
    value = 1.0 - unit_value if inverted else unit_value
    return float(round(255.0 * min(1.0, max(0.0, value))))


class G20Mapper:
    """Project the 21-DoF L20 URDF pose into the G20's 20 command slots.

    G20 has 16 actuated slots and four reserved ones. Each non-thumb tip motor
    represents the URDF PIP/DIP pair, and the thumb tip represents MCP/IP, so
    those pairs are averaged in normalized units before quantization.
    """

    def __init__(self, abduction_invert: bool = False) -> None:
        """Configure the abduction polarity.

        `ABDUCTION_INVERTED` carries the derived per-side polarity. Whether the
        vendor's positive lateral direction agrees with this URDF's is
        the one fact neither the vendor tables nor the URDF can settle, so it is a
        single flag: set it when the operator spreads and the hand closes its
        finger gaps instead of opening them.
        """
        self.abduction_invert = bool(abduction_invert)

    def map_packet(self, packet: QposPacket) -> tuple[float, ...]:
        return self.map_qpos(packet.side, packet.joint_names, packet.qpos)

    def map_qpos(
        self,
        side: str,
        joint_names: Sequence[str],
        qpos: Sequence[float],
    ) -> tuple[float, ...]:
        if side not in {"left", "right"}:
            raise ValueError(f"side must be left or right, got {side!r}")
        if len(joint_names) != len(qpos):
            raise ValueError("joint_names and qpos lengths differ")
        values = dict(zip(joint_names, (float(value) for value in qpos)))
        finger_limits, thumb_limits = _g20_limits(side)
        # The packet contract keeps thumb_ip for the left V10.1 URDF even
        # though that URDF calls the physical follower thumb_dip. Accept both
        # names so diagnostic and legacy senders remain readable.
        thumb_tip_name = "thumb_ip" if "thumb_ip" in values else "thumb_dip"
        required = {
            "thumb_cmc_yaw",
            "thumb_cmc_roll",
            "thumb_cmc_pitch",
            "thumb_mcp",
            thumb_tip_name,
        }
        for finger in FINGERS:
            required.update(
                {
                    f"{finger}_mcp_roll",
                    f"{finger}_mcp_pitch",
                    f"{finger}_pip",
                    f"{finger}_dip",
                }
            )
        missing = sorted(required.difference(values))
        if missing:
            raise ValueError(f"qpos packet is missing required L20 joints: {missing}")

        output = [0.0] * COMMAND_SLOTS
        output[0] = _raw(
            _unit(values["thumb_cmc_pitch"], thumb_limits["thumb_cmc_pitch"]),
            inverted=True,
        )
        for slot, finger in enumerate(FINGERS, start=1):
            output[slot] = _raw(
                _unit(values[f"{finger}_mcp_pitch"], finger_limits["mcp_pitch"]),
                inverted=True,
            )

        output[5] = _raw(
            _unit(values["thumb_cmc_roll"], thumb_limits["thumb_cmc_roll"]),
            inverted=True,
        )
        # One sign for all four fingers, matching both the URDF's shared roll axis
        # and the hardware. Opposition between fingers, and so spread, is carried
        # by the roll values themselves.
        inverted = ABDUCTION_INVERTED[side]
        if self.abduction_invert:
            inverted = not inverted
        for slot, finger in enumerate(FINGERS, start=6):
            output[slot] = _raw(
                _unit(values[f"{finger}_mcp_roll"], finger_limits["mcp_roll"]),
                inverted=inverted,
            )

        output[10] = _raw(
            _unit(values["thumb_cmc_yaw"], thumb_limits["thumb_cmc_yaw"]),
            inverted=True,
        )
        output[RESERVED_SLOTS] = [0.0] * 4

        thumb_flexion = 0.5 * (
            _unit(values["thumb_mcp"], thumb_limits["thumb_mcp"])
            + _unit(values[thumb_tip_name], thumb_limits["thumb_tip"])
        )
        output[15] = _raw(thumb_flexion, inverted=True)
        for slot, finger in enumerate(FINGERS, start=16):
            distal_flexion = 0.5 * (
                _unit(values[f"{finger}_pip"], finger_limits["pip"])
                + _unit(values[f"{finger}_dip"], finger_limits["dip"])
            )
            output[slot] = _raw(distal_flexion, inverted=True)
        return tuple(output)

    def home(self, side: str) -> tuple[float, ...]:
        """An open-hand command, used to seed the limiter when state is absent."""
        names: list[str] = []
        values: list[float] = []
        finger_limits, _ = _g20_limits(side)
        for finger in ("pinky", "ring", "middle", "index"):
            for suffix, limits in finger_limits.items():
                names.append(f"{finger}_{suffix}")
                values.append(0.0 if suffix != "mcp_roll" else sum(limits) / 2.0)
        for name in ("thumb_cmc_yaw", "thumb_cmc_roll", "thumb_cmc_pitch", "thumb_mcp"):
            names.append(name)
            values.append(0.0)
        names.append("thumb_ip" if side == "left" else "thumb_dip")
        values.append(0.0)
        return self.map_qpos(side, names, values)


class CommandLimiter:
    """Limit command slew in vendor range units per second."""

    def __init__(
        self,
        initial: Sequence[float],
        max_units_per_second: float,
        *,
        lower_bounds: Sequence[float] | None = None,
        upper_bounds: Sequence[float] | None = None,
        fixed_values: dict[int, float] | None = None,
    ) -> None:
        if not initial:
            raise ValueError("initial command must not be empty")
        if max_units_per_second <= 0.0:
            raise ValueError("max_units_per_second must be positive")
        size = len(initial)
        legacy_g20_defaults = (
            lower_bounds is None
            and upper_bounds is None
            and fixed_values is None
        )
        if legacy_g20_defaults and size != COMMAND_SLOTS:
            raise ValueError(
                f"initial command must contain {COMMAND_SLOTS} values"
            )
        self.lower_bounds = tuple(
            float(value)
            for value in (
                [0.0] * size if lower_bounds is None else lower_bounds
            )
        )
        self.upper_bounds = tuple(
            float(value)
            for value in (
                [255.0] * size if upper_bounds is None else upper_bounds
            )
        )
        if len(self.lower_bounds) != size or len(self.upper_bounds) != size:
            raise ValueError("limiter bounds must match the command length")
        if any(
            lower > upper
            for lower, upper in zip(self.lower_bounds, self.upper_bounds)
        ):
            raise ValueError("limiter lower bounds must not exceed upper bounds")
        self.fixed_values = dict(
            {
                index: 0.0
                for index in range(RESERVED_SLOTS.start, RESERVED_SLOTS.stop)
            }
            if legacy_g20_defaults
            else (fixed_values or {})
        )
        if any(index < 0 or index >= size for index in self.fixed_values):
            raise ValueError("fixed command index is out of range")
        self.value = tuple(float(value) for value in initial)
        self.max_units_per_second = float(max_units_per_second)
        self.last_time: float | None = None

    def reset(self, value: Sequence[float], now: float | None = None) -> None:
        if len(value) != len(self.value):
            raise ValueError(
                f"reset command must contain {len(self.value)} values"
            )
        self.value = tuple(float(item) for item in value)
        self.last_time = now

    def step(self, target: Sequence[float], now: float) -> tuple[float, ...]:
        if len(target) != len(self.value):
            raise ValueError(
                f"target command must contain {len(self.value)} values"
            )
        if self.last_time is None:
            self.last_time = float(now)
            return self.value
        # Clamping dt bounds the step taken after a stall, so a late wake-up
        # cannot be converted into one large jump.
        dt = max(0.0, min(0.25, float(now) - self.last_time))
        self.last_time = float(now)
        maximum_delta = self.max_units_per_second * dt
        result = []
        for current, desired, lower, upper in zip(
            self.value, target, self.lower_bounds, self.upper_bounds
        ):
            delta = min(max(float(desired) - current, -maximum_delta), maximum_delta)
            result.append(min(upper, max(lower, current + delta)))
        for index, value in self.fixed_values.items():
            result[index] = value
        self.value = tuple(result)
        return self.value
