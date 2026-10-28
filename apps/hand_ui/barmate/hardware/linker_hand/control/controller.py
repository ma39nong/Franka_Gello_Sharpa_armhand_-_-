"""Manual Linker Hand controller independent of the browser UI."""

from __future__ import annotations

import threading
from collections.abc import Mapping, Sequence
from typing import Protocol, cast

from barmate.core.types import FeatureSpec, JointState, RobotAction

from barmate.hardware.linker_hand.control.types import (
    DEFAULT_CLOSED_VALUE,
    DEFAULT_STEP,
    HandSlot,
    TACTILE_FINGER_KEYS,
    TactileFrame,
    clamp_hand_value,
    format_position,
    normalize_tactile_frame,
)


class LinkerHandControlHand(Protocol):
    """Hand interface used by the manual controller and GUI."""

    def connect(self) -> None: ...

    @property
    def is_connected(self) -> bool: ...

    @property
    def action_features(self) -> FeatureSpec: ...

    @property
    def speed(self) -> tuple[int, ...]: ...

    @property
    def torque(self) -> tuple[int, ...]: ...

    @property
    def supports_torque(self) -> bool: ...

    def read_joint_state(self) -> JointState: ...

    def send_action(self, action: RobotAction) -> RobotAction: ...

    def set_speed(self, speed: Sequence[int | float]) -> tuple[int, ...]: ...

    def set_torque(self, torque: Sequence[int | float]) -> tuple[int, ...]: ...


class ManualHandController:
    """Small testable controller for manual Linker Hand actions."""

    def __init__(
        self,
        left_hand: LinkerHandControlHand | None = None,
        right_hand: LinkerHandControlHand | None = None,
        *,
        step: int = DEFAULT_STEP,
        connect_on_start: bool = True,
    ) -> None:
        slots: list[HandSlot] = []
        if left_hand is not None:
            slots.append(HandSlot("left", left_hand))
        if right_hand is not None:
            slots.append(HandSlot("right", right_hand))
        if not slots:
            raise ValueError("At least one hand instance must be provided")
        self.slots = tuple(slots)
        self.step = int(step)
        self.connect_on_start = connect_on_start
        self.selected_hand_index = 0
        self.selected_joint_index = 0
        self.positions: dict[str, list[int]] = {}
        self._lock = threading.RLock()

    @property
    def selected_slot(self) -> HandSlot:
        return self.slots[self.selected_hand_index]

    @property
    def selected_hand_name(self) -> str:
        return self.selected_slot.name

    def start(self) -> None:
        """Connect hands if requested and cache their current positions."""

        with self._lock:
            for slot in self.slots:
                hand = self._hand(slot)
                if self.connect_on_start and not hand.is_connected:
                    hand.connect()
                self.positions[slot.name] = self._read_position(hand)

    def select_next_hand(self) -> str:
        with self._lock:
            self.selected_hand_index = (self.selected_hand_index + 1) % len(self.slots)
            self._clamp_selected_joint_index()
            return self.selected_hand_name

    def select_previous_hand(self) -> str:
        with self._lock:
            self.selected_hand_index = (self.selected_hand_index - 1) % len(self.slots)
            self._clamp_selected_joint_index()
            return self.selected_hand_name

    def select_next_joint(self) -> int:
        with self._lock:
            width = len(self.positions[self.selected_hand_name])
            self.selected_joint_index = (self.selected_joint_index + 1) % width
            return self.selected_joint_index

    def select_previous_joint(self) -> int:
        with self._lock:
            width = len(self.positions[self.selected_hand_name])
            self.selected_joint_index = (self.selected_joint_index - 1) % width
            return self.selected_joint_index

    def nudge_selected_joint(self, delta: int) -> RobotAction:
        with self._lock:
            values = list(self.positions[self.selected_hand_name])
            index = self.selected_joint_index
            values[index] = clamp_hand_value(values[index] + int(delta))
            return self.set_hand_position(self.selected_hand_name, values)

    def set_selected_hand_all(self, value: int) -> RobotAction:
        with self._lock:
            width = len(self.positions[self.selected_hand_name])
            return self.set_hand_position(self.selected_hand_name, [value] * width)

    def set_hand_position(
        self, hand_name: str, values: Sequence[int | float]
    ) -> RobotAction:
        """Command one hand using the canonical ``action_joint`` feature."""

        with self._lock:
            slot = self._slot_by_name(hand_name)
            hand = self._hand(slot)
            target = [clamp_hand_value(value) for value in values]
            width = self._joint_count(hand)
            if len(target) < width:
                target.extend([DEFAULT_CLOSED_VALUE] * (width - len(target)))
            target = target[:width]
            result = hand.send_action({"action_joint": target})
            self.positions[slot.name] = target
            return result

    def apply_hand_positions(
        self,
        positions: Mapping[str, Sequence[int | float]],
    ) -> dict[str, RobotAction]:
        """Apply externally-loaded hand positions without making the GUI own playback."""

        with self._lock:
            return {
                hand_name: self.set_hand_position(hand_name, values)
                for hand_name, values in positions.items()
                if hand_name in self.hand_names
            }

    def set_hand_speed(
        self, hand_name: str, speed: Sequence[int | float]
    ) -> tuple[int, ...]:
        """Configure one hand's finger speed while the GUI is running."""

        with self._lock:
            return self._hand(self._slot_by_name(hand_name)).set_speed(speed)

    def set_hand_torque(
        self, hand_name: str, torque: Sequence[int | float]
    ) -> tuple[int, ...]:
        """Configure one hand's finger torque while the GUI is running."""

        with self._lock:
            return self._hand(self._slot_by_name(hand_name)).set_torque(torque)

    def set_selected_hand_speed(self, speed: Sequence[int | float]) -> tuple[int, ...]:
        """Configure the selected hand's finger speed."""

        return self.set_hand_speed(self.selected_hand_name, speed)

    def set_selected_hand_torque(
        self, torque: Sequence[int | float]
    ) -> tuple[int, ...]:
        """Configure the selected hand's finger torque."""

        return self.set_hand_torque(self.selected_hand_name, torque)

    @property
    def hand_names(self) -> tuple[str, ...]:
        return tuple(slot.name for slot in self.slots)

    def joint_count(self, hand_name: str) -> int:
        return self._joint_count(self._hand(self._slot_by_name(hand_name)))

    def speed(self, hand_name: str) -> tuple[int, ...]:
        return self._hand(self._slot_by_name(hand_name)).speed

    def torque(self, hand_name: str) -> tuple[int, ...]:
        return self._hand(self._slot_by_name(hand_name)).torque

    def supports_torque(self, hand_name: str) -> bool:
        hand = self._hand(self._slot_by_name(hand_name))
        return bool(getattr(hand, "supports_torque", True))

    def position(self, hand_name: str) -> list[int]:
        with self._lock:
            return list(self.positions[hand_name])

    def tactile(self, hand_name: str) -> TactileFrame:
        with self._lock:
            hand = self._hand(self._slot_by_name(hand_name))
            reader = getattr(hand, "read_tactile", None)
            if not callable(reader):
                return {}
            return normalize_tactile_frame(reader())

    def has_tactile(self, hand_name: str) -> bool:
        hand = self._hand(self._slot_by_name(hand_name))
        features = getattr(hand, "observation_features", {})
        if isinstance(features, Mapping) and any(
            key in features for key in TACTILE_FINGER_KEYS
        ):
            return True
        return bool(self.tactile(hand_name))

    def status_lines(self) -> list[str]:
        """Return display lines for the current controller state."""

        with self._lock:
            lines = [
                "Linker Hand Manual Control",
                f"Selected: {self.selected_hand_name} joint {self.selected_joint_index}",
            ]
            for slot in self.slots:
                marker = "*" if slot.name == self.selected_hand_name else " "
                values = self.positions.get(slot.name, [])
                selected = self.selected_joint_index if marker == "*" else None
                lines.append(
                    f"{marker} {slot.name}: {format_position(values, selected)}"
                )
            return lines

    def _slot_by_name(self, hand_name: str) -> HandSlot:
        for slot in self.slots:
            if slot.name == hand_name:
                return slot
        raise KeyError(f"Unknown hand name: {hand_name}")

    def _read_position(self, hand: LinkerHandControlHand) -> list[int]:
        joint_state = hand.read_joint_state()
        values = [clamp_hand_value(value) for value in joint_state.position]
        width = self._joint_count(hand)
        if len(values) < width:
            values.extend([DEFAULT_CLOSED_VALUE] * (width - len(values)))
        return values[:width]

    def _joint_count(self, hand: LinkerHandControlHand) -> int:
        action_joint = hand.action_features.get("action_joint")
        if (
            isinstance(action_joint, tuple)
            and action_joint
            and isinstance(action_joint[0], int)
        ):
            return action_joint[0]
        return len(hand.read_joint_state().position)

    def _clamp_selected_joint_index(self) -> None:
        self.selected_joint_index = min(
            self.selected_joint_index,
            len(self.positions[self.selected_hand_name]) - 1,
        )

    def _hand(self, slot: HandSlot) -> LinkerHandControlHand:
        return cast(LinkerHandControlHand, slot.hand)
