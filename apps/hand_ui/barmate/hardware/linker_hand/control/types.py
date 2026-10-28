"""Shared types and formatting helpers for the Linker Hand control app."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

DEFAULT_STEP = 5
DEFAULT_HOME_VALUE = 128
DEFAULT_OPEN_VALUE = 255
DEFAULT_CLOSED_VALUE = 0
TACTILE_FINGER_KEYS = (
    "thumb_matrix",
    "index_matrix",
    "middle_matrix",
    "ring_matrix",
    "little_matrix",
)
TACTILE_DISPLAY_NAMES = {
    "thumb_matrix": "THUMB",
    "index_matrix": "INDEX",
    "middle_matrix": "MIDDLE",
    "ring_matrix": "RING",
    "little_matrix": "PINKY",
}
TactileMatrix = tuple[tuple[float, ...], ...]
TactileFrame = dict[str, TactileMatrix]


@dataclass(frozen=True, slots=True)
class HandSlot:
    """Named hand instance controlled by the GUI."""

    name: str
    hand: object


@dataclass(frozen=True, slots=True)
class HandPlaybackFrame:
    """One timestamped dual-hand playback frame."""

    timestamp: float
    left: tuple[int, ...]
    right: tuple[int, ...]


def clamp_hand_value(value: object) -> int:
    return max(DEFAULT_CLOSED_VALUE, min(DEFAULT_OPEN_VALUE, int(round(float(value)))))


def first_or_default(values: Sequence[int], default: int = DEFAULT_HOME_VALUE) -> int:
    return int(values[0]) if values else default


def format_position(values: Sequence[int], selected_index: int | None) -> str:
    formatted = []
    for index, value in enumerate(values):
        text = str(value)
        if index == selected_index:
            text = f"[{text}]"
        formatted.append(text)
    return " ".join(formatted)


def normalize_tactile_frame(observation: object) -> TactileFrame:
    if not isinstance(observation, dict):
        return {}
    frame: TactileFrame = {}
    for key in TACTILE_FINGER_KEYS:
        matrix = _normalize_tactile_matrix(observation.get(key))
        if matrix:
            frame[key] = matrix
    return frame


def tactile_frame_peak(frame: TactileFrame) -> float:
    peak = 0.0
    for matrix in frame.values():
        for row in matrix:
            for value in row:
                peak = max(peak, value)
    return peak


def tactile_frame_total(frame: TactileFrame) -> float:
    total = 0.0
    for matrix in frame.values():
        for row in matrix:
            total += sum(max(0.0, value) for value in row)
    return total


def _normalize_tactile_matrix(value: object) -> TactileMatrix:
    if hasattr(value, "tolist"):
        value = value.tolist()
    if not isinstance(value, Sequence) or isinstance(value, str | bytes):
        return ()
    rows: list[tuple[float, ...]] = []
    for raw_row in value:
        row_value: Any = raw_row
        if hasattr(row_value, "tolist"):
            row_value = row_value.tolist()
        if not isinstance(row_value, Sequence) or isinstance(row_value, str | bytes):
            continue
        row: list[float] = []
        for item in row_value:
            try:
                row.append(max(0.0, float(item)))
            except (TypeError, ValueError):
                row.append(0.0)
        if row:
            rows.append(tuple(row))
    return tuple(rows)
