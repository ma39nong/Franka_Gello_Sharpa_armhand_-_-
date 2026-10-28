"""Preset relative-action configuration and solved trajectory sampling."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re

import numpy as np
import yaml

from .types import SIDES


@dataclass(frozen=True)
class PresetAction:
    key: str
    label: str
    side: str
    action_name: str
    path: Path
    speed_scale: float


@dataclass(frozen=True)
class SolvedRelativeAction:
    name: str
    side: str
    time_sec: tuple[float, ...]
    positions: tuple[np.ndarray, ...]
    start_q: np.ndarray
    speed_scale: float

    @classmethod
    def from_dict(cls, data: dict) -> "SolvedRelativeAction":
        side = str(data.get("side", ""))
        if side not in SIDES:
            raise ValueError(f"invalid solved action side: {side}")
        times = tuple(float(value) for value in data.get("time_sec", ()))
        positions = tuple(
            np.asarray(values, dtype=float) for values in data.get("positions", ())
        )
        start_q = np.asarray(data.get("start_q", ()), dtype=float)
        speed_scale = float(data.get("speed_scale", 0.0))
        if not 3 <= len(times) == len(positions) <= 2000:
            raise ValueError("solved action must contain 3 to 2000 frames")
        if any(
            not math.isfinite(value)
            or (index and value <= times[index - 1])
            for index, value in enumerate(times)
        ):
            raise ValueError("solved action timestamps must increase")
        if (
            start_q.shape != (14,)
            or not np.all(np.isfinite(start_q))
            or any(
                values.shape != (7,) or not np.all(np.isfinite(values))
                for values in positions
            )
        ):
            raise ValueError("solved action contains invalid joint positions")
        if not 0.05 <= speed_scale <= 1.0:
            raise ValueError("solved action speed must be 5% to 100%")
        return cls(
            name=str(data.get("name", "preset")),
            side=side,
            time_sec=times,
            positions=positions,
            start_q=start_q,
            speed_scale=speed_scale,
        )

    @property
    def duration_sec(self) -> float:
        return (self.time_sec[-1] - self.time_sec[0]) / self.speed_scale


def _safe_filename(name: str) -> str:
    value = re.sub(r"[^\w.-]+", "_", name, flags=re.UNICODE).strip("._")
    return value or "action"


def load_preset_actions(
    config_path: str | Path, data_root: str | Path
) -> dict[str, PresetAction | None]:
    """Resolve Q/W/E slots without requiring action files to exist at startup."""
    with Path(config_path).open("r", encoding="utf-8") as stream:
        root = yaml.safe_load(stream) or {}
    slots = root.get("slots")
    if not isinstance(slots, dict) or set(slots) != {"q", "w", "e"}:
        raise ValueError("preset action config must define exactly q, w, and e")
    action_root = Path(data_root) / "arm_ui" / "actions"
    result: dict[str, PresetAction | None] = {}
    for key in ("q", "w", "e"):
        entry = slots[key]
        if entry is None:
            result[key] = None
            continue
        if not isinstance(entry, dict):
            raise ValueError(f"preset slot {key} must be a mapping or null")
        side = str(entry.get("side", ""))
        name = str(entry.get("action", "")).strip()
        speed_scale = float(entry.get("speed_scale", 0.5))
        if side not in SIDES or not name:
            raise ValueError(f"preset slot {key} requires side and action")
        if any(value in name for value in ("/", "\\", "\n", "\r")):
            raise ValueError(f"preset slot {key} has an invalid action name")
        if not 0.05 <= speed_scale <= 1.0:
            raise ValueError(f"preset slot {key} speed must be 5% to 100%")
        result[key] = PresetAction(
            key=key,
            label=str(entry.get("label", name)).strip() or name,
            side=side,
            action_name=name,
            path=action_root / f"{side}__{_safe_filename(name)}.yaml",
            speed_scale=speed_scale,
        )
    return result


def sample_solved_action(solution: SolvedRelativeAction, elapsed: float) -> np.ndarray:
    """Linearly sample a fully prechecked joint trajectory."""
    action_time = solution.time_sec[0] + max(0.0, elapsed) * solution.speed_scale
    if action_time <= solution.time_sec[0]:
        return solution.positions[0].copy()
    if action_time >= solution.time_sec[-1]:
        return solution.positions[-1].copy()
    upper = next(
        index
        for index, timestamp in enumerate(solution.time_sec)
        if timestamp >= action_time
    )
    lower = upper - 1
    span = solution.time_sec[upper] - solution.time_sec[lower]
    blend = (action_time - solution.time_sec[lower]) / span
    return (
        solution.positions[lower]
        + blend * (solution.positions[upper] - solution.positions[lower])
    )
