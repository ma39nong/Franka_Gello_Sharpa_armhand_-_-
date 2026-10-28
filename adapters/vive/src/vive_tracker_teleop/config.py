from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


SIDES = ("left", "right")


def _exact_mapping(value, fields: set[str], label: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    actual = set(value)
    if actual != fields:
        raise ValueError(
            f"{label} fields differ: missing={sorted(fields - actual)}, "
            f"unknown={sorted(actual - fields)}"
        )
    return value


def _positive(value, label: str) -> float:
    result = float(value)
    if not np.isfinite(result) or result <= 0.0:
        raise ValueError(f"{label} must be a positive finite number")
    return result


def _rotation(value, label: str) -> np.ndarray:
    rotation = np.asarray(value, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError(f"{label} must be a finite 3x3 matrix")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError(f"{label} must be orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6):
        raise ValueError(f"{label} determinant must be +1")
    return rotation.copy()


@dataclass(frozen=True)
class LocalTransformConfig:
    translation: np.ndarray
    rotation: np.ndarray


@dataclass(frozen=True)
class ViveTrackerConfig:
    serials: dict[str, str]
    world_to_control_rotation: np.ndarray
    tracker_to_control: dict[str, LocalTransformConfig]
    ready_timeout: float
    frozen_timeout: float
    max_position_jump: float
    max_rotation_jump: float
    max_linear_speed: float
    max_angular_speed: float


def _local_transform(raw, label: str) -> LocalTransformConfig:
    value = _exact_mapping(
        raw, {"translation_xyz", "quaternion_xyzw"}, label
    )
    translation = np.asarray(value["translation_xyz"], dtype=float)
    quaternion = np.asarray(value["quaternion_xyzw"], dtype=float)
    if translation.shape != (3,) or not np.all(np.isfinite(translation)):
        raise ValueError(f"{label}.translation_xyz must be a finite 3-vector")
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError(f"{label}.quaternion_xyzw must be a finite 4-vector")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-8:
        raise ValueError(f"{label}.quaternion_xyzw must be non-zero")
    x, y, z, w = quaternion / norm
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )
    return LocalTransformConfig(translation.copy(), rotation)


def load_vive_config(path) -> ViveTrackerConfig:
    config_path = Path(path)
    with config_path.open(encoding="utf-8") as stream:
        root = yaml.safe_load(stream)
    root = _exact_mapping(
        root,
        {
            "serials",
            "world_to_control_rotation",
            "tracker_to_control",
            "ready_timeout",
            "frozen_timeout",
            "max_position_jump",
            "max_rotation_jump",
            "max_linear_speed",
            "max_angular_speed",
        },
        str(config_path),
    )
    serials = _exact_mapping(root["serials"], set(SIDES), "serials")
    serials = {side: str(serials[side]).strip() for side in SIDES}
    if any(not serial for serial in serials.values()):
        raise ValueError("Both VIVE Tracker serial numbers must be non-empty")
    if serials["left"] == serials["right"]:
        raise ValueError("VIVE Tracker serial numbers must differ")
    transforms = _exact_mapping(
        root["tracker_to_control"], set(SIDES), "tracker_to_control"
    )
    return ViveTrackerConfig(
        serials=serials,
        world_to_control_rotation=_rotation(
            root["world_to_control_rotation"], "world_to_control_rotation"
        ),
        tracker_to_control={
            side: _local_transform(
                transforms[side], f"tracker_to_control.{side}"
            )
            for side in SIDES
        },
        ready_timeout=_positive(root["ready_timeout"], "ready_timeout"),
        frozen_timeout=_positive(root["frozen_timeout"], "frozen_timeout"),
        max_position_jump=_positive(
            root["max_position_jump"], "max_position_jump"
        ),
        max_rotation_jump=_positive(
            root["max_rotation_jump"], "max_rotation_jump"
        ),
        max_linear_speed=_positive(
            root["max_linear_speed"], "max_linear_speed"
        ),
        max_angular_speed=_positive(
            root["max_angular_speed"], "max_angular_speed"
        ),
    )
