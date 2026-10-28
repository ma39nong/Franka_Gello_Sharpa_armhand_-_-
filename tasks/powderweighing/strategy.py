"""Pure policy and fixed-pose adapter for the powder-weighing task."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import numpy as np


class PitchPhase(str, Enum):
    FREE = "FREE"
    PITCH_READY = "PITCH_READY"
    PITCH = "PITCH"


@dataclass(frozen=True)
class TriggerConfig:
    ready_enter_m: float
    ready_exit_m: float
    pitch_enter_m: float
    pitch_exit_m: float
    dwell_seconds: float
    selection_margin_m: float

    def __post_init__(self) -> None:
        values = tuple(float(value) for value in self.__dict__.values())
        if not all(np.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("trigger values must be finite and non-negative")
        if not self.pitch_enter_m < self.pitch_exit_m < self.ready_enter_m:
            raise ValueError(
                "require pitch_enter_m < pitch_exit_m < ready_enter_m"
            )
        if self.ready_exit_m <= self.ready_enter_m:
            raise ValueError("ready_exit_m must exceed ready_enter_m")
        if self.dwell_seconds <= 0.0:
            raise ValueError("dwell_seconds must be positive")


@dataclass(frozen=True)
class PowderWeighingConfig:
    side: str
    joint_names: tuple[str, ...]
    trigger: TriggerConfig
    max_joint_speed_rad_s: float
    pitch_ready_ticks: np.ndarray
    pitch_ticks: np.ndarray


def _pose_ticks(value, label: str, count: int) -> np.ndarray:
    if value is None:
        raise ValueError(
            f"{label} is not calibrated; run "
            "tasks/powderweighing/calibrate_poses.py"
        )
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (count,) or not np.isfinite(array).all():
        raise ValueError(f"{label} must contain {count} finite tick values")
    if np.any(array < 0.0) or np.any(array > 255.0):
        raise ValueError(f"{label} ticks must be within 0..255")
    return array


def load_config(path: str | Path) -> PowderWeighingConfig:
    data = json.loads(Path(path).read_text())
    if data.get("format") != "powderweighing-o30i-hardcoded-poses":
        raise ValueError("not a powderweighing O30i pose file")
    if data.get("version") != 1 or data.get("side") != "right":
        raise ValueError("only version 1 right-hand pose files are supported")
    names = tuple(str(name) for name in data["joint_names"])
    if len(names) != 20 or len(set(names)) != 20:
        raise ValueError("joint_names must contain 20 unique names")
    speed = float(data["max_joint_speed_rad_s"])
    if not np.isfinite(speed) or speed <= 0.0:
        raise ValueError("max_joint_speed_rad_s must be finite and positive")
    poses = data["poses_ticks"]
    return PowderWeighingConfig(
        side="right",
        joint_names=names,
        trigger=TriggerConfig(**data["trigger"]),
        max_joint_speed_rad_s=speed,
        pitch_ready_ticks=_pose_ticks(poses.get("pitch_ready"), "pitch_ready", 20),
        pitch_ticks=_pose_ticks(poses.get("pitch"), "pitch", 20),
    )


class PitchTrigger:
    """Hysteretic thumb-index gesture trigger with an entry dwell."""

    def __init__(self, config: TriggerConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.phase = PitchPhase.FREE
        self._candidate_since: float | None = None

    def update(
        self, now: float, index_gap_m: float, nearest_other_gap_m: float
    ) -> PitchPhase:
        cfg = self.config
        now = float(now)
        index_gap_m = float(index_gap_m)
        nearest_other_gap_m = float(nearest_other_gap_m)
        if not np.isfinite(index_gap_m) or not np.isfinite(nearest_other_gap_m):
            raise ValueError("MANUS gaps must be finite")

        if self.phase is PitchPhase.FREE:
            eligible = (
                index_gap_m <= cfg.ready_enter_m
                and nearest_other_gap_m
                >= index_gap_m + cfg.selection_margin_m
            )
            if eligible:
                if self._candidate_since is None:
                    self._candidate_since = now
                if now - self._candidate_since >= cfg.dwell_seconds:
                    self.phase = PitchPhase.PITCH_READY
            else:
                self._candidate_since = None
        elif self.phase is PitchPhase.PITCH_READY:
            if index_gap_m >= cfg.ready_exit_m:
                self.reset()
            elif index_gap_m <= cfg.pitch_enter_m:
                self.phase = PitchPhase.PITCH
        else:
            if index_gap_m >= cfg.ready_exit_m:
                self.reset()
            elif index_gap_m >= cfg.pitch_exit_m:
                self.phase = PitchPhase.PITCH_READY
        return self.phase


def manus_gaps(raw_keypoints: np.ndarray) -> tuple[float, float]:
    frame = np.asarray(raw_keypoints, dtype=np.float64)
    if frame.shape != (25, 7) or not np.isfinite(frame).all():
        raise ValueError("expected a finite raw 25x7 MANUS frame")
    points = frame[:, :3]
    thumb = points[4]
    gaps = np.array([
        np.linalg.norm(points[index] - thumb) for index in (9, 14, 19, 24)
    ])
    return float(gaps[0]), float(np.min(gaps[1:]))


class HardcodedPitchRetargeter:
    """Wrap the original retargeter and replace two gesture ranges by poses."""

    wants_raw_keypoints = True

    def __init__(self, base, config: PowderWeighingConfig) -> None:
        if set(base.joint_names) != set(config.joint_names):
            raise ValueError(
                "pose joint names do not match the right O30i retargeter"
            )
        self.base = base
        self.config = config
        self.joint_names = list(base.joint_names)
        self.lower = np.asarray(base.lower, dtype=np.float64)
        self.upper = np.asarray(base.upper, dtype=np.float64)
        span = self.upper - self.lower
        # Calibration is intentionally printed/stored in the driver's canonical
        # URDF order.  Sharpa's packet order comes from Pinocchio and differs
        # (index, middle, pinky, ring, thumb).  Names are the contract: reorder
        # before applying each packet slot's limits, never trust array position.
        ready_by_name = dict(zip(
            config.joint_names, config.pitch_ready_ticks, strict=True
        ))
        pitch_by_name = dict(zip(
            config.joint_names, config.pitch_ticks, strict=True
        ))
        self.pitch_ready_ticks = np.array(
            [ready_by_name[name] for name in self.joint_names]
        )
        self.pitch_ticks = np.array(
            [pitch_by_name[name] for name in self.joint_names]
        )
        self.pitch_ready = self.lower + self.pitch_ready_ticks / 255.0 * span
        self.pitch = self.lower + self.pitch_ticks / 255.0 * span
        self.trigger = PitchTrigger(config.trigger)
        self.last_qpos = np.asarray(base.last_qpos, dtype=np.float64).copy()
        self._last_time: float | None = None
        self._last_gap_m = float("nan")

    @property
    def phase(self) -> PitchPhase:
        return self.trigger.phase

    def retarget(self, keypoints) -> tuple[np.ndarray, dict]:
        frame = np.asarray(keypoints, dtype=np.float64)
        base_q, stats = self.base.retarget(frame)
        now = time.monotonic()
        index_gap, other_gap = manus_gaps(frame)
        phase = self.trigger.update(now, index_gap, other_gap)
        desired = {
            PitchPhase.FREE: np.asarray(base_q, dtype=np.float64),
            PitchPhase.PITCH_READY: self.pitch_ready,
            PitchPhase.PITCH: self.pitch,
        }[phase]

        if self._last_time is None:
            output = np.asarray(base_q, dtype=np.float64).copy()
            dt = 0.0
        else:
            dt = max(0.0, now - self._last_time)
            step = self.config.max_joint_speed_rad_s * dt
            output = self.last_qpos + np.clip(desired - self.last_qpos, -step, step)
        output = np.clip(output, self.lower, self.upper)
        self.last_qpos = output.copy()
        self._last_time = now
        self._last_gap_m = index_gap
        result_stats = dict(stats)
        result_stats["powderweighing"] = {
            "phase": phase.value,
            "manus_thumb_index_gap_mm": index_gap * 1000.0,
            "nearest_other_gap_mm": other_gap * 1000.0,
            "max_joint_step_rad": self.config.max_joint_speed_rad_s * dt,
        }
        return output, result_stats

    def reset(self) -> None:
        self.base.reset()
        self.trigger.reset()
        self.last_qpos = np.asarray(self.base.last_qpos, dtype=np.float64).copy()
        self._last_time = None
        self._last_gap_m = float("nan")

    def close(self) -> None:
        self.base.close()


def install_on_manus_pipeline(
    pipeline, config: PowderWeighingConfig
) -> HardcodedPitchRetargeter:
    """Install this task on only the right side of a bimanual MANUS pipeline."""
    if getattr(pipeline, "hands", {}).get("right") != "o30i":
        raise ValueError("powderweighing requires a right O30i hand")
    if getattr(pipeline, "methods", {}).get("right") != "sharpa":
        raise ValueError("powderweighing FREE mode requires right-hand Sharpa")
    if "right" not in getattr(pipeline, "retargeters", {}):
        raise ValueError("powderweighing requires an active right MANUS side")
    wrapped = HardcodedPitchRetargeter(pipeline.retargeters["right"], config)
    pipeline.retargeters["right"] = wrapped
    return wrapped
