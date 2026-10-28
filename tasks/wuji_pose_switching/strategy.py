"""Two-stage fixed-pose policy driven by saved Wuji UI MANUS gestures."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
from pathlib import Path
import time
from typing import Callable

import numpy as np

from apps.wuji_ui.models import ManusTrigger, WujiPoseRepository


TIP_INDICES = {"index": 8, "middle": 12, "ring": 16, "pinky": 20}


class PosePhase(str, Enum):
    FREE = "FREE"
    READY = "READY"
    ACTIVE = "ACTIVE"


@dataclass(frozen=True)
class WujiPoseSwitchingConfig:
    side: str
    pose_library: Path
    ready_trigger: str
    active_trigger: str
    max_joint_speed_rad_s: float


def load_config(path: str | Path) -> WujiPoseSwitchingConfig:
    source = Path(path).resolve()
    data = json.loads(source.read_text(encoding="utf-8"))
    if data.get("format") != "wuji-two-stage-manus-policy":
        raise ValueError("not a Wuji two-stage MANUS policy file")
    if data.get("version") != 1:
        raise ValueError("only Wuji two-stage MANUS policy version 1 is supported")
    side = str(data["side"]).strip().lower()
    if side not in {"left", "right"}:
        raise ValueError("policy side must be left or right")
    speed = float(data["max_joint_speed_rad_s"])
    if not np.isfinite(speed) or speed <= 0.0:
        raise ValueError("max_joint_speed_rad_s must be finite and positive")
    pose_library = Path(data["pose_library"])
    if not pose_library.is_absolute():
        pose_library = (source.parent / pose_library).resolve()
    return WujiPoseSwitchingConfig(
        side=side,
        pose_library=pose_library,
        ready_trigger=str(data["ready_trigger"]).strip(),
        active_trigger=str(data["active_trigger"]).strip(),
        max_joint_speed_rad_s=speed,
    )


def _thumb_tip_gap_m(landmarks: np.ndarray, finger: str) -> float:
    points = np.asarray(landmarks, dtype=np.float64)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        raise ValueError("expected finite 21x3 MANUS landmarks")
    try:
        tip = TIP_INDICES[finger]
    except KeyError as error:
        raise ValueError(f"unknown MANUS target finger: {finger}") from error
    return float(np.linalg.norm(points[4] - points[tip]))


class TwoStageGestureTrigger:
    """FREE -> READY -> ACTIVE state machine with independent hysteresis."""

    def __init__(self, ready: ManusTrigger, active: ManusTrigger) -> None:
        if ready.side != active.side:
            raise ValueError("ready and active triggers must use the same hand side")
        if ready.name == active.name:
            raise ValueError("ready and active triggers must be distinct")
        if ready.finger == active.finger and not (
            active.enter_max_m < active.exit_min_m < ready.enter_max_m < ready.exit_min_m
        ):
            raise ValueError(
                "same-finger stages require "
                "active_enter < active_exit < ready_enter < ready_exit"
            )
        self.ready = ready
        self.active = active
        self.reset()

    def reset(self) -> None:
        self.phase = PosePhase.FREE
        self._ready_candidate_since: float | None = None
        self._active_candidate_since: float | None = None

    def _go_free(self) -> None:
        self.phase = PosePhase.FREE
        self._ready_candidate_since = None
        self._active_candidate_since = None

    def update(
        self,
        now: float,
        ready_gap_m: float,
        active_gap_m: float,
    ) -> PosePhase:
        moment = float(now)
        ready_gap = float(ready_gap_m)
        active_gap = float(active_gap_m)
        if not all(np.isfinite(value) for value in (moment, ready_gap, active_gap)):
            raise ValueError("MANUS gesture values must be finite")
        if ready_gap < 0.0 or active_gap < 0.0:
            raise ValueError("MANUS gesture values are outside their valid range")

        if self.phase is PosePhase.FREE:
            self._active_candidate_since = None
            if ready_gap <= self.ready.enter_max_m:
                if self._ready_candidate_since is None:
                    self._ready_candidate_since = moment
                if moment - self._ready_candidate_since >= self.ready.dwell_seconds:
                    self.phase = PosePhase.READY
                    self._ready_candidate_since = None
            else:
                self._ready_candidate_since = None
        elif ready_gap >= self.ready.exit_min_m:
            self._go_free()
        elif self.phase is PosePhase.READY:
            if active_gap <= self.active.enter_max_m:
                if self._active_candidate_since is None:
                    self._active_candidate_since = moment
                if moment - self._active_candidate_since >= self.active.dwell_seconds:
                    self.phase = PosePhase.ACTIVE
                    self._active_candidate_since = None
            else:
                self._active_candidate_since = None
        elif active_gap >= self.active.exit_min_m:
            self.phase = PosePhase.READY
            self._active_candidate_since = None
        return self.phase


class TwoStagePoseRetargeter:
    """Wrap a Wuji retargeter and slew between live and two saved poses."""

    def __init__(
        self,
        base,
        *,
        ready: ManusTrigger,
        active: ManusTrigger,
        ready_pose_device,
        active_pose_device,
        device_joint_names,
        max_joint_speed_rad_s: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.base = base
        self.optimizer = base.optimizer
        self.clock = clock
        self.max_joint_speed_rad_s = float(max_joint_speed_rad_s)
        if not np.isfinite(self.max_joint_speed_rad_s) or (
            self.max_joint_speed_rad_s <= 0.0
        ):
            raise ValueError("max_joint_speed_rad_s must be finite and positive")

        optimizer_names = tuple(self.optimizer.robot.dof_joint_names)
        device_names = tuple(str(name) for name in device_joint_names)
        if (
            len(optimizer_names) != 20
            or len(device_names) != 20
            or set(optimizer_names) != set(device_names)
        ):
            raise ValueError("pose and Wuji retargeter joint names do not match")
        ready_by_name = dict(
            zip(device_names, np.asarray(ready_pose_device, dtype=float), strict=True)
        )
        active_by_name = dict(
            zip(device_names, np.asarray(active_pose_device, dtype=float), strict=True)
        )
        self.ready_pose = np.asarray(
            [ready_by_name[name] for name in optimizer_names], dtype=np.float64
        )
        self.active_pose = np.asarray(
            [active_by_name[name] for name in optimizer_names], dtype=np.float64
        )
        if (
            self.ready_pose.shape != (20,)
            or self.active_pose.shape != (20,)
            or not np.isfinite(self.ready_pose).all()
            or not np.isfinite(self.active_pose).all()
        ):
            raise ValueError("saved Wuji poses must contain 20 finite joint angles")

        robot_model = self.optimizer.robot.model
        self.lower = np.asarray(
            robot_model.lowerPositionLimit, dtype=np.float64
        ).copy()
        self.upper = np.asarray(
            robot_model.upperPositionLimit, dtype=np.float64
        ).copy()
        self.trigger = TwoStageGestureTrigger(ready, active)
        self.ready_finger = ready.finger
        self.active_finger = active.finger
        self.last_qpos: np.ndarray | None = None
        self._last_time: float | None = None
        self.last_ready_gap_m = float("nan")
        self.last_active_gap_m = float("nan")

    @property
    def phase(self) -> PosePhase:
        return self.trigger.phase

    def retarget(self, landmarks) -> np.ndarray:
        points = np.asarray(landmarks, dtype=np.float64)
        base_qpos = np.asarray(self.base.retarget(points), dtype=np.float64)
        now = float(self.clock())
        ready_gap = _thumb_tip_gap_m(points, self.ready_finger)
        active_gap = _thumb_tip_gap_m(points, self.active_finger)
        phase = self.trigger.update(now, ready_gap, active_gap)
        desired = {
            PosePhase.FREE: base_qpos,
            PosePhase.READY: self.ready_pose,
            PosePhase.ACTIVE: self.active_pose,
        }[phase]

        if self.last_qpos is None or self._last_time is None:
            output = base_qpos.copy()
        else:
            dt = max(0.0, min(now - self._last_time, 0.1))
            step = self.max_joint_speed_rad_s * dt
            output = self.last_qpos + np.clip(
                desired - self.last_qpos, -step, step
            )
        output = np.clip(output, self.lower, self.upper)
        self.last_qpos = output.copy()
        self._last_time = now
        self.last_ready_gap_m = ready_gap
        self.last_active_gap_m = active_gap
        return output

    def reset(self) -> None:
        self.base.reset()
        self.trigger.reset()
        self.last_qpos = None
        self._last_time = None
        self.last_ready_gap_m = float("nan")
        self.last_active_gap_m = float("nan")


def install_on_wuji_pipeline(
    pipeline,
    config: WujiPoseSwitchingConfig,
) -> TwoStagePoseRetargeter:
    """Install the configured policy on one Wuji Hand 2 pipeline side."""
    side = config.side
    if side not in getattr(pipeline, "retargeters", {}):
        raise ValueError(f"Wuji strategy side is not active: {side}")
    if getattr(pipeline, "models", {}).get(side) != "wuji_hand_2":
        raise ValueError("saved pose switching requires Wuji Hand 2")

    raw = json.loads(config.pose_library.read_text(encoding="utf-8"))
    stored_names = raw.get("joint_names")
    if not isinstance(stored_names, dict):
        raise ValueError("Wuji pose library is missing joint_names")
    repository = WujiPoseRepository(config.pose_library, stored_names)
    runtime_names = tuple(pipeline.joint_names[side])
    if tuple(repository.joint_names[side]) != runtime_names:
        raise ValueError(f"{side} pose joint order does not match the Wuji device")

    by_name = {trigger.name: trigger for trigger in repository.triggers(side)}
    try:
        ready = by_name[config.ready_trigger]
    except KeyError as error:
        raise ValueError(
            f"MANUS distance trigger does not exist: {config.ready_trigger}"
        ) from error
    try:
        active = by_name[config.active_trigger]
    except KeyError as error:
        raise ValueError(
            f"MANUS distance trigger does not exist: {config.active_trigger}"
        ) from error

    wrapped = TwoStagePoseRetargeter(
        pipeline.retargeters[side],
        ready=ready,
        active=active,
        ready_pose_device=repository.pose(side, ready.pose_name).qpos,
        active_pose_device=repository.pose(side, active.pose_name).qpos,
        device_joint_names=runtime_names,
        max_joint_speed_rad_s=config.max_joint_speed_rad_s,
    )
    pipeline.retargeters[side] = wrapped
    return wrapped
