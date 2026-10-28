"""Validated, atomically persisted Wuji Hand 2 pose library."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import tempfile
import threading
from typing import Mapping, Sequence

import numpy as np


SIDES = ("left", "right")
FORMAT = "wuji-hand-2-hardcoded-poses"
VERSION = 1
FINGERS = ("index", "middle", "ring", "pinky")
GESTURE_FEATURES = (
    "thumb_curl_rad",
    "index_curl_rad",
    "middle_curl_rad",
    "ring_curl_rad",
    "pinky_curl_rad",
    "thumb_index_gap_palm",
    "thumb_middle_gap_palm",
    "thumb_ring_gap_palm",
    "thumb_pinky_gap_palm",
    "index_middle_gap_palm",
    "middle_ring_gap_palm",
    "ring_pinky_gap_palm",
)


def _side(value: str) -> str:
    normalized = str(value).strip().lower()
    if normalized not in SIDES:
        raise ValueError(f"未知手侧: {value}")
    return normalized


def _name(value: object) -> str:
    name = str(value or "").strip()
    if not name:
        raise ValueError("姿态名称不能为空")
    if len(name) > 80:
        raise ValueError("姿态名称不能超过80个字符")
    return name


def unused_pose_name(existing: Sequence[str], base: object) -> str:
    """Return base, or base_2 / base_3 / ... if that name is already taken."""
    seed = _name(base)
    names = {str(name) for name in existing}
    if seed not in names:
        return seed
    index = 2
    while True:
        candidate = f"{seed}_{index}"
        if len(candidate) > 80:
            raise ValueError("无法为现有姿态生成不超过80个字符的副本名称")
        if candidate not in names:
            return candidate
        index += 1


@dataclass(frozen=True)
class HandPose:
    name: str
    side: str
    qpos: tuple[float, ...]

    @classmethod
    def create(cls, name: object, side: str, qpos: Sequence[float]) -> "HandPose":
        values = tuple(float(value) for value in qpos)
        if len(values) != 20 or not all(math.isfinite(value) for value in values):
            raise ValueError("Wuji Hand 2姿态必须包含20个有限关节角")
        return cls(_name(name), _side(side), values)


@dataclass(frozen=True)
class ManusTrigger:
    name: str
    side: str
    finger: str
    pose_name: str
    enter_max_m: float
    exit_min_m: float
    dwell_seconds: float
    calibration: Mapping[str, float | int]

    @classmethod
    def create(
        cls,
        name: object,
        side: str,
        finger: str,
        pose_name: object,
        *,
        enter_max_m: float,
        exit_min_m: float,
        dwell_seconds: float = 0.15,
        calibration: Mapping[str, float | int] | None = None,
    ) -> "ManusTrigger":
        normalized_finger = str(finger).strip().lower()
        if normalized_finger not in FINGERS:
            raise ValueError(f"未知MANUS目标手指: {finger}")
        enter = float(enter_max_m)
        exit_value = float(exit_min_m)
        dwell = float(dwell_seconds)
        if not all(math.isfinite(value) for value in (enter, exit_value, dwell)):
            raise ValueError("MANUS阈值必须为有限数值")
        if enter <= 0.0 or exit_value <= enter:
            raise ValueError("退出阈值必须大于正的进入阈值")
        if dwell <= 0.0:
            raise ValueError("触发停留时间必须为正数")
        stats = dict(calibration or {})
        if any(not math.isfinite(float(value)) for value in stats.values()):
            raise ValueError("MANUS采样统计必须为有限数值")
        return cls(
            _name(name),
            _side(side),
            normalized_finger,
            _name(pose_name),
            enter,
            exit_value,
            dwell,
            stats,
        )


@dataclass(frozen=True)
class ManusGestureTrigger:
    name: str
    side: str
    pose_name: str
    feature_ranges: Mapping[str, Mapping[str, float]]
    enter_match_fraction: float
    exit_match_fraction: float
    dwell_seconds: float
    sample_count: int

    @classmethod
    def create(
        cls,
        name: object,
        side: str,
        pose_name: object,
        feature_ranges: Mapping[str, Mapping[str, float]],
        *,
        enter_match_fraction: float = 0.80,
        exit_match_fraction: float = 0.60,
        dwell_seconds: float = 0.15,
        sample_count: int = 0,
    ) -> "ManusGestureTrigger":
        enter = float(enter_match_fraction)
        exit_value = float(exit_match_fraction)
        dwell = float(dwell_seconds)
        count = int(sample_count)
        if not 0.0 <= exit_value < enter <= 1.0:
            raise ValueError("复合手势要求0≤退出命中率<进入命中率≤1")
        if not math.isfinite(dwell) or dwell <= 0.0:
            raise ValueError("复合手势停留时间必须为正数")
        if count < 0:
            raise ValueError("复合手势样本数不能为负数")
        if set(feature_ranges) != set(GESTURE_FEATURES):
            raise ValueError("复合手势特征集合不完整")
        ranges: dict[str, dict[str, float]] = {}
        for feature in GESTURE_FEATURES:
            payload = feature_ranges[feature]
            low = float(payload["min"])
            high = float(payload["max"])
            median = float(payload.get("median", (low + high) / 2.0))
            if not all(math.isfinite(value) for value in (low, high, median)):
                raise ValueError(f"复合手势特征{feature}范围必须为有限数值")
            if low > median or median > high:
                raise ValueError(f"复合手势特征{feature}要求min≤median≤max")
            ranges[feature] = {"min": low, "max": high, "median": median}
        return cls(
            _name(name),
            _side(side),
            _name(pose_name),
            ranges,
            enter,
            exit_value,
            dwell,
            count,
        )


class GestureRangeGate:
    """Dwell and hysteresis gate for a composite gesture match fraction."""

    def __init__(
        self,
        *,
        enter_match_fraction: float,
        exit_match_fraction: float,
        dwell_seconds: float,
    ) -> None:
        self.enter = float(enter_match_fraction)
        self.exit = float(exit_match_fraction)
        self.dwell = float(dwell_seconds)
        if not 0.0 <= self.exit < self.enter <= 1.0:
            raise ValueError("复合手势要求0≤退出命中率<进入命中率≤1")
        if not math.isfinite(self.dwell) or self.dwell <= 0.0:
            raise ValueError("复合手势停留时间必须为正数")
        self.reset()

    def reset(self) -> None:
        self.active = False
        self.candidate_since: float | None = None

    @property
    def phase(self) -> str:
        if self.active:
            return "active"
        return "candidate" if self.candidate_since is not None else "inactive"

    def update(self, now: float, match_fraction: float) -> bool:
        moment = float(now)
        fraction = float(match_fraction)
        if not math.isfinite(moment) or not math.isfinite(fraction):
            raise ValueError("复合手势时间和命中率必须为有限数值")
        if not 0.0 <= fraction <= 1.0:
            raise ValueError("复合手势命中率必须在0到1之间")
        if self.active:
            if fraction < self.exit:
                self.reset()
            return self.active
        if fraction >= self.enter:
            if self.candidate_since is None:
                self.candidate_since = moment
            if moment - self.candidate_since >= self.dwell:
                self.active = True
        else:
            self.candidate_since = None
        return self.active


def _chain_curl(points: np.ndarray, indices: Sequence[int]) -> float:
    chain = points[np.asarray(indices, dtype=int)]
    segments = np.diff(chain, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    if np.any(lengths < 1e-6):
        raise ValueError("MANUS手指骨段长度过小")
    unit = segments / lengths[:, None]
    cosines = np.clip(np.sum(unit[:-1] * unit[1:], axis=1), -1.0, 1.0)
    return float(np.arccos(cosines).sum())


def manus_gesture_features(landmarks: np.ndarray) -> dict[str, float]:
    """Orientation-invariant hand-shape features from canonical 21 landmarks."""
    points = np.asarray(landmarks, dtype=np.float64)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        raise ValueError("复合手势需要有限的21x3 MANUS landmarks")
    palm_width = float(np.linalg.norm(points[5] - points[17]))
    if palm_width < 0.02:
        raise ValueError("MANUS掌宽无效")
    tips = {"thumb": 4, "index": 8, "middle": 12, "ring": 16, "pinky": 20}
    chains = {
        "thumb": (1, 2, 3, 4),
        "index": (5, 6, 7, 8),
        "middle": (9, 10, 11, 12),
        "ring": (13, 14, 15, 16),
        "pinky": (17, 18, 19, 20),
    }
    features = {
        f"{finger}_curl_rad": _chain_curl(points, chain)
        for finger, chain in chains.items()
    }
    for finger in FINGERS:
        features[f"thumb_{finger}_gap_palm"] = float(
            np.linalg.norm(points[tips["thumb"]] - points[tips[finger]])
            / palm_width
        )
    for first, second in (
        ("index", "middle"),
        ("middle", "ring"),
        ("ring", "pinky"),
    ):
        features[f"{first}_{second}_gap_palm"] = float(
            np.linalg.norm(points[tips[first]] - points[tips[second]])
            / palm_width
        )
    if set(features) != set(GESTURE_FEATURES) or not all(
        math.isfinite(value) for value in features.values()
    ):
        raise ValueError("MANUS复合手势特征无效")
    return features


def summarize_gesture_samples(
    samples: Sequence[Mapping[str, float]],
) -> dict[str, object]:
    if len(samples) < 30:
        raise ValueError("复合手势至少需要30个有效MANUS帧")
    matrix = np.asarray(
        [[float(sample[name]) for name in GESTURE_FEATURES] for sample in samples],
        dtype=np.float64,
    )
    if matrix.shape != (len(samples), len(GESTURE_FEATURES)) or not np.isfinite(
        matrix
    ).all():
        raise ValueError("复合手势样本特征无效")
    p05, median, p95 = np.percentile(matrix, [5.0, 50.0, 95.0], axis=0)
    feature_ranges: dict[str, dict[str, float]] = {}
    for index, name in enumerate(GESTURE_FEATURES):
        span = float(p95[index] - p05[index])
        absolute_margin = 0.05 if name.endswith("_rad") else 0.03
        margin = max(absolute_margin, 0.15 * span)
        feature_ranges[name] = {
            "min": max(0.0, float(p05[index]) - margin),
            "median": float(median[index]),
            "max": float(p95[index]) + margin,
        }
    return {"sample_count": len(samples), "feature_ranges": feature_ranges}


def gesture_match_fraction(
    features: Mapping[str, float],
    feature_ranges: Mapping[str, Mapping[str, float]],
) -> tuple[float, dict[str, bool]]:
    if set(features) != set(GESTURE_FEATURES):
        raise ValueError("实时复合手势特征集合不完整")
    matches = {
        name: (
            float(feature_ranges[name]["min"])
            <= float(features[name])
            <= float(feature_ranges[name]["max"])
        )
        for name in GESTURE_FEATURES
    }
    return sum(matches.values()) / len(matches), matches


def summarize_gap_samples(samples_m: Sequence[float]) -> dict[str, float | int]:
    values = np.asarray(samples_m, dtype=np.float64)
    if values.ndim != 1 or values.size < 10 or not np.isfinite(values).all():
        raise ValueError("至少需要10个有效MANUS距离样本")
    if np.any(values < 0.0):
        raise ValueError("MANUS距离样本不能为负数")
    p05, median, p95 = np.percentile(values, [5.0, 50.0, 95.0])
    margin = max(0.002, 0.1 * float(p95 - p05))
    enter = float(p95) + margin
    exit_value = enter + max(0.005, 0.2 * enter)
    return {
        "sample_count": int(values.size),
        "min_m": float(values.min()),
        "p05_m": float(p05),
        "median_m": float(median),
        "p95_m": float(p95),
        "max_m": float(values.max()),
        "recommended_enter_max_m": enter,
        "recommended_exit_min_m": exit_value,
    }


class WujiPoseRepository:
    """Thread-safe left/right pose library with an explicit joint contract."""

    def __init__(
        self, path: str | Path, joint_names: Mapping[str, Sequence[str]]
    ) -> None:
        self.path = Path(path)
        self.joint_names = {
            side: tuple(str(name) for name in joint_names[side]) for side in SIDES
        }
        for side, names in self.joint_names.items():
            if len(names) != 20 or len(set(names)) != 20:
                raise ValueError(f"{side} joint_names必须包含20个唯一名称")
        self._lock = threading.RLock()
        self._poses: dict[str, dict[str, HandPose]] = {side: {} for side in SIDES}
        self._triggers: dict[str, dict[str, ManusTrigger]] = {
            side: {} for side in SIDES
        }
        self._gesture_triggers: dict[str, dict[str, ManusGestureTrigger]] = {
            side: {} for side in SIDES
        }
        self.reload()

    def reload(self) -> None:
        with self._lock:
            self._poses = {side: {} for side in SIDES}
            self._triggers = {side: {} for side in SIDES}
            self._gesture_triggers = {side: {} for side in SIDES}
            if not self.path.exists():
                return
            data = json.loads(self.path.read_text(encoding="utf-8"))
            if data.get("format") != FORMAT or data.get("version") != VERSION:
                raise ValueError(f"不支持的Wuji姿态文件: {self.path}")
            if data.get("units") != "rad" or data.get("joint_order") != "device":
                raise ValueError("Wuji姿态文件必须使用device顺序和rad单位")
            stored_names = data.get("joint_names", {})
            for side in SIDES:
                if tuple(stored_names.get(side, ())) != self.joint_names[side]:
                    raise ValueError(f"{side}关节顺序与当前Wuji模型不一致")
                entries = data.get("poses", {}).get(side, {})
                if not isinstance(entries, dict):
                    raise ValueError(f"poses.{side}必须是对象")
                for name, qpos in entries.items():
                    pose = HandPose.create(name, side, qpos)
                    self._poses[side][pose.name] = pose
                triggers = data.get("manus_triggers", {}).get(side, {})
                if not isinstance(triggers, dict):
                    raise ValueError(f"manus_triggers.{side}必须是对象")
                for name, payload in triggers.items():
                    trigger = ManusTrigger.create(
                        name,
                        side,
                        payload["finger"],
                        payload["pose"],
                        enter_max_m=payload["enter_max_m"],
                        exit_min_m=payload["exit_min_m"],
                        dwell_seconds=payload.get("dwell_seconds", 0.15),
                        calibration=payload.get("calibration", {}),
                    )
                    if trigger.pose_name not in self._poses[side]:
                        raise ValueError(
                            f"触发器{trigger.name}引用不存在的姿态{trigger.pose_name}"
                        )
                    self._triggers[side][trigger.name] = trigger
                gesture_triggers = data.get("manus_gesture_triggers", {}).get(
                    side, {}
                )
                if not isinstance(gesture_triggers, dict):
                    raise ValueError(f"manus_gesture_triggers.{side}必须是对象")
                for name, payload in gesture_triggers.items():
                    trigger = ManusGestureTrigger.create(
                        name,
                        side,
                        payload["pose"],
                        payload["feature_ranges"],
                        enter_match_fraction=payload.get(
                            "enter_match_fraction", 0.80
                        ),
                        exit_match_fraction=payload.get(
                            "exit_match_fraction", 0.60
                        ),
                        dwell_seconds=payload.get("dwell_seconds", 0.15),
                        sample_count=payload.get("sample_count", 0),
                    )
                    if trigger.pose_name not in self._poses[side]:
                        raise ValueError(
                            f"复合触发器{trigger.name}引用不存在的姿态"
                            f"{trigger.pose_name}"
                        )
                    self._gesture_triggers[side][trigger.name] = trigger

    def poses(self, side: str) -> tuple[HandPose, ...]:
        with self._lock:
            return tuple(self._poses[_side(side)].values())

    def pose(self, side: str, name: str) -> HandPose:
        with self._lock:
            try:
                return self._poses[_side(side)][str(name)]
            except KeyError as error:
                raise KeyError(f"姿态不存在: {name}") from error

    def save(self, pose: HandPose, *, previous_name: str | None = None) -> None:
        with self._lock:
            poses = self._poses[_side(pose.side)]
            if previous_name is None and pose.name in poses:
                raise ValueError(f"姿态名称已存在: {pose.name}")
            if previous_name and previous_name != pose.name:
                if pose.name in poses:
                    raise ValueError(f"姿态名称已存在: {pose.name}")
                poses.pop(previous_name, None)
                for trigger_name, trigger in tuple(self._triggers[pose.side].items()):
                    if trigger.pose_name == previous_name:
                        self._triggers[pose.side][trigger_name] = ManusTrigger.create(
                            trigger.name,
                            trigger.side,
                            trigger.finger,
                            pose.name,
                            enter_max_m=trigger.enter_max_m,
                            exit_min_m=trigger.exit_min_m,
                            dwell_seconds=trigger.dwell_seconds,
                            calibration=trigger.calibration,
                        )
                for trigger_name, trigger in tuple(
                    self._gesture_triggers[pose.side].items()
                ):
                    if trigger.pose_name == previous_name:
                        self._gesture_triggers[pose.side][trigger_name] = (
                            ManusGestureTrigger.create(
                                trigger.name,
                                trigger.side,
                                pose.name,
                                trigger.feature_ranges,
                                enter_match_fraction=trigger.enter_match_fraction,
                                exit_match_fraction=trigger.exit_match_fraction,
                                dwell_seconds=trigger.dwell_seconds,
                                sample_count=trigger.sample_count,
                            )
                        )
            poses[pose.name] = pose
            self._persist()

    def delete(self, side: str, name: str) -> None:
        with self._lock:
            normalized_side = _side(side)
            used_by = [
                trigger.name
                for trigger in self._triggers[normalized_side].values()
                if trigger.pose_name == str(name)
            ]
            used_by.extend(
                trigger.name
                for trigger in self._gesture_triggers[normalized_side].values()
                if trigger.pose_name == str(name)
            )
            if used_by:
                raise ValueError("姿态正被MANUS触发器引用: " + ", ".join(used_by))
            if self._poses[normalized_side].pop(str(name), None) is None:
                raise KeyError(f"姿态不存在: {name}")
            self._persist()

    def triggers(self, side: str) -> tuple[ManusTrigger, ...]:
        with self._lock:
            return tuple(self._triggers[_side(side)].values())

    def save_trigger(
        self, trigger: ManusTrigger, *, previous_name: str | None = None
    ) -> None:
        with self._lock:
            if trigger.pose_name not in self._poses[trigger.side]:
                raise ValueError(f"对应姿态不存在: {trigger.pose_name}")
            triggers = self._triggers[trigger.side]
            if previous_name is None and trigger.name in triggers:
                raise ValueError(f"触发器名称已存在: {trigger.name}")
            if previous_name and previous_name != trigger.name:
                if trigger.name in triggers:
                    raise ValueError(f"触发器名称已存在: {trigger.name}")
                triggers.pop(previous_name, None)
            triggers[trigger.name] = trigger
            self._persist()

    def delete_trigger(self, side: str, name: str) -> None:
        with self._lock:
            if self._triggers[_side(side)].pop(str(name), None) is None:
                raise KeyError(f"触发器不存在: {name}")
            self._persist()

    def gesture_triggers(self, side: str) -> tuple[ManusGestureTrigger, ...]:
        with self._lock:
            return tuple(self._gesture_triggers[_side(side)].values())

    def save_gesture_trigger(
        self, trigger: ManusGestureTrigger, *, previous_name: str | None = None
    ) -> None:
        with self._lock:
            if trigger.pose_name not in self._poses[trigger.side]:
                raise ValueError(f"对应姿态不存在: {trigger.pose_name}")
            triggers = self._gesture_triggers[trigger.side]
            if previous_name is None and trigger.name in triggers:
                raise ValueError(f"复合触发器名称已存在: {trigger.name}")
            if previous_name and previous_name != trigger.name:
                if trigger.name in triggers:
                    raise ValueError(f"复合触发器名称已存在: {trigger.name}")
                triggers.pop(previous_name, None)
            triggers[trigger.name] = trigger
            self._persist()

    def delete_gesture_trigger(self, side: str, name: str) -> None:
        with self._lock:
            if self._gesture_triggers[_side(side)].pop(str(name), None) is None:
                raise KeyError(f"复合触发器不存在: {name}")
            self._persist()

    def _persist(self) -> None:
        data = {
            "format": FORMAT,
            "version": VERSION,
            "units": "rad",
            "joint_order": "device",
            "joint_names": {side: list(self.joint_names[side]) for side in SIDES},
            "poses": {
                side: {
                    name: list(pose.qpos)
                    for name, pose in self._poses[side].items()
                }
                for side in SIDES
            },
            "manus_triggers": {
                side: {
                    name: {
                        "finger": trigger.finger,
                        "pose": trigger.pose_name,
                        "metric": "thumb_tip_distance",
                        "enter_max_m": trigger.enter_max_m,
                        "exit_min_m": trigger.exit_min_m,
                        "dwell_seconds": trigger.dwell_seconds,
                        "calibration": dict(trigger.calibration),
                    }
                    for name, trigger in self._triggers[side].items()
                }
                for side in SIDES
            },
            "manus_gesture_triggers": {
                side: {
                    name: {
                        "pose": trigger.pose_name,
                        "metric": "composite_landmark_ranges_v1",
                        "feature_ranges": {
                            feature: dict(bounds)
                            for feature, bounds in trigger.feature_ranges.items()
                        },
                        "enter_match_fraction": trigger.enter_match_fraction,
                        "exit_match_fraction": trigger.exit_match_fraction,
                        "dwell_seconds": trigger.dwell_seconds,
                        "sample_count": trigger.sample_count,
                    }
                    for name, trigger in self._gesture_triggers[side].items()
                }
                for side in SIDES
            },
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{self.path.name}.", suffix=".tmp", dir=self.path.parent
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(data, stream, indent=2, ensure_ascii=False)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, self.path)
        except BaseException:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise
