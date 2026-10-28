"""Pure data processing and YAML storage for hand-guided motion captures."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import re
import tempfile
from typing import Iterable, Mapping, Sequence

import yaml

from .models import JOINT_COUNT, SIDES


def _finite_tuple(values: Iterable[object], width: int, label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != width or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label}必须包含{width}个有限数值")
    return result


def _normalize_quaternion(values: Iterable[object]) -> tuple[float, float, float, float]:
    quaternion = _finite_tuple(values, 4, "四元数")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-9:
        raise ValueError("四元数长度不能为零")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def _quaternion_conjugate(
    quaternion: Sequence[float],
) -> tuple[float, float, float, float]:
    return (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])


def _quaternion_multiply(
    left: Sequence[float], right: Sequence[float]
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate_vector(
    quaternion: Sequence[float], vector: Sequence[float]
) -> tuple[float, float, float]:
    pure = (vector[0], vector[1], vector[2], 0.0)
    rotated = _quaternion_multiply(
        _quaternion_multiply(quaternion, pure),
        _quaternion_conjugate(quaternion),
    )
    return rotated[:3]


def _dot(left: Sequence[float], right: Sequence[float]) -> float:
    return sum(a * b for a, b in zip(left, right, strict=True))


def _quaternion_slerp(
    start: Sequence[float], end: Sequence[float], fraction: float
) -> tuple[float, float, float, float]:
    left = _normalize_quaternion(start)
    right = _normalize_quaternion(end)
    cosine = _dot(left, right)
    if cosine < 0.0:
        right = tuple(-value for value in right)
        cosine = -cosine
    cosine = max(-1.0, min(1.0, cosine))
    if cosine > 0.9995:
        return _normalize_quaternion(
            left[index] + fraction * (right[index] - left[index])
            for index in range(4)
        )
    angle = math.acos(cosine)
    sine = math.sin(angle)
    left_weight = math.sin((1.0 - fraction) * angle) / sine
    right_weight = math.sin(fraction * angle) / sine
    return _normalize_quaternion(
        left_weight * left[index] + right_weight * right[index]
        for index in range(4)
    )


@dataclass(frozen=True)
class MotionSample:
    time_sec: float
    joints: tuple[float, ...]
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]

    @classmethod
    def create(
        cls,
        time_sec: object,
        joints: Iterable[object],
        position: Iterable[object],
        orientation_xyzw: Iterable[object],
    ) -> "MotionSample":
        timestamp = float(time_sec)
        if not math.isfinite(timestamp) or timestamp < 0.0:
            raise ValueError("采样时间必须是非负有限数值")
        return cls(
            timestamp,
            _finite_tuple(joints, JOINT_COUNT, "关节角"),
            _finite_tuple(position, 3, "末端位置"),  # type: ignore[arg-type]
            _normalize_quaternion(orientation_xyzw),
        )


@dataclass(frozen=True)
class RelativeMotionFrame:
    time_sec: float
    joint_delta: tuple[float, ...]
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class RecordedAction:
    name: str
    side: str
    base_frame: str
    tool_frame: str
    created_at: str
    raw_samples: tuple[MotionSample, ...]
    relative_frames: tuple[RelativeMotionFrame, ...]
    active_joints: tuple[int, ...]
    measured_rate_hz: float

    @property
    def duration_sec(self) -> float:
        return self.relative_frames[-1].time_sec


def smooth_relative_frames(
    frames: Sequence[RelativeMotionFrame],
    *,
    window_sec: float = 0.12,
    target_rate_hz: float = 30.0,
) -> tuple[RelativeMotionFrame, ...]:
    """Low-pass and uniformly resample an offline tool-relative path.

    The raw recording remains untouched. A symmetric triangular window avoids
    phase lag, quaternion signs are aligned before their weighted normalized
    average, and SLERP is used when resampling orientations.
    """
    source = tuple(frames)
    if len(source) < 3:
        raise ValueError("相对动作至少需要3帧")
    times = tuple(frame.time_sec for frame in source)
    if any(times[index] <= times[index - 1] for index in range(1, len(times))):
        raise ValueError("相对动作时间必须严格递增")
    window_sec = float(window_sec)
    target_rate_hz = float(target_rate_hz)
    if not math.isfinite(window_sec) or window_sec <= 0.0:
        raise ValueError("平滑窗口必须为正数")
    if not math.isfinite(target_rate_hz) or target_rate_hz <= 0.0:
        raise ValueError("重采样频率必须为正数")

    intervals = sorted(times[index] - times[index - 1] for index in range(1, len(times)))
    median_interval = intervals[len(intervals) // 2]
    radius = max(1, min(25, round(window_sec / (2.0 * median_interval))))
    smoothed: list[RelativeMotionFrame] = []
    for index, frame in enumerate(source):
        if index in {0, len(source) - 1}:
            smoothed.append(frame)
            continue
        first = max(0, index - radius)
        last = min(len(source), index + radius + 1)
        weighted_position = [0.0, 0.0, 0.0]
        weighted_joint_delta = [0.0] * len(frame.joint_delta)
        reference = frame.orientation_xyzw
        weighted_orientation = [0.0, 0.0, 0.0, 0.0]
        total_weight = 0.0
        for sample_index in range(first, last):
            weight = float(radius + 1 - abs(sample_index - index))
            sample = source[sample_index]
            total_weight += weight
            for axis in range(3):
                weighted_position[axis] += weight * sample.position[axis]
            for joint in range(len(weighted_joint_delta)):
                weighted_joint_delta[joint] += weight * sample.joint_delta[joint]
            orientation = sample.orientation_xyzw
            if _dot(reference, orientation) < 0.0:
                orientation = tuple(-value for value in orientation)
            for component in range(4):
                weighted_orientation[component] += weight * orientation[component]
        smoothed.append(
            RelativeMotionFrame(
                time_sec=frame.time_sec,
                joint_delta=tuple(value / total_weight for value in weighted_joint_delta),
                position=tuple(value / total_weight for value in weighted_position),  # type: ignore[arg-type]
                orientation_xyzw=_normalize_quaternion(weighted_orientation),
            )
        )

    duration = times[-1] - times[0]
    target_count = min(
        len(smoothed), max(3, int(math.ceil(duration * target_rate_hz)) + 1)
    )
    output_times = tuple(
        times[0] + duration * index / (target_count - 1)
        for index in range(target_count)
    )
    smoothed_times = tuple(frame.time_sec for frame in smoothed)
    output: list[RelativeMotionFrame] = []
    for timestamp in output_times:
        if timestamp <= smoothed_times[0]:
            output.append(smoothed[0])
            continue
        if timestamp >= smoothed_times[-1]:
            output.append(smoothed[-1])
            continue
        right_index = bisect_right(smoothed_times, timestamp)
        left = smoothed[right_index - 1]
        right = smoothed[right_index]
        fraction = (timestamp - left.time_sec) / (right.time_sec - left.time_sec)
        output.append(
            RelativeMotionFrame(
                time_sec=timestamp,
                joint_delta=tuple(
                    start + fraction * (end - start)
                    for start, end in zip(
                        left.joint_delta, right.joint_delta, strict=True
                    )
                ),
                position=tuple(
                    start + fraction * (end - start)
                    for start, end in zip(left.position, right.position, strict=True)
                ),  # type: ignore[arg-type]
                orientation_xyzw=_quaternion_slerp(
                    left.orientation_xyzw, right.orientation_xyzw, fraction
                ),
            )
        )
    return tuple(output)


def compose_pose(
    position: Sequence[float],
    orientation_xyzw: Sequence[float],
    relative_position: Sequence[float],
    relative_orientation_xyzw: Sequence[float],
) -> tuple[
    tuple[float, float, float], tuple[float, float, float, float]
]:
    """Apply a tool-local relative pose to a pose expressed in a fixed base."""
    base_position = _finite_tuple(position, 3, "起始末端位置")
    base_orientation = _normalize_quaternion(orientation_xyzw)
    offset = _finite_tuple(relative_position, 3, "相对末端位置")
    relative_orientation = _normalize_quaternion(relative_orientation_xyzw)
    rotated_offset = _rotate_vector(base_orientation, offset)
    composed_position = tuple(
        value + delta
        for value, delta in zip(base_position, rotated_offset, strict=True)
    )
    composed_orientation = _normalize_quaternion(
        _quaternion_multiply(base_orientation, relative_orientation)
    )
    return composed_position, composed_orientation


def build_recorded_action(
    *,
    name: str,
    side: str,
    base_frame: str,
    tool_frame: str,
    samples: Iterable[MotionSample],
    motion_threshold_rad: float = 0.003,
    trim_padding_sec: float = 0.10,
) -> RecordedAction:
    name = str(name).strip()
    if not name:
        raise ValueError("动作名称不能为空")
    if len(name) > 64 or any(value in name for value in ("/", "\\", "\n", "\r")):
        raise ValueError("动作名称不能超过64个字符，也不能包含斜杠或换行")
    if side not in SIDES:
        raise ValueError(f"未知机械臂: {side}")
    raw = tuple(samples)
    if len(raw) < 3:
        raise ValueError("录制样本太少")
    if any(raw[index].time_sec <= raw[index - 1].time_sec for index in range(1, len(raw))):
        raise ValueError("录制时间必须严格递增")

    ranges = tuple(
        max(sample.joints[joint] for sample in raw)
        - min(sample.joints[joint] for sample in raw)
        for joint in range(JOINT_COUNT)
    )
    active = tuple(index + 1 for index, value in enumerate(ranges) if value >= 0.01)
    if max(ranges) < motion_threshold_rad:
        raise ValueError("没有检测到有效机械臂运动")

    first_motion = next(
        index
        for index, sample in enumerate(raw)
        if max(
            abs(value - start)
            for value, start in zip(sample.joints, raw[0].joints, strict=True)
        )
        >= motion_threshold_rad
    )
    last_motion = next(
        index
        for index in range(len(raw) - 1, -1, -1)
        if max(
            abs(value - end)
            for value, end in zip(raw[index].joints, raw[-1].joints, strict=True)
        )
        >= motion_threshold_rad
    )
    start_time = max(raw[first_motion].time_sec - trim_padding_sec, raw[0].time_sec)
    end_time = min(raw[last_motion].time_sec + trim_padding_sec, raw[-1].time_sec)
    trimmed = tuple(sample for sample in raw if start_time <= sample.time_sec <= end_time)
    if len(trimmed) < 3:
        raise ValueError("裁剪后的动作样本太少")

    origin = trimmed[0]
    inverse_origin_orientation = _quaternion_conjugate(origin.orientation_xyzw)
    previous_orientation = origin.orientation_xyzw
    relative = []
    for sample in trimmed:
        orientation = sample.orientation_xyzw
        if _dot(previous_orientation, orientation) < 0.0:
            orientation = tuple(-value for value in orientation)
        previous_orientation = orientation
        world_offset = tuple(
            value - start
            for value, start in zip(sample.position, origin.position, strict=True)
        )
        relative_position = _rotate_vector(inverse_origin_orientation, world_offset)
        relative_orientation = _normalize_quaternion(
            _quaternion_multiply(inverse_origin_orientation, orientation)
        )
        relative.append(
            RelativeMotionFrame(
                time_sec=sample.time_sec - origin.time_sec,
                joint_delta=tuple(
                    value - start
                    for value, start in zip(sample.joints, origin.joints, strict=True)
                ),
                position=relative_position,
                orientation_xyzw=relative_orientation,
            )
        )
    duration = relative[-1].time_sec
    if duration <= 0.0:
        raise ValueError("动作持续时间必须大于零")
    return RecordedAction(
        name=name,
        side=side,
        base_frame=str(base_frame),
        tool_frame=str(tool_frame),
        created_at=datetime.now(timezone.utc).isoformat(),
        raw_samples=raw,
        relative_frames=tuple(relative),
        active_joints=active,
        measured_rate_hz=(len(raw) - 1) / (raw[-1].time_sec - raw[0].time_sec),
    )


def _safe_filename(name: str) -> str:
    value = re.sub(r"[^\w.-]+", "_", name, flags=re.UNICODE).strip("._")
    return value or "action"


class ActionStore:
    def __init__(self, root: str | Path) -> None:
        self.root = Path(root) / "actions"
        self.root.mkdir(parents=True, exist_ok=True)

    def path_for(self, side: str, name: str) -> Path:
        return self.root / f"{side}__{_safe_filename(name)}.yaml"

    def save(self, action: RecordedAction, *, overwrite: bool = False) -> Path:
        path = self.path_for(action.side, action.name)
        if path.exists() and not overwrite:
            raise ValueError(f"动作名称已存在: {action.name}")
        data = {
            "version": 1,
            "name": action.name,
            "side": action.side,
            "created_at": action.created_at,
            "base_frame": action.base_frame,
            "tool_frame": action.tool_frame,
            "units": {"joint": "rad", "position": "m", "time": "s"},
            "measured_rate_hz": action.measured_rate_hz,
            "active_joints": list(action.active_joints),
            "duration_sec": action.duration_sec,
            "raw_samples": [
                {
                    "time_sec": sample.time_sec,
                    "joints": list(sample.joints),
                    "tool_pose": {
                        "position": list(sample.position),
                        "orientation_xyzw": list(sample.orientation_xyzw),
                    },
                }
                for sample in action.raw_samples
            ],
            "relative_frames": [
                {
                    "time_sec": frame.time_sec,
                    "joint_delta": list(frame.joint_delta),
                    "tool_pose": {
                        "position": list(frame.position),
                        "orientation_xyzw": list(frame.orientation_xyzw),
                    },
                }
                for frame in action.relative_frames
            ],
        }
        self._write_yaml(path, data)
        return path

    def summaries(self, side: str) -> tuple[Mapping[str, object], ...]:
        summaries = []
        for path in sorted(self.root.glob(f"{side}__*.yaml")):
            with path.open("r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
            summaries.append(
                {
                    "name": data.get("name", path.stem),
                    "duration_sec": float(data.get("duration_sec", 0.0)),
                    "sample_count": len(data.get("relative_frames", [])),
                    "active_joints": tuple(data.get("active_joints", [])),
                    "path": path,
                }
            )
        return tuple(summaries)

    def load(self, side: str, name: str) -> RecordedAction:
        path = self.path_for(side, name)
        if not path.exists():
            raise KeyError(f"动作不存在: {name}")
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        if int(data.get("version", 0)) != 1:
            raise ValueError(f"不支持的动作文件版本: {data.get('version')}")
        stored_side = str(data.get("side", ""))
        if stored_side != side:
            raise ValueError(f"动作侧别不匹配: {stored_side}")
        raw_samples = tuple(
            MotionSample.create(
                item["time_sec"],
                item["joints"],
                item["tool_pose"]["position"],
                item["tool_pose"]["orientation_xyzw"],
            )
            for item in data.get("raw_samples", [])
        )
        relative_frames = tuple(
            RelativeMotionFrame(
                time_sec=float(item["time_sec"]),
                joint_delta=_finite_tuple(item["joint_delta"], JOINT_COUNT, "关节增量"),
                position=_finite_tuple(
                    item["tool_pose"]["position"], 3, "相对末端位置"
                ),  # type: ignore[arg-type]
                orientation_xyzw=_normalize_quaternion(
                    item["tool_pose"]["orientation_xyzw"]
                ),
            )
            for item in data.get("relative_frames", [])
        )
        if len(raw_samples) < 3 or len(relative_frames) < 3:
            raise ValueError("动作文件样本太少")
        if any(
            relative_frames[index].time_sec
            <= relative_frames[index - 1].time_sec
            for index in range(1, len(relative_frames))
        ):
            raise ValueError("动作文件时间必须严格递增")
        return RecordedAction(
            name=str(data.get("name", name)),
            side=stored_side,
            base_frame=str(data.get("base_frame", "")),
            tool_frame=str(data.get("tool_frame", "")),
            created_at=str(data.get("created_at", "")),
            raw_samples=raw_samples,
            relative_frames=relative_frames,
            active_joints=tuple(int(value) for value in data.get("active_joints", [])),
            measured_rate_hz=float(data.get("measured_rate_hz", 0.0)),
        )

    def delete(self, side: str, name: str) -> None:
        path = self.path_for(side, name)
        if not path.exists():
            raise KeyError(f"动作不存在: {name}")
        path.unlink()

    @staticmethod
    def _write_yaml(path: Path, data: Mapping[str, object]) -> None:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                yaml.safe_dump(dict(data), stream, allow_unicode=True, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, path)
        except BaseException:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise
