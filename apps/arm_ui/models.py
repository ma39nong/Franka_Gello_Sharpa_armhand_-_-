"""Validated YAML persistence for taught FR3 waypoints and routines."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import os
from pathlib import Path
import tempfile
import threading
from typing import Any, Iterable, Mapping

import yaml

SIDES = ("left", "right")
JOINT_COUNT = 7
MAX_ROUTINE_WAYPOINTS = 9


def _name(value: object, *, label: str) -> str:
    text = str(value).strip()
    if not text:
        raise ValueError(f"{label}不能为空")
    if len(text) > 64:
        raise ValueError(f"{label}不能超过64个字符")
    if any(character in text for character in ("/", "\\", "\0", "\n", "\r")):
        raise ValueError(f"{label}不能包含斜杠或换行")
    return text


def _side(value: object) -> str:
    side = str(value)
    if side not in SIDES:
        raise ValueError(f"未知机械臂: {side!r}")
    return side


def _joint_tuple(values: Iterable[object]) -> tuple[float, ...]:
    joints = tuple(float(value) for value in values)
    if len(joints) != JOINT_COUNT:
        raise ValueError(f"点位必须包含{JOINT_COUNT}个关节角")
    if not all(math.isfinite(value) for value in joints):
        raise ValueError("关节角必须是有限数值")
    return joints


def _scale(value: object, *, label: str) -> float:
    scale = float(value)
    if not math.isfinite(scale) or not 0.01 <= scale <= 1.0:
        raise ValueError(f"{label}必须在1%到100%之间")
    return scale


@dataclass(frozen=True)
class Waypoint:
    name: str
    side: str
    joints: tuple[float, ...]

    @classmethod
    def create(cls, name: object, side: object, joints: Iterable[object]) -> "Waypoint":
        return cls(_name(name, label="点位名称"), _side(side), _joint_tuple(joints))


@dataclass(frozen=True)
class Routine:
    name: str
    side: str
    waypoints: tuple[str, ...]
    velocity_scale: float = 0.10
    acceleration_scale: float = 0.10
    blend_radius_m: float = 0.005
    repeat: int = 1

    @classmethod
    def create(
        cls,
        name: object,
        side: object,
        waypoints: Iterable[object],
        *,
        velocity_scale: object = 0.10,
        acceleration_scale: object = 0.10,
        blend_radius_m: object = 0.005,
        repeat: object = 1,
    ) -> "Routine":
        names = tuple(_name(value, label="点位名称") for value in waypoints)
        if not 1 <= len(names) <= MAX_ROUTINE_WAYPOINTS:
            raise ValueError("任务必须包含1到9个点位")
        radius = float(blend_radius_m)
        if not math.isfinite(radius) or not 0.0 <= radius <= 0.10:
            raise ValueError("平滑半径必须在0到100毫米之间")
        repeat_count = int(repeat)
        if repeat_count != 1:
            raise ValueError(
                "当前版本只允许执行一次；循环将在真机验证后开放"
            )
        return cls(
            name=_name(name, label="任务名称"),
            side=_side(side),
            waypoints=names,
            velocity_scale=_scale(velocity_scale, label="速度比例"),
            acceleration_scale=_scale(acceleration_scale, label="加速度比例"),
            blend_radius_m=radius,
            repeat=repeat_count,
        )


class ArmRepository:
    """Thread-safe two-file YAML repository.

    Point definitions and routines are deliberately separate. Routines refer to
    point names so a taught point can be adjusted without duplicating joint data.
    """

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        self.waypoints_path = self.root / "waypoints.yaml"
        self.routines_path = self.root / "routines.yaml"
        self._lock = threading.RLock()
        self.root.mkdir(parents=True, exist_ok=True)
        self._waypoints: dict[str, dict[str, Waypoint]] = {side: {} for side in SIDES}
        self._routines: dict[str, dict[str, Routine]] = {side: {} for side in SIDES}
        self.reload()

    def reload(self) -> None:
        with self._lock:
            self._waypoints = {side: {} for side in SIDES}
            self._routines = {side: {} for side in SIDES}
            waypoint_data = self._read_yaml(self.waypoints_path)
            for side, entries in waypoint_data.get("arms", {}).items():
                side = _side(side)
                for name, payload in self._mapping(entries, "waypoints").items():
                    point = Waypoint.create(name, side, payload["joints"])
                    self._waypoints[side][point.name] = point

            routine_data = self._read_yaml(self.routines_path)
            for side, entries in routine_data.get("arms", {}).items():
                side = _side(side)
                for name, payload in self._mapping(entries, "routines").items():
                    routine = Routine.create(
                        name,
                        side,
                        payload["waypoints"],
                        velocity_scale=payload.get("velocity_scale", 0.10),
                        acceleration_scale=payload.get("acceleration_scale", 0.10),
                        blend_radius_m=payload.get("blend_radius_m", 0.005),
                        repeat=payload.get("repeat", 1),
                    )
                    self._routines[side][routine.name] = routine

    @staticmethod
    def _mapping(value: object, label: str) -> Mapping[str, Mapping[str, Any]]:
        if value is None:
            return {}
        if not isinstance(value, Mapping):
            raise ValueError(f"{label} YAML内容必须是映射")
        return value  # type: ignore[return-value]

    @staticmethod
    def _read_yaml(path: Path) -> dict[str, Any]:
        if not path.exists():
            return {}
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        if not isinstance(data, dict):
            raise ValueError(f"{path} 顶层必须是映射")
        version = data.get("version", 1)
        if version != 1:
            raise ValueError(f"不支持的YAML版本: {version!r}")
        return data

    @staticmethod
    def _write_yaml(path: Path, data: Mapping[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                yaml.safe_dump(
                    dict(data), stream, allow_unicode=True, sort_keys=False
                )
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, path)
        except BaseException:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise

    def waypoints(self, side: str) -> tuple[Waypoint, ...]:
        with self._lock:
            return tuple(self._waypoints[_side(side)].values())

    def waypoint(self, side: str, name: str) -> Waypoint:
        with self._lock:
            try:
                return self._waypoints[_side(side)][name]
            except KeyError as error:
                raise KeyError(f"点位不存在: {name}") from error

    def save_waypoint(self, waypoint: Waypoint, *, previous_name: str | None = None) -> None:
        with self._lock:
            points = self._waypoints[waypoint.side]
            if previous_name is None and waypoint.name in points:
                raise ValueError(f"点位名称已存在: {waypoint.name}")
            if previous_name and previous_name != waypoint.name:
                if waypoint.name in points:
                    raise ValueError(f"点位名称已存在: {waypoint.name}")
                points.pop(previous_name, None)
                for name, routine in tuple(self._routines[waypoint.side].items()):
                    if previous_name in routine.waypoints:
                        self._routines[waypoint.side][name] = Routine.create(
                            routine.name,
                            routine.side,
                            [
                                waypoint.name if item == previous_name else item
                                for item in routine.waypoints
                            ],
                            velocity_scale=routine.velocity_scale,
                            acceleration_scale=routine.acceleration_scale,
                            blend_radius_m=routine.blend_radius_m,
                        )
                self._persist_routines()
            points[waypoint.name] = waypoint
            self._persist_waypoints()

    def delete_waypoint(self, side: str, name: str) -> None:
        with self._lock:
            side = _side(side)
            used_by = [
                routine.name
                for routine in self._routines[side].values()
                if name in routine.waypoints
            ]
            if used_by:
                raise ValueError("点位正被任务引用: " + ", ".join(used_by))
            if self._waypoints[side].pop(name, None) is None:
                raise KeyError(f"点位不存在: {name}")
            self._persist_waypoints()

    def routines(self, side: str) -> tuple[Routine, ...]:
        with self._lock:
            return tuple(self._routines[_side(side)].values())

    def save_routine(self, routine: Routine) -> None:
        with self._lock:
            missing = [
                name
                for name in routine.waypoints
                if name not in self._waypoints[routine.side]
            ]
            if missing:
                raise ValueError("任务引用了不存在的点位: " + ", ".join(missing))
            self._routines[routine.side][routine.name] = routine
            self._persist_routines()

    def delete_routine(self, side: str, name: str) -> None:
        with self._lock:
            if self._routines[_side(side)].pop(name, None) is None:
                raise KeyError(f"任务不存在: {name}")
            self._persist_routines()

    def resolve(self, routine: Routine) -> tuple[Waypoint, ...]:
        with self._lock:
            return tuple(self.waypoint(routine.side, name) for name in routine.waypoints)

    def _persist_waypoints(self) -> None:
        data = {
            "version": 1,
            "units": "rad",
            "arms": {
                side: {
                    point.name: {"joints": list(point.joints)}
                    for point in points.values()
                }
                for side, points in self._waypoints.items()
            },
        }
        self._write_yaml(self.waypoints_path, data)

    def _persist_routines(self) -> None:
        data = {
            "version": 1,
            "arms": {
                side: {
                    routine.name: {
                        key: value
                        for key, value in asdict(routine).items()
                        if key not in {"name", "side"}
                    }
                    for routine in routines.values()
                }
                for side, routines in self._routines.items()
            },
        }
        self._write_yaml(self.routines_path, data)
