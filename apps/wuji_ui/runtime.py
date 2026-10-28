"""Single-owner hardware runtime behind the Wuji browser UI."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Mapping, Sequence

import numpy as np

from adapters.wuji.pipeline import WujiHandPipeline, canonical_landmarks

from .models import SIDES, manus_gesture_features


MAX_RUNTIME_KP = 10.0
MAX_RUNTIME_KD = 1.0


@dataclass(frozen=True)
class HandSnapshot:
    side: str
    qpos: tuple[float, ...]
    received_at: float


@dataclass(frozen=True)
class ManusGapSnapshot:
    side: str
    gaps_m: Mapping[str, float]
    features: Mapping[str, float]
    sequence: int
    received_at: float


@dataclass(frozen=True)
class PoseSequenceStatus:
    running: bool
    step_index: int
    step_count: int
    pose_name: str


@dataclass(frozen=True)
class JointDiagnosticSnapshot:
    side: str
    joint_index: int
    joint_name: str
    node_id: int
    target_position: float | None
    actual_position: float
    current_a: float
    bus_voltage_v: float
    temperature_c: float
    error_code: int
    ext_state: int
    ext_state_name: str
    position_limit_active: bool
    velocity_limit_active: bool
    current_limit_active: bool
    comm_response_rate_pct: int
    comm_timeout_total: int
    received_at: float

    @property
    def position_error(self) -> float | None:
        if self.target_position is None:
            return None
        return self.target_position - self.actual_position


def summarize_joint_hold_test(
    samples: Sequence[JointDiagnosticSnapshot], physical_observation: str
) -> dict[str, object]:
    """Summarize a stationary hold test without pretending to see mechanics."""
    if len(samples) < 5:
        raise ValueError("保持测试至少需要5个新诊断帧")
    errors = [
        abs(sample.position_error)
        for sample in samples
        if sample.position_error is not None
    ]
    if not errors:
        raise ValueError("保持测试没有有效目标位置；请先使能并保持")
    actual = np.asarray([sample.actual_position for sample in samples])
    max_error_rad = float(max(errors))
    max_deflection_rad = float(np.max(np.abs(actual - actual[0])))
    error_codes = sorted({sample.error_code for sample in samples if sample.error_code})
    current_limit_seen = any(sample.current_limit_active for sample in samples)
    result: dict[str, object] = {
        "sample_count": len(samples),
        "max_error_rad": max_error_rad,
        "max_deflection_rad": max_deflection_rad,
        "peak_abs_current_a": max(abs(sample.current_a) for sample in samples),
        "max_temperature_c": max(sample.temperature_c for sample in samples),
        "min_comm_response_rate_pct": min(
            sample.comm_response_rate_pct for sample in samples
        ),
        "current_limit_seen": current_limit_seen,
        "error_codes": error_codes,
    }
    two_degrees = np.deg2rad(2.0)
    if error_codes:
        conclusion = "驱动器报告非零错误码；先排查驱动/供电，不应提高增益。"
        category = "driver_fault"
    elif (
        physical_observation == "feedback_static"
        and max_deflection_rad < two_degrees
    ):
        conclusion = (
            "实体关节移动但电机反馈基本不动，机械传动松脱或间隙过大的可能性高。"
        )
        category = "mechanical_likely"
    elif physical_observation == "feedback_static":
        conclusion = (
            "人工观察称反馈基本不动，但采集到的反馈偏移已超过2°；"
            "证据不一致，请重测并确认页面实际角。"
        )
        category = "inconclusive"
    elif max_error_rad >= two_degrees and current_limit_seen:
        conclusion = (
            "反馈明显偏离固定目标且触发电流限幅：控制输出已到上限；"
            "可能是限流偏低、负载过大或机械阻力/传动异常。"
        )
        category = "current_limited"
    elif max_error_rad >= two_degrees:
        conclusion = (
            "反馈明显偏离固定目标但未见电流限幅：保持增益不足较可疑，"
            "仍应与健康ABD关节对比后再改单关节参数。"
        )
        category = "gain_suspect"
    elif physical_observation == "feedback_moves":
        conclusion = (
            "实体与反馈都移动，但本次目标误差较小；请更稳定地轻推重测，"
            "并与健康ABD关节做同样力度对比。"
        )
        category = "inconclusive"
    else:
        conclusion = (
            "电气数据未显示明显目标偏差。仅凭编码器数据无法排除编码器之后的"
            "机械松脱，请补充实体与页面反馈是否同步。"
        )
        category = "mechanical_check_needed"
    result["category"] = category
    result["conclusion"] = conclusion
    return result


class WujiUiRuntime:
    """Own both hands, MANUS retargeting, feedback, and bounded pose motion."""

    def __init__(
        self,
        *,
        addresses: Mapping[str, str],
        kp: float = 8.0,
        kd: float = 0.1,
        current_limit: float = 1.0,
        rate: float = 30.0,
        stale_timeout: float = 0.25,
        pose_speed_rad_s: float = 1.0,
        feedback_timeout: float = 0.5,
    ) -> None:
        if set(addresses) != set(SIDES) or any(not addresses[side] for side in SIDES):
            raise ValueError("Wuji UI需要左右两只Hand 2的明确地址")
        if not np.isfinite(pose_speed_rad_s) or pose_speed_rad_s <= 0.0:
            raise ValueError("姿态移动速度必须为正数")
        self.pose_speed_rad_s = float(pose_speed_rad_s)
        self.feedback_timeout = float(feedback_timeout)
        self.pipeline = WujiHandPipeline(
            sides=SIDES,
            models={side: "wuji_hand_2" for side in SIDES},
            addresses=dict(addresses),
            rate=rate,
            stale_timeout=stale_timeout,
            kp=kp,
            kd=kd,
            current_limit=current_limit,
            auto_enable=False,
        )
        self.joint_names = {
            side: tuple(self.pipeline.joint_names[side]) for side in SIDES
        }
        self.joint_limits = {
            side: tuple(self.pipeline.joint_limits[side]) for side in SIDES
        }
        self._lock = threading.RLock()
        self._hardware_lock = threading.RLock()
        self._stop = threading.Event()
        self._modes = {side: "manual" for side in SIDES}
        self._snapshots: dict[str, HandSnapshot] = {}
        self._manus_gaps: dict[str, ManusGapSnapshot] = {}
        self._manus_sequences = {side: -1 for side in SIDES}
        self._joint_diagnostics: dict[
            tuple[str, int], JointDiagnosticSnapshot
        ] = {}
        self._diagnostic_errors = {
            side: "等待关节诊断数据" for side in SIDES
        }
        self._pose_commands: dict[str, np.ndarray | None] = {
            side: None for side in SIDES
        }
        self._pose_targets: dict[str, np.ndarray | None] = {
            side: None for side in SIDES
        }
        self._errors = {side: "等待真实关节反馈" for side in SIDES}
        self._sequence_stop = {side: threading.Event() for side in SIDES}
        self._sequence_status = {
            side: PoseSequenceStatus(False, 0, 0, "") for side in SIDES
        }
        self._thread = threading.Thread(
            target=self._loop, name="wuji-ui-hardware", daemon=True
        )
        self._thread.start()

    def mode(self, side: str) -> str:
        with self._lock:
            return self._modes[side]

    def error(self, side: str) -> str:
        with self._lock:
            return self._errors[side]

    def sequence_status(self, side: str) -> PoseSequenceStatus:
        with self._lock:
            return self._sequence_status[side]

    def joint_diagnostic(
        self, side: str, joint_index: int, *, require_fresh: bool = True
    ) -> JointDiagnosticSnapshot:
        if side not in SIDES or not 0 <= int(joint_index) < 20:
            raise ValueError("关节诊断需要有效手侧和0..19关节索引")
        key = (side, int(joint_index))
        with self._lock:
            snapshot = self._joint_diagnostics.get(key)
            detail = self._diagnostic_errors[side]
        if snapshot is None:
            raise RuntimeError(detail or f"尚未收到{side}关节诊断数据")
        age = time.monotonic() - snapshot.received_at
        if require_fresh and age > self.feedback_timeout:
            raise RuntimeError(f"{side}关节诊断已过期（{age:.2f}秒）")
        return snapshot

    def snapshot(self, side: str, *, require_fresh: bool = True) -> HandSnapshot:
        with self._lock:
            snapshot = self._snapshots.get(side)
        if snapshot is None:
            raise RuntimeError(f"尚未收到{side} Wuji的完整关节反馈")
        age = time.monotonic() - snapshot.received_at
        if require_fresh and age > self.feedback_timeout:
            raise RuntimeError(f"{side} Wuji关节反馈已过期（{age:.2f}秒）")
        return snapshot

    def manus_gaps(
        self, side: str, *, require_fresh: bool = True
    ) -> ManusGapSnapshot:
        with self._lock:
            snapshot = self._manus_gaps.get(side)
        if snapshot is None:
            raise RuntimeError(f"尚未收到{side} MANUS骨骼数据")
        age = time.monotonic() - snapshot.received_at
        if require_fresh and age > self.feedback_timeout:
            raise RuntimeError(f"{side} MANUS骨骼数据已过期（{age:.2f}秒）")
        return snapshot

    def set_manual(self, side: str) -> None:
        self._sequence_stop[side].set()
        with self._hardware_lock:
            with self._lock:
                self._modes[side] = "manual"
                self._pose_commands[side] = None
                self._pose_targets[side] = None
            self.pipeline.set_enabled(side, False)

    def start_teleop(self, side: str) -> None:
        if self.sequence_status(side).running:
            raise RuntimeError(f"{side}姿态序列正在运行")
        snapshot = self.snapshot(side)
        with self._hardware_lock:
            self.pipeline.set_enabled(side, True)
            # Hold the measured pose until the next fresh MANUS solve arrives.
            self.pipeline.backends[side].send(np.asarray(snapshot.qpos))
            with self._lock:
                self._modes[side] = "teleop"
                self._pose_commands[side] = None
                self._pose_targets[side] = None

    def start_jog(self, side: str) -> None:
        """Enable motors and follow UI joint targets at pose_speed_rad_s."""
        if self.sequence_status(side).running:
            raise RuntimeError(f"{side}姿态序列正在运行")
        snapshot = self.snapshot(side)
        current = np.asarray(snapshot.qpos, dtype=np.float64)
        with self._hardware_lock:
            self.pipeline.set_enabled(side, True)
            self.pipeline.backends[side].send(current)
            with self._lock:
                self._pose_commands[side] = current.copy()
                self._pose_targets[side] = current.copy()
                self._modes[side] = "jog"

    def set_jog_target(self, side: str, qpos: Sequence[float]) -> None:
        target = self._validated_target(side, qpos)
        with self._lock:
            if self._modes[side] != "jog":
                raise RuntimeError(f"{side}不在滑块控制模式，无法更新目标")
            self._pose_targets[side] = target

    def hold_current(self, side: str) -> None:
        """Stop teleoperation and hold the freshly measured whole-hand pose."""
        if self.sequence_status(side).running:
            raise RuntimeError(f"{side}姿态序列正在运行")
        snapshot = self.snapshot(side)
        command = np.asarray(snapshot.qpos, dtype=np.float64)
        with self._hardware_lock:
            self.pipeline.set_enabled(side, True)
            self.pipeline.backends[side].send(command)
            with self._lock:
                self._modes[side] = "hold"
                self._pose_commands[side] = command.copy()
                self._pose_targets[side] = None

    def mit_gains(self, side: str) -> tuple[tuple[float, float], ...]:
        with self._hardware_lock:
            return tuple(self.pipeline.mit_gains(side))

    def set_mit_gains(
        self,
        side: str,
        *,
        kp: float,
        kd: float,
        joint_index: int | None = None,
    ) -> tuple[tuple[float, float], ...]:
        """Hold the hand, then apply bounded runtime-only MIT gains."""
        kp_value = float(kp)
        kd_value = float(kd)
        if (
            not np.isfinite((kp_value, kd_value)).all()
            or not 0.0 <= kp_value <= MAX_RUNTIME_KP
            or not 0.0 <= kd_value <= MAX_RUNTIME_KD
        ):
            raise ValueError(
                f"网页运行时调参范围：kp 0..{MAX_RUNTIME_KP:g}，"
                f"kd 0..{MAX_RUNTIME_KD:g}"
            )
        if joint_index is not None and not 0 <= int(joint_index) < 20:
            raise ValueError("关节索引必须在0..19")
        if self.sequence_status(side).running:
            raise RuntimeError(f"{side}姿态序列正在运行")
        snapshot = self.snapshot(side)
        command = np.asarray(snapshot.qpos, dtype=np.float64)
        with self._hardware_lock:
            self.pipeline.set_enabled(side, True)
            self.pipeline.backends[side].send(command)
            with self._lock:
                self._modes[side] = "hold"
                self._pose_commands[side] = command.copy()
                self._pose_targets[side] = None
            gains = self.pipeline.set_mit_gains(
                side,
                kp=kp_value,
                kd=kd_value,
                joint_index=joint_index,
            )
            self.pipeline.backends[side].send(command)
        return tuple(gains)

    def move_to_pose(self, side: str, qpos: Sequence[float]) -> None:
        if self.sequence_status(side).running:
            raise RuntimeError(f"{side}姿态序列正在运行")
        self._set_pose_target(side, qpos, mode="pose")

    def _set_pose_target(
        self, side: str, qpos: Sequence[float], *, mode: str
    ) -> None:
        target = np.asarray(qpos, dtype=np.float64)
        if target.shape != (20,) or not np.isfinite(target).all():
            raise ValueError("目标姿态必须包含20个有限关节角")
        limits = np.asarray(self.joint_limits[side], dtype=np.float64)
        if np.any(target < limits[:, 0] - 1e-6) or np.any(
            target > limits[:, 1] + 1e-6
        ):
            raise ValueError("目标姿态超出Wuji Hand 2模型关节范围")
        snapshot = self.snapshot(side)
        current = np.asarray(snapshot.qpos, dtype=np.float64)
        with self._hardware_lock:
            self.pipeline.set_enabled(side, True)
            self.pipeline.backends[side].send(current)
            with self._lock:
                self._pose_commands[side] = current
                self._pose_targets[side] = target.copy()
                self._modes[side] = mode

    def _validated_target(self, side: str, qpos: Sequence[float]) -> np.ndarray:
        target = np.asarray(qpos, dtype=np.float64)
        if target.shape != (20,) or not np.isfinite(target).all():
            raise ValueError("目标姿态必须包含20个有限关节角")
        limits = np.asarray(self.joint_limits[side], dtype=np.float64)
        if np.any(target < limits[:, 0] - 1e-6) or np.any(
            target > limits[:, 1] + 1e-6
        ):
            raise ValueError("目标姿态超出Wuji Hand 2模型关节范围")
        return target.copy()

    def execute_pose_sequence(
        self,
        side: str,
        poses: Sequence[tuple[str, Sequence[float]]],
        *,
        hold_seconds: float = 0.5,
        position_tolerance_rad: float = 0.08,
    ) -> None:
        """Execute named poses serially and wait for measured settling per step."""
        if not poses:
            raise ValueError("姿态序列不能为空")
        hold = float(hold_seconds)
        tolerance = float(position_tolerance_rad)
        if not np.isfinite(hold) or hold < 0.0:
            raise ValueError("姿态停留时间不能为负数")
        if not np.isfinite(tolerance) or tolerance <= 0.0:
            raise ValueError("姿态到位容差必须为正数")
        validated = []
        limits = np.asarray(self.joint_limits[side], dtype=np.float64)
        for name, values in poses:
            target = np.asarray(values, dtype=np.float64)
            if target.shape != (20,) or not np.isfinite(target).all():
                raise ValueError(f"姿态{name}必须包含20个有限关节角")
            if np.any(target < limits[:, 0] - 1e-6) or np.any(
                target > limits[:, 1] + 1e-6
            ):
                raise ValueError(f"姿态{name}超出Wuji Hand 2模型关节范围")
            validated.append((str(name), target.copy()))

        with self._lock:
            if self._sequence_status[side].running:
                raise RuntimeError(f"{side}姿态序列已经在运行")
            self._sequence_status[side] = PoseSequenceStatus(
                True, 0, len(validated), ""
            )
        stop = self._sequence_stop[side]
        stop.clear()
        try:
            for index, (name, target) in enumerate(validated, start=1):
                if stop.is_set() or self._stop.is_set():
                    raise InterruptedError("姿态序列已停止")
                with self._lock:
                    self._sequence_status[side] = PoseSequenceStatus(
                        True, index, len(validated), name
                    )
                start = np.asarray(self.snapshot(side).qpos, dtype=np.float64)
                expected_motion = float(np.max(np.abs(target - start))) / (
                    self.pose_speed_rad_s
                )
                deadline = time.monotonic() + expected_motion + 5.0
                self._set_pose_target(side, target, mode="sequence")
                while True:
                    if stop.wait(0.02) or self._stop.is_set():
                        raise InterruptedError("姿态序列已停止")
                    with self._lock:
                        command_done = self._pose_targets[side] is None
                    if command_done:
                        measured = np.asarray(self.snapshot(side).qpos)
                        if np.max(np.abs(target - measured)) <= tolerance:
                            break
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f"姿态{name}未在限定时间内到位")
                if hold > 0.0 and stop.wait(hold):
                    raise InterruptedError("姿态序列已停止")
            with self._lock:
                self._modes[side] = "hold"
        finally:
            with self._lock:
                self._sequence_status[side] = PoseSequenceStatus(
                    False, 0, 0, ""
                )
                if self._modes[side] == "sequence":
                    self._modes[side] = "hold"
                self._pose_targets[side] = None

    def stop_pose_sequence(self, side: str) -> bool:
        running = self.sequence_status(side).running
        self._sequence_stop[side].set()
        return running

    def _read_feedback(self, now: float) -> None:
        for side in SIDES:
            try:
                position = self.pipeline.feedback_position(side)
                if position is None:
                    continue
                snapshot = HandSnapshot(
                    side, tuple(float(value) for value in position), now
                )
                with self._lock:
                    self._snapshots[side] = snapshot
                    self._errors[side] = ""
            except Exception as error:
                with self._lock:
                    self._errors[side] = str(error)

    def _advance_poses(self, dt: float) -> None:
        for side in SIDES:
            with self._lock:
                mode = self._modes[side]
                if mode not in {"pose", "jog", "sequence"}:
                    continue
                command = self._pose_commands[side]
                target = self._pose_targets[side]
            if command is None or target is None:
                continue
            step = self.pose_speed_rad_s * max(0.0, min(dt, 0.1))
            next_command = command + np.clip(target - command, -step, step)
            self.pipeline.backends[side].send(next_command)
            reached = bool(np.max(np.abs(target - next_command)) < 1e-6)
            with self._lock:
                self._pose_commands[side] = next_command
                if not reached:
                    continue
                if mode == "pose":
                    self._modes[side] = "hold"
                    self._pose_targets[side] = None
                elif mode == "sequence":
                    self._pose_targets[side] = None
                # jog keeps following the latest slider target.

    def _read_manus_gaps(self, now: float) -> None:
        frames = getattr(self.pipeline, "last_frames", {})
        for side in SIDES:
            frame = frames.get(side)
            if frame is None:
                continue
            sequence = int(frame.sequence)
            if sequence == self._manus_sequences[side]:
                continue
            points = canonical_landmarks(frame)
            features = manus_gesture_features(points)
            thumb = points[4]
            gaps = {
                finger: float(np.linalg.norm(points[index] - thumb))
                for finger, index in {
                    "index": 8,
                    "middle": 12,
                    "ring": 16,
                    "pinky": 20,
                }.items()
            }
            with self._lock:
                self._manus_sequences[side] = sequence
                self._manus_gaps[side] = ManusGapSnapshot(
                    side, gaps, features, sequence, now
                )

    def _read_joint_diagnostics(self, now: float) -> None:
        for side in SIDES:
            try:
                diagnostics = self.pipeline.joint_diagnostics(side)
                if diagnostics is None:
                    continue
                backend = self.pipeline.backends[side]
                command = backend.last_command_position
                with self._lock:
                    position = self._snapshots.get(side)
                if position is None:
                    continue
                for index, entry in diagnostics.items():
                    target = None if command is None else float(command[index])
                    snapshot = JointDiagnosticSnapshot(
                        side=side,
                        joint_index=index,
                        joint_name=self.joint_names[side][index],
                        node_id=entry.node_id,
                        target_position=target,
                        actual_position=float(position.qpos[index]),
                        current_a=entry.current_a,
                        bus_voltage_v=entry.bus_voltage_v,
                        temperature_c=entry.temperature_c,
                        error_code=entry.error_code,
                        ext_state=entry.ext_state,
                        ext_state_name=entry.ext_state_name,
                        position_limit_active=entry.position_limit_active,
                        velocity_limit_active=entry.velocity_limit_active,
                        current_limit_active=entry.current_limit_active,
                        comm_response_rate_pct=entry.comm_response_rate_pct,
                        comm_timeout_total=entry.comm_timeout_total,
                        received_at=now,
                    )
                    with self._lock:
                        self._joint_diagnostics[(side, index)] = snapshot
                        self._diagnostic_errors[side] = ""
            except Exception as error:
                with self._lock:
                    self._diagnostic_errors[side] = str(error)

    def _loop(self) -> None:
        previous = time.monotonic()
        while not self._stop.wait(0.01):
            now = time.monotonic()
            try:
                with self._hardware_lock:
                    self._read_feedback(now)
                    with self._lock:
                        active = {
                            side: self._modes[side] == "teleop" for side in SIDES
                        }
                    self.pipeline.tick(now, active=active)
                    self._read_joint_diagnostics(now)
                    self._read_manus_gaps(now)
                    with self._lock:
                        for side in SIDES:
                            if self._modes[side] == "teleop":
                                fault = self.pipeline.status.sides[side].fault
                                if fault:
                                    self._errors[side] = str(fault)
                    self._advance_poses(now - previous)
            except Exception as error:
                with self._lock:
                    for side in SIDES:
                        if self._modes[side] != "manual":
                            self._errors[side] = str(error)
            previous = now

    def close(self) -> None:
        self._stop.set()
        for stop in self._sequence_stop.values():
            stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        with self._hardware_lock:
            self.pipeline.close()
