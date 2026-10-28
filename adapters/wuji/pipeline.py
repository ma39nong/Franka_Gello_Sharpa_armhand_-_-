"""MANUS-to-Wuji pipeline compatible with the existing HandWorker."""

from __future__ import annotations

import json
import time
from pathlib import Path

import mujoco
import numpy as np
import yaml

from manus_teleop.pipeline import ManusBridge, canonical_landmarks
from pico_bimanual_franka_teleop.hand_sender import HandStatus
from .wuji_retargeting import Retargeter

from .backend import WujiHand2Backend, WujiHandBackend

ROOT = Path(__file__).resolve().parent
MODELS = ("wuji_hand", "wuji_hand_2")

try:
    from teleop_core.contract import (
        WUJI_LEFT_JOINT_NAMES,
        WUJI_RIGHT_JOINT_NAMES,
        require_exact_joint_names,
    )
except ModuleNotFoundError:
    import sys

    sys.path.insert(0, str(ROOT.parents[1] / "ros_ws" / "src" / "teleop_core"))
    from teleop_core.contract import (
        WUJI_LEFT_JOINT_NAMES,
        WUJI_RIGHT_JOINT_NAMES,
        require_exact_joint_names,
    )


def _config_path(side: str, model: str) -> Path:
    infix = "_wuji_hand_2" if model == "wuji_hand_2" else ""
    return ROOT / "config" / f"retarget_manus{infix}_{side}.yaml"


def _device_permutation(retargeter: Retargeter, config_path: Path) -> np.ndarray:
    """Map optimizer qpos order to the MJCF/device joint order.

    The real Wuji Hand 2 path uses the MJCF joint order as the firmware command
    order, matching ``wuji-retargeting/example/teleop_real.py``.  Loading the
    model through MuJoCo is intentional: XML traversal order is not guaranteed
    to equal the compiled model's qpos order.
    """
    config = yaml.safe_load(config_path.read_text()) or {}
    relative = (config.get("optimizer") or {}).get("mjcf_path")
    source_names = list(retargeter.optimizer.robot.dof_joint_names)
    if not relative:
        return np.arange(len(source_names), dtype=int)
    mjcf = (config_path.parent / relative).resolve()
    model = mujoco.MjModel.from_xml_path(str(mjcf))
    destination_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(model.njnt)
    ]
    by_name = {name: index for index, name in enumerate(source_names)}
    try:
        permutation = np.asarray(
            [by_name[name] for name in destination_names], dtype=int
        )
    except KeyError as error:
        raise ValueError(
            f"URDF/MJCF Wuji joint order mismatch: missing {error.args[0]}"
        ) from error
    if len(permutation) != 20 or len(set(permutation.tolist())) != 20:
        raise ValueError("Wuji device joint order must contain 20 unique joints")
    return permutation


def _device_joint_names(retargeter: Retargeter, permutation: np.ndarray) -> tuple[str, ...]:
    source = tuple(retargeter.optimizer.robot.dof_joint_names)
    return tuple(source[index] for index in permutation)


class WujiHandPipeline:
    """Drive optional Wuji hands while Gello continues to own only the arms."""

    def __init__(
        self,
        *,
        sides=("left", "right"),
        models=None,
        addresses=None,
        serials=None,
        rate: float = 30.0,
        stale_timeout: float = 0.25,
        kp: float = 8.0,
        kd: float = 0.1,
        current_limit: float = 1.0,
        library: Path | None = None,
        calibration_dir: Path | None = None,
        debug_log: Path | None = None,
        auto_enable: bool = True,
    ) -> None:
        self.sides = tuple(sides)
        if not self.sides or set(self.sides).difference({"left", "right"}):
            raise ValueError(f"invalid Wuji sides: {self.sides}")
        self.models = dict(models or {side: "wuji_hand_2" for side in self.sides})
        if set(self.models) != set(self.sides) or any(
            model not in MODELS for model in self.models.values()
        ):
            raise ValueError(f"models must map every side to one of {MODELS}")
        self.feedback_sides = tuple(
            side for side in self.sides if self.models[side] == "wuji_hand_2"
        )
        self.addresses = dict(addresses or {})
        self.serials = dict(serials or {})
        if not 0.0 < rate <= 60.0:
            raise ValueError("Wuji command rate must be in (0, 60] Hz")
        if stale_timeout <= 0.0:
            raise ValueError("Wuji stale timeout must be positive")
        self.interval = 1.0 / float(rate)
        self.stale_timeout = float(stale_timeout)
        self.status = HandStatus()
        self.retargeters = {}
        self.permutations = {}
        self.joint_names = {}
        self.joint_limits = {}
        self.backends = {}
        self.last_frames = {side: None for side in self.sides}
        self.last_frame_at = {side: None for side in self.sides}
        self.last_feedback = {side: None for side in self.sides}
        # Read-only observation of commands that were actually accepted by the
        # SDK backend. HandWorker consumes this cache for telemetry; it never
        # feeds back into command generation.
        self.last_commands = {side: None for side in self.sides}
        self.next_due = {side: 0.0 for side in self.sides}
        self.open_until = {side: 0.0 for side in self.sides}
        self.was_following = {side: False for side in self.sides}
        self.was_opening = {side: False for side in self.sides}
        self.pose_moves = {side: None for side in self.sides}
        self._debug = None
        self.bridge = None

        try:
            for side in self.sides:
                path = _config_path(side, self.models[side])
                retargeter = Retargeter.from_yaml(str(path), side)
                self.retargeters[side] = retargeter
                self.permutations[side] = _device_permutation(retargeter, path)
                self.joint_names[side] = _device_joint_names(
                    retargeter, self.permutations[side]
                )
                require_exact_joint_names(
                    self.joint_names[side],
                    WUJI_LEFT_JOINT_NAMES if side == "left" else WUJI_RIGHT_JOINT_NAMES,
                    label=f"{side} Wuji device order",
                )
                lower = np.asarray(
                    retargeter.optimizer.robot.model.lowerPositionLimit,
                    dtype=np.float64,
                )[self.permutations[side]]
                upper = np.asarray(
                    retargeter.optimizer.robot.model.upperPositionLimit,
                    dtype=np.float64,
                )[self.permutations[side]]
                self.joint_limits[side] = tuple(
                    (float(low), float(high))
                    for low, high in zip(lower, upper, strict=True)
                )
                print(
                    f"{side} Wuji qpos mapping ({path.name}): "
                    f"URDF -> device {self.permutations[side].tolist()}"
                )

            repo = ROOT.parents[1]
            bridge_library = library or (
                repo / "adapters/manus/build/libmanus_skeleton_bridge.so"
            )
            calibrations = calibration_dir or (
                repo / "adapters/manus/config"
            )
            self.bridge = ManusBridge(Path(bridge_library).resolve())
            self.bridge.connect(Path(calibrations).resolve())

            for side in self.sides:
                if self.models[side] == "wuji_hand_2":
                    self.backends[side] = WujiHand2Backend(
                        side=side,
                        address=self.addresses.get(side, ""),
                        kp=kp,
                        kd=kd,
                        current_limit=current_limit,
                        auto_enable=auto_enable,
                    )
                else:
                    self.backends[side] = WujiHandBackend(
                        self.serials.get(side, "")
                    )
            if debug_log is not None:
                debug_path = Path(debug_log)
                debug_path.parent.mkdir(parents=True, exist_ok=True)
                self._debug = debug_path.open("a", encoding="utf-8")
        except BaseException:
            self.close()
            raise

    def set_enabled(self, side: str, enabled: bool) -> None:
        if side not in self.sides:
            raise ValueError(f"Wuji side is not configured: {side}")
        backend = self.backends[side]
        operation = getattr(backend, "enable" if enabled else "disable", None)
        if operation is None:
            raise RuntimeError(
                f"runtime enable switching is unavailable for {self.models[side]}"
            )
        operation()

    def feedback_position(self, side: str) -> np.ndarray | None:
        if side not in self.sides:
            raise ValueError(f"Wuji side is not configured: {side}")
        reader = getattr(self.backends[side], "read_position", None)
        if reader is None:
            raise RuntimeError(
                f"position feedback is unavailable for {self.models[side]}"
            )
        latest = reader()
        if latest is not None:
            self.last_feedback[side] = np.asarray(latest, dtype=np.float64).copy()
        return latest

    def joint_diagnostics(self, side: str):
        if side not in self.sides:
            raise ValueError(f"Wuji side is not configured: {side}")
        reader = getattr(self.backends[side], "read_diagnostics", None)
        if reader is None:
            raise RuntimeError(
                f"joint diagnostics are unavailable for {self.models[side]}"
            )
        return reader()

    def mit_gains(self, side: str):
        if side not in self.sides:
            raise ValueError(f"Wuji side is not configured: {side}")
        reader = getattr(self.backends[side], "mit_gains", None)
        if reader is None:
            raise RuntimeError(
                f"MIT gain access is unavailable for {self.models[side]}"
            )
        return reader()

    def set_mit_gains(
        self, side: str, *, kp: float, kd: float, joint_index: int | None = None
    ):
        if side not in self.sides:
            raise ValueError(f"Wuji side is not configured: {side}")
        setter = getattr(self.backends[side], "set_mit_gains", None)
        if setter is None:
            raise RuntimeError(
                f"MIT gain tuning is unavailable for {self.models[side]}"
            )
        return setter(kp=kp, kd=kd, joint_index=joint_index)

    def request_open(
        self,
        now: float | None = None,
        duration: float = 2.0,
        sides=None,
    ) -> None:
        moment = time.monotonic() if now is None else float(now)
        for side in self.sides if sides is None else sides:
            self.open_until[side] = moment + float(duration)
            self.pose_moves[side] = None

    def request_pose(
        self,
        positions: dict[str, tuple[float, ...]],
        *,
        max_speed: float = 0.5,
    ) -> None:
        """Schedule smooth measured-position moves for Wuji Hand 2."""
        moment = time.monotonic()
        if not np.isfinite(max_speed) or max_speed <= 0.0:
            raise ValueError("Wuji hand Home speed must be positive")
        prepared = {}
        for side, raw_target in positions.items():
            if side not in self.sides or self.models[side] != "wuji_hand_2":
                raise ValueError(f"Wuji Hand 2 pose is unavailable for {side}")
            target = np.asarray(raw_target, dtype=np.float64)
            if target.shape != (20,) or not np.isfinite(target).all():
                raise ValueError("Wuji Hand 2 pose must contain 20 finite positions")
            limits = self.joint_limits[side]
            if any(
                value < lower or value > upper
                for value, (lower, upper) in zip(target, limits, strict=True)
            ):
                raise ValueError(f"{side} Wuji Hand 2 Home exceeds joint limits")
            start = self.feedback_position(side)
            if start is None:
                start = self.last_feedback[side]
            if start is None:
                start = self.backends[side].last_command_position
            if start is None:
                raise RuntimeError(f"{side} Wuji Hand 2 has no pose to start from")
            distance = float(np.max(np.abs(target - start)))
            duration = 0.0 if distance == 0.0 else max(0.5, 1.875 * distance / max_speed)
            prepared[side] = {
                "start": np.asarray(start, dtype=np.float64),
                "target": target,
                "started_at": moment,
                "duration": duration,
            }
        for side, move in prepared.items():
            self.open_until[side] = 0.0
            self.pose_moves[side] = move

    def cancel_pose(self) -> None:
        for side in self.sides:
            self.pose_moves[side] = None

    def _open_pose(self, side: str) -> np.ndarray:
        robot = self.retargeters[side].optimizer.robot
        lower = np.asarray(robot.model.lowerPositionLimit, dtype=float)
        upper = np.asarray(robot.model.upperPositionLimit, dtype=float)
        return np.clip(np.zeros_like(lower), lower, upper)[self.permutations[side]]

    def tick(self, now: float | None = None, active=None) -> None:
        moment = time.monotonic() if now is None else float(now)
        enabled = dict(active or {})
        for side in self.sides:
            status = self.status.sides[side]
            try:
                frame = self.bridge.read(side, 0.0)
                if frame is not None:
                    self.last_frames[side] = frame
                    self.last_frame_at[side] = moment
            except Exception as error:
                self.status.errors += 1
                self.status.last_error = f"{side} MANUS read failed: {error}"
                status.sending = False
                status.fault = self.status.last_error
                continue

            following = bool(enabled.get(side, False))
            pose_move = self.pose_moves[side]
            if pose_move is not None:
                following = False
            opening = pose_move is None and moment < self.open_until[side]
            if (
                (self.was_following[side] and not following)
                or (opening and not self.was_opening[side])
            ):
                reset = getattr(self.retargeters[side], "reset", None)
                if reset is not None:
                    reset()
            self.was_following[side] = following
            self.was_opening[side] = opening
            if not following and not opening and pose_move is None:
                status.sending = False
                status.fault = "disengaged"
                continue
            if moment < self.next_due[side]:
                continue

            try:
                if pose_move is not None:
                    duration = float(pose_move["duration"])
                    progress = (
                        1.0
                        if duration == 0.0
                        else min(1.0, (moment - pose_move["started_at"]) / duration)
                    )
                    blend = progress**3 * (
                        progress * (progress * 6.0 - 15.0) + 10.0
                    )
                    command = pose_move["start"] + blend * (
                        pose_move["target"] - pose_move["start"]
                    )
                elif opening:
                    command = self._open_pose(side)
                else:
                    age = (
                        float("inf")
                        if self.last_frame_at[side] is None
                        else moment - self.last_frame_at[side]
                    )
                    if self.last_frames[side] is None or age > self.stale_timeout:
                        reset = getattr(self.retargeters[side], "reset", None)
                        if reset is not None:
                            reset()
                        status.sending = False
                        status.fault = "MANUS skeleton stale"
                        continue
                    landmarks = canonical_landmarks(self.last_frames[side])
                    started = time.monotonic()
                    qpos = self.retargeters[side].retarget(landmarks)
                    status.solve_seconds = time.monotonic() - started
                    command = qpos[self.permutations[side]]
                self.backends[side].send(command)
                sent_at = time.monotonic()
                self.last_commands[side] = {
                    "joint_names": tuple(self.joint_names[side]),
                    "positions": tuple(float(value) for value in command),
                    "monotonic_ns": int(sent_at * 1_000_000_000),
                    "wall_time_ns": time.time_ns(),
                    "engaged": bool(following),
                    "valid": True,
                }
                if pose_move is not None and progress >= 1.0:
                    self.pose_moves[side] = None
                self.next_due[side] = moment + self.interval
                status.sending = True
                status.fault = None
                status.sent += 1
                if self._debug is not None:
                    self._debug.write(
                        json.dumps(
                            {
                                "time": time.time(),
                                "side": side,
                                "model": self.models[side],
                                "opening": opening,
                                "qpos": np.asarray(command).tolist(),
                            }
                        )
                        + "\n"
                    )
                    self._debug.flush()
            except Exception as error:
                self.status.errors += 1
                self.status.last_error = f"{side} Wuji command failed: {error}"
                status.sending = False
                status.fault = self.status.last_error

    def command_snapshot(self, side: str) -> dict | None:
        """Return the latest command only after the hardware send succeeded."""
        if side not in self.sides:
            raise ValueError(f"Wuji side is not configured: {side}")
        snapshot = self.last_commands[side]
        return None if snapshot is None else dict(snapshot)

    def close(self) -> None:
        for backend in list(getattr(self, "backends", {}).values()):
            try:
                backend.close()
            except Exception:
                pass
        self.backends = {}
        bridge = getattr(self, "bridge", None)
        if bridge is not None:
            try:
                bridge.close()
            except Exception:
                pass
            self.bridge = None
        debug = getattr(self, "_debug", None)
        if debug is not None:
            debug.close()
            self._debug = None
