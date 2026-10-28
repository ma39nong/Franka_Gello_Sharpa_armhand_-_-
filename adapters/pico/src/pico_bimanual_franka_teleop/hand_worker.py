import threading
import time
import math

from .types import SIDES


class HandWorker:
    """Run a hand pipeline outside the deadline-critical arm loop."""

    def __init__(
        self,
        pipeline,
        tick_rate: float = 100.0,
        telemetry_sender=None,
        telemetry_stale_timeout: float = 0.15,
    ) -> None:
        if tick_rate <= 0:
            raise ValueError("Hand worker tick rate must be positive")
        self.pipeline = pipeline
        self.sides = tuple(pipeline.sides)
        self.status = pipeline.status
        self.dt = 1.0 / float(tick_rate)
        self._active = {side: False for side in SIDES}
        self._open_requests: list[tuple[tuple[str, ...] | None, float]] = []
        self._pose_requests: list[dict[str, tuple[float, ...]]] = []
        self._cancel_pose_requested = False
        self._feedback: dict[str, tuple[tuple[float, ...], float]] = {}
        self.telemetry_sender = telemetry_sender
        if telemetry_stale_timeout <= 0.0:
            raise ValueError("Telemetry stale timeout must be positive")
        self.telemetry_stale_timeout = float(telemetry_stale_timeout)
        self._telemetry_signatures = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._closed = False

    def start(self) -> None:
        if self._closed:
            raise RuntimeError("Hand worker is closed")
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run, name="hand-teleop", daemon=True
        )
        self._thread.start()

    def set_active(self, active: dict[str, bool]) -> None:
        with self._lock:
            self._active = {
                side: bool(active.get(side, False)) for side in SIDES
            }

    def request_open(
        self, sides: tuple[str, ...] | None = None, duration: float = 2.0
    ) -> None:
        selected = None if sides is None else tuple(sides)
        if duration <= 0.0:
            raise ValueError("Open duration must be positive")
        if selected is not None and set(selected).difference(self.sides):
            raise ValueError(f"Invalid hand sides: {selected}")
        with self._lock:
            for side in self.sides if selected is None else selected:
                self._active[side] = False
            self._open_requests.append((selected, float(duration)))

    def feedback_position(
        self, side: str, *, max_age: float = 0.5
    ) -> tuple[float, ...]:
        if side not in self.sides:
            raise ValueError(f"Invalid hand side: {side}")
        with self._lock:
            snapshot = self._feedback.get(side)
        if snapshot is None:
            raise RuntimeError(f"{side} Wuji Hand 2 feedback is unavailable")
        positions, received_at = snapshot
        age = time.monotonic() - received_at
        if age > max_age:
            raise RuntimeError(
                f"{side} Wuji Hand 2 feedback is stale ({age:.2f}s)"
            )
        return positions

    def feedback_snapshot(
        self, side: str
    ) -> tuple[tuple[float, ...], float] | None:
        """Return cached positions and their monotonic receipt time.

        Unlike ``feedback_position``, this exposes a stale sample so a dataset
        recorder can preserve timing and mark validity without fabricating data.
        """
        if side not in self.sides:
            raise ValueError(f"Invalid hand side: {side}")
        with self._lock:
            snapshot = self._feedback.get(side)
        if snapshot is None:
            return None
        positions, received_at = snapshot
        return tuple(positions), float(received_at)

    def request_pose(self, positions: dict[str, tuple[float, ...]]) -> None:
        selected = {side: tuple(values) for side, values in positions.items()}
        if not selected or set(selected).difference(self.sides):
            raise ValueError(f"Invalid hand pose sides: {tuple(selected)}")
        if any(
            len(values) != 20 or not all(math.isfinite(value) for value in values)
            for values in selected.values()
        ):
            raise ValueError("Wuji Hand 2 target must contain 20 finite positions")
        with self._lock:
            for side in selected:
                self._active[side] = False
            self._pose_requests.append(selected)

    def cancel_pose(self) -> None:
        with self._lock:
            self._pose_requests = []
            self._cancel_pose_requested = True

    def _snapshot(self):
        with self._lock:
            active = dict(self._active)
            requests = self._open_requests
            self._open_requests = []
            pose_requests = self._pose_requests
            self._pose_requests = []
            cancel_pose = self._cancel_pose_requested
            self._cancel_pose_requested = False
        return active, requests, pose_requests, cancel_pose

    def _run(self) -> None:
        deadline = time.monotonic()
        while not self._stop.is_set():
            active, requests, pose_requests, cancel_pose = self._snapshot()
            if cancel_pose:
                cancel = getattr(self.pipeline, "cancel_pose", None)
                if cancel is not None:
                    cancel()
            for sides, duration in requests:
                self.pipeline.request_open(sides=sides, duration=duration)
            for positions in pose_requests:
                try:
                    self.pipeline.request_pose(positions)
                except Exception as error:  # noqa: BLE001 - contain worker errors
                    self.status.errors += 1
                    self.status.last_error = f"hand Home request failed: {error}"
            self.pipeline.tick(active=active)
            feedback_reader = getattr(self.pipeline, "feedback_position", None)
            if feedback_reader is not None:
                for side in getattr(self.pipeline, "feedback_sides", self.sides):
                    try:
                        positions = feedback_reader(side)
                    except Exception as error:  # noqa: BLE001
                        self.status.errors += 1
                        self.status.last_error = f"{side} hand feedback failed: {error}"
                        continue
                    if positions is None:
                        continue
                    values = tuple(float(value) for value in positions)
                    if len(values) == 20 and all(math.isfinite(value) for value in values):
                        with self._lock:
                            self._feedback[side] = (values, time.monotonic())
            self._emit_telemetry(active)
            deadline += self.dt
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                deadline = time.monotonic()
                continue
            self._stop.wait(remaining)

    def _emit_telemetry(self, active: dict[str, bool]) -> None:
        sender = self.telemetry_sender
        if sender is None:
            return
        command_reader = getattr(self.pipeline, "command_snapshot", None)
        joint_names = getattr(self.pipeline, "joint_names", {})
        now = time.monotonic()
        now_ns = int(now * 1_000_000_000)
        wall_ns = time.time_ns()
        for side in self.sides:
            try:
                command = None if command_reader is None else command_reader(side)
                with self._lock:
                    feedback = self._feedback.get(side)
                command_time_ns = (
                    None if command is None else int(command["monotonic_ns"])
                )
                state_time_ns = (
                    None if feedback is None else int(feedback[1] * 1_000_000_000)
                )
                command_valid = bool(
                    command is not None
                    and command.get("valid", False)
                    and now_ns - command_time_ns
                    <= int(self.telemetry_stale_timeout * 1_000_000_000)
                )
                state_valid = bool(
                    feedback is not None
                    and now_ns - state_time_ns
                    <= int(self.telemetry_stale_timeout * 1_000_000_000)
                )
                signature = (
                    command_time_ns,
                    state_time_ns,
                    bool(active.get(side, False)),
                    command_valid,
                    state_valid,
                )
                if signature == self._telemetry_signatures.get(side):
                    continue
                self._telemetry_signatures[side] = signature
                sender.offer(
                    side=side,
                    source_wall_time_ns=wall_ns,
                    source_monotonic_ns=now_ns,
                    joint_names=joint_names[side],
                    command=(
                        None if command is None else command["positions"]
                    ),
                    command_monotonic_ns=command_time_ns,
                    state=(None if feedback is None else feedback[0]),
                    state_monotonic_ns=state_time_ns,
                    engaged=bool(active.get(side, False)),
                    command_valid=command_valid,
                    state_valid=state_valid,
                )
            except Exception:
                # Telemetry is intentionally lossy and cannot fault hand or arm
                # control.  UdpTelemetrySender accounts for queue/send errors.
                continue

    def close(self) -> None:
        if self._closed:
            return
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, 5.0 * self.dt))
            if self._thread.is_alive():
                raise RuntimeError("Hand worker did not stop")
        telemetry_sender = self.telemetry_sender
        self.telemetry_sender = None
        if telemetry_sender is not None:
            try:
                telemetry_sender.close()
            except Exception:
                pass
        self.pipeline.close()
        self._closed = True
