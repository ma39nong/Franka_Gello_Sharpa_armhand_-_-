"""CSV loading and threaded playback for Linker Hand control."""

from __future__ import annotations

import csv
import threading
import time
from collections.abc import Sequence
from pathlib import Path

from barmate.hardware.linker_hand.control.controller import ManualHandController
from barmate.hardware.linker_hand.control.types import (
    DEFAULT_CLOSED_VALUE,
    HandPlaybackFrame,
    clamp_hand_value,
)


class HandPlaybackService:
    """Threaded dual-hand playback service independent of the browser UI."""

    def __init__(
        self,
        controller: ManualHandController,
        *,
        time_fn=time.monotonic,
        sleep_fn=time.sleep,
    ) -> None:
        self.controller = controller
        self.time_fn = time_fn
        self.sleep_fn = sleep_fn
        self.records: list[HandPlaybackFrame] = []
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._error: BaseException | None = None
        self.poll_interval = 0.01

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def error(self) -> BaseException | None:
        return self._error

    def load_csv(self, path: str | Path) -> list[HandPlaybackFrame]:
        """Load dual hand playback records from a file or episode directory."""

        loaded = load_hand_playback_records(
            path,
            left_joint_count=self.controller.joint_count("left")
            if "left" in self.controller.hand_names
            else 0,
            right_joint_count=self.controller.joint_count("right")
            if "right" in self.controller.hand_names
            else 0,
        )
        self.records = loaded
        return loaded

    def start(
        self,
        records: Sequence[HandPlaybackFrame] | None = None,
        *,
        blocking: bool = False,
        start_wall_time: float | None = None,
    ) -> None:
        """Start playback in its own thread, or synchronously when ``blocking`` is true."""

        if records is not None:
            self.records = list(records)
        if not self.records:
            raise ValueError("Playback records are empty")
        self.stop()
        self._stop_event.clear()
        self._error = None
        if blocking:
            self._run(start_wall_time=start_wall_time)
            return
        self._thread = threading.Thread(
            target=self._run,
            kwargs={"start_wall_time": start_wall_time},
            name="linker-hand-playback",
            daemon=True,
        )
        self._thread.start()

    def start_csv(self, path: str | Path) -> list[HandPlaybackFrame]:
        records = self.load_csv(path)
        self.start(records)
        return records

    def stop(self) -> None:
        with self._lock:
            thread = self._thread
            if thread is None:
                return
            self._stop_event.set()
        thread.join(timeout=1.0)
        if not thread.is_alive():
            self._thread = None

    def _run(self, *, start_wall_time: float | None = None) -> None:
        self._error = None

        try:
            if start_wall_time is not None:
                while True:
                    if self._stop_event.is_set():
                        return
                    wait_time = float(start_wall_time) - self.time_fn()
                    if wait_time <= 0.0:
                        break
                    self.sleep_fn(min(wait_time, self.poll_interval))

            started_at = self.time_fn()
            for frame in self.records:
                while True:
                    if self._stop_event.is_set():
                        return

                    elapsed = self.time_fn() - started_at
                    wait_time = frame.timestamp - elapsed

                    if wait_time <= 0.0:
                        break

                    self.sleep_fn(min(wait_time, self.poll_interval))

                self.controller.apply_hand_positions(
                    {
                        "left": frame.left,
                        "right": frame.right,
                    }
                )

        except Exception as exc:
            self._error = exc


def load_hand_playback_records(
    path: str | Path,
    *,
    left_joint_count: int,
    right_joint_count: int,
) -> list[HandPlaybackFrame]:
    """Load hand playback records from a dual-pose CSV or sidecar CSV directory."""

    source = Path(path)
    if source.is_dir():
        return _load_sidecar_hand_records(source, left_joint_count, right_joint_count)
    return _load_dual_hand_pose_records(source, left_joint_count, right_joint_count)


def _load_dual_hand_pose_records(
    file_path: Path,
    left_joint_count: int,
    right_joint_count: int,
) -> list[HandPlaybackFrame]:
    records: list[HandPlaybackFrame] = []
    with file_path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.reader(csv_file)
        header = next(reader, None)
        if not header or header[0] != "elapsed_time_s":
            raise ValueError("not a dual hand pose csv")
        expected_columns = 1 + left_joint_count + right_joint_count
        if len(header) < expected_columns:
            raise ValueError(
                "pose csv column count is smaller than current hand configuration"
            )
        for row in reader:
            if not row:
                continue
            timestamp = float(row[0])
            left = _normalize_pose(row[1 : 1 + left_joint_count], left_joint_count)
            right = _normalize_pose(
                row[1 + left_joint_count : 1 + left_joint_count + right_joint_count],
                right_joint_count,
            )
            records.append(
                HandPlaybackFrame(timestamp=timestamp, left=left, right=right)
            )
    return records


def _load_sidecar_hand_records(
    directory: Path,
    left_joint_count: int,
    right_joint_count: int,
) -> list[HandPlaybackFrame]:
    left_rows = _load_sidecar_file(directory / "left_hand.csv", left_joint_count)
    right_rows = _load_sidecar_file(directory / "right_hand.csv", right_joint_count)
    records: list[HandPlaybackFrame] = []
    for index, (timestamp, left) in enumerate(left_rows):
        if index >= len(right_rows):
            break
        _, right = right_rows[index]
        records.append(HandPlaybackFrame(timestamp=timestamp, left=left, right=right))
    return records


def _load_sidecar_file(
    file_path: Path, joint_count: int
) -> list[tuple[float, tuple[int, ...]]]:
    rows: list[tuple[float, tuple[int, ...]]] = []
    with file_path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.reader(csv_file)
        for row in reader:
            if not row or row[0].startswith("#") or row[0] == "timestamp":
                continue
            rows.append(
                (float(row[0]), _normalize_pose(row[1 : 1 + joint_count], joint_count))
            )
    return rows


def _normalize_pose(values: Sequence[object], target_len: int) -> tuple[int, ...]:
    pose = [clamp_hand_value(value) for value in values]
    if len(pose) < target_len:
        pose.extend([DEFAULT_CLOSED_VALUE] * (target_len - len(pose)))
    return tuple(pose[:target_len])
