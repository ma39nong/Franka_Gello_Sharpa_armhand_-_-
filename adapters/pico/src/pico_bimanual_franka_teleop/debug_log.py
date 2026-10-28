"""Per-tick JSONL logging of the whole arm-following chain.

One row per 100 Hz control tick, capturing every stage between the tracker and
the command: the world-frame tracker pose, engagement, the mapped target, the
commanded configuration and its end-effector pose, and the measured robot
state. With all stages present, an offline analysis can attribute a following
deficit to the specific link that loses it: input, mapping, IK rate limiting,
or the real arm lagging the command.

Arm-follow rows are serialized into a bounded queue. A worker owns all file
writes and flushes; backpressure drops debug rows instead of blocking control.
"""

from __future__ import annotations

import json
import os
import queue
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

from .types import Pose, SIDES


def _pose_record(pose: Pose | None):
    if pose is None:
        return None
    return {
        "p": [round(float(v), 6) for v in pose.position],
        "r": [round(float(v), 6) for v in pin.log3(pose.rotation)],
    }


def _existing_log_path(path: str | Path) -> Path:
    """Require the log's directory to exist already, loudly.

    Creating parents silently has buried recordings twice: a mangled
    multi-line paste glued the next command onto the log argument, and
    mkdir(parents=True) happily materialized the garbage path. The runbook
    (and ops/run/run_teleop.sh) create RUN_DIR first, so a missing parent
    means the argument itself is wrong - refuse at startup.
    """
    resolved = Path(path)
    if not resolved.parent.is_dir():
        raise ValueError(
            f"debug log directory does not exist: {resolved.parent} - "
            "create RUN_DIR first (was the command pasted intact?)"
        )
    return resolved


class FollowDebugLogger:
    """Best-effort arm diagnostics; never wait for disk in ``record``."""

    def __init__(
        self, path: str | Path, flush_every: int = 100, *, queue_capacity: int = 256,
    ) -> None:
        self.path = _existing_log_path(path)
        self.status_path = self.path.with_name(self.path.stem + ".writer_status.json")
        self._flush_every = int(flush_every)
        if self._flush_every <= 0 or queue_capacity <= 0:
            raise ValueError("Debug flush interval and queue capacity must be positive")
        self._queue = queue.Queue(maxsize=queue_capacity)
        self._closed = threading.Event()
        self._close_requested = False
        self._rows = 0
        self.written_rows = 0
        self.dropped_rows = 0
        self.error: str | None = None
        self._failed = False
        self._header = (
            json.dumps(
                {
                    "schema": "follow-debug.v6",
                    "written_at": time.time(),
                    "writer": {"mode": "background", "queue_capacity": queue_capacity},
                    "fields": "t monotonic; q_measured, q_commanded 14 joints; "
                    "per side: engaged, raw_tracker/tracker/target/ee_cmd/"
                    "ee_meas poses with rotations as world-frame rotation "
                    "vectors; ik = {ep m, eo rad, sat 1-based saturated "
                    "joints, lim [joint, margin rad] near position limits} "
                    "for sides the IK stepped; feed = {frame_ts SDK device "
                    "frame timestamp ns, ts SDK motion timestamp ns, seq "
                    "local Motion callback sequence, age s since the last "
                    "new callback, callback_errors rejected callback fields/"
                    "frames, ok snapshot readable, n trackers the SDK listed} "
                    "when the input source reports it",
                }
            )
            + "\n"
        )
        self._thread = threading.Thread(
            target=self._write_rows, name="follow-debug-writer", daemon=True,
        )
        self._thread.start()

    def record(
        self,
        now: float,
        q_measured,
        q_commanded,
        tracker_poses: dict,
        engaged: dict,
        targets: dict,
        ee_poses: dict,
        raw_tracker_poses: dict | None = None,
        measured_ee_poses: dict | None = None,
        ik_diagnostics: dict | None = None,
        feed_state: dict | None = None,
    ) -> None:
        if self._failed or self._closed.is_set():
            return
        if self._queue.full():
            self.dropped_rows += 1
            return
        try:
            row = {
                "t": round(float(now), 4),
                "q_meas": [round(float(v), 5) for v in np.asarray(q_measured)],
                "q_cmd": [round(float(v), 5) for v in np.asarray(q_commanded)],
            }
            if feed_state is not None:
                row["feed"] = feed_state
            raw_tracker_poses = raw_tracker_poses or {}
            measured_ee_poses = measured_ee_poses or {}
            ik_diagnostics = ik_diagnostics or {}
            for side in SIDES:
                row[side] = {
                    "engaged": bool(engaged.get(side, False)),
                    "raw_tracker": _pose_record(raw_tracker_poses.get(side)),
                    "tracker": _pose_record(tracker_poses.get(side)),
                    "target": _pose_record(targets.get(side)),
                    "ee_cmd": _pose_record(ee_poses.get(side)),
                    "ee_meas": _pose_record(measured_ee_poses.get(side)),
                }
                diag = ik_diagnostics.get(side)
                if diag is not None:
                    row[side]["ik"] = {
                        "ep": round(float(diag["position_error"]), 6),
                        "eo": round(float(diag["orientation_error"]), 6),
                        "sat": list(diag["saturated_joints"]),
                        "lim": [
                            [joint, round(margin, 5)]
                            for joint, margin in diag["limit_joints"]
                        ],
                    }
            # Freeze the snapshot before returning: FK buffers and feed data
            # may change on the next tick. The worker only handles strings.
            line = json.dumps(row, separators=(",", ":")) + "\n"
            self._rows += 1
            self._queue.put_nowait(line)
        except queue.Full:
            self._rows -= 1
            self.dropped_rows += 1
        except Exception as error:  # noqa: BLE001 - logging must never break control
            self.dropped_rows += 1
            self.error = str(error)
            self._failed = True
            self._closed.set()

    def close(self) -> None:
        if self._close_requested:
            return
        self._close_requested = True
        self._closed.set()
        # Hardware cleanup happens first. A stuck writer cannot hold up exit.
        self._thread.join(timeout=0.2)

    def _write_status(self, state: str) -> None:
        payload = {
            "schema": "follow-debug-writer.v1", "state": state,
            "updated_at_ns": time.time_ns(), "path": str(self.path),
            "accepted_rows": self._rows, "written_rows": self.written_rows,
            "dropped_rows": self.dropped_rows,
            "pending_rows": self._rows - self.written_rows,
            "queue_capacity": self._queue.maxsize, "error": self.error,
        }
        temporary = self.status_path.with_name(self.status_path.name + ".tmp")
        try:
            temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
            os.replace(temporary, self.status_path)
        except OSError:
            # Disk failures can also prevent saving the diagnostic counters.
            # The worker reports the underlying log error separately.
            pass

    @staticmethod
    def _notice(message: str) -> None:
        try:
            print(f"[follow debug] {message}", file=sys.stderr, flush=True)
        except OSError:
            pass

    def _write_rows(self) -> None:
        reported_drops = 0
        next_status_at = 0.0
        try:
            with self.path.open("w", encoding="utf-8") as stream:
                stream.write(self._header)
                stream.flush()
                while not self._closed.is_set() or not self._queue.empty():
                    try:
                        line = self._queue.get(timeout=0.05)
                    except queue.Empty:
                        line = None
                    if line is not None:
                        stream.write(line)
                        self.written_rows += 1
                        if self.written_rows % self._flush_every == 0:
                            stream.flush()
                    now = time.monotonic()
                    if now >= next_status_at:
                        self._write_status("running")
                        if self.dropped_rows != reported_drops:
                            self._notice(
                                f"Dropped {self.dropped_rows} debug rows; writer queue full. "
                                f"Details: {self.status_path}"
                            )
                            reported_drops = self.dropped_rows
                        next_status_at = now + 5.0
                stream.flush()
        except Exception as error:  # noqa: BLE001 - file I/O stays on the worker
            self.error = str(error)
            self._failed = True
            self._closed.set()
        finally:
            if self._failed:
                self._notice(f"Logging disabled: {self.error}")
            elif self.dropped_rows != reported_drops:
                self._notice(f"Dropped {self.dropped_rows} debug rows. Details: {self.status_path}")
            self._write_status("failed" if self._failed else "closed")


class HandRetargetDebugLogger:
    """Buffered JSONL logger for replayable skeleton-to-hardware analysis."""

    def __init__(
        self,
        path: str | Path,
        flush_every: int = 60,
        *,
        metadata: dict | None = None,
    ) -> None:
        self.path = _existing_log_path(path)
        self._file = self.path.open("w", encoding="utf-8")
        self._flush_every = int(flush_every)
        self._rows = 0
        self._failed = False
        self._file.write(
            json.dumps(
                {
                    "schema": "hand-retarget-debug.v2",
                    "written_at": time.time(),
                    "metadata": {} if metadata is None else metadata,
                    "fields": "wall_time_ns and t monotonic; source frame identity; "
                    "raw source skeleton; canonical landmarks in metres; raw "
                    "solver and emitted named qpos in radians; UDP packet identity; "
                    "solver fidelity stats",
                }
            )
            + "\n"
        )

    def record(
        self,
        now: float,
        side: str,
        landmarks,
        qpos,
        stats: dict,
        *,
        joint_names=None,
        raw_qpos=None,
        source: dict | None = None,
        transport: dict | None = None,
        derived: dict | None = None,
    ) -> None:
        if self._failed:
            return
        try:
            row = {
                "wall_time_ns": time.time_ns(),
                "t": round(float(now), 6),
                "side": str(side),
                "source": source,
                "landmarks": np.asarray(landmarks, dtype=float)
                .round(7)
                .tolist(),
                "joint_names": (
                    None if joint_names is None else [str(name) for name in joint_names]
                ),
                "raw_qpos": (
                    None
                    if raw_qpos is None
                    else np.asarray(raw_qpos, dtype=float).round(6).tolist()
                ),
                "qpos": np.asarray(qpos, dtype=float).round(6).tolist(),
                "transport": transport,
                "derived": derived,
                "stats": {
                    key: (
                        bool(value)
                        if isinstance(value, (bool, np.bool_))
                        else (
                            int(value)
                            if isinstance(value, (int, np.integer))
                            else round(float(value), 7)
                        )
                    )
                    for key, value in stats.items()
                },
            }
            self._file.write(json.dumps(row, separators=(",", ":")) + "\n")
            self._rows += 1
            if self._rows % self._flush_every == 0:
                self._file.flush()
        except Exception:  # noqa: BLE001 - logging must never break control
            self._failed = True
            try:
                self._file.close()
            except Exception:  # noqa: BLE001
                pass

    def close(self) -> None:
        if not self._failed:
            try:
                self._file.flush()
                self._file.close()
            except Exception:  # noqa: BLE001
                pass
