import json
import io
from pathlib import Path
import threading

import numpy as np
import pytest

from pico_bimanual_franka_teleop.debug_log import (
    FollowDebugLogger,
    HandRetargetDebugLogger,
)
from pico_bimanual_franka_teleop.types import Pose


def test_follow_debug_log_records_raw_and_both_fk_streams(tmp_path):
    path = tmp_path / "follow.jsonl"
    identity = Pose(np.array([1.0, 2.0, 3.0]), np.eye(3))
    logger = FollowDebugLogger(path, flush_every=1)
    logger.record(
        12.5,
        np.arange(14),
        np.arange(14) + 0.5,
        {"left": identity},
        {"left": True},
        {"left": identity},
        {"left": identity},
        raw_tracker_poses={"left": identity},
        measured_ee_poses={"left": identity},
        ik_diagnostics={
            "left": {
                "position_error": 0.0123456789,
                "orientation_error": 0.05,
                "saturated_joints": (2, 4),
                "limit_joints": ((4, 0.012345),),
            }
        },
    )
    logger.close()

    header, row = [
        json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()
    ]
    assert header["schema"] == "follow-debug.v6"
    assert row["left"]["raw_tracker"]["p"] == [1.0, 2.0, 3.0]
    assert row["left"]["tracker"]["p"] == [1.0, 2.0, 3.0]
    assert row["left"]["ee_cmd"]["p"] == [1.0, 2.0, 3.0]
    # The v1 compatibility duplicate is gone; readers use ee_cmd.
    assert "ee" not in row["left"]
    assert row["left"]["ee_meas"]["p"] == [1.0, 2.0, 3.0]
    assert row["right"]["raw_tracker"] is None
    assert row["left"]["ik"] == {
        "ep": 0.012346,
        "eo": 0.05,
        "sat": [2, 4],
        "lim": [[4, 0.01235]],
    }
    assert "ik" not in row["right"]
    status = json.loads(logger.status_path.read_text())
    assert status["state"] == "closed"
    assert status["written_rows"] == 1
    assert status["dropped_rows"] == 0


def test_hand_debug_log_records_landmarks_commands_and_metrics(tmp_path):
    path = tmp_path / "hands.jsonl"
    logger = HandRetargetDebugLogger(
        path, flush_every=1, metadata={"source": "manus"}
    )
    logger.record(
        4.2,
        "right",
        np.zeros((21, 3)),
        np.arange(21) / 10,
        {"success": True, "iterations": 3, "thumb_tip_error": 0.012345678},
        joint_names=[f"joint_{index}" for index in range(21)],
        raw_qpos=np.arange(21) / 9,
        source={"sequence": 7, "timestamp_ns": 123},
        transport={"stream_id": "manus-right-o30i", "sequence": 4, "sent": True},
    )
    logger.close()

    header, row = [
        json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()
    ]
    assert header["schema"] == "hand-retarget-debug.v2"
    assert header["metadata"]["source"] == "manus"
    assert row["side"] == "right"
    assert isinstance(row["wall_time_ns"], int)
    assert row["source"]["sequence"] == 7
    assert row["transport"]["sequence"] == 4
    assert np.asarray(row["landmarks"]).shape == (21, 3)
    assert len(row["qpos"]) == 21
    assert len(row["raw_qpos"]) == 21
    assert len(row["joint_names"]) == 21
    assert row["stats"]["success"] is True
    assert row["stats"]["iterations"] == 3
    assert row["stats"]["thumb_tip_error"] == 0.0123457


def _record_follow(logger, now=0.0, q=None, feed=None):
    q = np.zeros(14) if q is None else q
    logger.record(now, q, q, {}, {}, {}, {}, feed_state=feed)


@pytest.mark.parametrize("blocked_operation", ["write", "flush"])
def test_follow_disk_stall_does_not_block_record_or_grow_queue(
    tmp_path, monkeypatch, blocked_operation,
):
    path = tmp_path / "follow.jsonl"
    blocked = threading.Event()
    release = threading.Event()
    io_threads = []

    class SlowFile(io.StringIO):
        def __exit__(self, *args):
            return False  # Keep the simulated disk contents readable by the test.

        def write(self, text):
            io_threads.append(threading.get_ident())
            if blocked_operation == "write" and '"q_meas"' in text and not blocked.is_set():
                blocked.set()
                assert release.wait(5), "Test did not release simulated disk write"
            return super().write(text)

        def flush(self):
            io_threads.append(threading.get_ident())
            if blocked_operation == "flush" and '"q_meas"' in self.getvalue() and not blocked.is_set():
                blocked.set()
                assert release.wait(5), "Test did not release simulated disk flush"
            return super().flush()

    disk = SlowFile()
    original_open = Path.open
    monkeypatch.setattr(
        Path, "open",
        lambda self, *args, **kwargs: disk if self == path else original_open(self, *args, **kwargs),
    )
    logger = FollowDebugLogger(path, flush_every=1, queue_capacity=4)
    producer = None
    closer = None
    try:
        _record_follow(logger)
        assert blocked.wait(2)
        q = np.zeros(14)
        feed = {"left": {"samples": 0}}

        def produce():
            for index in range(1, 21):
                q[:] = index
                feed["left"]["samples"] = index
                _record_follow(logger, index, q, feed)
            q[:] = -999
            feed["left"]["samples"] = -999

        producer = threading.Thread(target=produce, daemon=True)
        producer.start()
        producer.join(timeout=1)
        assert not producer.is_alive(), "Control-side logging waited for disk"
        assert logger._queue.qsize() == 4
        assert logger.dropped_rows == 16
        closer = threading.Thread(target=logger.close, daemon=True)
        closer.start()
        closer.join(timeout=1)
        assert not closer.is_alive(), "Logger shutdown waited indefinitely for disk"
        assert logger._thread.is_alive()
    finally:
        release.set()
        if producer is not None:
            producer.join(timeout=2)
        if closer is not None:
            closer.join(timeout=2)
        logger.close()
        logger._thread.join(timeout=2)

    assert not logger._thread.is_alive()
    rows = [json.loads(line) for line in disk.getvalue().splitlines()][1:]
    assert [row["t"] for row in rows] == [0, 1, 2, 3, 4]
    for index, row in enumerate(rows[1:], 1):
        assert row["q_meas"] == [index] * 14
        assert row["feed"]["left"]["samples"] == index
    assert set(io_threads) == {logger._thread.ident}
    status = json.loads(logger.status_path.read_text())
    assert status["accepted_rows"] == status["written_rows"] == 5
    assert status["pending_rows"] == 0
    assert status["dropped_rows"] == 16
    assert status["state"] == "closed"


@pytest.mark.parametrize("failed_operation", ["open", "write", "flush"])
def test_follow_io_failure_disables_only_debug_logging(tmp_path, monkeypatch, failed_operation):
    path = tmp_path / "follow.jsonl"
    original_open = Path.open

    class FailedFile(io.StringIO):
        def write(self, text):
            if failed_operation == "write":
                raise OSError("simulated disk failure")
            return super().write(text)

        def flush(self):
            if failed_operation == "flush":
                raise OSError("simulated disk failure")
            return super().flush()

    def open_file(self, *args, **kwargs):
        if self != path:
            return original_open(self, *args, **kwargs)
        if failed_operation == "open":
            raise OSError("simulated disk failure")
        return FailedFile()

    monkeypatch.setattr(Path, "open", open_file)
    logger = FollowDebugLogger(path)
    logger._thread.join(timeout=2)
    assert logger._failed
    for index in range(10):
        _record_follow(logger, index)
    logger.close()
    status = json.loads(logger.status_path.read_text())
    assert status["state"] == "failed"
    assert "simulated disk failure" in status["error"]


def test_follow_invalid_row_disables_logger_without_closing_file_on_caller(tmp_path):
    logger = FollowDebugLogger(tmp_path / "follow.jsonl")
    _record_follow(logger, feed={"not_json_serializable": object()})
    logger.close()
    assert logger._failed
    assert json.loads(logger.status_path.read_text())["state"] == "failed"
