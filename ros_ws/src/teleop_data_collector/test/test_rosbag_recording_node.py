import json
import socket
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")

from teleop_data_collector import rosbag_recording_node
from teleop_data_collector.rosbag_recording_node import (
    _has_qos_overrides_arg,
    _terminal_success,
    _topic_qos_overrides_yaml,
    _wait_for_required_topics,
)
from teleop_data_collector.collector_control import (
    CollectorController,
    CollectorControlServer,
)


class _Logger:
    def __init__(self):
        self.messages = []
        self.entries = []

    def info(self, message):
        self.messages.append(message)
        self.entries.append(("info", message))

    def warn(self, message):
        self.messages.append(message)
        self.entries.append(("warn", message))

    def error(self, message):
        self.messages.append(message)
        self.entries.append(("error", message))


class _Node:
    def __init__(self, failures):
        self._failures = iter(failures)
        self.calls = 0
        self.logger = _Logger()
        self.now_ns = 5_000_000_000

    def preflight_failures(self):
        self.calls += 1
        return next(self._failures)

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now_ns))


class _Recorder:
    def __init__(self, root: Path):
        self.root = root
        self.active = False
        self.current_bag_dir = None
        self.last_bag_dir = None
        self.milestones = []

    def mark_milestone(self):
        if not self.active:
            raise RuntimeError("no episode is currently recording")
        marker = {"id": "milestone_1", "timestamp_ns": 5_000_000_000, "clock": "ros"}
        self.milestones.append(marker)
        return marker

    def start(self):
        self.current_bag_dir = self.root / "episode0"
        self.current_bag_dir.mkdir()
        self.last_bag_dir = self.current_bag_dir
        self.active = True
        return self.current_bag_dir

    def stop(self, interrupted=False):
        self.active = False
        state = {
            "state": "interrupted" if interrupted else "finalized",
            "finalized": not interrupted,
            "failures": [],
            "transport_warnings": ["synthetic warning"],
        }
        (self.current_bag_dir / "collection_state.json").write_text(
            json.dumps(state), encoding="utf-8"
        )
        bag = self.current_bag_dir
        self.current_bag_dir = None
        return bag

    def mark_discarded(self):
        state = {
            "state": "discarded",
            "finalized": False,
            "failures": [],
        }
        (self.last_bag_dir / "collection_state.json").write_text(
            json.dumps(state), encoding="utf-8"
        )
        return self.last_bag_dir


def test_default_qos_expands_to_each_configured_topic():
    yaml_text = _topic_qos_overrides_yaml(
        topics=("/cam0/color/image_raw", "/cam0/depth/image_raw"),
        qos={"history": "keep_last", "depth": 100},
    )

    assert "/cam0/color/image_raw:" in yaml_text
    assert "/cam0/depth/image_raw:" in yaml_text
    assert yaml_text.count("history: keep_last") == 2
    assert yaml_text.count("depth: 100") == 2


def test_milestones_persist_without_stopping_and_reset_for_next_bag(tmp_path, monkeypatch):
    node = _Node([])
    processes = []

    def popen(*args, **kwargs):
        process = SimpleNamespace(poll=lambda: None, wait=lambda **kw: 0, returncode=0)
        processes.append(process)
        return process

    monkeypatch.setattr(rosbag_recording_node.shutil, "which", lambda _: "/usr/bin/ros2")
    monkeypatch.setattr(rosbag_recording_node.subprocess, "Popen", popen)
    monkeypatch.setattr(rosbag_recording_node, "_signal_process_group", lambda *_: None)
    recorder = rosbag_recording_node.RosbagEpisodeRecorder(
        node, tmp_path, "episode", (), 30.0, (), postflight=lambda _: ((), {}),
    )
    with pytest.raises(RuntimeError, match="no episode"):
        recorder.mark_milestone()
    bag = recorder.start()
    # rosbag creates its directory asynchronously: markers must persist even
    # before it exists, without pre-creating it and breaking ros2 bag record.
    marker = recorder.mark_milestone()
    sidecar = tmp_path / ".episode0.collection_state.json"
    assert json.loads(sidecar.read_text())["milestones"] == [marker]
    assert not bag.exists()
    bag.mkdir()
    node.now_ns += 1_000_000_000
    second = recorder.mark_milestone()
    state = json.loads((bag / "collection_state.json").read_text())
    assert state["milestones"] == [marker, second]
    assert state["state"] == "recording"
    assert recorder.active
    assert len(processes) == 1
    with pytest.raises(RuntimeError, match="increasing"):
        recorder.mark_milestone()
    recorder.stop()
    finalized = json.loads((bag / "collection_state.json").read_text())
    assert finalized["milestones"] == [marker, second]
    assert finalized["finalized"] is True
    recorder.mark_discarded()
    assert json.loads((bag / "collection_state.json").read_text())["milestones"] == [marker, second]
    recorder.start()
    assert recorder.milestones == []
    assert recorder._source_recording_id != state["source_recording_id"]


def test_failed_milestone_write_does_not_report_success(tmp_path, monkeypatch):
    recorder = rosbag_recording_node.RosbagEpisodeRecorder(
        _Node([]), tmp_path, "episode", (), 30.0, (),
    )
    recorder._current_bag_dir = tmp_path / "episode0"
    recorder._process = SimpleNamespace(poll=lambda: None)

    def fail(*args, **kwargs):
        raise OSError("disk full")

    monkeypatch.setattr(recorder, "_write_state", fail)
    with pytest.raises(OSError, match="disk full"):
        recorder.mark_milestone()
    assert recorder.milestones == []


def test_explicit_qos_overrides_arg_is_preserved():
    assert _has_qos_overrides_arg(
        ["--qos-profile-overrides-path", "custom_qos.yaml"]
    )
    assert not _has_qos_overrides_arg(["--storage-preset-profile", "fastwrite"])


def test_explicit_qos_file_prevents_generated_global_override(tmp_path, monkeypatch):
    commands = []

    def popen(command, **kwargs):
        commands.append(command)
        return SimpleNamespace(poll=lambda: None, wait=lambda **kw: 0, returncode=0)

    monkeypatch.setattr(rosbag_recording_node.shutil, "which", lambda _: "/usr/bin/ros2")
    monkeypatch.setattr(rosbag_recording_node.subprocess, "Popen", popen)
    recorder = rosbag_recording_node.RosbagEpisodeRecorder(
        _Node([]),
        tmp_path,
        "episode",
        ("/cam0/color/image_raw",),
        30.0,
        ("--qos-profile-overrides-path", "/config/per_topic.yaml"),
        default_qos={"reliability": "best_effort"},
    )

    recorder.start()

    assert commands[0].count("--qos-profile-overrides-path") == 1
    qos_flag = commands[0].index("--qos-profile-overrides-path")
    assert commands[0][qos_flag + 1] == "/config/per_topic.yaml"
    assert recorder._generated_qos_path is None


def test_success_message_is_green_only_for_an_attached_terminal():
    text = "Bag saved to /collection_data/bags/gello/episode165."
    assert _terminal_success(text, color_enabled=True) == (
        f"\033[1;32m{text}\033[0m"
    )
    assert _terminal_success(text, color_enabled=False) == text


@pytest.mark.parametrize("validator_raises", [False, True])
def test_stop_prints_validation_details_and_preserves_failed_state(tmp_path, monkeypatch, validator_raises):
    node = _Node([])

    def postflight(_path):
        if validator_raises:
            raise ValueError("/cam0: truncated image")
        return ("/cam1: gap exceeds 150ms",), {
            "boundary_warnings": ["/cam2: trimmed leading gap"],
            "transport_warnings": ["/cam0: receive gap"],
        }

    recorder = rosbag_recording_node.RosbagEpisodeRecorder(
        node, tmp_path, "episode", (), 30.0, (), postflight=postflight,
    )
    bag = tmp_path / "episode0"
    bag.mkdir()
    recorder._current_bag_dir = bag
    recorder._process = SimpleNamespace(poll=lambda: None, wait=lambda **kwargs: 0, returncode=0)
    monkeypatch.setattr(rosbag_recording_node, "_signal_process_group", lambda *_: None)
    recorder.stop()
    state = json.loads((bag / "collection_state.json").read_text())
    assert state["state"] == "incomplete"
    assert state["finalized"] is False
    assert any("Validation finished in" in text for text in node.logger.messages)
    for failure in state["failures"]:
        assert f"[validation FAILED] {failure}" in node.logger.messages
        assert ("error", f"[validation FAILED] {failure}") in node.logger.entries
    if not validator_raises:
        assert "[boundary warning] /cam2: trimmed leading gap" in node.logger.messages
        assert "[transport warning] /cam0: receive gap" in node.logger.messages


@pytest.mark.parametrize("has_boundary_warning", [False, True])
def test_successful_stop_preserves_report_and_summarizes_only_transport_warnings(
    tmp_path, monkeypatch, has_boundary_warning,
):
    node = _Node([])
    report = {
        "boundary_warnings": ["/cam2: trimmed leading gap"] if has_boundary_warning else [],
        "transport_warnings": ["/cam0: receive gap", "/cam1: receive gap"],
    }
    recorder = rosbag_recording_node.RosbagEpisodeRecorder(
        node, tmp_path, "episode", (), 30.0, (), postflight=lambda _: ((), report),
    )
    bag = tmp_path / "episode0"
    bag.mkdir()
    recorder._current_bag_dir = bag
    recorder._process = SimpleNamespace(poll=lambda: None, wait=lambda **kwargs: 0, returncode=0)
    monkeypatch.setattr(rosbag_recording_node, "_signal_process_group", lambda *_: None)
    recorder.stop()

    state = json.loads((bag / "collection_state.json").read_text())
    assert state["finalized"] is True
    assert state["failures"] == []
    assert state["validation_report"] == report
    assert state["transport_warnings"] == report["transport_warnings"]
    summaries = [(level, text) for level, text in node.logger.entries if "[transport info]" in text]
    if has_boundary_warning:
        assert not summaries
        assert ("warn", "[boundary warning] /cam2: trimmed leading gap") in node.logger.entries
        assert ("warn", "[transport warning] /cam0: receive gap") in node.logger.entries
    else:
        assert len(summaries) == 1
        level, text = summaries[0]
        assert level == "info"
        assert "2 receive-timing warnings" in text
        assert str(bag / "collection_state.json") in text
        assert not any(level in ("warn", "error") for level, _ in node.logger.entries)


def test_ready_waits_for_required_topics_to_stay_stable(monkeypatch):
    clock = {"now": 0.0}
    node = _Node(
        [
            ("/cam2/color/image_raw: missing",),
            (),
            (),
            (),
        ]
    )
    monkeypatch.setattr(rosbag_recording_node.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        rosbag_recording_node.time,
        "monotonic",
        lambda: clock["now"],
    )
    monkeypatch.setattr(
        rosbag_recording_node.time,
        "sleep",
        lambda seconds: clock.__setitem__("now", clock["now"] + seconds),
    )

    assert _wait_for_required_topics(node, stable_sec=2.0, poll_sec=1.0)
    assert node.calls == 4
    assert any("/cam2/color/image_raw: missing" in text for text in node.logger.messages)


def test_ui_controller_keeps_waiting_start_stop_and_discard_states(tmp_path):
    node = _Node([])
    recorder = _Recorder(tmp_path)
    controller = CollectorController(node, recorder, quality_dir=tmp_path / "数据分类")

    assert controller.status()["state"] == "WAITING"
    with pytest.raises(RuntimeError, match="waiting"):
        controller.start()

    controller.set_ready()
    assert controller.status()["can_start"] is True
    started = controller.start()
    assert started["state"] == "RECORDING"
    assert started["current_episode"] == "episode0"
    assert started["can_stop"] is True
    assert started["can_mark_milestone"] is True
    marked = controller.dispatch("mark_milestone")
    assert marked["state"] == "RECORDING"
    assert len(marked["milestones"]) == 1

    stopped = controller.stop()
    assert stopped["state"] == "FINALIZED"
    assert stopped["finalized"] is True
    assert stopped["transport_warnings"] == ["synthetic warning"]
    assert stopped["can_start"] is True
    assert stopped["can_discard"] is True
    assert stopped["can_mark_milestone"] is False
    with pytest.raises(RuntimeError, match="no episode"):
        controller.dispatch("mark_milestone")
    # The main loop can notice active=False just after the UI stop completes.
    # Reconciliation must be idempotent instead of issuing a second stop.
    assert controller.reconcile_exited_recorder()["state"] == "FINALIZED"

    discarded = controller.discard()
    assert discarded["state"] == "DISCARDED"
    assert discarded["finalized"] is False


def test_main_loop_marks_only_an_unexpected_recorder_exit_interrupted(tmp_path):
    recorder = _Recorder(tmp_path)
    controller = CollectorController(_Node([]), recorder, quality_dir=tmp_path / "数据分类")
    controller.set_ready()
    controller.start()

    # Model rosbag exiting on its own while its episode directory is still
    # current. The main-loop reconciliation owns this path.
    recorder.active = False
    status = controller.reconcile_exited_recorder()

    assert status["state"] == "INTERRUPTED"
    assert status["active"] is False
    assert status["current_episode"] is None


def test_collector_control_server_accepts_line_delimited_local_json(tmp_path):
    controller = CollectorController(_Node([]), _Recorder(tmp_path), quality_dir=tmp_path / "数据分类")
    controller.set_ready()
    server = CollectorControlServer(("127.0.0.1", 0), controller)
    thread = server.start_in_thread()
    try:
        with socket.create_connection(server.server_address, timeout=1.0) as client:
            client.sendall(b'{"id":7,"command":"start","arguments":{}}\n')
            response = json.loads(client.makefile("rb").readline())
        assert response["id"] == 7
        assert response["ok"] is True
        assert response["result"]["state"] == "RECORDING"
    finally:
        controller.stop(interrupted=True)
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.0)
