import json
import socket
from types import SimpleNamespace

import pytest

from teleop_data_collector import quality_lists
from teleop_data_collector.collector_control import CollectorController, CollectorControlServer
from teleop_data_collector.quality_lists import QUALITY_LABELS, save_quality


def test_lists_preserve_numbers_expand_ranges_and_move_rating(tmp_path):
    (tmp_path / "一般.txt").write_text("# 已有分类\n222-224\nepisode226\n", encoding="utf-8")
    save_quality(tmp_path, "episode223", "优等")
    save_quality(tmp_path, "episode223", "优等")
    assert (tmp_path / "优等.txt").read_text() == "223\n"
    assert (tmp_path / "一般.txt").read_text() == "# 已有分类\n222\n224\n226\n"
    save_quality(tmp_path, "episode223", "报错")
    assert (tmp_path / "优等.txt").read_text() == ""
    assert (tmp_path / "报错.txt").read_text() == "223\n"
    assert all((tmp_path / f"{label}.txt").exists() for label in QUALITY_LABELS)


def test_failed_staging_preserves_original_lists_and_can_retry(tmp_path, monkeypatch):
    save_quality(tmp_path, "episode7", "一般")
    before = {p.name: p.read_bytes() for p in tmp_path.glob("*.txt")}
    original = quality_lists.os.fsync

    def disk_full(_):
        raise OSError(28, "No space left on device")

    monkeypatch.setattr(quality_lists.os, "fsync", disk_full)
    with pytest.raises(OSError, match="No space"):
        save_quality(tmp_path, "episode7", "优等")
    assert {p.name: p.read_bytes() for p in tmp_path.glob("*.txt")} == before
    assert not list(tmp_path.glob(".quality-*"))
    monkeypatch.setattr(quality_lists.os, "fsync", original)
    save_quality(tmp_path, "episode7", "优等")
    assert (tmp_path / "优等.txt").read_text() == "7\n"
    assert (tmp_path / "一般.txt").read_text() == ""


def test_malformed_existing_list_is_not_overwritten(tmp_path):
    path = tmp_path / "报错.txt"
    path.write_text("invalid\n")
    with pytest.raises(ValueError, match="格式无效"):
        save_quality(tmp_path, "episode1", "优等")
    assert path.read_text() == "invalid\n"
    assert not (tmp_path / "优等.txt").exists()


class Recorder:
    def __init__(self, root):
        self.root = root
        self.active = False
        self.current_bag_dir = None
        self.last_bag_dir = None
        self.milestones = []
        self.index = 0

    def start(self):
        self.index += 1
        self.current_bag_dir = self.root / f"episode{self.index}"
        self.current_bag_dir.mkdir()
        self.last_bag_dir = self.current_bag_dir
        self.active = True
        return self.current_bag_dir

    def stop(self, interrupted=False):
        self.active = False
        self.current_bag_dir = None
        self._write_state("interrupted" if interrupted else "incomplete")
        return self.last_bag_dir

    def mark_discarded(self):
        self._write_state("discarded")
        return self.last_bag_dir

    def _write_state(self, state):
        (self.last_bag_dir / "collection_state.json").write_text(json.dumps({
            "state": state, "finalized": False, "failures": ["missing camera frames"],
        }))


def make_controller(tmp_path):
    logger = SimpleNamespace(info=lambda _: None, warn=lambda _: None)
    node = SimpleNamespace(get_logger=lambda: logger)
    controller = CollectorController(node, Recorder(tmp_path), quality_dir=tmp_path / "数据分类")
    controller.set_ready()
    return controller


def test_controller_quality_keeps_validation_and_rejects_stale_episode(tmp_path):
    controller = make_controller(tmp_path)
    assert not controller.status()["quality_pending"]
    controller.start()
    with pytest.raises(RuntimeError, match="等待"):
        controller.rate_quality("episode1", "优等")
    status = controller.stop()
    assert status["quality_pending"] and status["can_rate_quality"]
    before = (tmp_path / "episode1/collection_state.json").read_bytes()
    status = controller.rate_quality("episode1", "优等")
    assert status["quality"] == "优等" and not status["quality_pending"]
    assert status["state"] == "INCOMPLETE" and not status["finalized"]
    assert (tmp_path / "episode1/collection_state.json").read_bytes() == before
    controller.start()
    controller.stop()
    with pytest.raises(ValueError, match="编号已变化"):
        controller.rate_quality("episode1", "报错")
    assert (tmp_path / "数据分类/报错.txt").read_text() == ""


@pytest.mark.parametrize("via_button", [True, False])
def test_discard_records_number_and_keeps_bag(tmp_path, via_button):
    controller = make_controller(tmp_path)
    controller.start()
    controller.stop()
    controller.rate_quality("episode1", "一般")
    status = controller.discard() if via_button else controller.rate_quality("episode1", "放弃")
    assert status["quality"] == "放弃" and status["state"] == "DISCARDED"
    assert (tmp_path / "episode1").is_dir()
    assert (tmp_path / "数据分类/一般.txt").read_text() == ""
    assert (tmp_path / "数据分类/放弃.txt").read_text() == "1\n"


def test_quality_command_over_tcp_reports_save_error_and_retries(tmp_path):
    controller = make_controller(tmp_path)
    controller.start()
    controller.stop()
    server = CollectorControlServer(("127.0.0.1", 0), controller)
    thread = server.start_in_thread()
    try:
        with socket.create_connection(server.server_address, timeout=2) as client:
            stream = client.makefile("rb")

            def send(arguments):
                client.sendall((json.dumps({"id": 9, "command": "rate_quality", "arguments": arguments}) + "\n").encode())
                return json.loads(stream.readline())

            assert not send({"episode": "episode1", "quality": "invalid"})["ok"]
            blocked_path = tmp_path / "blocked"
            blocked_path.write_text("not a directory")
            controller._quality_dir = blocked_path
            assert not send({"episode": "episode1", "quality": "报错"})["ok"]
            assert controller.status()["quality_pending"]
            controller._quality_dir = tmp_path / "数据分类"
            response = send({"episode": "episode1", "quality": "报错"})
            assert response["id"] == 9 and response["ok"]
            assert response["result"]["quality"] == "报错"
            assert (tmp_path / "数据分类/报错.txt").read_text() == "1\n"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
