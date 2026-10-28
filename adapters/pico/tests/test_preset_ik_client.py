from pathlib import Path
import json
import socketserver
import threading

import pytest

from pico_bimanual_franka_teleop.preset_ik_client import invoke_preset_ik
from pico_bimanual_franka_teleop.relative_action import PresetAction


def _preset() -> PresetAction:
    return PresetAction(
        key="q",
        label="Left kuai1",
        side="left",
        action_name="kuai1",
        path=Path("/data/arm_ui/actions/left__kuai1.yaml"),
        speed_scale=0.5,
    )


def _valid_result() -> dict[str, object]:
    return {
        "name": "kuai1",
        "side": "left",
        "time_sec": [0.0, 0.1, 0.2],
        "positions": [[0.0] * 7, [0.01] * 7, [0.02] * 7],
        "start_q": [0.0] * 14,
        "speed_scale": 0.5,
    }


class _Handler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        request = json.loads(self.rfile.readline())
        self.server.requests.append(request)
        if self.server.error:
            payload = {
                "id": request.get("id"),
                "ok": False,
                "error": self.server.error,
            }
        else:
            payload = {
                "id": request.get("id"),
                "ok": True,
                "result": _valid_result(),
            }
        self.wfile.write((json.dumps(payload) + "\n").encode("utf-8"))
        self.wfile.flush()


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, error: str | None = None) -> None:
        super().__init__(("127.0.0.1", 0), _Handler)
        self.requests: list[dict] = []
        self.error = error


def _serve(error: str | None = None):
    server = _Server(error=error)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    host, port = server.server_address[:2]
    try:
        yield host, port, server
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.0)


@pytest.fixture
def ik_server():
    yield from _serve()


def test_invoke_preset_ik_reads_one_json_response(ik_server, monkeypatch) -> None:
    host, port, server = ik_server
    monkeypatch.delenv("PRESET_IK_HOST", raising=False)
    monkeypatch.delenv("PRESET_IK_PORT", raising=False)

    solution = invoke_preset_ik(
        _preset(),
        max_joint_speed=0.4,
        host=host,
        port=port,
        connect_timeout=1.0,
    )

    assert solution.name == "kuai1"
    assert solution.side == "left"
    assert solution.speed_scale == 0.5
    assert server.requests[0]["command"] == "solve"
    assert server.requests[0]["arguments"] == {
        "side": "left",
        "action": "kuai1",
        "speed_scale": 0.5,
        "max_joint_speed": 0.4,
    }


def test_invoke_preset_ik_raises_server_error(monkeypatch) -> None:
    monkeypatch.delenv("PRESET_IK_HOST", raising=False)
    monkeypatch.delenv("PRESET_IK_PORT", raising=False)
    for host, port, _server in _serve(error="first IK frame is 6.0 deg from the measured start"):
        with pytest.raises(RuntimeError, match="6.0 deg"):
            invoke_preset_ik(
                _preset(),
                max_joint_speed=0.5,
                host=host,
                port=port,
                connect_timeout=1.0,
            )


def test_invoke_preset_ik_requires_listening_server(monkeypatch) -> None:
    monkeypatch.delenv("PRESET_IK_HOST", raising=False)
    monkeypatch.delenv("PRESET_IK_PORT", raising=False)
    with pytest.raises(RuntimeError, match="not listening"):
        invoke_preset_ik(
            _preset(),
            max_joint_speed=0.5,
            host="127.0.0.1",
            port=1,
            connect_timeout=0.2,
        )
