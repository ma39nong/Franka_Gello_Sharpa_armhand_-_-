"""Client for the long-lived preset IK server used by Operator GUI Q/W/E."""

from __future__ import annotations

import json
import os
import socket
import time

from .relative_action import PresetAction, SolvedRelativeAction

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5591


def invoke_preset_ik(
    preset: PresetAction,
    *,
    max_joint_speed: float,
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    connect_timeout: float = 20.0,
    timeout: float = 90.0,
) -> SolvedRelativeAction:
    """Ask the warm MoveIt IK server to solve one recorded relative action."""
    host = os.environ.get("PRESET_IK_HOST", host)
    port = int(os.environ.get("PRESET_IK_PORT", str(port)))
    payload = {
        "id": 1,
        "command": "solve",
        "arguments": {
            "side": preset.side,
            "action": preset.action_name,
            "speed_scale": preset.speed_scale,
            "max_joint_speed": max_joint_speed,
        },
    }
    sock = _connect(host, port, connect_timeout)
    try:
        sock.settimeout(timeout)
        sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        line = _readline(sock)
    finally:
        sock.close()
    try:
        response = json.loads(line)
    except json.JSONDecodeError as error:
        raise RuntimeError(f"preset IK server returned invalid JSON: {line[:200]}") from error
    if not response.get("ok"):
        raise RuntimeError(str(response.get("error") or "preset IK failed"))
    result = response.get("result")
    if not isinstance(result, dict):
        raise RuntimeError("preset IK server returned no result")
    return SolvedRelativeAction.from_dict(result)


def _connect(host: str, port: int, connect_timeout: float) -> socket.socket:
    deadline = time.monotonic() + connect_timeout
    last_error: OSError | None = None
    while True:
        try:
            return socket.create_connection((host, port), timeout=2.0)
        except OSError as error:
            last_error = error
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"preset IK server is not listening on {host}:{port}"
                ) from last_error
            time.sleep(0.1)


def _readline(sock: socket.socket) -> str:
    chunks: list[bytes] = []
    while True:
        piece = sock.recv(4096)
        if not piece:
            break
        chunks.append(piece)
        if b"\n" in piece:
            break
    return b"".join(chunks).decode("utf-8").strip()
