"""Long-lived MoveIt IK helper for Operator GUI preset actions.

Keeps ArmRosRuntime warm so Q/W/E does not spawn `docker compose run tools`
on the Franka realtime CPUs.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import socketserver
import threading

from .preset_ik_cli import _wait_until_ready, build_preset_solution
from .recording import ActionStore, RecordedAction
from .runtime import ArmRosRuntime


class PresetIkSolver:
    def __init__(self, data_root: Path) -> None:
        self.store = ActionStore(data_root)
        self.runtime = ArmRosRuntime(robot_type="fr3", service_timeout_sec=8.0)
        self._lock = threading.Lock()
        self._cache: dict[tuple[str, str], tuple[float, int, RecordedAction]] = {}
        self._ready = threading.Event()
        self._warmup_error: BaseException | None = None
        threading.Thread(target=self._warmup, name="preset-ik-warmup", daemon=True).start()

    def _warmup(self) -> None:
        try:
            _wait_until_ready(self.runtime)
            print("preset IK server MoveIt warmup complete", flush=True)
        except BaseException as error:  # noqa: BLE001 - surface to the next Q/W/E
            self._warmup_error = error
        finally:
            self._ready.set()

    def close(self) -> None:
        self.runtime.shutdown()

    def _load_action(self, side: str, name: str) -> RecordedAction:
        path = self.store.path_for(side, name)
        stat = path.stat()
        key = (side, name)
        cached = self._cache.get(key)
        if (
            cached is not None
            and cached[0] == stat.st_mtime
            and cached[1] == stat.st_size
        ):
            return cached[2]
        action = self.store.load(side, name)
        self._cache[key] = (stat.st_mtime, stat.st_size, action)
        return action

    def solve(
        self,
        *,
        side: str,
        action: str,
        speed_scale: float,
        max_joint_speed: float,
    ) -> dict[str, object]:
        if not self._ready.wait(timeout=25.0):
            raise RuntimeError("MoveIt IK is not ready")
        if self._warmup_error is not None:
            raise RuntimeError(f"robot state/FK not ready: {self._warmup_error}")
        with self._lock:
            recorded = self._load_action(side, action)
            return build_preset_solution(
                self.runtime,
                recorded,
                speed_scale=speed_scale,
                max_joint_speed=max_joint_speed,
            )


class _Handler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        line = self.rfile.readline()
        if not line:
            return
        try:
            request = json.loads(line)
            arguments = request.get("arguments") or {}
            result = self.server.solver.solve(
                side=str(arguments.get("side", "")),
                action=str(arguments.get("action", "")),
                speed_scale=float(arguments.get("speed_scale", 0.5)),
                max_joint_speed=float(arguments.get("max_joint_speed", 0.5)),
            )
            response = {"id": request.get("id"), "ok": True, "result": result}
        except Exception as error:  # noqa: BLE001 - report to the operator process
            response = {
                "id": None,
                "ok": False,
                "error": str(error),
            }
            try:
                response["id"] = json.loads(line).get("id")
            except Exception:  # noqa: BLE001
                pass
        self.wfile.write((json.dumps(response) + "\n").encode("utf-8"))
        self.wfile.flush()


class PresetIkServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int], solver: PresetIkSolver) -> None:
        super().__init__(address, _Handler)
        self.solver = solver


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", default="/data/arm_ui")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5591)
    args = parser.parse_args()
    try:
        os.nice(10)
    except OSError:
        pass
    print(
        f"preset IK server starting on {args.host}:{args.port}; waiting for MoveIt",
        flush=True,
    )
    solver = PresetIkSolver(Path(args.data_root))
    server = PresetIkServer((args.host, args.port), solver)

    def _stop(*_args: object) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    print(f"preset IK server listening on {args.host}:{args.port}", flush=True)
    try:
        server.serve_forever(poll_interval=0.5)
    finally:
        server.server_close()
        solver.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
