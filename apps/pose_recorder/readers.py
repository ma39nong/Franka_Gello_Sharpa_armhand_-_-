"""Read-only snapshot sources for FR3 arms and Wuji Hand 2."""

from __future__ import annotations

import json
import subprocess
import threading
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
DOCKER_DIR = REPO_ROOT / "docker"
MJCF_DIR = REPO_ROOT / "adapters" / "wuji" / "models" / "hand2_beta" / "mjcf"


def _mjcf_joint_names(side: str) -> list[str]:
    try:
        import mujoco
    except ImportError:
        prefix = "l_" if side == "left" else "r_"
        fingers = (
            "thumb_cmc_flex",
            "thumb_cmc_abd",
            "thumb_mcp",
            "thumb_ip",
            "index_finger_mcp_flex",
            "index_finger_mcp_abd",
            "index_finger_pip",
            "index_finger_dip",
            "middle_finger_mcp_flex",
            "middle_finger_mcp_abd",
            "middle_finger_pip",
            "middle_finger_dip",
            "ring_finger_mcp_flex",
            "ring_finger_mcp_abd",
            "ring_finger_pip",
            "ring_finger_dip",
            "pinky_mcp_flex",
            "pinky_mcp_abd",
            "pinky_pip",
            "pinky_dip",
        )
        return [prefix + name for name in fingers]
    model = mujoco.MjModel.from_xml_path(str(MJCF_DIR / f"{side}.xml"))
    return [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(model.njnt)
    ]


class ArmStateReader:
    """Mirrors FR3 joint_states via a Compose tools watcher. Never publishes."""

    def __init__(self, sides: tuple[str, ...], cache_path: Path) -> None:
        self.sides = sides
        self.cache_path = cache_path
        self._proc: subprocess.Popen[str] | None = None
        self._lock = threading.Lock()

    def start(self) -> None:
        if self._proc is not None:
            return
        if not (DOCKER_DIR / ".env").exists():
            raise RuntimeError(
                f"missing {DOCKER_DIR / '.env'}; copy from docker/.env.example"
            )
        self.cache_path.parent.mkdir(parents=True, exist_ok=True)
        if self.cache_path.exists():
            self.cache_path.unlink()
        watcher = Path("/workspace/franka_upper_body_teleop") / "apps" / "pose_recorder" / "_arm_ros_watcher.py"
        cache = Path("/workspace/franka_upper_body_teleop") / self.cache_path.relative_to(REPO_ROOT)
        self._proc = subprocess.Popen(
            [
                "docker",
                "compose",
                "run",
                "--rm",
                "--no-deps",
                "tools",
                "python3",
                str(watcher),
                "--out",
                str(cache),
                "--sides",
                ",".join(self.sides),
            ],
            cwd=DOCKER_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

    def close(self) -> None:
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            if self._proc is not None and self._proc.poll() is not None:
                output = ""
                if self._proc.stdout is not None:
                    output = self._proc.stdout.read()[-800:]
                return {
                    "ok": False,
                    "fault": (
                        f"arm watcher exited ({self._proc.returncode}): "
                        f"{output.strip() or 'no output'}"
                    ),
                    "sides": {},
                }
            if not self.cache_path.exists():
                return {
                    "ok": False,
                    "fault": "waiting for FR3 joint_states (is franka-control up?)",
                    "sides": {},
                }
            try:
                payload = json.loads(self.cache_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as error:
                return {
                    "ok": False,
                    "fault": f"arm cache unreadable: {error}",
                    "sides": {},
                }
        arms = payload.get("arms") or {}
        faults = payload.get("faults") or {}
        sides_out: dict[str, Any] = {}
        ok = True
        for side in self.sides:
            sample = arms.get(side)
            fault = faults.get(side)
            if sample is None:
                ok = False
                sides_out[side] = {"ok": False, "fault": fault or "no sample yet"}
            else:
                sides_out[side] = {
                    "ok": True,
                    "fault": None,
                    "names": sample["names"],
                    "positions": sample["positions"],
                }
        return {
            "ok": ok,
            "fault": None if ok else "one or more arm sides missing",
            "updated_at": payload.get("updated_at"),
            "sides": sides_out,
        }


class WujiStateReader:
    """Connects to Wuji Hand 2 and only subscribes to joint_states."""

    def __init__(
        self,
        sides: tuple[str, ...],
        addresses: dict[str, str],
        stale_timeout: float = 0.5,
    ) -> None:
        self.sides = sides
        self.addresses = dict(addresses)
        self.stale_timeout = float(stale_timeout)
        self._lock = threading.Lock()
        self._hands: dict[str, Any] = {}
        self._subs: dict[str, Any] = {}
        self._latest: dict[str, dict[str, Any]] = {}
        self._faults: dict[str, str | None] = {side: "not connected" for side in sides}
        self._joint_names = {side: _mjcf_joint_names(side) for side in sides}
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        try:
            from wuji_sdk import SdkManager
        except ImportError as error:
            raise RuntimeError(
                "wuji-sdk is required; use conda env gello-upper-body-teleop"
            ) from error
        manager = SdkManager.instance()
        for side in self.sides:
            address = self.addresses.get(side, "").strip()
            if not address:
                raise ValueError(f"--wuji-{side}-address IP:PORT is required")
            hand = manager.connect(
                address=address, device_name=f"pose_recorder_wuji_{side}"
            )
            try:
                reported = str(hand.handedness().get()).lower()
                if reported != side:
                    raise RuntimeError(
                        f"Wuji hand at {address} reports {reported}, expected {side}"
                    )
                online = int(hand.online_joints_count().get())
                if online == 0:
                    raise RuntimeError(f"{side} Wuji Hand 2 has no online joints")
                sub = hand.joint_states().subscribe()
            except BaseException:
                try:
                    hand.disconnect()
                except Exception:
                    pass
                self.close()
                raise
            self._hands[side] = hand
            self._subs[side] = sub
            self._faults[side] = None
            print(
                f"{side} Wuji Hand 2 connected read-only at {address} "
                f"(online_joints={online})",
                flush=True,
            )
        self._thread = threading.Thread(target=self._poll_loop, name="wuji-state", daemon=True)
        self._thread.start()

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            for side, sub in list(self._subs.items()):
                try:
                    frame = sub.recv()
                except Exception as error:
                    with self._lock:
                        self._faults[side] = f"recv failed: {error}"
                    continue
                if frame is None:
                    continue
                entries = []
                by_nid: dict[int, float] = {}
                for entry in frame.joints:
                    nid = int(entry.nid)
                    position = float(entry.position)
                    by_nid[nid] = position
                    entries.append(
                        {
                            "nid": nid,
                            "position": position,
                            "velocity": float(entry.velocity),
                            "effort": float(entry.effort),
                        }
                    )
                entries.sort(key=lambda item: item["nid"])
                names = self._joint_names[side]
                ordered_nids = sorted(by_nid)
                if len(by_nid) == len(names) and ordered_nids == list(range(len(names))):
                    positions = [by_nid[index] for index in range(len(names))]
                    named = names
                else:
                    positions = [item["position"] for item in entries]
                    named = [f"nid_{item['nid']}" for item in entries]
                with self._lock:
                    self._latest[side] = {
                        "ok": True,
                        "fault": None,
                        "names": named,
                        "positions": positions,
                        "joints": entries,
                        "updated_at": time.time(),
                    }
                    self._faults[side] = None
            time.sleep(0.01)

    def close(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
            self._thread = None
        for side, sub in list(self._subs.items()):
            try:
                sub.close()
            except Exception:
                pass
            self._subs.pop(side, None)
        for side, hand in list(self._hands.items()):
            try:
                hand.disconnect()
            except Exception:
                pass
            self._hands.pop(side, None)

    def snapshot(self) -> dict[str, Any]:
        now = time.time()
        sides_out: dict[str, Any] = {}
        ok = True
        with self._lock:
            for side in self.sides:
                sample = self._latest.get(side)
                fault = self._faults.get(side)
                if sample is None:
                    ok = False
                    sides_out[side] = {
                        "ok": False,
                        "fault": fault or "no joint_states yet",
                    }
                    continue
                age = now - float(sample["updated_at"])
                if age > self.stale_timeout:
                    ok = False
                    sides_out[side] = {
                        "ok": False,
                        "fault": f"stale joint_states ({age:.2f}s)",
                        "names": sample["names"],
                        "positions": sample["positions"],
                        "joints": sample["joints"],
                        "age_s": age,
                    }
                    continue
                sides_out[side] = {
                    "ok": True,
                    "fault": None,
                    "names": sample["names"],
                    "positions": sample["positions"],
                    "joints": sample["joints"],
                    "age_s": age,
                }
        return {
            "ok": ok,
            "fault": None if ok else "one or more hand sides missing/stale",
            "sides": sides_out,
        }


class PoseRecorder:
    def __init__(
        self,
        *,
        sides: tuple[str, ...],
        output: Path,
        wuji_addresses: dict[str, str],
        record_arms: bool = True,
        record_hands: bool = True,
        cache_dir: Path | None = None,
    ) -> None:
        self.sides = sides
        self.output = output
        self.record_arms = record_arms
        self.record_hands = record_hands
        self._lock = threading.Lock()
        self._count = 0
        if self.output.exists():
            with self.output.open("r", encoding="utf-8") as handle:
                self._count = sum(1 for line in handle if line.strip())
        cache_root = cache_dir or (REPO_ROOT / "apps" / "pose_recorder" / ".cache")
        self.arm_reader = (
            ArmStateReader(sides, cache_root / "arm_state.json")
            if record_arms
            else None
        )
        self.hand_reader = (
            WujiStateReader(sides, wuji_addresses)
            if record_hands
            else None
        )

    def start(self) -> None:
        self.output.parent.mkdir(parents=True, exist_ok=True)
        if self.arm_reader is not None:
            self.arm_reader.start()
        if self.hand_reader is not None:
            self.hand_reader.start()

    def close(self) -> None:
        if self.hand_reader is not None:
            self.hand_reader.close()
        if self.arm_reader is not None:
            self.arm_reader.close()

    def live(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "sides": list(self.sides),
            "output": str(self.output.resolve()),
            "record_count": self._count,
            "arms": None,
            "hands": None,
        }
        if self.arm_reader is not None:
            payload["arms"] = self.arm_reader.snapshot()
        if self.hand_reader is not None:
            payload["hands"] = self.hand_reader.snapshot()
        return payload

    def record(self, note: str) -> dict[str, Any]:
        live = self.live()
        entry = {
            "timestamp": time.time(),
            "iso_time": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
            "note": note.strip(),
            "sides": list(self.sides),
            "arms": live.get("arms"),
            "hands": live.get("hands"),
        }
        line = json.dumps(entry, ensure_ascii=False)
        with self._lock:
            with self.output.open("a", encoding="utf-8") as handle:
                handle.write(line + "\n")
            self._count += 1
            count = self._count
        return {
            "ok": True,
            "record_count": count,
            "output": str(self.output.resolve()),
            "entry": entry,
        }
