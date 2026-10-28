#!/usr/bin/env python3
"""Read-only check of right O30i communication.

Checks:
  1. USB-CANFD adapter presence (VID:PID a8fa:8598)
  2. Fresh ROS feedback on /cb_right_hand_state (via docker compose tools)

Never opens the CAN bus itself and never publishes a hand command, so it can
run beside a live hand-control stack. If hand-control is down, the USB check
still runs and the ROS step reports that no state publisher is available.

    cd /home/descfly/llx/gello_upper_body_teleop
    python ops/diagnostics/check_right_hand_o30i.py
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DOCKER_DIR = REPO_ROOT / "docker"
EXPECTED_USB = ("a8fa", "8598")
STATE_TOPIC = "/cb_right_hand_state"


def _print_pass(message: str) -> None:
    print(f"[PASS] {message}", flush=True)


def _print_fail(message: str) -> None:
    print(f"[FAIL] {message}", flush=True)


def check_usb_adapter() -> bool:
    vendor, product = EXPECTED_USB
    try:
        completed = subprocess.run(
            ["lsusb"],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        _print_fail("lsusb is not available on this host")
        return False
    if completed.returncode != 0:
        _print_fail(f"lsusb failed: {completed.stderr.strip() or completed.stdout.strip()}")
        return False
    needle = f"{vendor}:{product}".lower()
    matches = [
        line.strip()
        for line in completed.stdout.splitlines()
        if needle in line.lower()
    ]
    if not matches:
        _print_fail(
            f"USB adapter {vendor}:{product} not found "
            "(expected Com Equipment CANFD Analyser for right O30i)"
        )
        return False
    _print_pass(f"USB adapter present: {matches[0]}")
    return True


def _compose_tools(command: str, *, timeout: float) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            "docker",
            "compose",
            "run",
            "--rm",
            "--no-deps",
            "tools",
            "bash",
            "-lc",
            command,
        ],
        cwd=DOCKER_DIR,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def check_ros_state(duration: float) -> bool:
    if not (DOCKER_DIR / ".env").exists():
        _print_fail(f"missing {DOCKER_DIR / '.env'}; copy from docker/.env.example")
        return False

    # topic info first: distinguishes "stack down" from "topic silent".
    try:
        info = _compose_tools(
            f"ros2 topic info {STATE_TOPIC}",
            timeout=60.0,
        )
    except subprocess.TimeoutExpired:
        _print_fail("timed out querying ROS topic info via docker compose tools")
        return False
    except FileNotFoundError:
        _print_fail("docker is not available")
        return False

    combined_info = (info.stdout or "") + (info.stderr or "")
    if info.returncode != 0:
        _print_fail(
            f"{STATE_TOPIC} is unavailable "
            f"(is hand-control running?): {combined_info.strip()[-400:]}"
        )
        return False

    publisher_match = re.search(r"Publisher count:\s*(\d+)", combined_info)
    publisher_count = int(publisher_match.group(1)) if publisher_match else 0
    if publisher_count < 1:
        _print_fail(
            f"{STATE_TOPIC} has no publishers; start hand-control "
            "(right O30i driver) before rechecking feedback"
        )
        return False
    _print_pass(f"{STATE_TOPIC} has {publisher_count} publisher(s)")

    # Sample messages without sending commands. Count finite-position frames.
    sample_script = f"""
import math
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("check_right_hand_o30i")
count = 0
last_names = ()
last_n = 0

def _cb(msg: JointState) -> None:
    global count, last_names, last_n
    if not msg.position:
        return
    if any(not math.isfinite(float(value)) for value in msg.position):
        return
    count += 1
    last_names = tuple(msg.name)
    last_n = len(msg.position)

sub = node.create_subscription(
    JointState, "{STATE_TOPIC}", _cb, qos_profile_sensor_data
)
deadline = time.monotonic() + {duration:.3f}
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
node.destroy_subscription(sub)
node.destroy_node()
rclpy.shutdown()
hz = count / {duration:.3f} if {duration:.3f} > 0 else 0.0
print(f"FRAMES={{count}} HZ={{hz:.1f}} DOF={{last_n}} NAMES={{len(last_names)}}")
"""
    try:
        sample = _compose_tools(
            "python3 - <<'PY'\n" + sample_script + "\nPY",
            timeout=max(90.0, duration + 60.0),
        )
    except subprocess.TimeoutExpired:
        _print_fail("timed out sampling right-hand state")
        return False

    output = ((sample.stdout or "") + (sample.stderr or "")).strip()
    frame_match = re.search(
        r"FRAMES=(\d+)\s+HZ=([0-9.]+)\s+DOF=(\d+)\s+NAMES=(\d+)",
        output,
    )
    if sample.returncode != 0 or frame_match is None:
        _print_fail(
            "could not sample right-hand state: "
            + (output[-500:] or f"exit {sample.returncode}")
        )
        return False

    frames = int(frame_match.group(1))
    hz = float(frame_match.group(2))
    dof = int(frame_match.group(3))
    if frames <= 0:
        _print_fail(
            f"no fresh finite feedback on {STATE_TOPIC} over {duration:.1f}s "
            "(USB may be present, but O30i state is no-state / stale)"
        )
        return False
    if dof < 1:
        _print_fail(f"received {frames} frames but position vectors were empty")
        return False
    _print_pass(
        f"right O30i feedback fresh: {frames} frames in {duration:.1f}s "
        f"({hz:.1f} Hz, {dof} DoF)"
    )
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="seconds to sample /cb_right_hand_state (default: 3)",
    )
    parser.add_argument(
        "--skip-ros",
        action="store_true",
        help="only check the USB adapter; do not query ROS feedback",
    )
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("--duration must be positive")

    print(
        "Right O30i communication check "
        "(read-only; no hand or robot command is sent)."
    )
    failures = 0
    if not check_usb_adapter():
        failures += 1
    if not args.skip_ros and not check_ros_state(args.duration):
        failures += 1

    if failures:
        print(
            f"Right-hand check failed with {failures} issue(s).",
            file=sys.stderr,
        )
        return 1
    print("Right-hand communication check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
