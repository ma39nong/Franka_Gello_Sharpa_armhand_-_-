#!/usr/bin/env python3
"""Long-lived ROS subscriber that mirrors FR3 joint_states into a JSON cache.

Intended to run inside the Compose ``tools`` container. Never publishes.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def _ordered_arm(names, positions, side: str) -> list[float] | None:
    import re

    found: dict[int, float] = {}
    for name, position in zip(names, positions):
        lowered = str(name).lower()
        if side not in lowered:
            continue
        match = re.search(r"joint([1-7])$", lowered)
        if match is None:
            continue
        value = float(position)
        if not math.isfinite(value):
            return None
        found[int(match.group(1))] = value
    if len(found) != 7:
        return None
    return [found[index] for index in range(1, 8)]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--sides",
        default="left,right",
        help="comma-separated sides to subscribe",
    )
    args = parser.parse_args()
    sides = tuple(side.strip() for side in args.sides.split(",") if side.strip())
    if not sides or any(side not in ("left", "right") for side in sides):
        raise SystemExit(f"invalid --sides: {args.sides}")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    latest: dict[str, object] = {
        "updated_at": None,
        "arms": {side: None for side in sides},
        "faults": {side: "waiting for /{}/franka/joint_states".format(side) for side in sides},
    }

    def write_cache() -> None:
        tmp = args.out.with_suffix(args.out.suffix + ".tmp")
        tmp.write_text(json.dumps(latest, ensure_ascii=False), encoding="utf-8")
        tmp.replace(args.out)

    write_cache()
    rclpy.init()
    node = rclpy.create_node("pose_recorder_arm_watcher")

    def make_cb(side: str):
        def _cb(message: JointState) -> None:
            ordered = _ordered_arm(message.name, message.position, side)
            if ordered is None:
                latest["faults"][side] = f"{side} joint_states incomplete"
                return
            latest["arms"][side] = {
                "names": [f"{side}_fr3v2_joint{index}" for index in range(1, 8)],
                "positions": ordered,
            }
            latest["faults"][side] = None
            latest["updated_at"] = time.time()
            write_cache()

        return _cb

    for side in sides:
        topic = f"/{side}/franka/joint_states"
        node.create_subscription(
            JointState, topic, make_cb(side), qos_profile_sensor_data
        )
        print(f"watching {topic}", flush=True)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
