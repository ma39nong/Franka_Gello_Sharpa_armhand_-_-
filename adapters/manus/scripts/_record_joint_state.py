"""Record JointState topics to JSONL, stamped with the host wall clock.

Runs under the system ROS interpreter (`/opt/ros/jazzy`), not the teleop conda
env, which has no rclpy. `teleop_manus_hands_debug.py` starts it as a child and
merges its output afterwards; it is not meant to be run by hand.

Every row carries `wall_time_ns` from `time.time_ns()` -- the same clock the
teleop debug log stamps its rows with, on the same machine -- which is what
makes the two streams mergeable. The message's own header stamp is kept beside
it so a clock disagreement would be visible rather than silent.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState


class Recorder(Node):
    def __init__(self, topics: dict[str, str], sink):
        super().__init__("manus_debug_joint_state_recorder")
        # NOT `self.handle`: rclpy.Node.handle is a read-only property and
        # assigning to it raises at construction.
        self.sink = sink
        self.counts = {side: 0 for side in topics}
        # A BEST_EFFORT subscription matches both a best-effort and a reliable
        # publisher, so it cannot fail to match on a QoS mismatch. The driver
        # currently publishes with the default (reliable) profile.
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        for side, topic in topics.items():
            self.create_subscription(
                JointState, topic, self._make_cb(side), qos)
            print(f"  recorder: subscribed {side} -> {topic}", flush=True)

    def _make_cb(self, side: str):
        def cb(msg: JointState) -> None:
            stamp = msg.header.stamp
            row = {
                "wall_time_ns": time.time_ns(),
                "header_ns": int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
                "side": side,
                "name": list(msg.name),
                "position": [float(v) for v in msg.position],
            }
            self.sink.write(json.dumps(row) + "\n")
            self.counts[side] += 1
            if sum(self.counts.values()) % 30 == 0:
                self.sink.flush()
        return cb


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--left-topic", default="")
    ap.add_argument("--right-topic", default="")
    args = ap.parse_args()

    topics = {s: t for s, t in (("left", args.left_topic),
                                ("right", args.right_topic)) if t}
    if not topics:
        print("  recorder: no topics requested; exiting", flush=True)
        return 0

    rclpy.init()
    with open(args.out, "w") as handle:
        node = Recorder(topics, handle)
        stopping = {"now": False}

        def stop(*_):
            stopping["now"] = True
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)

        try:
            while rclpy.ok() and not stopping["now"]:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            counts = dict(node.counts)
            handle.flush()
            node.destroy_node()
            rclpy.shutdown()
    print(f"  recorder: wrote {counts} rows to {args.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
