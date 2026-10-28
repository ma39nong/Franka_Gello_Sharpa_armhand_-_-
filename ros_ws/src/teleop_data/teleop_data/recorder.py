import argparse
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node

from .config import load_config, validate_topics
from .storage import (
    BagRecorder,
    bag_topic_counts,
    next_episode_path,
    pending_path,
    remove_path,
    write_manifest,
)


class EpisodeRecorder(Node):
    def __init__(self, config_path, data_root, qos_path):
        super().__init__("teleop_episode_recorder")
        self.config_path = Path(config_path)
        self.config = load_config(config_path)
        self.data_root = Path(data_root)
        self.qos_path = Path(qos_path)
        self.data_root.mkdir(parents=True, exist_ok=True)
        self.recorder = None
        self.pending = None

    def start(self):
        if self.recorder is not None or self.pending is not None:
            raise RuntimeError("Stop and save or discard the current episode first.")
        errors, warnings = validate_topics(
            self.config.topics, dict(self.get_topic_names_and_types())
        )
        for warning in warnings:
            self.get_logger().warn(warning)
        if errors:
            raise RuntimeError("Required topics unavailable: " + "; ".join(errors))
        self.pending = pending_path(self.data_root)
        self.recorder = BagRecorder(
            self.pending,
            [spec.topic for spec in self.config.topics],
            self.config.storage_id,
            self.qos_path,
        )
        self.recorder.start()
        time.sleep(1.0)
        if self.recorder.process.poll() is not None:
            code = self.recorder.process.returncode
            self.recorder = None
            remove_path(self.pending)
            self.pending = None
            raise RuntimeError(f"rosbag failed to start with code {code}.")

    def stop(self):
        if self.recorder is None:
            return
        recorder = self.recorder
        self.recorder = None
        recorder.stop()

    def save(self):
        self.stop()
        if self.pending is None:
            raise RuntimeError("There is no pending episode.")
        counts = bag_topic_counts(self.pending)
        empty = [
            spec.topic
            for spec in self.config.topics
            if spec.required and counts.get(spec.topic, 0) == 0
        ]
        if empty:
            raise RuntimeError(
                "Required topics contain no recorded messages: " + ", ".join(empty)
            )
        destination = next_episode_path(self.data_root)
        self.pending.rename(destination)
        write_manifest(
            destination / "manifest.json",
            {
                "schema_version": self.config.schema_version,
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "recording_config": str(self.config_path),
                "topic_message_counts": counts,
            },
        )
        self.pending = None
        return destination

    def discard(self):
        self.stop()
        if self.pending is None:
            raise RuntimeError("There is no pending episode.")
        remove_path(self.pending)
        self.pending = None


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--qos", type=Path, required=True)
    return parser.parse_args()


def main():
    options = parse_args()
    config = load_config(options.config)
    rclpy.init()
    node = EpisodeRecorder(
        options.config,
        config.data_root,
        options.qos,
    )
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    print("Commands: start, stop, save, discard, status, quit", flush=True)
    try:
        while True:
            command = input("record> ").strip().lower()
            try:
                if command == "start":
                    node.start()
                    print("Recording.", flush=True)
                elif command == "stop":
                    node.stop()
                    print("Stopped.", flush=True)
                elif command == "save":
                    print(f"Saved {node.save()}.", flush=True)
                elif command == "discard":
                    node.discard()
                    print("Discarded.", flush=True)
                elif command == "status":
                    state = "recording" if node.recorder else "pending" if node.pending else "idle"
                    print(state, flush=True)
                elif command == "quit":
                    break
                else:
                    print("Commands: start, stop, save, discard, status, quit")
            except Exception as exc:
                print(f"Error: {exc}", flush=True)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        node.stop()
        rclpy.shutdown()
        thread.join(timeout=5)
        node.destroy_node()
