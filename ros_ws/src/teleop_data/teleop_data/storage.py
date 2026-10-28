import json
import os
import re
import shutil
import signal
import subprocess
import threading
import uuid
from pathlib import Path

import yaml


class BagRecorder:
    def __init__(self, output, topics, storage_id, qos_overrides):
        self.output = Path(output)
        self.topics = list(topics)
        self.storage_id = str(storage_id)
        self.qos_overrides = Path(qos_overrides)
        if not self.qos_overrides.is_file():
            raise ValueError(f"Recording QoS config is missing: {self.qos_overrides}")
        self.process = None
        self.lock = threading.Lock()

    def start(self):
        if self.process is not None:
            raise RuntimeError("Recorder is already running.")
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.process = subprocess.Popen(
            [
                "ros2",
                "bag",
                "record",
                "--storage",
                self.storage_id,
                "--output",
                str(self.output),
                "--qos-profile-overrides-path",
                str(self.qos_overrides),
                *self.topics,
            ],
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )

    def stop(self):
        with self.lock:
            process = self.process
            if process is None:
                return
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            self.process = None
            if process.returncode != 0:
                raise RuntimeError(f"rosbag exited with code {process.returncode}.")


def pending_path(root):
    return Path(root) / f".pending-episode-{uuid.uuid4().hex}"


def next_episode_path(root):
    root = Path(root)
    pattern = re.compile(r"^episode(\d+)$")
    indices = [
        int(match.group(1))
        for child in root.iterdir()
        if child.is_dir() and (match := pattern.match(child.name))
    ] if root.exists() else []
    return root / f"episode{max(indices, default=-1) + 1}"


def remove_path(path):
    path = Path(path)
    if path.exists():
        shutil.rmtree(path)


def write_manifest(path, values):
    Path(path).write_text(json.dumps(values, indent=2) + "\n", encoding="utf-8")


def bag_topic_counts(path):
    metadata_path = Path(path) / "metadata.yaml"
    if not metadata_path.is_file():
        raise ValueError(f"Bag metadata is missing: {metadata_path}")
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    try:
        topics = metadata["rosbag2_bagfile_information"]["topics_with_message_count"]
        return {
            item["topic_metadata"]["name"]: int(item["message_count"])
            for item in topics
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"Invalid rosbag metadata: {metadata_path}") from exc
