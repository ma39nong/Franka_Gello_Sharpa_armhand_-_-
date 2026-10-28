import argparse
import time
from pathlib import Path

import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from .config import load_config
from .converter import message_time
from .portable_bag import camera_topics


class CameraReplay(Node):
    def __init__(self, topic_types, namespace):
        super().__init__("teleop_camera_replay")
        prefix = "/" + namespace.strip("/")
        self._camera_publishers = {
            topic: self.create_publisher(
                get_message(type_name),
                prefix + topic,
                qos_profile_sensor_data,
            )
            for topic, type_name in topic_types.items()
        }

    def publish(self, topic, message):
        stamp = self.get_clock().now().to_msg()
        if hasattr(message, "header"):
            message.header.stamp = stamp
        self._camera_publishers[topic].publish(message)


def replay_camera(bag, config_path, speed=1.0, namespace="replay"):
    if speed <= 0:
        raise ValueError("Replay speed must be positive.")
    config = load_config(config_path)
    camera = camera_topics(config)
    wanted = set(camera.values())
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(Path(bag).expanduser().resolve()),
            storage_id=config.storage_id,
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
        if item.name in wanted
    }
    missing = sorted(wanted.difference(topic_types))
    if missing:
        raise ValueError("Bag is missing camera topics: " + ", ".join(missing))
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(wanted)))
    node = CameraReplay(topic_types, namespace)
    first_time = None
    started = time.monotonic()
    count = 0
    while reader.has_next():
        topic, payload, receive_ns = reader.read_next()
        message = deserialize_message(payload, get_message(topic_types[topic]))
        timestamp = message_time(message, receive_ns)
        if first_time is None:
            first_time = timestamp
        relative = (timestamp - first_time) / speed
        deadline = started + relative
        while time.monotonic() < deadline:
            time.sleep(min(0.01, deadline - time.monotonic()))
        node.publish(topic, message)
        count += 1
    node.destroy_node()
    return count


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Replay an episode's RGB-D topics under /replay."
    )
    parser.add_argument("bag", type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--namespace", default="replay")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    rclpy.init()
    try:
        count = replay_camera(args.bag, args.config, args.speed, args.namespace)
        print(f"Replayed {count} camera messages.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
