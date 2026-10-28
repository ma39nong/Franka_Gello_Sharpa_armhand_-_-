from pathlib import Path

import numpy as np
from teleop_core.contract import ARM_STATE_TOPIC, VALIDATED_COMMAND_TOPIC
from teleop_core.joint_state import ordered_arm_positions

from .hand_profiles import G20_JOINT_NAMES, default_hand_profiles

HAND_STATE_TOPIC = "/cb_{side}_hand_state"
HAND_ACTION_TOPIC = "/cb_{side}_hand_control_cmd"


def recording_topics(config):
    by_name = {spec.name: spec.topic for spec in config.topics}
    required = {
        "camera_color",
        "camera_color_info",
        "camera_depth",
        "camera_depth_info",
        "left_hand_state",
        "right_hand_state",
        "left_hand_action",
        "right_hand_action",
    }
    missing = sorted(required.difference(by_name))
    if missing:
        raise ValueError(
            "Recording config is missing dataset topic entries: " + ", ".join(missing)
        )
    return {name: by_name[name] for name in required}


def camera_topics(config):
    topics = recording_topics(config)
    return {
        name: topic
        for name, topic in topics.items()
        if name in {
            "camera_color",
            "camera_color_info",
            "camera_depth",
            "camera_depth_info",
        }
    }


def read_raw_episode(bag, hand_profiles=None):
    hand_profiles = hand_profiles or default_hand_profiles()
    topics = {
        "left_state": ARM_STATE_TOPIC.format(side="left"),
        "right_state": ARM_STATE_TOPIC.format(side="right"),
        "action": VALIDATED_COMMAND_TOPIC,
        "left_hand_state": HAND_STATE_TOPIC.format(side="left"),
        "right_hand_state": HAND_STATE_TOPIC.format(side="right"),
        "left_hand_action": HAND_ACTION_TOPIC.format(side="left"),
        "right_hand_action": HAND_ACTION_TOPIC.format(side="right"),
    }
    topic_to_key = {topic: key for key, topic in topics.items()}
    raw = {
        "left_state": [],
        "right_state": [],
        "action": [],
        "active": [],
        "left_hand_state": [],
        "right_hand_state": [],
        "left_hand_action": [],
        "right_hand_action": [],
    }
    with _reader(bag) as reader:
        connections = [
            connection
            for connection in reader.connections
            if connection.topic in topic_to_key
        ]
        present = {connection.topic for connection in connections}
        missing = sorted(set(topic_to_key).difference(present))
        if missing:
            raise ValueError("Bag is missing required topics: " + ", ".join(missing))
        for connection, receive_ns, payload in reader.messages(connections=connections):
            message = reader.deserialize(payload, connection.msgtype)
            timestamp = message_time(message, receive_ns)
            key = topic_to_key[connection.topic]
            if key == "action":
                active = tuple(
                    side
                    for side in ("left", "right")
                    if any(side in name.lower() for name in message.name)
                )
                raw["action"].append((timestamp, (message.name, message.position)))
                raw["active"].append(
                    (timestamp, [side in active for side in ("left", "right")])
                )
            elif key in {"left_state", "right_state"}:
                side = "left" if key == "left_state" else "right"
                raw[key].append(
                    (
                        timestamp,
                        ordered_arm_positions(message.name, message.position, side),
                    )
                )
            else:
                try:
                    positions = ordered_hand_positions(
                        message.name,
                        message.position,
                        hand_profiles[
                            "left" if key.startswith("left_") else "right"
                        ],
                    )
                except ValueError:
                    if key.endswith("_state"):
                        # The vendor driver publishes a known 10-value/-1
                        # initializer before its first hardware measurement.
                        continue
                    raise
                raw[key].append((timestamp, positions))
    if any(not raw[key] for key in raw):
        raise ValueError("Bag does not contain complete states and validated actions.")
    return raw


def ordered_hand_positions(names, positions, profile_or_joint_names=G20_JOINT_NAMES):
    if len(names) != len(positions):
        raise ValueError("Hand joint names and positions have different lengths.")
    if len(names) != len(set(names)):
        raise ValueError("Hand joint state contains duplicate names.")
    values = dict(zip(names, positions))
    if hasattr(profile_or_joint_names, "joint_names"):
        profile = profile_or_joint_names
        joint_names = tuple(profile.joint_names)
        lower_bounds = np.asarray(profile.lower_bounds, dtype=float)
        upper_bounds = np.asarray(profile.upper_bounds, dtype=float)
    else:
        joint_names = tuple(profile_or_joint_names)
        lower_bounds = np.zeros(len(joint_names), dtype=float)
        upper_bounds = np.full(len(joint_names), 255.0, dtype=float)
    missing = [name for name in joint_names if name not in values]
    if missing:
        raise ValueError("Hand joint state is missing: " + ", ".join(missing))
    ordered = np.asarray([values[name] for name in joint_names], dtype=float)
    if ordered.shape != (len(joint_names),) or not np.all(np.isfinite(ordered)):
        raise ValueError(
            f"Hand joint state must contain {len(joint_names)} finite positions."
        )
    if np.any(ordered < lower_bounds) or np.any(ordered > upper_bounds):
        raise ValueError("Hand joint positions exceed the configured profile bounds.")
    return ordered


def stream_bounds(bag, topic, converter):
    first = last = None
    shape = None
    for timestamp, value in iter_topic(bag, topic, converter):
        if first is None:
            first = timestamp
            shape = value.shape
        last = timestamp
    if first is None or last is None or shape is None:
        raise ValueError(f"Bag contains no samples for {topic}: {bag}")
    return first, last, shape


def iter_topic(bag, topic, converter):
    with _reader(bag) as reader:
        connections = [
            connection for connection in reader.connections if connection.topic == topic
        ]
        if not connections:
            raise ValueError(f"Bag does not contain {topic}: {bag}")
        for connection, receive_ns, payload in reader.messages(connections=connections):
            message = reader.deserialize(payload, connection.msgtype)
            yield message_time(message, receive_ns), converter(message)


def read_camera_info(bag, topic):
    _, value = next(iter_topic(bag, topic, camera_info_to_arrays), (None, None))
    if value is None:
        raise ValueError(f"Bag contains no CameraInfo for {topic}: {bag}")
    return value


def camera_info_to_arrays(message):
    distortion = np.zeros(8, dtype=np.float32)
    values = np.asarray(message.d, dtype=np.float32)
    distortion[: min(8, values.size)] = values[:8]
    return np.asarray(message.k, dtype=np.float32), distortion


def image_to_rgb(message):
    encoding = message.encoding.lower()
    channels = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
    }.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported color encoding: {message.encoding}")
    rows = np.asarray(message.data, dtype=np.uint8).reshape(
        message.height, message.step
    )
    image = rows[:, : message.width * channels].reshape(
        message.height, message.width, channels
    )
    if encoding.startswith("bgr"):
        order = [2, 1, 0, 3] if channels == 4 else [2, 1, 0]
        image = image[..., order]
    return np.ascontiguousarray(image[..., :3])


def image_to_depth(message):
    if message.encoding.lower() not in {"16uc1", "mono16"}:
        raise ValueError(f"Unsupported depth encoding: {message.encoding}")
    byte_order = ">u2" if message.is_bigendian else "<u2"
    rows = np.asarray(message.data, dtype=np.uint8).reshape(
        message.height, message.step
    )
    depth = rows[:, : message.width * 2].view(byte_order).reshape(
        message.height, message.width, 1
    )
    return np.ascontiguousarray(depth.astype(np.uint16, copy=False))


def message_time(message, receive_ns):
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is not None and (stamp.sec or stamp.nanosec):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return float(receive_ns) * 1e-9


def _reader(bag):
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            "rosbags is unavailable; use the LeRobot runtime in the Docker image."
        ) from exc
    path = Path(bag).expanduser().resolve()
    if not (path / "metadata.yaml").is_file():
        raise ValueError(f"Not a rosbag directory: {path}")
    return AnyReader(
        [path],
        default_typestore=get_typestore(Stores.ROS2_HUMBLE),
    )
