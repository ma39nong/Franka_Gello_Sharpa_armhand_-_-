import argparse
from pathlib import Path

import numpy as np
from teleop_core.contract import (
    ARM_STATE_TOPIC,
    VALIDATED_COMMAND_TOPIC,
)
from teleop_core.joint_state import ordered_arm_positions

from .config import load_config
from .portable_bag import (
    HAND_ACTION_TOPIC,
    HAND_STATE_TOPIC,
    ordered_hand_positions,
)
from .hand_profiles import default_hand_profiles
from .timeseries import TimeSeries, fixed_rate_times


def message_time(message, receive_ns):
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is not None and (stamp.sec or stamp.nanosec):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return float(receive_ns) * 1e-9


def read_series(path, hand_profiles=None):
    hand_profiles = hand_profiles or default_hand_profiles()
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(Path(path).resolve()), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = {
        ARM_STATE_TOPIC.format(side="left"),
        ARM_STATE_TOPIC.format(side="right"),
        VALIDATED_COMMAND_TOPIC,
        HAND_STATE_TOPIC.format(side="left"),
        HAND_STATE_TOPIC.format(side="right"),
        HAND_ACTION_TOPIC.format(side="left"),
        HAND_ACTION_TOPIC.format(side="right"),
    }
    missing = sorted(required.difference(types))
    if missing:
        raise ValueError("Bag is missing required topics: " + ", ".join(missing))
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
    while reader.has_next():
        topic, payload, receive_ns = reader.read_next()
        if topic not in required:
            continue
        message = deserialize_message(payload, get_message(types[topic]))
        timestamp = message_time(message, receive_ns)
        if topic == VALIDATED_COMMAND_TOPIC:
            active = tuple(
                side for side in ("left", "right")
                if any(side in name.lower() for name in message.name)
            )
            raw["action"].append((timestamp, (message.name, message.position)))
            raw["active"].append((timestamp, [side in active for side in ("left", "right")]))
        elif topic in {
            ARM_STATE_TOPIC.format(side="left"),
            ARM_STATE_TOPIC.format(side="right"),
        }:
            side = "left" if topic == ARM_STATE_TOPIC.format(side="left") else "right"
            raw[f"{side}_state"].append(
                (timestamp, ordered_arm_positions(message.name, message.position, side))
            )
        else:
            side = "left" if "left" in topic else "right"
            kind = "state" if topic.endswith("_hand_state") else "action"
            try:
                positions = ordered_hand_positions(
                    message.name,
                    message.position,
                    hand_profiles[side],
                )
            except ValueError:
                if kind == "state":
                    # Ignore the vendor's known partial/-1 startup sample.
                    continue
                raise
            raw[f"{side}_hand_{kind}"].append((timestamp, positions))
    if any(not raw[key] for key in raw):
        raise ValueError("Bag does not contain complete states and validated actions.")
    return raw


def normalize_episode(raw, fps, hand_profiles=None):
    left = TimeSeries.from_samples(raw["left_state"])
    right = TimeSeries.from_samples(raw["right_state"])
    left_hand = TimeSeries.from_samples(raw["left_hand_state"])
    right_hand = TimeSeries.from_samples(raw["right_hand_state"])
    action_time = np.asarray([sample[0] for sample in raw["action"]])
    left_hand_action_time = np.asarray(
        [sample[0] for sample in raw["left_hand_action"]]
    )
    right_hand_action_time = np.asarray(
        [sample[0] for sample in raw["right_hand_action"]]
    )
    start = max(
        left.time[0],
        right.time[0],
        left_hand.time[0],
        right_hand.time[0],
        action_time[0],
        left_hand_action_time[0],
        right_hand_action_time[0],
    )
    end = min(
        left.time[-1],
        right.time[-1],
        left_hand.time[-1],
        right_hand.time[-1],
        action_time[-1],
        left_hand_action_time[-1],
        right_hand_action_time[-1],
    )
    timeline = fixed_rate_times(start, end, fps)
    arm_observation = np.concatenate(
        (left.linear(timeline), right.linear(timeline)), axis=1
    )
    hand_observation = np.concatenate(
        (left_hand.linear(timeline), right_hand.linear(timeline)), axis=1
    )
    arm_action = arm_observation.copy()
    active = np.zeros((timeline.size, 2), dtype=np.bool_)
    event_index = 0
    held = arm_observation[0].copy()
    held_active = np.zeros(2, dtype=np.bool_)
    for frame, timestamp in enumerate(timeline):
        while event_index < len(raw["action"]) and raw["action"][event_index][0] <= timestamp:
            _, (names, positions) = raw["action"][event_index]
            for side_index, side in enumerate(("left", "right")):
                if any(side in name.lower() for name in names):
                    held[side_index * 7:(side_index + 1) * 7] = ordered_arm_positions(
                        names, positions, side
                    )
            held_active = np.asarray(raw["active"][event_index][1], dtype=np.bool_)
            event_index += 1
        arm_action[frame] = held
        active[frame] = held_active
    left_hand_action = _held_action(raw["left_hand_action"], timeline)
    right_hand_action = _held_action(raw["right_hand_action"], timeline)
    hand_action = np.concatenate(
        (left_hand_action, right_hand_action), axis=1
    )
    episode = {
        "timestamp": (timeline - timeline[0]).astype(np.float64),
        "observation_arm_joint_position": arm_observation.astype(np.float32),
        "action_arm_joint_position": arm_action.astype(np.float32),
        "observation_hand_joint_position": hand_observation.astype(np.float32),
        "action_hand_joint_position": hand_action.astype(np.float32),
        "observation_joint_position": np.concatenate(
            (arm_observation, hand_observation), axis=1
        ).astype(np.float32),
        "action_joint_position": np.concatenate(
            (arm_action, hand_action), axis=1
        ).astype(np.float32),
        "active_sides": active,
        "fps": np.asarray(fps, dtype=np.int32),
        "schema_version": np.asarray(
            "franka_linker.teleop.normalized.v3"
            if hand_profiles is not None
            else "franka_linker.teleop.normalized.v2"
        ),
    }
    if hand_profiles is not None:
        for side in ("left", "right"):
            profile = hand_profiles[side]
            episode[f"{side}_hand_model"] = np.asarray(profile.model)
            episode[f"{side}_hand_joint_names"] = np.asarray(
                profile.joint_names
            )
    return episode


def _held_action(samples, timeline):
    width = np.asarray(samples[0][1], dtype=float).size
    output = np.empty((timeline.size, width), dtype=float)
    index = 0
    held = np.asarray(samples[0][1], dtype=float)
    for frame, timestamp in enumerate(timeline):
        while index < len(samples) and samples[index][0] <= timestamp:
            held = np.asarray(samples[index][1], dtype=float)
            index += 1
        output[frame] = held
    return output


def convert(input_path, output_path, fps, hand_profiles=None):
    episode = normalize_episode(
        read_series(input_path, hand_profiles=hand_profiles),
        fps,
        hand_profiles=hand_profiles,
    )
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output, **episode)
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    config = load_config(args.config)
    print(
        convert(
            args.input,
            args.output,
            config.conversion_fps,
            hand_profiles=config.hand_profiles,
        )
    )
