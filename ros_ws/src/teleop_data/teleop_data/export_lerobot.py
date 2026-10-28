import argparse
from pathlib import Path

import numpy as np

from .config import load_config
from .converter import normalize_episode
from .hand_profiles import default_hand_profiles
from .portable_bag import (
    image_to_depth,
    image_to_rgb,
    iter_topic,
    read_camera_info,
    read_raw_episode,
    recording_topics,
    stream_bounds,
)


STATE_NAMES = [
    *(f"left_fr3_joint{index}.position" for index in range(1, 8)),
    *(f"right_fr3_joint{index}.position" for index in range(1, 8)),
]
ACTION_NAMES = [
    *(f"left_fr3_joint{index}.target_position" for index in range(1, 8)),
    *(f"right_fr3_joint{index}.target_position" for index in range(1, 8)),
]


def _hand_feature_names(hand_profiles, suffix):
    return [
        f"{side}_linker_hand."
        f"{name.lower().replace(' ', '_')}.{suffix}"
        for side, profile in (
            (side, hand_profiles[side]) for side in ("left", "right")
        )
        for name in profile.joint_names
    ]


def features(color_shape, depth_shape, hand_profiles=None):
    hand_profiles = hand_profiles or default_hand_profiles()
    all_state_names = STATE_NAMES + _hand_feature_names(
        hand_profiles, "position"
    )
    all_action_names = ACTION_NAMES + _hand_feature_names(
        hand_profiles, "target_position"
    )
    total_width = len(all_state_names)
    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (total_width,),
            "names": all_state_names,
        },
        "action": {
            "dtype": "float32",
            "shape": (total_width,),
            "names": all_action_names,
        },
        "observation.active_sides": {
            "dtype": "float32",
            "shape": (2,),
            "names": ["left", "right"],
        },
        "observation.images.orbbec": {
            "dtype": "video",
            "shape": color_shape,
            "names": ["height", "width", "channels"],
        },
        "observation.images.orbbec_depth": {
            "dtype": "video",
            "shape": depth_shape,
            "names": ["height", "width", "channels"],
            "info": {
                "is_depth_map": True,
                "depth_unit": "mm",
                "invalid_value": 0,
                "registered_to": "observation.images.orbbec",
            },
        },
        "observation.camera.orbbec_rgb_age_ms": {
            "dtype": "float32",
            "shape": (1,),
            "names": ["age_ms"],
        },
        "observation.camera.orbbec_depth_age_ms": {
            "dtype": "float32",
            "shape": (1,),
            "names": ["age_ms"],
        },
        "observation.camera.orbbec_intrinsics": {
            "dtype": "float32",
            "shape": (9,),
            "names": ["fx", "s", "cx", "0", "fy", "cy", "0", "0", "1"],
        },
        "observation.camera.orbbec_distortion": {
            "dtype": "float32",
            "shape": (8,),
            "names": [f"d{index}" for index in range(8)],
        },
        "observation.camera.orbbec_depth_scale_m": {
            "dtype": "float32",
            "shape": (1,),
            "names": ["metres_per_unit"],
        },
    }


def export_bags(bags, output, repo_id, task, config_path, fps=None):
    output = Path(output).expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Output already exists: {output}")
    if not bags:
        raise ValueError("At least one bag is required.")
    config = load_config(config_path)
    fps = config.conversion_fps if fps is None else int(fps)
    if fps <= 0:
        raise ValueError("Export FPS must be positive.")
    topics = recording_topics(config)
    bags = [Path(bag).expanduser().resolve() for bag in bags]
    progress = _progress()

    stream_info = []
    for bag in bags:
        progress.write(f"Scanning {bag.name} camera streams...")
        color = stream_bounds(bag, topics["camera_color"], image_to_rgb)
        depth = stream_bounds(bag, topics["camera_depth"], image_to_depth)
        _validate_depth_range(bag, topics["camera_depth"])
        stream_info.append((color, depth))
    color_shape = stream_info[0][0][2]
    depth_shape = stream_info[0][1][2]
    if any(item[0][2] != color_shape for item in stream_info):
        raise ValueError("All episodes must use the same RGB image shape.")
    if any(item[1][2] != depth_shape for item in stream_info):
        raise ValueError("All episodes must use the same depth image shape.")

    LeRobotDataset = _lerobot_dataset()
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        root=output,
        robot_type="dual_fr3",
        features=features(color_shape, depth_shape, config.hand_profiles),
        use_videos=True,
        image_writer_threads=4,
        rgb_encoder=_rgb_encoder(),
        depth_encoder=_depth_encoder(),
        encoder_threads=2,
    )
    exported_frames = 0
    try:
        for bag, (color_info, depth_info) in zip(bags, stream_info):
            progress.write(f"Reading {bag.name} arm state and actions...")
            raw = read_raw_episode(bag, config.hand_profiles)
            episode = normalize_episode(
                raw, fps, hand_profiles=config.hand_profiles
            )
            absolute_start = max(
                raw["left_state"][0][0],
                raw["right_state"][0][0],
                raw["action"][0][0],
            )
            absolute_time = absolute_start + episode["timestamp"]
            start = max(color_info[0], depth_info[0])
            end = min(color_info[1], depth_info[1])
            mask = (absolute_time >= start) & (absolute_time <= end)
            if np.count_nonzero(mask) < 2:
                raise ValueError(f"No aligned RGB-D/control interval in {bag}.")
            selected = {
                "time": absolute_time[mask],
                "state": episode["observation_joint_position"][mask],
                "action": episode["action_joint_position"][mask],
                "active": episode["active_sides"][mask].astype(np.float32),
            }
            _append_episode(
                dataset,
                bag,
                selected,
                topics,
                task,
                progress,
            )
            exported_frames += selected["time"].size
            progress.write(f"Finalizing {bag.name} videos and metadata...")
            dataset.save_episode()
    finally:
        dataset.finalize()
    _make_world_writable(output)
    _verify_dataset(output, repo_id, exported_frames, progress)
    return output


def _append_episode(dataset, bag, episode, topics, task, progress):
    color_stream = iter(iter_topic(bag, topics["camera_color"], image_to_rgb))
    depth_stream = iter(iter_topic(bag, topics["camera_depth"], image_to_depth))
    upcoming = {
        "color": next(color_stream, None),
        "depth": next(depth_stream, None),
    }
    last = {"color": None, "depth": None}
    streams = {"color": color_stream, "depth": depth_stream}
    intrinsic, distortion = read_camera_info(bag, topics["camera_depth_info"])
    for index, policy_time in enumerate(
        progress(
            episode["time"],
            total=episode["time"].size,
            desc=f"{bag.name}: encoding RGB-D",
            unit="frame",
        )
    ):
        for name, stream in streams.items():
            while upcoming[name] is not None and upcoming[name][0] <= policy_time + 1e-6:
                last[name] = upcoming[name]
                upcoming[name] = next(stream, None)
        missing = [name for name, sample in last.items() if sample is None]
        if missing:
            raise ValueError(f"No sample at policy time for {', '.join(missing)}.")
        color_time, color = last["color"]
        depth_time, depth = last["depth"]
        dataset.add_frame(
            {
                "observation.state": episode["state"][index],
                "action": episode["action"][index],
                "observation.active_sides": episode["active"][index],
                "observation.images.orbbec": color,
                "observation.images.orbbec_depth": depth,
                "observation.camera.orbbec_rgb_age_ms": _age(
                    policy_time, color_time
                ),
                "observation.camera.orbbec_depth_age_ms": _age(
                    policy_time, depth_time
                ),
                "observation.camera.orbbec_intrinsics": intrinsic,
                "observation.camera.orbbec_distortion": distortion,
                "observation.camera.orbbec_depth_scale_m": np.asarray(
                    [0.001], dtype=np.float32
                ),
                "task": task,
            }
        )


def _validate_depth_range(bag, topic):
    maximum = max(int(depth.max()) for _, depth in iter_topic(bag, topic, image_to_depth))
    if maximum > 4095:
        raise ValueError(
            f"{bag} depth reaches {maximum} mm; exact lossless export supports "
            "at most 4095 mm."
        )


def _age(policy_time, sample_time):
    return np.asarray(
        [max(0.0, (policy_time - sample_time) * 1000.0)],
        dtype=np.float32,
    )


def _lerobot_dataset():
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        raise RuntimeError(
            "LeRobot 0.6 is unavailable; use ops/diagnostics/export_lerobot.sh."
        ) from exc
    return LeRobotDataset


def _rgb_encoder():
    from lerobot.configs.video import RGBEncoderConfig

    return RGBEncoderConfig(
        vcodec="h264",
        pix_fmt="yuv420p",
        g=30,
        crf=18,
        preset="medium",
    )


def _depth_encoder():
    from lerobot.configs.video import DepthEncoderConfig

    return DepthEncoderConfig(
        vcodec="hevc",
        pix_fmt="gray12le",
        g=30,
        crf=0,
        preset="medium",
        extra_options={"x265-params": "lossless=1"},
        depth_min=0.0,
        depth_max=4.095,
        shift=0.0,
        use_log=False,
    )


def _progress():
    try:
        from tqdm.auto import tqdm
    except ImportError as exc:
        raise RuntimeError("tqdm is required for LeRobot export.") from exc
    return tqdm


def _make_world_writable(root):
    root.chmod(0o777)
    for path in root.rglob("*"):
        path.chmod(0o777 if path.is_dir() else 0o666)


def _verify_dataset(output, repo_id, expected_frames, progress):
    dataset = _lerobot_dataset()(repo_id=repo_id, root=output)
    if len(dataset) != expected_frames:
        raise RuntimeError(
            f"LeRobot readback returned {len(dataset)} frames; "
            f"expected {expected_frames}."
        )
    sample = dataset[0]
    required = {
        "observation.state",
        "action",
        "observation.images.orbbec",
        "observation.images.orbbec_depth",
    }
    missing = sorted(required.difference(sample))
    if missing:
        raise RuntimeError("LeRobot readback is missing: " + ", ".join(missing))
    progress.write(f"Verified {len(dataset)} LeRobot RGB-D frames.")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Export dual-FR3 Orbbec rosbag episodes to LeRobot v3."
    )
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--repo-id", default="local/franka_upper_body_teleop")
    parser.add_argument("--task", required=True)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--fps", type=int)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    print(
        export_bags(
            args.bags,
            args.output,
            args.repo_id,
            args.task,
            args.config,
            args.fps,
        )
    )


if __name__ == "__main__":
    main()
