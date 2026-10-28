import json
from pathlib import Path

import numpy as np
import pytest

from apps.wuji_ui.models import (
    GESTURE_FEATURES,
    GestureRangeGate,
    HandPose,
    ManusGestureTrigger,
    ManusTrigger,
    WujiPoseRepository,
    gesture_match_fraction,
    manus_gesture_features,
    summarize_gap_samples,
    summarize_gesture_samples,
    unused_pose_name,
)


def names(side: str) -> tuple[str, ...]:
    return tuple(f"{side}_joint_{index}" for index in range(20))


def repository(path: Path) -> WujiPoseRepository:
    return WujiPoseRepository(
        path, {"left": names("left"), "right": names("right")}
    )


def test_left_and_right_poses_have_independent_names(tmp_path: Path) -> None:
    path = tmp_path / "poses.json"
    poses = repository(path)
    poses.save(HandPose.create("left_grasp", "left", [0.1] * 20))
    poses.save(HandPose.create("right_pinch", "right", [0.2] * 20))

    reloaded = repository(path)
    assert reloaded.pose("left", "left_grasp").qpos == pytest.approx([0.1] * 20)
    assert reloaded.pose("right", "right_pinch").qpos == pytest.approx([0.2] * 20)
    data = json.loads(path.read_text())
    assert data["units"] == "rad"
    assert data["joint_order"] == "device"


def test_pose_save_does_not_silently_overwrite(tmp_path: Path) -> None:
    poses = repository(tmp_path / "poses.json")
    poses.save(HandPose.create("pinch", "left", [0.1] * 20))
    with pytest.raises(ValueError, match="已存在"):
        poses.save(HandPose.create("pinch", "left", [0.2] * 20))


def test_unused_pose_name_keeps_original_and_appends_suffix() -> None:
    assert unused_pose_name([], "pinch") == "pinch"
    assert unused_pose_name(["pinch"], "pinch") == "pinch_2"
    assert unused_pose_name(["pinch", "pinch_2"], "pinch") == "pinch_3"


def test_rejects_pose_file_from_different_joint_contract(tmp_path: Path) -> None:
    path = tmp_path / "poses.json"
    repository(path).save(HandPose.create("pose", "left", [0.0] * 20))
    with pytest.raises(ValueError, match="关节顺序"):
        WujiPoseRepository(
            path,
            {
                "left": tuple(reversed(names("left"))),
                "right": names("right"),
            },
        )


def test_manus_trigger_round_trip_and_pose_reference(tmp_path: Path) -> None:
    path = tmp_path / "poses.json"
    poses = repository(path)
    poses.save(HandPose.create("pinch_pose", "right", [0.2] * 20))
    poses.save_trigger(
        ManusTrigger.create(
            "index_pinch",
            "right",
            "index",
            "pinch_pose",
            enter_max_m=0.025,
            exit_min_m=0.032,
            calibration={"sample_count": 100, "p95_m": 0.023},
        )
    )

    trigger = repository(path).triggers("right")[0]
    assert trigger.pose_name == "pinch_pose"
    assert trigger.enter_max_m == pytest.approx(0.025)
    with pytest.raises(ValueError, match="触发器引用"):
        poses.delete("right", "pinch_pose")


def test_gap_summary_uses_robust_percentiles_and_hysteresis() -> None:
    stats = summarize_gap_samples([0.010 + index * 0.001 for index in range(20)])
    assert stats["sample_count"] == 20
    assert stats["recommended_enter_max_m"] > stats["p95_m"]
    assert stats["recommended_exit_min_m"] > stats["recommended_enter_max_m"]


def test_composite_gesture_summary_match_and_round_trip(tmp_path: Path) -> None:
    samples = [
        {name: 0.5 + index * 0.001 for name in GESTURE_FEATURES}
        for index in range(60)
    ]
    model = summarize_gesture_samples(samples)
    score, matches = gesture_match_fraction(samples[30], model["feature_ranges"])
    assert score == pytest.approx(1.0)
    assert all(matches.values())

    path = tmp_path / "poses.json"
    poses = repository(path)
    poses.save(HandPose.create("bottle", "left", [0.1] * 20))
    poses.save_gesture_trigger(
        ManusGestureTrigger.create(
            "bottle_grasp",
            "left",
            "bottle",
            model["feature_ranges"],
            sample_count=model["sample_count"],
        )
    )
    loaded = repository(path).gesture_triggers("left")[0]
    assert loaded.pose_name == "bottle"
    assert loaded.sample_count == 60


def test_composite_features_are_translation_rotation_and_scale_invariant() -> None:
    points = np.asarray(
        [
            [0.00, 0.00, 0.00],
            [0.02, 0.00, 0.00], [0.04, 0.00, 0.00],
            [0.06, 0.01, 0.00], [0.07, 0.03, 0.00],
            [0.03, 0.04, 0.00], [0.03, 0.06, 0.00],
            [0.025, 0.075, 0.00], [0.015, 0.085, 0.00],
            [0.00, 0.05, 0.00], [0.00, 0.075, 0.00],
            [-0.005, 0.095, 0.00], [-0.015, 0.11, 0.00],
            [-0.03, 0.04, 0.00], [-0.03, 0.06, 0.00],
            [-0.035, 0.075, 0.00], [-0.045, 0.085, 0.00],
            [-0.06, 0.03, 0.00], [-0.06, 0.05, 0.00],
            [-0.065, 0.065, 0.00], [-0.075, 0.075, 0.00],
        ],
        dtype=float,
    )
    angle = np.deg2rad(37.0)
    rotation = np.asarray(
        [[np.cos(angle), -np.sin(angle), 0.0],
         [np.sin(angle), np.cos(angle), 0.0],
         [0.0, 0.0, 1.0]]
    )
    original = manus_gesture_features(points)
    transformed = manus_gesture_features((points @ rotation.T) * 1.7 + 0.3)
    assert transformed == pytest.approx(original)


def test_composite_gate_applies_dwell_and_exit_hysteresis() -> None:
    gate = GestureRangeGate(
        enter_match_fraction=0.8,
        exit_match_fraction=0.6,
        dwell_seconds=0.2,
    )
    assert not gate.update(1.0, 0.9)
    assert gate.phase == "candidate"
    assert not gate.update(1.1, 0.9)
    assert gate.update(1.21, 0.9)
    assert gate.phase == "active"
    assert gate.update(1.3, 0.7)
    assert not gate.update(1.4, 0.5)
    assert gate.phase == "inactive"
