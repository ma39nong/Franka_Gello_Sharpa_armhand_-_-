from pathlib import Path
from types import SimpleNamespace

import pytest

from teleop_core.contract import (
    LEFT_COMMAND_JOINT_NAMES,
    RIGHT_COMMAND_JOINT_NAMES,
    VALIDATED_COMMAND_TOPIC,
)
from teleop_data_collector.rosbag_to_lerobot import (
    EpisodeConverter,
    SamplingSpec,
    _depth_image_specs_from_config,
    _feature_specs_from_config,
    _load_conversion_config,
    _message_to_source_value,
    _mode_conversion_config,
    _source_names,
    _trim_source_window,
    _episode_segments,
    _video_specs_from_config,
)


REPO_ROOT = Path(__file__).resolve().parents[4]
CONVERSION_CONFIG = REPO_ROOT / "data_collection/config/convert_gello_lerobot_v2.yaml"


def test_milestone_selection_preserves_legacy_bags_and_supports_multiple_prefixes():
    assert _episode_segments({}, "all") == [("full", None)]
    state = {"milestones": [
        {"id": "milestone_1", "timestamp_ns": 100, "clock": "ros"},
        {"id": "milestone_2", "timestamp_ns": 200, "clock": "ros"},
    ]}
    assert _episode_segments(state, "all") == [
        ("milestone_1", 100), ("milestone_2", 200), ("full", None),
    ]
    assert _episode_segments(state, "full") == [("full", None)]
    assert _episode_segments(state, "milestones") == [("milestone_1", 100), ("milestone_2", 200)]
    with pytest.raises(ValueError, match="no milestones"):
        _episode_segments({}, "milestones")


@pytest.mark.parametrize("markers", [
    None,
    [None],
    [{"id": "milestone_1", "timestamp_ns": True, "clock": "ros"}],
    [{"id": "milestone_1", "timestamp_ns": 100, "clock": "wall"}],
    [{"id": "full", "timestamp_ns": 100, "clock": "ros"}],
    [{"id": "milestone_1", "timestamp_ns": 100, "clock": "ros"}] * 2,
    [{"id": "first", "timestamp_ns": 100, "clock": "ros"},
     {"id": "second", "timestamp_ns": 50, "clock": "ros"}],
])
def test_invalid_milestones_are_rejected(markers):
    with pytest.raises(ValueError):
        _episode_segments({"milestones": markers}, "all")


def _joint_state(names, positions):
    return SimpleNamespace(
        name=list(names),
        position=list(positions),
        velocity=[0.0] * len(names),
        effort=[],
    )


def test_gello_schema_is_exact_harvest_54_108_and_three_cameras():
    config = _mode_conversion_config("gello")
    features = {
        spec.column: spec for spec in _feature_specs_from_config(config)
    }
    assert len(_source_names(features["action"])) == 54
    assert len(_source_names(features["observation.state"])) == 108
    assert _source_names(features["observation.engaged"]) == [
        "left_arm",
        "right_arm",
        "left_hand",
        "right_hand",
    ]
    videos = {spec.column: spec for spec in _video_specs_from_config(config)}
    assert set(videos) == {
        "observation.images.cam0",
        "observation.images.cam1",
        "observation.images.cam2",
    }
    assert videos["observation.images.cam0"].fps == 20
    assert videos["observation.images.cam1"].fps == 30
    assert videos["observation.images.cam2"].fps == 30
    depth_specs = _depth_image_specs_from_config(config)
    assert len(depth_specs) == 1
    assert depth_specs[0].column == "observation.depths.cam0"
    assert depth_specs[0].topic == "/cam0/depth/image_raw"
    assert depth_specs[0].format == "raw16"
    assert depth_specs[0].fps == 20


def test_gello_conversion_config_uses_asymmetric_edge_trim():
    sampling = _load_conversion_config(CONVERSION_CONFIG)["sampling"]
    assert sampling["trim_start_sec"] == 1.0
    assert sampling["trim_end_sec"] == 0.2


def test_source_window_trims_both_edges_and_rejects_short_episode():
    assert _trim_source_window(
        10_000_000_000,
        15_000_000_000,
        trim_start_ns=1_000_000_000,
        trim_end_ns=1_000_000_000,
    ) == (11_000_000_000, 14_000_000_000)
    with pytest.raises(ValueError, match="removes the complete"):
        _trim_source_window(
            10_000_000_000,
            11_500_000_000,
            trim_start_ns=1_000_000_000,
            trim_end_ns=1_000_000_000,
        )


def test_combined_validated_action_splits_sides_by_name_and_normalizes_names():
    config = _mode_conversion_config("gello")
    action = _feature_specs_from_config(config)[0]
    assert action.sources[0].topic == VALIDATED_COMMAND_TOPIC
    assert action.sources[1].topic == VALIDATED_COMMAND_TOPIC
    message = _joint_state(
        [*RIGHT_COMMAND_JOINT_NAMES, *LEFT_COMMAND_JOINT_NAMES],
        [*range(7, 14), *range(7)],
    )
    left = _message_to_source_value(
        "sensor_msgs/msg/JointState",
        message,
        "position",
        expected_joint_names=list(LEFT_COMMAND_JOINT_NAMES),
        allow_extra_joint_names=True,
    )
    right = _message_to_source_value(
        "sensor_msgs/msg/JointState",
        message,
        "position",
        expected_joint_names=list(RIGHT_COMMAND_JOINT_NAMES),
        allow_extra_joint_names=True,
    )
    assert left == [float(value) for value in range(7)]
    assert right == [float(value) for value in range(7, 14)]


def test_inactive_side_is_missing_not_fabricated():
    message = _joint_state(LEFT_COMMAND_JOINT_NAMES, range(7))
    assert _message_to_source_value(
        "sensor_msgs/msg/JointState",
        message,
        "position",
        expected_joint_names=list(RIGHT_COMMAND_JOINT_NAMES),
        allow_extra_joint_names=True,
    ) is None


def test_partial_named_side_is_rejected():
    message = _joint_state(LEFT_COMMAND_JOINT_NAMES[:-1], range(6))
    with pytest.raises(ValueError, match="partial named group"):
        _message_to_source_value(
            "sensor_msgs/msg/JointState",
            message,
            "position",
            expected_joint_names=list(LEFT_COMMAND_JOINT_NAMES),
            allow_extra_joint_names=True,
        )


def test_status_messages_become_side_ordered_engagement_values():
    arm_status = SimpleNamespace(accepted_sides=["right"])
    assert _message_to_source_value(
        "teleop_interfaces/msg/ArmCommandStatus",
        arm_status,
        "accepted_sides.left",
    ) == [0.0]
    assert _message_to_source_value(
        "teleop_interfaces/msg/ArmCommandStatus",
        arm_status,
        "accepted_sides.right",
    ) == [1.0]

    hand_status = SimpleNamespace(side="left", engaged=False)
    assert _message_to_source_value(
        "teleop_interfaces/msg/HandTelemetryStatus",
        hand_status,
        "engaged.left",
    ) == [0.0]
    assert _message_to_source_value(
        "teleop_interfaces/msg/HandTelemetryStatus",
        hand_status,
        "engaged.right",
    ) is None


def test_disengaged_sources_hold_last_action_or_seed_from_measured_state():
    features = _feature_specs_from_config(_mode_conversion_config("gello"))
    converter = EpisodeConverter(
        bag_path=Path("bag"),
        output_dir=Path("output"),
        episode_index=0,
        global_start_index=0,
        storage_id=None,
        feature_specs=features,
        video_specs=[],
        depth_image_specs=[],
        pointcloud_specs=[],
        static_topic_specs=[],
        sampling=SamplingSpec(
            "fixed_hz",
            frequency_hz=30,
            max_staleness_ns=150_000_000,
        ),
        fps=30,
        task_index=0,
        chunk_size=1000,
        compression=None,
    )
    values = {
        ("observation.engaged", 0): [0.0],
        ("observation.engaged", 1): [1.0],
        ("observation.engaged", 2): [0.0],
        ("observation.engaged", 3): [1.0],
        ("observation.state", 0): [*map(float, range(7)), *([0.0] * 7)],
        ("observation.state", 2): [*map(float, range(20, 40)), *([0.0] * 20)],
        ("action", 1): list(map(float, range(10, 17))),
        ("action", 3): list(map(float, range(40, 60))),
    }
    times = {key: 100 for key in values}

    held_values, held_times = converter._apply_hold_policy(100, values, times)
    assert held_values[("action", 0)] == list(map(float, range(7)))
    assert held_values[("action", 2)] == list(map(float, range(20, 40)))
    assert held_times[("action", 0)] == 100

    values[("observation.state", 0)] = [*([99.0] * 7), *([0.0] * 7)]
    held_values, _ = converter._apply_hold_policy(120, values, times)
    assert held_values[("action", 0)] == list(map(float, range(7)))

    # The right side then disengages after having real commands. Its HOLD must
    # latch that command, rather than track later measured-state drift.
    values[("observation.engaged", 1)] = [0.0]
    times[("observation.engaged", 1)] = 130
    values[("observation.state", 1)] = [*([55.0] * 7), *([0.0] * 7)]
    held_values, _ = converter._apply_hold_policy(130, values, times)
    assert held_values[("action", 1)] == list(map(float, range(10, 17)))
    values[("observation.state", 1)] = [*([66.0] * 7), *([0.0] * 7)]
    held_values, _ = converter._apply_hold_policy(140, values, times)
    assert held_values[("action", 1)] == list(map(float, range(10, 17)))

    # Re-engagement becomes effective only together with the first new action;
    # a status/action ordering gap must not expose a stale pre-HOLD target.
    values[("observation.engaged", 1)] = [1.0]
    times[("observation.engaged", 1)] = 200_000_000
    aligned_values, _ = converter._apply_hold_policy(200_000_000, values, times)
    assert aligned_values[("observation.engaged", 1)] == [0.0]
    assert aligned_values[("action", 1)] == list(map(float, range(10, 17)))

    values[("action", 1)] = [77.0] * 7
    times[("action", 1)] = 201_000_000
    aligned_values, _ = converter._apply_hold_policy(201_000_000, values, times)
    assert aligned_values[("observation.engaged", 1)] == [1.0]
    assert aligned_values[("action", 1)] == [77.0] * 7
