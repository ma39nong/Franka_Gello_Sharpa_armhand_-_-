from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from teleop_core.contract import (
    LEFT_COMMAND_JOINT_NAMES,
    VALIDATED_COMMAND_TOPIC,
)
from teleop_data_collector.bag_validation import (
    StreamSamples,
    _engaged_action_failures,
    _inspect_message_content,
    _update_cumulative_counter,
    stream_timing_report,
    timing_failures,
    transport_timing_warnings,
    boundary_timing_warnings,
)
from teleop_data_collector.collector_contract import (
    SHARPA_TOPICS,
    topic_contracts,
    validate_recording_contract,
)


REPO_ROOT = Path(__file__).resolve().parents[4]
CONFIG = REPO_ROOT / "data_collection/config/record_gello.yaml"
SHARPA_CONFIG = REPO_ROOT / "data_collection/config/record_gello_sharpa.yaml"
SHARPA_TACTILE_CONFIG = (
    REPO_ROOT / "data_collection/config/record_gello_sharpa_tactile.yaml"
)
SHARPA_TACTILE_QOS = (
    REPO_ROOT / "data_collection/config/rosbag_qos_sharpa_tactile.yaml"
)


def _config():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
        "teleop_data_collector"
    ]["ros__parameters"]


def _sharpa_config():
    return yaml.safe_load(SHARPA_CONFIG.read_text(encoding="utf-8"))[
        "teleop_data_collector"
    ]["ros__parameters"]


def _sharpa_tactile_config():
    return yaml.safe_load(SHARPA_TACTILE_CONFIG.read_text(encoding="utf-8"))[
        "teleop_data_collector"
    ]["ros__parameters"]


def test_recording_contract_has_only_post_gateway_action_and_13_required_streams():
    config = _config()
    validate_recording_contract(config)
    assert config["teleoperator"] == "gello"
    assert config["trim_start_sec"] == 1.0
    assert config["trim_end_sec"] == 0.2
    assert config["control_bind_host"] == "127.0.0.1"
    assert config["control_port"] == 5592
    assert config["rosbag_record_args"] == [
        "--storage",
        "sqlite3",
        "--max-cache-size",
        "268435456",
    ]
    assert config["rosbag_record_default_qos"] == {
        "history": "keep_last",
        "depth": 10,
        "reliability": "reliable",
        "durability": "volatile",
    }
    topics = topic_contracts(config["topics"])
    assert len(topics) == 13
    assert VALIDATED_COMMAND_TOPIC in {item.topic for item in topics}
    assert "/teleop/arm_commands" not in {item.topic for item in topics}
    assert "/cam0/depth/image_raw" in {item.topic for item in topics}
    assert all(item.max_gap_ms == 150.0 for item in topics)
    by_topic = {item.topic: item for item in topics}
    assert by_topic["/cam0/color/image_raw"].min_frequency_hz == 18.0
    assert by_topic["/cam0/depth/image_raw"].min_frequency_hz == 18.0


def test_recording_contract_requires_explicit_hold_status_topics():
    config = _config()
    del config["topics"]["arm_command_status"]
    with pytest.raises(ValueError, match="HOLD status topic"):
        validate_recording_contract(config)


def test_sharpa_recording_contract_has_complete_dual_hand_and_camera_streams():
    config = _sharpa_config()
    validate_recording_contract(config)
    topics = topic_contracts(config["topics"])
    required = {item.topic for item in topics if item.required}

    assert len(topics) == 12
    assert SHARPA_TOPICS <= required
    assert "/teleop/wuji/telemetry_status" not in required
    assert required >= {
        "/teleop/validated_arm_commands",
        "/teleop/arm_command_status",
        "/left/franka/joint_states",
        "/right/franka/joint_states",
        "/cam0/color/image_raw",
        "/cam0/depth/image_raw",
        "/cam1/color/image_raw",
        "/cam2/color/image_raw",
    }


def test_sharpa_recording_contract_rejects_partial_hand_telemetry():
    config = _sharpa_config()
    del config["topics"]["right_hand_state"]

    with pytest.raises(ValueError, match="both command and state"):
        validate_recording_contract(config)


def test_sharpa_tactile_contract_adds_all_ten_finger_stream_pairs():
    config = _sharpa_tactile_config()
    validate_recording_contract(config)
    topics = topic_contracts(config["topics"])
    by_topic = {item.topic: item for item in topics}
    expected = {
        f"/sharpa/{side}/tactile/{finger}/{stream}"
        for side in ("left", "right")
        for finger in ("thumb", "index", "middle", "ring", "pinky")
        for stream in ("wrench", "deformation")
    }

    assert len(topics) == 32
    assert expected <= set(by_topic)
    assert all(by_topic[name].required for name in expected)
    assert all(by_topic[name].min_frequency_hz == 18.0 for name in expected)
    assert all(by_topic[name].max_gap_ms == 500.0 for name in expected)
    assert all(
        by_topic[name].type_name
        == (
            "geometry_msgs/msg/WrenchStamped"
            if name.endswith("/wrench")
            else "sensor_msgs/msg/Image"
        )
        for name in expected
    )
    assert "rosbag_record_default_qos" not in config
    qos_flag = config["rosbag_record_args"].index("--qos-profile-overrides-path")
    assert config["rosbag_record_args"].count("--qos-profile-overrides-path") == 1
    assert config["rosbag_record_args"][qos_flag + 1].endswith(
        "/data_collection/config/rosbag_qos_sharpa_tactile.yaml"
    )
    assert config["data_root"].endswith("/gello_sharpa_tactile")


def test_sharpa_tactile_rosbag_qos_matches_each_real_publisher():
    config = _sharpa_tactile_config()
    qos = yaml.safe_load(SHARPA_TACTILE_QOS.read_text(encoding="utf-8"))
    recorded = {
        raw["topic"]
        for section in ("topics", "static_topics")
        for raw in config[section].values()
    }

    assert set(qos) == recorded
    reliable = {
        "/teleop/validated_arm_commands",
        "/teleop/arm_command_status",
        "/left/franka/joint_states",
        "/right/franka/joint_states",
        "/sharpa/left/command",
        "/sharpa/right/command",
        "/cam0/color/image_raw",
        "/cam0/depth/image_raw",
        "/cam1/color/image_raw",
        "/cam2/color/image_raw",
        "/cam0/color/camera_info",
        "/cam0/depth/camera_info",
        "/cam1/color/camera_info",
        "/cam2/color/camera_info",
    }
    best_effort = recorded - reliable

    assert {
        "/sharpa/left/joint_states",
        "/sharpa/right/joint_states",
    } <= best_effort
    assert len(best_effort) == 22
    assert all(qos[topic]["reliability"] == "reliable" for topic in reliable)
    assert all(
        qos[topic]["reliability"] == "best_effort" for topic in best_effort
    )
    assert all(
        profile == {
            "history": "keep_last",
            "depth": 10,
            "reliability": profile["reliability"],
            "durability": "volatile",
        }
        for profile in qos.values()
    )


def test_telemetry_counters_use_episode_delta_not_process_lifetime_total():
    counters = {"velocity_sources": set()}
    _update_cumulative_counter(counters, "receiver_invalid_packets", 2)
    _update_cumulative_counter(counters, "receiver_invalid_packets", 2)
    assert counters["first_receiver_invalid_packets"] == 2
    assert counters["last_receiver_invalid_packets"] == 2
    assert counters["delta_receiver_invalid_packets"] == 0

    _update_cumulative_counter(counters, "receiver_invalid_packets", 4)
    assert counters["delta_receiver_invalid_packets"] == 2

    # Counter reset/session restart: a new value of one is one new event.
    _update_cumulative_counter(counters, "receiver_invalid_packets", 1)
    assert counters["delta_receiver_invalid_packets"] == 3


def test_timing_report_rejects_missing_low_rate_and_stale_interval():
    report = stream_timing_report(
        StreamSamples(
            bag_times_ns=[1_000_000_000, 1_040_000_000, 1_300_000_000],
            header_times_ns=[900_000_000, 940_000_000, 1_200_000_000],
        ),
        global_start_ns=1_000_000_000,
        global_end_ns=1_300_000_000,
        global_source_start_ns=900_000_000,
        global_source_end_ns=1_200_000_000,
    )
    failures = timing_failures(
        "camera", report, min_frequency_hz=10.0, max_gap_ms=150.0
    )
    assert any("below" in failure for failure in failures)
    assert any("source_max_internal_gap_ms" in failure for failure in failures)


def test_timing_report_separates_bag_congestion_from_source_continuity():
    report = stream_timing_report(
        StreamSamples(
            bag_times_ns=[
                1_000_000_000,
                1_010_000_000,
                1_260_000_000,
                1_300_000_000,
            ],
            header_times_ns=[
                900_000_000,
                950_000_000,
                1_000_000_000,
                1_050_000_000,
            ],
        ),
        global_start_ns=1_000_000_000,
        global_end_ns=1_300_000_000,
    )

    assert report["bag_timing"]["frequency_hz"] == pytest.approx(10.0)
    assert report["bag_timing"]["max_internal_gap_ms"] == 250.0
    assert report["source_timing"]["frequency_hz"] == pytest.approx(20.0)
    assert report["source_timing"]["max_internal_gap_ms"] == 50.0
    assert report["source_timing"]["nonmonotonic_count"] == 0
    failures = timing_failures(
        "camera", report, min_frequency_hz=18.0, max_gap_ms=150.0
    )
    assert failures == []
    assert transport_timing_warnings(
        "camera", report, max_gap_ms=150.0
    ) == ["camera: bag_max_internal_gap_ms 250.0 ms exceeds 150.0 ms"]


def test_recording_edges_use_bag_time_not_delayed_source_header():
    report = stream_timing_report(
        StreamSamples(
            bag_times_ns=[1_159_000_000, 1_209_000_000, 1_950_000_000],
            header_times_ns=[1_035_000_000, 1_085_000_000, 1_826_000_000],
        ),
        global_start_ns=1_000_000_000,
        global_end_ns=2_000_000_000,
        global_source_start_ns=1_000_000_000,
        global_source_end_ns=2_000_000_000,
    )

    # The source clock is 124 ms behind receive time. Its apparent 174 ms
    # trailing gap must not make an otherwise covered recording edge fail.
    assert report["source_timing"]["trailing_gap_ms"] == 174.0
    assert report["bag_timing"]["leading_gap_ms"] == 159.0
    failures = timing_failures(
        "camera", report, min_frequency_hz=18.0, max_gap_ms=150.0
    )
    assert not any("leading_gap" in item or "trailing_gap" in item for item in failures)


def test_recording_edge_beyond_gap_plus_one_period_is_rejected():
    report = stream_timing_report(
        StreamSamples(
            bag_times_ns=[1_210_000_000, 1_260_000_000],
            header_times_ns=[1_086_000_000, 1_136_000_000],
        ),
        global_start_ns=1_000_000_000,
        global_end_ns=1_300_000_000,
    )

    failures = timing_failures(
        "camera", report, min_frequency_hz=20.0, max_gap_ms=150.0
    )
    assert any("bag_leading_gap_ms" in item for item in failures)


def test_recording_edge_inside_configured_trim_is_only_a_warning():
    report = stream_timing_report(
        StreamSamples(
            bag_times_ns=[1_500_000_000, 1_550_000_000, 1_600_000_000],
            header_times_ns=[1_500_000_000, 1_550_000_000, 1_600_000_000],
        ),
        global_start_ns=1_000_000_000,
        global_end_ns=1_900_000_000,
    )

    failures = timing_failures(
        "camera",
        report,
        min_frequency_hz=20.0,
        max_gap_ms=150.0,
        trim_start_sec=1.0,
        trim_end_sec=1.0,
    )
    assert failures == []
    warnings = boundary_timing_warnings(
        "camera",
        report,
        min_frequency_hz=20.0,
        max_gap_ms=150.0,
        trim_start_sec=1.0,
        trim_end_sec=1.0,
    )
    assert len(warnings) == 2
    assert all("accepted by" in warning for warning in warnings)


@pytest.mark.parametrize("edge", ["leading_gap_ms", "trailing_gap_ms"])
@pytest.mark.parametrize("gap_ms, accepted", [(250.0, False), (450.0, True), (450.1, False), (6070.2, False)])
def test_boundary_warning_does_not_claim_failed_gap_is_accepted(edge, gap_ms, accepted):
    report = {"bag_timing": {"leading_gap_ms": 0.0, "trailing_gap_ms": 0.0}}
    report["bag_timing"][edge] = gap_ms
    warnings = boundary_timing_warnings(
        "arm", report, min_frequency_hz=10.0, max_gap_ms=150.0,
        trim_start_sec=0.2, trim_end_sec=0.2,
    )
    assert bool(warnings) is accepted


def test_action_gaps_are_allowed_only_during_explicit_disengagement():
    action_times = [100_000_000, 200_000_000, 800_000_000, 900_000_000]
    engagement = [
        (100_000_000, True),
        (200_000_000, True),
        (300_000_000, False),
        (700_000_000, False),
        (800_000_000, True),
        (900_000_000, True),
    ]
    assert not _engaged_action_failures(
        name="left action",
        engagement=engagement,
        action_times=action_times,
        max_gap_ms=150.0,
    )

    engagement[3] = (500_000_000, True)
    failures = _engaged_action_failures(
        name="left action",
        engagement=engagement,
        action_times=action_times,
        max_gap_ms=150.0,
    )
    assert any("engaged status samples have no action" in item for item in failures)


def test_engaged_action_check_ignores_startup_samples_outside_trim_window():
    # episode32: first engaged hand status is 21.4 ms before the first command.
    action_times = [
        1_021_400_000,
        1_121_400_000,
        2_000_000_000,
        3_000_000_000,
    ]
    engagement = [
        (1_000_000_000, True),
        (1_021_400_000, True),
        (1_121_400_000, True),
        (2_000_000_000, True),
        (3_000_000_000, True),
    ]
    failures = _engaged_action_failures(
        name="/teleop/wuji/left/command",
        engagement=engagement,
        action_times=action_times,
        max_gap_ms=150.0,
    )
    assert any("1 engaged status samples" in item for item in failures)
    assert not _engaged_action_failures(
        name="/teleop/wuji/left/command",
        engagement=engagement,
        action_times=action_times,
        max_gap_ms=150.0,
        window_start_ns=2_000_000_000,
        window_end_ns=3_000_000_000,
    )

    engagement.insert(-1, (2_500_000_000, True))
    failures = _engaged_action_failures(
        name="/teleop/wuji/left/command",
        engagement=engagement,
        action_times=action_times,
        max_gap_ms=150.0,
        window_start_ns=2_000_000_000,
        window_end_ns=3_000_000_000,
    )
    assert any("engaged status samples have no action" in item for item in failures)


def test_trimmed_engagement_window_can_match_a_pre_window_action():
    assert not _engaged_action_failures(
        name="/teleop/validated_arm_commands#left",
        engagement=[(1_000_000_000, True)],
        action_times=[950_000_000, 2_000_000_000],
        max_gap_ms=150.0,
        window_start_ns=1_000_000_000,
        window_end_ns=2_000_000_000,
    )


def test_partial_arm_side_is_rejected_and_inactive_side_is_not_invented():
    streams = {}
    counters = {
        "max_sender_dropped_packets": 0,
        "max_receiver_lost_packets": 0,
        "max_receiver_duplicate_packets": 0,
        "max_receiver_out_of_order_packets": 0,
        "max_receiver_stale_packets": 0,
        "max_receiver_invalid_packets": 0,
        "velocity_sources": set(),
    }
    partial = SimpleNamespace(
        name=list(LEFT_COMMAND_JOINT_NAMES[:-1]),
        position=[0.0] * 6,
    )
    with pytest.raises(ValueError, match="partial left"):
        _inspect_message_content(
            VALIDATED_COMMAND_TOPIC,
            partial,
            100,
            90,
            streams,
            counters,
        )

    complete = SimpleNamespace(
        name=list(LEFT_COMMAND_JOINT_NAMES),
        position=[0.0] * 7,
    )
    from collections import defaultdict

    actual_streams = defaultdict(StreamSamples)
    _inspect_message_content(
        VALIDATED_COMMAND_TOPIC,
        complete,
        100,
        90,
        actual_streams,
        counters,
    )
    assert actual_streams[f"{VALIDATED_COMMAND_TOPIC}#left"].bag_times_ns == [100]
    assert actual_streams[f"{VALIDATED_COMMAND_TOPIC}#right"].bag_times_ns == []


def test_head_depth_requires_native_packed_16uc1():
    streams = {}
    counters = {}
    valid = SimpleNamespace(
        width=640,
        height=400,
        encoding="16UC1",
        step=1280,
        data=bytes(640 * 400 * 2),
    )
    _inspect_message_content(
        "/cam0/depth/image_raw", valid, 100, 90, streams, counters
    )
    invalid = SimpleNamespace(
        width=640,
        height=400,
        encoding="32FC1",
        step=2560,
        data=bytes(640 * 400 * 4),
    )
    with pytest.raises(ValueError, match="16UC1"):
        _inspect_message_content(
            "/cam0/depth/image_raw", invalid, 100, 90, streams, counters
        )
