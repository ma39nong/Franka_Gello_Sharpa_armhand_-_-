import json
from pathlib import Path

import pytest


rosbag2_py = pytest.importorskip("rosbag2_py")
pytest.importorskip("pyarrow")

from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, JointState
from teleop_core.contract import (
    COMMAND_STATUS_TOPIC,
    DATA_LEFT_ARM_JOINT_NAMES,
    DATA_RIGHT_ARM_JOINT_NAMES,
    LEFT_COMMAND_JOINT_NAMES,
    RIGHT_COMMAND_JOINT_NAMES,
    VALIDATED_COMMAND_TOPIC,
    WUJI_COMMAND_TOPIC,
    WUJI_LEFT_JOINT_NAMES,
    WUJI_RIGHT_JOINT_NAMES,
    WUJI_STATE_TOPIC,
    WUJI_TELEMETRY_STATUS_TOPIC,
)
from teleop_interfaces.msg import ArmCommandStatus, HandTelemetryStatus
from teleop_data_collector.rosbag_to_lerobot import (
    _source_tree_snapshot,
    main,
)


REPO_ROOT = Path(__file__).resolve().parents[4]
CONFIG = REPO_ROOT / "data_collection/config/convert_gello_lerobot_v2.yaml"


def _joint_state(
    names: tuple[str, ...] | list[str],
    positions: list[float],
    velocities: list[float] | None = None,
) -> JointState:
    message = JointState()
    message.name = list(names)
    message.position = positions
    message.velocity = velocities if velocities is not None else [0.01] * len(names)
    return message


def _image(value: int) -> Image:
    message = Image()
    message.height = 24
    message.width = 32
    message.encoding = "rgb8"
    message.step = message.width * 3
    message.data = bytes([value % 255]) * (message.height * message.step)
    return message


def _depth_image(value: int) -> Image:
    message = Image()
    message.height = 24
    message.width = 32
    message.encoding = "16UC1"
    message.step = message.width * 2
    pixel = int(value).to_bytes(2, byteorder="little", signed=False)
    message.data = pixel * (message.height * message.width)
    return message


def _set_header_time(message, timestamp_ns: int) -> None:
    message.header.stamp.sec = timestamp_ns // 1_000_000_000
    message.header.stamp.nanosec = timestamp_ns % 1_000_000_000


def _write_synthetic_bag(path: Path, *, congested_bag_time: bool = False) -> None:
    # Deliberately put the validated command sides in right/left source order.
    # Conversion must select by name and emit the fixed left/right contract order.
    arm_status = ArmCommandStatus()
    arm_status.accepted_sides = ["left", "right"]
    hand_status = HandTelemetryStatus()
    hand_status.engaged = True
    messages = {
        COMMAND_STATUS_TOPIC: arm_status,
        VALIDATED_COMMAND_TOPIC: _joint_state(
            [*RIGHT_COMMAND_JOINT_NAMES, *LEFT_COMMAND_JOINT_NAMES],
            [*map(float, range(10, 17)), *map(float, range(0, 7))],
        ),
        WUJI_COMMAND_TOPIC.format(side="left"): _joint_state(
            WUJI_LEFT_JOINT_NAMES, list(map(float, range(20, 40)))
        ),
        WUJI_COMMAND_TOPIC.format(side="right"): _joint_state(
            WUJI_RIGHT_JOINT_NAMES, list(map(float, range(40, 60)))
        ),
        "/left/franka/joint_states": _joint_state(
            DATA_LEFT_ARM_JOINT_NAMES,
            list(map(float, range(100, 107))),
            list(map(float, range(110, 117))),
        ),
        "/right/franka/joint_states": _joint_state(
            DATA_RIGHT_ARM_JOINT_NAMES,
            list(map(float, range(120, 127))),
            list(map(float, range(130, 137))),
        ),
        WUJI_STATE_TOPIC.format(side="left"): _joint_state(
            WUJI_LEFT_JOINT_NAMES,
            list(map(float, range(140, 160))),
            list(map(float, range(160, 180))),
        ),
        WUJI_STATE_TOPIC.format(side="right"): _joint_state(
            WUJI_RIGHT_JOINT_NAMES,
            list(map(float, range(180, 200))),
            list(map(float, range(200, 220))),
        ),
        WUJI_TELEMETRY_STATUS_TOPIC: hand_status,
        "/cam0/color/image_raw": _image(32),
        "/cam0/depth/image_raw": _depth_image(1000),
        "/cam1/color/image_raw": _image(64),
        "/cam2/color/image_raw": _image(96),
    }

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {}
    for topic, message in messages.items():
        if isinstance(message, Image):
            type_name = "sensor_msgs/msg/Image"
        elif isinstance(message, JointState):
            type_name = "sensor_msgs/msg/JointState"
        elif isinstance(message, ArmCommandStatus):
            type_name = "teleop_interfaces/msg/ArmCommandStatus"
        else:
            type_name = "teleop_interfaces/msg/HandTelemetryStatus"
        topic_types[topic] = type_name
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=topic,
                type=type_name,
                serialization_format="cdr",
            )
        )
    start_ns = 1_000_000_000
    for frame in range(8):
        timestamp_ns = start_ns + frame * 33_333_333
        bag_time_ns = timestamp_ns
        if congested_bag_time and frame >= 3:
            bag_time_ns = start_ns + 316_666_666 + (frame - 3) * 1_000_000
        for topic, message in messages.items():
            if topic == WUJI_TELEMETRY_STATUS_TOPIC:
                for side_index, side in enumerate(("left", "right")):
                    message.side = side
                    _set_header_time(message, timestamp_ns - 2 + side_index)
                    writer.write(topic, serialize_message(message), bag_time_ns)
            else:
                _set_header_time(message, timestamp_ns)
                writer.write(topic, serialize_message(message), bag_time_ns)
    del writer

    state = {
        "bag_contract_version": 1,
        "state": "finalized",
        "finalized": True,
        "teleoperator": "gello",
        "source_bag": str(path),
        "workcell_id": "synthetic-dual-fr3-wuji",
        "workcell_config_hash": "synthetic-workcell-sha256",
        "control_config_id": "synthetic-gello-v1",
        "calibration_ids": {"arms": "synthetic", "hands": "synthetic"},
        "device_identities": {"workcell": "synthetic"},
        "source_topic_contract": topic_types,
        "timestamp_policy": "source_header_and_bag_receive_time",
    }
    (path / "collection_state.json").write_text(
        json.dumps(state, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def test_synthetic_finalized_bag_converts_atomically_to_exact_lerobot_v2(
    tmp_path: Path,
) -> None:
    import pyarrow.parquet as pq

    bag = tmp_path / "episode_000000"
    output = tmp_path / "lerobot_dataset"
    _write_synthetic_bag(bag)
    source_before = _source_tree_snapshot([bag])

    main([
        "--config", str(CONFIG), "--trim-start-sec", "0", "--trim-end-sec", "0",
        str(bag), "--output", str(output),
    ])

    assert _source_tree_snapshot([bag]) == source_before
    info = json.loads((output / "meta/info.json").read_text(encoding="utf-8"))
    metadata = json.loads(
        (output / "meta/conversion_metadata.json").read_text(encoding="utf-8")
    )
    assert info["codebase_version"] == "v2.0"
    assert info["teleoperator"] == "gello"
    assert info["features"]["action"]["shape"] == [54]
    assert info["features"]["observation.state"]["shape"] == [108]
    assert info["features"]["observation.engaged"]["shape"] == [4]
    assert set(metadata["episodes"][0]["video_paths"]) == {
        "observation.images.cam0",
        "observation.images.cam1",
        "observation.images.cam2",
    }
    assert set(metadata["episodes"][0]["depth_image_storage"]) == {
        "observation.depths.cam0",
    }
    depth_feature = info["features"]["observation.depths.cam0"]
    assert depth_feature["dtype"] == "image"
    assert depth_feature["info"]["image.format"] == "raw16"
    assert depth_feature["info"]["image.fps"] == 20
    assert depth_feature["info"]["image.is_depth_map"] is True
    assert "unmatched_by_stream" in metadata["episodes"][0]
    assert metadata["sampling"]["alignment_clock"] == "source_header"

    parquet_path = next((output / "data").rglob("*.parquet"))
    table = pq.read_table(
        parquet_path,
        columns=[
            "action",
            "observation.state",
            "observation.engaged",
            "observation.depths.cam0",
        ],
    )
    action = table["action"][0].as_py()
    state = table["observation.state"][0].as_py()
    engaged = table["observation.engaged"][0].as_py()
    depth = table["observation.depths.cam0"][0].as_py()
    assert depth["encoding"] == "16UC1"
    assert depth["format"] == "raw16"
    assert len(depth["data"]) == 24 * 32 * 2
    assert action == [
        *map(float, range(0, 7)),
        *map(float, range(10, 17)),
        *map(float, range(20, 40)),
        *map(float, range(40, 60)),
    ]
    assert engaged == [1.0, 1.0, 1.0, 1.0]
    assert state == [
        *map(float, range(100, 107)),
        *map(float, range(110, 117)),
        *map(float, range(120, 127)),
        *map(float, range(130, 137)),
        *map(float, range(140, 160)),
        *map(float, range(160, 180)),
        *map(float, range(180, 200)),
        *map(float, range(200, 220)),
    ]


def test_source_time_conversion_ignores_bag_receive_congestion(tmp_path: Path) -> None:
    import pyarrow.parquet as pq

    normal_bag = tmp_path / "episode_normal"
    congested_bag = tmp_path / "episode_congested"
    normal_output = tmp_path / "normal_dataset"
    congested_output = tmp_path / "congested_dataset"
    _write_synthetic_bag(normal_bag)
    _write_synthetic_bag(congested_bag, congested_bag_time=True)

    main([
        "--config", str(CONFIG), "--trim-start-sec", "0", "--trim-end-sec", "0",
        str(normal_bag), "--output", str(normal_output),
    ])
    main(
        [
            "--config",
            str(CONFIG),
            "--trim-start-sec",
            "0",
            "--trim-end-sec",
            "0",
            str(congested_bag),
            "--output",
            str(congested_output),
        ]
    )

    normal_table = pq.read_table(next((normal_output / "data").rglob("*.parquet")))
    congested_table = pq.read_table(
        next((congested_output / "data").rglob("*.parquet"))
    )
    assert normal_table.to_pylist() == congested_table.to_pylist()
    assert (congested_output.stat().st_mode & 0o777) == 0o755


def test_conversion_trims_source_edges_and_rebases_timestamp_to_zero(
    tmp_path: Path,
) -> None:
    import pyarrow.parquet as pq

    bag = tmp_path / "episode_trimmed"
    output = tmp_path / "trimmed_dataset"
    _write_synthetic_bag(bag)

    main(
        [
            "--config",
            str(CONFIG),
            "--trim-start-sec",
            "0.05",
            "--trim-end-sec",
            "0.05",
            str(bag),
            "--output",
            str(output),
        ]
    )

    table = pq.read_table(
        next((output / "data").rglob("*.parquet")), columns=["timestamp"]
    )
    timestamps = table["timestamp"].to_pylist()
    metadata = json.loads(
        (output / "meta/conversion_metadata.json").read_text(encoding="utf-8")
    )
    episode = metadata["episodes"][0]
    assert timestamps[0] == pytest.approx(0.0)
    assert timestamps[-1] <= 0.133_333_331
    assert metadata["sampling"]["trim_start_ns"] == 50_000_000
    assert metadata["sampling"]["trim_end_ns"] == 50_000_000
    assert episode["source_effective_start_ns"] == (
        episode["source_common_start_ns"] + 50_000_000
    )
    assert episode["source_effective_end_ns"] == (
        episode["source_common_end_ns"] - 50_000_000
    )


def test_failed_conversion_does_not_publish_partial_output(tmp_path: Path) -> None:
    bag = tmp_path / "episode_000000"
    output = tmp_path / "failed_dataset"
    _write_synthetic_bag(bag)
    state_path = bag / "collection_state.json"
    state = json.loads(state_path.read_text(encoding="utf-8"))
    state["state"] = "incomplete"
    state["finalized"] = False
    state_path.write_text(json.dumps(state), encoding="utf-8")
    source_before = _source_tree_snapshot([bag])

    with pytest.raises(SystemExit, match="not finalized"):
        main(["--config", str(CONFIG), str(bag), "--output", str(output)])

    assert _source_tree_snapshot([bag]) == source_before
    assert not output.exists()


@pytest.mark.parametrize("segments,expected_count", [("all", 2), ("full", 1), ("milestones", 1)])
def test_milestone_exports_prefix_and_full_with_source_clock_and_separate_tasks(
    tmp_path: Path, segments: str, expected_count: int,
) -> None:
    import cv2
    import pyarrow.parquet as pq
    import yaml

    bag = tmp_path / "episode_marked"
    output = tmp_path / "dataset"
    _write_synthetic_bag(bag, congested_bag_time=True)
    state_path = bag / "collection_state.json"
    state = json.loads(state_path.read_text())
    marker_ns = 1_150_000_000
    state["source_recording_id"] = "synthetic-recording-id"
    state["milestones"] = [{"id": "milestone_1", "timestamp_ns": marker_ns, "clock": "ros"}]
    state_path.write_text(json.dumps(state))
    source_before = _source_tree_snapshot([bag])
    config = yaml.safe_load(CONFIG.read_text())
    config["segment_tasks"] = {"milestone_1": "Pick up", "full": "Pick up and place"}
    config_path = tmp_path / "conversion.yaml"
    config_path.write_text(yaml.safe_dump(config))

    main([
        "--config", str(config_path), "--trim-start-sec", "0.02", "--trim-end-sec", "0.05",
        "--segments", segments, str(bag), "--output", str(output),
    ])

    metadata = json.loads((output / "meta/conversion_metadata.json").read_text())
    info = json.loads((output / "meta/info.json").read_text())
    tasks = [json.loads(line) for line in (output / "meta/tasks.jsonl").read_text().splitlines()]
    assert info["total_episodes"] == expected_count
    assert info["total_tasks"] == expected_count
    assert metadata["source_bags"] == [str(bag)]
    assert _source_tree_snapshot([bag]) == source_before
    tables = {}
    for episode in metadata["episodes"]:
        table = pq.read_table(output / episode["data_path"]).to_pylist()
        tables[episode["segment_id"]] = table
        assert episode["source_recording_id"] == "synthetic-recording-id"
        assert table[0]["timestamp"] == 0
        assert [row["next.done"] for row in table] == [False] * (len(table) - 1) + [True]
        assert {row["episode_index"] for row in table} == {episode["episode_index"]}
        assert {row["task_index"] for row in table} == {episode["task_index"]}
        if episode["segment_id"] == "milestone_1":
            assert episode["source_effective_end_ns"] == marker_ns
            assert episode["milestone_timestamp_ns"] == marker_ns
            assert len(table) == 4
            assert tasks[episode["task_index"]]["task"] == "Pick up"
            for video_path in episode["video_paths"].values():
                video = cv2.VideoCapture(str(output / video_path))
                try:
                    assert int(video.get(cv2.CAP_PROP_FRAME_COUNT)) == 5
                finally:
                    video.release()
        else:
            assert episode["source_effective_end_ns"] == episode["source_common_end_ns"] - 50_000_000
            assert len(table) == 5
            assert tasks[episode["task_index"]]["task"] == "Pick up and place"
    if segments == "all":
        for short_row, full_row in zip(tables["milestone_1"], tables["full"]):
            for column in ("timestamp", "action", "observation.state", "observation.depths.cam0"):
                assert short_row[column] == full_row[column]


@pytest.mark.parametrize("marker_ns", [1, 1_999_000_000])
def test_out_of_range_milestone_does_not_publish_output(tmp_path: Path, marker_ns: int) -> None:
    bag = tmp_path / "episode_invalid_marker"
    output = tmp_path / "dataset"
    _write_synthetic_bag(bag)
    path = bag / "collection_state.json"
    state = json.loads(path.read_text())
    state["milestones"] = [{"id": "milestone_1", "timestamp_ns": marker_ns, "clock": "ros"}]
    path.write_text(json.dumps(state))
    source_before = _source_tree_snapshot([bag])
    with pytest.raises(ValueError, match="outside.*usable source interval"):
        main([
            "--config", str(CONFIG), "--trim-start-sec", "0", "--trim-end-sec", "0",
            str(bag), "--output", str(output),
        ])
    assert _source_tree_snapshot([bag]) == source_before
    assert not output.exists()
    assert not list(tmp_path.glob(".dataset.tmp-*"))
