from collections import defaultdict

import pytest

pytest.importorskip("rclpy")
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image

from teleop_data_collector.image_metadata import read_image_metadata
from teleop_data_collector.bag_validation import StreamSamples, _inspect_message_content


@pytest.mark.parametrize("frame_id", ["", "a", "ab", "abc", "camera_optical_frame", "相机"])
@pytest.mark.parametrize("encoding,step", [("rgb8", 1920), ("16UC1", 1280)])
@pytest.mark.parametrize("pixel_endian", [0, 1])
def test_metadata_matches_ros_decoder_without_copying_pixels(frame_id, encoding, step, pixel_endian):
    message = Image()
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    message.header.frame_id = frame_id
    message.height = 400
    message.width = 640
    message.encoding = encoding
    message.is_bigendian = pixel_endian
    message.step = step
    message.data = bytes(step * message.height)
    serialized = serialize_message(message)
    expected = deserialize_message(serialized, Image)
    actual = read_image_metadata(serialized)
    assert actual.header_time_ns == 123_000_000_456
    for field in ("height", "width", "encoding", "is_bigendian", "step"):
        assert getattr(actual, field) == getattr(expected, field)
    assert bytes(actual.data) == bytes(expected.data)
    assert actual.data.obj is serialized
    if encoding == "16UC1":
        _inspect_message_content(
            "/cam0/depth/image_raw", actual, 1, 1,
            defaultdict(StreamSamples), {},
        )


def test_big_endian_cdr_fixture():
    # CDR_BE, stamp=(2,3), empty frame_id, 1x2 mono8, step=2, two pixels.
    serialized = bytes.fromhex(
        "00000000 00000002 00000003 00000001 00000000 "
        "00000001 00000002 00000006 6d6f6e6f3800 00 00 "
        "00000002 00000002 1122"
    )
    actual = read_image_metadata(serialized)
    assert actual.header_time_ns == 2_000_000_003
    assert (actual.height, actual.width, actual.encoding, actual.step) == (1, 2, "mono8", 2)
    assert bytes(actual.data) == b"\x11\x22"


def test_every_truncated_prefix_is_rejected_including_missing_pixels():
    message = Image()
    message.header.frame_id = "optical_frame"
    message.height = 1
    message.width = 2
    message.encoding = "rgb8"
    message.step = 6
    message.data = b"abcdef"
    serialized = serialize_message(message)
    for length in range(len(serialized)):
        with pytest.raises(ValueError):
            read_image_metadata(serialized[:length])


def test_unknown_cdr_representation_requests_standard_decoder():
    assert read_image_metadata(b"\x00\x07\x00\x00") is None


def test_missing_timestamp_and_invalid_depth_layout_keep_existing_policy():
    message = Image()
    message.height = 400
    message.width = 640
    message.encoding = "16UC1"
    message.step = 1280
    message.data = b"short but correctly serialized sequence"
    actual = read_image_metadata(serialize_message(message))
    assert actual.header_time_ns is None
    with pytest.raises(ValueError, match="byte layout"):
        _inspect_message_content(
            "/cam0/depth/image_raw", actual, 1, None,
            defaultdict(StreamSamples), {},
        )


@pytest.mark.parametrize("truncate_pixels", [False, True])
def test_bag_validation_checks_each_image_without_ros_pixel_decoder(tmp_path, monkeypatch, truncate_pixels):
    rosbag2_py = pytest.importorskip("rosbag2_py")
    import rclpy.serialization
    from teleop_data_collector.bag_validation import inspect_bag
    from teleop_data_collector.collector_contract import TopicContract

    bag = tmp_path / "images"
    topic = "/cam0/depth/image_raw"
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=topic, type="sensor_msgs/msg/Image", serialization_format="cdr",
    ))
    message = Image()
    message.height, message.width, message.step = 400, 640, 1280
    message.encoding = "16UC1"
    message.data = bytes(400 * 1280)
    for nanosec in (0, 50_000_000, 100_000_000):
        message.header.stamp.sec = 1
        message.header.stamp.nanosec = nanosec
        serialized = serialize_message(message)
        if truncate_pixels and nanosec == 50_000_000:
            serialized = serialized[:-1]
        writer.write(topic, serialized, 1_000_000_000 + nanosec)
    del writer

    def forbidden_decoder(*_args):
        raise AssertionError("Image pixels must not be deserialized")

    monkeypatch.setattr(rclpy.serialization, "deserialize_message", forbidden_decoder)
    failures, report = inspect_bag(bag, [TopicContract(
        "depth", topic, "sensor_msgs/msg/Image", min_frequency_hz=18.0,
    )])
    assert report["streams"][topic]["message_count"] == 3
    if truncate_pixels:
        assert any(topic in item and "truncated" in item for item in failures)
    else:
        assert failures == ()
        assert report["streams"][topic]["source_timing"]["frequency_hz"] == 20.0
