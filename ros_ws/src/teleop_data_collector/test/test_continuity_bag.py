"""Real serialized ROS bag checks, without cameras, robots or LeRobot conversion."""
import pytest

rosbag2_py = pytest.importorskip("rosbag2_py")
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image
from teleop_data_collector.bag_validation import inspect_bag
from teleop_data_collector.collector_contract import TopicContract


def write_camera_bag(path, hz, missing, *, congestion=False, separate_missing=False, gap_start=4):
    topic = "/cam0/color/image_raw" if hz == 20 else "/cam1/color/image_raw"
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type="sensor_msgs/msg/Image", serialization_format="cdr"))
    image = Image()
    image.height = 2
    image.width = 2
    image.step = 6
    image.encoding = "rgb8"
    image.data = bytes(12)
    skipped = {4 + 10 * i for i in range(missing)} if separate_missing else set(range(gap_start, gap_start + missing))
    for i in range(hz * 4):
        # Inject within the first second, which must not hide internal gaps.
        if i in skipped:
            continue
        stamp = 1_000_000_000 + round(i * 1_000_000_000 / hz)
        image.header.stamp.sec = stamp // 1_000_000_000
        image.header.stamp.nanosec = stamp % 1_000_000_000
        receive = stamp + (200_000_000 if congestion and i >= 4 else 0)
        writer.write(topic, serialize_message(image), receive)
    del writer
    return TopicContract("camera", topic, "sensor_msgs/msg/Image", min_frequency_hz=1)


@pytest.mark.parametrize("hz", [20, 30])
@pytest.mark.parametrize("missing", [0, 1, 2, 3])
def test_real_bag_uses_shared_policy_including_untrimmed_internal_gaps(tmp_path, hz, missing):
    path = tmp_path / "bag"
    contract = write_camera_bag(path, hz, missing)
    failures, report = inspect_bag(path, [contract], trim_start_sec=1, trim_end_sec=.2, continuity_enabled=True)
    assert bool(failures) == (missing >= 3)
    assert bool(report["source_warnings"]) == (0 < missing < 3)
    stats = report["continuity"]["streams"][contract.topic]
    assert stats["estimated_missing_frames_max_run"] == missing
    assert stats["source_gap_failure_count"] == int(missing == 3)


def test_transport_only_and_original_mode_compatibility(tmp_path):
    path = tmp_path / "congested"
    contract = write_camera_bag(path, 30, 0, congestion=True)
    failures, report = inspect_bag(path, [contract], continuity_enabled=True)
    assert not failures
    assert not report["source_warnings"]
    assert report["continuity"]["transport_warnings"]
    path = tmp_path / "legacy"
    contract = write_camera_bag(path, 30, 3)
    failures, report = inspect_bag(path, [contract])
    assert not failures  # 133 ms remains within the original 150 ms policy.
    assert report["continuity"] == {}


@pytest.mark.parametrize("hz", [20, 30])
def test_real_bag_three_separate_missing_frames_are_only_warnings(tmp_path, hz):
    path = tmp_path / "separate"
    contract = write_camera_bag(path, hz, 3, separate_missing=True)
    failures, report = inspect_bag(path, [contract], continuity_enabled=True)
    assert not failures
    stats = report["continuity"]["streams"][contract.topic]
    assert stats["estimated_missing_frames_total"] == 3
    assert stats["estimated_missing_frames_max_run"] == 1
    assert stats["source_gap_failure_count"] == 0
    assert stats["source_gap_warning_count"] == 3


@pytest.mark.parametrize('loss_event', [False, True])
def test_bag_driver_loss_event_fails_but_historical_counter_does_not(tmp_path, loss_event):
    import json
    from std_msgs.msg import String
    path = tmp_path/'driver_loss'
    image_topic = '/cam1/color/image_raw'
    health_topic = '/cam1/driver_health'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    for topic, cls in [(image_topic, 'sensor_msgs/msg/Image'), (health_topic, 'std_msgs/msg/String')]:
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=cls, serialization_format='cdr'))
    image = Image(height=2, width=2, step=6, encoding='rgb8', data=bytes(12))
    for i in range(120):
        stamp = 1_000_000_000 + round(i*1e9/30)
        image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        writer.write(image_topic, serialize_message(image), stamp)
        if i == 60:
            msg = String(data=json.dumps(dict(camera_uptime_sec=20, streams=[dict(
                stream='color', state='QUEUE_OVERFLOW' if loss_event else 'OK',
                queue_drop_total=1, last_queue_drop_steady_us=50_000_000)])))
            writer.write(health_topic, serialize_message(msg), stamp+1)
    del writer
    failures, report = inspect_bag(path, [
        TopicContract('camera', image_topic, 'sensor_msgs/msg/Image'),
        TopicContract('health', health_topic, 'std_msgs/msg/String', required=False)
    ], continuity_enabled=True)
    assert bool(failures) == loss_event
    if loss_event:
        assert report['continuity']['auto_stop_reason']['category'] == 'driver_queue'
        assert 1.9 < report['continuity']['auto_stop_reason']['recording_elapsed_sec'] < 2.1



def test_alignment_rejects_metadata_count_mismatch_even_when_grid_valid(tmp_path):
    import yaml
    from teleop_data_collector.alignment_quality import AlignmentPolicy
    path=tmp_path/'disk_counts';contract=write_camera_bag(path,30,0)
    policy=AlignmentPolicy(trim_start_sec=0,trim_end_sec=0)
    kwargs=dict(continuity_enabled=True,continuity_mode='conversion_aligned',alignment_quality=policy)
    failures,report=inspect_bag(path,[contract],**kwargs)
    assert not failures
    metadata=path/'metadata.yaml';data=yaml.safe_load(metadata.read_text())
    data['rosbag2_bagfile_information']['message_count']+=1
    metadata.write_text(yaml.safe_dump(data))
    failures,report=inspect_bag(path,[contract],**kwargs)
    assert report['alignment_quality']['first_failure'] is None
    assert any('message count' in f for f in failures)


@pytest.mark.parametrize('mode', ['legacy', 'conversion_aligned'])
@pytest.mark.parametrize('missing', [0, 15])
def test_tactile_switch_skips_missing_tactile_but_retains_camera_failures(tmp_path, mode, missing):
    path = tmp_path / 'switch'
    camera = write_camera_bag(path, 30, missing, gap_start=45)
    tactile = [TopicContract(f'{side}_{finger}_{kind}',
                f'/sharpa/{side}/tactile/{finger}/{kind}',
                'geometry_msgs/msg/WrenchStamped' if kind == 'wrench' else 'sensor_msgs/msg/Image')
               for side in ('left', 'right')
               for finger in ('thumb', 'index', 'middle', 'ring', 'little')
               for kind in ('wrench', 'deformation')]
    failures, report = inspect_bag(path, [camera, *tactile], continuity_enabled=True,
        continuity_mode=mode, tactile_detection_enabled=False)
    assert bool(failures) == bool(missing)
    assert report['tactile_detection_enabled'] is False
    assert not any('/tactile/' in t for t in report['streams'])
    assert not any('/tactile/' in t for t in report['continuity']['streams'])
    # Re-enabling the switch restores rejection of the same missing tactile topics.
    failures, _ = inspect_bag(path, [camera, *tactile], continuity_enabled=True,
        continuity_mode=mode, tactile_detection_enabled=True)
    assert failures
    assert any('/tactile/' in reason for reason in failures)
