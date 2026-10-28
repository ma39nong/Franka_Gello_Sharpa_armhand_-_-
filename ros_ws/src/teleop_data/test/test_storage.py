from pathlib import Path

import pytest
import yaml

from teleop_data.storage import bag_topic_counts


def test_bag_topic_counts_reads_rosbag_metadata(tmp_path):
    metadata = {
        "rosbag2_bagfile_information": {
            "topics_with_message_count": [
                {
                    "topic_metadata": {
                        "name": "/camera/color/image_raw",
                        "type": "sensor_msgs/msg/Image",
                    },
                    "message_count": 17,
                },
                {
                    "topic_metadata": {
                        "name": "/camera/depth/image_raw",
                        "type": "sensor_msgs/msg/Image",
                    },
                    "message_count": 16,
                },
            ]
        }
    }
    (tmp_path / "metadata.yaml").write_text(
        yaml.safe_dump(metadata), encoding="utf-8"
    )
    assert bag_topic_counts(tmp_path) == {
        "/camera/color/image_raw": 17,
        "/camera/depth/image_raw": 16,
    }


def test_bag_topic_counts_rejects_missing_metadata(tmp_path):
    with pytest.raises(ValueError, match="metadata is missing"):
        bag_topic_counts(tmp_path)


def test_bag_topic_counts_rejects_invalid_metadata(tmp_path):
    (tmp_path / "metadata.yaml").write_text("other: value\n", encoding="utf-8")
    with pytest.raises(ValueError, match="Invalid rosbag metadata"):
        bag_topic_counts(tmp_path)
