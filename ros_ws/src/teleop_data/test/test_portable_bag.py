from types import SimpleNamespace

import numpy as np
import pytest

from teleop_data.portable_bag import (
    G20_JOINT_NAMES,
    camera_info_to_arrays,
    image_to_depth,
    image_to_rgb,
    ordered_hand_positions,
    recording_topics,
)


def test_image_to_rgb_handles_stride_and_bgr():
    message = SimpleNamespace(
        encoding="bgr8",
        height=1,
        width=2,
        step=8,
        data=np.asarray([3, 2, 1, 6, 5, 4, 99, 99], dtype=np.uint8),
    )
    image = image_to_rgb(message)
    np.testing.assert_array_equal(
        image,
        np.asarray([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8),
    )
    assert image.flags.c_contiguous


def test_image_to_depth_handles_row_padding():
    values = np.asarray([1, 256], dtype="<u2").view(np.uint8)
    message = SimpleNamespace(
        encoding="16UC1",
        is_bigendian=False,
        height=1,
        width=2,
        step=6,
        data=np.concatenate((values, np.asarray([9, 9], dtype=np.uint8))),
    )
    depth = image_to_depth(message)
    np.testing.assert_array_equal(
        depth,
        np.asarray([[[1], [256]]], dtype=np.uint16),
    )


def test_image_converters_reject_unsupported_encodings():
    color = SimpleNamespace(encoding="mono8")
    depth = SimpleNamespace(encoding="32FC1")
    with pytest.raises(ValueError, match="Unsupported color encoding"):
        image_to_rgb(color)
    with pytest.raises(ValueError, match="Unsupported depth encoding"):
        image_to_depth(depth)


def test_camera_info_is_padded_to_eight_distortion_values():
    message = SimpleNamespace(k=list(range(9)), d=[0.1, 0.2])
    intrinsic, distortion = camera_info_to_arrays(message)
    assert intrinsic.shape == (9,)
    assert distortion.shape == (8,)
    np.testing.assert_allclose(distortion[:2], [0.1, 0.2])
    np.testing.assert_allclose(distortion[2:], 0.0)


def test_recording_topics_requires_complete_camera_contract():
    specs = [
        SimpleNamespace(name="camera_color", topic="/color"),
        SimpleNamespace(name="camera_color_info", topic="/color_info"),
        SimpleNamespace(name="camera_depth", topic="/depth"),
    ]
    with pytest.raises(ValueError, match="left_hand_action"):
        recording_topics(SimpleNamespace(topics=specs))


def test_ordered_hand_positions_uses_canonical_g20_order():
    names = list(reversed(G20_JOINT_NAMES))
    positions = list(range(20))
    result = ordered_hand_positions(names, positions)
    np.testing.assert_array_equal(result, list(reversed(range(20))))


def test_ordered_hand_positions_rejects_partial_vendor_initializer():
    with pytest.raises(ValueError, match="missing"):
        ordered_hand_positions(G20_JOINT_NAMES[:10], [0.0] * 10)
