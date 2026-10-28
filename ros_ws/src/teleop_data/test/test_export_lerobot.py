import numpy as np

from teleop_data.export_lerobot import _age, features
from teleop_data.hand_profiles import HandDataProfile


def test_lerobot_features_include_arms_hands_rgb_depth_and_calibration():
    result = features((800, 1280, 3), (800, 1280, 1))
    assert result["observation.state"]["shape"] == (54,)
    assert result["action"]["shape"] == (54,)
    assert result["observation.state"]["names"][0].startswith("left_fr3")
    assert result["observation.state"]["names"][14].startswith("left_linker_hand")
    assert result["observation.state"]["names"][34].startswith("right_linker_hand")
    assert result["observation.active_sides"]["shape"] == (2,)
    assert result["observation.images.orbbec"]["shape"] == (800, 1280, 3)
    depth = result["observation.images.orbbec_depth"]
    assert depth["shape"] == (800, 1280, 1)
    assert depth["info"]["depth_unit"] == "mm"
    assert result["observation.camera.orbbec_intrinsics"]["shape"] == (9,)


def test_camera_age_is_nonnegative_float32():
    np.testing.assert_array_equal(_age(2.0, 1.75), np.asarray([250], np.float32))
    np.testing.assert_array_equal(_age(1.0, 1.1), np.asarray([0], np.float32))


def test_lerobot_features_follow_configured_hand_widths():
    profiles = {
        "left": HandDataProfile("test2", ("a", "b"), (0.0,) * 2, (1.0,) * 2),
        "right": HandDataProfile("test1", ("c",), (0.0,), (1.0,)),
    }
    result = features((8, 8, 3), (8, 8, 1), profiles)
    assert result["observation.state"]["shape"] == (17,)
    assert result["action"]["shape"] == (17,)
