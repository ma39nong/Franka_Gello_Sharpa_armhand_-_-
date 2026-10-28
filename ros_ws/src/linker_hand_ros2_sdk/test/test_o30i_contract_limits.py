"""Guard the driver contract's O30i limits against drifting from the URDF.

The O30i joint limits exist in three places by design: the URDF (which the
retargeter clips to), the bridge profile, and this driver contract (which the
tick conversion is anchored on). Drift is silent at runtime, so it must fail
here.
"""

from pathlib import Path
from xml.etree import ElementTree

import pytest

from linker_hand_ros2_sdk.o30i_contract import (
    O30I_RIGHT_LOWER,
    O30I_RIGHT_UPPER,
    O30I_URDF_JOINT_NAMES,
)

URDF_PATH = (
    Path(__file__).resolve().parents[4]
    / "assets"
    / "linkerhand_o30i"
    / "right"
    / "linkerhand_o30i_right.urdf"
)


def _urdf_limits() -> dict[str, tuple[float, float]]:
    root = ElementTree.parse(URDF_PATH).getroot()
    limits = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is not None:
            limits[joint.attrib["name"]] = (
                float(limit.attrib["lower"]),
                float(limit.attrib["upper"]),
            )
    return limits


def test_contract_limits_match_urdf():
    if not URDF_PATH.is_file():
        pytest.skip(f"O30i URDF not available at {URDF_PATH}")
    limits = _urdf_limits()
    assert set(limits) == set(O30I_URDF_JOINT_NAMES)
    for name, lower, upper in zip(
        O30I_URDF_JOINT_NAMES, O30I_RIGHT_LOWER, O30I_RIGHT_UPPER, strict=True
    ):
        assert limits[name][0] == pytest.approx(lower, abs=1e-9), name
        assert limits[name][1] == pytest.approx(upper, abs=1e-9), name
