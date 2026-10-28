"""Guard the bridge's O30i limits against drifting from the URDF.

The O30i joint limits exist in three places by design: the URDF (which the
retargeter clips to), this bridge profile (which REJECTS out-of-limit packets),
and the driver contract. Drift between them is silent at runtime - the hand
just freezes while the bridge counts invalid packets - so it must fail here.
"""

from pathlib import Path
from xml.etree import ElementTree

import pytest

from linker_hand_bridge.profiles import (
    O30I_JOINT_NAMES,
    O30I_RIGHT_LOWER,
    O30I_RIGHT_UPPER,
)

ASSETS = Path(__file__).resolve().parents[4] / "assets" / "linkerhand_o30i"
URDF_PATH = ASSETS / "right" / "linkerhand_o30i_right.urdf"
LEFT_URDF_PATH = ASSETS / "left" / "linkerhand_o30i_left.urdf"


def _urdf_limits(path: Path) -> dict[str, tuple[float, float]]:
    root = ElementTree.parse(path).getroot()
    limits = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is not None:
            limits[joint.attrib["name"]] = (
                float(limit.attrib["lower"]),
                float(limit.attrib["upper"]),
            )
    return limits


def test_profile_limits_match_urdf():
    if not URDF_PATH.is_file():
        pytest.skip(f"O30i URDF not available at {URDF_PATH}")
    limits = _urdf_limits(URDF_PATH)
    assert set(limits) == set(O30I_JOINT_NAMES)
    for name, lower, upper in zip(
        O30I_JOINT_NAMES, O30I_RIGHT_LOWER, O30I_RIGHT_UPPER, strict=True
    ):
        assert limits[name][0] == pytest.approx(lower, abs=1e-9), name
        assert limits[name][1] == pytest.approx(upper, abs=1e-9), name


def test_left_urdf_shares_the_right_contract():
    """Both sides run one profile: the vendor's left URDF must carry the
    same joint names and limits (mirroring lives in link geometry)."""
    if not LEFT_URDF_PATH.is_file():
        pytest.skip(f"O30i left URDF not available at {LEFT_URDF_PATH}")
    assert _urdf_limits(LEFT_URDF_PATH) == _urdf_limits(URDF_PATH)


def test_left_profile_is_served():
    from linker_hand_bridge.profiles import create_hand_profile

    left = create_hand_profile("o30i", side="left")
    right = create_hand_profile("o30i", side="right")
    assert left.lower_bounds == right.lower_bounds
    assert left.upper_bounds == right.upper_bounds
    assert left.command_joint_names == right.command_joint_names
    assert left.mapper.home("left") == right.mapper.home("right")
