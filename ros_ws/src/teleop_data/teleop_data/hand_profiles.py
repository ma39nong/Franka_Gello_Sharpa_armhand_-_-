"""Dataset-facing hand layouts.

These profiles describe recorded vendor state/action vectors. They are kept
separate from the live bridge profiles so offline conversion remains ROS- and
hardware-driver-independent.
"""

from __future__ import annotations

from dataclasses import dataclass


G20_JOINT_NAMES = (
    "Thumb Base",
    "Index Finger Base",
    "Middle Finger Base",
    "Ring Finger Base",
    "Pinky Finger Base",
    "Thumb Abduction",
    "Index Finger Abduction",
    "Middle Finger Abduction",
    "Ring Finger Abduction",
    "Pinky Finger Abduction",
    "Thumb Horizontal Abduction",
    "Reserved 1",
    "Reserved 2",
    "Reserved 3",
    "Reserved 4",
    "Thumb Tip",
    "Index Finger Tip",
    "Middle Finger Tip",
    "Ring Finger Tip",
    "Pinky Finger Tip",
)

O30I_JOINT_NAMES = (
    "thumb_cmc_roll",
    "thumb_cmc_yaw",
    "thumb_mcp",
    "thumb_ip",
    "index_mcp_roll",
    "index_mcp_pitch",
    "index_pip",
    "index_dip",
    "middle_mcp_roll",
    "middle_mcp_pitch",
    "middle_pip",
    "middle_dip",
    "ring_mcp_roll",
    "ring_mcp_pitch",
    "ring_pip",
    "ring_dip",
    "pinky_mcp_roll",
    "pinky_mcp_pitch",
    "pinky_pip",
    "pinky_dip",
)
# 0803 vendor URDF limits (corrected from 0706; see o30i_contract.py)
O30I_RIGHT_LOWER = (
    0.0, 0.0, 0.0, 0.0,
    -0.3741, 0.0, 0.0, 0.0,
    -0.4906, 0.0, 0.0, 0.0,
    -0.1371, 0.0, 0.0, 0.0,
    -0.1835, 0.0, 0.0, 0.0,
)
O30I_RIGHT_UPPER = (
    0.5731, 1.9268, 1.5194, 1.5910,
    0.03711, 1.5050, 1.5765, 1.5016,
    0.05418, 1.6159, 1.5783, 1.5207,
    0.18823, 1.6596, 1.5009, 1.4986,
    0.2810, 1.6152, 1.4858, 1.5549,
)


@dataclass(frozen=True)
class HandDataProfile:
    model: str
    joint_names: tuple[str, ...]
    lower_bounds: tuple[float, ...]
    upper_bounds: tuple[float, ...]

    @property
    def width(self) -> int:
        return len(self.joint_names)


def create_hand_data_profile(model: str, *, side: str = "right") -> HandDataProfile:
    normalized = str(model).strip().lower()
    if normalized not in {"g20", "o30i"}:
        raise ValueError(
            f"unsupported recorded hand model {model!r}; "
            "registered models: ['g20', 'o30i']"
        )
    if normalized == "o30i":
        if side != "right":
            raise ValueError("the checked-in O30i data profile currently supports right only")
        return HandDataProfile(
            model="o30i",
            joint_names=O30I_JOINT_NAMES,
            lower_bounds=O30I_RIGHT_LOWER,
            upper_bounds=O30I_RIGHT_UPPER,
        )
    return HandDataProfile(
        model="g20",
        joint_names=G20_JOINT_NAMES,
        lower_bounds=(0.0,) * len(G20_JOINT_NAMES),
        upper_bounds=(255.0,) * len(G20_JOINT_NAMES),
    )


def default_hand_profiles() -> dict[str, HandDataProfile]:
    return {
        side: create_hand_data_profile("g20", side=side)
        for side in ("left", "right")
    }
