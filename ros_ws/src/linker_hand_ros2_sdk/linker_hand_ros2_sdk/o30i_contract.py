"""Canonical O30i radians contract and vendor HOP ordering.

Everything outside the physical driver uses the URDF joint names and radians.
Only this package converts those values to the O30i firmware's 0..255 ticks.
"""

from __future__ import annotations

O30I_URDF_JOINT_NAMES = (
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

O30I_DRIVER_JOINT_NAMES = (
    "thumb_roll",
    "thumb_yaw",
    "index_yaw",
    "middle_yaw",
    "ring_yaw",
    "little_yaw",
    "thumb_root1",
    "index_root1",
    "middle_root1",
    "ring_root1",
    "little_root1",
    "index_root2",
    "middle_root2",
    "ring_root2",
    "little_root2",
    "thumb_tip",
    "index_tip",
    "middle_tip",
    "ring_tip",
    "little_tip",
)

O30I_DRIVER_TO_URDF = {
    "thumb_roll": "thumb_cmc_roll",
    "thumb_yaw": "thumb_cmc_yaw",
    "index_yaw": "index_mcp_roll",
    "middle_yaw": "middle_mcp_roll",
    "ring_yaw": "ring_mcp_roll",
    "little_yaw": "pinky_mcp_roll",
    "thumb_root1": "thumb_mcp",
    "index_root1": "index_mcp_pitch",
    "middle_root1": "middle_mcp_pitch",
    "ring_root1": "ring_mcp_pitch",
    "little_root1": "pinky_mcp_pitch",
    "index_root2": "index_pip",
    "middle_root2": "middle_pip",
    "ring_root2": "ring_pip",
    "little_root2": "pinky_pip",
    "thumb_tip": "thumb_ip",
    "index_tip": "index_dip",
    "middle_tip": "middle_dip",
    "ring_tip": "ring_dip",
    "little_tip": "pinky_dip",
}

# Vendor 0803 URDF release: same structure and meshes as 0706, corrected
# LIMITS on every joint. The tick map anchors on these, so the 0706 values
# systematically under-rotated the metal (up to ~15 deg near range edges) --
# the sim-to-real pinch gap, closed on the metal 2026-08-04.
O30I_RIGHT_LOWER = (
    0.0,
    0.0,
    0.0,
    0.0,
    -0.3741,
    0.0,
    0.0,
    0.0,
    -0.4906,
    0.0,
    0.0,
    0.0,
    -0.1371,
    0.0,
    0.0,
    0.0,
    -0.1835,
    0.0,
    0.0,
    0.0,
)

O30I_RIGHT_UPPER = (
    0.5731,
    1.9268,
    1.5194,
    1.5910,
    0.03711,
    1.5050,
    1.5765,
    1.5016,
    0.05418,
    1.6159,
    1.5783,
    1.5207,
    0.18823,
    1.6596,
    1.5009,
    1.4986,
    0.2810,
    1.6152,
    1.4858,
    1.5549,
)


def radians_to_ticks(
    value: float,
    lower: float,
    upper: float,
    tick_at_lower: float,
    tick_at_upper: float,
) -> int:
    unit = (float(value) - lower) / (upper - lower)
    tick = tick_at_lower + unit * (tick_at_upper - tick_at_lower)
    return int(round(min(255.0, max(0.0, tick))))


def ticks_to_radians(
    tick: float,
    lower: float,
    upper: float,
    tick_at_lower: float,
    tick_at_upper: float,
) -> float:
    unit = (float(tick) - tick_at_lower) / (tick_at_upper - tick_at_lower)
    return lower + unit * (upper - lower)
