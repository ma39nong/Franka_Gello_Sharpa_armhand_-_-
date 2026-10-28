"""Real-time robot state visualization using robot hardware interfaces."""

from __future__ import annotations

from barmate.core.interfaces import RobotInterface
from barmate.core.types import RobotObservation


def snapshot_robot_state(robot: RobotInterface) -> RobotObservation:
    """Read one state snapshot for future visualization code."""

    return robot.get_observation()
