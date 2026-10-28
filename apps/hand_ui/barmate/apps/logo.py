"""Display robot logo/startup animation using robot hardware interfaces."""

from __future__ import annotations

from barmate.core.interfaces import RobotInterface

BARMATE_LOGO = "BarMate"


def display_logo(robot: RobotInterface | None = None) -> str:
    """Return the startup logo text; robot hooks will be added during implementation."""

    _ = robot
    return BARMATE_LOGO
