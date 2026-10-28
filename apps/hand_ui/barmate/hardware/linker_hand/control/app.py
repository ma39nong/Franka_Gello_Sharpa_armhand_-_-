"""NiceGUI Linker Hand Studio app entry point."""

from __future__ import annotations

import argparse

from barmate.hardware.linker_hand import LinkerHandMockRobot, LinkerHandRobot
from barmate.hardware.linker_hand.control.controller import ManualHandController
from barmate.hardware.linker_hand.control.playback import (
    HandPlaybackService,
    load_hand_playback_records,
)
from barmate.hardware.linker_hand.control.ros_backend import RosLinkerHandControlHand
from barmate.hardware.linker_hand.control.types import HandPlaybackFrame, HandSlot
from barmate.hardware.linker_hand.control.window import (
    LinkerHandControlWindow,
    run_linker_hand_control_gui,
)

__all__ = [
    "HandPlaybackFrame",
    "HandPlaybackService",
    "HandSlot",
    "LinkerHandControlWindow",
    "ManualHandController",
    "build_manual_hand_controller",
    "load_hand_playback_records",
    "main",
    "run_linker_hand_control_gui",
]


def build_manual_hand_controller(
    *,
    backend: str = "ros",
    mock: bool = False,
    left: bool = True,
    right: bool = True,
    left_model: str = "L20",
    right_model: str = "L20",
    left_can: str = "can0",
    right_can: str = "can1",
    connect_on_start: bool = True,
) -> ManualHandController:
    """Create the hand controller used by the browser UI."""

    if mock:
        left_hand = (
            LinkerHandMockRobot(hand_type="left", hand_joint=left_model)
            if left
            else None
        )
        right_hand = (
            LinkerHandMockRobot(hand_type="right", hand_joint=right_model)
            if right
            else None
        )
    elif backend == "ros":
        left_hand = (
            RosLinkerHandControlHand(side="left", model=left_model, can=left_can)
            if left
            else None
        )
        right_hand = (
            RosLinkerHandControlHand(side="right", model=right_model, can=right_can)
            if right
            else None
        )
    elif backend == "sdk":
        left_hand = (
            LinkerHandRobot(hand_type="left", hand_joint=left_model, can=left_can)
            if left
            else None
        )
        right_hand = (
            LinkerHandRobot(hand_type="right", hand_joint=right_model, can=right_can)
            if right
            else None
        )
    else:
        raise ValueError(f"Unsupported Linker Hand control backend: {backend}")
    return ManualHandController(
        left_hand=left_hand,
        right_hand=right_hand,
        connect_on_start=connect_on_start,
    )


def main(argv: list[str] | None = None) -> int:
    """Start the NiceGUI hand control studio."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--backend",
        choices=("sdk", "ros"),
        default="ros",
        help="Control backend: sdk connects through Python SDK/CAN; ros publishes ROS 2 topics.",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Use in-memory mock hands without connecting to hardware.",
    )
    parser.add_argument("--no-left", action="store_true", help="Disable left hand.")
    parser.add_argument("--no-right", action="store_true", help="Disable right hand.")
    parser.add_argument(
        "--left-model", default="O30I", help="Left model, e.g. G20/O30I."
    )
    parser.add_argument(
        "--right-model", default="O30I", help="Right model, e.g. G20/O30I."
    )
    parser.add_argument("--left-can", default="can1", help="Left hand CAN device.")
    parser.add_argument("--right-can", default="can0", help="Right hand CAN device.")
    parser.add_argument(
        "--no-connect",
        action="store_true",
        help="Start the UI without calling connect/read.",
    )
    parser.add_argument("--host", default="0.0.0.0", help="NiceGUI listen host.")
    parser.add_argument("--port", type=int, default=8080, help="NiceGUI listen port.")
    args = parser.parse_args(argv)
    controller = build_manual_hand_controller(
        backend=args.backend,
        mock=args.mock,
        left=not args.no_left,
        right=not args.no_right,
        left_model=args.left_model,
        right_model=args.right_model,
        left_can=args.left_can,
        right_can=args.right_can,
        connect_on_start=not args.no_connect,
    )
    run_linker_hand_control_gui(
        controller=controller, host=args.host, port=args.port, show=False
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
