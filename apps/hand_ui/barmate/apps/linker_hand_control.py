"""Compatibility wrapper for the Linker Hand control GUI entry point."""

from barmate.hardware.linker_hand.control import (
    HandPlaybackFrame,
    HandPlaybackService,
    HandSlot,
    LinkerHandControlWindow,
    ManualHandController,
    load_hand_playback_records,
    main,
    run_linker_hand_control_gui,
)

__all__ = [
    "HandPlaybackFrame",
    "HandPlaybackService",
    "HandSlot",
    "LinkerHandControlWindow",
    "ManualHandController",
    "load_hand_playback_records",
    "main",
    "run_linker_hand_control_gui",
]


if __name__ == "__main__":
    raise SystemExit(main())
