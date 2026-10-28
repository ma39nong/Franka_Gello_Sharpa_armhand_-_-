"""Small robot applications built on hardware-neutral interfaces."""

from __future__ import annotations

from importlib import import_module

__all__ = [
    "HandPlaybackFrame",
    "HandPlaybackService",
    "HandSlot",
    "LinkerHandControlWindow",
    "ManualHandController",
    "load_hand_playback_records",
    "run_linker_hand_control_gui",
]


def __getattr__(name: str) -> object:
    if name not in __all__:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module("barmate.hardware.linker_hand.control")
    return getattr(module, name)
