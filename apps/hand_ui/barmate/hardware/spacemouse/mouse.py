"""3Dconnexion SpaceMouse driver implementing the core SpaceMouseInterface."""

from __future__ import annotations

from typing import Any

from barmate.core.teleoperator import SpaceMouseInterface
from barmate.core.types import FeatureSpec, TeleoperatorAction, TeleoperatorFeedback

# Implements barmate.core.teleoperator.SpaceMouseInterface using pyspacemouse.


class SpaceMouse(SpaceMouseInterface):
    """Read six-degree-of-freedom input from a 3Dconnexion SpaceMouse."""

    def __init__(self, device: str | None = None) -> None:
        self.device = device
        self._pyspacemouse: Any | None = None
        self._device: Any | None = None

    def configure(self) -> None:
        """SpaceMouse currently has no runtime configuration."""

    def connect(self) -> None:
        if self._device is not None:
            return
        import pyspacemouse

        self._pyspacemouse = pyspacemouse
        if self.device is None:
            self._device = pyspacemouse.open()
        else:
            self._device = pyspacemouse.open(device=self.device)

    def disconnect(self) -> None:
        if self._device is not None:
            close = getattr(self._device, "close", None)
            if callable(close):
                close()
        self._device = None
        self._pyspacemouse = None

    @property
    def is_connected(self) -> bool:
        return self._device is not None

    @property
    def action_features(self) -> FeatureSpec:
        return {
            "translation": (3,),
            "rotation": (3,),
            "buttons": tuple,
        }

    @property
    def feedback_features(self) -> FeatureSpec:
        return {}

    def get_action(self) -> TeleoperatorAction:
        if self._device is None:
            raise RuntimeError("SpaceMouse is not connected")
        state = self._device.read()
        buttons = getattr(state, "buttons", ())
        return {
            "translation": (
                float(getattr(state, "x", 0.0)),
                float(getattr(state, "y", 0.0)),
                float(getattr(state, "z", 0.0)),
            ),
            "rotation": (
                float(getattr(state, "roll", 0.0)),
                float(getattr(state, "pitch", 0.0)),
                float(getattr(state, "yaw", 0.0)),
            ),
            "buttons": tuple(buttons) if buttons is not None else (),
        }

    def send_feedback(self, feedback: TeleoperatorFeedback) -> None:
        _ = feedback
