"""Lifecycle wrapper for Linker Hand replay sessions."""

from __future__ import annotations

import csv
import logging
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from barmate.hardware.linker_hand.control.controller import ManualHandController
from barmate.hardware.linker_hand.control.playback import HandPlaybackService
from barmate.hardware.linker_hand.control.types import HandPlaybackFrame
from barmate.hardware.linker_hand.control.app import run_linker_hand_control_gui
from barmate.hardware.linker_hand.linker_hand_sdk import (
    LinkerHandPairConfig,
    LinkerHandPairFactory,
)


logger = logging.getLogger(__name__)


@dataclass(slots=True, frozen=True)
class HandPlaybackConfig:
    trajectory_csv: Path
    left_hand_joint: str = "G20"
    right_hand_joint: str = "L10"
    left_can: str = "can1"
    right_can: str = "can0"
    no_hand: bool = False
    mock_hand: bool = False
    dry_run: bool = False
    hand_gui: bool = False


@dataclass(slots=True)
class HandPlaybackSession:
    """Own optional hand controller, playback service, and GUI thread."""

    controller: ManualHandController | None
    service: HandPlaybackService | None
    frames: list[HandPlaybackFrame]
    hand_gui: bool = False
    gui_thread: threading.Thread | None = None

    @classmethod
    def build(cls, config: HandPlaybackConfig) -> "HandPlaybackSession":
        if config.no_hand:
            return cls(controller=None, service=None, frames=[], hand_gui=config.hand_gui)
        controller, service, frames = _build_hand_controller(config)
        return cls(
            controller=controller,
            service=service,
            frames=frames,
            hand_gui=config.hand_gui,
        )

    def start_gui(self) -> None:
        if self.hand_gui and self.controller is not None:
            self.gui_thread = _start_hand_gui(self.controller)

    def start_playback(
        self,
        *,
        start_wall_time: float | None = None,
        playback_speed: float = 1.0,
    ) -> None:
        if self.service is None or not self.frames:
            return
        frames = _scale_playback_frames(self.frames, playback_speed=playback_speed)
        if start_wall_time is None:
            logger.debug("hand: starting playback with %s frame(s)", len(frames))
        else:
            logger.debug(
                "hand: starting playback with %s frame(s) at wall time %.6f",
                len(frames),
                start_wall_time,
            )
        self.service.start(frames, start_wall_time=start_wall_time)

    def wait_until_finished(self) -> None:
        if self.service is None:
            return
        logger.debug("hand: waiting for playback to finish")
        while self.service.is_running:
            time.sleep(0.01)
        if self.service.error is not None:
            raise self.service.error

    def stop(self) -> None:
        if self.service is not None:
            self.service.stop()


def _build_hand_controller(
    config: HandPlaybackConfig,
) -> tuple[ManualHandController, HandPlaybackService, list[HandPlaybackFrame]]:
    left, right = LinkerHandPairFactory().create_pair(
        LinkerHandPairConfig(
            left_hand_joint=config.left_hand_joint,
            right_hand_joint=config.right_hand_joint,
            left_can=config.left_can,
            right_can=config.right_can,
            mock=bool(config.mock_hand or config.dry_run),
        )
    )
    controller = ManualHandController(left, right)
    controller.start()
    service = HandPlaybackService(controller)
    frames = load_embedded_hand_frames(config.trajectory_csv)
    if not frames:
        frames = service.load_csv(config.trajectory_csv.parent)
    return controller, service, frames


def _scale_playback_frames(
    frames: list[HandPlaybackFrame], *, playback_speed: float
) -> list[HandPlaybackFrame]:
    speed = float(playback_speed)
    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError("hand playback speed must be positive")
    if speed == 1.0:
        return frames
    return [
        HandPlaybackFrame(
            timestamp=float(frame.timestamp) / speed,
            left=frame.left,
            right=frame.right,
        )
        for frame in frames
    ]


def load_embedded_hand_frames(path: Path) -> list[HandPlaybackFrame]:
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            return []
        fieldnames = list(reader.fieldnames)
        left_indices = _hand_indices(fieldnames, "left_hand")
        right_indices = _hand_indices(fieldnames, "right_hand")
        if not left_indices or not right_indices:
            return []

        frames: list[HandPlaybackFrame] = []
        t0: float | None = None
        for row in reader:
            timestamp = float(row["timestamp"])
            if t0 is None:
                t0 = timestamp
            frames.append(
                HandPlaybackFrame(
                    timestamp=max(timestamp - t0, 0.0),
                    left=tuple(
                        int(float(row.get(f"left_hand_{index}", "0") or "0"))
                        for index in left_indices
                    ),
                    right=tuple(
                        int(float(row.get(f"right_hand_{index}", "0") or "0"))
                        for index in right_indices
                    ),
                )
            )
    return frames


def _hand_indices(fieldnames: list[str], prefix: str) -> list[int]:
    prefix_text = f"{prefix}_"
    indices: list[int] = []
    for field in fieldnames:
        if field.startswith(prefix_text) and field[len(prefix_text) :].isdigit():
            indices.append(int(field[len(prefix_text) :]))
    return sorted(indices)


def _start_hand_gui(controller: ManualHandController) -> threading.Thread:
    thread = threading.Thread(
        target=run_linker_hand_control_gui,
        kwargs={"controller": controller, "show": False},
        name="linker-hand-gui",
        daemon=True,
    )
    thread.start()
    logger.debug("hand: GUI available at http://127.0.0.1:8080")
    return thread


__all__ = [
    "HandPlaybackConfig",
    "HandPlaybackSession",
    "load_embedded_hand_frames",
]
