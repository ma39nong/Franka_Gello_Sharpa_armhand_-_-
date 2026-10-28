"""Context-managed camera recording sessions for replay workflows."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path

from barmate.hardware.cameras.tools.process import (
    CameraToolProcess,
    start_camera_recording,
)


logger = logging.getLogger(__name__)


@dataclass(slots=True, frozen=True)
class CameraRecordingConfig:
    output_dir: Path | None = None
    backend: str = "orbbec"
    serials: list[str] | None = None
    width: int = 1280
    height: int = 800
    fps: int = 10
    warmup_frames: int = 30
    align_depth_to_color: bool = True
    preview: bool = False
    depth_colormap_alpha: float = 0.03
    pointcloud: bool = False
    frame_format: str = "pnm"
    writer_backend: str = "csv"

    def validate(self) -> None:
        if self.output_dir is None:
            return
        if self.width < 1:
            raise ValueError("--camera-width must be at least 1")
        if self.height < 1:
            raise ValueError("--camera-height must be at least 1")
        if self.fps < 1:
            raise ValueError("--camera-fps must be at least 1")
        if self.warmup_frames < 0:
            raise ValueError("--camera-warmup-frames must be non-negative")
        if self.frame_format not in {"pnm", "png"}:
            raise ValueError("--camera-frame-format must be one of: pnm, png")
        if self.writer_backend not in {"csv", "hdf5"}:
            raise ValueError("--camera-writer-backend must be one of: csv, hdf5")
        if self.pointcloud and self.backend != "orbbec":
            raise ValueError("--camera-pointcloud requires --camera-backend orbbec")
        if (
            self.pointcloud
            and self.writer_backend == "csv"
            and self.frame_format != "pnm"
        ):
            raise ValueError("--camera-pointcloud requires --camera-frame-format pnm")
        if self.pointcloud and not self.align_depth_to_color:
            raise ValueError("--camera-pointcloud requires depth aligned to color")


@dataclass(slots=True)
class CameraRecordingSession:
    """Optional camera recording lifecycle."""

    config: CameraRecordingConfig
    recording: CameraToolProcess | None = None

    def __enter__(self) -> "CameraRecordingSession":
        self.recording = self.start()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.stop()

    def start(self) -> CameraToolProcess | None:
        if self.config.output_dir is None:
            return None

        output_dir = self.config.output_dir.expanduser().resolve()
        logger.debug("camera: recording %s stream(s) to %s", self.config.backend, output_dir)
        return start_camera_recording(
            backend=self.config.backend,
            output_dir=output_dir,
            serial_numbers=self.config.serials,
            width=self.config.width,
            height=self.config.height,
            fps=self.config.fps,
            warmup_frames=self.config.warmup_frames,
            align_depth_to_color=self.config.align_depth_to_color,
            preview=self.config.preview,
            depth_colormap_alpha=self.config.depth_colormap_alpha,
            record_pointcloud=self.config.pointcloud,
            frame_format=self.config.frame_format,
            writer_backend=self.config.writer_backend,
        )

    def stop(self) -> None:
        if self.recording is None:
            return
        logger.debug("camera: stopping recording")
        self.recording.stop()
        self.recording = None


__all__ = ["CameraRecordingConfig", "CameraRecordingSession"]
