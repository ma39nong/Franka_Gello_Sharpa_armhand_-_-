"""Intel RealSense camera driver implementing the core CameraInterface."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from importlib import import_module
from typing import Any

import numpy as np
from numpy.typing import NDArray

from barmate.core.camera import CameraInterface
from barmate.core.types import CameraFrame, CameraMetadata


@dataclass(frozen=True, slots=True)
class RealSenseConfig:
    """Stream configuration for one RealSense camera."""

    width: int = 640
    height: int = 480
    fps: int = 30
    enable_depth: bool = True
    align_depth_to_color: bool = True


@dataclass(frozen=True, slots=True)
class RealSenseCapture:
    """Color/depth frame pair captured from one RealSense camera."""

    color: NDArray[np.uint8]
    depth: NDArray[np.uint16] | None = None
    timestamp: float | None = None
    timestamp_ms: float | None = None
    color_frame_number: int | None = None
    depth_frame_number: int | None = None
    metadata: CameraMetadata = field(default_factory=dict)
    _frames: Any = field(default=None, repr=False, compare=False)
    _color_frame: Any = field(default=None, repr=False, compare=False)
    _depth_frame: Any = field(default=None, repr=False, compare=False)


class RealSenseCamera(CameraInterface):
    """Read color and aligned depth frames from one Intel RealSense camera."""

    def __init__(
        self,
        serial: str | None = None,
        *,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        enable_depth: bool = True,
        align_depth_to_color: bool = True,
        rs_module: Any | None = None,
    ) -> None:
        if width < 1:
            raise ValueError("width must be at least 1")
        if height < 1:
            raise ValueError("height must be at least 1")
        if fps < 1:
            raise ValueError("fps must be at least 1")

        self.serial = serial
        self.config = RealSenseConfig(
            width=width,
            height=height,
            fps=fps,
            enable_depth=enable_depth,
            align_depth_to_color=align_depth_to_color,
        )
        self._rs = rs_module
        self._pipeline: Any | None = None
        self._profile: Any | None = None
        self._aligner: Any | None = None
        self._last_capture: RealSenseCapture | None = None
        self._last_capture_monotonic = 0.0
        self.depth_scale_meters: float | None = None
        self.color_intrinsic_matrix: list[list[float]] | None = None

    def connect(self) -> None:
        """Open the RealSense pipeline."""

        if self._pipeline is not None:
            return

        rs = self._load_realsense_module()
        pipeline = rs.pipeline()
        config = rs.config()
        if self.serial is not None:
            config.enable_device(self.serial)
        if self.config.enable_depth:
            config.enable_stream(
                rs.stream.depth,
                self.config.width,
                self.config.height,
                rs.format.z16,
                self.config.fps,
            )
        config.enable_stream(
            rs.stream.color,
            self.config.width,
            self.config.height,
            rs.format.bgr8,
            self.config.fps,
        )

        try:
            profile = pipeline.start(config)
            aligner = (
                rs.align(rs.stream.color)
                if self.config.enable_depth and self.config.align_depth_to_color
                else None
            )
            self._read_stream_metadata(rs, profile)
        except Exception:
            try:
                pipeline.stop()
            except RuntimeError:
                pass
            self._pipeline = None
            self._profile = None
            self._aligner = None
            self.depth_scale_meters = None
            self.color_intrinsic_matrix = None
            raise

        self._pipeline = pipeline
        self._profile = profile
        self._aligner = aligner

    def disconnect(self) -> None:
        """Close the RealSense pipeline."""

        try:
            if self._pipeline is not None:
                self._pipeline.stop()
        finally:
            self._pipeline = None
            self._profile = None
            self._aligner = None
            self._last_capture = None
            self._last_capture_monotonic = 0.0

    @property
    def is_connected(self) -> bool:
        """Whether the RealSense pipeline is active."""

        return self._pipeline is not None

    def read_frame(self) -> CameraFrame:
        """Read one blocking color frame with depth metadata when enabled."""

        capture = self.read_capture()
        return CameraFrame(
            image=capture.color,
            timestamp=capture.timestamp,
            metadata={
                **capture.metadata,
                "depth_image": capture.depth,
                "depth_scale_meters": self.depth_scale_meters,
                "intrinsic_matrix": self.color_intrinsic_matrix,
                "color_intrinsic_matrix": self.color_intrinsic_matrix,
            },
        )

    def read_capture(self) -> RealSenseCapture:
        """Read the next color/depth capture, waiting until the camera produces one."""

        if self._pipeline is None:
            raise RuntimeError("RealSense camera is not connected")
        frames = self._pipeline.wait_for_frames()
        return self._capture_from_frames(frames)

    def async_read_frame(self, timeout_ms: float = 200.0) -> NDArray[np.uint8]:
        """Return the next color frame, waiting up to ``timeout_ms``."""

        return self.async_read_capture(timeout_ms=timeout_ms).color

    def async_read_capture(self, timeout_ms: float = 200.0) -> RealSenseCapture:
        """Return the next color/depth capture, waiting up to ``timeout_ms``."""

        if self._pipeline is None:
            raise RuntimeError("RealSense camera is not connected")
        success, frames = self._pipeline.try_wait_for_frames(int(timeout_ms))
        if not success:
            raise TimeoutError(f"No RealSense frame arrived within {timeout_ms:.1f} ms")
        return self._capture_from_frames(frames)

    def read_latest(self, max_age_ms: int = 500) -> NDArray[np.uint8]:
        """Return the most recent color frame if fresh, otherwise read a new one."""

        if self._last_capture is not None:
            age_ms = (time.monotonic() - self._last_capture_monotonic) * 1000.0
            if age_ms <= max_age_ms:
                return self._last_capture.color
        return self.async_read_frame(timeout_ms=float(max_age_ms))

    def _load_realsense_module(self) -> Any:
        if self._rs is None:
            self._rs = import_module("pyrealsense2")
        return self._rs

    def _read_stream_metadata(self, rs: Any, profile: Any) -> None:
        if self.config.enable_depth:
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale_meters = float(depth_sensor.get_depth_scale())
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.color_intrinsic_matrix = _intrinsics_to_matrix(
            color_stream.get_intrinsics()
        )

    def _capture_from_frames(self, frames: Any) -> RealSenseCapture:
        if self._aligner is not None:
            frames = self._aligner.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame() if self.config.enable_depth else None
        if not color_frame:
            raise RuntimeError("RealSense frame set did not contain a color frame")
        if self.config.enable_depth and not depth_frame:
            raise RuntimeError("RealSense frame set did not contain a depth frame")

        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data()) if depth_frame else None
        timestamp_ms = float(frames.get_timestamp())
        capture = RealSenseCapture(
            color=color,
            depth=depth,
            timestamp=timestamp_ms / 1000.0,
            timestamp_ms=timestamp_ms,
            color_frame_number=int(color_frame.get_frame_number()),
            depth_frame_number=int(depth_frame.get_frame_number())
            if depth_frame
            else None,
            metadata={
                "serial": self.serial,
                "width": self.config.width,
                "height": self.config.height,
                "fps": self.config.fps,
                "color_format": "bgr8",
                "depth_format": "z16" if self.config.enable_depth else None,
            },
            _frames=frames,
            _color_frame=color_frame,
            _depth_frame=depth_frame,
        )
        self._last_capture = capture
        self._last_capture_monotonic = time.monotonic()
        return capture


def list_realsense_serials(rs_module: Any | None = None) -> list[str]:
    """Return connected RealSense device serial numbers."""

    rs = import_module("pyrealsense2") if rs_module is None else rs_module
    context = rs.context()
    return [
        device.get_info(rs.camera_info.serial_number)
        for device in context.query_devices()
    ]


def _intrinsics_to_matrix(intrinsics: Any) -> list[list[float]]:
    return [
        [float(intrinsics.fx), 0.0, float(intrinsics.ppx)],
        [0.0, float(intrinsics.fy), float(intrinsics.ppy)],
        [0.0, 0.0, 1.0],
    ]


__all__ = [
    "RealSenseCamera",
    "RealSenseCapture",
    "RealSenseConfig",
    "list_realsense_serials",
]
