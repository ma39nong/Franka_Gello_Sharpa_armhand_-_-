"""Non-blocking process helpers for RealSense camera tools."""

from __future__ import annotations

import signal
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(slots=True)
class CameraToolProcess:
    """Handle for a camera tool running as a sidecar process."""

    process: subprocess.Popen[Any]
    shutdown_timeout_sec: float = 15.0

    @property
    def pid(self) -> int | None:
        """Return the operating-system process id."""

        return self.process.pid

    def is_alive(self) -> bool:
        """Return whether the tool process is still running."""

        return self.process.poll() is None

    def stop(self, timeout_sec: float | None = None) -> None:
        """Request a clean shutdown, then terminate if the tool does not exit."""

        if self.process.poll() is not None:
            return
        timeout = self.shutdown_timeout_sec if timeout_sec is None else timeout_sec
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            self.process.wait(timeout=timeout)


def start_realsense_viewer(
    *,
    serial_numbers: tuple[str, ...] | list[str] | None = None,
    width: int = 640,
    height: int = 480,
    fps: int = 30,
    align_depth_to_color: bool = True,
    depth_colormap_alpha: float = 0.03,
    window_prefix: str = "BarMate RealSense",
) -> CameraToolProcess:
    """Start the RealSense realtime viewer in a non-blocking sidecar process."""

    return start_camera_viewer(
        backend="realsense",
        serial_numbers=serial_numbers,
        width=width,
        height=height,
        fps=fps,
        align_depth_to_color=align_depth_to_color,
        depth_colormap_alpha=depth_colormap_alpha,
        window_prefix=window_prefix,
    )


def start_camera_viewer(
    *,
    backend: str = "realsense",
    serial_numbers: tuple[str, ...] | list[str] | None = None,
    width: int = 640,
    height: int = 480,
    fps: int = 30,
    align_depth_to_color: bool = True,
    depth_colormap_alpha: float = 0.03,
    window_prefix: str = "BarMate Camera",
) -> CameraToolProcess:
    """Start the realtime viewer in a non-blocking sidecar process."""

    command = [
        sys.executable,
        "-m",
        "barmate.hardware.cameras.tools.viewer",
        "--backend",
        backend,
        "--width",
        str(width),
        "--height",
        str(height),
        "--fps",
        str(fps),
        "--depth-alpha",
        str(depth_colormap_alpha),
        "--window-prefix",
        window_prefix,
    ]
    _append_serial_args(command, serial_numbers)
    if not align_depth_to_color:
        command.append("--no-align-depth")
    return CameraToolProcess(subprocess.Popen(command))


def start_realsense_recording(
    *,
    output_dir: Path | str,
    serial_numbers: tuple[str, ...] | list[str] | None = None,
    width: int = 640,
    height: int = 480,
    fps: int = 30,
    warmup_frames: int = 30,
    align_depth_to_color: bool = True,
    preview: bool = False,
    depth_colormap_alpha: float = 0.03,
    record_pointcloud: bool = False,
    frame_format: str = "pnm",
    writer_backend: str = "csv",
) -> CameraToolProcess:
    """Start the RealSense recorder in a non-blocking sidecar process."""

    return start_camera_recording(
        backend="realsense",
        output_dir=output_dir,
        serial_numbers=serial_numbers,
        width=width,
        height=height,
        fps=fps,
        warmup_frames=warmup_frames,
        align_depth_to_color=align_depth_to_color,
        preview=preview,
        depth_colormap_alpha=depth_colormap_alpha,
        record_pointcloud=record_pointcloud,
        frame_format=frame_format,
        writer_backend=writer_backend,
    )


def start_camera_recording(
    *,
    backend: str = "realsense",
    output_dir: Path | str,
    serial_numbers: tuple[str, ...] | list[str] | None = None,
    width: int = 640,
    height: int = 480,
    fps: int = 30,
    warmup_frames: int = 30,
    align_depth_to_color: bool = True,
    preview: bool = False,
    depth_colormap_alpha: float = 0.03,
    record_pointcloud: bool = False,
    frame_format: str = "pnm",
    writer_backend: str = "csv",
) -> CameraToolProcess:
    """Start the recorder in a non-blocking sidecar process."""

    command = [
        sys.executable,
        "-m",
        "barmate.hardware.cameras.tools.recorder",
        "--backend",
        backend,
        "--output-dir",
        str(output_dir),
        "--width",
        str(width),
        "--height",
        str(height),
        "--fps",
        str(fps),
        "--warmup-frames",
        str(warmup_frames),
        "--depth-alpha",
        str(depth_colormap_alpha),
        "--frame-format",
        frame_format,
        "--writer-backend",
        writer_backend,
    ]
    _append_serial_args(command, serial_numbers)
    if not align_depth_to_color:
        command.append("--no-align-depth")
    if preview:
        command.append("--preview")
    if record_pointcloud:
        command.append("--pointcloud")
    return CameraToolProcess(subprocess.Popen(command), shutdown_timeout_sec=600.0)


def _append_serial_args(
    command: list[str], serial_numbers: tuple[str, ...] | list[str] | None
) -> None:
    if serial_numbers is None:
        return
    for serial in serial_numbers:
        command.extend(("--serial", serial))
