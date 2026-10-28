"""Record camera color/depth frames to disk."""

from __future__ import annotations

import argparse
import csv
import json
import queue
import struct
import threading
import time
import zlib
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray

from barmate.hardware.cameras.tools.backends import backend_choices, camera_backend
from barmate.hardware.cameras.tools.viewer import color_depth_preview


FRAME_NAME_WIDTH = 6
FRAME_FORMATS = ("pnm", "png")
WRITER_BACKENDS = ("csv", "hdf5")
FRAME_EXTENSIONS = {
    "pnm": ("ppm", "pgm"),
    "png": ("png", "png"),
}
WRITER_QUEUE_SECONDS = 5
WRITER_QUEUE_MAX_JOBS = 100
WRITER_SHUTDOWN_TIMEOUT_SECONDS = 5.0
_STOP_WRITER = object()


@dataclass(frozen=True)
class _FrameWriteJob:
    frame_name: str
    color_frame: NDArray[np.uint8]
    depth_frame: NDArray[np.uint16]
    color_frame_number: int | None
    depth_frame_number: int | None
    timestamp_ms: float | None


@dataclass(frozen=True)
class _FlushWriteJob:
    done: threading.Event | None = None


class RecordingWriter(Protocol):
    """Unified camera recording writer strategy."""

    def open(self) -> None:
        """Open files/directories needed by the writer."""

    def add_frame(self, job: _FrameWriteJob) -> None:
        """Persist one captured RGB-D frame pair and its timestamp metadata."""

    def flush(self) -> None:
        """Flush buffered primary evidence to the backing store."""

    def close(self) -> None:
        """Close all writer resources."""

    def read_color_frame(self, frame_name: str) -> NDArray[np.uint8]:
        """Read a saved RGB color frame for deferred post-processing."""

    def read_depth_frame(self, frame_name: str) -> NDArray[np.uint16]:
        """Read a saved depth frame for deferred post-processing."""


class CsvWriter:
    """Write frames as image files and frame metadata as ``timestamps.csv``."""

    def __init__(self, *, camera_dir: Path, frame_format: str) -> None:
        self.camera_dir = camera_dir
        self.image_dir = camera_dir / "image"
        self.depth_dir = camera_dir / "depth"
        self.frame_format = frame_format
        self.color_extension, self.depth_extension = FRAME_EXTENSIONS[frame_format]
        self._timestamps_file: Any | None = None
        self._timestamps_writer: Any | None = None

    def open(self) -> None:
        self.image_dir.mkdir(parents=True)
        self.depth_dir.mkdir(parents=True)
        self._timestamps_file = (self.camera_dir / "timestamps.csv").open(
            "w", newline="", encoding="ascii"
        )
        self._timestamps_writer = csv.writer(self._timestamps_file)
        self._timestamps_writer.writerow(
            ["index", "color_frame_number", "depth_frame_number", "timestamp_ms"]
        )

    def add_frame(self, job: _FrameWriteJob) -> None:
        self._write_frame_pair(job.frame_name, job.color_frame, job.depth_frame)
        self._write_timestamp(job)

    def flush(self) -> None:
        if self._timestamps_file is not None:
            self._timestamps_file.flush()

    def close(self) -> None:
        if self._timestamps_file is not None:
            self._timestamps_file.close()
            self._timestamps_file = None
        self._timestamps_writer = None

    def read_color_frame(self, frame_name: str) -> NDArray[np.uint8]:
        if self.frame_format != "pnm":
            raise RuntimeError(
                "Deferred pointcloud recording currently requires pnm frames"
            )
        return read_ppm(self.image_dir / f"{frame_name}.{self.color_extension}")

    def read_depth_frame(self, frame_name: str) -> NDArray[np.uint16]:
        if self.frame_format != "pnm":
            raise RuntimeError(
                "Deferred pointcloud recording currently requires pnm frames"
            )
        return read_pgm(self.depth_dir / f"{frame_name}.{self.depth_extension}")

    def _write_timestamp(self, job: _FrameWriteJob) -> None:
        if self._timestamps_writer is None:
            raise RuntimeError("timestamps writer is not open")
        self._timestamps_writer.writerow(
            [
                job.frame_name,
                job.color_frame_number,
                job.depth_frame_number,
                f"{job.timestamp_ms:.3f}" if job.timestamp_ms is not None else "",
            ]
        )

    def _write_frame_pair(
        self,
        frame_name: str,
        color_frame: NDArray[np.uint8],
        depth_frame: NDArray[np.uint16],
    ) -> None:
        color_path = self.image_dir / f"{frame_name}.{self.color_extension}"
        depth_path = self.depth_dir / f"{frame_name}.{self.depth_extension}"
        rgb_frame = color_frame[:, :, ::-1]
        if self.frame_format == "pnm":
            write_ppm(color_path, rgb_frame)
            write_pgm(depth_path, depth_frame)
            return
        if self.frame_format == "png":
            write_rgb_png(color_path, rgb_frame)
            write_depth_png(depth_path, depth_frame)
            return
        raise RuntimeError(f"Unsupported frame format: {self.frame_format}")


class Hdf5Writer:
    """Write frame data and metadata into one resizable HDF5 file."""

    def __init__(self, *, camera_dir: Path, filename: str = "recording.hdf5") -> None:
        self.camera_dir = camera_dir
        self.path = camera_dir / filename
        self._h5_file: Any | None = None
        self._color_dataset: Any | None = None
        self._depth_dataset: Any | None = None
        self._color_numbers: Any | None = None
        self._depth_numbers: Any | None = None
        self._timestamps: Any | None = None
        self._indices: Any | None = None
        self._frame_count = 0

    def open(self) -> None:
        h5py = _require_h5py()
        self._h5_file = h5py.File(self.path, "w")
        self._h5_file.attrs["format_version"] = "barmate.camera_recorder.v1"
        self._h5_file.attrs["color_space"] = "rgb"
        self._h5_file.attrs["depth_dtype"] = "uint16"

    def add_frame(self, job: _FrameWriteJob) -> None:
        h5_file = self._require_open_file()
        rgb_frame = np.ascontiguousarray(job.color_frame[:, :, ::-1], dtype=np.uint8)
        depth_frame = np.ascontiguousarray(job.depth_frame, dtype=np.uint16)
        if self._color_dataset is None:
            self._create_datasets(h5_file, rgb_frame, depth_frame)

        index = self._frame_count
        self._resize_datasets(index + 1)
        self._color_dataset[index] = rgb_frame
        self._depth_dataset[index] = depth_frame
        self._indices[index] = int(job.frame_name)
        self._color_numbers[index] = _optional_int(job.color_frame_number)
        self._depth_numbers[index] = _optional_int(job.depth_frame_number)
        self._timestamps[index] = (
            float(job.timestamp_ms) if job.timestamp_ms is not None else np.nan
        )
        self._frame_count += 1

    def flush(self) -> None:
        if self._h5_file is not None:
            self._h5_file.flush()

    def close(self) -> None:
        if self._h5_file is not None:
            self._h5_file.close()
            self._h5_file = None
        self._color_dataset = None
        self._depth_dataset = None
        self._color_numbers = None
        self._depth_numbers = None
        self._timestamps = None
        self._indices = None

    def read_color_frame(self, frame_name: str) -> NDArray[np.uint8]:
        if self._color_dataset is None:
            raise RuntimeError("HDF5 color dataset is not open")
        return np.asarray(self._color_dataset[int(frame_name)], dtype=np.uint8)

    def read_depth_frame(self, frame_name: str) -> NDArray[np.uint16]:
        if self._depth_dataset is None:
            raise RuntimeError("HDF5 depth dataset is not open")
        return np.asarray(self._depth_dataset[int(frame_name)], dtype=np.uint16)

    def _require_open_file(self) -> Any:
        if self._h5_file is None:
            raise RuntimeError("HDF5 writer is not open")
        return self._h5_file

    def _create_datasets(
        self,
        h5_file: Any,
        color_frame: NDArray[np.uint8],
        depth_frame: NDArray[np.uint16],
    ) -> None:
        self._color_dataset = h5_file.create_dataset(
            "color",
            shape=(0, *color_frame.shape),
            maxshape=(None, *color_frame.shape),
            chunks=(1, *color_frame.shape),
            dtype="uint8",
        )
        self._depth_dataset = h5_file.create_dataset(
            "depth",
            shape=(0, *depth_frame.shape),
            maxshape=(None, *depth_frame.shape),
            chunks=(1, *depth_frame.shape),
            dtype="uint16",
        )
        self._indices = h5_file.create_dataset(
            "index", shape=(0,), maxshape=(None,), dtype="int64"
        )
        self._color_numbers = h5_file.create_dataset(
            "color_frame_number", shape=(0,), maxshape=(None,), dtype="int64"
        )
        self._depth_numbers = h5_file.create_dataset(
            "depth_frame_number", shape=(0,), maxshape=(None,), dtype="int64"
        )
        self._timestamps = h5_file.create_dataset(
            "timestamp_ms", shape=(0,), maxshape=(None,), dtype="float64"
        )

    def _resize_datasets(self, frame_count: int) -> None:
        for dataset in (
            self._color_dataset,
            self._depth_dataset,
            self._indices,
            self._color_numbers,
            self._depth_numbers,
            self._timestamps,
        ):
            if dataset is None:
                raise RuntimeError("HDF5 datasets are not initialized")
            dataset.resize((frame_count, *dataset.shape[1:]))


def create_recording_writer(
    *, writer_backend: str, camera_dir: Path, frame_format: str
) -> RecordingWriter:
    """Create a concrete recording writer strategy."""

    if writer_backend == "csv":
        return CsvWriter(camera_dir=camera_dir, frame_format=frame_format)
    if writer_backend == "hdf5":
        return Hdf5Writer(camera_dir=camera_dir)
    raise ValueError(f"Unsupported writer backend: {writer_backend}")


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the command-line parser for the recorder tool."""

    parser = argparse.ArgumentParser(description="Record camera color/depth frames.")
    parser.add_argument(
        "--backend",
        choices=backend_choices(),
        default="realsense",
        help="Camera backend to use for recording.",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        type=Path,
        help="Empty directory where camera folders are created.",
    )
    parser.add_argument(
        "--serial",
        action="append",
        dest="serials",
        help="Camera serial to record; repeat for multiple cameras.",
    )
    parser.add_argument(
        "--width", type=int, default=640, help="Stream width in pixels."
    )
    parser.add_argument(
        "--height", type=int, default=480, help="Stream height in pixels."
    )
    parser.add_argument("--fps", type=int, default=30, help="Stream frame rate.")
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=30,
        help="Initial frames to discard per camera.",
    )
    parser.add_argument(
        "--duration-seconds", type=float, help="Optional duration after warmup."
    )
    parser.add_argument(
        "--max-frames", type=int, help="Optional saved frame limit per camera."
    )
    parser.add_argument(
        "--flush-every-frames",
        type=int,
        help="Timestamp CSV flush interval; defaults to fps.",
    )
    parser.add_argument(
        "--no-align-depth",
        action="store_true",
        help="Do not align depth to the color stream.",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show live OpenCV previews while recording.",
    )
    parser.add_argument(
        "--pointcloud",
        action="store_true",
        help="Save Orbbec point cloud PLY files alongside color/depth frames.",
    )
    parser.add_argument(
        "--frame-format",
        choices=FRAME_FORMATS,
        default="pnm",
        help="Frame file format: pnm writes raw .ppm/.pgm; png writes compressed .png files.",
    )
    parser.add_argument(
        "--writer-backend",
        choices=WRITER_BACKENDS,
        default="csv",
        help=(
            "Recording writer backend: csv writes image/depth files plus "
            "timestamps.csv; hdf5 writes recording.hdf5."
        ),
    )
    parser.add_argument(
        "--depth-alpha",
        type=float,
        default=0.03,
        help="Depth scale passed to OpenCV color mapping.",
    )
    return parser


def run_recorder(args: argparse.Namespace) -> int:
    """Run the recorder until interrupted, duration, or frame limit."""

    _validate_args(args)
    _ensure_clean_output_dir(args.output_dir)

    cv2: Any | None = None
    if args.preview:
        import cv2 as cv2_module

        cv2 = cv2_module

    backend = camera_backend(getattr(args, "backend", None))
    serials = tuple(args.serials) if args.serials else tuple(backend.list_serials())
    if not serials:
        raise RuntimeError(f"No {backend.display_name} devices found")
    _camera_dir_names(serials)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    recorders = [
        _CameraRecorder(serial=serial, root_dir=args.output_dir, args=args)
        for serial in serials
    ]
    opened_recorders: list[_CameraRecorder] = []

    start_time = time.monotonic()
    flush_every_frames = max(args.flush_every_frames or args.fps, 1)
    poll_period_s = 1.0 / max(float(args.fps), 1e-6)
    next_poll_at = start_time
    try:
        for recorder in recorders:
            recorder.open()
            opened_recorders.append(recorder)

        while True:
            if (
                args.duration_seconds is not None
                and time.monotonic() - start_time >= args.duration_seconds
            ):
                break
            now = time.monotonic()
            if now < next_poll_at:
                time.sleep(min(next_poll_at - now, 0.001))
                continue

            active_recorders = [
                recorder
                for recorder in recorders
                if not (
                    args.max_frames is not None
                    and recorder.saved_frames >= args.max_frames
                )
            ]
            if not active_recorders:
                break

            capture_timeout_ms = max(
                1.0, poll_period_s * 1000.0 / len(active_recorders)
            )
            rendered = False
            for recorder in active_recorders:
                try:
                    capture = recorder.camera.async_read_capture(
                        timeout_ms=capture_timeout_ms
                    )
                except TimeoutError:
                    continue
                if recorder.record_capture(capture):
                    if recorder.saved_frames % flush_every_frames == 0:
                        recorder.flush_async()
                    if cv2 is not None and capture.depth is not None:
                        cv2.imshow(
                            f"BarMate {backend.display_name} Recording {recorder.serial} color | depth",
                            color_depth_preview(
                                cv2, capture.color, capture.depth, args.depth_alpha
                            ),
                        )
                        rendered = True
            if cv2 is not None:
                key = int(cv2.waitKey(1 if rendered else 10)) & 0xFF
                if key in {27, ord("q")}:
                    break
            next_poll_at += poll_period_s
            if time.monotonic() - next_poll_at > poll_period_s:
                next_poll_at = time.monotonic() + poll_period_s
    except KeyboardInterrupt:
        print(f"[{backend.name}-recorder] Stopping capture...", flush=True)
    finally:
        close_errors: list[BaseException] = []
        for recorder in reversed(opened_recorders):
            try:
                recorder.close()
            except Exception as exc:
                close_errors.append(exc)
                print(
                    f"[{backend.name}-recorder] Failed to close {recorder.serial}: {exc}",
                    flush=True,
                )
        if cv2 is not None:
            cv2.destroyAllWindows()
        if close_errors:
            raise RuntimeError(
                f"Failed to close one or more {backend.display_name} recorders"
            ) from close_errors[0]
    return 0


class _CameraRecorder:
    def __init__(
        self, *, serial: str, root_dir: Path, args: argparse.Namespace
    ) -> None:
        self.serial = serial
        self.backend = camera_backend(getattr(args, "backend", None))
        self.camera_dir = root_dir / _safe_recording_dir_name(serial)
        self.pointcloud_dir = self.camera_dir / "pointcloud"
        self.camera = self.backend.camera_class(
            serial=serial,
            width=args.width,
            height=args.height,
            fps=args.fps,
            align_depth_to_color=not args.no_align_depth,
        )
        self.warmup_remaining = args.warmup_frames
        self.saved_frames = 0
        self.record_pointcloud = bool(getattr(args, "pointcloud", False))
        self.frame_format = str(getattr(args, "frame_format", "pnm"))
        self.writer_backend = str(getattr(args, "writer_backend", "csv"))
        self.color_extension, self.depth_extension = FRAME_EXTENSIONS[self.frame_format]
        self.writer = create_recording_writer(
            writer_backend=self.writer_backend,
            camera_dir=self.camera_dir,
            frame_format=self.frame_format,
        )
        self._write_queue_maxsize = min(
            max(args.fps * WRITER_QUEUE_SECONDS, 1), WRITER_QUEUE_MAX_JOBS
        )
        self._write_queue: queue.Queue[object] | None = None
        self._write_thread: threading.Thread | None = None
        self._write_error: BaseException | None = None

    def open(self) -> None:
        if self.camera_dir.exists():
            raise FileExistsError(
                f"Camera recording directory already exists: {self.camera_dir}"
            )
        try:
            self.camera.connect()
            self.camera_dir.mkdir(parents=True)
            if self.record_pointcloud:
                _validate_pointcloud_backend(self.backend.name)
                self.pointcloud_dir.mkdir(parents=True)
            self._write_metadata()
            self.writer.open()
            self._write_queue = queue.Queue(maxsize=self._write_queue_maxsize)
            self._write_thread = threading.Thread(
                target=self._write_worker,
                name=f"{self.backend.name}-recorder-writer-{self.serial}",
                daemon=True,
            )
            self._write_thread.start()
        except Exception:
            try:
                self.writer.close()
            finally:
                self.camera.disconnect()
            raise
        print(
            f"[{self.backend.name}-recorder] Recording {self.serial} to {self.camera_dir}",
            flush=True,
        )

    def record_capture(self, capture: Any) -> bool:
        if self.warmup_remaining > 0:
            self.warmup_remaining -= 1
            return False
        if capture.depth is None:
            raise RuntimeError(
                f"{self.backend.display_name} recorder requires depth frames"
            )
        frame_name = f"{self.saved_frames:0{FRAME_NAME_WIDTH}d}"
        self._enqueue_write(
            _FrameWriteJob(
                frame_name=frame_name,
                color_frame=np.array(capture.color, dtype=np.uint8, copy=True),
                depth_frame=np.array(capture.depth, dtype=np.uint16, copy=True),
                color_frame_number=capture.color_frame_number,
                depth_frame_number=capture.depth_frame_number,
                timestamp_ms=capture.timestamp_ms,
            ),
            wait_for_space=False,
        )
        self.saved_frames += 1
        return True

    def _enqueue_write(
        self, job: _FrameWriteJob | _FlushWriteJob, *, wait_for_space: bool
    ) -> None:
        self._raise_write_error()
        if self._write_queue is None:
            raise RuntimeError("recorder writer is not open")
        if not wait_for_space:
            try:
                self._write_queue.put_nowait(job)
            except queue.Full as exc:
                raise RuntimeError(
                    f"Recorder writer queue is full for {self.serial}; storage cannot keep up"
                ) from exc
            return

        deadline = time.monotonic() + WRITER_SHUTDOWN_TIMEOUT_SECONDS
        while True:
            self._raise_write_error()
            try:
                self._write_queue.put(job, timeout=0.1)
                return
            except queue.Full:
                if self._write_thread is not None and not self._write_thread.is_alive():
                    self._raise_write_error()
                    raise RuntimeError(f"Recorder writer stopped for {self.serial}")
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"Recorder writer queue timed out for {self.serial}")

    def _write_worker(self) -> None:
        write_queue = self._write_queue
        if write_queue is None:
            return
        try:
            while True:
                job = write_queue.get()
                if job is _STOP_WRITER:
                    return
                if isinstance(job, _FrameWriteJob):
                    self.writer.add_frame(job)
                    continue
                if isinstance(job, _FlushWriteJob):
                    try:
                        self.writer.flush()
                    finally:
                        if job.done is not None:
                            job.done.set()
                    continue
                raise RuntimeError(f"Unsupported recorder writer job: {job!r}")
        except BaseException as exc:
            self._write_error = exc

    def _raise_write_error(self) -> None:
        if self._write_error is not None:
            raise RuntimeError(
                f"Recorder writer failed for {self.serial}"
            ) from self._write_error

    def flush(self) -> None:
        if self._write_queue is not None:
            done = threading.Event()
            self._enqueue_write(_FlushWriteJob(done=done), wait_for_space=True)
            self._wait_for_flush(done)
        else:
            self.writer.flush()
        self._raise_write_error()
        self._print_saved_frames()

    def flush_async(self) -> None:
        if self._write_queue is not None:
            self._enqueue_write(_FlushWriteJob(), wait_for_space=False)
        else:
            self.writer.flush()
        self._print_saved_frames()

    def _wait_for_flush(self, done: threading.Event) -> None:
        deadline = time.monotonic() + WRITER_SHUTDOWN_TIMEOUT_SECONDS
        while not done.wait(timeout=0.1):
            if self._write_thread is not None and not self._write_thread.is_alive():
                self._raise_write_error()
                raise RuntimeError(f"Recorder writer stopped before flushing {self.serial}")
            if time.monotonic() >= deadline:
                raise RuntimeError(f"Recorder writer flush timed out for {self.serial}")

    def _print_saved_frames(self) -> None:
        print(
            f"[{self.backend.name}-recorder] {self.serial} saved {self.saved_frames} frame pairs",
            flush=True,
        )

    def close(self) -> None:
        close_error: BaseException | None = None
        try:
            try:
                if self._write_queue is not None:
                    self.flush()
                else:
                    self.writer.flush()
            except BaseException as exc:
                close_error = exc
            try:
                self._stop_writer()
            except BaseException as exc:
                if close_error is None:
                    close_error = exc
            try:
                self._write_deferred_pointclouds()
            except BaseException as exc:
                if close_error is None:
                    close_error = exc
        finally:
            try:
                self.writer.close()
            finally:
                self.camera.disconnect()
        if close_error is not None:
            raise close_error
        self._raise_write_error()

    def _stop_writer(self) -> None:
        write_thread = self._write_thread
        if (
            self._write_queue is not None
            and write_thread is not None
            and write_thread.is_alive()
        ):
            deadline = time.monotonic() + WRITER_SHUTDOWN_TIMEOUT_SECONDS
            while True:
                try:
                    self._write_queue.put(_STOP_WRITER, timeout=0.1)
                    break
                except queue.Full:
                    if not write_thread.is_alive():
                        break
                    if time.monotonic() >= deadline:
                        raise RuntimeError(
                            f"Recorder writer stop queue timed out for {self.serial}"
                        )
        if write_thread is not None:
            write_thread.join(timeout=WRITER_SHUTDOWN_TIMEOUT_SECONDS)
            if write_thread.is_alive():
                raise RuntimeError(f"Recorder writer did not stop for {self.serial}")
        self._write_thread = None
        self._write_queue = None

    def _write_deferred_pointclouds(self) -> None:
        if not self.record_pointcloud or self.saved_frames == 0:
            return
        depth_scale_meters = self.camera.depth_scale_meters
        intrinsic_matrix = self.camera.color_intrinsic_matrix
        if depth_scale_meters is None or intrinsic_matrix is None:
            raise RuntimeError("Cannot write pointclouds without depth scale and intrinsics")

        print(
            f"[{self.backend.name}-recorder] {self.serial} writing "
            f"{self.saved_frames} deferred pointcloud(s)",
            flush=True,
        )
        for index in range(self.saved_frames):
            frame_name = f"{index:0{FRAME_NAME_WIDTH}d}"
            color = self.writer.read_color_frame(frame_name)
            depth = self.writer.read_depth_frame(frame_name)
            write_pointcloud_ply(
                self.pointcloud_dir / f"{frame_name}.ply",
                color,
                depth,
                intrinsic_matrix,
                depth_scale_meters,
            )
        print(
            f"[{self.backend.name}-recorder] {self.serial} wrote "
            f"{self.saved_frames} deferred pointcloud(s)",
            flush=True,
        )

    def _write_metadata(self) -> None:
        metadata = {
            "created_at": datetime.now(timezone.utc).isoformat(),
            "backend": self.backend.name,
            "serial_number": self.serial,
            "width": self.camera.config.width,
            "height": self.camera.config.height,
            "fps": self.camera.config.fps,
            "warmup_frames": self.warmup_remaining,
            "align_depth_to_color": self.camera.config.align_depth_to_color,
            "record_pointcloud": self.record_pointcloud,
            "writer_backend": self.writer_backend,
            "frame_format": self.frame_format,
            "color_file_extension": self.color_extension,
            "depth_file_extension": self.depth_extension,
            "depth_scale_meters": self.camera.depth_scale_meters,
            "intrinsic_matrix": self.camera.color_intrinsic_matrix,
        }
        (self.camera_dir / "metadata.json").write_text(
            f"{json.dumps(metadata, indent=2)}\n", encoding="ascii"
        )


def _optional_int(value: int | None) -> int:
    return -1 if value is None else int(value)


def _require_h5py() -> Any:
    try:
        import h5py
    except ImportError as exc:
        raise RuntimeError(
            "--writer-backend hdf5 requires h5py; install the hdf5 extra or add h5py"
        ) from exc
    return h5py


def write_ppm(path: Path, rgb_frame: NDArray[np.uint8]) -> None:
    """Write an RGB frame as a binary PPM image."""

    height, width, channels = rgb_frame.shape
    if channels != 3:
        raise ValueError("RGB frame must have exactly 3 channels")
    header = f"P6\n{width} {height}\n255\n".encode("ascii")
    path.write_bytes(header + np.ascontiguousarray(rgb_frame, dtype=np.uint8).tobytes())


def write_pgm(path: Path, depth_frame: NDArray[np.uint16]) -> None:
    """Write a depth frame as a big-endian 16-bit PGM image."""

    height, width = depth_frame.shape
    header = f"P5\n{width} {height}\n65535\n".encode("ascii")
    path.write_bytes(header + np.ascontiguousarray(depth_frame, dtype=">u2").tobytes())


def write_rgb_png(path: Path, rgb_frame: NDArray[np.uint8]) -> None:
    """Write an RGB frame as a lossless compressed PNG image."""

    height, width, channels = rgb_frame.shape
    if channels != 3:
        raise ValueError("RGB frame must have exactly 3 channels")
    rows = np.ascontiguousarray(rgb_frame, dtype=np.uint8).reshape(height, width * channels)
    path.write_bytes(_png_bytes(width, height, bit_depth=8, color_type=2, rows=rows))


def write_depth_png(path: Path, depth_frame: NDArray[np.uint16]) -> None:
    """Write a depth frame as a lossless compressed 16-bit grayscale PNG image."""

    height, width = depth_frame.shape
    rows = np.ascontiguousarray(depth_frame, dtype=">u2").view(np.uint8).reshape(height, width * 2)
    path.write_bytes(_png_bytes(width, height, bit_depth=16, color_type=0, rows=rows))


def _png_bytes(
    width: int,
    height: int,
    *,
    bit_depth: int,
    color_type: int,
    rows: NDArray[np.uint8],
) -> bytes:
    header = struct.pack(">IIBBBBB", width, height, bit_depth, color_type, 0, 0, 0)
    image_data = zlib.compress(_png_scanlines(rows))
    return (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", header)
        + _png_chunk(b"IDAT", image_data)
        + _png_chunk(b"IEND", b"")
    )


def _png_scanlines(rows: NDArray[np.uint8]) -> bytes:
    return b"".join(b"\x00" + row.tobytes() for row in rows)


def _png_chunk(chunk_type: bytes, payload: bytes) -> bytes:
    checksum = zlib.crc32(payload, zlib.crc32(chunk_type)) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + chunk_type + payload + struct.pack(">I", checksum)


def read_ppm(path: Path) -> NDArray[np.uint8]:
    """Read a binary PPM image written by ``write_ppm``."""

    data = path.read_bytes()
    width, height, max_value, offset = _read_pnm_header(data, expected_magic=b"P6")
    if max_value != 255:
        raise ValueError(f"Unsupported PPM max value in {path}: {max_value}")
    expected_size = width * height * 3
    payload = data[offset : offset + expected_size]
    if len(payload) != expected_size:
        raise ValueError(f"PPM payload has wrong size: {path}")
    return np.frombuffer(payload, dtype=np.uint8).reshape((height, width, 3)).copy()


def read_pgm(path: Path) -> NDArray[np.uint16]:
    """Read a binary 16-bit PGM image written by ``write_pgm``."""

    data = path.read_bytes()
    width, height, max_value, offset = _read_pnm_header(data, expected_magic=b"P5")
    if max_value != 65535:
        raise ValueError(f"Unsupported PGM max value in {path}: {max_value}")
    expected_size = width * height * 2
    payload = data[offset : offset + expected_size]
    if len(payload) != expected_size:
        raise ValueError(f"PGM payload has wrong size: {path}")
    return np.frombuffer(payload, dtype=">u2").reshape((height, width)).astype(np.uint16)


def write_pointcloud_ply(
    path: Path,
    rgb_frame: NDArray[np.uint8],
    depth_frame: NDArray[np.uint16],
    intrinsic_matrix: Sequence[Sequence[float]],
    depth_scale_meters: float,
) -> None:
    """Write an aligned RGB-D frame pair as a binary little-endian PLY."""

    if rgb_frame.shape[:2] != depth_frame.shape:
        raise ValueError("RGB and depth frames must have matching dimensions")
    fx = float(intrinsic_matrix[0][0])
    fy = float(intrinsic_matrix[1][1])
    cx = float(intrinsic_matrix[0][2])
    cy = float(intrinsic_matrix[1][2])
    if fx == 0.0 or fy == 0.0:
        raise ValueError("Camera intrinsics must have non-zero focal lengths")

    depth_meters = depth_frame.astype(np.float32) * float(depth_scale_meters)
    valid = depth_frame > 0
    rows, cols = np.nonzero(valid)
    point_count = len(rows)
    z = depth_meters[rows, cols]
    x = (cols.astype(np.float32) - cx) * z / fx
    y = (rows.astype(np.float32) - cy) * z / fy
    colors = rgb_frame[rows, cols]

    points = np.empty(
        point_count,
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("red", "u1"),
            ("green", "u1"),
            ("blue", "u1"),
        ],
    )
    points["x"] = x
    points["y"] = y
    points["z"] = z
    points["red"] = colors[:, 0]
    points["green"] = colors[:, 1]
    points["blue"] = colors[:, 2]

    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {point_count}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    ).encode("ascii")
    path.write_bytes(header + points.tobytes())


def _read_pnm_header(
    data: bytes, *, expected_magic: bytes
) -> tuple[int, int, int, int]:
    tokens: list[bytes] = []
    offset = 0
    while len(tokens) < 4:
        offset = _skip_pnm_whitespace_and_comments(data, offset)
        token_start = offset
        while offset < len(data) and data[offset] not in b" \t\r\n":
            offset += 1
        if token_start == offset:
            raise ValueError("PNM header ended unexpectedly")
        tokens.append(data[token_start:offset])
    offset = _skip_single_pnm_whitespace(data, offset)
    magic, width, height, max_value = tokens
    if magic != expected_magic:
        raise ValueError(f"Expected {expected_magic.decode('ascii')}, got {magic!r}")
    return int(width), int(height), int(max_value), offset


def _skip_pnm_whitespace_and_comments(data: bytes, offset: int) -> int:
    while offset < len(data):
        if data[offset] in b" \t\r\n":
            offset += 1
            continue
        if data[offset] == ord("#"):
            while offset < len(data) and data[offset] not in b"\r\n":
                offset += 1
            continue
        break
    return offset


def _skip_single_pnm_whitespace(data: bytes, offset: int) -> int:
    if offset >= len(data) or data[offset] not in b" \t\r\n":
        raise ValueError("PNM header is missing payload separator")
    return offset + 1


def _validate_pointcloud_backend(backend_name: str) -> None:
    if backend_name != "orbbec":
        raise ValueError("--pointcloud is only supported with --backend orbbec")


def _validate_args(args: argparse.Namespace) -> None:
    if args.width < 1:
        raise ValueError("--width must be at least 1")
    if args.height < 1:
        raise ValueError("--height must be at least 1")
    if args.fps < 1:
        raise ValueError("--fps must be at least 1")
    if args.warmup_frames < 0:
        raise ValueError("--warmup-frames must be non-negative")
    if args.duration_seconds is not None and args.duration_seconds <= 0:
        raise ValueError("--duration-seconds must be greater than 0")
    if args.max_frames is not None and args.max_frames <= 0:
        raise ValueError("--max-frames must be greater than 0")
    if args.flush_every_frames is not None and args.flush_every_frames <= 0:
        raise ValueError("--flush-every-frames must be greater than 0")
    if getattr(args, "frame_format", "pnm") not in FRAME_FORMATS:
        raise ValueError(f"--frame-format must be one of: {', '.join(FRAME_FORMATS)}")
    if getattr(args, "writer_backend", "csv") not in WRITER_BACKENDS:
        raise ValueError(
            f"--writer-backend must be one of: {', '.join(WRITER_BACKENDS)}"
        )
    if getattr(args, "pointcloud", False) and getattr(args, "backend", None) != "orbbec":
        raise ValueError("--pointcloud requires --backend orbbec")
    if (
        getattr(args, "pointcloud", False)
        and getattr(args, "writer_backend", "csv") == "csv"
        and getattr(args, "frame_format", "pnm") != "pnm"
    ):
        raise ValueError("--pointcloud requires --frame-format pnm")
    if getattr(args, "pointcloud", False) and getattr(args, "no_align_depth", False):
        raise ValueError("--pointcloud requires depth aligned to color")


def _ensure_clean_output_dir(output_dir: Path) -> None:
    if not output_dir.exists():
        return
    if not output_dir.is_dir():
        raise NotADirectoryError(
            f"Recording output path is not a directory: {output_dir}"
        )
    if any(output_dir.iterdir()):
        raise FileExistsError(f"Recording output directory is not empty: {output_dir}")


def _safe_recording_dir_name(serial: str) -> str:
    safe_name = "".join(
        character if character.isalnum() or character in "_-" else "_"
        for character in serial
    ).strip("_")
    while "__" in safe_name:
        safe_name = safe_name.replace("__", "_")
    if safe_name in {"", ".", ".."}:
        return "camera"
    return safe_name


def _camera_dir_names(serials: Sequence[str]) -> dict[str, str]:
    names: dict[str, str] = {}
    used_names: set[str] = set()
    for serial in serials:
        name = _safe_recording_dir_name(serial)
        if name in used_names:
            raise ValueError(
                f"Camera serials produce duplicate recording directory: {name}"
            )
        names[serial] = name
        used_names.add(name)
    return names


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entry point."""

    return run_recorder(build_arg_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
