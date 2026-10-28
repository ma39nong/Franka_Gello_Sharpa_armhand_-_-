"""Realtime camera color/depth viewer."""

from __future__ import annotations

import argparse
import time
from collections.abc import Sequence
from importlib import import_module
from typing import Any

from barmate.hardware.cameras.tools.backends import backend_choices, camera_backend


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the command-line parser for the viewer tool."""

    parser = argparse.ArgumentParser(
        description="Display live camera color and depth streams."
    )
    parser.add_argument(
        "--backend",
        choices=backend_choices(),
        default="realsense",
        help="Camera backend to use for viewing.",
    )
    parser.add_argument(
        "--serial",
        action="append",
        dest="serials",
        help="Camera serial to display; repeat for multiple cameras.",
    )
    parser.add_argument(
        "--width", type=int, default=640, help="Stream width in pixels."
    )
    parser.add_argument(
        "--height", type=int, default=480, help="Stream height in pixels."
    )
    parser.add_argument("--fps", type=int, default=30, help="Stream frame rate.")
    parser.add_argument(
        "--no-align-depth",
        action="store_true",
        help="Do not align depth to the color stream.",
    )
    parser.add_argument(
        "--depth-alpha",
        type=float,
        default=0.03,
        help="Depth scale passed to OpenCV color mapping.",
    )
    parser.add_argument(
        "--window-prefix",
        default="BarMate RealSense",
        help="OpenCV window title prefix.",
    )
    parser.add_argument(
        "--duration-seconds", type=float, help="Optional duration before exiting."
    )
    parser.add_argument(
        "--max-frames", type=int, help="Optional total displayed frame limit."
    )
    return parser


def run_viewer(args: argparse.Namespace) -> int:
    """Run the realtime viewer until quit, duration, or frame limit."""

    if args.width < 1:
        raise ValueError("--width must be at least 1")
    if args.height < 1:
        raise ValueError("--height must be at least 1")
    if args.fps < 1:
        raise ValueError("--fps must be at least 1")
    if args.duration_seconds is not None and args.duration_seconds <= 0:
        raise ValueError("--duration-seconds must be greater than 0")
    if args.max_frames is not None and args.max_frames <= 0:
        raise ValueError("--max-frames must be greater than 0")

    backend = camera_backend(getattr(args, "backend", None))
    cv2 = import_module("cv2")
    serials = tuple(args.serials) if args.serials else tuple(backend.list_serials())
    if not serials:
        raise RuntimeError(f"No {backend.display_name} devices found")

    cameras = [
        backend.camera_class(
            serial=serial,
            width=args.width,
            height=args.height,
            fps=args.fps,
            align_depth_to_color=not args.no_align_depth,
        )
        for serial in serials
    ]
    frame_count = 0
    started_at = time.monotonic()
    poll_period_s = 1.0 / max(float(args.fps), 1e-6)
    next_poll_at = started_at
    try:
        for camera in cameras:
            camera.connect()
            print(f"[{backend.name}-viewer] Showing {camera.serial}", flush=True)

        while True:
            if args.max_frames is not None and frame_count >= args.max_frames:
                break
            if (
                args.duration_seconds is not None
                and time.monotonic() - started_at >= args.duration_seconds
            ):
                break
            now = time.monotonic()
            if now < next_poll_at:
                time.sleep(min(next_poll_at - now, 0.001))
                continue

            capture_timeout_ms = max(1.0, poll_period_s * 1000.0 / len(cameras))
            rendered = False
            for camera in cameras:
                try:
                    capture = camera.async_read_capture(timeout_ms=capture_timeout_ms)
                except TimeoutError:
                    continue
                if capture.depth is None:
                    cv2.imshow(
                        f"{args.window_prefix} {camera.serial} color", capture.color
                    )
                else:
                    cv2.imshow(
                        f"{args.window_prefix} {camera.serial} color | depth",
                        color_depth_preview(
                            cv2, capture.color, capture.depth, args.depth_alpha
                        ),
                    )
                frame_count += 1
                rendered = True

            key = int(cv2.waitKey(1 if rendered else 10)) & 0xFF
            if key in {27, ord("q")}:
                break
            next_poll_at += poll_period_s
            if time.monotonic() - next_poll_at > poll_period_s:
                next_poll_at = time.monotonic() + poll_period_s
    finally:
        for camera in cameras:
            camera.disconnect()
        cv2.destroyAllWindows()
    return 0


def color_depth_preview(
    cv2: Any, color_image: Any, depth_image: Any, depth_alpha: float
) -> Any:
    """Build a side-by-side OpenCV preview image."""

    np = import_module("numpy")
    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image, alpha=depth_alpha), cv2.COLORMAP_JET
    )
    if depth_colormap.shape[:2] != color_image.shape[:2]:
        depth_colormap = cv2.resize(
            depth_colormap,
            (color_image.shape[1], color_image.shape[0]),
            interpolation=cv2.INTER_AREA,
        )
    return np.hstack((color_image, depth_colormap))


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entry point."""

    return run_viewer(build_arg_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
