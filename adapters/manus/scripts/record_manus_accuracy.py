#!/usr/bin/env python3
"""Record a labelled, hardware-disabled MANUS hand-accuracy protocol.

The UDP output is captured by a private local sink, so this utility never
commands a hand. Stop the normal operator first: only one MANUS client may run.
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "adapters" / "pico" / "src"))
sys.path.insert(0, str(REPO_ROOT / "adapters" / "manus" / "python"))

from manus_teleop import ManusHandPipeline  # noqa: E402


PHASES = (
    (
        "fully_open",
        "Fully open and straighten all five human fingers.",
    ),
    (
        "four_fingers_fully_curled",
        "Fully curl index, middle, ring and pinky; keep the thumb open.",
    ),
    (
        "thumb_index_pinch",
        "Touch the thumb tip to the index fingertip in a firm pinch.",
    ),
    (
        "thumb_fully_curled",
        "Fully curl the thumb into the palm; keep the other fingers open.",
    ),
    (
        "thumb_middle_pinch",
        "Touch the thumb tip to the middle fingertip in a firm pinch.",
    ),
    (
        "index_middle_pinch",
        "Touch the index and middle fingertips together firmly with no object; "
        "keep both fingers straight and the thumb clear.",
    ),
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument(
        "--sides", choices=("left", "right", "both"), default="both"
    )
    parser.add_argument("--filter-alpha", type=float, default=0.85)
    parser.add_argument("--ready-timeout", type=float, default=15.0)
    args = parser.parse_args()
    if args.seconds <= 0.0 or args.ready_timeout <= 0.0:
        parser.error("--seconds and --ready-timeout must be positive")
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    sides = ("left", "right") if args.sides == "both" else (args.sides,)

    # An ephemeral private receiver guarantees that no live hand bridge can see
    # these packets. It also makes accidental use during a Compose bringup safe.
    sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sink.bind(("127.0.0.1", 0))
    sink.setblocking(False)
    pipeline = None
    try:
        pipeline = ManusHandPipeline(
            host="127.0.0.1",
            port=sink.getsockname()[1],
            rate=30.0,
            filter_alpha=args.filter_alpha,
            debug_log=args.output,
            hands={"left": "g20", "right": "o30i"},
            methods={"left": "landmark", "right": "landmark"},
            dynamic_sides=sides,
        )
        print("Connected to MANUS. No robot commands can leave this process.")
        print(f"Waiting up to {args.ready_timeout:.0f}s for {', '.join(sides)} gloves...")
        inactive = {side: False for side in ("left", "right")}
        ready_deadline = time.monotonic() + args.ready_timeout
        while time.monotonic() < ready_deadline:
            started = time.monotonic()
            pipeline.tick(started, active=inactive)
            if all(pipeline.last_frame[side] is not None for side in sides):
                break
            remaining = 0.01 - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)
        missing = [side for side in sides if pipeline.last_frame[side] is None]
        if missing:
            faults = {
                side: pipeline.status.sides[side].fault for side in missing
            }
            raise RuntimeError(
                f"no calibrated MANUS frames for {missing}; status={faults}. "
                "Check glove power, MANUS license, and calibration files."
            )
        print(f"Both requested gloves are live. Recording to {args.output}")
        active = {side: side in sides for side in ("left", "right")}
        minimum_frames = max(5, int(args.seconds * 2.0))
        for phase, instruction in PHASES:
            pipeline.set_debug_phase(None)
            input(f"\n{instruction}\nHold the pose, then press Enter to record: ")
            pipeline.set_debug_phase(phase)
            observed = {side: set() for side in sides}
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                started = time.monotonic()
                pipeline.tick(started, active=active)
                for side in sides:
                    frame = pipeline.last_frame[side]
                    if frame is not None:
                        observed[side].add(int(frame.sequence))
                try:
                    while sink.recv(65_535):
                        pass
                except BlockingIOError:
                    pass
                remaining = 0.01 - (time.monotonic() - started)
                if remaining > 0.0:
                    time.sleep(remaining)
            insufficient = {
                side: len(sequences)
                for side, sequences in observed.items()
                if len(sequences) < minimum_frames
            }
            if insufficient:
                raise RuntimeError(
                    f"phase {phase!r} did not receive enough new MANUS frames: "
                    f"{insufficient}; required at least {minimum_frames} per side"
                )
            print(
                f"Captured {phase}: "
                + ", ".join(
                    f"{side}={len(observed[side])} frames" for side in sides
                )
            )
        pipeline.set_debug_phase(None)
    finally:
        if pipeline is not None:
            pipeline.close()
        sink.close()
    print(f"Saved {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
