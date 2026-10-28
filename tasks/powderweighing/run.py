#!/usr/bin/env python3
"""Run the right MANUS/O30i with two powder-weighing hardcoded poses."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "adapters" / "pico" / "src"))
sys.path.insert(0, str(ROOT / "adapters" / "manus" / "python"))

from manus_teleop import ManusHandPipeline  # noqa: E402
from pico_bimanual_franka_teleop.xr_input import KeyboardActivation  # noqa: E402
from tasks.powderweighing.strategy import (  # noqa: E402
    HardcodedPitchRetargeter,
    load_config,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("config") / "poses.json",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5570)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--stale-timeout", type=float, default=0.25)
    parser.add_argument("--filter-alpha", type=float, default=0.85)
    parser.add_argument("--keyboard-device", default="/dev/tty")
    parser.add_argument("--debug-log")
    parser.add_argument("--duration", type=float, default=0.0)
    args = parser.parse_args()
    if args.duration < 0.0:
        parser.error("--duration must not be negative")
    try:
        config = load_config(args.config)
    except (OSError, KeyError, TypeError, ValueError) as error:
        parser.error(str(error))

    pipeline = None
    keyboard = None
    try:
        pipeline = ManusHandPipeline(
            dynamic_sides=("right",),
            host=args.host,
            port=args.port,
            rate=args.rate,
            stale_timeout=args.stale_timeout,
            filter_alpha=args.filter_alpha,
            debug_log=args.debug_log,
            hands={"left": "o30i", "right": "o30i"},
            methods={"left": "sharpa", "right": "sharpa"},
        )
        base = pipeline.retargeters["right"]
        pipeline.retargeters["right"] = HardcodedPitchRetargeter(base, config)
        keyboard = KeyboardActivation(args.keyboard_device, sides=("right",))
        keyboard.show(
            "POWDERWEIGHING RIGHT starts DISENGAGED. R toggles; X stops; "
            "O opens while disengaged; Q exits."
        )
        started = time.monotonic()
        deadline = started + args.duration if args.duration > 0.0 else None
        next_report = started
        previous_sent = 0
        while deadline is None or time.monotonic() < deadline:
            loop_started = time.monotonic()
            active = keyboard.poll()
            requests = keyboard.take_requests()
            if requests["open_hands"]:
                pipeline.request_open(loop_started, sides=("right",))
            pipeline.tick(loop_started, active=active)

            if loop_started >= next_report:
                status = pipeline.status.sides["right"]
                elapsed = max(loop_started - (next_report - 1.0), 1e-6)
                rate = (status.sent - previous_sent) / elapsed
                previous_sent = status.sent
                retargeter = pipeline.retargeters["right"]
                if status.sending:
                    keyboard.show(
                        f"RIGHT {retargeter.phase.value:<11} | "
                        f"MANUS T-I {retargeter._last_gap_m * 1000:5.1f} mm | "
                        f"send {rate:4.1f} Hz | "
                        f"solve {status.solve_seconds * 1000:4.1f} ms"
                    )
                else:
                    keyboard.show(f"RIGHT not sending ({status.fault})")
                next_report = loop_started + 1.0

            remaining = 0.01 - (time.monotonic() - loop_started)
            if remaining > 0.0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        if keyboard is not None:
            keyboard.close()
        if pipeline is not None:
            pipeline.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
