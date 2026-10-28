#!/usr/bin/env python3
"""Drive the LinkerHands from MANUS gloves, no arms - per-side activation.

`--sides left` tests only the left hand, `--sides right` only the right,
`both` (default) runs bimanual. Both mounts are O30i by default; a G20 side
is still selectable via `--left-hand`/`--right-hand`. Keys: `L`/`R` toggle
one side, `Space` toggles the selected sides together, `X` stops, `O`
requests the open pose while disengaged, `Q` exits.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "adapters" / "pico" / "src"))
sys.path.insert(0, str(REPO_ROOT / "adapters" / "manus" / "python"))

from manus_teleop import ManusHandPipeline  # noqa: E402
from manus_teleop.pipeline import (  # noqa: E402
    DEFAULT_HANDS,
    DEFAULT_METHODS,
    split_legacy_model,
)
from pico_bimanual_franka_teleop.xr_input import KeyboardActivation  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sides",
        default="both",
        choices=("left", "right", "both"),
        help="which hands follow their gloves (default: both)",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5570)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--stale-timeout", type=float, default=0.25)
    parser.add_argument(
        "--filter-alpha",
        type=float,
        default=0.85,
        help="EMA response in (0, 1]; 1 disables retarget-output smoothing",
    )
    # Which hand is attached is a fact about the robot; which method runs is a
    # free choice. They used to share one --*-model flag, whose value also
    # travelled on the wire as the hardware tag -- so selecting a method
    # silently made the bridge drop every packet.
    parser.add_argument(
        "--left-method",
        default=None,
        choices=("landmark", "sharpa"),
        help="left retargeting method: sharpa is the twelve-term objective "
             "ported from the SharpaWave optimiser, driven by an operator "
             "calibration; landmark is the earlier canonical-landmark solve "
             "(default: sharpa)",
    )
    parser.add_argument(
        "--right-method",
        default=None,
        choices=("landmark", "sharpa"),
        help="right retargeting method, same choices as --left-method "
             "(default: sharpa)",
    )
    parser.add_argument(
        "--right-hand",
        default=None,
        choices=("g20", "o30i"),
        help="physical right hand; must match the bridge's right_model in "
             "docker/compose.yaml (default: o30i)",
    )
    parser.add_argument(
        "--left-hand",
        default=None,
        choices=("g20", "o30i"),
        help="physical left hand; must match the bridge's left_model in "
             "docker/compose.yaml (default: o30i)",
    )
    parser.add_argument(
        "--left-model",
        default=None,
        choices=("g20", "g20_casadi"),
        help="deprecated: use --left-method",
    )
    parser.add_argument(
        "--right-model",
        default=None,
        choices=("g20", "o30i", "o30i_casadi"),
        help="deprecated: use --right-method and --right-hand",
    )
    parser.add_argument("--keyboard-device", default="/dev/tty")
    parser.add_argument("--debug-log")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="stop after this many seconds; zero runs until Q or Ctrl-C",
    )
    args = parser.parse_args()
    if args.duration < 0.0:
        parser.error("--duration must not be negative")
    sides = ("left", "right") if args.sides == "both" else (args.sides,)

    hands = dict(DEFAULT_HANDS)
    methods = dict(DEFAULT_METHODS)
    for side, legacy, new in (
        ("left", args.left_model, args.left_method),
        ("right", args.right_model, args.right_method),
    ):
        if legacy is not None:
            if new is not None:
                parser.error(
                    f"--{side}-model is deprecated; pass --{side}-method alone"
                )
            hands[side], methods[side] = split_legacy_model(legacy)
            print(
                f"  WARNING --{side}-model is deprecated: read as "
                f"--{side}-method {methods[side]}"
                + f" --{side}-hand {hands[side]}"
            )
        elif new is not None:
            methods[side] = new
    if args.right_hand is not None:
        if args.right_model is not None:
            parser.error("--right-hand conflicts with the deprecated --right-model")
        hands["right"] = args.right_hand
    if args.left_hand is not None:
        if args.left_model is not None:
            parser.error("--left-hand conflicts with the deprecated --left-model")
        hands["left"] = args.left_hand

    # State the wire contract before anything can fail. The bridge announces
    # the hands it was launched for; a disagreement between the two banners is
    # the whole failure mode and is otherwise invisible, because UDP sends
    # succeed while the bridge drops every packet it cannot match.
    for side in sides:
        print(
            f"  {side:5s}: hand={hands[side]} method={methods[side]} "
            f"-> packets tagged model={hands[side]!r} "
            f"(bridge must run with {side}_model:={hands[side]})"
        )

    pipeline = None
    keyboard = None
    try:
        pipeline = ManusHandPipeline(
            dynamic_sides=sides,
            host=args.host,
            port=args.port,
            rate=args.rate,
            stale_timeout=args.stale_timeout,
            filter_alpha=args.filter_alpha,
            debug_log=args.debug_log,
            hands=hands,
            methods=methods,
        )
        keyboard = KeyboardActivation(args.keyboard_device, sides=sides)
        keyboard.show(
            f"MANUS hands ({', '.join(sides)}) start DISENGAGED. "
            "L/R toggles one side, Space toggles together; X stops; "
            "O requests open while disengaged; Q exits."
        )
        started = time.monotonic()
        deadline = started + args.duration if args.duration > 0.0 else None
        next_report = started
        previous_sent = {side: 0 for side in sides}
        while deadline is None or time.monotonic() < deadline:
            loop_started = time.monotonic()
            active = keyboard.poll()
            requests = keyboard.take_requests()
            if requests["open_hands"]:
                pipeline.request_open(loop_started)
            pipeline.tick(loop_started, active=active)

            if loop_started >= next_report:
                parts = []
                elapsed = max(loop_started - (next_report - 1.0), 1e-6)
                for side in sides:
                    status = pipeline.status.sides[side]
                    send_rate = (status.sent - previous_sent[side]) / elapsed
                    previous_sent[side] = status.sent
                    parts.append(
                        f"{side}: sending {send_rate:.1f} Hz, solve "
                        f"{status.solve_seconds * 1e3:.1f} ms"
                        if status.sending
                        else f"{side}: not sending ({status.fault})"
                    )
                keyboard.show(" | ".join(parts))
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
