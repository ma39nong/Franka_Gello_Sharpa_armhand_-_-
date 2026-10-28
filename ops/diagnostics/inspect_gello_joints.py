#!/usr/bin/env python3
"""Inspect dual-GELLO joint streams without connecting to either Franka.

The upstream driver disables Dynamixel torque once during initialization. It
then reads present position/velocity only; this tool never enables torque and
never sends a goal position or current.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "adapters" / "pico" / "src"))

from pico_bimanual_franka_teleop.gello_input import (  # noqa: E402
    DualGelloJointInput,
    load_gello_config,
)
from pico_bimanual_franka_teleop.types import SIDES  # noqa: E402


class _PassiveOperator:
    def poll(self) -> dict[str, bool]:
        return {side: False for side in SIDES}

    def deny(self, _side: str, _reason: str) -> None:
        return None


def _degrees(values: np.ndarray) -> str:
    return "[" + ", ".join(f"{value:7.2f}" for value in np.degrees(values)) + "]"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Read both GELLO joint streams without connecting to either Franka. "
            "Initialization disables GELLO servo torque once."
        )
    )
    parser.add_argument(
        "--config", default=str(REPO_ROOT / "config" / "modes" / "gello.yaml")
    )
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--interval", type=float, default=0.02)
    args = parser.parse_args()
    if args.duration <= 0.0 or args.interval <= 0.0:
        parser.error("duration and interval must be positive")

    print("Opening motors 1-7 on both GELLOs; motor 8 is not opened.")
    print("The driver will disable GELLO servo torque; no goal command is sent.")
    try:
        source = DualGelloJointInput(
            load_gello_config(args.config),
            _PassiveOperator(),
        )
    except RuntimeError as error:
        print(f"[FAIL] {error}", file=sys.stderr)
        return 1
    started = time.monotonic()
    first: dict[str, np.ndarray] = {}
    previous: dict[str, np.ndarray] = {}
    minimum: dict[str, np.ndarray] = {}
    maximum: dict[str, np.ndarray] = {}
    max_step = {side: np.zeros(7) for side in SIDES}
    initial_state = source.debug_feed_state()
    start_counts = {
        side: int(initial_state[side]["samples"]) for side in SIDES
    }
    last_report = started
    try:
        while time.monotonic() - started < args.duration:
            sample = source.sample()
            if sample is None:
                time.sleep(args.interval)
                continue
            for side in SIDES:
                q = np.asarray(sample.positions[side], dtype=float)
                if side not in first:
                    first[side] = q.copy()
                    previous[side] = q.copy()
                    minimum[side] = q.copy()
                    maximum[side] = q.copy()
                max_step[side] = np.maximum(
                    max_step[side], np.abs(q - previous[side])
                )
                minimum[side] = np.minimum(minimum[side], q)
                maximum[side] = np.maximum(maximum[side], q)
                previous[side] = q.copy()
            now = time.monotonic()
            if now - last_report >= 1.0:
                print(source.status_summary())
                last_report = now
            time.sleep(args.interval)

        elapsed = time.monotonic() - started
        state = source.debug_feed_state()
        failures = 0
        for side in SIDES:
            side_state = state[side]
            if side not in first:
                print(f"[FAIL] {side}: no joint sample")
                failures += 1
                continue
            count = int(side_state["samples"]) - start_counts[side]
            rate = count / elapsed
            error = side_state["error"]
            age = side_state["age"]
            delta = previous[side] - first[side]
            span = maximum[side] - minimum[side]
            age_ms = float("inf") if age is None else age * 1e3
            print(f"\n{side}: {rate:.1f} driver samples/s, age={age_ms:.1f} ms")
            print(f"  start deg : {_degrees(first[side])}")
            print(f"  delta deg : {_degrees(delta)}")
            print(f"  span deg  : {_degrees(span)}")
            print(f"  max step  : {_degrees(max_step[side])}")
            if error:
                print(f"[FAIL] {side}: {error}")
                failures += 1
            elif age is None or age > source.config.stale_timeout:
                print(f"[FAIL] {side}: stream is stale")
                failures += 1
            elif count == 0:
                print(f"[FAIL] {side}: sample counter did not advance")
                failures += 1
            else:
                print(f"[PASS] {side}: live seven-joint stream")
        return 1 if failures else 0
    finally:
        source.close()


if __name__ == "__main__":
    raise SystemExit(main())
