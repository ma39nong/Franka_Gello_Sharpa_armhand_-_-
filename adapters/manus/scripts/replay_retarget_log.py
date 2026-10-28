#!/usr/bin/env python3
"""Re-run recorded MANUS landmarks through the deployed retargeter offline.

Input must be ``hand-retarget-debug.v2`` JSONL. No socket or hardware driver is
opened. Old qpos, derived FK, and method statistics are deliberately ignored;
the raw canonical landmarks are the replay contract.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "adapters" / "pico" / "src"))
sys.path.insert(0, str(REPO_ROOT / "adapters" / "manus" / "python"))

from manus_teleop.pipeline import _create_retargeter  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--side", choices=("left", "right", "both"), default="left")
    parser.add_argument("--filter-alpha", type=float, default=0.85)
    args = parser.parse_args()
    if not args.input.is_file():
        parser.error(f"input does not exist: {args.input}")
    if args.output.exists():
        parser.error(f"refusing to overwrite output: {args.output}")
    if not 0.0 < args.filter_alpha <= 1.0:
        parser.error("--filter-alpha must be in (0, 1]")

    sides = ("left", "right") if args.side == "both" else (args.side,)
    # This replay reproduces the landmark method; the sharpa method
    # need the raw 25x7 frames rather than the canonical landmarks logged here.
    hands = {"left": "g20", "right": "o30i"}
    methods = {side: "landmark" for side in ("left", "right")}
    retargeters = {
        side: _create_retargeter(
            side, hands[side], methods[side], args.filter_alpha
        )
        for side in sides
    }
    counts = {side: 0 for side in sides}
    losses = {side: [] for side in sides}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    try:
        with args.input.open(encoding="utf-8") as source, args.output.open(
            "x", encoding="utf-8"
        ) as sink:
            header_line = source.readline()
            try:
                old_header = json.loads(header_line)
            except json.JSONDecodeError as error:
                raise ValueError("input header is not JSON") from error
            if old_header.get("schema") != "hand-retarget-debug.v2":
                raise ValueError("input is not hand-retarget-debug.v2")
            sink.write(
                json.dumps(
                    {
                        "schema": "hand-retarget-debug.v2",
                        "written_at": time.time(),
                        "metadata": {
                            "source": "offline-retarget-replay",
                            "input": str(args.input.resolve()),
                            "input_metadata": old_header.get("metadata", {}),
                            "hands": hands,
                            "methods": methods,
                            "filter_alpha": args.filter_alpha,
                            "urdfs": {
                                side: str(retargeters[side].urdf_path)
                                for side in sides
                            },
                            "joint_contracts": {
                                side: {
                                    "joint_names": list(retargeters[side].joint_names),
                                    "lower": np.asarray(retargeters[side].lower).tolist(),
                                    "upper": np.asarray(retargeters[side].upper).tolist(),
                                }
                                for side in sides
                            },
                        },
                        "fields": "canonical landmarks replayed through current deployed retargeter; old qpos, derived data, and stats discarded",
                    },
                    separators=(",", ":"),
                )
                + "\n"
            )
            for line_number, line in enumerate(source, start=2):
                if not line.strip():
                    continue
                row = json.loads(line)
                side = row.get("side")
                if side not in retargeters:
                    continue
                landmarks = np.asarray(row.get("landmarks"), dtype=np.float64)
                retargeter = retargeters[side]
                qpos, stats = retargeter.retarget(landmarks)
                raw_qpos = qpos
                if hasattr(retargeter, "last_qpos"):
                    raw_qpos = retargeter.last_qpos
                    if hasattr(retargeter, "_expand_qpos"):
                        raw_qpos = retargeter._expand_qpos(raw_qpos)
                targets = retargeter.target_positions(landmarks)
                if np.asarray(targets).shape != (21, 3):
                    canonical_targets = np.zeros((21, 3), dtype=np.float64)
                    for target, point in zip(retargeter.targets, targets, strict=True):
                        canonical_targets[target.landmark_index] = point
                    targets = canonical_targets
                robot_points = (
                    retargeter.robot_landmarks(qpos)
                    if hands[side] == "o30i"
                    else retargeter.robot_landmarks()
                )
                replayed = {
                    "wall_time_ns": row.get("wall_time_ns"),
                    "t": row.get("t"),
                    "side": side,
                    "source": row.get("source"),
                    "landmarks": landmarks.round(7).tolist(),
                    "joint_names": list(retargeter.joint_names),
                    "raw_qpos": np.asarray(raw_qpos).round(6).tolist(),
                    "qpos": np.asarray(qpos).round(6).tolist(),
                    "transport": row.get("transport"),
                    "derived": {
                        "target_landmarks_robot": np.asarray(targets).round(7).tolist(),
                        "robot_landmarks_emitted": np.asarray(robot_points).round(7).tolist(),
                    },
                    "stats": {
                        key: bool(value) if isinstance(value, (bool, np.bool_))
                        else int(value) if isinstance(value, (int, np.integer))
                        else round(float(value), 7)
                        for key, value in stats.items()
                    },
                    "replay_input_line": line_number,
                }
                sink.write(json.dumps(replayed, separators=(",", ":")) + "\n")
                counts[side] += 1
                losses[side].append(float(stats["loss"]))
    finally:
        for retargeter in retargeters.values():
            retargeter.close()

    elapsed = time.monotonic() - started
    print(f"Wrote {args.output} in {elapsed:.1f}s")
    for side in sides:
        values = np.asarray(losses[side], dtype=np.float64)
        if values.size:
            print(
                f"{side}: {counts[side]} frames, loss median={np.median(values):.6f}, "
                f"p95={np.percentile(values, 95):.6f}"
            )
        else:
            print(f"{side}: no frames")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
