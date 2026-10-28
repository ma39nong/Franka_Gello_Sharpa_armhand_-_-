#!/usr/bin/env python3
"""Summarize the six labelled MANUS endpoint and pinch criteria."""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "adapters" / "pico" / "src"))
sys.path.insert(0, str(REPO_ROOT / "adapters" / "manus" / "python"))

FINGERS = ("index", "middle", "ring", "pinky")


def percentile(values: list[float]) -> str:
    data = np.asarray(values, dtype=float)
    return (
        f"median={np.median(data):.1f}, p05={np.percentile(data, 5):.1f}, "
        f"p95={np.percentile(data, 95):.1f}"
    )


def bend(points: np.ndarray, indices: tuple[int, int, int, int]) -> float:
    segments = np.diff(points[list(indices)], axis=0)
    total = 0.0
    for first, second in zip(segments[:-1], segments[1:], strict=True):
        denom = np.linalg.norm(first) * np.linalg.norm(second)
        if denom < 1e-12:
            return float("nan")
        total += np.arccos(np.clip(np.dot(first, second) / denom, -1.0, 1.0))
    return float(np.degrees(total))


def closure(row: dict, contract: dict, selected: tuple[str, ...]) -> float:
    names = row["joint_names"]
    qpos = np.asarray(row["qpos"], dtype=float)
    lower = dict(zip(contract["joint_names"], contract["lower"], strict=True))
    upper = dict(zip(contract["joint_names"], contract["upper"], strict=True))
    values = []
    for index, name in enumerate(names):
        ordinary_flex = name.startswith(selected) and any(
            name.endswith(suffix) for suffix in ("mcp_pitch", "pip", "dip")
        )
        thumb_flex = "thumb" in selected and name in {
            "thumb_cmc_pitch", "thumb_mcp", "thumb_ip", "thumb_dip"
        }
        if not (ordinary_flex or thumb_flex):
            continue
        span = upper[name] - lower[name]
        if span > 1e-9:
            values.append((qpos[index] - lower[name]) / span)
    return 100.0 * float(np.mean(values))


def hands_and_methods(metadata: dict) -> tuple[dict, dict]:
    """Per-side hardware and method, from either log schema.

    Logs written before the two were separated carry one combined "models"
    key; decode those through the same table the deprecated CLI flags use.
    """
    from manus_teleop.pipeline import split_legacy_model

    if "hands" in metadata and "methods" in metadata:
        return dict(metadata["hands"]), dict(metadata["methods"])
    legacy = {
        side: split_legacy_model(model)
        for side, model in metadata["models"].items()
    }
    return (
        {side: pair[0] for side, pair in legacy.items()},
        {side: pair[1] for side, pair in legacy.items()},
    )


def replay_current(rows: list[dict], metadata: dict) -> None:
    """Replace recorded method outputs using current code and exact inputs."""
    from manus_teleop.pipeline import _create_retargeter

    hands, methods = hands_and_methods(metadata)
    sides = sorted({row["side"] for row in rows})
    retargeters = {
        side: _create_retargeter(
            side, hands[side], methods[side], metadata["filter_alpha"]
        )
        for side in sides
    }
    try:
        for row in rows:
            retargeter = retargeters[row["side"]]
            qpos, stats = retargeter.retarget(np.asarray(row["landmarks"], dtype=float))
            robot = (
                retargeter.robot_landmarks(qpos)
                if (hands[row["side"]] == "o30i"
                    and methods[row["side"]] == "landmark")
                else retargeter.robot_landmarks()
            )
            row["qpos"] = np.asarray(qpos).tolist()
            row["stats"] = stats
            row.setdefault("derived", {})["robot_landmarks_emitted"] = (
                np.asarray(robot).tolist()
            )
    finally:
        for retargeter in retargeters.values():
            retargeter.close()


def pair_gaps(
    rows: list[dict], first_tip: int, second_tip: int
) -> tuple[list[float], list[float], list[float]]:
    robot_gaps = []
    target_gaps = []
    human_gaps = []
    for row in rows:
        landmarks = np.asarray(row.get("landmarks"), dtype=float)
        if landmarks.shape == (21, 3):
            human_gaps.append(
                1000.0
                * float(np.linalg.norm(landmarks[first_tip] - landmarks[second_tip]))
            )
        derived = row.get("derived") or {}
        robot = np.asarray(derived.get("robot_landmarks_emitted"), dtype=float)
        target = np.asarray(derived.get("target_landmarks_robot"), dtype=float)
        if robot.shape == (21, 3):
            robot_gaps.append(
                1000.0
                * float(np.linalg.norm(robot[first_tip] - robot[second_tip]))
            )
        if target.shape == (21, 3):
            target_gaps.append(
                1000.0
                * float(np.linalg.norm(target[first_tip] - target[second_tip]))
            )
    return robot_gaps, target_gaps, human_gaps


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording")
    parser.add_argument(
        "--replay-current",
        action="store_true",
        help="run the recorded landmarks through the current method before reporting",
    )
    args = parser.parse_args()
    with open(args.recording, encoding="utf-8") as stream:
        header = json.loads(next(stream))
        rows = [json.loads(line) for line in stream]
    if header.get("schema") != "hand-retarget-debug.v2":
        raise SystemExit("expected hand-retarget-debug.v2")
    if args.replay_current:
        replay_current(rows, header["metadata"])
        print("Replayed exact recorded landmarks through the current method.")
    contracts = header["metadata"].get("joint_contracts", {})
    grouped = defaultdict(list)
    for row in rows:
        phase = (row.get("transport") or {}).get("phase")
        if phase:
            grouped[(row["side"], phase)].append(row)
    if not grouped:
        raise SystemExit("recording has no guided protocol phase labels")

    for side in ("left", "right"):
        available = {phase for selected, phase in grouped if selected == side}
        if not available:
            continue
        log_hands, log_methods = hands_and_methods(header["metadata"])
        print(f"\n=== {side} ({log_hands[side]}, {log_methods[side]}) ===")
        contract = contracts[side]
        opened = [
            closure(row, contract, FINGERS + ("thumb",))
            for row in grouped[(side, "fully_open")]
        ]
        curled = [
            closure(row, contract, FINGERS)
            for row in grouped[(side, "four_fingers_fully_curled")]
        ]
        thumb = [
            closure(row, contract, ("thumb",))
            for row in grouped[(side, "thumb_fully_curled")]
        ]
        gaps, target_gaps, _ = pair_gaps(
            grouped[(side, "thumb_index_pinch")], 4, 8
        )
        middle_gaps, middle_target_gaps, _ = pair_gaps(
            grouped[(side, "thumb_middle_pinch")], 4, 12
        )
        index_middle_gaps, index_middle_targets, index_middle_human = pair_gaps(
            grouped[(side, "index_middle_pinch")], 8, 12
        )
        thumb_bends = []
        for row in grouped[(side, "thumb_fully_curled")]:
            robot = np.asarray(
                (row.get("derived") or {}).get("robot_landmarks_emitted"),
                dtype=float,
            )
            if robot.shape == (21, 3):
                thumb_bends.append(bend(robot, (1, 2, 3, 4)))

        print("1 fully open: emitted flex closure %, ideal 0")
        print("  " + percentile(opened))
        print("2 four fingers curled: emitted flex closure %, ideal 100")
        print("  all four: " + percentile(curled))
        finger_chains = {
            "index": (5, 6, 7, 8),
            "middle": (9, 10, 11, 12),
            "ring": (13, 14, 15, 16),
            "pinky": (17, 18, 19, 20),
        }
        for finger in FINGERS:
            phase_rows = grouped[(side, "four_fingers_fully_curled")]
            per_finger = [
                closure(row, contract, (finger,)) for row in phase_rows
            ]
            human_bends = [
                bend(np.asarray(row["landmarks"]), finger_chains[finger])
                for row in phase_rows
            ]
            print(
                f"  {finger}: {percentile(per_finger)}; "
                f"MANUS chain bend deg {percentile(human_bends)}"
            )
        physical_pinch = bool(
            (header["metadata"].get("left_hand_calibration") or {}).get(
                "physical_pinch_calibration"
            )
        )
        if side == "left" and physical_pinch:
            print(
                "3 thumb-index pinch: L20 URDF FK gap mm "
                "(diagnostic for the pose saved in this recording)"
            )
        else:
            print("3 thumb-index pinch: robot FK gap mm, ideal 0")
        print("  " + (percentile(gaps) if gaps else "no FK data"))
        if target_gaps:
            print("  normalized MANUS target gap mm: " + percentile(target_gaps))
        print("4 thumb fully curled: emitted flex closure %, ideal 100")
        print("  " + percentile(thumb))
        if thumb_bends:
            print("  robot FK total thumb bend deg: " + percentile(thumb_bends))
        print("5 thumb-middle pinch: robot FK gap mm, ideal 0")
        print("  " + (percentile(middle_gaps) if middle_gaps else "not recorded"))
        if middle_target_gaps:
            print(
                "  normalized MANUS target gap mm: "
                + percentile(middle_target_gaps)
            )
        print("6 index-middle contact: robot FK gap mm, ideal 0")
        print(
            "  "
            + (
                percentile(index_middle_gaps)
                if index_middle_gaps
                else "not recorded"
            )
        )
        if index_middle_targets:
            print(
                "  normalized MANUS target gap mm: "
                + percentile(index_middle_targets)
            )
        if index_middle_human:
            print(
                "  raw MANUS fingertip gap mm: "
                + percentile(index_middle_human)
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
