#!/usr/bin/env python3
"""Replay one recording through both retargeters and diff the joint angles.

    python adapters/manus/scripts/compare_retargeters.py \
        /home/descfly/franka_teleop_data/manus_accuracy/v101_retargeted/left_full_01_v101.jsonl \
        --profile config/calibration/hand_profiles/left_manus.json

The recorded ``qpos`` already is the deployed landmark retargeter's answer for
these frames, and ``source.keypoints`` is the raw MANUS skeleton both solvers
take. Same operator motion, same V10.1 model, same packet layout -- so the
difference between the two columns is the difference between the two solvers and
nothing else. No socket is opened and no hardware driver is loaded.

Agreement is not the goal and neither column is ground truth. The operator's own
joint angles are printed beside them: where the two disagree, that is the
referee. What this run is really for is the cheap check before hardware -- that
the ported solver produces finite, in-range angles at a usable rate on real
recorded motion.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir,
                 os.pardir)
)
sys.path.insert(0, os.path.join(REPO_ROOT, "adapters", "manus", "python"))
sys.path.insert(0, os.path.join(REPO_ROOT, "adapters", "pico", "src"))

from manus_teleop.casadi_hand.hands import human_bend_angle  # noqa: E402
from manus_teleop.casadi_retarget import CasadiHandRetargeter  # noqa: E402

# Packet slot <- the operator joint it should follow. Slots with no operator
# counterpart (abduction, the thumb root) are compared between solvers only.
TRACKED = {
    "index_mcp_pitch": ("index", "mcp"), "index_pip": ("index", "pip"),
    "index_dip": ("index", "dip"),
    "middle_mcp_pitch": ("middle", "mcp"), "middle_pip": ("middle", "pip"),
    "middle_dip": ("middle", "dip"),
    "ring_mcp_pitch": ("ring", "mcp"), "ring_pip": ("ring", "pip"),
    "ring_dip": ("ring", "dip"),
    "pinky_mcp_pitch": ("pinky", "mcp"), "pinky_pip": ("pinky", "pip"),
    "pinky_dip": ("pinky", "dip"),
    "thumb_mcp": ("thumb", "mcp"), "thumb_ip": ("thumb", "ip"),
}


def frames(path, stride, limit):
    with open(path) as fh:
        fh.readline()
        for i, line in enumerate(fh):
            if i % stride:
                continue
            if limit and len(_seen) >= limit:
                return
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            src = rec.get("source") or {}
            points = src.get("keypoints")
            if not points or len(points) != 25 or not rec.get("qpos"):
                continue
            frame = np.empty((25, 7), dtype=float)
            for n, point in enumerate(points):
                frame[n, :3] = point["position"]
                x, y, z, w = point["orientation_xyzw"]
                frame[n, 3:] = (w, x, y, z)
            if not np.isfinite(frame).all():
                continue
            _seen.append(i)
            yield frame, np.asarray(rec["qpos"], dtype=float), list(rec["joint_names"])


_seen: list[int] = []


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--profile", required=True)
    ap.add_argument("--side", default="left", choices=["left", "right"])
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--limit", type=int, default=400)
    args = ap.parse_args()

    rt = CasadiHandRetargeter(args.side, args.profile)
    print(f"  profile : {rt.profile['operator']}, recorded {rt.profile['recorded']}, "
          f"palm fit {rt.profile['palm_fit_mm'] * 1000:.1f} mm")

    ours, theirs, human, ms, failed, names = [], [], [], [], 0, None
    for frame, qpos, joint_names in frames(args.recording, args.stride, args.limit):
        names = names or joint_names
        if joint_names != names:
            raise SystemExit("packet joint order changes inside the recording")
        packet, stats = rt.retarget(frame)
        if not stats["success"]:
            failed += 1
        ours.append(packet)
        theirs.append(qpos)
        ms.append(stats["solve_ms"])
        human.append({k: np.degrees(human_bend_angle(frame[:, :3], *v))
                      for k, v in TRACKED.items()})

    if not ours:
        raise SystemExit("no comparable frames in that recording")
    ours, theirs = np.array(ours), np.array(theirs)
    if names != rt.joint_names:
        raise SystemExit(f"packet layout differs:\n  recorded {names}\n  ours "
                         f"{rt.joint_names}")

    print(f"  frames  : {len(ours)}   solve {np.mean(ms):.1f} ms mean, "
          f"{np.percentile(ms, 95):.1f} ms p95")
    finite = np.isfinite(ours).all()
    at_bound = int(np.sum((ours <= rt.lower + 1e-9) | (ours >= rt.upper - 1e-9)))
    print(f"  sanity  : all finite {finite}, non-converged {failed}/{len(ours)}, "
          f"{at_bound} joint-frames on a limit")

    print(f"\n  {'joint':<18}{'operator':>10}{'ours':>9}{'deployed':>10}"
          f"{'|diff|':>9}")
    for slot, name in enumerate(rt.joint_names):
        peak_o = np.degrees(ours[:, slot]).max()
        peak_t = np.degrees(theirs[:, slot]).max()
        diff = np.degrees(np.abs(ours[:, slot] - theirs[:, slot])).mean()
        if name in TRACKED:
            peak_h = f"{max(h[name] for h in human):>10.1f}"
        else:
            peak_h = f"{'-':>10}"
        print(f"  {name:<18}{peak_h}{peak_o:>9.1f}{peak_t:>10.1f}{diff:>9.1f}")

    def rms(column):
        errors = [np.degrees(column[:, rt.joint_names.index(k)]) - [h[k] for h in human]
                  for k in TRACKED]
        return float(np.sqrt(np.mean(np.array(errors) ** 2)))

    print(f"\n  peaks are per-joint maxima over the run, not one instant.")
    print(f"  mean |diff| over all slots : {np.degrees(np.abs(ours - theirs)).mean():.1f} deg")
    print(f"  RMS vs operator   ours {rms(ours):.1f} deg   deployed {rms(theirs):.1f} deg")


if __name__ == "__main__":
    main()
