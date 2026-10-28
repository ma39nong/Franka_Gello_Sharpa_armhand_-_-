#!/usr/bin/env python3
"""Derive a CasADi-retargeter operator profile from an existing MANUS recording.

    python adapters/manus/scripts/make_hand_profile.py \
        /home/descfly/franka_teleop_data/manus_accuracy/manus_six_pose_bimanual_20260730_215752.jsonl \
        --operator descfly --out config/calibration/hand_profiles/left_manus.json

The CasADi retargeter needs two operator poses -- hand flat and open, and a
tight fist -- before it will run. It has no defaults: a term whose calibration
is missing switches itself off rather than run on a plausible constant.

Nothing here asks the operator to do anything. Recorded MANUS frames already
carry the raw 25-node skeleton with orientations under ``source.keypoints``, and
the flat and fist poses are already in the six-pose captures, so the profile is
recovered from data that exists. The frames are scored and the best ones picked;
neighbours are kept as repeats so the spread across them is measurable, which is
the only honest error bar on a calibration.

The profile is bound to the operator and glove fit of that recording. Re-derive
it if the glove is re-seated or somebody else drives the hand.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from datetime import datetime, timezone

import numpy as np

REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir,
                 os.pardir)
)
sys.path.insert(0, os.path.join(REPO_ROOT, "adapters", "manus", "python"))

from manus_teleop.casadi_hand.hands import (  # noqa: E402
    FINGERS, HUMAN_CHAINS, get, human_bend_angle,
)
from manus_teleop.casadi_hand.retargeter import (  # noqa: E402
    PROFILE_FORMAT, PROFILE_LAYOUT, Retargeter,
)

BENDS = [(f, w) for f in FINGERS
         for w in (("mcp", "ip") if f == "thumb" else ("mcp", "pip", "dip"))]


def read_frames(path, stride):
    """Recorded frames as (index, 25x7) with quaternions in w,x,y,z order."""
    out = []
    with open(path) as fh:
        fh.readline()  # header line
        for i, line in enumerate(fh):
            if i % stride:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            src = rec.get("source") or {}
            points = src.get("keypoints")
            if not points or len(points) != 25:
                continue
            frame = np.empty((25, 7), dtype=float)
            for n, point in enumerate(points):
                frame[n, :3] = point["position"]
                x, y, z, w = point["orientation_xyzw"]
                frame[n, 3:] = (w, x, y, z)  # the retargeter reads w,x,y,z
            if np.isfinite(frame).all():
                out.append((i, frame))
    return out


def extension(points):
    """Chord over arc along each finger. 1.0 is perfectly straight."""
    out = {}
    for finger, chain in HUMAN_CHAINS.items():
        arc = sum(float(np.linalg.norm(points[b] - points[a]))
                  for a, b in zip(chain[:-1], chain[1:]))
        out[finger] = (float(np.linalg.norm(points[chain[-1]] - points[chain[0]]))
                       / arc) if arc > 1e-9 else 0.0
    return out


def total_flexion(points):
    return sum(human_bend_angle(points, f, w) for f, w in BENDS)


def pick(frames, score, count, keep):
    """The best frame from each of the `count` best separate holds of a pose.

    Repeats are only informative if they are separate observations, so the
    candidates are grouped into runs of consecutive qualifying frames -- one
    hold each -- and the best frame of each run is taken. Frames that do not
    qualify are never picked: taking neighbours at a fixed time offset instead
    pulls in whatever the hand was doing on the way out of the pose, which
    reads as a huge spread between repeats and hides the real one.
    """
    scored = [(i, score(frame)) for i, (_, frame) in enumerate(frames)]
    good = [i for i, s in scored if keep(s)]
    if not good:
        raise SystemExit(
            "no frame in this recording holds the pose well enough; use a "
            "recording that contains it, or record one")
    runs, run = [], [good[0]]
    for i in good[1:]:
        if i == run[-1] + 1:
            run.append(i)
        else:
            runs.append(run)
            run = [i]
    runs.append(run)
    runs.sort(key=lambda r: -max(scored[i][1] for i in r))
    chosen = [max(r, key=lambda i: scored[i][1]) for r in runs[:count]]
    return [frames[i] for i in sorted(chosen)], len(runs)


def rank_by_pinch(candidates, spec, fist_frame, trajectory):
    """Order flat holds by how well the pinch they produce tracks the operator's.

    A flat pose fixes one rigid transform, and its residual says only that the
    fit closed on something. It says nothing about the rotation about the
    knuckle line -- the four MCPs are coplanar, so that angle is barely
    constrained, and it is the one that decides where the thumb ends up
    relative to the fingers.

    Measured over nine flat holds of one operator: the residual and the pinch
    error are uncorrelated (r = +0.10), the hold with the *lowest* residual gave
    the *worst* pinch of the nine (51.8 mm RMS against 31.6 mm for the best),
    and finger flexion barely moved at all (3.1 deg across the same nine). So
    the calibration is ranked on the thing it actually decides, scored against
    the operator's own thumb-to-finger distances on a trajectory that contains
    pinching.
    """
    from manus_teleop.casadi_hand.hands import HUMAN_TIP  # noqa: PLC0415

    human = {f: np.array([np.linalg.norm(k[HUMAN_TIP[f], :3] - k[HUMAN_TIP["thumb"], :3])
                          for k in trajectory]) for f in FINGERS[1:]}
    ranked = []
    for index, frame in candidates:
        rt = Retargeter(spec)
        rt.calibrate(frame)
        rt.calibrate_fist(fist_frame)
        robot = {f: [] for f in FINGERS[1:]}
        for k in trajectory:
            rt.retarget(k)
            tips = rt.fingertips(rt.q_prev)
            for slot, f in enumerate(FINGERS[1:], start=1):
                robot[f].append(float(np.linalg.norm(tips[slot] - tips[0])))
        error = np.concatenate([np.asarray(robot[f]) - human[f] * rt.scale_gap[f]
                                for f in FINGERS[1:]])
        ranked.append((float(np.sqrt(np.mean(error ** 2))), index, frame))
    ranked.sort(key=lambda r: r[0])
    return ranked


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", nargs="+",
                    help="hand-retarget-debug JSONL files with source.keypoints. "
                         "Several are better than one: repeats of a pose are only "
                         "informative if they are separate observations, and a "
                         "single capture often holds each pose exactly once.")
    ap.add_argument("--operator", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--hand", default="l20_left")
    ap.add_argument("--repeats", type=int, default=3,
                    help="flat holds kept in the profile, best first")
    ap.add_argument("--candidates", type=int, default=12,
                    help="flat holds scored before ranking")
    ap.add_argument("--stride", type=int, default=5)
    args = ap.parse_args()

    frames = []
    for path in args.recording:
        # Offset the indices per file so runs never merge across a file
        # boundary and report two captures as one hold.
        start = (len(frames) + 1000) * 10
        got = read_frames(path, args.stride)
        frames.extend((start + i, f) for i, f in got)
        print(f"  {len(got):>5} frames  {os.path.basename(path)}")
    if len(frames) < args.repeats * 4:
        raise SystemExit(f"only {len(frames)} usable frames")

    # Flat qualifies at the same 90% bar calibrate() itself enforces. Fist has
    # no absolute bar -- how far an operator closes is what is being measured --
    # so it is taken relative to the deepest flexion seen in this recording.
    candidates, flat_runs = pick(
        frames, lambda p: min(extension(p[:, :3]).values()), args.candidates,
        keep=lambda s: s >= 0.90)
    deepest = max(total_flexion(f[:, :3]) for _, f in frames)
    fist, fist_runs = pick(
        frames, lambda p: total_flexion(p[:, :3]), args.repeats,
        keep=lambda s: s >= 0.95 * deepest)
    print(f"  {flat_runs} separate flat holds, {fist_runs} fist holds; "
          f"scoring the best {len(candidates)} flat")

    # Score every candidate on a trajectory that actually contains pinching,
    # subsampled -- this is a full solve per frame per candidate.
    trajectory = [f for _, f in frames[::max(1, len(frames) // 120)]]
    ranked = rank_by_pinch(candidates, get(args.hand), fist[0][1], trajectory)
    print(f"\n  flat holds ranked by pinch tracking on {len(trajectory)} frames")
    print(f"  {'frame':>10}{'pinch RMS':>12}{'palm fit':>11}")
    scratch = Retargeter(get(args.hand))
    for rms, index, frame in ranked:
        mm = scratch.calibrate(frame)["palm_fit_mm"] * 1000
        print(f"  {index:>10}{rms * 1000:>10.1f}mm{mm:>9.1f}mm")
    flat = [(index, frame) for _, index, frame in ranked[:args.repeats]]

    rt = Retargeter(get(args.hand))
    print(f"\n  flat pose, scored against {args.hand}")
    print(f"  {'frame':>8}{'palm fit':>11}{'flat?':>8}   least extended")
    fits = []
    for index, frame in flat:
        cal = rt.calibrate(frame)
        worst = min(cal["extension"], key=cal["extension"].get)
        fits.append((rt.align_R.copy(), rt.wrist_offset.copy()))
        print(f"  {index:>8}{cal['palm_fit_mm'] * 1000:>9.1f}mm"
              f"{'yes' if cal['flat'] else 'NO':>8}   {worst} "
              f"{cal['extension'][worst] * 100:.0f}%")

    def rot_angle(a, b):
        c = (np.trace(a.T @ b) - 1.0) / 2.0
        return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))

    spread_deg = max(rot_angle(a[0], b[0]) for a in fits for b in fits)
    spread_mm = max(float(np.linalg.norm(a[1] - b[1])) * 1000.0
                    for a in fits for b in fits)
    print(f"  spread across repeats: alignment {spread_deg:.1f} deg, "
          f"virtual wrist {spread_mm:.1f} mm")
    if spread_deg > 5.0:
        print("  -> large. These repeats do not agree on the palm orientation; "
              "the profile will not\n     transfer between sessions.")

    print("\n  fist pose, operator joint angles (deg)")
    fulls = [rt.calibrate_fist(frame) for _, frame in fist]
    print(f"  {'joint':<14}" + "".join(f"{'#' + str(i):>8}"
                                       for i, _ in enumerate(fist)) + f"{'used':>8}")
    used = {}
    for key in fulls[0]:
        values = [np.degrees(f[key]) for f in fulls]
        used[key] = max(values)
        print(f"  {key[0] + '.' + key[1]:<14}"
              + "".join(f"{v:>8.0f}" for v in values) + f"{used[key]:>8.0f}")
    dead = [k for k, v in used.items() if v < 10.0]
    if dead:
        print("\n  never moved: " + ", ".join(f"{f}.{w}" for f, w in dead)
              + "\n     Terms depending on these disable themselves; no retargeting "
                "can recover\n     motion the glove does not report.")

    profile = {
        "format": PROFILE_FORMAT,
        "version": 1,
        "operator": args.operator,
        "recorded": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "keypoint_layout": PROFILE_LAYOUT,
        "side": "left" if args.hand.endswith("left") else "right",
        "source_recordings": [os.path.abspath(p) for p in args.recording],
        "source_frames": {"flat": [i for i, _ in flat], "fist": [i for i, _ in fist]},
        "flat_frames": [f.tolist() for _, f in flat],
        "fist_frames": [f.tolist() for _, f in fist],
        "ranked_by": "pinch-tracking-rms",
        "measured_on": {"hand": args.hand, "align_deg_max": spread_deg,
                        "wrist_mm_max": spread_mm,
                        "pinch_rms_mm": [r[0] * 1000 for r in ranked[:args.repeats]]},
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as fh:
        json.dump(profile, fh, indent=1)
    print(f"\n  wrote {args.out}")


if __name__ == "__main__":
    main()
