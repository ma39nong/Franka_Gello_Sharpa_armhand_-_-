"""Stream a (N, 20) tick trajectory to the hand and log commanded vs reported.

    python3 replay_ticks.py --device 1 --npy replay_ticks.npy --rate 30

THIS MOVES THE HAND continuously for N/rate seconds. Columns must be in URDF
joint order (thumb, index, middle, ring, pinky x roll/pitch/pip/dip); the
reorder to the vendor's driver order happens here by name, like hold_pose.py.

Every frame: send set_target_position, read get_current_position. The log
(commanded and reported per frame, wall clock) is written next to the input
as <input>.log.npz for offline analysis. At the end prints per-joint tracking
stats, comparing each reported frame against the command LAG frames earlier
(the hand cannot be at the target the same instant it is commanded), plus
first-third vs last-third error to expose degradation (thermal yaw dropout).
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from linker_hand_ros2_sdk.LinkerHand import o30i_control  # noqa: E402
from linker_hand_ros2_sdk.o30i_contract import (  # noqa: E402
    O30I_DRIVER_TO_URDF, O30I_RIGHT_LOWER, O30I_RIGHT_UPPER,
    O30I_URDF_JOINT_NAMES)
from linker_hand_ros2_sdk.o30i_transport import (  # noqa: E402
    bundled_libcanbus_path, make_libcanbus_communication)

o30i_control.CANFDCommunication = make_libcanbus_communication(
    o30i_control, bundled_libcanbus_path())


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", type=int, default=1)
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--npy", required=True)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--lag", type=int, default=3,
                        help="frames of allowed tracking lag in the stats")
    args = parser.parse_args()

    traj = np.load(args.npy)
    if traj.ndim != 2 or traj.shape[1] != len(O30I_URDF_JOINT_NAMES):
        raise SystemExit(f"{args.npy}: expected (N, 20), got {traj.shape}")
    traj = traj.astype(int)

    driver_names = None
    hand = o30i_control.LinkerHandO30IController(
        hand_type="right", canfd_device=args.device, channel=args.channel,
        comm_type="libcanbus")
    if not hand.is_connected:
        print("no transport; is the bridge still holding the bus?")
        return 1
    model = str(hand.get_product_model() or "")
    if "O30" not in model.upper():
        print(f"index {args.device} is not an O30i ({model!r})")
        hand.close()
        return 1
    try:
        hand.setup()
        time.sleep(0.2)
        driver_names = list(o30i_control.JOINT_NAMES)
        urdf_index = {n: i for i, n in enumerate(O30I_URDF_JOINT_NAMES)}
        to_driver = [urdf_index[O30I_DRIVER_TO_URDF[d]] for d in driver_names]

        # ease to the first frame, then stream
        hand.set_target_position([int(traj[0][j]) for j in to_driver])
        time.sleep(1.5)

        period = 1.0 / args.rate
        n = len(traj)
        reported = np.full((n, len(O30I_URDF_JOINT_NAMES)), -1, dtype=int)
        stamps = np.zeros(n)
        t0 = time.monotonic()
        misses = 0
        for k in range(n):
            target = t0 + k * period
            now = time.monotonic()
            if now < target:
                time.sleep(target - now)
            hand.set_target_position([int(traj[k][j]) for j in to_driver])
            pos = hand.get_current_position()
            stamps[k] = time.monotonic() - t0
            if pos is None:
                misses += 1
            else:
                for d, v in zip(driver_names, pos):
                    reported[k][urdf_index[O30I_DRIVER_TO_URDF[d]]] = v
        wall = time.monotonic() - t0

        out = args.npy + ".log.npz"
        np.savez(out, commanded=traj, reported=reported, stamps=stamps)
        print(f"\n{n} frames in {wall:.1f} s "
              f"({n / wall:.1f} Hz achieved, {misses} missed reads)")
        print(f"log: {out}")

        lag = args.lag
        cmd = traj[: n - lag]
        rep = reported[lag:]
        ok = (rep >= 0).all(axis=1)
        cmd, rep = cmd[ok[: len(cmd)]], rep[ok[: len(cmd)]]
        err = np.abs(rep - cmd)
        third = len(err) // 3
        print(f"\n  {'joint':<20}{'mean':>7}{'p95':>6}{'max':>6}"
              f"{'mean(1st third)':>17}{'mean(last third)':>18}")
        for j, name in enumerate(O30I_URDF_JOINT_NAMES):
            print(f"  {name:<20}{err[:, j].mean():7.1f}"
                  f"{np.percentile(err[:, j], 95):6.0f}{err[:, j].max():6.0f}"
                  f"{err[:third, j].mean():17.1f}{err[-third:, j].mean():18.1f}")
        return 0
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130
    finally:
        try:
            # neutral (URDF 0 deg), not all-zero: yaw tick 0 is the lateral
            # extreme and the index's extreme presses the pinched cable.
            # Computed from the contract so limit updates cannot go stale.
            neutral = {
                name: round((min(max(0.0, lo), hi) - lo) / (hi - lo) * 255)
                for name, lo, hi in zip(
                    O30I_URDF_JOINT_NAMES, O30I_RIGHT_LOWER, O30I_RIGHT_UPPER)}
            hand.set_target_position(
                [neutral[O30I_DRIVER_TO_URDF[d]]
                 for d in (driver_names or list(o30i_control.JOINT_NAMES))])
            time.sleep(1.0)
        finally:
            hand.close()


if __name__ == "__main__":
    sys.exit(main())
