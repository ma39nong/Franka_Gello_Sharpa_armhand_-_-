"""Adapter exposing the CasADi retargeter under this repo's hand contract.

The solver in ``casadi_hand/`` was written against the LinkerHand L20 V10.1 and
O30i outside this tree and is copied in unchanged. Everything specific to this
repository -- the packet layout, the ``thumb_ip`` alias, mimic expansion, and
where the calibration comes from -- lives here, so the solver stays a plain
port that can be re-synced.

Two things differ from ``L20Retargeter`` and matter at the call site:

*It needs the raw MANUS frame.* The canonical 21-landmark array drops the node
orientations, and four of this solver's twelve cost terms are orientation terms.
``wants_raw_keypoints`` tells the pipeline to pass the 25x7 frame instead.

*It needs a calibration profile.* Hand dimensions, joint ranges, and the
transform from the operator's wrist frame into the robot's base frame are all
measured, never assumed, and a term whose measurement is missing switches itself
off instead of running on a plausible constant. The profile is recorded once by
``scripts/make_hand_profile.py`` and named explicitly here; nothing is loaded
implicitly, because a stale profile that applies silently looks calibrated and
is not.
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from .casadi_hand.hands import get
from .casadi_hand.retargeter import Retargeter


class CasadiHandRetargeter:
    """CasADi optimization retargeter behind ``retarget(frame) -> (qpos, stats)``."""

    # The pipeline passes the raw 25x7 MANUS frame rather than canonical
    # landmarks when it sees this.
    wants_raw_keypoints = True

    def __init__(
        self,
        side: str,
        profile: str | Path,
        *,
        hand: str | None = None,
        packet_joint_names: tuple[str, ...] | None = None,
    ) -> None:
        if side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        self.side = side
        self.spec = get(hand or f"l20_{side}")
        self.rt = Retargeter(self.spec)

        profile = Path(profile)
        if not profile.is_file():
            raise FileNotFoundError(
                f"hand profile not found: {profile}. Record one with "
                f"scripts/make_hand_profile.py; this retargeter has no defaults.")
        self.profile = self.rt.load_profile(str(profile))

        # The UDP contract predates V10.1 and calls the coupled distal thumb
        # joint thumb_ip where the L20 URDF calls it thumb_dip -- the same
        # rename the pinocchio retargeter absorbs. Keep the external name and
        # slot order. Derived from the URDF rather than from the side: the
        # alias belongs to the L20, and the O30i has a real thumb_ip joint that
        # needs none. Keying it on `side == "left"` only worked while the left
        # hand was the only L20.
        self._alias = (
            {"thumb_dip": "thumb_ip"}
            if "thumb_dip" in self.rt.kin.joints
            else {}
        )
        if packet_joint_names is None:
            from pico_bimanual_franka_teleop.hand_retarget import (  # noqa: PLC0415
                LEFT_G20_PACKET_JOINT_NAMES,
            )
            packet_joint_names = LEFT_G20_PACKET_JOINT_NAMES
        self.joint_names = list(packet_joint_names)

        joints = self.rt.kin.joints
        # Every packet slot is either solved directly or derived from one that
        # is. Building the plan once, and failing here rather than per frame,
        # means an asset change cannot silently produce a wrong packet.
        solved = {self._alias.get(n, n): i
                  for i, n in enumerate(self.rt.kin.joint_names)}
        self._direct: list[tuple[int, int]] = []
        self._derived: list[tuple[int, str, float, float]] = []
        for slot, name in enumerate(self.joint_names):
            if name in solved:
                self._direct.append((slot, solved[name]))
                continue
            urdf_name = next(
                (u for u, a in self._alias.items() if a == name), name)
            joint = joints.get(urdf_name)
            if joint is None or joint.mimic is None:
                raise ValueError(
                    f"packet slot {name!r} is neither solved nor a mimic joint "
                    f"of {self.spec.urdf_path}")
            source, multiplier, offset = joint.mimic
            self._derived.append(
                (slot, self._alias.get(source, source), multiplier, offset))

        lower = np.full(len(self.joint_names), -np.inf)
        upper = np.full(len(self.joint_names), np.inf)
        for slot, index in self._direct:
            lower[slot], upper[slot] = self.rt.lo[index], self.rt.hi[index]
        for slot, source, multiplier, offset in self._derived:
            joint = joints[next(u for u, a in self._alias.items() if a == source)
                           if source in self._alias.values() else source]
            bounds = [multiplier * b + offset
                      for b in (self.rt.lo[solved[source]], self.rt.hi[solved[source]])]
            lower[slot], upper[slot] = min(bounds), max(bounds)
        self.lower, self.upper = lower, upper
        self.last_qpos = self._to_packet(self.rt.q_rest)
        self._solve_ms = 0.0

    @property
    def dof(self) -> int:
        return len(self.joint_names)

    def _to_packet(self, q: np.ndarray) -> np.ndarray:
        out = np.zeros(len(self.joint_names), dtype=np.float64)
        for slot, index in self._direct:
            out[slot] = q[index]
        by_name = {self.joint_names[slot]: out[slot] for slot, _ in self._direct}
        for slot, source, multiplier, offset in self._derived:
            out[slot] = multiplier * by_name[source] + offset
        return np.clip(out, self.lower, self.upper)

    def retarget(self, keypoints) -> tuple[np.ndarray, dict]:
        """Solve one frame. `keypoints` is the raw 25x7 MANUS skeleton."""
        frame = np.asarray(keypoints, dtype=np.float64)
        if frame.shape != (25, 7):
            raise ValueError(
                f"expected the raw 25x7 MANUS frame, got {frame.shape}. The "
                f"canonical 21-landmark array drops the orientations this "
                f"solver's orientation terms need.")
        started = time.perf_counter()
        q = self.rt.retarget(frame)
        self._solve_ms = (time.perf_counter() - started) * 1000.0

        packet = self._to_packet(q)
        self.last_qpos = self._to_packet(self.rt.q_prev)  # unsmoothed
        stats = self.rt.last_stats
        return packet, {
            "success": bool(stats.get("ok", False)),
            "loss": float(stats.get("cost", float("nan"))),
            "iterations": int(stats.get("iters", -1)),
            "solve_ms": self._solve_ms,
            "palm_fit_mm": float(stats.get("palm_fit_mm") or 0.0) * 1000.0,
            "fist_terms_enabled": bool(stats.get("fist_terms_enabled", False)),
        }

    def reset(self) -> None:
        self.rt.reset()
        self.last_qpos = self._to_packet(self.rt.q_rest)

    def close(self) -> None:
        return None
