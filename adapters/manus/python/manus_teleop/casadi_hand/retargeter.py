"""Hand retargeting: human keypoints -> robot joint angles.

One bound-constrained NLP per frame (IPOPT), warm-started from the previous
solution and its bound multipliers.

Derived from the SharpaWave optimiser, which ships as a Cython `.so` and is
licensed to this project for use, reproduction and modification -- but not for
disclosure. **Do not publish this repository.**

The cost function is a term-by-term port of their objective, which
`vendor_ref/verify_objective.py` reconstructs from the `.so` to machine
precision; `vendor_ref/OBJECTIVE.md` is the human-readable spec. Every weight
below is theirs, every residual keeps their form (norms and absolute values,
not squared sums), and the gates and contact-snap remaps are copied exactly.
There are no fitted constants; the config weights are user multipliers that
default to 1.0.

What could not transfer verbatim is replaced deliberately, and each
replacement is marked ``PORT NOTE`` at the point of use:

- Their hand is human-shaped, so their targets are raw keypoints. Ours are
  not, so every operator quantity passes through the calibration in
  `calibrate()`: directions through the palm-fit rotation, distances through
  per-pair rest-pose scales. Gates and snaps stay in operator space (they
  describe the operator's fingers, e.g. "closer than 10 mm means touching"),
  magnitudes execute in robot space.
- Their thumb terms assume their thumb frame; the LinkerHand thumbs point
  73-94 deg away. The thumb *weights* transfer (they are per-term and
  asymmetric: tip position x0.2315, orientation x2.0, direction x0.8); the
  thumb *frames* go through the rest-pose realignments from calibration.
- Their soft joint-range penalties are human-anatomy numbers and are not
  ported (HANDOVER.md); these hands keep hard bounds. The joint-coupling cone
  ("abduction room shrinks with flexion") is ported as a shape, with this
  robot's own limits in place of their human constants.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass

import casadi as ca
import numpy as np

from .hand_kinematics import HandKinematics
from .hands import (FINGERS, HUMAN_CHAINS, HUMAN_DIP, HUMAN_MCP, HUMAN_PIP,
                   HUMAN_TIP, HUMAN_WRIST, HandSpec, human_bend_angle,
                   infer_side, mirror_keypoints)

EPS = 1e-9


# ---------------------------------------------------------------- oracle spec
#
# Constants recovered from the oracle's symbolic cost and proven exact by
# vendor_ref/verify_objective.py. Do not tune any of these: their provenance is
# the point. Names follow their OptimizationResult fields.

ORACLE_W = {           # outer weights on each term in their total
    "tip_pos": 0.2,
    "tip_ori": 0.5,
    "finger_ori": 1.0,
    "pinch": 5.0,
    "pinch_ori": 0.1,
    "dq": 5.0,
    "thumb_ang": 0.5,   # their unreported thumb-IP angle map
    "dip_ori": 0.5,
    "pip_pinch": 5.0,
    "fist": 1.0,
    "pip_gap": 5.0,
    "coupling": 0.5,
    "pad_ori": 0.5,    # tip_ori's weight, which it stands in for during a pinch
}

# Inner (per-finger) weights, all theirs.
W_TIP_POS_THUMB = 0.23147521650098235   # thumb tip position, ungated
W_TIP_ORI_THUMB = 2.0                   # thumb fingertip orientation (quat distance)
W_DIR_THUMB = 0.8                       # thumb share of the direction cubic
W_PINCH = {"index": 1.0, "middle": 1.0, "ring": 6.0, "pinky": 8.0}
W_PINCH_ORI = {"index": 1.0, "middle": 1.0, "ring": 0.1, "pinky": 0.1}
W_GAP_PAIR = (1.0, 0.5, 0.5, 0.5)       # thumb-index spacing counts double
W_PIP_PINCH_GATE = 0.5   # their gate is tanh(0.2*(d-0.03)): frozen at 0.5 over
                         # any real hand distance, so the term is always on.
                         # Ported as the constant it behaves as.

# Gates and the contact snap, operator-space metres.
GATE_K, GATE_R = 20.0, 0.03             # the pinch gate
GAP_GATE_K = 40.0                       # the sharper adjacent-spacing gate
SNAP_LO, SNAP_HI = 0.01, 0.03           # below LO the target is contact, exactly
RAMP_K, RAMP_R = 500.0, 0.001           # pinch_ori validity ramp at 1 mm
# The pinch gate saturates at 0.5*(1+tanh(20*0.03)) = 0.769, so the reversed
# gate never falls below 0.231 -- on the oracle's hand a harmless floor, on
# these hands the 23% residual of three faded terms together outbids the
# pinch and hovers the O30i 15 mm short of contact. The fade family therefore
# rescales the reversed gate so OPERATOR CONTACT MEANS FULLY YIELDED:
# fade = (revgate - floor) / (1 - floor), clipped at zero. Constants are the
# gate's own.
GATE_CEIL = 0.5 * (1.0 + np.tanh(GATE_K * GATE_R))
REV_FLOOR = 1.0 - GATE_CEIL


def _fade(revgate):
    return np.clip((revgate - REV_FLOOR) / (1.0 - REV_FLOOR), 0.0, 1.0)

# The fist term works in "remaining extension": their human hand budgets
# 3*pi/2 across MCP+PIP+DIP. The human-side thresholds transfer (they describe
# the operator); the robot side uses its own flexion budget (PORT NOTE below).
FIST_BUDGET_H = 1.5 * np.pi
FIST_SNAP_LO, FIST_SNAP_HI = np.radians(60.0), np.radians(100.0)

# Coupling cone slope: abduction allowance shrinks by 0.22 rad per rad of
# flexion. The slope is theirs; the intercept is this robot's own AA limit.
COUPLE_SLOPE = 0.22
COUPLE_AMP, COUPLE_K = 0.05, 20.0

# The oracle measures each digit's pointing direction from the MCP knuckle.
# Their thumb root is node 1 (node 2 is geometrically degenerate -- see
# hands.HUMAN_BEND_NODE).
DIR_ROOT = {"thumb": 1, **{f: HUMAN_MCP[f] for f in FINGERS[1:]}}

# Adjacent-digit spacing pairs at PIP level, their pip_gap: human keypoint
# nodes, thumb IP node against the finger PIP nodes.
GAP_NODES = [(HUMAN_DIP["thumb"], HUMAN_PIP["index"]),
             (HUMAN_PIP["index"], HUMAN_PIP["middle"]),
             (HUMAN_PIP["middle"], HUMAN_PIP["ring"]),
             (HUMAN_PIP["ring"], HUMAN_PIP["pinky"])]


def _gate(d, k=GATE_K, r=GATE_R):
    return 0.5 * (1.0 - np.tanh(k * (d - r)))


def _snap(d):
    """Their contact remap: identity when far, compressed inside 30 mm,
    exactly zero (touching) below 10 mm."""
    if d < SNAP_LO:
        return 0.0
    if d < SNAP_HI:
        return SNAP_HI * (d - SNAP_LO) / (SNAP_HI - SNAP_LO)
    return float(d)


def _fist_snap(theta):
    """Same shape in angle space: below 60 deg of remaining extension the
    target is a full fist."""
    if theta < FIST_SNAP_LO:
        return 0.0
    if theta < FIST_SNAP_HI:
        return FIST_SNAP_HI * (theta - FIST_SNAP_LO) / (FIST_SNAP_HI - FIST_SNAP_LO)
    return float(theta)


@dataclass
class RetargetConfig:
    """Per-term user multipliers over the oracle weights. All 1.0: the actual
    weights are the ORACLE_W constants, which are measurements, not choices."""

    w_tip_pos: float = 1.0
    w_tip_ori: float = 1.0
    w_finger_ori: float = 1.0
    w_pinch: float = 1.0
    w_pinch_ori: float = 1.0
    w_dq: float = 1.0
    w_thumb_ang: float = 1.0
    w_dip_ori: float = 1.0
    w_pip_pinch: float = 1.0
    w_fist: float = 1.0
    w_pip_gap: float = 1.0
    w_coupling: float = 1.0
    w_pad_ori: float = 1.0

    output_tau: float = 0.02     # time constant of the low-pass on the returned
                                 # command, in SECONDS -- not a per-call factor.
                                 #
                                 # A fixed per-call alpha makes the smoothing
                                 # depend on however fast the caller happens to
                                 # loop: alpha=0.25 settles in 52 ms at 200 Hz and
                                 # 174 ms at 60 Hz, so moving the same code from
                                 # the web viewer to the Qt one tripled the lag
                                 # without any of the smoothing changing. Deriving
                                 # alpha from the measured interval keeps the
                                 # response the same at any rate.
    max_iter: int = 100
    tol: float = 1e-4


# Operator profiles written by viz.py's side panel. Validated on load rather
# than trusted: link names repeat across hands, and a profile recorded
# against the wrong keypoint layout would silently mis-map every finger.
PROFILE_FORMAT = "linkerhand-retargeting/operator-profile"
PROFILE_LAYOUT = "manus-raw-25"

WEIGHT_KEYS = ("w_tip_pos", "w_tip_ori", "w_finger_ori", "w_pinch", "w_pinch_ori",
               "w_dq", "w_thumb_ang", "w_dip_ori", "w_pip_pinch", "w_fist",
               "w_pip_gap", "w_coupling", "w_pad_ori")


def _unit(v):
    if isinstance(v, (ca.MX, ca.SX, ca.DM)):
        return v / ca.sqrt(ca.sumsqr(v) + EPS)
    v = np.asarray(v, dtype=float)
    return v / (np.linalg.norm(v) + EPS)


def _umeyama(A: np.ndarray, B: np.ndarray):
    """Similarity transform (R, t, s) minimising |s*R@a + t - b| over point pairs."""
    mA, mB = A.mean(0), B.mean(0)
    A0, B0 = A - mA, B - mB
    U, S, Vt = np.linalg.svd(A0.T @ B0)
    d = float(np.sign(np.linalg.det(Vt.T @ U.T)))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    var = float((A0 ** 2).sum())
    s = float((S * np.array([1.0, 1.0, d])).sum() / var) if var > EPS else 1.0
    return R, mB - s * R @ mA, s


def _quat_to_matrix(q):
    w, x, y, z = q / (np.linalg.norm(q) + EPS)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class Retargeter:
    def __init__(self, spec: HandSpec, config: RetargetConfig | None = None):
        self.spec = spec
        self.cfg = config or RetargetConfig()
        self.kin: HandKinematics = spec.build()
        self.n = self.kin.n_dof

        # Solved by calibrate(), unless the spec pins it down. The origin and the
        # orientation come out of the same palm fit: they are the translation and
        # the rotation of one rigid transform from the operator's wrist frame into
        # the robot's base frame, not two independent quantities.
        self.wrist_is_annotated = spec.wrist_offset is not None
        self.wrist_offset = (np.asarray(spec.wrist_offset, dtype=float)
                             if self.wrist_is_annotated else np.zeros(3))
        self.align_R = np.eye(3)
        self.thumb_R = np.eye(3)

        self.lo, self.hi = self.kin.joint_bounds()
        self.q_rest = self.kin.rest_pose()

        self._measure_robot()
        self.calibration = None   # calibrate(open_hand)
        self.profile = None       # load_profile(path), explicit only
        self.human_full = None    # calibrate_fist(full_fist), flexion_guess only
        self._build_solver()

        self.q_prev = self.q_rest.copy()
        self.q_prev2 = self.q_rest.copy()
        self.q_filtered = self.q_rest.copy()
        self._lam_x0 = None
        self._last_call = None
        self._fk_tips = None
        self._q_oppose = None
        self.last_stats: dict = {}

    # ----------------------------------------------------------------- robot

    def _measure_robot(self):
        q0 = self.q_rest
        targets = ([self.spec.tip_frames[f] for f in FINGERS]
                   + [(self.spec.mcp_links[f], np.zeros(3)) for f in FINGERS]
                   + [(self.spec.distal_links[f], np.zeros(3)) for f in FINGERS]
                   + [(self.spec.pip_links[f], np.zeros(3)) for f in FINGERS])
        pts = np.array(self.kin.make_fk_function(targets)(q0))
        self.rest_tip = {f: pts[i] for i, f in enumerate(FINGERS)}
        self.rest_mcp = {f: pts[5 + i] for i, f in enumerate(FINGERS)}
        self.rest_dip = {f: pts[10 + i] for i, f in enumerate(FINGERS)}
        self.rest_pip = {f: pts[15 + i] for i, f in enumerate(FINGERS)}

        self.robot_finger_len = {
            f: float(np.linalg.norm(self.rest_tip[f] - self.rest_mcp[f])) for f in FINGERS
        }
        self.robot_tip_gap = {
            f: float(np.linalg.norm(self.rest_tip[f] - self.rest_tip["thumb"]))
            for f in FINGERS[1:]
        }
        # Opposition and spacing pairs at PIP level, matching the oracle's
        # links: thumbtip<->index PIP for pip_pinch; thumb DIP against index
        # PIP and then adjacent PIPs for pip_gap.
        self.rest_pp_gap = float(np.linalg.norm(self.rest_pip["index"]
                                                - self.rest_tip["thumb"]))
        gap_chain = [self.rest_dip["thumb"]] + [self.rest_pip[f] for f in FINGERS[1:]]
        self.rest_gap_pair = [float(np.linalg.norm(b - a))
                              for a, b in zip(gap_chain[:-1], gap_chain[1:])]
        # Rest orientation of each tip frame's link, for the roll reference.
        self.rest_robot_R = {
            f: np.array(ca.DM(self.kin.link_pose(self.spec.tip_frames[f][0],
                                                 ca.DM(q0))[0]))
            for f in FINGERS
        }
        # Robot palm frame, cached: across the palm and along it, from the
        # rigid rest geometry. The per-frame alignment below maps the
        # operator's palm frame onto this one.
        mcpc = np.mean([self.rest_mcp[f] for f in FINGERS[1:]], axis=0)
        aw = mcpc - (self.rest_tip["middle"] - mcpc)
        x = self.rest_mcp["index"] - self.rest_mcp["pinky"]
        x = x / (np.linalg.norm(x) + EPS)
        z = np.cross(x, self.rest_mcp["middle"] - aw)
        z = z / (np.linalg.norm(z) + EPS)
        self._palm_frame_robot = np.stack([x, np.cross(z, x), z], axis=1)

        # PORT NOTE (fist): their robot side is 3*pi/2 minus the summed flexion,
        # an identity map because their hand's ranges are human. Here the robot
        # budgets its own flexion span, and the human target is scaled by
        # span/budget_h so open maps to open and a full fist to a full fist.
        self._flex_by_finger = {
            f: [self.kin.index_of[j] for j in self.spec.flexion_joints
                if f in j and j in self.kin.index_of]
            for f in FINGERS[1:]
        }
        # On the anthropomorphic path the robot's budget IS the human budget
        # (their identity map); measuring (hi - lo) there would inflate the
        # target by the below-zero extension range.
        self.flex_budget = {
            f: (FIST_BUDGET_H if self.spec.anthropomorphic
                else sum(float(self.hi[i] - self.lo[i]) for i in idx) or 1.0)
            for f, idx in self._flex_by_finger.items()
        }

    # ----------------------------------------------------------- calibration

    def load_profile(self, path: str) -> dict:
        """Apply an operator profile recorded from viz.py's side panel.

        Always explicit -- nothing loads a profile on its own. A stale profile
        that silently applies is worse than no profile, because the numbers look
        calibrated and are not.

        The profile stores the raw capture, not the derived transform, so the fit
        is recomputed here for *this* hand. The flat frames are stored in the
        order the recording tool ranked them and the first is used; the fist is
        the per-joint deepest flexion across repeats, which is what "how far can
        this operator close" means.

        The loader deliberately does NOT re-rank the flat frames by palm-fit
        residual. Measured across nine flat holds of one operator, the residual
        does not predict retargeting quality (r = +0.10) and picking by it
        selected the worst calibration of the nine for pinch tracking -- 49.6 mm
        RMS where the best was 31.6 mm. A low residual only says the fit closed
        on something; it says nothing about the rotation about the knuckle line,
        which is the part that is barely constrained and the part that matters.
        Ranking needs a trajectory to score against, which the tool has and this
        does not, so the tool's order is respected.
        """
        with open(path) as fh:
            prof = json.load(fh)
        if prof.get("format") != PROFILE_FORMAT:
            raise ValueError(f"{path}: not an operator profile "
                             f"(format={prof.get('format')!r})")
        if prof.get("keypoint_layout") != PROFILE_LAYOUT:
            raise ValueError(f"{path}: keypoint layout {prof.get('keypoint_layout')!r}, "
                             f"expected {PROFILE_LAYOUT!r}")
        flat = [np.asarray(f, dtype=float) for f in prof["flat_frames"]]
        fist = [np.asarray(f, dtype=float) for f in prof["fist_frames"]]
        if not flat or not fist:
            raise ValueError(f"{path}: profile has no captured frames")
        for f in flat + fist:
            if f.shape != (25, 7):
                raise ValueError(f"{path}: frame shape {f.shape}, expected (25, 7)")

        # Sidedness is data, and left<->right has one exact conversion, so a
        # profile of either hand serves either robot hand: mirror on load when
        # the sides differ. Files record their side; older ones get it read
        # off the geometry (hands.infer_side).
        prof_side = prof.get("side") or infer_side(flat[0])
        if prof_side != self.spec.side:
            flat = [mirror_keypoints(f) for f in flat]
            fist = [mirror_keypoints(f) for f in fist]
            print(f"  profile is a {prof_side}-hand capture; mirrored for "
                  f"{self.spec.name}")

        cal = self.calibrate(flat[0])

        fulls = [self.calibrate_fist(kp) for kp in fist]
        self.human_full = {k: max(f[k] for f in fulls) for k in fulls[0]}

        self.profile = {
            "path": path,
            "operator": prof.get("operator", "?"),
            "recorded": prof.get("recorded", "?"),
            "repeats": len(flat),
            "ranked_by": prof.get("ranked_by", "unranked"),
            "palm_fit_mm": cal["palm_fit_mm"],
        }
        return self.profile

    def calibrate(self, open_hand: np.ndarray) -> dict:
        """Anchor on one frame of the operator holding a flat, open hand.

        Everything the cost function knows about the operator comes from here:
        finger lengths, reach, fingertip separations, and the fixed rotation
        between the operator's node frames and the robot's link frames.

        PORT NOTE: the oracle has no calibration at all -- its hand is
        human-shaped, so raw keypoints are already targets. This step is the
        deliberate replacement for that assumption: it constructs the maps that
        make an operator quantity mean the same thing on this robot.
        """
        kp = np.asarray(open_hand, dtype=float)
        p, quat = kp[:, :3], kp[:, 3:7]
        wrist = p[HUMAN_WRIST]

        # Chirality guard. The palm-fit residual does NOT catch a left capture
        # applied to a right hand or vice versa: the fitted points (coplanar
        # MCPs plus length-normalised tips) are near mirror-symmetric, so the
        # wrong side still closes at ~12 mm -- while every target lands 40-150
        # mm from the fingers (measured). Compare which side of the palm the
        # thumb is on instead: cross(across-palm, along-palm) . (thumb - wrist)
        # flips sign between hands.
        def handedness(w, mcp_i, mcp_p, mcp_m, thumb):
            n = np.cross(mcp_i - mcp_p, mcp_m - w)
            return float(np.sign(np.dot(n, thumb - w)))
        h_human = handedness(wrist, p[HUMAN_MCP["index"]], p[HUMAN_MCP["pinky"]],
                             p[HUMAN_MCP["middle"]], p[HUMAN_TIP["thumb"]])
        mcp_centroid = np.mean([self.rest_mcp[f] for f in FINGERS[1:]], axis=0)
        approx_wrist = mcp_centroid - (self.rest_tip["middle"] - mcp_centroid)
        h_robot = handedness(approx_wrist, self.rest_mcp["index"], self.rest_mcp["pinky"],
                             self.rest_mcp["middle"], self.rest_tip["thumb"])
        if h_human != 0 and h_robot != 0 and h_human != h_robot:
            raise ValueError(
                f"calibration capture is the WRONG HAND for {self.spec.name}: "
                "the thumb sits on the opposite side of the palm. Loaders "
                "normalise sidedness (load_profile, ReplaySource), so reaching "
                "this means raw frames of the wrong hand were passed directly "
                "-- convert with hands.mirror_keypoints().")

        cal = {"finger_len": {}, "wrist_to_tip": {}, "tip_gap": {},
               "extension": {}}
        for f in FINGERS:
            cal["finger_len"][f] = float(np.linalg.norm(p[HUMAN_TIP[f]] - p[HUMAN_MCP[f]]))
            cal["wrist_to_tip"][f] = float(np.linalg.norm(p[HUMAN_TIP[f]] - wrist))
            chain = HUMAN_CHAINS[f]
            bone = sum(float(np.linalg.norm(p[b] - p[a]))
                       for a, b in zip(chain[:-1], chain[1:]))
            line = float(np.linalg.norm(p[chain[-1]] - p[chain[0]]))
            cal["extension"][f] = line / bone if bone > EPS else 0.0
        for f in FINGERS[1:]:
            cal["tip_gap"][f] = float(np.linalg.norm(p[HUMAN_TIP[f]] - p[HUMAN_TIP["thumb"]]))
        cal["pp_gap"] = float(np.linalg.norm(p[HUMAN_PIP["index"]] - p[HUMAN_TIP["thumb"]]))
        cal["gap_pair"] = [float(np.linalg.norm(p[b] - p[a])) for a, b in GAP_NODES]
        cal["flat"] = min(cal["extension"].values()) >= 0.90

        # --- palm-fit residual, DIAGNOSTIC ONLY.
        #
        # The alignment used to come out of this Umeyama fit, and that was a
        # design error: the fitted point set (coplanar MCPs plus normalised
        # tips) leaves rotation about the knuckle line nearly unconstrained,
        # so the fitted rotation wandered 20 deg across three flat holds of
        # the same operator (36.7 deg across seventeen, HANDOVER.md) while the
        # residual stayed put at ~10 mm -- and the residual predicts nothing
        # (r = +0.10). The alignment is now CONSTRUCTED below; this fit is
        # kept because its residual and drawn points are still the honest
        # "did the capture look like a hand held flat" health check.
        keys = FINGERS[1:]
        reach = float(np.mean([self.robot_finger_len[f] for f in keys]))

        def normalised_tips(mcp, tip):
            out = []
            for a, b in zip(mcp, tip):
                out.append(a + _unit(b - a) * reach)
            return out

        A = np.array([p[HUMAN_MCP[f]] for f in keys] + normalised_tips(
            [p[HUMAN_MCP[f]] for f in keys], [p[HUMAN_TIP[f]] for f in keys]))
        B = np.array([self.rest_mcp[f] for f in keys] + normalised_tips(
            [self.rest_mcp[f] for f in keys], [self.rest_tip[f] for f in keys]))
        R, t, s = _umeyama(A, B)
        fitted = (s * (R @ A.T).T) + t
        cal["palm_fit_mm"] = float(np.linalg.norm(fitted - B, axis=1).mean())
        cal["palm_fit_max_mm"] = float(np.linalg.norm(fitted - B, axis=1).max())
        cal["palm_scale"] = s
        # Kept so the fit can be drawn: where the operator's palm landed in robot
        # space, against where the robot's own palm actually is.
        cal["fit_labels"] = [f"{f}_mcp" for f in keys] + [f"{f}_tip*" for f in keys]
        cal["fit_human"] = fitted
        cal["fit_robot"] = B

        # --- the alignment itself: constructed, not fitted.
        #
        # Both sides define the same anatomical frame from rigid-palm
        # landmarks -- across the palm (index MCP - pinky MCP) and along it
        # (middle MCP - wrist) -- and the alignment is the rotation between
        # the two frames. Zero leftover degrees of freedom, so it is exactly
        # repeatable where the fit wandered (0.0 vs 20.2 deg across the same
        # three holds), and on the recordings it retargets slightly better
        # (pinch median 2.2 vs 5.1 mm, everything else equal). The virtual
        # wrist comes from anchoring the MCP centroids and carrying the
        # operator's wrist->palm vector over at across-palm scale.
        if not self.wrist_is_annotated:
            self.align_R = self._palm_align(p)
            mcpc_h = np.mean([p[HUMAN_MCP[f]] for f in keys], axis=0)
            s_palm = (np.linalg.norm(self.rest_mcp["index"] - self.rest_mcp["pinky"])
                      / max(np.linalg.norm(p[HUMAN_MCP["index"]]
                                           - p[HUMAN_MCP["pinky"]]), EPS))
            self.wrist_offset = mcp_centroid - s_palm * (
                self.align_R @ (mcpc_h - wrist))
        self.robot_wrist_to_tip = {
            f: float(np.linalg.norm(self.rest_tip[f] - self.wrist_offset)) for f in FINGERS
        }

        # The thumb gets its own rotation on top of the palm fit.
        #
        # It is excluded from the palm fit because its base is mobile, and that
        # exclusion has a consequence: the palm rotation says nothing about where
        # the thumb points. On an anthropomorphic hand that hardly matters, but
        # the L20's thumb sits low and to the side while an operator's reaches
        # forward across the palm, so the palm rotation aims the target 157 mm
        # from anywhere the thumb can go -- scaling fixes length, never direction.
        # This is the minimal rotation carrying the operator's rest thumb
        # direction onto the robot's, so the rest pose maps to the rest pose and
        # what gets transferred is how the thumb moves from there.
        u_h = _unit(self.align_R @ (p[HUMAN_TIP["thumb"]] - wrist))
        u_r = _unit(self.rest_tip["thumb"] - self.wrist_offset)
        axis = np.cross(u_h, u_r)
        sin_a, cos_a = float(np.linalg.norm(axis)), float(np.dot(u_h, u_r))
        if sin_a < 1e-8:
            self.thumb_R = np.eye(3) if cos_a > 0 else -np.eye(3)
        else:
            k = axis / sin_a
            K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
            self.thumb_R = np.eye(3) + sin_a * K + (1 - cos_a) * (K @ K)
        cal["thumb_realign_deg"] = float(np.degrees(np.arctan2(sin_a, cos_a)))

        self.scale_tip = {f: self.robot_wrist_to_tip[f] / max(cal["wrist_to_tip"][f], EPS)
                          for f in FINGERS}
        self.scale_gap = {f: self.robot_tip_gap[f] / max(cal["tip_gap"][f], EPS)
                          for f in FINGERS[1:]}
        self.scale_pp = self.rest_pp_gap / max(cal["pp_gap"], EPS)
        self.scale_gap_pair = [r / max(h, EPS)
                               for r, h in zip(self.rest_gap_pair, cal["gap_pair"])]

        # Fixed rotation taking an operator node frame into the corresponding
        # robot link frame. Derived from the two rest poses, so the orientation
        # term never has to assume the two conventions agree.
        #
        # PORT NOTE (tip_ori): the oracle compares raw quaternions, because its
        # link frames are human frames. These are not, so the comparison happens
        # through this alignment: at the calibration pose the residual is zero
        # by construction, and what the term then measures is deviation of
        # relative motion -- the honest analogue of their raw comparison.
        # Node orientations are compared in palm-mapped form (align_R times
        # the quaternion matrix) so that, like the positions, they are
        # invariant to the stream's global orientation.
        self.frame_align = {
            f: self.rest_robot_R[f].T @ (self.align_R @ _quat_to_matrix(quat[HUMAN_DIP[f]]))
            for f in FINGERS
        }

        # PORT NOTE: on an anthropomorphic hand the oracle's calibration is
        # the identity -- its link origins sit where a human's joints are, raw
        # keypoints are already targets, and the only transform is a fixed
        # 10 mm base-to-wrist offset (OBJECTIVE.md, tip_pos). The palm fit
        # above still runs for its diagnostics, but the derived maps are
        # overridden so the wave reproduces the oracle's own targets exactly.
        if self.spec.anthropomorphic:
            self.align_R = np.eye(3)
            self.thumb_R = np.eye(3)
            self.wrist_offset = np.array([0.0, 0.0, 0.01])
            self.scale_tip = {f: 1.0 for f in FINGERS}
            self.scale_gap = {f: 1.0 for f in FINGERS[1:]}
            self.scale_pp = 1.0
            self.scale_gap_pair = [1.0] * 4
            self.frame_align = {f: np.eye(3) for f in FINGERS}

        self.calibration = cal
        return cal

    def calibrate_fist(self, full_fist: np.ndarray) -> dict:
        """Second calibration pose: the operator's tightest fist.

        Only `flexion_guess` (the cold-start initial guess) needs it now: the
        ported fist term measures the operator against the oracle's fixed
        human budget, exactly as they do, so it no longer switches off without
        this pose.
        """
        p = np.asarray(full_fist, dtype=float)[:, :3]
        full = {}
        for f in FINGERS:
            for which in (("mcp", "ip") if f == "thumb" else ("mcp", "pip", "dip")):
                full[(f, which)] = float(human_bend_angle(p, f, which))
        self.human_full = full
        return full

    # ---------------------------------------------------------------- solver

    def _palm_align(self, p: np.ndarray) -> np.ndarray:
        """Alignment for THIS frame: robot palm frame times the operator's.

        Recomputed per frame rather than frozen at calibration, so the
        operator's absolute palm orientation -- and any fixed rotation between
        data sources (live SDK stream vs workstation recordings differ by one)
        -- drops out entirely. These hands have no wrist joint; the palm
        orientation was never theirs to follow.
        """
        x = p[HUMAN_MCP["index"]] - p[HUMAN_MCP["pinky"]]
        x = x / (np.linalg.norm(x) + EPS)
        z = np.cross(x, p[HUMAN_MCP["middle"]] - p[HUMAN_WRIST])
        z = z / (np.linalg.norm(z) + EPS)
        F_h = np.stack([x, np.cross(z, x), z], axis=1)
        return self._palm_frame_robot @ F_h.T

    def _aa_fe_indices(self, f: str) -> tuple[int, int] | None:
        """(AA, FE) joint indices for one finger's MCP, by name convention:
        LinkerHands use `*_mcp_roll` / `*_mcp_pitch`, the SharpaWave
        `*_MCP_AA` / `*_MCP_FE`."""
        aa = fe = None
        for name, idx in self.kin.index_of.items():
            if f not in name:
                continue
            if name.endswith(("mcp_roll", "MCP_AA")):
                aa = idx
            elif name.endswith(("mcp_pitch", "MCP_FE")):
                fe = idx
        return (aa, fe) if aa is not None and fe is not None else None

    def _build_solver(self):
        q = ca.MX.sym("q", self.n)
        tip = {f: self.kin.point_position(*self.spec.tip_frames[f], q) for f in FINGERS}
        mcp = {f: self.kin.point_position(self.spec.mcp_links[f], np.zeros(3), q)
               for f in FINGERS}
        pip = {f: self.kin.point_position(self.spec.pip_links[f], np.zeros(3), q)
               for f in FINGERS}
        droot = {f: self.kin.point_position(
            self.spec.dir_root_links.get(f, self.spec.mcp_links[f]), np.zeros(3), q)
            for f in FINGERS}
        dip, dipR = {}, {}
        for f in FINGERS:
            R, pos = self.kin.link_pose(self.spec.distal_links[f], q)
            dip[f], dipR[f] = pos, R
        # The frame tip_ori aligns: the oracle compares the *fingertip link*
        # frame against the *tip node* quaternion. On the LinkerHands the tip
        # frame is the distal link, so this is the same frame as dipR there.
        tipR = {f: self.kin.link_pose(self.spec.tip_frames[f][0], q)[0]
                for f in FINGERS}

        def norm(v):
            # The oracle's norms are bare sqrt; the epsilon (the same 1e-12
            # their dq term carries) is solver hygiene -- IPOPT needs the
            # gradient finite at exact contact -- and shifts values by < 1e-6.
            return ca.sqrt(ca.sumsqr(v) + 1e-8)

        def sabs(x):
            # Smoothed |x| for the same reason; exact to 1e-6.
            return ca.sqrt(x * x + 1e-8)

        def shinge(x):
            # Smoothed max(0, x): one-sided distance residuals (PORT NOTE at
            # the pinch term).
            return 0.5 * (x + ca.sqrt(x * x + 1e-8))

        self._param_spec = [
            ("wrist", 3), ("tgt_tip", 15), ("revgate", 4), ("fade", 4),
            ("tgt_R", 45),
            ("tgt_dir", 15), ("tgt_dipdir", 12),
            ("pinch_tgt", 4), ("pinch_dir", 12), ("gate", 4), ("ori_gate", 4),
            ("pad_gate", 4), ("thumb_fade", 1), ("thumb_band", 1),
            ("pp_tgt", 1), ("pp_dir", 3), ("gap_tgt", 12), ("gap_gate", 4),
            ("fist_tgt", 4), ("fist_gate", 4), ("tgt_thumb", 2),
            ("q_prev", self.n), ("w", len(WEIGHT_KEYS)),
        ]
        P = {n: ca.MX.sym(n, s) for n, s in self._param_spec}
        # On an anthropomorphic hand the oracle's forms apply verbatim; the
        # PORT-NOTE replacements below exist for the hands where they cannot
        # (see HandSpec.anthropomorphic).
        A = self.spec.anthropomorphic
        wrist = P["wrist"]
        (w_tip_pos, w_tip_ori, w_finger_ori, w_pinch, w_pinch_ori, w_dq,
         w_thumb_ang, w_dip_ori, w_pip_pinch, w_fist, w_pip_gap, w_coupling,
         w_pad_ori) = ca.vertsplit(P["w"])

        # `losses` holds the RAW per-term values, matching the oracle's
        # get_losses semantics; the weights are applied once, in the total.
        losses = {}

        # tip_pos: wrist-relative fingertip positions, fading out during a
        # pinch (the reversed gate); thumb ungated at their 0.2315.
        e = W_TIP_POS_THUMB * norm((tip["thumb"] - wrist) - P["tgt_tip"][0:3])
        for i, f in enumerate(FINGERS[1:]):
            e = e + P["revgate"][i] * norm((tip[f] - wrist)
                                           - P["tgt_tip"][3 * (i + 1):3 * (i + 1) + 3])
        losses["tip_pos"] = e

        # PORT NOTE (the revgates below): the oracle fades tip_pos out during a
        # pinch -- its reversed gate -- so that contact beats absolute
        # geometry. On its human-shaped hand the orientation and direction
        # terms never conflict with contact, so only tip_pos needed the fade;
        # on these hands they do conflict (no single term, but their sum pins
        # the digits at the mapped geometry: ablating any one of them moves an
        # L20 pinch by < 20 mm while all of them together hold it 70 mm off
        # target). The fade is therefore applied per digit to tip_ori, dip_ori
        # and finger_ori as well: the same mechanism, where the same conflict
        # now exists. The thumb fades on the deepest of the four gates.

        # tip_ori: fingers align the distal z-axis; the thumb pays the full
        # quaternion distance at double weight, via the trace identity
        # |<q1,q2>| = sqrt((tr(R1^T R2) + 1) / 4).
        # PORT NOTE (thumb share, non-anthropomorphic): the thumb components
        # of tip_ori and finger_ori are DELETED off the anthropomorphic path,
        # not faded. No rotation makes the two thumb *motions* correspond --
        # the rest-aligned map carries the operator's bend along a path the
        # L20's MCP cannot follow -- so these terms pin the thumb straight:
        # measured at an open-hand bent-thumb frame, finger_ori's thumb share
        # pushed +0.47/rad against thumb_ang's full -0.50/rad, freezing the
        # joint at 7 deg while the operator covered 18-174. Their job is done
        # by thumb_ang (operator angles, transfers verbatim), pip_pinch
        # (opposition) and pinch/pad_ori (contact). This is the acceptance
        # rule "no homologue -> delete and replace, not scale down" -- the
        # measured, per-term form of what w_thumb_geom=0.03 once approximated.
        Rt_th = ca.reshape(P["tgt_R"][0:9], 3, 3)
        tr = ca.trace(ca.mtimes(tipR["thumb"].T, Rt_th))
        quat_th = W_TIP_ORI_THUMB * (1.0 - ca.sqrt(ca.fmax(tr + 1.0, 0.0) / 4.0))
        # Off the anthropomorphic path the thumb share lives in the APPROACH
        # BAND, gate x (1 - contact fraction): near zero with the hand open,
        # ~0.5 through the 15-40 mm approach, zero again at contact. Its job
        # there is to PARK the thumb where the L20's reliable pinch strategy
        # needs it (thumb held, index closes); open-handed it must yield to
        # thumb_ang or the thumb never bends, and at contact it must yield to
        # the contact terms. Measured on the decoupled pair of recordings
        # (thumb_bend_left = open-hand thumb sweeps, replay_left = grasping):
        #   always-on (fade by 1-max gate): pinch 2.0 mm @ 142 deg, but the
        #     thumb FROZEN at 0 deg through 19-172 deg of operator bend;
        #   deleted: thumb tracks (corr 0.67), pinch lands edge-on (92 deg);
        #   approach band + thumb_ang fading on the wide gate: thumb tracks
        #     (corr 0.65), pinch 5.1 mm @ 150 deg.
        # The band is the kept trade. If closure ever matters more than thumb
        # bend, the always-on variant is this gate swapped to P["thumb_fade"],
        # which is the rescaled reversed gate: on everywhere except at contact.
        _g_th = P["thumb_band"]
        e = (quat_th if A else _g_th * quat_th)
        for i, f in enumerate(FINGERS[1:], start=1):
            Rt = ca.reshape(P["tgt_R"][9 * i:9 * i + 9], 3, 3)
            e = e + (1.0 if A else P["fade"][i - 1]) * (
                1.0 - ca.dot(tipR[f][:, 2], Rt[:, 2]))
        losses["tip_ori"] = e

        # finger_ori: one direction per digit (MCP -> tip), summed, capped,
        # cubed. Negligible when directions roughly agree, dominant when any
        # digit points wrong.
        s = 0
        for i, f in enumerate(FINGERS):
            cosang = ca.dot(_unit(tip[f] - droot[f]), P["tgt_dir"][3 * i:3 * i + 3])
            if f == "thumb":
                g = 1.0 if A else _g_th             # see the tip_ori note
            else:
                g = 1.0 if A else P["fade"][i - 1]
            s = s + g * (W_DIR_THUMB if f == "thumb" else 1.0) * 2.0 * (1.0 - cosang)
        losses["finger_ori"] = ca.fmin(s, 5.0) ** 3

        # pinch: gated, contact-snapped residual thumbtip -> fingertip, ring
        # and pinky heavily up-weighted (their 6 and 8).
        #
        # PORT NOTE: theirs is a two-sided vector residual, and neither part
        # survives the geometry transfer. The vector: thumb-relative
        # *directions* have no map -- the LinkerHand thumbs point 73-94 deg
        # away, the mapped direction is routinely unreachable, and the solve
        # stalls in a certified local minimum (measured: 135 mm against a
        # 46 mm target on a pinch-only objective whose true minimum is
        # contact). The two-sidedness: their hand preserves the operator's
        # inter-digit distances, so far pairs sit at their targets for free;
        # after scaling to a non-anthropomorphic hand they cannot (pinching
        # the index puts the L20 thumb 120 mm from the pinky where the scaled
        # target is 237 mm), and with ring x6 / pinky x8 the far pairs outbid
        # the pinch itself -- measured, the solver walks *away* from a
        # near-perfect start. What transfers is "at least as closed as the
        # operator": a one-sided hinge on distance. Direction preference is
        # pinch_ori's job and spacing is pip_gap's, both soft.
        e = 0
        for i, f in enumerate(FINGERS[1:]):
            v = tip[f] - tip["thumb"]
            if A:
                r = norm(v - P["pinch_tgt"][i] * P["pinch_dir"][3 * i:3 * i + 3])
            else:
                r = shinge(norm(v) - P["pinch_tgt"][i])
            e = e + W_PINCH[f] * P["gate"][i] * r
        losses["pinch"] = e

        # pinch_ori: direction of the same pairs, ring and pinky nearly off.
        e = 0
        for i, f in enumerate(FINGERS[1:]):
            v = tip[f] - tip["thumb"]
            e = e + W_PINCH_ORI[f] * P["ori_gate"][i] * (
                1.0 - ca.dot(_unit(v), P["pinch_dir"][3 * i:3 * i + 3]))
        losses["pinch_ori"] = e

        # dq: an L2 norm, not a squared sum -- linear in the step, so it damps
        # without the quadratic's vanishing gradient at small steps.
        losses["dq"] = ca.sqrt(ca.sumsqr(0.01 * (q - P["q_prev"])) + 1e-12)

        # thumb_ang: their unreported always-on thumb-IP angle map, adapted to
        # this hand's thumb DOFs through spec.thumb_angle_map (the L20's one
        # joint drives MCP and IP together, so it tracks their sum). Two
        # non-anthropomorphic adaptations, both measured: it fades during a
        # pinch like the other absolute-geometry terms, and its residual is L1
        # rather than their square. Their IP is decoupled from opposition, but
        # the L20's thumb_mcp IS the contact degree of freedom; with the
        # square, the ~1.2 rad structural offset gives this prior a 0.32/rad
        # gradient that overpowers the pinch's bounded 0.15/rad and holds
        # pinches 32 mm open. L1 caps it at 0.13/rad -- the same reason every
        # other residual in their objective is a norm, applied here.
        # Its fade signal is the snap contact fraction (shared with pad_ori),
        # not the wide pinch gate: the wide gate is half-on through the whole
        # 30-50 mm approach band, which on the replay meant the thumb prior
        # was faded on 55% of ALL frames (median 0.41) and the thumb never
        # bent -- operator covered 18-174 deg of thumb bend, the robot 0-37.
        # With the narrow signal the prior only yields when contact is real.
        e = 0
        for k, jname in enumerate(self.spec.thumb_angle_map):
            r = q[self.kin.index_of[jname]] - P["tgt_thumb"][k]
            e = e + (r ** 2 if A else sabs(r))
        losses["thumb_ang"] = e if A else P["thumb_fade"] * e

        # dip_ori: per finger (no thumb), the MCP->DIP direction as a norm of
        # unit-vector difference; the oracle adds it twice, so the 2 stays.
        e = 0
        for i, f in enumerate(FINGERS[1:]):
            e = e + (1.0 if A else P["fade"][i]) * 2.0 * norm(
                _unit(dip[f] - droot[f]) - P["tgt_dipdir"][3 * i:3 * i + 3])
        losses["dip_ori"] = e

        # pip_pinch: thumbtip against the index PIP, snapped, with their frozen
        # gate ported as the constant 0.5 it is. Always on: this is what holds
        # the thumb opposed outside a pinch. One-sided scalar off the
        # anthropomorphic path, for the same reasons as pinch.
        v = pip["index"] - tip["thumb"]
        if A:
            r = norm(v - P["pp_tgt"] * P["pp_dir"])
        else:
            r = shinge(norm(v) - P["pp_tgt"])
        losses["pip_pinch"] = W_PIP_PINCH_GATE * r

        # fist: per finger, remaining extension against the snapped human
        # target, L1, gated on how closed the operator is.
        e = 0
        for i, f in enumerate(FINGERS[1:]):
            remaining = sum(float(self.hi[j]) - q[j] for j in self._flex_by_finger[f])
            e = e + P["fist_gate"][i] * sabs(remaining - P["fist_tgt"][i])
        losses["fist"] = e

        # pip_gap: adjacent-digit spacing at PIP level, two-sided -- it holds
        # fingers apart and together, tracking the operator's actual spread.
        # Replaces the one-sided anti-crossing term this repo used to have.
        # The finger-finger pairs keep their vector form everywhere (finger
        # directions go through the palm fit); the thumb-index pair exists
        # only on the anthropomorphic path -- scaled to a LinkerHand it holds
        # the thumb *out* at pinch-incompatible distances (PORT NOTE at the
        # pinch term), and the thumb it protects against crossing cannot cross.
        robot_gap_chain = [dip["thumb"]] + [pip[f] for f in FINGERS[1:]]
        e = 0
        if A:
            e = P["gap_gate"][0] * norm((robot_gap_chain[1] - robot_gap_chain[0])
                                        - P["gap_tgt"][0:3])
        for j in range(1, 4):
            v = robot_gap_chain[j + 1] - robot_gap_chain[j]
            e = e + P["gap_gate"][j] * norm(v - P["gap_tgt"][3 * j:3 * j + 3])
        losses["pip_gap"] = e

        # pad_ori: during a pinch the thumb pad and the finger pad should FACE
        # each other, not meet edge-on. PORT NOTE: this is the pinch-time
        # stand-in for tip_ori, which fades out during a pinch on these hands
        # (see the revgate note above) -- the oracle gets pad-to-pad for free
        # from its always-on tip_ori because its hand is human-shaped. It
        # carries tip_ori's own weight and gating; the normals are annotated
        # robot geometry (hands.TIP_NORMALS), no operator quantity involved.
        # Measured need: with contact points touching at 0.7 mm the pads met
        # at 90 deg on the replay. Off when the hand has no annotated normals.
        e = 0
        if self.spec.tip_normals:
            n_th = ca.DM(self.spec.tip_normals["thumb"])
            for i, f in enumerate(FINGERS[1:]):
                if f not in self.spec.tip_normals:
                    continue
                n_f = ca.DM(self.spec.tip_normals[f])
                cosnn = ca.dot(ca.mtimes(dipR[f], n_f),
                               ca.mtimes(dipR["thumb"], n_th))
                e = e + P["pad_gate"][i] * (1.0 + cosnn)
        losses["pad_ori"] = e

        # coupling: abduction room shrinks with flexion. On the anthropomorphic
        # path this is the oracle's own cone (|AA| <= 0.46774824 - 0.22*FE,
        # plus their asymmetric thumb-CMC pair); elsewhere the 0.22 slope is
        # kept and the intercept is this robot's own AA limit (PORT NOTE:
        # their intercept 0.4677 = 0.22*(pi/2) + 7 deg is a human-anatomy
        # number, and their thumb pair has no homologue on these thumbs).
        e = 0
        for f in FINGERS[1:]:
            pair = self._aa_fe_indices(f)
            if pair is None:
                continue
            aa, fe = pair
            if A:
                up = q[aa] + COUPLE_SLOPE * q[fe] - 0.46774823953448036
                dn = COUPLE_SLOPE * q[fe] - q[aa] - 0.46774823953448036
            else:
                flex = q[fe] - float(self.lo[fe])
                up = q[aa] - (float(self.hi[aa]) - COUPLE_SLOPE * flex)
                dn = (float(self.lo[aa]) + COUPLE_SLOPE * flex) - q[aa]
            e = e + (COUPLE_AMP * ca.log(1 + ca.exp(COUPLE_K * up))) ** 2 \
                  + (COUPLE_AMP * ca.log(1 + ca.exp(COUPLE_K * dn))) ** 2
        if A:
            cmc_fe = next((i for n, i in self.kin.index_of.items()
                           if n.endswith("thumb_CMC_FE")), None)
            cmc_aa = next((i for n, i in self.kin.index_of.items()
                           if n.endswith("thumb_CMC_AA")), None)
            if cmc_fe is not None and cmc_aa is not None:
                e = e + (COUPLE_AMP * ca.log(1 + ca.exp(COUPLE_K * (
                    q[cmc_aa] - 0.33 * q[cmc_fe] - 0.005235987755982982)))) ** 2
                e = e + (COUPLE_AMP * ca.log(1 + ca.exp(COUPLE_K * (
                    q[cmc_aa] + 0.66 * q[cmc_fe] - 1.2147491593880533)))) ** 2
        losses["coupling"] = e

        self._loss_names = list(losses)
        total = (w_tip_pos * ORACLE_W["tip_pos"] * losses["tip_pos"]
                 + w_tip_ori * ORACLE_W["tip_ori"] * losses["tip_ori"]
                 + w_finger_ori * ORACLE_W["finger_ori"] * losses["finger_ori"]
                 + w_pinch * ORACLE_W["pinch"] * losses["pinch"]
                 + w_pinch_ori * ORACLE_W["pinch_ori"] * losses["pinch_ori"]
                 + w_dq * ORACLE_W["dq"] * losses["dq"]
                 + w_thumb_ang * ORACLE_W["thumb_ang"] * losses["thumb_ang"]
                 + w_dip_ori * ORACLE_W["dip_ori"] * losses["dip_ori"]
                 + w_pip_pinch * ORACLE_W["pip_pinch"] * losses["pip_pinch"]
                 + w_fist * ORACLE_W["fist"] * losses["fist"]
                 + w_pip_gap * ORACLE_W["pip_gap"] * losses["pip_gap"]
                 + w_coupling * ORACLE_W["coupling"] * losses["coupling"]
                 + w_pad_ori * ORACLE_W["pad_ori"] * losses["pad_ori"])

        p_vec = ca.vertcat(*[P[n] for n, _ in self._param_spec])
        nlp = {"x": q, "f": total, "p": p_vec}
        # The ported objective is built from L1 norms, so it needs different
        # solver machinery than the old squared cost: with limited-memory BFGS
        # every frame dies at max_iter, with the exact Hessian, an adaptive
        # barrier and SX expansion the same trajectory is 70/70 Solve_Succeeded
        # at 12 ms mean (measured on wave_left, fist + pinch sweep).
        self.solver = ca.nlpsol("retarget", "ipopt", nlp, {
            "print_time": False, "expand": True,
            "ipopt": {
                "print_level": 0, "sb": "yes",
                "max_iter": self.cfg.max_iter, "tol": self.cfg.tol,
                "acceptable_tol": self.cfg.tol * 20, "acceptable_iter": 3,
                "warm_start_init_point": "yes",
                "warm_start_bound_push": 1e-8, "warm_start_mult_bound_push": 1e-8,
                "mu_strategy": "adaptive",
            },
        })
        self._loss_fn = ca.Function("losses", [q, p_vec], list(losses.values()),
                                    ["q", "p"], self._loss_names)

    # ------------------------------------------------------------- inference

    def _human_params(self, kp: np.ndarray) -> np.ndarray:
        if self.calibration is None:
            raise RuntimeError(
                "calibrate() has not been run. Every operator dimension the cost "
                "function uses is measured from that pose; there are no defaults.")
        p, quat = kp[:, :3], kp[:, 3:7]
        wrist = p[HUMAN_WRIST]

        # The alignment follows the operator's palm frame-by-frame (see
        # _palm_align); the anthropomorphic path keeps its identity mapping.
        if not self.spec.anthropomorphic and not self.wrist_is_annotated:
            self.align_R = self._palm_align(p)

        # Every human vector is expressed in the operator's wrist frame; align_R
        # is what carries it into the robot's base frame.
        # tip_ori's human frame: the oracle reads the tip node's quaternion;
        # off the anthropomorphic path the DIP node's carries the same roll and
        # is what frame_align was derived against.
        ori_node = HUMAN_TIP if self.spec.anthropomorphic else HUMAN_DIP
        tgt_tip, tgt_R, tgt_dir, tgt_dipdir = [], [], [], []
        for f in FINGERS:
            Rw = self.thumb_R @ self.align_R if f == "thumb" else self.align_R
            tgt_tip.append(Rw @ (p[HUMAN_TIP[f]] - wrist) * self.scale_tip[f])
            tgt_dir.append(_unit(Rw @ (p[HUMAN_TIP[f]] - p[DIR_ROOT[f]])))
            R_target = (self.align_R @ _quat_to_matrix(quat[ori_node[f]])) @ self.frame_align[f].T
            tgt_R.append(R_target.reshape(9, order="F"))
            if f != "thumb":
                tgt_dipdir.append(_unit(Rw @ (p[HUMAN_DIP[f]] - p[HUMAN_MCP[f]])))

        # Pinch pairs: gates and snaps read the operator (their thresholds
        # describe an operator's fingers -- 10 mm is touching pads); the target
        # magnitude is then scaled into robot space, so operator contact means
        # robot contact-point contact.
        pinch_d = np.array([float(np.linalg.norm(p[HUMAN_TIP[f]] - p[HUMAN_TIP["thumb"]]))
                            for f in FINGERS[1:]])
        gate = _gate(pinch_d)
        revgate = 1.0 - gate
        fade = _fade(revgate)
        ori_gate = gate / (1.0 + np.exp(-RAMP_K * (pinch_d - RAMP_R)))
        pinch_tgt, pinch_dir = np.zeros(4), []
        for i, f in enumerate(FINGERS[1:]):
            v = p[HUMAN_TIP[f]] - p[HUMAN_TIP["thumb"]]
            pinch_dir.append(_unit(self.align_R @ v))
            pinch_tgt[i] = _snap(pinch_d[i]) * self.scale_gap[f]

        v = p[HUMAN_PIP["index"]] - p[HUMAN_TIP["thumb"]]
        d = float(np.linalg.norm(v))
        pp_tgt = np.array([_snap(d) * self.scale_pp])
        pp_dir = _unit(self.align_R @ v)

        gap_tgt, gap_gate = [], np.zeros(4)
        for j, (a, b) in enumerate(GAP_NODES):
            v = p[b] - p[a]
            d = float(np.linalg.norm(v))
            gap_tgt.append(_unit(self.align_R @ v)
                           * (_snap(d) * self.scale_gap_pair[j]))
            gap_gate[j] = W_GAP_PAIR[j] * (1.0 - np.tanh(GAP_GATE_K * (d - GATE_R)))
        # A pinching finger's spacing pairs yield to the pinch (same fade family
        # as tip_ori/dip_ori/finger_ori): reaching the thumb can demand far more
        # curl than the operator's, and holding the finger's MP at the
        # operator-shaped spacing was one third of the coalition hovering the
        # O30i 15 mm short of contact (individually removing finger_ori,
        # pip_gap or thumb_ang moves the median <2 mm; removing all three
        # closes it, 14.8 -> 2.9). Pairs are (thumb,idx),(idx,mid),(mid,ring),
        # (ring,pinky); pair j >= 1 involves fingers j-1 and j.
        if not self.spec.anthropomorphic:
            for j in range(1, 4):
                gap_gate[j] *= min(fade[j - 1], fade[j])

        # fist: remaining extension per finger, from the operator's own joint
        # angles, against the oracle's fixed human budget.
        fist_tgt, fist_gate = np.zeros(4), np.zeros(4)
        for i, f in enumerate(FINGERS[1:]):
            bend = sum(human_bend_angle(p, f, w) for w in ("mcp", "pip", "dip"))
            theta = FIST_BUDGET_H - bend
            fist_tgt[i] = (self.flex_budget[f] / FIST_BUDGET_H) * _fist_snap(theta)
            fist_gate[i] = 0.5 * (1.0 - np.tanh(theta - FIST_SNAP_HI))

        # thumb angle map: raw operator angles onto the thumb joints, clipped
        # into range. Always on -- radians transfer without calibration.
        tgt_thumb = np.zeros(2)
        for k, (jname, parts) in enumerate(self.spec.thumb_angle_map.items()):
            idx = self.kin.index_of[jname]
            now = sum(human_bend_angle(p, "thumb", w) for w in parts)
            tgt_thumb[k] = float(np.clip(now, self.lo[idx], self.hi[idx]))

        # pad_ori's gate: the pinch gate times the CONTACT FRACTION of the
        # snap -- how much of the operator's gap the snap commands away. 1 when
        # the operator is truly at contact (< 10 mm), 0 beyond the snap band.
        # Without it the term judges pad facing in configurations that are not
        # meeting at all, where facing is cheapest when the pinch stays OPEN --
        # it was rewarding a 32 mm stall.
        contact_frac = np.array([1.0 - _snap(d) / max(d, 1e-9) for d in pinch_d])
        pad_gate = gate * contact_frac
        # thumb_ang's fade, put through the fade family like every other faded
        # term. It was the RAW `1 - gate.max()`, i.e. revgate.min() unrescaled,
        # so it floored at 0.231 at operator contact instead of reaching zero --
        # the one place the fade family's rescale was not applied, though
        # thumb_ang is one of the three terms the commit that introduced _fade
        # names as the coalition it was removing. The floor matters here and not
        # on the L20 because of how much of the thumb thumb_ang owns: this map
        # pins two of the O30i thumb's four joints (thumb_mcp, thumb_ip) against
        # one of the L20's four, leaving the pinch only cmc_roll and cmc_yaw to
        # close with -- and cmc_roll saturates at its 35 deg limit while it
        # tries. Measured, rescaling alone: O30i thumb-index 21.0 -> 2.3 mm
        # median at operator contact, pad angle 133 -> 139 deg; L20 left
        # near-contact 7.3 -> 4.2 mm at 152 deg; thumb bend tracking unchanged
        # on the open-hand sweeps (corr 0.54 -> 0.53).
        thumb_fade = np.array([_fade(revgate.min())])
        # The approach band that carries the thumb's tip_ori/finger_ori share.
        # Its own note requires it to be off in three states: open (the gate),
        # at contact (the contact fraction), and -- the part the max() lost --
        # at contact on ANY pair. Per pair the factor is right, but the max over
        # pairs is zero exactly for the pair that has arrived, so a neighbouring
        # finger still crossing the 30-50 mm band re-armed the prior while the
        # operator was fully pinched: median 0.43 over the middle pair's contact
        # frames, 100% of them above 0.2, against a thumb_fade of 0.09 on the
        # same frames. Middle is the worst pair because it is the only one with
        # a neighbour on each side to be re-armed by. Gating the band on
        # thumb_fade -- the file's existing "no pair has arrived" factor -- is
        # what "at contact it must yield to the contact terms" already meant.
        thumb_band = np.array([(gate * (1.0 - contact_frac)).max() * thumb_fade[0]])

        self._last = {"pinch_dist": pinch_d, "gate": gate,
                      "gap_target": pinch_tgt.copy()}
        return np.concatenate([
            self.wrist_offset,
            np.concatenate(tgt_tip), revgate, fade,
            np.concatenate(tgt_R),
            np.concatenate(tgt_dir), np.concatenate(tgt_dipdir),
            pinch_tgt, np.concatenate(pinch_dir), gate, ori_gate, pad_gate,
            thumb_fade, thumb_band,
            pp_tgt, pp_dir, np.concatenate(gap_tgt), gap_gate,
            fist_tgt, fist_gate, tgt_thumb, self.q_prev,
            np.array([getattr(self.cfg, k) for k in WEIGHT_KEYS]),
        ])

    def oppose_pose(self) -> np.ndarray:
        """The hand's own best thumb-index opposition, from its geometry alone.

        Solved once, multi-start, by minimising the thumb-index contact
        distance over the bounds. Its thumb joints seed a second start when a
        pinch is requested and the solve is stuck far from the target: from
        the rest pose the LinkerHand thumbs must swing around a cost hill to
        reach the fingers, and IPOPT does not cross hills -- measured on the
        L20, the warm-started path stalls at 81 mm on a pinch-only objective
        whose true minimum is exact contact. Same failure class as the fist
        cold start (`flexion_guess`), same medicine: a start in the right
        basin.
        """
        if self._q_oppose is not None:
            return self._q_oppose
        q = ca.MX.sym("q", self.n)
        Ri, oi = self.kin.link_pose(self.spec.distal_links["index"], q)
        ti = oi + ca.mtimes(Ri, ca.DM(np.asarray(self.spec.tip_frames["index"][1])))
        Rt, ot = self.kin.link_pose(self.spec.distal_links["thumb"], q)
        tt = ot + ca.mtimes(Rt, ca.DM(np.asarray(self.spec.tip_frames["thumb"][1])))
        d = ca.sqrt(ca.sumsqr(ti - tt) + 1e-12)
        opts = {"print_time": False, "expand": True,
                "ipopt": {"print_level": 0, "sb": "yes", "max_iter": 300}}
        rng = np.random.default_rng(0)
        # stage 1: touch
        s1 = ca.nlpsol("oppose", "ipopt", {"x": q, "f": d}, opts)
        best = None
        for _ in range(10):
            sol = s1(x0=rng.uniform(self.lo, self.hi), lbx=self.lo, ubx=self.hi)
            if best is None or float(sol["f"]) < best[0]:
                best = (float(sol["f"]), np.array(sol["x"]).flatten())
        q_touch = np.clip(best[1], self.lo, self.hi)
        # stage 2: keep touching, face the pads (only when normals are known).
        # Contact + antiparallel pads is reachable on the L20 -- verified by a
        # 20-start frontier solve -- but the warm-started teleop path cannot
        # find that basin on its own, which is exactly what this seed is for.
        if self.spec.tip_normals:
            facing = ca.dot(ca.mtimes(Ri, ca.DM(self.spec.tip_normals["index"])),
                            ca.mtimes(Rt, ca.DM(self.spec.tip_normals["thumb"])))
            s2 = ca.nlpsol("face", "ipopt", {"x": q, "f": facing, "g": d}, opts)
            best = None
            for k in range(10):
                x0 = q_touch if k == 0 else rng.uniform(self.lo, self.hi)
                sol = s2(x0=x0, lbx=self.lo, ubx=self.hi, lbg=0, ubg=0.002)
                st = s2.stats()
                ok = st.get("success") or "Acceptable" in st.get("return_status", "")
                if ok and (best is None or float(sol["f"]) < best[0]):
                    best = (float(sol["f"]), np.array(sol["x"]).flatten())
            if best is not None:
                q_touch = np.clip(best[1], self.lo, self.hi)
        self._q_oppose = q_touch
        return self._q_oppose

    def flexion_guess(self, keypoints: np.ndarray) -> np.ndarray | None:
        """An initial guess read straight off the operator's own joint angles.

        Each flexion joint is placed at the same fraction of its range that the
        operator's corresponding joint is of theirs. It is a crude pose -- it
        ignores abduction, the thumb, and the whole cost function -- but it is
        in the right *basin*, which is all an initial guess has to be.

        Needed because the rest pose is a bad guess for anything but an open
        hand, and IPOPT does not recover. Starting from rest with the operator
        in a fist, the solver settles with every finger against its lower bound
        and stays there: 0 deg of index PIP where reaching the same frame along
        a continuous trajectory gives 99 deg. That is not a marginal difference
        -- summed over the task terms the rest-start solution costs 22.3 against
        3.9 -- so the cost function prefers the right answer and the search
        simply failed to find it.

        Returns None until the fist pose is calibrated; there is no way to know
        what fraction of an operator's range a given bend represents without it.
        """
        if self.human_full is None:
            return None
        q = self.q_rest.copy()
        points = np.asarray(keypoints, dtype=float)[:, :3]
        for name in self.spec.flexion_joints:
            finger, _, which = name.rpartition("_")
            which = {"pitch": "mcp"}.get(which, which)
            finger = finger.replace("_mcp", "")
            key = (finger, which)
            if key not in self.human_full or name not in self.kin.index_of:
                continue
            full = self.human_full[key]
            if full < np.radians(10.0):    # a joint the glove does not report
                continue
            frac = float(np.clip(human_bend_angle(points, *key) / full, 0.0, 1.0))
            i = self.kin.index_of[name]
            q[i] = self.lo[i] + frac * (self.hi[i] - self.lo[i])
        return np.clip(q, self.lo, self.hi)

    def retarget(self, keypoints: np.ndarray, warm_start: bool = True) -> np.ndarray:
        kp = np.asarray(keypoints, dtype=float)
        par = self._human_params(kp)

        # The second start is insurance against beginning in the wrong basin,
        # and it is only worth paying for when the previous solution cannot be
        # trusted: the first solve after construction or reset, or after one
        # that failed. Measured over a 225-frame trajectory, once warm-started
        # the second start improved the cost on exactly zero frames (median and
        # maximum gain both 0.000) while doubling the time per frame. At the
        # first frame it is the difference between 0 and 99 deg of PIP.
        #
        # `_lam_x0` is the marker: it exists only after a solve has succeeded.
        starts = [self.q_prev]
        if self._lam_x0 is None or not self.last_stats.get("ok", True):
            guess = self.flexion_guess(kp)
            if guess is not None:
                starts.append(guess)
        # Basin insurance for the thumb: when the operator is pinching but the
        # previous solution's pinch is far from its target, offer a start whose
        # thumb sits in the hand's own opposition basin (see oppose_pose).
        gates, tgts = self._last["gate"], self._last["gap_target"]
        if gates.max() > 0.5:
            tips = self.fingertips(self.q_prev)
            worst = max(float(np.linalg.norm(tips[i + 1] - tips[0])) - tgts[i]
                        for i in range(4) if gates[i] > 0.5)
            if worst > 0.02:
                mix = (self.flexion_guess(kp) if self.human_full is not None
                       else self.q_prev).copy()
                # thumb AND index from the opposition pose: pad facing needs
                # both sides of the pinch in the right basin
                seed_idx = [i for j, i in self.kin.index_of.items()
                            if j in self.spec.thumb_joints or "index" in j]
                mix[seed_idx] = self.oppose_pose()[seed_idx]
                starts.append(np.clip(mix, self.lo, self.hi))

        best = None
        for index, start in enumerate(starts):
            kwargs = dict(x0=start, lbx=self.lo, ubx=self.hi, p=par)
            if index == 0 and warm_start and self._lam_x0 is not None:
                kwargs["lam_x0"] = self._lam_x0
            candidate = self.solver(**kwargs)
            cost = float(candidate["f"])
            if np.all(np.isfinite(np.array(candidate["x"]))) and (
                    best is None or cost < best[0]):
                best = (cost, candidate, self.solver.stats())
        if best is None:
            self.last_stats = {"ok": False, "status": "non-finite solution"}
            return self.q_filtered.copy()
        _, sol, stats = best
        q = np.array(sol["x"]).flatten()
        if not np.all(np.isfinite(q)):
            self.last_stats = {"ok": False, "status": "non-finite solution"}
            return self.q_filtered.copy()

        q = np.clip(q, self.lo, self.hi)
        self._lam_x0 = sol["lam_x"]
        self.q_prev2, self.q_prev = self.q_prev, q

        now = time.perf_counter()
        dt = now - self._last_call if self._last_call is not None else 0.0
        self._last_call = now
        tau = max(float(self.cfg.output_tau), 1e-4)
        a = 1.0 if dt <= 0.0 else float(np.clip(1.0 - np.exp(-dt / tau), 1e-3, 1.0))
        self.q_filtered = np.clip(a * q + (1.0 - a) * self.q_filtered, self.lo, self.hi)

        self.last_stats = {
            "ok": bool(stats.get("success", False)),
            "status": stats.get("return_status", "?"),
            "iters": stats.get("iter_count", -1),
            "cost": float(sol["f"]),
            "pinch_dist": self._last["pinch_dist"],
            "gap_target": self._last["gap_target"],
            "palm_fit_mm": self.calibration.get("palm_fit_mm"),
            "tip_targets": self.wrist_offset + par[3:18].reshape(5, 3),
            "fist_terms_enabled": True,
        }
        return self.q_filtered.copy()

    def loss_breakdown(self, q: np.ndarray, keypoints: np.ndarray) -> dict:
        """Raw per-term values, before the oracle weights -- the same
        convention as their get_casadi_cost_value(..., get_losses=True), so
        the two break downs compare directly."""
        vals = self._loss_fn(q, self._human_params(np.asarray(keypoints, dtype=float)))
        return {n: float(v) for n, v in zip(self._loss_names, vals)}

    def reset(self):
        self._last_call = None
        self.q_prev = self.q_rest.copy()
        self.q_prev2 = self.q_rest.copy()
        self.q_filtered = self.q_rest.copy()
        self._lam_x0 = None

    # -------------------------------------------------------------- readback

    def fingertips(self, q: np.ndarray) -> np.ndarray:
        if self._fk_tips is None:
            self._fk_tips = self.kin.make_fk_function(
                [self.spec.tip_frames[f] for f in FINGERS])
        return np.array(self._fk_tips(q))

    def joint_dict(self, q: np.ndarray) -> dict[str, float]:
        return dict(zip(self.kin.joint_names, map(float, q)))
