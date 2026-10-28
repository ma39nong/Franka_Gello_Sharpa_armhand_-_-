"""Per-hand configuration: URDF paths, joint order, and the human<->robot
keypoint correspondence.

Human keypoint layout is the 25-point Manus raw-skeleton ordering produced by
SharpaManusClient.cpp (wrist first, then thumb 4, then 4 fingers x 5):

     0  wrist
     1  thumb  CMC    2  thumb  MCP    3  thumb  IP    4  thumb  TIP
     5  index  MC     6  index  MCP    7  index  PIP   8  index  DIP   9  index  TIP
    10  middle MC    11  middle MCP   12  middle PIP  13  middle DIP  14  middle TIP
    15  ring   MC    16  ring   MCP   17  ring   PIP  18  ring   DIP  19  ring   TIP
    20  pinky  MC    21  pinky  MCP   22  pinky  PIP  23  pinky  DIP  24  pinky  TIP

The thumb has one fewer node than the other fingers because it has only two
phalanges. Any other source (MediaPipe, WiLoR, ...) just needs its own index
map here -- nothing downstream depends on the layout.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field, replace

import numpy as np

from .hand_kinematics import HandKinematics

FINGERS = ["thumb", "index", "middle", "ring", "pinky"]

# Fingertip contact points, as offsets in each `*_distal` link frame.
#
# The L20 values are HAND-ANNOTATED pad centres (annotations/l20_*.json,
# 2026-08-03): the centre of the palmar face a few mm below the tip apex, so a
# pinch closes pad-to-pad instead of apex-to-apex. The palmar side is +x on
# both the fingers and the thumb -- verified from flexion kinematics, the tip
# curls toward local +x. Do NOT regenerate these from the meshes: an
# axis-cross palmar heuristic picks the dorsal side on the L20, and a mesh
# centroid lands on the tip apex. (The script that did this, calibrate_tips.py,
# was deleted for that reason.) The right hand is the left mirrored across y
# (joint origins mirror exactly).
#
# The O30i values are hand-annotated too, on 2026-08-04 -- see the per-hand
# note on its dict below for the axis convention, which differs from the L20's.
# None of these can come from the URDFs: L20's `*_tip` frames put the thumb
# tip 62 mm from the mesh, behind the distal joint; O30i ships no tip frame at all, and its
# left hand reuses the right hand's un-mirrored thumb meshes, so its left
# thumb offset is mirrored from the right rather than measured.
TIP_OFFSETS = {
    "l20_left": {
        "thumb": (+0.001600, -0.000018, +0.024500),
        "index": (-0.005107, +0.000154, +0.017000),
        "middle": (-0.005107, +0.000154, +0.017000),
        "ring": (-0.005107, +0.000154, +0.017000),
        "pinky": (-0.005107, +0.000154, +0.017000),
    },
    "l20_right": {
        "thumb": (+0.001600, +0.000018, +0.024500),
        "index": (-0.005107, -0.000154, +0.017000),
        "middle": (-0.005107, -0.000154, +0.017000),
        "ring": (-0.005107, -0.000154, +0.017000),
        "pinky": (-0.005107, -0.000154, +0.017000),
    },
    # O30i: hand-annotated pad centres like the L20 (2026-08-04), annotated on
    # the RIGHT hand -- its meshes are the authentic ones (the left reuses the
    # right's thumb mesh un-mirrored) -- and mirrored to the left across y.
    # Fingers share the L20 convention (palmar +x, long axis z); the THUMB
    # does not: its long axis is y and its palmar side is +z, both verified
    # from flexion kinematics.
    "o30i_left": {
        "thumb": (-0.000045, -0.016000, +0.002486),
        "index": (+0.002768, -0.000027, +0.017000),
        "middle": (+0.002768, -0.000027, +0.017000),
        "ring": (+0.002768, -0.000027, +0.017000),
        "pinky": (+0.002768, -0.000027, +0.017000),
    },
    "o30i_right": {
        "thumb": (-0.000045, +0.016000, +0.002486),
        "index": (+0.002768, +0.000027, +0.017000),
        "middle": (+0.002768, +0.000027, +0.017000),
        "ring": (+0.002768, +0.000027, +0.017000),
        "pinky": (+0.002768, +0.000027, +0.017000),
    },
}


# Outward surface normal at each contact point, in the same distal-link frame:
# the direction the pad pushes. Fitted to the mesh around the annotated point
# (local-plane SVD, 3.5 mm radius). Drives the pad-facing term: during a pinch
# the thumb pad normal and the finger pad normal should oppose, or the pads
# meet edge-on -- measured at 90 deg on the replay before this existed, with
# the contact points already touching at 0.7 mm. Only hands with annotated pad
# centres have one; without it the term stays off.
TIP_NORMALS = {
    "l20_left": {
        "thumb": (+0.727, +0.002, +0.687),
        "index": (+0.687, +0.002, +0.727),
        "middle": (+0.687, +0.002, +0.727),
        "ring": (+0.687, +0.002, +0.727),
        "pinky": (+0.687, +0.002, +0.727),
    },
    "l20_right": {
        "thumb": (+0.727, -0.002, +0.687),
        "index": (+0.687, -0.002, +0.727),
        "middle": (+0.687, -0.002, +0.727),
        "ring": (+0.687, -0.002, +0.727),
        "pinky": (+0.687, -0.002, +0.727),
    },
    # O30i thumb normal is z-major -- its palmar side is +z (see TIP_OFFSETS).
    "o30i_right": {
        "thumb": (+0.005, +0.443, +0.896),
        "index": (+0.894, -0.003, +0.447),
        "middle": (+0.894, -0.003, +0.447),
        "ring": (+0.894, -0.003, +0.447),
        "pinky": (+0.894, -0.003, +0.447),
    },
    "o30i_left": {
        "thumb": (+0.005, -0.443, +0.896),
        "index": (+0.894, +0.003, +0.447),
        "middle": (+0.894, +0.003, +0.447),
        "ring": (+0.894, +0.003, +0.447),
        "pinky": (+0.894, +0.003, +0.447),
    },
}


# Contact-point inset, in metres along -normal INTO the link. It existed to
# compensate a 5-10 mm sim-to-real pinch gap whose cause was later found: the
# 0706 URDF's joint limits were wrong, and the tick map inherits them. With
# the 0803 limits (assets URDFs + o30i_contract.py, updated 2026-08-04) the
# metal closes exactly where the model does, so the compensation is retired
# -- kept at 0.0 as the knob for any future measured, deliberate offset.
# Keep this file identical to the linkerhand-retargeting repo's hands.py.
CONTACT_INSET = {
    "o30i_left": 0.0,
    "o30i_right": 0.0,
}


def _tip_frames(name: str) -> dict[str, tuple[str, np.ndarray]]:
    inset = CONTACT_INSET.get(name, 0.0)
    normals = _tip_normals(name)
    return {f: (f"{f}_distal", np.array(o) - inset * normals.get(f, np.zeros(3)))
            for f, o in TIP_OFFSETS[name].items()}


def _tip_normals(name: str) -> dict[str, np.ndarray]:
    raw = TIP_NORMALS.get(name, {})
    return {f: np.asarray(n, dtype=float) / np.linalg.norm(n)
            for f, n in raw.items()}


# Skeleton chains of the 25-point layout: wrist first, then outward along each
# finger. Used for drawing, and to check that a calibration pose is really flat.
HUMAN_CHAINS = {
    "thumb": [0, 1, 2, 3, 4],
    "index": [0, 5, 6, 7, 8, 9],
    "middle": [0, 10, 11, 12, 13, 14],
    "ring": [0, 15, 16, 17, 18, 19],
    "pinky": [0, 20, 21, 22, 23, 24],
}

HUMAN_WRIST = 0
HUMAN_MCP = {"thumb": 2, "index": 6, "middle": 11, "ring": 16, "pinky": 21}
HUMAN_TIP = {"thumb": 4, "index": 9, "middle": 14, "ring": 19, "pinky": 24}
# Last joint before the tip: IP on the thumb, DIP on the fingers.
HUMAN_DIP = {"thumb": 3, "index": 8, "middle": 13, "ring": 18, "pinky": 23}
# Middle joint of each finger; the thumb has none (two phalanges).
HUMAN_PIP = {"index": 7, "middle": 12, "ring": 17, "pinky": 22}

# Which keypoint each human bend angle is measured at. The angle is the turn
# between the incoming and outgoing bone, so 0 means the joint is straight.
HUMAN_BEND_NODE = {
    # The thumb's proximal bend is node 1, not node 2. At node 2 the incoming
    # and outgoing segments are nearly collinear, so the angle there is
    # degenerate: 0-4 deg over a 900-frame recording while node 1 covers 17-61
    # and the node's own quaternion turns through 98. Measuring at node 2 reads
    # as a dead sensor and is not one.
    ("thumb", "mcp"): 1, ("thumb", "ip"): 3,
    ("index", "mcp"): 6, ("index", "pip"): 7, ("index", "dip"): 8,
    ("middle", "mcp"): 11, ("middle", "pip"): 12, ("middle", "dip"): 13,
    ("ring", "mcp"): 16, ("ring", "pip"): 17, ("ring", "dip"): 18,
    ("pinky", "mcp"): 21, ("pinky", "pip"): 22, ("pinky", "dip"): 23,
}


def mirror_keypoints(kp: np.ndarray) -> np.ndarray:
    """The exact left<->right transform for keypoint frames: y := -y, and the
    quaternions conjugated by that reflection (x, z := -x, -z).

    A mirrored left frame IS a valid right frame and vice versa -- sidedness is
    a property of the data, and this is its only well-defined conversion. One
    definition, used by every loader; do not re-derive it inline.
    Accepts (25, 7) or (N, 25, 7).
    """
    kp = np.asarray(kp, dtype=float).copy()
    kp[..., 1] *= -1.0
    kp[..., 4] *= -1.0
    kp[..., 6] *= -1.0
    return kp


def infer_side(kp: np.ndarray) -> str:
    """Which hand a keypoint frame is from, read off its geometry.

    The thumb sits on opposite sides of the palm on the two hands, so the sign
    of cross(across-palm, along-palm) . (thumb - wrist) decides. Used to label
    recordings and profiles that predate the explicit side field; the palm-fit
    residual can NOT catch a swapped side (the fitted point set is near
    mirror-symmetric, so the wrong hand still closes at ~12 mm while every
    target lands 40-150 mm off -- measured).
    """
    p = np.asarray(kp, dtype=float)[..., :3]
    if p.ndim == 3:
        p = p[0]
    w = p[HUMAN_WRIST]
    n = np.cross(p[HUMAN_MCP["index"]] - p[HUMAN_MCP["pinky"]],
                 p[HUMAN_MCP["middle"]] - w)
    s = float(np.dot(n, p[HUMAN_TIP["thumb"]] - w))
    # Sign convention anchored on the recorded left-hand data (replay_left,
    # left_manus, thumb_bend_left): those all measure negative.
    return "left" if s < 0 else "right"


def human_bend_angle(kp: np.ndarray, finger: str, which: str) -> float:
    """Bend angle at one human joint, in radians, from the keypoint triple."""
    n = HUMAN_BEND_NODE[(finger, which)]
    chain = HUMAN_CHAINS[finger]
    i = chain.index(n)
    a, b, c = kp[chain[i - 1]], kp[n], kp[chain[i + 1]]
    u, v = b - a, c - b
    d = np.linalg.norm(u) * np.linalg.norm(v)
    if d < 1e-9:
        return 0.0
    return float(np.arccos(np.clip(np.dot(u, v) / d, -1.0, 1.0)))


@dataclass
class HandSpec:
    name: str
    urdf_path: str
    root_link: str
    joint_names: list[str]
    # finger -> (link, offset in link frame) for the fingertip contact point
    tip_frames: dict[str, tuple[str, np.ndarray]]
    # finger -> link whose origin sits at the MCP joint
    mcp_links: dict[str, str]
    # finger -> link whose origin is the last joint before the tip (DIP, or IP
    # on the thumb). Its frame also carries the fingertip's orientation.
    distal_links: dict[str, str] = field(default_factory=dict)
    # finger -> link whose origin is the middle joint, for PIP-level opposition
    pip_links: dict[str, str] = field(default_factory=dict)
    # finger -> outward pad normal at the contact point, distal-link frame.
    # Empty when the pad has not been annotated; the pad-facing term needs it.
    tip_normals: dict[str, np.ndarray] = field(default_factory=dict)
    # finger -> link the digit's pointing direction is measured from, where it
    # differs from mcp_links. The oracle roots the thumb at CMC_VL and the
    # pinky at its metacarpal, so the directions include CMC opposition and
    # pinky cupping; mcp_links stays the anatomical knuckle for the palm fit.
    dir_root_links: dict[str, str] = field(default_factory=dict)
    # robot thumb joint -> the operator thumb angles it should track. The L20
    # gets one entry because its thumb_dip mimics thumb_mcp; the O30i gets two.
    thumb_angle_map: dict[str, tuple[str, ...]] = field(default_factory=dict)
    # joints summed for the overall-closure term
    flexion_joints: list[str] = field(default_factory=list)
    # every thumb joint, for the thumb acceleration term
    thumb_joints: list[str] = field(default_factory=list)
    # Fingers cannot be closer than they are wide. Measured across the distal
    # meshes, so the anti-crossing term needs no invented clearance.
    min_finger_gap: float = 0.0
    # Virtual wrist in the root frame. The URDF root of these hands sits at the
    # mounting flange, not at the anatomical wrist, so wrist-relative vectors
    # need this correction or they are biased by the whole palm length.
    # Origin every fingertip vector is measured from. hand_base_link is a
    # mounting flange, not an anatomical wrist -- 148 mm from the MCPs on the
    # L20 -- so this cannot be read off the URDF. Left None it is estimated and
    # flagged as provisional everywhere it matters; it is annotation item #1.
    wrist_offset: np.ndarray | None = None
    # Axis roles in the root frame, as (normal, lateral, longitudinal) indices
    # into xyz. Both LinkerHand families point the fingers along +z and spread
    # them along y. Used to scale across-palm distance separately from reach:
    # these palms are wider than a scaled human palm, and forcing the fingertips
    # to the human's lateral spacing pins every abduction joint to its limit.
    palm_axes: tuple[int, int, int] = (0, 1, 2)
    # True when the hand preserves human geometry well enough that operator
    # directions transfer directly -- thumb-relative directions included. The
    # retargeter then keeps the oracle's exact two-sided vector residuals and
    # per-digit weights; when False, the thumb-involved terms fall back to
    # one-sided distances and the geometry terms fade during a pinch (the PORT
    # NOTEs in retargeter.py). Only the SharpaWave qualifies: the L20 thumb
    # points 73-94 deg away from an operator's.
    anthropomorphic: bool = False

    @property
    def side(self) -> str:
        return "left" if self.name.endswith("left") else "right"

    def build(self) -> HandKinematics:
        return HandKinematics(self.urdf_path, self.root_link, list(self.joint_names))



# --------------------------------------------------------------------------
# LinkerHand L20 V10.1  --  16 actuated DOF, 5 mimic joints
#
#   thumb : cmc_roll, cmc_yaw, cmc_pitch, mcp     (thumb_dip = 1.0142 * thumb_mcp)
#   x4    : mcp_roll, mcp_pitch, pip              (*_dip     = 0.7879 * *_pip)
#
# DIP is not a decision variable -- HandKinematics substitutes the mimic
# relation into the FK, so q is 16-dimensional.
# --------------------------------------------------------------------------

# Ported from the linkerhand-retargeting tree, where these URDFs are vendored
# under its own urdf/. Here the models this repository already ships are used
# instead, so there is exactly one copy of each asset and no second source of
# truth about which model a hand is. `wave_*` is dropped: SharpaWave is not
# part of this robot; it exists in the source tree as a reference embodiment.
_REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), *([os.pardir] * 5))
)
_ASSETS = os.path.join(_REPO_ROOT, "assets")

_URDF_PATHS = {
    "l20_left": os.path.join(
        _ASSETS, "linkerhand_l20_v101", "linkerhand_L20_V10.1_left.urdf",
        "linkerhand_L20v10.1_left.urdf"),
    "l20_right": os.path.join(
        _ASSETS, "linkerhand_l20_v101", "linkerhand_L20_V10.1_right.urdf",
        "linkerhand_L20v10.1_right.urdf"),
    "o30i_left": os.path.join(
        _ASSETS, "linkerhand_o30i", "left", "linkerhand_o30i_left.urdf"),
    "o30i_right": os.path.join(
        _ASSETS, "linkerhand_o30i", "right", "linkerhand_o30i_right.urdf"),
}

_L20_JOINTS = [
    "thumb_cmc_roll", "thumb_cmc_yaw", "thumb_cmc_pitch", "thumb_mcp",
    "index_mcp_roll", "index_mcp_pitch", "index_pip",
    "middle_mcp_roll", "middle_mcp_pitch", "middle_pip",
    "ring_mcp_roll", "ring_mcp_pitch", "ring_pip",
    "pinky_mcp_roll", "pinky_mcp_pitch", "pinky_pip",
]


def _l20(side: str) -> HandSpec:
    return HandSpec(
        name=f"linkerhand_L20_v10.1_{side}",
        urdf_path=_URDF_PATHS[f"l20_{side}"],
        root_link="hand_base_link",
        joint_names=list(_L20_JOINTS),
        tip_frames=_tip_frames(f"l20_{side}"),
        tip_normals=_tip_normals(f"l20_{side}"),
        mcp_links={f: f"{f}_proximal" for f in FINGERS},
        distal_links={f: f"{f}_distal" for f in FINGERS},
        pip_links={**{f: f"{f}_middle" for f in FINGERS[1:]},
                   "thumb": "thumb_proximal"},
        thumb_angle_map={"thumb_mcp": ("mcp", "ip")},
        flexion_joints=[f"{f}_mcp_pitch" for f in FINGERS[1:]]
                       + [f"{f}_pip" for f in FINGERS[1:]],
        thumb_joints=["thumb_cmc_roll", "thumb_cmc_yaw", "thumb_cmc_pitch", "thumb_mcp"],
        min_finger_gap=0.0154,
    )


# --------------------------------------------------------------------------
# LinkerHand O30i  --  20 actuated DOF, no mimic
#
#   thumb : cmc_roll, cmc_yaw, mcp, ip     (note: no cmc_pitch, unlike L20)
#   x4    : mcp_roll, mcp_pitch, pip, dip
#
# There is no fingertip frame -- the chain ends at *_distal; see TIP_OFFSETS.
# --------------------------------------------------------------------------

_O30_JOINTS = [
    "thumb_cmc_roll", "thumb_cmc_yaw", "thumb_mcp", "thumb_ip",
    "index_mcp_roll", "index_mcp_pitch", "index_pip", "index_dip",
    "middle_mcp_roll", "middle_mcp_pitch", "middle_pip", "middle_dip",
    "ring_mcp_roll", "ring_mcp_pitch", "ring_pip", "ring_dip",
    "pinky_mcp_roll", "pinky_mcp_pitch", "pinky_pip", "pinky_dip",
]


def _o30(side: str) -> HandSpec:
    return HandSpec(
        name=f"linkerhand_O30i_{side}",
        urdf_path=_URDF_PATHS[f"o30i_{side}"],
        root_link="hand_base_link",
        joint_names=list(_O30_JOINTS),
        tip_frames=_tip_frames(f"o30i_{side}"),
        tip_normals=_tip_normals(f"o30i_{side}"),
        mcp_links={f: f"{f}_proximal" for f in FINGERS},
        distal_links={f: f"{f}_distal" for f in FINGERS},
        pip_links={**{f: f"{f}_middle" for f in FINGERS[1:]},
                   "thumb": "thumb_proximal"},
        thumb_angle_map={"thumb_mcp": ("mcp",), "thumb_ip": ("ip",)},
        flexion_joints=[f"{f}_mcp_pitch" for f in FINGERS[1:]]
                       + [f"{f}_pip" for f in FINGERS[1:]]
                       + [f"{f}_dip" for f in FINGERS[1:]],
        thumb_joints=["thumb_cmc_roll", "thumb_cmc_yaw", "thumb_mcp", "thumb_ip"],
        min_finger_gap=0.0158,
    )


HANDS = {
    "l20_left": _l20("left"),
    "l20_right": _l20("right"),
    "o30i_left": _o30("left"),
    "o30i_right": _o30("right"),
}


def load_annotation(spec: HandSpec, path: str) -> HandSpec:
    """Apply a fingertip-contact annotation from the annotator tool.

    Only the contact points come from the file. Link roles, joint limits and
    mimic ratios stay derived from the URDF: the annotation records the one thing
    that needs a human looking at the hardware.
    """
    import json

    with open(path, encoding="utf-8") as fh:
        data = json.load(fh)
    if data.get("format") != "linkerhand-retarget-annotation/1":
        raise ValueError(f"{path}: not a hand-contact annotation "
                         f"(format={data.get('format')!r})")
    if data.get("keypoint_layout") != "manus25":
        raise ValueError(f"{path}: keypoint layout {data.get('keypoint_layout')!r}, "
                         "expected 'manus25'. The 21-point MANO layout puts every "
                         "finger's tip one joint short.")
    if data["base_link"] != spec.root_link:
        raise ValueError(f"{path}: base_link {data['base_link']!r} != {spec.root_link!r}")
    # base_link and the link names are identical across these hands, so matching
    # those proves nothing -- an L20 annotation loads cleanly onto an O30i and
    # silently moves every fingertip. The URDF path is what actually identifies it.
    if os.path.realpath(data["urdf_path"]) != os.path.realpath(spec.urdf_path):
        raise ValueError(
            f"{path}: annotated against {data['urdf_path']}, but this is "
            f"{spec.urdf_path}")

    tips = dict(spec.tip_frames)
    seen = set()
    for entry in data["fingertips"]:
        f = entry["finger"]
        if f not in FINGERS:
            raise ValueError(f"{path}: unknown finger {f!r}")
        if entry["human_tip_id"] != HUMAN_TIP[f]:
            raise ValueError(f"{path}: {f} maps to keypoint {entry['human_tip_id']}, "
                             f"expected {HUMAN_TIP[f]}")
        tips[f] = (entry["link"], np.array(entry["contact_point"], dtype=float))
        seen.add(f)
    missing = set(FINGERS) - seen
    if missing:
        raise ValueError(f"{path}: no contact point for {sorted(missing)}")

    return replace(spec, tip_frames=tips, name=f"{spec.name}+{data['name']}")


def get(name: str) -> HandSpec:
    if name not in HANDS:
        raise KeyError(f"unknown hand {name!r}; available: {sorted(HANDS)}")
    return HANDS[name]
