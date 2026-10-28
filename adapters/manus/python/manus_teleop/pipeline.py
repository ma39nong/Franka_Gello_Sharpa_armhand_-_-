"""MANUS glove source with the same lifecycle as the PICO hand pipeline.

Bimanual: each side in ``dynamic_sides`` follows its glove through the
native skeleton bridge (left uses full L20-URDF optimization for the G20,
right uses the O30i method); every other side streams its default pose. A glove only delivers frames once its
calibration file exists next to the bridge's calibration directory
(``Calibration_left.mcal`` / ``Calibration_right.mcal``) - an uncalibrated
side simply reports waiting and the rest keeps working.
"""

from __future__ import annotations

import ctypes
import hashlib
import subprocess
import time
from pathlib import Path
from xml.etree import ElementTree

import numpy as np

from pico_bimanual_franka_teleop.hand_sender import HandCommandSender, HandStatus
from pico_bimanual_franka_teleop.hand_stream import HARDWARE_MODELS

KEYPOINT_COUNT = 25
SIDE_CODES = {"left": 1, "right": 2}
CANONICAL_FROM_MANUS = (
    0,
    1, 2, 3, 4,
    6, 7, 8, 9,
    11, 12, 13, 14,
    16, 17, 18, 19,
    21, 22, 23, 24,
)
REPO_ROOT = Path(__file__).resolve().parents[4]

# The physical full-retarget run showed a 17.1 mm p95 MANUS skeleton gap
# during unambiguous thumb-index contact, so the deployed O30i-style distance
# objective uses an outward-rounded 18 mm deadzone. Values are recorded in
# every debug header.
MANUS_LEFT_CONTACT_DEADZONE = 0.018
MANUS_LEFT_DISTANCE_WEIGHT_SCALE = 10.0

# Explicit right-glove endpoint and contact calibration from the labelled
# right_o30_accuracy_20260730_201148 recording, followed by hardware validation.
MANUS_RIGHT_CONTACT_DEADZONE = 0.015
MANUS_RIGHT_DISTANCE_WEIGHT_SCALE = 10.0
MANUS_RIGHT_THUMB_OPEN_BEND_THRESHOLD = np.deg2rad(10.0)
MANUS_RIGHT_FINGER_OPEN_RANGES = {
    "index": (np.deg2rad(20.0), np.deg2rad(40.0)),
    "middle": (np.deg2rad(5.0), np.deg2rad(30.0)),
    "ring": (np.deg2rad(5.0), np.deg2rad(25.0)),
    "pinky": (np.deg2rad(25.0), np.deg2rad(45.0)),
}
MANUS_RIGHT_FINGER_CURL_RANGES = {
    "index": (np.deg2rad(160.0), np.deg2rad(180.0)),
    "middle": (np.deg2rad(155.0), np.deg2rad(175.0)),
    "ring": (np.deg2rad(160.0), np.deg2rad(178.0)),
    "pinky": (np.deg2rad(90.0), np.deg2rad(112.0)),
}
# Nearest O30i model pose within 1 mm of middle-thumb contact while preserving
# the recorded method configuration. Physical contact still gates acceptance.
MANUS_RIGHT_MIDDLE_PINCH_START = 0.040
MANUS_RIGHT_MIDDLE_PINCH_ACTIVATION_STEP = 0.08
MANUS_RIGHT_MIDDLE_PINCH_ANCHOR = {
    "thumb_cmc_roll": 0.6108,
    "thumb_cmc_yaw": 1.387,
    "thumb_mcp": 0.973,
    "thumb_ip": 0.0,
    "middle_mcp_roll": -0.055,
    "middle_mcp_pitch": 1.244,
    "middle_pip": 0.300,
    "middle_dip": 1.176,
}

# Index-middle contact is a separate gesture: both fingers and the thumb stay
# approximately straight, the thumb remains clear, and the two fingertips
# touch. Thresholds are the outward-rounded p95 contact gaps from
# manus_six_pose_bimanual_20260730_215752. The anchors are the nearest
# zero-gap URDF poses to each recorded method pose; physical validation is
# still required, especially for the G20 represented by an L20 URDF.
MANUS_LEFT_INDEX_MIDDLE_CONTACT_DISTANCE = 0.028
MANUS_RIGHT_INDEX_MIDDLE_CONTACT_DISTANCE = 0.024
MANUS_INDEX_MIDDLE_START_DISTANCE = 0.040
MANUS_INDEX_MIDDLE_ACTIVATION_STEP = 0.08
MANUS_LEFT_INDEX_MIDDLE_ANCHOR = {
    # Re-solved against the verified left L20 V10.1 kinematics from the stable
    # portion of manus_six_pose_bimanual_20260730_215752. This is the nearest
    # zero-tip-gap pose to the unanchored median method pose in normalized
    # joint distance; hardware contact remains the acceptance criterion.
    "index_mcp_roll": 0.22690,
    "index_mcp_pitch": 0.28952,
    "index_pip": 0.01731,
    "middle_mcp_roll": -0.02321,
    "middle_mcp_pitch": 0.0,
    "middle_pip": 0.44250,
}
MANUS_RIGHT_INDEX_MIDDLE_ANCHOR = {
    "index_mcp_roll": -0.19600,
    "index_mcp_pitch": 0.04298,
    "index_pip": 0.01072,
    "index_dip": 0.00182,
    "middle_mcp_roll": -0.38000,
    "middle_mcp_pitch": 0.0,
    "middle_pip": 0.06494,
    "middle_dip": 0.04417,
}

# Start blending into the mechanical endpoint only near the labelled left
# full-curl distribution. Values are radians of MANUS segment-chain bend.
MANUS_LEFT_FINGER_CURL_RANGES = {
    "index": (np.deg2rad(140.0), np.deg2rad(165.0)),
    "middle": (np.deg2rad(145.0), np.deg2rad(175.0)),
    "ring": (np.deg2rad(120.0), np.deg2rad(148.0)),
    "pinky": (np.deg2rad(75.0), np.deg2rad(100.0)),
}


class ManusPose(ctypes.Structure):
    _fields_ = [
        ("position_x", ctypes.c_float),
        ("position_y", ctypes.c_float),
        ("position_z", ctypes.c_float),
        ("orientation_x", ctypes.c_float),
        ("orientation_y", ctypes.c_float),
        ("orientation_z", ctypes.c_float),
        ("orientation_w", ctypes.c_float),
    ]


class ManusFrame(ctypes.Structure):
    _fields_ = [
        ("side", ctypes.c_int32),
        ("keypoint_count", ctypes.c_uint32),
        ("timestamp_ns", ctypes.c_uint64),
        ("sequence", ctypes.c_uint64),
        ("keypoints", ManusPose * KEYPOINT_COUNT),
    ]


class ManusBridge:
    """Small ctypes owner for the vendored native MANUS bridge."""

    def __init__(self, library: Path) -> None:
        self.library = ctypes.CDLL(str(library))
        self.library.litchi_manus_bridge_abi_version.restype = ctypes.c_uint32
        self.library.litchi_manus_frame_size.restype = ctypes.c_uint32
        self.library.litchi_manus_connect.argtypes = [
            ctypes.c_uint32,
            ctypes.c_int32,
            ctypes.c_char_p,
        ]
        self.library.litchi_manus_connect.restype = ctypes.c_int32
        self.library.litchi_manus_disconnect.restype = ctypes.c_int32
        self.library.litchi_manus_read_frame.argtypes = [
            ctypes.c_int32,
            ctypes.c_uint32,
            ctypes.POINTER(ManusFrame),
        ]
        self.library.litchi_manus_read_frame.restype = ctypes.c_int32
        self.library.litchi_manus_available_sides.restype = ctypes.c_uint32
        self.library.litchi_manus_last_error.restype = ctypes.c_char_p
        if self.library.litchi_manus_bridge_abi_version() != 1:
            raise RuntimeError("unsupported MANUS skeleton bridge ABI")
        if self.library.litchi_manus_frame_size() != ctypes.sizeof(ManusFrame):
            raise RuntimeError("MANUS frame ABI size mismatch")
        self.connected = False

    def error(self) -> str:
        message = self.library.litchi_manus_last_error()
        return message.decode("utf-8", errors="replace") if message else "unknown error"

    def connect(self, calibration_dir: Path) -> None:
        result = self.library.litchi_manus_connect(
            1, 1, str(calibration_dir).encode("utf-8")
        )
        if result != 0:
            raise RuntimeError(self.error())
        self.connected = True

    def available_sides(self) -> tuple[str, ...]:
        mask = int(self.library.litchi_manus_available_sides())
        return tuple(
            side for side, code in SIDE_CODES.items() if mask & (1 << (code - 1))
        )

    def read(self, side: str, timeout_s: float = 0.0) -> ManusFrame | None:
        code = SIDE_CODES[side]
        frame = ManusFrame()
        result = self.library.litchi_manus_read_frame(
            code,
            max(0, round(float(timeout_s) * 1000)),
            ctypes.byref(frame),
        )
        if result < 0:
            raise RuntimeError(self.error())
        if result == 0:
            return None
        if frame.side != code or frame.keypoint_count != KEYPOINT_COUNT:
            raise RuntimeError(f"invalid {side} MANUS skeleton frame")
        return frame

    def close(self) -> None:
        if self.connected:
            self.library.litchi_manus_disconnect()
            self.connected = False


def frame_record(frame: ManusFrame) -> dict:
    """Return the complete native frame as JSON-safe diagnostic data."""
    return {
        "sequence": int(frame.sequence),
        "timestamp_ns": int(frame.timestamp_ns),
        "keypoints": [
            {
                "position": [
                    float(point.position_x),
                    float(point.position_y),
                    float(point.position_z),
                ],
                "orientation_xyzw": [
                    float(point.orientation_x),
                    float(point.orientation_y),
                    float(point.orientation_z),
                    float(point.orientation_w),
                ],
            }
            for point in frame.keypoints
        ],
    }


def canonical_landmarks(frame: ManusFrame) -> np.ndarray:
    points = np.asarray(
        [
            (point.position_x, point.position_y, point.position_z)
            for point in frame.keypoints
        ],
        dtype=np.float64,
    )
    selected = points[list(CANONICAL_FROM_MANUS)]
    if selected.shape != (21, 3) or not np.isfinite(selected).all():
        raise ValueError("MANUS skeleton contains invalid landmarks")
    return selected


def raw_keypoints(frame: ManusFrame) -> np.ndarray:
    """The full 25-node skeleton, position and orientation, as (25, 7).

    ``canonical_landmarks`` selects 21 of the 25 nodes and drops the
    orientations. The CasADi retargeter needs them: four of its twelve cost
    terms are orientation terms, and without them the fingertip roll and the
    per-phalanx directions are unconstrained.

    Quaternions come out w,x,y,z -- the order that retargeter reads. The SDK
    struct stores x,y,z,w.
    """
    out = np.empty((KEYPOINT_COUNT, 7), dtype=np.float64)
    for index, point in enumerate(frame.keypoints):
        out[index, 0] = point.position_x
        out[index, 1] = point.position_y
        out[index, 2] = point.position_z
        out[index, 3] = point.orientation_w
        out[index, 4] = point.orientation_x
        out[index, 5] = point.orientation_y
        out[index, 6] = point.orientation_z
    if not np.isfinite(out).all():
        raise ValueError("MANUS skeleton contains invalid keypoints")
    return out


# Operator calibration for the CasADi retargeter. It has no defaults -- hand
# dimensions, joint ranges and the operator-to-robot frame all come from two
# recorded poses -- so the file is named here rather than discovered, and a
# missing one is an error at construction instead of a silently untracked hand.
# Operator calibration for the sharpa method, PER SIDE.
#
# The loader can mirror a left capture onto a right hand, and this used to lean
# on that: one constant served both sides. That is geometrically valid and
# anatomically wrong -- a mirrored left hand is not this operator's right hand,
# and everything the cost function knows about them (finger lengths, reach,
# fingertip separations, the wrist-frame alignment) comes from the capture. Use
# the capture of the side being driven; mirroring is the fallback for when one
# does not exist, not the arrangement.
SHARPA_PROFILES = {
    "left": (
        REPO_ROOT
        / "config"
        / "calibration"
        / "hand_profiles"
        / "left_manus_gui.json"
    ),
    "right": (
        REPO_ROOT
        / "config"
        / "calibration"
        / "hand_profiles"
        / "right_manus_gui.json"
    ),
}

# Which physical hand is on each side. A FACT about the robot -- it changes only
# when hardware is re-cabled -- and it must agree with the bridge's own
# configuration in docker/compose.yaml (left_model / right_model), because that
# is what the packet's model tag is checked against. Two configs, one meaning;
# tests/test_manus_pipeline.py pins them together so they cannot drift apart.
# Both mounts are O30i since 2026-08-04 (the left G20 was swapped out).
DEFAULT_HANDS = {"left": "o30i", "right": "o30i"}

# Which retargeting algorithm runs on each side. A CHOICE, orthogonal to the
# hardware: both drive the same physical hand and emit the same packet layout,
# so the method name must never reach the wire. Conflating the two is what made
# every ported-objective packet get dropped by the bridge as a model mismatch
# while the sender reported a healthy send rate.
#
#   landmark -- the canonical-landmark solve these hands started with: match
#               the 21 landmarks and per-segment directions, with hand-tuned
#               contact deadzones and pinch anchors per hand.
#   sharpa   -- the twelve-term objective ported from the SharpaWave
#               optimiser, with the operator mapping derived from a
#               calibration capture.
#
# Named for the algorithm, not for the numerical library each happens to use:
# both are optimisations, and "pinocchio vs casadi" said nothing about what
# actually differs between them.
METHODS = ("landmark", "sharpa")
DEFAULT_METHODS = {"left": "sharpa", "right": "sharpa"}

# Combined names from before the two concepts were separated. One decoder,
# shared by the deprecated CLI flags and by readers of older debug logs.
LEGACY_MODELS = {
    "g20": ("g20", "landmark"),
    "g20_casadi": ("g20", "sharpa"),
    "o30i": ("o30i", "landmark"),
    "o30i_casadi": ("o30i", "sharpa"),
}


def split_legacy_model(model: str) -> tuple[str, str]:
    """Decode a legacy combined name into (hardware model, method)."""
    try:
        return LEGACY_MODELS[str(model).strip().lower()]
    except KeyError:
        raise ValueError(
            f"unknown legacy hand model {model!r}; "
            f"expected one of {sorted(LEGACY_MODELS)}"
        ) from None


def _resolve_side_map(given, defaults, label, allowed, sides):
    """Normalise a per-side configuration map and reject unknown values."""
    if given is not None and set(given) != set(sides):
        raise ValueError(f"{label} must define exactly {' and '.join(sides)}")
    resolved = (
        dict(defaults)
        if given is None
        else {side: str(given[side]).strip().lower() for side in sides}
    )
    unsupported = {
        side: value for side, value in resolved.items() if value not in allowed
    }
    if unsupported:
        raise ValueError(
            f"unsupported {label}: {unsupported}; allowed: {list(allowed)}"
        )
    return resolved


def _create_retargeter(side: str, hand: str, method: str, filter_alpha: float):
    """Build one side's retargeter from the hardware it drives and the method.

    Every (hand, method) pair is supported. The method changes which algorithm
    runs and nothing the bridge can observe: both emit the same joint names in
    the same packet layout for a given hand.
    """
    if method not in METHODS:
        raise ValueError(
            f"method must be one of {list(METHODS)}, got {method!r}")
    if method == "sharpa":
        from .casadi_retarget import CasadiHandRetargeter

        if hand == "g20":
            return CasadiHandRetargeter(side, SHARPA_PROFILES[side])
        # The O30i packet follows the URDF joint order exactly as
        # O30IRetargeter derives it (pinocchio idx_q order), so both right-hand
        # methods emit interchangeable packets; a name mismatch fails at
        # construction, not per frame.
        import pinocchio as pin  # noqa: PLC0415

        urdf = (REPO_ROOT / "assets" / "linkerhand_o30i" / side
                / f"linkerhand_o30i_{side}.urdf")
        pin_model = pin.buildModelFromUrdf(str(urdf))
        packet_joint_names = tuple(
            name for _, name in sorted(
                (pin_model.joints[j].idx_q, pin_model.names[j])
                for j in range(1, pin_model.njoints)))
        return CasadiHandRetargeter(
            side, SHARPA_PROFILES[side], hand=f"o30i_{side}",
            packet_joint_names=packet_joint_names)
    if hand == "o30i":
        from .o30i_retarget import O30IRetargeter

        return O30IRetargeter(
            REPO_ROOT
            / "assets"
            / "linkerhand_o30i"
            / side
            / f"linkerhand_o30i_{side}.urdf",
            side,
            filter_alpha=filter_alpha,
            contact_deadzone=MANUS_RIGHT_CONTACT_DEADZONE,
            distance_weight_scale=MANUS_RIGHT_DISTANCE_WEIGHT_SCALE,
            finger_open_ranges=MANUS_RIGHT_FINGER_OPEN_RANGES,
            finger_curl_ranges=MANUS_RIGHT_FINGER_CURL_RANGES,
            thumb_open_bend_threshold=(
                MANUS_RIGHT_THUMB_OPEN_BEND_THRESHOLD
            ),
            middle_pinch_anchor=MANUS_RIGHT_MIDDLE_PINCH_ANCHOR,
            middle_pinch_start=MANUS_RIGHT_MIDDLE_PINCH_START,
            middle_pinch_activation_step=(
                MANUS_RIGHT_MIDDLE_PINCH_ACTIVATION_STEP
            ),
            index_middle_pinch_anchor=MANUS_RIGHT_INDEX_MIDDLE_ANCHOR,
            index_middle_contact_distance=(
                MANUS_RIGHT_INDEX_MIDDLE_CONTACT_DISTANCE
            ),
            index_middle_start_distance=MANUS_INDEX_MIDDLE_START_DISTANCE,
            index_middle_activation_step=MANUS_INDEX_MIDDLE_ACTIVATION_STEP,
        )
    # The left G20 uses the vendor L20 URDF and the O30i-style full thumb
    # solve: yaw, roll, pitch, and the coupled MCP/IP actuator are optimized
    # from every MANUS frame. A fallback right-G20 profile retains the
    # established fixed opposition.
    from pico_bimanual_franka_teleop.hand_profiles import g20_urdf_path
    from pico_bimanual_franka_teleop.hand_retarget import (
        L20Retargeter,
        THUMB_OPPOSITION_YAW_ROLL,
    )

    calibrated_left = side == "left"
    return L20Retargeter(
        g20_urdf_path(REPO_ROOT / "assets", side),
        side,
        filter_alpha=filter_alpha,
        thumb_opposition_fixed=(
            None if calibrated_left else THUMB_OPPOSITION_YAW_ROLL[side]
        ),
        thumb_contact_deadzone=(
            MANUS_LEFT_CONTACT_DEADZONE if calibrated_left else 0.0
        ),
        thumb_distance_weight_scale=(
            MANUS_LEFT_DISTANCE_WEIGHT_SCALE if calibrated_left else 1.0
        ),
        finger_curl_ranges=(
            MANUS_LEFT_FINGER_CURL_RANGES if side == "left" else None
        ),
        solve_thumb_flex=calibrated_left,
        index_middle_pinch_anchor=(
            MANUS_LEFT_INDEX_MIDDLE_ANCHOR if calibrated_left else None
        ),
        index_middle_contact_distance=(
            MANUS_LEFT_INDEX_MIDDLE_CONTACT_DISTANCE if calibrated_left else 0.0
        ),
        index_middle_start_distance=(
            MANUS_INDEX_MIDDLE_START_DISTANCE if calibrated_left else 0.0
        ),
        index_middle_activation_step=MANUS_INDEX_MIDDLE_ACTIVATION_STEP,
    )


def _default_joint_names(side: str) -> tuple[str, ...]:
    from pico_bimanual_franka_teleop.hand_profiles import g20_urdf_path
    from pico_bimanual_franka_teleop.hand_retarget import (
        LEFT_G20_PACKET_JOINT_NAMES,
    )

    if side == "left":
        return LEFT_G20_PACKET_JOINT_NAMES
    root = ElementTree.parse(g20_urdf_path(REPO_ROOT / "assets", side)).getroot()
    return tuple(
        element.attrib["name"]
        for element in root.findall("joint")
        if element.attrib.get("type") in {"revolute", "continuous"}
    )


class ManusHandPipeline:
    """Bimanual MANUS source: dynamic sides follow their gloves, the rest
    stream their default pose. At most one retargeting solve per tick."""

    sides = ("left", "right")

    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        port: int = 5570,
        rate: float = 30.0,
        stale_timeout: float = 0.25,
        filter_alpha: float = 0.85,
        library: Path | None = None,
        calibration_dir: Path | None = None,
        debug_log: str | Path | None = None,
        hands: dict[str, str] | None = None,
        methods: dict[str, str] | None = None,
        dynamic_sides: tuple[str, ...] = ("left", "right"),
        bridge_factory=ManusBridge,
    ) -> None:
        if stale_timeout <= 0.0:
            raise ValueError("MANUS stale timeout must be positive")
        if not dynamic_sides or set(dynamic_sides).difference(self.sides):
            raise ValueError(f"Invalid dynamic sides: {dynamic_sides}")
        self.dynamic_sides = tuple(
            side for side in self.sides if side in dynamic_sides
        )
        # Hardware identity and method choice are configured separately because
        # they are different kinds of thing: see DEFAULT_HANDS/DEFAULT_METHODS.
        # Only the hardware reaches the wire.
        hands = _resolve_side_map(
            hands, DEFAULT_HANDS, "hands", HARDWARE_MODELS, self.sides
        )
        methods = _resolve_side_map(
            methods, DEFAULT_METHODS, "methods", METHODS, self.sides
        )
        self.hands = dict(hands)
        self.methods = dict(methods)
        self.filter_alpha = float(filter_alpha)
        self.stale_timeout = float(stale_timeout)
        self.status = HandStatus()
        self.sender = HandCommandSender(
            host=host,
            port=port,
            rate=rate,
            sides=self.sides,
            models=hands,
            status=self.status,
        )
        self.retargeters = {}
        try:
            for side in self.dynamic_sides:
                self.retargeters[side] = _create_retargeter(
                    side, hands[side], methods[side], filter_alpha
                )
        except BaseException:
            for retargeter in self.retargeters.values():
                retargeter.close()
            self.sender.close()
            raise
        library_path = (
            Path(library)
            if library is not None
            else REPO_ROOT
            / "adapters"
            / "manus"
            / "build"
            / "libmanus_skeleton_bridge.so"
        )
        calibration_path = (
            Path(calibration_dir)
            if calibration_dir is not None
            else REPO_ROOT / "adapters" / "manus" / "config"
        )
        try:
            self.bridge = bridge_factory(library_path.resolve())
            self.bridge.connect(calibration_path.resolve())
        except BaseException:
            for retargeter in self.retargeters.values():
                retargeter.close()
            self.sender.close()
            raise
        self.default_joint_names = {
            side: _default_joint_names(side)
            for side in self.sides
            if side not in self.dynamic_sides
        }
        self.open_until = {side: 0.0 for side in self.dynamic_sides}
        self.last_frame: dict[str, ManusFrame | None] = {
            side: None for side in self.dynamic_sides
        }
        self.last_frame_at: dict[str, float | None] = {
            side: None for side in self.dynamic_sides
        }
        self.was_following = {side: False for side in self.dynamic_sides}
        # Round-robin start point, so one side cannot starve the other when
        # both come due on the same tick (same rule as the PICO pipeline).
        self._preferred = 0
        self.debug_logger = None
        self.debug_phase: str | None = None
        if debug_log is not None:
            from pico_bimanual_franka_teleop.debug_log import HandRetargetDebugLogger

            calibration_hashes = {}
            for side in self.dynamic_sides:
                path = calibration_path / f"Calibration_{side}.mcal"
                calibration_hashes[side] = (
                    hashlib.sha256(path.read_bytes()).hexdigest()
                    if path.is_file()
                    else None
                )
            try:
                revision = subprocess.run(
                    ["git", "rev-parse", "HEAD"],
                    cwd=REPO_ROOT,
                    check=True,
                    capture_output=True,
                    text=True,
                ).stdout.strip()
                dirty = bool(
                    subprocess.run(
                        ["git", "status", "--porcelain"],
                        cwd=REPO_ROOT,
                        check=True,
                        capture_output=True,
                        text=True,
                    ).stdout
                )
            except (OSError, subprocess.SubprocessError):
                revision = None
                dirty = None
            self.debug_logger = HandRetargetDebugLogger(
                debug_log,
                metadata={
                    "source": "manus",
                    # Recorded separately since they are separate facts. Logs
                    # written before this split carry a single combined
                    # "models" key; readers decode those with
                    # pipeline.split_legacy_model.
                    "hands": self.hands,
                    "methods": self.methods,
                    "dynamic_sides": list(self.dynamic_sides),
                    "filter_alpha": self.filter_alpha,
                    # Derived, never hardcoded: a log whose metadata describes a
                    # retargeter that did not produce it is worse than no
                    # metadata, because it reads as evidence.
                    "left_thumb_policy": (
                        "sharpa_12_term" if self.methods.get("left") == "sharpa"
                        else "o30i_style_full_l20_ik"
                    ),
                    "left_retargeter": type(
                        self.retargeters["left"]).__name__
                    if "left" in self.retargeters else None,
                    "right_hand_calibration": (
                        {
                            "contact_deadzone_m": MANUS_RIGHT_CONTACT_DEADZONE,
                            "distance_weight_scale": (
                                MANUS_RIGHT_DISTANCE_WEIGHT_SCALE
                            ),
                            "thumb_open_bend_threshold_rad": (
                                MANUS_RIGHT_THUMB_OPEN_BEND_THRESHOLD
                            ),
                            "finger_open_ranges_rad": {
                                finger: list(values)
                                for finger, values in MANUS_RIGHT_FINGER_OPEN_RANGES.items()
                            },
                            "finger_curl_ranges_rad": {
                                finger: list(values)
                                for finger, values in MANUS_RIGHT_FINGER_CURL_RANGES.items()
                            },
                            "middle_pinch_start_m": MANUS_RIGHT_MIDDLE_PINCH_START,
                            "middle_pinch_activation_step": (
                                MANUS_RIGHT_MIDDLE_PINCH_ACTIVATION_STEP
                            ),
                            "middle_pinch_anchor": MANUS_RIGHT_MIDDLE_PINCH_ANCHOR,
                            "index_middle_contact_distance_m": (
                                MANUS_RIGHT_INDEX_MIDDLE_CONTACT_DISTANCE
                            ),
                            "index_middle_start_distance_m": (
                                MANUS_INDEX_MIDDLE_START_DISTANCE
                            ),
                            "index_middle_activation_step": (
                                MANUS_INDEX_MIDDLE_ACTIVATION_STEP
                            ),
                            "index_middle_anchor": MANUS_RIGHT_INDEX_MIDDLE_ANCHOR,
                            "source_recording": (
                                "right_o30_accuracy_20260730_201148.jsonl"
                            ),
                            "index_middle_source_recording": (
                                "manus_six_pose_bimanual_20260730_215752.jsonl"
                            ),
                            "index_middle_physical_validation": "pending",
                            "physical_validation": "2026-07-30",
                        }
                        if "right" in self.dynamic_sides
                        else None
                    ),
                    "left_hand_calibration": (
                        dict(getattr(self.retargeters.get("left"), "profile", {}) or {})
                        if self.methods.get("left") == "sharpa"
                        else {
                            "thumb_mode": "o30i_style_full_l20_ik",
                            "thumb_model": "assets/linkerhand_l20_v101/linkerhand_L20_V10.1_left.urdf/linkerhand_L20v10.1_left.urdf",
                            "thumb_contact_deadzone_m": MANUS_LEFT_CONTACT_DEADZONE,
                            "thumb_trust_region_rad_per_tick": 0.35,
                            "solve_thumb_flex": True,
                            "thumb_distance_weight_scale": (
                                MANUS_LEFT_DISTANCE_WEIGHT_SCALE
                            ),
                            "finger_curl_ranges_rad": {
                                finger: list(values)
                                for finger, values in MANUS_LEFT_FINGER_CURL_RANGES.items()
                            },
                            "index_middle_contact_distance_m": (
                                MANUS_LEFT_INDEX_MIDDLE_CONTACT_DISTANCE
                            ),
                            "index_middle_start_distance_m": (
                                MANUS_INDEX_MIDDLE_START_DISTANCE
                            ),
                            "index_middle_activation_step": (
                                MANUS_INDEX_MIDDLE_ACTIVATION_STEP
                            ),
                            "index_middle_anchor": MANUS_LEFT_INDEX_MIDDLE_ANCHOR,
                            "source_recording": "left_full_01.jsonl",
                            "pinch_source_recording": (
                                "left_full_retarget_physical.jsonl"
                            ),
                            "index_middle_source_recording": (
                                "manus_six_pose_bimanual_20260730_215752.jsonl"
                            ),
                            "index_middle_physical_validation": "pending",
                            "physical_pinch_calibration": "2026-07-30",
                        }
                        if "left" in self.dynamic_sides
                        else None
                    ),
                    "joint_contracts": {
                        side: {
                            "joint_names": list(self.retargeters[side].joint_names),
                            "lower": np.asarray(
                                self.retargeters[side].lower, dtype=float
                            ).tolist(),
                            "upper": np.asarray(
                                self.retargeters[side].upper, dtype=float
                            ).tolist(),
                        }
                        for side in self.dynamic_sides
                    },
                    "calibration_sha256": calibration_hashes,
                    "git_revision": revision,
                    "git_dirty": dirty,
                },
            )

    def set_debug_phase(self, phase: str | None) -> None:
        """Label subsequent diagnostic rows with a guided protocol phase."""
        self.debug_phase = None if phase is None else str(phase)

    def request_open(
        self,
        now: float | None = None,
        duration: float = 2.0,
        sides: tuple[str, ...] | None = None,
    ) -> None:
        if duration <= 0.0:
            raise ValueError("Open duration must be positive")
        moment = time.monotonic() if now is None else float(now)
        selected = self.dynamic_sides if sides is None else sides
        for side in selected:
            if side in self.open_until:
                self.open_until[side] = moment + float(duration)

    def _read_side(self, side: str, moment: float) -> bool:
        """Pull the freshest frame for one side; False when the source broke."""
        status = self.status.sides[side]
        try:
            frame = self.bridge.read(side, 0.0)
        except Exception as error:  # noqa: BLE001 - contain source failure
            self.status.errors += 1
            self.status.last_error = f"{side} MANUS read failed: {error}"
            status.sending = False
            status.fault = self.status.last_error
            return False
        if frame is not None:
            self.last_frame[side] = frame
            self.last_frame_at[side] = moment
        return True

    def tick(
        self,
        now: float | None = None,
        active: dict[str, bool] | None = None,
    ) -> None:
        moment = time.monotonic() if now is None else float(now)
        for side, joint_names in self.default_joint_names.items():
            if self.sender.due(side, moment):
                self.sender.emit(
                    f"manus-{side}-default",
                    side,
                    joint_names,
                    [0.0] * len(joint_names),
                    moment,
                    f"{side} default command failed",
                )

        following = {}
        for side in self.dynamic_sides:
            if not self._read_side(side, moment):
                following[side] = False
                self.was_following[side] = False
                continue
            allowed = active is None or bool(active.get(side, False))
            fresh = (
                self.last_frame[side] is not None
                and self.last_frame_at[side] is not None
                and moment - self.last_frame_at[side] <= self.stale_timeout
            )
            follows = allowed and fresh
            if follows:
                self.open_until[side] = 0.0
            elif self.was_following[side]:
                self.retargeters[side].reset()
            following[side] = follows
            self.was_following[side] = follows

            if follows:
                continue
            status = self.status.sides[side]
            if (
                not allowed
                and moment < self.open_until[side]
                and self.sender.due(side, moment)
            ):
                names = self.retargeters[side].joint_names
                self.sender.emit(
                    f"manus-{side}-open",
                    side,
                    names,
                    [0.0] * len(names),
                    moment,
                    f"{side} MANUS open command failed",
                )
                continue
            status.sending = False
            if not allowed:
                status.fault = "disengaged by operator"
            elif self.last_frame[side] is None:
                status.fault = (
                    f"waiting for calibrated {side} MANUS glove "
                    f"(Calibration_{side}.mcal present?)"
                )
            else:
                status.fault = f"{side} MANUS frame is stale"

        # Solve at most one side per tick so a tick never costs two solves.
        order = [
            self.dynamic_sides[
                (self._preferred + offset) % len(self.dynamic_sides)
            ]
            for offset in range(len(self.dynamic_sides))
        ]
        for side in order:
            if not following.get(side) or not self.sender.due(side, moment):
                continue
            status = self.status.sides[side]
            try:
                started = time.monotonic()
                landmarks = canonical_landmarks(self.last_frame[side])
                retargeter = self.retargeters[side]
                # The canonical array stays the logged one either way, so
                # recordings and the replay tooling keep one schema.
                method_input = (
                    raw_keypoints(self.last_frame[side])
                    if getattr(retargeter, "wants_raw_keypoints", False)
                    else landmarks
                )
                qpos, stats = retargeter.retarget(method_input)
                elapsed = time.monotonic() - started
                raw_qpos = qpos
                if hasattr(retargeter, "last_qpos"):
                    raw_qpos = retargeter.last_qpos
                    if hasattr(retargeter, "_expand_qpos"):
                        raw_qpos = retargeter._expand_qpos(raw_qpos)
                derived = None
                if hasattr(retargeter, "target_positions") and hasattr(
                    retargeter, "robot_landmarks"
                ):
                    targets = retargeter.target_positions(landmarks)
                    if np.asarray(targets).shape != (21, 3):
                        canonical_targets = np.zeros((21, 3), dtype=np.float64)
                        for target, point in zip(
                            retargeter.targets, targets, strict=True
                        ):
                            canonical_targets[target.landmark_index] = point
                        targets = canonical_targets
                    # Only O30IRetargeter takes the pose; L20Retargeter leaves
                    # its FK state at the emitted pose. CasADi retargeters
                    # expose neither and never reach this branch.
                    if (self.hands[side] == "o30i"
                            and self.methods[side] == "landmark"):
                        robot_points = retargeter.robot_landmarks(qpos)
                    else:
                        # L20 leaves its FK state at the filtered/emitted pose.
                        robot_points = retargeter.robot_landmarks()
                    derived = {
                        "target_landmarks_robot": np.asarray(targets)
                        .round(7)
                        .tolist(),
                        "robot_landmarks_emitted": np.asarray(robot_points)
                        .round(7)
                        .tolist(),
                    }
            except Exception as error:  # noqa: BLE001 - never break arm control
                self.status.errors += 1
                self.status.last_error = f"{side} MANUS retargeting failed: {error}"
                status.sending = False
                status.fault = str(error)
                return
            # Free-form: the bridge uses stream_id only to notice that the
            # sequence counter restarted, so it can carry the method as
            # provenance. Bridge logs quote it as "{stream_id}:{sequence}",
            # which is then enough to tell which method drove the hand.
            stream_id = f"manus-{side}-{self.hands[side]}-{self.methods[side]}"
            packet_sequence = self.sender.next_sequence(side)
            sent = self.sender.emit(
                stream_id,
                side,
                self.retargeters[side].joint_names,
                qpos,
                moment,
                f"{side} MANUS send failed",
            )
            if self.debug_logger is not None:
                self.debug_logger.record(
                    moment,
                    side,
                    landmarks,
                    qpos,
                    stats,
                    joint_names=self.retargeters[side].joint_names,
                    raw_qpos=raw_qpos,
                    source=frame_record(self.last_frame[side]),
                    transport={
                        "stream_id": stream_id,
                        "sequence": packet_sequence,
                        "sent": sent,
                        "phase": self.debug_phase,
                    },
                    derived=derived,
                )
            if sent:
                self._preferred = (
                    self.dynamic_sides.index(side) + 1
                ) % len(self.dynamic_sides)
                status.solve_seconds = elapsed
            return

    def close(self) -> None:
        try:
            if self.debug_logger is not None:
                self.debug_logger.close()
        finally:
            try:
                self.sender.close()
            finally:
                try:
                    for retargeter in self.retargeters.values():
                        retargeter.close()
                finally:
                    self.bridge.close()
