"""URDF -> CasADi symbolic forward kinematics, with mimic-joint support.

Transforms are carried as (R, p) pairs rather than 4x4 homogeneous matrices --
CasADi builds a smaller expression graph that way, which matters because the
whole chain gets differentiated on every solver build.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import casadi as ca
import numpy as np


def rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _axis_rotation(axis, q):
    """Rodrigues rotation about `axis` by symbolic angle `q`."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([
        [0.0, -a[2], a[1]],
        [a[2], 0.0, -a[0]],
        [-a[1], a[0], 0.0],
    ])
    return ca.DM(np.eye(3)) + ca.sin(q) * ca.DM(K) + (1.0 - ca.cos(q)) * ca.DM(K @ K)


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray | None
    lower: float | None
    upper: float | None
    mimic: tuple[str, float, float] | None = None  # (source joint, multiplier, offset)

    @property
    def is_actuated(self) -> bool:
        return self.type in ("revolute", "continuous", "prismatic") and self.mimic is None


@dataclass
class HandKinematics:
    """Symbolic FK over a URDF.

    `joint_names` fixes the order of the decision vector q. Mimic joints are
    substituted symbolically, so they never enter q.
    """

    urdf_path: str
    root_link: str
    joint_names: list[str] = field(default_factory=list)

    def __post_init__(self):
        self.joints: dict[str, Joint] = {}
        self.parent_joint: dict[str, Joint] = {}  # child link -> joint
        self._parse()

        if not self.joint_names:
            self.joint_names = [j.name for j in self.joints.values() if j.is_actuated]

        missing = [n for n in self.joint_names if n not in self.joints]
        if missing:
            raise ValueError(f"joints not present in URDF: {missing}")
        self.index_of = {n: i for i, n in enumerate(self.joint_names)}
        self.n_dof = len(self.joint_names)

    # ------------------------------------------------------------------ parse

    def _parse(self):
        root = ET.parse(self.urdf_path).getroot()
        for j in root.findall("joint"):
            origin = j.find("origin")
            xyz = np.zeros(3)
            rpy = np.zeros(3)
            if origin is not None:
                if origin.get("xyz"):
                    xyz = np.array([float(v) for v in origin.get("xyz").split()])
                if origin.get("rpy"):
                    rpy = np.array([float(v) for v in origin.get("rpy").split()])

            axis_el = j.find("axis")
            axis = None
            if axis_el is not None and axis_el.get("xyz"):
                axis = np.array([float(v) for v in axis_el.get("xyz").split()])

            lim = j.find("limit")
            lo = hi = None
            if lim is not None and lim.get("lower") is not None:
                lo, hi = float(lim.get("lower")), float(lim.get("upper"))

            mim_el = j.find("mimic")
            mimic = None
            if mim_el is not None:
                mimic = (
                    mim_el.get("joint"),
                    float(mim_el.get("multiplier") or 1.0),
                    float(mim_el.get("offset") or 0.0),
                )

            joint = Joint(
                name=j.get("name"),
                type=j.get("type"),
                parent=j.find("parent").get("link"),
                child=j.find("child").get("link"),
                origin_xyz=xyz,
                origin_rpy=rpy,
                axis=axis,
                lower=lo,
                upper=hi,
                mimic=mimic,
            )
            self.joints[joint.name] = joint
            self.parent_joint[joint.child] = joint

    # ------------------------------------------------------------------ limits

    def joint_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """Bounds on q, tightened so that every mimic child also stays in range.

        A mimic joint has its own <limit> in the URDF, and it is not always
        consistent with multiplier * parent_range. Ignoring that lets the solver
        command a DIP angle the hardware will clip, which shows up as the
        fingertip silently missing its target.
        """
        lo = np.full(self.n_dof, -np.pi)
        hi = np.full(self.n_dof, np.pi)
        for name, idx in self.index_of.items():
            j = self.joints[name]
            if j.lower is not None:
                lo[idx], hi[idx] = j.lower, j.upper

        for j in self.joints.values():
            if j.mimic is None or j.lower is None:
                continue
            src, mult, off = j.mimic
            if src not in self.index_of or mult == 0.0:
                continue
            i = self.index_of[src]
            a, b = (j.lower - off) / mult, (j.upper - off) / mult
            if mult < 0:
                a, b = b, a
            lo[i], hi[i] = max(lo[i], a), min(hi[i], b)
        return lo, hi

    def rest_pose(self) -> np.ndarray:
        """Zero clamped into bounds -- these hands rest at q=0 (fully open)."""
        lo, hi = self.joint_bounds()
        return np.clip(np.zeros(self.n_dof), lo, hi)

    # ------------------------------------------------------------------ FK

    def _joint_value(self, joint: Joint, q):
        if joint.mimic is not None:
            src, mult, off = joint.mimic
            return mult * q[self.index_of[src]] + off
        return q[self.index_of[joint.name]]

    def _chain(self, link: str) -> list[Joint]:
        chain, cur = [], link
        while cur != self.root_link:
            if cur not in self.parent_joint:
                raise ValueError(f"link {link!r} does not connect to root {self.root_link!r}")
            j = self.parent_joint[cur]
            chain.append(j)
            cur = j.parent
        return list(reversed(chain))

    def link_pose(self, link: str, q):
        """Symbolic (R, p) of `link` in the root frame."""
        R = ca.DM(np.eye(3))
        p = ca.DM(np.zeros(3))
        for j in self._chain(link):
            R_o = ca.DM(rpy_to_matrix(j.origin_rpy))
            p = p + R @ ca.DM(j.origin_xyz)
            R = R @ R_o
            if j.type in ("revolute", "continuous"):
                R = R @ _axis_rotation(j.axis, self._joint_value(j, q))
            elif j.type == "prismatic":
                p = p + R @ (ca.DM(j.axis) * self._joint_value(j, q))
        return R, p

    def point_position(self, link: str, offset, q):
        """Symbolic position of a point rigidly attached to `link`."""
        R, p = self.link_pose(link, q)
        return p + R @ ca.DM(np.asarray(offset, dtype=float))

    def make_fk_function(self, targets: list[tuple[str, np.ndarray]]) -> ca.Function:
        """Compile a numeric FK for (link, offset) pairs -- used for calibration
        and visualisation, not inside the solver."""
        q = ca.MX.sym("q", self.n_dof)
        pts = [self.point_position(link, off, q) for link, off in targets]
        return ca.Function("fk", [q], [ca.horzcat(*pts).T])
