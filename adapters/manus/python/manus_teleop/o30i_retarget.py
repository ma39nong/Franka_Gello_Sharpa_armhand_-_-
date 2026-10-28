"""MANUS canonical-landmark retargeting for the right Linker O30i."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.optimize import minimize

from pico_bimanual_franka_teleop.hand_landmarks import (
    CANONICAL_LANDMARK_COUNT,
    finger_base_centroid,
    palm_scale,
)
from pico_bimanual_franka_teleop.hand_retarget import (
    CANONICAL_FINGERS,
    CHAIN_WEIGHTS,
    chain_bend_angle,
    index_middle_pinch_request,
    THUMB_DISTANCE_THRESHOLD,
    THUMB_DISTANCE_WEIGHTS,
    orthonormal_palm_frame,
)

# Per-segment direction objectives, applied to every finger. Pure position
# matching reproduced the human's inter-segment bend but under-flexed the MCP
# knuckle against the palm by 15-19 degrees at full curl (measured on the
# 2026-07-29 session): the solver dumped curl into the saturated distal
# joints. Matching each segment's DIRECTION pins the joint distribution to
# the human's, which position residuals alone leave underdetermined.
SEGMENT_DIRECTION_WEIGHTS = (1.0, 1.0, 0.9)

_TARGET_LINKS = {
    "thumb": (
        "thumb_metacarpals_base1",
        "thumb_metacarpals_base2",
        "thumb_proximal",
    ),
    "index": (
        "index_proximal",
        "index_middle",
        "index_distal",
    ),
    "middle": (
        "middle_proximal",
        "middle_middle",
        "middle_distal",
    ),
    "ring": (
        "ring_proximal",
        "ring_middle",
        "ring_distal",
    ),
    "pinky": (
        "pinky_proximal",
        "pinky_middle",
        "pinky_distal",
    ),
}
_TIP_LINKS = {
    "thumb": "thumb_distal",
    "index": "index_distal",
    "middle": "middle_distal",
    "ring": "ring_distal",
    "pinky": "pinky_distal",
}
_TIP_OFFSETS = {
    "thumb": np.array([0.0, 0.02329, 0.0]),
    "index": np.array([0.0, 0.0, 0.02346]),
    "middle": np.array([0.0, 0.0, 0.02346]),
    "ring": np.array([0.0, 0.0, 0.02346]),
    "pinky": np.array([0.0, 0.0, 0.02346]),
}


class O30IRetargeter:
    """Solve all 20 independent O30i URDF joints in canonical radians."""

    def __init__(
        self,
        urdf_path: str | Path,
        side: str,
        *,
        smooth_weight: float = 2.5e-3,
        filter_alpha: float = 0.7,
        max_iterations: int = 30,
        direction_weight: float = 1.0,
        pinch_weight: float = 1.0,
        contact_deadzone: float = 0.0,
        distance_weight_scale: float = 1.0,
        finger_open_ranges: dict[str, tuple[float, float]] | None = None,
        finger_curl_ranges: dict[str, tuple[float, float]] | None = None,
        thumb_open_bend_threshold: float = 0.0,
        middle_pinch_anchor: dict[str, float] | None = None,
        middle_pinch_start: float = 0.0,
        middle_pinch_activation_step: float = 1.0,
        index_middle_pinch_anchor: dict[str, float] | None = None,
        index_middle_contact_distance: float = 0.0,
        index_middle_start_distance: float = 0.0,
        index_middle_activation_step: float = 1.0,
    ) -> None:
        if side != "right":
            raise ValueError("the checked-in O30i retargeter currently supports right only")
        if not 0.0 < filter_alpha <= 1.0:
            raise ValueError("filter_alpha must be in (0, 1]")
        if direction_weight < 0.0 or pinch_weight < 0.0:
            raise ValueError("direction_weight and pinch_weight must not be negative")
        if contact_deadzone < 0.0 or distance_weight_scale <= 0.0:
            raise ValueError(
                "contact_deadzone must not be negative and "
                "distance_weight_scale must be positive"
            )
        self.direction_weight = float(direction_weight)
        self.pinch_weight = float(pinch_weight)
        self.contact_deadzone = float(contact_deadzone)
        self.distance_weight_scale = float(distance_weight_scale)
        if thumb_open_bend_threshold < 0.0:
            raise ValueError("thumb_open_bend_threshold must not be negative")
        self.thumb_open_bend_threshold = float(thumb_open_bend_threshold)
        if middle_pinch_start < contact_deadzone:
            raise ValueError("middle_pinch_start must not be below contact_deadzone")
        if not 0.0 < middle_pinch_activation_step <= 1.0:
            raise ValueError("middle_pinch_activation_step must be in (0, 1]")
        self.middle_pinch_start = float(middle_pinch_start)
        self.middle_pinch_activation_step = float(middle_pinch_activation_step)
        if index_middle_contact_distance < 0.0:
            raise ValueError("index_middle_contact_distance must not be negative")
        if index_middle_pinch_anchor and (
            index_middle_start_distance <= index_middle_contact_distance
        ):
            raise ValueError(
                "index_middle_start_distance must exceed contact distance"
            )
        if not 0.0 < index_middle_activation_step <= 1.0:
            raise ValueError("index_middle_activation_step must be in (0, 1]")
        self.index_middle_contact_distance = float(index_middle_contact_distance)
        self.index_middle_start_distance = float(index_middle_start_distance)
        self.index_middle_activation_step = float(index_middle_activation_step)
        self.finger_open_ranges = self._validate_bend_ranges(
            finger_open_ranges, "finger_open_ranges"
        )
        self.finger_curl_ranges = self._validate_bend_ranges(
            finger_curl_ranges, "finger_curl_ranges"
        )
        self.side = side
        self.urdf_path = Path(urdf_path).resolve()
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"O30i URDF not found: {self.urdf_path}")
        self.model = pin.buildModelFromUrdf(str(self.urdf_path))
        entries = sorted(
            (self.model.joints[joint_id].idx_q, self.model.names[joint_id])
            for joint_id in range(1, self.model.njoints)
        )
        self.joint_names = [name for _, name in entries]
        if self.model.nq != 20 or any(
            self.model.joints[joint_id].nq != 1
            for joint_id in range(1, self.model.njoints)
        ):
            raise ValueError("O30i URDF must contain 20 independent single-DoF joints")
        self.lower = np.asarray(self.model.lowerPositionLimit, dtype=np.float64)
        self.upper = np.asarray(self.model.upperPositionLimit, dtype=np.float64)
        self.middle_pinch_anchor = {
            str(name): float(value)
            for name, value in (middle_pinch_anchor or {}).items()
        }
        self.index_middle_pinch_anchor = {
            str(name): float(value)
            for name, value in (index_middle_pinch_anchor or {}).items()
        }
        if set(self.middle_pinch_anchor).difference(self.joint_names):
            raise ValueError("middle_pinch_anchor contains unknown joints")
        if set(self.index_middle_pinch_anchor).difference(self.joint_names):
            raise ValueError("index_middle_pinch_anchor contains unknown joints")
        for name, value in {
            **self.middle_pinch_anchor,
            **self.index_middle_pinch_anchor,
        }.items():
            index = self.joint_names.index(name)
            if (
                not np.isfinite(value)
                or value < self.lower[index]
                or value > self.upper[index]
            ):
                raise ValueError(f"middle_pinch_anchor[{name!r}] is out of range")
        self._middle_pinch_activation = 0.0
        self._index_middle_activation = 0.0
        self.smooth_weight = float(smooth_weight)
        self.filter_alpha = float(filter_alpha)
        self.max_iterations = int(max_iterations)
        self.last_qpos = np.clip(np.zeros(self.model.nq), self.lower, self.upper)
        self.filtered_qpos: np.ndarray | None = None

        self._joint_indices = {
            finger: np.asarray(
                [
                    index
                    for index, name in enumerate(self.joint_names)
                    if name.startswith(finger)
                ],
                dtype=int,
            )
            for finger in CANONICAL_FINGERS
        }
        self._frames: dict[str, list[int]] = {}
        for finger, links in _TARGET_LINKS.items():
            frame_ids = [self._body_frame(name) for name in links]
            distal_id = self._body_frame(_TIP_LINKS[finger])
            distal = self.model.frames[distal_id]
            tip_id = self.model.addFrame(
                pin.Frame(
                    f"{finger}_tip_target",
                    distal.parentJoint,
                    distal_id,
                    distal.placement
                    * pin.SE3(np.eye(3), _TIP_OFFSETS[finger].copy()),
                    pin.FrameType.OP_FRAME,
                )
            )
            self._frames[finger] = [*frame_ids, tip_id]
        self.data = self.model.createData()

        neutral = self.robot_landmarks(self.last_qpos)
        self.robot_frame = orthonormal_palm_frame(neutral)
        self.robot_centroid = finger_base_centroid(neutral)
        self.robot_scale = palm_scale(neutral)
        self.robot_finger_lengths = {
            finger: self._chain_length(neutral, chain)
            for finger, chain in CANONICAL_FINGERS.items()
        }
        if self.robot_scale < 1e-6:
            raise ValueError("invalid O30i URDF palm geometry")

    @staticmethod
    def _validate_bend_ranges(
        ranges: dict[str, tuple[float, float]] | None, label: str
    ) -> dict[str, tuple[float, float]]:
        result = {}
        for finger, limits in (ranges or {}).items():
            if finger not in {"index", "middle", "ring", "pinky"}:
                raise ValueError(f"invalid {label} finger {finger!r}")
            values = np.asarray(limits, dtype=np.float64)
            if (
                values.shape != (2,)
                or not np.all(np.isfinite(values))
                or values[0] >= values[1]
            ):
                raise ValueError(
                    f"{label}[{finger!r}] must be finite increasing values"
                )
            result[finger] = (float(values[0]), float(values[1]))
        return result

    def _body_frame(self, link_name: str) -> int:
        if not self.model.existFrame(link_name):
            raise ValueError(f"O30i URDF has no link named {link_name!r}")
        return self.model.getFrameId(link_name)

    @staticmethod
    def _chain_length(points: np.ndarray, chain: tuple[int, ...]) -> float:
        return sum(
            float(np.linalg.norm(points[chain[index + 1]] - points[chain[index]]))
            for index in range(3)
        )

    def _update(self, qpos: np.ndarray, *, jacobians: bool = False) -> None:
        if jacobians:
            pin.computeJointJacobians(self.model, self.data, qpos)
        else:
            pin.forwardKinematics(self.model, self.data, qpos)
        pin.updateFramePlacements(self.model, self.data)

    def robot_landmarks(self, qpos: np.ndarray | None = None) -> np.ndarray:
        values = self.last_qpos if qpos is None else np.asarray(qpos, dtype=np.float64)
        self._update(values)
        points = np.zeros((CANONICAL_LANDMARK_COUNT, 3), dtype=np.float64)
        for finger, chain in CANONICAL_FINGERS.items():
            for landmark, frame_id in zip(chain, self._frames[finger], strict=True):
                points[landmark] = self.data.oMf[frame_id].translation
        return points

    def target_positions(self, landmarks: np.ndarray) -> np.ndarray:
        points = np.asarray(landmarks, dtype=np.float64)
        if points.shape != (CANONICAL_LANDMARK_COUNT, 3):
            raise ValueError(
                f"expected canonical landmarks shape {(CANONICAL_LANDMARK_COUNT, 3)}, "
                f"got {points.shape}"
            )
        if not np.isfinite(points).all():
            raise ValueError("canonical landmarks contain NaN or infinity")
        human_scale = palm_scale(points)
        if human_scale < 1e-6:
            raise ValueError("invalid MANUS palm geometry")
        human_frame = orthonormal_palm_frame(points - points[0])
        centered = points - finger_base_centroid(points)
        transformed = (
            (centered @ human_frame)
            * (self.robot_scale / human_scale)
            @ self.robot_frame.T
            + self.robot_centroid
        )
        for finger, chain in CANONICAL_FINGERS.items():
            segments = [
                transformed[chain[index + 1]] - transformed[chain[index]]
                for index in range(3)
            ]
            human_length = sum(
                float(np.linalg.norm(segment)) for segment in segments
            )
            if human_length < 1e-8:
                continue
            ratio = self.robot_finger_lengths[finger] / human_length
            cursor = transformed[chain[0]].copy()
            for index, segment in enumerate(segments):
                cursor = cursor + segment * ratio
                transformed[chain[index + 1]] = cursor
        return transformed

    def _flex_indices(self, finger: str) -> np.ndarray:
        return np.asarray(
            [
                index
                for index in self._joint_indices[finger]
                if not self.joint_names[index].endswith("mcp_roll")
            ],
            dtype=int,
        )

    def _apply_endpoint_calibration(
        self, solution: np.ndarray, raw: np.ndarray
    ) -> None:
        """Map labelled MANUS open/full gestures to mechanical endpoints."""
        for finger in ("index", "middle", "ring", "pinky"):
            human_bend = chain_bend_angle(
                raw[list(CANONICAL_FINGERS[finger])]
            )
            indices = self._flex_indices(finger)
            if finger in self.finger_open_ranges:
                opened, released = self.finger_open_ranges[finger]
                activation = 1.0 - float(
                    np.clip(
                        (human_bend - opened) / (released - opened), 0.0, 1.0
                    )
                )
                solution[indices] += activation * (
                    self.lower[indices] - solution[indices]
                )
            if finger in self.finger_curl_ranges:
                started, curled = self.finger_curl_ranges[finger]
                activation = float(
                    np.clip(
                        (human_bend - started) / (curled - started), 0.0, 1.0
                    )
                )
                solution[indices] += activation * (
                    self.upper[indices] - solution[indices]
                )

        thumb_bend = chain_bend_angle(
            raw[list(CANONICAL_FINGERS["thumb"])]
        )
        if (
            self.thumb_open_bend_threshold > 0.0
            and thumb_bend <= self.thumb_open_bend_threshold
        ):
            for name in ("thumb_mcp", "thumb_ip"):
                index = self.joint_names.index(name)
                solution[index] = self.lower[index]

    def _apply_middle_pinch_anchor(
        self, solution: np.ndarray, raw: np.ndarray
    ) -> None:
        if not self.middle_pinch_anchor:
            return
        raw_distance = float(np.linalg.norm(raw[4] - raw[12]))
        requested = (
            float(
                np.clip(
                    (self.middle_pinch_start - raw_distance)
                    / (self.middle_pinch_start - self.contact_deadzone),
                    0.0,
                    1.0,
                )
            )
            if self.middle_pinch_start > self.contact_deadzone
            else 0.0
        )
        delta = np.clip(
            requested - self._middle_pinch_activation,
            -self.middle_pinch_activation_step,
            self.middle_pinch_activation_step,
        )
        self._middle_pinch_activation += float(delta)
        for name, target in self.middle_pinch_anchor.items():
            index = self.joint_names.index(name)
            solution[index] += self._middle_pinch_activation * (
                target - solution[index]
            )

    def _apply_index_middle_pinch_anchor(
        self, solution: np.ndarray, raw: np.ndarray
    ) -> float:
        requested = 0.0
        if self.index_middle_pinch_anchor:
            requested = index_middle_pinch_request(
                raw,
                contact_distance=self.index_middle_contact_distance,
                start_distance=self.index_middle_start_distance,
            )
        delta = np.clip(
            requested - self._index_middle_activation,
            -self.index_middle_activation_step,
            self.index_middle_activation_step,
        )
        self._index_middle_activation += float(delta)
        for name, target in self.index_middle_pinch_anchor.items():
            index = self.joint_names.index(name)
            solution[index] += self._index_middle_activation * (
                target - solution[index]
            )
        return requested

    def retarget(
        self, landmarks: np.ndarray
    ) -> tuple[np.ndarray, dict[str, float | int | bool]]:
        wanted = self.target_positions(landmarks)
        raw = np.asarray(landmarks, dtype=np.float64)

        # Activated pinch objectives: when the human thumb tip approaches a
        # fingertip, the robot thumb is pulled toward that fingertip's solved
        # position. Distances are hand-scale-normalized and only EXCESS robot
        # distance is penalized, so open-hand poses are unaffected. Ported
        # from the L20 solver; unlike the G20 mechanism, the O30i URDF can
        # close the thumb-index gap to exactly zero (measured oracle), so the
        # term can genuinely converge.
        middle_chain = CANONICAL_FINGERS["middle"]
        human_middle = sum(
            float(np.linalg.norm(raw[middle_chain[i + 1]] - raw[middle_chain[i]]))
            for i in range(3)
        )
        distance_scale = self.robot_finger_lengths["middle"] / max(
            human_middle, 1e-8
        )
        pinch_terms = []
        pinch_activation = 0.0
        if self.pinch_weight > 0.0:
            for finger_name, tip_landmark, weight in zip(
                ("index", "middle", "ring", "pinky"),
                (8, 12, 16, 20),
                THUMB_DISTANCE_WEIGHTS,
            ):
                raw_distance = float(np.linalg.norm(raw[4] - raw[tip_landmark]))
                activation = max(
                    0.0, 1.0 - raw_distance / THUMB_DISTANCE_THRESHOLD
                )
                if activation <= 0.0:
                    continue
                pinch_activation = max(pinch_activation, activation)
                pinch_terms.append(
                    (
                        self._frames[finger_name][-1],
                        max(0.0, raw_distance - self.contact_deadzone)
                        * distance_scale,
                        self.pinch_weight
                        * weight
                        * activation
                        * self.distance_weight_scale,
                    )
                )
        thumb_tip_frame = self._frames["thumb"][-1]

        start = np.clip(self.last_qpos, self.lower, self.upper)
        solution = start.copy()
        total_loss = 0.0
        iterations = 0
        evaluations = 0
        success = True
        # Thumb last, so its pinch terms see the ordinary fingers' solved tips.
        for finger in ("index", "middle", "ring", "pinky", "thumb"):
            indices = self._joint_indices[finger]
            frames = self._frames[finger]
            chain = CANONICAL_FINGERS[finger]
            targets = wanted[list(chain)]
            direction_terms = []
            if self.direction_weight > 0.0:
                for pair, weight in enumerate(SEGMENT_DIRECTION_WEIGHTS):
                    desired = wanted[chain[pair + 1]] - wanted[chain[pair]]
                    length = float(np.linalg.norm(desired))
                    if length < 1e-8:
                        continue
                    direction_terms.append(
                        (
                            frames[pair],
                            frames[pair + 1],
                            desired / length,
                            self.direction_weight * weight,
                        )
                    )
            anchor = start[indices]
            q_work = solution.copy()

            def objective(finger_q: np.ndarray) -> tuple[float, np.ndarray]:
                q_work[indices] = finger_q
                self._update(q_work, jacobians=True)
                loss = 0.0
                gradient = np.zeros(len(indices), dtype=np.float64)
                # One Jacobian extraction per frame; the position, direction,
                # and pinch terms all reuse these four slices.
                jacobians = {
                    frame_id: pin.getFrameJacobian(
                        self.model,
                        self.data,
                        frame_id,
                        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                    )[:3, indices]
                    for frame_id in frames
                }
                for frame_id, target, weight in zip(
                    frames, targets, CHAIN_WEIGHTS, strict=True
                ):
                    residual = self.data.oMf[frame_id].translation - target
                    loss += weight * float(residual @ residual)
                    gradient += 2.0 * weight * (jacobians[frame_id].T @ residual)
                for origin_id, target_id, desired, weight in direction_terms:
                    vector = (
                        self.data.oMf[target_id].translation
                        - self.data.oMf[origin_id].translation
                    )
                    length = float(np.linalg.norm(vector))
                    if length < 1e-8:
                        continue
                    direction = vector / length
                    cosine = float(direction @ desired)
                    loss += weight * (1.0 - cosine)
                    derivative = -(desired - cosine * direction) / length
                    gradient += weight * (
                        derivative @ (jacobians[target_id] - jacobians[origin_id])
                    )
                if finger == "thumb":
                    for finger_tip_id, desired_distance, weight in pinch_terms:
                        vector = (
                            self.data.oMf[finger_tip_id].translation
                            - self.data.oMf[thumb_tip_frame].translation
                        )
                        distance = float(np.linalg.norm(vector))
                        if distance < 1e-8:
                            continue
                        excess = distance - desired_distance
                        if excess <= 0.0:
                            continue
                        loss += weight * excess * excess
                        # The fingertip frame does not move with thumb joints.
                        gradient += (
                            -2.0
                            * weight
                            * excess
                            * ((vector / distance) @ jacobians[thumb_tip_frame])
                        )
                delta = finger_q - anchor
                loss += self.smooth_weight * float(delta @ delta)
                gradient += 2.0 * self.smooth_weight * delta
                return loss, gradient

            result = minimize(
                objective,
                start[indices],
                method="L-BFGS-B",
                jac=True,
                bounds=list(zip(self.lower[indices], self.upper[indices])),
                options={
                    "maxiter": (
                        max(40, self.max_iterations)
                        if finger == "thumb"
                        else self.max_iterations
                    ),
                    # The L20 solver's tolerances: the direction terms' O(1)
                    # gradient scale makes tighter ones burn iterations for
                    # sub-micrometre gains.
                    "ftol": 1e-9,
                    "gtol": 1e-6,
                },
            )
            solution[indices] = np.clip(
                result.x, self.lower[indices], self.upper[indices]
            )
            total_loss += float(result.fun)
            iterations += int(result.nit)
            evaluations += int(result.nfev)
            success = success and bool(result.success)
        self._apply_endpoint_calibration(solution, raw)
        self._apply_middle_pinch_anchor(solution, raw)
        index_middle_request = self._apply_index_middle_pinch_anchor(solution, raw)

        self.last_qpos = solution
        if self.filtered_qpos is None:
            self.filtered_qpos = solution.copy()
        else:
            self.filtered_qpos += self.filter_alpha * (
                solution - self.filtered_qpos
            )
        emitted = self.filtered_qpos.copy()
        emitted_landmarks = self.robot_landmarks(emitted)
        return emitted, {
            "success": success,
            "loss": total_loss,
            "iterations": iterations,
            "function_evaluations": evaluations,
            "pinch_activation": pinch_activation,
            "middle_pinch_activation": self._middle_pinch_activation,
            "index_middle_pinch_request": index_middle_request,
            "index_middle_pinch_activation": self._index_middle_activation,
            "index_middle_gap": float(
                np.linalg.norm(emitted_landmarks[8] - emitted_landmarks[12])
            ),
        }

    def reset(self) -> None:
        self.last_qpos = np.clip(np.zeros(self.model.nq), self.lower, self.upper)
        self.filtered_qpos = None
        self._middle_pinch_activation = 0.0
        self._index_middle_activation = 0.0

    def close(self) -> None:
        """Pinocchio owns no external resource."""
