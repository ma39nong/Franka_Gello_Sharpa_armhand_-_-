from __future__ import annotations

from pathlib import Path

import numpy as np


def horizontal_world_to_control(
    forward_displacement: np.ndarray,
    left_displacement: np.ndarray,
    *,
    minimum_displacement: float = 0.05,
) -> np.ndarray:
    """Fit OpenVR horizontal directions to control +X forward and +Y left."""
    forward = np.asarray(forward_displacement, dtype=float)
    left = np.asarray(left_displacement, dtype=float)
    if forward.shape != (3,) or left.shape != (3,):
        raise ValueError("calibration displacements must be 3-vectors")
    if not np.all(np.isfinite(forward)) or not np.all(np.isfinite(left)):
        raise ValueError("calibration displacements must be finite")
    if minimum_displacement <= 0.0:
        raise ValueError("minimum_displacement must be positive")

    openvr_up = np.array([0.0, 1.0, 0.0])
    forward -= openvr_up * float(forward @ openvr_up)
    left -= openvr_up * float(left @ openvr_up)
    forward_distance = float(np.linalg.norm(forward))
    left_distance = float(np.linalg.norm(left))
    if forward_distance < minimum_displacement:
        raise ValueError(
            f"forward movement is only {forward_distance:.3f} m; "
            f"move at least {minimum_displacement:.3f} m"
        )
    if left_distance < minimum_displacement:
        raise ValueError(
            f"left movement is only {left_distance:.3f} m; "
            f"move at least {minimum_displacement:.3f} m"
        )

    forward_axis = forward / forward_distance
    # In a right-handed source basis, source_left x source_up = source_forward.
    forward_from_left = np.cross(left / left_distance, openvr_up)
    agreement = float(forward_axis @ forward_from_left)
    if agreement < 0.7:
        raise ValueError(
            "forward and left captures are inconsistent "
            f"(direction agreement={agreement:.3f}); repeat the calibration"
        )
    source_forward = forward_axis + forward_from_left
    source_forward /= np.linalg.norm(source_forward)
    source_left = np.cross(openvr_up, source_forward)
    measured_left = left / left_distance
    if float(source_left @ measured_left) < 0.7:
        raise ValueError("left capture points opposite the fitted +Y direction")

    # Rows are the source directions whose scalar components become control
    # X, Y and Z. This maps source_forward/left/up to the matching unit axes.
    result = np.vstack((source_forward, source_left, openvr_up))
    if not np.allclose(result @ result.T, np.eye(3), atol=1e-6):
        raise RuntimeError("internal calibration result is not orthonormal")
    if not np.isclose(np.linalg.det(result), 1.0, atol=1e-6):
        raise RuntimeError("internal calibration result is not right-handed")
    return result


def write_world_to_control_rotation(path, rotation: np.ndarray) -> None:
    """Replace only the root YAML rotation block, preserving all other text."""
    config_path = Path(path)
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    if not np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-6):
        raise ValueError("rotation must be orthonormal")
    if not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-6):
        raise ValueError("rotation must be right-handed")

    lines = config_path.read_text(encoding="utf-8").splitlines(keepends=True)
    matches = [
        index
        for index, line in enumerate(lines)
        if line.rstrip("\r\n") == "world_to_control_rotation:"
    ]
    if len(matches) != 1:
        raise ValueError(
            f"{config_path} must contain exactly one root "
            "world_to_control_rotation block"
        )
    start = matches[0]
    end = start + 1
    while end < len(lines):
        text = lines[end].rstrip("\r\n")
        if text and not text[0].isspace():
            break
        end += 1
    rows = ["world_to_control_rotation:\n"]
    rows.extend(
        "  - [" + ", ".join(f"{value:.9f}" for value in row) + "]\n"
        for row in matrix
    )
    # Keep one blank line between this root block and the next section.
    rows.append("\n")
    while end > start + 1 and not lines[end - 1].strip():
        end -= 1
    config_path.write_text(
        "".join([*lines[:start], *rows, *lines[end:]]), encoding="utf-8"
    )
