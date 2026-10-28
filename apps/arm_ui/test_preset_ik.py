import math

import pytest

from apps.arm_ui.preset_ik_cli import serialize_preset_solution
from apps.arm_ui.runtime import IkValidation, RelativeIkSolution


def _solution(
    *,
    start: tuple[float, ...] = (0.0,) * 7,
    frames: tuple[tuple[float, ...], ...] | None = None,
    times: tuple[float, ...] = (0.0, 0.1, 0.2),
) -> RelativeIkSolution:
    if frames is None:
        frames = (start, (0.01,) + start[1:], (0.02,) + start[1:])
    return RelativeIkSolution(
        side="left",
        joint_names=tuple(f"joint{index}" for index in range(7)),
        start_positions=start,
        time_sec=times,
        positions=frames,
        validation=IkValidation(
            checked_frames=len(frames),
            total_frames=len(frames),
            max_joint_step_rad=0.02,
        ),
    )


def test_serialize_preset_solution_keeps_checked_speed_and_offset() -> None:
    payload = serialize_preset_solution(
        _solution(),
        name="kuai1",
        start_q=(0.0,) * 14,
        speed_scale=0.5,
        max_joint_speed=0.5,
    )

    assert payload["name"] == "kuai1"
    assert payload["side"] == "left"
    assert payload["speed_scale"] == 0.5
    assert payload["checked_frames"] == 3
    assert payload["start_q"] == [0.0] * 14


def test_serialize_preset_solution_rejects_first_frame_offset() -> None:
    offset = 0.06
    with pytest.raises(RuntimeError, match=f"{math.degrees(offset):.1f} deg"):
        serialize_preset_solution(
            _solution(frames=((offset,) + (0.0,) * 6,) * 3),
            name="kuai1",
            start_q=(0.0,) * 14,
            speed_scale=0.5,
            max_joint_speed=0.5,
        )


def test_serialize_preset_solution_rejects_joint_speed() -> None:
    frames = (
        (0.0,) * 7,
        (0.12,) + (0.0,) * 6,
        (0.24,) + (0.0,) * 6,
    )
    with pytest.raises(RuntimeError, match="exceeds 0.50 rad/s"):
        serialize_preset_solution(
            _solution(frames=frames),
            name="kuai1",
            start_q=(0.0,) * 14,
            speed_scale=0.5,
            max_joint_speed=0.5,
        )
