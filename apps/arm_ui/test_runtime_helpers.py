from unittest.mock import Mock

import pytest

from apps.arm_ui.runtime import ArmRosRuntime


def test_switch_mode_allows_both_arms_in_teach_mode() -> None:
    runtime = Mock(spec=ArmRosRuntime)
    runtime.is_running.return_value = False
    runtime.recording_status.return_value = {"active": False}
    runtime.absolute_recording_status.return_value = {"active": False}
    runtime._graph_controller_mode.return_value = "teach"

    ArmRosRuntime.switch_mode(runtime, "right", "teach")

    runtime._switch_mode_unchecked.assert_called_once_with("right", "teach")
    runtime._graph_controller_mode.assert_not_called()


@pytest.mark.parametrize("side", ["left", "right"])
def test_switch_mode_still_rejects_changes_during_execution(side: str) -> None:
    runtime = Mock(spec=ArmRosRuntime)
    runtime.is_running.return_value = True

    with pytest.raises(RuntimeError, match="轨迹正在执行"):
        ArmRosRuntime.switch_mode(runtime, side, "teach")

    runtime._switch_mode_unchecked.assert_not_called()


def test_finite_difference_velocities_keep_endpoints_at_rest() -> None:
    velocities = ArmRosRuntime._finite_difference_velocities(
        (0.1, 0.5, 1.0),
        ((0.0, 0.0), (0.1, -0.2), (0.3, -0.4)),
    )

    assert velocities[0] == (0.0, 0.0)
    assert velocities[1] == pytest.approx((1.0 / 3.0, -4.0 / 9.0))
    assert velocities[2] == (0.0, 0.0)


def test_finite_difference_velocities_reject_non_increasing_time() -> None:
    with pytest.raises(ValueError, match="严格递增"):
        ArmRosRuntime._finite_difference_velocities(
            (0.1, 0.1), ((0.0,), (0.2,))
        )


def test_cubic_extrema_detect_position_spike_hidden_by_central_difference() -> None:
    times = (0.0, 1.0, 2.0)
    positions = ((0.0,), (1.0,), (0.0,))
    velocities = ArmRosRuntime._finite_difference_velocities(times, positions)

    maximum_velocity, maximum_acceleration = (
        ArmRosRuntime._cubic_trajectory_extrema(times, positions, velocities)
    )

    assert velocities == ((0.0,), (0.0,), (0.0,))
    assert maximum_velocity == pytest.approx((1.5,))
    assert maximum_acceleration == pytest.approx((6.0,))


def test_cubic_extrema_reject_non_increasing_time() -> None:
    with pytest.raises(ValueError, match="严格递增"):
        ArmRosRuntime._cubic_trajectory_extrema(
            (0.0, 0.0),
            ((0.0,), (0.1,)),
            ((0.0,), (0.0,)),
        )
