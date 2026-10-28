import pytest

from linker_hand_ros2_sdk.o30i_contract import (
    O30I_DRIVER_JOINT_NAMES,
    O30I_DRIVER_TO_URDF,
    O30I_RIGHT_LOWER,
    O30I_RIGHT_UPPER,
    O30I_URDF_JOINT_NAMES,
    radians_to_ticks,
    ticks_to_radians,
)


def test_o30i_contract_is_bijective_and_twenty_joint():
    assert len(O30I_URDF_JOINT_NAMES) == 20
    assert len(O30I_DRIVER_JOINT_NAMES) == 20
    assert set(O30I_DRIVER_TO_URDF) == set(O30I_DRIVER_JOINT_NAMES)
    assert set(O30I_DRIVER_TO_URDF.values()) == set(O30I_URDF_JOINT_NAMES)


@pytest.mark.parametrize("index", range(20))
def test_o30i_endpoint_calibration_round_trips(index):
    lower = O30I_RIGHT_LOWER[index]
    upper = O30I_RIGHT_UPPER[index]
    for tick_lower, tick_upper in ((0.0, 255.0), (255.0, 0.0)):
        assert radians_to_ticks(
            lower, lower, upper, tick_lower, tick_upper
        ) == round(tick_lower)
        assert radians_to_ticks(
            upper, lower, upper, tick_lower, tick_upper
        ) == round(tick_upper)
        midpoint = (lower + upper) / 2.0
        tick = radians_to_ticks(
            midpoint, lower, upper, tick_lower, tick_upper
        )
        reconstructed = ticks_to_radians(
            tick, lower, upper, tick_lower, tick_upper
        )
        assert reconstructed == pytest.approx(
            midpoint, abs=(upper - lower) / 255.0
        )
