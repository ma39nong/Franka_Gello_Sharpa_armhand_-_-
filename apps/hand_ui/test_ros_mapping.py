import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parent))

from barmate.hardware.linker_hand.control.ros_backend import (  # noqa: E402
    _G20_COMMAND_NAMES,
    _O30I_JOINT_NAMES,
    _controller_position_to_raw,
    _mapping_for,
    _raw_to_controller_position,
)


def test_o30i_ui_mapping_uses_current_workcell_contract():
    mapping = _mapping_for("O30I", "right")
    assert mapping.command_joint_names == _O30I_JOINT_NAMES
    assert len(mapping.command_joint_names) == 20
    for index in range(20):
        assert _controller_position_to_raw(
            _raw_to_controller_position(0, mapping, index), mapping, index
        ) == 0
        assert _controller_position_to_raw(
            _raw_to_controller_position(255, mapping, index), mapping, index
        ) == 255


def test_g20_manual_bridge_contract_has_twenty_unique_slots():
    assert len(_G20_COMMAND_NAMES) == 20
    assert len(set(_G20_COMMAND_NAMES)) == 20
