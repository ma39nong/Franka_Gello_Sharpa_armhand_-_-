from types import SimpleNamespace

import numpy as np
import pytest

from adapters.wuji.backend import (
    WujiHand2Backend,
    _connect_hand2,
    _hand2_diagnostics,
    _hand2_feedback_positions,
)


def frame(*, missing_node=None):
    node_ids = [finger * 5 + joint for finger in range(5) for joint in range(1, 5)]
    entries = [
        SimpleNamespace(nid=node_id, position=float(index))
        for index, node_id in enumerate(node_ids)
        if node_id != missing_node
    ]
    return SimpleNamespace(num_joints=len(entries), joints=list(reversed(entries)))


def test_hand2_feedback_is_reordered_by_node_id() -> None:
    np.testing.assert_array_equal(_hand2_feedback_positions(frame()), np.arange(20))


def test_hand2_feedback_requires_all_twenty_joints() -> None:
    with pytest.raises(RuntimeError, match="missing joints"):
        _hand2_feedback_positions(frame(missing_node=7))


def test_hand2_diagnostics_maps_ring_abd_node_to_device_index_13() -> None:
    status = SimpleNamespace(
        ext_state=2,
        ext_state_name="Enabled",
        position_limit_active=False,
        velocity_limit_active=False,
        current_limit_active=True,
    )
    entry = SimpleNamespace(
        nid=17,
        status_word=status,
        current=0.42,
        vbus_v_fb=23.8,
        mcu_temp_c_fb=34.5,
        error_code_current=0,
        comm_response_rate_pct=99,
        comm_timeout_total=2,
    )
    diagnostic = _hand2_diagnostics(
        SimpleNamespace(num_joints=1, joints=[entry])
    )[13]
    assert diagnostic.node_id == 17
    assert diagnostic.current_a == pytest.approx(0.42)
    assert diagnostic.current_limit_active
    assert diagnostic.ext_state_name == "Enabled"


def test_hand2_backend_preserves_other_gains_for_single_joint_update() -> None:
    written = []
    resource = SimpleNamespace(set=lambda value: written.append(value))
    backend = WujiHand2Backend.__new__(WujiHand2Backend)
    backend._hand = SimpleNamespace(mit_params=lambda: resource)
    backend._kp_values = np.full(20, 4.0)
    backend._kd_values = np.full(20, 0.1)

    gains = backend.set_mit_gains(kp=5.0, kd=0.15, joint_index=13)

    assert gains[13] == (5.0, 0.15)
    assert gains[12] == (4.0, 0.1)
    assert len(written[-1]) == 20
    assert written[-1][13] == (5.0, 0.15)


def test_hand2_connection_disables_unneeded_cross_process_bridge() -> None:
    calls = []

    class FakeOptions:
        def __init__(self, **kwargs) -> None:
            self.kwargs = kwargs

    class FakeManager:
        def connect(self, **kwargs):
            calls.append(kwargs)
            return "hand"

    sdk = SimpleNamespace(ConnectOptions=FakeOptions)

    result = _connect_hand2(
        FakeManager(), sdk, address="192.168.2.111:7447", side="right"
    )

    assert result == "hand"
    assert len(calls) == 1
    assert calls[0]["address"] == "192.168.2.111:7447"
    assert calls[0]["device_name"] == "wuji_hand_2_right"
    assert calls[0]["options"].kwargs == {"enable_bridge": False}
