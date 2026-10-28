import time
from types import SimpleNamespace

import numpy as np
import pytest

import apps.wuji_ui.runtime as runtime_module
from apps.wuji_ui.runtime import (
    JointDiagnosticSnapshot,
    WujiUiRuntime,
    summarize_joint_hold_test,
)


class FakeBackend:
    def __init__(self, on_send=None) -> None:
        self.enabled = False
        self.sent = []
        self.on_send = on_send
        self.gains = [(4.0, 0.1)] * 20

    @property
    def last_command_position(self):
        return None if not self.sent else self.sent[-1].copy()

    def send(self, command) -> None:
        assert self.enabled
        values = np.asarray(command).copy()
        self.sent.append(values)
        if self.on_send is not None:
            self.on_send(values)

    def mit_gains(self):
        return tuple(self.gains)

    def set_mit_gains(self, *, kp, kd, joint_index=None):
        if joint_index is None:
            self.gains = [(kp, kd)] * 20
        else:
            self.gains[joint_index] = (kp, kd)
        return tuple(self.gains)


class FakePipeline:
    def __init__(self, **kwargs) -> None:
        assert kwargs["auto_enable"] is False
        self.joint_names = {
            side: tuple(f"{side}_j{i}" for i in range(20))
            for side in ("left", "right")
        }
        self.joint_limits = {
            side: tuple((-1.0, 1.5) for _ in range(20))
            for side in ("left", "right")
        }
        self.positions = {
            "left": np.full(20, 0.1),
            "right": np.full(20, 0.2),
        }
        self.backends = {
            side: FakeBackend(
                lambda values, side=side: self.positions.__setitem__(
                    side, values.copy()
                )
            )
            for side in ("left", "right")
        }
        canonical = np.asarray(
            [
                [0.00, 0.00, 0.00],
                [0.02, 0.00, 0.00], [0.04, 0.00, 0.00],
                [0.06, 0.00, 0.00], [0.08, 0.00, 0.00],
                [0.03, 0.04, 0.00], [0.03, 0.06, 0.00],
                [0.03, 0.08, 0.00], [0.03, 0.10, 0.00],
                [0.00, 0.05, 0.00], [0.00, 0.075, 0.00],
                [0.00, 0.10, 0.00], [0.00, 0.125, 0.00],
                [-0.03, 0.04, 0.00], [-0.03, 0.06, 0.00],
                [-0.03, 0.08, 0.00], [-0.03, 0.10, 0.00],
                [-0.06, 0.03, 0.00], [-0.06, 0.05, 0.00],
                [-0.06, 0.07, 0.00], [-0.06, 0.09, 0.00],
            ]
        )
        raw = np.zeros((25, 3), dtype=float)
        raw[[0, 1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14,
             16, 17, 18, 19, 21, 22, 23, 24]] = canonical
        points = [
            SimpleNamespace(
                position_x=point[0], position_y=point[1], position_z=point[2]
            )
            for point in raw
        ]
        self.last_frames = {
            side: SimpleNamespace(sequence=1, keypoints=points)
            for side in ("left", "right")
        }
        self.closed = False
        self.status = SimpleNamespace(
            sides={
                side: SimpleNamespace(fault=None, sending=False)
                for side in ("left", "right")
            }
        )

    def set_enabled(self, side, enabled) -> None:
        self.backends[side].enabled = enabled

    def feedback_position(self, side):
        return self.positions[side].copy()

    def tick(self, _now, active) -> None:
        for side, enabled in active.items():
            if enabled:
                self.backends[side].send(np.full(20, 0.3))

    def joint_diagnostics(self, _side):
        status = SimpleNamespace(
            node_id=17,
            current_a=0.2,
            bus_voltage_v=24.0,
            temperature_c=32.0,
            error_code=0,
            ext_state=2,
            ext_state_name="Enabled",
            position_limit_active=False,
            velocity_limit_active=False,
            current_limit_active=False,
            comm_response_rate_pct=100,
            comm_timeout_total=0,
        )
        result = {}
        for index in range(20):
            values = vars(status).copy()
            values["node_id"] = (index // 4) * 5 + index % 4 + 1
            result[index] = SimpleNamespace(**values)
        return result

    def mit_gains(self, side):
        return self.backends[side].mit_gains()

    def set_mit_gains(self, side, *, kp, kd, joint_index=None):
        return self.backends[side].set_mit_gains(
            kp=kp, kd=kd, joint_index=joint_index
        )

    def close(self) -> None:
        self.closed = True


def test_runtime_starts_manual_and_switches_each_side_independently(monkeypatch) -> None:
    monkeypatch.setattr(runtime_module, "WujiHandPipeline", FakePipeline)
    runtime = WujiUiRuntime(addresses={"left": "left:1", "right": "right:2"})
    try:
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            try:
                runtime.snapshot("left")
                break
            except RuntimeError:
                time.sleep(0.01)
        assert runtime.mode("left") == "manual"
        assert runtime.mode("right") == "manual"
        deadline = time.monotonic() + 0.5
        while True:
            try:
                gaps = runtime.manus_gaps("left")
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.01)
        assert gaps.gaps_m["index"] == pytest.approx(np.hypot(0.05, 0.10))
        assert len(gaps.features) == 12
        assert "index_curl_rad" in gaps.features

        runtime.start_teleop("left")
        assert runtime.mode("left") == "teleop"
        assert runtime.mode("right") == "manual"
        assert runtime.pipeline.backends["left"].enabled
        assert not runtime.pipeline.backends["right"].enabled

        runtime.move_to_pose("right", [0.4] * 20)
        assert runtime.mode("right") == "pose"
        assert runtime.pipeline.backends["right"].enabled

        runtime.set_manual("left")
        assert runtime.mode("left") == "manual"
        assert not runtime.pipeline.backends["left"].enabled
    finally:
        pipeline = runtime.pipeline
        runtime.close()
    assert pipeline.closed


def test_pose_sequence_runs_in_order_and_can_be_stopped(monkeypatch) -> None:
    monkeypatch.setattr(runtime_module, "WujiHandPipeline", FakePipeline)
    runtime = WujiUiRuntime(
        addresses={"left": "left:1", "right": "right:2"},
        pose_speed_rad_s=10.0,
    )
    try:
        deadline = time.monotonic() + 0.5
        while True:
            try:
                runtime.snapshot("left")
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.01)
        runtime.execute_pose_sequence(
            "left",
            [("first", [0.2] * 20), ("second", [0.4] * 20)],
            hold_seconds=0.0,
        )
        assert runtime.mode("left") == "hold"
        assert not runtime.sequence_status("left").running
        np.testing.assert_allclose(
            runtime.pipeline.backends["left"].sent[-1], [0.4] * 20
        )
    finally:
        runtime.close()


def test_runtime_jog_follows_slider_target_with_rate_limit(monkeypatch) -> None:
    monkeypatch.setattr(runtime_module, "WujiHandPipeline", FakePipeline)
    runtime = WujiUiRuntime(
        addresses={"left": "left:1", "right": "right:2"},
        pose_speed_rad_s=10.0,
    )
    try:
        deadline = time.monotonic() + 0.5
        while True:
            try:
                runtime.snapshot("right")
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.01)

        with pytest.raises(RuntimeError, match="不在滑块控制模式"):
            runtime.set_jog_target("right", [0.5] * 20)

        runtime.start_jog("right")
        assert runtime.mode("right") == "jog"
        assert runtime.pipeline.backends["right"].enabled
        runtime.set_jog_target("right", [0.5] * 20)

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            sent = runtime.pipeline.backends["right"].sent
            if sent and np.allclose(sent[-1], 0.5):
                break
            time.sleep(0.01)
        else:
            raise AssertionError("jog target was not reached")
        assert runtime.mode("right") == "jog"

        runtime.set_manual("right")
        assert runtime.mode("right") == "manual"
        assert not runtime.pipeline.backends["right"].enabled
    finally:
        runtime.close()


def test_runtime_jog_rejects_out_of_limit_target(monkeypatch) -> None:
    monkeypatch.setattr(runtime_module, "WujiHandPipeline", FakePipeline)
    runtime = WujiUiRuntime(addresses={"left": "left:1", "right": "right:2"})
    try:
        deadline = time.monotonic() + 0.5
        while True:
            try:
                runtime.snapshot("left")
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.01)
        runtime.start_jog("left")
        with pytest.raises(ValueError, match="关节范围"):
            runtime.set_jog_target("left", [9.0] * 20)
    finally:
        runtime.close()


def _diagnostic_sample(*, actual, current=0.2, limited=False, error_code=0):
    return JointDiagnosticSnapshot(
        side="left",
        joint_index=13,
        joint_name="l_ring_finger_mcp_abd",
        node_id=17,
        target_position=0.0,
        actual_position=actual,
        current_a=current,
        bus_voltage_v=24.0,
        temperature_c=35.0,
        error_code=error_code,
        ext_state=2,
        ext_state_name="Enabled",
        position_limit_active=False,
        velocity_limit_active=False,
        current_limit_active=limited,
        comm_response_rate_pct=100,
        comm_timeout_total=0,
        received_at=time.monotonic(),
    )


def test_hold_summary_uses_operator_observation_for_mechanical_slip() -> None:
    samples = [_diagnostic_sample(actual=0.001 * index) for index in range(6)]
    result = summarize_joint_hold_test(samples, "feedback_static")
    assert result["category"] == "mechanical_likely"


def test_hold_summary_flags_current_limited_position_error() -> None:
    samples = [
        _diagnostic_sample(actual=0.05, current=0.9, limited=True)
        for _ in range(6)
    ]
    result = summarize_joint_hold_test(samples, "feedback_moves")
    assert result["category"] == "current_limited"


def test_runtime_applies_gains_to_one_joint_and_holds(monkeypatch) -> None:
    monkeypatch.setattr(runtime_module, "WujiHandPipeline", FakePipeline)
    runtime = WujiUiRuntime(addresses={"left": "left:1", "right": "right:2"})
    try:
        deadline = time.monotonic() + 0.5
        while True:
            try:
                runtime.snapshot("left")
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.01)
        gains = runtime.set_mit_gains(
            "left", kp=5.0, kd=0.15, joint_index=13
        )
        assert gains[13] == (5.0, 0.15)
        assert gains[12] == (4.0, 0.1)
        assert runtime.mode("left") == "hold"
        assert runtime.pipeline.backends["left"].enabled
    finally:
        runtime.close()


def test_runtime_rejects_out_of_ui_range_gains(monkeypatch) -> None:
    monkeypatch.setattr(runtime_module, "WujiHandPipeline", FakePipeline)
    runtime = WujiUiRuntime(addresses={"left": "left:1", "right": "right:2"})
    try:
        with pytest.raises(ValueError, match="网页运行时调参范围"):
            runtime.set_mit_gains("left", kp=100.0, kd=0.1, joint_index=13)
    finally:
        runtime.close()
