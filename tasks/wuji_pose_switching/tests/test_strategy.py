from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from apps.wuji_ui.models import ManusTrigger
from tasks.wuji_pose_switching.strategy import (
    PosePhase,
    TwoStageGestureTrigger,
    TwoStagePoseRetargeter,
    load_config,
)


def _triggers() -> tuple[ManusTrigger, ManusTrigger]:
    ready = ManusTrigger.create(
        "11",
        "left",
        "index",
        "ready",
        enter_max_m=0.05,
        exit_min_m=0.06,
        dwell_seconds=0.15,
    )
    active = ManusTrigger.create(
        "55",
        "left",
        "index",
        "active",
        enter_max_m=0.02,
        exit_min_m=0.03,
        dwell_seconds=0.15,
    )
    return ready, active


def _landmarks(*, index_xy=(0.015, 0.085)) -> np.ndarray:
    points = np.asarray(
        [
            [0.00, 0.00, 0.00],
            [0.02, 0.00, 0.00],
            [0.04, 0.00, 0.00],
            [0.06, 0.01, 0.00],
            [0.07, 0.03, 0.00],
            [0.03, 0.04, 0.00],
            [0.03, 0.06, 0.00],
            [0.025, 0.075, 0.00],
            [index_xy[0], index_xy[1], 0.00],
            [0.00, 0.05, 0.00],
            [0.00, 0.075, 0.00],
            [-0.005, 0.095, 0.00],
            [-0.015, 0.11, 0.00],
            [-0.03, 0.04, 0.00],
            [-0.03, 0.06, 0.00],
            [-0.035, 0.075, 0.00],
            [-0.045, 0.085, 0.00],
            [-0.06, 0.03, 0.00],
            [-0.06, 0.05, 0.00],
            [-0.065, 0.065, 0.00],
            [-0.075, 0.075, 0.00],
        ],
        dtype=np.float64,
    )
    return points


def test_two_stage_trigger_applies_both_dwells_and_hysteresis() -> None:
    ready, active = _triggers()
    trigger = TwoStageGestureTrigger(ready, active)

    assert trigger.update(1.0, 0.04, 0.04) is PosePhase.FREE
    assert trigger.update(1.16, 0.04, 0.04) is PosePhase.READY
    assert trigger.update(1.17, 0.04, 0.015) is PosePhase.READY
    assert trigger.update(1.33, 0.015, 0.015) is PosePhase.ACTIVE
    assert trigger.update(1.4, 0.025, 0.025) is PosePhase.ACTIVE
    assert trigger.update(1.5, 0.031, 0.031) is PosePhase.READY
    assert trigger.update(1.6, 0.059, 0.059) is PosePhase.READY
    assert trigger.update(1.7, 0.06, 0.06) is PosePhase.FREE


def test_same_finger_stages_must_nest() -> None:
    ready, _active = _triggers()
    overlapping = ManusTrigger.create(
        "55",
        "left",
        "index",
        "active",
        enter_max_m=0.04,
        exit_min_m=0.07,
        dwell_seconds=0.15,
    )
    with pytest.raises(ValueError, match="same-finger"):
        TwoStageGestureTrigger(ready, overlapping)


def test_retargeter_reorders_device_poses_and_resets() -> None:
    points = _landmarks(index_xy=(0.068, 0.032))
    ready, active = _triggers()
    device_names = tuple(f"j{index}" for index in range(20))
    optimizer_names = tuple(reversed(device_names))

    class FakeBase:
        def __init__(self):
            self.reset_count = 0
            model = SimpleNamespace(
                lowerPositionLimit=np.full(20, -2.0),
                upperPositionLimit=np.full(20, 2.0),
            )
            robot = SimpleNamespace(dof_joint_names=optimizer_names, model=model)
            self.optimizer = SimpleNamespace(robot=robot)

        def retarget(self, _points):
            return np.zeros(20)

        def reset(self):
            self.reset_count += 1

    moments = iter((0.0, 0.2, 0.4, 0.56))
    base = FakeBase()
    ready_device = np.arange(20, dtype=float) / 100.0
    active_device = np.full(20, 0.5)
    wrapped = TwoStagePoseRetargeter(
        base,
        ready=ready,
        active=active,
        ready_pose_device=ready_device,
        active_pose_device=active_device,
        device_joint_names=device_names,
        max_joint_speed_rad_s=10.0,
        clock=lambda: next(moments),
    )

    np.testing.assert_array_equal(wrapped.retarget(points), np.zeros(20))
    np.testing.assert_allclose(wrapped.retarget(points), ready_device[::-1])
    wrapped.retarget(points)
    np.testing.assert_allclose(wrapped.retarget(points), np.full(20, 0.5))
    assert wrapped.phase is PosePhase.ACTIVE

    wrapped.reset()
    assert wrapped.phase is PosePhase.FREE
    assert wrapped.last_qpos is None
    assert base.reset_count == 1


def test_checked_in_policy_resolves_the_ui_pose_library() -> None:
    path = Path(__file__).parents[1] / "config" / "policy.json"
    config = load_config(path)

    assert config.side == "left"
    assert config.ready_trigger == "11"
    assert config.active_trigger == "55"
    assert config.pose_library.name == "wuji_hand_2_poses.json"
    assert config.pose_library.is_file()


def test_policy_rejects_non_positive_speed(tmp_path: Path) -> None:
    path = tmp_path / "policy.json"
    path.write_text(
        '{"format":"wuji-two-stage-manus-policy","version":1,'
        '"side":"left","pose_library":"poses.json",'
        '"ready_trigger":"11","active_trigger":"55",'
        '"max_joint_speed_rad_s":0}'
    )

    with pytest.raises(ValueError, match="positive"):
        load_config(path)
