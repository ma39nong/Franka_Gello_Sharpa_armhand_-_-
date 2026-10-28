from pathlib import Path

import pytest
import yaml

from apps.arm_ui.models import ArmRepository, Routine, Waypoint


def test_waypoints_and_routines_round_trip(tmp_path: Path) -> None:
    repository = ArmRepository(tmp_path)
    repository.save_waypoint(Waypoint.create("home", "left", range(7)))
    repository.save_waypoint(Waypoint.create("取料", "left", [0.1] * 7))
    repository.save_routine(
        Routine.create(
            "取放",
            "left",
            ["home", "取料"],
            velocity_scale=0.2,
            acceleration_scale=0.15,
            blend_radius_m=0.004,
        )
    )

    reloaded = ArmRepository(tmp_path)
    assert reloaded.waypoint("left", "取料").joints == (0.1,) * 7
    assert reloaded.routines("left")[0].waypoints == ("home", "取料")
    assert yaml.safe_load((tmp_path / "waypoints.yaml").read_text())["units"] == "rad"


def test_rename_updates_routine_reference(tmp_path: Path) -> None:
    repository = ArmRepository(tmp_path)
    repository.save_waypoint(Waypoint.create("old", "right", [0.0] * 7))
    repository.save_routine(Routine.create("task", "right", ["old"]))

    repository.save_waypoint(
        Waypoint.create("new", "right", [0.01] * 7), previous_name="old"
    )

    assert repository.routines("right")[0].waypoints == ("new",)
    with pytest.raises(KeyError):
        repository.waypoint("right", "old")


def test_cannot_delete_referenced_waypoint(tmp_path: Path) -> None:
    repository = ArmRepository(tmp_path)
    repository.save_waypoint(Waypoint.create("point", "left", [0.0] * 7))
    repository.save_routine(Routine.create("task", "left", ["point"]))

    with pytest.raises(ValueError, match="任务引用"):
        repository.delete_waypoint("left", "point")


def test_new_point_cannot_silently_overwrite_existing_name(tmp_path: Path) -> None:
    repository = ArmRepository(tmp_path)
    repository.save_waypoint(Waypoint.create("point", "left", [0.0] * 7))

    with pytest.raises(ValueError, match="已存在"):
        repository.save_waypoint(Waypoint.create("point", "left", [0.1] * 7))


@pytest.mark.parametrize("count", [0, 10])
def test_routine_requires_one_to_nine_waypoints(count: int) -> None:
    with pytest.raises(ValueError, match="1到9"):
        Routine.create("task", "left", [f"p{i}" for i in range(count)])
