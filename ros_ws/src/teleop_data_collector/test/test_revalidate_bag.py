import json
from pathlib import Path

import pytest

from teleop_data_collector.revalidate_bag import revalidate_bag


def _bag(tmp_path: Path, *, state: str = "incomplete") -> Path:
    bag = tmp_path / "episode0"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    (bag / "collection_state.json").write_text(
        json.dumps(
            {
                "state": state,
                "finalized": False,
                "failures": ["old bag-time failure"],
                "source_topic_contract": {
                    "cam0": {
                        "topic": "/cam0/color/image_raw",
                        "type": "sensor_msgs/msg/Image",
                        "min_frequency_hz": 18.0,
                        "max_gap_ms": 150.0,
                        "required": True,
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    return bag


def test_revalidation_finalizes_source_complete_bag_and_preserves_warning(
    tmp_path: Path,
) -> None:
    bag = _bag(tmp_path)

    def inspect(path, contracts, **kwargs):
        assert path == bag
        assert contracts[0].topic == "/cam0/color/image_raw"
        assert kwargs == {"trim_start_sec": 1.0, "trim_end_sec": 1.0}
        return (), {
            "boundary_warnings": ["cam0 edge accepted by trim"],
            "transport_warnings": ["cam0 bag gap 250 ms"],
        }

    state = revalidate_bag(
        bag, trim_start_sec=1.0, trim_end_sec=1.0, inspect=inspect
    )

    assert state["state"] == "finalized"
    assert state["finalized"] is True
    assert state["failures"] == []
    assert state["transport_warnings"] == ["cam0 bag gap 250 ms"]
    assert state["boundary_warnings"] == ["cam0 edge accepted by trim"]
    assert state["trim_start_sec"] == 1.0
    assert state["trim_end_sec"] == 1.0
    assert state["validation_policy"] == "source_header_continuity_trimmed_boundary_v3"
    assert state["revalidation_history"][-1]["previous_state"] == "incomplete"
    assert json.loads((bag / "collection_state.json").read_text()) == state


def test_revalidation_does_not_promote_interrupted_bag(tmp_path: Path) -> None:
    bag = _bag(tmp_path, state="interrupted")

    with pytest.raises(ValueError, match="Refusing to revalidate"):
        revalidate_bag(bag, inspect=lambda *_: ((), {}))
