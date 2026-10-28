import pickle

import numpy as np
import pytest

from adapters.wuji.sim import (
    DEFAULT_REPLAYS,
    ReplayRecorder,
    replay_frames,
    selected_replay_path,
)


def test_default_replays_are_the_recorded_left_and_right_pinch_sequences():
    assert DEFAULT_REPLAYS["left"].name == "l_pinch_2.pkl"
    assert DEFAULT_REPLAYS["right"].name == "r_pinch_2.pkl"
    for side, path in DEFAULT_REPLAYS.items():
        assert path.is_file()
        replay = replay_frames(path, side)
        assert next(replay).shape == (21, 3)

    override = DEFAULT_REPLAYS["right"].with_name("another.pkl")
    assert selected_replay_path("right", None) == DEFAULT_REPLAYS["right"]
    assert selected_replay_path("right", override) == override


def test_recorded_manus_landmarks_can_be_replayed(tmp_path, monkeypatch):
    moments = iter((10.0, 10.1, 10.2))
    monkeypatch.setattr("adapters.wuji.sim.time.monotonic", lambda: next(moments))
    output = tmp_path / "right.pkl"
    first = np.arange(63, dtype=np.float64).reshape(21, 3)
    second = first + 1.0

    recorder = ReplayRecorder(output, "right")
    recorder.add(first)
    recorder.add(second)
    assert recorder.save() == 2

    with output.open("rb") as stream:
        rows = pickle.load(stream)
    assert [row["t"] for row in rows] == pytest.approx([0.1, 0.2])
    assert all(row["left_fingers"] is None for row in rows)
    np.testing.assert_array_equal(rows[0]["right_fingers"], first)
    np.testing.assert_array_equal(rows[1]["right_fingers"], second)

    replay = replay_frames(output, "right")
    np.testing.assert_array_equal(next(replay), first)
    np.testing.assert_array_equal(next(replay), second)
    np.testing.assert_array_equal(next(replay), first)


def test_recorder_refuses_to_overwrite_existing_replay(tmp_path):
    output = tmp_path / "existing.pkl"
    output.write_bytes(b"keep me")
    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        ReplayRecorder(output, "right")
    assert output.read_bytes() == b"keep me"
