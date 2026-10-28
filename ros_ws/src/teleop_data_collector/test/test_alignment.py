import pytest

from teleop_data_collector.alignment import (
    TimedValue,
    align_nearest,
    interpolate_linear,
)


def test_nearest_alignment_is_deterministic_for_mixed_rates() -> None:
    report = align_nearest(
        [100, 200, 300],
        {
            "arm": [TimedValue(90, (1.0,)), TimedValue(210, (2.0,)), TimedValue(290, (3.0,))],
            "camera": [TimedValue(80, (10.0,)), TimedValue(280, (20.0,))],
        },
        tolerance_ns=120,
    )
    assert [row["arm"].value for row in report.rows] == [(1.0,), (2.0,), (3.0,)]
    assert [row["camera"].value for row in report.rows] == [(10.0,), (20.0,), (20.0,)]
    assert report.unmatched_by_stream == {"arm": 0, "camera": 0}


def test_nearest_alignment_reports_tolerance_misses() -> None:
    report = align_nearest(
        [100, 200], {"camera": [TimedValue(10, (1.0,))]}, tolerance_ns=50
    )
    assert report.rows == ({}, {})
    assert report.unmatched_by_stream == {"camera": 2}


def test_non_monotonic_source_time_is_rejected() -> None:
    with pytest.raises(ValueError, match="strictly increasing"):
        align_nearest(
            [100],
            {"arm": [TimedValue(20, (1.0,)), TimedValue(10, (2.0,))]},
            tolerance_ns=100,
        )


def test_global_bag_interleaving_does_not_change_per_stream_alignment() -> None:
    streams = {
        "left": [TimedValue(90, (1.0,)), TimedValue(190, (2.0,))],
        "right": [TimedValue(95, (3.0,)), TimedValue(195, (4.0,))],
    }
    assert align_nearest([100, 200], streams, tolerance_ns=20) == align_nearest(
        [100, 200], dict(reversed(list(streams.items()))), tolerance_ns=20
    )


def test_linear_interpolation_preserves_target_timestamp() -> None:
    value = interpolate_linear(5, TimedValue(0, (0.0, 2.0)), TimedValue(10, (10.0, 4.0)))
    assert value == TimedValue(5, (5.0, 3.0))
