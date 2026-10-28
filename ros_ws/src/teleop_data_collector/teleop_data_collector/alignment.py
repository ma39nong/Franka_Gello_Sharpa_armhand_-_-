from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
from typing import Mapping, Sequence


@dataclass(frozen=True)
class TimedValue:
    timestamp_ns: int
    value: tuple[float, ...]


@dataclass(frozen=True)
class AlignmentReport:
    rows: tuple[dict[str, TimedValue], ...]
    unmatched_by_stream: dict[str, int]


def align_nearest(
    timeline_ns: Sequence[int],
    streams: Mapping[str, Sequence[TimedValue]],
    *,
    tolerance_ns: int,
) -> AlignmentReport:
    if tolerance_ns < 0:
        raise ValueError("alignment tolerance must be non-negative")
    _strictly_increasing(timeline_ns, "timeline")
    prepared = {}
    for name, samples in streams.items():
        timestamps = [sample.timestamp_ns for sample in samples]
        _strictly_increasing(timestamps, f"stream {name}")
        prepared[name] = (timestamps, tuple(samples))

    rows = []
    unmatched = {name: 0 for name in streams}
    for target in timeline_ns:
        row = {}
        for name, (timestamps, samples) in prepared.items():
            index = bisect_left(timestamps, target)
            candidates = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(samples)]
            if not candidates:
                unmatched[name] += 1
                continue
            # Earlier sample wins an exact-distance tie, independent of bag
            # interleaving/arrival order across different topics.
            best = min(candidates, key=lambda item: (abs(timestamps[item] - target), timestamps[item]))
            if abs(timestamps[best] - target) > tolerance_ns:
                unmatched[name] += 1
                continue
            row[name] = samples[best]
        rows.append(row)
    return AlignmentReport(tuple(rows), unmatched)


def interpolate_linear(
    target_ns: int,
    before: TimedValue,
    after: TimedValue,
) -> TimedValue:
    if before.timestamp_ns >= after.timestamp_ns:
        raise ValueError("interpolation samples must have increasing timestamps")
    if not before.timestamp_ns <= target_ns <= after.timestamp_ns:
        raise ValueError("interpolation target must be bracketed by samples")
    if len(before.value) != len(after.value):
        raise ValueError("interpolation sample dimensions must match")
    fraction = (target_ns - before.timestamp_ns) / (after.timestamp_ns - before.timestamp_ns)
    return TimedValue(
        target_ns,
        tuple(
            float(left + fraction * (right - left))
            for left, right in zip(before.value, after.value, strict=True)
        ),
    )


def _strictly_increasing(values: Sequence[int], label: str) -> None:
    if any(current <= previous for previous, current in zip(values, values[1:])):
        raise ValueError(f"{label} timestamps must be strictly increasing")
