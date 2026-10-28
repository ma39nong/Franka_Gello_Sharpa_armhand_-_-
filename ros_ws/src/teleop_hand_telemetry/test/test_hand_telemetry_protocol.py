import json
import sys
import time
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT))

from teleop_runtime.hand_telemetry import (
    PROTOCOL_VERSION,
    HandTelemetryPacket,
    PacketTracker,
    VelocityEstimator,
    decode_packet,
    encode_packet,
)


NAMES = tuple(f"joint_{index}" for index in range(20))


def _packet(*, sequence=1, session="session-a", source_monotonic_ns=1_000_000_000):
    return HandTelemetryPacket(
        version=PROTOCOL_VERSION,
        session_id=session,
        sequence=sequence,
        side="left",
        source_wall_time_ns=2_000_000_000,
        source_monotonic_ns=source_monotonic_ns,
        joint_names=NAMES,
        command=tuple(float(index) for index in range(20)),
        command_monotonic_ns=source_monotonic_ns - 2_000_000,
        state=tuple(float(index) / 2.0 for index in range(20)),
        state_monotonic_ns=source_monotonic_ns - 3_000_000,
        engaged=True,
        command_valid=True,
        state_valid=True,
        sender_dropped_packets=3,
    )


def test_protocol_round_trip_preserves_named_values_and_timestamps():
    packet = _packet()
    assert decode_packet(encode_packet(packet)) == packet


def test_protocol_rejects_tamper_and_unknown_version():
    envelope = json.loads(encode_packet(_packet()))
    envelope["payload"]["side"] = "right"
    with pytest.raises(ValueError, match="checksum"):
        decode_packet(json.dumps(envelope).encode())

    with pytest.raises(ValueError, match="version"):
        encode_packet(_packet().__class__(**{**_packet().__dict__, "version": 99}))


def test_tracker_rejects_duplicates_and_out_of_order_and_counts_loss():
    tracker = PacketTracker(max_staleness_ns=150_000_000)
    assert tracker.accept(_packet(sequence=1), received_monotonic_ns=1_010_000_000)
    assert not tracker.accept(_packet(sequence=1), received_monotonic_ns=1_011_000_000)
    assert not tracker.accept(_packet(sequence=0), received_monotonic_ns=1_012_000_000)
    assert tracker.accept(_packet(sequence=4), received_monotonic_ns=1_013_000_000)
    assert tracker.duplicates == 1
    assert tracker.out_of_order == 1
    assert tracker.lost == 2


def test_tracker_rejects_stale_packet_and_accepts_new_session():
    tracker = PacketTracker(max_staleness_ns=150_000_000)
    assert not tracker.accept(
        _packet(sequence=1, source_monotonic_ns=1_000_000_000),
        received_monotonic_ns=1_200_000_001,
    )
    assert tracker.stale == 1
    assert tracker.accept(
        _packet(sequence=0, session="session-b"),
        received_monotonic_ns=1_010_000_000,
    )


def test_velocity_is_derived_only_from_named_monotonic_samples():
    estimator = VelocityEstimator()
    first = estimator.update("left", NAMES, [0.0] * 20, 1_000_000_000)
    assert first.values is None
    assert first.source == "unavailable_first_sample"

    second = estimator.update("left", NAMES, [0.1] * 20, 1_100_000_000)
    assert second.values == pytest.approx((1.0,) * 20)
    assert second.source == "finite_difference"

    with pytest.raises(ValueError, match="joint names changed"):
        estimator.update("left", tuple(reversed(NAMES)), [0.2] * 20, 1_200_000_000)


def test_repeated_state_timestamp_reseeds_velocity_without_invalidating_position():
    estimator = VelocityEstimator()
    estimator.update("left", NAMES, [0.0] * 20, 1_000_000_000)
    repeated = estimator.update("left", NAMES, [0.1] * 20, 1_000_000_000)

    assert repeated.values is None
    assert repeated.source == "unavailable_nonmonotonic_reset"
    resumed = estimator.update("left", NAMES, [0.2] * 20, 1_100_000_000)
    assert resumed.values == pytest.approx((1.0,) * 20)
    assert resumed.source == "finite_difference"


def test_udp_sender_drops_when_full_without_blocking_the_caller():
    import threading

    from teleop_runtime.hand_telemetry import UdpTelemetrySender

    release = threading.Event()

    class BlockingSocket:
        def __init__(self, *_args, **_kwargs):
            self.sent = 0

        def settimeout(self, _timeout):
            return None

        def sendto(self, payload, destination):
            release.wait(timeout=1.0)
            self.sent += 1

        def close(self):
            release.set()

    sender = UdpTelemetrySender(
        "127.0.0.1",
        5602,
        queue_size=1,
        socket_factory=BlockingSocket,
    )
    try:
        packet = {
            "side": "left",
            "source_wall_time_ns": 2_000_000_000,
            "source_monotonic_ns": 1_000_000_000,
            "joint_names": NAMES,
            "command": tuple(0.1 for _ in range(20)),
            "command_monotonic_ns": 900_000_000,
            "state": tuple(0.2 for _ in range(20)),
            "state_monotonic_ns": 800_000_000,
            "engaged": True,
            "command_valid": True,
            "state_valid": True,
        }
        accepted = 0
        dropped = 0
        started = time.monotonic()
        for _ in range(32):
            if sender.offer(**packet):
                accepted += 1
            else:
                dropped += 1
        elapsed = time.monotonic() - started
        assert elapsed < 0.05
        assert accepted >= 1
        assert dropped >= 1
        assert sender.dropped_packets == dropped
    finally:
        release.set()
        sender.close()
