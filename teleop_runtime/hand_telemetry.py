"""Versioned, checksummed, one-way Wuji telemetry transport.

This module deliberately has no ROS or hardware-SDK dependency.  The existing
Operator process only enqueues snapshots; a daemon thread performs UDP I/O.
The ROS receiver imports the same codec and can only publish observations.
"""

from __future__ import annotations

import json
import math
import queue
import socket
import threading
import uuid
import zlib
from dataclasses import asdict, dataclass
from typing import Callable, Sequence


PROTOCOL_VERSION = 1
SIDES = ("left", "right")
JOINT_COUNT = 20
MAX_DATAGRAM_BYTES = 60_000


@dataclass(frozen=True)
class HandTelemetryPacket:
    version: int
    session_id: str
    sequence: int
    side: str
    source_wall_time_ns: int
    source_monotonic_ns: int
    joint_names: tuple[str, ...]
    command: tuple[float, ...] | None
    command_monotonic_ns: int | None
    state: tuple[float, ...] | None
    state_monotonic_ns: int | None
    engaged: bool
    command_valid: bool
    state_valid: bool
    sender_dropped_packets: int

    def validate(self) -> None:
        if self.version != PROTOCOL_VERSION:
            raise ValueError(f"unsupported telemetry protocol version {self.version}")
        if not self.session_id or len(self.session_id) > 128:
            raise ValueError("session_id must be a non-empty bounded string")
        if self.sequence < 0:
            raise ValueError("sequence must be non-negative")
        if self.side not in SIDES:
            raise ValueError(f"invalid telemetry side {self.side!r}")
        if self.source_wall_time_ns <= 0 or self.source_monotonic_ns <= 0:
            raise ValueError("source timestamps must be positive")
        _validate_names(self.joint_names)
        _validate_vector("command", self.command)
        _validate_vector("state", self.state)
        if (self.command is None) != (self.command_monotonic_ns is None):
            raise ValueError("command and command timestamp must appear together")
        if (self.state is None) != (self.state_monotonic_ns is None):
            raise ValueError("state and state timestamp must appear together")
        for label, timestamp in (
            ("command", self.command_monotonic_ns),
            ("state", self.state_monotonic_ns),
        ):
            if timestamp is not None:
                if timestamp <= 0 or timestamp > self.source_monotonic_ns:
                    raise ValueError(f"invalid {label} monotonic timestamp")
        if self.command_valid and self.command is None:
            raise ValueError("command_valid requires a command")
        if self.state_valid and self.state is None:
            raise ValueError("state_valid requires a state")
        if self.sender_dropped_packets < 0:
            raise ValueError("sender_dropped_packets must be non-negative")


@dataclass(frozen=True)
class VelocityResult:
    values: tuple[float, ...] | None
    source: str


def _validate_names(names: Sequence[str]) -> None:
    if len(names) != JOINT_COUNT:
        raise ValueError(f"joint_names must contain exactly {JOINT_COUNT} names")
    cleaned = tuple(str(name) for name in names)
    if any(not name for name in cleaned) or len(set(cleaned)) != JOINT_COUNT:
        raise ValueError("joint_names must be non-empty and unique")


def _validate_vector(label: str, values: Sequence[float] | None) -> None:
    if values is None:
        return
    if len(values) != JOINT_COUNT:
        raise ValueError(f"{label} must contain exactly {JOINT_COUNT} values")
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError(f"{label} contains a non-finite value")


def _canonical_payload(packet: HandTelemetryPacket) -> bytes:
    payload = asdict(packet)
    return json.dumps(
        payload,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def encode_packet(packet: HandTelemetryPacket) -> bytes:
    packet.validate()
    payload_bytes = _canonical_payload(packet)
    envelope = {
        "crc32": f"{zlib.crc32(payload_bytes) & 0xFFFFFFFF:08x}",
        "payload": json.loads(payload_bytes),
    }
    encoded = json.dumps(
        envelope,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    if len(encoded) > MAX_DATAGRAM_BYTES:
        raise ValueError("telemetry packet exceeds safe UDP datagram size")
    return encoded


def decode_packet(data: bytes) -> HandTelemetryPacket:
    if not data or len(data) > MAX_DATAGRAM_BYTES:
        raise ValueError("invalid telemetry datagram size")
    try:
        envelope = json.loads(data.decode("utf-8"))
        payload = envelope["payload"]
        expected_crc = str(envelope["crc32"])
    except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError) as error:
        raise ValueError("invalid telemetry envelope") from error
    payload_bytes = json.dumps(
        payload,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    actual_crc = f"{zlib.crc32(payload_bytes) & 0xFFFFFFFF:08x}"
    if actual_crc != expected_crc:
        raise ValueError("telemetry checksum mismatch")
    try:
        packet = HandTelemetryPacket(
            version=int(payload["version"]),
            session_id=str(payload["session_id"]),
            sequence=int(payload["sequence"]),
            side=str(payload["side"]),
            source_wall_time_ns=int(payload["source_wall_time_ns"]),
            source_monotonic_ns=int(payload["source_monotonic_ns"]),
            joint_names=tuple(str(name) for name in payload["joint_names"]),
            command=(
                None
                if payload["command"] is None
                else tuple(float(value) for value in payload["command"])
            ),
            command_monotonic_ns=(
                None
                if payload["command_monotonic_ns"] is None
                else int(payload["command_monotonic_ns"])
            ),
            state=(
                None
                if payload["state"] is None
                else tuple(float(value) for value in payload["state"])
            ),
            state_monotonic_ns=(
                None
                if payload["state_monotonic_ns"] is None
                else int(payload["state_monotonic_ns"])
            ),
            engaged=bool(payload["engaged"]),
            command_valid=bool(payload["command_valid"]),
            state_valid=bool(payload["state_valid"]),
            sender_dropped_packets=int(payload["sender_dropped_packets"]),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("invalid telemetry payload") from error
    packet.validate()
    return packet


class PacketTracker:
    """Reject duplicate/out-of-order/stale packets and retain loss counters."""

    def __init__(self, *, max_staleness_ns: int) -> None:
        if max_staleness_ns <= 0:
            raise ValueError("max_staleness_ns must be positive")
        self.max_staleness_ns = int(max_staleness_ns)
        self.session_id: str | None = None
        self.last_sequence: int | None = None
        self.accepted = 0
        self.duplicates = 0
        self.out_of_order = 0
        self.lost = 0
        self.stale = 0

    def accept(
        self, packet: HandTelemetryPacket, *, received_monotonic_ns: int
    ) -> bool:
        packet.validate()
        age = int(received_monotonic_ns) - packet.source_monotonic_ns
        if age < 0 or age > self.max_staleness_ns:
            self.stale += 1
            return False
        if packet.session_id != self.session_id:
            self.session_id = packet.session_id
            self.last_sequence = None
        if self.last_sequence is not None:
            if packet.sequence == self.last_sequence:
                self.duplicates += 1
                return False
            if packet.sequence < self.last_sequence:
                self.out_of_order += 1
                return False
            if packet.sequence > self.last_sequence + 1:
                self.lost += packet.sequence - self.last_sequence - 1
        self.last_sequence = packet.sequence
        self.accepted += 1
        return True


class VelocityEstimator:
    """Finite-difference state velocity with an explicit provenance marker."""

    def __init__(self) -> None:
        self._previous: dict[str, tuple[tuple[str, ...], tuple[float, ...], int]] = {}

    def update(
        self,
        side: str,
        joint_names: Sequence[str],
        positions: Sequence[float],
        timestamp_ns: int,
    ) -> VelocityResult:
        if side not in SIDES:
            raise ValueError(f"invalid side {side!r}")
        names = tuple(str(name) for name in joint_names)
        values = tuple(float(value) for value in positions)
        _validate_names(names)
        _validate_vector("state", values)
        timestamp = int(timestamp_ns)
        previous = self._previous.get(side)
        if previous is None:
            self._previous[side] = (names, values, timestamp)
            return VelocityResult(None, "unavailable_first_sample")
        old_names, old_values, old_timestamp = previous
        if names != old_names:
            raise ValueError(f"{side} hand joint names changed")
        if timestamp <= old_timestamp:
            # A Home/engagement transition can emit a new telemetry envelope
            # around the same cached feedback sample. The position remains
            # usable, but deriving velocity from a zero/negative dt is not.
            # Re-seed this side so the next fresh sample resumes finite
            # differences without invalidating the complete telemetry packet.
            self._previous[side] = (names, values, timestamp)
            return VelocityResult(None, "unavailable_nonmonotonic_reset")
        dt = (timestamp - old_timestamp) / 1_000_000_000.0
        velocity = tuple(
            (value - old_value) / dt
            for value, old_value in zip(values, old_values, strict=True)
        )
        _validate_vector("derived velocity", velocity)
        self._previous[side] = (names, values, timestamp)
        return VelocityResult(velocity, "finite_difference")


class UdpTelemetrySender:
    """Bounded non-blocking producer with UDP work isolated in one thread."""

    _STOP = object()

    def __init__(
        self,
        host: str,
        port: int,
        *,
        queue_size: int = 64,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        if not host:
            raise ValueError("telemetry host must not be empty")
        if not 0 < int(port) < 65536:
            raise ValueError("telemetry port must be in 1..65535")
        if queue_size <= 0:
            raise ValueError("telemetry queue_size must be positive")
        self.destination = (host, int(port))
        self.session_id = uuid.uuid4().hex
        self._sequence = 0
        self._queue: queue.Queue[HandTelemetryPacket | object] = queue.Queue(
            maxsize=int(queue_size)
        )
        self._socket_factory = socket_factory
        self.dropped_packets = 0
        self.send_errors = 0
        self.last_error: str | None = None
        self._closed = False
        self._thread = threading.Thread(
            target=self._run,
            name="wuji-telemetry-udp",
            daemon=True,
        )
        self._thread.start()

    def offer(
        self,
        *,
        side: str,
        source_wall_time_ns: int,
        source_monotonic_ns: int,
        joint_names: Sequence[str],
        command: Sequence[float] | None,
        command_monotonic_ns: int | None,
        state: Sequence[float] | None,
        state_monotonic_ns: int | None,
        engaged: bool,
        command_valid: bool,
        state_valid: bool,
    ) -> bool:
        """Queue a packet immediately; failures never escape into control code."""
        if self._closed:
            return False
        sequence = self._sequence
        self._sequence += 1
        try:
            packet = HandTelemetryPacket(
                version=PROTOCOL_VERSION,
                session_id=self.session_id,
                sequence=sequence,
                side=str(side),
                source_wall_time_ns=int(source_wall_time_ns),
                source_monotonic_ns=int(source_monotonic_ns),
                joint_names=tuple(str(name) for name in joint_names),
                command=(
                    None if command is None else tuple(float(value) for value in command)
                ),
                command_monotonic_ns=(
                    None
                    if command_monotonic_ns is None
                    else int(command_monotonic_ns)
                ),
                state=(None if state is None else tuple(float(value) for value in state)),
                state_monotonic_ns=(
                    None if state_monotonic_ns is None else int(state_monotonic_ns)
                ),
                engaged=bool(engaged),
                command_valid=bool(command_valid),
                state_valid=bool(state_valid),
                sender_dropped_packets=self.dropped_packets,
            )
            packet.validate()
            self._queue.put_nowait(packet)
            return True
        except queue.Full:
            self.dropped_packets += 1
            return False
        except Exception as error:  # validation is telemetry-only
            self.dropped_packets += 1
            self.last_error = str(error)
            return False

    def _run(self) -> None:
        udp_socket = None
        try:
            udp_socket = self._socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
            udp_socket.settimeout(0.1)
            while True:
                item = self._queue.get()
                if item is self._STOP:
                    return
                try:
                    udp_socket.sendto(encode_packet(item), self.destination)
                except Exception as error:
                    self.send_errors += 1
                    self.last_error = str(error)
        except Exception as error:
            self.send_errors += 1
            self.last_error = str(error)
        finally:
            if udp_socket is not None:
                try:
                    udp_socket.close()
                except Exception:
                    pass

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._queue.put_nowait(self._STOP)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(self._STOP)
            except queue.Full:
                pass
        self._thread.join(timeout=1.0)
