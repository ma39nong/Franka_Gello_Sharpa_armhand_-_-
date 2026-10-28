import json
import threading

import pytest

from pico_bimanual_franka_teleop import loop_timing


class Clock:
    ns = 1_000_000_000
    cpu_ns = 0

    def advance(self, ms, cpu_ms=0):
        self.ns += int(ms * 1e6)
        self.cpu_ns += int(cpu_ms * 1e6)


@pytest.fixture
def clock(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(loop_timing.time, "monotonic_ns", lambda: clock.ns)
    monkeypatch.setattr(loop_timing.time, "thread_time_ns", lambda: clock.cpu_ns)
    return clock


def read_events(path, timing):
    timing.close()
    timing._thread.join(timeout=2)
    assert not timing._thread.is_alive()
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_normal_cycles_do_not_write_per_cycle_records(tmp_path, clock):
    path = tmp_path / "timing.jsonl"
    timing = loop_timing.LoopTimingDiagnostics(path, session_id="session", period_sec=0.01)
    for sequence in range(100):
        timing.begin_cycle()
        clock.advance(1, cpu_ms=0.1)
        timing.mark("send_command")
        timing.command_sent(sequence, ("left", "right"))
        timing.mark("loop_wait")
        clock.advance(9)
    events = read_events(path, timing)
    assert [event["event"] for event in events] == ["start", "stop"]
    assert events[-1]["cycles"] == 100
    assert events[-1]["anomalies"] == 0


def test_slow_log_and_next_send_gap_keep_previous_cycle_context(tmp_path, clock):
    path = tmp_path / "timing.jsonl"
    timing = loop_timing.LoopTimingDiagnostics(path, session_id="session", period_sec=0.01)
    timing.begin_cycle()
    timing.mark("send_command")
    timing.command_sent(10, ("left",))
    timing.mark("debug_log_and_fk")
    clock.advance(650, cpu_ms=2)
    timing.mark("loop_wait")
    timing.begin_cycle()
    timing.mark("send_command")
    timing.command_sent(11, ("left",))
    clock.advance(10)
    events = read_events(path, timing)
    stalls = [e for e in events if e["event"] == "loop_stall"]
    assert len(stalls) == 2
    assert ["debug_log_and_fk", 650_000_000, 2_000_000] in stalls[0]["current"]["stages"]
    assert stalls[1]["current"]["send"]["gap_ns"] == 650_000_000
    assert stalls[1]["current"]["send"]["previous_sequence"] == 10
    assert stalls[1]["current"]["send"]["sequence"] == 11
    assert stalls[1]["previous"] == stalls[0]["current"]
    assert isinstance(stalls[1]["wall_time_ns"], int)
    assert stalls[1]["session_id"] == "session"


def test_wait_overshoot_is_recorded_and_shutdown_is_not_a_control_stall(tmp_path, clock):
    path = tmp_path / "timing.jsonl"
    timing = loop_timing.LoopTimingDiagnostics(path, session_id="session", period_sec=0.01)
    timing.begin_cycle()
    timing.mark("loop_wait")
    clock.advance(300)
    timing.end_cycle()
    clock.advance(2000)  # Closing hardware must not appear as another loop stall.
    events = read_events(path, timing)
    stalls = [e for e in events if e["event"] == "loop_stall"]
    assert len(stalls) == 1
    assert ["loop_wait", 300_000_000, 0] in stalls[0]["current"]["stages"]


def test_blocked_writer_drops_diagnostics_instead_of_waiting(tmp_path, clock):
    release = threading.Event()

    class BlockedWriter(loop_timing.LoopTimingDiagnostics):
        def _write(self):
            release.wait()
            super()._write()

    timing = BlockedWriter(tmp_path / "timing.jsonl", session_id="session", period_sec=0.01)

    def produce():
        for sequence in range(100):
            timing.begin_cycle()
            timing.command_sent(sequence, ())
            clock.advance(200)
        timing.end_cycle()

    producer = threading.Thread(target=produce, daemon=True)
    try:
        producer.start()
        producer.join(timeout=2)
        assert not producer.is_alive(), "Diagnostic producer waited for disk writer"
        assert timing.dropped == 36
        assert timing._queue.qsize() == 64
    finally:
        release.set()
        producer.join(timeout=2)
        timing.close()
        timing._thread.join(timeout=2)


def test_unwritable_diagnostic_path_does_not_break_producer(tmp_path, clock):
    timing = loop_timing.LoopTimingDiagnostics(
        tmp_path / "missing" / "timing.jsonl", session_id="session", period_sec=0.01,
    )
    timing._thread.join(timeout=2)
    assert timing._failed
    timing.begin_cycle()
    timing.command_sent(1, ())
    clock.advance(650)
    timing.begin_cycle()
    timing.command_sent(2, ())
    timing.close()
    assert timing._last_sequence == 2


def test_hardware_probes_capture_stall_without_changing_commands(tmp_path, monkeypatch, clock):
    from types import SimpleNamespace
    import numpy as np
    from pico_bimanual_franka_teleop import hardware
    from pico_bimanual_franka_teleop.types import JointTeleopSample, Pose

    measured = (hardware.LOWER_LIMITS + hardware.UPPER_LIMITS) / 2
    commands = []
    closed = []

    class Robot:
        def __init__(self, **kwargs):
            self.stream_id = "integration"
            self.sequence = 0

        def wait_for_state(self, **kwargs):
            return measured.copy()

        def receive_state(self):
            if self.sequence == 2:
                raise KeyboardInterrupt
            return measured.copy()

        def take_gateway_faults(self):
            return ()

        def send_command(self, q, sides):
            commands.append((q.copy(), sides))
            self.sequence += 1

        def close(self):
            closed.append("robot")

    class IK:
        def __init__(self, **kwargs):
            self.last_diagnostics = {}

        def set_posture_reference(self, q):
            pass

        def frame_poses(self, q):
            return {s: Pose(np.zeros(3), np.eye(3)) for s in ("left", "right")}

    def slow_debug_record(*args, **kwargs):
        clock.advance(650, cpu_ms=1)

    monkeypatch.setattr(hardware, "UdpRobotBackend", Robot)
    monkeypatch.setattr(hardware, "BimanualPinkIK", IK)
    monkeypatch.setattr(hardware.time, "sleep", lambda _: None)
    source = SimpleNamespace(
        output_kind="joint",
        sample=lambda: JointTeleopSample(
            {s: np.zeros(7) for s in ("left", "right")},
            {s: True for s in ("left", "right")}, 0.0,
        ),
        close=lambda: closed.append("source"),
    )
    operator = SimpleNamespace(show=lambda _: None, take_requests=lambda: {})
    teleop = hardware.DualFr3HardwareTeleop(
        command_host="unused", command_port=1, state_host="unused", state_port=2,
        state_timeout=0.25, translation_scale=1.0, rotation_scale=1.0,
        control_rate=100.0, max_joint_speed=0.5, robot_state_wait_timeout=0.1,
        arm_source=source, operator=operator,
        debug_logger=SimpleNamespace(
            path=tmp_path / "ee_jitter.jsonl", record=slow_debug_record, close=lambda: None,
        ),
    )
    with pytest.raises(KeyboardInterrupt):
        teleop.run()
    assert closed == ["robot", "source"]
    assert len(commands) == 2
    for q, sides in commands:
        np.testing.assert_allclose(q, measured)
        assert sides == ("left", "right")
    records = [json.loads(line) for line in (tmp_path / "ee_jitter.loop_timing.jsonl").read_text().splitlines()]
    stalls = [r for r in records if r["event"] == "loop_stall"]
    assert any(["debug_log_and_fk", 650_000_000, 1_000_000] in r["current"]["stages"] for r in stalls)
    assert any(r["current"]["send"] and r["current"]["send"]["gap_ns"] == 650_000_000 for r in stalls)
