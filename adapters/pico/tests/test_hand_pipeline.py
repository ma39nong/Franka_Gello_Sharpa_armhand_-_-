import json
import socket
from pathlib import Path

import numpy as np
import pytest
from hand_fixtures import synthetic_skeleton

from pico_bimanual_franka_teleop.hand_teleop import HandPipeline

REPO_ROOT = Path(__file__).resolve().parents[3]
ASSETS = REPO_ROOT / "assets"


class FakeXrt:
    """Serves synthetic skeletons with optical-style jitter on every read.

    The jitter matters: the liveness gate treats a bitwise-constant skeleton as a
    frozen cache, because measured PICO output never repeats a frame exactly.
    """

    def __init__(self) -> None:
        self.left_active = 1
        self.right_active = 1
        self.raise_on_read = False
        self._count = 0

    def _jittered(self, mirror: bool) -> np.ndarray:
        skeleton = synthetic_skeleton(mirror=mirror)
        self._count += 1
        skeleton[:, 0] += 1e-5 * self._count
        return skeleton

    def get_left_hand_tracking_state(self):
        if self.raise_on_read:
            raise RuntimeError("SDK exploded")
        return self._jittered(mirror=True)

    def get_right_hand_tracking_state(self):
        if self.raise_on_read:
            raise RuntimeError("SDK exploded")
        return self._jittered(mirror=False)

    def get_left_hand_is_active(self):
        return self.left_active

    def get_right_hand_is_active(self):
        return self.right_active


@pytest.fixture
def sink():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(0.5)
    yield sock
    sock.close()


def make_pipeline(xrt, sink, **kwargs):
    return HandPipeline(
        xrt,
        assets_root=ASSETS,
        host="127.0.0.1",
        port=sink.getsockname()[1],
        **kwargs,
    )


def drive(pipeline, ticks, start=100.0, dt=0.01):
    for step in range(ticks):
        pipeline.tick(start + step * dt)


def drain(sink):
    messages = []
    while True:
        try:
            payload, _ = sink.recvfrom(65536)
        except socket.timeout:
            return messages
        messages.append(json.loads(payload.decode()))


def test_pipeline_validates_its_arguments(sink):
    with pytest.raises(ValueError):
        make_pipeline(None, sink)
    with pytest.raises(ValueError):
        make_pipeline(FakeXrt(), sink, rate=0.0)
    with pytest.raises(ValueError):
        make_pipeline(FakeXrt(), sink, rate=120.0)
    with pytest.raises(ValueError):
        make_pipeline(FakeXrt(), sink, sides=("middle",))
    with pytest.raises(ValueError, match="registered"):
        make_pipeline(
            FakeXrt(),
            sink,
            sides=("left",),
            models={"left": "o30"},
        )


def test_pipeline_sends_valid_commands_for_both_sides(sink):
    pipeline = make_pipeline(FakeXrt(), sink)
    try:
        drive(pipeline, 40)
    finally:
        pipeline.close()
    messages = drain(sink)
    by_side = {"left": [], "right": []}
    for message in messages:
        by_side[message["side"]].append(message)
    for side, side_messages in by_side.items():
        assert side_messages, f"nothing sent for {side}"
        last = side_messages[-1]
        assert last["model"] == "g20"
        assert len(last["joint_names"]) == 21
        expected_tip = "thumb_ip" if side == "left" else "thumb_dip"
        assert expected_tip in last["joint_names"]
        qpos = np.asarray(last["qpos"], dtype=float)
        assert np.isfinite(qpos).all()
        sequences = [m["sequence"] for m in side_messages]
        assert all(b > a for a, b in zip(sequences, sequences[1:]))
    status = pipeline.status
    assert status.errors == 0
    assert all(s.sending for s in status.sides.values())


def test_pipeline_respects_the_send_rate(sink):
    pipeline = make_pipeline(FakeXrt(), sink, rate=30.0)
    try:
        # 100 ticks over one simulated second at 100 Hz.
        drive(pipeline, 100, dt=0.01)
    finally:
        pipeline.close()
    messages = drain(sink)
    per_side = {"left": 0, "right": 0}
    for message in messages:
        per_side[message["side"]] += 1
    for side, count in per_side.items():
        # Preserve the fractional 33.3 ms deadline on a 10 ms owner loop.
        # Re-basing every deadline on the current tick silently produced 25 Hz.
        assert 29 <= count <= 31, (side, count)


def test_inactive_side_stops_sending_and_reports_a_fault(sink):
    xrt = FakeXrt()
    pipeline = make_pipeline(xrt, sink)
    try:
        drive(pipeline, 20)
        drain(sink)
        xrt.left_active = 0
        drive(pipeline, 20, start=101.0)
        messages = drain(sink)
    finally:
        pipeline.close()
    assert all(m["side"] == "right" for m in messages)
    assert messages, "the healthy side must keep sending"
    assert pipeline.status.sides["left"].sending is False
    assert "inactive" in pipeline.status.sides["left"].fault
    assert pipeline.status.sides["right"].sending is True


def test_sdk_failure_is_contained(sink):
    # A throwing SDK read is caught per side inside the reader, surfaces as a
    # side fault, and stops that side's sending. It never raises out of tick()
    # and never becomes a pipeline-level error.
    xrt = FakeXrt()
    pipeline = make_pipeline(xrt, sink)
    try:
        drive(pipeline, 10)
        xrt.raise_on_read = True
        drive(pipeline, 10, start=101.0)
    finally:
        pipeline.close()
    for side in ("left", "right"):
        status = pipeline.status.sides[side]
        assert status.sending is False
        assert "SDK read failed" in status.fault


def test_disengaged_side_stops_sending(sink):
    # One keyboard drives arm and hand together: a side whose arm is off must
    # stop commanding its hand, so the bridge watchdog holds it.
    pipeline = make_pipeline(FakeXrt(), sink)
    try:
        for step in range(40):
            pipeline.tick(100.0 + step * 0.01, active={"left": False, "right": True})
        messages = drain(sink)
    finally:
        pipeline.close()
    assert messages, "the engaged side must keep sending"
    assert all(m["side"] == "right" for m in messages)
    assert pipeline.status.sides["left"].sending is False
    assert pipeline.status.sides["left"].fault == "disengaged by operator"
    assert pipeline.status.sides["right"].sending is True


def test_request_open_streams_the_open_pose(sink):
    pipeline = make_pipeline(FakeXrt(), sink)
    inactive = {"left": False, "right": False}
    try:
        pipeline.request_open(now=100.0, duration=2.0)
        for step in range(40):
            pipeline.tick(100.0 + step * 0.01, active=inactive)
        open_messages = drain(sink)
        # Past the open deadline nothing may be sent for a disengaged side.
        for step in range(40):
            pipeline.tick(103.0 + step * 0.01, active=inactive)
        after_messages = drain(sink)
    finally:
        pipeline.close()
    assert open_messages, "the open pose must be streamed"
    sides_seen = {m["side"] for m in open_messages}
    assert sides_seen == {"left", "right"}
    for message in open_messages:
        assert message["stream_id"].endswith("-open")
        assert all(value == 0.0 for value in message["qpos"])
    assert not after_messages


def test_following_supersedes_a_pending_open(sink):
    pipeline = make_pipeline(FakeXrt(), sink)
    engaged = {"left": True, "right": True}
    try:
        # Warm up until both skeletons are live and following.
        for step in range(10):
            pipeline.tick(100.0 + step * 0.01, active=engaged)
        drain(sink)
        pipeline.request_open(now=100.2, duration=2.0)
        for step in range(40):
            pipeline.tick(100.2 + step * 0.01, active=engaged)
        messages = drain(sink)
    finally:
        pipeline.close()
    assert messages
    assert all(not m["stream_id"].endswith("-open") for m in messages)


def test_at_most_one_solve_per_tick(sink):
    pipeline = make_pipeline(FakeXrt(), sink, rate=60.0)
    try:
        # Warm up until both sides are live.
        drive(pipeline, 10)
        drain(sink)
        before = {s: pipeline.status.sides[s].sent for s in ("left", "right")}
        pipeline.tick(200.0)
        after = {s: pipeline.status.sides[s].sent for s in ("left", "right")}
    finally:
        pipeline.close()
    total = sum(after.values()) - sum(before.values())
    assert total <= 1, "a tick must never pay for two solves"


def test_sides_alternate_rather_than_starve(sink):
    pipeline = make_pipeline(FakeXrt(), sink, rate=60.0)
    try:
        drive(pipeline, 60)
    finally:
        pipeline.close()
    sent = {s: pipeline.status.sides[s].sent for s in ("left", "right")}
    assert sent["left"] > 0 and sent["right"] > 0
    ratio = max(sent.values()) / max(1, min(sent.values()))
    assert ratio < 2.0, f"one side is being starved: {sent}"


def test_request_open_for_one_side_only(sink):
    pipeline = make_pipeline(FakeXrt(), sink)
    inactive = {"left": False, "right": False}
    try:
        pipeline.request_open(now=100.0, duration=2.0, sides=("left",))
        for step in range(40):
            pipeline.tick(100.0 + step * 0.01, active=inactive)
        messages = drain(sink)
    finally:
        pipeline.close()
    assert messages, "the selected side must stream its open pose"
    assert {message["side"] for message in messages} == {"left"}
