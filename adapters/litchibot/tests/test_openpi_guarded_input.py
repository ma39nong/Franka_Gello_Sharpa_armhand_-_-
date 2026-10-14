from types import SimpleNamespace
import time

import pytest

from adapters.litchibot.full_runtime import FullHandRuntime
from adapters.litchibot.retarget import JOINT_NAMES
from adapters.litchibot.validation_safety import RawContinuityGate


class FakeClock:
    @property
    def nanoseconds(self):
        return time.time_ns()


class FakeHost:
    def __init__(self):
        self.errors = []

    def get_clock(self):
        return SimpleNamespace(now=lambda: FakeClock())

    def stop_validation(self, error):
        self.errors.append(str(error))


def target(side="left", names=JOINT_NAMES, sequence=0):
    stamp = time.time_ns() - 10_000_000
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000),
            frame_id=f"openpi:{side}:session:{sequence}",
        ),
        name=list(names),
        position=[0.05] * 22,
    )


def test_openpi_uses_guarded_process_and_rejects_wrong_names():
    host = FakeHost()
    runtime = object.__new__(FullHandRuntime)
    runtime.node = host
    runtime.raw = RawContinuityGate()
    runtime.process = lambda side, packet, stamp: setattr(runtime, "accepted", (side, packet, stamp))
    runtime.openpi("left", target())
    assert not host.errors
    assert runtime.accepted[1]["session_id"] == "session"
    assert runtime.accepted[1]["sequence"] == 0
    runtime.openpi("left", target(names=list(JOINT_NAMES)[::-1], sequence=1))
    assert host.errors and "joint order" in host.errors[-1]
