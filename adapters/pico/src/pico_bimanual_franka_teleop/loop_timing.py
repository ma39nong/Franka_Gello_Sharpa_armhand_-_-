"""Temporary, anomaly-only diagnostics for the GELLO command-loop stalls.

Remove these probes after the cause is fixed and verified. They observe timing;
they never resend commands, change engagement, or relax freshness checks.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import queue
import sys
import threading
import time


class LoopTimingDiagnostics:
    def __init__(self, path, *, session_id, period_sec, threshold_sec=0.1):
        self.path = Path(path)
        self.session_id = session_id
        self.period_sec = period_sec
        self.threshold_ns = int(threshold_sec * 1e9)
        self._queue = queue.Queue(maxsize=64)
        self._closed = threading.Event()
        self._failed = False
        self._started_ns = None
        self._last_send_ns = None
        self._last_sequence = None
        self._previous = None
        self.cycles = 0
        self.anomalies = 0
        self.dropped = 0
        self._thread = threading.Thread(
            target=self._write, name="loop-timing-writer", daemon=True,
        )
        self._thread.start()

    def begin_cycle(self):
        now = time.monotonic_ns()
        cpu = time.thread_time_ns()
        self._finish_cycle(now, cpu)
        self._started_ns = self._checkpoint_ns = now
        self._checkpoint_cpu_ns = cpu
        self._stage = "service_requests"
        self._stages = []
        self._send = None

    def mark(self, next_stage):
        """Finish the previous stage, then label the work about to run."""
        now = time.monotonic_ns()
        cpu = time.thread_time_ns()
        self._checkpoint(now, cpu)
        self._stage = next_stage

    def command_sent(self, sequence, active_sides):
        now = time.monotonic_ns()
        cpu = time.thread_time_ns()
        self._checkpoint(now, cpu)
        self._stage = "after_send"
        self._send = {
            "previous_monotonic_ns": self._last_send_ns,
            "monotonic_ns": now,
            "previous_sequence": self._last_sequence,
            "sequence": sequence,
            "active_sides": tuple(active_sides),
            "gap_ns": 0 if self._last_send_ns is None else now - self._last_send_ns,
        }
        self._last_send_ns = now
        self._last_sequence = sequence

    def _checkpoint(self, now, cpu):
        self._stages.append((
            self._stage, now - self._checkpoint_ns, cpu - self._checkpoint_cpu_ns,
        ))
        self._checkpoint_ns = now
        self._checkpoint_cpu_ns = cpu

    def _finish_cycle(self, now, cpu):
        if self._started_ns is None:
            return
        self._checkpoint(now, cpu)
        self.cycles += 1
        cycle = {
            "cycle": self.cycles,
            "start_monotonic_ns": self._started_ns,
            "end_monotonic_ns": now,
            "elapsed_ns": now - self._started_ns,
            "stages": self._stages,
            "send": self._send,
        }
        gap = 0 if self._send is None else self._send["gap_ns"]
        if max(cycle["elapsed_ns"], gap) > self.threshold_ns:
            self.anomalies += 1
            # Wall/monotonic anchor ties this event to the episode's ROS stamps.
            # Include the previous cycle because a send gap can span its tail.
            event = {
                "event": "loop_stall", "session_id": self.session_id,
                "wall_time_ns": time.time_ns(), "monotonic_ns": now,
                "current": cycle, "previous": self._previous,
                "dropped_events": self.dropped,
            }
            if not self._failed:
                try:
                    self._queue.put_nowait(event)
                except queue.Full:
                    self.dropped += 1
        self._previous = cycle

    def end_cycle(self):
        self._finish_cycle(time.monotonic_ns(), time.thread_time_ns())
        self._started_ns = None

    def close(self):
        if self._closed.is_set():
            return
        self.end_cycle()
        self._closed.set()
        # Called after hardware cleanup. A stuck disk must not prevent exit.
        self._thread.join(timeout=0.2)

    def _write(self):
        try:
            # All file operations and terminal output stay on this worker.
            with self.path.open("a", encoding="utf-8") as stream:
                header = {
                    "schema": "loop-timing.v1", "event": "start",
                    "pid": os.getpid(), "session_id": self.session_id,
                    "wall_time_ns": time.time_ns(),
                    "monotonic_ns": time.monotonic_ns(),
                    "period_sec": self.period_sec,
                    "threshold_ns": self.threshold_ns,
                    "stage_columns": ["name", "elapsed_ns", "thread_cpu_ns"],
                }
                stream.write(json.dumps(header) + "\n")
                stream.flush()
                print(f"[loop timing] Temporary diagnostics -> {self.path}", flush=True)
                last_notice = float("-inf")
                while not self._closed.is_set() or not self._queue.empty():
                    try:
                        event = self._queue.get(timeout=0.05)
                    except queue.Empty:
                        continue
                    stream.write(json.dumps(event, separators=(",", ":")) + "\n")
                    stream.flush()
                    now = time.monotonic()
                    if now - last_notice >= 5.0:
                        cycle = event["current"]
                        worst = max(cycle["stages"], key=lambda stage: stage[1])
                        send = cycle["send"]
                        gap_ms = 0.0 if send is None else send["gap_ns"] / 1e6
                        print(
                            f"[loop timing] Slow cycle {cycle['elapsed_ns'] / 1e6:.1f} ms; "
                            f"send gap {gap_ms:.1f} ms; longest stage {worst[0]} "
                            f"{worst[1] / 1e6:.1f} ms. Details: {self.path}",
                            flush=True,
                        )
                        last_notice = now
                stream.write(json.dumps({
                    "event": "stop", "session_id": self.session_id,
                    "cycles": self.cycles, "anomalies": self.anomalies,
                    "dropped_events": self.dropped,
                }) + "\n")
        except Exception as error:  # Diagnostics must not stop robot control.
            self._failed = True
            print(f"[loop timing] Diagnostics disabled: {error}", file=sys.stderr, flush=True)
