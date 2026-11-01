"""Optional MANUS runtime tuning; SDK calls remain on their original owner thread."""
import json
import os
from pathlib import Path
import socket
import time

from .normal_telemetry import DeferredTelemetry


SETTINGS = Path(__file__).resolve().parents[2] / 'config/sharpa_runtime.json'


def optimization_enabled():
    override = os.environ.get('SHARPA_RUNTIME_OPTIMIZATION')
    if override is not None:
        return override == '1'
    try:
        return json.loads(SETTINGS.read_text()).get('enabled') is True
    except (OSError, ValueError, AttributeError):
        # Missing or invalid settings use the original implementation.
        return False


def tune_udp_buffers(ports=(50011, 50021), requested_bytes=4 * 1024 * 1024):
    """Enlarge only this process's bound Sharpa UDP data sockets, never shrink.

    Linux reports twice SO_RCVBUF's requested value. No global sysctl, port
    rebinding, device connection, or socket ownership change is involved.
    """
    results = []
    for entry in list(Path('/proc/self/fd').iterdir()):
        duplicate = None
        try:
            duplicate = os.dup(int(entry.name))
            with socket.socket(fileno=duplicate) as handle:
                duplicate = None
                if handle.family != socket.AF_INET or handle.getsockopt(socket.SOL_SOCKET, socket.SO_TYPE) != socket.SOCK_DGRAM:
                    continue
                port = handle.getsockname()[1]
                if port not in ports:
                    continue
                before = handle.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
                if before < requested_bytes * 2:
                    handle.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, requested_bytes)
                after = handle.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
                results.append({'port': port, 'before': before, 'after': after})
        except OSError:
            # Some descriptors are regular files, and the OS may cap buffers.
            continue
        finally:
            if duplicate is not None:
                os.close(duplicate)
    return results


class RuntimeDiagnostics:
    """Bounded, off-thread logging; all device inspection stays on the owner."""
    def __init__(self, logger):
        self.logger = logger
        self.worker = DeferredTelemetry()
        self.last_notice = {}
        self.last_state_tick = None
        self.next_fault_check = time.monotonic() + 5
        self.faults = {}

    def warn(self, key, message):
        now = time.monotonic()
        if now - self.last_notice.get(key, -float('inf')) < 1:
            return
        self.last_notice[key] = now
        self.worker.submit(('runtime', key), lambda: self.logger.warning(message))

    def observe(self, operation, started):
        elapsed_ms = (time.perf_counter() - started) * 1000
        if elapsed_ms >= 10:
            self.warn(operation, f'SHARPA_TIMING {operation}: {elapsed_ms:.3f} ms')

    def state_tick(self):
        now = time.perf_counter()
        if self.last_state_tick is not None:
            gap_ms = (now - self.last_state_tick) * 1000
            if gap_ms >= 55:
                self.warn('state_interval', f'SHARPA_TIMING state callback interval: {gap_ms:.3f} ms')
        self.last_state_tick = now

    def inspect_faults(self, hands):
        now = time.monotonic()
        if now < self.next_fault_check:
            return
        self.next_fault_check = now + 5
        for side, hand in hands.items():
            wave = getattr(hand, '_wave', None)
            if wave is None or not hasattr(wave, 'get_fault_code'):
                continue
            started = time.perf_counter()
            try:
                faults = str(wave.get_fault_code())
                if faults != self.faults.get(side):
                    self.faults[side] = faults
                    message = f'SHARPA_FAULT {side.value}: {faults}'
                    self.worker.submit(('fault', side.value), lambda m=message: self.logger.info(m))
            except Exception as error:
                self.warn(('fault', side.value), f'SHARPA_FAULT {side.value}: inspection failed: {error}')
            finally:
                self.observe(f'{side.value}.get_fault_code', started)

    def close(self):
        self.worker.close()
