"""Optional MANUS runtime tuning; SDK calls remain on their original owner thread."""
import json
import os
from pathlib import Path
import socket
import time

from .normal_telemetry import DeferredTelemetry


SETTINGS = Path(__file__).resolve().parents[2] / 'config/sharpa_runtime.json'


FEATURES = ('diagnostics_enabled', 'tactile_batching_enabled', 'udp_buffer_tuning_enabled')


def _settings():
    try:
        settings = json.loads(SETTINGS.read_text())
        return settings if isinstance(settings, dict) else {}
    except (OSError, ValueError):
        return {}


def _legacy_enabled(settings):
    override = os.environ.get('SHARPA_RUNTIME_OPTIMIZATION')
    return override == '1' if override is not None else settings.get('enabled') is True


def optimization_enabled():
    """Legacy umbrella value; individual features use optimization_settings()."""
    return _legacy_enabled(_settings())


def optimization_settings():
    """Explicit booleans override legacy defaults; environment 0 disables all.

    Preserve the old fail-closed behavior for other environment values too.
    Invalid explicit fields disable that feature rather than accepting truthy
    strings or numbers. Missing/invalid files use legacy defaults.
    """
    settings = _settings()
    override = os.environ.get('SHARPA_RUNTIME_OPTIMIZATION')
    if override is not None and override != '1':
        return dict.fromkeys(FEATURES, False)
    fallback = _legacy_enabled(settings)
    return {key: settings[key] is True if key in settings else fallback for key in FEATURES}


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
    """Existing bounded timing logs only; never queries a device or SDK."""
    def __init__(self, logger):
        self.logger = logger
        self.worker = DeferredTelemetry()
        self.last_notice = {}
        self.last_state_tick = None

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

    def close(self):
        self.worker.close()
