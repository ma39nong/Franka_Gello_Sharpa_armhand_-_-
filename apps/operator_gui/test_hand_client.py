"""Real Qt/TCP opening requests across slow hardware preparation."""
import os
import threading
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PySide6.QtWidgets import QApplication

from adapters.litchibot.full_control import HandControlServer
from apps.operator_gui.hand_client import HandClient


@pytest.mark.parametrize('side', ['left', 'right', 'both'])
def test_open_refreshes_lease_and_heartbeats_resume_after_slow_reply(side):
    app = QApplication.instance() or QApplication([])
    server = HandControlServer(0, 'a' * 64)
    server.reply_timeout = None
    client = HandClient(server.server_address[1], 'a' * 64, normal_forwarding=True)
    stop = threading.Event()
    requests = []
    prepared = []
    errors = []
    client.event.connect(errors.append)

    def dispatch(request):
        requests.append(request)
        if request['command'] == 'open_hand':
            time.sleep(.35)
            prepared.append(time.monotonic())
        return {'normal_forwarding': True, 'hands': {}}

    def serve():
        while not stop.wait(.005):
            server.drain(dispatch, lambda: None)

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()

    def wait(predicate):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            app.processEvents()
            if predicate():return
            time.sleep(.002)
        raise AssertionError((requests, errors))

    try:
        wait(lambda: bool(requests))
        # Force the click to refresh the lease even with the periodic timer OFF.
        client.timer.stop()
        client.request('open_hand', {'side': side})
        wait(lambda: bool(prepared))
        index = next(i for i, r in enumerate(requests) if r['command'] == 'open_hand')
        assert requests[index - 1]['command'] == 'heartbeat'
        assert requests[index]['arguments'] == {'side': side}
        client.timer.start()
        wait(lambda: any(r['command'] == 'heartbeat' and r['monotonic_ns'] / 1e9 > prepared[0]
                         for r in requests[index + 1:]))
        assert not any('rejected' in error or 'offline' in error for error in errors)
    finally:
        client.close()
        app.processEvents()
        stop.set()
        thread.join(1)
        server.close()
