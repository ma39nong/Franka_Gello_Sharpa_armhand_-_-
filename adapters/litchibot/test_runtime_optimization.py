import json
import socket
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

from adapters.litchibot import runtime_optimization as runtime


def test_rollback_setting_and_environment(tmp_path, monkeypatch):
    settings=tmp_path/'runtime.json'
    monkeypatch.setattr(runtime,'SETTINGS',settings)
    monkeypatch.delenv('SHARPA_RUNTIME_OPTIMIZATION',raising=False)
    assert not runtime.optimization_enabled()
    settings.write_text(json.dumps({'enabled':True}))
    assert runtime.optimization_enabled()
    monkeypatch.setenv('SHARPA_RUNTIME_OPTIMIZATION','0')
    assert not runtime.optimization_enabled()
    settings.write_text(json.dumps({'enabled':False}))
    monkeypatch.setenv('SHARPA_RUNTIME_OPTIMIZATION','1')
    assert runtime.optimization_enabled()


def test_udp_tuning_preserves_socket_ownership_and_other_buffers():
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as target, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as other:
        target.bind(('127.0.0.1',0));other.bind(('127.0.0.1',0))
        port=target.getsockname()[1]
        original=other.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF)
        result=runtime.tune_udp_buffers(ports=(port,))
        assert len(result)==1 and result[0]['port']==port
        assert result[0]['after']>=result[0]['before']
        assert other.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF)==original
        # Duplicate inspection must leave the actual SDK socket usable.
        other.sendto(b'still owned',target.getsockname())
        target.settimeout(1)
        assert target.recv(64)==b'still owned'
        before=target.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF)
        runtime.tune_udp_buffers(ports=(port,),requested_bytes=4096)
        assert target.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF)==before


def test_slow_logger_does_not_block_owner_and_fault_reads_stay_on_owner():
    entered=threading.Event();release=threading.Event();owner=threading.get_ident();reads=[]
    def slow_log(message):
        entered.set();assert release.wait(2)
    logger=SimpleNamespace(warning=slow_log,info=Mock())
    diagnostics=runtime.RuntimeDiagnostics(logger)
    try:
        diagnostics.observe('left.send_action',time.perf_counter()-.02)
        assert entered.wait(1)
        diagnostics.observe('right.send_action',time.perf_counter()-.02)
        # A tiny enum has the driver's normal side semantics.
        from enum import Enum
        class Side(Enum):LEFT='left'
        wave=SimpleNamespace(get_fault_code=lambda:reads.append(threading.get_ident()) or {})
        diagnostics.next_fault_check=0
        diagnostics.inspect_faults({Side.LEFT:SimpleNamespace(_wave=wave)})
        assert reads==[owner]
    finally:
        release.set();diagnostics.close()
