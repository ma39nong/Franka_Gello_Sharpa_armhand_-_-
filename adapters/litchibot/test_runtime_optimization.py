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


def test_slow_logger_does_not_block_owner_and_has_no_fault_polling():
    entered=threading.Event();release=threading.Event()
    def slow_log(message):
        entered.set();assert release.wait(2)
    logger=SimpleNamespace(warning=slow_log,info=Mock())
    diagnostics=runtime.RuntimeDiagnostics(logger)
    try:
        diagnostics.observe('left.send_action',time.perf_counter()-.02)
        assert entered.wait(1)
        diagnostics.observe('right.send_action',time.perf_counter()-.02)
        assert not hasattr(diagnostics, 'inspect_faults')
        assert not hasattr(diagnostics, 'next_fault_check')
        assert not hasattr(diagnostics, 'faults')
    finally:
        release.set();diagnostics.close()


def test_independent_settings_compatibility(tmp_path, monkeypatch):
    settings=tmp_path/'runtime.json'
    monkeypatch.setattr(runtime,'SETTINGS',settings)
    monkeypatch.delenv('SHARPA_RUNTIME_OPTIMIZATION',raising=False)
    def check(config, expected):
        settings.write_text(json.dumps(config))
        assert runtime.optimization_settings()==dict(zip(runtime.FEATURES,expected))
    check({'enabled':True},(True,True,True))
    check({'enabled':False},(False,False,False))
    check({'enabled':False,'diagnostics_enabled':True,'udp_buffer_tuning_enabled':True},(True,False,True))
    check({'enabled':True,'diagnostics_enabled':False,'tactile_batching_enabled':False},(False,False,True))
    monkeypatch.setenv('SHARPA_RUNTIME_OPTIMIZATION','1')
    check({'enabled':False,'diagnostics_enabled':False},(False,True,True))
    monkeypatch.setenv('SHARPA_RUNTIME_OPTIMIZATION','0')
    check(dict.fromkeys(runtime.FEATURES,True),(False,False,False))
    monkeypatch.setenv('SHARPA_RUNTIME_OPTIMIZATION','invalid')
    check({'enabled':True},(False,False,False))


def test_invalid_settings_are_fail_closed(tmp_path,monkeypatch):
    settings=tmp_path/'runtime.json';monkeypatch.setattr(runtime,'SETTINGS',settings)
    monkeypatch.delenv('SHARPA_RUNTIME_OPTIMIZATION',raising=False)
    for contents in (None,'{broken','[]','null','true','42','{}'):
        if contents is None:settings.unlink(missing_ok=True)
        else:settings.write_text(contents)
        assert not any(runtime.optimization_settings().values())
    settings.write_text(json.dumps({'enabled':True, 'diagnostics_enabled':'true',
        'tactile_batching_enabled':1,'udp_buffer_tuning_enabled':None}))
    assert not any(runtime.optimization_settings().values())
    settings.unlink();monkeypatch.setenv('SHARPA_RUNTIME_OPTIMIZATION','1')
    assert all(runtime.optimization_settings().values())
