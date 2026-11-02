"""Camera owner reuse checks without starting Docker or robot processes."""
import json
import os
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('running,mismatch,blocked,expected', [
    (True,False,False,0), (True,True,False,1),
    (False,False,False,0), (False,False,True,1),
])
def test_shared_camera_script_reuses_or_refuses_without_restarting(tmp_path, running, mismatch, blocked, expected):
    fake = tmp_path/'docker'
    fake.write_text('''#!/usr/bin/env python3
import os,sys,json
with open(os.environ['CALLS'],'a') as f:f.write(json.dumps(sys.argv[1:])+'\\n')
a=sys.argv[1:]
if a[:3]==['compose','ps','-q'] and a[-1]=='teleop-cameras' and os.environ['RUNNING']=='1':print('camera-id')
elif a[0]=='inspect':print('different' if os.environ['MISMATCH']=='1' else 'TELEOP_CAMERA_CONFIG_HASH='+os.environ['TELEOP_CAMERA_CONFIG_HASH'])
elif a[0]=='ps' and os.environ['BLOCKED']=='1':print('existing-legacy-collector')
''')
    fake.chmod(0o755)
    pgrep=tmp_path/'pgrep';pgrep.write_text('#!/bin/sh\nexit 1\n');pgrep.chmod(0o755)
    cfg=tmp_path/'cameras.yaml';cfg.write_text('camera_bringup: {}')
    calls=tmp_path/'calls'
    result=subprocess.run([str(ROOT/'ops/run/start_teleop_cameras.sh'),str(cfg)],capture_output=True,text=True,
        env={**os.environ,'PATH':str(tmp_path)+':'+os.environ['PATH'], 'ORBBEC_SDK_LICENSE_ACCEPTED':'YES',
             'RUNNING':str(int(running)),'MISMATCH':str(int(mismatch)),'BLOCKED':str(int(blocked)),'CALLS':str(calls)})
    assert result.returncode==expected,result.stderr
    commands=[json.loads(line) for line in calls.read_text().splitlines()]
    starts=[a for a in commands if a[:2]==['compose','up']]
    assert bool(starts)==(not running and not blocked)
    assert not any('stop' in a or 'down' in a or 'restart' in a for a in commands)
    assert all('franka-control' not in a and 'hand-control' not in a for a in commands)
    if running and not mismatch:assert 'Reusing shared' in result.stdout
    if expected==0:assert any(a[:3]==['compose','exec','-T'] for a in commands)


def test_full_camera_start_is_optional_and_does_not_replace_motion_wrappers(monkeypatch):
    from unittest.mock import Mock
    from teleop_runtime import full_launcher
    run=Mock()
    monkeypatch.setattr(full_launcher.subprocess,'run',run)
    assert full_launcher.start_camera_preview(full_launcher.parser().parse_args(['--hardware']))
    command=run.call_args.args[0]
    assert command==[str(ROOT/'ops/run/start_teleop_cameras.sh')]
    assert full_launcher.start_camera_preview(full_launcher.parser().parse_args(['--hardware','--takeover']))
    assert run.call_args.args[0]==[str(ROOT/'ops/run/start_teleop_cameras.sh'),'--takeover']
    run.reset_mock()
    assert not full_launcher.start_camera_preview(full_launcher.parser().parse_args(['--hardware','--no-cameras']))
    assert not full_launcher.start_camera_preview(full_launcher.parser().parse_args(['--dry-run']))
    run.assert_not_called()
    run.side_effect=subprocess.CalledProcessError(1,command)
    assert not full_launcher.start_camera_preview(full_launcher.parser().parse_args(['--hardware']))

@pytest.mark.parametrize('takeover,probe,retry,block,expected,rebuilds', [
    (True,3,0,'',0,1), (True,3,3,'',3,1), (True,1,0,'',1,0), (True,130,0,'',130,0),
    (True,3,0,'collector',1,0), (True,3,0,'port',1,0), (True,3,0,'unknown_port',1,0),
    (True,3,0,'docker',1,0), (True,3,0,'viewer',1,0), (True,3,0,'lease',1,0),
    (False,3,0,'',3,0), (False,0,0,'',0,0), (True,0,0,'',0,0),
])
def test_startup_recovery_is_bounded_and_protects_collectors(tmp_path, takeover, probe, retry, block, expected, rebuilds):
    import fcntl
    import sys
    fake=tmp_path/'docker'
    fake.write_text(f'#!{sys.executable}\n'+'''import os,sys,json
from pathlib import Path
a=sys.argv[1:]
with open(os.environ['CALLS'],'a') as f:f.write(json.dumps(a)+'\\n')
if a[:3]==['compose','ps','-q'] and a[-1]=='teleop-cameras':print('camera-id')
elif a[0]=='inspect':print('TELEOP_CAMERA_CONFIG_HASH='+os.environ['TELEOP_CAMERA_CONFIG_HASH'])
elif a[0]=='ps':
 if os.environ['BLOCK']=='collector':print('collector')
 if os.environ['BLOCK']=='docker':sys.exit(1)
elif a[:3]==['compose','exec','-T']:
 p=Path(os.environ['COUNTER']);count=int(p.read_text()) if p.exists() else 0;p.write_text(str(count+1))
 sys.exit(int(os.environ['PROBE'] if count==0 else os.environ['RETRY']))
''');fake.chmod(0o755)
    pgrep=tmp_path/'pgrep';pgrep.write_text('#!/bin/sh\n[ "$BLOCK" = viewer ]\n');pgrep.chmod(0o755)
    # Host ownership check is mocked here; its socket outcomes are tested below.
    python=tmp_path/'python3';python.write_text('#!/bin/sh\ncase "$BLOCK" in port|unknown_port) exit 1;; *) exit 0;; esac\n');python.chmod(0o755)
    # Use an isolated UID for locks so tests never interact with hardware startup.
    uid=200000+os.getpid()
    fake_id=tmp_path/'id';fake_id.write_text(f'#!/bin/sh\necho {uid}\n');fake_id.chmod(0o755)
    cfg=tmp_path/'cameras.yaml';cfg.write_text('camera_bringup: {}')
    lease=open(f'/tmp/gello-camera-use-{uid}.lock','a')
    if block=='lease':fcntl.flock(lease,fcntl.LOCK_SH)
    try:
        result=subprocess.run([str(ROOT/'ops/run/start_teleop_cameras.sh'),str(cfg)]+(['--takeover'] if takeover else []),capture_output=True,text=True,
            env={**os.environ,'PATH':str(tmp_path)+':'+os.environ['PATH'],'ORBBEC_SDK_LICENSE_ACCEPTED':'YES',
                 'CALLS':str(tmp_path/'calls'),'COUNTER':str(tmp_path/'counter'),'PROBE':str(probe),'RETRY':str(retry),'BLOCK':block},timeout=5)
    finally:lease.close()
    assert result.returncode==expected,result.stderr
    commands=[json.loads(line) for line in (tmp_path/'calls').read_text().splitlines()]
    assert sum('--force-recreate' in a for a in commands)==rebuilds
    for a in commands:
        if '--force-recreate' in a:assert a==['compose','up','-d','--no-deps','--force-recreate','teleop-cameras']
        assert not any(service in a for service in ('franka-control','teleop-control','gello-bridge'))
    waits=[a for a in commands if a[:3]==['compose','exec','-T']]
    assert len(waits)==1+rebuilds
    assert waits[0][waits[0].index('--timeout-sec')+1]=='8'
    if rebuilds:assert waits[1][waits[1].index('--timeout-sec')+1]=='90'
    if not takeover and probe==3:assert '--takeover' in result.stderr


def test_camera_recovery_guard_fails_closed(monkeypatch):
    import importlib.util
    import errno
    spec=importlib.util.spec_from_file_location('camera_recovery_guard',ROOT/'ops/lib/camera_recovery_guard.py')
    guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
    def refused(*args,**kwargs):raise ConnectionRefusedError(errno.ECONNREFUSED,'refused')
    monkeypatch.setattr(guard.socket,'create_connection',refused)
    assert guard.main()==0
    def uncertain(*args,**kwargs):raise TimeoutError('timed out')
    monkeypatch.setattr(guard.socket,'create_connection',uncertain)
    assert guard.main()==1
    from unittest.mock import MagicMock
    monkeypatch.setattr(guard.socket,'create_connection',MagicMock())
    assert guard.main()==1
    monkeypatch.setenv('COLLECTION_CONTROL_PORT','invalid')
    assert guard.main()==1
