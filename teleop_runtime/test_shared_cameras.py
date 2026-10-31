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
    run.reset_mock()
    assert not full_launcher.start_camera_preview(full_launcher.parser().parse_args(['--hardware','--no-cameras']))
    assert not full_launcher.start_camera_preview(full_launcher.parser().parse_args(['--dry-run']))
    run.assert_not_called()
    run.side_effect=subprocess.CalledProcessError(1,command)
    assert not full_launcher.start_camera_preview(full_launcher.parser().parse_args(['--hardware']))
