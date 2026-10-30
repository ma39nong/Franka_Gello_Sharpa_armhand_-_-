"""Takeover lifecycle checks, with all Docker and port operations mocked."""
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock
import subprocess

import pytest

from teleop_runtime import dds_buffers, full_launcher


@pytest.fixture
def environment(monkeypatch):
    monkeypatch.setattr(dds_buffers, 'check_receive_buffer', Mock())
    monkeypatch.setattr(full_launcher, 'deployment_port', lambda root, key, default: default)
    ports = MagicMock()
    monkeypatch.setattr(full_launcher.socket, 'socket', ports)
    return ports


def docker(monkeypatch, before, after='', failure=None):
    snapshots = iter((before, after))
    def execute(command, **kwargs):
        if command[2] == 'ps':
            return SimpleNamespace(stdout=next(snapshots))
        assert command[:5] == ['docker', 'compose', 'stop', '--timeout', '30']
        if failure:raise failure
        return SimpleNamespace(returncode=0)
    run = Mock(side_effect=execute)
    monkeypatch.setattr(full_launcher.subprocess, 'run', run)
    return run


def stops(run):
    return [call.args[0] for call in run.call_args_list if call.args[0][2] == 'stop']


def test_takeover_requires_explicit_flag(environment, monkeypatch):
    run = docker(monkeypatch, 'franka-control\nteleop-control\nhand-control\n')
    with pytest.raises(RuntimeError, match='use --takeover'):
        full_launcher.preflight(5593)
    assert stops(run) == []
    assert full_launcher.parser().parse_args(['--takeover']).takeover is True


def test_takeover_stops_arm_and_hand_producers_before_franka_and_keeps_cameras(environment, monkeypatch):
    run = docker(monkeypatch, 'franka-control\nteleop-control\nhand-control\nopenpi-cameras\ncollector\n',
                 'openpi-cameras\ncollector\n')
    full_launcher.preflight(5593, takeover=True)
    assert stops(run) == [
        ['docker','compose','stop','--timeout','30','hand-control','teleop-control'],
        ['docker','compose','stop','--timeout','30','franka-control'],
    ]
    assert all(call.kwargs['cwd'] == full_launcher.ROOT/'docker' for call in run.call_args_list)
    assert len([call for call in run.call_args_list if call.args[0][2]=='ps']) == 2


@pytest.mark.parametrize('blocker', sorted(full_launcher.TAKEOVER_BLOCKERS))
def test_takeover_refuses_other_motion_owner_before_stopping_anything(environment, monkeypatch, blocker):
    run = docker(monkeypatch, f'franka-control\nhand-control\n{blocker}\n')
    with pytest.raises(RuntimeError, match='MoveIt/UI'):
        full_launcher.preflight(5593, takeover=True)
    assert stops(run) == []


def test_takeover_checks_ownership_released(environment, monkeypatch):
    docker(monkeypatch, 'franka-control\n', 'franka-control\n')
    with pytest.raises(RuntimeError, match='did not release'):
        full_launcher.preflight(5593, takeover=True)


@pytest.mark.parametrize('failure', [
    subprocess.CalledProcessError(1, ['docker','compose','stop']),
    subprocess.TimeoutExpired(['docker','compose','stop'],45),
])
def test_stop_failure_aborts_before_stopping_hardware_owner(environment, monkeypatch, failure):
    run = docker(monkeypatch, 'teleop-control\nhand-control\nfranka-control\n', failure=failure)
    with pytest.raises((RuntimeError,subprocess.CalledProcessError)):
        full_launcher.preflight(5593, takeover=True)
    assert len(stops(run)) == 1 and 'franka-control' not in stops(run)[0]


def test_live_host_operator_port_is_not_force_killed(environment, monkeypatch):
    environment.return_value.__enter__.return_value.bind.side_effect = OSError('in use')
    run = docker(monkeypatch, 'franka-control\n')
    with pytest.raises(RuntimeError, match='Control port'):
        full_launcher.preflight(5593, takeover=True)
    run.assert_not_called()


def test_takeover_without_existing_controllers_does_not_stop_independent_services(environment, monkeypatch):
    run = docker(monkeypatch, 'openpi-cameras\ncollector\n')
    full_launcher.preflight(5593, takeover=True)
    assert stops(run) == []
