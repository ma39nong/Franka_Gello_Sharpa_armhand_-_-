"""Deletion boundaries use disposable directories; never touch real user logs."""
import os
from pathlib import Path

from apps.operator_gui.log_retention import cleanup_logs

NOW = 1800000000
OLD = NOW-31*86400


def write(path, timestamp=OLD):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('old runtime log')
    os.utime(path, (timestamp, timestamp))
    return path


def test_retention_deletes_only_closed_recognized_logs(tmp_path):
    root, home = tmp_path/'repo', tmp_path/'home'
    diagnostics = home/'franka_teleop_data/diagnostics'
    old = write(diagnostics/'20260901_120000.abc/ee_jitter.jsonl', NOW-30*86400)
    recent = write(diagnostics/'20261020_120000.abc/ee_jitter.jsonl', NOW-29*86400)
    opened = write(diagnostics/'20260901_120000.open/ee_jitter.jsonl')
    mixed = write(diagnostics/'20260901_120000.mixed/ee_jitter.jsonl')
    write(mixed.parent/'ee_jitter.writer_status.json', NOW)
    unknown = write(diagnostics/'20260901_120000.unknown/notes.md')
    bag = write(diagnostics/'20260901_120000.bag/ee_jitter.jsonl')
    write(bag.parent/'metadata.yaml')
    actual_episode = write(home/'franka_teleop_data/bags/episode0/episode0.db3')
    sdk = write(root/'Log/OrbbecSDK.log.txt')
    with opened.open('rb'):
        result = cleanup_logs(root,home=home,environ={},now=NOW)
    assert result.files==2
    assert not old.exists() and not sdk.exists()
    assert all(p.exists() for p in (recent,opened,mixed,unknown,bag,actual_episode))
    assert not old.parent.exists()


def test_ros_filter_symlinks_and_dry_run(tmp_path):
    home, root = tmp_path/'home', tmp_path/'repo'
    logs = home/'.ros/log'
    ros = write(logs/'2026-09-01-12-00-00-session/launch.log')
    ros.write_text('[manus_driver-1] started')
    os.utime(ros,(OLD,OLD))
    other = write(logs/'2026-09-01-12-00-00-other/launch.log')
    node = write(logs/'sharpa_driver_123.log')
    unrelated = write(logs/'super_lio_node_123.log')
    outside = write(tmp_path/'outside/valuable.log')
    (logs/'teleop_link.log').symlink_to(outside)
    (logs/'latest').symlink_to(ros.parent, target_is_directory=True)
    dry = cleanup_logs(root,home=home,environ={},now=NOW,dry_run=True)
    assert dry.files==2 and ros.exists() and node.exists()
    result = cleanup_logs(root,home=home,environ={},now=NOW)
    assert result.files==2 and not ros.exists() and not node.exists()
    assert outside.exists() and unrelated.exists() and other.exists()
    assert (logs/'latest').is_symlink()


def test_configured_diagnostic_path_and_incomplete_ros_bag_are_preserved(tmp_path):
    diagnostics = tmp_path/'custom'
    log = write(diagnostics/'20260901_120000.abc/hand_fidelity.jsonl')
    write(log.parent/'video.mcap')
    root = tmp_path/'repo'
    result = cleanup_logs(root,home=tmp_path/'home',environ={'TELEOP_DIAGNOSTICS_ROOT':str(diagnostics)},now=NOW)
    assert result.files==0 and log.exists()
