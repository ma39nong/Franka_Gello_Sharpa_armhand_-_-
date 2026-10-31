"""Retention for recognized runtime logs; recording directories are never candidates."""
from dataclasses import dataclass
import os
from pathlib import Path
import re
import stat
import time

RETENTION_DAYS = 30
RUN_NAME = re.compile(r'^\d{8}_\d{6}\.[A-Za-z0-9]+$')
ROS_RUN_NAME = re.compile(r'^\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}-')
PROJECT_NODE = re.compile(r'(?:manus_driver|sharpa|gello|teleop_|franka_control|franka-control)', re.I)
LOG_SUFFIXES = {'.log', '.jsonl'}


@dataclass
class CleanupResult:
    files: int = 0
    bytes: int = 0
    skipped: int = 0
    errors: int = 0

    def message(self):
        return (f'[日志清理] 保留最近 {RETENTION_DAYS} 天；已删除 {self.files} 个旧日志文件'
                f'（{self.bytes / 1024 / 1024:.1f} MiB），跳过 {self.skipped} 项，失败 {self.errors} 项')


def deployment(root):
    values = {}
    try:
        for line in (root/'docker/.env').read_text().splitlines():
            key, sep, value = line.partition('=')
            if sep: values[key.strip()] = value.strip().strip('\"\'')
    except OSError: pass
    return values


def open_files():
    """Use inode identity, also covering files opened through a different path."""
    opened = set()
    for process in Path('/proc').iterdir():
        if not process.name.isdigit(): continue
        try:
            for fd in (process/'fd').iterdir():
                try:
                    info = fd.stat()
                    if stat.S_ISREG(info.st_mode): opened.add((info.st_dev, info.st_ino))
                except OSError: pass
        except OSError: pass  # Other users/containers may hide their descriptors.
    return opened


def tree_files(path):
    """Never follow symlinks, and exclude a whole tree if it contains recording data."""
    files, directories = [], []
    for parent, dirs, names in os.walk(path, followlinks=False):
        parent = Path(parent)
        dirs[:] = [d for d in dirs if not (parent/d).is_symlink()]
        directories.append(parent)
        for name in names:
            item = parent/name
            if item.is_symlink(): continue
            if name in {'metadata.yaml', 'collection_state.json'} or item.suffix in {'.db3', '.mcap'}:
                return [], []
            files.append(item)
    return files, directories


def candidates(root, home, environ):
    values = deployment(root)
    data_root = Path(environ.get('TELEOP_DATA_ROOT') or values.get('TELEOP_DATA_ROOT') or home/'franka_teleop_data')
    diagnostics = Path(environ.get('TELEOP_DIAGNOSTICS_ROOT') or data_root/'diagnostics')
    # Only the timestamped run directories created by run_teleop.sh.
    if diagnostics.is_dir() and not diagnostics.is_symlink():
        for entry in diagnostics.iterdir():
            if entry.is_dir() and not entry.is_symlink() and RUN_NAME.fullmatch(entry.name):
                yield entry, 'diagnostics'
    ros_logs = Path(environ.get('ROS_LOG_DIR') or Path(environ.get('ROS_HOME', str(home/'.ros')))/'log')
    if ros_logs.is_dir() and not ros_logs.is_symlink():
        for entry in ros_logs.iterdir():
            if entry.is_symlink(): continue
            if entry.is_file() and entry.suffix == '.log' and PROJECT_NODE.search(entry.name):
                yield entry, 'file'
            elif entry.is_dir() and ROS_RUN_NAME.match(entry.name):
                launch = entry/'launch.log'
                try:
                    if launch.is_symlink(): continue
                    with launch.open('r', errors='replace') as stream: text = stream.read(65536)
                    if PROJECT_NODE.search(text): yield entry, 'ros'
                except OSError: continue
    sdk_log = root/'Log/OrbbecSDK.log.txt'
    if not sdk_log.parent.is_symlink() and sdk_log.is_file() and not sdk_log.is_symlink():
        yield sdk_log, 'file'


def cleanup_logs(root, *, home=None, environ=None, now=None, dry_run=False):
    root = Path(root)
    home = Path.home() if home is None else Path(home)
    environ = os.environ if environ is None else environ
    cutoff = (time.time() if now is None else now) - RETENTION_DAYS*86400
    result = CleanupResult()
    opened = open_files()
    for entry, kind in candidates(root, home, environ):
        try:
            files, directories = ([entry], []) if kind == 'file' else tree_files(entry)
            if not files: continue
            info = [(file, file.stat()) for file in files]
            # Keep a complete run if any part is recent, unknown, or still open.
            if any(not stat.S_ISREG(s.st_mode) or s.st_mtime > cutoff or (s.st_dev,s.st_ino) in opened
                   or (kind != 'file' and p.suffix not in LOG_SUFFIXES and not p.name.endswith('.writer_status.json'))
                   for p,s in info):
                result.skipped += 1
                continue
            for file, before in info:
                current = file.lstat()
                if (current.st_ino, current.st_dev, current.st_mtime_ns, current.st_size) != (
                    before.st_ino, before.st_dev, before.st_mtime_ns, before.st_size):
                    result.skipped += 1
                    continue
                if not dry_run: file.unlink()
                result.files += 1
                result.bytes += before.st_size
            if not dry_run:
                for directory in reversed(directories):
                    try: directory.rmdir()
                    except OSError: pass  # Never remove unexamined files or symlinks.
        except OSError:
            result.errors += 1
    return result
