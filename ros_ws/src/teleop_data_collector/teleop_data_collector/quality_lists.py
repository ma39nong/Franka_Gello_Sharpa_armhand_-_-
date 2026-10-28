"""Human quality labels, stored as converter-compatible episode number lists."""

from __future__ import annotations

import fcntl
import os
from pathlib import Path
import re
import tempfile


DEFAULT_QUALITY_DIR = Path("/home/user/franka_teleop_data/数据分类")
QUALITY_LABELS = ("优等", "一般", "报错", "放弃")


def _read_numbers(path: Path) -> tuple[list[str], set[int]]:
    comments: list[str] = []
    numbers: set[int] = set()
    if not path.exists():
        return comments, numbers
    for line in path.read_text(encoding="utf-8-sig").splitlines():
        content, separator, comment = line.partition("#")
        if separator:
            comments.append("#" + comment)
        for token in re.split(r"[\s,，]+", content.strip()):
            if not token:
                continue
            match = re.fullmatch(r"(?:episode)?(\d+)(?:-(?:episode)?(\d+))?", token)
            if not match:
                raise ValueError(f"{path.name} 中的编号格式无效：{token}")
            start = int(match[1])
            end = int(match[2]) if match[2] is not None else start
            if end < start:
                raise ValueError(f"{path.name} 中的编号范围无效：{token}")
            numbers.update(range(start, end + 1))
    return comments, numbers


def save_quality(directory: Path, episode: str, quality: str) -> Path:
    """Stage all files before replacing any; repeated saves are idempotent.

    A filesystem failure is propagated so callers retain the pending rating and
    can retry. A retry also repairs a partially completed move between lists.
    """
    if quality not in QUALITY_LABELS:
        raise ValueError(f"未知数据质量：{quality}")
    match = re.fullmatch(r".*?(\d+)", episode)
    if not match or "/" in episode or "\\" in episode:
        raise ValueError(f"无法提取数据编号：{episode}")
    number = int(match[1])
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / ".quality.lock").open("a", encoding="utf-8") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        contents = {}
        for label in QUALITY_LABELS:
            path = directory / f"{label}.txt"
            comments, numbers = _read_numbers(path)
            numbers.discard(number)
            if label == quality:
                numbers.add(number)
            contents[path] = "".join(
                line + "\n" for line in comments + [str(n) for n in sorted(numbers)]
            )
        staged = []
        try:
            for path, content in contents.items():
                with tempfile.NamedTemporaryFile(
                    mode="w", encoding="utf-8", dir=directory,
                    prefix=".quality-", delete=False,
                ) as stream:
                    temporary = Path(stream.name)
                    staged.append((temporary, path))
                    stream.write(content)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.chmod(temporary, path.stat().st_mode & 0o777 if path.exists() else 0o644)
            for temporary, path in staged:
                os.replace(temporary, path)
        finally:
            for temporary, _ in staged:
                temporary.unlink(missing_ok=True)
    return directory / f"{quality}.txt"
