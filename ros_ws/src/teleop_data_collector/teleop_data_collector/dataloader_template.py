#!/usr/bin/env python3
"""Standalone PyTorch dataloader for this LeRobot-style dataset.

Self-contained: copy this file wherever the dataset goes. It has no project
imports and only needs ``torch``, ``numpy``, ``pyarrow``, and (for RGB video)
``opencv-python``. ``root`` defaults to this file's own directory.

Layout it reads, as written by ``data_collector.rosbag_to_parquet``:

.. code-block:: text

    <root>/
    ├── dataloader.py                 <- this file
    ├── data/chunk-000/episode_000000.parquet
    ├── videos/chunk-000/observation.images.cam0_rgb/episode_000000.mp4
    └── meta/{info,episodes,tasks,episodes_stats}.{json,jsonl}

Vector features (``observation.state``, ``action``) live in the parquet rows.
RGB streams live in MP4 files that the parquet references by path and video
timestamp. Depth and tactile streams are raw image bytes stored inline in the
parquet.

Usage::

    from dataloader import LeRobotEpisodeDataset, make_dataloader

    dataset = LeRobotEpisodeDataset(video_keys="all", image_keys="all", action_horizon=16)
    loader = make_dataloader(dataset, batch_size=32, num_workers=4)

Or inspect it from the command line::

    python dataloader.py --media
"""

from __future__ import annotations

import json
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset

__all__ = [
    "DatasetInfo",
    "EpisodeIndex",
    "LeRobotEpisodeDataset",
    "decode_raw_image",
    "make_dataloader",
]

DATASET_ROOT = Path(__file__).resolve().parent
VECTOR_DTYPES = frozenset({"float32", "float64", "int32", "int64"})


@dataclass(frozen=True, slots=True)
class DatasetInfo:
    """Parsed ``meta/info.json``."""

    root: Path
    fps: int
    robot_type: str
    total_episodes: int
    total_frames: int
    features: Mapping[str, Mapping[str, Any]]

    @classmethod
    def load(cls, root: Path) -> DatasetInfo:
        info_path = root / "meta" / "info.json"
        if not info_path.is_file():
            raise FileNotFoundError(f"Not a LeRobot dataset root, missing {info_path}")
        raw = json.loads(info_path.read_text())
        return cls(
            root=root,
            fps=int(raw["fps"]),
            robot_type=str(raw.get("robot_type", "unknown")),
            total_episodes=int(raw.get("total_episodes", 0)),
            total_frames=int(raw.get("total_frames", 0)),
            features=raw.get("features", {}),
        )

    def keys_with_dtype(self, *dtypes: str) -> list[str]:
        return [key for key, spec in self.features.items() if str(spec.get("dtype")) in dtypes]

    def vector_keys(self) -> list[str]:
        """Feature columns stored as flat float/int vectors."""
        return [
            key
            for key, spec in self.features.items()
            if str(spec.get("dtype")) in VECTOR_DTYPES
            and len(spec.get("shape", [])) == 1
            and int(spec["shape"][0]) > 1
        ]

    def video_keys(self) -> list[str]:
        return self.keys_with_dtype("video")

    def image_keys(self) -> list[str]:
        """Columns stored as raw image bytes in parquet (depth and tactile)."""
        return self.keys_with_dtype("image")

    def names(self, key: str) -> list[str] | None:
        names = self.features.get(key, {}).get("names")
        if isinstance(names, list) and all(isinstance(name, str) for name in names):
            return list(names)
        return None

    def video_fps(self, key: str) -> float:
        info = self.features.get(key, {}).get("info") or {}
        return float(info.get("video.fps") or self.fps)


@dataclass(frozen=True, slots=True)
class EpisodeIndex:
    """Where one episode's rows live."""

    episode_index: int
    length: int
    parquet_path: Path
    tasks: tuple[str, ...]


def decode_raw_image(cell: Mapping[str, Any]) -> np.ndarray:
    """Decode one inline depth/tactile image cell into ``(H, W)`` or ``(H, W, C)``.

    ``cell`` is a row of an ``observation.depths.*`` / ``observation.tactile.*``
    column: raw ROS ``sensor_msgs/msg/Image`` payload bytes plus the geometry
    needed to interpret them.
    """
    encoding = str(cell["encoding"])
    height = int(cell["height"])
    width = int(cell["width"])
    step = int(cell["step"])
    big_endian = bool(cell.get("is_bigendian", False))

    dtype, channels = _encoding_layout(encoding)
    if big_endian and dtype.itemsize > 1:
        dtype = dtype.newbyteorder(">")

    buffer = np.frombuffer(cell["data"], dtype=np.uint8)
    expected = step * height
    if buffer.size < expected:
        raise ValueError(
            f"Truncated image payload for encoding {encoding!r}: "
            f"{buffer.size} bytes, expected {expected}"
        )
    # Rows can be padded to `step`; slice each row down to its real width.
    rows = buffer[:expected].reshape(height, step)
    row_bytes = width * channels * dtype.itemsize
    image = rows[:, :row_bytes].copy().view(dtype).reshape(height, width, channels)
    return image[:, :, 0] if channels == 1 else image


def _encoding_layout(encoding: str) -> tuple[np.dtype, int]:
    layouts: dict[str, tuple[str, int]] = {
        "mono8": ("uint8", 1),
        "8UC1": ("uint8", 1),
        "mono16": ("uint16", 1),
        "16UC1": ("uint16", 1),
        "rgb8": ("uint8", 3),
        "bgr8": ("uint8", 3),
        "8UC3": ("uint8", 3),
        "rgba8": ("uint8", 4),
        "bgra8": ("uint8", 4),
        "32FC1": ("float32", 1),
    }
    if encoding not in layouts:
        raise ValueError(f"Unsupported image encoding: {encoding!r}")
    name, channels = layouts[encoding]
    return np.dtype(name), channels


class _VideoFrameReader:
    """Lazily-opened ``cv2.VideoCapture`` pool, keyed by file path.

    Held per dataset instance. With ``num_workers > 0`` each worker forks its
    own copy, so captures are never shared across processes.
    """

    def __init__(self) -> None:
        self._captures: dict[Path, Any] = {}

    def read(self, path: Path, frame_index: int) -> np.ndarray:
        import cv2

        capture = self._captures.get(path)
        if capture is None:
            capture = cv2.VideoCapture(str(path))
            if not capture.isOpened():
                raise RuntimeError(f"Could not open video: {path}")
            self._captures[path] = capture

        current = int(capture.get(cv2.CAP_PROP_POS_FRAMES))
        if current != frame_index:
            capture.set(cv2.CAP_PROP_POS_FRAMES, float(frame_index))
        ok, frame_bgr = capture.read()
        if not ok:
            raise RuntimeError(f"Could not read frame {frame_index} from {path}")
        return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

    def close(self) -> None:
        for capture in self._captures.values():
            capture.release()
        self._captures.clear()

    def __del__(self) -> None:  # best-effort cleanup
        try:
            self.close()
        except Exception:
            pass


class LeRobotEpisodeDataset(Dataset):
    """Frame-indexed view over a converted LeRobot-style dataset.

    Each item is a ``dict[str, torch.Tensor]`` holding the selected vector
    features plus, when requested, decoded RGB frames (``uint8`` ``CHW``) and
    decoded depth/tactile frames (``(H, W)``).

    Args:
        root: Dataset root containing ``meta/info.json``. Defaults to the
            directory this file lives in.
        vector_keys: Vector columns to return. Defaults to the policy features
            (``observation.state`` and ``action``). The audit-only
            ``observation.engaged`` feature must be requested explicitly.
        video_keys: RGB video columns to decode. Defaults to none, because MP4
            seeking dominates the per-item cost; pass ``"all"`` for every one.
        image_keys: Inline depth/tactile columns to decode. Defaults to none;
            pass ``"all"`` for every one.
        action_horizon: When set, ``action`` is returned as an
            ``(action_horizon, action_dim)`` chunk starting at the current
            frame, clamped at the episode end by repeating the last action.
        episodes: Restrict to these episode indices.
        transform: Optional callable applied to each assembled item.
    """

    def __init__(
        self,
        root: str | Path = DATASET_ROOT,
        *,
        vector_keys: Sequence[str] | None = None,
        video_keys: Sequence[str] | str | None = None,
        image_keys: Sequence[str] | str | None = None,
        action_horizon: int | None = None,
        episodes: Iterable[int] | None = None,
        transform: Callable[[dict[str, Any]], dict[str, Any]] | None = None,
    ) -> None:
        self.root = Path(root).expanduser().resolve()
        self.info = DatasetInfo.load(self.root)
        self.transform = transform

        if action_horizon is not None and action_horizon < 1:
            raise ValueError("action_horizon must be >= 1")
        self.action_horizon = action_horizon

        self.vector_keys = (
            list(vector_keys)
            if vector_keys is not None
            else [
                key
                for key in self.info.vector_keys()
                if key != "observation.engaged"
            ]
        )
        self.video_keys = _resolve_keys(video_keys, self.info.video_keys(), "video_keys")
        self.image_keys = _resolve_keys(image_keys, self.info.image_keys(), "image_keys")
        _check_known(self.vector_keys, self.info.features, "vector_keys")

        self.episodes = self._load_episodes(episodes)
        if not self.episodes:
            raise ValueError(f"No episodes selected in {self.root}")

        # (episode position, row within episode) for every global frame index.
        self._frame_map: list[tuple[int, int]] = [
            (position, row)
            for position, episode in enumerate(self.episodes)
            for row in range(episode.length)
        ]
        self._tables: dict[int, Any] = {}
        self._videos = _VideoFrameReader()

    # -- construction helpers -------------------------------------------------

    def _load_episodes(self, selected: Iterable[int] | None) -> list[EpisodeIndex]:
        wanted = set(selected) if selected is not None else None
        episodes: list[EpisodeIndex] = []
        for row in _read_jsonl(self.root / "meta" / "episodes.jsonl"):
            index = int(row["episode_index"])
            if wanted is not None and index not in wanted:
                continue
            relative = row.get("data_path")
            path = self.root / str(relative) if relative else self._find_episode_parquet(index)
            tasks = row.get("tasks") or []
            episodes.append(
                EpisodeIndex(
                    episode_index=index,
                    length=int(row["length"]),
                    parquet_path=path,
                    tasks=tuple(str(task) for task in tasks),
                )
            )
        if wanted is not None:
            missing = wanted - {episode.episode_index for episode in episodes}
            if missing:
                raise ValueError(f"Episodes not in dataset: {sorted(missing)}")
        return episodes

    def _find_episode_parquet(self, episode_index: int) -> Path:
        """Locate an episode's parquet when episodes.jsonl carries no path.

        The converter's chunk size is not recorded in the metadata, so try the
        default layout first and fall back to a glob across all chunks.
        """
        default = self.root / _default_data_path(episode_index)
        if default.is_file():
            return default
        name = f"episode_{episode_index:06d}.parquet"
        matches = sorted((self.root / "data").glob(f"chunk-*/{name}"))
        if not matches:
            raise FileNotFoundError(
                f"No parquet found for episode {episode_index} under {self.root}"
            )
        return matches[0]

    def _table(self, position: int) -> Any:
        table = self._tables.get(position)
        if table is None:
            import pyarrow.parquet as pq

            table = pq.read_table(self.episodes[position].parquet_path)
            self._tables[position] = table
        return table

    # -- Dataset protocol -----------------------------------------------------

    def __len__(self) -> int:
        return len(self._frame_map)

    def __getitem__(self, index: int) -> dict[str, Any]:
        position, row_index = self._frame_map[index]
        episode = self.episodes[position]
        table = self._table(position)

        item: dict[str, Any] = {
            "episode_index": torch.tensor(episode.episode_index, dtype=torch.int64),
            "frame_index": torch.tensor(row_index, dtype=torch.int64),
            "timestamp": torch.tensor(
                float(table["timestamp"][row_index].as_py()), dtype=torch.float32
            ),
            "task": episode.tasks[0] if episode.tasks else "",
        }

        for key in self.vector_keys:
            values = table[key][row_index].as_py()
            if key == "action" and self.action_horizon is not None:
                item[key] = self._action_chunk(table, row_index, episode.length)
            else:
                item[key] = torch.tensor(values, dtype=torch.float32)

        for key in self.video_keys:
            cell = table[key][row_index].as_py()
            if cell is None:
                raise ValueError(f"Missing video reference for {key} at frame {index}")
            frame_index = int(round(float(cell["timestamp"]) * self.info.video_fps(key)))
            frame = self._videos.read(self.root / str(cell["path"]), frame_index)
            item[key] = torch.from_numpy(np.ascontiguousarray(frame.transpose(2, 0, 1)))

        for key in self.image_keys:
            cell = table[key][row_index].as_py()
            if cell is None:
                raise ValueError(f"Missing image payload for {key} at frame {index}")
            item[key] = torch.from_numpy(decode_raw_image(cell))

        return self.transform(item) if self.transform is not None else item

    def _action_chunk(self, table: Any, row_index: int, length: int) -> torch.Tensor:
        assert self.action_horizon is not None
        last = length - 1
        rows = [
            table["action"][min(row_index + offset, last)].as_py()
            for offset in range(self.action_horizon)
        ]
        return torch.tensor(rows, dtype=torch.float32)

    # -- convenience ----------------------------------------------------------

    @property
    def fps(self) -> int:
        return self.info.fps

    def feature_names(self, key: str) -> list[str] | None:
        """Per-dimension labels for a vector feature, when the converter kept them."""
        return self.info.names(key)

    def close(self) -> None:
        self._videos.close()
        self._tables.clear()

    def __repr__(self) -> str:
        return (
            f"{type(self).__name__}(root={str(self.root)!r}, episodes={len(self.episodes)}, "
            f"frames={len(self)}, fps={self.info.fps})"
        )


def make_dataloader(
    dataset: LeRobotEpisodeDataset,
    *,
    batch_size: int = 32,
    shuffle: bool = True,
    num_workers: int = 0,
    **kwargs: Any,
) -> DataLoader:
    """Build a ``DataLoader`` with settings that suit this dataset.

    The default collate handles the tensor entries; ``task`` stays a list of
    strings. ``pin_memory`` and ``persistent_workers`` default to whatever makes
    sense for ``num_workers``.
    """
    kwargs.setdefault("pin_memory", torch.cuda.is_available())
    if num_workers > 0:
        kwargs.setdefault("persistent_workers", True)
        kwargs.setdefault("prefetch_factor", 2)
    return DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        num_workers=num_workers,
        drop_last=False,
        **kwargs,
    )


def _resolve_keys(
    requested: Sequence[str] | str | None,
    available: list[str],
    label: str,
) -> list[str]:
    if requested is None:
        return []
    if requested == "all":
        return list(available)
    if isinstance(requested, str):
        raise TypeError(f"{label} must be a sequence of keys or the string 'all'")
    keys = list(requested)
    unknown = sorted(set(keys) - set(available))
    if unknown:
        raise ValueError(f"Unknown {label}: {unknown}. Available: {available}")
    return keys


def _check_known(keys: Sequence[str], features: Mapping[str, Any], label: str) -> None:
    unknown = sorted(set(keys) - set(features))
    if unknown:
        raise ValueError(f"Unknown {label}: {unknown}. Available: {sorted(features)}")


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        raise FileNotFoundError(f"Missing dataset metadata: {path}")
    rows = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if line:
            rows.append(json.loads(line))
    return rows


def _default_data_path(episode_index: int, chunk_size: int = 1000) -> str:
    chunk = episode_index // chunk_size
    return f"data/chunk-{chunk:03d}/episode_{episode_index:06d}.parquet"


def _main(argv: list[str] | None = None) -> None:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "root",
        type=Path,
        nargs="?",
        default=DATASET_ROOT,
        help="Dataset root containing meta/info.json. Defaults to this file's directory.",
    )
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--action-horizon", type=int)
    parser.add_argument(
        "--media",
        action="store_true",
        help="Also decode RGB video and inline depth/tactile frames.",
    )
    args = parser.parse_args(argv)

    dataset = LeRobotEpisodeDataset(
        args.root,
        video_keys="all" if args.media else None,
        image_keys="all" if args.media else None,
        action_horizon=args.action_horizon,
    )
    print(dataset)
    for key in dataset.vector_keys:
        names = dataset.feature_names(key)
        print(f"  {key}: dim={dataset.info.features[key]['shape'][0]} named={names is not None}")

    loader = make_dataloader(
        dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
    )
    batch = next(iter(loader))
    print(f"first batch ({args.batch_size}):")
    for key, value in batch.items():
        shape = tuple(value.shape) if isinstance(value, torch.Tensor) else type(value).__name__
        dtype = value.dtype if isinstance(value, torch.Tensor) else ""
        print(f"  {key:45s} {str(shape):22s} {dtype}")
    dataset.close()


if __name__ == "__main__":
    _main()
