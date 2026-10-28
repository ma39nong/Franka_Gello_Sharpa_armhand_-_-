import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np


_HELPER = r"""
import sys
import numpy as np
import pyarrow.dataset as ds

root, episode, output = sys.argv[1], int(sys.argv[2]), sys.argv[3]
dataset = ds.dataset(root + "/data", format="parquet")
available = set(dataset.schema.names)
required = {"episode_index", "timestamp", "action"}
missing = required - available
if missing:
    sys.exit("Dataset is missing columns: " + ", ".join(sorted(missing)))
columns = ["episode_index", "timestamp", "action"]
active_key = "observation.active_sides"
if active_key in available:
    columns.append(active_key)
table = dataset.to_table(
    columns=columns,
    filter=ds.field("episode_index") == episode,
).sort_by([("timestamp", "ascending")])
if table.num_rows == 0:
    sys.exit(f"Episode {episode} does not exist.")
action = np.asarray(table["action"].to_pylist(), dtype=np.float32)
active = (
    np.asarray(table[active_key].to_pylist(), dtype=np.float32) > 0.5
    if active_key in columns
    else np.ones((table.num_rows, 2), dtype=np.bool_)
)
np.savez(
    output,
    timestamp=np.asarray(table["timestamp"].to_pylist(), dtype=np.float64),
    action=action,
    active=active,
)
"""


def load_lerobot_episode(root, episode_index):
    root = Path(root).expanduser().resolve()
    info_path = root / "meta" / "info.json"
    if not info_path.is_file():
        raise ValueError(f"Not a LeRobot dataset: {info_path}")
    json.loads(info_path.read_text(encoding="utf-8"))
    python = _find_pyarrow_python()
    with tempfile.TemporaryDirectory() as temporary:
        script = Path(temporary) / "extract.py"
        output = Path(temporary) / "episode.npz"
        script.write_text(_HELPER, encoding="utf-8")
        result = subprocess.run(
            [python, str(script), str(root), str(episode_index), str(output)],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            raise RuntimeError(
                (result.stderr or result.stdout).strip()
                or f"Failed to load LeRobot episode {episode_index}."
            )
        with np.load(output, allow_pickle=False) as data:
            return (
                np.asarray(data["timestamp"], dtype=float),
                np.asarray(data["action"], dtype=float),
                np.asarray(data["active"], dtype=np.bool_),
            )


def _find_pyarrow_python():
    candidates = [
        os.environ.get("LEROBOT_DATA_PYTHON"),
        shutil.which("lerobot-python"),
        "/opt/lerobot_venv/bin/python",
    ]
    for python in candidates:
        if not python or not Path(python).exists():
            continue
        probe = subprocess.run(
            [python, "-c", "import pyarrow"],
            capture_output=True,
            text=True,
        )
        if probe.returncode == 0:
            return python
    raise RuntimeError(
        "No pyarrow-capable Python found. Rebuild the Docker image or set "
        "LEROBOT_DATA_PYTHON."
    )
