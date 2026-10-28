"""MuJoCo-only Wuji Hand 2 simulation.

This entry point never connects to a Wuji hand or publishes a hardware command.
It accepts the bundled replay by default and can optionally read one live MANUS
glove through the existing native bridge. Live landmarks can be recorded for
later replay without a glove.
"""

from __future__ import annotations

import argparse
import pickle
import time
from pathlib import Path
from typing import Iterator

import mujoco
import numpy as np
import yaml

from adapters.wuji.wuji_retargeting import Retargeter


ROOT = Path(__file__).resolve().parent
CONFIG_DIR = ROOT / "config"
DEFAULT_REPLAYS = {
    "left": Path(
        "/home/descfly/franka_teleop_data/wuji_replays/l_pinch_2.pkl"
    ),
    "right": Path(
        "/home/descfly/franka_teleop_data/wuji_replays/r_pinch_2.pkl"
    ),
}


def config_path(side: str) -> Path:
    return CONFIG_DIR / f"retarget_manus_wuji_hand_2_{side}.yaml"


def selected_replay_path(side: str, override: Path | None) -> Path:
    return override if override is not None else DEFAULT_REPLAYS[side]


def model_path(config: Path) -> Path:
    content = yaml.safe_load(config.read_text()) or {}
    relative = (content.get("optimizer") or {}).get("mjcf_path")
    if not relative:
        raise ValueError(f"optimizer.mjcf_path is missing from {config}")
    resolved = (config.parent / relative).resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"MuJoCo model is missing: {resolved}")
    return resolved


def actuator_permutation(
    retargeter: Retargeter, model: mujoco.MjModel
) -> np.ndarray:
    source_names = list(retargeter.optimizer.robot.dof_joint_names)
    destination_names = [
        mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            int(model.actuator_trnid[index, 0]),
        )
        for index in range(model.nu)
    ]
    source_index = {name: index for index, name in enumerate(source_names)}
    if None in destination_names or set(destination_names) != set(source_names):
        raise ValueError("URDF and MJCF joint names do not describe the same hand")
    return np.asarray(
        [source_index[name] for name in destination_names], dtype=int
    )


def replay_frames(path: Path, side: str) -> Iterator[np.ndarray]:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"replay is missing: {path}")
    with path.open("rb") as stream:
        rows = pickle.load(stream)  # trusted, repository-bundled fixture
    key = f"{side}_fingers"
    valid = []
    for row in rows:
        value = row.get(key)
        if value is None:
            continue
        frame = np.asarray(value, dtype=np.float64)
        if frame.shape == (21, 3) and np.isfinite(frame).all():
            valid.append(frame)
    if not valid:
        raise ValueError(f"replay contains no valid {side} hand frames")
    while True:
        yield from valid


def manus_frames(side: str) -> Iterator[np.ndarray]:
    from manus_teleop.pipeline import ManusBridge, canonical_landmarks

    library = ROOT.parent / "manus" / "build" / "libmanus_skeleton_bridge.so"
    calibrations = ROOT.parent / "manus" / "config"
    if not library.is_file():
        raise FileNotFoundError(
            f"MANUS bridge is missing: {library}; run adapters/manus/scripts/build.sh"
        )
    bridge = ManusBridge(library)
    bridge.connect(calibrations)
    try:
        while True:
            frame = bridge.read(side, 0.1)
            if frame is not None:
                yield canonical_landmarks(frame)
    finally:
        bridge.close()


class ReplayRecorder:
    """Collect raw canonical MANUS landmarks in the bundled replay format."""

    def __init__(self, path: Path, side: str) -> None:
        self.path = path.expanduser().resolve()
        if self.path.exists():
            raise FileExistsError(
                f"refusing to overwrite existing replay: {self.path}"
            )
        self.side = side
        self.started = time.monotonic()
        self.rows = []

    def add(self, landmarks: np.ndarray) -> None:
        frame = np.asarray(landmarks, dtype=np.float64)
        if frame.shape != (21, 3) or not np.isfinite(frame).all():
            raise ValueError("recorded MANUS frame must contain 21 finite landmarks")
        self.rows.append(
            {
                "t": time.monotonic() - self.started,
                "left_fingers": frame.copy() if self.side == "left" else None,
                "right_fingers": frame.copy() if self.side == "right" else None,
            }
        )

    def save(self) -> int:
        if not self.rows:
            return 0
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Exclusive creation protects an existing recording even if it appears
        # after argument validation but before Ctrl+C triggers this save.
        with self.path.open("xb") as stream:
            pickle.dump(self.rows, stream, protocol=pickle.HIGHEST_PROTOCOL)
        return len(self.rows)


def run(args: argparse.Namespace) -> None:
    config = config_path(args.side)
    retargeter = Retargeter.from_yaml(str(config), args.side)
    mjcf_path = model_path(config)
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    permutation = actuator_permutation(retargeter, model)

    replay = selected_replay_path(args.side, args.replay)
    frames = (
        replay_frames(replay, args.side)
        if args.input == "replay"
        else manus_frames(args.side)
    )
    recorder = ReplayRecorder(args.record, args.side) if args.record else None
    maximum_frames = args.frames or (300 if args.headless else None)
    steps_per_frame = max(1, round(1.0 / (args.fps * model.opt.timestep)))

    viewer = None
    if not args.headless:
        from mujoco import viewer as mujoco_viewer

        viewer = mujoco_viewer.launch_passive(model, data)
        viewer.cam.azimuth = 180
        viewer.cam.elevation = -20
        viewer.cam.distance = 0.5
        viewer.cam.lookat[:] = [0.0, 0.0, 0.05]

    print("Wuji simulation (hardware output disabled)")
    print(f"  model: hand2_beta ({mjcf_path})")
    print(f"  input: {args.input}")
    print(f"  side: {args.side}")
    if args.input == "replay":
        print(f"  replay: {replay.expanduser().resolve()}")
    if recorder is not None:
        print(f"  recording raw MANUS landmarks: {recorder.path}")
        print("  press Ctrl+C or close the viewer to save")
    started = time.monotonic()
    processed = 0
    try:
        for landmarks in frames:
            tick_started = time.monotonic()
            if recorder is not None:
                recorder.add(landmarks)
            qpos = retargeter.retarget(landmarks)
            data.ctrl[:] = qpos[permutation]
            for _ in range(steps_per_frame):
                mujoco.mj_step(model, data)

            processed += 1
            if viewer is not None:
                if not viewer.is_running():
                    break
                viewer.sync()
                remaining = 1.0 / args.fps - (time.monotonic() - tick_started)
                if remaining > 0.0:
                    time.sleep(remaining)
            if maximum_frames is not None and processed >= maximum_frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None:
            viewer.close()
        close_frames = getattr(frames, "close", None)
        if callable(close_frames):
            close_frames()
        if recorder is not None:
            saved = recorder.save()
            if saved:
                print(f"saved {saved} raw MANUS frames to {recorder.path}")
            else:
                print("no MANUS frames received; no replay file was written")

    elapsed = max(time.monotonic() - started, 1e-9)
    print(f"processed {processed} frames at {processed / elapsed:.1f} frame/s")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument(
        "--input", choices=("replay", "manus"), default="replay"
    )
    parser.add_argument(
        "--replay",
        type=Path,
        default=None,
        help="override the side-specific l_pinch_2/r_pinch_2 replay",
    )
    parser.add_argument(
        "--record",
        type=Path,
        default=None,
        metavar="FILE",
        help="with --input manus, save raw landmarks for later replay",
    )
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="stop after N frames; 0 means viewer-controlled (300 headless)",
    )
    parser.add_argument(
        "--headless", action="store_true", help="run without opening a viewer"
    )
    args = parser.parse_args()
    if args.fps <= 0.0:
        parser.error("--fps must be positive")
    if args.frames < 0:
        parser.error("--frames cannot be negative")
    if args.record is not None and args.input != "manus":
        parser.error("--record requires --input manus")
    run(args)


if __name__ == "__main__":
    main()
