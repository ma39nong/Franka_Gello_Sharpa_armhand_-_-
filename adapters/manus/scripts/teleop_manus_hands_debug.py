"""Teleop the MANUS hands and capture everything a post-mortem needs, in one run.

    python teleop_manus_hands_debug.py --out-dir /home/descfly/franka_teleop_data \
        --sides both

Every argument other than `--out-dir`, `--left-state-topic`, `--right-state-topic`
and `--tag` is passed through to `teleop_manus_hands.py` unchanged, so this is a
drop-in wrapper: same keyboard UI, same flags, same behaviour.

WHY THIS EXISTS. Diagnosing a following failure needs three streams together:
what the glove reported, what the solver commanded, and what the hand actually
did. The first two are already one row per tick in the teleop debug log. The
third only exists as a ROS topic published by the hand driver, inside the
container -- and capturing it separately, in another terminal, leaves two files
with no guaranteed relationship. That is exactly the gap that made an earlier
investigation guess instead of measure.

HOW IT LINES UP. The teleop debug log stamps every row with `time.time_ns()`.
The recorder stamps every JointState with `time.time_ns()` too, from the same
host clock, and both run as children of this process for the same interval.
Merging is then a nearest-timestamp join with a stated tolerance, and the join
reports its own coverage rather than assuming it worked.

WHY TWO INTERPRETERS. The teleop stack runs in the `gello-upper-body-teleop` conda
env, which has casadi and pinocchio but no rclpy; rclpy lives in the system ROS
install. Rather than force one environment to hold both, the recorder is a
child process under `/opt/ros/*/setup.bash` and the two meet at the wall clock.

The merged file is the deliverable: one row per teleop tick, with `measured`
attached where the hand reported within tolerance. The unmerged inputs are kept
beside it, because a merge that silently dropped rows is worse than no merge.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

SCRIPTS = Path(__file__).resolve().parent
TELEOP = SCRIPTS / "teleop_manus_hands.py"
RECORDER = SCRIPTS / "_record_joint_state.py"

# How far a measured sample may sit from a teleop tick and still be called its
# measurement. The driver publishes at 30 Hz and teleop runs at 30 Hz, so half a
# period is the natural bound: beyond it the pairing is with a neighbouring
# tick, which would smear exactly the lag this file exists to measure.
MERGE_TOLERANCE_S = 1.0 / 60.0


def _ros_setup() -> str | None:
    """The newest ROS setup script on this host, or None if ROS is absent."""
    found = sorted(glob.glob("/opt/ros/*/setup.bash"))
    return found[-1] if found else None


def _ros_python(setup: str) -> str | None:
    """The system interpreter the ROS install was actually built against.

    Sourcing setup.bash is not enough. This wrapper runs under `conda run -n
    gello-upper-body-teleop`, so the child inherits a PATH whose `python3` is the
    conda 3.10; rclpy's compiled extension is built for the ROS distro's
    version (3.12 on jazzy) and the import dies with a bare
    ModuleNotFoundError for `rclpy._rclpy_pybind11`. Pin the interpreter to the
    one matching `/opt/ros/<distro>/lib/python3.X` instead of trusting PATH.
    """
    for site in sorted(Path(setup).parent.glob("lib/python3.*/site-packages")):
        candidate = Path("/usr/bin") / site.parent.name
        if candidate.exists():
            return str(candidate)
    return None


def _hand_container() -> str | None:
    """The running hand-control container, if there is one.

    The recorder belongs INSIDE it. The host carries ROS 2 jazzy and the image
    carries humble, and those two do not match endpoints: a host subscriber
    discovers nothing and records zero rows while looking perfectly healthy.
    Same distro on both ends is the only reliable arrangement, and the
    container's clock is the host's, so the merge still lines up.
    """
    try:
        result = subprocess.run(
            ["docker", "ps", "--filter", "name=hand-control",
             "--format", "{{.Names}}"],
            capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return None
    names = [name for name in result.stdout.split() if name]
    return names[0] if names else None


def _data_root() -> Path | None:
    """TELEOP_DATA_ROOT, which the compose file mounts at /data."""
    env_file = SCRIPTS.parents[2] / "docker" / ".env"
    try:
        for line in env_file.read_text().splitlines():
            key, _, value = line.partition("=")
            if key.strip() == "TELEOP_DATA_ROOT" and value.strip():
                return Path(value.strip())
    except OSError:
        pass
    return None


def _wait_for_first_row(path: Path, process: subprocess.Popen,
                        timeout: float = 10.0) -> bool:
    """Block until the recorder has actually written something.

    Subscribing is not receiving. A QoS or distro mismatch, a stopped driver,
    or a renamed topic all leave a live recorder producing an empty file, and
    that is indistinguishable from success until the run is over and wasted.
    Wait for evidence instead of for a timer.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        if process.poll() is not None:
            return False
        try:
            if path.stat().st_size > 0:
                return True
        except OSError:
            pass
        time.sleep(0.25)
    return False


def _ros_child_env() -> dict[str, str]:
    """A conda-free environment for the recorder child.

    The interpreter alone is not sufficient either: conda's PYTHONPATH,
    PYTHONHOME and LD_LIBRARY_PATH would still pull 3.10 libraries into a 3.12
    process. Drop them and let setup.bash rebuild what ROS needs.
    """
    env = {
        key: value for key, value in os.environ.items()
        if not key.startswith(("CONDA", "PYTHON", "_CE_"))
        and key not in ("LD_LIBRARY_PATH",)
    }
    env["PATH"] = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    # Must match the container's, or the subscription silently never matches.
    env.setdefault("ROS_DOMAIN_ID", os.environ.get("ROS_DOMAIN_ID", "0"))
    return env


def _existing_dir(value: str) -> Path:
    """Require the output directory to exist already, loudly.

    Deliberately does NOT create parents: a mangled paste that glues the next
    command onto this argument would otherwise materialise the garbage path and
    bury the recording somewhere nobody looks.
    """
    path = Path(value).expanduser()
    if not path.is_dir():
        raise SystemExit(
            f"--out-dir {path} does not exist. Create it deliberately, or point "
            f"at the recording directory you meant.")
    return path


def _load_jsonl(path: Path) -> tuple[dict | None, list[dict]]:
    """Return (metadata row, data rows). The teleop log's first row is metadata."""
    head, rows = None, []
    with open(path) as handle:
        for index, line in enumerate(handle):
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if index == 0 and "metadata" in record:
                head = record
                continue
            rows.append(record)
    return head, rows


def merge(teleop_path: Path, state_path: Path, out_path: Path) -> dict:
    """Nearest-timestamp join of measured state onto teleop ticks."""
    head, ticks = _load_jsonl(teleop_path)
    _, samples = _load_jsonl(state_path)

    by_side: dict[str, tuple[np.ndarray, list[dict]]] = {}
    for side in ("left", "right"):
        chosen = [s for s in samples if s.get("side") == side]
        if chosen:
            chosen.sort(key=lambda s: s["wall_time_ns"])
            by_side[side] = (
                np.array([s["wall_time_ns"] for s in chosen], dtype=np.int64),
                chosen)

    matched = {side: 0 for side in by_side}
    total = {side: 0 for side in ("left", "right")}
    with open(out_path, "w") as handle:
        if head is not None:
            head.setdefault("metadata", {})["measured_state"] = {
                "merge_tolerance_s": MERGE_TOLERANCE_S,
                "samples": {s: len(v[1]) for s, v in by_side.items()},
            }
            handle.write(json.dumps(head) + "\n")
        for tick in ticks:
            side = tick.get("side")
            if side in total:
                total[side] += 1
            entry = by_side.get(side)
            if entry is not None and "wall_time_ns" in tick:
                stamps, rows = entry
                index = int(np.searchsorted(stamps, tick["wall_time_ns"]))
                best, gap = None, None
                for candidate in (index - 1, index):
                    if 0 <= candidate < len(rows):
                        delta = abs(
                            int(stamps[candidate]) - int(tick["wall_time_ns"])) / 1e9
                        if gap is None or delta < gap:
                            best, gap = candidate, delta
                if best is not None and gap is not None and gap <= MERGE_TOLERANCE_S:
                    tick["measured"] = {
                        "name": rows[best]["name"],
                        "position": rows[best]["position"],
                        "age_s": round(gap, 6),
                    }
                    matched[side] += 1
            handle.write(json.dumps(tick) + "\n")
    return {"ticks": total, "matched": matched}


def summarise(merged_path: Path) -> None:
    """Command versus measurement, per joint. The whole point of the capture."""
    _, rows = _load_jsonl(merged_path)
    for side in ("left", "right"):
        chosen = [r for r in rows
                  if r.get("side") == side and r.get("measured") and r.get("qpos")]
        if not chosen:
            continue
        names = chosen[0]["joint_names"]
        measured_names = chosen[0]["measured"]["name"]
        try:
            order = [measured_names.index(n) for n in names]
        except ValueError:
            print(f"  {side}: joint names differ between command and "
                  f"measurement; not comparing")
            continue
        command = np.array([r["qpos"] for r in chosen], dtype=float)
        actual = np.array([[r["measured"]["position"][i] for i in order]
                           for r in chosen], dtype=float)
        error = np.degrees(actual - command)
        print(f"\n  {side}: {len(chosen)} ticks with a measurement")
        print(f"    {'joint':<22}{'cmd mean':>10}{'meas mean':>11}"
              f"{'mean err':>10}{'p95 |err|':>11}")
        for index, name in enumerate(names):
            print(f"    {name:<22}{np.degrees(command[:, index]).mean():>10.1f}"
                  f"{np.degrees(actual[:, index]).mean():>11.1f}"
                  f"{error[:, index].mean():>10.2f}"
                  f"{np.percentile(np.abs(error[:, index]), 95):>11.2f}")
        worst = int(np.argmax(np.abs(error).mean(axis=0)))
        print(f"    worst-tracked joint: {names[worst]} "
              f"({np.abs(error[:, worst]).mean():.1f} deg mean |error|)")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out-dir", required=True, type=_existing_dir,
                        help="directory for the capture; must already exist")
    parser.add_argument("--tag", default="",
                        help="appended to the timestamped file names")
    parser.add_argument("--right-state-topic", default="/cb_right_hand_state",
                        help="measured JointState for the right hand; empty disables")
    parser.add_argument("--left-state-topic", default="",
                        help="measured JointState for the left hand; empty disables")
    parser.add_argument("--no-measured", action="store_true",
                        help="record teleop only; the capture then cannot show "
                             "command versus measurement")
    args, passthrough = parser.parse_known_args()

    if "--debug-log" in passthrough:
        raise SystemExit(
            "--debug-log is managed by this wrapper; drop it and use --out-dir")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    suffix = f"_{args.tag}" if args.tag else ""
    teleop_path = args.out_dir / f"capture_{stamp}{suffix}.teleop.jsonl"
    state_path = args.out_dir / f"capture_{stamp}{suffix}.measured.jsonl"
    merged_path = args.out_dir / f"capture_{stamp}{suffix}.merged.jsonl"

    # A capture without the measured stream cannot answer the question this
    # script exists for -- whether the hand reached what it was told -- so a
    # recorder that fails to start is fatal, not a warning. Opting out is
    # available, but it has to be deliberate.
    def _refuse(reason: str) -> None:
        raise SystemExit(
            f"\n  cannot capture measured state: {reason}\n"
            f"  Fix it, or pass --no-measured to record teleop only "
            f"(the capture then cannot show command-vs-measurement).\n")

    setup = _ros_setup()
    recorder = None
    want_state = (args.right_state_topic or args.left_state_topic) and not args.no_measured
    if args.no_measured:
        print("  --no-measured: capturing teleop only, by request")
    elif not (args.right_state_topic or args.left_state_topic):
        print("  no state topics requested; capturing teleop only")
    elif setup is None:
        _refuse("no /opt/ros/*/setup.bash on this host")
    if want_state:
        container = _hand_container()
        topics = (f" --right-topic '{args.right_state_topic}'"
                  f" --left-topic '{args.left_state_topic}'")
        if container is not None:
            # Inside the container: same ROS distro as the publisher, and its
            # clock is the host's, so the merge is unaffected. The repo is
            # mounted at /workspace and TELEOP_DATA_ROOT at /data.
            root = _data_root()
            if root is None or root not in state_path.parents:
                _refuse(
                    f"the recorder must write under TELEOP_DATA_ROOT "
                    f"({root}) so the container can see it, but --out-dir is "
                    f"{args.out_dir}")
            inner_out = Path("/data") / state_path.relative_to(root)
            inner_recorder = (Path("/workspace/franka_upper_body_teleop")
                              / RECORDER.relative_to(SCRIPTS.parents[2]))
            command = (f"source /opt/ros/*/setup.bash >/dev/null 2>&1 && "
                       f"exec python3 -u {inner_recorder} --out {inner_out}{topics}")
            recorder = subprocess.Popen(
                ["docker", "exec", container, "bash", "-lc", command],
                stderr=subprocess.PIPE, text=True)
            where = f"container {container}"
        else:
            interpreter = _ros_python(setup)
            if interpreter is None:
                _refuse(f"no system interpreter matching "
                        f"{Path(setup).parent}/lib/python3.*")
            command = (
                f"source {setup} >/dev/null 2>&1 && exec {interpreter} -u "
                f"{RECORDER} --out {state_path}{topics}")
            # `bash -c`, not `-lc`: a login shell re-runs the profile that puts
            # conda back on PATH.
            recorder = subprocess.Popen(
                ["bash", "-c", command], env=_ros_child_env(),
                stderr=subprocess.PIPE, text=True)
            where = f"host {interpreter} (ROS {Path(setup).parent.name})"
            print("  WARNING no hand-control container found; recording from "
                  "the host. If its ROS distro differs from the driver's, "
                  "nothing will match.")

        # Subscribing is not receiving: wait for a row to actually land.
        if not _wait_for_first_row(state_path, recorder):
            stderr = recorder.stderr.read() if recorder.stderr else ""
            if recorder.poll() is None:
                recorder.send_signal(signal.SIGINT)
                recorder.wait(timeout=10)
            _refuse(
                f"the recorder started ({where}) but received nothing in 10 s.\n"
                f"  Check: is the hand driver running and publishing "
                f"{args.right_state_topic}, and does its ROS distro match the "
                f"recorder's?\n"
                + (f"  recorder said: {stderr.strip()}\n" if stderr.strip() else ""))
        print(f"  recorder running in {where}, receiving")

    print(f"\n  teleop  -> {teleop_path}")
    print(f"  measured-> {state_path if recorder else '(none)'}")
    print(f"  merged  -> {merged_path}\n")

    teleop_command = [sys.executable, str(TELEOP),
                      "--debug-log", str(teleop_path), *passthrough]
    status = 0
    try:
        # stdin is inherited: the keyboard activation UI needs the tty.
        status = subprocess.call(teleop_command)
    except KeyboardInterrupt:
        status = 130
    finally:
        if recorder is not None and recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:
                recorder.wait(timeout=10)
            except subprocess.TimeoutExpired:
                recorder.kill()
                recorder.wait()

    if not teleop_path.exists():
        print("  no teleop log was written; nothing to merge")
        return status or 1
    if not state_path.exists():
        print("  no measured state was captured; the teleop log stands alone")
        shutil.copyfile(teleop_path, merged_path)
        return status

    report = merge(teleop_path, state_path, merged_path)
    print("\n  merge coverage (ticks with a measurement within "
          f"{MERGE_TOLERANCE_S * 1000:.0f} ms):")
    for side, count in report["ticks"].items():
        if count:
            got = report["matched"].get(side, 0)
            print(f"    {side}: {got}/{count}  ({got / count * 100:.0f}%)")
            if got == 0:
                print(f"      -> nothing matched. Was the hand driver running, "
                      f"and is ROS_DOMAIN_ID the same as the container's?")
    summarise(merged_path)
    print(f"\n  merged capture: {merged_path}")
    return status


if __name__ == "__main__":
    sys.exit(main())
