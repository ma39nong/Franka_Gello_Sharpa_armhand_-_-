"""Solve one recorded relative action through Arm UI's MoveIt IK path."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from .recording import ActionStore, RecordedAction
from .runtime import ArmRosRuntime, RelativeIkSolution


RESULT_PREFIX = "PRESET_IK_RESULT="


def _wait_until_ready(runtime: ArmRosRuntime, timeout: float = 20.0) -> None:
    deadline = time.monotonic() + timeout
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            for side in ("left", "right"):
                runtime.snapshot(side)
            if not runtime.compute_ik_client.wait_for_service(timeout_sec=0.05):
                raise RuntimeError("MoveIt /compute_ik 不可用")
            if not runtime.compute_fk_client.wait_for_service(timeout_sec=0.05):
                raise RuntimeError("MoveIt /compute_fk 不可用")
            runtime._lookup_tool_transform("left")
            runtime._lookup_tool_transform("right")
            return
        except (RuntimeError, TimeoutError) as error:
            last_error = error
            time.sleep(0.05)
    raise RuntimeError(f"robot state/FK not ready: {last_error}")


def serialize_preset_solution(
    solution: RelativeIkSolution,
    *,
    name: str,
    start_q: tuple[float, ...],
    speed_scale: float,
    max_joint_speed: float,
) -> dict[str, object]:
    times = solution.time_sec
    initial_offset = max(
        abs(value - start)
        for value, start in zip(
            solution.positions[0], solution.start_positions, strict=True
        )
    )
    if initial_offset > 0.05:
        raise RuntimeError(
            f"first IK frame is {math.degrees(initial_offset):.1f} deg "
            "from the measured start"
        )
    for index in range(1, len(times)):
        elapsed = (times[index] - times[index - 1]) / speed_scale
        speed = max(
            abs(current - previous) / elapsed
            for current, previous in zip(
                solution.positions[index],
                solution.positions[index - 1],
                strict=True,
            )
        )
        if speed > max_joint_speed:
            raise RuntimeError(
                f"trajectory exceeds {max_joint_speed:.2f} rad/s "
                f"at frame {index + 1}: {speed:.3f} rad/s"
            )
    return {
        "name": name,
        "side": solution.side,
        "time_sec": list(times),
        "positions": [list(values) for values in solution.positions],
        "start_q": list(start_q),
        "speed_scale": speed_scale,
        "checked_frames": solution.validation.checked_frames,
        "max_joint_step_rad": solution.validation.max_joint_step_rad,
    }


def build_preset_solution(
    runtime: ArmRosRuntime,
    action: RecordedAction,
    *,
    speed_scale: float,
    max_joint_speed: float,
) -> dict[str, object]:
    """Solve and sanity-check one recorded action from the current pose."""
    if not math.isfinite(speed_scale) or not 0.05 <= speed_scale <= 1.0:
        raise ValueError("speed scale must be between 5% and 100%")
    if not math.isfinite(max_joint_speed) or max_joint_speed <= 0.0:
        raise ValueError("maximum joint speed must be positive")
    start_q = tuple(
        value
        for side in ("left", "right")
        for value in runtime.snapshot(side).positions
    )
    solution = runtime.solve_relative_action(action)
    return serialize_preset_solution(
        solution,
        name=action.name,
        start_q=start_q,
        speed_scale=speed_scale,
        max_joint_speed=max_joint_speed,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", default="/data/arm_ui")
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--action", required=True)
    parser.add_argument("--speed-scale", type=float, default=0.5)
    parser.add_argument("--max-joint-speed", type=float, default=0.5)
    args = parser.parse_args()

    store = ActionStore(Path(args.data_root))
    action = store.load(args.side, args.action)
    runtime = ArmRosRuntime(robot_type="fr3", service_timeout_sec=8.0)
    try:
        _wait_until_ready(runtime)
        payload = build_preset_solution(
            runtime,
            action,
            speed_scale=args.speed_scale,
            max_joint_speed=args.max_joint_speed,
        )
        print(RESULT_PREFIX + json.dumps(payload, separators=(",", ":")))
        return 0
    finally:
        runtime.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
