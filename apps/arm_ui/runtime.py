"""ROS 2 runtime for hand guiding and MoveIt sequence execution.

The web layer never publishes hardware commands. This runtime is the only arm
UI component allowed to switch controllers or submit a MoveIt action.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from types import SimpleNamespace
from typing import Callable, Sequence

from .absolute_recording import DualArmSample
from .models import Routine, SIDES, Waypoint
from .recording import (
    MotionSample,
    RecordedAction,
    compose_pose,
    smooth_relative_frames,
)


@dataclass(frozen=True)
class JointSnapshot:
    side: str
    names: tuple[str, ...]
    positions: tuple[float, ...]
    received_at: float


@dataclass(frozen=True)
class MotionCapture:
    side: str
    base_frame: str
    tool_frame: str
    samples: tuple[MotionSample, ...]
    skipped_tf_samples: int


@dataclass(frozen=True)
class AbsoluteMotionCapture:
    joint_names: dict[str, tuple[str, ...]]
    samples: tuple[DualArmSample, ...]


@dataclass(frozen=True)
class IkValidation:
    checked_frames: int
    total_frames: int
    max_joint_step_rad: float


@dataclass(frozen=True)
class RelativeIkSolution:
    side: str
    joint_names: tuple[str, ...]
    start_positions: tuple[float, ...]
    time_sec: tuple[float, ...]
    positions: tuple[tuple[float, ...], ...]
    validation: IkValidation


class TrajectoryCancelled(RuntimeError):
    """MoveIt reports PREEMPTED after an operator STOP request."""


class ArmRosRuntime:
    """Owns ROS subscriptions, controller switches, and MoveIt sequence goals."""

    def __init__(
        self,
        *,
        robot_type: str = "fr3",
        state_timeout_sec: float = 1.0,
        service_timeout_sec: float = 5.0,
        action_timeout_sec: float = 300.0,
        status_callback: Callable[[str, str], None] | None = None,
    ) -> None:
        try:
            from action_msgs.msg import GoalStatus
            from action_msgs.srv import CancelGoal
            from builtin_interfaces.msg import Duration
            from control_msgs.action import FollowJointTrajectory
            import rclpy
            from controller_manager_msgs.srv import (
                ConfigureController,
                ListControllers,
                LoadController,
                SwitchController,
            )
            from moveit_msgs.action import MoveGroupSequence
            from moveit_msgs.msg import (
                Constraints,
                JointConstraint,
                MoveItErrorCodes,
                MotionSequenceItem,
            )
            from moveit_msgs.srv import GetPositionFK, GetPositionIK
            from geometry_msgs.msg import PoseStamped
            from rclpy.action import ActionClient
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.time import Time
            from sensor_msgs.msg import JointState
            from trajectory_msgs.msg import JointTrajectoryPoint
            import tf2_ros
        except ImportError as error:
            raise RuntimeError(
                "arm_ui必须在已source ROS 2和工作区的环境中运行"
            ) from error

        self.ros = {
            "rclpy": rclpy,
            "ActionClient": ActionClient,
            "Duration": Duration,
            "CancelGoal": CancelGoal,
            "ConfigureController": ConfigureController,
            "Constraints": Constraints,
            "JointConstraint": JointConstraint,
            "JointState": JointState,
            "ListControllers": ListControllers,
            "LoadController": LoadController,
            "MotionSequenceItem": MotionSequenceItem,
            "MoveItErrorCodes": MoveItErrorCodes,
            "MoveGroupSequence": MoveGroupSequence,
            "GetPositionFK": GetPositionFK,
            "GetPositionIK": GetPositionIK,
            "GoalStatus": GoalStatus,
            "FollowJointTrajectory": FollowJointTrajectory,
            "JointTrajectoryPoint": JointTrajectoryPoint,
            "PoseStamped": PoseStamped,
            "MultiThreadedExecutor": MultiThreadedExecutor,
            "SwitchController": SwitchController,
            "Time": Time,
            "tf2_ros": tf2_ros,
        }
        self.robot_type = str(robot_type)
        self.state_timeout_sec = float(state_timeout_sec)
        self.service_timeout_sec = float(service_timeout_sec)
        self.action_timeout_sec = float(action_timeout_sec)
        self.status_callback = status_callback
        self._snapshots: dict[str, JointSnapshot] = {}
        self._snapshot_lock = threading.RLock()
        self._goal_lock = threading.RLock()
        self._execution_lock = threading.Lock()
        self._stop_requested = threading.Event()
        self._active_goal = None
        self._running_side: str | None = None
        self._controller_mode = {side: "unknown" for side in SIDES}
        self._record_lock = threading.RLock()
        self._record_side: str | None = None
        self._record_started_at = 0.0
        self._record_last_sample_at = -math.inf
        self._record_period_sec = 0.01
        self._record_samples: list[MotionSample] = []
        self._record_skipped_tf = 0
        self._absolute_recording = False
        self._absolute_record_started_at = 0.0
        self._absolute_record_last_sample_at = -math.inf
        self._absolute_record_period_sec = 0.02
        self._absolute_record_samples: list[DualArmSample] = []
        # `world` is only an SRDF virtual-joint parent and is not published by
        # robot_state_publisher. The URDF root is present in both fake and real
        # stacks and is a stable fixed base for relative-pose recording.
        self.base_frame = "lychee_root"

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node("arm_ui")
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self.node, spin_thread=False
        )
        self._spin_thread = threading.Thread(
            target=self.executor.spin, name="arm-ui-ros", daemon=True
        )

        self._subscriptions = []
        self._manager_clients: dict[str, dict[str, object]] = {}
        for side in SIDES:
            self._subscriptions.append(
                self.node.create_subscription(
                    JointState,
                    f"/{side}/franka/joint_states",
                    lambda message, side=side: self._on_joint_state(side, message),
                    10,
                )
            )
            manager = f"/{side}/controller_manager"
            self._manager_clients[side] = {
                "load": self.node.create_client(
                    LoadController, f"{manager}/load_controller"
                ),
                "configure": self.node.create_client(
                    ConfigureController, f"{manager}/configure_controller"
                ),
                "list": self.node.create_client(
                    ListControllers, f"{manager}/list_controllers"
                ),
                "switch": self.node.create_client(
                    SwitchController, f"{manager}/switch_controller"
                ),
            }
        # Fake hardware uses one root controller manager and a combined state
        # topic; real hardware uses one namespaced manager/topic per arm.
        self._subscriptions.append(
            self.node.create_subscription(
                JointState, "/joint_states", self._on_combined_joint_state, 10
            )
        )
        self._manager_clients["combined"] = {
            "load": self.node.create_client(
                LoadController, "/controller_manager/load_controller"
            ),
            "configure": self.node.create_client(
                ConfigureController, "/controller_manager/configure_controller"
            ),
            "list": self.node.create_client(
                ListControllers, "/controller_manager/list_controllers"
            ),
            "switch": self.node.create_client(
                SwitchController, "/controller_manager/switch_controller"
            ),
        }
        self._trajectory_cancel_clients: dict[str, tuple[object, object]] = {}
        for side in SIDES:
            self._trajectory_cancel_clients[side] = (
                self.node.create_client(
                    CancelGoal,
                    f"/{side}/{side}_arm_controller/"
                    "follow_joint_trajectory/_action/cancel_goal",
                ),
                self.node.create_client(
                    CancelGoal,
                    f"/{side}_arm_controller/"
                    "follow_joint_trajectory/_action/cancel_goal",
                ),
            )

        self.sequence_client = ActionClient(
            self.node, MoveGroupSequence, "/sequence_move_group"
        )
        self._trajectory_action_clients: dict[str, tuple[object, object]] = {}
        for side in SIDES:
            self._trajectory_action_clients[side] = (
                ActionClient(
                    self.node,
                    FollowJointTrajectory,
                    f"/{side}/{side}_arm_controller/follow_joint_trajectory",
                ),
                ActionClient(
                    self.node,
                    FollowJointTrajectory,
                    f"/{side}_arm_controller/follow_joint_trajectory",
                ),
            )
        self.compute_ik_client = self.node.create_client(GetPositionIK, "/compute_ik")
        self.compute_fk_client = self.node.create_client(GetPositionFK, "/compute_fk")
        self._spin_thread.start()

    def _notify(self, side: str, message: str) -> None:
        if self.status_callback is not None:
            self.status_callback(side, message)

    def joint_names(self, side: str) -> tuple[str, ...]:
        return tuple(f"{side}_{self.robot_type}_joint{index}" for index in range(1, 8))

    def _on_joint_state(self, side: str, message: object) -> None:
        names = tuple(str(name) for name in message.name)
        positions = tuple(float(value) for value in message.position)
        index_by_name = {name: index for index, name in enumerate(names)}
        expected = self.joint_names(side)
        if any(name not in index_by_name for name in expected):
            return
        ordered = tuple(positions[index_by_name[name]] for name in expected)
        if not all(math.isfinite(value) for value in ordered):
            return
        with self._snapshot_lock:
            self._snapshots[side] = JointSnapshot(
                side, expected, ordered, time.monotonic()
            )
        self._capture_recording_sample(side, ordered)
        self._capture_absolute_recording_sample()

    def _on_combined_joint_state(self, message: object) -> None:
        for side in SIDES:
            self._on_joint_state(side, message)

    def snapshot(self, side: str, *, require_fresh: bool = True) -> JointSnapshot:
        if side not in SIDES:
            raise ValueError(f"未知机械臂: {side}")
        with self._snapshot_lock:
            snapshot = self._snapshots.get(side)
        if snapshot is None:
            raise RuntimeError(f"尚未收到{side}机械臂关节状态")
        age = time.monotonic() - snapshot.received_at
        if require_fresh and age > self.state_timeout_sec:
            raise RuntimeError(f"{side}机械臂关节状态已过期（{age:.2f}秒）")
        return snapshot

    def controller_mode(self, side: str) -> str:
        return self._controller_mode[side]

    def tool_frame(self, side: str) -> str:
        if side not in SIDES:
            raise ValueError(f"未知机械臂: {side}")
        return f"{side}_{self.robot_type}_link8"

    def _combined_joint_state(self) -> tuple[list[str], list[float]]:
        names: list[str] = []
        positions: list[float] = []
        with self._snapshot_lock:
            snapshots = dict(self._snapshots)
        for side in SIDES:
            snapshot = snapshots.get(side)
            if snapshot is None:
                continue
            names.extend(snapshot.names)
            positions.extend(snapshot.positions)
        if not names:
            raise RuntimeError("还没有机械臂关节状态")
        return names, positions

    def _lookup_tool_transform(self, side: str) -> object:
        try:
            return self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame(side),
                self.ros["Time"](),
            )
        except self.ros["tf2_ros"].TransformException:
            return self._lookup_tool_transform_via_fk(side)

    def _lookup_tool_transform_via_fk(self, side: str) -> object:
        """Use MoveIt FK when /tf is split across franka-control and moveit-ik."""
        names, positions = self._combined_joint_state()
        request = self.ros["GetPositionFK"].Request()
        request.header.frame_id = self.base_frame
        request.fk_link_names = [self.tool_frame(side)]
        request.robot_state.joint_state.name = names
        request.robot_state.joint_state.position = positions
        response = self._call(
            self.compute_fk_client, request, f"compute_fk ({side})"
        )
        if response.error_code.val != self.ros["MoveItErrorCodes"].SUCCESS:
            raise RuntimeError(
                f"无法通过FK读取{self.base_frame}到{self.tool_frame(side)}: "
                f"error_code={response.error_code.val}"
            )
        if not response.pose_stamped:
            raise RuntimeError(
                f"FK未返回{self.tool_frame(side)}位姿"
            )
        pose = response.pose_stamped[0].pose
        return SimpleNamespace(
            transform=SimpleNamespace(
                translation=pose.position,
                rotation=pose.orientation,
            )
        )

    def start_recording(self, side: str, *, rate_hz: float = 100.0) -> None:
        if side not in SIDES:
            raise ValueError(f"未知机械臂: {side}")
        if self.is_running():
            raise RuntimeError("轨迹正在执行，不能开始录制")
        if self.absolute_recording_status()["active"]:
            raise RuntimeError("双臂绝对轨迹正在录制")
        if not math.isfinite(rate_hz) or not 10.0 <= rate_hz <= 250.0:
            raise ValueError("录制频率必须在10Hz到250Hz之间")
        if self._graph_controller_mode(side) != "teach":
            raise RuntimeError("请先让所选机械臂进入零力矩拖动模式")
        _ = self.snapshot(side)
        _ = self._lookup_tool_transform(side)
        self._start_recording_unchecked(side, rate_hz=rate_hz)

    def _start_recording_unchecked(self, side: str, *, rate_hz: float) -> None:
        """Begin capture after the caller has established a safe control mode."""
        with self._record_lock:
            if self._record_side is not None:
                raise RuntimeError("已有动作正在录制")
            self._record_side = side
            self._record_started_at = time.monotonic()
            self._record_last_sample_at = -math.inf
            self._record_period_sec = 1.0 / rate_hz
            self._record_samples = []
            self._record_skipped_tf = 0
        self._notify(side, "动作录制中")

    def _capture_recording_sample(
        self, side: str, positions: tuple[float, ...]
    ) -> None:
        now = time.monotonic()
        with self._record_lock:
            if self._record_side != side:
                return
            if now - self._record_last_sample_at < self._record_period_sec:
                return
            started_at = self._record_started_at
        try:
            transform = self._lookup_tool_transform(side).transform
        except RuntimeError:
            with self._record_lock:
                self._record_skipped_tf += 1
            return
        sample = MotionSample.create(
            now - started_at,
            positions,
            (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ),
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
        )
        with self._record_lock:
            if self._record_side != side:
                return
            self._record_samples.append(sample)
            self._record_last_sample_at = now

    def recording_status(self) -> dict[str, object]:
        with self._record_lock:
            side = self._record_side
            return {
                "active": side is not None,
                "side": side,
                "elapsed_sec": (
                    time.monotonic() - self._record_started_at if side else 0.0
                ),
                "sample_count": len(self._record_samples),
                "skipped_tf_samples": self._record_skipped_tf,
            }

    def start_absolute_recording(self, *, rate_hz: float = 50.0) -> None:
        """Capture synchronized measured joints for both hand-guided arms."""
        if self.is_running():
            raise RuntimeError("轨迹正在执行，不能开始录制")
        if self.recording_status()["active"]:
            raise RuntimeError("单臂相对动作正在录制")
        if not math.isfinite(rate_hz) or not 10.0 <= rate_hz <= 100.0:
            raise ValueError("双臂绝对轨迹录制频率必须在10Hz到100Hz之间")
        modes = {side: self._graph_controller_mode(side) for side in SIDES}
        if any(mode != "teach" for mode in modes.values()):
            raise RuntimeError("请先让左右臂都进入零力矩拖动模式")
        for side in SIDES:
            _ = self.snapshot(side)
        with self._record_lock:
            if self._absolute_recording:
                raise RuntimeError("双臂绝对轨迹已经在录制")
            self._absolute_recording = True
            self._absolute_record_started_at = time.monotonic()
            self._absolute_record_last_sample_at = -math.inf
            self._absolute_record_period_sec = 1.0 / rate_hz
            self._absolute_record_samples = []
        for side in SIDES:
            self._notify(side, "双臂绝对轨迹录制中")

    def _capture_absolute_recording_sample(self) -> None:
        now = time.monotonic()
        with self._record_lock:
            if not self._absolute_recording:
                return
            if now - self._absolute_record_last_sample_at < self._absolute_record_period_sec:
                return
            started_at = self._absolute_record_started_at
        try:
            snapshots = {side: self.snapshot(side) for side in SIDES}
        except RuntimeError:
            return
        if (
            max(snapshot.received_at for snapshot in snapshots.values())
            - min(snapshot.received_at for snapshot in snapshots.values())
            > 0.05
        ):
            return
        sample = DualArmSample.create(
            now - started_at,
            {side: snapshots[side].positions for side in SIDES},
        )
        with self._record_lock:
            if not self._absolute_recording:
                return
            self._absolute_record_samples.append(sample)
            self._absolute_record_last_sample_at = now

    def absolute_recording_status(self) -> dict[str, object]:
        with self._record_lock:
            return {
                "active": self._absolute_recording,
                "elapsed_sec": (
                    time.monotonic() - self._absolute_record_started_at
                    if self._absolute_recording
                    else 0.0
                ),
                "sample_count": len(self._absolute_record_samples),
            }

    def stop_absolute_recording(self) -> AbsoluteMotionCapture:
        with self._record_lock:
            if not self._absolute_recording:
                raise RuntimeError("当前没有正在录制的双臂绝对轨迹")
            samples = tuple(self._absolute_record_samples)
            self._absolute_recording = False
            self._absolute_record_samples = []
        names = {side: self.joint_names(side) for side in SIDES}
        for side in SIDES:
            self._notify(side, "双臂绝对轨迹录制完成")
        return AbsoluteMotionCapture(names, samples)

    def discard_absolute_recording(self) -> bool:
        with self._record_lock:
            if not self._absolute_recording:
                return False
            self._absolute_recording = False
            self._absolute_record_samples = []
        for side in SIDES:
            self._notify(side, "双臂绝对轨迹录制已放弃")
        return True

    def stop_recording(self) -> MotionCapture:
        with self._record_lock:
            side = self._record_side
            if side is None:
                raise RuntimeError("当前没有正在录制的动作")
            samples = tuple(self._record_samples)
            skipped = self._record_skipped_tf
            self._record_side = None
            self._record_samples = []
        self._notify(side, "动作录制完成，机械臂仍处于拖动模式")
        return MotionCapture(
            side=side,
            base_frame=self.base_frame,
            tool_frame=self.tool_frame(side),
            samples=samples,
            skipped_tf_samples=skipped,
        )

    def validate_relative_action(
        self, action: RecordedAction, *, max_frames: int = 250
    ) -> IkValidation:
        """Check a recorded relative path from the current pose without moving."""
        return self._solve_relative_action(action, max_frames=max_frames).validation

    def solve_relative_action(self, action: RecordedAction) -> RelativeIkSolution:
        """Solve every frame from the current measured pose without moving."""
        return self._solve_relative_action(action, max_frames=None)

    def _solve_relative_action(
        self,
        action: RecordedAction,
        *,
        max_frames: int | None = None,
        allow_running: bool = False,
    ) -> RelativeIkSolution:
        if action.side not in SIDES:
            raise ValueError(f"未知机械臂: {action.side}")
        if self.is_running() and not allow_running:
            raise RuntimeError("轨迹正在执行，不能验证IK")
        if self.recording_status()["active"]:
            raise RuntimeError("请先停止动作录制，再验证IK")
        if self.absolute_recording_status()["active"]:
            raise RuntimeError("请先停止双臂绝对轨迹录制，再验证IK")
        if action.tool_frame != self.tool_frame(action.side):
            raise ValueError(
                f"动作末端坐标系不匹配: {action.tool_frame} != {self.tool_frame(action.side)}"
            )
        if max_frames is not None and not 2 <= max_frames <= 2000:
            raise ValueError("IK验证帧数必须在2到2000之间")

        snapshot = self.snapshot(action.side)
        transform = self._lookup_tool_transform(action.side).transform
        start_position = (
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
        )
        start_orientation = (
            float(transform.rotation.x),
            float(transform.rotation.y),
            float(transform.rotation.z),
            float(transform.rotation.w),
        )
        frames = smooth_relative_frames(action.relative_frames)
        if max_frames is None and len(frames) > 2000:
            raise RuntimeError("动作超过2000帧，请缩短录制后再低速试运行")
        if max_frames is None or len(frames) <= max_frames:
            selected_indices = tuple(range(len(frames)))
        else:
            selected_indices = tuple(
                round(index * (len(frames) - 1) / (max_frames - 1))
                for index in range(max_frames)
            )

        seed_names = snapshot.names
        seed_positions = snapshot.positions
        maximum_step = 0.0
        solved_positions: list[tuple[float, ...]] = []
        for frame_index in selected_indices:
            frame = frames[frame_index]
            position, orientation = compose_pose(
                start_position,
                start_orientation,
                frame.position,
                frame.orientation_xyzw,
            )
            request = self.ros["GetPositionIK"].Request()
            request.ik_request.group_name = f"{action.side}_arm"
            request.ik_request.ik_link_name = self.tool_frame(action.side)
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout.sec = 0
            request.ik_request.timeout.nanosec = 100_000_000
            request.ik_request.robot_state.is_diff = True
            request.ik_request.robot_state.joint_state.name = list(seed_names)
            request.ik_request.robot_state.joint_state.position = list(seed_positions)
            pose = self.ros["PoseStamped"]()
            pose.header.frame_id = self.base_frame
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = orientation
            request.ik_request.pose_stamped = pose
            response = self._call(
                self.compute_ik_client,
                request,
                f"compute_ik ({action.side}, frame {frame_index + 1}/{len(frames)})",
            )
            if response.error_code.val != self.ros["MoveItErrorCodes"].SUCCESS:
                raise RuntimeError(
                    f"IK不可达：第{frame_index + 1}/{len(frames)}帧，"
                    f"MoveIt错误码 {response.error_code.val}"
                )
            solution = response.solution.joint_state
            by_name = {
                str(name): float(value)
                for name, value in zip(solution.name, solution.position, strict=True)
            }
            try:
                solved = tuple(by_name[name] for name in seed_names)
            except KeyError as error:
                raise RuntimeError(f"IK结果缺少关节: {error.args[0]}") from error
            step = max(
                abs(value - previous)
                for value, previous in zip(solved, seed_positions, strict=True)
            )
            maximum_step = max(maximum_step, step)
            # A large discontinuity indicates an IK branch jump and would not
            # be safe to turn into a continuous replay trajectory.
            if step > 0.35:
                raise RuntimeError(
                    f"IK在第{frame_index + 1}/{len(frames)}帧发生关节跳变："
                    f"{math.degrees(step):.1f}°"
                )
            solved_positions.append(solved)
            seed_positions = solved
        validation = IkValidation(
            len(selected_indices), len(frames), maximum_step
        )
        return RelativeIkSolution(
            side=action.side,
            joint_names=seed_names,
            start_positions=snapshot.positions,
            time_sec=tuple(frames[index].time_sec for index in selected_indices),
            positions=tuple(solved_positions),
            validation=validation,
        )

    @staticmethod
    def _finite_difference_velocities(
        times: Sequence[float], positions: Sequence[Sequence[float]]
    ) -> tuple[tuple[float, ...], ...]:
        if len(times) != len(positions) or len(times) < 2:
            raise ValueError("轨迹时间和位置数量不匹配")
        if any(times[index] <= times[index - 1] for index in range(1, len(times))):
            raise ValueError("轨迹时间必须严格递增")
        width = len(positions[0])
        if width == 0 or any(len(point) != width for point in positions):
            raise ValueError("轨迹关节数量不一致")
        velocities: list[tuple[float, ...]] = [(0.0,) * width]
        for index in range(1, len(positions) - 1):
            delta_time = times[index + 1] - times[index - 1]
            velocities.append(
                tuple(
                    (positions[index + 1][joint] - positions[index - 1][joint])
                    / delta_time
                    for joint in range(width)
                )
            )
        velocities.append((0.0,) * width)
        return tuple(velocities)

    @staticmethod
    def _cubic_trajectory_extrema(
        times: Sequence[float],
        positions: Sequence[Sequence[float]],
        velocities: Sequence[Sequence[float]],
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        if len(times) != len(positions) or len(times) != len(velocities):
            raise ValueError("轨迹位置、速度和时间数量不匹配")
        if len(times) < 2:
            raise ValueError("轨迹至少需要两个点")
        width = len(positions[0])
        if width == 0 or any(
            len(values) != width for values in (*positions, *velocities)
        ):
            raise ValueError("轨迹关节数量不一致")
        maximum_velocity = [0.0] * width
        maximum_acceleration = [0.0] * width
        for index in range(len(times) - 1):
            duration = times[index + 1] - times[index]
            if duration <= 0.0:
                raise ValueError("轨迹时间必须严格递增")
            for joint in range(width):
                delta = positions[index + 1][joint] - positions[index][joint]
                start_velocity = velocities[index][joint]
                end_velocity = velocities[index + 1][joint]
                quadratic = (
                    3.0 * delta / (duration * duration)
                    - (2.0 * start_velocity + end_velocity) / duration
                )
                cubic = (
                    -2.0 * delta / (duration * duration * duration)
                    + (start_velocity + end_velocity) / (duration * duration)
                )
                velocity_candidates = [abs(start_velocity), abs(end_velocity)]
                if abs(cubic) > 1e-12:
                    stationary_time = -quadratic / (3.0 * cubic)
                    if 0.0 < stationary_time < duration:
                        velocity_candidates.append(
                            abs(
                                start_velocity
                                + 2.0 * quadratic * stationary_time
                                + 3.0 * cubic * stationary_time * stationary_time
                            )
                        )
                acceleration_candidates = (
                    abs(2.0 * quadratic),
                    abs(2.0 * quadratic + 6.0 * cubic * duration),
                )
                maximum_velocity[joint] = max(
                    maximum_velocity[joint], *velocity_candidates
                )
                maximum_acceleration[joint] = max(
                    maximum_acceleration[joint], *acceleration_candidates
                )
        return tuple(maximum_velocity), tuple(maximum_acceleration)

    def execute_relative_action(
        self, action: RecordedAction, *, speed_scale: float = 0.15
    ) -> IkValidation:
        """Solve and execute an action once at restricted preview speed."""
        if action.side not in SIDES:
            raise ValueError(f"未知机械臂: {action.side}")
        speed_scale = float(speed_scale)
        if not math.isfinite(speed_scale) or not 0.05 <= speed_scale <= 1.0:
            raise ValueError("试运行速度必须在5%到100%之间")
        if self.recording_status()["active"]:
            raise RuntimeError("动作正在录制，不能低速试运行")
        if self.absolute_recording_status()["active"]:
            raise RuntimeError("双臂绝对轨迹正在录制，不能低速试运行")
        if not self._execution_lock.acquire(blocking=False):
            raise RuntimeError("已有轨迹正在执行")
        side = action.side
        self._stop_requested.clear()
        with self._goal_lock:
            self._running_side = side
        try:
            other_side = "right" if side == "left" else "left"
            if self._graph_controller_mode(other_side) == "teach":
                raise RuntimeError(
                    f"{other_side}机械臂仍在拖动模式，不能开始低速试运行"
                )
            mode = self._graph_controller_mode(side)
            if mode not in {"teach", "trajectory"}:
                raise RuntimeError("所选机械臂控制器未处于拖动或保持模式")

            self._notify(side, "正在消抖并从当前姿态求解完整IK轨迹")
            solution = self._solve_relative_action(
                action, max_frames=None, allow_running=True
            )
            if self._stop_requested.is_set():
                raise TrajectoryCancelled("试运行已由操作员停止")
            if len(solution.positions) < 3:
                raise RuntimeError("IK轨迹帧数太少")
            initial_offset = max(
                abs(value - start)
                for value, start in zip(
                    solution.positions[0], solution.start_positions, strict=True
                )
            )
            if initial_offset > 0.05:
                raise RuntimeError(
                    f"IK第一帧偏离当前姿态{math.degrees(initial_offset):.1f}°，"
                    "拒绝试运行"
                )
            latest = self.snapshot(side)
            moved_during_solve = max(
                abs(value - start)
                for value, start in zip(
                    latest.positions, solution.start_positions, strict=True
                )
            )
            if moved_during_solve > 0.03:
                raise RuntimeError(
                    "IK求解期间机械臂发生移动，请保持不动后重新试运行"
                )

            action_duration = (
                solution.time_sec[-1] - solution.time_sec[0]
            ) / speed_scale
            if action_duration <= 0.0 or action_duration > 120.0:
                raise RuntimeError(
                    f"低速试运行时长{action_duration:.1f}秒不在允许范围内"
                )
            self._switch_mode_unchecked(side, "trajectory")
            if self._stop_requested.is_set():
                raise TrajectoryCancelled("试运行已由操作员停止")
            held = self.snapshot(side)
            switch_offset = max(
                abs(value - start)
                for value, start in zip(
                    held.positions, solution.start_positions, strict=True
                )
            )
            if switch_offset > 0.05:
                raise RuntimeError("控制器切换后机械臂位置变化过大，拒绝试运行")

            goal, trajectory_duration = self._make_relative_trajectory_goal(
                solution, held.positions, speed_scale
            )
            client = self._trajectory_action_client(side)
            if not client.wait_for_server(timeout_sec=self.service_timeout_sec):
                raise RuntimeError(f"{side} FollowJointTrajectory action不可用")
            self._notify(side, f"低速试运行中（{speed_scale * 100:.0f}%）")
            future = client.send_goal_async(goal)
            event = threading.Event()
            future.add_done_callback(lambda _future: event.set())
            if not event.wait(self.service_timeout_sec):
                raise TimeoutError("低速试运行轨迹请求超时")
            handle = future.result()
            if handle is None or not handle.accepted:
                raise RuntimeError("机械臂控制器拒绝了低速试运行轨迹")
            with self._goal_lock:
                self._active_goal = handle
            if self._stop_requested.is_set():
                self.stop()
            result_future = handle.get_result_async()
            result_event = threading.Event()
            result_future.add_done_callback(lambda _future: result_event.set())
            timeout = min(self.action_timeout_sec, trajectory_duration + 15.0)
            if not result_event.wait(timeout):
                self.stop()
                raise TimeoutError("低速试运行超时，已请求停止")
            wrapped = result_future.result()
            if wrapped is None:
                raise RuntimeError("低速试运行没有返回结果")
            canceled = (
                wrapped.status == self.ros["GoalStatus"].STATUS_CANCELED
                or self._stop_requested.is_set()
            )
            if canceled:
                raise TrajectoryCancelled("试运行已由操作员停止")
            error_code = int(wrapped.result.error_code)
            success = int(self.ros["FollowJointTrajectory"].Result.SUCCESSFUL)
            if error_code != success:
                raise RuntimeError(f"低速试运行失败，控制器错误码: {error_code}")
            self._controller_mode[side] = "trajectory"
            self._notify(side, "低速试运行完成并保持末点")
            return solution.validation
        except TrajectoryCancelled:
            self._notify(side, "低速试运行已停止并保持当前位置")
            raise
        except Exception as error:
            self._notify(side, f"低速试运行失败：{error}")
            raise
        finally:
            with self._goal_lock:
                self._active_goal = None
                self._running_side = None
            self._stop_requested.clear()
            self._execution_lock.release()

    def _make_relative_trajectory_goal(
        self,
        solution: RelativeIkSolution,
        held_positions: Sequence[float],
        speed_scale: float,
    ) -> tuple[object, float]:
        lead_start = 0.10
        action_start = 0.50
        relative_start = solution.time_sec[0]
        times = (lead_start,) + tuple(
            action_start + (timestamp - relative_start) / speed_scale
            for timestamp in solution.time_sec
        )
        positions = (tuple(float(value) for value in held_positions),) + tuple(
            solution.positions
        )
        velocities = self._finite_difference_velocities(times, positions)
        velocity_limits = (2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61)
        acceleration_limits = (3.75, 1.875, 2.5, 3.125, 3.75, 5.0, 5.0)
        maximum_velocities, maximum_accelerations = self._cubic_trajectory_extrema(
            times, positions, velocities
        )
        for joint, value in enumerate(maximum_velocities):
            if value > velocity_limits[joint] * 0.8:
                raise RuntimeError(
                    f"J{joint + 1}三次样条速度超限: "
                    f"{math.degrees(value):.1f}°/s"
                )
        for joint, value in enumerate(maximum_accelerations):
            if value > acceleration_limits[joint] * 0.6:
                raise RuntimeError(
                    f"J{joint + 1}三次样条加速度超限: "
                    f"{math.degrees(value):.1f}°/s²"
                )

        goal = self.ros["FollowJointTrajectory"].Goal()
        goal.trajectory.joint_names = list(solution.joint_names)
        for timestamp, joint_positions, joint_velocities in zip(
            times, positions, velocities, strict=True
        ):
            point = self.ros["JointTrajectoryPoint"]()
            point.positions = list(joint_positions)
            point.velocities = list(joint_velocities)
            total_nanoseconds = int(round(timestamp * 1_000_000_000))
            point.time_from_start = self.ros["Duration"](
                sec=total_nanoseconds // 1_000_000_000,
                nanosec=total_nanoseconds % 1_000_000_000,
            )
            goal.trajectory.points.append(point)
        goal.goal_time_tolerance = self.ros["Duration"](sec=2)
        return goal, times[-1]

    def _trajectory_action_client(self, side: str) -> object:
        namespaced, combined = self._trajectory_action_clients[side]
        if namespaced.wait_for_server(timeout_sec=0.05):
            return namespaced
        if combined.wait_for_server(timeout_sec=0.05):
            return combined
        return namespaced

    def discard_recording(self) -> bool:
        with self._record_lock:
            if self._record_side is None:
                return False
            side = self._record_side
            self._record_side = None
            self._record_samples = []
        self._notify(side, "动作录制已放弃")
        return True

    def is_running(self) -> bool:
        with self._goal_lock:
            return self._running_side is not None

    def _call(self, client: object, request: object, operation: str) -> object:
        if not client.wait_for_service(timeout_sec=self.service_timeout_sec):
            raise RuntimeError(f"ROS服务不可用: {operation}")
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(self.service_timeout_sec):
            raise TimeoutError(f"ROS服务超时: {operation}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"ROS服务无响应: {operation}")
        return result

    def _controllers(self, side: str) -> dict[str, object]:
        service = self.ros["ListControllers"]
        clients = self._clients(side)
        response = self._call(
            clients["list"],
            service.Request(),
            f"controller_manager/list_controllers ({side})",
        )
        return {controller.name: controller for controller in response.controller}

    def _graph_controller_mode(self, side: str) -> str:
        controllers = self._controllers(side)
        teach = controllers.get(f"{side}_teach_controller")
        arm = controllers.get(f"{side}_arm_controller")
        if teach is not None and teach.state == "active":
            mode = "teach"
        elif arm is not None and arm.state == "active":
            mode = "trajectory"
        else:
            mode = "inactive"
        self._controller_mode[side] = mode
        return mode

    def _clients(self, side: str) -> dict[str, object]:
        per_arm = self._manager_clients[side]
        if per_arm["list"].wait_for_service(timeout_sec=0.05):
            return per_arm
        combined = self._manager_clients["combined"]
        if combined["list"].wait_for_service(timeout_sec=0.05):
            return combined
        # Preserve the more informative per-arm service name in the eventual
        # timeout when neither stack has started yet.
        return per_arm

    def _ensure_configured(self, side: str, name: str) -> None:
        clients = self._clients(side)
        controllers = self._controllers(side)
        if name not in controllers:
            service = self.ros["LoadController"]
            request = service.Request()
            request.name = name
            response = self._call(
                clients["load"], request, f"load {name}"
            )
            if not response.ok:
                raise RuntimeError(
                    f"无法加载{name}；请确认lychee_teach_controllers已构建"
                )
            controllers = self._controllers(side)
        state = controllers[name].state
        if state == "unconfigured":
            service = self.ros["ConfigureController"]
            request = service.Request()
            request.name = name
            response = self._call(
                clients["configure"],
                request,
                f"configure {name}",
            )
            if not response.ok:
                raise RuntimeError(f"无法配置控制器{name}")
        elif state not in {"inactive", "active"}:
            raise RuntimeError(f"控制器{name}状态异常: {state}")

    def switch_mode(self, side: str, mode: str) -> None:
        if side not in SIDES or mode not in {"teach", "trajectory"}:
            raise ValueError("无效的机械臂或控制模式")
        if self.is_running():
            raise RuntimeError("轨迹正在执行，不能切换控制器")
        if self.recording_status()["active"]:
            raise RuntimeError("动作正在录制，请先停止并保存或放弃录制")
        if self.absolute_recording_status()["active"]:
            raise RuntimeError("双臂绝对轨迹正在录制，请先停止或放弃录制")
        self._switch_mode_unchecked(side, mode)

    def _switch_mode_unchecked(self, side: str, mode: str) -> None:
        wanted = f"{side}_{'teach' if mode == 'teach' else 'arm'}_controller"
        conflicting = (
            f"{side}_arm_controller" if mode == "teach" else f"{side}_teach_controller"
        )
        self._ensure_configured(side, wanted)
        clients = self._clients(side)
        controllers = self._controllers(side)
        activate = [] if controllers[wanted].state == "active" else [wanted]
        deactivate = (
            [conflicting]
            if conflicting in controllers and controllers[conflicting].state == "active"
            else []
        )
        if activate or deactivate:
            service = self.ros["SwitchController"]
            request = service.Request()
            request.activate_controllers = activate
            request.deactivate_controllers = deactivate
            request.strictness = service.Request.STRICT
            request.activate_asap = True
            request.timeout.sec = int(self.service_timeout_sec)
            response = self._call(
                clients["switch"],
                request,
                f"switch {side} to {mode}",
            )
            if not response.ok:
                raise RuntimeError(f"{side}机械臂切换到{mode}模式失败")
        self._controller_mode[side] = mode
        self._notify(side, "拖动模式" if mode == "teach" else "保持模式")

    def execute(self, routine: Routine, waypoints: Sequence[Waypoint]) -> None:
        if not waypoints:
            raise ValueError("任务中没有点位")
        if any(point.side != routine.side for point in waypoints):
            raise ValueError("任务包含另一条机械臂的点位")
        if self.recording_status()["active"]:
            raise RuntimeError("动作正在录制，不能执行轨迹")
        if self.absolute_recording_status()["active"]:
            raise RuntimeError("双臂绝对轨迹正在录制，不能执行轨迹")
        if not self._execution_lock.acquire(blocking=False):
            raise RuntimeError("已有轨迹正在执行")
        side = routine.side
        self._stop_requested.clear()
        with self._goal_lock:
            self._running_side = side
        try:
            other_side = "right" if side == "left" else "left"
            if self._graph_controller_mode(other_side) == "teach":
                raise RuntimeError(
                    f"{other_side}机械臂仍在拖动模式，不能开始轨迹"
                )
            _ = self.snapshot(side)
            self._switch_mode_unchecked(side, "trajectory")
            if self._stop_requested.is_set():
                raise TrajectoryCancelled("轨迹已由操作员停止")
            # Re-read after switching so planning starts from the held real state.
            start = self.snapshot(side)
            goal = self._make_sequence_goal(routine, waypoints, start)
            if not self.sequence_client.wait_for_server(
                timeout_sec=self.service_timeout_sec
            ):
                raise RuntimeError("MoveIt sequence_move_group action不可用")

            self._notify(side, "规划中")
            future = self.sequence_client.send_goal_async(goal)
            event = threading.Event()
            future.add_done_callback(lambda _future: event.set())
            if not event.wait(self.service_timeout_sec):
                raise TimeoutError("MoveIt轨迹请求超时")
            handle = future.result()
            if handle is None or not handle.accepted:
                raise RuntimeError("MoveIt拒绝了轨迹任务")
            with self._goal_lock:
                self._active_goal = handle
            self._notify(side, "运行中")
            if self._stop_requested.is_set():
                self.stop()
            result_future = handle.get_result_async()
            result_event = threading.Event()
            result_future.add_done_callback(lambda _future: result_event.set())
            if not result_event.wait(self.action_timeout_sec):
                self.stop()
                raise TimeoutError("轨迹执行超时，已请求停止")
            wrapped = result_future.result()
            if wrapped is None:
                raise RuntimeError("MoveIt轨迹执行无结果")
            error_code = wrapped.result.response.error_code.val
            if int(error_code) == -4:
                raise TrajectoryCancelled("轨迹已由操作员停止")
            if int(error_code) != 1:
                raise RuntimeError(f"MoveIt轨迹失败，错误码: {error_code}")
            self._controller_mode[side] = "trajectory"
            self._notify(side, "完成并保持末点")
        except TrajectoryCancelled:
            self._notify(side, "已停止并保持当前位置")
            raise
        except Exception as error:
            self._notify(side, f"失败：{error}")
            raise
        finally:
            with self._goal_lock:
                self._active_goal = None
                self._running_side = None
            self._stop_requested.clear()
            self._execution_lock.release()

    def _make_sequence_goal(
        self,
        routine: Routine,
        waypoints: Sequence[Waypoint],
        start: JointSnapshot,
    ) -> object:
        goal = self.ros["MoveGroupSequence"].Goal()
        for index, point in enumerate(waypoints):
            item = self.ros["MotionSequenceItem"]()
            item.req.group_name = f"{routine.side}_arm"
            item.req.pipeline_id = "pilz_industrial_motion_planner"
            item.req.planner_id = "PTP"
            item.req.allowed_planning_time = 5.0
            item.req.num_planning_attempts = 1
            item.req.max_velocity_scaling_factor = routine.velocity_scale
            item.req.max_acceleration_scaling_factor = routine.acceleration_scale
            constraints = self.ros["Constraints"]()
            constraints.name = point.name
            for joint_name, position in zip(start.names, point.joints, strict=True):
                constraint = self.ros["JointConstraint"]()
                constraint.joint_name = joint_name
                constraint.position = position
                constraint.tolerance_above = 1e-4
                constraint.tolerance_below = 1e-4
                constraint.weight = 1.0
                constraints.joint_constraints.append(constraint)
            item.req.goal_constraints = [constraints]
            if index == 0:
                item.req.start_state.is_diff = True
                item.req.start_state.joint_state.name = list(start.names)
                item.req.start_state.joint_state.position = list(start.positions)
            item.blend_radius = (
                routine.blend_radius_m if index < len(waypoints) - 1 else 0.0
            )
            goal.request.items.append(item)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        return goal

    def stop(self) -> bool:
        with self._goal_lock:
            handle = self._active_goal
            side = self._running_side
        if side is None:
            return False
        self._stop_requested.set()
        if handle is None:
            self._notify(side, "将在控制器切换完成后停止并保持")
            return True
        # Ask the top-level planner to unwind, but do not rely on it for timely
        # stopping: Pilz may not answer this request until execution returns.
        _ = handle.cancel_goal_async()

        cancel_client = self._trajectory_cancel_client(side)
        request = self.ros["CancelGoal"].Request()
        # A zero goal ID and zero timestamp cancels every goal on this action
        # server, which is safe because each arm controller has one owner in
        # MoveIt mode.
        response = self._call(
            cancel_client,
            request,
            f"cancel {side} FollowJointTrajectory",
        )
        delivered = response is not None
        if delivered:
            self._notify(side, "正在停止并保持")
        return delivered

    def _trajectory_cancel_client(self, side: str) -> object:
        namespaced, combined = self._trajectory_cancel_clients[side]
        if namespaced.wait_for_service(timeout_sec=0.05):
            return namespaced
        if combined.wait_for_service(timeout_sec=0.05):
            return combined
        return namespaced

    def shutdown(self) -> None:
        self.discard_recording()
        self.discard_absolute_recording()
        self.executor.shutdown(timeout_sec=2.0)
        self._spin_thread.join(timeout=2.0)
        self.node.destroy_node()
        if self.ros["rclpy"].ok():
            self.ros["rclpy"].shutdown()
