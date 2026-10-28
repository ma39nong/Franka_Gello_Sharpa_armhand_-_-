"""Explicit GUI-owned Sharpa opening, separate from glove-follow permits.

The existing Sharpa schema defines OPEN as all 22 joints at zero radians.
Only the managed MANUS normal/fake path uses this fixed, bounded action.
"""
from dataclasses import dataclass
import time

import numpy as np

from .retarget import JOINT_NAMES, JOINT_LIMITS
from .validation_safety import TIMEOUT_S

OPEN_POSITION = np.zeros(len(JOINT_NAMES))
OPEN_SPEED_RAD_S = 0.5
OPEN_RATE_HZ = 30.0
# Opening is a coarse posture action. Physical hands can retain roughly
# 0.17–0.22 rad of joint error at the zero target; this is NOT a joint-limit
# allowance and does not change the commanded target or motion/lease checks.
SETTLE_TOLERANCE_RAD = 0.25
SETTLE_SECONDS = 0.15
SETTLE_TIMEOUT_SECONDS = 3.0


def checked_feedback(values, tolerance=0.02):
    q = np.asarray(values, dtype=float)
    if q.shape != OPEN_POSITION.shape or not np.isfinite(q).all():
        raise ValueError('Open hand requires 22 finite measured joint positions')
    if np.any(q < JOINT_LIMITS[:, 0] - tolerance) or np.any(q > JOINT_LIMITS[:, 1] + tolerance):
        raise ValueError('Open hand feedback outside measurement allowance')
    return np.clip(q, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])


@dataclass
class Opening:
    start: np.ndarray
    started: float
    duration: float
    last_send: float = -float('inf')
    settled_since: float | None = None
    desired: np.ndarray | None = None
    prepared_at: float | None = None

    def target(self, now):
        fraction = min(1.0, max(0.0, (now - self.started) / self.duration))
        return self.start * (1.0 - fraction)


class SharpaOpenController:
    def __init__(self, runtime):
        self.runtime = runtime
        self.operations = {}
        self.results = {}

    def _check_lease(self, now):
        r = self.runtime
        if r.config['source'] != 'manus' or r.config['mode'] not in ('normal', 'fake'):
            raise ValueError('Open hand is only available in managed MANUS normal/fake mode')
        if r.latch.phase == 'FAULT':
            raise ValueError('Open hand refused: ' + r.latch.reason)
        if r.latch.last_gui is None or not 0 <= now - r.latch.last_gui <= TIMEOUT_S:
            raise ValueError('Open hand requires a live GUI lease')

    def hand(self, side):
        return self.runtime.node._hands[next(s for s in self.runtime.node._hands if s.value == side)]

    def cancel(self, side):
        if self.operations.pop(side, None) is not None:
            self.results[side] = '张开已取消，遥操使能关闭'

    def start(self, side):
        sides = ('left', 'right') if side == 'both' else (side,)
        if any(s not in ('left', 'right') for s in sides):
            raise ValueError('Open hand requires left, right or both')
        self._check_lease(time.monotonic())
        # Revoke every selected permit before sampling. No actuator command is
        # issued until all selected feedback has been checked successfully.
        for s in sides:
            self.runtime.request(s, False)
        starts = {s: checked_feedback(self.hand(s).read_joint_state().position,
                   getattr(self.runtime.latch, 'feedback_limit_tolerance', 0.0)) for s in sides}
        # Native enable verification can take longer than the GUI lease. Do it
        # before creating any motion target, then require a NEW heartbeat after
        # preparation. The control connection can reply and resume heartbeats
        # before the first command is authorized.
        try:
            for s in sides:
                enum = next(sid for sid in self.runtime.node._hands if sid.value == s)
                self.runtime.node.prepare_open(enum)
        except Exception:
            for s in sides:
                self.runtime.stop_hardware_side(s)
            raise
        now = time.monotonic()
        for s, q in starts.items():
            self.operations[s] = Opening(q, now, max(2.0, float(np.max(np.abs(q))) / OPEN_SPEED_RAD_S), prepared_at=now)
            self.results[s] = '张开准备完成，等待新 GUI 心跳；遥操使能关闭'

    def authorize(self, side, message):
        now = time.monotonic()
        self._check_lease(now)
        job = self.operations.get(side)
        if job is None or job.prepared_at is not None or job.desired is None or self.runtime.latch.requested[side]:
            raise ValueError('No authoritative explicit opening action')
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (self.runtime.node.get_clock().now().nanoseconds - stamp) / 1e9
        if not 0 <= age <= TIMEOUT_S:
            raise ValueError('Stale/future opening command')
        expected_id = 'supervisor_open:' + side + ':' + self.runtime.config['token'][:16]
        if (tuple(message.name) != JOINT_NAMES or message.header.frame_id != expected_id
                or not np.array_equal(np.asarray(message.position), job.desired)):
            raise ValueError('Opening command does not match its fixed action')

    def tick(self):
        if not self.operations:
            return
        try:
            now = time.monotonic()
            preparing = [job for job in self.operations.values() if job.prepared_at is not None]
            if preparing:
                last_gui = self.runtime.latch.last_gui
                if last_gui is None or any(last_gui <= job.prepared_at for job in preparing):
                    if any(now - job.prepared_at > TIMEOUT_S for job in preparing):
                        raise ValueError('Open hand requires a fresh GUI lease after hardware preparation')
                    return
                self._check_lease(now)
                # Sample again after enable: the disabled hand may have moved.
                starts = {s: checked_feedback(self.hand(s).read_joint_state().position,
                    getattr(self.runtime.latch, 'feedback_limit_tolerance', 0.0))
                    for s, job in self.operations.items() if job.prepared_at is not None}
                now = time.monotonic()
                self._check_lease(now)
                for s, q in starts.items():
                    job = self.operations[s]
                    job.start = q
                    job.started = now
                    job.duration = max(2.0, float(np.max(np.abs(q))) / OPEN_SPEED_RAD_S)
                    job.prepared_at = None
                    self.results[s] = '正在张开，遥操使能关闭'
            self._check_lease(now)
            from sensor_msgs.msg import JointState
            for side, job in list(self.operations.items()):
                if now - job.last_send < 1.0 / OPEN_RATE_HZ:
                    continue
                measured = checked_feedback(self.hand(side).read_joint_state().position,
                    getattr(self.runtime.latch, 'feedback_limit_tolerance', 0.0))
                if now - job.started > job.duration + SETTLE_TIMEOUT_SECONDS:
                    error = np.abs(measured - OPEN_POSITION)
                    joint = int(np.argmax(error))
                    raise ValueError(f'{side} Open hand did not settle before timeout: '
                        f'joint={JOINT_NAMES[joint]}, measured={measured[joint]:.5f} rad, '
                        f'target={OPEN_POSITION[joint]:.5f} rad, error={error[joint]:.5f} rad, '
                        f'tolerance={SETTLE_TOLERANCE_RAD:.5f} rad')
                job.desired = job.target(now)
                msg = JointState()
                msg.header.stamp = self.runtime.node.get_clock().now().to_msg()
                msg.header.frame_id = 'supervisor_open:' + side + ':' + self.runtime.config['token'][:16]
                msg.name = list(JOINT_NAMES)
                msg.position = job.desired.tolist()
                enum = next(s for s in self.runtime.node._hands if s.value == side)
                self.runtime.node.forward_open(enum, msg)
                self.runtime.publishers[side].publish(msg)
                job.last_send = now
                if now - job.started >= job.duration and np.max(np.abs(measured)) <= SETTLE_TOLERANCE_RAD:
                    job.settled_since = now if job.settled_since is None else job.settled_since
                    if now - job.settled_since >= SETTLE_SECONDS:
                        self.runtime.stop_hardware_side(side)
                        self.operations.pop(side)
                        self.results[side] = '已张开，遥操使能关闭；点击 Start hand 才恢复'
                else:
                    job.settled_since = None
        except Exception as error:
            for side in self.operations:
                self.results[side] = '张开失败：' + str(error)
            self.operations.clear()
            self.runtime.node.stop_validation('Open hand failed: ' + str(error))
