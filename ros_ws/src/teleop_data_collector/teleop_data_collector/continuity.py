"""Bounded source-time continuity statistics shared by live and bag checks.

Sharpa stamps describe ROS publication continuity, not SDK acquisition time.
Receive silence is deliberately reported separately from proven source gaps.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
import threading
import uuid
from typing import Any

from teleop_core.contract import (
    COMMAND_STATUS_TOPIC, VALIDATED_COMMAND_TOPIC,
    LEFT_COMMAND_JOINT_NAMES, RIGHT_COMMAND_JOINT_NAMES,
)


def detection_enabled(topic: str, tactile_detection_enabled: bool = True) -> bool:
    """Select recorder checks only; leave recorded topics and control untouched."""
    return tactile_detection_enabled or not (
        topic.startswith("/sharpa/") and "/tactile/" in topic
    )


@dataclass(frozen=True)
class ContinuityPolicy:
    frequency_hz: float | None = 30.0
    warning_gap_ms: float = 55.0
    failure_gap_ms: float = 121.6666666667
    kind: str = "freshness"
    silence_sec: float = 1.0

    @classmethod
    def periodic(cls, hz: float, kind: str) -> ContinuityPolicy:
        period = 1000.0 / hz
        return cls(hz, 1.5 * period + 5.0, 3.5 * period + 5.0, kind)

    def missing(self, gap_ms: float) -> int | None:
        if self.frequency_hz is None:
            return None
        return max(0, math.floor((gap_ms - 5.0) * self.frequency_hz / 1000.0 + 0.5) - 1)

    def severity(self, gap_ms: float) -> str | None:
        missing = self.missing(gap_ms)
        if missing is not None:
            return "failure" if missing >= 3 else "warning" if missing >= 1 else None
        return "failure" if gap_ms > self.failure_gap_ms else "warning" if gap_ms > self.warning_gap_ms else None

    def report(self) -> dict[str, Any]:
        return {"expected_frequency_hz": self.frequency_hz,
                "warning_gap_ms": self.warning_gap_ms, "failure_gap_ms": self.failure_gap_ms,
                "kind": self.kind, "silence_sec": self.silence_sec, "jitter_allowance_ms": 5.0}


def policy_for_topic(topic: str) -> ContinuityPolicy:
    if topic.startswith("/sharpa/") and "/tactile/" in topic:
        return ContinuityPolicy(None, 250.0, 500.0, "publication_gap")
    if topic.startswith("/cam"):
        return ContinuityPolicy.periodic(20.0 if topic.startswith("/cam0/") else 30.0, "missing_frames")
    return ContinuityPolicy.periodic(30.0, "freshness")


@dataclass
class StreamStats:
    policy: ContinuityPolicy
    enabled: bool = True
    last_source_ns: int | None = None
    engagement_source_ns: int | None = None
    last_receive: float | None = None
    waiting_since: float = 0.0
    sample_count: int = 0
    max_source_gap_ms: float = 0.0
    max_receive_gap_ms: float = 0.0
    source_gap_warning_count: int = 0
    source_gap_failure_count: int = 0
    transport_warning_count: int = 0
    timestamp_failure_count: int = 0
    silence_failure_count: int = 0
    estimated_missing_frames_total: int = 0
    estimated_missing_frames_max_run: int = 0
    silence_reported: bool = False
    last_notice: dict[str, float] = field(default_factory=dict)

    def report(self) -> dict[str, Any]:
        return {**self.policy.report(), **{key: getattr(self, key) for key in (
            "enabled", "sample_count", "max_source_gap_ms", "max_receive_gap_ms",
            "source_gap_warning_count", "source_gap_failure_count", "transport_warning_count",
            "timestamp_failure_count", "silence_failure_count",
            "estimated_missing_frames_total", "estimated_missing_frames_max_run")}}


class ContinuityMonitor:
    def __init__(self, topics: list[str], *, mode="legacy", alignment_policy=None):
        if mode not in ('legacy', 'conversion_aligned'):
            raise ValueError('Unsupported continuity_mode: ' + str(mode))
        self.mode = mode
        self.alignment_policy = alignment_policy
        self.alignment = None
        self._topics = topics
        self._lock = threading.RLock()
        self.session_id = uuid.uuid4().hex
        self._sequence = 0
        self.active = False
        self.episode = ""
        self.streams: dict[str, StreamStats] = {}
        self.alerts: dict[tuple[str, str], dict[str, Any]] = {}
        self.first_failure: dict[str, Any] | None = None
        self.started_at = 0.0
        self._ended_at: float | None = None
        self.driver_health: dict[str, dict[str, Any]] = {}

    def begin(self, episode: str, now: float) -> None:
        with self._lock:
            self.active = True
            self.started_at = now
            self._ended_at = None
            self.episode = episode
            self.alerts = {}
            self.first_failure = None
            self.streams = {}
            if self.mode == 'conversion_aligned':
                from .alignment_quality import AlignmentQuality
                self.alignment = AlignmentQuality(self._topics, self.alignment_policy)
            for topic in self._topics:
                if topic == VALIDATED_COMMAND_TOPIC:
                    for side in ("left", "right"):
                        self.streams[f"{topic}#{side}"] = StreamStats(policy_for_topic(topic), False, waiting_since=now)
                else:
                    self.streams[topic] = StreamStats(policy_for_topic(topic), waiting_since=now)

    def end(self, now: float | None = None, *, alignment_window=None) -> dict[str, Any]:
        with self._lock:
            if self.alignment is not None and alignment_window is not None:
                self.alignment.advance(now or time.monotonic(), final=True,
                    start_ns=alignment_window[0], end_ns=alignment_window[1])
                self._alignment_notice(now or time.monotonic())
            self.active = False
            self._ended_at = time.monotonic() if now is None else now
            return self.snapshot()

    def observe_message(self, topic: str, message: Any, header_ns: int | None, now: float) -> None:
        with self._lock:
            if not self.active:
                return
            if self.alignment is not None:
                self.alignment.observe(topic, message, header_ns, now)
            if topic == COMMAND_STATUS_TOPIC:
                accepted = set(message.accepted_sides)
                for side in ("left", "right"):
                    stats = self.streams.get(f"{VALIDATED_COMMAND_TOPIC}#{side}")
                    if stats is not None and stats.enabled != (side in accepted):
                        stats.enabled = side in accepted
                        stats.last_source_ns = None
                        stats.engagement_source_ns = header_ns if stats.enabled else None
                        stats.last_receive = None
                        stats.waiting_since = now
                        stats.silence_reported = False
            if topic == VALIDATED_COMMAND_TOPIC:
                names = set(message.name)
                for side, expected in (("left", LEFT_COMMAND_JOINT_NAMES), ("right", RIGHT_COMMAND_JOINT_NAMES)):
                    if set(expected).issubset(names):
                        self.observe(f"{topic}#{side}", header_ns, now)
            else:
                self.observe(topic, header_ns, now)

    def observe(self, topic: str, header_ns: int | None, now: float) -> None:
        with self._lock:
            stats = self.streams.get(topic)
            if not self.active or stats is None or not stats.enabled:
                return
            stats.sample_count += 1
            receive_gap = (now - stats.last_receive) * 1000 if stats.last_receive is not None else 0.0
            stats.max_receive_gap_ms = max(stats.max_receive_gap_ms, receive_gap)
            stats.last_receive = now
            stats.silence_reported = False
            if header_ns is None or header_ns <= 0 or (stats.last_source_ns is not None and header_ns <= stats.last_source_ns):
                stats.timestamp_failure_count += 1
                previous_ns = stats.last_source_ns
                if header_ns is None or header_ns <= 0:
                    message = "source 时间戳缺失或为零"
                elif header_ns == previous_ns:
                    message = "source 时间戳重复"
                else:
                    message = f"source 时间戳倒退 {(previous_ns - header_ns) / 1e6:.3f} ms"
                self._notice(topic, "failure", "timestamp", None, None, now, message)
                alert = self.alerts[(topic, "timestamp")]
                alert.update(previous_source_ns=previous_ns, current_source_ns=header_ns)
                if self.first_failure is not None and self.first_failure["id"] == alert["id"]:
                    self.first_failure.update(previous_source_ns=previous_ns, current_source_ns=header_ns)
                return
            baseline = stats.last_source_ns if stats.last_source_ns is not None else stats.engagement_source_ns
            if baseline is not None:
                gap = max(0.0, (header_ns - baseline) / 1e6)
                stats.max_source_gap_ms = max(stats.max_source_gap_ms, gap)
                missing = stats.policy.missing(gap)
                if missing is not None:
                    stats.estimated_missing_frames_total += missing
                    stats.estimated_missing_frames_max_run = max(stats.estimated_missing_frames_max_run, missing)
                severity = stats.policy.severity(gap)
                if severity:
                    field_name = f"source_gap_{severity}_count"
                    setattr(stats, field_name, getattr(stats, field_name) + 1)
                    if self.mode == 'conversion_aligned':
                        severity = 'warning'
                    self._notice(topic, severity, "source", gap, missing, now,
                                 "source 缺帧" if stats.policy.kind == "missing_frames" else "发布新鲜度缺口")
                elif receive_gap > stats.policy.warning_gap_ms:
                    stats.transport_warning_count += 1
                    self._notice(topic, "warning", "transport", receive_gap, None, now,
                                 "接收抖动，source 时间戳连续")
            stats.last_source_ns = header_ns

    def invalid(self, topic: str, error: str, now: float) -> None:
        with self._lock:
            if self.active and topic in self.streams:
                self.streams[topic].timestamp_failure_count += 1
                self._notice(topic, "failure", "invalid", None, None, now, error)

    def observe_driver_health(self, topic: str, health: dict[str, Any], now: float, *, bag_event=False, from_bag=False) -> None:
        with self._lock:
            if topic not in self._topics:
                return
            # One bounded summary per required image stream, including idle uptime.
            previous = self.driver_health.get(topic, {})
            if previous.get('pipeline_start_steady_us') != health.get('pipeline_start_steady_us'):
                previous = {}
            # Immediate loss events are intentionally small; retain the latest
            # SDK/publish metrics from the same Pipeline generation.
            self.driver_health[topic] = {**previous, **health,
                'recording_elapsed_sec': max(0.0, now-self.started_at) if self.active else None}
            if not self.active or topic not in self.streams:
                return
            dropped_at = float(health.get("last_queue_drop_steady_us") or 0) / 1e6
            if (bag_event if from_bag else dropped_at >= self.started_at) and health.get("queue_drop_total", 0) > 0:
                previous = self.alerts.get((topic, "driver_queue"))
                if previous is None:
                    self._notice(topic, "warning" if self.mode == 'conversion_aligned' else "failure", "driver_queue", None, None, now,
                                 "相机驱动颜色队列溢出，主动丢弃帧")

    def watchdog(self, now: float) -> None:
        with self._lock:
            if not self.active:
                return
            self.advance_alignment(now)
            for topic, stats in self.streams.items():
                age = now - (stats.last_receive if stats.last_receive is not None else stats.waiting_since)
                if stats.enabled and not stats.silence_reported and age > stats.policy.silence_sec:
                    stats.silence_reported = True
                    stats.silence_failure_count += 1
                    self._notice(topic, "failure", "silence", age * 1000, None, now,
                                 "接收中断，源或传输原因待确认")

    def advance_alignment(self, now):
        with self._lock:
            if self.active and self.alignment is not None:
                self.alignment.advance(now)
                self._alignment_notice(now)

    def _alignment_notice(self, now):
        reason = self.alignment.first_failure
        if reason is None:
            return
        topic = reason.get('topic')
        if topic not in self.streams:
            topic = next(iter(self.streams))
        key = (topic, 'alignment')
        if key in self.alerts:
            return
        message = reason['message']
        if reason.get('grid_index') is not None:
            failed_topic = reason.get('topic', topic)
            age = reason.get('ages_ns', {}).get(failed_topic)
            age_text = '无可用历史样本' if age is None else f'age={age / 1e6:.3f} ms'
            message = f"30 Hz 对齐质量失败：内部网格 #{reason['grid_index']}，{failed_topic}，{age_text}"
        self._notice(topic, 'failure', 'alignment', None, None, now, message)
        self.alerts[key].update(alignment=dict(reason))
        if self.first_failure is not None and self.first_failure['id'] == self.alerts[key]['id']:
            self.first_failure.update(alignment=dict(reason))

    def _notice(self, topic: str, severity: str, category: str, gap: float | None,
                missing: int | None, now: float, message: str) -> None:
        stats = self.streams[topic]
        key = (topic, category)
        previous = self.alerts.get(key)
        if previous and previous["severity"] == "failure" and severity != "failure":
            previous["count"] += 1
            return
        # Aggregate warnings at most every 2 s; a severity escalation is immediate.
        if previous and previous["severity"] == severity and now - stats.last_notice.get(category, -math.inf) < 2.0:
            previous["count"] += 1
            return
        self._sequence += 1
        alert = {"id": self._sequence, "session_id": self.session_id, "episode": self.episode,
                 "topic": topic, "severity": severity, "category": category,
                 "gap_ms": gap, "estimated_missing_frames": missing,
                 "kind": stats.policy.kind, "message": message,
                 "count": (previous["count"] + 1) if previous else 1}
        alert["recording_elapsed_sec"] = max(0.0, now - self.started_at)
        health = self.driver_health.get(topic, {})
        alert["camera_uptime_sec"] = health.get("camera_uptime_sec")
        self.alerts[key] = alert
        stats.last_notice[category] = now
        if severity == "failure" and self.first_failure is None:
            self.first_failure = dict(alert)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            alerts = [dict(value) for value in self.alerts.values()]
            return {"continuity_mode": self.mode,
                    "alignment_quality": self.alignment.report() if self.alignment is not None else {},
                    "legacy_gap_diagnostics": {"exceedance_as_warning_only": self.mode == 'conversion_aligned'},
                    "episode": self.episode, "session_id": self.session_id,
                    "active": self.active,
                    "recording_elapsed_sec": max(0.0, (self._ended_at if self._ended_at is not None else time.monotonic()) - self.started_at) if self.episode else None,
                    "driver_health": {topic: dict(value) for topic, value in self.driver_health.items()},
                    "streams": {topic: stats.report() for topic, stats in self.streams.items()},
                    "alerts": alerts, "auto_stop_reason": dict(self.first_failure) if self.first_failure else None,
                    "source_warnings": [format_alert(a) for a in alerts if a["severity"] == "warning" and a["category"] == "source"],
                    "transport_warnings": [format_alert(a) for a in alerts if a["category"] == "transport"],
                    "failures": [format_alert(a) for a in alerts if a["severity"] == "failure"],
                    "timestamp_semantics": "Sharpa header stamps describe ROS publication continuity"}


def format_alert(alert: dict[str, Any]) -> str:
    detail = f"{alert['topic']}: {alert['message']}"
    if alert.get("gap_ms") is not None:
        detail += f", gap={alert['gap_ms']:.1f} ms"
    if alert.get("estimated_missing_frames") is not None:
        label = "estimated_missing_frames" if alert.get("kind") == "missing_frames" else "missing_30hz_steps"
        detail += f", {label}={alert['estimated_missing_frames']}"
    return detail
