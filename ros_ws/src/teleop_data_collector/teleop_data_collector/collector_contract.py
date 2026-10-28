from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from teleop_core.contract import (
    COMMAND_STATUS_TOPIC,
    DATA_TELEOPERATORS,
    VALIDATED_COMMAND_TOPIC,
    WUJI_TELEMETRY_STATUS_TOPIC,
)


SUPPORTED_TELEOPERATORS = DATA_TELEOPERATORS
SHARPA_TOPICS = {
    "/sharpa/left/command",
    "/sharpa/right/command",
    "/sharpa/left/joint_states",
    "/sharpa/right/joint_states",
}


@dataclass(frozen=True)
class TopicContract:
    name: str
    topic: str
    type_name: str
    required: bool = True
    min_frequency_hz: float = 1.0
    max_gap_ms: float = 150.0


@dataclass(frozen=True)
class RecordingOutcome:
    state: str
    finalized: bool
    failures: tuple[str, ...]


def recording_outcome(
    *,
    exit_code: int,
    stream_failures: Sequence[str] = (),
    interrupted: bool = False,
    discarded: bool = False,
) -> RecordingOutcome:
    failures = tuple(stream_failures)
    if discarded:
        return RecordingOutcome("discarded", False, failures)
    if interrupted:
        return RecordingOutcome("interrupted", False, failures)
    if exit_code != 0:
        return RecordingOutcome(
            "incomplete", False, failures + (f"recorder exit code {exit_code}",)
        )
    if failures:
        return RecordingOutcome("incomplete", False, failures)
    return RecordingOutcome("finalized", True, ())


def validate_recording_contract(config: Mapping[str, Any]) -> None:
    if config.get("bag_contract_version") != 1:
        raise ValueError("bag_contract_version must equal 1")
    teleoperator = _text(config, "teleoperator")
    if teleoperator not in SUPPORTED_TELEOPERATORS:
        raise ValueError(
            f"teleoperator must be one of {SUPPORTED_TELEOPERATORS}"
        )
    for field in (
        "workcell_id",
        "workcell_config_hash",
        "control_config_id",
        "timestamp_policy",
    ):
        _text(config, field)
    for field in ("trim_start_sec", "trim_end_sec"):
        value = float(config.get(field, 0.0))
        if value < 0.0:
            raise ValueError(f"{field} must be non-negative")
    for field in ("calibration_ids", "device_identities"):
        value = config.get(field)
        if not isinstance(value, Mapping) or not value:
            raise ValueError(f"{field} must be a non-empty mapping")

    topics = topic_contracts(config.get("topics"))
    required_topics = {topic.topic for topic in topics if topic.required}
    if VALIDATED_COMMAND_TOPIC not in required_topics:
        raise ValueError(
            "recording contract is missing the safety-gateway output"
        )
    if COMMAND_STATUS_TOPIC not in required_topics:
        raise ValueError(
            f"recording contract is missing HOLD status topic {COMMAND_STATUS_TOPIC}"
        )
    wuji_topics = {
        topic for topic in required_topics if topic.startswith("/teleop/wuji/")
    }
    if wuji_topics and WUJI_TELEMETRY_STATUS_TOPIC not in required_topics:
        raise ValueError(
            "recording contract is missing HOLD status topic "
            f"{WUJI_TELEMETRY_STATUS_TOPIC}"
        )
    sharpa_topics = required_topics.intersection(SHARPA_TOPICS)
    if sharpa_topics and sharpa_topics != SHARPA_TOPICS:
        missing = sorted(SHARPA_TOPICS.difference(sharpa_topics))
        raise ValueError(
            "Sharpa recording contract must include both command and state "
            f"topics for both hands; missing {missing}"
        )
    forbidden = {"/teleop/arm_commands", "/target_robot/joint_commands"}
    if forbidden & {topic.topic for topic in topics}:
        raise ValueError("recording contract includes a pre-gateway/control-bus topic")


def topic_contracts(raw_topics: Any) -> tuple[TopicContract, ...]:
    if not isinstance(raw_topics, Mapping) or not raw_topics:
        raise ValueError("topics must be a non-empty mapping")
    result = []
    for name, raw in sorted(raw_topics.items()):
        if not isinstance(raw, Mapping):
            raise ValueError(f"topics.{name} must be a mapping")
        min_frequency_hz = float(raw.get("min_frequency_hz", 1.0))
        max_gap_ms = float(raw.get("max_gap_ms", 150.0))
        if min_frequency_hz <= 0.0 or max_gap_ms <= 0.0:
            raise ValueError(
                f"topics.{name} frequency and max gap must be positive"
            )
        result.append(
            TopicContract(
                name=str(name),
                topic=_text(raw, "topic"),
                type_name=_text(raw, "type"),
                required=bool(raw.get("required", True)),
                min_frequency_hz=min_frequency_hz,
                max_gap_ms=max_gap_ms,
            )
        )
    names = [topic.topic for topic in result]
    if len(names) != len(set(names)):
        raise ValueError("recording topic names must be unique")
    return tuple(result)


def topic_preflight_failures(
    configured: Sequence[TopicContract], available: Mapping[str, Sequence[str]]
) -> tuple[str, ...]:
    failures = []
    for topic in configured:
        if not topic.required:
            continue
        types = tuple(available.get(topic.topic, ()))
        if not types:
            failures.append(f"{topic.topic}: missing")
        elif topic.type_name not in types:
            failures.append(
                f"{topic.topic}: type mismatch, expected {topic.type_name}, got {list(types)}"
            )
    return tuple(failures)


def _text(parent: Mapping[str, Any], key: str) -> str:
    value = str(parent.get(key, "")).strip()
    if not value:
        raise ValueError(f"{key} must be a non-empty string")
    return value
