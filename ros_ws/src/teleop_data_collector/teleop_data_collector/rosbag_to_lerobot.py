from __future__ import annotations

import argparse
import json
import re
import shutil
import os
import tempfile
import uuid
from collections import defaultdict
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from dataclasses import field as dataclass_field
from itertools import groupby
from pathlib import Path
from typing import Any

from teleop_core.contract import (
    CAMERA_COLOR_TOPIC,
    COMMAND_STATUS_TOPIC,
    DATA_LEFT_ARM_JOINT_NAMES,
    DATA_RIGHT_ARM_JOINT_NAMES,
    DATA_TELEOPERATORS,
    LEFT_COMMAND_JOINT_NAMES,
    RIGHT_COMMAND_JOINT_NAMES,
    VALIDATED_COMMAND_TOPIC,
    WUJI_COMMAND_TOPIC,
    WUJI_LEFT_JOINT_NAMES,
    WUJI_RIGHT_JOINT_NAMES,
    WUJI_STATE_TOPIC,
    WUJI_TELEMETRY_STATUS_TOPIC,
)

FORMAT_VERSION = "teleop.harvest.lerobot.v2.0"
LEROBOT_CODEBASE_VERSION = "v2.0"
DEFAULT_CHUNK_SIZE = 1000
DEFAULT_FPS = 10
DEFAULT_TASK = "default"
# Standalone dataloader copied into every converted dataset root.
DATALOADER_TEMPLATE_NAME = "dataloader_template.py"
ENGAGEMENT_ACTION_SKEW_NS = 20_000_000


@dataclass(frozen=True)
class SourceSpec:
    topic: str
    type_name: str | None = None
    field: str | None = None
    fields: list[str] = dataclass_field(default_factory=list)
    required: bool = True
    names: list[str] = dataclass_field(default_factory=list)
    joint_names: list[str] = dataclass_field(default_factory=list)
    allow_extra_joint_names: bool = False


@dataclass(frozen=True)
class FeatureSpec:
    column: str
    dtype: str
    sources: list[SourceSpec]
    names: list[str] = dataclass_field(default_factory=list)


@dataclass(frozen=True)
class VideoSpec:
    column: str
    topic: str
    type_name: str | None = "sensor_msgs/msg/Image"
    required: bool = True
    fps: int | None = None


@dataclass(frozen=True)
class DepthImageSpec:
    column: str
    topic: str
    type_name: str | None = "sensor_msgs/msg/Image"
    required: bool = True
    format: str = "raw16"
    fps: int | None = None


@dataclass(frozen=True)
class PointCloudSpec:
    column: str
    topic: str
    type_name: str | None = "sensor_msgs/msg/PointCloud2"
    required: bool = True
    primary: str = "cdr"
    derived_formats: list[str] = dataclass_field(default_factory=list)


@dataclass(frozen=True)
class StaticTopicSpec:
    key: str
    topic: str
    type_name: str | None = None
    required: bool = True


@dataclass(frozen=True)
class SamplingSpec:
    strategy: str
    topic: str | None = None
    frequency_hz: float | None = None
    max_staleness_ns: int | None = None
    trim_start_ns: int = 0
    trim_end_ns: int = 0


@dataclass
class StaticTopicState:
    key: str
    topic: str
    type_name: str
    message_count: int = 0
    first_bag_time_ns: int | None = None
    last_bag_time_ns: int | None = None
    first_value: Any | None = None
    last_value: Any | None = None
    changed: bool = False
    _first_json: str | None = None

    def add_message(self, bag_time_ns: int, value: Any) -> None:
        value_json = _json_dumps(value)
        if self.message_count == 0:
            self.first_bag_time_ns = bag_time_ns
            self.first_value = value
            self._first_json = value_json
        elif value_json != self._first_json:
            self.changed = True

        self.message_count += 1
        self.last_bag_time_ns = bag_time_ns
        self.last_value = value

    def to_dict(self) -> dict[str, Any]:
        return {
            "key": self.key,
            "topic": self.topic,
            "type_name": self.type_name,
            "message_count": self.message_count,
            "first_bag_time_ns": self.first_bag_time_ns,
            "last_bag_time_ns": self.last_bag_time_ns,
            "changed": self.changed,
            "value": self.first_value,
            "last_value": self.last_value if self.changed else None,
        }


@dataclass
class EpisodeResult:
    episode_index: int
    bag_path: Path
    length: int
    data_path: str
    video_paths: dict[str, str]
    depth_image_storage: dict[str, str]
    pointcloud_storage: dict[str, str]
    static_topics: list[dict[str, Any]]
    feature_dims: dict[str, int]
    video_shapes: dict[str, list[int]]
    depth_image_shapes: dict[str, list[int]]
    message_counts: dict[str, int]
    stats: dict[str, Any]
    rows_skipped_incomplete: int
    unmatched_by_stream: dict[str, int]
    source_timeline_start_ns: int
    source_common_start_ns: int
    source_common_end_ns: int
    source_effective_start_ns: int
    source_effective_end_ns: int
    segment_id: str = "full"
    task_index: int = 0
    source_recording_id: str = ""
    milestone_timestamp_ns: int | None = None


@dataclass
class SourceTimedEvent:
    source_time_ns: int
    bag_time_ns: int
    sequence: int
    topic: str
    feature_updates: list[tuple[tuple[str, int], list[float] | list[str]]]
    video_update: tuple[str, dict[str, Any]] | None = None
    depth_image_update: tuple[str, dict[str, Any]] | None = None
    pointcloud_update: tuple[str, dict[str, Any]] | None = None


class EpisodeVideoWriter:
    def __init__(
        self,
        *,
        output_path: Path,
        relative_path: str,
        fps: int,
    ) -> None:
        self.output_path = output_path
        self.relative_path = relative_path
        self.fps = fps
        self.frame_count = 0
        self.shape: list[int] | None = None
        self._writer: Any | None = None

    def write(self, message: Any) -> dict[str, Any]:
        import cv2

        frame_bgr = _image_message_to_bgr(message)
        height, width = frame_bgr.shape[:2]
        if self._writer is None:
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._writer = cv2.VideoWriter(
                str(self.output_path),
                fourcc,
                float(self.fps),
                (width, height),
            )
            if not self._writer.isOpened():
                raise RuntimeError(f"Could not open video writer: {self.output_path}")
            self.shape = [height, width, 3]
        elif self.shape != [height, width, 3]:
            raise ValueError(
                f"Video frame shape changed for {self.output_path}: "
                f"{self.shape} -> {[height, width, 3]}"
            )

        timestamp = self.frame_count / float(self.fps)
        self._writer.write(frame_bgr)
        self.frame_count += 1
        return {"path": self.relative_path, "timestamp": timestamp}

    def close(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self._writer = None


class EpisodeDepthImageWriter:
    def __init__(
        self,
        *,
        format: str,
    ) -> None:
        self.format = format
        self.frame_count = 0
        self.shape: list[int] | None = None

    def write(
        self,
        message: Any,
    ) -> dict[str, Any]:
        if self.format not in {"raw16", "raw"}:
            raise ValueError(f"Unsupported depth image format: {self.format}")

        height = int(message.height)
        width = int(message.width)
        step = int(message.step)
        shape = [height, width, 1]
        if self.shape is None:
            self.shape = shape
        elif self.shape != shape:
            raise ValueError(
                f"Depth frame shape changed: {self.shape} -> {shape}"
            )

        frame_index = self.frame_count
        self.frame_count += 1
        return {
            "data": bytes(message.data),
            "frame_index": frame_index,
            "format": self.format,
            "encoding": str(message.encoding),
            "height": height,
            "width": width,
            "step": step,
            "is_bigendian": bool(message.is_bigendian),
        }


class EpisodePointCloudWriter:
    def __init__(
        self,
        *,
        primary: str,
    ) -> None:
        self.primary = primary
        self.frame_count = 0

    def write(
        self,
        *,
        serialized: Any,
        message: Any,
    ) -> dict[str, Any]:
        frame_index = self.frame_count
        data: bytes | None = None
        if self.primary == "cdr":
            data = _serialized_to_bytes(serialized)
        elif self.primary not in {"none", ""}:
            raise ValueError(f"Unsupported pointcloud primary format: {self.primary}")

        self.frame_count += 1
        return {
            "data": data,
            "frame_index": frame_index,
            "format": self.primary,
            "point_count": int(message.width) * int(message.height),
            "height": int(message.height),
            "width": int(message.width),
            "point_step": int(message.point_step),
            "row_step": int(message.row_step),
            "is_dense": bool(message.is_dense),
        }


class EpisodeConverter:
    def __init__(
        self,
        *,
        bag_path: Path,
        output_dir: Path,
        episode_index: int,
        global_start_index: int,
        storage_id: str | None,
        feature_specs: list[FeatureSpec],
        video_specs: list[VideoSpec],
        depth_image_specs: list[DepthImageSpec],
        pointcloud_specs: list[PointCloudSpec],
        static_topic_specs: list[StaticTopicSpec],
        sampling: SamplingSpec,
        fps: int,
        task_index: int,
        chunk_size: int,
        compression: str | None,
        source_end_ns: int | None = None,
        segment_id: str = "full",
    ) -> None:
        self.bag_path = bag_path
        self.output_dir = output_dir
        self.episode_index = episode_index
        self.global_start_index = global_start_index
        self.storage_id = storage_id
        self.feature_specs = feature_specs
        self.video_specs = video_specs
        self.depth_image_specs = depth_image_specs
        self.pointcloud_specs = pointcloud_specs
        self.static_topic_specs = static_topic_specs
        self.sampling = sampling
        self.fps = fps
        self.task_index = task_index
        self.chunk_size = chunk_size
        self.compression = compression
        self.source_end_ns = source_end_ns
        self.segment_id = segment_id
        self.unmatched_by_stream: dict[str, int] = defaultdict(int)
        self._held_action_values: dict[int, list[float] | list[str]] = {}
        self._engagement_state: dict[int, bool] = {}
        self._engagement_status_at: dict[int, int] = {}
        self._engage_transition_at: dict[int, int] = {}

    def convert(self) -> EpisodeResult:
        reader, topic_types = _open_reader(self.bag_path, self.storage_id)
        _validate_topics(
            topic_types,
            self.feature_specs,
            self.video_specs,
            self.depth_image_specs,
            self.pointcloud_specs,
            self.static_topic_specs,
            self.sampling,
        )

        feature_sources_by_topic: dict[
            str, list[tuple[FeatureSpec, int, SourceSpec]]
        ] = defaultdict(list)
        for feature in self.feature_specs:
            for source_index, source in enumerate(feature.sources):
                feature_sources_by_topic[source.topic].append((feature, source_index, source))

        video_by_topic = {spec.topic: spec for spec in self.video_specs}
        depth_image_by_topic = {spec.topic: spec for spec in self.depth_image_specs}
        pointcloud_by_topic = {spec.topic: spec for spec in self.pointcloud_specs}
        active_video_columns = {
            spec.column
            for spec in self.video_specs
            if spec.topic in topic_types
        }
        active_depth_image_columns = {
            spec.column
            for spec in self.depth_image_specs
            if spec.topic in topic_types
        }
        active_pointcloud_columns = {
            spec.column
            for spec in self.pointcloud_specs
            if spec.topic in topic_types
        }
        static_by_topic = {
            spec.topic: StaticTopicState(
                key=spec.key,
                topic=spec.topic,
                type_name=topic_types.get(spec.topic) or spec.type_name or "unknown",
            )
            for spec in self.static_topic_specs
            if spec.topic in topic_types
        }

        read_topics = (
            set(feature_sources_by_topic)
            | set(video_by_topic)
            | set(depth_image_by_topic)
            | set(pointcloud_by_topic)
            | set(static_by_topic)
        )
        if self.sampling.topic is not None:
            read_topics.add(self.sampling.topic)
        message_classes = {
            topic: _resolve_message_class(topic_types[topic])
            for topic in read_topics
            if topic in topic_types
        }

        message_counts: dict[str, int] = defaultdict(int)
        period_ns = _sampling_period_ns(self.sampling)
        required_timed_topics = {
            source.topic
            for feature in self.feature_specs
            for source in feature.sources
            if source.required
        }
        required_timed_topics.update(
            spec.topic for spec in self.video_specs if spec.required
        )
        required_timed_topics.update(
            spec.topic for spec in self.depth_image_specs if spec.required
        )
        required_timed_topics.update(
            spec.topic for spec in self.pointcloud_specs if spec.required
        )
        if self.sampling.topic is not None:
            required_timed_topics.add(self.sampling.topic)

        events: list[SourceTimedEvent] = []
        source_bounds: dict[str, list[int]] = {}
        last_source_time_by_topic: dict[str, int] = {}

        video_writers = self._create_video_writers()
        depth_image_writers = self._create_depth_image_writers()
        pointcloud_writers = self._create_pointcloud_writers()

        try:
            sequence = 0
            while reader.has_next():
                topic, serialized, bag_time_ns = reader.read_next()
                if topic not in read_topics:
                    continue

                bag_time_ns = int(bag_time_ns)
                message = _deserialize_message(serialized, message_classes[topic])
                message_counts[topic] += 1

                if topic in static_by_topic:
                    static_by_topic[topic].add_message(
                        bag_time_ns,
                        _message_to_metadata_value(topic_types[topic], message),
                    )

                is_timed = (
                    topic in feature_sources_by_topic
                    or topic in video_by_topic
                    or topic in depth_image_by_topic
                    or topic in pointcloud_by_topic
                    or topic == self.sampling.topic
                )
                if not is_timed:
                    continue
                source_time_ns = _message_source_time_ns(message)
                if source_time_ns is None:
                    raise ValueError(
                        f"Source timestamp is missing or zero for {topic}"
                    )
                previous_source_time = last_source_time_by_topic.get(topic)
                if (
                    previous_source_time is not None
                    and source_time_ns <= previous_source_time
                ):
                    raise ValueError(
                        f"Source timestamps are not strictly increasing for {topic}"
                    )
                last_source_time_by_topic[topic] = source_time_ns
                bounds = source_bounds.setdefault(
                    topic, [source_time_ns, source_time_ns]
                )
                bounds[1] = source_time_ns

                # Keep original stream bounds for endpoint validation, but do
                # not encode frames or consume actions beyond this completion.
                if self.source_end_ns is not None and source_time_ns > self.source_end_ns:
                    continue

                feature_updates = []
                for feature, source_index, source in feature_sources_by_topic.get(topic, []):
                    value = _message_to_source_value(
                        topic_types[topic],
                        message,
                        source.field,
                        source.fields,
                        source.joint_names,
                        source.allow_extra_joint_names,
                    )
                    # The validated arm topic may carry only one engaged side.
                    # No value is fabricated for the absent side; its previous
                    # value will become stale and the aligned row is rejected.
                    if value is None:
                        continue
                    feature_updates.append(((feature.column, source_index), value))

                video_update = None
                if topic in video_by_topic:
                    video_spec = video_by_topic[topic]
                    video_update = (
                        video_spec.column,
                        video_writers[video_spec.column].write(message),
                    )

                depth_image_update = None
                if topic in depth_image_by_topic:
                    depth_image_spec = depth_image_by_topic[topic]
                    depth_image_update = (
                        depth_image_spec.column,
                        depth_image_writers[depth_image_spec.column].write(message),
                    )

                pointcloud_update = None
                if topic in pointcloud_by_topic:
                    pointcloud_spec = pointcloud_by_topic[topic]
                    pointcloud_update = (
                        pointcloud_spec.column,
                        pointcloud_writers[pointcloud_spec.column].write(
                            serialized=serialized,
                            message=message,
                        ),
                    )
                events.append(
                    SourceTimedEvent(
                        source_time_ns=source_time_ns,
                        bag_time_ns=bag_time_ns,
                        sequence=sequence,
                        topic=topic,
                        feature_updates=feature_updates,
                        video_update=video_update,
                        depth_image_update=depth_image_update,
                        pointcloud_update=pointcloud_update,
                    )
                )
                sequence += 1
        finally:
            for writer in video_writers.values():
                writer.close()

        missing_source_bounds = required_timed_topics.difference(source_bounds)
        if missing_source_bounds:
            raise ValueError(
                "Required source-time streams have no messages: "
                + ", ".join(sorted(missing_source_bounds))
            )
        required_first_times = [
            source_bounds[topic][0] for topic in required_timed_topics
        ]
        required_last_times = [
            source_bounds[topic][1] for topic in required_timed_topics
        ]
        source_timeline_start_ns = min(required_first_times)
        common_start_ns = max(required_first_times)
        common_end_ns = min(required_last_times)
        if common_end_ns < common_start_ns:
            raise ValueError("Required streams have no common source-time interval")
        effective_start_ns, effective_end_ns = _trim_source_window(
            common_start_ns,
            common_end_ns,
            trim_start_ns=self.sampling.trim_start_ns,
            trim_end_ns=self.sampling.trim_end_ns,
        )
        if self.source_end_ns is not None:
            if not effective_start_ns < self.source_end_ns <= effective_end_ns:
                raise ValueError(
                    f"{self.segment_id}: milestone timestamp {self.source_end_ns} is outside "
                    f"the usable source interval ({effective_start_ns}, {effective_end_ns}]; "
                    "check the marker and ROS/source clock alignment"
                )
            # trim_end belongs to the physical recording stop, not a milestone.
            effective_end_ns = self.source_end_ns
        # LeRobot timestamps are relative to the first usable instant, so the
        # fixed-rate production timeline starts at exactly zero after trimming.
        output_time_origin_ns = effective_start_ns

        latest_source_values: dict[tuple[str, int], list[float] | list[str]] = {}
        latest_source_times_ns: dict[tuple[str, int], int] = {}
        latest_video_values: dict[str, dict[str, Any]] = {}
        latest_video_times_ns: dict[str, int] = {}
        latest_depth_image_values: dict[str, dict[str, Any]] = {}
        latest_depth_image_times_ns: dict[str, int] = {}
        latest_pointcloud_values: dict[str, dict[str, Any]] = {}
        latest_pointcloud_times_ns: dict[str, int] = {}
        rows: list[dict[str, Any]] = []
        rows_skipped_incomplete = 0
        next_sample_ns = (
            _first_grid_time_ns(output_time_origin_ns, effective_start_ns, period_ns)
            if period_ns is not None
            else None
        )

        def emit_row(sample_time_ns: int) -> None:
            nonlocal rows_skipped_incomplete
            row = self._make_row(
                sample_time_ns=sample_time_ns,
                episode_start_ns=output_time_origin_ns,
                rows_so_far=len(rows),
                latest_source_values=latest_source_values,
                latest_source_times_ns=latest_source_times_ns,
                latest_video_values=latest_video_values,
                latest_video_times_ns=latest_video_times_ns,
                latest_depth_image_values=latest_depth_image_values,
                latest_depth_image_times_ns=latest_depth_image_times_ns,
                latest_pointcloud_values=latest_pointcloud_values,
                latest_pointcloud_times_ns=latest_pointcloud_times_ns,
                active_video_columns=active_video_columns,
                active_depth_image_columns=active_depth_image_columns,
                active_pointcloud_columns=active_pointcloud_columns,
            )
            if row is None:
                rows_skipped_incomplete += 1
            else:
                rows.append(row)

        events.sort(key=lambda event: (event.source_time_ns, event.sequence))
        for source_time_ns, grouped_events in groupby(
            events, key=lambda event: event.source_time_ns
        ):
            if source_time_ns > effective_end_ns:
                break
            if next_sample_ns is not None:
                while next_sample_ns < source_time_ns:
                    if next_sample_ns >= effective_start_ns:
                        emit_row(next_sample_ns)
                    next_sample_ns += period_ns

            sample_on_topic = False
            for event in grouped_events:
                for key, value in event.feature_updates:
                    latest_source_values[key] = value
                    latest_source_times_ns[key] = source_time_ns
                if event.video_update is not None:
                    column, value = event.video_update
                    latest_video_values[column] = value
                    latest_video_times_ns[column] = source_time_ns
                if event.depth_image_update is not None:
                    column, value = event.depth_image_update
                    value["timestamp"] = (
                        source_time_ns - output_time_origin_ns
                    ) / 1_000_000_000
                    latest_depth_image_values[column] = value
                    latest_depth_image_times_ns[column] = source_time_ns
                if event.pointcloud_update is not None:
                    column, value = event.pointcloud_update
                    value["timestamp"] = (
                        source_time_ns - output_time_origin_ns
                    ) / 1_000_000_000
                    latest_pointcloud_values[column] = value
                    latest_pointcloud_times_ns[column] = source_time_ns
                sample_on_topic = sample_on_topic or (
                    self.sampling.strategy == "topic"
                    and event.topic == self.sampling.topic
                )

            if source_time_ns < effective_start_ns:
                continue
            if sample_on_topic:
                emit_row(source_time_ns)
            if next_sample_ns is not None:
                while next_sample_ns <= source_time_ns:
                    emit_row(next_sample_ns)
                    next_sample_ns += period_ns

        if next_sample_ns is not None:
            while next_sample_ns <= effective_end_ns:
                emit_row(next_sample_ns)
                next_sample_ns += period_ns

        rows, feature_dims = self._finalize_feature_columns(rows)
        for row_index, row in enumerate(rows):
            row["next.done"] = row_index == len(rows) - 1

        data_path = self._write_episode_parquet(rows, feature_dims)
        stats = _compute_episode_stats(rows, self.feature_specs)
        return EpisodeResult(
            episode_index=self.episode_index,
            bag_path=self.bag_path,
            length=len(rows),
            data_path=data_path,
            video_paths={
                column: writer.relative_path
                for column, writer in video_writers.items()
                if writer.frame_count > 0
            },
            depth_image_storage={
                column: "parquet"
                for column, writer in depth_image_writers.items()
                if writer.frame_count > 0
            },
            pointcloud_storage={
                column: "parquet"
                for column, writer in pointcloud_writers.items()
                if writer.frame_count > 0
            },
            static_topics=[state.to_dict() for state in static_by_topic.values()],
            feature_dims=feature_dims,
            video_shapes={
                column: writer.shape
                for column, writer in video_writers.items()
                if writer.shape is not None
            },
            depth_image_shapes={
                column: writer.shape
                for column, writer in depth_image_writers.items()
                if writer.shape is not None
            },
            message_counts=dict(message_counts),
            stats=stats,
            rows_skipped_incomplete=rows_skipped_incomplete,
            unmatched_by_stream=dict(self.unmatched_by_stream),
            source_timeline_start_ns=source_timeline_start_ns,
            source_common_start_ns=common_start_ns,
            source_common_end_ns=common_end_ns,
            source_effective_start_ns=effective_start_ns,
            source_effective_end_ns=effective_end_ns,
            segment_id=self.segment_id,
            task_index=self.task_index,
            milestone_timestamp_ns=self.source_end_ns,
        )

    def _create_video_writers(self) -> dict[str, EpisodeVideoWriter]:
        writers = {}
        chunk = _chunk_name(self.episode_index, self.chunk_size)
        for spec in self.video_specs:
            relative_path = f"videos/{chunk}/{spec.column}/episode_{self.episode_index:06d}.mp4"
            writers[spec.column] = EpisodeVideoWriter(
                output_path=self.output_dir / relative_path,
                relative_path=relative_path,
                fps=spec.fps or self.fps,
            )
        return writers

    def _create_depth_image_writers(self) -> dict[str, EpisodeDepthImageWriter]:
        writers = {}
        for spec in self.depth_image_specs:
            writers[spec.column] = EpisodeDepthImageWriter(
                format=spec.format,
            )
        return writers

    def _create_pointcloud_writers(self) -> dict[str, EpisodePointCloudWriter]:
        writers = {}
        for spec in self.pointcloud_specs:
            writers[spec.column] = EpisodePointCloudWriter(
                primary=spec.primary,
            )
        return writers

    def _make_row(
        self,
        *,
        sample_time_ns: int,
        episode_start_ns: int,
        rows_so_far: int,
        latest_source_values: Mapping[tuple[str, int], list[float] | list[str]],
        latest_source_times_ns: Mapping[tuple[str, int], int],
        latest_video_values: Mapping[str, dict[str, Any]],
        latest_video_times_ns: Mapping[str, int],
        latest_depth_image_values: Mapping[str, dict[str, Any]],
        latest_depth_image_times_ns: Mapping[str, int],
        latest_pointcloud_values: Mapping[str, dict[str, Any]],
        latest_pointcloud_times_ns: Mapping[str, int],
        active_video_columns: set[str],
        active_depth_image_columns: set[str],
        active_pointcloud_columns: set[str],
    ) -> dict[str, Any] | None:
        latest_source_values, latest_source_times_ns = self._apply_hold_policy(
            sample_time_ns,
            latest_source_values,
            latest_source_times_ns,
        )
        timestamp = (sample_time_ns - episode_start_ns) / 1_000_000_000
        row: dict[str, Any] = {
            "timestamp": timestamp,
            "frame_index": rows_so_far,
            "episode_index": self.episode_index,
            "index": self.global_start_index + rows_so_far,
            "task_index": self.task_index,
            "next.done": False,
            "__feature_sources": {},
        }

        has_feature_value = False
        required_source_missing = False
        for feature in self.feature_specs:
            feature_values: dict[int, Any] = {}
            for source_index, source in enumerate(feature.sources):
                key = (feature.column, source_index)
                source_time_ns = latest_source_times_ns.get(key)
                if source_time_ns is None:
                    self.unmatched_by_stream[
                        f"{feature.column}:{source.topic}:{source_index}"
                    ] += 1
                    required_source_missing = required_source_missing or source.required
                    continue
                if (
                    self.sampling.max_staleness_ns is not None
                    and sample_time_ns - source_time_ns > self.sampling.max_staleness_ns
                ):
                    self.unmatched_by_stream[
                        f"{feature.column}:{source.topic}:{source_index}"
                    ] += 1
                    required_source_missing = required_source_missing or source.required
                    continue
                feature_values[source_index] = latest_source_values[key]
                has_feature_value = True
            row["__feature_sources"][feature.column] = feature_values

        if required_source_missing:
            return None

        for spec in self.video_specs:
            row[spec.column] = self._fresh_media_value(
                spec.column,
                sample_time_ns,
                latest_video_values,
                latest_video_times_ns,
            )
        for spec in self.depth_image_specs:
            row[spec.column] = self._fresh_media_value(
                spec.column,
                sample_time_ns,
                latest_depth_image_values,
                latest_depth_image_times_ns,
            )
        for spec in self.pointcloud_specs:
            row[spec.column] = self._fresh_media_value(
                spec.column,
                sample_time_ns,
                latest_pointcloud_values,
                latest_pointcloud_times_ns,
            )

        if any(row.get(column) is None for column in active_video_columns):
            return None
        if any(row.get(column) is None for column in active_depth_image_columns):
            return None
        if any(row.get(column) is None for column in active_pointcloud_columns):
            return None

        if (
            not has_feature_value
            and not any(row.get(spec.column) for spec in self.video_specs)
            and not any(row.get(spec.column) for spec in self.depth_image_specs)
            and not any(row.get(spec.column) for spec in self.pointcloud_specs)
        ):
            return None
        return row

    def _apply_hold_policy(
        self,
        sample_time_ns: int,
        latest_values: Mapping[tuple[str, int], list[float] | list[str]],
        latest_times_ns: Mapping[tuple[str, int], int],
    ) -> tuple[
        dict[tuple[str, int], list[float] | list[str]],
        dict[tuple[str, int], int],
    ]:
        """Materialize an effective action for explicitly disengaged sides.

        The control path remains untouched: disengage still stops hardware
        commands.  For the offline dataset, each inactive source holds its last
        validated/sent target.  A side that starts inactive is seeded from the
        corresponding measured position, never from zeros or the other side.
        """
        values = dict(latest_values)
        times = dict(latest_times_ns)
        action_feature = next(
            (item for item in self.feature_specs if item.column == "action"),
            None,
        )
        engaged_feature = next(
            (
                item
                for item in self.feature_specs
                if item.column == "observation.engaged"
            ),
            None,
        )
        state_feature = next(
            (
                item
                for item in self.feature_specs
                if item.column == "observation.state"
            ),
            None,
        )
        if (
            action_feature is None
            or engaged_feature is None
            or state_feature is None
            or len(action_feature.sources) != 4
            or len(engaged_feature.sources) != 4
            or len(state_feature.sources) != 4
        ):
            return values, times

        position_dims = (7, 7, 20, 20)
        for source_index, position_dim in enumerate(position_dims):
            engaged_key = (engaged_feature.column, source_index)
            action_key = (action_feature.column, source_index)
            engaged_at = times.get(engaged_key)
            engaged_value = values.get(engaged_key)
            engagement_is_fresh = (
                engaged_at is not None
                and engaged_value is not None
                and (
                    self.sampling.max_staleness_ns is None
                    or sample_time_ns - engaged_at
                    <= self.sampling.max_staleness_ns
                )
            )
            if not engagement_is_fresh:
                continue

            active = bool(float(engaged_value[0]))
            if engaged_at != self._engagement_status_at.get(source_index):
                previous = self._engagement_state.get(source_index)
                self._engagement_status_at[source_index] = engaged_at
                self._engagement_state[source_index] = active
                if active and previous is not True:
                    self._engage_transition_at[source_index] = engaged_at

            if active:
                transition_at = self._engage_transition_at.get(source_index)
                action_at = times.get(action_key)
                action_matches_transition = (
                    transition_at is None
                    or (
                        action_at is not None
                        and action_at + ENGAGEMENT_ACTION_SKEW_NS >= transition_at
                    )
                )
                if not action_matches_transition:
                    held = self._held_action_values.get(source_index)
                    if held is not None:
                        values[action_key] = held
                        times[action_key] = sample_time_ns
                        # Export the effective, causally aligned state. The raw
                        # status transition remains preserved in the source bag.
                        values[engaged_key] = [0.0]
                    continue
                self._engage_transition_at.pop(source_index, None)
                self._held_action_values.pop(source_index, None)
                continue

            self._engage_transition_at.pop(source_index, None)
            if source_index not in self._held_action_values:
                hold_value = values.get(action_key)
                if hold_value is None:
                    state_value = values.get((state_feature.column, source_index))
                    if state_value is not None and len(state_value) >= position_dim:
                        hold_value = list(state_value[:position_dim])
                if hold_value is not None:
                    self._held_action_values[source_index] = list(hold_value)

            held = self._held_action_values.get(source_index)
            if held is not None:
                values[action_key] = held
                times[action_key] = sample_time_ns
        return values, times

    def _fresh_media_value(
        self,
        column: str,
        sample_time_ns: int,
        latest_values: Mapping[str, dict[str, Any]],
        latest_times_ns: Mapping[str, int],
    ) -> dict[str, Any] | None:
        source_time_ns = latest_times_ns.get(column)
        if source_time_ns is None:
            self.unmatched_by_stream[column] += 1
            return None
        if (
            self.sampling.max_staleness_ns is not None
            and sample_time_ns - source_time_ns > self.sampling.max_staleness_ns
        ):
            self.unmatched_by_stream[column] += 1
            return None
        return latest_values.get(column)

    def _finalize_feature_columns(
        self,
        rows: list[dict[str, Any]],
    ) -> tuple[list[dict[str, Any]], dict[str, int]]:
        source_dims: dict[tuple[str, int], int] = {}
        for row in rows:
            feature_sources = row["__feature_sources"]
            for feature in self.feature_specs:
                for source_index in range(len(feature.sources)):
                    value = feature_sources.get(feature.column, {}).get(source_index)
                    if value is not None:
                        source_dims[(feature.column, source_index)] = max(
                            source_dims.get((feature.column, source_index), 0),
                            len(value),
                        )

        feature_dims: dict[str, int] = {}
        for row in rows:
            feature_sources = row.pop("__feature_sources")
            for feature in self.feature_specs:
                values: list[Any] = []
                for source_index in range(len(feature.sources)):
                    dim = source_dims.get((feature.column, source_index), 0)
                    value = feature_sources.get(feature.column, {}).get(source_index)
                    if value is None:
                        values.extend([None] * dim)
                    else:
                        values.extend(value)
                row[feature.column] = values
                feature_dims[feature.column] = len(values)
        return rows, feature_dims

    def _write_episode_parquet(
        self,
        rows: list[dict[str, Any]],
        feature_dims: Mapping[str, int],
    ) -> str:
        import pyarrow as pa
        import pyarrow.parquet as pq

        chunk = _chunk_name(self.episode_index, self.chunk_size)
        relative_path = f"data/{chunk}/episode_{self.episode_index:06d}.parquet"
        output_path = self.output_dir / relative_path
        output_path.parent.mkdir(parents=True, exist_ok=True)
        table = pa.Table.from_pylist(
            rows,
            schema=_episode_schema(
                pa,
                self.feature_specs,
                self.video_specs,
                self.depth_image_specs,
                self.pointcloud_specs,
            ),
        )
        pq.write_table(table, output_path, compression=self.compression)
        return relative_path


class LeRobotDatasetConverter:
    def __init__(
        self,
        *,
        bag_paths: list[Path],
        output_dir: Path,
        storage_id: str | None,
        feature_specs: list[FeatureSpec],
        video_specs: list[VideoSpec],
        depth_image_specs: list[DepthImageSpec],
        pointcloud_specs: list[PointCloudSpec],
        static_topic_specs: list[StaticTopicSpec],
        sampling: SamplingSpec,
        fps: int,
        task: str,
        robot_type: str,
        chunk_size: int,
        compression: str | None,
        teleoperator: str,
        source_provenance: list[dict[str, Any]],
        segments: str = "all",
        segment_tasks: Mapping[str, str] | None = None,
    ) -> None:
        self.bag_paths = bag_paths
        self.output_dir = output_dir
        self.storage_id = storage_id
        self.feature_specs = feature_specs
        self.video_specs = video_specs
        self.depth_image_specs = depth_image_specs
        self.pointcloud_specs = pointcloud_specs
        self.static_topic_specs = static_topic_specs
        self.sampling = sampling
        self.fps = fps
        self.task = task
        self.robot_type = robot_type
        self.chunk_size = chunk_size
        self.compression = compression
        self.teleoperator = teleoperator
        self.source_provenance = source_provenance
        self.segments = segments
        self.segment_tasks = dict(segment_tasks or {})
        if any(not isinstance(value, str) or not value.strip() for value in self.segment_tasks.values()):
            raise ValueError("segment_tasks must map segment IDs to non-empty task descriptions")
        self.tasks: list[str] = []

    def convert(self) -> dict[str, Any]:
        results: list[EpisodeResult] = []
        global_index = 0
        self.tasks = []
        for bag_path, provenance in zip(self.bag_paths, self.source_provenance, strict=True):
            for segment_id, end_ns in _episode_segments(provenance, self.segments):
                task = self.segment_tasks.get(
                    segment_id, self.task if segment_id == "full" else segment_id
                )
                if task not in self.tasks:
                    self.tasks.append(task)
                converter = EpisodeConverter(
                    bag_path=bag_path,
                    output_dir=self.output_dir,
                    episode_index=len(results),
                    global_start_index=global_index,
                    storage_id=self.storage_id,
                    feature_specs=self.feature_specs,
                    video_specs=self.video_specs,
                    depth_image_specs=self.depth_image_specs,
                    pointcloud_specs=self.pointcloud_specs,
                    static_topic_specs=self.static_topic_specs,
                    sampling=self.sampling,
                    fps=self.fps,
                    task_index=self.tasks.index(task),
                    chunk_size=self.chunk_size,
                    compression=self.compression,
                    source_end_ns=end_ns,
                    segment_id=segment_id,
                )
                result = converter.convert()
                result.source_recording_id = str(
                    provenance.get("source_recording_id") or provenance["source_bag"]
                )
                results.append(result)
                global_index += result.length

        manifest = self._write_lerobot_metadata(results)
        _write_dataset_readme(self.output_dir, manifest)
        _write_dataset_dataloader(self.output_dir)
        return manifest

    def _write_lerobot_metadata(self, results: list[EpisodeResult]) -> dict[str, Any]:
        provenance_by_bag = dict(zip(self.bag_paths, self.source_provenance, strict=True))
        total_frames = sum(result.length for result in results)
        total_videos = sum(len(result.video_paths) for result in results)
        total_depth_images = sum(len(result.depth_image_storage) for result in results)
        total_pointclouds = sum(len(result.pointcloud_storage) for result in results)
        features = _lerobot_features(
            self.feature_specs,
            self.video_specs,
            self.depth_image_specs,
            self.pointcloud_specs,
            results,
            self.fps,
        )

        meta_dir = self.output_dir / "meta"
        meta_dir.mkdir(parents=True, exist_ok=True)

        info = {
            "codebase_version": LEROBOT_CODEBASE_VERSION,
            "robot_type": self.robot_type,
            "fps": self.fps,
            "total_episodes": len(results),
            "total_frames": total_frames,
            "total_tasks": len(self.tasks),
            "total_videos": total_videos,
            "total_depth_image_streams": total_depth_images,
            "total_pointcloud_streams": total_pointclouds,
            "splits": {"train": f"0:{len(results)}"},
            "features": features,
            "teleoperator": self.teleoperator,
        }
        (meta_dir / "info.json").write_text(
            json.dumps(info, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        _write_jsonl(
            meta_dir / "tasks.jsonl",
            [{"task_index": index, "task": task} for index, task in enumerate(self.tasks)],
        )
        _write_jsonl(
            meta_dir / "episodes.jsonl",
            [
                {
                    "episode_index": result.episode_index,
                    "tasks": [self.tasks[result.task_index]],
                    "length": result.length,
                    "segment_id": result.segment_id,
                    "source_recording_id": result.source_recording_id,
                }
                for result in results
            ],
        )
        _write_jsonl(
            meta_dir / "episodes_stats.jsonl",
            [
                {
                    "episode_index": result.episode_index,
                    "stats": result.stats,
                }
                for result in results
            ],
        )

        metadata = {
            "format": FORMAT_VERSION,
            "source_bags": [str(path) for path in self.bag_paths],
            "segments": self.segments,
            "teleoperator": self.teleoperator,
            "source_provenance": [provenance_by_bag[result.bag_path] for result in results],
            "sampling": {
                "strategy": self.sampling.strategy,
                "topic": self.sampling.topic,
                "frequency_hz": self.sampling.frequency_hz,
                "max_staleness_ns": self.sampling.max_staleness_ns,
                "trim_start_ns": self.sampling.trim_start_ns,
                "trim_end_ns": self.sampling.trim_end_ns,
                "alignment_clock": "source_header",
            },
            "action_hold_policy": {
                "mode": "offline_latched_target",
                "engagement_feature": "observation.engaged",
                "source_order": [
                    "left_arm",
                    "right_arm",
                    "left_hand",
                    "right_hand",
                ],
                "initial_hold_seed": "measured_position",
                "hardware_command_path_modified": False,
            },
            "timing_summary": {
                "total_rows_skipped_incomplete": sum(
                    result.rows_skipped_incomplete for result in results
                ),
                "episodes": [
                    {
                        "episode_index": result.episode_index,
                        "rows_skipped_incomplete": result.rows_skipped_incomplete,
                        "message_counts": result.message_counts,
                        "unmatched_by_stream": result.unmatched_by_stream,
                        "source_timeline_start_ns": result.source_timeline_start_ns,
                        "source_common_start_ns": result.source_common_start_ns,
                        "source_common_end_ns": result.source_common_end_ns,
                        "source_effective_start_ns": result.source_effective_start_ns,
                        "source_effective_end_ns": result.source_effective_end_ns,
                    }
                    for result in results
                ],
            },
            "episodes": [
                {
                    "episode_index": result.episode_index,
                    "bag_path": str(result.bag_path),
                    "source_recording_id": result.source_recording_id,
                    "segment_id": result.segment_id,
                    "task_index": result.task_index,
                    "milestone_timestamp_ns": result.milestone_timestamp_ns,
                    "data_path": result.data_path,
                    "video_paths": result.video_paths,
                    "depth_image_storage": result.depth_image_storage,
                    "pointcloud_storage": result.pointcloud_storage,
                    "length": result.length,
                    "rows_skipped_incomplete": result.rows_skipped_incomplete,
                    "message_counts": result.message_counts,
                    "unmatched_by_stream": result.unmatched_by_stream,
                    "source_timeline_start_ns": result.source_timeline_start_ns,
                    "source_common_start_ns": result.source_common_start_ns,
                    "source_common_end_ns": result.source_common_end_ns,
                    "source_effective_start_ns": result.source_effective_start_ns,
                    "source_effective_end_ns": result.source_effective_end_ns,
                    "static_topics": result.static_topics,
                }
                for result in results
            ],
        }
        (meta_dir / "conversion_metadata.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return {
            **metadata,
            "info": info,
        }


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(argv)
    bag_paths = _discover_bag_paths(args.bags)
    if not bag_paths:
        raise SystemExit("No finalized rosbag directories found.")

    output_dir = (args.output or bag_paths[0].with_name("lerobot_dataset")).expanduser().resolve()
    if any(output_dir == bag_path for bag_path in bag_paths):
        raise SystemExit("--output must not be one of the source bag directories.")
    if output_dir.exists():
        if not args.overwrite:
            raise SystemExit(
                f"Output already exists: {output_dir}. Pass --overwrite to replace it."
            )
        if not _safe_to_replace_output(output_dir):
            raise SystemExit(
                f"Refusing to overwrite non-converter output directory: {output_dir}"
            )

    config = _load_conversion_config(args.config)
    teleoperator = str(config.get("teleoperator") or "").strip()
    if teleoperator not in DATA_TELEOPERATORS:
        raise SystemExit(
            f"Conversion config must select teleoperator {DATA_TELEOPERATORS}."
        )
    source_provenance = [
        _load_bag_provenance(path, teleoperator) for path in bag_paths
    ]
    feature_specs = _feature_specs_from_config(config)
    raw_video_specs = _video_specs_from_config(config)
    depth_image_specs = _depth_image_specs_from_config(config, raw_video_specs)
    video_specs = [spec for spec in raw_video_specs if not _looks_like_depth_stream(spec)]
    pointcloud_specs = _pointcloud_specs_from_config(config)
    static_topic_specs = _static_specs_from_config(config)
    fps = int(args.fps or config.get("fps") or DEFAULT_FPS)
    if fps <= 0:
        raise SystemExit("--fps/config fps must be positive.")
    sampling = _sampling_from_args_and_config(
        args,
        config,
        fps,
        feature_specs,
        video_specs,
        depth_image_specs,
        pointcloud_specs,
    )
    task = str(args.task or config.get("task") or DEFAULT_TASK)
    robot_type = str(
        args.robot_type or config.get("robot_type") or "dual_fr3_wuji"
    )

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    temporary_output = Path(
        tempfile.mkdtemp(prefix=f".{output_dir.name}.tmp-", dir=output_dir.parent)
    )
    # mkdtemp creates mode 0700. Conversion normally runs as root in Docker,
    # so make the atomically published dataset readable by the host data user.
    temporary_output.chmod(0o755)
    backup_output = output_dir.with_name(f".{output_dir.name}.backup-{uuid.uuid4().hex}")
    source_snapshot = _source_tree_snapshot(bag_paths)
    manifest = None
    moved_existing = False
    try:
        converter = LeRobotDatasetConverter(
            bag_paths=bag_paths,
            output_dir=temporary_output,
            storage_id=args.storage_id,
            feature_specs=feature_specs,
            video_specs=video_specs,
            depth_image_specs=depth_image_specs,
            pointcloud_specs=pointcloud_specs,
            static_topic_specs=static_topic_specs,
            sampling=sampling,
            fps=fps,
            task=task,
            robot_type=robot_type,
            chunk_size=args.chunk_size,
            compression=args.compression,
            teleoperator=teleoperator,
            source_provenance=source_provenance,
            segments=args.segments or config.get("segments", "all"),
            segment_tasks=config.get("segment_tasks"),
        )
        manifest = converter.convert()
        _validate_lerobot_v2_output(temporary_output, manifest)
        if _source_tree_snapshot(bag_paths) != source_snapshot:
            raise RuntimeError("source ROS bag changed during conversion")
        if output_dir.exists():
            os.replace(output_dir, backup_output)
            moved_existing = True
        os.replace(temporary_output, output_dir)
        if moved_existing:
            shutil.rmtree(backup_output)
    except Exception:
        if moved_existing and backup_output.exists() and not output_dir.exists():
            os.replace(backup_output, output_dir)
        raise
    finally:
        if temporary_output.exists():
            shutil.rmtree(temporary_output)
    assert manifest is not None
    print(f"Wrote LeRobot dataset to {output_dir}")
    print(f"  episodes: {manifest['info']['total_episodes']}")
    print(f"  frames: {manifest['info']['total_frames']}")
    print(f"  videos: {manifest['info']['total_videos']}")
    print(f"  depth image streams: {manifest['info']['total_depth_image_streams']}")
    print(f"  pointcloud streams: {manifest['info']['total_pointcloud_streams']}")


def _episode_segments(
    state: Mapping[str, Any], mode: str,
) -> list[tuple[str, int | None]]:
    if mode not in {"all", "full", "milestones"}:
        raise ValueError("segments must be all, full, or milestones")
    if mode == "full":
        return [("full", None)]
    markers = state.get("milestones", [])
    if not isinstance(markers, list):
        raise ValueError("milestones must be a list")
    segments: list[tuple[str, int | None]] = []
    previous_ns = 0
    seen = {"full"}
    for marker in markers:
        if not isinstance(marker, dict):
            raise ValueError("each milestone must be an object")
        marker_id = marker.get("id")
        timestamp_ns = marker.get("timestamp_ns")
        if not isinstance(marker_id, str) or not marker_id.strip() or marker_id in seen:
            raise ValueError("milestone IDs must be non-empty, unique, and not 'full'")
        if marker.get("clock") != "ros":
            raise ValueError(f"{marker_id}: milestone clock must be ros")
        if type(timestamp_ns) is not int or timestamp_ns <= previous_ns:
            raise ValueError("milestone timestamps must be positive, increasing integer nanoseconds")
        segments.append((marker_id, timestamp_ns))
        seen.add(marker_id)
        previous_ns = timestamp_ns
    if mode == "all":
        segments.append(("full", None))
    elif not segments:
        raise ValueError("milestones export requested but this bag has no milestones")
    return segments


def _load_bag_provenance(bag_path: Path, teleoperator: str) -> dict[str, Any]:
    state_path = bag_path / "collection_state.json"
    if not state_path.exists():
        raise SystemExit(f"Bag is missing collection_state.json: {bag_path}")
    try:
        state = json.loads(state_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Invalid collection state for {bag_path}: {exc}") from exc
    if state.get("finalized") is not True or state.get("state") != "finalized":
        raise SystemExit(f"Bag is not finalized and cannot be converted: {bag_path}")
    if state.get("teleoperator") != teleoperator:
        raise SystemExit(
            f"Bag teleoperator {state.get('teleoperator')!r} does not match {teleoperator!r}: "
            f"{bag_path}"
        )
    required = (
        "bag_contract_version",
        "source_bag",
        "workcell_id",
        "workcell_config_hash",
        "control_config_id",
        "calibration_ids",
        "device_identities",
        "source_topic_contract",
        "timestamp_policy",
    )
    missing = [name for name in required if state.get(name) in (None, "", [], {})]
    if missing:
        raise SystemExit(f"Bag provenance is incomplete ({', '.join(missing)}): {bag_path}")
    return state


def _source_tree_snapshot(bag_paths: list[Path]) -> tuple[tuple[str, int, int], ...]:
    snapshot = []
    for bag_path in bag_paths:
        for path in sorted(item for item in bag_path.rglob("*") if item.is_file()):
            stat = path.stat()
            snapshot.append((str(path.resolve()), stat.st_size, stat.st_mtime_ns))
    return tuple(snapshot)


def _validate_lerobot_v2_output(
    output_dir: Path, manifest: Mapping[str, Any]
) -> None:
    info = manifest.get("info")
    if not isinstance(info, Mapping) or info.get("codebase_version") != "v2.0":
        raise ValueError("LeRobot output codebase_version must be v2.0")
    features = info.get("features")
    if not isinstance(features, Mapping):
        raise ValueError("LeRobot output features are missing")
    expected_shapes = {
        "action": [54],
        "observation.state": [108],
        "observation.engaged": [4],
    }
    for name, shape in expected_shapes.items():
        feature = features.get(name)
        if not isinstance(feature, Mapping) or feature.get("shape") != shape:
            raise ValueError(f"LeRobot feature {name} must have shape {shape}")
    arm_names = [*DATA_LEFT_ARM_JOINT_NAMES, *DATA_RIGHT_ARM_JOINT_NAMES]
    hand_names = [*WUJI_LEFT_JOINT_NAMES, *WUJI_RIGHT_JOINT_NAMES]
    expected_feature_names = {
        "action": [*arm_names, *hand_names],
        "observation.state": [
            *(f"{name}.position" for name in DATA_LEFT_ARM_JOINT_NAMES),
            *(f"{name}.velocity" for name in DATA_LEFT_ARM_JOINT_NAMES),
            *(f"{name}.position" for name in DATA_RIGHT_ARM_JOINT_NAMES),
            *(f"{name}.velocity" for name in DATA_RIGHT_ARM_JOINT_NAMES),
            *(f"{name}.position" for name in WUJI_LEFT_JOINT_NAMES),
            *(f"{name}.velocity" for name in WUJI_LEFT_JOINT_NAMES),
            *(f"{name}.position" for name in WUJI_RIGHT_JOINT_NAMES),
            *(f"{name}.velocity" for name in WUJI_RIGHT_JOINT_NAMES),
        ],
        "observation.engaged": [
            "left_arm",
            "right_arm",
            "left_hand",
            "right_hand",
        ],
    }
    for name, expected_names in expected_feature_names.items():
        if features[name].get("names") != expected_names:
            raise ValueError(f"LeRobot feature {name} has the wrong joint order")
    for camera in ("cam0", "cam1", "cam2"):
        if f"observation.images.{camera}" not in features:
            raise ValueError(f"LeRobot output is missing camera {camera}")
    depth_feature = features.get("observation.depths.cam0")
    if not isinstance(depth_feature, Mapping):
        raise ValueError("LeRobot output is missing raw head-camera depth")
    depth_info = depth_feature.get("info")
    if (
        depth_feature.get("dtype") != "image"
        or not isinstance(depth_info, Mapping)
        or depth_info.get("image.format") != "raw16"
        or depth_info.get("image.is_depth_map") is not True
    ):
        raise ValueError("LeRobot head-camera depth must be stored as a raw depth image")
    if int(info.get("total_frames", 0)) <= 0:
        raise ValueError("LeRobot output contains no frames")
    if info.get("teleoperator") not in DATA_TELEOPERATORS:
        raise ValueError("LeRobot output teleoperator is invalid")
    provenance = manifest.get("source_provenance")
    if not isinstance(provenance, list) or len(provenance) != int(
        info.get("total_episodes", 0)
    ):
        raise ValueError("LeRobot output source provenance is incomplete")
    timing_summary = manifest.get("timing_summary")
    if not isinstance(timing_summary, Mapping) or not isinstance(
        timing_summary.get("episodes"), list
    ):
        raise ValueError("LeRobot output timing summary is missing")
    for episode in manifest.get("episodes", []):
        video_paths = episode.get("video_paths", {})
        expected_cameras = {
            "observation.images.cam0",
            "observation.images.cam1",
            "observation.images.cam2",
        }
        if set(video_paths) != expected_cameras:
            raise ValueError("LeRobot output episode does not contain all three camera videos")
        for relative_path in video_paths.values():
            video_path = output_dir / str(relative_path)
            if not video_path.is_file() or video_path.stat().st_size == 0:
                raise ValueError(f"LeRobot video is missing or empty: {relative_path}")
        if set(episode.get("depth_image_storage", {})) != {
            "observation.depths.cam0"
        }:
            raise ValueError("LeRobot output episode does not contain raw head-camera depth")

    import numpy as np
    import pyarrow.parquet as pq

    parquet_paths = sorted((output_dir / "data").rglob("*.parquet"))
    if len(parquet_paths) != int(info.get("total_episodes", 0)):
        raise ValueError("LeRobot output parquet episode count is inconsistent")
    for parquet_path in parquet_paths:
        table = pq.read_table(
            parquet_path,
            columns=[
                "timestamp",
                "action",
                "observation.state",
                "observation.engaged",
            ],
        )
        timestamps = np.asarray(table["timestamp"].to_pylist(), dtype=float)
        if not np.isfinite(timestamps).all() or np.any(np.diff(timestamps) <= 0):
            raise ValueError(f"non-finite or non-monotonic timestamps in {parquet_path}")
        for column, dimension in (
            ("action", 54),
            ("observation.state", 108),
            ("observation.engaged", 4),
        ):
            values = np.asarray(table[column].to_pylist(), dtype=float)
            if values.ndim != 2 or values.shape[1] != dimension:
                raise ValueError(f"{column} does not have dimension {dimension}")
            if not np.isfinite(values).all():
                raise ValueError(f"{column} contains non-finite values")
            if column == "observation.engaged" and not np.isin(
                values, [0.0, 1.0]
            ).all():
                raise ValueError("observation.engaged must contain only 0 or 1")


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert finalized ROS 2 bag episodes into a LeRobot-style dataset.",
    )
    parser.add_argument(
        "bags",
        nargs="+",
        type=Path,
        help="Rosbag directories, or parent directories containing episode bag directories.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Output LeRobot dataset directory. Defaults to lerobot_dataset next to the first bag.",
    )
    parser.add_argument(
        "-c",
        "--config",
        type=Path,
        help="YAML/JSON topic-to-feature mapping config.",
    )
    parser.add_argument(
        "--storage-id",
        help="rosbag2 storage id. Defaults to metadata.yaml or sqlite3.",
    )
    parser.add_argument(
        "--fps",
        type=int,
        help="Dataset/control timeline fps. Video topics can override fps in config.",
    )
    parser.add_argument("--sample-topic", help="Emit one row whenever this topic has a message.")
    parser.add_argument("--sample-hz", type=float, help="Emit fixed-rate rows using latest values.")
    parser.add_argument(
        "--max-staleness-ms",
        type=float,
        help="Omit a source value when its latest message is older than this limit.",
    )
    parser.add_argument(
        "--trim-start-sec",
        type=float,
        help="Exclude this many seconds from the start of the common source interval.",
    )
    parser.add_argument(
        "--trim-end-sec",
        type=float,
        help="Exclude this many seconds from the end of the common source interval.",
    )
    parser.add_argument("--task", help="Task label stored in meta/tasks.jsonl.")
    parser.add_argument(
        "--segments", choices=("all", "full", "milestones"),
        help="Export milestone prefixes and/or the full episode (default: all).",
    )
    parser.add_argument("--robot-type", help="Robot type stored in meta/info.json.")
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=DEFAULT_CHUNK_SIZE,
        help="Episodes per chunk directory.",
    )
    parser.add_argument(
        "--compression",
        default="zstd",
        help="Parquet compression codec passed to pyarrow. Use 'none' to disable.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace existing converter output.",
    )
    args = parser.parse_args(argv)
    if args.chunk_size <= 0:
        parser.error("--chunk-size must be positive")
    if args.sample_hz is not None and args.sample_hz <= 0:
        parser.error("--sample-hz must be positive")
    if args.sample_topic and args.sample_hz is not None:
        parser.error("--sample-topic and --sample-hz are mutually exclusive")
    if args.max_staleness_ms is not None and args.max_staleness_ms <= 0:
        parser.error("--max-staleness-ms must be positive")
    if args.trim_start_sec is not None and args.trim_start_sec < 0:
        parser.error("--trim-start-sec must be non-negative")
    if args.trim_end_sec is not None and args.trim_end_sec < 0:
        parser.error("--trim-end-sec must be non-negative")
    if args.compression.lower() == "none":
        args.compression = None
    return args


def _discover_bag_paths(inputs: Iterable[Path]) -> list[Path]:
    bag_paths: list[Path] = []
    for raw_path in inputs:
        path = raw_path.expanduser().resolve()
        if not path.exists():
            raise SystemExit(f"Bag path does not exist: {path}")
        if (path / "metadata.yaml").exists():
            bag_paths.append(path)
            continue
        children = sorted(child for child in path.iterdir() if (child / "metadata.yaml").exists())
        if not children:
            raise SystemExit(f"No rosbag metadata.yaml found in {path}")
        bag_paths.extend(children)
    return list(dict.fromkeys(bag_paths))


def _open_reader(bag_path: Path, storage_id: str | None) -> tuple[Any, dict[str, str]]:
    import rosbag2_py

    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id=storage_id or _metadata_storage_id(bag_path) or "sqlite3",
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    return reader, topics


def _metadata_storage_id(bag_path: Path) -> str | None:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.exists():
        return None
    try:
        import yaml

        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
        bag_info = metadata.get("rosbag2_bagfile_information", {})
        storage_id = bag_info.get("storage_identifier")
        return str(storage_id) if storage_id else None
    except Exception:
        match = re.search(
            r"^\s*storage_identifier:\s*['\"]?([^'\"\s]+)",
            metadata_path.read_text(encoding="utf-8"),
            flags=re.MULTILINE,
        )
        return match.group(1) if match else None


def _safe_to_replace_output(output_dir: Path) -> bool:
    if not any(output_dir.iterdir()):
        return True
    metadata_path = output_dir / "meta" / "conversion_metadata.json"
    info_path = output_dir / "meta" / "info.json"
    if not metadata_path.exists() and not info_path.exists():
        return False
    if metadata_path.exists():
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            return False
        return str(metadata.get("format", "")).startswith(
            "teleop.harvest.lerobot."
        )
    return True


def _load_conversion_config(config_path: Path | None) -> dict[str, Any]:
    if config_path is None:
        return {}
    path = config_path.expanduser()
    if not path.exists():
        raise SystemExit(f"Conversion config does not exist: {path}")
    if path.suffix.lower() == ".json":
        config = json.loads(path.read_text(encoding="utf-8"))
    else:
        import yaml

        config = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    teleoperator = config.get("teleoperator")
    if teleoperator is not None:
        if config.get("contract_version") != 1:
            raise ValueError("conversion contract_version must equal 1")
        if config.get("lerobot_version") != "v2.0":
            raise ValueError("conversion lerobot_version must equal v2.0")
        expanded = _mode_conversion_config(str(teleoperator))
        expanded.update(config)
        config = expanded
    return config


def _mode_conversion_config(teleoperator: str) -> dict[str, Any]:
    if teleoperator not in DATA_TELEOPERATORS:
        raise ValueError(
            f"teleoperator must be one of {DATA_TELEOPERATORS}"
        )

    arm_names = {
        "left": list(DATA_LEFT_ARM_JOINT_NAMES),
        "right": list(DATA_RIGHT_ARM_JOINT_NAMES),
    }
    command_names = {
        "left": list(LEFT_COMMAND_JOINT_NAMES),
        "right": list(RIGHT_COMMAND_JOINT_NAMES),
    }
    hand_names = {
        "left": list(WUJI_LEFT_JOINT_NAMES),
        "right": list(WUJI_RIGHT_JOINT_NAMES),
    }

    action_sources = [
        {
            "topic": VALIDATED_COMMAND_TOPIC,
            "type": "sensor_msgs/msg/JointState",
            "field": "position",
            "joint_names": command_names[side],
            "names": arm_names[side],
            "allow_extra_joint_names": True,
        }
        for side in ("left", "right")
    ]
    action_sources.extend(
        {
            "topic": WUJI_COMMAND_TOPIC.format(side=side),
            "type": "sensor_msgs/msg/JointState",
            "field": "position",
            "joint_names": hand_names[side],
            "names": hand_names[side],
        }
        for side in ("left", "right")
    )

    observation_sources = []
    for side in ("left", "right"):
        names = arm_names[side]
        observation_sources.append(
            {
                "topic": f"/{side}/franka/joint_states",
                "type": "sensor_msgs/msg/JointState",
                "fields": ["position", "velocity"],
                "joint_names": names,
                "names": [
                    *(f"{name}.position" for name in names),
                    *(f"{name}.velocity" for name in names),
                ],
            }
        )
    for side in ("left", "right"):
        names = hand_names[side]
        observation_sources.append(
            {
                "topic": WUJI_STATE_TOPIC.format(side=side),
                "type": "sensor_msgs/msg/JointState",
                "fields": ["position", "velocity"],
                "joint_names": names,
                "names": [
                    *(f"{name}.position" for name in names),
                    *(f"{name}.velocity" for name in names),
                ],
            }
        )
    engagement_sources = [
        {
            "topic": COMMAND_STATUS_TOPIC,
            "type": "teleop_interfaces/msg/ArmCommandStatus",
            "field": f"accepted_sides.{side}",
            "names": [f"{side}_arm"],
        }
        for side in ("left", "right")
    ]
    engagement_sources.extend(
        {
            "topic": WUJI_TELEMETRY_STATUS_TOPIC,
            "type": "teleop_interfaces/msg/HandTelemetryStatus",
            "field": f"engaged.{side}",
            "names": [f"{side}_hand"],
        }
        for side in ("left", "right")
    )
    return {
        "features": {
            "action": {"dtype": "float32", "sources": action_sources},
            "observation.state": {
                "dtype": "float32",
                "sources": observation_sources,
            },
            "observation.engaged": {
                "dtype": "float32",
                "sources": engagement_sources,
            },
        },
        "videos": {
            f"observation.images.cam{index}": {
                "topic": CAMERA_COLOR_TOPIC.format(index=index),
                "type": "sensor_msgs/msg/Image",
                "fps": 20 if index == 0 else 30,
            }
            for index in range(3)
        },
        "depth_images": {
            "observation.depths.cam0": {
                "topic": "/cam0/depth/image_raw",
                "type": "sensor_msgs/msg/Image",
                "format": "raw16",
                "fps": 20,
            }
        },
    }


def _feature_specs_from_config(config: Mapping[str, Any]) -> list[FeatureSpec]:
    raw_features = config.get("features")
    if raw_features is None:
        raw_features = config.get("columns")
    if not raw_features:
        return []
    if not isinstance(raw_features, Mapping):
        raise ValueError("features/columns must be a mapping.")

    specs: list[FeatureSpec] = []
    for column, raw_feature in raw_features.items():
        if isinstance(raw_feature, str):
            raw_feature = {"sources": [{"topic": raw_feature}]}
        if not isinstance(raw_feature, Mapping):
            raise ValueError(f"Feature mapping for {column!r} must be an object.")
        raw_sources = raw_feature.get("sources")
        if raw_sources is None:
            raw_sources = [raw_feature]
        if not isinstance(raw_sources, list):
            raise ValueError(f"Feature {column!r} sources must be a list.")
        sources = [
            _source_spec_from_config(source, f"features.{column}.sources")
            for source in raw_sources
        ]
        specs.append(
            FeatureSpec(
                column=str(column),
                dtype=str(raw_feature.get("dtype") or "float32"),
                sources=sources,
                names=_string_list_config(raw_feature.get("names") or []),
            )
        )
    return specs


def _source_spec_from_config(raw_source: Any, path: str) -> SourceSpec:
    if isinstance(raw_source, str):
        return SourceSpec(topic=raw_source)
    if not isinstance(raw_source, Mapping):
        raise ValueError(f"{path} entries must be topic strings or objects.")
    field = _optional_string(raw_source, "field")
    fields = _string_list_config(raw_source.get("fields") or [])
    if field is not None and fields:
        raise ValueError(f"{path} entries must use either field or fields, not both.")
    if field == "all" or "all" in fields:
        raise ValueError(
            f"{path} entries must use fields to list explicit JointState fields; "
            "field 'all' is not supported."
        )
    return SourceSpec(
        topic=_required_string(raw_source, "topic", path),
        type_name=_optional_type_name(raw_source),
        field=field,
        fields=fields,
        required=bool(raw_source.get("required", True)),
        names=_string_list_config(raw_source.get("names") or []),
        joint_names=_string_list_config(raw_source.get("joint_names") or []),
        allow_extra_joint_names=bool(
            raw_source.get("allow_extra_joint_names", False)
        ),
    )


def _video_specs_from_config(config: Mapping[str, Any]) -> list[VideoSpec]:
    raw_videos = config.get("videos") or config.get("video_topics") or {}
    if not isinstance(raw_videos, Mapping):
        raise ValueError("videos/video_topics must be a mapping.")
    specs = []
    for column, raw_video in raw_videos.items():
        if isinstance(raw_video, str):
            specs.append(VideoSpec(column=str(column), topic=raw_video))
            continue
        if not isinstance(raw_video, Mapping):
            raise ValueError(f"Video mapping for {column!r} must be a topic string or object.")
        specs.append(
            VideoSpec(
                column=str(column),
                topic=_required_string(raw_video, "topic", f"videos.{column}"),
                type_name=_optional_type_name(raw_video) or "sensor_msgs/msg/Image",
                required=bool(raw_video.get("required", True)),
                fps=_optional_positive_int(raw_video, "fps"),
            )
        )
    return specs


def _depth_image_specs_from_config(
    config: Mapping[str, Any],
    video_specs: list[VideoSpec] | None = None,
) -> list[DepthImageSpec]:
    raw_depth_images = config.get("depth_images") or config.get("depth_image_topics") or {}
    if not isinstance(raw_depth_images, Mapping):
        raise ValueError("depth_images/depth_image_topics must be a mapping.")

    specs: list[DepthImageSpec] = []
    for column, raw_depth_image in raw_depth_images.items():
        if isinstance(raw_depth_image, str):
            specs.append(DepthImageSpec(column=str(column), topic=raw_depth_image))
            continue
        if not isinstance(raw_depth_image, Mapping):
            raise ValueError(
                f"Depth image mapping for {column!r} must be a topic string or object."
            )
        specs.append(
            DepthImageSpec(
                column=str(column),
                topic=_required_string(raw_depth_image, "topic", f"depth_images.{column}"),
                type_name=_optional_type_name(raw_depth_image) or "sensor_msgs/msg/Image",
                required=bool(raw_depth_image.get("required", True)),
                format=str(raw_depth_image.get("format") or "raw16"),
                fps=_optional_positive_int(raw_depth_image, "fps"),
            )
        )

    configured_topics = {spec.topic for spec in specs}
    configured_columns = {spec.column for spec in specs}
    for video_spec in video_specs or []:
        if (
            _looks_like_depth_stream(video_spec)
            and video_spec.topic not in configured_topics
            and video_spec.column not in configured_columns
        ):
            specs.append(
                DepthImageSpec(
                    column=video_spec.column,
                    topic=video_spec.topic,
                    type_name=video_spec.type_name,
                    required=video_spec.required,
                    fps=video_spec.fps,
                )
            )
    return specs


def _pointcloud_specs_from_config(config: Mapping[str, Any]) -> list[PointCloudSpec]:
    raw_pointclouds = config.get("pointclouds") or config.get("pointcloud_topics") or {}
    if not isinstance(raw_pointclouds, Mapping):
        raise ValueError("pointclouds/pointcloud_topics must be a mapping.")

    specs: list[PointCloudSpec] = []
    for column, raw_pointcloud in raw_pointclouds.items():
        if isinstance(raw_pointcloud, str):
            specs.append(PointCloudSpec(column=str(column), topic=raw_pointcloud))
            continue
        if not isinstance(raw_pointcloud, Mapping):
            raise ValueError(
                f"Pointcloud mapping for {column!r} must be a topic string or object."
            )
        specs.append(
            PointCloudSpec(
                column=str(column),
                topic=_required_string(raw_pointcloud, "topic", f"pointclouds.{column}"),
                type_name=_optional_type_name(raw_pointcloud) or "sensor_msgs/msg/PointCloud2",
                required=bool(raw_pointcloud.get("required", True)),
                primary=str(raw_pointcloud.get("primary") or "cdr"),
                derived_formats=[],
            )
        )
    return specs


def _looks_like_depth_stream(spec: VideoSpec) -> bool:
    label = f"{spec.column} {spec.topic}".lower()
    return "depth" in label


def _static_specs_from_config(config: Mapping[str, Any]) -> list[StaticTopicSpec]:
    raw_static = config.get("metadata_topics") or config.get("static_topics") or {}
    if isinstance(raw_static, list):
        return [
            StaticTopicSpec(key=_topic_stem(str(topic)), topic=str(topic))
            for topic in raw_static
        ]
    if not isinstance(raw_static, Mapping):
        raise ValueError("metadata_topics/static_topics must be a list or mapping.")
    specs = []
    for key, raw_static_topic in raw_static.items():
        if isinstance(raw_static_topic, str):
            specs.append(StaticTopicSpec(key=str(key), topic=raw_static_topic))
            continue
        if not isinstance(raw_static_topic, Mapping):
            raise ValueError(f"Static topic mapping for {key!r} must be a topic string or object.")
        specs.append(
            StaticTopicSpec(
                key=str(key),
                topic=_required_string(raw_static_topic, "topic", f"metadata_topics.{key}"),
                type_name=_optional_type_name(raw_static_topic),
                required=bool(raw_static_topic.get("required", True)),
            )
        )
    return specs


def _sampling_from_args_and_config(
    args: argparse.Namespace,
    config: Mapping[str, Any],
    fps: int,
    feature_specs: list[FeatureSpec],
    video_specs: list[VideoSpec],
    depth_image_specs: list[DepthImageSpec] | None = None,
    pointcloud_specs: list[PointCloudSpec] | None = None,
) -> SamplingSpec:
    sampling_config = config.get("sampling") or {}
    if not isinstance(sampling_config, Mapping):
        raise ValueError("sampling must be an object.")
    max_staleness_ms = args.max_staleness_ms
    if max_staleness_ms is None and sampling_config.get("max_staleness_ms") is not None:
        max_staleness_ms = float(sampling_config["max_staleness_ms"])
    max_staleness_ns = int(max_staleness_ms * 1_000_000) if max_staleness_ms is not None else None
    trim_start_sec = getattr(args, "trim_start_sec", None)
    trim_end_sec = getattr(args, "trim_end_sec", None)
    if trim_start_sec is None:
        trim_start_sec = float(sampling_config.get("trim_start_sec", 0.0))
    if trim_end_sec is None:
        trim_end_sec = float(sampling_config.get("trim_end_sec", 0.0))
    if trim_start_sec < 0.0 or trim_end_sec < 0.0:
        raise ValueError("sampling trim values must be non-negative")
    timing_options = {
        "max_staleness_ns": max_staleness_ns,
        "trim_start_ns": int(trim_start_sec * 1_000_000_000),
        "trim_end_ns": int(trim_end_sec * 1_000_000_000),
    }

    if args.sample_topic:
        return SamplingSpec("topic", topic=args.sample_topic, **timing_options)
    if args.sample_hz is not None:
        return SamplingSpec(
            "fixed_hz",
            frequency_hz=args.sample_hz,
            **timing_options,
        )
    if sampling_config.get("topic") or sampling_config.get("sample_topic"):
        return SamplingSpec(
            "topic",
            topic=str(sampling_config.get("topic") or sampling_config.get("sample_topic")),
            **timing_options,
        )
    if sampling_config.get("frequency_hz") or sampling_config.get("sample_hz"):
        return SamplingSpec(
            "fixed_hz",
            frequency_hz=float(
                sampling_config.get("frequency_hz") or sampling_config.get("sample_hz")
            ),
            **timing_options,
        )
    if video_specs or depth_image_specs or pointcloud_specs:
        return SamplingSpec("fixed_hz", frequency_hz=float(fps), **timing_options)
    if feature_specs and feature_specs[0].sources:
        return SamplingSpec(
            "topic",
            topic=feature_specs[0].sources[0].topic,
            **timing_options,
        )
    raise ValueError("No features or videos configured; cannot choose a sampling strategy.")


def _validate_topics(
    topic_types: Mapping[str, str],
    feature_specs: list[FeatureSpec],
    video_specs: list[VideoSpec],
    depth_image_specs: list[DepthImageSpec],
    pointcloud_specs: list[PointCloudSpec],
    static_topic_specs: list[StaticTopicSpec],
    sampling: SamplingSpec,
) -> None:
    missing = []
    for feature in feature_specs:
        for source in feature.sources:
            if source.required and source.topic not in topic_types:
                missing.append(source.topic)
    for video in video_specs:
        if video.required and video.topic not in topic_types:
            missing.append(video.topic)
    for depth_image in depth_image_specs:
        if depth_image.required and depth_image.topic not in topic_types:
            missing.append(depth_image.topic)
    for pointcloud in pointcloud_specs:
        if pointcloud.required and pointcloud.topic not in topic_types:
            missing.append(pointcloud.topic)
    for static_topic in static_topic_specs:
        if static_topic.required and static_topic.topic not in topic_types:
            missing.append(static_topic.topic)
    if sampling.strategy == "topic" and sampling.topic not in topic_types:
        missing.append(str(sampling.topic))
    if missing:
        raise ValueError(f"Configured topics not present in bag: {', '.join(sorted(set(missing)))}")


def _sampling_period_ns(sampling: SamplingSpec) -> int | None:
    if sampling.strategy != "fixed_hz":
        return None
    if sampling.frequency_hz is None or sampling.frequency_hz <= 0:
        raise ValueError("fixed_hz sampling requires a positive frequency_hz.")
    period_ns = int(1_000_000_000 / sampling.frequency_hz)
    if period_ns <= 0:
        raise ValueError("fixed_hz sampling period must be at least 1 ns.")
    return period_ns


def _trim_source_window(
    common_start_ns: int,
    common_end_ns: int,
    *,
    trim_start_ns: int,
    trim_end_ns: int,
) -> tuple[int, int]:
    if trim_start_ns < 0 or trim_end_ns < 0:
        raise ValueError("Source-time trim values must be non-negative")
    effective_start_ns = common_start_ns + trim_start_ns
    effective_end_ns = common_end_ns - trim_end_ns
    if effective_end_ns < effective_start_ns:
        duration_sec = max(0, common_end_ns - common_start_ns) / 1_000_000_000
        raise ValueError(
            "Trim removes the complete common source-time interval "
            f"({duration_sec:.3f} s available)"
        )
    return effective_start_ns, effective_end_ns


def _first_grid_time_ns(
    episode_start_ns: int,
    common_start_ns: int,
    period_ns: int,
) -> int:
    offset_ns = max(0, common_start_ns - episode_start_ns)
    periods = (offset_ns + period_ns - 1) // period_ns
    return episode_start_ns + periods * period_ns


def _message_source_time_ns(message: Any) -> int | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    timestamp = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return timestamp if timestamp > 0 else None


def _episode_schema(
    pa: Any,
    feature_specs: list[FeatureSpec],
    video_specs: list[VideoSpec],
    depth_image_specs: list[DepthImageSpec],
    pointcloud_specs: list[PointCloudSpec],
) -> Any:
    fields = [
        ("timestamp", pa.float32()),
        ("frame_index", pa.int64()),
        ("episode_index", pa.int64()),
        ("index", pa.int64()),
        ("task_index", pa.int64()),
        ("next.done", pa.bool_()),
    ]
    for feature in feature_specs:
        fields.append((feature.column, pa.list_(pa.float32())))
    video_type = pa.struct([("path", pa.string()), ("timestamp", pa.float32())])
    for video in video_specs:
        fields.append((video.column, video_type))
    depth_image_type = pa.struct(
        [
            ("data", pa.binary()),
            ("timestamp", pa.float32()),
            ("frame_index", pa.int64()),
            ("format", pa.string()),
            ("encoding", pa.string()),
            ("height", pa.int64()),
            ("width", pa.int64()),
            ("step", pa.int64()),
            ("is_bigendian", pa.bool_()),
        ]
    )
    for depth_image in depth_image_specs:
        fields.append((depth_image.column, depth_image_type))
    pointcloud_type = pa.struct(
        [
            ("data", pa.binary()),
            ("timestamp", pa.float32()),
            ("frame_index", pa.int64()),
            ("format", pa.string()),
            ("point_count", pa.int64()),
            ("height", pa.int64()),
            ("width", pa.int64()),
            ("point_step", pa.int64()),
            ("row_step", pa.int64()),
            ("is_dense", pa.bool_()),
        ]
    )
    for pointcloud in pointcloud_specs:
        fields.append((pointcloud.column, pointcloud_type))
    return pa.schema(fields)


def _lerobot_features(
    feature_specs: list[FeatureSpec],
    video_specs: list[VideoSpec],
    depth_image_specs: list[DepthImageSpec],
    pointcloud_specs: list[PointCloudSpec],
    results: list[EpisodeResult],
    fps: int,
) -> dict[str, Any]:
    features: dict[str, Any] = {
        "timestamp": {"dtype": "float32", "shape": [1], "names": None},
        "frame_index": {"dtype": "int64", "shape": [1], "names": None},
        "episode_index": {"dtype": "int64", "shape": [1], "names": None},
        "index": {"dtype": "int64", "shape": [1], "names": None},
        "task_index": {"dtype": "int64", "shape": [1], "names": None},
        "next.done": {"dtype": "bool", "shape": [1], "names": None},
    }
    for feature in feature_specs:
        dim = max((result.feature_dims.get(feature.column, 0) for result in results), default=0)
        names = feature.names or _source_names(feature)
        features[feature.column] = {
            "dtype": feature.dtype,
            "shape": [dim],
            "names": names if len(names) == dim else None,
        }
    for video in video_specs:
        shape = next(
            (
                result.video_shapes[video.column]
                for result in results
                if video.column in result.video_shapes
            ),
            [0, 0, 3],
        )
        features[video.column] = {
            "dtype": "video",
            "shape": shape,
            "names": ["height", "width", "channel"],
            "info": {
                "video.fps": video.fps or fps,
                "video.codec": "mp4v",
                "video.pix_fmt": "yuv420p",
                "video.is_depth_map": "depth" in video.column,
            },
        }
    for depth_image in depth_image_specs:
        shape = next(
            (
                result.depth_image_shapes[depth_image.column]
                for result in results
                if depth_image.column in result.depth_image_shapes
            ),
            [0, 0, 1],
        )
        features[depth_image.column] = {
            "dtype": "image",
            "shape": shape,
            "names": ["height", "width", "channel"],
            "info": {
                "image.format": depth_image.format,
                "image.fps": depth_image.fps or fps,
                "image.is_depth_map": True,
                "image.storage": "parquet_binary",
            },
        }
    for pointcloud in pointcloud_specs:
        features[pointcloud.column] = {
            "dtype": "pointcloud",
            "shape": [1],
            "names": None,
            "info": {
                "pointcloud.primary": pointcloud.primary,
                "pointcloud.storage": "parquet_binary",
                "pointcloud.derived_formats": [],
            },
        }
    return features


def _source_names(feature: FeatureSpec) -> list[str]:
    names: list[str] = []
    for source in feature.sources:
        names.extend(source.names or _generated_source_names(source))
    return names


def _generated_source_names(source: SourceSpec) -> list[str]:
    if source.type_name != "sensor_msgs/msg/JointState" or not source.joint_names:
        return []
    names: list[str] = []
    for field_name in _joint_state_fields(source.field, source.fields):
        names.extend(f"{joint_name}.{field_name}" for joint_name in source.joint_names)
    return names


def _message_to_source_value(
    type_name: str,
    message: Any,
    field_name: str | None,
    fields: list[str] | None = None,
    expected_joint_names: list[str] | None = None,
    allow_extra_joint_names: bool = False,
) -> list[float] | list[str] | None:
    if type_name == "sensor_msgs/msg/JointState":
        expected = list(expected_joint_names or [])
        if expected and allow_extra_joint_names:
            actual = [str(name) for name in message.name]
            overlap = set(actual).intersection(expected)
            if not overlap:
                return None
            missing = sorted(set(expected).difference(actual))
            if missing:
                raise ValueError(
                    f"JointState contains a partial named group; missing={missing}"
                )
        values: list[Any] = []
        for selected in _joint_state_fields(field_name, fields):
            values.extend(
                _joint_state_field_value(
                    message,
                    selected,
                    expected,
                    allow_extra_joint_names=allow_extra_joint_names,
                )
            )
        return values

    if type_name == "geometry_msgs/msg/PoseStamped":
        selected = field_name or "pose"
        position = message.pose.position
        orientation = message.pose.orientation
        if selected == "position":
            return [float(position.x), float(position.y), float(position.z)]
        if selected == "orientation":
            return [
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ]
        if selected == "pose":
            return [
                float(position.x),
                float(position.y),
                float(position.z),
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ]
        raise ValueError(f"Unsupported PoseStamped field: {selected}")

    if type_name == "teleop_interfaces/msg/ArmCommandStatus":
        selected = field_name or ""
        prefix = "accepted_sides."
        if not selected.startswith(prefix):
            raise ValueError(f"Unsupported ArmCommandStatus field: {selected}")
        side = selected.removeprefix(prefix)
        if side not in {"left", "right"}:
            raise ValueError(f"Unsupported ArmCommandStatus side: {side}")
        return [float(side in {str(item) for item in message.accepted_sides})]

    if type_name == "teleop_interfaces/msg/HandTelemetryStatus":
        selected = field_name or ""
        prefix = "engaged."
        if not selected.startswith(prefix):
            raise ValueError(f"Unsupported HandTelemetryStatus field: {selected}")
        side = selected.removeprefix(prefix)
        if side not in {"left", "right"}:
            raise ValueError(f"Unsupported HandTelemetryStatus side: {side}")
        if str(message.side) != side:
            return None
        return [float(bool(message.engaged))]

    if type_name == "std_msgs/msg/Float32MultiArray":
        return _float_list(message.data)

    value = _message_to_plain(message)
    if isinstance(value, list):
        return _float_list(value)
    raise ValueError(f"Unsupported feature source type: {type_name}")


def _joint_state_fields(field_name: str | None, fields: list[str] | None = None) -> list[str]:
    if fields:
        return list(fields)
    return [field_name or "position"]


def _joint_state_field_value(
    message: Any,
    selected: str,
    expected_joint_names: list[str] | None = None,
    *,
    allow_extra_joint_names: bool = False,
) -> list[float] | list[str]:
    expected = list(expected_joint_names or [])
    actual = [str(name) for name in message.name]
    if expected:
        if len(actual) != len(set(actual)):
            raise ValueError("JointState contains duplicate joint names")
        if (
            set(actual) != set(expected)
            and not (
                allow_extra_joint_names
                and set(expected).issubset(actual)
            )
        ):
            missing = sorted(set(expected) - set(actual))
            unknown = sorted(set(actual) - set(expected))
            raise ValueError(
                f"JointState name mismatch: missing={missing}, unknown={unknown}"
            )
        indices = [actual.index(name) for name in expected]
    else:
        indices = list(range(len(actual)))
    if selected == "name":
        return expected or actual
    if selected == "position":
        values = _float_list(message.position)
        return _reorder_joint_values(values, indices, actual, selected)
    if selected == "velocity":
        values = _float_list(message.velocity)
        return _reorder_joint_values(values, indices, actual, selected)
    if selected == "effort":
        values = _float_list(message.effort)
        return _reorder_joint_values(values, indices, actual, selected)
    raise ValueError(f"Unsupported JointState field: {selected}")


def _reorder_joint_values(
    values: list[float], indices: list[int], names: list[str], field_name: str
) -> list[float]:
    if len(values) != len(names):
        raise ValueError(
            f"JointState {field_name} length {len(values)} does not match "
            f"name length {len(names)}"
        )
    return [values[index] for index in indices]


def _image_message_to_bgr(message: Any) -> Any:
    import cv2
    import numpy as np

    height = int(message.height)
    width = int(message.width)
    encoding = str(message.encoding).lower()
    raw = bytes(message.data)

    if encoding in {"rgb8", "bgr8"}:
        frame = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 3))
        if encoding == "rgb8":
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame
    if encoding in {"rgba8", "bgra8"}:
        frame = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 4))
        if encoding == "rgba8":
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    if encoding in {"mono8", "8uc1"}:
        frame = np.frombuffer(raw, dtype=np.uint8).reshape((height, width))
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if encoding in {"mono16", "16uc1"}:
        frame = np.frombuffer(raw, dtype=np.uint16).reshape((height, width))
        finite = frame[np.isfinite(frame)]
        if finite.size == 0 or int(finite.max()) == int(finite.min()):
            scaled = np.zeros((height, width), dtype=np.uint8)
        else:
            scaled = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        return cv2.cvtColor(scaled, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"Unsupported image encoding for video conversion: {message.encoding}")


def _serialized_to_bytes(serialized: Any) -> bytes:
    if isinstance(serialized, bytes):
        return serialized
    if isinstance(serialized, bytearray):
        return bytes(serialized)
    try:
        return bytes(serialized)
    except TypeError:
        if hasattr(serialized, "buffer"):
            return bytes(serialized.buffer)
        raise


def _message_to_metadata_value(type_name: str, message: Any) -> Any:
    if type_name == "sensor_msgs/msg/CameraInfo":
        return {
            "height": int(message.height),
            "width": int(message.width),
            "distortion_model": str(message.distortion_model),
            "d": _float_list(message.d),
            "k": _float_list(message.k),
            "r": _float_list(message.r),
            "p": _float_list(message.p),
            "binning_x": int(message.binning_x),
            "binning_y": int(message.binning_y),
            "roi": _message_to_plain(message.roi),
        }
    return _message_to_plain(message)


def _compute_episode_stats(
    rows: list[dict[str, Any]],
    feature_specs: list[FeatureSpec],
) -> dict[str, Any]:
    import numpy as np

    stats: dict[str, Any] = {}
    if rows:
        timestamps = np.asarray([row["timestamp"] for row in rows], dtype=np.float32)
        stats["timestamp"] = _array_stats(timestamps)
    for feature in feature_specs:
        values = [row.get(feature.column) for row in rows if row.get(feature.column)]
        if not values:
            continue
        array = np.asarray(values, dtype=np.float32)
        stats[feature.column] = _array_stats(array)
    return stats


def _array_stats(array: Any) -> dict[str, Any]:
    import numpy as np

    with np.errstate(invalid="ignore"):
        return {
            "min": np.nanmin(array, axis=0).tolist(),
            "max": np.nanmax(array, axis=0).tolist(),
            "mean": np.nanmean(array, axis=0).tolist(),
            "std": np.nanstd(array, axis=0).tolist(),
            "count": int(array.shape[0]),
        }


def _resolve_message_class(type_name: str) -> Any:
    from rosidl_runtime_py.utilities import get_message

    return get_message(type_name)


def _deserialize_message(serialized: Any, message_class: Any) -> Any:
    from rclpy.serialization import deserialize_message

    return deserialize_message(serialized, message_class)


def _message_to_plain(message: Any) -> Any:
    from rosidl_runtime_py.convert import message_to_ordereddict

    return _plain_value(message_to_ordereddict(message))


def _plain_value(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _plain_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain_value(item) for item in value]
    if isinstance(value, bytes):
        return value.hex()
    if hasattr(value, "tolist"):
        return value.tolist()
    return value


def _float_list(values: Iterable[Any]) -> list[float]:
    return [float(value) for value in values]


def _string_list(values: Iterable[Any]) -> list[str]:
    return [str(value) for value in values]


def _json_dumps(value: Any) -> str:
    return json.dumps(value, ensure_ascii=True, separators=(",", ":"), sort_keys=True)


def _topic_stem(topic: str) -> str:
    stem = re.sub(r"[^A-Za-z0-9]+", "_", topic.strip("/")).strip("_").lower()
    return stem or "root"


def _chunk_name(episode_index: int, chunk_size: int) -> str:
    return f"chunk-{episode_index // chunk_size:03d}"


def _write_jsonl(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        for row in rows:
            file.write(json.dumps(row, sort_keys=True) + "\n")


def _required_string(config: Mapping[str, Any], key: str, path: str) -> str:
    value = config.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{path}.{key} must be a non-empty string.")
    return value


def _optional_string(config: Mapping[str, Any], key: str) -> str | None:
    value = config.get(key)
    if value is None:
        return None
    if not isinstance(value, str) or not value:
        raise ValueError(f"{key} must be a non-empty string when provided.")
    return value


def _optional_positive_int(config: Mapping[str, Any], key: str) -> int | None:
    value = config.get(key)
    if value is None:
        return None
    parsed = int(value)
    if parsed <= 0:
        raise ValueError(f"{key} must be positive when provided.")
    return parsed


def _optional_type_name(config: Mapping[str, Any]) -> str | None:
    value = config.get("type_name")
    if value is None:
        value = config.get("type")
    if value is None:
        return None
    if not isinstance(value, str) or not value:
        raise ValueError("type/type_name must be a non-empty string when provided.")
    return value


def _string_list_config(value: Any) -> list[str]:
    if value is None:
        return []
    if not isinstance(value, list):
        raise ValueError("names must be a list of strings.")
    return [str(item) for item in value]


def _write_dataset_readme(output_dir: Path, manifest: dict[str, Any]) -> None:
    readme = [
        "# Pico FR3/Wuji LeRobot Dataset",
        "",
        "Generated manually from finalized ROS 2 bags by `teleop_data_collector`.",
        "",
        "Load parquet rows with Hugging Face Datasets:",
        "",
        "```python",
        "from datasets import load_dataset",
        "",
        "dataset = load_dataset('parquet', data_files={'train': 'data/chunk-*/*.parquet'})",
        "```",
        "",
        "Or load frames — including decoded video, depth, and tactile — with the",
        "standalone loader shipped in this directory:",
        "",
        "```python",
        "from dataloader import LeRobotEpisodeDataset, make_dataloader",
        "",
        "dataset = LeRobotEpisodeDataset(video_keys='all', image_keys='all')",
        "loader = make_dataloader(dataset, batch_size=32, num_workers=4)",
        "```",
        "",
        "It needs only torch, numpy, pyarrow, and opencv-python. Run",
        "`python dataloader.py --media` to inspect a batch.",
        "",
        f"Episodes: {manifest['info']['total_episodes']}",
        f"Frames: {manifest['info']['total_frames']}",
        f"Videos: {manifest['info']['total_videos']}",
        "",
    ]
    (output_dir / "README.md").write_text("\n".join(readme), encoding="utf-8")
    (output_dir / "meta" / "README.md").write_text("\n".join(readme), encoding="utf-8")


def _write_dataset_dataloader(output_dir: Path) -> None:
    """Drop the standalone PyTorch loader into the dataset root.

    The dataset is meant to travel on its own, so the loader ships inside it
    rather than being imported from this repo. `--overwrite` wipes the output
    directory, so it is rewritten on every conversion.
    """
    template = Path(__file__).resolve().parent / DATALOADER_TEMPLATE_NAME
    if not template.is_file():
        print(f"Warning: dataloader template not found at {template}, skipping.")
        return
    shutil.copyfile(template, output_dir / "dataloader.py")


if __name__ == "__main__":
    main()
