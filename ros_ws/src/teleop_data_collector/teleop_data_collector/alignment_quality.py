"""Metadata-only, bounded 30 Hz causal quality gate; no SDK or image payloads.

Finality uses per-stream source watermarks from settled *received* samples,
never an extrapolated wall/source clock. A later fully valid point proves a
hole is internal; trailing/leading invalid points alone cannot stop an episode.
"""
from __future__ import annotations
from bisect import bisect_right
from dataclasses import asdict, dataclass
from typing import Any
from teleop_core.contract import COMMAND_STATUS_TOPIC, VALIDATED_COMMAND_TOPIC, LEFT_COMMAND_JOINT_NAMES, RIGHT_COMMAND_JOINT_NAMES

NS = 1_000_000_000
POLICY_VERSION = 'conversion-aligned-v1'


@dataclass(frozen=True)
class AlignmentPolicy:
    output_fps: int = 30
    max_staleness_ms: int = 150
    camera_short_gap_max_frames: int = 2
    camera_short_gap_max_age_ms: int = 250
    depth_short_gap_policy: str = 'known_compact_seams_only'
    trim_start_sec: float = 1.0
    trim_end_sec: float = .2
    arrival_settle_ms: int = 250
    stop_on_confirmed_internal_invalid: bool = True

    def __post_init__(self):
        import math
        if any(type(v) is not int for v in (self.output_fps, self.max_staleness_ms, self.camera_short_gap_max_frames, self.camera_short_gap_max_age_ms, self.arrival_settle_ms)):
            raise ValueError('Alignment grid and age parameters must be integer values')
        if (self.output_fps != 30 or self.max_staleness_ms != 150 or
            self.camera_short_gap_max_frames not in (0, 1, 2) or self.camera_short_gap_max_age_ms != 250 or
            self.depth_short_gap_policy != 'known_compact_seams_only'):
            raise ValueError('Unsupported conversion alignment policy; oracle uses 30 Hz, 150 ms, RGB <=2/250 ms')
        if any(not math.isfinite(v) or v < 0 for v in (self.trim_start_sec, self.trim_end_sec, self.arrival_settle_ms)):
            raise ValueError('Alignment trim and settle durations must be finite and nonnegative')
        if self.stop_on_confirmed_internal_invalid is not True:
            raise ValueError('conversion_aligned requires confirmed internal invalid auto-stop')

    def report(self):
        return {'version': POLICY_VERSION, **asdict(self), 'buffer_capacity_per_stream': 4096,
                'timestamp_semantics': 'source header time; Sharpa is ROS publication time',
                'conversion_oracle_commit': 'f5df024a40088bfdd73b44e9e155f6b5968af296',
                'conversion_oracle_config_sha256': '0f06df4dcd38ea6cd0d4144d7579f6424afe2cd9fe13a96295d21c2d712ee221',
                'finality': 'settled observed source watermarks; no prediction of future cleaner cuts'}


class Samples:
    def __init__(self):
        self.times = []
        self.values = []
        self.received = []
        self.first = None
        self.last_status = None

    def append(self, stamp, value, now):
        if self.times and stamp <= self.times[-1]:
            return False
        if self.first is None:
            self.first = stamp
        self.times.append(stamp); self.values.append(value); self.received.append(now)
        return True

    def at(self, stamp):
        i = bisect_right(self.times, stamp) - 1
        return (self.times[i], self.values[i]) if i >= 0 else (None, None)

    def prune(self, stamp):
        i = max(0, bisect_right(self.times, stamp) - 1)
        if i:
            del self.times[:i]; del self.values[:i]; del self.received[:i]

    def watermark(self, now, settle):
        i = bisect_right(self.received, now - settle) - 1
        return self.times[i] if i >= 0 else None


class AlignmentQuality:
    def __init__(self, topics, policy=None, *, compact_boundaries_ns=()):
        self.policy = policy or AlignmentPolicy()
        self.compact_boundaries_ns = tuple(compact_boundaries_ns)
        self.pending_depth = []
        self.long_depth = False
        self.previous_other_good = False
        self.previous_depth_grid = None
        self.accepted_depth_frames = 0
        self.topics = list(topics)
        self.samples = {t: Samples() for t in topics if t != VALIDATED_COMMAND_TOPIC}
        if VALIDATED_COMMAND_TOPIC in topics:
            for side in ('left', 'right'):
                self.samples[f'{VALIDATED_COMMAND_TOPIC}#{side}'] = Samples()
        self.anchor = None
        self.index = 0
        self.first_failure = None
        self.pending_visual = []
        self.long_visual = False
        self.previous_base = False
        self.first_valid = None
        self.last_valid = None
        self.valid_count = 0
        self.invalid_count = 0
        self.pending_internal = None
        self.accepted_rgb_frames = 0
        self.peak_samples = 0
        self.finished = False
        self._physical_first = {}
        self._physical_last = {}

    def observe(self, topic, message, stamp, now):
        if topic not in self.topics or not stamp or stamp <= 0:
            return
        self._physical_first.setdefault(topic, stamp)
        self._physical_last[topic] = stamp
        if topic == VALIDATED_COMMAND_TOPIC:
            names = set(message.name)
            for side, expected in (('left', LEFT_COMMAND_JOINT_NAMES), ('right', RIGHT_COMMAND_JOINT_NAMES)):
                if set(expected).issubset(names):
                    self._append(f'{topic}#{side}', stamp, None, now)
        elif topic == COMMAND_STATUS_TOPIC:
            stream = self.samples[topic]
            previous = stream.last_status or {}
            value = {}
            for side in ('left', 'right'):
                engaged = side in message.accepted_sides
                old = previous.get(side)
                transition = stamp if old is None or old[0] != engaged else old[2]
                value[side] = (engaged, not bool(message.faults), transition)
            stream.last_status = value
            self._append(topic, stamp, value, now)
        else:
            self._append(topic, stamp, None, now)

    def _append(self, topic, stamp, value, now):
        stream = self.samples[topic]
        stream.append(stamp, value, now)
        self.peak_samples = max(self.peak_samples, len(stream.times))
        if len(stream.times) > 4096:
            self._fail({'topic': topic, 'message': 'quality metadata buffer exhausted', 'category': 'alignment_capacity'})
            # Explicit refusal, never silently drop data and call it valid.
            del stream.times[0]; del stream.values[0]; del stream.received[0]

    def _fail(self, reason):
        if self.first_failure is None:
            self.first_failure = {'policy_version': POLICY_VERSION, **reason}

    def advance(self, now, *, final=False, start_ns=None, end_ns=None):
        if self.finished:
            return
        if not self.topics or any(t not in self._physical_first for t in self.topics):
            if final:
                self._fail({'category': 'alignment', 'message': 'required streams contain no samples'})
                self.finished = True
            return
        if self.anchor is None:
            self.anchor = start_ns if start_ns is not None else max(self._physical_first.values()) + int(self.policy.trim_start_sec * NS)
        if final:
            end = end_ns if end_ns is not None else min(v for t, v in self._physical_last.items() if t != VALIDATED_COMMAND_TOPIC) - int(self.policy.trim_end_sec * NS)
        else:
            watermarks = [s.watermark(now, self.policy.arrival_settle_ms / 1000)
                          for t, s in self.samples.items() if not t.startswith(VALIDATED_COMMAND_TOPIC + '#')]
            if not watermarks or any(w is None for w in watermarks):
                return
            end = min(watermarks) - int(self.policy.trim_end_sec * NS)
        # Grid construction is the converter's integer-nanosecond expression.
        budget = 128
        while self.anchor + self.index * NS // self.policy.output_fps <= end:
            if not final:
                budget -= 1
                if budget < 0:
                    return
            stamp = self.anchor + self.index * NS // self.policy.output_fps
            point = self._point(stamp)
            if not final:
                # Active actions also need a source watermark. No assumption
                # that a timer or an inactive status makes a delayed action final.
                for side in point['engaged']:
                    key = f'{VALIDATED_COMMAND_TOPIC}#{side}'
                    watermark = self.samples[key].watermark(now, self.policy.arrival_settle_ms / 1000)
                    if watermark is None or watermark < stamp:
                        return
            point.update(grid_index=self.index, source_timestamp_ns=stamp)
            self._visual(point)
            self.index += 1
            for sample in self.samples.values():
                sample.prune(stamp)
        if final:
            # A trailing RGB hole has no right boundary and cannot be bridged.
            for point in self.pending_visual:
                self._record(point, False)
            self.pending_visual.clear()
            for point in self.pending_depth:
                self._final_record(point, False)
            self.pending_depth.clear()
            if self.valid_count < 2:
                self._fail({'category': 'alignment', 'message': 'fewer than two valid aligned output frames'})
            self.finished = True

    def _point(self, stamp):
        max_age = self.policy.max_staleness_ms * 1_000_000
        ages, base_bad, extra_bad, rgb_bad, engaged = {}, [], [], [], []
        for topic, stream in self.samples.items():
            if topic.startswith(VALIDATED_COMMAND_TOPIC + '#'):
                continue
            source, value = stream.at(stamp)
            age = stamp - source if source is not None else None
            ages[topic] = age
            good = age is not None and 0 <= age <= max_age
            if topic == COMMAND_STATUS_TOPIC:
                for side in ('left', 'right'):
                    if value is None:
                        continue
                    active, valid, transition = value[side]
                    if not valid:
                        base_bad.append(topic + '#' + side + ':invalid_status')
                    if active and VALIDATED_COMMAND_TOPIC in self.topics:
                        engaged.append(side)
                        key = f'{VALIDATED_COMMAND_TOPIC}#{side}'
                        command_time, _ = self.samples[key].at(stamp)
                        action_age = stamp - command_time if command_time is not None else None
                        ages[key] = action_age
                        if (action_age is None or not 0 <= action_age <= max_age or command_time + 20_000_000 < transition):
                            base_bad.append(key)
            if not good:
                if topic.startswith('/cam') and '/color/image_raw' in topic:
                    rgb_bad.append(topic)
                elif topic == '/cam0/depth/image_raw' or '/tactile/' in topic:
                    extra_bad.append(topic)
                else:
                    base_bad.append(topic)
        return dict(ages_ns=ages, base_bad=base_bad, extra_bad=extra_bad, rgb_bad=rgb_bad,
                    engaged=engaged, base_good=not base_bad)

    def _visual(self, point):
        if point['rgb_bad']:
            if not self.long_visual:
                self.pending_visual.append(point)
                if len(self.pending_visual) <= self.policy.camera_short_gap_max_frames:
                    return
                for pending in self.pending_visual:
                    self._record(pending, False)
                self.pending_visual.clear()
                self.long_visual = True
            else:
                self._record(point, False)
            return
        if self.pending_visual:
            rgb_topics = [t for t in self.topics if t.startswith('/cam') and '/color/image_raw' in t]
            allowed = (self.previous_base and point['base_good'] and
                       all(p['base_good'] and all(p['ages_ns'].get(t) is not None and
                           0 <= p['ages_ns'][t] <= self.policy.camera_short_gap_max_age_ms * 1_000_000
                           for t in rgb_topics) for p in self.pending_visual))
            for pending in self.pending_visual:
                self._record(pending, allowed)
            if allowed:
                self.accepted_rgb_frames += len(self.pending_visual)
            self.pending_visual.clear()
        self.long_visual = False
        self._record(point, True)
        self.previous_base = point['base_good']

    def _record(self, point, rgb_good):
        point = dict(point, rgb_good=rgb_good)
        depth = '/cam0/depth/image_raw'
        if not self.compact_boundaries_ns:
            return self._final_record(point, point['base_good'] and not point['extra_bad'] and rgb_good)
        other_good = point['base_good'] and rgb_good and not any(t != depth for t in point['extra_bad'])
        point['other_good'] = other_good
        if depth in point['extra_bad']:
            if not self.long_depth:
                self.pending_depth.append(point)
                if len(self.pending_depth) <= self.policy.camera_short_gap_max_frames:
                    return
                for pending in self.pending_depth:
                    self._final_record(pending, False)
                self.pending_depth.clear(); self.long_depth = True
            else:
                self._final_record(point, False)
            return
        if self.pending_depth:
            allowed = (self.previous_other_good and other_good and
                any(self.previous_depth_grid <= b <= point['source_timestamp_ns'] for b in self.compact_boundaries_ns) and
                any(p['rgb_bad'] and p['rgb_good'] for p in self.pending_depth) and
                all(p['other_good'] and p['ages_ns'].get(depth) is not None and
                    0 <= p['ages_ns'][depth] <= self.policy.camera_short_gap_max_age_ms * 1_000_000 for p in self.pending_depth))
            for pending in self.pending_depth:
                self._final_record(pending, allowed)
            if allowed:
                self.accepted_depth_frames += len(self.pending_depth)
            self.pending_depth.clear()
        self.long_depth = False
        self._final_record(point, other_good)
        self.previous_other_good = other_good
        self.previous_depth_grid = point['source_timestamp_ns']

    def _final_record(self, point, good):
        rgb_good = point['rgb_good']
        if good:
            if self.first_valid is None:
                self.first_valid = point['grid_index']
            if self.pending_internal is not None:
                self._fail(self.pending_internal)
            self.last_valid = point['grid_index']
            self.valid_count += 1
            self.pending_internal = None
        else:
            self.invalid_count += 1
            if self.first_valid is not None and self.pending_internal is None:
                bad = point['base_bad'] + point['extra_bad'] + ([] if rgb_good else point['rgb_bad'])
                self.pending_internal = {'category': 'alignment', 'message': 'confirmed internal invalid 30 Hz grid point',
                    'grid_index': point['grid_index'], 'source_timestamp_ns': point['source_timestamp_ns'],
                    'topic': bad[0] if bad else 'alignment', 'invalid_topics': bad,
                    'ages_ns': dict(point['ages_ns']), 'rgb_short_gap_accepted': rgb_good and bool(point['rgb_bad'])}

    def report(self):
        return {'policy': self.policy.report(), 'grid_anchor_ns': self.anchor, 'checked_grid_points': self.index,
                'valid_grid_points': self.valid_count, 'invalid_grid_points': self.invalid_count,
                'first_valid_grid': self.first_valid, 'last_valid_grid': self.last_valid,
                'accepted_rgb_grid_points': self.accepted_rgb_frames, 'accepted_depth_seam_grid_points': self.accepted_depth_frames, 'peak_samples_per_stream': self.peak_samples,
                'pending_visual_points': len(self.pending_visual), 'first_failure': self.first_failure,
                'finished': self.finished}


def validate_metadata(topic, message):
    """B preserves structural refusal independently of timing acceptance."""
    import math
    from teleop_core.contract import DATA_LEFT_ARM_JOINT_NAMES, DATA_RIGHT_ARM_JOINT_NAMES, SHARPA_JOINT_NAMES
    expected = None
    fields = ('position',)
    if topic in ('/left/franka/joint_states', '/right/franka/joint_states'):
        expected = DATA_LEFT_ARM_JOINT_NAMES if topic.startswith('/left/') else DATA_RIGHT_ARM_JOINT_NAMES
        fields = ('position', 'velocity')
    elif topic.startswith('/sharpa/') and topic.endswith(('/command', '/joint_states')):
        side = topic.split('/')[2]
        expected = SHARPA_JOINT_NAMES if topic.endswith('/command') else tuple(f'{side}_{name}' for name in SHARPA_JOINT_NAMES)
        if topic.endswith('/joint_states'):
            fields = ('position', 'velocity')
    elif topic == VALIDATED_COMMAND_TOPIC:
        names = list(message.name)
        if len(names) != len(set(names)):
            raise ValueError('duplicate action joint names')
        matched = False
        for group in (LEFT_COMMAND_JOINT_NAMES, RIGHT_COMMAND_JOINT_NAMES):
            if set(names).intersection(group):
                if not set(group).issubset(names):
                    raise ValueError('partial validated arm action')
                matched = True
        if not matched or len(message.position) != len(names) or not all(math.isfinite(v) for v in message.position):
            raise ValueError('invalid validated arm action')
    elif topic == COMMAND_STATUS_TOPIC:
        if set(message.accepted_sides).difference(('left', 'right')) or message.faults:
            raise ValueError('arm status reports faults or unknown sides')
    elif topic.endswith('/wrench'):
        w = message.wrench
        if not all(math.isfinite(v) for v in (w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z)):
            raise ValueError('nonfinite tactile wrench')
    if expected is not None:
        names = list(message.name)
        if len(names) != len(set(names)) or set(names) != set(expected):
            raise ValueError('invalid required joint names: ' + topic)
        for field in fields:
            values = getattr(message, field)
            if len(values) != len(names) or not all(math.isfinite(v) for v in values):
                raise ValueError('invalid joint array: ' + topic + ':' + field)
    if '/image_raw' in topic or topic.endswith('/deformation'):
        encoding = message.encoding.lower()
        expected_encoding = '16uc1' if topic == '/cam0/depth/image_raw' else 'mono8' if '/tactile/' in topic else None
        channels = {'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4,'mono8':1,'16uc1':2}
        if (encoding not in channels or (expected_encoding is None and encoding not in ('rgb8','bgr8','rgba8','bgra8','mono8')) or (expected_encoding and encoding != expected_encoding) or
            message.width <= 0 or message.height <= 0 or message.step < message.width * channels[encoding] or
            len(message.data) != message.step * message.height):
            raise ValueError('invalid image encoding/shape/payload: ' + topic)
        if topic == '/cam0/depth/image_raw' and (message.width, message.height, message.step) != (640, 400, 1280):
            raise ValueError('head depth must be packed 640x400 uint16')
