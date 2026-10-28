"""State, presets, and command formatting for the Linker Hand control UI."""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from importlib import import_module
from pathlib import Path
from typing import Protocol, cast

from barmate.hardware.linker_hand.control.types import HandPlaybackFrame
from barmate.hardware.linker_hand.linker_hand_sdk import (
    MODEL_DEFAULT_SPEED,
    MODEL_DEFAULT_TORQUE,
    MODEL_JOINT_COUNTS,
)

MIN_CONTROL_VALUE = 0
MAX_CONTROL_VALUE = 255
DEFAULT_MODEL = "L20"
DEFAULT_ACTION_SEQUENCE_LOOP_COUNT = 1
DEFAULT_ACTION_SEQUENCE_LOOP_INTERVAL_SECONDS = 0.0
CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
SIDE_CONFIG_FILES = {
    "LEFT_HAND": "left_hand.yaml",
    "RIGHT_HAND": "right_hand.yaml",
}

FINGER_NAMES = ("拇指", "食指", "中指", "无名指", "小指")
HAND_SIDES = ("LEFT_HAND", "RIGHT_HAND")
HAND_LABELS = {"LEFT_HAND": "Left hand", "RIGHT_HAND": "Right hand"}
RuntimeConfigKey = tuple[str, str]
TIME_EPSILON_SECONDS = 1e-9


class YamlModule(Protocol):
    def safe_load(self, stream: str) -> object: ...

    def safe_dump(
        self, data: object, *, allow_unicode: bool, sort_keys: bool
    ) -> str: ...


# SDK 的 action_joint 不是所有型号都按手指连续排列。这里保存 UI 卡片到
# SDK 指令下标的映射；未列出的型号才退回到平均分配。
FingerJointLayout = tuple[tuple[int, str], ...]
ModelFingerLayout = tuple[
    FingerJointLayout,
    FingerJointLayout,
    FingerJointLayout,
    FingerJointLayout,
    FingerJointLayout,
]

MODEL_FINGER_JOINTS: dict[str, ModelFingerLayout] = {
    "L6": (
        ((0, "根部"), (1, "外展")),
        ((2, "根部"),),
        ((3, "根部"),),
        ((4, "根部"),),
        ((5, "根部"),),
    ),
    "O6": (
        ((0, "根部"), (1, "外展")),
        ((2, "根部"),),
        ((3, "根部"),),
        ((4, "根部"),),
        ((5, "根部"),),
    ),
    "L7": (
        ((0, "根部"), (1, "外展"), (6, "旋转")),
        ((2, "根部"),),
        ((3, "根部"),),
        ((4, "根部"),),
        ((5, "根部"),),
    ),
    "L10": (
        ((0, "根部"), (1, "外展"), (9, "旋转")),
        ((2, "根部"), (6, "外展")),
        ((3, "根部"),),
        ((4, "根部"), (7, "外展")),
        ((5, "根部"), (8, "外展")),
    ),
    "L20": (
        ((0, "根部"), (5, "外展"), (10, "旋转"), (15, "远端")),
        ((1, "根部"), (6, "外展"), (16, "远端")),
        ((2, "根部"), (7, "外展"), (17, "远端")),
        ((3, "根部"), (8, "外展"), (18, "远端")),
        ((4, "根部"), (9, "外展"), (19, "远端")),
    ),
    "G20": (
        ((0, "根部"), (5, "外展"), (10, "旋转"), (15, "远端")),
        ((1, "根部"), (6, "外展"), (16, "远端")),
        ((2, "根部"), (7, "外展"), (17, "远端")),
        ((3, "根部"), (8, "外展"), (18, "远端")),
        ((4, "根部"), (9, "外展"), (19, "远端")),
    ),
    "O30I": (
        ((0, "横滚"), (1, "航向"), (6, "指根1"), (15, "指尖")),
        ((2, "航向"), (7, "指根1"), (11, "指根2"), (16, "指尖")),
        ((3, "航向"), (8, "指根1"), (12, "指根2"), (17, "指尖")),
        ((4, "航向"), (9, "指根1"), (13, "指根2"), (18, "指尖")),
        ((5, "航向"), (10, "指根1"), (14, "指根2"), (19, "指尖")),
    ),
    "L25": (
        ((0, "根部"), (5, "外展"), (10, "旋转"), (15, "近端"), (20, "远端")),
        ((1, "根部"), (6, "外展"), (16, "近端"), (21, "远端")),
        ((2, "根部"), (7, "外展"), (17, "近端"), (22, "远端")),
        ((3, "根部"), (8, "外展"), (18, "近端"), (23, "远端")),
        ((4, "根部"), (9, "外展"), (19, "近端"), (24, "远端")),
    ),
}

# 只用于未显式列出 SDK 下标映射的型号兜底。
MODEL_FINGER_COUNTS: dict[str, tuple[int, int, int, int, int]] = {
    "L6": (2, 1, 1, 1, 1),
    "L7": (3, 1, 1, 1, 1),
    "L10": (2, 2, 2, 2, 2),
    "L20": (4, 4, 4, 4, 4),
    "L21": (5, 4, 4, 4, 4),
    "L25": (5, 5, 5, 5, 5),
    "G20": (4, 4, 4, 4, 4),
    "O6": (2, 1, 1, 1, 1),
    "O30I": (4, 4, 4, 4, 4),
}


@dataclass(frozen=True, slots=True)
class JointSpec:
    """单个 UI 关节的显示信息。"""

    index: int
    finger: str
    name: str


@dataclass(frozen=True, slots=True)
class FingerSpec:
    """一根手指及其包含的关节。"""

    name: str
    joints: tuple[JointSpec, ...]


@dataclass(frozen=True, slots=True)
class HandPreset:
    """单只手的预设姿态。"""

    name: str
    values: tuple[int, ...]
    source: str = "Configured"
    id: str = ""


@dataclass(frozen=True, slots=True)
class ActionSequenceStep:
    """Action sequence 中的一次定时预设触发。"""

    timestamp: float
    preset_id: str


@dataclass(frozen=True, slots=True)
class HandActionSequence:
    """单只手的一组 timestamp -> preset 动作序列。"""

    id: str
    name: str
    steps: tuple[ActionSequenceStep, ...]
    loop_count: int = DEFAULT_ACTION_SEQUENCE_LOOP_COUNT
    loop_interval_seconds: float = DEFAULT_ACTION_SEQUENCE_LOOP_INTERVAL_SECONDS


@dataclass(slots=True)
class HandTask:
    """用户任务：只保存控制实体 ID 的展示分组。"""

    id: str
    name: str
    preset_ids: list[str] = field(default_factory=list)
    action_sequence_ids: list[str] = field(default_factory=list)


@dataclass(slots=True)
class HandState:
    """单只手的 UI 控制状态。"""

    side: str
    model: str = DEFAULT_MODEL
    values: list[int] = field(default_factory=list)
    speed: int = MAX_CONTROL_VALUE
    torque: int = 200
    active_group: str = ""
    presets: OrderedDict[str, HandPreset] = field(default_factory=OrderedDict)
    action_sequences: OrderedDict[str, HandActionSequence] = field(
        default_factory=OrderedDict
    )
    tasks: OrderedDict[str, HandTask] = field(default_factory=OrderedDict)
    preset_groups: OrderedDict[str, list[HandPreset]] = field(
        default_factory=OrderedDict
    )
    action_sequence_groups: OrderedDict[str, list[HandActionSequence]] = field(
        default_factory=OrderedDict
    )
    new_task_name: str = ""
    new_preset_name: str = ""

    @property
    def label(self) -> str:
        return HAND_LABELS[self.side]

    @property
    def joint_count(self) -> int:
        return MODEL_JOINT_COUNTS[self.model]

    @property
    def fingers(self) -> tuple[FingerSpec, ...]:
        return build_finger_specs(self.model)

    def task_label(self, task_id: str) -> str:
        task = self.tasks.get(task_id)
        return task.name if task is not None else task_id


class ControlPageState:
    """页面状态集中存放，便于页面描述与控制逻辑分离。"""

    def __init__(self, model: str = DEFAULT_MODEL) -> None:
        model = normalize_model(model)
        self.hands: dict[str, HandState] = {
            side: build_hand_state(side, model) for side in HAND_SIDES
        }
        self.status: str = "Ready"
        self.trajectory_path: str = ""
        self.playback_frames: list[HandPlaybackFrame] = []
        self.is_playing: bool = False
        self.command_console_open: bool = False
        self.command_log: list[str] = []

    def rebuild_hand_for_model(self, side: str, model: str) -> None:
        hand = self.hands[side]
        hand.model = normalize_model(model)
        hand.values[:] = normalize_pose(hand.values, hand.joint_count)
        hand.speed = default_speed_value(hand.model)
        hand.torque = default_torque_value(hand.model)
        rebuild_presets(hand)
        self.status = f"{hand.label}: {hand.model}"

    def append_command(self, command: str) -> None:
        self.command_log.append(command)
        if len(self.command_log) > 200:
            del self.command_log[: len(self.command_log) - 200]

    def newest_commands(self) -> list[str]:
        return list(reversed(self.command_log))


@dataclass(slots=True)
class RuntimeConfigCommandFilter:
    """Throttle live speed/torque command streams while preserving the latest value."""

    interval_seconds: float
    last_sent_at: dict[RuntimeConfigKey, float] = field(default_factory=dict)
    pending_values: dict[RuntimeConfigKey, int] = field(default_factory=dict)

    def should_send(self, side: str, kind: str, value: int, now: float) -> bool:
        key = (side, kind)
        last_sent_at = self.last_sent_at.get(key)
        if (
            last_sent_at is None
            or now - last_sent_at + TIME_EPSILON_SECONDS >= self.interval_seconds
        ):
            self.last_sent_at[key] = now
            _ = self.pending_values.pop(key, None)
            return True
        self.pending_values[key] = value
        return False

    def pop_due(self, now: float) -> list[tuple[str, str, int]]:
        due: list[tuple[str, str, int]] = []
        for key, value in list(self.pending_values.items()):
            last_sent_at = self.last_sent_at.get(key)
            if (
                last_sent_at is None
                or now - last_sent_at + TIME_EPSILON_SECONDS >= self.interval_seconds
            ):
                self.last_sent_at[key] = now
                del self.pending_values[key]
                side, kind = key
                due.append((side, kind, value))
        return due


def build_hand_state(side: str, model: str = DEFAULT_MODEL) -> HandState:
    hand = HandState(side=side, model=normalize_model(model))
    hand.values = [MIN_CONTROL_VALUE] * hand.joint_count
    hand.speed = default_speed_value(hand.model)
    hand.torque = default_torque_value(hand.model)
    rebuild_presets(hand)
    return hand


def normalize_model(model: str) -> str:
    return model if model in MODEL_JOINT_COUNTS else DEFAULT_MODEL


def default_speed_value(model: str) -> int:
    return first_config_value(MODEL_DEFAULT_SPEED.get(model, (MAX_CONTROL_VALUE,)))


def default_torque_value(model: str) -> int:
    return first_config_value(MODEL_DEFAULT_TORQUE.get(model, (200,)))


def first_config_value(values: Sequence[int]) -> int:
    return clamp_control_value(values[0]) if values else MAX_CONTROL_VALUE


def emergency_stop_values(hand: HandState) -> list[int]:
    """Return a model-safe pose after interrupting active playback."""

    if hand.model == "O30I":
        return list(hand.values)
    return [MAX_CONTROL_VALUE] * hand.joint_count


def rebuild_presets(hand: HandState) -> None:
    load_configured_presets(hand)


def add_configured_task(hand: HandState, name: str) -> HandTask:
    """创建任务并立即落盘到当前手侧配置文件。"""

    task_name = name.strip()
    task_id = unique_config_id(task_name, "task", hand.tasks)
    task = HandTask(task_id, task_name)
    hand.tasks[task_id] = task
    hand.active_group = task_id
    sync_preset_groups(hand)
    save_configured_presets(hand)
    return task


def save_configured_preset(hand: HandState, name: str) -> HandPreset:
    """把当前关节值保存为预设，并把预设 ID 加入当前任务。"""

    if hand.active_group not in hand.tasks:
        raise ValueError("Create a task first")
    preset_name = name.strip() or f"Custom pose {len(hand.presets) + 1}"
    preset_id = unique_config_id(preset_name, "preset", hand.presets)
    preset = HandPreset(
        preset_name,
        tuple(normalize_pose(hand.values, hand.joint_count)),
        "Configured",
        preset_id,
    )
    hand.presets[preset_id] = preset
    task = hand.tasks[hand.active_group]
    task.preset_ids.append(preset_id)
    sync_preset_groups(hand)
    save_configured_presets(hand)
    return preset


def save_configured_action_sequence(
    hand: HandState,
    name: str,
    steps: Sequence[ActionSequenceStep],
    sequence_id: str | None = None,
    *,
    loop_count: object = DEFAULT_ACTION_SEQUENCE_LOOP_COUNT,
    loop_interval_seconds: object = DEFAULT_ACTION_SEQUENCE_LOOP_INTERVAL_SECONDS,
) -> HandActionSequence:
    """创建或更新当前任务下的 action sequence 并写入配置文件。"""

    if hand.active_group not in hand.tasks:
        raise ValueError("Create a task first")
    sequence_name = name.strip() or f"Action sequence {len(hand.action_sequences) + 1}"
    normalized_steps = normalize_action_sequence_steps(steps, hand.presets)
    if not normalized_steps:
        raise ValueError("Add at least one valid action")
    if sequence_id is None:
        sequence_id = unique_config_id(sequence_name, "sequence", hand.action_sequences)
    sequence = HandActionSequence(
        sequence_id,
        sequence_name,
        tuple(normalized_steps),
        normalize_action_sequence_loop_count(loop_count),
        normalize_action_sequence_loop_interval_seconds(loop_interval_seconds),
    )
    hand.action_sequences[sequence_id] = sequence
    task = hand.tasks[hand.active_group]
    if sequence_id not in task.action_sequence_ids:
        task.action_sequence_ids.append(sequence_id)
    sync_preset_groups(hand)
    save_configured_presets(hand)
    return sequence


def add_task_item(hand: HandState, task_id: str, item_kind: str, item_id: str) -> bool:
    """把已有 preset/action sequence 引用加入任务，不复制控制实体。"""

    task = hand.tasks.get(task_id)
    if task is None:
        raise ValueError("Task does not exist")
    target_ids, source_items = task_items_for_kind(hand, task, item_kind)
    if item_id not in source_items:
        raise ValueError("Action does not exist")
    if item_id in target_ids:
        return False
    target_ids.append(item_id)
    sync_preset_groups(hand)
    save_configured_presets(hand)
    return True


def remove_task_item(
    hand: HandState, task_id: str, item_kind: str, item_id: str
) -> bool:
    """从任务中移除 preset/action sequence 引用，保留实体本身。"""

    task = hand.tasks.get(task_id)
    if task is None:
        raise ValueError("Task does not exist")
    target_ids, _ = task_items_for_kind(hand, task, item_kind)
    if item_id not in target_ids:
        return False
    target_ids[:] = [
        selected_id for selected_id in target_ids if selected_id != item_id
    ]
    sync_preset_groups(hand)
    save_configured_presets(hand)
    return True


def task_items_for_kind(
    hand: HandState,
    task: HandTask,
    item_kind: str,
) -> tuple[
    list[str], OrderedDict[str, HandPreset] | OrderedDict[str, HandActionSequence]
]:
    if item_kind == "preset":
        return task.preset_ids, hand.presets
    if item_kind == "sequence":
        return task.action_sequence_ids, hand.action_sequences
    raise ValueError(f"Unknown action type: {item_kind}")


def clamp_control_value(value: object) -> int:
    """将输入控制值限制在 0-255，避免手动输入或配置越界。"""

    return max(MIN_CONTROL_VALUE, min(MAX_CONTROL_VALUE, int(round(float(str(value))))))


def normalize_pose(values: Sequence[object], joint_count: int) -> list[int]:
    """把不同型号的预设姿态裁剪或补齐到当前型号关节数。"""

    normalized = [clamp_control_value(value) for value in values[:joint_count]]
    normalized.extend([MIN_CONTROL_VALUE] * (joint_count - len(normalized)))
    return normalized


def normalize_timestamp(value: object) -> float:
    return max(0.0, float(str(value)))


def normalize_action_sequence_loop_count(value: object) -> int:
    raw_loop_count = float(str(value))
    loop_count = int(raw_loop_count)
    if raw_loop_count != loop_count or loop_count == 0 or loop_count < -1:
        raise ValueError("Loop count must be -1 or a positive integer")
    return loop_count


def normalize_action_sequence_loop_interval_seconds(value: object) -> float:
    return max(0.0, float(str(value)))


def normalize_action_sequence_steps(
    steps: Sequence[ActionSequenceStep],
    presets: OrderedDict[str, HandPreset],
) -> list[ActionSequenceStep]:
    normalized = [
        ActionSequenceStep(normalize_timestamp(step.timestamp), step.preset_id)
        for step in steps
        if step.preset_id in presets
    ]
    return sorted(normalized, key=lambda step: step.timestamp)


def build_finger_specs(model: str) -> tuple[FingerSpec, ...]:
    """根据型号总关节数生成 5 张手指卡片的关节信息。"""

    layout = MODEL_FINGER_JOINTS.get(model)
    if layout is not None:
        return tuple(
            FingerSpec(
                finger_name,
                tuple(
                    JointSpec(index, finger_name, format_joint_name(index, role_name))
                    for index, role_name in joints
                ),
            )
            for finger_name, joints in zip(FINGER_NAMES, layout, strict=True)
        )

    counts = MODEL_FINGER_COUNTS.get(
        model, evenly_split_joints(MODEL_JOINT_COUNTS[model])
    )
    specs: list[FingerSpec] = []
    joint_index = 0
    for finger_name, count in zip(FINGER_NAMES, counts, strict=True):
        joints: list[JointSpec] = []
        for local_index in range(count):
            role_name = joint_role_name(local_index, count)
            joints.append(
                JointSpec(
                    joint_index, finger_name, format_joint_name(joint_index, role_name)
                )
            )
            joint_index += 1
        specs.append(FingerSpec(finger_name, tuple(joints)))
    return tuple(specs)


def format_joint_name(index: int, role_name: str) -> str:
    joint_name = f"J{index + 1:02d}"
    return joint_name if not role_name else f"{joint_name} · {role_name}"


def evenly_split_joints(joint_count: int) -> tuple[int, int, int, int, int]:
    """未知型号兜底：尽量平均分配到 5 根手指。"""

    base, extra = divmod(joint_count, len(FINGER_NAMES))
    return (
        base + (1 if 0 < extra else 0),
        base + (1 if 1 < extra else 0),
        base + (1 if 2 < extra else 0),
        base + (1 if 3 < extra else 0),
        base + (1 if 4 < extra else 0),
    )


def joint_role_name(local_index: int, count: int) -> str:
    """给不同数量的关节取可读中文名称。"""

    if count == 1:
        return ""
    if count == 2:
        names = ("近端", "远端")
    elif count == 3:
        names = ("根部", "近端", "远端")
    elif count == 4:
        names = ("外展", "掌指", "近端", "远端")
    else:
        names = ("旋转", "外展", "掌指", "近端", "远端")
    return names[local_index] if local_index < len(names) else f"J{local_index + 1}"


def load_configured_presets(hand: HandState) -> None:
    """只从项目配置目录读取当前 hand side + model 的预设与任务。"""

    raw_config = load_side_config(hand.side)
    model_config = model_config_from(raw_config, hand.model)
    presets: OrderedDict[str, HandPreset] = OrderedDict()
    raw_presets = config_list(model_config.get("presets", []))
    for raw_preset in raw_presets:
        preset_config = config_mapping(raw_preset)
        if not preset_config:
            continue
        name = str(preset_config.get("name") or "Untitled preset")
        preset_id = str(
            preset_config.get("id") or unique_config_id(name, "preset", presets)
        )
        if preset_id in presets:
            continue
        values = config_list(preset_config.get("values", []))
        presets[preset_id] = HandPreset(
            name,
            tuple(normalize_pose(values, hand.joint_count)),
            "Configured",
            preset_id,
        )

    action_sequences: OrderedDict[str, HandActionSequence] = OrderedDict()
    raw_sequences = config_list(model_config.get("action_sequences", []))
    for raw_sequence in raw_sequences:
        sequence_config = config_mapping(raw_sequence)
        if not sequence_config:
            continue
        name = str(sequence_config.get("name") or "Untitled sequence")
        sequence_id = str(
            sequence_config.get("id")
            or unique_config_id(name, "sequence", action_sequences)
        )
        if sequence_id in action_sequences:
            continue
        steps: list[ActionSequenceStep] = []
        for raw_step in config_list(sequence_config.get("steps", [])):
            step_config = config_mapping(raw_step)
            preset_id = str(
                step_config.get("preset_id") or step_config.get("action") or ""
            )
            if preset_id not in presets:
                continue
            try:
                timestamp = normalize_timestamp(step_config.get("timestamp") or 0)
            except ValueError:
                continue
            steps.append(ActionSequenceStep(timestamp, preset_id))
        normalized_steps = normalize_action_sequence_steps(steps, presets)
        if normalized_steps:
            try:
                loop_count = normalize_action_sequence_loop_count(
                    sequence_config.get(
                        "loop_count", DEFAULT_ACTION_SEQUENCE_LOOP_COUNT
                    )
                )
            except (TypeError, ValueError):
                loop_count = DEFAULT_ACTION_SEQUENCE_LOOP_COUNT
            try:
                loop_interval_seconds = normalize_action_sequence_loop_interval_seconds(
                    sequence_config.get(
                        "loop_interval_seconds",
                        sequence_config.get(
                            "loop_interval",
                            DEFAULT_ACTION_SEQUENCE_LOOP_INTERVAL_SECONDS,
                        ),
                    )
                )
            except (TypeError, ValueError):
                loop_interval_seconds = DEFAULT_ACTION_SEQUENCE_LOOP_INTERVAL_SECONDS
            action_sequences[sequence_id] = HandActionSequence(
                sequence_id,
                name,
                tuple(normalized_steps),
                loop_count,
                loop_interval_seconds,
            )

    tasks: OrderedDict[str, HandTask] = OrderedDict()
    raw_tasks = config_list(model_config.get("tasks", []))
    for raw_task in raw_tasks:
        task_config = config_mapping(raw_task)
        if not task_config:
            continue
        name = str(task_config.get("name") or task_config.get("id") or "Untitled task")
        task_id = str(task_config.get("id") or unique_config_id(name, "task", tasks))
        if task_id in tasks:
            continue
        raw_preset_ids = config_list(task_config.get("preset_ids", []))
        preset_ids = [
            str(preset_id) for preset_id in raw_preset_ids if str(preset_id) in presets
        ]
        raw_action_sequence_ids = config_list(
            task_config.get("action_sequence_ids", [])
        )
        action_sequence_ids = [
            str(sequence_id)
            for sequence_id in raw_action_sequence_ids
            if str(sequence_id) in action_sequences
        ]
        tasks[task_id] = HandTask(task_id, name, preset_ids, action_sequence_ids)

    hand.presets = presets
    hand.action_sequences = action_sequences
    hand.tasks = tasks
    if hand.active_group not in hand.tasks:
        hand.active_group = next(iter(hand.tasks), "")
    sync_preset_groups(hand)


def save_configured_presets(hand: HandState) -> None:
    """把当前 hand side + model 的预设与任务写回侧专属 YAML 文件。"""

    path = side_config_path(hand.side)
    raw_config = load_side_config(hand.side)
    models = config_mapping(raw_config.get("models", {}))
    models[hand.model] = {
        "presets": [
            {
                "id": preset_id,
                "name": preset.name,
                "values": normalize_pose(preset.values, hand.joint_count),
            }
            for preset_id, preset in hand.presets.items()
        ],
        "action_sequences": [
            {
                "id": sequence_id,
                "name": sequence.name,
                "loop_count": sequence.loop_count,
                "loop_interval_seconds": sequence.loop_interval_seconds,
                "steps": [
                    {
                        "timestamp": step.timestamp,
                        "preset_id": step.preset_id,
                    }
                    for step in normalize_action_sequence_steps(
                        sequence.steps,
                        hand.presets,
                    )
                ],
            }
            for sequence_id, sequence in hand.action_sequences.items()
        ],
        "tasks": [
            {
                "id": task.id,
                "name": task.name,
                "preset_ids": [
                    preset_id
                    for preset_id in task.preset_ids
                    if preset_id in hand.presets
                ],
                "action_sequence_ids": [
                    sequence_id
                    for sequence_id in task.action_sequence_ids
                    if sequence_id in hand.action_sequences
                ],
            }
            for task in hand.tasks.values()
        ],
    }
    raw_config = {
        "schema_version": 1,
        "side": hand.side,
        "models": models,
    }

    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    yaml = cast(YamlModule, cast(object, import_module("yaml")))
    _ = path.write_text(
        format_preset_values_inline(
            yaml.safe_dump(raw_config, allow_unicode=True, sort_keys=False)
        ),
        encoding="utf-8",
    )


def sync_preset_groups(hand: HandState) -> None:
    """生成当前 UI 仍然使用的 task -> preset 展示视图。"""

    hand.preset_groups = OrderedDict(
        (
            task_id,
            [
                hand.presets[preset_id]
                for preset_id in task.preset_ids
                if preset_id in hand.presets
            ],
        )
        for task_id, task in hand.tasks.items()
    )
    hand.action_sequence_groups = OrderedDict(
        (
            task_id,
            [
                hand.action_sequences[sequence_id]
                for sequence_id in task.action_sequence_ids
                if sequence_id in hand.action_sequences
            ],
        )
        for task_id, task in hand.tasks.items()
    )


def load_side_config(side: str) -> dict[str, object]:
    path = side_config_path(side)
    if not path.exists() or path.stat().st_size == 0:
        return empty_side_config(side)
    yaml = cast(YamlModule, cast(object, import_module("yaml")))
    raw_data = config_mapping(yaml.safe_load(path.read_text(encoding="utf-8")))
    if raw_data.get("side") != side:
        return empty_side_config(side)
    return raw_data


def empty_side_config(side: str) -> dict[str, object]:
    return {"schema_version": 1, "side": side, "models": {}}


def model_config_from(raw_config: dict[str, object], model: str) -> dict[str, object]:
    models = config_mapping(raw_config.get("models", {}))
    return config_mapping(models.get(model, {}))


def side_config_path(side: str) -> Path:
    return CONFIG_DIR / SIDE_CONFIG_FILES[side]


def unique_config_id(name: str, fallback: str, existing: Iterable[str]) -> str:
    existing_ids = set(existing)
    stem = "-".join(part for part in slug_parts(name) if part) or fallback
    candidate = stem
    suffix = 2
    while candidate in existing_ids:
        candidate = f"{stem}-{suffix}"
        suffix += 1
    return candidate


def config_mapping(value: object) -> dict[str, object]:
    if not isinstance(value, dict):
        return {}
    mapping = cast(dict[object, object], value)
    return {str(key): item for key, item in mapping.items()}


def config_list(value: object) -> list[object]:
    if not isinstance(value, list):
        return []
    return list(cast(list[object], value))


def format_preset_values_inline(yaml_text: str) -> str:
    """Render preset `values` as one-line arrays while leaving other lists readable."""

    lines = yaml_text.splitlines()
    formatted: list[str] = []
    index = 0
    while index < len(lines):
        line = lines[index]
        stripped = line.lstrip()
        indent = line[: len(line) - len(stripped)]
        if stripped == "values:":
            values: list[str] = []
            probe = index + 1
            while probe < len(lines):
                value_line = lines[probe]
                value_stripped = value_line.lstrip()
                value_indent = value_line[: len(value_line) - len(value_stripped)]
                if value_indent != indent or not value_stripped.startswith("- "):
                    break
                values.append(value_stripped[2:].strip())
                probe += 1
            if values:
                formatted.append(f"{indent}values: [{', '.join(values)}]")
                index = probe
                continue
        formatted.append(line)
        index += 1
    return "\n".join(formatted) + "\n"


def slug_parts(value: str) -> list[str]:
    parts: list[str] = []
    current: list[str] = []
    for character in value.strip().lower():
        if character.isalnum():
            current.append(character)
        elif current:
            parts.append("".join(current))
            current.clear()
    if current:
        parts.append("".join(current))
    return parts
