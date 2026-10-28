"""NiceGUI dual-hand control interface."""

from __future__ import annotations

import time
from collections.abc import Callable, Sequence
from importlib import import_module
from typing import Any

from barmate.hardware.linker_hand.control.controller import ManualHandController
from barmate.hardware.linker_hand.control.playback import load_hand_playback_records
from barmate.hardware.linker_hand.control.state import (
    ActionSequenceStep,
    DEFAULT_ACTION_SEQUENCE_LOOP_COUNT,
    DEFAULT_ACTION_SEQUENCE_LOOP_INTERVAL_SECONDS,
    DEFAULT_MODEL,
    HAND_LABELS,
    HAND_SIDES,
    HandActionSequence,
    MAX_CONTROL_VALUE,
    MIN_CONTROL_VALUE,
    ControlPageState,
    FingerSpec,
    HandState,
    RuntimeConfigCommandFilter,
    add_configured_task,
    add_task_item,
    clamp_control_value,
    emergency_stop_values,
    normalize_pose,
    remove_task_item,
    save_configured_action_sequence,
    save_configured_preset,
)
from barmate.hardware.linker_hand.control.styles import install_styles
from barmate.hardware.linker_hand.control.types import (
    TACTILE_DISPLAY_NAMES,
    TactileFrame,
    TactileMatrix,
    tactile_frame_peak,
    tactile_frame_total,
)
from barmate.hardware.linker_hand.linker_hand_sdk import MODEL_JOINT_COUNTS


class NiceGuiProxy:
    """Delay NiceGUI imports so state logic remains importable in tests."""

    def __getattr__(self, name: str) -> Any:
        return getattr(self._load(), name)

    def page(self, route: str) -> Any:
        try:
            return self._load().page(route)
        except ModuleNotFoundError:
            return lambda function: function

    def _load(self) -> Any:
        return getattr(import_module("nicegui"), "ui")


ui = NiceGuiProxy()

FINGER_DISPLAY_NAMES = {
    "拇指": "Thumb",
    "食指": "Index",
    "中指": "Middle",
    "无名指": "Ring",
    "小指": "Pinky",
}

JOINT_DISPLAY_NAMES = {
    "旋转": "Rot",
    "外展": "Abd",
    "根部": "MCP",
    "掌指": "MCP",
    "近端": "Prox",
    "远端": "Dist",
}

HAND_DISPLAY_NAMES = {"LEFT_HAND": "Left hand", "RIGHT_HAND": "Right hand"}
HAND_PORTS = {"LEFT_HAND": "can0", "RIGHT_HAND": "can1"}
HAND_ICONS = {"LEFT_HAND": "front_hand", "RIGHT_HAND": "back_hand"}
RUNTIME_CONFIG_COMMAND_INTERVAL_SECONDS = 0.2
TACTILE_REFRESH_INTERVAL_SECONDS = 0.2
_runtime_controller: ManualHandController | None = None


class _CompatValueControl:
    """Small headless control used by legacy tests and imports."""

    def __init__(
        self, value: int, on_change: Callable[[int], None] | None = None
    ) -> None:
        self.value = value
        self._on_change = on_change

    def set_value(self, value: object) -> None:
        self.value = clamp_control_value(value)
        if self._on_change is not None:
            self._on_change(self.value)


class LinkerHandControlWindow:
    """Import-compatible page object with lightweight headless controls."""

    def __init__(
        self,
        controller: ManualHandController | None = None,
        *_,
        model: str = DEFAULT_MODEL,
        **__,
    ) -> None:
        self.state = ControlPageState(model=model)
        self.controller = controller
        self.sliders: dict[tuple[str, int], _CompatValueControl] = {}
        self.speed_input: _CompatValueControl | None = None
        self.torque_input: _CompatValueControl | None = None
        if controller is not None:
            self._install_legacy_controls(controller)

    def _install_legacy_controls(self, controller: ManualHandController) -> None:
        for hand_name in controller.hand_names:
            for index, value in enumerate(controller.position(hand_name)):
                self.sliders[(hand_name, index)] = _CompatValueControl(
                    value,
                    lambda new_value, selected_hand=hand_name, selected_index=index: (
                        self._set_legacy_joint(
                            controller, selected_hand, selected_index, new_value
                        )
                    ),
                )
        self.speed_input = _CompatValueControl(
            controller.speed(controller.selected_hand_name)[0]
        )
        self.torque_input = _CompatValueControl(
            controller.torque(controller.selected_hand_name)[0]
        )

    def _set_legacy_joint(
        self,
        controller: ManualHandController,
        hand_name: str,
        index: int,
        value: int,
    ) -> None:
        values = controller.position(hand_name)
        values[index] = value
        controller.set_hand_position(hand_name, values)

    def apply_selected_speed(self) -> None:
        if self.controller is None or self.speed_input is None:
            return
        width = self.controller.joint_count(self.controller.selected_hand_name)
        self.controller.set_selected_hand_speed([self.speed_input.value] * width)

    def apply_selected_torque(self) -> None:
        if self.controller is None or self.torque_input is None:
            return
        width = self.controller.joint_count(self.controller.selected_hand_name)
        self.controller.set_selected_hand_torque([self.torque_input.value] * width)

    def shutdown(self) -> None:
        self.sliders.clear()


def set_label_text(label: Any, text: str) -> None:
    label.set_text(text)


def display_group_name(group: str, hand: HandState | None = None) -> str:
    if hand is not None:
        return hand.task_label(group)
    if group == "基础动作":
        return "Basic"
    return group


def format_sequence_step_label(step: ActionSequenceStep, hand: HandState) -> str:
    preset = hand.presets.get(step.preset_id)
    preset_name = preset.name if preset is not None else step.preset_id
    return f"{step.timestamp:.3f}s → {preset_name}"


def format_sequence_loop_label(sequence: HandActionSequence) -> str:
    loop_text = "∞" if sequence.loop_count == -1 else str(sequence.loop_count)
    return f"loops={loop_text}, interval={sequence.loop_interval_seconds:.3f}s"


def display_joint_name(joint_name: str) -> str:
    if "·" not in joint_name:
        return joint_name
    _, role = (part.strip() for part in joint_name.split("·", 1))
    return JOINT_DISPLAY_NAMES.get(role, role)


def tactile_cell_style(value: float, peak: float) -> str:
    intensity = 0.0 if peak <= 0.0 else max(0.0, min(1.0, value / peak))
    hue = 196.0 - 154.0 * intensity
    lightness = 93.0 - 38.0 * intensity
    alpha = 0.58 + 0.34 * intensity
    return (
        f"background: hsla({hue:.0f}, 76%, {lightness:.0f}%, {alpha:.2f}); "
        f"border-color: rgba(31, 42, 37, {0.06 + 0.14 * intensity:.2f});"
    )


def tactile_matrix_summary(matrix: TactileMatrix) -> str:
    total = sum(max(0.0, value) for row in matrix for value in row)
    peak = max((value for row in matrix for value in row), default=0.0)
    return f"{total:.0f} / {peak:.0f}"


def set_runtime_controller(controller: ManualHandController | None) -> None:
    global _runtime_controller

    _runtime_controller = controller


def controller_hand_name(
    side: str, controller: ManualHandController | None
) -> str | None:
    if controller is None:
        return None
    hand_name = "left" if side == "LEFT_HAND" else "right"
    return hand_name if hand_name in controller.hand_names else None


def controller_hand_model(hand: object) -> str | None:
    model = getattr(hand, "hand_joint", None)
    if isinstance(model, str) and model in MODEL_JOINT_COUNTS:
        return model
    config = getattr(hand, "config", None)
    config_model = getattr(config, "model", None)
    if isinstance(config_model, str) and config_model in MODEL_JOINT_COUNTS:
        return config_model
    return None


def controller_hand_port(hand: object, fallback: str) -> str:
    can = getattr(hand, "can", None)
    if isinstance(can, str) and can:
        return can
    config = getattr(hand, "config", None)
    config_can = getattr(config, "can", None)
    return config_can if isinstance(config_can, str) and config_can else fallback


def create_finger(
    finger: FingerSpec,
    hand: HandState,
    sliders: dict[tuple[str, int], Any],
    value_labels: dict[tuple[str, int], Any],
    on_control_change: Any,
) -> None:
    """Render one finger lane and its joint controls."""

    with ui.element("div").classes("finger-card gap-3"):
        ui.label(FINGER_DISPLAY_NAMES.get(finger.name, finger.name.upper())).classes(
            "finger-name"
        )
        for joint in finger.joints:
            key = (hand.side, joint.index)
            with ui.column().classes("joint-row w-full gap-1"):
                with ui.row().classes(
                    "w-full items-center justify-between gap-1 no-wrap"
                ):
                    ui.label(display_joint_name(joint.name)).classes("joint-title")
                    value_labels[key] = ui.label(str(hand.values[joint.index])).classes(
                        "joint-value"
                    )
                with ui.row().classes(
                    "joint-control-row w-full items-center gap-1 no-wrap"
                ):
                    ui.button(
                        icon="remove",
                        on_click=lambda side=hand.side, index=joint.index: (
                            on_control_change(side, index, hand.values[index] - 1)
                        ),
                    ).props("flat dense round").classes("joint-step-button")
                    ui.tooltip("Decrease")
                    slider = (
                        ui.slider(
                            min=MIN_CONTROL_VALUE,
                            max=MAX_CONTROL_VALUE,
                            step=1,
                            value=hand.values[joint.index],
                        )
                        .props("dense color=green thumb-size=16px track-size=3px")
                        .classes("compact-slider flex-1")
                    )
                    ui.button(
                        icon="add",
                        on_click=lambda side=hand.side, index=joint.index: (
                            on_control_change(side, index, hand.values[index] + 1)
                        ),
                    ).props("flat dense round").classes("joint-step-button")
                    ui.tooltip("Increase")
                slider.on_value_change(
                    lambda event, side=hand.side, index=joint.index: on_control_change(
                        side, index, event.value
                    )
                )
                sliders[key] = slider


@ui.page("/")
def linker_hand_page() -> None:
    """Build the dual-hand control page and trajectory controls."""

    install_styles(ui)
    controller = _runtime_controller
    state = ControlPageState()
    sliders: dict[tuple[str, int], Any] = {}
    value_labels: dict[tuple[str, int], Any] = {}
    runtime_sliders: dict[tuple[str, str], Any] = {}
    runtime_value_labels: dict[tuple[str, str], Any] = {}
    runtime_config_filter = RuntimeConfigCommandFilter(
        RUNTIME_CONFIG_COMMAND_INTERVAL_SECONDS
    )
    syncing = False
    playback_started_at = 0.0
    playback_index = 0
    action_sequence_playbacks: dict[str, dict[str, object]] = {}
    tactile_frames: dict[str, TactileFrame] = {side: {} for side in HAND_SIDES}
    status_label: Any | None = None

    def current_models_text() -> str:
        return (
            f"Left {state.hands['LEFT_HAND'].model} "
            f"/ Right {state.hands['RIGHT_HAND'].model}"
        )

    def tactile_status_text(side: str) -> str:
        frame = tactile_frames.get(side, {})
        hand = state.hands[side]
        hand_name = controller_hand_name(side, controller)
        if frame:
            return (
                f"Total {tactile_frame_total(frame):.0f} "
                f"· Peak {tactile_frame_peak(frame):.0f}"
            )
        if (
            controller is not None
            and hand_name is not None
            and controller.has_tactile(hand_name)
        ):
            return "Waiting"
        if hand.model == "G20":
            return "No data"
        return "Unsupported"

    def controller_hand_for_side(side: str) -> object | None:
        hand_name = controller_hand_name(side, controller)
        if controller is None or hand_name is None:
            return None
        for slot in controller.slots:
            if slot.name == hand_name:
                return slot.hand
        return None

    def hand_port_text(side: str) -> str:
        hand = controller_hand_for_side(side)
        return (
            controller_hand_port(hand, HAND_PORTS[side])
            if hand is not None
            else HAND_PORTS[side]
        )

    def command_timestamp() -> str:
        return time.strftime("%H:%M:%S")

    def append_command(command: str) -> None:
        state.append_command(f"{command_timestamp()}  {command}")
        command_log_console.refresh()

    def dispatch_pose_to_controller(side: str, source: str) -> bool:
        hand_name = controller_hand_name(side, controller)
        hand = state.hands[side]
        command = (
            f"{source} {side} model={hand.model} SEND_ACTION action_joint={hand.values}"
        )
        if controller is None or hand_name is None:
            append_command(command)
            return True
        try:
            result = controller.set_hand_position(hand_name, hand.values)
        except Exception as exc:
            state.status = f"{hand.label}: command failed ({exc})"
            append_command(f"{command} FAILED error={exc!r}")
            ui.notify(state.status, type="negative")
            return False
        append_command(f"{command} result={result}")
        return True

    def dispatch_all_poses_to_controller(source: str) -> bool:
        if controller is None:
            left = state.hands["LEFT_HAND"]
            right = state.hands["RIGHT_HAND"]
            append_command(
                f"{source} LEFT_HAND SEND_ACTION action_joint={left.values} "
                f"RIGHT_HAND SEND_ACTION action_joint={right.values}"
            )
            return True
        positions: dict[str, Sequence[int | float]] = {}
        for side in HAND_SIDES:
            hand_name = controller_hand_name(side, controller)
            if hand_name is not None:
                positions[hand_name] = state.hands[side].values
        try:
            result = controller.apply_hand_positions(positions)
        except Exception as exc:
            state.status = f"Playback command failed: {exc}"
            append_command(f"{source} FAILED error={exc!r}")
            ui.notify(state.status, type="negative")
            return False
        append_command(f"{source} result={result}")
        return True

    def runtime_config_payload(side: str, kind: str, value: int) -> list[int]:
        hand_name = controller_hand_name(side, controller)
        if controller is None or hand_name is None:
            return [value] * 5
        current = (
            controller.speed(hand_name)
            if kind == "speed"
            else controller.torque(hand_name)
        )
        width = len(current) if current else controller.joint_count(hand_name)
        return [value] * width

    def dispatch_runtime_config_to_controller(side: str, kind: str, value: int) -> bool:
        hand = state.hands[side]
        payload = runtime_config_payload(side, kind, value)
        command = f"SET_{kind.upper()} {side} values={payload}"
        hand_name = controller_hand_name(side, controller)
        if controller is None or hand_name is None:
            append_command(command)
            return True
        try:
            result = (
                controller.set_hand_speed(hand_name, payload)
                if kind == "speed"
                else controller.set_hand_torque(hand_name, payload)
            )
        except Exception as exc:
            state.status = f"{hand.label}: {kind} command failed ({exc})"
            append_command(f"{command} FAILED error={exc!r}")
            ui.notify(state.status, type="negative")
            return False
        append_command(f"{command} result={result}")
        return True

    def initialize_from_controller() -> None:
        if controller is None:
            return
        for side in HAND_SIDES:
            hand_name = controller_hand_name(side, controller)
            hand = state.hands[side]
            controller_hand = controller_hand_for_side(side)
            model = (
                controller_hand_model(controller_hand)
                if controller_hand is not None
                else None
            )
            if model is not None and model != hand.model:
                state.rebuild_hand_for_model(side, model)
                hand = state.hands[side]
            if hand_name is not None:
                hand.values[:] = normalize_pose(
                    controller.position(hand_name), hand.joint_count
                )
                speed = controller.speed(hand_name)
                torque = controller.torque(hand_name)
                if speed:
                    hand.speed = clamp_control_value(speed[0])
                if torque:
                    hand.torque = clamp_control_value(torque[0])
        state.status = "Controller connected"

    def sync_controls() -> None:
        """Synchronize state values into sliders and labels."""

        nonlocal syncing
        syncing = True
        try:
            for side, hand in state.hands.items():
                for index, value in enumerate(hand.values):
                    key = (side, index)
                    if key in sliders:
                        sliders[key].set_value(value)
                    if key in value_labels:
                        set_label_text(value_labels[key], str(value))
                for kind, value in (("speed", hand.speed), ("torque", hand.torque)):
                    runtime_key = (side, kind)
                    if runtime_key in runtime_sliders:
                        runtime_sliders[runtime_key].set_value(value)
                    if runtime_key in runtime_value_labels:
                        set_label_text(runtime_value_labels[runtime_key], str(value))
        finally:
            syncing = False
        if status_label is not None:
            status_label.set_text(state.status)
        model_badge.set_text(current_models_text())
        playback_badge.set_text(
            f"{len(state.playback_frames)} frames"
            + (" · Playing" if state.is_playing else "")
        )

    def apply_pose_to_side(
        side: str, values: Sequence[object], action_name: str, *, source: str
    ) -> None:
        hand = state.hands[side]
        hand.values[:] = normalize_pose(values, hand.joint_count)
        state.status = f"{hand.label}: {action_name}"
        dispatch_pose_to_controller(side, source)
        sync_controls()

    def emergency_stop(side: str) -> None:
        stopped_sequence = action_sequence_playbacks.pop(side, None)
        if stopped_sequence is not None:
            sequence = stopped_sequence.get("sequence")
            sequence_id = (
                sequence.id if isinstance(sequence, HandActionSequence) else ""
            )
            append_command(
                f"ACTION_SEQUENCE_STOP {side} sequence={sequence_id!r} reason='emergency_stop'"
            )
        hand = state.hands[side]
        hand.values[:] = emergency_stop_values(hand)
        state.status = f"{hand.label}: stopped"
        dispatch_pose_to_controller(side, "EMERGENCY_STOP")
        sync_controls()

    def on_control_change(side: str, index: int, value: object) -> None:
        """Update a joint value and record the command."""

        if syncing:
            return
        hand = state.hands[side]
        hand.values[index] = clamp_control_value(value)
        set_label_text(value_labels[(side, index)], str(hand.values[index]))
        state.status = f"{HAND_LABELS[side]} J{index + 1:02d} = {hand.values[index]}"
        dispatch_pose_to_controller(
            side, f"SET_JOINT joint=J{index + 1:02d} value={hand.values[index]}"
        )
        sync_controls()

    def on_runtime_config_change(side: str, kind: str, value: object) -> None:
        if syncing:
            return
        hand = state.hands[side]
        normalized = clamp_control_value(value)
        if kind == "speed":
            hand.speed = normalized
            label = "Speed"
        else:
            hand.torque = normalized
            label = "Torque"
        key = (side, kind)
        if key in runtime_value_labels:
            set_label_text(runtime_value_labels[key], str(normalized))
        state.status = f"{hand.label}: {label} = {normalized}"
        if runtime_config_filter.should_send(
            side, kind, normalized, time.perf_counter()
        ):
            dispatch_runtime_config_to_controller(side, kind, normalized)
        sync_controls()

    def flush_runtime_config_commands() -> None:
        flushed = False
        for side, kind, value in runtime_config_filter.pop_due(time.perf_counter()):
            dispatch_runtime_config_to_controller(side, kind, value)
            flushed = True
        if flushed:
            sync_controls()

    def on_hand_model_change(side: str, event: Any) -> None:
        if (
            controller is not None
            and controller_hand_name(side, controller) is not None
        ):
            ui.notify(
                "This hand is bound to hardware, so the model comes from device configuration.",
                type="warning",
            )
            sync_controls()
            return
        state.rebuild_hand_for_model(side, str(event.value))
        append_command(f"SET_MODEL {side} model={state.hands[side].model}")
        sliders.clear()
        value_labels.clear()
        hands_area.refresh()
        sync_controls()

    def add_task_group(side: str, task_name_input: Any) -> None:
        hand = state.hands[side]
        name = hand.new_task_name.strip()
        if not name:
            ui.notify("Enter a task name", type="warning")
            return
        task = add_configured_task(hand, name)
        hand.new_task_name = ""
        state.status = f"{hand.label}: added task {task.name}"
        task_name_input.set_value("")
        append_command(f"CREATE_TASK {side} task={task.id!r} name={task.name!r}")
        hands_area.refresh()
        sync_controls()

    def save_current_preset(side: str, preset_name_input: Any) -> None:
        hand = state.hands[side]
        if hand.active_group not in hand.tasks:
            ui.notify("Create a task first", type="warning")
            return
        preset = save_configured_preset(hand, hand.new_preset_name)
        hand.new_preset_name = ""
        state.status = f"{hand.label}: saved {preset.name}"
        preset_name_input.set_value("")
        append_command(
            f"SAVE_PRESET {side} task={hand.active_group!r} name={preset.name!r} values={hand.values}"
        )
        hands_area.refresh()
        sync_controls()

    def open_action_sequence_dialog(
        side: str,
        sequence: HandActionSequence | None = None,
    ) -> None:
        hand = state.hands[side]
        if hand.active_group not in hand.tasks:
            ui.notify("Create a task first", type="warning")
            return
        if not hand.presets:
            ui.notify("Save or configure a preset first", type="warning")
            return

        step_controls: list[dict[str, Any]] = []
        preset_options = {
            preset_id: preset.name for preset_id, preset in hand.presets.items()
        }
        default_preset_id = next(iter(hand.presets))

        with ui.dialog() as dialog, ui.card().classes("gap-3 min-w-[520px]"):
            ui.label("Action sequence").classes("section-title")
            name_input = (
                ui.input(
                    "Sequence Name",
                    value=sequence.name if sequence is not None else "",
                    placeholder="lemon-grasp-sequence",
                )
                .props("outlined dense clearable")
                .classes("w-full")
            )
            with ui.row().classes("w-full gap-2 no-wrap"):
                loop_count_input = (
                    ui.number(
                        "Loop Count (-1 = forever)",
                        value=(
                            sequence.loop_count
                            if sequence is not None
                            else DEFAULT_ACTION_SEQUENCE_LOOP_COUNT
                        ),
                        step=1,
                    )
                    .props("outlined dense")
                    .classes("flex-1")
                )
                loop_interval_input = (
                    ui.number(
                        "Loop Interval (s)",
                        value=(
                            sequence.loop_interval_seconds
                            if sequence is not None
                            else DEFAULT_ACTION_SEQUENCE_LOOP_INTERVAL_SECONDS
                        ),
                        min=0,
                        step=0.1,
                    )
                    .props("outlined dense")
                    .classes("flex-1")
                )
            rows = ui.column().classes("w-full gap-2")

            def add_step_row(
                timestamp: float = 0.0,
                preset_id: str = default_preset_id,
            ) -> None:
                row_state: dict[str, Any] = {"deleted": False}
                with rows:
                    with ui.row().classes("w-full items-center gap-2 no-wrap") as row:
                        time_input = (
                            ui.number(
                                "Time from 0s",
                                value=timestamp,
                                min=0,
                                step=0.1,
                            )
                            .props("outlined dense")
                            .classes("w-36")
                        )
                        preset_select = (
                            ui.select(
                                preset_options,
                                value=preset_id
                                if preset_id in hand.presets
                                else default_preset_id,
                                label="Preset",
                            )
                            .props("outlined dense options-dense")
                            .classes("flex-1")
                        )
                        ui.button(
                            icon="delete",
                            on_click=lambda selected=row_state: remove_step_row(
                                selected
                            ),
                        ).props("flat dense round color=red")
                row_state.update(
                    {"row": row, "time": time_input, "preset": preset_select}
                )
                step_controls.append(row_state)

            def remove_step_row(row_state: dict[str, Any]) -> None:
                row_state["deleted"] = True
                row_state["row"].delete()

            for step in (
                sequence.steps
                if sequence is not None
                else (ActionSequenceStep(0.0, default_preset_id),)
            ):
                add_step_row(step.timestamp, step.preset_id)

            with ui.row().classes("w-full justify-between items-center"):
                ui.button(
                    "Add Step",
                    icon="add",
                    on_click=lambda: add_step_row(),
                ).props("outline color=cyan no-caps").classes("soft-button")
                with ui.row().classes("gap-2"):
                    ui.button("Cancel", on_click=dialog.close).props(
                        "flat color=blue-grey no-caps"
                    )

                    def save_sequence() -> None:
                        steps: list[ActionSequenceStep] = []
                        for row_state in step_controls:
                            if row_state["deleted"]:
                                continue
                            preset_id = str(row_state["preset"].value or "")
                            try:
                                timestamp = float(str(row_state["time"].value or 0))
                            except ValueError:
                                ui.notify("Time must be numeric", type="warning")
                                return
                            steps.append(ActionSequenceStep(timestamp, preset_id))
                        try:
                            saved = save_configured_action_sequence(
                                hand,
                                str(name_input.value or ""),
                                steps,
                                sequence.id if sequence is not None else None,
                                loop_count=loop_count_input.value,
                                loop_interval_seconds=loop_interval_input.value,
                            )
                        except ValueError as exc:
                            ui.notify(str(exc), type="warning")
                            return
                        state.status = f"{hand.label}: saved sequence {saved.name}"
                        append_command(
                            f"SAVE_ACTION_SEQUENCE {side} task={hand.active_group!r} "
                            + f"sequence={saved.id!r} steps={len(saved.steps)}"
                        )
                        dialog.close()
                        hands_area.refresh()
                        sync_controls()

                    ui.button("Save", icon="save", on_click=save_sequence).props(
                        "outline color=cyan no-caps"
                    ).classes("soft-button")
        dialog.open()

    def open_task_action_picker(side: str) -> None:
        hand = state.hands[side]
        task_id = hand.active_group
        task = hand.tasks.get(task_id)
        if task is None:
            ui.notify("Choose a task first", type="warning")
            return
        available_presets = [
            preset
            for preset_id, preset in hand.presets.items()
            if preset_id not in task.preset_ids
        ]
        available_sequences = [
            sequence
            for sequence_id, sequence in hand.action_sequences.items()
            if sequence_id not in task.action_sequence_ids
        ]

        def add_item(item_kind: str, item_id: str, item_name: str) -> None:
            try:
                changed = add_task_item(hand, task_id, item_kind, item_id)
            except ValueError as exc:
                ui.notify(str(exc), type="warning")
                return
            if changed:
                state.status = f"{hand.label}: added {item_name}"
                append_command(
                    f"ADD_TASK_ITEM {side} task={task_id!r} kind={item_kind!r} item={item_id!r}"
                )
            dialog.close()
            hands_area.refresh()
            sync_controls()

        with ui.dialog() as dialog, ui.card().classes("task-picker gap-3"):
            ui.label(f"Add to {hand.task_label(task_id)}").classes("section-title")
            if not available_presets and not available_sequences:
                ui.label("No available actions").classes("section-title")
            if available_presets:
                ui.label("Presets").classes("section-title")
                with ui.element("div").classes("picker-grid w-full"):
                    for preset in available_presets:
                        ui.button(
                            preset.name,
                            icon="add",
                            on_click=lambda selected=preset: add_item(
                                "preset", selected.id, selected.name
                            ),
                        ).props("outline color=cyan no-caps").classes(
                            "soft-button action-chip"
                        )
            if available_sequences:
                ui.label("Action sequences").classes("section-title")
                with ui.element("div").classes("picker-grid w-full"):
                    for sequence in available_sequences:
                        ui.button(
                            sequence.name,
                            icon="playlist_add",
                            on_click=lambda selected=sequence: add_item(
                                "sequence", selected.id, selected.name
                            ),
                        ).props("outline color=purple no-caps").classes(
                            "soft-button action-chip"
                        )
            with ui.row().classes("w-full justify-end"):
                ui.button("Cancel", on_click=dialog.close).props(
                    "flat color=blue-grey no-caps"
                )
        dialog.open()

    def remove_action_from_task(
        side: str,
        item_kind: str,
        item_id: str,
        item_name: str,
    ) -> None:
        hand = state.hands[side]
        task_id = hand.active_group
        try:
            changed = remove_task_item(hand, task_id, item_kind, item_id)
        except ValueError as exc:
            ui.notify(str(exc), type="warning")
            return
        if changed:
            state.status = f"{hand.label}: removed {item_name}"
            append_command(
                f"REMOVE_TASK_ITEM {side} task={task_id!r} kind={item_kind!r} item={item_id!r}"
            )
            hands_area.refresh()
            sync_controls()

    def start_action_sequence(side: str, sequence: HandActionSequence) -> None:
        if not sequence.steps:
            ui.notify("Action sequence has no steps", type="warning")
            return
        hand = state.hands[side]
        action_sequence_playbacks[side] = {
            "sequence": sequence,
            "started_at": time.perf_counter(),
            "index": 0,
            "completed_loops": 0,
        }
        state.status = f"{hand.label}: playing sequence {sequence.name}"
        append_command(
            f"ACTION_SEQUENCE_START {side} sequence={sequence.id!r} steps={len(sequence.steps)} "
            + f"loop_count={sequence.loop_count} loop_interval_seconds={sequence.loop_interval_seconds:.3f}"
        )
        sync_controls()

    def stop_action_sequence(side: str) -> None:
        playback = action_sequence_playbacks.get(side)
        if playback is None:
            return
        sequence = playback.get("sequence")
        sequence_name = (
            sequence.name
            if isinstance(sequence, HandActionSequence)
            else "Unknown sequence"
        )
        sequence_id = sequence.id if isinstance(sequence, HandActionSequence) else ""
        started_at = playback.get("started_at")
        step_index = playback.get("index")
        completed_loops = playback.get("completed_loops")
        if (
            isinstance(started_at, float)
            and isinstance(step_index, int)
            and isinstance(completed_loops, int)
            and completed_loops > 0
            and step_index == 0
            and playback.get("awaiting_next_loop")
        ):
            del action_sequence_playbacks[side]
            state.status = f"{state.hands[side].label}: stopped {sequence_name}"
            append_command(
                f"ACTION_SEQUENCE_STOP {side} sequence={sequence_id!r} reason='between_loops'"
            )
            sync_controls()
            return
        if playback.get("stop_after_current_loop"):
            return
        playback["stop_after_current_loop"] = True
        state.status = (
            f"{state.hands[side].label}: {sequence_name} will stop after this loop"
        )
        append_command(
            f"ACTION_SEQUENCE_STOP_REQUESTED {side} sequence={sequence_id!r}"
        )
        sync_controls()

    def action_sequence_tick() -> None:
        for side, playback in list(action_sequence_playbacks.items()):
            sequence = playback["sequence"]
            if not isinstance(sequence, HandActionSequence):
                del action_sequence_playbacks[side]
                continue
            hand = state.hands[side]
            started_at = playback["started_at"]
            step_index = playback["index"]
            completed_loops = playback.get("completed_loops")
            stop_after_current_loop = bool(playback.get("stop_after_current_loop"))
            if (
                not isinstance(started_at, float)
                or not isinstance(step_index, int)
                or not isinstance(completed_loops, int)
            ):
                del action_sequence_playbacks[side]
                continue
            now = time.perf_counter()
            elapsed = now - started_at
            index = step_index
            changed = False
            if playback.get("awaiting_next_loop") and elapsed >= 0.0:
                playback["awaiting_next_loop"] = False
            while (
                index < len(sequence.steps)
                and sequence.steps[index].timestamp <= elapsed
            ):
                step = sequence.steps[index]
                preset = hand.presets.get(step.preset_id)
                if preset is not None:
                    hand.values[:] = normalize_pose(preset.values, hand.joint_count)
                    state.status = f"{hand.label}: {sequence.name} · {preset.name}"
                    dispatch_pose_to_controller(
                        side, f"ACTION_SEQUENCE_STEP t={step.timestamp:.3f}"
                    )
                    changed = True
                index += 1
            playback["index"] = index
            if index >= len(sequence.steps):
                completed_loops += 1
                should_continue = not stop_after_current_loop and (
                    sequence.loop_count == -1 or completed_loops < sequence.loop_count
                )
                if should_continue:
                    playback["started_at"] = now + sequence.loop_interval_seconds
                    playback["index"] = 0
                    playback["completed_loops"] = completed_loops
                    playback["awaiting_next_loop"] = True
                    next_loop = completed_loops + 1
                    loop_text = (
                        "∞" if sequence.loop_count == -1 else str(sequence.loop_count)
                    )
                    state.status = (
                        f"{hand.label}: {sequence.name} loop {next_loop}/{loop_text}"
                    )
                    append_command(
                        f"ACTION_SEQUENCE_LOOP {side} sequence={sequence.id!r} "
                        + f"completed_loops={completed_loops} next_loop={next_loop}"
                    )
                else:
                    del action_sequence_playbacks[side]
                    state.status = (
                        f"{hand.label}: stopped sequence {sequence.name}"
                        if stop_after_current_loop
                        else f"{hand.label}: completed sequence {sequence.name}"
                    )
                    reason = " stop_requested=True" if stop_after_current_loop else ""
                    append_command(
                        f"ACTION_SEQUENCE_DONE {side} sequence={sequence.id!r} loops={completed_loops}"
                        + reason
                    )
                changed = True
            if changed:
                sync_controls()

    def load_trajectory() -> None:
        path = state.trajectory_path.strip()
        if not path:
            ui.notify("Enter a trajectory path", type="warning")
            return
        try:
            frames = load_hand_playback_records(
                path,
                left_joint_count=state.hands["LEFT_HAND"].joint_count,
                right_joint_count=state.hands["RIGHT_HAND"].joint_count,
            )
        except (OSError, ValueError) as exc:
            state.status = f"Load failed: {exc}"
            append_command(f"LOAD_TRAJECTORY_FAILED path={path!r} error={exc!r}")
            sync_controls()
            ui.notify(state.status, type="negative")
            return
        state.playback_frames = frames
        state.is_playing = False
        state.status = f"Loaded {len(frames)} frames"
        append_command(f"LOAD_TRAJECTORY path={path!r} frames={len(frames)}")
        sync_controls()

    def start_playback() -> None:
        nonlocal playback_started_at, playback_index
        if not state.playback_frames:
            ui.notify("Load a trajectory first", type="warning")
            return
        playback_started_at = time.perf_counter()
        playback_index = 0
        state.is_playing = True
        state.status = f"Playing {len(state.playback_frames)} frames"
        append_command(f"PLAYBACK_START frames={len(state.playback_frames)}")
        sync_controls()

    def stop_playback() -> None:
        state.is_playing = False
        state.status = "Stopped"
        append_command("PLAYBACK_STOP")
        sync_controls()

    def playback_tick() -> None:
        nonlocal playback_index
        if not state.is_playing:
            return
        elapsed = time.perf_counter() - playback_started_at
        changed = False
        while (
            playback_index < len(state.playback_frames)
            and state.playback_frames[playback_index].timestamp <= elapsed
        ):
            frame = state.playback_frames[playback_index]
            state.hands["LEFT_HAND"].values[:] = normalize_pose(
                frame.left, state.hands["LEFT_HAND"].joint_count
            )
            state.hands["RIGHT_HAND"].values[:] = normalize_pose(
                frame.right, state.hands["RIGHT_HAND"].joint_count
            )
            dispatch_all_poses_to_controller(f"PLAYBACK_FRAME t={frame.timestamp:.3f}")
            playback_index += 1
            changed = True
        if playback_index >= len(state.playback_frames):
            state.is_playing = False
            state.status = "Playback complete"
            append_command("PLAYBACK_DONE")
            changed = True
        if changed:
            sync_controls()

    def clear_command_log() -> None:
        state.command_log.clear()
        command_log_console.refresh()

    def update_tactile_frames() -> None:
        changed = False
        for side in HAND_SIDES:
            hand_name = controller_hand_name(side, controller)
            frame: TactileFrame = {}
            if controller is not None and hand_name is not None:
                try:
                    frame = controller.tactile(hand_name)
                except Exception as exc:
                    state.status = (
                        f"{state.hands[side].label}: tactile read failed ({exc})"
                    )
                    frame = {}
            if tactile_frames.get(side) != frame:
                tactile_frames[side] = frame
                changed = True
        if changed:
            tactile_area.refresh()
            if status_label is not None:
                status_label.set_text(state.status)

    @ui.refreshable
    def tactile_area(hand: HandState) -> None:
        frame = tactile_frames.get(hand.side, {})
        peak = tactile_frame_peak(frame)
        with ui.element("section").classes("tactile-card w-full gap-3"):
            with ui.row().classes("w-full items-center justify-between no-wrap"):
                ui.label("Touch").classes("section-title")
                ui.label(tactile_status_text(hand.side)).classes("tactile-summary")
            if not frame:
                with ui.element("div").classes("tactile-empty w-full"):
                    ui.icon("sensors").classes("runtime-icon")
                    ui.label(tactile_status_text(hand.side)).classes("section-title")
                return
            with ui.element("div").classes("tactile-grid"):
                for key, matrix in frame.items():
                    with ui.element("div").classes("tactile-finger"):
                        with ui.row().classes(
                            "w-full items-center justify-between no-wrap"
                        ):
                            ui.label(
                                TACTILE_DISPLAY_NAMES.get(key, key.upper())
                            ).classes("tactile-finger-name")
                            ui.label(tactile_matrix_summary(matrix)).classes(
                                "tactile-finger-value"
                            )
                        with ui.element("div").classes("tactile-matrix"):
                            for row in matrix:
                                for value in row:
                                    ui.element("div").classes("tactile-cell").style(
                                        tactile_cell_style(value, peak)
                                    )

    @ui.refreshable
    def preset_area(hand: HandState) -> None:
        groups = list(hand.preset_groups)
        if groups and hand.active_group not in hand.preset_groups:
            hand.active_group = groups[0]
        with ui.element("section").classes("preset-card w-full gap-3"):
            with ui.row().classes("w-full items-center justify-between no-wrap"):
                ui.label("Gestures").classes("section-title")
            if not groups:
                ui.label("No tasks").classes("section-title")
                return
            with ui.tabs(value=hand.active_group).classes("w-full") as tabs:
                for group in groups:
                    ui.tab(group, label=display_group_name(group, hand))
            tabs.on_value_change(
                lambda event, selected_hand=hand: setattr(
                    selected_hand, "active_group", str(event.value)
                )
            )
            with ui.tab_panels(tabs, value=hand.active_group).classes(
                "w-full bg-transparent"
            ):
                for group in groups:
                    with ui.tab_panel(group):
                        presets = hand.preset_groups[group]
                        sequences = hand.action_sequence_groups[group]
                        if not presets and not sequences:
                            ui.label("No actions").classes("section-title")
                        with ui.element("div").classes("preset-scroll w-full"):
                            for preset in presets:
                                with ui.element("div").classes("preset-button-group"):
                                    ui.button(
                                        preset.name,
                                        on_click=lambda selected=preset, side=hand.side: (
                                            apply_pose_to_side(
                                                side,
                                                selected.values,
                                                selected.name,
                                                source="APPLY_PRESET",
                                            )
                                        ),
                                    ).props("flat color=cyan no-caps").classes(
                                        "preset-apply-button"
                                    )
                                    ui.button(
                                        icon="close",
                                        on_click=lambda selected=preset, side=hand.side: (
                                            remove_action_from_task(
                                                side,
                                                "preset",
                                                selected.id,
                                                selected.name,
                                            )
                                        ),
                                    ).props("flat color=blue-grey dense").classes(
                                        "preset-remove-button"
                                    )
                                    ui.tooltip("Remove from task")
                            for sequence in sequences:
                                with ui.row().classes(
                                    "sequence-chip action-item items-center gap-1 no-wrap"
                                ):
                                    ui.button(
                                        sequence.name,
                                        on_click=lambda selected=sequence, side=hand.side: (
                                            start_action_sequence(side, selected)
                                        ),
                                    ).props("outline color=purple no-caps").classes(
                                        "soft-button sequence-run-button"
                                    )
                                    ui.button(
                                        icon="stop",
                                        on_click=lambda side=hand.side: (
                                            stop_action_sequence(side)
                                        ),
                                    ).props("outline color=red dense round").classes(
                                        "mini-icon-button"
                                    )
                                    ui.tooltip("Stop sequence")
                                    ui.button(
                                        icon="info",
                                        on_click=lambda selected=sequence, side=hand.side: (
                                            open_action_sequence_dialog(side, selected)
                                        ),
                                    ).props("outline color=purple dense round").classes(
                                        "mini-icon-button"
                                    )
                                    ui.tooltip(
                                        "\n".join(
                                            [format_sequence_loop_label(sequence)]
                                            + [
                                                format_sequence_step_label(step, hand)
                                                for step in sequence.steps
                                            ]
                                        )
                                    )
                                    ui.button(
                                        icon="close",
                                        on_click=lambda selected=sequence, side=hand.side: (
                                            remove_action_from_task(
                                                side,
                                                "sequence",
                                                selected.id,
                                                selected.name,
                                            )
                                        ),
                                    ).props("flat color=blue-grey dense round").classes(
                                        "action-remove-button"
                                    )
                                    ui.tooltip("Remove from task")
                            ui.button(
                                icon="add",
                                on_click=lambda side=hand.side: open_task_action_picker(
                                    side
                                ),
                            ).props("outline color=cyan no-caps").classes(
                                "soft-button add-action-button"
                            )
                            ui.tooltip("Add existing preset or sequence")

    @ui.refreshable
    def hands_area() -> None:
        sliders.clear()
        value_labels.clear()
        runtime_sliders.clear()
        runtime_value_labels.clear()
        with ui.element("div").classes("hands-grid w-full"):
            for side in HAND_SIDES:
                hand = state.hands[side]
                with ui.element("section").classes("hand-panel"):
                    with ui.element("div").classes("hand-card-header w-full"):
                        with ui.column().classes("hand-identity gap-0"):
                            with ui.row().classes("items-center gap-3 no-wrap"):
                                ui.icon(HAND_ICONS[side]).classes("hand-icon")
                                ui.label(HAND_DISPLAY_NAMES[side]).classes("hand-title")
                            ui.label(f"{hand_port_text(side)} connected").classes(
                                "hand-subtitle"
                            )
                        with ui.column().classes("hand-model-control gap-1"):
                            ui.label("Model").classes("hand-field-label")
                            model_select = ui.select(
                                list(MODEL_JOINT_COUNTS),
                                value=hand.model,
                                on_change=lambda event, selected_side=side: (
                                    on_hand_model_change(selected_side, event)
                                ),
                            )
                            model_props = "outlined dense options-dense"
                            if controller_hand_name(side, controller) is not None:
                                model_props += " disable"
                            model_select.props(model_props).classes("w-full")
                        with ui.column().classes("hand-emergency-zone gap-1"):
                            ui.label("Safety").classes("hand-field-label")
                            ui.button(
                                "Stop",
                                icon="warning",
                                on_click=lambda selected_side=side: emergency_stop(
                                    selected_side
                                ),
                            ).props("unelevated color=red no-caps").classes(
                                "emergency-button w-full"
                            )
                    with ui.element("div").classes("runtime-inline w-full"):
                        runtime_controls = [("speed", "Speed", "speed")]
                        hand_name = controller_hand_name(side, controller)
                        torque_supported = (
                            controller.supports_torque(hand_name)
                            if controller is not None and hand_name is not None
                            else hand.model != "O30I"
                        )
                        if torque_supported:
                            runtime_controls.append(("torque", "Torque", "tune"))
                        for kind, title, icon_name in runtime_controls:
                            value = hand.speed if kind == "speed" else hand.torque
                            with ui.element("div").classes(
                                "runtime-control-card gap-2"
                            ):
                                with ui.row().classes(
                                    "w-full items-center justify-between no-wrap"
                                ):
                                    with ui.row().classes("items-center gap-2 no-wrap"):
                                        ui.icon(icon_name).classes("runtime-icon")
                                        ui.label(title).classes("section-title")
                                    runtime_value_labels[(side, kind)] = ui.label(
                                        str(value)
                                    ).classes("runtime-value")
                                slider = (
                                    ui.slider(
                                        min=MIN_CONTROL_VALUE,
                                        max=MAX_CONTROL_VALUE,
                                        step=1,
                                        value=value,
                                    )
                                    .props(
                                        "dense color=green thumb-size=16px track-size=3px"
                                    )
                                    .classes("compact-slider w-full")
                                )
                                slider.on_value_change(
                                    lambda event, selected_side=side, selected_kind=kind: (
                                        on_runtime_config_change(
                                            selected_side,
                                            selected_kind,
                                            event.value,
                                        )
                                    )
                                )
                                runtime_sliders[(side, kind)] = slider
                        if not torque_supported:
                            with ui.element("div").classes(
                                "runtime-control-card gap-2"
                            ):
                                ui.label("Torque").classes("section-title")
                                ui.label("Unsupported by O30I").classes("section-note")
                    with ui.element("section").classes("joint-surface w-full gap-3"):
                        with ui.row().classes(
                            "w-full items-center justify-between no-wrap"
                        ):
                            ui.label("Joint Control").classes("section-title")
                            ui.label(f"{hand.joint_count} channels").classes(
                                "section-note"
                            )
                        with ui.element("div").classes("finger-grid"):
                            for finger in hand.fingers:
                                create_finger(
                                    finger,
                                    hand,
                                    sliders,
                                    value_labels,
                                    on_control_change,
                                )
                    with ui.element("div").classes("action-surface w-full"):
                        preset_area(hand)
                        tactile_area(hand)
                    with ui.expansion(
                        "Library", icon="inventory_2", value=False
                    ).classes("setup-drawer w-full"):
                        with ui.element("div").classes("setup-grid w-full"):
                            with ui.element("div").classes("library-action-group"):
                                task_name_input = (
                                    ui.input(
                                        "Task",
                                        placeholder="lemon-task",
                                        on_change=lambda event, selected_hand=hand: (
                                            setattr(
                                                selected_hand,
                                                "new_task_name",
                                                str(event.value or ""),
                                            )
                                        ),
                                    )
                                    .props("outlined dense clearable")
                                    .classes("task-input")
                                )
                                ui.button(
                                    "Add Task",
                                    icon="add",
                                    on_click=lambda selected_side=side, input_element=task_name_input: (
                                        add_task_group(selected_side, input_element)
                                    ),
                                ).props("outline color=cyan no-caps").classes(
                                    "soft-button task-button"
                                )
                            with ui.element("div").classes("library-action-group"):
                                preset_name_input = (
                                    ui.input(
                                        "Preset",
                                        placeholder="pre-grasp-1",
                                        on_change=lambda event, selected_hand=hand: (
                                            setattr(
                                                selected_hand,
                                                "new_preset_name",
                                                str(event.value or ""),
                                            )
                                        ),
                                    )
                                    .props("outlined dense clearable")
                                    .classes("task-input")
                                )
                                ui.button(
                                    "Save Pose",
                                    icon="save",
                                    on_click=lambda selected_side=side, input_element=preset_name_input: (
                                        save_current_preset(
                                            selected_side, input_element
                                        )
                                    ),
                                ).props("outline color=blue-grey no-caps").classes(
                                    "soft-button task-button"
                                )
                            with ui.element("div").classes(
                                "library-action-group library-action-single"
                            ):
                                ui.button(
                                    "New Sequence",
                                    icon="playlist_add",
                                    on_click=lambda selected_side=side: (
                                        open_action_sequence_dialog(selected_side)
                                    ),
                                ).props("outline color=purple no-caps").classes(
                                    "soft-button task-button w-full"
                                )

    @ui.refreshable
    def command_log_console() -> None:
        nonlocal status_label

        with ui.card().classes("command-console p-2 gap-1"):
            with ui.row().classes("w-full items-center justify-between no-wrap"):
                status_label = ui.label(state.status).classes("status-bar flex-1")
                ui.label(f"{len(state.command_log)} events").classes("command-count")
                ui.button(icon="delete", on_click=clear_command_log).props(
                    "flat dense round color=blue-grey"
                )
                ui.tooltip("Clear log")
            with ui.expansion("Event stream", value=state.command_console_open).classes(
                "command-details w-full"
            ) as expansion:
                expansion.on_value_change(
                    lambda event: setattr(
                        state, "command_console_open", bool(event.value)
                    )
                )
                ui.textarea(
                    value="\n".join(state.newest_commands()) or "No commands"
                ).props("readonly outlined dense").classes("command-log w-full")

    ui.timer(0.03, playback_tick)
    ui.timer(0.03, action_sequence_tick)
    ui.timer(0.05, flush_runtime_config_commands)
    ui.timer(TACTILE_REFRESH_INTERVAL_SECONDS, update_tactile_frames)

    with ui.column().classes("hand-shell w-full"):
        with ui.element("header").classes(
            "top-appbar w-full flex items-center justify-between gap-4"
        ):
            with ui.row().classes("items-center gap-4 no-wrap"):
                ui.label("Barmate Hand Studio").classes("brand-title")
                model_badge = ui.label("").classes("model-badge")
            with ui.row().classes("header-trajectory items-center gap-2 no-wrap"):
                trajectory_path_input = (
                    ui.input(
                        "Trajectory path",
                        placeholder="/data/trajectories/current_path.bin",
                        on_change=lambda event: setattr(
                            state, "trajectory_path", str(event.value or "")
                        ),
                    )
                    .props("outlined dense clearable")
                    .classes("trajectory-field")
                )
                ui.button(icon="upload", on_click=load_trajectory).props(
                    "outline color=blue-grey dense round"
                ).classes("icon-button")
                ui.tooltip("Load trajectory")
                ui.button(icon="play_arrow", on_click=start_playback).props(
                    "outline color=blue-grey dense round"
                ).classes("icon-button")
                ui.tooltip("Replay")
                ui.button(icon="stop", on_click=stop_playback).props(
                    "outline color=red dense round"
                ).classes("icon-button")
                ui.tooltip("Stop replay")
                playback_badge = ui.label("0 frames").classes("model-badge frame-badge")
            _ = trajectory_path_input

        with ui.element("main").classes("control-canvas w-full"):
            initialize_from_controller()
            hands_area()
            sync_controls()
        command_log_console()


def run_linker_hand_control_gui(
    *_: Any,
    controller: ManualHandController | None = None,
    show: bool = True,
    reload: bool = False,
    **run_options: Any,
) -> None:
    """Start the NiceGUI control studio."""

    if controller is not None:
        controller.start()
    set_runtime_controller(controller)
    ui.run(
        title="Barmate Hand Studio",
        reload=reload,
        show=show,
        **run_options,
    )


if __name__ == "__main__":
    run_linker_hand_control_gui()
