"""NiceGUI entry point for FR3 hand-guided waypoint programming."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import threading

from nicegui import events, run, ui

from operator_tasks import DEFAULT_OPERATOR_TASK, OPERATOR_TASKS

from .absolute_recording import AbsoluteTrajectoryStore, build_absolute_trajectory
from .models import ArmRepository, MAX_ROUTINE_WAYPOINTS, Routine, Waypoint
from .recording import ActionStore, build_recorded_action
from .runtime import ArmRosRuntime, TrajectoryCancelled

SIDE_LABELS = {"left": "左臂", "right": "右臂"}


def _degrees(radians: float) -> float:
    return round(math.degrees(radians), 3)


def _radians(degrees: object) -> float:
    return math.radians(float(degrees))


class ArmUiApplication:
    def __init__(self, repository: ArmRepository, runtime: ArmRosRuntime) -> None:
        self.repository = repository
        self.action_store = ActionStore(repository.root)
        self.absolute_store = AbsoluteTrajectoryStore(repository.root)
        self.runtime = runtime
        self._status_lock = threading.RLock()
        self._status = {"left": "等待关节状态", "right": "等待关节状态"}

    def set_status(self, side: str, message: str) -> None:
        with self._status_lock:
            self._status[side] = message

    def status(self, side: str) -> str:
        with self._status_lock:
            return self._status[side]

    def build_page(self) -> None:
        state = {
            "side": "left",
            "selected": None,
            "sequence": [],
            "routine": None,
            "preview_action": None,
            "busy": False,
        }

        ui.add_head_html(
            """
            <style>
              body { background: #f4f6f8; }
              .arm-card { border: 1px solid #d9e0e7; border-radius: 10px; }
              .drop-zone { min-height: 68px; border: 2px dashed #b8c4ce;
                           border-radius: 10px; padding: 8px; }
              .joint-number input { text-align: right; font-family: monospace; }
              .status-pill { border-radius: 999px; padding: 5px 12px; }
              .joint-grid {
                display: grid;
                grid-template-columns: 92px repeat(7, 112px);
                column-gap: 12px;
                row-gap: 10px;
              }
              .arm-workspace { min-width: 1816px; }
            </style>
            """
        )

        with ui.header().classes("items-center justify-between bg-slate-800"):
            ui.label("FR3 点位示教与轨迹编排").classes("text-xl font-semibold")
            side_toggle = ui.toggle(SIDE_LABELS, value="left").props("no-caps")

        with ui.column().classes("w-full max-w-[1880px] mx-auto p-5 gap-5"):
            with ui.row().classes("w-full items-center gap-3"):
                connection_label = ui.label("等待机械臂状态").classes(
                    "status-pill bg-amber-100 text-amber-900"
                )
                mode_label = ui.label("控制模式：未知")
                ui.space()
                teach_button = ui.button("进入零力矩拖动", icon="pan_tool")
                hold_button = ui.button("退出拖动并保持", icon="lock").props(
                    "outline"
                )

            with ui.row().classes(
                "arm-workspace w-full items-stretch gap-5 flex-nowrap overflow-x-auto pb-2"
            ):
                with ui.card().classes("arm-card w-[1036px] min-w-[1036px] p-5"):
                    ui.label("点位库").classes("text-lg font-semibold")
                    ui.label(
                        "拖动真机后记录当前值；修改角度后需保存点位，编辑不会驱动机械臂。"
                    ).classes("text-sm text-slate-500")
                    with ui.row().classes("w-full items-end"):
                        point_name = ui.input("点位名称").classes("grow")
                        record_button = ui.button("记录当前位置", icon="add_location")
                        new_button = ui.button("新建/清空", icon="restart_alt").props(
                            "flat"
                        )

                    current_labels = []
                    joint_inputs = []
                    with ui.element("div").classes("joint-grid w-full items-center"):
                        ui.label("关节").classes("font-medium")
                        for index in range(1, 8):
                            ui.label(f"J{index}").classes("text-center font-medium")
                        ui.label("当前角度")
                        for _ in range(7):
                            current_labels.append(
                                ui.label("--").classes(
                                    "text-center font-mono whitespace-nowrap"
                                )
                            )
                        ui.label("保存角度")
                        for _ in range(7):
                            joint_inputs.append(
                                ui.number(value=0.0, format="%.3f")
                                .props("dense outlined suffix=° step=0.1")
                                .classes("joint-number w-[112px]")
                            )

                    with ui.element("div").classes("joint-grid w-full items-center"):
                        with ui.column().classes("gap-1"):
                            ui.label("微调步长").classes("text-sm")
                            nudge_step = ui.number(
                                value=0.1, min=0.001, max=10.0
                            ).props("dense outlined suffix=° step=0.1").classes(
                                "w-[92px]"
                            )
                        for index in range(7):
                            with ui.row().classes(
                                "w-[112px] items-center justify-center gap-0 flex-nowrap"
                            ):
                                ui.button(icon="remove", color="grey").props(
                                    "dense flat round"
                                ).tooltip(f"J{index + 1} 减小").on(
                                    "click",
                                    lambda _event, index=index: _nudge(index, -1),
                                )
                                ui.label(f"J{index + 1}").classes(
                                    "w-8 text-center text-sm text-slate-500"
                                )
                                ui.button(icon="add", color="grey").props(
                                    "dense flat round"
                                ).tooltip(f"J{index + 1} 增大").on(
                                    "click",
                                    lambda _event, index=index: _nudge(index, 1),
                                )

                    with ui.row().classes("w-full"):
                        save_point_button = ui.button("保存点位", icon="save")
                        delete_point_button = ui.button(
                            "删除点位", icon="delete", color="negative"
                        ).props("outline")
                        ui.space()
                        download_points_button = ui.button(
                            "下载点位 YAML", icon="download"
                        ).props("flat")

                    @ui.refreshable
                    def point_list() -> None:
                        points = self.repository.waypoints(state["side"])
                        if not points:
                            ui.label(
                                "还没有点位，请进入拖动模式后记录。"
                            ).classes("text-slate-400")
                            return
                        with ui.column().classes("w-full gap-2"):
                            for point in points:
                                payload = json.dumps(
                                    {"name": point.name}, ensure_ascii=False
                                )
                                with ui.card().classes(
                                    "arm-card w-full py-2 px-3 cursor-grab"
                                ).props("draggable=true").on(
                                    "dragstart",
                                    js_handler=(
                                        "(e) => e.dataTransfer.setData('text/plain', "
                                        + json.dumps(payload)
                                        + ")"
                                    ),
                                ):
                                    with ui.row().classes("w-full items-center"):
                                        ui.icon("drag_indicator").classes(
                                            "text-slate-400"
                                        )
                                        ui.label(point.name).classes("font-medium grow")
                                        ui.label(
                                            "  ".join(
                                                f"{_degrees(value):.1f}°"
                                                for value in point.joints
                                            )
                                        ).classes(
                                            "text-sm font-mono text-slate-500 whitespace-nowrap"
                                        )
                                        ui.button(icon="edit").props("flat dense").on(
                                            "click",
                                            lambda _event, point=point: _select_point(
                                                point
                                            ),
                                        )
                                        ui.button(icon="add").props("flat dense").on(
                                            "click",
                                            lambda _event, name=point.name: _add_to_sequence(
                                                name
                                            ),
                                        )

                    point_list()

                with ui.card().classes("arm-card w-[760px] min-w-[760px] p-5"):
                    ui.label("轨迹任务").classes("text-lg font-semibold")
                    ui.label(
                        "将左侧点位拖入这里，最多9个；中间点通过Pilz blend连续过渡。"
                    ).classes("text-sm text-slate-500")
                    with ui.row().classes("w-full items-end"):
                        routine_name = ui.input("任务名称").classes("grow")
                        routine_select = ui.select(
                            options=[], label="载入任务", clearable=True
                        ).classes("w-52")
                        save_routine_button = ui.button("保存任务", icon="save")
                        delete_routine_button = ui.button(
                            "删除任务", icon="delete", color="negative"
                        ).props("outline")

                    with ui.row().classes("w-full items-end"):
                        velocity = ui.number(
                            "速度比例", value=10, min=1, max=100, step=1
                        ).props("outlined suffix=%").classes("w-40")
                        acceleration = ui.number(
                            "加速度比例", value=10, min=1, max=100, step=1
                        ).props("outlined suffix=%").classes("w-40")
                        blend = ui.number(
                            "平滑半径", value=5, min=0.1, max=100, step=0.5
                        ).props("outlined suffix=mm").classes("w-40")
                        ui.label("循环：1次（真机验证后开放）").classes(
                            "pb-4 text-slate-500"
                        )

                    def _drop_handler(event: events.GenericEventArguments) -> None:
                        try:
                            data = event.args
                            payload = json.loads(data["payload"])
                            target = int(data["target"])
                            name = str(payload["name"])
                            source = payload.get("source")
                            sequence = state["sequence"]
                            if source is not None:
                                source = int(source)
                                if 0 <= source < len(sequence):
                                    moved = sequence.pop(source)
                                    if source < target:
                                        target -= 1
                                    sequence.insert(
                                        max(0, min(target, len(sequence))), moved
                                    )
                            else:
                                if len(sequence) >= MAX_ROUTINE_WAYPOINTS:
                                    raise ValueError("最多只能加入9个点位")
                                self.repository.waypoint(state["side"], name)
                                sequence.insert(
                                    max(0, min(target, len(sequence))), name
                                )
                            state["routine"] = None
                            sequence_view.refresh()
                        except Exception as error:
                            ui.notify(str(error), color="negative")

                    @ui.refreshable
                    def sequence_view() -> None:
                        sequence = state["sequence"]
                        with ui.column().classes("drop-zone w-full gap-2").on(
                            "dragover", js_handler="(e) => e.preventDefault()"
                        ).on(
                            "drop",
                            _drop_handler,
                            js_handler=(
                                "(e) => {e.preventDefault(); emit({payload: "
                                "e.dataTransfer.getData('text/plain'), target: 999});}"
                            ),
                        ):
                            if not sequence:
                                ui.label("拖入点位或点击点位右侧的 +").classes(
                                    "text-slate-400 self-center py-4"
                                )
                            for index, name in enumerate(sequence):
                                payload = json.dumps(
                                    {"name": name, "source": index}, ensure_ascii=False
                                )
                                with ui.card().classes(
                                    "arm-card w-full py-2 px-3 cursor-grab"
                                ).props("draggable=true").on(
                                    "dragstart",
                                    js_handler=(
                                        "(e) => e.dataTransfer.setData('text/plain', "
                                        + json.dumps(payload)
                                        + ")"
                                    ),
                                ).on(
                                    "dragover", js_handler="(e) => e.preventDefault()"
                                ).on(
                                    "drop",
                                    _drop_handler,
                                    js_handler=(
                                        "(e) => {e.preventDefault(); e.stopPropagation(); "
                                        "emit({payload: e.dataTransfer.getData"
                                        f"('text/plain'), target: {index}}});}}"
                                    ),
                                ):
                                    with ui.row().classes("w-full items-center"):
                                        ui.label(str(index + 1)).classes(
                                            "rounded-full bg-slate-200 px-2 py-1"
                                        )
                                        ui.icon("drag_indicator").classes(
                                            "text-slate-400"
                                        )
                                        ui.label(name).classes("grow font-medium")
                                        ui.button(icon="close").props("flat dense").on(
                                            "click",
                                            lambda _event, index=index: _remove_sequence(
                                                index
                                            ),
                                        )

                    sequence_view()

                    ui.separator()
                    with ui.row().classes("w-full items-center"):
                        start_button = ui.button(
                            "START", icon="play_arrow", color="positive"
                        ).classes("h-14 px-10 text-lg")
                        stop_button = ui.button(
                            "STOP", icon="stop", color="negative"
                        ).classes("h-14 px-8 text-lg")
                        ui.space()
                        download_routines_button = ui.button(
                            "下载任务 YAML", icon="download"
                        ).props("flat")
                    execution_label = ui.label("未运行").classes(
                        "w-full rounded bg-slate-100 p-3"
                    )

            with ui.card().classes(
                "arm-card arm-workspace w-full min-w-[1816px] p-5"
            ):
                with ui.row().classes("w-full items-start"):
                    with ui.column().classes("gap-1 grow"):
                        ui.label("动作示教（相对末端轨迹）").classes(
                            "text-lg font-semibold"
                        )
                        ui.label(
                            "进入零力矩拖动后录制。系统同时保存原始关节/末端轨迹，"
                            "并生成相对动作；停止录制后机械臂仍保持拖动模式。"
                        ).classes("text-sm text-slate-500")
                    capture_frame_label = ui.label(
                        f"基准：{self.runtime.base_frame} → {self.runtime.tool_frame(state['side'])}"
                    ).classes("text-sm font-mono text-slate-500")

                with ui.row().classes("w-full items-end gap-3"):
                    action_name = ui.input(
                        "动作名称", placeholder="例如 scoop_food_1"
                    ).classes("w-96")
                    capture_rate = ui.number(
                        "采样频率", value=100, min=10, max=250, step=10
                    ).props("outlined suffix=Hz").classes("w-40")
                    start_capture_button = ui.button(
                        "开始录制动作", icon="fiber_manual_record", color="negative"
                    ).classes("h-12")
                    stop_capture_button = ui.button(
                        "停止并保存", icon="stop", color="positive"
                    ).classes("h-12")
                    discard_capture_button = ui.button(
                        "放弃录制", icon="delete_sweep", color="warning"
                    ).props("outline").classes("h-12")
                    capture_status_label = ui.label("未录制").classes(
                        "grow rounded bg-slate-100 p-3 font-mono"
                    )

                with ui.row().classes("w-full items-end gap-3"):
                    preview_speed = ui.number(
                        "试运行速度", value=15, min=5, max=100, step=5
                    ).props("outlined suffix=%").classes("w-48")
                    ui.button(
                        "设为原速 100%", icon="speed", color="negative"
                    ).props("outline").on(
                        "click", lambda: preview_speed.set_value(100)
                    )
                    preview_stop_button = ui.button(
                        "停止试运行", icon="stop", color="negative"
                    ).classes("h-12")
                    preview_status_label = ui.label(
                        "先将机械臂拖到新起点，验证 IK 后再试运行"
                    ).classes("grow rounded bg-amber-50 p-3 font-mono")

                @ui.refreshable
                def action_list() -> None:
                    actions = self.action_store.summaries(state["side"])
                    if not actions:
                        ui.label("还没有保存的动作录制。").classes(
                            "text-slate-400"
                        )
                        return
                    with ui.column().classes("w-full gap-2"):
                        for summary in actions:
                            active = ", ".join(
                                f"J{index}" for index in summary["active_joints"]
                            ) or "未识别"
                            with ui.card().classes(
                                "arm-card w-full py-2 px-4"
                            ):
                                with ui.row().classes("w-full items-center"):
                                    ui.icon("gesture").classes("text-blue-500")
                                    ui.label(str(summary["name"])).classes(
                                        "w-64 font-medium"
                                    )
                                    ui.label(
                                        f"{summary['duration_sec']:.2f}s"
                                    ).classes("w-24 font-mono")
                                    ui.label(
                                        f"{summary['sample_count']} 帧"
                                    ).classes("w-24 font-mono")
                                    ui.label(f"活动关节：{active}").classes("grow")
                                    async def validate_selected_action(
                                        _event: object = None,
                                        name: object = summary["name"],
                                    ) -> None:
                                        await _validate_action(str(name))

                                    def open_selected_preview(
                                        _event: object = None,
                                        name: object = summary["name"],
                                    ) -> None:
                                        _open_preview(str(name))

                                    ui.button(
                                        "当前位置验证 IK", icon="fact_check"
                                    ).props("outline").on(
                                        "click", validate_selected_action
                                    )
                                    ui.button(
                                        "动作试运行", icon="slow_motion_video",
                                        color="warning",
                                    ).on("click", open_selected_preview)
                                    ui.button(
                                        "下载 YAML", icon="download"
                                    ).props("flat").on(
                                        "click",
                                        lambda _event, path=summary["path"]: _download(
                                            path
                                        ),
                                    )
                                    ui.button(
                                        icon="delete", color="negative"
                                    ).props("flat round").on(
                                        "click",
                                        lambda _event,
                                        name=summary["name"]: _delete_action(
                                            str(name)
                                        ),
                                    )

                action_list()

            with ui.card().classes(
                "arm-card arm-workspace w-full min-w-[1816px] p-5"
            ):
                ui.label("Operator GUI：双臂绝对 Ready → Home 轨迹").classes(
                    "text-lg font-semibold"
                )
                ui.label(
                    "先在 Operator GUI 记录共享 Ready 和各任务 Home。然后让左右臂都进入"
                    "零力矩拖动，从 Ready 开始同步拖动双臂，到所选任务 Home 后停止保存。"
                ).classes("text-sm text-slate-500")
                with ui.row().classes("w-full items-end gap-3"):
                    absolute_task = ui.select(
                        OPERATOR_TASKS,
                        value=DEFAULT_OPERATOR_TASK,
                        label="对应任务",
                    ).classes("w-64")
                    absolute_rate = ui.number(
                        "采样频率", value=50, min=10, max=100, step=10
                    ).props("outlined suffix=Hz").classes("w-40")
                    start_absolute_button = ui.button(
                        "开始双臂绝对录制",
                        icon="fiber_manual_record",
                        color="negative",
                    ).classes("h-12")
                    stop_absolute_button = ui.button(
                        "停止并保存/覆盖", icon="stop", color="positive"
                    ).classes("h-12")
                    discard_absolute_button = ui.button(
                        "放弃录制", icon="delete_sweep", color="warning"
                    ).props("outline").classes("h-12")
                    absolute_status_label = ui.label("未录制").classes(
                        "grow rounded bg-slate-100 p-3 font-mono"
                    )

        def _nudge(index: int, direction: int) -> None:
            try:
                step = float(nudge_step.value or 0.1)
                joint_inputs[index].value = round(
                    float(joint_inputs[index].value or 0.0) + direction * step, 3
                )
            except (TypeError, ValueError):
                ui.notify("请输入有效的微调步长", color="negative")

        def _clear_editor() -> None:
            state["selected"] = None
            point_name.value = ""
            for field in joint_inputs:
                field.value = 0.0

        def _select_point(point: Waypoint) -> None:
            state["selected"] = point.name
            point_name.value = point.name
            for field, value in zip(joint_inputs, point.joints, strict=True):
                field.value = _degrees(value)

        def _record_current() -> None:
            try:
                snapshot = self.runtime.snapshot(state["side"])
                state["selected"] = None
                if not point_name.value:
                    point_count = len(self.repository.waypoints(state["side"]))
                    point_name.value = f"point_{point_count + 1}"
                for field, value in zip(joint_inputs, snapshot.positions, strict=True):
                    field.value = _degrees(value)
                ui.notify("已读取真实关节角；请命名并保存", color="positive")
            except Exception as error:
                ui.notify(str(error), color="negative")

        def _save_point() -> None:
            try:
                point = Waypoint.create(
                    point_name.value,
                    state["side"],
                    [_radians(field.value) for field in joint_inputs],
                )
                self.repository.save_waypoint(
                    point, previous_name=state["selected"]
                )
                state["selected"] = point.name
                point_list.refresh()
                _refresh_routines()
                ui.notify(f"点位“{point.name}”已保存", color="positive")
            except Exception as error:
                ui.notify(str(error), color="negative")

        def _delete_point() -> None:
            try:
                name = state["selected"]
                if not name:
                    raise ValueError("请先选择要删除的点位")
                self.repository.delete_waypoint(state["side"], name)
                _clear_editor()
                point_list.refresh()
                ui.notify(f"点位“{name}”已删除")
            except Exception as error:
                ui.notify(str(error), color="negative")

        def _add_to_sequence(name: str) -> None:
            try:
                if len(state["sequence"]) >= MAX_ROUTINE_WAYPOINTS:
                    raise ValueError("最多只能加入9个点位")
                self.repository.waypoint(state["side"], name)
                state["sequence"].append(name)
                state["routine"] = None
                sequence_view.refresh()
            except Exception as error:
                ui.notify(str(error), color="negative")

        def _remove_sequence(index: int) -> None:
            if 0 <= index < len(state["sequence"]):
                state["sequence"].pop(index)
                state["routine"] = None
                sequence_view.refresh()

        def _current_routine() -> Routine:
            return Routine.create(
                routine_name.value or "临时任务",
                state["side"],
                state["sequence"],
                velocity_scale=float(velocity.value) / 100.0,
                acceleration_scale=float(acceleration.value) / 100.0,
                blend_radius_m=float(blend.value) / 1000.0,
            )

        def _refresh_routines() -> None:
            options = [
                routine.name for routine in self.repository.routines(state["side"])
            ]
            routine_select.options = options
            routine_select.update()

        def _save_routine() -> None:
            try:
                routine = _current_routine()
                self.repository.save_routine(routine)
                state["routine"] = routine.name
                routine_select.value = routine.name
                _refresh_routines()
                ui.notify(f"任务“{routine.name}”已保存", color="positive")
            except Exception as error:
                ui.notify(str(error), color="negative")

        def _load_routine(name: str | None) -> None:
            if not name:
                return
            try:
                routine = next(
                    item
                    for item in self.repository.routines(state["side"])
                    if item.name == name
                )
                state["routine"] = routine.name
                state["sequence"] = list(routine.waypoints)
                routine_name.value = routine.name
                velocity.value = routine.velocity_scale * 100.0
                acceleration.value = routine.acceleration_scale * 100.0
                blend.value = routine.blend_radius_m * 1000.0
                sequence_view.refresh()
            except (StopIteration, ValueError) as error:
                ui.notify(str(error), color="negative")

        def _delete_routine() -> None:
            try:
                name = state["routine"] or routine_select.value
                if not name:
                    raise ValueError("请先载入要删除的任务")
                self.repository.delete_routine(state["side"], str(name))
                state["routine"] = None
                routine_select.value = None
                _refresh_routines()
                ui.notify(f"任务“{name}”已删除")
            except Exception as error:
                ui.notify(str(error), color="negative")

        async def _switch_mode(mode: str) -> None:
            if state["busy"]:
                return
            state["busy"] = True
            try:
                await run.io_bound(self.runtime.switch_mode, state["side"], mode)
                ui.notify(
                    "已进入零力矩拖动模式，请托住机械臂"
                    if mode == "teach"
                    else "已退出拖动并保持当前位置",
                    color="positive",
                )
            except Exception as error:
                ui.notify(str(error), color="negative", timeout=8)
            finally:
                state["busy"] = False

        async def _start() -> None:
            if state["busy"]:
                return
            try:
                routine = _current_routine()
                points = self.repository.resolve(routine)
            except Exception as error:
                ui.notify(str(error), color="negative")
                return
            state["busy"] = True
            execution_label.text = "Start：读取真实起点并提交MoveIt规划…"
            try:
                await run.io_bound(self.runtime.execute, routine, points)
                execution_label.text = "完成：机械臂保持在最后一个点位"
                ui.notify("轨迹执行完成", color="positive")
            except TrajectoryCancelled:
                execution_label.text = "已停止：机械臂保持当前位置"
                ui.notify("轨迹已停止", color="warning")
            except Exception as error:
                execution_label.text = f"失败：{error}"
                ui.notify(str(error), color="negative", timeout=10)
            finally:
                state["busy"] = False

        async def _stop() -> None:
            try:
                accepted = await run.io_bound(self.runtime.stop)
                if accepted:
                    execution_label.text = "已发送停止请求，等待机械臂保持"
                    preview_status_label.text = "已发送停止请求，等待机械臂保持"
                else:
                    ui.notify("当前没有正在执行的轨迹")
            except Exception as error:
                ui.notify(str(error), color="negative", timeout=10)

        async def _start_capture() -> None:
            if state["busy"]:
                return
            name = str(action_name.value or "").strip()
            if not name:
                ui.notify("请先填写动作名称", color="warning")
                return
            if self.action_store.path_for(state["side"], name).exists():
                ui.notify(f"动作名称已存在: {name}", color="negative")
                return
            state["busy"] = True
            try:
                await run.io_bound(
                    self.runtime.start_recording,
                    state["side"],
                    rate_hz=float(capture_rate.value),
                )
                ui.notify("动作录制已开始，请拖动机械臂", color="negative")
            except Exception as error:
                ui.notify(str(error), color="negative", timeout=10)
            finally:
                state["busy"] = False

        async def _stop_capture() -> None:
            if state["busy"]:
                return
            state["busy"] = True
            try:
                capture = await run.io_bound(self.runtime.stop_recording)
                action = await run.io_bound(
                    build_recorded_action,
                    name=str(action_name.value or "").strip(),
                    side=capture.side,
                    base_frame=capture.base_frame,
                    tool_frame=capture.tool_frame,
                    samples=capture.samples,
                )
                path = await run.io_bound(self.action_store.save, action)
                action_list.refresh()
                ui.notify(
                    f"动作“{action.name}”已保存：{path.name}", color="positive"
                )
            except Exception as error:
                ui.notify(str(error), color="negative", timeout=10)
            finally:
                state["busy"] = False

        async def _discard_capture() -> None:
            try:
                discarded = await run.io_bound(self.runtime.discard_recording)
                ui.notify("已放弃本次动作录制" if discarded else "当前没有录制")
            except Exception as error:
                ui.notify(str(error), color="negative")

        async def _start_absolute_capture() -> None:
            if state["busy"]:
                return
            state["busy"] = True
            try:
                await run.io_bound(
                    self.runtime.start_absolute_recording,
                    rate_hz=float(absolute_rate.value),
                )
                ui.notify(
                    "双臂绝对轨迹录制已开始；请从 Ready 同步拖动到任务 Home",
                    color="negative",
                )
            except Exception as error:
                ui.notify(str(error), color="negative", timeout=10)
            finally:
                state["busy"] = False

        async def _stop_absolute_capture() -> None:
            if state["busy"]:
                return
            state["busy"] = True
            try:
                capture = await run.io_bound(self.runtime.stop_absolute_recording)
                trajectory = await run.io_bound(
                    build_absolute_trajectory,
                    absolute_task.value,
                    capture.samples,
                    capture.joint_names,
                )
                path = await run.io_bound(self.absolute_store.save, trajectory)
                ui.notify(
                    f"“{trajectory.label}” Ready → Home 已保存：{path.name}",
                    color="positive",
                )
            except Exception as error:
                ui.notify(str(error), color="negative", timeout=10)
            finally:
                state["busy"] = False

        async def _discard_absolute_capture() -> None:
            try:
                discarded = await run.io_bound(
                    self.runtime.discard_absolute_recording
                )
                ui.notify(
                    "已放弃双臂绝对轨迹录制" if discarded else "当前没有双臂录制"
                )
            except Exception as error:
                ui.notify(str(error), color="negative")

        async def _validate_action(name: str) -> None:
            if state["busy"]:
                return
            state["busy"] = True
            try:
                action = await run.io_bound(
                    self.action_store.load, state["side"], name
                )
                result = await run.io_bound(
                    self.runtime.validate_relative_action, action
                )
                ui.notify(
                    f"IK验证通过：检查 {result.checked_frames}/{result.total_frames} 帧，"
                    f"最大相邻关节变化 {math.degrees(result.max_joint_step_rad):.2f}°；"
                    "机械臂未运动",
                    color="positive",
                    timeout=10,
                )
            except Exception as error:
                ui.notify(str(error), color="negative", timeout=12)
            finally:
                state["busy"] = False

        def _open_preview(name: str) -> None:
            if state["busy"]:
                ui.notify("当前有操作正在进行", color="warning")
                return
            state["preview_action"] = name
            preview_dialog_action.text = f"动作：{name}"
            speed_percent = float(preview_speed.value)
            preview_dialog_speed.text = f"执行速度：{speed_percent:.0f}%"
            if speed_percent > 30.0:
                preview_dialog_warning.text = (
                    "高于30%属于高速真机执行。仅在该动作已逐级完成低速验证、"
                    "工作区清空且硬件停止手段可触达时继续。"
                )
                preview_dialog_warning.set_visibility(True)
            else:
                preview_dialog_warning.set_visibility(False)
            preview_dialog.open()

        async def _preview_action() -> None:
            name = state["preview_action"]
            if not name or state["busy"]:
                return
            state["busy"] = True
            preview_status_label.text = f"正在消抖并从当前姿态求解动作“{name}”…"
            try:
                action = await run.io_bound(
                    self.action_store.load, state["side"], str(name)
                )
                result = await run.io_bound(
                    self.runtime.execute_relative_action,
                    action,
                    speed_scale=float(preview_speed.value) / 100.0,
                )
                preview_status_label.text = (
                    f"完成：{result.checked_frames}帧IK全部通过，保持在动作末点"
                )
                ui.notify("低速试运行完成，机械臂保持在末点", color="positive")
            except TrajectoryCancelled:
                preview_status_label.text = "已停止：机械臂保持当前位置"
                ui.notify("低速试运行已停止", color="warning")
            except Exception as error:
                preview_status_label.text = f"试运行失败：{error}"
                ui.notify(str(error), color="negative", timeout=12)
            finally:
                state["busy"] = False

        def _delete_action(name: str) -> None:
            try:
                self.action_store.delete(state["side"], name)
                action_list.refresh()
                ui.notify(f"动作“{name}”已删除")
            except Exception as error:
                ui.notify(str(error), color="negative")

        def _change_side(side: str) -> None:
            if state["busy"]:
                side_toggle.value = state["side"]
                ui.notify("运行或切换控制器期间不能更换机械臂", color="warning")
                return
            state["side"] = side
            state["selected"] = None
            state["sequence"] = []
            state["routine"] = None
            _clear_editor()
            routine_name.value = ""
            routine_select.value = None
            point_list.refresh()
            sequence_view.refresh()
            _refresh_routines()
            capture_frame_label.text = (
                f"基准：{self.runtime.base_frame} → {self.runtime.tool_frame(state['side'])}"
            )
            action_list.refresh()

        def _refresh_state() -> None:
            side = state["side"]
            try:
                snapshot = self.runtime.snapshot(side)
                for label, value in zip(current_labels, snapshot.positions, strict=True):
                    label.text = f"{_degrees(value):.3f}°"
                connection_label.text = f"{SIDE_LABELS[side]}状态正常"
                connection_label.classes(
                    remove="bg-amber-100 text-amber-900 bg-red-100 text-red-900",
                    add="bg-green-100 text-green-900",
                )
            except Exception as error:
                connection_label.text = str(error)
                connection_label.classes(
                    remove="bg-amber-100 text-amber-900 bg-green-100 text-green-900",
                    add="bg-red-100 text-red-900",
                )
            mode = self.runtime.controller_mode(side)
            mode_text = {"teach": "零力矩拖动", "trajectory": "轨迹保持"}.get(
                mode, "未知"
            )
            mode_label.text = f"控制模式：{mode_text}｜{self.status(side)}"
            capture_status = self.runtime.recording_status()
            if capture_status["active"]:
                capture_status_label.text = (
                    f"录制中  {capture_status['elapsed_sec']:.1f}s  "
                    f"{capture_status['sample_count']}帧  "
                    f"TF跳过:{capture_status['skipped_tf_samples']}"
                )
                capture_status_label.classes(
                    remove="bg-slate-100 bg-red-100 text-red-900",
                    add="bg-red-100 text-red-900",
                )
                start_capture_button.disable()
                stop_capture_button.enable()
                discard_capture_button.enable()
            else:
                capture_status_label.text = "未录制"
                capture_status_label.classes(
                    remove="bg-red-100 text-red-900", add="bg-slate-100"
                )
                start_capture_button.enable()
                stop_capture_button.disable()
                discard_capture_button.disable()
            absolute_status = self.runtime.absolute_recording_status()
            if absolute_status["active"]:
                absolute_status_label.text = (
                    f"双臂录制中  {absolute_status['elapsed_sec']:.1f}s  "
                    f"{absolute_status['sample_count']}帧"
                )
                absolute_status_label.classes(
                    remove="bg-slate-100", add="bg-red-100 text-red-900"
                )
                start_absolute_button.disable()
                stop_absolute_button.enable()
                discard_absolute_button.enable()
            else:
                path = self.absolute_store.path(absolute_task.value)
                absolute_status_label.text = (
                    f"已保存：{path.name}" if path.exists() else "当前任务尚未录制"
                )
                absolute_status_label.classes(
                    remove="bg-red-100 text-red-900", add="bg-slate-100"
                )
                start_absolute_button.enable()
                stop_absolute_button.disable()
                discard_absolute_button.disable()
            if capture_status["active"]:
                start_absolute_button.disable()
            if absolute_status["active"]:
                start_capture_button.disable()
            if self.runtime.is_running():
                preview_stop_button.enable()
            else:
                preview_stop_button.disable()

        def _download(path: Path) -> None:
            if not path.exists():
                ui.notify("还没有可下载的YAML文件", color="warning")
                return
            ui.download(path)

        async def _confirm_teach() -> None:
            teach_dialog.close()
            await _switch_mode("teach")

        async def _confirm_start() -> None:
            start_dialog.close()
            await _start()

        async def _hold() -> None:
            await _switch_mode("trajectory")

        async def _confirm_preview() -> None:
            preview_dialog.close()
            await _preview_action()

        with ui.dialog() as teach_dialog, ui.card().classes("max-w-lg"):
            ui.label("确认进入零力矩拖动模式").classes("text-lg font-semibold")
            ui.label(
                "切换后机械臂将依靠本体重力补偿。请先托住机械臂，"
                "确认负载和质心配置正确，并确保硬件停止手段可触达。"
                "允许左右臂同时处于拖动模式；切换另一侧前请确保当前臂状态稳定。"
            )
            with ui.row().classes("w-full justify-end"):
                ui.button("取消", on_click=teach_dialog.close).props("flat")
                ui.button(
                    "我已托住，进入拖动",
                    on_click=_confirm_teach,
                    color="warning",
                )

        with ui.dialog() as start_dialog, ui.card().classes("max-w-lg"):
            ui.label("确认执行轨迹").classes("text-lg font-semibold")
            ui.label(
                "Start 会读取真实起点、切换到轨迹控制并由 MoveIt 规划执行。"
                "请确认机械臂周围无人且硬件停止手段可用。"
            )
            with ui.row().classes("w-full justify-end"):
                ui.button("取消", on_click=start_dialog.close).props("flat")
                ui.button(
                    "确认 START",
                    on_click=_confirm_start,
                    color="positive",
                )

        with ui.dialog() as preview_dialog, ui.card().classes("max-w-xl"):
            ui.label("确认试运行相对动作").classes(
                "text-lg font-semibold"
            )
            preview_dialog_action = ui.label("动作：").classes("font-mono")
            preview_dialog_speed = ui.label("执行速度：").classes("font-mono")
            preview_dialog_warning = ui.label("").classes(
                "rounded bg-red-100 p-3 font-semibold text-red-900"
            )
            ui.label(
                "系统会先对末端轨迹做零相位消抖并重采样到最高30Hz，再从当前真实"
                "末端姿态重新求解完整IK轨迹。验证通过后自动退出零力矩拖动并驱动"
                "机械臂。请松开机械臂、清空周围空间，并确保STOP和硬件停止手段可触达。"
            )
            ui.label("试运行结束后机械臂保持在动作末点。")
            with ui.row().classes("w-full justify-end"):
                ui.button("取消", on_click=preview_dialog.close).props("flat")
                ui.button(
                    "确认试运行",
                    on_click=_confirm_preview,
                    color="warning",
                )

        side_toggle.on_value_change(lambda event: _change_side(event.value))
        record_button.on("click", lambda: _record_current())
        new_button.on("click", lambda: _clear_editor())
        save_point_button.on("click", lambda: _save_point())
        delete_point_button.on("click", lambda: _delete_point())
        save_routine_button.on("click", lambda: _save_routine())
        delete_routine_button.on("click", lambda: _delete_routine())
        routine_select.on_value_change(lambda event: _load_routine(event.value))
        teach_button.on("click", teach_dialog.open)
        hold_button.on("click", _hold)
        start_button.on("click", start_dialog.open)
        stop_button.on("click", _stop)
        start_capture_button.on("click", _start_capture)
        stop_capture_button.on("click", _stop_capture)
        discard_capture_button.on("click", _discard_capture)
        start_absolute_button.on("click", _start_absolute_capture)
        stop_absolute_button.on("click", _stop_absolute_capture)
        discard_absolute_button.on("click", _discard_absolute_capture)
        preview_stop_button.on("click", _stop)
        download_points_button.on(
            "click", lambda: _download(self.repository.waypoints_path)
        )
        download_routines_button.on(
            "click", lambda: _download(self.repository.routines_path)
        )
        _refresh_routines()
        ui.timer(0.2, _refresh_state)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--data-root", default="/data/arm_ui")
    parser.add_argument("--robot-type", default="fr3", choices=("fr3", "fr3v2"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repository = ArmRepository(args.data_root)
    runtime = ArmRosRuntime(robot_type=args.robot_type)
    application = ArmUiApplication(repository, runtime)
    runtime.status_callback = application.set_status
    ui.page("/")(application.build_page)
    try:
        ui.run(
            host=args.host,
            port=args.port,
            title="FR3 Arm UI",
            reload=False,
            show=False,
        )
    finally:
        runtime.shutdown()
    return 0


if __name__ in {"__main__", "__mp_main__"}:
    raise SystemExit(main())
