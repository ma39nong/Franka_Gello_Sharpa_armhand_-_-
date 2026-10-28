# Apps

本目录只放面向操作员的交互应用：

- `operator_gui`：控制统一 Operator 后端的桌面 GUI。
- `hand_ui`：通过 LinkerHand 安全桥接控制 O30i/G20 的网页 UI。
- `arm_ui`：FR3 零力矩拖动示教、相对末端动作录制/IK 验证、关节点位管理和单臂平滑轨迹编排网页 UI。
- `wuji_ui`：双 Wuji Hand 2 手动/Manus 遥操、实际关节姿态记录和限速回放网页 UI；完整说明见 [`docs/WUJI_HAND_UI.md`](../docs/WUJI_HAND_UI.md)。

任务策略不放在这里；称粉等工作流位于 `tasks/`。
