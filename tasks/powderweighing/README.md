# Powder weighing：右手固定姿态任务

这是一个独立的临时任务策略，不修改原始 MANUS、Sharpa、ROS bridge 或
O30i 驱动。FREE 状态继续使用原始 Sharpa；MANUS 拇指—食指进入指定范围后，
右手切换到两个经过真机手调的固定姿态：

```text
FREE --稳定接近--> PITCH_READY --完全捏合--> PITCH
  ^                    ^                         |
  |                    +-----松开到 30 mm-------+
  +----------------张开到 55 mm-----------------+
```

固定姿态以 20 个 tick 保存，顺序是 O30i URDF 顺序。运行时按照原始
`0..255 → URDF lower..upper` 映射转换成 radians；不加载任何零位补偿。

该任务只有在显式传入 `--right-hand-strategy-config` 时才会安装，正式代码、
配置和测试均只保留在本目录。

## 第一步：交互调出两个姿态

先停止正在运行的 `hand-control`，因为调姿工具需要独占右手 CAN：

```bash
cd ~/llx/gello_upper_body_teleop/docker
docker compose stop hand-control
```

然后运行：

```bash
docker compose \
  -f compose.yaml \
  -f compose.right-only.yaml \
  run --rm --no-deps hand-control \
  python3 /workspace/franka_upper_body_teleop/tasks/powderweighing/calibrate_poses.py \
  --device 1
```

程序读取右手当前反馈作为起点，不会假设“全 0 tick”是自然姿态。按 Enter
后可以调节：

- `,` / `.`：上一个/下一个关节；
- `-` / `=`：当前关节减/加 1 tick；
- `_` / `+`：当前关节减/加 5 tick；
- `p`：打印当前 20 维 tick；
- `m`：打印命令与反馈；
- `r`：恢复当前阶段的起始姿态；
- `s`：保存当前阶段并进入下一阶段；
- `q`：放弃，本次不修改配置。

先调 `pitch_ready`，第一次按 `s` 后再调 `pitch`，第二次按 `s` 会把两组
数据写入 `tasks/powderweighing/config/poses.json`。所有退出路径都会将右手停回 URDF
neutral。

## 后续重新调整 pose

不需要修改策略代码或重新编译。重新运行交互标定程序并保存，就会覆盖
`config/poses.json` 中的 `pitch_ready` 和 `pitch`。

1. 先在任务程序中按 `Q` 退出，然后停止手部驱动：

   ```bash
   cd ~/llx/gello_upper_body_teleop/docker
   docker compose stop hand-control
   ```

2. 重新运行交互标定：

   ```bash
   docker compose \
     -f compose.yaml \
     -f compose.right-only.yaml \
     run --rm --no-deps hand-control \
     python3 /workspace/franka_upper_body_teleop/tasks/powderweighing/calibrate_poses.py \
     --device 1
   ```

3. 调好 `pitch_ready` 后按 `s`，调好 `pitch` 后再按一次 `s`。第二次保存
   会原子更新 `tasks/powderweighing/config/poses.json`。按 `q` 或 Ctrl-C 会放弃本次
   修改并将右手停回 neutral。

默认从右手当前反馈开始。如果只想在已有 `pitch_ready` 上小幅精调，可以
把 `config/poses.json` 中最新的 `pitch_ready` 数组传给 `--seed-ticks`。例如当前
这版为：

```bash
docker compose \
  -f compose.yaml \
  -f compose.right-only.yaml \
  run --rm --no-deps hand-control \
  python3 /workspace/franka_upper_body_teleop/tasks/powderweighing/calibrate_poses.py \
  --device 1 \
  --seed-ticks 93,184,71,108,92,140,171,63,219,255,230,182,150,248,247,101,77,249,201,0
```

保存后按下面“第二步”的命令重新启动原始右手驱动和任务程序，新 pose 会
直接生效。

## 第二步：双臂双手统一遥操（推荐）

终端 1 启动双臂控制、GELLO bridge 和双手 O30i 驱动：

```bash
cd ~/llx/gello_upper_body_teleop
./ops/run/run_robot_stack.sh
```

终端 2 启动统一 operator，并选择本任务的右手配置：

```bash
cd ~/llx/gello_upper_body_teleop
./ops/run/run_operator.sh \
  --right-hand-strategy-config tasks/powderweighing/config/poses.json
```

左手继续使用原始 MANUS/Sharpa；右手在 `FREE` 状态也使用原始 Sharpa，
仅在手势进入 `PITCH_READY` 或 `PITCH` 时使用本目录中标定的固定姿态。
双臂和双手仍通过同一个 operator GUI 启用、停止和张开。

不传 `--right-hand-strategy-config` 时，`run_operator.sh` 的行为与原来完全
相同，左右手都只使用默认 Sharpa 策略。

## 仅运行右手专项策略（调试）

不要添加左手零位标定 overlay：

```bash
cd ~/llx/gello_upper_body_teleop/docker
docker compose -f compose.yaml -f compose.right-only.yaml up hand-control
```

另开终端运行任务策略：

```bash
cd ~/llx/gello_upper_body_teleop
python -m tasks.powderweighing.run \
  --config tasks/powderweighing/config/poses.json \
  --debug-log /tmp/powderweighing_right.jsonl
```

程序默认 DISENGAGED。按 `R` 启用右手，`X` 停止，`O` 请求张开，`Q` 退出。
终端会显示 `FREE`、`PITCH_READY`、`PITCH` 和当前 MANUS 拇指—食指距离。

## 参数

触发参数也在 `config/poses.json`：

- `ready_enter_m=0.045`：稳定低于 45 mm 后进入 `PITCH_READY`；
- `pitch_enter_m=0.018`：低于 18 mm 后进入 `PITCH`；
- `pitch_exit_m=0.030`：重新大于 30 mm 后退回 `PITCH_READY`；
- `ready_exit_m=0.055`：重新大于 55 mm 后回到 FREE；
- `dwell_seconds=0.15`：准备动作的稳定判定时间；
- `selection_margin_m=0.005`：食指必须比其他三指至少近 5 mm；
- `max_joint_speed_rad_s=3.0`：固定姿态切换的额外关节限速。

如果 `poses_ticks` 仍为 `null`，运行器会拒绝启动，避免把占位值发送给真机。
