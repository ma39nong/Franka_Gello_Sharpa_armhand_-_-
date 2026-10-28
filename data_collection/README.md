# Harvest 数据采集

该子系统只读取安全网关后的机械臂 action、实测状态、Operator 导出的 Wuji
telemetry 和三路相机。在线阶段只写 ROS bag；停止录制不会转换、上传或删除任何
bag。转换必须由操作者另行执行。

第一期只支持 **GELLO 臂 + Wuji 手 + 三相机**。现有
`./ops/run/start_wuji_teleop.sh` 入口不变；采集是独立的只读进程，故障不能阻塞
或接管遥操。

现有 `teleop_data`（O30i/G20 + 单路 `/camera/*`）保持独立，不要把它改成 Harvest
格式，也不要用 Harvest recorder 去订 `/cb_*`。

## 进程边界

```text
终端 1  现有 Operator（唯一 GELLO/MANUS/Wuji client）
  └─ 有界非阻塞队列 → 单向 UDP sender（默认 127.0.0.1:5602）
                         └─ teleop_hand_telemetry（仅 ROS publishers）

现有 safety gateway → /teleop/validated_arm_commands ─┐
双 FR3 实测状态 ───────────────────────────────────────┤
三台 Orbbec（cam0/cam1/cam2）──────────────────────────┼→ ros2 bag record
Wuji ROS telemetry ────────────────────────────────────┘

finalized ROS bag --操作者手动命令--> 临时 LeRobot v2 输出
                                      └─ 完整验证后原子发布
```

谁可以 publish：

| 进程 | 可以 publish | 禁止 |
| --- | --- | --- |
| safety gateway | `/teleop/validated_arm_commands`、FR3 command bus | 第二套控制源 |
| Operator / HandWorker | Wuji SDK 命令；UDP telemetry（本机） | ROS 手/臂命令 |
| `teleop_hand_telemetry` | `/teleop/wuji/{left,right}/command\|joint_states`、status | 任何硬件命令 |
| `teleop_camera_bringup` | `/cam{0,1,2}/color/image_raw`、`/cam0/depth/image_raw` | FR3/Wuji 命令 |
| `teleop_data_collector` | 无（只 `ros2 bag record`） | 任何 robot/hand 命令 |

采集容器不启动 Operator、GELLO/Pico bridge、safety gateway、controller 或任何
Wuji/Pico/MANUS SDK client。杀掉 recorder 或 telemetry receiver 后，控制进程
必须继续运行。

## 部署配置与 Orbbec 许可

先编辑 `data_collection/config/cameras.yaml`，为 cam0/cam1/cam2 填入三个唯一的
序列号和物理语义。任何 `REPLACE_*`、空白或重复 SN 都会阻止启动。不要把真实 SN
提交进 Git。

仓库里的 `ros_ws/src/orbbec_camera` 默认带 `COLCON_IGNORE`，以免 overlay 现有
Docker `orbbec` 单相机服务所用的 vendor 包。Orbbec wrapper 保留 Apache-2.0
LICENSE/NOTICE；SDK 2.8.6 的 LICENSE 和 EULA 位于
`ros_ws/src/orbbec_camera/SDK`。普通构建不会构建 SDK runtime。阅读并接受 EULA
后，必须显式构建：

```bash
./ops/setup/build.sh --accept-orbbec-eula
```

该参数只对本次构建有效。运行时 `start_recording.sh` 会读本机已 gitignore 的
`docker/.env`。在阅读 EULA 并接受后，把 `ORBBEC_SDK_LICENSE_ACCEPTED=YES`
写进该文件即可，不必每次录制前再 `export`。未写入则拒绝启动三相机。

Harvest 三相机 bringup 与 compose `orbbec`、`start_orbbec_viewer.sh` 互斥。
`start_recording.sh` 会拒绝 hand-control、compose `orbbec` 和 OrbbecViewer。

## 遥操与录制

终端一继续使用现有 Wuji 入口。Operator 默认把现有 Wuji 连接的只读 telemetry
发往本机 UDP 5602，采集未启动时不得影响控制：

```bash
ROS_DOMAIN_ID=1 TELEOP_ROS_DOMAIN_ID=1 ./ops/run/start_wuji_teleop.sh \
  --wuji-left-address LEFT_IP:PORT \
  --wuji-right-address RIGHT_IP:PORT
```

终端二在确认本机 `docker/.env` 已有 `ORBBEC_SDK_LICENSE_ACCEPTED=YES` 后启动
独立采集（不要焊进 `start_wuji_teleop.sh`）：

```bash
./ops/run/start_recording.sh \
  --data-root /home/user/franka_teleop_data
```

仍然需要分别启动遥操和采集两个终端。采集器 READY 后，遥操 UI 的“数据采集”区域
会通过仅监听 `127.0.0.1:5592` 的独立接口自动连接。一个按钮按当前状态执行
“开始录制”或“结束并校验”（快捷键 `L`），`Space` / “中间完成标记”在录制中保存
一个完成时间点并继续录制；“丢弃最近一次”保留按钮，无快捷键。
UI 的 `A` / `C` 切换左/右臂 HOLD，左右 hand 跟随只保留 GUI 按钮。
UI 关闭或连接失败不会停止采集，也不会影响遥操；终端里的 `L`、`SPACE`、`D`
分别作为起停、完成标记、丢弃的备用控制。

停止录制并完成校验后，UI 自动弹出“数据质量评价”，显示原始 episode 编号和
核验结果。选择“优等 / 一般 / 报错 / 放弃”后，由采集端保存到
`<data-root>/数据分类/优等.txt`、`一般.txt`、`报错.txt`、`放弃.txt`。
使用上述启动命令时，主机目录是 `/home/user/franka_teleop_data/数据分类/`。
四个文件采用 UTF-8，每行一个原始数字编号（例如 `episode384` 写成 `384`），
可直接复制到转换工程作为 episode list。已有注释和编号会保留，连续范围会展开；
重新评价会去重并从原分类移除该编号。

每条数据只自动弹窗一次；关闭弹窗后可通过“数据质量”重新打开，也可在开始下一条
录制前修改最近一次的评价。UI 在评价保存成功后才允许开始下一条；保存失败会显示
错误并允许重试，断线重连后会重新核对状态。人工质量评价不改变自动校验结果。
“放弃”和原有“丢弃最近一次”都会保留 bag、标记弃用并写入 `放弃.txt`；已弃用数据
不能通过修改质量标签恢复。分类文件独立于转换工程，程序不会写入其列表目录。

数据必须写在仓库外。`--data-root` 若落在 Git 仓库内会被拒绝。

- `L`：开始/停止 episode。开始前检查 topic 名称和 ROS 类型；停止后读取整包，
  以 source header 检查消息数、频率、150 ms gap 和单调性；bag receive gap 单独作为
  传输拥塞告警，不会把源端连续的数据判坏。默认开头 1 秒、结尾 0.2 秒是操作缓冲区；校验只用
  中间有效 source 区间判断连续性，落在缓冲区内的 receive-time 边界缺口单列为
  `boundary_warnings`。同时检查 joint name、有限值、双侧
  action、双手、三路 RGB、头部原始深度和 telemetry 丢包计数。
- 启动时先显示 `WAITING`；所有必需 topic 均已出现、类型匹配并连续稳定 2 秒后，
  才显示 `READY` 和上述快捷键。看到 `READY` 前不要开始任务动作。
- `D`：将最近 episode 标记为 `discarded`，不删除源文件。
- `SPACE`：录制中保存 `milestone_1`、`milestone_2` 等完成标记，不停止、不复制 bag。
  UI 显示已保存的标记数；UI 快捷键长按不会自动重复。时间为采集端处理请求时的 ROS
  时间，使用与 source header 相同的时钟域，不使用 UI 计时或时间百分比。
  标记先落盘，再报告成功，结束录制、丢弃和重校验都会保留标记。
- `Ctrl-C`：active episode 标记为 `interrupted`。

停止时终端会显示校验开始、耗时，并逐条打印 `[validation FAILED]` 原因。
校验通过且只有接收时间抖动时，终端合并为一条 `[transport info]` 普通信息，
显示告警数量及 `collection_state.json` 路径；详细数值仍完整保存在报告中。
存在校验失败或裁剪边界告警时，仍逐条打印 `[boundary warning]` 和
`[transport warning]`。只有确实落在裁剪容许范围内的边界间隔，才会提示裁剪后接受。
完整结果仍保存在该 episode 的 `collection_state.json`。
图像校验逐帧读取 CDR 时间戳和格式信息，以只读视图检查像素数据长度，
避免将整幅像素反序列化成 ROS 数组；不支持的 CDR 编码回退到 ROS 解码器。
这项优化保留全帧连续性和深度格式检查，仍需从存储中读取原始 bag。

现有 Operator 的 `disengage` 同时作为数据语义中的 HOLD。原始 bag 只记录真实
command 和双侧 engagement 状态；转换时才对明确 disengage 的一侧锁存最后有效
action。若该侧从 episode 开始即 disengage，则以同侧实测位置初始化。输出额外保存
`observation.engaged=[左臂,右臂,左手,右手]`，但默认不作为策略输入。

状态只有 `recording`、`finalized`、`incomplete`、`interrupted`、`discarded`。
缺任一臂、手或相机流不得 `finalized`。

旧 bag 可用相同规则独立重校验；命令只原子更新 `collection_state.json`，不修改
数据库和 `metadata.yaml`：

```bash
./ops/run/revalidate_recording.sh \
  /home/user/franka_teleop_data/bags/gello/episode12 \
  --trim-start-sec 1 --trim-end-sec 0.2
```

## 手动转换

仅 `finalized` bag 可转换：

```bash
./ops/run/convert_recording.sh \
  /home/user/franka_teleop_data/bags/gello/episode0 \
  /home/user/franka_teleop_data/datasets/episode0
```

默认 `segments: all`：没有标记的 bag 仍输出一条完整 episode；有一个标记则输出
“有效开始→标记”和“有效开始→有效结束”两条 episode，放在同一数据集中。
多个标记按时间顺序各生成一条前缀，最后生成完整 episode。图像、深度、action
和 state 使用相同截止时间；每条 episode 独立从时间 0 开始，最后一行 `next.done=true`。
完整数据沿用首尾裁剪；前缀沿用相同有效开始，在标记处结束，不再额外减去结尾 0.2 秒。
标记必须位于完整数据的有效区间内，否则转换失败且不发布部分输出。

需要两套独立数据集时，可以分别运行：

```bash
./ops/run/convert_recording.sh SOURCE_BAG PREFIX_DATASET --segments milestones
./ops/run/convert_recording.sh SOURCE_BAG FULL_DATASET --segments full
```

任务描述可以以后再填。默认前缀任务名为 `milestone_1` 等占位编号，完整任务名
沿用配置 `task: teleoperation`。在 `data_collection/config/convert_gello_lerobot_v2.yaml`
补充以下配置后重新转换，即可批量赋予任务描述，无需修改原始 bag：

```yaml
segment_tasks:
  milestone_1: 把物体拿起
  full: 把物体拿起并放进盒子
```

`segment_tasks` 优先于 `task` / `--task`；`--task` 只提供完整 episode 的默认描述。
`meta/episodes.jsonl` 和 `meta/conversion_metadata.json` 保存 `segment_id` 与
`source_recording_id`，同一次采集的前缀和完整数据共享该 ID；训练/验证划分须按该 ID
分组。原始 bag 只存一份，导出的不同 episode 各有自己的媒体文件。

本版本仍要求整包校验通过，不会从 `incomplete` / `interrupted` / `discarded` bag
自动提取前半段；完成标记不等同于数据质量校验通过。标记错误时可用 `--segments full`
只转换完整数据。

脚本将源 bag 只读挂载；转换前后比较源文件大小和 mtime。输出写到相邻临时目录，
54/108 维、joint order、有限值、严格单调时间、三视频、头部 raw16 深度和
provenance 全部验证通过后才原子发布。失败时不改源 bag，也不暴露目标目录中的部分数据。
默认转换取所有 required stream 的 source 公共区间，再裁掉开头 1 秒和结尾 0.2 秒；Parquet
训练时间戳从裁剪后区间的 0 秒重新开始。原始 bag 始终保持完整。

详细 topic、顺序和时间策略见
[`docs/DATA_FORMAT.md`](docs/DATA_FORMAT.md)。

代码已落地与仍待验收的条目见仓库根目录 [`采集进度.md`](../采集进度.md)。
