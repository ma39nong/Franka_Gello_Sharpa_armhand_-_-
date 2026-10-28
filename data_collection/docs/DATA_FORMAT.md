# Harvest / LeRobot v2 数据格式

机器可读契约是 `data_collection/contract/dataset_contract.yaml`；机器人 topic 和
joint name 的代码单一来源仍是 `teleop_core.contract`，测试会检查 YAML 与该来源的
关键映射。第一期 `teleoperator=gello`。Pico 臂若以后接入，只加
`record_pico.yaml`，LeRobot 特征不变，metadata 写 `teleoperator=pico`。

## ROS bag topic/type

| 数据 | Topic | ROS type |
|---|---|---|
| 双臂安全后 action | `/teleop/validated_arm_commands` | `sensor_msgs/msg/JointState` |
| 双臂 engage/校验状态 | `/teleop/arm_command_status` | `teleop_interfaces/msg/ArmCommandStatus` |
| 左/右臂实测 | `/left/franka/joint_states`, `/right/franka/joint_states` | `sensor_msgs/msg/JointState` |
| 左/右手已发送 command | `/teleop/wuji/{left,right}/command` | `sensor_msgs/msg/JointState` |
| 左/右手实测 state | `/teleop/wuji/{left,right}/joint_states` | `sensor_msgs/msg/JointState` |
| 手 telemetry 状态 | `/teleop/wuji/telemetry_status` | `teleop_interfaces/msg/HandTelemetryStatus` |
| 三路 RGB | `/cam{0,1,2}/color/image_raw` | `sensor_msgs/msg/Image` |
| 头部原始深度 | `/cam0/depth/image_raw` | `sensor_msgs/msg/Image` (`16UC1`) |
| 可选相机内参 | `/cam{0,1,2}/color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| 可选头部深度内参 | `/cam0/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |

训练用 arm action 只取 `/teleop/validated_arm_commands`。不要把
`/teleop/arm_commands` 或 `/{side}/gello/joint_states` 当作训练 action。
validated topic 可只含当前 active side；转换器按名字拆分
`left/right_fr3v2_joint*` 并归一化为稳定的 `left/right_fr3_joint*`。在明确
`engaged=true` 的区间，缺失的一侧绝不复制、填零或伪造，超过默认 150 ms 后该
时间点报告 unmatched。只有明确 `engaged=false` 的 HOLD 区间允许离线锁存：使用
该侧最后一个有效 action；如果 episode 开始时该侧已经 disengage，则使用同侧实测
位置初始化。原始 bag 不增加伪命令，也不修改硬件命令链。

## 精确 feature 顺序

单位一律 radian。一律按 joint name 归一化，禁止依赖消息数组下标。

`action: float32[54]`：

1. left arm position：7，`left_fr3_joint1..7`
2. right arm position：7，`right_fr3_joint1..7`
3. left Wuji command：20，按 `teleop_core.contract.WUJI_LEFT_JOINT_NAMES`
4. right Wuji command：20，按 `teleop_core.contract.WUJI_RIGHT_JOINT_NAMES`

`observation.state: float32[108]`：

1. left arm position 7
2. left arm velocity 7
3. right arm position 7
4. right arm velocity 7
5. left hand position 20
6. left hand velocity 20
7. right hand position 20
8. right hand velocity 20

`observation.engaged: float32[4]`：左臂、右臂、左手、右手；1 表示真实跟随，0
表示 disengage/HOLD。该字段默认用于转换审计和采样，不要求作为策略网络输入。

每个输入都按显式 joint name 重排。Wuji SDK 当前只提供位置反馈，因此 receiver 用
相邻、同名且 source monotonic timestamp 严格递增的样本做有限差分；首帧不发布
state，后续消息以 `velocity=finite_difference` 和 status 字段明确标记，绝不以零
冒充实测速度。

图像映射固定为：

```text
observation.images.cam0 <- /cam0/color/image_raw
observation.images.cam1 <- /cam1/color/image_raw
observation.images.cam2 <- /cam2/color/image_raw
observation.depths.cam0 <- /cam0/depth/image_raw (raw16)
```

动作、状态和数据集行使用 30 Hz 时间轴；cam1/cam2 RGB 原生 30 Hz，cam0 RGB
和原始深度原生 20 Hz。cam0 媒体帧各只存储一次，30 Hz 行按时间戳引用最近且不超过
150 ms 的源帧，不生成插值 RGB 或深度。深度不做 D2C 注册、不生成点云、不做去畸变。

cam0/cam1/cam2 的序列号及物理语义写入 `collection_state.json`，并随
`source_provenance` 进入 `meta/conversion_metadata.json`。

## 时间与完整性

录制中的完成标记保存在 `collection_state.json` 的 `milestones` 列表中，例如：

```json
{
  "source_recording_id": "一次原始采集的 UUID",
  "milestones": [
    {"id": "milestone_1", "timestamp_ns": 1780000000123456789, "clock": "ros"}
  ]
}
```

`timestamp_ns` 是采集端处理标记请求时的 ROS 时间，须与机器人和相机 source header
处于同一时钟域；网络请求和人工按键存在延迟，不代表自动检测到任务完成。
任务描述不要求在采集时填写。默认转换按每个标记生成从相同有效开始到该标记的
前缀 episode，再输出完整 episode；`segments: full` / `milestones` 可分别导出。
标记必须严格递增且落在原始录制裁剪后的有效区间内；前缀终点不再应用 `trim_end_sec`。
前缀不会编码标记之后的视频帧，输出行依旧按 30 Hz 网格采样，最后一行不晚于标记。

`meta/episodes.jsonl` 和转换 metadata 中的 `segment_id` 区分 `milestone_1` 与 `full`，
`source_recording_id` 关联共同来源，用于按原始采集划分训练/验证集。旧 bag 没有 UUID
时使用其 provenance 中原始 `source_bag` 路径作分组 ID。`source_provenance` 每个输出
episode 一项，共享来源的条目相同。`source_bags` 只列输入 bag。
`task_index` 与任务描述按 segment 映射，未填写时前缀使用标记 ID 占位。

ROS bag 同时保留原始消息 header timestamp 与 rosbag receive time。状态 sidecar
记录每条 required stream 的首末 receive time、source header 首末值、消息数、
频率、最大内部 gap、首尾 gap 和非单调计数。内部连续性以 source header 时间为准；
默认从 required stream 的 source 公共区间裁掉开头 1 秒和结尾 0.2 秒；裁剪区内不参与训练区间的
连续性判定。录制窗口首尾覆盖使用 bag receive 时间；超出普通边界容差但仍落在裁剪
缓冲区内的缺口记为 `boundary_warnings`。bag receive 的内部大 gap 记录为
`transport_warnings`，用于诊断 DDS/磁盘写入拥塞。
手 telemetry 的 dropped/lost/duplicate/out-of-order/stale/invalid 都是接收进程生命周期
累计计数。校验报告保存每个计数的 `first/last/max/delta`，仅以 episode 内实际增长的
`delta` 判定失败；episode 开始前已经存在且录制期间未增长的历史计数不影响本条数据。
重复或倒退的手反馈时间戳不参与速度差分，接收器会重建该侧速度基线，同时保留该包的
position、command 和 status。
转换先按 source header 排序，取相同公共区间并裁开头 1 秒和结尾 0.2 秒，再从 0 秒构造 30 Hz
确定性时间线，仅选取不超过 150 ms 的
最新历史样本，不使用未来样本；metadata 报告每个 source 的 unmatched 数和被跳过
的不完整行数，以及裁剪前后的绝对 source 时间边界。原始 bag 不被截断或改写。

## LeRobot v2 输出

```text
<dataset>/
├── data/chunk-000/episode_000000.parquet
├── videos/chunk-000/observation.images.cam0/episode_000000.mp4
├── videos/chunk-000/observation.images.cam1/episode_000000.mp4
├── videos/chunk-000/observation.images.cam2/episode_000000.mp4
├── data/.../observation.depths.cam0  # Parquet 内联 raw16 字节与图像元数据
├── meta/info.json
├── meta/episodes.jsonl
├── meta/tasks.jsonl
├── meta/episodes_stats.jsonl
├── meta/conversion_metadata.json
├── README.md
└── dataloader.py
```

## 与 wuji-openpi 的已知差异

`/home/user/llx/harvest_ws/wuji-openpi` 当前与本契约不一致。本任务不修改
OpenPI。接入训练时必须用显式 loader/repack，禁止按数组位置或相似 feature 名
静默兼容。

| 项目 | 本仓库 Harvest/LeRobot v2 | wuji-openpi |
| --- | --- | --- |
| action 54 维顺序 | 左臂 7、右臂 7、左手 20、右手 20 | 左臂 7、左手 20、右臂 7、右手 20 |
| observation.state | 108 维（关节位 + 速度） | 默认约 54 维（关节位） |
| observation.engaged | 4 维审计字段，默认不送入策略 | 无对应字段 |
| 相机 feature | `observation.images.cam{0,1,2}` | `cam_high` / `cam_left_wrist` / `cam_right_wrist` 一类 |
| 头部深度 | `observation.depths.cam0` raw16 | 默认 RGB 训练入口不消费 depth |
