# GELLO + Wuji 遥操数据录制与 ROS bag 格式说明

> 适用范围：本仓库 `data_collection` 下的 Harvest 采集链，即 **GELLO 双臂 +
> Wuji 双手 + 三台 Orbbec 相机**。本文不描述另一套 O30i/G20
> `ros_ws/src/teleop_data` 录制器。
>
> 核查日期：2026-09-10。仓库内没有提交真实采集 episode；本文的 topic、类型、
> 频率和文件布局来自当前录制配置与实现。消息数、录制时长、实际 RGB
> `encoding` 等运行时信息，应对具体 bag 执行 `ros2 bag info` 后填写或确认。
> 当前工作树也没有本机部署文件 `data_collection/config/cameras.yaml`；正式录制前
> 需由 `cameras.yaml.example` 生成本机文件，填好三台相机唯一序列号和物理语义。

## 1. 一句话结论

这套功能在线只读取遥操系统的数据，并调用 `ros2 bag record` 保存
原始 ROS 2 topic；不控制机器人、不改写消息，也不自动删除原始数据。

正式入口是：

```bash
./ops/run/start_recording.sh \
  --data-root /home/user/franka_teleop_data
```

默认原始数据保存在：

```text
/home/user/franka_teleop_data/bags/gello/episode0
/home/user/franka_teleop_data/bags/gello/episode1
...
```

录制时按 `SPACE` 开始/停止；遥操 UI 中也可用“开始录制/结束并校验”按钮或快捷键
`L`。`D`（UI 中为“丢弃最近一次”，快捷键 `A`）只把 episode 标记为
`discarded`，不会删除源文件。

### 录制环境：ROS 2 Humble + Docker

当前仓库把 **ROS 2 Humble** 定为标准录制环境，并通过 Docker 启动：

- 镜像 `franka-upper-body-teleop:latest` 基于 `ros:humble-ros-base`；
- 镜像安装 `ros-humble-rosbag2` 和
  `ros-humble-rosbag2-storage-default-plugins`；
- 容器 entrypoint 会依次 source `/opt/ros/humble/setup.bash`、vendor workspace 和
  本仓库的 `ros_ws/install/setup.bash`；
- 宿主机的 `start_recording.sh` 最终执行
  `docker compose run --rm data-collection`。

`data-collection` 是临时容器，但 bag 目录会通过 bind mount 写回宿主机的
`--data-root`；因此容器退出并被 `--rm` 删除后，原始 bag 仍然保留。容器使用
host network、Cyclone DDS、`ROS_LOCALHOST_ONLY=1` 和 `docker/.env` 中的
`TELEOP_ROS_DOMAIN_ID`，以便和同机已运行的遥操/控制 ROS graph 通信。

正式录制应使用上述入口和镜像，不建议用其他 ROS 2 发行版在宿主机直接
调用 recorder。这样可以固定 rosbag2 SQLite 插件、自定义消息定义、DDS 实现和 QoS
行为。其他环境可以作为离线读取端，但必须能正确解析 Humble 生成的 bag 和本仓库
的自定义消息。

### 原始数据统一口径

团队内若要把录制包作为可交付的统一原始数据，建议直接以下列为契约：

- 契约版本：`bag_contract_version=1`；
- 录制环境：本仓库 `franka-upper-body-teleop:latest` 镜像中的 ROS 2 Humble；
- 存储：rosbag2 SQLite3，serialization format 为 CDR，当前不启用 bag 压缩或分片；
- topic：必须包含本文的 13 个必需流，4 个 `CameraInfo` 为可选流；
- 命令数据：只保存 safety gateway 后的
  `/teleop/validated_arm_commands`，不把 gateway 前的 `/teleop/arm_commands`
  或硬件命令总线 `/target_robot/joint_commands` 当作原始 action；
- 关节数据：按 joint name 解析，不依赖消息数组的当前下标顺序；
- 时间：同时保留 bag receive timestamp 和消息 `header.stamp`，不在原包内重采样；
- 深度：`cam0` 保存 640×400、`16UC1` packed uint16 原始帧，不做 D2C
  注册、去畸变或点云化；
- 交付目录：必须保留 `metadata.yaml`、所有 `.db3` 文件和
  `collection_state.json`；
- 合格标志：只有 `collection_state.json` 中 `state=finalized` 且
  `finalized=true` 的 episode 算合格原始包。

若以后更改 topic、消息类型、关节集合、存储插件或时间语义，应升级
`bag_contract_version`，而不要在版本号不变的情况下混用两种格式。

## 2. 录制链路

```text
GELLO/MANUS Operator
  ├─ 臂命令 → safety gateway
  │             ├─ /teleop/validated_arm_commands
  │             └─ /teleop/arm_command_status
  └─ Wuji command/state → 本机 UDP 5602
                          └─ teleop_hand_telemetry → 手部 ROS topics

双 FR3 实测 joint state ─────────┐
三台 Orbbec 图像 ───────────────┼─→ ros2 bag record → episodeN/
安全后的臂 action ──────────────┤
Wuji command/state/status ──────┘
```

采集器必须等 13 个必需 topic 的名称和 ROS 类型连续稳定 2 秒后才显示
`READY`。停止录制后，它会逐条读取 bag，检查消息内容、频率、时间戳连续性、最大
间隔、双侧 engagement、手部丢包计数等，再写入 episode 状态。

## 3. 一个原始 episode 的磁盘格式

当前参数明确使用 ROS 2 Humble `rosbag2` 的 SQLite3 存储插件：

```text
episodeN/
├── metadata.yaml             # rosbag2 原生元数据：topic、类型、消息数、时长等
├── episodeN_0.db3            # SQLite3 数据库；当前未配置按大小/时长分片
└── collection_state.json     # 本项目附加的采集来源、状态和校验报告
```

实际命令等价于：

```bash
ros2 bag record \
  --output <data-root>/bags/gello/episodeN \
  --storage sqlite3 \
  --max-cache-size 268435456 \
  --qos-profile-overrides-path <临时生成的 qos.yaml> \
  <下文列出的 17 个 topic>
```

其中 13 个 topic 是必需流，4 个 `CameraInfo` 是可选流。可选表示“缺少时仍可完成
episode”，但只要相机发布了它们，bag 仍会记录。

`.db3` 并不是把关节或图像展开成 SQL 数值列。数据库中的每条 message 主要由
`topic_id`、bag receive timestamp（纳秒）和 CDR 序列化 BLOB 组成；消息的字段结构
由 topic 对应的 ROS type 决定。读取数据时应优先使用 `ros2 bag`、`rosbag2_py` 或
`rosbags` 做反序列化，不要直接解析 BLOB。

所有录制订阅统一覆盖为：

```yaml
history: keep_last
depth: 10
reliability: reliable
durability: volatile
```

## 4. bag 内的 topic 清单

### 4.1 13 个必需流

| 类别 | Topic | ROS type | 期望源频率下限 | 主要内容 |
| --- | --- | --- | ---: | --- |
| 臂 action | `/teleop/validated_arm_commands` | `sensor_msgs/msg/JointState` | 5 Hz | safety gateway 校验通过、实际发送到控制链的双臂目标位置 |
| 臂状态 | `/teleop/arm_command_status` | `teleop_interfaces/msg/ArmCommandStatus` | 5 Hz | 命令来源、session、sequence、当前被接受的左右侧、fault |
| 左臂实测 | `/left/franka/joint_states` | `sensor_msgs/msg/JointState` | 10 Hz | 左 FR3 7 关节实测位置和速度 |
| 右臂实测 | `/right/franka/joint_states` | `sensor_msgs/msg/JointState` | 10 Hz | 右 FR3 7 关节实测位置和速度 |
| 左手 action | `/teleop/wuji/left/command` | `sensor_msgs/msg/JointState` | 10 Hz | 实际发给左 Wuji 手的 20 关节目标位置 |
| 右手 action | `/teleop/wuji/right/command` | `sensor_msgs/msg/JointState` | 10 Hz | 实际发给右 Wuji 手的 20 关节目标位置 |
| 左手实测 | `/teleop/wuji/left/joint_states` | `sensor_msgs/msg/JointState` | 10 Hz | 左手 20 关节位置及有限差分速度 |
| 右手实测 | `/teleop/wuji/right/joint_states` | `sensor_msgs/msg/JointState` | 10 Hz | 右手 20 关节位置及有限差分速度 |
| 手部状态 | `/teleop/wuji/telemetry_status` | `teleop_interfaces/msg/HandTelemetryStatus` | 左右侧各 10 Hz | 左右手消息交错在同一 topic，记录 engagement、有效性、freshness 和丢包统计 |
| 头部 RGB | `/cam0/color/image_raw` | `sensor_msgs/msg/Image` | 验收下限 18 Hz | 配置为 640×400、20 Hz |
| 头部深度 | `/cam0/depth/image_raw` | `sensor_msgs/msg/Image` | 验收下限 18 Hz | 640×400、20 Hz、`16UC1`、packed uint16 原始深度 |
| 相机 1 RGB | `/cam1/color/image_raw` | `sensor_msgs/msg/Image` | 10 Hz | 配置为 640×480、30 Hz |
| 相机 2 RGB | `/cam2/color/image_raw` | `sensor_msgs/msg/Image` | 10 Hz | 配置为 640×480、30 Hz |

所有必需流的允许最大 source-header 内部间隔均为 150 ms。表中的频率是“episode
通过校验的最低频率”，不一定等于发布器标称频率。

RGB 的分辨率和标称帧率由相机配置确定，但驱动的 `color_format` 当前为 `ANY`，
录制器也未锁死 RGB `Image.encoding`。因此真正使用某个 bag 前，应查看其中
`sensor_msgs/Image.encoding`；不能仅根据配置假定一定是 `rgb8` 或 `bgr8`。

### 4.2 4 个可选相机内参流

| Topic | ROS type | 必需性 |
| --- | --- | --- |
| `/cam0/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 可选 |
| `/cam0/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 可选 |
| `/cam1/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 可选 |
| `/cam2/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 可选 |

当前采集不记录 IR、点云、彩色点云，也不做 depth-to-color 注册或去畸变。只有
`cam0` 记录深度，`cam1` 和 `cam2` 只记录 RGB。

## 5. 各 ROS 消息的数据结构

### 5.1 `sensor_msgs/msg/JointState`

```text
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
```

本项目按 joint name 取值，禁止依赖数组下标猜测语义。

- `/teleop/validated_arm_commands`：`position` 是 safety gateway 校验后的实际 arm action；
  消息可能只含
  当前 active side 的 7 个关节，也可能同时含左右 14 个关节。左、右命令名分别为
  `left_fr3v2_joint1..7` 和 `right_fr3v2_joint1..7`。
- `/{left,right}/franka/joint_states`：要求恰好包含对应侧
  `{left,right}_fr3_joint1..7`，并且 `position`、`velocity` 都是 7 个有限数值。bag
  保存原始消息的全部字段。
- `/teleop/wuji/{left,right}/command`：要求 20 个 joint name 和 20 个
  `position`。这是已经发出的手部 command，不是 MANUS 原始 landmarks。
- `/teleop/wuji/{left,right}/joint_states`：要求 20 个 `position` 和 20 个
  `velocity`。Wuji SDK 当前只有位置反馈，速度由 receiver 对同名、时间戳严格递增的
  相邻样本做有限差分。第一帧没有速度基线，因此不发布 state；重复或倒退时间戳也不
  参与差分。

Wuji 每侧的固定 20 关节顺序为：

```text
thumb_cmc_flex, thumb_cmc_abd, thumb_mcp, thumb_ip,
index_finger_mcp_flex, index_finger_mcp_abd, index_finger_pip, index_finger_dip,
middle_finger_mcp_flex, middle_finger_mcp_abd, middle_finger_pip, middle_finger_dip,
ring_finger_mcp_flex, ring_finger_mcp_abd, ring_finger_pip, ring_finger_dip,
pinky_mcp_flex, pinky_mcp_abd, pinky_pip, pinky_dip
```

左侧每个名字加 `l_` 前缀，右侧加 `r_` 前缀。

### 5.2 `teleop_interfaces/msg/ArmCommandStatus`

```text
std_msgs/Header header
string source
string session_id
uint64 sequence
string[] accepted_sides
string[] faults
```

`accepted_sides` 只允许 `left`、`right`。它是双臂是否真实跟随的依据；空列表表示
没有一侧被接受。`faults` 非空会使当前 bag 校验失败。

### 5.3 `teleop_interfaces/msg/HandTelemetryStatus`

```text
std_msgs/Header header
string session_id
uint64 sequence
string side
builtin_interfaces/Time source_wall_stamp
uint64 source_monotonic_ns
uint64 command_monotonic_ns
uint64 state_monotonic_ns
bool engaged
bool command_valid
bool state_valid
float32 command_freshness_sec
float32 state_freshness_sec
string state_velocity_source
uint64 sender_dropped_packets
uint64 receiver_lost_packets
uint64 receiver_duplicate_packets
uint64 receiver_out_of_order_packets
uint64 receiver_stale_packets
uint64 receiver_invalid_packets
```

同一 topic 中用 `side` 区分左右手。计数器是 receiver 进程生命周期累计值；episode
验收使用“末值减基线”的增量，录制前已有但录制期间不再增长的历史错误不会使本条
episode 失败。有效 hand state 的 `state_velocity_source` 必须是
`finite_difference`。

### 5.4 `sensor_msgs/msg/Image`

```text
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
```

头部深度额外执行严格校验：`width=640`、`height=400`、
`encoding="16UC1"`、`step=width*2`、`len(data)=step*height`。因此每个未压缩深度帧
的 payload 是 512,000 字节。深度值按原始 uint16 保存；录制链没有在这里定义米制
缩放，使用时应结合具体相机型号/SDK 标定解释数值单位。

RGB 也是原始 `sensor_msgs/Image` 消息，不是压缩图像 topic。每帧字节数由实际
`encoding` 和 `step` 决定。

### 5.5 `sensor_msgs/msg/CameraInfo`

主要包括图像宽高、畸变模型和参数 `d`、内参矩阵 `k`、校正矩阵 `r`、投影矩阵
`p`、binning 和 ROI。它们是可选流；若具体 bag 中缺少，必须从相机标定/部署记录
另外取得，不能假定 bag 一定自包含全部内参。

## 6. 时间戳和对齐语义

每条数据同时保留两类时间：

- rosbag receive time：rosbag2 把消息收到并写入 bag 时的纳秒时间戳，位于 `.db3`
  message record 中。
- source/header time：ROS 消息的 `header.stamp`。内部连续性校验主要使用它。

手部 command/state 的 `header.stamp` 来自 Operator 数据包的 source wall time；手部
status 的 `header.stamp` 是 receiver wall time，同时另存 `source_wall_stamp` 和三个
monotonic 时间。相机、FR3 state 和安全网关消息保留各自发布器写入的 header 时间。

默认从全部必需流的 source-header 公共区间中，再裁掉开头 1.0 秒和结尾 0.2 秒作为
有效校验区间。这个“裁剪”只影响校验边界，原始 bag 本身不会被裁剪或修改。
bag receive time 的大间隔作为
`transport_warnings` 记录；source-header 的低频、非单调或大于 150 ms 的内部间隔
会影响 episode 是否能 `finalized`。

对 action 的检查是 engagement-aware：

- `engaged=true` 时，该侧必须有新鲜 action，不能静默补零或复制另一侧。
- 明确 `engaged=false` 表示 HOLD，允许原始 action topic 暂时没有该侧数据。
- 原始 bag 只保存真实的 action 和 engagement 状态，不伪造 HOLD 期间的命令。

## 7. `collection_state.json`

这是本项目放在 rosbag 目录里的附加 JSON，不属于 rosbag2 标准文件。主要字段包括：

```json
{
  "bag_contract_version": 1,
  "teleoperator": "gello",
  "workcell_id": "gello-dual-fr3-wuji",
  "workcell_config_hash": "<sha256>",
  "control_config_id": "gello-incremental-safety-gateway-v1",
  "timestamp_policy": "preserve_source_header_and_bag_receive_time",
  "trim_start_sec": 1.0,
  "trim_end_sec": 0.2,
  "calibration_ids": {"...": "..."},
  "device_identities": {
    "cam0_serial": "...",
    "cam0_semantic": "...",
    "cam1_serial": "...",
    "cam1_semantic": "...",
    "cam2_serial": "...",
    "cam2_semantic": "..."
  },
  "source_bag": "<episode 的绝对路径>",
  "source_topic_contract": {"...": "..."},
  "state": "finalized",
  "finalized": true,
  "failures": [],
  "validation_policy": "source_header_continuity_trimmed_boundary_v3",
  "validation_report": {"...": "..."},
  "boundary_warnings": [],
  "transport_warnings": [],
  "updated_at_ns": 0
}
```

`state` 只会是以下五种之一：

| 状态 | 含义 | 是否为合格原始包 |
| --- | --- | --- |
| `recording` | 正在录制 | 否 |
| `finalized` | 正常停止且通过完整性校验 | 是 |
| `incomplete` | recorder 异常或校验失败 | 否 |
| `interrupted` | 录制中被 Ctrl-C/信号中断 | 否 |
| `discarded` | 操作者标记为丢弃；源文件仍保留 | 否 |

`validation_report.streams` 会按 topic 保存消息数、首末 bag time、首末 source time、
频率、最大内部 gap、首尾 gap 和非单调计数；此外还保存 telemetry counter 增量和
各侧 engaged/disengaged 样本数。

## 8. 如何检查一个真实 bag

以下命令只读，不会修改 bag：

```bash
# 1. 查看 ROS 原生概要、topic/type、消息数、时长
ros2 bag info /path/to/episodeN

# 2. 查看本项目的 episode 状态和校验报告
python3 -m json.tool /path/to/episodeN/collection_state.json | less

# 3. 查看 SQLite 文件和 metadata
ls -lh /path/to/episodeN
sed -n '1,240p' /path/to/episodeN/metadata.yaml

# 4. 回放单个低带宽 topic 检查字段；回放图像前注意网络/GUI负载
ros2 bag play /path/to/episodeN \
  --topics /teleop/arm_command_status

# 另开一个已 source ROS 环境的终端
ros2 topic echo /teleop/arm_command_status
```

建议对每个要交付或共享的 bag 至少确认：

- `collection_state.json` 中 `state == "finalized"` 且 `finalized == true`；
- 13 个必需 topic 均存在并有消息；
- `failures` 为空，并审阅 `boundary_warnings`、`transport_warnings`；
- 三个相机 serial/semantic 与实际工位一致；
- 深度确为 640×400 `16UC1`，RGB 的实际 encoding 符合下游解码预期；
- episode 时长和各 topic 消息数符合本次操作预期。

若只想用当前规则重新验收旧 bag，可执行：

```bash
./ops/run/revalidate_recording.sh /path/to/episodeN \
  --trim-start-sec 1 --trim-end-sec 0.2
```

该命令会更新 `collection_state.json`，但不会修改 `.db3` 和 `metadata.yaml`；因此它
不是完全只读检查。

## 9. 代码与配置依据

- 录制 topic、类型、频率和 QoS：
  [`../config/record_gello.yaml`](../config/record_gello.yaml)
- 录制入口和数据目录约束：
  [`../../ops/run/start_recording.sh`](../../ops/run/start_recording.sh)
- `ros2 bag record` 调用、episode 命名和状态写入：
  [`../../ros_ws/src/teleop_data_collector/teleop_data_collector/rosbag_recording_node.py`](../../ros_ws/src/teleop_data_collector/teleop_data_collector/rosbag_recording_node.py)
- 停止后的 bag 内容/时间校验：
  [`../../ros_ws/src/teleop_data_collector/teleop_data_collector/bag_validation.py`](../../ros_ws/src/teleop_data_collector/teleop_data_collector/bag_validation.py)
- 双臂/Wuji topic 与 joint name 单一来源：
  [`../../ros_ws/src/teleop_core/teleop_core/contract.py`](../../ros_ws/src/teleop_core/teleop_core/contract.py)
- 自定义状态消息：
  [`../../ros_ws/src/teleop_interfaces/msg/ArmCommandStatus.msg`](../../ros_ws/src/teleop_interfaces/msg/ArmCommandStatus.msg)、
  [`../../ros_ws/src/teleop_interfaces/msg/HandTelemetryStatus.msg`](../../ros_ws/src/teleop_interfaces/msg/HandTelemetryStatus.msg)
