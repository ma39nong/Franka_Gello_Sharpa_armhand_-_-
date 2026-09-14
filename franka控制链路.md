# Franka FR3 控制链路

本文说明本仓库在 GELLO 遥操作模式下，如何从 Python 代码一路驱动双 Franka FR3。

## 总体结论

这套系统不是由 Python 直接调用 libfranka。完整链路是：

```text
Python 读取 GELLO、生成目标关节角
    ↓
本机 UDP
    ↓
ROS 2 Bridge
    ↓
ROS 2 Safety Gateway
    ↓
ROS 2 Joint Splitter
    ↓
C++ ros2_control 关节阻抗控制器
    ↓
franka_hardware
    ↓
libfranka / FCI
    ↓
Franka FR3
```

更准确地说：Python 负责生成目标关节角；ROS 2 负责传递、仲裁和安全检查；仓库中的 C++ 实时控制器把位置误差转换成关节力矩；`franka_hardware` 通过 libfranka 和 FCI 将力矩发送给 FR3。

## 完整数据流

```mermaid
flowchart LR
    G[GELLO 关节设备] -->|USB 串口| P[Python Operator<br/>100 Hz]
    P -->|UDP 5560<br/>目标关节角| B[Python ROS 2 Bridge]
    B -->|ArmCommand<br/>/teleop/arm_commands| S[Python Safety Gateway]
    S -->|JointState<br/>/target_robot/joint_commands| J[Python Joint Splitter]
    J -->|/left/right/gello/joint_states| C[C++ Joint Impedance Controller<br/>约 1 kHz]
    C -->|effort command interfaces| H[franka_hardware<br/>ros2_control 硬件插件]
    H -->|libfranka / FCI<br/>实时以太网| R[Franka FR3]
```

## 1. 启动系统

标准遥操作入口是：

```bash
./ops/run/start_teleop.sh
```

这个脚本启动以下主要服务：

```text
franka-control
teleop-control
moveit-ik
preset-ik
gello-bridge
hand-control
Python Operator 和 GUI
```

其中与 FR3 实时控制直接相关的是：

- `franka-control`：启动双臂 ros2_control、Franka 硬件插件和 C++ 阻抗控制器。
- `teleop-control`：启动 Safety Gateway 和 Joint Splitter。
- `gello-bridge`：把 Operator 发出的 UDP 关节命令转换为 ROS 2 `ArmCommand`。
- Python Operator：读取 GELLO，计算目标关节角。

相关入口：

- `ops/run/start_teleop.sh`
- `ops/run/run_operator.sh`
- `ops/run/run_teleop.sh`
- `docker/compose.yaml`

## 2. Python 读取 GELLO

`ops/run/run_teleop.sh` 默认以以下参数启动 Operator：

```text
--arm-source gello
--gello-config config/modes/gello.yaml
```

最终执行：

```bash
python -m teleop_runtime.cli
```

`teleop_runtime/cli.py` 创建 `DualGelloJointInput`。底层通过 Dynamixel/OpenRB 串口读取左右 GELLO，每侧得到 7 个关节角。

GELLO 的硬件和映射配置位于：

```text
config/modes/gello.yaml
```

其中包括：

- 左右 GELLO 的稳定串口路径和序列号
- 电机 ID
- 关节方向
- 关节灵敏度
- 单次 Engage 的相对运动范围
- FR3 关节限位 margin
- GELLO 状态过期时间
- 目标关节最大速度

## 3. Python 生成 FR3 目标关节角

主要控制类是：

```text
adapters/pico/src/pico_bimanual_franka_teleop/hardware.py
```

对应类：

```python
DualFr3HardwareTeleop
```

控制循环默认运行在 100 Hz，配置位于：

```yaml
host:
  control_rate: 100.0
```

对于 GELLO 输入，系统采用增量关节映射：

```text
开始 Engage 时：
    记录 GELLO 当前角度
    记录 FR3 当前实测角度

之后：
    FR3 目标角度
      = Engage 时的 FR3 角度
      + GELLO 相对位移 × 灵敏度和方向
```

这种设计意味着 Engage 时不会要求 GELLO 和 FR3 处在相同的绝对角度，也能避免开始跟随时机械臂突然跳动。

每个控制周期最终调用：

```python
self.robot.send_command(self.hold_q, active_sides)
```

其中：

- `self.hold_q` 是左右臂共 14 个目标关节角。
- `active_sides` 表示当前允许控制左臂、右臂或双臂。

## 4. Python Operator 通过 UDP 发命令

Operator 进程不直接使用 ROS 2，而是由 `UdpRobotBackend` 通过本机 UDP 发送命令。

代码位置：

```text
adapters/pico/src/pico_bimanual_franka_teleop/robot_udp.py
```

默认网络配置：

```yaml
udp:
  command_host: 127.0.0.1
  command_port: 5560
  state_host: 127.0.0.1
  state_port: 5561
  state_timeout: 0.25
```

命令包包含：

- `kind=command`
- 唯一的 `stream_id`
- 单调递增的 `sequence`
- 时间戳
- 当前激活的机械臂
- 关节名称
- 目标关节角

`stream_id` 和 `sequence` 用于识别新会话、拒绝旧包和乱序包。

## 5. ROS 2 Bridge 把 UDP 转换成 ArmCommand

Docker 服务 `gello-bridge` 运行：

```text
pico_teleop_bridge
```

虽然 ROS 包名中包含 `pico`，它同时也是 GELLO、PICO 和 VIVE 的统一 UDP-to-ROS bridge；启动 GELLO 模式时，`source_id` 被设置为 `gello`。

代码位置：

```text
ros_ws/src/pico_teleop_bridge/pico_teleop_bridge/node.py
```

Bridge 每 10 ms 执行一次 `_tick()`：

1. 接收最新 UDP command packet。
2. 验证 packet 类型、stream 和 sequence。
3. 构造 `teleop_interfaces/msg/ArmCommand`。
4. 发布到 `/teleop/arm_commands`。

`ArmCommand` 定义为：

```text
std_msgs/Header header
string source
string session_id
uint64 sequence
string[] active_sides
string[] joint_names
float64[] positions
```

关节名称采用仓库统一契约：

```text
left_fr3v2_joint1 ... left_fr3v2_joint7
right_fr3v2_joint1 ... right_fr3v2_joint7
```

## 6. Safety Gateway 检查命令

`teleop-control` 启动：

```text
teleop_safety_gateway
joint_splitter
```

启动文件：

```text
ros_ws/src/teleop_core/launch/control.launch.py
```

Safety Gateway 的代码位于：

```text
ros_ws/src/teleop_core/teleop_core/safety_gateway.py
```

它订阅：

```text
/teleop/arm_commands
/left/franka/joint_states
/right/franka/joint_states
/left/franka_robot_state_broadcaster/external_joint_torques
/right/franka_robot_state_broadcaster/external_joint_torques
/reset_to_initial_pose/active
```

Safety Gateway 主要检查：

1. `source` 是否属于允许的控制源。
2. `session_id` 是否有效。
3. `sequence` 是否单调递增。
4. 输入命令是否过期。
5. 双臂实测状态是否存在并且足够新鲜。
6. `active_sides`、关节名称和位置数组是否严格匹配。
7. 目标是否超过 FR3 关节限位。
8. 一侧第一次 Engage 时，目标与实测位置是否足够接近。
9. 相邻输出之间的关节变化是否超过 slew limit。
10. 外部关节力矩过大时，命令是否还在向接触物继续施压。

主要参数位于：

```text
config/modes/teleop_control.yaml
```

当前关键值为：

```yaml
state_timeout: 0.25
command_timeout: 0.25
max_joint_speed: 0.7
max_initial_delta: 0.05
nominal_dt: 0.01
contact_torque_thresholds: [6.0, 6.0, 6.0, 6.0, 3.0, 3.0, 3.0]
```

命令通过检查后，Safety Gateway 发布两个 topic：

```text
/teleop/validated_arm_commands
/target_robot/joint_commands
```

其中 `/target_robot/joint_commands` 是真正面向机械臂控制器的内部命令总线。按照本仓库架构约束，只有 Safety Gateway 可以发布该 topic。

## 7. Joint Splitter 拆分左右臂命令

代码位置：

```text
ros_ws/src/teleop_core/teleop_core/joint_splitter.py
```

Joint Splitter 订阅：

```text
/target_robot/joint_commands
```

然后将双臂命令拆成：

```text
/left/gello/joint_states
/right/gello/joint_states
```

同时把带左右前缀的统一关节名称转换为单臂控制器需要的名称：

```text
fr3_joint1
fr3_joint2
...
fr3_joint7
```

如果某一侧没有出现在当前命令中，就不会向该侧控制器发布新目标。

这里的 `gello/joint_states` 是历史遗留的 topic 名称。它表达的是“关节阻抗控制器的目标关节角输入”，并不表示 C++ 控制器直接读取 GELLO 硬件。

## 8. C++ 关节阻抗控制器

每个 FR3 都运行一个仓库自带的 C++ ros2_control 控制器：

```text
franka_fr3_arm_controllers/JointImpedanceController
```

实现位于：

```text
ros_ws/src/franka_fr3_arm_controllers/src/joint_impedance_controller.cpp
```

控制器订阅其 namespace 下的：

```text
gello/joint_states
```

因此完整 topic 是：

```text
/left/gello/joint_states
/right/gello/joint_states
```

收到 ROS 消息后，回调函数只负责：

1. 检查位置数组和关节名称。
2. 按 `fr3_joint1..7` 排序目标。
3. 记录消息时间和接收时间。
4. 将目标写入 `realtime_tools::RealtimeBuffer`。

真正的控制计算发生在 ros2_control 的实时 `update()` 中。

控制器从 ros2_control state interfaces 读取：

```text
每个关节的 position
每个关节的 velocity
```

然后根据目标位置计算关节力矩：

```text
tau = K × (q_goal - q) - D × dq_filtered
```

其中：

- `q_goal`：Python 和 ROS 2 链路送来的目标关节角。
- `q`：FR3 当前实测关节角。
- `dq_filtered`：滤波后的实测关节速度。
- `K`：关节刚度。
- `D`：关节阻尼。
- `tau`：最终目标关节力矩。

控制器还将力矩限制在 FR3 的范围内：

```text
J1-J4: ±87 Nm
J5-J7: ±12 Nm
```

计算完成后，控制器写入 7 个 effort command interfaces：

```cpp
command_interfaces_[i].set_value(tau_d_calculated(i));
```

控制增益位于：

```text
ros_ws/src/franka_fr3_arm_controllers/config/controllers.yaml
```

当前配置为：

```yaml
k_gains: [600, 600, 600, 600, 250, 150, 50]
d_gains: [30, 30, 30, 25, 25, 25, 15]
```

控制器还包含两层超时保护：

- ROS 消息时间戳无效时拒绝目标。
- 约 0.25 秒没有收到新命令时，不再继续旧运动，改为保持当前实测位置。

## 9. ros2_control 实时循环

本仓库使用自定义的 controller manager 可执行程序：

```text
ros_ws/src/franka_fr3_arm_controllers/src/franka_ros2_control_node.cpp
```

它的实时线程循环执行：

```cpp
cm->read(...);
cm->update(...);
cm->write(...);
```

三步的含义是：

1. `read()`：硬件插件读取 FR3 最新状态。
2. `update()`：运行 Joint Impedance Controller，计算下一帧目标力矩。
3. `write()`：硬件插件向 FR3 发送目标力矩。

`controllers.yaml` 请求 1001 Hz 的 host update rate。Franka 硬件接口的 `read()` 会等待下一帧 FCI 状态，因此实际循环由机器人节拍驱动，约为 1 kHz。

左右臂 controller manager 被放在指定 CPU 上，并使用实时调度优先级，以降低 FCI 通信抖动。

## 10. franka_hardware 和 libfranka

启动时，以下 launch 文件处理 Franka 硬件配置：

```text
ros_ws/src/franka_fr3_arm_controllers/launch/robot_control.launch.py
ros_ws/src/franka_fr3_arm_controllers/launch/franka_fr3_arm_controllers.launch.py
ros_ws/src/franka_fr3_arm_controllers/launch/franka.launch.py
```

`franka.launch.py` 使用上游 `franka_description` 生成包含 ros2_control 配置的 URDF，并传入：

- `arm_id`
- `arm_prefix`
- `robot_ip`
- `use_fake_hardware`
- `fake_sensor_commands`

在真机模式下，URDF 加载上游 `franka_hardware` ros2_control 硬件插件。

本仓库的 C++ 阻抗控制器并不直接创建 `franka::Robot`，它只读写 ros2_control 的 state/command interfaces。真正调用 libfranka 的是 `franka_hardware`。

职责关系如下：

```text
JointImpedanceController
    ↓ ros2_control effort interfaces
franka_hardware
    ↓ libfranka API
libfranka
    ↓ FCI 实时以太网协议
FR3 控制柜
    ↓
机械臂电机
```

Docker 镜像安装的版本为：

```text
libfranka 0.19.0
franka_ros2 2.0.4
ROS 2 Humble
```

相关构建定义位于：

```text
docker/Dockerfile
```

## 11. FR3 网络地址

双臂配置位于：

```text
config/workcell/current.yaml
```

当前配置为：

```text
左臂：172.16.0.2
右臂：172.16.1.2
```

`franka-control` 容器以 host network 和 privileged 模式运行，并将这个配置文件交给 Franka launch。libfranka 最终使用这些地址连接左右 FR3 控制柜。

## 12. 状态反馈链

机械臂状态沿相反方向返回：

```text
FR3
    ↓ FCI
libfranka
    ↓
franka_hardware state interfaces
    ↓
joint_state_broadcaster
    ↓
/left/franka/joint_states
/right/franka/joint_states
    ↓
Safety Gateway 和 UDP Bridge
    ↓ UDP 5561
Python Operator
```

Franka Robot State Broadcaster 还发布外部关节力矩：

```text
/left/franka_robot_state_broadcaster/external_joint_torques
/right/franka_robot_state_broadcaster/external_joint_torques
```

Safety Gateway 使用这些外力矩判断当前命令是否可能继续向障碍物施压。

Python Operator 每个周期都会读取真实 FR3 状态，并根据实测位置计算下一帧目标。因此它是带状态反馈的控制链，而不是只发送一条开环轨迹。

## 13. 各层职责总结

| 层 | 语言 | 频率 | 主要职责 |
| --- | --- | ---: | --- |
| GELLO reader / Operator | Python | 100 Hz | 读取 GELLO，执行增量映射，生成目标关节角 |
| UDP ROS Bridge | Python / rclpy | 100 Hz | UDP 与 ROS 2 `ArmCommand` 互转 |
| Safety Gateway | Python / rclpy | 随命令约 100 Hz | 控制源仲裁、限位、限速、新鲜度和接触检查 |
| Joint Splitter | Python / rclpy | 随命令约 100 Hz | 将双臂命令拆成左右单臂目标 |
| Joint Impedance Controller | C++ / ros2_control | 约 1 kHz | 将目标关节角转换成目标关节力矩 |
| franka_hardware | C++ / ros2_control | 约 1 kHz | 在 ros2_control 接口和 libfranka 之间传递状态与命令 |
| libfranka / FCI | C++ / 网络协议 | 约 1 kHz | 与 FR3 控制柜进行实时通信 |

## 14. 建议阅读顺序

如果要顺着一次命令逐层学习，建议按以下顺序阅读：

1. `ops/run/start_teleop.sh`
2. `ops/run/run_teleop.sh`
3. `teleop_runtime/cli.py`
4. `adapters/pico/src/pico_bimanual_franka_teleop/gello_input.py`
5. `adapters/pico/src/pico_bimanual_franka_teleop/hardware.py`
6. `adapters/pico/src/pico_bimanual_franka_teleop/robot_udp.py`
7. `ros_ws/src/pico_teleop_bridge/pico_teleop_bridge/node.py`
8. `ros_ws/src/teleop_core/teleop_core/safety_gateway.py`
9. `ros_ws/src/teleop_core/teleop_core/safety.py`
10. `ros_ws/src/teleop_core/teleop_core/joint_splitter.py`
11. `ros_ws/src/franka_fr3_arm_controllers/src/joint_impedance_controller.cpp`
12. `ros_ws/src/franka_fr3_arm_controllers/src/franka_ros2_control_node.cpp`
13. `ros_ws/src/franka_fr3_arm_controllers/launch/franka.launch.py`
14. `docker/Dockerfile`

## 一句话总结

本仓库的 Python 代码通过 UDP 和 ROS 2 发出目标关节角；ROS 2 中的安全节点校验并拆分命令；C++ ros2_control 阻抗控制器把目标角转换成关节力矩；`franka_hardware` 再调用 libfranka，通过 FCI 驱动 Franka FR3。
