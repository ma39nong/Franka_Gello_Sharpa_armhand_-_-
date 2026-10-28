# Franka 双臂通信问题排查、修复与验证记录

日期：2026-08-17（Asia/Shanghai）  
对象：双 FR3，左臂 `172.16.0.2`，右臂 `172.16.1.2`  
主机：Ubuntu 24.04，Intel Core i9-12900K，`6.8.0-rt8 PREEMPT_RT`

## 1. 当前结论

反复出现的 `communication_constraints_violation` 已定位并修复。最终证据不支持“网线丢包”“DDS 丢 FCI 包”“CPU 抢占”或“机器人本体故障”作为本次主因。

真正的问题是：ros2_control 2.54.0 的主循环按主机时钟固定累加 `1 ms` 截止时间，而 FR3 按自己独立的 1 kHz 时钟发送状态。两只时钟存在很小的频率偏差，使用完全相同的 nominal 1000 Hz 时，状态包到同 ID 命令包的相位会持续漂移。左臂故障前，该延迟从约 `0.05 ms` 线性增长到 `0.72–0.82 ms`；机器人随后触发通信约束反射。逐包检查确认状态 ID 和命令 ID 连续、内核/网卡没有丢包、实时线程没有同期调度尖峰。因此这里丢失的是**机器人控制周期内的发送余量**，不是数据包 ID。

修复是把 controller manager 请求频率设为 `1001 Hz`。`FrankaHardwareInterface::read()` 会阻塞等待下一帧 FCI 状态，因此实际循环仍由机器人稳定地节拍在约 1000 Hz；略快的主机名义频率只防止主机定时器落到机器人后面。修复后：

- 10 分钟双臂逐包测试通过，状态到同 ID 命令延迟稳定在约 `0.05 ms`，没有再随时间漂移；
- 30 分 02 秒双臂静止阻抗保持通过，通信/反射错误为 0；
- 30 分 07 秒 GELLO + Operator + MoveIt IK 双臂核心负载通过，包含左臂、右臂和双臂联动；
- 30 分钟内精确 FCI socket drop 没有新增，左右 NIC drop 均为 0；
- 静止测试停止时两臂均完成 hardware deactivate/shutdown，Franka 容器在 `0.436 s` 内退出；完整栈最终复测的 5 个核心服务也全部退出码 0。

结论边界：静止通信基线和 GELLO/MoveIt 核心负载已经通过；Orbbec 相机当时没有出现在预期的 `192.168.1.10:8090`，空闲第四网口 `enp2s0f3` 也没有 carrier，因此相机流并发尚未测试。当前可以确认 **FR3 通信修复通过、核心遥操作负载通过**，但不能把“含相机的完整系统最终验收”标成已完成。

## 2. 排查中出现的两个独立问题

### 2.1 左臂第三关节靠近下限，不是通信故障

一次启动时左臂立即出现 `joint_velocity_violation`。完整报文检查显示机器人只接受了 4 个主动控制命令就置错误位，命令力矩最大约 `0.026 Nm`，实测速度最大约 `0.0013 rad/s`，报文 ID 和往返延迟均正常。

当时左臂第三关节为 `-2.90061 rad`，FR3 下限为 `-2.90070 rad`，余量只有约 `0.00009 rad`（`0.005°`）。人工把该关节移离限位后，启动恢复正常。这个事件是关节限位附近的安全反射，不能计入通信故障，也没有通过放宽安全限制规避。

### 2.2 1001 Hz 暴露了上游关机竞争

1001 Hz 消除通信相位漂移后，Humble 的标准 `ros2_control_node` 偶发无法退出。抓取到的状态是：主线程等待实时线程，实时线程仍在运行；与此同时 `ControllerManager` 的 rclcpp pre-shutdown callback 已开始拆控制器和硬件。两者并发进入硬件接口会发生时序竞争，最后由 ROS launch 升级到 SIGKILL。

仓库内新增 `franka_ros2_control_node`，同步等待 SIGINT/SIGTERM，执行顺序改为：

1. 停止实时 `read/update/write` 循环；
2. `join` 实时线程；
3. 调用 `rclcpp::shutdown()`；
4. 让 ControllerManager 在没有并发实时访问时停控制器和硬件。

控制器实现、安全网关、碰撞阈值、命令新鲜度、关节限制和 slew 限制均未改变。

验证结果：连续 3 次双臂真机启动/停止耗时 `0.428 s`、`0.464 s`、`0.466 s`，随后 30 分钟测试停止耗时 `0.436 s`；每次两侧都有 `trying to Stop`、`Stopped`、hardware shutdown 和控制进程 clean exit，无 SIGTERM/SIGKILL。完整栈测试又发现并修复了 ROS launch 容器和 Operator 后端的两个外围退出问题，详见 3.6。

## 3. 已实施的修改

### 3.1 ROS 2 全部切换 Cyclone DDS 并限定本机发现

- `docker/Dockerfile` 安装 `ros-humble-rmw-cyclonedds-cpp`。
- `docker/compose.yaml` 的全部 15 个服务统一设置：
  - `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  - `ROS_LOCALHOST_ONLY=1`
  - `CYCLONEDDS_URI=file:///workspace/franka_upper_body_teleop/config/cyclonedds.xml`
- `config/cyclonedds.xml` 使用自动 participant index，上限 32。
- 主机 Jazzy 环境使用 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`。

切换后验证过 16 个并发 participant。回环接口的 `not multicast-capable: disabling multicast` 是预期提示。本项避免 DDS 发现/话题流量进入两张 FCI 专网，并解决 Cyclone 默认 participant index 不足；它应保留，但逐包证据表明它不是 FCI 相位漂移的直接修复。

### 3.2 controller manager 改为 1001 Hz 名义频率

`ros_ws/src/franka_fr3_arm_controllers/config/controllers.yaml`：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1001
    thread_priority: 98
    overruns:
      manage: false
```

`overruns.manage: false` 原本就正确，不是本轮新增。1001 Hz 会让 100/200 Hz broadcaster 打印“不是整数除数”的警告，但 ros2_control 实测仍按最近的 100/200 Hz 更新它们；这只影响非关键遥测调度，不影响 FCI 的实际 1 kHz 节拍。

### 3.3 新增有序关机控制节点

涉及：

- `ros_ws/src/franka_fr3_arm_controllers/src/franka_ros2_control_node.cpp`
- `ros_ws/src/franka_fr3_arm_controllers/launch/franka.launch.py`
- `ros_ws/src/franka_fr3_arm_controllers/CMakeLists.txt`
- `ros_ws/src/franka_fr3_arm_controllers/package.xml`

真机 launch 改用仓库内节点。实现保留 ros2_control 2.54.0 原有 read/update/write、内存锁定、CPU affinity 和 FIFO 设置，只改变信号处理与关闭顺序。

### 3.4 状态 broadcaster 与实时日志

`franka_robot_state_broadcaster` 设置：

```yaml
update_rate: 100
lock_update_success: true
lock_log_error: false
```

非阻塞 publisher 取锁失败时丢弃一次遥测样本，不从 1 kHz 实时路径刷日志。FCI read/write 结果完全不受影响。

自定义 joint impedance controller 的普通 ROS 日志也已从实时 `update()` 移到 1 Hz 非实时 timer；实时路径只更新原子状态。该修改排除了日志锁阻塞，但修改后 1000 Hz 仍可复现通信错误，所以它是正确的实时性清理，不是最终根因。

### 3.5 CPU、IRQ 和 NIC 调优

- 左 controller manager：CPU 8–9；左 I350 IRQ：CPU 11 / FIFO 80。
- 右 controller manager：CPU 12–13；右 I350 IRQ：CPU 15 / FIFO 80。
- FCI 网卡 EEE、RX coalescing、GRO/GSO/TSO 均关闭。
- CPU 8–15 锁定最高频率并禁用退出延迟超过 10 µs 的深 C-state。
- controller manager 实际取得 `SCHED_FIFO 98`。
- housekeeping 为 CPU `0–7,16–23`，不进入 controller/IRQ 核。

`ops/diagnostics/check_franka_cpu_layout.sh` 已验证当前映射。CPU 交叉 A/B 后故障不跟随 CPU，说明这些设置是良好基线，但不是本次相位漂移的根因。

### 3.6 整栈优雅退出

所有由 ROS launch 驱动的服务，包括真/假 Franka、手、teleop、MoveIt、Pico、GELLO、Vive 和 Orbbec，均使用：

```yaml
stop_signal: SIGINT
stop_grace_period: 30s
```

另外完成了三处退出修复：

1. `reset_to_initial_pose.py` 在 rclpy context 已失效时不再做关机后 publish，消除了 `publisher's context is invalid` traceback。
2. `ops/run/run_operator.sh` 判断进程组是否存活时忽略 zombie-only 进程组，避免已经退出的子进程被误判为仍运行并升级信号。
3. Bash 后台启动的 Python 进程继承了 `SIGINT=SIG_IGN`；`teleop_runtime/cli.py` 现在在入口显式恢复 Python 的 SIGINT 默认处理器和 SIGTERM 默认处理器，确保进入现有 `finally`，关闭调试记录、UDP、GELLO、控制服务器等资源。

最终真机、未 engage 的整栈启动/停止复测中，Franka、teleop、GELLO、preset 和 MoveIt 约在 `0.4–1.2 s` 内结束，全部退出码 0、`OOMKilled=false`；两臂均完成 hardware clean shutdown，Operator 输出 `teleop stopped`，没有再升级到 SIGKILL。

## 4. 通信链路逐层证据

### 4.1 物理链路和单臂基线

| 项目 | 左臂 | 右臂 |
|---|---:|---:|
| NIC / IP | `enp2s0f0` / `172.16.0.6` | `enp2s0f1` / `172.16.1.6` |
| Robot IP | `172.16.0.2` | `172.16.1.2` |
| 链路 | 1 Gb/s，全双工 | 1 Gb/s，全双工 |
| 2,000 包并行 ping | 0% loss | 0% loss |
| ping 平均 / 最大 | 0.105 / 0.166 ms | 0.107 / 0.172 ms |
| NIC RX/TX error/drop | 0 | 0 |
| 官方 `communication_test` | 10,000 周期，min/avg/max 1.00 | 10,000 周期，min/avg/max 1.00 |

官方测试会进入主动控制并移动机器人。它证明两条单臂 libfranka + FCI + RT 基线良好，但不能覆盖双臂 ros2_control 长时间相位关系。

### 4.2 网线、CPU 和 IP 交叉的正确解读

排查早期曾观察到错误随一根实体网线交换位置，因而临时怀疑网线。用户随后更换全新网线，错误仍可复现；因此网线不是充分根因，早期短样本的“跟随网线”是相关性线索，不是最终结论。

其他 A/B：

- 交换 controller CPU 后，错误仍出现在连接物理左臂的链路，不跟随 CPU；
- 交换 namespace/IP 后，错误跟随物理链路，不跟随 ROS namespace；
- 新线下逐包捕获显示没有缺失状态 ID 或命令 ID。

最终结论是：换线排除了线缆作为持续故障原因；CPU/IP A/B 排除了 namespace、固定进程和绑核；必须继续深入到每个 1 kHz 周期内的报文时序才能找到根因。

### 4.3 被动 FCI 状态接收

自定义只读 probe 不进入运动控制，同时读取两臂 60 秒：

| 指标 | 左臂 | 右臂 |
|---|---:|---:|
| 样本 | 60,003 | 60,002 |
| 平均间隔 | 0.999973 ms | 0.999984 ms |
| 最大间隔 | 1.118630 ms | 1.109583 ms |
| 大于 2 ms | 0 | 0 |
| 大于 5 ms | 0 | 0 |
| UDP error 增量 | 0 | 0 |

因此 robot → NIC → kernel → libfranka 的被动状态链稳定。

### 4.4 1000 Hz 主动控制故障的逐包结果

在关节已离开限位后，双臂静止主动控制约 196 秒时，左臂发生 `communication_constraints_violation`。故障窗口：

- 左右状态包连续；
- 左右命令包连续；
- 同一报文 ID 一一对应，没有缺命令 ID；
- 左入站最大间隔约 `1.134 ms`，出站最大间隔约 `1.272 ms`；
- pcap kernel drops 为 0；
- 左精确 FCI socket drop 保持 247，右保持 19，稳态没有增长；
- 左 FIFO 98 线程累计 scheduler wait 约 `19.313 ms`，故障前没有尖峰；右约 `0.324 ms`。

唯一持续变化的是左侧状态到同 ID 命令的延迟：

| 主动控制时间 | 中位延迟 |
|---:|---:|
| 约 165 s | 0.384 ms |
| 约 174 s | 0.503 ms |
| 约 180 s | 0.577 ms |
| 约 190 s | 0.684 ms |
| 约 195 s | 0.724 ms（p99 0.776，max 0.825 ms） |

ros2_control 2.54.0 的 `ros2_control_node.cpp` 使用固定 `next_iteration_time += period` 后 `sleep_until(next_iteration_time)`，与上述线性漂移机制吻合。

### 4.5 1001 Hz 对照测试

双臂静止主动控制超过 10 分钟：

- 无 communication fault；
- 左右每 30 秒窗口的同 ID 中位延迟约 `0.046–0.053 ms`；
- p99 小于约 `0.096 ms`，max 小于约 `0.157 ms`；
- 没有随运行时间上升；
- 左右精确 FCI socket drop 仅有启动基线，稳态不增长；
- 两张 pcap 的 kernel drops 都为 0。

这不是简单“提高控制频率”：阻塞式 FCI read 仍把实际循环限制在机器人约 1000 Hz。它只让主机 deadline 永远略早于下一帧机器人状态，从而收到状态后立即回命令。

### 4.6 正式 30 分钟静止测试

本测试只运行 `franka-control`；没有 GELLO、MoveIt、手、相机或运动轨迹。控制器在无外部目标时保持启动姿态。

| 指标 | 结果 |
|---|---:|
| 有效持续时间 | 1,802 s（30 分 02 秒） |
| communication / reflex / 进程错误 | 0 |
| 低频监控样本 | 60 |
| 精确 FCI socket 样本 | 14,022 |
| 左 socket drop | 188 → 188 |
| 右 socket drop | 23 → 23 |
| 左/右最大瞬时 receive queue | 2,304 B / 2,304 B，下一采样清空 |
| 全局 `UdpRcvbufErrors` | 211 → 211 |
| 全局 `UdpInErrors` | 211 → 211 |
| 左右 NIC RX/TX drop | 全部 0，零增长 |
| 停止结果 | 两侧 hardware shutdown + clean exit |
| Compose 停止耗时 | 0.435621 s |

原始证据位于 `docs/diagnostics/franka_static_30min_20260817/`。其中 `runtime.log` 是控制阶段日志，`fci_socket_telemetry_runtime.tsv` 是 0.1 秒级 socket 采样，`telemetry.tsv` 是 30 秒级系统采样，`full.log` 含退出过程。

### 4.7 GELLO + MoveIt 核心负载与运动测试

运行组合为：

- 双臂 `franka-control`；
- `teleop-control` 和 Operator GUI/后端；
- `gello-bridge`；
- `moveit-ik` 和 `preset-ik`。

这里的 MoveIt 只承担碰撞模型/IK 计算负载，不启动 MoveIt 真机 controller；FR3 command bus 仍只有安全网关一个发布者。测试按无运动、左臂、右臂、双臂的顺序进行，每一步均由现场人员 engage/disengage。

| 阶段 | 持续/动作 | FR3 通信结果 | 安全与跟踪结果 |
|---|---:|---|---|
| 核心栈、双臂 disengaged | 5 分 20 秒 | 5 个服务持续 running；通信/反射错误 0；FCI drop 左 `258→258`、右 `20→20` | 无外部目标，保持姿态 |
| 左臂单侧 | engage 约 24.1 秒 | communication/reflex 错误 0；drop 不变 | 接触门限曾在 J4/J1 约 6 Nm 保持目标，外力解除后恢复；没有放宽门限 |
| 右臂单侧 | engage 约 44.0 秒 | communication/reflex 错误 0；drop 不变 | 最大关节目标跨度约 1.25 rad，最大瞬时跟踪差约 0.258 rad；接触门限在 J3 附近保持并恢复 |
| 双臂联动 | engage 约 51.3 秒 | 两侧 feed error 均为空；communication/reflex 错误 0；drop 不变 | 左最大目标跨度约 0.551 rad、最大跟踪差约 0.103 rad；右约 0.860 rad / 0.101 rad；右 J3 接触门限峰值约 7.7 Nm，正常保持/释放 |
| 联动后持续运行 | 核心栈累计 1,807 秒（30 分 07 秒） | 服务均持续 running；FR3/teleop 错误 0 | Operator 最终两臂均 disengaged，feed error 为空 |

整个核心负载运行期：

- 全局 `UdpRcvbufErrors` 和 `UdpInErrors` 均保持 `508→508`；
- 左右 FCI socket drop 保持 `258→258`、`20→20`；
- receive queue 瞬时最大 2,304 B，并在后续采样清空；
- 左右 FCI NIC RX/TX error/drop 均为 0；
- 最后一次 socket 采样发生在控制进程拆除后，采集脚本记录了无 socket 的 `0/0`，不属于运行期 drop 清零；统计时只使用进程和 socket 仍存在的 3,080 行。

MoveIt 日志有一条 `No 3D sensor plugin(s) defined for octomap updates`。它说明本次 `moveit-ik` 配置没有给 OctoMap 接入 3D sensor plugin；这不是 FR3 通信错误，也不能作为 Orbbec 成败的判断依据。Orbbec 是否通过必须以相机进程和真实图像流单独确认。

原始证据位于 `docs/diagnostics/franka_full_load_20260817/`。Operator 的完整高频日志保存在 `/home/user/franka_teleop_data/diagnostics/20260817_142516.4lI1mT/ee_jitter.jsonl`；仓库中没有保留重复的 196 MB 副本。

### 4.8 相机尝试与当前物理阻塞

启动 `orbbec` 后，相机进程报告：

```text
VendorTCPClient: Connect to server failed addr=192.168.1.10 port=8090
```

随后相机进程结束，没有任何图像流。现场网络检查结果：

- `enp2s0f2` 为 `192.168.1.100`，能看到 `192.168.1.110`，这是左侧 Wuji 手链路；
- `enp11s0` 为 `192.168.2.100`，能看到 `192.168.2.111`，这是右侧 Wuji 手链路；
- 空闲第四个 I350 口 `enp2s0f3` 为 DOWN/no carrier；
- 扫描没有发现 `192.168.1.10`，TCP 8090 不可达。

因此本轮没有相机数据负载。最可能的下一步是给相机通电并把其网线接到 `enp2s0f3`；确认 carrier 和实际拓扑后，再决定是否给该口配置专用 `/32` 地址/路由，以避免和左手的 `192.168.1.0/24` 发生路由冲突。物理拓扑未确认前没有贸然修改主机路由。

### 4.9 软件回归与测试限制

- reset/launch 相关 pytest：`25 passed`；
- 完整负载验收时，实际 Conda 运行环境的扩展适配器测试为 `20 passed, 1 failed`；唯一失败来自配置断言仍期望旧 GELLO sensitivity。随后按操作员要求把右 GELLO J7 sensitivity 从 `0.9` 调到 `1.2`，同步断言为当前左侧 `[1,1,1,1,1,1,1]`、右侧 `[0.6,0.6,0.6,0.6,0.6,0.6,1.2]`，GELLO 映射测试复跑为 `14 passed`；
- 右 GELLO J7 纯输入测试记录到 `461.87°` 连续跨度、约 `689 Hz` 采样且无 stale/jump，确认原先约 90° 的 FR3 行程来自全局 `max_relative_delta=1.5 rad`，不是编码器或解包故障。按操作员要求，配置已改为分侧、逐关节上限：左侧仍为每轴 `1.5 rad`，右侧使用当前系统 FR3 绝对上下限的完整跨度；绝对关节限位、`0.7 rad/s` 限速、接触门限和首次目标检查均保留。新增运行时传递测试后 GELLO 测试为 `15 passed`；
- 旧右 GELLO 的故障最终跟随设备本体，换用 OpenRB-150 GELLO 后，左右固定身份分别更新为 `4303A73A5157375037202020FF100616` 和 `17E84ADC5157375037202020FF10131E`。双侧 ID 1–7 的 ping 均为 `comm=0 / model=1200 / dxl_error=0`，5 秒实时流约为左 `802 Hz`、右 `795 Hz`，新鲜度均小于 `1 ms`；
- 曾实现过“每端预留物理行程 2.5%”的逐关节软限位方案；操作员随后要求在最终配置中取消，因此 `joint_limit_margin` 配置、映射逻辑和对应测试已完整移除。最终范围恢复为旧 GELLO 的最后确认版本：左侧每轴 `1.5 rad`，右侧为七轴各自完整 FR3 物理跨度；右侧倍率仍为 `[0.6,0.6,0.6,0.6,0.6,0.6,1.2]`，`0.7 rad/s` 限速和安全网关硬限位保留；GELLO 与遥操语义测试恢复为 `39 passed`；
- 新 GELLO 首次真机逐轴方向检查发现左 J4/J6、右 J2/J4/J6 与 FR3 期望方向相反；在双侧 Disengaged 且 `q_cmd == q_meas` 后优雅停止整栈，逐位翻转后最终 `direction_correction` 为左 `[1,1,1,1,1,1,1]`、右 `[1,-1,1,1,1,1,1]`。倍率、相对行程及安全网关参数未改变；
- 新 GELLO 方向确认后，右侧 J1/J2/J3 sensitivity 按操作员要求由 `0.6` 调为 `1.0`，最终右侧向量为 `[1.0,1.0,1.0,0.6,0.6,0.6,1.2]`。提出的 `0.9 rad/s` 限速没有应用：GELLO 映射层和安全网关继续保持 `0.7 rad/s`，避免放宽既有 slew 安全保护；
- 随后按操作员最终要求重新加入更小的防限位余量：左右两臂每个关节在上下限端各预留物理总行程的 `1%`，保留中间 `98%`，不改变左侧 `1.5 rad` 或右侧完整 FR3 跨度的 `max_relative_delta`。若 Engage 时实测关节已在软限位外，映射保持当前姿态、阻止继续向外，并始终允许朝安全区回撤；负余量、无可用范围、上下限裁剪和回撤语义均有测试覆盖，GELLO 与遥操语义测试为 `42 passed`。配置后的硬件流复检尚未执行：两块 OpenRB 于 `22:51:54` 同时从 USB 断开，当前无 `/dev/ttyACM*` 枚举；
- 操作员随后要求单独反转右 GELLO J2 映射，右侧 `direction_correction` 最终更新为 `[1,1,1,1,1,1,1]`；其余方向、倍率、1% 防限位和限速均未改变。硬件仍未枚举，因此本次只完成配置与软件回归，待重新连接后真机确认；
- 两块 OpenRB 于 `23:15` 重新枚举后完成补测：双端口身份预检通过，5 秒实时流左约 `804 Hz`、右约 `807 Hz`，两侧均为七关节实时数据且无通信错误；右 J2 的实际 FR3 方向仍需下一次低幅 Engage 真机确认；
- 操作员继续调节后的当前右侧 sensitivity 为 `[1.3,2.2,1.3,1.0,1.0,1.0,1.6]`，其中 J2 从 `2.0` 提高到 `2.2`，配置校验上限仅同步提高到 `2.2`。工作区同时已有 GELLO 映射层 `0.8 rad/s` 的操作员改动，予以保留；安全网关仍限制最终输出为 `0.7 rad/s`；
- `bash -n` 与 `docker compose config` 通过；
- 系统 Python 缺少 PySide6，无法在该解释器中跑 GUI pytest；实际 preflight 确认 Conda GUI 环境 PySide6 可用，本轮真实 GUI 也正常运行。Conda base 有 PySide6 但没有 pytest，因此这项自动化覆盖仍未补齐；
- 主机未安装 `shellcheck`，所以只完成了 shell 语法检查，没有声称完成 shellcheck 静态检查。

## 5. 文档步骤逐项状态

| 项目 | 状态 | 结论 |
|---|---|---|
| Cyclone DDS | 已完成 | 建议保留；解决 DDS 隔离/participant 问题，但不是 FCI 相位根因 |
| DDS 仅本机发现 | 已完成 | 有必要，防止 ROS 流量进入 FCI 专网 |
| 大小核、controller 与 IRQ 分离 | 已完成并验证 | 当前布局合理 |
| realtime 组、limits、PREEMPT_RT | 已完成并验证 | CM 实际 FIFO 98 |
| `overruns.manage: false` | 原配置已正确 | 保留 |
| 网卡 EEE/coalescing/offload 调优 | 已完成 | 保留 |
| 实时路径普通 ROS 日志 | 已移除 | 保留 |
| 1000 Hz 时序根因 | 已逐包定位 | 1001 Hz 对照与长测通过 |
| Franka 优雅退出 | 已修复并真机重复验证 | 通过 |
| Operator/ROS launch 整栈退出 | 已修复并真机复测 | 5 个核心服务全部退出码 0 |
| 关闭 SMT | 未做 | 当前不必要 |
| 30–60 分钟静止测试 | 已完成 30 分钟 | 通过 |
| GELLO + Operator + MoveIt IK 核心负载 | 已完成 30 分 07 秒，含左右单臂和双臂动作 | 通过 |
| Orbbec 相机流 | 未完成 | 相机 IP/物理链路不可达 |
| 含相机的最终完整并发负载 | 未完成 | 相机恢复后补测 |

## 6. SMT（超线程）可行性与必要性

当前 i9-12900K 布局把左/右 controller 各放在一整颗 P 核的两个 sibling 上，把两张 FCI IRQ 放在另外两颗独占 P 核上。housekeeping 不进入这些物理核。

不建议现在关闭 SMT：

1. CPU 交叉 A/B 后故障不跟随 CPU；逐包结果直接指向独立时钟相位漂移。
2. 1001 Hz 下 10 分钟逐包、30 分钟静止和 30 分钟核心负载均通过，线程调度没有故障前尖峰。
3. `nosmt=force` 会下线当前使用的 9、11、13、15 等 sibling 编号，现有 controller、IRQ、GRUB isolation、Compose cpuset 和调优服务必须一起重映射。
4. 关闭 SMT 需要修改 GRUB 并重启，影响和回滚成本明显高于当前收益。

技术上仍可做严格 A/B，但优先级低。触发条件应是：恢复相机后的完整负载再次失败，而且 trace 显示 controller/IRQ 所在物理核存在可重复的调度或缓存争用；仅凭一次通信错误不应关闭 SMT。

## 7. 未做事项的可行性和必要性

| 未做事项 | 可行性 | 当前必要性 | 判断 |
|---|---|---|---|
| 恢复 Orbbec 物理链路并确认 `192.168.1.10:8090` | 可行，需现场检查供电、线缆和接入拓扑 | 高 | 没有图像流就不能完成最终验收；先接空闲第四口，不动 FR3 网线 |
| 相机 + GELLO + MoveIt IK + 双 FR3 最终并发负载 | 可行，需相机先可达且现场监护 | 高 | 核心负载已通过；补 15–30 分钟相机流，并重复低幅单臂/双臂动作和优雅退出 |
| 长时间连续人工运动 | 可行，但增加操作和碰撞风险 | 低至中 | 当前单臂/双臂动作已覆盖命令链；没有必要为了“满 30 分钟”持续移动，按实际任务场景验收更有价值 |
| 延长到 60 分钟纯静止 | 可行 | 低至中 | 已有 30 分钟纯静止加 30 分钟核心负载；若含相机负载失败或生产规范明确要求再补 |
| 关闭 SMT | 可行但需重启和全量重映射 | 低 | 当前没有指向 SMT 的证据 |
| 增大 UDP rmem/wmem | 可行 | 低 | 精确 FCI socket 稳态 drop 为 0；全局 UDP 只在 ROS 启动阶段增长 |
| 升级 libfranka/franka_ros2/机器人系统镜像 | 可行但涉及兼容矩阵 | 条件性 | 当前版本已稳定；除非供应商版本要求或恢复相机后仍失败，不在现场临时升级 |
| 把 1001 Hz 改成上游正式的机器人节拍主循环 | 技术上可行 | 中长期 | 当前 workaround 已验证；后续可向 ros2_control/franka_ros2 上游反馈并减少本地维护 |
| Fast DDS 与 Cyclone 多轮 A/B | 可行 | 低 | 生产已统一 Cyclone；继续 A/B 对当前根因帮助有限 |
| 长时间调度 trace | 可行但有额外负载/数据量 | 条件性 | 仅在错误复现时触发最有价值 |

## 8. 完整负载执行情况与剩余计划

已按原计划完成：

1. 只读 preflight：GELLO 序列号、端口、电机 1–7、CPU/IRQ 和 Compose 通过。
2. 启动 `franka-control + teleop-control + moveit-ik + preset-ik + gello-bridge`，双臂 disengaged 观察超过 5 分钟，通过。
3. 急停可触达、工作区清空后，依次完成左臂、右臂和双臂 engage 动作，通过。
4. 核心栈持续到 30 分 07 秒，FCI socket、NIC、控制日志均未出现通信退化。
5. 最终两臂均 disengaged。第一次停止暴露 teleop/GELLO/MoveIt 容器退出码 137 和 Operator 信号继承问题；完成 3.6 的修复后重新真机启动/停止，所有核心服务退出码 0，两侧 hardware clean shutdown。

剩余最终验收必须按以下顺序完成：

1. 相机通电，把网线接到空闲第四口 `enp2s0f3`；如果实际使用交换机或其他拓扑，先记录拓扑，不能猜路由。
2. 确认网口 carrier、相机实际 IP 和 TCP 8090；必要时只给第四口增加精确主机路由，不能影响左右 FR3/手链路。
3. 启动 Orbbec 并确认真实图像连续更新，而不只是容器处于 running。
4. 在相机流下启动已通过的核心栈，先 disengaged 观察 5–10 分钟，再依次做左、右、双臂低幅低速动作。
5. 保持含相机的完整负载至少 15–30 分钟，重复检查 FCI socket drop、NIC drop、FR3 错误、相机帧流和 Operator feed error。
6. 执行 `DISENGAGE ALL` 后停止整栈；两侧 hardware clean shutdown、所有服务无需 SIGKILL 才算最终通过。

## 9. 安全与回滚

- 安全网关仍是 FR3 command bus 的唯一发布者。
- 没有放宽 freshness、acquisition、关节限制、首目标距离、碰撞阈值或 slew 检查。
- GELLO 与 MoveIt 真机控制器不得同时运行；启动脚本已有冲突检查。
- 任何 stale sample、GUI 丢失、串口错误或 robot state 丢失都应 disengage 对应路径。

若需要回退控制时序，只需把 `update_rate` 改回 1000 并恢复标准 `controller_manager/ros2_control_node`；但这会恢复已逐包确认的相位漂移和关机竞争，不建议用于生产。Cyclone 回退必须让同一 ROS domain 的全部服务一起改回 Fast DDS，不能混用实现并期待互通。

## 10. 参考

- [Franka：PREEMPT_RT 与实时权限](https://frankarobotics.github.io/docs/doc/libfranka/docs/real_time_kernel.html)
- [Franka：通信约束错误排查](https://frankarobotics.github.io/docs/troubleshooting.html)
- [libfranka `communication_test`](https://frankarobotics.github.io/libfranka/0.15.3/communication_test_8cpp-example.html)
- [franka_ros2 issue #108：ROS 2 下间歇通信约束错误](https://github.com/frankarobotics/franka_ros2/issues/108)
- [Cyclone DDS ParticipantIndex 配置](https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/docs/manual/options.md)
- [Linux `nosmt` 内核参数](https://www.kernel.org/doc/html/latest/admin-guide/kernel-parameters.html)
