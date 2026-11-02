# 三路相机驱动修复实施与离线验收（2026-11-01）

实施位置：当前 Codex 工作树。依据用户提供的 `camera_driver_agreed_fix_plan_2026-11-01.md` 与补充截图。
未连接或检查 cam0，未运行任何真实相机 Pipeline、设备查询或机器人程序。保留原工作树中用户已删除的 `采集进度.md` 状态。

## 已完成的代码修改

| 项目 | 实现与关键差异 |
|---|---|
| P0-A 正式参数 | `camera_primary_params.yaml` 显式 `diagnostic_period: 0.0`；厂商 launch 的默认 1.0 未改，腕部相机未禁用温度诊断。只生产化既有网络相机规避设置，不声称解决全部断连。 |
| P0-B CSV | 三份生产 profile 显式关闭 CSV。保留临时开启和唯一诊断文件接口。实际原始图像订阅者决定是否期待发布；无订阅者、提前返回、异常、队列丢弃、退出都有完成原因。 |
| CSV 生命周期 | 未完成行最多 256、2 秒超时，完成队列最多 1000（写线程另有一个同样有界批次）；超限计数并在指数次数记录告警。超时、取消、容量、写入失败都有统计，CSV 增加完成原因列。先停止 SDK 生产者和消费线程，再释放 logger/缓冲。 |
| CSV 关联 | 不仅凭帧序号，也不凭 C++ 对象地址：SDK 对同一帧会创建不同包装对象。采用帧序号＋设备时间＋SDK 接收时间复合键；重复键显式完成旧行。离线测试覆盖同一 SDK 帧的不同包装对象与复用的帧序号。 |
| P0-C 颜色队列 | 普通/左/右三个消费者都只在取帧、出队和等待时持锁。解码、消息生成、图像发布和点云处理在锁外，每路仍有唯一消费线程和独立 RGB buffer/decoded flag。 |
| 队列上限 | 新参数 `color_frame_queue_capacity` 默认 4，约容纳 30 Hz 时 133 ms、20 Hz 时 200 ms 的排队容量；吸收小突发，同时限制陈旧帧与内存。满时丢最旧完整 FrameSet，不拆分彩色/深度配对。 |
| 丢弃可追踪 | 每次溢出发出小型 `QUEUE_OVERFLOW` 事件，记录计数及单调时刻；定期健康摘要也保留累计值。仅触觉配置把健康主题作为可选主题录入 bag。实时端沿原 CollectorController 报错收尾；离线 bag 检查补查事件。历史累计计数不使新 episode 失败。 |
| P0-D Watchdog | 每 500 ms 检查实际启用的 color/depth/left/right 流，区分 `SDK_STREAM_STALLED`、`PUBLISH_STALLED`，无原始图像需求时不报发布停滞。所有超时依据 steady clock。无新控制传输或设备查询。 |
| 两个时长 | `camera_uptime_sec` 从 Pipeline 成功 start 返回计时；驱动健康信息、watchdog 告警携带此值。采集端关联 `recording_elapsed_sec`，保存在告警、健康摘要和 `collection_state.json`；驱动不依赖录制状态。 |
| 预览默认关闭 | 统一入口继续按原流程启动共享相机服务，但不再设置自动播放标志；GUI 不再响应旧的 `TELEOP_CAMERA_PREVIEW_AUTO=1` 自动播放。每次打开需操作员点击“当前视频流”。 |

## 监测接口与参数

每台相机的低频 JSON 主题为 `/camN/driver_health`（`std_msgs/msg/String`，可靠、volatile）。正常摘要 2 Hz；发生队列丢弃时另发及时的丢弃事件。字段包括：

- Pipeline 开始和报告的 steady 微秒时刻、相机运行时长。
- SDK/发布计数、最后时刻及帧序号，各流独立。
- `queue_size`、`queue_high_watermark`、`queue_drop_total`、最后丢弃时刻。
- 解码、消费处理和排队耗时（最近值及最大值，微秒）。
- 超过 240 ms 的 SDK/有发布需求时的发布间隔计数；它们只是观测指标，不替代录制质量规则。
- CSV 开启时的 pending/mapping/completed 数量及取消、容量、超时、完成队列丢弃、写入失败统计。

| 参数 | 默认值 |
|---|---:|
| `color_frame_queue_capacity` | 4 |
| `frame_watchdog_enabled` | true（控制断流告警；基础健康和丢弃记录仍保留） |
| `frame_watchdog_timeout_sec` | 1.0 |
| `frame_watchdog_grace_sec` | 3.0 |
| `frame_watchdog_log_interval_sec` | 5.0 |
| `frame_watchdog_recovery_sec` | 2.0 |
| `frame_watchdog_gap_ms` | 240.0 |

Watchdog 本身只监测/上报，不 stop/start Pipeline、不重启设备、不停止 bag、不发布机器人运动或 Home。队列丢弃事件由触觉质量监控接收后执行既有报错流程。无触觉配置未改，不新增上述健康订阅或质量策略。

## 离线验证

1. 容器内以保留的 SDK 2.8.6 编译驱动及合成测试，没有替换 SDK 二进制。
2. `camera_driver_offline_test`：合成 SDK 帧，不创建 Device、Context 或 Pipeline。覆盖 CSV 关闭、不同 wrapper、序号复用、订阅者离开、成对流部分取消、取消/发布失败、超时、pending/完成队列上限、文件写入失败、幂等退出与重新创建 logger。
3. 同一测试使用生产代码中的队列模板：消费者模拟解码期间提交 10,000 帧，容量 4，丢弃 9,996 帧；生产者可在解码解除前完成，空队列和有积压队列都能唤醒退出。一次离线运行提交耗时约 1.1 ms。
4. Watchdog 单元覆盖 SDK 断流、SDK 活跃但不发布、无订阅者、启动宽限、恢复与各流独立；正常关闭预览不作为发布质量缺口。
5. Python 采集器/相机回归：230 项通过，包括源缺帧标准、旧模式、自动分类、停止并发、Home 协调与归档；新增队列事件实时报错与实际 rosbag 落盘补查。
6. GUI/共享相机入口回归：58 项通过，包括设置旧自动播放环境变量时仍不自动开启预览。
7. 一次包含三轮 CSV 超时等待的合成测试耗时约 3.35 秒，用户 CPU 0.029 秒、系统 CPU 0.019 秒，进程峰值 RSS 约 76,032 KiB（包括加载的 ROS/OpenCV/SDK）。这些不是实际相机满负载 CPU 或解码性能测量。

测试以 `--network none`、无设备挂载的容器运行；配置验证不等于 cam0 的实机启动证明。没有修改机械臂控制代码、源 header 时间戳或原先 1～2 帧预警、单次连续缺 ≥3 帧失败、1 秒断流、3 秒新段就绪规则。

## 2026-11-02 主工程部署与现场验证

- 已将本次 29 个源码、配置、测试和说明文件同步到 `/home/descfly/ZH/gello_upper_body_teleop`。同步前逐文件核对主工程与修改前基线一致；保留用户已有的 `采集进度.md` 删除，不覆盖其他改动。
- 源码备份：`/home/descfly/.codex/backups/camera_deploy_20261102_102844/source_before.tar.gz`，同目录保存同步清单和部署前 Git 状态。
- 在现用 `franka-upper-body-teleop:latest` 环境中重建 `orbbec_camera_msgs`、`orbbec_camera`、`teleop_core`、`teleop_data_collector`、`teleop_camera_bringup`。服务从主工程 bind mount 的 `ros_ws/install` 加载新产物，基础镜像和 SDK 2.8.6 未更换。构建发现标记 `COLCON_IGNORE` 已恢复。
- 已重启共享相机服务并通过连续稳定 3 秒就绪检查。未启动或重启臂手控制、未创建录制 episode。
- 参数实查：cam0 `diagnostic_period=0.0`，cam1/cam2 保留 1.0；三台 CSV 关闭、颜色队列上限 4、主机同步 true、时间域 global。
- 35 秒图像探测：cam0 RGB/depth 各 699 帧，最大源间隔约 51.0 ms，最大接收间隔约 71.0 ms；cam1 1052 帧，最大源间隔 33.537 ms；cam2 1051 帧，最大源间隔 34.163 ms。未发现源时间戳重复或倒退。
- 三路 `/camN/driver_health` 正常，所有流 `state=OK`，包含 `camera_uptime_sec`。cam0 启动后约 0.2 秒出现 17 次队列丢弃，之后包括图像探测和退出订阅后均未继续增长；这是 episode 外启动事件，不能按累计值使下一段录制失败。未人为制造录制故障。
- 停止图像探测后再次只订阅健康主题 4 秒：所有流 `image_demand=false` 且 `state=OK`，没有把关闭预览误报为发布断流。
- 主工程回归：230 项采集器/相机测试、58 项 GUI/启动测试及 C++ `camera_driver_offline` 通过。GUI 默认关闭预览的代码已同步，下一次打开统一入口 GUI 加载；本次没有自动打开 GUI 或视频。
- 原始现场证据见同目录 `camera_driver_deployment_20261102.json`。现场验证限于短时相机输出，长时间满负载录制、受控断流和真实 episode 关联仍需后续验收。
- 自动恢复状态机仍属于独立后续阶段，本次未实现或开启。

## 主要修改文件索引

- 正式参数：`ros_ws/src/teleop_camera_bringup/config/camera_{primary,secondary,cam1_global}_params.yaml`。
- 驱动：`ros_ws/src/orbbec_camera/src/ob_camera_node.cpp`、`include/orbbec_camera/ob_camera_node.h`、新增 `include/orbbec_camera/frame_stream_health.h`。
- CSV：`src/frame_timestamp_csv_logger.cpp` 与同名头文件。
- 构建/参数接口：`ros_ws/src/orbbec_camera/CMakeLists.txt`、`launch/gemini435_le.launch.py`。
- 主题定义：`ros_ws/src/teleop_core/teleop_core/contract.py`。
- 触觉采集：`data_collection/config/record_gello_sharpa_tactile.yaml`、`rosbag_qos_sharpa_tactile.yaml`；采集包的 `live_continuity.py`、`continuity.py`、`bag_validation.py`。
- 默认关闭预览：`teleop_runtime/full_launcher.py`、`apps/operator_gui/operator_gui.py`。
- 测试：新增 `orbbec_camera/test/camera_driver_offline_test.cpp`，补充相机参数、实时/落盘连续性、质量收尾、GUI 与启动回归测试。
