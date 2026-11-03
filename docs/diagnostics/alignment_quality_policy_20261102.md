# 录制 B 策略：30 Hz 转换兼容质量判定（2026-11-02）

## 范围和基准

按用户提供的 B 方案修改采集质量，不修改运动控制安全、SDK、相机、DDS 或转换器生产代码。当前采集基线是分离 HEAD `8122a7569ec463a41bba2bd47381545e601c48a4`，保留此前未提交的相机/手部修改。

只读 oracle：`/home/descfly/文档/ChatGPT/数据管线/convert_data_job-worker-ui`，仓库 `ma39nong/convert_data_job-worker`，分支 `auto-convert-transfer-ui`，commit `f5df024a40088bfdd73b44e9e155f6b5968af296`。`configs/sharpa.yaml` SHA256：`0f06df4dcd38ea6cd0d4144d7579f6424afe2cd9fe13a96295d21c2d712ee221`。真实 `align()` / `prepare()` / `output_segments()` 作为判定基准，未用自制转换器代替 oracle。

## 政策差异

| 项目 | legacy | conversion_aligned（触觉版默认） |
|---|---|---|
| 相邻 source gap | 周期估算缺 ≥3 帧可直接失败 | 只作诊断；不按累计或单次原始 gap 直接失败 |
| 数据时间质量 | 各 topic 原始 gap / 部分均频、覆盖规则 | 共享 30 Hz 整数 ns 网格，因果选择最近真实样本，年龄 <=150 ms |
| 三路 RGB | 分别检测原始 gap | 三路联合失效 1–2 个网格可有条件保持至 <=250 ms；不能叠加放行 |
| 深度与 20 路触觉 | 旧 gap/均频基线 | 原始录制严格 <=150 ms；不通用放宽到250/500 ms |
| 臂动作 | engaged 时旧约121.67 ms覆盖规则 | 分侧 actual status，active command 年龄<=150 ms，且 command_time+20ms>=真实切换时刻；HOLD 不要求新动作 |
| 平均频率、bag 接收边缘 | 可独立拒绝 | 报告和告警；时间有效性由共享网格判定 |
| 队列溢出 | 单事件直接质量失败 | 告警，由实际数据网格/落盘结果判断是否不可用 |
| timestamp 缺失、重复、倒退，结构/类型损坏 | 拒绝 | 保留拒绝，不改 source stamp |
| 应更新流超过1秒接收中断 | 自动停录 | 保留独立接收中断拒绝，不等同于已证明的源缺帧 |

新模式不是全面放宽：例如旧触觉 300 ms raw gap 可能未失败，B 会识别其中实际 >150 ms 的网格并拒绝，避免最终转换失败。

新算法只缓存元数据，每路最多4096条，RGB待定最多2个网格；不保留图像像素副本，不新增解码/序列化。当前网格确认使用各流已收到且经过250 ms观察的真实 source 水位；不拿 monotonic now 外推 source time。active action 也要求其自己的水位，单侧数据不能覆盖另一侧。每次在线推进最多128格，防止异常源时钟跳跃导致单次无界工作。

只有存在前后有效点的内部无效区间触发质量停止，首端/尾端无效点不直接作为内部失败。原有 `trim_start_sec=1.0` / `trim_end_sec=0.2` 未改；网格锚点与 sidecar 的实际选定 source 窗口一致。HOLD 结束段不错误地把最后一条 action 当作整段窗口终点。

原始采集没有 cleaner seam，因此深度不走例外；纯判定器在显式已知 seam 的离线对照中验证了与真实 recipe 相同的有界深度/RGB交叠规则，不能从普通缺帧自创 seam。

## 接入与最终状态

- `alignment_quality.py`：独立、有界、只读 ROS 元数据的网格判定与结构校验。
- `continuity.py`：保留原计数，将 B 的旧 gap/队列事件降为告警；只有网格实际失败或保留的 timestamp/content/silence 等拒绝可锁存原因。
- `live_continuity.py`：使用配置选择模式，沿已有独立 executor；没有新增 SDK/控制线程。`CollectorController.check_continuity()` 继续复用原 `request_stop`、幂等收尾和报错分类路径。
- `bag_validation.py`：实际落盘同算法补查；B 不再让旧 action 覆盖、均频、原始 gap、接收边缘规则重新造成独立时间拒绝。类型、结构、source时间戳与必需真实样本保留检查；核对实际消息/逐topic计数与metadata一致，并只读执行SQLite quick_check。
- `rosbag_recording_node.py`：保存模式、阈值、oracle版本、网格/各模态 age 和报告；任何真实 failures 都不允许写成 finalized。实时故障不会被后验成功覆盖。
- `revalidate_bag.py`：使用该 episode 记录的政策，不能将已有 incomplete/有实际失败的记录静默提升为合格；未改写任何真实历史 episode。
- `teleop_core.contract`：补齐录制 Sharpa 名称常量，状态带侧前缀、command不带侧前缀，匹配真实转换合同；不改变发送消息。

## 离线验收

- 255 项网格/真实转换器差异用例通过：左右手 state/action 的122/132/149/150/151/170/250 ms、多种30 Hz相位、左右臂HOLD/重启/单侧缺action、20路触觉140/151/300/500 ms、三路RGB联合1/2/3格、深度/触觉整数ns闭区间、已知seam正反例、迟到/突发/跨topic乱序（含400 ms延迟）、裁剪边缘、缓存上限、结构拒绝和实际控制器停录请求。
- 234 项原采集器/质量/并发/重验/相机回归通过，legacy 保持原政策，除按用户要求收紧的“不得自动提升旧失败记录”重验行为。
- 独立无网络 ROS 启动自检通过：真实触觉 YAML 加载 `conversion_aligned`，32路 required，35个订阅，无机器人连接或录制。
- 11个实际合成CDR/SQLite完整episode，使用真实bag后校验与原 recorder sidecar 写入代码；6个B接受的episode全部通过原 `check_ready → trim_bag → convert → verify(compare_source/full_video)` 流程。oracle 从固定git SHA导出到隔离 `/tmp`，没有改动原转换器源码/配置/outputs。

| 场景 | old_stop/拒绝 | B_stop/拒绝 | raw prepare/full-all | 实际完整转换 | 后验 |
|---|---|---|---|---|---|
| 正常三相机/臂手/触觉 | 否 | 否 | PASS | completed | finalized，无failures |
| 手状态132 ms | 是 | 否 | PASS | completed | finalized，无failures |
| 手状态170 ms，相位19 ms | 是 | 否 | PASS | completed | finalized，无failures |
| 双臂action170 ms，相位19 ms | 是 | 否 | PASS | completed | finalized，无failures |
| 手状态7.5 Hz，每4个30Hz样本保留1个 | 是 | 否 | PASS | completed | finalized，无failures |
| 单路RGB10 Hz，因果保持合法 | 是 | 否 | PASS | completed | finalized，无failures |
| 手状态250 ms | 是 | 是 | FAIL | 不具转换入口资格 | incomplete |
| 触觉300 ms | 否 | 是 | FAIL | 不具转换入口资格 | incomplete |
| 普通深度300 ms | 是 | 是 | FAIL | 不具转换入口资格 | incomplete |
| RGB联合3格 | 是 | 是 | FAIL | 不具转换入口资格 | incomplete |
| 坏段处于未来清洗将删的静止区 | 是 | 是 | FAIL | 仅test_only资格副本清洗后completed | 原失败保留 |

本表 stop 表示政策的停录/拒绝结论；CDR用例未实际连接机器人录制。合成测试减少旧误拒5例，不是现场误报率估计。所有fixture标为test_only，不提交生产数据集。未来清洗反例只在隔离副本临时适配资格，原始测试失败sidecar哈希不变；真实历史sidecar从未改写。

指定 episode590/596/599/600/602/582 的原始 Sharpa bag 在本次可访问路径中不可取得（仅发现部分同名旧清洗/元数据记录，不能当作所指原始录制）。没有将缺少原始db3的记录声称为已回放通过。受测证据与每例第一原因见 `alignment_quality_oracle_20261102.json`；完整隔离产物位于 `/tmp/sharpa-b-oracle-20261102-run6`。

## 测试命令与复现

```bash
CONVERSION_ORACLE_ROOT='/home/descfly/文档/ChatGPT/数据管线/convert_data_job-worker-ui' \
PYTHONPATH=ros_ws/src/teleop_data_collector:ros_ws/src/teleop_core \
'/home/descfly/文档/ChatGPT/数据管线/convert_data_job-worker-ui/.venv/bin/python' \
-m pytest -q -p no:cacheprovider \
ros_ws/src/teleop_data_collector/test/test_alignment_quality.py

'/home/descfly/文档/ChatGPT/数据管线/convert_data_job-worker-ui/.venv/bin/python' \
ops/diagnostics/validate_alignment_policy.py \
--oracle-root '/home/descfly/文档/ChatGPT/数据管线/convert_data_job-worker-ui' \
--work-dir /tmp/NEW_EMPTY_WORK_DIRECTORY
```

ROS回归使用 `docker run --rm --network none`、工程只读挂载到 `/workspace/franka_upper_body_teleop`，`PYTHONPATH` 指向本工程 collector/core 源码，先构建 `teleop_interfaces` 作为离线消息依赖，再 `python3 -m pytest -p no:cacheprovider -q` 运行 collector/test 与 camera_bringup/test，排除已单独运行的 `test_alignment_quality.py`。Python编译与 `git diff --check` 通过。

## 上线与回退

离线 Go 只覆盖上述受测输入与固定 oracle，不声称对任意未来设备延迟/清洗/时钟变化保证成功。未来清洗反例说明在线不能预知静止段删除，仍存在保守误停；本次不在录制时运行转换器。

触觉配置已选择 B，普通无触觉配置不变。同步到主工程后重新启动采集终端3即可加载；GUI和臂手/相机不需因此重启，不自动启动真实录制。

一键政策回退：将 `data_collection/config/record_gello_sharpa_tactile.yaml` 的 `continuity_mode: conversion_aligned` 改成 `continuity_mode: legacy`，再重启采集终端。`continuity_enabled` 保持true，不触及控制安全。既有失败episode保持失败，不因切换政策变合格。

部署结果：已同步18个相关文件到主工程 `/home/descfly/ZH/gello_upper_body_teleop`，同步前逐文件核对原基线。备份：`/home/descfly/.codex/backups/recording_alignment_B_20261102_165547`。实际运行镜像通过主工程源码链接成功导入新模块；无网络隔离环境中使用主工程已安装包加载真实触觉YAML并创建32路required/35个订阅，启动自检通过。部署时无采集器运行，未启动录制、未重启相机或臂手。下次启动触觉采集终端即使用B。
