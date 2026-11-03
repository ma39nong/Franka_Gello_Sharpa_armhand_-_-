# Sharpa 状态回调与独立开关修改（2026-11-02）

仅实施用户选定的两项：移除状态发布中的自动故障码查询，拆分 UDP 缓冲、触觉分批与已有诊断开关。不实施附件的分层计时、耗时分布、统计汇总或时间对照检测。

基线：当前分离 HEAD `8122a7569ec463a41bba2bd47381545e601c48a4`，与附件优化提交一致。保留已有相机/GUI 未提交修改及用户删除文件。

## 修改

- `adapters/litchibot/validation_driver.py`：移除 `_publish_state()` 的 `inspect_faults()` 自动调用；保留父类状态发布、`state_tick()` 与 `observe('state_publish', ...)`。三个功能分别取配置，诊断关闭时不创建 `RuntimeDiagnostics` 及其 worker。
- `adapters/litchibot/runtime_optimization.py`：自动轮询无其他生产调用者，移除 `inspect_faults()`、`next_fault_check`、`faults`。保留已有有界异步日志；没有新增 SDK 查询或将 SDK 调用移到后台。新增 `optimization_settings()`；旧 `optimization_enabled()` 兼容接口保留。
- `config/sharpa_runtime.json`：三个显式布尔字段均为 true，保留原有正常 MANUS 行为，故障码查询始终不再自动执行。
- 对应测试：`adapters/litchibot/test_runtime_optimization.py`、`adapters/manus/test_normal_hand_runtime.py`。

## 配置与兼容性

```json
{
  "enabled": true,
  "diagnostics_enabled": true,
  "tactile_batching_enabled": true,
  "udp_buffer_tuning_enabled": true
}
```

| 设置 | 行为 |
|---|---|
| 新字段为布尔 true/false | 独立决定对应功能，不受旧 enabled 默认值约束 |
| 新字段缺失 | 回退旧 enabled；环境变量为 1 时回退 true |
| `SHARPA_RUNTIME_OPTIMIZATION=0` | 强制关闭三项，显式 true 也不能覆盖 |
| `SHARPA_RUNTIME_OPTIMIZATION=1` | 未显式设置的新字段开启；显式 false 保留 |
| 配置文件缺失、损坏或非对象 | 使用关闭默认值；环境变量 1 仍可启用旧默认行为 |
| 显式字段类型错误 | 对应功能关闭，不把字符串/数字当布尔值 |
| 其他环境变量值 | 延续旧实现的关闭行为 |

只影响原先启用这些优化的 MANUS normal/fake 路径。fake 模式仍不调整真实 UDP socket。触觉数据发布所需的 worker 不属于可关闭的诊断 worker。

单独关闭诊断可将 `diagnostics_enabled` 改为 false。关闭三项可设置 `SHARPA_RUNTIME_OPTIMIZATION=0` 后重新启动手部运行程序；这不会恢复自动故障码轮询。需要恢复旧配置格式时可移除三个新字段，只保留 enabled。开关在节点初始化读取，不热更新。

没有改变 UDP 缓冲值/端口、批大小、每指轮询频率、关节状态频率、SDK 线程所有权、控制权限/限位/停手、MANUS 连接检查或采集阈值。没有重启服务、连接设备或部署到其他工作树。

## 验证

无设备、无网络容器 `franka-upper-body-teleop:latest` 中，通过只读挂载主工程的现有 `litchi_hardware` 作为测试依赖，物理 SDK 加载被 Mock 禁止。

- `adapters/manus/test_normal_hand_runtime.py` 与 `adapters/litchibot/test_full_tactile.py`：52 项通过。包括全部 8 种开关组合、无故障码调用、父类发布和原计时仍执行；随后加强左右手实际 publish 检查并重跑 8 个组合，全部通过。
- `adapters/litchibot/` 下 `test_validation_safety.py`、`test_normal_driver.py`、`test_normal_forwarding.py`、`test_normal_shutdown.py`、`test_normal_telemetry.py`、`test_fault_severity.py`、`test_native_reenable.py`、`test_runtime_optimization.py`：99 项先通过；6 项因依赖 launch 路径未挂载失败，补齐只读测试依赖挂载后这 6 项全部通过。共 105 项通过。
- 总计 157 个相关用例通过。Python 编译检查及 `git diff --check` 通过。

运行方法：容器使用 `--network none`，工程挂载到 `/workspace/franka_upper_body_teleop:ro`，现有 `portable_deps/litchi_hardware` 只读挂载到工程对应目录；`PYTHONPATH` 包含工程、该依赖的 `src`、`ros2/src/sharpa_hand/sharpa_driver`。设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，执行 `python3 -m pytest -p no:cacheprovider -q` 并传入上述测试路径。失败补跑使用 `test_validation_safety.py test_native_reenable.py -k launch`。
