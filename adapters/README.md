# Adapters

本目录只放输入设备和灵巧手硬件的适配层：

- `gello`：GELLO 的稳定仓库级入口；实现复用 PICO Python 包中的共享控制代码。
- `pico`、`vive`：机械臂姿态输入及相关诊断、仿真。
- `manus`：手套采集、手部重定向及原生 bridge。
- `wuji`：可选 Wuji 手重定向和硬件后端。

适配器不负责启动整套系统；统一编排在 `teleop_runtime`，运维入口在 `ops/run`。
机械臂适配器只能通过 `teleop_core.contract` 定义的 `ArmCommand` 接入 ROS 安全
网关，不得直接发布 FR3 命令总线。
