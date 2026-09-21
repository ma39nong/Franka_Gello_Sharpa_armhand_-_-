# Operations

- `run/`：标准遥操、仅双臂、Wuji、手动 UI、MoveIt 等运行入口。Harvest
  采集入口 `start_recording.sh` / `convert_recording.sh` 是独立只读进程，
  不替代 `start_wuji_teleop.sh`。
- `setup/`：Conda、本地包、Docker/ROS workspace 和 GELLO 驱动安装。
- `diagnostics/`：只读检查或显式诊断工具；可能移动硬件的工具会在自身文档中
  标明。

所有脚本都从自身位置解析仓库根目录，可以从任意当前工作目录调用。

迁移整套 GELLO + Sharpa + 触觉采集工作站时，直接复制完整工程文件夹，并在新电脑
执行根目录的 `./一键安装环境.sh`。完整步骤、许可证边界、端口和首次硬件检查见
[`工程迁移与一键安装说明.md`](../工程迁移与一键安装说明.md)。
