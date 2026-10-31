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

## GUI 启动时清理运行日志

正常启动 Operator GUI 时，在后台自动清理最后修改时间距今 **30 天及以上**的已识别运行日志；不会阻塞界面或遥操定时器。事件日志会报告删除文件数、释放空间、跳过及失败数。统一入口与原双终端入口共用此行为。

清理范围：

- `TELEOP_DIAGNOSTICS_ROOT`，或 `TELEOP_DATA_ROOT/diagnostics` 下由启动脚本创建的 `YYYYMMDD_HHMMSS.随机串` 运行目录中的 `.log`、`.jsonl`、`.writer_status.json`。
- 宿主机 `ROS_LOG_DIR`（默认 `$ROS_HOME/log` 或 `~/.ros/log`）中能通过节点名或 `launch.log` 内容识别为本遥操/数采程序的日志；不清理其他项目的 ROS 日志。
- 本工程 `Log/OrbbecSDK.log.txt`。

目录中有近期文件、未知文件类型或被进程打开的日志时，整次运行目录会被保留。不跟随符号链接；遇到 `metadata.yaml`、`collection_state.json`、`.db3` 或 `.mcap` 的目录整棵跳过，不清理录制样本。Docker 管理的容器日志不在此清理范围内。无需改动原有启动命令。

实现：`apps/operator_gui/log_retention.py`。删除边界测试使用临时目录，不删除实际用户日志。

## 统一遥操入口的共享相机

`./ops/run/run_gello_full_teleop.sh --hand-source manus --hardware`（含 `--takeover`）会先启动或复用独立的 `teleop-cameras` Compose 服务，通过相机就绪检查后自动选中 GUI 的“当前视频流”。LitchiBot 手套使用同一逻辑。相机异常仅报告错误，不阻止原来的臂/手启动；可加 `--no-cameras` 跳过相机启动及自动预览。默认 fake/dry-run 模式不自动打开相机。

- `start_sharpa_arm_recording.sh --with-cameras` 及触觉版本会使用同一服务，采集器以 `COLLECTION_CAMERA_EXTERNAL=true` 复用图像话题，保留原有三路彩色/一路深度的就绪检查和录制参数，不再重复启动驱动。
- 无共享相机服务时，数采入口也会先启动它。因此原双终端操作命令仍可使用。容器内部脚本未设置外部相机标志时保留原来自行启动相机的行为。
- 启动由文件锁串行化；正在运行的共享服务不会被重启。相机配置不一致、旧采集容器或独立 Orbbec 驱动占用设备时拒绝再开一套驱动。旧版本正在运行的采集器应先正常结束，再切换到新入口。
- 相机服务独立于 GUI、臂、手、采集器的退出过程；`--takeover` 继续保留相机和采集器。关闭 GUI 或停止数采不会关闭共享相机。确认已结束录制后，可显式执行 `docker compose -f docker/compose.yaml stop teleop-cameras` 停止相机。
- 仅检查相机、臂手未上电时：`./ops/run/start_teleop_cameras.sh`。此命令不启动臂、手或采集器，不需要重建镜像；需使用已有部署环境且已接受 SDK 许可。

本次在臂、手关闭的条件下验证了相机启动/复用、GUI 自动三路预览和相机落盘；完整生产采集器仍要求原有臂/手及安全网关话题，未绕过此要求。结果见 `docs/diagnostics/gui_camera_preview_20261030/shared_camera_results.json`。
