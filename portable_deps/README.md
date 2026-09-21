# 内置迁移依赖

此目录使整个 `gello_upper_body_teleop` 文件夹可以直接复制到另一台电脑后安装。

- `litchi_hardware/`：Sharpa/MANUS 源码及供应商运行库；
- `gello_software/`：GELLO 驱动及 DynamixelSDK。

旧电脑生成的 `.pixi`、`.venv`、ROS `build/install/log` 和 Git 历史未复制。目标电脑
执行根目录的 `一键安装环境.sh` 后会在本机重新生成环境，避免保留旧绝对路径。

这里包含受供应商许可证约束的内容。只能把完整工程复制到已获授权的电脑，不要公开
发布或提交这些依赖文件。
