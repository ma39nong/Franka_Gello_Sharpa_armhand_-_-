# Pose Recorder

只读网页工具：抓取当前双 FR3 与 Wuji Hand 2 关节位置，追加写入 JSONL，并支持备注。

**不会** 向机械臂或灵巧手下发任何命令；Wuji 仅 `connect` + 订阅 `joint_states`，不 `enable`。

## 启动

先保证 `franka-control`（或 fake）已在跑，才能读到臂状态。Wuji 地址需显式给出；不要与正在占用同一只手的 Operator/Wuji teleop 同时连接。

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python -m apps.pose_recorder \
  --sides both \
  --wuji-left-address 192.168.1.111:50001 \
  --wuji-right-address 192.168.1.112:50001 \
  --output /home/user/lpy/gello-retarget/apps/pose_recorder/records/poses.jsonl
```

浏览器打开 <http://127.0.0.1:8091>。

常用参数：

| 参数 | 含义 |
| --- | --- |
| `--sides left\|right\|both` | 记录哪些侧 |
| `--output PATH` | JSONL 存放路径 |
| `--wuji-<side>-address IP:PORT` | 该侧 Wuji Hand 2 |
| `--no-arms` / `--no-hands` | 只记手或只记臂 |
| `--host` / `--port` | 网页监听地址 |

## JSONL 字段

每行一个对象，大致包含：

- `timestamp` / `iso_time` / `note` / `sides`
- `arms.sides.<side>.names|positions`
- `hands.sides.<side>.names|positions|joints`
