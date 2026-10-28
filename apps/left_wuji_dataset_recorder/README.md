# Left FR3 + Wuji Hand 2 Dataset Recorder

该记录器集成在现有遥操进程内，不会创建第二个 Wuji SDK 连接，也不会发布任何
机械臂或手部命令。一次遥操进程对应一个 episode。

## 记录内容

最终 `.npz` 包含：

| 字段 | shape | 含义 |
| --- | --- | --- |
| `wall_time_ns` | `[T]` | Unix wall-clock 纳秒时间戳 |
| `monotonic_time_s` | `[T]` | 同机单调时钟，计算帧间隔时使用 |
| `arm_q` | `[T, 7]` | 左 FR3 实测关节角，单位 rad |
| `hand_q` | `[T, 20]` | 左 Wuji Hand 2 实测关节角，单位 rad |
| `hand_sample_time_s` | `[T]` | 此手反馈实际被遥操线程收到的单调时间 |
| `hand_age_s` | `[T]` | 手反馈相对当前臂帧的时间差 |
| `hand_valid` | `[T]` | 手反馈存在且不超过 0.5 s |
| `ee_position` | `[T, 3]` | 左臂末端位置，单位 m |
| `ee_quaternion_xyzw` | `[T, 4]` | 左臂末端姿态，四元数顺序 xyzw |
| `arm_active` | `[T]` | 该帧左臂是否处于遥操 engage |
| `hand_active` | `[T]` | 该帧左手是否处于遥操 engage |

文件还包含关节名称、采样率、schema、坐标系和丢帧计数。末端帧是
`left_fr3v2_link8`，相对于 `lychee_root`。末端位姿由实测 `arm_q` 和当前
双臂 URDF 做正运动学得到，不是遥操目标位姿。

运行期间同时写 `<episode>.raw.jsonl`。它每 100 帧 flush 一次，异常退出时可用于
恢复；正常退出后 `.npz` 和 raw 文件都会保留。

## 使用方法

先创建数据目录，并为每次运行使用一个新文件名：

```bash
mkdir -p /home/user/franka_teleop_data/datasets

ops/run/start_wuji_teleop.sh \
  --wuji-sides left \
  --wuji-left-address 192.168.1.111:50001 \
  --record-left-dataset \
    /home/user/franka_teleop_data/datasets/episode_$(date +%Y%m%d_%H%M%S).npz
```

照常在 Operator GUI 中 engage 左臂和左手。停止遥操进程时记录器原子生成 NPZ；
已有文件不会被覆盖。

快速检查：

```bash
python - <<'PY'
import numpy as np

path = "/home/user/franka_teleop_data/datasets/episode_YYYYmmdd_HHMMSS.npz"
with np.load(path) as data:
    for key in data.files:
        print(key, data[key].shape, data[key].dtype)
    usable = data["arm_active"] & data["hand_active"] & data["hand_valid"]
    print("usable frames:", int(usable.sum()), "/", len(usable))
PY
```

训练前至少检查：`dropped_samples == 0`、四元数范数接近 1、关节值没有 NaN，
并根据任务决定是否只保留 `arm_active & hand_active & hand_valid` 的帧。

