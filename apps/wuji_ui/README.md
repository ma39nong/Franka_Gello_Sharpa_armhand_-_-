# Wuji Hand 2 Pose UI

This local NiceGUI application owns two network Wuji Hand 2 devices and the
MANUS bridge. It starts with both hands disabled, displays actual motor joint
feedback, and stores independently named left/right poses in one JSON file.

## Start

Install the Wuji environment once, then pass both explicit device addresses:

```bash
./ops/setup/setup_wuji_env.sh
./ops/run/start_wuji_ui.sh \
  --wuji-left-address 192.168.2.111:7447 \
  --wuji-right-address 192.168.2.112:7447
```

The launcher opens <http://127.0.0.1:8082>. Pass `--no-browser` to suppress
automatic browser opening. The default pose file is
`config/calibration/wuji_hand_2_poses.json`; override it with `--output PATH`.

## Operator workflow

1. Both hands start in **manual** mode with motors disabled. Move either hand
   by hand, click **读取当前位置**, enter an independent name, and save it.
   Each pending angle can be adjusted with a model-bounded slider or its linked
   numeric degree field; editing these controls never commands hardware unless
   that side is in **滑块实时控制**. Clicking a saved pose loads it as a starting
   point. **另存为新姿态** always creates a new named record (appending `_2` if
   the current name is still selected); **覆盖选中** is the only way to rewrite
   the original pose and requires confirmation.
2. To jog from the UI, confirm **滑块实时控制** for either side. Sliders and
   degree fields then become rate-limited joint targets (`--pose-speed`). Return
   to **手动 / 失能** before touching or repositioning that hand directly.
3. To teach from MANUS, confirm **启动 MANUS 遥操** for either side. Return to
   **手动 / 失能** before touching or repositioning that hand directly.
4. Clicking a saved pose only selects it and shows its 20 angles. **移动到选中姿态**
   requires a separate confirmation and moves at the configured bounded speed.
5. Closing the server disables and disconnects both hands.

## MANUS threshold capture

The **MANUS 距离阈值采集** card works while the Wuji motors are disabled. Pick
a side, one thumb-to-fingertip distance, and an existing pose. Start capture,
repeat the intended gesture through its normal variation, then stop. The UI
shows live millimetres and robust min/p05/median/p95/max statistics.

The suggested enter threshold is the observed p95 plus a small margin. The
exit threshold is larger to provide hysteresis, and both remain editable. A
saved mapping contains the side, target finger, pose name, enter/exit values,
dwell time, and calibration statistics in `manus_triggers` in the same JSON.
At least 10 distinct MANUS frames are required.

This version records and validates the mapping but deliberately does not yet
execute a saved pose automatically when the gesture occurs. Gesture-triggered
hardware execution remains the separate next step.

## Composite gesture learning

For bottle grasps and other whole-hand shapes, use **MANUS 复合手势学习**.
Select a side and saved pose, start learning, then repeat the intended gesture
3–5 times while naturally varying the aperture. At least 30 distinct frames
are required.

The model uses five finger-curl features and seven fingertip-distance features.
Distances are divided by palm width, and all features are independent of hand
translation and wrist orientation. Each learned p05–p95 interval receives a
small margin. Live validation reports how many of the 12 ranges match; default
entry requires 80%, release occurs below 60%, and dwell defaults to 0.20 s.
These percentages remain editable before saving.

Saved composite mappings live under `manus_gesture_triggers` in the same JSON.
They are calibration/validation data only: this version still does not execute
the mapped hardware pose automatically.

## Pose sequence preview

Use **姿态串联预览** to inspect several saved poses as one motion. Select one
side, add at least two poses, and reorder or remove steps with the row buttons.
Configure the per-step hold time and measured-position tolerance, then confirm
execution.

Each segment uses the global `--pose-speed` limit. The runtime waits for both
the command interpolation and fresh motor feedback to reach the target before
starting the hold and next segment. A timeout aborts the remaining sequence.
`STOP` cancels remaining steps and holds the current command; use manual mode
or the top-level dual-disable button if the motors should be disabled.

Sequences are intentionally temporary previews and are not stored in JSON yet.
Left and right sequences are edited and executed independently.

## Single-joint hold diagnostics

The **单关节保持诊断** card reads the Hand 2 diagnostic stream and defaults to
the ring-finger ABD joint (device index 13, node 17). It shows the last command,
actual position, position error, motor current, current-limit flag, controller
temperature, bus voltage, communication response rate, drive state, and error
code.

The five-second test leaves MANUS teleoperation and holds the freshly measured
whole-hand pose. Gently perturb only the selected joint and record whether the
physical link and displayed feedback angle move together. If the physical link
moves while feedback stays nearly fixed, downstream mechanical play or a loose
transmission is likely. If feedback departs from the fixed target, the result
uses current limiting and drive errors to distinguish insufficient control
authority from a drive fault. The test never changes gains or current limits.

The same card provides runtime MIT `kp`/`kd` tuning. It reads the current
per-joint values and can apply a new pair either to the selected joint or all 20
joints on one hand. Applying gains exits MANUS teleoperation and holds the
freshly measured pose. The browser safety range is `kp=0..10`, `kd=0..1`; every
write requires confirmation. Changes survive disable/enable within this UI
process but are intentionally not written to JSON or persistent device storage.
Restarting the UI restores the command-line `--wuji-kp`/`--wuji-kd` values.

Only fresh, complete 20-joint motor feedback can be recorded. JSON stores
radians in firmware/device command order and includes all joint names so later
gesture-triggered policies can reorder by name rather than assuming indices.
