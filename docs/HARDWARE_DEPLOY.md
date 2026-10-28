# Hardware runbook

This is the short operational path for dual GELLO, dual FR3, left G20,
right O30i, MANUS input, and the Orbbec camera. See
[GELLO_TELEOP.md](GELLO_TELEOP.md) for GELLO-specific preflight and anchoring.

## Safety

- Keep the physical emergency stop reachable. GUI disengage stops following but is not a power cut.
- Never start a second PICO or MANUS client beside a live operator.
- Do not reconfigure `enp6s0`, run Home, replay a dataset, or send diagnostic hand poses unless you intend to move hardware.
- On every bringup, verify both `accepted the collision thresholds` messages in `franka-control` and `Contact torque gating active` in `teleop-control`.
- After a reflex or unexplained fault, save `docker compose logs franka-control teleop-control` before stopping the stack.

## Standard startup

Terminal 1:

```bash
cd /home/descfly/llx/gello_upper_body_teleop/docker
docker compose up franka-control teleop-control moveit-ik preset-ik gello-bridge hand-control
```

Terminal 2:

```bash
cd /home/descfly/llx/gello_upper_body_teleop
ops/run/run_teleop.sh
```

Terminal 3:

```bash
cd /home/descfly/llx/gello_upper_body_teleop
conda activate base
python apps/operator_gui/operator_gui.py
```

The backend starts disengaged and writes a fresh diagnostics directory. GUI
loss disengages all sides. The Operator GUI independently gates each arm and
hand: `L`/`R` control left arm/left hand, `A`/`B` control right arm/right
hand, and `Space` homes both arms. `Home both arms` moves both arms and
disengages all followers for reset ownership; `Open hand`
stops only that hand's following and opens the selected hand.

The normal stack now also starts `moveit-ik` and `preset-ik`. `moveit-ik` is a
read-only `move_group` process used by Q/W/E preset prechecks; it subscribes to
measured joint states and exposes `/compute_ik`, but starts no controller
manager and cannot publish the FR3 command bus. `preset-ik` keeps that solver
warm on `127.0.0.1:5591` so each Q/W/E does not spawn a tools container.
Both stay on `HOUSEKEEPING_CPUSET`, away from `FRANKA_TELEOP_CPUSET`.
Other non-FCI containers and the native Operator use the same housekeeping
set; only `franka-control` uses the teleop realtime set.
With hands enabled the normal internal services are `franka-control`,
`teleop-control`, `moveit-ik`, `preset-ik`, `gello-bridge`, and
`hand-control`; the host Operator backend/GUI remain outside Docker.

## Stop

Disengage in the GUI, stop the operator, stop Compose, then disable FCI. Do not run another VIVE, PICO, or MANUS diagnostic until the operator has exited.

## Tracker checks

Tracker availability is per-side. A missing tracker refuses that side; loss or a motion fault while engaged disengages both sides.

Before connecting any robot process, the default VIVE read-only connectivity check is:

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python adapters/vive/scripts/inspect_vive_trackers.py \
  --config config/modes/vive.yaml --watch
```

For optional PICO motion-tracker arms, replace `vive-bridge` with
`pico-bridge`; never run both because they own the same host UDP ports:

```bash
cd /home/descfly/llx/gello_upper_body_teleop/docker
docker compose --profile pico up franka-control teleop-control pico-bridge hand-control
```

```bash
cd /home/descfly/llx/gello_upper_body_teleop
ops/run/run_pico_teleop.sh
```

With teleoperation stopped, PICO trackers can be inspected or assigned with:

```bash
conda run --no-capture-output -n gello-upper-body-teleop python adapters/pico/scripts/hardware/inspect_motion_trackers.py
conda run --no-capture-output -n gello-upper-body-teleop python adapters/pico/scripts/hardware/calibrate_tracker_sides.py --write
```

PICO diagnostics use `follow-debug.v6`: `seq` is accepted Motion callbacks,
`callback_errors` counts rejected SDK fields/frames, `n` is usable trackers in
the last parsed frame, and `age` is time since `seq` advanced.

## Alternate input and hand modes

Use PICO optical hands:

```bash
ops/run/run_teleop.sh --hand-source pico
```

Use controllers for arms:

```bash
conda run --no-capture-output -n gello-upper-body-teleop python -m teleop_runtime.cli --config config/modes/pico.yaml --arm-source controllers
```

Test MANUS hands without arms:

```bash
cd /home/descfly/llx/gello_upper_body_teleop/docker
docker compose up hand-control
```

```bash
cd /home/descfly/llx/gello_upper_body_teleop
conda run --no-capture-output -n gello-upper-body-teleop python adapters/manus/scripts/teleop_manus_hands.py --sides both
```

The hands-only controls are `L`/`R`, `Space`, `X`, `O`, and `Q`. The left G20 uses full L20-URDF retargeting. MANUS gloves require `Calibration_left.mcal` and `Calibration_right.mcal` in `adapters/manus/config`.

## Arm home and hand open

Use the GUI `Home both arms (Space)` control to reset both arms together, and
the per-side `Open hand` actions independently. There is deliberately no
combined arm-and-hand home action; request both actions explicitly when both
are wanted. The arm service equivalent is:

```bash
cd /home/descfly/llx/gello_upper_body_teleop/docker
docker compose run --rm tools ros2 service call /reset_to_initial_pose std_srvs/srv/Trigger '{}'
```

Per-side services are `/reset_to_initial_pose/left` and `/reset_to_initial_pose/right`. `/capture_initial_pose` replaces the saved home. Reset is joint interpolation, not collision planning.

The GUI's per-side `Record current as Home` action calls
`/capture_initial_pose/left` or `/capture_initial_pose/right`. It requires that
arm's follower to be stopped, reads a fresh measured joint state, and atomically
updates only that side of `config/initial_pose.yaml`; it does not move either
arm. The legacy `/capture_initial_pose` service still captures both sides.
After pulling this change, rebuild `franka_fr3_arm_controllers` and restart the
Compose stack so the per-side ROS services are registered; no Docker image
rebuild is required for this source-only change.

```bash
cd /home/descfly/llx/gello_upper_body_teleop/docker
docker compose run --rm -T tools bash -lc \
  'source /opt/ros/humble/setup.bash && source /opt/vendor_ws/install/setup.bash && cd ros_ws && colcon build --symlink-install --packages-select franka_fr3_arm_controllers --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

## Camera

The default Compose stack includes Orbbec. For camera-only diagnosis:

```bash
cd /home/descfly/llx/gello_upper_body_teleop/docker
docker compose up -d orbbec
docker compose logs -f orbbec
docker compose run --rm tools ros2 topic list | grep '^/camera/'
```

Expected images are `/camera/color/image_raw` and `/camera/depth/image_raw`. Do not run OrbbecViewer while ROS owns the camera; network recovery is in [ORBBEC_CAMERA.md](ORBBEC_CAMERA.md).

## Record

```bash
cd /home/descfly/llx/gello_upper_body_teleop/docker
docker compose run --rm tools ros2 run teleop_data operator \
  --config /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording.yaml \
  --qos /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording_qos.yaml
```

Record only after `/status` reports `arms=2/2 | record_topics=12/12 | reset=ready`. The commands are `/record`, `/stop`, and `/save`; episodes go to `/home/descfly/franka_teleop_data/episodes/episodeN`.

## Export and replay

```bash
./ops/diagnostics/export_lerobot.sh /data/episodes/episode0 \
  --output /data/lerobot/my_dataset \
  --task "describe the demonstrated task" \
  --config /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording.yaml \
  --fps 10
```

Replay moves both arms and hands:

```bash
cd /home/descfly/llx/gello_upper_body_teleop/docker
docker compose run --rm tools ros2 run teleop_data replay /data/lerobot/my_dataset \
  --episode-index 0 \
  --config /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording.yaml
```

Disengage teleop, inspect the preposition path, and keep the emergency stop reachable.

## Verification

```bash
cd /home/descfly/llx/gello_upper_body_teleop
conda run -n gello-upper-body-teleop pytest -q adapters/pico/tests
PYTHONPATH=ros_ws/src/teleop_core python3 -m pytest -q ros_ws/src/teleop_core/test/
```
