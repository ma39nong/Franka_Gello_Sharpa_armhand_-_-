# Dual FR3 MoveIt mode

The `lychee_fr3_description` and `lychee_fr3_moveit_config` ROS 2 packages are
copied from `lychee_barmate` and built with the rest of this workspace.  They
are an optional operating mode: MoveIt trajectory controllers and the normal
Gello safety gateway must never own the arms at the same time.

Rebuild the image/workspace after the first checkout:

```bash
./ops/setup/build.sh
```

Validate planning with fake hardware:

```bash
./ops/run/run_moveit.sh --fake
```

The launcher also starts the FR3 teach-and-repeat web UI on
`http://localhost:8081`. Named points and routines are persisted under the
configured `TELEOP_DATA_ROOT` at `arm_ui/`. Fake mode can validate editing and
trajectory playback, but zero-effort hand guiding is available only with the
real effort-interface controllers.

After the usual workcell checks, use both physical FR3 arms:

```bash
./ops/run/run_moveit.sh --real
```

In real mode the UI can strictly switch either arm between its
`*_teach_controller` and `*_arm_controller`. The former continuously commands
zero effort so the robot's configured gravity compensation permits hand
guiding. Correct payload and center-of-mass configuration, physical support of
the arm during the switch, and reachable hardware stopping remain mandatory.

The launcher refuses to run while any Gello/PICO/VIVE arm-control service is
active.  Real mode runs `ops/run/preflight.sh --arms-only --skip-gello` (Docker,
ROS workspace, Compose, and FR3 CPU layout only; no GELLO or MANUS hardware)
and uses the workcell's FR3 addresses (`172.16.0.3`, `172.16.0.2`).

Continuous relative end-effector action recording, low-speed replay, current
limitations, and the planned GELLO/foot-pedal integration are documented in
[RELATIVE_ACTION_TEACHING.md](RELATIVE_ACTION_TEACHING.md).
