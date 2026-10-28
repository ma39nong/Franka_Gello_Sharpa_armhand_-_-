# FR3 Arm UI

`arm_ui` is the operator-facing, single-arm teach-and-repeat application. It
keeps the browser separate from ROS control: the UI edits named joint
waypoints and routines, while `ArmRosRuntime` alone reads fresh robot state,
switches controllers, and submits a MoveIt Pilz sequence.

## Workflow

1. Select the left or right arm.
2. Click **进入零力矩拖动**, support the arm, and guide it by hand.
3. Click **记录当前位置**, name the captured seven-joint configuration, and
   save it. Editing degrees only changes the saved target; it never commands
   the robot.
4. Drag 1–9 named points into the task list and set velocity, acceleration, and
   blend radius.
5. Click **START**. The backend reads a fresh joint state, switches that arm to
   its trajectory controller, and sends one `MoveGroupSequence` goal. The arm
   stays under trajectory control at the final point.
6. Click **进入零力矩拖动** explicitly before guiding the arm again.

### Relative action recording

The lower **动作示教（相对末端轨迹）** panel records continuous hand-guided
motion for actions such as a wrist scooping gesture:

1. Select one arm and enter zero-effort hand-guiding mode.
2. Enter an action name, click **开始录制动作**, and guide the real arm through
   the motion. The default sample rate is 100 Hz.
3. Click **停止并保存**. Static lead-in/out is trimmed, while both the raw
   seven-joint samples and the end-effector path relative to its initial pose
   are retained.
4. Hand-guide the arm to another candidate start pose and click
   **当前位置验证 IK**. MoveIt sequentially solves the relative end-effector
   path from that pose and checks for IK branch jumps. This validation never
   sends a motion command.
5. Set **试运行速度** between 5% and 100% (15% by default), then click the
   saved action's **动作试运行** button. **设为原速 100%** only fills in the
   requested speed; execution still requires confirmation and all trajectory
   safety checks. After confirmation, the backend solves
   every frame again, rejects movement during solving, switches the selected
   arm to trajectory control, and submits one time-scaled spline trajectory.
6. **停止试运行** cancels the active controller goal and holds the current
   position. A completed preview stays at the action's final pose; explicitly
   re-enter zero-effort mode before hand-guiding again.

Recording uses `lychee_root` as the fixed URDF base and the selected arm's
`link8` as the tool frame. A saved relative frame is
`T_initial_tool^-1 * T_sample_tool`; during validation it is reapplied as
`T_current_tool * T_relative`. IK validation keeps MoveIt's collision,
reachability, and continuity checks enabled. The two arms are not coordinated
or moved by this feature.

Preview is deliberately restricted to one execution, 5%–100% speed,
at most 2,000 recorded frames, and at most 120 seconds after scaling. The first
IK point must remain within 0.05 rad of the current joints, and movement greater
than 0.03 rad while IK is being solved aborts before controller switching.
Adjacent IK jumps and generated joint-velocity limit violations also abort the
preview.

### Dual-arm absolute Ready-to-Home recording

The bottom **Operator GUI: dual-arm absolute Ready → Home trajectory** panel
records measured joint positions for both arms at once. Put both arms in teach
mode, start at the shared Ready pose, select one of `粉末称量`, `装配`, or
`夹豆`, record while guiding both arms to that task's Home, then stop and save.
The file is absolute (no IK or rebasing) and overwrites that task's previous
recording. Operator playback independently verifies its first and last frames
against the currently recorded Ready and task Home poses.

`STOP` requests cancellation through the MoveIt action. It is not a hardware
emergency stop.

## Storage

Files are written atomically under `/data/arm_ui` in the container:

- `waypoints.yaml`: named left/right joint configurations in radians.
- `routines.yaml`: named sequences referencing those point names, plus speed,
  acceleration, and blend settings.
- `actions/<side>__<name>.yaml`: raw hand-guided joint/tool samples and the
  derived tool-relative action frames.
- `absolute_trajectories/<task>.yaml`: synchronized dual-arm absolute joint
  samples used by Operator GUI's Ready-to-Home command.

The browser also exposes download buttons for both files. The `/data` volume is
provided by `TELEOP_DATA_ROOT` in `docker/.env`.

## Start

The normal MoveIt launcher starts the UI together with fake or real hardware:

```bash
./ops/run/run_moveit.sh --fake
./ops/run/run_moveit.sh --real
```

Open <http://localhost:8081>. Fake mode supports point editing and playback,
but its velocity-interface controller configuration does not provide the
zero-effort teach controller. Hand-guiding is a real-hardware feature.

On a graphical workstation `run_moveit.sh` opens the page automatically after
startup. Set `ARM_UI_NO_BROWSER=1` when running unattended or when browser
launching is not wanted.

The real configuration uses one controller manager per arm. The teach and arm
controllers claim the same seven effort interfaces and therefore can never be
active together. The runtime uses strict controller switches and refuses a
mode switch while a trajectory action is active. Because the two arms use
independent controller managers and disjoint effort interfaces, both arms may
be placed in zero-effort mode at the same time. Support each arm while switching
it and keep the workspace clear. Before starting a task or action preview, exit
zero-effort mode on the other arm; the runtime rejects execution otherwise.

Intermediate points use the configured Pilz blend radius. A non-zero radius
means continuous motion passes near the intermediate target instead of
stopping exactly on it; the final point always has zero blend and is held.
