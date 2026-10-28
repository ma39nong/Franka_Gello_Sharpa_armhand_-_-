# MANUS hand input

MANUS supplies calibrated hand skeletons to the unified operator. Each dynamic side is retargeted to its configured robot model and sent as named joint radians to `linker_hand_bridge`; arm and hand activation are coupled per side.

## Requirements

- Only one MANUS CoreSDK client may run at a time.
- Each glove needs `adapters/manus/config/Calibration_left.mcal` or `Calibration_right.mcal`.
- The standard models are the full L20-URDF retargeter for the left G20 and the right O30i solver.

The bridge explicitly enables MANUS automatic user assignment on connection.
Core Integrated otherwise inherits `userSettings.autoAssignment` from the
machine's saved settings; when it is disabled, gloves can be detected and
calibrated but produce no raw skeleton frames because no user is assigned.
This setting affects MANUS input only; robot output still requires engagement.

Build the native bridge:

```bash
adapters/manus/scripts/build.sh
```

Inspect connected gloves with teleop stopped:

```bash
conda run --no-capture-output -n gello-upper-body-teleop python adapters/manus/scripts/inspect_manus_gloves.py
```

## Standard operator

Use the repository wrapper:

```bash
ops/run/run_teleop.sh
```

It starts the unified PICO motion-tracker and bimanual MANUS backend. Use the PySide6 operator GUI to engage or disengage each side, home arms, and open hands as independent per-side actions.

## Hands-only

Start the hand drivers:

```bash
cd docker
docker compose up hand-control
```

Then start MANUS:

```bash
cd /home/descfly/llx/gello_upper_body_teleop
conda run --no-capture-output -n gello-upper-body-teleop python adapters/manus/scripts/teleop_manus_hands.py --sides both
```

Controls are `L`/`R`, `Space`, `X`, `O`, and `Q`.

The left G20 uses the vendor L20 URDF and full per-frame thumb retargeting.
The left thumb follows the same optimization policy as the O30i: every actuated
thumb coordinate—CMC yaw/roll/pitch and the coupled MCP/IP flex actuator—is
solved from landmark positions, segment directions, and activated fingertip
distance objectives. `left_full_retarget_physical.jsonl` supplies the 18 mm
MANUS contact deadzone; the pinch objective uses the O30i's validated 10x
distance weighting. There is no thumb-index pose anchor. The solve retains its
warm start, activation-release smoothing, 0.35 rad/tick thumb trust region,
output EMA, and joint limits. Four-finger curl endpoints still use the labelled
MANUS ranges.

## O30i behavior

The right O30i retargeter uses recorded MANUS open/curl endpoints, removes the
glove's residual contact gap for thumb-index pinch, and smoothly blends both
the thumb and middle finger into a hardware-validated middle-pinch anchor. The
ordinary mid-range poses remain solver-driven.

The driver maps URDF lower and upper limits to normalized ticks 0 and 255 unless
`O30_TICKS_AT_LOWER` and `O30_TICKS_AT_UPPER` provide measured endpoints. It
verifies model and handedness, requires fresh calibrated feedback before
commanding, holds across normal command gaps, and disables on feedback loss or
command failure.

## Accuracy recording

The endpoint protocol records six explicit expectations: a fully open human
hand maps to an open robot, four fully curled fingers map to their closed range,
thumb-index and thumb-middle pinches reach contact, a fully curled thumb reaches
its closed range, and the index and middle fingertips contact directly with the
thumb clear. Separate zero-gap G20 and O30i
candidate anchors are fitted from `manus_six_pose_bimanual_20260730_215752` and
activate only for that labelled gesture. Offline replay reaches a 0.0 mm median
FK gap without activating in the other five phases; physical contact validation
is still required before calling those anchors final. Stop the normal operator
first, then record without hardware:

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python adapters/manus/scripts/record_manus_accuracy.py \
  --output /home/descfly/franka_teleop_data/manus_accuracy/run1.jsonl \
  --sides both

conda run --no-capture-output -n gello-upper-body-teleop \
  python adapters/manus/scripts/analyze_manus_accuracy.py \
  /home/descfly/franka_teleop_data/manus_accuracy/run1.jsonl

# Evaluate new solver code against the exact same recorded landmarks:
conda run --no-capture-output -n gello-upper-body-teleop \
  python adapters/manus/scripts/analyze_manus_accuracy.py \
  --replay-current /home/descfly/franka_teleop_data/manus_accuracy/run1.jsonl
```

The recorder sends only to a private local UDP sink. Its v2 rows include the
complete 25-keypoint MANUS frame and orientations, source sequence/timestamp,
canonical landmarks, raw and filtered named radians, robot targets/FK,
calibration hashes, solver configuration, and the would-be UDP identity.

For an end-to-end hardware trial, use the hands-only operator with
`--debug-log FILE` and simultaneously record these ROS topics:

```text
/linker_hand_bridge/left/mapped_command
/linker_hand_bridge/right/mapped_command
/cb_left_hand_control_cmd
/cb_right_hand_control_cmd
/cb_left_hand_state
/cb_right_hand_state
```

The normal teleop-data recorder already captures the four `cb_*` command/state
topics. Bridge command headers carry `stream_id:sequence`, matching the host
JSONL transport identity, so solver output, slew-limited command, and feedback
can be aligned later.

## Retargeting policy

Do not add hidden operator-specific gains. Derive calibration from repeatable,
labelled multi-pose recordings. Analyze recordings offline before changing
objectives or limits.

The skeleton pipeline above is the single supported MANUS path.
