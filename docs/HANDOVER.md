# Repository handover

Current GELLO integration state as of 2026-08-01. Use
[GELLO_TELEOP.md](GELLO_TELEOP.md) for first-stage operation and
[HARDWARE_DEPLOY.md](HARDWARE_DEPLOY.md) for the inherited workcell.

## Workcell

```text
left FR3    172.16.0.3          right FR3  172.16.0.2
host        enp6s0: 172.16.0.6/24, 192.168.1.53/24
Orbbec      192.168.1.123:8090
left hand   G20  can0 0x28      right hand O30i libcanbus USB a8fa:8598
VIVE hand tracker ids: config/modes/vive.yaml    PICO fallback ids: config/modes/pico.yaml
```

## Current status

- Standard startup is `docker compose up franka-control teleop-control moveit-ik preset-ik gello-bridge hand-control`, then `ops/run/run_teleop.sh`, then `python apps/operator_gui/operator_gui.py`. `moveit-ik` is read-only and starts no controller manager. `preset-ik` is the warm TCP IK helper for Q/W/E. VIVE and PICO remain optional fallbacks; never run multiple UDP bridges.
- Arms are settled; contact torque gating and collision thresholds are hardware-validated. Do not retune without reading the relevant git history.
- The false-stale PICO regression is resolved and hardware-verified. Motion is parsed first, published atomically, and considered fresh only when the local callback sequence advances; no native parsing exception may cross the vendor callback boundary.
- Do not restore the former multi-getter consistency loop or cached-snapshot engagement grace. An invalid atomic snapshot disengages immediately.
- Bimanual MANUS uses the optimization retargeter by default. The left G20 now solves against the verified official L20 V10.1 model. CMC yaw/roll/pitch and coupled MCP/DIP flex are jointly optimized using position, segment-direction, and activated excess-distance terms, with the recorded 18 mm MANUS contact deadzone and 10x distance weighting. There is no thumb-index pose anchor. Warm start, activation release, a 0.35 rad/tick thumb trust region, output EMA, and joint limits remain active. The right O30i is unchanged.
- The left model is `assets/linkerhand_l20_v101/linkerhand_L20_V10.1_left.urdf/linkerhand_L20v10.1_left.urdf`. FK uses its `thumb_dip`; the UDP contract deliberately retains `thumb_ip`. Bridge normalization and abduction polarity use V10.1 limits and axes. Existing MANUS landmarks were replayed into `/home/descfly/franka_teleop_data/manus_accuracy/v101_retargeted/`; the labelled six-pose model-space medians are 0.6 mm thumb-index, 3.9 mm thumb-middle, and 0.0 mm index-middle. Physical V10.1 acceptance is still pending, so the old-model physical claims must not be treated as validation of this migration.
- Arm sources, operator state, and hands are injected into the hardware coordinator. PICO SDK ownership is explicit and hand retargeting runs outside the arm loop; add future arm adapters without importing them into the coordinator.

## Invariants

- Never run two PICO clients or two MANUS clients simultaneously.
- GUI loss, tracker loss while engaged, stale robot state, gateway rejection, or an invalid tracker snapshot disengages affected control.
- `seq` is local receipt of a parsed Motion object; `ts` is only a vendor payload timestamp. Rising `callback_errors` means SDK fields or frames were rejected. `n=0` does not prove the physical trackers are inactive.
- Position and rotation liveness are checked independently. Do not weaken freshness, jump, speed, torque, slew, or acquisition checks to hide a fault.
- Only the safety gateway publishes the FR3 command bus. The host operator stays ROS-free.
- Home and replay move hardware. Preserve logs before shutting down after any reflex or unexplained fault.

## Next work

- Physically validate the migrated left V10.1 open/curl, thumb-index, thumb-middle, index-middle, and thumb-rotation behavior before marking it final. Keep hardware disengaged for the first inspection and use the normal gateway/slew protections.
- Improve hand fidelity only from recordings that include landmarks, solved radians, commands, and feedback; do not add posture-specific thumb anchors when the continuous optimizer can represent the motion.

## Verification

PICO: `conda run -n gello-upper-body-teleop pytest -q adapters/pico/tests`.

Gateway: `PYTHONPATH=ros_ws/src/teleop_core python3 -m pytest -q ros_ws/src/teleop_core/test/`.

Data lives under `/home/descfly/franka_teleop_data/`; `hand_fidelity*.jsonl` is replayable retargeting input.
