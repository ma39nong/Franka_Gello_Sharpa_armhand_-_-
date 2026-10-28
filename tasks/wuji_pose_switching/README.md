# Wuji Hand 2 two-stage gesture policy

This optional task policy uses two distance mappings saved by `apps/wuji_ui`:

- left distance trigger `11` selects pose `神勺准备`;
- left distance trigger `55` selects pose `9999`.

It is not enabled by default. Start the normal Wuji teleoperation stack and
explicitly pass the policy:

```bash
./ops/run/start_wuji_teleop.sh \
  --wuji-left-address 192.168.1.110:7447 \
  --wuji-right-address 192.168.2.111:7447 \
  --wuji-hand-strategy-config tasks/wuji_pose_switching/config/policy.json
```

Without `--wuji-hand-strategy-config`, both hands retain the original live
MANUS retargeting behavior.

## State transitions

The configured left hand starts in `FREE`, where it follows live MANUS
retargeting. Holding trigger `11` inside its entry distance and dwell moves to
`READY` and commands `神勺准备`. While in `READY`, holding trigger `55` inside
its tighter entry distance and dwell moves to `ACTIVE` and commands `9999`.

Opening past trigger `55`'s exit distance returns `ACTIVE` to `READY`.
Opening past trigger `11`'s exit distance returns either fixed-pose state to
`FREE`. Operator disengage, an open request, or stale MANUS data resets the
policy to `FREE`.

Because both mappings use the same thumb-to-index distance, the saved
thresholds must nest: `55` enter/exit must stay inside `11` enter/exit.
Startup rejects overlapping same-finger stages.

All transitions use `max_joint_speed_rad_s` from `config/policy.json`, enforce
the Wuji model limits, and remain gated by the existing Operator hand engage
and MANUS freshness checks.

## Calibration contract

`config/policy.json` references
`config/calibration/wuji_hand_2_poses.json`; it does not copy pose angles or
gesture thresholds. Relearning `11` or `55` in the UI therefore updates the
next teleoperation run. Startup fails if either trigger, its pose, the selected
side, or the 20-joint device contract does not match.

For the first physical run, lower `--wuji-current-limit`, keep an emergency
stop accessible, engage only the left hand, and verify `FREE → READY → FREE`
before testing `ACTIVE`.
