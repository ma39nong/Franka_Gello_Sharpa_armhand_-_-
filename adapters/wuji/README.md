# Wuji hand integration

This optional backend vendors the retargeting core and MANUS configurations
from `wuji-retargeting`.  It uses the repository's existing MANUS native bridge
and the existing Operator GUI: each side starts disengaged, follows only while
that side is engaged, stops sending on stale MANUS data, and disables the Wuji
hardware during cleanup.

The vendored source is synchronized to upstream revision `66693c2`. Wuji Hand 2
configs use the correct official `hand2_beta` URDF, MJCF, and meshes in
`models/hand2_beta`; no legacy Hand 2 model is retained.

The original O30i stack is retained unchanged and remains the default path of
`ops/run/start_teleop.sh`.  Wuji uses a separate entry point and does not start
the Linker Hand/O30i container.

The real-hand entry point uses these same `hand2_beta` configs. Retargeted qpos
is reordered from the URDF/Pinocchio order into the compiled MJCF/device order
by joint name before it is sent to the SDK. The resolved permutation is printed
at startup so the active mapping can be checked before engaging either hand.

The right MANUS profile has localized `pinch_tip_scaling.index` and
`pinch_tip_scaling.middle` corrections. They change only the corresponding
right fingertip target while its thumb pinch mode is active; the left profile,
open-hand mapping and other fingers retain the upstream mapping. During a
thumb-index pinch, the right thumb target also shifts 4 mm toward the center of
the index pad; this correction fades with the pinch activation and is inactive
during other pinch gestures.

Install the additional Python dependencies once:

```bash
./ops/setup/setup_wuji_env.sh
```

## Simulation

The simulation entry point never connects to Wuji hardware or publishes a
hardware command. Its side-specific defaults are the recorded raw MANUS data:

- right: `/home/descfly/franka_teleop_data/wuji_replays/r_pinch_2.pkl`
- left: `/home/descfly/franka_teleop_data/wuji_replays/l_pinch_2.pkl`

Pass `--replay FILE` only when intentionally testing another recording.

Open the `hand2_beta` model:

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python -m adapters.wuji.sim --side right
```

Run a repeatable check without a viewer:

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python -m adapters.wuji.sim \
  --side right --headless --frames 300
```

Use `--side left` for the left hand. To visualize a live MANUS glove instead of
the replay, stop every other MANUS client first, start MANUS Core, then run:

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python -m adapters.wuji.sim --side right --input manus
```

The live MANUS mode reads the glove and drives MuJoCo only; it still does not
connect to either O30i or Wuji hand hardware.

Record new raw right MANUS landmarks while watching that same simulation:

```bash
mkdir -p /home/descfly/franka_teleop_data/wuji_replays
conda run --no-capture-output -n gello-upper-body-teleop \
  python -m adapters.wuji.sim --side right --input manus \
  --record /home/descfly/franka_teleop_data/wuji_replays/right_pinch.pkl \
  --frames 900
```

During the roughly 30-second capture, perform open-hand, fist, and several slow
thumb-index pinches. It saves automatically after 900 valid frames; `Ctrl+C` or
closing the viewer also saves. The file uses the same raw 21-landmark format as
the bundled replay, and an existing file is never overwritten. Replay it later
without MANUS Core or a glove:

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python -m adapters.wuji.sim --side right \
  --replay /home/descfly/franka_teleop_data/wuji_replays/right_pinch.pkl
```

For two network Wuji Hand 2 devices, addresses are deliberately explicit so
left/right cannot be selected by network discovery incorrectly:

```bash
./ops/run/start_wuji_teleop.sh \
  --wuji-left-address 192.168.1.111:50001 \
  --wuji-right-address 192.168.1.112:50001
```

## Hand-only real-hardware entry

To drive MANUS to Wuji without starting Docker, GELLO, FR3, or any arm
controller, use the hand-only entry. It defaults to the right side and requires
an explicit Wuji Hand 2 address:

```bash
./ops/run/start_wuji_hand_only.sh \
  --wuji-sides right \
  --wuji-right-model wuji_hand_2 \
  --wuji-right-address 192.168.2.111:7447 \
  --wuji-kp 1.0 \
  --wuji-kd 0.1 \
  --wuji-current-limit 0.5
```

The hardware connects and enables during startup but command streaming begins
disengaged. Press `R` (or `Space`) to toggle right-hand following, `X` to stop,
`O` to request the open pose while disengaged, and `Q` or `Ctrl-C` to disable,
disconnect, and exit. Use `L` for the left side. The same `--wuji-*` device and
gain flags are accepted by both the full and hand-only entries.

To test the integrated GELLO + Operator + MANUS + Wuji path without physical
FR3 arms, add `--fake-franka` to the full entry:

```bash
./ops/run/start_wuji_teleop.sh --fake-franka \
  --wuji-sides right \
  --wuji-right-address 192.168.2.111:7447 \
  --wuji-kp 1.0 --wuji-kd 0.1 --wuji-current-limit 0.5
```

This substitutes the Compose fake-FR3 controller only. GELLO serial input,
the Operator, MANUS, and the real Wuji hand all use their production paths.

One original USB Wuji Hand is also supported:

```bash
./ops/run/start_wuji_teleop.sh \
  --wuji-sides right \
  --wuji-right-model wuji_hand \
  --wuji-right-serial SERIAL
```

## Dual Hand 2 pose UI

For manual/MANUS teaching and independently named left/right pose capture, use
the local browser UI. Both hands connect with motors disabled, and only actual
20-joint feedback is recorded:

```bash
./ops/run/start_wuji_ui.sh \
  --wuji-left-address 192.168.2.111:7447 \
  --wuji-right-address 192.168.2.112:7447
```

The browser opens at <http://127.0.0.1:8082>. See
[`apps/wuji_ui/README.md`](../../apps/wuji_ui/README.md) for the safety workflow
and JSON contract, or [`docs/WUJI_HAND_UI.md`](../../docs/WUJI_HAND_UI.md) for
the complete Chinese runbook and code map.

Wuji Hand 2 now defaults to `kp=4.0`, `kd=0.1`, and a per-joint current limit
of `1.0 A`. Start with one hand in free space and watch current and motor
temperature during hardware validation. Override them with `--wuji-kp`,
`--wuji-kd`, and `--wuji-current-limit`.
