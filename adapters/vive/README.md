# HTC VIVE Tracker arm input

This adapter uses two hand-mounted VIVE Trackers as an `ArmPoseSource` for the
existing dual-FR3 mapping, IK, UDP, ROS, and safety pipeline. It does not publish
robot commands itself and does not use the pelvis Tracker.

Configured devices:

```text
left   LHR-306ED0EE
right  LHR-4A29398A
```

## Prerequisites

SteamVR must already be running, both configured Trackers must be green, and
OpenVR must be configured for `TrackingUniverseStanding`. The Python client uses
`VRApplication_Background`; it does not start SteamVR.

Install/update the managed environment:

```bash
./ops/setup/setup_teleop_env.sh
```

## Read-only connectivity check

This command initializes OpenVR but does not start ROS or connect to a robot:

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python adapters/vive/scripts/inspect_vive_trackers.py \
  --config config/modes/vive.yaml
```

Continuously watch pose and connectivity:

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python adapters/vive/scripts/inspect_vive_trackers.py \
  --config config/modes/vive.yaml --watch
```

A successful snapshot ends with `configured Trackers: READY`. The adapter also
records a per-side reason such as `device disconnected`, `OpenVR pose invalid`,
or a non-`Running_OK` tracking result; these are OpenVR acquisition failures,
not local freeze-threshold decisions.

## XY calibration

If height is correct but horizontal directions are not, stop teleoperation and
run the read-only guided calibration. Prefer a Tracker that remains continuously
valid:

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python adapters/vive/scripts/calibrate_vive_xy.py \
  --config config/modes/vive.yaml --side right
```

The default is a dry run. Review the fitted matrix, repeat the captures if they
are inconsistent, and only then add `--write`. The tool averages stationary
OpenVR samples, requires at least 8 cm movement in intended robot +X (forward)
and +Y (left), preserves OpenVR +Y as robot +Z, and never starts ROS or sends a
robot command.

## Teleoperation

VIVE is the default ROS bridge and host operator mode:

```bash
cd docker
docker compose up franka-control teleop-control vive-bridge hand-control
```

Then start the host operator (VIVE arms and MANUS hands):

```bash
ops/run/run_teleop.sh
```

`run_vive_teleop.sh` remains as an explicit alias. PICO is optional through
`pico-bridge` and `ops/run/run_pico_teleop.sh`.

Start the ordinary operator GUI separately. Tracking begins only after explicit
GUI engagement. A missing disengaged Tracker denies only that side; loss or an
input motion fault while engaged disengages both sides, matching the existing
PICO motion-tracker policy.

## Coordinates

`config/modes/vive.yaml` contains the workcell-calibrated axis mapping: OpenVR `+X`
to control `+X`, OpenVR `-Z` to control `+Y`, and OpenVR `+Y` to control `+Z`.
The existing relative mapper anchors each Tracker to the measured end-effector pose
at engagement, so the SteamVR world origin is not an absolute robot target.
