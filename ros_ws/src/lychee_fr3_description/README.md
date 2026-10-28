# Lychee FR3 Description

Dual-FR3 robot description for the lemon-cutting workspace. This package
wraps the official `franka_description` FR3 model and owns only the project
mounting geometry, prefixes, home positions, and launch entry point.

Default xacro:

```bash
xacro $(ros2 pkg prefix lychee_fr3_description)/share/lychee_fr3_description/robots/lychee_dual_fr3/lychee_dual_fr3.urdf.xacro hand:=false
```

Publish `robot_description`:

```bash
ros2 launch lychee_fr3_description description.launch.py
```

The default model has no hands attached. Linker hand geometry should be added
later as a separate description dependency instead of blocking the arm-only
URDF.
