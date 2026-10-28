# lychee_fr3_moveit_config

MoveIt configuration for the Lychee dual-FR3 robot.

This package consumes the URDF/SRDF from `lychee_fr3_description` and keeps the
motion-planning layer separate from task behavior trees and atomic capability
packages. The robot model is based on the official `franka_description` FR3
macros; FK/IK should therefore follow the official franka_ros2/MoveIt stack
instead of reusing old hand-written IK from the reference code.

## Launch

Start a planning-only `move_group`:

```bash
ros2 launch lychee_fr3_moveit_config move_group.launch.py
```

Start MoveIt with fake ros2_control hardware and RViz:

```bash
ros2 launch lychee_fr3_moveit_config moveit.launch.py use_fake_hardware:=true
```

The default robot model uses two official `fr3` arms, with `left` and `right`
prefixes. Grippers are disabled by default because the copied URDF still needs a
separate, explicit hand-description dependency before linker-hand meshes and
joints should be enabled.
