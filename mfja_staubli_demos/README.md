# Stage 1 — Staubli Cartesian Planning

This introductory exercise plans straight Cartesian `tool0` motions for the
Room 315 Staubli TX2-60L with HPP, then executes them in Gazebo.

Every run builds the HPP problem afresh. HPP constrains the tool path and
continuously checks the complete arm and gripper against the shared Room 315
cell model, including the glass.

## Requirements

Install and build this repository with the
[top-level instructions](../README.md). The recommended planning environment is
the `hpp-exec` Docker container:

```bash
git clone -b devel https://github.com/humanoid-path-planner/hpp-exec.git \
  "$HOME/devel/hpp-exec"
"$HOME/devel/hpp-exec/run.sh" bash -lc \
  'cd ~/devel/src && make hpp-python.install'
```

The wrappers find this common location automatically. Set
`HPP_EXEC_DIR=/another/path/hpp-exec` when it is installed elsewhere.

## Room registration

The Room 315 Staubli world pose is `x=-15.251`, `y=-6`, `z=1`, `yaw=0`
(metres and radians, with zero roll and pitch). The Gazebo spawn configuration
uses that pose, and HPP applies its inverse to express the fixed cell in the
robot base frame.

## Exercise

First check that HPP can construct and solve the default line:

```bash
ros2 run mfja_staubli_demos room315_hpp_line.sh --plan-only
```

Expected output includes:

```text
max straight-line deviation: 0.0000xx m
```

Use two sourced terminals for execution. Start Gazebo in terminal 1:

```bash
ros2 run mfja_staubli_demos room315_demo.sh
```

In terminal 2, move from the simulation spawn pose to the exercise start, then
run the vertical line:

```bash
ros2 run mfja_staubli_demos room315_hpp_line.sh --goto-start
ros2 run mfja_staubli_demos room315_hpp_line.sh
```

Return along the opposite line:

```bash
ros2 run mfja_staubli_demos room315_hpp_line.sh --line 0 0 -0.4
```

Try another displacement in the Staubli base frame, in metres:

```bash
ros2 run mfja_staubli_demos room315_hpp_line.sh \
  --line 0 0.2 0 --duration 8
```

A line starts at the current tool pose. Return first if a new line is not
reachable from the current configuration.

## Main options

| Option | Default | Meaning |
|---|---|---|
| `--line DX DY DZ` | `0 0 0.4` | Tool displacement in the Staubli base frame, metres. |
| `--duration` | `5.0` | Execution duration, seconds. |
| `--samples` | `80` | HPP path samples sent to Gazebo. |
| `--q-start J1 ... J6` | `0 50 70 0 55 0` degrees, converted to radians | Planning start for `--plan-only`; target for `--goto-start`. |
| `--goto-start` | off | Move to the exercise start instead of planning a line. |
| `--plan-only` | off | Plan and validate without ROS output. |
| `--robot-name` | `staubli1` | Gazebo robot namespace. |

## Implementation map

- `hpp/room315_hpp_line.py` builds the HPP problem, projects the Cartesian
  target, installs line and orientation constraints, and samples the checked
  path.
- `hpp/staubli_tx2_60l.srdf` contains the arm self-collision exclusions.
- `mfja_3rd_floor_description/urdf/staubli_tx2_60l.urdf` is the shared robot
  model.
- `mfja_3rd_floor_description/urdf/room315_cell.urdf` is the shared fixed-cell
  collision model.
- `launch/room_315_staubli_cartesian_demo.launch.py` starts Room 315 and the
  Staubli simulation.
- `scripts/room315_hpp_line.sh` mounts the installed package shares in
  `hpp-exec`, so both normal and symlinked colcon installs work.

For live runs, the planner reads `/staubli1/joint_states` and publishes one
`trajectory_msgs/msg/JointTrajectory` on `/staubli1/joint_trajectory`. The
first point is held briefly so the Gazebo controller settles.

`--goto-start` is a simulation setup helper. If Gazebo starts in a configuration
that the HPP model reports in collision, it performs a slow unchecked retreat
before reaching the validated exercise pose. Do not use that recovery behavior
on hardware.

## Local HPP alternative

Docker is optional if the local environment provides `pyhpp`, `hpp_exec`,
Pinocchio, and ROS 2. Add the repository root to `ROS_PACKAGE_PATH`, source the
MFJA workspace, and run `hpp/room315_hpp_line.py` directly.

Use a distinct `ROS_DOMAIN_ID` for each training workstation. The wrappers
default to domain `7`.
