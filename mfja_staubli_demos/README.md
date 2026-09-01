# Stage 1 — Staubli Cartesian Planning

This introductory exercise plans straight Cartesian `tool0` motions for the
Room 315 Staubli TX2-60L with HPP, then executes them in Gazebo.

Every run builds the HPP problem afresh. HPP constrains the tool path and
continuously checks the complete arm and gripper against the shared Room 315
cell model, including the glass.

## Requirements

Use the standard host underlay/overlay from the
[top-level instructions](../README.md):

```bash
source "$MFJA_WORK_DIR/setup.bash"
"$MFJA_WORK_DIR/mfja_3rd_floor_gz/mfja_staubli_demos/scripts/room315_check_integration.sh"
```

The one MFJA setup chains ROS Jazzy and the installed HPP Python 3.12
underlay.

## Room registration

The Room 315 Staubli world pose is `x=-15.03`, `y=-6`, `z=1`, `yaw=0`
(metres and radians, with zero roll and pitch). The Gazebo spawn configuration
uses that pose, and HPP applies its inverse to express the fixed cell in the
robot base frame.

The default scene, planning, and trajectory values live in
`config/room315_cartesian_line.yaml`.

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

A line starts at the current tool pose. Return to the previous pose first when
the next line requires that configuration for reachability.

## Real Staubli feedback and trajectory export

These commands connect to physical hardware. The direct driver bringup provides
the robot model, state feedback, trajectory action, IO, and system services.
Use an authorized robot terminal for this section:

```bash
source "$MFJA_WORK_DIR/setup.bash"
read -r -p "Staubli controller IP: " ROBOT_IP
ros2 launch mfja_staubli_manipulation_demos \
  room_315_staubli_hardware.launch.py \
  robot_ip:="$ROBOT_IP"
```

Use another terminal to inspect state feedback:

```bash
source "$MFJA_WORK_DIR/setup.bash"
ros2 topic echo /joint_states
ros2 run mfja_staubli_demos room315_read_configuration.py \
  --topic /joint_states
```

Once the reported configuration matches the physical robot, export a line from
that live state to a JSON trajectory:

```bash
ros2 run mfja_staubli_demos room315_export_staubli_line.py \
  --joint-states-topic /joint_states \
  --line 0 0 0.10 \
  --duration 5 \
  --samples 80 \
  > /tmp/room315_joint_trajectory.json
```

Inspect the saved trajectory, its first point, the current cell and tool
geometry, and the robot mode before an authorized operator publishes it on
`/joint_path_command`. Build and integration checks are read-only.

## Main options

| Option | Default | Meaning |
|---|---|---|
| `--line DX DY DZ` | `0 0 0.4` | Tool displacement in the Staubli base frame, metres. |
| `--duration` | `5.0` | Execution duration, seconds. |
| `--samples` | `80` | HPP path samples sent to Gazebo. |
| `--q-start J1 ... J6` | `0 50 70 0 55 0` degrees, converted to radians | Planning start for `--plan-only`; target for `--goto-start`. |
| `--goto-start` | off | Move to the exercise start. |
| `--plan-only` | off | Plan and validate locally. |
| `--joint-states-topic TOPIC` | unset | Read a named-joint start configuration for trajectory export. |
| `--print-joint-trajectory` | off | Print the Staubli `JointTrajectory` payload for review. |
| `--robot-name` | `staubli1` | Gazebo robot namespace. |

## Implementation map

- `hpp/room315_cartesian_line.py` builds the HPP problem, projects the
  Cartesian target, installs line and orientation constraints, and samples the
  checked path.
- `hpp/room315_cartesian_config.py` loads the YAML configuration.
- `hpp/room315_hpp_line.py` provides the Gazebo and real-feedback command-line
  entry points.
- `hpp/staubli_trajectory_export.py` renders the driver's
  `JointTrajectory` payload.
- `mfja_3rd_floor_description/srdf/staubli_tx2_60l.srdf` contains the shared
  gripper and arm self-collision semantics.
- `mfja_3rd_floor_description/urdf/staubli_tx2_60l.urdf` is the shared robot
  model.
- `mfja_3rd_floor_description/urdf/room315_cell.urdf` is the shared fixed-cell
  collision model.
- `launch/room_315_staubli_cartesian_demo.launch.py` starts Room 315 and the
  Staubli simulation.
- `scripts/room315_hpp_line.sh` runs the installed planner in the active
  HPP/ROS environment.

For live runs, the planner reads `/staubli1/joint_states` and publishes one
`trajectory_msgs/msg/JointTrajectory` on `/staubli1/joint_trajectory`. The
first point is held briefly so the Gazebo controller settles.

`--goto-start` is a simulation setup helper. If Gazebo starts in a configuration
that the HPP model reports in collision, it performs a slow unchecked retreat
before reaching the validated exercise pose. Hardware startup uses the measured
current configuration and a commissioned path.

## Alternative HPP installations

The MFJA overlay can use any setup file that exposes compatible `pyhpp`,
`pyhpp_toppra`, `hpp_exec`, Pinocchio, and `rclpy` modules to one Python
interpreter:

```bash
HPP_SETUP=/path/to/hpp/environment.sh \
  ./mfja_staubli_demos/scripts/room315_build_overlay.sh
```

Use a distinct `ROS_DOMAIN_ID` for each simulation workstation. For hardware,
use the commissioned cell domain in every terminal.
