# Room 315 Staubli pick-and-place

This package plans one table pick-and-place for the Staubli TX2-60L. The box
starts on the left side of the Staubli table and is placed 20 cm to the right.
HPP checks the robot, gripper, box, table, and Room 315 fixture collision meshes.

## Setup

Build the workspace as described in the [top-level README](../README.md), then:

```bash
source "$MFJA_WORK_DIR/setup.bash"
ros2 run mfja_staubli_manipulation_demos room315_check_setup.sh
```

Planning computes and validates the trajectory locally by default:

```bash
ros2 run mfja_staubli_manipulation_demos room315_pick_place.sh
```

Use `--build-only` to construct and inspect the HPP scene.

## Viser

Plan and open the HPP scene in a browser:

```bash
ros2 run mfja_staubli_manipulation_demos room315_pick_place.sh --viser
```

The page is served at `http://localhost:8000`. Select a path in the Viser path
player and press Ctrl-C in the terminal when finished.

## Gazebo

```bash
# Terminal 1
source "$MFJA_WORK_DIR/setup.bash"
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch mfja_staubli_manipulation_demos \
  room_315_staubli_pick_place_sim.launch.py

# Terminal 2, after Gazebo is ready
source "$MFJA_WORK_DIR/setup.bash"
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 run mfja_staubli_manipulation_demos \
  room315_pick_place.sh --execute
```

The launch starts one Staubli, its gripper, and the payload box.

## Hardware

Use an authorized robot terminal for this section. If the cell uses a
non-default `ROS_DOMAIN_ID`, export the same commissioned value in both
terminals:

```bash
# Terminal 1
source "$MFJA_WORK_DIR/setup.bash"
read -r -p "Staubli controller IP: " ROBOT_IP
ros2 launch mfja_staubli_manipulation_demos \
  room_315_staubli_hardware.launch.py robot_ip:="$ROBOT_IP"

# Terminal 2: read the current joint positions, then plan only
source "$MFJA_WORK_DIR/setup.bash"
ros2 run mfja_staubli_demos room315_read_configuration.py

# Copy the six values from the reported "positions" array.
ros2 run mfja_staubli_manipulation_demos \
  room315_pick_place.sh --execution-profile hardware \
  --q-start Q1 Q2 Q3 Q4 Q5 Q6
```

After the workcell and path have been approved, add `--execute` to the second
command. `Q1 ... Q6` must be a fresh measured configuration in radians. The
hardware launch provides the direct driver, state feedback, and robot model;
`--execute` submits the reviewed trajectory.

## Options

```text
room315_pick_place.sh
  [--execution-profile simulation|hardware]
  [--build-only | --viser | --execute]
  [--q-start Q1 Q2 Q3 Q4 Q5 Q6]
```

## Physical commissioning

Offline planning validates the current Room 315 collision model. Physical
commissioning adds measured tool and grasp transforms, the real workpiece
model, payload data, IO checks, low-speed limits, and operator approval.

See [the code map](docs/room315_pick_place_walkthrough.md) for the implementation
layout.
