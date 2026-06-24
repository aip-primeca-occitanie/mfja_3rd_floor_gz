# mfja_staubli_manipulation_demos

Room 315 Staubli manipulation demos for moving a small payload between the
right-rail shuttles and the Staubli table.

The package contains one shared HPP problem and small target-specific execution
outputs. Keep planning geometry, contact surfaces, and manipulation graph common
between Gazebo and the real robot; switch only the launch/output layer.

## Layout

- `launch/room_315_staubli_shuttle_manipulation_demo.launch.py`: Gazebo Room 315
  scene with `staubli1` and the right shuttle rail.
- `hpp/room315_shuttle_manipulation.py`: HPP planning, phase sampling, and
  execution output adapters.
- `hpp/*.urdf`, `hpp/*.srdf`: HPP robot/object/environment models and semantic
  surfaces.
- `models/staubli_tx2_60l_gripper`, `urdf/staubli_tx2_60l_gripper.urdf`: Gazebo
  and ROS description for the Staubli with the passive open gripper geometry.
- `scripts/room315_demo.sh`: launch the Gazebo scene.
- `scripts/room315_hpp_manipulation.sh`: run the HPP problem in `hpp-exec`.
- `scripts/room315_moving_shuttle_demo.sh`: orchestrate the two-shuttle sequence.

## Gazebo

Launch the scene:

```bash
mfja_staubli_manipulation_demos/scripts/room315_demo.sh
```

For the moving-shuttle setup, start the pickup shuttle away from the robot:

```bash
mfja_staubli_manipulation_demos/scripts/room315_demo.sh gui:=false right_start_slot:=1
```

Run the default two-shuttle sequence:

```bash
HPP_EXEC_DIR=/home/psardin/devel/nix-hpp/src/hpp-exec \
  mfja_staubli_manipulation_demos/scripts/room315_moving_shuttle_demo.sh --replace-box
```

The moving-shuttle helper prepares the rail route, starts the Staubli
preposition trajectory, then immediately starts the pickup shuttle. The HPP
execution waits until both the shuttle has arrived and the Staubli is at the HPP
start.

Gazebo execution uses:

- arm output: `/staubli1/joint_trajectory`
- gripper output: `/staubli1/gripper_joint_trajectory` in the moving-shuttle
  helper; direct HPP execution can enable it with
  `--gripper-output joint-trajectory`
- payload output: Gazebo spawn/set-pose/remove services

The fixed gripper geometry must leave room around the payload during the
approach. Closing belongs to the semantic grasp action; do not hide a bad
approach by disabling payload/gripper collisions.

The Gazebo grasp is intentionally kinematic. The fingers open and close for
visual and timing feedback, while the payload is attached by following the HPP
object pose during the grasp-transfer phase instead of relying on unstable
contact physics. The visible Gazebo payload is visual-only; HPP remains the
collision source of truth for the box, gripper, shuttle, and table.

## HPP Checks

Build the HPP scene:

```bash
mfja_staubli_manipulation_demos/scripts/room315_hpp_manipulation.sh --build-only
```

Preview the main plans:

```bash
mfja_staubli_manipulation_demos/scripts/room315_hpp_manipulation.sh --plan-only \
  --direction shuttle-to-table

mfja_staubli_manipulation_demos/scripts/room315_hpp_manipulation.sh --plan-only \
  --direction shuttle-to-shuttle \
  --shuttle-pose -15.310 -5.536 0.839346 0 0 -0.002 \
  --destination-shuttle-pose -14.770 -5.536 0.839346 0 0 -0.0014
```

Execute one HPP cycle in the running Gazebo scene:

```bash
mfja_staubli_manipulation_demos/scripts/room315_hpp_manipulation.sh --execute \
  --replace-box \
  --gripper-output joint-trajectory
```

## Real Robot

Use the same HPP problem and phase sampling, but disable Gazebo-only outputs and
select the real gripper output:

```bash
mfja_staubli_manipulation_demos/scripts/room315_hpp_manipulation.sh --execute \
  --payload-output none \
  --gripper-output joint-trajectory \
  --gripper-trajectory-topic /staubli1/gripper_joint_trajectory \
  --gripper-joints gripper_finger_joint
```

For a digital open/close gripper:

```bash
mfja_staubli_manipulation_demos/scripts/room315_hpp_manipulation.sh --execute \
  --payload-output none \
  --gripper-output bool \
  --gripper-command-topic /staubli1/gripper/command
```

Before sending motion to hardware, run `--plan-only`, check the real current
joint seed with `--q-start` when needed, and confirm the measured gripper TCP,
finger geometry, opening width, speeds, and clearances.

## Current Surfaces

- gripper: `staubli/tool0_gripper`
- payload handle: `box/top_handle`
- payload support contact: `box/bottom_surface`
- payload size: `0.07 x 0.05 x 0.06 m`
- supports: `shuttle/top_surface`, `drop_shuttle/top_surface`,
  `staubli_table/drop_zone`
