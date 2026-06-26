# mfja_staubli_manipulation_demos

Room 315 Staubli manipulation demos for moving a small payload between the
right-rail shuttles and the Staubli table.

The package contains one shared HPP problem and small target-specific execution
outputs. Keep planning geometry, contact surfaces, and manipulation graph common
between Gazebo and the real robot; switch only the launch/output layer.

For the full call order, ROS interfaces, Gazebo controllers, and file-by-file
explanation, see
[`docs/room315_pick_place_walkthrough.md`](docs/room315_pick_place_walkthrough.md).

The commands below are written from the repository root:

```bash
cd "$MFJA_WS/src/mfja_3rd_floor_gz"
source "$MFJA_WS/install/setup.bash"
```

The demo scripts default to `ROS_DOMAIN_ID=7` and pass the same domain to
`hpp-exec`. Set `ROS_DOMAIN_ID` before running the scripts if your classroom
setup uses another domain.

## Layout

- `launch/room_315_staubli_shuttle_manipulation_demo.launch.py`: Gazebo Room 315
  scene with `staubli1` and the right shuttle rail.
- `hpp/room315_shuttle_manipulation.py`: command-line entry point for one HPP
  manipulation cycle.
- `hpp/room315_problem.py`: HPP model constants, pose helpers, and problem/graph
  construction.
- `hpp/room315_planning.py`: pick target selection, transition planning, and
  phase sampling.
- `hpp/room315_execution.py`: ROS, Gazebo payload, and gripper execution outputs.
- `hpp/*.urdf`, `hpp/*.srdf`: HPP robot/object/environment models and semantic
  surfaces.
- `models/staubli_tx2_60l_gripper`, `urdf/staubli_tx2_60l_gripper.urdf`: Gazebo
  and ROS/HPP descriptions for the Staubli with placeholder gripper geometry.
- `scripts/room315_demo.sh`: launch the Gazebo scene.
- `scripts/room315_hpp_manipulation.sh`: run the HPP problem in `hpp-exec`.
- `scripts/room315_moving_shuttle_demo.sh`: orchestrate the two-shuttle sequence.

## Gazebo

Launch the scene:

```bash
mfja_staubli_manipulation_demos/scripts/room315_demo.sh
```

For the moving-shuttle setup, the pickup shuttle starts upstream of the robot by
default:

```bash
mfja_staubli_manipulation_demos/scripts/room315_demo.sh gui:=false
```

Run the default two-shuttle sequence:

```bash
HPP_EXEC_DIR=$HOME/devel/nix-hpp/src/hpp-exec \
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
- gripper stroke: the Gazebo finger joints use the SCHUNK PGN-plus-P 40
  2.5 mm per-jaw stroke (`0.028` open, `0.0255` closed)
- payload output: Gazebo spawn/set-pose/remove services

HPP keeps the gripper geometry fixed and conservative; Gazebo animates small
finger joints for visual timing. Closing belongs to the semantic grasp action;
do not hide a bad approach by disabling payload/gripper collisions.

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

Use the same HPP problem and phase sampling, but disable Gazebo-only outputs.
The Staubli gripper is a SCHUNK PGN-plus-P 40 pneumatic gripper. With the
current Staubli ROS 2 driver, command the valve through the VAL3 IO service:

```bash
mfja_staubli_manipulation_demos/scripts/room315_hpp_manipulation.sh --execute \
  --payload-output none \
  --gripper-output staubli-io \
  --staubli-io-pin PIN_TO_CONFIRM
```

By default this uses `/io_interface/write_single_io`, module
`staubli_msgs/msg/IOModule.VALVE_OUT`, `state=True` for close, and
`state=False` for open. Add `--staubli-io-inverted` if the valve wiring uses the
opposite polarity.

For the full moving-shuttle sequence, pass the same gripper options and forward
real-robot-only HPP options such as `--payload-output none`:

```bash
mfja_staubli_manipulation_demos/scripts/room315_moving_shuttle_demo.sh \
  --gripper-output staubli-io \
  --staubli-io-pin PIN_TO_CONFIRM \
  --payload-output none
```

Before sending motion to hardware, run `--plan-only`, check the real current
joint seed with `--q-start` when needed, and confirm the arm execution topic or
action, measured gripper TCP, finger geometry, opening width, speeds, and
clearances.

If the real arm bridge accepts `trajectory_msgs/msg/JointTrajectory` on a topic,
select it with `--trajectory-topic` and `--joint-state-topic`. If it only exposes
a `FollowJointTrajectory` action, add an action adapter before running on
hardware.

## Current Surfaces

- gripper: `staubli/tool0_gripper`
- payload handle: `box/top_handle`
- payload support contact: `box/bottom_surface`
- payload size: `0.07 x 0.05 x 0.06 m`
- supports: `shuttle/top_surface`, `drop_shuttle/top_surface`,
  `staubli_table/drop_zone`
