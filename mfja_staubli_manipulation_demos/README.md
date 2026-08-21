# Stage 2 — Room 315 Staubli Manipulation

This package teaches HPP manipulation planning with a Staubli TX2-60L, a
pneumatic parallel gripper, a payload, and two Room 315 shuttles.

The supported classroom exercise is one two-shuttle transfer. Use the
[training runbook](docs/room315_training_runbook.md) during a session and the
[architecture walkthrough](docs/room315_pick_place_walkthrough.md) when
explaining or maintaining the implementation.

## Entry points

Each command owns one layer:

| Command | Responsibility |
|---|---|
| `room315_demo.sh` | Start the Room 315 scene. It creates a stopped shuttle by default but never commands the rail. |
| `room315_manipulation_demo.sh` | Prepare the payload, then run one fixed shuttle-to-table manipulation. It does not import or use the rail API. |
| `room315_moving_shuttle_demo.sh` | Add rail routing, shuttle motion, measured support poses, and safe `OFF` cleanup around the same manipulation runner. |
| `room315_hpp_manipulation.sh` | Build, plan, and optionally execute one HPP cycle for poses supplied on the command line. |

## Prepare a training PC

Install and build the MFJA workspace with the
[repository instructions](../README.md). Install `hpp-exec` once:

```bash
git clone -b devel https://github.com/humanoid-path-planner/hpp-exec.git \
  "$HOME/devel/hpp-exec"
"$HOME/devel/hpp-exec/run.sh" bash -lc \
  'cd ~/devel/src && make all'
```

Docker must be usable without `sudo`. Then validate the host and HPP model:

```bash
source "${MFJA_WS:-$HOME/mfja_ws}/install/setup.bash"
ros2 run mfja_staubli_manipulation_demos room315_check_setup.sh
ros2 run mfja_staubli_manipulation_demos \
  room315_hpp_manipulation.sh --build-only
```

The wrappers discover common workspace and `hpp-exec` locations. Override only
non-standard installations:

- `MFJA_WS=/path/to/workspace` or
  `MFJA_SETUP=/path/to/install/setup.bash`;
- `HPP_EXEC_DIR=/path/to/hpp-exec`;
- `ROS_SETUP=/opt/ros/<distribution>/setup.bash`;
- `ROS_DOMAIN_ID=<id>`, default `7`;
- `STAUBLI_SETUP=/path/to/staubli_ws/install/local_setup.bash`, for real
  Staubli messages.

Give every workstation on the same network a different `ROS_DOMAIN_ID`.

## Fixed-support simulation

Use this path to test the manipulation without moving a shuttle:

```bash
# Terminal 1: put one stopped shuttle directly at pickup slot 3
source "${MFJA_WS:-$HOME/mfja_ws}/install/setup.bash"
ros2 run mfja_staubli_manipulation_demos \
  room315_demo.sh right_start_slot:=3

# Terminal 2, after Gazebo is ready
source "${MFJA_WS:-$HOME/mfja_ws}/install/setup.bash"
ros2 run mfja_staubli_manipulation_demos \
  room315_manipulation_demo.sh
```

During scene startup, the simulated arm controller targets
`[0, 50, 70, 0, 55, 0]` degrees. The fixed runner creates the visible payload
and executes a shuttle-to-table cycle. It does not import
`mfja_rail_interfaces`, publish a shuttle command, or wait for rail sensors.
The shuttle remains only as a stationary collision/contact support. Pass
`--shuttle-pose X Y Z ROLL PITCH YAW` when using a support pose other than
slot 3. This option changes the pose used by the payload and HPP; it does not
move the Gazebo shuttle. It must match the `right_start_slot` selected when the
scene was launched.

For scene-only diagnostics that do not run either manipulation cycle, the
launch also accepts `enable_shuttles:=false`. The fixed runner still models the
shuttle as its pickup support and must not be used with that setting.

## Classroom simulation

The short procedure is in the [training runbook](docs/room315_training_runbook.md).
The two commands are:

```bash
# Terminal 1
source "${MFJA_WS:-$HOME/mfja_ws}/install/setup.bash"
ros2 run mfja_staubli_manipulation_demos room315_demo.sh

# Terminal 2, after Gazebo is ready
source "${MFJA_WS:-$HOME/mfja_ws}/install/setup.bash"
ros2 run mfja_staubli_manipulation_demos room315_moving_shuttle_demo.sh
```

The coordinator follows one linear sequence:

1. add the destination shuttle and initialize the visible payload;
2. command and verify the rail route;
3. stop the shuttle and read its stable measured pose;
4. plan the transfer with HPP;
5. verify the spawned arm start and execute approach, transfer, and retreat;
6. close the simulated fingers before transfer and open them before retreat.

Any active shuttle receives `OFF` after an error or Ctrl-C. Stop terminal 2
before stopping Gazebo.

The mobile entry point exposes the scenario identifiers that a demonstrator is
likely to change: `--pickup-shuttle-name`, `--drop-shuttle-name`,
`--drop-start-slot`, `--pickup-sensor`, and `--drop-sensor`. Run it with
`--help` for the current defaults. The destination shuttle is deliberately
created stopped. Transport topics and timing tolerances remain implementation
defaults in the adapter.

The grasp is deliberately kinematic. HPP owns payload collisions and contact
semantics; Gazebo visualizes the finger and payload motion. During transfer,
the visible box follows the HPP object pose synchronized to measured arm
progress rather than unstable contact physics.

## HPP checks

Planning is the default; `--execute` is required for ROS output:

```bash
ros2 run mfja_staubli_manipulation_demos \
  room315_hpp_manipulation.sh \
  --direction shuttle-to-shuttle \
  --shuttle-pose -15.310 -5.536 0.839346 0 0 -0.002 \
  --destination-shuttle-pose -14.770 -5.536 0.839346 0 0 -0.0014
```

These are example measured stopping poses from the moving scenario. The fixed
runner instead uses the nominal slot-3 pose
`-15.240 -5.536 0.839 0 0 0` by default.

The Staubli itself is registered in the Room 315 world at `x=-15.251`, `y=-6`,
`z=1`, `yaw=0` (metres and radians, with zero roll and pitch). Gazebo uses this
pose directly; HPP applies its inverse when loading the fixed cell.

The manipulation-specific SDF owns the simulation start configuration, while
the fixed and moving demo runners own payload creation. A direct `--execute`
invocation therefore requires the payload entity and arm start to be prepared
already.

## Model organization

Shared descriptions live in `mfja_3rd_floor_description`:

- `urdf/staubli_tx2_60l.urdf`: canonical HPP robot and conservative gripper
  collision envelope;
- `urdf/room315_cell.urdf`: canonical fixed-cell HPP obstacles;
- `models/staubli_tx2_60l/meshes/gripper`: Ali's complete edited visual and
  the supplied body, jaw, and robot-side adapter meshes;
- `models/staubli_tx2_60l/cad`: source STEP files retained for provenance and
  future model work.

This package contains only manipulation-specific descriptions:

- `hpp/staubli_tx2_60l_manipulation.srdf`: gripper semantics and required
  self-collision exclusions;
- `hpp/room315_payload_box.*`, `room315_shuttle_deck.*`, and
  `room315_staubli_table_drop_zone.*`: object/support collision and contact
  models;
- `models/staubli_tx2_60l_gripper/model.sdf`: articulated Gazebo gripper and
  arm-controller startup target.

Gazebo renders the supplied 50 x 24.7 x 25 mm SCHUNK body, custom jaws, and
72 mm robot-side adapter at millimetre scale. Its collision primitives,
gripper-side attachment, joint reference, pneumatic timing, and dynamics are
still simulation approximations.

HPP uses a fixed conservative mount/body/finger envelope covering the delivered
CAD revisions and full jaw sweep. The rear adapter remains an environment
collision link; only unavoidable internal wrist overlaps are disabled.

Confirmed model inputs are:

- SCHUNK PGN-plus-P 40, product ID 318448;
- pneumatic actuation and 2.5 mm stroke per jaw;
- the supplied split CAD and Ali's complete edited assembly;
- a 64.7 mm body-local jaw-tip coordinate from the source assembly.

The CAD does not calibrate the Staubli `tool0` registration or a physical
contact TCP. Ali's complete visual has a 75 mm rear plate while the newer split
adapter is 72 mm, and their jaw reference poses differ by 0.65 mm per side.
The conservative HPP envelope covers both until the installed revision is
observed.

## Known real-robot interfaces

No custom Staubli arm adapter is needed. The execution path is:

```text
hpp_exec.execute_segments
  -> FollowJointTrajectory
     /manipulator_controller/joint_trajectory_action
  -> driver-internal JointTrajectory
     /joint_path_command
```

The driver joints are `joint_1` through `joint_6`; measured state is read from
`/joint_states`. On the deployed robot, verify the action before commissioning:

```bash
ros2 action list -t
```

The pneumatic gripper command is also confirmed:

```bash
# Open
ros2 service call /io_interface/write_single_io staubli_msgs/srv/WriteSingleIO \
  "{module: {id: 2}, pin: 0, state: true}"

# Close
ros2 service call /io_interface/write_single_io staubli_msgs/srv/WriteSingleIO \
  "{module: {id: 2}, pin: 0, state: false}"
```

`response.code.val == 1` means the controller accepted the IO write; `-1`
means failure. It does not measure pressure, jaw position, contact, or grasp.

After the remaining model commissioning is complete and motion is authorized,
the one-cycle transport flags are:

```bash
ros2 run mfja_staubli_manipulation_demos \
  room315_hpp_manipulation.sh --execute \
  --payload-output none \
  --trajectory-action /manipulator_controller/joint_trajectory_action \
  --joint-state-topic /joint_states \
  --gripper-output staubli-io \
  --q-start Q1 Q2 Q3 Q4 Q5 Q6
```

`Q1 ... Q6` must be the measured, validated HPP start in radians. The executor
requires the arm to be at this start and does not preposition hardware. It
does not automatically open the real gripper; confirm that the unloaded tool is
open before approach.

For `staubli_msgs`, expose the external driver workspace:

```bash
export STAUBLI_SETUP=/absolute/path/to/staubli_ws/install/local_setup.bash
```

## Current validation status and later commissioning

Diane's controller staging configuration is `[0, 50, 70, 0, 55, 0]` degrees
(`[0, 0.8726646260, 1.2217304764, 0, 0.9599310886, 0]` radians). With the
corrected Staubli world pose, it passes the current HPP arm/cell configuration
validation and is now the simulation/HPP default start. This model result does
not authorize physical motion.

The following are later prerequisites for a physical grasp, not missing ROS
interface information:

- measure the assembled `tool0`-to-gripper and intended contact-TCP transform,
  and identify the installed adapter and jaw open/closed state;
- model the real workpiece and grasp feature—the 70 x 50 x 60 mm teaching box
  is semantic and cannot fit between the narrow custom jaws;
- test unloaded IO and establish pressure, settle time, payload mass/centre of
  mass, low-speed limits, and operator approval;
- verify physical rail interfaces before running the two-shuttle coordinator
  outside Gazebo;
- obtain redistribution permission before publishing third-party CAD.

See `mfja_3rd_floor_description/THIRD_PARTY.md` for asset provenance.

## Tests

```bash
colcon test --packages-select mfja_staubli_manipulation_demos \
  --event-handlers console_direct+
colcon test-result --verbose
```

Current HPP semantics:

- gripper: `staubli/tool0_gripper`;
- payload handle: `box/top_handle`;
- payload support: `box/bottom_surface`;
- supports: `shuttle/top_surface`, `drop_shuttle/top_surface`,
  `staubli_table/drop_zone`.
