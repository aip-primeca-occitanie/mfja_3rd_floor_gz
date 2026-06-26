# Room 315 Pick And Place Walkthrough

This document is the detailed map for the Room 315 Staubli shuttle pick and
place demo. Read it with the source tree open. The quick mental model is:

1. `scripts/room315_demo.sh` starts Gazebo, the Room 315 world, the Staubli
   model, ROS-GZ bridges, and the right-rail shuttle node.
2. `scripts/room315_moving_shuttle_demo.sh` runs the shuttle demo coordinator.
   It moves the pickup shuttle, prepositions the robot, adds the drop shuttle,
   and starts the HPP cycle.
3. `scripts/room315_hpp_manipulation.sh` runs the HPP Python problem inside the
   `hpp-exec` container.
4. `hpp/room315_shuttle_manipulation.py` parses the one-cycle HPP command and
   calls focused modules for graph construction, planning, phase sampling, and
   execution output.

The planning model is shared between Gazebo and real execution. The outputs are
target-specific: Gazebo gets joint trajectories plus visual payload set-pose
updates; the real robot should get only the real arm/gripper commands.

## File Map

Top-level package files:

- `README.md`: quickstart commands and current surfaces.
- `docs/room315_pick_place_walkthrough.md`: this detailed call and setup map.
- `package.xml`: ROS package dependencies.
- `setup.py`: installs launch, config, HPP models/scripts, Gazebo model, and
  URDF files into the ROS package share directory.

Launch and configuration:

- `launch/room_315_staubli_shuttle_manipulation_demo.launch.py`: Gazebo launch
  wrapper for this demo.
- `config/robots_room315_gripper.yaml`: tells the Room 315 bringup to spawn one
  robot named `staubli1` using the `staubli_tx2_60l_gripper` Gazebo model.

User-facing scripts:

- `scripts/room315_env.sh`: sources the local ROS workspace environment and sets
  demo defaults.
- `scripts/room315_demo.sh`: starts a clean Room 315 Gazebo scene.
- `scripts/room315_moving_shuttle_demo.sh`: starts the moving-shuttle sequence.
- `scripts/room315_hpp_manipulation.sh`: enters `hpp-exec` and runs the HPP
  planner/executor.
- `scripts/room315_moving_shuttle_sequence.py`: ROS coordinator for shuttle
  movement, prepositioning, payload visual setup, and HPP subprocess control.

HPP planning and execution code:

- `hpp/room315_shuttle_manipulation.py`: command-line entry point for one HPP
  manipulation cycle.
- `hpp/room315_problem.py`: HPP model constants, pose conversions, and
  `build_problem()`.
- `hpp/room315_planning.py`: target sampling, pick-chain selection, transition
  planning, and execution-phase sampling.
- `hpp/room315_execution.py`: ROS publishers/subscribers, Gazebo payload
  services, gripper outputs, and phase execution.

HPP semantic models:

- `hpp/staubli_tx2_60l_manipulation.srdf`: HPP gripper semantics for the Staubli
  gripper.
- `hpp/room315_payload_box.urdf` and `.srdf`: freeflyer payload geometry, handle,
  and support contact surface.
- `hpp/room315_shuttle_deck.urdf` and `.srdf`: support deck/contact surface used
  by HPP for the shuttle.
- `hpp/room315_staubli_table_drop_zone.urdf` and `.srdf`: table drop contact.
- `hpp/room315_cell.urdf` and `.srdf`: fixed Room 315 collision environment.

Gazebo and ROS robot description:

- `models/staubli_tx2_60l_gripper/model.sdf`: actual Gazebo robot model, arm
  controller plugin, gripper controller plugin, and joint-state plugin.
- `models/staubli_tx2_60l_gripper/model.config`: Gazebo model metadata.
- `urdf/staubli_tx2_60l_gripper.urdf`: ROS/HPP robot description with fixed,
  conservative gripper links used for planning.

## Launch Flow

Start command:

```bash
mfja_staubli_manipulation_demos/scripts/room315_demo.sh
```

Call flow:

1. `room315_demo.sh`
   - sources `scripts/room315_env.sh`;
   - defaults to `ROS_DOMAIN_ID=7` unless the caller already set another
     domain;
   - checks for an existing `gz sim ... room_315` process;
   - runs `ros2 launch mfja_staubli_manipulation_demos
     room_315_staubli_shuttle_manipulation_demo.launch.py "$@"`.
2. `room_315_staubli_shuttle_manipulation_demo.launch.py`
   - declares launch arguments:
     - `gz_partition`, default `room_315_staubli_manipulation_<pid>`;
     - `gui`, default `true`;
     - `gui_render_engine`, default `ogre`;
     - `right_start_slot`, default `1`;
     - `shuttle_speed`, default `0.3`;
   - sets `GZ_PARTITION` so multiple Gazebo instances do not share transport;
   - prepends this package and `mfja_3rd_floor_description` to
     `GZ_SIM_MODEL_PATH` and `GZ_SIM_RESOURCE_PATH`;
   - includes `mfja_3rd_floor_bringup/launch/room_315_only.launch.py` with:
     - `robots=staubli`;
     - `robot_config=<this package>/config/robots_room315_gripper.yaml`;
     - right rail enabled, left rail disabled;
     - one right-rail shuttle;
     - shuttle start disabled, so the coordinator controls motion;
     - `room315_shuttle_speed=<shuttle_speed>`;
   - starts an extra `ros_gz_bridge parameter_bridge` for the gripper topic;
   - starts the Gazebo GUI after a 5 second delay if `gui:=true`.

The included Room 315 bringup owns the world, robot spawn, robot state publisher,
default arm bridges, world service bridges, and the kinematic shuttle node. This
demo launch only selects the gripper model and adds the gripper bridge.

## Gazebo Controllers

The Gazebo model is `models/staubli_tx2_60l_gripper/model.sdf`.

Plugins in that SDF:

- `gz::sim::systems::JointStatePublisher`
  - publishes arm and gripper joint states from Gazebo;
  - includes `joint_1` through `joint_6`;
  - includes `gripper_left_finger_joint` and `gripper_right_finger_joint`.
- `gz::sim::systems::JointTrajectoryController` for the arm
  - controls `joint_1` through `joint_6`;
  - receives Gazebo joint trajectories on the model trajectory topic bridged by
    the Room 315 bringup.
- `gz::sim::systems::JointTrajectoryController` for the gripper
  - explicit topic: `/staubli1/gripper_joint_trajectory`;
  - controls `gripper_left_finger_joint` and `gripper_right_finger_joint`;
  - initial positions are open at `0.028`;
  - closed positions are `0.0255`, matching the SCHUNK PGN-plus-P 40
    2.5 mm per-jaw stroke.

There is no `ros2_control` controller manager in this Gazebo demo. Control is:

```text
ROS JointTrajectory topic
  -> ros_gz_bridge
  -> Gazebo JointTrajectoryController plugin
  -> simulated joints
  -> Gazebo joint state
  -> ros_gz_bridge
  -> ROS JointState topic
```

## Main ROS Interfaces

Robot:

- `/staubli1/joint_trajectory`
  - type: `trajectory_msgs/msg/JointTrajectory`;
  - published by the moving-shuttle coordinator for prepositioning;
  - published by the HPP executor for manipulation phases;
  - bridged to the Gazebo arm trajectory controller.
- `/staubli1/joint_states`
  - type: `sensor_msgs/msg/JointState`;
  - read by the coordinator to know when prepositioning is done;
  - read by the HPP executor to synchronize execution and payload following.
- `/staubli1/gripper_joint_trajectory`
  - type: `trajectory_msgs/msg/JointTrajectory`;
  - published by the HPP executor at semantic grasp/release when
    `--gripper-output joint-trajectory`;
  - bridged by this package launch file to the Gazebo gripper controller.
- `/io_interface/write_single_io`
  - type: `staubli_msgs/srv/WriteSingleIO`;
  - provided by the Staubli ROS 2 VAL3 IO interface;
  - used by `--gripper-output staubli-io` for the real pneumatic gripper.

Gazebo world services:

- `/world/room_315_only/create`
  - type: `ros_gz_interfaces/srv/SpawnEntity`;
  - used to spawn the visible payload box.
- `/world/room_315_only/remove`
  - type: `ros_gz_interfaces/srv/DeleteEntity`;
  - used by `--replace-box`.
- `/world/room_315_only/set_pose`
  - type: `ros_gz_interfaces/srv/SetEntityPose`;
  - used to keep the visual payload on the shuttle before pickup and attached
    to the HPP object pose during transfer.

Right rail:

- `/room_315/rails/right/shuttles/add`
  - type: `mfja_rail_interfaces/srv/AddShuttle`;
  - used to add the second/drop shuttle in the default scenario.
- `/room_315/rails/right/shuttles/command`
  - type: `mfja_rail_interfaces/msg/ShuttleCommand`;
  - used to start and stop the pickup shuttle.
- `/room_315/rails/right/shuttles/state`
  - type: `mfja_rail_interfaces/msg/ShuttleState`;
  - used to track commanded shuttle state.
- `/room_315/rails/right/shuttles/pose_cmd`
  - type: `geometry_msgs/msg/PoseStamped`;
  - used to read the pickup shuttle pose.
- `/room_315/rails/right/shuttles/<drop-name>/pose_cmd`
  - type: `geometry_msgs/msg/PoseStamped`;
  - used to read the drop shuttle pose.
- `/room_315/rails/right/sensors/feedback`
  - type: `mfja_rail_interfaces/msg/SensorFeedback`;
  - used to detect arrival at the pickup sensor.
- `/room_315/rails/right/switches/command`
  - type: `mfja_rail_interfaces/msg/SwitchCommand`;
  - used to set right-rail switches to the route layout.
- `/room_315/rails/right/stoppers/command`
  - type: `mfja_rail_interfaces/msg/StopperCommand`;
  - used to open stoppers before shuttle motion.

## Moving-Shuttle Coordinator Call Order

Entry point:

```bash
mfja_staubli_manipulation_demos/scripts/room315_moving_shuttle_demo.sh --replace-box
```

Script flow:

1. `room315_moving_shuttle_demo.sh`
   - sources `room315_env.sh`;
   - runs `python3 -u scripts/room315_moving_shuttle_sequence.py`;
   - passes `--hpp-script scripts/room315_hpp_manipulation.sh`;
   - forwards unknown arguments to HPP.
2. `room315_moving_shuttle_sequence.py:main()`
   - parses coordinator options and separates unknown HPP options;
   - inserts `--start-tolerance <hpp_start_tolerance>` unless the user already
     passed one to HPP;
   - creates `MovingShuttleCoordinator(args)`;
   - waits for publishers/subscribers and initial sensor feedback;
   - optionally adds the drop shuttle;
   - spawns or updates the visible payload on the pickup shuttle;
   - optionally starts an early HPP preplan subprocess;
   - prepares the rail route;
   - starts robot prepositioning;
   - starts the pickup shuttle and waits for arrival;
   - waits for robot preposition completion;
   - triggers the ready preplanned HPP execution, or discards it and starts a
     fresh HPP run if the real shuttle pose differs too much from the nominal
     preplanned pose.

Important methods in `MovingShuttleCoordinator`:

- `__init__(args)`
  - creates all ROS publishers, subscribers, and service clients.
- `wait_for_publishers(timeout)`
  - waits until the arm, switch, stopper, and shuttle command topics have
    subscribers.
- `add_drop_shuttle()`
  - calls `/room_315/rails/right/shuttles/add`;
  - waits for the drop shuttle pose/sensor state.
- `ensure_payload_on_shuttle(shuttle_name)`
  - optionally deletes the existing Gazebo payload;
  - spawns the visible payload;
  - sets the payload pose from the current shuttle pose.
- `prepare_route()`
  - publishes switch commands;
  - publishes stopper-open commands;
  - waits `--route-settle-s`.
- `start_preposition_arm()`
  - publishes a conservative trajectory from current joint state to the HPP
    start configuration on `/staubli1/joint_trajectory`;
  - returns timing/target data for later waiting.
- `move_to_pickup_slot(...)`
  - publishes shuttle ON;
  - watches the configured rail sensor;
  - publishes shuttle OFF at arrival;
  - waits for a stable stopped pose;
  - streams the visual payload pose while the shuttle moves.
- `wait_preposition_arm(preposition)`
  - waits until `/staubli1/joint_states` is close enough to the HPP start.
- `start_hpp_cycle_preplan(...)`
  - launches `room315_hpp_manipulation.sh` as a subprocess;
  - passes `--ready-file`, `--start-file`, and `--abort-file`;
  - HPP plans immediately, touches `ready-file`, then waits.
- `use_preplanned_or_run(...)`
  - compares actual shuttle pose to the nominal preplanned pose;
  - if close enough, creates the `start-file` and waits for HPP to execute;
  - otherwise creates the `abort-file` and launches HPP again with the actual
    shuttle pose.

The key speed optimization is that prepositioning and shuttle arrival overlap:

```text
prepare route
start arm preposition
start shuttle arrival
wait shuttle stopped
wait arm at HPP start
trigger HPP execution
```

## HPP Wrapper

`scripts/room315_hpp_manipulation.sh` is the boundary between ROS/Gazebo on the
host and HPP inside `hpp-exec`.

It does the following:

1. Computes `MFJA_REPO` as the parent repo path.
2. Optionally sources `scripts/room315_local_env.sh` if it exists.
3. Requires `HPP_EXEC_DIR` and checks that `$HPP_EXEC_DIR/run.sh` exists.
4. Mounts the MFJA repo read-only into the container at
   `/home/user/mfja_3rd_floor_gz`.
5. Runs `hpp-exec/run.sh --domain-id "${ROS_DOMAIN_ID:-7}"`; the host demo
   scripts use the same default domain.
6. Inside the container:
   - sources `/home/user/devel/config.sh`;
   - sets `LD_LIBRARY_PATH`, `PYTHONPATH`, and `ROS_PACKAGE_PATH`;
   - runs
     `/home/user/mfja_3rd_floor_gz/mfja_staubli_manipulation_demos/hpp/room315_shuttle_manipulation.py`.

This is why host ROS topics are still visible to HPP execution: the wrapper uses
the same ROS domain ID as the host.

## HPP Planner Call Order

Entry point:

```bash
mfja_staubli_manipulation_demos/scripts/room315_hpp_manipulation.sh --plan-only
```

or, during the full demo, the moving-shuttle coordinator calls the same wrapper
with `--execute` and pose arguments.

High-level call tree:

```text
room315_shuttle_manipulation.py:main()
  parse HPP/execution arguments
  room315_problem.py:build_problem()
  build source/destination payload configurations
  room315_problem.py:project_free_configuration()
  room315_planning.py:direction_endpoints()
  room315_planning.py:plan_manipulation()
    generate_pick_chain(source)
    generate_pick_chain(destination)
    plan_transition(...) for pick, transfer, release transitions
  room315_planning.py:format_plan()
  room315_planning.py:build_execution_phases()
    build_execution_phase(approach, fixed payload)
    build_execution_phase(grasp-transfer, payload follows gripper)
    build_execution_phase(release-retreat, fixed payload)
  if --execute:
    room315_execution.py:wait_for_execution_start()
    room315_execution.py:execute_plan()
```

`room315_problem.py:build_problem(shuttle_pose, destination_shuttle_pose)`:

- creates one HPP `Device`;
- loads the Staubli gripper URDF/SRDF as `staubli`;
- loads the Room 315 fixed collision model as `room315`;
- loads the pickup shuttle deck as `shuttle`;
- optionally loads the destination shuttle deck as `drop_shuttle`;
- loads the table drop zone as `staubli_table`;
- loads the payload as a freeflyer object named `box`;
- sets bounds on `box/root_joint`;
- enables `CollisionValidation` and `JointBoundValidation`;
- creates a manipulation `Graph`;
- creates a `ConstraintGraphFactory`;
- declares:
  - gripper: `staubli/tool0_gripper`;
  - object: `box`;
  - handle: `box/top_handle`;
  - object contact: `box/bottom_surface`;
  - environment contacts: shuttle, optional drop shuttle, and table;
- applies security margins;
- initializes the graph.

`room315_planning.py:plan_manipulation(...)`:

- sets the graph on the HPP problem;
- creates a `TransitionPlanner`;
- calls `generate_pick_chain()` for the source support;
- calls `generate_pick_chain()` for the destination support, preferring a
  posture close to the source pick posture;
- scores several valid pick chains with `score_pick_chain()` so repeated HPP
  target generation prefers shorter motion, a compatible final posture, and less
  wrist wrapping;
- plans all transition segments:
  - approach source;
  - grasp source;
  - transfer while carrying the object;
  - release at destination;
  - retreat back to a free state.

`room315_planning.py:build_execution_phases(...)`:

- finds the semantic grasp transition;
- finds the semantic release transition;
- splits the planned path into three execution phases:
  - approach source with fixed payload;
  - grasp-transfer with payload following the HPP object pose;
  - release-retreat with fixed destination payload;
- samples every HPP path;
- validates every sampled robot/object configuration;
- retimes arm samples into ROS `JointTrajectory` timing.

## HPP Execution Call Order

`room315_execution.py:execute_plan(robot, phases, q_source, args)`:

1. Calls `rclpy.init()` and creates node `room315_hpp_manipulation`.
2. Creates the arm trajectory publisher on `/<robot-name>/joint_trajectory`.
3. Creates `JointStateTracker` on `/<robot-name>/joint_states`.
4. Creates the configured gripper output:
   - `none`: no command;
   - `bool`: publish `std_msgs/msg/Bool`;
   - `joint-trajectory`: publish `trajectory_msgs/msg/JointTrajectory`;
   - `staubli-io`: call the Staubli VAL3
     `/io_interface/write_single_io` service.
5. If `--payload-output gazebo`:
   - creates clients for `/world/<world>/create`, `/remove`, and `/set_pose`;
   - deletes the existing payload if `--replace-box`;
   - spawns the visible payload box;
   - sets it to the planned source pose.
6. Handles the start pose using `--start-mode`:
   - `check`: require the arm to already be near the planned start;
   - `move`: publish a conservative start trajectory;
   - `snap`: publish a short simulation-only start command.
7. Executes phase 0 with `execute_phase(...)`.
8. Runs `semantic_grasp(...)`:
   - closes the gripper output if configured;
   - makes sure the visual payload is at the phase start pose.
9. Executes phase 1 with `execute_phase(...)`.
10. Runs `semantic_release(...)`:
    - opens the gripper output if configured;
    - sets the visual payload at the phase end pose.
11. Executes phase 2 with `execute_phase(...)`.

`execute_phase(...)`:

- converts the sampled arm configurations into a `JointTrajectory`;
- publishes the trajectory;
- if the phase payload mode is `follow`, calls `follow_payload(...)`;
- waits for the phase end with measured joint-state feedback.

`follow_payload(...)`:

- reads the measured arm state;
- finds the nearest planned arm progress;
- sends the corresponding HPP payload pose to Gazebo with `/set_pose`;
- keeps the visual box synchronized with actual robot progress instead of wall
  clock time.

This measured-progress synchronization is important. It prevents the visual box
from flying ahead of the gripper when Gazebo executes the arm slower than the
trajectory timestamps.

## Payload And Grasp Semantics

The Gazebo payload is intentionally not a physical grasp proof. In this demo:

- HPP owns collision checking and contact semantics.
- Gazebo shows the gripper opening/closing and the payload motion.
- HPP uses fixed conservative gripper geometry. Gazebo uses the real gripper
  stroke, but the gripper body and fingers stay as placeholder geometry until
  the real CAD/mounting is available.
- The visible Gazebo payload is kinematically moved through `/set_pose`.
- The actual transfer pose comes from the HPP object configuration.

That is deliberate because small rigid-body grasps in Gazebo are often sensitive
to contact parameters, friction, solver settings, timestep, and controller lag.
For this demo, Gazebo should verify geometry, timing, topic wiring, clearances,
and the manipulation sequence. The real grasp quality is verified on hardware
with low speed, measured TCP/finger dimensions, and conservative clearances.

## Gazebo Versus Real Robot Separation

Shared between Gazebo and real robot:

- HPP robot/object/environment URDF/SRDF files.
- Manipulation graph.
- Contact surfaces and handle definitions.
- Path planning and execution phase sampling.
- Semantic sequence: approach, close, transfer, open, retreat.

Gazebo-only:

- `launch/room_315_staubli_shuttle_manipulation_demo.launch.py`;
- Gazebo robot SDF;
- ROS-GZ bridges;
- Gazebo world services;
- `--payload-output gazebo`;
- visual payload spawning and set-pose following.

Real robot layer:

- keep the same HPP script;
- use `--payload-output none`;
- use `--trajectory-topic` and `--joint-state-topic` if the real bridge exposes
  different topic names;
- use `--gripper-output staubli-io` for the SCHUNK PGN-plus-P 40 pneumatic
  gripper once the real valve pin is known;
- add an action adapter if the real arm controller only exposes
  `FollowJointTrajectory`;
- validate real TCP, tool transform, gripper stroke, and approach clearances
  before increasing speed.

The clean rule is: keep planning common, swap execution outputs.

## Useful Debug Commands

Check robot topics:

```bash
ros2 topic list | grep staubli1
ros2 topic info /staubli1/joint_trajectory
ros2 topic echo /staubli1/joint_states --once
```

Check gripper command path:

```bash
ros2 topic info /staubli1/gripper_joint_trajectory
ros2 topic echo /staubli1/gripper_joint_trajectory --once
ros2 service type /io_interface/write_single_io
ros2 service call /io_interface/write_single_io staubli_msgs/srv/WriteSingleIO \
  "{module: {id: 2}, pin: PIN_TO_CONFIRM, state: true}"
```

Check Gazebo services:

```bash
ros2 service list | grep /world/room_315_only
```

Check shuttle state and sensors:

```bash
ros2 topic echo /room_315/rails/right/shuttles/state
ros2 topic echo /room_315/rails/right/sensors/feedback
ros2 topic echo /room_315/rails/right/shuttles/pose_cmd
```

Dry-run HPP:

```bash
mfja_staubli_manipulation_demos/scripts/room315_hpp_manipulation.sh --build-only
mfja_staubli_manipulation_demos/scripts/room315_hpp_manipulation.sh --plan-only \
  --direction shuttle-to-shuttle
```

Run one HPP cycle against a running Gazebo scene:

```bash
mfja_staubli_manipulation_demos/scripts/room315_hpp_manipulation.sh --execute \
  --replace-box \
  --gripper-output joint-trajectory
```

Run the full moving-shuttle flow:

```bash
HPP_EXEC_DIR=$HOME/devel/nix-hpp/src/hpp-exec \
  mfja_staubli_manipulation_demos/scripts/room315_moving_shuttle_demo.sh --replace-box
```

## Common Failure Points

Robot not visible in Gazebo:

- make sure only one Room 315 Gazebo instance is running;
- make sure the package was rebuilt/installed after adding
  `robots_room315_gripper.yaml` and the gripper model;
- confirm `GZ_SIM_MODEL_PATH` includes this package install/source model path.

Gazebo gripper does not move:

- confirm the extra bridge exists:
  `/staubli1/gripper_joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory`;
- confirm HPP is run with `--gripper-output joint-trajectory`;
- confirm the SDF gripper controller topic is
  `/staubli1/gripper_joint_trajectory`.

Real pneumatic gripper does not move:

- launch the Staubli IO interface from `staubli_val3_driver`;
- confirm `/io_interface/write_single_io` is available;
- confirm HPP is run with `--gripper-output staubli-io --staubli-io-pin <pin>`;
- confirm the module id, pin number, and open/close polarity on the real cell.

Payload looks offset or jumps:

- use `--replace-box` to remove stale Gazebo payload entities;
- check that the HPP payload size matches the Gazebo visual payload size;
- keep `follow_payload()` synchronized from measured joint states;
- avoid tuning payload motion from wall-clock trajectory time alone.

HPP preplan is discarded:

- the actual shuttle pose differed from the nominal preplan pose;
- either improve shuttle stopping repeatability, pass the actual pose, or adjust
  `--preplan-position-tolerance` and `--preplan-yaw-tolerance`.

Execution times out at phase end:

- Gazebo may be lagging behind the planned timestamps;
- check `/staubli1/joint_states`;
- reduce speed or increase execution tolerance only after confirming the arm is
  following the intended path safely.

## How To Explain The Demo

The demo starts a Room 315 Gazebo world with a Staubli TX2-60L and a right-rail
kinematic shuttle. A coordinator node adds/positions shuttles, puts the visual
payload on the pickup shuttle, moves the robot toward the HPP start posture, and
brings the pickup shuttle to the robot. In parallel, HPP builds a manipulation
graph from the Staubli gripper, the payload handle/contact, the shuttle deck,
and the drop support. Once shuttle and robot are ready, HPP publishes three arm
trajectory phases: approach, grasp-transfer, and release-retreat. Gazebo gripper
commands show close/open actions, while the payload visual follows the HPP
object pose through Gazebo set-pose services. For the real robot, the planning
stays the same, the Gazebo-only payload/world outputs are disabled, and semantic
open/close actions call the Staubli IO service.
