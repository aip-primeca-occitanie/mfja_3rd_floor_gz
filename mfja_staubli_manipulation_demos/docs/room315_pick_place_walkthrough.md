# Room 315 Manipulation Architecture

This is the instructor and maintainer map for the Stage 2 exercise. Learners
should use the [classroom runbook](room315_training_runbook.md); installation,
model status, and real-robot readiness live in the package
[README](../README.md).

## Components

| File | Responsibility |
|---|---|
| `launch/room_315_staubli_shuttle_manipulation_demo.launch.py` | Start Room 315, Staubli, bridges, optional shuttle subsystem, and optional GUI. |
| `scripts/room315_manipulation_sequence.py` | Prepare the payload, then execute a fixed-support manipulation without importing rail interfaces. |
| `scripts/room315_moving_shuttle_sequence.py` | Add the supported two-shuttle sequence and measured poses around the fixed-support runner. |
| `hpp/room315_shuttle_manipulation.py` | Build endpoints, plan one manipulation cycle, and optionally execute it. |
| `hpp/room315_problem.py` | Load models, define contacts/handles, and construct the manipulation graph. |
| `hpp/room315_planning.py` | Generate grasp targets, plan graph transitions, sample paths, and form three execution segments. |
| `hpp/room315_execution.py` | Select arm/gripper outputs, synchronize the Gazebo payload, and verify measured endpoints. |
| `mfja_3rd_floor_description/urdf/staubli_tx2_60l.urdf` | Canonical HPP arm/gripper model shared by both course stages. |
| `mfja_3rd_floor_description/urdf/room315_cell.urdf` | Canonical fixed Room 315 HPP collision environment. |

The package wrappers resolve ROS package shares and mount only the demo and
description packages read-only in `hpp-exec`. This works with copied and
symlinked colcon installs and avoids source-tree-only paths.

The dependency direction is deliberate:

```text
room315_manipulation_demo.sh
  -> room315_manipulation_sequence.py
     -> room315_hpp_manipulation.sh
        -> HPP problem + planning + execution

room315_moving_shuttle_demo.sh
  -> room315_moving_shuttle_sequence.py (rail adapter)
     -> room315_manipulation_sequence.py (shared simulation runner)
```

Removing shuttle motion therefore means selecting the fixed entry point, not
commenting out part of the mobile scenario. Both paths keep the HPP plan and
execution code unchanged.

| Layer | Commands rail | Initializes payload | Runs HPP |
|---|---:|---:|---:|
| Scene launch | no | no | no |
| Fixed manipulation runner | no | yes | yes |
| Moving-shuttle adapter | yes | via shared runner | via shared runner |
| HPP core | no | no | yes |

## Coordinate conventions

The cell URDF stores fixture poses in the Gazebo world frame. The Staubli world
pose is:

```text
(-15.03, -6.0, 1.0, roll=0, pitch=0, yaw=0)
```

`build_problem()` loads the cell with the inverse robot world transform so the
combined HPP device is expressed in the Staubli base frame. Shuttle and table
world poses are converted with the same transform.

The payload is a freeflyer HPP object. Its seven configuration values follow
the six arm joints. The helper functions in `room315_problem.py` are the single
place where world poses are converted to and from this HPP configuration.

## Gazebo launch

The launch file:

1. creates a process-specific Gazebo transport partition;
2. starts `room_315_only.launch.py` without its GUI;
3. enables one stopped right-rail shuttle when `enable_shuttles:=true`;
4. spawns the articulated Staubli/gripper SDF with the validated HPP start as
   its arm-controller target;
5. adds ROS-Gazebo bridges for arm trajectory/state, gripper trajectory, world
   services, and starts the optional ROS rail subsystem;
6. starts the GUI as a separate optional process.

Keeping the GUI separate lets the simulation survive a GUI crash. The shell
wrapper refuses to start a second Room 315 instance because duplicate robot and
rail topics would interleave.

## Fixed-support sequence

`room315_manipulation_sequence.py` runs one direct story:

```text
initialize the payload on the fixed slot-3 shuttle
run one HPP shuttle-to-table cycle
```

Start the scene with `right_start_slot:=3`. The runner uses the matching
nominal support pose and never creates rail publishers or subscribers.

## Moving-shuttle adapter sequence

`MovingShuttleCoordinator` deliberately supports one classroom story:

```text
add destination shuttle
initialize payload on pickup shuttle
command route
wait for fresh switch and stopper acknowledgements
start pickup shuttle
wait for sensor arrival
command shuttle OFF
wait for fresh WAITING state and stable measured pose
run HPP shuttle-to-shuttle cycle using both measured shuttle poses
```

Planning starts only after the shuttle stops, so the measured pose and any
planning failure remain visible in the teaching flow.

### Rail safety behavior

- A route is accepted only after all switch states report `E` and all stoppers
  report `0` in updates received after the command.
- Arrival requires the configured sensor to report either the expected shuttle
  name or the simulator's legacy empty shuttle name.
- The moving pickup requires newer pose and state messages, `WAITING`, and a
  stable interval after `OFF`. The destination shuttle is created disabled, so
  its initial pose requires fresh stable pose samples but not `WAITING`.
- `FALLING` is a hard error.
- Every active shuttle receives `OFF` in both the motion method and top-level
  cleanup.

### Simulation start configuration

The manipulation-specific Gazebo SDF gives its six-joint arm controller the
startup target `[0, 50, 70, 0, 55, 0]` degrees, matching `DEFAULT_Q_START` in
`room315_problem.py`. The arm converges to that target while the scene starts;
the coordinators publish no setup trajectory. Before any execution, the HPP
executor still reads measured joint state and rejects a start outside its
configured tolerance. Hardware must already be at a separately validated HPP
start.

## HPP problem

`room315_problem.py:build_problem()` loads:

- Staubli as the anchored robot;
- the fixed Room 315 cell as an anchored environment;
- pickup and destination shuttle bodies as anchored supports;
- the Staubli table as an anchored support;
- the teaching payload as a freeflyer object.

It enables collision and joint-bound validation, then creates a manipulation
graph with:

- gripper `staubli/tool0_gripper`;
- object `box`;
- handle `box/top_handle`;
- object contact `box/bottom_surface`;
- environment contacts on both shuttle decks and the table drop zone.

Security margins are applied between the payload/robot and fixed cell before
the graph is initialized.

## Planning call order

`room315_shuttle_manipulation.py:main()`:

```text
build_problem()
project source and destination free configurations
direction_endpoints()
plan_manipulation()
  generate_pick_chains(source)
  generate_pick_chains(destination)
  try source/destination chain pairs
    plan each pick, transfer, release, and retreat transition
format_plan()
build_execution_plan()
if --execute:
  execute_plan()
```

For the moving entry point, DZI3R is an arrival-zone signal rather than the
final support pose. The coordinator keeps the shuttle moving to the canonical
slot position, stops it, and passes that measured stopped pose into this call
order.

Planning is the default. `--build-only` stops after graph construction;
`--execute` is the only mode that writes to ROS.

### Target and transition planning

`generate_pick_chains()` samples graph-compatible arm postures while retaining
the payload pose. It validates and scores complete source/destination chains,
preferring shorter arm motion, compatible posture, and less wrist wrapping.
The classroom planner discards a first approach that requires more than 2 rad
on any arm joint, which keeps the solution on the compact IK branch.

`plan_manipulation()` tries ordered source/destination chain pairs. Each
transition first attempts a checked direct path, then invokes
`TransitionPlanner` if needed. A pair is discarded when any transition cannot
be planned. It also rejects a first approach longer than 3 path units or a
complete cycle longer than 8 path units, so a distant branch is never sent to
the executor.

### Execution segments

The semantic grasp and release transitions split the successful graph path:

| Segment | Payload mode | Pre-actions |
|---|---|---|
| approach to pregrasp | fixed on pickup support | open the simulated gripper |
| grasp and transfer | follows HPP object pose | close the gripper, then attach the Gazebo payload |
| release and retreat | fixed on destination support | open the gripper, then place the Gazebo payload |

Every sampled robot/object configuration is validated with the corresponding
transition path validation before it becomes ROS output. Arm samples are
retimed from maximum joint motion. These three slices are represented by
`hpp_exec.Segment`; grasp and release commands are boolean pre-actions on the
segments beginning at the matching graph transitions.

## Execution outputs

`room315_execution.py:execute_plan()` always:

1. creates a measured joint-state tracker;
2. creates the selected gripper output;
3. verifies the measured arm is at the first planned configuration;
4. attaches gripper and payload pre-actions to the HPP-exec segments;
5. checks every measured segment endpoint through a post-action.

The two arm transports consume the same segment and action schedule:

| Target | Arm output | Joint state | Payload |
|---|---|---|---|
| Gazebo | `JointTrajectory` on `/staubli1/joint_trajectory` | `/staubli1/joint_states` | existing Gazebo entity |
| Staubli driver | `FollowJointTrajectory` action on `/manipulator_controller/joint_trajectory_action` through `hpp_exec.execute_segments` | `/joint_states` | physical world (`none`) |

Action output uses `hpp_exec.execute_segments()` directly. Gazebo uses the same
`hpp_exec.Segment` pre/post actions with its raw-topic sender because payload
animation must run concurrently with the arm trajectory. Action output still
requires `--payload-output none`.

Gripper outputs are:

- `joint-trajectory`: Gazebo finger joints;
- `staubli-io`: `/io_interface/write_single_io`, module `2`, pin `0`,
  `true` open and `false` close, accepting only return code `1`;
- `none`: retain semantic boundaries without a physical command.

The real gripper is not opened automatically at startup. That prevents an
unexpected hardware action before the operator confirms the unloaded tool
state.

## Gazebo payload ownership

`ManipulationCoordinator` owns spawn/delete and initial placement for both
simulation entry points:

1. spawn the visual box on the pickup support;
2. place it at the fixed pose, or let the moving adapter stream its pose while
   the shuttle moves;
3. leave the existing entity for HPP execution.

The HPP executor only calls `/set_pose`. During fixed segments it keeps the box
on its support; during transfer `follow_payload()`:

1. reads measured arm joints;
2. finds nearest forward progress on the sampled arm path;
3. interpolates the matching HPP object configuration;
4. sends that world pose to Gazebo;
5. fixes the final pose when the arm reaches the endpoint or measured progress
   reaches the last sample; a near-end timeout may also use the final pose,
   after which the normal endpoint check still runs.

Measured-progress synchronization prevents the box from flying ahead when
Gazebo executes more slowly than trajectory timestamps.

## Model boundary

Gazebo uses the supplied body, custom-jaw, and robot-side adapter meshes for
visual fidelity and articulated jaw timing. HPP uses conservative primitives
for collision checking. The payload transfer is not a contact-physics grasp
proof.

The delivered CAD confirms product geometry and a body-local tip coordinate,
but not the physical Staubli mount or contact TCP. Those calibration tasks and
the staging-configuration validation status are tracked in the package README
rather than embedded as provisional constants in the teaching flow.

## Failure interpretation

- No arm subscriber: Gazebo/controller is absent or the configured topic is
  wrong.
- Action sender failure: stop before any subsequent gripper action.
- Segment endpoint timeout: inspect measured joint state; do not loosen a
  tolerance until the commanded path and controller state are understood.
- Payload set-pose failure: reset the Gazebo scene; do not continue with a
  detached visual.
- Rail acknowledgement, arrival, stability, or `FALLING` failure: the
  coordinator stops active shuttles and exits.
- Gripper IO return `-1` or timeout: stop the manipulation sequence; a return
  `1` still requires separate observation of pneumatic motion during hardware
  commissioning.
