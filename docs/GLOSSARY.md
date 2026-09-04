# Glossary

This glossary defines the terms used across the MFJA user, operator,
maintenance, perception, planning, and safety documentation.

## Platform and Build Terms

**ament**

The ROS 2 build/package framework used by all four packages in this repository.

**colcon workspace**

A directory containing `src/`, with generated `build/`, `install/`, and `log/`
directories after a build. This repository belongs under the workspace `src/`
directory but is not itself a ROS package.

**overlay**

The environment created by sourcing a workspace's `install/setup.bash`. The
overlay tells ROS 2 where the built MFJA packages, executables, interfaces, and
resources are installed.

**package share**

The installed data directory returned by
`ros2 pkg prefix --share <package>`. Launch files, configuration, worlds, and
models are normally resolved from a package share rather than the source path.

**rosdep**

The ROS dependency resolver that reads `package.xml` and installs system
packages for the selected ROS distribution.

**symlink install**

A colcon mode that links many installed non-compiled resources back to their
source files. It shortens development cycles but does not remove the need to
rebuild after adding install entries or changing compiled/interfaces metadata.

## ROS and Gazebo Terms

**Gazebo Harmonic**

The simulator generation used here. Its main library/API level in this
workspace is `gz-sim8`.

**ROS-Gazebo bridge (`ros_gz`)**

Processes and services that translate ROS 2 messages/services to Gazebo
Transport and back.

**simulation time**

Time published on `/clock`. Nodes with `use_sim_time:=true` advance according to
Gazebo, not wall-clock time. A paused or slow simulation pauses or slows their
timers.

**Gazebo partition (`GZ_PARTITION`)**

A Gazebo Transport isolation name. It does not rename the fixed ROS rail topics,
so it does not make two Room 315 floor launches safe on one ROS graph.

**SDF**

Simulation Description Format. The source of Gazebo world/model geometry,
plugins, joints, sensors, and physics settings in this repository.

**URDF**

Unified Robot Description Format. Used here mainly by
`robot_state_publisher` for robot links, joints, and ROS transforms.

**entity**

A named object in a Gazebo world, such as a robot, shuttle, switch, marker, or
table.

**model URI**

An SDF reference such as `model://room315_shuttle_R1`, resolved through Gazebo
resource/model paths.

## Room 315 Rail Terms

**rail side**

The independent `right` or `left` Room 315 directed rail network. Each side has
its own node, fleet, topics, devices, and configuration.

**segment**

A named, forward-only portion of the rail graph. Its geometry comes from an
explicitly referenced CSV and its connectivity comes from a network YAML.

**public segment name**

The label published through ROS and used in documentation/scenarios. The left
rail has a deliberate mapping between internal geometry names and public names.

**arc length (`s`)**

Distance along one segment from its start, in meters.

**normalized arc length (`s_ratio`)**

Position along a segment divided by segment length. `0` is the segment start and
`1` is the segment end.

**path backend**

The geometry sampling method. `cubic_hermite` is the normal smooth,
arc-length-parameterized backend; `polyline` follows CSV edges directly for
comparison/debugging.

**slot**

One of four configured rail positions per side used for shuttle startup,
station goals, and indexing-zone occupancy.

**shuttle identity**

A stable public identifier `R1`-`R4` or `L1`-`L4`, coupled to a Gazebo entity,
visual tags/colors, and dataset/planning identity.

**switch**

A routing device named `A1`-`A4`. State `E`/`EXTERIOR` selects the exterior
branch and `I`/`INTERIOR` selects the interior branch.

**stopper**

An independent hold/release device named `A1`-`A4`. State `0` means open/pass;
state `1` means closed/stop.

**DZI sensor**

A binary indexing-zone occupancy sensor associated with a slot area.

**DA sensor**

A binary approach/branch occupancy sensor around a switch. Suffixes distinguish
main, exterior, and interior detector positions.

**stopper-linked sensor**

A normal binary position sensor whose point is derived from a stopper position
minus `before_stopper_m`.

**device marker**

A visual-only, collision-free Gazebo entity showing a configured sensor or
stopper position/state.

**kinematic shuttle**

A shuttle moved by sampling the rail graph and calling Gazebo `set_pose`, rather
than by wheel/contact forces.

**`DISABLED`**

Shuttle mode confirming that drive has explicitly been turned off.

**`WAITING`**

Shuttle mode indicating that motion is currently held, for example by a closed
stopper or shuttle-spacing rule. It is not an `OFF` acknowledgement.

**`FALLING`**

A latched fault mode entered when the directed route has no valid successor for
the actual switch configuration.

**headway/collision distance**

The minimum configured center-to-center separation used by the kinematic
multi-shuttle protection rule.

## Robot Terms

**robot selector**

A launch/helper token that can be an exact instance name, supported alias,
numeric YAML order, `all`, or `none` depending on the command.

**joint trajectory**

A ROS command containing named joint positions and a time from start, bridged to
the simulated robot controller.

**symmetric gripper**

The custom Gazebo mechanism/controller that moves two opposing industrial
gripper jaws using one bounded per-jaw target.

**frame prefix**

A robot-instance prefix applied to ROS transforms so multiple fixed-base robots
do not publish duplicate frame names.

## AI, Perception, and Planning Terms

**vision-language-action (VLA)**

An external architecture family in which vision and language condition action
selection. Room 315 does not implement an end-to-end learned VLA policy: its
language model and visual-state model are separate, PlanSys2 selects symbolic
actions, and the deterministic rail-safety supervisor alone authorizes typed
rail commands.

**model input**

The fields declared as inputs to a learned model. Exact Gazebo pose, true
segment, binary rail feedback, and oracle labels are excluded from the current
visual model input.

**visual fact**

A structured learned output such as presence-qualified bounding box, rail block
or along-segment location, and loaded/empty classification.

**privileged evaluation**

Simulator/controller truth retained outside model input for reset, auditing,
safety vetoes where declared, labels, and evaluation.

**presence provider**

The boundary that determines whether each canonical shuttle is present. Gazebo
`ShuttleState` names/timestamps are used for simulation presence, not visual
localization.

**raw observation**

A constructed visual-state message before the complete validation/acceptance
gate.

**accepted observation**

A fresh `VisualStateObservation` that passes the configured contract and may be
used for grounding/planning.

**state fusion**

The declared combination of visual facts with limited deterministic state such
as presence and device/safety information, without silently replacing learned
localization with oracle pose.

**TaskGoal**

The validated, confirmed structured representation of one supported English
transport or inspection request.

**PDDL**

Planning Domain Definition Language. It describes symbolic objects, predicates,
actions, preconditions, and effects for Room 315 planning.

**PlanSys2**

The ROS 2 planning framework used to request a plan from the current PDDL
problem/domain. POPF is the configured planning backend in the live workflow.

**primitive command**

A single bounded action request, such as a switch mask, stopper mask, or shuttle
command, produced after plan translation and validation.

**supervisor**

The runtime boundary that validates primitive JSON commands against emergency,
identity, route, device, headway, and other safety rules before publishing typed
rail commands.

**closed-loop execution**

Execution of at most the next validated primitive, followed by effect checking,
a fresh observation, and replanning rather than assuming a stale plan remains
valid.

**fail-closed**

Behavior that rejects, stops, or leaves execution disabled when required input,
freshness, identity, configuration, or artifact integrity is unknown.

## Data and Artifact Terms

**episode**

One recorded task/data-collection unit with frames, events, images, metadata,
and validation outcome.

**`data.jsonl`**

The framewise replay/debug stream written by the dataset recorder.

**`events.jsonl`**

The event-level training surface. It represents the observation before a
decision and the next symbolic event action.

**sidecar**

Metadata stored beside a model/checkpoint, such as schema, vectorizer,
calibration, metrics, or training configuration.

**runtime candidate**

A proposed immutable collection of a model and its required contract artifacts
before promotion.

**promotion manifest**

A checksum-bound manifest identifying the exact candidate approved for a
runtime stage such as shadow or active operation.

**execution authorization**

A separate checksum-bound record allowing a qualified runtime candidate to
enter the supervised task-execution path. It is not equivalent to a checkpoint
or promotion manifest.

**shadow mode**

Running model inference/comparison without allowing its output to drive the
active command path.

**canary**

A limited evaluation partition/run used to detect problems before a broader or
final evaluation.

**frozen split/final test**

An immutable dataset partition whose members and hashes must not be changed
after the evaluation protocol is fixed.

**evidence manifest**

A record connecting reported results to exact source/artifact hashes and output
files.
