# System Architecture

This document explains how the repository is divided, what starts at runtime,
which component owns each fact, and where the simulation and research safety
boundaries are located.

## Scope and Boundaries

The repository models the MFJA third floor and provides a detailed Room 315
rail-cell runtime. Its maintained base platform is ROS 2 Jazzy with Gazebo
Harmonic on Ubuntu 24.04.

The following boundaries are intentional:

- Rail propulsion is kinematic. The controller samples a calibrated directed
  path and calls Gazebo pose services. It does not simulate motor, wheel, or
  rail-contact dynamics.
- Shuttle-to-shuttle protection is a software center-distance/headway rule, not
  a complete rigid-body or certified block-control system.
- The industrial robot assets are intentionally collision-free, and Room 315
  shuttle collision masks disable physics contact. Visual overlap is therefore
  not proof of a protected physical clearance.
- Industrial gripper jaws are animated and range-limited, but they do not
  attach or carry physics payloads.
- Binary rail sensors are simulated occupancy events. Their messages also carry
  simulator-derived segment and arc-length metadata for debugging/evaluation;
  these values are not extra physical sensors.
- The visual-state model emits visual facts only. The optional intent model
  emits a constrained draft for deterministic validation and user
  confirmation. Neither directly emits PDDL actions or rail commands.
- Task execution is disabled by default and requires explicit, hash-verified
  authorization. The code is research software, not a physical-machine safety
  system.
- Checked-in V4 runtime configuration contains paths from the qualification
  host. A clean clone can run Gazebo, robots, rails, cameras, the primitive
  supervisor, and dataset tools, but active learned inference/execution needs
  the separate qualified artifacts.

## Package Dependency Graph

The repository root is a colcon meta-repository. It does not have its own
`package.xml`.

```text
mfja_3rd_floor_bringup
  -> mfja_robot_control_config
       -> mfja_3rd_floor_description
       -> mfja_rail_interfaces
  -> mfja_3rd_floor_description

mfja_3rd_floor_description   (Gazebo/URDF assets and C++ plugin)
mfja_rail_interfaces         (ROS interface definitions)
```

| Package | Owns | Important contents |
| --- | --- | --- |
| `mfja_3rd_floor_bringup` | Stable user-facing launch entry points | Room-only, full-floor, isolated-robot wrappers and shared floor launch policy |
| `mfja_3rd_floor_description` | Simulation description | Worlds, SDF models, meshes, URDF files, third-party attribution, collision/model tests, symmetric gripper plugin |
| `mfja_rail_interfaces` | Public typed rail/visual contracts | Eleven messages plus `AddShuttle.srv`; generated language bindings are build output |
| `mfja_robot_control_config` | Runtime and research logic | Robot spawning, Gazebo bridges, rail nodes, VLA/planning nodes, Python tools, YAML/JSON/PDDL configuration, and the main test suite |

The higher-level packages should depend on lower-level contracts and assets;
lower-level packages should not import bringup behavior.

## High-Level Launch Sequence

`room_315_only.launch.py` and `full_floor.launch.py` delegate to
`mfja_3rd_floor_bringup/launch/room_315_floor_common.py` with different
profiles.

```text
time 0 s
  acquire exclusive Room 315 host lock
  clear optional obstacle pose cache
  include multi_robot_sim.launch.py
    -> set Gazebo resource paths and transport partition
    -> start Gazebo server and optional GUI
    -> start visual switch controller

time 3 s
  start the /clock and world create/remove/set_pose bridges
  spawn selected robots and start their bridges/state publishers

time 4 s
  include room_315_dual_kinematic_shuttles.launch.py
    -> right rail node, if enabled
    -> left rail node, if enabled

time 5 s
  optionally include room_315_vla_supervisor.launch.py
    -> overhead-camera bridge
    -> primitive supervisor
    -> optional dataset recorder
```

Allow at least five seconds before treating a missing late-start process or
topic as a fault.

### Launch Profiles

| Setting | Room 315 only | Full floor |
| --- | --- | --- |
| World file/entity name | `room_315_only` | `mfja_3rd_floor` |
| Robot YAML | `robots_room_315_only.yaml` | `robots.yaml` |
| Default simulation state | Running | Paused |
| Default GUI config | `room315_runtime_safe.gui.config` | `mfja_light.gui.config` |
| Initial shuttle count | Zero on each rail | Zero on each rail |
| Rails | Enabled | Enabled |
| VLA supervisor | Disabled | Disabled |

If the `robots` argument is empty, every YAML entry whose `enabled` field is
true is spawned. Pass `robots:=none` for a lightweight rail-only run.

The high-level launch uses an advisory file lock at
`/tmp/mfja_room315_floor_runtime.lock`. Only one Room 315/full-floor runtime may
own the fixed ROS rail namespaces at a time. The file may remain after a clean
shutdown, but the operating-system lock is released when its owner exits.

## Gazebo Worlds and Resource Resolution

The description package contains three worlds:

| File | Internal world name | Use |
| --- | --- | --- |
| `worlds/room_315_only.world` | `room_315_only` | Focused Room 315 operation and data collection |
| `worlds/mfja_3rd_floor.world` | `mfja_3rd_floor` | Complete-floor scene |
| `worlds/isolated_industrial_robot.world` | `isolated_industrial_robot` | Minimal ground-plane world for one industrial robot/table |

The launch adds `mfja_3rd_floor_description/models` to both
`GZ_SIM_MODEL_PATH` and `GZ_SIM_RESOURCE_PATH` while preserving existing
values. SDF references such as `model://room315_shuttle_R1` are resolved from
that model directory.

For a custom world, keep the filename stem and the XML `<world name="...">`
identical. The Gazebo bridge derives services from the internal XML name, while
the high-level shuttle launch is configured from the requested world filename.
A mismatch produces different service names and prevents shuttle pose/spawn
operations.

Both main world sources include removable VLA obstacle entities. Unless
`enable_room315_vla_obstacles:=true` is passed, the launcher writes an
obstacle-free temporary world under `/tmp` and runs that copy. The source world
is not modified.

## Models, URDF, and Plugins

Each reusable Gazebo model normally contains:

```text
models/<model_name>/
  model.config
  model.sdf
  meshes/...
```

Industrial robots also have a matching `urdf/<model_name>.urdf`. Gazebo SDF
provides simulated joints/controllers and visuals; URDF is supplied to
`robot_state_publisher` for ROS frames.

The description package builds
`mfja-symmetric-gripper-controller-system`, a Gazebo system plugin used by the
industrial models. During launch, gripper ranges from
`gripper_command_defaults.yaml` are applied to a temporary SDF and an in-memory
`robot_description` derived from the source URDF. Edit the source
config/templates, never the generated `/tmp` files.

Room 315 loads eight identity-specific shuttle entities (`R1`-`R4` and
`L1`-`L4`) below the scene until selected. The kinematic rail node reveals,
spawns, moves, or removes the active entities.

## Room 315 Physical/Logical Layout

The canonical station association is:

| Rail/slots | Robot station |
| --- | --- |
| Right slots 1-2 | Yaskawa HC10 |
| Right slots 3-4 | Staeubli TX2 |
| Left slots 1-2 | Yaskawa HC10DT |
| Left slots 3-4 | KUKA KR6 |

The right overhead camera is centered over the right rail near the HC10 and
Staeubli side; the left overhead camera covers the HC10DT and KUKA side.

## Rail Runtime

One `room_315_kinematic_shuttle_node.py` instance runs per enabled rail side.
Each node loads:

1. `rail_network_<side>.yaml` for directed topology, nodes, switches, routing,
   and explicit segment-to-CSV references.
2. `raw_segments/*.csv` for measured path samples.
3. `rail_devices_<side>.yaml` for slots, binary sensors, stoppers, radii, and
   default stopper states.
4. calibration and public-name defaults from
   `room_315_rail_defaults.py`.
5. identity conventions and Gazebo model paths used by the multi-shuttle
   manager.

The normal path backend is cubic Hermite interpolation with arc-length
parameterization. The `polyline` backend exists for direct CSV comparison.
Every segment is forward-only. At a controlled node, the delayed actual switch
state selects the valid successor. A missing successor latches `FALLING`.

The left rail reuses calibrated geometry but has an explicit internal-to-public
segment-name mapping. Public and internal names must not be interchanged even
when the current mapping happens to be symmetric for some values.

### Rail State Model

Common shuttle modes include:

- `DISABLED`: drive explicitly off.
- `MOVING`: advancing along the selected route.
- `WAITING`: enabled but held by a stopper, headway/collision rule, or another
  guarded condition.
- `FALLING`: no valid route successor was available and the fault is latched.

`ShuttleState.speed` is the configured travel-speed setting retained for a
future `ON`; it is not instantaneous velocity. A successful `OFF` check must
observe a newer state with mode `DISABLED` rather than infer stopping from the
speed field.

## Canonical ROS API

Replace `<side>` with `right` or `left`.

| Name | Type | Direction/owner | Purpose |
| --- | --- | --- | --- |
| `/room_315/rails/<side>/shuttles/command` | `mfja_rail_interfaces/msg/ShuttleCommand` | Input to rail node | `ON`, `OFF`, `RESET`, `REMOVE`, optional speed and target slot |
| `/room_315/rails/<side>/shuttles/state` | `mfja_rail_interfaces/msg/ShuttleState` | Rail node output | Mode, segment, simulated coordinates, configured speed, reached target |
| `/room_315/rails/<side>/shuttles/add` | `mfja_rail_interfaces/srv/AddShuttle` | Rail node service | Add a named/automatic shuttle at a start slot |
| `/room_315/rails/<side>/switches/command` | `mfja_rail_interfaces/msg/SwitchCommand` | Input to rail node | Request `E`/`EXTERIOR` or `I`/`INTERIOR` |
| `/room_315/rails/<side>/switches/state` | `mfja_rail_interfaces/msg/SwitchState` | Rail node output | Actual state after motion delay |
| `/room_315/rails/<side>/stoppers/command` | `mfja_rail_interfaces/msg/StopperCommand` | Input to rail node | Request `0` open/pass or `1` closed/stop |
| `/room_315/rails/<side>/stoppers/state` | `mfja_rail_interfaces/msg/StopperState` | Rail node output | Actual state after motion delay |
| `/room_315/rails/<side>/sensors/feedback` | `mfja_rail_interfaces/msg/SensorFeedback` | Rail node output | Binary occupancy readings and simulator debug metadata |
| `/room_315/rails/<side>/shuttles/payload_command` | `std_msgs/msg/String` JSON | Input to rail node | Privileged simulation payload toggle |
| `/room_315/rails/<side>/shuttles/payload_state` | `std_msgs/msg/String` JSON | Rail node output | Privileged payload state for debug/dataset metadata |
| `/room_315/rails/<side>/shuttles/pose_offset_command` | `std_msgs/msg/String` | Input to rail node | Runtime-only pose calibration offset |
| `/room_315/rails/<side>/shuttles/pose_cmd` | `geometry_msgs/msg/PoseStamped` | Rail node output | First shuttle's commanded pose |
| `/room_315/rails/<side>/shuttles/<safe_entity>/pose_cmd` | `geometry_msgs/msg/PoseStamped` | Rail node output | Additional shuttle commanded poses |
| `/world/<world>/set_pose` | `ros_gz_interfaces/srv/SetEntityPose` | Gazebo service | Kinematic entity pose update |
| `/world/<world>/create` | `ros_gz_interfaces/srv/SpawnEntity` | Gazebo service | Runtime entity/marker creation |
| `/world/<world>/remove` | `ros_gz_interfaces/srv/DeleteEntity` | Gazebo service | Runtime entity/marker removal |

The typed public switch topics are authoritative. `/mfja/conveyor/switch_cmd`
and `/mfja/conveyor/switch_states` belong to the visual switch controller; the
rail node mirrors accepted delayed state to that layer.

### Interface Field Notes

- `ShuttleCommand.target_slot` requests a closed-loop stop at an authoritative
  configured slot. Empty preserves continuous motion.
- `ShuttleState.reached_target_slot` is populated only after the low-level
  controller stops at that explicit setpoint.
- `SensorReading.active` is the binary occupancy result. `segment`, `s`, and
  `s_ratio` are privileged simulator metadata associated with that event, not a
  learned or physical continuous sensor.
- `VisualStateObservation` contains acceptance/readiness flags, hashes,
  validation reasons, timings, counters, and per-shuttle visual facts. It is a
  separate perception contract from `ShuttleState`.

Use `ros2 interface show <type>` for the exact installed definition.

## Robot Runtime

`multi_robot_sim.launch.py` reads the selected robot YAML, validates unique
names, resolves aliases, and creates a bridge configuration under `/tmp` for
each instance.

Every robot exposes:

- `/<name>/joint_trajectory` from ROS to Gazebo.
- `/<name>/joint_trajectory_progress` from Gazebo to ROS.
- `/<name>/joint_states` from Gazebo to ROS.
- `/<name>/tf` and `/<name>/tf_static` from its namespaced
  `robot_state_publisher`.

Industrial robots additionally expose
`/<name>/gripper/position_command`. TIAGo variants additionally expose
`/<name>/cmd_vel` and `/<name>/odom`; their Gazebo DiffDrive bridge also
publishes transforms on `/<name>/tf`.

Industrial `robot_state_publisher` frame IDs are instance-prefixed. Mobile
frame IDs are currently unprefixed even though their topics are namespaced, so
two TIAGo variants still collide on frame IDs such as `odom` and
`base_footprint`. Launch at most one mobile instance in a ROS graph.

The active robot list is configuration, not world composition: robot entities
are spawned after Gazebo starts. Static room, rail, furniture, camera, and
preloaded shuttle entities remain in the world SDF.

## VLA and Planning Architecture

The advanced runtime separates perception, planning, supervision, and
actuation:

```text
overhead RGB images + deterministic shuttle presence
  -> V4 visual-state inference
  -> validation and state fusion
  -> accepted VisualStateObservation

English input
  -> deterministic extraction + optional local semantic-model draft
  -> TaskGoal validation/clarification and user confirmation
  -> grounded PDDL problem using accepted facts
  -> PlanSys2/POPF plan
  -> translate and validate one primitive
  -> VLA supervisor safety gate
  -> typed rail command
  -> deterministic effect check + fresh visual observation
  -> replan or finish
```

### Topic Boundary

| Topic | Type | Role |
| --- | --- | --- |
| `/room_315/vla/right_rail_rgbd/image` | `sensor_msgs/msg/Image` | Right visual input |
| `/room_315/vla/left_rail_rgbd/image` | `sensor_msgs/msg/Image` | Left visual input |
| `/room_315/vla/{right,left}_rail_rgbd/camera_info` | `sensor_msgs/msg/CameraInfo` | Per-side camera calibration |
| `/room_315/vla/{right,left}_rail_rgbd/depth_image` | `sensor_msgs/msg/Image` | Per-side depth image |
| `/room_315/vla/{right,left}_rail_rgbd/points` | `sensor_msgs/msg/PointCloud2` | Per-side RGB-D point cloud |
| `/room_315/visual_state/raw` | `mfja_rail_interfaces/msg/VisualStateObservation` | Unaccepted constructed observation |
| `/room_315/visual_state/raw_model_prediction` | `std_msgs/msg/String` | Auditable raw learned output |
| `/room_315/visual_state/validation` | `mfja_rail_interfaces/msg/VisualStateObservation` | Validation result |
| `/room_315/visual_state/observed_state` | `mfja_rail_interfaces/msg/VisualStateObservation` | Accepted state for planning |
| `/room_315/task_goal` | `std_msgs/msg/String` JSON | Confirmed task request |
| `/room_315/task_goal/status` | `std_msgs/msg/String` JSON | Task lifecycle/result |
| `/room_315/vla/command` | `std_msgs/msg/String` JSON | Validated primitive input to supervisor |
| `/room_315/vla/status` | `std_msgs/msg/String` JSON | Supervisor state and result |
| `/room_315/vla/emergency_stop` | `std_msgs/msg/Bool` | Virtual emergency-stop input |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Readiness, validation, and runtime health |

The visual inference node has no direct rail-command publisher. The task
execution gateway publishes only to the supervisor after authorization,
planning, translation, and validation.

### Information Ownership

- At the learned-model input/fusion boundary, controller shuttle state
  contributes deterministic presence only. Separately, the supervisor retains
  trusted controller mode, segment, arc position, pose, and speed for FALLING,
  movement/localization, headway, and command-safety checks.
- The learned model owns visual block/location, bounding box, and payload
  classification within the accepted observation.
- Device state comes from deterministic switch/stopper controllers.
- Identity-bearing DZI occupancy plus an explicit controller `DISABLED` state
  verifies final arrival/stop in the current simulation runtime.
- Controller/Gazebo coordinates may veto unsafe execution where explicitly
  documented, but they do not silently replace learned localization.
- Exact poses, oracle labels, and validation metadata belong to privileged
  evaluation or safety surfaces, not `model_input`.

## Files Written at Runtime

| Location | Contents | Persistence |
| --- | --- | --- |
| `~/.ros/log/` | ROS process logs | Local, disposable after diagnosis |
| `~/.ros/room315_visual_state_datasets/<run>/` | Recorded events, frames, images, episode metadata | User data; keep outside Git |
| `~/.ros/room315_vla_obstacles.json` | Optional obstacle pose cache | Cleared by high-level launch by default |
| `/tmp/*_bridge.yaml` | Generated robot bridge mappings | Temporary; never edit |
| `/tmp/*_mobile_model.sdf` | Instance-specific mobile model | Temporary; never edit |
| `/tmp/mfja_*_gripper_range_*.sdf` | Configured gripper SDF | Temporary; never edit |
| `/tmp/*_without_room315_vla_obstacles_*.world` | Obstacle-filtered world | Temporary; never edit |
| User-selected external directories | Datasets, checkpoints, promotion/authorization bundles, benchmark results | Preserve according to experiment policy |

Repository-local `build/`, `install/`, and `log/` directories are also derived
colcon output and must not be edited by hand.

## Sources of Truth

Use this precedence when investigating a discrepancy:

1. `.msg`/`.srv` files for interface fields.
2. current launch code and `--show-args` for launch inputs/defaults.
3. versioned YAML/JSON/PDDL files for configuration contracts.
4. node implementation for runtime behavior.
5. operational documentation for supported procedures.
6. research/audit/progress records for historical context.

See [Configuration and Customization](CONFIGURATION.md) for the exact file and
test set associated with each common modification.
