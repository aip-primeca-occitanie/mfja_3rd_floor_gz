# Room 315 Naming Migration

The Room 315 implementation is a modular neuro-symbolic closed loop, not a
single learned vision-language-action policy. It combines four separately
owned components:

1. a local language model that proposes constrained task-goal fields;
2. a visual-state model that estimates structured scene facts from paired RGB
   images;
3. PlanSys2, which creates the symbolic plan; and
4. a deterministic rail-safety supervisor, which validates primitives and is
   the only component allowed to publish typed rail commands.

The names below replace the earlier project-wide `VLA` shorthand with names
that identify the actual owner and authority boundary.

## Documentation

| Previous path | Canonical path |
| --- | --- |
| `docs/ROOM315_VLA_RESEARCH.md` | `docs/ROOM315_VISUAL_STATE_RESEARCH.md` |
| `docs/ROOM315_VLA_OPERATIONS.md` | `docs/ROOM315_NEURO_SYMBOLIC_CLOSED_LOOP_OPERATIONS.md` |
| `docs/vla_progress_report_week_1.md` | `docs/neuro_symbolic_progress_report_week_1.md` |
| `docs/vla_progress_report_week_2.md` | `docs/neuro_symbolic_progress_report_week_2.md` |
| `docs/vla overview/room_315_vla_architecture.png` | `docs/neuro_symbolic_overview/room_315_neuro_symbolic_architecture.svg` |
| `runbookvla.html` | `room315_closed_loop_runbook.html` |
| `report/chapters/09_vla_evolution.tex` | `report/chapters/09_ai_inputs_development.tex` |

## Scripts and Launch File

| Previous name | Canonical name |
| --- | --- |
| `room_315_vla_benchmark_suite.py` | `room_315_visual_planning_benchmark_suite.py` |
| `room_315_vla_dataset_recorder.py` | `room_315_visual_state_dataset_recorder.py` |
| `room_315_vla_event_extractor.py` | `room_315_visual_training_event_extractor.py` |
| `room_315_vla_obstacle_tool.py` | `room_315_visual_obstacle_tool.py` |
| `room_315_vla_shuttle_identity_tracker.py` | `room_315_privileged_shuttle_identity_tracker.py` |
| `room_315_vla_split_dataset.py` | `room_315_visual_state_split_dataset.py` |
| `room_315_vla_supervisor.py` | `room_315_rail_safety_supervisor.py` |
| `room_315_vla_to_lerobot.py` | `room_315_visual_state_to_lerobot.py` |
| `room_315_vla_train_local.py` | `room_315_visual_state_train_local.py` |
| `room_315_vla_train_v4.py` | `room_315_visual_state_train_v4.py` |
| `room_315_vla_supervisor.launch.py` | `room_315_perception_and_safety.launch.py` |

## Configuration Layout

The former `config/room_315_vla/` directory was split by responsibility:

| Content | Canonical directory |
| --- | --- |
| Payload scenarios and case matrix | `config/room_315_payload_cases/` |
| PDDL domains | `config/room_315_planning/` |
| Identity and marker mapping | `config/room_315_shuttle_identity/` |
| Task execution and `rail_safety_supervisor.yaml` | `config/room_315_task_execution/` |
| English task-goal configuration and benchmark | `config/room_315_task_goal/` |
| Visual training, evaluation, calibration, and runtime | `config/room_315_visual_state/` |

The dataset tools now read `ROOM315_VISUAL_DATASET_ROOT` and
`ROOM315_VISUAL_SPLITS_DIR` instead of `ROOM315_VLA_DATASET_ROOT` and
`ROOM315_VLA_SPLITS_DIR`.

## Launch Arguments

| Previous argument | Canonical argument |
| --- | --- |
| `enable_room315_vla` | `enable_room315_rail_safety_supervisor` |
| `enable_room315_vla_camera_bridge` | `enable_room315_rgbd_camera_bridge` |
| `enable_room315_vla_dataset_recorder` | `enable_room315_visual_state_dataset_recorder` |
| `enable_room315_vla_obstacles` | `enable_room315_visual_obstacles` |
| `room315_clear_vla_obstacle_pose_cache` | `room315_clear_visual_obstacle_pose_cache` |
| `room315_vla_obstacle_pose_file` | `room315_visual_obstacle_pose_file` |
| `room315_vla_dataset_dir` | `room315_visual_dataset_dir` |
| `room315_vla_dataset_sample_period_s` | `room315_visual_dataset_sample_period_s` |

The former umbrella argument did not start the language model, visual-state
inference, PlanSys2, or the task gateway. The canonical arguments expose the
rail-safety supervisor, camera bridge, and recorder separately.

## Compatibility Impact

This is an intentional breaking rename, not an alias layer. Rebuild the
workspace and source the new overlay before testing it. Existing launch files,
shell scripts, parameter overrides, ROS remaps, dashboards, bag-processing
tools, and external automation must adopt the canonical names above. A process
using an old launch argument will fail argument validation; an old topic name
will not connect to the renamed publisher or subscriber; and an old Gazebo
model URI will no longer resolve. Update host-local environment files that set
the dataset-root variables as well.

Do not rewrite archived bags, manifests, or records merely to make their names
look current. Treat those names as part of the recorded interface version and
use an explicit replay/remapping adapter when historical data must be consumed
by the current graph.

## ROS Topics

| Previous topic | Canonical topic |
| --- | --- |
| `/room_315/vla/<side>_rail_rgbd/...` | `/room_315/perception/<side>_rail_rgbd/...` |
| `/room_315/vla/command` | `/room_315/rail_safety/primitive_command` |
| `/room_315/vla/status` | `/room_315/rail_safety/status` |
| `/room_315/vla/emergency_stop` | `/room_315/rail_safety/emergency_stop` |
| `/room_315/vla/user_goal` | `/room_315/visual_dataset/goal_text` |
| `/room_315/vla/episode_control` | `/room_315/visual_dataset/episode_control` |
| `/room_315/vla/dataset_status` | `/room_315/visual_dataset/status` |

The task gateway continues to use `/room_315/task_goal` and
`/room_315/task_goal/status`; those are distinct from the recorder's optional
goal-text metadata topic.

## Gazebo Models

| Previous name family | Canonical name family |
| --- | --- |
| `room315_vla_overhead_devices` | `room315_visual_observation_rig` |
| `room315_vla_payload_*` | `room315_payload_*` |
| `room315_vla_removable_obstacle_marker` | `room315_visual_obstacle_marker` |

## Historical Evidence and External Model Names

Files under `report/evidence/**` intentionally retain the names that were
present when each run was captured. Topic names, source paths, manifests,
checksums, and provenance records in that directory are historical evidence;
rewriting them would invalidate hashes or make a recorded run no longer
reproducible. Current documentation may link to those records, but must not
silently rewrite their contents.

Names of real external model families and publications, including TinyVLA and
RT-2, are also unchanged. In those contexts, VLA has its standard
Vision-Language-Action meaning. It is removed only where it incorrectly labels
a Room 315 component or the modular Room 315 implementation as a joint learned
action policy.
