# MFJA Documentation Hub

This page is the entry point for all English operational, technical, research,
and maintenance documentation in this repository. Start with the short reading
path for your role, then use the categorized index when you need detail.

## Recommended Reading Paths

### New User

1. [Installation and Workspace Setup](INSTALLATION.md)
2. [Project README](../README.md)
3. [Quick Start and Feature Guide](QUICK_START_AND_FEATURE_GUIDE.md)
4. [Troubleshooting](TROUBLESHOOTING.md)

### Simulation Operator

1. [Quick Start and Feature Guide](QUICK_START_AND_FEATURE_GUIDE.md)
2. [Room 315 Rail Reference](ROOM315_RAIL_REFERENCE.md)
3. [Shuttle, Switch, Stopper, and Debug Reference](SHUTTLE_SWITCH_STOPPER_REFERENCE.md)
4. [Full Floor and Robot Reference](FULL_FLOOR_AND_ROBOTS.md)

### Maintainer or Developer

1. [System Architecture](SYSTEM_ARCHITECTURE.md)
2. [Configuration and Customization](CONFIGURATION.md)
3. [Maintenance Guide](MAINTENANCE.md)
4. [Troubleshooting](TROUBLESHOOTING.md)
5. [Third-Party Asset Attribution](../mfja_3rd_floor_description/THIRD_PARTY.md)

### VLA or Planning Researcher

1. [Room 315 VLA Research](ROOM315_VLA_RESEARCH.md)
2. [Room 315 VLA Operations](ROOM315_VLA_OPERATIONS.md)
3. [Visual-State Runtime Integration](room315_visual_runtime_integration.md)
4. [Task-Goal Understanding](ROOM315_TASK_GOAL_UNDERSTANDING.md)
5. [PDDL Planning](ROOM315_PDDL_PLANNING.md)
6. [Language-to-Motion Runtime](ROOM315_LANGUAGE_TO_MOTION_RUNTIME.md)
7. [Dataset Role and Provenance Registry](room315_dataset_role_registry.md)

## Document Status Labels

- **Operational:** intended to be followed on the current source tree.
- **Reference:** detailed current behavior or API information.
- **Research:** current concepts, experiment boundaries, or evaluation rules;
  not necessarily a turnkey operator procedure.
- **Historical:** a dated audit, earlier dataset generation, or progress record.
  It is retained for traceability and must not override current operational
  instructions.

When a historical document contains a command that differs from an operational
document, use the operational document and verify launch arguments with
`ros2 launch ... --show-args`.

## Core User and Operator Documentation

| Status | Document | Purpose |
| --- | --- | --- |
| Operational | [Installation and Workspace Setup](INSTALLATION.md) | Host requirements, dependency installation, build, sourcing, Nix, verification, and updates |
| Operational | [Quick Start and Feature Guide](QUICK_START_AND_FEATURE_GUIDE.md) | Room 315/full-floor startup and step-by-step rail operations |
| Reference | [Room 315 Rail Reference](ROOM315_RAIL_REFERENCE.md) | Rail backend, launch details, quick commands, and typed interfaces |
| Reference | [Shuttle, Switch, Stopper, and Debug Reference](SHUTTLE_SWITCH_STOPPER_REFERENCE.md) | Multi-shuttle operation, calibration, parameters, states, and rail troubleshooting |
| Reference | [Room 315 Rail Devices and Tests](ROOM315_RAIL_DEVICES_AND_TESTS.md) | Device YAML, markers, collision checks, message types, and launch names |
| Operational | [Full Floor and Robot Reference](FULL_FLOOR_AND_ROBOTS.md) | World services, robot selection, joint commands, grippers, and TIAGo motion |
| Operational | [Troubleshooting](TROUBLESHOOTING.md) | Symptom-to-diagnosis-to-remedy procedures for installation and runtime faults |
| Reference | [Glossary](GLOSSARY.md) | Definitions of ROS, Gazebo, rail, VLA, planning, and artifact terms |

## Architecture, Configuration, and Maintenance

| Status | Document | Purpose |
| --- | --- | --- |
| Reference | [System Architecture](SYSTEM_ARCHITECTURE.md) | Package boundaries, launch sequence, runtime data flow, ROS API, safety boundaries, and output locations |
| Operational | [Configuration and Customization](CONFIGURATION.md) | Sources of truth and safe recipes for common changes |
| Operational | [Maintenance Guide](MAINTENANCE.md) | Build/test matrix, extension recipes, repository hygiene, release artifacts, and handover checklist |
| Reference | [Kinematic Rail Configuration](../mfja_robot_control_config/config/room_315_kinematics/README.md) | Detailed rail topology, calibration, device, marker, and sensor implementation reference |
| Reference | [Segment CSV Migration](../mfja_robot_control_config/config/room_315_kinematics/SEGMENT_CSV_MIGRATION.md) | Rules for the explicit segment-to-CSV schema |
| Reference | [Visual-State Scenario Configuration](../mfja_robot_control_config/config/room_315_visual_state/README.md) | Scenario generation and visual-state dataset configuration |
| Reference | [Third-Party Asset Attribution](../mfja_3rd_floor_description/THIRD_PARTY.md) | Imported robot/CAD provenance and license constraints |

## Current VLA, Planning, and Dataset Documentation

| Status | Document | Purpose |
| --- | --- | --- |
| Research | [Room 315 VLA Research](ROOM315_VLA_RESEARCH.md) | Model-input boundary, visual facts, case matrix, and evaluation intent |
| Operational | [Room 315 VLA Operations](ROOM315_VLA_OPERATIONS.md) | Supervisor, task-goal, dataset, benchmark, and primitive-command workflows |
| Operational | [Visual-State Runtime Integration](room315_visual_runtime_integration.md) | Current V4 runtime contract, artifact validation, launch, diagnostics, and failure behavior |
| Operational | [Task-Goal Understanding](ROOM315_TASK_GOAL_UNDERSTANDING.md) | English parsing, confirmation, local model, validation, and public API |
| Reference | [Room 315 PDDL Planning](ROOM315_PDDL_PLANNING.md) | Atomic goal contract, topology planning, case generation, and batch runs |
| Operational | [Language-to-Motion Runtime](ROOM315_LANGUAGE_TO_MOTION_RUNTIME.md) | End-to-end confirmed English command to supervised simulation motion |
| Operational | [Seeded Benchmark Runbook](ROOM315_BENCHMARK_RUNBOOK.md) | Case generation, method comparison, claims, and limitations |
| Reference | [Dataset Role and Provenance Registry](room315_dataset_role_registry.md) | Dataset roles, split boundaries, fingerprints, provenance, and reproduction |
| Research | [Neuro-Symbolic Gap Analysis](ROOM315_NEURO_SYMBOLIC_GAP_ANALYSIS.md) | Current architecture decisions and acceptance criteria |

## Historical and Audit Records

These files explain how earlier artifacts or design decisions were produced.
They are valuable for provenance, but they are not the first place to obtain a
current command line.

| Status | Document | Purpose |
| --- | --- | --- |
| Historical | [Visual Runtime Integration Audit](room315_visual_runtime_integration_audit.md) | Pre-integration audit and decision record |
| Historical | [Hard-Case Visual Dataset V3 Audit](room315_hard_case_visual_dataset_v3_audit.md) | Initial V3 repository/data audit |
| Historical | [Hard-Case Visual Dataset V3](room315_hard_case_visual_dataset_v3.md) | Earlier V3 generation and split procedure |
| Historical | [Hard-Case Visual Dataset V3R1](room315_hard_case_visual_dataset_v3r1.md) | Earlier V3R1 procedure and immutable roots |
| Historical | [VLA Progress Report, Week 1](vla_progress_report_week_1.md) | Short dated progress summary |
| Historical | [VLA Progress Report, Week 2](vla_progress_report_week_2.md) | Short dated progress summary |

The tracked legacy page
[`ROOM315_TASK_GOAL_AR.html`](ROOM315_TASK_GOAL_AR.html) is now an English
compatibility redirect to the maintained task-goal guide. Its filename is kept
only so existing bookmarks do not break.

## Datasets, Models, and Evidence

Large datasets and learned checkpoints are intentionally not stored as ordinary
Git source files.

- The V4 reproduction companion is published under the GitHub release tag
  [`v4-seed31520260811-dataset-v1`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/tag/v4-seed31520260811-dataset-v1).
- Its small verification/control record is
  [the V4 dataset release package](../report/evidence/room315_visual_v4_dataset_release_v1/README.md).
- The project-level release relationship is recorded in
  [ROOM315_VISUAL_V4_DATASET_RELEASE.md](../report/evidence/ROOM315_VISUAL_V4_DATASET_RELEASE.md).
- English report sources and their build procedure are indexed by
  [report/README.md](../report/README.md).
- Runtime candidates, checkpoints, promotion manifests, and authorization files
  must live outside the repository and retain their recorded hashes.
- Default runtime YAML paths identify the machine used for the qualified run;
  they are not portable installation paths. Create or select a host-local
  configuration that references a complete verified artifact bundle.

Do not place an extracted dataset containing `package.xml` inside a colcon
workspace source tree. Colcon may discover the frozen package copy and report a
duplicate package name. Keep datasets outside the workspace or isolate their
source snapshots with `COLCON_IGNORE`.

## Standalone Runbooks

- [Rail runbook](../runbook.html)
- [VLA runbook](../runbookvla.html)

The Markdown operational documents above are easier to review alongside code.
If a standalone HTML runbook differs from current launch arguments, prefer the
installed launch interface and the operational Markdown guide.

## Keeping Documentation Correct

Every behavior change should update the smallest authoritative document plus
this index when a document is added, renamed, or reclassified. At minimum:

1. Verify launch options with `ros2 launch <package> <file> --show-args`.
2. Verify interfaces with `ros2 interface show <type>`.
3. Verify executable availability with `ros2 pkg executables <package>` after
   rebuilding and sourcing.
4. Check relative Markdown links and run `git diff --check`.
5. Keep new operational documentation entirely in English.
