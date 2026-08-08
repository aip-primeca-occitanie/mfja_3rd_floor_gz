# Multi-robot cold-start interface smoke protocol

Protocol fixed before execution on 7 August 2026.

## Scope

One headless cold start of `full_floor.launch.py` with `robots:=all`, the rail
runtime disabled, `ROS_DOMAIN_ID=150`, and Gazebo partition
`mfja_report_smoke_20260807`. No robot motion command is sent. This smoke test
checks launch composition and the observable command/feedback surfaces; it is
not a robot-motion, controller-performance, or reliability experiment.

## Source identity

- Git HEAD: `c44c501640e3dab35ab241384c99963e1b9d8745`
- Working-tree patch SHA-256:
  `d446c63192386b32b0d92e24aed6210b59bc53bfcb81c9af05a644ed421db0bd`
- Runtime source tree: 483 files, aggregate SHA-256
  `326ccd537a65c43bca5780610d4fb9a0d7bd95791b047f3a1e174fc481f889f9`

These identifiers must match the integrated-campaign environment record.

## Declared expected instances

`kuka1`, `staubli1`, `yaskawa_hc10_1`, `yaskawa_hc10dt_1`, `tiago1`, and
`tiago_base1`, as selected from `mfja_robot_control_config/config/robots.yaml`.

## Acceptance criteria

1. The launch process remains alive until the checks are complete.
2. The isolated ROS graph exposes `/clock` and the Gazebo world service
   surfaces for `mfja_3rd_floor`.
3. Every declared instance exposes its namespaced `joint_trajectory` command
   topic and `joint_states` feedback topic.
4. `tiago1` and `tiago_base1` additionally expose namespaced `cmd_vel` topics.
5. A fresh `joint_states` message can be read for every declared instance.
6. The active log contains no `ERROR`, `FATAL`, traceback, process-death,
   segmentation-fault, or core-dump marker before planned shutdown.
7. All commands, graph snapshots, sampled feedback, process log and final
   result are retained in this evidence directory with a checksum list.
