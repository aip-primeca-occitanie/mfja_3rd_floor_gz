# Pick-and-place code map

The public workflow has three entry points:

| File | Purpose |
|---|---|
| `config/room315_pick_place.yaml` | Configure the scene, planning limits, and execution routes. |
| `launch/room_315_staubli_pick_place_sim.launch.py` | Start Gazebo, the Staubli, and the box. |
| `launch/room_315_staubli_hardware.launch.py` | Start the direct VAL3 driver and robot description. |
| `scripts/room315_pick_place.sh` | Plan or execute one table pick-and-place. |

The planner path is:

```text
room315_pick_place.py
  -> room315_problem.build_problem
  -> room315_planning.plan_manipulation
  -> room315_planning.build_execution_plan
  -> room315_execution.execute_plan  # selected by --execute
```

`room315_config.py` loads the YAML values used by the planner.
`room315_problem.py` uses that configuration to load the Staubli, gripper,
payload, table, and Room 315 fixture collision meshes. The default source and
destination are 10 cm to either side of the table centre.

`room315_planning.py` creates the approach, grasp, transfer, release, and
retreat paths. `room315_execution.py` preserves those boundaries for the
gripper commands and arm trajectory.

`room315_execution_profiles.py` applies either the Gazebo topics or the direct
VAL3 action and IO service defined in YAML. The HPP problem and path-generation
code are shared.

Planning validates the trajectory locally. `--execute` selects execution, and
the hardware profile also requires a measured six-joint `--q-start`. `--viser`
loads the planned segments into the browser path player.
