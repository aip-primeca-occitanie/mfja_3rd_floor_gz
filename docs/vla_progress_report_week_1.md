# VLA Progress Report - Week 1

> **Historical progress record:** descriptions below reflect the dated period,
> not necessarily the current runtime. Use the [Documentation Hub](README.md)
> for maintained operational instructions.

Period: June 18, 2026 - June 25, 2026

## Main Progress

- This week focused on building the Room 315 VLA pipeline from planning to execution and data collection.
- Expanded shuttle motion planning inside Room 315 to cover both the left and right rails and loaded payload cases.
- Connected the workflow between PDDL, the scenario generator, primitive commands, the safety supervisor, and the training data recorder.
- Improved `room_315_pddl_scenario_generator.py` so training cases can be converted into executable symbolic command sequences.
- Updated the PDDL domain and contract-level primitive commands to support payload actions, shuttle selection, and movement targets.
- Added initial payload training scenarios covering loaded shuttles, target slots, and start-slot configurations.
- Added batch case execution and review tools to run cases and inspect results in an organized way.
- Improved `room_315_vla_supervisor.py` so symbolic commands are validated before real rail commands are published.
- Expanded the dataset recorder and event extractor to produce `training_events.jsonl` suitable for VLA training.
- Added multi-robot / multi-shuttle launch support for more realistic Room 315 experiments.
- Improved shuttle and payload visuals in Gazebo so the training images are clearer.
