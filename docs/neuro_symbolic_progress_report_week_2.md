# Neuro-Symbolic Progress Report - Week 2

> **Historical progress record:** descriptions below reflect the dated period,
> not necessarily the current runtime. Use the [Documentation Hub](README.md)
> for maintained operational instructions.

Period: June 26, 2026 - July 2, 2026

## Main Progress

- Updated the PDDL payload training flow and kept the model input boundary clear: task language, camera images, previous command, and observable state only.
- Removed retired direct-action goals and unused benchmark/teleop tools so the project stays focused on the modular visual-state, planning, and supervised-execution workflow.
- Added route-topology blocker metadata to describe blockage locations and clearance routes in the scenarios.
- Adopted `payload_training_cases_expanded_160_speed_sweep.yaml` as the main source of truth for training cases.
- Stabilized 160 successful speed-sweep cases covering both rails, loaded shuttles, no-blocker cases, and blocker clearance to a stopper or the interior loop.
- Added a dataset splitting tool for train/val/test while keeping speed variants inside the same case family.
- Added a Room 315 to LeRobot converter for the `visual_state` dataset mode with physically separate oracle labels.
- Added a local PyTorch visual-state training helper that reports label metrics, target stats, checkpoints, and model-boundary metadata.
