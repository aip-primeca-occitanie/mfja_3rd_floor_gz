# Room 315 Neuro-Symbolic Gap Analysis

Audit date: 2026-07-11
Branch: `ali/neuro-symbolic-closed-loop`
Base commit: `b111e3d`

Target contract:

```text
high-level goal -> ObservedState -> PDDL/PlanSys2 -> one atomic action -> safety -> execute -> re-observe/replan
```

## Summary

The current code has strong pieces for event-level data generation, action-vector safety validation, dataset splitting, LeRobot conversion, local training, and offline SmolVLA evaluation. It is not yet a closed-loop neuro-symbolic controller.

The largest gaps are:

- `generate_scenario` still bypasses PlanSys2 for slot-selection and blocker-clearance cases.
- `room_315_real_vla_agent.py` asks the model for a direct primitive/action vector and publishes it to the supervisor; it does not route the high-level goal through ObservedState -> PlanSys2 -> one-step execution -> replan.
- The PDDL domain remains coarse and does not model switch/stopper values, rail blocks, headway, payload/load state transitions, or image-derived perception.
- Compact32 LeRobot conversion leaks non-`model_input` metadata into `observation.state` by deriving task side, target slot, and payload presence from row-level fields such as `pddl_problem`, `pddl_goal`, and `payload_present`.
- Missing image refs/files silently become blank images or zero tensors in conversion/training, which can hide recorder/export failures.
- Bbox identity tracking exists only as privileged metadata; there is no bbox-to-rail pose/load perception feeding an allowed ObservedState.
- CMake installs and registers the new scripts/tests, but colcon currently times out the scenario generator test at 60 seconds, even though direct pytest passes.

## Capability Status

| Capability | Status | Evidence | Gap |
|---|---|---|---|
| High-level goal input | Partial | Real agent consumes `/room_315/vla/user_goal`; scenario generator consumes case IDs and language templates. | Goal is not compiled into a live PDDL problem in the real agent. |
| ObservedState from deployable signals | Partial | `model_input` is limited to `language`, `overhead_images`, `last_command`, and `observable_state`; boundary checks reject privileged fields. | ObservedState has sensors/switches/stoppers only; no allowed rail pose/load perception from vision. |
| PlanSys2 planning | Partial | `PlanSysPlannerBackend` calls `/planner/get_plan` and fallback backends are rejected. | `generate_scenario` still creates local plans for `clearance_steps`, `blocker_shuttle`, and `target_slot + selection_policy`. |
| PDDL domain | Partial | Domain includes station, slot, shuttle, payload predicates, and coarse actions. | Switch/stopper device states, block reservations, safety constraints, headway, and load transitions are outside PDDL. |
| Plan to one atomic action | Implemented | Plans translate to primitive commands and schema-v3 action vectors. | The real VLA agent bypasses symbolic planning and emits direct primitive/action-vector output. |
| Safety gate | Implemented | Supervisor decoder validates schema-v3 commands, rejects unsafe/ambiguous actions, and records metrics. | Rejected actions stop the current scenario path; they do not trigger automatic re-observe/replan. |
| Execute through supervisor | Implemented | Scenario execution publishes to the VLA supervisor, waits for decisions, and waits for switch/stopper/shuttle/arrival outcomes. | No live controller loop selects only the next PlanSys2 action after every observation. |
| Re-observe/replan | Missing | Execution waits for outcomes and verifies final goal. | There is no loop that rebuilds ObservedState and replans after each atomic action or after safety rejection/world drift. |
| Local deterministic symbolic planning branches | Legacy | `generate_scenario` can synthesize blocker, clearance, and selected-slot plans before the PlanSys2 backend is reached. | These branches should be removed from active generation or restricted to explicitly named review/debug paths. |
| Offline review planner | Legacy | `room_315_payload_case_review.py` defines `ReviewPlanner` for review-pack generation. | Useful for inspection, but it can mask whether an active case is PlanSys2-solvable. |
| Direct model-to-action agent mode | Legacy | `room_315_real_vla_agent.py` accepts provider `action_vector` or primitive JSON command output. | Keep only as an experimental baseline once the neuro-symbolic controller exists. |
| Dataset split | Implemented | Splitter groups speed variants by base family and requires approved validation by default. | Splitter does not validate image existence or model-facing feature purity. |
| LeRobot conversion | Partial | Converter writes full/compact32 state, images, language, and action vectors. | Compact32 leaks row-level planner/payload metadata; missing images become black frames. |
| Local PyTorch training | Partial | Helper builds vocab/state vectorizer/target stats/checkpoints/metrics from splits. | Missing images become zeros; training is behavior cloning over event labels, not closed-loop replanning. |
| SmolVLA evaluation | Partial | Evaluator computes action-vector metrics, per-field accuracies, and per-task summaries. | It is offline only; no safety, execution, re-observation, or replanning metrics. |
| Reports | Partial | Week 1/2 reports accurately capture progress on schema-v3, splitting, LeRobot, training, and evaluation. | They should explicitly call out the PlanSys2 bypasses, compact32 leakage, and closed-loop gap. |
| Packaging/install | Partial | CMake installs split/conversion/training/eval scripts and registers `test_room315_vla_training_tools`. | `package.xml` does not declare optional Python training deps; CTest timeout is too low for scenario generator. |
| Test coverage | Partial | Direct focused pytest passed 125 tests; model-input, PlanSys2 backend, validation, agent schema-v3, recorder, extractor, and training tools are covered. | Missing fail-closed tests for image refs/files, compact32 leakage, no-bypass PlanSys2, and re-observe/replan behavior. |

## Keep

| Area | Keep | Why |
|---|---|---|
| Model-input boundary | Keep `model_input_is_clean`, validation-gate privileged path checks, and dataset-recorder validation. | These are the right guardrails for preventing privileged state from entering learned policy input. |
| PlanSys2 adapter | Keep `PlanSys2GetPlanClient` and fail-closed errors when PlanSys2 is missing. | The production backend boundary is clear and mockable. |
| Event-level action schema | Keep schema-v3 action vectors and symbolic event labels. | They provide an atomic action target that can pass through the supervisor safety decoder. |
| Safety decoder | Keep supervisor-side validation and metrics. | It is the strongest existing safety boundary before rail execution. |
| Validation gate | Keep fail-closed validation for execution success, final goal satisfaction, recorded events, and privileged model inputs. | This protects training exports from unapproved episodes. |
| Split-by-family logic | Keep speed variants grouped in the same train/val/test family. | It avoids leakage across speed-sweep variants. |

## Refactor

| Area | Refactor | Acceptance target |
|---|---|---|
| `generate_scenario` planning | Remove local symbolic-plan branches for selection and blocker cases, or mark them explicitly as legacy review-only paths. | Every active training case obtains its symbolic plan from PlanSys2. |
| Real VLA agent | Replace direct model-to-command planning with a neuro-symbolic orchestrator. | A high-level goal produces a PDDL problem from ObservedState, requests PlanSys2, executes one action, then re-observes. |
| Compact32 conversion | Stop deriving model-facing state from `pddl_goal`, `pddl_problem`, or top-level `payload_present`. | `observation.state` is derived only from allowed `model_input` fields, or the feature is marked privileged and excluded. |
| Missing image handling | Make conversion/training fail closed by default when required image refs or files are missing. | Blank/zero images require an explicit debug flag and are counted in the summary. |
| Test registration | Increase or split the CTest scenario-generator registration. | `colcon test --packages-select mfja_robot_control_config` passes without timeout. |

## Add

| Area | Add | Acceptance target |
|---|---|---|
| Closed-loop controller | Add a controller that owns the target contract end to end. | One tick performs observe -> plan -> choose next atomic action -> safety -> execute -> observe/replan. |
| PDDL state builder | Add ObservedState-to-PDDL problem generation from live supervisor/allowed perception state. | No row-level planner metadata is needed to plan from the current world. |
| Richer PDDL domain | Add switch/stopper states, block occupancy/reservations, headway predicates, payload/load state, and safety preconditions. | PlanSys2 can express the blocker and slot-selection cases without local code branches. |
| Bbox-to-rail/load perception | Add an allowed perception module that maps detections to rail-side/slot/load observations with confidence. | Load/identity/slot facts can be used for planning without exposing privileged Gazebo pose. |
| Dataset integrity checks | Add image existence/freshness checks to split/conversion/training tests. | Missing images fail in normal mode and appear in manifest metrics. |
| Closed-loop evaluation | Add simulator evaluation metrics for plan validity, safety rejection recovery, replan count, task success, and timeouts. | Offline SmolVLA accuracy is complemented by execution-level success. |

## Defer

| Area | Defer | Reason |
|---|---|---|
| Full SmolVLA dependency packaging | Defer hard ROS package deps for `torch`/`lerobot` if the local training venv remains the intended runtime. | These are heavy optional research dependencies. |
| Real robot deployment | Defer until Gazebo closed-loop acceptance passes. | The controller still needs re-observe/replan and richer safety recovery. |
| Learned high-level planner | Defer any model that emits PDDL goals/plans. | The immediate need is a symbolic PlanSys2 loop with learned atomic-action proposal/evaluation only where safe. |

## Acceptance Criteria

| Contract step | Current state | Acceptance criteria |
|---|---|---|
| Goal -> ObservedState | Goal text exists; ObservedState exists as `model_input.observable_state`. | A live goal tick snapshots fresh status/images into an `ObservedState` object with allowed fields only and freshness checks for required cameras. |
| ObservedState -> PDDL/PlanSys2 | Some cases call PlanSys2; active slot/blocker cases may bypass it. | All active cases build a PDDL problem from ObservedState and call PlanSys2 exactly once per planning cycle. |
| PlanSys2 -> one atomic action | Scenario generator translates whole plans to primitive command lists/action vectors. | Runtime selects only the next applicable plan step, emits one atomic command/action vector, and stores plan provenance outside model input. |
| One action -> safety | Supervisor safety decoder exists. | Every command goes through the supervisor; rejection prevents execution and triggers re-observation/replanning or a safe stop. |
| Safety -> execute | Scenario executor publishes through supervisor and waits for decisions/outcomes. | Runtime execution waits for the specific postcondition of the one action before continuing. |
| Execute -> re-observe/replan | Final verification and waits exist, but no loop. | After every action, rebuild ObservedState; if the plan tail is invalid, call PlanSys2 again before the next action. |
| Model-input purity | Recorder/extractor/agent guard the JSON `model_input`. | LeRobot `observation.state` and all training features obey the same boundary; compact32 no longer reads `pddl_*` or top-level payload labels. |
| Image integrity | Converter/training use black/zero fallback. | Required image refs/files must exist by default; any fallback mode is opt-in and reported. |
| Perception | Bbox identity tracks are privileged metadata only. | Detection bboxes produce allowed rail/slot/load facts with confidence and tests against occlusion cases. |
| Tests/package | Direct focused pytest passes; colcon times out one long test. | Focused pytest and colcon both pass; CMake timeout or test split reflects the scenario generator runtime. |

## Validation Run

| Command | Result |
|---|---|
| `python3 -m pytest -q mfja_robot_control_config/test/test_room315_pddl_planner_backend.py mfja_robot_control_config/test/test_room315_pddl_scenario_generator.py mfja_robot_control_config/test/test_room315_pddl_scenario_validation_gate.py mfja_robot_control_config/test/test_room315_vla_model_input_boundary.py mfja_robot_control_config/test/test_room315_real_vla_agent.py mfja_robot_control_config/test/test_room315_real_vla_agent_schema_v3.py mfja_robot_control_config/test/test_room315_vla_event_extractor.py mfja_robot_control_config/test/test_room315_vla_dataset_recorder.py mfja_robot_control_config/test/test_room315_vla_training_tools.py` | Passed: 125 tests in 136.29s. |
| `colcon test --packages-select mfja_robot_control_config --event-handlers console_direct+` | 36/37 CTest entries passed; `test_room315_pddl_scenario_generator` timed out at the registered 60s CTest timeout. |
| `ctest --test-dir build/mfja_robot_control_config -R test_room315_pddl_scenario_generator --timeout 180 --output-on-failure` | Still timed out at the test registration's 60s limit; direct pytest proves the assertions pass when not constrained by CTest timeout. |

## Suspected Gaps Verified

| Suspected gap | Verdict | Notes |
|---|---|---|
| PlanSys2 bypasses in `generate_scenario` | Verified | Local symbolic plan generators are still used before `PlanSysPlannerBackend` for blocker, clearance, and selected target-slot paths. |
| Direct-action output in `room_315_real_vla_agent.py` | Verified | The agent posts `model_input` to HTTP and accepts `action_vector` or primitive JSON command output directly. |
| Coarse PDDL domain | Verified | The domain comments and predicates/actions keep low-level devices, safety, speed, and route detail outside PDDL. |
| Missing bbox-to-rail/load perception | Verified | Marker bboxes become privileged identity tracks; payload load comes from status messages, not image/bbox-derived allowed perception. |
| Model-input leakage | Verified in compact32 conversion | Recorder/extractor/agent JSON boundaries are clean, but compact32 LeRobot state reads non-model row fields. |
| Silent missing images | Verified | LeRobot conversion returns blank images and local training returns zero tensors for missing refs/files. |
| Incomplete install/test registration | Partial | Scripts and pytest are registered, but CTest timeout causes a colcon failure; optional Python deps are not declared as package dependencies. |
