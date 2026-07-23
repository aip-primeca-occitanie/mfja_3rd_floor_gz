# Room 315 Neuro-Symbolic Current Architecture

Audit date: 2026-07-18
Branch: `ali/neuro-symbolic-closed-loop`

Current contract:

```text
TaskGoal -> ObservedState -> PDDL/PlanSys2 -> one atomic primitive command
-> supervisor safety -> execute -> re-observe/replan
```

## Current Status

The active Room 315 stack is planner based. A user request is parsed into a
validated `TaskGoal`, observations are represented through versioned
`ObservedState` contracts, and PDDL problems are built from validated
`ObservedState + TaskGoal`. Production execution uses PlanSys2 plans and sends
only one primitive command at a time through the supervisor.

Learned components are restricted to visual-state perception. They may output
visual facts such as shuttle bbox, identity, rail side, slot/segment estimate,
loaded/empty/unknown state, visible switch state, obstacle evidence,
confidence, timestamp, and calibration/schema version. They do not output PDDL,
plans, primitive commands, rail commands, or editable safety constraints.

## Keep

| Area | Current implementation | Why keep it |
|---|---|---|
| Versioned contracts | `ObservedFact`, `ObservedState`, `TaskGoal`, `PlanStep`, `PrimitiveCommand`, and `StepResult`. | They define the production boundary between perception, planning, safety, and execution. |
| ObservedState providers | Oracle, fused, and visual providers produce validated facts with source, freshness, confidence, and status. | Planner state is separated from learned visual inputs and from oracle-only truth. |
| Task-goal dialogue | English deterministic parser, local strict-JSON semantic parser, structured-form parser, clarification, and confirmation policy. | User text becomes an immutable `TaskGoal` only after deterministic validation. |
| PlanSys2 path | PDDL problem builder, planner backend, symbolic plan translation, and closed-loop executive. | Production scenarios use symbolic planning before any rail command. |
| Supervisor safety | Stale/conflicting state, occupied target, reservation, headway, switch, obstacle, dropout, timeout, and emergency-stop gates. | Every primitive command is checked before rail execution. |
| Visual-state dataset mode | Split, LeRobot conversion, local training, evaluation, metrics, and label sidecars. | Learned output is perception state, not direct control. |

## Removed

| Removed item | Current replacement |
|---|---|
| Direct model-to-rail agent executable | Task-goal parser + ObservedState providers + PlanSys2 executive. |
| Numeric rail-control output datasets | Visual-state labels physically separate from model inputs. |
| Offline direct-control evaluator | Visual-state metrics and planner/executive benchmark comparison. |
| Action-space YAML for numeric rail-control outputs | Versioned contracts plus structured visual-state label vectorizer. |
| Tests that accepted direct numeric control output | Tests now assert command-like model outputs are rejected or scrubbed. |

## Acceptance Criteria

| Contract step | Required behavior |
|---|---|
| Goal understanding | Free text returns a validated `TaskGoalDraft`, asks clarification when needed, and finalizes only through the dialogue policy. |
| Observation | Provider output includes source, timestamp, confidence, freshness, and known/unknown/stale/conflicting status. |
| Planning | Active scenarios build PDDL from `ObservedState + TaskGoal` and request PlanSys2. Unknown facts fail closed or trigger recovery. |
| Execution | The executive sends exactly one atomic primitive command to the supervisor per cycle. |
| Safety | The supervisor rejects unsafe or uncertain commands and starts safe-stop/reobserve/replan recovery when possible. |
| Learning boundary | Visual models consume only declared visual inputs and output visual facts only. |
| Dataset integrity | Missing images fail by default; blank images require an explicit debug flag and are reported. |
| Evaluation | Reports separate Gazebo planning evidence from real-image perception claims and include fingerprints. |

## Notes

The code still contains forbidden-field guards for removed numeric control
payloads. Those guards are intentional: they reject stale artifacts at the
contract and supervisor boundaries so older experiment data cannot silently enter
the production path.
