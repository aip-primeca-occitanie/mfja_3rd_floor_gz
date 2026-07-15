# Room 315 Task-Goal Understanding

Room 315 task-goal understanding is an offline, English-only front end for the
closed-loop symbolic executive. Its production responsibility is intentionally
narrow: convert a human or local semantic-model request into a validated,
immutable `TaskGoal`. It never produces PDDL, plans, action vectors, primitive
commands, device commands, or editable safety constraints.

## Contract Flow

The production flow is:

```text
English request or structured form
  -> TaskGoalDraft
  -> deterministic Room 315 validation
  -> optional clarification
  -> risk-based confirmation
  -> immutable TaskGoal
  -> ObservedState + PlanSys2 + supervisor
```

`TaskGoalDraft` is nullable by design. Missing fields remain `null` until the
dialogue layer asks for exactly the missing information. The validator must not
guess ambiguous or incomplete values.

## TaskGoalDraft

`TaskGoalDraft` is defined in:

```text
mfja_robot_control_config/scripts/room_315_task_goal_schema.py
```

The draft separates shuttle choice from payload choice:

```text
selection_strategy: nearest | explicit | any | null
payload_filter:     loaded  | empty    | any | null
```

This avoids the old overloaded meaning where values such as `loaded`,
`nearest`, and `explicit` all lived in one selection field. Legacy compatibility
keys are still generated in final `TaskGoal.constraints` where practical:

```text
shuttle_selection
payload_required
```

The strict local-model JSON schema accepts only draft fields. Any model output
containing PDDL, plans, action vectors, primitive commands, rail commands,
device commands, privileged state, or safety constraints is rejected before
domain validation.

## Parsers

Parser modules live in:

```text
mfja_robot_control_config/scripts/room_315_task_goal_parsers.py
```

The parser interface is `TaskGoalDraftParser`. Implementations are:

```text
StructuredFormParser
DeterministicEnglishParser
LocalSemanticModelAdapter
RegexFallbackParser
ParserPipeline
```

`StructuredFormParser` accepts trusted structured input dictionaries and
normalizes aliases such as `station`, `slot`, `shuttle_selection`, and
`payload_required` into the draft schema.

`DeterministicEnglishParser` is the production offline English parser. It
extracts only grounded Room 315 entities such as rail side, shuttle IDs,
stations, slots, inspection targets, payload filters, and selection strategy.
Non-English text is rejected.

`LocalSemanticModelAdapter` is replaceable. The local model callable may return
strict draft JSON only. The adapter validates that JSON with
`strict_model_draft_from_json()` before anything reaches the domain validator.

`RegexFallbackParser` keeps the current regex-style baseline available as a
fast fallback and compatibility path.

## Domain Validation

Domain validation lives in:

```text
mfja_robot_control_config/scripts/room_315_task_goal_validation.py
```

`Room315DomainValidator` checks every draft against deterministic Room 315
ground truth:

```text
rails:    right, left
stations: right -> yaskawa, staubli
          left  -> yaskawa, kuka
slots:    1, 2, 3, 4
shuttles: R1-R4, L1-L4
goals:    transport, inspection
```

Validation fails closed for:

```text
unknown shuttles
invalid slots
station/side mismatches
shuttle/side conflicts
unsupported goal types
unsupported target kinds
forbidden model fields
malformed model JSON
```

Validation asks for clarification for:

```text
missing side
missing station or slot target
missing selection strategy
missing payload filter
ambiguous yaskawa side
missing explicit shuttle ID
missing inspection subject
```

When validation succeeds, the validator builds an immutable `TaskGoal` with a
deterministic ID derived from the normalized constraints.

## Dialogue And Confirmation

Dialogue state lives in:

```text
mfja_robot_control_config/scripts/room_315_task_goal_dialogue.py
```

`TaskGoalDialogueManager` keeps a pending `TaskGoalDraft`, merges short answers,
and asks field-specific questions. For example, if the user says:

```text
move the loaded shuttle to slot 3
```

the manager keeps the incomplete draft and asks for the missing rail side.
A short answer such as:

```text
right
```

is merged into the pending draft instead of being treated as a new goal.

Clarification attempts are bounded. If the user keeps providing unclear or
contradictory answers, no unsafe automatic resolution is made and no `TaskGoal`
is finalized.

Risk-based confirmation is required before finalizing medium- or high-risk
goals through the dialogue layer. Transport goals are high risk. Inspection
goals involving a shuttle, rail, or shuttle selection are medium risk.

## Public API Compatibility

The existing public entry points remain:

```text
build_task_goal(request, ...)
parse_model_task_goal_json(model_output, ...)
```

Both now route through `TaskGoalDraft`, strict schema validation, deterministic
domain validation, and final `TaskGoal` construction.

Code that still reads legacy `TaskGoal.constraints` can continue using:

```text
shuttle_selection
payload_required
```

New code should prefer:

```text
selection_strategy
payload_filter
```

The PDDL scenario generator accepts both forms and normalizes them before
building PlanSys2 goal data.

## Safety Boundary

Task-goal understanding does not execute anything. It is upstream of
`ObservedState`, PDDL problem construction, PlanSys2 planning, and supervisor
execution. The only production output from this layer is a validated `TaskGoal`.

Learned components may suggest a draft goal, but they cannot:

```text
publish rail commands
emit primitive commands
write plans
write PDDL
set safety constraints
edit supervisor gates
inject privileged simulator/device state
```

Any such field in local-model output is treated as malicious or out of scope and
is rejected.

## Tests

Primary tests are in:

```text
mfja_robot_control_config/test/test_room315_task_goal_builder.py
```

They cover:

```text
clear transport requests
nearest loaded shuttle
inspection requests
ambiguous yaskawa side
shuttle/side conflicts
invalid slots
strict local-model JSON
malicious model fields
malformed JSON
parser fallback
multi-turn clarification
confirmation decline
deterministic IDs
zero unsafe automatic resolution
```

Run the focused suite with:

```bash
pytest -q mfja_robot_control_config/test/test_room315_task_goal_builder.py
```

Run the related planning/contract regression set with:

```bash
pytest -q \
  mfja_robot_control_config/test/test_room315_task_goal_builder.py \
  mfja_robot_control_config/test/test_room315_contracts.py \
  mfja_robot_control_config/test/test_room315_pddl_planning.py \
  mfja_robot_control_config/test/test_room315_pddl_planner_backend.py \
  mfja_robot_control_config/test/test_room315_pddl_scenario_generator.py
```
