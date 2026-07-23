# Room 315 Task-Goal Understanding

Room 315 task-goal understanding is an offline, English-only front end for the
closed-loop symbolic executive. Its production responsibility is intentionally
narrow: convert a human or local semantic-model request into a validated,
immutable `TaskGoal`. It never produces PDDL, plans, primitive commands, device
commands, rail-control payloads, or editable safety constraints.

## Contract Flow

The production flow is:

```text
English request or structured form
  -> TaskGoalDraft
  -> ParseTrace audit
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

This keeps payload constraints separate from shuttle-selection strategy.
Compatibility keys are still generated in final `TaskGoal.constraints` where
practical:

```text
shuttle_selection
payload_required
```

The semantic model outputs a strict `SemanticParseEnvelope`, not a final
`TaskGoal`. The envelope is fused with deterministic explicit facts before a
draft is built. Any model output containing PDDL, plans, primitive commands,
rail commands, device commands, privileged state, or safety constraints is
rejected before fusion and domain validation.

## Parsers

Parser modules live in:

```text
mfja_robot_control_config/scripts/room_315_task_goal_parsers.py
```

The parser interface is `TaskGoalDraftParser`. Implementations are:

```text
StructuredFormParser
ConversationalIntentGatewayParser
DeterministicEnglishParser
LocalSemanticModelAdapter
RegexFallbackParser
ParserPipeline
```

`StructuredFormParser` accepts trusted structured input dictionaries and
normalizes aliases such as `station`, `slot`, `shuttle_selection`, and
`payload_required` into the draft schema.

`ConversationalIntentGatewayParser` is the active production free-text parser.
It runs two components for English text: a high-precision deterministic
explicit-fact extractor and a local semantic-model backend.

The deterministic extractor captures exact Room 315 facts with source spans:

```text
R1-R4, L1-L4
right, left
slot 1-4
yaskawa, staubli, kuka
loaded, empty
nearest, closest
explicit transport and inspection terms
```

It favors precision over recall and does not perform broad semantic guessing.
The semantic model may fill missing semantic fields, but it may not override
explicit shuttle IDs, sides, slots, stations, payload terms, or confirmed
dialogue values.

`DeterministicEnglishParser` remains available as a compatibility parser.
Non-English text is rejected.

`LocalSemanticModelAdapter` is retained as a compatibility adapter for strict
draft JSON. The active free-text pipeline uses the semantic envelope backend.

`RegexFallbackParser` keeps a fast deterministic fallback available for simple
phrases and compatibility paths.

## Semantic Envelope

The semantic envelope schema lives in:

```text
mfja_robot_control_config/scripts/room_315_task_goal_semantic.py
```

The model must return one strict JSON object:

```text
contract_type: SemanticParseEnvelope
dialogue_act: new_goal | answer | correction | confirm | reject | cancel | restart | help
draft_patch: nullable TaskGoalDraft fields only
evidence: optional per-field evidence text
provenance: explicit_text | explicit_correction | confirmed_context | semantic_inference | structured_form | user_edited
alternatives: optional bounded alternative interpretations
confidence: diagnostics only
```

Unknown or forbidden envelope fields are rejected before fusion.

## Local Model Runtime

The concrete production backend is:

```text
LlamaCppSemanticBackend
```

It loads a local GGUF checkpoint through `llama-cpp-python` in CPU-first mode.
Runtime never downloads weights and refuses URI, relative, missing, or checksum
mismatched model paths. `TransformersSemanticBackend` remains available for
explicit local Hugging Face directories, but the active YAML selects
`llama_cpp`.

Configuration is versioned here:

```text
mfja_robot_control_config/config/room_315_vla/task_goal_understanding.yaml
```

Configurable fields include backend, model path, SHA-256 checksum, device,
quantization, context size, thread/GPU-layer settings, timeout, retry count,
deterministic generation settings, shadow mode, and deterministic-only mode.
The package YAML uses `ROOM315_INTENT_MODEL_PATH` as an environment override so
the real absolute checkpoint path stays outside Git.

Generation is deterministic: temperature 0, bounded output tokens, strict JSON,
JSON Schema constrained decoding for the llama.cpp backend, and no
chain-of-thought output.

The selected local checkpoint is:

```text
repo: Qwen/Qwen2.5-1.5B-Instruct-GGUF
file: qwen2.5-1.5b-instruct-q4_k_m.gguf
path: /home/tiago/models/room315_intent/qwen2.5-1.5b-instruct-q4_k_m.gguf
sha256: 6a1a2eb6d15622bf3c96857206351ba97e1af16c30d7a74ee38970e434e9407e
```

Set up or verify the local model explicitly:

```bash
python3 mfja_robot_control_config/scripts/setup_room315_intent_model.py
source /home/tiago/models/room315_intent/room315_intent.env
```

The setup script is idempotent. It installs `llama-cpp-python==0.3.16` if
needed, downloads the GGUF only when the script is explicitly run and the file
is missing, verifies SHA-256, and writes local config/env files under
`/home/tiago/models/room315_intent/`. Do not commit checkpoints, pip caches,
virtual environments, or generated local configs.

If the model is unavailable, unhealthy, times out, or returns invalid JSON, the
parser continues with deterministic extraction and targeted clarification. The
fallback reason is recorded in `ParseTrace`.

## Fusion And Audit

Evidence fusion lives in:

```text
mfja_robot_control_config/scripts/room_315_task_goal_fusion.py
```

`EvidenceAwareFusionResolver` applies this precedence:

```text
structured or user-edited values
explicit correction in the current turn
explicit deterministic facts from current text
previously confirmed dialogue values
semantic-model inference
null
```

The final `TaskGoal` schema is not polluted with trace data. Audit metadata is
returned separately in `normalized_request.parse_trace`: source spans, model
evidence, parser disagreements, fallback reason, model fingerprint,
prompt/schema versions, latency, validation result, final decision,
`model_ready`, `semantic_model_invoked`, and `fallback_used`.

If explicit text and the semantic model disagree, exact explicit text wins and
the disagreement is logged. If the user text itself contains conflicting
explicit facts, the parser returns structured clarification and never resolves
automatically.

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
handles corrections, cancellation, restart, confirmation/rejection, grounded
references, and asks field-specific questions. For example, if the user says:

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
Any goal that required clarification or correction also requires confirmation.

Before finalization, the dialogue manager provides a canonical summary:

```text
Action: Transport
Shuttle: nearest loaded shuttle
Rail: Right
Destination: Slot 3
```

## Public API Compatibility

The existing public entry points remain:

```text
build_task_goal(request, ...)
parse_model_task_goal_json(model_output, ...)
```

Both now route through `TaskGoalDraft`, strict schema validation, deterministic
domain validation, and final `TaskGoal` construction.

Code that still reads compatibility `TaskGoal.constraints` keys can continue using:

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

## CLI And Health Checks

Offline smoke test:

```bash
source /home/tiago/models/room315_intent/room315_intent.env
PYTHONPATH=mfja_robot_control_config/scripts \
python3 mfja_robot_control_config/scripts/room_315_task_goal_semantic_smoke.py \
  --text "please move the nearest loaded right shuttle to slot 3"
```

Require a real local model and prove semantic inference was used:

```bash
source /home/tiago/models/room315_intent/room315_intent.env
PYTHONPATH=mfja_robot_control_config/scripts \
python3 mfja_robot_control_config/scripts/room_315_task_goal_semantic_smoke.py \
  --require-real-model \
  --expect-semantic \
  --expect-draft-field selection_strategy=nearest \
  --expect-draft-field payload_filter=loaded \
  --expect-draft-field side=right \
  --expect-draft-field target_slot=3 \
  --text "Could you send whichever carrier is closest and holding a component to the third position on the right-hand line?"
```

Prove fully offline execution after the checkpoint is present:

```bash
bwrap --unshare-net --ro-bind / / --dev /dev --proc /proc --tmpfs /tmp \
  --chdir /home/tiago/mfja_3rd_floor_ros2_ws/src/mfja_3rd_floor_gz \
  bash -lc 'ROOM315_INTENT_MODEL_PATH=/home/tiago/models/room315_intent/qwen2.5-1.5b-instruct-q4_k_m.gguf HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 PYTHONPATH=mfja_robot_control_config/scripts python3 mfja_robot_control_config/scripts/room_315_task_goal_semantic_smoke.py --require-real-model --expect-semantic --expect-draft-field selection_strategy=nearest --expect-draft-field payload_filter=loaded --expect-draft-field side=right --expect-draft-field target_slot=3 --text "Could you send whichever carrier is closest and holding a component to the third position on the right-hand line?"'
```

Interactive user-facing command entry uses the dialogue manager, not
`build_task_goal()` directly:

```bash
source /home/tiago/models/room315_intent/room315_intent.env
PYTHONPATH=mfja_robot_control_config/scripts \
python3 mfja_robot_control_config/scripts/room_315_task_goal_cli.py
```

Transport goals are high risk. The CLI asks clarifying questions and shows the
canonical summary, then prints a final validated `TaskGoal` only after an
explicit `yes`.

Shadow mode keeps deterministic behavior authoritative while logging the model
result:

```bash
ros2 run mfja_robot_control_config room_315_task_goal_semantic_smoke.py \
  --shadow-mode \
  --text "move the nearest loaded right shuttle to slot 3"
```

Benchmark report:

```bash
ros2 run mfja_robot_control_config room_315_task_goal_benchmark.py \
  --corpus mfja_robot_control_config/config/room_315_vla/task_goal_english_benchmark.yaml \
  --output /tmp/room315_task_goal_benchmark_report.json
```

The benchmark reports field-level accuracy, exact TaskGoal accuracy,
ambiguity-detection recall, mean clarification turns, correction/cancellation
accuracy, parser disagreement rate, fallback rate, p50/p95 latency, and unsafe
automatic resolution count.

## Tests

Primary tests are in:

```text
mfja_robot_control_config/test/test_room315_task_goal_builder.py
mfja_robot_control_config/test/test_room315_task_goal_semantic_gateway.py
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
semantic envelope validation
fake backend invocation
model timeout and unhealthy fallback
parser disagreement audit
shadow mode
cancel/restart/correction dialogue
grounded and unresolved references
```

Run the focused suite with:

```bash
pytest -q mfja_robot_control_config/test/test_room315_task_goal_builder.py
pytest -q mfja_robot_control_config/test/test_room315_task_goal_semantic_gateway.py
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
