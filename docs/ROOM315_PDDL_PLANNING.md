# Room 315 PDDL planning

## Live request contract

The live planner accepts one atomic, grounded goal at a time:

- transport one present `L1`–`L4` or `R1`–`R4` shuttle to an exact slot or a
  configured station;
- choose it explicitly, by `any`, or by shortest authoritative topology
  distance, with an optional loaded/empty filter; or
- perform a non-actuating inspection of a valid system, rail, present-shuttle,
  slot, or station subject.

Inspection of an unnamed shuttle supports deterministic `any` selection with
an optional loaded/empty filter. A concrete inspection names one shuttle.
`nearest` inspection fails during TaskGoal validation because the atomic
inspection schema has no independent slot/station distance reference; it is
never accepted and deferred to runtime grounding. System, rail, slot, and
station inspections reject shuttle selector or payload-filter fields.
The terminal inspection marker is accepted only after both the observation ID
and timestamp advance; replayed frames fail closed as unknown freshness.

Natural-language parsing is not allowed to invent a PDDL fact. The accepted
visual state supplies segment/location and payload facts, while the
deterministic presence provider supplies presence only. Unknown, stale,
duplicate, conflicting, absent-explicit, or physically infeasible states fail
closed before planning.

For an atomic transport goal, exact slot-anchor versus learned-position
consistency is enforced on the requested rail. The opposite rail remains in
the authoritative presence, identity, occupancy, device, obstacle, and global
safety checks, but a transient location disagreement there is reported in the
problem provenance instead of vetoing an independent-rail route. Planning a
later goal on that rail makes the same disagreement route-relevant and fails
closed. System inspection retains all-rail location validation.

When a route-relevant input remains uncertain, a recovery retry is consumed
only after the accepted visual `state_id` advances. Rebuilding the same cached
frame is not counted as another observation; if no newer frame arrives within
the bounded observation window, the executive aborts safely with an explicit
fresh-observation-unavailable reason.

The problem generator reads the authoritative rail topology and enumerates all
valid switch routes, including every public segment and switch assignment. It
distinguishes continuous segment origins from exact slot origins. The latter
must use actions whose effects release the source slot as the destination is
occupied.

Blocker handling is closed-loop and capacity-aware:

1. PlanSys2 selects a route and one immediate supervised action.
2. The executive verifies the physical effect and obtains a fresh visual
   observation.
3. The problem is regenerated before the next action.
4. Topology-route setup requires a verified normal route, so a segment-origin
   clearance phase must finish explicitly before target routing can begin.
5. At most two safely separated shuttles are staged in A34I.
6. Receding-horizon execution still accounts for every blocker visible in the
   current accepted frame when selecting the first staging pose. With two
   interior relocations it preserves the `0.95 m` and `0.35 m` endpoint pair;
   only the first action is committed before re-observation.
7. A three-blocker full-rail case uses a certified clearance pause, verified
   normal-route restoration, and an intermediate-slot token advance before
   clearance resumes.
8. If another shuttle occupies the requested target slot, it is the first
   blocker. It is relocated to an audited reachable free slot (or to a
   capacity-checked interior clearance pose), its identity-bearing arrival is
   verified, and the problem is rebuilt. An unanchored short "nudge" is not a
   valid clearance destination because it cannot prove exact occupancy or a
   deterministic stopped effect.

The expert and runtime domains intentionally expose the same executable PDDL
actions. Terminal actions are non-actuating bookkeeping; device and shuttle
actions have explicit translators, validation-gate rules, supervisor checks,
and postcondition verification.

Requests requiring compound agendas, simultaneous swaps, load/unload, metric
coordinate goals, or unrestricted speed/deadline optimization need a new
versioned goal schema. They are not silently approximated by this domain.

## Curated payload-case workflow

The active planning workflow is case based. The checked-in source of truth for
regression is the curated 160 payload speed-sweep cases:

```text
mfja_robot_control_config/config/room_315_vla/payload_training_cases_expanded_160_speed_sweep.yaml
```

Each case declares the side, target slot, loaded shuttle candidates, starting
slots, optional blocker-clearance strategy, launch parameters, and expected
selected shuttle. The scenario generator resolves one case into symbolic steps,
primitive commands, expected event targets, and planner provenance.

## Dry Run

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --case-id right_loaded_r1_s1_to_slot3_no_blocker_speed008 \
  --language-template-id loaded_shuttle_to_slot \
  --dry-run
```

## Preflight

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --case-id right_loaded_r1_s1_to_slot3_no_blocker_speed008 \
  --language-template-id loaded_shuttle_to_slot \
  --command-timeout-s 8 \
  --preflight-only \
  --ready-line
```

## Execute

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --case-id right_loaded_r1_s1_to_slot3_no_blocker_speed008 \
  --language-template-id loaded_shuttle_to_slot \
  --command-timeout-s 30 \
  --arrival-timeout-s 120 \
  --require-dataset-recorder \
  --output /tmp/right_loaded_r1_s1_to_slot3_no_blocker_speed008_execute.json \
  --quiet \
  --execute
```

## Batch Runner

```bash
ros2 run mfja_robot_control_config room_315_payload_case_batch_runner.py \
  --case-config mfja_robot_control_config/config/room_315_vla/payload_training_cases_expanded_160_speed_sweep.yaml \
  --dataset-dir ~/room315_payload_expanded_160_speed_sweep \
  --results-dir /tmp/room315_payload_expanded_160_speed_sweep
```

The batch runner launches Room 315 for each selected case, checks preflight,
executes the generated primitive sequence, waits for a complete dataset episode,
and writes `payload_case_batch_summary.json`.

## Seeded Benchmark Expansion

Use the benchmark suite to create a deterministic extension manifest without
committing generated artifacts:

```bash
ros2 run mfja_robot_control_config room_315_vla_benchmark_suite.py generate-cases \
  --extension-case-count 320 \
  --seed 315 \
  --output /tmp/room315_seeded_balanced_cases.yaml
```

The generated file embeds the 160-case regression subset and adds 100 to 1000
balanced extension cases. The extension explicitly covers 4+4 fleets,
loaded/empty shuttle selection, blockers, occupied targets, unknown positions,
sensor dropout, obstacles, inspection, and simultaneous requests.

Comparison reports should keep `oracle_plansys2`, `frozen_visual_plansys2`, and
`lora_visual_plansys2` in separate rows, and should keep Gazebo planning
evidence separate from real-image perception claims. See
[ROOM315_BENCHMARK_RUNBOOK.md](ROOM315_BENCHMARK_RUNBOOK.md).
