# Room 315 Seeded Benchmark Runbook

This benchmark layer extends the checked-in 160 payload speed-sweep cases with a
seeded, balanced extension. The generated YAML is a run artifact: keep it under
`/tmp` or another experiment directory, and do not commit generated datasets or
checkpoints.

## Generate Cases

The retained regression subset is always the existing 160-case file:

```text
mfja_robot_control_config/config/room_315_vla/payload_training_cases_expanded_160_speed_sweep.yaml
```

Generate 100 to 1000 additional balanced cases:

```bash
ros2 run mfja_robot_control_config room_315_vla_benchmark_suite.py generate-cases \
  --extension-case-count 320 \
  --seed 315 \
  --output /tmp/room315_seeded_balanced_cases.yaml
```

The output contains:

| Subset | Purpose |
|---|---|
| `regression_160` | The existing curated speed-sweep cases, copied unchanged except for benchmark metadata. |
| `seeded_balanced_extension` | Balanced coverage for 4+4 fleets, loaded and empty selection, blockers, occupied targets, unknown positions, sensor dropout, obstacles, inspection, and simultaneous requests. |

The generated extension is deterministic for a given seed. Its family counts are
balanced by construction, with a maximum count difference of one when the count
is not divisible by the number of benchmark families.

## Compare Methods

Collect one result JSON per method, then normalize them into one comparison:

```bash
ros2 run mfja_robot_control_config room_315_vla_benchmark_suite.py compare-results \
  --result-json /tmp/room315_oracle_plansys2.json \
  --result-json /tmp/room315_frozen_visual_plansys2.json \
  --result-json /tmp/room315_lora_visual_plansys2.json \
  --result-json /tmp/room315_legacy_direct_action_smolvla.json \
  --output /tmp/room315_method_comparison.json
```

The comparison expects these method names or aliases:

| Method | Meaning |
|---|---|
| `oracle_plansys2` | Gazebo/device truth observed state fused into PlanSys2. |
| `frozen_visual_plansys2` | Frozen compact visual model feeding state fusion and PlanSys2. |
| `lora_visual_plansys2` | LoRA-adapted compact visual model feeding state fusion and PlanSys2. |
| `legacy_direct_action_smolvla` | Disabled legacy direct-action baseline, not a PlanSys2 loop. |

Required comparison metrics are success rate, false success rate, safety
violation rate, supervisor rejection rate, mean replans, mean route length,
mean completion time, p50 latency, and p95 latency. Missing metrics remain
`null`; the tool does not fabricate proxy values.

## Claim Boundaries

Simulator/Gazebo planning results and real-image perception claims must be
reported separately.

| Scope | Allowed claim |
|---|---|
| `gazebo_planning` | Planner, executive, safety, route, and timing behavior in simulation. |
| `real_image_perception` | Visual-state perception claims from real images, with calibration, split, and checkpoint fingerprints. |
| `legacy_direct_action_offline` | Direct-action SmolVLA action-vector metrics only. This is not closed-loop PlanSys2 evidence. |

Visual methods are not treated as real-image perception results unless their
result JSON explicitly declares `perception_source: real_image`.

## Limitations

- The generated manifest is a benchmark case specification, not a recorded
  dataset. It does not include images, checkpoints, or model outputs.
- Stressors such as unknown position, dropout, obstacle appearance, and
  simultaneous requests are encoded as benchmark metadata so the executive and
  safety layers can be evaluated without pretending they are ordinary clean
  transport episodes.
- `oracle_plansys2` uses trusted simulator/device facts and should be reported
  as Gazebo planning evidence only.
- Frozen and LoRA visual methods must include checkpoint fingerprints and
  visual split fingerprints before their real-image perception numbers can be
  compared.
- Legacy direct-action SmolVLA can be listed in the same table for contrast, but
  it cannot satisfy the closed-loop contract because it does not observe,
  request PlanSys2, execute one atomic step, and re-observe/replan.
