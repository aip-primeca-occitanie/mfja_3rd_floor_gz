# Room 315 Direct-Action Baseline

Audit date: 2026-07-11
Branch: `ali/neuro-symbolic-closed-loop`
Base commit before this report: `075f488375b8528de838dcdd40b6f8042dcfe191`
Frozen workflow id: `legacy_direct_action_baseline`

## Scope

This freezes the existing direct-action workflow as a legacy baseline:

```text
model_input -> direct action_vector prediction -> schema-v3 decode -> supervisor/offline safety proxy
```

It is intentionally not the neuro-symbolic closed loop. There is no PlanSys2 planning, execution, re-observation, or replanning in these results.

## Clean Manifests

| Split | Rows | JSONL sha256 | Bytes |
|---|---:|---|---:|
| train | 2280 | `4fe10e1071828b1ac70421e3bf972edd14c8cd8e28c8a95005b6a0c615fcb418` | 8997747 |
| val | 344 | `d472c9678f8bbb3ad7a55e2ac4349dbd91de13b5d796d53e25afaede45d692fa` | 1371799 |
| test | 256 | `026eb4df3fa1da539dba9ab169e67ee47aeb575944b5b58db92e8951c67169bb` | 1007494 |

All local evaluations used `/home/tiago/room315_local_training/splits/{train,val,test}.jsonl` with dataset root `/home/tiago/room315_payload_expanded_160_merged_final`. Image checks were fail-closed; complete row rate was `1.0` on all local splits and no blank-image debug mode was used.

## Commands Run

Local PyTorch baseline, full train/val/test:

```bash
/home/tiago/room315_local_training/venv/bin/python mfja_robot_control_config/scripts/room_315_vla_train_local.py \
  --eval-checkpoint /home/tiago/room315_local_training/checkpoints/v0/best.pt \
  --splits-dir /home/tiago/room315_local_training/splits \
  --dataset-root /home/tiago/room315_payload_expanded_160_merged_final \
  --eval-output-dir /home/tiago/room315_local_training/legacy_direct_action_baseline/local_v0_best \
  --eval-splits train,val,test \
  --device cuda \
  --progress-every 500
```

Local smoke checkpoint, full train/val/test:

```bash
/home/tiago/room315_local_training/venv/bin/python mfja_robot_control_config/scripts/room_315_vla_train_local.py \
  --eval-checkpoint /home/tiago/room315_local_training/checkpoints/smoke/best.pt \
  --splits-dir /home/tiago/room315_local_training/splits \
  --dataset-root /home/tiago/room315_payload_expanded_160_merged_final \
  --eval-output-dir /home/tiago/room315_local_training/legacy_direct_action_baseline/local_smoke_best \
  --eval-splits train,val,test \
  --device cuda \
  --progress-every 500
```

SmolVLA v1 compact32 checkpoint 040000, invalid-for-comparison lineage:

```bash
/home/tiago/room315_local_training/venv/bin/python mfja_robot_control_config/scripts/room_315_smolvla_eval.py \
  --checkpoint /home/tiago/room315_local_training/smolvla_runs/v1_pretrained_compact32/checkpoints/040000/pretrained_model \
  --split test \
  --dataset-root /home/tiago/room315_local_training/lerobot/room315_vla_test_compact32 \
  --source-jsonl /home/tiago/room315_local_training/splits/test.jsonl \
  --output-dir /home/tiago/room315_local_training/legacy_direct_action_baseline/smolvla_v1_compact32_040000_invalid \
  --progress-every 64
```

```bash
/home/tiago/room315_local_training/venv/bin/python mfja_robot_control_config/scripts/room_315_smolvla_eval.py \
  --checkpoint /home/tiago/room315_local_training/smolvla_runs/v1_pretrained_compact32/checkpoints/040000/pretrained_model \
  --split val \
  --dataset-root /home/tiago/room315_local_training/lerobot/room315_vla_val_compact32 \
  --source-jsonl /home/tiago/room315_local_training/splits/val.jsonl \
  --output-dir /home/tiago/room315_local_training/legacy_direct_action_baseline/smolvla_v1_compact32_040000_invalid \
  --progress-every 64
```

```bash
/home/tiago/room315_local_training/venv/bin/python mfja_robot_control_config/scripts/room_315_smolvla_eval.py \
  --checkpoint /home/tiago/room315_local_training/smolvla_runs/v1_pretrained_compact32/checkpoints/040000/pretrained_model \
  --split train \
  --dataset-root /home/tiago/room315_local_training/lerobot/room315_vla_train_compact32 \
  --source-jsonl /home/tiago/room315_local_training/splits/train.jsonl \
  --output-dir /home/tiago/room315_local_training/legacy_direct_action_baseline/smolvla_v1_compact32_040000_invalid \
  --stride 8 \
  --progress-every 64
```

The SmolVLA train command used `--stride 8`, matching the historical train evaluation size of 285 samples. Val/test used all rows.

## Checkpoints

| Checkpoint | Validity | Main fingerprint |
|---|---|---|
| `/home/tiago/room315_local_training/checkpoints/v0/best.pt` | valid-for-comparison | checkpoint sha256 `fb658c340732b08902c5f3d94f48ba1788a9e83583b40414ac353f2acd312f7b`, 1705065 bytes |
| `/home/tiago/room315_local_training/checkpoints/smoke/best.pt` | valid-for-comparison smoke only | checkpoint sha256 `62d477c452938e90b777ecd722dfb94037cf93ef31e2de950281a9db8adb9652`, 1640553 bytes |
| `/home/tiago/room315_local_training/smolvla_runs/v1_pretrained_compact32/checkpoints/040000/pretrained_model` | invalid-for-comparison | `model.safetensors` sha256 `106c30eca57e602e82d2e6ad3478d5c8922f62f3ebeae1744b3ae8c34fcaef2b`, 1197789224 bytes |

Local `v0` artifact fingerprints:

| Artifact | sha256 | Bytes |
|---|---|---:|
| `state_vectorizer.json` | `23406df37116e2d63896d384c681375bd9ef81d021dfed8579d376632b6c63ae` | 5703 |
| `target_stats.json` | `9a004c05702aa9c447b471c426fb588db78ef27da858213ad98667388c5fb842` | 932 |
| `training_config.json` | `488da52f2809c1663473d774573d7e4964d2e30ec2e2be4a36e34b53e4c9078c` | 511 |
| `vocab.json` | `6b360f2ad10a8fb1440146fa394b3ba2798376d452992f8648a968b4a2f236b8` | 171 |

SmolVLA compact32 leakage:

| Artifact | sha256 | Finding |
|---|---|---|
| `/home/tiago/room315_local_training/lerobot/room315_state_vectorizer_compact32.json` | `71eeee19048e66e9eccb1a40e8790c3e23b98c15283c60f49dd601619865836a` | invalid-for-comparison; includes `payload_present` and `step_index_norm` |

## Local Results

`v0/best.pt`, valid-for-comparison:

| Split | Rows | Exact action | Primitive | Target id | Schema legal | Supervisor rejection | p50 infer s | p95 infer s | Total cycle s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| train | 2280 | 0.519737 | 0.984211 | 0.656579 | 0.964912 | 0.035088 | 0.000417 | 0.000625 | 10.572737 |
| val | 344 | 0.514535 | 0.988372 | 0.648256 | 0.965116 | 0.034884 | 0.000418 | 0.000558 | 1.500418 |
| test | 256 | 0.523438 | 0.957031 | 0.652344 | 0.937500 | 0.062500 | 0.000417 | 0.000592 | 1.124711 |

Aggregate: 2880 samples, exact action `0.519444`, primitive `0.982292`, target id `0.655208`, schema decode `1.0`, schema legal `0.9625`, supervisor rejection `0.0375`, p50 inference `0.000418s`, p95 inference `0.000608s`, total cycle `13.197866s`, peak CUDA allocated `10986496` bytes.

`smoke/best.pt`, valid-for-comparison but smoke only:

| Split | Rows | Exact action | Primitive | Target id | Schema legal | Supervisor rejection | p50 infer s | p95 infer s | Total cycle s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| train | 2280 | 0.000000 | 0.364912 | 0.000000 | 1.000000 | 0.000000 | 0.000422 | 0.000621 | 10.631889 |
| val | 344 | 0.000000 | 0.372093 | 0.000000 | 1.000000 | 0.000000 | 0.000424 | 0.000669 | 1.590012 |
| test | 256 | 0.000000 | 0.375000 | 0.000000 | 1.000000 | 0.000000 | 0.000422 | 0.000589 | 1.119950 |

## Local Per-Family Metrics

`v0/best.pt` validation families:

| Family | Samples | Exact action | Primitive | Target id | Schema legal | Reject |
|---|---:|---:|---:|---:|---:|---:|
| `left_four_shuttles_loaded_l4_s4_to_slot1_clear_l2_interior_l1_s2` | 84 | 0.630952 | 1.000000 | 0.797619 | 1.000000 | 0.000000 |
| `left_loaded_l1_s1_blocker_l2_s2_interior_to_slot3` | 68 | 0.323529 | 1.000000 | 0.470588 | 0.941176 | 0.058824 |
| `right_four_shuttles_loaded_r2_s4_to_slot1_clear_r3_interior_r1_s2` | 84 | 0.571429 | 1.000000 | 0.630952 | 1.000000 | 0.000000 |
| `right_four_shuttles_loaded_r3_s1_to_slot3_clear_r1_a4_r4_s4_r2_interior` | 108 | 0.500000 | 0.962963 | 0.657407 | 0.925926 | 0.074074 |

`v0/best.pt` test families:

| Family | Samples | Exact action | Primitive | Target id | Schema legal | Reject |
|---|---:|---:|---:|---:|---:|---:|
| `left_four_shuttles_loaded_l1_s4_to_slot1_clear_l3_interior_l2_s2` | 84 | 0.464286 | 0.928571 | 0.619048 | 0.952381 | 0.047619 |
| `left_loaded_l1_s2_blocker_l2_s3_clear_s1_to_slot3` | 44 | 0.522727 | 1.000000 | 0.590909 | 0.909091 | 0.090909 |
| `right_four_shuttles_loaded_r1_s1_to_slot3_clear_r4_a4_r3_s4_r2_interior` | 108 | 0.555556 | 1.000000 | 0.694444 | 0.962963 | 0.037037 |
| `right_loaded_r2_s2_to_slot3_no_blocker` | 20 | 0.600000 | 0.750000 | 0.700000 | 0.800000 | 0.200000 |

The full 32-family train table is in `/home/tiago/room315_local_training/legacy_direct_action_baseline/local_v0_best/legacy_direct_action_eval.json`.

## SmolVLA Results

All v1 compact32 SmolVLA results below are invalid-for-comparison because the saved LeRobot state vectorizer includes row-level leakage features: `payload_present` and `step_index_norm`.

Checkpoint 040000, updated extended metrics:

| Split | Samples | Stride | Exact action | Primitive | Target id | Schema legal | Supervisor rejection | p50 infer s | p95 infer s | Total cycle s | Peak CUDA bytes |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| train | 285 | 8 | 0.182456 | 0.684211 | 0.375439 | 1.000000 | 0.000000 | 0.229296 | 0.237904 | 69.467722 | 1268146176 |
| val | 344 | 1 | 0.180233 | 0.639535 | 0.404070 | 1.000000 | 0.000000 | 0.229826 | 0.238214 | 80.845479 | 1268146176 |
| test | 256 | 1 | 0.187500 | 0.667969 | 0.378906 | 1.000000 | 0.000000 | 0.229897 | 0.238482 | 60.157236 | 1268146176 |

Checkpoint 040000 test families:

| Family | Samples | Exact action | Primitive | Target id | Schema legal | Reject |
|---|---:|---:|---:|---:|---:|---:|
| `left_four_shuttles_loaded_l1_s4_to_slot1_clear_l3_interior_l2_s2` | 84 | 0.190476 | 0.642857 | 0.392857 | 1.000000 | 0.000000 |
| `left_loaded_l1_s2_blocker_l2_s3_clear_s1_to_slot3` | 44 | 0.204545 | 0.863636 | 0.409091 | 1.000000 | 0.000000 |
| `right_four_shuttles_loaded_r1_s1_to_slot3_clear_r4_a4_r3_s4_r2_interior` | 108 | 0.138889 | 0.583333 | 0.324074 | 1.000000 | 0.000000 |
| `right_loaded_r2_s2_to_slot3_no_blocker` | 20 | 0.400000 | 0.800000 | 0.550000 | 1.000000 | 0.000000 |

Historical SmolVLA v1 compact32 trend, also invalid-for-comparison:

| Checkpoint | Split | Samples | Exact action | Primitive | Target id |
|---|---|---:|---:|---:|---:|
| 002000 | train | 285 | 0.007018 | 0.343860 | 0.133333 |
| 002000 | val | 344 | 0.008721 | 0.369186 | 0.165698 |
| 002000 | test | 256 | 0.027344 | 0.386719 | 0.156250 |
| 010000 | val | 344 | 0.093023 | 0.470930 | 0.287791 |
| 010000 | test | 256 | 0.074219 | 0.468750 | 0.261719 |
| 020000 | val | 344 | 0.116279 | 0.552326 | 0.325581 |
| 020000 | test | 256 | 0.121094 | 0.625000 | 0.296875 |
| 040000 | val | 344 | 0.180233 | 0.639535 | 0.404070 |
| 040000 | test | 256 | 0.187500 | 0.667969 | 0.378906 |

Available SmolVLA checkpoint directories include `000250` through `040000`, plus smoke runs. The v1 compact32 lineage shares the leaked compact32 state vectorizer, so earlier and intermediate checkpoints are invalid for comparison unless regenerated from a clean LeRobot conversion.

## Leakage-Free Retraining Command

These commands were prepared, not run in this audit. They intentionally start from `lerobot/smolvla_base`, not from any leaked v1 compact32 checkpoint.

```bash
/home/tiago/room315_local_training/venv/bin/python mfja_robot_control_config/scripts/room_315_vla_to_lerobot.py \
  /home/tiago/room315_local_training/splits/train.jsonl \
  --dataset-root /home/tiago/room315_payload_expanded_160_merged_final \
  --output-root /home/tiago/room315_local_training/legacy_direct_action_baseline/lerobot_clean \
  --name room315_vla_train_compact32_clean \
  --repo-id room315/room315_vla_train_compact32_clean \
  --state-mode compact32 \
  --state-vectorizer-out /home/tiago/room315_local_training/legacy_direct_action_baseline/lerobot_clean/room315_state_vectorizer_compact32_clean.json \
  --image-width 320 \
  --image-height 240 \
  --overwrite
```

```bash
/home/tiago/room315_local_training/venv/bin/python mfja_robot_control_config/scripts/room_315_vla_to_lerobot.py \
  /home/tiago/room315_local_training/splits/val.jsonl \
  --dataset-root /home/tiago/room315_payload_expanded_160_merged_final \
  --output-root /home/tiago/room315_local_training/legacy_direct_action_baseline/lerobot_clean \
  --name room315_vla_val_compact32_clean \
  --repo-id room315/room315_vla_val_compact32_clean \
  --state-mode compact32 \
  --state-vectorizer /home/tiago/room315_local_training/legacy_direct_action_baseline/lerobot_clean/room315_state_vectorizer_compact32_clean.json \
  --image-width 320 \
  --image-height 240 \
  --overwrite
```

```bash
/home/tiago/room315_local_training/venv/bin/python mfja_robot_control_config/scripts/room_315_vla_to_lerobot.py \
  /home/tiago/room315_local_training/splits/test.jsonl \
  --dataset-root /home/tiago/room315_payload_expanded_160_merged_final \
  --output-root /home/tiago/room315_local_training/legacy_direct_action_baseline/lerobot_clean \
  --name room315_vla_test_compact32_clean \
  --repo-id room315/room315_vla_test_compact32_clean \
  --state-mode compact32 \
  --state-vectorizer /home/tiago/room315_local_training/legacy_direct_action_baseline/lerobot_clean/room315_state_vectorizer_compact32_clean.json \
  --image-width 320 \
  --image-height 240 \
  --overwrite
```

```bash
/home/tiago/room315_local_training/venv/bin/lerobot-train \
  --dataset.repo_id room315/room315_vla_train_compact32_clean \
  --dataset.root /home/tiago/room315_local_training/legacy_direct_action_baseline/lerobot_clean/room315_vla_train_compact32_clean \
  --policy.type smolvla \
  --policy.pretrained_path lerobot/smolvla_base \
  --policy.max_state_dim 32 \
  --policy.max_action_dim 32 \
  --policy.device cuda \
  --policy.use_amp true \
  --policy.freeze_vision_encoder true \
  --policy.train_expert_only true \
  --policy.train_state_proj true \
  --batch_size 1 \
  --steps 40000 \
  --save_freq 5000 \
  --eval_freq 20000 \
  --output_dir /home/tiago/room315_local_training/smolvla_runs/v1_pretrained_compact32_clean \
  --seed 1000 \
  --wandb.enable false
```

## Acceptance Criteria

| Criterion | Result |
|---|---|
| Freeze direct-action baseline as `legacy_direct_action_baseline` | Passed. The new metrics helper and reports use this workflow id; no closed-loop behavior is claimed. |
| Evaluate local checkpoints on clean train/val/test manifests | Passed. `v0/best.pt` and `smoke/best.pt` ran on all three JSONLs. |
| Evaluate SmolVLA latest checkpoint on clean train/val/test manifests | Partial. Checkpoint 040000 ran on val/test full and train stride-8; results are invalid-for-comparison due leakage. |
| Per-family metrics | Passed for evaluator outputs. Validation/test tables are shown above; full train family metrics are in generated JSON artifacts. |
| Action legality after schema decoding | Passed. Added schema decode and schema legality rates. |
| Supervisor rejection rate | Passed as an offline schema-v3 proxy. Live `Room315VlaSupervisor` was not executed. |
| p50/p95 inference latency | Passed for local and SmolVLA updated runs. |
| Total cycle time | Passed as summed per-sample dataset load, preprocessing, inference, postprocessing, and metric time. |
| Peak GPU memory | Passed for CUDA runs. |
| Leaked-feature checkpoints labeled invalid | Passed. SmolVLA v1 compact32 is explicitly invalid-for-comparison. |
| No fabricated or silently replaced results | Passed. Historical SmolVLA metrics are labeled historical and invalid; clean retraining is a command, not a substituted result. |

## Limitations

- The local `v0` and `smoke` training configs predate the hardened `dataset_report` field, although their saved state vectorizers do not contain the known leakage fields.
- Supervisor rejection is an offline schema-v3 proxy. It catches malformed decoded actions such as selected switches/stoppers with unchanged values, but it is not a live rail-state safety verdict.
- SmolVLA compact32 results are invalid-for-comparison because `observation.state` includes `payload_present` and `step_index_norm`.
- The SmolVLA train split was sampled with `--stride 8`; val/test were full split passes.
- The baseline is a direct-action behavior-cloning comparison only. It does not satisfy `goal -> ObservedState -> PlanSys2 -> one action -> safety -> execute -> re-observe/replan`.
