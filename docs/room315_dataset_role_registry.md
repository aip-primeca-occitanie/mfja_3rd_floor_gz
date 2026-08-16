# Room 315 visual-dataset role and provenance registry

Status date: 12 August 2026. This registry records the roles that were actually
implemented. It does not rename or rewrite any historical dataset, checkpoint,
log, or sealed Final-Test artefact.

## Naming rule

Use these names in new documentation:

- **Historical grouped predecessor validation** for the 256-row file still
  physically named `validation.jsonl` in the legacy grouped package.
- **Consumed historical grouped Test** for the 256-row file still physically
  named `test.jsonl`. It was evaluated once on 30 July 2026 and is not unseen or
  currently locked.
- **Hard-case Validation** for the 512-scene active V4 development validation.
- **Development Canary** for the 256-scene post-selection regression set.
- **Preregistered Final Test** for the completed 1,040-scene one-shot evidence
  set.

Do not rename the materialized files: their names and hashes are part of the
experimental record.

## Canonical roles

| Dataset or partition | Rows | Present targets | Unique image contents | Historical implemented role | Current implemented role | Status |
|---|---:|---:|---:|---|---|---|
| Legacy Training | 1,528 | 6,032 | 2,809 | Predecessor training | Direct V4 replay-training source | Active training input |
| Historical grouped predecessor validation | 256 | 1,088 | 512 | Per-epoch validation, smoke authorization, epoch/checkpoint selection, patience | Not loaded directly; indirectly selected the predecessor backbone used to initialize V4 | Historical/dormant |
| Consumed historical grouped Test | 256 | 1,072 | 512 | One explicit post-selection evaluation and historical acceptance review | Data unused and denylisted | Historical/consumed |
| Hard-case Training | 4,000 | 17,620 | 7,730 | Later V3R1 training | Direct V4 training source | Active training input |
| Hard-case Validation | 512 | 2,254 | 889 | Later V3R1 development validation | Per-epoch selection, patience, post-selection validation, temperature fit, selective-threshold fit, and 76 validation gates | Active development input |
| Development Canary | 256 | 1,085 | 458 | Post-selection regression reserve | Post-selection regression and 74 promotion gates; reuses validation temperature and does not refit it | Active development/promotion input |
| Preregistered Final Test | 1,040 | 4,680 | 2,080 | V4 one-shot final protocol | Frozen-checkpoint final acceptance and report evidence | Consumed final evidence |

“Unique image contents” counts unique SHA-256 values rather than path strings.
Some generated variants intentionally share one rendered camera image within a
partition.

## Decision-influence matrix

| Partition | Gradient updates | Evaluated during model development | Selects checkpoint | Controls patience/early stop | Fits temperature | Fits confidence threshold | Other implemented decision |
|---|---|---|---|---|---|---|---|
| Legacy Training | Yes, historically and as V4 replay | Yes | No | No | No | No | Fits training-side normalization/vectorizer state in the predecessor protocol |
| Historical grouped predecessor validation | No | Yes, historically | Yes: predecessor epoch 14 | Yes; configured, but the completed 15-epoch run did not stop early | No | No | Authorized full legacy training after smoke validation; indirectly determines the V4 backbone initialization lineage |
| Consumed historical grouped Test | No | No before selection | No | No | No | No | Produced one historical acceptance review and deployment constraints after selection |
| Hard-case Training | Yes | Yes | No | No | No | No | Supplies the current hard-case gradient source |
| Hard-case Validation | No | Yes | Yes: current V4 epoch 11 | Yes; configured, but the completed 12-epoch run did not stop early | Yes | Yes: deployed segment-confidence floor | Produces current validation metrics and acceptance gates |
| Development Canary | No | Yes, only after selection/calibration | No | No | No; reuses the validation value | No | Can block/manual-gate promotion, without modifying model parameters |
| Preregistered Final Test | No | No | No | No | No | No | Produces the current one-shot final acceptance decision and report evidence |

There is no executed automated architecture or hyperparameter search attached
to either legacy 256-row partition. The predecessor command executed only the
`partial_finetune` variant. Later architecture choices may have been informed by
earlier results as human engineering history, but that is not an implemented
selection algorithm and should not be reported as one.

## Creation and consumption provenance

### Legacy grouped protocol

The legacy capture used seed `31520260729`. The grouped splitter used seed
`31520260730` and assigned whole image-connected presence components to
191/32/32 configurations, yielding 1,528/256/256 rows.

- Definition and writer:
  [`room_315_grouped_visual_splits.py`](../mfja_robot_control_config/scripts/room_315_grouped_visual_splits.py), especially
  `assign_components()`, `_split_sets()`, `leakage_audit()`, and
  `create_package()`.
- Materialized package:
  `~/room315_arbitrary_subset_visual_splits_v1_seed31520260730` by default;
  the audit command accepts a relocated root.
- Predecessor loading/selection:
  [`train_visual_state()`](../mfja_robot_control_config/scripts/room_315_vla_train_local.py)
  loads training and validation, evaluates validation each epoch, saves the
  lowest-validation-loss checkpoint, and advances patience.
- Explicit historical Test unlock:
  `~/room315_test_evaluation_approved_archive_seed31520260730/results/test_access_log.json`
  by default.
- Historical selected checkpoint: epoch 14, SHA-256
  `8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d`.
- V4 lineage:
  [`initialize_v4_backbone_from_v3_model_state_dict()`](../mfja_robot_control_config/scripts/room_315_visual_model_v4.py)
  copies the selected predecessor backbone but not its heads.

### Current development protocol

Hard Training, Hard-case Validation, and Canary use generation seed
`31520260730`. This is a reuse of the numeric legacy split seed, not a reuse of
the same generator namespace or exact generated samples.

- Counts and generation:
  [`room_315_visual_v3r1_common.py`](../mfja_robot_control_config/scripts/room_315_visual_v3r1_common.py)
  and
  [`room_315_visual_v3r1_generator.py`](../mfja_robot_control_config/scripts/room_315_visual_v3r1_generator.py).
- Implemented source boundary:
  [`room_315_visual_v3r1_splitter.py`](../mfja_robot_control_config/scripts/room_315_visual_v3r1_splitter.py)
  imports legacy **Training** only. It does not import either legacy 256-row
  partition.
- Current V4 source/role configuration:
  [`visual_state_training_v4.json`](../mfja_robot_control_config/config/room_315_vla/visual_state_training_v4.json).
- Checkpoint selection and post-selection Canary flow:
  [`room_315_vla_train_v4.py`](../mfja_robot_control_config/scripts/room_315_vla_train_v4.py).
- Validation-only calibration:
  [`room_315_visual_calibration_v4.py`](../mfja_robot_control_config/scripts/room_315_visual_calibration_v4.py).
- Legacy-Test denylist:
  [`room_315_visual_v3_common.py`](../mfja_robot_control_config/scripts/room_315_visual_v3_common.py).

### Final-evidence protocol

The Final-Test generation seed is `3152026081101`. The original 1,024-scene
plan was extended before inference to the completed 1,040-scene dataset.

- Preregistration/generation:
  [`room_315_visual_v4_final_test.py`](../mfja_robot_control_config/scripts/room_315_visual_v4_final_test.py).
- One-shot evaluator and exposed-Test guards:
  [`room_315_visual_final_test_v4.py`](../mfja_robot_control_config/scripts/room_315_visual_final_test_v4.py).
- Materialized finalized set:
  `~/room315_visual_v4_final_test_seed3152026081101_coveragecompat/finalized`
  by default; the audit command accepts a relocated root.

“Independent” means procedural and exact sample/identifier/fingerprint/image
separation under the frozen protocol. It does not mean a different simulator,
generator domain, or abstract presence-configuration universe.

## Immutable input fingerprints

| Partition | Rows SHA-256 | Labels SHA-256 |
|---|---|---|
| Legacy Training | `beb6618c5c0bee80e7ec78fa7782e6a2b75c4aabf46e5745a97d6e3871a59095` | `0cebc68d99db5e364d0637336244456be05b96edad5f8f176eb0176c7883e583` |
| Historical grouped predecessor validation | `5119937c489fc100c73ca6637697044ee7a3b5e37e8ce76c4c542ed80475858a` | `218039e54ac0abf3de940f8abf92aff8c91a75333d991eb213a73101939fab71` |
| Consumed historical grouped Test | `2fcf78c0034fe290c39b2816e12076300decf5f7818538357fae072231b9b502` | `1dc97b0836f40c53810306e9a09874967fa7e1067cd5de315cba0e00570277e3` |
| Hard-case Training | `396e3b83822dcd2ed541025fc033802592a609e288dbb28be555c6d9f586361c` | `ec98fd5a94ed9d29fbb0b33dbed33877d571d263ea4a497be99d088673b71921` |
| Hard-case Validation | `a4c90ac7c1043450830f69ad90094e9aacac92ad57f24fcd4439b0b2a14c9fd7` | `d62310046e9a6737e69d7d0e702f05e1073ae7de8f12b2c64655d510b410e1ab` |
| Development Canary | `28568e8ebf793e0a0a18ad9327f36639b2fd9c27021b9bccc4b318dd48192541` | `42d1d6ccab49d4a6bfdb2c2b79d77e404e5f9cb23066a816c1a3851d552b02db` |
| Preregistered Final Test | `eee18e9823feb195a831952df90e11f0d05509314f9f020bb8d2e02de705e1a9` | `021fbccd001e7d6657a4c29546ba69140d3d61db2901cc726fb7f8595b5cdbf4` |

## Supplemental cross-protocol overlap result

The reproducible post-hoc audit is
[`overlap_audit.json`](../report/evidence/room315_dataset_partition_overlap_post_hoc_2026-08-12/overlap_audit.json).
It compares all 21 partition pairs after reading every row, label, and image.
It does not modify or replace the preregistered Final-Test disjoint audit.

For each legacy 256-row partition versus Legacy Training, the other legacy
256-row partition, Hard Training, Hard-case Validation, Canary, and Final Test,
the following overlaps are all zero:

- sample, episode, and scenario IDs;
- individual image content and two-camera image-pair content;
- complete `visual_state_labels` target payloads and complete label rows;
- scenario families, geometry fingerprints, and generated-variant
  fingerprints; and
- exact trajectory and exact trajectory-plus-loaded-state fingerprints.

Those dimensions are present and comparable on both sides. Legacy rows do not
carry the later-schema `configuration_family_id`,
`configuration_core_family_id`, or `capture_configuration_fingerprint` fields,
so cross-protocol comparisons for those names are recorded as
`not_comparable_missing_field`, not as zero overlap. `source_scenario_id` is
likewise unavailable for almost all partitions; the primary scenario IDs above
remain fully comparable. The generated-variant fingerprint is still comparable
because its schema-aware payload uses the available legacy `v2_plan_id` and the
later spec/configuration identifiers.

Abstract presence support is not disjoint across protocols:

| Legacy partition compared with | Legacy Training | Other legacy 256 | Hard Training | Hard Validation | Canary | Final Test |
|---|---:|---:|---:|---:|---:|---:|
| Historical grouped predecessor validation | 0/32 | 0/32 | 31/32 | 16/32 | 3/32 | 30/32 |
| Consumed historical grouped Test | 0/32 | 0/32 | 27/32 | 10/32 | 3/32 | 31/32 |

The post-hoc audit also exposes a narrower current-set overlap that the original
V3R1 split audit did not measure:

- Hard Training and Hard-case Validation share 23 unique complete visual-target
  payload hashes, 14 trajectory fingerprints, 14 geometry fingerprints, and 23
  trajectory-plus-loaded-state fingerprints.
- Hard Training and Canary share 14 unique complete visual-target payload
  hashes, eight trajectory fingerprints, eight geometry fingerprints, and 14
  trajectory-plus-loaded-state fingerprints.
- Hard-case Validation and Canary share zero complete visual target payloads or
  exact trajectory/geometry states.

In both non-zero cases, sample/episode/scenario IDs, image contents, complete
label rows, configuration families, and generated-variant fingerprints remain
disjoint. This is **target/state support overlap**, not reuse of the same sample
or image. It must not be summarized as either “all labels are disjoint” or
“samples leaked.”

## Published V4 reproduction companion

The complete post-experiment distribution for the reported V4 procedure is
published from this project's
[`v4-seed31520260811-dataset-v1`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/tag/v4-seed31520260811-dataset-v1)
GitHub Release. Its repository-side control package is retained at
[`report/evidence/room315_visual_v4_dataset_release_v1/`](../report/evidence/room315_visual_v4_dataset_release_v1/README.md).
The release contains the active V4 inputs listed above: 1,528 Legacy Training
plus 4,000 Hard-case Training scenes, 512 Hard-case Validation scenes, 256
Development-Canary scenes and 1,040 Preregistered Final-Test scenes. It also
provides both required checkpoints, the frozen source and stateless evaluation
tooling. The historical grouped predecessor Validation and consumed Test
partitions are not V4 inputs and are not included.

Canary and Final-Test disclosure occurred only after the historical experiment
and its immutable attempts completed; later publication changes availability,
not the roles recorded in this registry. The repository and release assets are
publicly accessible for inspection and reproducibility, but that accessibility
does not make them an open dataset or grant a data or model-weight
reuse licence. Asset counts and SHA-256 values are indexed in
[`ROOM315_VISUAL_V4_DATASET_RELEASE.md`](../report/evidence/ROOM315_VISUAL_V4_DATASET_RELEASE.md).

## Reproduction

From the repository root, with the seven materialized datasets available at the
recorded defaults:

```bash
python3 mfja_robot_control_config/scripts/room_315_dataset_partition_overlap_post_hoc.py
pytest -q mfja_robot_control_config/test/test_room315_dataset_partition_overlap_post_hoc.py
```

Use `--legacy-root`, `--hard-root`, `--canary-root`, `--final-root`, and the
corresponding `--*-dataset-root` arguments for relocated copies. The tool hashes
the image bytes itself and checks any legacy per-image digest declarations. Its
JSON output is deterministic: it contains no run timestamp and sorts every set
before serialization.

This audit is post-hoc documentation. It has no training, selection, early-stop,
calibration, threshold, promotion, acceptance-gate, or final-evidence effect.
