# Post-hoc Room 315 dataset-partition overlap supplement

This directory is a new provenance supplement created on 12 August 2026 after
the listed datasets and the current Final-Test evaluation already existed. It
does not alter, supersede, or form part of any historical or preregistered
evidence bundle.

`overlap_audit.json` compares all 21 pairs among legacy Training, historical
grouped predecessor validation, consumed historical grouped Test, Hard-case
Training, Hard-case Validation, Development Canary, and the Preregistered Final
Test. It deliberately separates:

- exact sample/episode/scenario identifiers;
- image and image-pair content hashes;
- complete visual target payloads versus complete label-row identity;
- generated families, fingerprints, and variants;
- exact trajectory/semantic target states;
- abstract presence-configuration support; and
- reuse of a numeric seed value.

Every pair/metric records extraction coverage. A field that does not exist in
one schema is reported as `not_comparable_missing_field` with a JSON `null`
overlap count; absence is never represented as proven zero overlap.

The audit is documentation only. It was not used for training, checkpoint
selection, early stopping, hyperparameter or architecture selection,
calibration, threshold fitting, promotion, acceptance, or final evidence.

Reproduce it from the repository root with:

```bash
python3 mfja_robot_control_config/scripts/room_315_dataset_partition_overlap_post_hoc.py
```

The command has explicit root overrides for relocated materialized datasets.
See `docs/room315_dataset_role_registry.md` for the role interpretation and
input fingerprints.
