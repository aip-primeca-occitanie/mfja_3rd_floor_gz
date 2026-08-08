# Preserved visual-model evaluation records

The root JSON records are byte-for-byte copies of the approved Room 315
visual-state evaluation and grouped-split audits used by the report. The
`training_sidecars` directory preserves the small configuration, metric,
metadata, target, vectoriser, dataset-audit and preflight records that support
the reported training selection. The `capture_audits` directory records the
approved 2,040-scenario production capture and camera/box checks, while
`test_evaluation/visual_state_eval.json` retains the detailed locked-Test
denominators and per-field measurements behind the final review. These files
are kept with the report so the stated dataset partition, leakage checks,
configured-checkpoint identity and locked-Test metrics can be reviewed without
depending on the original absolute host paths.

Original sources:

- `/home/tiago/room315_test_evaluation_approved_archive_seed31520260730/final_test_evaluation_review.json`
- `/home/tiago/room315_kairos_visual_state_training_v1_seed31520260730/dataset/splits/leakage_audit.json`
- `/home/tiago/room315_kairos_visual_state_training_v1_seed31520260730/dataset/splits/split_manifest.json`
- `/home/tiago/room315_arbitrary_subset_visual_2040_capture_v2_seed31520260729/`
- `/home/tiago/room315_test_evaluation_approved_archive_seed31520260730/results/metrics/visual_state_eval.json`

The final review records an accepted evaluation of checkpoint
`8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d`.
The split manifest records 1,528 training, 256 validation and 256 locked-Test
scenarios. The leakage audit records 2,040 assigned and 2,040 unique scenarios,
with zero overlap for every declared hard-overlap field.
