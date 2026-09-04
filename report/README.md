# MFJA internship report

This directory contains the English LaTeX report:

> *Development of a Unified Digital Twin for Multi-Robot Integration,
> Perception, and Neuro-Symbolic Planning*

The report covers the complete internship work from the third-floor digital
twin and heterogeneous robot integration through the Room 315 rail system,
typed interfaces, deterministic safety, PDDL/PlanSys2, separate language and
visual-state models, visual training,
experiments, limitations and handover.

## Build

Requirements: `pdflatex`, `bibtex`, and the LaTeX packages loaded by
`preamble.tex`.

```bash
cd <repository-root>/report
make
```

The output is `main.pdf`; the submission copy is
`IBRAHIM_Ali_Master_ISC_FI_Parcours_RI_RAPPORT_FINAL_le18Aout2026.pdf`.
`make check` also reports
the page count and rejects undefined citations/references. The report is
written in English and describes the final retained implementation. Active
experimental records and their claim boundaries are indexed in
`evidence/ACTIVE_EVIDENCE.md`. A reproducible handover must archive or commit
the exact source and configuration identity used for its test results.
Historical and current visual-dataset terminology is normalized in
`../docs/room315_dataset_role_registry.md`; that registry does not rename the
hash-identified materialized files.
The current build is 73 A4 pages (4,696,675 bytes). `main.pdf` and the
submission copy are byte-identical at SHA-256
`db2130c34d37293647e3ad131449b08203fe974c89cdab1ee796d97bee21d8af`.
The retained log and fingerprint record are
`evidence/final_report_build_2026-08-16.log` and
`evidence/final_report_sha256_2026-08-16.txt`. This rebuilt delivery incorporates
the dataset-role and terminology clarifications, the published V4 full-evidence
identity, the V4-only runtime alignment and the public project
data-and-reproduction release without changing any experiment, checkpoint,
metric or acceptance result.
The repository revision carrying this report commits the evaluated source,
active runtime configurations, tests, submission PDF and lightweight
evidence together. A clean checkout of that revision reconstructs the
source-side handover. The checkpoint, sealed dataset and raw ROS bags are not
stored in Git; their byte-exact V4 copies are published in the full-evidence
release indexed by `evidence/ROOM315_VISUAL_V4_RELEASE.md`.
The configured checkpoint and Gazebo runtime remain bound to their manifests
and content hashes. The task default is fail closed with
`execution_enabled: false`; an operator must explicitly opt in after
reverifying the runtime authorisation. No profile in the report approves
physical deployment or constitutes certified machinery safety.

The primary visual-model record is
`evidence/room315_visual_v4_submission_2026-08-11/`. It contains the relative
manifest, checksums, sealed Final Test controls, immutable one-shot completion
record, the positive and fault campaign summaries, runtime authorisation and
exact claim pointers used by the English report. Its `evidence_manifest.json`,
`SHA256SUMS` and `README.md` have SHA-256 values
`840787b617e0f671628dfc7d8122ff559d76664506ff8f94ac31d043737da446`,
`dec74565a8ccfba57a32d83dfdd03236daeafeff226d5243594e328fa4adc5ab`
and
`33b3a44f2e5e12b34721abcd1b47af609db3b55557c94c6871fa9c6eeb602a3e`.
The checkpoint, large Test payloads and runtime bags are external
hash-identified objects rather than Git blobs. Their published copies are in
the [Room 315 Visual Model V4 full-evidence release](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/tag/room315-visual-v4-evidence-2026-08-11):
the `247,825,706`-byte archive has SHA-256
`35b583baca4f45eed6aad659c253180d00f1a5830ce389266ba714a4445a8ecc`.
It includes the selected checkpoint, sealed 1,040-scene Test, 2,080 images and
18 MCAP recordings. All reported runtime authority remains Gazebo-only.

The complementary, publicly accessible
[Room 315 Visual V4 data-and-reproduction release](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/tag/v4-seed31520260811-dataset-v1)
is published from this source project itself. The release associated with tag
`v4-seed31520260811-dataset-v1` contains all labelled inputs used by the
reported V4 procedure: 5,528 Training,
512 Validation, 256 post-selection-regression and 1,040 Final-Test scenes, together
with the predecessor initialisation checkpoint, selected epoch-11 checkpoint,
frozen source and stateless replay tooling. The six assets total 810,803,201
bytes. Their control package is retained at
`evidence/room315_visual_v4_dataset_release_v1/` and their landing record is
`evidence/ROOM315_VISUAL_V4_DATASET_RELEASE.md`. They were published only after
the historical experiment completed, so availability changes no experimental
role or result. Public accessibility enables inspection and reproduction; it
does not make the material an open dataset or grant a data or model-weight
reuse licence.

Large datasets, checkpoints and raw campaign bags are not tracked in this
directory. Their identifiers, hashes, release location and metrics are
documented in the report, the relative evidence manifests, repository runbooks,
`evidence/ACTIVE_EVIDENCE.md` and
`evidence/ROOM315_VISUAL_V4_RELEASE.md`; the complete reproduction distribution
is indexed by `evidence/ROOM315_VISUAL_V4_DATASET_RELEASE.md`. In the retained
`room315-evidence-v2` release,
`v2` identifies revision 2 of the integrated campaign/evidence package, not the
visual-model generation; that archived campaign used
`room315.visual_state.v3`. The release supports archival reconstruction and
does not support the current closed-loop claims.
