# MFJA internship report

This directory contains the English LaTeX report:

> *From a Multi-Robot Digital Twin to Safety-Gated Neuro-Symbolic
> Language-to-Motion Control*

The report covers the complete internship work from the third-floor digital
twin and heterogeneous robot integration through the Room 315 rail system,
typed interfaces, safety, PDDL/PlanSys2, VLA design, visual training,
experiments, limitations and handover.

## Build

Requirements: `pdflatex`, `bibtex`, and the LaTeX packages loaded by
`preamble.tex`.

```bash
cd <repository-root>/report
make
```

The output is `main.pdf`; the submission copy is
`Ali_IBRAHIM_MFJA_Internship_Report_2025-2026.pdf`. `make check` also reports
the page count and rejects undefined citations/references. The report is
written in English and describes the final retained implementation. Active
experimental records and their claim boundaries are indexed in
`evidence/ACTIVE_EVIDENCE.md`. A reproducible handover must archive or commit
the exact source and configuration identity used for its test results.
The final 12 August build is 87 A4 pages; `main.pdf` and the submission copy are
byte-identical at SHA-256
`0181723704cc77675768e946ccfd325c37b4c882f98a2d351c8cdb62e3698f00`.
The retained log and complete fingerprint record are
`evidence/final_report_build_2026-08-12.log` and
`evidence/final_report_sha256_2026-08-12.txt`.
The repository revision carrying this report commits the evaluated source,
active and recovery configurations, tests, submission PDF and lightweight
evidence together. A clean checkout of that revision reconstructs the
source-side handover; the checkpoint, sealed dataset and raw ROS bags remain
external hash-identified objects.
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
hash-identified objects; all reported runtime authority is Gazebo-only.

Large datasets, checkpoints and raw campaign bags are not tracked in this
directory. Their identifiers, hashes and metrics are documented in the report,
the relative evidence manifests, repository runbooks and
`evidence/ACTIVE_EVIDENCE.md`. The `room315-evidence-v2` release is retained for
archival reconstruction and does not support the current closed-loop claims.
