# MFJA internship report

This directory contains the English LaTeX report:

> *From a Multi-Robot Digital Twin to Safety-Gated Neuro-Symbolic
> Language-to-Motion Control*

The report covers the complete internship work from the third-floor digital
twin and heterogeneous robot integration through the Room 315 rail system,
typed interfaces, safety, PDDL/PlanSys2, VLA evolution, visual training,
experiments, limitations and handover.

## Build

Requirements: `pdflatex`, `bibtex`, and the LaTeX packages loaded by
`preamble.tex`.

```bash
cd /home/tiago/mfja_3rd_floor_ros2_ws/src/mfja_3rd_floor_gz/report
make
```

The output is `main.pdf`; the submission copy is
`Ali_IBRAHIM_MFJA_Internship_Report_2025-2026.pdf`. `make check` also reports
the page count and rejects undefined citations/references. The report is
written in English and describes the final retained implementation. Active
experimental records and their claim boundaries are indexed in
`evidence/ACTIVE_EVIDENCE.md`. A reproducible handover must archive or commit
the exact source and configuration identity used for its test results.
The configured visual checkpoint remains bound to its manifest and content
hash; this establishes no institutional, physical-actuation or machine-safety
authority.

Large datasets, checkpoints and raw campaign bags are not tracked in this
directory. Their release locations, hashes and metrics are documented in the
report, repository runbooks and `evidence/ACTIVE_EVIDENCE.md`. The complete
Room 315 campaign is published under the `room315-evidence-v2` release tag.
