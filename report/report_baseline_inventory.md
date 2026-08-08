# Report Baseline Inventory

> Historical metadata only. The binary baseline artefacts were removed from
> the current worktree on 8 August 2026 to reduce repository size; they remain
> recoverable from earlier Git history.

## Scope and preservation

- Baseline captured before the evidence-first revision.
- Source directory: `report/`.
- The former untouched PDF, rebuilt PDF and build log were removed with the
  other baseline artefacts after final validation.
- Build command: `make check`.
- Build result: success.
- No commit or push was performed.
- The two pre-existing working-tree modifications below belong to the user and
  are outside the report scope:
  - `mfja_robot_control_config/scripts/room_315_pddl_scenario_generator.py`
  - `mfja_robot_control_config/test/test_room315_pddl_scenario_generator.py`

## Baseline pagination

| Region | Physical PDF pages | Count |
|---|---:|---:|
| Front matter and generated lists | 1--13 | 13 |
| Main narrative, Introduction through Conclusion | 14--76 | 63 |
| Appendices A--D | 77--90 | 14 |
| Bibliography | 91--92 | 2 |
| **Total** | **1--92** | **92** |

Appendix A begins on physical page 77. Therefore “pre-appendix” is 76
physical pages: 13 pages of front matter and generated lists followed by
63 pages of main narrative. Physical page 76 is the second page of the
Conclusion, not a separator.

## Baseline chapter starts

| Part | Start page |
|---|---:|
| General Introduction | 14 |
| Chapter 2 | 17 |
| Chapter 3 | 20 |
| Chapter 4 | 24 |
| Chapter 5 | 28 |
| Chapter 6 | 32 |
| Chapter 7 | 35 |
| Chapter 8 | 40 |
| Chapter 9 | 45 |
| Chapter 10 | 49 |
| Chapter 11 | 55 |
| Chapter 12 | 60 |
| Chapter 13 | 66 |
| Chapter 14 | 70 |
| Conclusion | 75--76 |
| Appendix A | 77 |
| Appendix B | 82 |
| Appendix C | 86 |
| Appendix D | 89 |
| Bibliography | 91 |

## LaTeX and bibliography diagnostics

| Check | Baseline result |
|---|---:|
| Undefined references or citations | 0 |
| Missing figure/file errors | 0 |
| Multiply defined labels | 0 |
| BibTeX warnings/errors | 0 |
| Overfull boxes | 9 |
| Underfull boxes | 90 |

The largest reported overfull boxes are in narrow tables or long code
identifiers. They will be assessed visually and corrected where they impair A4
readability; underfull-box counts alone are not treated as defects.

## PDF metadata

| Field | Baseline value |
|---|---|
| Title | From a Multi-Robot Digital Twin to Safe Neuro-Symbolic Language-to-Motion Control |
| Author | Ali IBRAHIM |
| Subject | M2 Industrial Robotics internship report |
| Producer | pdfTeX-1.40.25 |
| Page size | A4, 595.276 x 841.89 pt |
| Pages | 92 |
