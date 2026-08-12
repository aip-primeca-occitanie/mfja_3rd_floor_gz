# Report Synthesis and Page-Reduction Report

> Historical synthesis note: the counts below describe an earlier 84-page
> intermediate build. The current 82-page submission is governed by
> `report_revision_log.md`, `evidence/ACTIVE_EVIDENCE.md` and the named delivery
> PDF; this file is retained only to document the earlier reduction work.

## Result

The measured PDF was reduced from 93 to 84 A4 pages, a reduction of 9 pages
(9.7%). This is within the requested 8--12-page range and retains all fifteen
technical chapters, the bibliography and the three compiled appendices.

| Region | Before | After | Reduction |
|---|---:|---:|---:|
| Front matter and generated lists | 15 | 13 | 2 |
| Chapter 1 | 5 | 2 | 3 |
| Chapter 3 | 9 | 6 | 3 |
| Chapter 7 | 7 | 6 | 1 |
| Other chapters, bibliography and appendices | 57 | 57 | 0 |
| **Total** | **93** | **84** | **9** |

## Synthesis changes

- Chapter 1 now contains only context/problem, objectives and principal
  contribution, general method and headline results, and a short report map.
  Repository and test counts remain in Chapter 12; the six evidence-source
  groups are consolidated in Appendix A; detailed technical boundaries remain
  in Chapter 14.
- The former Figure 1.1 moved to Chapter 5 and was combined with the focused
  Room 315 view as one two-panel building/cell figure.
- Chapter 3's obstacle--resolution table is now the sole stage-by-stage
  synthesis. The six narrative restatements, a duplicate chronology table and
  repeated final-status material were removed; technical details are reached
  through chapter cross-references.
- Figure 7.1 was reduced, given a shorter caption and fixed in the surrounding
  discussion so it no longer occupies a sparse figure-only page.
- The Abstract is 292 words and the French Résumé is 316 words. They retain
  the problem, method, three contributions, two strongest measured outcomes,
  the simulation-only boundary, the evaluated research-candidate status and
  institutional value without dataset or test inventories. Candidate
  classification does not activate the checkpoint or change the default
  configuration and establishes no physical-actuation or machine-safety authority.
- The Conclusion remains 422 words after adding the follow-up manual outcome;
  it is still 18.1\% shorter than the original 515-word version.
  Detailed dataset, test and checkpoint-governance inventories are referenced to Chapter
  12 rather than repeated.
- The final supplementary manual result is a follow-up campaign covering four
  distinct goal families and five successful runs out of five. The two
  quantitative-result graphics were combined to absorb this evidence without
  creating a sparse figure page.
- Bibliography now appears directly after the Conclusion, followed by
  Appendices A--C.

## Final page map

| Region | Final pages |
|---|---:|
| Front matter | 1--13 |
| Chapter 1 | 14--15 |
| Chapters 2--15 | 16--75 |
| Bibliography | 76--77 |
| Appendix A | 78--80 |
| Appendix B | 81--82 |
| Appendix C | 83--84 |

## Validation

The final `make` build succeeds. The PDF is A4 and has resolved citations and
references, correctly ordered table-of-contents entries, no duplicate
destinations, no overfull boxes and no LaTeX errors. A page-density scan found
no remaining low-text narrative or isolated-figure page between the
Introduction and Conclusion.
