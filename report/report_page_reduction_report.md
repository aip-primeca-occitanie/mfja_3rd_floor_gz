# Final Report Synthesis and Page-Reduction Record

## Result

The final synthesis and terminology pass reduced the English report from 88 to
71 A4 pages, a reduction of 17 pages (19.3%). The university guidance requires at least 40
pages of significant documentation and warns that excessive volume may indicate
insufficient synthesis; the final technical narrative remains comfortably above
that minimum.

No font, margin or figure scaling was used to obtain the reduction. The change
came from removing repeated explanations and secondary digest inventories,
consolidating each numerical result at one canonical location, defining opaque
terms concisely at first use, and directing implementation details to the
retained runbook and evidence index.

## Synthesis changes

- The Abstract and French Résumé were reduced to one page each while retaining
  the problem, method, headline measured results and Gazebo-only boundary. The
  generative-AI declaration retained its content; only two spellings were
  standardised to British English at the author's request.
- The visual-model chapter retains the corpus, historical/current partition roles, training, the
  V3-to-V4 design rationale, the final architecture and the one-shot protocol;
  detailed measured results remain canonical in Chapter 11.
- The runtime chapter retains the end-to-end request-to-motion explanation and
  the B03 walkthrough without repeating interface and safety material already
  established in Chapters 7--9.
- Chapter 11 retains the complete campaign matrix, defined metric denominators,
  principal metric tables,
  fault/runtime boundaries and objective assessment. A duplicate campaign
  summary figure and repeated prose were removed.
- Limitations and perspectives retain all scope boundaries and future-work
  priorities. The Conclusion now synthesises the contribution instead of
  repeating the Results chapter.
- Appendix A remains as a two-page reproducibility index. Detailed per-file
  hashes stay in `evidence/ACTIVE_EVIDENCE.md` and the checksum-bound evidence
  package. The former interface appendix is retained as source documentation
  but is no longer compiled because it duplicated the runbook and Chapters
  6--10.

## Final page map

| Region | Final pages |
|---|---:|
| Front matter and generated lists | 1--13 |
| Chapter 1 | 14 |
| Chapters 2--8 | 15--46 |
| Chapter 9 | 47--52 |
| Chapter 10 | 53--57 |
| Chapter 11 | 58--64 |
| Chapters 12--13 | 65--67 |
| Bibliography | 68--69 |
| Appendix A | 70--71 |

## Validation

The final `make check` build succeeds as a 71-page A4 PDF with 25 resolved
bibliography entries, no undefined citations or references and no overfull
boxes. A page-density scan and visual inspection confirmed that the Abstract,
Résumé, glossary, Figure 2.1, metric tables, limitations and reproducibility
index remain readable at A4 print size and that no sparse glossary page remains.
The named delivery PDF is byte-identical to `main.pdf`; both files are
4,687,897 bytes with SHA-256
`32b4d41cd0587c7217096b728ce4133091ec43123cf8997d12e382ea32b38d06`.
