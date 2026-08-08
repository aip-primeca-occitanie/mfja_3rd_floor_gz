# Final English-report revision log

## Scope and preservation

- Only the English report under `report/` was revised; `report_ar/` was not
  modified.
- The English report source and lightweight evidence are versioned in this
  repository; `report_ar/` remains excluded.

## Evidence added or refreshed

| Evidence | Verified outcome | Report use |
|---|---|---|
| Integrated Room 315 campaign | 12/12 isolated cold starts, 24/24 step postconditions, 48/48 supervisor decisions, 24 plan attempts, four occupancy-triggered replan events, zero unknown retries and zero safe aborts | Primary positive end-to-end evidence |
| Pinned language-contract evaluation | 10/10 declared final outcomes, 9/9 non-control backend invocations, 8/9 strict envelopes without fallback, one expected strict-schema rejection, cancellation without inference and zero unsafe automatic resolutions | Bounded language-interface evidence |
| Current-source pytest suites | 190/190 focused checks, 96/96 transport checks and 42/42 operator-capability checks; overlapping suites are reported separately | Component and controlled-integration evidence |
| Full-floor all-robot smoke | Six configured models, six command topics, six feedback topics and fresh state samples in one isolated cold start; no motion command | O1 launch/interface evidence |
| Visual-model evidence | Capture audits, grouped split and leakage records, training sidecars, raw locked-Test output and final review | Dataset, training and Test-metric provenance |
| B03 camera figure | Initial/final frames extracted from the preserved MCAP with timestamp and checksum provenance | Direct experimental illustration |

Every active result is indexed in `evidence/ACTIVE_EVIDENCE.md`. Each evidence
bundle keeps its protocol or command, raw output, source/configuration identity
and checksums at the level required by its claim.

## Report improvements

- Reframed objectives O1--O4 and connected each objective directly to a
  demonstrated result and its evaluation scope.
- Reworked the abstract, introduction, design narrative, runtime chapter,
  results and conclusion so that they tell the project story without repeating
  the same architecture statement or experimental counts in every chapter.
- Replaced remaining highly parallel summary sentences with shorter project
  narration. Chapter 3 now closes on the concrete rail-name and slot-arrival
  lessons, and the conclusion states the author's own learning outcome instead
  of ending with a generic methodological slogan.
- Kept the detailed command and state rules in Chapter 8, used Chapter 10 for
  one concrete request-to-motion case, and concentrated measurements in Chapter
  11 and reproducibility details in Appendix A.
- Simplified the project-progression, software-check, runtime, campaign-summary
  and evaluation-layer figures for readable A4 presentation; retained the B03
  camera comparison as a direct experimental illustration.
- Replaced unsupported or overly broad wording with descriptions that match the
  all-robot launch check, language evaluation, controller readings and planned
  campaign cases.
- Reduced repeated assurance terminology throughout the main body. Terms such
  as `fail-closed` remain only where they name a specific implemented policy;
  detailed result provenance remains in the reproducibility appendix.
- Replaced repeated defensive formulations such as “does not demonstrate” and
  “not a reliability estimate” with positive descriptions of what each
  experiment covered. Detailed limits now appear once in Chapter 12, while the
  result chapter retains only the denominators and conditions needed to
  interpret 12/12, 10/10 and the all-robot launch check correctly.
- Reworked Figure 8.2 and the campaign/evaluation figures to describe data flow
  and coverage directly instead of repeating no-actuation or claim-boundary
  warnings. Appendix A is now a reproducibility record, and Appendix B replaces
  the authority/forbidden-substitution matrix with a concise responsibility
  summary.
- Aligned the English Abstract and French Résumé by reporting the visual model's
  65.2985% exact-block accuracy in both summaries.
- Split the glossary page into clearly labelled acronym and project-specific
  term sections so that each list has an explicit organising principle.
- Removed the word “content” from the generative-AI declaration while retaining
  the stated review responsibility for AI-assisted code, figures and suggestions.
- Added one direct future-work sentence to the Conclusion, linking the project
  outcome to the controlled-seed and live-graph fault-injection priorities in
  Chapter 12 while keeping the proposed extension within simulation.
- Kept limitations concise and simulation-focused while presenting concrete
  continuation experiments.
- Completed the identification page, supervisor contact information,
  generative-AI declaration and citations required by the university guidance.
- Rewrote the acknowledgements in a personal first-person voice and stated the
  specific contribution of the scientific supervisors, academic supervisor and
  MFJA support staff.
- Added primary references for Qwen2.5 and the official `llama.cpp` software
  source.
- Added a compact Chapter 4 comparison of end-to-end VLA policies,
  language/vision models combined with external skills or planning,
  neuro-symbolic planning and the implemented Room 315 architecture. The table
  links each alternative to the available training data and the fixed rail
  topology rather than presenting the final design as the only possible option.
- Clarified in the Abstract that Phase 2 was inspired by VLA research but the
  implemented result is a neuro-symbolic language-and-vision interface, not an
  end-to-end VLA policy. Misleading shorthand such as “VLA cameras,” “VLA
  operation” and “VLA supervisor” was removed from descriptive prose; the
  historical executable filename remains explicitly identified as such.
- Replaced the matrix-led engineering-qualities discussion with two concise
  first-person case studies covering initiative, autonomy, weekly supervisory
  review, resource constraints, controlled migration and professional learning;
  retained one competency table as a synthesis.
- Corrected the rail-topology evidence boundary: the configured anchor
  tolerance is documented but not enforced by the runtime loader, whose legal
  connectivity comes from explicit routing tables.

## Final validation contract

The final source freeze is accepted only after `make check` reports an A4 PDF
with no undefined citations or references and no overfull boxes. The retained
build log is listed in `evidence/ACTIVE_EVIDENCE.md`; the named delivery PDF is
copied from the same validated `main.pdf`.

Final validation completed on 8 August 2026: 73 A4 pages, no undefined
citations or references, and no overfull boxes. The delivery PDF and `main.pdf`
are byte-identical and share SHA-256
`026e3ebd9b1a69159f0c12b59101bccae5f46fd18f6ff60584abe1387414eae0`.
