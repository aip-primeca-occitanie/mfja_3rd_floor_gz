# Final English-report revision log

## Scope and preservation

- Only the English report under `report/` was revised; `report_ar/` was not
  modified.
- The English report source and lightweight evidence are versioned in this
  repository; `report_ar/` remains excluded.

## Evidence added or refreshed

| Evidence | Verified outcome | Report use |
|---|---|---|
| V4 positive Room 315 campaign | 12/12 isolated cold starts, 1,784 V4 and zero V3 observations, 24/24 step postconditions, 48 supervisor decisions, 24 plan attempts, four occupancy-triggered replans, zero safe aborts and every controller stopped | Primary positive closed-loop evidence |
| V4 closed-loop fault campaign | F01--F05 passed 5/5, 424 V4 and zero V3 observations, zero false success and every final controller disabled | Bounded live fault evidence |
| Final V4 Gazebo runtime | `active_closed_loop_runtime` under `gazebo_v4_closed_loop_runtime_only`; manually approved, automatic promotion disabled and physical deployment not approved | Hash-bound active simulation profile |
| Post-promotion B01 smoke | One selected case passed with 88 V4 and zero V3 observations, one actuating step and postcondition, two accepted supervisor decisions, verified terminal/final effect and stopped controller | Partial confirmation of the final bundle; not a replacement for 12/12 |
| Pinned language-contract evaluation | 10/10 declared final outcomes, 9/9 non-control backend invocations, 8/9 strict envelopes without fallback, one expected strict-schema rejection, cancellation without inference and zero unsafe automatic resolutions | Bounded language-interface evidence |
| Recorded 7 August pytest suites | 190/190 focused checks, 96/96 transport checks and 42/42 operator-capability checks; overlapping suites are reported separately | Component and controlled-integration evidence for their recorded source scope |
| Full-floor all-robot smoke | Six configured models, six command topics, six feedback topics and fresh state samples in one isolated cold start; no motion command | O1 launch/interface evidence |
| V4 visual-model evidence | Validation-only selection records, sealed 1,040-scene dataset controls, frozen one-shot contract, immutable completion ledger and Final Test result | Primary visual Test provenance; V3 retained only for history and rollback |
| V4 B03 camera figure | Initial/final V4-bound frames extracted from the preserved MCAP with timestamp and checksum provenance | Direct experimental illustration |
| Active and rollback defaults | V4 is selected by both checked-in defaults; task execution remains fail closed until explicit launch opt-in; named V3 rollback files are preserved | Current selection and recoverable predecessor path |

Every active result is indexed in `evidence/ACTIVE_EVIDENCE.md`. Each evidence
bundle keeps its protocol or command, source/configuration identity and
checksums at the level required by its claim; deliberately excluded raw objects
remain identified by their source manifests and hashes.

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
- Aligned the English Abstract and French Résumé around the same V4 Final Test
  headline and synthetic/Gazebo-only scope.
- Split the glossary page into clearly labelled acronym and project-specific
  term sections so that each list has an explicit organising principle.
- Preserved the generative-AI declaration verbatim; this revision does not
  alter its wording.
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

## V4 visual-state, Final Test and Gazebo dry-run revision---11 August 2026

This revision updates only the English report. The Arabic report and its
figures were left unchanged, and the V3 locked-Test bundle remains a preserved
historical record rather than being rewritten as V4 evidence.

- Replaced the obsolete joint-head visual-model description with the V4
  split-rail contract: one shared stem through ResNet-18 layer~3, independent
  rail-specific layer~4 branches and heads, and no cross-camera feature path.
- Documented the 168 learned outputs and the fields derived by contract: rail
  side from identity, and metric longitudinal position from the predicted
  ratio and fingerprinted public-segment length.
- Separated checkpoint-selection Validation and the previously exposed
  post-selection development Canary from a newly generated, sealed V4 Final
  Test. The selected epoch-11 checkpoint was evaluated once under the frozen
  contract; the immutable completion record reports `completed` and `passed`.
- Recorded the fail-closed pre-inference sequence: the original 1,024-scene
  capture was not inferred after its support-only audit; a preregistered
  16-scene extension reached the frozen coverage minima, and a schema-only
  compatibility projection changed no captured data before reservation.
- Added the primary Final Test measurements: 99.936% segment top-1, 99.912%
  segment macro-F1, 99.915% loaded-state accuracy, 90.897% joint
  correct-segment/within-5%-of-segment accuracy, 0.01573 m correct-segment
  physical-position MAE and 0.8340 mean bounding-box IoU on 4,680 visible slots.
- Added the immutable evidence identities for the selected epoch-11 checkpoint,
  Validation and Canary acceptance, same-frame shadow comparison, the seven
  scenario-grounded observation campaign, eight fail-closed fault cases, V3
  rollback smoke, manual decision and active-profile dry-run smoke.
- Recorded the exact authority boundary: the manual decision covers only
  `gazebo_runtime_dry_run_only`; PlanSys2 updates, actuation, automatic runtime
  switching and physical deployment remain disabled.
- Updated the architecture, data pipeline, runtime, result, limitation,
  conclusion, interface and reproducibility sections to use the V4 contract
  and to retain the V3 result only as a labelled predecessor.
- Re-audited all 27 citation commands against their surrounding claims and
  primary sources. All 25 bibliography entries are used, resolvable and
  project-relevant; wording and metadata were tightened where a source defined
  a technology but did not itself prove a local project choice.
- Reworked Figure 10.1 for A4 print legibility: the rail-branch cards now use
  guaranteed vertical gaps, concise labels and an unobstructed Panel B entry
  path while retaining the 9-point figure text without global scaling.
- Reworked Figure 9.2 at native A4 text size: the external-registry arrow now
  bypasses the deterministic-derivation card, and the shared-parameter and
  no-cross-camera annotations have a wider, full-footnotesize centre lane.
- Replaced the missing academic-supervisor telephone with the publicly listed
  IBISC institutional contact, explicitly labelled as the IBISC secretariat.
- Added a six-millimetre separation between the three command-flow cards and
  the state-transition matrix in Figure 8.1(b), removing their visual contact.

The primary lightweight evidence package for this revision is
`evidence/room315_visual_v4_submission_2026-08-11/`. Its final manifest has
SHA-256
`2fd0a47f16f4a7e8c86cd221715104a070e067b2b52e8f08e2c99a4709c1abf7`,
and its 57-entry `SHA256SUMS` file has SHA-256
`1045bee81d0d696cedb6404de2f61cd8110d6ce1c2c351e1562f8fe2a6bdafc3`.

Final validation completed on 11 August 2026: `make check` produced 82 A4
pages with no undefined citations or references and no overfull boxes. The
delivery PDF and `main.pdf` are byte-identical and share SHA-256
`eb118c934d9a7390843b56c507abae136b57cf35d876c1b2dc9b5bc83a8f70f7`.
The retained build log has SHA-256
`c942ee39040d17bc6780f0e0509b2b192c74aed24c7da45b8c45053ffa8da6c1`.

## V4 closed-loop runtime and active-default revision---12 August 2026

This revision publishes the English report together with the evaluated visual
runtime source, active and recovery configurations, tests and lightweight
evidence package. The Arabic report remains byte-identical, and the
generative-AI declaration is preserved verbatim.

- Replaced the predecessor command-path narrative with the completed V4
  positive campaign: 12/12 cold-start cases, 1,784 V4 observations and no V3
  observation, 24/24 postconditions, 48 accepted supervisor decisions, four
  replans, no safe abort and every controller stopped.
- Added the independent F01--F05 closed-loop fault campaign: 5/5 passed, 424 V4
  observations and no V3 observation, no false success and every final
  controller disabled. The summary, evidence manifest and source checksum list
  have SHA-256 values
  `330a207603425ce46b72c689c78cbe92aa804570c2a9e35caab8e51d6a8d8fd7`,
  `71c6286f021d4dce993d40603df5714506f0732d0739a937f1ab0cfce7ee7d55`
  and
  `be40c331b30c7375886b42a576ec2ac97e21c0bcd940140a77a2c170d102ecd8`.
- Recorded the final manually approved Gazebo runtime. Its promotion manifest,
  candidate state and source checksum list have SHA-256 values
  `506cae0511cf1675fdd666103ce7fc0b5980eb5e68d4cbadf0af99d9ee9560da`,
  `14cedafe28c999786a66934a523db5757e1ccdd7ae34705d5a2df58488fc8df1`
  and
  `5fae4bc7430606bc474dbbc78c9f89de29b3747d1425054e806b6fae62783f55`.
  The state is `active_closed_loop_runtime`, scope is
  `gazebo_v4_closed_loop_runtime_only`, review is manual, automatic promotion is
  false and physical deployment is not approved.
- Ran B01 once after promotion against the exact final runtime manifest. This
  `partial_smoke` passed 1/1 with 88 V4 observations and no V3 observation, one
  actuating step, one satisfied postcondition, two accepted supervisor
  decisions, a verified terminal state and final effect, and a stopped
  controller. Its summary, evidence manifest and source checksum list have
  SHA-256 values
  `15d5f9cf9503f414c0baa3a4ff84932a717aab3add2f3334e88577b4fc70c576`,
  `dd2b3e7a6b3cb07a14538287329a13d3607bec8fd483a9210eebbec21e3b6574`
  and
  `e215ad6391dade682b397614ddcf8ece5ba76653b9728c28b8302c25126ea28d`.
  It records no physical deployment and does not replace the full 12/12
  campaign.
- Clarified that `dry_run_state_fusion=true` and
  `plansys2_update_enabled=false` prevent the inference node's redundant
  ProblemExpert mutation; they do not make the final task gateway
  observation-only. The separately authorised gateway owns the Gazebo planning
  and actuation path.
- Switched both checked-in defaults to V4 while retaining fail-closed task
  startup. `visual_state_runtime.yaml` and `task_execution_runtime.yaml` have
  SHA-256 values
  `22f12a9f96b3d54e0ab3d0bc05c202024ac6912cb50dd6e29ceb4a0a564d24f8`
  and
  `08eaedd7d6feed3dd1268ef18bfa2545348f203f1ff0dad3c2e9fb1a9f25b6ca`.
  The named visual/task rollback files have SHA-256 values
  `a61b72dacbb928b3170d2439c744ff0f2e2ccb227a83e70aab84e4cbdac26cd4`
  and
  `8c57c69448cb2c649ff972ef4ec327d7af32458c0c777048f6a25f3e0ad955d7`.
- Extended `evidence/room315_visual_v4_submission_2026-08-11/` with byte-exact
  positive, fault, final-runtime and post-promotion smoke records. Its final
  evidence manifest, package checksum list and README have SHA-256 values
  `840787b617e0f671628dfc7d8122ff559d76664506ff8f94ac31d043737da446`,
  `dec74565a8ccfba57a32d83dfdd03236daeafeff226d5243594e328fa4adc5ab`
  and
  `33b3a44f2e5e12b34721abcd1b47af609db3b55557c94c6871fa9c6eeb602a3e`.
  The manifest records 67/67 byte-exact payload comparisons and 87 claim
  pointers; the package checksum list contains 69 entries.
- Updated the Abstract, Résumé, V4 evolution, results, limitations, conclusion,
  evidence ladder and training-pipeline figure to distinguish staged
  pre-promotion evidence from the final active Gazebo runtime.
- Preserved the predecessor campaign, source snapshot, model records and report
  freezes as explicitly historical evidence rather than deleting or relabelling
  them as current V4 results.

The final `make check` produced an 88-page A4 PDF (3,542,433 bytes) with no
undefined references or citations and no overfull boxes. `main.pdf` and the
named delivery PDF are byte-identical, with SHA-256
`31f107aa8bf9ff9e666dc7537f0fb84c23caac90f0f0e2a200a1872d148f55bd`.
The retained build log has SHA-256
`8f3efe415f8f899478e45ec1d13cc26226ce75c8d983ead649870191e69e2dad`;
the three identities are recorded in
`evidence/final_report_sha256_2026-08-12.txt`.
