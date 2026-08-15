# Natural-language revision report — 12 August 2026

## Scope and safeguards

This pass revised wording that sounded unnecessarily formulaic, legalistic or
audit-like. It did not alter the report structure, figures, equations,
technical artefacts, experimental method or supported conclusions. The
official report instructions and grading rubric were used as constraints,
especially for the host context, personal responsibility, engineering choices,
results, limitations and conclusion.

A final layout and terminology pass also removed the two-line spill on physical
page 48, standardised British spelling and thousands separators in displayed
report text, and shortened the Introduction's repeated metrics.

The content of the `Use of Generative Artificial Intelligence` declaration was
preserved. At the author's explicit request, only the spellings
`organization`/`optimization` were standardised to British English as
`organisation`/`optimisation`; the disclosure itself was not otherwise edited.

## Significant changes

### Front matter

| Original wording | Revised wording | Reason |
|---|---|---|
| “On a preregistered, one-shot Final Test disjoint from … the frozen epoch-11 checkpoint processed …” | “The frozen epoch-11 checkpoint was then evaluated once on a preregistered Final Test that was separate from …” | Keeps the frozen, disjoint and single-evaluation protocol while using a more natural sentence structure. |
| “all 80/80 frozen acceptance gates passed” | “all 80 predefined gates passed” | Preserves their pre-test definition while avoiding duplicated pass-count notation. |
| “These results authorise only … automatic authorisation and physical deployment remain forbidden.” | “The reported results apply only to … automatic activation is disabled and physical deployment is not approved.” | Separates the scope of the evidence from the runtime setting and physical-deployment approval. |
| “le point de contrôle figé de l’époque 11” | “Le checkpoint retenu à l’issue de l’époque 11 a été figé avant d’être évalué …” | Uses idiomatic technical French and retains the freeze-before-evaluation meaning. |

### Chapters 1–3 — context, host and personal contribution

| Original wording | Revised wording | Reason |
|---|---|---|
| “The challenge was therefore broader than constructing …” | “The work went beyond constructing …” | Removes a repeated formula without weakening the project context. |
| “No commercial proxy or unsupported financial figure is introduced here.” | Removed; the paragraph now states directly that the university-wide budget figures provide context, not project return. | Keeps the required economic discussion without sounding defensive. |
| “Once a decision had been agreed, I was responsible for implementing it consistently …” | “After we agreed on a decision, I implemented it in the relevant configuration, code, tests and documentation.” | Makes the student’s responsibility concrete and direct. |
| “This decision taught me to match model fidelity …” | “This experience showed me that model fidelity had to follow the question being studied.” | Retains rubric-relevant reflection while tying it to the actual modelling decision. |
| “That lesson guided the later integration work.” | Replaced by three concrete questions about available data, required Room 315 behaviour and testability. | Avoids generic reflective prose and shows the engineering method used. |

### Chapters 8–10 — interfaces, model evaluation and runtime

| Original wording | Revised wording | Reason |
|---|---|---|
| “This makes multi-shuttle coordination predictable …” | “This provides predictable semantics for multi-shuttle coordination …” | Removes a repeated template while retaining the interface property. |
| “the configuration made Test access an explicit error; the sealed evaluation used a separate, one-shot-only contract” | “the configuration treated Test access as an error. The final evaluation used a separate one-shot contract.” | Improves readability without weakening leakage control or the one-shot policy. |
| “A static metadata audit …” | “A static metadata check …” | “Check” accurately describes the operation and is more natural in an engineering report. |
| “The manually approved Gazebo profile keeps …; automatic switching and physical deployment remain forbidden.” | “The reviewed Gazebo profile requires manual activation …; automatic activation is disabled, and the profile does not approve physical deployment.” | Preserves the exact control boundary while separating review, activation and deployment approval. |
| “physical deployment remains outside the authorised profile” | “The profile is limited to Gazebo and does not approve physical deployment.” | States the same safety scope more directly. |

### Chapters 11–13 — results, limitations and conclusion

| Original wording | Revised wording | Reason |
|---|---|---|
| “Recorded result and claim boundary” | “Recorded result and scope” | The subsection describes experimental scope; the shorter heading is clearer and less legalistic. |
| “This is therefore positive-case Gazebo qualification evidence rather than …” | “These runs therefore provide positive-case Gazebo qualification evidence; they do not evaluate …” | Keeps all interpretation limits in a more direct two-part statement. |
| “authority boundary” / “Control boundary” | “execution scope” / “Control scope” | Preserves the distinction between observation-only checks and actuation without regulatory-style wording. |
| “The resulting state is … with … manual approval, automatic activation disabled and no physical-deployment approval.” | Split into short statements naming the configured state, manual approval, disabled automatic activation and absent physical approval. | Reduces noun stacking while retaining every runtime condition. |
| “rejected fail closed” | “rejected under the fail-closed policy” | Corrects an awkward construction while retaining the defined safety behaviour. |
| “Manual authority binds the recorded campaigns …” | “Manual approval links the recorded campaigns to a Gazebo-only runtime …” | Uses ordinary engineering language without implying physical approval. |
| “Observation-only qualification remained dry-run and granted no control action.” | “The observation-only checks ran in dry-run mode without issuing control actions.” | Replaces abstract nominalisation with a concrete description. |
| “1040 independent Gazebo scenes” | “1040 Gazebo scenes in the independent Final Test partition” | Makes clear that independence refers to the dataset partition, not a statistical claim about individual scenes. |

### Appendix A — reproducibility

| Original wording | Revised wording | Reason |
|---|---|---|
| “claim-to-record pointers” | “map each reported result to its source record” | Explains the same traceability mechanism in ordinary prose. |
| “the package marks those embedded paths as non-normative” | “They are retained for provenance but are not submission locations” | Preserves the archival/submission distinction without standards-style wording. |
| “Canonical records for the reported results” | “Records supporting the reported results” | The paths remain exact; “canonical” was unnecessary in the caption. |

### Figures 9.3 and 11.2

| Original wording | Revised wording | Reason |
|---|---|---|
| “Audit the 1024-scene base plan” | “Check the 1024-scene base plan” | Describes the same pre-inference metadata operation in ordinary engineering language. |
| “Seal the primary result” | “Record the final result” | Keeps the immutable result record while avoiding compliance-document phrasing. |
| “manual, hash-bound closed-loop authority” | “manually approved, hash-identified profile” | Retains approval and content identity with a clearer description. |
| “physical deployment remain forbidden” | “physical deployment is not approved” | Preserves the safety boundary in wording consistent with the evidence record. |

## Formal terminology intentionally retained

- **Frozen checkpoint, fixed gates and one-shot reservation/contract:** these
  terms describe the leakage-control and single-evaluation protocol. Removing
  them would weaken the scientific meaning.
- **Immutable `TaskGoal`, qualification record and completion ledger:** these
  identify objects that must not change after publication or reservation.
- **Canonical schemas and canonical rail graph:** these name authoritative
  project contracts used by code and evidence.
- **Fail-closed:** this is a defined safety behaviour, not decorative wording.
- **Cryptographic binding, SHA-256 values, fingerprints and JSON Pointers:**
  these are necessary for reproducibility and traceability.
- **Personal responsibility, resource management, autonomy and reflective
  engineering examples:** their substance was retained because it directly
  supports the grading rubric.
- **Verified motion of the industrial and mobile robots:** retained because it
  describes the implemented first-stage simulation work; the separate
  all-robot cold-start check remains correctly limited to launch and feedback.
- **Gazebo-only scope, manual approval, disabled automatic activation and no
  physical-deployment approval:** these limits remain wherever they materially
  qualify a result.

## Verification

- Final build: 73 A4 pages, 3,486,164 bytes.
- Bibliography: 25 cited entries resolved; no undefined citations or references.
- Layout: no overfull horizontal or vertical boxes; sampled pages were visually
  checked at A4 scale.
- Source invariants: numerical tokens, citation commands, cross-reference
  identifiers and technical `code`/`path` identifiers match the pre-edit source
  inventory.
- AI declaration: its content is unchanged apart from the two requested British
  spellings; its final chapter-block SHA-256 is
  `81668d73e5752b17ab3033c005649dfaaa768c1b133dc5a4b2be125a7199d2f5`.
- Delivery PDF: SHA-256
  `c1fdaeff971765e48e7adfa983c3b2539b36ab8fdd60e86a7f6241096d16df82`.

No technical result, numerical value, supported claim or evidence identity was
intentionally changed during this language revision.
