# Room 315 V4 submission evidence

Package status: `final`.

This lightweight package preserves the selected V4 model evidence, the
development Canary, the sealed Final Test, and the final Gazebo closed-loop
qualification and runtime-promotion evidence. Every payload listed in
`evidence_manifest.json` is a byte-exact source copy. The manifest gives the
SHA-256 digest of each copy and exact JSON Pointers for every reported claim.

## Final Test headline

The frozen epoch-11 candidate was evaluated once on 1,040 sealed Gazebo scenes
containing 4,680 visible shuttle slots. The immutable result reports:

- acceptance passed, with no automatic runtime transition;
- segment top-1 accuracy `0.9993589743589744` and segment macro-F1
  `0.9991171826996464`;
- loaded-state accuracy `0.9991452693939209`;
- joint correct-segment and position-within-5%-of-segment accuracy
  `0.908974358974359`;
- correct-segment physical-position MAE `0.01572997309267521 m` and p95
  `0.058498889207839966 m`;
- mean bounding-box IoU `0.8339976072311401`;
- left/right segment top-1 accuracy `0.9987261146496815` / `1.0`, with the
  lowest side-by-segment top-1 value `0.9464285714285714` on left `A14`
  (`56` visible slots);
- fixed segment-confidence coverage `0.9995726495726496`, fixed joint
  confidence coverage `0.9995726495726496`, and loaded-confidence coverage
  `1.0`;
- zero maximum opposite-camera influence under the blank/shuffle isolation
  checks, while swapping camera order reduced segment top-1 by
  `0.9213675213675214`.

These values are synthetic/Gazebo-only evidence. The Final Test did not trigger
an automatic runtime switch. A later, explicitly reviewed promotion is recorded
below; it remains limited to the Gazebo V4 closed-loop runtime and does not
approve physical deployment.

## V4 closed-loop runtime evidence

The positive closed-loop campaign passed all `12/12` declared cases (`6` per
rail). It recorded `1,784` V4 observations and zero V3 observations, executed
and verified `24/24` postconditions, recorded `48` accepted supervisor
decisions and `4` replans, produced zero safe aborts, and left every controller
stopped. Its byte-exact summary is
`runtime/closed_loop/positive/summary.json` (SHA-256
`1504f25742d135c32ea879651336ff22a64cdfb2e09870be81ea7fb68b279479`).

The fail-closed campaign passed scenarios `F01`--`F05` (`5/5`), with `424` V4
observations, zero V3 observations, zero false-success counts, and every final
controller disabled. Its byte-exact summary is
`runtime/closed_loop/fault/summary.json` (SHA-256
`330a207603425ce46b72c689c78cbe92aa804570c2a9e35caab8e51d6a8d8fd7`).

The final candidate state is `active_closed_loop_runtime` under the exact scope
`gazebo_v4_closed_loop_runtime_only`. It binds both campaign hashes, enables
Gazebo actuation, keeps authoritative-state fusion in dry-run, leaves PlanSys2
updates disabled, and explicitly sets `physical_deployment_approved = false`.
The byte-exact promotion manifest and candidate state are under
`runtime/closed_loop/active_runtime/` with SHA-256 values
`506cae0511cf1675fdd666103ce7fc0b5980eb5e68d4cbadf0af99d9ee9560da` and
`14cedafe28c999786a66934a523db5757e1ccdd7ae34705d5a2df58488fc8df1`,
respectively.

After that promotion, a deliberately partial smoke run re-executed selected
case `B01` against the final runtime manifest. Its status is `partial` because
it selected one case rather than repeating the full 12-case campaign; the
selected result passed `1/1`. The run recorded `88` V4 observations and zero V3
observations, one actuating step, one satisfied postcondition, and two accepted
supervisor decisions. Terminal status, final-effect, and post-task controller
checks all passed, and every controller stopped. The summary binds runtime
manifest SHA-256
`506cae0511cf1675fdd666103ce7fc0b5980eb5e68d4cbadf0af99d9ee9560da`,
sets `physical_deployment = false`, and is preserved at
`runtime/closed_loop/active_runtime/smoke_B01/summary.json` (SHA-256
`15d5f9cf9503f414c0baa3a4ff84932a717aab3add2f3334e88577b4fc70c576`).

This is bounded simulation evidence, not a proof of unbounded operational
safety or physical-robot generalization. It grants no permission for physical
deployment.

## Fail-closed provenance

The original 1,024-scene V1 dataset was captured and sealed with
`inference_status = not_run`. A support-only pre-inference audit found aggregate
support of `2`, `2`, and `6` for `adjacent_branch`, `behind_region`, and
`intermediate_route`, respectively, below the frozen minimum of `8`. The V1
contract was therefore not executed.

A 16-scene coverage extension was then preregistered before model inference.
It preserved the byte-identical 1,024-scene prefix and raised the composite
design to 1,040 scenes. The V2 extension seal also records zero inference.
Because the already-frozen evaluator accepts the canonical V1 control schemas,
a pre-inference compatibility projection recreated only the control envelopes
in a fresh root; its own controls state that neither manifest nor captured
content changed. The final contract, protocol lock, sealed controls, result,
and completion record are under `final_test/final/`.

## Contents

- `model/`: selected-candidate training and validation artifacts. Validation
  evidence is development evidence, not Final Test evidence.
- `canary/`: development-Canary report and immutable completion record. The
  Canary had already been exposed during development and is not an independent
  Test set.
- `runtime/`: earlier Gazebo shadow/dry-run evidence plus the final V4 positive
  closed-loop campaign, fail-closed campaign, and manually authorized active
  Gazebo runtime state, including its post-promotion B01 partial smoke check.
- `final_test/plan/` and `final_test/protocol/`: original V1 preregistration and
  the evaluator source/tests frozen before the Final Test sequence.
- `final_test/provenance/`: compact byte-exact evidence for the V1 support
  NO-GO and the preregistered V2 coverage extension.
- `final_test/final/controls/`: the schema-compatible 1,040-scene dataset
  controls, disjointness audit, seal, protocol lock, and evaluation contract.
- `final_test/final/results/`: the immutable completion ledger and final result
  artifacts.
- `evidence_manifest.json`: inventory, provenance chain, external-object
  digests, and claim-to-JSON-Pointer mapping.
- `SHA256SUMS`: integrity hashes for every packaged file except itself.

## Path and exclusion policy

All paths authored by this package are relative to this directory. Copied
machine-generated JSON files can contain archival absolute paths from their
original execution environment; those embedded paths are preserved
byte-for-byte, are non-normative, and are not submission locations.

The checkpoint, sealed dataset, and runtime bags remain external
hash-identified objects. This package intentionally excludes checkpoint
binaries, Final Test rows, labels, images, captured episodes, runtime bags,
predictions, and logs. The included finalization records contain only integrity
and support metadata needed to verify the external dataset. The copied source
manifests and source `SHA256SUMS` files may index those deliberately excluded
objects; they are preserved byte-for-byte as provenance indexes and are not the
package-level integrity ledger.
