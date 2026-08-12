# Active report evidence

Only the current items below support numerical or experimental claims in the
final English report. Archived records are listed separately and are excluded
from those claims.

## Current evidence

- `room315_visual_v4_submission_2026-08-11/`: primary repository-relative
  visual-model evidence. It binds the selected checkpoint, sealed 1,040-scene
  Final Test, development records, observation-only checks, both closed-loop
  campaigns, the manually approved Gazebo runtime and its B01 smoke.
  Its administration files are:
  - `evidence_manifest.json`, SHA-256
    `840787b617e0f671628dfc7d8122ff559d76664506ff8f94ac31d043737da446`;
  - `SHA256SUMS`, SHA-256
    `dec74565a8ccfba57a32d83dfdd03236daeafeff226d5243594e328fa4adc5ab`;
  - `README.md`, SHA-256
    `33b3a44f2e5e12b34721abcd1b47af609db3b55557c94c6871fa9c6eeb602a3e`.
  The package records 67/67 byte-exact payload comparisons, 69 checksum
  entries and 87 verified claim-to-JSON-Pointer mappings. Checkpoint, Final
  Test payloads and runtime bags remain external hash-identified objects.
- `runtime/closed_loop/positive/` within that package: the positive campaign
  passed 12/12 cases, recorded 1,784 selected-schema observations and none from
  the alternate schema, verified 24/24 postconditions and 48 supervisor
  decisions, and stopped every controller. Summary, evidence-manifest and
  source-checksum-list SHA-256 values
  are respectively
  `1504f25742d135c32ea879651336ff22a64cdfb2e09870be81ea7fb68b279479`,
  `15d6362980751dd3e2bdd7fdab2b5ccb70118f73e1bd2cb2cf71d87bc96fdece`
  and
  `8ec1b8a456c5f83034fce0d92a3dc067d9d271e02e66a4471bc51acd9dc0e9f8`.
- `runtime/closed_loop/fault/` within that package: F01--F05 passed 5/5 with
  424 selected-schema observations, none from the alternate schema, no false
  success and every final controller disabled. Summary, evidence-manifest and
  source-checksum-list SHA-256 values are respectively
  `330a207603425ce46b72c689c78cbe92aa804570c2a9e35caab8e51d6a8d8fd7`,
  `71c6286f021d4dce993d40603df5714506f0732d0739a937f1ab0cfce7ee7d55`
  and
  `be40c331b30c7375886b42a576ec2ac97e21c0bcd940140a77a2c170d102ecd8`.
- `runtime/closed_loop/active_runtime/` within that package: the authorisation
  manifest, runtime state and source checksum list have SHA-256 values
  `506cae0511cf1675fdd666103ce7fc0b5980eb5e68d4cbadf0af99d9ee9560da`,
  `14cedafe28c999786a66934a523db5757e1ccdd7ae34705d5a2df58488fc8df1`
  and
  `5fae4bc7430606bc474dbbc78c9f89de29b3747d1425054e806b6fae62783f55`.
  The state is `active_closed_loop_runtime` under
  `gazebo_v4_closed_loop_runtime_only`; review is manual, automatic selection
  is disabled and physical deployment is not approved.
- `runtime/closed_loop/active_runtime/smoke_B01/` within that package: a
  `partial_smoke` using the exact authorised runtime manifest. B01 passed 1/1
  with 88 selected-schema observations and none from the alternate schema, one
  actuating step, one satisfied postcondition, two accepted supervisor
  decisions, a verified final effect, a successful terminal state and a stopped
  controller. Summary, evidence-manifest and source-checksum-list SHA-256 values
  are respectively
  `15d5f9cf9503f414c0baa3a4ff84932a717aab3add2f3334e88577b4fc70c576`,
  `dd2b3e7a6b3cb07a14538287329a13d3607bec8fd483a9210eebbec21e3b6574`
  and
  `e215ad6391dade682b397614ddcf8ece5ba76653b9728c28b8302c25126ea28d`.
  This selected-case smoke records `physical_deployment=false` and does not
  replace the full 12/12 positive campaign.
- `room315_v4_closed_loop_campaign_figures_2026-08-12/`: checksum-identified
  B03 frame extraction and provenance. The composite image has SHA-256
  `3afef390900901598926f0d11226bb460a70b818ce0c4c765594aefb6fd614dc`;
  its `SHA256SUMS` has SHA-256
  `9914684e2333a5ccc0053c0521f4bbdade217374c253875c31475fcaa20af39c`.
- `room315_language_semantic_evaluation_v3/`: pinned ten-case offline language
  contract evaluation, including raw backend outputs, strict-envelope and
  fusion traces, environment, source snapshot and checksums. Its directory name
  is an evidence identifier and does not select the visual runtime.
- `multi_robot_cold_start_smoke_2026-08-07/`: predeclared one-run full-floor
  all-robot launch/interface smoke.
- Current runtime defaults:
  - `mfja_robot_control_config/config/room_315_vla/visual_state_runtime.yaml`,
    SHA-256
    `22f12a9f96b3d54e0ab3d0bc05c202024ac6912cb50dd6e29ceb4a0a564d24f8`;
  - `mfja_robot_control_config/config/room_315_vla/task_execution_runtime.yaml`,
    SHA-256
    `08eaedd7d6feed3dd1268ef18bfa2545348f203f1ff0dad3c2e9fb1a9f25b6ca`.
  The task default is fail closed with `execution_enabled: false`; a guarded
  launch must explicitly opt in after reverifying the runtime authorisation.
- Explicit rollback defaults:
  - `visual_state_runtime_v3_rollback.yaml`, SHA-256
    `a61b72dacbb928b3170d2439c744ff0f2e2ccb227a83e70aab84e4cbdac26cd4`;
  - `task_execution_runtime_v3_rollback.yaml`, SHA-256
    `8c57c69448cb2c649ff972ef4ec327d7af32458c0c777048f6a25f3e0ad955d7`.
- Recorded 7 August component evidence remains applicable to its stated test
  scope: `pytest_current_source_focused_2026-08-07.log` (190/190),
  `pytest_current_source_transport_2026-08-07.log` (96/96), and
  `pytest_current_source_runbook_capabilities_2026-08-07.log` (42/42).

## Archived records retained

- `room315_integrated_campaign_v2/` and
  `ROOM315_INTEGRATED_CAMPAIGN_ARCHIVE.md`: archived campaign records, release
  identity and extraction instructions. They do not support the current
  closed-loop claims.
- `room315_campaign_figures_2026-08-07/`: alternate B03 derivation retained for
  reconstruction; the report uses the derivation listed above.
- `visual_model_evaluation_2026-07-30/`: rollback-model training sidecars,
  grouped split/leakage audit and locked-Test review.
- `current_source_identity_2026-08-07.txt` and
  `current_source_supplement_2026-08-07/`: archived source identity and its
  two untracked test files.
- `final_report_build_2026-08-08.log`,
  `final_report_sha256_2026-08-08.txt`, `final_report_build_2026-08-11.log` and
  `final_report_sha256_2026-08-11.txt`: preserved report freezes.
- `final_report_build_2026-08-12.log` and
  `final_report_sha256_2026-08-12.txt`: final 88-page A4 English-report build
  and fingerprint. The retained log has SHA-256
  `8f3efe415f8f899478e45ec1d13cc26226ce75c8d983ead649870191e69e2dad`;
  `main.pdf` and the named delivery PDF are byte-identical at SHA-256
  `31f107aa8bf9ff9e666dc7537f0fb84c23caac90f0f0e2a200a1872d148f55bd`.

The three recorded pytest suites overlap and their pass counts must not be
summed. The positive campaign is a bounded acceptance matrix rather than a
reliability estimate; the fault campaign covers five declared scenarios rather
than unbounded recovery behaviour. The ten-case language result is an interface
acceptance set rather than a general language benchmark. All current runtime
evidence is limited to Gazebo simulation and establishes neither physical-camera
generalisation, certified machinery safety nor physical-deployment permission.
The repository revision carrying this index commits the evaluated
implementation, active and recovery configurations, tests, submission PDF and
lightweight evidence together. A clean checkout of that revision reconstructs
the source-side handover. The checkpoint, sealed dataset and raw ROS bags remain
external hash-identified objects and are intentionally not tracked here.
