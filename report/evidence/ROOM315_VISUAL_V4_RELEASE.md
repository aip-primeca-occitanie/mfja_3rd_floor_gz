# Room 315 Visual Model V4 full-evidence release

The repository retains the compact, inspectable V4 evidence package at
`room315_visual_v4_submission_2026-08-11/`. The selected checkpoint, sealed
Final Test payload and raw positive, fail-closed and post-promotion-smoke
recordings are published outside Git as one checksum-bound GitHub Release
asset.

Naming note: `V4` identifies the visual-model generation
`room315.visual_state.v4`. It is unrelated to the historical repository tag
`version-4` and is not a campaign/evidence revision number.

- Release tag: `room315-visual-v4-evidence-2026-08-11`
- Release page: <https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/tag/room315-visual-v4-evidence-2026-08-11>
- Target commit: `0d19e1601d57416b83c871c1a8d413ec0dd523a6`
- Asset: `room315_visual_v4_full_evidence_2026-08-11.tar.zst`
- Direct download: <https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/download/room315-visual-v4-evidence-2026-08-11/room315_visual_v4_full_evidence_2026-08-11.tar.zst>
- Checksum sidecar: `room315_visual_v4_full_evidence_2026-08-11.tar.zst.sha256`
  (`117` bytes)
- Sidecar download: <https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/download/room315-visual-v4-evidence-2026-08-11/room315_visual_v4_full_evidence_2026-08-11.tar.zst.sha256>
- Size: `247,825,706` bytes
- SHA-256: `35b583baca4f45eed6aad659c253180d00f1a5830ce389266ba714a4445a8ecc`
- Archive root: `room315_visual_v4_full_evidence_2026-08-11/`
- Contents: `6,492` regular files, including `18` MCAP recordings

The archive contains:

- the byte-exact compact evidence package tracked in Git;
- the final active Gazebo runtime bundle and the single selected epoch-11
  checkpoint (`85,876,329` bytes, SHA-256
  `869d64049b0092c37d21a4c8b910dc6b91954527e0e49c5694fa82dce570f40d`);
- the sealed 1,040-scene Final Test with 2,080 synthetic images and dataset
  fingerprint
  `226f4b5889f7d8b66bccc87d3e8dd494f710c8f4b228fab4851d6da7448e2eef`;
- the final 12/12 positive campaign with twelve MCAP bags;
- the final 5/5 fail-closed campaign with five MCAP bags; and
- the passed post-promotion B01 smoke with one MCAP bag.

## Complete data-and-reproduction companion

This full-evidence archive preserves the accepted runtime and Final-Test
evidence, but it is not the complete V4 training distribution. The complete
post-experiment distribution is published from this same project in its
[`v4-seed31520260811-dataset-v1`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/tag/v4-seed31520260811-dataset-v1)
release. It adds every labelled Training, Validation and Development-Canary input,
the Final Test, the V3 initialisation checkpoint, selected V4 checkpoint,
frozen source and stateless replay tooling. Its small control package is
retained at `room315_visual_v4_dataset_release_v1/`. See
[`ROOM315_VISUAL_V4_DATASET_RELEASE.md`](ROOM315_VISUAL_V4_DATASET_RELEASE.md)
for its six asset hashes and reproduction instructions. The two releases are
complementary releases of this project; neither broadens the reported
experimental or physical-deployment claims. Public accessibility of the data
release is not an open-data or model-weight reuse licence.

Verify and extract it with:

```bash
sha256sum -c room315_visual_v4_full_evidence_2026-08-11.tar.zst.sha256
tar --zstd -xf room315_visual_v4_full_evidence_2026-08-11.tar.zst
cd room315_visual_v4_full_evidence_2026-08-11
sha256sum --strict -c SHA256SUMS
```

The public asset was downloaded after publication and reproduced the archive
SHA-256 above. The release is not stored in Git and therefore does not enlarge
the repository history.

This is final accepted Gazebo evidence, not every development attempt. It
excludes superseded attempts, intermediate training checkpoints, duplicate
checkpoint copies and intermediate Final Test roots. The raw records preserve
archival local paths and experiment-host provenance byte-for-byte. They grant
no physical-deployment permission and establish neither physical-camera
generalisation nor unbounded operational safety.
