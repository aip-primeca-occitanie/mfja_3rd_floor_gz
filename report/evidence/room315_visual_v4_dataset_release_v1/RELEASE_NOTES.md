# Release notes — `v4-seed31520260811-dataset-v1`

This is the first complete post-experiment reproduction release for the Room
315 visual-state V4 results. The release and this lightweight control package
belong to the project repository
[`aip-primeca-occitanie/mfja_3rd_floor_gz`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz).

## Published release receipt

GitHub release [`371403091`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/tag/v4-seed31520260811-dataset-v1)
was published at `2026-08-16T18:40:57Z`. It targets commit
[`a26dbd5569b5bb1ab4f794f96fbbd3e8486aca30`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/commit/a26dbd5569b5bb1ab4f794f96fbbd3e8486aca30)
and is public, with `draft = false` and `prerelease = false`. GitHub's SHA-256
metadata for all 6 uploaded assets (810803201 bytes total) was verified against
the control manifest; every name, size, and digest matched.

## Release inventory

The six archives total `810803201` bytes. Dataset counts are 7,336
paired-camera scenes and 14,672 JPEG images: 5,528 Training, 512 Validation,
256 Development-Canary, and 1,040 Final-Test scenes.

| Asset | Logical count | Bytes | SHA-256 |
|---|---:|---:|---|
| `room315_visual_v4_train_5528_seed31520260811_v1.tar.zst` | 5,528 scenes / 11,056 JPEGs | 441324291 | `054d00d06f5fc88a199a712cdcbfed4c1b34c70196d19362a9727563d9a1cf16` |
| `room315_visual_v4_validation_512_v3r1_v1.tar.zst` | 512 scenes / 1,024 JPEGs | 51996651 | `72773a44e594d9029126b6e6d921abe01ea00c44ac738fcb4d9d0ea324ea69ad` |
| `room315_visual_v4_canary_256_v3r1_v1.tar.zst` | 256 scenes / 512 JPEGs | 21345446 | `54b9f4a7e30f2286ea6b7512ffb6f5d7035c2afaa8b45931124f03e647bad143` |
| `room315_visual_v4_final_test_1040_coverage_v2_v1.tar.zst` | 1,040 scenes / 2,080 JPEGs | 109616317 | `46ba09d8ee6fee39e453a0cd06e80f17d2ca2339289817cc7b0f0f502ac1044f` |
| `room315_visual_v4_models_init8a2d-and-epoch011_869d_v1.tar.zst` | V3 initialization + selected V4 checkpoint | 185354488 | `82dbd47357c07f14d68c8d492fad575d62f3cc0f60c9e10852c66949a04e7f78` |
| `room315_visual_v4_frozen_source_0d19e160_v1.tar.zst` | 1 protocol-locked source tree + replay entry point | 1166008 | `287583d23cecfd3e2408b40ba5a4a3e725a788d55e58822afd8e774b889e293f` |

The selected V4 checkpoint SHA-256 is
`869d64049b0092c37d21a4c8b910dc6b91954527e0e49c5694fa82dce570f40d`.
The frozen evaluation-protocol-lock SHA-256 is
`03104b8e3585710b96571dfc723120f45b1acbd26351d2fcc5a17655906eb182`.

## Anonymous download and verification

Provide `git`, `curl`, GNU `sha256sum`, GNU `tar` plus `zstd`, and Python 3.12.
No GitHub CLI, GitHub account, or authentication token is required.

```bash
git clone --branch v4-seed31520260811-dataset-v1 --depth 1 https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz.git
cd mfja_3rd_floor_gz
test "$(git rev-parse HEAD)" = a26dbd5569b5bb1ab4f794f96fbbd3e8486aca30
cd report/evidence/room315_visual_v4_dataset_release_v1
room315_tag=v4-seed31520260811-dataset-v1
room315_base="https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/download/$room315_tag"
mkdir -p release-assets
while read -r _digest room315_asset; do
  curl --fail --location --retry 5 \
    --output "release-assets/$room315_asset" \
    "$room315_base/$room315_asset"
done < SHA256SUMS
(cd release-assets && sha256sum --strict -c ../SHA256SUMS)
python3 scripts/verify_release.py release-assets
```

This block deliberately does not call the publication-time tag's legacy
`download_release.sh`, which used an authenticated GitHub CLI. The inline
public-HTTPS flow works from the fixed tag without moving it and verifies every
archive against the tag's `SHA256SUMS` before use. A later branch correction
also supplies a resumable, fail-closed `curl` downloader.

## Full replay verification

After packaging, all six archive hashes were verified and the merged release
was replayed locally on CUDA. The replay completed with
`all_requested_comparisons_passed = true` for:

- Validation, Canary, and Final-Test primary metrics;
- camera counterfactuals;
- segment calibration;
- acceptance gates; and
- Final-Test dataset coverage and frozen runtime-threshold reports.

The compact verification record is the tag-qualified
[`provenance/local_cuda_full_replay.json`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/blob/v4-seed31520260811-dataset-v1/report/evidence/room315_visual_v4_dataset_release_v1/provenance/local_cuda_full_replay.json).
It records CPython 3.12.3, PyTorch 2.10.0+cu128, torchvision 0.25.0+cu128, CUDA
12.8, and cuDNN 91002. The extraction and stateless replay commands are
documented in the publication-time, tag-qualified
[`README.md`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/blob/v4-seed31520260811-dataset-v1/report/evidence/room315_visual_v4_dataset_release_v1/README.md);
use the anonymous download block above in place of that snapshot's legacy
download script. Stateless replay does not reopen or mutate the consumed
one-shot ledgers.

## Source and full evidence

- Frozen evaluator source:
  [`aip-primeca-occitanie/mfja_3rd_floor_gz@0d19e160`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/tree/0d19e1601d57416b83c871c1a8d413ec0dd523a6)
- Complete organization-side report/evidence snapshot:
  [`room315_visual_v4_submission_2026-08-11@503e13ee`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/tree/503e13ee81afbb553d0a0150f52175451e0b96d1/report/evidence/room315_visual_v4_submission_2026-08-11)
- Companion full-evidence release:
  [`room315-visual-v4-evidence-2026-08-11`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/tag/room315-visual-v4-evidence-2026-08-11)
- Project dataset release:
  [`v4-seed31520260811-dataset-v1`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/tag/v4-seed31520260811-dataset-v1)

The implementation, report evidence, control package, and release assets all
remain anchored to the same project repository and their recorded commits.

## Legal and deployment status

Public visibility is provided so readers can verify and reproduce the reported
experiment. It is not a standalone open-data or model-weight reuse license. No
permission to redistribute, modify, or reuse the images, labels, or weights is
granted by this metadata release; see the tag-qualified
[`LICENSE_STATUS.md`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/blob/v4-seed31520260811-dataset-v1/report/evidence/room315_visual_v4_dataset_release_v1/LICENSE_STATUS.md).

Canary and Final Test are published only after the historical experiment and
do not change their original experimental roles. The evidence is synthetic
Gazebo evidence and does not approve physical deployment or an automatic
runtime transition.
