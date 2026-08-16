# Room 315 visual V4 dataset

Complete post-experiment release-control and reproduction package for the
reported Room 315 visual-state V4 experiment in
[`aip-primeca-occitanie/mfja_3rd_floor_gz`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz).
Large data, checkpoints, and the frozen source tree are attached to that
project's GitHub Release `v4-seed31520260811-dataset-v1` rather than committed
to Git history.

## Project release location

- Project repository:
  [`aip-primeca-occitanie/mfja_3rd_floor_gz`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz)
- Dataset release:
  [`v4-seed31520260811-dataset-v1`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/tag/v4-seed31520260811-dataset-v1)
- Publication-time control package:
  [fixed tag-qualified repository tree](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/tree/v4-seed31520260811-dataset-v1/report/evidence/room315_visual_v4_dataset_release_v1)
- Complete project-side V4 evidence:
  [frozen evidence snapshot](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/tree/503e13ee81afbb553d0a0150f52175451e0b96d1/report/evidence/room315_visual_v4_submission_2026-08-11)

## Publication receipt

GitHub release `371403091` was published at `2026-08-16T18:40:57Z` from commit
[`a26dbd5569b5bb1ab4f794f96fbbd3e8486aca30`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/commit/a26dbd5569b5bb1ab4f794f96fbbd3e8486aca30).
Its actual status is public, `draft = false`, and `prerelease = false`.
GitHub's SHA-256 metadata for all 6 uploaded assets (810803201 bytes total) was
checked against `manifests/release_manifest.json` and matched every recorded
digest.

The release tag remains fixed at that publication commit and is not retargeted
by later documentation. Its publication-time `download_release.sh` used the
GitHub CLI. This branch revision records the publication receipt and supplies
the corrected anonymous `curl` downloader below. Readers working directly from
the fixed tag should use the self-contained anonymous block in the live release
notes or `ROOM315_VISUAL_V4_DATASET_RELEASE.md`, rather than the tag's legacy
download script.

## Contents

| Asset | Role |
|---|---|
| `room315_visual_v4_train_5528_seed31520260811_v1.tar.zst` | Exact 5,528 training scenes and labels |
| `room315_visual_v4_validation_512_v3r1_v1.tar.zst` | Disjoint validation used for checkpoint selection |
| `room315_visual_v4_canary_256_v3r1_v1.tar.zst` | Post-selection Development-Canary inputs, labels, results, and ledger |
| `room315_visual_v4_final_test_1040_coverage_v2_v1.tar.zst` | Canonical Final-Test inputs, labels, controls, results, and ledger |
| `room315_visual_v4_models_init8a2d-and-epoch011_869d_v1.tar.zst` | V3 initialization, selected V4 checkpoint, and frozen candidate |
| `room315_visual_v4_frozen_source_0d19e160_v1.tar.zst` | Exact locked source tree and stateless replay entry point |

In total: 7,336 paired-camera scenes and 14,672 JPEG files. Canary and
Final Test are public only now, after the historical experiment and its
immutable attempt had completed.

## System prerequisites

Download, verification, and extraction require `curl`, `sha256sum` (GNU
coreutils), `git`, GNU `tar` with Zstandard support, the `zstd` executable, and
Python 3.12. The downloader uses public HTTPS release URLs; it does not require
GitHub CLI, a GitHub account, or authentication. See `ENVIRONMENT.md` and
`requirements-replay.txt` for the replay environment.

## Download and verify

The following command applies to this corrected branch revision of the control
package:

```bash
bash scripts/download_release.sh release-assets
python3 scripts/verify_release.py release-assets
mkdir -p extracted
for archive in release-assets/*.tar.zst; do
  tar --zstd -xf "$archive" -C extracted
done
python3 scripts/verify_extracted_dataset.py \
  extracted/room315_visual_v4_dataset_v1
```

Interrupted transfers remain as `*.part` and resume on the next identical
command. To fetch only selected assets, append their exact basenames after the
destination directory; omitting basenames downloads all six assets.

## Re-evaluate the published checkpoint

Create the recorded Python environment described in `ENVIRONMENT.md`, then:

```bash
python3 scripts/reproduce_results.py \
  --release-root extracted/room315_visual_v4_dataset_v1 \
  --source-repo extracted/room315_visual_v4_dataset_v1/source/frozen_tree \
  --split all --full \
  --device cuda \
  --output reproduced-results.json
```

This stateless command independently recomputes metrics; it does not reopen
or mutate the consumed historical one-shot ledgers. The original and
recomputed results are compared in the generated report.

The complete extracted release was replayed on CUDA after packaging. All
nested comparisons passed for Validation, Canary, and Final Test, including
acceptance, Final-Test coverage, and runtime thresholds. See
`provenance/local_cuda_full_replay.json` for the machine-readable verification
summary. This successful check used the recorded torch/CUDA stack with newer
NumPy/Pillow/PyYAML versions; `ENVIRONMENT.md` still preserves the historical
version lock.

## Repeat the training procedure

The portable training configuration and V3 initialization checkpoint are
both included. Run from the directory containing the merged dataset root:

```bash
room315_release_repo="$PWD"
cd "$room315_release_repo/extracted"
python3 room315_visual_v4_dataset_v1/source/frozen_tree/mfja_robot_control_config/scripts/room_315_vla_train_v4.py \
  preflight --decode-images \
  --config "$room315_release_repo/configs/visual_state_training_v4_portable.json" \
  --output v4-preflight
python3 room315_visual_v4_dataset_v1/source/frozen_tree/mfja_robot_control_config/scripts/room_315_vla_train_v4.py \
  train --device cuda --decode-images-preflight \
  --config "$room315_release_repo/configs/visual_state_training_v4_portable.json" \
  --output v4-retrained
```

The procedure, inputs, seed, and environment are fixed; bit-for-bit identity
of a newly trained checkpoint across different CUDA hardware is not claimed.

## Provenance

Experiment source: [`aip-primeca-occitanie/mfja_3rd_floor_gz` at commit
`0d19e1601d57416b83c871c1a8d413ec0dd523a6`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/tree/0d19e1601d57416b83c871c1a8d413ec0dd523a6),
with one protocol-locked CMake blob recorded in the source manifest. The later
[full report/evidence snapshot at commit
`503e13ee81afbb553d0a0150f52175451e0b96d1`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/tree/503e13ee81afbb553d0a0150f52175451e0b96d1/report/evidence/room315_visual_v4_submission_2026-08-11)
remains in the organization project. See `RELEASE_NOTES.md`, `DATASET_CARD.md`,
`LICENSE_STATUS.md`, and `manifests/release_manifest.json`.
