# Room 315 Visual V4 data and reproduction companion

This is the complete data-and-reproduction companion to the Room 315 Visual
Model V4 evidence package. It is published as a GitHub Release of the MFJA
source project itself, with the small control files retained in the repository
and the large archives kept outside Git history. Immutable commit identifiers
bind the distribution to the evaluated implementation and report evidence.

- Project repository: <https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz>
- Release tag: `v4-seed31520260811-dataset-v1`
- Release page: <https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/tag/v4-seed31520260811-dataset-v1>
- Repository control package: `room315_visual_v4_dataset_release_v1/`
- Release target/control commit: `a26dbd5569b5bb1ab4f794f96fbbd3e8486aca30`
- Evaluated source commit: `0d19e1601d57416b83c871c1a8d413ec0dd523a6`
- Frozen report/evidence snapshot: `503e13ee81afbb553d0a0150f52175451e0b96d1`
- Companion full-evidence release: <https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/tag/room315-visual-v4-evidence-2026-08-11>

## Publication receipt

The following values were read back from the GitHub Releases API after
publication. They describe the public object, not only the local release plan.

| Field | Published value |
|---|---|
| Repository visibility | `PUBLIC` |
| Release ID | `371403091` |
| Release state | published; `draft=false`; `prerelease=false` |
| Target commit | `a26dbd5569b5bb1ab4f794f96fbbd3e8486aca30` |
| Published at | `2026-08-16T18:40:57Z` |
| Assets | 6 uploaded archives; 810,803,201 bytes total |

GitHub reported an asset state of `uploaded` for all six archives. Each
GitHub-generated `sha256:` digest and byte size matched the corresponding row
below and the repository control manifest. The release page and tag-pinned
control files were also reachable without GitHub authentication after
publication.

The six release assets are:

| Asset | Bytes | SHA-256 |
|---|---:|---|
| `room315_visual_v4_train_5528_seed31520260811_v1.tar.zst` | 441,324,291 | `054d00d06f5fc88a199a712cdcbfed4c1b34c70196d19362a9727563d9a1cf16` |
| `room315_visual_v4_validation_512_v3r1_v1.tar.zst` | 51,996,651 | `72773a44e594d9029126b6e6d921abe01ea00c44ac738fcb4d9d0ea324ea69ad` |
| `room315_visual_v4_canary_256_v3r1_v1.tar.zst` | 21,345,446 | `54b9f4a7e30f2286ea6b7512ffb6f5d7035c2afaa8b45931124f03e647bad143` |
| `room315_visual_v4_final_test_1040_coverage_v2_v1.tar.zst` | 109,616,317 | `46ba09d8ee6fee39e453a0cd06e80f17d2ca2339289817cc7b0f0f502ac1044f` |
| `room315_visual_v4_models_init8a2d-and-epoch011_869d_v1.tar.zst` | 185,354,488 | `82dbd47357c07f14d68c8d492fad575d62f3cc0f60c9e10852c66949a04e7f78` |
| `room315_visual_v4_frozen_source_0d19e160_v1.tar.zst` | 1,166,008 | `287583d23cecfd3e2408b40ba5a4a3e725a788d55e58822afd8e774b889e293f` |

Their combined size is 810,803,201 bytes (773.24 MiB). They publish every
labelled partition used by the reported procedure: 5,528 Training scenes,
512 Validation scenes, 256 Development-Canary scenes and 1,040 Final-Test
scenes. This is 7,336 paired-camera scenes and 14,672 JPEG images.

The model asset includes the predecessor V3 initialisation checkpoint and the
selected V4 epoch-11 checkpoint. The source asset includes the protocol-locked
evaluation implementation, generation provenance and the stateless replay
runner. The operational old-replay rows are path-sanitised, while their
byte-exact originals and selected event/validation trace chain are retained
separately for historical hash audit.

## Download, verify and replay

Integrity verification and extraction require `git`, `curl`, `sha256sum`,
Python 3 and GNU `tar` with Zstandard support. Python 3.12, the packages in
`requirements-replay.txt`, and the CUDA-compatible environment described in
`ENVIRONMENT.md` are required for the closest full metric replay. A GPU is not
required merely to download, hash or inspect the extracted files. Reserve
enough disk space for both the 773.24 MiB download and the extracted tree.

The following public workflow uses no GitHub account, token or `gh` client:

```bash
git clone --branch v4-seed31520260811-dataset-v1 --depth 1 \
  https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz.git
cd mfja_3rd_floor_gz
test "$(git rev-parse HEAD)" = a26dbd5569b5bb1ab4f794f96fbbd3e8486aca30
cd report/evidence/room315_visual_v4_dataset_release_v1

room315_tag=v4-seed31520260811-dataset-v1
room315_base="https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/download/$room315_tag"
room315_assets=(
  room315_visual_v4_train_5528_seed31520260811_v1.tar.zst
  room315_visual_v4_validation_512_v3r1_v1.tar.zst
  room315_visual_v4_canary_256_v3r1_v1.tar.zst
  room315_visual_v4_final_test_1040_coverage_v2_v1.tar.zst
  room315_visual_v4_models_init8a2d-and-epoch011_869d_v1.tar.zst
  room315_visual_v4_frozen_source_0d19e160_v1.tar.zst
)
mkdir -p release-assets
for room315_asset in "${room315_assets[@]}"; do
  curl --fail --location --retry 3 \
    --output "release-assets/$room315_asset" \
    "$room315_base/$room315_asset"
done
(cd release-assets && sha256sum --strict -c ../SHA256SUMS)
python3 scripts/verify_release.py release-assets
mkdir -p extracted
for room315_archive in release-assets/*.tar.zst; do
  tar --zstd -xf "$room315_archive" -C extracted
done
python3 scripts/verify_extracted_dataset.py \
  extracted/room315_visual_v4_dataset_v1
python3 scripts/reproduce_results.py \
  --release-root extracted/room315_visual_v4_dataset_v1 \
  --source-repo extracted/room315_visual_v4_dataset_v1/source/frozen_tree \
  --split all --full --device cuda \
  --output reproduced-results.json
```

The archive build was independently extracted and replayed on CUDA. All nested
comparisons passed for Validation, Canary and Final Test: primary metrics,
camera counterfactuals, segment calibration and acceptance, plus Final-Test
coverage and runtime thresholds. The machine-readable verification summary is
[`room315_visual_v4_dataset_release_v1/provenance/local_cuda_full_replay.json`](room315_visual_v4_dataset_release_v1/provenance/local_cuda_full_replay.json).
The control package's `README.md` and `ENVIRONMENT.md` give the portable
retraining and environment instructions.

## Relationship to the full-evidence release

This six-asset release supplies the complete labelled Training, Validation,
Development-Canary and Final-Test inputs, both required checkpoints, the
frozen source and the stateless metric-replay tooling. The earlier
[`room315-visual-v4-evidence-2026-08-11`](https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/tag/room315-visual-v4-evidence-2026-08-11)
release instead preserves the accepted runtime bundle, Final-Test evidence and
18 raw positive, fail-closed and post-promotion-smoke MCAP recordings. The two
releases are complementary: use this release to repeat training and offline
evaluation, and the full-evidence release to audit the recorded runtime
campaigns.

## Tag-pinned documentation visibility

At publication time, the original landing record and control package were
present in the recorded release target/tag and in branch
`ali/neuro-symbolic-closed-loop`, but not in the repository's default `main`
branch. The tag remains fixed at its recorded target; its publication-time
`download_release.sh` uses the GitHub CLI. This post-publication branch
correction does not retarget the tag. For an anonymous tag checkout, use the
self-contained `curl` and `SHA256SUMS` flow above, which deliberately does not
invoke that legacy script. This branch-only documentation constraint does not
affect the public visibility, byte identity or anonymous downloadability of the
six release assets.

## Experimental and legal scope

Publication occurred after checkpoint selection, Canary and the immutable
Final-Test attempt had all completed. Later disclosure changes availability,
not the experimental role of any partition or the reported result. The
historical one-shot ledgers are checksum-verified but are not reopened by the
stateless replay runner.

The captures are synthetic Gazebo data and establish no physical-deployment
authority. The repository and release assets are publicly accessible for
inspection and reproducibility; public accessibility does not make this an
open dataset or grant a standalone data or model-weight reuse licence. The
applicable notice is
[`room315_visual_v4_dataset_release_v1/LICENSE_STATUS.md`](room315_visual_v4_dataset_release_v1/LICENSE_STATUS.md)
in the repository control package.
