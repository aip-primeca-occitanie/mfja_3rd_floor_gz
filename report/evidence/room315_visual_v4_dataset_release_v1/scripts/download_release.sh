#!/usr/bin/env bash
set -euo pipefail

destination="${1:-release-assets}"
mkdir -p "$destination"
gh release download v4-seed31520260811-dataset-v1 \
  --repo aip-primeca-occitanie/mfja_3rd_floor_gz \
  --dir "$destination" \
  --pattern '*.tar.zst'
