#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/.." && pwd)"
checksums="$repo_root/SHA256SUMS"
base_url="https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/download/v4-seed31520260811-dataset-v1"
destination="${1:-release-assets}"
if (( $# > 0 )); then
  shift
fi
requested_assets=("$@")

if [[ ! -f "$checksums" ]]; then
  echo "missing checksum file: $checksums" >&2
  exit 1
fi
mkdir -p -- "$destination"

sha256_of() {
  local digest remainder
  read -r digest remainder < <(sha256sum -- "$1")
  printf '%s\n' "$digest"
}

is_requested() {
  local candidate
  if (( ${#requested_assets[@]} == 0 )); then
    return 0
  fi
  for candidate in "${requested_assets[@]}"; do
    if [[ "$candidate" == "$1" ]]; then
      return 0
    fi
  done
  return 1
}

selected=0
while read -r expected name extra; do
  [[ -z "${expected:-}" ]] && continue
  if [[ -n "${extra:-}" || ! "$expected" =~ ^[0-9a-f]{64}$ ||
        ! "$name" =~ ^[A-Za-z0-9._-]+\.tar\.zst$ ]]; then
    echo "invalid SHA256SUMS entry" >&2
    exit 1
  fi
  if ! is_requested "$name"; then
    continue
  fi
  selected=$((selected + 1))

  final_path="$destination/$name"
  partial_path="$final_path.part"
  if [[ -f "$final_path" ]]; then
    if [[ "$(sha256_of "$final_path")" == "$expected" ]]; then
      echo "verified existing $name"
      continue
    fi
    echo "existing asset failed SHA-256: $final_path" >&2
    echo "remove or relocate it explicitly before retrying" >&2
    exit 1
  fi

  if [[ -f "$partial_path" &&
        "$(sha256_of "$partial_path")" == "$expected" ]]; then
    mv -- "$partial_path" "$final_path"
    echo "verified completed partial $name"
    continue
  fi

  echo "downloading $name"
  curl --fail --location --retry 5 --retry-delay 2 \
    --retry-connrefused --continue-at - \
    --output "$partial_path" "$base_url/$name"
  actual="$(sha256_of "$partial_path")"
  if [[ "$actual" != "$expected" ]]; then
    echo "downloaded asset failed SHA-256: $name" >&2
    echo "expected $expected" >&2
    echo "actual   $actual" >&2
    echo "partial file retained for inspection: $partial_path" >&2
    exit 1
  fi
  mv -- "$partial_path" "$final_path"
  echo "downloaded and verified $name"
done < "$checksums"

if (( selected == 0 )); then
  echo "no release assets selected" >&2
  exit 1
fi
if (( ${#requested_assets[@]} > 0 &&
      selected != ${#requested_assets[@]} )); then
  echo "one or more requested asset names are not in SHA256SUMS" >&2
  exit 1
fi
