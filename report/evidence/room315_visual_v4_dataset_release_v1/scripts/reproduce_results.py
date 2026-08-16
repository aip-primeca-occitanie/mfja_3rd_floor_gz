#!/usr/bin/env python3
"""Stateless replay of the published Room 315 visual-state V4 evaluations.

This program deliberately bypasses the original Canary and Final-Test attempt
ledgers.  Those ledgers protected the one-shot experimental protocol; they are
not suitable for an independent reader who needs to recompute already-published
results.  The numerical work still uses the frozen repository's lower-level V4
model, dataset-contract, loss, metric, counterfactual, and calibration code.

The default path is fail closed:

* verify the extracted release checksum manifests;
* verify row/label pairing, safe relative image paths, and every image hash;
* verify the selected checkpoint and its isolation/model/topology contract;
* audit the supplied source checkout against the frozen implementation hashes
  when the Final-Test protocol lock is present; and
* compare the recomputed metrics with the published JSON recursively.

The program never trains, selects a checkpoint, fits on Canary/Final-Test data,
touches an attempt ledger, or changes a runtime.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import math
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


SCHEMA_VERSION = "room315.visual_v4.portable_replay.v1"
FROZEN_EFFECTIVE_CONFIG_FILE_SHA256 = (
    "53f68426ceb0f79fb2c44dbd85302f2d9dad0da364da66959682ae6f3f512371"
)
FROZEN_EFFECTIVE_CONFIG_CANONICAL_SHA256 = (
    "719c4c8eaa3a16c98c1346cc6e5d6259c2e8c77d0a325802b72abc13c9e3b523"
)
CAMERAS = ("left_rail_rgb", "right_rail_rgb")
SPLIT_ALIASES = {
    "validation": "validation",
    "canary": "canary",
    "final-test": "final_test",
    "final_test": "final_test",
}
PUBLISHED_FILES = {
    "validation": {
        "metrics": "final_validation_metrics.json",
        "counterfactuals": "validation_camera_counterfactuals.json",
        "calibration": "validation_segment_calibration.json",
        "acceptance": "validation_acceptance.json",
    },
    "canary": {
        "metrics": "canary_metrics.json",
        "counterfactuals": "canary_camera_counterfactuals.json",
        "calibration": "canary_segment_calibration.json",
        "acceptance": "canary_acceptance.json",
    },
    "final_test": {
        "metrics": "final_test_metrics.json",
        "counterfactuals": "final_test_camera_counterfactuals.json",
        "calibration": "final_test_segment_calibration.json",
        "acceptance": "final_test_acceptance.json",
        "runtime_thresholds": "final_test_runtime_thresholds.json",
        "coverage_audit": "final_test_coverage_audit.json",
    },
}
METRIC_COMPARISON_IGNORED_KEYS = frozenset({
    # These fields were deliberately amended after the lower-level metric pass
    # by the one-shot wrappers.  They are protocol state, not model outputs.
    "acceptance_gates_evaluated",
    "loss_aggregation",
    "pending_evaluations",
    "planning_selection_score_role",
    "selection_key",
    "used_for_checkpoint_selection",
})
CALIBRATION_COMPARISON_IGNORED_KEYS = frozenset()
CALIBRATION_COMPARISON_IGNORED_PATHS = frozenset({"source_artifact.path"})
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


class ReplayError(RuntimeError):
    """Input, provenance, or execution contract failure."""


@dataclass(frozen=True)
class ReleasedSource:
    role: str
    source: dict[str, Any]
    manifest: dict[str, Any]
    manifest_path: Path


@dataclass(frozen=True)
class ArtifactView:
    """Small adapter required by the pure frozen Final-Test report helpers."""

    path: Path
    root: Path

    def as_dict(self) -> dict[str, Any]:
        return {
            "path": self.path.relative_to(self.root).as_posix(),
            "sha256": sha256_file(self.path),
            "bytes": self.path.stat().st_size,
        }


def canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )


def sha256_file(path: Path, cache: dict[Path, str] | None = None) -> str:
    resolved = path.resolve(strict=True)
    if cache is not None and resolved in cache:
        return cache[resolved]
    digest = hashlib.sha256()
    with resolved.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    value = digest.hexdigest()
    if cache is not None:
        cache[resolved] = value
    return value


def fingerprint(path: Path, cache: dict[Path, str] | None = None) -> dict[str, Any]:
    resolved = path.resolve(strict=True)
    return {
        "bytes": resolved.stat().st_size,
        "sha256": sha256_file(resolved, cache),
    }


def read_json_object(path: Path, context: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ReplayError(f"missing {context}: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ReplayError(f"invalid JSON in {context} at {path}:{exc.lineno}") from exc
    if not isinstance(value, dict):
        raise ReplayError(f"{context} must be a JSON object: {path}")
    return value


def read_jsonl(path: Path, context: str) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    try:
        stream = path.open("r", encoding="utf-8")
    except FileNotFoundError as exc:
        raise ReplayError(f"missing {context}: {path}") from exc
    with stream:
        for line_number, line in enumerate(stream, 1):
            try:
                value = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ReplayError(
                    f"invalid JSONL in {context} at {path}:{line_number}"
                ) from exc
            if not isinstance(value, dict):
                raise ReplayError(
                    f"non-object JSONL record in {context} at {path}:{line_number}"
                )
            records.append(value)
    return records


def normalize_release_root(candidate: Path) -> Path:
    root = candidate.expanduser().resolve(strict=True)
    if (root / "manifests").is_dir():
        return root
    children = [
        path
        for path in root.iterdir()
        if path.is_dir() and (path / "manifests").is_dir()
    ]
    if len(children) == 1:
        return children[0].resolve(strict=True)
    raise ReplayError(
        f"release root must contain manifests/ (or one such child): {root}"
    )


def _reject_symlink_components(root: Path, relative: PurePosixPath) -> None:
    current = root
    for component in relative.parts:
        current = current / component
        if current.is_symlink():
            raise ReplayError(f"release path contains a symlink: {current}")


def safe_release_path(
    root: Path,
    raw_value: Any,
    context: str,
    *,
    expect_file: bool = True,
) -> Path:
    value = str(raw_value or "")
    portable = PurePosixPath(value)
    if (
        not value
        or "\\" in value
        or "\x00" in value
        or portable.is_absolute()
        or portable == PurePosixPath(".")
        or any(part in {"", ".", ".."} for part in portable.parts)
    ):
        raise ReplayError(f"unsafe relative path in {context}: {value!r}")
    _reject_symlink_components(root, portable)
    candidate = (root / Path(*portable.parts)).resolve(strict=True)
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise ReplayError(f"path escaped release root in {context}: {value!r}") from exc
    if expect_file and not candidate.is_file():
        raise ReplayError(f"expected regular file in {context}: {candidate}")
    if not expect_file and not candidate.is_dir():
        raise ReplayError(f"expected directory in {context}: {candidate}")
    return candidate


def load_asset_manifests(root: Path) -> tuple[dict[str, ReleasedSource], dict[str, Any]]:
    by_role: dict[str, ReleasedSource] = {}
    model_manifest: dict[str, Any] | None = None
    paths = sorted((root / "manifests").glob("*_asset.json"))
    if not paths:
        raise ReplayError(f"no release asset manifests found under {root / 'manifests'}")
    for path in paths:
        manifest = read_json_object(path, "asset manifest")
        asset = str(manifest.get("asset") or "")
        if asset == "model":
            if model_manifest is not None:
                raise ReplayError("multiple model asset manifests found")
            model_manifest = manifest
        sources = manifest.get("sources") or []
        if not isinstance(sources, list):
            raise ReplayError(f"manifest sources must be a list: {path}")
        for source in sources:
            if not isinstance(source, dict):
                raise ReplayError(f"manifest source must be an object: {path}")
            role = SPLIT_ALIASES.get(str(source.get("role") or ""))
            if role is None:
                continue
            if role in by_role:
                raise ReplayError(f"multiple released sources declare role {role}")
            by_role[role] = ReleasedSource(role, source, manifest, path)
    if model_manifest is None:
        raise ReplayError("model asset manifest is missing")
    return by_role, model_manifest


def verify_checksum_manifest(
    root: Path,
    asset: str,
    cache: dict[Path, str],
) -> dict[str, Any]:
    checksum_path = root / "checksums" / f"{asset}_files.sha256"
    if checksum_path.is_symlink() or not checksum_path.is_file():
        raise ReplayError(f"missing checksum manifest for asset {asset}: {checksum_path}")
    checked = 0
    checked_bytes = 0
    seen: set[str] = set()
    for line_number, line in enumerate(
        checksum_path.read_text(encoding="utf-8").splitlines(), 1
    ):
        try:
            expected, relative = line.split("  ", 1)
        except ValueError as exc:
            raise ReplayError(
                f"malformed checksum line at {checksum_path}:{line_number}"
            ) from exc
        expected = expected.casefold()
        if not SHA256_RE.fullmatch(expected) or relative in seen:
            raise ReplayError(
                f"invalid or duplicate checksum at {checksum_path}:{line_number}"
            )
        seen.add(relative)
        target = safe_release_path(root, relative, "checksum manifest")
        actual = sha256_file(target, cache)
        if actual != expected:
            raise ReplayError(
                f"checksum mismatch for {relative}: {actual} != {expected}"
            )
        checked += 1
        checked_bytes += target.stat().st_size
    if checked == 0:
        raise ReplayError(f"empty checksum manifest: {checksum_path}")
    return {
        "path": checksum_path.relative_to(root).as_posix(),
        "files_verified": checked,
        "bytes_verified": checked_bytes,
        "passed": True,
    }


def expected_fingerprint(
    path: Path,
    declaration: Any,
    context: str,
    cache: dict[Path, str],
) -> dict[str, Any]:
    if not isinstance(declaration, Mapping):
        raise ReplayError(f"missing fingerprint declaration for {context}")
    expected_sha = str(declaration.get("sha256") or "").casefold()
    try:
        expected_bytes = int(declaration["bytes"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ReplayError(f"invalid byte declaration for {context}") from exc
    actual = fingerprint(path, cache)
    if not SHA256_RE.fullmatch(expected_sha):
        raise ReplayError(f"invalid SHA-256 declaration for {context}")
    if actual != {"bytes": expected_bytes, "sha256": expected_sha}:
        raise ReplayError(
            f"fingerprint mismatch for {context}: actual={actual}, "
            f"expected={{'bytes': {expected_bytes}, 'sha256': '{expected_sha}'}}"
        )
    return actual


def explicit_sample_id(record: Mapping[str, Any], context: str) -> str:
    value = str(record.get("sample_id") or "").strip()
    if not value:
        raise ReplayError(f"missing sample_id in {context}")
    return value


def verify_and_load_records(
    released: ReleasedSource,
    root: Path,
    trainer: Any,
    cache: dict[Path, str],
) -> tuple[list[Any], dict[str, Any]]:
    source = released.source
    source_name = str(source.get("name") or released.role)
    rows_path = safe_release_path(root, source.get("rows"), f"{source_name} rows")
    labels_path = safe_release_path(root, source.get("labels"), f"{source_name} labels")
    dataset_root = safe_release_path(
        root,
        source.get("dataset_root"),
        f"{source_name} dataset root",
        expect_file=False,
    )
    packaged = source.get("packaged_files")
    if not isinstance(packaged, Mapping):
        raise ReplayError(f"{source_name} lacks packaged row/label fingerprints")
    rows_fingerprint = expected_fingerprint(
        rows_path, packaged.get("rows"), f"{source_name} rows", cache
    )
    labels_fingerprint = expected_fingerprint(
        labels_path, packaged.get("labels"), f"{source_name} labels", cache
    )

    rows = read_jsonl(rows_path, f"{source_name} rows")
    labels = read_jsonl(labels_path, f"{source_name} labels")
    try:
        expected_count = int(source["records"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ReplayError(f"invalid record count for {source_name}") from exc
    if len(rows) != expected_count or len(labels) != expected_count:
        raise ReplayError(
            f"{source_name} count mismatch: rows={len(rows)}, "
            f"labels={len(labels)}, expected={expected_count}"
        )

    row_ids = [
        explicit_sample_id(record, f"{source_name} rows[{index}]")
        for index, record in enumerate(rows)
    ]
    label_ids = [
        explicit_sample_id(record, f"{source_name} labels[{index}]")
        for index, record in enumerate(labels)
    ]
    if len(set(row_ids)) != len(row_ids):
        raise ReplayError(f"duplicate row sample_id in {source_name}")
    if len(set(label_ids)) != len(label_ids):
        raise ReplayError(f"duplicate label sample_id in {source_name}")
    if row_ids != label_ids:
        raise ReplayError(f"row/label sample_id order differs in {source_name}")
    sample_id_sha = hashlib.sha256(
        ("\n".join(row_ids) + "\n").encode("utf-8")
    ).hexdigest()
    if sample_id_sha != str(source.get("sample_id_sha256") or "").casefold():
        raise ReplayError(f"sample_id sequence hash mismatch in {source_name}")

    label_by_id = dict(zip(label_ids, labels, strict=True))
    records: list[Any] = []
    image_entries: list[tuple[str, str, int, str]] = []
    image_references: set[str] = set()
    for index, row in enumerate(rows):
        sample_id = row_ids[index]
        label = label_by_id[sample_id]
        try:
            model_input = trainer.validate_visual_model_input(
                dict(row), context=f"portable replay {source_name}:{sample_id}"
            )
            normalized = trainer.normalize_visual_state_labels(
                dict(label), context=f"portable replay {source_name}:{sample_id}"
            )
        except Exception as exc:
            raise ReplayError(
                f"camera/label contract failed for {source_name}:{sample_id}: {exc}"
            ) from exc
        references = model_input.get("overhead_images")
        if not isinstance(references, Mapping) or set(references) != set(CAMERAS):
            raise ReplayError(
                f"{source_name}:{sample_id} does not have exactly two rail cameras"
            )
        image_paths: dict[str, Path] = {}
        for camera in CAMERAS:
            reference = str(references[camera])
            if reference in image_references:
                raise ReplayError(
                    f"duplicate image reference within {source_name}: {reference}"
                )
            image_references.add(reference)
            portable = PurePosixPath(reference)
            if (
                not reference
                or "\\" in reference
                or portable.is_absolute()
                or any(part in {"", ".", ".."} for part in portable.parts)
            ):
                raise ReplayError(
                    f"unsafe image reference for {source_name}:{sample_id}: {reference!r}"
                )
            _reject_symlink_components(dataset_root, portable)
            image_path = (dataset_root / Path(*portable.parts)).resolve(strict=True)
            try:
                image_path.relative_to(dataset_root)
            except ValueError as exc:
                raise ReplayError(
                    f"image escaped dataset root for {source_name}:{sample_id}"
                ) from exc
            if not image_path.is_file():
                raise ReplayError(f"missing image: {image_path}")
            digest = sha256_file(image_path, cache)
            image_paths[camera] = image_path
            image_entries.append(
                (reference, digest, image_path.stat().st_size, camera)
            )
        trace = row.get("traceability_metadata") or {}
        if not isinstance(trace, Mapping):
            raise ReplayError(
                f"traceability_metadata must be an object for {source_name}:{sample_id}"
            )
        records.append(trainer.PairedRecord(
            sample_id=sample_id,
            source=source_name,
            role=released.role,
            dataset_root=dataset_root,
            row=dict(row),
            label=dict(label),
            normalized_label=dict(normalized),
            image_paths=image_paths,
            trace=dict(trace),
        ))

    index_digest = hashlib.sha256()
    for reference, digest, size, camera in sorted(image_entries):
        index_digest.update(
            f"{reference}\0{digest}\0{size}\0{camera}\n".encode("utf-8")
        )
    observed_index = index_digest.hexdigest()
    if observed_index != str(source.get("image_index_sha256") or "").casefold():
        raise ReplayError(f"image-index hash mismatch in {source_name}")
    expected_images = expected_count * len(CAMERAS)
    declared_images = int(source.get("image_count", -1))
    if declared_images != expected_images:
        raise ReplayError(f"image count declaration mismatch in {source_name}")
    if len(image_entries) != expected_images:
        raise ReplayError(f"image count mismatch in {source_name}")
    observed_image_bytes = sum(item[2] for item in image_entries)
    if observed_image_bytes != int(source.get("image_bytes", -1)):
        raise ReplayError(f"image byte-count mismatch in {source_name}")
    return records, {
        "name": source_name,
        "role": released.role,
        "records": len(records),
        "rows": rows_fingerprint,
        "labels": labels_fingerprint,
        "sample_id_sha256": sample_id_sha,
        "image_count": len(image_entries),
        "image_bytes": observed_image_bytes,
        "image_index_sha256": observed_index,
        "safe_relative_paths": True,
        "unique_sample_ids": True,
        "row_label_order_identical": True,
        "all_image_hashes_verified": True,
    }


def find_published_file(root: Path, filename: str, context: str) -> Path:
    candidates = []
    for path in root.rglob(filename):
        if path.is_symlink() or not path.is_file():
            continue
        resolved = path.resolve(strict=True)
        try:
            resolved.relative_to(root)
        except ValueError:
            continue
        candidates.append(resolved)
    if not candidates:
        raise ReplayError(f"published {context} is missing ({filename})")
    hashes = {sha256_file(path) for path in candidates}
    if len(hashes) != 1:
        listed = ", ".join(str(path.relative_to(root)) for path in candidates)
        raise ReplayError(f"conflicting published {context} files: {listed}")
    return min(candidates, key=lambda path: (len(path.parts), path.as_posix()))


def source_scripts_directory(source_repo: Path) -> Path:
    repository = source_repo.expanduser().resolve(strict=True)
    scripts = repository / "mfja_robot_control_config" / "scripts"
    if not scripts.is_dir():
        raise ReplayError(f"source repository lacks V4 scripts: {scripts}")
    return scripts


def git_output(repository: Path, *arguments: str) -> str | None:
    completed = subprocess.run(
        ["git", "-C", str(repository), *arguments],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    if completed.returncode != 0:
        return None
    return completed.stdout.strip()


def audit_source_checkout(
    root: Path,
    source_repo: Path,
    model_manifest: Mapping[str, Any],
) -> dict[str, Any]:
    scripts = source_scripts_directory(source_repo)
    revision = git_output(source_repo, "rev-parse", "HEAD")
    dirty = git_output(
        source_repo,
        "status",
        "--porcelain",
        "--",
        "mfja_robot_control_config/scripts",
    )
    manifest_revision = str(model_manifest.get("source_commit") or "").strip()
    audit: dict[str, Any] = {
        "git_revision": revision,
        "asset_manifest_revision": manifest_revision or None,
        "critical_scripts_dirty": bool(dirty),
        "method": "asset_manifest_git_revision",
        "checks": {},
    }

    protocol_candidates = sorted(root.rglob("evaluation_protocol_lock.json"))
    protocol_candidates = [
        path for path in protocol_candidates if path.is_file() and not path.is_symlink()
    ]
    if protocol_candidates:
        hashes = {sha256_file(path) for path in protocol_candidates}
        if len(hashes) != 1:
            raise ReplayError("conflicting evaluation_protocol_lock.json files")
        protocol_path = protocol_candidates[0]
        protocol = read_json_object(protocol_path, "evaluation protocol lock")
        implementation = protocol.get("implementation_artifacts")
        code = implementation.get("code") if isinstance(implementation, Mapping) else None
        if not isinstance(code, Mapping):
            raise ReplayError("protocol lock lacks implementation_artifacts.code")
        checks: dict[str, Any] = {}
        for name, declaration in sorted(code.items()):
            if not isinstance(declaration, Mapping):
                raise ReplayError(f"invalid frozen code declaration: {name}")
            original_path = str(declaration.get("path") or "")
            basename = PurePosixPath(original_path.replace("\\", "/")).name
            expected = str(declaration.get("sha256") or "").casefold()
            if not basename or not SHA256_RE.fullmatch(expected):
                raise ReplayError(f"invalid frozen code fingerprint: {name}")
            candidate_locations = (
                scripts / basename,
                source_repo / "mfja_robot_control_config" / "test" / basename,
            )
            candidate = next((path for path in candidate_locations if path.is_file()), None)
            if candidate is None:
                actual = None
                passed = False
            else:
                actual = sha256_file(candidate)
                passed = actual == expected
            checks[str(name)] = {
                "repository_path": (
                    candidate.relative_to(source_repo).as_posix()
                    if candidate is not None
                    else None
                ),
                "expected_sha256": expected,
                "actual_sha256": actual,
                "passed": passed,
            }
        audit.update({
            "method": "frozen_final_test_implementation_hashes",
            "protocol_lock": protocol_path.relative_to(root).as_posix(),
            "protocol_lock_sha256": next(iter(hashes)),
            "implementation_aggregate_sha256": protocol.get(
                "implementation_aggregate_sha256"
            ),
            "checks": checks,
        })
        passed = bool(checks) and all(item["passed"] for item in checks.values())
    else:
        passed = bool(
            revision
            and manifest_revision
            and revision == manifest_revision
            and not dirty
        )
        audit["checks"] = {
            "git_revision_matches_asset_manifest": {
                "expected": manifest_revision or None,
                "actual": revision,
                "passed": revision == manifest_revision if revision else False,
            },
            "critical_scripts_clean": {
                "passed": not bool(dirty),
            },
        }
    audit["passed"] = passed and not bool(dirty)
    return audit


def import_trainer(scripts: Path) -> Any:
    sys.path.insert(0, str(scripts))
    try:
        return importlib.import_module("room_315_vla_train_v4")
    except Exception as exc:
        raise ReplayError(f"cannot import V4 evaluation code from {scripts}: {exc}") from exc


def load_config(root: Path) -> tuple[dict[str, Any], Path, bool]:
    frozen = root / "model" / "frozen_candidate" / "effective_config.json"
    # Canary and Final-Test output bundles may also contain effective_config.json.
    # The byte-exact frozen candidate is the configuration of record.  Older
    # release drafts only had a path-sanitized metadata copy, retained below as
    # a compatibility fallback for diagnostics.
    if frozen.is_file() and not frozen.is_symlink():
        path = frozen.resolve(strict=True)
        frozen_candidate = True
    else:
        preferred = (
            root
            / "model"
            / "v4"
            / "metadata"
            / "training"
            / "effective_config.json"
        )
        if preferred.is_file() and not preferred.is_symlink():
            path = preferred.resolve(strict=True)
        else:
            path = find_published_file(
                root, "effective_config.json", "effective training configuration"
            )
        frozen_candidate = False
    config = read_json_object(path, "effective training configuration")
    for key in ("model", "image_preprocessing", "training", "loss", "evaluation"):
        if not isinstance(config.get(key), Mapping):
            raise ReplayError(f"effective configuration lacks {key}")
    if frozen_candidate:
        file_sha = sha256_file(path)
        canonical_sha = hashlib.sha256(
            canonical_json(config).encode("utf-8")
        ).hexdigest()
        if file_sha != FROZEN_EFFECTIVE_CONFIG_FILE_SHA256:
            raise ReplayError(
                "byte-exact frozen effective configuration SHA-256 mismatch: "
                f"{file_sha} != {FROZEN_EFFECTIVE_CONFIG_FILE_SHA256}"
            )
        if canonical_sha != FROZEN_EFFECTIVE_CONFIG_CANONICAL_SHA256:
            raise ReplayError(
                "canonical frozen effective configuration SHA-256 mismatch: "
                f"{canonical_sha} != {FROZEN_EFFECTIVE_CONFIG_CANONICAL_SHA256}"
            )
    return config, path, frozen_candidate


def load_checkpoint_and_model(
    root: Path,
    model_manifest: Mapping[str, Any],
    config: Mapping[str, Any],
    trainer: Any,
    device_name: str,
    cache: dict[Path, str],
    *,
    require_effective_config_match: bool,
    released_config_canonical_sha256: str,
) -> tuple[Any, Any, Any, Any, Mapping[str, Any], dict[str, Any]]:
    model_declaration = model_manifest.get("model")
    checkpoint_declaration = (
        model_declaration.get("checkpoint")
        if isinstance(model_declaration, Mapping)
        else None
    )
    if not isinstance(checkpoint_declaration, Mapping):
        raise ReplayError("model asset manifest lacks model.checkpoint")
    checkpoint_path = safe_release_path(
        root, checkpoint_declaration.get("path"), "selected V4 checkpoint"
    )
    checkpoint_fingerprint = expected_fingerprint(
        checkpoint_path,
        checkpoint_declaration,
        "selected V4 checkpoint",
        cache,
    )
    try:
        torch_module, torchvision_module, training_api = trainer.require_training_stack()
    except Exception as exc:
        raise ReplayError(str(exc)) from exc
    if device_name == "auto":
        selected_device = "cuda" if torch_module.cuda.is_available() else "cpu"
    else:
        selected_device = device_name
    if selected_device == "cuda" and not torch_module.cuda.is_available():
        raise ReplayError("--device cuda requested, but CUDA is unavailable")
    device = torch_module.device(selected_device)
    trainer.set_deterministic(torch_module, int(config["training"]["seed"]))
    try:
        model = trainer.build_configured_model(
            config, torch_module, torchvision_module
        ).to(device)
        checkpoint = trainer._torch_load(  # noqa: SLF001
            torch_module, checkpoint_path, map_location="cpu"
        )
    except Exception as exc:
        raise ReplayError(f"cannot construct/load V4 model: {exc}") from exc
    if not isinstance(checkpoint, Mapping):
        raise ReplayError("selected checkpoint is not an object")
    replay_config_sha = hashlib.sha256(
        canonical_json(config).encode("utf-8")
    ).hexdigest()
    checks = {
        "schema_version": checkpoint.get("schema_version")
        == trainer.CHECKPOINT_SCHEMA_VERSION,
        "model_kind": checkpoint.get("model_kind") == trainer.V4_MODEL_KIND,
        "slot_order": tuple(checkpoint.get("slot_order") or ())
        == tuple(trainer.FIXED_IDENTITIES),
        "segment_order": tuple(checkpoint.get("segment_order") or ())
        == tuple(trainer.SEGMENT_CLASSES),
        "selected_on_validation_only": checkpoint.get("checkpoint_selection_role")
        == "validation_only",
        "canary_unseen_at_selection": checkpoint.get("canary_seen") is False,
        "test_unseen_at_selection": checkpoint.get("test_seen") is False,
        "train_class_weights_present": checkpoint.get("class_weights_by_side")
        is not None,
        "topology_lengths_present": isinstance(
            checkpoint.get("topology_lengths_by_side"), Mapping
        ),
    }
    if require_effective_config_match:
        checks["effective_config_sha256"] = (
            checkpoint.get("effective_config_sha256")
            == released_config_canonical_sha256
        )
    public_topology = checkpoint.get("public_topology_contract")
    if not isinstance(public_topology, Mapping):
        checks["embedded_public_topology"] = False
    else:
        checks["embedded_public_topology"] = (
            public_topology.get("lengths_by_side")
            == checkpoint.get("topology_lengths_by_side")
            and public_topology.get("fingerprint_sha256")
            == checkpoint.get("topology_length_mapping_fingerprint_sha256")
        )
    failed = sorted(name for name, value in checks.items() if not value)
    if failed:
        raise ReplayError(f"selected checkpoint contract failed: {failed}")
    try:
        model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    except Exception as exc:
        raise ReplayError(f"strict checkpoint state_dict load failed: {exc}") from exc
    return (
        torch_module,
        training_api,
        device,
        model,
        checkpoint,
        {
            "path": checkpoint_path.relative_to(root).as_posix(),
            **checkpoint_fingerprint,
            "epoch": int(checkpoint.get("epoch", -1)),
            "contract_checks": checks,
            "strict_state_dict_load": True,
            "checkpoint_effective_config_sha256": checkpoint.get(
                "effective_config_sha256"
            ),
            "released_config_canonical_sha256": released_config_canonical_sha256,
            "replay_config_canonical_sha256": replay_config_sha,
            "byte_exact_frozen_config_used": require_effective_config_match,
            "config_hash_difference_expected_from_path_sanitization": (
                not require_effective_config_match
                and checkpoint.get("effective_config_sha256")
                != released_config_canonical_sha256
            ),
        },
    )


def compare_values(
    expected: Any,
    actual: Any,
    *,
    abs_tol: float,
    rel_tol: float,
    ignored_keys: frozenset[str],
    max_mismatches: int,
    ignored_paths: frozenset[str] = frozenset(),
) -> dict[str, Any]:
    mismatches: list[dict[str, Any]] = []

    def add(path: str, expected_value: Any, actual_value: Any, reason: str) -> None:
        if len(mismatches) < max_mismatches:
            mismatches.append({
                "path": path or "$",
                "expected": expected_value,
                "actual": actual_value,
                "reason": reason,
            })

    def visit(expected_value: Any, actual_value: Any, path: str) -> None:
        if isinstance(expected_value, Mapping):
            if not isinstance(actual_value, Mapping):
                add(path, expected_value, actual_value, "type mismatch")
                return
            for key, child in expected_value.items():
                name = str(key)
                child_path = f"{path}.{name}" if path else name
                if name in ignored_keys or child_path in ignored_paths:
                    continue
                if key not in actual_value:
                    add(child_path, child, None, "missing recomputed key")
                else:
                    visit(child, actual_value[key], child_path)
            return
        if isinstance(expected_value, list):
            if not isinstance(actual_value, (list, tuple)):
                add(path, expected_value, actual_value, "type mismatch")
                return
            if len(expected_value) != len(actual_value):
                add(path, len(expected_value), len(actual_value), "list length mismatch")
                return
            for index, child in enumerate(expected_value):
                visit(child, actual_value[index], f"{path}[{index}]")
            return
        if (
            isinstance(expected_value, (int, float))
            and not isinstance(expected_value, bool)
            and isinstance(actual_value, (int, float))
            and not isinstance(actual_value, bool)
        ):
            if isinstance(expected_value, int) and isinstance(actual_value, int):
                if expected_value != actual_value:
                    add(path, expected_value, actual_value, "integer mismatch")
                return
            expected_float = float(expected_value)
            actual_float = float(actual_value)
            if (
                not math.isfinite(expected_float)
                or not math.isfinite(actual_float)
                or not math.isclose(
                    expected_float,
                    actual_float,
                    rel_tol=rel_tol,
                    abs_tol=abs_tol,
                )
            ):
                add(path, expected_value, actual_value, "numeric tolerance exceeded")
            return
        if expected_value != actual_value:
            add(path, expected_value, actual_value, "value mismatch")

    visit(expected, actual, "")
    return {
        "passed": not mismatches,
        "absolute_tolerance": abs_tol,
        "relative_tolerance": rel_tol,
        "mismatch_count_reported": len(mismatches),
        "mismatches_truncated": len(mismatches) >= max_mismatches,
        "mismatches": mismatches,
    }


def calibration_view(
    logits: Any,
    targets: Any,
    mask: Any,
    *,
    temperature: float,
    coverages: Sequence[float],
    ece_bins: int,
    calibration_api: Any,
    torch_module: Any,
) -> dict[str, Any]:
    visible_count = int(mask.sum())
    if visible_count <= 0:
        return {
            "available": False,
            "visible_count": 0,
            "temperature": temperature,
            "nll": None,
            "ece": None,
            "accuracy": None,
            "mean_confidence": None,
            "selective_curve": [],
        }
    nll = calibration_api.segment_negative_log_likelihood(
        logits, targets, mask, temperature=temperature
    )
    ece = calibration_api.segment_expected_calibration_error(
        logits,
        targets,
        mask,
        temperature=temperature,
        ece_bins=ece_bins,
    )
    curve = calibration_api.segment_selective_accuracy_curve(
        logits,
        targets,
        mask,
        temperature=temperature,
        coverages=coverages,
    )
    probabilities = torch_module.softmax(
        logits.detach().to(dtype=torch_module.float64) / temperature,
        dim=-1,
    )
    confidence, prediction = probabilities.max(dim=-1)
    visible_confidence = confidence[mask]
    visible_correct = prediction[mask].eq(targets[mask])
    return {
        "available": True,
        "visible_count": visible_count,
        "temperature": temperature,
        "nll": float(nll),
        "ece": float(ece),
        "accuracy": float(visible_correct.to(dtype=torch_module.float64).mean()),
        "mean_confidence": float(visible_confidence.mean()),
        "selective_curve": curve,
    }


def evaluate_final_test_calibration(
    root: Path,
    model: Any,
    records: Sequence[Any],
    config: Mapping[str, Any],
    validation_calibration: Mapping[str, Any],
    trainer: Any,
    torch_module: Any,
    device: Any,
    epoch: int,
) -> dict[str, Any]:
    try:
        calibration_api = importlib.import_module("room_315_visual_calibration_v4")
        logits, targets, visibility = trainer._collect_segment_calibration_tensors(  # noqa: SLF001
            model,
            records,
            config,
            torch_module=torch_module,
            device=device,
            epoch=epoch,
        )
    except Exception as exc:
        raise ReplayError(f"cannot compute Final-Test calibration: {exc}") from exc
    temperature = float(validation_calibration["temperature"])
    coverages = tuple(validation_calibration["coverage_targets"])
    ece_bins = int(validation_calibration["ece_bins"])
    visibility = visibility.to(dtype=torch_module.bool)
    slot_indexes = torch_module.arange(len(trainer.FIXED_IDENTITIES)).unsqueeze(0)
    per_side: dict[str, Any] = {}
    for side in trainer.SIDES:
        side_mask = visibility & (
            (slot_indexes < 4).expand_as(visibility)
            if side == "left"
            else (slot_indexes >= 4).expand_as(visibility)
        )
        per_side[side] = calibration_view(
            logits,
            targets,
            side_mask,
            temperature=temperature,
            coverages=coverages,
            ece_bins=ece_bins,
            calibration_api=calibration_api,
            torch_module=torch_module,
        )
    global_view = calibration_view(
        logits,
        targets,
        visibility,
        temperature=temperature,
        coverages=coverages,
        ece_bins=ece_bins,
        calibration_api=calibration_api,
        torch_module=torch_module,
    )
    source_path = find_published_file(
        root,
        PUBLISHED_FILES["validation"]["calibration"],
        "Validation calibration source",
    )
    return {
        "schema_version": "room315.visual_v4.final_test_fixed_calibration.v1",
        "data_role": "sealed_final_test_only",
        "source_temperature_role": "validation",
        "source_artifact": ArtifactView(source_path, root).as_dict(),
        "temperature": temperature,
        "fit_performed": False,
        "threshold_selection_performed": False,
        "coverage_targets": list(coverages),
        "ece_bins": ece_bins,
        **global_view,
        "per_side": per_side,
    }


def canary_acceptance_inputs(
    root: Path,
    published_acceptance: Mapping[str, Any],
) -> tuple[float, tuple[str, ...], dict[str, Any]]:
    baseline_candidates = sorted(root.rglob("approved_v3_canary_baseline.json"))
    baseline_candidates = [
        path for path in baseline_candidates if path.is_file() and not path.is_symlink()
    ]
    baseline_source: str
    if baseline_candidates:
        hashes = {sha256_file(path) for path in baseline_candidates}
        if len(hashes) != 1:
            raise ReplayError("conflicting approved V3 Canary baseline files")
        baseline_path = baseline_candidates[0]
        baseline = read_json_object(baseline_path, "approved V3 Canary baseline")
        try:
            loaded_accuracy = float(baseline["loaded_accuracy"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ReplayError("Canary baseline lacks loaded_accuracy") from exc
        baseline_source = baseline_path.relative_to(root).as_posix()
    else:
        # Older release drafts embedded the frozen scalar only in the immutable
        # acceptance artifact.  This remains an auditable fallback because that
        # artifact is covered by the model asset checksum manifest.
        try:
            gate = published_acceptance["per_gate"][
                "loaded_accuracy.maximum_approved_v3_drop"
            ]
            loaded_accuracy = float(
                gate["evidence"]["approved_v3_loaded_accuracy"]
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ReplayError(
                "published Canary evidence lacks the approved V3 loaded baseline"
            ) from exc
        baseline_source = "canary_acceptance.json gate evidence"

    contract_path = find_published_file(
        root, "canary_coverage_contract.json", "Canary coverage contract"
    )
    contract = read_json_object(contract_path, "Canary coverage contract")
    densities = contract.get("required_scene_presence_densities")
    if not isinstance(densities, list) or not densities:
        raise ReplayError("Canary coverage contract lacks required presence densities")
    return (
        loaded_accuracy,
        tuple(str(value) for value in densities),
        {
            "approved_v3_loaded_accuracy_source": baseline_source,
            "coverage_contract": contract_path.relative_to(root).as_posix(),
        },
    )


def evaluate_final_test_acceptance_family(
    *,
    root: Path,
    records: Sequence[Any],
    metrics: Mapping[str, Any],
    counterfactuals: Mapping[str, Any],
    config: Mapping[str, Any],
    validation_calibration: Mapping[str, Any],
    model: Any,
    checkpoint: Mapping[str, Any],
    trainer: Any,
    torch_module: Any,
    device: Any,
) -> dict[str, Any]:
    try:
        final_api = importlib.import_module("room_315_visual_final_test_v4")
        acceptance_api = importlib.import_module("room_315_visual_acceptance_v4")
    except Exception as exc:
        raise ReplayError(f"cannot import frozen Final-Test report helpers: {exc}") from exc

    published_coverage_path = find_published_file(
        root,
        PUBLISHED_FILES["final_test"]["coverage_audit"],
        "Final-Test coverage audit",
    )
    published_coverage = read_json_object(
        published_coverage_path, "published Final-Test coverage audit"
    )
    coverage_contract = published_coverage.get("coverage_contract")
    if not isinstance(coverage_contract, Mapping):
        raise ReplayError("published Final-Test coverage audit lacks its contract")
    try:
        coverage_contract = final_api._validate_coverage_contract(  # noqa: SLF001
            coverage_contract
        )
        for record in records:
            final_api._verify_fresh_row_metadata(  # noqa: SLF001
                record.row,
                record.label,
                sample_id=record.sample_id,
            )
        support = final_api._compute_support_summary(records)  # noqa: SLF001
        coverage_audit = final_api._validate_support_coverage(  # noqa: SLF001
            support, coverage_contract
        )
        tensors = final_api._collect_confidence_tensors(  # noqa: SLF001
            model,
            records,
            config,
            torch_module=torch_module,
            device=device,
            epoch=int(checkpoint.get("epoch", 0)),
        )
    except Exception as exc:
        raise ReplayError(f"Final-Test support/confidence replay failed: {exc}") from exc

    runtime_manifest_path = find_published_file(
        root, "runtime_promotion_manifest.json", "frozen runtime promotion manifest"
    )
    runtime_manifest = read_json_object(
        runtime_manifest_path, "frozen runtime promotion manifest"
    )

    class BundleView:
        pass

    bundle = BundleView()
    bundle.runtime_manifest = runtime_manifest
    bundle.validation_calibration = dict(validation_calibration)
    bundle.artifacts = {
        "runtime_promotion_manifest": ArtifactView(runtime_manifest_path, root)
    }
    try:
        runtime_thresholds = final_api._runtime_threshold_report(  # noqa: SLF001
            tensors, bundle, torch_module=torch_module
        )
        approved_baseline = config.get("approved_v3_validation_baseline")
        if not isinstance(approved_baseline, Mapping):
            raise ReplayError("effective config lacks approved V3 Validation baseline")
        base_acceptance = acceptance_api.evaluate_visual_acceptance_v4(
            metrics,
            config["pilot_acceptance_gates"],
            counterfactual_report=counterfactuals,
            approved_v3_loaded_accuracy=float(approved_baseline["loaded_accuracy"]),
            required_scene_presence_densities=tuple(
                coverage_contract["required_scene_presence_densities"]
            ),
        )
        acceptance = final_api._combine_final_test_acceptance(  # noqa: SLF001
            base_acceptance,
            runtime_thresholds,
            coverage_audit,
            coverage_contract,
        )
    except ReplayError:
        raise
    except Exception as exc:
        raise ReplayError(f"Final-Test acceptance replay failed: {exc}") from exc
    return {
        "coverage_audit": coverage_audit,
        "published_coverage_audit_path": published_coverage_path,
        "runtime_thresholds": runtime_thresholds,
        "runtime_manifest_path": runtime_manifest_path,
        "acceptance": acceptance,
    }


def atomic_write_json(path: Path, value: Any) -> None:
    target = path.expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{target.name}.", suffix=".tmp", dir=target.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary_name, target)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def selected_splits(value: str) -> list[str]:
    if value == "all":
        return ["validation", "canary", "final_test"]
    normalized = SPLIT_ALIASES.get(value)
    if normalized is None:
        raise ReplayError(f"unsupported split: {value}")
    return [normalized]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Statelessly recompute and compare the published Room 315 V4 "
            "Validation, Canary, and Final-Test results."
        )
    )
    parser.add_argument(
        "--release-root",
        type=Path,
        required=True,
        help="extracted release dataset root (or its single-child parent)",
    )
    parser.add_argument(
        "--source-repo",
        type=Path,
        required=True,
        help="mfja_3rd_floor_gz checkout at the frozen evaluation revision",
    )
    parser.add_argument(
        "--split",
        choices=("validation", "canary", "final-test", "all"),
        default="all",
    )
    parser.add_argument(
        "--device", choices=("auto", "cpu", "cuda"), default="auto"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("room315_visual_v4_replay_report.json"),
    )
    parser.add_argument(
        "--counterfactuals",
        action="store_true",
        help="also recompute and compare camera counterfactual reports",
    )
    parser.add_argument(
        "--calibration",
        action="store_true",
        help="also recompute and compare segment calibration reports",
    )
    parser.add_argument(
        "--acceptance",
        action="store_true",
        help=(
            "also recompute and compare acceptance gates (and Final-Test "
            "coverage/runtime-threshold reports)"
        ),
    )
    parser.add_argument(
        "--full",
        action="store_true",
        help="recompute metrics, counterfactuals, calibration, and acceptance",
    )
    parser.add_argument(
        "--skip-asset-checksums",
        action="store_true",
        help=(
            "skip whole-asset checksum manifests (row/label/checkpoint and every "
            "referenced image are still hashed)"
        ),
    )
    parser.add_argument(
        "--allow-source-mismatch",
        action="store_true",
        help="run despite a source revision/implementation hash mismatch",
    )
    parser.add_argument("--absolute-tolerance", type=float, default=1.0e-6)
    parser.add_argument("--relative-tolerance", type=float, default=1.0e-6)
    parser.add_argument("--max-mismatches", type=int, default=100)
    parser.add_argument(
        "--num-workers",
        type=int,
        help="override the frozen DataLoader worker count (use 0 for portability)",
    )
    return parser


def run(args: argparse.Namespace) -> tuple[dict[str, Any], bool]:
    if args.absolute_tolerance < 0.0 or args.relative_tolerance < 0.0:
        raise ReplayError("comparison tolerances must be non-negative")
    if args.max_mismatches <= 0:
        raise ReplayError("--max-mismatches must be positive")
    if args.num_workers is not None and args.num_workers < 0:
        raise ReplayError("--num-workers must be non-negative")

    root = normalize_release_root(args.release_root)
    source_repo = args.source_repo.expanduser().resolve(strict=True)
    requested = selected_splits(args.split)
    include_acceptance = bool(args.acceptance or args.full)
    include_counterfactuals = bool(
        args.counterfactuals or args.full or include_acceptance
    )
    include_calibration = bool(args.calibration or args.full)
    released_by_role, model_manifest = load_asset_manifests(root)
    missing = [role for role in requested if role not in released_by_role]
    if missing:
        raise ReplayError(f"requested split assets are missing: {missing}")

    cache: dict[Path, str] = {}
    source_audit = audit_source_checkout(root, source_repo, model_manifest)
    if not source_audit["passed"] and not args.allow_source_mismatch:
        failed = [
            name
            for name, item in source_audit.get("checks", {}).items()
            if not item.get("passed")
        ]
        raise ReplayError(
            "source checkout does not match frozen evaluation code; "
            f"failed={failed}. Check out the frozen revision or pass "
            "--allow-source-mismatch for a non-exact diagnostic run."
        )
    scripts = source_scripts_directory(source_repo)
    trainer = import_trainer(scripts)

    checksum_audits: dict[str, Any] = {}
    if not args.skip_asset_checksums:
        assets = {"model"}
        if (root / "manifests" / "source_asset.json").is_file():
            assets.add("source")
        assets.update(str(released_by_role[role].manifest["asset"]) for role in requested)
        for asset in sorted(assets):
            checksum_audits[asset] = verify_checksum_manifest(root, asset, cache)

    config, config_path, frozen_candidate_config = load_config(root)
    released_config_canonical_sha = hashlib.sha256(
        canonical_json(config).encode("utf-8")
    ).hexdigest()
    if args.num_workers is not None:
        config = json.loads(json.dumps(config))
        config["training"]["num_workers"] = args.num_workers
        config["training"]["persistent_workers"] = (
            args.num_workers > 0
            and bool(config["training"].get("persistent_workers", False))
        )
    (
        torch_module,
        training_api,
        device,
        model,
        checkpoint,
        checkpoint_audit,
    ) = load_checkpoint_and_model(
        root,
        model_manifest,
        config,
        trainer,
        args.device,
        cache,
        require_effective_config_match=frozen_candidate_config,
        released_config_canonical_sha256=released_config_canonical_sha,
    )
    try:
        current_topology = trainer.load_public_topology_contract(config)
    except Exception as exc:
        raise ReplayError(f"cannot load public topology contract: {exc}") from exc
    if current_topology != checkpoint.get("public_topology_contract"):
        raise ReplayError(
            "source checkout public topology differs from the selected checkpoint"
        )

    report: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "status": "running",
        "protocol": {
            "stateless": True,
            "training_performed": False,
            "checkpoint_selection_performed": False,
            "attempt_ledger_read_or_written": False,
            "runtime_changed": False,
            "canary_or_final_test_refit_performed": False,
        },
        "release_root": str(root),
        "source_repository": str(source_repo),
        "source_audit": source_audit,
        "source_mismatch_override": bool(args.allow_source_mismatch),
        "asset_checksum_audits": checksum_audits,
        "config": {
            "path": config_path.relative_to(root).as_posix(),
            "sha256": sha256_file(config_path, cache),
            "byte_exact_frozen_candidate": frozen_candidate_config,
            "released_canonical_sha256": released_config_canonical_sha,
            "replay_canonical_sha256": hashlib.sha256(
                canonical_json(config).encode("utf-8")
            ).hexdigest(),
            "data_loader_workers": int(config["training"].get("num_workers", 0)),
        },
        "checkpoint": checkpoint_audit,
        "environment": {
            "python": sys.version.split()[0],
            "torch": str(torch_module.__version__),
            "torchvision": str(importlib.import_module("torchvision").__version__),
            "device": str(device),
            "cuda_available": bool(torch_module.cuda.is_available()),
            "cuda_runtime": str(torch_module.version.cuda),
            "cudnn": (
                int(torch_module.backends.cudnn.version())
                if torch_module.backends.cudnn.version() is not None
                else None
            ),
        },
        "comparison_tolerances": {
            "absolute": args.absolute_tolerance,
            "relative": args.relative_tolerance,
        },
        "result_family_policy": {
            "recomputed": [
                "primary_metrics",
                *(["camera_counterfactuals"] if include_counterfactuals else []),
                *(["segment_calibration"] if include_calibration else []),
                *(
                    [
                        "acceptance_gates",
                        "final_test_dataset_coverage",
                        "final_test_runtime_thresholds",
                    ]
                    if include_acceptance
                    else []
                ),
            ],
            "integrity_verified_but_not_reexecuted": [
                "one_shot_attempt_ledgers",
                "historical_disjointness_declarations",
                "capture_and_finalization_protocol_state",
                "training_history_and_wall_clock_metadata",
                "runtime_promotion_or_closed_loop_actions",
            ],
            "reason_not_reexecuted": (
                "protocol/ledger artifacts describe historical state; their bytes are "
                "verified, while this stateless tool recomputes numerical results"
            ),
        },
        "splits": {},
    }
    all_comparisons_passed = bool(source_audit["passed"])
    validation_calibration: dict[str, Any] | None = None
    if (
        (include_calibration or include_acceptance)
        and any(role in requested for role in ("canary", "final_test"))
    ):
        validation_calibration_path = find_published_file(
            root,
            PUBLISHED_FILES["validation"]["calibration"],
            "Validation calibration",
        )
        validation_calibration = read_json_object(
            validation_calibration_path, "published Validation calibration"
        )

    for role in requested:
        records, data_audit = verify_and_load_records(
            released_by_role[role], root, trainer, cache
        )
        try:
            topology_label_audit = trainer.audit_labels_against_public_topology(
                [records], current_topology
            )
            metrics = trainer.evaluate_model(
                model,
                records,
                config,
                torch_module=torch_module,
                training_api=training_api,
                device=device,
                class_weights=checkpoint["class_weights_by_side"],
                topology_lengths=checkpoint["topology_lengths_by_side"],
                epoch=int(checkpoint.get("epoch", 0)),
                require_full_side_segment_support=True,
            )
        except Exception as exc:
            raise ReplayError(f"{role} primary evaluation failed: {exc}") from exc
        published_metrics_path = find_published_file(
            root, PUBLISHED_FILES[role]["metrics"], f"{role} metrics"
        )
        published_metrics = read_json_object(
            published_metrics_path, f"published {role} metrics"
        )
        metrics_comparison = compare_values(
            published_metrics,
            metrics,
            abs_tol=args.absolute_tolerance,
            rel_tol=args.relative_tolerance,
            ignored_keys=METRIC_COMPARISON_IGNORED_KEYS,
            max_mismatches=args.max_mismatches,
        )
        split_report: dict[str, Any] = {
            "input_audit": data_audit,
            "topology_label_audit": topology_label_audit,
            "metrics": metrics,
            "published_metrics": published_metrics_path.relative_to(root).as_posix(),
            "metrics_comparison": metrics_comparison,
        }
        all_comparisons_passed &= bool(metrics_comparison["passed"])

        counterfactuals: dict[str, Any] | None = None
        if include_counterfactuals:
            try:
                counterfactuals = trainer.evaluate_camera_counterfactuals(
                    model,
                    records,
                    config,
                    torch_module=torch_module,
                    training_api=training_api,
                    device=device,
                    topology_lengths=checkpoint["topology_lengths_by_side"],
                    epoch=int(checkpoint.get("epoch", 0)),
                )
            except Exception as exc:
                raise ReplayError(f"{role} counterfactual evaluation failed: {exc}") from exc
            published_path = find_published_file(
                root,
                PUBLISHED_FILES[role]["counterfactuals"],
                f"{role} counterfactuals",
            )
            published = read_json_object(
                published_path, f"published {role} counterfactuals"
            )
            comparison = compare_values(
                published,
                counterfactuals,
                abs_tol=args.absolute_tolerance,
                rel_tol=args.relative_tolerance,
                ignored_keys=frozenset(),
                max_mismatches=args.max_mismatches,
            )
            split_report["counterfactuals"] = counterfactuals
            split_report["published_counterfactuals"] = (
                published_path.relative_to(root).as_posix()
            )
            split_report["counterfactual_comparison"] = comparison
            all_comparisons_passed &= bool(comparison["passed"])

        if include_calibration:
            try:
                if role == "validation":
                    calibration = trainer.fit_validation_segment_calibration(
                        model,
                        records,
                        config,
                        torch_module=torch_module,
                        device=device,
                        epoch=int(checkpoint.get("epoch", 0)),
                    )
                    validation_calibration = calibration
                elif role == "canary":
                    if validation_calibration is None:
                        raise ReplayError("published Validation calibration is unavailable")
                    calibration = trainer.evaluate_fixed_segment_calibration(
                        model,
                        records,
                        config,
                        temperature=float(validation_calibration["temperature"]),
                        data_role="canary",
                        torch_module=torch_module,
                        device=device,
                        epoch=int(checkpoint.get("epoch", 0)),
                    )
                else:
                    if validation_calibration is None:
                        raise ReplayError("published Validation calibration is unavailable")
                    calibration = evaluate_final_test_calibration(
                        root,
                        model,
                        records,
                        config,
                        validation_calibration,
                        trainer,
                        torch_module,
                        device,
                        int(checkpoint.get("epoch", 0)),
                    )
            except Exception as exc:
                if isinstance(exc, ReplayError):
                    raise
                raise ReplayError(f"{role} calibration evaluation failed: {exc}") from exc
            published_path = find_published_file(
                root,
                PUBLISHED_FILES[role]["calibration"],
                f"{role} calibration",
            )
            published = read_json_object(
                published_path, f"published {role} calibration"
            )
            comparison = compare_values(
                published,
                calibration,
                abs_tol=args.absolute_tolerance,
                rel_tol=args.relative_tolerance,
                ignored_keys=CALIBRATION_COMPARISON_IGNORED_KEYS,
                max_mismatches=args.max_mismatches,
                ignored_paths=CALIBRATION_COMPARISON_IGNORED_PATHS,
            )
            split_report["calibration"] = calibration
            split_report["published_calibration"] = (
                published_path.relative_to(root).as_posix()
            )
            split_report["calibration_comparison"] = comparison
            all_comparisons_passed &= bool(comparison["passed"])

        if include_acceptance:
            if counterfactuals is None:
                raise ReplayError(f"{role} acceptance requires counterfactual results")
            published_acceptance_path = find_published_file(
                root,
                PUBLISHED_FILES[role]["acceptance"],
                f"{role} acceptance",
            )
            published_acceptance = read_json_object(
                published_acceptance_path, f"published {role} acceptance"
            )
            acceptance_input_audit: dict[str, Any] = {}
            if role == "final_test":
                if validation_calibration is None:
                    raise ReplayError(
                        "published Validation calibration is required for "
                        "Final-Test runtime thresholds"
                    )
                family = evaluate_final_test_acceptance_family(
                    root=root,
                    records=records,
                    metrics=metrics,
                    counterfactuals=counterfactuals,
                    config=config,
                    validation_calibration=validation_calibration,
                    model=model,
                    checkpoint=checkpoint,
                    trainer=trainer,
                    torch_module=torch_module,
                    device=device,
                )
                acceptance = family["acceptance"]

                published_coverage = read_json_object(
                    family["published_coverage_audit_path"],
                    "published Final-Test coverage audit",
                )
                coverage_comparison = compare_values(
                    published_coverage,
                    family["coverage_audit"],
                    abs_tol=args.absolute_tolerance,
                    rel_tol=args.relative_tolerance,
                    ignored_keys=frozenset(),
                    max_mismatches=args.max_mismatches,
                )
                split_report["coverage_audit"] = family["coverage_audit"]
                split_report["published_coverage_audit"] = (
                    family["published_coverage_audit_path"]
                    .relative_to(root)
                    .as_posix()
                )
                split_report["coverage_audit_comparison"] = coverage_comparison
                all_comparisons_passed &= bool(coverage_comparison["passed"])

                published_runtime_path = find_published_file(
                    root,
                    PUBLISHED_FILES[role]["runtime_thresholds"],
                    "Final-Test runtime thresholds",
                )
                published_runtime = read_json_object(
                    published_runtime_path,
                    "published Final-Test runtime thresholds",
                )
                runtime_comparison = compare_values(
                    published_runtime,
                    family["runtime_thresholds"],
                    abs_tol=args.absolute_tolerance,
                    rel_tol=args.relative_tolerance,
                    ignored_keys=frozenset(),
                    max_mismatches=args.max_mismatches,
                    ignored_paths=frozenset({"runtime_manifest.path"}),
                )
                split_report["runtime_thresholds"] = family["runtime_thresholds"]
                split_report["published_runtime_thresholds"] = (
                    published_runtime_path.relative_to(root).as_posix()
                )
                split_report["runtime_threshold_comparison"] = runtime_comparison
                all_comparisons_passed &= bool(runtime_comparison["passed"])
                acceptance_input_audit = {
                    "approved_v3_loaded_accuracy_source": (
                        "frozen effective config Validation baseline"
                    ),
                    "required_scene_presence_densities_source": (
                        "preregistered Final-Test coverage contract"
                    ),
                    "runtime_manifest": family["runtime_manifest_path"]
                    .relative_to(root)
                    .as_posix(),
                }
            else:
                try:
                    acceptance_api = importlib.import_module(
                        "room_315_visual_acceptance_v4"
                    )
                except Exception as exc:
                    raise ReplayError(f"cannot import V4 acceptance API: {exc}") from exc
                if role == "validation":
                    baseline = config.get("approved_v3_validation_baseline")
                    if not isinstance(baseline, Mapping):
                        raise ReplayError(
                            "effective config lacks approved V3 Validation baseline"
                        )
                    approved_loaded = float(baseline["loaded_accuracy"])
                    required_densities = None
                    acceptance_input_audit = {
                        "approved_v3_loaded_accuracy_source": (
                            "frozen effective config Validation baseline"
                        ),
                        "comparison_normalization": [
                            (
                                "actual-only /inputs/required_scene_presence_densities "
                                "is tolerated because the historical Validation artifact "
                                "predates that informational field"
                            )
                        ],
                    }
                else:
                    (
                        approved_loaded,
                        required_densities,
                        acceptance_input_audit,
                    ) = canary_acceptance_inputs(root, published_acceptance)
                try:
                    acceptance = acceptance_api.evaluate_visual_acceptance_v4(
                        metrics,
                        config["pilot_acceptance_gates"],
                        counterfactual_report=counterfactuals,
                        approved_v3_loaded_accuracy=approved_loaded,
                        required_scene_presence_densities=required_densities,
                    )
                except Exception as exc:
                    raise ReplayError(f"{role} acceptance replay failed: {exc}") from exc

            acceptance_comparison = compare_values(
                published_acceptance,
                acceptance,
                abs_tol=args.absolute_tolerance,
                rel_tol=args.relative_tolerance,
                ignored_keys=frozenset(),
                max_mismatches=args.max_mismatches,
            )
            split_report["acceptance"] = acceptance
            split_report["acceptance_input_audit"] = acceptance_input_audit
            split_report["published_acceptance"] = (
                published_acceptance_path.relative_to(root).as_posix()
            )
            split_report["acceptance_comparison"] = acceptance_comparison
            all_comparisons_passed &= bool(acceptance_comparison["passed"])

        report["splits"][role] = split_report

    report["status"] = "passed" if all_comparisons_passed else "mismatch"
    report["all_requested_comparisons_passed"] = all_comparisons_passed
    return report, all_comparisons_passed


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        report, passed = run(args)
        atomic_write_json(args.output, report)
    except ReplayError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({
        "status": report["status"],
        "output": str(args.output.expanduser().resolve()),
        "splits": list(report["splits"]),
        "device": report["environment"]["device"],
    }, indent=2, sort_keys=True))
    return 0 if passed else 3


if __name__ == "__main__":
    raise SystemExit(main())
