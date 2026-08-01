#!/usr/bin/env python3
"""Static and runtime fail-closed verification for Experiment A."""

from __future__ import annotations

import argparse
import json
import platform
import re
import sys
from collections import Counter
from pathlib import Path
from typing import Any

from PIL import Image

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from experiment_a_core import (  # noqa: E402
    APPROVED_CHECKPOINT_SHA256,
    CATEGORICAL_FIELDS,
    IDENTITIES,
    VECTOR_DIMENSION,
    VISUAL_SCHEMA,
    ExperimentAError,
    assert_disjoint,
    canonical_json,
    categorical_output_index,
    categorical_output_name,
    decode_categorical_target,
    ensure_training_roles,
    expand_path,
    read_json,
    read_jsonl,
    reject_forbidden_artifact,
    row_id,
    sampling_report,
    sha256_file,
    source_balanced_epoch,
    strict_load_approved_checkpoint,
    validate_label,
    validate_target_stats,
    validate_vectorizer,
    vectorize_label,
)
from room_315_visual_model import build_visual_state_model  # noqa: E402


STALE_ENCODER_PATTERNS = (
    re.compile(r"categorical_values\s*\.\s*items\s*\("),
    re.compile(r"vector\s*\.\s*extend\s*\([^\n]*raw\s*=="),
)


def scan_stale_encoder(package_root: Path) -> dict[str, Any]:
    matches = []
    scanned = 0
    for path in sorted(package_root.rglob("*")):
        if not path.is_file() or path.suffix not in {".py", ".sh"}:
            continue
        scanned += 1
        text = path.read_text(encoding="utf-8")
        for pattern in STALE_ENCODER_PATTERNS:
            if pattern.search(text):
                matches.append({
                    "path": path.relative_to(package_root).as_posix(),
                    "pattern": pattern.pattern,
                })
    if matches:
        raise ExperimentAError(f"stale categorical encoder found: {matches}")
    return {"passed": True, "files_scanned": scanned, "matches": []}


def _synthetic_label(identity: str, side: str, block: str, loaded: str) -> dict[str, Any]:
    shuttles = []
    for fixed_identity in IDENTITIES:
        shuttles.append({
            "id": fixed_identity,
            "presence": False,
            "visually_available": False,
            "loaded_state": "unknown",
            "bbox": [0.0, 0.0, 0.0, 0.0],
            "location": {
                "side": "left" if fixed_identity.startswith("L") else "right",
                "block": "unknown",
            },
            "rail_position": {
                "available": False,
                "s_m": 0.0,
                "s_ratio": 0.0,
                "segment_length_m": 0.0,
            },
        })
    shuttle = shuttles[IDENTITIES.index(identity)]
    shuttle.update({
        "presence": True,
        "visually_available": True,
        "loaded_state": loaded,
        "bbox": [10.0, 20.0, 30.0, 40.0],
        "location": {"side": side, "block": block},
        "rail_position": {
            "available": True,
            "s_m": 0.75,
            "s_ratio": 0.25,
            "segment_length_m": 3.0,
        },
    })
    return {
        "visual_state_labels": {
            "schema_version": VISUAL_SCHEMA,
            "shuttles": shuttles,
        }
    }


def semantic_self_test(vectorizer: dict[str, Any]) -> dict[str, Any]:
    cases = (
        ("right", "A34E", "loaded"),
        ("left", "A1E", "empty"),
        ("right", "A23", "empty"),
        ("left", "A12I", "loaded"),
    )
    checked = []
    historical_failures = 0
    numeric_count = len(vectorizer["numeric_keys"])
    historical_field_order = ("loaded_state", "block", "side")
    for identity in IDENTITIES:
        for side, block, loaded in cases:
            row = _synthetic_label(identity, side, block, loaded)
            vector, mask = vectorize_label(row, vectorizer)
            expected = {"side": side, "block": block, "loaded_state": loaded}
            decoded = {
                field: decode_categorical_target(
                    vector, vectorizer, identity, field
                )
                for field in ("side", "block", "loaded_state")
            }
            if decoded != expected:
                raise ExperimentAError(
                    f"synthetic categorical round trip failed: {identity} {decoded}"
                )
            exact_indexes = {
                field: categorical_output_index(
                    vectorizer, identity, field, value
                )
                for field, value in expected.items()
            }
            if any(float(vector[index]) != 1.0 for index in exact_indexes.values()):
                raise ExperimentAError("synthetic categorical exact-index test failed")
            for field, (_, vocabulary) in CATEGORICAL_FIELDS.items():
                active = [
                    float(vector[categorical_output_index(vectorizer, identity, field, value)])
                    for value in vocabulary
                ]
                if sum(active) != 1.0 or any(value not in (0.0, 1.0) for value in active):
                    raise ExperimentAError("synthetic categorical one-hot test failed")

            # Reproduce the historical positional contract using an explicit,
            # deliberately wrong field order.  This is diagnostic-only code;
            # production encoding never derives positions from section order.
            broken = [0.0] * VECTOR_DIMENSION
            cursor = numeric_count
            for fixed_identity in IDENTITIES:
                fixed = row["visual_state_labels"]["shuttles"][IDENTITIES.index(fixed_identity)]
                fixed_raw = {
                    "side": fixed["location"]["side"],
                    "block": fixed["location"]["block"],
                    "loaded_state": fixed["loaded_state"],
                }
                for field in historical_field_order:
                    _, vocabulary = CATEGORICAL_FIELDS[field]
                    for value in vocabulary:
                        broken[cursor] = float(fixed_raw[field] == value)
                        cursor += 1
            try:
                broken_decoded = {
                    field: decode_categorical_target(
                        broken, vectorizer, identity, field
                    )
                    for field in ("side", "block", "loaded_state")
                }
            except ExperimentAError:
                historical_failures += 1
            else:
                if broken_decoded != expected:
                    historical_failures += 1
            checked.append({
                "identity": identity,
                "expected": expected,
                "decoded": decoded,
                "exact_indexes": exact_indexes,
                "present_mask_values": sorted({
                    float(mask[index])
                    for index, name in enumerate(vectorizer["names"])
                    if name.startswith(f"shuttles.{IDENTITIES.index(identity)}.")
                }),
            })
    if historical_failures == 0:
        raise ExperimentAError("semantic self-test did not reject historical encoder")
    return {
        "passed": True,
        "cases_checked": len(checked),
        "identities_checked": list(IDENTITIES),
        "historical_broken_encoder_rejected_cases": historical_failures,
        "historical_broken_encoder_total_cases": len(checked),
        "cases": checked,
    }


def strict_checkpoint_load_audit(config: dict[str, Any]) -> dict[str, Any]:
    try:
        import torch
        import torchvision
    except Exception as exc:
        raise ExperimentAError(f"Torch/TorchVision import failed: {exc}") from exc
    model = build_visual_state_model(
        torch,
        torchvision,
        output_dim=VECTOR_DIMENSION,
        adaptation_mode="partial_finetune",
    )
    checkpoint = strict_load_approved_checkpoint(
        torch_module=torch,
        model=model,
        checkpoint_path=expand_path(config["artifacts"]["approved_checkpoint"]["path"]),
    )
    return {
        **checkpoint["_strict_load_verification"],
        "checkpoint_sha256": APPROVED_CHECKPOINT_SHA256,
        "checkpoint_epoch": int(checkpoint["epoch"]),
    }


def verify_file(spec: dict[str, Any], *, label: str) -> dict[str, Any]:
    path = expand_path(spec["path"])
    if not path.is_file():
        raise ExperimentAError(f"missing {label}: {path}")
    actual = sha256_file(path)
    expected = str(spec["sha256"])
    if actual != expected:
        raise ExperimentAError(f"{label} SHA-256 mismatch: {actual} != {expected}")
    return {"path": str(path), "sha256": actual, "bytes": path.stat().st_size}


def verify_source(name: str, spec: dict[str, Any], *, decode_images: bool) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    rows_file = verify_file(spec["rows_artifact"], label=f"{name} rows")
    labels_file = verify_file(spec["labels_artifact"], label=f"{name} labels")
    rows = read_jsonl(expand_path(spec["rows"])); labels = read_jsonl(expand_path(spec["labels"]))
    expected_count = int(spec["expected_scenarios"])
    if len(rows) != expected_count or len(labels) != expected_count:
        raise ExperimentAError(f"{name} count mismatch: rows={len(rows)}, labels={len(labels)}, expected={expected_count}")
    row_ids = [row_id(row) for row in rows]; label_ids = [row_id(label) for label in labels]
    if len(set(row_ids)) != len(row_ids): raise ExperimentAError(f"duplicate scenario/sample IDs in {name} rows")
    if len(set(label_ids)) != len(label_ids): raise ExperimentAError(f"duplicate scenario/sample IDs in {name} labels")
    if row_ids != label_ids: raise ExperimentAError(f"row/label ordering mismatch in {name}")
    image_root = expand_path(spec["dataset_root"]); decoded = 0; image_count = 0
    presence = Counter()
    for row, label_row in zip(rows, labels):
        label = validate_label(label_row)
        vector, mask = vectorize_label(label_row, read_json(expand_path(spec["vectorizer_path"])))
        if len(vector) != VECTOR_DIMENSION or len(mask) != VECTOR_DIMENSION:
            raise ExperimentAError(f"{name} vector dimension mismatch")
        presence[sum(bool(item.get("presence")) for item in label["shuttles"])] += 1
        refs = row.get("model_input", {}).get("overhead_images", {})
        for camera in ("left_rail_rgb", "right_rail_rgb"):
            ref = str(refs.get(camera) or "")
            if not ref: raise ExperimentAError(f"{name} {row_id(row)} lacks {camera}")
            image = image_root / ref; reject_forbidden_artifact(image)
            if not image.is_file(): raise ExperimentAError(f"missing image: {image}")
            image_count += 1
            if decode_images:
                try:
                    with Image.open(image) as opened:
                        opened.verify()
                except Exception as exc:
                    raise ExperimentAError(f"image decode failed: {image}") from exc
                decoded += 1
    return rows, {
        "rows": len(rows), "labels": len(labels), "images": image_count,
        "images_decoded": decoded, "rows_artifact": rows_file,
        "labels_artifact": labels_file,
        "presence_cardinality": dict(sorted(presence.items())),
    }


def verify_package_manifest(package_root: Path) -> dict[str, Any]:
    manifest_path = package_root / "package_manifest.json"
    if not manifest_path.is_file():
        return {
            "available": False,
            "reason": "emitted only after the static audit succeeds",
        }
    manifest = read_json(manifest_path); checked = 0
    for item in manifest.get("files", []):
        path = package_root / item["path"]
        if not path.is_file() or sha256_file(path) != item["sha256"] or path.stat().st_size != item["bytes"]:
            raise ExperimentAError(f"package manifest mismatch: {item['path']}")
        checked += 1
    return {"available": True, "checked_files": checked, "tree_sha256": manifest.get("tree_sha256")}


def static_audit(config_path: Path, package_root: Path, *, decode_images: bool) -> dict[str, Any]:
    config = read_json(config_path); ensure_training_roles(config)
    if config["model"]["output_dimension"] != VECTOR_DIMENSION or tuple(config["model"]["identity_order"]) != IDENTITIES:
        raise ExperimentAError("model contract mismatch")
    artifacts = {name: verify_file(spec, label=name) for name, spec in config["artifacts"].items()}
    if artifacts["approved_checkpoint"]["sha256"] != APPROVED_CHECKPOINT_SHA256:
        raise ExperimentAError("wrong approved continuation checkpoint")
    vectorizer = read_json(expand_path(config["artifacts"]["vectorizer"]["path"])); validate_vectorizer(vectorizer)
    stats = read_json(expand_path(config["artifacts"]["target_stats"]["path"])); validate_target_stats(stats)
    semantics = semantic_self_test(vectorizer)
    stale_encoder = scan_stale_encoder(package_root)
    sources: dict[str, Any] = {}; rows: dict[str, list[dict[str, Any]]] = {}
    requested_sources = tuple(config.get("verification_sources") or (
        "old_replay", "v3r1_train", "v3r1_validation", "v3r1_canary"
    ))
    allowed_sources = {
        "old_replay", "v3r1_train", "v3r1_validation", "v3r1_canary"
    }
    if not requested_sources or any(
        source not in allowed_sources for source in requested_sources
    ):
        raise ExperimentAError("invalid verification source scope")
    for source_name in requested_sources:
        if source_name not in config["data"]:
            raise ExperimentAError(
                f"verification source is absent from configuration: {source_name}"
            )
        source_spec = dict(config["data"][source_name]); source_spec["vectorizer_path"] = config["artifacts"]["vectorizer"]["path"]
        rows[source_name], sources[source_name] = verify_source(source_name, source_spec, decode_images=decode_images)
    overlap = assert_disjoint(*[(name, rows[name]) for name in rows])
    old_ids = [row_id(row) for row in rows["old_replay"]]; new_ids = [row_id(row) for row in rows["v3r1_train"]]
    sampler = {}
    for epoch in (1, 2):
        first = source_balanced_epoch(old_ids, new_ids, seed=int(config["seed"]), epoch=epoch)
        second = source_balanced_epoch(old_ids, new_ids, seed=int(config["seed"]), epoch=epoch)
        if canonical_json(first) != canonical_json(second): raise ExperimentAError("sampler is not deterministic")
        sampler[str(epoch)] = sampling_report(first)
    if sampler["1"]["selection_fingerprint_sha256"] == sampler["2"]["selection_fingerprint_sha256"]:
        raise ExperimentAError("epoch-specific sampler ordering did not change")
    if any(abs(value - 0.5) > 1e-12 for value in sampler["1"]["source_fractions"].values()):
        raise ExperimentAError("source sampler is not exactly balanced")
    smoke_checks: dict[str, Any] = {"not_applicable": True}
    if config.get("stage") == "smoke":
        smoke = read_json(package_root / "config" / "smoke_selection.json")
        smoke_checks = {
            "old_train_rows": len(smoke["train"]["old_replay"]),
            "new_train_rows": len(smoke["train"]["v3r1_hard_case"]),
            "validation_rows": len(smoke["validation"]),
            "coverage": smoke["coverage"],
        }
        if (smoke_checks["old_train_rows"], smoke_checks["new_train_rows"], smoke_checks["validation_rows"]) != (64, 64, 32):
            raise ExperimentAError("smoke selection counts are invalid")
        required_coverage = ("L4_loaded", "R4_loaded", "exact_L2_L4_R4", "right_slot3_deliberate_offset", "hard_payload")
        if not all(smoke["coverage"].get(item, 0) > 0 for item in required_coverage):
            raise ExperimentAError("smoke selection lacks required hard-case coverage")
    text_files = []
    for path in sorted(package_root.rglob("*")):
        if path.is_file() and path.suffix in {".py", ".json", ".md", ".sh"}:
            text_files.append(path.relative_to(package_root).as_posix())
    return {
        "schema_version": "room315.experiment_a.package_audit.v1",
        "passed": True,
        "training_executed": False,
        "legacy_evaluation_data_accessed": False,
        "approved_checkpoint": artifacts["approved_checkpoint"],
        "artifacts": artifacts,
        "dataset_sources": sources,
        "dataset_overlap_counts": overlap,
        "schema": VISUAL_SCHEMA,
        "vector_dimension": VECTOR_DIMENSION,
        "identity_order": list(IDENTITIES),
        "categorical_semantic_self_test": semantics,
        "stale_encoder_scan": stale_encoder,
        "sampler": sampler,
        "smoke_selection": smoke_checks,
        "data_roles": config["data_roles"],
        "verification_sources": list(requested_sources),
        "package_manifest": verify_package_manifest(package_root),
        "text_file_inventory": text_files,
    }


def runtime_environment() -> dict[str, Any]:
    if platform.machine() != "aarch64": raise ExperimentAError(f"aarch64 required, found {platform.machine()}")
    try:
        import torch
        import torchvision
    except Exception as exc:
        raise ExperimentAError(f"Torch/TorchVision import failed: {exc}") from exc
    if not torch.cuda.is_available(): raise ExperimentAError("CUDA is unavailable")
    gpu = torch.cuda.get_device_name(0)
    if "GH200" not in gpu.upper(): raise ExperimentAError(f"GH200 required, found {gpu}")
    return {"architecture": platform.machine(), "torch": torch.__version__, "torchvision": torchvision.__version__, "cuda": torch.version.cuda, "gpu": gpu}


def main() -> None:
    parser = argparse.ArgumentParser(); parser.add_argument("--config", type=Path, required=True); parser.add_argument("--package-root", type=Path, required=True); parser.add_argument("--output", type=Path); parser.add_argument("--decode-images", action="store_true"); parser.add_argument("--require-gh200", action="store_true"); parser.add_argument("--verify-checkpoint-load", action="store_true")
    args = parser.parse_args()
    report = static_audit(args.config, args.package_root, decode_images=args.decode_images)
    if args.require_gh200: report["runtime_environment"] = runtime_environment()
    if args.verify_checkpoint_load:
        report["strict_checkpoint_load"] = strict_checkpoint_load_audit(
            read_json(args.config)
        )
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output: args.output.write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__": main()
