#!/usr/bin/env python3
"""Balanced, baseline-first local Smoke V2 for Room 315 Experiment A."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import random
import statistics
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from experiment_a_core import (  # noqa: E402
    APPROVED_CHECKPOINT_SHA256,
    IDENTITIES,
    LOCAL_DEFAULT_PROFILE,
    SEED,
    VECTOR_DIMENSION,
    ExperimentAError,
    canonical_json,
    effective_training_config,
    expand_path,
    read_json,
    read_jsonl,
    row_id,
    sampling_report,
    sha256_file,
    source_balanced_epoch,
    strict_load_approved_checkpoint,
    validate_target_stats,
    validate_vectorizer,
    vectorize_label,
)
from experiment_a_train import (  # noqa: E402
    evaluate,
    load_source,
    loader,
    paired_records,
    require_torch,
    set_deterministic,
    visual_loss,
)
from room_315_visual_model import build_visual_state_model  # noqa: E402


SMOKE_V1_ROOT = Path(
    "/home/tiago/room315_experiment_a_local_outputs/"
    "smoke_seed31520260730_attempt1"
)
SMOKE_V1_TREE_SHA256 = (
    "811e7706560b9540dbdf2ec676858382a01f23d7cbc0dd8ed2d5021fc1ad146d"
)
INVALID_SMOKE_V2_ATTEMPT1_ROOT = Path(
    "/home/tiago/room315_experiment_a_local_outputs/"
    "smoke_v2_seed31520260730_attempt1"
)
INVALID_SMOKE_V2_ATTEMPT1_TREE_SHA256 = (
    "3ca035edcc22c046126f0317093439e5a55fa25dde7a21619fe08c7c64a7b051"
)
VALIDATION_SIZE = 128
TRAIN_PER_SOURCE = 128
OLD_REPLAY_EVAL_SIZE = 128
MIN_PAYLOAD_PER_IDENTITY = 8
VALIDATION_SEED = SEED + 404
OLD_TRAIN_SEED = SEED + 501
NEW_TRAIN_SEED = SEED + 502
OLD_EVAL_SEED = SEED + 503


def rows_fingerprint(ids: list[str]) -> str:
    return hashlib.sha256(canonical_json(ids).encode("utf-8")).hexdigest()


def tree_manifest(root: Path) -> dict[str, Any]:
    files = []
    digest = hashlib.sha256()
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(root).as_posix()
        item = {
            "path": relative,
            "bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        }
        files.append(item)
        digest.update(
            f'{relative}\0{item["bytes"]}\0{item["sha256"]}\n'.encode()
        )
    return {"tree_sha256": digest.hexdigest(), "files": files}


def verify_smoke_v1_immutable() -> dict[str, Any]:
    report = tree_manifest(SMOKE_V1_ROOT)
    if report["tree_sha256"] != SMOKE_V1_TREE_SHA256:
        raise ExperimentAError("completed Smoke V1 artifacts changed")
    return report


def verify_invalid_smoke_v2_attempt1_immutable() -> dict[str, Any]:
    report = tree_manifest(INVALID_SMOKE_V2_ATTEMPT1_ROOT)
    if report["tree_sha256"] != INVALID_SMOKE_V2_ATTEMPT1_TREE_SHA256:
        raise ExperimentAError("diagnostic Smoke V2 attempt 1 artifacts changed")
    return report


def deterministic_ids(
    rows: list[dict[str, Any]], count: int, seed: int
) -> list[str]:
    indexes = list(range(len(rows)))
    random.Random(seed).shuffle(indexes)
    return [row_id(rows[index]) for index in indexes[:count]]


def payload_selection_audit(
    rows: list[dict[str, Any]],
    labels: list[dict[str, Any]],
    vectorizer: dict[str, Any],
) -> dict[str, Any]:
    if len(rows) != len(labels):
        raise ExperimentAError("selection rows and labels have different counts")
    payload_side = Counter()
    payload_identity = Counter()
    identity_payload_presence = Counter()
    presence_class = Counter()
    relations = Counter()
    offsets = Counter()
    features = Counter()
    target_one_hot_failures = 0
    categorical_semantic_failures = 0
    masked_absent_failures = 0
    present_mask_failures = 0
    for row, label_row in zip(rows, labels):
        trace = row.get("traceability_metadata") or {}
        presence_class[str(trace.get("presence_class"))] += 1
        relations[str(trace.get("relation_family"))] += 1
        offsets[str(trace.get("target_offset_bucket"))] += 1
        active = set(trace.get("active_identities") or [])
        if {"L4", "R4"}.issubset(active):
            features["combined_L4_R4"] += 1
        if active == {"L2", "L4", "R4"}:
            features["exact_L2_L4_R4"] += 1
        if trace.get("operational_target_name") == "right_slot_3":
            features["deliberate_right_slot3_offset"] += 1
        label = label_row["visual_state_labels"]
        vector, mask = vectorize_label(label_row, vectorizer)
        names = vectorizer["names"]
        for slot, shuttle in enumerate(label["shuttles"]):
            identity = str(shuttle["id"])
            present = bool(shuttle.get("presence")) and bool(
                shuttle.get("visually_available")
            )
            payload = str(shuttle.get("loaded_state"))
            identity_payload_presence[(identity, payload, "present" if present else "absent")] += 1
            slot_indexes = [
                index
                for index, name in enumerate(names)
                if name.startswith(f"shuttles.{slot}.")
            ]
            payload_indexes = [
                names.index(f"shuttles.{slot}.loaded_state==empty"),
                names.index(f"shuttles.{slot}.loaded_state==loaded"),
            ]
            side_indexes = [
                names.index(f"shuttles.{slot}.location.side==left"),
                names.index(f"shuttles.{slot}.location.side==right"),
            ]
            block_indexes = [
                index
                for index, name in enumerate(names)
                if name.startswith(f"shuttles.{slot}.location.block==")
            ]
            if not present:
                if any(float(mask[index]) != 0.0 for index in slot_indexes):
                    masked_absent_failures += 1
                continue
            if any(float(mask[index]) != 1.0 for index in slot_indexes):
                present_mask_failures += 1
            if sum(float(vector[index]) == 1.0 for index in payload_indexes) != 1:
                target_one_hot_failures += 1
            if any(float(vector[index]) not in (0.0, 1.0) for index in payload_indexes):
                target_one_hot_failures += 1
            side = str(shuttle["location"]["side"])
            expected_payload = [
                1.0 if payload == "empty" else 0.0,
                1.0 if payload == "loaded" else 0.0,
            ]
            expected_side = [
                1.0 if side == "left" else 0.0,
                1.0 if side == "right" else 0.0,
            ]
            block = str(shuttle["location"]["block"])
            encoded_block_names = [
                names[index].rsplit("==", 1)[1]
                for index in block_indexes
                if float(vector[index]) == 1.0
            ]
            if [float(vector[index]) for index in payload_indexes] != expected_payload:
                categorical_semantic_failures += 1
            if [float(vector[index]) for index in side_indexes] != expected_side:
                categorical_semantic_failures += 1
            if encoded_block_names != [block]:
                categorical_semantic_failures += 1
            payload_side[(payload, side)] += 1
            payload_identity[(identity, payload)] += 1
            if payload not in {"loaded", "empty"}:
                target_one_hot_failures += 1
    per_identity = {
        identity: {
            "loaded": payload_identity[(identity, "loaded")],
            "empty": payload_identity[(identity, "empty")],
        }
        for identity in IDENTITIES
    }
    minimum_met = all(
        values["loaded"] >= MIN_PAYLOAD_PER_IDENTITY
        and values["empty"] >= MIN_PAYLOAD_PER_IDENTITY
        for values in per_identity.values()
    )
    required_features = {
        "left_loaded": payload_side[("loaded", "left")],
        "left_empty": payload_side[("empty", "left")],
        "right_loaded": payload_side[("loaded", "right")],
        "right_empty": payload_side[("empty", "right")],
        "L4_loaded": payload_identity[("L4", "loaded")],
        "L4_empty": payload_identity[("L4", "empty")],
        "R4_loaded": payload_identity[("R4", "loaded")],
        "R4_empty": payload_identity[("R4", "empty")],
        **dict(features),
    }
    payload_side_present = all(
        payload_side[(payload, side)] > 0
        for payload in ("loaded", "empty")
        for side in ("left", "right")
    )
    passed = (
        len(rows) >= VALIDATION_SIZE
        and minimum_met
        and payload_side_present
        and all(required_features.get(name, 0) > 0 for name in (
            "L4_loaded", "L4_empty", "R4_loaded", "R4_empty",
            "combined_L4_R4", "exact_L2_L4_R4",
            "deliberate_right_slot3_offset",
        ))
        and all(presence_class[name] > 0 for name in ("sparse", "medium", "dense"))
        and len([value for value in relations.values() if value > 0]) >= 3
        and target_one_hot_failures == 0
        and masked_absent_failures == 0
        and present_mask_failures == 0
        and categorical_semantic_failures == 0
    )
    return {
        "passed": passed,
        "scenario_count": len(rows),
        "minimum_loaded_and_empty_per_identity_required": MIN_PAYLOAD_PER_IDENTITY,
        "minimum_coverage_met": minimum_met,
        "payload_by_side": {
            f"{payload}_{side}": payload_side[(payload, side)]
            for payload in ("loaded", "empty")
            for side in ("left", "right")
        },
        "payload_by_identity": per_identity,
        "identity_payload_presence": {
            "|".join(key): value
            for key, value in sorted(identity_payload_presence.items())
        },
        "presence_class": dict(sorted(presence_class.items())),
        "relation_family": dict(sorted(relations.items())),
        "target_offset_bucket": dict(sorted(offsets.items())),
        "required_features": required_features,
        "target_one_hot_failures": target_one_hot_failures,
        "masked_absent_failures": masked_absent_failures,
        "present_mask_failures": present_mask_failures,
        "categorical_semantic_failures": categorical_semantic_failures,
        "payload_determined_by_side": not payload_side_present,
    }


def build_selection(config: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    old_rows = read_jsonl(expand_path(config["data"]["old_replay"]["rows"]))
    new_rows = read_jsonl(expand_path(config["data"]["v3r1_train"]["rows"]))
    validation_rows = read_jsonl(
        expand_path(config["data"]["v3r1_validation"]["rows"])
    )
    validation_labels = read_jsonl(
        expand_path(config["data"]["v3r1_validation"]["labels"])
    )
    vectorizer = read_json(expand_path(config["artifacts"]["vectorizer"]["path"]))
    validate_vectorizer(vectorizer)
    old_train = deterministic_ids(old_rows, TRAIN_PER_SOURCE, OLD_TRAIN_SEED)
    new_train = deterministic_ids(new_rows, TRAIN_PER_SOURCE, NEW_TRAIN_SEED)
    validation = deterministic_ids(
        validation_rows, VALIDATION_SIZE, VALIDATION_SEED
    )
    old_candidates = deterministic_ids(
        old_rows, len(old_rows), OLD_EVAL_SEED
    )
    old_train_set = set(old_train)
    old_eval = [value for value in old_candidates if value not in old_train_set][
        :OLD_REPLAY_EVAL_SIZE
    ]
    val_row_by_id = {row_id(row): row for row in validation_rows}
    val_label_by_id = {row_id(row): row for row in validation_labels}
    selected_rows = [val_row_by_id[value] for value in validation]
    selected_labels = [val_label_by_id[value] for value in validation]
    audit = payload_selection_audit(selected_rows, selected_labels, vectorizer)
    if not audit["passed"]:
        raise ExperimentAError("deterministic Smoke V2 validation selection failed")
    selection = {
        "schema_version": "room315.experiment_a.smoke_v2_selection.v1",
        "seed": SEED,
        "train": {
            "old_replay": old_train,
            "v3r1_hard_case": new_train,
        },
        "validation": validation,
        "old_replay_regression": old_eval,
        "fingerprints": {
            "old_train": rows_fingerprint(old_train),
            "new_train": rows_fingerprint(new_train),
            "validation": rows_fingerprint(validation),
            "old_replay_regression": rows_fingerprint(old_eval),
        },
        "counts": {
            "old_train": len(old_train),
            "new_train": len(new_train),
            "total_train": len(old_train) + len(new_train),
            "validation": len(validation),
            "old_replay_regression": len(old_eval),
        },
        "legacy_test_used": False,
        "canary_used": False,
    }
    audit["validation_fingerprint_sha256"] = selection["fingerprints"]["validation"]
    audit["selection_counts"] = selection["counts"]
    return selection, audit


def select_records(records: list[dict[str, Any]], ids: list[str]) -> list[dict[str, Any]]:
    by_id = {row_id(record["row"]): record for record in records}
    missing = [value for value in ids if value not in by_id]
    if missing:
        raise ExperimentAError(f"selection contains missing rows: {missing[:3]}")
    return [by_id[value] for value in ids]


def metric_snapshot(metrics: dict[str, Any], fingerprint: str) -> dict[str, Any]:
    return {
        "validation_subset_fingerprint_sha256": fingerprint,
        "total_validation_loss": metrics["losses"]["total"],
        "per_head_losses": {
            key: metrics["losses"][key]
            for key in ("segment_location", "loaded_state", "bbox", "s_m", "s_ratio")
        },
        "loaded_state_accuracy": metrics["loaded_state_accuracy"],
        "per_identity_loaded_state": metrics["per_identity_loaded_state"],
        "side_accuracy": metrics["side_accuracy"],
        "block_top1_accuracy": metrics["block_top1_accuracy"],
        "block_top2_accuracy": metrics["block_top2_accuracy"],
        "bbox_mae": metrics["bbox_mae"],
        "s_m": {
            "mean": metrics["s_m_mae"],
            "median": metrics["s_m_median"],
            "p95": metrics["s_m_p95"],
        },
        "s_ratio": {
            "mean": metrics["s_ratio_mae"],
            "median": metrics["s_ratio_median"],
            "p95": metrics["s_ratio_p95"],
        },
    }


def side_diagnostics(
    retained: list[tuple[Any, ...]], names: list[str]
) -> dict[str, Any]:
    per_identity = {}
    total = 0
    correct = 0
    malformed_targets = 0
    for slot, identity in enumerate(IDENTITIES):
        indexes = [
            names.index(f"shuttles.{slot}.location.side==left"),
            names.index(f"shuttles.{slot}.location.side==right"),
        ]
        counts = Counter()
        for prediction, target, mask, _ in retained:
            if float(mask[indexes].sum()) <= 0:
                continue
            target_values = [float(target[index]) for index in indexes]
            if sum(value == 1.0 for value in target_values) != 1:
                malformed_targets += 1
                continue
            expected = int(target_values[1] > target_values[0])
            predicted = int(float(prediction[indexes[1]]) > float(prediction[indexes[0]]))
            counts[f"target_{'right' if expected else 'left'}"] += 1
            counts[f"predicted_{'right' if predicted else 'left'}"] += 1
            counts["correct"] += expected == predicted
            counts["present"] += 1
        per_identity[identity] = {
            **dict(counts),
            "accuracy": counts["correct"] / max(1, counts["present"]),
        }
        total += counts["present"]
        correct += counts["correct"]
    return {
        "independent_side_accuracy": correct / max(1, total),
        "present_slots": total,
        "malformed_present_targets": malformed_targets,
        "per_identity": per_identity,
    }


def evaluate_point(
    torch: Any,
    model: Any,
    records: list[dict[str, Any]],
    vectorizer: dict[str, Any],
    stats: dict[str, Any],
    runtime: dict[str, Any],
    fingerprint: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    metrics = evaluate(
        torch,
        model,
        loader(
            torch,
            records,
            vectorizer,
            stats,
            batch_size=int(runtime["batch_size"]),
            num_workers=int(runtime["num_workers"]),
            pin_memory=bool(runtime["pin_memory"]),
            persistent_workers=bool(runtime["persistent_workers"]),
        ),
        device="cuda",
        stats=stats,
        names=vectorizer["names"],
        automatic_mixed_precision=bool(runtime["automatic_mixed_precision"]),
        retain=True,
    )
    retained = metrics.pop("_records")
    snapshot = metric_snapshot(metrics, fingerprint)
    return snapshot, side_diagnostics(retained, vectorizer["names"])


def regression_report(
    baseline: dict[str, Any],
    point: dict[str, Any],
    old_baseline: dict[str, Any],
    old_point: dict[str, Any],
) -> dict[str, Any]:
    flags = {
        "side_accuracy": point["side_accuracy"] < baseline["side_accuracy"] - 0.05,
        "block_accuracy": point["block_top1_accuracy"] < baseline["block_top1_accuracy"] - 0.05,
        "s_m_mean": point["s_m"]["mean"] > baseline["s_m"]["mean"] * 1.10,
        "s_ratio_mean": point["s_ratio"]["mean"] > baseline["s_ratio"]["mean"] * 1.10,
        "old_replay_total_loss": old_point["total_validation_loss"] > old_baseline["total_validation_loss"] * 1.10,
        "old_replay_block_accuracy": old_point["block_top1_accuracy"] < old_baseline["block_top1_accuracy"] - 0.05,
        "old_replay_side_accuracy": old_point["side_accuracy"] < old_baseline["side_accuracy"] - 0.05,
    }
    return {
        "material_regression": any(flags.values()),
        "thresholds": {
            "accuracy_absolute_drop": 0.05,
            "error_or_loss_relative_increase": 0.10,
        },
        "flags": flags,
    }


def side_accuracy_diagnosis(
    *,
    balanced_baseline: dict[str, Any],
    old_baseline: dict[str, Any],
    balanced_epoch_1: dict[str, Any],
    balanced_epoch_2: dict[str, Any],
    baseline_side: dict[str, Any],
    load_report: dict[str, Any],
) -> dict[str, Any]:
    baseline_value = float(balanced_baseline["side_accuracy"])
    old_value = float(old_baseline["side_accuracy"])
    epoch_1_value = float(balanced_epoch_1["side_accuracy"])
    epoch_2_value = float(balanced_epoch_2["side_accuracy"])
    labels_valid = baseline_side["malformed_present_targets"] == 0
    metric_matches = abs(
        baseline_side["independent_side_accuracy"] - baseline_value
    ) < 1e-12
    checkpoint_valid = bool(
        load_report["strict"]
        and not load_report["missing_keys"]
        and not load_report["unexpected_keys"]
        and load_report["all_prediction_head_tensors_equal_checkpoint"]
        and not load_report["prediction_head_reinitialized"]
    )
    selection_bias_supported = False
    v3r1_domain_shift_supported = old_value >= 0.90 and baseline_value < 0.90
    continuation_degradation_supported = min(epoch_1_value, epoch_2_value) < (
        baseline_value - 0.05
    )
    primary = (
        "Smoke V1 and preserved Smoke V2 attempt 1 used categorical_values JSON-object "
        "iteration order (loaded, block, side) to build targets while metric decoding "
        "used vectorizer.names order (side, block, loaded). Side metrics therefore read "
        "payload bits as side targets. The current attempt encodes by each explicit "
        "vectorizer name and verifies raw-label equality before training."
    )
    return {
        "smoke_v1_reported_side_accuracy": 0.4642857142857143,
        "measured_side_accuracy": {
            "approved_checkpoint_balanced_v3r1": baseline_value,
            "approved_checkpoint_old_replay": old_value,
            "continuation_epoch_1_balanced_v3r1": epoch_1_value,
            "continuation_epoch_2_balanced_v3r1": epoch_2_value,
        },
        "candidate_causes": {
            "v1_validation_payload_side_confound_confirmed": True,
            "validation_selection_bias_as_sole_cause_supported": selection_bias_supported,
            "label_semantics_defect_supported": not labels_valid,
            "current_vectorizer_or_metric_decoding_defect_supported": not metric_matches,
            "historical_v1_vectorizer_order_defect_confirmed": True,
            "fixed_slot_masking_defect_supported": baseline_side["present_slots"] <= 0,
            "checkpoint_loading_defect_supported": not checkpoint_valid,
            "v3r1_visual_domain_shift_supported": v3r1_domain_shift_supported,
            "continuation_degradation_supported": continuation_degradation_supported,
        },
        "primary_interpretation": primary,
        "historical_vectorizer_order_evidence": {
            "incorrect_encoded_group_order": "loaded_state, location.block, location.side",
            "declared_vector_name_group_order": "location.side, location.block, loaded_state",
            "impact": "categorical targets were assigned to the wrong named output indexes",
            "corrected_contract": "encode categorical values in vectorizer.names order",
        },
    }


def run(config_path: Path, package_root: Path, output: Path) -> dict[str, Any]:
    if output.exists():
        raise ExperimentAError(f"refusing to overwrite Smoke V2 output: {output}")
    v1_before = verify_smoke_v1_immutable()
    invalid_v2_before = verify_invalid_smoke_v2_attempt1_immutable()
    config = read_json(config_path)
    selection = read_json(package_root / "config" / "smoke_v2_selection.json")
    audit = read_json(package_root / "smoke_v2_selection_audit.json")
    if not audit.get("passed"):
        raise ExperimentAError("Smoke V2 static selection audit did not pass")
    runtime = effective_training_config(config, LOCAL_DEFAULT_PROFILE)
    if int(runtime["batch_size"]) != 32 or not runtime["automatic_mixed_precision"]:
        raise ExperimentAError("Smoke V2 local execution profile changed")
    if int(config["training"]["maximum_continuation_epochs"]) != 2:
        raise ExperimentAError("Smoke V2 must train exactly two epochs")

    torch, torchvision = require_torch()
    set_deterministic(torch, int(config["seed"]))
    vectorizer = read_json(expand_path(config["artifacts"]["vectorizer"]["path"]))
    stats = read_json(expand_path(config["artifacts"]["target_stats"]["path"]))
    validate_vectorizer(vectorizer)
    validate_target_stats(stats)
    old_all = load_source(config["data"]["old_replay"], "old_replay")
    new_all = load_source(config["data"]["v3r1_train"], "v3r1_hard_case")
    validation_all = load_source(
        config["data"]["v3r1_validation"], "v3r1_validation"
    )
    old_train = select_records(old_all, selection["train"]["old_replay"])
    new_train = select_records(new_all, selection["train"]["v3r1_hard_case"])
    validation = select_records(validation_all, selection["validation"])
    old_regression = select_records(
        old_all, selection["old_replay_regression"]
    )
    validation_fingerprint = rows_fingerprint(
        [row_id(record["row"]) for record in validation]
    )
    if validation_fingerprint != selection["fingerprints"]["validation"]:
        raise ExperimentAError("validation selection fingerprint mismatch")

    output.mkdir(parents=True)
    model = build_visual_state_model(
        torch,
        torchvision,
        output_dim=VECTOR_DIMENSION,
        adaptation_mode="partial_finetune",
    ).to("cuda")
    checkpoint = strict_load_approved_checkpoint(
        torch_module=torch,
        model=model,
        checkpoint_path=expand_path(config["artifacts"]["approved_checkpoint"]["path"]),
    )
    load_report = {
        **checkpoint["_strict_load_verification"],
        "approved_checkpoint_sha256": APPROVED_CHECKPOINT_SHA256,
        "approved_checkpoint_epoch": int(checkpoint["epoch"]),
        "model_output_dimension": VECTOR_DIMENSION,
        "prediction_head_reinitialized": False,
    }
    (output / "checkpoint_loading_verification.json").write_text(
        json.dumps(load_report, indent=2, sort_keys=True) + "\n"
    )

    execution_order = []
    baseline, baseline_side = evaluate_point(
        torch, model, validation, vectorizer, stats, runtime, validation_fingerprint
    )
    old_baseline, old_baseline_side = evaluate_point(
        torch,
        model,
        old_regression,
        vectorizer,
        stats,
        runtime,
        selection["fingerprints"]["old_replay_regression"],
    )
    execution_order.append("approved_checkpoint_baseline_evaluated_before_optimizer_creation")
    (output / "baseline_validation_metrics.json").write_text(
        json.dumps(baseline, indent=2, sort_keys=True) + "\n"
    )

    optimizer = torch.optim.AdamW(
        (parameter for parameter in model.parameters() if parameter.requires_grad),
        lr=float(config["optimizer"]["learning_rate"]),
        weight_decay=float(config["optimizer"]["weight_decay"]),
    )
    scaler = torch.amp.GradScaler("cuda", enabled=True)
    old_map = {row_id(record["row"]): record for record in old_train}
    new_map = {row_id(record["row"]): record for record in new_train}
    epoch_results = []
    started = time.time()
    for epoch in (1, 2):
        plan = source_balanced_epoch(
            list(old_map), list(new_map), seed=SEED, epoch=epoch,
            per_source=TRAIN_PER_SOURCE,
        )
        plan_report = sampling_report(plan)
        if plan_report["source_counts"] != {
            "old_replay": TRAIN_PER_SOURCE,
            "v3r1_hard_case": TRAIN_PER_SOURCE,
        }:
            raise ExperimentAError("Smoke V2 source balance changed")
        records = [
            (old_map if item["source"] == "old_replay" else new_map)[item["sample_id"]]
            for item in plan
        ]
        train_loader = loader(
            torch, records, vectorizer, stats,
            batch_size=32, num_workers=4, pin_memory=True,
            persistent_workers=True,
        )
        model.train()
        losses_accumulator = defaultdict(float)
        batch_count = 0
        epoch_started = time.perf_counter()
        for batch in train_loader:
            image = batch["image"].to("cuda")
            target = batch["target"].to("cuda")
            mask = batch["target_mask"].to("cuda")
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast(device_type="cuda", dtype=torch.float16):
                prediction = model(image)
                losses = visual_loss(
                    torch, prediction, target, mask, vectorizer["names"]
                )
            if not all(
                math.isfinite(float(value.detach().item()))
                for value in losses.values()
            ):
                raise ExperimentAError("Smoke V2 training loss is non-finite")
            scaler.scale(losses["total"]).backward()
            scaler.step(optimizer)
            scaler.update()
            for name, value in losses.items():
                losses_accumulator[name] += float(value.detach().item())
            batch_count += 1
        validation_point, side_point = evaluate_point(
            torch, model, validation, vectorizer, stats, runtime,
            validation_fingerprint,
        )
        old_point, old_side_point = evaluate_point(
            torch, model, old_regression, vectorizer, stats, runtime,
            selection["fingerprints"]["old_replay_regression"],
        )
        execution_order.append(f"continuation_epoch_{epoch}_then_same_validation_subset")
        checkpoint_path = output / f"epoch_{epoch:02d}.pt"
        torch.save({
            "model_state_dict": model.state_dict(),
            "optimizer_state_dict": optimizer.state_dict(),
            "epoch": 14 + epoch,
            "continuation_epoch": epoch,
            "approved_source_checkpoint_sha256": APPROVED_CHECKPOINT_SHA256,
            "validation_subset_fingerprint_sha256": validation_fingerprint,
        }, checkpoint_path)
        result = {
            "continuation_epoch": epoch,
            "absolute_epoch": 14 + epoch,
            "validation": validation_point,
            "old_replay_regression": old_point,
            "side_diagnostics": side_point,
            "old_replay_side_diagnostics": old_side_point,
            "train_losses": {
                name: value / max(1, batch_count)
                for name, value in losses_accumulator.items()
            },
            "source_balance": plan_report,
            "runtime_s": time.perf_counter() - epoch_started,
            "checkpoint": str(checkpoint_path),
            "checkpoint_sha256": sha256_file(checkpoint_path),
        }
        epoch_results.append(result)
        (output / f"epoch_{epoch:02d}_validation_metrics.json").write_text(
            json.dumps(validation_point, indent=2, sort_keys=True) + "\n"
        )

    fingerprints = [
        baseline["validation_subset_fingerprint_sha256"],
        *[
            result["validation"]["validation_subset_fingerprint_sha256"]
            for result in epoch_results
        ],
    ]
    if len(set(fingerprints)) != 1:
        raise ExperimentAError("baseline and epoch validation subsets differ")
    regressions = {
        f"epoch_{result['continuation_epoch']}": regression_report(
            baseline,
            result["validation"],
            old_baseline,
            result["old_replay_regression"],
        )
        for result in epoch_results
    }
    epoch2 = epoch_results[-1]["validation"]
    full_approved = (
        not regressions["epoch_2"]["material_regression"]
        and epoch2["side_accuracy"] >= 0.90
        and audit["passed"]
    )
    comparison = {
        "schema_version": "room315.experiment_a.smoke_v2_comparison.v1",
        "validation_subset_fingerprint_sha256": validation_fingerprint,
        "identical_validation_subset_all_points": True,
        "approved_checkpoint_baseline": baseline,
        "continuation_epoch_1": epoch_results[0]["validation"],
        "continuation_epoch_2": epoch_results[1]["validation"],
        "old_replay_performance": {
            "baseline": old_baseline,
            "epoch_1": epoch_results[0]["old_replay_regression"],
            "epoch_2": epoch_results[1]["old_replay_regression"],
        },
        "regression_audit": regressions,
        "full_local_run_scientifically_approved": full_approved,
        "full_local_run_status": "approved" if full_approved else "blocked",
    }
    (output / "comparison_report.json").write_text(
        json.dumps(comparison, indent=2, sort_keys=True) + "\n"
    )
    side_investigation = {
        **side_accuracy_diagnosis(
            balanced_baseline=baseline,
            old_baseline=old_baseline,
            balanced_epoch_1=epoch_results[0]["validation"],
            balanced_epoch_2=epoch_results[1]["validation"],
            baseline_side=baseline_side,
            load_report=load_report,
        ),
        "baseline_balanced_v2_validation": baseline_side,
        "baseline_old_replay_regression": old_baseline_side,
        "epoch_1_balanced_v2_validation": epoch_results[0]["side_diagnostics"],
        "epoch_2_balanced_v2_validation": epoch_results[1]["side_diagnostics"],
        "label_semantics": {
            "fixed_identity_side": {
                identity: "left" if identity.startswith("L") else "right"
                for identity in IDENTITIES
            },
            "malformed_present_side_targets": baseline_side["malformed_present_targets"],
        },
        "vectorizer_decoding": {
            "side_metric": "independent two-class argmax per present fixed slot",
            "target_means": "left slots [1,0], right slots [0,1]",
            "target_stds": "[1,1] for every side pair",
            "independent_metric_matches_reported": abs(
                baseline_side["independent_side_accuracy"] - baseline["side_accuracy"]
            ) < 1e-12,
        },
        "fixed_slot_masking": {
            "present_slots_evaluated": baseline_side["present_slots"],
            "absent_slots_excluded": True,
        },
        "checkpoint_loading": load_report,
    }
    (output / "side_accuracy_investigation.json").write_text(
        json.dumps(side_investigation, indent=2, sort_keys=True) + "\n"
    )
    (output / "execution_order.json").write_text(
        json.dumps({"events": execution_order}, indent=2, sort_keys=True) + "\n"
    )
    v1_after = verify_smoke_v1_immutable()
    invalid_v2_after = verify_invalid_smoke_v2_attempt1_immutable()
    final = {
        "status": "completed",
        "epochs_completed": 2,
        "runtime_s": time.time() - started,
        "selection_audit_passed": audit["passed"],
        "checkpoint_loading_verification": load_report,
        "validation_subset_fingerprint_sha256": validation_fingerprint,
        "smoke_v1_tree_sha256_before": v1_before["tree_sha256"],
        "smoke_v1_tree_sha256_after": v1_after["tree_sha256"],
        "smoke_v1_unchanged": v1_before["tree_sha256"] == v1_after["tree_sha256"],
        "invalid_smoke_v2_attempt1_tree_sha256_before": invalid_v2_before["tree_sha256"],
        "invalid_smoke_v2_attempt1_tree_sha256_after": invalid_v2_after["tree_sha256"],
        "invalid_smoke_v2_attempt1_unchanged": (
            invalid_v2_before["tree_sha256"] == invalid_v2_after["tree_sha256"]
        ),
        "legacy_test_accessed": False,
        "canary_accessed": False,
        "full_training_started": False,
        "full_training_authorized": False,
        "comparison_report": str(output / "comparison_report.json"),
    }
    (output / "final_report.json").write_text(
        json.dumps(final, indent=2, sort_keys=True) + "\n"
    )
    return final


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--package-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.config, args.package_root, args.output), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
