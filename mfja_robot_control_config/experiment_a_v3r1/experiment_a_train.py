#!/usr/bin/env python3
"""Continuation trainer and post-training Canary evaluator for Experiment A."""

from __future__ import annotations

import argparse
import json
import os
import random
import statistics
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from experiment_a_core import (  # noqa: E402
    APPROVED_CHECKPOINT_SHA256,
    CHECKPOINT_SELECTION,
    CONFIGURED_DEFAULT_PROFILE,
    GLOBAL_BLOCKS,
    IDENTITIES,
    LOSS_HEADS,
    PYTHON_HASH_SEED,
    SEED,
    VECTOR_DIMENSION,
    ExperimentAError,
    categorical_output_index,
    expand_path,
    effective_training_config,
    loss_head_indexes,
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
from room_315_visual_model import build_visual_state_model  # noqa: E402


NORMALIZATION_MEAN = (0.485, 0.456, 0.406)
NORMALIZATION_STD = (0.229, 0.224, 0.225)


def require_torch() -> tuple[Any, Any]:
    try:
        import torch
        import torchvision
    except Exception as exc:
        raise SystemExit(f"Torch and TorchVision are required for this execution profile: {exc}") from exc
    return torch, torchvision


def set_deterministic(torch: Any, seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed % (2**32))
    torch.manual_seed(seed % (2**63))
    torch.cuda.manual_seed_all(seed % (2**63))
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    torch.use_deterministic_algorithms(True, warn_only=True)


def visual_loss(torch: Any, prediction: Any, target: Any, mask: Any, names: list[str]) -> dict[str, Any]:
    indexes = loss_head_indexes(names)
    elements = torch.nn.functional.smooth_l1_loss(prediction, target, reduction="none")
    result: dict[str, Any] = {}
    terms = []
    for head in LOSS_HEADS:
        selected_mask = mask[:, indexes[head]]
        counts = selected_mask.sum(dim=1)
        valid = counts > 0
        if not bool(valid.any()):
            raise ExperimentAError(f"batch contains no valid sample for loss head {head}")
        per_sample = (elements[:, indexes[head]] * selected_mask).sum(dim=1) / counts.clamp_min(1.0)
        value = per_sample[valid].mean()
        result[head] = value
        terms.append(value)
    result["total"] = sum(terms) / float(len(terms))
    return result


class SelectedDataset:
    def __init__(self, records: list[dict[str, Any]], vectorizer: dict[str, Any], stats: dict[str, Any], torch: Any):
        self.records = records
        self.vectorizer = vectorizer
        self.mean = np.asarray(stats["mean"], dtype=np.float32)
        self.std = np.asarray(stats["std"], dtype=np.float32)
        self.torch = torch

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, index: int) -> dict[str, Any]:
        record = self.records[index]
        row = record["row"]
        refs = row["model_input"]["overhead_images"]
        images = []
        for camera in ("left_rail_rgb", "right_rail_rgb"):
            path = Path(record["dataset_root"], refs[camera])
            with Image.open(path) as raw:
                rgb = raw.convert("RGB").resize((224, 224), Image.Resampling.BILINEAR)
                array = np.asarray(rgb, dtype=np.float32) / 255.0
            array = np.transpose(array, (2, 0, 1))
            array = (array - np.asarray(NORMALIZATION_MEAN, dtype=np.float32)[:, None, None]) / np.asarray(NORMALIZATION_STD, dtype=np.float32)[:, None, None]
            images.append(array)
        raw_target, mask = vectorize_label(record["label"], self.vectorizer)
        raw_array = np.asarray(raw_target, dtype=np.float32)
        return {
            "image": self.torch.as_tensor(np.concatenate(images), dtype=self.torch.float32),
            "target": self.torch.as_tensor((raw_array - self.mean) / self.std, dtype=self.torch.float32),
            "raw_target": self.torch.as_tensor(raw_array, dtype=self.torch.float32),
            "target_mask": self.torch.as_tensor(mask, dtype=self.torch.float32),
            "source": record["source"],
            "sample_id": row_id(row),
            "trace_json": json.dumps(
                row.get("traceability_metadata") or {}, sort_keys=True
            ),
        }


def paired_records(rows: list[dict[str, Any]], labels: list[dict[str, Any]], *, source: str, dataset_root: Path) -> list[dict[str, Any]]:
    by_id = {row_id(label): label for label in labels}
    if len(by_id) != len(labels):
        raise ExperimentAError(f"duplicate label IDs in {source}")
    result = []
    for row in rows:
        identifier = row_id(row)
        if identifier not in by_id:
            raise ExperimentAError(f"missing label for {identifier}")
        result.append({"row": row, "label": by_id[identifier], "source": source, "dataset_root": dataset_root})
    return result


def load_source(spec: dict[str, Any], source: str) -> list[dict[str, Any]]:
    rows = read_jsonl(expand_path(spec["rows"]))
    labels = read_jsonl(expand_path(spec["labels"]))
    return paired_records(rows, labels, source=source, dataset_root=expand_path(spec["dataset_root"]))


def pick_ids(records: list[dict[str, Any]], ids: list[str]) -> list[dict[str, Any]]:
    mapping = {row_id(item["row"]): item for item in records}
    missing = [identifier for identifier in ids if identifier not in mapping]
    if missing:
        raise ExperimentAError(f"selection references missing samples: {missing[:3]}")
    return [mapping[identifier] for identifier in ids]


def metric_groups(names: list[str]) -> dict[str, Any]:
    vectorizer = {"names": names}
    groups = {"bbox": [], "s_m": [], "s_ratio": [], "side": {}, "block": {}, "loaded": {}}
    for slot, identity in enumerate(IDENTITIES):
        groups["side"][slot] = [
            categorical_output_index(vectorizer, identity, "side", value)
            for value in ("left", "right")
        ]
        groups["block"][slot] = [
            categorical_output_index(vectorizer, identity, "block", value)
            for value in GLOBAL_BLOCKS
        ]
        groups["loaded"][slot] = [
            categorical_output_index(vectorizer, identity, "loaded_state", value)
            for value in ("empty", "loaded")
        ]
        groups["bbox"].extend(i for i, name in enumerate(names) if name.startswith(f"shuttles.{slot}.bbox."))
        groups["s_m"].append(names.index(f"shuttles.{slot}.rail_position.s_m"))
        groups["s_ratio"].append(names.index(f"shuttles.{slot}.rail_position.s_ratio"))
    return groups


def summarize_predictions(predictions: list[np.ndarray], targets: list[np.ndarray], masks: list[np.ndarray], names: list[str]) -> dict[str, Any]:
    groups = metric_groups(names)
    counts = defaultdict(int)
    errors: dict[str, list[float]] = {"bbox": [], "s_m": [], "s_ratio": []}
    per_identity = {identity: defaultdict(int) for identity in IDENTITIES}
    for prediction, target, mask in zip(predictions, targets, masks):
        for slot, identity in enumerate(IDENTITIES):
            side = groups["side"][slot]; block = groups["block"][slot]; loaded = groups["loaded"][slot]
            if mask[side].sum() <= 0:
                continue
            counts["present"] += 1
            side_correct = int(np.argmax(prediction[side]) == np.argmax(target[side]))
            block_order = np.argsort(prediction[block])[-2:]
            true_block = int(np.argmax(target[block]))
            block_correct = int(int(block_order[-1]) == true_block)
            counts["side_correct"] += side_correct
            counts["block_correct"] += block_correct
            counts["full_location_correct"] += side_correct and block_correct
            counts["top2_block_correct"] += true_block in set(int(v) for v in block_order)
            true_loaded = int(np.argmax(target[loaded]))
            pred_loaded = int(np.argmax(prediction[loaded]))
            counts["loaded_correct"] += pred_loaded == true_loaded
            per_identity[identity]["present"] += 1
            if true_loaded == 1:
                per_identity[identity]["loaded"] += 1
                per_identity[identity]["loaded_true_positive"] += pred_loaded == 1
            else:
                per_identity[identity]["empty"] += 1
                per_identity[identity]["empty_true_negative"] += pred_loaded == 0
            bbox_indexes = [i for i in groups["bbox"] if names[i].startswith(f"shuttles.{slot}.")]
            errors["bbox"].extend(np.abs(prediction[bbox_indexes] - target[bbox_indexes]).tolist())
            for key in ("s_m", "s_ratio"):
                idx = groups[key][slot]
                errors[key].append(abs(float(prediction[idx] - target[idx])))
    present = max(1, counts["present"])
    result: dict[str, Any] = {
        "present_slot_count": counts["present"],
        "loaded_state_accuracy": counts["loaded_correct"] / present,
        "side_accuracy": counts["side_correct"] / present,
        "block_top1_accuracy": counts["block_correct"] / present,
        "block_top2_accuracy": counts["top2_block_correct"] / present,
        "full_location_accuracy": counts["full_location_correct"] / present,
    }
    for key, values in errors.items():
        result[f"{key}_mae"] = statistics.fmean(values) if values else None
        if key != "bbox":
            result[f"{key}_median"] = statistics.median(values) if values else None
            result[f"{key}_p95"] = float(np.percentile(values, 95)) if values else None
    result["per_identity_loaded_state"] = {
        identity: {
            "loaded_recall": values["loaded_true_positive"] / max(1, values["loaded"]),
            "empty_specificity": values["empty_true_negative"] / max(1, values["empty"]),
            "loaded_count": values["loaded"],
            "empty_count": values["empty"],
        } for identity, values in per_identity.items()
    }
    return result


def evaluate(
    torch: Any,
    model: Any,
    loader: Any,
    *,
    device: str,
    stats: dict[str, Any],
    names: list[str],
    automatic_mixed_precision: bool = False,
    retain: bool = False,
) -> dict[str, Any]:
    model.eval()
    mean = torch.as_tensor(stats["mean"], dtype=torch.float32, device=device)
    std = torch.as_tensor(stats["std"], dtype=torch.float32, device=device)
    loss_sums = defaultdict(float); batches = 0
    predictions: list[np.ndarray] = []; targets: list[np.ndarray] = []; masks: list[np.ndarray] = []; traces: list[dict[str, Any]] = []
    start = time.perf_counter(); sample_count = 0
    with torch.no_grad():
        for batch in loader:
            image = batch["image"].to(device); target = batch["target"].to(device); mask = batch["target_mask"].to(device)
            with torch.autocast(
                device_type="cuda",
                dtype=torch.float16,
                enabled=automatic_mixed_precision,
            ):
                pred_norm = model(image)
                losses = visual_loss(torch, pred_norm, target, mask, names)
            for key, value in losses.items(): loss_sums[key] += float(value.item())
            pred = pred_norm * std + mean
            predictions.extend(pred.cpu().numpy()); targets.extend(batch["raw_target"].numpy()); masks.extend(mask.cpu().numpy())
            traces.extend(json.loads(value) for value in batch["trace_json"])
            batches += 1
            sample_count += image.shape[0]
    elapsed = time.perf_counter() - start
    result = {
        "losses": {key: value / max(1, batches) for key, value in loss_sums.items()},
        **summarize_predictions(predictions, targets, masks, names),
        "inference_latency_ms_per_sample": elapsed * 1000.0 / max(1, sample_count),
        "automatic_mixed_precision": automatic_mixed_precision,
    }
    if retain: result["_records"] = list(zip(predictions, targets, masks, traces))
    return result


def loader(
    torch: Any,
    records: list[dict[str, Any]],
    vectorizer: dict[str, Any],
    stats: dict[str, Any],
    *,
    batch_size: int,
    num_workers: int = 4,
    pin_memory: bool = True,
    persistent_workers: bool = False,
    shuffle: bool = False,
) -> Any:
    workers = max(0, int(num_workers))
    return torch.utils.data.DataLoader(
        SelectedDataset(records, vectorizer, stats, torch),
        batch_size=batch_size,
        shuffle=shuffle,
        num_workers=workers,
        pin_memory=bool(pin_memory),
        persistent_workers=bool(persistent_workers) and workers > 0,
    )


def _driver_version() -> str | None:
    try:
        return subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=driver_version",
                "--format=csv,noheader",
            ],
            check=True,
            text=True,
            capture_output=True,
        ).stdout.strip().splitlines()[0]
    except Exception:
        return None


def evaluate_records(
    torch: Any,
    model: Any,
    records: list[dict[str, Any]],
    vectorizer: dict[str, Any],
    stats: dict[str, Any],
    *,
    batch_size: int,
    num_workers: int,
    pin_memory: bool,
    persistent_workers: bool,
    automatic_mixed_precision: bool,
    retain: bool = False,
) -> dict[str, Any]:
    """One evaluation path shared by baseline, epochs, best, and Canary."""
    return evaluate(
        torch,
        model,
        loader(
            torch,
            records,
            vectorizer,
            stats,
            batch_size=batch_size,
            num_workers=num_workers,
            pin_memory=pin_memory,
            persistent_workers=persistent_workers,
        ),
        device="cuda",
        stats=stats,
        names=vectorizer["names"],
        automatic_mixed_precision=automatic_mixed_precision,
        retain=retain,
    )


def payload_warning_metrics(
    metrics: dict[str, Any],
    *,
    point: str,
    source_specific_train_losses: dict[str, Any] | None = None,
) -> dict[str, Any]:
    per_identity = metrics["per_identity_loaded_state"]
    return {
        "point": point,
        "overall_loaded_accuracy": metrics["loaded_state_accuracy"],
        "per_identity": {
            identity: {
                "loaded_recall": per_identity[identity]["loaded_recall"],
                "empty_specificity": per_identity[identity]["empty_specificity"],
                "loaded_count": per_identity[identity]["loaded_count"],
                "empty_count": per_identity[identity]["empty_count"],
            }
            for identity in IDENTITIES
        },
        "L4_loaded_recall": per_identity["L4"]["loaded_recall"],
        "R4_loaded_recall": per_identity["R4"]["loaded_recall"],
        "R3_loaded_recall": per_identity["R3"]["loaded_recall"],
        "source_specific_loaded_state_loss": {
            source: values["loaded_state"]
            for source, values in (source_specific_train_losses or {}).items()
            if "loaded_state" in values
        },
    }


def train(
    config_path: Path,
    output: Path,
    *,
    execution_profile: str = CONFIGURED_DEFAULT_PROFILE,
) -> None:
    if output.exists(): raise ExperimentAError(f"refusing to overwrite immutable output: {output}")
    config = read_json(config_path); torch, torchvision = require_torch(); set_deterministic(torch, int(config["seed"]))
    runtime = effective_training_config(config, execution_profile)
    batch_size = int(runtime["batch_size"])
    accumulation_steps = int(runtime.get("gradient_accumulation_steps", 1))
    amp_enabled = bool(runtime.get("automatic_mixed_precision", False))
    num_workers = int(runtime.get("num_workers", 4))
    pin_memory = bool(runtime.get("pin_memory", True))
    persistent_workers = bool(runtime.get("persistent_workers", False))
    if batch_size <= 0 or accumulation_steps <= 0:
        raise ExperimentAError("batch size and gradient accumulation must be positive")
    vectorizer = read_json(expand_path(config["artifacts"]["vectorizer"]["path"])); validate_vectorizer(vectorizer)
    stats = read_json(expand_path(config["artifacts"]["target_stats"]["path"])); validate_target_stats(stats)
    old = load_source(config["data"]["old_replay"], "old_replay"); new = load_source(config["data"]["v3r1_train"], "v3r1_hard_case"); val = load_source(config["data"]["v3r1_validation"], "v3r1_validation")
    selection = read_json(expand_path(config["smoke_selection"])) if config["stage"] == "smoke" else None
    if selection:
        old = pick_ids(old, selection["train"]["old_replay"]); new = pick_ids(new, selection["train"]["v3r1_hard_case"]); val = pick_ids(val, selection["validation"])
    device = "cuda"; model = build_visual_state_model(torch, torchvision, output_dim=VECTOR_DIMENSION, adaptation_mode="partial_finetune").to(device)
    checkpoint_path = expand_path(config["artifacts"]["approved_checkpoint"]["path"])
    checkpoint = strict_load_approved_checkpoint(torch_module=torch, model=model, checkpoint_path=checkpoint_path)
    output.mkdir(parents=True); (output / "selection_traces").mkdir()
    free_vram, total_vram = torch.cuda.mem_get_info()
    metadata = {
        "approved_checkpoint_sha256": APPROVED_CHECKPOINT_SHA256,
        "approved_checkpoint_epoch": 14,
        "strict_checkpoint_loaded": True,
        "checkpoint_selection": CHECKPOINT_SELECTION,
        "initialization_history": "TorchVision ResNet18_Weights.IMAGENET1K_V1 followed by approved epoch-14 checkpoint",
        "optimizer_state_restored": False,
        "vector_dimension": VECTOR_DIMENSION,
        "identity_order": IDENTITIES,
        "seed": SEED,
        "python_hash_seed": PYTHON_HASH_SEED,
        "torch": torch.__version__,
        "torchvision": torchvision.__version__,
        "cuda": torch.version.cuda,
        "gpu": torch.cuda.get_device_name(0),
        "driver_version": _driver_version(),
        "total_vram_bytes": int(total_vram),
        "free_vram_bytes_at_start": int(free_vram),
        "execution_profile": execution_profile,
        "automatic_mixed_precision": amp_enabled,
        "batch_size": batch_size,
        "gradient_accumulation_steps": accumulation_steps,
        "effective_batch_size": batch_size * accumulation_steps,
        "num_workers": num_workers,
        "pin_memory": pin_memory,
        "persistent_workers": persistent_workers,
        "checkpoint_parameter_verification": checkpoint["_strict_load_verification"],
        "categorical_encoding_contract": "explicit_vectorizer_name_index_v1",
    }
    (output / "run_metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")
    payload_history: list[dict[str, Any]] = []
    if config["stage"] == "full":
        baseline = evaluate_records(
            torch, model, val, vectorizer, stats,
            batch_size=batch_size,
            num_workers=num_workers,
            pin_memory=pin_memory,
            persistent_workers=persistent_workers,
            automatic_mixed_precision=amp_enabled,
        )
        (output / "full_baseline_validation_metrics.json").write_text(
            json.dumps(baseline, indent=2, sort_keys=True) + "\n"
        )
        payload_history.append(
            payload_warning_metrics(baseline, point="approved_epoch_14_baseline")
        )
        (output / "payload_warning_history.json").write_text(
            json.dumps(payload_history, indent=2, sort_keys=True) + "\n"
        )
    optimizer = torch.optim.AdamW((p for p in model.parameters() if p.requires_grad), lr=float(config["optimizer"]["learning_rate"]), weight_decay=float(config["optimizer"]["weight_decay"]))
    best = float("inf"); patience = 0; history = []; start = time.time()
    scaler = torch.amp.GradScaler("cuda", enabled=amp_enabled)
    torch.cuda.reset_peak_memory_stats()
    old_map = {row_id(x["row"]): x for x in old}; new_map = {row_id(x["row"]): x for x in new}
    for continuation_epoch in range(1, int(config["training"]["maximum_continuation_epochs"]) + 1):
        plan = source_balanced_epoch(list(old_map), list(new_map), seed=SEED, epoch=continuation_epoch, per_source=max(len(old_map), len(new_map)))
        report = sampling_report(plan); (output / "selection_traces" / f"epoch_{continuation_epoch:02d}.json").write_text(json.dumps({"report": report, "selection": plan}, indent=2, sort_keys=True) + "\n")
        records = [(old_map if item["source"] == "old_replay" else new_map)[item["sample_id"]] for item in plan]
        train_loader = loader(
            torch,
            records,
            vectorizer,
            stats,
            batch_size=batch_size,
            num_workers=num_workers,
            pin_memory=pin_memory,
            persistent_workers=persistent_workers,
        )
        model.train(); accum = defaultdict(float); source_accum: dict[str, defaultdict[str, float]] = {"old_replay": defaultdict(float), "v3r1_hard_case": defaultdict(float)}; batch_count = 0; source_batches = defaultdict(int)
        epoch_start = time.perf_counter()
        optimizer.zero_grad(set_to_none=True)
        for batch_index, batch in enumerate(train_loader, 1):
            image = batch["image"].to(device); target = batch["target"].to(device); mask = batch["target_mask"].to(device)
            with torch.autocast(
                device_type="cuda", dtype=torch.float16, enabled=amp_enabled
            ):
                prediction = model(image)
                losses = visual_loss(
                    torch, prediction, target, mask, vectorizer["names"]
                )
                backward_loss = losses["total"] / float(accumulation_steps)
            scaler.scale(backward_loss).backward()
            if (
                batch_index % accumulation_steps == 0
                or batch_index == len(train_loader)
            ):
                scaler.step(optimizer)
                scaler.update()
                optimizer.zero_grad(set_to_none=True)
            for key, value in losses.items(): accum[key] += float(value.item())
            for source in source_accum:
                indexes = [i for i, item in enumerate(batch["source"]) if item == source]
                if indexes:
                    part = visual_loss(torch, prediction[indexes], target[indexes], mask[indexes], vectorizer["names"])
                    for key, value in part.items(): source_accum[source][key] += float(value.item())
                    source_batches[source] += 1
            batch_count += 1
        validation = evaluate_records(
            torch, model, val, vectorizer, stats,
            batch_size=batch_size,
            num_workers=num_workers,
            pin_memory=pin_memory,
            persistent_workers=persistent_workers,
            automatic_mixed_precision=amp_enabled,
        )
        source_losses = {s: {k:v/max(1,source_batches[s]) for k,v in values.items()} for s,values in source_accum.items()}
        entry = {"continuation_epoch": continuation_epoch, "absolute_epoch": 14 + continuation_epoch, "learning_rate": optimizer.param_groups[0]["lr"], "sampler": report, "train_losses": {k: v/max(1,batch_count) for k,v in accum.items()}, "source_specific_train_losses": source_losses, "validation": validation, "automatic_mixed_precision": amp_enabled, "batch_size": batch_size, "gradient_accumulation_steps": accumulation_steps, "epoch_runtime_s": time.perf_counter() - epoch_start, "peak_allocated_vram_bytes": torch.cuda.max_memory_allocated(), "peak_reserved_vram_bytes": torch.cuda.max_memory_reserved()}
        history.append(entry); (output / "history.json").write_text(json.dumps(history, indent=2, sort_keys=True) + "\n")
        payload_history.append(payload_warning_metrics(
            validation,
            point=f"continuation_epoch_{continuation_epoch}",
            source_specific_train_losses=source_losses,
        ))
        (output / "payload_warning_history.json").write_text(
            json.dumps(payload_history, indent=2, sort_keys=True) + "\n"
        )
        current = float(validation["losses"]["total"])
        state = {"model_state_dict": model.state_dict(), "optimizer_state_dict": optimizer.state_dict(), "epoch": 14 + continuation_epoch, "continuation_epoch": continuation_epoch, "approved_source_checkpoint_sha256": APPROVED_CHECKPOINT_SHA256, "label_vectorizer": vectorizer, "target_stats": stats, "validation_metrics": validation}
        torch.save(state, output / "last.pt")
        if current < best:
            best = current; patience = 0; torch.save(state, output / "best.pt")
        else: patience += 1
        if patience >= int(config["training"]["early_stopping_patience"]): break
    best_checkpoint = torch.load(output / "best.pt", map_location="cpu", weights_only=False)
    model.load_state_dict(best_checkpoint["model_state_dict"], strict=True)
    best_validation = evaluate_records(
        torch, model, val, vectorizer, stats,
        batch_size=batch_size,
        num_workers=num_workers,
        pin_memory=pin_memory,
        persistent_workers=persistent_workers,
        automatic_mixed_precision=amp_enabled,
    )
    (output / "final_best_validation_metrics.json").write_text(
        json.dumps(best_validation, indent=2, sort_keys=True) + "\n"
    )
    final = {"status": "completed", "runtime_s": time.time()-start, "epochs_completed": len(history), "best_validation_total_weighted_loss": best, "best_checkpoint_sha256": sha256_file(output / "best.pt"), "last_checkpoint_sha256": sha256_file(output / "last.pt"), "best_checkpoint_validation_metrics": best_validation, "checkpoint_selected_using": "V3R1 validation total weighted loss only", "deployment_approval_requires_separate_canary_comparison": True, "peak_gpu_memory_bytes": torch.cuda.max_memory_allocated(), "peak_allocated_vram_bytes": torch.cuda.max_memory_allocated(), "peak_reserved_vram_bytes": torch.cuda.max_memory_reserved(), "automatic_mixed_precision": amp_enabled, "execution_profile": execution_profile}
    (output / "final_report.json").write_text(json.dumps(final, indent=2, sort_keys=True) + "\n")


def canary(
    config_path: Path,
    checkpoint_path: Path,
    output: Path,
    *,
    execution_profile: str = CONFIGURED_DEFAULT_PROFILE,
) -> None:
    if output.exists(): raise ExperimentAError(f"refusing to overwrite immutable output: {output}")
    config = read_json(config_path); torch, torchvision = require_torch(); set_deterministic(torch, SEED)
    vectorizer = read_json(
        expand_path(config["artifacts"]["vectorizer"]["path"])
    )
    stats = read_json(
        expand_path(config["artifacts"]["target_stats"]["path"])
    )
    runtime = effective_training_config(config, execution_profile)
    amp_enabled = bool(runtime.get("automatic_mixed_precision", False))
    batch_size = int(runtime.get("batch_size", 32))
    records = load_source(config["data"]["v3r1_canary"], "v3r1_canary")
    baseline_model = build_visual_state_model(torch, torchvision, output_dim=VECTOR_DIMENSION, adaptation_mode="partial_finetune").to("cuda")
    baseline_checkpoint = strict_load_approved_checkpoint(
        torch_module=torch,
        model=baseline_model,
        checkpoint_path=expand_path(config["artifacts"]["approved_checkpoint"]["path"]),
    )
    baseline_metrics = evaluate_records(
        torch, baseline_model, records, vectorizer, stats,
        batch_size=batch_size,
        num_workers=int(runtime.get("num_workers", 4)),
        pin_memory=bool(runtime.get("pin_memory", True)),
        persistent_workers=bool(runtime.get("persistent_workers", False)),
        automatic_mixed_precision=amp_enabled,
    )
    model = build_visual_state_model(torch, torchvision, output_dim=VECTOR_DIMENSION, adaptation_mode="partial_finetune").to("cuda")
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False); model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    metrics = evaluate_records(
        torch, model, records, vectorizer, stats,
        batch_size=batch_size,
        num_workers=int(runtime.get("num_workers", 4)),
        pin_memory=bool(runtime.get("pin_memory", True)),
        persistent_workers=bool(runtime.get("persistent_workers", False)),
        automatic_mixed_precision=amp_enabled,
        retain=True,
    )
    retained = metrics.pop("_records"); groups: dict[str, list[tuple[Any,...]]] = defaultdict(list)
    for record in retained:
        trace = record[3]
        for key, value in (("presence_class", trace.get("presence_class")), ("relation_family", trace.get("relation_family")), ("target_offset_bucket", trace.get("target_offset_bucket")), ("matched_pair_role", trace.get("matched_pair_role"))): groups[f"{key}:{value}"].append(record)
        active = set(trace.get("active_identities") or [])
        if {"L4", "R4"}.issubset(active): groups["combined_L4_R4"].append(record)
        if active == {"L2", "L4", "R4"}: groups["exact_L2_L4_R4"].append(record)
        if trace.get("operational_target_name") == "right_slot_3": groups["deliberate_right_slot_3_offsets"].append(record)
    subgroup = {name: summarize_predictions([r[0] for r in values], [r[1] for r in values], [r[2] for r in values], vectorizer["names"]) for name, values in groups.items()}
    payload = {"role": "post_training_development_regression_only", "training_performed": False, "checkpoint_selection_performed": False, "approved_baseline_checkpoint_sha256": APPROVED_CHECKPOINT_SHA256, "approved_baseline_checkpoint_load": baseline_checkpoint["_strict_load_verification"], "full_checkpoint_sha256": sha256_file(checkpoint_path), "approved_baseline": baseline_metrics, "full_best": metrics, "loaded_accuracy_delta": metrics["loaded_state_accuracy"] - baseline_metrics["loaded_state_accuracy"], "block_top1_delta": metrics["block_top1_accuracy"] - baseline_metrics["block_top1_accuracy"], "s_m_mae_delta": metrics["s_m_mae"] - baseline_metrics["s_m_mae"], "L4_loaded_recall": metrics["per_identity_loaded_state"]["L4"]["loaded_recall"], "R4_loaded_recall": metrics["per_identity_loaded_state"]["R4"]["loaded_recall"], "R3_loaded_recall": metrics["per_identity_loaded_state"]["R3"]["loaded_recall"], "subgroups": subgroup, "automatic_mixed_precision": amp_enabled, "execution_profile": execution_profile, "peak_allocated_vram_bytes": torch.cuda.max_memory_allocated(), "peak_reserved_vram_bytes": torch.cuda.max_memory_reserved(), "automatic_deployment_approval": False}
    output.mkdir(parents=True); (output / "canary_metrics.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(); parser.add_argument("--config", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); parser.add_argument("--mode", choices=("train", "canary"), required=True); parser.add_argument("--checkpoint", type=Path); parser.add_argument("--execution-profile", default=CONFIGURED_DEFAULT_PROFILE)
    args = parser.parse_args()
    if args.mode == "train": train(args.config, args.output, execution_profile=args.execution_profile)
    else:
        if not args.checkpoint: parser.error("--checkpoint is required for Canary")
        canary(args.config, args.checkpoint, args.output, execution_profile=args.execution_profile)


if __name__ == "__main__": main()
