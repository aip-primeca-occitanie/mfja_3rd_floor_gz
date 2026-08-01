#!/usr/bin/env python3
"""Read-only RTX memory preflight for local Experiment-A execution."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from experiment_a_core import (  # noqa: E402
    LOCAL_DEFAULT_PROFILE,
    LOCAL_FALLBACK_PROFILE,
    VECTOR_DIMENSION,
    ExperimentAError,
    effective_training_config,
    expand_path,
    read_json,
    row_id,
    source_balanced_epoch,
    strict_load_approved_checkpoint,
    validate_target_stats,
    validate_vectorizer,
)
from experiment_a_train import (  # noqa: E402
    load_source,
    loader,
    pick_ids,
    require_torch,
    set_deterministic,
    visual_loss,
)
from experiment_a_verify import static_audit  # noqa: E402
from room_315_visual_model import build_visual_state_model  # noqa: E402


def nvidia_driver() -> str | None:
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


def runtime_report(torch: Any, torchvision: Any, config: dict[str, Any]) -> dict[str, Any]:
    if not torch.cuda.is_available():
        raise ExperimentAError("local RTX profile requires CUDA")
    gpu = torch.cuda.get_device_name(0)
    expected_gpu = str(config["local_runtime"]["expected_gpu_name"])
    if expected_gpu.casefold() not in gpu.casefold():
        raise ExperimentAError(f"expected local GPU {expected_gpu!r}, found {gpu!r}")
    if not torch.amp.autocast_mode.is_autocast_available("cuda"):
        raise ExperimentAError("CUDA automatic mixed precision is unavailable")
    free_bytes, total_bytes = torch.cuda.mem_get_info()
    return {
        "cuda_available": True,
        "gpu_name": gpu,
        "total_vram_bytes": int(total_bytes),
        "free_vram_bytes": int(free_bytes),
        "driver_version": nvidia_driver(),
        "cuda_version": torch.version.cuda,
        "torch_version": torch.__version__,
        "torchvision_version": torchvision.__version__,
        "amp_available": True,
        "expected_versions": config["local_runtime"]["expected_versions"],
    }


def training_records(config: dict[str, Any]) -> tuple[list[dict[str, Any]], dict[str, Any], dict[str, Any]]:
    old = load_source(config["data"]["old_replay"], "old_replay")
    new = load_source(config["data"]["v3r1_train"], "v3r1_hard_case")
    if config["stage"] == "smoke":
        selection = read_json(expand_path(config["smoke_selection"]))
        old = pick_ids(old, selection["train"]["old_replay"])
        new = pick_ids(new, selection["train"]["v3r1_hard_case"])
    old_map = {row_id(item["row"]): item for item in old}
    new_map = {row_id(item["row"]): item for item in new}
    plan = source_balanced_epoch(
        list(old_map),
        list(new_map),
        seed=int(config["seed"]),
        epoch=1,
        per_source=max(len(old_map), len(new_map)),
    )
    records = [
        (old_map if item["source"] == "old_replay" else new_map)[item["sample_id"]]
        for item in plan
    ]
    return records, old_map, new_map


def memory_preflight(
    *,
    config_path: Path,
    package_root: Path,
    output: Path,
    report_path: Path,
    execution_profile: str,
    fallback_command: str,
) -> dict[str, Any]:
    if output.exists():
        raise ExperimentAError(f"immutable local output already exists: {output}")
    if report_path.exists():
        raise ExperimentAError(f"preflight report already exists: {report_path}")
    config = read_json(config_path)
    runtime = effective_training_config(config, execution_profile)
    if execution_profile == LOCAL_FALLBACK_PROFILE and not bool(
        config["execution_profiles"][execution_profile].get("explicit_cli_flag")
    ):
        raise ExperimentAError("fallback profile must require an explicit CLI flag")
    if bool(config.get("automatic_fallback", True)):
        raise ExperimentAError("automatic fallback must be disabled")

    static = static_audit(config_path, package_root, decode_images=False)
    if "v3r1_canary" in static["verification_sources"]:
        raise ExperimentAError("local training preflight must leave Canary untouched")
    torch, torchvision = require_torch()
    set_deterministic(torch, int(config["seed"]))
    environment = runtime_report(torch, torchvision, config)
    vectorizer = read_json(expand_path(config["artifacts"]["vectorizer"]["path"]))
    stats = read_json(expand_path(config["artifacts"]["target_stats"]["path"]))
    validate_vectorizer(vectorizer)
    validate_target_stats(stats)
    records, _, _ = training_records(config)
    batch_size = int(runtime["batch_size"])
    sample_loader = loader(
        torch,
        records[:batch_size],
        vectorizer,
        stats,
        batch_size=batch_size,
        num_workers=int(runtime.get("num_workers", 4)),
        pin_memory=bool(runtime.get("pin_memory", True)),
        persistent_workers=bool(runtime.get("persistent_workers", False)),
    )
    model = build_visual_state_model(
        torch,
        torchvision,
        output_dim=VECTOR_DIMENSION,
        adaptation_mode="partial_finetune",
    ).to("cuda")
    strict_load_approved_checkpoint(
        torch_module=torch,
        model=model,
        checkpoint_path=expand_path(config["artifacts"]["approved_checkpoint"]["path"]),
    )
    scaler = torch.amp.GradScaler("cuda", enabled=bool(runtime["automatic_mixed_precision"]))
    torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats()
    started = time.perf_counter()
    try:
        batch = next(iter(sample_loader))
        image = batch["image"].to("cuda")
        target = batch["target"].to("cuda")
        mask = batch["target_mask"].to("cuda")
        model.train()
        with torch.autocast(
            device_type="cuda",
            dtype=torch.float16,
            enabled=bool(runtime["automatic_mixed_precision"]),
        ):
            prediction = model(image)
            if tuple(prediction.shape) != (batch_size, VECTOR_DIMENSION):
                raise ExperimentAError(
                    f"preflight output shape mismatch: {tuple(prediction.shape)}"
                )
            losses = visual_loss(torch, prediction, target, mask, vectorizer["names"])
        if not all(math.isfinite(float(value.detach().item())) for value in losses.values()):
            raise ExperimentAError("preflight produced a non-finite loss")
        scaler.scale(losses["total"]).backward()
        torch.cuda.synchronize()
    except torch.OutOfMemoryError as exc:
        torch.cuda.empty_cache()
        failure = {
            "passed": False,
            "failure": "cuda_out_of_memory",
            "execution_profile": execution_profile,
            "runtime": environment,
            "fallback_was_started": False,
            "fallback_command": fallback_command,
        }
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(failure, indent=2, sort_keys=True) + "\n")
        print("LOCAL_MEMORY_PREFLIGHT_CUDA_OOM", file=sys.stderr)
        print("Training was not started and no automatic retry was attempted.", file=sys.stderr)
        print(f"Run this explicit fallback command:\n{fallback_command}", file=sys.stderr)
        raise SystemExit(42) from exc
    report = {
        "passed": True,
        "kind": "one_forward_backward_batch",
        "execution_profile": execution_profile,
        "batch_size": batch_size,
        "gradient_accumulation_steps": int(runtime["gradient_accumulation_steps"]),
        "automatic_mixed_precision": bool(runtime["automatic_mixed_precision"]),
        "output_dimension": VECTOR_DIMENSION,
        "losses_finite": True,
        "runtime_s": time.perf_counter() - started,
        "peak_allocated_vram_bytes": int(torch.cuda.max_memory_allocated()),
        "peak_reserved_vram_bytes": int(torch.cuda.max_memory_reserved()),
        "runtime": environment,
        "static_verification_sources": static["verification_sources"],
        "categorical_semantic_self_test": static["categorical_semantic_self_test"],
        "stale_encoder_scan": static["stale_encoder_scan"],
        "fallback_was_started": False,
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, indent=2, sort_keys=True))
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--package-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument(
        "--execution-profile",
        choices=(LOCAL_DEFAULT_PROFILE, LOCAL_FALLBACK_PROFILE),
        required=True,
    )
    parser.add_argument("--fallback-command", required=True)
    args = parser.parse_args()
    memory_preflight(
        config_path=args.config,
        package_root=args.package_root,
        output=args.output,
        report_path=args.report,
        execution_profile=args.execution_profile,
        fallback_command=args.fallback_command,
    )


if __name__ == "__main__":
    main()
