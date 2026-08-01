#!/usr/bin/env python3
"""Stage only authorized Experiment-A inputs for transfer to Kairos."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from experiment_a_core import (  # noqa: E402
    ExperimentAError,
    expand_path,
    read_json,
    read_jsonl,
    reject_forbidden_artifact,
    sha256_file,
)


def copy_verified(source: Path, destination: Path, expected: str) -> None:
    reject_forbidden_artifact(source)
    if sha256_file(source) != expected:
        raise ExperimentAError(f"source hash mismatch: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    if sha256_file(destination) != expected:
        raise ExperimentAError(f"staged copy hash mismatch: {destination}")


def copy_images(rows_path: Path, source_root: Path, destination_root: Path) -> int:
    count = 0
    for row in read_jsonl(rows_path):
        refs = row.get("model_input", {}).get("overhead_images", {})
        for camera in ("left_rail_rgb", "right_rail_rgb"):
            relative = Path(str(refs.get(camera) or ""))
            if not relative.as_posix() or relative.is_absolute() or ".." in relative.parts:
                raise ExperimentAError(f"unsafe or missing {camera} reference")
            source = source_root / relative
            destination = destination_root / relative
            reject_forbidden_artifact(source)
            if not source.is_file():
                raise ExperimentAError(f"missing authorized image: {source}")
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)
            count += 1
    return count


def stage(config_path: Path, output: Path) -> dict[str, Any]:
    if output.exists():
        raise ExperimentAError(f"refusing to overwrite input staging root: {output}")
    config = read_json(config_path)
    output.mkdir(parents=True)
    layout = {
        "approved_run": output / "approved_run",
        "old_splits": output / "old_splits",
        "old_dataset": output / "old_dataset",
        "v3r1_splits": output / "v3r1_splits",
        "v3r1_dataset": output / "v3r1_dataset",
        "v3r1_canary": output / "v3r1_canary",
        "v3r1_guard": output / "v3r1_guard",
    }
    artifact_destinations = {
        "approved_checkpoint": layout["approved_run"] / "best.pt",
        "target_stats": layout["approved_run"] / "target_stats.json",
        "vectorizer": layout["approved_run"] / "visual_label_vectorizer.json",
        "training_config": layout["approved_run"] / "training_config.json",
        "run_metadata": layout["approved_run"] / "run_metadata.json",
        "v3r1_package_manifest": layout["v3r1_splits"] / "package_manifest.json",
        "v3r1_final_audit": layout["v3r1_guard"] / "dataset_v3r1_audit.json",
        "v3r1_dataset_manifest": layout["v3r1_guard"] / "dataset_manifest.json",
    }
    files = []
    for name, destination in artifact_destinations.items():
        spec = config["artifacts"][name]
        source = expand_path(spec["path"])
        copy_verified(source, destination, spec["sha256"])
        files.append({"role": name, "path": destination.relative_to(output).as_posix(), "sha256": spec["sha256"]})

    source_layout = {
        "old_replay": (layout["old_splits"], layout["old_dataset"]),
        "v3r1_train": (layout["v3r1_splits"], layout["v3r1_dataset"]),
        "v3r1_validation": (layout["v3r1_splits"], layout["v3r1_dataset"]),
        "v3r1_canary": (layout["v3r1_canary"] / "finalized", layout["v3r1_canary"] / "dataset"),
    }
    image_counts: dict[str, int] = {}
    copied_refs: set[tuple[str, str]] = set()
    for name in ("old_replay", "v3r1_train", "v3r1_validation", "v3r1_canary"):
        spec = config["data"][name]
        split_destination, image_destination = source_layout[name]
        rows_source = expand_path(spec["rows"])
        labels_source = expand_path(spec["labels"])
        rows_destination = split_destination / rows_source.name
        labels_destination = split_destination / labels_source.name
        copy_verified(rows_source, rows_destination, spec["rows_artifact"]["sha256"])
        copy_verified(labels_source, labels_destination, spec["labels_artifact"]["sha256"])
        files.extend([
            {"role": f"{name}_rows", "path": rows_destination.relative_to(output).as_posix(), "sha256": spec["rows_artifact"]["sha256"]},
            {"role": f"{name}_labels", "path": labels_destination.relative_to(output).as_posix(), "sha256": spec["labels_artifact"]["sha256"]},
        ])
        image_counts[name] = copy_images(
            rows_source, expand_path(spec["dataset_root"]), image_destination
        )
    report = {
        "schema_version": "room315.experiment_a.staged_inputs.v1",
        "authorized_sources_only": True,
        "legacy_evaluation_data_included": False,
        "files": files,
        "image_counts": image_counts,
        "environment": {
            "ROOM315_APPROVED_RUN_ROOT": str(layout["approved_run"]),
            "ROOM315_OLD_SPLITS_ROOT": str(layout["old_splits"]),
            "ROOM315_OLD_DATASET_ROOT": str(layout["old_dataset"]),
            "ROOM315_V3R1_SPLITS_ROOT": str(layout["v3r1_splits"]),
            "ROOM315_V3R1_DATASET_ROOT": str(layout["v3r1_dataset"]),
            "ROOM315_V3R1_CANARY_ROOT": str(layout["v3r1_canary"]),
            "ROOM315_V3R1_GUARD_ROOT": str(layout["v3r1_guard"]),
        },
    }
    (output / "staged_inputs_manifest.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(stage(args.config, args.output), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
