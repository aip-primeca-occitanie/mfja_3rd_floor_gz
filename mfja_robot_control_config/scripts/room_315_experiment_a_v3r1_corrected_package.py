#!/usr/bin/env python3
"""Build the immutable corrected Room 315 Experiment-A V3R1 package."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path
from typing import Any


SEED = 31520260730
PACKAGE_NAME = f"room315_experiment_a_v3r1_corrected_package_seed{SEED}"
DEFAULT_OUTPUT = Path("/home/tiago") / PACKAGE_NAME
DEFAULT_ARCHIVE = Path("/home/tiago") / f"{PACKAGE_NAME}.tar.gz"
LOCAL_OUTPUT_ROOT = Path("/home/tiago/room315_experiment_a_corrected_local_outputs")
LOCAL_GUARD = Path("/home/tiago/room315_experiment_a_corrected_local_guard_state.json")
LOCAL_PREFLIGHT_ROOT = Path("/home/tiago/room315_experiment_a_corrected_local_preflight_reports")
LOCAL_TORCH_PYTHON = Path("/home/tiago/room315_local_training/venv/bin/python")
APPROVED_SHA = "8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d"
IDENTITIES = ("L1", "L2", "L3", "L4", "R1", "R2", "R3", "R4")

FROZEN_TREES = {
    "/home/tiago/room315_kairos_visual_state_experiment_a_v3r1_package_seed31520260730":
        "54856ba028e08f90f1e6307a4d01a50441ba6ed0a249521ed1db6c21b7c9a14b",
    "/home/tiago/room315_experiment_a_local_outputs/smoke_seed31520260730_attempt1":
        "811e7706560b9540dbdf2ec676858382a01f23d7cbc0dd8ed2d5021fc1ad146d",
    "/home/tiago/room315_experiment_a_local_outputs/smoke_v2_seed31520260730_attempt1":
        "3ca035edcc22c046126f0317093439e5a55fa25dde7a21619fe08c7c64a7b051",
    "/home/tiago/room315_experiment_a_local_outputs/smoke_v2_seed31520260730_attempt2":
        "8576bd7fad94b321da97c1a012dcfca1abb990c7eb2f0c0b4f76f14e9096bfb8",
    "/home/tiago/room315_experiment_a_local_smoke_v2_package_seed31520260730_attempt2":
        "25b5cbd6d0d1360ce3793abf8c2297851f1baeeca8006f6f120b5399300ebca2",
}
FROZEN_FILES = {
    "/home/tiago/room315_kairos_visual_state_experiment_a_v3r1_package_seed31520260730.tar.gz":
        "19dd5fd97b9f7c298df0f42d9f8e58264a5a312ef64cbac7bb2282d5026f2403",
    "/home/tiago/room315_experiment_a_local_smoke_v2_package_seed31520260730_attempt2.tar.gz":
        "36aa896bd038725a5554b1e63d40bb15aa46593f79e8a4029fc3886d417b103c",
    "/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run/best.pt":
        APPROVED_SHA,
}
STALE_PATTERNS = (
    re.compile(r"categorical_values\s*\.\s*items\s*\("),
    re.compile(r"vector\s*\.\s*extend\s*\([^\n]*raw\s*=="),
)


class CorrectedPackageError(RuntimeError):
    pass


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


def write(path: Path, text: str, *, executable: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    if executable:
        path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP)


def write_json(path: Path, value: Any) -> None:
    write(path, json.dumps(value, indent=2, sort_keys=True) + "\n")


def tree_sha256(root: Path, sha_file) -> str:
    import hashlib
    digest = hashlib.sha256()
    for path in sorted(root.rglob("*")):
        if not path.is_file() or "__pycache__" in path.parts or path.suffix == ".pyc":
            continue
        relative = path.relative_to(root).as_posix()
        digest.update(
            f"{relative}\0{path.stat().st_size}\0{sha_file(path)}\n".encode()
        )
    return digest.hexdigest()


def frozen_integrity(base) -> dict[str, Any]:
    trees = {}
    files = {}
    for raw_path, expected in FROZEN_TREES.items():
        path = Path(raw_path)
        actual = tree_sha256(path, base.sha)
        if actual != expected:
            raise CorrectedPackageError(f"frozen tree changed: {path}")
        trees[raw_path] = {"expected": expected, "actual": actual, "unchanged": True}
    for raw_path, expected in FROZEN_FILES.items():
        path = Path(raw_path)
        actual = base.sha(path)
        if actual != expected:
            raise CorrectedPackageError(f"frozen file changed: {path}")
        files[raw_path] = {"expected": expected, "actual": actual, "unchanged": True}
    return {"passed": True, "trees": trees, "files": files}


def corrected_config(base, stage: str, *, local: bool) -> dict[str, Any]:
    config = (
        base.local_config(stage, "${ROOM315_EXPERIMENT_A_PACKAGE_ROOT}")
        if local
        else base.base_config(stage, "${ROOM315_EXPERIMENT_A_PACKAGE_ROOT}")
    )
    config["schema_version"] = (
        "room315.experiment_a.corrected.local_config.v1"
        if local else "room315.experiment_a.corrected.kairos_config.v1"
    )
    config["experiment"] = "Experiment-A corrected explicit-name continuation baseline"
    config["categorical_encoding"] = {
        "contract": "explicit_vectorizer_name_index_v1",
        "authoritative_order": ["side", "block", "loaded_state"],
        "dictionary_order_is_semantic": False,
        "fail_closed_one_hot": True,
    }
    config["verification_sources"] = (
        ["old_replay", "v3r1_train", "v3r1_validation", "v3r1_canary"]
        if stage == "canary"
        else ["old_replay", "v3r1_train", "v3r1_validation"]
    )
    config["training"].update({
        "batch_size": 32,
        "gradient_accumulation_steps": 1,
        "automatic_mixed_precision": True,
        "num_workers": 4,
        "pin_memory": True,
        "persistent_workers": True,
        "maximum_continuation_epochs": 2 if stage == "smoke" else 10,
        "early_stopping_patience": 3,
        "old_replay_references_per_epoch": 4000,
        "v3r1_hard_case_references_per_epoch": 4000,
    })
    if stage == "canary":
        config["data_roles"]["checkpoint_selection"] = "none_canary_evaluation"
        config["data_roles"]["training_sources"] = []
        config["training"].update({
            "training_enabled": False,
            "maximum_continuation_epochs": 0,
            "early_stopping_patience": 0,
            "checkpoint_selection": "none",
            "old_replay_references_per_epoch": 0,
            "v3r1_hard_case_references_per_epoch": 0,
            "source_balance": {},
        })
        config["canary_contract"] = {
            "training_performed": False,
            "checkpoint_selection_performed": False,
            "same_examples_for_approved_and_candidate": True,
            "automatic_deployment_approval": False,
        }
    config["full_baseline"] = {
        "checkpoint": "approved_epoch_14",
        "validation_scenarios": 512,
        "must_run_before_optimizer_creation": True,
        "output": "full_baseline_validation_metrics.json",
    }
    config["payload_warning_metrics"] = {
        "immutable_per_epoch": True,
        "overall_loaded_accuracy": True,
        "per_identity_loaded_recall": True,
        "per_identity_empty_specificity": True,
        "highlight_identities": ["L4", "R4", "R3"],
        "source_specific_loaded_state_loss": True,
        "checkpoint_selection_impact": "none",
    }
    config["deployment"] = {
        "automatic_approval": False,
        "requires_separate_canary_against_approved_epoch_14": True,
    }
    if local:
        config["automatic_fallback"] = False
        config["local_isolation"] = {
            "output_root": str(LOCAL_OUTPUT_ROOT),
            "guard_state": str(LOCAL_GUARD),
            "preflight_reports": str(LOCAL_PREFLIGHT_ROOT),
            "kairos_output_root_must_not_be_used": True,
        }
    return config


def guard_template(*, local: bool) -> dict[str, Any]:
    stages = (
        ("corrected_local_full", "corrected_local_canary")
        if local
        else ("corrected_kairos_smoke", "corrected_kairos_full", "corrected_kairos_canary")
    )
    return {
        "schema_version": (
            "room315.experiment_a.corrected.local_guard.v1"
            if local else "room315.experiment_a.corrected.kairos_guard.v1"
        ),
        "approved_checkpoint_sha256": APPROVED_SHA,
        "legacy_evaluation_authorized": False,
        "automatic_retry": False,
        "output_root": str(LOCAL_OUTPUT_ROOT) if local else None,
        "stages": {
            stage: {"state": "unauthorized", "attempts": 0, "output": None}
            for stage in stages
        },
    }


def local_launcher(stage: str) -> str:
    is_canary = stage == "canary"
    guard_stage = f"corrected_local_{stage}"
    config = f"local_{stage}_{'evaluation' if is_canary else 'training'}.json"
    mode = "canary" if is_canary else "train"
    checkpoint_setup = (
        ': "${ROOM315_EXPERIMENT_A_CORRECTED_LOCAL_BEST_CHECKPOINT:?set the completed corrected Full best.pt}"\n'
        if is_canary else ""
    )
    checkpoint_arg = (
        ' --checkpoint "$ROOM315_EXPERIMENT_A_CORRECTED_LOCAL_BEST_CHECKPOINT"'
        if is_canary else ""
    )
    preflight = ""
    if not is_canary:
        preflight = f'''"$PYTHON" "$PACKAGE_ROOT/scripts/experiment_a_local.py" \\
  --config "$CONFIG" --package-root "$PACKAGE_ROOT" \\
  --output "$OUTPUT" --report "$PREFLIGHT" \\
  --execution-profile default_batch32_amp \\
  --fallback-command "No automatic fallback; inspect and launch a new guarded attempt only after review."
'''
    else:
        preflight = '''"$PYTHON" "$PACKAGE_ROOT/scripts/experiment_a_verify.py" \\
  --config "$CONFIG" --package-root "$PACKAGE_ROOT" \\
  --verify-checkpoint-load >/dev/null
'''
    return f'''#!/usr/bin/env bash
set -Eeuo pipefail
PACKAGE_ROOT="$(cd -- "$(dirname -- "${{BASH_SOURCE[0]}}")" && pwd)"
PYTHON="${{ROOM315_LOCAL_PYTHON:-{LOCAL_TORCH_PYTHON}}}"
GUARD="${{ROOM315_EXPERIMENT_A_CORRECTED_LOCAL_GUARD:-{LOCAL_GUARD}}}"
OUTPUT_ROOT="${{ROOM315_EXPERIMENT_A_CORRECTED_LOCAL_OUTPUT_ROOT:-{LOCAL_OUTPUT_ROOT}}}"
PREFLIGHT_ROOT="${{ROOM315_EXPERIMENT_A_CORRECTED_LOCAL_PREFLIGHT_ROOT:-{LOCAL_PREFLIGHT_ROOT}}}"
[[ -x "$PYTHON" ]] || {{ echo "missing local Torch Python: $PYTHON" >&2; exit 1; }}
[[ -f "$GUARD" ]] || {{ echo "missing corrected local guard: $GUARD" >&2; exit 1; }}
export ROOM315_EXPERIMENT_A_PACKAGE_ROOT="$PACKAGE_ROOT"
export ROOM315_APPROVED_RUN_ROOT="${{ROOM315_APPROVED_RUN_ROOT:-/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run}}"
export ROOM315_OLD_SPLITS_ROOT="${{ROOM315_OLD_SPLITS_ROOT:-/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730}}"
export ROOM315_OLD_DATASET_ROOT="${{ROOM315_OLD_DATASET_ROOT:-/home/tiago/room315_arbitrary_subset_visual_2040_capture_v2_seed31520260729/dataset}}"
export ROOM315_V3R1_SPLITS_ROOT="${{ROOM315_V3R1_SPLITS_ROOT:-/home/tiago/room315_hard_case_visual_v3r1_splits_seed31520260730}}"
export ROOM315_V3R1_DATASET_ROOT="${{ROOM315_V3R1_DATASET_ROOT:-/home/tiago/room315_hard_case_visual_v3r1_capture_seed31520260730/dataset}}"
export ROOM315_V3R1_CANARY_ROOT="${{ROOM315_V3R1_CANARY_ROOT:-/home/tiago/room315_hard_case_visual_v3r1_canary_seed31520260730}}"
export ROOM315_V3R1_GUARD_ROOT="${{ROOM315_V3R1_GUARD_ROOT:-/home/tiago/room315_hard_case_visual_v3r1_guard_seed31520260730}}"
export PYTHONHASHSEED=1455489658
export CUBLAS_WORKSPACE_CONFIG=:4096:8
{checkpoint_setup}CONFIG="$PACKAGE_ROOT/config/{config}"
OUTPUT="$OUTPUT_ROOT/{stage}_seed{SEED}_attempt1"
PREFLIGHT="$PREFLIGHT_ROOT/{stage}_batch32_amp_seed{SEED}.json"
[[ ! -e "$OUTPUT" ]] || {{ echo "refusing to overwrite: $OUTPUT" >&2; exit 1; }}
{preflight}"$PYTHON" "$PACKAGE_ROOT/scripts/experiment_a_guard.py" begin \\
  --guard "$GUARD" --stage {guard_stage} --output "$OUTPUT"
finish() {{ "$PYTHON" "$PACKAGE_ROOT/scripts/experiment_a_guard.py" "$1" --guard "$GUARD" --stage {guard_stage}; }}
trap 'finish fail' ERR
"$PYTHON" "$PACKAGE_ROOT/scripts/experiment_a_train.py" \\
  --config "$CONFIG" --mode {mode} --output "$OUTPUT" \\
  --execution-profile default_batch32_amp{checkpoint_arg}
trap - ERR
finish complete
'''


def inside_container() -> str:
    return '''#!/usr/bin/env python3
import argparse, os, subprocess, sys
from pathlib import Path

def verify(a):
    subprocess.run([
        sys.executable, str(a.package_root / "scripts/experiment_a_verify.py"),
        "--config", str(a.config), "--package-root", str(a.package_root),
        "--require-gh200", "--decode-images", "--verify-checkpoint-load",
    ], check=True)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--package-root",type=Path,required=True)
    parser.add_argument("--config",type=Path,required=True)
    parser.add_argument("--mode",choices=("preflight","train","canary"),required=True)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--checkpoint",type=Path)
    args=parser.parse_args()
    os.environ["PYTHONHASHSEED"]="1455489658"
    os.environ["CUBLAS_WORKSPACE_CONFIG"]=":4096:8"
    verify(args)
    if args.mode == "preflight": return
    command=[sys.executable,str(args.package_root/"scripts/experiment_a_train.py"),
             "--config",str(args.config),"--mode",args.mode,
             "--output",str(args.output)]
    if args.checkpoint: command += ["--checkpoint",str(args.checkpoint)]
    subprocess.run(command,check=True)
if __name__ == "__main__": main()
'''


def kairos_launcher(stage: str) -> str:
    is_canary = stage == "canary"
    guard_stage = f"corrected_kairos_{stage}"
    config = f"{stage}_{'evaluation' if is_canary else 'training'}.json"
    mode = "canary" if is_canary else "train"
    checkpoint_setup = (
        ': "${ROOM315_EXPERIMENT_A_CORRECTED_BEST_CHECKPOINT:?set the completed corrected Full best.pt}"\n'
        if is_canary else ""
    )
    checkpoint_arg = (
        ' --checkpoint "$ROOM315_EXPERIMENT_A_CORRECTED_BEST_CHECKPOINT"'
        if is_canary else ""
    )
    return f'''#!/usr/bin/env bash
set -Eeuo pipefail
PACKAGE_ROOT="$(cd -- "$(dirname -- "${{BASH_SOURCE[0]}}")" && pwd)"
CONTAINER=/work/conteneurs/shared/AI/nemo_25.04.03_arm.sif
: "${{ROOM315_EXPERIMENT_A_CORRECTED_GUARD:?set corrected Kairos guard}}"
: "${{ROOM315_EXPERIMENT_A_CORRECTED_OUTPUT_ROOT:?set corrected Kairos output root}}"
[[ -f "$CONTAINER" ]] || {{ echo "missing container: $CONTAINER" >&2; exit 1; }}
command -v apptainer >/dev/null || {{ echo "apptainer unavailable" >&2; exit 1; }}
export ROOM315_EXPERIMENT_A_PACKAGE_ROOT="$PACKAGE_ROOT"
{checkpoint_setup}CONFIG="$PACKAGE_ROOT/config/{config}"
OUTPUT="$ROOM315_EXPERIMENT_A_CORRECTED_OUTPUT_ROOT/{stage}_seed{SEED}_attempt1"
[[ ! -e "$OUTPUT" ]] || {{ echo "refusing to overwrite: $OUTPUT" >&2; exit 1; }}
apptainer exec --nv --bind "$PACKAGE_ROOT:$PACKAGE_ROOT" "$CONTAINER" \\
  python3 "$PACKAGE_ROOT/scripts/experiment_a_inside_container.py" \\
  --package-root "$PACKAGE_ROOT" --config "$CONFIG" --mode preflight
python3 "$PACKAGE_ROOT/scripts/experiment_a_guard.py" begin \\
  --guard "$ROOM315_EXPERIMENT_A_CORRECTED_GUARD" --stage {guard_stage} --output "$OUTPUT"
finish() {{ python3 "$PACKAGE_ROOT/scripts/experiment_a_guard.py" "$1" --guard "$ROOM315_EXPERIMENT_A_CORRECTED_GUARD" --stage {guard_stage}; }}
trap 'finish fail' ERR
apptainer exec --nv --bind "$PACKAGE_ROOT:$PACKAGE_ROOT" "$CONTAINER" \\
  python3 "$PACKAGE_ROOT/scripts/experiment_a_inside_container.py" \\
  --package-root "$PACKAGE_ROOT" --config "$CONFIG" --mode {mode} \\
  --output "$OUTPUT"{checkpoint_arg}
trap - ERR
finish complete
'''


def readme() -> str:
    return f'''# Corrected Room 315 Experiment A V3R1

This immutable package replaces the historical order-dependent categorical
target encoder with explicit vectorizer-name indexing. It preserves the model,
approved epoch-14 initialization, target statistics, loss, optimizer, splits,
and validation-only checkpoint-selection contract.

Local output root: `{LOCAL_OUTPUT_ROOT}`

Local guard: `{LOCAL_GUARD}`

## Local initialization (do not authorize until ready)

```bash
cp /home/tiago/{PACKAGE_NAME}/config/corrected_local_guard_template.json \\
  {LOCAL_GUARD}
```

## Future corrected local Full

```bash
{LOCAL_TORCH_PYTHON} \\
  /home/tiago/{PACKAGE_NAME}/scripts/experiment_a_guard.py authorize \\
  --guard {LOCAL_GUARD} --stage corrected_local_full
/home/tiago/{PACKAGE_NAME}/run_local_experiment_a_corrected_full.sh
```

## Future corrected local Canary, only after Full completes

```bash
export ROOM315_EXPERIMENT_A_CORRECTED_LOCAL_BEST_CHECKPOINT=\
{LOCAL_OUTPUT_ROOT}/full_seed{SEED}_attempt1/best.pt
{LOCAL_TORCH_PYTHON} \\
  /home/tiago/{PACKAGE_NAME}/scripts/experiment_a_guard.py authorize \\
  --guard {LOCAL_GUARD} --stage corrected_local_canary
/home/tiago/{PACKAGE_NAME}/run_local_experiment_a_corrected_canary.sh
```

The Full launcher runs an explicit semantic and one-batch forward/backward
memory preflight before consuming authorization. It does not automatically
retry or switch batch sizes. Canary performs no training or checkpoint
selection and compares the frozen Full best checkpoint against the untouched
approved epoch-14 checkpoint.

Kairos launchers preserve `aarch64`, GH200, CUDA, Torch/TorchVision, Apptainer,
and `/work/conteneurs/shared/AI/nemo_25.04.03_arm.sif` requirements.
'''


def scan_tree(root: Path) -> dict[str, Any]:
    scanned = 0
    matches = []
    for path in sorted(root.rglob("*")):
        if not path.is_file() or path.suffix not in {".py", ".sh"}:
            continue
        scanned += 1
        text = path.read_text(encoding="utf-8")
        for pattern in STALE_PATTERNS:
            if pattern.search(text):
                matches.append({"path": path.relative_to(root).as_posix(), "pattern": pattern.pattern})
    if matches:
        raise CorrectedPackageError(f"stale encoder in package: {matches}")
    return {"passed": True, "files_scanned": scanned, "matches": []}


def scan_archive(archive: Path) -> dict[str, Any]:
    matches = []
    scanned = 0
    with tarfile.open(archive, "r:gz") as bundle:
        for member in bundle.getmembers():
            if not member.isfile() or Path(member.name).suffix not in {".py", ".sh"}:
                continue
            stream = bundle.extractfile(member)
            assert stream is not None
            text = stream.read().decode("utf-8")
            scanned += 1
            for pattern in STALE_PATTERNS:
                if pattern.search(text):
                    matches.append({"path": member.name, "pattern": pattern.pattern})
    if matches:
        raise CorrectedPackageError(f"stale encoder in archive: {matches}")
    return {"passed": True, "files_scanned": scanned, "matches": []}


def build(repo: Path, output: Path, archive: Path) -> dict[str, Any]:
    if output.exists() or archive.exists() or archive.with_suffix(archive.suffix + ".sha256").exists():
        raise CorrectedPackageError("refusing to overwrite corrected package/archive")
    base = load_module(
        "room315_experiment_a_base_builder",
        repo / "mfja_robot_control_config/scripts/room_315_experiment_a_v3r1_package.py",
    )
    frozen_before = frozen_integrity(base)
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.", dir=output.parent))
    module_root = repo / "mfja_robot_control_config/experiment_a_v3r1"
    try:
        (staging / "scripts").mkdir(parents=True, exist_ok=True)
        required = (
            "experiment_a_core.py", "experiment_a_guard.py", "experiment_a_local.py",
            "experiment_a_prepare_inputs.py", "experiment_a_train.py", "experiment_a_verify.py",
        )
        for name in required:
            shutil.copy2(module_root / name, staging / "scripts" / name)
        shutil.copy2(
            repo / "mfja_robot_control_config/scripts/room_315_visual_model.py",
            staging / "scripts/room_315_visual_model.py",
        )

        new_rows = base.jsonl(base.LOCAL_PATHS["v3r1_splits"] / "train.jsonl")
        val_rows = base.jsonl(base.LOCAL_PATHS["v3r1_splits"] / "validation.jsonl")
        old_rows = base.jsonl(base.LOCAL_PATHS["old_splits"] / "train.jsonl")
        import random
        indexes = list(range(len(old_rows))); random.Random(SEED + 101).shuffle(indexes)
        old_ids = [base.sample_id(old_rows[index]) for index in indexes[:64]]
        required_flags = ("L4_loaded", "R4_loaded", "exact_L2_L4_R4", "right_slot3_deliberate_offset", "hard_payload")
        new_ids, coverage = base.choose_smoke(new_rows, 64, required=required_flags, seed=SEED + 202)
        indexes = list(range(len(val_rows))); random.Random(SEED + 303).shuffle(indexes)
        val_ids = [base.sample_id(val_rows[index]) for index in indexes[:32]]
        write_json(staging / "config/smoke_selection.json", {
            "seed": SEED,
            "train": {"old_replay": old_ids, "v3r1_hard_case": new_ids},
            "validation": val_ids,
            "coverage": coverage,
            "canary_used": False,
            "categorical_encoding": "explicit_vectorizer_name_index_v1",
        })

        configs = {
            "smoke_training.json": corrected_config(base, "smoke", local=False),
            "full_training.json": corrected_config(base, "full", local=False),
            "canary_evaluation.json": corrected_config(base, "canary", local=False),
            "local_full_training.json": corrected_config(base, "full", local=True),
            "local_canary_evaluation.json": corrected_config(base, "canary", local=True),
        }
        for name, value in configs.items():
            write_json(staging / "config" / name, value)
        write_json(staging / "config/corrected_local_guard_template.json", guard_template(local=True))
        write_json(staging / "config/corrected_kairos_guard_template.json", guard_template(local=False))

        write(staging / "scripts/experiment_a_inside_container.py", inside_container(), executable=True)
        write(staging / "run_local_experiment_a_corrected_full.sh", local_launcher("full"), executable=True)
        write(staging / "run_local_experiment_a_corrected_canary.sh", local_launcher("canary"), executable=True)
        for stage in ("smoke", "full", "canary"):
            write(staging / f"run_kairos_experiment_a_corrected_{stage}.sh", kairos_launcher(stage), executable=True)
        write(staging / "README.md", readme())

        transfers = []
        for name, path in (
            ("approved_checkpoint", base.LOCAL_PATHS["approved_run"] / "best.pt"),
            ("target_stats", base.LOCAL_PATHS["approved_run"] / "target_stats.json"),
            ("vectorizer", base.LOCAL_PATHS["approved_run"] / "visual_label_vectorizer.json"),
            ("training_config", base.LOCAL_PATHS["approved_run"] / "training_config.json"),
            ("run_metadata", base.LOCAL_PATHS["approved_run"] / "run_metadata.json"),
            ("old_train_rows", base.LOCAL_PATHS["old_splits"] / "train.jsonl"),
            ("old_train_labels", base.LOCAL_PATHS["old_splits"] / "train_visual_labels.jsonl"),
            ("v3r1_train_rows", base.LOCAL_PATHS["v3r1_splits"] / "train.jsonl"),
            ("v3r1_train_labels", base.LOCAL_PATHS["v3r1_splits"] / "train_visual_labels.jsonl"),
            ("v3r1_validation_rows", base.LOCAL_PATHS["v3r1_splits"] / "validation.jsonl"),
            ("v3r1_validation_labels", base.LOCAL_PATHS["v3r1_splits"] / "validation_visual_labels.jsonl"),
            ("canary_rows", base.LOCAL_PATHS["canary_root"] / "finalized/canary.jsonl"),
            ("canary_labels", base.LOCAL_PATHS["canary_root"] / "finalized/canary_visual_labels.jsonl"),
            ("v3r1_package_manifest", base.LOCAL_PATHS["v3r1_splits"] / "package_manifest.json"),
            ("v3r1_final_audit", base.LOCAL_PATHS["guard_root"] / "dataset_v3r1_audit.json"),
            ("v3r1_dataset_manifest", base.LOCAL_PATHS["guard_root"] / "dataset_manifest.json"),
        ):
            transfers.append({"name": name, "local_path": str(path), "bytes": path.stat().st_size, "sha256": base.sha(path)})
        write_json(staging / "data_transfer_manifest.json", {
            "schema_version": "room315.experiment_a.corrected.data_transfer.v1",
            "files": transfers,
            "legacy_evaluation_data_included": False,
        })

        verify_script = '''#!/usr/bin/env python3
import subprocess,sys
from pathlib import Path
root=Path(__file__).resolve().parent
raise SystemExit(subprocess.run([
    sys.executable,str(root/"scripts/experiment_a_verify.py"),
    "--config",str(root/"config/full_training.json"),
    "--package-root",str(root),"--decode-images","--verify-checkpoint-load",
]).returncode)
'''
        write(staging / "verify_package.py", verify_script, executable=True)

        environment = {
            "ROOM315_EXPERIMENT_A_PACKAGE_ROOT": str(staging),
            "ROOM315_APPROVED_RUN_ROOT": str(base.LOCAL_PATHS["approved_run"]),
            "ROOM315_OLD_SPLITS_ROOT": str(base.LOCAL_PATHS["old_splits"]),
            "ROOM315_OLD_DATASET_ROOT": str(base.LOCAL_PATHS["old_images"]),
            "ROOM315_V3R1_SPLITS_ROOT": str(base.LOCAL_PATHS["v3r1_splits"]),
            "ROOM315_V3R1_DATASET_ROOT": str(base.LOCAL_PATHS["v3r1_images"]),
            "ROOM315_V3R1_CANARY_ROOT": str(base.LOCAL_PATHS["canary_root"]),
            "ROOM315_V3R1_GUARD_ROOT": str(base.LOCAL_PATHS["guard_root"]),
        }
        old_environment = {key: os.environ.get(key) for key in environment}
        os.environ.update(environment)
        sys.path.insert(0, str(staging / "scripts"))
        verifier = load_module("corrected_package_verifier", staging / "scripts/experiment_a_verify.py")
        audit = verifier.static_audit(staging / "config/full_training.json", staging, decode_images=True)
        audit["kairos_smoke"] = verifier.static_audit(staging / "config/smoke_training.json", staging, decode_images=False)
        audit["canary_configuration"] = verifier.static_audit(staging / "config/canary_evaluation.json", staging, decode_images=False)
        audit["local_full"] = verifier.static_audit(staging / "config/local_full_training.json", staging, decode_images=False)
        audit["local_canary"] = verifier.static_audit(staging / "config/local_canary_evaluation.json", staging, decode_images=False)
        scan = scan_tree(staging)
        completed = subprocess.run(
            [str(LOCAL_TORCH_PYTHON), str(staging / "scripts/experiment_a_verify.py"),
             "--config", str(staging / "config/full_training.json"),
             "--package-root", str(staging), "--verify-checkpoint-load"],
            check=True,
            text=True,
            capture_output=True,
            env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
        )
        strict_report = json.loads(completed.stdout)["strict_checkpoint_load"]
        for key, value in old_environment.items():
            if value is None: os.environ.pop(key, None)
            else: os.environ[key] = value
        frozen_after = frozen_integrity(base)
        audit.update({
            "schema_version": "room315.experiment_a.corrected.package_audit.v1",
            "corrected_categorical_encoder_packaged": True,
            "strict_checkpoint_load": strict_report,
            "stale_encoder_scan": scan,
            "frozen_artifacts_before": frozen_before,
            "frozen_artifacts_after": frozen_after,
            "full_training_executed": False,
            "canary_evaluation_executed": False,
            "legacy_test_accessed": False,
            "local_and_kairos_outputs_isolated": True,
        })
        write_json(staging / "corrected_package_audit.json", audit)
        write_json(staging / "frozen_artifact_integrity.json", frozen_after)
        write_json(staging / "package_manifest.json", base.manifest(staging))
        checksums = [
            f'{item["sha256"]}  {item["path"]}'
            for item in base.package_files(staging, exclude_manifests=False)
        ]
        write(staging / "SHA256SUMS", "\n".join(checksums) + "\n")
        os.replace(staging, output)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise

    archive_sha = base.deterministic_archive(output, archive)
    archive_scan = scan_archive(archive)
    write(
        archive.with_suffix(archive.suffix + ".sha256"),
        f"{archive_sha}  {archive.name}\n",
    )
    frozen_integrity(base)
    manifest = json.loads((output / "package_manifest.json").read_text())
    return {
        "package_root": str(output),
        "archive": str(archive),
        "archive_sha256": archive_sha,
        "package_tree_sha256": manifest["tree_sha256"],
        "archive_stale_encoder_scan": archive_scan,
        "audit_passed": True,
        "full_training_executed": False,
        "canary_evaluation_executed": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    args = parser.parse_args()
    print(json.dumps(build(args.repo.resolve(), args.output.resolve(), args.archive.resolve()), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
