#!/usr/bin/env python3
"""Build the data-light, guarded Room 315 Experiment-A Kairos package."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
import random
import shutil
import stat
import sys
import tarfile
import tempfile
from pathlib import Path
from typing import Any, Callable


SEED = 31520260730
PACKAGE_NAME = f"room315_kairos_visual_state_experiment_a_v3r1_package_seed{SEED}"
DEFAULT_OUTPUT = Path("/home/tiago") / PACKAGE_NAME
DEFAULT_ARCHIVE = Path("/home/tiago") / f"{PACKAGE_NAME}.tar.gz"
APPROVED_SHA = "8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d"
OLDER_PILOT_SHA = "61acabfeb75ca29e4612e51ccdcf233723d9b22c3600f396d9a5cf50c8487f73"
IDENTITIES = ("L1", "L2", "L3", "L4", "R1", "R2", "R3", "R4")

LOCAL_PATHS = {
    "approved_run": Path("/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run"),
    "old_splits": Path("/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730"),
    "old_images": Path("/home/tiago/room315_arbitrary_subset_visual_2040_capture_v2_seed31520260729/dataset"),
    "v3r1_splits": Path("/home/tiago/room315_hard_case_visual_v3r1_splits_seed31520260730"),
    "v3r1_images": Path("/home/tiago/room315_hard_case_visual_v3r1_capture_seed31520260730/dataset"),
    "canary_root": Path("/home/tiago/room315_hard_case_visual_v3r1_canary_seed31520260730"),
    "guard_root": Path("/home/tiago/room315_hard_case_visual_v3r1_guard_seed31520260730"),
}

HASHES = {
    "approved_checkpoint": APPROVED_SHA,
    "target_stats": "2d48078641842aa2db7a59b9285fc5bbedaaa3a0039fc39986ca230db983b18c",
    "vectorizer": "637c854556f3331c4e187db4aa7fc70457f01df8877947b9a0e988a543f7113e",
    "training_config": "5c45544af7766afff397dafa7c14c0b3b05083f07a93122308ef50c2e8f452eb",
    "run_metadata": "d86c0ebfda3f5b174fc3c06f4ce8a3e083d2048db7b44d20efe951aaa7e5428d",
    "old_rows": "beb6618c5c0bee80e7ec78fa7782e6a2b75c4aabf46e5745a97d6e3871a59095",
    "old_labels": "0cebc68d99db5e364d0637336244456be05b96edad5f8f176eb0176c7883e583",
    "new_rows": "396e3b83822dcd2ed541025fc033802592a609e288dbb28be555c6d9f586361c",
    "new_labels": "ec98fd5a94ed9d29fbb0b33dbed33877d571d263ea4a497be99d088673b71921",
    "validation_rows": "a4c90ac7c1043450830f69ad90094e9aacac92ad57f24fcd4439b0b2a14c9fd7",
    "validation_labels": "d62310046e9a6737e69d7d0e702f05e1073ae7de8f12b2c64655d510b410e1ab",
    "canary_rows": "28568e8ebf793e0a0a18ad9327f36639b2fd9c27021b9bccc4b318dd48192541",
    "canary_labels": "42d1d6ccab49d4a6bfdb2c2b79d77e404e5f9cb23066a816c1a3851d552b02db",
    "v3r1_package_manifest": "c4b93e90ecc25bf71dfaaaa42895cb24ebcd0b3ef5a8af211d37bbde623c058a",
    "v3r1_final_audit": "b90ea95e23f34f87a146ab3061bfaa9150e26c9c98858a5e79c13d9fb7f706a3",
    "v3r1_dataset_manifest": "c78502901e8a213af4137fac3ff1d784cae7172c7030dd21eaa838bc822dfe1c",
}


class PackageError(RuntimeError):
    pass


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write(path: Path, text: str, *, executable: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    if executable: path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP)


def write_json(path: Path, value: Any) -> None:
    write(path, json.dumps(value, indent=2, sort_keys=True) + "\n")


def jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if line.strip(): rows.append(json.loads(line))
    return rows


def sample_id(row: dict[str, Any]) -> str:
    return str(row.get("sample_id") or row.get("episode_id"))


def hard_flags(row: dict[str, Any]) -> set[str]:
    trace = row.get("traceability_metadata") or {}
    active = set(trace.get("active_identities") or [])
    loaded = set(trace.get("loaded_identities") or [])
    flags = set()
    if "L4" in loaded: flags.add("L4_loaded")
    if "R4" in loaded: flags.add("R4_loaded")
    if active == {"L2", "L4", "R4"}: flags.add("exact_L2_L4_R4")
    if trace.get("operational_target_name") == "right_slot_3" and trace.get("target_offset") is not None: flags.add("right_slot3_deliberate_offset")
    if loaded & {"L4", "R4"}: flags.add("hard_payload")
    return flags


def choose_smoke(rows: list[dict[str, Any]], count: int, *, required: tuple[str, ...], seed: int) -> tuple[list[str], dict[str, int]]:
    chosen: list[dict[str, Any]] = []
    seen = set()
    for requirement in required:
        if any(requirement in hard_flags(row) for row in chosen):
            continue
        candidate = next((row for row in rows if requirement in hard_flags(row) and sample_id(row) not in seen), None)
        if candidate is None: raise PackageError(f"no smoke candidate for {requirement}")
        chosen.append(candidate); seen.add(sample_id(candidate))
    indexes = list(range(len(rows))); random.Random(seed).shuffle(indexes)
    for index in indexes:
        row = rows[index]
        if sample_id(row) not in seen:
            chosen.append(row); seen.add(sample_id(row))
        if len(chosen) == count: break
    coverage = {name: sum(name in hard_flags(row) for row in chosen) for name in required}
    return [sample_id(row) for row in chosen], coverage


def source_spec(rows: str, labels: str, root: str, count: int, rows_hash: str, labels_hash: str) -> dict[str, Any]:
    return {
        "rows": rows, "labels": labels, "dataset_root": root,
        "expected_scenarios": count,
        "rows_artifact": {"path": rows, "sha256": rows_hash},
        "labels_artifact": {"path": labels, "sha256": labels_hash},
    }


def base_config(stage: str, package_root_expr: str) -> dict[str, Any]:
    return {
        "schema_version": "room315.experiment_a.training_config.v1",
        "experiment": "Experiment-A data-only corrected continuation baseline",
        "stage": stage,
        "seed": SEED,
        "python_hash_seed": 1455489658,
        "cublas_workspace_config": ":4096:8",
        "artifacts": {
            "approved_checkpoint": {"path": "${ROOM315_APPROVED_RUN_ROOT}/best.pt", "sha256": HASHES["approved_checkpoint"]},
            "target_stats": {"path": "${ROOM315_APPROVED_RUN_ROOT}/target_stats.json", "sha256": HASHES["target_stats"]},
            "vectorizer": {"path": "${ROOM315_APPROVED_RUN_ROOT}/visual_label_vectorizer.json", "sha256": HASHES["vectorizer"]},
            "training_config": {"path": "${ROOM315_APPROVED_RUN_ROOT}/training_config.json", "sha256": HASHES["training_config"]},
            "run_metadata": {"path": "${ROOM315_APPROVED_RUN_ROOT}/run_metadata.json", "sha256": HASHES["run_metadata"]},
            "v3r1_package_manifest": {"path": "${ROOM315_V3R1_SPLITS_ROOT}/package_manifest.json", "sha256": HASHES["v3r1_package_manifest"]},
            "v3r1_final_audit": {"path": "${ROOM315_V3R1_GUARD_ROOT}/dataset_v3r1_audit.json", "sha256": HASHES["v3r1_final_audit"]},
            "v3r1_dataset_manifest": {"path": "${ROOM315_V3R1_GUARD_ROOT}/dataset_manifest.json", "sha256": HASHES["v3r1_dataset_manifest"]},
        },
        "data": {
            "old_replay": source_spec("${ROOM315_OLD_SPLITS_ROOT}/train.jsonl", "${ROOM315_OLD_SPLITS_ROOT}/train_visual_labels.jsonl", "${ROOM315_OLD_DATASET_ROOT}", 1528, HASHES["old_rows"], HASHES["old_labels"]),
            "v3r1_train": source_spec("${ROOM315_V3R1_SPLITS_ROOT}/train.jsonl", "${ROOM315_V3R1_SPLITS_ROOT}/train_visual_labels.jsonl", "${ROOM315_V3R1_DATASET_ROOT}", 4000, HASHES["new_rows"], HASHES["new_labels"]),
            "v3r1_validation": source_spec("${ROOM315_V3R1_SPLITS_ROOT}/validation.jsonl", "${ROOM315_V3R1_SPLITS_ROOT}/validation_visual_labels.jsonl", "${ROOM315_V3R1_DATASET_ROOT}", 512, HASHES["validation_rows"], HASHES["validation_labels"]),
            "v3r1_canary": source_spec("${ROOM315_V3R1_CANARY_ROOT}/finalized/canary.jsonl", "${ROOM315_V3R1_CANARY_ROOT}/finalized/canary_visual_labels.jsonl", "${ROOM315_V3R1_CANARY_ROOT}/dataset", 256, HASHES["canary_rows"], HASHES["canary_labels"]),
        },
        "data_roles": {"training_sources": ["old_replay", "v3r1_hard_case"], "checkpoint_selection": "validation_only", "canary": "post_training_development_regression_only", "final_evaluation": "not_present_and_not_authorized"},
        "model": {"architecture": "paired shared TorchVision ResNet-18", "input_shape": ["B", 6, 224, 224], "initialization_history": "ResNet18_Weights.IMAGENET1K_V1 then approved epoch-14 continuation checkpoint", "adaptation": "partial_finetune", "trainable_scope": ["backbone.layer4", "head"], "output_dimension": 200, "identity_order": list(IDENTITIES), "visual_schema": "room315.visual_state.v3", "strict_checkpoint_loading": True, "augmentations": []},
        "loss": {"kind": "masked_per_sample_smooth_l1_equal_head_weight_v1", "heads": ["segment_location", "loaded_state", "bbox", "s_m", "s_ratio"], "weights": {name: 1.0 for name in ("segment_location", "loaded_state", "bbox", "s_m", "s_ratio")}},
        "optimizer": {"kind": "AdamW", "learning_rate": 0.0001, "weight_decay": 0.0001},
        "training": {"batch_size": 32, "maximum_continuation_epochs": 2 if stage == "smoke" else 10, "early_stopping_patience": 3, "device": "cuda", "source_balance": {"old_replay": 0.5, "v3r1_hard_case": 0.5}, "checkpoint_selection": "V3R1 validation total weighted loss only"},
        "smoke_selection": f"{package_root_expr}/config/smoke_selection.json",
    }


def local_config(stage: str, package_root_expr: str) -> dict[str, Any]:
    config = base_config(stage, package_root_expr)
    config["schema_version"] = "room315.experiment_a.local_training_config.v1"
    config["execution_environment"] = "local_nvidia_rtx"
    config["verification_sources"] = [
        "old_replay", "v3r1_train", "v3r1_validation"
    ]
    config["automatic_fallback"] = False
    config["local_runtime"] = {
        "expected_gpu_name": "NVIDIA GeForce RTX 3080 Laptop GPU",
        "expected_total_vram_mib": 16384,
        "expected_versions": {
            "driver": "580.95.05",
            "torch": "2.10.0+cu128",
            "torchvision": "0.25.0+cu128",
            "cuda": "12.8",
        },
        "requires": ["cuda", "nvidia_rtx", "automatic_mixed_precision"],
        "forbids": ["aarch64_requirement", "gh200_claim", "apptainer_requirement"],
    }
    config["training"].update({
        "batch_size": 32,
        "gradient_accumulation_steps": 1,
        "automatic_mixed_precision": True,
        "num_workers": 4,
        "pin_memory": True,
        "persistent_workers": True,
    })
    config["execution_profiles"] = {
        "default_batch32_amp": {
            "name": "default_batch32_amp",
            "batch_size": 32,
            "gradient_accumulation_steps": 1,
            "automatic_mixed_precision": True,
            "num_workers": 4,
            "pin_memory": True,
            "persistent_workers": True,
            "automatic_selection": False,
        },
        "fallback_batch16_accum2": {
            "name": "fallback_batch16_accum2",
            "batch_size": 16,
            "gradient_accumulation_steps": 2,
            "automatic_mixed_precision": True,
            "num_workers": 4,
            "pin_memory": True,
            "persistent_workers": True,
            "automatic_selection": False,
            "explicit_cli_flag": "--fallback-batch16-accum2",
        },
    }
    config["local_isolation"] = {
        "output_root": "/home/tiago/room315_experiment_a_local_outputs",
        "guard_state": "/home/tiago/room315_experiment_a_local_guard_state.json",
        "preflight_reports": "/home/tiago/room315_experiment_a_local_preflight_reports",
        "kairos_output_root_must_not_be_used": True,
    }
    return config


def launcher(stage: str) -> str:
    config = "smoke_training.json" if stage == "smoke" else "full_training.json"
    mode = "canary" if stage == "canary" else "train"
    checkpoint_arg = ' --checkpoint "$ROOM315_EXPERIMENT_A_BEST_CHECKPOINT"' if stage == "canary" else ""
    return f'''#!/usr/bin/env bash
set -Eeuo pipefail
PACKAGE_ROOT="$(cd -- "$(dirname -- "${{BASH_SOURCE[0]}}")" && pwd)"
CONTAINER=/work/conteneurs/shared/AI/nemo_25.04.03_arm.sif
: "${{ROOM315_EXPERIMENT_A_GUARD_STATE:?set ROOM315_EXPERIMENT_A_GUARD_STATE}}"
: "${{ROOM315_EXPERIMENT_A_OUTPUT_ROOT:?set ROOM315_EXPERIMENT_A_OUTPUT_ROOT}}"
[[ -f "$CONTAINER" ]] || {{ echo "missing container: $CONTAINER" >&2; exit 1; }}
command -v apptainer >/dev/null || {{ echo "apptainer is unavailable" >&2; exit 1; }}
export ROOM315_EXPERIMENT_A_PACKAGE_ROOT="$PACKAGE_ROOT"
OUTPUT="$ROOM315_EXPERIMENT_A_OUTPUT_ROOT/{stage}_seed{SEED}_attempt1"
python3 "$PACKAGE_ROOT/scripts/experiment_a_guard.py" begin --guard "$ROOM315_EXPERIMENT_A_GUARD_STATE" --stage {stage} --output "$OUTPUT"
finish() {{ python3 "$PACKAGE_ROOT/scripts/experiment_a_guard.py" "$1" --guard "$ROOM315_EXPERIMENT_A_GUARD_STATE" --stage {stage}; }}
trap 'finish fail' ERR
apptainer exec --nv \
  --bind "$PACKAGE_ROOT:$PACKAGE_ROOT" \
  "$CONTAINER" python3 "$PACKAGE_ROOT/scripts/experiment_a_inside_container.py" \
  --package-root "$PACKAGE_ROOT" --config "$PACKAGE_ROOT/config/{config}" \
  --mode {mode} --output "$OUTPUT"{checkpoint_arg}
trap - ERR
finish complete
'''


def inside_container() -> str:
    return '''#!/usr/bin/env python3
import argparse, os, subprocess, sys
from pathlib import Path

def main():
    p=argparse.ArgumentParser(); p.add_argument("--package-root",type=Path,required=True); p.add_argument("--config",type=Path,required=True); p.add_argument("--mode",choices=("train","canary"),required=True); p.add_argument("--output",type=Path,required=True); p.add_argument("--checkpoint",type=Path); a=p.parse_args()
    os.environ["PYTHONHASHSEED"]="1455489658"; os.environ["CUBLAS_WORKSPACE_CONFIG"]=":4096:8"
    verify=[sys.executable,str(a.package_root/"scripts/experiment_a_verify.py"),"--config",str(a.config),"--package-root",str(a.package_root),"--require-gh200","--decode-images"]
    subprocess.run(verify,check=True)
    command=[sys.executable,str(a.package_root/"scripts/experiment_a_train.py"),"--config",str(a.config),"--mode",a.mode,"--output",str(a.output)]
    if a.checkpoint: command += ["--checkpoint",str(a.checkpoint)]
    subprocess.run(command,check=True)
if __name__=="__main__": main()
'''


def local_launcher(stage: str) -> str:
    if stage not in {"smoke", "full", "canary"}:
        raise ValueError(stage)
    config = "local_smoke_training.json" if stage == "smoke" else "local_full_training.json"
    mode = "canary" if stage == "canary" else "train"
    checkpoint_setup = (
        ': "${ROOM315_EXPERIMENT_A_LOCAL_BEST_CHECKPOINT:?set the local best checkpoint for Canary}"\n'
        if stage == "canary" else ""
    )
    checkpoint_arg = (
        ' --checkpoint "$ROOM315_EXPERIMENT_A_LOCAL_BEST_CHECKPOINT"'
        if stage == "canary" else ""
    )
    fallback_parsing = ""
    if stage != "canary":
        fallback_parsing = '''
PROFILE=default_batch32_amp
if [[ "${1:-}" == "--fallback-batch16-accum2" ]]; then
  PROFILE=fallback_batch16_accum2
  shift
fi
[[ "$#" -eq 0 ]] || { echo "usage: $0 [--fallback-batch16-accum2]" >&2; exit 64; }
'''
    else:
        fallback_parsing = '''
PROFILE=default_batch32_amp
[[ "$#" -eq 0 ]] || { echo "usage: $0" >&2; exit 64; }
'''
    fallback_command = (
        '"$PACKAGE_ROOT/run_local_experiment_a_' + stage + '.sh --fallback-batch16-accum2"'
        if stage != "canary"
        else '"not_applicable_to_canary"'
    )
    return f'''#!/usr/bin/env bash
set -Eeuo pipefail
PACKAGE_ROOT="$(cd -- "$(dirname -- "${{BASH_SOURCE[0]}}")" && pwd)"
LOCAL_PYTHON="${{ROOM315_LOCAL_PYTHON:-/home/tiago/room315_local_training/venv/bin/python}}"
LOCAL_OUTPUT_ROOT="${{ROOM315_EXPERIMENT_A_LOCAL_OUTPUT_ROOT:-/home/tiago/room315_experiment_a_local_outputs}}"
LOCAL_GUARD="${{ROOM315_EXPERIMENT_A_LOCAL_GUARD_STATE:-/home/tiago/room315_experiment_a_local_guard_state.json}}"
LOCAL_PREFLIGHT_ROOT="${{ROOM315_EXPERIMENT_A_LOCAL_PREFLIGHT_ROOT:-/home/tiago/room315_experiment_a_local_preflight_reports}}"
[[ -x "$LOCAL_PYTHON" ]] || {{ echo "local Torch Python is unavailable: $LOCAL_PYTHON" >&2; exit 1; }}
[[ -f "$LOCAL_GUARD" ]] || {{
  echo "local guard is not initialized; run:" >&2
  echo "cp '$PACKAGE_ROOT/config/local_training_guard_template.json' '$LOCAL_GUARD'" >&2
  exit 1
}}
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
{checkpoint_setup}{fallback_parsing}
CONFIG="$PACKAGE_ROOT/config/{config}"
OUTPUT="$LOCAL_OUTPUT_ROOT/{stage}_seed{SEED}_attempt1"
PREFLIGHT_REPORT="$LOCAL_PREFLIGHT_ROOT/{stage}_${{PROFILE}}_seed{SEED}.json"
FALLBACK_COMMAND={fallback_command}
"$LOCAL_PYTHON" "$PACKAGE_ROOT/scripts/experiment_a_local.py" \
  --config "$CONFIG" --package-root "$PACKAGE_ROOT" \
  --output "$OUTPUT" --report "$PREFLIGHT_REPORT" \
  --execution-profile "$PROFILE" --fallback-command "$FALLBACK_COMMAND"
"$LOCAL_PYTHON" "$PACKAGE_ROOT/scripts/experiment_a_guard.py" begin \
  --guard "$LOCAL_GUARD" --stage {stage} --output "$OUTPUT"
finish() {{ "$LOCAL_PYTHON" "$PACKAGE_ROOT/scripts/experiment_a_guard.py" "$1" --guard "$LOCAL_GUARD" --stage {stage}; }}
trap 'finish fail' ERR
"$LOCAL_PYTHON" "$PACKAGE_ROOT/scripts/experiment_a_train.py" \
  --config "$CONFIG" --mode {mode} --output "$OUTPUT" \
  --execution-profile "$PROFILE"{checkpoint_arg}
trap - ERR
finish complete
'''


def readme() -> str:
    return f'''# Room 315 Experiment A: V3R1 data-only continuation

This package continues the approved epoch-14 paired-camera ResNet-18 model. It
does not fit a vectorizer or target normalization. Every epoch deterministically
selects 4,000 old-replay references (controlled replay cycles, maximum expected
multiplicity 3) and 4,000 V3R1 references, then applies an epoch-specific seeded
shuffle. Validation alone selects checkpoints. Canary is a separately authorized
post-training development regression; no final-evaluation split or evaluator is
present.

The preparation step did not run training. The package contains no checkpoint,
images, or JSONL datasets.

## Transfer

Transfer the package/archive and the allowed inputs listed in
`data_transfer_manifest.json`. On Kairos set the seven roots shown below to the
transferred locations. The consumed legacy evaluation split is neither needed
nor authorized.

Create a separate input bundle containing only the authorized rows and their
referenced images (this intentionally does not copy the full old split root):

```bash
export ROOM315_APPROVED_RUN_ROOT=/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run
export ROOM315_OLD_SPLITS_ROOT=/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730
export ROOM315_OLD_DATASET_ROOT=/home/tiago/room315_arbitrary_subset_visual_2040_capture_v2_seed31520260729/dataset
export ROOM315_V3R1_SPLITS_ROOT=/home/tiago/room315_hard_case_visual_v3r1_splits_seed31520260730
export ROOM315_V3R1_DATASET_ROOT=/home/tiago/room315_hard_case_visual_v3r1_capture_seed31520260730/dataset
export ROOM315_V3R1_CANARY_ROOT=/home/tiago/room315_hard_case_visual_v3r1_canary_seed31520260730
export ROOM315_V3R1_GUARD_ROOT=/home/tiago/room315_hard_case_visual_v3r1_guard_seed31520260730
python3 /home/tiago/{PACKAGE_NAME}/scripts/experiment_a_prepare_inputs.py \
  --config /home/tiago/{PACKAGE_NAME}/config/full_training.json \
  --output /home/tiago/room315_experiment_a_inputs_seed{SEED}
```

Transfer through the active CALMIP VPN tunnel:

```bash
rsync -avh --partial --info=progress2 -e 'ssh -p 11220' \
  /home/tiago/{PACKAGE_NAME}.tar.gz \
  /home/tiago/room315_experiment_a_inputs_seed{SEED}/ \
  p26065brhm@127.0.0.1:~/
```

On Kairos:

```bash
cd "$HOME"
tar -xzf {PACKAGE_NAME}.tar.gz
```

## Environment

```bash
export ROOM315_EXPERIMENT_A_PACKAGE_ROOT="$HOME/{PACKAGE_NAME}"
export ROOM315_APPROVED_RUN_ROOT="$HOME/room315_experiment_a_inputs_seed{SEED}/approved_run"
export ROOM315_OLD_SPLITS_ROOT="$HOME/room315_experiment_a_inputs_seed{SEED}/old_splits"
export ROOM315_OLD_DATASET_ROOT="$HOME/room315_experiment_a_inputs_seed{SEED}/old_dataset"
export ROOM315_V3R1_SPLITS_ROOT="$HOME/room315_experiment_a_inputs_seed{SEED}/v3r1_splits"
export ROOM315_V3R1_DATASET_ROOT="$HOME/room315_experiment_a_inputs_seed{SEED}/v3r1_dataset"
export ROOM315_V3R1_CANARY_ROOT="$HOME/room315_experiment_a_inputs_seed{SEED}/v3r1_canary"
export ROOM315_V3R1_GUARD_ROOT="$HOME/room315_experiment_a_inputs_seed{SEED}/v3r1_guard"
export ROOM315_EXPERIMENT_A_OUTPUT_ROOT="$HOME/room315_experiment_a_outputs"
export ROOM315_EXPERIMENT_A_GUARD_STATE="$HOME/room315_experiment_a_guard_state.json"
mkdir -p "$ROOM315_EXPERIMENT_A_OUTPUT_ROOT"
cp "$ROOM315_EXPERIMENT_A_PACKAGE_ROOT/config/training_guard_template.json" "$ROOM315_EXPERIMENT_A_GUARD_STATE"
```

## Static verification

```bash
python3 "$ROOM315_EXPERIMENT_A_PACKAGE_ROOT/verify_package.py"
```

## Future smoke

```bash
python3 "$ROOM315_EXPERIMENT_A_PACKAGE_ROOT/scripts/experiment_a_guard.py" authorize --guard "$ROOM315_EXPERIMENT_A_GUARD_STATE" --stage smoke
"$ROOM315_EXPERIMENT_A_PACKAGE_ROOT/run_kairos_experiment_a_smoke.sh"
```

## Future full run (authorize only after reviewing smoke)

```bash
python3 "$ROOM315_EXPERIMENT_A_PACKAGE_ROOT/scripts/experiment_a_guard.py" authorize --guard "$ROOM315_EXPERIMENT_A_GUARD_STATE" --stage full
"$ROOM315_EXPERIMENT_A_PACKAGE_ROOT/run_kairos_experiment_a_full.sh"
```

## Future Canary regression (after the frozen best checkpoint exists)

```bash
export ROOM315_EXPERIMENT_A_BEST_CHECKPOINT="$ROOM315_EXPERIMENT_A_OUTPUT_ROOT/full_seed{SEED}_attempt1/best.pt"
python3 "$ROOM315_EXPERIMENT_A_PACKAGE_ROOT/scripts/experiment_a_guard.py" authorize --guard "$ROOM315_EXPERIMENT_A_GUARD_STATE" --stage canary
"$ROOM315_EXPERIMENT_A_PACKAGE_ROOT/run_kairos_experiment_a_canary.sh"
```

## Local NVIDIA RTX profile

The local profile is isolated from Kairos and defaults to:

- Python: `/home/tiago/room315_local_training/venv/bin/python`
- GPU: NVIDIA GeForce RTX 3080 Laptop GPU
- outputs: `/home/tiago/room315_experiment_a_local_outputs`
- guard: `/home/tiago/room315_experiment_a_local_guard_state.json`
- batch 32, AMP enabled, four workers, pinned persistent workers

Initialize and authorize only the local Smoke stage:

```bash
cp "$ROOM315_EXPERIMENT_A_PACKAGE_ROOT/config/local_training_guard_template.json" \
  /home/tiago/room315_experiment_a_local_guard_state.json
/home/tiago/room315_local_training/venv/bin/python \
  "$ROOM315_EXPERIMENT_A_PACKAGE_ROOT/scripts/experiment_a_guard.py" authorize \
  --guard /home/tiago/room315_experiment_a_local_guard_state.json --stage smoke
```

Run local Smoke:

```bash
"$ROOM315_EXPERIMENT_A_PACKAGE_ROOT/run_local_experiment_a_smoke.sh"
```

The launcher performs a one-batch forward/backward memory preflight before it
consumes the guard authorization or creates the training output. If batch 32
runs out of memory, it stops with exit code 42 and prints this explicit command:

```bash
"$ROOM315_EXPERIMENT_A_PACKAGE_ROOT/run_local_experiment_a_smoke.sh" \
  --fallback-batch16-accum2
```

It never starts that fallback automatically. The local Full and Canary stages
remain unauthorized until separately approved. Their launchers are
`run_local_experiment_a_full.sh` and `run_local_experiment_a_canary.sh`.

The Kairos launchers require aarch64, CUDA, a GPU name containing GH200, Torch,
TorchVision, all allowed inputs and hashes, all 12,592 image references,
and an unused immutable output path. The full guard permits exactly one attempt
and does not automatically restart a completed or failed run.
'''


def package_files(root: Path, *, exclude_manifests: bool = True) -> list[dict[str, Any]]:
    excluded = {"package_manifest.json", "SHA256SUMS"} if exclude_manifests else {"SHA256SUMS"}
    result=[]
    for path in sorted(root.rglob("*")):
        if path.is_file() and path.relative_to(root).as_posix() not in excluded and "__pycache__" not in path.parts and path.suffix != ".pyc":
            result.append({"path": path.relative_to(root).as_posix(), "bytes": path.stat().st_size, "sha256": sha(path)})
    return result


def manifest(root: Path) -> dict[str, Any]:
    files=package_files(root); digest=hashlib.sha256()
    for item in files: digest.update(f'{item["path"]}\0{item["bytes"]}\0{item["sha256"]}\n'.encode())
    return {"schema_version":"room315.experiment_a.package_manifest.v1","approved_checkpoint_sha256":APPROVED_SHA,"immutable_package_payload":True,"file_count":len(files),"total_bytes":sum(x["bytes"] for x in files),"tree_sha256":digest.hexdigest(),"files":files}


def deterministic_archive(root: Path, archive: Path) -> str:
    with archive.open("wb") as raw:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode="w", format=tarfile.PAX_FORMAT) as tar:
                for path in sorted([root, *root.rglob("*")], key=lambda p: p.relative_to(root.parent).as_posix()):
                    if "__pycache__" in path.parts or path.suffix == ".pyc": continue
                    info=tar.gettarinfo(str(path), arcname=path.relative_to(root.parent).as_posix()); info.uid=0; info.gid=0; info.uname=""; info.gname=""; info.mtime=0
                    if path.is_file():
                        with path.open("rb") as stream: tar.addfile(info, stream)
                    else: tar.addfile(info)
    return sha(archive)


def build(repo_root: Path, output: Path, archive: Path) -> dict[str, Any]:
    if output.exists(): raise PackageError(f"refusing to overwrite package: {output}")
    if archive.exists(): raise PackageError(f"refusing to overwrite archive: {archive}")
    template_root=repo_root/"mfja_robot_control_config/experiment_a_v3r1"; model=repo_root/"mfja_robot_control_config/scripts/room_315_visual_model.py"
    staging=Path(tempfile.mkdtemp(prefix=f".{output.name}.",dir=output.parent))
    try:
        for source in sorted(template_root.glob("*.py")): shutil.copy2(source,staging/"scripts"/source.name) if (staging/"scripts").mkdir(parents=True,exist_ok=True) is None else None
        shutil.copy2(model,staging/"scripts"/model.name)
        new_rows=jsonl(LOCAL_PATHS["v3r1_splits"]/"train.jsonl"); val_rows=jsonl(LOCAL_PATHS["v3r1_splits"]/"validation.jsonl"); old_rows=jsonl(LOCAL_PATHS["old_splits"]/"train.jsonl")
        old_indexes=list(range(len(old_rows))); random.Random(SEED+101).shuffle(old_indexes); old_ids=[sample_id(old_rows[i]) for i in old_indexes[:64]]
        required=("L4_loaded","R4_loaded","exact_L2_L4_R4","right_slot3_deliberate_offset","hard_payload")
        new_ids,coverage=choose_smoke(new_rows,64,required=required,seed=SEED+202)
        val_indexes=list(range(len(val_rows))); random.Random(SEED+303).shuffle(val_indexes); val_ids=[sample_id(val_rows[i]) for i in val_indexes[:32]]
        write_json(staging/"config/smoke_selection.json",{"seed":SEED,"train":{"old_replay":old_ids,"v3r1_hard_case":new_ids},"validation":val_ids,"coverage":coverage,"canary_used":False})
        write_json(staging/"config/smoke_training.json",base_config("smoke","${ROOM315_EXPERIMENT_A_PACKAGE_ROOT}")); write_json(staging/"config/full_training.json",base_config("full","${ROOM315_EXPERIMENT_A_PACKAGE_ROOT}")); write_json(staging/"config/canary_evaluation.json",base_config("canary","${ROOM315_EXPERIMENT_A_PACKAGE_ROOT}"))
        write_json(staging/"config/local_smoke_training.json",local_config("smoke","${ROOM315_EXPERIMENT_A_PACKAGE_ROOT}"))
        write_json(staging/"config/local_full_training.json",local_config("full","${ROOM315_EXPERIMENT_A_PACKAGE_ROOT}"))
        guard={"schema_version":"room315.experiment_a.training_guard.v1","approved_checkpoint_sha256":APPROVED_SHA,"legacy_evaluation_authorized":False,"stages":{stage:{"state":"unauthorized","attempts":0,"output":None} for stage in ("smoke","full","canary")}}
        write_json(staging/"config/training_guard_template.json",guard)
        local_guard={**guard,"schema_version":"room315.experiment_a.local_training_guard.v1","execution_environment":"local_nvidia_rtx","output_root":"/home/tiago/room315_experiment_a_local_outputs","kairos_outputs_authorized":False}
        write_json(staging/"config/local_training_guard_template.json",local_guard)
        write_json(staging/"checkpoint_hash_resolution.json",{"approved_continuation_checkpoint":{"path":"/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run/best.pt","epoch":14,"sha256":APPROVED_SHA},"incorrectly_reported_value":{"sha256":OLDER_PILOT_SHA,"resolved_artifact":"/home/tiago/Downloads/kairos_room315_h200_pilot_results/best.pt","classification":"older frozen pilot checkpoint; not approved for Experiment A"}})
        transfers=[]
        for name,path in (("approved_checkpoint",LOCAL_PATHS["approved_run"]/"best.pt"),("target_stats",LOCAL_PATHS["approved_run"]/"target_stats.json"),("vectorizer",LOCAL_PATHS["approved_run"]/"visual_label_vectorizer.json"),("training_config",LOCAL_PATHS["approved_run"]/"training_config.json"),("run_metadata",LOCAL_PATHS["approved_run"]/"run_metadata.json"),("old_train_rows",LOCAL_PATHS["old_splits"]/"train.jsonl"),("old_train_labels",LOCAL_PATHS["old_splits"]/"train_visual_labels.jsonl"),("v3r1_train_rows",LOCAL_PATHS["v3r1_splits"]/"train.jsonl"),("v3r1_train_labels",LOCAL_PATHS["v3r1_splits"]/"train_visual_labels.jsonl"),("v3r1_validation_rows",LOCAL_PATHS["v3r1_splits"]/"validation.jsonl"),("v3r1_validation_labels",LOCAL_PATHS["v3r1_splits"]/"validation_visual_labels.jsonl"),("canary_rows",LOCAL_PATHS["canary_root"]/"finalized/canary.jsonl"),("canary_labels",LOCAL_PATHS["canary_root"]/"finalized/canary_visual_labels.jsonl"),("v3r1_package_manifest",LOCAL_PATHS["v3r1_splits"]/"package_manifest.json"),("v3r1_final_audit",LOCAL_PATHS["guard_root"]/"dataset_v3r1_audit.json"),("v3r1_dataset_manifest",LOCAL_PATHS["guard_root"]/"dataset_manifest.json")):
            transfers.append({"name":name,"local_path":str(path),"bytes":path.stat().st_size,"sha256":sha(path)})
        write_json(staging/"data_transfer_manifest.json",{"schema_version":"room315.experiment_a.data_transfer.v1","files":transfers,"image_roots":[{"role":"old_replay_train_only","local_path":str(LOCAL_PATHS["old_images"]),"referenced_images":3056},{"role":"v3r1_train_and_validation","local_path":str(LOCAL_PATHS["v3r1_images"]),"referenced_images":9024},{"role":"v3r1_canary_development_regression","local_path":str(LOCAL_PATHS["canary_root"]/"dataset"),"referenced_images":512}],"legacy_evaluation_data_included":False})
        write(staging/"scripts/experiment_a_inside_container.py",inside_container(),executable=True)
        for stage in ("smoke","full","canary"): write(staging/f"run_kairos_experiment_a_{stage}.sh",launcher(stage),executable=True)
        for stage in ("smoke","full","canary"): write(staging/f"run_local_experiment_a_{stage}.sh",local_launcher(stage),executable=True)
        write(staging/"README.md",readme())
        verify_script='''#!/usr/bin/env python3\nimport os,subprocess,sys\nfrom pathlib import Path\nROOT=Path(__file__).resolve().parent\ncmd=[sys.executable,str(ROOT/"scripts/experiment_a_verify.py"),"--config",str(ROOT/"config/full_training.json"),"--package-root",str(ROOT),"--decode-images"]\nraise SystemExit(subprocess.run(cmd).returncode)\n'''
        write(staging/"verify_package.py",verify_script,executable=True)
        env={"ROOM315_EXPERIMENT_A_PACKAGE_ROOT":str(staging),"ROOM315_APPROVED_RUN_ROOT":str(LOCAL_PATHS["approved_run"]),"ROOM315_OLD_SPLITS_ROOT":str(LOCAL_PATHS["old_splits"]),"ROOM315_OLD_DATASET_ROOT":str(LOCAL_PATHS["old_images"]),"ROOM315_V3R1_SPLITS_ROOT":str(LOCAL_PATHS["v3r1_splits"]),"ROOM315_V3R1_DATASET_ROOT":str(LOCAL_PATHS["v3r1_images"]),"ROOM315_V3R1_CANARY_ROOT":str(LOCAL_PATHS["canary_root"]),"ROOM315_V3R1_GUARD_ROOT":str(LOCAL_PATHS["guard_root"])}
        old_env={key:os.environ.get(key) for key in env}; os.environ.update(env)
        sys.path.insert(0,str(staging/"scripts")); from experiment_a_verify import static_audit
        audit=static_audit(staging/"config/full_training.json",staging,decode_images=True)
        audit["local_profiles"]={
            "smoke":static_audit(staging/"config/local_smoke_training.json",staging,decode_images=False),
            "full":static_audit(staging/"config/local_full_training.json",staging,decode_images=False),
            "training_executed":False,
            "memory_preflight_executed":False,
        }
        for key,value in old_env.items():
            if value is None: os.environ.pop(key,None)
            else: os.environ[key]=value
        write_json(staging/"experiment_a_package_audit.json",audit)
        md=f'''# Experiment-A package audit\n\nResult: **PASS**\n\n- approved checkpoint: `{APPROVED_SHA}` (epoch 14)\n- rows: old replay 1528; V3R1 train 4000; validation 512; Canary 256\n- image references checked and decoded: 12,592\n- schema/dimension: `room315.visual_state.v3` / 200\n- identities: {", ".join(IDENTITIES)}\n- source sampler: deterministic 4,000 + 4,000 per full epoch\n- overlap: zero across all four allowed sources\n- checkpoint selection: Validation total weighted loss only\n- training executed during packaging: no\n- legacy evaluation access: no\n'''
        write(staging/"experiment_a_package_audit.md",md)
        write_json(staging/"package_manifest.json",manifest(staging))
        sums=[]
        for item in package_files(staging,exclude_manifests=False): sums.append(f'{item["sha256"]}  {item["path"]}')
        write(staging/"SHA256SUMS","\n".join(sums)+"\n")
        os.replace(staging,output)
    except BaseException:
        shutil.rmtree(staging,ignore_errors=True); raise
    archive_sha=deterministic_archive(output,archive); write(archive.with_suffix(archive.suffix+".sha256"),f"{archive_sha}  {archive.name}\n")
    return {"package_root":str(output),"archive":str(archive),"archive_sha256":archive_sha,"package_tree_sha256":json.loads((output/"package_manifest.json").read_text())["tree_sha256"],"audit_passed":True}


def main() -> None:
    parser=argparse.ArgumentParser(); parser.add_argument("--repo-root",type=Path,default=Path(__file__).resolve().parents[2]); parser.add_argument("--output",type=Path,default=DEFAULT_OUTPUT); parser.add_argument("--archive",type=Path,default=DEFAULT_ARCHIVE); args=parser.parse_args(); print(json.dumps(build(args.repo_root.resolve(),args.output.resolve(),args.archive.resolve()),indent=2,sort_keys=True))


if __name__=="__main__": main()
