#!/usr/bin/env python3
"""Build and verify the frozen Room 315 Kairos visual-state training package."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import stat
import tempfile
from pathlib import Path
from typing import Any, Iterable


SEED = 31520260730
SCHEMA_VERSION = 'room315.kairos_visual_training_package.v1'
VISUAL_SCHEMA = 'room315.visual_state.v3'
IDENTITIES = ('L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4')
CAMERAS = ('left_rail_rgb', 'right_rail_rgb')
BLOCKS = (
    'A12E', 'A12I', 'A14', 'A1E', 'A1I', 'A23', 'A2E',
    'A2I', 'A34E', 'A34I', 'A3E', 'A3I', 'A4E', 'A4I',
)
SPLIT_COUNTS = {'train': 1528, 'validation': 256, 'test': 256}
CONFIGURATION_COUNTS = {'train': 191, 'validation': 32, 'test': 32}
CHECKPOINT_NAME = 'resnet18-f37072fd.pth'
CHECKPOINT_SHA256 = (
    'f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec'
)
PRETRAINED_IDENTIFIER = 'ResNet18_Weights.IMAGENET1K_V1'
PRETRAINED_SOURCE = 'torchvision:resnet18:IMAGENET1K_V1'
CONTAINER = '/work/conteneurs/shared/AI/nemo_25.04.03_arm.sif'
PROHIBITED = {
    'target_identity',
    'target_zone',
    'relation_family',
    'relation_identities',
    'scenario_id',
    'v2_plan_id',
    'presence_configuration_id',
    'switches',
    'split_name',
}


class TrainingPackageError(ValueError):
    """Raised when package creation or verification must fail closed."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise TrainingPackageError(f'expected JSON object: {path}')
    return value


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise TrainingPackageError(
                    f'{path}:{line_number}: expected an object'
                )
            rows.append(value)
    return rows


def _atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{path.name}.',
        suffix='.tmp',
        dir=path.parent,
    )
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def _atomic_json(path: Path, value: Any) -> None:
    _atomic_text(
        path,
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
    )


def _copy_file(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def _make_executable(path: Path) -> None:
    path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP)


def _manifest_files(root: Path) -> list[dict[str, Any]]:
    result = []
    for path in sorted(root.rglob('*')):
        if not path.is_file():
            continue
        relative = path.relative_to(root).as_posix()
        if (
            relative == 'package_manifest.json'
            or relative.startswith('outputs/')
            or path.suffix == '.pyc'
            or '__pycache__' in path.parts
        ):
            continue
        result.append({
            'path': relative,
            'bytes': path.stat().st_size,
            'sha256': _sha256(path),
        })
    return result


def _manifest(root: Path, source_inputs: dict[str, Any]) -> dict[str, Any]:
    files = _manifest_files(root)
    tree = hashlib.sha256()
    for item in files:
        tree.update(
            f'{item["path"]}\0{item["bytes"]}\0{item["sha256"]}\n'.encode()
        )
    return {
        'schema_version': SCHEMA_VERSION,
        'immutable_payload': True,
        'generated_outputs_excluded': 'outputs/',
        'file_count': len(files),
        'total_bytes': sum(item['bytes'] for item in files),
        'tree_sha256': tree.hexdigest(),
        'files': files,
        'source_inputs': source_inputs,
    }


def _fleet_module(source_metadata: dict[str, Any]) -> str:
    metadata = json.dumps(source_metadata, indent=4, sort_keys=True)
    return f'''#!/usr/bin/env python3
"""Frozen authoritative Room 315 fleet snapshot for this training package."""

from __future__ import annotations

from typing import Any


FIXED_VISUAL_SHUTTLE_IDENTITIES = {IDENTITIES!r}
AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY = {BLOCKS!r}
_SOURCE_METADATA = {metadata}


class VisualFleetError(ValueError):
    pass


def identity_side(identity: str) -> str:
    if identity not in FIXED_VISUAL_SHUTTLE_IDENTITIES:
        raise VisualFleetError(f"unknown fixed identity: {{identity!r}}")
    return "left" if identity.startswith("L") else "right"


def block_vocabulary_metadata() -> dict[str, Any]:
    return {{
        "vocabulary": list(AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY),
        "source": _SOURCE_METADATA,
        "dataset_inferred": False,
        "shared_by_every_fixed_entry": True,
        "frozen_package_snapshot": True,
    }}


AUTHORITATIVE_VISUAL_FLEET = {{
    "fixed_identity_order": list(FIXED_VISUAL_SHUTTLE_IDENTITIES),
    "maximum_shuttles_per_side": 4,
    "maximum_simultaneous_shuttles": 8,
    "dataset_inferred": False,
    "source": _SOURCE_METADATA,
}}
'''


def _smoke_module() -> str:
    return '''#!/usr/bin/env python3
"""Frozen model-output boundary declaration used by the Kairos trainer."""

from typing import Any


def visual_state_plansys2_smoke() -> dict[str, Any]:
    return {
        "passed": True,
        "kind": "frozen_visual_state_output_boundary",
        "model_output": "structured visual facts only",
        "direct_planning_actions": False,
        "direct_rail_commands": False,
        "runtime_integration_smoke_deferred": True,
    }


def load_local_script_module(name: str) -> Any:
    raise RuntimeError(
        f"runtime integration module {name!r} is outside this frozen training package"
    )


def visual_label_to_provider_compact_scene(*args: Any, **kwargs: Any) -> dict[str, Any]:
    raise RuntimeError("runtime state-fusion conversion is outside training")
'''


def _package_checks() -> str:
    return f'''#!/usr/bin/env python3
"""Kairos GH200 preflight for the frozen Room 315 training package."""

import argparse
import json
import platform
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from room_315_kairos_visual_training_package import verify_training_preflight


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--package-root", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--require-gh200", action="store_true")
    args = parser.parse_args()
    report = verify_training_preflight(args.package_root)
    if args.require_gh200:
        if platform.machine() != "aarch64":
            raise SystemExit("FAIL: architecture must be aarch64")
        try:
            import torch
            import torchvision
        except Exception as exc:
            raise SystemExit(f"FAIL: Torch/TorchVision import failed: {{exc}}") from exc
        if not torch.cuda.is_available():
            raise SystemExit("FAIL: CUDA is unavailable")
        gpu_name = torch.cuda.get_device_name(0)
        if "GH200" not in gpu_name.upper():
            raise SystemExit(f"FAIL: expected GH200 GPU, found {{gpu_name}}")
        report["runtime"] = {{
            "architecture": platform.machine(),
            "gpu_name": gpu_name,
            "cuda": torch.version.cuda,
            "torch": torch.__version__,
            "torchvision": torchvision.__version__,
            "nvidia_smi": subprocess.run(
                ["nvidia-smi", "--query-gpu=name,driver_version,memory.total",
                 "--format=csv,noheader"],
                check=True, text=True, capture_output=True,
            ).stdout.strip(),
        }}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\\n")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
'''


def _inside_container(mode: str) -> str:
    if mode not in {'smoke', 'full'}:
        raise ValueError(mode)
    epochs = 2 if mode == 'smoke' else 15
    patience = 0 if mode == 'smoke' else 3
    limits = (
        '  --limit-train-rows 64 \\\n'
        '  --limit-val-rows 16 \\\n'
        if mode == 'smoke'
        else ''
    )
    return f'''#!/usr/bin/env bash
set -Eeuo pipefail

[[ "$#" -eq 2 ]] || {{ echo "usage: $0 PACKAGE_ROOT OUTPUT_DIR" >&2; exit 64; }}
ROOM315_PACKAGE_ROOT="$1"
ROOM315_OUTPUT_DIR="$2"
ROOM315_PYTHON="${{ROOM315_PYTHON:-python3}}"
export TORCH_HOME="$ROOM315_PACKAGE_ROOT/torch_cache"
export PYTHONPATH="$ROOM315_PACKAGE_ROOT/scripts${{PYTHONPATH:+:$PYTHONPATH}}"

echo "mode={mode}"
echo "architecture=$(uname -m)"
echo "python=$($ROOM315_PYTHON --version 2>&1)"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader

"$ROOM315_PYTHON" "$ROOM315_PACKAGE_ROOT/scripts/kairos_package_checks.py" \
  --package-root "$ROOM315_PACKAGE_ROOT" \
  --require-gh200 \
  --output "$ROOM315_OUTPUT_DIR/preflight.json"

"$ROOM315_PYTHON" "$ROOM315_PACKAGE_ROOT/scripts/room_315_visual_state_train_local.py" \
  --splits-dir "$ROOM315_PACKAGE_ROOT/dataset/splits" \
  --dataset-root "$ROOM315_PACKAGE_ROOT/dataset" \
  --output-dir "$ROOM315_OUTPUT_DIR/run" \
  --dataset-mode visual_state \
  --train-file train.jsonl \
  --val-file validation.jsonl \
  --epochs {epochs} \
  --batch-size 32 \
  --early-stopping-patience {patience} \
  --learning-rate 0.001 \
  --weight-decay 0.0001 \
  --image-width 224 \
  --image-height 224 \
  --num-workers 4 \
  --device cuda \
  --seed {SEED} \
{limits}  --visual-adaptation partial_finetune \
  --visual-pretrained-backbone {PRETRAINED_SOURCE}
'''


def _launcher(mode: str) -> str:
    output = (
        f'kairos_gh200_smoke_seed{SEED}'
        if mode == 'smoke'
        else f'kairos_gh200_full_seed{SEED}'
    )
    return f'''#!/usr/bin/env bash
set -Eeuo pipefail

ROOM315_PACKAGE_ROOT="$(cd -- "$(dirname -- "${{BASH_SOURCE[0]}}")" && pwd)"
ROOM315_CONTAINER="{CONTAINER}"
ROOM315_OUTPUT_DIR="$ROOM315_PACKAGE_ROOT/outputs/{output}"
[[ "$(uname -m)" == "aarch64" ]] || {{ echo "FAIL: aarch64 required" >&2; exit 1; }}
command -v apptainer >/dev/null || {{ echo "FAIL: apptainer unavailable" >&2; exit 1; }}
[[ -f "$ROOM315_CONTAINER" ]] || {{ echo "FAIL: missing container $ROOM315_CONTAINER" >&2; exit 1; }}
[[ ! -e "$ROOM315_OUTPUT_DIR" ]] || {{ echo "FAIL: refusing overwrite: $ROOM315_OUTPUT_DIR" >&2; exit 1; }}
(
  cd "$ROOM315_PACKAGE_ROOT/torch_cache/hub/checkpoints"
  sha256sum --check "{CHECKPOINT_NAME}.sha256"
)
mkdir -p "$ROOM315_OUTPUT_DIR"
apptainer exec --nv \
  --bind "$ROOM315_PACKAGE_ROOT:$ROOM315_PACKAGE_ROOT" \
  --env "TORCH_HOME=$ROOM315_PACKAGE_ROOT/torch_cache" \
  "$ROOM315_CONTAINER" \
  "$ROOM315_PACKAGE_ROOT/scripts/run_inside_container_{mode}.sh" \
  "$ROOM315_PACKAGE_ROOT" "$ROOM315_OUTPUT_DIR" \
  2>&1 | tee "$ROOM315_OUTPUT_DIR/kairos_{mode}.log"
'''


def _resume_script() -> str:
    return f'''#!/usr/bin/env bash
set -Eeuo pipefail
[[ "$#" -eq 2 ]] || {{
  echo "usage: $0 CHECKPOINT NEW_OUTPUT_DIRECTORY" >&2
  exit 64
}}
ROOM315_PACKAGE_ROOT="$(cd -- "$(dirname -- "${{BASH_SOURCE[0]}}")" && pwd)"
ROOM315_CHECKPOINT="$(realpath "$1")"
ROOM315_OUTPUT_DIR="$(realpath -m "$2")"
ROOM315_CONTAINER="{CONTAINER}"
[[ "$(uname -m)" == "aarch64" ]] || {{ echo "FAIL: aarch64 required" >&2; exit 1; }}
[[ -f "$ROOM315_CHECKPOINT" ]] || {{ echo "FAIL: checkpoint missing" >&2; exit 1; }}
[[ ! -e "$ROOM315_OUTPUT_DIR" ]] || {{ echo "FAIL: refusing overwrite" >&2; exit 1; }}
mkdir -p "$ROOM315_OUTPUT_DIR"
apptainer exec --nv \
  --bind "$ROOM315_PACKAGE_ROOT:$ROOM315_PACKAGE_ROOT" \
  --bind "$(dirname "$ROOM315_CHECKPOINT"):$(dirname "$ROOM315_CHECKPOINT")" \
  --bind "$ROOM315_OUTPUT_DIR:$ROOM315_OUTPUT_DIR" \
  --env "TORCH_HOME=$ROOM315_PACKAGE_ROOT/torch_cache" \
  "$ROOM315_CONTAINER" bash -lc '
    export PYTHONPATH="$1/scripts${{PYTHONPATH:+:$PYTHONPATH}}"
    python3 "$1/scripts/kairos_package_checks.py" --package-root "$1" --require-gh200 --output "$3/preflight.json"
    python3 "$1/scripts/room_315_visual_state_train_local.py" \
      --splits-dir "$1/dataset/splits" --dataset-root "$1/dataset" \
      --output-dir "$3/run" --train-file train.jsonl --val-file validation.jsonl \
      --epochs 15 --batch-size 32 --early-stopping-patience 3 \
      --learning-rate 0.001 --weight-decay 0.0001 \
      --image-width 224 --image-height 224 --num-workers 4 --device cuda \
      --seed {SEED} --visual-adaptation partial_finetune \
      --visual-pretrained-backbone {PRETRAINED_SOURCE} \
      --resume-checkpoint "$2"
  ' bash "$ROOM315_PACKAGE_ROOT" "$ROOM315_CHECKPOINT" "$ROOM315_OUTPUT_DIR"
'''


def _evaluate_test() -> str:
    return f'''#!/usr/bin/env python3
"""Explicit, logged, evaluation-only test unlock. Never called by training."""

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--unlock-test", action="store_true")
    args = parser.parse_args()
    if not args.unlock_test:
        parser.error("test is locked; explicit --unlock-test is required")
    package = Path(__file__).resolve().parents[1]
    checkpoint = args.checkpoint.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    if not checkpoint.is_file():
        parser.error(f"checkpoint not found: {{checkpoint}}")
    if output.exists():
        parser.error(f"refusing to overwrite: {{output}}")
    output.mkdir(parents=True)
    access = {{
        "schema_version": "room315.explicit_test_access.v1",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "checkpoint": str(checkpoint),
        "explicit_unlock": True,
        "training_process": False,
        "evaluated_split": "test",
    }}
    (output / "test_access_log.json").write_text(
        json.dumps(access, indent=2, sort_keys=True) + "\\n"
    )
    command = [
        sys.executable, str(package / "scripts" / "room_315_visual_state_train_local.py"),
        "--splits-dir", str(package / "dataset" / "splits"),
        "--dataset-root", str(package / "dataset"),
        "--eval-checkpoint", str(checkpoint),
        "--eval-output-dir", str(output / "metrics"),
        "--eval-splits", "test",
        "--unlock-test",
        "--device", "cuda",
        "--seed", "{SEED}",
    ]
    os.execv(command[0], command)


if __name__ == "__main__":
    main()
'''


def _readme() -> str:
    return f'''# Room 315 frozen Kairos visual-state training package

This immutable package trains the paired-camera, fixed-eight Room 315 visual
state model. Model inputs are only the two RGB images. Oracle labels are stored
in separate sidecars. `target_identity`, `target_zone`, `relation_family`, and
all traceability fields are metadata only and are not inputs or targets.

## Frozen model and optimization contract

- schema: `{VISUAL_SCHEMA}`; vector dimension: 200
- identity slots: `{",".join(IDENTITIES)}`
- shared TorchVision ResNet-18; `{PRETRAINED_IDENTIFIER}`
- two independent 224x224 RGB passes through the shared backbone
- ImageNet mean `[0.485,0.456,0.406]`, std `[0.229,0.224,0.225]`
- no augmentation
- partial fine-tuning: `layer4` plus the 200-output head
- parameters: 11,335,560 total; 8,552,776 trainable; 2,782,784 frozen
- masked per-sample Smooth L1; five equal heads: location, payload, bbox,
  `s_m`, and `s_ratio`
- AdamW, learning rate 0.001, weight decay 0.0001, batch size 32
- seed {SEED}
- full: at most 15 epochs, patience 3, best checkpoint by validation total
  weighted loss only

Training and smoke scripts never load test data. Test access is possible only
through `evaluate_test.py` with explicit `--unlock-test`, and is logged.

## Local static verification

```bash
cd /home/tiago/room315_kairos_visual_state_training_v1_seed{SEED}
python3 verify_package.py
```

## Transfer through the active CALMIP VPN tunnel

```bash
rsync -avh --partial --info=progress2 -e 'ssh -p 11220' \\
  /home/tiago/room315_kairos_visual_state_training_v1_seed{SEED}/ \\
  p26065brhm@127.0.0.1:~/room315_kairos_visual_state_training_v1_seed{SEED}/
```

## Kairos commands

```bash
cd ~/room315_kairos_visual_state_training_v1_seed{SEED}
./run_kairos_gh200_smoke.sh
```

After the smoke output is reviewed:

```bash
./run_kairos_gh200_full.sh
```

Resume creates a new output and never overwrites:

```bash
./resume_from_checkpoint.sh /absolute/path/to/last.pt \\
  "$PWD/outputs/kairos_gh200_resume_01_seed{SEED}"
```

The test set remains locked. A future, separately authorized final evaluation
must be explicit:

```bash
apptainer exec --nv --bind "$PWD:$PWD" \\
  --env "TORCH_HOME=$PWD/torch_cache" {CONTAINER} \\
  python3 "$PWD/evaluate_test.py" --checkpoint /absolute/path/to/best.pt \\
  --output-dir "$PWD/outputs/final_test_once" --unlock-test
```
'''


def _schema_sources(repo_root: Path, staging: Path) -> dict[str, Any]:
    source_paths = {
        'rail_network_left': (
            repo_root / 'mfja_robot_control_config/config/'
            'room_315_kinematics/rail_network_left.yaml'
        ),
        'rail_network_right': (
            repo_root / 'mfja_robot_control_config/config/'
            'room_315_kinematics/rail_network_right.yaml'
        ),
        'shuttle_identity': (
            repo_root / 'mfja_robot_control_config/config/'
            'room_315_shuttle_identity/shuttle_identity.yaml'
        ),
        'gazebo_world': (
            repo_root
            / 'mfja_3rd_floor_description/worlds/room_315_only.world'
        ),
    }
    report: dict[str, Any] = {
        'loader': 'frozen snapshots of authoritative repository topology/configuration',
        'files': {},
    }
    for name, source in source_paths.items():
        if not source.is_file():
            raise TrainingPackageError(f'missing authoritative source: {source}')
        suffix = source.suffix
        destination = staging / 'schema_sources' / f'{name}{suffix}'
        _copy_file(source, destination)
        report['files'][name] = {
            'package_path': destination.relative_to(staging).as_posix(),
            'source_path': str(source.resolve()),
            'sha256': _sha256(source),
        }
    return report


def _copy_splits(split_root: Path, staging: Path) -> None:
    destination = staging / 'dataset' / 'splits'
    for source in sorted(split_root.iterdir()):
        if source.is_file():
            _copy_file(source, destination / source.name)


def _copy_images(
    split_root: Path,
    capture_root: Path,
    staging: Path,
) -> dict[str, Any]:
    expected_by_ref: dict[str, str] = {}
    for split in SPLIT_COUNTS:
        for row in _read_jsonl(split_root / f'{split}.jsonl'):
            inputs = row.get('model_input', {}).get('overhead_images', {})
            trace = row.get('traceability_metadata', {}).get('source_images', {})
            for camera in CAMERAS:
                relative = str(inputs.get(camera) or '')
                expected_hash = str(trace.get(camera, {}).get('sha256') or '')
                if not relative or not expected_hash:
                    raise TrainingPackageError(
                        f'{row.get("sample_id")} lacks {camera} image provenance'
                    )
                prior = expected_by_ref.setdefault(relative, expected_hash)
                if prior != expected_hash:
                    raise TrainingPackageError(
                        f'conflicting source image hash for {relative}'
                    )
    for relative, expected_hash in sorted(expected_by_ref.items()):
        source = capture_root / 'dataset' / relative
        if not source.is_file() or _sha256(source) != expected_hash:
            raise TrainingPackageError(
                f'source image missing or changed: {source}'
            )
        destination = staging / 'dataset' / relative
        _copy_file(source, destination)
        if _sha256(destination) != expected_hash:
            raise TrainingPackageError(f'copied image hash mismatch: {destination}')
    return {
        'image_count': len(expected_by_ref),
        'expected_image_count': sum(SPLIT_COUNTS.values()) * 2,
        'all_hashes_verified_before_and_after_copy': True,
    }


def _training_configs(staging: Path) -> None:
    core = {
        'schema_version': 'room315.visual_training_config.v1',
        'seed': SEED,
        'schema': VISUAL_SCHEMA,
        'vector_dimension': 200,
        'fixed_identity_order': list(IDENTITIES),
        'model_inputs': ['model_input.overhead_images.left_rail_rgb',
                         'model_input.overhead_images.right_rail_rgb'],
        'metadata_only': sorted(PROHIBITED),
        'architecture': {
            'kind': 'paired_camera_shared_torchvision_resnet18',
            'pretrained_identifier': PRETRAINED_IDENTIFIER,
            'checkpoint_sha256': CHECKPOINT_SHA256,
            'adaptation': 'partial_finetune',
            'trainable_backbone_scope': 'layer4',
            'input_resolution_per_camera': [224, 224],
            'normalization_mean': [0.485, 0.456, 0.406],
            'normalization_std': [0.229, 0.224, 0.225],
            'augmentations': [],
            'output_dimension': 200,
            'total_parameters': 11335560,
            'trainable_parameters': 8552776,
            'frozen_parameters': 2782784,
        },
        'loss': {
            'kind': 'masked_per_sample_smooth_l1_equal_head_weight_v1',
            'heads': ['segment_location', 'loaded_state', 'bbox', 's_m', 's_ratio'],
            'head_weights': {
                'segment_location': 1.0,
                'loaded_state': 1.0,
                'bbox': 1.0,
                's_m': 1.0,
                's_ratio': 1.0,
            },
            'absent_slots_masked': True,
            'opposite_camera_bbox_loss': 0.0,
        },
        'optimizer': {
            'kind': 'AdamW',
            'learning_rate': 0.001,
            'weight_decay': 0.0001,
        },
        'batch_size': 32,
        'checkpoint_policy': {
            'best_criterion': 'validation_total_weighted_loss_only',
            'write_best': True,
            'write_last': True,
            'resume_supported': True,
            'test_used_for_selection': False,
        },
    }
    smoke = {
        **core,
        'stage': 'smoke',
        'train_file': 'train.jsonl',
        'validation_file': 'validation.jsonl',
        'deterministic_row_limits': {'train': 64, 'validation': 16},
        'epochs': 2,
        'early_stopping_patience': 0,
        'test_access': 'forbidden',
    }
    full = {
        **core,
        'stage': 'full',
        'train_file': 'train.jsonl',
        'validation_file': 'validation.jsonl',
        'train_rows': 1528,
        'validation_rows': 256,
        'epochs': 15,
        'early_stopping_patience': 3,
        'test_access': 'locked_until_explicit_evaluation_only_command',
    }
    _atomic_json(staging / 'config' / 'smoke_training.json', smoke)
    _atomic_json(staging / 'config' / 'full_training.json', full)
    _atomic_json(
        staging / 'config' / 'test_evaluation_locked.json',
        {
            'schema_version': 'room315.test_lock.v1',
            'locked': True,
            'training_may_load_test': False,
            'validation_may_load_test': False,
            'explicit_future_command': (
                'evaluate_test.py --checkpoint CHECKPOINT '
                '--output-dir NEW_DIR --unlock-test'
            ),
            'access_is_logged': True,
        },
    )
    _atomic_json(staging / 'model_architecture_audit.json', core)


def create_package(
    *,
    repo_root: Path,
    capture_root: Path,
    split_root: Path,
    checkpoint: Path,
    output: Path,
) -> dict[str, Any]:
    repo_root = repo_root.expanduser().resolve()
    capture_root = capture_root.expanduser().resolve()
    split_root = split_root.expanduser().resolve()
    checkpoint = checkpoint.expanduser().resolve()
    output = output.expanduser().resolve()
    if output.exists():
        raise TrainingPackageError(f'refusing to overwrite: {output}')
    if _sha256(checkpoint) != CHECKPOINT_SHA256:
        raise TrainingPackageError('official ResNet-18 checkpoint hash mismatch')
    split_verification = _read_json(split_root / 'package_manifest.json')
    if (
        not _read_json(split_root / 'leakage_audit.json').get('passed')
        or not _read_json(split_root / 'target_contract_audit.json').get('passed')
    ):
        raise TrainingPackageError('split leakage/target contract audit did not pass')

    staging = Path(tempfile.mkdtemp(prefix=f'.{output.name}.', dir=output.parent))
    try:
        source_metadata = _schema_sources(repo_root, staging)
        _copy_splits(split_root, staging)
        image_report = _copy_images(split_root, capture_root, staging)

        script_root = repo_root / 'mfja_robot_control_config' / 'scripts'
        for name in (
            'room_315_json_io.py',
            'room_315_visual_state_dataset.py',
            'room_315_visual_state_train_local.py',
            'room_315_kairos_visual_training_package.py',
        ):
            _copy_file(script_root / name, staging / 'scripts' / name)
        _atomic_text(
            staging / 'scripts' / 'room_315_visual_fleet.py',
            _fleet_module(source_metadata),
        )
        _atomic_text(
            staging / 'scripts' / 'room_315_visual_state_smoke.py',
            _smoke_module(),
        )
        _atomic_text(
            staging / 'scripts' / 'kairos_package_checks.py',
            _package_checks(),
        )
        _atomic_text(
            staging / 'scripts' / 'run_inside_container_smoke.sh',
            _inside_container('smoke'),
        )
        _atomic_text(
            staging / 'scripts' / 'run_inside_container_full.sh',
            _inside_container('full'),
        )
        _atomic_text(staging / 'run_kairos_gh200_smoke.sh', _launcher('smoke'))
        _atomic_text(staging / 'run_kairos_gh200_full.sh', _launcher('full'))
        _atomic_text(staging / 'resume_from_checkpoint.sh', _resume_script())
        _atomic_text(staging / 'evaluate_test.py', _evaluate_test())
        _atomic_text(
            staging / 'verify_package.py',
            '''#!/usr/bin/env python3
from pathlib import Path
import json
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent / "scripts"))
from room_315_kairos_visual_training_package import verify_package
print(json.dumps(verify_package(Path(__file__).resolve().parent), indent=2, sort_keys=True))
''',
        )
        _atomic_text(staging / 'README.md', _readme())
        _training_configs(staging)

        checkpoint_destination = (
            staging / 'torch_cache' / 'hub' / 'checkpoints' / CHECKPOINT_NAME
        )
        _copy_file(checkpoint, checkpoint_destination)
        _atomic_text(
            checkpoint_destination.with_suffix('.pth.sha256'),
            f'{CHECKPOINT_SHA256}  {CHECKPOINT_NAME}\n',
        )

        for path in (
            staging / 'run_kairos_gh200_smoke.sh',
            staging / 'run_kairos_gh200_full.sh',
            staging / 'resume_from_checkpoint.sh',
            staging / 'evaluate_test.py',
            staging / 'verify_package.py',
            staging / 'scripts' / 'kairos_package_checks.py',
            staging / 'scripts' / 'run_inside_container_smoke.sh',
            staging / 'scripts' / 'run_inside_container_full.sh',
            staging / 'scripts' / 'room_315_visual_state_train_local.py',
        ):
            _make_executable(path)

        source_inputs = {
            'capture_root': str(capture_root),
            'capture_manifest_sha256': _sha256(
                capture_root / 'scenario_manifest.jsonl'
            ),
            'split_root': str(split_root),
            'split_package_tree_sha256': split_verification.get('tree_sha256'),
            'checkpoint_source_path': str(checkpoint),
            'checkpoint_sha256': CHECKPOINT_SHA256,
            'image_copy': image_report,
            'schema_sources': source_metadata,
        }
        _atomic_json(staging / 'package_manifest.json', _manifest(staging, source_inputs))
        os.replace(staging, output)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return verify_package(output)


def _assert(condition: bool, message: str) -> None:
    if not condition:
        raise TrainingPackageError(message)


def _check_shell(path: Path) -> None:
    result = os.system(f'bash -n {str(path)!r}')
    if result != 0:
        raise TrainingPackageError(f'shell syntax failed: {path}')


def verify_training_preflight(root: Path) -> dict[str, Any]:
    """Verify train/validation payload only; never open test data."""
    root = root.expanduser().resolve()
    manifest = _read_json(root / 'package_manifest.json')
    _assert(
        manifest.get('schema_version') == SCHEMA_VERSION,
        'package manifest schema mismatch',
    )
    checkpoint = root / 'torch_cache/hub/checkpoints' / CHECKPOINT_NAME
    _assert(
        checkpoint.is_file() and _sha256(checkpoint) == CHECKPOINT_SHA256,
        'official checkpoint is missing or changed',
    )
    counts: dict[str, int] = {}
    image_refs: set[str] = set()
    files_read: list[str] = [
        'package_manifest.json',
        f'torch_cache/hub/checkpoints/{CHECKPOINT_NAME}',
    ]
    for split in ('train', 'validation'):
        row_path = root / 'dataset/splits' / f'{split}.jsonl'
        label_path = (
            root / 'dataset/splits' / f'{split}_visual_labels.jsonl'
        )
        rows = _read_jsonl(row_path)
        labels = _read_jsonl(label_path)
        files_read.extend([
            row_path.relative_to(root).as_posix(),
            label_path.relative_to(root).as_posix(),
        ])
        _assert(len(rows) == SPLIT_COUNTS[split], f'{split} row count mismatch')
        _assert(
            len(labels) == SPLIT_COUNTS[split],
            f'{split} label count mismatch',
        )
        _assert(
            [row.get('sample_id') for row in rows]
            == [row.get('sample_id') for row in labels],
            f'{split} row/label ordering mismatch',
        )
        counts[split] = len(rows)
        for row in rows:
            model_input = row.get('model_input')
            _assert(
                isinstance(model_input, dict)
                and set(model_input) == {'overhead_images'},
                f'{row.get("sample_id")}: model input is not camera-only',
            )
            provenance = row.get(
                'traceability_metadata', {}
            ).get('source_images', {})
            for camera, relative in model_input['overhead_images'].items():
                relative = str(relative)
                _assert(
                    not relative.startswith('test'),
                    'test image reference entered training preflight',
                )
                image_path = root / 'dataset' / relative
                _assert(image_path.is_file(), f'missing image {relative}')
                expected_hash = str(
                    provenance.get(camera, {}).get('sha256') or ''
                )
                _assert(
                    expected_hash
                    and _sha256(image_path) == expected_hash,
                    f'train/validation image hash mismatch: {relative}',
                )
                image_refs.add(relative)
    _assert(len(image_refs) == 3568, 'train/validation image count mismatch')
    _assert(
        not any(
            Path(relative).name.startswith('test')
            for relative in files_read
        ),
        'training preflight opened a test file',
    )
    return {
        'passed': True,
        'kind': 'train_validation_only_preflight',
        'package_root': str(root),
        'scenario_counts': counts,
        'unique_images': len(image_refs),
        'checkpoint_sha256': CHECKPOINT_SHA256,
        'test_files_read': [],
        'test_data_touched': False,
        'test_metrics_computed': False,
    }


def verify_package(root: Path) -> dict[str, Any]:
    root = root.expanduser().resolve()
    manifest = _read_json(root / 'package_manifest.json')
    actual_files = _manifest_files(root)
    _assert(manifest.get('files') == actual_files, 'package file hashes changed')
    actual_manifest = _manifest(root, manifest.get('source_inputs') or {})
    _assert(
        manifest.get('tree_sha256') == actual_manifest['tree_sha256'],
        'package tree hash changed',
    )
    _assert(
        manifest.get('file_count') == len(actual_files),
        'package file count changed',
    )
    checkpoint = root / 'torch_cache/hub/checkpoints' / CHECKPOINT_NAME
    _assert(
        checkpoint.is_file() and _sha256(checkpoint) == CHECKPOINT_SHA256,
        'official checkpoint is missing or changed',
    )
    leakage = _read_json(root / 'dataset/splits/leakage_audit.json')
    contract = _read_json(root / 'dataset/splits/target_contract_audit.json')
    _assert(leakage.get('passed') is True, 'copied leakage audit is not PASS')
    _assert(contract.get('passed') is True, 'copied target audit is not PASS')

    scenario_total = 0
    configuration_counts: dict[str, int] = {}
    image_refs: set[str] = set()
    forbidden_input_hits: list[str] = []
    forbidden_target_hits: list[str] = []
    opposite_camera_bbox_weight = 0.0
    for split, expected in SPLIT_COUNTS.items():
        rows = _read_jsonl(root / 'dataset/splits' / f'{split}.jsonl')
        labels = _read_jsonl(
            root / 'dataset/splits' / f'{split}_visual_labels.jsonl'
        )
        _assert(len(rows) == expected, f'{split} row count mismatch')
        _assert(len(labels) == expected, f'{split} label count mismatch')
        _assert(
            [row.get('sample_id') for row in rows]
            == [row.get('sample_id') for row in labels],
            f'{split} row/label ordering mismatch',
        )
        config_ids = {
            str(row.get('traceability_metadata', {}).get(
                'presence_configuration_id'
            ))
            for row in rows
        }
        configuration_counts[split] = len(config_ids)
        _assert(
            len(config_ids) == CONFIGURATION_COUNTS[split],
            f'{split} configuration count mismatch',
        )
        for row, label_row in zip(rows, labels):
            model_input = row.get('model_input')
            _assert(
                isinstance(model_input, dict)
                and set(model_input) == {'overhead_images'},
                f'{row.get("sample_id")}: model input is not camera-only',
            )
            serialized_input = json.dumps(model_input, sort_keys=True)
            forbidden_input_hits.extend(
                field for field in PROHIBITED if field in serialized_input
            )
            images = model_input['overhead_images']
            _assert(set(images) == set(CAMERAS), 'paired camera input incomplete')
            for camera in CAMERAS:
                relative = str(images[camera])
                image_path = root / 'dataset' / relative
                _assert(image_path.is_file(), f'missing image {relative}')
                image_refs.add(relative)
            label = label_row.get('visual_state_labels')
            _assert(
                isinstance(label, dict)
                and label.get('schema_version') == VISUAL_SCHEMA,
                'visual label schema mismatch',
            )
            shuttles = label.get('shuttles')
            _assert(
                [item.get('id') for item in shuttles] == list(IDENTITIES),
                'fixed identity order mismatch',
            )
            for identity, shuttle in zip(IDENTITIES, shuttles):
                own = 'left_rail_rgb' if identity.startswith('L') else 'right_rail_rgb'
                other = 'right_rail_rgb' if own == 'left_rail_rgb' else 'left_rail_rgb'
                observations = shuttle.get('camera_observations', {})
                _assert(
                    observations.get(other, {}).get('bbox_target_mask')
                    == [0.0, 0.0, 0.0, 0.0],
                    f'{row.get("sample_id")}:{identity}: opposite bbox unmasked',
                )
                opposite_camera_bbox_weight += sum(
                    observations.get(other, {}).get('bbox_target_mask') or []
                )
        scenario_total += len(rows)
    _assert(not forbidden_input_hits, 'prohibited metadata entered model input')
    declared_targets = set(contract.get('prediction_target_fields') or [])
    vectorizer_names = contract.get('vectorizer', {}).get('names') or []
    for field in PROHIBITED:
        if field in declared_targets or any(field in name for name in vectorizer_names):
            forbidden_target_hits.append(field)
    _assert(not forbidden_target_hits, 'prohibited metadata entered target labels')
    _assert(scenario_total == 2040, 'total scenario count mismatch')
    _assert(len(image_refs) == 4080, 'unique image count mismatch')
    _assert(opposite_camera_bbox_weight == 0.0, 'opposite bbox loss is nonzero')

    for path in (
        *sorted((root / 'scripts').glob('*.py')),
        root / 'evaluate_test.py',
        root / 'verify_package.py',
    ):
        compile(path.read_text(encoding='utf-8'), str(path), 'exec')
    for path in (
        root / 'run_kairos_gh200_smoke.sh',
        root / 'run_kairos_gh200_full.sh',
        root / 'resume_from_checkpoint.sh',
        root / 'scripts/run_inside_container_smoke.sh',
        root / 'scripts/run_inside_container_full.sh',
    ):
        _check_shell(path)

    smoke_text = (root / 'scripts/run_inside_container_smoke.sh').read_text()
    full_text = (root / 'scripts/run_inside_container_full.sh').read_text()
    _assert('test.jsonl' not in smoke_text + full_text, 'training script names test')
    _assert('--eval-splits' not in smoke_text + full_text, 'training evaluates a split')
    eval_text = (root / 'evaluate_test.py').read_text()
    _assert(
        '--unlock-test' in eval_text and '"test"' in eval_text,
        'explicit test unlock wrapper is incomplete',
    )
    return {
        'passed': True,
        'schema_version': SCHEMA_VERSION,
        'package_root': str(root),
        'verified_file_count': len(actual_files),
        'verified_bytes': sum(item['bytes'] for item in actual_files),
        'tree_sha256': actual_manifest['tree_sha256'],
        'scenario_counts': dict(SPLIT_COUNTS),
        'configuration_counts': configuration_counts,
        'unique_images': len(image_refs),
        'checkpoint_sha256': CHECKPOINT_SHA256,
        'model_input_prohibited_hits': [],
        'prediction_target_prohibited_hits': [],
        'opposite_camera_bbox_loss_weight_sum': opposite_camera_bbox_weight,
        'test_lock': {
            'training_scripts_reference_test': False,
            'test_requires_explicit_unlock': True,
            'test_evaluation_executed': False,
        },
        'static_only': True,
        'training_executed': False,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command', required=True)
    create = subparsers.add_parser('create')
    create.add_argument('--repo-root', type=Path, required=True)
    create.add_argument('--capture-root', type=Path, required=True)
    create.add_argument('--split-root', type=Path, required=True)
    create.add_argument('--checkpoint', type=Path, required=True)
    create.add_argument('--output', type=Path, required=True)
    verify = subparsers.add_parser('verify')
    verify.add_argument('--package', type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    if args.command == 'create':
        result = create_package(
            repo_root=args.repo_root,
            capture_root=args.capture_root,
            split_root=args.split_root,
            checkpoint=args.checkpoint,
            output=args.output,
        )
    else:
        result = verify_package(args.package)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
