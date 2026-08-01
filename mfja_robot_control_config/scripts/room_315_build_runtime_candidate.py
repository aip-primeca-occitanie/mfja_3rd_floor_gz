#!/usr/bin/env python3
"""Build the immutable corrected Experiment-A Full runtime candidate."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any


CHECKPOINT = Path(
    '/home/tiago/room315_experiment_a_corrected_local_outputs/'
    'full_seed31520260730_attempt1/best.pt'
)
CHECKPOINT_SHA256 = (
    '4cb9cd88b0199bf38bcfb08741e22bcadca54aeb0036d757784531a38cdd6a70'
)
SOURCE_OUTPUT = CHECKPOINT.parent
AUTHORITATIVE_SIDECARS = Path(
    '/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run'
)
ROLLBACK_CHECKPOINT = AUTHORITATIVE_SIDECARS / 'best.pt'
ROLLBACK_SHA256 = (
    '8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d'
)
CANDIDATE_ID = (
    'room315_visual_runtime_candidate_experiment_a_full_'
    'seed31520260730_epoch24_4cb9cd88'
)
DEFAULT_OUTPUT = Path('/home/tiago') / CANDIDATE_ID
IDENTITIES = ['L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4']
SCHEMA = 'room315.visual_state.v3'
MODEL_KIND = 'structured_visual_state_torchvision_resnet18_fixed8_v3'
RIGHT_SLOT3_RATIO = 0.447469343


class CandidateBuildError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


def validate_and_load_checkpoint() -> dict[str, Any]:
    if not CHECKPOINT.is_file():
        raise CandidateBuildError(f'checkpoint is missing: {CHECKPOINT}')
    actual = sha256_file(CHECKPOINT)
    if actual != CHECKPOINT_SHA256:
        raise CandidateBuildError(
            f'checkpoint SHA-256 mismatch: expected {CHECKPOINT_SHA256}, got {actual}'
        )
    if sha256_file(ROLLBACK_CHECKPOINT) != ROLLBACK_SHA256:
        raise CandidateBuildError('rollback checkpoint SHA-256 mismatch')
    try:
        import torch
        import torchvision
    except ImportError as exc:
        raise CandidateBuildError('Torch and TorchVision are required') from exc

    script_dir = Path(__file__).resolve().parent
    if str(script_dir) not in sys.path:
        sys.path.insert(0, str(script_dir))
    from room_315_visual_model import build_visual_state_model

    checkpoint = torch.load(CHECKPOINT, map_location='cpu', weights_only=False)
    if not isinstance(checkpoint, dict):
        raise CandidateBuildError('checkpoint root is not a dictionary')
    if int(checkpoint.get('epoch', -1)) != 24:
        raise CandidateBuildError('corrected Full checkpoint is not epoch 24')
    state = checkpoint.get('model_state_dict')
    if not isinstance(state, dict):
        raise CandidateBuildError('model_state_dict is missing')
    model = build_visual_state_model(
        torch,
        torchvision,
        output_dim=200,
        adaptation_mode='partial_finetune',
        lora_rank=4,
    )
    try:
        result = model.load_state_dict(state, strict=True)
    except Exception as exc:
        raise CandidateBuildError('strict checkpoint loading failed') from exc
    if result.missing_keys or result.unexpected_keys:
        raise CandidateBuildError('strict checkpoint load returned incompatible keys')
    if int(model.head[-1].out_features) != 200:
        raise CandidateBuildError('prediction head dimension is not 200')
    return checkpoint


def acceptance_scenarios() -> dict[str, Any]:
    def row(
        scenario_id: str,
        coverage: list[str],
        left: list[tuple[str, str, str]],
        right: list[tuple[str, str, str]],
        relations: list[dict[str, Any]],
        goal: str,
    ) -> dict[str, Any]:
        present = left + right
        return {
            'scenario_id': scenario_id,
            'coverage': coverage,
            'gazebo_setup': {
                'left_active_identities': [item[0] for item in left],
                'right_active_identities': [item[0] for item in right],
                'left_start_positions': [item[1] for item in left],
                'right_start_positions': [item[1] for item in right],
                'left_loaded_identities': [
                    item[0] for item in left if item[2] == 'loaded'
                ],
                'right_loaded_identities': [
                    item[0] for item in right if item[2] == 'loaded'
                ],
            },
            'ground_truth': {
                'schema_version': SCHEMA,
                'present_identities': [item[0] for item in present],
                'shuttles': [
                    {
                        'identity': identity,
                        'side': 'left' if identity.startswith('L') else 'right',
                        'segment': position.split('@', 1)[0],
                        's_ratio': float(position.split('@', 1)[1]),
                        'loaded_state': payload,
                    }
                    for identity, position, payload in present
                ],
                'relations': relations,
                'source': 'acceptance_scenario_configuration_oracle',
                'model_prediction_target': False,
            },
            'suggested_natural_language_goal': goal,
        }

    return {
        'schema_version': 'room315.runtime_acceptance_scenarios.v1',
        'candidate_id': CANDIDATE_ID,
        'right_slot_3_authoritative': {
            'source': (
                'mfja_robot_control_config/config/room_315_kinematics/'
                'rail_devices_right.yaml:slots.slot_3'
            ),
            'segment': 'A34E',
            's_ratio': RIGHT_SLOT3_RATIO,
        },
        'scenarios': [
            row(
                'accept_l4_loaded', ['l4_loaded'],
                [('L4', 'A34E@0.200000000', 'loaded')],
                [('R1', 'A12E@0.200000000', 'empty')],
                [], 'Move shuttle L4 to slot 3 on the left rail.',
            ),
            row(
                'accept_r4_loaded', ['r4_loaded'],
                [('L1', 'A12E@0.200000000', 'empty')],
                [('R4', 'A34E@0.700000000', 'loaded')],
                [], 'Move shuttle R4 to slot 3 on the right rail.',
            ),
            row(
                'accept_exact_l2_l4_r4', ['exact_l2_l4_r4'],
                [
                    ('L2', 'A12E@0.180000000', 'empty'),
                    ('L4', 'A34E@0.720000000', 'loaded'),
                ],
                [('R4', 'A34E@0.300000000', 'empty')],
                [], 'Move the loaded left shuttle to slot 3.',
            ),
            row(
                'accept_right_slot3_plus_005',
                ['right_slot3_deliberate_offset'],
                [('L1', 'A12E@0.250000000', 'empty')],
                [('R4', 'A34E@0.497469343', 'loaded')],
                [{
                    'kind': 'right_slot3_offset',
                    'identity': 'R4',
                    'offset_s_ratio': 0.05,
                }],
                'Move shuttle R4 to slot 3 on the right rail.',
            ),
            row(
                'accept_sparse', ['sparse_scene'],
                [('L3', 'A14@0.400000000', 'loaded')],
                [('R2', 'A34I@0.500000000', 'empty')],
                [], 'Move the loaded left shuttle to slot 1.',
            ),
            row(
                'accept_dense', ['dense_scene'],
                [
                    ('L1', 'A1E@0.200000000', 'empty'),
                    ('L2', 'A12I@0.400000000', 'loaded'),
                    ('L3', 'A3E@0.300000000', 'empty'),
                    ('L4', 'A34I@0.650000000', 'loaded'),
                ],
                [
                    ('R1', 'A1I@0.250000000', 'loaded'),
                    ('R2', 'A12E@0.600000000', 'empty'),
                    ('R3', 'A3I@0.350000000', 'loaded'),
                    ('R4', 'A34E@0.760000000', 'empty'),
                ],
                [], 'Move shuttle R4 to slot 3 on the right rail.',
            ),
            row(
                'accept_multi_blocker', ['multi_blocker_scene'],
                [('L4', 'A34E@0.180000000', 'loaded')],
                [
                    ('R4', 'A12E@0.120000000', 'loaded'),
                    ('R2', 'A12E@0.500000000', 'empty'),
                    ('R1', 'A12E@0.880000000', 'empty'),
                ],
                [
                    {'kind': 'ahead_blocker', 'subject': 'R2', 'target': 'R4'},
                    {'kind': 'multi_blocker', 'subject': 'R1', 'target': 'R4'},
                ],
                'Move shuttle R4 to slot 4 on the right rail.',
            ),
        ],
    }


def minimal_sidecars(checkpoint: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    training_config = {
        'schema_version': 'room315.runtime_training_contract.v1',
        'visual_model_kind': MODEL_KIND,
        'output_dim': 200,
        'image_resize': 'direct_bilinear_resize',
        'augmentations': [],
        'visual_adaptation': 'partial_finetune',
        'visual_lora_rank': 4,
        'visual_model': {
            'model_kind': MODEL_KIND,
            'backbone_architecture': 'resnet18',
            'backbone_library': 'torchvision',
            'adaptation_mode': 'partial_finetune',
            'backbone_trainable_scope': 'layer4',
            'image_preprocessing': {
                'input_resolution': [224, 224],
            },
        },
        'dataset_paths_included': False,
        'runtime_only': True,
    }
    run_metadata = {
        'schema_version': 'room315.visual_training_run.v2',
        'seed': 31520260730,
        'checkpoint_epoch': int(checkpoint['epoch']),
        'continuation_epoch': int(checkpoint.get('continuation_epoch', 10)),
        'image_preprocessing': {
            'augmentations': [],
            'input_resolution': [224, 224],
            'normalization_mean_per_rgb_view': [0.485, 0.456, 0.406],
            'normalization_std_per_rgb_view': [0.229, 0.224, 0.225],
            'resize': 'direct_bilinear_resize',
            'value_range': [0.0, 1.0],
        },
        'dataset_paths_included': False,
        'runtime_only': True,
    }
    return training_config, run_metadata


def build(output: Path) -> None:
    output = output.expanduser().resolve()
    if output.exists():
        raise CandidateBuildError(f'refusing to overwrite candidate: {output}')
    checkpoint = validate_and_load_checkpoint()
    authoritative_vectorizer = json.loads(
        (AUTHORITATIVE_SIDECARS / 'visual_label_vectorizer.json').read_text()
    )
    authoritative_stats = json.loads(
        (AUTHORITATIVE_SIDECARS / 'target_stats.json').read_text()
    )
    if checkpoint.get('label_vectorizer') != authoritative_vectorizer:
        raise CandidateBuildError('checkpoint vectorizer differs from authoritative sidecar')
    if checkpoint.get('target_stats') != authoritative_stats:
        raise CandidateBuildError('checkpoint target stats differ from authoritative sidecar')
    if int(authoritative_vectorizer.get('dim', -1)) != 200:
        raise CandidateBuildError('authoritative vectorizer dimension is not 200')
    if authoritative_vectorizer.get('fixed_identity_order') != IDENTITIES:
        raise CandidateBuildError('authoritative identity order is incompatible')

    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f'.{output.name}.staging-', dir=output.parent))
    try:
        shutil.copyfile(CHECKPOINT, staging / 'best.pt')
        shutil.copyfile(
            AUTHORITATIVE_SIDECARS / 'visual_label_vectorizer.json',
            staging / 'visual_label_vectorizer.json',
        )
        shutil.copyfile(
            AUTHORITATIVE_SIDECARS / 'target_stats.json',
            staging / 'target_stats.json',
        )
        training_config, run_metadata = minimal_sidecars(checkpoint)
        write_json(staging / 'training_config.json', training_config)
        write_json(staging / 'run_metadata.json', run_metadata)
        initial_names = (
            'best.pt',
            'target_stats.json',
            'visual_label_vectorizer.json',
            'training_config.json',
            'run_metadata.json',
        )
        artifact_hashes = {
            name: sha256_file(staging / name) for name in initial_names
        }
        runtime_configuration = {
            'schema_version': 'room315.visual_runtime_candidate.v1',
            'candidate_id': CANDIDATE_ID,
            'deployment_state': 'candidate',
            'checkpoint': {
                'filename': 'best.pt',
                'epoch': 24,
                'continuation_epoch': 10,
                'sha256': CHECKPOINT_SHA256,
            },
            'model_contract': {
                'architecture': 'paired_torchvision_resnet18',
                'backbone_weights_lineage': 'ResNet18_Weights.IMAGENET1K_V1',
                'adaptation': 'partial_finetune_layer4_plus_head',
                'paired_rgb_input_shape': ['B', 6, 224, 224],
                'output_dimension': 200,
                'identity_order': IDENTITIES,
                'vectorizer_schema': SCHEMA,
                'checkpoint_loading': 'strict',
                'normalization_mean_per_rgb_view': [0.485, 0.456, 0.406],
                'normalization_std_per_rgb_view': [0.229, 0.224, 0.225],
            },
            'artifact_sha256': artifact_hashes,
            'selection': {
                'ros_parameter': 'checkpoint_path',
                'environment_variable': 'ROOM315_VISUAL_MODEL_PATH',
            },
            'automatic_deployment_approval': False,
        }
        write_json(staging / 'runtime_configuration.json', runtime_configuration)
        runtime_hash = sha256_file(staging / 'runtime_configuration.json')

        write_json(staging / 'acceptance_scenarios.json', acceptance_scenarios())
        candidate_state = {
            'schema_version': 'room315.deployment_candidate_state.v1',
            'candidate_id': CANDIDATE_ID,
            'checkpoint_sha256': CHECKPOINT_SHA256,
            'state': 'candidate',
            'approved': False,
            'automatic_approval_allowed': False,
            'acceptance_execution_status': 'not_run',
        }
        write_json(staging / 'candidate_state.json', candidate_state)
        provenance = {
            'schema_version': 'room315.deployment_provenance.v1',
            'candidate_id': CANDIDATE_ID,
            'experiment_a_full': {
                'output_directory': str(SOURCE_OUTPUT),
                'checkpoint': str(CHECKPOINT),
                'checkpoint_sha256': CHECKPOINT_SHA256,
                'run_metadata_sha256': sha256_file(SOURCE_OUTPUT / 'run_metadata.json'),
                'final_report_sha256': sha256_file(SOURCE_OUTPUT / 'final_report.json'),
                'selection': 'validation total weighted loss only',
                'epoch': 24,
                'continuation_epoch': 10,
            },
            'initialization_checkpoint': {
                'path': str(ROLLBACK_CHECKPOINT),
                'sha256': ROLLBACK_SHA256,
                'epoch': 14,
            },
            'canary_lineage': {
                'relationship': (
                    'Experiment-A Full declares that deployment approval requires '
                    'a separate Canary comparison.'
                ),
                'canary_accessed_during_candidate_build': False,
                'canary_artifacts_copied': False,
                'canary_result_asserted': False,
                'status': 'external prerequisite; not imported or evaluated',
            },
            'data_access': {
                'training_or_evaluation_performed': False,
                'legacy_test_accessed': False,
                'canary_accessed': False,
            },
        }
        write_json(staging / 'provenance.json', provenance)
        write_json(staging / 'rollback_option.json', {
            'schema_version': 'room315.runtime_rollback.v1',
            'checkpoint_path': str(ROLLBACK_CHECKPOINT),
            'checkpoint_sha256': ROLLBACK_SHA256,
            'preserved_unchanged': True,
            'default_runtime_yaml_still_selects_rollback': True,
        })
        write_json(staging / 'acceptance_report_not_run.json', {
            'schema_version': 'room315.runtime_acceptance_report.v1',
            'candidate_id': CANDIDATE_ID,
            'checkpoint_sha256': CHECKPOINT_SHA256,
            'deployment_state': 'candidate',
            'acceptance_status': 'not_run',
            'automatic_deployment_approval': False,
            'approval': {'approved': False},
        })

        yaml = f"""room_315_visual_state_inference_node:
  ros__parameters:
    use_sim_time: true
    checkpoint_path: {staging / 'best.pt'}
    sidecar_directory: {staging}
    expected_checkpoint_sha256: {artifact_hashes['best.pt']}
    expected_target_stats_sha256: {artifact_hashes['target_stats.json']}
    expected_vectorizer_sha256: {artifact_hashes['visual_label_vectorizer.json']}
    expected_training_config_sha256: {artifact_hashes['training_config.json']}
    expected_run_metadata_sha256: {artifact_hashes['run_metadata.json']}
    expected_runtime_configuration_sha256: {runtime_hash}
    device: auto
    presence_state_timeout_s: 1.0
    presence_warmup_s: 0.5
    reconcile_position_consistency: true
    position_reconciliation_policy: canonical_s_m
    max_position_reconciliation_error_m: 0.40
    dry_run_state_fusion: true
    plansys2_update_enabled: false
    raw_model_prediction_topic: /room_315/visual_state/raw_model_prediction
"""
        # Replace the staging prefix before atomically publishing the directory.
        yaml = yaml.replace(str(staging), str(output))
        (staging / 'runtime_ros_parameters.yaml').write_text(yaml, encoding='utf-8')
        env = f"""export ROOM315_VISUAL_MODEL_PATH='{output / 'best.pt'}'
export ROOM315_VISUAL_SIDECAR_DIRECTORY='{output}'
export ROOM315_VISUAL_EXPECTED_CHECKPOINT_SHA256='{artifact_hashes['best.pt']}'
export ROOM315_VISUAL_EXPECTED_TARGET_STATS_SHA256='{artifact_hashes['target_stats.json']}'
export ROOM315_VISUAL_EXPECTED_VECTORIZER_SHA256='{artifact_hashes['visual_label_vectorizer.json']}'
export ROOM315_VISUAL_EXPECTED_TRAINING_CONFIG_SHA256='{artifact_hashes['training_config.json']}'
export ROOM315_VISUAL_EXPECTED_RUN_METADATA_SHA256='{artifact_hashes['run_metadata.json']}'
export ROOM315_VISUAL_EXPECTED_RUNTIME_CONFIGURATION_SHA256='{runtime_hash}'
"""
        (staging / 'activate_candidate.env').write_text(env, encoding='utf-8')

        source_report = Path(__file__).with_name(
            'room_315_runtime_acceptance_report.py'
        )
        shutil.copyfile(source_report, staging / source_report.name)
        run_script = f"""#!/usr/bin/env bash
set -euo pipefail
CANDIDATE='{output}'
if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo 'usage: run_gazebo_runtime_acceptance.sh SCENARIO_ID NEW_OUTPUT_ROOT [--execute]' >&2
  exit 2
fi
SCENARIO_ID="$1"
OUTPUT_ROOT="$2"
if [[ -e "$OUTPUT_ROOT" ]]; then
  echo "refusing to overwrite acceptance output: $OUTPUT_ROOT" >&2
  exit 3
fi
EXECUTION_ARGS=(enable_task_execution:=false execution_enabled:=false)
if [[ "${{3:-}}" == '--execute' ]]; then
  EXECUTION_ARGS=(enable_task_execution:=true execution_enabled:=true)
elif [[ $# -eq 3 ]]; then
  echo 'third argument must be --execute' >&2
  exit 2
fi
ros2 launch mfja_robot_control_config room_315_runtime_acceptance.launch.py \\
  candidate_directory:="$CANDIDATE" \\
  scenario_id:="$SCENARIO_ID" \\
  output_root:="$OUTPUT_ROOT" \\
  "${{EXECUTION_ARGS[@]}}"
"""
        (staging / 'run_gazebo_runtime_acceptance.sh').write_text(
            run_script, encoding='utf-8'
        )
        report_script = f"""#!/usr/bin/env bash
set -euo pipefail
CANDIDATE='{output}'
if [[ $# -ne 1 ]]; then
  echo 'usage: generate_acceptance_report.sh EXISTING_OUTPUT_ROOT' >&2
  exit 2
fi
python3 "$CANDIDATE/room_315_runtime_acceptance_report.py" \\
  --candidate-directory "$CANDIDATE" \\
  --event-directory "$1/events" \\
  --output "$1/acceptance_report.json"
"""
        (staging / 'generate_acceptance_report.sh').write_text(
            report_script, encoding='utf-8'
        )
        readme = f"""# Room 315 corrected Experiment-A Full runtime candidate

Candidate ID: `{CANDIDATE_ID}`

State: **candidate**. This package is not deployment approval. It contains no
dataset and its acceptance tooling cannot approve deployment automatically.

The default repository runtime YAML still points to the epoch-14 rollback
checkpoint. Select this candidate explicitly with:

```bash
source {output / 'activate_candidate.env'}
ros2 launch mfja_robot_control_config room_315_visual_state_runtime.launch.py
```

Run one observation-only Gazebo acceptance scenario into a new output path:

```bash
{output / 'run_gazebo_runtime_acceptance.sh'} accept_l4_loaded \\
  /home/tiago/room315_runtime_acceptance_outputs/accept_l4_loaded_attempt1
```

Passing `--execute` is a separate explicit actuation opt-in. Submit the
scenario's `suggested_natural_language_goal` only after inspecting its setup.

After all seven scenario event files have been collected under one output
root, generate the non-approving report:

```bash
{output / 'generate_acceptance_report.sh'} NEW_OUTPUT_ROOT
```

`SHA256SUMS` covers every package payload except itself. The package directory
and files are permission-hardened after atomic publication.
"""
        (staging / 'README.md').write_text(readme, encoding='utf-8')

        payload_files = sorted(
            path for path in staging.iterdir()
            if path.is_file() and path.name not in {'deployment_manifest.json', 'SHA256SUMS'}
        )
        manifest = {
            'schema_version': 'room315.immutable_deployment_manifest.v1',
            'candidate_id': CANDIDATE_ID,
            'deployment_state': 'candidate',
            'immutable_policy': 'atomic creation; refuse overwrite; read/execute-only permissions',
            'files': [
                {
                    'path': path.name,
                    'size_bytes': path.stat().st_size,
                    'sha256': sha256_file(path),
                }
                for path in payload_files
            ],
        }
        write_json(staging / 'deployment_manifest.json', manifest)
        sums_files = sorted(
            path for path in staging.iterdir()
            if path.is_file() and path.name != 'SHA256SUMS'
        )
        (staging / 'SHA256SUMS').write_text(
            ''.join(f'{sha256_file(path)}  {path.name}\n' for path in sums_files),
            encoding='utf-8',
        )
        for path in staging.iterdir():
            if path.is_file():
                executable = path.suffix in {'.sh', '.py'}
                path.chmod(0o555 if executable else 0o444)
        os.replace(staging, output)
        output.chmod(0o555)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise

    print(json.dumps({
        'status': 'CANDIDATE_CREATED',
        'candidate': str(output),
        'checkpoint_sha256': CHECKPOINT_SHA256,
        'deployment_state': 'candidate',
    }, sort_keys=True))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    build(args.output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
