#!/usr/bin/env python3
"""Build the immutable, shadow-only Room 315 V4 runtime candidate.

The source paths and hashes in this module deliberately identify one selected
validation checkpoint and its already-completed Canary handoff.  Building a
candidate performs no training, evaluation, dataset access, or Test access.
It verifies the pinned evidence and strictly loads the checkpoint before a
staging directory is created, then publishes one read-only directory with an
atomic rename.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import re
import shutil
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping


TRAINING_RUN = Path(
    '/home/tiago/room315_visual_v4_outputs/'
    'pilot_v4_seed31520260811_attempt1'
)
CANARY_RUN = Path(
    '/home/tiago/room315_visual_v4_outputs/'
    'canary_v4_e5b24314038c7d92_attempt1'
)
CANARY_LEDGER_ROOT = Path(
    '/home/tiago/room315_visual_v4_outputs/canary_attempt_ledger_v1'
)
CANARY_ATTEMPT_KEY = (
    'e5b24314038c7d92596b7153f6c0013688de04c312706c0f3a5dce5cdd100807'
)

CHECKPOINT = TRAINING_RUN / 'checkpoints/epoch_011.pt'
TRAINING_FINAL_REPORT = TRAINING_RUN / 'final_report.json'
VALIDATION_ACCEPTANCE = TRAINING_RUN / 'validation_acceptance.json'
VALIDATION_SEGMENT_CALIBRATION = (
    TRAINING_RUN / 'validation_segment_calibration.json'
)
EFFECTIVE_CONFIG = TRAINING_RUN / 'effective_config.json'
PUBLIC_TOPOLOGY_CONTRACT = TRAINING_RUN / 'public_topology_contract.json'
CANARY_FINAL_REPORT = CANARY_RUN / 'final_report.json'
CANARY_COMPLETION_LEDGER = (
    CANARY_LEDGER_ROOT / f'{CANARY_ATTEMPT_KEY}.completed.json'
)

CHECKPOINT_SHA256 = (
    '869d64049b0092c37d21a4c8b910dc6b91954527e0e49c5694fa82dce570f40d'
)
TRAINING_FINAL_REPORT_SHA256 = (
    'd1e2fd2d360f724fb092fe22d8cd3b9237d80f8bd4c62ba134b61ee9a8875218'
)
VALIDATION_ACCEPTANCE_SHA256 = (
    'e87e90fb72f24bd2fecfb54eb758a899f95a3851a0cdd9293b61125193581a6a'
)
VALIDATION_SEGMENT_CALIBRATION_SHA256 = (
    'e86c9cc084a7b0260f2749ee49eed3ba5d4027383ac8e8de75031019abec44d0'
)
EFFECTIVE_CONFIG_FILE_SHA256 = (
    '53f68426ceb0f79fb2c44dbd85302f2d9dad0da364da66959682ae6f3f512371'
)
EFFECTIVE_CONFIG_FINGERPRINT_SHA256 = (
    '719c4c8eaa3a16c98c1346cc6e5d6259c2e8c77d0a325802b72abc13c9e3b523'
)
PUBLIC_TOPOLOGY_CONTRACT_SHA256 = (
    '6d2c4fd5bb42b90c89058094f91d1aba03c63d2db22499a45231b8c46f9fb90d'
)
CANARY_FINAL_REPORT_SHA256 = (
    'fda9a772fac67d36d59544af0299c3aad4fba599c1ea8c0edf39b56f62b42e04'
)
CANARY_COMPLETION_LEDGER_SHA256 = (
    'db43722df617a1a05b884fa3ebd6b35e9d6d87258c2d3cb9f871e70844c2b251'
)
TOPOLOGY_FINGERPRINT_SHA256 = (
    '02ddcd78b1e1f410e7565ab018a9b4f25297ccf2d74c41232e981ebfc9eed344'
)

CANDIDATE_ID = (
    'room315_visual_runtime_candidate_v4_seed31520260811_'
    'epoch11_869d6404_shadow'
)
DEFAULT_OUTPUT = Path('/home/tiago') / CANDIDATE_ID
PROMOTION_MANIFEST_NAME = 'runtime_promotion_manifest.json'
PROMOTION_SCHEMA = 'room315.visual_runtime_promotion.v4.v1'
CHECKPOINT_SCHEMA = 'room315.visual_training.v4.checkpoint.v1'
MODEL_KIND = 'room315_visual_state_resnet18_split_rails_v4'
CALIBRATION_SCHEMA = 'room315.visual_segment_calibration.v4.v1'
TOPOLOGY_SCHEMA = 'room315.public_segment_length_contract.v1'
IDENTITIES = ('L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4')
SEGMENTS = (
    'A12E', 'A12I', 'A14', 'A1E', 'A1I', 'A23', 'A2E', 'A2I',
    'A34E', 'A34I', 'A3E', 'A3I', 'A4E', 'A4I',
)
OUTPUT_KEYS = ('segment_logits', 'loaded_logits', 'bbox', 's_ratio')
VALIDATION_TEMPERATURE = 0.6007370031399095
MINIMUM_SEGMENT_CONFIDENCE = 0.9139089813389109
MINIMUM_LOADED_CONFIDENCE = 0.5
S_RATIO_DIAGNOSTIC_TOLERANCE = 0.05
S_RATIO_ACCEPTANCE_TOLERANCE = 0.12
HEX_SHA256 = re.compile(r'^[0-9a-f]{64}$')


class CandidateV4BuildError(RuntimeError):
    """Raised before a partial V4 candidate can be published."""


@dataclass(frozen=True)
class SourceArtifact:
    source: Path
    expected_sha256: str
    bundle_name: str


@dataclass(frozen=True)
class ValidatedSources:
    checkpoint: Mapping[str, Any]
    training_report: Mapping[str, Any]
    validation_acceptance: Mapping[str, Any]
    validation_calibration: Mapping[str, Any]
    effective_config: Mapping[str, Any]
    topology: Mapping[str, Any]
    canary_report: Mapping[str, Any]
    canary_handoff: Mapping[str, Any]


SOURCE_ARTIFACTS = {
    'checkpoint': SourceArtifact(
        CHECKPOINT, CHECKPOINT_SHA256, 'checkpoint_epoch_011.pt'
    ),
    'training_final_report': SourceArtifact(
        TRAINING_FINAL_REPORT,
        TRAINING_FINAL_REPORT_SHA256,
        'training_final_report.json',
    ),
    'validation_acceptance': SourceArtifact(
        VALIDATION_ACCEPTANCE,
        VALIDATION_ACCEPTANCE_SHA256,
        'validation_acceptance.json',
    ),
    'validation_segment_calibration': SourceArtifact(
        VALIDATION_SEGMENT_CALIBRATION,
        VALIDATION_SEGMENT_CALIBRATION_SHA256,
        'validation_segment_calibration.json',
    ),
    'effective_config': SourceArtifact(
        EFFECTIVE_CONFIG,
        EFFECTIVE_CONFIG_FILE_SHA256,
        'effective_config.json',
    ),
    'public_topology_contract': SourceArtifact(
        PUBLIC_TOPOLOGY_CONTRACT,
        PUBLIC_TOPOLOGY_CONTRACT_SHA256,
        'public_topology_contract.json',
    ),
    'canary_final_report': SourceArtifact(
        CANARY_FINAL_REPORT,
        CANARY_FINAL_REPORT_SHA256,
        'canary_final_report.json',
    ),
    'canary_completion_ledger': SourceArtifact(
        CANARY_COMPLETION_LEDGER,
        CANARY_COMPLETION_LEDGER_SHA256,
        'canary_completion_ledger.json',
    ),
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=True,
        sort_keys=True,
        separators=(',', ':'),
    ).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def load_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CandidateV4BuildError(f'cannot load JSON artifact {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise CandidateV4BuildError(f'JSON artifact root is not an object: {path}')
    return value


def write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise CandidateV4BuildError(message)


def _verify_pinned_files() -> None:
    for name, artifact in SOURCE_ARTIFACTS.items():
        _require(artifact.source.is_file(), f'missing pinned {name}: {artifact.source}')
        _require(
            HEX_SHA256.fullmatch(artifact.expected_sha256) is not None,
            f'invalid pinned SHA-256 for {name}',
        )
        actual = sha256_file(artifact.source)
        _require(
            actual == artifact.expected_sha256,
            f'{name} SHA-256 mismatch: expected '
            f'{artifact.expected_sha256}, got {actual}',
        )


def _strict_load_checkpoint(
    checkpoint: Mapping[str, Any],
    effective_config: Mapping[str, Any],
) -> None:
    try:
        import torch
        import torchvision
    except Exception as exc:
        raise CandidateV4BuildError(
            'Torch and TorchVision are required to validate the V4 checkpoint'
        ) from exc

    script_directory = Path(__file__).resolve().parent
    if str(script_directory) not in sys.path:
        sys.path.insert(0, str(script_directory))
    try:
        from room_315_visual_model_v4 import build_visual_state_model_v4
    except Exception as exc:
        raise CandidateV4BuildError('cannot import the V4 model builder') from exc

    model_config = effective_config.get('model')
    _require(isinstance(model_config, Mapping), 'effective config lacks model contract')
    model = build_visual_state_model_v4(
        torch,
        torchvision,
        head_type=str(model_config.get('head_type')),
        hidden_dim=int(model_config.get('hidden_dim', -1)),
        attention_heads=int(model_config.get('attention_heads', -1)),
        dropout=float(model_config.get('dropout', -1.0)),
    )
    state = checkpoint.get('model_state_dict')
    _require(isinstance(state, Mapping), 'checkpoint lacks model_state_dict')
    try:
        result = model.load_state_dict(state, strict=True)
    except Exception as exc:
        raise CandidateV4BuildError('strict V4 checkpoint loading failed') from exc
    _require(
        not result.missing_keys and not result.unexpected_keys,
        'strict V4 checkpoint load returned incompatible keys',
    )


def _full_coverage_segment_floor(
    calibration: Mapping[str, Any],
) -> float:
    curve = calibration.get('selective_curve')
    _require(isinstance(curve, list), 'validation calibration lacks selective curve')
    full_coverage = [
        row for row in curve
        if isinstance(row, Mapping)
        and float(row.get('requested_coverage', -1.0)) == 1.0
        and float(row.get('achieved_coverage', -1.0)) == 1.0
    ]
    _require(
        len(full_coverage) == 1,
        'validation calibration must contain exactly one full-coverage row',
    )
    threshold = float(full_coverage[0].get('confidence_threshold', -1.0))
    _require(0.0 < threshold <= 1.0, 'invalid full-coverage segment threshold')
    return threshold


def validate_sources() -> ValidatedSources:
    """Verify every pinned input and strictly load the model, without data access."""

    _verify_pinned_files()
    training_report = load_json_object(TRAINING_FINAL_REPORT)
    validation_acceptance = load_json_object(VALIDATION_ACCEPTANCE)
    validation_calibration = load_json_object(VALIDATION_SEGMENT_CALIBRATION)
    effective_config = load_json_object(EFFECTIVE_CONFIG)
    topology = load_json_object(PUBLIC_TOPOLOGY_CONTRACT)
    canary_report = load_json_object(CANARY_FINAL_REPORT)

    _require(
        canonical_sha256(effective_config) == EFFECTIVE_CONFIG_FINGERPRINT_SHA256,
        'effective configuration canonical fingerprint mismatch',
    )
    selected = training_report.get('selected_checkpoint')
    _require(isinstance(selected, Mapping), 'training report lacks selected checkpoint')
    _require(training_report.get('status') == 'completed', 'training run is incomplete')
    _require(
        training_report.get('validation_acceptance_status') == 'passed',
        'validation acceptance did not pass',
    )
    _require(training_report.get('test_loaded') is False, 'training loaded Test data')
    _require(
        training_report.get('canary_used_for_selection') is False,
        'Canary influenced checkpoint selection',
    )
    _require(int(selected.get('epoch', -1)) == 11, 'selected epoch is not 11')
    _require(
        selected.get('sha256') == CHECKPOINT_SHA256,
        'training report does not bind the selected checkpoint SHA-256',
    )
    _require(
        training_report.get('effective_config_sha256')
        == EFFECTIVE_CONFIG_FINGERPRINT_SHA256,
        'training report effective-config fingerprint mismatch',
    )
    _require(
        training_report.get('validation_acceptance') == validation_acceptance,
        'validation acceptance sidecar differs from training report evidence',
    )
    _require(
        training_report.get('validation_segment_calibration')
        == validation_calibration,
        'validation calibration sidecar differs from training report evidence',
    )
    _require(
        training_report.get('public_topology_contract') == topology,
        'public topology sidecar differs from training report evidence',
    )
    _require(
        validation_acceptance.get('accepted') is True
        and validation_acceptance.get('status') == 'passed',
        'validation acceptance artifact is not passed',
    )
    summary = validation_acceptance.get('summary')
    _require(
        isinstance(summary, Mapping)
        and int(summary.get('failed', -1)) == 0
        and int(summary.get('pending', -1)) == 0,
        'validation acceptance contains failed or pending gates',
    )

    _require(
        validation_calibration.get('schema_version') == CALIBRATION_SCHEMA,
        'validation calibration schema mismatch',
    )
    _require(
        validation_calibration.get('data_role') == 'validation'
        and validation_calibration.get('fit_scope') == 'validation_only',
        'segment calibration is not validation-only',
    )
    _require(
        float(validation_calibration.get('temperature', -1.0))
        == VALIDATION_TEMPERATURE,
        'validation calibration temperature mismatch',
    )
    _require(
        _full_coverage_segment_floor(validation_calibration)
        == MINIMUM_SEGMENT_CONFIDENCE,
        'segment runtime threshold is not the validation full-coverage floor',
    )

    _require(topology.get('schema_version') == TOPOLOGY_SCHEMA, 'topology schema mismatch')
    _require(topology.get('authoritative') is True, 'topology is not authoritative')
    _require(
        topology.get('fingerprint_sha256') == TOPOLOGY_FINGERPRINT_SHA256,
        'public topology fingerprint mismatch',
    )
    _require(tuple(topology.get('side_order', ())) == ('left', 'right'), 'side order mismatch')
    _require(tuple(topology.get('segment_order', ())) == SEGMENTS, 'segment order mismatch')

    try:
        import torch
        checkpoint = torch.load(CHECKPOINT, map_location='cpu', weights_only=False)
    except TypeError:
        checkpoint = torch.load(CHECKPOINT, map_location='cpu')
    except Exception as exc:
        raise CandidateV4BuildError(f'cannot load V4 checkpoint: {exc}') from exc
    _require(isinstance(checkpoint, Mapping), 'checkpoint root is not a mapping')
    _require(checkpoint.get('schema_version') == CHECKPOINT_SCHEMA, 'checkpoint schema mismatch')
    _require(checkpoint.get('model_kind') == MODEL_KIND, 'checkpoint model kind mismatch')
    _require(int(checkpoint.get('epoch', -1)) == 11, 'checkpoint epoch mismatch')
    _require(tuple(checkpoint.get('slot_order', ())) == IDENTITIES, 'checkpoint slot order mismatch')
    _require(tuple(checkpoint.get('segment_order', ())) == SEGMENTS, 'checkpoint segment order mismatch')
    _require(checkpoint.get('canary_seen') is False, 'checkpoint has seen Canary data')
    _require(checkpoint.get('test_seen') is False, 'checkpoint has seen Test data')
    _require(
        checkpoint.get('checkpoint_selection_role') == 'validation_only',
        'checkpoint was not selected on validation only',
    )
    _require(
        checkpoint.get('effective_config_sha256')
        == EFFECTIVE_CONFIG_FINGERPRINT_SHA256,
        'checkpoint effective-config fingerprint mismatch',
    )
    _require(
        checkpoint.get('topology_length_mapping_fingerprint_sha256')
        == TOPOLOGY_FINGERPRINT_SHA256,
        'checkpoint topology fingerprint mismatch',
    )
    _strict_load_checkpoint(checkpoint, effective_config)

    _require(
        canary_report.get('checkpoint', {}).get('sha256') == CHECKPOINT_SHA256,
        'Canary report checkpoint SHA-256 mismatch',
    )
    _require(
        canary_report.get('acceptance_status') == 'passed'
        and canary_report.get('promotion_status')
        == 'eligible_for_manual_runtime_review',
        'Canary report is not eligible for manual runtime review',
    )
    _require(canary_report.get('training_performed') is False, 'Canary retrained the model')
    _require(
        canary_report.get('checkpoint_selection_performed') is False
        and canary_report.get('canary_used_for_checkpoint_selection') is False,
        'Canary influenced checkpoint selection',
    )
    _require(canary_report.get('test_loaded') is False, 'Canary loaded Test data')
    _require(
        canary_report.get('automatic_runtime_switch') is False,
        'Canary automatically switched runtime',
    )

    script_directory = Path(__file__).resolve().parent
    if str(script_directory) not in sys.path:
        sys.path.insert(0, str(script_directory))
    try:
        from room_315_visual_state_train_v4 import verify_completed_canary_handoff

        handoff = verify_completed_canary_handoff(CANARY_FINAL_REPORT)
    except Exception as exc:
        raise CandidateV4BuildError(
            f'trusted Canary completion handoff verification failed: {exc}'
        ) from exc
    _require(handoff.get('trusted') is True, 'Canary handoff is not trusted')
    _require(
        handoff.get('attempt_key') == CANARY_ATTEMPT_KEY,
        'Canary attempt key mismatch',
    )
    _require(
        handoff.get('completion_ledger', {}).get('sha256')
        == CANARY_COMPLETION_LEDGER_SHA256,
        'Canary handoff completion-ledger SHA-256 mismatch',
    )

    return ValidatedSources(
        checkpoint=checkpoint,
        training_report=training_report,
        validation_acceptance=validation_acceptance,
        validation_calibration=validation_calibration,
        effective_config=effective_config,
        topology=topology,
        canary_report=canary_report,
        canary_handoff=handoff,
    )


def acceptance_scenarios_v4() -> dict[str, Any]:
    """Return the existing seven runtime scenarios bound to this candidate."""

    script_directory = Path(__file__).resolve().parent
    if str(script_directory) not in sys.path:
        sys.path.insert(0, str(script_directory))
    from room_315_build_runtime_candidate import acceptance_scenarios

    scenarios = copy.deepcopy(acceptance_scenarios())
    scenarios['candidate_id'] = CANDIDATE_ID
    scenarios['runtime_candidate'] = {
        'runtime_generation': 'v4',
        'runtime_mode': 'shadow',
        'automatic_promotion_allowed': False,
    }
    return scenarios


def runtime_policy(validated: ValidatedSources) -> dict[str, Any]:
    metrics = validated.training_report['final_validation_metrics']
    return {
        'presence_contract': {
            'source': 'authoritative_runtime_presence_provider',
            'threshold_scope': 'present_slots_only',
            'absent_slots': 'masked_and_never_accepted',
            'missing_or_stale_presence': 'reject_frame',
        },
        'minimum_segment_confidence': MINIMUM_SEGMENT_CONFIDENCE,
        'segment_confidence': {
            'calibrated': True,
            'temperature': VALIDATION_TEMPERATURE,
            'derivation': 'validation_selective_curve_100_percent_coverage_floor',
            'requested_coverage': 1.0,
        },
        'minimum_loaded_confidence': MINIMUM_LOADED_CONFIDENCE,
        'loaded_confidence': {
            'calibrated': False,
            'interpretation': 'uncalibrated_softmax_argmax_decision_floor',
            'probability_guarantee': False,
            'derivation': 'binary_argmax_floor; validation loaded accuracy is diagnostic',
            'validation_loaded_accuracy': float(metrics['loaded_accuracy']),
        },
        'active_frame_policy': 'all_present_slots_must_pass',
        's_ratio': {
            'acceptance_absolute_error_tolerance': S_RATIO_ACCEPTANCE_TOLERANCE,
            'diagnostic_absolute_error_tolerance': S_RATIO_DIAGNOSTIC_TOLERANCE,
            'threshold_source': 'validation_only_task_contract',
            'validation_joint_accuracy_at_diagnostic_tolerance': float(
                metrics['joint_localization_accuracy_s_ratio_0_05']
            ),
            'validation_joint_accuracy_at_acceptance_tolerance': float(
                metrics['joint_localization_accuracy_s_ratio_0_12']
            ),
            'runtime_ground_truth_error_available': False,
            'confidence_gate': False,
        },
    }


def promotion_manifest(validated: ValidatedSources) -> dict[str, Any]:
    policy = runtime_policy(validated)
    artifact_entries = {
        name: {
            'path': artifact.bundle_name,
            'sha256': artifact.expected_sha256,
        }
        for name, artifact in SOURCE_ARTIFACTS.items()
    }
    model_config = validated.effective_config['model']
    preprocessing = validated.effective_config['image_preprocessing']
    return {
        'schema_version': PROMOTION_SCHEMA,
        'candidate_id': CANDIDATE_ID,
        'immutable': True,
        'manual_review_approved': False,
        'manual_runtime_review_status': 'pending',
        'shadow_execution_authorized': True,
        'deployment_mode': 'shadow',
        'automatic_promotion_allowed': False,
        'manual_only': True,
        'eligibility': {
            'shadow_load_and_evaluation': True,
            'active_runtime_review': False,
            'active_runtime_selected': False,
            'active_transition_requires_new_immutable_manifest': True,
        },
        'artifacts': artifact_entries,
        'model_contract': {
            'checkpoint_schema_version': CHECKPOINT_SCHEMA,
            'model_kind': MODEL_KIND,
            'head_type': str(model_config['head_type']),
            'hidden_dim': int(model_config['hidden_dim']),
            'attention_heads': int(model_config['attention_heads']),
            'dropout': float(model_config['dropout']),
            'slot_order': list(IDENTITIES),
            'segment_order': list(SEGMENTS),
            'output_keys': list(OUTPUT_KEYS),
            'checkpoint_epoch': 11,
            'checkpoint_loading': 'strict',
            'effective_config_fingerprint_sha256': (
                EFFECTIVE_CONFIG_FINGERPRINT_SHA256
            ),
            'cross_camera_feature_path': False,
            'side_source': 'fixed_identity_prefix',
        },
        'preprocessing_contract': {
            'camera_order': ['left_rail_rgb', 'right_rail_rgb'],
            'channel_order': ['left_rgb', 'right_rgb'],
            'input_shape': [
                'B', 6, int(preprocessing['height']), int(preprocessing['width'])
            ],
            'width': int(preprocessing['width']),
            'height': int(preprocessing['height']),
            'resize': str(preprocessing['resize']),
            'resampling': 'bilinear',
            'normalization_mean': list(preprocessing['normalization_mean']),
            'normalization_std': list(preprocessing['normalization_std']),
            'runtime_augmentations': False,
        },
        'topology_contract': {
            'schema_version': TOPOLOGY_SCHEMA,
            'source': 'room_315_rail_defaults.public_rail_segment_lengths',
            'segment_name_domain': 'public_ros_label',
            'fingerprint_sha256': TOPOLOGY_FINGERPRINT_SHA256,
            'side_order': ['left', 'right'],
            'segment_order': list(SEGMENTS),
        },
        'calibration_contract': {
            'schema_version': CALIBRATION_SCHEMA,
            'temperature': VALIDATION_TEMPERATURE,
            'fit_scope': 'validation_only',
            'data_role': 'validation',
        },
        'acceptance_thresholds': policy,
        'evidence_policy': {
            'checkpoint_selection': 'validation_only',
            'canary_role': 'post_selection_development_regression',
            'canary_handoff_trusted': bool(validated.canary_handoff['trusted']),
            'canary_attempt_key': CANARY_ATTEMPT_KEY,
            'test_loaded': False,
            'datasets_bundled': False,
            'training_or_evaluation_during_build': False,
        },
    }


def _runtime_yaml(output: Path, manifest_sha256: str) -> str:
    return f"""room_315_visual_state_inference_node:
  ros__parameters:
    use_sim_time: true
    runtime_generation: v4
    runtime_mode: shadow
    v4_promotion_manifest_path: {output / PROMOTION_MANIFEST_NAME}
    expected_v4_promotion_manifest_sha256: {manifest_sha256}
    device: auto
    presence_state_timeout_s: 1.0
    presence_warmup_s: 0.5
    dry_run_state_fusion: true
    plansys2_update_enabled: false
    raw_observation_topic: /room_315/visual_state/shadow_v4/raw
    raw_model_prediction_topic: /room_315/visual_state/shadow_v4/raw_model_prediction
    validation_topic: /room_315/visual_state/shadow_v4/validation
    accepted_observed_state_topic: /room_315/visual_state/shadow_v4/observed_state
"""


def _readme(output: Path, manifest_sha256: str) -> str:
    return f"""# Room 315 V4 immutable shadow candidate

Candidate ID: `{CANDIDATE_ID}`

This host bundle is authorized for **shadow loading and evaluation only**. It
does not select an active runtime, permits no automatic promotion, and an
active transition requires a new immutable manifest after runtime review.

The selected V4 epoch-11 checkpoint is strictly loaded during candidate build.
Its segment confidence uses the validation-only temperature
`{VALIDATION_TEMPERATURE}` and the 100%-coverage validation floor
`{MINIMUM_SEGMENT_CONFIDENCE}`. Loaded confidence is not calibrated: `0.5` is
only the binary argmax decision floor. The `0.05` ratio tolerance is diagnostic;
the task acceptance tolerance remains `0.12`.

The package copies exact validation and trusted one-shot Canary evidence. It
contains no dataset, no image, and no Test artifact. The Canary is a previously
exposed development regression set, not a pristine final holdout.

Promotion manifest SHA-256: `{manifest_sha256}`

V4 ROS parameters: `{output / 'runtime_ros_parameters.yaml'}`

Seven existing Gazebo acceptance configurations are preserved in
`acceptance_scenarios.json`, with this candidate ID. They are not automatic
approval and should initially be observed without actuation.

The bundle contains only the V4 candidate and its review inputs. `SHA256SUMS`
binds every payload file except itself. The directory is published atomically
and permission-hardened read-only.
"""


def build(output: Path) -> dict[str, Any]:
    output = output.expanduser().resolve()
    if output.exists():
        raise CandidateV4BuildError(f'refusing to overwrite candidate: {output}')

    # All expensive and trust-bearing validation occurs before staging exists.
    validated = validate_sources()
    manifest = promotion_manifest(validated)

    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(prefix=f'.{output.name}.staging-', dir=output.parent)
    )
    try:
        for artifact in SOURCE_ARTIFACTS.values():
            shutil.copyfile(artifact.source, staging / artifact.bundle_name)

        write_json(staging / 'acceptance_scenarios.json', acceptance_scenarios_v4())
        write_json(staging / 'candidate_state.json', {
            'schema_version': 'room315.deployment_candidate_state.v4.v1',
            'candidate_id': CANDIDATE_ID,
            'state': 'shadow_authorized_pending_manual_review',
            'deployment_mode': 'shadow',
            'manual_review_approved': False,
            'manual_runtime_review_status': 'pending',
            'shadow_execution_authorized': True,
            'automatic_promotion_allowed': False,
            'active_runtime_selected': False,
            'active_transition_requires_new_immutable_manifest': True,
            'checkpoint_sha256': CHECKPOINT_SHA256,
            'checkpoint_filename': 'checkpoint_epoch_011.pt',
            'validation_acceptance': 'passed',
            'trusted_canary_acceptance': 'passed',
            'gazebo_acceptance_execution_status': 'not_run',
            'runtime_topics': {
                'raw_prediction': (
                    '/room_315/visual_state/shadow_v4/raw_model_prediction'
                ),
                'raw_observation': '/room_315/visual_state/shadow_v4/raw',
                'validation': '/room_315/visual_state/shadow_v4/validation',
                'accepted_observed_state': (
                    '/room_315/visual_state/shadow_v4/observed_state'
                ),
            },
        })
        write_json(staging / PROMOTION_MANIFEST_NAME, manifest)
        manifest_sha256 = sha256_file(staging / PROMOTION_MANIFEST_NAME)
        (staging / 'runtime_ros_parameters.yaml').write_text(
            _runtime_yaml(output, manifest_sha256), encoding='utf-8'
        )
        (staging / 'README.md').write_text(
            _readme(output, manifest_sha256), encoding='utf-8'
        )

        # Recheck every copied evidence byte before sealing the package.
        for artifact in SOURCE_ARTIFACTS.values():
            copied_hash = sha256_file(staging / artifact.bundle_name)
            _require(
                copied_hash == artifact.expected_sha256,
                f'copied artifact changed: {artifact.bundle_name}',
            )
        _require(
            sha256_file(staging / PROMOTION_MANIFEST_NAME) == manifest_sha256,
            'promotion manifest changed while staging',
        )

        sum_files = sorted(
            path for path in staging.iterdir()
            if path.is_file() and path.name != 'SHA256SUMS'
        )
        (staging / 'SHA256SUMS').write_text(
            ''.join(f'{sha256_file(path)}  {path.name}\n' for path in sum_files),
            encoding='utf-8',
        )
        for path in staging.iterdir():
            _require(path.is_file(), f'unexpected non-file candidate payload: {path}')
            path.chmod(0o444)

        os.replace(staging, output)
        output.chmod(0o555)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise

    result = {
        'status': 'CANDIDATE_CREATED',
        'candidate': str(output),
        'candidate_id': CANDIDATE_ID,
        'deployment_mode': 'shadow',
        'checkpoint_sha256': CHECKPOINT_SHA256,
        'promotion_manifest': str(output / PROMOTION_MANIFEST_NAME),
        'promotion_manifest_sha256': manifest_sha256,
        'automatic_promotion_allowed': False,
    }
    print(json.dumps(result, sort_keys=True))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Build the pinned immutable Room 315 V4 shadow candidate.'
    )
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    build(arguments.output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
