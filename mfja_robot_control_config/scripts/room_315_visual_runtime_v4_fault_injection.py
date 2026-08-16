#!/usr/bin/env python3
"""Deterministic V4 fault-injection verification.

This utility deliberately never opens a training, validation, Canary, or Test
dataset. ``faults`` verifies a real immutable V4 shadow candidate and then
injects eight metadata, input, and presence faults into temporary data only.
Every fault must be rejected before a PlanSys2 predicate or actuation command
can be emitted.

Reports are atomically published with mode 0444. Their embedded integrity
digest covers canonical JSON with the ``integrity`` field omitted.
"""

from __future__ import annotations

import argparse
import ast
import copy
import hashlib
import json
import os
import re
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np
import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_presence_provider import (  # noqa: E402
    PRESENCE_ABSENT,
    PRESENCE_PRESENT,
    PresenceEntry,
    PresenceSnapshot,
    ShuttleStatePresenceProvider,
)
from room_315_visual_runtime_fusion import (  # noqa: E402
    DeterministicPlanSys2FactGate,
    fuse_validated_visual_state,
)
from room_315_visual_runtime_validation import (  # noqa: E402
    ValidationConfig,
    validate_prediction,
)
from room_315_visual_runtime_v4 import (  # noqa: E402
    StructuredVisualOutputV4,
    VisualRuntimeV4Error,
    decode_active_slots_v4,
    verify_v4_runtime_promotion,
)
from room_315_visual_model_v4 import V4_SLOT_ORDER  # noqa: E402


FAULT_REPORT_SCHEMA = 'room315.visual_runtime_v4.fault_injection.v1'
V4_MODEL_SCHEMA = 'room315.visual_state.v4'
EXPECTED_FAULT_NAMES = (
    'manifest_hash_mismatch',
    'checkpoint_hash_mismatch',
    'topology_fingerprint_mismatch',
    'camera_order_mismatch',
    'stale_left_image',
    'stale_right_image',
    'stale_presence',
    'unknown_identity',
)
SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')

DEFAULT_CANDIDATE_DIRECTORY = Path(
    '/home/tiago/room315_visual_runtime_candidate_v4_seed31520260811_'
    'epoch11_869d6404_shadow'
)


class FaultInjectionVerificationError(RuntimeError):
    """Raised when a requested verification cannot be completed safely."""


def sha256_file(path: Path | str) -> str:
    candidate = Path(path)
    digest = hashlib.sha256()
    try:
        with candidate.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
    except OSError as exc:
        raise FaultInjectionVerificationError(
            f'cannot read file for SHA-256: {candidate}'
        ) from exc
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=True,
        sort_keys=True,
        separators=(',', ':'),
    ).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def write_immutable_report(path: Path | str, report: Mapping[str, Any]) -> str:
    """Atomically publish one new report without replacing existing evidence."""

    destination = Path(path).expanduser().resolve()
    if destination.exists():
        raise FaultInjectionVerificationError(
            f'refusing to replace existing immutable report: {destination}'
        )
    destination.parent.mkdir(parents=True, exist_ok=True)
    payload = copy.deepcopy(dict(report))
    payload.pop('integrity', None)
    payload['integrity'] = {
        'algorithm': 'sha256',
        'scope': 'canonical_json_excluding_integrity',
        'sha256': canonical_sha256(payload),
    }
    serialized = (
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + '\n'
    ).encode('utf-8')

    temporary_path: Path | None = None
    try:
        descriptor, raw_temporary_path = tempfile.mkstemp(
            prefix=f'.{destination.name}.',
            suffix='.tmp',
            dir=str(destination.parent),
        )
        temporary_path = Path(raw_temporary_path)
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(serialized)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary_path, 0o444)
        # A hard link is an atomic no-clobber publication on the same file
        # system.  It fails if another process published the target first.
        os.link(temporary_path, destination)
        temporary_path.unlink()
        temporary_path = None
    except FileExistsError as exc:
        raise FaultInjectionVerificationError(
            f'refusing to replace existing immutable report: {destination}'
        ) from exc
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
    return hashlib.sha256(serialized).hexdigest()


def run_fault_verification(
    *,
    candidate_directory: Path | str,
    expected_manifest_sha256: str,
) -> dict[str, Any]:
    """Run the exact eight fail-closed checks against a real V4 candidate."""

    candidate = Path(candidate_directory).expanduser().resolve()
    manifest_path = candidate / 'runtime_promotion_manifest.json'
    expected_sha = _required_sha256(
        expected_manifest_sha256,
        'expected V4 promotion manifest SHA-256',
    )
    started = _utc_now()
    report: dict[str, Any] = {
        'schema_version': FAULT_REPORT_SCHEMA,
        'immutable': True,
        'command': 'faults',
        'created_at_utc': started,
        'candidate': {
            'directory': str(candidate),
            'promotion_manifest_path': str(manifest_path),
            'expected_promotion_manifest_sha256': expected_sha,
            'actual_promotion_manifest_sha256': (
                sha256_file(manifest_path) if manifest_path.is_file() else ''
            ),
        },
        'data_access': {
            'datasets_opened': False,
            'training_split_opened': False,
            'validation_split_opened': False,
            'canary_split_opened': False,
            'test_split_opened': False,
            'note': 'only promotion artifacts, runtime configuration, and source contracts were read',
        },
        'expected_faults': list(EXPECTED_FAULT_NAMES),
        'checks': [],
    }

    try:
        if not candidate.is_dir():
            raise FaultInjectionVerificationError(
                f'V4 candidate directory is missing: {candidate}'
            )
        if not manifest_path.is_file():
            raise FaultInjectionVerificationError(
                f'V4 promotion manifest is missing: {manifest_path}'
            )
        promotion = verify_v4_runtime_promotion(manifest_path, expected_sha)
        if promotion.deployment_mode != 'shadow':
            raise FaultInjectionVerificationError(
                'fault injection is permitted only for a shadow promotion'
            )
        safety = _verify_shadow_safety(candidate, expected_sha)
        report['candidate'].update({
            'candidate_id': _load_json_object(manifest_path).get('candidate_id', ''),
            'deployment_mode': promotion.deployment_mode,
            'model_schema_version': V4_MODEL_SCHEMA,
            'checkpoint_sha256': promotion.checkpoint_sha256,
            'baseline_verified': True,
        })
        report['shadow_safety'] = safety
        manifest_checks = _run_manifest_faults(
            manifest_path=manifest_path,
            expected_manifest_sha256=expected_sha,
        )
        input_checks = _run_input_and_presence_faults(promotion)
        report['checks'] = [*manifest_checks, *input_checks]
    except Exception as exc:  # noqa: BLE001 - report the fail-closed result
        report['candidate']['baseline_verified'] = False
        report['fatal_error'] = {
            'type': type(exc).__name__,
            'message': str(exc),
        }
        completed = {str(item.get('name')) for item in report['checks']}
        for name in EXPECTED_FAULT_NAMES:
            if name not in completed:
                report['checks'].append(_failed_check(
                    name,
                    f'not executed because baseline verification failed: {exc}',
                ))

    names = tuple(str(item.get('name')) for item in report['checks'])
    all_passed = (
        names == EXPECTED_FAULT_NAMES
        and bool(report['candidate'].get('baseline_verified'))
        and all(bool(item.get('passed')) for item in report['checks'])
        and bool(report.get('shadow_safety', {}).get('passed'))
    )
    report['summary'] = {
        'expected_fault_count': len(EXPECTED_FAULT_NAMES),
        'executed_fault_count': len(report['checks']),
        'passed_fault_count': sum(
            bool(item.get('passed')) for item in report['checks']
        ),
        'failed_fault_count': sum(
            not bool(item.get('passed')) for item in report['checks']
        ),
        'every_fault_failed_closed': all(
            bool(item.get('fail_closed')) for item in report['checks']
        ),
        'plan_client_present': False,
        'plansys2_update_attempt_count': 0,
        'plansys2_predicates_added': 0,
        'plansys2_predicates_removed': 0,
        'actuation_command_count': 0,
    }
    report['overall_passed'] = all_passed
    report['status'] = 'passed' if all_passed else 'failed'
    _apply_fault_promotion_contract(report)
    report['completed_at_utc'] = _utc_now()
    return report


def _run_manifest_faults(
    *,
    manifest_path: Path,
    expected_manifest_sha256: str,
) -> list[dict[str, Any]]:
    baseline = _load_json_object(manifest_path)
    absolute_baseline = copy.deepcopy(baseline)
    artifact_entries = absolute_baseline.get('artifacts')
    if not isinstance(artifact_entries, dict):
        raise FaultInjectionVerificationError(
            'promotion manifest artifacts must be an object'
        )
    for value in artifact_entries.values():
        if not isinstance(value, dict):
            raise FaultInjectionVerificationError(
                'promotion artifact entry must be an object'
            )
        raw_path = Path(str(value.get('path') or '')).expanduser()
        if not raw_path.is_absolute():
            raw_path = manifest_path.parent / raw_path
        value['path'] = str(raw_path.resolve())

    checks = [
        _expected_exception_check(
            name='manifest_hash_mismatch',
            detection_layer='v4_promotion_manifest_digest',
            injection='expected manifest SHA-256 changed in memory',
            expected_message='promotion manifest SHA-256 mismatch',
            action=lambda: verify_v4_runtime_promotion(
                manifest_path,
                _different_sha256(expected_manifest_sha256),
            ),
        ),
    ]
    mutators: tuple[
        tuple[str, str, str, Callable[[dict[str, Any]], None]], ...
    ] = (
        (
            'checkpoint_hash_mismatch',
            'v4_artifact_digest',
            'temporary manifest pins a wrong checkpoint digest',
            lambda manifest: manifest['artifacts']['checkpoint'].__setitem__(
                'sha256',
                _different_sha256(
                    str(manifest['artifacts']['checkpoint']['sha256'])
                ),
            ),
        ),
        (
            'topology_fingerprint_mismatch',
            'v4_topology_contract',
            'temporary manifest pins a wrong public-topology fingerprint',
            lambda manifest: manifest['topology_contract'].__setitem__(
                'fingerprint_sha256',
                _different_sha256(
                    str(manifest['topology_contract']['fingerprint_sha256'])
                ),
            ),
        ),
        (
            'camera_order_mismatch',
            'v4_preprocessing_contract',
            'temporary manifest reverses the fixed left/right camera order',
            lambda manifest: manifest['preprocessing_contract'].__setitem__(
                'camera_order',
                ['right_rail_rgb', 'left_rail_rgb'],
            ),
        ),
    )
    expected_message = {
        'checkpoint_hash_mismatch': 'artifact SHA-256 mismatch for checkpoint',
        'topology_fingerprint_mismatch': 'topology artifact fingerprint mismatch',
        'camera_order_mismatch': 'preprocessing contract is incompatible',
    }
    with tempfile.TemporaryDirectory(prefix='room315_v4_fault_manifests_') as raw:
        temporary_directory = Path(raw)
        for name, layer, injection, mutate in mutators:
            mutated = copy.deepcopy(absolute_baseline)
            mutate(mutated)
            temporary_manifest = temporary_directory / f'{name}.json'
            temporary_manifest.write_text(
                json.dumps(mutated, ensure_ascii=False, indent=2, sort_keys=True)
                + '\n',
                encoding='utf-8',
            )
            temporary_sha = sha256_file(temporary_manifest)
            checks.append(_expected_exception_check(
                name=name,
                detection_layer=layer,
                injection=injection,
                expected_message=expected_message[name],
                action=lambda path=temporary_manifest, digest=temporary_sha:
                    verify_v4_runtime_promotion(path, digest),
            ))
    return checks


def _run_input_and_presence_faults(promotion: Any) -> list[dict[str, Any]]:
    output = _accepted_synthetic_v4_output()
    fresh_presence = _presence_snapshot(('L1', 'R1'), timestamp_s=10.0)
    valid_prediction = decode_active_slots_v4(
        output,
        promotion=promotion,
        presence=fresh_presence,
        timestamp_s=10.0,
        left_image_stamp_s=10.0,
        right_image_stamp_s=10.0,
        left_image_size=(640, 480),
        right_image_size=(640, 480),
    )
    validator = ValidationConfig(
        stale_image_timeout_s=1.0,
        maximum_timestamp_difference_s=5.0,
        reconcile_position_consistency=True,
        position_reconciliation_policy='canonical_s_m',
    )
    baseline_validation = validate_prediction(
        valid_prediction,
        fresh_presence,
        now_s=10.0,
        config=validator,
    )
    if not baseline_validation.accepted:
        raise FaultInjectionVerificationError(
            'synthetic V4 validation baseline was rejected: '
            + ','.join(baseline_validation.reasons)
        )

    checks = []
    for name, left_stamp, right_stamp, expected_reason in (
        ('stale_left_image', 8.0, 10.0, 'left_image_is_stale'),
        ('stale_right_image', 10.0, 8.0, 'right_image_is_stale'),
    ):
        prediction = decode_active_slots_v4(
            output,
            promotion=promotion,
            presence=fresh_presence,
            timestamp_s=10.0,
            left_image_stamp_s=left_stamp,
            right_image_stamp_s=right_stamp,
            left_image_size=(640, 480),
            right_image_size=(640, 480),
        )
        validation = validate_prediction(
            prediction,
            fresh_presence,
            now_s=10.0,
            config=validator,
        )
        checks.append(_validation_rejection_check(
            name=name,
            detection_layer='node_prediction_validation',
            injection=f'{name.removeprefix("stale_")} timestamp is 2.0 s old',
            validation=validation,
            expected_reason=expected_reason,
            presence=fresh_presence,
            checkpoint_sha256=promotion.checkpoint_sha256,
        ))

    stale_provider = ShuttleStatePresenceProvider(timeout_s=1.0, warmup_s=0.0)
    for side in ('left', 'right'):
        stale_provider.observe(
            topic_side=side,
            entity_name='',
            source_stamp_s=10.0,
            receive_time_s=10.0,
        )
    stale_presence = stale_provider.snapshot(now_s=12.0)
    checks.append(_presence_rejection_check(
        name='stale_presence',
        detection_layer='presence_provider+v4_decode+node_prediction_validation',
        injection='both initialized presence sources are 2.0 s old',
        expected_reason='presence_source_stale',
        presence=stale_presence,
        valid_prediction=valid_prediction,
        output=output,
        promotion=promotion,
        validator=validator,
    ))

    unknown_provider = ShuttleStatePresenceProvider(timeout_s=1.0, warmup_s=0.0)
    for side in ('left', 'right'):
        unknown_provider.observe(
            topic_side=side,
            entity_name='',
            source_stamp_s=10.0,
            receive_time_s=10.0,
        )
    unknown_provider.observe(
        topic_side='left',
        entity_name='room315_unregistered_shuttle_99',
        source_stamp_s=10.0,
        receive_time_s=10.0,
    )
    unknown_presence = unknown_provider.snapshot(now_s=10.0)
    checks.append(_presence_rejection_check(
        name='unknown_identity',
        detection_layer='presence_provider+v4_decode+node_prediction_validation',
        injection='unregistered non-empty entity on the left presence source',
        expected_reason='unknown_presence_entity',
        presence=unknown_presence,
        valid_prediction=valid_prediction,
        output=output,
        promotion=promotion,
        validator=validator,
    ))
    return checks


def _expected_exception_check(
    *,
    name: str,
    detection_layer: str,
    injection: str,
    expected_message: str,
    action: Callable[[], Any],
) -> dict[str, Any]:
    try:
        action()
    except Exception as exc:  # noqa: BLE001 - exact rejection is evidence
        observed = str(exc)
        passed = expected_message.lower() in observed.lower()
        return _check_result(
            name=name,
            passed=passed,
            detection_layer=detection_layer,
            injection=injection,
            expected_rejection=expected_message,
            observed_rejection=observed,
            error_type=type(exc).__name__,
        )
    return _check_result(
        name=name,
        passed=False,
        detection_layer=detection_layer,
        injection=injection,
        expected_rejection=expected_message,
        observed_rejection='no exception; corrupted input was accepted',
        error_type='',
    )


def _validation_rejection_check(
    *,
    name: str,
    detection_layer: str,
    injection: str,
    validation: Any,
    expected_reason: str,
    presence: PresenceSnapshot,
    checkpoint_sha256: str,
) -> dict[str, Any]:
    reasons = tuple(str(value) for value in validation.reasons)
    reason_found = any(expected_reason in reason for reason in reasons)
    plan_safety = _zero_plansys_result(
        validation,
        presence,
        checkpoint_sha256=checkpoint_sha256,
        schema_version=V4_MODEL_SCHEMA,
    )
    passed = not validation.accepted and reason_found and plan_safety['passed']
    return _check_result(
        name=name,
        passed=passed,
        detection_layer=detection_layer,
        injection=injection,
        expected_rejection=expected_reason,
        observed_rejection=','.join(reasons),
        validation_reasons=list(reasons),
        plansys_evidence=plan_safety,
    )


def _presence_rejection_check(
    *,
    name: str,
    detection_layer: str,
    injection: str,
    expected_reason: str,
    presence: PresenceSnapshot,
    valid_prediction: Any,
    output: StructuredVisualOutputV4,
    promotion: Any,
    validator: ValidationConfig,
) -> dict[str, Any]:
    core_error = ''
    try:
        decode_active_slots_v4(
            output,
            promotion=promotion,
            presence=presence,
            timestamp_s=10.0,
            left_image_stamp_s=10.0,
            right_image_stamp_s=10.0,
            left_image_size=(640, 480),
            right_image_size=(640, 480),
        )
    except VisualRuntimeV4Error as exc:
        core_error = str(exc)
    validation = validate_prediction(
        valid_prediction,
        presence,
        now_s=float(presence.timestamp_s),
        config=validator,
    )
    observed_reasons = tuple(str(value) for value in presence.reasons)
    expected_found = any(expected_reason in value for value in observed_reasons)
    plan_safety = _zero_plansys_result(
        validation,
        presence,
        checkpoint_sha256=promotion.checkpoint_sha256,
        schema_version=V4_MODEL_SCHEMA,
    )
    passed = bool(
        not presence.ready
        and core_error
        and not validation.accepted
        and expected_found
        and plan_safety['passed']
    )
    return _check_result(
        name=name,
        passed=passed,
        detection_layer=detection_layer,
        injection=injection,
        expected_rejection=expected_reason,
        observed_rejection=core_error or 'V4 decode unexpectedly accepted presence',
        presence_reasons=list(observed_reasons),
        validation_reasons=list(validation.reasons),
        plansys_evidence=plan_safety,
    )


def _check_result(
    *,
    name: str,
    passed: bool,
    detection_layer: str,
    injection: str,
    expected_rejection: str,
    observed_rejection: str,
    **evidence: Any,
) -> dict[str, Any]:
    return {
        'name': name,
        'case_id': name,
        'status': 'passed' if passed else 'failed',
        'passed': bool(passed),
        'fail_closed': bool(passed),
        'rejected': bool(passed),
        'detection_layer': detection_layer,
        'injection': injection,
        'expected_rejection': expected_rejection,
        'observed_rejection': observed_rejection,
        'safety_effects': {
            'shadow_plan_client_present': False,
            'plansys2_update_attempt_count': 0,
            'plansys2_predicates_added': 0,
            'plansys2_predicates_removed': 0,
            'actuation_command_count': 0,
        },
        **evidence,
    }


def _apply_fault_promotion_contract(report: dict[str, Any]) -> None:
    """Expose the exact top-level handoff consumed by V4 promotion."""

    candidate = report.get('candidate')
    if not isinstance(candidate, Mapping):
        candidate = {}
    checks = report.get('checks')
    if not isinstance(checks, list):
        checks = []
    report['candidate_id'] = str(candidate.get('candidate_id') or '')
    report['checkpoint_sha256'] = str(
        candidate.get('checkpoint_sha256') or ''
    )
    report['all_faults_rejected'] = bool(
        report.get('overall_passed') is True
        and tuple(str(row.get('case_id') or '') for row in checks)
        == EXPECTED_FAULT_NAMES
        and all(
            row.get('passed') is True and row.get('rejected') is True
            for row in checks
        )
    )
    report['plansys2_mutation_count'] = 0
    report['actuation_command_count'] = 0
    # ``cases`` is the canonical promotion input.  ``checks`` remains as the
    # detailed audit view, and every row carries both naming conventions.
    report['cases'] = copy.deepcopy(checks)


def _failed_check(name: str, message: str) -> dict[str, Any]:
    return _check_result(
        name=name,
        passed=False,
        detection_layer='baseline_precondition',
        injection='not executed',
        expected_rejection='fault must be rejected fail-closed',
        observed_rejection=message,
    )


def _zero_plansys_result(
    validation: Any,
    presence: PresenceSnapshot,
    *,
    checkpoint_sha256: str,
    schema_version: str,
) -> dict[str, Any]:
    fusion = fuse_validated_visual_state(
        validation,
        presence,
        checkpoint_sha256=checkpoint_sha256,
        schema_version=schema_version,
        stale_after_s=1.0,
        state_id='fault-injection',
    )
    update = DeterministicPlanSys2FactGate().build_update(
        fusion,
        model_ready=True,
        input_ready=False,
        safety_ready=False,
        enabled=False,
    )
    passed = bool(
        not update.accepted
        and update.add_predicates == ()
        and update.remove_predicates == ()
        and 'plansys2_updates_disabled' in update.reasons
    )
    return {
        'passed': passed,
        'update_accepted': update.accepted,
        'reasons': list(update.reasons),
        'add_predicates': list(update.add_predicates),
        'remove_predicates': list(update.remove_predicates),
    }


def _verify_shadow_safety(
    candidate: Path,
    expected_manifest_sha256: str,
) -> dict[str, Any]:
    runtime_parameters_path = candidate / 'runtime_ros_parameters.yaml'
    candidate_state_path = candidate / 'candidate_state.json'
    runtime_parameters = _ros_parameters(
        runtime_parameters_path,
        'room_315_visual_state_inference_node',
    )
    candidate_state = _load_json_object(candidate_state_path)
    expected_topics = {
        'raw_observation_topic': '/room_315/visual_state/shadow_v4/raw',
        'raw_model_prediction_topic': (
            '/room_315/visual_state/shadow_v4/raw_model_prediction'
        ),
        'validation_topic': '/room_315/visual_state/shadow_v4/validation',
        'accepted_observed_state_topic': (
            '/room_315/visual_state/shadow_v4/observed_state'
        ),
    }
    configuration_checks = {
        'runtime_generation_v4': runtime_parameters.get('runtime_generation') == 'v4',
        'runtime_mode_shadow': runtime_parameters.get('runtime_mode') == 'shadow',
        'dry_run_enabled': runtime_parameters.get('dry_run_state_fusion') is True,
        'plansys2_disabled': runtime_parameters.get('plansys2_update_enabled') is False,
        'manifest_sha_exact': (
            runtime_parameters.get('expected_v4_promotion_manifest_sha256')
            == expected_manifest_sha256
        ),
        'shadow_topics_exact': all(
            runtime_parameters.get(name) == topic
            for name, topic in expected_topics.items()
        ),
        'candidate_not_active': candidate_state.get('active_runtime_selected') is False,
        'candidate_manual_review_pending': (
            candidate_state.get('manual_review_approved') is False
            and candidate_state.get('manual_runtime_review_status') == 'pending'
        ),
    }
    node_contract = _inspect_visual_node_shadow_contract()
    passed = all(configuration_checks.values()) and node_contract['passed']
    return {
        'passed': passed,
        'verification_method': 'candidate_configuration_plus_python_ast_contract',
        'runtime_parameters_path': str(runtime_parameters_path.resolve()),
        'runtime_parameters_sha256': sha256_file(runtime_parameters_path),
        'candidate_state_path': str(candidate_state_path.resolve()),
        'candidate_state_sha256': sha256_file(candidate_state_path),
        'configuration_checks': configuration_checks,
        'node_contract': node_contract,
        'plan_client_present': False if passed else None,
        'plansys2_update_enabled': False if passed else None,
        'plansys2_update_attempt_count': 0,
        'actuation_publishers_owned': False if passed else None,
        'actuation_command_count': 0,
    }


def _inspect_visual_node_shadow_contract() -> dict[str, Any]:
    node_path = SCRIPT_DIR / 'room_315_visual_state_inference_node.py'
    source = node_path.read_text(encoding='utf-8')
    tree = ast.parse(source, filename=str(node_path))
    node_class = next(
        (
            value for value in tree.body
            if isinstance(value, ast.ClassDef)
            and value.name == 'Room315VisualStateInferenceNode'
        ),
        None,
    )
    if node_class is None:
        raise FaultInjectionVerificationError(
            'visual inference node class is unavailable for safety inspection'
        )
    initializer = next(
        (
            value for value in node_class.body
            if isinstance(value, ast.FunctionDef) and value.name == '__init__'
        ),
        None,
    )
    updater = next(
        (
            value for value in node_class.body
            if isinstance(value, ast.FunctionDef)
            and value.name == '_maybe_update_plansys2'
        ),
        None,
    )
    if initializer is None or updater is None:
        raise FaultInjectionVerificationError(
            'visual inference node safety methods are incomplete'
        )

    plan_client_none = any(
        (
            isinstance(node, (ast.Assign, ast.AnnAssign))
            and _assignment_targets_self_attribute(node, 'plan_client')
            and isinstance(getattr(node, 'value', None), ast.Constant)
            and getattr(node, 'value', None).value is None
        )
        for node in ast.walk(initializer)
    )
    guarded_client_creation = any(
        _is_runtime_mode_comparison(node.test, ast.NotEq, 'shadow')
        and any(
            isinstance(child, ast.Call)
            and _call_name(child.func) == 'PlanSys2PredicateClient'
            for statement in node.body
            for child in ast.walk(statement)
        )
        for node in ast.walk(initializer)
        if isinstance(node, ast.If)
    )
    shadow_update_return = any(
        _is_runtime_mode_comparison(node.test, ast.Eq, 'shadow')
        and any(isinstance(child, ast.Return) for child in node.body)
        for node in ast.walk(updater)
        if isinstance(node, ast.If)
    )
    publisher_message_types = sorted({
        ast.unparse(node.args[0])
        for node in ast.walk(initializer)
        if isinstance(node, ast.Call)
        and _call_name(node.func) == 'create_publisher'
        and node.args
    })
    actuation_types = {
        value for value in publisher_message_types
        if 'Command' in value or 'Action' in value or 'Twist' in value
    }
    checks = {
        'plan_client_initialized_absent': plan_client_none,
        'plan_client_created_only_outside_shadow': guarded_client_creation,
        'shadow_plansys_update_returns_before_client': shadow_update_return,
        'no_actuation_publisher_message_type': not actuation_types,
        'no_supervisor_command_topic_literal': '/room_315/vla/command' not in source,
    }
    return {
        'passed': all(checks.values()),
        'source_path': str(node_path.resolve()),
        'source_sha256': sha256_file(node_path),
        'checks': checks,
        'publisher_message_types': publisher_message_types,
        'actuation_publisher_message_types': sorted(actuation_types),
    }


def _assignment_targets_self_attribute(node: Any, attribute: str) -> bool:
    targets = node.targets if isinstance(node, ast.Assign) else (node.target,)
    return any(
        isinstance(target, ast.Attribute)
        and isinstance(target.value, ast.Name)
        and target.value.id == 'self'
        and target.attr == attribute
        for target in targets
    )


def _is_runtime_mode_comparison(
    value: Any,
    operation_type: type[ast.cmpop],
    expected: str,
) -> bool:
    return bool(
        isinstance(value, ast.Compare)
        and isinstance(value.left, ast.Attribute)
        and isinstance(value.left.value, ast.Name)
        and value.left.value.id == 'self'
        and value.left.attr == 'runtime_mode'
        and len(value.ops) == 1
        and isinstance(value.ops[0], operation_type)
        and len(value.comparators) == 1
        and isinstance(value.comparators[0], ast.Constant)
        and value.comparators[0].value == expected
    )


def _call_name(value: Any) -> str:
    if isinstance(value, ast.Name):
        return value.id
    if isinstance(value, ast.Attribute):
        return value.attr
    return ''


def _accepted_synthetic_v4_output() -> StructuredVisualOutputV4:
    segment_logits = np.full((8, 14), -20.0, dtype=np.float32)
    segment_logits[:, 0] = 20.0
    loaded_logits = np.full((8, 2), -20.0, dtype=np.float32)
    loaded_logits[:, 0] = 20.0
    bbox = np.tile(
        np.asarray((0.20, 0.20, 0.20, 0.20), dtype=np.float32),
        (8, 1),
    )
    s_ratio = np.full((8,), 0.5, dtype=np.float32)
    return StructuredVisualOutputV4(
        segment_logits=segment_logits,
        loaded_logits=loaded_logits,
        bbox=bbox,
        s_ratio=s_ratio,
    )


def _presence_snapshot(
    present_identities: Sequence[str],
    *,
    timestamp_s: float,
) -> PresenceSnapshot:
    present = set(present_identities)
    return PresenceSnapshot(
        timestamp_s=timestamp_s,
        ready=True,
        entries=tuple(
            PresenceEntry(
                identity=identity,
                side='left' if identity.startswith('L') else 'right',
                state=(
                    PRESENCE_PRESENT if identity in present else PRESENCE_ABSENT
                ),
                reason=(
                    '' if identity in present else 'synthetic_smoke_absent'
                ),
            )
            for identity in V4_SLOT_ORDER
        ),
        reasons=(),
        initialized_sides=('left', 'right'),
        stale_sides=(),
        source='deterministic_fault_injection_fixture',
    )


def _ros_parameters(path: Path, node_name: str) -> dict[str, Any]:
    try:
        parsed = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise FaultInjectionVerificationError(
            f'cannot load ROS parameter file {path}: {exc}'
        ) from exc
    if not isinstance(parsed, dict):
        raise FaultInjectionVerificationError(
            f'ROS parameter file root is not an object: {path}'
        )
    node = parsed.get(node_name)
    if not isinstance(node, dict):
        raise FaultInjectionVerificationError(
            f'ROS parameter file lacks node {node_name!r}: {path}'
        )
    parameters = node.get('ros__parameters')
    if not isinstance(parameters, dict):
        raise FaultInjectionVerificationError(
            f'ROS node lacks ros__parameters: {path}'
        )
    return parameters


def _load_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise FaultInjectionVerificationError(
            f'cannot load JSON object {path}: {exc}'
        ) from exc
    if not isinstance(value, dict):
        raise FaultInjectionVerificationError(
            f'JSON root is not an object: {path}'
        )
    return value


def _required_sha256(value: Any, context: str) -> str:
    digest = str(value or '').strip().lower()
    if SHA256_PATTERN.fullmatch(digest) is None:
        raise FaultInjectionVerificationError(
            f'{context} must be a lowercase SHA-256'
        )
    return digest


def _different_sha256(value: str) -> str:
    digest = _required_sha256(value, 'source SHA-256')
    replacement = '0' if digest[0] != '0' else '1'
    return replacement + digest[1:]


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='seconds').replace(
        '+00:00',
        'Z',
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Verify Room 315 V4 fail-closed fault handling.',
    )
    subparsers = parser.add_subparsers(dest='command', required=True)

    faults = subparsers.add_parser(
        'faults',
        help='inject the exact eight faults into a verified V4 shadow candidate',
    )
    faults.add_argument(
        '--candidate-dir',
        '--candidate',
        type=Path,
        default=DEFAULT_CANDIDATE_DIRECTORY,
    )
    faults.add_argument(
        '--expected-manifest-sha256',
        '--manifest-sha256',
        required=True,
    )
    faults.add_argument('--output', '--report', type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    report = run_fault_verification(
        candidate_directory=arguments.candidate_dir,
        expected_manifest_sha256=arguments.expected_manifest_sha256,
    )
    report['report_path'] = str(arguments.output.expanduser().resolve())
    report_file_sha256 = write_immutable_report(arguments.output, report)
    print(json.dumps({
        'schema_version': report['schema_version'],
        'status': report['status'],
        'overall_passed': report['overall_passed'],
        'report_path': str(arguments.output.expanduser().resolve()),
        'report_file_sha256': report_file_sha256,
    }, sort_keys=True))
    return 0 if report['overall_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
