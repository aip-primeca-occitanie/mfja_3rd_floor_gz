#!/usr/bin/env python3
"""Create a manually approved, immutable Room 315 V4 active bundle.

This command is deliberately a packaging gate, not a runtime switch.  It
requires externally pinned hashes for the shadow candidate and every runtime
evidence report, verifies all of them fail-closed, and publishes a new
read-only bundle with an atomic rename.  It never edits a checked-in runtime
configuration and it never enables PlanSys2 updates or actuation.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import re
import shutil
import stat
import sys
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping


PROMOTION_SCHEMA = 'room315.visual_runtime_promotion.v4.v1'
SHADOW_REPORT_SCHEMA = 'room315.visual_shadow_comparison.v4.v1'
ACCEPTANCE_REPORT_SCHEMA = 'room315.runtime_acceptance_report.v1'
FAULT_REPORT_SCHEMA = 'room315.visual_runtime_v4.fault_injection.v1'
MANUAL_DECISION_SCHEMA = 'room315.visual_runtime_v4.manual_decision.v1'
ACTIVE_STATE_SCHEMA = 'room315.deployment_candidate_state.v4.v1'

PROMOTION_MANIFEST_NAME = 'runtime_promotion_manifest.json'
EXPECTED_SCENARIO_COUNT = 7
V4_SCHEMA_VERSION = 'room315.visual_state.v4'
# Historical same-frame comparison reference. It is evidence only and is never
# packaged as a selectable runtime.
SHADOW_REFERENCE_CHECKPOINT_SHA256 = (
    '8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d'
)
APPROVED_DECISION = 'approved'
APPROVED_SCOPE = 'gazebo_runtime_dry_run_only'
HEX_SHA256 = re.compile(r'[0-9a-f]{64}')
SAFE_FILENAME = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,191}')

REQUIRED_V4_ARTIFACTS = (
    'checkpoint',
    'training_final_report',
    'validation_acceptance',
    'canary_final_report',
    'canary_completion_ledger',
    'effective_config',
    'validation_segment_calibration',
    'public_topology_contract',
)
REQUIRED_FAULT_CASES = (
    'manifest_hash_mismatch',
    'checkpoint_hash_mismatch',
    'topology_fingerprint_mismatch',
    'camera_order_mismatch',
    'stale_left_image',
    'stale_right_image',
    'stale_presence',
    'unknown_identity',
)
STANDARD_RUNTIME_TOPICS = {
    'raw_prediction': '/room_315/visual_state/raw_model_prediction',
    'raw_observation': '/room_315/visual_state/raw',
    'validation': '/room_315/visual_state/validation',
    'accepted_observed_state': '/room_315/visual_state/observed_state',
}
SHADOW_RUNTIME_TOPICS = {
    'raw_prediction': '/room_315/visual_state/shadow_v4/raw_model_prediction',
    'raw_observation': '/room_315/visual_state/shadow_v4/raw',
    'validation': '/room_315/visual_state/shadow_v4/validation',
    'accepted_observed_state': '/room_315/visual_state/shadow_v4/observed_state',
}


class PromotionV4Error(RuntimeError):
    """Raised before an unsafe or incomplete active bundle can be published."""


@dataclass(frozen=True)
class EvidenceInput:
    path: Path
    expected_sha256: str
    bundle_name: str
    label: str


@dataclass(frozen=True)
class PromotionInputs:
    candidate_directory: Path
    expected_candidate_manifest_sha256: str
    shadow_report: Path
    expected_shadow_report_sha256: str
    acceptance_report: Path
    expected_acceptance_report_sha256: str
    fault_injection_report: Path
    expected_fault_injection_report_sha256: str
    reviewer: str
    decision: str
    scope: str
    output: Path


@dataclass(frozen=True)
class VerifiedCandidate:
    directory: Path
    candidate_id: str
    state: Mapping[str, Any]
    manifest: Mapping[str, Any]
    checkpoint_sha256: str
    checkpoint_filename: str
    manifest_sha256: str
    scenario_ids: tuple[str, ...]
    artifact_paths: Mapping[str, Path]


@dataclass(frozen=True)
class VerifiedEvidence:
    shadow: Mapping[str, Any]
    acceptance: Mapping[str, Any]
    faults: Mapping[str, Any]
    inputs: tuple[EvidenceInput, ...]


def sha256_file(path: Path | str) -> str:
    candidate = Path(path)
    digest = hashlib.sha256()
    try:
        with candidate.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
    except OSError as exc:
        raise PromotionV4Error(f'cannot hash required input: {candidate}') from exc
    return digest.hexdigest()


def load_json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise PromotionV4Error(f'cannot load {label}: {path}') from exc
    if not isinstance(value, dict):
        raise PromotionV4Error(f'{label} root must be a JSON object')
    return value


def write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise PromotionV4Error(message)


def _required_sha256(value: Any, label: str) -> str:
    text = str(value or '').strip().lower()
    _require(HEX_SHA256.fullmatch(text) is not None, f'{label} is not a SHA-256')
    return text


def _required_text(value: Any, label: str, *, maximum: int = 256) -> str:
    _require(isinstance(value, str), f'{label} must be a string')
    text = value.strip()
    _require(bool(text), f'{label} must not be empty')
    _require(len(text) <= maximum, f'{label} is too long')
    _require(
        all(character.isprintable() and character not in '\r\n' for character in text),
        f'{label} contains control characters',
    )
    return text


def _integer(value: Any, label: str) -> int:
    _require(isinstance(value, int) and not isinstance(value, bool), f'{label} must be an integer')
    return int(value)


def _finite_float(value: Any, label: str) -> float:
    _require(isinstance(value, (int, float)) and not isinstance(value, bool), f'{label} must be numeric')
    result = float(value)
    _require(math.isfinite(result), f'{label} must be finite')
    return result


def _safe_bundle_filename(value: Any, label: str) -> str:
    _require(isinstance(value, str), f'{label} must be a string')
    _require(SAFE_FILENAME.fullmatch(value) is not None, f'{label} is unsafe')
    _require(Path(value).name == value and value not in {'.', '..'}, f'{label} is unsafe')
    return value


def _verify_sha256sums(candidate: Path) -> None:
    sums_path = candidate / 'SHA256SUMS'
    _require(sums_path.is_file() and not sums_path.is_symlink(), 'candidate SHA256SUMS is missing or unsafe')
    try:
        lines = sums_path.read_text(encoding='utf-8').splitlines()
    except OSError as exc:
        raise PromotionV4Error('cannot read candidate SHA256SUMS') from exc
    _require(bool(lines), 'candidate SHA256SUMS is empty')
    declared: dict[str, str] = {}
    for line in lines:
        pieces = line.split('  ', 1)
        _require(len(pieces) == 2, 'candidate SHA256SUMS has a malformed line')
        digest = _required_sha256(pieces[0], 'candidate payload digest')
        name = _safe_bundle_filename(pieces[1], 'candidate payload filename')
        _require(name != 'SHA256SUMS' and name not in declared, 'candidate SHA256SUMS has a duplicate or recursive entry')
        declared[name] = digest
    payloads = {
        path.name
        for path in candidate.iterdir()
        if path.name != 'SHA256SUMS' and path.is_file() and not path.is_symlink()
    }
    _require(set(declared) == payloads, 'candidate SHA256SUMS payload set mismatch')
    for name, expected in declared.items():
        path = candidate / name
        _require(path.is_file() and not path.is_symlink(), f'candidate payload is missing or unsafe: {name}')
        _require(sha256_file(path) == expected, f'candidate payload SHA-256 mismatch: {name}')


def _verify_core_manifest(path: Path, expected_sha256: str) -> Any:
    """Use the runtime's complete metadata verifier without loading Torch."""

    script_directory = Path(__file__).resolve().parent
    if str(script_directory) not in sys.path:
        sys.path.insert(0, str(script_directory))
    try:
        from room_315_visual_runtime_v4 import verify_v4_runtime_promotion

        return verify_v4_runtime_promotion(path, expected_sha256)
    except Exception as exc:
        raise PromotionV4Error(f'V4 runtime-core manifest verification failed: {exc}') from exc


def verify_candidate(
    candidate_directory: Path,
    expected_manifest_sha256: str,
) -> VerifiedCandidate:
    candidate = candidate_directory.expanduser().resolve()
    _require(candidate.is_dir() and not candidate.is_symlink(), f'immutable shadow candidate does not exist: {candidate}')
    _require(
        stat.S_IMODE(candidate.stat().st_mode) & 0o222 == 0,
        'shadow candidate directory is not permission-hardened read-only',
    )
    for path in candidate.iterdir():
        _require(path.is_file() and not path.is_symlink(), f'candidate contains an unsafe payload: {path.name}')
        _require(stat.S_IMODE(path.stat().st_mode) & 0o222 == 0, f'candidate payload is writable: {path.name}')
    _verify_sha256sums(candidate)

    expected = _required_sha256(expected_manifest_sha256, 'expected candidate manifest SHA-256')
    manifest_path = candidate / PROMOTION_MANIFEST_NAME
    _require(manifest_path.is_file() and not manifest_path.is_symlink(), 'candidate promotion manifest is missing or unsafe')
    actual = sha256_file(manifest_path)
    _require(actual == expected, f'candidate promotion manifest SHA-256 mismatch: {actual} != {expected}')
    manifest = load_json_object(manifest_path, 'candidate promotion manifest')
    state = load_json_object(candidate / 'candidate_state.json', 'candidate state')
    scenarios = load_json_object(candidate / 'acceptance_scenarios.json', 'candidate acceptance scenarios')

    _require(manifest.get('schema_version') == PROMOTION_SCHEMA, 'candidate promotion schema mismatch')
    _require('rollback_contract' not in manifest, 'candidate manifest contains a rollback contract')
    _require(not (candidate / 'rollback_option.json').exists(), 'candidate contains a rollback option')
    _require(manifest.get('immutable') is True, 'candidate manifest is not immutable')
    _require(manifest.get('deployment_mode') == 'shadow', 'candidate is not in shadow deployment mode')
    _require(manifest.get('manual_review_approved') is False, 'candidate was already marked manually approved')
    _require(manifest.get('manual_runtime_review_status') == 'pending', 'candidate manual review is not pending')
    _require(manifest.get('shadow_execution_authorized') is True, 'candidate is not authorized for shadow evidence')
    _require(manifest.get('automatic_promotion_allowed') is False, 'candidate permits forbidden automatic promotion')
    _require(manifest.get('manual_only') is True, 'candidate is not manual-only')
    _require(manifest.get('eligibility') == {
        'shadow_load_and_evaluation': True,
        'active_runtime_review': False,
        'active_runtime_selected': False,
        'active_transition_requires_new_immutable_manifest': True,
    }, 'candidate eligibility contract is not shadow-only')

    candidate_id = _required_text(state.get('candidate_id'), 'candidate ID')
    _require(manifest.get('candidate_id') == candidate_id, 'candidate state/manifest ID mismatch')
    _require(state.get('state') == 'shadow_authorized_pending_manual_review', 'candidate state is not pending manual review')
    _require(state.get('deployment_mode') == 'shadow', 'candidate state is not shadow')
    _require(state.get('manual_review_approved') is False, 'candidate state is already approved')
    _require(state.get('manual_runtime_review_status') == 'pending', 'candidate state manual review is not pending')
    _require(state.get('shadow_execution_authorized') is True, 'candidate state lacks shadow authorization')
    _require(state.get('automatic_promotion_allowed') is False, 'candidate state permits automatic promotion')
    _require(state.get('active_runtime_selected') is False, 'candidate state already selects an active runtime')
    _require(state.get('active_transition_requires_new_immutable_manifest') is True, 'candidate does not require a new immutable active manifest')
    _require(state.get('runtime_topics') == SHADOW_RUNTIME_TOPICS, 'candidate runtime topics are not isolated V4 shadow topics')
    checkpoint_sha = _required_sha256(state.get('checkpoint_sha256'), 'candidate checkpoint SHA-256')
    checkpoint_filename = _safe_bundle_filename(state.get('checkpoint_filename'), 'candidate checkpoint filename')
    checkpoint_path = candidate / checkpoint_filename
    _require(checkpoint_path.is_file() and not checkpoint_path.is_symlink(), 'candidate checkpoint is missing or unsafe')
    _require(sha256_file(checkpoint_path) == checkpoint_sha, 'candidate state checkpoint SHA-256 mismatch')

    artifacts = manifest.get('artifacts')
    _require(isinstance(artifacts, Mapping), 'candidate artifacts must be an object')
    _require(set(REQUIRED_V4_ARTIFACTS).issubset(artifacts), 'candidate artifact set is incomplete')
    artifact_paths: dict[str, Path] = {}
    for name in REQUIRED_V4_ARTIFACTS:
        entry = artifacts.get(name)
        _require(isinstance(entry, Mapping), f'candidate artifact entry is invalid: {name}')
        filename = _safe_bundle_filename(entry.get('path'), f'candidate artifact path for {name}')
        digest = _required_sha256(entry.get('sha256'), f'candidate artifact SHA-256 for {name}')
        path = candidate / filename
        _require(path.is_file() and not path.is_symlink(), f'candidate artifact is missing or unsafe: {name}')
        _require(sha256_file(path) == digest, f'candidate artifact SHA-256 mismatch: {name}')
        artifact_paths[name] = path
    _require(artifacts['checkpoint'].get('sha256') == checkpoint_sha, 'manifest/state checkpoint SHA-256 mismatch')
    _require(artifact_paths['checkpoint'] == checkpoint_path, 'manifest/state checkpoint filename mismatch')

    _require(scenarios.get('candidate_id') == candidate_id, 'acceptance-scenario candidate ID mismatch')
    _require(scenarios.get('schema_version') == 'room315.runtime_acceptance_scenarios.v1', 'acceptance-scenario schema mismatch')
    _require(scenarios.get('runtime_candidate') == {
        'runtime_generation': 'v4',
        'runtime_mode': 'shadow',
        'automatic_promotion_allowed': False,
    }, 'acceptance scenarios are not bound to the isolated V4 shadow')
    rows = scenarios.get('scenarios')
    _require(isinstance(rows, list) and len(rows) == EXPECTED_SCENARIO_COUNT, 'candidate must contain exactly seven acceptance scenarios')
    scenario_ids: list[str] = []
    for row in rows:
        _require(isinstance(row, Mapping), 'candidate acceptance scenario must be an object')
        scenario_id = _required_text(row.get('scenario_id'), 'acceptance scenario ID')
        _require(scenario_id not in scenario_ids, 'candidate acceptance scenario IDs are duplicated')
        scenario_ids.append(scenario_id)

    verified = _verify_core_manifest(manifest_path, expected)
    _require(getattr(verified, 'deployment_mode', None) == 'shadow', 'runtime core did not verify a shadow candidate')
    _require(getattr(verified, 'checkpoint_sha256', None) == checkpoint_sha, 'runtime-core/candidate checkpoint mismatch')
    return VerifiedCandidate(
        directory=candidate,
        candidate_id=candidate_id,
        state=state,
        manifest=manifest,
        checkpoint_sha256=checkpoint_sha,
        checkpoint_filename=checkpoint_filename,
        manifest_sha256=expected,
        scenario_ids=tuple(scenario_ids),
        artifact_paths=artifact_paths,
    )


def _load_hashed_evidence(evidence: EvidenceInput) -> dict[str, Any]:
    path = evidence.path.expanduser().resolve()
    expected = _required_sha256(evidence.expected_sha256, f'expected {evidence.label} SHA-256')
    _require(path.is_file() and not path.is_symlink(), f'{evidence.label} is missing or unsafe: {path}')
    actual = sha256_file(path)
    _require(actual == expected, f'{evidence.label} SHA-256 mismatch: {actual} != {expected}')
    return load_json_object(path, evidence.label)


def _verify_shadow_report(report: Mapping[str, Any], candidate: VerifiedCandidate) -> None:
    _require(report.get('schema_version') == SHADOW_REPORT_SCHEMA, 'shadow report schema mismatch')
    _require(report.get('status') == 'passed', 'shadow comparator did not pass')
    _require(report.get('role') == 'observation_only_same_frame_shadow', 'shadow comparator role mismatch')
    _require(report.get('automatic_runtime_switch') is False, 'shadow comparator reports an automatic runtime switch')
    checkpoints = report.get('expected_checkpoint_sha256')
    _require(isinstance(checkpoints, Mapping), 'shadow report lacks checkpoint bindings')
    _require(checkpoints.get('v4') == candidate.checkpoint_sha256, 'shadow report V4 checkpoint mismatch')
    _require(
        checkpoints.get('v3') == SHADOW_REFERENCE_CHECKPOINT_SHA256,
        'shadow report reference checkpoint mismatch',
    )
    if 'candidate_id' in report:
        _require(report.get('candidate_id') == candidate.candidate_id, 'shadow report candidate ID mismatch')
    _require(report.get('wrong_v4_side_identities') == [], 'shadow report contains wrong-side V4 identities')
    _require(report.get('errors') == [], 'shadow comparator contains errors')
    paired = _integer(report.get('paired_frame_count'), 'shadow paired-frame count')
    minimum = _integer(report.get('minimum_paired_frames'), 'shadow minimum paired-frame count')
    accepted = _integer(report.get('v4_accepted_frame_count'), 'shadow accepted-frame count')
    coverage = _finite_float(report.get('v4_acceptance_coverage'), 'shadow V4 acceptance coverage')
    _require(minimum > 0 and paired >= minimum, 'shadow paired-frame coverage is incomplete')
    _require(0 <= accepted <= paired, 'shadow accepted-frame count is invalid')
    _require(abs(coverage - accepted / paired) <= 1e-12, 'shadow acceptance coverage/count mismatch')
    _require(coverage >= 0.90, 'shadow V4 acceptance coverage is below 90%')
    isolation = report.get('control_isolation')
    _require(isinstance(isolation, Mapping), 'shadow report lacks explicit control isolation evidence')
    _require(isolation.get('dry_run_state_fusion') is True, 'shadow V4 was not dry-run isolated')
    _require(isolation.get('plansys2_update_enabled') is False, 'shadow V4 enabled PlanSys2 updates')
    _require(isolation.get('plansys2_mutation_count') == 0, 'shadow V4 mutated PlanSys2 state')
    _require(isolation.get('actuation_command_count') == 0, 'shadow V4 emitted actuation commands')
    _require(isolation.get('comparator_owns_command_publisher') is False, 'shadow comparator owned a command publisher')


def _verify_acceptance_report(report: Mapping[str, Any], candidate: VerifiedCandidate) -> None:
    _require(report.get('schema_version') == ACCEPTANCE_REPORT_SCHEMA, 'runtime acceptance report schema mismatch')
    _require(report.get('candidate_id') == candidate.candidate_id, 'runtime acceptance candidate ID mismatch')
    _require(report.get('checkpoint_sha256') == candidate.checkpoint_sha256, 'runtime acceptance checkpoint SHA-256 mismatch')
    _require(report.get('acceptance_status') == 'complete_pending_human_decision', 'runtime acceptance is not complete and pending human decision')
    _require(report.get('automatic_deployment_approval') is False, 'runtime acceptance performed forbidden automatic approval')
    _require(report.get('scenario_count') == EXPECTED_SCENARIO_COUNT, 'runtime acceptance scenario count is not seven')
    _require(report.get('complete_scenario_count') == EXPECTED_SCENARIO_COUNT, 'runtime acceptance did not complete seven scenarios')
    _require(report.get('failed_scenario_count') == 0, 'runtime acceptance contains failed scenarios')
    approval = report.get('approval')
    _require(isinstance(approval, Mapping) and approval.get('approved') is False, 'runtime acceptance report must remain non-approving')

    quantitative = report.get('quantitative_acceptance')
    _require(isinstance(quantitative, Mapping), 'runtime acceptance lacks quantitative evidence')
    _require(quantitative.get('all_scenarios_quantitatively_complete') is True, 'runtime quantitative acceptance is incomplete')
    _require(quantitative.get('ground_truth_used_as_model_input') is False, 'runtime ground truth was used as model input')
    _require(quantitative.get('required_exact_fields') == ['side', 'segment', 'loaded_state'], 'runtime exact-field acceptance contract mismatch')
    _require(float(quantitative.get('required_planning_s_ratio_tolerance', -1.0)) == 0.12, 'runtime planning ratio tolerance mismatch')
    quantitative_rows = quantitative.get('scenarios')
    _require(isinstance(quantitative_rows, list) and len(quantitative_rows) == EXPECTED_SCENARIO_COUNT, 'runtime quantitative scenario evidence is incomplete')
    quantitative_by_id: dict[str, Mapping[str, Any]] = {}
    for row in quantitative_rows:
        _require(isinstance(row, Mapping), 'runtime quantitative scenario row is invalid')
        scenario_id = str(row.get('scenario_id') or '')
        _require(scenario_id and scenario_id not in quantitative_by_id, 'runtime quantitative scenario IDs are invalid')
        quantitative_by_id[scenario_id] = row
    _require(set(quantitative_by_id) == set(candidate.scenario_ids), 'runtime quantitative scenario IDs do not match candidate')
    for scenario_id, row in quantitative_by_id.items():
        _require(row.get('record_status') == 'complete', f'{scenario_id}: quantitative record is incomplete')
        _require(_integer(row.get('ground_truth_matching_reobservation_count'), f'{scenario_id}: matching reobservation count') >= 3, f'{scenario_id}: fewer than three ground-truth matching reobservations')
        _require(row.get('all_recorded_segments_payloads_and_planning_positions_match') is True, f'{scenario_id}: quantitative ground-truth comparison failed')

    records = report.get('records')
    _require(isinstance(records, list) and len(records) == EXPECTED_SCENARIO_COUNT, 'runtime acceptance records are incomplete')
    record_by_id: dict[str, Mapping[str, Any]] = {}
    for record in records:
        _require(isinstance(record, Mapping), 'runtime acceptance record is invalid')
        scenario_id = str(record.get('scenario_id') or '')
        _require(scenario_id and scenario_id not in record_by_id, 'runtime acceptance record IDs are invalid')
        record_by_id[scenario_id] = record
    _require(set(record_by_id) == set(candidate.scenario_ids), 'runtime acceptance record IDs do not match candidate')
    for scenario_id, record in record_by_id.items():
        _require(record.get('record_status') == 'complete', f'{scenario_id}: acceptance record is incomplete')
        _require(record.get('failure_reasons') in (None, []), f'{scenario_id}: acceptance record contains failures')
        execution = record.get('execution_decision')
        _require(isinstance(execution, Mapping) and execution.get('allowed') is False, f'{scenario_id}: observation-only execution isolation failed')
        reobservation = record.get('reobservation_and_effect_verification')
        _require(isinstance(reobservation, Mapping) and reobservation.get('actuation_performed') is False, f'{scenario_id}: acceptance campaign performed actuation')


def _verify_fault_report(report: Mapping[str, Any], candidate: VerifiedCandidate) -> None:
    _require(report.get('schema_version') == FAULT_REPORT_SCHEMA, 'fault-injection report schema mismatch')
    _require(report.get('candidate_id') == candidate.candidate_id, 'fault-injection candidate ID mismatch')
    _require(report.get('checkpoint_sha256') == candidate.checkpoint_sha256, 'fault-injection checkpoint SHA-256 mismatch')
    _require(report.get('status') == 'passed', 'fault-injection report did not pass')
    _require(report.get('all_faults_rejected') is True, 'not all injected faults were rejected')
    _require(report.get('plansys2_mutation_count') == 0, 'fault injection mutated PlanSys2 state')
    _require(report.get('actuation_command_count') == 0, 'fault injection emitted actuation commands')
    cases = report.get('cases')
    _require(isinstance(cases, list) and len(cases) == len(REQUIRED_FAULT_CASES), 'fault-injection case set is incomplete')
    by_id: dict[str, Mapping[str, Any]] = {}
    for row in cases:
        _require(isinstance(row, Mapping), 'fault-injection case is invalid')
        case_id = str(row.get('case_id') or '')
        _require(case_id and case_id not in by_id, 'fault-injection case IDs are invalid')
        by_id[case_id] = row
    _require(set(by_id) == set(REQUIRED_FAULT_CASES), 'fault-injection case names mismatch')
    for case_id, row in by_id.items():
        _require(row.get('passed') is True and row.get('rejected') is True, f'fault case was not rejected: {case_id}')


def verify_evidence(inputs: PromotionInputs, candidate: VerifiedCandidate) -> VerifiedEvidence:
    evidence_inputs = (
        EvidenceInput(inputs.shadow_report, inputs.expected_shadow_report_sha256, 'shadow_comparison_report.json', 'shadow comparison report'),
        EvidenceInput(inputs.acceptance_report, inputs.expected_acceptance_report_sha256, 'runtime_acceptance_report.json', 'runtime acceptance report'),
        EvidenceInput(inputs.fault_injection_report, inputs.expected_fault_injection_report_sha256, 'fault_injection_report.json', 'fault-injection report'),
    )
    shadow, acceptance, faults = (
        _load_hashed_evidence(item) for item in evidence_inputs
    )
    _verify_shadow_report(shadow, candidate)
    _verify_acceptance_report(acceptance, candidate)
    _verify_fault_report(faults, candidate)
    return VerifiedEvidence(
        shadow=shadow,
        acceptance=acceptance,
        faults=faults,
        inputs=evidence_inputs,
    )


def _validate_manual_decision(inputs: PromotionInputs) -> tuple[str, str, str]:
    reviewer = _required_text(inputs.reviewer, 'manual reviewer')
    decision = _required_text(inputs.decision, 'manual decision').lower()
    scope = _required_text(inputs.scope, 'manual review scope').lower()
    _require(decision == APPROVED_DECISION, f'promotion requires decision={APPROVED_DECISION!r}')
    _require(scope == APPROVED_SCOPE, f'promotion requires scope={APPROVED_SCOPE!r}')
    return reviewer, decision, scope


def _evidence_reference(item: EvidenceInput, payload: Mapping[str, Any]) -> dict[str, Any]:
    return {
        'path': item.bundle_name,
        'sha256': _required_sha256(item.expected_sha256, f'{item.label} SHA-256'),
        'schema_version': payload.get('schema_version'),
        'status': payload.get('status', payload.get('acceptance_status')),
    }


def _manual_decision_record(
    candidate: VerifiedCandidate,
    evidence: VerifiedEvidence,
    *,
    reviewer: str,
    decision: str,
    scope: str,
    reviewed_at_utc: str,
) -> dict[str, Any]:
    payloads = (evidence.shadow, evidence.acceptance, evidence.faults)
    return {
        'schema_version': MANUAL_DECISION_SCHEMA,
        'candidate_id': candidate.candidate_id,
        'checkpoint_sha256': candidate.checkpoint_sha256,
        'source_shadow_candidate': {
            'path': str(candidate.directory),
            'promotion_manifest_sha256': candidate.manifest_sha256,
            'deployment_mode': 'shadow',
        },
        'decision': decision,
        'reviewer': reviewer,
        'scope': scope,
        'reviewed_at_utc': reviewed_at_utc,
        'automatic_promotion': False,
        'physical_deployment_approved': False,
        'runtime_guards': {
            'dry_run_state_fusion': True,
            'plansys2_update_enabled': False,
            'actuation_enabled': False,
        },
        'evidence': {
            item.label.replace('-', '_').replace(' ', '_'): _evidence_reference(item, payload)
            for item, payload in zip(evidence.inputs, payloads)
        },
        'review_assertions': {
            'same_checkpoint_bound_across_v4_evidence': True,
            'same_frame_shadow_passed': True,
            'wrong_side_identity_count': 0,
            'seven_scenarios_quantitatively_complete': True,
            'all_faults_rejected': True,
            'plansys2_mutation_count': 0,
            'actuation_command_count': 0,
        },
    }


def _active_manifest(
    candidate: VerifiedCandidate,
    evidence: VerifiedEvidence,
    decision_record_sha256: str,
) -> dict[str, Any]:
    manifest = copy.deepcopy(dict(candidate.manifest))
    manifest.update({
        'deployment_mode': 'active',
        'manual_review_approved': True,
        'manual_runtime_review_status': 'approved',
        'shadow_execution_authorized': False,
        'automatic_promotion_allowed': False,
        'manual_only': True,
        'eligibility': {
            'shadow_load_and_evaluation': True,
            'active_runtime_review': True,
            'active_runtime_selected': True,
            'active_transition_requires_new_immutable_manifest': False,
        },
        'source_shadow_candidate': {
            'candidate_id': candidate.candidate_id,
            'promotion_manifest_path': 'shadow_candidate_promotion_manifest.json',
            'promotion_manifest_sha256': candidate.manifest_sha256,
        },
        'manual_decision_record': {
            'path': 'manual_decision_record.json',
            'sha256': decision_record_sha256,
            'schema_version': MANUAL_DECISION_SCHEMA,
        },
        'runtime_review_evidence': {
            'shadow_comparison': _evidence_reference(evidence.inputs[0], evidence.shadow),
            'seven_scenario_quantitative_acceptance': _evidence_reference(evidence.inputs[1], evidence.acceptance),
            'fault_injection': _evidence_reference(evidence.inputs[2], evidence.faults),
        },
    })
    return manifest


def _runtime_yaml(output: Path, manifest_sha256: str) -> str:
    manifest_path = json.dumps(str(output / PROMOTION_MANIFEST_NAME))
    manifest_hash = json.dumps(manifest_sha256)
    return f"""room_315_visual_state_inference_node:
  ros__parameters:
    use_sim_time: true
    runtime_generation: v4
    runtime_mode: active
    v4_promotion_manifest_path: {manifest_path}
    expected_v4_promotion_manifest_sha256: {manifest_hash}
    device: auto
    presence_state_timeout_s: 1.0
    presence_warmup_s: 0.5
    dry_run_state_fusion: true
    plansys2_update_enabled: false
    raw_observation_topic: /room_315/visual_state/raw
    raw_model_prediction_topic: /room_315/visual_state/raw_model_prediction
    validation_topic: /room_315/visual_state/validation
    accepted_observed_state_topic: /room_315/visual_state/observed_state
"""


def _active_candidate_state(
    candidate: VerifiedCandidate,
    manifest_sha256: str,
    decision_sha256: str,
    evidence: VerifiedEvidence,
) -> dict[str, Any]:
    return {
        'schema_version': ACTIVE_STATE_SCHEMA,
        'candidate_id': candidate.candidate_id,
        'state': 'active_selected_manual_review_approved',
        'deployment_mode': 'active',
        'manual_review_approved': True,
        'manual_runtime_review_status': 'approved',
        'shadow_execution_authorized': False,
        'automatic_promotion_allowed': False,
        'active_runtime_selected': True,
        'checkpoint_sha256': candidate.checkpoint_sha256,
        'checkpoint_filename': candidate.checkpoint_filename,
        'promotion_manifest_filename': PROMOTION_MANIFEST_NAME,
        'promotion_manifest_sha256': manifest_sha256,
        'manual_decision_record_filename': 'manual_decision_record.json',
        'manual_decision_record_sha256': decision_sha256,
        'runtime_topics': dict(STANDARD_RUNTIME_TOPICS),
        'runtime_guards': {
            'dry_run_state_fusion': True,
            'plansys2_update_enabled': False,
            'actuation_enabled': False,
        },
        'required_task_visual_allowlist': {
            'model_schema_version': V4_SCHEMA_VERSION,
            'checkpoint_sha256': candidate.checkpoint_sha256,
        },
        'runtime_evidence_sha256': {
            'shadow_comparison': evidence.inputs[0].expected_sha256,
            'seven_scenario_quantitative_acceptance': evidence.inputs[1].expected_sha256,
            'fault_injection': evidence.inputs[2].expected_sha256,
        },
    }


def _readme(candidate: VerifiedCandidate, manifest_sha256: str) -> str:
    return f"""# Room 315 V4 manually approved active bundle

Candidate ID: `{candidate.candidate_id}`

This immutable bundle records an explicit manual decision for the scope
`{APPROVED_SCOPE}`.  Its active manifest SHA-256 is `{manifest_sha256}`.

The included runtime parameters use the standard visual-state topics but keep
state fusion in dry-run mode, keep PlanSys2 updates disabled, and authorize no
actuation.  Creating this bundle does not edit or select any checked-in
configuration.  Physical deployment is not approved.

The same-frame V3/V4 comparison is retained only as observation evidence; the
bundle emits V4 runtime parameters only. Every payload is bound by
`SHA256SUMS`; the directory and files are read-only.
"""


def _copy_verified(source: Path, destination: Path, expected_sha256: str, label: str) -> None:
    _require(source.is_file() and not source.is_symlink(), f'{label} source became missing or unsafe')
    expected = _required_sha256(expected_sha256, f'{label} SHA-256')
    _require(sha256_file(source) == expected, f'{label} source changed before copy')
    shutil.copyfile(source, destination)
    _require(sha256_file(destination) == expected, f'{label} changed during copy')
    _require(sha256_file(source) == expected, f'{label} source changed during copy')


def promote(inputs: PromotionInputs) -> dict[str, Any]:
    """Verify all inputs and atomically publish a new active bundle."""

    reviewer, decision, scope = _validate_manual_decision(inputs)
    output = inputs.output.expanduser().resolve()
    _require(not output.exists() and not output.is_symlink(), f'refusing to overwrite active bundle: {output}')
    candidate = verify_candidate(
        inputs.candidate_directory,
        inputs.expected_candidate_manifest_sha256,
    )
    evidence = verify_evidence(inputs, candidate)
    reviewed_at = datetime.now(timezone.utc).isoformat(timespec='seconds').replace('+00:00', 'Z')
    decision_record = _manual_decision_record(
        candidate,
        evidence,
        reviewer=reviewer,
        decision=decision,
        scope=scope,
        reviewed_at_utc=reviewed_at,
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f'.{output.name}.staging-', dir=output.parent))
    try:
        for name in REQUIRED_V4_ARTIFACTS:
            source = candidate.artifact_paths[name]
            destination_name = _safe_bundle_filename(
                candidate.manifest['artifacts'][name]['path'],
                f'active artifact filename for {name}',
            )
            _copy_verified(
                source,
                staging / destination_name,
                candidate.manifest['artifacts'][name]['sha256'],
                f'V4 artifact {name}',
            )

        _copy_verified(
            candidate.directory / PROMOTION_MANIFEST_NAME,
            staging / 'shadow_candidate_promotion_manifest.json',
            candidate.manifest_sha256,
            'shadow candidate promotion manifest',
        )
        _copy_verified(
            candidate.directory / 'candidate_state.json',
            staging / 'shadow_candidate_state.json',
            sha256_file(candidate.directory / 'candidate_state.json'),
            'shadow candidate state',
        )
        _copy_verified(
            candidate.directory / 'acceptance_scenarios.json',
            staging / 'acceptance_scenarios.json',
            sha256_file(candidate.directory / 'acceptance_scenarios.json'),
            'acceptance scenarios',
        )
        for item in evidence.inputs:
            _copy_verified(
                item.path.expanduser().resolve(),
                staging / item.bundle_name,
                item.expected_sha256,
                item.label,
            )

        write_json(staging / 'manual_decision_record.json', decision_record)
        decision_sha = sha256_file(staging / 'manual_decision_record.json')
        active_manifest = _active_manifest(candidate, evidence, decision_sha)
        write_json(staging / PROMOTION_MANIFEST_NAME, active_manifest)
        active_manifest_sha = sha256_file(staging / PROMOTION_MANIFEST_NAME)

        verified_active = _verify_core_manifest(
            staging / PROMOTION_MANIFEST_NAME,
            active_manifest_sha,
        )
        _require(getattr(verified_active, 'deployment_mode', None) == 'active', 'runtime core did not verify the active manifest')
        _require(getattr(verified_active, 'checkpoint_sha256', None) == candidate.checkpoint_sha256, 'active runtime-core checkpoint mismatch')

        write_json(
            staging / 'candidate_state.json',
            _active_candidate_state(
                candidate,
                active_manifest_sha,
                decision_sha,
                evidence,
            ),
        )
        (staging / 'runtime_ros_parameters.yaml').write_text(
            _runtime_yaml(output, active_manifest_sha),
            encoding='utf-8',
        )
        (staging / 'README.md').write_text(
            _readme(candidate, active_manifest_sha),
            encoding='utf-8',
        )

        # Re-verify all external trust inputs immediately before publication.
        _verify_sha256sums(candidate.directory)
        _require(
            sha256_file(candidate.directory / PROMOTION_MANIFEST_NAME)
            == candidate.manifest_sha256,
            'candidate manifest changed during active packaging',
        )
        for item in evidence.inputs:
            _require(
                sha256_file(item.path.expanduser().resolve())
                == _required_sha256(item.expected_sha256, f'{item.label} SHA-256'),
                f'{item.label} changed during active packaging',
            )

        sum_files = sorted(path for path in staging.iterdir() if path.is_file() and path.name != 'SHA256SUMS')
        (staging / 'SHA256SUMS').write_text(
            ''.join(f'{sha256_file(path)}  {path.name}\n' for path in sum_files),
            encoding='utf-8',
        )
        for path in staging.iterdir():
            _require(path.is_file() and not path.is_symlink(), f'unexpected active-bundle payload: {path.name}')
            path.chmod(0o444)
        os.replace(staging, output)
        output.chmod(0o555)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise

    result = {
        'status': 'ACTIVE_BUNDLE_CREATED',
        'candidate_id': candidate.candidate_id,
        'checkpoint_sha256': candidate.checkpoint_sha256,
        'deployment_mode': 'active',
        'manual_review_approved': True,
        'review_scope': scope,
        'active_bundle': str(output),
        'promotion_manifest': str(output / PROMOTION_MANIFEST_NAME),
        'promotion_manifest_sha256': active_manifest_sha,
        'automatic_runtime_switch': False,
        'checked_in_defaults_modified': False,
        'dry_run_state_fusion': True,
        'plansys2_update_enabled': False,
    }
    print(json.dumps(result, sort_keys=True))
    return result


def parse_args(argv: list[str] | None = None) -> PromotionInputs:
    parser = argparse.ArgumentParser(
        description=(
            'Build a fail-closed, manually approved Room 315 V4 active bundle. '
            'This command does not select or launch it.'
        )
    )
    parser.add_argument('--candidate-directory', type=Path, required=True)
    parser.add_argument(
        '--expected-candidate-manifest-sha256',
        '--expected-manifest-sha256',
        dest='expected_candidate_manifest_sha256',
        required=True,
    )
    parser.add_argument('--shadow-report', type=Path, required=True)
    parser.add_argument('--expected-shadow-report-sha256', required=True)
    parser.add_argument('--acceptance-report', type=Path, required=True)
    parser.add_argument('--expected-acceptance-report-sha256', required=True)
    parser.add_argument('--fault-injection-report', type=Path, required=True)
    parser.add_argument('--expected-fault-injection-report-sha256', required=True)
    parser.add_argument('--reviewer', required=True)
    parser.add_argument('--decision', required=True)
    parser.add_argument('--scope', required=True)
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args(argv)
    return PromotionInputs(**vars(arguments))


def main(argv: list[str] | None = None) -> int:
    try:
        promote(parse_args(argv))
    except PromotionV4Error as exc:
        print(json.dumps({
            'status': 'PROMOTION_REJECTED',
            'reason': str(exc),
            'automatic_runtime_switch': False,
        }, sort_keys=True), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
