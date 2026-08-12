#!/usr/bin/env python3
"""Create immutable V4 Gazebo closed-loop qualification/runtime bundles.

This tool never changes checked-in defaults and never authorizes a physical
robot.  ``qualify`` converts the already reviewed V4 active dry-run bundle
into a one-purpose Gazebo qualification bundle after an explicit manual
decision.  ``promote`` is available only after a separately hashed 12-case
closed-loop campaign report passes; it creates the final Gazebo-runtime
bundle without reusing or overwriting either input bundle.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import shutil
import stat
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

from room_315_visual_runtime_v4 import CLOSED_LOOP_QUALIFICATION_SCOPE
from room_315_visual_runtime_v4 import CLOSED_LOOP_RUNTIME_SCOPE
from room_315_visual_runtime_v4 import DRY_RUN_AUTHORIZATION_SCOPE
from room_315_visual_runtime_v4 import MANUAL_DECISION_SCHEMA_VERSION
from room_315_visual_runtime_v4 import verify_v4_runtime_promotion


CAMPAIGN_SCHEMA = 'room315.v4_closed_loop_campaign.v1'
FAULT_CAMPAIGN_SCHEMA = 'room315.v4_closed_loop_fault_campaign.v1'
FAULT_EVIDENCE_SCHEMA = 'room315.v4_closed_loop_fault_evidence.v1'
V4_SCHEMA = 'room315.visual_state.v4'
EXPECTED_CASE_COUNT = 12
EXPECTED_FAULT_SCENARIOS = (
    ('F01', 'corrupt_authorization'),
    ('F02', 'wrong_promotion_manifest'),
    ('F03', 'planner_unavailable'),
    ('F04', 'emergency_stop_during_long_move'),
    ('F05', 'right_sensor_feedback_loss'),
)
HEX = frozenset('0123456789abcdef')


class ClosedLoopAuthorizationError(RuntimeError):
    """Raised before an incomplete or unsafe authorization is published."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _sha(value: Any, label: str) -> str:
    text = str(value or '').strip().lower()
    if len(text) != 64 or any(char not in HEX for char in text):
        raise ClosedLoopAuthorizationError(f'{label} is not a lowercase SHA-256')
    return text


def _load_json(path: Path, label: str) -> dict[str, Any]:
    def reject_duplicate_pairs(
        pairs: list[tuple[str, Any]],
    ) -> dict[str, Any]:
        value: dict[str, Any] = {}
        for key, item in pairs:
            if key in value:
                raise ValueError(f'duplicate JSON key: {key}')
            value[key] = item
        return value

    def reject_constant(value: str) -> None:
        raise ValueError(f'non-finite JSON value: {value}')

    try:
        value = json.loads(
            path.read_text(encoding='utf-8'),
            object_pairs_hook=reject_duplicate_pairs,
            parse_constant=reject_constant,
        )
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        raise ClosedLoopAuthorizationError(f'cannot load {label}: {path}') from exc
    if not isinstance(value, dict):
        raise ClosedLoopAuthorizationError(f'{label} must be a JSON object')
    return value


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ClosedLoopAuthorizationError(message)


def _verify_bundle(root: Path, expected_manifest_sha256: str):
    candidate = root.expanduser().resolve()
    _require(candidate.is_dir() and not candidate.is_symlink(), 'source bundle is missing or unsafe')
    _require(stat.S_IMODE(candidate.stat().st_mode) & 0o222 == 0, 'source bundle must be read-only')
    sums = candidate / 'SHA256SUMS'
    _require(sums.is_file() and not sums.is_symlink(), 'source SHA256SUMS is missing')
    declared: dict[str, str] = {}
    for line in sums.read_text(encoding='utf-8').splitlines():
        pieces = line.split('  ', 1)
        _require(len(pieces) == 2, 'source SHA256SUMS is malformed')
        digest, name = _sha(pieces[0], 'source payload digest'), pieces[1]
        _require(Path(name).name == name and name not in declared, 'source SHA256SUMS path is unsafe')
        declared[name] = digest
    payloads = {
        path.name for path in candidate.iterdir()
        if path.is_file() and not path.is_symlink() and path.name != 'SHA256SUMS'
    }
    _require(set(declared) == payloads, 'source SHA256SUMS payload set mismatch')
    for name, digest in declared.items():
        _require(_sha256(candidate / name) == digest, f'source payload hash mismatch: {name}')
    manifest = candidate / 'runtime_promotion_manifest.json'
    promotion = verify_v4_runtime_promotion(
        manifest,
        _sha(expected_manifest_sha256, 'expected source manifest SHA-256'),
    )
    return candidate, promotion, _load_json(manifest, 'source promotion manifest')


def _verify_campaign(
    path: Path,
    expected_sha256: str,
    *,
    qualification_manifest_sha256: str,
    checkpoint_sha256: str,
) -> dict[str, Any]:
    report_path = path.expanduser().resolve()
    _require(report_path.is_file() and not report_path.is_symlink(), 'campaign report is missing or unsafe')
    _require(
        stat.S_IMODE(report_path.stat().st_mode) & 0o222 == 0,
        'campaign report must be read-only',
    )
    _require(_sha256(report_path) == _sha(expected_sha256, 'campaign report SHA-256'), 'campaign report SHA-256 mismatch')
    report = _load_json(report_path, 'closed-loop campaign report')
    _require(report.get('schema_version') == CAMPAIGN_SCHEMA, 'campaign report schema mismatch')
    _require(report.get('status') == 'passed', 'closed-loop campaign did not pass')
    _require(report.get('visual_schema_version') == V4_SCHEMA, 'campaign did not use V4 observations')
    _require(report.get('checkpoint_sha256') == checkpoint_sha256, 'campaign checkpoint mismatch')
    _require(
        report.get('qualification_manifest_sha256') == qualification_manifest_sha256,
        'campaign qualification-manifest mismatch',
    )
    count_fields = {
        'case_count': EXPECTED_CASE_COUNT,
        'passed_case_count': EXPECTED_CASE_COUNT,
        'failed_case_count': 0,
        'v3_observation_count': 0,
    }
    for field, expected in count_fields.items():
        actual = report.get(field)
        _require(
            isinstance(actual, int) and not isinstance(actual, bool)
            and actual == expected,
            f'campaign {field} mismatch',
        )
    _require(report.get('physical_deployment') is False, 'campaign claims physical deployment')
    _require(report.get('all_terminal_statuses_succeeded') is True, 'campaign has a non-success terminal status')
    _require(report.get('all_final_effects_verified') is True, 'campaign final effects were not all verified')
    _require(report.get('all_controllers_stopped') is True, 'campaign did not prove controller stop')
    _require(report.get('safe_abort_count') == 0, 'campaign emitted a safe abort')
    return report


def _exact_integer(
    value: Mapping[str, Any],
    field: str,
    expected: int,
    *,
    label: str,
) -> None:
    actual = value.get(field)
    _require(
        isinstance(actual, int) and not isinstance(actual, bool)
        and actual == expected,
        f'{label} {field} mismatch',
    )


def _verify_fault_campaign(
    path: Path,
    expected_sha256: str,
    *,
    qualification_manifest_sha256: str,
    checkpoint_sha256: str,
) -> tuple[dict[str, Any], Path]:
    requested_path = path.expanduser()
    _require(
        requested_path.is_file() and not requested_path.is_symlink(),
        'fault-campaign report is missing or unsafe',
    )
    report_path = requested_path.resolve()
    _require(
        report_path.name == 'summary.json',
        'fault-campaign report must be the producer summary.json',
    )
    _require(
        stat.S_IMODE(report_path.stat().st_mode) & 0o222 == 0,
        'fault-campaign report must be read-only',
    )
    report_sha256 = _sha(
        expected_sha256,
        'fault-campaign report SHA-256',
    )
    _require(
        _sha256(report_path) == report_sha256,
        'fault-campaign report SHA-256 mismatch',
    )
    report = _load_json(report_path, 'closed-loop fault-campaign report')
    _require(
        report.get('schema_version') == FAULT_CAMPAIGN_SCHEMA,
        'fault-campaign report schema mismatch',
    )
    _require(report.get('status') == 'passed', 'closed-loop fault campaign did not pass')
    _require(
        report.get('authorization_scope') == CLOSED_LOOP_QUALIFICATION_SCOPE,
        'fault campaign authorization scope mismatch',
    )
    _require(
        report.get('visual_schema_version') == V4_SCHEMA,
        'fault campaign did not use V4 observations',
    )
    _require(
        report.get('visual_checkpoint_sha256') == checkpoint_sha256,
        'fault-campaign checkpoint mismatch',
    )
    _require(
        report.get('qualification_manifest_sha256')
        == qualification_manifest_sha256,
        'fault-campaign qualification-manifest mismatch',
    )
    for field, expected in {
        'declared_scenario_count': len(EXPECTED_FAULT_SCENARIOS),
        'selected_scenario_count': len(EXPECTED_FAULT_SCENARIOS),
        'completed_scenario_count': len(EXPECTED_FAULT_SCENARIOS),
        'passed_scenario_count': len(EXPECTED_FAULT_SCENARIOS),
        'failed_scenario_count': 0,
        'v3_observation_count': 0,
    }.items():
        _exact_integer(report, field, expected, label='fault campaign')
    _require(
        report.get('all_false_success_counts_zero') is True,
        'fault campaign recorded a false success',
    )
    _require(
        report.get('all_controllers_disabled') is True,
        'fault campaign did not leave every controller disabled',
    )
    _require(
        report.get('physical_deployment') is False,
        'fault campaign claims physical deployment',
    )
    _require(
        report.get('failure_reason') == '',
        'passed fault campaign contains a failure reason',
    )
    v4_observation_count = report.get('v4_observation_count')
    _require(
        isinstance(v4_observation_count, int)
        and not isinstance(v4_observation_count, bool)
        and v4_observation_count >= len(EXPECTED_FAULT_SCENARIOS),
        'fault campaign V4 observation count is insufficient',
    )

    results = report.get('results')
    _require(isinstance(results, list), 'fault-campaign results must be an array')
    observed_scenarios: list[tuple[str, str]] = []
    for index, result in enumerate(results):
        _require(
            isinstance(result, Mapping),
            f'fault-campaign result {index} must be an object',
        )
        observed_scenarios.append((
            str(result.get('scenario_id') or ''),
            str(result.get('fault') or ''),
        ))
        label = f'fault-campaign result {index}'
        _require(result.get('status') == 'passed', f'{label} did not pass')
        _require(
            result.get('visual_schema_version') == V4_SCHEMA,
            f'{label} visual schema mismatch',
        )
        _require(
            result.get('checkpoint_sha256') == checkpoint_sha256,
            f'{label} checkpoint mismatch',
        )
        _require(
            result.get('qualification_manifest_sha256')
            == qualification_manifest_sha256,
            f'{label} qualification-manifest mismatch',
        )
        _exact_integer(result, 'false_success_count', 0, label=label)
        _exact_integer(result, 'v3_observation_count', 0, label=label)
        result_v4_count = result.get('v4_observation_count')
        _require(
            isinstance(result_v4_count, int)
            and not isinstance(result_v4_count, bool)
            and result_v4_count >= 1,
            f'{label} has no accepted V4 observation',
        )
        _require(
            result.get('controller_final_mode') == 'DISABLED',
            f'{label} controller was not disabled',
        )
        _require(
            result.get('physical_deployment') is False,
            f'{label} claims physical deployment',
        )
    _require(
        tuple(observed_scenarios) == EXPECTED_FAULT_SCENARIOS,
        'fault campaign must contain exact ordered scenarios F01--F05',
    )
    return report, report_path


def _safe_evidence_relative_path(raw_path: Any) -> Path:
    _require(
        isinstance(raw_path, str) and raw_path.strip() == raw_path
        and raw_path,
        'fault-evidence path is invalid',
    )
    relative = Path(raw_path)
    _require(
        not relative.is_absolute()
        and relative.parts
        and '..' not in relative.parts
        and '.' not in relative.parts,
        f'fault-evidence path is unsafe: {raw_path}',
    )
    return relative


def _verify_fault_evidence_sidecars(
    report_path: Path,
    report_sha256: str,
) -> dict[str, Any]:
    evidence_root = report_path.parent
    _require(
        evidence_root.is_dir() and not evidence_root.is_symlink(),
        'fault-evidence root is missing or unsafe',
    )
    _require(
        stat.S_IMODE(evidence_root.stat().st_mode) & 0o222 == 0,
        'fault-evidence root must be read-only',
    )
    manifest_path = evidence_root / 'manifest.json'
    checksums_path = evidence_root / 'SHA256SUMS'
    for path, label in (
        (manifest_path, 'fault-evidence manifest'),
        (checksums_path, 'fault-evidence SHA256SUMS'),
    ):
        _require(
            path.is_file() and not path.is_symlink(),
            f'{label} is missing or unsafe',
        )
        _require(
            stat.S_IMODE(path.stat().st_mode) & 0o222 == 0,
            f'{label} must be read-only',
        )

    declared: dict[str, str] = {}
    try:
        checksum_lines = checksums_path.read_text(encoding='utf-8').splitlines()
    except (OSError, UnicodeError) as exc:
        raise ClosedLoopAuthorizationError(
            f'cannot read fault-evidence SHA256SUMS: {checksums_path}'
        ) from exc
    _require(checksum_lines, 'fault-evidence SHA256SUMS is empty')
    for line in checksum_lines:
        pieces = line.split('  ', 1)
        _require(len(pieces) == 2, 'fault-evidence SHA256SUMS is malformed')
        digest = _sha(pieces[0], 'fault-evidence payload SHA-256')
        relative = _safe_evidence_relative_path(pieces[1])
        name = relative.as_posix()
        _require(name not in declared, 'fault-evidence SHA256SUMS has a duplicate path')
        declared[name] = digest

    actual: dict[str, Path] = {}
    for candidate in evidence_root.rglob('*'):
        _require(not candidate.is_symlink(), 'fault evidence must not contain symlinks')
        if candidate.is_dir():
            _require(
                stat.S_IMODE(candidate.stat().st_mode) & 0o222 == 0,
                f'fault-evidence directory must be read-only: {candidate}',
            )
            continue
        _require(candidate.is_file(), f'unsafe fault-evidence payload: {candidate}')
        _require(
            stat.S_IMODE(candidate.stat().st_mode) & 0o222 == 0,
            f'fault-evidence payload must be read-only: {candidate}',
        )
        if candidate == checksums_path:
            continue
        name = candidate.relative_to(evidence_root).as_posix()
        actual[name] = candidate
    _require(
        set(declared) == set(actual),
        'fault-evidence SHA256SUMS payload set mismatch',
    )
    for name, path in actual.items():
        _require(
            _sha256(path) == declared[name],
            f'fault-evidence payload hash mismatch: {name}',
        )
    _require(
        declared.get('summary.json') == report_sha256,
        'fault-evidence SHA256SUMS does not bind the campaign summary',
    )
    manifest_sha256 = _sha256(manifest_path)
    _require(
        declared.get('manifest.json') == manifest_sha256,
        'fault-evidence SHA256SUMS does not bind its manifest',
    )

    manifest = _load_json(manifest_path, 'fault-evidence manifest')
    _require(
        manifest.get('schema_version') == FAULT_EVIDENCE_SCHEMA,
        'fault-evidence manifest schema mismatch',
    )
    _require(
        manifest.get('campaign_schema') == FAULT_CAMPAIGN_SCHEMA,
        'fault-evidence campaign schema mismatch',
    )
    _require(
        manifest.get('campaign_status') == 'passed',
        'fault-evidence manifest does not record a passed campaign',
    )
    _require(
        manifest.get('physical_deployment') is False,
        'fault-evidence manifest claims physical deployment',
    )
    files = manifest.get('files')
    _require(isinstance(files, list), 'fault-evidence manifest files must be an array')
    _exact_integer(
        manifest,
        'file_count_excluding_manifest_and_checksums',
        len(files),
        label='fault-evidence manifest',
    )
    manifest_rows: dict[str, tuple[int, str]] = {}
    for index, row in enumerate(files):
        _require(isinstance(row, Mapping), f'fault-evidence row {index} must be an object')
        name = _safe_evidence_relative_path(row.get('path')).as_posix()
        _require(
            name not in {'manifest.json', 'SHA256SUMS'}
            and name not in manifest_rows,
            f'fault-evidence manifest has an unsafe or duplicate row: {name}',
        )
        size = row.get('size_bytes')
        _require(
            isinstance(size, int) and not isinstance(size, bool) and size >= 0,
            f'fault-evidence size is invalid: {name}',
        )
        manifest_rows[name] = (
            size,
            _sha(row.get('sha256'), f'fault-evidence manifest SHA-256: {name}'),
        )
    expected_manifest_names = set(declared) - {'manifest.json'}
    _require(
        set(manifest_rows) == expected_manifest_names,
        'fault-evidence manifest payload set mismatch',
    )
    for name, (size, digest) in manifest_rows.items():
        _require(
            actual[name].stat().st_size == size and declared[name] == digest,
            f'fault-evidence manifest metadata mismatch: {name}',
        )
    _require(
        manifest_rows.get('summary.json')
        == (report_path.stat().st_size, report_sha256),
        'fault-evidence manifest does not bind the campaign summary',
    )
    return {
        'manifest_path': manifest_path,
        'manifest_sha256': manifest_sha256,
        'sha256s_path': checksums_path,
        'sha256s_sha256': _sha256(checksums_path),
    }


def _decision_record(
    *,
    source_manifest_sha256: str,
    source_candidate_id: str,
    checkpoint_sha256: str,
    reviewer: str,
    scope: str,
    campaign: Mapping[str, Any] | None,
    campaign_sha256: str,
    fault_campaign: Mapping[str, Any] | None = None,
    fault_campaign_sha256: str = '',
    fault_evidence: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        'schema_version': MANUAL_DECISION_SCHEMA_VERSION,
        'candidate_id': source_candidate_id,
        'checkpoint_sha256': checkpoint_sha256,
        'decision': 'approved',
        'reviewer': reviewer,
        'reviewed_at_utc': datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z'),
        'scope': scope,
        'automatic_promotion': False,
        'physical_deployment_approved': False,
        'runtime_guards': {
            # V4 observations feed the task gateway.  The visual node's
            # redundant Problem-Expert predicate mirror remains disabled.
            'dry_run_state_fusion': True,
            'plansys2_update_enabled': False,
            'actuation_enabled': True,
        },
        'source_active_manifest_sha256': source_manifest_sha256,
        'qualification_only': scope == CLOSED_LOOP_QUALIFICATION_SCOPE,
        'closed_loop_campaign': (
            None if campaign is None else {
                'schema_version': campaign.get('schema_version'),
                'sha256': campaign_sha256,
                'case_count': campaign.get('case_count'),
                'passed_case_count': campaign.get('passed_case_count'),
                'failed_case_count': campaign.get('failed_case_count'),
            }
        ),
        'closed_loop_fault_campaign': (
            None if fault_campaign is None else {
                'schema_version': fault_campaign.get('schema_version'),
                'sha256': fault_campaign_sha256,
                'scenario_ids': [
                    result.get('scenario_id')
                    for result in fault_campaign.get('results', [])
                ],
                'passed_scenario_count': fault_campaign.get(
                    'passed_scenario_count'
                ),
                'failed_scenario_count': fault_campaign.get(
                    'failed_scenario_count'
                ),
                'evidence_manifest_sha256': (
                    fault_evidence or {}
                ).get('manifest_sha256'),
                'evidence_sha256s_sha256': (
                    fault_evidence or {}
                ).get('sha256s_sha256'),
            }
        ),
        'review_assertions': {
            'v4_is_authoritative_visual_observation': True,
            'plansys2_problem_built_from_v4_observed_state': True,
            'optional_problem_expert_predicate_mirror_disabled': True,
            'supervisor_and_dzi_safety_gates_required': True,
            'rollback_preserved': True,
        },
    }


def _runtime_yaml(output: Path, manifest_sha256: str) -> str:
    return f'''room_315_visual_state_inference_node:
  ros__parameters:
    use_sim_time: true
    runtime_generation: v4
    runtime_mode: active
    v4_promotion_manifest_path: "{output / 'runtime_promotion_manifest.json'}"
    expected_v4_promotion_manifest_sha256: "{manifest_sha256}"
    device: auto
    presence_state_timeout_s: 1.0
    presence_warmup_s: 0.5
    dry_run_state_fusion: true
    plansys2_update_enabled: false
    raw_observation_topic: /room_315/visual_state/raw
    raw_model_prediction_topic: /room_315/visual_state/raw_model_prediction
    validation_topic: /room_315/visual_state/validation
    accepted_observed_state_topic: /room_315/visual_state/observed_state
'''


def create_bundle(
    *,
    source: Path,
    source_manifest_sha256: str,
    output: Path,
    reviewer: str,
    scope: str,
    campaign_report: Path | None = None,
    campaign_report_sha256: str = '',
    fault_campaign_report: Path | None = None,
    fault_campaign_report_sha256: str = '',
) -> dict[str, Any]:
    source_root, promotion, source_manifest = _verify_bundle(
        source,
        source_manifest_sha256,
    )
    _require(scope in {CLOSED_LOOP_QUALIFICATION_SCOPE, CLOSED_LOOP_RUNTIME_SCOPE}, 'unsupported output scope')
    if scope == CLOSED_LOOP_QUALIFICATION_SCOPE:
        _require(promotion.authorization_scope == DRY_RUN_AUTHORIZATION_SCOPE, 'qualification source must be approved dry-run V4')
        _require(
            campaign_report is None and not campaign_report_sha256,
            'qualification must not consume campaign evidence',
        )
        _require(
            fault_campaign_report is None and not fault_campaign_report_sha256,
            'qualification must not consume fault-campaign evidence',
        )
        campaign = None
        campaign_sha = ''
        fault_campaign = None
        fault_campaign_sha = ''
        fault_evidence = None
        verified_fault_report_path = None
    else:
        _require(promotion.authorization_scope == CLOSED_LOOP_QUALIFICATION_SCOPE, 'runtime source must be a V4 qualification bundle')
        _require(campaign_report is not None, 'runtime promotion requires campaign evidence')
        _require(
            fault_campaign_report is not None,
            'runtime promotion requires fault-campaign evidence',
        )
        campaign_sha = _sha(campaign_report_sha256, 'campaign report SHA-256')
        campaign = _verify_campaign(
            campaign_report,
            campaign_sha,
            qualification_manifest_sha256=promotion.manifest_sha256,
            checkpoint_sha256=promotion.checkpoint_sha256,
        )
        fault_campaign_sha = _sha(
            fault_campaign_report_sha256,
            'fault-campaign report SHA-256',
        )
        fault_campaign, verified_fault_report_path = _verify_fault_campaign(
            fault_campaign_report,
            fault_campaign_sha,
            qualification_manifest_sha256=promotion.manifest_sha256,
            checkpoint_sha256=promotion.checkpoint_sha256,
        )
        fault_evidence = _verify_fault_evidence_sidecars(
            verified_fault_report_path,
            fault_campaign_sha,
        )

    reviewer_text = str(reviewer or '').strip()
    _require(reviewer_text and '\n' not in reviewer_text, 'reviewer must be a non-empty single line')
    destination = output.expanduser().resolve()
    _require(not destination.exists(), f'refusing to replace output: {destination}')
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f'.{destination.name}.', dir=destination.parent))
    try:
        for source_path in source_root.iterdir():
            if source_path.name in {
                'SHA256SUMS', 'runtime_promotion_manifest.json',
                'manual_decision_record.json', 'runtime_ros_parameters.yaml',
                'candidate_state.json', 'README.md',
            }:
                continue
            if source_path.is_file() and not source_path.is_symlink():
                shutil.copy2(source_path, staging / source_path.name)
        if campaign_report is not None:
            shutil.copy2(campaign_report, staging / 'closed_loop_campaign_report.json')
        if verified_fault_report_path is not None and fault_evidence is not None:
            fault_payloads = (
                (
                    verified_fault_report_path,
                    'closed_loop_fault_campaign_report.json',
                    fault_campaign_sha,
                ),
                (
                    Path(fault_evidence['manifest_path']),
                    'closed_loop_fault_campaign_evidence_manifest.json',
                    str(fault_evidence['manifest_sha256']),
                ),
                (
                    Path(fault_evidence['sha256s_path']),
                    'closed_loop_fault_campaign_source_SHA256SUMS',
                    str(fault_evidence['sha256s_sha256']),
                ),
            )
            for source_path, output_name, expected_sha256 in fault_payloads:
                output_path = staging / output_name
                shutil.copy2(source_path, output_path)
                _require(
                    _sha256(output_path) == expected_sha256,
                    f'fault-campaign evidence changed while copying: {output_name}',
                )

        decision = _decision_record(
            source_manifest_sha256=promotion.manifest_sha256,
            source_candidate_id=str(source_manifest.get('candidate_id') or ''),
            checkpoint_sha256=promotion.checkpoint_sha256,
            reviewer=reviewer_text,
            scope=scope,
            campaign=campaign,
            campaign_sha256=campaign_sha,
            fault_campaign=fault_campaign,
            fault_campaign_sha256=fault_campaign_sha,
            fault_evidence=fault_evidence,
        )
        _write_json(staging / 'manual_decision_record.json', decision)
        decision_sha = _sha256(staging / 'manual_decision_record.json')

        manifest = copy.deepcopy(source_manifest)
        manifest.update({
            'manual_review_approved': True,
            'manual_runtime_review_status': 'approved',
            'shadow_execution_authorized': False,
            'automatic_promotion_allowed': False,
            'manual_only': True,
            'manual_decision_record': {
                'path': 'manual_decision_record.json',
                'sha256': decision_sha,
                'schema_version': MANUAL_DECISION_SCHEMA_VERSION,
            },
            'source_active_manifest_sha256': promotion.manifest_sha256,
            'closed_loop_qualification': {
                'qualification_only': scope == CLOSED_LOOP_QUALIFICATION_SCOPE,
                'campaign_report_path': (
                    'closed_loop_campaign_report.json' if campaign is not None else None
                ),
                'campaign_report_sha256': campaign_sha or None,
                'physical_deployment_approved': False,
            },
            'closed_loop_fault_campaign': (
                None if fault_campaign is None else {
                    'schema_version': fault_campaign.get('schema_version'),
                    'report_path': 'closed_loop_fault_campaign_report.json',
                    'report_sha256': fault_campaign_sha,
                    'scenario_ids': [
                        result.get('scenario_id')
                        for result in fault_campaign.get('results', [])
                    ],
                    'passed_scenario_count': fault_campaign.get(
                        'passed_scenario_count'
                    ),
                    'failed_scenario_count': fault_campaign.get(
                        'failed_scenario_count'
                    ),
                    'evidence_manifest_path': (
                        'closed_loop_fault_campaign_evidence_manifest.json'
                    ),
                    'evidence_manifest_sha256': fault_evidence[
                        'manifest_sha256'
                    ],
                    'source_sha256s_path': (
                        'closed_loop_fault_campaign_source_SHA256SUMS'
                    ),
                    'source_sha256s_sha256': fault_evidence['sha256s_sha256'],
                    'physical_deployment': False,
                }
            ),
        })
        _write_json(staging / 'runtime_promotion_manifest.json', manifest)
        manifest_sha = _sha256(staging / 'runtime_promotion_manifest.json')
        (staging / 'runtime_ros_parameters.yaml').write_text(
            _runtime_yaml(destination, manifest_sha), encoding='utf-8'
        )
        _write_json(staging / 'candidate_state.json', {
            'schema_version': 'room315.deployment_candidate_state.v4.v1',
            'candidate_id': manifest.get('candidate_id'),
            'state': (
                'active_closed_loop_qualification'
                if scope == CLOSED_LOOP_QUALIFICATION_SCOPE
                else 'active_closed_loop_runtime'
            ),
            'deployment_mode': 'active',
            'authorization_scope': scope,
            'manual_review_approved': True,
            'automatic_promotion_allowed': False,
            'physical_deployment_approved': False,
            'checkpoint_filename': Path(promotion.artifact('checkpoint').path).name,
            'checkpoint_sha256': promotion.checkpoint_sha256,
            'promotion_manifest_sha256': manifest_sha,
            'runtime_guards': decision['runtime_guards'],
        })
        (staging / 'README.md').write_text(
            '# Room 315 V4 Gazebo closed-loop bundle\n\n'
            f'Authorization scope: `{scope}`.\n\n'
            'V4 is the sole visual observation admitted to task planning. '
            'Physical deployment and automatic promotion remain forbidden.\n',
            encoding='utf-8',
        )
        payloads = sorted(path for path in staging.iterdir() if path.is_file())
        (staging / 'SHA256SUMS').write_text(
            ''.join(f'{_sha256(path)}  {path.name}\n' for path in payloads),
            encoding='utf-8',
        )
        for path in staging.iterdir():
            os.chmod(path, 0o444)
        os.chmod(staging, 0o555)
        staging.rename(destination)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise

    verified = verify_v4_runtime_promotion(
        destination / 'runtime_promotion_manifest.json',
        manifest_sha,
    )
    _require(verified.authorization_scope == scope, 'published scope verification failed')
    return {
        'output': str(destination),
        'scope': scope,
        'manifest_sha256': manifest_sha,
        'checkpoint_sha256': verified.checkpoint_sha256,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=('qualify', 'promote'))
    parser.add_argument('--source', required=True)
    parser.add_argument('--source-manifest-sha256', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--reviewer', required=True)
    parser.add_argument('--campaign-report', default='')
    parser.add_argument('--campaign-report-sha256', default='')
    parser.add_argument('--fault-campaign-report', default='')
    parser.add_argument('--fault-campaign-report-sha256', default='')
    args = parser.parse_args()
    if args.phase == 'promote' and (
        not args.fault_campaign_report
        or not args.fault_campaign_report_sha256
    ):
        parser.error(
            'promote requires --fault-campaign-report and '
            '--fault-campaign-report-sha256'
        )
    if args.phase == 'qualify' and (
        args.fault_campaign_report or args.fault_campaign_report_sha256
    ):
        parser.error('qualify does not accept fault-campaign evidence')
    result = create_bundle(
        source=Path(args.source),
        source_manifest_sha256=args.source_manifest_sha256,
        output=Path(args.output),
        reviewer=args.reviewer,
        scope=(
            CLOSED_LOOP_QUALIFICATION_SCOPE
            if args.phase == 'qualify'
            else CLOSED_LOOP_RUNTIME_SCOPE
        ),
        campaign_report=(Path(args.campaign_report) if args.campaign_report else None),
        campaign_report_sha256=args.campaign_report_sha256,
        fault_campaign_report=(
            Path(args.fault_campaign_report)
            if args.fault_campaign_report else None
        ),
        fault_campaign_report_sha256=args.fault_campaign_report_sha256,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
