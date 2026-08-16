#!/usr/bin/env python3

from __future__ import annotations

import copy
import hashlib
import json
import os
import sys
from pathlib import Path
from typing import Any

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_task_execution_config as execution_config


def _manual_decision_payload() -> dict[str, Any]:
    return {
        'schema_version': (
            execution_config.V4_TASK_EXECUTION_MANUAL_DECISION_SCHEMA_VERSION
        ),
        'scope': execution_config.V4_TASK_EXECUTION_AUTHORIZATION_SCOPE,
        'candidate_id': execution_config.V4_TASK_EXECUTION_CANDIDATE_ID,
        'checkpoint_sha256': (
            execution_config.V4_TASK_EXECUTION_CHECKPOINT_SHA256
        ),
        'decision': 'approved',
        'qualification_only': True,
        'physical_deployment_approved': False,
        'automatic_promotion': False,
        'source_active_manifest_sha256': (
            execution_config.V4_TASK_EXECUTION_SOURCE_ACTIVE_MANIFEST_SHA256
        ),
        'runtime_guards': copy.deepcopy(
            execution_config.V4_TASK_EXECUTION_RUNTIME_GUARDS
        ),
        'closed_loop_campaign': None,
    }


def _promotion_payload(manual_sha256: str) -> dict[str, Any]:
    return {
        'schema_version': (
            execution_config.V4_TASK_EXECUTION_PROMOTION_SCHEMA_VERSION
        ),
        'immutable': True,
        'automatic_promotion_allowed': False,
        'manual_only': True,
        'manual_review_approved': True,
        'manual_runtime_review_status': 'approved',
        'deployment_mode': 'active',
        'closed_loop_qualification': {
            'qualification_only': True,
            'physical_deployment_approved': False,
            'campaign_report_path': None,
            'campaign_report_sha256': None,
        },
        'eligibility': {
            'active_runtime_review': True,
            'active_runtime_selected': True,
            'active_transition_requires_new_immutable_manifest': False,
        },
        'artifacts': {
            'checkpoint': {
                'path': 'checkpoint_epoch_011.pt',
                'sha256': (
                    execution_config.V4_TASK_EXECUTION_CHECKPOINT_SHA256
                ),
            },
        },
        'model_contract': {
            'checkpoint_epoch': 11,
            'checkpoint_loading': 'strict',
            'checkpoint_schema_version': (
                'room315.visual_training.v4.checkpoint.v1'
            ),
            'model_kind': 'room315_visual_state_resnet18_split_rails_v4',
            'cross_camera_feature_path': False,
            'side_source': 'fixed_identity_prefix',
        },
        'manual_decision_record': {
            'path': 'manual_decision_record.json',
            'schema_version': (
                execution_config
                .V4_TASK_EXECUTION_MANUAL_DECISION_SCHEMA_VERSION
            ),
            'sha256': manual_sha256,
        },
    }


def _authorization_payload(promotion_sha256: str) -> dict[str, Any]:
    return {
        'authorization_scope': (
            execution_config.V4_TASK_EXECUTION_AUTHORIZATION_SCOPE
        ),
        'automatic_promotion_allowed': False,
        'candidate_id': execution_config.V4_TASK_EXECUTION_CANDIDATE_ID,
        'checkpoint_filename': 'checkpoint_epoch_011.pt',
        'checkpoint_sha256': (
            execution_config.V4_TASK_EXECUTION_CHECKPOINT_SHA256
        ),
        'deployment_mode': 'active',
        'manual_review_approved': True,
        'physical_deployment_approved': False,
        'promotion_manifest_sha256': promotion_sha256,
        'runtime_guards': copy.deepcopy(
            execution_config.V4_TASK_EXECUTION_RUNTIME_GUARDS
        ),
        'schema_version': (
            execution_config.V4_TASK_EXECUTION_AUTHORIZATION_SCHEMA_VERSION
        ),
        'state': 'active_closed_loop_qualification',
    }


def _write_json(path: Path, payload: dict[str, Any], mode: int = 0o444) -> str:
    raw = json.dumps(
        payload,
        sort_keys=True,
        separators=(',', ':'),
    ).encode('utf-8')
    path.write_bytes(raw)
    path.chmod(mode)
    return hashlib.sha256(raw).hexdigest()


def _build_bundle(
    tmp_path: Path,
    monkeypatch,
    *,
    manual_overrides: dict[str, Any] | None = None,
    promotion_overrides: dict[str, Any] | None = None,
    authorization_overrides: dict[str, Any] | None = None,
    authorization_mode: int = 0o444,
    promotion_mode: int = 0o444,
    manual_mode: int = 0o444,
) -> dict[str, Any]:
    tmp_path.mkdir(parents=True, exist_ok=True)
    manual = _manual_decision_payload()
    manual.update(manual_overrides or {})
    manual_path = tmp_path / 'manual_decision_record.json'
    manual_sha256 = _write_json(manual_path, manual, manual_mode)

    promotion = _promotion_payload(manual_sha256)
    promotion.update(promotion_overrides or {})
    promotion_path = tmp_path / 'runtime_promotion_manifest.json'
    promotion_sha256 = _write_json(
        promotion_path,
        promotion,
        promotion_mode,
    )
    monkeypatch.setattr(
        execution_config,
        'V4_TASK_EXECUTION_PROMOTION_MANIFEST_SHA256',
        promotion_sha256,
    )

    authorization = _authorization_payload(promotion_sha256)
    authorization.update(authorization_overrides or {})
    authorization_path = tmp_path / 'candidate_state.json'
    authorization_sha256 = _write_json(
        authorization_path,
        authorization,
        authorization_mode,
    )
    parameters = copy.deepcopy(
        execution_config.TASK_EXECUTION_PARAMETER_DEFAULTS
    )
    parameters.update({
        'execution_enabled': True,
        'allowed_visual_schema_version': (
            execution_config.V4_TASK_EXECUTION_VISUAL_SCHEMA_VERSION
        ),
        'allowed_visual_checkpoint_sha256': (
            execution_config.V4_TASK_EXECUTION_CHECKPOINT_SHA256
        ),
        'task_execution_authorization_path': str(authorization_path),
        'task_execution_authorization_sha256': authorization_sha256,
        'task_execution_promotion_manifest_path': str(promotion_path),
    })
    return {
        'parameters': parameters,
        'authorization_path': authorization_path,
        'authorization_sha256': authorization_sha256,
        'promotion_path': promotion_path,
        'promotion_sha256': promotion_sha256,
        'manual_path': manual_path,
        'manual_sha256': manual_sha256,
    }


def _runtime_campaign_payload() -> dict[str, Any]:
    return {
        'schema_version': (
            execution_config
            .V4_TASK_EXECUTION_CLOSED_LOOP_CAMPAIGN_SCHEMA_VERSION
        ),
        'status': 'passed',
        'visual_schema_version': (
            execution_config.V4_TASK_EXECUTION_VISUAL_SCHEMA_VERSION
        ),
        'checkpoint_sha256': (
            execution_config.V4_TASK_EXECUTION_CHECKPOINT_SHA256
        ),
        'qualification_manifest_sha256': (
            execution_config.V4_TASK_EXECUTION_PROMOTION_MANIFEST_SHA256
        ),
        'case_count': 12,
        'passed_case_count': 12,
        'failed_case_count': 0,
        'v3_observation_count': 0,
        'v4_observation_count': 1784,
        'physical_deployment': False,
        'all_terminal_statuses_succeeded': True,
        'all_final_effects_verified': True,
        'all_controllers_stopped': True,
        'safe_abort_count': 0,
    }


def _runtime_fault_payload() -> dict[str, Any]:
    results = []
    for scenario_id, fault in (
        execution_config.V4_TASK_EXECUTION_FAULT_SCENARIOS
    ):
        results.append({
            'scenario_id': scenario_id,
            'fault': fault,
            'status': 'passed',
            'visual_schema_version': (
                execution_config.V4_TASK_EXECUTION_VISUAL_SCHEMA_VERSION
            ),
            'checkpoint_sha256': (
                execution_config.V4_TASK_EXECUTION_CHECKPOINT_SHA256
            ),
            'qualification_manifest_sha256': (
                execution_config.V4_TASK_EXECUTION_PROMOTION_MANIFEST_SHA256
            ),
            'false_success_count': 0,
            'v3_observation_count': 0,
            'v4_observation_count': 1,
            'controller_final_mode': 'DISABLED',
            'physical_deployment': False,
        })
    scenario_count = len(results)
    return {
        'schema_version': (
            execution_config.V4_TASK_EXECUTION_FAULT_CAMPAIGN_SCHEMA_VERSION
        ),
        'status': 'passed',
        'authorization_scope': (
            execution_config.V4_TASK_EXECUTION_AUTHORIZATION_SCOPE
        ),
        'visual_schema_version': (
            execution_config.V4_TASK_EXECUTION_VISUAL_SCHEMA_VERSION
        ),
        'visual_checkpoint_sha256': (
            execution_config.V4_TASK_EXECUTION_CHECKPOINT_SHA256
        ),
        'qualification_manifest_sha256': (
            execution_config.V4_TASK_EXECUTION_PROMOTION_MANIFEST_SHA256
        ),
        'declared_scenario_count': scenario_count,
        'selected_scenario_count': scenario_count,
        'completed_scenario_count': scenario_count,
        'passed_scenario_count': scenario_count,
        'failed_scenario_count': 0,
        'v3_observation_count': 0,
        'v4_observation_count': scenario_count,
        'all_false_success_counts_zero': True,
        'all_controllers_disabled': True,
        'physical_deployment': False,
        'failure_reason': '',
        'results': results,
    }


def _write_bytes(path: Path, raw: bytes, mode: int = 0o444) -> str:
    path.write_bytes(raw)
    path.chmod(mode)
    return hashlib.sha256(raw).hexdigest()


def _build_runtime_bundle(
    tmp_path: Path,
    *,
    campaign_overrides: dict[str, Any] | None = None,
    fault_overrides: dict[str, Any] | None = None,
    manual_overrides: dict[str, Any] | None = None,
    promotion_overrides: dict[str, Any] | None = None,
    authorization_overrides: dict[str, Any] | None = None,
    modes: dict[str, int] | None = None,
) -> dict[str, Any]:
    tmp_path.mkdir(parents=True, exist_ok=True)
    modes = modes or {}
    campaign = _runtime_campaign_payload()
    campaign.update(campaign_overrides or {})
    campaign_path = tmp_path / 'closed_loop_campaign_report.json'
    campaign_sha256 = _write_json(
        campaign_path,
        campaign,
        modes.get('campaign', 0o444),
    )

    fault = _runtime_fault_payload()
    fault.update(fault_overrides or {})
    fault_path = tmp_path / 'closed_loop_fault_campaign_report.json'
    fault_sha256 = _write_json(
        fault_path,
        fault,
        modes.get('fault', 0o444),
    )

    evidence_rows = [{
        'path': 'summary.json',
        'sha256': fault_sha256,
        'size_bytes': fault_path.stat().st_size,
    }]
    for index, (scenario_id, _fault) in enumerate(
        execution_config.V4_TASK_EXECUTION_FAULT_SCENARIOS,
        start=1,
    ):
        evidence_rows.append({
            'path': f'cases/{scenario_id}/case_summary.json',
            'sha256': f'{index:x}' * 64,
            'size_bytes': index,
        })
    evidence_manifest = {
        'schema_version': (
            execution_config.V4_TASK_EXECUTION_FAULT_EVIDENCE_SCHEMA_VERSION
        ),
        'campaign_schema': (
            execution_config.V4_TASK_EXECUTION_FAULT_CAMPAIGN_SCHEMA_VERSION
        ),
        'campaign_status': 'passed',
        'physical_deployment': False,
        'file_count_excluding_manifest_and_checksums': len(evidence_rows),
        'files': evidence_rows,
    }
    evidence_manifest_path = (
        tmp_path / 'closed_loop_fault_campaign_evidence_manifest.json'
    )
    evidence_manifest_sha256 = _write_json(
        evidence_manifest_path,
        evidence_manifest,
        modes.get('evidence_manifest', 0o444),
    )
    sums_rows = [
        f'{row["sha256"]}  {row["path"]}\n'
        for row in evidence_rows
    ]
    sums_rows.append(
        f'{evidence_manifest_sha256}  manifest.json\n'
    )
    source_sums_path = (
        tmp_path / 'closed_loop_fault_campaign_source_SHA256SUMS'
    )
    source_sums_sha256 = _write_bytes(
        source_sums_path,
        ''.join(sums_rows).encode('utf-8'),
        modes.get('source_sums', 0o444),
    )

    scenario_ids = [
        scenario_id
        for scenario_id, _fault
        in execution_config.V4_TASK_EXECUTION_FAULT_SCENARIOS
    ]
    manual = {
        'schema_version': (
            execution_config.V4_TASK_EXECUTION_MANUAL_DECISION_SCHEMA_VERSION
        ),
        'scope': (
            execution_config.V4_TASK_EXECUTION_RUNTIME_AUTHORIZATION_SCOPE
        ),
        'candidate_id': execution_config.V4_TASK_EXECUTION_CANDIDATE_ID,
        'checkpoint_sha256': (
            execution_config.V4_TASK_EXECUTION_CHECKPOINT_SHA256
        ),
        'decision': 'approved',
        'reviewer': 'operator',
        'reviewed_at_utc': '2026-08-12T00:00:00Z',
        'qualification_only': False,
        'physical_deployment_approved': False,
        'automatic_promotion': False,
        'source_active_manifest_sha256': (
            execution_config.V4_TASK_EXECUTION_PROMOTION_MANIFEST_SHA256
        ),
        'runtime_guards': copy.deepcopy(
            execution_config.V4_TASK_EXECUTION_RUNTIME_GUARDS
        ),
        'closed_loop_campaign': {
            'schema_version': (
                execution_config
                .V4_TASK_EXECUTION_CLOSED_LOOP_CAMPAIGN_SCHEMA_VERSION
            ),
            'sha256': campaign_sha256,
            'case_count': 12,
            'passed_case_count': 12,
            'failed_case_count': 0,
        },
        'closed_loop_fault_campaign': {
            'schema_version': (
                execution_config
                .V4_TASK_EXECUTION_FAULT_CAMPAIGN_SCHEMA_VERSION
            ),
            'sha256': fault_sha256,
            'scenario_ids': scenario_ids,
            'passed_scenario_count': len(scenario_ids),
            'failed_scenario_count': 0,
            'evidence_manifest_sha256': evidence_manifest_sha256,
            'evidence_sha256s_sha256': source_sums_sha256,
        },
        'review_assertions': {
            'v4_is_authoritative_visual_observation': True,
            'plansys2_problem_built_from_v4_observed_state': True,
            'optional_problem_expert_predicate_mirror_disabled': True,
            'supervisor_and_dzi_safety_gates_required': True,
        },
    }
    manual.update(manual_overrides or {})
    manual_path = tmp_path / 'manual_decision_record.json'
    manual_sha256 = _write_json(
        manual_path,
        manual,
        modes.get('manual', 0o444),
    )

    promotion = _promotion_payload(manual_sha256)
    promotion.update({
        'source_active_manifest_sha256': (
            execution_config.V4_TASK_EXECUTION_PROMOTION_MANIFEST_SHA256
        ),
        'closed_loop_qualification': {
            'qualification_only': False,
            'physical_deployment_approved': False,
            'campaign_report_path': campaign_path.name,
            'campaign_report_sha256': campaign_sha256,
        },
        'closed_loop_fault_campaign': {
            'schema_version': (
                execution_config
                .V4_TASK_EXECUTION_FAULT_CAMPAIGN_SCHEMA_VERSION
            ),
            'report_path': fault_path.name,
            'report_sha256': fault_sha256,
            'scenario_ids': scenario_ids,
            'passed_scenario_count': len(scenario_ids),
            'failed_scenario_count': 0,
            'evidence_manifest_path': evidence_manifest_path.name,
            'evidence_manifest_sha256': evidence_manifest_sha256,
            'source_sha256s_path': source_sums_path.name,
            'source_sha256s_sha256': source_sums_sha256,
            'physical_deployment': False,
        },
    })
    promotion.update(promotion_overrides or {})
    promotion_path = tmp_path / 'runtime_promotion_manifest.json'
    promotion_sha256 = _write_json(
        promotion_path,
        promotion,
        modes.get('promotion', 0o444),
    )

    authorization = _authorization_payload(promotion_sha256)
    authorization.update({
        'authorization_scope': (
            execution_config.V4_TASK_EXECUTION_RUNTIME_AUTHORIZATION_SCOPE
        ),
        'state': 'active_closed_loop_runtime',
    })
    authorization.update(authorization_overrides or {})
    authorization_path = tmp_path / 'candidate_state.json'
    authorization_sha256 = _write_json(
        authorization_path,
        authorization,
        modes.get('authorization', 0o444),
    )
    parameters = copy.deepcopy(
        execution_config.TASK_EXECUTION_PARAMETER_DEFAULTS
    )
    parameters.update({
        'execution_enabled': True,
        'allowed_visual_schema_version': (
            execution_config.V4_TASK_EXECUTION_VISUAL_SCHEMA_VERSION
        ),
        'allowed_visual_checkpoint_sha256': (
            execution_config.V4_TASK_EXECUTION_CHECKPOINT_SHA256
        ),
        'task_execution_authorization_path': str(authorization_path),
        'task_execution_authorization_sha256': authorization_sha256,
        'task_execution_promotion_manifest_path': str(promotion_path),
    })
    return {
        'parameters': parameters,
        'authorization_path': authorization_path,
        'authorization_sha256': authorization_sha256,
        'promotion_path': promotion_path,
        'promotion_sha256': promotion_sha256,
        'manual_path': manual_path,
        'manual_sha256': manual_sha256,
        'campaign_path': campaign_path,
        'campaign_sha256': campaign_sha256,
        'fault_path': fault_path,
        'fault_sha256': fault_sha256,
        'evidence_manifest_path': evidence_manifest_path,
        'evidence_manifest_sha256': evidence_manifest_sha256,
        'source_sums_path': source_sums_path,
        'source_sums_sha256': source_sums_sha256,
    }


def test_frozen_v4_qualification_identifiers_are_exact():
    assert execution_config.V4_TASK_EXECUTION_CHECKPOINT_SHA256 == (
        '869d64049b0092c37d21a4c8b910dc6b91954527e0e49c5694fa82dce570f40d'
    )
    assert execution_config.V4_TASK_EXECUTION_PROMOTION_MANIFEST_SHA256 == (
        '6f9828219c22599825f5a14e405c8f11ce017984cc0d65821a357240d6529e2a'
    )
    assert execution_config.V4_TASK_EXECUTION_AUTHORIZATION_SCOPE == (
        'gazebo_v4_closed_loop_qualification_only'
    )


def test_execution_disabled_may_omit_all_authorization_parameters():
    parameters = copy.deepcopy(
        execution_config.TASK_EXECUTION_PARAMETER_DEFAULTS
    )
    for name in execution_config.TASK_EXECUTION_AUTHORIZATION_PARAMETER_DEFAULTS:
        parameters.pop(name)

    assert execution_config.validate_task_execution_parameters(parameters) is None


def test_enabled_accepts_exact_immutable_qualification_bundle(
    tmp_path,
    monkeypatch,
):
    bundle = _build_bundle(tmp_path, monkeypatch)

    verified = execution_config.validate_task_execution_parameters(
        bundle['parameters']
    )

    assert verified == {
        'path': str(bundle['authorization_path']),
        'sha256': bundle['authorization_sha256'],
        'promotion_manifest_path': str(bundle['promotion_path']),
        'visual_state_schema_version': (
            execution_config.V4_TASK_EXECUTION_VISUAL_SCHEMA_VERSION
        ),
        'visual_checkpoint_sha256': (
            execution_config.V4_TASK_EXECUTION_CHECKPOINT_SHA256
        ),
        'promotion_manifest_sha256': bundle['promotion_sha256'],
        'manual_decision_path': str(bundle['manual_path']),
        'manual_decision_sha256': bundle['manual_sha256'],
        'authorization_scope': (
            execution_config.V4_TASK_EXECUTION_AUTHORIZATION_SCOPE
        ),
    }


def test_enabled_requires_authorization_and_promotion_paths(
    tmp_path,
    monkeypatch,
):
    bundle = _build_bundle(tmp_path, monkeypatch)
    parameters = bundle['parameters']
    parameters['task_execution_authorization_path'] = ''
    with pytest.raises(ValueError, match='requires.*authorization_path'):
        execution_config.validate_task_execution_parameters(parameters)

    parameters['task_execution_authorization_path'] = str(
        bundle['authorization_path']
    )
    parameters['task_execution_promotion_manifest_path'] = ''
    with pytest.raises(ValueError, match='requires.*promotion_manifest_path'):
        execution_config.validate_task_execution_parameters(parameters)


@pytest.mark.parametrize(
    ('parameter_name', 'value', 'message'),
    (
        (
            'allowed_visual_schema_version',
            'room315.visual_state.v3',
            'allowed_visual_schema_version must be room315.visual_state.v4',
        ),
        (
            'allowed_visual_checkpoint_sha256',
            '8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d',
            'allowed_visual_checkpoint_sha256 must be the exact authorized V4 checkpoint',
        ),
    ),
)
def test_enabled_rejects_non_v4_visual_allowlist(
    tmp_path,
    monkeypatch,
    parameter_name,
    value,
    message,
):
    bundle = _build_bundle(tmp_path, monkeypatch)
    bundle['parameters'][parameter_name] = value

    with pytest.raises(ValueError, match=message):
        execution_config.validate_task_execution_parameters(
            bundle['parameters']
        )


@pytest.mark.parametrize(
    ('field', 'value'),
    (
        ('schema_version', 'room315.deployment_candidate_state.v3.v1'),
        ('authorization_scope', 'physical_closed_loop'),
        ('automatic_promotion_allowed', True),
        ('candidate_id', 'other_candidate'),
        ('checkpoint_filename', 'other.pt'),
        ('checkpoint_sha256', '0' * 64),
        ('deployment_mode', 'shadow'),
        ('manual_review_approved', False),
        ('physical_deployment_approved', True),
        ('promotion_manifest_sha256', '0' * 64),
        ('runtime_guards', {'actuation_enabled': False}),
        ('state', 'active'),
    ),
)
def test_authorization_contract_fields_are_exact(
    tmp_path,
    monkeypatch,
    field,
    value,
):
    bundle = _build_bundle(
        tmp_path,
        monkeypatch,
        authorization_overrides={field: value},
    )

    with pytest.raises(ValueError, match=f'field mismatch: {field}'):
        execution_config.validate_task_execution_parameters(
            bundle['parameters']
        )


@pytest.mark.parametrize(
    ('field', 'value'),
    (
        ('scope', 'gazebo_closed_loop_only'),
        ('decision', 'rejected'),
        ('qualification_only', False),
        ('physical_deployment_approved', True),
        ('automatic_promotion', True),
        ('checkpoint_sha256', '0' * 64),
        ('runtime_guards', {'actuation_enabled': False}),
    ),
)
def test_manual_decision_scope_and_guards_are_exact(
    tmp_path,
    monkeypatch,
    field,
    value,
):
    bundle = _build_bundle(
        tmp_path,
        monkeypatch,
        manual_overrides={field: value},
    )

    with pytest.raises(
        ValueError,
        match=f'manual decision field mismatch: {field}',
    ):
        execution_config.validate_task_execution_parameters(
            bundle['parameters']
        )


def test_authorization_file_must_be_read_only(tmp_path, monkeypatch):
    bundle = _build_bundle(
        tmp_path,
        monkeypatch,
        authorization_mode=0o644,
    )

    with pytest.raises(ValueError, match='must have no write bits'):
        execution_config.validate_task_execution_parameters(
            bundle['parameters']
        )


def test_promotion_manifest_and_manual_record_are_opened_read_only(
    tmp_path,
    monkeypatch,
):
    promotion_bundle = _build_bundle(
        tmp_path / 'promotion',
        monkeypatch,
        promotion_mode=0o644,
    )
    with pytest.raises(
        ValueError,
        match='promotion manifest must have no write bits',
    ):
        execution_config.validate_task_execution_parameters(
            promotion_bundle['parameters']
        )

    manual_bundle = _build_bundle(
        tmp_path / 'manual',
        monkeypatch,
        manual_mode=0o644,
    )
    with pytest.raises(
        ValueError,
        match='manual decision must have no write bits',
    ):
        execution_config.validate_task_execution_parameters(
            manual_bundle['parameters']
        )


def test_authorization_hash_and_final_symlink_are_rejected(
    tmp_path,
    monkeypatch,
):
    bundle = _build_bundle(tmp_path, monkeypatch)
    bundle['parameters']['task_execution_authorization_sha256'] = '0' * 64
    with pytest.raises(ValueError, match='authorization SHA-256 mismatch'):
        execution_config.validate_task_execution_parameters(
            bundle['parameters']
        )

    bundle['parameters']['task_execution_authorization_sha256'] = (
        bundle['authorization_sha256']
    )
    link = tmp_path / 'candidate_state_link.json'
    os.symlink(bundle['authorization_path'], link)
    bundle['parameters']['task_execution_authorization_path'] = str(link)
    with pytest.raises(ValueError, match='cannot be opened'):
        execution_config.validate_task_execution_parameters(
            bundle['parameters']
        )


def test_enabled_accepts_sha_pinned_runtime_bundle_without_frozen_manifest_sha(
    tmp_path,
):
    bundle = _build_runtime_bundle(tmp_path)

    verified = execution_config.validate_task_execution_parameters(
        bundle['parameters']
    )

    assert verified['authorization_scope'] == (
        execution_config.V4_TASK_EXECUTION_RUNTIME_AUTHORIZATION_SCOPE
    )
    assert verified['promotion_manifest_sha256'] == bundle['promotion_sha256']
    assert verified['closed_loop_campaign_report_sha256'] == (
        bundle['campaign_sha256']
    )
    assert verified['fault_campaign_report_sha256'] == bundle['fault_sha256']
    assert verified['fault_evidence_manifest_sha256'] == (
        bundle['evidence_manifest_sha256']
    )
    assert verified['fault_evidence_sha256s_sha256'] == (
        bundle['source_sums_sha256']
    )
    assert bundle['promotion_sha256'] != (
        execution_config.V4_TASK_EXECUTION_PROMOTION_MANIFEST_SHA256
    )


@pytest.mark.parametrize(
    ('overrides', 'message'),
    (
        (
            {'state': 'active_closed_loop_qualification'},
            'field mismatch: state',
        ),
        (
            {
                'authorization_scope': (
                    execution_config.V4_TASK_EXECUTION_AUTHORIZATION_SCOPE
                ),
                'state': 'active_closed_loop_qualification',
            },
            'field mismatch: promotion_manifest_sha256',
        ),
        (
            {'promotion_manifest_sha256': '0' * 64},
            'promotion manifest SHA-256 mismatch',
        ),
    ),
)
def test_runtime_candidate_state_rejects_qualification_or_wrong_binding(
    tmp_path,
    overrides,
    message,
):
    bundle = _build_runtime_bundle(
        tmp_path,
        authorization_overrides=overrides,
    )

    with pytest.raises(ValueError, match=message):
        execution_config.validate_task_execution_parameters(
            bundle['parameters']
        )


def test_runtime_manifest_requires_exact_source_qualification(tmp_path):
    bundle = _build_runtime_bundle(
        tmp_path,
        promotion_overrides={'source_active_manifest_sha256': '0' * 64},
    )

    with pytest.raises(
        ValueError,
        match='field mismatch: source_active_manifest_sha256',
    ):
        execution_config.validate_task_execution_parameters(
            bundle['parameters']
        )


@pytest.mark.parametrize(
    'artifact_name',
    ('campaign_path', 'fault_path', 'evidence_manifest_path', 'source_sums_path'),
)
def test_runtime_rejects_missing_bound_evidence(tmp_path, artifact_name):
    bundle = _build_runtime_bundle(tmp_path)
    bundle[artifact_name].unlink()

    with pytest.raises(ValueError, match='cannot be opened'):
        execution_config.validate_task_execution_parameters(
            bundle['parameters']
        )


@pytest.mark.parametrize(
    ('mode_name', 'label'),
    (
        ('campaign', 'campaign report'),
        ('fault', 'fault-campaign report'),
        ('evidence_manifest', 'fault-evidence manifest'),
        ('source_sums', 'source SHA256SUMS'),
    ),
)
def test_runtime_rejects_writable_bound_evidence(
    tmp_path,
    mode_name,
    label,
):
    bundle = _build_runtime_bundle(tmp_path, modes={mode_name: 0o644})

    with pytest.raises(ValueError, match=f'{label} must have no write bits'):
        execution_config.validate_task_execution_parameters(
            bundle['parameters']
        )


@pytest.mark.parametrize(
    'artifact_name',
    ('campaign_path', 'fault_path', 'evidence_manifest_path', 'source_sums_path'),
)
def test_runtime_rejects_tampered_bound_evidence(tmp_path, artifact_name):
    bundle = _build_runtime_bundle(tmp_path)
    artifact = bundle[artifact_name]
    artifact.chmod(0o644)
    artifact.write_bytes(artifact.read_bytes() + b'\n')
    artifact.chmod(0o444)

    with pytest.raises(ValueError, match='SHA-256 mismatch'):
        execution_config.validate_task_execution_parameters(
            bundle['parameters']
        )


@pytest.mark.parametrize(
    ('manual_overrides', 'message'),
    (
        (
            {'scope': execution_config.V4_TASK_EXECUTION_AUTHORIZATION_SCOPE},
            'manual decision field mismatch: scope',
        ),
        (
            {'qualification_only': True},
            'manual decision field mismatch: qualification_only',
        ),
        (
            {'runtime_guards': {'actuation_enabled': False}},
            'manual decision field mismatch: runtime_guards',
        ),
        (
            {'physical_deployment_approved': True},
            'manual decision field mismatch: physical_deployment_approved',
        ),
    ),
)
def test_runtime_manual_decision_is_exact(
    tmp_path,
    manual_overrides,
    message,
):
    bundle = _build_runtime_bundle(
        tmp_path,
        manual_overrides=manual_overrides,
    )

    with pytest.raises(ValueError, match=message):
        execution_config.validate_task_execution_parameters(
            bundle['parameters']
        )


@pytest.mark.parametrize(
    ('kind', 'overrides', 'message'),
    (
        ('campaign', {'status': 'partial'}, 'campaign report field mismatch'),
        (
            'campaign',
            {'v3_observation_count': 1},
            'campaign report field mismatch',
        ),
        ('fault', {'status': 'failed'}, 'fault-campaign report field mismatch'),
        (
            'fault',
            {'all_controllers_disabled': False},
            'fault-campaign report field mismatch',
        ),
    ),
)
def test_runtime_rejects_failed_campaign_contracts(
    tmp_path,
    kind,
    overrides,
    message,
):
    arguments = {f'{kind}_overrides': overrides}
    bundle = _build_runtime_bundle(tmp_path, **arguments)

    with pytest.raises(ValueError, match=message):
        execution_config.validate_task_execution_parameters(
            bundle['parameters']
        )
