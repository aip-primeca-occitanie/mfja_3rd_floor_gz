from __future__ import annotations

import hashlib
import json
import os
import sys
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import room_315_v4_closed_loop_fault_campaign as fault_producer

from room_315_authorize_runtime_v4_closed_loop import CAMPAIGN_SCHEMA
from room_315_authorize_runtime_v4_closed_loop import CLOSED_LOOP_QUALIFICATION_SCOPE
from room_315_authorize_runtime_v4_closed_loop import CLOSED_LOOP_RUNTIME_SCOPE
from room_315_authorize_runtime_v4_closed_loop import ClosedLoopAuthorizationError
from room_315_authorize_runtime_v4_closed_loop import EXPECTED_CASE_COUNT
from room_315_authorize_runtime_v4_closed_loop import EXPECTED_FAULT_SCENARIOS
from room_315_authorize_runtime_v4_closed_loop import FAULT_CAMPAIGN_SCHEMA
from room_315_authorize_runtime_v4_closed_loop import FAULT_EVIDENCE_SCHEMA
from room_315_authorize_runtime_v4_closed_loop import V4_SCHEMA
from room_315_authorize_runtime_v4_closed_loop import _decision_record
from room_315_authorize_runtime_v4_closed_loop import _load_json
from room_315_authorize_runtime_v4_closed_loop import _verify_campaign
from room_315_authorize_runtime_v4_closed_loop import _verify_fault_campaign
from room_315_authorize_runtime_v4_closed_loop import _verify_fault_evidence_sidecars


CHECKPOINT_SHA256 = '8' * 64
QUALIFICATION_SHA256 = '6' * 64


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _campaign_payload() -> dict[str, object]:
    return {
        'schema_version': CAMPAIGN_SCHEMA,
        'status': 'passed',
        'visual_schema_version': V4_SCHEMA,
        'checkpoint_sha256': CHECKPOINT_SHA256,
        'qualification_manifest_sha256': QUALIFICATION_SHA256,
        'case_count': EXPECTED_CASE_COUNT,
        'passed_case_count': EXPECTED_CASE_COUNT,
        'failed_case_count': 0,
        'v3_observation_count': 0,
        'physical_deployment': False,
        'all_terminal_statuses_succeeded': True,
        'all_final_effects_verified': True,
        'all_controllers_stopped': True,
        'safe_abort_count': 0,
    }


def _fault_campaign_payload() -> dict[str, object]:
    results = [
        {
            'schema_version': fault_producer.CASE_SCHEMA,
            'scenario_id': scenario.scenario_id,
            'fault': scenario.fault,
            'status': 'passed',
            'authorization_scope': CLOSED_LOOP_QUALIFICATION_SCOPE,
            'visual_schema_version': V4_SCHEMA,
            'checkpoint_sha256': CHECKPOINT_SHA256,
            'qualification_manifest_sha256': QUALIFICATION_SHA256,
            'physical_deployment': False,
            'gateway_refused': scenario.expected_terminal == 'gateway_refused',
            'terminal_status': (
                None if scenario.expected_terminal == 'gateway_refused'
                else 'failed'
            ),
            'false_success_count': 0,
            'actuating_command_count': (
                1 if scenario.expect_actuating_command else 0
            ),
            'zero_motion_proved': not scenario.expect_actuating_command,
            'controller_final_mode': 'DISABLED',
            'v4_observation_count': 2,
            'v3_observation_count': 0,
        }
        for scenario in fault_producer.FAULT_SCENARIOS
    ]
    return {
        'schema_version': fault_producer.CAMPAIGN_SCHEMA,
        'status': 'passed',
        'authorization_scope': CLOSED_LOOP_QUALIFICATION_SCOPE,
        'visual_schema_version': V4_SCHEMA,
        'visual_checkpoint_sha256': CHECKPOINT_SHA256,
        'qualification_manifest_sha256': QUALIFICATION_SHA256,
        'physical_deployment': False,
        'declared_scenario_count': len(fault_producer.FAULT_SCENARIOS),
        'selected_scenario_count': len(fault_producer.FAULT_SCENARIOS),
        'completed_scenario_count': len(fault_producer.FAULT_SCENARIOS),
        'passed_scenario_count': len(fault_producer.FAULT_SCENARIOS),
        'failed_scenario_count': 0,
        'all_false_success_counts_zero': True,
        'all_controllers_disabled': True,
        'v3_observation_count': 0,
        'v4_observation_count': sum(
            int(result['v4_observation_count']) for result in results
        ),
        'failure_reason': '',
        'results': results,
    }


def _write_read_only(path: Path, payload: object) -> str:
    path.write_text(
        json.dumps(payload, sort_keys=True) + '\n', encoding='utf-8'
    )
    os.chmod(path, 0o444)
    return _sha256(path)


def _write_fault_evidence(
    tmp_path: Path,
    payload: dict[str, object] | None = None,
    *,
    include_summary_manifest_row: bool = True,
) -> tuple[Path, str]:
    root = tmp_path / 'fault_evidence'
    root.mkdir()
    summary = root / 'summary.json'
    summary.write_text(
        json.dumps(payload or _fault_campaign_payload(), indent=2, sort_keys=True)
        + '\n',
        encoding='utf-8',
    )
    summary_sha256 = _sha256(summary)
    rows = []
    if include_summary_manifest_row:
        rows.append({
            'path': 'summary.json',
            'size_bytes': summary.stat().st_size,
            'sha256': summary_sha256,
        })
    manifest = root / 'manifest.json'
    manifest.write_text(
        json.dumps({
            'schema_version': FAULT_EVIDENCE_SCHEMA,
            'campaign_schema': FAULT_CAMPAIGN_SCHEMA,
            'campaign_status': 'passed',
            'physical_deployment': False,
            'file_count_excluding_manifest_and_checksums': len(rows),
            'files': rows,
        }, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    checksums = root / 'SHA256SUMS'
    checksums.write_text(
        f'{_sha256(manifest)}  manifest.json\n'
        f'{summary_sha256}  summary.json\n',
        encoding='utf-8',
    )
    for path in (summary, manifest, checksums):
        os.chmod(path, 0o444)
    os.chmod(root, 0o555)
    return summary, summary_sha256


def test_exact_closed_loop_campaign_is_accepted(tmp_path: Path) -> None:
    path = tmp_path / 'campaign.json'
    digest = _write_read_only(path, _campaign_payload())

    verified = _verify_campaign(
        path,
        digest,
        qualification_manifest_sha256=QUALIFICATION_SHA256,
        checkpoint_sha256=CHECKPOINT_SHA256,
    )

    assert verified['passed_case_count'] == EXPECTED_CASE_COUNT


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('status', 'partial'),
        ('visual_schema_version', 'room315.visual_state.v3'),
        ('checkpoint_sha256', '0' * 64),
        ('qualification_manifest_sha256', '1' * 64),
        ('case_count', 11),
        ('passed_case_count', 11),
        ('failed_case_count', 1),
        ('v3_observation_count', 1),
        ('physical_deployment', True),
        ('all_terminal_statuses_succeeded', False),
        ('all_final_effects_verified', False),
        ('all_controllers_stopped', False),
        ('safe_abort_count', 1),
    ],
)
def test_campaign_contract_rejects_any_failed_guard(
    tmp_path: Path,
    field: str,
    value: object,
) -> None:
    payload = _campaign_payload()
    payload[field] = value
    path = tmp_path / 'campaign.json'
    digest = _write_read_only(path, payload)

    with pytest.raises(ClosedLoopAuthorizationError):
        _verify_campaign(
            path,
            digest,
            qualification_manifest_sha256=QUALIFICATION_SHA256,
            checkpoint_sha256=CHECKPOINT_SHA256,
        )


@pytest.mark.parametrize('value', [False, 12.0, '12'])
def test_campaign_counts_require_json_integers(
    tmp_path: Path,
    value: object,
) -> None:
    payload = _campaign_payload()
    payload['case_count'] = value
    path = tmp_path / 'campaign.json'
    digest = _write_read_only(path, payload)

    with pytest.raises(ClosedLoopAuthorizationError, match='case_count'):
        _verify_campaign(
            path,
            digest,
            qualification_manifest_sha256=QUALIFICATION_SHA256,
            checkpoint_sha256=CHECKPOINT_SHA256,
        )


def test_campaign_report_must_be_read_only(tmp_path: Path) -> None:
    path = tmp_path / 'campaign.json'
    path.write_text(json.dumps(_campaign_payload()), encoding='utf-8')

    with pytest.raises(ClosedLoopAuthorizationError, match='read-only'):
        _verify_campaign(
            path,
            _sha256(path),
            qualification_manifest_sha256=QUALIFICATION_SHA256,
            checkpoint_sha256=CHECKPOINT_SHA256,
        )


def test_fault_consumer_contract_matches_fault_campaign_producer() -> None:
    assert FAULT_CAMPAIGN_SCHEMA == fault_producer.CAMPAIGN_SCHEMA
    assert FAULT_EVIDENCE_SCHEMA == (
        'room315.v4_closed_loop_fault_evidence.v1'
    )
    assert EXPECTED_FAULT_SCENARIOS == tuple(
        (scenario.scenario_id, scenario.fault)
        for scenario in fault_producer.FAULT_SCENARIOS
    )


def test_exact_fault_campaign_and_complete_sidecars_are_accepted(
    tmp_path: Path,
) -> None:
    path, digest = _write_fault_evidence(tmp_path)

    verified, resolved_path = _verify_fault_campaign(
        path,
        digest,
        qualification_manifest_sha256=QUALIFICATION_SHA256,
        checkpoint_sha256=CHECKPOINT_SHA256,
    )
    sidecars = _verify_fault_evidence_sidecars(resolved_path, digest)

    assert verified['passed_scenario_count'] == len(EXPECTED_FAULT_SCENARIOS)
    assert [item['scenario_id'] for item in verified['results']] == [
        'F01', 'F02', 'F03', 'F04', 'F05',
    ]
    assert sidecars['manifest_sha256'] == _sha256(
        path.parent / 'manifest.json'
    )
    assert sidecars['sha256s_sha256'] == _sha256(
        path.parent / 'SHA256SUMS'
    )


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('schema_version', 'room315.v4_closed_loop_fault_campaign.v0'),
        ('status', 'partial'),
        ('authorization_scope', 'gazebo_runtime_dry_run_only'),
        ('visual_schema_version', 'room315.visual_state.v3'),
        ('visual_checkpoint_sha256', '0' * 64),
        ('qualification_manifest_sha256', '1' * 64),
        ('declared_scenario_count', 4),
        ('selected_scenario_count', 4),
        ('completed_scenario_count', 4),
        ('passed_scenario_count', 4),
        ('failed_scenario_count', 1),
        ('all_false_success_counts_zero', False),
        ('all_controllers_disabled', False),
        ('v3_observation_count', 1),
        ('v4_observation_count', 4),
        ('physical_deployment', True),
        ('failure_reason', 'unexpected fault result'),
    ],
)
def test_fault_campaign_rejects_any_summary_guard_failure(
    tmp_path: Path,
    field: str,
    value: object,
) -> None:
    payload = _fault_campaign_payload()
    payload[field] = value
    path, digest = _write_fault_evidence(tmp_path, payload)

    with pytest.raises(ClosedLoopAuthorizationError):
        _verify_fault_campaign(
            path,
            digest,
            qualification_manifest_sha256=QUALIFICATION_SHA256,
            checkpoint_sha256=CHECKPOINT_SHA256,
        )


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('scenario_id', 'F99'),
        ('fault', 'different_fault'),
        ('status', 'failed'),
        ('visual_schema_version', 'room315.visual_state.v3'),
        ('checkpoint_sha256', '0' * 64),
        ('qualification_manifest_sha256', '1' * 64),
        ('false_success_count', 1),
        ('v3_observation_count', 1),
        ('v4_observation_count', 0),
        ('controller_final_mode', 'ENABLED'),
        ('physical_deployment', True),
    ],
)
def test_fault_campaign_rejects_any_per_scenario_guard_failure(
    tmp_path: Path,
    field: str,
    value: object,
) -> None:
    payload = _fault_campaign_payload()
    results = payload['results']
    assert isinstance(results, list)
    results[0][field] = value
    path, digest = _write_fault_evidence(tmp_path, payload)

    with pytest.raises(ClosedLoopAuthorizationError):
        _verify_fault_campaign(
            path,
            digest,
            qualification_manifest_sha256=QUALIFICATION_SHA256,
            checkpoint_sha256=CHECKPOINT_SHA256,
        )


@pytest.mark.parametrize('value', [False, 5.0, '5'])
def test_fault_campaign_counts_require_json_integers(
    tmp_path: Path,
    value: object,
) -> None:
    payload = _fault_campaign_payload()
    payload['passed_scenario_count'] = value
    path, digest = _write_fault_evidence(tmp_path, payload)

    with pytest.raises(
        ClosedLoopAuthorizationError,
        match='passed_scenario_count',
    ):
        _verify_fault_campaign(
            path,
            digest,
            qualification_manifest_sha256=QUALIFICATION_SHA256,
            checkpoint_sha256=CHECKPOINT_SHA256,
        )


def test_fault_campaign_report_requires_exact_external_sha256(
    tmp_path: Path,
) -> None:
    path, _ = _write_fault_evidence(tmp_path)

    with pytest.raises(ClosedLoopAuthorizationError, match='SHA-256 mismatch'):
        _verify_fault_campaign(
            path,
            '0' * 64,
            qualification_manifest_sha256=QUALIFICATION_SHA256,
            checkpoint_sha256=CHECKPOINT_SHA256,
        )


def test_fault_evidence_sidecars_must_cover_summary_exactly(
    tmp_path: Path,
) -> None:
    path, digest = _write_fault_evidence(
        tmp_path,
        include_summary_manifest_row=False,
    )
    _, resolved_path = _verify_fault_campaign(
        path,
        digest,
        qualification_manifest_sha256=QUALIFICATION_SHA256,
        checkpoint_sha256=CHECKPOINT_SHA256,
    )

    with pytest.raises(
        ClosedLoopAuthorizationError,
        match='manifest payload set mismatch',
    ):
        _verify_fault_evidence_sidecars(resolved_path, digest)


def test_runtime_decision_binds_positive_and_fault_campaign_hashes() -> None:
    fault_campaign = _fault_campaign_payload()
    decision = _decision_record(
        source_manifest_sha256=QUALIFICATION_SHA256,
        source_candidate_id='candidate-v4',
        checkpoint_sha256=CHECKPOINT_SHA256,
        reviewer='operator',
        scope=CLOSED_LOOP_RUNTIME_SCOPE,
        campaign=_campaign_payload(),
        campaign_sha256='a' * 64,
        fault_campaign=fault_campaign,
        fault_campaign_sha256='b' * 64,
        fault_evidence={
            'manifest_sha256': 'c' * 64,
            'sha256s_sha256': 'd' * 64,
        },
    )

    binding = decision['closed_loop_fault_campaign']
    assert binding == {
        'schema_version': FAULT_CAMPAIGN_SCHEMA,
        'sha256': 'b' * 64,
        'scenario_ids': ['F01', 'F02', 'F03', 'F04', 'F05'],
        'passed_scenario_count': 5,
        'failed_scenario_count': 0,
        'evidence_manifest_sha256': 'c' * 64,
        'evidence_sha256s_sha256': 'd' * 64,
    }
    assert decision['physical_deployment_approved'] is False


def test_strict_json_rejects_duplicate_and_nonfinite_values(
    tmp_path: Path,
) -> None:
    duplicate = tmp_path / 'duplicate.json'
    duplicate.write_text('{"status":"passed","status":"failed"}\n')
    with pytest.raises(ClosedLoopAuthorizationError):
        _load_json(duplicate, 'duplicate fixture')

    nonfinite = tmp_path / 'nonfinite.json'
    nonfinite.write_text('{"value":NaN}\n')
    with pytest.raises(ClosedLoopAuthorizationError):
        _load_json(nonfinite, 'non-finite fixture')


def test_qualification_decision_is_v4_only_and_not_physical() -> None:
    decision = _decision_record(
        source_manifest_sha256='2' * 64,
        source_candidate_id='candidate-v4',
        checkpoint_sha256=CHECKPOINT_SHA256,
        reviewer='operator',
        scope=CLOSED_LOOP_QUALIFICATION_SCOPE,
        campaign=None,
        campaign_sha256='',
    )

    assert decision['scope'] == CLOSED_LOOP_QUALIFICATION_SCOPE
    assert decision['qualification_only'] is True
    assert decision['physical_deployment_approved'] is False
    assert decision['automatic_promotion'] is False
    assert decision['runtime_guards'] == {
        'dry_run_state_fusion': True,
        'plansys2_update_enabled': False,
        'actuation_enabled': True,
    }
    assert decision['review_assertions'][
        'v4_is_authoritative_visual_observation'
    ] is True
