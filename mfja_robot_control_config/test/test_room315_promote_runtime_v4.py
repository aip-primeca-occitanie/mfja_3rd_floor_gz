import json
import stat
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))

import room_315_promote_runtime_v4 as promotion


SCENARIO_IDS = (
    'accept_l4_loaded',
    'accept_r4_loaded',
    'accept_exact_l2_l4_r4',
    'accept_right_slot3_plus_005',
    'accept_sparse',
    'accept_dense',
    'accept_multi_blocker',
)


def _write_json(path, value):
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


def _hash(path):
    return promotion.sha256_file(path)


def _seal_candidate(root):
    payloads = sorted(path for path in root.iterdir() if path.name != 'SHA256SUMS')
    (root / 'SHA256SUMS').write_text(
        ''.join(f'{_hash(path)}  {path.name}\n' for path in payloads),
        encoding='utf-8',
    )
    for path in root.iterdir():
        path.chmod(0o444)
    root.chmod(0o555)


def _restore_tree(root):
    if root.exists():
        root.chmod(0o755)
        for path in root.iterdir():
            path.chmod(0o644)


@pytest.fixture(autouse=True)
def fake_runtime_core_verifier(monkeypatch):
    def verify(path, expected_sha256):
        assert _hash(path) == expected_sha256
        manifest = json.loads(path.read_text(encoding='utf-8'))
        return SimpleNamespace(
            deployment_mode=manifest['deployment_mode'],
            checkpoint_sha256=manifest['artifacts']['checkpoint']['sha256'],
        )

    monkeypatch.setattr(promotion, '_verify_core_manifest', verify)


@pytest.fixture
def candidate(tmp_path):
    root = tmp_path / 'shadow_candidate'
    root.mkdir()
    artifacts = {}
    artifact_names = {
        'checkpoint': 'checkpoint_epoch_011.pt',
        'training_final_report': 'training_final_report.json',
        'validation_acceptance': 'validation_acceptance.json',
        'canary_final_report': 'canary_final_report.json',
        'canary_completion_ledger': 'canary_completion_ledger.json',
        'effective_config': 'effective_config.json',
        'validation_segment_calibration': 'validation_segment_calibration.json',
        'public_topology_contract': 'public_topology_contract.json',
    }
    for index, (name, filename) in enumerate(artifact_names.items()):
        path = root / filename
        path.write_bytes(f'fixture-{name}-{index}'.encode())
        artifacts[name] = {'path': filename, 'sha256': _hash(path)}
    checkpoint_sha = artifacts['checkpoint']['sha256']
    candidate_id = 'room315-v4-fixture-candidate'
    manifest = {
        'schema_version': promotion.PROMOTION_SCHEMA,
        'candidate_id': candidate_id,
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
        'artifacts': artifacts,
        'model_contract': {'fixture': True},
        'preprocessing_contract': {'fixture': True},
        'topology_contract': {'fixture': True},
        'calibration_contract': {'fixture': True},
        'acceptance_thresholds': {'fixture': True},
    }
    _write_json(root / promotion.PROMOTION_MANIFEST_NAME, manifest)
    _write_json(root / 'candidate_state.json', {
        'schema_version': promotion.ACTIVE_STATE_SCHEMA,
        'candidate_id': candidate_id,
        'state': 'shadow_authorized_pending_manual_review',
        'deployment_mode': 'shadow',
        'manual_review_approved': False,
        'manual_runtime_review_status': 'pending',
        'shadow_execution_authorized': True,
        'automatic_promotion_allowed': False,
        'active_runtime_selected': False,
        'active_transition_requires_new_immutable_manifest': True,
        'checkpoint_filename': artifact_names['checkpoint'],
        'checkpoint_sha256': checkpoint_sha,
        'runtime_topics': promotion.SHADOW_RUNTIME_TOPICS,
    })
    _write_json(root / 'acceptance_scenarios.json', {
        'schema_version': 'room315.runtime_acceptance_scenarios.v1',
        'candidate_id': candidate_id,
        'runtime_candidate': {
            'runtime_generation': 'v4',
            'runtime_mode': 'shadow',
            'automatic_promotion_allowed': False,
        },
        'scenarios': [{'scenario_id': value} for value in SCENARIO_IDS],
    })
    (root / 'runtime_ros_parameters.yaml').write_text('runtime_mode: shadow\n')
    (root / 'README.md').write_text('fixture\n')
    _seal_candidate(root)
    yield root
    _restore_tree(root)


def _shadow(checkpoint_sha):
    return {
        'schema_version': promotion.SHADOW_REPORT_SCHEMA,
        'candidate_id': 'room315-v4-fixture-candidate',
        'status': 'passed',
        'role': 'observation_only_same_frame_shadow',
        'automatic_runtime_switch': False,
        'expected_checkpoint_sha256': {
            'v3': promotion.SHADOW_REFERENCE_CHECKPOINT_SHA256,
            'v4': checkpoint_sha,
        },
        'minimum_paired_frames': 20,
        'paired_frame_count': 20,
        'v4_accepted_frame_count': 20,
        'v4_acceptance_coverage': 1.0,
        'wrong_v4_side_identities': [],
        'errors': [],
        'control_isolation': {
            'dry_run_state_fusion': True,
            'plansys2_update_enabled': False,
            'plansys2_mutation_count': 0,
            'actuation_command_count': 0,
            'comparator_owns_command_publisher': False,
        },
    }


def _acceptance(checkpoint_sha):
    quantitative = []
    records = []
    for scenario_id in SCENARIO_IDS:
        quantitative.append({
            'scenario_id': scenario_id,
            'record_status': 'complete',
            'ground_truth_matching_reobservation_count': 3,
            'all_recorded_segments_payloads_and_planning_positions_match': True,
        })
        records.append({
            'scenario_id': scenario_id,
            'record_status': 'complete',
            'failure_reasons': [],
            'execution_decision': {'allowed': False},
            'reobservation_and_effect_verification': {
                'actuation_performed': False,
            },
        })
    return {
        'schema_version': promotion.ACCEPTANCE_REPORT_SCHEMA,
        'candidate_id': 'room315-v4-fixture-candidate',
        'checkpoint_sha256': checkpoint_sha,
        'acceptance_status': 'complete_pending_human_decision',
        'automatic_deployment_approval': False,
        'scenario_count': 7,
        'complete_scenario_count': 7,
        'failed_scenario_count': 0,
        'approval': {'approved': False},
        'quantitative_acceptance': {
            'all_scenarios_quantitatively_complete': True,
            'ground_truth_used_as_model_input': False,
            'required_exact_fields': ['side', 'segment', 'loaded_state'],
            'required_planning_s_ratio_tolerance': 0.12,
            'scenarios': quantitative,
        },
        'records': records,
    }


def _faults(checkpoint_sha):
    return {
        'schema_version': promotion.FAULT_REPORT_SCHEMA,
        'candidate_id': 'room315-v4-fixture-candidate',
        'checkpoint_sha256': checkpoint_sha,
        'status': 'passed',
        'all_faults_rejected': True,
        'plansys2_mutation_count': 0,
        'actuation_command_count': 0,
        'cases': [
            {'case_id': case_id, 'passed': True, 'rejected': True}
            for case_id in promotion.REQUIRED_FAULT_CASES
        ],
    }


@pytest.fixture
def environment(candidate, tmp_path):
    manifest_sha = _hash(candidate / promotion.PROMOTION_MANIFEST_NAME)
    state = json.loads((candidate / 'candidate_state.json').read_text())
    reports = {
        'shadow': tmp_path / 'shadow.json',
        'acceptance': tmp_path / 'acceptance.json',
        'fault': tmp_path / 'fault.json',
    }
    payloads = {
        'shadow': _shadow(state['checkpoint_sha256']),
        'acceptance': _acceptance(state['checkpoint_sha256']),
        'fault': _faults(state['checkpoint_sha256']),
    }
    for name, path in reports.items():
        _write_json(path, payloads[name])
    inputs = promotion.PromotionInputs(
        candidate_directory=candidate,
        expected_candidate_manifest_sha256=manifest_sha,
        shadow_report=reports['shadow'],
        expected_shadow_report_sha256=_hash(reports['shadow']),
        acceptance_report=reports['acceptance'],
        expected_acceptance_report_sha256=_hash(reports['acceptance']),
        fault_injection_report=reports['fault'],
        expected_fault_injection_report_sha256=_hash(reports['fault']),
        reviewer='integration-reviewer',
        decision=promotion.APPROVED_DECISION,
        scope=promotion.APPROVED_SCOPE,
        output=tmp_path / 'active_bundle',
    )
    return inputs, reports


def _rewrite_report(inputs, reports, name, mutate):
    path = reports[name]
    value = json.loads(path.read_text())
    mutate(value)
    _write_json(path, value)
    field = {
        'shadow': 'expected_shadow_report_sha256',
        'acceptance': 'expected_acceptance_report_sha256',
        'fault': 'expected_fault_injection_report_sha256',
    }[name]
    return replace(inputs, **{field: _hash(path)})


def test_pass_creates_atomic_read_only_active_bundle(environment):
    inputs, _reports = environment
    result = promotion.promote(inputs)
    output = inputs.output

    assert result['status'] == 'ACTIVE_BUNDLE_CREATED'
    assert result['deployment_mode'] == 'active'
    assert result['automatic_runtime_switch'] is False
    assert result['checked_in_defaults_modified'] is False
    assert not stat.S_IMODE(output.stat().st_mode) & 0o222
    assert all(
        path.is_file() and not stat.S_IMODE(path.stat().st_mode) & 0o222
        for path in output.iterdir()
    )

    manifest = json.loads((output / promotion.PROMOTION_MANIFEST_NAME).read_text())
    assert manifest['deployment_mode'] == 'active'
    assert manifest['manual_review_approved'] is True
    assert manifest['manual_runtime_review_status'] == 'approved'
    assert manifest['automatic_promotion_allowed'] is False
    assert 'rollback_contract' not in manifest
    assert manifest['manual_decision_record']['sha256'] == _hash(
        output / 'manual_decision_record.json'
    )
    decision = json.loads((output / 'manual_decision_record.json').read_text())
    assert decision['decision'] == 'approved'
    assert decision['scope'] == promotion.APPROVED_SCOPE
    assert decision['physical_deployment_approved'] is False
    assert all('rollback' not in key for key in decision['evidence'])
    assert all('rollback' not in key for key in decision['review_assertions'])

    state = json.loads((output / 'candidate_state.json').read_text())
    assert state['state'] == 'active_selected_manual_review_approved'
    assert state['active_runtime_selected'] is True
    assert state['runtime_topics'] == promotion.STANDARD_RUNTIME_TOPICS
    assert state['required_task_visual_allowlist'] == {
        'model_schema_version': promotion.V4_SCHEMA_VERSION,
        'checkpoint_sha256': result['checkpoint_sha256'],
    }
    assert all('rollback' not in key for key in state)
    assert not (output / 'rollback_option.json').exists()
    assert all('rollback' not in path.name for path in output.iterdir())
    runtime_yaml = (output / 'runtime_ros_parameters.yaml').read_text()
    assert 'runtime_mode: active' in runtime_yaml
    assert 'dry_run_state_fusion: true' in runtime_yaml
    assert 'plansys2_update_enabled: false' in runtime_yaml
    assert '/room_315/visual_state/shadow_v4/' not in runtime_yaml
    assert result['promotion_manifest_sha256'] in runtime_yaml

    sums = (output / 'SHA256SUMS').read_text().splitlines()
    assert {line.split('  ', 1)[1] for line in sums} == {
        path.name for path in output.iterdir() if path.name != 'SHA256SUMS'
    }
    assert all(_hash(output / line.split('  ', 1)[1]) == line.split('  ', 1)[0] for line in sums)
    _restore_tree(output)


@pytest.mark.parametrize(
    'field',
    (
        'candidate_directory',
        'shadow_report',
        'acceptance_report',
        'fault_injection_report',
    ),
)
def test_every_required_input_missing_fails_before_output(environment, field):
    inputs, _reports = environment
    changed = replace(inputs, **{field: inputs.output.parent / f'missing-{field}'})
    with pytest.raises(promotion.PromotionV4Error):
        promotion.promote(changed)
    assert not inputs.output.exists()


@pytest.mark.parametrize(
    'field',
    (
        'expected_candidate_manifest_sha256',
        'expected_shadow_report_sha256',
        'expected_acceptance_report_sha256',
        'expected_fault_injection_report_sha256',
    ),
)
def test_every_external_hash_mismatch_fails_before_output(environment, field):
    inputs, _reports = environment
    with pytest.raises(promotion.PromotionV4Error, match='SHA-256 mismatch'):
        promotion.promote(replace(inputs, **{field: '0' * 64}))
    assert not inputs.output.exists()


SEMANTIC_FAILURES = (
    ('shadow', lambda value: value.update(status='failed'), 'did not pass'),
    ('shadow', lambda value: value['expected_checkpoint_sha256'].update(v4='0' * 64), 'V4 checkpoint mismatch'),
    ('shadow', lambda value: value.update(wrong_v4_side_identities=['L1']), 'wrong-side'),
    ('shadow', lambda value: value['control_isolation'].update(plansys2_mutation_count=1), 'mutated PlanSys2'),
    ('shadow', lambda value: value['control_isolation'].update(comparator_owns_command_publisher=True), 'command publisher'),
    ('acceptance', lambda value: value.update(checkpoint_sha256='0' * 64), 'checkpoint SHA-256 mismatch'),
    ('acceptance', lambda value: value.update(complete_scenario_count=6), 'complete seven'),
    ('acceptance', lambda value: value['quantitative_acceptance'].update(all_scenarios_quantitatively_complete=False), 'quantitative acceptance is incomplete'),
    ('acceptance', lambda value: value.update(failed_scenario_count=1), 'failed scenarios'),
    ('acceptance', lambda value: value['records'][0].update(failure_reasons=['failure']), 'contains failures'),
    ('fault', lambda value: value.update(checkpoint_sha256='0' * 64), 'checkpoint SHA-256 mismatch'),
    ('fault', lambda value: value.update(all_faults_rejected=False), 'not all injected'),
    ('fault', lambda value: value['cases'].pop(), 'case set is incomplete'),
    ('fault', lambda value: value.update(plansys2_mutation_count=1), 'mutated PlanSys2'),
)


@pytest.mark.parametrize('report_name,mutate,error', SEMANTIC_FAILURES)
def test_all_runtime_evidence_mismatches_fail_closed(
    environment,
    report_name,
    mutate,
    error,
):
    inputs, reports = environment
    changed = _rewrite_report(inputs, reports, report_name, mutate)
    with pytest.raises(promotion.PromotionV4Error, match=error):
        promotion.promote(changed)
    assert not inputs.output.exists()


@pytest.mark.parametrize(
    'changes,error',
    (
        ({'reviewer': '   '}, 'reviewer'),
        ({'decision': 'rejected'}, 'decision'),
        ({'scope': 'physical_deployment'}, 'scope'),
    ),
)
def test_manual_decision_must_be_explicit_and_narrow(environment, changes, error):
    inputs, _reports = environment
    with pytest.raises(promotion.PromotionV4Error, match=error):
        promotion.promote(replace(inputs, **changes))
    assert not inputs.output.exists()


def test_candidate_payload_tampering_is_rejected(environment):
    inputs, _reports = environment
    candidate = inputs.candidate_directory
    manifest = candidate / promotion.PROMOTION_MANIFEST_NAME
    candidate.chmod(0o755)
    manifest.chmod(0o644)
    with manifest.open('a', encoding='utf-8') as stream:
        stream.write(' ')
    manifest.chmod(0o444)
    candidate.chmod(0o555)
    with pytest.raises(promotion.PromotionV4Error, match='payload SHA-256 mismatch'):
        promotion.promote(inputs)
    assert not inputs.output.exists()


def test_existing_output_is_never_overwritten(environment):
    inputs, _reports = environment
    inputs.output.mkdir()
    marker = inputs.output / 'keep'
    marker.write_text('unchanged')
    with pytest.raises(promotion.PromotionV4Error, match='overwrite'):
        promotion.promote(inputs)
    assert marker.read_text() == 'unchanged'


def test_active_core_rejection_leaves_no_bundle_or_staging(
    environment,
    monkeypatch,
):
    inputs, _reports = environment

    def reject_active(path, expected_sha256):
        manifest = json.loads(path.read_text())
        if manifest['deployment_mode'] == 'active':
            raise promotion.PromotionV4Error('active core rejection')
        return SimpleNamespace(
            deployment_mode='shadow',
            checkpoint_sha256=manifest['artifacts']['checkpoint']['sha256'],
        )

    monkeypatch.setattr(promotion, '_verify_core_manifest', reject_active)
    with pytest.raises(promotion.PromotionV4Error, match='active core rejection'):
        promotion.promote(inputs)
    assert not inputs.output.exists()
    assert not list(inputs.output.parent.glob(f'.{inputs.output.name}.staging-*'))
