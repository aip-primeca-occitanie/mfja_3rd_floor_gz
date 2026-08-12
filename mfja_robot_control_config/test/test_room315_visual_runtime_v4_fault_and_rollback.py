#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import json
import stat
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import room_315_visual_runtime_v4_fault_and_rollback as verifier  # noqa: E402
import room_315_promote_runtime_v4 as promotion  # noqa: E402


def test_exact_report_schemas_and_fault_order_are_stable():
    assert verifier.FAULT_REPORT_SCHEMA == (
        'room315.visual_runtime_v4.fault_injection.v1'
    )


def test_default_config_path_supports_source_and_installed_layouts(tmp_path):
    source_scripts = tmp_path / 'source_package' / 'scripts'
    source_config = (
        tmp_path / 'source_package' / 'config' / 'room_315_vla' / 'runtime.yaml'
    )
    source_scripts.mkdir(parents=True)
    source_config.parent.mkdir(parents=True)
    source_config.write_text('source\n', encoding='utf-8')
    assert verifier.default_package_config_path(
        'room_315_vla/runtime.yaml', script_directory=source_scripts
    ) == source_config

    installed_scripts = tmp_path / 'install' / 'pkg' / 'lib' / 'pkg'
    installed_config = (
        tmp_path / 'install' / 'pkg' / 'share' / 'mfja_robot_control_config'
        / 'config' / 'room_315_vla' / 'runtime.yaml'
    )
    installed_scripts.mkdir(parents=True)
    installed_config.parent.mkdir(parents=True)
    installed_config.write_text('installed\n', encoding='utf-8')
    assert verifier.default_package_config_path(
        'room_315_vla/runtime.yaml', script_directory=installed_scripts
    ) == installed_config
    assert verifier.ROLLBACK_REPORT_SCHEMA == (
        'room315.visual_runtime_v4.rollback_smoke.v1'
    )
    assert verifier.EXPECTED_FAULT_NAMES == (
        'manifest_hash_mismatch',
        'checkpoint_hash_mismatch',
        'topology_fingerprint_mismatch',
        'camera_order_mismatch',
        'stale_left_image',
        'stale_right_image',
        'stale_presence',
        'unknown_identity',
    )
    assert verifier.DEFAULT_V3_RUNTIME_CONFIG.name == (
        'visual_state_runtime_v3_rollback.yaml'
    )
    assert verifier.DEFAULT_TASK_RUNTIME_CONFIG.name == (
        'task_execution_runtime_v3_rollback.yaml'
    )


def test_immutable_report_is_integrity_bound_read_only_and_no_clobber(tmp_path):
    output = tmp_path / 'report.json'
    report = {
        'schema_version': verifier.FAULT_REPORT_SCHEMA,
        'immutable': True,
        'status': 'passed',
        'overall_passed': True,
        'checks': [],
    }
    file_digest = verifier.write_immutable_report(output, report)
    loaded = json.loads(output.read_text(encoding='utf-8'))
    integrity = loaded.pop('integrity')

    assert integrity['algorithm'] == 'sha256'
    assert integrity['scope'] == 'canonical_json_excluding_integrity'
    assert integrity['sha256'] == verifier.canonical_sha256(loaded)
    assert file_digest == hashlib.sha256(output.read_bytes()).hexdigest()
    assert stat.S_IMODE(output.stat().st_mode) == 0o444
    with pytest.raises(verifier.FaultRollbackVerificationError, match='replace'):
        verifier.write_immutable_report(output, report)


def test_expected_exception_check_accepts_only_the_expected_fail_closed_error():
    def expected_failure():
        raise verifier.VisualRuntimeV4Error('camera order contract mismatch')

    passed = verifier._expected_exception_check(
        name='camera_order_mismatch',
        detection_layer='unit',
        injection='swap',
        expected_message='camera order',
        action=expected_failure,
    )
    unexpected = verifier._expected_exception_check(
        name='camera_order_mismatch',
        detection_layer='unit',
        injection='swap',
        expected_message='topology',
        action=expected_failure,
    )
    accepted = verifier._expected_exception_check(
        name='camera_order_mismatch',
        detection_layer='unit',
        injection='swap',
        expected_message='camera order',
        action=lambda: None,
    )

    assert passed['passed'] and passed['fail_closed']
    assert passed['safety_effects']['plansys2_update_attempt_count'] == 0
    assert passed['safety_effects']['actuation_command_count'] == 0
    assert not unexpected['passed']
    assert not accepted['passed']


def test_fault_report_exactly_satisfies_promotion_contract():
    checks = [
        verifier._check_result(
            name=case_id,
            passed=True,
            detection_layer='unit',
            injection='unit',
            expected_rejection='reject',
            observed_rejection='reject',
        )
        for case_id in verifier.EXPECTED_FAULT_NAMES
    ]
    report = {
        'schema_version': verifier.FAULT_REPORT_SCHEMA,
        'status': 'passed',
        'overall_passed': True,
        'candidate': {
            'candidate_id': 'candidate-under-test',
            'checkpoint_sha256': 'a' * 64,
        },
        'checks': checks,
    }
    verifier._apply_fault_promotion_contract(report)

    promotion._verify_fault_report(
        report,
        SimpleNamespace(
            candidate_id='candidate-under-test',
            checkpoint_sha256='a' * 64,
        ),
    )
    assert report['all_faults_rejected']
    assert report['plansys2_mutation_count'] == 0
    assert report['actuation_command_count'] == 0
    assert [row['case_id'] for row in report['checks']] == list(
        verifier.EXPECTED_FAULT_NAMES
    )
    assert all(row['rejected'] for row in report['cases'])


def test_rollback_report_exactly_satisfies_promotion_contract():
    check_names = (
        'v3_selected_and_v4_not_selected',
        'v3_checkpoint_strict_model_load',
        'v3_raw_observation_possible',
        'v3_accepted_observation_possible',
        'task_visual_allowlist_v3_exact',
    )
    report = {
        'schema_version': verifier.ROLLBACK_REPORT_SCHEMA,
        'status': 'passed',
        'checks': [
            {'name': name, 'passed': True}
            for name in check_names
        ],
        'runtime': {
            'schema_version': verifier.V3_MODEL_SCHEMA,
            'checkpoint_sha256': promotion.V3_CHECKPOINT_SHA256,
            'model_ready': True,
        },
        'safety': {
            'plansys2_update_enabled': False,
            'plansys2_predicates_added': 0,
            'plansys2_predicates_removed': 0,
            'actuation_command_count': 0,
        },
    }
    verifier._apply_rollback_promotion_contract(report)

    promotion._verify_rollback_report(report)
    assert report['checkpoint_sha256'] == promotion.V3_CHECKPOINT_SHA256
    assert report['raw_prediction_received']
    assert report['accepted_observation_received']
    assert report['plansys2_mutation_count'] == 0
    assert not report['v4_selected']


def test_presence_faults_are_real_provider_failures():
    stale = verifier.ShuttleStatePresenceProvider(timeout_s=1.0, warmup_s=0.0)
    for side in ('left', 'right'):
        stale.observe(
            topic_side=side,
            entity_name='',
            source_stamp_s=1.0,
            receive_time_s=1.0,
        )
    stale_snapshot = stale.snapshot(now_s=3.0)

    unknown = verifier.ShuttleStatePresenceProvider(timeout_s=1.0, warmup_s=0.0)
    for side in ('left', 'right'):
        unknown.observe(
            topic_side=side,
            entity_name='',
            source_stamp_s=1.0,
            receive_time_s=1.0,
        )
    unknown.observe(
        topic_side='left',
        entity_name='not_a_room315_entity',
        source_stamp_s=1.0,
        receive_time_s=1.0,
    )
    unknown_snapshot = unknown.snapshot(now_s=1.0)

    assert not stale_snapshot.ready
    assert set(stale_snapshot.stale_sides) == {'left', 'right'}
    assert any('presence_source_stale' in value for value in stale_snapshot.reasons)
    assert not unknown_snapshot.ready
    assert any('unknown_presence_entity' in value for value in unknown_snapshot.reasons)


def test_synthetic_inputs_are_deterministic_and_contract_shaped():
    first_left, first_right = verifier._synthetic_paired_rgb()
    second_left, second_right = verifier._synthetic_paired_rgb()
    output = verifier._accepted_synthetic_v4_output()
    presence = verifier._presence_snapshot(('L1', 'R1'), timestamp_s=10.0)

    assert first_left.shape == first_right.shape == (480, 640, 3)
    assert first_left.dtype == first_right.dtype
    assert (first_left == second_left).all()
    assert (first_right == second_right).all()
    assert output.segment_logits.shape == (8, 14)
    assert output.loaded_logits.shape == (8, 2)
    assert output.bbox.shape == (8, 4)
    assert output.s_ratio.shape == (8,)
    assert presence.ready
    assert tuple(
        entry.identity for entry in presence.entries
        if entry.state == verifier.PRESENCE_PRESENT
    ) == ('L1', 'R1')


def test_visual_node_ast_contract_has_no_shadow_plan_client_or_actuation():
    result = verifier._inspect_visual_node_shadow_contract()

    assert result['passed']
    assert result['checks']['plan_client_initialized_absent']
    assert result['checks']['plan_client_created_only_outside_shadow']
    assert result['checks']['shadow_plansys_update_returns_before_client']
    assert result['checks']['no_actuation_publisher_message_type']
    assert not result['actuation_publisher_message_types']


def test_cli_requires_pinned_manifest_sha_and_report_path():
    parser = verifier._parser()
    arguments = parser.parse_args([
        'faults',
        '--expected-manifest-sha256',
        'a' * 64,
        '--output',
        '/tmp/not-written-by-parser.json',
    ])

    assert arguments.command == 'faults'
    assert arguments.expected_manifest_sha256 == 'a' * 64
    assert arguments.output == Path('/tmp/not-written-by-parser.json')
    assert arguments.candidate_dir == verifier.DEFAULT_CANDIDATE_DIRECTORY
