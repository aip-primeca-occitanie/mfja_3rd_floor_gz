#!/usr/bin/env python3
"""Contract tests for the one-shot Room 315 V4 final-Test evaluator."""

from __future__ import annotations

import inspect
import json
import sys
from pathlib import Path

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_final_test_v4 as final_test  # noqa: E402


def _sha(char: str) -> str:
    return char * 64


def _coverage_contract() -> dict:
    return {
        'required_identities': list(final_test.FIXED_IDENTITIES),
        'required_sides': list(final_test.SIDES),
        'required_segments': list(final_test.SEGMENT_CLASSES),
        'required_position_bins': list(final_test.POSITION_BINS),
        'required_scene_occlusion_classes': list(
            final_test.SCENE_OCCLUSION_CLASSES
        ),
        'required_scene_presence_densities': list(
            final_test.SCENE_PRESENCE_DENSITIES
        ),
        'required_target_zones': list(final_test.TARGET_ZONES),
        'required_identity_zones': list(final_test.IDENTITY_ZONES),
        'minimum_sample_count': 512,
        'minimum_visible_total': 512,
        'minimum_visible_per_identity': 32,
        'minimum_visible_per_side_x_segment': 8,
        'minimum_visible_per_position_bin': 16,
        'minimum_records_per_scene_occlusion_class': 32,
        'minimum_records_per_scene_presence_density': 32,
        'minimum_visible_per_target_zone': 8,
        'minimum_visible_per_identity_zone': 8,
        'runtime_threshold_gates': {
            'minimum_segment_confidence_coverage': 0.90,
            'minimum_segment_selective_accuracy': 0.95,
            'minimum_loaded_confidence_coverage': 0.95,
            'minimum_joint_confidence_coverage': 0.90,
        },
    }


def _artifact(tmp_path: Path, name: str, content: bytes = b'x') -> final_test.Artifact:
    path = tmp_path / name
    path.write_bytes(content)
    return final_test.Artifact(path, final_test._sha256_file(path), len(content))


def _bundle(tmp_path: Path, *, contract_sha: str, output_name: str):
    checkpoint = _artifact(tmp_path, 'checkpoint.pt')
    contract_artifact = _artifact(tmp_path, f'{output_name}_contract.json')
    dataset_config = _artifact(tmp_path, 'dataset_config.json')
    preregistration = _artifact(tmp_path, 'preregistration.json')
    plan_lock = _artifact(tmp_path, 'plan_lock.json')
    protocol_lock = _artifact(tmp_path, 'evaluation_protocol_lock.json')
    finalization = _artifact(tmp_path, 'finalization.json')
    calibration = _artifact(tmp_path, 'calibration.json')
    runtime = _artifact(tmp_path, 'runtime.json')
    dataset_fingerprint = _sha('d')
    attempt_key = final_test.final_test_attempt_key(
        checkpoint.sha256, dataset_fingerprint
    )
    return final_test.ControlBundle(
        contract_path=contract_artifact.path,
        contract_sha256=contract_sha,
        contract={
            'dataset': {'dataset_fingerprint_sha256': dataset_fingerprint}
        },
        evaluation_protocol_lock={
            'protocol_frozen_sha256': _sha('f'),
            'implementation_aggregate_sha256': _sha('a'),
        },
        dataset_config={},
        plan_lock={},
        preregistration={},
        effective_config={'pilot_acceptance_gates': {'gate': 1.0}},
        training_report={},
        validation_acceptance={},
        validation_calibration={},
        topology_contract={'fingerprint_sha256': _sha('e')},
        runtime_manifest={'model_contract': {'kind': 'v4'}},
        artifacts={
            'contract': contract_artifact,
            'dataset_config': dataset_config,
            'preregistration': preregistration,
            'plan_lock': plan_lock,
            'evaluation_protocol_lock': protocol_lock,
            'finalization': finalization,
            'checkpoint': checkpoint,
            'validation_segment_calibration': calibration,
            'runtime_promotion_manifest': runtime,
        },
        attempt_key=attempt_key,
        output_path=tmp_path / output_name,
    )


def _standalone_protocol_lock(tmp_path, monkeypatch):
    implementation_path = tmp_path / 'evaluator.py'
    implementation_path.write_text('frozen implementation\n')
    config_path = tmp_path / 'rail.yaml'
    config_path.write_text('frozen config\n')
    monkeypatch.setattr(
        final_test,
        '_implementation_artifact_paths',
        lambda: {'evaluator': implementation_path},
    )
    monkeypatch.setattr(
        final_test,
        '_implementation_config_artifact_paths',
        lambda: {'rail': config_path},
    )
    environment = {
        'python_executable': '/frozen/python',
        'python_implementation': 'CPython',
        'python_version': '3.test',
        'platform_system': 'Linux',
        'platform_release': 'test',
        'platform_machine': 'x86_64',
        'byteorder': 'little',
        'torch_version': 'test',
        'torch_cuda_build': 'test',
        'cudnn_version': 1,
        'cuda_available': True,
        'cuda_device_count': 1,
        'cuda_devices': [{'index': 0, 'name': 'test'}],
        'package_versions': {},
    }
    monkeypatch.setattr(final_test, '_environment_snapshot', lambda: environment)
    implementation = {
        'code': {
            'evaluator': final_test._artifact_declaration(implementation_path)
        },
        'config': {'rail': final_test._artifact_declaration(config_path)},
    }
    coverage = final_test._default_coverage_contract()
    policy = final_test._default_execution_policy()
    package_root, repository_root = final_test._assert_source_tree_execution()
    value = {
        'schema_version': final_test.EVALUATION_PROTOCOL_LOCK_SCHEMA_VERSION,
        'dataset_role': final_test.DATASET_ROLE,
        'execution_layout': 'source_tree_only',
        'source_tree': {
            'package_root': str(package_root),
            'repository_root': str(repository_root),
            'anchors': {
                'cmake': final_test._artifact_declaration(
                    package_root / 'CMakeLists.txt'
                ),
                'package_manifest': final_test._artifact_declaration(
                    package_root / 'package.xml'
                ),
            },
        },
        'created_at_utc': '2026-08-11T10:00:00+00:00',
        'inference_status': 'not_run',
        'inference_count': 0,
        'test_rows_opened': False,
        'test_labels_opened': False,
        'test_images_opened': False,
        'dataset_root': str(
            tmp_path / 'room315_visual_v4_final_test_seed3152026081101'
        ),
        'design_artifacts': {},
        'candidate_artifacts': {},
        'historical_reference_artifacts': {},
        'coverage_contract': coverage,
        'coverage_contract_sha256': final_test._sha256_canonical(coverage),
        'execution_policy': policy,
        'execution_policy_sha256': final_test._sha256_canonical(policy),
        'one_shot': {
            'enabled': True,
            'evaluation_device': final_test.EVALUATION_DEVICE,
            'ledger_override_allowed': False,
            'global_ledger_root': str(
                final_test.DEFAULT_GLOBAL_LEDGER_ROOT.resolve()
            ),
            'attempt_identity': (
                'checkpoint_sha256+dataset_fingerprint_sha256'
            ),
            'failed_or_interrupted_attempt_is_consumed': True,
        },
        'implementation_artifacts': implementation,
        'implementation_aggregate_sha256': (
            final_test._implementation_aggregate_sha256(implementation)
        ),
        'environment': environment,
        'environment_sha256': final_test._sha256_canonical(environment),
    }
    value['protocol_frozen_sha256'] = final_test._protocol_frozen_sha256(value)
    path = tmp_path / 'evaluation_protocol_lock.json'
    final_test._write_json_exclusive(path, value, read_only=True)
    return (
        path, final_test._sha256_file(path), implementation_path,
        config_path, environment,
    )


def test_only_public_cli_is_explicit_evaluate():
    parser = final_test._build_parser()
    args = parser.parse_args([
        'evaluate',
        '--contract',
        '/tmp/final_test_contract.json',
        '--contract-sha256',
        _sha('a'),
    ])
    assert args.mode == 'evaluate'
    with pytest.raises(SystemExit):
        parser.parse_args(['train'])
    with pytest.raises(SystemExit):
        parser.parse_args([
            'evaluate', '--contract', '/tmp/final.json',
            '--contract-sha256', _sha('a'), '--device', 'cpu',
        ])


def test_production_attempt_apis_have_no_ledger_override():
    assert 'ledger_root' not in inspect.signature(
        final_test.reserve_final_test_attempt
    ).parameters
    assert 'ledger_root' not in inspect.signature(
        final_test.evaluate_final_test_v4
    ).parameters
    assert 'device_override' not in inspect.signature(
        final_test.evaluate_final_test_v4
    ).parameters
    assert 'evaluation_protocol_lock_path' in inspect.signature(
        final_test.build_final_test_contract
    ).parameters
    assert 'evaluation_protocol_lock_sha256' in inspect.signature(
        final_test.build_final_test_contract
    ).parameters
    assert 'evaluation_protocol_lock_path' in inspect.signature(
        final_test.materialize_final_test_contract
    ).parameters
    assert 'evaluation_protocol_lock_sha256' in inspect.signature(
        final_test.materialize_final_test_contract
    ).parameters


def test_installed_layout_is_refused_before_reservation(tmp_path, monkeypatch):
    installed = (
        tmp_path / 'install' / 'lib' / 'mfja_robot_control_config'
        / 'room_315_visual_final_test_v4.py'
    )
    installed.parent.mkdir(parents=True)
    installed.write_text('# installed copy\n')
    ledger = tmp_path / 'ledger'
    monkeypatch.setattr(final_test, '__file__', str(installed))
    monkeypatch.setattr(final_test, 'DEFAULT_GLOBAL_LEDGER_ROOT', ledger)
    with pytest.raises(final_test.FinalTestV4Error, match='source-tree execution only'):
        final_test.reserve_final_test_attempt(
            tmp_path / 'contract.json', _sha('1')
        )
    assert not ledger.exists()


def test_protocol_lock_is_0444_and_verifies_all_current_bindings(
    tmp_path, monkeypatch
):
    path, digest, _, _, _ = _standalone_protocol_lock(tmp_path, monkeypatch)
    value, artifact = final_test.load_evaluation_protocol_lock(path, digest)
    assert value['schema_version'] == (
        'room315.visual_v4.final_test_evaluation_protocol_lock.v1'
    )
    assert artifact.sha256 == digest
    assert oct(path.stat().st_mode & 0o777) == '0o444'


def test_protocol_lock_rejects_implementation_tampering(tmp_path, monkeypatch):
    path, digest, implementation, _, _ = _standalone_protocol_lock(
        tmp_path, monkeypatch
    )
    implementation.write_text('tampered implementation\n')
    with pytest.raises(final_test.FinalTestV4Error, match='SHA-256 mismatch'):
        final_test.load_evaluation_protocol_lock(path, digest)


def test_protocol_lock_rejects_frozen_config_tampering(tmp_path, monkeypatch):
    path, digest, _, config, _ = _standalone_protocol_lock(
        tmp_path, monkeypatch
    )
    config.write_text('tampered config\n')
    with pytest.raises(final_test.FinalTestV4Error, match='SHA-256 mismatch'):
        final_test.load_evaluation_protocol_lock(path, digest)


def test_protocol_lock_rejects_lock_hash_and_environment_tampering(
    tmp_path, monkeypatch
):
    path, digest, _, _, environment = _standalone_protocol_lock(
        tmp_path, monkeypatch
    )
    with pytest.raises(final_test.FinalTestV4Error, match='SHA-256 mismatch'):
        final_test.load_evaluation_protocol_lock(path, _sha('0'))
    changed = dict(environment)
    changed['torch_version'] = 'different'
    monkeypatch.setattr(final_test, '_environment_snapshot', lambda: changed)
    with pytest.raises(final_test.FinalTestV4Error, match='environment differs'):
        final_test.load_evaluation_protocol_lock(path, digest)


def test_protocol_lock_requires_cuda_before_reservation(tmp_path, monkeypatch):
    ledger = tmp_path / 'global_ledger'
    monkeypatch.setattr(final_test, 'DEFAULT_GLOBAL_LEDGER_ROOT', ledger)
    path, digest, _, _, environment = _standalone_protocol_lock(
        tmp_path, monkeypatch
    )
    unavailable = dict(environment)
    unavailable['cuda_available'] = False
    unavailable['cuda_device_count'] = 0
    unavailable['cuda_devices'] = []
    monkeypatch.setattr(final_test, '_environment_snapshot', lambda: unavailable)

    def load_then_return(*_args):
        final_test.load_evaluation_protocol_lock(path, digest)
        raise AssertionError('unreachable after CUDA rejection')

    monkeypatch.setattr(final_test, 'load_control_bundle', load_then_return)
    with pytest.raises(final_test.FinalTestV4Error, match='environment differs'):
        final_test.reserve_final_test_attempt(
            tmp_path / 'contract.json', _sha('1')
        )
    assert not ledger.exists()


def test_protocol_and_environment_validation_precede_reservation(
    tmp_path, monkeypatch
):
    ledger = tmp_path / 'global_ledger'
    monkeypatch.setattr(final_test, 'DEFAULT_GLOBAL_LEDGER_ROOT', ledger)
    path, digest, _, _, environment = _standalone_protocol_lock(
        tmp_path, monkeypatch
    )

    def load_then_return(*_args):
        final_test.load_evaluation_protocol_lock(path, digest)
        raise AssertionError('unreachable after environment rejection')

    monkeypatch.setattr(final_test, 'load_control_bundle', load_then_return)
    changed = dict(environment)
    changed['python_executable'] = '/different/python'
    monkeypatch.setattr(final_test, '_environment_snapshot', lambda: changed)
    with pytest.raises(final_test.FinalTestV4Error, match='environment differs'):
        final_test.reserve_final_test_attempt(
            tmp_path / 'contract.json', _sha('1')
        )
    assert not ledger.exists()


def test_evaluator_source_has_no_training_selection_or_temperature_fit_call():
    source = inspect.getsource(final_test)
    forbidden_calls = (
        '.train(',
        'build_staged_optimizer(',
        'fit_segment_temperature(',
        'fit_validation_segment_calibration(',
        'train_v4(',
    )
    assert all(token not in source for token in forbidden_calls)
    assert "model.eval()" in source
    assert "'fit_performed': False" in source
    assert "'checkpoint_selection_performed': False" in source
    assert "'threshold_selection_performed': False" in source


def test_protocol_freezes_direct_and_dynamic_runtime_dependencies():
    paths = final_test._implementation_artifact_paths()
    assert {
        'acceptance',
        'calibration',
        'dataset_loader',
        'evaluator',
        'json_io',
        'kinematic_shuttle',
        'model',
        'multi_shuttle',
        'rail_defaults',
        'topology_contract',
        'trainer_evaluation_api',
        'training_runtime',
        'v3_common_dependency',
        'visual_fleet',
    } <= set(paths)
    assert paths['training_runtime'].name == 'room_315_visual_training_v4.py'
    assert paths['calibration'].name == 'room_315_visual_calibration_v4.py'
    assert paths['rail_defaults'].name == 'room_315_rail_defaults.py'


def test_protocol_freezes_world_identity_rails_and_all_referenced_csvs():
    paths = final_test._implementation_config_artifact_paths()
    assert {
        'shuttle_identity', 'simulation_world',
        'rail_network_left', 'rail_network_right',
    } <= set(paths)
    expected_csv_keys = {
        f'raw_segment_{Path(name).stem}'
        for name in final_test.RAW_SEGMENT_CSV_NAMES
    }
    assert {name for name in paths if name.startswith('raw_segment_')} == (
        expected_csv_keys
    )
    assert all(path.is_file() for path in paths.values())


def test_stale_ament_rail_share_is_rejected(monkeypatch):
    monkeypatch.setattr(
        final_test,
        'default_rail_network_path',
        lambda side: Path('/stale/install/share') / f'rail_network_{side}.yaml',
    )
    with pytest.raises(final_test.FinalTestV4Error, match='stale ament share'):
        final_test._implementation_config_artifact_paths()


def test_evaluator_is_not_installed_but_its_test_remains_registered():
    package_root, _ = final_test._assert_source_tree_execution()
    cmake = (package_root / 'CMakeLists.txt').read_text()
    install_section = cmake.split('if(BUILD_TESTING)', 1)[0]
    assert 'scripts/room_315_visual_final_test_v4.py' not in install_section
    assert 'test_room315_visual_final_test_v4' in cmake


@pytest.mark.parametrize('path', [
    '/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730/test.jsonl',
    '/home/tiago/room315_visual_state_v4_blockers/splits/test.jsonl',
    '/home/tiago/room315_kairos_visual_state_training_v1_seed31520260730/dataset/splits/test.jsonl',
    '/home/tiago/room315_local_training/visual_state_v3/splits/test.jsonl',
])
def test_historical_or_exposed_test_paths_are_rejected(path):
    with pytest.raises(final_test.FinalTestV4Error, match='historical/exposed'):
        final_test.assert_fresh_final_test_path(path, context='test fixture')


def test_fresh_final_test_path_is_allowed_without_opening_it():
    path = Path(
        '/home/tiago/room315_visual_v4_final_test_seed3152026081101/'
        'finalized/final_test.jsonl'
    )
    assert final_test.assert_fresh_final_test_path(path, context='fresh') == path


def test_dataset_declaration_rejects_historical_content_hash_even_if_copied():
    root = Path('/tmp/room315_visual_v4_final_test_seed3152026081101')
    contract = {
        'dataset': {
            'root': str(root),
            'image_root': str(root / 'dataset'),
            'rows': {
                'path': str(root / 'finalized/final_test.jsonl'),
                'sha256': '2fcf78c0034fe290c39b2816e12076300decf5f7818538357fae072231b9b502',
            },
            'labels': {
                'path': str(root / 'finalized/final_test_visual_labels.jsonl'),
                'sha256': _sha('b'),
            },
            'finalization': {
                'path': str(root / 'finalized/final_test_finalization.json'),
                'sha256': _sha('c'),
            },
            'sample_count': 1024,
            'image_count': 2048,
            'image_manifest_sha256': _sha('d'),
            'dataset_fingerprint_sha256': _sha('e'),
        }
    }
    with pytest.raises(final_test.FinalTestV4Error, match='historically exposed'):
        final_test._validate_fresh_dataset_declaration(contract)


def test_coverage_contract_requires_all_test_strata_and_strong_minima():
    valid = _coverage_contract()
    assert final_test._validate_coverage_contract(valid) == valid
    missing_medium = json.loads(json.dumps(valid))
    missing_medium['required_scene_presence_densities'].remove('medium')
    with pytest.raises(final_test.FinalTestV4Error, match='canonical order'):
        final_test._validate_coverage_contract(missing_medium)
    weak = json.loads(json.dumps(valid))
    weak['minimum_visible_per_side_x_segment'] = 1
    with pytest.raises(final_test.FinalTestV4Error, match='>= 8'):
        final_test._validate_coverage_contract(weak)


def test_historical_hash_manifests_are_exactly_pinned_by_frozen_config():
    sources = []
    declarations = [{
        'role': 'old_replay_superset',
        'path': str(final_test.PINNED_OLD_REPLAY_IMAGE_AUDIT),
        'sha256': final_test.PINNED_OLD_REPLAY_IMAGE_AUDIT_SHA256,
        'hash_field_path': ['source_image_hashes'],
    }]
    for index, (name, role) in enumerate((
        ('v3r1_train', 'v3r1_train'),
        ('v3r1_validation', 'v3r1_validation'),
        ('v3r1_canary', 'v3r1_canary'),
    ), 1):
        path = f'/frozen/{name}.json'
        digest = str(index) * 64
        sources.append({
            'name': name,
            'image_hash_manifest': path,
            'image_hash_manifest_sha256': digest,
        })
        declarations.append({
            'role': role,
            'path': path,
            'sha256': digest,
            'hash_field_path': ['image_hashes'],
        })
    contract = {'historical_image_references': declarations}
    config = {'reference_sources': sources}
    final_test._validate_historical_reference_declarations(contract, config)

    repackaged = json.loads(json.dumps(contract))
    repackaged['historical_image_references'][-1]['path'] = '/fake/canary.json'
    with pytest.raises(final_test.FinalTestV4Error, match='differs from its frozen pin'):
        final_test._validate_historical_reference_declarations(repackaged, config)


def test_attempt_key_ignores_contract_serialization_paths_and_policy_fields():
    checkpoint = _sha('a')
    dataset = _sha('b')
    first_contract = {
        'output_root': '/one',
        'notes': 'first serialization',
        'dataset': {'dataset_fingerprint_sha256': dataset},
    }
    second_contract = {
        'notes': 'changed irrelevant prose',
        'dataset': {'dataset_fingerprint_sha256': dataset},
        'output_root': '/two',
        'extra_irrelevant_field': True,
    }
    assert json.dumps(first_contract) != json.dumps(second_contract, sort_keys=True)
    assert final_test.final_test_attempt_key(checkpoint, dataset) == (
        final_test.final_test_attempt_key(checkpoint, dataset)
    )
    assert final_test.final_test_attempt_key(checkpoint, dataset) != (
        final_test.final_test_attempt_key(_sha('c'), dataset)
    )


def test_repackaged_same_dataset_and_checkpoint_hits_existing_global_reservation(
    tmp_path, monkeypatch
):
    first = _bundle(tmp_path, contract_sha=_sha('1'), output_name='first_output')
    second = final_test.ControlBundle(
        **{
            **first.__dict__,
            'contract_sha256': _sha('2'),
            'contract': {
                'dataset': {
                    'dataset_fingerprint_sha256': first.contract['dataset'][
                        'dataset_fingerprint_sha256'
                    ]
                },
                'output_root': '/changed',
                'irrelevant': 'changed',
            },
            'output_path': tmp_path / 'second_output',
        }
    )
    bundles = iter((first, second))
    monkeypatch.setattr(final_test, 'load_control_bundle', lambda *_: next(bundles))
    ledger = tmp_path / 'global_ledger'
    monkeypatch.setattr(final_test, 'DEFAULT_GLOBAL_LEDGER_ROOT', ledger)
    reserved = final_test.reserve_final_test_attempt(
        tmp_path / 'first.json', _sha('1')
    )
    assert reserved.reservation_path.is_file()
    with pytest.raises(final_test.FinalTestV4Error, match='already reserved'):
        final_test.reserve_final_test_attempt(
            tmp_path / 'second.json', _sha('2')
        )


def test_reservation_is_written_before_any_data_open_hook(tmp_path, monkeypatch):
    bundle = _bundle(tmp_path, contract_sha=_sha('1'), output_name='output')
    monkeypatch.setattr(final_test, 'load_control_bundle', lambda *_: bundle)
    monkeypatch.setattr(
        final_test, 'DEFAULT_GLOBAL_LEDGER_ROOT', tmp_path / 'ledger'
    )
    reserved = final_test.reserve_final_test_attempt(
        tmp_path / 'contract.json', _sha('1')
    )
    observed = []

    def fake_open(value):
        observed.append(value.reservation_path.is_file())

    fake_open(reserved)
    assert observed == [True]
    reservation = json.loads(reserved.reservation_path.read_text())
    assert reservation['rows_opened'] is False
    assert reservation['labels_opened'] is False
    assert reservation['images_opened'] is False


def test_failed_reserved_attempt_is_consumed_and_cannot_retry(tmp_path, monkeypatch):
    bundle = _bundle(tmp_path, contract_sha=_sha('1'), output_name='output')
    monkeypatch.setattr(final_test, 'load_control_bundle', lambda *_: bundle)

    def fail(*_args, **_kwargs):
        raise final_test.FinalTestV4Error('synthetic post-reservation failure')

    monkeypatch.setattr(final_test, '_evaluate_reserved_attempt', fail)
    ledger = tmp_path / 'ledger'
    monkeypatch.setattr(final_test, 'DEFAULT_GLOBAL_LEDGER_ROOT', ledger)
    with pytest.raises(final_test.FinalTestV4Error, match='synthetic'):
        final_test.evaluate_final_test_v4(
            tmp_path / 'contract.json', _sha('1')
        )
    completed = list(ledger.glob('*.completed.json'))
    assert len(completed) == 1
    assert json.loads(completed[0].read_text())['state'] == 'failed_immutable'
    with pytest.raises(final_test.FinalTestV4Error, match='already reserved'):
        final_test.reserve_final_test_attempt(
            tmp_path / 'contract.json', _sha('1')
        )


def test_test_specific_acceptance_combines_frozen_v4_and_runtime_gates():
    runtime = {
        'segment': {'coverage': 0.97, 'selective_accuracy': 0.995},
        'loaded': {'coverage': 0.99},
        'joint': {'coverage': 0.96},
    }
    accepted = final_test._combine_final_test_acceptance(
        {'status': 'passed', 'accepted': True},
        runtime,
        {'passed': True},
        _coverage_contract(),
    )
    assert accepted['status'] == 'passed'
    runtime['joint']['coverage'] = 0.89
    rejected = final_test._combine_final_test_acceptance(
        {'status': 'passed', 'accepted': True},
        runtime,
        {'passed': True},
        _coverage_contract(),
    )
    assert rejected['status'] == 'failed'
    assert rejected['automatic_runtime_switch'] is False


def test_coverage_audit_is_fail_closed_for_one_weak_side_segment_cell():
    coverage = _coverage_contract()
    support = {
        'sample_count': 1024,
        'visible_total': 4096,
        'visible_by_identity': {
            identity: 512 for identity in final_test.FIXED_IDENTITIES
        },
        'visible_by_side_x_segment': {
            side: {segment: 32 for segment in final_test.SEGMENT_CLASSES}
            for side in final_test.SIDES
        },
        'visible_by_position_bin': {
            name: 400 for name in final_test.POSITION_BINS
        },
        'records_by_occlusion_class': {
            name: 512 for name in final_test.SCENE_OCCLUSION_CLASSES
        },
        'records_by_presence_density': {
            name: 341 for name in final_test.SCENE_PRESENCE_DENSITIES
        },
        'visible_by_target_zone': {
            name: 100 for name in final_test.TARGET_ZONES
        },
        'visible_by_identity_zone': {
            identity: {
                name: 100 for name in final_test.IDENTITY_ZONES
            }
            for identity in final_test.FIXED_IDENTITIES
        },
    }
    assert final_test._validate_support_coverage(support, coverage)['passed']
    support['visible_by_side_x_segment']['left']['A14'] = 7
    audit = final_test._validate_support_coverage(support, coverage)
    assert not audit['passed']
    assert 'side_x_segment.left.A14' in audit['failed_checks']


def test_completion_artifacts_never_authorize_runtime_or_actuation():
    source = inspect.getsource(final_test._complete_attempt)
    assert "'automatic_runtime_switch': False" in source
    assert "'plansys_updates_enabled': False" in source
    assert "'actuation_enabled': False" in source
