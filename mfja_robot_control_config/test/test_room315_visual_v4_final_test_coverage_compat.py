#!/usr/bin/env python3
"""Contracts for the evaluator-schema compatibility projection."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_final_test_v4 as evaluator
import room_315_visual_v4_final_test as canonical
import room_315_visual_v4_final_test_coverage_compat as compatibility
import room_315_visual_v4_final_test_coverage_extension as coverage_v2


CONFIG = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'room_315_vla'
    / 'visual_state_final_test_v4_coverage_compat_v1.json'
)
CANDIDATE = Path(
    '/home/tiago/room315_visual_runtime_candidate_v4_seed31520260811_epoch11_869d6404_shadow'
)
OLD_REPLAY_AUDIT = Path(
    '/home/tiago/room315_arbitrary_subset_visual_2040_capture_v2_seed31520260729/'
    'captured_production_audit.json'
)


@pytest.fixture(scope='module', autouse=True)
def isolated_prior_inference_artifacts(
    tmp_path_factory: pytest.TempPathFactory,
):
    """Keep compatibility contracts independent of production ledgers."""
    isolated = tmp_path_factory.mktemp('coverage-compat-inference-state')
    patch = pytest.MonkeyPatch()
    patch.setattr(
        coverage_v2,
        'FORBIDDEN_PRIOR_INFERENCE_ARTIFACTS',
        (isolated / 'attempt_ledger', isolated / 'outputs'),
    )
    yield
    patch.undo()


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


def test_prior_inference_guard_remains_fail_closed(tmp_path, monkeypatch) -> None:
    consumed = tmp_path / 'attempt_ledger'
    consumed.mkdir()
    monkeypatch.setattr(
        coverage_v2,
        'FORBIDDEN_PRIOR_INFERENCE_ARTIFACTS',
        (consumed,),
    )
    config = json.loads(CONFIG.read_text(encoding='utf-8'))
    with pytest.raises(canonical.FinalTestError, match='inference attempt'):
        compatibility._validate_v2_source(config)


def test_config_and_cli_are_evaluator_canonical_without_evaluate() -> None:
    config = json.loads(CONFIG.read_text(encoding='utf-8'))
    assert compatibility.CONFIG_SCHEMA == canonical.CONFIG_SCHEMA
    assert compatibility.PREREGISTRATION_SCHEMA == canonical.PREREGISTRATION_SCHEMA
    assert compatibility.PLAN_LOCK_SCHEMA == canonical.PLAN_LOCK_SCHEMA
    assert compatibility.PLAN_SUMMARY_SCHEMA == canonical.PLAN_SUMMARY_SCHEMA
    assert compatibility.FINALIZATION_SCHEMA == canonical.FINALIZATION_SCHEMA
    assert compatibility.DISJOINT_AUDIT_SCHEMA == canonical.DISJOINT_AUDIT_SCHEMA
    assert compatibility.SEAL_SCHEMA == canonical.SEAL_SCHEMA
    assert config['schema_version'] == evaluator.FINAL_TEST_CONFIG_SCHEMA_VERSION
    assert config['scenario_count'] == 1040
    assert config['composition']['lattice_scenarios'] == 1008
    assert config['composition']['stress_scenarios'] == 32
    assert config['composition']['evaluator_control_schemas'] == {
        'config': canonical.CONFIG_SCHEMA,
        'preregistration': evaluator.PREREGISTRATION_SCHEMA_VERSION,
        'plan_summary': canonical.PLAN_SUMMARY_SCHEMA,
        'plan_lock': evaluator.PLAN_LOCK_SCHEMA_VERSION,
        'finalization': evaluator.FINALIZATION_SCHEMA_VERSION,
        'disjoint_audit': 'room315.visual_v4.final_test_disjoint_audit.v1',
        'seal': canonical.SEAL_SCHEMA,
    }
    assert evaluator.FRESH_DATASET_ROOT_PATTERN.fullmatch(
        Path(config['output_root']).name
    )
    source = config['coverage_v2_compatibility_source']
    assert source['projection_type'] == (
        'schema_only_pre_inference_compatibility_projection'
    )
    assert source['manifest_copy_policy'] == 'byte_identical'
    assert source['preserve_v2_row_provenance'] is True
    assert source['source_inference_status'] == 'not_run'
    assert source['source_inference_count'] == 0

    parser = compatibility._parser()
    action = next(
        item for item in parser._actions if getattr(item, 'choices', None)
    )
    assert set(action.choices) == {
        'plan', 'verify-plan', 'status', 'finalize', 'verify-seal'
    }
    assert 'evaluate' not in action.choices


def test_compatibility_manifest_is_exact_v2_with_exact_v1_prefix() -> None:
    config = compatibility.load_config(CONFIG)
    references = canonical._reference_index(config)
    rows = compatibility.materialize_plan(config, references)
    v2_manifest = compatibility.V2_ARTIFACT_PINS['scenario_manifest'][0]
    v1_manifest = coverage_v2.V1_CONTROL_PINS['scenario_manifest'][0]
    assert len(rows) == 1040
    assert compatibility._rows_bytes(rows) == v2_manifest.read_bytes()
    assert compatibility._rows_bytes(rows[:1024]) == v1_manifest.read_bytes()
    assert canonical.sha256_file(v2_manifest) == compatibility.V2_MANIFEST_SHA256
    assert all(
        row.get('coverage_extension_version') == coverage_v2.GENERATOR_VERSION
        for row in rows[1024:]
    )


def test_evaluator_contract_materializer_accepts_control_only_fixture(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    environment = {
        'python_executable': '/test/python',
        'python_prefix': '/test',
        'python_base_prefix': '/test',
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
    monkeypatch.setattr(evaluator, '_environment_snapshot', lambda: environment)
    root = tmp_path / (
        'room315_visual_v4_final_test_seed3152026081101_compatfixture'
    )
    fixture_config = tmp_path / 'visual_state_final_test_compat_fixture.json'
    config = json.loads(CONFIG.read_text(encoding='utf-8'))
    config['output_root'] = str(root)
    _write_json(fixture_config, config)

    plan = compatibility.create_plan(fixture_config, output_root=root)
    assert plan['scenario_count'] == 1040
    assert plan['manifest_sha256'] == compatibility.V2_MANIFEST_SHA256
    preregistration = json.loads((root / 'preregistration.json').read_text())
    plan_lock = json.loads((root / 'plan_lock.json').read_text())
    summary = json.loads((root / 'scenario_summary.json').read_text())
    assert preregistration['schema_version'] == (
        evaluator.PREREGISTRATION_SCHEMA_VERSION
    )
    assert plan_lock['schema_version'] == evaluator.PLAN_LOCK_SCHEMA_VERSION
    assert summary['schema_version'] == canonical.PLAN_SUMMARY_SCHEMA
    assert (root / 'scenario_manifest.jsonl').read_bytes() == (
        compatibility.V2_ARTIFACT_PINS['scenario_manifest'][0].read_bytes()
    )

    finalization = compatibility.build_control_fixture_finalization(
        root, fixture_config
    )
    assert finalization['schema_version'] == evaluator.FINALIZATION_SCHEMA_VERSION
    assert finalization['compatibility_provenance_v2'][
        'rows_labels_images_opened'
    ] is False
    finalization_path = root / 'finalized' / 'final_test_finalization.json'
    _write_json(finalization_path, finalization)

    protocol_path = tmp_path / 'compat_evaluation_protocol_lock.json'
    protocol_artifact = evaluator.materialize_evaluation_protocol_lock(
        protocol_path,
        dataset_config_path=fixture_config,
        dataset_root=root,
        candidate_root=CANDIDATE,
        old_replay_image_audit_path=OLD_REPLAY_AUDIT,
    )
    contract_path = tmp_path / 'compat_evaluation_contract.json'
    contract_artifact = evaluator.materialize_final_test_contract(
        contract_path,
        dataset_config_path=fixture_config,
        dataset_root=root,
        candidate_root=CANDIDATE,
        old_replay_image_audit_path=OLD_REPLAY_AUDIT,
        evaluation_protocol_lock_path=protocol_path,
        evaluation_protocol_lock_sha256=protocol_artifact.sha256,
        output_root=tmp_path / 'evaluation_outputs',
    )
    bundle = evaluator.load_control_bundle(
        contract_path, contract_artifact.sha256
    )
    assert bundle.dataset_config['schema_version'] == (
        evaluator.FINAL_TEST_CONFIG_SCHEMA_VERSION
    )
    assert bundle.preregistration['schema_version'] == (
        evaluator.PREREGISTRATION_SCHEMA_VERSION
    )
    assert bundle.plan_lock['schema_version'] == evaluator.PLAN_LOCK_SCHEMA_VERSION
    assert bundle.contract['dataset']['sample_count'] == 1040
    assert bundle.contract['dataset']['image_count'] == 2080
    assert bundle.contract['prior_exposure'] is False
    assert bundle.evaluation_protocol_lock['test_rows_opened'] is False
    assert bundle.evaluation_protocol_lock['test_labels_opened'] is False
    assert bundle.evaluation_protocol_lock['test_images_opened'] is False
