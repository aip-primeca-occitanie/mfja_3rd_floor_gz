import json
import stat
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))

import room_315_build_runtime_candidate_v4 as candidate_v4


@pytest.fixture(scope='module')
def validated_sources():
    return candidate_v4.validate_sources()


@pytest.fixture(scope='module')
def built_candidate(tmp_path_factory):
    output = tmp_path_factory.mktemp('room315_v4_candidate_parent') / 'candidate'
    result = candidate_v4.build(output)
    return output, result


def test_pinned_sources_strictly_load_and_trusted_canary_handoff_is_bound(
    validated_sources,
):
    assert validated_sources.checkpoint['schema_version'] == (
        candidate_v4.CHECKPOINT_SCHEMA
    )
    assert validated_sources.checkpoint['epoch'] == 11
    assert validated_sources.checkpoint['model_kind'] == candidate_v4.MODEL_KIND
    assert validated_sources.checkpoint['checkpoint_selection_role'] == (
        'validation_only'
    )
    assert validated_sources.checkpoint['canary_seen'] is False
    assert validated_sources.checkpoint['test_seen'] is False
    assert validated_sources.canary_handoff['trusted'] is True
    assert validated_sources.canary_handoff['attempt_key'] == (
        candidate_v4.CANARY_ATTEMPT_KEY
    )
    assert all(validated_sources.canary_handoff['checks'].values())


def test_runtime_thresholds_are_presence_aware_and_validation_derived(
    validated_sources,
):
    policy = candidate_v4.runtime_policy(validated_sources)
    calibration = validated_sources.validation_calibration
    full_coverage = next(
        row for row in calibration['selective_curve']
        if row['requested_coverage'] == 1.0
    )

    assert policy['minimum_segment_confidence'] == (
        full_coverage['confidence_threshold']
    )
    assert policy['minimum_segment_confidence'] == pytest.approx(
        0.9139089813389109, abs=0.0
    )
    assert policy['segment_confidence'] == {
        'calibrated': True,
        'temperature': 0.6007370031399095,
        'derivation': 'validation_selective_curve_100_percent_coverage_floor',
        'requested_coverage': 1.0,
    }
    assert policy['minimum_loaded_confidence'] == 0.5
    assert policy['loaded_confidence']['calibrated'] is False
    assert policy['loaded_confidence']['probability_guarantee'] is False
    assert policy['active_frame_policy'] == 'all_present_slots_must_pass'
    assert policy['presence_contract']['threshold_scope'] == 'present_slots_only'
    assert policy['presence_contract']['missing_or_stale_presence'] == (
        'reject_frame'
    )
    assert policy['s_ratio']['diagnostic_absolute_error_tolerance'] == 0.05
    assert policy['s_ratio']['acceptance_absolute_error_tolerance'] == 0.12
    assert policy['s_ratio']['confidence_gate'] is False


def test_promotion_manifest_matches_v4_core_contract(validated_sources):
    manifest = candidate_v4.promotion_manifest(validated_sources)
    assert manifest['schema_version'] == (
        'room315.visual_runtime_promotion.v4.v1'
    )
    assert manifest['immutable'] is True
    assert manifest['manual_review_approved'] is False
    assert manifest['manual_runtime_review_status'] == 'pending'
    assert manifest['shadow_execution_authorized'] is True
    assert manifest['deployment_mode'] == 'shadow'
    assert manifest['automatic_promotion_allowed'] is False
    assert manifest['manual_only'] is True
    assert manifest['eligibility'] == {
        'shadow_load_and_evaluation': True,
        'active_runtime_review': False,
        'active_runtime_selected': False,
        'active_transition_requires_new_immutable_manifest': True,
    }

    required_artifacts = {
        'checkpoint',
        'training_final_report',
        'canary_final_report',
        'canary_completion_ledger',
        'effective_config',
        'validation_acceptance',
        'validation_segment_calibration',
        'public_topology_contract',
    }
    assert set(manifest['artifacts']) == required_artifacts
    assert all(
        not Path(item['path']).is_absolute()
        for item in manifest['artifacts'].values()
    )
    assert all(
        candidate_v4.HEX_SHA256.fullmatch(item['sha256'])
        for item in manifest['artifacts'].values()
    )

    assert manifest['model_contract'] == {
        'checkpoint_schema_version': candidate_v4.CHECKPOINT_SCHEMA,
        'model_kind': candidate_v4.MODEL_KIND,
        'head_type': 'spatial_query',
        'hidden_dim': 256,
        'attention_heads': 8,
        'dropout': 0.1,
        'slot_order': list(candidate_v4.IDENTITIES),
        'segment_order': list(candidate_v4.SEGMENTS),
        'output_keys': list(candidate_v4.OUTPUT_KEYS),
        'checkpoint_epoch': 11,
        'checkpoint_loading': 'strict',
        'effective_config_fingerprint_sha256': (
            candidate_v4.EFFECTIVE_CONFIG_FINGERPRINT_SHA256
        ),
        'cross_camera_feature_path': False,
        'side_source': 'fixed_identity_prefix',
    }
    assert manifest['preprocessing_contract']['camera_order'] == [
        'left_rail_rgb',
        'right_rail_rgb',
    ]
    assert manifest['preprocessing_contract']['input_shape'] == [
        'B', 6, 240, 320,
    ]
    assert manifest['calibration_contract']['fit_scope'] == 'validation_only'
    assert manifest['topology_contract']['fingerprint_sha256'] == (
        candidate_v4.TOPOLOGY_FINGERPRINT_SHA256
    )
    assert manifest['evidence_policy']['test_loaded'] is False
    assert manifest['evidence_policy']['datasets_bundled'] is False
    assert 'rollback_contract' not in manifest


def test_existing_seven_acceptance_scenarios_are_rebound_to_v4_candidate():
    manifest = candidate_v4.acceptance_scenarios_v4()
    assert manifest['candidate_id'] == candidate_v4.CANDIDATE_ID
    assert manifest['runtime_candidate'] == {
        'runtime_generation': 'v4',
        'runtime_mode': 'shadow',
        'automatic_promotion_allowed': False,
    }
    assert len(manifest['scenarios']) == 7
    assert {row['scenario_id'] for row in manifest['scenarios']} == {
        'accept_l4_loaded',
        'accept_r4_loaded',
        'accept_exact_l2_l4_r4',
        'accept_right_slot3_plus_005',
        'accept_sparse',
        'accept_dense',
        'accept_multi_blocker',
    }
    assert all(
        row['ground_truth']['model_prediction_target'] is False
        for row in manifest['scenarios']
    )


def test_builder_atomically_publishes_complete_read_only_shadow_bundle(
    built_candidate,
):
    output, result = built_candidate
    assert result['status'] == 'CANDIDATE_CREATED'
    assert result['deployment_mode'] == 'shadow'
    assert result['automatic_promotion_allowed'] is False
    assert candidate_v4.sha256_file(
        output / candidate_v4.PROMOTION_MANIFEST_NAME
    ) == result['promotion_manifest_sha256']

    sums = (output / 'SHA256SUMS').read_text(encoding='utf-8').splitlines()
    expected_names = {
        path.name for path in output.iterdir()
        if path.name != 'SHA256SUMS'
    }
    assert {line.split('  ', 1)[1] for line in sums} == expected_names
    for line in sums:
        digest, name = line.split('  ', 1)
        assert candidate_v4.sha256_file(output / name) == digest

    assert not stat.S_IMODE(output.stat().st_mode) & 0o222
    assert all(
        not stat.S_IMODE(path.stat().st_mode) & 0o222
        for path in output.iterdir()
    )
    assert all(path.is_file() for path in output.iterdir())
    assert not any(path.suffix in {'.jsonl', '.png', '.jpg', '.jpeg'} for path in output.iterdir())
    assert not any('test' in path.name.casefold() for path in output.iterdir())

    manifest = json.loads(
        (output / candidate_v4.PROMOTION_MANIFEST_NAME).read_text()
    )
    assert 'rollback_contract' not in manifest
    assert not (output / 'rollback_option.json').exists()
    assert all('rollback' not in path.name for path in output.iterdir())
    for entry in manifest['artifacts'].values():
        artifact_path = output / entry['path']
        assert artifact_path.is_file()
        assert candidate_v4.sha256_file(artifact_path) == entry['sha256']

    runtime_yaml = (output / 'runtime_ros_parameters.yaml').read_text()
    assert 'runtime_generation: v4' in runtime_yaml
    assert 'runtime_mode: shadow' in runtime_yaml
    assert 'v4_promotion_manifest_path:' in runtime_yaml
    assert 'expected_v4_promotion_manifest_sha256:' in runtime_yaml
    assert 'backend:' not in runtime_yaml
    assert 'v4_shadow_mode:' not in runtime_yaml
    assert 'dry_run_state_fusion: true' in runtime_yaml
    assert 'plansys2_update_enabled: false' in runtime_yaml
    assert 'raw_observation_topic: /room_315/visual_state/shadow_v4/raw' in (
        runtime_yaml
    )
    assert '/room_315/visual_state/shadow_v4/raw_model_prediction' in runtime_yaml
    assert 'validation_topic: /room_315/visual_state/shadow_v4/validation' in (
        runtime_yaml
    )
    assert '/room_315/visual_state/shadow_v4/observed_state' in runtime_yaml
    assert result['promotion_manifest_sha256'] in runtime_yaml
    state = json.loads((output / 'candidate_state.json').read_text())
    assert state['state'] == 'shadow_authorized_pending_manual_review'
    assert state['manual_review_approved'] is False
    assert state['shadow_execution_authorized'] is True
    assert state['active_runtime_selected'] is False
    assert state['checkpoint_filename'] == 'checkpoint_epoch_011.pt'
    assert state['runtime_topics'] == {
        'raw_prediction': (
            '/room_315/visual_state/shadow_v4/raw_model_prediction'
        ),
        'raw_observation': '/room_315/visual_state/shadow_v4/raw',
        'validation': '/room_315/visual_state/shadow_v4/validation',
        'accepted_observed_state': (
            '/room_315/visual_state/shadow_v4/observed_state'
        ),
    }


def test_published_manifest_is_accepted_by_fail_closed_v4_core(built_candidate):
    from room_315_visual_runtime_v4 import verify_v4_runtime_promotion

    output, result = built_candidate
    verified = verify_v4_runtime_promotion(
        output / candidate_v4.PROMOTION_MANIFEST_NAME,
        result['promotion_manifest_sha256'],
    )
    assert verified is not None


def test_existing_output_is_never_overwritten(built_candidate):
    output, _ = built_candidate
    marker = candidate_v4.sha256_file(output / 'candidate_state.json')
    with pytest.raises(candidate_v4.CandidateV4BuildError, match='overwrite'):
        candidate_v4.build(output)
    assert candidate_v4.sha256_file(output / 'candidate_state.json') == marker


def test_pinned_hash_mismatch_fails_before_staging(tmp_path, monkeypatch):
    altered = dict(candidate_v4.SOURCE_ARTIFACTS)
    original = altered['validation_acceptance']
    altered['validation_acceptance'] = candidate_v4.SourceArtifact(
        original.source,
        '0' * 64,
        original.bundle_name,
    )
    monkeypatch.setattr(candidate_v4, 'SOURCE_ARTIFACTS', altered)
    output = tmp_path / 'must_not_exist'
    with pytest.raises(candidate_v4.CandidateV4BuildError, match='SHA-256 mismatch'):
        candidate_v4.build(output)
    assert not output.exists()
    assert not list(tmp_path.glob('.*.staging-*'))
