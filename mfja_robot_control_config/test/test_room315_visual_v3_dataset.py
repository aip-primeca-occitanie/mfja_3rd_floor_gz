import copy
import json
import sys
from pathlib import Path

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_v3_audit as audit
import room_315_visual_v3_common as common
import room_315_visual_v3_generator as generator
import room_315_visual_v3_quota_planner as planner
import room_315_visual_v3_splitter as splitter
import room_315_visual_v3_pipeline as pipeline
from room_315_visual_state_capture import _apply_render_variation


def test_legacy_test_path_is_rejected_without_opening_or_hashing(tmp_path):
    called = False

    def hasher(_path):
        nonlocal called
        called = True
        raise AssertionError('must not hash a path rejected by name')

    forbidden = tmp_path / 'test.jsonl'
    forbidden.write_text('{}\n', encoding='utf-8')
    with pytest.raises(common.VisualV3Error, match='legacy Test'):
        common.assert_allowed_input(forbidden, hasher=hasher)
    assert called is False


def test_forbidden_test_hash_is_rejected_for_an_explicit_non_test_path(tmp_path):
    candidate = tmp_path / 'candidate.jsonl'
    candidate.write_text('{}\n', encoding='utf-8')
    forbidden_hash = next(iter(common.FORBIDDEN_TEST_HASHES))
    with pytest.raises(common.VisualV3Error, match='prohibited legacy Test'):
        common.assert_allowed_input(candidate, hasher=lambda _path: forbidden_hash)


def test_quota_plan_is_deterministic_feasible_and_exact():
    first = planner.quota_plan()
    second = planner.quota_plan()
    assert first == second
    assert first['passed'] is True
    assert first['unsatisfied_cells'] == []
    assert first['valid_conditional_cell_count'] == 8 * 2 * 14 * 9
    assert first['expected_scenario_total'] == 4512
    assert first['expected_canary_total'] == 256


def test_identity_block_validation_uses_authoritative_vocabulary():
    for identity in common.IDENTITIES:
        for block in common.BLOCKS:
            common.validate_identity_block(identity, block)
    with pytest.raises(common.VisualV3Error):
        common.validate_identity_block('L4', 'NOT_A_BLOCK')
    with pytest.raises(common.VisualV3Error):
        common.validate_identity_block('L9', 'A1E')


def test_position_bin_and_target_offset_assignment():
    assert common.position_bin(0.05) == 'p05'
    assert common.position_bin(0.501) == 'p50'
    assert common.position_bin(0.95) == 'p95'
    assert common.target_offset_bucket(-0.02) == 'minus_0.02'
    assert common.target_offset_bucket(0.0) == 'target'
    assert common.target_offset_bucket(0.15) == 'plus_0.15'
    with pytest.raises(common.VisualV3Error):
        common.target_offset_bucket(0.03)


def test_complete_target_lattice_is_allocated_before_relational_rows():
    rows = planner.build_specs('train')
    cells = {
        (
            row['target_identity'],
            row['target_loaded_state'],
            row['target_block'],
            row['target_position_bin'],
        )
        for row in rows[:2016]
    }
    assert len(cells) == 2016
    assert all(row['relation_family'] == planner.RELATIONS[0] for row in rows[:2016])


def test_presence_cardinalities_payloads_and_canary_exclusion():
    for profile, expected in (('train', 4000), ('validation', 512), ('canary', 256)):
        rows = planner.build_specs(profile)
        assert len(rows) == expected
        assert {len(row['active_identities']) for row in rows} == set(range(1, 9))
        assert all(row['profile'] == profile for row in rows)
    canary = planner.build_specs('canary')
    assert all(row['canary_family'] for row in canary)
    assert not any(row['profile'] in {'train', 'validation'} for row in canary)


def test_configuration_family_hash_uses_semantic_fields_not_timestamp():
    record = {
        'active_identities': ['L2', 'L4', 'R4'],
        'loaded_identities': ['L4', 'R4'],
        'identity_to_block': {'L2': 'A12E', 'L4': 'A34E', 'R4': 'A12E'},
        'identity_to_position_bin': {'L2': 'p40', 'L4': 'p60', 'R4': 'p40'},
        'relation_family': 'no_relation_observation',
        'target_zone': 'slot',
        'occlusion_class': 'clear',
        'render_bucket': 'nominal',
    }
    first = common.configuration_family_id(record)
    changed = dict(record, capture_timestamp='different')
    assert common.configuration_family_id(changed) == first
    changed['loaded_identities'] = ['L2']
    assert common.configuration_family_id(changed) != first


def _split_row(name, family, core):
    return {
        'sample_id': name,
        'traceability_metadata': {
            'scenario_id': name,
            'configuration_family_id': family,
            'configuration_core_family_id': core,
        },
    }


def test_grouped_split_and_image_overlap_detection():
    clean = splitter.overlap_audit(
        [_split_row('train', 'family-train', 'core-train')],
        [_split_row('validation', 'family-validation', 'core-validation')],
        [_split_row('canary', 'family-canary', 'core-canary')],
        old_replay_core={'core-old'},
        image_hashes={
            'train': {'image-train'},
            'validation': {'image-validation'},
            'canary': {'image-canary'},
        },
    )
    assert clean['passed'] is True
    leaked = splitter.overlap_audit(
        [_split_row('train', 'same-family', 'core-train')],
        [_split_row('validation', 'same-family', 'core-old')],
        [_split_row('canary', 'family-canary', 'core-canary')],
        old_replay_core={'core-old'},
        image_hashes={
            'train': {'same-image'},
            'validation': {'same-image'},
            'canary': {'image-canary'},
        },
    )
    assert leaked['passed'] is False
    assert leaked['pairwise']['train_vs_validation']['configuration_family_overlap']
    assert leaked['pairwise']['train_vs_validation']['image_sha256_overlap']
    assert leaked['validation_vs_old_replay_core_family_overlap']


def test_duplicate_scenario_detection():
    row = {
        'scenario_id': 'duplicate',
        'relation_family': 'no_relation_observation',
        'active_identities': [],
        'loaded_identities': [],
        'identity_to_block': {},
        'configuration_family_id': 'duplicate-family',
    }
    result = generator.static_manifest_audit([row, copy.deepcopy(row)])
    assert result['passed'] is False
    assert 'duplicate scenario IDs' in result['errors']


def test_safe_resume_and_atomic_manifest_write(tmp_path):
    root = tmp_path / 'output'
    configuration = {'schema_version': common.PACKAGE_SCHEMA, 'seed': common.SEED}
    first = common.prepare_output_root(root, configuration, resume=False)
    second = common.prepare_output_root(root, configuration, resume=True)
    assert first == second
    with pytest.raises(common.VisualV3Error, match='mismatch'):
        common.prepare_output_root(
            root,
            {**configuration, 'seed': common.SEED + 1},
            resume=True,
        )
    target = root / 'atomic.json'
    common.atomic_json(target, {'value': 1})
    assert json.loads(target.read_text(encoding='utf-8')) == {'value': 1}
    assert not list(root.glob('.atomic.json.*.tmp'))


def test_hard_case_detection_includes_anchor_and_right_slot3():
    def row(sample, trace):
        return {'sample_id': sample, 'traceability_metadata': trace}

    anchor = {
        'active_identities': ['L2', 'L4', 'R4'],
        'loaded_identities': ['L4', 'R4'],
        'identity_to_block': {'L2': 'A12E', 'L4': 'A34E', 'R4': 'A12E'},
        'target_identity': 'L4',
        'target_offset_bucket': 'not_operational_target',
    }
    arrival = {
        'active_identities': ['R4'],
        'loaded_identities': ['R4'],
        'identity_to_block': {'R4': 'A34E'},
        'target_identity': 'R4',
        'target_offset_bucket': 'plus_0.02',
    }
    result = audit._hard_cases({
        'train': ([row('anchor', anchor), row('arrival', arrival)], [{}, {}]),
    })
    assert result['combined']['complete_anchor_scene'] == 1
    assert result['combined']['right_slot3_position_samples'] == 1
    assert result['combined']['l4_r4_both_loaded'] == 1


def test_expected_render_relation_and_loaded_distributions_exist():
    plan = planner.quota_plan()
    for profile in ('train', 'validation', 'canary'):
        distribution = plan['distributions'][profile]
        assert distribution['scenario_count'] == planner.COUNT_BY_PROFILE[profile]
        assert set(distribution['render_bucket']) == set(common.RENDER_BUCKETS)
        assert set(distribution['relation_family']) == set(common.RELATIONS)
        rows = planner.build_specs(profile)
        for identity in common.IDENTITIES:
            states = {
                row['payload_assignment'][identity]
                for row in rows
                if identity in row['active_identities']
            }
            assert states == {'empty', 'loaded'}


def test_no_test_role_can_be_configured_or_written(tmp_path):
    with pytest.raises(common.VisualV3Error, match='Test key'):
        common.assert_no_test_role({'test_path': '/tmp/anything'})
    with pytest.raises(common.VisualV3Error, match='Test key'):
        common.atomic_json(tmp_path / 'bad.json', {'test_evaluation': False})


def test_capture_render_variation_is_small_deterministic_and_opt_in():
    frame = np.full((16, 16, 3), 100, dtype=np.uint8)
    assert np.array_equal(
        _apply_render_variation(frame, None, camera='left_rail_rgb'),
        frame,
    )
    variation = {'bucket': 'noise_low', 'deterministic_seed': 315}
    first = _apply_render_variation(frame, variation, camera='left_rail_rgb')
    second = _apply_render_variation(frame, variation, camera='left_rail_rgb')
    assert np.array_equal(first, second)
    assert not np.array_equal(first, frame)
    assert abs(float(first.mean()) - 100.0) < 2.0
    with pytest.raises(ValueError, match='unsupported render variation'):
        _apply_render_variation(
            frame,
            {'bucket': 'unrealistic'},
            camera='left_rail_rgb',
        )


def test_full_pipeline_is_guarded_by_passing_smoke_report(tmp_path):
    with pytest.raises(common.VisualV3Error, match='missing smoke report'):
        pipeline.run_pipeline(tmp_path, start_at='train')
    report = tmp_path / 'smoke' / 'smoke_generation_report.json'
    report.parent.mkdir(parents=True)
    report.write_text('{"passed":false}\n', encoding='utf-8')
    with pytest.raises(common.VisualV3Error, match='smoke audit passes'):
        pipeline.run_pipeline(tmp_path, start_at='train')
