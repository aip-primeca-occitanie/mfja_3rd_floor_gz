import copy
import json
import os
import sys
from collections import Counter
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_v3r1_audit as audit
import room_315_visual_v3r1_common as common
import room_315_visual_v3r1_generator as generator
import room_315_visual_v3r1_pipeline as pipeline
import room_315_visual_v3r1_quota_planner as planner
import room_315_visual_v3r1_reuse as reuse

from room_315_visual_v3_common import TARGET_OFFSETS
from room_315_visual_v3_common import VisualV3Error
from room_315_visual_v3_common import sha256_file
from room_315_visual_v3_common import target_offset_bucket


def _assert_exact_profile(profile, expected_count, expected_per_offset):
    rows = planner.explicit_specs(profile, start_index=0)
    assert len(rows) == expected_count
    by_offset = Counter(row['target_offset'] for row in rows)
    assert by_offset == Counter({
        offset: expected_per_offset for offset in TARGET_OFFSETS
    })
    for row in rows:
        assert row['target_identity'] == 'R4'
        assert row['target_block'] == 'A34E'
        assert row['target_ratio'] == pytest.approx(0.447469343)
        assert row['target_s_ratio'] == pytest.approx(
            common.OPERATIONAL_TARGET_RATIO + row['target_offset']
        )
        assert row['target_offset_bucket'] == target_offset_bucket(
            row['target_offset']
        )
        assert row['operational_target_name'] == 'right_slot_3'
        assert row['operational_target_segment'] == 'A34E'
    return rows


def test_authoritative_target_ratio_and_all_nine_buckets():
    assert common.OPERATIONAL_TARGET_RATIO == pytest.approx(0.447469343)
    assert tuple(TARGET_OFFSETS) == (
        -0.15, -0.10, -0.05, -0.02, 0.0,
        0.02, 0.05, 0.10, 0.15,
    )
    assert len({
        target_offset_bucket(offset) for offset in TARGET_OFFSETS
    }) == 9


def test_train_exact_allocation_balance_presence_and_relations():
    rows = _assert_exact_profile('train', 540, 60)
    for offset in TARGET_OFFSETS:
        selected = [row for row in rows if row['target_offset'] == offset]
        assert Counter(row['target_loaded_state'] for row in selected) == {
            'empty': 30, 'loaded': 30,
        }
        assert Counter(row['presence_class'] for row in selected) == {
            'sparse': 20, 'medium': 20, 'dense': 20,
        }
        assert set(row['relation_family'] for row in selected) == set(
            common.TRAIN_VALIDATION_RELATIONS
        )
        assert len({
            row['configuration_variant'] for row in selected
        }) == 2


def test_validation_exact_allocation_balance_presence_and_relations():
    rows = _assert_exact_profile('validation', 270, 30)
    for offset in TARGET_OFFSETS:
        selected = [row for row in rows if row['target_offset'] == offset]
        assert Counter(row['target_loaded_state'] for row in selected) == {
            'empty': 15, 'loaded': 15,
        }
        assert Counter(row['presence_class'] for row in selected) == {
            'sparse': 10, 'medium': 10, 'dense': 10,
        }
        assert set(row['relation_family'] for row in selected) == set(
            common.TRAIN_VALIDATION_RELATIONS
        )


def test_canary_exact_allocation_and_exclusion():
    rows = _assert_exact_profile('canary', 108, 12)
    for offset in TARGET_OFFSETS:
        selected = [row for row in rows if row['target_offset'] == offset]
        assert Counter(row['target_loaded_state'] for row in selected) == {
            'empty': 6, 'loaded': 6,
        }
        assert Counter(row['presence_class'] for row in selected) == {
            'sparse': 6, 'dense': 6,
        }
        assert set(row['relation_family'] for row in selected) == set(
            common.CANARY_RELATIONS
        )
    assert all(row['profile'] == 'canary' for row in rows)
    assert all(row['canary_family'] for row in rows)


def test_correction_smoke_count_balance_and_relation_diversity():
    rows = planner.smoke_specs()
    assert len(rows) == 36
    assert len({row['spec_id'] for row in rows}) == 36
    assert len({row['matched_pair_id'] for row in rows}) == 18
    assert len({row['relation_family'] for row in rows}) >= 4
    for offset in TARGET_OFFSETS:
        selected = [row for row in rows if row['target_offset'] == offset]
        assert len(selected) == 4
        assert set(row['target_loaded_state'] for row in selected) == {
            'empty', 'loaded',
        }
        assert set(row['presence_class'] for row in selected) == {
            'sparse', 'dense',
        }


def _audit_row(*, deliberate, ratio, family):
    row = {
        'active_identities': ['R4'],
        'loaded_identities': ['R4'],
        'payload_assignment': {'R4': 'loaded'},
        'identity_to_block': {'R4': 'A34E'},
        'identity_to_s_ratio': {'R4': ratio},
        'configuration_family_id': family,
        'relation_family': 'no_relation_observation',
        'presence_class': 'sparse',
        'target_identity': 'R4',
        'target_ratio': (
            common.OPERATIONAL_TARGET_RATIO if deliberate else None
        ),
        'target_offset': 0.0 if deliberate else None,
        'target_offset_bucket': (
            'target' if deliberate else 'not_operational_target'
        ),
        'operational_target_name': (
            'right_slot_3' if deliberate else None
        ),
        'operational_target_segment': 'A34E' if deliberate else None,
    }
    return row


def test_dedicated_and_incidental_counters_are_separate():
    detail = audit._offset_detail([
        _audit_row(
            deliberate=True,
            ratio=common.OPERATIONAL_TARGET_RATIO,
            family='deliberate',
        ),
        _audit_row(
            deliberate=False,
            ratio=common.OPERATIONAL_TARGET_RATIO + 0.01,
            family='incidental',
        ),
    ])
    assert detail['deliberate_exact_offset_count'] == 1
    assert detail['incidental_nearby_count'] == 1


def test_remaining_capacity_guard_fails_closed():
    plan = planner.quota_plan(1356)
    assert plan['remaining_train_capacity'] == 2644
    assert plan['capacity_guard_passed'] is True
    with pytest.raises(VisualV3Error, match='at least 540'):
        planner.quota_plan(common.TRAIN_COUNT - 539)


def test_v3r1_provenance_is_explicit_on_new_rows():
    spec = planner.explicit_specs('train', start_index=1356)[0]
    assert spec['source_profile'] == 'train'
    assert spec['imported_from_v3'] is False
    assert spec['source_scenario_id'] is None
    assert spec['source_manifest_sha256'] is None
    assert spec['v3r1_manifest_revision'] == 'V3R1'


def test_family_isolation_detects_overlap():
    def row(family, core, capture):
        return {
            'configuration_family_id': family,
            'configuration_core_family_id': core,
            'capture_configuration_fingerprint': capture,
        }

    clean = generator._family_overlap({
        'train': [row('ft', 'ct', 'pt')],
        'validation': [row('fv', 'cv', 'pv')],
        'canary': [row('fc', 'cc', 'pc')],
    })
    assert clean['passed'] is True
    leaked = generator._family_overlap({
        'train': [row('same', 'ct', 'pt')],
        'validation': [row('same', 'cv', 'pv')],
        'canary': [row('fc', 'cc', 'pc')],
    })
    assert leaked['passed'] is False
    assert leaked['pairwise']['train_vs_validation']['family_overlap']


def test_hard_link_or_verified_copy_import_preserves_bytes(tmp_path):
    source = tmp_path / 'source.bin'
    destination = tmp_path / 'nested' / 'destination.bin'
    source.write_bytes(b'room315-v3-immutable-source')
    source_before = sha256_file(source)
    method = reuse._link_or_copy(source, destination)
    assert method in {'hard_link', 'verified_copy'}
    assert sha256_file(destination) == source_before
    assert sha256_file(source) == source_before
    if method == 'hard_link':
        assert os.stat(source).st_ino == os.stat(destination).st_ino
    assert reuse._link_or_copy(source, destination) == 'existing_verified'


def test_import_refuses_failed_reuse_scan(tmp_path):
    with pytest.raises(VisualV3Error, match='failed V3 reuse scan'):
        reuse.import_reusable(
            {'passed': False},
            source_capture_root=tmp_path / 'source',
            destination_capture_root=tmp_path / 'destination',
            guard_root=tmp_path / 'guard',
        )


def test_scan_reports_source_preservation_with_fixture(monkeypatch, tmp_path):
    paths = {
        'scenario_manifest': tmp_path / 'manifest.jsonl',
        'training_events': tmp_path / 'events.jsonl',
        'capture_fingerprints': tmp_path / 'fingerprints.jsonl',
        'pipeline_state': tmp_path / 'state.json',
    }
    scenario = {'scenario_id': 's1', 'generation_index': 0}
    event = {'episode_id': 's1', 'sample_id': 'sample1'}
    fingerprint = {'sample_id': 'sample1', 'image_pair_sha256': 'pair'}
    for key, value in (
        ('scenario_manifest', scenario),
        ('training_events', event),
        ('capture_fingerprints', fingerprint),
        ('pipeline_state', {'status': 'stopped'}),
    ):
        paths[key].write_text(json.dumps(value) + '\n', encoding='utf-8')

    monkeypatch.setattr(reuse, '_source_paths', lambda *_args: paths)
    monkeypatch.setattr(
        reuse,
        '_validate_episode',
        lambda *_args, **_kwargs: {
            'scenario_id': 's1',
            'image_pair_sha256': 'pair',
        },
    )
    report = reuse.scan_reusable(
        source_capture_root=tmp_path,
        source_guard_root=tmp_path,
    )
    assert report['reusable_scenario_count'] == 1
    assert report['source_preserved'] is True
    assert report['passed'] is True


def test_imported_manifest_provenance_preserves_source_ids(
    monkeypatch, tmp_path
):
    manifest = tmp_path / 'manifests' / 'train_scenarios.jsonl'
    manifest.parent.mkdir(parents=True)
    source = {
        'scenario_id': 'legacy-scenario',
        'active_identities': ['R4'],
        'generation_index': 7,
    }
    manifest.write_text(json.dumps(source) + '\n', encoding='utf-8')
    monkeypatch.setattr(generator, 'presence_class', lambda _count: 'sparse')
    rows = generator._imported_scenarios(
        {
            'reusable': [{'scenario_id': 'legacy-scenario'}],
            'reusable_scenario_count': 1,
        },
        source_capture_root=tmp_path,
    )
    assert rows[0]['scenario_id'] == 'legacy-scenario'
    assert rows[0]['source_scenario_id'] == 'legacy-scenario'
    assert rows[0]['imported_from_v3'] is True
    assert rows[0]['source_manifest_sha256'] == sha256_file(manifest)
    assert rows[0]['v3r1_manifest_revision'] == 'V3R1'


def test_pipeline_requires_all_passing_gates(tmp_path):
    with pytest.raises(VisualV3Error, match='missing V3R1 quota_plan gate'):
        pipeline.run_pipeline(tmp_path, start_at='train')
    for name in (
        'quota_plan', 'reuse', 'static', 'family_overlap', 'smoke'
    ):
        path = {
            'quota_plan': common.quota_plan_path(tmp_path),
            'reuse': tmp_path / 'v3_to_v3r1_reuse_audit.json',
            'static': tmp_path / 'v3r1_static_audit.json',
            'family_overlap': (
                tmp_path / 'static_family_overlap_audit.json'
            ),
            'smoke': tmp_path / 'v3r1_position_smoke_report.json',
        }[name]
        path.write_text('{"passed": false}\n', encoding='utf-8')
    common.quota_plan_path(tmp_path).write_text(
        '{"passed": true}\n', encoding='utf-8'
    )
    with pytest.raises(VisualV3Error, match='reuse gate did not pass'):
        pipeline.run_pipeline(tmp_path, start_at='train')


def test_quota_plan_path_is_shared_by_planner_audit_and_pipeline(
    monkeypatch, tmp_path
):
    expected = common.quota_plan_path(tmp_path)
    assert common.QUOTA_PLAN_FILENAME == (
        'room315_visual_v3r1_quota_plan.json'
    )
    assert planner.quota_plan_path is common.quota_plan_path
    assert generator.quota_plan_path is common.quota_plan_path
    assert audit.quota_plan_path is common.quota_plan_path
    assert pipeline.quota_plan_path is common.quota_plan_path
    assert pipeline._gate_paths(tmp_path)['quota_plan'] == expected
    assert planner._parser().parse_args([
        '--reuse-count', '1356'
    ]).output == common.quota_plan_path()

    observed = {}
    monkeypatch.setattr(
        audit,
        'static_audit',
        lambda **_kwargs: {'passed': True},
    )
    monkeypatch.setattr(
        audit,
        'run_v3_full_audit',
        lambda **kwargs: observed.update(kwargs) or {'passed': True},
    )
    monkeypatch.setattr(audit, 'atomic_json', lambda *_args: None)
    result = audit.full_audit(
        capture_root=tmp_path / 'capture',
        split_root=tmp_path / 'split',
        canary_root=tmp_path / 'canary',
        guard_root=tmp_path,
    )
    assert result['passed'] is True
    assert observed['quota_plan_path'] == expected


def test_audit_only_pipeline_preserves_successful_stage_results(tmp_path):
    for path in pipeline._gate_paths(tmp_path).values():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('{"passed": true}\n', encoding='utf-8')
    previous_results = {
        stage: {'returncode': 0, 'elapsed_seconds': index + 1.0}
        for index, stage in enumerate(pipeline.AUDIT_PREREQUISITES)
    }
    state_path = tmp_path / 'full_pipeline_state.json'
    state_path.write_text(json.dumps({
        'schema_version': common.PACKAGE_SCHEMA,
        'seed': common.SEED,
        'started_at': '2026-07-30T00:00:00+00:00',
        'status': 'failed',
        'completed_stages': list(pipeline.AUDIT_PREREQUISITES),
        'stage_results': {
            **previous_results,
            'audit': {'returncode': 1, 'elapsed_seconds': 2.0},
        },
        'failure': 'old audit failure',
        'failed_at': '2026-07-31T00:00:00+00:00',
        'elapsed_seconds': 100.0,
    }) + '\n', encoding='utf-8')
    original_commands = pipeline._commands
    pipeline._commands = lambda: {
        'audit': [sys.executable, '-c', 'raise SystemExit(0)'],
    }
    try:
        result = pipeline.run_pipeline(tmp_path, start_at='audit')
    finally:
        pipeline._commands = original_commands
    assert result['status'] == 'completed'
    assert result['completed_stages'] == [
        'train', 'validation', 'canary', 'split', 'audit'
    ]
    assert 'failure' not in result
    assert 'failed_at' not in result
    for stage, previous in previous_results.items():
        assert result['stage_results'][stage] == previous
    assert result['stage_results']['audit']['returncode'] == 0


def test_experiment_config_excludes_canary_and_legacy_test():
    config_path = (
        REPO_ROOT
        / 'mfja_robot_control_config'
        / 'config'
        / 'room_315_vla'
        / 'visual_state_experiment_a_dataset_v3r1.yaml'
    )
    assert audit.default_experiment_config_path(config_path.name) == config_path
    config = config_path.read_text(encoding='utf-8')
    assert 'old_replay_train: 0.5' in config
    assert 'new_hard_case_train: 0.5' in config
    assert 'permitted_for_training: false' in config
    assert 'permitted_for_checkpoint_selection: false' in config
    assert 'create_new_final_test: false' in config
    assert '/test.jsonl' not in config
