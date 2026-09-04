#!/usr/bin/env python3

import copy
import hashlib
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
CONFIG_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'config'
    / 'room_315_visual_state'
    / 'blocker_training_scenarios.yaml'
)
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_kairos_package_checks as kairos_checks
import room_315_visual_fleet as fleet
import room_315_visual_scenario_generator as generator
import room_315_visual_state_dataset as visual


def _config():
    return yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))


def _visible_label(identity: str = 'R1'):
    side = 'left' if identity.startswith('L') else 'right'
    return {
        'schema_version': visual.VISUAL_STATE_SCHEMA_VERSION,
        'calibration_version': 'test',
        'confidence': 1.0,
        'shuttles': [{
            'id': identity,
            'presence': True,
            'visually_available': True,
            'bbox': [10.0, 20.0, 30.0, 40.0],
            'location': {'side': side, 'block': 'A12E'},
            'rail_position': {
                'available': True,
                's_m': 0.5,
                's_ratio': 0.25,
                'segment_length_m': 2.0,
                'position_uncertainty_m': 0.0,
            },
            'loaded_state': 'loaded',
            'confidence': 1.0,
        }],
        'switches': [],
        'obstacles': [],
    }


def test_authoritative_visual_fleet_reconciles_exactly_eight_identities():
    report = fleet.authoritative_visual_fleet()

    assert report['schema_order'] == [
        'L1', 'L2', 'L3', 'L4',
        'R1', 'R2', 'R3', 'R4',
    ]
    assert report['max_shuttles_per_side'] == 4
    assert set(report['world_entities']) == set(report['schema_order'])
    assert report['world_entities']['L4'] == 'room315_left_shuttle_4'
    assert report['world_entities']['R4'] == 'room315_right_shuttle_4'


def test_twenty_scene_plan_has_all_roles_balanced_targets_and_exact_cardinality():
    scenarios = generator.generate_scenarios(
        _config(),
        count=20,
        seed=31520260727,
    )
    scopes = Counter(row['rail_scope'] for row in scenarios)
    roles = defaultdict(Counter)
    segments = set()

    assert scopes == {
        'left_four': 8,
        'right_four': 8,
        'dual_four_plus_four': 4,
    }
    for scenario in scenarios:
        probe = scenario['relation_probe']
        active = scenario['scene']['rails'][probe['side']]['shuttles']
        assert len(active) == 4
        assert {row['id'] for row in active} == set(
            fleet.identities_for_side(probe['side'])
        )
        assert not generator.scenario_physical_conflicts(scenario)
        roles[probe['target_shuttle_id']]['target'] += 1
        related = set()
        for relation in probe['relations']:
            identity = relation['other_shuttle_id']
            related.add(identity)
            roles[identity][
                'non_blocker'
                if 'non_blocker' in relation['relation']
                else 'blocker'
            ] += 1
        neutrals = set(probe['relation_neutral_shuttle_ids'])
        assert neutrals.isdisjoint(related | {probe['target_shuttle_id']})
        assert neutrals | related | {probe['target_shuttle_id']} == {
            row['id'] for row in active
        }
        for identity in (
            list(neutrals)
            + list(probe['opposite_rail_neutral_shuttle_ids'])
        ):
            roles[identity]['relation_neutral'] += 1
        for side in generator.SIDES:
            for shuttle in scenario['scene']['rails'][side]['shuttles']:
                segments.add((side, shuttle['start_position']['segment']))

    expected_roles = {'target', 'blocker', 'non_blocker', 'relation_neutral'}
    assert all(
        set(roles[identity]) == expected_roles
        for identity in fleet.FIXED_VISUAL_SHUTTLE_IDENTITIES
    )
    for side in ('left', 'right'):
        target_counts = [
            roles[identity]['target']
            for identity in fleet.identities_for_side(side)
        ]
        assert max(target_counts) - min(target_counts) <= 1
    assert segments == {
        (side, block)
        for side in generator.SIDES
        for block in generator.valid_public_segments(side)
    }


def test_l4_or_r4_cannot_be_dropped_from_active_four_shuttle_scene():
    scenario = generator.generate_scenarios(
        _config(),
        count=20,
        seed=31520260727,
    )[0]
    corrupted = copy.deepcopy(scenario)
    side = corrupted['relation_probe']['side']
    corrupted['scene']['rails'][side]['shuttles'] = [
        shuttle
        for shuttle in corrupted['scene']['rails'][side]['shuttles']
        if shuttle['id'] not in {'L4', 'R4'}
    ]

    with pytest.raises(
        generator.VisualScenarioError,
        match='all four identities',
    ):
        generator.validate_scenario(corrupted)


def test_fixed_schema_vectorizer_has_eight_entries_and_explicit_masks():
    normalized = visual.normalize_visual_state_labels(_visible_label('R1'))
    vectorizer = visual.VisualStateLabelVectorizer.fit([normalized])
    mask = vectorizer.target_mask(normalized)

    assert tuple(row['id'] for row in normalized['shuttles']) == (
        'L1', 'L2', 'L3', 'L4',
        'R1', 'R2', 'R3', 'R4',
    )
    assert vectorizer.dim == 200
    assert not any('identity' in name for name in vectorizer.names)
    assert all(
        vectorizer.categorical_values[f'shuttles.{slot}.location.block']
        == list(fleet.AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY)
        for slot in range(8)
    )
    assert all(
        mask[index] == 1.0
        for index, name in enumerate(vectorizer.names)
        if name.startswith('shuttles.4.')
    )
    assert all(
        mask[index] == 0.0
        for index, name in enumerate(vectorizer.names)
        if not name.startswith('shuttles.4.')
    )
    vectorizer.validate_target(normalized)


def test_fixed_location_metrics_do_not_report_identity_accuracy():
    normalized = visual.normalize_visual_state_labels(_visible_label('R1'))
    vectorizer = visual.VisualStateLabelVectorizer.fit([normalized])
    target = vectorizer.transform(normalized)
    record = {
        'true_raw': target,
        'pred_raw': list(target),
        'target_mask': vectorizer.target_mask(normalized),
    }

    metrics = visual.visual_state_metrics([record], vectorizer.names)

    assert 'identity_accuracy' not in metrics
    assert metrics['identity_classification_supported'] is False
    assert metrics['side_accuracy'] == 1.0
    assert metrics['block_accuracy'] == 1.0
    assert metrics['full_location_accuracy'] == 1.0
    assert metrics['top2_block_accuracy'] == 1.0


def test_kairos_package_validation_fails_when_identities_are_absent(tmp_path):
    labels_dir = tmp_path / 'dataset'
    labels_dir.mkdir()
    (labels_dir / 'train_visual_labels.jsonl').write_text(
        json.dumps({
            'sample_id': 'only-r1',
            'visual_state_labels': _visible_label('R1'),
        }) + '\n',
        encoding='utf-8',
    )

    with pytest.raises(
        kairos_checks.KairosPackageValidationError,
        match='no present samples for identities',
    ):
        kairos_checks.validate_kairos_package(tmp_path)


def test_visual_pipeline_has_no_literal_three_slot_iteration():
    paths = [
        SCRIPT_DIR / name
        for name in (
            'room_315_visual_scenario_generator.py',
            'room_315_visual_state_dataset.py',
            'room_315_visual_dataset_audit.py',
            'room_315_visual_error_analysis.py',
            'room_315_visual_state_train_local.py',
            'room_315_kairos_package_checks.py',
        )
    ]
    violations = [
        str(path)
        for path in paths
        if 'range(3)' in path.read_text(encoding='utf-8')
    ]
    assert violations == []


def test_frozen_pilot_files_match_recorded_pre_analysis_fingerprints():
    fingerprint_path = Path(
        '/home/tiago/Downloads/kairos_room315_h200_pilot_results/'
        'error_analysis_baseline_61acabfeb75c/'
        'baseline_artifact_fingerprints_before.json'
    )
    recorded = json.loads(fingerprint_path.read_text(encoding='utf-8'))
    mismatches = []
    for metadata in recorded['files'].values():
        path = Path(metadata['path'])
        if not path.is_file():
            mismatches.append(f'missing:{path}')
            continue
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != metadata['sha256'] or path.stat().st_size != metadata['bytes']:
            mismatches.append(str(path))
    assert mismatches == []
