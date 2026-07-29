#!/usr/bin/env python3

import copy
import hashlib
import json
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_arbitrary_subset_visual as arbitrary
import room_315_multi_shuttle as multi
import room_315_visual_scenario_generator as scenarios
import room_315_visual_state_dataset as visual


DENSE_ROOT = Path(
    '/home/tiago/room315_eight_shuttle_visual_320_seed31520260727'
)
PILOT_ROOT = Path(
    '/home/tiago/Downloads/kairos_room315_h200_pilot_results'
)


def _tree_fingerprint(root: Path) -> str:
    lines = []
    for path in sorted(item for item in root.rglob('*') if item.is_file()):
        lines.append(
            hashlib.sha256(path.read_bytes()).hexdigest()
            + '  '
            + str(path.relative_to(root))
            + '\n'
        )
    return hashlib.sha256(''.join(lines).encode('utf-8')).hexdigest()


@pytest.fixture(scope='module')
def inventory():
    return arbitrary.presence_inventory()


@pytest.fixture(scope='module')
def smoke(inventory):
    return arbitrary.generate_smoke_manifest(inventory)


def _visible(identity: str) -> dict:
    side = 'left' if identity.startswith('L') else 'right'
    return {
        'id': identity,
        'presence': True,
        'visually_available': True,
        'bbox': [10.0, 20.0, 30.0, 40.0],
        'location': {'side': side, 'block': 'A1E'},
        'rail_position': {
            'available': True,
            's_m': 0.5,
            's_ratio': 0.25,
            'segment_length_m': 2.0,
            'position_uncertainty_m': 0.0,
        },
        'loaded_state': 'empty',
        'confidence': 1.0,
    }


def test_all_255_exact_presence_configurations_are_enumerated(inventory):
    audit = arbitrary.inventory_audit(inventory)

    assert audit['passed']
    assert len(inventory) == 255
    assert len({row['canonical_subset_key'] for row in inventory}) == 255
    assert audit['count_pair_count'] == 24
    assert set(audit['total_count_distribution']) == set(range(1, 9))


def test_all_empty_configuration_is_rejected():
    with pytest.raises(
        arbitrary.ArbitrarySubsetError,
        match='1..255',
    ):
        arbitrary.configuration_record(0)


def test_nonprefix_identity_resolver_preserves_exact_ids():
    left = multi.shuttle_specs_for_identities('left', 'L3')
    right = multi.shuttle_specs_for_identities('right', ['R2', 'R4'])

    assert [spec.short_id for spec in left] == ['L3']
    assert [spec.gazebo_entity_name for spec in left] == [
        'room315_left_shuttle_3'
    ]
    assert [spec.short_id for spec in right] == ['R2', 'R4']
    assert [spec.gazebo_entity_name for spec in right] == [
        'room315_right_shuttle_2',
        'room315_right_shuttle_4',
    ]


def test_arbitrary_smoke_preserves_requested_subsets_and_launch_args(smoke):
    examples = {
        (('L3',), ()),
        (('L2', 'L4'), ()),
        ((), ('R4',)),
        (('L1', 'L4'), ('R2',)),
    }
    actual_examples = {
        (
            tuple(row['left_active_identities']),
            tuple(row['right_active_identities']),
        )
        for row in smoke
    }

    assert examples <= actual_examples
    for row in smoke:
        launch = row['setup']['launch_arguments']
        assert launch['room315_identity_selection_mode'] == 'explicit'
        for side in ('left', 'right'):
            expected = row[f'{side}_active_identities']
            actual = [
                shuttle['id']
                for shuttle in row['scene']['rails'][side]['shuttles']
            ]
            assert actual == expected
            assert launch[f'room315_{side}_active_identities'] == ','.join(
                expected
            )
            assert launch[f'room315_{side}_shuttle_count'] == len(expected)


def test_prefix_substitution_is_rejected(smoke):
    source = next(
        row for row in smoke
        if row['left_active_identities'] == ['L2', 'L4']
        and not row['right_active_identities']
    )
    corrupted = copy.deepcopy(source)
    corrupted['scene']['rails']['left']['shuttles'][0]['id'] = 'L1'

    with pytest.raises(
        scenarios.VisualScenarioError,
        match='exactly match',
    ):
        scenarios.validate_scenario(corrupted)


def test_relation_eligibility_depends_on_exact_target_rail_count():
    assert arbitrary.relation_families(1) == [
        arbitrary.NO_RELATION
    ]
    assert arbitrary.relation_families(2) == list(
        arbitrary.SINGLE_RELATION_FAMILIES
    )
    assert 'multi_blocker' not in arbitrary.relation_families(2)
    assert arbitrary.relation_families(3) == list(
        arbitrary.ALL_RELATION_FAMILIES
    )
    assert arbitrary.relation_families(4) == list(
        arbitrary.ALL_RELATION_FAMILIES
    )


def test_every_smoke_target_is_active_and_relation_is_eligible(
    smoke,
    inventory,
):
    by_id = {
        row['configuration_id']: row
        for row in inventory
    }
    for row in smoke:
        target = row['relation_probe']['target_shuttle_id']
        active = (
            row['left_active_identities']
            + row['right_active_identities']
        )
        assert target in active
        assert row['relation_family'] in by_id[
            row['presence_configuration_id']
        ]['relation_eligibility'][target]


def test_fixed_slots_use_public_identity_not_variable_list_order():
    first = {
        'schema_version': visual.VISUAL_STATE_SCHEMA_VERSION,
        'confidence': 1.0,
        'shuttles': [_visible('R4'), _visible('L3')],
        'switches': [],
        'obstacles': [],
    }
    second = copy.deepcopy(first)
    second['shuttles'].reverse()

    normalized_first = visual.normalize_visual_state_labels(first)
    normalized_second = visual.normalize_visual_state_labels(second)

    assert normalized_first == normalized_second
    assert [row['id'] for row in normalized_first['shuttles']] == list(
        arbitrary.GLOBAL_IDENTITIES
    )
    assert [
        row['presence'] for row in normalized_first['shuttles']
    ] == [False, False, True, False, False, False, False, True]


def test_absent_fixed_identities_are_fully_masked():
    labels = visual.normalize_visual_state_labels({
        'schema_version': visual.VISUAL_STATE_SCHEMA_VERSION,
        'confidence': 1.0,
        'shuttles': [_visible('L3')],
        'switches': [],
        'obstacles': [],
    })
    vectorizer = visual.VisualStateLabelVectorizer()
    mask = vectorizer.target_mask(labels)

    assert vectorizer.dim == 200
    for slot in range(8):
        slot_values = [
            available
            for name, available in zip(vectorizer.names, mask)
            if name.startswith(f'shuttles.{slot}.')
        ]
        assert slot_values
        assert all(slot_values) if slot == 2 else not any(slot_values)


def test_static_smoke_audit_passes(smoke, inventory):
    audit = arbitrary.static_smoke_audit(smoke, inventory)

    assert audit['passed']
    assert audit['scenario_count'] == arbitrary.SMOKE_SCENARIO_COUNT
    assert audit['checks']['no_prefix_substitution']
    assert audit['checks']['all_24_count_pairs']
    assert audit['checks']['fixed_vectorizer_dimension_200']
    assert not any(audit['violations'].values())


def test_full_2040_production_design_covers_every_exact_configuration(inventory):
    rows, summary = arbitrary.production_plan(inventory)
    audit = arbitrary.production_plan_audit(inventory, rows)

    assert len(rows) == 2040
    assert summary['planned_partition_roles_only_no_split_files_created'] == {
        'future_train': 1530,
        'future_validation': 255,
        'future_blind_test': 255,
    }
    assert summary['minimum_alternative']['total_scenarios'] == 1020
    assert audit['passed']
    assert audit['checks']['all_255_configurations']
    assert audit['checks']['exactly_eight_variants_each']


def test_completed_dense_dataset_remains_byte_identical():
    assert _tree_fingerprint(DENSE_ROOT) == (
        'ee89d87260e08e83b2b7e7d93135544f624fd911b8056f9ee4be7111627bdedf'
    )


def test_frozen_pilot_remains_byte_identical():
    assert _tree_fingerprint(PILOT_ROOT) == (
        '6952e51d0bc71c66cffe715b35c0763a133f815650e423a75391c49ec1745b3d'
    )


def test_package_planner_exposes_no_training_kairos_or_split_action():
    source = (
        SCRIPT_DIR / 'room_315_arbitrary_subset_visual.py'
    ).read_text(encoding='utf-8')

    assert 'apptainer exec' not in source
    assert 'ssh ' not in source
    assert 'torch.' not in source
    assert 'train_visual_labels.jsonl' not in source
    assert 'test_visual_labels.jsonl' not in source
