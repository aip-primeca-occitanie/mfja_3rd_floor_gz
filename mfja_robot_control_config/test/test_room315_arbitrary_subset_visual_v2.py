#!/usr/bin/env python3

import copy
import hashlib
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_arbitrary_subset_visual_v2 as v2
import room_315_visual_scenario_generator as scenarios


PACKAGE_ROOT = Path(
    '/home/tiago/room315_arbitrary_subset_visual_2040_v2_seed31520260729'
)


@pytest.fixture(scope='module')
def v1_rows():
    return v2.read_jsonl(v2.V1_PLAN)


@pytest.fixture(scope='module')
def rows():
    return v2.read_jsonl(
        PACKAGE_ROOT / 'configuration_variant_plan_v2.jsonl'
    )


@pytest.fixture(scope='module')
def audit():
    return json.loads(
        (PACKAGE_ROOT / 'design_v2_audit.json').read_text(
            encoding='utf-8'
        )
    )


def _file_map(root: Path) -> dict[str, str]:
    return {
        str(path.relative_to(root)): hashlib.sha256(
            path.read_bytes()
        ).hexdigest()
        for path in sorted(item for item in root.rglob('*') if item.is_file())
    }


def test_v1_independent_assignment_failure_is_reproducible(v1_rows):
    failure = v2._v1_failure_audit(v1_rows)

    assert failure['expected_confirmed_counts_match']
    assert failure['infeasible_unique_rows'] == 1299
    assert failure['initially_feasible_rows'] == 741
    assert failure['target_segment_incompatible_with_assigned_zone'] == 924
    assert failure['same_segment_relation_physically_impossible'] == 597
    assert failure['adjacent_branch_target_without_suffix'] == 42


@pytest.mark.parametrize('side', ('left', 'right'))
def test_A1E_cannot_host_a_same_segment_pair(side):
    length = scenarios._segment_lengths(side)['A1E']
    required_ratio = max(
        scenarios.MIN_SAME_SEGMENT_START_SEPARATION_RATIO,
        scenarios.MIN_SAME_SEGMENT_START_SEPARATION_M / length,
    )

    assert length < scenarios.MIN_SAME_SEGMENT_START_SEPARATION_M
    assert required_ratio > 1.0
    assert not v2.relation_zone_feasible(
        'blocker_ahead_same_segment',
        side,
        'switch',
    )
    assert not v2.relation_zone_feasible(
        'nonblocker_behind_same_segment',
        side,
        'switch',
    )


def test_incompatible_zone_segment_pair_is_rejected():
    assert 'A1E' not in scenarios._zone_segments('right', 'slot')
    assert not v2.ratio_matches_zone(
        'right',
        'A1E',
        'slot',
        0.5,
    )


def test_adjacent_relation_requires_branch_topology(rows):
    source = next(
        row
        for row in rows
        if row['relation_family'] == 'nonblocker_adjacent_branch'
    )
    corrupted = copy.deepcopy(source)
    relation_id = corrupted['relation_identities'][0]
    side = 'left' if relation_id.startswith('L') else 'right'
    relation_shuttle = next(
        shuttle
        for shuttle in corrupted['scene']['rails'][side]['shuttles']
        if shuttle['id'] == relation_id
    )
    target_segment = corrupted['target_segment']
    relation_shuttle['start_position']['segment'] = (
        'A14' if target_segment != 'A14' else 'A23'
    )

    with pytest.raises(
        scenarios.VisualScenarioError,
        match='does not preserve relation',
    ):
        scenarios.validate_scenario(
            corrupted,
            check_physical_geometry=False,
        )


def test_joint_solver_preserves_exact_global_totals(v1_rows):
    assigned, solver = v2.assign_target_zones(v1_rows)

    assert Counter(assigned) == Counter(v2.EXPECTED_ZONE_TOTALS)
    assert solver['retained_original_zone_rows'] == 1564
    assert solver['changed_zone_rows'] == 476
    assert all(
        v2.relation_zone_feasible(
            source['relation_family'],
            v2._side(source['target_identity']),
            zone,
        )
        for source, zone in zip(v1_rows, assigned)
    )


def test_v2_preserves_exact_subsets_targets_payload_and_future_roles(
    v1_rows,
    rows,
):
    assert len(rows) == len(v1_rows) == 2040
    by_parent = {
        row['source_v1_plan_id']: row
        for row in rows
    }
    assert set(by_parent) == {
        row['plan_id'] for row in v1_rows
    }
    for source in v1_rows:
        row = by_parent[source['plan_id']]
        for field in v2.UNCHANGED_V1_FIELDS:
            assert row[field] == source[field]
        for side in ('left', 'right'):
            assert [
                shuttle['id']
                for shuttle in row['scene']['rails'][side]['shuttles']
            ] == source[f'{side}_active_identities']


def test_all_2040_rows_pass_full_physical_topology_and_camera_validation(
    v1_rows,
    rows,
):
    source_by_id = {
        row['plan_id']: row
        for row in v1_rows
    }
    failures = {}
    for row in rows:
        errors = v2.validate_v2_row(
            row,
            source_by_id[row['source_v1_plan_id']],
        )
        if errors:
            failures[row['plan_id']] = errors

    assert not failures


def test_exact_design_distributions_and_fixed_schema(rows, audit):
    assert audit['passed']
    assert not [
        name
        for name, passed in audit['checks'].items()
        if not passed
    ]
    assert Counter(row['relation_family'] for row in rows) == Counter(
        v2.EXPECTED_RELATION_TOTALS
    )
    assert Counter(row['target_zone'] for row in rows) == Counter(
        v2.EXPECTED_ZONE_TOTALS
    )
    assert audit['distributions']['total_active_count'] == (
        v2.EXPECTED_TOTAL_ACTIVE
    )
    assert audit['fixed_schema'] == {
        'dataset_inferred_capacity': False,
        'fixed_identity_order': list(v2.GLOBAL_IDENTITIES),
        'vectorizer_dimension': 200,
        'visual_state_schema': 'room315.visual_state.v3',
    }
    assert audit['preservation'][
        'retained_original_relation_family_rows'
    ] == 2040
    assert audit['preservation']['relation_reassignment_rows'] == 0


def test_all_segments_identity_states_and_roles_are_covered(audit):
    segments = audit['distributions']['all_active_segments']
    assert all(
        segments[f'{side}:{segment}'] > 0
        for side in ('left', 'right')
        for segment in scenarios.valid_public_segments(side)
    )
    for identity in v2.GLOBAL_IDENTITIES:
        assert audit['distributions']['identity_presence'][identity] == 1024
        assert audit['distributions']['identity_absence'][identity] == 1016
        assert audit['distributions']['identity_loaded'][identity] == 512
        assert audit['distributions']['identity_empty'][identity] == 512
        assert audit['distributions']['identity_alone'][identity] == 8
        assert set(
            audit['distributions']['identity_target_zones'][identity]
        ) == set(scenarios.POSITION_ZONES)
        assert all(
            count > 0
            for count in audit['distributions']['identity_roles'][
                identity
            ].values()
        )


def test_every_configuration_has_eight_meaningfully_unique_geometries(rows):
    by_config = defaultdict(list)
    for row in rows:
        by_config[row['configuration_id']].append(row)

    assert len(by_config) == 255
    for variants in by_config.values():
        assert len(variants) == 8
        assert len({row['geometry_key'] for row in variants}) == 8
        canonical_geometries = {
            json.dumps({
                side: {
                    'switches': row['scene']['rails'][side]['switches'],
                    'positions': [
                        (
                            shuttle['id'],
                            shuttle['start_position']['segment'],
                            shuttle['start_position']['s_ratio'],
                        )
                        for shuttle in row['scene']['rails'][side]['shuttles']
                    ],
                }
                for side in ('left', 'right')
            }, sort_keys=True)
            for row in variants
        }
        assert len(canonical_geometries) == 8


def test_target_metric_positions_are_valid(rows):
    for row in rows:
        assert 0.0 <= row['target_s_ratio'] <= 1.0
        assert 0.0 <= row['target_s_m'] <= row[
            'target_segment_length_m'
        ]
        assert row['target_s_m'] == pytest.approx(
            row['target_s_ratio'] * row['target_segment_length_m'],
            abs=1e-6,
        )


def test_static_camera_projectability_is_complete(audit):
    camera = audit['camera_projectability']

    assert camera['projectable_active_identity_instances'] == 8192
    assert camera['invalid_bbox_count'] == 0
    assert camera['invisible_unique_color_region_count'] == 0
    assert audit['checks']['zero_nonprojectable_active_identities']
    assert audit['checks']['zero_invalid_bounding_box_projections']
    assert audit['checks'][
        'every_active_identity_has_visible_unique_color_region'
    ]


def test_package_has_only_design_v2_outputs():
    expected = {
        'README.md',
        'configuration_variant_plan_v2.jsonl',
        'design_v2_audit.json',
        'design_v2_audit.md',
        'infeasible_candidate_summary.json',
        'package_manifest.json',
        'relation_reassignment_log.jsonl',
        'topology_zone_compatibility.json',
        'topology_zone_compatibility.md',
        'v1_to_v2_mapping.jsonl',
    }
    actual = {
        path.name
        for path in PACKAGE_ROOT.iterdir()
    }

    assert actual == expected
    assert (PACKAGE_ROOT / 'relation_reassignment_log.jsonl').read_text(
        encoding='utf-8'
    ) == ''
    assert not any(
        name.startswith(('capture_', 'train_', 'validation_', 'test_'))
        for name in actual
    )


def test_package_manifest_verifies():
    validation = v2.validate_package_manifest(PACKAGE_ROOT)

    assert validation['passed']
    assert validation['verified_file_count'] == 9
    assert not validation['failures']


def test_deterministic_regeneration_is_byte_identical(tmp_path):
    regenerated = tmp_path / 'regenerated'
    repeated = tmp_path / 'repeated'
    v2.prepare_v2_package(
        regenerated,
        declared_root=PACKAGE_ROOT,
    )
    v2.prepare_v2_package(
        repeated,
        declared_root=PACKAGE_ROOT,
    )

    # Reproducibility is a statement about identical inputs.  The archived
    # package records the rail-network hashes that were current when it was
    # frozen; a present-day regeneration intentionally binds the current
    # authoritative networks.  Compare two current regenerations byte for byte
    # and separately prove that topology-independent frozen payloads did not
    # drift.
    regenerated_files = _file_map(regenerated)
    assert regenerated_files == _file_map(repeated)

    topology_derived = {
        'package_manifest.json',
        'topology_zone_compatibility.json',
    }
    archived_files = _file_map(PACKAGE_ROOT)
    assert {
        name: digest
        for name, digest in regenerated_files.items()
        if name not in topology_derived
    } == {
        name: digest
        for name, digest in archived_files.items()
        if name not in topology_derived
    }

    topology = json.loads(
        (regenerated / 'topology_zone_compatibility.json').read_text(
            encoding='utf-8'
        )
    )
    for source in topology['authoritative_sources'].values():
        path = Path(source['path'])
        assert path.is_file()
        assert source['sha256'] == hashlib.sha256(
            path.read_bytes()
        ).hexdigest()

    validation = v2.validate_package_manifest(regenerated)
    assert validation['passed']
    assert validation['verified_file_count'] == 9
    assert not validation['failures']


def test_all_protected_artifacts_remain_byte_identical():
    protected = v2.protected_artifact_audit()

    assert protected['passed']
    assert all(
        result['tree_sha256'] == result['expected_tree_sha256']
        and result['file_count'] == result['expected_file_count']
        for result in protected['artifacts'].values()
    )


def test_no_capture_training_kairos_split_or_approval_action_is_exposed():
    source = (
        SCRIPT_DIR / 'room_315_arbitrary_subset_visual_v2.py'
    ).read_text(encoding='utf-8')

    assert 'apptainer exec' not in source
    assert 'ssh ' not in source
    assert 'torch.' not in source
    assert 'capture_state.json' not in source
    assert 'approved_for_capture\': True' not in source
    assert 'train_visual_labels.jsonl' not in source
    assert 'validation_visual_labels.jsonl' not in source
    assert 'test_visual_labels.jsonl' not in source
