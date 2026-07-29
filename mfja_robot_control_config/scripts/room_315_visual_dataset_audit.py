#!/usr/bin/env python3
"""Audit Room 315 visual-state scenarios, oracle labels, and dataset splits."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_json_io import iter_jsonl_objects
from room_315_visual_scenario_generator import (
    BLOCKER_SCENE_TYPES,
    POSITION_ZONES,
    SIDES,
    _load_config,
    _named_scene_counts,
    _rail_scope_counts,
    generate_scenarios,
    normalized_blocker_scene_type_weights,
    scenario_physical_conflicts,
    valid_public_segments,
    validate_scenarios,
)
from room_315_shuttle_geometry import ShuttlePosition
from room_315_shuttle_geometry import shuttle_position_conflicts
from room_315_visual_state_dataset import (
    VISUAL_STATE_SCHEMA_VERSION,
    VisualStateLabelVectorizer,
    is_model_prediction_target,
    normalize_visual_state_labels,
)
from room_315_visual_fleet import AUTHORITATIVE_VISUAL_FLEET
from room_315_visual_fleet import AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY
from room_315_visual_fleet import FIXED_VISUAL_SHUTTLE_IDENTITIES


AUDIT_SCHEMA_VERSION = 'room315.visual_dataset_audit.v1'
FORBIDDEN_DERIVED_LABEL_KEYS = {
    'safety_margin',
    'occupied_interval',
    'blocks_route',
    'route_clear',
    'planning_action',
    'planning_actions',
    'next_action',
    'action',
    'action_vector',
}


class VisualDatasetAuditError(ValueError):
    """Raised when an audit input cannot be parsed or validated."""


def _canonical_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'))


def _walk_keys(value: Any) -> set[str]:
    keys: set[str] = set()
    if isinstance(value, dict):
        for key, child in value.items():
            keys.add(str(key))
            keys.update(_walk_keys(child))
    elif isinstance(value, list):
        for child in value:
            keys.update(_walk_keys(child))
    return keys


def _jsonl_rows(path: Path) -> list[dict[str, Any]]:
    return list(
        iter_jsonl_objects(
            path.expanduser(),
            error_type=VisualDatasetAuditError,
            require_object=True,
        )
    )


def _check(passed: bool, **details: Any) -> dict[str, Any]:
    return {'passed': bool(passed), **details}


def audit_scenarios(
    config: dict[str, Any],
    scenarios: list[dict[str, Any]],
    *,
    count: int | None = None,
    seed: int | None = None,
) -> dict[str, Any]:
    validate_scenarios(scenarios, check_physical_geometry=False)
    generator_config = config.get('generator') or {}
    effective_count = int(count if count is not None else generator_config['scenario_count'])
    effective_seed = int(seed if seed is not None else generator_config.get('seed', 315))
    first = generate_scenarios(config, count=effective_count, seed=effective_seed)
    second = generate_scenarios(config, count=effective_count, seed=effective_seed)
    other_seed = generate_scenarios(
        config,
        count=effective_count,
        seed=effective_seed + 1,
    )

    scene_counts = Counter(row['scene_type'] for row in scenarios)
    expected_scene_counts = _named_scene_counts(
        effective_count,
        normalized_blocker_scene_type_weights(config.get('scene_type_weights')),
        BLOCKER_SCENE_TYPES,
    )
    scope_counts_expected = _rail_scope_counts(
        effective_count,
        config.get('rail_scope_weights'),
    )
    payload_states = Counter()
    payload_by_identity: dict[str, Counter[str]] = {
        identity: Counter()
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
    }
    roles_by_identity: dict[str, Counter[str]] = {
        identity: Counter()
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
    }
    segments_by_identity: dict[str, set[str]] = {
        identity: set()
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
    }
    zones_by_identity: dict[str, set[str]] = {
        identity: set()
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
    }
    segment_counts_by_identity: dict[str, Counter[str]] = {
        identity: Counter()
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
    }
    target_zone_counts_by_identity: dict[str, Counter[str]] = {
        identity: Counter()
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
    }
    identity_instances = Counter()
    rail_scopes = Counter()
    side_instances = Counter()
    shuttle_cardinalities = Counter()
    target_zones = Counter()
    target_positions: set[tuple[str, str, float]] = set()
    target_segments: dict[str, set[str]] = {side: set() for side in SIDES}
    segment_instances = Counter()
    missing_positions: list[str] = []
    physical_conflicts: dict[str, list[dict[str, Any]]] = {}
    duplicate_identity_violations: list[str] = []
    unexpected_identity_violations: list[str] = []
    unrepresentable_blocks: list[str] = []
    all_zero_present_block_targets: list[str] = []
    multi_hot_present_block_targets: list[str] = []
    for scenario in scenarios:
        rail_scopes[str(scenario.get('rail_scope') or '')] += 1
        active_count = sum(
            len(scenario['scene']['rails'][side]['shuttles'])
            for side in SIDES
        )
        shuttle_cardinalities[str(active_count)] += 1
        scenario_identities = [
            str(shuttle['id'])
            for side in SIDES
            for shuttle in scenario['scene']['rails'][side]['shuttles']
        ]
        if len(scenario_identities) != len(set(scenario_identities)):
            duplicate_identity_violations.append(scenario['scenario_id'])
        unexpected = sorted(
            set(scenario_identities) - set(FIXED_VISUAL_SHUTTLE_IDENTITIES)
        )
        if unexpected:
            unexpected_identity_violations.append(
                f'{scenario["scenario_id"]}:{",".join(unexpected)}'
            )
        scenario_conflicts = scenario_physical_conflicts(scenario)
        if scenario_conflicts:
            physical_conflicts[scenario['scenario_id']] = scenario_conflicts
        probe = scenario.get('relation_probe') or {}
        active_side = str(probe.get('side') or '')
        target_identity = str(probe.get('target_shuttle_id') or '')
        if target_identity in roles_by_identity:
            roles_by_identity[target_identity]['target'] += 1
        for relation in probe.get('relations') or []:
            identity = str(relation.get('other_shuttle_id') or '')
            relation_kind = str(relation.get('relation') or '')
            if identity in roles_by_identity:
                roles_by_identity[identity][
                    'non_blocker' if 'non_blocker' in relation_kind else 'blocker'
                ] += 1
        for identity in probe.get('relation_neutral_shuttle_ids') or []:
            if identity in roles_by_identity:
                roles_by_identity[identity]['relation_neutral'] += 1
        for identity in probe.get('opposite_rail_neutral_shuttle_ids') or []:
            if identity in roles_by_identity:
                roles_by_identity[identity]['opposite_rail_neutral'] += 1
        for side in SIDES:
            for shuttle in scenario['scene']['rails'][side]['shuttles']:
                identity = str(shuttle['id'])
                identity_instances[identity] += 1
                side_instances[side] += 1
                payload_states[shuttle['loaded_state']] += 1
                if identity in payload_by_identity:
                    payload_by_identity[identity][shuttle['loaded_state']] += 1
                position = shuttle.get('start_position')
                if not isinstance(position, dict):
                    missing_positions.append(f'{scenario["scenario_id"]}:{shuttle["id"]}')
                    continue
                segment_instances[f'{side}:{position["segment"]}'] += 1
                if identity in segments_by_identity:
                    segments_by_identity[identity].add(str(position['segment']))
                    segment_counts_by_identity[identity][
                        str(position['segment'])
                    ] += 1
                    zones_by_identity[identity].add(
                        str(position.get('position_zone') or '')
                    )
                block = str(position['segment'])
                block_one_count = int(
                    block in AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY
                )
                if block_one_count == 0:
                    violation = (
                        f'{scenario["scenario_id"]}:{identity}:{block}'
                    )
                    unrepresentable_blocks.append(violation)
                    all_zero_present_block_targets.append(violation)
                if block_one_count > 1:
                    multi_hot_present_block_targets.append(
                        f'{scenario["scenario_id"]}:{identity}:{block}'
                    )
                if side == active_side and shuttle['id'] == probe.get('target_shuttle_id'):
                    target_zones[str(position.get('position_zone') or '')] += 1
                    target_zone_counts_by_identity[identity][
                        str(position.get('position_zone') or '')
                    ] += 1
                    target_segments[side].add(str(position['segment']))
                    target_positions.add((
                        side,
                        str(position['segment']),
                        float(position['s_ratio']),
                    ))

    expected_segments = {
        f'{side}:{segment}'
        for side in SIDES
        for segment in valid_public_segments(side)
    }
    missing_segments = sorted(expected_segments - set(segment_instances))
    zone_counts = [target_zones[zone] for zone in POSITION_ZONES]
    payload_difference = abs(
        payload_states.get('loaded', 0) - payload_states.get('empty', 0)
    )
    missing_identities = sorted(
        set(FIXED_VISUAL_SHUTTLE_IDENTITIES) - set(identity_instances)
    )
    missing_roles = {
        identity: sorted(
            {
                'target',
                'blocker',
                'non_blocker',
                'relation_neutral',
                'opposite_rail_neutral',
            }
            - set(roles_by_identity[identity])
        )
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
    }
    missing_roles = {
        identity: roles
        for identity, roles in missing_roles.items()
        if roles
    }
    payload_identity_imbalances = {
        identity: dict(payload_by_identity[identity])
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
        if abs(
            payload_by_identity[identity].get('loaded', 0)
            - payload_by_identity[identity].get('empty', 0)
        ) > 1
    }
    # A 20-scene smoke plan gives each identity 12 placements, so exhaustive
    # 14-block per-identity coverage is mathematically impossible.  The full
    # plan is held to exhaustive coverage; smoke requires meaningful diversity
    # while global side/block coverage remains exhaustive.
    per_identity_segment_minimum = (
        len(valid_public_segments('left'))
        if effective_count >= 320
        else 12
        if effective_count >= 64
        else min(6, effective_count)
    )
    per_identity_zone_minimum = (
        len(POSITION_ZONES)
        if effective_count >= 64
        else min(4, len(POSITION_ZONES))
    )
    insufficient_identity_coverage = {
        identity: {
            'segments': sorted(segments_by_identity[identity]),
            'zones': sorted(zones_by_identity[identity]),
        }
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
        if (
            len(segments_by_identity[identity]) < per_identity_segment_minimum
            or len(zones_by_identity[identity]) < per_identity_zone_minimum
        )
    }
    four_left = sum(
        len(row['scene']['rails']['left']['shuttles']) == 4
        and len(row['scene']['rails']['right']['shuttles']) == 0
        for row in scenarios
    )
    four_right = sum(
        len(row['scene']['rails']['right']['shuttles']) == 4
        and len(row['scene']['rails']['left']['shuttles']) == 0
        for row in scenarios
    )
    four_plus_four = sum(
        all(len(row['scene']['rails'][side]['shuttles']) == 4 for side in SIDES)
        for row in scenarios
    )
    vectorizer = VisualStateLabelVectorizer()
    target_counts = {
        identity: roles_by_identity[identity].get('target', 0)
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
    }
    target_rotation_imbalance = (
        max(target_counts.values(), default=0)
        - min(target_counts.values(), default=0)
    )
    dropped_identity_violations = [
        scenario['scenario_id']
        for scenario in scenarios
        if any(
            set(shuttle['id'] for shuttle in scenario['scene']['rails'][side]['shuttles'])
            != (
                set(scenario.get(f'{side}_active_identities') or [])
                if scenario.get('presence_profile')
                == 'arbitrary_identity_subset'
                else
                set(FIXED_VISUAL_SHUTTLE_IDENTITIES[:4])
                if len(scenario['scene']['rails'][side]['shuttles']) == 4
                and side == 'left'
                else set(FIXED_VISUAL_SHUTTLE_IDENTITIES[4:])
                if len(scenario['scene']['rails'][side]['shuttles']) == 4
                and side == 'right'
                else set()
            )
            for side in SIDES
        )
    ]
    checks = {
        'json_serializable': _check(
            all(_canonical_json(row) for row in scenarios),
            rows=len(scenarios),
        ),
        'reproducible_by_seed': _check(
            first == second,
            seed=effective_seed,
        ),
        'manifest_matches_generator': _check(
            scenarios == first,
            rows=len(scenarios),
        ),
        'seed_changes_positions': _check(
            first != other_seed,
            comparison_seed=effective_seed + 1,
        ),
        'every_shuttle_has_continuous_position': _check(
            not missing_positions,
            missing=missing_positions[:20],
        ),
        'collision_free_world_geometry': _check(
            not physical_conflicts,
            affected_scenarios=len(physical_conflicts),
            conflict_pairs=sum(len(value) for value in physical_conflicts.values()),
            violations=dict(list(physical_conflicts.items())[:20]),
        ),
        'authoritative_inventory_exactly_eight': _check(
            AUTHORITATIVE_VISUAL_FLEET['schema_order']
            == list(FIXED_VISUAL_SHUTTLE_IDENTITIES),
            identities=AUTHORITATIVE_VISUAL_FLEET['schema_order'],
        ),
        'all_eight_identities_present': _check(
            not missing_identities,
            counts=dict(sorted(identity_instances.items())),
            missing=missing_identities,
        ),
        'all_identity_roles_covered': _check(
            not missing_roles,
            counts={
                identity: dict(sorted(counts.items()))
                for identity, counts in roles_by_identity.items()
            },
            missing=missing_roles,
        ),
        'target_rotation_balanced': _check(
            target_rotation_imbalance == 0,
            counts=target_counts,
            imbalance=target_rotation_imbalance,
        ),
        'balanced_payload_per_identity': _check(
            not payload_identity_imbalances,
            counts={
                identity: dict(sorted(counts.items()))
                for identity, counts in payload_by_identity.items()
            },
            imbalances=payload_identity_imbalances,
        ),
        'per_identity_segment_and_zone_diversity': _check(
            not insufficient_identity_coverage,
            policy={
                'scenario_count': effective_count,
                'minimum_unique_segments': per_identity_segment_minimum,
                'minimum_unique_zones': per_identity_zone_minimum,
                'full_plan_exhaustive_threshold': 320,
            },
            coverage={
                identity: {
                    'segments': sorted(segments_by_identity[identity]),
                    'zones': sorted(zones_by_identity[identity]),
                }
                for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
            },
            insufficient=insufficient_identity_coverage,
        ),
        'four_shuttle_and_four_plus_four_cardinality': _check(
            four_left == scope_counts_expected['left_four']
            and four_right == scope_counts_expected['right_four']
            and four_plus_four == scope_counts_expected['dual_four_plus_four'],
            four_left=four_left,
            four_right=four_right,
            four_plus_four=four_plus_four,
            expected=scope_counts_expected,
        ),
        'maximum_simultaneous_shuttle_count': _check(
            max((int(value) for value in shuttle_cardinalities), default=0) == 8,
            maximum=max(
                (int(value) for value in shuttle_cardinalities),
                default=0,
            ),
            distribution=dict(sorted(shuttle_cardinalities.items())),
        ),
        'no_duplicate_or_unexpected_identities': _check(
            not duplicate_identity_violations
            and not unexpected_identity_violations,
            duplicate_identities=duplicate_identity_violations,
            unexpected_identities=unexpected_identity_violations,
        ),
        'no_fixed_identity_drops': _check(
            not dropped_identity_violations,
            violations=dropped_identity_violations[:20],
        ),
        'topology_and_relation_validation': _check(
            True,
            topology_violations=0,
            relation_violations=0,
            role_assignment_violations=0,
            validator='room_315_visual_scenario_generator.validate_scenarios',
        ),
        'fixed_schema_and_vectorizer_representability': _check(
            vectorizer.dim == 200
            and vectorizer.to_json()['fixed_identity_order']
            == list(FIXED_VISUAL_SHUTTLE_IDENTITIES)
            and not unrepresentable_blocks
            and not all_zero_present_block_targets
            and not multi_hot_present_block_targets,
            schema_version=VISUAL_STATE_SCHEMA_VERSION,
            vectorizer_dimension=vectorizer.dim,
            fixed_identity_order=list(FIXED_VISUAL_SHUTTLE_IDENTITIES),
            global_block_vocabulary=list(
                AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY
            ),
            unrepresentable_block_targets=unrepresentable_blocks[:20],
            all_zero_present_block_targets=all_zero_present_block_targets[:20],
            multi_hot_present_block_targets=multi_hot_present_block_targets[:20],
            capacity_inferred_from_dataset=False,
        ),
        'target_position_diversity': _check(
            len(target_positions) == len(scenarios)
            and all(
                len(target_segments[side])
                >= min(6, max(4, effective_count // 4))
                for side in SIDES
            ),
            unique_positions=len(target_positions),
            minimum_unique_segments_per_side=min(
                6,
                max(4, effective_count // 4),
            ),
            unique_segments_by_side={
                side: len(target_segments[side])
                for side in SIDES
            },
        ),
        'all_segment_coverage': _check(
            not missing_segments,
            expected_side_segment_pairs=len(expected_segments),
            observed_side_segment_pairs=len(segment_instances),
            missing=missing_segments,
        ),
        'balanced_zone_coverage': _check(
            set(target_zones) == set(POSITION_ZONES)
            and max(zone_counts, default=0) - min(zone_counts, default=0) <= 1,
            counts=dict(sorted(target_zones.items())),
        ),
        'balanced_scenario_families': _check(
            dict(scene_counts) == expected_scene_counts,
            expected=expected_scene_counts,
            actual=dict(sorted(scene_counts.items())),
        ),
        'balanced_payload_states': _check(
            payload_difference <= 1,
            counts=dict(sorted(payload_states.items())),
            difference=payload_difference,
        ),
        'unique_scenario_families': _check(
            len({row['scenario_family'] for row in scenarios}) == len(scenarios),
            families=len({row['scenario_family'] for row in scenarios}),
        ),
    }
    return {
        'count': len(scenarios),
        'seed': effective_seed,
        'checks': checks,
        'distributions': {
            'rail_scope': dict(sorted(rail_scopes.items())),
            'shuttle_cardinality': dict(sorted(shuttle_cardinalities.items())),
            'identity_occurrences': dict(sorted(identity_instances.items())),
            'roles_by_identity': {
                identity: dict(sorted(roles_by_identity[identity].items()))
                for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
            },
            'payload_by_identity': {
                identity: dict(sorted(payload_by_identity[identity].items()))
                for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
            },
            'side_occurrences': dict(sorted(side_instances.items())),
            'segments': dict(sorted(segment_instances.items())),
            'segments_by_identity': {
                identity: dict(
                    sorted(segment_counts_by_identity[identity].items())
                )
                for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
            },
            'target_zones': dict(sorted(target_zones.items())),
            'target_zones_by_identity': {
                identity: dict(
                    sorted(target_zone_counts_by_identity[identity].items())
                )
                for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
            },
            'relation_families': dict(sorted(scene_counts.items())),
            'block_vocabulary_coverage': sorted({
                segment.split(':', 1)[1]
                for segment in segment_instances
            }),
            'payload': dict(sorted(payload_states.items())),
        },
        'passed': all(check['passed'] for check in checks.values()),
    }


def audit_visual_labels(paths: list[Path]) -> dict[str, Any]:
    if not paths:
        return {'status': 'not_run', 'files': [], 'passed': True}
    rows_checked = 0
    labels = []
    forbidden: dict[str, list[str]] = {}
    nonzero_oracle_uncertainty: list[str] = []
    fixed_entry_violations: list[str] = []
    unavailable_mask_violations: list[str] = []
    physical_conflicts: dict[str, list[dict[str, Any]]] = {}
    schema_versions = Counter()
    for path in paths:
        rows = _jsonl_rows(path)
        rows_checked += len(rows)
        for index, row in enumerate(rows):
            label = normalize_visual_state_labels(
                row,
                context=f'{path}:{index + 1}',
            )
            labels.append(label)
            identities = tuple(shuttle['id'] for shuttle in label['shuttles'])
            if identities != FIXED_VISUAL_SHUTTLE_IDENTITIES:
                fixed_entry_violations.append(f'{path}:{index + 1}')
            positions = []
            for shuttle in label['shuttles']:
                rail_position = shuttle['rail_position']
                if not rail_position['available']:
                    continue
                positions.append(ShuttlePosition(
                    shuttle_id=str(shuttle['id']),
                    side=str(shuttle['location']['side']),
                    segment=str(shuttle['location']['block']),
                    s_ratio=float(rail_position['s_ratio']),
                ))
            row_conflicts = shuttle_position_conflicts(positions)
            if row_conflicts:
                physical_conflicts[f'{path}:{index + 1}'] = row_conflicts
            schema_versions[label['schema_version']] += 1
            leaked = sorted(_walk_keys(row) & FORBIDDEN_DERIVED_LABEL_KEYS)
            if leaked:
                forbidden[f'{path}:{index + 1}'] = leaked
            for shuttle in label['shuttles']:
                uncertainty = float(
                    shuttle['rail_position']['position_uncertainty_m']
                )
                if uncertainty != 0.0:
                    nonzero_oracle_uncertainty.append(
                        f'{path}:{index + 1}:{shuttle["id"]}'
                    )
    vectorizer = VisualStateLabelVectorizer.fit(labels)
    for label_index, label in enumerate(labels):
        vectorizer.validate_target(label)
        mask = vectorizer.target_mask(label)
        for slot, shuttle in enumerate(label['shuttles']):
            slot_indexes = [
                index
                for index, name in enumerate(vectorizer.names)
                if name.startswith(f'shuttles.{slot}.')
            ]
            expected = 1.0 if (
                shuttle['presence'] and shuttle['visually_available']
            ) else 0.0
            if any(mask[index] != expected for index in slot_indexes):
                unavailable_mask_violations.append(
                    f'label:{label_index + 1}:{shuttle["id"]}'
                )
    target_keys = (
        list(vectorizer.numeric_keys)
        + list(vectorizer.categorical_values)
    )
    invalid_targets = sorted(
        key for key in target_keys
        if not is_model_prediction_target(key)
        or key.endswith('.position_uncertainty_m')
    )
    checks = {
        'jsonl_validity': _check(True, rows=rows_checked),
        'fixed_eight_schema_v3_preserved': _check(
            set(schema_versions) == {VISUAL_STATE_SCHEMA_VERSION},
            versions=dict(sorted(schema_versions.items())),
        ),
        'all_eight_fixed_entries_present_in_schema': _check(
            not fixed_entry_violations,
            expected=list(FIXED_VISUAL_SHUTTLE_IDENTITIES),
            violations=fixed_entry_violations[:20],
        ),
        'presence_and_visual_masks_valid': _check(
            not unavailable_mask_violations,
            violations=unavailable_mask_violations[:20],
        ),
        'ratio_consistency_and_bounds': _check(
            True,
            labels=len(labels),
        ),
        'collision_free_world_geometry': _check(
            not physical_conflicts,
            affected_rows=len(physical_conflicts),
            conflict_pairs=sum(len(value) for value in physical_conflicts.values()),
            violations=dict(list(physical_conflicts.items())[:20]),
        ),
        'no_planning_or_safety_labels': _check(
            not forbidden,
            violations=forbidden,
        ),
        'gazebo_uncertainty_is_oracle_zero': _check(
            not nonzero_oracle_uncertainty,
            violations=nonzero_oracle_uncertainty[:20],
        ),
        'oracle_uncertainty_excluded_from_targets': _check(
            not invalid_targets,
            invalid_targets=invalid_targets,
            target_count=vectorizer.dim,
        ),
    }
    return {
        'status': 'checked',
        'files': [str(path.expanduser().resolve()) for path in paths],
        'rows': rows_checked,
        'checks': checks,
        'passed': all(check['passed'] for check in checks.values()),
    }


def audit_manifest_label_consistency(
    scenarios: list[dict[str, Any]],
    paths: list[Path],
    *,
    ratio_tolerance: float = 0.015,
) -> dict[str, Any]:
    if not paths:
        return {'status': 'not_run', 'passed': True}
    scenario_by_id = {
        str(scenario['scenario_id']): scenario
        for scenario in scenarios
    }
    rows_by_episode: dict[str, dict[str, Any]] = {}
    duplicate_episodes = []
    missing_episode_ids = []
    for path in paths:
        for index, row in enumerate(_jsonl_rows(path), start=1):
            episode_id = str(row.get('episode_id') or '').strip()
            if not episode_id:
                missing_episode_ids.append(f'{path}:{index}')
                continue
            if episode_id in rows_by_episode:
                duplicate_episodes.append(episode_id)
            rows_by_episode[episode_id] = row

    expected_ids = set(scenario_by_id)
    actual_ids = set(rows_by_episode)
    missing_scenarios = sorted(expected_ids - actual_ids)
    unexpected_episodes = sorted(actual_ids - expected_ids)
    family_mismatches = []
    identity_mismatches = []
    payload_mismatches = []
    position_mismatches = []
    relation_scenario_mismatches = set()
    for episode_id in sorted(expected_ids & actual_ids):
        scenario = scenario_by_id[episode_id]
        row = rows_by_episode[episode_id]
        if str(row.get('scenario_family') or '') != str(scenario['scenario_family']):
            family_mismatches.append(episode_id)
        label = normalize_visual_state_labels(
            row,
            context=f'episode {episode_id}',
        )
        actual_shuttles = {
            str(shuttle['id']): shuttle
            for shuttle in label['shuttles']
            if shuttle['presence']
        }
        visible_shuttles = {
            str(shuttle['id'])
            for shuttle in label['shuttles']
            if shuttle['presence'] and shuttle['visually_available']
        }
        expected_shuttles = {
            str(shuttle['id']): (side, shuttle)
            for side in SIDES
            for shuttle in scenario['scene']['rails'][side]['shuttles']
        }
        if set(actual_shuttles) != set(expected_shuttles):
            identity_mismatches.append({
                'episode_id': episode_id,
                'expected': sorted(expected_shuttles),
                'actual': sorted(actual_shuttles),
            })
            if scenario.get('relation_probe'):
                relation_scenario_mismatches.add(episode_id)
            continue
        if visible_shuttles != set(expected_shuttles):
            identity_mismatches.append({
                'episode_id': episode_id,
                'expected_visible': sorted(expected_shuttles),
                'actual_visible': sorted(visible_shuttles),
            })
            if scenario.get('relation_probe'):
                relation_scenario_mismatches.add(episode_id)
            continue
        for shuttle_id, (expected_side, expected) in expected_shuttles.items():
            actual = actual_shuttles[shuttle_id]
            if actual['loaded_state'] != expected['loaded_state']:
                payload_mismatches.append(f'{episode_id}:{shuttle_id}')
            expected_position = expected.get('start_position')
            if not isinstance(expected_position, dict):
                continue
            actual_position = actual['rail_position']
            segment_matches = (
                str(actual['location']['side']) == expected_side
                and str(actual['location']['block'])
                == str(expected_position['segment'])
            )
            ratio_error = abs(
                float(actual_position['s_ratio'])
                - float(expected_position['s_ratio'])
            )
            if (
                not actual_position['available']
                or not segment_matches
                or ratio_error > ratio_tolerance
            ):
                position_mismatches.append({
                    'episode_id': episode_id,
                    'shuttle_id': shuttle_id,
                    'expected_side': expected_side,
                    'actual_side': str(actual['location']['side']),
                    'expected_segment': str(expected_position['segment']),
                    'actual_segment': str(actual['location']['block']),
                    'expected_s_ratio': float(expected_position['s_ratio']),
                    'actual_s_ratio': float(actual_position['s_ratio']),
                    'absolute_ratio_error': round(ratio_error, 6),
                })
                if scenario.get('relation_probe'):
                    relation_scenario_mismatches.add(episode_id)

    checks = {
        'manifest_and_label_episode_ids_match': _check(
            not (
                missing_episode_ids
                or duplicate_episodes
                or missing_scenarios
                or unexpected_episodes
            ),
            missing_episode_id_rows=missing_episode_ids[:20],
            duplicate_episodes=sorted(set(duplicate_episodes))[:20],
            missing_scenarios=missing_scenarios[:20],
            unexpected_episodes=unexpected_episodes[:20],
        ),
        'scenario_families_match': _check(
            not family_mismatches,
            violations=family_mismatches[:20],
        ),
        'shuttle_identities_match': _check(
            not identity_mismatches,
            violations=identity_mismatches[:20],
        ),
        'payload_states_match': _check(
            not payload_mismatches,
            violations=payload_mismatches[:20],
        ),
        'captured_positions_match_manifest': _check(
            not position_mismatches,
            ratio_tolerance=ratio_tolerance,
            violations=position_mismatches[:20],
        ),
        'scenario_relations_preserved_after_capture': _check(
            not relation_scenario_mismatches,
            checked_scenarios=sum(
                bool(scenario.get('relation_probe'))
                for scenario in scenarios
            ),
            violations=sorted(relation_scenario_mismatches)[:20],
        ),
    }
    return {
        'status': 'checked',
        'scenario_rows': len(scenarios),
        'label_rows': len(rows_by_episode),
        'checks': checks,
        'passed': all(check['passed'] for check in checks.values()),
    }


def audit_split_families(splits_dir: Path | None) -> dict[str, Any]:
    if splits_dir is None:
        return {'status': 'not_run', 'passed': True}
    root = splits_dir.expanduser().resolve()
    manifest_path = root / 'split_manifest.json'
    if not manifest_path.is_file():
        raise VisualDatasetAuditError(f'missing split manifest: {manifest_path}')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    split_entries = manifest.get('splits')
    if not isinstance(split_entries, dict):
        raise VisualDatasetAuditError('split_manifest.json is missing splits')
    family_sets: dict[str, set[str]] = {}
    jsonl_family_sets: dict[str, set[str]] = {}
    for split_name in ('train', 'val', 'test'):
        entry = split_entries.get(split_name)
        if not isinstance(entry, dict):
            raise VisualDatasetAuditError(f'split manifest is missing {split_name}')
        family_sets[split_name] = {
            str(value) for value in entry.get('families') or []
        }
        data_path = root / str(entry.get('file') or f'{split_name}.jsonl')
        rows = _jsonl_rows(data_path)
        jsonl_family_sets[split_name] = {
            str(row.get('scenario_family') or '').strip()
            for row in rows
        }
    overlaps: dict[str, list[str]] = {}
    names = ('train', 'val', 'test')
    for first_index, first_name in enumerate(names):
        for second_name in names[first_index + 1:]:
            leaked = sorted(family_sets[first_name] & family_sets[second_name])
            leaked.extend(sorted(
                jsonl_family_sets[first_name] & jsonl_family_sets[second_name]
            ))
            if leaked:
                overlaps[f'{first_name}/{second_name}'] = sorted(set(leaked))
    manifest_matches_rows = all(
        family_sets[name] == jsonl_family_sets[name]
        for name in names
    )
    checks = {
        'split_jsonl_validity': _check(True),
        'manifest_matches_rows': _check(
            manifest_matches_rows,
            manifest={
                name: sorted(values)
                for name, values in family_sets.items()
            },
            rows={
                name: sorted(values)
                for name, values in jsonl_family_sets.items()
            },
        ),
        'no_scenario_family_leakage': _check(
            not overlaps,
            overlaps=overlaps,
        ),
    }
    return {
        'status': 'checked',
        'directory': str(root),
        'checks': checks,
        'passed': all(check['passed'] for check in checks.values()),
    }


def build_audit_report(
    *,
    config_path: Path,
    scenario_manifest: Path | None = None,
    count: int | None = None,
    seed: int | None = None,
    label_paths: list[Path] | None = None,
    splits_dir: Path | None = None,
    require_complete: bool = False,
) -> dict[str, Any]:
    config = _load_config(config_path)
    scenarios = (
        _jsonl_rows(scenario_manifest)
        if scenario_manifest is not None
        else generate_scenarios(config, count=count, seed=seed)
    )
    labels = list(label_paths or [])
    if splits_dir is not None and not labels:
        labels = sorted(splits_dir.expanduser().glob('*_visual_labels.jsonl'))
    scenario_report = audit_scenarios(
        config,
        scenarios,
        count=count,
        seed=seed,
    )
    label_report = audit_visual_labels(labels)
    consistency_report = audit_manifest_label_consistency(scenarios, labels)
    split_report = audit_split_families(splits_dir)
    complete = (
        label_report['status'] == 'checked'
        and consistency_report['status'] == 'checked'
        and split_report['status'] == 'checked'
    )
    passed = (
        scenario_report['passed']
        and label_report['passed']
        and consistency_report['passed']
        and split_report['passed']
        and (complete or not require_complete)
    )
    return {
        'schema_version': AUDIT_SCHEMA_VERSION,
        'configuration': str(config_path.expanduser().resolve()),
        'scenario_manifest': (
            str(scenario_manifest.expanduser().resolve())
            if scenario_manifest is not None
            else 'generated_in_memory'
        ),
        'scenario_audit': scenario_report,
        'visual_label_audit': label_report,
        'manifest_label_consistency_audit': consistency_report,
        'split_audit': split_report,
        'complete_dataset_checks_run': complete,
        'passed': passed,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Audit Room 315 visual scenario reproducibility, coverage, oracle '
            'labels, and train/val/test family isolation.'
        )
    )
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--scenario-manifest', type=Path)
    parser.add_argument('--count', type=int)
    parser.add_argument('--seed', type=int)
    parser.add_argument('--labels', type=Path, nargs='*', default=[])
    parser.add_argument('--splits-dir', type=Path)
    parser.add_argument('--report', type=Path)
    parser.add_argument(
        '--require-complete',
        action='store_true',
        help='Fail unless visual label files and split files are also audited.',
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = build_audit_report(
        config_path=args.config,
        scenario_manifest=args.scenario_manifest,
        count=args.count,
        seed=args.seed,
        label_paths=args.labels,
        splits_dir=args.splits_dir,
        require_complete=args.require_complete,
    )
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    if args.report is not None:
        report_path = args.report.expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(rendered + '\n', encoding='utf-8')
    print(rendered)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
