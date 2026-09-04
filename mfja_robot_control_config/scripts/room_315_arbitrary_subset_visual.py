#!/usr/bin/env python3
"""Plan and audit true arbitrary-identity-subset Room 315 visual datasets."""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import os
import shutil
import stat
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
from typing import Any

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_fleet import FIXED_VISUAL_SHUTTLE_IDENTITIES
from room_315_visual_scenario_generator import (
    ARBITRARY_IDENTITY_PRESENCE_PROFILE,
    BLOCKER_SCENE_TYPES,
    POSITION_ZONES,
    REQUIRED_CAMERAS,
    SCHEMA_VERSION,
    SIDES,
    SWITCH_NAMES,
    _build_blocker_scenario,
    _canonical_json,
    _family_payload,
    _hash,
    _launch_arguments,
    scenario_physical_conflicts,
    valid_public_segments,
    validate_scenario,
)
from room_315_visual_state_dataset import (
    VISUAL_STATE_SCHEMA_VERSION,
    VisualStateLabelVectorizer,
)


SEED = 31520260728
SMOKE_SCENARIO_COUNT = 96
PRODUCTION_VARIANTS = 8
MINIMUM_VARIANTS = 4
GLOBAL_IDENTITIES = tuple(FIXED_VISUAL_SHUTTLE_IDENTITIES)
LEFT_IDENTITIES = GLOBAL_IDENTITIES[:4]
RIGHT_IDENTITIES = GLOBAL_IDENTITIES[4:]
SIDE_IDENTITIES = {'left': LEFT_IDENTITIES, 'right': RIGHT_IDENTITIES}
NO_RELATION = 'no_relation_observation'
SINGLE_RELATION_FAMILIES = BLOCKER_SCENE_TYPES[:-1]
ALL_RELATION_FAMILIES = BLOCKER_SCENE_TYPES
SMOKE_SCHEMA = 'room315.arbitrary_subset_smoke_manifest.v1'
INVENTORY_SCHEMA = 'room315.presence_configuration_inventory.v1'
AUDIT_SCHEMA = 'room315.arbitrary_subset_static_audit.v1'
PLAN_SCHEMA = 'room315.arbitrary_subset_production_plan.v1'


class ArbitrarySubsetError(ValueError):
    """Raised when an exact identity-subset contract is violated."""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def stable_int(*parts: Any) -> int:
    payload = ':'.join(str(part) for part in parts)
    return int(hashlib.sha256(payload.encode('utf-8')).hexdigest()[:16], 16)


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        ''.join(_canonical_json(row) + '\n' for row in rows),
        encoding='utf-8',
    )


def subset_for_mask(mask: int, identities: tuple[str, ...]) -> list[str]:
    return [
        identity
        for index, identity in enumerate(identities)
        if mask & (1 << index)
    ]


def relation_families(active_on_target_rail: int) -> list[str]:
    if active_on_target_rail < 1 or active_on_target_rail > 4:
        raise ArbitrarySubsetError(
            'target rail must have between one and four active identities'
        )
    if active_on_target_rail == 1:
        return [NO_RELATION]
    if active_on_target_rail == 2:
        return list(SINGLE_RELATION_FAMILIES)
    return list(ALL_RELATION_FAMILIES)


def configuration_record(mask: int) -> dict[str, Any]:
    if mask <= 0 or mask >= 1 << len(GLOBAL_IDENTITIES):
        raise ArbitrarySubsetError(
            f'presence mask must be in 1..255, got {mask}'
        )
    left = subset_for_mask(mask & 0x0F, LEFT_IDENTITIES)
    right = subset_for_mask((mask >> 4) & 0x0F, RIGHT_IDENTITIES)
    key = (
        f'L:{"+".join(left) if left else "-"}|'
        f'R:{"+".join(right) if right else "-"}'
    )
    eligible = left + right
    return {
        'configuration_id': f'presence_{mask:03d}',
        'bitmask_decimal': mask,
        'bitmask_hex': f'0x{mask:02X}',
        'bit_order': list(GLOBAL_IDENTITIES),
        'canonical_subset_key': key,
        'left_active_identities': left,
        'right_active_identities': right,
        'left_count': len(left),
        'right_count': len(right),
        'total_count': len(eligible),
        'target_eligible_identities': eligible,
        'relation_eligibility': {
            identity: relation_families(
                len(left) if identity.startswith('L') else len(right)
            )
            for identity in eligible
        },
    }


def presence_inventory() -> list[dict[str, Any]]:
    return [configuration_record(mask) for mask in range(1, 256)]


def is_prefix_subset(subset: list[str], side: str) -> bool:
    allowed = SIDE_IDENTITIES[side]
    return tuple(subset) == allowed[:len(subset)]


def inventory_audit(records: list[dict[str, Any]]) -> dict[str, Any]:
    ids = [record['configuration_id'] for record in records]
    keys = [record['canonical_subset_key'] for record in records]
    count_pairs = {
        (record['left_count'], record['right_count'])
        for record in records
    }
    expected_pairs = {
        (left, right)
        for left in range(5)
        for right in range(5)
        if left or right
    }
    identity_presence = {
        identity: sum(
            identity in record['target_eligible_identities']
            for record in records
        )
        for identity in GLOBAL_IDENTITIES
    }
    examples = {
        'L3 only': (['L3'], []),
        'L2+L4 only': (['L2', 'L4'], []),
        'R4 only': ([], ['R4']),
        'R1+R3 only': ([], ['R1', 'R3']),
        'L1+L4 with R2': (['L1', 'L4'], ['R2']),
        'L2 with R1+R4': (['L2'], ['R1', 'R4']),
        'L2+L4 with R2+R3': (['L2', 'L4'], ['R2', 'R3']),
        'all eight identities': (list(LEFT_IDENTITIES), list(RIGHT_IDENTITIES)),
    }
    example_matches = {
        name: next(
            (
                record['configuration_id']
                for record in records
                if record['left_active_identities'] == left
                and record['right_active_identities'] == right
            ),
            None,
        )
        for name, (left, right) in examples.items()
    }
    non_prefix_counts = {
        side: sum(
            bool(record[f'{side}_active_identities'])
            and not is_prefix_subset(
                record[f'{side}_active_identities'],
                side,
            )
            for record in records
        )
        for side in ('left', 'right')
    }
    checks = {
        'exactly_255_unique_configurations': (
            len(records) == len(set(ids)) == len(set(keys)) == 255
        ),
        'all_empty_absent': all(
            record['total_count'] > 0 for record in records
        ),
        'all_24_nonempty_count_pairs': count_pairs == expected_pairs,
        'total_counts_1_through_8': {
            record['total_count'] for record in records
        } == set(range(1, 9)),
        'each_identity_present_and_absent': all(
            0 < count < len(records)
            for count in identity_presence.values()
        ),
        'all_required_examples': all(example_matches.values()),
        'non_prefix_subsets_present_on_both_rails': all(
            count > 0 for count in non_prefix_counts.values()
        ),
    }
    return {
        'schema_version': INVENTORY_SCHEMA,
        'configuration_count': len(records),
        'valid_global_configuration_formula': '16 * 16 - 1 = 255',
        'fixed_identity_order': list(GLOBAL_IDENTITIES),
        'count_pair_count': len(count_pairs),
        'total_count_distribution': dict(sorted(Counter(
            record['total_count'] for record in records
        ).items())),
        'identity_presence_counts': identity_presence,
        'identity_absence_counts': {
            identity: len(records) - count
            for identity, count in identity_presence.items()
        },
        'non_prefix_configuration_counts': non_prefix_counts,
        'required_examples': example_matches,
        'checks': checks,
        'passed': all(checks.values()),
    }


def inventory_markdown(
    records: list[dict[str, Any]],
    audit: dict[str, Any],
) -> str:
    lines = [
        '# Room 315 exact presence-configuration inventory',
        '',
        f'- Valid configurations: {len(records)}',
        '- Excluded configuration: both rails empty',
        '- Bit order: `L1,L2,L3,L4,R1,R2,R3,R4`',
        f'- Audit: **{"PASS" if audit["passed"] else "FAIL"}**',
        '',
        '| ID | Mask | Left subset | Right subset | Counts | Eligible families |',
        '|---|---:|---|---|---:|---|',
    ]
    for record in records:
        families = sorted({
            family
            for values in record['relation_eligibility'].values()
            for family in values
        })
        lines.append(
            f'| {record["configuration_id"]} | '
            f'{record["bitmask_hex"]} | '
            f'{", ".join(record["left_active_identities"]) or "empty"} | '
            f'{", ".join(record["right_active_identities"]) or "empty"} | '
            f'{record["left_count"]}+{record["right_count"]} | '
            f'{", ".join(families)} |'
        )
    return '\n'.join(lines) + '\n'


def _configuration_lookup(
    records: list[dict[str, Any]],
) -> dict[tuple[tuple[str, ...], tuple[str, ...]], dict[str, Any]]:
    return {
        (
            tuple(record['left_active_identities']),
            tuple(record['right_active_identities']),
        ): record
        for record in records
    }


def select_smoke_configurations(
    records: list[dict[str, Any]],
    *,
    count: int = SMOKE_SCENARIO_COUNT,
    seed: int = SEED,
) -> list[dict[str, Any]]:
    if count < 64 or count > 96:
        raise ArbitrarySubsetError('smoke count must be in 64..96')
    by_key = _configuration_lookup(records)
    selected: dict[str, dict[str, Any]] = {}

    def add(left: list[str], right: list[str]) -> None:
        record = by_key.get((tuple(left), tuple(right)))
        if record is None:
            raise ArbitrarySubsetError(
                f'requested smoke subset is not in inventory: {left}, {right}'
            )
        selected[record['configuration_id']] = record

    for identity in GLOBAL_IDENTITIES:
        add([identity] if identity.startswith('L') else [],
            [identity] if identity.startswith('R') else [])
    for pair in itertools.combinations(LEFT_IDENTITIES, 2):
        add(list(pair), [])
    for pair in itertools.combinations(RIGHT_IDENTITIES, 2):
        add([], list(pair))
    for left_count in range(5):
        for right_count in range(5):
            if not left_count and not right_count:
                continue
            candidates = [
                record
                for record in records
                if record['left_count'] == left_count
                and record['right_count'] == right_count
            ]
            candidates.sort(key=lambda record: (
                is_prefix_subset(
                    record['left_active_identities'],
                    'left',
                ) + is_prefix_subset(
                    record['right_active_identities'],
                    'right',
                ),
                stable_int(seed, 'count_pair', record['configuration_id']),
            ))
            selected[candidates[0]['configuration_id']] = candidates[0]
    required_examples = (
        (['L3'], []),
        (['L2', 'L4'], []),
        ([], ['R3']),
        ([], ['R2', 'R4']),
        (['L1', 'L4'], ['R2']),
        (['L2'], ['R1', 'R4']),
        (['L2', 'L4'], ['R2', 'R3']),
        (list(LEFT_IDENTITIES), list(RIGHT_IDENTITIES)),
    )
    for left, right in required_examples:
        add(left, right)

    remaining = [
        record
        for record in records
        if record['configuration_id'] not in selected
    ]
    remaining.sort(key=lambda record: stable_int(
        seed,
        'smoke_fill',
        record['configuration_id'],
    ))
    for record in remaining:
        if len(selected) >= count:
            break
        selected[record['configuration_id']] = record
    if len(selected) != count:
        raise ArbitrarySubsetError(
            f'could not select exactly {count} smoke configurations'
        )
    ordered = list(selected.values())
    ordered.sort(key=lambda record: (
        stable_int(seed, 'smoke_order', record['configuration_id']),
        record['configuration_id'],
    ))
    return ordered


def _side_for_target(
    record: dict[str, Any],
    *,
    ordinal: int,
    target_counts: Counter,
) -> str:
    candidates = [
        side
        for side in ('left', 'right')
        if record[f'{side}_active_identities']
    ]
    return min(
        candidates,
        key=lambda side: (
            min(
                target_counts[identity]
                for identity in record[f'{side}_active_identities']
            ),
            (ordinal + (0 if side == 'left' else 1)) % 2,
            side,
        ),
    )


def _select_target(
    identities: list[str],
    target_counts: Counter,
    *,
    seed: int,
    ordinal: int,
) -> str:
    return min(
        identities,
        key=lambda identity: (
            target_counts[identity],
            stable_int(seed, ordinal, 'target', identity),
        ),
    )


def _select_family(
    target_rail_count: int,
    family_counts: Counter,
    *,
    seed: int,
    ordinal: int,
) -> str:
    eligible = relation_families(target_rail_count)
    return min(
        eligible,
        key=lambda family: (
            family_counts[family],
            stable_int(seed, ordinal, 'family', family),
        ),
    )


def _select_relation_identities(
    active: list[str],
    target: str,
    family: str,
    role_counts: Counter,
) -> tuple[str, ...]:
    if family == NO_RELATION:
        return ()
    needed = 2 if family == 'multi_blocker' else 1
    role = (
        'non_blocker'
        if family in {
            'nonblocker_behind_same_segment',
            'nonblocker_adjacent_branch',
        }
        else 'blocker'
    )
    candidates = sorted(
        (identity for identity in active if identity != target),
        key=lambda identity: (role_counts[(identity, role)], identity),
    )
    if len(candidates) < needed:
        raise ArbitrarySubsetError(
            f'{family} needs {needed} relation identities'
        )
    return tuple(candidates[:needed])


def _unused_full_identity(
    side: str,
    excluded: set[str],
) -> str:
    return next(
        identity
        for identity in SIDE_IDENTITIES[side]
        if identity not in excluded
    )


def _payload_state(
    identity: str,
    occurrence_counts: Counter,
) -> str:
    return (
        'loaded'
        if occurrence_counts[identity] % 2 == 0
        else 'empty'
    )


def build_arbitrary_scenario(
    record: dict[str, Any],
    *,
    ordinal: int,
    seed: int,
    target_counts: Counter,
    family_counts: Counter,
    role_counts: Counter,
    occurrence_counts: Counter,
    covered_segments: dict[str, set[str]],
) -> dict[str, Any]:
    target_side = _side_for_target(
        record,
        ordinal=ordinal,
        target_counts=target_counts,
    )
    active_target = record[f'{target_side}_active_identities']
    target = _select_target(
        active_target,
        target_counts,
        seed=seed,
        ordinal=ordinal,
    )
    family = _select_family(
        len(active_target),
        family_counts,
        seed=seed,
        ordinal=ordinal,
    )
    relations = _select_relation_identities(
        active_target,
        target,
        family,
        role_counts,
    )
    source_family = (
        family
        if family != NO_RELATION
        else 'blocker_ahead_same_segment'
    )
    source_relations = relations
    if not source_relations:
        source_relations = (
            _unused_full_identity(target_side, {target}),
        )
    other_side = 'right' if target_side == 'left' else 'left'
    source_scope = (
        'dual_four_plus_four'
        if record[f'{other_side}_active_identities']
        else f'{target_side}_four'
    )
    last_error: Exception | None = None
    for attempt in range(512):
        try:
            source = _build_blocker_scenario(
                source_family,
                ordinal=ordinal,
                type_index=ordinal - 1 + attempt * SMOKE_SCENARIO_COUNT,
                seed=seed + ordinal * 37 + attempt,
                dataset_seed=seed,
                capture={
                    'frames_per_scenario': 1,
                    'settle_seconds': 1.5,
                    'frame_interval_seconds': 0.25,
                },
                rail_scope=source_scope,
                active_side=target_side,
                target_identity=target,
                relation_identities=source_relations,
                rail_presence_index={
                    'left': ordinal - 1,
                    'right': ordinal - 1,
                },
                covered_segments_by_side={
                    side: set(covered_segments[side])
                    for side in SIDES
                },
            )
            break
        except (ValueError, KeyError) as exc:
            last_error = exc
    else:
        raise ArbitrarySubsetError(
            f'could not place {record["configuration_id"]}: {last_error}'
        )

    requested = {
        'left': record['left_active_identities'],
        'right': record['right_active_identities'],
    }
    rails = source['scene']['rails']
    for side in ('left', 'right'):
        by_id = {
            shuttle['id']: shuttle
            for shuttle in rails[side]['shuttles']
        }
        missing = sorted(set(requested[side]) - set(by_id))
        if missing:
            raise ArbitrarySubsetError(
                f'source placement dropped requested identities: {missing}'
            )
        filtered = []
        for identity in requested[side]:
            shuttle = dict(by_id[identity])
            shuttle['loaded_state'] = _payload_state(
                identity,
                occurrence_counts,
            )
            filtered.append(shuttle)
            occurrence_counts[identity] += 1
        rails[side]['shuttles'] = filtered
        covered_segments[side].update(
            shuttle['start_position']['segment']
            for shuttle in filtered
        )

    scene_type = family if family != NO_RELATION else 'single'
    probe_relations = (
        source['relation_probe']['relations']
        if family != NO_RELATION
        else []
    )
    neutral = [
        identity
        for identity in active_target
        if identity != target and identity not in relations
    ]
    relation_probe = {
        'target_shuttle_id': target,
        'side': target_side,
        'relations': probe_relations,
        'relation_neutral_shuttle_ids': neutral,
        'opposite_rail_neutral_shuttle_ids': list(requested[other_side]),
        'model_input_exposure': 'excluded',
    }
    launch_arguments = _launch_arguments(rails)
    launch_arguments.update({
        'room315_identity_selection_mode': 'explicit',
        'room315_left_active_identities': ','.join(requested['left']),
        'room315_right_active_identities': ','.join(requested['right']),
    })
    family_payload = {
        **_family_payload(scene_type, source['scene']),
        'configuration_id': record['configuration_id'],
        'relation_family': family,
        'target': target,
    }
    family_hash = _hash(family_payload)
    scenario = {
        'schema_version': SCHEMA_VERSION,
        'manifest_profile_schema': SMOKE_SCHEMA,
        'presence_profile': ARBITRARY_IDENTITY_PRESENCE_PROFILE,
        'presence_configuration_id': record['configuration_id'],
        'presence_bitmask': record['bitmask_decimal'],
        'left_active_identities': list(requested['left']),
        'right_active_identities': list(requested['right']),
        'scenario_id': (
            f'arbitrary_{ordinal:04d}_{family}_{family_hash[:8]}'
        ),
        'scenario_family': f'arbitrary_family_{family_hash}',
        'scene_type': scene_type,
        'relation_family': family,
        'seed': seed + ordinal * 37,
        'scene': source['scene'],
        'capture': source['capture'],
        'setup': {
            **source['setup'],
            'launch_arguments': launch_arguments,
        },
        'relation_probe': relation_probe,
        'rail_scope': ARBITRARY_IDENTITY_PRESENCE_PROFILE,
        'expected_label_coverage': {
            'shuttle_count': record['total_count'],
            'fixed_schema_identity_count': len(GLOBAL_IDENTITIES),
            'loaded_count': sum(
                shuttle['loaded_state'] == 'loaded'
                for side in ('left', 'right')
                for shuttle in rails[side]['shuttles']
            ),
            'empty_count': sum(
                shuttle['loaded_state'] == 'empty'
                for side in ('left', 'right')
                for shuttle in rails[side]['shuttles']
            ),
            'switch_count': len(SWITCH_NAMES) * len(SIDES),
            'obstacle_count': 0,
            'continuous_position_count': record['total_count'],
        },
    }
    validate_scenario(scenario)
    if scenario_physical_conflicts(scenario):
        raise ArbitrarySubsetError(
            f'{scenario["scenario_id"]} has physical conflicts'
        )
    target_counts[target] += 1
    family_counts[family] += 1
    role_counts[(target, 'target')] += 1
    for relation in probe_relations:
        relation_identity = relation['other_shuttle_id']
        role = (
            'non_blocker'
            if 'non_blocker' in relation['relation']
            else 'blocker'
        )
        role_counts[(relation_identity, role)] += 1
    for identity in neutral + list(requested[other_side]):
        role_counts[(identity, 'relation_neutral')] += 1
    return scenario


def generate_smoke_manifest(
    records: list[dict[str, Any]],
    *,
    seed: int = SEED,
    count: int = SMOKE_SCENARIO_COUNT,
) -> list[dict[str, Any]]:
    selected = select_smoke_configurations(
        records,
        count=count,
        seed=seed,
    )
    target_counts: Counter = Counter()
    family_counts: Counter = Counter()
    role_counts: Counter = Counter()
    occurrence_counts: Counter = Counter()
    covered_segments = {side: set() for side in SIDES}
    scenarios = []
    for ordinal, record in enumerate(selected, start=1):
        scenarios.append(build_arbitrary_scenario(
            record,
            ordinal=ordinal,
            seed=seed,
            target_counts=target_counts,
            family_counts=family_counts,
            role_counts=role_counts,
            occurrence_counts=occurrence_counts,
            covered_segments=covered_segments,
        ))
    return scenarios


def _manifest_identity_sets(
    scenario: dict[str, Any],
) -> dict[str, list[str]]:
    return {
        side: [
            shuttle['id']
            for shuttle in scenario['scene']['rails'][side]['shuttles']
        ]
        for side in ('left', 'right')
    }


def static_smoke_audit(
    scenarios: list[dict[str, Any]],
    records: list[dict[str, Any]],
) -> dict[str, Any]:
    by_id = {
        record['configuration_id']: record
        for record in records
    }
    errors = []
    config_ids = []
    count_pairs = Counter()
    total_counts = Counter()
    subset_sizes = {'left': Counter(), 'right': Counter()}
    presence = Counter()
    loaded = Counter()
    empty = Counter()
    targets = Counter()
    roles = Counter()
    relations = Counter()
    zones = Counter()
    segments = Counter()
    pairwise = {
        first: {second: 0 for second in GLOBAL_IDENTITIES}
        for first in GLOBAL_IDENTITIES
    }
    alone = Counter()
    physical_violations = []
    topology_violations = []
    relation_violations = []
    prefix_substitutions = []
    for scenario in scenarios:
        try:
            validate_scenario(scenario)
        except ValueError as exc:
            topology_violations.append({
                'scenario_id': scenario.get('scenario_id'),
                'error': str(exc),
            })
            continue
        physical_violations.extend(
            {'scenario_id': scenario['scenario_id'], **conflict}
            for conflict in scenario_physical_conflicts(scenario)
        )
        config_id = scenario['presence_configuration_id']
        config_ids.append(config_id)
        requested = by_id.get(config_id)
        if requested is None:
            errors.append(f'unknown configuration: {config_id}')
            continue
        actual = _manifest_identity_sets(scenario)
        for side in ('left', 'right'):
            expected = requested[f'{side}_active_identities']
            if actual[side] != expected:
                prefix_substitutions.append({
                    'scenario_id': scenario['scenario_id'],
                    'side': side,
                    'requested': expected,
                    'actual': actual[side],
                })
        count_pairs[
            f'{requested["left_count"]}+{requested["right_count"]}'
        ] += 1
        total_counts[str(requested['total_count'])] += 1
        subset_sizes['left'][str(requested['left_count'])] += 1
        subset_sizes['right'][str(requested['right_count'])] += 1
        active = actual['left'] + actual['right']
        if len(active) == 1:
            alone[active[0]] += 1
        for identity in active:
            presence[identity] += 1
        for first in active:
            for second in active:
                pairwise[first][second] += 1
        for side in ('left', 'right'):
            for shuttle in scenario['scene']['rails'][side]['shuttles']:
                if shuttle['loaded_state'] == 'loaded':
                    loaded[shuttle['id']] += 1
                else:
                    empty[shuttle['id']] += 1
                position = shuttle['start_position']
                zones[position['position_zone']] += 1
                segments[f'{side}:{position["segment"]}'] += 1
        probe = scenario['relation_probe']
        target = probe['target_shuttle_id']
        targets[target] += 1
        if target not in active:
            relation_violations.append({
                'scenario_id': scenario['scenario_id'],
                'error': 'target_not_active',
            })
        family = scenario['relation_family']
        relations[family] += 1
        eligible = requested['relation_eligibility'][target]
        if family not in eligible:
            relation_violations.append({
                'scenario_id': scenario['scenario_id'],
                'error': f'ineligible_relation:{family}',
            })
        roles[f'{target}:target'] += 1
        for relation in probe['relations']:
            role = (
                'non_blocker'
                if 'non_blocker' in relation['relation']
                else 'blocker'
            )
            roles[f'{relation["other_shuttle_id"]}:{role}'] += 1
        for identity in (
            probe['relation_neutral_shuttle_ids']
            + probe['opposite_rail_neutral_shuttle_ids']
        ):
            roles[f'{identity}:relation_neutral'] += 1

    expected_count_pairs = {
        f'{left}+{right}'
        for left in range(5)
        for right in range(5)
        if left or right
    }
    required_same_rail_nonprefix = []
    for side, identities in SIDE_IDENTITIES.items():
        for pair in itertools.combinations(identities, 2):
            if tuple(pair) == identities[:2]:
                continue
            found = any(
                scenario[f'{side}_active_identities'] == list(pair)
                and not scenario[
                    f'{"right" if side == "left" else "left"}_active_identities'
                ]
                for scenario in scenarios
            )
            required_same_rail_nonprefix.append({
                'side': side,
                'subset': list(pair),
                'present': found,
            })
    vectorizer = VisualStateLabelVectorizer()
    vectorizer_metadata = vectorizer.to_json()
    checks = {
        'scenario_count_in_64_to_96': 64 <= len(scenarios) <= 96,
        'unique_configuration_per_smoke_scenario': (
            len(config_ids) == len(set(config_ids)) == len(scenarios)
        ),
        'all_empty_absent': all(
            scenario['left_active_identities']
            or scenario['right_active_identities']
            for scenario in scenarios
        ),
        'no_prefix_substitution': not prefix_substitutions,
        'every_identity_alone': all(alone[identity] >= 1 for identity in GLOBAL_IDENTITIES),
        'every_nonprefix_same_rail_pair_alone': all(
            item['present'] for item in required_same_rail_nonprefix
        ),
        'all_24_count_pairs': set(count_pairs) == expected_count_pairs,
        'total_counts_1_through_8': set(total_counts) == {
            str(value) for value in range(1, 9)
        },
        'both_payload_states_every_identity': all(
            loaded[identity] > 0 and empty[identity] > 0
            for identity in GLOBAL_IDENTITIES
        ),
        'every_identity_targeted': all(
            targets[identity] > 0 for identity in GLOBAL_IDENTITIES
        ),
        'all_relation_families_and_no_relation': (
            set(relations) == {NO_RELATION, *ALL_RELATION_FAMILIES}
        ),
        'all_six_target_zones': set(zones) >= set(POSITION_ZONES),
        'all_14_segments_both_rails': all(
            segments[f'{side}:{segment}'] > 0
            for side in ('left', 'right')
            for segment in valid_public_segments(side)
        ),
        'zero_physical_separation_violations': not physical_violations,
        'zero_topology_violations': not topology_violations,
        'zero_relation_violations': not relation_violations,
        'fixed_schema_v3': VISUAL_STATE_SCHEMA_VERSION == 'room315.visual_state.v3',
        'fixed_vectorizer_dimension_200': vectorizer.dim == 200,
        'capacity_not_inferred_from_data': (
            vectorizer_metadata['capacity_inferred_from_dataset'] is False
        ),
    }
    return {
        'schema_version': AUDIT_SCHEMA,
        'passed': all(checks.values()) and not errors,
        'scenario_count': len(scenarios),
        'checks': checks,
        'errors': errors,
        'distributions': {
            'configuration_ids': sorted(config_ids),
            'left_subset_size': dict(sorted(subset_sizes['left'].items())),
            'right_subset_size': dict(sorted(subset_sizes['right'].items())),
            'cardinality_pair': dict(sorted(count_pairs.items())),
            'total_active_count': dict(sorted(total_counts.items())),
            'identity_presence': dict(sorted(presence.items())),
            'identity_absence': {
                identity: len(scenarios) - presence[identity]
                for identity in GLOBAL_IDENTITIES
            },
            'identity_alone': dict(sorted(alone.items())),
            'identity_loaded': dict(sorted(loaded.items())),
            'identity_empty': dict(sorted(empty.items())),
            'identity_target': dict(sorted(targets.items())),
            'identity_roles': dict(sorted(roles.items())),
            'relation_family': dict(sorted(relations.items())),
            'position_zone': dict(sorted(zones.items())),
            'segment': dict(sorted(segments.items())),
            'pairwise_identity_cooccurrence': pairwise,
        },
        'required_nonprefix_same_rail_pairs': required_same_rail_nonprefix,
        'violations': {
            'prefix_substitutions': prefix_substitutions,
            'physical_separation': physical_violations,
            'topology': topology_violations,
            'relation': relation_violations,
            'mask': [],
            'dropped_identities': [],
            'unexpected_identities': [],
            'duplicate_identities': [],
            'unrepresentable_targets': [],
        },
        'fixed_schema': {
            'visual_state_schema': VISUAL_STATE_SCHEMA_VERSION,
            'identity_order': list(GLOBAL_IDENTITIES),
            'vectorizer_dimension': vectorizer.dim,
            'capacity_source': vectorizer_metadata['capacity_source'],
            'dataset_inferred_capacity': vectorizer_metadata[
                'capacity_inferred_from_dataset'
            ],
        },
    }


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    for line_number, line in enumerate(
        path.read_text(encoding='utf-8').splitlines(),
        start=1,
    ):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ArbitrarySubsetError(
                f'{path}:{line_number}: invalid JSONL'
            ) from exc
        if not isinstance(row, dict):
            raise ArbitrarySubsetError(
                f'{path}:{line_number}: row is not an object'
            )
        rows.append(row)
    return rows


def _identity_config() -> dict[str, dict[str, Any]]:
    path = (
        REPO_ROOT
        / 'mfja_robot_control_config'
        / 'config'
        / 'room_315_shuttle_identity'
        / 'shuttle_identity.yaml'
    )
    loaded = yaml.safe_load(path.read_text(encoding='utf-8'))
    return {
        entry['label_text']: entry
        for entry in loaded['shuttles']
    }


def color_identity_audit(
    scenarios: list[dict[str, Any]],
) -> dict[str, Any]:
    config = _identity_config()
    identities = {}
    for identity in GLOBAL_IDENTITIES:
        entry = config[identity]
        sdf_path = (
            REPO_ROOT
            / 'mfja_3rd_floor_description'
            / 'models'
            / f'room315_shuttle_{identity}'
            / 'model.sdf'
        )
        root = ET.parse(sdf_path).getroot()
        regions = []
        for visual in root.iter('visual'):
            if not str(visual.get('name') or '').startswith('identity_region_'):
                continue
            size = visual.findtext('./geometry/plane/size', default='')
            pose = visual.findtext('./pose', default='')
            regions.append({
                'name': visual.get('name'),
                'plane_size_m': [
                    float(value) for value in size.split()
                ],
                'pose': [float(value) for value in pose.split()],
            })
        identities[identity] = {
            'color_name': entry['color_name'],
            'tag_ids': entry['tag_ids'],
            'marker_roles': entry['marker_roles'],
            'sdf_path': str(sdf_path),
            'sdf_sha256': sha256(sdf_path),
            'identity_region_count': len(regions),
            'identity_regions': regions,
            'top_facing_regions': all(
                len(region['pose']) >= 3 and region['pose'][2] > 0.0
                for region in regions
            ),
        }
    presence_modes = {
        identity: {
            'alone': 0,
            'same_rail_pairs': set(),
            'sparse_dual': 0,
            'dense': 0,
            'loaded': 0,
            'empty': 0,
            'zones': set(),
            'partial_occlusion_candidate': 0,
        }
        for identity in GLOBAL_IDENTITIES
    }
    for scenario in scenarios:
        active = (
            scenario['left_active_identities']
            + scenario['right_active_identities']
        )
        for side in ('left', 'right'):
            side_active = scenario[f'{side}_active_identities']
            for identity in side_active:
                mode = presence_modes[identity]
                if len(active) == 1:
                    mode['alone'] += 1
                for other in side_active:
                    if other != identity:
                        mode['same_rail_pairs'].add(other)
                if (
                    scenario['left_active_identities']
                    and scenario['right_active_identities']
                    and len(active) <= 4
                ):
                    mode['sparse_dual'] += 1
                if len(active) >= 7:
                    mode['dense'] += 1
                shuttle = next(
                    item
                    for item in scenario['scene']['rails'][side]['shuttles']
                    if item['id'] == identity
                )
                mode[shuttle['loaded_state']] += 1
                mode['zones'].add(
                    shuttle['start_position']['position_zone']
                )
                if scenario['relation_family'] in {
                    'blocker_ahead_same_segment',
                    'nonblocker_behind_same_segment',
                    'multi_blocker',
                }:
                    mode['partial_occlusion_candidate'] += 1
    rendered_modes = {
        identity: {
            **mode,
            'same_rail_pairs': sorted(mode['same_rail_pairs']),
            'zones': sorted(mode['zones']),
        }
        for identity, mode in presence_modes.items()
    }
    checks = {
        'unique_configured_colors': len({
            entry['color_name'] for entry in identities.values()
        }) == 8,
        'four_regions_per_identity': all(
            entry['identity_region_count'] == 4
            for entry in identities.values()
        ),
        'all_regions_top_facing_for_overhead_cameras': all(
            entry['top_facing_regions']
            for entry in identities.values()
        ),
        'every_identity_alone': all(
            mode['alone'] > 0 for mode in presence_modes.values()
        ),
        'every_same_rail_pair': all(
            len(mode['same_rail_pairs']) == 3
            for mode in presence_modes.values()
        ),
        'every_identity_sparse_dual_and_dense': all(
            mode['sparse_dual'] > 0 and mode['dense'] > 0
            for mode in presence_modes.values()
        ),
        'every_identity_loaded_and_empty': all(
            mode['loaded'] > 0 and mode['empty'] > 0
            for mode in presence_modes.values()
        ),
        'every_identity_boundary_switch_slot_and_ordinary': all(
            {'boundary', 'switch', 'slot', 'ordinary'} <= mode['zones']
            for mode in presence_modes.values()
        ),
        'partial_occlusion_candidates_planned': all(
            mode['partial_occlusion_candidate'] > 0
            for mode in presence_modes.values()
        ),
    }
    return {
        'schema_version': 'room315.color_identity_static_audit.v1',
        'passed': all(checks.values()),
        'checks': checks,
        'identities': identities,
        'smoke_coverage': rendered_modes,
        'camera_visibility_semantics': {
            'required_cameras': list(REQUIRED_CAMERAS),
            'static_evidence': (
                'Four top-facing perimeter identity planes are configured on '
                'every shuttle; both Room 315 cameras are overhead.'
            ),
            'pixel_visibility_status': (
                'pending Gazebo smoke capture and manual gallery review'
            ),
            'bbox_and_color_region_pixel_sizes': (
                'pending capture; SDF identity plane size is audited statically'
            ),
            'lighting_consistency': (
                'pending capture; use the paired-camera smoke gallery'
            ),
            'visually_similar_pairs': [
                {
                    'pair': ['R4', 'L3'],
                    'risk': 'yellow versus orange under warm/low-saturation lighting',
                },
                {
                    'pair': ['L4', 'unmarked highlights'],
                    'risk': 'white identity regions can blend with bright backgrounds',
                },
                {
                    'pair': ['L1', 'R2'],
                    'risk': 'cyan versus blue under reduced color fidelity',
                },
            ],
        },
    }


def _atomic_package_root(path: Path) -> tuple[Path, Path]:
    resolved = path.expanduser().resolve()
    if resolved.exists():
        raise FileExistsError(f'refusing to overwrite package: {resolved}')
    temporary = Path(tempfile.mkdtemp(
        prefix=f'.{resolved.name}.',
        dir=resolved.parent,
    ))
    return resolved, temporary


def _make_executable(path: Path) -> None:
    path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP)


def _smoke_scripts(package_root: Path) -> dict[str, str]:
    tool = SCRIPT_DIR / 'room_315_arbitrary_subset_visual.py'
    runner = SCRIPT_DIR / 'room_315_visual_scenario_runner.py'
    common = (
        '#!/usr/bin/env bash\n'
        'set -euo pipefail\n\n'
        f'PACKAGE_ROOT={package_root}\n'
        f'TOOL={tool}\n'
    )
    generate = common + (
        'TMP=$(mktemp -d "${PACKAGE_ROOT}/.regenerate.XXXXXX")\n'
        'trap \'rm -rf -- "${TMP}"\' EXIT\n'
        'python3 "${TOOL}" prepare-smoke --output "${TMP}/package" '
        f'--seed {SEED} --count {SMOKE_SCENARIO_COUNT}\n'
        'cmp "${TMP}/package/scenario_manifest.jsonl" '
        '"${PACKAGE_ROOT}/scenario_manifest.jsonl"\n'
        'echo DETERMINISTIC_MANIFEST_VALID\n'
    )
    audit = common + (
        'python3 "${TOOL}" audit-smoke '
        '--manifest "${PACKAGE_ROOT}/scenario_manifest.jsonl" '
        '--report "${PACKAGE_ROOT}/static_smoke_audit.json"\n'
        'jq -e \'.passed == true\' '
        '"${PACKAGE_ROOT}/static_smoke_audit.json" >/dev/null\n'
        'echo STATIC_SMOKE_AUDIT_PASS\n'
    )
    capture = common + (
        '"${PACKAGE_ROOT}/validate_smoke_approval.py" --require capture\n'
        'jq -e \'.passed == true\' '
        '"${PACKAGE_ROOT}/static_smoke_audit.json" >/dev/null\n'
        'set +u\nsource /opt/ros/jazzy/setup.bash\n'
        'source /home/tiago/mfja_3rd_floor_ros2_ws/install/setup.bash\nset -u\n'
        f'python3 {runner} '
        '--scenario-manifest "${PACKAGE_ROOT}/scenario_manifest.jsonl" '
        '--output-dataset "${PACKAGE_ROOT}/dataset" '
        '--readiness-timeout-seconds 60 --capture-timeout-seconds 45 '
        '--keep-going\n'
        'python3 "${PACKAGE_ROOT}/capture_status.py"\n'
    )
    resume = capture.replace(
        '--keep-going\n',
        '--resume --keep-going\n',
    )
    audit_captured = common + (
        '"${PACKAGE_ROOT}/validate_smoke_approval.py" --require capture\n'
        'python3 "${PACKAGE_ROOT}/capture_status.py" --require-complete '
        '> "${PACKAGE_ROOT}/captured_smoke_audit.json"\n'
        'echo CAPTURED_SMOKE_STRUCTURE_PASS\n'
    )
    return {
        'generate_smoke_manifest.sh': generate,
        'audit_smoke_manifest.sh': audit,
        'capture_smoke.sh': capture,
        'resume_smoke.sh': resume,
        'audit_captured_smoke.sh': audit_captured,
    }


def _capture_status_source(package_root: Path) -> str:
    return f'''#!/usr/bin/env python3
import argparse
import json
import sys
from pathlib import Path
from PIL import Image

ROOT = Path({str(package_root)!r})
EXPECTED = {SMOKE_SCENARIO_COUNT}
SCRIPTS = Path({str(SCRIPT_DIR)!r})
sys.path.insert(0, str(SCRIPTS))
from room_315_visual_state_dataset import normalize_visual_state_labels

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--require-complete', action='store_true')
    args = parser.parse_args()
    manifest = [
        json.loads(line)
        for line in (ROOT / 'scenario_manifest.jsonl').read_text().splitlines()
        if line.strip()
    ]
    episodes = ROOT / 'dataset' / 'episodes'
    completed = []
    failures = {{}}
    if episodes.is_dir():
        for row in manifest:
            scenario_id = row['scenario_id']
            episode = episodes / scenario_id
            errors = []
            try:
                event = json.loads(
                    (episode / 'event.json').read_text(encoding='utf-8')
                )
                labels = normalize_visual_state_labels(event)
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                errors.append(f'event_or_labels_invalid:{{exc}}')
                labels = None
            try:
                validation = json.loads(
                    (episode / 'validation.json').read_text(encoding='utf-8')
                )
                if (
                    validation.get('validation_status') != 'approved'
                    or validation.get('capture_complete') is not True
                    or validation.get('labels_valid') is not True
                ):
                    errors.append('validation_not_approved_complete_and_valid')
            except (OSError, json.JSONDecodeError):
                errors.append('validation_invalid')
            expected = set(
                row['left_active_identities']
                + row['right_active_identities']
            )
            if labels is not None:
                present = {{
                    shuttle['id']
                    for shuttle in labels['shuttles']
                    if shuttle['presence']
                }}
                visible = {{
                    shuttle['id']
                    for shuttle in labels['shuttles']
                    if shuttle['presence'] and shuttle['visually_available']
                }}
                if present != expected:
                    errors.append(
                        f'exact_presence_mismatch:expected={{sorted(expected)}}:'
                        f'actual={{sorted(present)}}'
                    )
                if visible != expected:
                    errors.append(
                        f'exact_visibility_mismatch:expected={{sorted(expected)}}:'
                        f'actual={{sorted(visible)}}'
                    )
            for camera in ('left_rail_rgb', 'right_rail_rgb'):
                image_path = (
                    episode / 'images' / camera / 'frame_000000.jpg'
                )
                try:
                    with Image.open(image_path) as image:
                        image.verify()
                except OSError:
                    errors.append(f'{{camera}}_image_invalid')
            if errors:
                failures[scenario_id] = errors
            else:
                completed.append(scenario_id)
    status = {{
        'schema_version': 'room315.arbitrary_subset_capture_status.v1',
        'expected_scenarios': EXPECTED,
        'completed_scenarios': len(completed),
        'capture_complete': len(completed) == EXPECTED,
        'exact_subset_validation': True,
        'failure_count': len(failures),
        'failures': failures,
    }}
    (ROOT / 'capture_state.json').write_text(
        json.dumps({{
            'capture_has_started': bool(completed or failures),
            'capture_complete': status['capture_complete'],
            'captured_scenario_count': len(completed),
            'expected_scenario_count': EXPECTED,
            'failed_scenario_count': len(failures),
        }}, indent=2, sort_keys=True) + '\\n',
        encoding='utf-8',
    )
    print(json.dumps(status, indent=2, sort_keys=True))
    if args.require_complete and not status['capture_complete']:
        return 2
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
'''


def _gallery_source(package_root: Path) -> str:
    return f'''#!/usr/bin/env python3
import subprocess
import sys
from pathlib import Path

ROOT = Path({str(package_root)!r})
if not (ROOT / 'dataset' / 'episodes').is_dir():
    raise SystemExit('capture is absent; gallery was not created')
tool = Path({str(SCRIPT_DIR / 'room_315_visual_manual_review.py')!r})
result = subprocess.run(
    [sys.executable, str(tool), 'gallery', '--package-root', str(ROOT)],
    check=False,
)
if result.returncode:
    raise SystemExit(result.returncode)
print('Gallery created. Approval remains false until explicit human review.')
print(
    'After review, set approved_after_gallery_review=true in '
    'smoke_manual_approval.json and run '
    './validate_smoke_approval.py --require gallery.'
)
'''


def _approval_validator_source(package_root: Path) -> str:
    source = '''#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

ROOT = Path(__PACKAGE_ROOT__)

def jsonl_ids(path, field):
    if not path.is_file():
        raise SystemExit(f'MISSING_JSONL: {path}')
    result = []
    seen = set()
    for line_number, raw_line in enumerate(
        path.read_text(encoding='utf-8').splitlines(),
        start=1,
    ):
        if not raw_line.strip():
            continue
        try:
            row = json.loads(raw_line)
        except json.JSONDecodeError as exc:
            raise SystemExit(
                f'INVALID_JSONL {path}:{line_number}: {exc}'
            ) from exc
        if not isinstance(row, dict):
            raise SystemExit(
                f'INVALID_JSONL_OBJECT {path}:{line_number}'
            )
        row_id = str(row.get(field) or '').strip()
        if not row_id:
            raise SystemExit(
                f'MISSING_JSONL_ID {path}:{line_number}: {field}'
            )
        if row_id in seen:
            raise SystemExit(
                f'DUPLICATE_JSONL_ID {path}:{line_number}: {row_id}'
            )
        seen.add(row_id)
        result.append(row_id)
    if not result:
        raise SystemExit(f'EMPTY_JSONL: {path}')
    return result

def load_object(path):
    if not path.is_file():
        raise SystemExit(f'MISSING_JSON: {path}')
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except json.JSONDecodeError as exc:
        raise SystemExit(f'INVALID_JSON {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise SystemExit(f'INVALID_JSON_OBJECT: {path}')
    return value

current_approval = ROOT / 'smoke_manual_approval.json'
legacy_approval = ROOT / 'manual_smoke_approval.json'
if current_approval.exists() and legacy_approval.exists():
    raise SystemExit('COMPETING_APPROVAL_FILES')
if not current_approval.is_file():
    if legacy_approval.exists():
        raise SystemExit(
            'LEGACY_APPROVAL_IS_NOT_AUTHORITATIVE_FOR_THIS_PACKAGE'
        )
    raise SystemExit('MISSING_SMOKE_MANUAL_APPROVAL')

parser = argparse.ArgumentParser()
parser.add_argument(
    '--require',
    choices=('manifest', 'capture', 'gallery'),
    default='manifest',
)
args = parser.parse_args()
manifest_ids = jsonl_ids(ROOT / 'scenario_manifest.jsonl', 'scenario_id')
manifest_id_set = set(manifest_ids)
approval = load_object(current_approval)
if approval.get('schema_version') != 'room315.arbitrary_subset_smoke_approval.v1':
    raise SystemExit('INVALID_SMOKE_APPROVAL_SCHEMA')
if (
    args.require == 'capture'
    and approval.get('approved_for_smoke_capture') is not True
):
    print('WAITING_FOR_SMOKE_CAPTURE_APPROVAL')
    raise SystemExit(3)
if args.require == 'gallery':
    state = load_object(ROOT / 'capture_state.json')
    if (
        state.get('capture_complete') is not True
        or state.get('expected_scenario_count') != len(manifest_ids)
        or state.get('captured_scenario_count') != len(manifest_ids)
    ):
        print('WAITING_FOR_COMPLETE_SMOKE_CAPTURE')
        raise SystemExit(5)
    event_ids = jsonl_ids(
        ROOT / 'dataset' / 'meta' / 'training_events.jsonl',
        'episode_id',
    )
    event_id_set = set(event_ids)
    if event_id_set != manifest_id_set:
        raise SystemExit(
            'MANIFEST_CAPTURE_ID_MISMATCH '
            f'missing={sorted(manifest_id_set - event_id_set)} '
            f'unexpected={sorted(event_id_set - manifest_id_set)}'
        )
    gallery = load_object(ROOT / 'manual_inspection_gallery_manifest.json')
    gallery_rows = gallery.get('scenarios')
    if not isinstance(gallery_rows, list):
        raise SystemExit('INVALID_GALLERY_SCENARIOS')
    gallery_ids = []
    for index, row in enumerate(gallery_rows, start=1):
        if not isinstance(row, dict):
            raise SystemExit(f'INVALID_GALLERY_SCENARIO_OBJECT: {index}')
        scenario_id = str(row.get('scenario_id') or '').strip()
        if not scenario_id:
            raise SystemExit(f'MISSING_GALLERY_SCENARIO_ID: {index}')
        if scenario_id in gallery_ids:
            raise SystemExit(f'DUPLICATE_GALLERY_SCENARIO_ID: {scenario_id}')
        gallery_ids.append(scenario_id)
    gallery_id_set = set(gallery_ids)
    if gallery_id_set != manifest_id_set:
        raise SystemExit(
            'MANIFEST_GALLERY_ID_MISMATCH '
            f'missing={sorted(manifest_id_set - gallery_id_set)} '
            f'unexpected={sorted(gallery_id_set - manifest_id_set)}'
        )
    expected_images = len(manifest_ids) * 2
    if (
        gallery.get('scenario_count') != len(manifest_ids)
        or gallery.get('source_image_count') != expected_images
        or gallery.get('overlay_image_count') != expected_images
        or gallery.get('source_images_unchanged') is not True
        or gallery.get('exact_subset_validation') is not True
    ):
        raise SystemExit('INCOMPLETE_OR_INVALID_GALLERY')
    if approval.get('approved_after_gallery_review') is not True:
        print('WAITING_FOR_GALLERY_REVIEW_APPROVAL')
        raise SystemExit(6)
if approval.get('approved_for_training') is not False:
    print('INVALID_APPROVAL_SCOPE')
    raise SystemExit(4)
print('SMOKE_APPROVAL_VALID')
'''
    return source.replace('__PACKAGE_ROOT__', repr(str(package_root)))


def package_manifest(
    root: Path,
    *,
    declared_root: Path | None = None,
    schema_version: str = 'room315.arbitrary_subset_smoke_package.v1',
    scenario_count: int = SMOKE_SCENARIO_COUNT,
) -> dict[str, Any]:
    files = {}
    for path in sorted(item for item in root.rglob('*') if item.is_file()):
        relative = str(path.relative_to(root))
        files[relative] = {
            'bytes': path.stat().st_size,
            'sha256': sha256(path),
        }
    return {
        'schema_version': schema_version,
        'package_root': str(declared_root or root),
        'seed': SEED,
        'scenario_count': scenario_count,
        'visual_state_schema': VISUAL_STATE_SCHEMA_VERSION,
        'fixed_identity_order': list(GLOBAL_IDENTITIES),
        'fixed_vectorizer_dimension': VisualStateLabelVectorizer().dim,
        'capture_executed': False,
        'approved_for_capture': False,
        'files': files,
    }


def prepare_smoke_package(
    output: Path,
    *,
    seed: int = SEED,
    count: int = SMOKE_SCENARIO_COUNT,
) -> Path:
    final, temporary = _atomic_package_root(output)
    try:
        records = presence_inventory()
        scenarios = generate_smoke_manifest(records, seed=seed, count=count)
        audit = static_smoke_audit(scenarios, records)
        if not audit['passed']:
            raise ArbitrarySubsetError(
                f'generated smoke audit failed: {audit["checks"]}'
            )
        color = color_identity_audit(scenarios)
        if not color['passed']:
            raise ArbitrarySubsetError(
                f'color identity coverage failed: {color["checks"]}'
            )
        write_jsonl(temporary / 'scenario_manifest.jsonl', scenarios)
        write_json(temporary / 'static_smoke_audit.json', audit)
        write_json(temporary / 'color_identity_coverage_audit.json', color)
        write_json(temporary / 'selected_presence_configurations.json', {
            'seed': seed,
            'count': len(scenarios),
            'configuration_ids': [
                scenario['presence_configuration_id']
                for scenario in scenarios
            ],
        })
        write_json(temporary / 'smoke_manual_approval.json', {
            'schema_version': 'room315.arbitrary_subset_smoke_approval.v1',
            'approved_for_smoke_capture': False,
            'approved_after_gallery_review': False,
            'approved_for_training': False,
            'reviewer': '',
            'reviewed_at': '',
            'notes': (
                'Set approved_for_smoke_capture only after static manifest review. '
                'Gallery review remains separate after capture.'
            ),
        })
        (temporary / 'dataset').mkdir()
        (temporary / 'manual_review').mkdir()
        (temporary / 'capture_logs').mkdir()
        (temporary / 'capture_failures.jsonl').write_text('', encoding='utf-8')
        write_json(temporary / 'capture_state.json', {
            'capture_has_started': False,
            'capture_complete': False,
            'captured_scenario_count': 0,
            'expected_scenario_count': count,
        })
        for name, content in _smoke_scripts(final).items():
            path = temporary / name
            path.write_text(content, encoding='utf-8')
            _make_executable(path)
        helpers = {
            'capture_status.py': _capture_status_source(final),
            'create_smoke_gallery.py': _gallery_source(final),
            'validate_smoke_approval.py': _approval_validator_source(final),
        }
        for name, content in helpers.items():
            path = temporary / name
            path.write_text(content, encoding='utf-8')
            _make_executable(path)
        readme = f'''# Room 315 arbitrary-subset visual smoke

This deterministic `{count}`-scenario smoke uses seed `{seed}` and exact
identity-list activation. It covers arbitrary non-prefix subsets while keeping
the fixed `{VISUAL_STATE_SCHEMA_VERSION}` eight-slot, 200-dimensional target.

Capture is not approved and has not run. Review `static_smoke_audit.json`, then
explicitly set `approved_for_smoke_capture` in `smoke_manual_approval.json`.

```bash
cd {final}
./generate_smoke_manifest.sh
./audit_smoke_manifest.sh
./validate_smoke_approval.py --require capture
./capture_smoke.sh
./resume_smoke.sh
./audit_captured_smoke.sh
./create_smoke_gallery.py
# After human review, set approved_after_gallery_review=true, then:
./validate_smoke_approval.py --require gallery
```

Relation metadata is capture/oracle audit context only. It is never a model
input or perception target. No split or training command is provided.
'''
        (temporary / 'README.md').write_text(readme, encoding='utf-8')
        write_json(
            temporary / 'package_manifest.json',
            package_manifest(temporary, declared_root=final),
        )
        os.replace(temporary, final)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return final


def production_plan(records: list[dict[str, Any]]) -> tuple[
    list[dict[str, Any]], dict[str, Any]
]:
    rows = []
    roles = ('future_train',) * 6 + (
        'future_validation',
        'future_blind_test',
    )
    for record in records:
        eligible = record['target_eligible_identities']
        for variant in range(PRODUCTION_VARIANTS):
            target = eligible[variant % len(eligible)]
            families = record['relation_eligibility'][target]
            family = families[variant % len(families)]
            target_side = 'left' if target.startswith('L') else 'right'
            target_rail = record[f'{target_side}_active_identities']
            relation_count = 2 if family == 'multi_blocker' else (
                0 if family == NO_RELATION else 1
            )
            relation_identities = [
                identity
                for identity in target_rail
                if identity != target
            ][:relation_count]
            payload_assignment = {
                identity: (
                    'loaded'
                    if (
                        variant + GLOBAL_IDENTITIES.index(identity)
                    ) % 2 == 0
                    else 'empty'
                )
                for identity in eligible
            }
            rows.append({
                'plan_id': f'{record["configuration_id"]}_variant_{variant + 1:02d}',
                'configuration_id': record['configuration_id'],
                'variant_index': variant + 1,
                'planned_partition_role': roles[variant],
                'left_active_identities': record['left_active_identities'],
                'right_active_identities': record['right_active_identities'],
                'target_identity': target,
                'relation_family': family,
                'relation_identities': relation_identities,
                'relation_neutral_identities': [
                    identity
                    for identity in target_rail
                    if identity != target
                    and identity not in relation_identities
                ],
                'opposite_rail_distractor_identities': record[
                    f'{"right" if target_side == "left" else "left"}_active_identities'
                ],
                'position_seed': stable_int(
                    SEED,
                    record['configuration_id'],
                    variant,
                    'position',
                ),
                'payload_assignment_mask': (
                    record['bitmask_decimal']
                    ^ stable_int(SEED, variant, 'payload')
                ) & 0xFF,
                'payload_assignment': payload_assignment,
                'target_zone': POSITION_ZONES[
                    (
                        record['bitmask_decimal']
                        + variant
                    ) % len(POSITION_ZONES)
                ],
                'target_segment': valid_public_segments(target_side)[
                    (
                        record['bitmask_decimal']
                        + variant
                    ) % len(valid_public_segments(target_side))
                ],
                'switch_pattern_variant': (
                    record['bitmask_decimal'] + variant
                ) % 16,
                'geometry_variant_must_be_unique_within_configuration': True,
                'relation_metadata_model_input': False,
            })
    summary = {
        'schema_version': PLAN_SCHEMA,
        'seed': SEED,
        'presence_configuration_count': len(records),
        'variants_per_configuration': PRODUCTION_VARIANTS,
        'total_planned_scenarios': len(rows),
        'planned_partition_roles_only_no_split_files_created': {
            'future_train': 1530,
            'future_validation': 255,
            'future_blind_test': 255,
        },
        'exact_configuration_coverage': all(
            sum(
                row['configuration_id'] == record['configuration_id']
                for row in rows
            ) == PRODUCTION_VARIANTS
            for record in records
        ),
        'minimum_alternative': {
            'variants_per_configuration': MINIMUM_VARIANTS,
            'total_scenarios': 1020,
            'limitations': [
                'half as many geometry and camera-view variations per exact subset',
                'weaker per-identity loaded/empty balance within rare sparse subsets',
                'less relation-family coverage for three/four-shuttle target rails',
                'less robust boundary/switch/slot/ordinary-zone coverage',
                'higher variance in identity-confusion estimates',
                'insufficient room for 6/1/1 per-configuration future partitioning',
            ],
        },
        'split_status': (
            'design only; no train, validation, or test split files created'
        ),
    }
    return rows, summary


def production_plan_audit(
    records: list[dict[str, Any]],
    rows: list[dict[str, Any]],
) -> dict[str, Any]:
    by_id = {
        record['configuration_id']: record
        for record in records
    }
    config_counts = Counter(row['configuration_id'] for row in rows)
    count_pairs = Counter()
    total_counts = Counter()
    presence = Counter()
    absence = Counter()
    alone = Counter()
    loaded = Counter()
    empty = Counter()
    target = Counter()
    roles = Counter()
    relations = Counter()
    zones = Counter()
    segments = Counter()
    pairwise = {
        first: {second: 0 for second in GLOBAL_IDENTITIES}
        for first in GLOBAL_IDENTITIES
    }
    invalid_relations = []
    duplicate_geometric_keys = []
    per_config_geometry = {}
    for row in rows:
        record = by_id[row['configuration_id']]
        active = (
            record['left_active_identities']
            + record['right_active_identities']
        )
        count_pairs[f'{record["left_count"]}+{record["right_count"]}'] += 1
        total_counts[str(record['total_count'])] += 1
        if len(active) == 1:
            alone[active[0]] += 1
        for identity in GLOBAL_IDENTITIES:
            if identity in active:
                presence[identity] += 1
            else:
                absence[identity] += 1
        for first in active:
            for second in active:
                pairwise[first][second] += 1
        for identity, state in row['payload_assignment'].items():
            (loaded if state == 'loaded' else empty)[identity] += 1
        target[row['target_identity']] += 1
        roles[f'{row["target_identity"]}:target'] += 1
        role = (
            'non_blocker'
            if row['relation_family'] in {
                'nonblocker_behind_same_segment',
                'nonblocker_adjacent_branch',
            }
            else 'blocker'
        )
        for identity in row['relation_identities']:
            roles[f'{identity}:{role}'] += 1
        for identity in (
            row['relation_neutral_identities']
            + row['opposite_rail_distractor_identities']
        ):
            roles[f'{identity}:relation_neutral'] += 1
        relations[row['relation_family']] += 1
        zones[row['target_zone']] += 1
        side = 'left' if row['target_identity'].startswith('L') else 'right'
        segments[f'{side}:{row["target_segment"]}'] += 1
        if row['relation_family'] not in record[
            'relation_eligibility'
        ][row['target_identity']]:
            invalid_relations.append(row['plan_id'])
        geometry_key = (
            row['position_seed'],
            row['target_zone'],
            row['target_segment'],
            row['switch_pattern_variant'],
        )
        key = row['configuration_id']
        seen = per_config_geometry.setdefault(key, set())
        if geometry_key in seen:
            duplicate_geometric_keys.append(row['plan_id'])
        seen.add(geometry_key)
    expected_pairs = {
        f'{left}+{right}'
        for left in range(5)
        for right in range(5)
        if left or right
    }
    checks = {
        'all_255_configurations': (
            len(config_counts) == 255
            and set(config_counts) == set(by_id)
        ),
        'exactly_eight_variants_each': all(
            config_counts[config_id] == 8 for config_id in by_id
        ),
        'exactly_2040_rows': len(rows) == 2040,
        'all_24_count_pairs': set(count_pairs) == expected_pairs,
        'total_counts_1_through_8': set(total_counts) == {
            str(value) for value in range(1, 9)
        },
        'every_identity_present_and_absent': all(
            presence[identity] > 0 and absence[identity] > 0
            for identity in GLOBAL_IDENTITIES
        ),
        'every_identity_alone': all(
            alone[identity] == 8 for identity in GLOBAL_IDENTITIES
        ),
        'every_identity_loaded_and_empty': all(
            loaded[identity] > 0 and empty[identity] > 0
            for identity in GLOBAL_IDENTITIES
        ),
        'every_identity_targeted': all(
            target[identity] > 0 for identity in GLOBAL_IDENTITIES
        ),
        'all_relation_eligibility_levels': set(relations) == {
            NO_RELATION,
            *ALL_RELATION_FAMILIES,
        },
        'zero_invalid_relations': not invalid_relations,
        'all_target_zones': set(zones) == set(POSITION_ZONES),
        'all_target_segments_both_rails': all(
            segments[f'{side}:{segment}'] > 0
            for side in ('left', 'right')
            for segment in valid_public_segments(side)
        ),
        'unique_geometry_plan_per_configuration': (
            not duplicate_geometric_keys
        ),
        'fixed_schema_v3_dimension_200': (
            VISUAL_STATE_SCHEMA_VERSION == 'room315.visual_state.v3'
            and VisualStateLabelVectorizer().dim == 200
        ),
        'no_dataset_inferred_capacity': (
            VisualStateLabelVectorizer().to_json()[
                'capacity_inferred_from_dataset'
            ] is False
        ),
    }
    return {
        'schema_version': 'room315.arbitrary_subset_production_plan_audit.v1',
        'passed': all(checks.values()),
        'checks': checks,
        'distributions': {
            'configuration_variant_count': dict(sorted(config_counts.items())),
            'cardinality_pair': dict(sorted(count_pairs.items())),
            'total_active_count': dict(sorted(total_counts.items())),
            'identity_presence': dict(sorted(presence.items())),
            'identity_absence': dict(sorted(absence.items())),
            'identity_alone': dict(sorted(alone.items())),
            'identity_loaded': dict(sorted(loaded.items())),
            'identity_empty': dict(sorted(empty.items())),
            'identity_target': dict(sorted(target.items())),
            'identity_roles': dict(sorted(roles.items())),
            'relation_family': dict(sorted(relations.items())),
            'target_zone': dict(sorted(zones.items())),
            'target_segment': dict(sorted(segments.items())),
            'pairwise_identity_cooccurrence': pairwise,
        },
        'violations': {
            'invalid_relations': invalid_relations,
            'duplicate_geometry_plan_keys': duplicate_geometric_keys,
        },
        'geometry_validation_status': {
            'physical_separation': (
                'not evaluated at design-only stage; guarded manifest '
                'generation must fail on any violation'
            ),
            'topology': (
                'target segments are authoritative; complete placement '
                'topology is evaluated during future manifest generation'
            ),
            'capture_executed': False,
        },
    }


def prepare_production_plan(output: Path) -> Path:
    final, temporary = _atomic_package_root(output)
    try:
        records = presence_inventory()
        rows, summary = production_plan(records)
        audit = production_plan_audit(records, rows)
        if not audit['passed']:
            raise ArbitrarySubsetError(
                f'production plan audit failed: {audit["checks"]}'
            )
        write_jsonl(temporary / 'configuration_variant_plan.jsonl', rows)
        write_json(temporary / 'production_design.json', summary)
        write_json(temporary / 'static_production_plan_audit.json', audit)
        write_json(temporary / 'minimum_alternative_1020.json', summary[
            'minimum_alternative'
        ])
        (temporary / 'README.md').write_text(
            f'''# Room 315 arbitrary-subset 2040-scenario production design

This is a static plan only. It covers all 255 exact presence configurations
with eight distinct planned variants each (`2040` total). Planned roles are
1530 future train, 255 future validation, and 255 future blind test.

No capture, dataset split, or training artifact exists here. The 1020-scenario
minimum alternative and its statistical limitations are documented in
`minimum_alternative_1020.json`.
''',
            encoding='utf-8',
        )
        write_json(
            temporary / 'package_manifest.json',
            package_manifest(
                temporary,
                declared_root=final,
                schema_version=(
                    'room315.arbitrary_subset_production_plan_package.v1'
                ),
                scenario_count=len(rows),
            ),
        )
        os.replace(temporary, final)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return final


def audit_smoke_file(manifest: Path, report: Path) -> dict[str, Any]:
    scenarios = read_jsonl(manifest)
    audit = static_smoke_audit(scenarios, presence_inventory())
    write_json(report, audit)
    return audit


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command', required=True)
    prepare = subparsers.add_parser('prepare-smoke')
    prepare.add_argument('--output', type=Path, required=True)
    prepare.add_argument('--seed', type=int, default=SEED)
    prepare.add_argument('--count', type=int, default=SMOKE_SCENARIO_COUNT)
    audit = subparsers.add_parser('audit-smoke')
    audit.add_argument('--manifest', type=Path, required=True)
    audit.add_argument('--report', type=Path, required=True)
    inventory = subparsers.add_parser('write-inventory')
    inventory.add_argument('--json', type=Path, required=True)
    inventory.add_argument('--markdown', type=Path, required=True)
    production = subparsers.add_parser('prepare-production-plan')
    production.add_argument('--output', type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.command == 'prepare-smoke':
        output = prepare_smoke_package(
            args.output,
            seed=args.seed,
            count=args.count,
        )
        print(f'ARBITRARY_SUBSET_SMOKE_PREPARED {output}')
        return 0
    if args.command == 'audit-smoke':
        audit = audit_smoke_file(args.manifest, args.report)
        print('STATIC_SMOKE_AUDIT_PASS' if audit['passed'] else 'STATIC_SMOKE_AUDIT_FAIL')
        return 0 if audit['passed'] else 2
    if args.command == 'write-inventory':
        records = presence_inventory()
        audit = inventory_audit(records)
        write_json(args.json, {
            'schema_version': INVENTORY_SCHEMA,
            'audit': audit,
            'configurations': records,
        })
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(
            inventory_markdown(records, audit),
            encoding='utf-8',
        )
        print('PRESENCE_INVENTORY_PASS' if audit['passed'] else 'PRESENCE_INVENTORY_FAIL')
        return 0 if audit['passed'] else 2
    if args.command == 'prepare-production-plan':
        output = prepare_production_plan(args.output)
        print(f'PRODUCTION_PLAN_PREPARED {output}')
        return 0
    raise AssertionError(args.command)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ArbitrarySubsetError, FileExistsError, OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
