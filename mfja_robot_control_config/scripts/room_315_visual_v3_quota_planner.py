#!/usr/bin/env python3
"""Deterministic quota planner for Room 315 hard-case visual dataset V3."""

from __future__ import annotations

import argparse
import itertools
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_scenario_generator import SLOT_RATIOS
from room_315_visual_v3_common import BLOCKS
from room_315_visual_v3_common import DEFAULT_GUARD_ROOT
from room_315_visual_v3_common import IDENTITIES
from room_315_visual_v3_common import OCCLUSION_CLASSES
from room_315_visual_v3_common import PACKAGE_SCHEMA
from room_315_visual_v3_common import POSITION_BINS
from room_315_visual_v3_common import POSITION_RATIOS
from room_315_visual_v3_common import RELATIONS
from room_315_visual_v3_common import RENDER_BUCKETS
from room_315_visual_v3_common import SEED
from room_315_visual_v3_common import TARGET_OFFSETS
from room_315_visual_v3_common import TARGET_OFFSET_BUCKETS
from room_315_visual_v3_common import VISUAL_SCHEMA
from room_315_visual_v3_common import ZONES
from room_315_visual_v3_common import VisualV3Error
from room_315_visual_v3_common import atomic_json
from room_315_visual_v3_common import side_for_identity
from room_315_visual_v3_common import stable_int
from room_315_visual_v3_common import target_offset_bucket
from room_315_visual_v3_common import value_sha256


TRAIN_COUNT = 4000
VALIDATION_COUNT = 512
CANARY_COUNT = 256
COUNT_BY_PROFILE = {
    'train': TRAIN_COUNT,
    'validation': VALIDATION_COUNT,
    'canary': CANARY_COUNT,
}
CARDINALITY_COUNTS = {
    'train': {1: 400, 2: 480, 3: 600, 4: 680, 5: 600, 6: 480, 7: 400, 8: 360},
    'validation': {1: 48, 2: 64, 3: 72, 4: 80, 5: 72, 6: 64, 7: 56, 8: 56},
    'canary': {1: 24, 2: 32, 3: 40, 4: 40, 5: 40, 6: 32, 7: 24, 8: 24},
}
ANCHOR_IDENTITIES = ('L2', 'L4', 'R4')
ANCHOR_PAYLOAD = {'L2': 'empty', 'L4': 'loaded', 'R4': 'loaded'}
ANCHOR_BLOCKS = {'L2': 'A12E', 'L4': 'A34E', 'R4': 'A12E'}
CANARY_FAMILY_COUNTS = {
    'l4_payload': 40,
    'r4_payload': 40,
    'combined_l4_r4': 48,
    'generic_command_perception': 40,
    'r4_right_slot3_position_arrival': 48,
    'negative_payload_matched_pair': 40,
}


def _interleaved_cardinalities(profile: str) -> list[int]:
    expected = CARDINALITY_COUNTS[profile]
    remaining = dict(expected)
    result = []
    while sum(remaining.values()):
        candidates = [value for value, count in remaining.items() if count]
        selected = min(
            candidates,
            key=lambda value: (
                len(result) * expected[value] / sum(expected.values())
                - (expected[value] - remaining[value]),
                stable_int(SEED, profile, len(result), value),
            ),
        )
        result.append(selected)
        remaining[selected] -= 1
    if Counter(result) != Counter(expected):
        raise VisualV3Error(f'cardinality allocation failed for {profile}')
    return result


def _active_subset(
    target: str,
    cardinality: int,
    *,
    profile: str,
    index: int,
    relation: str,
) -> tuple[str, ...]:
    if not 1 <= cardinality <= len(IDENTITIES):
        raise VisualV3Error(f'invalid active cardinality: {cardinality}')
    same_side = [
        identity
        for identity in IDENTITIES
        if identity != target and side_for_identity(identity) == side_for_identity(target)
    ]
    other_side = [
        identity
        for identity in IDENTITIES
        if identity != target and side_for_identity(identity) != side_for_identity(target)
    ]
    minimum_peers = 2 if relation == 'multi_blocker' else int(relation != RELATIONS[0])
    ranked_same = sorted(
        same_side,
        key=lambda identity: stable_int(SEED, profile, index, 'same', identity),
    )
    ranked_other = sorted(
        other_side,
        key=lambda identity: stable_int(SEED, profile, index, 'other', identity),
    )
    selected = [target]
    selected.extend(ranked_same[:min(minimum_peers, cardinality - 1)])
    remaining = [identity for identity in ranked_same + ranked_other if identity not in selected]
    remaining.sort(
        key=lambda identity: stable_int(SEED, profile, index, 'fill', identity)
    )
    selected.extend(remaining[:cardinality - len(selected)])
    return tuple(identity for identity in IDENTITIES if identity in selected)


def _payload_assignment(
    active: tuple[str, ...],
    target: str,
    target_state: str,
    *,
    profile: str,
    index: int,
) -> dict[str, str]:
    desired_loaded = index % (len(active) + 1)
    loaded = {target} if target_state == 'loaded' else set()
    available = [identity for identity in active if identity != target]
    available.sort(
        key=lambda identity: stable_int(SEED, profile, index, 'payload', identity)
    )
    needed = max(0, min(len(available), desired_loaded - len(loaded)))
    loaded.update(available[:needed])
    if target_state == 'empty':
        loaded.discard(target)
    return {
        identity: ('loaded' if identity in loaded else 'empty')
        for identity in active
    }


def _eligible_relation(requested: str, target: str, active: tuple[str, ...]) -> str:
    rail_count = sum(
        side_for_identity(identity) == side_for_identity(target)
        for identity in active
    )
    if requested == 'multi_blocker':
        return requested if rail_count >= 3 else RELATIONS[0]
    if requested != RELATIONS[0]:
        return requested if rail_count >= 2 else RELATIONS[0]
    return requested


def _zone_for(block: str, ratio: float, index: int) -> str:
    if block in {'A12E', 'A34E'}:
        target_ratios = tuple(
            ratio_value
            for side_values in SLOT_RATIOS.values()
            for candidate_block, ratios in side_values.items()
            if candidate_block == block
            for ratio_value in ratios
        )
        if target_ratios and min(abs(ratio - value) for value in target_ratios) <= 0.025:
            return 'slot'
    if ratio <= 0.07 or ratio >= 0.93:
        return 'boundary'
    return ZONES[index % len(ZONES)]


def _base_spec(profile: str, index: int, cardinality: int) -> dict[str, Any]:
    cell_index = index % (len(IDENTITIES) * 2 * len(BLOCKS) * len(POSITION_BINS))
    # Interleave identities first so every prefix (especially validation)
    # rotates targets across the complete fleet rather than exhausting L1
    # cells before reaching later identities.
    identity_index = cell_index % len(IDENTITIES)
    cell_index //= len(IDENTITIES)
    state_index = cell_index % 2
    cell_index //= 2
    block_index = cell_index % len(BLOCKS)
    position_index = (cell_index // len(BLOCKS)) % len(POSITION_BINS)
    target = IDENTITIES[identity_index]
    target_state = ('empty', 'loaded')[state_index]
    block = BLOCKS[block_index]
    ratio = POSITION_RATIOS[position_index]
    # The first complete training lattice is relation-neutral so every exact
    # identity/payload/block/position cell survives materialisation unchanged.
    # Remaining rows carry the balanced relational hard cases.
    requested_relation = (
        RELATIONS[0]
        if profile == 'train' and index < 2016
        else RELATIONS[index % len(RELATIONS)]
    )
    active = _active_subset(
        target,
        cardinality,
        profile=profile,
        index=index,
        relation=requested_relation,
    )
    relation = _eligible_relation(requested_relation, target, active)
    payload = _payload_assignment(
        active,
        target,
        target_state,
        profile=profile,
        index=index,
    )
    zone = _zone_for(block, ratio, index)
    return {
        'profile': profile,
        'generation_index': index,
        'seed': SEED,
        'active_identities': list(active),
        'loaded_identities': [
            identity for identity in active if payload[identity] == 'loaded'
        ],
        'payload_assignment': payload,
        'target_identity': target,
        'target_loaded_state': target_state,
        'target_block': block,
        'target_s_ratio': ratio,
        'target_position_bin': POSITION_BINS[position_index],
        'target_zone': zone,
        'target_offset_bucket': 'not_operational_target',
        'target_offset': None,
        'relation_family': relation,
        'occlusion_class': OCCLUSION_CLASSES[(index // 3) % len(OCCLUSION_CLASSES)],
        'render_bucket': RENDER_BUCKETS[index % len(RENDER_BUCKETS)],
        'render_parameters': {
            'bucket': RENDER_BUCKETS[index % len(RENDER_BUCKETS)],
            'deterministic_seed': stable_int(SEED, profile, index, 'render') % (2**31),
        },
        'approach_direction': ('increasing_s', 'decreasing_s')[(index // 2) % 2],
        'canary_family': None,
        'matched_pair_id': None,
        'matched_pair_role': None,
        'geometry_seed_index': index,
        'geometry_seed_key': f'{profile}:{index}',
        'position_overrides': {},
    }


def _apply_anchor(spec: dict[str, Any], variant: int) -> None:
    spec['active_identities'] = list(ANCHOR_IDENTITIES)
    spec['loaded_identities'] = [
        identity for identity in ANCHOR_IDENTITIES
        if ANCHOR_PAYLOAD[identity] == 'loaded'
    ]
    spec['payload_assignment'] = dict(ANCHOR_PAYLOAD)
    if spec['target_identity'] not in ANCHOR_IDENTITIES:
        spec['target_identity'] = ANCHOR_IDENTITIES[
            variant % len(ANCHOR_IDENTITIES)
        ]
    spec['target_loaded_state'] = ANCHOR_PAYLOAD[spec['target_identity']]
    spec['target_block'] = ANCHOR_BLOCKS[spec['target_identity']]
    if variant >= 200:
        base_ratios = {'L2': 0.15, 'L4': 0.85, 'R4': 0.15}
    elif variant >= 100:
        base_ratios = {'L2': 0.25, 'L4': 0.75, 'R4': 0.25}
    else:
        base_ratios = {'L2': 0.40, 'L4': 0.60, 'R4': 0.40}
    # Use a dense deterministic micro-grid so oversampling never renders an
    # exact duplicate while all variants remain in the same operational
    # region and coarse position bins.
    jitter = ((variant % 97) - 48) * 0.0007
    overrides = {
        identity: {
            'segment': block,
            's_ratio': max(0.05, min(0.95, base_ratios[identity] + jitter)),
        }
        for identity, block in ANCHOR_BLOCKS.items()
    }
    target_position = overrides[spec['target_identity']]
    spec['target_s_ratio'] = target_position['s_ratio']
    spec['target_position_bin'] = POSITION_BINS[min(
        range(len(POSITION_RATIOS)),
        key=lambda index: abs(POSITION_RATIOS[index] - target_position['s_ratio']),
    )]
    spec['target_zone'] = _zone_for(
        spec['target_block'],
        spec['target_s_ratio'],
        variant,
    )
    spec['relation_family'] = RELATIONS[0]
    spec['position_overrides'] = overrides
    spec['hard_case_tags'] = ['exact_l2_l4_r4_anchor']


def _apply_canary(spec: dict[str, Any], family: str, family_index: int) -> None:
    spec['canary_family'] = family
    if family == 'l4_payload':
        target = 'L4'
    elif family == 'r4_payload':
        target = 'R4'
    elif family in {
        'combined_l4_r4',
        'negative_payload_matched_pair',
    }:
        target = ('L4', 'R4')[family_index % 2]
    elif family == 'r4_right_slot3_position_arrival':
        target = 'R4'
    else:
        target = ('L2', 'L4', 'R2', 'R4')[family_index % 4]
    active = set(spec['active_identities'])
    active.add(target)
    if family == 'combined_l4_r4':
        active.update(('L4', 'R4'))
        if family_index % 2 == 0:
            active.add('L2')
    desired = len(spec['active_identities'])
    ranked = sorted(
        (identity for identity in IDENTITIES if identity not in active),
        key=lambda identity: stable_int(SEED, family, family_index, identity),
    )
    while len(active) < desired:
        active.add(ranked.pop(0))
    while len(active) > desired and len(active) > 1:
        removable = next(
            (
                identity for identity in reversed(IDENTITIES)
                if identity in active and identity not in {target, 'L4', 'R4'}
            ),
            None,
        )
        if removable is None:
            break
        active.remove(removable)
    spec['active_identities'] = [identity for identity in IDENTITIES if identity in active]
    spec['target_identity'] = target
    spec['target_loaded_state'] = ('empty', 'loaded')[family_index % 2]
    payload = _payload_assignment(
        tuple(spec['active_identities']),
        target,
        spec['target_loaded_state'],
        profile='canary',
        index=spec['generation_index'] + family_index,
    )
    if family == 'combined_l4_r4':
        pair_states = (
            ('loaded', 'loaded'),
            ('loaded', 'empty'),
            ('empty', 'loaded'),
            ('empty', 'empty'),
        )[family_index % 4]
        payload['L4'], payload['R4'] = pair_states
    spec['payload_assignment'] = payload
    spec['loaded_identities'] = [
        identity for identity in spec['active_identities']
        if payload[identity] == 'loaded'
    ]
    if family == 'r4_right_slot3_position_arrival':
        authoritative = SLOT_RATIOS['right']['A34E'][0]
        offset = TARGET_OFFSETS[family_index % len(TARGET_OFFSETS)]
        ratio = authoritative + offset
        if not 0.0 <= ratio <= 1.0:
            raise VisualV3Error('right-slot-3 canary ratio clipped unexpectedly')
        spec.update({
            'target_block': 'A34E',
            'target_s_ratio': ratio,
            'target_position_bin': POSITION_BINS[min(
                range(len(POSITION_RATIOS)),
                key=lambda index: abs(POSITION_RATIOS[index] - ratio),
            )],
            'target_zone': 'slot',
            'target_offset_bucket': target_offset_bucket(offset),
            'target_offset': offset,
            'relation_family': RELATIONS[0],
        })


def build_specs(profile: str, count: int | None = None) -> list[dict[str, Any]]:
    if profile not in COUNT_BY_PROFILE:
        raise VisualV3Error(f'unsupported profile: {profile}')
    expected_count = COUNT_BY_PROFILE[profile] if count is None else int(count)
    if expected_count <= 0:
        raise VisualV3Error('scenario count must be positive')
    if count is not None:
        cardinalities = [
            1 + index % len(IDENTITIES)
            for index in range(expected_count)
        ]
    else:
        cardinalities = _interleaved_cardinalities(profile)
    specs = [
        _base_spec(profile, index, cardinality)
        for index, cardinality in enumerate(cardinalities)
    ]

    if profile == 'train':
        anchor_candidates = [
            index for index, spec in enumerate(specs)
            if (
                index >= 2016
                and len(spec['active_identities']) == 3
                and spec['target_identity'] in ANCHOR_IDENTITIES
            )
        ][:96]
        for variant, index in enumerate(anchor_candidates):
            _apply_anchor(specs[index], variant)
    elif profile == 'validation':
        anchor_candidates = [
            index for index, spec in enumerate(specs)
            if (
                len(spec['active_identities']) == 3
                and spec['target_identity'] in ANCHOR_IDENTITIES
            )
        ][:24]
        for variant, index in enumerate(anchor_candidates):
            _apply_anchor(specs[index], variant + 101)
    else:
        families = list(itertools.chain.from_iterable(
            itertools.repeat(family, amount)
            for family, amount in CANARY_FAMILY_COUNTS.items()
        ))
        if len(families) != expected_count:
            if count is not None:
                families = [tuple(CANARY_FAMILY_COUNTS)[index % 6] for index in range(expected_count)]
            else:
                raise VisualV3Error('canary family total mismatch')
        family_indices = Counter()
        for spec, family in zip(specs, families):
            _apply_canary(spec, family, family_indices[family])
            family_indices[family] += 1
        anchor_candidates = [
            index for index, spec in enumerate(specs)
            if spec['canary_family'] == 'combined_l4_r4'
        ][:16]
        for variant, index in enumerate(anchor_candidates):
            family = specs[index]['canary_family']
            _apply_anchor(specs[index], variant + 203)
            specs[index]['canary_family'] = family
        negative_indices = [
            index for index, spec in enumerate(specs)
            if spec['canary_family'] == 'negative_payload_matched_pair'
        ]
        if len(negative_indices) % 2:
            raise VisualV3Error('negative canary family must contain complete pairs')
        for pair_index, (first_index, second_index) in enumerate(
            zip(negative_indices[::2], negative_indices[1::2])
        ):
            first = specs[first_index]
            second = specs[second_index]
            pair_id = f'canary_negative_pair_{pair_index + 1:03d}'
            geometry_fields = (
                'active_identities',
                'target_identity',
                'target_block',
                'target_s_ratio',
                'target_position_bin',
                'target_zone',
                'target_offset_bucket',
                'target_offset',
                'relation_family',
                'occlusion_class',
                'render_bucket',
                'render_parameters',
                'approach_direction',
                'position_overrides',
            )
            for field in geometry_fields:
                second[field] = json.loads(json.dumps(first[field]))
            first['geometry_seed_index'] = first['generation_index']
            second['geometry_seed_index'] = first['generation_index']
            first['geometry_seed_key'] = pair_id
            second['geometry_seed_key'] = pair_id
            first['matched_pair_id'] = pair_id
            second['matched_pair_id'] = pair_id
            first['matched_pair_role'] = 'payload_a'
            second['matched_pair_role'] = 'payload_b'
            first_payload = dict(first['payload_assignment'])
            second_payload = dict(first_payload)
            target = first['target_identity']
            second_payload[target] = (
                'empty' if first_payload[target] == 'loaded' else 'loaded'
            )
            second['payload_assignment'] = second_payload
            second['target_loaded_state'] = second_payload[target]
            second['loaded_identities'] = [
                identity for identity in second['active_identities']
                if second_payload[identity] == 'loaded'
            ]

    for spec in specs:
        spec['relation_family'] = _eligible_relation(
            spec['relation_family'],
            spec['target_identity'],
            tuple(spec['active_identities']),
        )
        spec['spec_id'] = (
            f'{profile}_v3_{spec["generation_index"] + 1:04d}_'
            f'{value_sha256(spec)[:12]}'
        )
    return specs


def quota_plan() -> dict[str, Any]:
    specs_by_profile = {
        profile: build_specs(profile)
        for profile in ('train', 'validation', 'canary')
    }
    cells = []
    train_counts = Counter(
        (
            spec['target_identity'],
            spec['target_loaded_state'],
            spec['target_block'],
            spec['target_position_bin'],
        )
        for spec in specs_by_profile['train']
    )
    for identity, state, block, position in itertools.product(
        IDENTITIES,
        ('empty', 'loaded'),
        BLOCKS,
        POSITION_BINS,
    ):
        key = (identity, state, block, position)
        cells.append({
            'identity': identity,
            'loaded_state': state,
            'block': block,
            'position_bin': position,
            'requested_count': 1,
            'allocated_target_count': train_counts[key],
            'physically_valid': True,
            'infeasible_reason': None,
        })
    unsatisfied = [cell for cell in cells if cell['allocated_target_count'] < 1]
    distributions: dict[str, Any] = {}
    for profile, specs in specs_by_profile.items():
        distributions[profile] = {
            'scenario_count': len(specs),
            'expected_image_count': len(specs) * len(('left', 'right')),
            'expected_active_shuttle_instances': sum(
                len(spec['active_identities']) for spec in specs
            ),
            'presence_cardinality': dict(sorted(Counter(
                len(spec['active_identities']) for spec in specs
            ).items())),
            'loaded_count': dict(sorted(Counter(
                len(spec['loaded_identities']) for spec in specs
            ).items())),
            'relation_family': dict(sorted(Counter(
                spec['relation_family'] for spec in specs
            ).items())),
            'render_bucket': dict(sorted(Counter(
                spec['render_bucket'] for spec in specs
            ).items())),
            'hard_case_anchor_count': sum(
                'exact_l2_l4_r4_anchor' in spec.get('hard_case_tags', [])
                for spec in specs
            ),
            'canary_family': dict(sorted(Counter(
                spec['canary_family'] for spec in specs
                if spec['canary_family']
            ).items())),
        }
    plan = {
        'schema_version': PACKAGE_SCHEMA,
        'visual_schema_version': VISUAL_SCHEMA,
        'seed': SEED,
        'fixed_identity_order': list(IDENTITIES),
        'authoritative_public_blocks': list(BLOCKS),
        'position_ratios': list(POSITION_RATIOS),
        'position_bins': list(POSITION_BINS),
        'target_offsets': list(TARGET_OFFSETS),
        'target_offset_buckets': list(TARGET_OFFSET_BUCKETS),
        'valid_conditional_cells': cells,
        'valid_conditional_cell_count': len(cells),
        'requested_count_per_valid_cell': 1,
        'unsatisfied_cells': unsatisfied,
        'infeasible_cells': [],
        'expected_scenario_total': TRAIN_COUNT + VALIDATION_COUNT,
        'expected_canary_total': CANARY_COUNT,
        'distributions': distributions,
        'hard_case_allocation': {
            profile: distributions[profile]['hard_case_anchor_count']
            for profile in distributions
        },
        'validation_family_allocation': {
            'scenario_count': VALIDATION_COUNT,
            'selection_basis': 'planned before capture; no model predictions',
        },
        'canary_allocation': CANARY_FAMILY_COUNTS,
        'passed': not unsatisfied,
    }
    plan['quota_plan_sha256'] = value_sha256(plan)
    return plan


def write_quota_plan(output: Path) -> dict[str, Any]:
    plan = quota_plan()
    if not plan['passed']:
        raise VisualV3Error(
            f'quota plan has {len(plan["unsatisfied_cells"])} unsatisfied cells'
        )
    atomic_json(output, plan)
    return plan


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output',
        type=Path,
        default=DEFAULT_GUARD_ROOT / 'room315_visual_v3_quota_plan.json',
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    plan = write_quota_plan(args.output)
    print(json.dumps({
        'output': str(args.output),
        'quota_plan_sha256': plan['quota_plan_sha256'],
        'valid_conditional_cells': plan['valid_conditional_cell_count'],
        'passed': plan['passed'],
    }, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, VisualV3Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
