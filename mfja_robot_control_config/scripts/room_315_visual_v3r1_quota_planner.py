#!/usr/bin/env python3
"""Deterministic exact-offset quota planner for Room 315 visual V3R1."""

from __future__ import annotations

import argparse
import copy
import itertools
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_v3_common import IDENTITIES
from room_315_visual_v3_common import POSITION_BINS
from room_315_visual_v3_common import POSITION_RATIOS
from room_315_visual_v3_common import RENDER_BUCKETS
from room_315_visual_v3_common import SEED
from room_315_visual_v3_common import TARGET_OFFSETS
from room_315_visual_v3_common import VisualV3Error
from room_315_visual_v3_common import atomic_json
from room_315_visual_v3_common import position_bin
from room_315_visual_v3_common import stable_int
from room_315_visual_v3_common import target_offset_bucket
from room_315_visual_v3_common import value_sha256
from room_315_visual_v3_quota_planner import build_specs as build_v3_specs
from room_315_visual_v3r1_common import CANARY_COUNT
from room_315_visual_v3r1_common import CANARY_EXPLICIT_COUNT
from room_315_visual_v3r1_common import CANARY_RELATIONS
from room_315_visual_v3r1_common import DEFAULT_GUARD_ROOT
from room_315_visual_v3r1_common import GENERATOR_VERSION
from room_315_visual_v3r1_common import MANIFEST_REVISION
from room_315_visual_v3r1_common import OPERATIONAL_TARGET_IDENTITY
from room_315_visual_v3r1_common import OPERATIONAL_TARGET_NAME
from room_315_visual_v3r1_common import OPERATIONAL_TARGET_RATIO
from room_315_visual_v3r1_common import OPERATIONAL_TARGET_SEGMENT
from room_315_visual_v3r1_common import PACKAGE_SCHEMA
from room_315_visual_v3r1_common import PRESENCE_CARDINALITY
from room_315_visual_v3r1_common import SMOKE_COUNT
from room_315_visual_v3r1_common import SMOKE_RELATIONS
from room_315_visual_v3r1_common import TRAIN_COUNT
from room_315_visual_v3r1_common import TRAIN_EXPLICIT_COUNT
from room_315_visual_v3r1_common import TRAIN_VALIDATION_RELATIONS
from room_315_visual_v3r1_common import VALIDATION_COUNT
from room_315_visual_v3r1_common import VALIDATION_EXPLICIT_COUNT
from room_315_visual_v3r1_common import quota_plan_path


_ACTIVE_PATTERNS = {
    ('train', 'sparse', 0): ('L1', 'R1', 'R4'),
    ('train', 'sparse', 1): ('L2', 'R2', 'R4'),
    ('train', 'medium', 0): ('L1', 'L2', 'R1', 'R4'),
    ('train', 'medium', 1): ('L3', 'L4', 'R2', 'R4'),
    ('train', 'dense', 0): ('L1', 'L2', 'L3', 'L4', 'R1', 'R4'),
    ('train', 'dense', 1): ('L1', 'L2', 'L3', 'R2', 'R3', 'R4'),
    ('validation', 'sparse', 0): ('L3', 'R3', 'R4'),
    ('validation', 'medium', 0): ('L1', 'L4', 'R3', 'R4'),
    ('validation', 'dense', 0): ('L2', 'L3', 'L4', 'R1', 'R3', 'R4'),
    ('canary', 'sparse', 0): ('L4', 'R2', 'R4'),
    ('canary', 'dense', 0): ('L1', 'L3', 'L4', 'R1', 'R2', 'R4'),
    ('smoke', 'sparse', 0): ('L2', 'R3', 'R4'),
    ('smoke', 'dense', 0): ('L1', 'L2', 'L4', 'R1', 'R3', 'R4'),
}


def _peer(active: tuple[str, ...]) -> str:
    return next(identity for identity in active if identity.startswith('R') and identity != 'R4')


def _relation_geometry(
    relation: str,
    *,
    active: tuple[str, ...],
    ratio: float,
    variant: int,
    profile: str,
) -> tuple[dict[str, dict[str, Any]], list[dict[str, str]]]:
    overrides: dict[str, dict[str, Any]] = {
        'R4': {
            'segment': OPERATIONAL_TARGET_SEGMENT,
            's_ratio': ratio,
            'position_zone': 'slot',
        },
    }
    if relation == 'no_relation_observation':
        return overrides, []
    peer = _peer(active)
    # A34E is curved: a ratio separation that clears the along-track
    # threshold can still be too close in world-space.  These values remain
    # valid at both extreme operational offsets and clear both validators.
    separation = 0.27 + 0.01 * variant
    kind = ''
    if relation == 'blocker_ahead_same_segment':
        segment = 'A34E'
        peer_ratio = ratio + separation
        zone = 'ahead_region'
        kind = 'ahead_blocker'
    elif relation == 'nonblocker_behind_same_segment':
        segment = 'A34E'
        peer_ratio = ratio - separation
        zone = 'behind_region'
        kind = 'behind_non_blocker'
    elif relation == 'blocker_intermediate_segment':
        segment = {
            'train': ('A4E', 'A14')[variant],
            'validation': ('A1E',)[0],
            'canary': ('A4E',)[0],
            'smoke': ('A14',)[0],
        }[profile]
        peer_ratio = 0.35 + 0.1 * variant
        zone = 'intermediate_route'
        kind = 'intermediate_blocker'
    elif relation == 'nonblocker_adjacent_branch':
        segment = 'A34I'
        peer_ratio = ratio
        zone = 'adjacent_branch'
        kind = 'adjacent_branch_non_blocker'
    else:
        raise VisualV3Error(f'unsupported V3R1 explicit relation: {relation}')
    if not 0.0 <= peer_ratio <= 1.0:
        raise VisualV3Error(
            f'{relation} peer ratio outside [0,1]: {peer_ratio}'
        )
    overrides[peer] = {
        'segment': segment,
        's_ratio': peer_ratio,
        'position_zone': zone,
    }
    return overrides, [{'other_shuttle_id': peer, 'relation': kind}]


def _payload(
    active: tuple[str, ...],
    target_state: str,
    *,
    cell_index: int,
) -> dict[str, str]:
    result = {}
    for identity in active:
        if identity == 'R4':
            result[identity] = target_state
        else:
            result[identity] = (
                'loaded'
                if stable_int(SEED, cell_index, identity, 'v3r1-payload') % 2
                else 'empty'
            )
    return result


def explicit_specs(
    profile: str,
    *,
    start_index: int,
) -> list[dict[str, Any]]:
    if profile == 'train':
        presence_classes = ('sparse', 'medium', 'dense')
        relations = TRAIN_VALIDATION_RELATIONS
        variants = (0, 1)
    elif profile == 'validation':
        presence_classes = ('sparse', 'medium', 'dense')
        relations = TRAIN_VALIDATION_RELATIONS
        variants = (0,)
    elif profile == 'canary':
        presence_classes = ('sparse', 'dense')
        relations = CANARY_RELATIONS
        variants = (0,)
    else:
        raise VisualV3Error(f'unsupported explicit profile: {profile}')
    rows = []
    for cell_index, (
        offset,
        target_state,
        presence,
        relation,
        variant,
    ) in enumerate(itertools.product(
        TARGET_OFFSETS,
        ('empty', 'loaded'),
        presence_classes,
        relations,
        variants,
    )):
        generation_index = start_index + cell_index
        active = _ACTIVE_PATTERNS[(profile, presence, variant)]
        ratio = OPERATIONAL_TARGET_RATIO + float(offset)
        if not 0.0 <= ratio <= 1.0:
            raise VisualV3Error(
                f'{profile}: operational target ratio invalid: {ratio}'
            )
        overrides, explicit_relations = _relation_geometry(
            relation,
            active=active,
            ratio=ratio,
            variant=variant,
            profile=profile,
        )
        payload_seed = stable_int(
            SEED,
            profile,
            float(offset),
            presence,
            relation,
            variant,
            'v3r1-payload-pair',
        )
        payload = _payload(
            active,
            target_state,
            cell_index=payload_seed,
        )
        bucket = target_offset_bucket(offset)
        geometry_pair_seed = stable_int(
            SEED,
            profile,
            float(offset),
            presence,
            relation,
            variant,
            'v3r1-geometry-pair',
        )
        render_bucket = RENDER_BUCKETS[
            geometry_pair_seed % len(RENDER_BUCKETS)
        ]
        pair_id = (
            f'v3r1_pair_{profile}_{bucket}_{presence}_{relation}_{variant}'
        )
        spec = {
            'profile': profile,
            'generation_index': generation_index,
            'seed': SEED,
            'active_identities': list(active),
            'loaded_identities': [
                identity for identity in active
                if payload[identity] == 'loaded'
            ],
            'payload_assignment': payload,
            'target_identity': OPERATIONAL_TARGET_IDENTITY,
            'target_loaded_state': target_state,
            'target_block': OPERATIONAL_TARGET_SEGMENT,
            'target_s_ratio': ratio,
            'target_position_bin': position_bin(ratio),
            'target_zone': 'slot',
            'target_ratio': OPERATIONAL_TARGET_RATIO,
            'target_offset_bucket': bucket,
            'target_offset': float(offset),
            'operational_target_name': OPERATIONAL_TARGET_NAME,
            'operational_target_segment': OPERATIONAL_TARGET_SEGMENT,
            'presence_class': presence,
            'configuration_variant': variant,
            'relation_family': relation,
            'explicit_relations': explicit_relations,
            'position_overrides': overrides,
            'occlusion_class': 'partial_risk' if presence == 'dense' else 'clear',
            'render_bucket': render_bucket,
            'render_parameters': {
                'bucket': render_bucket,
                'deterministic_seed': stable_int(
                    SEED,
                    profile,
                    geometry_pair_seed,
                    'v3r1-render-pair',
                ) % (2**31),
            },
            'approach_direction': (
                'increasing_s' if profile != 'validation' else 'decreasing_s'
            ),
            'canary_family': (
                'r4_right_slot3_position_arrival_v3r1'
                if profile == 'canary'
                else None
            ),
            'matched_pair_id': pair_id,
            'matched_pair_role': target_state,
            'hard_case_tags': [
                'deliberate_right_slot3_exact_offset',
                f'presence_{presence}',
                f'relation_{relation}',
            ],
            'geometry_seed_index': (
                900000 + geometry_pair_seed % 100000
            ),
            'geometry_seed_key': (
                f'v3r1:{profile}:{bucket}:{presence}:{relation}:{variant}'
            ),
            'family_avoidance_attempt': 0,
            'source_profile': profile,
            'imported_from_v3': False,
            'source_scenario_id': None,
            'source_manifest_sha256': None,
            'v3r1_manifest_revision': MANIFEST_REVISION,
        }
        spec['spec_id'] = (
            f'v3r1_{profile}_offset_{cell_index + 1:04d}_'
            f'{value_sha256(spec)[:12]}'
        )
        rows.append(spec)
    expected = {
        'train': TRAIN_EXPLICIT_COUNT,
        'validation': VALIDATION_EXPLICIT_COUNT,
        'canary': CANARY_EXPLICIT_COUNT,
    }[profile]
    if len(rows) != expected:
        raise VisualV3Error(
            f'{profile}: explicit quota mismatch {len(rows)} != {expected}'
        )
    return rows


def smoke_specs() -> list[dict[str, Any]]:
    rows = []
    for cell_index, (offset, state, presence) in enumerate(
        itertools.product(
            TARGET_OFFSETS,
            ('empty', 'loaded'),
            ('sparse', 'dense'),
        )
    ):
        offset_index = TARGET_OFFSETS.index(offset)
        presence_index = ('sparse', 'dense').index(presence)
        relation = SMOKE_RELATIONS[
            (offset_index * 2 + presence_index)
            % len(SMOKE_RELATIONS)
        ]
        active = _ACTIVE_PATTERNS[('smoke', presence, 0)]
        ratio = OPERATIONAL_TARGET_RATIO + float(offset)
        overrides, explicit_relations = _relation_geometry(
            relation,
            active=active,
            ratio=ratio,
            variant=0,
            profile='smoke',
        )
        payload_seed = stable_int(
            SEED,
            'smoke',
            float(offset),
            presence,
            relation,
            'v3r1-payload-pair',
        )
        payload = _payload(
            active,
            state,
            cell_index=payload_seed,
        )
        bucket = target_offset_bucket(offset)
        geometry_pair_seed = stable_int(
            SEED,
            'smoke',
            float(offset),
            presence,
            relation,
            'v3r1-geometry-pair',
        )
        render_bucket = RENDER_BUCKETS[
            geometry_pair_seed % len(RENDER_BUCKETS)
        ]
        spec = {
            'profile': 'smoke',
            'generation_index': cell_index,
            'seed': SEED,
            'active_identities': list(active),
            'loaded_identities': [
                identity for identity in active
                if payload[identity] == 'loaded'
            ],
            'payload_assignment': payload,
            'target_identity': 'R4',
            'target_loaded_state': state,
            'target_block': 'A34E',
            'target_s_ratio': ratio,
            'target_position_bin': position_bin(ratio),
            'target_zone': 'slot',
            'target_ratio': OPERATIONAL_TARGET_RATIO,
            'target_offset_bucket': bucket,
            'target_offset': float(offset),
            'operational_target_name': OPERATIONAL_TARGET_NAME,
            'operational_target_segment': OPERATIONAL_TARGET_SEGMENT,
            'presence_class': presence,
            'configuration_variant': 0,
            'relation_family': relation,
            'explicit_relations': explicit_relations,
            'position_overrides': overrides,
            'occlusion_class': 'partial_risk' if presence == 'dense' else 'clear',
            'render_bucket': render_bucket,
            'render_parameters': {
                'bucket': render_bucket,
                'deterministic_seed': stable_int(
                    SEED, 'v3r1-smoke', geometry_pair_seed
                ) % (2**31),
            },
            'approach_direction': 'increasing_s',
            'canary_family': 'v3r1_position_correction_smoke',
            'matched_pair_id': f'v3r1_smoke_{bucket}_{presence}_{relation}',
            'matched_pair_role': state,
            'hard_case_tags': ['deliberate_right_slot3_exact_offset'],
            'geometry_seed_index': (
                990000 + geometry_pair_seed % 10000
            ),
            'geometry_seed_key': (
                f'v3r1:smoke:{bucket}:{presence}:{relation}'
            ),
            'family_avoidance_attempt': 0,
            'source_profile': 'smoke',
            'imported_from_v3': False,
            'source_scenario_id': None,
            'source_manifest_sha256': None,
            'v3r1_manifest_revision': MANIFEST_REVISION,
        }
        spec['spec_id'] = (
            f'v3r1_smoke_{cell_index + 1:04d}_{value_sha256(spec)[:12]}'
        )
        rows.append(spec)
    if len(rows) != SMOKE_COUNT:
        raise VisualV3Error('V3R1 smoke quota must contain exactly 36 rows')
    return rows


def generic_specs(
    profile: str,
    count: int,
    *,
    start_index: int,
    source_start: int = 0,
) -> list[dict[str, Any]]:
    source = build_v3_specs(profile)
    if profile == 'validation':
        indices = [
            round(index * (len(source) - 1) / max(1, count - 1))
            for index in range(count)
        ]
    elif profile == 'canary':
        eligible = [
            index for index, spec in enumerate(source)
            if spec.get('canary_family')
            != 'r4_right_slot3_position_arrival'
        ]
        indices = eligible[:count]
    else:
        indices = list(range(source_start, source_start + count))
    if len(indices) != count or max(indices, default=-1) >= len(source):
        raise VisualV3Error(f'{profile}: insufficient generic source specs')
    rows = []
    for output_index, source_index in enumerate(indices):
        spec = copy.deepcopy(source[source_index])
        spec['profile'] = profile
        spec['generation_index'] = start_index + output_index
        spec['geometry_seed_index'] = (
            500000 + start_index + output_index
        )
        spec['geometry_seed_key'] = (
            f'v3r1:{profile}:generic:{source_index}:{output_index}'
        )
        spec['source_profile'] = profile
        spec['imported_from_v3'] = False
        spec['source_scenario_id'] = None
        spec['source_manifest_sha256'] = None
        spec['v3r1_manifest_revision'] = MANIFEST_REVISION
        spec['target_ratio'] = None
        spec['operational_target_name'] = None
        spec['operational_target_segment'] = None
        spec['presence_class'] = (
            'sparse'
            if len(spec['active_identities']) <= 3
            else 'medium'
            if len(spec['active_identities']) == 4
            else 'dense'
        )
        spec['configuration_variant'] = output_index
        spec['spec_id'] = (
            f'v3r1_{profile}_generic_{output_index + 1:04d}_'
            f'{value_sha256(spec)[:12]}'
        )
        rows.append(spec)
    return rows


def quota_plan(reuse_count: int) -> dict[str, Any]:
    remaining = TRAIN_COUNT - int(reuse_count)
    generic_train = remaining - TRAIN_EXPLICIT_COUNT
    if remaining < TRAIN_EXPLICIT_COUNT:
        raise VisualV3Error(
            f'only {remaining} train slots remain; at least '
            f'{TRAIN_EXPLICIT_COUNT} are required'
        )
    cells = list(itertools.product(
        TARGET_OFFSETS,
        ('empty', 'loaded'),
        ('sparse', 'medium', 'dense'),
        TRAIN_VALIDATION_RELATIONS,
    ))
    plan = {
        'schema_version': PACKAGE_SCHEMA,
        'generator_version': GENERATOR_VERSION,
        'seed': SEED,
        'v3_reuse_count': int(reuse_count),
        'remaining_train_capacity': remaining,
        'capacity_guard_required': TRAIN_EXPLICIT_COUNT,
        'capacity_guard_passed': remaining >= TRAIN_EXPLICIT_COUNT,
        'counts': {
            'train_total': TRAIN_COUNT,
            'train_imported': int(reuse_count),
            'train_explicit_offsets': TRAIN_EXPLICIT_COUNT,
            'train_generic_new': generic_train,
            'validation_total': VALIDATION_COUNT,
            'validation_explicit_offsets': VALIDATION_EXPLICIT_COUNT,
            'validation_generic_new': (
                VALIDATION_COUNT - VALIDATION_EXPLICIT_COUNT
            ),
            'canary_total': CANARY_COUNT,
            'canary_explicit_offsets': CANARY_EXPLICIT_COUNT,
            'canary_generic_new': CANARY_COUNT - CANARY_EXPLICIT_COUNT,
            'correction_smoke': SMOKE_COUNT,
        },
        'operational_target': {
            'name': OPERATIONAL_TARGET_NAME,
            'identity': OPERATIONAL_TARGET_IDENTITY,
            'segment': OPERATIONAL_TARGET_SEGMENT,
            'ratio': OPERATIONAL_TARGET_RATIO,
            'offsets': list(TARGET_OFFSETS),
        },
        'train_valid_cells': [
            {
                'offset': offset,
                'payload': payload,
                'presence_class': presence,
                'relation_family': relation,
                'configuration_families': 2,
            }
            for offset, payload, presence, relation in cells
        ],
        'validation_configuration_families_per_cell': 1,
        'canary_relations': list(CANARY_RELATIONS),
        'unsatisfied_cells': [],
        'passed': True,
    }
    plan['quota_plan_sha256'] = value_sha256(plan)
    return plan


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reuse-count', type=int, required=True)
    parser.add_argument(
        '--output',
        type=Path,
        default=quota_plan_path(),
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    plan = quota_plan(args.reuse_count)
    atomic_json(args.output, plan)
    print(json.dumps(plan, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, VisualV3Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
