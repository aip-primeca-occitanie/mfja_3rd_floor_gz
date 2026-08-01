#!/usr/bin/env python3
"""Shared constants and semantic helpers for Room 315 visual V3R1."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from room_315_visual_scenario_generator import SLOT_RATIOS
from room_315_visual_v3_common import IDENTITIES
from room_315_visual_v3_common import SEED
from room_315_visual_v3_common import TARGET_OFFSETS
from room_315_visual_v3_common import value_sha256


PACKAGE_SCHEMA = 'room315.hard_case_visual_v3r1.v1'
GENERATOR_VERSION = 'room315.visual_v3r1_generator.v1'
MANIFEST_REVISION = 'V3R1'
OPERATIONAL_TARGET_NAME = 'right_slot_3'
OPERATIONAL_TARGET_IDENTITY = 'R4'
OPERATIONAL_TARGET_SEGMENT = 'A34E'
OPERATIONAL_TARGET_RATIO = float(
    SLOT_RATIOS['right'][OPERATIONAL_TARGET_SEGMENT][0]
)

TRAIN_COUNT = 4000
VALIDATION_COUNT = 512
CANARY_COUNT = 256
TRAIN_EXPLICIT_COUNT = 540
VALIDATION_EXPLICIT_COUNT = 270
CANARY_EXPLICIT_COUNT = 108
SMOKE_COUNT = 36

PRESENCE_CARDINALITY = {
    'sparse': 3,
    'medium': 4,
    'dense': 6,
}
TRAIN_VALIDATION_RELATIONS = (
    'no_relation_observation',
    'blocker_ahead_same_segment',
    'nonblocker_behind_same_segment',
    'blocker_intermediate_segment',
    'nonblocker_adjacent_branch',
)
CANARY_RELATIONS = (
    'no_relation_observation',
    'blocker_ahead_same_segment',
    'nonblocker_adjacent_branch',
)
SMOKE_RELATIONS = (
    'no_relation_observation',
    'blocker_ahead_same_segment',
    'blocker_intermediate_segment',
    'nonblocker_adjacent_branch',
)

DEFAULT_V3_CAPTURE_ROOT = Path(
    '/home/tiago/room315_hard_case_visual_v3_capture_seed31520260730'
)
DEFAULT_V3_GUARD_ROOT = Path(
    '/home/tiago/room315_hard_case_visual_v3_guard_seed31520260730'
)
DEFAULT_CAPTURE_ROOT = Path(
    '/home/tiago/room315_hard_case_visual_v3r1_capture_seed31520260730'
)
DEFAULT_SPLIT_ROOT = Path(
    '/home/tiago/room315_hard_case_visual_v3r1_splits_seed31520260730'
)
DEFAULT_CANARY_ROOT = Path(
    '/home/tiago/room315_hard_case_visual_v3r1_canary_seed31520260730'
)
DEFAULT_GUARD_ROOT = Path(
    '/home/tiago/room315_hard_case_visual_v3r1_guard_seed31520260730'
)
QUOTA_PLAN_FILENAME = 'room315_visual_v3r1_quota_plan.json'


def quota_plan_path(guard_root: Path = DEFAULT_GUARD_ROOT) -> Path:
    """Return the single canonical V3R1 quota-plan path."""
    return Path(guard_root) / QUOTA_PLAN_FILENAME


def presence_class(cardinality: int) -> str:
    if 1 <= cardinality <= 3:
        return 'sparse'
    if cardinality == 4:
        return 'medium'
    if 5 <= cardinality <= len(IDENTITIES):
        return 'dense'
    raise ValueError(f'invalid Room 315 presence cardinality: {cardinality}')


def is_deliberate_offset(row: dict[str, Any]) -> bool:
    return (
        row.get('target_identity') == OPERATIONAL_TARGET_IDENTITY
        and row.get('identity_to_block', {}).get(
            OPERATIONAL_TARGET_IDENTITY
        ) == OPERATIONAL_TARGET_SEGMENT
        and row.get('operational_target_name') == OPERATIONAL_TARGET_NAME
        and row.get('operational_target_segment')
        == OPERATIONAL_TARGET_SEGMENT
        and row.get('target_offset') in TARGET_OFFSETS
        and row.get('target_offset_bucket') != 'not_operational_target'
    )


def v3r1_family_payload(
    row: dict[str, Any],
    *,
    include_render: bool,
) -> dict[str, Any]:
    active = tuple(row['active_identities'])
    payload = {
        'active_identities': active,
        'loaded_identities': tuple(row['loaded_identities']),
        'identity_to_block': {
            identity: row['identity_to_block'][identity]
            for identity in active
        },
        'identity_to_position_bin': {
            identity: row['identity_to_position_bin'][identity]
            for identity in active
        },
        'relation_family': row['relation_family'],
        'target_identity': row['target_identity'],
        'target_zone': row['target_zone'],
        'target_offset_bucket': row['target_offset_bucket'],
        'approach_direction': row['approach_direction'],
        'occlusion_class': row['occlusion_class'],
    }
    if include_render:
        payload['render_bucket'] = row['render_variation']['bucket']
    return payload


def v3r1_family_id(
    row: dict[str, Any],
    *,
    include_render: bool = True,
) -> str:
    return 'v3r1_family_' + value_sha256(
        v3r1_family_payload(row, include_render=include_render)
    )
