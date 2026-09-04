#!/usr/bin/env python3
"""Prepare and operate the guarded Room 315 arbitrary-subset capture package.

The preparation path is design-only.  Runtime capture is available only through
explicit, fingerprinted approval gates in the generated package.
"""

from __future__ import annotations

import argparse
import copy
import fcntl
import hashlib
import html
import json
import os
import shutil
import subprocess
import sys
import tempfile
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw


REPOSITORY_ROOT = Path(
    '/home/tiago/mfja_3rd_floor_ros2_ws/src/mfja_3rd_floor_gz'
)
REPOSITORY_SCRIPT_DIR = (
    REPOSITORY_ROOT / 'mfja_robot_control_config' / 'scripts'
)
if str(REPOSITORY_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_SCRIPT_DIR))

from room_315_arbitrary_subset_visual import (  # noqa: E402
    GLOBAL_IDENTITIES,
    NO_RELATION,
    read_jsonl,
    sha256,
    stable_int,
    write_json,
    write_jsonl,
)
from room_315_arbitrary_subset_visual_v2 import (  # noqa: E402
    EXPECTED_RELATION_TOTALS,
    EXPECTED_TOTAL_ACTIVE,
    EXPECTED_ZONE_TOTALS,
    PROTECTED_ROOTS,
    V1_PLAN,
    V2_PLAN_SCHEMA,
    V2_SEED,
    design_v2_audit,
    protected_artifact_audit,
    ratio_matches_zone,
    scenario_projectability,
    tree_fingerprint,
    validate_package_manifest as validate_v2_package_manifest,
    validate_v2_row,
)
from room_315_visual_fleet import (  # noqa: E402
    AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY,
    AUTHORITATIVE_VISUAL_FLEET,
)
from room_315_visual_scenario_generator import (  # noqa: E402
    POSITION_ZONES,
    scenario_physical_conflicts,
    valid_public_segments,
    validate_scenario,
)
from room_315_visual_state_dataset import (  # noqa: E402
    IMAGE_KEYS,
    VISUAL_STATE_SCHEMA_VERSION,
    VisualStateLabelVectorizer,
    camera_observation_for_shuttle,
    canonical_camera_for_identity,
    normalize_visual_state_labels,
    valid_bbox,
)


CAPTURE_PACKAGE_SCHEMA = 'room315.arbitrary_subset_capture_package.v2'
EXECUTABLE_MANIFEST_SCHEMA = 'room315.arbitrary_subset_executable_manifest.v2'
STATIC_AUDIT_SCHEMA = 'room315.arbitrary_subset_production_manifest_audit.v2'
CANARY_SCHEMA = 'room315.arbitrary_subset_production_canary.v2'
CANARY_AUDIT_SCHEMA = 'room315.arbitrary_subset_production_canary_audit.v2'
APPROVAL_SCHEMA = 'room315.arbitrary_subset_production_capture_approval.v2'
CAPTURE_STATE_SCHEMA = 'room315.arbitrary_subset_capture_state.v2'
CAPTURE_STATUS_SCHEMA = 'room315.arbitrary_subset_capture_status.v2'
CAPTURE_AUDIT_SCHEMA = 'room315.arbitrary_subset_captured_audit.v2'
CAMERA_BBOX_AUDIT_SCHEMA = 'room315.camera_bbox_semantics_audit.v1'
CAMERA_BBOX_CORRECTION_SCHEMA = 'room315.camera_bbox_semantics_correction.v1'
EXPECTED_SCENARIOS = 2040
EXPECTED_IMAGES = 4080
CANARY_COUNT = 64
V2_ROOT = Path(
    '/home/tiago/room315_arbitrary_subset_visual_2040_v2_seed31520260729'
)
V2_PLAN = V2_ROOT / 'configuration_variant_plan_v2.jsonl'
EXPECTED_V2_SHA256 = (
    '23cb73bf1c98ded0a21ee74314e4d448bd7525f65fd2afcdf23f309179ce17c1'
)
DEFAULT_OUTPUT = Path(
    '/home/tiago/'
    'room315_arbitrary_subset_visual_2040_capture_v2_seed31520260729'
)
CAMERAS = ('left_rail_rgb', 'right_rail_rgb')
if CAMERAS != IMAGE_KEYS:
    raise RuntimeError('capture cameras and visual-state cameras disagree')
CAMERA_TOPICS = {
    'left_rail_rgb': '/room_315/perception/left_rail_rgbd/image',
    'right_rail_rgb': '/room_315/perception/right_rail_rgbd/image',
}
APPROVAL_FIELDS = (
    'approved_for_canary_capture',
    'approved_after_canary_gallery_review',
    'approved_for_full_capture',
    'approved_after_full_gallery_review',
    'approved_for_training',
)
CANARY_REVIEW_FINGERPRINT_FILES = {
    'captured_canary_audit_sha256': 'captured_canary_audit.json',
    'camera_bbox_semantics_audit_sha256': (
        'camera_bbox_semantics_audit.json'
    ),
    'canary_gallery_manifest_sha256': 'canary_gallery_manifest.json',
    'canary_gallery_html_sha256': 'canary_gallery.html',
}
CANARY_REVIEW_OVERLAY_FINGERPRINT_FIELD = (
    'canary_gallery_overlay_set_sha256'
)
FULL_REVIEW_FINGERPRINT_FILES = {
    'captured_production_audit_sha256': (
        'captured_production_audit.json'
    ),
    'production_camera_bbox_semantics_audit_sha256': (
        'production_camera_bbox_semantics_audit.json'
    ),
    'production_review_gallery_manifest_sha256': (
        'production_review_gallery_manifest.json'
    ),
    'production_review_gallery_html_sha256': (
        'production_review_gallery.html'
    ),
}
FULL_REVIEW_OVERLAY_FINGERPRINT_FIELD = (
    'production_review_gallery_overlay_set_sha256'
)
STATIC_PACKAGE_FILES = (
    'README.md',
    'audit_captured_canary.sh',
    'audit_captured_production.sh',
    'audit_production_manifest.sh',
    'capture_canary.sh',
    'capture_full.sh',
    'capture_status.py',
    'create_canary_gallery.py',
    'create_production_review_gallery.py',
    'generate_production_manifest.sh',
    'production_canary_audit.json',
    'production_canary_audit.md',
    'production_canary_scenario_ids.json',
    'production_manifest_audit.json',
    'production_manifest_audit.md',
    'resume_capture.sh',
    'scenario_manifest.jsonl',
    'validate_canary_gallery_approval.py',
    'validate_full_gallery_approval.py',
    'validate_production_approval.py',
)
MUTABLE_PACKAGE_FILES = (
    'capture_state.json',
    'production_capture_approval.json',
)
FORBIDDEN_INITIAL_FILES = (
    'captured_canary_audit.json',
    'captured_production_audit.json',
    'canary_gallery.html',
    'canary_gallery_manifest.json',
    'production_review_gallery.html',
    'production_review_gallery_manifest.json',
)
GATE_EXIT_CODES = {
    'manifest': 10,
    'canary': 11,
    'canary_capture_incomplete': 12,
    'canary-gallery': 13,
    'full-capture': 14,
    'full_capture_incomplete': 15,
    'full-gallery': 16,
    'training': 17,
    'fingerprint': 18,
}


class CapturePackageError(ValueError):
    """Raised when the guarded production package contract is violated."""


def _canonical(value: Any) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    )


def _atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{path.name}.',
        suffix='.tmp',
        dir=path.parent,
    )
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def _atomic_json(path: Path, value: Any) -> None:
    _atomic_text(
        path,
        json.dumps(value, indent=2, sort_keys=True) + '\n',
    )


def _atomic_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    _atomic_text(
        path,
        ''.join(_canonical(row) + '\n' for row in rows),
    )


def _load_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise CapturePackageError(f'cannot read JSON object: {path}') from exc
    if not isinstance(value, dict):
        raise CapturePackageError(f'JSON value is not an object: {path}')
    return value


def _identity_side(identity: str) -> str:
    return 'left' if identity.startswith('L') else 'right'


def _entity(identity: str) -> str:
    return AUTHORITATIVE_VISUAL_FLEET['world_entities'][identity]


def _active_ids(row: dict[str, Any]) -> list[str]:
    return (
        list(row['left_active_identities'])
        + list(row['right_active_identities'])
    )


def _row_sha256(row: dict[str, Any]) -> str:
    return hashlib.sha256(_canonical(row).encode('utf-8')).hexdigest()


def executable_manifest_rows(
    v2_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Add runtime expectations without altering any v2 semantics/geometry."""
    rows = []
    all_identities = set(GLOBAL_IDENTITIES)
    for index, source in enumerate(v2_rows):
        row = copy.deepcopy(source)
        active = _active_ids(source)
        inactive = [
            identity
            for identity in GLOBAL_IDENTITIES
            if identity not in active
        ]
        positions = {
            shuttle['id']: copy.deepcopy(shuttle['start_position'])
            for side in ('left', 'right')
            for shuttle in source['scene']['rails'][side]['shuttles']
        }
        row.update({
            'executable_manifest_schema': EXECUTABLE_MANIFEST_SCHEMA,
            'v2_plan_id': source['plan_id'],
            'v2_design_row_index': index,
            'v2_design_row_sha256': _row_sha256(source),
            'v2_design_source': {
                'path': str(V2_PLAN),
                'sha256': EXPECTED_V2_SHA256,
            },
            'output_episode_id': source['scenario_id'],
            'active_gazebo_entities': {
                identity: _entity(identity)
                for identity in active
            },
            'inactive_identities': inactive,
            'hidden_gazebo_entities_expected': {
                identity: _entity(identity)
                for identity in inactive
            },
            'camera_topics': dict(CAMERA_TOPICS),
            'oracle_expectations': {
                'visual_state_schema': VISUAL_STATE_SCHEMA_VERSION,
                'fixed_identity_order': list(GLOBAL_IDENTITIES),
                'fixed_vector_dimension': VisualStateLabelVectorizer().dim,
                'exact_present_identities': active,
                'exact_absent_identities': inactive,
                'exact_gazebo_entities': {
                    identity: _entity(identity)
                    for identity in GLOBAL_IDENTITIES
                },
                'payload_assignment': copy.deepcopy(
                    source['payload_assignment']
                ),
                'positions': positions,
                'target_identity': source['target_identity'],
                'relation_family': source['relation_family'],
                'relation_identities': copy.deepcopy(
                    source['relation_identities']
                ),
                'relation_metadata_model_input': False,
                'required_cameras': list(CAMERAS),
                'expected_source_image_count': 2,
            },
        })
        if (
            set(row['active_gazebo_entities'])
            | set(row['hidden_gazebo_entities_expected'])
        ) != all_identities:
            raise CapturePackageError(
                f'{row["plan_id"]}: entity inventory is incomplete'
            )
        rows.append(row)
    return rows


def _semantic_equality_errors(
    source: dict[str, Any],
    executable: dict[str, Any],
) -> list[str]:
    exact_fields = (
        'source_v1_plan_id',
        'plan_id',
        'configuration_id',
        'presence_configuration_id',
        'presence_bitmask',
        'canonical_subset_key',
        'left_active_identities',
        'right_active_identities',
        'target_identity',
        'relation_family',
        'relation_identities',
        'relation_neutral_identities',
        'opposite_rail_distractor_identities',
        'payload_assignment',
        'target_zone',
        'target_segment',
        'target_s_m',
        'target_s_ratio',
        'target_segment_length_m',
        'geometry_key',
        'planned_partition_role',
        'relation_metadata_model_input',
        'scene',
        'setup',
        'static_camera_projectability',
    )
    errors = [
        field
        for field in exact_fields
        if executable.get(field) != source.get(field)
    ]
    if executable.get('v2_plan_id') != source.get('plan_id'):
        errors.append('v2_plan_id')
    if executable.get('v2_design_row_sha256') != _row_sha256(source):
        errors.append('v2_design_row_sha256')
    if executable.get('output_episode_id') != source.get('scenario_id'):
        errors.append('output_episode_id')
    return errors


def static_manifest_audit(
    v2_rows: list[dict[str, Any]],
    rows: list[dict[str, Any]],
) -> dict[str, Any]:
    if len(v2_rows) != len(rows):
        raise CapturePackageError('v2/executable manifest row count mismatch')
    equality = {}
    full_validation = {}
    projectability = {}
    physical_conflicts = {}
    invalid_entities = []
    representability = []
    duplicate_identities = []
    wrong_side_identities = []
    inactive_targets = []
    inactive_relation_participants = []
    prefix_substitutions = []
    invalid_segments = []
    invalid_s_m = []
    invalid_s_ratio = []
    invalid_segment_lengths = []
    invalid_target_zones = []
    config_counts = Counter()
    cardinality_pairs = Counter()
    total_active = Counter()
    presence = Counter()
    absence = Counter()
    loaded = Counter()
    empty = Counter()
    alone = Counter()
    relations = Counter()
    zones = Counter()
    segments = Counter()
    roles = Counter()
    future_roles = Counter()
    geometry_by_config: dict[str, set[str]] = defaultdict(set)
    scenario_ids = []
    vectorizer = VisualStateLabelVectorizer()
    for source, row in zip(v2_rows, rows):
        plan_id = source['plan_id']
        semantic = _semantic_equality_errors(source, row)
        if semantic:
            equality[plan_id] = semantic
        errors = validate_v2_row(row, source)
        try:
            validate_scenario(row)
        except ValueError as exc:
            errors.append(f'validate_scenario:{exc}')
        conflicts = scenario_physical_conflicts(row)
        if conflicts:
            physical_conflicts[plan_id] = conflicts
            errors.append(f'physical_conflicts:{len(conflicts)}')
        projected = scenario_projectability(row)
        if not projected['passed']:
            projectability[plan_id] = projected
            errors.append('projectability')
        if errors:
            full_validation[plan_id] = errors
        active = _active_ids(row)
        active_set = set(active)
        actual_by_side = {
            side: [
                shuttle['id']
                for shuttle in row['scene']['rails'][side]['shuttles']
            ]
            for side in ('left', 'right')
        }
        if len(active) != len(active_set):
            duplicate_identities.append(plan_id)
        for side, identities in actual_by_side.items():
            if len(identities) != len(set(identities)):
                duplicate_identities.append(plan_id)
            for identity in identities:
                if _identity_side(identity) != side:
                    wrong_side_identities.append(
                        f'{plan_id}:{side}:{identity}'
                    )
            if identities != row[f'{side}_active_identities']:
                prefix_substitutions.append(f'{plan_id}:{side}:manifest')
            if identities != source[f'{side}_active_identities']:
                prefix_substitutions.append(f'{plan_id}:{side}:v2')
        if row['target_identity'] not in active_set:
            inactive_targets.append(plan_id)
        if not set(row['relation_identities']) <= active_set:
            inactive_relation_participants.append(plan_id)
        expected_entities = {
            identity: _entity(identity)
            for identity in active
        }
        if row.get('active_gazebo_entities') != expected_entities:
            invalid_entities.append(plan_id)
        for identity in active:
            shuttle = next(
                item
                for side in ('left', 'right')
                for item in row['scene']['rails'][side]['shuttles']
                if item['id'] == identity
            )
            position = shuttle['start_position']
            side = _identity_side(identity)
            block = position['segment']
            if block not in AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY:
                representability.append(
                    f'{row["scenario_id"]}:{identity}:{block}'
                )
            if block not in valid_public_segments(side):
                invalid_segments.append(
                    f'{row["scenario_id"]}:{identity}:{block}'
                )
            try:
                length = float(position['segment_length_m'])
                s_m = float(position['s_m'])
                ratio = float(position['s_ratio'])
            except (KeyError, TypeError, ValueError):
                invalid_segment_lengths.append(
                    f'{row["scenario_id"]}:{identity}:non_numeric'
                )
                continue
            if length <= 0.0:
                invalid_segment_lengths.append(
                    f'{row["scenario_id"]}:{identity}:{length}'
                )
            if not 0.0 <= ratio <= 1.0:
                invalid_s_ratio.append(
                    f'{row["scenario_id"]}:{identity}:{ratio}'
                )
            if (
                length <= 0.0
                or not 0.0 <= s_m <= length + 1e-9
                or abs(s_m - ratio * length) > 1e-6
            ):
                invalid_s_m.append(
                    f'{row["scenario_id"]}:{identity}:{s_m}'
                )
            if (
                identity == row['target_identity']
                and not ratio_matches_zone(
                    side,
                    block,
                    row['target_zone'],
                    ratio,
                )
            ):
                invalid_target_zones.append(row['scenario_id'])
        scenario_ids.append(row['scenario_id'])
        config = row['configuration_id']
        config_counts[config] += 1
        left_count = len(row['left_active_identities'])
        right_count = len(row['right_active_identities'])
        cardinality_pairs[f'{left_count}+{right_count}'] += 1
        total_active[str(len(active))] += 1
        if len(active) == 1:
            alone[active[0]] += 1
        for identity in GLOBAL_IDENTITIES:
            if identity in active_set:
                presence[identity] += 1
            else:
                absence[identity] += 1
        for identity, state in row['payload_assignment'].items():
            (loaded if state == 'loaded' else empty)[identity] += 1
        relations[row['relation_family']] += 1
        zones[row['target_zone']] += 1
        for side in ('left', 'right'):
            for shuttle in row['scene']['rails'][side]['shuttles']:
                segments[
                    f'{side}:{shuttle["start_position"]["segment"]}'
                ] += 1
        roles[f'{row["target_identity"]}:target'] += 1
        relation_role = (
            'non_blocker'
            if row['relation_family'] in {
                'nonblocker_behind_same_segment',
                'nonblocker_adjacent_branch',
            }
            else 'blocker'
        )
        for identity in row['relation_identities']:
            roles[f'{identity}:{relation_role}'] += 1
        for identity in (
            row['relation_neutral_identities']
            + row['opposite_rail_distractor_identities']
        ):
            roles[f'{identity}:relation_neutral'] += 1
        future_roles[row['planned_partition_role']] += 1
        geometry_by_config[config].add(row['geometry_key'])

    expected_pairs = {
        f'{left}+{right}'
        for left in range(5)
        for right in range(5)
        if left or right
    }
    checks = {
        'exactly_2040_rows': len(rows) == EXPECTED_SCENARIOS,
        'exactly_2040_unique_scenario_ids': (
            len(scenario_ids) == len(set(scenario_ids)) == EXPECTED_SCENARIOS
        ),
        'exactly_255_configurations': len(config_counts) == 255,
        'exactly_8_variants_per_configuration': all(
            value == 8 for value in config_counts.values()
        ),
        'no_all_empty_scenes': all(
            row['left_active_identities']
            or row['right_active_identities']
            for row in rows
        ),
        'all_24_cardinality_pairs': set(cardinality_pairs) == expected_pairs,
        'exact_total_active_distribution': (
            dict(sorted(total_active.items())) == EXPECTED_TOTAL_ACTIVE
        ),
        'every_identity_present_1024': all(
            presence[identity] == 1024 for identity in GLOBAL_IDENTITIES
        ),
        'every_identity_absent_1016': all(
            absence[identity] == 1016 for identity in GLOBAL_IDENTITIES
        ),
        'every_identity_loaded_512': all(
            loaded[identity] == 512 for identity in GLOBAL_IDENTITIES
        ),
        'every_identity_empty_512': all(
            empty[identity] == 512 for identity in GLOBAL_IDENTITIES
        ),
        'every_identity_alone_8': all(
            alone[identity] == 8 for identity in GLOBAL_IDENTITIES
        ),
        'exact_relation_totals': (
            dict(sorted(relations.items())) == EXPECTED_RELATION_TOTALS
        ),
        'exact_target_zone_totals': (
            dict(sorted(zones.items())) == EXPECTED_ZONE_TOTALS
        ),
        'all_14_public_segments_both_rails': all(
            segments[f'{side}:{segment}'] > 0
            for side in ('left', 'right')
            for segment in valid_public_segments(side)
        ),
        'row_by_row_v2_semantic_equality': not equality,
        'zero_duplicate_identities': not duplicate_identities,
        'zero_wrong_side_identities': not wrong_side_identities,
        'zero_inactive_targets': not inactive_targets,
        'zero_inactive_relation_participants': (
            not inactive_relation_participants
        ),
        'zero_prefix_substitutions': not prefix_substitutions,
        'zero_physical_conflicts': not physical_conflicts,
        'zero_pairwise_separation_violations': not physical_conflicts,
        'zero_topology_violations': not full_validation,
        'zero_switch_routing_violations': not full_validation,
        'zero_relation_violations': not full_validation,
        'zero_zone_segment_violations': (
            not invalid_segments and not invalid_target_zones
        ),
        'zero_invalid_s_m_values': not invalid_s_m,
        'zero_invalid_s_ratio_values': not invalid_s_ratio,
        'zero_invalid_segment_lengths': not invalid_segment_lengths,
        'zero_full_validation_violations': not full_validation,
        'zero_projectability_violations': not projectability,
        'zero_entity_inventory_violations': not invalid_entities,
        'zero_representability_violations': not representability,
        'eight_unique_geometries_per_configuration': all(
            len(values) == 8
            for values in geometry_by_config.values()
        ),
        'future_partition_metadata_1530_255_255': (
            future_roles == Counter({
                'future_train': 1530,
                'future_validation': 255,
                'future_blind_test': 255,
            })
        ),
        'future_partition_six_one_one_per_configuration': all(
            Counter(
                row['planned_partition_role']
                for row in rows
                if row['configuration_id'] == config
            ) == Counter({
                'future_train': 6,
                'future_validation': 1,
                'future_blind_test': 1,
            })
            for config in config_counts
        ),
        'fixed_schema_v3': (
            VISUAL_STATE_SCHEMA_VERSION == 'room315.visual_state.v3'
        ),
        'fixed_vector_dimension_200': vectorizer.dim == 200,
        'dataset_inferred_capacity_false': (
            vectorizer.to_json()['capacity_inferred_from_dataset'] is False
        ),
    }
    return {
        'schema_version': STATIC_AUDIT_SCHEMA,
        'passed': all(checks.values()),
        'checks': checks,
        'v2_source': {
            'path': str(V2_PLAN),
            'sha256': sha256(V2_PLAN),
        },
        'scenario_count': len(rows),
        'expected_image_count': len(rows) * len(CAMERAS),
        'distributions': {
            'configuration_variant_count': dict(sorted(config_counts.items())),
            'cardinality_pair': dict(sorted(cardinality_pairs.items())),
            'total_active_count': dict(sorted(total_active.items())),
            'identity_presence': dict(sorted(presence.items())),
            'identity_absence': dict(sorted(absence.items())),
            'identity_loaded': dict(sorted(loaded.items())),
            'identity_empty': dict(sorted(empty.items())),
            'identity_alone': dict(sorted(alone.items())),
            'identity_roles': dict(sorted(roles.items())),
            'relation_family': dict(sorted(relations.items())),
            'target_zone': dict(sorted(zones.items())),
            'all_active_segments': dict(sorted(segments.items())),
            'future_partition_role': dict(sorted(future_roles.items())),
        },
        'violations': {
            'semantic_equality': equality,
            'full_validation': full_validation,
            'projectability': projectability,
            'physical_conflicts': physical_conflicts,
            'entity_inventory': invalid_entities,
            'representability': representability,
            'duplicate_identities': sorted(set(duplicate_identities)),
            'wrong_side_identities': wrong_side_identities,
            'inactive_targets': inactive_targets,
            'inactive_relation_participants': (
                inactive_relation_participants
            ),
            'prefix_substitutions': prefix_substitutions,
            'invalid_segments': invalid_segments,
            'invalid_s_m': invalid_s_m,
            'invalid_s_ratio': invalid_s_ratio,
            'invalid_segment_lengths': invalid_segment_lengths,
            'invalid_target_zones': invalid_target_zones,
        },
        'fixed_schema': {
            'visual_state_schema': VISUAL_STATE_SCHEMA_VERSION,
            'identity_order': list(GLOBAL_IDENTITIES),
            'vectorizer_dimension': vectorizer.dim,
            'dataset_inferred_capacity': vectorizer.to_json()[
                'capacity_inferred_from_dataset'
            ],
        },
        'capture_executed': False,
    }


def _canary_tokens(row: dict[str, Any]) -> set[str]:
    left = tuple(row['left_active_identities'])
    right = tuple(row['right_active_identities'])
    active = left + right
    tokens = {
        f'pair:{len(left)}+{len(right)}',
        f'total:{len(active)}',
        f'relation:{row["relation_family"]}',
        f'zone:{row["target_zone"]}',
    }
    if not left:
        tokens.add('left_empty')
    if not right:
        tokens.add('right_empty')
    if left and right:
        tokens.add('both_rails')
    if len(active) <= 3:
        tokens.add('sparse')
    if len(active) >= 7:
        tokens.add('dense')
    if len(active) == 8:
        tokens.add('all_eight')
    if row['target_zone'] == 'switch':
        tokens.add('switch_heavy')
    if row['target_zone'] == 'slot':
        tokens.add('slot_scene')
    if row['target_zone'] == 'merge_conflict':
        tokens.add('merge_scene')
    if row['target_zone'] == 'boundary':
        tokens.add('boundary_scene')
    for identity in active:
        tokens.add(f'present:{identity}')
        tokens.add(
            f'payload:{identity}:{row["payload_assignment"][identity]}'
        )
    if len(active) == 1:
        tokens.add(f'singleton:{active[0]}')
    for side in ('left', 'right'):
        for shuttle in row['scene']['rails'][side]['shuttles']:
            tokens.add(
                f'segment:{side}:{shuttle["start_position"]["segment"]}'
            )
    exact = {
        (('L3',), ()): 'required:L3_only',
        (('L2', 'L4'), ()): 'required:L2_L4',
        ((), ('R4',)): 'required:R4_only',
        ((), ('R1', 'R3')): 'required:R1_R3',
        (('L1', 'L4'), ('R2',)): 'required:L1_L4_R2',
        (('L2',), ('R1', 'R4')): 'required:L2_R1_R4',
        (('L2', 'L4'), ('R2', 'R3')): 'required:L2_L4_R2_R3',
        (
            ('L1', 'L2', 'L3', 'L4'),
            ('R1', 'R2', 'R3', 'R4'),
        ): 'required:all_eight_subset',
    }
    if (left, right) in exact:
        tokens.add(exact[(left, right)])
    active_set = set(active)
    for pair in (('R4', 'L3'), ('L1', 'R2')):
        if set(pair) <= active_set:
            tokens.add(f'color_risk:{"+".join(pair)}')
    if 'L4' in active_set:
        tokens.add('color_risk:L4_bright')
    if row['static_camera_projectability'][
        'partial_occlusion_risk_pairs'
    ]:
        tokens.add('partial_occlusion_risk')
    return tokens


def _required_canary_tokens(rows: list[dict[str, Any]]) -> set[str]:
    tokens = {
        *(f'pair:{left}+{right}' for left in range(5) for right in range(5)
          if left or right),
        *(f'total:{count}' for count in range(1, 9)),
        *(f'relation:{family}' for family in EXPECTED_RELATION_TOTALS),
        *(f'zone:{zone}' for zone in EXPECTED_ZONE_TOTALS),
        *(f'singleton:{identity}' for identity in GLOBAL_IDENTITIES),
        *(f'payload:{identity}:{state}'
          for identity in GLOBAL_IDENTITIES for state in ('loaded', 'empty')),
        *(f'segment:{side}:{segment}'
          for side in ('left', 'right')
          for segment in valid_public_segments(side)),
        'left_empty',
        'right_empty',
        'both_rails',
        'sparse',
        'dense',
        'all_eight',
        'switch_heavy',
        'slot_scene',
        'merge_scene',
        'boundary_scene',
        'required:L3_only',
        'required:L2_L4',
        'required:R4_only',
        'required:R1_R3',
        'required:L1_L4_R2',
        'required:L2_R1_R4',
        'required:L2_L4_R2_R3',
        'required:all_eight_subset',
        'color_risk:R4+L3',
        'color_risk:L1+R2',
        'color_risk:L4_bright',
        'partial_occlusion_risk',
    }
    available = set().union(*(_canary_tokens(row) for row in rows))
    missing = tokens - available
    if missing:
        raise CapturePackageError(
            f'canary requirements are absent from production manifest: '
            f'{sorted(missing)}'
        )
    return tokens


def select_canary(rows: list[dict[str, Any]]) -> list[str]:
    required = _required_canary_tokens(rows)
    uncovered = set(required)
    selected: list[dict[str, Any]] = []
    remaining = list(rows)
    while uncovered and len(selected) < CANARY_COUNT:
        ranked = sorted(
            remaining,
            key=lambda row: (
                -len(_canary_tokens(row) & uncovered),
                stable_int(V2_SEED, row['scenario_id'], 'canary'),
                row['scenario_id'],
            ),
        )
        chosen = ranked[0]
        gain = _canary_tokens(chosen) & uncovered
        if not gain:
            break
        selected.append(chosen)
        remaining.remove(chosen)
        uncovered -= gain
    if uncovered:
        raise CapturePackageError(
            f'64-scenario canary cannot satisfy coverage: '
            f'{sorted(uncovered)}'
        )
    while len(selected) < CANARY_COUNT:
        covered_configs = Counter(
            row['configuration_id'] for row in selected
        )
        covered_geometry = {
            row['geometry_key'] for row in selected
        }
        chosen = min(
            remaining,
            key=lambda row: (
                covered_configs[row['configuration_id']],
                row['geometry_key'] in covered_geometry,
                stable_int(V2_SEED, row['scenario_id'], len(selected)),
                row['scenario_id'],
            ),
        )
        selected.append(chosen)
        remaining.remove(chosen)
    return [row['scenario_id'] for row in selected]


def canary_audit(
    rows: list[dict[str, Any]],
    canary_ids: list[str],
) -> dict[str, Any]:
    by_id = {row['scenario_id']: row for row in rows}
    selected = [by_id[scenario_id] for scenario_id in canary_ids if scenario_id in by_id]
    covered = set().union(*(_canary_tokens(row) for row in selected))
    required = _required_canary_tokens(rows)
    missing = sorted(required - covered)
    checks = {
        'exactly_64_ids': len(canary_ids) == CANARY_COUNT,
        'all_ids_unique': len(canary_ids) == len(set(canary_ids)),
        'strict_subset_of_production_manifest': (
            set(canary_ids) < set(by_id)
        ),
        'all_ids_in_production_manifest': set(canary_ids) <= set(by_id),
        'all_required_coverage_tokens': not missing,
    }
    distributions = {
        'cardinality_pair': dict(sorted(Counter(
            f'{len(row["left_active_identities"])}+'
            f'{len(row["right_active_identities"])}'
            for row in selected
        ).items())),
        'total_active_count': dict(sorted(Counter(
            str(len(_active_ids(row))) for row in selected
        ).items())),
        'relation_family': dict(sorted(Counter(
            row['relation_family'] for row in selected
        ).items())),
        'target_zone': dict(sorted(Counter(
            row['target_zone'] for row in selected
        ).items())),
    }
    return {
        'schema_version': CANARY_AUDIT_SCHEMA,
        'passed': all(checks.values()),
        'checks': checks,
        'scenario_count': len(selected),
        'required_token_count': len(required),
        'covered_token_count': len(required & covered),
        'missing_required_tokens': missing,
        'covered_requirements': sorted(required & covered),
        'distributions': distributions,
        'capture_executed': False,
    }


def _static_audit_markdown(audit: dict[str, Any]) -> str:
    failed = [key for key, passed in audit['checks'].items() if not passed]
    return f'''# Room 315 production executable-manifest audit

Verdict: **{"PASS" if audit["passed"] else "FAIL"}**

- Scenarios: {audit["scenario_count"]}
- Expected RGB images: {audit["expected_image_count"]}
- Failed checks: {", ".join(failed) if failed else "none"}
- Semantic-equality violations: {len(audit["violations"]["semantic_equality"])}
- Full validator violations: {len(audit["violations"]["full_validation"])}
- Projectability violations: {len(audit["violations"]["projectability"])}
- Representability violations: {len(audit["violations"]["representability"])}

This is a static design audit. Gazebo capture has not run.
'''


def _canary_markdown(audit: dict[str, Any]) -> str:
    return f'''# Room 315 64-scenario production canary audit

Verdict: **{"PASS" if audit["passed"] else "FAIL"}**

- Unique production scenarios: {audit["scenario_count"]}
- Required coverage tokens: {audit["required_token_count"]}
- Covered coverage tokens: {audit["covered_token_count"]}
- Missing tokens: {", ".join(audit["missing_required_tokens"]) or "none"}

The canary is a strict ID subset of the 2040-row executable manifest. No
scenario row or alternate geometry is duplicated.
'''


def _approval_template(
    manifest_sha: str,
    canary_sha: str,
) -> dict[str, Any]:
    return {
        'schema_version': APPROVAL_SCHEMA,
        'package_schema_version': CAPTURE_PACKAGE_SCHEMA,
        'production_manifest_sha256': manifest_sha,
        'canary_id_list_sha256': canary_sha,
        'v2_source_sha256': EXPECTED_V2_SHA256,
        **{field: False for field in APPROVAL_FIELDS},
        'canary_reviewer': '',
        'canary_reviewed_at': '',
        'canary_review_notes': '',
        'canary_gallery_reviewer': '',
        'canary_gallery_reviewed_at': '',
        'canary_gallery_review_notes': '',
        'full_capture_reviewer': '',
        'full_capture_reviewed_at': '',
        'full_capture_review_notes': '',
        'full_gallery_reviewer': '',
        'full_gallery_reviewed_at': '',
        'full_gallery_review_notes': '',
        'training_reviewer': '',
        'training_reviewed_at': '',
        'training_review_notes': '',
        'notes': (
            'All approvals start false. Enable only the next documented gate '
            'after completing the preceding audit and human review.'
        ),
    }


def _initial_capture_state(manifest_sha: str) -> dict[str, Any]:
    return {
        'schema_version': CAPTURE_STATE_SCHEMA,
        'capture_has_started': False,
        'capture_complete': False,
        'expected_scenario_count': EXPECTED_SCENARIOS,
        'expected_image_count': EXPECTED_IMAGES,
        'completed_scenarios': [],
        'current_scenario': None,
        'historical_failures': [],
        'unresolved_failures': [],
        'captured_scenario_count': 0,
        'valid_image_count': 0,
        'missing_image_count': EXPECTED_IMAGES,
        'exact_subset_validation': None,
        'manifest_sha256': manifest_sha,
        'canary_scenario_count': CANARY_COUNT,
        'canary_completed_count': 0,
        'canary_complete': False,
    }


def _script_sources(package_root: Path) -> dict[str, str]:
    root = str(package_root)
    py = str(package_root / 'capture_status.py')
    approval = str(package_root / 'validate_production_approval.py')
    common = '#!/usr/bin/env bash\nset -euo pipefail\n'
    return {
        'generate_production_manifest.sh': (
            common
            + f'exec python3 {py!r} validate-static --package-root {root!r}\n'
        ),
        'audit_production_manifest.sh': (
            common
            + f'exec python3 {py!r} validate-static --package-root {root!r}\n'
        ),
        'capture_canary.sh': (
            common
            + f'python3 {approval!r} --require canary\n'
            + f'exec python3 {py!r} capture --package-root {root!r} --mode canary "$@"\n'
        ),
        'capture_full.sh': (
            common
            + f'python3 {approval!r} --require full-capture\n'
            + f'exec python3 {py!r} capture --package-root {root!r} --mode full "$@"\n'
        ),
        'resume_capture.sh': (
            common
            + f'exec python3 {py!r} resume --package-root {root!r} "$@"\n'
        ),
        'audit_captured_canary.sh': (
            common
            + f'exec python3 {py!r} audit-captured --package-root {root!r} '
            '--scope canary --output '
            + repr(str(package_root / 'captured_canary_audit.json'))
            + '\n'
        ),
        'audit_captured_production.sh': (
            common
            + f'exec python3 {py!r} audit-captured --package-root {root!r} '
            '--scope full --output '
            + repr(str(package_root / 'captured_production_audit.json'))
            + '\n'
        ),
        'create_canary_gallery.py': (
            '#!/usr/bin/env python3\n'
            'import subprocess,sys\n'
            f'raise SystemExit(subprocess.call([sys.executable,{py!r},'
            f'\"gallery\",\"--package-root\",{root!r},\"--scope\",\"canary\"]))\n'
        ),
        'create_production_review_gallery.py': (
            '#!/usr/bin/env python3\n'
            'import subprocess,sys\n'
            f'raise SystemExit(subprocess.call([sys.executable,{py!r},'
            f'\"gallery\",\"--package-root\",{root!r},\"--scope\",\"full\"]))\n'
        ),
        'validate_canary_gallery_approval.py': (
            '#!/usr/bin/env python3\n'
            'import subprocess,sys\n'
            f'raise SystemExit(subprocess.call([sys.executable,{approval!r},'
            '\"--require\",\"canary-gallery\"]))\n'
        ),
        'validate_full_gallery_approval.py': (
            '#!/usr/bin/env python3\n'
            'import subprocess,sys\n'
            f'raise SystemExit(subprocess.call([sys.executable,{approval!r},'
            '\"--require\",\"full-gallery\"]))\n'
        ),
        'validate_production_approval.py': (
            '#!/usr/bin/env python3\n'
            'import importlib.util,pathlib,sys\n'
            f'_p=pathlib.Path({py!r})\n'
            '_s=importlib.util.spec_from_file_location("room315_capture_status",_p)\n'
            '_m=importlib.util.module_from_spec(_s);_s.loader.exec_module(_m)\n'
            f'raise SystemExit(_m.approval_cli(pathlib.Path({root!r}),sys.argv[1:]))\n'
        ),
    }


def _readme(package_root: Path) -> str:
    return f'''# Room 315 arbitrary-subset guarded production capture v2

This package contains 2040 executable scenarios derived byte-for-byte in
semantics and geometry from `{V2_PLAN}` (`{EXPECTED_V2_SHA256}`).

Capture and training are initially blocked. Do not change more than the next
documented approval field.

## Workflow

```bash
cd {package_root}

# 1. Static audit
./audit_production_manifest.sh

# 2. Inspect production_manifest_audit.json and production_canary_audit.json.
# 3. Enable canary capture only (replace the reviewer and notes).
./validate_production_approval.py --enable-canary \\
  --reviewer "YOUR_NAME" --notes "Reviewed static production and canary audits"

# 4. Validate the canary gate.
./validate_production_approval.py --require canary

# 5. Capture the exact reusable 64-scenario canary.
./capture_canary.sh

# 6. Monitor or resume.
./capture_status.py
./resume_capture.sh

# 7. Audit the captured canary and create its gallery.
./audit_captured_canary.sh
./create_canary_gallery.py

# The gallery renderer is camera-specific:
# left_rail_rgb -> L1-L4 only; right_rail_rgb -> R1-R4 only.
# Fail-closed audit (the relabel command is a one-time migration for captures
# made before explicit camera_observations were stored):
./capture_status.py audit-camera-bboxes --scope canary --require-gallery \
  --output-json camera_bbox_semantics_audit.json \
  --output-md camera_bbox_semantics_audit.md

# 8. Human-review canary_gallery.html, edit the single authoritative
#    approval through the fingerprint-pinned CLI.
./validate_production_approval.py --approve-canary-gallery \
  --reviewer "YOUR_NAME" --notes "Reviewed corrected canary gallery"
./validate_canary_gallery_approval.py

# 9. Enable full capture only after that gate, then validate.
./validate_production_approval.py --enable-full-capture \
  --reviewer "YOUR_NAME" --notes "Authorized remaining production capture"
./validate_production_approval.py --require full-capture
./capture_full.sh

# 10. Monitor/resume, audit the full dataset, and build the stratified gallery.
./capture_status.py
./resume_capture.sh
./audit_captured_production.sh
./create_production_review_gallery.py

# 11. After human review set approved_after_full_gallery_review=true.
./validate_full_gallery_approval.py
```

`approved_for_training` must remain false during this phase. Planned future
train/validation/blind-test roles are metadata only; no split files exist.

## Approval exit codes

{json.dumps(GATE_EXIT_CODES, indent=2, sort_keys=True)}

Episodes are committed through a temporary directory and atomic rename by the
validated recorder. The wrapper locks capture, validates every completed
episode, rebuilds aggregate JSONL atomically, updates capture state atomically,
never overwrites a valid episode, and reuses canary episodes during full
capture.
'''


def _package_manifest(root: Path, declared_root: Path) -> dict[str, Any]:
    files = {}
    for relative in STATIC_PACKAGE_FILES:
        path = root / relative
        files[relative] = {
            'bytes': path.stat().st_size,
            'sha256': sha256(path),
        }
    return {
        'schema_version': CAPTURE_PACKAGE_SCHEMA,
        'package_root': str(declared_root),
        'seed': V2_SEED,
        'scenario_count': EXPECTED_SCENARIOS,
        'expected_image_count': EXPECTED_IMAGES,
        'canary_scenario_count': CANARY_COUNT,
        'v2_source': str(V2_PLAN),
        'v2_source_sha256': EXPECTED_V2_SHA256,
        'fixed_identity_order': list(GLOBAL_IDENTITIES),
        'visual_state_schema': VISUAL_STATE_SCHEMA_VERSION,
        'fixed_vector_dimension': VisualStateLabelVectorizer().dim,
        'dataset_inferred_capacity': False,
        'static_files': files,
        'mutable_files_excluded_from_static_hashes': list(
            MUTABLE_PACKAGE_FILES
        ),
        'capture_executed': False,
        'approved_for_capture': False,
        'approved_for_training': False,
    }


def verify_package(root: Path) -> dict[str, Any]:
    manifest = _load_object(root / 'package_manifest.json')
    failures = []
    for relative, expected in manifest.get('static_files', {}).items():
        path = root / relative
        if not path.is_file():
            failures.append(f'missing:{relative}')
            continue
        if path.stat().st_size != expected['bytes']:
            failures.append(f'bytes:{relative}')
        if sha256(path) != expected['sha256']:
            failures.append(f'sha256:{relative}')
    for relative in MUTABLE_PACKAGE_FILES:
        if not (root / relative).is_file():
            failures.append(f'missing_mutable:{relative}')
    return {
        'passed': not failures,
        'verified_static_file_count': len(
            manifest.get('static_files', {})
        ),
        'failures': failures,
    }


def prepare_package(
    output: Path,
    *,
    declared_root: Path | None = None,
) -> Path:
    output = output.expanduser().resolve()
    declared_root = (
        declared_root.expanduser().resolve()
        if declared_root is not None
        else output
    )
    if output.exists():
        raise FileExistsError(f'refusing to overwrite package: {output}')
    if sha256(V2_PLAN) != EXPECTED_V2_SHA256:
        raise CapturePackageError('authoritative v2 plan SHA-256 mismatch')
    if not validate_v2_package_manifest(V2_ROOT)['passed']:
        raise CapturePackageError('authoritative v2 package manifest failed')
    protected_before = _capture_protected_audit()
    if not protected_before['passed']:
        raise CapturePackageError(
            f'protected artifact preflight failed: {protected_before}'
        )
    v2_audit = _load_object(V2_ROOT / 'design_v2_audit.json')
    required_v2_checks = (
        'exactly_2040_rows',
        'exactly_255_configurations',
        'exactly_eight_variants_per_configuration',
        'zero_invalid_zone_segment_or_ratio',
        'zero_same_segment_physical_impossibilities',
        'zero_topology_or_switch_routing_violations',
        'zero_relation_violations',
        'zero_pairwise_separation_violations',
        'zero_exact_subset_or_prefix_substitutions',
        'zero_nonprojectable_active_identities',
        'fixed_vectorizer_dimension_200',
    )
    if (
        v2_audit.get('passed') is not True
        or any(
            v2_audit.get('checks', {}).get(check) is not True
            for check in required_v2_checks
        )
    ):
        raise CapturePackageError('authoritative v2 audit contract failed')
    v2_rows = read_jsonl(V2_PLAN)
    rows = executable_manifest_rows(v2_rows)
    audit = static_manifest_audit(v2_rows, rows)
    if not audit['passed']:
        raise CapturePackageError(
            f'executable manifest audit failed: {audit["checks"]}'
        )
    canary_ids = select_canary(rows)
    canary_report = canary_audit(rows, canary_ids)
    if not canary_report['passed']:
        raise CapturePackageError(
            f'production canary audit failed: {canary_report["checks"]}'
        )
    protected_after = _capture_protected_audit()
    if protected_before != protected_after or not protected_after['passed']:
        raise CapturePackageError('protected artifacts changed during preparation')

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(
        prefix=f'.{output.name}.',
        dir=output.parent,
    ))
    try:
        write_jsonl(temporary / 'scenario_manifest.jsonl', rows)
        write_json(temporary / 'production_manifest_audit.json', audit)
        (temporary / 'production_manifest_audit.md').write_text(
            _static_audit_markdown(audit),
            encoding='utf-8',
        )
        write_json(temporary / 'production_canary_scenario_ids.json', {
            'schema_version': CANARY_SCHEMA,
            'seed': V2_SEED,
            'scenario_count': CANARY_COUNT,
            'source_manifest_scenario_count': EXPECTED_SCENARIOS,
            'scenario_ids': canary_ids,
        })
        write_json(temporary / 'production_canary_audit.json', canary_report)
        (temporary / 'production_canary_audit.md').write_text(
            _canary_markdown(canary_report),
            encoding='utf-8',
        )
        manifest_sha = sha256(temporary / 'scenario_manifest.jsonl')
        canary_sha = sha256(
            temporary / 'production_canary_scenario_ids.json'
        )
        write_json(
            temporary / 'production_capture_approval.json',
            _approval_template(manifest_sha, canary_sha),
        )
        write_json(
            temporary / 'capture_state.json',
            _initial_capture_state(manifest_sha),
        )
        for directory in (
            'dataset/episodes',
            'dataset/meta',
            'logs',
            'manual_review_overlays',
            'canary_review_overlays',
        ):
            (temporary / directory).mkdir(parents=True)
        shutil.copy2(Path(__file__).resolve(), temporary / 'capture_status.py')
        scripts = _script_sources(declared_root)
        for name, source in scripts.items():
            path = temporary / name
            path.write_text(source, encoding='utf-8')
            path.chmod(path.stat().st_mode | 0o111)
        (temporary / 'capture_status.py').chmod(
            (temporary / 'capture_status.py').stat().st_mode | 0o111
        )
        (temporary / 'README.md').write_text(
            _readme(declared_root),
            encoding='utf-8',
        )
        write_json(
            temporary / 'package_manifest.json',
            _package_manifest(temporary, declared_root),
        )
        result = verify_package(temporary)
        if not result['passed']:
            raise CapturePackageError(
                f'generated package verification failed: {result}'
            )
        os.replace(temporary, output)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return output


CAPTURE_PROTECTED = {
    **{
        name: {
            'path': contract['path'],
            'file_count': contract['file_count'],
            'tree_sha256': contract['tree_sha256'],
        }
        for name, contract in PROTECTED_ROOTS.items()
    },
    'authoritative_v2': {
        'path': V2_ROOT,
        'file_count': 10,
        'tree_sha256': (
            '449a12629edafb3e2a64dbe45d56f5b1becef331236ff41f734ee1ebf3e2ed2a'
        ),
    },
}


def _capture_protected_audit() -> dict[str, Any]:
    results = {}
    for name, expected in CAPTURE_PROTECTED.items():
        count, digest = tree_fingerprint(expected['path'])
        results[name] = {
            'path': str(expected['path']),
            'file_count': count,
            'expected_file_count': expected['file_count'],
            'tree_sha256': digest,
            'expected_tree_sha256': expected['tree_sha256'],
            'passed': (
                count == expected['file_count']
                and digest == expected['tree_sha256']
            ),
        }
    return {
        'passed': all(row['passed'] for row in results.values()),
        'artifacts': results,
    }


def _manifest_rows(root: Path) -> list[dict[str, Any]]:
    return read_jsonl(root / 'scenario_manifest.jsonl')


def _canary_ids(root: Path) -> list[str]:
    value = _load_object(root / 'production_canary_scenario_ids.json')
    ids = value.get('scenario_ids')
    if not isinstance(ids, list):
        raise CapturePackageError('canary scenario_ids must be a list')
    return [str(value) for value in ids]


def validate_static_package(root: Path) -> dict[str, Any]:
    package = verify_package(root)
    audit = _load_object(root / 'production_manifest_audit.json')
    canary = _load_object(root / 'production_canary_audit.json')
    approval = _load_object(root / 'production_capture_approval.json')
    fingerprints = {
        'production_manifest_sha256': sha256(
            root / 'scenario_manifest.jsonl'
        ),
        'canary_id_list_sha256': sha256(
            root / 'production_canary_scenario_ids.json'
        ),
        'v2_source_sha256': sha256(V2_PLAN),
    }
    fingerprint_match = all(
        approval.get(field) == digest
        for field, digest in fingerprints.items()
    )
    result = {
        'passed': (
            package['passed']
            and audit.get('passed') is True
            and canary.get('passed') is True
            and fingerprint_match
        ),
        'package_manifest': package,
        'production_manifest_audit_passed': audit.get('passed') is True,
        'production_canary_audit_passed': canary.get('passed') is True,
        'fingerprints': fingerprints,
        'approval_fingerprints_match': fingerprint_match,
    }
    return result


def _gallery_overlay_set_sha256(
    root: Path,
    gallery: dict[str, Any],
    *,
    expected_scenario_count: int = CANARY_COUNT,
) -> str:
    rows = []
    source_count = 0
    overlay_count = 0
    for scenario in gallery.get('scenarios') or []:
        scenario_id = str(scenario.get('scenario_id') or '')
        for camera in CAMERAS:
            view = (scenario.get('views') or {}).get(camera)
            if not isinstance(view, dict):
                raise CapturePackageError(
                    f'{scenario_id}:{camera}: gallery view is missing'
                )
            for kind in ('source', 'overlay'):
                relative = str(view.get(kind) or '')
                expected = str(view.get(f'{kind}_sha256') or '')
                path = root / relative
                if (
                    not relative
                    or not path.is_file()
                    or sha256(path) != expected
                ):
                    raise CapturePackageError(
                        f'{scenario_id}:{camera}:{kind} fingerprint mismatch'
                    )
                rows.append(f'{kind}:{relative}:{expected}')
                if kind == 'source':
                    source_count += 1
                else:
                    overlay_count += 1
    if (
        source_count != expected_scenario_count * len(CAMERAS)
        or overlay_count != expected_scenario_count * len(CAMERAS)
    ):
        raise CapturePackageError(
            'gallery source/overlay counts do not match the scenario count'
        )
    return hashlib.sha256(
        '\n'.join(sorted(rows)).encode('utf-8')
    ).hexdigest()


def _validated_canary_review_artifacts(
    root: Path,
    *,
    require_clean_capture_state: bool = True,
) -> dict[str, str]:
    paths = {
        field: root / relative
        for field, relative in CANARY_REVIEW_FINGERPRINT_FILES.items()
    }
    missing = [
        str(path) for path in paths.values()
        if not path.is_file()
    ]
    if missing:
        raise CapturePackageError(
            f'canary review artifacts are missing: {missing}'
        )
    captured = _load_object(root / 'captured_canary_audit.json')
    camera = _load_object(root / 'camera_bbox_semantics_audit.json')
    gallery = _load_object(root / 'canary_gallery_manifest.json')
    required_camera_checks = (
        'left_has_zero_valid_right_boxes',
        'right_has_zero_valid_left_boxes',
        'every_valid_bbox_is_camera_compatible',
        'all_unavailable_bbox_targets_masked',
        'masked_boxes_excluded_from_training_loss',
        'masked_boxes_not_rendered',
        'no_empty_placeholder_boxes',
        'maximum_four_boxes_per_camera',
    )
    if (
        captured.get('passed') is not True
        or captured.get('expected_scenario_count') != CANARY_COUNT
        or captured.get('valid_scenario_count') != CANARY_COUNT
        or captured.get('valid_image_count') != CANARY_COUNT * len(CAMERAS)
    ):
        raise CapturePackageError('captured canary audit is not complete')
    if (
        camera.get('passed') is not True
        or camera.get('scope') != 'canary'
        or camera.get('scenario_count') != CANARY_COUNT
        or any(
            camera.get('checks', {}).get(check) is not True
            for check in required_camera_checks
        )
        or camera.get('after', {}).get(
            'opposite_rail_bbox_violation_count'
        ) != 0
        or camera.get('after', {}).get('empty_placeholder_count') != 0
    ):
        raise CapturePackageError(
            'corrected canary camera-bbox audit is not valid'
        )
    if (
        gallery.get('passed') is not True
        or gallery.get('scope') != 'canary'
        or gallery.get('scenario_count') != CANARY_COUNT
        or gallery.get('source_image_count') != CANARY_COUNT * len(CAMERAS)
        or gallery.get('overlay_image_count') != CANARY_COUNT * len(CAMERAS)
        or gallery.get('source_images_unchanged') is not True
    ):
        raise CapturePackageError('corrected canary gallery is not valid')
    status = status_report(root)
    if (
        status['completed_scenarios'] < CANARY_COUNT
        or not status['canary_complete']
        or status['valid_images'] < CANARY_COUNT * len(CAMERAS)
        or status['duplicate_event_count'] != 0
        or status['unexpected_event_count'] != 0
        or status['unexpected_episode_count'] != 0
        or status['exact_subset_validation'] is not True
        or status['manifest_oracle_equality'] is not True
    ):
        raise CapturePackageError(
            'current captured canary state is not approval-safe'
        )
    if (
        require_clean_capture_state
        and status['unresolved_failure_count'] != 0
    ):
        raise CapturePackageError(
            'capture has unresolved failures and cannot be newly approved'
        )
    fresh_camera = camera_bbox_semantics_audit(
        root,
        'canary',
        require_gallery=True,
    )
    if not fresh_camera['passed']:
        raise CapturePackageError(
            'fresh canary camera-bbox audit did not pass'
        )
    fingerprints = {
        field: sha256(path) for field, path in paths.items()
    }
    fingerprints[CANARY_REVIEW_OVERLAY_FINGERPRINT_FIELD] = (
        _gallery_overlay_set_sha256(root, gallery)
    )
    return fingerprints


def _canary_review_fingerprints_match(
    root: Path,
    approval: dict[str, Any],
    *,
    require_clean_capture_state: bool = False,
) -> bool:
    try:
        current = _validated_canary_review_artifacts(
            root,
            require_clean_capture_state=require_clean_capture_state,
        )
    except (CapturePackageError, OSError, ValueError):
        return False
    return all(
        approval.get(field) == digest
        for field, digest in current.items()
    )


def _validated_full_review_artifacts(
    root: Path,
) -> dict[str, str]:
    paths = {
        field: root / relative
        for field, relative in FULL_REVIEW_FINGERPRINT_FILES.items()
    }
    missing = [
        str(path) for path in paths.values()
        if not path.is_file()
    ]
    if missing:
        raise CapturePackageError(
            f'full production review artifacts are missing: {missing}'
        )
    captured = _load_object(root / 'captured_production_audit.json')
    camera = _load_object(
        root / 'production_camera_bbox_semantics_audit.json'
    )
    gallery = _load_object(
        root / 'production_review_gallery_manifest.json'
    )
    required_camera_checks = (
        'left_has_zero_valid_right_boxes',
        'right_has_zero_valid_left_boxes',
        'every_valid_bbox_is_camera_compatible',
        'all_unavailable_bbox_targets_masked',
        'masked_boxes_excluded_from_training_loss',
        'masked_boxes_not_rendered',
        'no_empty_placeholder_boxes',
        'maximum_four_boxes_per_camera',
        'gallery_contract_valid_if_required',
    )
    if (
        captured.get('passed') is not True
        or captured.get('expected_scenario_count') != EXPECTED_SCENARIOS
        or captured.get('valid_scenario_count') != EXPECTED_SCENARIOS
        or captured.get('valid_image_count')
        != EXPECTED_SCENARIOS * len(CAMERAS)
        or captured.get('duplicate_event_count') != 0
        or captured.get('unexpected_event_ids')
        or captured.get('validation_failures')
        or captured.get('changed_source_images')
    ):
        raise CapturePackageError(
            'captured production audit is not approval-safe'
        )
    if (
        camera.get('passed') is not True
        or camera.get('scope') != 'full'
        or camera.get('scenario_count') != EXPECTED_SCENARIOS
        or any(
            camera.get('checks', {}).get(check) is not True
            for check in required_camera_checks
        )
        or camera.get('after', {}).get(
            'opposite_rail_bbox_violation_count'
        ) != 0
        or camera.get('after', {}).get('empty_placeholder_count') != 0
    ):
        raise CapturePackageError(
            'production camera-bbox audit is not valid'
        )
    gallery_scenarios = gallery.get('scenarios') or []
    gallery_count = len(gallery_scenarios)
    if (
        gallery.get('passed') is not True
        or gallery.get('scope') != 'full'
        or gallery_count <= 0
        or gallery.get('scenario_count') != gallery_count
        or gallery.get('source_image_count')
        != gallery_count * len(CAMERAS)
        or gallery.get('overlay_image_count')
        != gallery_count * len(CAMERAS)
        or gallery.get('source_images_unchanged') is not True
    ):
        raise CapturePackageError(
            'production review gallery is not valid'
        )
    status = status_report(root)
    if (
        status['completed_scenarios'] != EXPECTED_SCENARIOS
        or status['remaining_scenarios'] != 0
        or status['valid_images']
        != EXPECTED_SCENARIOS * len(CAMERAS)
        or status['missing_images'] != 0
        or status['blank_images'] != 0
        or status['duplicate_event_count'] != 0
        or status['unexpected_event_count'] != 0
        or status['unexpected_episode_count'] != 0
        or status['unresolved_failure_count'] != 0
        or status['capture_complete'] is not True
        or status['exact_subset_validation'] is not True
        or status['manifest_oracle_equality'] is not True
    ):
        raise CapturePackageError(
            'current full capture state is not approval-safe'
        )
    fresh_camera = camera_bbox_semantics_audit(
        root,
        'full',
        require_gallery=True,
    )
    if not fresh_camera['passed']:
        raise CapturePackageError(
            'fresh production camera-bbox audit did not pass'
        )
    fingerprints = {
        field: sha256(path) for field, path in paths.items()
    }
    fingerprints[FULL_REVIEW_OVERLAY_FINGERPRINT_FIELD] = (
        _gallery_overlay_set_sha256(
            root,
            gallery,
            expected_scenario_count=gallery_count,
        )
    )
    return fingerprints


def _full_review_fingerprints_match(
    root: Path,
    approval: dict[str, Any],
) -> bool:
    try:
        current = _validated_full_review_artifacts(root)
    except (CapturePackageError, OSError, ValueError):
        return False
    return all(
        approval.get(field) == digest
        for field, digest in current.items()
    )


def approval_gate(root: Path, required: str) -> tuple[bool, int, str]:
    if required not in {
        'manifest',
        'canary',
        'canary-gallery',
        'full-capture',
        'full-gallery',
        'training',
    }:
        raise CapturePackageError(f'unsupported approval gate: {required}')
    static = validate_static_package(root)
    if not static['passed']:
        code = (
            GATE_EXIT_CODES['fingerprint']
            if not static['approval_fingerprints_match']
            or not static['package_manifest']['passed']
            else GATE_EXIT_CODES['manifest']
        )
        return False, code, 'MANIFEST_GATE_FAILED'
    if required == 'manifest':
        return True, 0, 'MANIFEST_GATE_VALID'
    approval = _load_object(root / 'production_capture_approval.json')
    if approval.get('approved_for_canary_capture') is not True:
        return False, GATE_EXIT_CODES['canary'], 'CANARY_APPROVAL_REQUIRED'
    if required == 'canary':
        return True, 0, 'CANARY_GATE_VALID'
    canary_audit_path = root / 'captured_canary_audit.json'
    if (
        not canary_audit_path.is_file()
        or _load_object(canary_audit_path).get('passed') is not True
    ):
        return (
            False,
            GATE_EXIT_CODES['canary_capture_incomplete'],
            'CANARY_CAPTURE_AUDIT_REQUIRED',
        )
    if approval.get('approved_after_canary_gallery_review') is not True:
        return (
            False,
            GATE_EXIT_CODES['canary-gallery'],
            'CANARY_GALLERY_APPROVAL_REQUIRED',
        )
    gallery = root / 'canary_gallery_manifest.json'
    if (
        not gallery.is_file()
        or _load_object(gallery).get('passed') is not True
    ):
        return (
            False,
            GATE_EXIT_CODES['canary-gallery'],
            'CANARY_GALLERY_REQUIRED',
        )
    if not _canary_review_fingerprints_match(root, approval):
        return (
            False,
            GATE_EXIT_CODES['fingerprint'],
            'CANARY_REVIEW_FINGERPRINT_MISMATCH',
        )
    if required == 'canary-gallery':
        return True, 0, 'CANARY_GALLERY_GATE_VALID'
    if approval.get('approved_for_full_capture') is not True:
        return (
            False,
            GATE_EXIT_CODES['full-capture'],
            'FULL_CAPTURE_APPROVAL_REQUIRED',
        )
    if required == 'full-capture':
        return True, 0, 'FULL_CAPTURE_GATE_VALID'
    full_audit_path = root / 'captured_production_audit.json'
    if (
        not full_audit_path.is_file()
        or _load_object(full_audit_path).get('passed') is not True
    ):
        return (
            False,
            GATE_EXIT_CODES['full_capture_incomplete'],
            'FULL_CAPTURE_AUDIT_REQUIRED',
        )
    if approval.get('approved_after_full_gallery_review') is not True:
        return (
            False,
            GATE_EXIT_CODES['full-gallery'],
            'FULL_GALLERY_APPROVAL_REQUIRED',
        )
    full_gallery = root / 'production_review_gallery_manifest.json'
    if (
        not full_gallery.is_file()
        or _load_object(full_gallery).get('passed') is not True
    ):
        return (
            False,
            GATE_EXIT_CODES['full-gallery'],
            'FULL_GALLERY_REQUIRED',
        )
    if not _full_review_fingerprints_match(root, approval):
        return (
            False,
            GATE_EXIT_CODES['fingerprint'],
            'FULL_REVIEW_FINGERPRINT_MISMATCH',
        )
    if required == 'full-gallery':
        return True, 0, 'FULL_GALLERY_GATE_VALID'
    if approval.get('approved_for_training') is not True:
        return (
            False,
            GATE_EXIT_CODES['training'],
            'TRAINING_APPROVAL_REQUIRED',
        )
    return True, 0, 'TRAINING_GATE_VALID'


def enable_canary_approval(
    root: Path,
    *,
    reviewer: str,
    notes: str,
) -> None:
    valid, code, message = approval_gate(root, 'manifest')
    if not valid:
        raise CapturePackageError(f'{message} ({code})')
    if not reviewer.strip() or not notes.strip():
        raise CapturePackageError('reviewer and notes are required')
    approval = _load_object(root / 'production_capture_approval.json')
    if any(
        approval.get(field) is not False
        for field in APPROVAL_FIELDS
    ):
        raise CapturePackageError(
            'enable-canary requires every approval field to still be false'
        )
    approval['approved_for_canary_capture'] = True
    approval['canary_reviewer'] = reviewer.strip()
    approval['canary_reviewed_at'] = (
        __import__('datetime').datetime.now(
            __import__('datetime').timezone.utc
        ).isoformat()
    )
    approval['canary_review_notes'] = notes.strip()
    _atomic_json(root / 'production_capture_approval.json', approval)


def approve_canary_gallery(
    root: Path,
    *,
    reviewer: str,
    notes: str,
) -> None:
    valid, code, message = approval_gate(root, 'canary')
    if not valid:
        raise CapturePackageError(f'{message} ({code})')
    if not reviewer.strip() or not notes.strip():
        raise CapturePackageError('reviewer and notes are required')
    approval = _load_object(root / 'production_capture_approval.json')
    if approval.get('approved_after_canary_gallery_review') is not False:
        raise CapturePackageError('canary gallery is already approved')
    if any(
        approval.get(field) is not False
        for field in (
            'approved_for_full_capture',
            'approved_after_full_gallery_review',
            'approved_for_training',
        )
    ):
        raise CapturePackageError(
            'later approval fields must remain false'
        )
    fingerprints = _validated_canary_review_artifacts(root)
    approval.update(fingerprints)
    approval['approved_after_canary_gallery_review'] = True
    approval['canary_gallery_reviewer'] = reviewer.strip()
    approval['canary_gallery_reviewed_at'] = (
        __import__('datetime').datetime.now(
            __import__('datetime').timezone.utc
        ).isoformat()
    )
    approval['canary_gallery_review_notes'] = notes.strip()
    _atomic_json(root / 'production_capture_approval.json', approval)
    valid, code, message = approval_gate(root, 'canary-gallery')
    if not valid:
        raise CapturePackageError(f'{message} ({code})')


def enable_full_capture_approval(
    root: Path,
    *,
    reviewer: str,
    notes: str,
) -> None:
    valid, code, message = approval_gate(root, 'canary-gallery')
    if not valid:
        raise CapturePackageError(f'{message} ({code})')
    if not reviewer.strip() or not notes.strip():
        raise CapturePackageError('reviewer and notes are required')
    approval = _load_object(root / 'production_capture_approval.json')
    if approval.get('approved_for_full_capture') is not False:
        raise CapturePackageError('full capture is already approved')
    if any(
        approval.get(field) is not False
        for field in (
            'approved_after_full_gallery_review',
            'approved_for_training',
        )
    ):
        raise CapturePackageError(
            'final-gallery and training approvals must remain false'
        )
    if not _canary_review_fingerprints_match(
        root,
        approval,
        require_clean_capture_state=True,
    ):
        raise CapturePackageError(
            'canary review artifacts changed after human approval'
        )
    approval['approved_for_full_capture'] = True
    approval['full_capture_reviewer'] = reviewer.strip()
    approval['full_capture_reviewed_at'] = (
        __import__('datetime').datetime.now(
            __import__('datetime').timezone.utc
        ).isoformat()
    )
    approval['full_capture_review_notes'] = notes.strip()
    _atomic_json(root / 'production_capture_approval.json', approval)
    valid, code, message = approval_gate(root, 'full-capture')
    if not valid:
        raise CapturePackageError(f'{message} ({code})')


def approve_full_gallery(
    root: Path,
    *,
    reviewer: str,
    notes: str,
) -> None:
    valid, code, message = approval_gate(root, 'full-capture')
    if not valid:
        raise CapturePackageError(f'{message} ({code})')
    if not reviewer.strip() or not notes.strip():
        raise CapturePackageError('reviewer and notes are required')
    approval = _load_object(root / 'production_capture_approval.json')
    if approval.get('approved_after_full_gallery_review') is not False:
        raise CapturePackageError(
            'full production gallery is already approved'
        )
    if approval.get('approved_for_training') is not False:
        raise CapturePackageError(
            'training approval must remain false'
        )
    fingerprints = _validated_full_review_artifacts(root)
    approval.update(fingerprints)
    approval['approved_after_full_gallery_review'] = True
    approval['full_gallery_reviewer'] = reviewer.strip()
    approval['full_gallery_reviewed_at'] = (
        __import__('datetime').datetime.now(
            __import__('datetime').timezone.utc
        ).isoformat()
    )
    approval['full_gallery_review_notes'] = notes.strip()
    _atomic_json(root / 'production_capture_approval.json', approval)
    valid, code, message = approval_gate(root, 'full-gallery')
    if not valid:
        raise CapturePackageError(f'{message} ({code})')


def approval_cli(root: Path, argv: list[str]) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--require', choices=(
        'manifest',
        'canary',
        'canary-gallery',
        'full-capture',
        'full-gallery',
        'training',
    ))
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument('--enable-canary', action='store_true')
    actions.add_argument('--approve-canary-gallery', action='store_true')
    actions.add_argument('--enable-full-capture', action='store_true')
    actions.add_argument('--approve-full-gallery', action='store_true')
    parser.add_argument('--reviewer', default='')
    parser.add_argument('--notes', default='')
    args = parser.parse_args(argv)
    if args.enable_canary:
        try:
            enable_canary_approval(
                root,
                reviewer=args.reviewer,
                notes=args.notes,
            )
        except (CapturePackageError, OSError) as exc:
            print(f'CANARY_APPROVAL_NOT_CHANGED: {exc}', file=sys.stderr)
            return GATE_EXIT_CODES['manifest']
        print('CANARY_APPROVAL_ENABLED_ONLY')
        return 0
    if args.approve_canary_gallery:
        try:
            approve_canary_gallery(
                root,
                reviewer=args.reviewer,
                notes=args.notes,
            )
        except (CapturePackageError, OSError) as exc:
            print(
                f'CANARY_GALLERY_APPROVAL_NOT_CHANGED: {exc}',
                file=sys.stderr,
            )
            return GATE_EXIT_CODES['canary-gallery']
        print('CANARY_GALLERY_APPROVED_ONLY')
        return 0
    if args.enable_full_capture:
        try:
            enable_full_capture_approval(
                root,
                reviewer=args.reviewer,
                notes=args.notes,
            )
        except (CapturePackageError, OSError) as exc:
            print(
                f'FULL_CAPTURE_APPROVAL_NOT_CHANGED: {exc}',
                file=sys.stderr,
            )
            return GATE_EXIT_CODES['full-capture']
        print('FULL_CAPTURE_APPROVED_ONLY')
        return 0
    if args.approve_full_gallery:
        try:
            approve_full_gallery(
                root,
                reviewer=args.reviewer,
                notes=args.notes,
            )
        except (CapturePackageError, OSError) as exc:
            print(
                f'FULL_GALLERY_APPROVAL_NOT_CHANGED: {exc}',
                file=sys.stderr,
            )
            return GATE_EXIT_CODES['full-gallery']
        print('FULL_GALLERY_APPROVED_ONLY')
        return 0
    if not args.require:
        parser.error('an approval action or --require is required')
    valid, code, message = approval_gate(root, args.require)
    print(message)
    return 0 if valid else code


def _image_blank(path: Path) -> bool:
    with Image.open(path) as image:
        image.verify()
    with Image.open(path) as image:
        extrema = image.convert('RGB').getextrema()
    return all(maximum - minimum <= 1 for minimum, maximum in extrema)


def _episode_validation(
    root: Path,
    scenario: dict[str, Any],
) -> dict[str, Any]:
    scenario_id = scenario['scenario_id']
    episode = root / 'dataset' / 'episodes' / scenario_id
    required = [
        episode / 'event.json',
        episode / 'validation.json',
        *[
            episode / 'images' / camera / 'frame_000000.jpg'
            for camera in CAMERAS
        ],
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        return {'valid': False, 'error': 'missing_files', 'missing': missing}
    try:
        event = _load_object(required[0])
        validation = _load_object(required[1])
        if (
            validation.get('validation_status') != 'approved'
            or validation.get('capture_complete') is not True
            or validation.get('labels_valid') is not True
        ):
            raise CapturePackageError('episode validation gate is not approved')
        labels = normalize_visual_state_labels(
            event,
            context=f'episode {scenario_id}',
        )
        if event.get('episode_id') != scenario_id:
            raise CapturePackageError('event episode_id does not match manifest')
        if labels.get('schema_version') != VISUAL_STATE_SCHEMA_VERSION:
            raise CapturePackageError('visual-state schema mismatch')
        vectorizer = VisualStateLabelVectorizer()
        vectorizer.validate_target(labels)
        camera_masks = vectorizer.camera_target_masks(labels)
        if vectorizer.dim != 200:
            raise CapturePackageError('fixed vector dimension is not 200')
        present = {
            shuttle['id']: shuttle
            for shuttle in labels['shuttles']
            if shuttle['presence']
        }
        expected = {
            shuttle['id']: (side, shuttle)
            for side in ('left', 'right')
            for shuttle in scenario['scene']['rails'][side]['shuttles']
        }
        if set(present) != set(expected):
            raise CapturePackageError('manifest/oracle identity mismatch')
        for identity, (side, shuttle) in expected.items():
            actual = present[identity]
            position = shuttle['start_position']
            rail = actual['rail_position']
            if actual['loaded_state'] != shuttle['loaded_state']:
                raise CapturePackageError(f'{identity}: payload mismatch')
            if (
                actual['location']['side'] != side
                or actual['location']['block'] != position['segment']
                or not rail['available']
                or abs(float(rail['s_ratio']) - float(position['s_ratio'])) > 0.015
                or abs(float(rail['s_m']) - float(position['s_m'])) > 0.04
                or abs(
                    float(rail['segment_length_m'])
                    - float(position['segment_length_m'])
                ) > 1e-5
                or any(float(value) < 0.0 for value in actual['bbox'][2:])
                or float(actual['bbox'][2]) <= 0.0
                or float(actual['bbox'][3]) <= 0.0
            ):
                raise CapturePackageError(f'{identity}: label mismatch')
            canonical_camera = canonical_camera_for_identity(identity)
            opposite_camera = next(
                camera for camera in CAMERAS
                if camera != canonical_camera
            )
            canonical = camera_observation_for_shuttle(
                actual,
                canonical_camera,
            )
            opposite = camera_observation_for_shuttle(
                actual,
                opposite_camera,
            )
            if (
                canonical['visual_available'] is not True
                or not valid_bbox(canonical['bbox'])
                or opposite['visual_available'] is not False
                or any(float(value) != 0.0 for value in opposite['bbox'])
                or any(
                    float(value) != 0.0
                    for value in opposite['bbox_target_mask']
                )
            ):
                raise CapturePackageError(
                    f'{identity}: per-camera bbox contract mismatch'
                )
            slot = list(GLOBAL_IDENTITIES).index(identity)
            bbox_indexes = [
                index
                for index, name in enumerate(vectorizer.names)
                if name.startswith(f'shuttles.{slot}.bbox.')
            ]
            if (
                len(bbox_indexes) != 4
                or not all(
                    camera_masks[canonical_camera][index] == 1.0
                    for index in bbox_indexes
                )
                or any(
                    camera_masks[opposite_camera][index] != 0.0
                    for index in bbox_indexes
                )
            ):
                raise CapturePackageError(
                    f'{identity}: per-camera bbox training mask mismatch'
                )
        image_hashes = {}
        for path in required[2:]:
            if _image_blank(path):
                raise CapturePackageError(f'blank image: {path}')
            image_hashes[str(path.relative_to(root))] = sha256(path)
        return {
            'valid': True,
            'event': event,
            'image_hashes': image_hashes,
            'exact_subset_validation': True,
            'manifest_oracle_equality': True,
        }
    except (
        CapturePackageError,
        OSError,
        ValueError,
        json.JSONDecodeError,
    ) as exc:
        return {'valid': False, 'error': str(exc)}


def _aggregate_rows(root: Path) -> tuple[list[dict[str, Any]], int]:
    path = root / 'dataset' / 'meta' / 'training_events.jsonl'
    if not path.is_file():
        return [], 0
    rows = read_jsonl(path)
    ids = [str(row.get('episode_id') or '') for row in rows]
    return rows, len(ids) - len(set(ids))


def status_report(root: Path) -> dict[str, Any]:
    scenarios = _manifest_rows(root)
    canary = set(_canary_ids(root))
    expected_ids = {row['scenario_id'] for row in scenarios}
    episode_root = root / 'dataset' / 'episodes'
    unexpected = sorted(
        path.name
        for path in episode_root.iterdir()
        if path.is_dir()
        and not path.name.startswith('.')
        and path.name not in expected_ids
    ) if episode_root.is_dir() else []
    completed = []
    failures = {}
    valid_images = 0
    blank_images = 0
    camera_counts = Counter()
    exact_values = []
    oracle_values = []
    for scenario in scenarios:
        episode = episode_root / scenario['scenario_id']
        if not episode.is_dir():
            continue
        result = _episode_validation(root, scenario)
        if result['valid']:
            completed.append(scenario['scenario_id'])
            valid_images += len(result['image_hashes'])
            for relative in result['image_hashes']:
                camera_counts[Path(relative).parent.name] += 1
            exact_values.append(result['exact_subset_validation'])
            oracle_values.append(result['manifest_oracle_equality'])
        else:
            failures[scenario['scenario_id']] = result
            for camera in CAMERAS:
                image = (
                    episode / 'images' / camera / 'frame_000000.jpg'
                )
                if image.is_file():
                    try:
                        blank_images += int(_image_blank(image))
                    except OSError:
                        pass
    aggregate, duplicates = _aggregate_rows(root)
    aggregate_ids = {
        str(row.get('episode_id') or '')
        for row in aggregate
    }
    unexpected_events = sorted(aggregate_ids - expected_ids)
    state = _load_object(root / 'capture_state.json')
    completed_set = set(completed)
    canary_completed = sorted(completed_set & canary)
    return {
        'schema_version': CAPTURE_STATUS_SCHEMA,
        'capture_has_started': bool(
            state.get('capture_has_started') or completed
        ),
        'capture_complete': len(completed) == EXPECTED_SCENARIOS,
        'expected_scenarios': EXPECTED_SCENARIOS,
        'completed_scenarios': len(completed),
        'completed_scenario_ids': completed,
        'remaining_scenarios': EXPECTED_SCENARIOS - len(completed),
        'current_scenario': state.get('current_scenario'),
        'expected_images': EXPECTED_IMAGES,
        'valid_images': valid_images,
        'missing_images': EXPECTED_IMAGES - valid_images,
        'blank_images': blank_images,
        'per_camera_image_counts': {
            camera: camera_counts[camera] for camera in CAMERAS
        },
        'historical_failure_count': len(
            state.get('historical_failures') or []
        ),
        'unresolved_failure_count': len(
            state.get('unresolved_failures') or []
        ),
        'failures': failures,
        'canary_expected': CANARY_COUNT,
        'canary_completed': len(canary_completed),
        'canary_remaining': CANARY_COUNT - len(canary_completed),
        'canary_complete': len(canary_completed) == CANARY_COUNT,
        'exact_subset_validation': (
            all(exact_values) if exact_values else None
        ),
        'manifest_oracle_equality': (
            all(oracle_values) if oracle_values else None
        ),
        'duplicate_event_count': duplicates,
        'unexpected_event_count': len(unexpected_events),
        'unexpected_event_ids': unexpected_events,
        'unexpected_episode_count': len(unexpected),
        'unexpected_episode_ids': unexpected,
    }


def _write_state_from_status(
    root: Path,
    report: dict[str, Any],
    state: dict[str, Any],
) -> None:
    state.update({
        'capture_has_started': report['capture_has_started'],
        'capture_complete': report['capture_complete'],
        'completed_scenarios': report['completed_scenario_ids'],
        'captured_scenario_count': report['completed_scenarios'],
        'valid_image_count': report['valid_images'],
        'missing_image_count': report['missing_images'],
        'exact_subset_validation': report['exact_subset_validation'],
        'canary_completed_count': report['canary_completed'],
        'canary_complete': report['canary_complete'],
    })
    _atomic_json(root / 'capture_state.json', state)


def rebuild_aggregate(root: Path) -> dict[str, Any]:
    scenarios = _manifest_rows(root)
    expected = {row['scenario_id'] for row in scenarios}
    episode_root = root / 'dataset' / 'episodes'
    unexpected = sorted(
        path.name
        for path in episode_root.iterdir()
        if path.is_dir()
        and not path.name.startswith('.')
        and path.name not in expected
    )
    if unexpected:
        raise CapturePackageError(
            f'unexpected episode directories: {unexpected[:20]}'
        )
    events = []
    for scenario in scenarios:
        result = _episode_validation(root, scenario)
        if result['valid']:
            events.append(result['event'])
        elif (
            episode_root / scenario['scenario_id']
        ).exists():
            raise CapturePackageError(
                f'invalid existing episode cannot be indexed: '
                f'{scenario["scenario_id"]}: {result}'
            )
    ids = [event['episode_id'] for event in events]
    if len(ids) != len(set(ids)):
        raise CapturePackageError('duplicate episode IDs during aggregate rebuild')
    _atomic_jsonl(
        root / 'dataset' / 'meta' / 'training_events.jsonl',
        events,
    )
    return {'event_count': len(events), 'duplicate_count': 0}


class CaptureLock:
    def __init__(self, root: Path):
        self.path = root / '.capture.lock'
        self.stream: Any = None

    def __enter__(self):
        self.stream = self.path.open('a+', encoding='utf-8')
        try:
            fcntl.flock(
                self.stream.fileno(),
                fcntl.LOCK_EX | fcntl.LOCK_NB,
            )
        except BlockingIOError as exc:
            self.stream.close()
            raise CapturePackageError('parallel capture is already running') from exc
        self.stream.seek(0)
        self.stream.truncate()
        self.stream.write(str(os.getpid()))
        self.stream.flush()
        return self

    def __exit__(self, *_args):
        fcntl.flock(self.stream.fileno(), fcntl.LOCK_UN)
        self.stream.close()


def _pending_ids(root: Path, mode: str) -> list[str]:
    scenarios = _manifest_rows(root)
    if mode == 'canary':
        requested = set(_canary_ids(root))
        scenarios = [
            row for row in scenarios if row['scenario_id'] in requested
        ]
    pending = []
    for scenario in scenarios:
        episode = root / 'dataset' / 'episodes' / scenario['scenario_id']
        result = _episode_validation(root, scenario)
        if result['valid']:
            continue
        if episode.exists():
            raise CapturePackageError(
                f'invalid existing episode will not be overwritten: '
                f'{scenario["scenario_id"]}: {result}'
            )
        pending.append(scenario['scenario_id'])
    return pending


def run_capture(
    root: Path,
    *,
    mode: str,
    extra_args: list[str],
) -> int:
    required_gate = 'canary' if mode == 'canary' else 'full-capture'
    valid, code, message = approval_gate(root, required_gate)
    if not valid:
        print(message, file=sys.stderr)
        return code
    with CaptureLock(root):
        scenarios = {
            row['scenario_id']: row
            for row in _manifest_rows(root)
        }
        pending = _pending_ids(root, mode)
        state = _load_object(root / 'capture_state.json')
        state['capture_has_started'] = True
        for scenario_id in pending:
            state['current_scenario'] = scenario_id
            _atomic_json(root / 'capture_state.json', state)
            command = [
                'ros2',
                'run',
                'mfja_robot_control_config',
                'room_315_visual_scenario_runner.py',
                '--scenario-manifest',
                str(root / 'scenario_manifest.jsonl'),
                '--output-dataset',
                str(root / 'dataset'),
                '--scenario-id',
                scenario_id,
                '--resume',
                *extra_args,
            ]
            completed = subprocess.run(command, check=False)
            if completed.returncode != 0:
                failure = {
                    'scenario_id': scenario_id,
                    'returncode': completed.returncode,
                }
                state.setdefault('historical_failures', []).append(failure)
                state['unresolved_failures'] = [
                    *[
                        row
                        for row in state.get('unresolved_failures') or []
                        if row.get('scenario_id') != scenario_id
                    ],
                    failure,
                ]
                state['current_scenario'] = None
                _atomic_json(root / 'capture_state.json', state)
                return completed.returncode
            result = _episode_validation(root, scenarios[scenario_id])
            if not result['valid']:
                raise CapturePackageError(
                    f'captured episode failed atomic post-validation: '
                    f'{scenario_id}: {result}'
                )
            state['unresolved_failures'] = [
                row
                for row in state.get('unresolved_failures') or []
                if row.get('scenario_id') != scenario_id
            ]
            rebuild_aggregate(root)
            report = status_report(root)
            _write_state_from_status(root, report, state)
        state['current_scenario'] = None
        report = status_report(root)
        _write_state_from_status(root, report, state)
    return 0


def resume_capture(root: Path, extra_args: list[str]) -> int:
    report = status_report(root)
    mode = 'full' if report['canary_complete'] else 'canary'
    return run_capture(root, mode=mode, extra_args=extra_args)


def _captured_scope_distributions(
    scenarios: list[dict[str, Any]],
) -> dict[str, dict[str, int]]:
    configuration_variants = Counter()
    geometry_variants: dict[str, set[str]] = defaultdict(set)
    presence = Counter()
    absence = Counter()
    loaded = Counter()
    empty = Counter()
    relation_families = Counter()
    target_zones = Counter()
    cardinality_pairs = Counter()
    total_active = Counter()
    segments = Counter()
    roles = Counter()
    for scenario in scenarios:
        configuration = scenario['configuration_id']
        configuration_variants[configuration] += 1
        geometry_variants[configuration].add(scenario['geometry_key'])
        active = _active_ids(scenario)
        active_set = set(active)
        cardinality_pairs[
            f'{len(scenario["left_active_identities"])}'
            f'+{len(scenario["right_active_identities"])}'
        ] += 1
        total_active[str(len(active))] += 1
        for identity in GLOBAL_IDENTITIES:
            if identity in active_set:
                presence[identity] += 1
            else:
                absence[identity] += 1
        for identity, state in scenario['payload_assignment'].items():
            (loaded if state == 'loaded' else empty)[identity] += 1
        relation_families[scenario['relation_family']] += 1
        target_zones[scenario['target_zone']] += 1
        roles[f'{scenario["target_identity"]}:target'] += 1
        relation_role = (
            'non_blocker'
            if scenario['relation_family'] in {
                'nonblocker_behind_same_segment',
                'nonblocker_adjacent_branch',
            }
            else 'blocker'
        )
        for identity in scenario['relation_identities']:
            roles[f'{identity}:{relation_role}'] += 1
        for identity in (
            scenario['relation_neutral_identities']
            + scenario['opposite_rail_distractor_identities']
        ):
            roles[f'{identity}:relation_neutral'] += 1
        for side in ('left', 'right'):
            for shuttle in scenario['scene']['rails'][side]['shuttles']:
                segments[
                    f'{side}:{shuttle["start_position"]["segment"]}'
                ] += 1
    return {
        'configuration_variant_count': dict(
            sorted(configuration_variants.items())
        ),
        'configuration_unique_geometry_count': {
            key: len(value)
            for key, value in sorted(geometry_variants.items())
        },
        'cardinality_pair': dict(sorted(cardinality_pairs.items())),
        'total_active_count': dict(sorted(total_active.items())),
        'identity_presence': dict(sorted(presence.items())),
        'identity_absence': dict(sorted(absence.items())),
        'identity_loaded': dict(sorted(loaded.items())),
        'identity_empty': dict(sorted(empty.items())),
        'identity_roles': dict(sorted(roles.items())),
        'relation_family': dict(sorted(relation_families.items())),
        'target_zone': dict(sorted(target_zones.items())),
        'all_active_segments': dict(sorted(segments.items())),
    }


def captured_audit(root: Path, scope: str) -> dict[str, Any]:
    scenarios = _manifest_rows(root)
    if scope == 'canary':
        selected_ids = set(_canary_ids(root))
        scenarios = [
            row for row in scenarios if row['scenario_id'] in selected_ids
        ]
    expected_ids = {row['scenario_id'] for row in scenarios}
    validations = {}
    image_hashes = {}
    valid_scenarios = []
    for scenario in scenarios:
        result = _episode_validation(root, scenario)
        if not result['valid']:
            validations[scenario['scenario_id']] = result
        else:
            valid_scenarios.append(scenario)
            image_hashes.update(result['image_hashes'])
    aggregate, duplicates = _aggregate_rows(root)
    aggregate_ids = [
        str(row.get('episode_id') or '')
        for row in aggregate
    ]
    aggregate_id_set = set(aggregate_ids)
    full_manifest_ids = {
        row['scenario_id'] for row in _manifest_rows(root)
    }
    unexpected_events = sorted(set(aggregate_ids) - full_manifest_ids)
    previous_path = root / (
        'captured_canary_audit.json'
        if scope == 'canary'
        else 'captured_production_audit.json'
    )
    previous = _load_object(previous_path) if previous_path.is_file() else {}
    previous_hashes = previous.get('source_image_hashes') or {}
    changed_images = sorted(
        relative
        for relative, digest in previous_hashes.items()
        if relative in image_hashes and image_hashes[relative] != digest
    )
    expected_images = len(scenarios) * len(CAMERAS)
    complete = (
        len(image_hashes) == expected_images
        and not validations
        and len({
            row['scenario_id']
            for row in scenarios
            if _episode_validation(root, row)['valid']
        }) == len(scenarios)
    )
    static = validate_static_package(root)
    camera_bbox = camera_bbox_semantics_audit(
        root,
        scope,
        require_gallery=False,
    )
    distributions = _captured_scope_distributions(valid_scenarios)
    all_expected_events_indexed = expected_ids <= aggregate_id_set
    checks = {
        'nonempty_expected_scope': bool(scenarios),
        'all_expected_episodes_valid': complete,
        'exact_source_rgb_image_count': len(image_hashes) == expected_images,
        'both_cameras_per_episode': len(image_hashes) == expected_images,
        'all_expected_event_rows_indexed': all_expected_events_indexed,
        'zero_duplicate_event_rows': duplicates == 0,
        'zero_unexpected_event_rows': not unexpected_events,
        'source_image_hashes_stable': not changed_images,
        'exact_manifest_oracle_identity_equality': complete,
        'zero_prefix_substitutions': complete,
        'payload_equality': complete,
        'valid_presence_absence_masks': complete,
        'valid_bbox_s_m_s_ratio_and_segment_length': complete,
        'valid_relation_metadata': (
            _load_object(root / 'production_manifest_audit.json')
            .get('checks', {})
            .get('zero_relation_violations') is True
        ),
        'package_and_manifest_fingerprints_valid': static['passed'],
        'fixed_schema_v3_dimension_200': (
            VISUAL_STATE_SCHEMA_VERSION == 'room315.visual_state.v3'
            and VisualStateLabelVectorizer().dim == 200
        ),
        'dataset_inferred_capacity_false': (
            VisualStateLabelVectorizer().to_json()[
                'capacity_inferred_from_dataset'
            ] is False
        ),
        'per_camera_bbox_semantics_valid': camera_bbox['passed'],
    }
    if scope == 'full':
        configuration_variants = distributions[
            'configuration_variant_count'
        ]
        geometry_variants = distributions[
            'configuration_unique_geometry_count'
        ]
        checks.update({
            'exactly_255_captured_presence_configurations': (
                len(configuration_variants) == 255
            ),
            'exactly_8_captured_variants_per_configuration': (
                len(configuration_variants) == 255
                and all(
                    count == 8
                    for count in configuration_variants.values()
                )
            ),
            'eight_unique_captured_geometries_per_configuration': (
                len(geometry_variants) == 255
                and all(
                    count == 8 for count in geometry_variants.values()
                )
            ),
            'every_identity_captured_present_1024': all(
                distributions['identity_presence'].get(identity) == 1024
                for identity in GLOBAL_IDENTITIES
            ),
            'every_identity_captured_absent_1016': all(
                distributions['identity_absence'].get(identity) == 1016
                for identity in GLOBAL_IDENTITIES
            ),
            'every_identity_captured_loaded_512': all(
                distributions['identity_loaded'].get(identity) == 512
                for identity in GLOBAL_IDENTITIES
            ),
            'every_identity_captured_empty_512': all(
                distributions['identity_empty'].get(identity) == 512
                for identity in GLOBAL_IDENTITIES
            ),
            'exact_captured_relation_family_totals': (
                distributions['relation_family']
                == EXPECTED_RELATION_TOTALS
            ),
            'exact_captured_target_zone_totals': (
                distributions['target_zone'] == EXPECTED_ZONE_TOTALS
            ),
            'all_14_public_segments_captured_on_both_rails': all(
                distributions['all_active_segments'].get(
                    f'{side}:{segment}',
                    0,
                ) > 0
                for side in ('left', 'right')
                for segment in valid_public_segments(side)
            ),
        })
    return {
        'schema_version': CAPTURE_AUDIT_SCHEMA,
        'scope': scope,
        'passed': all(checks.values()),
        'checks': checks,
        'expected_scenario_count': len(scenarios),
        'valid_scenario_count': len(scenarios) - len(validations),
        'expected_image_count': expected_images,
        'valid_image_count': len(image_hashes),
        'validation_failures': validations,
        'duplicate_event_count': duplicates,
        'unexpected_event_ids': unexpected_events,
        'indexed_expected_event_count': len(
            expected_ids & aggregate_id_set
        ),
        'changed_source_images': changed_images,
        'source_image_hashes': dict(sorted(image_hashes.items())),
        'captured_distributions': distributions,
        'manifest_sha256': sha256(root / 'scenario_manifest.jsonl'),
        'package_static_verification': verify_package(root),
        'camera_bbox_semantics': {
            'passed': camera_bbox['passed'],
            'checks': camera_bbox['checks'],
            'per_camera': camera_bbox['after']['per_camera'],
        },
    }


def _camera_render_plan(
    labels: dict[str, Any],
    camera: str,
) -> list[dict[str, Any]]:
    if camera not in CAMERAS:
        raise CapturePackageError(f'unsupported gallery camera: {camera}')
    plan = []
    for shuttle in labels['shuttles']:
        observation = camera_observation_for_shuttle(shuttle, camera)
        if not shuttle['presence'] or not observation['applicable']:
            continue
        if not observation['visual_available']:
            continue
        if (
            not valid_bbox(observation['bbox'])
            or any(
                float(value) != 1.0
                for value in observation['bbox_target_mask']
            )
        ):
            raise CapturePackageError(
                f'{shuttle["id"]}: invalid unmasked bbox for {camera}'
            )
        if canonical_camera_for_identity(shuttle['id']) != camera:
            raise CapturePackageError(
                f'{shuttle["id"]}: opposite-rail bbox for {camera}'
            )
        plan.append({
            'identity': shuttle['id'],
            'bbox': [float(value) for value in observation['bbox']],
        })
    if len(plan) > 4:
        raise CapturePackageError(
            f'{camera}: renderer plan contains more than four boxes'
        )
    return plan


def _legacy_renderer_summary(
    rows: list[tuple[str, dict[str, Any]]],
) -> dict[str, Any]:
    """Reproduce the pre-fix renderer's global-label behavior exactly."""
    per_camera = {
        camera: {
            'rendered_rectangle_count': 0,
            'opposite_rail_bbox_violation_count': 0,
            'empty_placeholder_count': 0,
            'scenario_examples': [],
        }
        for camera in CAMERAS
    }
    for scenario_id, labels in rows:
        for camera in CAMERAS:
            cross = []
            for shuttle in labels['shuttles']:
                if (
                    not shuttle['presence']
                    or not shuttle['visually_available']
                ):
                    continue
                if not valid_bbox(shuttle['bbox']):
                    per_camera[camera]['empty_placeholder_count'] += 1
                    continue
                per_camera[camera]['rendered_rectangle_count'] += 1
                if canonical_camera_for_identity(shuttle['id']) != camera:
                    per_camera[camera][
                        'opposite_rail_bbox_violation_count'
                    ] += 1
                    cross.append(shuttle['id'])
            if cross and len(
                per_camera[camera]['scenario_examples']
            ) < 10:
                per_camera[camera]['scenario_examples'].append({
                    'scenario_id': scenario_id,
                    'opposite_identities_rendered': cross,
                })
    return {
        'renderer_contract': (
            'pre-fix renderer drew every globally present and visually '
            'available shuttle on both camera overlays'
        ),
        'per_camera': per_camera,
        'rendered_rectangle_count': sum(
            values['rendered_rectangle_count']
            for values in per_camera.values()
        ),
        'opposite_rail_bbox_violation_count': sum(
            values['opposite_rail_bbox_violation_count']
            for values in per_camera.values()
        ),
        'empty_placeholder_count': sum(
            values['empty_placeholder_count']
            for values in per_camera.values()
        ),
    }


def _scope_scenarios(
    root: Path,
    scope: str,
) -> list[dict[str, Any]]:
    scenarios = _manifest_rows(root)
    if scope == 'canary':
        selected = set(_canary_ids(root))
        scenarios = [
            row for row in scenarios
            if row['scenario_id'] in selected
        ]
    return scenarios


def _example_subsets(
    scenarios: list[dict[str, Any]],
) -> dict[str, str | None]:
    expected = {
        'L3': (('L3',), ()),
        'R4': ((), ('R4',)),
        'L2+L4': (('L2', 'L4'), ()),
        'R1+R3': ((), ('R1', 'R3')),
        '4+4': (
            ('L1', 'L2', 'L3', 'L4'),
            ('R1', 'R2', 'R3', 'R4'),
        ),
    }
    result: dict[str, str | None] = {}
    for name, (left, right) in expected.items():
        match = next(
            (
                row['scenario_id']
                for row in scenarios
                if tuple(row['left_active_identities']) == left
                and tuple(row['right_active_identities']) == right
            ),
            None,
        )
        result[name] = match
    return result


def camera_bbox_semantics_audit(
    root: Path,
    scope: str,
    *,
    require_gallery: bool,
) -> dict[str, Any]:
    scenarios = _scope_scenarios(root, scope)
    vectorizer = VisualStateLabelVectorizer()
    per_camera = {
        camera: {
            'valid_bbox_count': 0,
            'masked_bbox_count': 0,
            'opposite_rail_bbox_violation_count': 0,
            'empty_placeholder_count': 0,
            'rendered_rectangle_count': 0,
            'maximum_valid_bbox_count_in_one_scenario': 0,
            'bbox_training_mask_violation_count': 0,
        }
        for camera in CAMERAS
    }
    normalized_rows: list[tuple[str, dict[str, Any]]] = []
    validation_failures = {}
    raw_missing_camera_observation_slots = 0
    raw_explicit_camera_observation_slots = 0
    present_but_opposite_camera_masked_count = 0
    rendered_by_scenario: dict[str, dict[str, list[str]]] = {}
    for scenario in scenarios:
        scenario_id = scenario['scenario_id']
        event_path = (
            root / 'dataset' / 'episodes' / scenario_id / 'event.json'
        )
        if not event_path.is_file():
            validation_failures[scenario_id] = 'missing event.json'
            continue
        try:
            event = _load_object(event_path)
            raw_labels = event.get('visual_state_labels')
            if not isinstance(raw_labels, dict):
                raise CapturePackageError('visual_state_labels is missing')
            raw_shuttles = raw_labels.get('shuttles')
            if not isinstance(raw_shuttles, list):
                raise CapturePackageError('shuttles is not a list')
            for shuttle in raw_shuttles:
                if isinstance(shuttle, dict) and isinstance(
                    shuttle.get('camera_observations'),
                    dict,
                ):
                    raw_explicit_camera_observation_slots += 1
                else:
                    raw_missing_camera_observation_slots += 1
            labels = normalize_visual_state_labels(
                event,
                context=f'camera bbox audit {scenario_id}',
            )
            vectorizer.validate_target(labels)
            masks = vectorizer.camera_target_masks(labels)
            rendered_by_scenario[scenario_id] = {}
            for camera in CAMERAS:
                plan = _camera_render_plan(labels, camera)
                rendered_ids = [item['identity'] for item in plan]
                rendered_by_scenario[scenario_id][camera] = rendered_ids
                per_camera[camera]['rendered_rectangle_count'] += len(plan)
                per_camera[camera][
                    'maximum_valid_bbox_count_in_one_scenario'
                ] = max(
                    per_camera[camera][
                        'maximum_valid_bbox_count_in_one_scenario'
                    ],
                    len(plan),
                )
                for slot, shuttle in enumerate(labels['shuttles']):
                    observation = camera_observation_for_shuttle(
                        shuttle,
                        camera,
                    )
                    bbox_indexes = [
                        index
                        for index, name in enumerate(vectorizer.names)
                        if name.startswith(f'shuttles.{slot}.bbox.')
                    ]
                    bbox_mask = [masks[camera][index] for index in bbox_indexes]
                    compatible = (
                        canonical_camera_for_identity(shuttle['id'])
                        == camera
                    )
                    is_valid = (
                        observation['visual_available']
                        and valid_bbox(observation['bbox'])
                        and all(
                            float(value) == 1.0
                            for value in observation['bbox_target_mask']
                        )
                    )
                    is_masked = (
                        not observation['visual_available']
                        and all(
                            float(value) == 0.0
                            for value in observation['bbox_target_mask']
                        )
                        and all(float(value) == 0.0 for value in bbox_mask)
                    )
                    if is_valid:
                        per_camera[camera]['valid_bbox_count'] += 1
                    if is_masked:
                        per_camera[camera]['masked_bbox_count'] += 1
                    if (
                        not compatible
                        and (
                            observation['visual_available']
                            or valid_bbox(observation['bbox'])
                            or any(
                                float(value) != 0.0
                                for value in observation['bbox_target_mask']
                            )
                        )
                    ):
                        per_camera[camera][
                            'opposite_rail_bbox_violation_count'
                        ] += 1
                    if (
                        not observation['visual_available']
                        and (
                            valid_bbox(observation['bbox'])
                            or shuttle['id'] in rendered_ids
                        )
                    ):
                        per_camera[camera][
                            'empty_placeholder_count'
                        ] += 1
                    expected_mask = 1.0 if is_valid else 0.0
                    if (
                        len(bbox_mask) != 4
                        or any(value != expected_mask for value in bbox_mask)
                    ):
                        per_camera[camera][
                            'bbox_training_mask_violation_count'
                        ] += 1
                    if (
                        shuttle['presence']
                        and not compatible
                        and is_masked
                    ):
                        present_but_opposite_camera_masked_count += 1
            normalized_rows.append((scenario_id, labels))
        except (CapturePackageError, ValueError) as exc:
            validation_failures[scenario_id] = str(exc)

    gallery_checks = {
        'required': require_gallery,
        'manifest_exists': False,
        'manifest_passed': False,
        'render_counts_match': False,
        'rendered_identities_match': False,
        'violations': [],
    }
    manifest_path = root / (
        'canary_gallery_manifest.json'
        if scope == 'canary'
        else 'production_review_gallery_manifest.json'
    )
    if manifest_path.is_file():
        gallery_checks['manifest_exists'] = True
        gallery = _load_object(manifest_path)
        gallery_checks['manifest_passed'] = gallery.get('passed') is True
        gallery_counts_match = True
        gallery_identities_match = True
        for scenario in gallery.get('scenarios') or []:
            scenario_id = str(scenario.get('scenario_id') or '')
            for camera in CAMERAS:
                view = (scenario.get('views') or {}).get(camera) or {}
                expected_ids = (
                    rendered_by_scenario.get(scenario_id, {}).get(camera)
                )
                if expected_ids is None:
                    continue
                actual_ids = view.get('rendered_identities')
                actual_count = view.get('rendered_rectangle_count')
                if actual_count != len(expected_ids):
                    gallery_counts_match = False
                    gallery_checks['violations'].append(
                        f'{scenario_id}:{camera}:count'
                    )
                if actual_ids != expected_ids:
                    gallery_identities_match = False
                    gallery_checks['violations'].append(
                        f'{scenario_id}:{camera}:identities'
                    )
        gallery_checks['render_counts_match'] = gallery_counts_match
        gallery_checks['rendered_identities_match'] = (
            gallery_identities_match
        )

    legacy = _legacy_renderer_summary(normalized_rows)
    expected_slots = len(scenarios) * len(GLOBAL_IDENTITIES)
    after_violation_count = sum(
        values['opposite_rail_bbox_violation_count']
        for values in per_camera.values()
    )
    after_empty_count = sum(
        values['empty_placeholder_count']
        for values in per_camera.values()
    )
    checks = {
        'all_scope_events_valid': (
            len(normalized_rows) == len(scenarios)
            and not validation_failures
        ),
        'fixed_global_identity_order': all(
            [shuttle['id'] for shuttle in labels['shuttles']]
            == list(GLOBAL_IDENTITIES)
            for _, labels in normalized_rows
        ),
        'fixed_vector_dimension_200': vectorizer.dim == 200,
        'left_has_zero_valid_right_boxes': (
            per_camera['left_rail_rgb'][
                'opposite_rail_bbox_violation_count'
            ] == 0
        ),
        'right_has_zero_valid_left_boxes': (
            per_camera['right_rail_rgb'][
                'opposite_rail_bbox_violation_count'
            ] == 0
        ),
        'every_valid_bbox_is_camera_compatible': (
            after_violation_count == 0
        ),
        'all_unavailable_bbox_targets_masked': all(
            values['valid_bbox_count'] + values['masked_bbox_count']
            == expected_slots
            for values in per_camera.values()
        ),
        'masked_boxes_excluded_from_training_loss': all(
            values['bbox_training_mask_violation_count'] == 0
            for values in per_camera.values()
        ),
        'masked_boxes_not_rendered': all(
            values['rendered_rectangle_count']
            == values['valid_bbox_count']
            for values in per_camera.values()
        ),
        'no_empty_placeholder_boxes': after_empty_count == 0,
        'maximum_four_boxes_per_camera': all(
            values['maximum_valid_bbox_count_in_one_scenario'] <= 4
            for values in per_camera.values()
        ),
        'global_presence_independent_of_camera_visibility': (
            present_but_opposite_camera_masked_count > 0
        ),
        'gallery_contract_valid_if_required': (
            not require_gallery
            or (
                gallery_checks['manifest_exists']
                and gallery_checks['manifest_passed']
                and gallery_checks['render_counts_match']
                and gallery_checks['rendered_identities_match']
                and not gallery_checks['violations']
            )
        ),
    }
    return {
        'schema_version': CAMERA_BBOX_AUDIT_SCHEMA,
        'scope': scope,
        'passed': all(checks.values()),
        'checks': checks,
        'classification': {
            'overlay_bug': True,
            'oracle_bbox_geometry_bug': False,
            'label_contract_ambiguity': True,
            'paired_training_mask_bug': False,
            'camera_specific_training_mask_support_was_missing': True,
            'audit_bug': True,
            'evidence': (
                'capture projected each shuttle with cameras[side], so the '
                'global bbox is a canonical own-rail-camera box; the old '
                'renderer reused it on both views and no audit rejected that'
            ),
        },
        'scenario_count': len(scenarios),
        'expected_fixed_slots_per_camera': expected_slots,
        'raw_label_metadata': {
            'explicit_camera_observation_slot_count': (
                raw_explicit_camera_observation_slots
            ),
            'missing_camera_observation_slot_count': (
                raw_missing_camera_observation_slots
            ),
        },
        'before': legacy,
        'after': {
            'per_camera': per_camera,
            'rendered_rectangle_count': sum(
                values['rendered_rectangle_count']
                for values in per_camera.values()
            ),
            'opposite_rail_bbox_violation_count': after_violation_count,
            'empty_placeholder_count': after_empty_count,
            'present_but_opposite_camera_masked_count': (
                present_but_opposite_camera_masked_count
            ),
        },
        'gallery_validation': gallery_checks,
        'required_scenario_examples': _example_subsets(scenarios),
        'validation_failures': validation_failures,
        'fixed_schema': {
            'visual_state_schema': VISUAL_STATE_SCHEMA_VERSION,
            'identity_order': list(GLOBAL_IDENTITIES),
            'vectorizer_dimension': vectorizer.dim,
        },
    }


def _camera_bbox_audit_markdown(audit: dict[str, Any]) -> str:
    lines = [
        '# Room 315 per-camera bounding-box semantics audit',
        '',
        f'Verdict: **{"PASS" if audit["passed"] else "FAIL"}**',
        '',
        (
            'The captured canonical bbox geometry was generated from each '
            'shuttle rail’s own camera. The defect was the cross-camera '
            'gallery renderer, missing explicit per-camera label/mask '
            'semantics, and missing fail-closed audit coverage.'
        ),
        '',
        '| Camera | Valid boxes | Masked boxes | Opposite violations | '
        'Empty placeholders | Rendered rectangles | Maximum/scenario |',
        '|---|---:|---:|---:|---:|---:|---:|',
    ]
    for camera, values in audit['after']['per_camera'].items():
        lines.append(
            f'| {camera} | {values["valid_bbox_count"]} | '
            f'{values["masked_bbox_count"]} | '
            f'{values["opposite_rail_bbox_violation_count"]} | '
            f'{values["empty_placeholder_count"]} | '
            f'{values["rendered_rectangle_count"]} | '
            f'{values["maximum_valid_bbox_count_in_one_scenario"]} |'
        )
    lines.extend([
        '',
        '## Before/after',
        '',
        (
            '- Opposite-rail rectangles: '
            f'{audit["before"]["opposite_rail_bbox_violation_count"]} → '
            f'{audit["after"]["opposite_rail_bbox_violation_count"]}'
        ),
        (
            '- Empty placeholder rectangles: '
            f'{audit["before"]["empty_placeholder_count"]} → '
            f'{audit["after"]["empty_placeholder_count"]}'
        ),
        (
            '- Fixed schema: '
            f'{audit["fixed_schema"]["visual_state_schema"]}, '
            f'{audit["fixed_schema"]["vectorizer_dimension"]} dimensions, '
            + ', '.join(audit['fixed_schema']['identity_order'])
        ),
        '',
        'All eight global slots remain present in metadata. Per-camera bbox '
        'availability and masks are independent from global presence.',
        '',
    ])
    return '\n'.join(lines)


def _source_image_hashes(
    root: Path,
    scenarios: list[dict[str, Any]],
) -> dict[str, str]:
    hashes = {}
    for scenario in scenarios:
        for camera in CAMERAS:
            path = (
                root / 'dataset' / 'episodes'
                / scenario['scenario_id'] / 'images' / camera
                / 'frame_000000.jpg'
            )
            if not path.is_file():
                raise CapturePackageError(f'missing source image: {path}')
            hashes[str(path.relative_to(root))] = sha256(path)
    return dict(sorted(hashes.items()))


def relabel_camera_bboxes(root: Path) -> dict[str, Any]:
    """Add explicit, derivable per-camera bbox metadata to captured canary."""
    correction_path = root / 'camera_bbox_semantics_correction_log.json'
    if correction_path.exists():
        raise CapturePackageError(
            'camera bbox correction log already exists; refusing to relabel twice'
        )
    approval = _load_object(root / 'production_capture_approval.json')
    if approval.get('approved_for_canary_capture') is not True:
        raise CapturePackageError('canary capture approval is not recorded')
    for field in (
        'approved_after_canary_gallery_review',
        'approved_for_full_capture',
        'approved_after_full_gallery_review',
        'approved_for_training',
    ):
        if approval.get(field) is not False:
            raise CapturePackageError(
                f'{field} must remain false during bbox correction'
            )
    status = status_report(root)
    if (
        status['completed_scenarios'] != CANARY_COUNT
        or not status['canary_complete']
        or status['capture_complete']
    ):
        raise CapturePackageError(
            'bbox correction is restricted to the completed 64-scenario '
            'canary before full capture'
        )
    scenarios = _scope_scenarios(root, 'canary')
    source_before = _source_image_hashes(root, scenarios)
    before = camera_bbox_semantics_audit(
        root,
        'canary',
        require_gallery=False,
    )
    if before['validation_failures']:
        raise CapturePackageError(
            'existing canary labels cannot be safely normalized'
        )

    staged_events: dict[Path, str] = {}
    staged_validations: dict[Path, str] = {}
    corrected_rows = []
    corrections = []
    for scenario in scenarios:
        scenario_id = scenario['scenario_id']
        episode = root / 'dataset' / 'episodes' / scenario_id
        event_path = episode / 'event.json'
        validation_path = episode / 'validation.json'
        event = _load_object(event_path)
        validation = _load_object(validation_path)
        original_event_sha = sha256(event_path)
        original_validation_sha = sha256(validation_path)
        labels = normalize_visual_state_labels(
            event,
            context=f'camera bbox relabel {scenario_id}',
        )
        VisualStateLabelVectorizer().validate_target(labels)
        for camera in CAMERAS:
            _camera_render_plan(labels, camera)
        event['visual_state_labels'] = labels
        event['camera_bbox_semantics'] = {
            'schema_version': CAMERA_BBOX_CORRECTION_SCHEMA,
            'derivation': (
                'canonical own-rail bbox retained; opposite-rail camera '
                'availability and bbox target mask set to zero'
            ),
            'source_oracle_geometry_changed': False,
            'source_images_changed': False,
        }
        validation['camera_bbox_semantics'] = {
            'validated': True,
            'left_rail_rgb_identities': list(GLOBAL_IDENTITIES[:4]),
            'right_rail_rgb_identities': list(GLOBAL_IDENTITIES[4:]),
            'fixed_identity_order': list(GLOBAL_IDENTITIES),
            'fixed_vector_dimension': 200,
        }
        event_text = json.dumps(event, indent=2, sort_keys=True) + '\n'
        validation_text = (
            json.dumps(validation, indent=2, sort_keys=True) + '\n'
        )
        staged_events[event_path] = event_text
        staged_validations[validation_path] = validation_text
        corrected_rows.append(event)
        corrections.append({
            'scenario_id': scenario_id,
            'event_path': str(event_path.relative_to(root)),
            'event_sha256_before': original_event_sha,
            'event_sha256_after': hashlib.sha256(
                event_text.encode('utf-8')
            ).hexdigest(),
            'validation_path': str(validation_path.relative_to(root)),
            'validation_sha256_before': original_validation_sha,
            'validation_sha256_after': hashlib.sha256(
                validation_text.encode('utf-8')
            ).hexdigest(),
            'global_present_identities': _active_ids(scenario),
        })

    aggregate_path = root / 'dataset' / 'meta' / 'training_events.jsonl'
    aggregate_text = ''.join(
        _canonical(row) + '\n' for row in corrected_rows
    )
    original_bytes = {
        path: path.read_bytes()
        for path in (
            *staged_events,
            *staged_validations,
            aggregate_path,
        )
    }
    try:
        with CaptureLock(root):
            for path, text_value in staged_events.items():
                _atomic_text(path, text_value)
            for path, text_value in staged_validations.items():
                _atomic_text(path, text_value)
            _atomic_text(aggregate_path, aggregate_text)
            after = camera_bbox_semantics_audit(
                root,
                'canary',
                require_gallery=False,
            )
            source_after = _source_image_hashes(root, scenarios)
            if not after['passed']:
                raise CapturePackageError(
                    'corrected labels failed camera bbox audit'
                )
            if source_after != source_before:
                raise CapturePackageError(
                    'source image hashes changed during metadata correction'
                )
            if after['raw_label_metadata'][
                'missing_camera_observation_slot_count'
            ] != 0:
                raise CapturePackageError(
                    'corrected labels still lack explicit camera observations'
                )
            log = {
                'schema_version': CAMERA_BBOX_CORRECTION_SCHEMA,
                'passed': True,
                'method': (
                    'metadata-only deterministic relabel from trusted '
                    'own-rail-camera oracle bbox geometry'
                ),
                'recapture_required': False,
                'scenario_count': len(scenarios),
                'source_image_count': len(source_before),
                'source_images_byte_identical': True,
                'source_image_hashes_before': source_before,
                'source_image_hashes_after': source_after,
                'aggregate_path': str(aggregate_path.relative_to(root)),
                'aggregate_sha256_after': hashlib.sha256(
                    aggregate_text.encode('utf-8')
                ).hexdigest(),
                'before_summary': before['before'],
                'after_summary': after['after'],
                'corrections': corrections,
            }
            _atomic_json(correction_path, log)
            _atomic_json(
                root / 'camera_bbox_semantics_audit.json',
                after,
            )
            _atomic_text(
                root / 'camera_bbox_semantics_audit.md',
                _camera_bbox_audit_markdown(after),
            )
            return log
    except BaseException:
        for path, content in original_bytes.items():
            _atomic_text(path, content.decode('utf-8'))
        raise


def _overlay(
    source: Path,
    labels: dict[str, Any],
    target: Path,
    camera: str,
) -> dict[str, Any]:
    with Image.open(source) as opened:
        image = opened.convert('RGB')
    draw = ImageDraw.Draw(image)
    plan = _camera_render_plan(labels, camera)
    for item in plan:
        x, y, width, height = item['bbox']
        draw.rectangle(
            (x, y, x + width, y + height),
            outline=(0, 255, 0),
            width=3,
        )
        draw.text(
            (x + 2, y + 2),
            item['identity'],
            fill=(255, 255, 0),
        )
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix('.tmp.png')
    image.save(temporary)
    os.replace(temporary, target)
    status = {}
    for shuttle in labels['shuttles']:
        observation = camera_observation_for_shuttle(shuttle, camera)
        status[shuttle['id']] = (
            'absent'
            if not shuttle['presence']
            else 'not_applicable_to_camera'
            if not observation['applicable']
            else 'visible_with_bbox'
            if observation['visual_available']
            else 'not_visible_in_camera'
        )
    return {
        'camera': camera,
        'rendered_rectangle_count': len(plan),
        'rendered_identities': [item['identity'] for item in plan],
        'identity_camera_status': status,
    }


def _full_gallery_ids(
    root: Path,
    rows: list[dict[str, Any]],
) -> list[str]:
    selected = {}
    for row in rows:
        selected.setdefault(row['configuration_id'], row['scenario_id'])
    for row in rows:
        active = _active_ids(row)
        left = row['left_active_identities']
        right = row['right_active_identities']
        nonprefix_pair = (
            len(active) == 2
            and (not left or not right)
            and active not in [
                ['L1', 'L2'],
                ['R1', 'R2'],
            ]
        )
        risk = bool(
            row['static_camera_projectability'][
                'partial_occlusion_risk_pairs'
            ]
        )
        high_color = (
            {'R4', 'L3'} <= set(active)
            or {'L1', 'R2'} <= set(active)
            or 'L4' in active
        )
        if len(active) == 1 or nonprefix_pair or risk or high_color:
            selected.setdefault(row['scenario_id'], row['scenario_id'])
    state = _load_object(root / 'capture_state.json')
    for failure in state.get('historical_failures') or []:
        scenario_id = str(failure.get('scenario_id') or '')
        if scenario_id:
            selected[scenario_id] = scenario_id
    required_groups = [
        ('relation_family', EXPECTED_RELATION_TOTALS),
        ('target_zone', EXPECTED_ZONE_TOTALS),
    ]
    for field, values in required_groups:
        for value in values:
            row = next(item for item in rows if item[field] == value)
            selected[row['scenario_id']] = row['scenario_id']
    for count in range(1, 9):
        row = next(item for item in rows if len(_active_ids(item)) == count)
        selected[row['scenario_id']] = row['scenario_id']
    return sorted(set(selected.values()))


def create_gallery(root: Path, scope: str) -> dict[str, Any]:
    audit_path = root / (
        'captured_canary_audit.json'
        if scope == 'canary'
        else 'captured_production_audit.json'
    )
    if (
        not audit_path.is_file()
        or _load_object(audit_path).get('passed') is not True
    ):
        raise CapturePackageError(
            f'{scope} captured audit must PASS before gallery generation'
        )
    rows = _manifest_rows(root)
    by_id = {row['scenario_id']: row for row in rows}
    selected_ids = (
        _canary_ids(root)
        if scope == 'canary'
        else _full_gallery_ids(root, rows)
    )
    overlay_root = root / (
        'canary_review_overlays'
        if scope == 'canary'
        else 'manual_review_overlays'
    )
    staging = Path(tempfile.mkdtemp(
        prefix=f'.{overlay_root.name}.',
        dir=root,
    ))
    source_before = {}
    scenarios = []
    try:
        for scenario_id in selected_ids:
            scenario = by_id[scenario_id]
            event = _load_object(
                root / 'dataset' / 'episodes' / scenario_id / 'event.json'
            )
            labels = normalize_visual_state_labels(event)
            views = {}
            for camera in CAMERAS:
                source = (
                    root
                    / 'dataset'
                    / 'episodes'
                    / scenario_id
                    / 'images'
                    / camera
                    / 'frame_000000.jpg'
                )
                source_before[str(source)] = sha256(source)
                target = staging / scenario_id / f'{camera}_overlay.png'
                render_metadata = _overlay(
                    source,
                    labels,
                    target,
                    camera,
                )
                views[camera] = {
                    'source': str(source.relative_to(root)),
                    'source_sha256': source_before[str(source)],
                    'overlay': str(
                        (overlay_root / scenario_id / target.name).relative_to(root)
                    ),
                    'overlay_sha256': sha256(target),
                    **render_metadata,
                }
            scenarios.append({
                'scenario_id': scenario_id,
                'v2_plan_id': scenario['v2_plan_id'],
                'presence_configuration_id': scenario[
                    'presence_configuration_id'
                ],
                'left_active_identities': scenario[
                    'left_active_identities'
                ],
                'right_active_identities': scenario[
                    'right_active_identities'
                ],
                'absent_identities': scenario['inactive_identities'],
                'target_identity': scenario['target_identity'],
                'relation_family': scenario['relation_family'],
                'relation_identities': scenario['relation_identities'],
                'payload_assignment': scenario['payload_assignment'],
                'positions': scenario['oracle_expectations']['positions'],
                'target_zone': scenario['target_zone'],
                'switches': {
                    side: scenario['scene']['rails'][side]['switches']
                    for side in ('left', 'right')
                },
                'identity_colors': {
                    identity: scenario['static_camera_projectability'][
                        'active_identities'
                    ][identity]['identity_color']
                    for identity in _active_ids(scenario)
                },
                'views': views,
            })
        source_after = {
            path: sha256(Path(path)) for path in source_before
        }
        changed = sorted(
            path for path in source_before
            if source_before[path] != source_after[path]
        )
        if changed:
            raise CapturePackageError(
                f'gallery modified source images: {changed[:20]}'
            )
        if overlay_root.exists():
            shutil.rmtree(overlay_root)
        os.replace(staging, overlay_root)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    gallery = {
        'schema_version': 'room315.production_review_gallery.v2',
        'scope': scope,
        'passed': True,
        'scenario_count': len(scenarios),
        'source_image_count': len(source_before),
        'overlay_image_count': len(source_before),
        'source_images_unchanged': True,
        'camera_bbox_contract': {
            'left_rail_rgb': list(GLOBAL_IDENTITIES[:4]),
            'right_rail_rgb': list(GLOBAL_IDENTITIES[4:]),
            'global_presence_is_independent': True,
            'maximum_rendered_boxes_per_camera': 4,
        },
        'selection': (
            'all exact 64 canary IDs'
            if scope == 'canary'
            else (
                'deterministic stratification: every configuration, '
                'singletons, non-prefix pairs, color/occlusion risks, '
                'failures, relations, zones, cardinalities, and 4+4'
            )
        ),
        'scenarios': scenarios,
    }
    manifest_name = (
        'canary_gallery_manifest.json'
        if scope == 'canary'
        else 'production_review_gallery_manifest.json'
    )
    html_name = (
        'canary_gallery.html'
        if scope == 'canary'
        else 'production_review_gallery.html'
    )
    _atomic_json(root / manifest_name, gallery)
    cards = []
    for row in scenarios:
        views = ''.join(
            f'<figure><img src="{html.escape(view["overlay"])}">'
            f'<figcaption>{html.escape(camera)}</figcaption></figure>'
            for camera, view in row['views'].items()
        )
        cards.append(
            '<section><h2>'
            + html.escape(row['scenario_id'])
            + '</h2><pre>'
            + html.escape(json.dumps({
                key: value
                for key, value in row.items()
                if key != 'views'
            }, indent=2, sort_keys=True))
            + '</pre><div class="views">'
            + views
            + '</div></section>'
        )
    _atomic_text(
        root / html_name,
        '<!doctype html><meta charset="utf-8"><title>Room 315 review</title>'
        '<style>body{background:#111;color:#eee;font-family:sans-serif}'
        'section{border:1px solid #555;margin:1em;padding:1em}'
        '.views{display:grid;grid-template-columns:1fr 1fr;gap:1em}'
        'img{max-width:100%}pre{white-space:pre-wrap}</style>'
        '<h1>Human review required; no approval is automatic.</h1>'
        + ''.join(cards),
    )
    camera_audit = camera_bbox_semantics_audit(
        root,
        scope,
        require_gallery=True,
    )
    if not camera_audit['passed']:
        raise CapturePackageError(
            'generated gallery failed per-camera bbox semantics audit'
        )
    camera_audit_name = (
        'camera_bbox_semantics_audit.json'
        if scope == 'canary'
        else 'production_camera_bbox_semantics_audit.json'
    )
    camera_audit_md_name = (
        'camera_bbox_semantics_audit.md'
        if scope == 'canary'
        else 'production_camera_bbox_semantics_audit.md'
    )
    _atomic_json(root / camera_audit_name, camera_audit)
    _atomic_text(
        root / camera_audit_md_name,
        _camera_bbox_audit_markdown(camera_audit),
    )
    return gallery


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest='command')
    prepare = commands.add_parser('prepare')
    prepare.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    prepare.add_argument('--declared-root', type=Path)
    for name in ('validate-static', 'status'):
        command = commands.add_parser(name)
        command.add_argument('--package-root', type=Path)
    capture = commands.add_parser('capture')
    capture.add_argument('--package-root', type=Path)
    capture.add_argument('--mode', choices=('canary', 'full'), required=True)
    capture.add_argument('runner_args', nargs=argparse.REMAINDER)
    resume = commands.add_parser('resume')
    resume.add_argument('--package-root', type=Path)
    resume.add_argument('runner_args', nargs=argparse.REMAINDER)
    audit = commands.add_parser('audit-captured')
    audit.add_argument('--package-root', type=Path)
    audit.add_argument('--scope', choices=('canary', 'full'), required=True)
    audit.add_argument('--output', type=Path, required=True)
    gallery = commands.add_parser('gallery')
    gallery.add_argument('--package-root', type=Path)
    gallery.add_argument('--scope', choices=('canary', 'full'), required=True)
    bbox_audit = commands.add_parser('audit-camera-bboxes')
    bbox_audit.add_argument('--package-root', type=Path)
    bbox_audit.add_argument(
        '--scope',
        choices=('canary', 'full'),
        required=True,
    )
    bbox_audit.add_argument('--require-gallery', action='store_true')
    bbox_audit.add_argument('--output-json', type=Path)
    bbox_audit.add_argument('--output-md', type=Path)
    relabel = commands.add_parser('relabel-camera-bboxes')
    relabel.add_argument('--package-root', type=Path)
    verify = commands.add_parser('verify-package')
    verify.add_argument('--package-root', type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        argv = ['status', '--package-root', str(Path(__file__).resolve().parent)]
    args = _build_parser().parse_args(argv)
    if args.command == 'prepare':
        output = prepare_package(
            args.output,
            declared_root=args.declared_root,
        )
        print(f'CAPTURE_V2_PACKAGE_PREPARED {output}')
        return 0
    root = (
        (args.package_root or Path(__file__).resolve().parent)
        .expanduser()
        .resolve()
    )
    if args.command == 'validate-static':
        report = validate_static_package(root)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if report['passed'] else 2
    if args.command == 'status':
        print(json.dumps(status_report(root), indent=2, sort_keys=True))
        return 0
    if args.command == 'capture':
        return run_capture(
            root,
            mode=args.mode,
            extra_args=args.runner_args,
        )
    if args.command == 'resume':
        return resume_capture(root, args.runner_args)
    if args.command == 'audit-captured':
        report = captured_audit(root, args.scope)
        _atomic_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if report['passed'] else 3
    if args.command == 'gallery':
        report = create_gallery(root, args.scope)
        print(json.dumps({
            key: value for key, value in report.items()
            if key != 'scenarios'
        }, indent=2, sort_keys=True))
        return 0
    if args.command == 'audit-camera-bboxes':
        report = camera_bbox_semantics_audit(
            root,
            args.scope,
            require_gallery=args.require_gallery,
        )
        if args.output_json:
            _atomic_json(args.output_json, report)
        if args.output_md:
            _atomic_text(
                args.output_md,
                _camera_bbox_audit_markdown(report),
            )
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if report['passed'] else 4
    if args.command == 'relabel-camera-bboxes':
        report = relabel_camera_bboxes(root)
        print(json.dumps({
            key: value
            for key, value in report.items()
            if key not in {
                'corrections',
                'source_image_hashes_before',
                'source_image_hashes_after',
            }
        }, indent=2, sort_keys=True))
        return 0
    if args.command == 'verify-package':
        report = verify_package(root)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if report['passed'] else 2
    raise AssertionError(args.command)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (
        CapturePackageError,
        FileExistsError,
        OSError,
        ValueError,
    ) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
