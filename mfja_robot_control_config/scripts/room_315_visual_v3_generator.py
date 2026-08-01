#!/usr/bin/env python3
"""Materialise deterministic, collision-free Room 315 visual V3 manifests."""

from __future__ import annotations

import argparse
import copy
import itertools
import json
import random
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_arbitrary_subset_visual import ARBITRARY_IDENTITY_PRESENCE_PROFILE
from room_315_arbitrary_subset_visual import NO_RELATION
from room_315_arbitrary_subset_visual_v2 import _identity_visual_contract
from room_315_arbitrary_subset_visual_v2 import _default_camera_model_path
from room_315_arbitrary_subset_visual_v2 import scenario_projectability
from room_315_rail_defaults import public_rail_segment_lengths
from room_315_visual_label_exporter import load_camera_projections
from room_315_visual_scenario_generator import MAX_PHYSICAL_PLACEMENT_ATTEMPTS
from room_315_visual_scenario_generator import REQUIRED_CAMERAS
from room_315_visual_scenario_generator import SCHEMA_VERSION
from room_315_visual_scenario_generator import SWITCH_NAMES
from room_315_visual_scenario_generator import _blocker_positions
from room_315_visual_scenario_generator import _launch_arguments
from room_315_visual_scenario_generator import _place_neutral_shuttles
from room_315_visual_scenario_generator import _relation_roles
from room_315_visual_scenario_generator import _switch_command
from room_315_visual_scenario_generator import scenario_physical_conflicts
from room_315_visual_scenario_generator import validate_scenario
from room_315_visual_v3_common import BLOCKS
from room_315_visual_v3_common import DEFAULT_CANARY_ROOT
from room_315_visual_v3_common import DEFAULT_CAPTURE_ROOT
from room_315_visual_v3_common import DEFAULT_GUARD_ROOT
from room_315_visual_v3_common import DEFAULT_OLD_TRAIN
from room_315_visual_v3_common import DEFAULT_OLD_TRAIN_LABELS
from room_315_visual_v3_common import IDENTITIES
from room_315_visual_v3_common import PACKAGE_SCHEMA
from room_315_visual_v3_common import POSITION_BINS
from room_315_visual_v3_common import POSITION_RATIOS
from room_315_visual_v3_common import RENDER_BUCKETS
from room_315_visual_v3_common import SCENARIO_SCHEMA
from room_315_visual_v3_common import SEED
from room_315_visual_v3_common import VISUAL_SCHEMA
from room_315_visual_v3_common import VisualV3Error
from room_315_visual_v3_common import atomic_json
from room_315_visual_v3_common import atomic_jsonl
from room_315_visual_v3_common import configuration_family_id
from room_315_visual_v3_common import position_bin
from room_315_visual_v3_common import prepare_output_root
from room_315_visual_v3_common import sha256_file
from room_315_visual_v3_common import side_for_identity
from room_315_visual_v3_common import stable_int
from room_315_visual_v3_common import value_sha256
from room_315_visual_v3_quota_planner import CANARY_COUNT
from room_315_visual_v3_quota_planner import TRAIN_COUNT
from room_315_visual_v3_quota_planner import VALIDATION_COUNT
from room_315_visual_v3_quota_planner import build_specs
from room_315_visual_v3_quota_planner import quota_plan


GENERATOR_VERSION = 'room315.visual_v3_generator.v1'
REPO_ROOT = Path(__file__).resolve().parents[2]
SEGMENT_LENGTHS = {
    side: public_rail_segment_lengths(side)
    for side in ('left', 'right')
}


def _augment_position(side: str, position: dict[str, Any]) -> dict[str, Any]:
    segment = str(position['segment'])
    if segment not in SEGMENT_LENGTHS[side]:
        raise VisualV3Error(f'{side}:{segment} is not an authoritative public block')
    ratio = round(float(position['s_ratio']), 9)
    if not 0.0 <= ratio <= 1.0:
        raise VisualV3Error(f'{side}:{segment} ratio outside [0,1]: {ratio}')
    length = float(SEGMENT_LENGTHS[side][segment])
    result = dict(position)
    result.update({
        'segment': segment,
        's_ratio': ratio,
        'segment_length_m': round(length, 9),
        's_m': round(ratio * length, 9),
    })
    return result


def _branch_for_segment(segment: str) -> str:
    if segment.endswith('I'):
        return 'interior'
    return 'exterior'


def _relation_positions(
    spec: dict[str, Any],
    *,
    attempt: int,
) -> tuple[dict[str, dict[str, Any]], list[dict[str, str]], str]:
    target = str(spec['target_identity'])
    side = side_for_identity(target)
    family = str(spec['relation_family'])
    geometry_index = int(
        spec.get('geometry_seed_index', spec['generation_index'])
    )
    geometry_key = str(spec.get('geometry_seed_key') or geometry_index)
    active_side = [
        identity for identity in spec['active_identities']
        if side_for_identity(identity) == side
    ]
    roles = _relation_roles(family)
    peers = [identity for identity in active_side if identity != target]
    peers.sort(
        key=lambda identity: stable_int(
            SEED,
            geometry_key,
            attempt,
            'relation-peer',
            identity,
        )
    )
    if len(peers) < len(roles):
        raise VisualV3Error(
            f'{spec["spec_id"]}: insufficient same-rail relation peers'
        )
    positions, relations, branch = _blocker_positions(
        family,
        side=side,
        type_index=geometry_index + attempt * 5000,
        target_zone=str(spec['target_zone']),
        seed=SEED + geometry_index + attempt,
        dataset_seed=SEED,
    )
    selected = peers[:len(roles)]
    mapped = {target: positions[0]}
    remapped = []
    for identity, position, relation in zip(selected, positions[1:], relations):
        mapped[identity] = position
        relation_record = dict(relation)
        relation_record['other_shuttle_id'] = identity
        remapped.append(relation_record)
    return mapped, remapped, branch


def _place_positions(
    spec: dict[str, Any],
    *,
    attempt: int,
) -> tuple[dict[str, dict[str, dict[str, Any]]], list[dict[str, str]], str]:
    target = str(spec['target_identity'])
    target_side = side_for_identity(target)
    geometry_index = int(
        spec.get('geometry_seed_index', spec['generation_index'])
    )
    positions = {'left': {}, 'right': {}}
    relations: list[dict[str, str]] = []
    if spec.get('position_overrides'):
        for identity, raw_position in spec['position_overrides'].items():
            if identity not in spec['active_identities']:
                continue
            positions[side_for_identity(identity)][identity] = {
                'segment': str(raw_position['segment']),
                's_ratio': float(raw_position['s_ratio']),
                'position_zone': (
                    spec['target_zone']
                    if identity == target
                    else str(
                        raw_position.get('position_zone')
                        or 'relation_neutral'
                    )
                ),
            }
        relations = copy.deepcopy(spec.get('explicit_relations') or [])
        active_branch = _branch_for_segment(
            positions[target_side][target]['segment']
        )
    elif spec['relation_family'] == NO_RELATION:
        positions[target_side][target] = {
            'segment': str(spec['target_block']),
            's_ratio': float(spec['target_s_ratio']),
            'position_zone': str(spec['target_zone']),
        }
        active_branch = _branch_for_segment(str(spec['target_block']))
    else:
        relation_positions, relations, active_branch = _relation_positions(
            spec,
            attempt=attempt,
        )
        positions[target_side].update(relation_positions)

    for side in ('left', 'right'):
        remaining = tuple(
            identity for identity in spec['active_identities']
            if side_for_identity(identity) == side and identity not in positions[side]
        )
        _place_neutral_shuttles(
            side,
            remaining,
            positions[side],
            ordinal=geometry_index + 1 + attempt * 5000,
            dataset_seed=SEED,
            active_branch=(active_branch if side == target_side else None),
            target_segment=(
                str(positions[target_side][target]['segment'])
                if side == target_side
                else None
            ),
            covered_segments=set(),
        )
    return positions, relations, active_branch


def _scenario_metadata(
    spec: dict[str, Any],
    positions: dict[str, dict[str, dict[str, Any]]],
) -> dict[str, Any]:
    blocks = {}
    bins = {}
    s_m = {}
    s_ratio = {}
    lengths = {}
    zones = {}
    for side in ('left', 'right'):
        for identity, position in positions[side].items():
            augmented = _augment_position(side, position)
            blocks[identity] = augmented['segment']
            bins[identity] = position_bin(augmented['s_ratio'])
            s_m[identity] = augmented['s_m']
            s_ratio[identity] = augmented['s_ratio']
            lengths[identity] = augmented['segment_length_m']
            zones[identity] = str(position.get('position_zone') or 'ordinary')
    record = {
        'active_identities': list(spec['active_identities']),
        'loaded_identities': list(spec['loaded_identities']),
        'identity_to_block': blocks,
        'identity_to_position_bin': bins,
        'relation_family': spec['relation_family'],
        'target_identity': spec['target_identity'],
        'target_zone': spec['target_zone'],
        'target_offset_bucket': spec['target_offset_bucket'],
        'approach_direction': spec['approach_direction'],
        'occlusion_class': spec['occlusion_class'],
        'render_bucket': spec['render_bucket'],
    }
    return {
        **record,
        'identity_to_s_m': s_m,
        'identity_to_s_ratio': s_ratio,
        'identity_to_segment_length_m': lengths,
        'identity_to_zone': zones,
        'configuration_family_id': configuration_family_id(record),
        'configuration_core_family_id': configuration_family_id(
            record,
            include_render=False,
        ),
    }


def materialize_spec(
    spec: dict[str, Any],
    *,
    cameras: dict[str, Any] | None = None,
    identity_contract: dict[str, dict[str, Any]] | None = None,
) -> dict[str, Any]:
    if set(spec['active_identities']) - set(IDENTITIES):
        raise VisualV3Error(f'{spec["spec_id"]}: invalid active identity')
    if set(spec['payload_assignment']) != set(spec['active_identities']):
        raise VisualV3Error(f'{spec["spec_id"]}: payload assignment mismatch')
    cameras = cameras or load_camera_projections(_default_camera_model_path())
    identity_contract = identity_contract or _identity_visual_contract()
    last_error: Exception | None = None
    for attempt in range(MAX_PHYSICAL_PLACEMENT_ATTEMPTS):
        try:
            positions, relations, active_branch = _place_positions(spec, attempt=attempt)
            metadata = _scenario_metadata(spec, positions)
            target = str(spec['target_identity'])
            target_side = side_for_identity(target)
            rails = {}
            for side in ('left', 'right'):
                branch = active_branch if side == target_side else 'exterior'
                switches = dict(zip(SWITCH_NAMES, (branch,) * len(SWITCH_NAMES)))
                shuttles = []
                for identity in IDENTITIES:
                    if identity not in positions[side]:
                        continue
                    shuttles.append({
                        'id': identity,
                        'start_slot': int(identity[1:]),
                        'start_position': _augment_position(
                            side,
                            positions[side][identity],
                        ),
                        'loaded_state': spec['payload_assignment'][identity],
                    })
                rails[side] = {
                    'switch_pattern': f'all_{branch}',
                    'switches': switches,
                    'shuttles': shuttles,
                }
            launch_arguments = _launch_arguments(rails)
            launch_arguments.update({
                'room315_identity_selection_mode': 'explicit',
                'room315_left_active_identities': ','.join(
                    identity for identity in spec['active_identities']
                    if side_for_identity(identity) == 'left'
                ),
                'room315_right_active_identities': ','.join(
                    identity for identity in spec['active_identities']
                    if side_for_identity(identity) == 'right'
                ),
            })
            geometry = {
                side: [
                    {
                        'id': shuttle['id'],
                        'block': shuttle['start_position']['segment'],
                        's_ratio': shuttle['start_position']['s_ratio'],
                    }
                    for shuttle in rails[side]['shuttles']
                ]
                for side in ('left', 'right')
            }
            geometry_sha = value_sha256(geometry)
            capture_configuration_sha = value_sha256({
                'geometry_sha256': geometry_sha,
                'payload_assignment': spec['payload_assignment'],
                'render_bucket': spec['render_bucket'],
            })
            capture_family_sha = value_sha256({
                'capture_configuration_sha256': capture_configuration_sha,
                'spec_id': spec['spec_id'],
            })
            scenario_id = (
                f'hard_v3_{spec["profile"]}_{spec["generation_index"] + 1:04d}_'
                f'{geometry_sha[:12]}'
            )
            relation_peers = {
                relation['other_shuttle_id'] for relation in relations
            }
            active_target_side = {
                identity for identity in spec['active_identities']
                if side_for_identity(identity) == target_side
            }
            scenario = {
                'schema_version': SCHEMA_VERSION,
                'manifest_profile_schema': PACKAGE_SCHEMA,
                'visual_label_schema': VISUAL_SCHEMA,
                'generator_version': GENERATOR_VERSION,
                'scenario_id': scenario_id,
                # The legacy capture runner requires this transport-level
                # family to be unique per manifest. Semantic grouping and
                # leakage checks use configuration_family_id below.
                'scenario_family': f'hard_v3_capture_{capture_family_sha}',
                'geometry_fingerprint': geometry_sha,
                'capture_configuration_fingerprint': capture_configuration_sha,
                'configuration_family_id': metadata['configuration_family_id'],
                'configuration_core_family_id': metadata[
                    'configuration_core_family_id'
                ],
                'scene_type': (
                    'single'
                    if spec['relation_family'] == NO_RELATION
                    else spec['relation_family']
                ),
                'relation_family': spec['relation_family'],
                'seed': SEED,
                'generation_index': int(spec['generation_index']),
                'family_avoidance_attempt': int(
                    spec.get('family_avoidance_attempt', 0)
                ),
                'spec_id': spec['spec_id'],
                'dataset_partition': spec['profile'],
                'presence_profile': ARBITRARY_IDENTITY_PRESENCE_PROFILE,
                'left_active_identities': [
                    identity for identity in spec['active_identities']
                    if side_for_identity(identity) == 'left'
                ],
                'right_active_identities': [
                    identity for identity in spec['active_identities']
                    if side_for_identity(identity) == 'right'
                ],
                'active_identities': list(spec['active_identities']),
                'loaded_identities': list(spec['loaded_identities']),
                'payload_assignment': copy.deepcopy(spec['payload_assignment']),
                'target_identity': target,
                'target_zone': spec['target_zone'],
                'target_offset_bucket': spec['target_offset_bucket'],
                'target_offset': spec['target_offset'],
                'target_ratio': spec.get('target_ratio'),
                'operational_target_name': spec.get(
                    'operational_target_name'
                ),
                'operational_target_segment': spec.get(
                    'operational_target_segment'
                ),
                'presence_class': spec.get('presence_class'),
                'configuration_variant': spec.get(
                    'configuration_variant'
                ),
                'approach_direction': spec['approach_direction'],
                'occlusion_class': spec['occlusion_class'],
                'render_variation': {
                    **copy.deepcopy(spec['render_parameters']),
                    'application': 'capture_image_postprocess',
                    'model_input_exposure': 'image_pixels_only',
                },
                'canary_family': spec['canary_family'],
                'matched_pair_id': spec.get('matched_pair_id'),
                'matched_pair_role': spec.get('matched_pair_role'),
                'hard_case_tags': list(spec.get('hard_case_tags', [])),
                **metadata,
                'scene': {'rails': rails, 'obstacles': []},
                'capture': {
                    'mode': 'settled_static',
                    'cameras': list(REQUIRED_CAMERAS),
                    'frames': 1,
                    'settle_seconds': 1.5,
                    'frame_interval_seconds': 0.25,
                },
                'setup': {
                    'launch_package': 'mfja_3rd_floor_bringup',
                    'launch_file': 'room_315_only.launch.py',
                    'launch_arguments': launch_arguments,
                    'switch_topic': '/mfja/conveyor/switch_cmd',
                    'switch_command': _switch_command(rails),
                },
                'relation_probe': {
                    'target_shuttle_id': target,
                    'side': target_side,
                    'relations': relations,
                    'relation_neutral_shuttle_ids': sorted(
                        active_target_side - {target} - relation_peers,
                        key=IDENTITIES.index,
                    ),
                    'opposite_rail_neutral_shuttle_ids': [
                        identity for identity in spec['active_identities']
                        if side_for_identity(identity) != target_side
                    ],
                    'model_input_exposure': 'excluded',
                },
                'rail_scope': ARBITRARY_IDENTITY_PRESENCE_PROFILE,
                'expected_label_coverage': {
                    'shuttle_count': len(spec['active_identities']),
                    'fixed_schema_identity_count': len(IDENTITIES),
                    'loaded_count': len(spec['loaded_identities']),
                    'empty_count': (
                        len(spec['active_identities']) - len(spec['loaded_identities'])
                    ),
                    'switch_count': len(SWITCH_NAMES) * 2,
                    'obstacle_count': 0,
                    'continuous_position_count': len(spec['active_identities']),
                },
            }
            validate_scenario(scenario)
            conflicts = scenario_physical_conflicts(scenario)
            if conflicts:
                raise VisualV3Error(f'physical conflict: {conflicts[0]}')
            projectability = scenario_projectability(
                scenario,
                cameras=cameras,
                identity_contract=identity_contract,
            )
            if not projectability['passed']:
                raise VisualV3Error(f'projectability failed: {projectability}')
            scenario['static_camera_projectability'] = projectability
            if projectability['partial_occlusion_risk_pairs']:
                scenario['occlusion_class'] = 'partial_risk'
            else:
                scenario['occlusion_class'] = 'clear'
            # Recompute the family after static occlusion classification.
            family_record = {
                'active_identities': scenario['active_identities'],
                'loaded_identities': scenario['loaded_identities'],
                'identity_to_block': scenario['identity_to_block'],
                'identity_to_position_bin': scenario['identity_to_position_bin'],
                'relation_family': scenario['relation_family'],
                'target_identity': scenario['target_identity'],
                'target_zone': scenario['target_zone'],
                'target_offset_bucket': scenario['target_offset_bucket'],
                'approach_direction': scenario['approach_direction'],
                'occlusion_class': scenario['occlusion_class'],
                'render_bucket': scenario['render_variation']['bucket'],
            }
            scenario['configuration_family_id'] = configuration_family_id(
                family_record
            )
            scenario['configuration_core_family_id'] = configuration_family_id(
                family_record,
                include_render=False,
            )
            return scenario
        except (KeyError, ValueError) as exc:
            last_error = exc
    raise VisualV3Error(
        f'{spec["spec_id"]}: materialisation failed after '
        f'{MAX_PHYSICAL_PLACEMENT_ATTEMPTS} attempts: {last_error}'
    )


def materialize_specs(specs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    cameras = load_camera_projections(_default_camera_model_path())
    identity_contract = _identity_visual_contract()
    rows = []
    scenario_ids = set()
    capture_configurations = set()
    for spec in specs:
        row = materialize_spec(
            spec,
            cameras=cameras,
            identity_contract=identity_contract,
        )
        if row['scenario_id'] in scenario_ids:
            raise VisualV3Error(f'duplicate scenario ID: {row["scenario_id"]}')
        if row['capture_configuration_fingerprint'] in capture_configurations:
            raise VisualV3Error(
                'duplicate capture configuration would produce an exact image '
                f'pair: {row["scenario_id"]}'
            )
        scenario_ids.add(row['scenario_id'])
        capture_configurations.add(row['capture_configuration_fingerprint'])
        rows.append(row)
    return rows


def _avoidance_variant(
    source: dict[str, Any],
    attempt: int,
) -> dict[str, Any]:
    candidate = copy.deepcopy(source)
    if attempt == 0:
        candidate['family_avoidance_attempt'] = 0
        return candidate
    candidate['family_avoidance_attempt'] = attempt
    candidate['geometry_seed_index'] = (
        int(source.get('geometry_seed_index', source['generation_index']))
        + attempt * 10007
    )
    candidate['geometry_seed_key'] = (
        f'{source.get("geometry_seed_key", source["spec_id"])}:avoid:{attempt}'
    )
    candidate['approach_direction'] = (
        'increasing_s',
        'decreasing_s',
    )[(attempt // 2) % 2]
    bucket = RENDER_BUCKETS[
        (RENDER_BUCKETS.index(source['render_bucket']) + attempt) % len(RENDER_BUCKETS)
    ]
    candidate['render_bucket'] = bucket
    candidate['render_parameters']['bucket'] = bucket
    candidate['render_parameters']['deterministic_seed'] = stable_int(
        SEED,
        source['spec_id'],
        'avoid-render',
        attempt,
    ) % (2**31)
    if candidate.get('position_overrides'):
        for identity, position in candidate['position_overrides'].items():
            original_bin = min(
                range(len(POSITION_RATIOS)),
                key=lambda index: abs(
                    POSITION_RATIOS[index] - float(position['s_ratio'])
                ),
            )
            position['s_ratio'] = POSITION_RATIOS[
                (original_bin + attempt) % len(POSITION_RATIOS)
            ]
        target_position = candidate['position_overrides'][
            candidate['target_identity']
        ]
        candidate['target_s_ratio'] = target_position['s_ratio']
        candidate['target_position_bin'] = position_bin(
            candidate['target_s_ratio']
        )
    elif (
        candidate.get('canary_family')
        == 'r4_right_slot3_position_arrival'
    ):
        target = candidate['target_identity']
        cardinality = len(candidate['active_identities'])
        subsets = [
            subset
            for subset in itertools.combinations(IDENTITIES, cardinality)
            if target in subset
        ]
        selected = subsets[attempt % len(subsets)]
        original_loaded_count = len(candidate['loaded_identities'])
        target_loaded = (
            candidate['payload_assignment'][target] == 'loaded'
        )
        peers = [identity for identity in selected if identity != target]
        peers.sort(
            key=lambda identity: stable_int(
                SEED,
                source['spec_id'],
                attempt,
                'canary-presence-avoidance',
                identity,
            )
        )
        loaded = {target} if target_loaded else set()
        loaded.update(peers[:max(0, original_loaded_count - len(loaded))])
        candidate['active_identities'] = list(selected)
        candidate['payload_assignment'] = {
            identity: ('loaded' if identity in loaded else 'empty')
            for identity in selected
        }
        candidate['loaded_identities'] = [
            identity for identity in selected if identity in loaded
        ]
    else:
        block_index = BLOCKS.index(source['target_block'])
        position_index = POSITION_BINS.index(source['target_position_bin'])
        candidate['target_block'] = BLOCKS[
            (block_index + attempt) % len(BLOCKS)
        ]
        candidate['target_s_ratio'] = POSITION_RATIOS[
            (position_index + attempt // len(BLOCKS) + 1)
            % len(POSITION_RATIOS)
        ]
        candidate['target_position_bin'] = position_bin(
            candidate['target_s_ratio']
        )
    candidate['spec_id'] = (
        f'{source["spec_id"]}_avoid_{attempt:03d}_'
        f'{value_sha256(candidate)[:8]}'
    )
    return candidate


def materialize_specs_avoiding(
    specs: list[dict[str, Any]],
    *,
    forbidden_core_families: set[str],
    forbidden_capture_configurations: set[str],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Materialise grouped families unseen relative to earlier partitions."""
    cameras = load_camera_projections(_default_camera_model_path())
    identity_contract = _identity_visual_contract()
    accepted_rows: list[dict[str, Any]] = []
    accepted_specs: list[dict[str, Any]] = []
    local_capture = set()
    handled = set()
    for index, source in enumerate(specs):
        if index in handled:
            continue
        pair_id = source.get('matched_pair_id')
        group_indices = (
            [
                candidate_index
                for candidate_index, candidate in enumerate(specs)
                if candidate.get('matched_pair_id') == pair_id
            ]
            if pair_id
            else [index]
        )
        handled.update(group_indices)
        group_sources = [specs[group_index] for group_index in group_indices]
        for avoidance_attempt in range(1 + len(BLOCKS) * len(POSITION_BINS) * 4):
            group_rows = []
            group_specs = []
            group_error = False
            shared_geometry: dict[str, Any] | None = None
            for member_index, group_source in enumerate(group_sources):
                candidate = _avoidance_variant(group_source, avoidance_attempt)
                if member_index and pair_id and shared_geometry is not None:
                    for field in (
                        'target_block',
                        'target_s_ratio',
                        'target_position_bin',
                        'position_overrides',
                        'geometry_seed_index',
                        'geometry_seed_key',
                        'approach_direction',
                        'render_bucket',
                        'render_parameters',
                    ):
                        candidate[field] = copy.deepcopy(shared_geometry[field])
                elif pair_id:
                    shared_geometry = candidate
                try:
                    row = materialize_spec(
                        candidate,
                        cameras=cameras,
                        identity_contract=identity_contract,
                    )
                except VisualV3Error:
                    group_error = True
                    break
                if (
                    row['configuration_core_family_id'] in forbidden_core_families
                    or row['capture_configuration_fingerprint']
                    in forbidden_capture_configurations
                    or row['capture_configuration_fingerprint'] in local_capture
                    or any(
                        previous['capture_configuration_fingerprint']
                        == row['capture_configuration_fingerprint']
                        for previous in group_rows
                    )
                ):
                    group_error = True
                    break
                group_rows.append(row)
                group_specs.append(candidate)
            if group_error:
                continue
            accepted_rows.extend(group_rows)
            accepted_specs.extend(group_specs)
            local_capture.update(
                row['capture_configuration_fingerprint'] for row in group_rows
            )
            break
        else:
            raise VisualV3Error(
                f'could not isolate configuration family for {source["spec_id"]}'
            )
    if len(accepted_rows) != len(specs):
        raise VisualV3Error('family-avoidance materialisation changed row count')
    return accepted_specs, accepted_rows


def _repository_state() -> dict[str, Any]:
    commit = subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    status = subprocess.run(
        ['git', 'status', '--short'],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
    return {'commit': commit, 'dirty': bool(status), 'status': status}


def _write_manifest_lock(
    root: Path,
    lock: dict[str, Any],
    *,
    resume: bool,
) -> None:
    path = root / 'manifest_lock.json'
    if path.is_file():
        existing = json.loads(path.read_text(encoding='utf-8'))
        if existing == lock:
            return
        captured_episodes = [
            entry
            for entry in (root / 'dataset' / 'episodes').glob('*')
            if entry.is_dir() and entry.name != '.capture_tmp'
        ]
        if captured_episodes:
            raise VisualV3Error(
                f'manifest lock changed after capture began: {path}'
            )
        if not resume:
            raise VisualV3Error(f'manifest lock changed without --resume: {path}')
    atomic_json(path, lock)


def _configuration(mode: str) -> dict[str, Any]:
    return {
        'schema_version': PACKAGE_SCHEMA,
        'generator_version': GENERATOR_VERSION,
        'visual_schema': VISUAL_SCHEMA,
        'scenario_schema': SCENARIO_SCHEMA,
        'seed': SEED,
        'mode': mode,
        'fixed_identity_order': list(IDENTITIES),
        'public_blocks': list(BLOCKS),
        'capture': {
            'cameras': list(REQUIRED_CAMERAS),
            'frames': 1,
            'settle_seconds': 1.5,
            'frame_interval_seconds': 0.25,
        },
    }


def _smoke_specs(count: int) -> list[dict[str, Any]]:
    if count < 32:
        raise VisualV3Error('smoke capture requires at least 32 scenarios')
    canary = build_specs('canary')
    by_family: dict[str, list[dict[str, Any]]] = {}
    for spec in canary:
        by_family.setdefault(str(spec['canary_family']), []).append(spec)
    selected = []
    families = tuple(by_family)
    family_indices = Counter()
    while len(selected) < count:
        family = families[len(selected) % len(families)]
        rows = by_family[family]
        selected.append(copy.deepcopy(rows[family_indices[family] % len(rows)]))
        family_indices[family] += 1
    anchor = next(
        (
            copy.deepcopy(spec)
            for spec in canary
            if 'exact_l2_l4_r4_anchor' in spec.get('hard_case_tags', [])
        ),
        None,
    )
    if anchor is not None:
        selected[0] = anchor
    used_source_ids = set()
    for index, spec in enumerate(selected):
        source_id = spec['spec_id']
        if source_id in used_source_ids:
            family = str(spec['canary_family'])
            candidates = by_family[family]
            replacement = next(
                (
                    copy.deepcopy(candidate)
                    for candidate in candidates
                    if candidate['spec_id'] not in used_source_ids
                ),
                None,
            )
            if replacement is None:
                raise VisualV3Error(
                    f'smoke selection exhausted unique {family} candidates'
                )
            selected[index] = replacement
            source_id = replacement['spec_id']
        used_source_ids.add(source_id)
    for index, spec in enumerate(selected):
        spec['profile'] = 'smoke'
        spec['generation_index'] = index
        spec['spec_id'] = f'smoke_v3_{index + 1:04d}_{value_sha256(spec)[:12]}'
    return selected


def prepare_manifests(
    *,
    mode: str,
    capture_root: Path,
    canary_root: Path,
    guard_root: Path,
    smoke_count: int,
    resume: bool,
) -> dict[str, Any]:
    if mode not in {'smoke', 'full'}:
        raise VisualV3Error(f'unsupported mode: {mode}')
    configuration = _configuration(mode)
    if mode == 'smoke':
        smoke_root = guard_root / 'smoke'
        config_sha = prepare_output_root(smoke_root, configuration, resume=resume)
        specs = _smoke_specs(smoke_count)
        scenarios = materialize_specs(specs)
        spec_path = smoke_root / 'smoke_specs.jsonl'
        manifest_path = smoke_root / 'scenario_manifest.jsonl'
        atomic_jsonl(spec_path, specs)
        atomic_jsonl(manifest_path, scenarios)
        result = {
            'mode': mode,
            'root': str(smoke_root),
            'configuration_sha256': config_sha,
            'scenario_count': len(scenarios),
            'expected_image_count': len(scenarios) * 2,
            'spec_path': str(spec_path),
            'scenario_manifest': str(manifest_path),
            'scenario_manifest_sha256': sha256_file(manifest_path),
        }
        _write_manifest_lock(
            smoke_root,
            {
                'schema_version': PACKAGE_SCHEMA,
                'seed': SEED,
                'profile': 'smoke',
                'scenario_count': len(scenarios),
                'scenario_manifest': str(manifest_path),
                'scenario_manifest_sha256': sha256_file(manifest_path),
            },
            resume=resume,
        )
        atomic_json(smoke_root / 'static_manifest_audit.json', static_manifest_audit(scenarios))
        atomic_json(smoke_root / 'package_manifest.json', result)
        return result

    config_sha = prepare_output_root(capture_root, configuration, resume=resume)
    canary_config = {**configuration, 'mode': 'canary'}
    canary_sha = prepare_output_root(canary_root, canary_config, resume=resume)
    guard_config = {**configuration, 'mode': 'guard'}
    guard_marker = guard_root / 'generation_configuration.json'
    if (
        guard_root.is_dir()
        and not guard_marker.exists()
        and {entry.name for entry in guard_root.iterdir()} <= {'smoke'}
    ):
        marker_record = dict(guard_config)
        marker_record['configuration_sha256'] = value_sha256(guard_config)
        atomic_json(guard_marker, marker_record, overwrite=False)
    else:
        prepare_output_root(guard_root, guard_config, resume=resume)
    plan = quota_plan()
    atomic_json(guard_root / 'room315_visual_v3_quota_plan.json', plan)
    result: dict[str, Any] = {
        'mode': mode,
        'capture_root': str(capture_root),
        'canary_root': str(canary_root),
        'guard_root': str(guard_root),
        'configuration_sha256': config_sha,
        'canary_configuration_sha256': canary_sha,
        'quota_plan_sha256': plan['quota_plan_sha256'],
        'repository': _repository_state(),
        'manifests': {},
    }
    train_specs = build_specs('train')
    train_scenarios = materialize_specs(train_specs)
    from room_315_visual_v3_splitter import _old_replay_core_families

    old_replay_core_families = _old_replay_core_families(
        DEFAULT_OLD_TRAIN,
        DEFAULT_OLD_TRAIN_LABELS,
    )
    validation_specs, validation_scenarios = materialize_specs_avoiding(
        build_specs('validation'),
        forbidden_core_families=(
            {
                row['configuration_core_family_id'] for row in train_scenarios
            }
            | old_replay_core_families
        ),
        forbidden_capture_configurations={
            row['capture_configuration_fingerprint'] for row in train_scenarios
        },
    )
    canary_specs, canary_scenarios = materialize_specs_avoiding(
        build_specs('canary'),
        forbidden_core_families={
            row['configuration_core_family_id']
            for row in train_scenarios + validation_scenarios
        },
        forbidden_capture_configurations={
            row['capture_configuration_fingerprint']
            for row in train_scenarios + validation_scenarios
        },
    )
    planned = {
        'train': (train_specs, train_scenarios, capture_root, TRAIN_COUNT),
        'validation': (
            validation_specs,
            validation_scenarios,
            capture_root,
            VALIDATION_COUNT,
        ),
        'canary': (canary_specs, canary_scenarios, canary_root, CANARY_COUNT),
    }
    for profile, (specs, scenarios, root, expected) in planned.items():
        if len(scenarios) != expected:
            raise VisualV3Error(f'{profile} count mismatch')
        manifest_dir = root / 'manifests'
        spec_path = manifest_dir / f'{profile}_specs.jsonl'
        manifest_path = manifest_dir / f'{profile}_scenarios.jsonl'
        atomic_jsonl(spec_path, specs)
        atomic_jsonl(manifest_path, scenarios)
        audit = static_manifest_audit(scenarios)
        if not audit['passed']:
            raise VisualV3Error(f'{profile} static manifest audit failed')
        atomic_json(manifest_dir / f'{profile}_static_audit.json', audit)
        result['manifests'][profile] = {
            'scenario_count': len(scenarios),
            'expected_image_count': len(scenarios) * 2,
            'spec_path': str(spec_path),
            'spec_sha256': sha256_file(spec_path),
            'scenario_manifest': str(manifest_path),
            'scenario_manifest_sha256': sha256_file(manifest_path),
            'static_audit': str(manifest_dir / f'{profile}_static_audit.json'),
        }
    _write_manifest_lock(
        capture_root,
        {
            'schema_version': PACKAGE_SCHEMA,
            'seed': SEED,
            'profiles': {
                profile: result['manifests'][profile]
                for profile in ('train', 'validation')
            },
        },
        resume=resume,
    )
    _write_manifest_lock(
        canary_root,
        {
            'schema_version': PACKAGE_SCHEMA,
            'seed': SEED,
            'profiles': {'canary': result['manifests']['canary']},
        },
        resume=resume,
    )
    atomic_json(guard_root / 'static_package_manifest.json', result)
    return result


def static_manifest_audit(rows: list[dict[str, Any]]) -> dict[str, Any]:
    errors = []
    ids = [row.get('scenario_id') for row in rows]
    if len(ids) != len(set(ids)):
        errors.append('duplicate scenario IDs')
    capture_fingerprints = [
        row.get('capture_configuration_fingerprint') for row in rows
    ]
    if len(capture_fingerprints) != len(set(capture_fingerprints)):
        errors.append('duplicate capture configurations')
    for row in rows:
        try:
            validate_scenario(row)
        except ValueError as exc:
            errors.append(f'{row.get("scenario_id")}: {exc}')
        if row.get('visual_label_schema') != VISUAL_SCHEMA:
            errors.append(f'{row.get("scenario_id")}: wrong visual schema')
        if row.get('active_identities') != [
            identity for identity in IDENTITIES if identity in row.get('active_identities', [])
        ]:
            errors.append(f'{row.get("scenario_id")}: identity order mismatch')
        if set(row.get('identity_to_block', {}).values()) - set(BLOCKS):
            errors.append(f'{row.get("scenario_id")}: invalid public block')
        if scenario_physical_conflicts(row):
            errors.append(f'{row.get("scenario_id")}: physical conflict')
    relation_counts = Counter(row['relation_family'] for row in rows)
    return {
        'schema_version': PACKAGE_SCHEMA,
        'scenario_count': len(rows),
        'expected_image_count': len(rows) * 2,
        'scenario_id_unique': len(ids) == len(set(ids)),
        'capture_configuration_unique': (
            len(capture_fingerprints) == len(set(capture_fingerprints))
        ),
        'presence_cardinality': dict(sorted(Counter(
            len(row['active_identities']) for row in rows
        ).items())),
        'relation_family': dict(sorted(relation_counts.items())),
        'identity_occurrences': dict(sorted(Counter(
            identity
            for row in rows
            for identity in row['active_identities']
        ).items())),
        'loaded_identity_occurrences': dict(sorted(Counter(
            identity
            for row in rows
            for identity in row['loaded_identities']
        ).items())),
        'block_occurrences': dict(sorted(Counter(
            block
            for row in rows
            for block in row['identity_to_block'].values()
        ).items())),
        'configuration_family_count': len({
            row['configuration_family_id'] for row in rows
        }),
        'physical_conflict_count': sum(
            bool(scenario_physical_conflicts(row)) for row in rows
        ),
        'errors': errors,
        'passed': not errors,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('smoke', 'full'), required=True)
    parser.add_argument('--capture-root', type=Path, default=DEFAULT_CAPTURE_ROOT)
    parser.add_argument('--canary-root', type=Path, default=DEFAULT_CANARY_ROOT)
    parser.add_argument('--guard-root', type=Path, default=DEFAULT_GUARD_ROOT)
    parser.add_argument('--smoke-count', type=int, default=32)
    parser.add_argument('--resume', action='store_true')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    result = prepare_manifests(
        mode=args.mode,
        capture_root=args.capture_root,
        canary_root=args.canary_root,
        guard_root=args.guard_root,
        smoke_count=args.smoke_count,
        resume=args.resume,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, VisualV3Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
