#!/usr/bin/env python3
"""Build and audit the physically feasible Room 315 arbitrary-subset v2 design.

This module deliberately consumes the immutable v1 design as a traceability
source.  It jointly assigns target zones and physical geometry, materializes
every row as a complete static scenario, and runs the same topology and
world-space collision validators used by the approved smoke workflow.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import itertools
import json
import math
import os
import random
import shutil
import tempfile
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict, deque
from pathlib import Path
from typing import Any

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]

from room_315_arbitrary_subset_visual import (  # noqa: E402
    GLOBAL_IDENTITIES,
    NO_RELATION,
    PLAN_SCHEMA,
    SIDE_IDENTITIES,
    ArbitrarySubsetError,
    presence_inventory,
    read_jsonl,
    sha256,
    stable_int,
    write_json,
    write_jsonl,
)
from room_315_rail_defaults import (  # noqa: E402
    LEFT_PUBLIC_SEGMENT_NAME_MAP,
    default_rail_network_path,
)
from room_315_shuttle_geometry import (  # noqa: E402
    ShuttlePosition,
    shuttle_position_conflicts,
    shuttle_world_pose,
)
from room_315_visual_label_exporter import (  # noqa: E402
    _default_camera_model_path,
    load_camera_projections,
    shuttle_bbox,
)
from room_315_visual_scenario_generator import (  # noqa: E402
    ADJACENT_PREFIXES_BY_ZONE,
    ARBITRARY_IDENTITY_PRESENCE_PROFILE,
    BLOCKER_SCENE_TYPES,
    LEFT_BRANCH_CYCLES,
    MAX_PHYSICAL_PLACEMENT_ATTEMPTS,
    MIN_SAME_SEGMENT_START_SEPARATION_M,
    MIN_SAME_SEGMENT_START_SEPARATION_RATIO,
    POSITION_ZONES,
    REQUIRED_CAMERAS,
    RIGHT_BRANCH_CYCLES,
    SCHEMA_VERSION,
    SIDES,
    SLOT_RATIOS,
    SWITCH_NAMES,
    _blocker_positions,
    _branch_for_segment,
    _canonical_json,
    _family_payload,
    _hash,
    _launch_arguments,
    _place_neutral_shuttles,
    _sample_ratio,
    _segment_lengths,
    _switch_command,
    _zone_segments,
    scenario_physical_conflicts,
    valid_public_segments,
    validate_scenario,
)
from room_315_visual_state_dataset import (  # noqa: E402
    VISUAL_STATE_SCHEMA_VERSION,
    VisualStateLabelVectorizer,
)


V2_SEED = 31520260729
V2_PLAN_SCHEMA = 'room315.arbitrary_subset_production_plan.v2'
V2_AUDIT_SCHEMA = 'room315.arbitrary_subset_production_plan_audit.v2'
V2_PACKAGE_SCHEMA = 'room315.arbitrary_subset_production_plan_package.v2'
V1_ROOT = Path(
    '/home/tiago/room315_arbitrary_subset_visual_2040_seed31520260728'
)
V1_PLAN = V1_ROOT / 'configuration_variant_plan.jsonl'
DEFAULT_OUTPUT = Path(
    '/home/tiago/room315_arbitrary_subset_visual_2040_v2_seed31520260729'
)

EXPECTED_RELATION_TOTALS = {
    'blocker_ahead_same_segment': 414,
    'blocker_intermediate_segment': 422,
    'multi_blocker': 85,
    'no_relation_observation': 388,
    'nonblocker_adjacent_branch': 293,
    'nonblocker_behind_same_segment': 438,
}
EXPECTED_ZONE_TOTALS = {
    'boundary': 339,
    'buffer': 340,
    'merge_conflict': 341,
    'ordinary': 339,
    'slot': 341,
    'switch': 340,
}
EXPECTED_TOTAL_ACTIVE = {
    '1': 64,
    '2': 224,
    '3': 448,
    '4': 560,
    '5': 448,
    '6': 224,
    '7': 64,
    '8': 8,
}
PROTECTED_ROOTS = {
    'approved_smoke': {
        'path': Path(
            '/home/tiago/room315_arbitrary_subset_visual_smoke_seed31520260728'
        ),
        'file_count': 699,
        'tree_sha256': (
            '8769131286197ec863b3ab6911d162bb57b2413d388791ce958f55ed9582e224'
        ),
    },
    'immutable_v1': {
        'path': V1_ROOT,
        'file_count': 6,
        'tree_sha256': (
            '0a7c14d2186d065d355837740928d0260f7192e5c4f28dfe904d69f9963de987'
        ),
    },
    'frozen_dense': {
        'path': Path(
            '/home/tiago/room315_eight_shuttle_visual_320_seed31520260727'
        ),
        'file_count': 1709,
        'tree_sha256': (
            'ee89d87260e08e83b2b7e7d93135544f624fd911b8056f9ee4be7111627bdedf'
        ),
    },
    'frozen_pilot': {
        'path': Path(
            '/home/tiago/Downloads/kairos_room315_h200_pilot_results'
        ),
        'file_count': 175,
        'tree_sha256': (
            '6952e51d0bc71c66cffe715b35c0763a133f815650e423a75391c49ec1745b3d'
        ),
    },
}
SAME_SEGMENT_FAMILIES = {
    'blocker_ahead_same_segment',
    'nonblocker_behind_same_segment',
    'multi_blocker',
}
ADJACENT_FEASIBLE_ZONES = {
    'boundary',
    'buffer',
    'ordinary',
    'slot',
}
UNCHANGED_V1_FIELDS = (
    'plan_id',
    'configuration_id',
    'variant_index',
    'planned_partition_role',
    'left_active_identities',
    'right_active_identities',
    'target_identity',
    'relation_family',
    'relation_identities',
    'relation_neutral_identities',
    'opposite_rail_distractor_identities',
    'position_seed',
    'payload_assignment_mask',
    'payload_assignment',
    'switch_pattern_variant',
    'geometry_variant_must_be_unique_within_configuration',
    'relation_metadata_model_input',
)


def tree_fingerprint(root: Path) -> tuple[int, str]:
    """Return the repository's established byte-for-byte tree fingerprint."""
    lines = []
    for path in sorted(item for item in root.rglob('*') if item.is_file()):
        lines.append(
            hashlib.sha256(path.read_bytes()).hexdigest()
            + '  '
            + str(path.relative_to(root))
            + '\n'
        )
    return len(lines), hashlib.sha256(
        ''.join(lines).encode('utf-8')
    ).hexdigest()


def protected_artifact_audit() -> dict[str, Any]:
    results = {}
    for name, contract in PROTECTED_ROOTS.items():
        root = contract['path']
        if not root.is_dir():
            results[name] = {
                'path': str(root),
                'exists': False,
                'passed': False,
            }
            continue
        count, digest = tree_fingerprint(root)
        results[name] = {
            'path': str(root),
            'exists': True,
            'file_count': count,
            'expected_file_count': contract['file_count'],
            'tree_sha256': digest,
            'expected_tree_sha256': contract['tree_sha256'],
            'passed': (
                count == contract['file_count']
                and digest == contract['tree_sha256']
            ),
        }
    return {
        'passed': all(result['passed'] for result in results.values()),
        'artifacts': results,
    }


def _side(identity: str) -> str:
    return 'left' if identity.startswith('L') else 'right'


def _public_name(side: str, canonical: str) -> str:
    canonical = str(canonical).upper()
    if side == 'left':
        return LEFT_PUBLIC_SEGMENT_NAME_MAP.get(canonical, canonical)
    return canonical


def _network_sources() -> dict[str, Path]:
    return {
        side: default_rail_network_path(side).resolve()
        for side in ('left', 'right')
    }


def _device_source(side: str) -> Path:
    return (
        REPO_ROOT
        / 'mfja_robot_control_config'
        / 'config'
        / 'room_315_kinematics'
        / f'rail_devices_{side}.yaml'
    )


def _zone_intervals(
    side: str,
    segment: str,
    zone: str,
) -> list[list[float]]:
    if zone == 'boundary':
        return [[0.04, 0.10], [0.90, 0.96]]
    if zone == 'switch':
        return [[0.07, 0.15], [0.85, 0.93]]
    if zone == 'merge_conflict':
        return [[0.38, 0.62]]
    if zone == 'buffer':
        return [[0.28, 0.72]]
    if zone == 'ordinary':
        return [[0.20, 0.80]]
    if zone == 'slot':
        return [
            [
                round(max(0.04, anchor - 0.025), 9),
                round(min(0.96, anchor + 0.025), 9),
            ]
            for anchor in SLOT_RATIOS[side].get(segment, ())
        ]
    raise ArbitrarySubsetError(f'unsupported target zone: {zone!r}')


def ratio_matches_zone(
    side: str,
    segment: str,
    zone: str,
    ratio: float,
) -> bool:
    if segment not in _zone_segments(side, zone):
        return False
    return any(
        low - 1e-6 <= ratio <= high + 1e-6
        for low, high in _zone_intervals(side, segment, zone)
    )


def relation_zone_feasible(family: str, side: str, zone: str) -> bool:
    """Return hard family/zone feasibility under the physical model."""
    if family == NO_RELATION:
        return bool(_zone_segments(side, zone))
    if family in SAME_SEGMENT_FAMILIES:
        return (
            zone != 'switch'
            and any(
                _segment_lengths(side)[segment] >= 1.0
                for segment in _zone_segments(side, zone)
            )
        )
    if family == 'nonblocker_adjacent_branch':
        return zone in ADJACENT_FEASIBLE_ZONES
    if family == 'blocker_intermediate_segment':
        return bool(_zone_segments(side, zone))
    raise ArbitrarySubsetError(f'unsupported relation family: {family!r}')


class _FlowEdge:
    def __init__(self, to: int, reverse: int, capacity: int, cost: int):
        self.to = to
        self.reverse = reverse
        self.capacity = capacity
        self.cost = cost
        self.original_capacity = capacity


def _add_flow_edge(
    graph: list[list[_FlowEdge]],
    source: int,
    target: int,
    capacity: int,
    cost: int,
) -> _FlowEdge:
    forward = _FlowEdge(target, len(graph[target]), capacity, cost)
    reverse = _FlowEdge(source, len(graph[source]), 0, -cost)
    graph[source].append(forward)
    graph[target].append(reverse)
    return forward


def assign_target_zones(
    v1_rows: list[dict[str, Any]],
) -> tuple[list[str], dict[str, Any]]:
    """Solve exact zone capacities while maximizing v1 row-level retention."""
    groups: dict[tuple[str, str, str], list[int]] = defaultdict(list)
    for index, row in enumerate(v1_rows):
        groups[(
            str(row['relation_family']),
            _side(str(row['target_identity'])),
            str(row['target_zone']),
        )].append(index)
    group_keys = sorted(groups)
    zones = sorted(EXPECTED_ZONE_TOTALS)
    source = 0
    first_group = 1
    first_zone = first_group + len(group_keys)
    sink = first_zone + len(zones)
    graph: list[list[_FlowEdge]] = [[] for _ in range(sink + 1)]
    assignment_edges: dict[tuple[int, str], _FlowEdge] = {}
    for group_index, key in enumerate(group_keys):
        node = first_group + group_index
        _add_flow_edge(graph, source, node, len(groups[key]), 0)
        family, side, original_zone = key
        for zone_index, zone in enumerate(zones):
            if not relation_zone_feasible(family, side, zone):
                continue
            changed = zone != original_zone
            tie_break = stable_int(V2_SEED, key, zone) % 1000
            cost = 0 if not changed else 10_000_000 + tie_break
            edge = _add_flow_edge(
                graph,
                node,
                first_zone + zone_index,
                len(groups[key]),
                cost,
            )
            assignment_edges[(group_index, zone)] = edge
    for zone_index, zone in enumerate(zones):
        _add_flow_edge(
            graph,
            first_zone + zone_index,
            sink,
            EXPECTED_ZONE_TOTALS[zone],
            0,
        )

    required_flow = len(v1_rows)
    flow = 0
    total_cost = 0
    while flow < required_flow:
        distance = [math.inf] * len(graph)
        parent: list[tuple[int, int] | None] = [None] * len(graph)
        in_queue = [False] * len(graph)
        distance[source] = 0
        queue = deque([source])
        in_queue[source] = True
        while queue:
            node = queue.popleft()
            in_queue[node] = False
            for edge_index, edge in enumerate(graph[node]):
                if edge.capacity <= 0:
                    continue
                candidate = distance[node] + edge.cost
                if candidate >= distance[edge.to]:
                    continue
                distance[edge.to] = candidate
                parent[edge.to] = (node, edge_index)
                if not in_queue[edge.to]:
                    queue.append(edge.to)
                    in_queue[edge.to] = True
        if parent[sink] is None:
            raise ArbitrarySubsetError(
                'exact target-zone totals are mathematically infeasible'
            )
        amount = required_flow - flow
        node = sink
        while node != source:
            previous, edge_index = parent[node]  # type: ignore[misc]
            amount = min(amount, graph[previous][edge_index].capacity)
            node = previous
        node = sink
        while node != source:
            previous, edge_index = parent[node]  # type: ignore[misc]
            edge = graph[previous][edge_index]
            edge.capacity -= amount
            graph[node][edge.reverse].capacity += amount
            total_cost += amount * edge.cost
            node = previous
        flow += amount

    assigned = [''] * len(v1_rows)
    transport = {}
    for group_index, key in enumerate(group_keys):
        row_indices = sorted(
            groups[key],
            key=lambda index: str(v1_rows[index]['plan_id']),
        )
        allocations = []
        for zone in zones:
            edge = assignment_edges.get((group_index, zone))
            count = 0 if edge is None else edge.original_capacity - edge.capacity
            if count:
                allocations.extend([zone] * count)
                transport[
                    f'{key[0]}|{key[1]}|{key[2]}->{zone}'
                ] = count
        original = key[2]
        allocations.sort(key=lambda zone: (zone != original, zone))
        if len(allocations) != len(row_indices):
            raise ArbitrarySubsetError('zone flow did not consume a complete group')
        for row_index, zone in zip(row_indices, allocations):
            assigned[row_index] = zone

    counts = Counter(assigned)
    if dict(sorted(counts.items())) != EXPECTED_ZONE_TOTALS:
        raise ArbitrarySubsetError(
            f'zone solver produced wrong totals: {dict(counts)}'
        )
    invalid = [
        v1_rows[index]['plan_id']
        for index, zone in enumerate(assigned)
        if not relation_zone_feasible(
            str(v1_rows[index]['relation_family']),
            _side(str(v1_rows[index]['target_identity'])),
            zone,
        )
    ]
    if invalid:
        raise ArbitrarySubsetError(
            f'zone solver retained infeasible assignments: {invalid[:5]}'
        )
    retained = sum(
        zone == row['target_zone']
        for row, zone in zip(v1_rows, assigned)
    )
    return assigned, {
        'method': (
            'deterministic integer min-cost flow over '
            '(relation_family, side, original_zone) groups'
        ),
        'seed': V2_SEED,
        'hard_constraints': [
            'family-zone physical feasibility',
            'exact six-zone capacities',
        ],
        'objective_order': [
            'minimize changed v1 target zones',
            'stable SHA-256 tie break',
        ],
        'flow': flow,
        'cost': total_cost,
        'retained_original_zone_rows': retained,
        'changed_zone_rows': len(v1_rows) - retained,
        'transport': dict(sorted(transport.items())),
        'zone_totals': dict(sorted(counts.items())),
    }


def _adjacent_positions(
    *,
    side: str,
    target_zone: str,
    position_seed: int,
    type_index: int,
) -> tuple[list[dict[str, Any]], list[dict[str, str]], str]:
    if target_zone not in ADJACENT_FEASIBLE_ZONES:
        raise ArbitrarySubsetError(
            f'adjacent relation is infeasible in target zone {target_zone}'
        )
    prefixes = ('A12', 'A34')
    for attempt in range(MAX_PHYSICAL_PLACEMENT_ATTEMPTS):
        rng = random.Random(stable_int(
            V2_SEED,
            position_seed,
            side,
            target_zone,
            type_index,
            attempt,
            'adjacent',
        ))
        prefix = prefixes[(type_index + attempt) % len(prefixes)]
        if target_zone == 'slot':
            target_suffix = 'E'
        elif target_zone == 'buffer':
            target_suffix = 'I'
        else:
            target_suffix = ('E', 'I')[(type_index + attempt) % 2]
        target_segment = prefix + target_suffix
        other_segment = prefix + ('I' if target_suffix == 'E' else 'E')
        target_ratio = _sample_ratio(
            target_zone,
            side=side,
            segment=target_segment,
            rng=rng,
            boundary_preference=(
                ('start', 'end')[(type_index + attempt) % 2]
                if target_zone == 'boundary'
                else None
            ),
        )
        other_grid = [
            round(0.04 + 0.03 * index, 6)
            for index in range(31)
        ] + [0.96]
        rotation = stable_int(
            position_seed, type_index, attempt, 'other-ratio'
        ) % len(other_grid)
        other_grid = other_grid[rotation:] + other_grid[:rotation]
        for other_ratio in other_grid:
            positions = [
                {
                    'segment': target_segment,
                    's_ratio': target_ratio,
                    'position_zone': target_zone,
                },
                {
                    'segment': other_segment,
                    's_ratio': other_ratio,
                    'position_zone': 'adjacent_branch',
                },
            ]
            conflicts = shuttle_position_conflicts([
                ShuttlePosition('target', side, target_segment, target_ratio),
                ShuttlePosition('other', side, other_segment, other_ratio),
            ])
            if conflicts:
                continue
            return (
                positions,
                [{
                    'other_shuttle_id': '',
                    'relation': 'adjacent_branch_non_blocker',
                }],
                'exterior' if target_suffix == 'E' else 'interior',
            )
    raise ArbitrarySubsetError(
        f'could not materialize adjacent relation on {side}:{target_zone}'
    )


def _single_target_position(
    source: dict[str, Any],
    *,
    side: str,
    target_zone: str,
    type_index: int,
    attempt: int,
) -> tuple[dict[str, Any], str]:
    candidates = list(_zone_segments(side, target_zone))
    original = str(source['target_segment'])
    candidates.sort(key=lambda segment: (
        segment != original,
        stable_int(V2_SEED, source['plan_id'], segment) % 10000,
        segment,
    ))
    segment = candidates[(type_index + attempt) % len(candidates)]
    rng = random.Random(stable_int(
        V2_SEED,
        source['position_seed'],
        target_zone,
        segment,
        attempt,
        'single-target',
    ))
    ratio = _sample_ratio(
        target_zone,
        side=side,
        segment=segment,
        rng=rng,
        boundary_preference=(
            ('start', 'end')[(type_index + attempt) % 2]
            if target_zone == 'boundary'
            else None
        ),
    )
    branch = _branch_for_segment(segment, rng)
    return {
        'segment': segment,
        's_ratio': ratio,
        'position_zone': target_zone,
    }, branch


def _augment_position(side: str, position: dict[str, Any]) -> dict[str, Any]:
    result = dict(position)
    segment = str(result['segment'])
    ratio = round(float(result['s_ratio']), 6)
    length = float(_segment_lengths(side)[segment])
    result.update({
        'segment': segment,
        's_ratio': ratio,
        'segment_length_m': round(length, 9),
        's_m': round(ratio * length, 9),
    })
    return result


def _identity_visual_contract() -> dict[str, dict[str, Any]]:
    config_path = (
        REPO_ROOT
        / 'mfja_robot_control_config'
        / 'config'
        / 'room_315_shuttle_identity'
        / 'shuttle_identity.yaml'
    )
    loaded = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    configured = {
        str(entry['label_text']): entry
        for entry in loaded['shuttles']
    }
    result = {}
    for identity in GLOBAL_IDENTITIES:
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
            name = str(visual.get('name') or '')
            if not name.startswith('identity_region_'):
                continue
            pose = [
                float(value)
                for value in visual.findtext('pose', default='').split()
            ]
            size = [
                float(value)
                for value in visual.findtext(
                    './geometry/plane/size',
                    default='',
                ).split()
            ]
            if len(pose) < 6 or len(size) != 2:
                raise ArbitrarySubsetError(
                    f'{identity}:{name} has invalid identity-region geometry'
                )
            regions.append({
                'name': name,
                'pose': pose,
                'size': size,
            })
        entry = configured[identity]
        result[identity] = {
            'color_name': str(entry['color_name']),
            'regions': regions,
            'sdf_path': str(sdf_path),
            'sdf_sha256': sha256(sdf_path),
        }
    colors = [entry['color_name'] for entry in result.values()]
    if len(colors) != len(set(colors)) or any(
        len(entry['regions']) != 4 for entry in result.values()
    ):
        raise ArbitrarySubsetError(
            'identity colors/regions do not provide a unique four-region contract'
        )
    return result


def _project_region(
    camera: Any,
    pose: Any,
    region: dict[str, Any],
) -> dict[str, Any] | None:
    local_x, local_y, local_z, _, _, local_yaw = region['pose']
    size_x, size_y = region['size']
    cos_pose = math.cos(pose.yaw)
    sin_pose = math.sin(pose.yaw)
    cos_local = math.cos(local_yaw)
    sin_local = math.sin(local_yaw)
    pixels = []
    for dx in (-size_x / 2.0, size_x / 2.0):
        for dy in (-size_y / 2.0, size_y / 2.0):
            marker_x = local_x + cos_local * dx - sin_local * dy
            marker_y = local_y + sin_local * dx + cos_local * dy
            world = (
                pose.x + cos_pose * marker_x - sin_pose * marker_y,
                pose.y + sin_pose * marker_x + cos_pose * marker_y,
                pose.z + local_z,
            )
            pixel = camera.project(world)
            if pixel is None:
                return None
            pixels.append(pixel)
    x_min = max(0.0, min(pixel[0] for pixel in pixels))
    y_min = max(0.0, min(pixel[1] for pixel in pixels))
    x_max = min(float(camera.width), max(pixel[0] for pixel in pixels))
    y_max = min(float(camera.height), max(pixel[1] for pixel in pixels))
    if x_max <= x_min or y_max <= y_min:
        return None
    return {
        'bbox_xywh': [
            round(x_min, 3),
            round(y_min, 3),
            round(x_max - x_min, 3),
            round(y_max - y_min, 3),
        ],
        'area_px2': round((x_max - x_min) * (y_max - y_min), 3),
    }


def scenario_projectability(
    scenario: dict[str, Any],
    *,
    cameras: dict[str, Any] | None = None,
    identity_contract: dict[str, dict[str, Any]] | None = None,
) -> dict[str, Any]:
    cameras = cameras or load_camera_projections(_default_camera_model_path())
    identity_contract = identity_contract or _identity_visual_contract()
    active = {}
    invalid_bbox = []
    nonprojectable = []
    invisible_colors = []
    for side in ('left', 'right'):
        camera = cameras[side]
        camera_name = f'{side}_rail_rgb'
        for shuttle in scenario['scene']['rails'][side]['shuttles']:
            identity = str(shuttle['id'])
            position = shuttle['start_position']
            pose = shuttle_world_pose(
                side,
                str(position['segment']),
                float(position['s_ratio']),
            )
            bbox = shuttle_bbox(
                camera,
                (pose.x, pose.y, pose.z, pose.yaw),
            )
            if (
                bbox is None
                or bbox[2] <= 0.0
                or bbox[3] <= 0.0
                or bbox[0] < 0.0
                or bbox[1] < 0.0
                or bbox[0] + bbox[2] > camera.width + 1e-6
                or bbox[1] + bbox[3] > camera.height + 1e-6
            ):
                invalid_bbox.append(identity)
                bbox = [0.0, 0.0, 0.0, 0.0]
            regions = [
                projected
                for region in identity_contract[identity]['regions']
                if (
                    projected := _project_region(camera, pose, region)
                ) is not None
                and projected['area_px2'] >= 4.0
            ]
            if bbox[2] <= 0.0 or bbox[3] <= 0.0:
                nonprojectable.append(identity)
            if not regions:
                invisible_colors.append(identity)
            active[identity] = {
                'camera': camera_name,
                'camera_resolution': [camera.width, camera.height],
                'shuttle_bbox_xywh': bbox,
                'identity_color': identity_contract[identity]['color_name'],
                'projected_identity_region_count': len(regions),
                'largest_identity_region_area_px2': (
                    max(
                        region['area_px2']
                        for region in regions
                    )
                    if regions
                    else 0.0
                ),
                'visible_unique_color_region': bool(regions),
            }
    partial_overlap_pairs = []
    for first, second in itertools.combinations(sorted(active), 2):
        first_view = active[first]
        second_view = active[second]
        if first_view['camera'] != second_view['camera']:
            continue
        ax, ay, aw, ah = first_view['shuttle_bbox_xywh']
        bx, by, bw, bh = second_view['shuttle_bbox_xywh']
        overlap = (
            max(0.0, min(ax + aw, bx + bw) - max(ax, bx))
            * max(0.0, min(ay + ah, by + bh) - max(ay, by))
        )
        if overlap > 0.0:
            partial_overlap_pairs.append({
                'identities': [first, second],
                'camera': first_view['camera'],
                'bbox_overlap_px2': round(overlap, 3),
            })
    return {
        'passed': (
            not invalid_bbox
            and not nonprojectable
            and not invisible_colors
        ),
        'active_identities': active,
        'invalid_bbox_identities': invalid_bbox,
        'nonprojectable_identities': nonprojectable,
        'invisible_identity_color_identities': invisible_colors,
        'partial_occlusion_risk_pairs': partial_overlap_pairs,
    }


def _positions_for_row(
    source: dict[str, Any],
    target_zone: str,
    *,
    row_index: int,
    family_index: int,
    attempt: int,
    covered_segments: dict[str, set[str]],
) -> tuple[
    dict[str, dict[str, dict[str, Any]]],
    list[dict[str, str]],
    str,
]:
    target = str(source['target_identity'])
    side = _side(target)
    other_side = 'right' if side == 'left' else 'left'
    family = str(source['relation_family'])
    relation_identities = tuple(source['relation_identities'])
    if family == NO_RELATION:
        target_position, active_branch = _single_target_position(
            source,
            side=side,
            target_zone=target_zone,
            type_index=family_index,
            attempt=attempt,
        )
        raw_positions = [target_position]
        relations: list[dict[str, str]] = []
    elif family == 'nonblocker_adjacent_branch':
        raw_positions, relations, active_branch = _adjacent_positions(
            side=side,
            target_zone=target_zone,
            position_seed=int(source['position_seed']),
            type_index=family_index + attempt * 2040,
        )
    else:
        raw_positions, relations, active_branch = _blocker_positions(
            family,
            side=side,
            type_index=family_index + attempt * 2040,
            target_zone=target_zone,
            seed=int(source['position_seed']) + attempt,
            dataset_seed=V2_SEED,
        )
    if len(raw_positions) != 1 + len(relation_identities):
        raise ArbitrarySubsetError(
            f'{source["plan_id"]} relation-position cardinality mismatch'
        )
    target_positions = {target: raw_positions[0]}
    remapped_relations = []
    for identity, position, relation in zip(
        relation_identities,
        raw_positions[1:],
        relations,
    ):
        target_positions[str(identity)] = position
        remapped = dict(relation)
        remapped['other_shuttle_id'] = str(identity)
        remapped_relations.append(remapped)

    active_target = tuple(source[f'{side}_active_identities'])
    neutral = tuple(
        identity
        for identity in active_target
        if identity not in target_positions
    )
    _place_neutral_shuttles(
        side,
        neutral,
        target_positions,
        ordinal=row_index + 1 + attempt * 2040,
        dataset_seed=V2_SEED,
        active_branch=active_branch,
        target_segment=str(raw_positions[0]['segment']),
        covered_segments=set(covered_segments[side]),
    )
    opposite_positions: dict[str, dict[str, Any]] = {}
    _place_neutral_shuttles(
        other_side,
        tuple(source[f'{other_side}_active_identities']),
        opposite_positions,
        ordinal=row_index + 1 + attempt * 2040,
        dataset_seed=V2_SEED,
        active_branch=None,
        target_segment=None,
        covered_segments=set(covered_segments[other_side]),
    )
    positions = {
        side: target_positions,
        other_side: opposite_positions,
    }
    return positions, remapped_relations, active_branch


def _geometry_payload(
    rails: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    return {
        side: {
            'switches': rails[side]['switches'],
            'shuttles': [
                {
                    'id': shuttle['id'],
                    'segment': shuttle['start_position']['segment'],
                    's_ratio': shuttle['start_position']['s_ratio'],
                }
                for shuttle in rails[side]['shuttles']
            ],
        }
        for side in ('left', 'right')
    }


def materialize_v2_row(
    source: dict[str, Any],
    target_zone: str,
    *,
    row_index: int,
    family_index: int,
    covered_segments: dict[str, set[str]],
    cameras: dict[str, Any],
    identity_contract: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    target = str(source['target_identity'])
    active_side = _side(target)
    inactive_side = 'right' if active_side == 'left' else 'left'
    family = str(source['relation_family'])
    last_error: Exception | None = None
    for attempt in range(MAX_PHYSICAL_PLACEMENT_ATTEMPTS):
        try:
            positions, relations, active_branch = _positions_for_row(
                source,
                target_zone,
                row_index=row_index,
                family_index=family_index,
                attempt=attempt,
                covered_segments=covered_segments,
            )
            rails = {}
            for side in ('left', 'right'):
                switches = dict(zip(
                    SWITCH_NAMES,
                    (
                        (active_branch,) * len(SWITCH_NAMES)
                        if side == active_side
                        else ('exterior',) * len(SWITCH_NAMES)
                    ),
                ))
                shuttles = []
                for identity in source[f'{side}_active_identities']:
                    shuttles.append({
                        'id': identity,
                        'start_slot': int(identity[1:]),
                        'start_position': _augment_position(
                            side,
                            positions[side][identity],
                        ),
                        'loaded_state': source['payload_assignment'][identity],
                    })
                rails[side] = {
                    'switch_pattern': (
                        f'all_{active_branch}'
                        if side == active_side
                        else 'all_exterior'
                    ),
                    'switches': switches,
                    'shuttles': shuttles,
                }
            scene_type = family if family != NO_RELATION else 'single'
            relation_neutral = list(source['relation_neutral_identities'])
            opposite_neutral = list(
                source['opposite_rail_distractor_identities']
            )
            scene = {'rails': rails, 'obstacles': []}
            scenario_hash = _hash({
                **_family_payload(scene_type, scene),
                'source_v1_plan_id': source['plan_id'],
                'relation_family': family,
                'target_zone': target_zone,
            })
            launch_arguments = _launch_arguments(rails)
            launch_arguments.update({
                'room315_identity_selection_mode': 'explicit',
                'room315_left_active_identities': ','.join(
                    source['left_active_identities']
                ),
                'room315_right_active_identities': ','.join(
                    source['right_active_identities']
                ),
            })
            base = copy.deepcopy(source)
            target_position = positions[active_side][target]
            base.update({
                'design_schema_version': V2_PLAN_SCHEMA,
                'source_v1_plan_id': source['plan_id'],
                'presence_configuration_id': source['configuration_id'],
                'presence_bitmask': int(
                    str(source['configuration_id']).split('_')[-1]
                ),
                'canonical_subset_key': (
                    f'L:{"+".join(source["left_active_identities"]) or "-"}|'
                    f'R:{"+".join(source["right_active_identities"]) or "-"}'
                ),
                'left_count': len(source['left_active_identities']),
                'right_count': len(source['right_active_identities']),
                'total_active_count': (
                    len(source['left_active_identities'])
                    + len(source['right_active_identities'])
                ),
                'target_zone': target_zone,
                'target_segment': target_position['segment'],
                'target_s_ratio': round(
                    float(target_position['s_ratio']),
                    6,
                ),
                'target_segment_length_m': round(
                    float(_segment_lengths(active_side)[
                        str(target_position['segment'])
                    ]),
                    9,
                ),
                'target_s_m': round(
                    float(target_position['s_ratio'])
                    * float(_segment_lengths(active_side)[
                        str(target_position['segment'])
                    ]),
                    9,
                ),
                'geometry_key': hashlib.sha256(
                    _canonical_json(
                        _geometry_payload(rails)
                    ).encode('utf-8')
                ).hexdigest(),
                'schema_version': SCHEMA_VERSION,
                'manifest_profile_schema': V2_PLAN_SCHEMA,
                'presence_profile': ARBITRARY_IDENTITY_PRESENCE_PROFILE,
                'scenario_id': (
                    f'arbitrary_v2_{row_index + 1:04d}_'
                    f'{family}_{scenario_hash[:8]}'
                ),
                'scenario_family': f'arbitrary_v2_family_{scenario_hash}',
                'scene_type': scene_type,
                'seed': V2_SEED,
                'scene': scene,
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
                    'side': active_side,
                    'relations': relations,
                    'relation_neutral_shuttle_ids': relation_neutral,
                    'opposite_rail_neutral_shuttle_ids': opposite_neutral,
                    'model_input_exposure': 'excluded',
                },
                'rail_scope': ARBITRARY_IDENTITY_PRESENCE_PROFILE,
                'expected_label_coverage': {
                    'shuttle_count': (
                        len(source['left_active_identities'])
                        + len(source['right_active_identities'])
                    ),
                    'fixed_schema_identity_count': len(GLOBAL_IDENTITIES),
                    'loaded_count': sum(
                        state == 'loaded'
                        for state in source['payload_assignment'].values()
                    ),
                    'empty_count': sum(
                        state == 'empty'
                        for state in source['payload_assignment'].values()
                    ),
                    'switch_count': len(SWITCH_NAMES) * len(SIDES),
                    'obstacle_count': 0,
                    'continuous_position_count': (
                        len(source['left_active_identities'])
                        + len(source['right_active_identities'])
                    ),
                },
            })
            validate_scenario(base)
            if scenario_physical_conflicts(base):
                raise ArbitrarySubsetError('world-space physical conflict')
            if not ratio_matches_zone(
                active_side,
                str(target_position['segment']),
                target_zone,
                float(target_position['s_ratio']),
            ):
                raise ArbitrarySubsetError(
                    'target position does not match the assigned target zone'
                )
            projectability = scenario_projectability(
                base,
                cameras=cameras,
                identity_contract=identity_contract,
            )
            if not projectability['passed']:
                raise ArbitrarySubsetError(
                    f'static projectability failed: {projectability}'
                )
            base['static_camera_projectability'] = projectability
            for side in ('left', 'right'):
                covered_segments[side].update(
                    shuttle['start_position']['segment']
                    for shuttle in base['scene']['rails'][side]['shuttles']
                )
            return base
        except (ValueError, KeyError) as exc:
            last_error = exc
    raise ArbitrarySubsetError(
        f'{source["plan_id"]} could not be materialized after '
        f'{MAX_PHYSICAL_PLACEMENT_ATTEMPTS} attempts: {last_error}'
    )


def materialize_v2_rows(
    v1_rows: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    assigned_zones, solver = assign_target_zones(v1_rows)
    cameras = load_camera_projections(_default_camera_model_path())
    identity_contract = _identity_visual_contract()
    family_indices = Counter()
    covered_segments = {side: set() for side in ('left', 'right')}
    rows = []
    for row_index, (source, target_zone) in enumerate(
        zip(v1_rows, assigned_zones)
    ):
        key = (source['relation_family'], _side(source['target_identity']))
        rows.append(materialize_v2_row(
            source,
            target_zone,
            row_index=row_index,
            family_index=family_indices[key],
            covered_segments=covered_segments,
            cameras=cameras,
            identity_contract=identity_contract,
        ))
        family_indices[key] += 1
    return rows, solver


def _topology_graph(
    side: str,
) -> tuple[dict[str, set[str]], dict[str, set[str]]]:
    cycles = RIGHT_BRANCH_CYCLES if side == 'right' else LEFT_BRANCH_CYCLES
    predecessors = {
        segment: set()
        for segment in valid_public_segments(side)
    }
    successors = {
        segment: set()
        for segment in valid_public_segments(side)
    }
    for cycle in cycles.values():
        for index, segment in enumerate(cycle):
            successor = cycle[(index + 1) % len(cycle)]
            successors[segment].add(successor)
            predecessors[successor].add(segment)
    return predecessors, successors


def _switch_associations(side: str) -> dict[str, set[str]]:
    source = _network_sources()[side]
    loaded = yaml.safe_load(source.read_text(encoding='utf-8'))
    associations = {
        segment: set()
        for segment in valid_public_segments(side)
    }

    def visit(value: Any, switch_name: str) -> None:
        if isinstance(value, dict):
            for key, child in value.items():
                if (
                    key in {
                        'segment',
                        'incoming_segment',
                        'connector_segment',
                    }
                    and isinstance(child, str)
                ):
                    public = _public_name(side, child)
                    if public in associations:
                        associations[public].add(switch_name)
                elif key in {'incoming_segments', 'outgoing_segments'}:
                    for segment in child or []:
                        public = _public_name(side, segment)
                        if public in associations:
                            associations[public].add(switch_name)
                else:
                    visit(child, switch_name)
        elif isinstance(value, list):
            for child in value:
                visit(child, switch_name)

    for name, switch in (loaded.get('switches') or {}).items():
        visit(switch, str(name))
    return associations


def topology_zone_compatibility() -> dict[str, Any]:
    sources = {}
    for side, path in _network_sources().items():
        sources[f'{side}_rail_network'] = {
            'path': str(path),
            'sha256': sha256(path),
        }
        device = _device_source(side)
        sources[f'{side}_rail_devices'] = {
            'path': str(device),
            'sha256': sha256(device),
        }
    generator_source = SCRIPT_DIR / 'room_315_visual_scenario_generator.py'
    geometry_source = SCRIPT_DIR / 'room_315_shuttle_geometry.py'
    camera_source = _default_camera_model_path()
    for name, path in {
        'validated_scenario_generator': generator_source,
        'world_space_geometry': geometry_source,
        'overhead_camera_model': camera_source,
    }.items():
        sources[name] = {'path': str(path), 'sha256': sha256(path)}

    rail_tables = {}
    for side in ('left', 'right'):
        predecessors, successors = _topology_graph(side)
        switch_associations = _switch_associations(side)
        device = yaml.safe_load(
            _device_source(side).read_text(encoding='utf-8')
        )
        slots_by_segment: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for slot in device.get('slots') or []:
            segment = _public_name(side, slot['segment'])
            slots_by_segment[segment].append({
                'name': slot['name'],
                's_ratio': float(slot['s_ratio']),
            })
        segments = {}
        for segment, length in _segment_lengths(side).items():
            valid_zones = [
                zone
                for zone in POSITION_ZONES
                if segment in _zone_segments(side, zone)
            ]
            minimum_ratio = max(
                MIN_SAME_SEGMENT_START_SEPARATION_RATIO,
                MIN_SAME_SEGMENT_START_SEPARATION_M / length,
            )
            maximum_count = min(
                4,
                int(math.floor(1.0 / minimum_ratio + 1e-9)) + 1,
            )
            segments[segment] = {
                'side': side,
                'public_segment': segment,
                'segment_length_m': round(length, 9),
                'predecessor_segments': sorted(predecessors[segment]),
                'successor_segments': sorted(successors[segment]),
                'branch_suffix': (
                    segment[-1]
                    if segment[-1:] in {'E', 'I'}
                    else None
                ),
                'valid_target_zones': valid_zones,
                'switch_associations': sorted(
                    switch_associations[segment]
                ),
                'merge_conflict_association': (
                    'room315_merge_or_conflict_geometry'
                    if segment in _zone_segments(side, 'merge_conflict')
                    else None
                ),
                'slot_associations': slots_by_segment[segment],
                'boundary_intervals': _zone_intervals(
                    side, segment, 'boundary'
                ),
                'buffer_intervals': (
                    _zone_intervals(side, segment, 'buffer')
                    if segment in _zone_segments(side, 'buffer')
                    else []
                ),
                'ordinary_intervals': _zone_intervals(
                    side, segment, 'ordinary'
                ),
                'valid_s_ratio_intervals_by_zone': {
                    zone: _zone_intervals(side, segment, zone)
                    for zone in valid_zones
                },
                'required_same_segment_center_separation_m': (
                    MIN_SAME_SEGMENT_START_SEPARATION_M
                ),
                'required_same_segment_separation_ratio': round(
                    minimum_ratio,
                    9,
                ),
                'maximum_physically_feasible_same_segment_shuttle_count': (
                    maximum_count
                ),
            }
        rail_tables[side] = segments
    family_zone = {
        family: {
            side: {
                zone: relation_zone_feasible(family, side, zone)
                for zone in POSITION_ZONES
            }
            for side in ('left', 'right')
        }
        for family in (*BLOCKER_SCENE_TYPES, NO_RELATION)
    }
    return {
        'schema_version': 'room315.topology_zone_compatibility.v2',
        'authoritative_sources': sources,
        'clearance_contract': {
            'minimum_same_segment_start_separation_m': (
                MIN_SAME_SEGMENT_START_SEPARATION_M
            ),
            'minimum_same_segment_start_separation_ratio': (
                MIN_SAME_SEGMENT_START_SEPARATION_RATIO
            ),
            'world_space_validator': (
                'room_315_shuttle_geometry.shuttle_position_conflicts'
            ),
        },
        'relation_family_zone_feasibility': family_zone,
        'rails': rail_tables,
    }


def _v1_failure_audit(v1_rows: list[dict[str, Any]]) -> dict[str, Any]:
    zone_incompatible = []
    same_impossible = []
    adjacent_without_suffix = []
    infeasible = set()
    for row in v1_rows:
        side = _side(row['target_identity'])
        plan_id = row['plan_id']
        if row['target_segment'] not in _zone_segments(
            side, row['target_zone']
        ):
            zone_incompatible.append(plan_id)
            infeasible.add(plan_id)
        if (
            row['relation_family'] in SAME_SEGMENT_FAMILIES
            and _segment_lengths(side)[row['target_segment']]
            < MIN_SAME_SEGMENT_START_SEPARATION_M
        ):
            same_impossible.append(plan_id)
            infeasible.add(plan_id)
        if (
            row['relation_family'] == 'nonblocker_adjacent_branch'
            and str(row['target_segment'])[-1:] not in {'E', 'I'}
        ):
            adjacent_without_suffix.append(plan_id)
            infeasible.add(plan_id)
    return {
        'method': 'read-only reproduction of the confirmed necessary v1 checks',
        'row_count': len(v1_rows),
        'infeasible_unique_rows': len(infeasible),
        'initially_feasible_rows': len(v1_rows) - len(infeasible),
        'target_segment_incompatible_with_assigned_zone': len(
            zone_incompatible
        ),
        'same_segment_relation_physically_impossible': len(
            same_impossible
        ),
        'adjacent_branch_target_without_suffix': len(
            adjacent_without_suffix
        ),
        'categories_overlap': True,
        'expected_confirmed_counts_match': (
            len(infeasible) == 1299
            and len(zone_incompatible) == 924
            and len(same_impossible) == 597
            and len(adjacent_without_suffix) == 42
        ),
    }


def validate_v2_row(
    row: dict[str, Any],
    source: dict[str, Any],
) -> list[str]:
    errors = []
    for field in UNCHANGED_V1_FIELDS:
        if row.get(field) != source.get(field):
            errors.append(f'preserved_field_changed:{field}')
    if row.get('source_v1_plan_id') != source.get('plan_id'):
        errors.append('source_v1_plan_id_mismatch')
    try:
        validate_scenario(row)
    except ValueError as exc:
        errors.append(f'full_scenario_validator:{exc}')
    conflicts = scenario_physical_conflicts(row)
    if conflicts:
        errors.append(f'physical_conflicts:{len(conflicts)}')
    target = str(row.get('target_identity') or '')
    side = _side(target) if target in GLOBAL_IDENTITIES else ''
    if not side:
        errors.append('invalid_target_identity')
        return errors
    actual_by_side = {
        rail_side: [
            shuttle['id']
            for shuttle in row['scene']['rails'][rail_side]['shuttles']
        ]
        for rail_side in ('left', 'right')
    }
    for rail_side in ('left', 'right'):
        if actual_by_side[rail_side] != source[
            f'{rail_side}_active_identities'
        ]:
            errors.append(f'exact_subset_mismatch:{rail_side}')
    if target not in actual_by_side[side]:
        errors.append('target_not_active')
    target_shuttle = next(
        (
            shuttle
            for shuttle in row['scene']['rails'][side]['shuttles']
            if shuttle['id'] == target
        ),
        None,
    )
    if target_shuttle is None:
        return errors
    position = target_shuttle['start_position']
    if (
        position['segment'] != row.get('target_segment')
        or float(position['s_ratio']) != float(row.get('target_s_ratio'))
    ):
        errors.append('target_position_summary_mismatch')
    if not ratio_matches_zone(
        side,
        str(position['segment']),
        str(row.get('target_zone')),
        float(position['s_ratio']),
    ):
        errors.append('invalid_zone_segment_or_ratio')
    length = float(position['segment_length_m'])
    s_m = float(position['s_m'])
    ratio = float(position['s_ratio'])
    if (
        not 0.0 <= ratio <= 1.0
        or not 0.0 <= s_m <= length + 1e-9
        or abs(s_m - ratio * length) > 1e-6
    ):
        errors.append('invalid_metric_position')
    active_target = set(actual_by_side[side])
    relations = row['relation_probe']['relations']
    relation_ids = {
        relation['other_shuttle_id']
        for relation in relations
    }
    if relation_ids != set(source['relation_identities']):
        errors.append('relation_participant_mismatch')
    if not relation_ids <= active_target - {target}:
        errors.append('inactive_relation_participant')
    if row['relation_family'] not in (
        presence_inventory()[
            int(str(source['configuration_id']).split('_')[-1]) - 1
        ]['relation_eligibility'][target]
    ):
        errors.append('ineligible_relation_family')
    projectability = row.get('static_camera_projectability') or {}
    if projectability.get('passed') is not True:
        errors.append('camera_projectability_failed')
    if set((projectability.get('active_identities') or {})) != set(
        actual_by_side['left'] + actual_by_side['right']
    ):
        errors.append('camera_projectability_identity_set_mismatch')
    return errors


def design_v2_audit(
    v1_rows: list[dict[str, Any]],
    rows: list[dict[str, Any]],
    solver: dict[str, Any],
    topology: dict[str, Any],
    protected_before: dict[str, Any],
    protected_after: dict[str, Any],
) -> dict[str, Any]:
    source_by_id = {
        row['plan_id']: row
        for row in v1_rows
    }
    errors = {}
    physical = []
    topology_errors = []
    zone_errors = []
    relation_errors = []
    projectability_errors = []
    prefix_substitutions = []
    config_counts = Counter()
    total_active = Counter()
    presence = Counter()
    absence = Counter()
    loaded = Counter()
    empty = Counter()
    alone = Counter()
    target_counts = Counter()
    relation_counts = Counter()
    target_zones = Counter()
    target_segments = Counter()
    all_segments = Counter()
    roles = Counter()
    identity_target_zones: dict[str, set[str]] = defaultdict(set)
    geometry_by_config: dict[str, set[str]] = defaultdict(set)
    duplicate_geometry = []
    scenario_ids = set()
    scenario_families = set()
    high_risk_color = []
    partial_occlusion = []
    retained_relation = 0
    retained_zone = 0
    preserved_field_violations = []
    for row in rows:
        plan_id = str(row.get('source_v1_plan_id') or '')
        source = source_by_id.get(plan_id)
        if source is None:
            errors[row.get('plan_id', '<unknown>')] = ['unknown_v1_parent']
            continue
        row_errors = validate_v2_row(row, source)
        if row_errors:
            errors[plan_id] = row_errors
        for error in row_errors:
            if error.startswith('full_scenario_validator'):
                topology_errors.append({'plan_id': plan_id, 'error': error})
            if error == 'invalid_zone_segment_or_ratio':
                zone_errors.append(plan_id)
            if (
                'relation' in error
                or error == 'ineligible_relation_family'
            ):
                relation_errors.append({'plan_id': plan_id, 'error': error})
            if 'projectability' in error:
                projectability_errors.append({
                    'plan_id': plan_id,
                    'error': error,
                })
            if error.startswith('preserved_field_changed'):
                preserved_field_violations.append({
                    'plan_id': plan_id,
                    'error': error,
                })
            if error.startswith('exact_subset_mismatch'):
                prefix_substitutions.append({
                    'plan_id': plan_id,
                    'error': error,
                })
        physical.extend({
            'plan_id': plan_id,
            **conflict,
        } for conflict in scenario_physical_conflicts(row))
        retained_relation += (
            row['relation_family'] == source['relation_family']
        )
        retained_zone += row['target_zone'] == source['target_zone']
        config = row['configuration_id']
        config_counts[config] += 1
        active = row['left_active_identities'] + row['right_active_identities']
        total_active[str(len(active))] += 1
        if len(active) == 1:
            alone[active[0]] += 1
        for identity in GLOBAL_IDENTITIES:
            if identity in active:
                presence[identity] += 1
            else:
                absence[identity] += 1
        for identity, state in row['payload_assignment'].items():
            (loaded if state == 'loaded' else empty)[identity] += 1
        target = row['target_identity']
        target_counts[target] += 1
        roles[f'{target}:target'] += 1
        identity_target_zones[target].add(row['target_zone'])
        relation_counts[row['relation_family']] += 1
        target_zones[row['target_zone']] += 1
        target_side = _side(target)
        target_segments[f'{target_side}:{row["target_segment"]}'] += 1
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
        for side in ('left', 'right'):
            for shuttle in row['scene']['rails'][side]['shuttles']:
                all_segments[
                    f'{side}:{shuttle["start_position"]["segment"]}'
                ] += 1
        geometry = row['geometry_key']
        if geometry in geometry_by_config[config]:
            duplicate_geometry.append(plan_id)
        geometry_by_config[config].add(geometry)
        if row['scenario_id'] in scenario_ids:
            topology_errors.append({
                'plan_id': plan_id,
                'error': 'duplicate_scenario_id',
            })
        scenario_ids.add(row['scenario_id'])
        if row['scenario_family'] in scenario_families:
            topology_errors.append({
                'plan_id': plan_id,
                'error': 'duplicate_scenario_family',
            })
        scenario_families.add(row['scenario_family'])
        projection = row['static_camera_projectability']
        if projection['partial_occlusion_risk_pairs']:
            partial_occlusion.append({
                'plan_id': plan_id,
                'pairs': projection['partial_occlusion_risk_pairs'],
            })
        active_set = set(active)
        for pair in (('R4', 'L3'), ('L1', 'R2')):
            if set(pair) <= active_set:
                high_risk_color.append({
                    'plan_id': plan_id,
                    'pair': list(pair),
                })
        if 'L4' in active_set:
            high_risk_color.append({
                'plan_id': plan_id,
                'pair': ['L4', 'bright_background'],
            })

    expected_configs = {
        row['configuration_id']
        for row in presence_inventory()
    }
    role_coverage = {
        identity: {
            role: roles[f'{identity}:{role}']
            for role in ('target', 'blocker', 'non_blocker', 'relation_neutral')
        }
        for identity in GLOBAL_IDENTITIES
    }
    projectable_identity_count = sum(
        len(row['static_camera_projectability']['active_identities'])
        for row in rows
    )
    checks = {
        'exactly_2040_rows': len(rows) == 2040,
        'exactly_255_configurations': (
            set(config_counts) == expected_configs
            and len(config_counts) == 255
        ),
        'exactly_eight_variants_per_configuration': all(
            config_counts[config] == 8 for config in expected_configs
        ),
        'all_24_nonempty_cardinality_pairs': len({
            (
                len(row['left_active_identities']),
                len(row['right_active_identities']),
            )
            for row in rows
        }) == 24,
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
        'exact_relation_family_totals': (
            dict(sorted(relation_counts.items()))
            == EXPECTED_RELATION_TOTALS
        ),
        'exact_target_zone_totals': (
            dict(sorted(target_zones.items())) == EXPECTED_ZONE_TOTALS
        ),
        'all_14_public_segments_both_rails': all(
            all_segments[f'{side}:{segment}'] > 0
            for side in ('left', 'right')
            for segment in valid_public_segments(side)
        ),
        'zero_invalid_zone_segment_or_ratio': not zone_errors,
        'zero_same_segment_physical_impossibilities': not physical,
        'zero_topology_or_switch_routing_violations': not topology_errors,
        'zero_relation_violations': not relation_errors,
        'zero_pairwise_separation_violations': not physical,
        'zero_exact_subset_or_prefix_substitutions': not prefix_substitutions,
        'zero_preserved_field_violations': not preserved_field_violations,
        'zero_nonprojectable_active_identities': (
            not projectability_errors
            and projectable_identity_count == sum(
                len(row['left_active_identities'])
                + len(row['right_active_identities'])
                for row in rows
            )
        ),
        'zero_invalid_bounding_box_projections': all(
            not row['static_camera_projectability']['invalid_bbox_identities']
            for row in rows
        ),
        'every_active_identity_has_visible_unique_color_region': all(
            not row['static_camera_projectability'][
                'invisible_identity_color_identities'
            ]
            for row in rows
        ),
        'eight_unique_geometry_keys_per_configuration': (
            not duplicate_geometry
            and all(
                len(geometry_by_config[config]) == 8
                for config in expected_configs
            )
        ),
        'every_identity_all_six_target_zones': all(
            identity_target_zones[identity] == set(POSITION_ZONES)
            for identity in GLOBAL_IDENTITIES
        ),
        'every_identity_all_applicable_roles': all(
            all(value > 0 for value in coverage.values())
            for coverage in role_coverage.values()
        ),
        'all_v1_relation_families_retained': retained_relation == len(rows),
        'fixed_visual_schema_v3': (
            VISUAL_STATE_SCHEMA_VERSION == 'room315.visual_state.v3'
        ),
        'fixed_vectorizer_dimension_200': (
            VisualStateLabelVectorizer().dim == 200
        ),
        'dataset_inferred_capacity_false': (
            VisualStateLabelVectorizer().to_json()[
                'capacity_inferred_from_dataset'
            ] is False
        ),
        'protected_artifacts_unchanged': (
            protected_before['passed']
            and protected_after['passed']
            and protected_before == protected_after
        ),
        'all_rows_pass_full_validation': not errors,
    }
    return {
        'schema_version': V2_AUDIT_SCHEMA,
        'seed': V2_SEED,
        'passed': all(checks.values()),
        'checks': checks,
        'source': {
            'v1_plan_path': str(V1_PLAN),
            'v1_plan_schema': PLAN_SCHEMA,
            'v1_plan_sha256': sha256(V1_PLAN),
            'v1_failure_reproduction': _v1_failure_audit(v1_rows),
        },
        'joint_assignment': solver,
        'preservation': {
            'unchanged_fields_checked_row_by_row': list(
                UNCHANGED_V1_FIELDS
            ),
            'retained_original_relation_family_rows': retained_relation,
            'relation_reassignment_rows': len(rows) - retained_relation,
            'retained_original_target_zone_rows': retained_zone,
            'changed_target_zone_rows': len(rows) - retained_zone,
        },
        'distributions': {
            'configuration_variant_count': dict(sorted(config_counts.items())),
            'total_active_count': dict(sorted(total_active.items())),
            'identity_presence': dict(sorted(presence.items())),
            'identity_absence': dict(sorted(absence.items())),
            'identity_loaded': dict(sorted(loaded.items())),
            'identity_empty': dict(sorted(empty.items())),
            'identity_alone': dict(sorted(alone.items())),
            'identity_target': dict(sorted(target_counts.items())),
            'identity_roles': role_coverage,
            'identity_target_zones': {
                identity: sorted(zones)
                for identity, zones in sorted(identity_target_zones.items())
            },
            'relation_family': dict(sorted(relation_counts.items())),
            'target_zone': dict(sorted(target_zones.items())),
            'target_segment': dict(sorted(target_segments.items())),
            'all_active_segments': dict(sorted(all_segments.items())),
        },
        'violations': {
            'row_validation': errors,
            'physical_or_pairwise_separation': physical,
            'topology_or_switch_routing': topology_errors,
            'zone_segment_or_ratio': zone_errors,
            'relation': relation_errors,
            'exact_subset_or_prefix_substitution': prefix_substitutions,
            'preserved_fields': preserved_field_violations,
            'camera_projectability': projectability_errors,
            'duplicate_geometry_keys': duplicate_geometry,
        },
        'camera_projectability': {
            'camera_calibration_path': str(_default_camera_model_path()),
            'camera_calibration_sha256': sha256(
                _default_camera_model_path()
            ),
            'projectable_active_identity_instances': (
                projectable_identity_count
            ),
            'invalid_bbox_count': sum(
                len(row['static_camera_projectability'][
                    'invalid_bbox_identities'
                ])
                for row in rows
            ),
            'invisible_unique_color_region_count': sum(
                len(row['static_camera_projectability'][
                    'invisible_identity_color_identities'
                ])
                for row in rows
            ),
            'partial_occlusion_risk_row_count': len(partial_occlusion),
            'partial_occlusion_risk_sample': partial_occlusion[:100],
            'high_risk_color_candidate_count': len(high_risk_color),
            'high_risk_color_candidate_sample': high_risk_color[:100],
            'static_limit': (
                'Projectability, bbox, identity-plane area, payload keepout, '
                'and physical non-overlap are checked. Pixel appearance still '
                'requires a later captured gallery review.'
            ),
        },
        'fixed_schema': {
            'visual_state_schema': VISUAL_STATE_SCHEMA_VERSION,
            'fixed_identity_order': list(GLOBAL_IDENTITIES),
            'vectorizer_dimension': VisualStateLabelVectorizer().dim,
            'dataset_inferred_capacity': VisualStateLabelVectorizer().to_json()[
                'capacity_inferred_from_dataset'
            ],
        },
        'protected_artifacts_before': protected_before,
        'protected_artifacts_after': protected_after,
        'topology_compatibility_schema': topology['schema_version'],
        'capture_executed': False,
        'training_executed': False,
        'kairos_accessed': False,
        'split_files_created': False,
        'approved_for_capture': False,
    }


def _compatibility_markdown(topology: dict[str, Any]) -> str:
    lines = [
        '# Room 315 topology and target-zone compatibility',
        '',
        'This table is generated from the authoritative rail networks, device '
        'configuration, calibrated path lengths, validated scenario generator, '
        'world-space shuttle geometry, and overhead-camera model.',
        '',
        '| Side | Segment | Length m | Predecessors | Successors | Suffix | '
        'Valid target zones | Switches | Slots | Max same-segment shuttles |',
        '|---|---|---:|---|---|---|---|---|---|---:|',
    ]
    for side in ('left', 'right'):
        for segment, entry in topology['rails'][side].items():
            lines.append(
                f'| {side} | {segment} | {entry["segment_length_m"]:.6f} | '
                f'{", ".join(entry["predecessor_segments"])} | '
                f'{", ".join(entry["successor_segments"])} | '
                f'{entry["branch_suffix"] or "-"} | '
                f'{", ".join(entry["valid_target_zones"])} | '
                f'{", ".join(entry["switch_associations"]) or "-"} | '
                f'{", ".join(slot["name"] for slot in entry["slot_associations"]) or "-"} | '
                f'{entry["maximum_physically_feasible_same_segment_shuttle_count"]} |'
            )
    lines.extend([
        '',
        '## Relation-family zone feasibility',
        '',
        '| Family | Side | Boundary | Switch | Slot | Merge/conflict | Buffer | Ordinary |',
        '|---|---|---|---|---|---|---|---|',
    ])
    for family, sides in topology[
        'relation_family_zone_feasibility'
    ].items():
        for side, zones in sides.items():
            flag = lambda name: 'yes' if zones[name] else 'no'
            lines.append(
                f'| {family} | {side} | {flag("boundary")} | '
                f'{flag("switch")} | {flag("slot")} | '
                f'{flag("merge_conflict")} | {flag("buffer")} | '
                f'{flag("ordinary")} |'
            )
    return '\n'.join(lines) + '\n'


def _audit_markdown(audit: dict[str, Any]) -> str:
    relation = audit['distributions']['relation_family']
    zones = audit['distributions']['target_zone']
    return f'''# Room 315 arbitrary-subset production design v2 audit

Verdict: **{"PASS" if audit["passed"] else "FAIL"}**

- Rows: {sum(audit["distributions"]["total_active_count"].values())}
- Configurations: {len(audit["distributions"]["configuration_variant_count"])}
- Retained v1 relation families: {audit["preservation"]["retained_original_relation_family_rows"]}
- Relation reassignments: {audit["preservation"]["relation_reassignment_rows"]}
- Retained v1 target zones: {audit["preservation"]["retained_original_target_zone_rows"]}
- Changed target zones: {audit["preservation"]["changed_target_zone_rows"]}
- Physical conflicts: {len(audit["violations"]["physical_or_pairwise_separation"])}
- Topology/switch violations: {len(audit["violations"]["topology_or_switch_routing"])}
- Zone/segment/ratio violations: {len(audit["violations"]["zone_segment_or_ratio"])}
- Camera-projectability violations: {len(audit["violations"]["camera_projectability"])}
- Prefix substitutions: {len(audit["violations"]["exact_subset_or_prefix_substitution"])}
- Duplicate geometry keys: {len(audit["violations"]["duplicate_geometry_keys"])}

## Exact relation totals

```json
{json.dumps(relation, indent=2, sort_keys=True)}
```

## Exact target-zone totals

```json
{json.dumps(zones, indent=2, sort_keys=True)}
```

No Gazebo capture, dataset split, training, Kairos access, or approval action is
part of this design-only package.
'''


def _mapping_rows(
    v1_rows: list[dict[str, Any]],
    rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    result = []
    for source, row in zip(v1_rows, rows):
        changed = {}
        for field in (
            'target_zone',
            'target_segment',
            'relation_family',
            'relation_identities',
            'relation_neutral_identities',
            'switch_pattern_variant',
        ):
            if source.get(field) != row.get(field):
                changed[field] = {
                    'v1': source.get(field),
                    'v2': row.get(field),
                }
        result.append({
            'source_v1_plan_id': source['plan_id'],
            'v2_plan_id': row['plan_id'],
            'configuration_id': row['configuration_id'],
            'variant_index': row['variant_index'],
            'unchanged_fields_verified': list(UNCHANGED_V1_FIELDS),
            'changed_authorized_fields': changed,
            'v2_geometry_key': row['geometry_key'],
            'v2_scenario_id': row['scenario_id'],
        })
    return result


def _relation_reassignments(
    v1_rows: list[dict[str, Any]],
    rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    return [
        {
            'source_v1_plan_id': source['plan_id'],
            'original_relation_family': source['relation_family'],
            'replacement_relation_family': row['relation_family'],
            'reason': (
                'original family had no physically valid joint topology/'
                'zone/segment assignment'
            ),
        }
        for source, row in zip(v1_rows, rows)
        if source['relation_family'] != row['relation_family']
    ]


def _infeasible_summary(
    v1_rows: list[dict[str, Any]],
    rows: list[dict[str, Any]],
    solver: dict[str, Any],
) -> dict[str, Any]:
    original_family_zone = Counter(
        f'{row["relation_family"]}|{_side(row["target_identity"])}|{row["target_zone"]}'
        for row in v1_rows
        if not relation_zone_feasible(
            row['relation_family'],
            _side(row['target_identity']),
            row['target_zone'],
        )
    )
    return {
        'schema_version': 'room315.infeasible_candidate_summary.v2',
        'v1_failure_reproduction': _v1_failure_audit(v1_rows),
        'joint_candidate_rejections': {
            'infeasible_original_family_side_zone_rows': sum(
                original_family_zone.values()
            ),
            'by_family_side_zone': dict(sorted(original_family_zone.items())),
            'short_connector_same_segment_pairs_rejected': True,
            'A1E_same_segment_pair_rejected': all(
                _segment_lengths(side)['A1E']
                < MIN_SAME_SEGMENT_START_SEPARATION_M
                for side in ('left', 'right')
            ),
            'adjacent_switch_connectors_rejected_by_world_geometry': True,
            'adjacent_merge_connectors_rejected_by_world_geometry': True,
        },
        'solver': solver,
        'final_rows': len(rows),
        'final_infeasible_rows': 0,
        'formal_infeasibility': None,
    }


def _package_manifest(
    root: Path,
    *,
    declared_root: Path,
) -> dict[str, Any]:
    files = {}
    for path in sorted(item for item in root.rglob('*') if item.is_file()):
        relative = str(path.relative_to(root))
        if relative == 'package_manifest.json':
            continue
        files[relative] = {
            'bytes': path.stat().st_size,
            'sha256': sha256(path),
        }
    return {
        'schema_version': V2_PACKAGE_SCHEMA,
        'package_root': str(declared_root),
        'seed': V2_SEED,
        'scenario_count': 2040,
        'source_v1_plan': str(V1_PLAN),
        'source_v1_sha256': sha256(V1_PLAN),
        'visual_state_schema': VISUAL_STATE_SCHEMA_VERSION,
        'fixed_identity_order': list(GLOBAL_IDENTITIES),
        'fixed_vectorizer_dimension': VisualStateLabelVectorizer().dim,
        'capacity_inferred_from_dataset': False,
        'capture_executed': False,
        'approved_for_capture': False,
        'split_files_created': False,
        'training_executed': False,
        'files': files,
    }


def validate_package_manifest(root: Path) -> dict[str, Any]:
    manifest = json.loads(
        (root / 'package_manifest.json').read_text(encoding='utf-8')
    )
    failures = []
    for relative, expected in manifest['files'].items():
        path = root / relative
        if not path.is_file():
            failures.append(f'missing:{relative}')
            continue
        if path.stat().st_size != expected['bytes']:
            failures.append(f'size:{relative}')
        if sha256(path) != expected['sha256']:
            failures.append(f'sha256:{relative}')
    undeclared = sorted(
        str(path.relative_to(root))
        for path in root.rglob('*')
        if path.is_file()
        and str(path.relative_to(root)) != 'package_manifest.json'
        and str(path.relative_to(root)) not in manifest['files']
    )
    failures.extend(f'undeclared:{relative}' for relative in undeclared)
    return {
        'passed': not failures,
        'verified_file_count': len(manifest['files']),
        'failures': failures,
    }


def _readme(package_root: Path, audit: dict[str, Any]) -> str:
    return f'''# Room 315 arbitrary-subset production design v2

This immutable-input, design-only package materializes all 2,040 planned rows
from `{V1_PLAN}` using deterministic seed `{V2_SEED}`. Every row preserves its
exact v1 identity subset, target, payload assignment, planned future role, and
relation family. Target zones and complete geometry are jointly constrained.

Static audit verdict: `{"PASS" if audit["passed"] else "FAIL"}`.

Important boundaries:

- This is not a capture-ready package.
- No capture state, capture script, dataset directory, approval file, or split
  JSONL is included.
- No Gazebo capture, training, or Kairos operation has run.
- `planned_partition_role` is preserved historical planning metadata only; it
  is not an actual split.
- Static camera checks do not replace a later human review of captured pixels.

Regenerate to a new empty path:

```bash
python3 {SCRIPT_DIR / "room_315_arbitrary_subset_visual_v2.py"} prepare \\
  --v1-plan {V1_PLAN} \\
  --output /path/to/new-empty-directory \\
  --declared-root {package_root}
```

Validate this package:

```bash
python3 {SCRIPT_DIR / "room_315_arbitrary_subset_visual_v2.py"} validate-package \\
  --package {package_root}
```

Proceeding to a separate capture-ready-package task is allowed only while
`design_v2_audit.json` remains PASS and the protected fingerprints remain
unchanged. This package itself does not approve capture.
'''


def prepare_v2_package(
    output: Path,
    *,
    v1_plan: Path = V1_PLAN,
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
    if v1_plan.expanduser().resolve() != V1_PLAN.resolve():
        raise ArbitrarySubsetError(
            f'v2 must consume the immutable authoritative v1 plan: {V1_PLAN}'
        )
    protected_before = protected_artifact_audit()
    if not protected_before['passed']:
        raise ArbitrarySubsetError(
            f'protected artifact preflight failed: {protected_before}'
        )
    v1_rows = read_jsonl(V1_PLAN)
    if (
        len(v1_rows) != 2040
        or sha256(V1_PLAN)
        != 'd27619a77b9dab1737809882c31b41ba3075e4f4026915bc4b645679f2e0113f'
    ):
        raise ArbitrarySubsetError(
            'immutable v1 row count or SHA-256 does not match the contract'
        )
    topology = topology_zone_compatibility()
    rows, solver = materialize_v2_rows(v1_rows)
    protected_after = protected_artifact_audit()
    audit = design_v2_audit(
        v1_rows,
        rows,
        solver,
        topology,
        protected_before,
        protected_after,
    )
    if not audit['passed']:
        raise ArbitrarySubsetError(
            f'v2 full audit failed: {audit["checks"]}'
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(
        prefix=f'.{output.name}.',
        dir=str(output.parent),
    ))
    try:
        write_jsonl(
            temporary / 'configuration_variant_plan_v2.jsonl',
            rows,
        )
        write_json(temporary / 'design_v2_audit.json', audit)
        (temporary / 'design_v2_audit.md').write_text(
            _audit_markdown(audit),
            encoding='utf-8',
        )
        write_json(
            temporary / 'topology_zone_compatibility.json',
            topology,
        )
        (temporary / 'topology_zone_compatibility.md').write_text(
            _compatibility_markdown(topology),
            encoding='utf-8',
        )
        write_jsonl(
            temporary / 'relation_reassignment_log.jsonl',
            _relation_reassignments(v1_rows, rows),
        )
        write_jsonl(
            temporary / 'v1_to_v2_mapping.jsonl',
            _mapping_rows(v1_rows, rows),
        )
        write_json(
            temporary / 'infeasible_candidate_summary.json',
            _infeasible_summary(v1_rows, rows, solver),
        )
        (temporary / 'README.md').write_text(
            _readme(declared_root, audit),
            encoding='utf-8',
        )
        write_json(
            temporary / 'package_manifest.json',
            _package_manifest(temporary, declared_root=declared_root),
        )
        validation = validate_package_manifest(temporary)
        if not validation['passed']:
            raise ArbitrarySubsetError(
                f'package manifest validation failed: {validation}'
            )
        os.replace(temporary, output)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return output


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest='command', required=True)
    prepare = commands.add_parser('prepare')
    prepare.add_argument('--v1-plan', type=Path, default=V1_PLAN)
    prepare.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    prepare.add_argument('--declared-root', type=Path)
    validate = commands.add_parser('validate-package')
    validate.add_argument('--package', type=Path, required=True)
    topology = commands.add_parser('write-topology')
    topology.add_argument('--json', type=Path, required=True)
    topology.add_argument('--markdown', type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.command == 'prepare':
        output = prepare_v2_package(
            args.output,
            v1_plan=args.v1_plan,
            declared_root=args.declared_root,
        )
        print(f'ARBITRARY_SUBSET_V2_PREPARED {output}')
        return 0
    if args.command == 'validate-package':
        result = validate_package_manifest(args.package)
        print(
            'V2_PACKAGE_VALID'
            if result['passed']
            else 'V2_PACKAGE_INVALID'
        )
        if not result['passed']:
            print(json.dumps(result, sort_keys=True))
        return 0 if result['passed'] else 2
    if args.command == 'write-topology':
        topology = topology_zone_compatibility()
        write_json(args.json, topology)
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(
            _compatibility_markdown(topology),
            encoding='utf-8',
        )
        print('TOPOLOGY_ZONE_COMPATIBILITY_WRITTEN')
        return 0
    raise AssertionError(args.command)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (
        ArbitrarySubsetError,
        FileExistsError,
        OSError,
        ValueError,
    ) as exc:
        print(f'error: {exc}', file=os.sys.stderr)
        raise SystemExit(1) from None
