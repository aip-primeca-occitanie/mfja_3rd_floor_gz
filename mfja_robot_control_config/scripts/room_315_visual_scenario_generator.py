#!/usr/bin/env python3
"""Generate deterministic Gazebo scenes for Room 315 visual-state training.

The output is a capture manifest, not a model-input file.  Each manifest row
describes how Gazebo must be configured before the two overhead images are
captured.  Labels are produced later from simulator oracle state and are never
included under ``model_input``.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import random
import sys
from collections import Counter
from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_rail_defaults import public_rail_segment_lengths
from room_315_shuttle_geometry import ShuttlePosition
from room_315_shuttle_geometry import shuttle_position_conflicts
from room_315_visual_fleet import AUTHORITATIVE_VISUAL_FLEET
from room_315_visual_fleet import identities_for_side


SCHEMA_VERSION = 'room315.visual_capture_scenario.v1'
REQUIRED_CAMERAS = ('left_rail_rgb', 'right_rail_rgb')
SIDES = ('right', 'left')
SWITCH_NAMES = ('A1', 'A2', 'A3', 'A4')
START_SLOTS = (1, 2, 3, 4)
SCENE_TYPES = ('empty', 'single', 'multi_same_rail', 'dual_rail')
BLOCKER_SCENE_TYPES = (
    'blocker_ahead_same_segment',
    'nonblocker_behind_same_segment',
    'blocker_intermediate_segment',
    'nonblocker_adjacent_branch',
    'multi_blocker',
)
BLOCKER_SCENE_ALIASES = {
    # The corrected-pipeline requirement used this shortened spelling.  Keep
    # the established hard-negative semantic/name canonical while accepting
    # the spelling in new configuration files.
    'blocker_adjacent_branch': 'nonblocker_adjacent_branch',
}
BLOCKER_RAIL_SCOPES = (
    'left_four',
    'right_four',
    'dual_four_plus_four',
)
ARBITRARY_IDENTITY_PRESENCE_PROFILE = 'arbitrary_identity_subset'
DEFAULT_BLOCKER_RAIL_SCOPE_WEIGHTS = {
    'left_four': 0.40,
    'right_four': 0.40,
    'dual_four_plus_four': 0.20,
}
ALL_SCENE_TYPES = SCENE_TYPES + BLOCKER_SCENE_TYPES
MIN_SAME_SEGMENT_START_SEPARATION_RATIO = 0.21
MIN_SAME_SEGMENT_START_SEPARATION_M = 0.46
MAX_PHYSICAL_PLACEMENT_ATTEMPTS = 512
# Phase the mirrored rail cycles so their intermediate blockers cover all
# public branch segments within the default 120-scenario plan.
ROUTE_INTERMEDIATE_OFFSET_PHASE = {'right': 2, 'left': 0}
POSITION_ZONES = (
    'boundary',
    'switch',
    'slot',
    'merge_conflict',
    'buffer',
    'ordinary',
)
RELATION_POSITION_ZONES = {
    'ahead_region',
    'behind_region',
    'adjacent_branch',
    'intermediate_route',
    'relation_neutral',
}
BRANCH_SEGMENT_PAIRS = tuple(
    (f'A{prefix}E', f'A{prefix}I')
    for prefix in ('1', '2', '3', '4', '12', '34')
)
ADJACENT_PREFIXES_BY_ZONE = {
    'slot': ('A12', 'A34'),
    'buffer': ('A12', 'A34'),
    'merge_conflict': ('A12', 'A34'),
    'switch': ('A12', 'A34'),
    'boundary': ('A12', 'A34'),
    'ordinary': ('A12', 'A34'),
}
RIGHT_BRANCH_CYCLES = {
    'exterior': (
        'A23', 'A3E', 'A34E', 'A4E',
        'A14', 'A1E', 'A12E', 'A2E',
    ),
    'interior': (
        'A23', 'A3I', 'A34I', 'A4I',
        'A14', 'A1I', 'A12I', 'A2I',
    ),
}
LEFT_BRANCH_CYCLES = {
    'exterior': (
        'A14', 'A1E', 'A12E', 'A2E',
        'A23', 'A3E', 'A34E', 'A4E',
    ),
    'interior': (
        'A14', 'A1I', 'A12I', 'A2I',
        'A23', 'A3I', 'A34I', 'A4I',
    ),
}
SLOT_RATIOS = {
    'right': {
        'A12E': (0.411866742, 0.653073633),
        'A34E': (0.447469343, 0.683726992),
    },
    'left': {
        'A12E': (0.428330934, 0.674370792),
        'A34E': (0.427586575, 0.668424664),
    },
}
LEGACY_KEYS = {
    'action',
    'action_vector',
    'language',
    'last_command',
    'next_action',
    'observable_state',
    'observation.state',
    'pddl_problem',
    'privileged_eval',
    'symbolic_next_action',
    'task',
}


class VisualScenarioError(ValueError):
    """Raised when a visual capture scenario is invalid."""


def _pretty_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True)


def _canonical_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'))


def _hash(value: Any, length: int = 12) -> str:
    return hashlib.sha256(_canonical_json(value).encode('utf-8')).hexdigest()[:length]


def _stable_seed(*parts: Any) -> int:
    payload = ':'.join(str(part) for part in parts)
    return int(hashlib.sha256(payload.encode('utf-8')).hexdigest()[:16], 16)


@lru_cache(maxsize=2)
def _segment_lengths(side: str) -> dict[str, float]:
    lengths = {
        str(segment).upper(): float(length)
        for segment, length in public_rail_segment_lengths(side).items()
    }
    if set(lengths) != {
        segment
        for pair in BRANCH_SEGMENT_PAIRS
        for segment in pair
    } | {'A14', 'A23'}:
        raise VisualScenarioError(
            f'{side} rail topology does not expose the expected 14 public segments'
        )
    return dict(sorted(lengths.items()))


def valid_public_segments(side: str) -> tuple[str, ...]:
    return tuple(_segment_lengths(side))


def _branch_cycle(side: str, branch: str) -> tuple[str, ...]:
    cycles = RIGHT_BRANCH_CYCLES if side == 'right' else LEFT_BRANCH_CYCLES
    cycle = cycles[branch]
    missing = sorted(set(cycle) - set(_segment_lengths(side)))
    if missing:
        raise VisualScenarioError(f'{side} {branch} cycle has unknown segments: {missing}')
    return cycle


def _branch_for_segment(segment: str, rng: random.Random) -> str:
    if segment.endswith('I'):
        return 'interior'
    if segment.endswith('E'):
        return 'exterior'
    return rng.choice(('exterior', 'interior'))


def _zone_segments(side: str, zone: str) -> tuple[str, ...]:
    all_segments = set(valid_public_segments(side))
    if zone == 'slot':
        selected = set(SLOT_RATIOS[side])
    elif zone == 'buffer':
        selected = {'A12I', 'A34I'}
    elif zone == 'merge_conflict':
        selected = {
            'A14', 'A23',
            'A2E', 'A2I', 'A4E', 'A4I',
        }
    elif zone == 'switch':
        selected = {
            f'A{prefix}{branch}'
            for prefix in ('1', '2', '3', '4')
            for branch in ('E', 'I')
        }
    elif zone in {'boundary', 'ordinary'}:
        selected = all_segments
    else:
        raise VisualScenarioError(f'unsupported position zone: {zone!r}')
    return tuple(sorted(selected & all_segments))


def _seeded_cycle_pick(
    options: list[str] | tuple[str, ...],
    *,
    dataset_seed: int,
    key: str,
    index: int,
) -> str:
    ordered = sorted(set(options))
    if not ordered:
        raise VisualScenarioError(f'no valid rail segments for {key}')
    random.Random(_stable_seed(dataset_seed, key)).shuffle(ordered)
    return ordered[index % len(ordered)]


def _sample_ratio(
    zone: str,
    *,
    side: str,
    segment: str,
    rng: random.Random,
    minimum: float = 0.04,
    maximum: float = 0.96,
    boundary_preference: str | None = None,
) -> float:
    if minimum > maximum:
        raise VisualScenarioError(
            f'cannot place a shuttle on {side}:{segment}; empty ratio interval'
        )
    if zone == 'slot':
        slot_segment = segment if segment in SLOT_RATIOS[side] else f'{segment[:-1]}E'
        anchors = SLOT_RATIOS[side].get(slot_segment)
        if not anchors:
            raise VisualScenarioError(f'{side}:{segment} has no adjacent slot zone')
        anchor = rng.choice(anchors)
        sampled = anchor + rng.uniform(-0.025, 0.025)
    elif zone == 'boundary':
        edge = boundary_preference or rng.choice(('start', 'end'))
        sampled = (
            rng.uniform(0.04, 0.10)
            if edge == 'start'
            else rng.uniform(0.90, 0.96)
        )
    elif zone == 'switch':
        sampled = rng.choice((
            rng.uniform(0.07, 0.15),
            rng.uniform(0.85, 0.93),
        ))
    elif zone == 'merge_conflict':
        sampled = rng.uniform(0.38, 0.62)
    elif zone == 'buffer':
        sampled = rng.uniform(0.28, 0.72)
    elif zone == 'ordinary':
        sampled = rng.uniform(0.20, 0.80)
    else:
        raise VisualScenarioError(f'unsupported position zone: {zone!r}')
    return round(min(max(sampled, minimum), maximum), 6)


def _load_config(path: Path) -> dict[str, Any]:
    loaded = yaml.safe_load(path.expanduser().read_text(encoding='utf-8')) or {}
    if not isinstance(loaded, dict):
        raise VisualScenarioError(f'configuration must contain an object: {path}')
    if loaded.get('schema_version') != SCHEMA_VERSION:
        raise VisualScenarioError(
            f'configuration schema_version must be {SCHEMA_VERSION!r}'
        )
    return loaded


def _positive_int(raw: Any, *, context: str, minimum: int = 1) -> int:
    try:
        value = int(raw)
    except (TypeError, ValueError) as exc:
        raise VisualScenarioError(f'{context} must be an integer') from exc
    if value < minimum:
        raise VisualScenarioError(f'{context} must be at least {minimum}')
    return value


def _weights(config: dict[str, Any]) -> dict[str, float]:
    raw = config.get('scene_type_weights')
    if not isinstance(raw, dict):
        raise VisualScenarioError('scene_type_weights must be an object')
    unexpected = sorted(set(raw) - set(SCENE_TYPES))
    missing = sorted(set(SCENE_TYPES) - set(raw))
    if unexpected or missing:
        raise VisualScenarioError(
            f'scene_type_weights mismatch; missing={missing}, unexpected={unexpected}'
        )
    parsed = {}
    for name in SCENE_TYPES:
        try:
            parsed[name] = float(raw[name])
        except (TypeError, ValueError) as exc:
            raise VisualScenarioError(
                f'scene_type_weights.{name} must be numeric'
            ) from exc
        if parsed[name] < 0.0:
            raise VisualScenarioError(f'scene_type_weights.{name} cannot be negative')
    if sum(parsed.values()) <= 0.0:
        raise VisualScenarioError('scene_type_weights must contain a positive weight')
    return parsed


def allocate_scene_counts(total: int, weights: dict[str, float]) -> dict[str, int]:
    """Allocate an exact total using the largest-remainder method."""
    total = _positive_int(total, context='scenario_count')
    weight_sum = sum(weights.values())
    exact = {name: total * weights[name] / weight_sum for name in SCENE_TYPES}
    counts = {name: int(exact[name]) for name in SCENE_TYPES}
    remainder = total - sum(counts.values())
    order = sorted(
        SCENE_TYPES,
        key=lambda name: (exact[name] - counts[name], weights[name], name),
        reverse=True,
    )
    for name in order[:remainder]:
        counts[name] += 1
    return counts


def _switch_patterns() -> tuple[tuple[str, tuple[str, ...]], ...]:
    return (
        ('all_exterior', ('exterior',) * 4),
        ('all_interior', ('interior',) * 4),
        ('alternating_exterior_first', ('exterior', 'interior', 'exterior', 'interior')),
        ('alternating_interior_first', ('interior', 'exterior', 'interior', 'exterior')),
        ('outer_interior', ('interior', 'exterior', 'exterior', 'interior')),
        ('inner_interior', ('exterior', 'interior', 'interior', 'exterior')),
        ('a1_interior', ('interior', 'exterior', 'exterior', 'exterior')),
        ('a2_interior', ('exterior', 'interior', 'exterior', 'exterior')),
        ('a3_interior', ('exterior', 'exterior', 'interior', 'exterior')),
        ('a4_interior', ('exterior', 'exterior', 'exterior', 'interior')),
        ('a1_exterior', ('exterior', 'interior', 'interior', 'interior')),
        ('a2_exterior', ('interior', 'exterior', 'interior', 'interior')),
        ('a3_exterior', ('interior', 'interior', 'exterior', 'interior')),
        ('a4_exterior', ('interior', 'interior', 'interior', 'exterior')),
        ('front_pair_interior', ('interior', 'interior', 'exterior', 'exterior')),
        ('rear_pair_interior', ('exterior', 'exterior', 'interior', 'interior')),
    )


def _switches(pattern_index: int) -> tuple[str, dict[str, str]]:
    name, values = _switch_patterns()[pattern_index % len(_switch_patterns())]
    return name, dict(zip(SWITCH_NAMES, values))


def _shuttle_id(side: str, index: int) -> str:
    return f'{"R" if side == "right" else "L"}{index}'


def _rail_scene(
    side: str,
    count: int,
    *,
    scene_index: int,
    variant: int,
    duplicate_attempt: int = 0,
) -> dict[str, Any]:
    if count < 0 or count > 4:
        raise VisualScenarioError(f'{side} shuttle count must be in [0, 4]')
    permutations = tuple(itertools.permutations(START_SLOTS, count)) if count else ((),)
    if duplicate_attempt > 1000:
        fallback_seed = int(
            hashlib.sha256(
                f'{side}:{count}:{scene_index}:{variant}:{duplicate_attempt}'.encode()
            ).hexdigest()[:16],
            16,
        )
        fallback = random.Random(fallback_seed)
        slots = permutations[fallback.randrange(len(permutations))]
        load_mask = fallback.randrange(1 << count) if count else 0
        switch_pattern_index = fallback.randrange(len(_switch_patterns()))
    else:
        slots = permutations[(scene_index * 5 + variant * 7) % len(permutations)]
        load_mask = (scene_index * 3 + variant + count) % (1 << count) if count else 0
        switch_pattern_index = (
            scene_index + variant * 3 + (0 if side == 'right' else 5)
        )
    shuttles = [
        {
            'id': _shuttle_id(side, index),
            'start_slot': int(slots[index - 1]),
            'loaded_state': 'loaded' if load_mask & (1 << (index - 1)) else 'empty',
        }
        for index in range(1, count + 1)
    ]
    pattern_name, switches = _switches(switch_pattern_index)
    return {
        'switch_pattern': pattern_name,
        'switches': switches,
        'shuttles': shuttles,
    }


def _scene_counts(scene_type: str, scene_index: int) -> tuple[int, int]:
    if scene_type == 'empty':
        return 0, 0
    if scene_type == 'single':
        return (1, 0) if scene_index % 2 == 0 else (0, 1)
    if scene_type == 'multi_same_rail':
        count = 2 + (scene_index % 3)
        return (count, 0) if scene_index % 2 == 0 else (0, count)
    if scene_type == 'dual_rail':
        right_count = 1 + (scene_index % 4)
        left_count = 1 + ((scene_index // 2 + 1) % 4)
        return right_count, left_count
    raise VisualScenarioError(f'unsupported scene type: {scene_type!r}')


def _launch_arguments(rails: dict[str, dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {
        'robots': 'none',
        'start_paused': False,
        'enable_room315_kinematic_shuttles': True,
        'enable_room315_rail_safety_supervisor': True,
        'enable_room315_visual_state_dataset_recorder': False,
        'enable_room315_visual_obstacles': False,
        'room315_enable_payload_visuals': True,
        'room315_shuttles_start_enabled': False,
        'room315_visual_debug_colors': False,
        'room315_show_device_markers': False,
    }
    for side in SIDES:
        shuttles = rails[side]['shuttles']
        result[f'enable_room315_{side}_rail'] = True
        result[f'room315_{side}_shuttle_count'] = len(shuttles)
        result[f'room315_{side}_start_slots'] = ','.join(
            str(shuttle['start_slot']) for shuttle in shuttles
        )
        result[f'room315_{side}_start_positions'] = ','.join(
            (
                f'{shuttle["start_position"]["segment"]}@'
                f'{float(shuttle["start_position"]["s_ratio"]):.6f}'
            )
            for shuttle in shuttles
            if isinstance(shuttle.get('start_position'), dict)
        )
        result[f'room315_{side}_loaded_shuttles'] = ','.join(
            shuttle['id']
            for shuttle in shuttles
            if shuttle['loaded_state'] == 'loaded'
        )
    return result


def _switch_command(rails: dict[str, dict[str, Any]]) -> str:
    assignments = []
    for side, suffix in (('right', 'R'), ('left', 'L')):
        assignments.extend(
            f'{name}{suffix}={state.upper()}'
            for name, state in rails[side]['switches'].items()
        )
    return ','.join(assignments)


def _family_payload(scene_type: str, scene: dict[str, Any]) -> dict[str, Any]:
    return {
        'scene_type': scene_type,
        'rails': scene['rails'],
        'obstacles': scene['obstacles'],
    }


def _build_scenario(
    scene_type: str,
    *,
    ordinal: int,
    type_index: int,
    seed: int,
    capture: dict[str, Any],
    duplicate_attempt: int = 0,
) -> dict[str, Any]:
    right_count, left_count = _scene_counts(scene_type, type_index)
    rails = {
        'right': _rail_scene(
            'right',
            right_count,
            scene_index=type_index,
            variant=seed % 97,
            duplicate_attempt=duplicate_attempt,
        ),
        'left': _rail_scene(
            'left',
            left_count,
            scene_index=type_index,
            variant=(seed + 11) % 97,
            duplicate_attempt=duplicate_attempt,
        ),
    }
    scene = {
        'rails': rails,
        'obstacles': [],
    }
    family_hash = _hash(_family_payload(scene_type, scene))
    scenario_id = f'visual_{ordinal:04d}_{scene_type}_{family_hash[:8]}'
    shuttles = [
        shuttle
        for side in SIDES
        for shuttle in rails[side]['shuttles']
    ]
    return {
        'schema_version': SCHEMA_VERSION,
        'scenario_id': scenario_id,
        'scenario_family': f'visual_family_{family_hash}',
        'scene_type': scene_type,
        'seed': seed,
        'scene': scene,
        'capture': {
            'mode': 'settled_static',
            'cameras': list(REQUIRED_CAMERAS),
            'frames': int(capture['frames_per_scenario']),
            'settle_seconds': float(capture['settle_seconds']),
            'frame_interval_seconds': float(capture['frame_interval_seconds']),
        },
        'setup': {
            'launch_package': 'mfja_3rd_floor_bringup',
            'launch_file': 'room_315_only.launch.py',
            'launch_arguments': _launch_arguments(rails),
            'switch_topic': '/mfja/conveyor/switch_cmd',
            'switch_command': _switch_command(rails),
        },
        'expected_label_coverage': {
            'shuttle_count': len(shuttles),
            'loaded_count': sum(
                shuttle['loaded_state'] == 'loaded' for shuttle in shuttles
            ),
            'empty_count': sum(
                shuttle['loaded_state'] == 'empty' for shuttle in shuttles
            ),
            'switch_count': len(SWITCH_NAMES) * len(SIDES),
            'obstacle_count': 0,
        },
    }


def _named_scene_counts(
    total: int,
    raw_weights: Any,
    scene_types: tuple[str, ...],
) -> dict[str, int]:
    if not isinstance(raw_weights, dict):
        raise VisualScenarioError('scene_type_weights must be an object')
    if set(raw_weights) != set(scene_types):
        raise VisualScenarioError(
            f'scene_type_weights must contain exactly {list(scene_types)}'
        )
    weights = {}
    for name in scene_types:
        try:
            weights[name] = float(raw_weights[name])
        except (TypeError, ValueError) as exc:
            raise VisualScenarioError(
                f'scene_type_weights.{name} must be numeric'
            ) from exc
        if weights[name] < 0.0:
            raise VisualScenarioError(
                f'scene_type_weights.{name} cannot be negative'
            )
    weight_sum = sum(weights.values())
    if weight_sum <= 0.0:
        raise VisualScenarioError('scene_type_weights must contain a positive weight')
    exact = {
        name: total * weights[name] / weight_sum
        for name in scene_types
    }
    counts = {name: int(exact[name]) for name in scene_types}
    remainder = total - sum(counts.values())
    order = sorted(
        scene_types,
        key=lambda name: (exact[name] - counts[name], weights[name], name),
        reverse=True,
    )
    for name in order[:remainder]:
        counts[name] += 1
    return counts


def normalized_blocker_scene_type_weights(raw_weights: Any) -> Any:
    """Accept documented aliases without changing the canonical family names."""
    if not isinstance(raw_weights, dict):
        return raw_weights
    normalized = dict(raw_weights)
    for alias, canonical in BLOCKER_SCENE_ALIASES.items():
        if alias not in normalized:
            continue
        if canonical in normalized:
            raise VisualScenarioError(
                f'scene_type_weights cannot contain both {alias!r} and '
                f'its canonical name {canonical!r}'
            )
        normalized[canonical] = normalized.pop(alias)
    return normalized


def _interleaved_named_types(
    counts: dict[str, int],
    scene_types: tuple[str, ...],
) -> list[str]:
    remaining = dict(counts)
    result = []
    while sum(remaining.values()):
        ranked = sorted(
            (name for name in scene_types if remaining[name]),
            key=lambda name: (
                remaining[name] / max(1, counts[name]),
                remaining[name],
                name,
            ),
            reverse=True,
        )
        selected = ranked[0]
        result.append(selected)
        remaining[selected] -= 1
    return result


def _sample_blocker_positions_once(
    scene_type: str,
    *,
    side: str,
    type_index: int,
    target_zone: str,
    seed: int,
    dataset_seed: int,
    placement_attempt: int,
) -> tuple[list[dict[str, Any]], list[dict[str, str]], str]:
    rng = random.Random(
        _stable_seed(
            seed,
            scene_type,
            side,
            type_index,
            'physical-placement',
            placement_attempt,
        )
    )
    zone = target_zone
    # ``type_index`` is explicitly the occurrence index for this
    # scene-family/side pair.  It is no longer derived from the size or order
    # of a temporary target/blocker position list.
    cycle_index = type_index
    lengths = _segment_lengths(side)

    def target_segment(candidates: set[str]) -> str:
        zone_candidates = set(_zone_segments(side, zone))
        eligible = sorted(candidates & zone_candidates)
        if not eligible:
            eligible = sorted(candidates)
        return _seeded_cycle_pick(
            eligible,
            dataset_seed=dataset_seed,
            key=f'{scene_type}:{side}:{zone}:target',
            index=cycle_index,
        )

    def position(
        segment: str,
        ratio: float,
        position_zone: str,
    ) -> dict[str, Any]:
        return {
            'segment': segment,
            's_ratio': round(ratio, 6),
            'position_zone': position_zone,
        }

    same_segment_candidates = {
        segment
        for segment, length in lengths.items()
        if length >= 1.0
    }
    if scene_type == 'blocker_ahead_same_segment':
        segment = target_segment(same_segment_candidates)
        separation = max(
            MIN_SAME_SEGMENT_START_SEPARATION_RATIO,
            MIN_SAME_SEGMENT_START_SEPARATION_M / lengths[segment],
        ) + rng.uniform(0.015, 0.055)
        selected = _sample_ratio(
            zone,
            side=side,
            segment=segment,
            rng=rng,
            maximum=0.96 - separation,
            boundary_preference='start',
        )
        blocker = selected + separation
        return (
            [
                position(segment, selected, zone),
                position(segment, blocker, 'ahead_region'),
            ],
            [{'other_shuttle_id': _shuttle_id(side, 2), 'relation': 'ahead_blocker'}],
            _branch_for_segment(segment, rng),
        )
    if scene_type == 'nonblocker_behind_same_segment':
        segment = target_segment(same_segment_candidates)
        separation = max(
            MIN_SAME_SEGMENT_START_SEPARATION_RATIO,
            MIN_SAME_SEGMENT_START_SEPARATION_M / lengths[segment],
        ) + rng.uniform(0.015, 0.055)
        selected = _sample_ratio(
            zone,
            side=side,
            segment=segment,
            rng=rng,
            minimum=0.04 + separation,
            boundary_preference='end',
        )
        behind = selected - separation
        return (
            [
                position(segment, selected, zone),
                position(segment, behind, 'behind_region'),
            ],
            [{'other_shuttle_id': _shuttle_id(side, 2), 'relation': 'behind_non_blocker'}],
            _branch_for_segment(segment, rng),
        )
    if scene_type == 'blocker_intermediate_segment':
        if zone == 'merge_conflict':
            segment = min(('A14', 'A23'), key=lambda name: lengths[name])
        else:
            segment = target_segment(set(lengths))
        branch = _branch_for_segment(segment, rng)
        cycle = _branch_cycle(side, branch)
        blocker_segment = cycle[
            (
                cycle.index(segment)
                + 1
                + (
                    cycle_index
                    + ROUTE_INTERMEDIATE_OFFSET_PHASE[side]
                    + placement_attempt
                )
                % 3
            )
            % len(cycle)
        ]
        return (
            [
                position(
                    segment,
                    _sample_ratio(zone, side=side, segment=segment, rng=rng),
                    zone,
                ),
                position(
                    blocker_segment,
                    _sample_ratio(
                        'ordinary',
                        side=side,
                        segment=blocker_segment,
                        rng=rng,
                    ),
                    'intermediate_route',
                ),
            ],
            [{
                'other_shuttle_id': _shuttle_id(side, 2),
                'relation': 'intermediate_blocker',
            }],
            branch,
        )
    if scene_type == 'nonblocker_adjacent_branch':
        prefix = _seeded_cycle_pick(
            ADJACENT_PREFIXES_BY_ZONE[zone],
            dataset_seed=dataset_seed,
            key=f'{scene_type}:{side}:{zone}:prefix',
            index=cycle_index + placement_attempt,
        )
        target_branch = rng.choice(('E', 'I'))
        segment = f'{prefix}{target_branch}'
        adjacent_segment = (
            f'{segment[:-1]}I'
            if segment.endswith('E')
            else f'{segment[:-1]}E'
        )
        # A12/A34 meet their switches and merge points at the segment
        # boundaries. Sampling the middle of the connector as a
        # "merge_conflict" would misclassify its physical location.
        target_sampling_zone = (
            'boundary' if zone == 'merge_conflict' else zone
        )
        return (
            [
                position(
                    segment,
                    _sample_ratio(
                        target_sampling_zone,
                        side=side,
                        segment=segment,
                        rng=rng,
                    ),
                    zone,
                ),
                position(
                    adjacent_segment,
                    _sample_ratio(
                        target_sampling_zone,
                        side=side,
                        segment=adjacent_segment,
                        rng=rng,
                    ),
                    'adjacent_branch',
                ),
            ],
            [{
                'other_shuttle_id': _shuttle_id(side, 2),
                'relation': 'adjacent_branch_non_blocker',
            }],
            _branch_for_segment(segment, rng),
        )
    if scene_type == 'multi_blocker':
        segment = target_segment(same_segment_candidates)
        separation = max(
            MIN_SAME_SEGMENT_START_SEPARATION_RATIO,
            MIN_SAME_SEGMENT_START_SEPARATION_M / lengths[segment],
        ) + rng.uniform(0.015, 0.045)
        selected = _sample_ratio(
            zone,
            side=side,
            segment=segment,
            rng=rng,
            maximum=0.96 - separation,
            boundary_preference='start',
        )
        branch = _branch_for_segment(segment, rng)
        cycle = _branch_cycle(side, branch)
        intermediate_segment = cycle[
            (
                cycle.index(segment)
                + 1
                + (
                    cycle_index
                    + ROUTE_INTERMEDIATE_OFFSET_PHASE[side]
                    + placement_attempt
                )
                % 3
            )
            % len(cycle)
        ]
        return (
            [
                position(segment, selected, zone),
                position(segment, selected + separation, 'ahead_region'),
                position(
                    intermediate_segment,
                    _sample_ratio(
                        'ordinary',
                        side=side,
                        segment=intermediate_segment,
                        rng=rng,
                    ),
                    'intermediate_route',
                ),
            ],
            [
                {
                    'other_shuttle_id': _shuttle_id(side, 2),
                    'relation': 'ahead_blocker',
                },
                {
                    'other_shuttle_id': _shuttle_id(side, 3),
                    'relation': 'intermediate_blocker',
                },
            ],
            branch,
        )
    raise VisualScenarioError(f'unsupported blocker scene type: {scene_type!r}')


def _position_list_conflicts(
    side: str,
    positions: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    return shuttle_position_conflicts(
        ShuttlePosition(
            shuttle_id=_shuttle_id(side, index),
            side=side,
            segment=str(position['segment']),
            s_ratio=float(position['s_ratio']),
        )
        for index, position in enumerate(positions, start=1)
    )


def _blocker_positions(
    scene_type: str,
    *,
    side: str,
    type_index: int,
    target_zone: str,
    seed: int,
    dataset_seed: int,
) -> tuple[list[dict[str, Any]], list[dict[str, str]], str]:
    for placement_attempt in range(MAX_PHYSICAL_PLACEMENT_ATTEMPTS):
        positions, relations, branch = _sample_blocker_positions_once(
            scene_type,
            side=side,
            type_index=type_index,
            target_zone=target_zone,
            seed=seed,
            dataset_seed=dataset_seed,
            placement_attempt=placement_attempt,
        )
        if not _position_list_conflicts(side, positions):
            return positions, relations, branch
    raise VisualScenarioError(
        f'could not place a collision-free {scene_type} scenario on the '
        f'{side} rail after {MAX_PHYSICAL_PLACEMENT_ATTEMPTS} attempts'
    )


def _relation_roles(scene_type: str) -> tuple[str, ...]:
    if scene_type in {
        'blocker_ahead_same_segment',
        'blocker_intermediate_segment',
    }:
        return ('blocker',)
    if scene_type in {
        'nonblocker_behind_same_segment',
        'nonblocker_adjacent_branch',
    }:
        return ('non_blocker',)
    if scene_type == 'multi_blocker':
        return ('blocker', 'blocker')
    raise VisualScenarioError(f'unsupported blocker scene type: {scene_type!r}')


def _select_relation_identities(
    side: str,
    target_identity: str,
    roles: tuple[str, ...],
    *,
    role_counts: dict[tuple[str, str, str], int],
    ordinal: int,
    future_targets_by_role: dict[str, tuple[str, ...]] | None = None,
) -> tuple[str, ...]:
    """Assign actors independently of the number/order of sampled positions."""
    available = set(identities_for_side(side)) - {target_identity}
    selected: list[str] = []
    pending_counts = dict(role_counts)
    future_targets_by_role = future_targets_by_role or {}
    for role_index, role in enumerate(roles):
        ranked = sorted(
            available,
            key=lambda identity: (
                pending_counts.get((side, role, identity), 0),
                sum(
                    future_target != identity
                    for future_target in future_targets_by_role.get(role, ())
                ),
                (
                    int(identity[1:])
                    - 1
                    - ordinal
                    - role_index
                ) % len(identities_for_side(side)),
                identity,
            ),
        )
        if not ranked:
            raise VisualScenarioError(
                f'{side} has no identity available for relation role {role!r}'
            )
        identity = ranked[0]
        selected.append(identity)
        available.remove(identity)
        key = (side, role, identity)
        pending_counts[key] = pending_counts.get(key, 0) + 1
    return tuple(selected)


def _positions_conflict(
    side: str,
    positions_by_identity: dict[str, dict[str, Any]],
) -> bool:
    if shuttle_position_conflicts(
        ShuttlePosition(
            shuttle_id=identity,
            side=side,
            segment=str(position['segment']),
            s_ratio=float(position['s_ratio']),
        )
        for identity, position in positions_by_identity.items()
    ):
        return True
    for first, second in itertools.combinations(
        positions_by_identity.values(),
        2,
    ):
        if first['segment'] != second['segment']:
            continue
        minimum_ratio = max(
            MIN_SAME_SEGMENT_START_SEPARATION_RATIO,
            MIN_SAME_SEGMENT_START_SEPARATION_M
            / _segment_lengths(side)[str(first['segment'])],
        )
        if abs(float(first['s_ratio']) - float(second['s_ratio'])) < minimum_ratio:
            return True
    return False


def _neutral_candidate_segments(
    side: str,
    *,
    active_branch: str | None,
    target_segment: str | None,
) -> tuple[str, ...]:
    segments = set(valid_public_segments(side))
    if active_branch is not None:
        inactive_suffix = 'I' if active_branch == 'exterior' else 'E'
        segments = {
            segment
            for segment in segments
            if segment.endswith(inactive_suffix)
        }
        # An opposite branch with the same prefix is the adjacent hard
        # negative, not relation-neutral.
        if target_segment and target_segment[-1:] in {'E', 'I'}:
            segments.discard(f'{target_segment[:-1]}{inactive_suffix}')
    if not segments:
        raise VisualScenarioError(
            f'no relation-neutral segment candidates for {side}:{active_branch}'
        )
    return tuple(sorted(segments))


def _place_neutral_shuttles(
    side: str,
    identities: tuple[str, ...],
    positions_by_identity: dict[str, dict[str, Any]],
    *,
    ordinal: int,
    dataset_seed: int,
    active_branch: str | None,
    target_segment: str | None,
    covered_segments: set[str],
) -> None:
    candidates = _neutral_candidate_segments(
        side,
        active_branch=active_branch,
        target_segment=target_segment,
    )
    for neutral_index, identity in enumerate(identities):
        placed = False
        requested_zone = POSITION_ZONES[
            (
                ordinal
                + int(identity[1:])
                + (0 if side == 'left' else 3)
            )
            % len(POSITION_ZONES)
        ]
        zone_candidates = tuple(
            segment
            for segment in candidates
            if segment in _zone_segments(side, requested_zone)
        )
        effective_zone = requested_zone if zone_candidates else 'ordinary'
        effective_candidates = zone_candidates or candidates
        for attempt in range(MAX_PHYSICAL_PLACEMENT_ATTEMPTS):
            preserve_requested_zone = attempt < max(
                24,
                len(effective_candidates) * 8,
            )
            attempt_candidates = (
                effective_candidates
                if preserve_requested_zone
                else candidates
            )
            attempt_zone = (
                effective_zone
                if preserve_requested_zone
                else 'ordinary'
            )
            uncovered_candidates = tuple(
                segment
                for segment in attempt_candidates
                if segment not in covered_segments
            )
            segment = _seeded_cycle_pick(
                (
                    uncovered_candidates
                    if uncovered_candidates and attempt < max(
                        8,
                        len(uncovered_candidates) * 4,
                    )
                    else attempt_candidates
                ),
                dataset_seed=dataset_seed,
                key=f'{side}:{identity}:relation-neutral-segment',
                index=ordinal + neutral_index + attempt,
            )
            rng = random.Random(_stable_seed(
                dataset_seed,
                ordinal,
                side,
                identity,
                'relation-neutral',
                attempt,
            ))
            candidate = {
                'segment': segment,
                's_ratio': _sample_ratio(
                    attempt_zone,
                    side=side,
                    segment=segment,
                    rng=rng,
                ),
                'position_zone': attempt_zone,
            }
            positions_by_identity[identity] = candidate
            if not _positions_conflict(side, positions_by_identity):
                placed = True
                covered_segments.add(segment)
                break
            positions_by_identity.pop(identity, None)
        if not placed:
            raise VisualScenarioError(
                f'could not place relation-neutral shuttle {identity} without '
                f'a physical conflict after {MAX_PHYSICAL_PLACEMENT_ATTEMPTS} attempts'
            )


def _rail_scope_counts(
    scenario_count: int,
    raw_weights: Any,
) -> dict[str, int]:
    return _named_scene_counts(
        scenario_count,
        raw_weights or DEFAULT_BLOCKER_RAIL_SCOPE_WEIGHTS,
        BLOCKER_RAIL_SCOPES,
    )


def _active_side_for_scope(scope: str, dual_index: int) -> str:
    if scope == 'left_four':
        return 'left'
    if scope == 'right_four':
        return 'right'
    if scope == 'dual_four_plus_four':
        return ('left', 'right')[dual_index % 2]
    raise VisualScenarioError(f'unsupported blocker rail scope: {scope!r}')


def _blocker_scope_schedule(
    ordered_types: list[str],
    scope_counts: dict[str, int],
) -> list[tuple[str, str]]:
    """Allocate exact rail scopes while balancing every family across sides."""
    remaining = dict(scope_counts)
    dual_total = remaining['dual_four_plus_four']
    dual_side_remaining = {
        'left': (dual_total + 1) // 2,
        'right': dual_total // 2,
    }
    desired_active = {
        'left': remaining['left_four'] + dual_side_remaining['left'],
        'right': remaining['right_four'] + dual_side_remaining['right'],
    }
    family_side_counts = Counter()
    active_counts = Counter()
    schedule: list[tuple[str, str]] = []
    for ordinal, scene_type in enumerate(ordered_types, start=1):
        candidates = []
        for scope in BLOCKER_RAIL_SCOPES:
            if remaining[scope] <= 0:
                continue
            if scope == 'dual_four_plus_four':
                sides = tuple(
                    side
                    for side in ('left', 'right')
                    if dual_side_remaining[side] > 0
                )
            else:
                sides = ('left',) if scope == 'left_four' else ('right',)
            for side in sides:
                candidates.append((
                    (
                        family_side_counts[(scene_type, side)],
                        active_counts[side] / max(1, desired_active[side]),
                        # Spread dual scenes rather than clustering them.
                        int(bool(
                            scope == 'dual_four_plus_four'
                            and schedule
                            and schedule[-1][0] == 'dual_four_plus_four'
                        )),
                        (ordinal + (0 if side == 'left' else 1)) % 2,
                        BLOCKER_RAIL_SCOPES.index(scope),
                    ),
                    scope,
                    side,
                ))
        if not candidates:
            raise VisualScenarioError('could not allocate blocker rail-scope schedule')
        _, scope, active_side = min(candidates)
        remaining[scope] -= 1
        if scope == 'dual_four_plus_four':
            dual_side_remaining[active_side] -= 1
        family_side_counts[(scene_type, active_side)] += 1
        active_counts[active_side] += 1
        schedule.append((scope, active_side))
    if any(remaining.values()) or any(dual_side_remaining.values()):
        raise VisualScenarioError(
            'blocker rail-scope allocation did not consume exact configured counts'
        )
    return schedule


def _build_blocker_scenario(
    scene_type: str,
    *,
    ordinal: int,
    type_index: int,
    seed: int,
    dataset_seed: int,
    capture: dict[str, Any],
    rail_scope: str,
    active_side: str,
    target_identity: str,
    relation_identities: tuple[str, ...],
    rail_presence_index: dict[str, int],
    covered_segments_by_side: dict[str, set[str]],
) -> dict[str, Any]:
    positions, relations, active_branch = _blocker_positions(
        scene_type,
        side=active_side,
        type_index=type_index,
        target_zone=POSITION_ZONES[(ordinal - 1) % len(POSITION_ZONES)],
        seed=seed,
        dataset_seed=dataset_seed,
    )
    if len(relation_identities) != len(positions) - 1:
        raise VisualScenarioError(
            f'{scene_type} expected {len(positions) - 1} relation identities, '
            f'found {len(relation_identities)}'
        )
    active_positions = {target_identity: positions[0]}
    remapped_relations = []
    for relation_identity, position, relation in zip(
        relation_identities,
        positions[1:],
        relations,
    ):
        active_positions[relation_identity] = position
        remapped = dict(relation)
        remapped['other_shuttle_id'] = relation_identity
        remapped_relations.append(remapped)
    covered_segments_by_side[active_side].update(
        str(position['segment'])
        for position in active_positions.values()
    )
    active_neutral_identities = tuple(
        identity
        for identity in identities_for_side(active_side)
        if identity not in active_positions
    )
    _place_neutral_shuttles(
        active_side,
        active_neutral_identities,
        active_positions,
        ordinal=ordinal,
        dataset_seed=dataset_seed,
        active_branch=active_branch,
        target_segment=str(positions[0]['segment']),
        covered_segments=covered_segments_by_side[active_side],
    )

    positions_by_side = {active_side: active_positions}
    inactive_side = 'left' if active_side == 'right' else 'right'
    if rail_scope == 'dual_four_plus_four':
        inactive_positions: dict[str, dict[str, Any]] = {}
        _place_neutral_shuttles(
            inactive_side,
            identities_for_side(inactive_side),
            inactive_positions,
            ordinal=ordinal,
            dataset_seed=dataset_seed,
            active_branch=None,
            target_segment=None,
            covered_segments=covered_segments_by_side[inactive_side],
        )
        positions_by_side[inactive_side] = inactive_positions
    else:
        positions_by_side[inactive_side] = {}

    shuttles_by_side = {}
    for side in ('left', 'right'):
        side_shuttles = []
        for identity in identities_for_side(side):
            if identity not in positions_by_side[side]:
                continue
            identity_index = int(identity[1:])
            side_shuttles.append({
                'id': identity,
                'start_slot': identity_index,
                'start_position': positions_by_side[side][identity],
                'loaded_state': (
                    'loaded'
                    if (rail_presence_index[side] + identity_index) % 2
                    else 'empty'
                ),
            })
        shuttles_by_side[side] = side_shuttles

    active_switches = dict(zip(SWITCH_NAMES, (active_branch,) * 4))
    inactive_pattern = 'all_exterior'
    inactive_switches = dict(zip(SWITCH_NAMES, ('exterior',) * 4))
    rails = {
        active_side: {
            'switch_pattern': f'all_{active_branch}',
            'switches': active_switches,
            'shuttles': shuttles_by_side[active_side],
        },
        inactive_side: {
            'switch_pattern': inactive_pattern,
            'switches': inactive_switches,
            'shuttles': shuttles_by_side[inactive_side],
        },
    }
    scene = {
        'rails': rails,
        'obstacles': [],
    }
    family_hash = _hash(_family_payload(scene_type, scene))
    scenario_id = f'visual_{ordinal:04d}_{scene_type}_{family_hash[:8]}'
    return {
        'schema_version': SCHEMA_VERSION,
        'scenario_id': scenario_id,
        'scenario_family': f'visual_family_{family_hash}',
        'scene_type': scene_type,
        'seed': seed,
        'scene': scene,
        'capture': {
            'mode': 'settled_static',
            'cameras': list(REQUIRED_CAMERAS),
            'frames': int(capture['frames_per_scenario']),
            'settle_seconds': float(capture['settle_seconds']),
            'frame_interval_seconds': float(capture['frame_interval_seconds']),
        },
        'setup': {
            'launch_package': 'mfja_3rd_floor_bringup',
            'launch_file': 'room_315_only.launch.py',
            'launch_arguments': _launch_arguments(rails),
            'switch_topic': '/mfja/conveyor/switch_cmd',
            'switch_command': _switch_command(rails),
        },
        'relation_probe': {
            'target_shuttle_id': target_identity,
            'side': active_side,
            'relations': remapped_relations,
            'relation_neutral_shuttle_ids': list(active_neutral_identities),
            'opposite_rail_neutral_shuttle_ids': (
                list(identities_for_side(inactive_side))
                if rail_scope == 'dual_four_plus_four'
                else []
            ),
            'model_input_exposure': 'excluded',
        },
        'rail_scope': rail_scope,
        'expected_label_coverage': {
            'shuttle_count': sum(len(rail['shuttles']) for rail in rails.values()),
            'fixed_schema_identity_count': len(AUTHORITATIVE_VISUAL_FLEET['schema_order']),
            'active_rail_shuttle_count': len(rails[active_side]['shuttles']),
            'simultaneous_four_plus_four': rail_scope == 'dual_four_plus_four',
            'loaded_count': sum(
                shuttle['loaded_state'] == 'loaded'
                for rail in rails.values()
                for shuttle in rail['shuttles']
            ),
            'empty_count': sum(
                shuttle['loaded_state'] == 'empty'
                for rail in rails.values()
                for shuttle in rail['shuttles']
            ),
            'switch_count': len(SWITCH_NAMES) * len(SIDES),
            'obstacle_count': 0,
            'continuous_position_count': sum(
                len(rail['shuttles']) for rail in rails.values()
            ),
        },
    }


def _generate_blocker_scenarios(
    config: dict[str, Any],
    *,
    count: int | None,
    seed: int | None,
) -> list[dict[str, Any]]:
    generator = config.get('generator')
    capture = config.get('capture')
    if not isinstance(generator, dict) or not isinstance(capture, dict):
        raise VisualScenarioError('generator and capture must be objects')
    scenario_count = _positive_int(
        count if count is not None else generator.get('scenario_count'),
        context='generator.scenario_count',
    )
    scenario_seed = int(seed if seed is not None else generator.get('seed', 315))
    normalized_capture = {
        'frames_per_scenario': _positive_int(
            capture.get('frames_per_scenario'),
            context='capture.frames_per_scenario',
        ),
        'settle_seconds': float(capture.get('settle_seconds', 1.5)),
        'frame_interval_seconds': float(capture.get('frame_interval_seconds', 0.25)),
    }
    counts = _named_scene_counts(
        scenario_count,
        normalized_blocker_scene_type_weights(config.get('scene_type_weights')),
        BLOCKER_SCENE_TYPES,
    )
    ordered_types = _interleaved_named_types(counts, BLOCKER_SCENE_TYPES)
    scope_counts = _rail_scope_counts(
        scenario_count,
        config.get('rail_scope_weights'),
    )
    scope_schedule = _blocker_scope_schedule(ordered_types, scope_counts)
    side_target_occurrences = Counter()
    planned_scenarios = []
    for scene_type, (rail_scope, active_side) in zip(
        ordered_types,
        scope_schedule,
    ):
        side_identities = identities_for_side(active_side)
        occurrence = side_target_occurrences[active_side]
        target_identity = side_identities[
            (
                occurrence
                + occurrence // (2 * len(side_identities))
            )
            % len(side_identities)
        ]
        side_target_occurrences[active_side] += 1
        planned_scenarios.append((
            scene_type,
            rail_scope,
            active_side,
            target_identity,
        ))
    side_type_indices = Counter()
    target_counts = Counter()
    rail_presence_counts = Counter()
    role_counts: dict[tuple[str, str, str], int] = {}
    covered_segments_by_side = {side: set() for side in SIDES}
    randomizer = random.Random(scenario_seed)
    seed_offsets = list(range(scenario_count * 4))
    randomizer.shuffle(seed_offsets)
    scenarios = []
    seen_families = set()
    for ordinal, (
        scene_type,
        rail_scope,
        active_side,
        target_identity,
    ) in enumerate(
        planned_scenarios,
        start=1,
    ):
        relation_roles = _relation_roles(scene_type)
        future_targets_by_role: dict[str, list[str]] = {}
        for (
            future_scene_type,
            _future_scope,
            future_side,
            future_target,
        ) in planned_scenarios[ordinal:]:
            if future_side != active_side:
                continue
            for future_role in _relation_roles(future_scene_type):
                future_targets_by_role.setdefault(future_role, []).append(
                    future_target
                )
        relation_identities = _select_relation_identities(
            active_side,
            target_identity,
            relation_roles,
            role_counts=role_counts,
            ordinal=ordinal,
            future_targets_by_role={
                role: tuple(targets)
                for role, targets in future_targets_by_role.items()
            },
        )
        side_type_index = side_type_indices[(scene_type, active_side)]
        present_sides = (
            ('left', 'right')
            if rail_scope == 'dual_four_plus_four'
            else (active_side,)
        )
        rail_presence_index = {
            side: int(rail_presence_counts[side])
            for side in ('left', 'right')
        }
        attempt = 0
        while True:
            scenario = _build_blocker_scenario(
                scene_type,
                ordinal=ordinal,
                type_index=(
                    side_type_index
                    + attempt * max(1, counts[scene_type])
                ),
                seed=scenario_seed + seed_offsets[ordinal - 1] + attempt,
                dataset_seed=scenario_seed,
                capture=normalized_capture,
                rail_scope=rail_scope,
                active_side=active_side,
                target_identity=target_identity,
                relation_identities=relation_identities,
                rail_presence_index=rail_presence_index,
                covered_segments_by_side={
                    side: set(covered_segments_by_side[side])
                    for side in SIDES
                },
            )
            if scenario['scenario_family'] not in seen_families:
                break
            attempt += 1
            if attempt > 1000:
                raise VisualScenarioError(
                    f'could not generate a unique {scene_type} scenario'
                )
        validate_scenario(scenario)
        seen_families.add(scenario['scenario_family'])
        scenarios.append(scenario)
        target_counts[(active_side, target_identity)] += 1
        side_type_indices[(scene_type, active_side)] += 1
        for role, identity in zip(relation_roles, relation_identities):
            key = (active_side, role, identity)
            role_counts[key] = role_counts.get(key, 0) + 1
        for side in present_sides:
            rail_presence_counts[side] += 1
            covered_segments_by_side[side].update(
                str(shuttle['start_position']['segment'])
                for shuttle in scenario['scene']['rails'][side]['shuttles']
            )
    return scenarios


def _interleaved_scene_types(counts: dict[str, int]) -> list[str]:
    remaining = dict(counts)
    result = []
    while sum(remaining.values()):
        ranked = sorted(
            (name for name in SCENE_TYPES if remaining[name]),
            key=lambda name: (
                remaining[name] / max(1, counts[name]),
                remaining[name],
                name,
            ),
            reverse=True,
        )
        selected = ranked[0]
        result.append(selected)
        remaining[selected] -= 1
    return result


def generate_scenarios(
    config: dict[str, Any],
    *,
    count: int | None = None,
    seed: int | None = None,
) -> list[dict[str, Any]]:
    profile = str(config.get('scenario_profile') or 'general_visual_state').strip()
    if profile == 'blocker_localization':
        return _generate_blocker_scenarios(config, count=count, seed=seed)
    if profile != 'general_visual_state':
        raise VisualScenarioError(f'unsupported scenario_profile: {profile!r}')
    generator = config.get('generator')
    capture = config.get('capture')
    if not isinstance(generator, dict):
        raise VisualScenarioError('generator must be an object')
    if not isinstance(capture, dict):
        raise VisualScenarioError('capture must be an object')
    scenario_count = _positive_int(
        count if count is not None else generator.get('scenario_count'),
        context='generator.scenario_count',
    )
    scenario_seed = int(seed if seed is not None else generator.get('seed', 315))
    frames = _positive_int(
        capture.get('frames_per_scenario'),
        context='capture.frames_per_scenario',
    )
    normalized_capture = {
        'frames_per_scenario': frames,
        'settle_seconds': float(capture.get('settle_seconds', 1.5)),
        'frame_interval_seconds': float(capture.get('frame_interval_seconds', 0.25)),
    }
    if normalized_capture['settle_seconds'] < 0.0:
        raise VisualScenarioError('capture.settle_seconds cannot be negative')
    if normalized_capture['frame_interval_seconds'] <= 0.0:
        raise VisualScenarioError('capture.frame_interval_seconds must be positive')

    counts = allocate_scene_counts(scenario_count, _weights(config))
    ordered_types = _interleaved_scene_types(counts)
    type_indices = Counter()
    scenarios = []
    seen_families = set()
    randomizer = random.Random(scenario_seed)
    seed_offsets = list(range(scenario_count * 4))
    randomizer.shuffle(seed_offsets)
    for ordinal, scene_type in enumerate(ordered_types, start=1):
        type_index = type_indices[scene_type]
        type_indices[scene_type] += 1
        attempt = 0
        while True:
            scenario = _build_scenario(
                scene_type,
                ordinal=ordinal,
                type_index=type_index + attempt * max(1, counts[scene_type]),
                seed=scenario_seed + seed_offsets[ordinal - 1] + attempt,
                capture=normalized_capture,
                duplicate_attempt=attempt,
            )
            if scenario['scenario_family'] not in seen_families:
                break
            attempt += 1
            if attempt > 5000:
                raise VisualScenarioError(
                    f'could not generate a unique {scene_type} scenario'
                )
        validate_scenario(scenario)
        seen_families.add(scenario['scenario_family'])
        scenarios.append(scenario)
    return scenarios


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


def scenario_physical_conflicts(scenario: dict[str, Any]) -> list[dict[str, Any]]:
    """Return world-space shuttle footprint conflicts for one scenario."""
    scene = scenario.get('scene')
    rails = scene.get('rails') if isinstance(scene, dict) else None
    if not isinstance(rails, dict):
        return []
    positions = []
    for side in SIDES:
        rail = rails.get(side)
        if not isinstance(rail, dict):
            continue
        for shuttle in rail.get('shuttles') or []:
            start_position = shuttle.get('start_position')
            if not isinstance(start_position, dict):
                continue
            positions.append(ShuttlePosition(
                shuttle_id=str(shuttle.get('id') or ''),
                side=side,
                segment=str(start_position.get('segment') or ''),
                s_ratio=float(start_position.get('s_ratio')),
            ))
    return shuttle_position_conflicts(positions)


def validate_scenario(
    scenario: dict[str, Any],
    *,
    check_physical_geometry: bool = True,
) -> None:
    if scenario.get('schema_version') != SCHEMA_VERSION:
        raise VisualScenarioError('scenario has an unsupported schema_version')
    leaked = sorted(_walk_keys(scenario) & LEGACY_KEYS)
    if leaked:
        raise VisualScenarioError(f'scenario contains legacy fields: {leaked}')
    if scenario.get('scene_type') not in ALL_SCENE_TYPES:
        raise VisualScenarioError('scenario has an unsupported scene_type')
    if not str(scenario.get('scenario_id') or '').strip():
        raise VisualScenarioError('scenario is missing scenario_id')
    if not str(scenario.get('scenario_family') or '').strip():
        raise VisualScenarioError('scenario is missing scenario_family')
    capture = scenario.get('capture')
    if not isinstance(capture, dict):
        raise VisualScenarioError('scenario.capture must be an object')
    if tuple(capture.get('cameras') or ()) != REQUIRED_CAMERAS:
        raise VisualScenarioError(
            f'scenario.capture.cameras must be {list(REQUIRED_CAMERAS)}'
        )
    scene = scenario.get('scene')
    if not isinstance(scene, dict) or not isinstance(scene.get('rails'), dict):
        raise VisualScenarioError('scenario.scene.rails must be an object')
    if scene.get('obstacles') != []:
        raise VisualScenarioError(
            'pilot scenarios require obstacles=[] until bbox labelling is available'
        )
    seen_ids = set()
    for side in SIDES:
        rail = scene['rails'].get(side)
        if not isinstance(rail, dict):
            raise VisualScenarioError(f'scenario is missing scene.rails.{side}')
        switches = rail.get('switches')
        if not isinstance(switches, dict) or set(switches) != set(SWITCH_NAMES):
            raise VisualScenarioError(f'{side} switches must contain A1-A4')
        invalid_states = sorted(
            state
            for state in switches.values()
            if state not in {'interior', 'exterior'}
        )
        if invalid_states:
            raise VisualScenarioError(f'{side} has invalid switch states: {invalid_states}')
        shuttles = rail.get('shuttles')
        if not isinstance(shuttles, list) or len(shuttles) > 4:
            raise VisualScenarioError(f'{side} shuttles must be a list of at most four')
        slots = []
        continuous_positions = []
        segment_lengths = _segment_lengths(side)
        allowed_identity_order = identities_for_side(side)
        actual_identity_order = tuple(
            str(shuttle.get('id') or '')
            for shuttle in shuttles
        )
        expected_identity_order = tuple(
            identity
            for identity in allowed_identity_order
            if identity in actual_identity_order
        )
        if actual_identity_order != expected_identity_order:
            raise VisualScenarioError(
                f'{side} shuttle identities must be a unique ordered subset '
                f'of {list(allowed_identity_order)}; got '
                f'{list(actual_identity_order)}'
            )
        for shuttle in shuttles:
            if shuttle.get('id') not in allowed_identity_order:
                raise VisualScenarioError(
                    f'{side} has unsupported shuttle identity '
                    f'{shuttle.get("id")!r}'
                )
            if shuttle['id'] in seen_ids:
                raise VisualScenarioError(f'duplicate shuttle id: {shuttle["id"]}')
            seen_ids.add(shuttle['id'])
            slot = int(shuttle.get('start_slot', 0))
            if slot not in START_SLOTS:
                raise VisualScenarioError(f'{shuttle["id"]} has invalid start_slot {slot}')
            slots.append(slot)
            if shuttle.get('loaded_state') not in {'loaded', 'empty'}:
                raise VisualScenarioError(
                    f'{shuttle["id"]} loaded_state must be loaded or empty'
                )
            start_position = shuttle.get('start_position')
            if start_position is not None:
                if not isinstance(start_position, dict):
                    raise VisualScenarioError(
                        f'{shuttle["id"]} start_position must be an object'
                    )
                segment = str(start_position.get('segment') or '').strip().upper()
                if not segment:
                    raise VisualScenarioError(
                        f'{shuttle["id"]} start_position is missing segment'
                    )
                if segment not in segment_lengths:
                    raise VisualScenarioError(
                        f'{shuttle["id"]} start_position.segment is not valid '
                        f'for the {side} rail: {segment}'
                    )
                try:
                    s_ratio = float(start_position.get('s_ratio'))
                except (TypeError, ValueError) as exc:
                    raise VisualScenarioError(
                        f'{shuttle["id"]} start_position.s_ratio must be numeric'
                    ) from exc
                if not 0.0 <= s_ratio <= 1.0:
                    raise VisualScenarioError(
                        f'{shuttle["id"]} start_position.s_ratio must be in [0, 1]'
                    )
                position_zone = str(
                    start_position.get('position_zone') or ''
                ).strip()
                if (
                    scenario['scene_type'] in BLOCKER_SCENE_TYPES
                    and position_zone not in set(POSITION_ZONES) | RELATION_POSITION_ZONES
                ):
                    raise VisualScenarioError(
                        f'{shuttle["id"]} has invalid position_zone {position_zone!r}'
                    )
                continuous_positions.append((shuttle['id'], segment, s_ratio))
        if len(slots) != len(set(slots)):
            raise VisualScenarioError(f'{side} contains duplicate start slots')
        for first, second in itertools.combinations(continuous_positions, 2):
            if first[1] != second[1]:
                continue
            separation = abs(first[2] - second[2])
            minimum_ratio = max(
                MIN_SAME_SEGMENT_START_SEPARATION_RATIO,
                MIN_SAME_SEGMENT_START_SEPARATION_M / segment_lengths[first[1]],
            )
            if separation < minimum_ratio:
                raise VisualScenarioError(
                    f'{first[0]} and {second[0]} are only {separation:.6f} apart '
                    f'on {first[1]}; require at least '
                    f'{minimum_ratio:.6f} normalized '
                    'rail distance to avoid overlapping shuttle geometry'
                )
    if check_physical_geometry:
        physical_conflicts = scenario_physical_conflicts(scenario)
        if physical_conflicts:
            first = physical_conflicts[0]
            raise VisualScenarioError(
                'world-space shuttle collision: '
                f'{first["first_id"]} on {first["first_segment"]} and '
                f'{first["second_id"]} on {first["second_segment"]} have only '
                f'{first["center_distance_m"]:.6f} m between collision centers'
            )
    arbitrary_presence = (
        scenario.get('presence_profile')
        == ARBITRARY_IDENTITY_PRESENCE_PROFILE
    )
    if arbitrary_presence:
        active_by_side = {
            side: [
                shuttle['id']
                for shuttle in scenario['scene']['rails'][side]['shuttles']
            ]
            for side in SIDES
        }
        if not any(active_by_side.values()):
            raise VisualScenarioError(
                'the all-empty arbitrary identity configuration is invalid'
            )
        for side in SIDES:
            field = f'{side}_active_identities'
            if scenario.get(field) != active_by_side[side]:
                raise VisualScenarioError(
                    f'{field} must exactly match the scene identities'
                )
        launch_arguments = (
            (scenario.get('setup') or {}).get('launch_arguments') or {}
        )
        if launch_arguments.get('room315_identity_selection_mode') != 'explicit':
            raise VisualScenarioError(
                'arbitrary identity scenarios require explicit launch mode'
            )
        for side in SIDES:
            encoded = ','.join(active_by_side[side])
            if launch_arguments.get(
                f'room315_{side}_active_identities'
            ) != encoded:
                raise VisualScenarioError(
                    f'launch {side}_active_identities does not exactly match '
                    'the scenario subset'
                )
            if int(launch_arguments.get(
                f'room315_{side}_shuttle_count', -1
            )) != len(active_by_side[side]):
                raise VisualScenarioError(
                    f'launch {side}_shuttle_count does not match the explicit '
                    'identity subset'
                )
    if scenario['scene_type'] in BLOCKER_SCENE_TYPES:
        scope = str(scenario.get('rail_scope') or '')
        if (
            scope not in BLOCKER_RAIL_SCOPES
            and not (
                arbitrary_presence
                and scope == ARBITRARY_IDENTITY_PRESENCE_PROFILE
            )
        ):
            raise VisualScenarioError(
                f'blocker scenario has invalid rail_scope: {scope!r}'
            )
        probe_side = str((scenario.get('relation_probe') or {}).get('side') or '')
        if probe_side not in SIDES:
            raise VisualScenarioError('blocker scenario has invalid relation rail')
        counts_by_side = {
            side: len(scenario['scene']['rails'][side]['shuttles'])
            for side in SIDES
        }
        if arbitrary_presence:
            target_rail_count = counts_by_side[probe_side]
            if target_rail_count < 2:
                raise VisualScenarioError(
                    'relation scenarios require at least two active identities '
                    'on the target rail'
                )
            if (
                scenario['scene_type'] == 'multi_blocker'
                and target_rail_count < 3
            ):
                raise VisualScenarioError(
                    'multi_blocker requires at least three active identities '
                    'on the target rail'
                )
        elif counts_by_side[probe_side] != 4:
            raise VisualScenarioError(
                'every blocker scenario must launch all four identities on '
                'the active relation rail'
            )
        if (
            not arbitrary_presence
            and
            scope == 'dual_four_plus_four'
            and any(count != 4 for count in counts_by_side.values())
        ):
            raise VisualScenarioError(
                f'dual_four_plus_four requires 4+4 shuttles: {counts_by_side}'
            )
        if (
            not arbitrary_presence
            and
            scope != 'dual_four_plus_four'
            and sorted(counts_by_side.values()) != [0, 4]
        ):
            raise VisualScenarioError(
                f'single-rail four-shuttle scope requires 4+0 shuttles: '
                f'{counts_by_side}'
            )
        _validate_relation_probe(scenario)


def _validate_relation_probe(scenario: dict[str, Any]) -> None:
    probe = scenario.get('relation_probe')
    if not isinstance(probe, dict):
        raise VisualScenarioError('blocker scenario is missing relation_probe')
    side = str(probe.get('side') or '').strip().lower()
    if side not in SIDES:
        raise VisualScenarioError('relation_probe.side must be right or left')
    shuttles = scenario['scene']['rails'][side]['shuttles']
    by_id = {
        shuttle['id']: shuttle['start_position']
        for shuttle in shuttles
    }
    target_id = str(probe.get('target_shuttle_id') or '').strip()
    if target_id not in by_id:
        raise VisualScenarioError('relation_probe target shuttle is not in the scene')
    relations = probe.get('relations')
    if not isinstance(relations, list) or not relations:
        raise VisualScenarioError('relation_probe.relations must be a non-empty list')
    related_ids = {
        str(relation.get('other_shuttle_id') or '')
        for relation in relations
        if isinstance(relation, dict)
    }
    neutral_ids = set(probe.get('relation_neutral_shuttle_ids') or [])
    expected_neutral = set(by_id) - {target_id} - related_ids
    if neutral_ids != expected_neutral:
        raise VisualScenarioError(
            'relation-neutral identity set does not match active-rail identities; '
            f'expected={sorted(expected_neutral)}, actual={sorted(neutral_ids)}'
        )
    target = by_id[target_id]
    branch = next(iter(scenario['scene']['rails'][side]['switches'].values()))
    cycle = _branch_cycle(side, branch)
    for relation in relations:
        if not isinstance(relation, dict):
            raise VisualScenarioError('relation_probe relation must be an object')
        other_id = str(relation.get('other_shuttle_id') or '').strip()
        if other_id not in by_id:
            raise VisualScenarioError(
                f'relation_probe shuttle is not in the scene: {other_id}'
            )
        other = by_id[other_id]
        kind = str(relation.get('relation') or '').strip()
        same_segment = target['segment'] == other['segment']
        target_ratio = float(target['s_ratio'])
        other_ratio = float(other['s_ratio'])
        if kind == 'ahead_blocker':
            valid = same_segment and other_ratio > target_ratio
        elif kind == 'behind_non_blocker':
            valid = same_segment and other_ratio < target_ratio
        elif kind == 'adjacent_branch_non_blocker':
            valid = (
                target['segment'][:-1] == other['segment'][:-1]
                and {target['segment'][-1], other['segment'][-1]} == {'E', 'I'}
            )
        elif kind == 'intermediate_blocker':
            valid = (
                target['segment'] in cycle
                and other['segment'] in cycle
                and other['segment'] != target['segment']
                and (
                    cycle.index(other['segment']) - cycle.index(target['segment'])
                ) % len(cycle) in {1, 2, 3}
            )
        else:
            raise VisualScenarioError(f'unsupported relation kind: {kind!r}')
        if not valid:
            raise VisualScenarioError(
                f'{target_id}->{other_id} does not preserve relation {kind}'
            )


def validate_scenarios(
    scenarios: list[dict[str, Any]],
    *,
    check_physical_geometry: bool = True,
) -> None:
    if not scenarios:
        raise VisualScenarioError('scenario manifest is empty')
    ids = []
    families = []
    for scenario in scenarios:
        validate_scenario(
            scenario,
            check_physical_geometry=check_physical_geometry,
        )
        ids.append(scenario['scenario_id'])
        families.append(scenario['scenario_family'])
    if len(ids) != len(set(ids)):
        raise VisualScenarioError('scenario manifest contains duplicate scenario_id values')
    if len(families) != len(set(families)):
        raise VisualScenarioError('scenario manifest contains duplicate scenario families')


def scenario_summary(
    scenarios: list[dict[str, Any]],
    *,
    config_path: Path,
) -> dict[str, Any]:
    type_counts = Counter()
    side_active = Counter()
    shuttle_counts = Counter()
    payload_states = Counter()
    switch_states = Counter()
    position_zones = Counter()
    segment_instances = Counter()
    target_positions = set()
    rail_scopes = Counter()
    identity_roles = Counter()
    for scenario in scenarios:
        type_counts[scenario['scene_type']] += 1
        if scenario.get('rail_scope'):
            rail_scopes[str(scenario['rail_scope'])] += 1
        total_shuttles = 0
        for side in SIDES:
            rail = scenario['scene']['rails'][side]
            if rail['shuttles']:
                side_active[side] += 1
            total_shuttles += len(rail['shuttles'])
            payload_states.update(
                shuttle['loaded_state'] for shuttle in rail['shuttles']
            )
            for shuttle in rail['shuttles']:
                start_position = shuttle.get('start_position')
                if not isinstance(start_position, dict):
                    continue
                segment_instances[f'{side}:{start_position["segment"]}'] += 1
                position_zone = str(start_position.get('position_zone') or '')
                if position_zone in POSITION_ZONES:
                    position_zones[position_zone] += 1
            switch_states.update(rail['switches'].values())
        probe = scenario.get('relation_probe')
        if isinstance(probe, dict):
            side = str(probe['side'])
            target_id = str(probe['target_shuttle_id'])
            target = next(
                shuttle
                for shuttle in scenario['scene']['rails'][side]['shuttles']
                if shuttle['id'] == target_id
            )
            target_position = target['start_position']
            target_positions.add((
                side,
                str(target_position['segment']),
                float(target_position['s_ratio']),
            ))
            identity_roles[f'{target_id}:target'] += 1
            for relation in probe.get('relations') or []:
                relation_kind = str(relation.get('relation') or '')
                role = (
                    'non_blocker'
                    if 'non_blocker' in relation_kind
                    else 'blocker'
                )
                identity_roles[
                    f'{relation["other_shuttle_id"]}:{role}'
                ] += 1
            for identity in probe.get('relation_neutral_shuttle_ids') or []:
                identity_roles[f'{identity}:relation_neutral'] += 1
            for identity in probe.get('opposite_rail_neutral_shuttle_ids') or []:
                identity_roles[f'{identity}:relation_neutral'] += 1
        shuttle_counts[str(total_shuttles)] += 1
    manifest_payload = '\n'.join(_canonical_json(row) for row in scenarios) + '\n'
    return {
        'tool': 'room_315_visual_scenario_generator',
        'schema_version': SCHEMA_VERSION,
        'configuration': str(config_path.expanduser().resolve()),
        'scenarios': len(scenarios),
        'scenario_families': len({row['scenario_family'] for row in scenarios}),
        'planned_image_pairs': sum(row['capture']['frames'] for row in scenarios),
        'required_cameras': list(REQUIRED_CAMERAS),
        'scene_type_counts': dict(sorted(type_counts.items())),
        'active_side_counts': dict(sorted(side_active.items())),
        'rail_scope_counts': dict(sorted(rail_scopes.items())),
        'identity_role_counts': dict(sorted(identity_roles.items())),
        'total_shuttle_count_distribution': dict(sorted(shuttle_counts.items())),
        'payload_state_instances': dict(sorted(payload_states.items())),
        'switch_state_instances': dict(sorted(switch_states.items())),
        'position_zone_instances': dict(sorted(position_zones.items())),
        'segment_instances': dict(sorted(segment_instances.items())),
        'unique_target_positions': len(target_positions),
        'obstacle_scenarios': 0,
        'manifest_sha256': hashlib.sha256(manifest_payload.encode('utf-8')).hexdigest(),
        'quality_gate': {
            'unique_scenario_ids': True,
            'unique_scenario_families': True,
            'legacy_fields_present': [],
            'model_input_fields_in_manifest': [],
            'camera_pair_required': True,
            'obstacles_deferred_until_bbox_labels': True,
            'all_public_segments_covered': all(
                f'{side}:{segment}' in segment_instances
                for side in SIDES
                for segment in valid_public_segments(side)
            ) if segment_instances else None,
            'all_position_zones_covered': (
                set(position_zones) == set(POSITION_ZONES)
                if position_zones
                else None
            ),
        },
    }


def write_scenario_plan(
    output_dir: Path,
    scenarios: list[dict[str, Any]],
    summary: dict[str, Any],
    *,
    force: bool = False,
) -> tuple[Path, Path]:
    output_dir = output_dir.expanduser().resolve()
    manifest_path = output_dir / 'scenario_manifest.jsonl'
    summary_path = output_dir / 'scenario_summary.json'
    existing = [path for path in (manifest_path, summary_path) if path.exists()]
    if existing and not force:
        raise FileExistsError(
            f'refusing to overwrite existing scenario plan files: {existing}'
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    with manifest_path.open('w', encoding='utf-8') as stream:
        for scenario in scenarios:
            stream.write(_canonical_json(scenario) + '\n')
    summary_path.write_text(_pretty_json(summary) + '\n', encoding='utf-8')
    return manifest_path, summary_path


def _read_manifest(path: Path) -> list[dict[str, Any]]:
    scenarios = []
    with path.expanduser().open('r', encoding='utf-8') as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            if not raw_line.strip():
                continue
            try:
                parsed = json.loads(raw_line)
            except json.JSONDecodeError as exc:
                raise VisualScenarioError(
                    f'{path}:{line_number}: invalid JSON: {exc}'
                ) from exc
            if not isinstance(parsed, dict):
                raise VisualScenarioError(f'{path}:{line_number}: row must be an object')
            scenarios.append(parsed)
    validate_scenarios(scenarios)
    return scenarios


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Generate new Room 315 image-to-visual-state Gazebo scenarios '
            'without task, PDDL, language, command, or action fields.'
        )
    )
    parser.add_argument('--config', type=Path)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--count', type=int)
    parser.add_argument('--seed', type=int)
    parser.add_argument('--force', action='store_true')
    parser.add_argument(
        '--validate-manifest',
        type=Path,
        help='Validate an existing scenario_manifest.jsonl and exit.',
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.validate_manifest:
        scenarios = _read_manifest(args.validate_manifest)
        print(f'valid scenarios: {len(scenarios)}')
        return 0
    if args.config is None or args.output_dir is None:
        raise VisualScenarioError('--config and --output-dir are required for generation')
    config = _load_config(args.config)
    scenarios = generate_scenarios(config, count=args.count, seed=args.seed)
    validate_scenarios(scenarios)
    summary = scenario_summary(scenarios, config_path=args.config)
    manifest_path, summary_path = write_scenario_plan(
        args.output_dir,
        scenarios,
        summary,
        force=args.force,
    )
    print(f'wrote {len(scenarios)} scenarios: {manifest_path}')
    print(f'wrote coverage summary: {summary_path}')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (FileExistsError, OSError, VisualScenarioError, yaml.YAMLError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
