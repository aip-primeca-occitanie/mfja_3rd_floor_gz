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
from pathlib import Path
from typing import Any

import yaml


SCHEMA_VERSION = 'room315.visual_capture_scenario.v1'
REQUIRED_CAMERAS = ('left_rail_rgb', 'right_rail_rgb')
SIDES = ('right', 'left')
SWITCH_NAMES = ('A1', 'A2', 'A3', 'A4')
START_SLOTS = (1, 2, 3, 4)
SCENE_TYPES = ('empty', 'single', 'multi_same_rail', 'dual_rail')
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
        'enable_room315_vla': True,
        'enable_room315_vla_dataset_recorder': False,
        'enable_room315_vla_obstacles': False,
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


def validate_scenario(scenario: dict[str, Any]) -> None:
    if scenario.get('schema_version') != SCHEMA_VERSION:
        raise VisualScenarioError('scenario has an unsupported schema_version')
    leaked = sorted(_walk_keys(scenario) & LEGACY_KEYS)
    if leaked:
        raise VisualScenarioError(f'scenario contains legacy fields: {leaked}')
    if scenario.get('scene_type') not in SCENE_TYPES:
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
        for expected_index, shuttle in enumerate(shuttles, start=1):
            expected_id = _shuttle_id(side, expected_index)
            if shuttle.get('id') != expected_id:
                raise VisualScenarioError(
                    f'{side} shuttle identity order must start at {expected_id}'
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
        if len(slots) != len(set(slots)):
            raise VisualScenarioError(f'{side} contains duplicate start slots')


def validate_scenarios(scenarios: list[dict[str, Any]]) -> None:
    if not scenarios:
        raise VisualScenarioError('scenario manifest is empty')
    ids = []
    families = []
    for scenario in scenarios:
        validate_scenario(scenario)
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
    for scenario in scenarios:
        type_counts[scenario['scene_type']] += 1
        total_shuttles = 0
        for side in SIDES:
            rail = scenario['scene']['rails'][side]
            if rail['shuttles']:
                side_active[side] += 1
            total_shuttles += len(rail['shuttles'])
            payload_states.update(
                shuttle['loaded_state'] for shuttle in rail['shuttles']
            )
            switch_states.update(rail['switches'].values())
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
        'total_shuttle_count_distribution': dict(sorted(shuttle_counts.items())),
        'payload_state_instances': dict(sorted(payload_states.items())),
        'switch_state_instances': dict(sorted(switch_states.items())),
        'obstacle_scenarios': 0,
        'manifest_sha256': hashlib.sha256(manifest_payload.encode('utf-8')).hexdigest(),
        'quality_gate': {
            'unique_scenario_ids': True,
            'unique_scenario_families': True,
            'legacy_fields_present': [],
            'model_input_fields_in_manifest': [],
            'camera_pair_required': True,
            'obstacles_deferred_until_bbox_labels': True,
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
