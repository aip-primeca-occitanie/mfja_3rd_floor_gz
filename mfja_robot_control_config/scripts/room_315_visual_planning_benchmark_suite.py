#!/usr/bin/env python3
"""Seeded Room 315 benchmark case and method-comparison utilities.

The case generator writes YAML manifests only. It never writes datasets,
checkpoints, or model outputs, and it keeps the existing 160 speed-sweep cases
as an explicit regression subset inside each generated manifest.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_REGRESSION_CASE_CONFIG = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'config'
    / 'room_315_payload_cases'
    / 'payload_training_cases_expanded_160_speed_sweep.yaml'
)

REGRESSION_SUBSET_ID = 'payload_training_cases_expanded_160_speed_sweep'
BALANCED_EXTENSION_ID = 'seeded_balanced_extension'
MIN_EXTENSION_CASES = 100
MAX_EXTENSION_CASES = 1000
DEFAULT_EXTENSION_CASES = 320
DEFAULT_SEED = 315

REQUIRED_BENCHMARK_FAMILIES = (
    'four_plus_four_fleet',
    'loaded_selection',
    'empty_selection',
    'blocker_clearance',
    'occupied_target',
    'unknown_position',
    'sensor_dropout',
    'obstacle',
    'inspection',
    'simultaneous_requests',
)

METHODS = (
    'oracle_plansys2',
    'frozen_visual_plansys2',
    'lora_visual_plansys2',
)

METHOD_ALIASES = {
    'oracle': 'oracle_plansys2',
    'oracle+plansys2': 'oracle_plansys2',
    'oracle_plansys2': 'oracle_plansys2',
    'frozen visual+plansys2': 'frozen_visual_plansys2',
    'frozen_visual+plansys2': 'frozen_visual_plansys2',
    'frozen_visual_plansys2': 'frozen_visual_plansys2',
    'frozen_backbone_plansys2': 'frozen_visual_plansys2',
    'lora visual+plansys2': 'lora_visual_plansys2',
    'lora_visual+plansys2': 'lora_visual_plansys2',
    'lora_visual_plansys2': 'lora_visual_plansys2',
}

COMPARISON_METRIC_ALIASES = {
    'success_rate': ('success_rate', 'task_success', 'success', 'task_success_rate'),
    'false_success_rate': ('false_success_rate', 'false_success', 'unsafe_success_rate'),
    'safety_violation_rate': (
        'safety_violation_rate',
        'unsafe_rate',
        'safety_failure_rate',
        'collision_rate',
    ),
    'supervisor_rejection_rate': (
        'supervisor_rejection_rate',
        'rejected_action_rate',
        'rejection_rate',
    ),
    'mean_replans': ('mean_replans', 'replans', 'replan_count', 'avg_replans'),
    'mean_route_length': ('mean_route_length', 'route_length', 'command_count'),
    'mean_completion_time_s': (
        'mean_completion_time_s',
        'completion_time_s',
        'completion_time',
        'total_cycle_time_s',
    ),
    'latency_p50_s': ('latency_p50_s', 'p50_latency_s', 'p50_inference_latency_s'),
    'latency_p95_s': ('latency_p95_s', 'p95_latency_s', 'p95_inference_latency_s'),
}
COMPARISON_CSV_FIELDS = (
    'method',
    'status',
    'result_scope',
    'success_rate',
    'false_success_rate',
    'safety_violation_rate',
    'supervisor_rejection_rate',
    'mean_replans',
    'mean_route_length',
    'mean_completion_time_s',
    'latency_p50_s',
    'latency_p95_s',
    'real_image_perception_claim',
    'gazebo_planning_result',
    'missing_metrics',
)


def _pretty_json(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True)


def _repo_relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path.expanduser())


def _load_yaml_mapping(path: Path) -> dict[str, Any]:
    loaded = yaml.safe_load(path.expanduser().read_text(encoding='utf-8')) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f'{path} must contain a YAML mapping')
    return loaded


def _slot_station(side: str, slot: str) -> str:
    slot_text = str(slot)
    if side == 'right':
        return 'yaskawa' if slot_text in {'1', '2'} else 'staubli'
    return 'yaskawa' if slot_text in {'1', '2'} else 'kuka'


def _side_letter(side: str) -> str:
    return 'R' if side == 'right' else 'L'


def _shuttle(side: str, index: int) -> str:
    return f'{side}_shuttle_{index}'


def _shuttle_sort_key(shuttle: str) -> tuple[str, int]:
    try:
        return str(shuttle).rsplit('_', 1)[0], int(str(shuttle).rsplit('_', 1)[-1])
    except ValueError:
        return str(shuttle), 0


def _ensure_unique_start_slots(
    start_slots: dict[str, str],
    *,
    preserve: tuple[str, ...] = (),
) -> None:
    valid_slots = {'1', '2', '3', '4'}
    used: set[str] = set()
    deferred: list[str] = []
    for shuttle_id in preserve:
        slot = str(start_slots.get(shuttle_id) or '')
        if slot in valid_slots and slot not in used:
            used.add(slot)
        else:
            deferred.append(shuttle_id)
    for shuttle_id in sorted(start_slots, key=_shuttle_sort_key):
        if shuttle_id in preserve:
            continue
        slot = str(start_slots.get(shuttle_id) or '')
        if slot in valid_slots and slot not in used:
            used.add(slot)
        else:
            deferred.append(shuttle_id)
    missing = [slot for slot in ('1', '2', '3', '4') if slot not in used]
    for shuttle_id, slot in zip(deferred, missing):
        start_slots[shuttle_id] = slot


def _launch_loaded_labels(side: str, loaded_shuttles: list[str]) -> str:
    prefix = _side_letter(side)
    labels = []
    for shuttle in loaded_shuttles:
        try:
            labels.append(f'{prefix}{int(str(shuttle).rsplit("_", 1)[-1])}')
        except ValueError:
            continue
    return ','.join(labels)


def _case_family_from_regression_case(case: dict[str, Any]) -> str:
    if case.get('clearance_steps'):
        return 'four_plus_four_fleet' if len(case.get('clearance_steps') or []) > 1 else 'blocker_clearance'
    if case.get('blocker_shuttle'):
        return 'blocker_clearance'
    starts = case.get('start_slots_by_shuttle') if isinstance(case.get('start_slots_by_shuttle'), dict) else {}
    if len(starts) >= 4:
        return 'four_plus_four_fleet'
    return 'loaded_selection'


def _with_regression_metadata(
    case: dict[str, Any],
    *,
    source_config: Path,
    ordinal: int,
) -> dict[str, Any]:
    item = deepcopy(case)
    item['benchmark_subset'] = 'regression_160'
    item['benchmark_family'] = _case_family_from_regression_case(item)
    item.setdefault('benchmark_conditions', {})
    item['benchmark_conditions'] = {
        **dict(item.get('benchmark_conditions') or {}),
        'source': 'gazebo_planning_regression_case',
        'source_config': _repo_relative(source_config),
        'regression_subset_id': REGRESSION_SUBSET_ID,
        'regression_subset_ordinal': ordinal,
    }
    return item


def _base_transport_case(
    *,
    case_id: str,
    title: str,
    family: str,
    side: str,
    target_slot: str,
    selected_index: int,
    start_slots: dict[str, str],
    loaded_shuttles: list[str],
    payload_condition: str,
    speed: float,
    selection_policy: str = 'nearest_loaded_to_target_slot_then_lowest_id',
    explicit: bool = False,
    benchmark_conditions: dict[str, Any] | None = None,
) -> dict[str, Any]:
    selected = _shuttle(side, selected_index)
    selected_start = start_slots[selected]
    case: dict[str, Any] = {
        'case_id': case_id,
        'title': title,
        'side': side,
        'speed': round(float(speed), 3),
        'target_slot': str(target_slot),
        'payload_condition': payload_condition,
        'selection_policy': '' if explicit else selection_policy,
        'loaded_shuttles': list(loaded_shuttles),
        'start_slots_by_shuttle': dict(start_slots),
        'launch': {
            f'{side}_shuttle_count': len(start_slots),
            f'{side}_start_slots': ','.join(start_slots[_shuttle(side, i)] for i in range(1, len(start_slots) + 1)),
            f'{side}_loaded_shuttles': _launch_loaded_labels(side, loaded_shuttles),
        },
        'expected': {
            'selected_shuttle': selected,
            'selected_target_slot': str(target_slot),
        },
        'base_case_id': case_id.rsplit('_', 1)[0],
        'benchmark_subset': BALANCED_EXTENSION_ID,
        'benchmark_family': family,
        'benchmark_conditions': {
            'result_scope': 'gazebo_planning',
            'real_image_perception_claim': False,
            **dict(benchmark_conditions or {}),
        },
        'augmentation': {
            'type': 'seeded_balanced_benchmark',
            'balanced_family': family,
        },
    }
    if explicit:
        case.update({
            'shuttle': selected,
            'source': _slot_station(side, selected_start),
            'target': _slot_station(side, target_slot),
        })
    return case


def _case_variant(
    *,
    family: str,
    seed: int,
    global_index: int,
    family_index: int,
    rng: random.Random,
) -> dict[str, Any]:
    side = 'right' if (global_index + seed) % 2 == 0 else 'left'
    slots = ['1', '2', '3', '4']
    rotated = slots[family_index % len(slots):] + slots[:family_index % len(slots)]
    speed = [0.06, 0.08, 0.1, 0.12][global_index % 4]
    target_slot = rotated[2]
    selected_index = (family_index % 4) + 1
    selected_start = rotated[0]
    if selected_start == target_slot:
        selected_start = rotated[1]
    start_slots = {
        _shuttle(side, index): rotated[index - 1]
        for index in range(1, 5)
    }
    selected_shuttle = _shuttle(side, selected_index)
    original_selected_slot = start_slots[selected_shuttle]
    for shuttle_id, slot in list(start_slots.items()):
        if shuttle_id != selected_shuttle and slot == selected_start:
            start_slots[shuttle_id] = original_selected_slot
    start_slots[selected_shuttle] = selected_start
    _ensure_unique_start_slots(start_slots, preserve=(selected_shuttle,))
    loaded_shuttles = [_shuttle(side, selected_index)]
    case_id = f'seeded_{family}_{seed}_{global_index:04d}'
    title = f'Seeded {family.replace("_", " ")} case {family_index + 1}'

    if family == 'loaded_selection':
        other_loaded = next(
            _shuttle(side, index)
            for index in range(1, 5)
            if index != selected_index and start_slots[_shuttle(side, index)] != target_slot
        )
        loaded_shuttles = sorted({loaded_shuttles[0], other_loaded})
        return _base_transport_case(
            case_id=case_id,
            title=title,
            family=family,
            side=side,
            target_slot=target_slot,
            selected_index=selected_index,
            start_slots=start_slots,
            loaded_shuttles=loaded_shuttles,
            payload_condition='loaded',
            speed=speed,
            benchmark_conditions={'selection': 'loaded_or_nearest_loaded'},
        )

    if family == 'empty_selection':
        loaded_shuttles = [
            _shuttle(side, index)
            for index in range(1, 5)
            if index != selected_index and (index + family_index) % 2 == 0
        ]
        return _base_transport_case(
            case_id=case_id,
            title=title,
            family=family,
            side=side,
            target_slot=target_slot,
            selected_index=selected_index,
            start_slots=start_slots,
            loaded_shuttles=loaded_shuttles,
            payload_condition='empty',
            speed=speed,
            explicit=True,
            benchmark_conditions={'selection': 'explicit_empty_shuttle'},
        )

    if family in {'blocker_clearance', 'occupied_target'}:
        blocker_index = selected_index % 4 + 1
        blocker = _shuttle(side, blocker_index)
        clear_slot = next(slot for slot in slots if slot not in {target_slot, selected_start})
        for shuttle_id, slot in list(start_slots.items()):
            if shuttle_id != blocker and slot == target_slot:
                start_slots[shuttle_id] = clear_slot
        start_slots[blocker] = str(target_slot)
        _ensure_unique_start_slots(start_slots, preserve=(_shuttle(side, selected_index), blocker))
        case = _base_transport_case(
            case_id=case_id,
            title=title,
            family=family,
            side=side,
            target_slot=target_slot,
            selected_index=selected_index,
            start_slots=start_slots,
            loaded_shuttles=loaded_shuttles,
            payload_condition='loaded',
            speed=speed,
            benchmark_conditions={
                'target_initially_occupied': True,
                'recovery_expected': 'clear_or_fail_closed',
            },
        )
        case.update({
            'blocker_shuttle': blocker,
            'blocker_start_slot': str(target_slot),
            'blocker_clear_slot': clear_slot,
            'blocker_clear_sensor': f'A{clear_slot}_STOPPER_SENSOR',
            'blocker_clear_stopper': f'A{clear_slot}',
            'clearance_strategy': (
                'occupied_target_requires_recovery'
                if family == 'occupied_target'
                else 'clear_blocker_to_free_slot_then_move_loaded'
            ),
        })
        case['expected'].update({
            'blocker_clear_slot': clear_slot,
            'selected_shuttle': _shuttle(side, selected_index),
        })
        return case

    if family == 'four_plus_four_fleet':
        case = _base_transport_case(
            case_id=case_id,
            title=title,
            family=family,
            side=side,
            target_slot=target_slot,
            selected_index=selected_index,
            start_slots=start_slots,
            loaded_shuttles=loaded_shuttles,
            payload_condition='loaded',
            speed=speed,
            benchmark_conditions={
                'right_shuttle_count': 4,
                'left_shuttle_count': 4,
                'fleet_shape': '4+4',
            },
        )
        blocker_a = _shuttle(side, selected_index % 4 + 1)
        blocker_b = _shuttle(side, (selected_index + 1) % 4 + 1)
        case['clearance_strategy'] = 'multi_blocker_clear_then_move_loaded'
        case['clearance_steps'] = [
            {
                'shuttle': blocker_a,
                'start_slot': start_slots[blocker_a],
                'clear_target': 'interior_loop',
                'clear_sensor': 'DA3IR' if side == 'right' else 'DA3IL',
                'clear_stopper': 'A3',
            },
            {
                'shuttle': blocker_b,
                'start_slot': start_slots[blocker_b],
                'clear_slot': rotated[3],
                'clear_sensor': f'A{rotated[3]}_STOPPER_SENSOR',
                'clear_stopper': f'A{rotated[3]}',
            },
        ]
        case['launch'].update({
            'right_shuttle_count': 4,
            'left_shuttle_count': 4,
            'right_start_slots': '1,2,3,4',
            'left_start_slots': '1,2,3,4',
        })
        return case

    if family == 'inspection':
        case = _base_transport_case(
            case_id=case_id,
            title=title,
            family=family,
            side=side,
            target_slot=target_slot,
            selected_index=selected_index,
            start_slots=start_slots,
            loaded_shuttles=loaded_shuttles,
            payload_condition='loaded',
            speed=speed,
            explicit=True,
            benchmark_conditions={
                'goal_type': 'inspection',
                'inspection_subject': f'{side}_rail_state',
                'expected_symbolic_goal': 'inspection_done',
            },
        )
        case['goal_type'] = 'inspection'
        case['inspection_subject'] = f'{side}_rail_state'
        return case

    if family == 'simultaneous_requests':
        alternate_slot = rotated[3] if rotated[3] != target_slot else rotated[1]
        alternate_index = selected_index % 4 + 1
        case = _base_transport_case(
            case_id=case_id,
            title=title,
            family=family,
            side=side,
            target_slot=target_slot,
            selected_index=selected_index,
            start_slots=start_slots,
            loaded_shuttles=loaded_shuttles,
            payload_condition='loaded',
            speed=speed,
            benchmark_conditions={
                'simultaneous_request_count': 2,
                'tie_breaking': 'deterministic_lowest_priority_tuple',
            },
        )
        case['simultaneous_requests'] = [
            {
                'request_id': f'{case_id}_primary',
                'target_shuttle': _shuttle(side, selected_index),
                'target_slot': target_slot,
                'priority': 0,
            },
            {
                'request_id': f'{case_id}_secondary',
                'target_shuttle': _shuttle(side, alternate_index),
                'target_slot': alternate_slot,
                'priority': 1,
            },
        ]
        return case

    challenge_by_family = {
        'unknown_position': {
            'unknown_position_shuttles': [_shuttle(side, selected_index)],
            'expected_recovery': 'observe_or_fail_closed_before_planning',
        },
        'sensor_dropout': {
            'dropout': {
                'streams': [f'overhead_{side}_rgbd'],
                'device_sensors': [f'DZI{target_slot}{"R" if side == "right" else "L"}'],
            },
            'expected_recovery': 'safe_stop_reobserve_replan',
        },
        'obstacle': {
            'obstacles': [{
                'id': f'{side}_obstacle_{family_index:03d}',
                'side': side,
                'slot': target_slot,
                'confidence': 0.95,
            }],
            'expected_recovery': 'stop_for_obstacle_then_replan_or_abort',
        },
    }
    return _base_transport_case(
        case_id=case_id,
        title=title,
        family=family,
        side=side,
        target_slot=target_slot,
        selected_index=selected_index,
        start_slots=start_slots,
        loaded_shuttles=loaded_shuttles,
        payload_condition='loaded',
        speed=speed,
        benchmark_conditions=challenge_by_family.get(family, {}),
    )


def generate_seeded_balanced_case_config(
    *,
    extension_case_count: int = DEFAULT_EXTENSION_CASES,
    seed: int = DEFAULT_SEED,
    regression_case_config: Path = DEFAULT_REGRESSION_CASE_CONFIG,
) -> dict[str, Any]:
    """Return a benchmark YAML mapping with 160 regression + balanced extension cases."""

    extension_count = int(extension_case_count)
    if extension_count < MIN_EXTENSION_CASES or extension_count > MAX_EXTENSION_CASES:
        raise ValueError(
            f'extension_case_count must be between {MIN_EXTENSION_CASES} and '
            f'{MAX_EXTENSION_CASES}; got {extension_case_count!r}'
        )
    regression_path = regression_case_config.expanduser()
    regression_config = _load_yaml_mapping(regression_path)
    regression_cases = [
        _with_regression_metadata(case, source_config=regression_path, ordinal=index)
        for index, case in enumerate(regression_config.get('cases') or [], start=1)
        if isinstance(case, dict)
    ]
    if len(regression_cases) != 160:
        raise ValueError(
            f'{regression_path} must contain the 160 regression cases; '
            f'found {len(regression_cases)}'
        )

    rng = random.Random(seed)
    family_order = list(REQUIRED_BENCHMARK_FAMILIES)
    rng.shuffle(family_order)
    family_counts = {family: 0 for family in REQUIRED_BENCHMARK_FAMILIES}
    extension_cases: list[dict[str, Any]] = []
    for index in range(extension_count):
        family = family_order[index % len(family_order)]
        family_index = family_counts[family]
        family_counts[family] += 1
        extension_cases.append(
            _case_variant(
                family=family,
                seed=int(seed),
                global_index=index + 1,
                family_index=family_index,
                rng=rng,
            )
        )

    return {
        'schema_version': 2,
        'description': (
            'Seeded Room 315 benchmark manifest. The checked-in 160 speed-sweep '
            'cases are retained as regression_160; generated cases are balanced '
            'across recovery, perception, inspection, and request-concurrency families.'
        ),
        'generation': {
            'generator': 'room_315_visual_planning_benchmark_suite.py',
            'seed': int(seed),
            'extension_case_count': extension_count,
            'total_case_count': len(regression_cases) + len(extension_cases),
            'families': list(REQUIRED_BENCHMARK_FAMILIES),
            'no_generated_datasets_or_checkpoints': True,
        },
        'regression_subset': {
            'subset_id': REGRESSION_SUBSET_ID,
            'case_count': len(regression_cases),
            'source_config': _repo_relative(regression_path),
        },
        'balanced_extension': {
            'subset_id': BALANCED_EXTENSION_ID,
            'case_count': len(extension_cases),
            'family_counts': family_counts,
            'balance_max_minus_min': max(family_counts.values()) - min(family_counts.values()),
        },
        'claim_boundary': {
            'gazebo_planning_results': (
                'These cases can drive simulator planning/execution measurements.'
            ),
            'real_image_perception_claims': (
                'Real-image perception claims require separate real-image manifests, '
                'camera calibration fingerprints, and checkpoint fingerprints.'
            ),
        },
        'cases': regression_cases + extension_cases,
    }


def summarize_case_config(config: dict[str, Any]) -> dict[str, Any]:
    cases = [case for case in config.get('cases') or [] if isinstance(case, dict)]
    subset_counts: dict[str, int] = {}
    family_counts: dict[str, int] = {}
    extension_family_counts: dict[str, int] = {}
    for case in cases:
        subset = str(case.get('benchmark_subset') or 'unknown')
        family = str(case.get('benchmark_family') or 'unknown')
        subset_counts[subset] = subset_counts.get(subset, 0) + 1
        family_counts[family] = family_counts.get(family, 0) + 1
        if subset == BALANCED_EXTENSION_ID:
            extension_family_counts[family] = extension_family_counts.get(family, 0) + 1
    required = set(REQUIRED_BENCHMARK_FAMILIES)
    covered = required.intersection(family_counts)
    extension_counts = {
        family: extension_family_counts.get(family, 0)
        for family in REQUIRED_BENCHMARK_FAMILIES
    }
    return {
        'case_count': len(cases),
        'subset_counts': subset_counts,
        'family_counts': family_counts,
        'required_families': list(REQUIRED_BENCHMARK_FAMILIES),
        'required_family_coverage_complete': covered == required,
        'missing_required_families': sorted(required - covered),
        'extension_family_balance_max_minus_min': (
            max(extension_counts.values()) - min(extension_counts.values())
            if extension_counts else 0
        ),
        'regression_subset_retained': subset_counts.get('regression_160') == 160,
    }


def write_case_config(config: dict[str, Any], output: Path) -> None:
    output = output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')


def _canonical_method(raw: Any) -> str:
    text = str(raw or '').strip().casefold().replace('-', '_')
    text = text.replace('+', '+').replace(' ', '_')
    if text in METHOD_ALIASES:
        return METHOD_ALIASES[text]
    compact = text.replace('_', ' ')
    return METHOD_ALIASES.get(compact, text)


def _load_json(path: Path) -> dict[str, Any]:
    parsed = json.loads(path.expanduser().read_text(encoding='utf-8'))
    if not isinstance(parsed, dict):
        raise ValueError(f'{path} must contain a JSON object')
    return parsed


def _numeric_or_none(raw: Any) -> float | None:
    if raw is None or raw == '':
        return None
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return round(value, 6)


def _first_metric(metrics: dict[str, Any], names: tuple[str, ...]) -> float | None:
    for name in names:
        if name in metrics:
            value = _numeric_or_none(metrics[name])
            if value is not None:
                return value
    return None


def _flatten_metrics(raw: dict[str, Any]) -> dict[str, Any]:
    metrics = {}
    for source in (
        raw,
        raw.get('metrics') if isinstance(raw.get('metrics'), dict) else {},
        raw.get('aggregate') if isinstance(raw.get('aggregate'), dict) else {},
        raw.get('aggregate_metrics') if isinstance(raw.get('aggregate_metrics'), dict) else {},
    ):
        if isinstance(source, dict):
            metrics.update(source)
    latency = raw.get('latency') if isinstance(raw.get('latency'), dict) else {}
    if latency:
        metrics.setdefault('latency_p50_s', latency.get('p50_s'))
        metrics.setdefault('latency_p95_s', latency.get('p95_s'))
    return metrics


def _infer_scope(method: str, raw: dict[str, Any]) -> str:
    explicit = str(raw.get('result_scope') or '').strip()
    if explicit:
        return explicit
    perception_source = str(raw.get('perception_source') or '').strip().casefold()
    if perception_source == 'real_image':
        return 'real_image_perception'
    return 'gazebo_planning'


def normalize_method_result(raw: dict[str, Any], *, source: str = '') -> dict[str, Any]:
    method = _canonical_method(
        raw.get('method')
        or raw.get('baseline')
        or raw.get('name')
        or raw.get('policy')
    )
    if method not in METHODS:
        raise ValueError(f'unknown benchmark method {method!r} from {source or "inline result"}')
    metrics = _flatten_metrics(raw)
    normalized_metrics: dict[str, float | None] = {}
    missing = []
    for output_name, aliases in COMPARISON_METRIC_ALIASES.items():
        value = _first_metric(metrics, aliases)
        normalized_metrics[output_name] = value
        if value is None:
            missing.append(output_name)
    scope = _infer_scope(method, raw)
    real_image_claim = bool(
        scope == 'real_image_perception'
        and str(raw.get('perception_source') or '').strip().casefold() == 'real_image'
    )
    return {
        'method': method,
        'status': 'complete' if not missing else 'partial',
        'source': source,
        'result_scope': scope,
        'gazebo_planning_result': scope == 'gazebo_planning',
        'real_image_perception_claim': real_image_claim,
        'metrics': normalized_metrics,
        'missing_metrics': missing,
        'fingerprints': raw.get('fingerprints') or raw.get('checkpoint_fingerprints') or {},
        'limitations': list(raw.get('limitations') or []),
    }


def compare_method_results(
    raw_results: list[dict[str, Any]],
    *,
    allow_missing_methods: bool = False,
) -> dict[str, Any]:
    rows = [normalize_method_result(raw, source=str(raw.get('_source') or '')) for raw in raw_results]
    by_method = {row['method']: row for row in rows}
    missing_methods = [method for method in METHODS if method not in by_method]
    if missing_methods and not allow_missing_methods:
        raise ValueError(
            'missing benchmark method result(s): ' + ', '.join(missing_methods)
        )
    for method in missing_methods:
        by_method[method] = {
            'method': method,
            'status': 'missing_result',
            'source': '',
            'result_scope': 'unverified',
            'gazebo_planning_result': False,
            'real_image_perception_claim': False,
            'metrics': {name: None for name in COMPARISON_METRIC_ALIASES},
            'missing_metrics': list(COMPARISON_METRIC_ALIASES),
            'fingerprints': {},
            'limitations': ['No result JSON was supplied; metrics are intentionally blank.'],
        }
    ordered = [by_method[method] for method in METHODS]
    return {
        'schema_version': 1,
        'methods': ordered,
        'missing_methods': missing_methods,
        'metric_fields': list(COMPARISON_METRIC_ALIASES),
        'claim_boundary': {
            'gazebo_planning_methods': [
                row['method'] for row in ordered if row['gazebo_planning_result']
            ],
            'real_image_perception_methods': [
                row['method'] for row in ordered if row['real_image_perception_claim']
            ],
            'separation_policy': (
                'Gazebo planning/execution metrics and real-image perception claims are '
                'reported as separate scopes. Visual methods are not counted as real-image '
                'claims unless their result declares perception_source=real_image.'
            ),
        },
        'limitations': [
            'Missing metrics remain null and are not replaced by proxy values.',
        ],
    }


def write_comparison_report(report: dict[str, Any], output: Path) -> dict[str, str]:
    output = output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(_pretty_json(report) + '\n', encoding='utf-8')
    csv_path = output.with_suffix('.csv')
    with csv_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=COMPARISON_CSV_FIELDS)
        writer.writeheader()
        for method in report['methods']:
            row = {
                'method': method['method'],
                'status': method['status'],
                'result_scope': method['result_scope'],
                'real_image_perception_claim': method['real_image_perception_claim'],
                'gazebo_planning_result': method['gazebo_planning_result'],
                'missing_metrics': ','.join(method['missing_metrics']),
            }
            row.update(method['metrics'])
            writer.writerow(row)
    return {'json': str(output), 'csv': str(csv_path)}


def _cmd_generate_cases(args: argparse.Namespace) -> int:
    config = generate_seeded_balanced_case_config(
        extension_case_count=args.extension_case_count,
        seed=args.seed,
        regression_case_config=args.regression_case_config,
    )
    write_case_config(config, args.output)
    summary = summarize_case_config(config)
    print(_pretty_json({'output': str(args.output.expanduser()), **summary}))
    return 0


def _cmd_compare_results(args: argparse.Namespace) -> int:
    raw_results = []
    for path in args.result_json:
        item = _load_json(path)
        item['_source'] = str(path.expanduser())
        raw_results.append(item)
    report = compare_method_results(
        raw_results,
        allow_missing_methods=args.allow_missing_methods,
    )
    paths = write_comparison_report(report, args.output)
    print(_pretty_json({'outputs': paths, 'missing_methods': report['missing_methods']}))
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Generate Room 315 seeded benchmark cases and compare method results.'
    )
    subparsers = parser.add_subparsers(dest='command', required=True)

    generate = subparsers.add_parser(
        'generate-cases',
        help='Write a seeded balanced benchmark YAML manifest.',
    )
    generate.add_argument('--output', type=Path, required=True)
    generate.add_argument(
        '--extension-case-count',
        type=int,
        default=DEFAULT_EXTENSION_CASES,
        help='Balanced generated cases to add after the retained 160 regression cases.',
    )
    generate.add_argument('--seed', type=int, default=DEFAULT_SEED)
    generate.add_argument(
        '--regression-case-config',
        type=Path,
        default=DEFAULT_REGRESSION_CASE_CONFIG,
    )
    generate.set_defaults(func=_cmd_generate_cases)

    compare = subparsers.add_parser(
        'compare-results',
        help='Normalize and compare Oracle/Visual benchmark result JSON files.',
    )
    compare.add_argument('--output', type=Path, required=True)
    compare.add_argument(
        '--result-json',
        type=Path,
        action='append',
        default=[],
        help='Result JSON with a method name and benchmark metrics. Repeat for each method.',
    )
    compare.add_argument(
        '--allow-missing-methods',
        action='store_true',
        help='Emit null rows for missing methods instead of failing.',
    )
    compare.set_defaults(func=_cmd_compare_results)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
