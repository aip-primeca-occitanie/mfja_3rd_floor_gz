#!/usr/bin/env python3
"""Coverage report for manual and PDDL-generated Room 315 VLA datasets."""

import argparse
import json
import re
import sys
from collections import Counter
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_pddl_validation_gate import load_validation_result
from room_315_pddl_validation_gate import validation_approves_training


def _json_dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True)


def iter_jsonl(path: Path):
    with path.open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                parsed = json.loads(text)
            except json.JSONDecodeError as exc:
                raise ValueError(f'{path}:{line_number}: invalid JSONL row: {exc}') from exc
            if isinstance(parsed, dict):
                yield parsed


def event_files_from_input(input_path: Path | str) -> tuple[list[Path], Path]:
    path = Path(input_path).expanduser()
    if path.is_file():
        return [path.resolve()], path.resolve().parent
    dataset_dir = path.resolve()
    root_events = dataset_dir / 'events.jsonl'
    if root_events.exists():
        return [root_events], dataset_dir
    episode_events = sorted((dataset_dir / 'episodes').glob('episode_*/events.jsonl'))
    if episode_events:
        return episode_events, dataset_dir
    training_events = dataset_dir / 'meta' / 'training_events.jsonl'
    if training_events.exists():
        return [training_events], dataset_dir
    recursive_events = sorted(dataset_dir.rglob('events.jsonl'))
    return recursive_events, dataset_dir


def load_event_rows(input_path: Path | str) -> tuple[list[dict[str, Any]], Path, list[str]]:
    event_files, dataset_root = event_files_from_input(input_path)
    rows: list[dict[str, Any]] = []
    for event_file in event_files:
        for row in iter_jsonl(event_file):
            normalized = dict(row)
            if not normalized.get('episode_id'):
                normalized['episode_id'] = _episode_id_from_path(event_file)
            normalized['_event_file'] = str(event_file)
            rows.append(normalized)
    return rows, dataset_root, [str(path) for path in event_files]


def load_validation_results(dataset_root: Path | str) -> dict[str, dict[str, Any]]:
    root = Path(dataset_root).expanduser().resolve()
    validations: dict[str, dict[str, Any]] = {}
    for validation_path in sorted((root / 'episodes').glob('episode_*/validation.json')):
        validation = load_validation_result(validation_path.parent)
        if not isinstance(validation, dict):
            continue
        episode_id = str(validation.get('episode_id') or validation_path.parent.name)
        validations[episode_id] = validation
    return validations


def build_dataset_report(input_path: Path | str) -> dict[str, Any]:
    rows, dataset_root, event_files = load_event_rows(input_path)
    validations = load_validation_results(dataset_root)
    episode_ids = sorted({
        str(row.get('episode_id') or '')
        for row in rows
        if str(row.get('episode_id') or '')
    } | set(validations))
    approved_episode_ids = sorted(
        episode_id
        for episode_id in episode_ids
        if validation_approves_training(validations.get(episode_id))
    )
    approved_episode_set = set(approved_episode_ids)
    failed_episode_ids = sorted(set(episode_ids) - approved_episode_set)
    approved_rows = [
        row
        for row in rows
        if str(row.get('episode_id') or '') in approved_episode_set
    ]
    skipped_event_count = len(rows) - len(approved_rows)
    dataset_source_by_event = [_dataset_source(row) for row in approved_rows]
    source_counts = Counter(dataset_source_by_event)
    episode_summaries = _episode_summaries(rows, validations)
    plan_lengths = [
        summary['symbolic_plan_length']
        for summary in episode_summaries
        if summary.get('approved_for_training') is True
        if summary['symbolic_plan_length'] > 0
    ]
    task_success = _task_success_summary(approved_rows)
    report = {
        'dataset_root': str(dataset_root),
        'event_files': event_files,
        'dataset_source': _overall_dataset_source(source_counts),
        'dataset_source_distribution': dict(sorted(source_counts.items())),
        'number_of_episodes': len(episode_ids),
        'total_episodes': len(episode_ids),
        'approved_episodes': len(approved_episode_ids),
        'failed_episodes': len(failed_episode_ids),
        'approval_rate': (
            None if not episode_ids else round(len(approved_episode_ids) / len(episode_ids), 6)
        ),
        'failure_reasons_distribution': _failure_reasons_distribution(
            episode_ids,
            validations,
        ),
        'number_of_events': len(rows),
        'approved_event_count': len(approved_rows),
        'skipped_event_count': skipped_event_count,
        'goals_covered': _goals_covered(approved_rows),
        'language_variants_count': len(_language_variants(approved_rows)),
        'generated_language_template_ids': sorted(_language_template_ids(approved_rows)),
        'average_plan_length': (
            0.0 if not plan_lengths else round(sum(plan_lengths) / len(plan_lengths), 4)
        ),
        'action_primitive_distribution': dict(
            sorted(_action_primitive_distribution(approved_rows).items())
        ),
        'side_distribution': dict(sorted(_side_distribution(approved_rows).items())),
        'speed_distribution': dict(sorted(_speed_distribution(approved_rows).items())),
        'shuttle_id_accuracy': _shuttle_id_accuracy(approved_rows),
        'wrong_shuttle_command_rate': _wrong_shuttle_command_rate(approved_rows),
        'identity_grounding_accuracy': _shuttle_id_accuracy(approved_rows),
        'target_shuttle_selection_accuracy': _shuttle_id_accuracy(approved_rows),
        'target_shuttle_distribution': dict(
            sorted(_target_shuttle_distribution(approved_rows).items())
        ),
        'visible_marker_count_distribution': dict(
            sorted(_visible_marker_count_distribution(approved_rows).items())
        ),
        'identity_occlusion_level_distribution': dict(
            sorted(_identity_occlusion_level_distribution(approved_rows).items())
        ),
        'occluded_identity_success_rate': _success_rate_for_rows(
            approved_rows,
            lambda row: _identity_occlusion_level(row) not in {'', 'none', 'visible'},
        ),
        'partial_occlusion_success_rate': _success_rate_for_rows(
            approved_rows,
            lambda row: _identity_occlusion_level(row) == 'partial',
        ),
        'loaded_shuttle_success_rate': _success_rate_for_rows(
            approved_rows,
            lambda row: _payload_present(row) is True,
        ),
        'unloaded_shuttle_success_rate': _success_rate_for_rows(
            approved_rows,
            lambda row: _payload_present(row) is False,
        ),
        'collision_or_near_collision_count': _numeric_metric(
            approved_rows,
            'collision_or_near_collision_count',
            default=0,
        ),
        'headway_violation_count': _validation_sum(validations, episode_ids, 'headway_violation_count'),
        'block_occupancy_violation_count': _validation_sum(
            validations,
            episode_ids,
            'block_occupancy_violation_count',
        ),
        'block_reservation_rejection_count': _validation_sum(
            validations,
            episode_ids,
            'block_reservation_rejection_count',
        ),
        'deadlock_detected_count': _validation_sum(validations, episode_ids, 'deadlock_detected_count'),
        'deadlock_avoided_count': _validation_sum(validations, episode_ids, 'deadlock_avoided_count'),
        'wrong_shuttle_command_count': _validation_sum(
            validations,
            episode_ids,
            'wrong_shuttle_command_count',
        ),
        'headway_violation_rate': _metric_rate_from_count(approved_rows, 'headway_violation_count'),
        'block_reservation_success_rate': _block_reservation_success_rate(approved_rows),
        'deadlock_rate': _metric_rate_from_count(approved_rows, 'deadlock_detected_count'),
        'deadlock_avoidance_success_rate': _deadlock_avoidance_success_rate(approved_rows),
        'average_wait_time': _average_wait_time(approved_rows),
        'fleet_throughput_tasks_per_minute': _numeric_metric(
            approved_rows,
            'fleet_throughput_tasks_per_minute',
        ),
        'per_side_success_rate': _per_group_success_rate(approved_rows, _row_side),
        'per_shuttle_success_rate': _per_group_success_rate(approved_rows, _target_shuttle),
        'illegal_proposal_rate': _numeric_metric(approved_rows, 'illegal_proposal_rate'),
        'rejected_action_rate': _rejected_action_rate(approved_rows),
        'task_success_rate_for_approved_episodes': _validation_bool_rate(
            validations,
            approved_episode_ids,
            'task_success',
        ),
        'rejected_action_rate_for_approved_episodes': _validation_average(
            validations,
            approved_episode_ids,
            'rejected_action_rate',
        ),
        'task_success': task_success,
        'episodes': episode_summaries,
    }
    return report


def write_report(path: Path | str, report: dict[str, Any]) -> None:
    output_path = Path(path).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(_json_dumps(report) + '\n', encoding='utf-8')


def _episode_id_from_path(path: Path) -> str:
    if path.parent.name.startswith('episode_'):
        return path.parent.name
    return path.stem


def _dataset_source(row: dict[str, Any]) -> str:
    source = str(row.get('dataset_source') or row.get('planning_source') or '').strip().casefold()
    if source == 'pddl':
        return 'pddl'
    if _pddl_goal(row) or _pddl_problem(row) or _symbolic_plan(row):
        return 'pddl'
    return 'manual'


def _overall_dataset_source(counts: Counter) -> str:
    active = {source for source, count in counts.items() if count > 0}
    if active == {'pddl'}:
        return 'pddl'
    if active == {'manual'}:
        return 'manual'
    if not active:
        return 'manual'
    return 'mixed'


def _episode_summaries(
    rows: list[dict[str, Any]],
    validations: dict[str, dict[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    validations = validations or {}
    by_episode: dict[str, list[dict[str, Any]]] = {}
    for index, row in enumerate(rows):
        episode_id = str(row.get('episode_id') or f'episode_{index:06d}')
        by_episode.setdefault(episode_id, []).append(row)
    for episode_id in validations:
        by_episode.setdefault(episode_id, [])

    summaries = []
    for episode_id in sorted(by_episode):
        episode_rows = by_episode[episode_id]
        first = episode_rows[0] if episode_rows else {}
        validation = validations.get(episode_id, {})
        source_counts = Counter(_dataset_source(row) for row in episode_rows)
        symbolic_plan = _first_non_empty(_symbolic_plan(row) for row in episode_rows)
        plan_length = _symbolic_plan_length(symbolic_plan, episode_rows)
        summaries.append({
            'episode_id': episode_id,
            'validation_status': str(validation.get('validation_status') or 'missing'),
            'approved_for_training': validation_approves_training(validation),
            'failure_reason': str(validation.get('failure_reason') or ''),
            'dataset_source': _overall_dataset_source(source_counts),
            'number_of_events': len(episode_rows),
            'pddl_goal': _first_non_empty(_pddl_goal(row) for row in episode_rows),
            'pddl_problem': _first_non_empty(_pddl_problem(row) for row in episode_rows),
            'symbolic_plan_length': plan_length,
            'generated_language_template_id': _first_non_empty(
                _language_template_id(row) for row in episode_rows
            ),
            'language': _language(first),
            'task_success': (
                validation.get('task_success')
                if isinstance(validation.get('task_success'), bool)
                else _episode_task_success(episode_rows)
            ),
        })
    return summaries


def _goals_covered(rows: list[dict[str, Any]]) -> list[str]:
    goals = {
        _goal_for_row(row)
        for row in rows
        if _goal_for_row(row)
    }
    return sorted(goals)


def _goal_for_row(row: dict[str, Any]) -> str:
    return _pddl_goal(row) or str(row.get('task') or _language(row) or '').strip()


def _language_variants(rows: list[dict[str, Any]]) -> set[str]:
    variants = {
        _language_template_id(row)
        for row in rows
        if _language_template_id(row)
    }
    if variants:
        return variants
    return {
        _language(row)
        for row in rows
        if _language(row)
    }


def _language_template_ids(rows: list[dict[str, Any]]) -> set[str]:
    return {
        _language_template_id(row)
        for row in rows
        if _language_template_id(row)
    }


def _action_primitive_distribution(rows: list[dict[str, Any]]) -> Counter:
    counts: Counter = Counter()
    for row in rows:
        primitive = str(_action(row).get('primitive') or '').strip()
        if primitive:
            counts[primitive] += 1
    return counts


def _side_distribution(rows: list[dict[str, Any]]) -> Counter:
    counts: Counter = Counter()
    for row in rows:
        side = str(_action(row).get('side') or '').strip()
        if side:
            counts[side] += 1
    return counts


def _speed_distribution(rows: list[dict[str, Any]]) -> Counter:
    counts: Counter = Counter()
    for row in rows:
        speed = _speed_from_row(row)
        if speed is None or speed <= 0.0:
            continue
        counts[f'{speed:.4g}'] += 1
    return counts


def _speed_from_row(row: dict[str, Any]) -> float | None:
    action = _action(row)
    for value in (action.get('speed_mps'), action.get('speed')):
        parsed = _safe_float(value)
        if parsed is not None:
            return parsed
    original = row.get('original_command')
    if isinstance(original, dict):
        parsed = _safe_float(original.get('speed'))
        if parsed is not None:
            return parsed
    return None


def _target_shuttle_distribution(rows: list[dict[str, Any]]) -> Counter:
    counts: Counter = Counter()
    for row in rows:
        target = _target_shuttle(row)
        if target:
            counts[target] += 1
    return counts


def _visible_marker_count_distribution(rows: list[dict[str, Any]]) -> Counter:
    counts: Counter = Counter()
    for row in rows:
        marker_count = _visible_marker_count(row)
        if marker_count is not None:
            counts[str(marker_count)] += 1
    return counts


def _identity_occlusion_level_distribution(rows: list[dict[str, Any]]) -> Counter:
    counts: Counter = Counter()
    for row in rows:
        level = _identity_occlusion_level(row)
        if level:
            counts[level] += 1
    return counts


def _shuttle_id_accuracy(rows: list[dict[str, Any]]) -> float | None:
    comparisons = []
    for row in rows:
        expected = _normalize_shuttle_label(
            _privileged_eval(row).get('correct_target_shuttle')
            or row.get('target_shuttle_id')
        )
        chosen = _normalize_shuttle_label(
            _action(row).get('shuttle_id')
            or _action(row).get('target_id')
            or row.get('executed_shuttle_id')
        )
        if expected and chosen:
            comparisons.append(expected == chosen)
    if not comparisons:
        return None
    return round(sum(1 for ok in comparisons if ok) / len(comparisons), 6)


def _wrong_shuttle_command_rate(rows: list[dict[str, Any]]) -> float | None:
    explicit = _numeric_metric(rows, 'wrong_shuttle_command_rate')
    if explicit is not None:
        return explicit
    wrong_count = _numeric_metric(rows, 'wrong_shuttle_command_count')
    total = _numeric_metric(rows, 'total_proposed_actions')
    if wrong_count is not None and total is not None and total > 0:
        return round(wrong_count / total, 6)
    accuracy = _shuttle_id_accuracy(rows)
    if accuracy is None:
        return None
    return round(1.0 - accuracy, 6)


def _identity_occlusion_level(row: dict[str, Any]) -> str:
    return str(
        row.get('identity_occlusion_level')
        or _privileged_eval(row).get('identity_occlusion_level')
        or ''
    ).strip().casefold()


def _payload_present(row: dict[str, Any]) -> bool | None:
    for value in (row.get('payload_present'), _privileged_eval(row).get('payload_present')):
        if isinstance(value, bool):
            return value
    return None


def _visible_marker_count(row: dict[str, Any]) -> int | None:
    for value in (
        row.get('visible_marker_count'),
        _privileged_eval(row).get('visible_marker_count_for_target'),
        _privileged_eval(row).get('visible_marker_count'),
    ):
        parsed = _safe_int_or_none(value)
        if parsed is not None:
            return parsed
    expected_ids = row.get('expected_visible_ids') or _privileged_eval(row).get('expected_visible_ids')
    if isinstance(expected_ids, list):
        return len(expected_ids)
    return None


def _success_rate_for_rows(rows: list[dict[str, Any]], predicate) -> float | None:
    outcomes = []
    for row in rows:
        if not predicate(row):
            continue
        outcome = _row_task_success(row)
        if outcome is not None:
            outcomes.append(outcome)
    if not outcomes:
        return None
    return round(sum(1 for outcome in outcomes if outcome) / len(outcomes), 6)


def _metric_rate_from_count(rows: list[dict[str, Any]], count_key: str) -> float | None:
    explicit_rate = _numeric_metric(rows, count_key.replace('_count', '_rate'))
    if explicit_rate is not None:
        return explicit_rate
    count = _numeric_metric(rows, count_key)
    total = _numeric_metric(rows, 'total_proposed_actions')
    if count is None or total is None or total <= 0:
        return None
    return round(count / total, 6)


def _block_reservation_success_rate(rows: list[dict[str, Any]]) -> float | None:
    explicit = _numeric_metric(rows, 'block_reservation_success_rate')
    if explicit is not None:
        return explicit
    rejected = _numeric_metric(rows, 'block_reservation_rejection_count')
    accepted = _numeric_metric(rows, 'block_reservation_success_count')
    if accepted is None and rejected is None:
        return None
    accepted = accepted or 0
    rejected = rejected or 0
    total = accepted + rejected
    return None if total <= 0 else round(accepted / total, 6)


def _deadlock_avoidance_success_rate(rows: list[dict[str, Any]]) -> float | None:
    explicit = _numeric_metric(rows, 'deadlock_avoidance_success_rate')
    if explicit is not None:
        return explicit
    avoided = _numeric_metric(rows, 'deadlock_avoided_count')
    detected = _numeric_metric(rows, 'deadlock_detected_count')
    if avoided is None and detected is None:
        return None
    avoided = avoided or 0
    detected = detected or 0
    total = avoided + detected
    return None if total <= 0 else round(avoided / total, 6)


def _average_wait_time(rows: list[dict[str, Any]]) -> float | None:
    explicit = _numeric_metric(rows, 'average_wait_time')
    if explicit is not None:
        return explicit
    values = []
    for row in rows:
        metrics = _safety_metrics(row)
        waits = metrics.get('average_wait_time_by_shuttle')
        if isinstance(waits, dict):
            for value in waits.values():
                parsed = _safe_float(value)
                if parsed is not None:
                    values.append(parsed)
    if not values:
        return None
    return round(sum(values) / len(values), 6)


def _per_group_success_rate(rows: list[dict[str, Any]], group_getter) -> dict[str, float]:
    grouped: dict[str, list[bool]] = {}
    for episode_rows in _rows_by_episode(rows).values():
        group = _first_non_empty(group_getter(row) for row in episode_rows)
        if not group:
            continue
        outcome = _episode_task_success(episode_rows)
        if outcome is None:
            continue
        grouped.setdefault(str(group), []).append(outcome)
    return {
        key: round(sum(1 for outcome in outcomes if outcome) / len(outcomes), 6)
        for key, outcomes in sorted(grouped.items())
        if outcomes
    }


def _row_side(row: dict[str, Any]) -> str:
    return str(_action(row).get('side') or '').strip()


def _target_shuttle(row: dict[str, Any]) -> str:
    return _normalize_shuttle_label(
        row.get('target_shuttle_id')
        or _privileged_eval(row).get('correct_target_shuttle')
        or _action(row).get('shuttle_id')
        or _action(row).get('target_id')
    )


def _privileged_eval(row: dict[str, Any]) -> dict[str, Any]:
    value = row.get('privileged_eval')
    return value if isinstance(value, dict) else {}


def _safety_metrics(row: dict[str, Any]) -> dict[str, Any]:
    metrics = row.get('safety_decoder_metrics')
    if isinstance(metrics, dict):
        return metrics
    status = row.get('status')
    if isinstance(status, dict):
        safety = status.get('safety_decoder')
        if isinstance(safety, dict) and isinstance(safety.get('metrics'), dict):
            return safety['metrics']
    return {}


def _numeric_metric(rows: list[dict[str, Any]], key: str, default: Any = None) -> float | None:
    best_value = None
    best_total = -1.0
    for row in rows:
        candidates = [
            row.get(key),
            _privileged_eval(row).get(key),
            _safety_metrics(row).get(key),
        ]
        for value in candidates:
            parsed = _safe_float(value)
            if parsed is None:
                continue
            total = _safe_float(_safety_metrics(row).get('total_proposed_actions')) or 0.0
            if total >= best_total:
                best_total = total
                best_value = parsed
    if best_value is None:
        return default
    return best_value


def _row_task_success(row: dict[str, Any]) -> bool | None:
    metrics = row.get('event_generation_metrics')
    if isinstance(metrics, dict) and isinstance(metrics.get('task_success'), bool):
        return metrics['task_success']
    if isinstance(row.get('task_success'), bool):
        return row['task_success']
    return None


def _normalize_shuttle_label(value: Any) -> str:
    text = str(value or '').strip()
    if not text:
        return ''
    upper = text.upper()
    match = re.fullmatch(r'([RL])([1-4])', upper)
    if match:
        return f'{match.group(1)}{match.group(2)}'
    lowered = text.casefold()
    match = re.fullmatch(r'(?:room315_)?(right|left)_shuttle_?([1-4])', lowered)
    if match:
        return f'{"R" if match.group(1) == "right" else "L"}{match.group(2)}'
    return ''


def _rejected_action_rate(rows: list[dict[str, Any]]) -> float | None:
    best_metrics = None
    best_total = -1.0
    for row in rows:
        metrics = row.get('safety_decoder_metrics', {})
        if not isinstance(metrics, dict):
            continue
        total = _safe_float(metrics.get('total_proposed_actions')) or 0.0
        if total >= best_total:
            best_total = total
            best_metrics = metrics
    if not best_metrics:
        return None
    explicit = _safe_float(best_metrics.get('rejected_action_rate'))
    if explicit is not None:
        return explicit
    rejected = _safe_float(best_metrics.get('rejected_actions'))
    total = _safe_float(best_metrics.get('total_proposed_actions'))
    if rejected is None or total is None or total <= 0.0:
        return None
    return round(rejected / total, 6)


def _task_success_summary(rows: list[dict[str, Any]]) -> dict[str, Any] | None:
    outcomes = [
        _episode_task_success(episode_rows)
        for episode_rows in _rows_by_episode(rows).values()
    ]
    known = [outcome for outcome in outcomes if outcome is not None]
    if not known:
        return None
    successes = sum(1 for outcome in known if outcome is True)
    failures = sum(1 for outcome in known if outcome is False)
    return {
        'successes': successes,
        'failures': failures,
        'unknown': len(outcomes) - len(known),
        'success_rate': round(successes / len(known), 6) if known else None,
    }


def _episode_task_success(rows: list[dict[str, Any]]) -> bool | None:
    outcome = None
    for row in rows:
        metrics = row.get('event_generation_metrics', {})
        if isinstance(metrics, dict) and isinstance(metrics.get('task_success'), bool):
            outcome = metrics['task_success']
        if isinstance(row.get('task_success'), bool):
            outcome = row['task_success']
        action = _action(row)
        if str(action.get('primitive') or '').upper() == 'DONE':
            status = _status_text(row).casefold()
            reason = str(action.get('reason') or '').casefold()
            action_status = str(action.get('status') or '').casefold()
            if (
                'success' in status
                or 'succeeded' in status
                or reason == 'task_succeeded'
                or action_status in {'success', 'succeeded'}
            ):
                outcome = True
            elif (
                'failure' in status
                or 'failed' in status
                or 'discarded' in status
                or reason in {'task_failed', 'episode_discarded'}
                or action_status in {'failure', 'failed', 'discarded'}
            ):
                outcome = False
    return outcome


def _failure_reasons_distribution(
    episode_ids: list[str],
    validations: dict[str, dict[str, Any]],
) -> dict[str, int]:
    counts: Counter = Counter()
    for episode_id in episode_ids:
        validation = validations.get(episode_id)
        if validation_approves_training(validation):
            continue
        if not isinstance(validation, dict):
            counts['unvalidated_episode'] += 1
            continue
        reason = str(
            validation.get('failure_reason')
            or validation.get('validation_status')
            or 'failed_validation'
        ).strip()
        counts[reason or 'failed_validation'] += 1
    return dict(sorted(counts.items()))


def _validation_sum(
    validations: dict[str, dict[str, Any]],
    episode_ids: list[str],
    key: str,
) -> int:
    total = 0
    for episode_id in episode_ids:
        validation = validations.get(episode_id, {})
        parsed = _safe_int_or_none(validation.get(key)) if isinstance(validation, dict) else None
        total += parsed or 0
    return total


def _validation_average(
    validations: dict[str, dict[str, Any]],
    episode_ids: list[str],
    key: str,
) -> float | None:
    values = []
    for episode_id in episode_ids:
        validation = validations.get(episode_id, {})
        if not isinstance(validation, dict):
            continue
        parsed = _safe_float(validation.get(key))
        if parsed is not None:
            values.append(parsed)
    if not values:
        return None
    return round(sum(values) / len(values), 6)


def _validation_bool_rate(
    validations: dict[str, dict[str, Any]],
    episode_ids: list[str],
    key: str,
) -> float | None:
    values = []
    for episode_id in episode_ids:
        validation = validations.get(episode_id, {})
        if isinstance(validation, dict) and isinstance(validation.get(key), bool):
            values.append(validation[key])
    if not values:
        return None
    return round(sum(1 for value in values if value) / len(values), 6)


def _rows_by_episode(rows: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for index, row in enumerate(rows):
        episode_id = str(row.get('episode_id') or f'episode_{index:06d}')
        grouped.setdefault(episode_id, []).append(row)
    return grouped


def _action(row: dict[str, Any]) -> dict[str, Any]:
    action = row.get('action')
    if isinstance(action, dict):
        return action
    next_action = row.get('next_action')
    return next_action if isinstance(next_action, dict) else {}


def _language(row: dict[str, Any]) -> str:
    model_input = row.get('model_input')
    if isinstance(model_input, dict):
        language = model_input.get('language')
        if language:
            return str(language)
    return str(row.get('generated_language') or row.get('language') or row.get('task') or '')


def _status_text(row: dict[str, Any]) -> str:
    if row.get('task_status'):
        return str(row.get('task_status') or '')
    status = row.get('status')
    if isinstance(status, dict):
        return str(status.get('event_status') or '')
    return str(row.get('status') or '')


def _pddl_goal(row: dict[str, Any]) -> str:
    return str(row.get('pddl_goal') or '').strip()


def _pddl_problem(row: dict[str, Any]) -> str:
    return str(row.get('pddl_problem') or '').strip()


def _symbolic_plan(row: dict[str, Any]) -> list[Any]:
    plan = row.get('symbolic_plan')
    return list(plan) if isinstance(plan, list) else []


def _symbolic_plan_length(plan: list[Any], rows: list[dict[str, Any]]) -> int:
    if plan:
        return len(plan)
    for row in rows:
        parsed = _safe_int_or_none(row.get('symbolic_plan_length'))
        if parsed is not None:
            return parsed
    indices = [
        _safe_int_or_none(row.get('plan_step_index'))
        for row in rows
    ]
    known_indices = [index for index in indices if index is not None and index >= 0]
    return 0 if not known_indices else max(known_indices) + 1


def _language_template_id(row: dict[str, Any]) -> str:
    return str(
        row.get('generated_language_template_id')
        or row.get('language_template_id')
        or ''
    ).strip()


def _first_non_empty(values) -> Any:
    for value in values:
        if value:
            return value
    return ''


def _safe_float(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _safe_int_or_none(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Report coverage metrics for Room 315 VLA events.jsonl datasets.'
    )
    parser.add_argument('dataset', type=Path, help='Dataset directory or events JSONL file.')
    parser.add_argument('--output', type=Path, default=None, help='Optional JSON report path.')
    args = parser.parse_args(argv)

    report = build_dataset_report(args.dataset)
    if args.output is not None:
        write_report(args.output, report)
    print(_json_dumps(report))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
