#!/usr/bin/env python3
"""Shared offline metrics for the frozen Room 315 direct-action baseline."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any

try:
    from room_315_multi_shuttle import decode_action_v3
except ModuleNotFoundError:
    _multi_shuttle_path = Path(__file__).with_name('room_315_multi_shuttle.py')
    _spec = importlib.util.spec_from_file_location('room_315_multi_shuttle', _multi_shuttle_path)
    if _spec is None or _spec.loader is None:
        raise
    _multi_shuttle = importlib.util.module_from_spec(_spec)
    sys.modules[_spec.name] = _multi_shuttle
    _spec.loader.exec_module(_multi_shuttle)
    decode_action_v3 = _multi_shuttle.decode_action_v3


BASELINE_ID = 'legacy_direct_action_baseline'
SPEED_FIELD = 'speed_mps'
SLOT_NAMES = ('A1', 'A2', 'A3', 'A4')
LEAKAGE_FEATURE_NAMES = {
    'payload_present',
    'step_index_norm',
    'pddl_goal',
    'pddl_problem',
    'plan_step_index',
    'payload_condition',
}


def pretty_json(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True)


def file_fingerprint(path: Path) -> dict[str, Any]:
    path = path.expanduser().resolve()
    digest = hashlib.sha256()
    size = 0
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
            size += len(chunk)
    return {
        'path': str(path),
        'sha256': digest.hexdigest(),
        'bytes': size,
    }


def directory_fingerprint(root: Path) -> dict[str, Any]:
    root = root.expanduser().resolve()
    digest = hashlib.sha256()
    file_count = 0
    total_bytes = 0
    for path in sorted(item for item in root.rglob('*') if item.is_file()):
        try:
            relative = path.relative_to(root).as_posix()
            size = path.stat().st_size
        except OSError:
            continue
        file_count += 1
        total_bytes += size
        digest.update(relative.encode('utf-8'))
        digest.update(b'\0')
        digest.update(str(size).encode('ascii'))
        digest.update(b'\n')
    return {
        'path': str(root),
        'sha256': digest.hexdigest(),
        'files': file_count,
        'bytes': total_bytes,
        'fingerprint_kind': 'relative paths and file sizes',
    }


def rows_fingerprint(rows: list[dict[str, Any]]) -> str:
    digest = hashlib.sha256()
    for row in rows:
        payload = json.dumps(row, ensure_ascii=False, sort_keys=True, separators=(',', ':'))
        digest.update(payload.encode('utf-8'))
        digest.update(b'\n')
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    with path.expanduser().open('r', encoding='utf-8') as stream:
        parsed = json.load(stream)
    return parsed if isinstance(parsed, dict) else {}


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.expanduser().open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            text = line.strip()
            if not text:
                continue
            parsed = json.loads(text)
            if not isinstance(parsed, dict):
                raise ValueError(f'{path}:{line_number}: JSONL row must be an object')
            rows.append(parsed)
    return rows


def _safe_int(raw: Any, fallback: int = 0) -> int:
    try:
        return int(raw)
    except (TypeError, ValueError):
        return fallback


def _finite_float(raw: Any, fallback: float = 0.0) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return fallback
    return value if math.isfinite(value) else fallback


def _normalise_family(text: str) -> str:
    text = str(text or '').strip()
    if not text:
        return 'unknown'
    if 'room315-' in text:
        text = text.split('room315-', 1)[1]
    if '_speed' in text:
        text = text.rsplit('_speed', 1)[0]
    return text or 'unknown'


def family_from_row(
    row: dict[str, Any],
    episode_family_lookup: dict[str, str] | None = None,
) -> str:
    problem = str(row.get('pddl_problem') or '').strip()
    if problem:
        return _normalise_family(problem)
    episode_id = str(row.get('episode_id') or '').strip()
    if episode_id and episode_family_lookup and episode_id in episode_family_lookup:
        return episode_family_lookup[episode_id]
    if episode_id:
        return _normalise_family(episode_id)
    task = str(row.get('task') or '').strip().lower()
    return _normalise_family(task.replace(' ', '_'))


def episode_family_lookup(rows: list[dict[str, Any]]) -> dict[str, str]:
    lookup: dict[str, str] = {}
    for row in rows:
        episode_id = str(row.get('episode_id') or '').strip()
        problem = str(row.get('pddl_problem') or '').strip()
        if episode_id and problem:
            lookup[episode_id] = _normalise_family(problem)
    return lookup


def quantize_action(
    values: Any,
    fields: list[str],
    ranges: dict[str, tuple[int, int]],
) -> list[float]:
    raw = list(values)
    quantized: list[float] = []
    for index, field in enumerate(fields):
        value = _finite_float(raw[index] if index < len(raw) else 0.0)
        if field != SPEED_FIELD:
            lower, upper = ranges.get(field, (0, 1))
            value = float(min(max(round(value), lower), upper))
        quantized.append(value)
    return quantized


def _selected_slots(action: dict[str, Any], mask_key: str) -> list[str]:
    mask = action.get(mask_key)
    if not isinstance(mask, dict):
        return []
    selected = []
    for slot in SLOT_NAMES:
        if _finite_float(mask.get(slot), 0.0) >= 0.5:
            selected.append(slot)
    return selected


def offline_supervisor_decision(action_values: Any) -> dict[str, Any]:
    values = [float(value) for value in list(action_values)]
    try:
        decoded = decode_action_v3(values)
    except Exception as exc:
        return {
            'accepted': False,
            'schema_decoded': False,
            'reason': f'schema decode failed: {exc}',
            'decoded_action': None,
            'offline_proxy': True,
            'live_supervisor': False,
        }

    primitive = str(decoded.get('primitive') or '').upper()
    reason = 'accepted_offline_proxy'
    accepted = True
    if primitive == 'SET_SWITCHES':
        selected = _selected_slots(decoded, 'switch_mask')
        if not selected:
            accepted = False
            reason = 'SET_SWITCHES has no selected switch'
        elif all(str(decoded.get('switch_values', {}).get(slot)) == 'UNCHANGED' for slot in selected):
            accepted = False
            reason = 'SET_SWITCHES selected only UNCHANGED switch values'
    elif primitive == 'SET_STOPPERS':
        selected = _selected_slots(decoded, 'stopper_mask')
        if not selected:
            accepted = False
            reason = 'SET_STOPPERS has no selected stopper'
        elif all(str(decoded.get('stopper_values', {}).get(slot)) == 'UNCHANGED' for slot in selected):
            accepted = False
            reason = 'SET_STOPPERS selected only UNCHANGED stopper values'
    elif primitive == 'SHUTTLE_ON':
        if _finite_float(decoded.get('speed_mps'), 0.0) <= 0.0:
            accepted = False
            reason = 'SHUTTLE_ON has non-positive speed'
    elif primitive not in {
        'WAIT',
        'DONE',
        'STOP_NOW',
        'EMERGENCY_STOP',
        'SET_SWITCHES',
        'SET_STOPPERS',
        'SHUTTLE_ON',
    }:
        accepted = False
        reason = f'unsupported primitive {primitive!r}'

    return {
        'accepted': accepted,
        'schema_decoded': True,
        'reason': reason,
        'decoded_action': decoded,
        'offline_proxy': True,
        'live_supervisor': False,
    }


def latency_summary(values: list[float]) -> dict[str, Any]:
    clean = sorted(value for value in values if math.isfinite(value) and value >= 0.0)
    if not clean:
        return {
            'count': 0,
            'total': None,
            'mean': None,
            'p50': None,
            'p95': None,
            'max': None,
        }

    def percentile(frac: float) -> float:
        index = min(len(clean) - 1, max(0, math.ceil(frac * len(clean)) - 1))
        return round(clean[index], 6)

    total = sum(clean)
    return {
        'count': len(clean),
        'total': round(total, 6),
        'mean': round(total / len(clean), 6),
        'p50': percentile(0.50),
        'p95': percentile(0.95),
        'max': round(max(clean), 6),
    }


def _field_accuracy(
    true_values: list[list[float]],
    pred_values: list[list[float]],
    fields: list[str],
    field: str,
) -> float | None:
    if field not in fields or not true_values:
        return None
    index = fields.index(field)
    correct = sum(1 for true, pred in zip(true_values, pred_values) if true[index] == pred[index])
    return round(correct / len(true_values), 6)


def _grouped_field_accuracy(
    true_values: list[list[float]],
    pred_values: list[list[float]],
    fields: list[str],
    prefix: str,
) -> float | None:
    indexes = [index for index, field in enumerate(fields) if field.startswith(prefix)]
    if not indexes or not true_values:
        return None
    total = len(true_values) * len(indexes)
    correct = 0
    for true, pred in zip(true_values, pred_values):
        correct += sum(1 for index in indexes if true[index] == pred[index])
    return round(correct / total, 6)


def action_metrics(
    records: list[dict[str, Any]],
    fields: list[str],
    *,
    speed_tolerance: float,
) -> dict[str, Any]:
    if not records:
        return {'samples': 0}
    true_q = [list(record['true_quantized']) for record in records]
    pred_q = [list(record['pred_quantized']) for record in records]
    true_raw = [list(record['true_raw']) for record in records]
    pred_raw = [list(record['pred_raw']) for record in records]
    speed_index = fields.index(SPEED_FIELD)
    discrete_indexes = [index for index, field in enumerate(fields) if field != SPEED_FIELD]
    abs_errors = [
        [abs(float(pred[index]) - float(true[index])) for index in range(len(fields))]
        for true, pred in zip(true_raw, pred_raw)
    ]
    exact_discrete = [
        all(true[index] == pred[index] for index in discrete_indexes)
        for true, pred in zip(true_q, pred_q)
    ]
    exact_action = [
        discrete_ok and abs(float(pred[speed_index]) - float(true[speed_index])) <= speed_tolerance
        for discrete_ok, true, pred in zip(exact_discrete, true_raw, pred_raw)
    ]
    decisions = [
        record.get('supervisor_decision') or offline_supervisor_decision(record['pred_quantized'])
        for record in records
    ]
    accepted = [bool(decision.get('accepted')) for decision in decisions]
    decoded = [bool(decision.get('schema_decoded')) for decision in decisions]
    rejection_reasons = Counter(
        str(decision.get('reason') or 'unknown')
        for decision in decisions
        if not decision.get('accepted')
    )
    total_abs_error = sum(sum(row) for row in abs_errors)
    return {
        'samples': len(records),
        'action_mae': round(total_abs_error / max(1, len(records) * len(fields)), 6),
        'discrete_field_accuracy': round(
            sum(
                1
                for true, pred in zip(true_q, pred_q)
                for index in discrete_indexes
                if true[index] == pred[index]
            ) / max(1, len(records) * len(discrete_indexes)),
            6,
        ),
        'exact_discrete_action_accuracy': round(sum(exact_discrete) / len(records), 6),
        'exact_action_accuracy': round(sum(exact_action) / len(records), 6),
        'primitive_accuracy': _field_accuracy(true_q, pred_q, fields, 'primitive_id'),
        'side_accuracy': _field_accuracy(true_q, pred_q, fields, 'side_id'),
        'target_id_accuracy': _field_accuracy(true_q, pred_q, fields, 'target_id'),
        'switch_mask_accuracy': _grouped_field_accuracy(true_q, pred_q, fields, 'switch_mask_'),
        'switch_value_accuracy': _grouped_field_accuracy(true_q, pred_q, fields, 'switch_value_'),
        'stopper_mask_accuracy': _grouped_field_accuracy(true_q, pred_q, fields, 'stopper_mask_'),
        'stopper_value_accuracy': _grouped_field_accuracy(true_q, pred_q, fields, 'stopper_value_'),
        'action_schema_decode_success_rate': round(sum(decoded) / len(records), 6),
        'action_schema_legality_rate': round(sum(accepted) / len(records), 6),
        'supervisor_rejection_rate': round((len(records) - sum(accepted)) / len(records), 6),
        'supervisor_rejection_reasons': dict(rejection_reasons.most_common()),
        'supervisor_rejection_source': (
            'offline schema-v3 proxy; live Room315VlaSupervisor execution was not run'
        ),
        'inference_latency_seconds': latency_summary([
            _finite_float(record.get('inference_latency_seconds'), math.nan)
            for record in records
        ]),
        'cycle_time_seconds': latency_summary([
            _finite_float(record.get('cycle_time_seconds'), math.nan)
            for record in records
        ]),
    }


def action_metrics_by_family(
    records: list[dict[str, Any]],
    fields: list[str],
    *,
    speed_tolerance: float,
) -> dict[str, dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for record in records:
        grouped.setdefault(str(record.get('task_family') or 'unknown'), []).append(record)
    return {
        family: action_metrics(family_records, fields, speed_tolerance=speed_tolerance)
        for family, family_records in sorted(grouped.items())
    }


def detect_vectorizer_leakage(vectorizer_json: Path | None) -> dict[str, Any]:
    if vectorizer_json is None:
        return {
            'checked': False,
            'comparison_valid': None,
            'reason': 'no state vectorizer JSON supplied',
        }
    path = vectorizer_json.expanduser()
    if not path.exists():
        return {
            'checked': False,
            'comparison_valid': None,
            'reason': f'state vectorizer JSON not found: {path}',
        }
    parsed = load_json(path)
    names = parsed.get('names')
    if not isinstance(names, list):
        names = list(parsed.get('numeric_keys') or [])
        categorical_values = parsed.get('categorical_values')
        if isinstance(categorical_values, dict):
            for key, values in categorical_values.items():
                if isinstance(values, list):
                    names.extend(f'{key}=={value}' for value in values)
    leaked = sorted(set(str(name) for name in names) & LEAKAGE_FEATURE_NAMES)
    return {
        'checked': True,
        'state_vectorizer': str(path),
        'comparison_valid': not leaked,
        'leaked_features': leaked,
        'reason': (
            'no known leakage features found'
            if not leaked
            else 'state vectorizer includes row-level payload/PDDL/step leakage features'
        ),
    }
