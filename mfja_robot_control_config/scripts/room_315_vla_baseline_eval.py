#!/usr/bin/env python3

import argparse
import csv
import hashlib
import json
import math
import random
from collections import Counter
from pathlib import Path
from typing import Any

try:
    import cv2
except ImportError:  # pragma: no cover - exercised only on minimal machines.
    cv2 = None


BASELINE_NAMES = ('state_only', 'vla', 'oracle')
METRIC_FIELDS = (
    'baseline',
    'task_family',
    'train_rows',
    'eval_rows',
    'action_accuracy',
    'exact_action_accuracy',
    'primitive_accuracy',
    'side_accuracy',
    'device_accuracy',
    'wait_condition_accuracy',
    'target_id_accuracy',
    'seen_feature_rate',
)
TASK_FAMILY_METRIC_FIELDS = (
    'task_family',
    'episodes',
    'tasks',
    'task_success',
    'completion_time',
    'command_count',
    'skipped_redundant_event_count',
    'redundant_action_rate',
    'noop_action_rate',
    'effective_action_rate',
    'illegal_proposal_rate',
    'rejected_action_rate',
)
TASK_FAMILY_KEYWORDS = (
    ('loop_entry', ('enter_interior_loop', 'interior loop', 'interior mode')),
    ('transport', ('loaded_', 'payload', 'to_slot', 'to slot', 'clear blocker')),
    ('station_navigation', ('go to staubli', 'go to yaskawa', 'go to kuka', 'center at station')),
    ('stopper', ('stopper', 'stop at a1', 'stop at a2', 'stop at a3', 'stop at a4')),
    ('exterior_loop', ('exterior loop',)),
    ('emergency', ('emergency', 'stop_all')),
)


def _json_dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, sort_keys=True, separators=(',', ':'))


def _pretty_json(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True)


def _iter_jsonl(path: Path):
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


def _safe_int(raw: Any, fallback: int = 0) -> int:
    try:
        return int(raw)
    except (TypeError, ValueError):
        return fallback


def _normalize_text(raw: Any) -> str:
    return ' '.join(str(raw or '').strip().casefold().split())


def _status_text(row: dict[str, Any]) -> str:
    if row.get('task_status'):
        return str(row.get('task_status') or '')
    status = row.get('status')
    if isinstance(status, dict):
        return str(status.get('event_status') or '')
    return ''


def _action_from_row(row: dict[str, Any]) -> dict[str, Any]:
    action = row.get('action')
    if not isinstance(action, dict):
        action = row.get('next_action')
    if not isinstance(action, dict):
        raise ValueError(
            f'row for episode {row.get("episode_id", "")!r} is missing event action'
        )
    return action


def _image_fields(row: dict[str, Any]) -> dict[str, str]:
    images = {
        key.removeprefix('observation.images.'): str(value)
        for key, value in row.items()
        if key.startswith('observation.images.') and value
    }
    if images:
        return images
    refs = row.get('image_frame_refs', {})
    if isinstance(refs, dict):
        return {
            str(camera_name): str(image_ref)
            for camera_name, image_ref in refs.items()
            if image_ref
        }
    model_input = row.get('model_input', {})
    if isinstance(model_input, dict) and isinstance(model_input.get('overhead_images'), dict):
        return {
            str(camera_name): str(image_ref)
            for camera_name, image_ref in model_input['overhead_images'].items()
            if image_ref
        }
    return {}


def _event_row(row: dict[str, Any], fallback_step_index: int) -> dict[str, Any]:
    model_input = row.get('model_input', {})
    if not isinstance(model_input, dict):
        model_input = {}
    return {
        'episode_id': str(row.get('episode_id') or ''),
        'step_index': _safe_int(row.get('step_index', row.get('event_index')), fallback_step_index),
        'task': str(row.get('task') or model_input.get('language') or ''),
        'template': str(row.get('template') or row.get('task_template') or ''),
        'phase': str(row.get('phase') or ''),
        'task_status': _status_text(row),
        'event_type': str(row.get('event_type') or ''),
        'timestamp': row.get('timestamp'),
        'model_input': model_input,
        'observation_state': row.get('observation.state', []),
        'images': _image_fields(row),
        'action': _action_from_row(row),
        'auxiliary_targets': row.get('auxiliary_targets', {}),
        'privileged_eval': row.get('privileged_eval', {}),
        'safety_decoder_metrics': row.get('safety_decoder_metrics', {}),
        'event_generation_metrics': row.get('event_generation_metrics', {}),
        'original_command': row.get('original_command', {}),
    }


def _infer_dataset_root(input_path: Path) -> Path:
    if input_path.is_dir():
        return input_path.expanduser().resolve()
    resolved = input_path.expanduser().resolve()
    if resolved.parent.name == 'meta':
        return resolved.parent.parent
    for parent in resolved.parents:
        if parent.name == 'episodes':
            return parent.parent
    return resolved.parent


def _event_files_from_input(input_path: Path) -> tuple[list[Path], Path, str]:
    input_path = input_path.expanduser()
    if input_path.is_file():
        return [input_path.resolve()], _infer_dataset_root(input_path), str(input_path.resolve())
    dataset_dir = input_path.resolve()
    training_events = dataset_dir / 'meta' / 'training_events.jsonl'
    if training_events.exists():
        return [training_events], dataset_dir, str(training_events)
    event_files = sorted((dataset_dir / 'episodes').glob('episode_*/events.jsonl'))
    return event_files, dataset_dir, 'episodes/*/events.jsonl'


def load_event_rows(input_path: Path) -> tuple[list[dict[str, Any]], Path, str]:
    event_files, dataset_root, source = _event_files_from_input(input_path)
    rows: list[dict[str, Any]] = []
    for event_file in event_files:
        for fallback_step_index, raw_row in enumerate(_iter_jsonl(event_file)):
            row = _event_row(raw_row, fallback_step_index)
            if not row['episode_id']:
                row['episode_id'] = event_file.parent.name
            rows.append(row)
    return rows, dataset_root, source


def _language(row: dict[str, Any]) -> str:
    model_input = row.get('model_input', {})
    language = model_input.get('language') if isinstance(model_input, dict) else ''
    return _normalize_text(language or row.get('task', ''))


def _binary_state(row: dict[str, Any]) -> dict[str, Any]:
    model_input = row.get('model_input', {})
    if (
        isinstance(model_input, dict)
        and (
            'binary_sensor_bits' in model_input
            or 'switch_states' in model_input
            or 'stopper_states' in model_input
        )
    ):
        return {
            'binary_sensor_bits': model_input.get('binary_sensor_bits', {}),
            'switch_states': model_input.get('switch_states', {}),
            'stopper_states': model_input.get('stopper_states', {}),
            'last_command': model_input.get('last_command', {}),
            'shuttle_command_state': model_input.get('shuttle_command_state', {}),
        }
    return {
        'observation_state': row.get('observation_state', []),
    }


def state_only_feature(row: dict[str, Any]) -> str:
    return _json_dumps({
        'language': _language(row),
        'binary_state': _binary_state(row),
    })


def _resolve_image_path(dataset_root: Path, image_ref: str) -> Path:
    image_path = Path(image_ref).expanduser()
    if image_path.is_absolute():
        return image_path
    return dataset_root / image_path


def _bucket_float(value: float, bucket_size: float) -> int:
    if not math.isfinite(value):
        return 0
    return int(round(value / bucket_size))


def _cv2_image_feature(image_path: Path) -> dict[str, Any] | None:
    if cv2 is None:
        return None
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        return None
    height, width = image.shape[:2]
    means = image.mean(axis=(0, 1))
    stds = image.std(axis=(0, 1))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hist = cv2.calcHist([gray], [0], None, [8], [0, 256]).flatten()
    total = float(hist.sum()) or 1.0
    return {
        'shape_bucket': [
            max(1, int(round(width / 32.0))),
            max(1, int(round(height / 32.0))),
        ],
        'mean_bgr_q16': [_bucket_float(float(value), 16.0) for value in means],
        'std_bgr_q16': [_bucket_float(float(value), 16.0) for value in stds],
        'gray_hist_pct': [int(round(float(value) * 100.0 / total)) for value in hist],
    }


def _byte_image_feature(image_path: Path) -> dict[str, Any]:
    data = image_path.read_bytes()
    sample = data[:4096]
    return {
        'size': len(data),
        'sha1_12': hashlib.sha1(sample).hexdigest()[:12],
    }


def image_feature(image_ref: str, dataset_root: Path) -> dict[str, Any]:
    if not image_ref:
        return {'status': 'missing'}
    image_path = _resolve_image_path(dataset_root, image_ref)
    if not image_path.exists() or not image_path.is_file():
        return {'status': 'missing'}
    cv_feature = _cv2_image_feature(image_path)
    if cv_feature is not None:
        return {'status': 'ok', 'kind': 'cv2_color_stats', **cv_feature}
    return {'status': 'ok', 'kind': 'byte_digest', **_byte_image_feature(image_path)}


def vla_feature(row: dict[str, Any], dataset_root: Path) -> str:
    model_input = row.get('model_input', {})
    if not isinstance(model_input, dict):
        model_input = {}
    images = {
        camera_name: image_feature(image_ref, dataset_root)
        for camera_name, image_ref in sorted(row.get('images', {}).items())
    }
    return _json_dumps({
        'language': _language(row),
        'last_command': model_input.get('last_command', {}),
        'overhead_images': images,
    })


def action_key(action: dict[str, Any]) -> str:
    return _json_dumps(action)


def task_family(row: dict[str, Any]) -> str:
    text = _normalize_text(
        ' '.join(
            str(value or '')
            for value in (
                row.get('template'),
                row.get('task'),
                row.get('phase'),
                row.get('event_type'),
            )
        )
    ).replace('-', '_')
    for family, keywords in TASK_FAMILY_KEYWORDS:
        if any(keyword in text for keyword in keywords):
            return family
    return 'other'


def _selected_device_map(action: dict[str, Any]) -> dict[str, dict[str, str]]:
    selected: dict[str, dict[str, str]] = {'switches': {}, 'stoppers': {}}
    for kind, mask_key, value_key in (
        ('switches', 'switch_mask', 'switch_values'),
        ('stoppers', 'stopper_mask', 'stopper_values'),
    ):
        mask = action.get(mask_key, {})
        values = action.get(value_key, {})
        if not isinstance(mask, dict) or not isinstance(values, dict):
            continue
        for name in ('A1', 'A2', 'A3', 'A4'):
            if str(mask.get(name, 0)).lower() not in {'1', '1.0', 'true'}:
                continue
            selected[kind][name] = str(values.get(name, 'UNCHANGED'))
    return selected


def _device_accuracy(eval_rows: list[dict[str, Any]], predictions: list[dict[str, Any]]) -> float:
    if not eval_rows:
        return 0.0
    correct = 0
    for row, prediction in zip(eval_rows, predictions):
        if _selected_device_map(prediction) == _selected_device_map(row['action']):
            correct += 1
    return round(correct / len(eval_rows), 6)


def _safe_float(raw: Any) -> float | None:
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return value


def _terminal_success(row: dict[str, Any]) -> bool | None:
    action = row.get('action', {})
    if not isinstance(action, dict):
        return None
    primitive = str(action.get('primitive') or '').upper()
    reason = str(action.get('reason') or '').lower()
    status = str(row.get('task_status') or '').lower()
    event_type = str(row.get('event_type') or '').lower()
    if primitive != 'DONE' and 'terminal' not in event_type:
        return None
    if reason == 'task_succeeded' or status in {'success', 'succeeded'}:
        return True
    if reason in {'task_failed', 'episode_discarded'} or status in {'failure', 'failed', 'discarded'}:
        return False
    return None


def _latest_runtime_metrics(rows: list[dict[str, Any]]) -> dict[str, Any]:
    latest: dict[str, Any] = {}
    for row in rows:
        merged = {}
        safety = row.get('safety_decoder_metrics')
        event_generation = row.get('event_generation_metrics')
        if isinstance(safety, dict):
            merged.update(safety)
        if isinstance(event_generation, dict):
            merged.update(event_generation)
        if merged:
            latest = merged
    return latest


def _metric_float(metrics: dict[str, Any], key: str) -> float | None:
    if not metrics:
        return None
    if key in metrics:
        value = _safe_float(metrics.get(key))
        if value is not None:
            return round(value, 6)
    return None


def _rate_from_runtime(metrics: dict[str, Any], key: str) -> float | None:
    direct = _metric_float(metrics, key)
    if direct is not None:
        return direct
    total = _safe_float(metrics.get('total_proposed_actions'))
    if key == 'rejected_action_rate':
        if not total:
            return None
        rejected = _safe_float(metrics.get('rejected_actions')) or 0.0
        return round(rejected / total, 6)
    candidates = _safe_float(metrics.get('event_candidate_count'))
    if not candidates:
        return None
    if key == 'redundant_action_rate':
        skipped = _safe_float(metrics.get('skipped_redundant_event_count')) or 0.0
        return round(skipped / candidates, 6)
    if key == 'noop_action_rate':
        noop = _safe_float(metrics.get('noop_action_count')) or 0.0
        return round(noop / candidates, 6)
    if key == 'effective_action_rate':
        recorded = _safe_float(metrics.get('recorded_event_count')) or 0.0
        return round(recorded / candidates, 6)
    return None


def _mean_or_none(values: list[float]) -> float | None:
    if not values:
        return None
    return round(sum(values) / len(values), 6)


def _episode_completion_time(rows: list[dict[str, Any]]) -> float | None:
    timestamps = [
        value
        for value in (_safe_float(row.get('timestamp')) for row in rows)
        if value is not None
    ]
    if len(timestamps) >= 2:
        return round(max(timestamps) - min(timestamps), 6)
    return None


def _aggregate_task_metrics(rows: list[dict[str, Any]], family_name: str = 'all') -> dict[str, Any]:
    episodes: dict[str, list[dict[str, Any]]] = {}
    for index, row in enumerate(rows):
        episode_id = str(row.get('episode_id') or f'row_{index}')
        episodes.setdefault(episode_id, []).append(row)

    terminal_values: list[bool] = []
    completion_times: list[float] = []
    command_counts: list[float] = []
    all_family_rows: list[dict[str, Any]] = []
    for episode_rows in episodes.values():
        all_family_rows.extend(episode_rows)
        terminal = None
        for row in episode_rows:
            maybe_terminal = _terminal_success(row)
            if maybe_terminal is not None:
                terminal = maybe_terminal
        if terminal is not None:
            terminal_values.append(terminal)
        completion_time = _episode_completion_time(episode_rows)
        if completion_time is not None:
            completion_times.append(completion_time)
        command_counts.append(float(len(episode_rows)))

    latest_runtime = _latest_runtime_metrics(all_family_rows)
    success_rate = None if not terminal_values else round(
        sum(1 for value in terminal_values if value) / len(terminal_values),
        6,
    )
    metrics = {
        'task_family': family_name,
        'episodes': len(episodes),
        'tasks': len(terminal_values),
        'task_success': success_rate,
        'completion_time': _mean_or_none(completion_times),
        'command_count': _mean_or_none(command_counts),
        'skipped_redundant_event_count': _metric_float(
            latest_runtime,
            'skipped_redundant_event_count',
        ),
        'redundant_action_rate': _rate_from_runtime(latest_runtime, 'redundant_action_rate'),
        'noop_action_rate': _rate_from_runtime(latest_runtime, 'noop_action_rate'),
        'effective_action_rate': _rate_from_runtime(latest_runtime, 'effective_action_rate'),
        'illegal_proposal_rate': _rate_from_runtime(latest_runtime, 'illegal_proposal_rate'),
        'rejected_action_rate': _rate_from_runtime(latest_runtime, 'rejected_action_rate'),
    }
    return metrics


def task_family_metrics(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = {'all': list(rows)}
    for row in rows:
        grouped.setdefault(task_family(row), []).append(row)
    return [
        _aggregate_task_metrics(grouped[family_name], family_name)
        for family_name in sorted(grouped)
    ]


class MemorizedEventPolicy:
    def __init__(self, feature_fn):
        self.feature_fn = feature_fn
        self._feature_to_counts: dict[str, Counter[str]] = {}
        self._action_by_key: dict[str, dict[str, Any]] = {}
        self._global_counts: Counter[str] = Counter()

    def fit(self, rows: list[dict[str, Any]]) -> None:
        for row in rows:
            key = self.feature_fn(row)
            label = action_key(row['action'])
            self._feature_to_counts.setdefault(key, Counter())[label] += 1
            self._action_by_key[label] = row['action']
            self._global_counts[label] += 1

    def predict(self, row: dict[str, Any]) -> tuple[dict[str, Any], bool]:
        key = self.feature_fn(row)
        counts = self._feature_to_counts.get(key)
        seen = bool(counts)
        if not counts:
            counts = self._global_counts
        if not counts:
            return {}, False
        label = counts.most_common(1)[0][0]
        return self._action_by_key[label], seen


class OraclePolicy:
    def fit(self, _rows: list[dict[str, Any]]) -> None:
        return None

    @staticmethod
    def predict(row: dict[str, Any]) -> tuple[dict[str, Any], bool]:
        return row['action'], True


def _split_rows(
    rows: list[dict[str, Any]],
    *,
    holdout_fraction: float,
    seed: int,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], str]:
    if len(rows) < 2 or holdout_fraction <= 0:
        return rows, rows, 'train_equals_eval'
    rng = random.Random(seed)
    episode_ids = sorted({row.get('episode_id', '') for row in rows if row.get('episode_id')})
    if len(episode_ids) >= 2:
        shuffled = list(episode_ids)
        rng.shuffle(shuffled)
        eval_count = max(1, min(len(shuffled) - 1, int(round(len(shuffled) * holdout_fraction))))
        eval_ids = set(shuffled[:eval_count])
        train_rows = [row for row in rows if row.get('episode_id') not in eval_ids]
        eval_rows = [row for row in rows if row.get('episode_id') in eval_ids]
        return train_rows, eval_rows, 'episode_holdout'

    indices = list(range(len(rows)))
    rng.shuffle(indices)
    eval_count = max(1, min(len(indices) - 1, int(round(len(indices) * holdout_fraction))))
    eval_indices = set(indices[:eval_count])
    train_rows = [row for index, row in enumerate(rows) if index not in eval_indices]
    eval_rows = [row for index, row in enumerate(rows) if index in eval_indices]
    return train_rows, eval_rows, 'row_holdout'


def _field_accuracy(eval_rows: list[dict[str, Any]], predictions: list[dict[str, Any]], field: str) -> float:
    if not eval_rows:
        return 0.0
    correct = 0
    for row, prediction in zip(eval_rows, predictions):
        if prediction.get(field) == row['action'].get(field):
            correct += 1
    return round(correct / len(eval_rows), 6)


def evaluate_policy(
    baseline_name: str,
    policy: Any,
    train_rows: list[dict[str, Any]],
    eval_rows: list[dict[str, Any]],
    *,
    family_name: str = 'all',
) -> dict[str, Any]:
    policy.fit(train_rows)
    predictions: list[dict[str, Any]] = []
    seen = 0
    exact = 0
    for row in eval_rows:
        prediction, feature_seen = policy.predict(row)
        predictions.append(prediction)
        seen += 1 if feature_seen else 0
        exact += 1 if action_key(prediction) == action_key(row['action']) else 0

    eval_count = len(eval_rows)
    action_accuracy = 0.0 if eval_count == 0 else round(exact / eval_count, 6)
    metrics = {
        'baseline': baseline_name,
        'task_family': family_name,
        'train_rows': len(train_rows),
        'eval_rows': eval_count,
        'action_accuracy': action_accuracy,
        'exact_action_accuracy': action_accuracy,
        'primitive_accuracy': _field_accuracy(eval_rows, predictions, 'primitive'),
        'side_accuracy': _field_accuracy(eval_rows, predictions, 'side'),
        'device_accuracy': _device_accuracy(eval_rows, predictions),
        'wait_condition_accuracy': _field_accuracy(eval_rows, predictions, 'wait_condition'),
        'target_id_accuracy': _field_accuracy(eval_rows, predictions, 'target_id'),
        'seen_feature_rate': 0.0 if eval_count == 0 else round(seen / eval_count, 6),
    }
    return metrics


def evaluate_policy_by_family(
    baseline_name: str,
    policy_factory,
    train_rows: list[dict[str, Any]],
    eval_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = {'all': list(eval_rows)}
    for row in eval_rows:
        grouped.setdefault(task_family(row), []).append(row)
    metrics = []
    for family_name in sorted(grouped):
        policy = policy_factory()
        metrics.append(
            evaluate_policy(
                baseline_name,
                policy,
                train_rows,
                grouped[family_name],
                family_name=family_name,
            )
        )
    return metrics


def run_baseline_eval(
    input_path: Path,
    output_dir: Path | None = None,
    *,
    holdout_fraction: float = 0.2,
    seed: int = 13,
) -> dict[str, Any]:
    rows, dataset_root, source = load_event_rows(input_path)
    if not rows:
        raise ValueError(f'no event rows found in {input_path}')

    if output_dir is None:
        output_dir = dataset_root / 'meta' / 'baseline_eval'
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    train_rows, eval_rows, split_mode = _split_rows(
        rows,
        holdout_fraction=holdout_fraction,
        seed=seed,
    )
    baseline_factories = {
        'state_only': lambda: MemorizedEventPolicy(state_only_feature),
        'vla': lambda: MemorizedEventPolicy(lambda row: vla_feature(row, dataset_root)),
        'oracle': OraclePolicy,
    }
    metrics = [
        evaluate_policy(name, factory(), train_rows, eval_rows)
        for name, factory in baseline_factories.items()
    ]
    baseline_family_metrics = [
        row
        for name, factory in baseline_factories.items()
        for row in evaluate_policy_by_family(name, factory, train_rows, eval_rows)
    ]
    family_metrics = task_family_metrics(eval_rows)

    summary = {
        'input_path': str(input_path.expanduser()),
        'dataset_root': str(dataset_root),
        'source': source,
        'output_dir': str(output_dir),
        'rows': len(rows),
        'train_rows': len(train_rows),
        'eval_rows': len(eval_rows),
        'split_mode': split_mode,
        'holdout_fraction': holdout_fraction,
        'seed': seed,
        'baselines': metrics,
        'baseline_task_family_metrics': baseline_family_metrics,
        'task_family_metrics': family_metrics,
        'notes': {
            'state_only': 'language + binary_state -> event action; no overhead images or privileged pose.',
            'vla': (
                'language + overhead_images + previous last_command -> event action; '
                'no binary sensor state or privileged pose.'
            ),
            'oracle': 'privileged replay upper bound; uses the event action generated by the expert/oracle labels.',
        },
    }

    json_path = output_dir / 'baseline_metrics.json'
    csv_path = output_dir / 'baseline_metrics.csv'
    baseline_family_csv_path = output_dir / 'baseline_task_family_metrics.csv'
    task_family_csv_path = output_dir / 'task_family_metrics.csv'
    json_path.write_text(_pretty_json(summary) + '\n', encoding='utf-8')
    with csv_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=METRIC_FIELDS)
        writer.writeheader()
        for row in metrics:
            writer.writerow({field: row.get(field, '') for field in METRIC_FIELDS})
    with baseline_family_csv_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=METRIC_FIELDS)
        writer.writeheader()
        for row in baseline_family_metrics:
            writer.writerow({field: row.get(field, '') for field in METRIC_FIELDS})
    with task_family_csv_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=TASK_FAMILY_METRIC_FIELDS)
        writer.writeheader()
        for row in family_metrics:
            writer.writerow({field: row.get(field, '') for field in TASK_FAMILY_METRIC_FIELDS})
    summary['metrics_json'] = str(json_path)
    summary['metrics_csv'] = str(csv_path)
    summary['baseline_task_family_metrics_csv'] = str(baseline_family_csv_path)
    summary['task_family_metrics_csv'] = str(task_family_csv_path)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Train/evaluate simple Room 315 VLA baselines on event-level labels.'
    )
    parser.add_argument(
        'input',
        type=Path,
        help='Dataset directory, episodes/events.jsonl, or meta/training_events.jsonl.',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=None,
        help='Directory for baseline_metrics.json and baseline_metrics.csv.',
    )
    parser.add_argument(
        '--holdout-fraction',
        type=float,
        default=0.2,
        help='Evaluation holdout fraction. Uses episode holdout when multiple episodes exist.',
    )
    parser.add_argument(
        '--seed',
        type=int,
        default=13,
        help='Deterministic train/eval split seed.',
    )
    args = parser.parse_args()
    summary = run_baseline_eval(
        args.input,
        args.output_dir,
        holdout_fraction=max(0.0, min(float(args.holdout_fraction), 0.95)),
        seed=args.seed,
    )
    print(_pretty_json(summary))


if __name__ == '__main__':
    main()
