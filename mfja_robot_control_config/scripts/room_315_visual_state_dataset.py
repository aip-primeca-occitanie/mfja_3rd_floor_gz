#!/usr/bin/env python3
"""Shared helpers for the Room 315 visual-state dataset mode.

The visual-state mode is deliberately separate from the legacy 24-value
direct-action dataset. Model inputs are camera references only; oracle labels
are loaded from a physically separate JSONL file or from an explicit oracle
label field before split-time sanitisation.
"""

import hashlib
import json
import math
from collections import Counter
from pathlib import Path
from typing import Any

DATASET_MODE_LEGACY_ACTION = 'legacy_action'
DATASET_MODE_VISUAL_STATE = 'visual_state'
VISUAL_STATE_SCHEMA_VERSION = 'room315.visual_state.v1'
VISUAL_LABEL_SUFFIX = '_visual_labels.jsonl'
VISUAL_MODEL_INPUT_KEYS = {'overhead_images'}
IMAGE_KEYS = ('left_rail_rgb', 'right_rail_rgb')
SHUTTLE_LABEL_FIELDS = {
    'bbox',
    'location',
    'visually_available_identity',
    'visible_identity',
    'identity_available',
    'loaded_state',
    'loaded',
    'payload_state',
}
LABEL_LEAKAGE_KEYS = {
    'visual_state_labels',
    'oracle_visual_state',
    'oracle_labels',
    'privileged_eval',
    'bbox',
    'bounding_box',
    'location',
    'loaded_state',
    'payload_state',
    'visually_available_identity',
    'visible_identity',
    'obstacles',
    'switch_states',
    'calibration_version',
}


class VisualStateValidationError(ValueError):
    """Raised when a visual-state row or label violates the schema."""


def pretty_json(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True)


def iter_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.expanduser().open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                parsed = json.loads(text)
            except json.JSONDecodeError as exc:
                raise VisualStateValidationError(
                    f'{path}:{line_number}: invalid JSONL row: {exc}'
                ) from exc
            if not isinstance(parsed, dict):
                raise VisualStateValidationError(f'{path}:{line_number}: row must be an object')
            rows.append(parsed)
    return rows


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, separators=(',', ':')) + '\n')


def rows_fingerprint(rows: list[dict[str, Any]]) -> str:
    digest = hashlib.sha256()
    for row in rows:
        payload = json.dumps(row, ensure_ascii=False, sort_keys=True, separators=(',', ':'))
        digest.update(payload.encode('utf-8'))
        digest.update(b'\n')
    return digest.hexdigest()


def sample_id_for_row(row: dict[str, Any], row_index: int | None = None) -> str:
    sample_id = str(row.get('sample_id') or '').strip()
    if sample_id:
        return sample_id
    episode_id = str(row.get('episode_id') or '').strip()
    step = row.get('step_index', row.get('event_index', row_index))
    if episode_id:
        return f'{episode_id}:step:{step}'
    if row_index is None:
        raise VisualStateValidationError('visual-state row is missing sample_id and episode_id')
    return f'row:{row_index}'


def visual_label_path_for_split(split_file: Path) -> Path:
    split_file = split_file.expanduser()
    return split_file.with_name(f'{split_file.stem}{VISUAL_LABEL_SUFFIX}')


def _finite_float(raw: Any, *, context: str) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise VisualStateValidationError(f'{context} must be a finite number') from exc
    if not math.isfinite(value):
        raise VisualStateValidationError(f'{context} must be finite')
    return value


def _confidence(raw: Any, *, context: str) -> float:
    if raw is None:
        raise VisualStateValidationError(f'{context} is missing confidence')
    value = _finite_float(raw, context=f'{context}.confidence')
    if value < 0.0 or value > 1.0:
        raise VisualStateValidationError(f'{context}.confidence must be in [0, 1]')
    return round(value, 6)


def _bbox(raw: Any, *, context: str) -> list[float]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 4:
        raise VisualStateValidationError(f'{context}.bbox must contain four numbers')
    return [round(_finite_float(value, context=f'{context}.bbox'), 6) for value in raw]


def _clean_text(raw: Any, fallback: str = 'unknown') -> str:
    text = str(raw if raw is not None else '').strip()
    return text if text else fallback


def _loaded_state(raw: Any) -> str:
    if isinstance(raw, bool):
        return 'loaded' if raw else 'empty'
    text = _clean_text(raw).strip().lower()
    if text in {'loaded', 'empty', 'unknown'}:
        return text
    if text in {'true', 'present', 'with_payload', 'payload_present'}:
        return 'loaded'
    if text in {'false', 'absent', 'without_payload', 'unloaded', 'no_payload'}:
        return 'empty'
    raise VisualStateValidationError(f'unsupported loaded_state: {raw!r}')


def _switch_state(raw: Any) -> str:
    text = _clean_text(raw).strip().upper()
    if text in {'INTERIOR', 'EXTERIOR', 'UNKNOWN'}:
        return text.lower()
    if text in {'0', 'EXT'}:
        return 'exterior'
    if text in {'1', 'INT'}:
        return 'interior'
    raise VisualStateValidationError(f'unsupported switch state: {raw!r}')


def _location(raw: Any) -> dict[str, Any]:
    if isinstance(raw, str):
        return {'entity': _clean_text(raw)}
    if not isinstance(raw, dict):
        raise VisualStateValidationError('location must be an object or entity string')
    allowed = ('side', 'slot', 'block', 'station', 'entity', 'status')
    location = {
        key: _clean_text(raw.get(key))
        for key in allowed
        if raw.get(key) not in (None, '')
    }
    if not location:
        raise VisualStateValidationError('location must include side/slot/block/station/entity/status')
    return dict(sorted(location.items()))


def _raw_label_payload(row: dict[str, Any]) -> dict[str, Any]:
    for key in ('visual_state_labels', 'oracle_visual_state', 'labels'):
        value = row.get(key)
        if isinstance(value, dict):
            return value
    privileged_eval = row.get('privileged_eval')
    if isinstance(privileged_eval, dict):
        for key in ('visual_state_labels', 'oracle_visual_state', 'labels'):
            value = privileged_eval.get(key)
            if isinstance(value, dict):
                return value
    if any(key in row for key in ('shuttles', 'switches', 'obstacles')):
        return row
    raise VisualStateValidationError('row is missing visual_state_labels/oracle_visual_state')


def _iter_entities(raw: Any, *, entity_name: str) -> list[dict[str, Any]]:
    if raw is None:
        return []
    if isinstance(raw, dict):
        entities = []
        for entity_id, value in raw.items():
            if isinstance(value, dict):
                item = dict(value)
            else:
                item = {'state': value}
            item.setdefault('id', entity_id)
            entities.append(item)
        return entities
    if isinstance(raw, list):
        if not all(isinstance(item, dict) for item in raw):
            raise VisualStateValidationError(f'{entity_name} entries must be objects')
        return list(raw)
    raise VisualStateValidationError(f'{entity_name} must be a list or mapping')


def normalize_visual_state_labels(row_or_label: dict[str, Any], *, context: str = 'row') -> dict[str, Any]:
    raw = _raw_label_payload(row_or_label)
    schema_version = _clean_text(raw.get('schema_version'), VISUAL_STATE_SCHEMA_VERSION)
    calibration_version = _clean_text(raw.get('calibration_version'))
    confidence = _confidence(raw.get('confidence'), context=context)

    shuttles = []
    for index, shuttle in enumerate(_iter_entities(raw.get('shuttles'), entity_name='shuttles')):
        item_context = f'{context}.shuttles[{index}]'
        identity = _clean_text(
            shuttle.get('visually_available_identity')
            or shuttle.get('visible_identity')
            or shuttle.get('identity')
            or shuttle.get('id')
        )
        shuttles.append({
            'id': _clean_text(shuttle.get('id') or shuttle.get('shuttle_id') or identity),
            'visually_available_identity': identity,
            'identity_available': bool(shuttle.get('identity_available', identity != 'unknown')),
            'bbox': _bbox(shuttle.get('bbox') or shuttle.get('bounding_box'), context=item_context),
            'location': _location(shuttle.get('location')),
            'loaded_state': _loaded_state(
                shuttle.get('loaded_state', shuttle.get('payload_state', shuttle.get('loaded')))
            ),
            'confidence': _confidence(shuttle.get('confidence'), context=item_context),
        })

    switches = []
    raw_switches = raw.get('switches', raw.get('switch_states'))
    for index, switch in enumerate(_iter_entities(raw_switches, entity_name='switches')):
        item_context = f'{context}.switches[{index}]'
        switches.append({
            'id': _clean_text(switch.get('id') or switch.get('switch_id')),
            'state': _switch_state(switch.get('state', switch.get('position'))),
            'confidence': _confidence(switch.get('confidence'), context=item_context),
        })

    obstacles = []
    for index, obstacle in enumerate(_iter_entities(raw.get('obstacles'), entity_name='obstacles')):
        item_context = f'{context}.obstacles[{index}]'
        obstacles.append({
            'id': _clean_text(obstacle.get('id') or obstacle.get('obstacle_id') or f'obstacle_{index}'),
            'bbox': _bbox(obstacle.get('bbox') or obstacle.get('bounding_box'), context=item_context),
            'location': _location(obstacle.get('location', {'status': 'unknown'})),
            'confidence': _confidence(obstacle.get('confidence'), context=item_context),
        })

    shuttles.sort(key=lambda item: (item['id'], item['visually_available_identity']))
    switches.sort(key=lambda item: item['id'])
    obstacles.sort(key=lambda item: item['id'])
    return {
        'schema_version': schema_version,
        'calibration_version': calibration_version,
        'confidence': confidence,
        'shuttles': shuttles,
        'switches': switches,
        'obstacles': obstacles,
    }


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


def validate_visual_model_input(row: dict[str, Any], *, context: str = 'row') -> dict[str, Any]:
    model_input = row.get('model_input')
    if not isinstance(model_input, dict):
        raise VisualStateValidationError(f'{context} is missing model_input')
    unexpected = sorted(set(model_input) - VISUAL_MODEL_INPUT_KEYS)
    if unexpected:
        raise VisualStateValidationError(
            f'{context} visual_state model_input has undeclared fields: {unexpected}'
        )
    leaked = sorted(_walk_keys(model_input) & LABEL_LEAKAGE_KEYS)
    if leaked:
        raise VisualStateValidationError(
            f'{context} visual_state model_input contains oracle/label leakage keys: {leaked}'
        )
    overhead_images = model_input.get('overhead_images')
    if not isinstance(overhead_images, dict):
        raise VisualStateValidationError(f'{context} model_input.overhead_images must be an object')
    return {'overhead_images': dict(overhead_images)}


def visual_model_input_image_refs(row: dict[str, Any]) -> dict[str, str]:
    model_input = validate_visual_model_input(row)
    return {
        str(key): str(value)
        for key, value in model_input['overhead_images'].items()
        if value
    }


def scenario_family_from_row(row: dict[str, Any], label: dict[str, Any] | None = None) -> str:
    for key in ('scenario_family', 'family_id', 'task_family', 'base_case_id'):
        value = str(row.get(key) or '').strip()
        if value:
            return value
    problem = str(row.get('pddl_problem') or '').strip()
    if problem:
        if 'room315-' in problem:
            problem = problem.split('room315-', 1)[1]
        return problem.rsplit('_speed', 1)[0]
    payload = label or _raw_label_payload(row)
    for key in ('scenario_family', 'family_id', 'task_family', 'base_case_id'):
        value = str(payload.get(key) or '').strip()
        if value:
            return value
    raise VisualStateValidationError('visual-state row is missing scenario family')


def sanitized_visual_state_row(row: dict[str, Any], row_index: int) -> tuple[dict[str, Any], dict[str, Any]]:
    sample_id = sample_id_for_row(row, row_index)
    labels = normalize_visual_state_labels(row, context=f'row {row_index}')
    model_row = {
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'sample_id': sample_id,
        'episode_id': str(row.get('episode_id') or ''),
        'step_index': row.get('step_index', row.get('event_index', row_index)),
        'task': str(row.get('task') or ''),
        'scenario_family': scenario_family_from_row(row, labels),
        'model_input': {
            'overhead_images': dict(
                (
                    (row.get('model_input') or {}).get('overhead_images')
                    if isinstance(row.get('model_input'), dict)
                    else {}
                )
                or {}
            ),
        },
    }
    validate_visual_model_input(model_row, context=f'row {row_index}')
    label_row = {
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'sample_id': sample_id,
        'episode_id': model_row['episode_id'],
        'step_index': model_row['step_index'],
        'scenario_family': model_row['scenario_family'],
        'label_source': 'oracle',
        'model_input_exposure': 'excluded',
        'visual_state_labels': labels,
    }
    return model_row, label_row


def load_visual_labels_for_rows(
    rows: list[dict[str, Any]],
    labels_path: Path | None = None,
) -> list[dict[str, Any]]:
    if labels_path is not None:
        label_rows = iter_jsonl(labels_path)
        labels_by_sample: dict[str, dict[str, Any]] = {}
        for index, label_row in enumerate(label_rows):
            sample_id = sample_id_for_row(label_row, index)
            if sample_id in labels_by_sample:
                raise VisualStateValidationError(f'duplicate visual label sample_id: {sample_id}')
            labels_by_sample[sample_id] = label_row
        labels: list[dict[str, Any]] = []
        missing: list[str] = []
        for index, row in enumerate(rows):
            sample_id = sample_id_for_row(row, index)
            label_row = labels_by_sample.get(sample_id)
            if label_row is None:
                missing.append(sample_id)
                continue
            labels.append(normalize_visual_state_labels(label_row, context=f'label {sample_id}'))
        if missing:
            raise VisualStateValidationError(f'missing visual labels for samples: {missing[:5]}')
        return labels
    return [
        normalize_visual_state_labels(row, context=f'row {index}')
        for index, row in enumerate(rows)
    ]


def validate_visual_state_rows(
    rows: list[dict[str, Any]],
    labels_path: Path | None = None,
) -> dict[str, Any]:
    issues: list[dict[str, Any]] = []
    for index, row in enumerate(rows):
        try:
            validate_visual_model_input(row, context=f'row {index}')
        except VisualStateValidationError as exc:
            if len(issues) < 20:
                issues.append({
                    'row_index': index,
                    'sample_id': str(row.get('sample_id') or ''),
                    'reason': str(exc),
                })
    if issues:
        raise VisualStateValidationError(f'visual_state model_input integrity failed: {issues[0]}')
    labels = load_visual_labels_for_rows(rows, labels_path)
    return {
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'rows_checked': len(rows),
        'labels_checked': len(labels),
        'allowed_model_input_fields': sorted(VISUAL_MODEL_INPUT_KEYS),
        'production_feature_source': 'model_input.overhead_images only',
        'oracle_label_source': 'separate_jsonl' if labels_path is not None else 'explicit_oracle_label_field',
        'oracle_labels_physically_separate': labels_path is not None,
        'label_schema_version': VISUAL_STATE_SCHEMA_VERSION,
        'row_level_metadata_used_as_features': [],
    }


def visual_state_class_balance(labels: list[dict[str, Any]]) -> dict[str, Any]:
    loaded = Counter()
    identities = Counter()
    switch_states = Counter()
    obstacle_counts = Counter()
    schema_versions = Counter()
    calibration_versions = Counter()
    for label in labels:
        schema_versions[str(label.get('schema_version') or '')] += 1
        calibration_versions[str(label.get('calibration_version') or '')] += 1
        obstacle_counts[str(len(label.get('obstacles') or []))] += 1
        for shuttle in label.get('shuttles') or []:
            loaded[str(shuttle.get('loaded_state') or 'unknown')] += 1
            identities[str(shuttle.get('visually_available_identity') or 'unknown')] += 1
        for switch in label.get('switches') or []:
            switch_states[str(switch.get('state') or 'unknown')] += 1
    return {
        'labels': len(labels),
        'loaded_state': dict(sorted(loaded.items())),
        'visually_available_identity': dict(sorted(identities.items())),
        'visible_switch_state': dict(sorted(switch_states.items())),
        'obstacle_count': dict(sorted(obstacle_counts.items())),
        'schema_version': dict(sorted(schema_versions.items())),
        'calibration_version': dict(sorted(calibration_versions.items())),
    }


def _flatten(prefix: str, value: Any, output: dict[str, Any]) -> None:
    if isinstance(value, dict):
        for key in sorted(value):
            _flatten(f'{prefix}.{key}' if prefix else str(key), value[key], output)
        return
    if isinstance(value, list):
        for index, child in enumerate(value):
            _flatten(f'{prefix}.{index}' if prefix else str(index), child, output)
        return
    output[prefix] = value


def _to_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _label_values(label: dict[str, Any]) -> dict[str, Any]:
    values: dict[str, Any] = {}
    _flatten('', normalize_visual_state_labels(label), values)
    return values


class VisualStateLabelVectorizer:
    def __init__(self, numeric_keys: list[str], categorical_values: dict[str, list[str]]) -> None:
        self.numeric_keys = list(numeric_keys)
        self.categorical_values = {
            str(key): list(values)
            for key, values in categorical_values.items()
        }

    @classmethod
    def fit(cls, labels: list[dict[str, Any]]) -> 'VisualStateLabelVectorizer':
        numeric_keys: set[str] = set()
        categorical_values: dict[str, set[str]] = {}
        for label in labels:
            for key, value in _label_values(label).items():
                if _to_float(value) is None:
                    categorical_values.setdefault(key, set()).add(str(value).strip().lower())
                else:
                    numeric_keys.add(key)
        return cls(
            sorted(numeric_keys),
            {key: sorted(values) for key, values in sorted(categorical_values.items())},
        )

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> 'VisualStateLabelVectorizer':
        if data.get('kind') != 'room315_visual_state_label_vectorizer':
            raise VisualStateValidationError('visual label vectorizer JSON has unexpected kind')
        numeric_keys = data.get('numeric_keys')
        categorical_values = data.get('categorical_values')
        if not isinstance(numeric_keys, list) or not isinstance(categorical_values, dict):
            raise VisualStateValidationError('visual label vectorizer is missing numeric/categorical keys')
        return cls(
            [str(key) for key in numeric_keys],
            {
                str(key): [str(value) for value in values]
                for key, values in categorical_values.items()
                if isinstance(values, list)
            },
        )

    @property
    def names(self) -> list[str]:
        names = list(self.numeric_keys)
        for key, values in self.categorical_values.items():
            names.extend(f'{key}=={value}' for value in values)
        return names

    @property
    def dim(self) -> int:
        return len(self.names)

    def transform(self, label: dict[str, Any]) -> list[float]:
        values = _label_values(label)
        vector: list[float] = []
        for key in self.numeric_keys:
            vector.append(_to_float(values.get(key)) or 0.0)
        for key, allowed_values in self.categorical_values.items():
            raw = str(values.get(key, '')).strip().lower()
            vector.extend(1.0 if raw == allowed else 0.0 for allowed in allowed_values)
        return vector

    def to_json(self) -> dict[str, Any]:
        return {
            'kind': 'room315_visual_state_label_vectorizer',
            'dataset_mode': DATASET_MODE_VISUAL_STATE,
            'schema_version': VISUAL_STATE_SCHEMA_VERSION,
            'numeric_keys': self.numeric_keys,
            'categorical_values': self.categorical_values,
            'names': self.names,
            'dim': self.dim,
            'output_semantics': 'visual_state_labels_not_rail_commands',
        }


def visual_target_stats(
    labels: list[dict[str, Any]],
    vectorizer: VisualStateLabelVectorizer,
) -> tuple[list[float], list[float]]:
    if not labels:
        raise VisualStateValidationError('cannot compute visual target stats from empty labels')
    columns = list(zip(*(vectorizer.transform(label) for label in labels)))
    mean = [sum(column) / len(column) for column in columns]
    std = []
    for column, avg in zip(columns, mean):
        variance = sum((value - avg) ** 2 for value in column) / len(column)
        sigma = math.sqrt(variance)
        std.append(sigma if sigma >= 1e-6 else 1.0)
    return mean, std


def _mae(records: list[dict[str, Any]], indexes: list[int]) -> float | None:
    if not records or not indexes:
        return None
    total = 0.0
    count = 0
    for record in records:
        true = record['true_raw']
        pred = record['pred_raw']
        for index in indexes:
            total += abs(float(pred[index]) - float(true[index]))
            count += 1
    return round(total / max(1, count), 6)


def _binary_accuracy(records: list[dict[str, Any]], indexes: list[int]) -> float | None:
    if not records or not indexes:
        return None
    correct = 0
    total = 0
    for record in records:
        true = record['true_raw']
        pred = record['pred_raw']
        for index in indexes:
            correct += int((float(pred[index]) >= 0.5) == (float(true[index]) >= 0.5))
            total += 1
    return round(correct / max(1, total), 6)


def _categorical_group_accuracy(
    records: list[dict[str, Any]],
    names: list[str],
    token: str,
) -> float | None:
    groups: dict[str, list[int]] = {}
    for index, name in enumerate(names):
        if token in name and '==' in name:
            groups.setdefault(name.split('==', 1)[0], []).append(index)
    if not records or not groups:
        return None
    correct = 0
    total = 0
    for record in records:
        true = record['true_raw']
        pred = record['pred_raw']
        for indexes in groups.values():
            true_index = max(indexes, key=lambda idx: float(true[idx]))
            pred_index = max(indexes, key=lambda idx: float(pred[idx]))
            correct += int(true_index == pred_index)
            total += 1
    return round(correct / max(1, total), 6)


def visual_state_metrics(records: list[dict[str, Any]], label_names: list[str]) -> dict[str, Any]:
    if not records:
        raise VisualStateValidationError('cannot summarise empty visual-state predictions')
    all_indexes = list(range(len(label_names)))
    bbox_indexes = [idx for idx, name in enumerate(label_names) if '.bbox.' in name]
    confidence_indexes = [idx for idx, name in enumerate(label_names) if name.endswith('.confidence') or name == 'confidence']
    obstacle_presence = [
        idx
        for idx, name in enumerate(label_names)
        if name.startswith('obstacles.') and name.endswith('.confidence')
    ]
    return {
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'samples': len(records),
        'label_mae': _mae(records, all_indexes),
        'bbox_mae': _mae(records, bbox_indexes),
        'confidence_mae': _mae(records, confidence_indexes),
        'identity_accuracy': _categorical_group_accuracy(
            records,
            label_names,
            '.visually_available_identity',
        ),
        'loaded_state_accuracy': _categorical_group_accuracy(
            records,
            label_names,
            '.loaded_state',
        ),
        'switch_state_accuracy': _categorical_group_accuracy(records, label_names, '.state'),
        'location_accuracy': _categorical_group_accuracy(records, label_names, '.location.'),
        'obstacle_presence_accuracy': _binary_accuracy(records, obstacle_presence),
    }
