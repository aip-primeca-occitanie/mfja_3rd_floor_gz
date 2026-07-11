#!/usr/bin/env python3

import argparse
import hashlib
import json
import math
import os
import re
import shutil
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from PIL import Image


def _env_path(name: str, fallback: str) -> Path:
    return Path(os.environ.get(name, fallback))


DEFAULT_DATASET_ROOT = _env_path('ROOM315_VLA_DATASET_ROOT', 'room315_payload_dataset')
DEFAULT_SPLITS_DIR = _env_path('ROOM315_VLA_SPLITS_DIR', 'room315_local_training/splits')
DEFAULT_OUTPUT_ROOT = _env_path('ROOM315_LEROBOT_OUTPUT_ROOT', 'room315_local_training/lerobot')
DEFAULT_ACTION_SPACE = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'room_315_vla'
    / 'action_space.yaml'
)
DEFAULT_STATE_VECTORIZER = DEFAULT_OUTPUT_ROOT / 'room315_state_vectorizer.json'
DEFAULT_COMPACT_STATE_VECTORIZER = DEFAULT_OUTPUT_ROOT / 'room315_state_vectorizer_compact32.json'
IMAGE_KEYS = ('left_rail_rgb', 'right_rail_rgb')
SIDES = ('right', 'left')
SLOTS = ('A1', 'A2', 'A3', 'A4')
MODEL_INPUT_KEYS = {'language', 'overhead_images', 'last_command', 'observable_state'}
COMPACT32_STATE_NAMES = (
    'task.side_left',
    'task.target_slot_norm',
    'language.payload_hint',
    'last.command_present',
    'right.slot.A1.occupied',
    'right.slot.A2.occupied',
    'right.slot.A3.occupied',
    'right.slot.A4.occupied',
    'left.slot.A1.occupied',
    'left.slot.A2.occupied',
    'left.slot.A3.occupied',
    'left.slot.A4.occupied',
    'right.route.A1.occupied',
    'right.route.A2.occupied',
    'right.route.A3.occupied',
    'right.route.A4.occupied',
    'left.route.A1.occupied',
    'left.route.A2.occupied',
    'left.route.A3.occupied',
    'left.route.A4.occupied',
    'right.switch.A1.position',
    'right.switch.A2.position',
    'right.switch.A3.position',
    'right.switch.A4.position',
    'left.switch.A1.position',
    'left.switch.A2.position',
    'left.switch.A3.position',
    'left.switch.A4.position',
    'last.primitive_norm',
    'last.side_left',
    'last.target_norm',
    'last.speed_norm',
)


PRIMITIVE_IDS = {
    'WAIT': 0,
    'DONE': 1,
    'SET_SWITCHES': 2,
    'SET_STOPPERS': 3,
    'SHUTTLE_ON': 4,
    'STOP_NOW': 5,
    'EMERGENCY_STOP': 6,
}


def _pretty_json(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True)


def _iter_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                parsed = json.loads(text)
            except json.JSONDecodeError as exc:
                raise ValueError(f'{path}:{line_number}: invalid JSONL row: {exc}') from exc
            if not isinstance(parsed, dict):
                raise ValueError(f'{path}:{line_number}: JSONL row must be an object')
            rows.append(parsed)
    return rows


def _file_fingerprint(path: Path) -> dict[str, Any]:
    digest = hashlib.sha256()
    size = 0
    lines = 0
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
            size += len(chunk)
            lines += chunk.count(b'\n')
    return {
        'path': str(path),
        'sha256': digest.hexdigest(),
        'bytes': size,
        'newline_count': lines,
    }


def _rows_fingerprint(rows: list[dict[str, Any]]) -> str:
    digest = hashlib.sha256()
    for row in rows:
        payload = json.dumps(row, ensure_ascii=False, sort_keys=True, separators=(',', ':'))
        digest.update(payload.encode('utf-8'))
        digest.update(b'\n')
    return digest.hexdigest()


def _resolve_input_path(input_path: Path) -> Path:
    input_path = input_path.expanduser()
    if input_path.is_dir():
        candidate = input_path / 'meta' / 'training_events.jsonl'
        if candidate.exists():
            return candidate.resolve()
        raise FileNotFoundError(f'dataset directory has no meta/training_events.jsonl: {input_path}')
    if not input_path.exists():
        raise FileNotFoundError(f'input JSONL not found: {input_path}')
    return input_path.resolve()


def _episode_number(episode_id: str) -> int:
    match = re.search(r'episode_(\d+)', str(episode_id or ''))
    return int(match.group(1)) if match else 0


def _safe_int(raw: Any, fallback: int = 0) -> int:
    try:
        return int(raw)
    except (TypeError, ValueError):
        return fallback


def _model_input(row: dict[str, Any], *, context: str = 'row') -> dict[str, Any]:
    model_input = row.get('model_input')
    if not isinstance(model_input, dict):
        raise ValueError(f'{context} is missing model_input')
    unexpected = sorted(set(model_input) - MODEL_INPUT_KEYS)
    if unexpected:
        raise ValueError(f'{context} model_input has undeclared fields: {unexpected}')
    missing = sorted(MODEL_INPUT_KEYS - set(model_input))
    if missing:
        raise ValueError(f'{context} model_input is missing fields: {missing}')
    if not isinstance(model_input.get('overhead_images'), dict):
        raise ValueError(f'{context} model_input.overhead_images must be an object')
    if not isinstance(model_input.get('observable_state'), dict):
        raise ValueError(f'{context} model_input.observable_state must be an object')
    if not isinstance(model_input.get('last_command'), dict):
        raise ValueError(f'{context} model_input.last_command must be an object')
    return model_input


def _language(row: dict[str, Any]) -> str:
    language = str(_model_input(row).get('language') or '')
    if not language:
        raise ValueError(f'row for episode {row.get("episode_id", "")!r} has empty model_input.language')
    return language


def _flatten(prefix: str, value: Any, output: dict[str, Any]) -> None:
    if isinstance(value, dict):
        for key in sorted(value):
            child_prefix = f'{prefix}.{key}' if prefix else str(key)
            _flatten(child_prefix, value[key], output)
        return
    if isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            child_prefix = f'{prefix}.{index}' if prefix else str(index)
            _flatten(child_prefix, child, output)
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


def _model_input_state(row: dict[str, Any]) -> dict[str, Any]:
    model_input = _model_input(row)
    state: dict[str, Any] = {}
    _flatten('observable_state', model_input.get('observable_state', {}), state)
    _flatten('last_command', model_input.get('last_command', {}), state)
    return state


class Room315StateVectorizer:
    def __init__(
        self,
        numeric_keys: list[str],
        categorical_values: dict[str, list[str]],
    ) -> None:
        self.numeric_keys = list(numeric_keys)
        self.categorical_values = {
            str(key): list(values)
            for key, values in categorical_values.items()
        }

    @classmethod
    def fit(cls, rows: list[dict[str, Any]]) -> 'Room315StateVectorizer':
        numeric_keys: set[str] = set()
        categorical_values: dict[str, set[str]] = {}
        for row in rows:
            for key, value in _model_input_state(row).items():
                if _to_float(value) is None:
                    categorical_values.setdefault(key, set()).add(str(value).strip().lower())
                else:
                    numeric_keys.add(key)
        return cls(
            sorted(numeric_keys),
            {key: sorted(values) for key, values in sorted(categorical_values.items())},
        )

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> 'Room315StateVectorizer':
        numeric_keys = data.get('numeric_keys')
        categorical_values = data.get('categorical_values')
        if not isinstance(numeric_keys, list) or not isinstance(categorical_values, dict):
            raise ValueError('state vectorizer JSON is missing numeric_keys/categorical_values')
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

    def transform(self, row: dict[str, Any]) -> np.ndarray:
        values = _model_input_state(row)
        vector: list[float] = []
        for key in self.numeric_keys:
            vector.append(_to_float(values.get(key)) or 0.0)
        for key, allowed_values in self.categorical_values.items():
            raw = str(values.get(key, '')).strip().lower()
            vector.extend(1.0 if raw == allowed else 0.0 for allowed in allowed_values)
        return np.asarray(vector, dtype=np.float32)

    def to_json(self) -> dict[str, Any]:
        return {
            'kind': 'room315_state_vectorizer',
            'state_mode': 'full',
            'numeric_keys': self.numeric_keys,
            'categorical_values': self.categorical_values,
            'names': self.names,
            'dim': self.dim,
        }


def _nested(data: dict[str, Any], *keys: str) -> Any:
    value: Any = data
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _sensor_value(observable_state: dict[str, Any], side: str, sensor_name: str) -> float:
    return 1.0 if _to_float(_nested(observable_state, side, 'sensors', sensor_name)) else 0.0


def _route_sensor_names(side: str, slot_index: int) -> tuple[str, str, str]:
    suffix = 'R' if side == 'right' else 'L'
    branch = 'E' if side == 'right' else 'E'
    return (
        f'DA{slot_index}{suffix}',
        f'DA{slot_index}{branch}{suffix}',
        f'DA{slot_index}I{suffix}',
    )


def _route_occupied(observable_state: dict[str, Any], side: str, slot_index: int) -> float:
    return 1.0 if any(
        _sensor_value(observable_state, side, sensor_name)
        for sensor_name in _route_sensor_names(side, slot_index)
    ) else 0.0


def _switch_position(value: Any) -> float:
    text = str(value or '').strip().upper()
    if text == 'INTERIOR':
        return 1.0
    if text == 'EXTERIOR':
        return -1.0
    return 0.0


def _side_left(value: Any) -> float:
    text = str(value or '').strip().lower()
    if text == 'left':
        return 1.0
    if text == 'right':
        return 0.0
    return -1.0


def _target_slot(row: dict[str, Any]) -> int:
    text = _language(row).lower()
    match = re.search(r'\bslot[_\s-]*(\d+)\b', text)
    if match:
        return max(1, min(4, _safe_int(match.group(1), 0)))
    match = re.search(r'\b(?:to|target)[_\s-]*a(\d)\b', text)
    if match:
        return max(1, min(4, _safe_int(match.group(1), 0)))
    return 0


def _task_side(row: dict[str, Any]) -> str:
    text = _language(row).lower()
    if re.search(r'\bleft\b', text):
        return 'left'
    if re.search(r'\bright\b', text):
        return 'right'
    return ''


def _payload_hint_from_language(row: dict[str, Any]) -> float:
    text = _language(row).lower()
    if any(token in text for token in ('loaded', 'with payload', 'carrying', 'with a box')):
        return 1.0
    if any(token in text for token in ('empty', 'without payload', 'no payload', 'unloaded')):
        return 0.0
    return -1.0


def _last_command_present(last_command: dict[str, Any]) -> float:
    action = str(last_command.get('primitive') or last_command.get('action') or '').strip()
    return 1.0 if action else 0.0


def _normalise_target_id(value: Any) -> float:
    text = str(value or '').strip()
    if not text:
        return 0.0
    slot_match = re.fullmatch(r'A([1-4])', text.upper())
    if slot_match:
        return _safe_int(slot_match.group(1), 0) / 4.0
    sensor_match = re.fullmatch(r'DZI([1-4])[RL]', text.upper())
    if sensor_match:
        return _safe_int(sensor_match.group(1), 0) / 4.0
    shuttle_match = re.fullmatch(r'(?:right|left)_shuttle_([1-4])', text.lower())
    if shuttle_match:
        return _safe_int(shuttle_match.group(1), 0) / 4.0
    if text.lower() in {'terminal', 'all_switches', 'all_stoppers', 'multiple_devices'}:
        return 1.0
    return 0.0


class Room315CompactStateVectorizer:
    @classmethod
    def fit(cls, rows: list[dict[str, Any]]) -> 'Room315CompactStateVectorizer':
        del rows
        return cls()

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> 'Room315CompactStateVectorizer':
        names = data.get('names')
        if names is not None and list(names) != list(COMPACT32_STATE_NAMES):
            raise ValueError('compact32 state vectorizer JSON has unexpected feature names')
        return cls()

    @property
    def names(self) -> list[str]:
        return list(COMPACT32_STATE_NAMES)

    @property
    def dim(self) -> int:
        return len(COMPACT32_STATE_NAMES)

    def transform(self, row: dict[str, Any]) -> np.ndarray:
        model_input = row.get('model_input')
        observable_state = (
            model_input.get('observable_state', {})
            if isinstance(model_input, dict) else {}
        )
        last_command = (
            model_input.get('last_command', {})
            if isinstance(model_input, dict) else {}
        )
        target_slot = _target_slot(row)
        values: list[float] = [
            _side_left(_task_side(row)),
            target_slot / 4.0 if target_slot else 0.0,
            _payload_hint_from_language(row),
            _last_command_present(last_command),
        ]
        for side in SIDES:
            suffix = 'R' if side == 'right' else 'L'
            for slot_index in range(1, 5):
                values.append(_sensor_value(observable_state, side, f'DZI{slot_index}{suffix}'))
        for side in SIDES:
            for slot_index in range(1, 5):
                values.append(_route_occupied(observable_state, side, slot_index))
        for side in SIDES:
            for slot in SLOTS:
                values.append(_switch_position(_nested(observable_state, side, 'switches', slot)))

        primitive = str(last_command.get('primitive') or last_command.get('action') or '').upper()
        primitive_id = PRIMITIVE_IDS.get(primitive, 0)
        speed = _to_float(last_command.get('speed_mps')) or 0.0
        values.extend([
            primitive_id / max(PRIMITIVE_IDS.values()),
            _side_left(last_command.get('side')),
            _normalise_target_id(last_command.get('target_id')),
            min(max(speed, 0.0), 0.1) / 0.1,
        ])
        if len(values) != len(COMPACT32_STATE_NAMES):
            raise AssertionError(
                f'compact32 state produced {len(values)} values, '
                f'expected {len(COMPACT32_STATE_NAMES)}'
            )
        return np.asarray(values, dtype=np.float32)

    def to_json(self) -> dict[str, Any]:
        return {
            'kind': 'room315_compact_state_vectorizer',
            'state_mode': 'compact32',
            'names': self.names,
            'dim': self.dim,
            'description': (
                'Fixed 32-value Room 315 state for the SmolVLA direct-action baseline. '
                'All production features are derived only from model_input: language, '
                'observable_state, and last_command.'
            ),
        }


StateVectorizer = Room315StateVectorizer | Room315CompactStateVectorizer


def fit_state_vectorizer(rows: list[dict[str, Any]], state_mode: str) -> StateVectorizer:
    if state_mode == 'full':
        return Room315StateVectorizer.fit(rows)
    if state_mode == 'compact32':
        return Room315CompactStateVectorizer.fit(rows)
    raise ValueError(f'unknown state mode: {state_mode}')


def load_state_vectorizer(path: Path) -> StateVectorizer:
    with path.expanduser().open('r', encoding='utf-8') as stream:
        parsed = json.load(stream)
    if not isinstance(parsed, dict):
        raise ValueError(f'state vectorizer must be a JSON object: {path}')
    if parsed.get('kind') == 'room315_compact_state_vectorizer' or parsed.get('state_mode') == 'compact32':
        return Room315CompactStateVectorizer.from_json(parsed)
    return Room315StateVectorizer.from_json(parsed)


def save_state_vectorizer(vectorizer: StateVectorizer, path: Path) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(_pretty_json(vectorizer.to_json()) + '\n', encoding='utf-8')


def action_vector(row: dict[str, Any], *, expected_dim: int) -> np.ndarray:
    raw = row.get('action_vector')
    if not isinstance(raw, list):
        raise ValueError(f'row for episode {row.get("episode_id", "")!r} is missing action_vector')
    if len(raw) != expected_dim:
        raise ValueError(
            f'row for episode {row.get("episode_id", "")!r} has action_vector length '
            f'{len(raw)}, expected {expected_dim}'
        )
    return np.asarray([float(value) for value in raw], dtype=np.float32)


def load_action_vector_fields(path: Path = DEFAULT_ACTION_SPACE) -> list[str]:
    with path.expanduser().open('r', encoding='utf-8') as stream:
        parsed = yaml.safe_load(stream)
    fields = parsed.get('action_vector_fields') if isinstance(parsed, dict) else None
    if not isinstance(fields, list) or not fields:
        raise ValueError(f'action_space YAML has no action_vector_fields: {path}')
    return [str(field) for field in fields]


def build_lerobot_features(
    *,
    state_names: list[str],
    action_names: list[str],
    image_height: int,
    image_width: int,
    use_videos: bool,
) -> dict[str, dict[str, Any]]:
    image_dtype = 'video' if use_videos else 'image'
    features: dict[str, dict[str, Any]] = {
        'observation.state': {
            'dtype': 'float32',
            'shape': (len(state_names),),
            'names': state_names,
        },
        'action': {
            'dtype': 'float32',
            'shape': (len(action_names),),
            'names': action_names,
        },
    }
    for image_key in IMAGE_KEYS:
        features[f'observation.images.{image_key}'] = {
            'dtype': image_dtype,
            'shape': (image_height, image_width, 3),
            'names': ['height', 'width', 'channels'],
        }
    return features


def _image_refs(row: dict[str, Any]) -> dict[str, str]:
    overhead_images = _model_input(row).get('overhead_images', {})
    return {
        str(key): str(value)
        for key, value in overhead_images.items()
        if value
    }


def _resolve_image_path(dataset_root: Path, image_ref: str) -> Path:
    image_path = Path(str(image_ref)).expanduser()
    return image_path if image_path.is_absolute() else dataset_root / image_path


def _blank_image(*, image_width: int, image_height: int) -> Image.Image:
    return Image.new('RGB', (image_width, image_height), color=(0, 0, 0))


def _missing_image_error(row: dict[str, Any], image_key: str, reason: str, ref: str = '') -> str:
    episode_id = str(row.get('episode_id') or '')
    detail = f' ({ref})' if ref else ''
    return f'episode {episode_id!r} image {image_key!r} is {reason}{detail}'


def load_image(
    row: dict[str, Any],
    dataset_root: Path,
    image_key: str,
    *,
    image_width: int,
    image_height: int,
    allow_blank_images: bool = False,
) -> Image.Image:
    image_ref = _image_refs(row).get(image_key, '')
    if not image_ref:
        if allow_blank_images:
            return _blank_image(image_width=image_width, image_height=image_height)
        raise FileNotFoundError(_missing_image_error(row, image_key, 'missing from model_input.overhead_images'))
    image_path = _resolve_image_path(dataset_root, image_ref)
    if not image_path.exists():
        if allow_blank_images:
            return _blank_image(image_width=image_width, image_height=image_height)
        raise FileNotFoundError(_missing_image_error(row, image_key, 'missing on disk', image_ref))
    try:
        with Image.open(image_path) as image:
            rgb = image.convert('RGB')
            if rgb.size != (image_width, image_height):
                rgb = rgb.resize((image_width, image_height), Image.BILINEAR)
            return rgb.copy()
    except Exception as exc:
        if allow_blank_images:
            return _blank_image(image_width=image_width, image_height=image_height)
        raise RuntimeError(_missing_image_error(row, image_key, 'unreadable', image_ref)) from exc


def image_integrity_report(
    rows: list[dict[str, Any]],
    dataset_root: Path,
    *,
    allow_blank_images: bool = False,
) -> dict[str, Any]:
    per_camera = {
        camera: {
            'referenced_rows': 0,
            'missing_ref_rows': 0,
            'existing_files': 0,
            'missing_files': 0,
            'unreadable_files': 0,
            'blank_substitutions': 0,
        }
        for camera in IMAGE_KEYS
    }
    complete_rows = 0
    problems: list[dict[str, Any]] = []
    for row_index, row in enumerate(rows):
        refs = _image_refs(row)
        row_complete = True
        for camera in IMAGE_KEYS:
            ref = refs.get(camera, '')
            stats = per_camera[camera]
            if not ref:
                stats['missing_ref_rows'] += 1
                row_complete = False
                if allow_blank_images:
                    stats['blank_substitutions'] += 1
                if len(problems) < 20:
                    problems.append({
                        'row_index': row_index,
                        'episode_id': str(row.get('episode_id') or ''),
                        'camera': camera,
                        'reason': 'missing_ref',
                    })
                continue
            stats['referenced_rows'] += 1
            image_path = _resolve_image_path(dataset_root, ref)
            if not image_path.exists():
                stats['missing_files'] += 1
                row_complete = False
                if allow_blank_images:
                    stats['blank_substitutions'] += 1
                if len(problems) < 20:
                    problems.append({
                        'row_index': row_index,
                        'episode_id': str(row.get('episode_id') or ''),
                        'camera': camera,
                        'reason': 'missing_file',
                        'ref': ref,
                    })
                continue
            try:
                with Image.open(image_path) as image:
                    image.verify()
                stats['existing_files'] += 1
            except Exception:
                stats['unreadable_files'] += 1
                row_complete = False
                if allow_blank_images:
                    stats['blank_substitutions'] += 1
                if len(problems) < 20:
                    problems.append({
                        'row_index': row_index,
                        'episode_id': str(row.get('episode_id') or ''),
                        'camera': camera,
                        'reason': 'unreadable_file',
                        'ref': ref,
                    })
        if row_complete:
            complete_rows += 1
    problem_count = sum(
        stats['missing_ref_rows'] + stats['missing_files'] + stats['unreadable_files']
        for stats in per_camera.values()
    )
    if problem_count and not allow_blank_images:
        first = problems[0] if problems else {}
        raise FileNotFoundError(f'image integrity check failed before conversion: {first}')
    total = len(rows)
    return {
        'required_cameras': list(IMAGE_KEYS),
        'total_rows': total,
        'complete_rows': complete_rows,
        'complete_row_rate': round(complete_rows / max(1, total), 6),
        'allow_blank_images': bool(allow_blank_images),
        'debug_blank_image_mode': bool(allow_blank_images),
        'per_camera': per_camera,
        'problem_examples': problems,
    }


def group_rows_by_episode(rows: list[dict[str, Any]]) -> list[tuple[str, list[dict[str, Any]]]]:
    grouped: dict[str, list[tuple[int, dict[str, Any]]]] = {}
    for row_index, row in enumerate(rows):
        episode_id = str(row.get('episode_id') or '').strip()
        if not episode_id:
            raise ValueError(f'row {row_index} is missing episode_id')
        grouped.setdefault(episode_id, []).append((row_index, row))
    return [
        (
            episode_id,
            [
                row
                for _, row in sorted(
                    items,
                    key=lambda item: (
                        _safe_int(item[1].get('step_index', item[1].get('event_index')), item[0]),
                        item[0],
                    ),
                )
            ],
        )
        for episode_id, items in sorted(
            grouped.items(),
            key=lambda item: (_episode_number(item[0]), item[0]),
        )
    ]


def validate_model_input_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    issues: list[dict[str, Any]] = []
    for row_index, row in enumerate(rows):
        try:
            _model_input(row, context=f'row {row_index}')
            _language(row)
        except ValueError as exc:
            if len(issues) < 20:
                issues.append({
                    'row_index': row_index,
                    'episode_id': str(row.get('episode_id') or ''),
                    'reason': str(exc),
                })
    if issues:
        raise ValueError(f'model_input integrity check failed: {issues[0]}')
    return {
        'rows_checked': len(rows),
        'allowed_model_input_fields': sorted(MODEL_INPUT_KEYS),
        'production_feature_source': 'model_input only',
        'row_level_metadata_excluded_from_features': [
            'pddl_goal',
            'pddl_problem',
            'payload_present',
            'payload_condition',
            'step_index',
            'event_index',
        ],
    }


def _rounded_key(value: Any) -> str:
    try:
        return f'{float(value):.4g}'
    except (TypeError, ValueError):
        return 'invalid'


def class_balance_report(rows: list[dict[str, Any]], *, expected_dim: int) -> dict[str, Any]:
    primitive = Counter()
    side = Counter()
    shuttle = Counter()
    target = Counter()
    speed = Counter()
    invalid_rows = 0
    for row in rows:
        vector = row.get('action_vector')
        if not isinstance(vector, list) or len(vector) != expected_dim:
            invalid_rows += 1
            continue
        try:
            primitive[str(int(round(float(vector[0]))))] += 1
            side[str(int(round(float(vector[1]))))] += 1
            shuttle[str(int(round(float(vector[2]))))] += 1
            speed[_rounded_key(vector[19])] += 1
            target[str(int(round(float(vector[21]))))] += 1
        except (TypeError, ValueError):
            invalid_rows += 1
    return {
        'rows': len(rows),
        'invalid_action_vector_rows': invalid_rows,
        'primitive_id': dict(sorted(primitive.items())),
        'side_id': dict(sorted(side.items())),
        'shuttle_index': dict(sorted(shuttle.items())),
        'target_id': dict(sorted(target.items())),
        'speed_mps': dict(sorted(speed.items())),
    }


def _require_lerobot_dataset():
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        raise SystemExit(
            'LeRobot is required for conversion but is not installed in this Python environment.\n'
            'Activate your training environment and install it first:\n'
            '  python -m pip install lerobot'
        ) from exc
    return LeRobotDataset


def _conversion_root(output_root: Path, name: str) -> Path:
    return output_root.expanduser().resolve() / name


def _prepare_output_root(root: Path, *, overwrite: bool) -> None:
    if root.exists():
        if not overwrite:
            raise FileExistsError(f'output dataset already exists; pass --overwrite: {root}')
        shutil.rmtree(root)
    root.parent.mkdir(parents=True, exist_ok=True)


def convert_room315_to_lerobot(
    input_path: Path,
    *,
    dataset_root: Path,
    output_root: Path,
    name: str,
    repo_id: str,
    state_mode: str,
    state_vectorizer_path: Path | None,
    state_vectorizer_out: Path,
    image_width: int,
    image_height: int,
    fps: int,
    max_episodes: int | None = None,
    max_rows: int | None = None,
    use_videos: bool = False,
    allow_blank_images: bool = False,
    overwrite: bool = False,
) -> dict[str, Any]:
    LeRobotDataset = _require_lerobot_dataset()
    input_file = _resolve_input_path(input_path)
    rows = _iter_jsonl(input_file)
    if max_rows is not None and max_rows > 0:
        rows = rows[:max_rows]
    episodes = group_rows_by_episode(rows)
    if max_episodes is not None and max_episodes > 0:
        allowed = {episode_id for episode_id, _ in episodes[:max_episodes]}
        rows = [row for row in rows if str(row.get('episode_id') or '') in allowed]
        episodes = group_rows_by_episode(rows)
    if not rows:
        raise ValueError(f'no rows selected from {input_file}')

    model_input_integrity = validate_model_input_rows(rows)
    image_integrity = image_integrity_report(
        rows,
        dataset_root,
        allow_blank_images=allow_blank_images,
    )

    if state_vectorizer_path is not None:
        vectorizer = load_state_vectorizer(state_vectorizer_path)
        fitted_state_vectorizer = False
    else:
        vectorizer = fit_state_vectorizer(rows, state_mode)
        fitted_state_vectorizer = True

    state_mode = str(vectorizer.to_json().get('state_mode') or state_mode)
    action_names = load_action_vector_fields()
    features = build_lerobot_features(
        state_names=vectorizer.names,
        action_names=action_names,
        image_height=image_height,
        image_width=image_width,
        use_videos=use_videos,
    )
    root = _conversion_root(output_root, name)
    _prepare_output_root(root, overwrite=overwrite)

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        features=features,
        root=root,
        robot_type='room315_rail_cell',
        use_videos=use_videos,
        image_writer_threads=4,
    )
    frame_count = 0
    task_counts: Counter[str] = Counter()
    for episode_id, episode_rows in episodes:
        for row in episode_rows:
            task = _language(row)
            task_counts[task] += 1
            frame = {
                'task': task,
                'observation.state': vectorizer.transform(row),
                'action': action_vector(row, expected_dim=len(action_names)),
            }
            for image_key in IMAGE_KEYS:
                frame[f'observation.images.{image_key}'] = load_image(
                    row,
                    dataset_root,
                    image_key,
                    image_width=image_width,
                    image_height=image_height,
                    allow_blank_images=allow_blank_images,
                )
            dataset.add_frame(frame)
            frame_count += 1
        dataset.save_episode()
        print(f'converted {episode_id}: {len(episode_rows)} frames', flush=True)
    dataset.finalize()

    if fitted_state_vectorizer:
        save_state_vectorizer(vectorizer, state_vectorizer_out)

    summary = {
        'tool': 'room_315_vla_to_lerobot',
        'baseline_purpose': 'direct-action action_vector behavior-cloning dataset conversion',
        'source': str(input_file),
        'source_fingerprint': _file_fingerprint(input_file),
        'row_fingerprint': _rows_fingerprint(rows),
        'dataset_root': str(dataset_root.expanduser().resolve()),
        'output_root': str(root),
        'repo_id': repo_id,
        'name': name,
        'frames': frame_count,
        'episodes': len(episodes),
        'tasks': len(task_counts),
        'fps': fps,
        'use_videos': use_videos,
        'image_width': image_width,
        'image_height': image_height,
        'state_mode': state_mode,
        'state_dim': vectorizer.dim,
        'action_dim': len(action_names),
        'state_vectorizer': str(
            state_vectorizer_path.expanduser().resolve()
            if state_vectorizer_path is not None
            else state_vectorizer_out.expanduser().resolve()
        ),
        'state_vectorizer_fitted_from_input': fitted_state_vectorizer,
        'feature_keys': sorted(features),
        'model_input_integrity': model_input_integrity,
        'camera_completeness': image_integrity,
        'class_balance': class_balance_report(rows, expected_dim=len(action_names)),
        'feature_purity': {
            'production_features_from': 'model_input',
            'row_level_metadata_used_as_features': [],
            'debug_or_ablation_features': [],
        },
    }
    (root / 'room315_conversion.json').write_text(_pretty_json(summary) + '\n', encoding='utf-8')
    print(_pretty_json(summary))
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Convert Room 315 VLA event JSONL data into a local LeRobot dataset.'
    )
    parser.add_argument(
        'input',
        nargs='?',
        type=Path,
        default=DEFAULT_SPLITS_DIR / 'train.jsonl',
        help=(
            'Room 315 JSONL split, full training_events.jsonl, or dataset directory. '
            'Defaults to ROOM315_VLA_SPLITS_DIR/train.jsonl or '
            'room315_local_training/splits/train.jsonl.'
        ),
    )
    parser.add_argument(
        '--dataset-root',
        type=Path,
        default=DEFAULT_DATASET_ROOT,
        help='Dataset root for relative image refs. Defaults to ROOM315_VLA_DATASET_ROOT.',
    )
    parser.add_argument(
        '--output-root',
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
        help='LeRobot output root. Defaults to ROOM315_LEROBOT_OUTPUT_ROOT.',
    )
    parser.add_argument('--name', default='room315_vla_train')
    parser.add_argument('--repo-id', default='room315/room315_vla_train')
    parser.add_argument(
        '--state-mode',
        choices=('full', 'compact32'),
        default='full',
        help='State representation to write. Use compact32 when fine-tuning SmolVLA base.',
    )
    parser.add_argument(
        '--state-vectorizer',
        type=Path,
        default=None,
        help='Existing state vectorizer JSON. Use this for val/test conversions.',
    )
    parser.add_argument(
        '--state-vectorizer-out',
        type=Path,
        default=None,
        help='Where to write a fitted state vectorizer when --state-vectorizer is omitted.',
    )
    parser.add_argument('--image-width', type=int, default=320)
    parser.add_argument('--image-height', type=int, default=240)
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--max-episodes', type=int, default=None)
    parser.add_argument('--max-rows', type=int, default=None)
    parser.add_argument('--use-videos', action='store_true')
    parser.add_argument(
        '--allow-blank-images',
        action='store_true',
        help='Debug only: substitute black frames for missing/unreadable required images.',
    )
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    state_vectorizer_out = args.state_vectorizer_out
    if state_vectorizer_out is None:
        state_vectorizer_out = (
            DEFAULT_COMPACT_STATE_VECTORIZER
            if args.state_mode == 'compact32'
            else DEFAULT_STATE_VECTORIZER
        )

    convert_room315_to_lerobot(
        args.input,
        dataset_root=args.dataset_root.expanduser().resolve(),
        output_root=args.output_root,
        name=args.name,
        repo_id=args.repo_id,
        state_mode=args.state_mode,
        state_vectorizer_path=args.state_vectorizer,
        state_vectorizer_out=state_vectorizer_out,
        image_width=args.image_width,
        image_height=args.image_height,
        fps=args.fps,
        max_episodes=args.max_episodes,
        max_rows=args.max_rows,
        use_videos=args.use_videos,
        allow_blank_images=args.allow_blank_images,
        overwrite=args.overwrite,
    )


if __name__ == '__main__':
    main()
