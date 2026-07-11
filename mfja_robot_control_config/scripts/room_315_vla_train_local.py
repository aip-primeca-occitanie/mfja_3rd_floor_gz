#!/usr/bin/env python3
"""Small custom Room 315 direct-action baseline trainer.

This is intentionally not a SmolVLA trainer. It trains a compact local PyTorch
baseline over event-level action vectors using production features declared in
`model_input` only.
"""

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
import random
import re
import time
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from PIL import Image

try:
    from room_315_legacy_direct_action_metrics import (
        BASELINE_ID,
        action_metrics,
        action_metrics_by_family,
        detect_vectorizer_leakage,
        episode_family_lookup,
        family_from_row,
        file_fingerprint as baseline_file_fingerprint,
        offline_supervisor_decision,
        quantize_action as baseline_quantize_action,
        rows_fingerprint as baseline_rows_fingerprint,
    )
except ModuleNotFoundError:
    _metrics_path = Path(__file__).with_name('room_315_legacy_direct_action_metrics.py')
    _spec = importlib.util.spec_from_file_location(
        'room_315_legacy_direct_action_metrics',
        _metrics_path,
    )
    if _spec is None or _spec.loader is None:
        raise
    _metrics = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_metrics)
    BASELINE_ID = _metrics.BASELINE_ID
    action_metrics = _metrics.action_metrics
    action_metrics_by_family = _metrics.action_metrics_by_family
    detect_vectorizer_leakage = _metrics.detect_vectorizer_leakage
    episode_family_lookup = _metrics.episode_family_lookup
    family_from_row = _metrics.family_from_row
    baseline_file_fingerprint = _metrics.file_fingerprint
    offline_supervisor_decision = _metrics.offline_supervisor_decision
    baseline_quantize_action = _metrics.quantize_action
    baseline_rows_fingerprint = _metrics.rows_fingerprint

try:
    from room_315_visual_state_dataset import (
        DATASET_MODE_LEGACY_ACTION,
        DATASET_MODE_VISUAL_STATE,
        VISUAL_LABEL_SUFFIX,
        VISUAL_STATE_SCHEMA_VERSION,
        VisualStateLabelVectorizer,
        load_visual_labels_for_rows,
        pretty_json as visual_pretty_json,
        validate_visual_state_rows,
        visual_label_path_for_split,
        visual_model_input_image_refs,
        visual_state_class_balance,
        visual_state_metrics,
        visual_target_stats,
    )
except ModuleNotFoundError:
    _visual_state_path = Path(__file__).with_name('room_315_visual_state_dataset.py')
    _visual_spec = importlib.util.spec_from_file_location(
        'room_315_visual_state_dataset',
        _visual_state_path,
    )
    if _visual_spec is None or _visual_spec.loader is None:
        raise
    _visual_state = importlib.util.module_from_spec(_visual_spec)
    _visual_spec.loader.exec_module(_visual_state)
    DATASET_MODE_LEGACY_ACTION = _visual_state.DATASET_MODE_LEGACY_ACTION
    DATASET_MODE_VISUAL_STATE = _visual_state.DATASET_MODE_VISUAL_STATE
    VISUAL_LABEL_SUFFIX = _visual_state.VISUAL_LABEL_SUFFIX
    VISUAL_STATE_SCHEMA_VERSION = _visual_state.VISUAL_STATE_SCHEMA_VERSION
    VisualStateLabelVectorizer = _visual_state.VisualStateLabelVectorizer
    load_visual_labels_for_rows = _visual_state.load_visual_labels_for_rows
    visual_pretty_json = _visual_state.pretty_json
    validate_visual_state_rows = _visual_state.validate_visual_state_rows
    visual_label_path_for_split = _visual_state.visual_label_path_for_split
    visual_model_input_image_refs = _visual_state.visual_model_input_image_refs
    visual_state_class_balance = _visual_state.visual_state_class_balance
    visual_state_metrics = _visual_state.visual_state_metrics
    visual_target_stats = _visual_state.visual_target_stats


def _env_path(name: str, fallback: str) -> Path:
    return Path(os.environ.get(name, fallback))


DEFAULT_SPLITS_DIR = _env_path('ROOM315_VLA_SPLITS_DIR', 'room315_local_training/splits')
DEFAULT_OUTPUT_DIR = _env_path('ROOM315_LOCAL_BASELINE_OUTPUT_DIR', 'room315_local_training/checkpoints/v0')
DEFAULT_DATASET_ROOT = _env_path('ROOM315_VLA_DATASET_ROOT', 'room315_payload_dataset')
DEFAULT_ACTION_SPACE = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'room_315_vla'
    / 'action_space.yaml'
)
IMAGE_KEYS = ('left_rail_rgb', 'right_rail_rgb')
MODEL_INPUT_KEYS = {'language', 'overhead_images', 'last_command', 'observable_state'}
DATASET_MODES = (DATASET_MODE_LEGACY_ACTION, DATASET_MODE_VISUAL_STATE)
VISUAL_STATE_NAMES = ('visual_state.constant_zero_no_privileged_state',)
TEXT_PAD = '<pad>'
TEXT_UNK = '<unk>'
SPEED_INDEX = 19
SPEED_FIELD = 'speed_mps'


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


def _require_torch():
    try:
        import torch
    except ImportError as exc:
        raise SystemExit(
            'PyTorch is required for local neural training but is not installed.\n'
            'Install it first, then rerun this command. For CUDA 12.8, start with:\n'
            '  python3 -m pip install --user torch --index-url https://download.pytorch.org/whl/cu128\n'
            'Or choose the current command from https://pytorch.org/get-started/locally/.'
        ) from exc
    return torch


def _load_manifest(splits_dir: Path) -> dict[str, Any]:
    manifest_path = splits_dir / 'split_manifest.json'
    if not manifest_path.exists():
        return {}
    with manifest_path.open('r', encoding='utf-8') as stream:
        parsed = json.load(stream)
    return parsed if isinstance(parsed, dict) else {}


def _load_json(path: Path) -> dict[str, Any]:
    with path.expanduser().open('r', encoding='utf-8') as stream:
        parsed = json.load(stream)
    return parsed if isinstance(parsed, dict) else {}


def _safe_int(raw: Any, fallback: int = 0) -> int:
    try:
        return int(raw)
    except (TypeError, ValueError):
        return fallback


def _resolve_dataset_root(dataset_root: Path | None, splits_dir: Path) -> Path:
    if dataset_root is not None:
        return dataset_root.expanduser().resolve()
    manifest = _load_manifest(splits_dir)
    root = str(manifest.get('dataset_root') or '').strip()
    if root:
        return Path(root).expanduser().resolve()
    return DEFAULT_DATASET_ROOT.expanduser().resolve()


def _mapping_range(config: dict[str, Any], key: str, *, fallback: tuple[int, int]) -> tuple[int, int]:
    values = config.get(key)
    if not isinstance(values, dict) or not values:
        return fallback
    numeric_values = [_safe_int(value) for value in values.values()]
    return min(numeric_values), max(numeric_values)


def load_action_space(path: Path = DEFAULT_ACTION_SPACE) -> dict[str, Any]:
    with path.expanduser().open('r', encoding='utf-8') as stream:
        parsed = yaml.safe_load(stream)
    if not isinstance(parsed, dict):
        raise ValueError(f'action space YAML must contain a mapping: {path}')
    fields = parsed.get('action_vector_fields')
    if not isinstance(fields, list) or not fields:
        raise ValueError(f'action space YAML has no action_vector_fields: {path}')
    return parsed


def action_vector_fields(action_space: dict[str, Any]) -> list[str]:
    return [str(field) for field in action_space['action_vector_fields']]


def action_field_ranges(action_space: dict[str, Any]) -> dict[str, tuple[int, int]]:
    ranges: dict[str, tuple[int, int]] = {
        'primitive_id': _mapping_range(action_space, 'primitive_ids', fallback=(0, 6)),
        'side_id': _mapping_range(action_space, 'side_ids', fallback=(0, 1)),
        'shuttle_index': (-1, 3),
        'wait_condition_id': _mapping_range(action_space, 'wait_condition_ids', fallback=(0, 9)),
        'target_id': _mapping_range(action_space, 'target_ids', fallback=(0, 35)),
        'reason_id': _mapping_range(action_space, 'reason_ids', fallback=(0, 21)),
        'coordination_mode': _mapping_range(action_space, 'coordination_mode_ids', fallback=(0, 5)),
    }
    for slot in ('A1', 'A2', 'A3', 'A4'):
        ranges[f'switch_mask_{slot}'] = _mapping_range(
            action_space, 'device_mask_ids', fallback=(0, 1)
        )
        ranges[f'stopper_mask_{slot}'] = _mapping_range(
            action_space, 'device_mask_ids', fallback=(0, 1)
        )
        ranges[f'switch_value_{slot}'] = _mapping_range(
            action_space, 'switch_value_ids', fallback=(0, 2)
        )
        ranges[f'stopper_value_{slot}'] = _mapping_range(
            action_space, 'stopper_value_ids', fallback=(0, 2)
        )
    return ranges


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


def tokenize(text: Any) -> list[str]:
    return re.findall(r'[A-Za-z0-9_]+', str(text or '').lower())


def _language(row: dict[str, Any]) -> str:
    language = str(_model_input(row).get('language') or '')
    if not language:
        raise ValueError(f'row for episode {row.get("episode_id", "")!r} has empty model_input.language')
    return language


def build_vocab(
    rows: list[dict[str, Any]],
    *,
    max_vocab_size: int = 2048,
    min_freq: int = 1,
) -> dict[str, int]:
    counts: Counter[str] = Counter()
    for row in rows:
        counts.update(tokenize(_language(row)))
    tokens = [
        token
        for token, count in sorted(counts.items(), key=lambda item: (-item[1], item[0]))
        if count >= min_freq
    ]
    vocab = {TEXT_PAD: 0, TEXT_UNK: 1}
    for token in tokens[:max(0, max_vocab_size - len(vocab))]:
        vocab[token] = len(vocab)
    return vocab


def encode_text(text: Any, vocab: dict[str, int], max_tokens: int) -> np.ndarray:
    token_ids = [vocab.get(token, vocab[TEXT_UNK]) for token in tokenize(text)]
    token_ids = token_ids[:max_tokens]
    if len(token_ids) < max_tokens:
        token_ids.extend([vocab[TEXT_PAD]] * (max_tokens - len(token_ids)))
    return np.asarray(token_ids, dtype=np.int64)


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
    state = {}
    _flatten('observable_state', model_input.get('observable_state', {}), state)
    _flatten('last_command', model_input.get('last_command', {}), state)
    return state


class StateVectorizer:
    def __init__(
        self,
        numeric_keys: list[str],
        categorical_values: dict[str, list[str]],
    ) -> None:
        self.numeric_keys = list(numeric_keys)
        self.categorical_values = {
            key: list(values)
            for key, values in categorical_values.items()
        }

    @classmethod
    def fit(cls, rows: list[dict[str, Any]]) -> 'StateVectorizer':
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
    def from_json(cls, data: dict[str, Any]) -> 'StateVectorizer':
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
    def dim(self) -> int:
        return len(self.numeric_keys) + sum(len(values) for values in self.categorical_values.values())

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
            'numeric_keys': self.numeric_keys,
            'categorical_values': self.categorical_values,
            'dim': self.dim,
        }


def _image_refs(row: dict[str, Any], *, dataset_mode: str = DATASET_MODE_LEGACY_ACTION) -> dict[str, str]:
    if dataset_mode == DATASET_MODE_VISUAL_STATE:
        return visual_model_input_image_refs(row)
    overhead_images = _model_input(row).get('overhead_images', {})
    return {
        str(key): str(value)
        for key, value in overhead_images.items()
        if value
    }


def _resolve_image_path(dataset_root: Path, image_ref: str) -> Path:
    path = Path(str(image_ref)).expanduser()
    return path if path.is_absolute() else dataset_root / path


def _blank_image_tensor(*, width: int, height: int) -> np.ndarray:
    return np.zeros((3, height, width), dtype=np.float32)


def _missing_image_error(row: dict[str, Any], image_key: str, reason: str, ref: str = '') -> str:
    episode_id = str(row.get('episode_id') or '')
    detail = f' ({ref})' if ref else ''
    return f'episode {episode_id!r} image {image_key!r} is {reason}{detail}'


def load_image_tensor(
    dataset_root: Path,
    image_ref: str,
    *,
    width: int,
    height: int,
    row: dict[str, Any] | None = None,
    image_key: str = '',
    allow_blank_images: bool = False,
) -> np.ndarray:
    if not image_ref:
        if allow_blank_images:
            return _blank_image_tensor(width=width, height=height)
        raise FileNotFoundError(
            _missing_image_error(row or {}, image_key, 'missing from model_input.overhead_images')
        )
    image_path = _resolve_image_path(dataset_root, image_ref)
    if not image_path.exists():
        if allow_blank_images:
            return _blank_image_tensor(width=width, height=height)
        raise FileNotFoundError(_missing_image_error(row or {}, image_key, 'missing on disk', image_ref))
    try:
        with Image.open(image_path) as image:
            rgb = image.convert('RGB').resize((width, height), Image.BILINEAR)
            array = np.asarray(rgb, dtype=np.float32) / 255.0
    except Exception as exc:
        if allow_blank_images:
            return _blank_image_tensor(width=width, height=height)
        raise RuntimeError(_missing_image_error(row or {}, image_key, 'unreadable', image_ref)) from exc
    return np.transpose(array, (2, 0, 1))


def load_paired_images(
    row: dict[str, Any],
    dataset_root: Path,
    *,
    width: int,
    height: int,
    allow_blank_images: bool = False,
    dataset_mode: str = DATASET_MODE_LEGACY_ACTION,
) -> np.ndarray:
    refs = _image_refs(row, dataset_mode=dataset_mode)
    left = load_image_tensor(
        dataset_root,
        refs.get('left_rail_rgb', ''),
        width=width,
        height=height,
        row=row,
        image_key='left_rail_rgb',
        allow_blank_images=allow_blank_images,
    )
    right = load_image_tensor(
        dataset_root,
        refs.get('right_rail_rgb', ''),
        width=width,
        height=height,
        row=row,
        image_key='right_rail_rgb',
        allow_blank_images=allow_blank_images,
    )
    return np.concatenate([left, right], axis=0).astype(np.float32)


def validate_model_input_rows(rows: list[dict[str, Any]], *, split_name: str) -> dict[str, Any]:
    issues: list[dict[str, Any]] = []
    for row_index, row in enumerate(rows):
        try:
            _model_input(row, context=f'{split_name} row {row_index}')
            _language(row)
        except ValueError as exc:
            if len(issues) < 20:
                issues.append({
                    'row_index': row_index,
                    'episode_id': str(row.get('episode_id') or ''),
                    'reason': str(exc),
                })
    if issues:
        raise ValueError(f'{split_name} model_input integrity check failed: {issues[0]}')
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


def image_integrity_report(
    rows: list[dict[str, Any]],
    dataset_root: Path,
    *,
    split_name: str,
    allow_blank_images: bool = False,
    dataset_mode: str = DATASET_MODE_LEGACY_ACTION,
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
        refs = _image_refs(row, dataset_mode=dataset_mode)
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
        raise FileNotFoundError(f'{split_name} image integrity check failed before training: {first}')
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
            speed[_rounded_key(vector[SPEED_INDEX])] += 1
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


def split_integrity_report(
    train_rows: list[dict[str, Any]],
    val_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    train_episodes = {str(row.get('episode_id') or '') for row in train_rows}
    val_episodes = {str(row.get('episode_id') or '') for row in val_rows}
    overlap = sorted((train_episodes & val_episodes) - {''})
    return {
        'train_rows': len(train_rows),
        'val_rows': len(val_rows),
        'train_episode_count': len(train_episodes - {''}),
        'val_episode_count': len(val_episodes - {''}),
        'episode_overlap': overlap,
        'disjoint_episodes': not overlap,
        'train_row_fingerprint': _rows_fingerprint(train_rows),
        'val_row_fingerprint': _rows_fingerprint(val_rows),
    }


def action_vector(row: dict[str, Any]) -> np.ndarray:
    raw = row.get('action_vector')
    if not isinstance(raw, list):
        raise ValueError(f'row for episode {row.get("episode_id", "")!r} is missing action_vector')
    return np.asarray([float(value) for value in raw], dtype=np.float32)


def target_stats(rows: list[dict[str, Any]]) -> tuple[np.ndarray, np.ndarray]:
    targets = np.stack([action_vector(row) for row in rows], axis=0).astype(np.float32)
    mean = targets.mean(axis=0)
    std = targets.std(axis=0)
    std[std < 1e-6] = 1.0
    return mean.astype(np.float32), std.astype(np.float32)


class Room315EventDataset:
    def __init__(
        self,
        rows: list[dict[str, Any]],
        *,
        dataset_root: Path,
        vocab: dict[str, int],
        state_vectorizer: StateVectorizer,
        target_mean: np.ndarray,
        target_std: np.ndarray,
        max_tokens: int,
        image_width: int,
        image_height: int,
        torch_module: Any,
        allow_blank_images: bool = False,
    ) -> None:
        self.rows = rows
        self.dataset_root = dataset_root
        self.vocab = vocab
        self.state_vectorizer = state_vectorizer
        self.target_mean = target_mean
        self.target_std = target_std
        self.max_tokens = max_tokens
        self.image_width = image_width
        self.image_height = image_height
        self.torch = torch_module
        self.allow_blank_images = allow_blank_images

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int) -> dict[str, Any]:
        row = self.rows[index]
        target = action_vector(row)
        normalized_target = (target - self.target_mean) / self.target_std
        return {
            'text': self.torch.as_tensor(
                encode_text(_language(row), self.vocab, self.max_tokens),
                dtype=self.torch.long,
            ),
            'state': self.torch.as_tensor(
                self.state_vectorizer.transform(row),
                dtype=self.torch.float32,
            ),
            'image': self.torch.as_tensor(
                load_paired_images(
                    row,
                    self.dataset_root,
                    width=self.image_width,
                    height=self.image_height,
                    allow_blank_images=self.allow_blank_images,
                ),
                dtype=self.torch.float32,
            ),
            'target': self.torch.as_tensor(normalized_target, dtype=self.torch.float32),
            'raw_target': self.torch.as_tensor(target, dtype=self.torch.float32),
        }


class Room315VisualStateDataset:
    def __init__(
        self,
        rows: list[dict[str, Any]],
        labels: list[dict[str, Any]],
        *,
        dataset_root: Path,
        label_vectorizer: VisualStateLabelVectorizer,
        target_mean: np.ndarray,
        target_std: np.ndarray,
        max_tokens: int,
        image_width: int,
        image_height: int,
        torch_module: Any,
        allow_blank_images: bool = False,
    ) -> None:
        if len(rows) != len(labels):
            raise ValueError('visual-state rows and labels must have the same length')
        self.rows = rows
        self.labels = labels
        self.dataset_root = dataset_root
        self.label_vectorizer = label_vectorizer
        self.target_mean = target_mean
        self.target_std = target_std
        self.max_tokens = max_tokens
        self.image_width = image_width
        self.image_height = image_height
        self.torch = torch_module
        self.allow_blank_images = allow_blank_images

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int) -> dict[str, Any]:
        row = self.rows[index]
        target = np.asarray(self.label_vectorizer.transform(self.labels[index]), dtype=np.float32)
        normalized_target = (target - self.target_mean) / self.target_std
        return {
            'text': self.torch.zeros((self.max_tokens,), dtype=self.torch.long),
            'state': self.torch.zeros((len(VISUAL_STATE_NAMES),), dtype=self.torch.float32),
            'image': self.torch.as_tensor(
                load_paired_images(
                    row,
                    self.dataset_root,
                    width=self.image_width,
                    height=self.image_height,
                    allow_blank_images=self.allow_blank_images,
                    dataset_mode=DATASET_MODE_VISUAL_STATE,
                ),
                dtype=self.torch.float32,
            ),
            'target': self.torch.as_tensor(normalized_target, dtype=self.torch.float32),
            'raw_target': self.torch.as_tensor(target, dtype=self.torch.float32),
        }


def _build_model(torch_module: Any, *, vocab_size: int, state_dim: int, output_dim: int):
    nn = torch_module.nn

    class Room315LocalVlaModel(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.embedding = nn.Embedding(vocab_size, 96, padding_idx=0)
            self.image_encoder = nn.Sequential(
                nn.Conv2d(6, 24, kernel_size=5, stride=2, padding=2),
                nn.ReLU(inplace=True),
                nn.Conv2d(24, 48, kernel_size=3, stride=2, padding=1),
                nn.ReLU(inplace=True),
                nn.Conv2d(48, 96, kernel_size=3, stride=2, padding=1),
                nn.ReLU(inplace=True),
                nn.AdaptiveAvgPool2d((1, 1)),
                nn.Flatten(),
            )
            self.state_encoder = nn.Sequential(
                nn.Linear(max(1, state_dim), 96),
                nn.ReLU(inplace=True),
                nn.Linear(96, 96),
                nn.ReLU(inplace=True),
            )
            self.head = nn.Sequential(
                nn.Linear(96 + 96 + 96, 192),
                nn.ReLU(inplace=True),
                nn.Dropout(0.1),
                nn.Linear(192, output_dim),
            )

        def forward(self, text, state, image):
            embedded = self.embedding(text)
            mask = (text != 0).unsqueeze(-1).float()
            text_features = (embedded * mask).sum(dim=1) / mask.sum(dim=1).clamp_min(1.0)
            image_features = self.image_encoder(image)
            state_features = self.state_encoder(state)
            return self.head(torch_module.cat([text_features, image_features, state_features], dim=1))

    return Room315LocalVlaModel()


def _choose_device(torch_module: Any, requested: str) -> str:
    requested = str(requested or 'auto').strip().lower()
    if requested == 'auto':
        return 'cuda' if torch_module.cuda.is_available() else 'cpu'
    if requested == 'cuda' and not torch_module.cuda.is_available():
        raise SystemExit('CUDA was requested, but torch.cuda.is_available() is false.')
    return requested


def _set_seed(torch_module: Any, seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch_module.manual_seed(seed)
    if torch_module.cuda.is_available():
        torch_module.cuda.manual_seed_all(seed)


def _row_limit(rows: list[dict[str, Any]], limit: int | None) -> list[dict[str, Any]]:
    if limit is None or limit <= 0:
        return rows
    return rows[:limit]


def _evaluate(
    torch_module: Any,
    model: Any,
    loader: Any,
    *,
    device: str,
    target_mean: np.ndarray,
    target_std: np.ndarray,
) -> dict[str, float]:
    model.eval()
    loss_fn = torch_module.nn.SmoothL1Loss(reduction='sum')
    mean_tensor = torch_module.as_tensor(target_mean, dtype=torch_module.float32, device=device)
    std_tensor = torch_module.as_tensor(target_std, dtype=torch_module.float32, device=device)
    rows = 0
    loss_total = 0.0
    mae_total = 0.0
    discrete_correct = 0
    discrete_total = 0
    exact_count = 0
    primitive_correct = 0
    side_correct = 0
    target_correct = 0
    speed_error_total = 0.0
    with torch_module.no_grad():
        for batch in loader:
            text = batch['text'].to(device)
            state = batch['state'].to(device)
            image = batch['image'].to(device)
            target = batch['target'].to(device)
            raw_target = batch['raw_target'].to(device)
            pred_norm = model(text, state, image)
            loss_total += float(loss_fn(pred_norm, target).item())
            pred = pred_norm * std_tensor + mean_tensor
            abs_error = (pred - raw_target).abs()
            batch_size = int(raw_target.shape[0])
            rows += batch_size
            mae_total += float(abs_error.sum().item())
            speed_error_total += float(abs_error[:, SPEED_INDEX].sum().item())

            discrete_indices = [idx for idx in range(raw_target.shape[1]) if idx != SPEED_INDEX]
            pred_discrete = pred[:, discrete_indices].round()
            true_discrete = raw_target[:, discrete_indices].round()
            matches = pred_discrete.eq(true_discrete)
            discrete_correct += int(matches.sum().item())
            discrete_total += int(matches.numel())
            speed_ok = abs_error[:, SPEED_INDEX] <= 0.005
            exact = matches.all(dim=1) & speed_ok
            exact_count += int(exact.sum().item())
            primitive_correct += int(pred[:, 0].round().eq(raw_target[:, 0].round()).sum().item())
            side_correct += int(pred[:, 1].round().eq(raw_target[:, 1].round()).sum().item())
            target_correct += int(pred[:, 21].round().eq(raw_target[:, 21].round()).sum().item())
    denom = max(1, rows)
    return {
        'loss': round(loss_total / denom, 6),
        'vector_mae': round(mae_total / max(1, rows * len(target_mean)), 6),
        'discrete_field_accuracy': round(discrete_correct / max(1, discrete_total), 6),
        'exact_action_accuracy': round(exact_count / denom, 6),
        'primitive_accuracy': round(primitive_correct / denom, 6),
        'side_accuracy': round(side_correct / denom, 6),
        'target_id_accuracy': round(target_correct / denom, 6),
        'speed_mae': round(speed_error_total / denom, 6),
    }


def _evaluate_visual_state(
    torch_module: Any,
    model: Any,
    loader: Any,
    *,
    device: str,
    target_mean: np.ndarray,
    target_std: np.ndarray,
    label_names: list[str],
) -> dict[str, Any]:
    model.eval()
    loss_fn = torch_module.nn.SmoothL1Loss(reduction='sum')
    mean_tensor = torch_module.as_tensor(target_mean, dtype=torch_module.float32, device=device)
    std_tensor = torch_module.as_tensor(target_std, dtype=torch_module.float32, device=device)
    rows = 0
    loss_total = 0.0
    records: list[dict[str, Any]] = []
    with torch_module.no_grad():
        for batch in loader:
            text = batch['text'].to(device)
            state = batch['state'].to(device)
            image = batch['image'].to(device)
            target = batch['target'].to(device)
            raw_target = batch['raw_target'].to(device)
            pred_norm = model(text, state, image)
            loss_total += float(loss_fn(pred_norm, target).item())
            pred = pred_norm * std_tensor + mean_tensor
            batch_size = int(raw_target.shape[0])
            rows += batch_size
            for true_row, pred_row in zip(raw_target.detach().cpu().numpy(), pred.detach().cpu().numpy()):
                records.append({
                    'true_raw': true_row.astype(np.float32).tolist(),
                    'pred_raw': pred_row.astype(np.float32).tolist(),
                })
    metrics = visual_state_metrics(records, label_names)
    metrics['loss'] = round(loss_total / max(1, rows), 6)
    return metrics


def _save_checkpoint(
    torch_module: Any,
    path: Path,
    *,
    model: Any,
    optimizer: Any,
    epoch: int,
    metrics: dict[str, Any],
    config: dict[str, Any],
) -> None:
    torch_module.save(
        {
            'epoch': epoch,
            'model_state_dict': model.state_dict(),
            'optimizer_state_dict': optimizer.state_dict(),
            'metrics': metrics,
            'config': config,
        },
        path,
    )


def _torch_load_checkpoint(torch_module: Any, path: Path) -> dict[str, Any]:
    try:
        loaded = torch_module.load(path, map_location='cpu', weights_only=False)
    except TypeError:
        loaded = torch_module.load(path, map_location='cpu')
    if not isinstance(loaded, dict) or 'model_state_dict' not in loaded:
        raise ValueError(f'checkpoint does not contain a model_state_dict: {path}')
    return loaded


def _artifact_path(checkpoint: Path, name: str) -> Path:
    return checkpoint.expanduser().resolve().parent / name


def _load_target_stats(path: Path) -> tuple[np.ndarray, np.ndarray]:
    parsed = _load_json(path)
    mean = np.asarray(parsed.get('mean'), dtype=np.float32)
    std = np.asarray(parsed.get('std'), dtype=np.float32)
    if mean.ndim != 1 or std.ndim != 1 or mean.shape != std.shape:
        raise ValueError(f'target stats must contain same-length mean/std arrays: {path}')
    std = std.copy()
    std[std < 1e-6] = 1.0
    return mean, std


def _checkpoint_training_config(checkpoint: dict[str, Any], checkpoint_path: Path) -> dict[str, Any]:
    config = checkpoint.get('config')
    if isinstance(config, dict):
        return dict(config)
    path = _artifact_path(checkpoint_path, 'training_config.json')
    return _load_json(path) if path.exists() else {}


def _peak_gpu_memory(torch_module: Any, device: str) -> dict[str, Any]:
    if device != 'cuda' or not torch_module.cuda.is_available():
        return {
            'device': device,
            'cuda_available': bool(torch_module.cuda.is_available()),
            'allocated_bytes': None,
            'reserved_bytes': None,
        }
    return {
        'device': device,
        'cuda_available': True,
        'allocated_bytes': int(torch_module.cuda.max_memory_allocated()),
        'reserved_bytes': int(torch_module.cuda.max_memory_reserved()),
    }


def _row_tensor_inputs(
    torch_module: Any,
    row: dict[str, Any],
    *,
    dataset_root: Path,
    vocab: dict[str, int],
    state_vectorizer: StateVectorizer,
    max_tokens: int,
    image_width: int,
    image_height: int,
    allow_blank_images: bool,
    device: str,
) -> tuple[Any, Any, Any]:
    text = torch_module.as_tensor(
        encode_text(_language(row), vocab, max_tokens),
        dtype=torch_module.long,
        device=device,
    ).unsqueeze(0)
    state = torch_module.as_tensor(
        state_vectorizer.transform(row),
        dtype=torch_module.float32,
        device=device,
    ).unsqueeze(0)
    image = torch_module.as_tensor(
        load_paired_images(
            row,
            dataset_root,
            width=image_width,
            height=image_height,
            allow_blank_images=allow_blank_images,
        ),
        dtype=torch_module.float32,
        device=device,
    ).unsqueeze(0)
    return text, state, image


def _write_prediction_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    speed_index = fields.index(SPEED_FIELD)
    primitive_index = fields.index('primitive_id')
    side_index = fields.index('side_id')
    target_index = fields.index('target_id')
    fieldnames = [
        'split',
        'sample_index',
        'episode_id',
        'task_family',
        'true_primitive_id',
        'pred_primitive_id',
        'true_side_id',
        'pred_side_id',
        'true_target_id',
        'pred_target_id',
        'true_speed_mps',
        'pred_speed_mps',
        'action_schema_legal',
        'supervisor_rejection_reason',
        'inference_latency_s',
        'cycle_time_s',
    ]
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for record in rows:
            true_q = record['true_quantized']
            pred_q = record['pred_quantized']
            decision = record.get('supervisor_decision') or {}
            writer.writerow({
                'split': record['split'],
                'sample_index': record['sample_index'],
                'episode_id': record['episode_id'],
                'task_family': record['task_family'],
                'true_primitive_id': int(true_q[primitive_index]),
                'pred_primitive_id': int(pred_q[primitive_index]),
                'true_side_id': int(true_q[side_index]),
                'pred_side_id': int(pred_q[side_index]),
                'true_target_id': int(true_q[target_index]),
                'pred_target_id': int(pred_q[target_index]),
                'true_speed_mps': round(float(record['true_raw'][speed_index]), 5),
                'pred_speed_mps': round(float(record['pred_raw'][speed_index]), 5),
                'action_schema_legal': bool(decision.get('accepted')),
                'supervisor_rejection_reason': str(decision.get('reason') or ''),
                'inference_latency_s': round(float(record['inference_latency_seconds']), 6),
                'cycle_time_s': round(float(record['cycle_time_seconds']), 6),
            })


def _visual_labels_path_for(splits_dir: Path, split_file: str, override: Path | None = None) -> Path:
    if override is not None:
        return override.expanduser().resolve()
    return visual_label_path_for_split(splits_dir / split_file).expanduser().resolve()


def _load_visual_split(
    splits_dir: Path,
    split_file: str,
    labels_path: Path,
    *,
    limit: int | None,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    rows = _row_limit(_iter_jsonl(splits_dir / split_file), limit)
    if not rows:
        raise ValueError(f'{split_file} split is empty')
    labels = load_visual_labels_for_rows(rows, labels_path)
    integrity = validate_visual_state_rows(rows, labels_path)
    return rows, labels, integrity


def _save_visual_label_vectorizer(vectorizer: VisualStateLabelVectorizer, path: Path) -> None:
    path.write_text(visual_pretty_json(vectorizer.to_json()) + '\n', encoding='utf-8')


def train_visual_state(args: argparse.Namespace) -> dict[str, Any]:
    torch_module = _require_torch()
    splits_dir = args.splits_dir.expanduser().resolve()
    dataset_root = _resolve_dataset_root(args.dataset_root, splits_dir)
    train_labels_path = _visual_labels_path_for(splits_dir, args.train_file, args.visual_train_labels)
    val_labels_path = _visual_labels_path_for(splits_dir, args.val_file, args.visual_val_labels)
    train_rows, train_labels, train_input = _load_visual_split(
        splits_dir,
        args.train_file,
        train_labels_path,
        limit=args.limit_train_rows,
    )
    val_rows, val_labels, val_input = _load_visual_split(
        splits_dir,
        args.val_file,
        val_labels_path,
        limit=args.limit_val_rows,
    )
    train_image_integrity = image_integrity_report(
        train_rows,
        dataset_root,
        split_name='train',
        allow_blank_images=args.allow_blank_images,
        dataset_mode=DATASET_MODE_VISUAL_STATE,
    )
    val_image_integrity = image_integrity_report(
        val_rows,
        dataset_root,
        split_name='val',
        allow_blank_images=args.allow_blank_images,
        dataset_mode=DATASET_MODE_VISUAL_STATE,
    )

    _set_seed(torch_module, args.seed)
    device = _choose_device(torch_module, args.device)
    label_vectorizer = VisualStateLabelVectorizer.fit(train_labels)
    target_mean_list, target_std_list = visual_target_stats(train_labels, label_vectorizer)
    target_mean = np.asarray(target_mean_list, dtype=np.float32)
    target_std = np.asarray(target_std_list, dtype=np.float32)
    output_dim = int(target_mean.shape[0])
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    vocab = {TEXT_PAD: 0, TEXT_UNK: 1}
    dataset_report = {
        'tool': 'room_315_vla_train_local',
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'baseline_purpose': 'small custom visual-state label predictor; outputs are not rail commands',
        'source_files': {
            'train': _file_fingerprint(splits_dir / args.train_file),
            'val': _file_fingerprint(splits_dir / args.val_file),
            'train_labels': _file_fingerprint(train_labels_path),
            'val_labels': _file_fingerprint(val_labels_path),
        },
        'row_fingerprint': {
            'train': _rows_fingerprint(train_rows),
            'val': _rows_fingerprint(val_rows),
            'train_labels': baseline_rows_fingerprint([
                {'visual_state_labels': label}
                for label in train_labels
            ]),
            'val_labels': baseline_rows_fingerprint([
                {'visual_state_labels': label}
                for label in val_labels
            ]),
        },
        'model_input_integrity': {
            'train': train_input,
            'val': val_input,
        },
        'camera_completeness': {
            'train': train_image_integrity,
            'val': val_image_integrity,
        },
        'class_balance': {
            'train': visual_state_class_balance(train_labels),
            'val': visual_state_class_balance(val_labels),
        },
        'split_integrity': split_integrity_report(train_rows, val_rows),
        'feature_purity': {
            'production_features_from': 'model_input.overhead_images',
            'oracle_labels_physically_separate_from_model_input': True,
            'row_level_metadata_used_as_features': [],
            'debug_or_ablation_features': ['allow_blank_images'] if args.allow_blank_images else [],
        },
        'schema': {
            'visual_state_schema_version': VISUAL_STATE_SCHEMA_VERSION,
            'label_vector_names': label_vectorizer.names,
        },
    }
    config = {
        'tool': 'room_315_vla_train_local',
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'baseline_purpose': 'small custom visual-state label predictor; outputs are not rail commands',
        'dataset_root': str(dataset_root),
        'splits_dir': str(splits_dir),
        'train_file': args.train_file,
        'val_file': args.val_file,
        'train_labels': str(train_labels_path),
        'val_labels': str(val_labels_path),
        'seed': args.seed,
        'device': device,
        'epochs': args.epochs,
        'batch_size': args.batch_size,
        'learning_rate': args.learning_rate,
        'weight_decay': args.weight_decay,
        'image_width': args.image_width,
        'image_height': args.image_height,
        'max_tokens': args.max_tokens,
        'train_rows': len(train_rows),
        'val_rows': len(val_rows),
        'output_dim': output_dim,
        'state_dim': len(VISUAL_STATE_NAMES),
        'vocab_size': len(vocab),
        'allow_blank_images': bool(args.allow_blank_images),
        'debug_blank_image_mode': bool(args.allow_blank_images),
        'production_feature_source': 'model_input.overhead_images only',
        'dataset_report': str(output_dir / 'dataset_report.json'),
    }

    (output_dir / 'dataset_report.json').write_text(_pretty_json(dataset_report) + '\n', encoding='utf-8')
    (output_dir / 'vocab.json').write_text(_pretty_json(vocab) + '\n', encoding='utf-8')
    (output_dir / 'visual_label_vectorizer.json').write_text(
        visual_pretty_json(label_vectorizer.to_json()) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'state_vectorizer.json').write_text(
        _pretty_json({
            'kind': 'room315_visual_state_constant_state',
            'dataset_mode': DATASET_MODE_VISUAL_STATE,
            'names': list(VISUAL_STATE_NAMES),
            'dim': len(VISUAL_STATE_NAMES),
        }) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'target_stats.json').write_text(
        _pretty_json({
            'mean': target_mean.tolist(),
            'std': target_std.tolist(),
        }) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'training_config.json').write_text(_pretty_json(config) + '\n', encoding='utf-8')

    train_dataset = Room315VisualStateDataset(
        train_rows,
        train_labels,
        dataset_root=dataset_root,
        label_vectorizer=label_vectorizer,
        target_mean=target_mean,
        target_std=target_std,
        max_tokens=args.max_tokens,
        image_width=args.image_width,
        image_height=args.image_height,
        torch_module=torch_module,
        allow_blank_images=args.allow_blank_images,
    )
    val_dataset = Room315VisualStateDataset(
        val_rows,
        val_labels,
        dataset_root=dataset_root,
        label_vectorizer=label_vectorizer,
        target_mean=target_mean,
        target_std=target_std,
        max_tokens=args.max_tokens,
        image_width=args.image_width,
        image_height=args.image_height,
        torch_module=torch_module,
        allow_blank_images=args.allow_blank_images,
    )
    train_loader = torch_module.utils.data.DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=(device == 'cuda'),
    )
    val_loader = torch_module.utils.data.DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=(device == 'cuda'),
    )
    model = _build_model(
        torch_module,
        vocab_size=len(vocab),
        state_dim=len(VISUAL_STATE_NAMES),
        output_dim=output_dim,
    ).to(device)
    optimizer = torch_module.optim.AdamW(
        model.parameters(),
        lr=args.learning_rate,
        weight_decay=args.weight_decay,
    )
    loss_fn = torch_module.nn.SmoothL1Loss()
    history: list[dict[str, Any]] = []
    best_val_loss = float('inf')
    best_epoch = 0

    for epoch in range(1, args.epochs + 1):
        model.train()
        running_loss = 0.0
        seen = 0
        for batch in train_loader:
            text = batch['text'].to(device)
            state = batch['state'].to(device)
            image = batch['image'].to(device)
            target = batch['target'].to(device)
            optimizer.zero_grad(set_to_none=True)
            pred = model(text, state, image)
            loss = loss_fn(pred, target)
            loss.backward()
            optimizer.step()
            batch_size = int(target.shape[0])
            running_loss += float(loss.item()) * batch_size
            seen += batch_size
        train_loss = running_loss / max(1, seen)
        val_metrics = _evaluate_visual_state(
            torch_module,
            model,
            val_loader,
            device=device,
            target_mean=target_mean,
            target_std=target_std,
            label_names=label_vectorizer.names,
        )
        epoch_metrics = {
            'epoch': epoch,
            'train_loss': round(train_loss, 6),
            **{f'val_{key}': value for key, value in val_metrics.items()},
        }
        history.append(epoch_metrics)
        print(_pretty_json(epoch_metrics), flush=True)
        _save_checkpoint(
            torch_module,
            output_dir / 'last.pt',
            model=model,
            optimizer=optimizer,
            epoch=epoch,
            metrics=epoch_metrics,
            config=config,
        )
        if float(val_metrics['loss']) < best_val_loss:
            best_val_loss = float(val_metrics['loss'])
            best_epoch = epoch
            _save_checkpoint(
                torch_module,
                output_dir / 'best.pt',
                model=model,
                optimizer=optimizer,
                epoch=epoch,
                metrics=epoch_metrics,
                config=config,
            )

    summary = {
        'tool': 'room_315_vla_train_local',
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'baseline_purpose': 'small custom visual-state label predictor; outputs are not rail commands',
        'output_dir': str(output_dir),
        'best_checkpoint': str(output_dir / 'best.pt'),
        'last_checkpoint': str(output_dir / 'last.pt'),
        'best_epoch': best_epoch,
        'best_val_loss': best_val_loss,
        'history': history,
        'config': config,
        'dataset_report': dataset_report,
    }
    (output_dir / 'metrics.json').write_text(_pretty_json(summary) + '\n', encoding='utf-8')
    return summary


def _load_visual_label_vectorizer(path: Path) -> VisualStateLabelVectorizer:
    parsed = _load_json(path)
    return VisualStateLabelVectorizer.from_json(parsed)


def evaluate_visual_state_checkpoint(args: argparse.Namespace) -> dict[str, Any]:
    torch_module = _require_torch()
    checkpoint_path = args.eval_checkpoint.expanduser().resolve()
    if not checkpoint_path.exists():
        raise FileNotFoundError(f'checkpoint not found: {checkpoint_path}')
    splits_dir = args.splits_dir.expanduser().resolve()
    dataset_root = _resolve_dataset_root(args.dataset_root, splits_dir)
    output_dir = (
        args.eval_output_dir.expanduser().resolve()
        if args.eval_output_dir is not None
        else checkpoint_path.parent / 'visual_state_eval'
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    checkpoint = _torch_load_checkpoint(torch_module, checkpoint_path)
    config = _checkpoint_training_config(checkpoint, checkpoint_path)
    vocab = _load_json(_artifact_path(checkpoint_path, 'vocab.json'))
    label_vectorizer = _load_visual_label_vectorizer(
        _artifact_path(checkpoint_path, 'visual_label_vectorizer.json')
    )
    target_mean, target_std = _load_target_stats(_artifact_path(checkpoint_path, 'target_stats.json'))
    _set_seed(torch_module, args.seed)
    device = _choose_device(torch_module, args.device)
    model = _build_model(
        torch_module,
        vocab_size=max(2, len(vocab)),
        state_dim=len(VISUAL_STATE_NAMES),
        output_dim=int(target_mean.shape[0]),
    ).to(device)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()
    if device == 'cuda' and torch_module.cuda.is_available():
        torch_module.cuda.reset_peak_memory_stats()

    split_names = [
        name.strip()
        for name in str(args.eval_splits or '').split(',')
        if name.strip()
    ]
    if not split_names:
        raise ValueError('--eval-splits selected no splits')

    image_width = _safe_int(config.get('image_width'), args.image_width)
    image_height = _safe_int(config.get('image_height'), args.image_height)
    max_tokens = _safe_int(config.get('max_tokens'), args.max_tokens)
    mean_tensor = torch_module.as_tensor(target_mean, dtype=torch_module.float32, device=device)
    std_tensor = torch_module.as_tensor(target_std, dtype=torch_module.float32, device=device)
    started = time.perf_counter()
    splits: dict[str, Any] = {}
    all_records: list[dict[str, Any]] = []

    with torch_module.no_grad():
        for split_name in split_names:
            split_file = f'{split_name}.jsonl'
            labels_path = visual_label_path_for_split(splits_dir / split_file)
            rows, labels, input_integrity = _load_visual_split(
                splits_dir,
                split_file,
                labels_path,
                limit=args.limit_eval_rows,
            )
            image_report = image_integrity_report(
                rows,
                dataset_root,
                split_name=split_name,
                allow_blank_images=args.allow_blank_images,
                dataset_mode=DATASET_MODE_VISUAL_STATE,
            )
            split_records: list[dict[str, Any]] = []
            for sample_index, (row, label) in enumerate(zip(rows, labels)):
                cycle_start = time.perf_counter()
                text = torch_module.zeros((1, max_tokens), dtype=torch_module.long, device=device)
                state = torch_module.zeros((1, len(VISUAL_STATE_NAMES)), dtype=torch_module.float32, device=device)
                image = torch_module.as_tensor(
                    load_paired_images(
                        row,
                        dataset_root,
                        width=image_width,
                        height=image_height,
                        allow_blank_images=args.allow_blank_images,
                        dataset_mode=DATASET_MODE_VISUAL_STATE,
                    ),
                    dtype=torch_module.float32,
                    device=device,
                ).unsqueeze(0)
                if device == 'cuda' and torch_module.cuda.is_available():
                    torch_module.cuda.synchronize()
                inference_start = time.perf_counter()
                pred_norm = model(text, state, image)
                if device == 'cuda' and torch_module.cuda.is_available():
                    torch_module.cuda.synchronize()
                inference_latency = time.perf_counter() - inference_start
                pred = (pred_norm[0] * std_tensor + mean_tensor).detach().cpu().numpy()
                true_raw = np.asarray(label_vectorizer.transform(label), dtype=np.float32)
                record = {
                    'split': split_name,
                    'sample_index': sample_index,
                    'episode_id': str(row.get('episode_id') or ''),
                    'task_family': str(row.get('scenario_family') or ''),
                    'true_raw': true_raw.tolist(),
                    'pred_raw': pred[: len(true_raw)].astype(np.float32).tolist(),
                    'inference_latency_seconds': inference_latency,
                    'cycle_time_seconds': time.perf_counter() - cycle_start,
                }
                split_records.append(record)
            splits[split_name] = {
                'source_file': baseline_file_fingerprint(splits_dir / split_file),
                'label_file': baseline_file_fingerprint(labels_path),
                'row_fingerprint': baseline_rows_fingerprint(rows),
                'label_fingerprint': baseline_rows_fingerprint([
                    {'visual_state_labels': label}
                    for label in labels
                ]),
                'rows': len(rows),
                'model_input_integrity': input_integrity,
                'image_integrity': image_report,
                'class_balance': visual_state_class_balance(labels),
                'metrics': visual_state_metrics(split_records, label_vectorizer.names),
            }
            all_records.extend(split_records)

    summary_path = output_dir / 'visual_state_eval.json'
    summary = {
        'tool': 'room_315_vla_train_local',
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'baseline_purpose': 'small custom visual-state label checkpoint evaluation',
        'checkpoint': str(checkpoint_path),
        'checkpoint_epoch': checkpoint.get('epoch'),
        'checkpoint_metrics': checkpoint.get('metrics'),
        'checkpoint_training_config': config,
        'dataset_root': str(dataset_root),
        'splits_dir': str(splits_dir),
        'evaluated_splits': split_names,
        'seed': args.seed,
        'device': device,
        'allow_blank_images': bool(args.allow_blank_images),
        'debug_blank_image_mode': bool(args.allow_blank_images),
        'production_feature_source': 'model_input.overhead_images only',
        'feature_purity': {
            'production_features_from': 'model_input.overhead_images',
            'oracle_labels_physically_separate_from_model_input': True,
            'row_level_metadata_used_as_features': [],
            'debug_or_ablation_features': ['allow_blank_images'] if args.allow_blank_images else [],
        },
        'artifact_fingerprints': {
            name: baseline_file_fingerprint(path)
            for name, path in {
                'checkpoint': checkpoint_path,
                'vocab': _artifact_path(checkpoint_path, 'vocab.json'),
                'visual_label_vectorizer': _artifact_path(checkpoint_path, 'visual_label_vectorizer.json'),
                'target_stats': _artifact_path(checkpoint_path, 'target_stats.json'),
                'training_config': _artifact_path(checkpoint_path, 'training_config.json'),
            }.items()
            if path.exists()
        },
        'label_names': label_vectorizer.names,
        'splits': splits,
        'aggregate_metrics': visual_state_metrics(all_records, label_vectorizer.names),
        'peak_gpu_memory': _peak_gpu_memory(torch_module, device),
        'elapsed_seconds': round(time.perf_counter() - started, 3),
        'summary_json': str(summary_path),
    }
    summary_path.write_text(_pretty_json(summary) + '\n', encoding='utf-8')
    return summary


def evaluate_checkpoint(args: argparse.Namespace) -> dict[str, Any]:
    if getattr(args, 'dataset_mode', DATASET_MODE_LEGACY_ACTION) == DATASET_MODE_VISUAL_STATE:
        return evaluate_visual_state_checkpoint(args)
    torch_module = _require_torch()
    checkpoint_path = args.eval_checkpoint.expanduser().resolve()
    if not checkpoint_path.exists():
        raise FileNotFoundError(f'checkpoint not found: {checkpoint_path}')
    splits_dir = args.splits_dir.expanduser().resolve()
    dataset_root = _resolve_dataset_root(args.dataset_root, splits_dir)
    output_dir = (
        args.eval_output_dir.expanduser().resolve()
        if args.eval_output_dir is not None
        else checkpoint_path.parent / BASELINE_ID
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    checkpoint = _torch_load_checkpoint(torch_module, checkpoint_path)
    config = _checkpoint_training_config(checkpoint, checkpoint_path)
    vocab = _load_json(_artifact_path(checkpoint_path, 'vocab.json'))
    vectorizer_path = _artifact_path(checkpoint_path, 'state_vectorizer.json')
    state_vectorizer = StateVectorizer.from_json(_load_json(vectorizer_path))
    target_mean, target_std = _load_target_stats(_artifact_path(checkpoint_path, 'target_stats.json'))
    action_space = load_action_space(args.action_space)
    fields = action_vector_fields(action_space)
    ranges = action_field_ranges(action_space)
    if len(fields) != int(target_mean.shape[0]):
        raise ValueError(
            f'action-space field count {len(fields)} does not match checkpoint output '
            f'dim {int(target_mean.shape[0])}'
        )

    _set_seed(torch_module, args.seed)
    device = _choose_device(torch_module, args.device)
    model = _build_model(
        torch_module,
        vocab_size=len(vocab),
        state_dim=state_vectorizer.dim,
        output_dim=int(target_mean.shape[0]),
    ).to(device)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()
    if device == 'cuda' and torch_module.cuda.is_available():
        torch_module.cuda.reset_peak_memory_stats()

    image_width = _safe_int(config.get('image_width'), args.image_width)
    image_height = _safe_int(config.get('image_height'), args.image_height)
    max_tokens = _safe_int(config.get('max_tokens'), args.max_tokens)
    mean_tensor = torch_module.as_tensor(target_mean, dtype=torch_module.float32, device=device)
    std_tensor = torch_module.as_tensor(target_std, dtype=torch_module.float32, device=device)

    split_names = [
        name.strip()
        for name in str(args.eval_splits or '').split(',')
        if name.strip()
    ]
    if not split_names:
        raise ValueError('--eval-splits selected no splits')

    started = time.perf_counter()
    splits: dict[str, Any] = {}
    all_records: list[dict[str, Any]] = []
    with torch_module.no_grad():
        for split_name in split_names:
            split_file = splits_dir / f'{split_name}.jsonl'
            rows = _row_limit(_iter_jsonl(split_file), args.limit_eval_rows)
            if not rows:
                raise ValueError(f'{split_name} split is empty')
            validate_model_input_rows(rows, split_name=split_name)
            image_report = image_integrity_report(
                rows,
                dataset_root,
                split_name=split_name,
                allow_blank_images=args.allow_blank_images,
            )
            family_lookup = episode_family_lookup(rows)
            split_records: list[dict[str, Any]] = []
            for sample_index, row in enumerate(rows):
                cycle_start = time.perf_counter()
                text, state, image = _row_tensor_inputs(
                    torch_module,
                    row,
                    dataset_root=dataset_root,
                    vocab=vocab,
                    state_vectorizer=state_vectorizer,
                    max_tokens=max_tokens,
                    image_width=image_width,
                    image_height=image_height,
                    allow_blank_images=args.allow_blank_images,
                    device=device,
                )
                if device == 'cuda' and torch_module.cuda.is_available():
                    torch_module.cuda.synchronize()
                inference_start = time.perf_counter()
                pred_norm = model(text, state, image)
                if device == 'cuda' and torch_module.cuda.is_available():
                    torch_module.cuda.synchronize()
                inference_latency = time.perf_counter() - inference_start
                pred = (pred_norm[0] * std_tensor + mean_tensor).detach().cpu().numpy()
                true_raw = action_vector(row)[: len(fields)]
                pred_raw = np.asarray(pred[: len(fields)], dtype=np.float32)
                true_quantized = baseline_quantize_action(true_raw, fields, ranges)
                pred_quantized = baseline_quantize_action(pred_raw, fields, ranges)
                decision = offline_supervisor_decision(pred_quantized)
                record = {
                    'split': split_name,
                    'sample_index': sample_index,
                    'episode_id': str(row.get('episode_id') or ''),
                    'task': str(row.get('task') or ''),
                    'task_family': family_from_row(row, family_lookup),
                    'true_raw': true_raw.tolist(),
                    'pred_raw': pred_raw.tolist(),
                    'true_quantized': true_quantized,
                    'pred_quantized': pred_quantized,
                    'supervisor_decision': decision,
                    'inference_latency_seconds': inference_latency,
                    'cycle_time_seconds': time.perf_counter() - cycle_start,
                }
                split_records.append(record)
                if args.progress_every > 0 and (sample_index + 1) % args.progress_every == 0:
                    print(
                        f'evaluated {split_name} {sample_index + 1}/{len(rows)} samples',
                        flush=True,
                    )

            split_prediction_path = output_dir / f'{split_name}_predictions.csv'
            _write_prediction_csv(split_prediction_path, split_records, fields)
            split_metrics = action_metrics(
                split_records,
                fields,
                speed_tolerance=args.speed_tolerance,
            )
            splits[split_name] = {
                'source_file': baseline_file_fingerprint(split_file),
                'row_fingerprint': baseline_rows_fingerprint(rows),
                'rows': len(rows),
                'image_integrity': image_report,
                'class_balance': class_balance_report(rows, expected_dim=len(fields)),
                'metrics': split_metrics,
                'family_metrics': action_metrics_by_family(
                    split_records,
                    fields,
                    speed_tolerance=args.speed_tolerance,
                ),
                'predictions_csv': str(split_prediction_path),
            }
            all_records.extend(split_records)

    artifact_paths = {
        'checkpoint': checkpoint_path,
        'vocab': _artifact_path(checkpoint_path, 'vocab.json'),
        'state_vectorizer': vectorizer_path,
        'target_stats': _artifact_path(checkpoint_path, 'target_stats.json'),
        'training_config': _artifact_path(checkpoint_path, 'training_config.json'),
    }
    leakage = detect_vectorizer_leakage(vectorizer_path)
    summary_path = output_dir / 'legacy_direct_action_eval.json'
    summary = {
        'tool': 'room_315_vla_train_local',
        'workflow_id': BASELINE_ID,
        'baseline_purpose': (
            'small custom direct-action action_vector checkpoint evaluation; no PlanSys2, '
            'execution, re-observation, or replanning loop'
        ),
        'checkpoint': str(checkpoint_path),
        'checkpoint_epoch': checkpoint.get('epoch'),
        'checkpoint_metrics': checkpoint.get('metrics'),
        'checkpoint_training_config': config,
        'dataset_root': str(dataset_root),
        'splits_dir': str(splits_dir),
        'evaluated_splits': split_names,
        'speed_tolerance': args.speed_tolerance,
        'seed': args.seed,
        'device': device,
        'allow_blank_images': bool(args.allow_blank_images),
        'debug_blank_image_mode': bool(args.allow_blank_images),
        'production_feature_source': 'model_input only',
        'feature_purity': {
            'production_features_from': 'model_input',
            'row_level_metadata_used_as_features': [],
            'debug_or_ablation_features': ['allow_blank_images'] if args.allow_blank_images else [],
        },
        'comparison_validity': {
            **leakage,
            'validity_label': (
                'valid-for-comparison'
                if leakage.get('comparison_valid') is True
                else 'invalid-for-comparison'
                if leakage.get('comparison_valid') is False
                else 'unknown'
            ),
            'legacy_training_config': not bool(config.get('dataset_report')),
            'legacy_training_config_reason': (
                'training_config predates dataset_report/image-integrity metadata'
                if not bool(config.get('dataset_report'))
                else 'training_config contains hardened dataset report pointer'
            ),
        },
        'artifact_fingerprints': {
            name: baseline_file_fingerprint(path)
            for name, path in artifact_paths.items()
            if path.exists()
        },
        'action_vector_fields': fields,
        'splits': splits,
        'aggregate_metrics': action_metrics(
            all_records,
            fields,
            speed_tolerance=args.speed_tolerance,
        ),
        'aggregate_family_metrics': action_metrics_by_family(
            all_records,
            fields,
            speed_tolerance=args.speed_tolerance,
        ),
        'peak_gpu_memory': _peak_gpu_memory(torch_module, device),
        'elapsed_seconds': round(time.perf_counter() - started, 3),
        'summary_json': str(summary_path),
    }
    summary_path.write_text(_pretty_json(summary) + '\n', encoding='utf-8')
    return summary


def train_local(args: argparse.Namespace) -> dict[str, Any]:
    torch_module = _require_torch()
    splits_dir = args.splits_dir.expanduser().resolve()
    dataset_root = _resolve_dataset_root(args.dataset_root, splits_dir)
    train_file_path = splits_dir / args.train_file
    val_file_path = splits_dir / args.val_file
    train_rows = _row_limit(_iter_jsonl(train_file_path), args.limit_train_rows)
    val_rows = _row_limit(_iter_jsonl(val_file_path), args.limit_val_rows)
    if not train_rows:
        raise ValueError('train split is empty')
    if not val_rows:
        raise ValueError('validation split is empty')

    train_model_input = validate_model_input_rows(train_rows, split_name='train')
    val_model_input = validate_model_input_rows(val_rows, split_name='val')
    train_image_integrity = image_integrity_report(
        train_rows,
        dataset_root,
        split_name='train',
        allow_blank_images=args.allow_blank_images,
    )
    val_image_integrity = image_integrity_report(
        val_rows,
        dataset_root,
        split_name='val',
        allow_blank_images=args.allow_blank_images,
    )

    _set_seed(torch_module, args.seed)
    device = _choose_device(torch_module, args.device)
    vocab = build_vocab(
        train_rows,
        max_vocab_size=args.max_vocab_size,
        min_freq=args.min_token_frequency,
    )
    state_vectorizer = StateVectorizer.fit(train_rows)
    target_mean, target_std = target_stats(train_rows)
    output_dim = int(target_mean.shape[0])
    output_dir = args.output_dir.expanduser().resolve()
    dataset_report = {
        'tool': 'room_315_vla_train_local',
        'baseline_purpose': (
            'small custom Room 315 direct-action action_vector behavior-cloning baseline'
        ),
        'source_files': {
            'train': _file_fingerprint(train_file_path),
            'val': _file_fingerprint(val_file_path),
        },
        'row_fingerprint': {
            'train': _rows_fingerprint(train_rows),
            'val': _rows_fingerprint(val_rows),
        },
        'model_input_integrity': {
            'train': train_model_input,
            'val': val_model_input,
        },
        'camera_completeness': {
            'train': train_image_integrity,
            'val': val_image_integrity,
        },
        'class_balance': {
            'train': class_balance_report(train_rows, expected_dim=output_dim),
            'val': class_balance_report(val_rows, expected_dim=output_dim),
        },
        'split_integrity': split_integrity_report(train_rows, val_rows),
        'feature_purity': {
            'production_features_from': 'model_input',
            'row_level_metadata_used_as_features': [],
            'debug_or_ablation_features': [],
        },
    }
    config = {
        'tool': 'room_315_vla_train_local',
        'baseline_purpose': (
            'small custom Room 315 direct-action action_vector behavior-cloning baseline'
        ),
        'dataset_root': str(dataset_root),
        'splits_dir': str(splits_dir),
        'train_file': args.train_file,
        'val_file': args.val_file,
        'seed': args.seed,
        'device': device,
        'epochs': args.epochs,
        'batch_size': args.batch_size,
        'learning_rate': args.learning_rate,
        'weight_decay': args.weight_decay,
        'image_width': args.image_width,
        'image_height': args.image_height,
        'max_tokens': args.max_tokens,
        'max_vocab_size': args.max_vocab_size,
        'train_rows': len(train_rows),
        'val_rows': len(val_rows),
        'output_dim': output_dim,
        'state_dim': state_vectorizer.dim,
        'vocab_size': len(vocab),
        'allow_blank_images': bool(args.allow_blank_images),
        'debug_blank_image_mode': bool(args.allow_blank_images),
        'production_feature_source': 'model_input only',
        'dataset_report': str(output_dir / 'dataset_report.json'),
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'dataset_report.json').write_text(
        _pretty_json(dataset_report) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'vocab.json').write_text(_pretty_json(vocab) + '\n', encoding='utf-8')
    (output_dir / 'state_vectorizer.json').write_text(
        _pretty_json(state_vectorizer.to_json()) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'target_stats.json').write_text(
        _pretty_json({
            'mean': target_mean.tolist(),
            'std': target_std.tolist(),
        }) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'training_config.json').write_text(_pretty_json(config) + '\n', encoding='utf-8')

    train_dataset = Room315EventDataset(
        train_rows,
        dataset_root=dataset_root,
        vocab=vocab,
        state_vectorizer=state_vectorizer,
        target_mean=target_mean,
        target_std=target_std,
        max_tokens=args.max_tokens,
        image_width=args.image_width,
        image_height=args.image_height,
        torch_module=torch_module,
        allow_blank_images=args.allow_blank_images,
    )
    val_dataset = Room315EventDataset(
        val_rows,
        dataset_root=dataset_root,
        vocab=vocab,
        state_vectorizer=state_vectorizer,
        target_mean=target_mean,
        target_std=target_std,
        max_tokens=args.max_tokens,
        image_width=args.image_width,
        image_height=args.image_height,
        torch_module=torch_module,
        allow_blank_images=args.allow_blank_images,
    )
    train_loader = torch_module.utils.data.DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=(device == 'cuda'),
    )
    val_loader = torch_module.utils.data.DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=(device == 'cuda'),
    )

    model = _build_model(
        torch_module,
        vocab_size=len(vocab),
        state_dim=state_vectorizer.dim,
        output_dim=output_dim,
    ).to(device)
    optimizer = torch_module.optim.AdamW(
        model.parameters(),
        lr=args.learning_rate,
        weight_decay=args.weight_decay,
    )
    loss_fn = torch_module.nn.SmoothL1Loss()
    history: list[dict[str, Any]] = []
    best_val_loss = float('inf')
    best_epoch = 0

    for epoch in range(1, args.epochs + 1):
        model.train()
        running_loss = 0.0
        seen = 0
        for batch in train_loader:
            text = batch['text'].to(device)
            state = batch['state'].to(device)
            image = batch['image'].to(device)
            target = batch['target'].to(device)
            optimizer.zero_grad(set_to_none=True)
            pred = model(text, state, image)
            loss = loss_fn(pred, target)
            loss.backward()
            optimizer.step()
            batch_size = int(target.shape[0])
            running_loss += float(loss.item()) * batch_size
            seen += batch_size
        train_loss = running_loss / max(1, seen)
        val_metrics = _evaluate(
            torch_module,
            model,
            val_loader,
            device=device,
            target_mean=target_mean,
            target_std=target_std,
        )
        epoch_metrics = {
            'epoch': epoch,
            'train_loss': round(train_loss, 6),
            **{f'val_{key}': value for key, value in val_metrics.items()},
        }
        history.append(epoch_metrics)
        print(_pretty_json(epoch_metrics), flush=True)
        _save_checkpoint(
            torch_module,
            output_dir / 'last.pt',
            model=model,
            optimizer=optimizer,
            epoch=epoch,
            metrics=epoch_metrics,
            config=config,
        )
        if val_metrics['loss'] < best_val_loss:
            best_val_loss = val_metrics['loss']
            best_epoch = epoch
            _save_checkpoint(
                torch_module,
                output_dir / 'best.pt',
                model=model,
                optimizer=optimizer,
                epoch=epoch,
                metrics=epoch_metrics,
                config=config,
            )

    summary = {
        'tool': 'room_315_vla_train_local',
        'baseline_purpose': (
            'small custom Room 315 direct-action action_vector behavior-cloning baseline'
        ),
        'output_dir': str(output_dir),
        'best_checkpoint': str(output_dir / 'best.pt'),
        'last_checkpoint': str(output_dir / 'last.pt'),
        'best_epoch': best_epoch,
        'best_val_loss': best_val_loss,
        'history': history,
        'config': config,
        'dataset_report': dataset_report,
    }
    (output_dir / 'metrics.json').write_text(_pretty_json(summary) + '\n', encoding='utf-8')
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Train a small custom Room 315 direct-action baseline from train/val JSONL splits.'
        )
    )
    parser.add_argument(
        '--splits-dir',
        type=Path,
        default=DEFAULT_SPLITS_DIR,
        help='Split directory. Defaults to ROOM315_VLA_SPLITS_DIR or room315_local_training/splits.',
    )
    parser.add_argument(
        '--dataset-root',
        type=Path,
        default=None,
        help=(
            'Dataset root for relative image refs. Defaults to the split manifest '
            'dataset_root, ROOM315_VLA_DATASET_ROOT, or room315_payload_dataset.'
        ),
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=(
            'Output checkpoint directory. Defaults to ROOM315_LOCAL_BASELINE_OUTPUT_DIR '
            'or room315_local_training/checkpoints/v0.'
        ),
    )
    parser.add_argument(
        '--dataset-mode',
        choices=DATASET_MODES,
        default=DATASET_MODE_LEGACY_ACTION,
        help=(
            'legacy_action preserves the small custom 24-value direct-action baseline. '
            f'{DATASET_MODE_VISUAL_STATE} trains/evaluates image-to-visual-state labels from '
            f'separate *{VISUAL_LABEL_SUFFIX} sidecars.'
        ),
    )
    parser.add_argument('--train-file', default='train.jsonl')
    parser.add_argument('--val-file', default='val.jsonl')
    parser.add_argument(
        '--visual-train-labels',
        type=Path,
        default=None,
        help='Visual-state train label JSONL. Defaults to sibling train_visual_labels.jsonl.',
    )
    parser.add_argument(
        '--visual-val-labels',
        type=Path,
        default=None,
        help='Visual-state val label JSONL. Defaults to sibling val_visual_labels.jsonl.',
    )
    parser.add_argument('--epochs', type=int, default=5)
    parser.add_argument('--batch-size', type=int, default=8)
    parser.add_argument('--learning-rate', type=float, default=1e-3)
    parser.add_argument('--weight-decay', type=float, default=1e-4)
    parser.add_argument('--image-width', type=int, default=160)
    parser.add_argument('--image-height', type=int, default=120)
    parser.add_argument('--max-tokens', type=int, default=32)
    parser.add_argument('--max-vocab-size', type=int, default=2048)
    parser.add_argument('--min-token-frequency', type=int, default=1)
    parser.add_argument('--num-workers', type=int, default=2)
    parser.add_argument('--device', default='auto', help='auto, cuda, or cpu')
    parser.add_argument('--seed', type=int, default=13)
    parser.add_argument('--limit-train-rows', type=int, default=None)
    parser.add_argument('--limit-val-rows', type=int, default=None)
    parser.add_argument(
        '--allow-blank-images',
        action='store_true',
        help='Debug only: substitute zero tensors for missing/unreadable required images.',
    )
    parser.add_argument(
        '--eval-checkpoint',
        type=Path,
        default=None,
        help='Evaluate an existing local checkpoint instead of training.',
    )
    parser.add_argument(
        '--eval-output-dir',
        type=Path,
        default=None,
        help='Output directory for --eval-checkpoint artifacts.',
    )
    parser.add_argument(
        '--eval-splits',
        default='train,val,test',
        help='Comma-separated JSONL split names to evaluate with --eval-checkpoint.',
    )
    parser.add_argument(
        '--limit-eval-rows',
        type=int,
        default=None,
        help='Optional debug row limit per split during --eval-checkpoint.',
    )
    parser.add_argument('--action-space', type=Path, default=DEFAULT_ACTION_SPACE)
    parser.add_argument('--speed-tolerance', type=float, default=0.015)
    parser.add_argument('--progress-every', type=int, default=100)
    args = parser.parse_args()
    if args.eval_checkpoint is not None:
        summary = evaluate_checkpoint(args)
    elif args.dataset_mode == DATASET_MODE_VISUAL_STATE:
        summary = train_visual_state(args)
    else:
        summary = train_local(args)
    print(_pretty_json(summary))


if __name__ == '__main__':
    main()
