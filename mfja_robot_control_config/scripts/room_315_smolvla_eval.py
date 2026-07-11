#!/usr/bin/env python3

import argparse
import csv
import json
import math
import random
import time
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import numpy as np
import yaml


DEFAULT_CHECKPOINT = Path(
    '/home/tiago/room315_local_training/smolvla_runs/'
    'v1_pretrained_compact32/checkpoints/002000/pretrained_model'
)
DEFAULT_OUTPUT_DIR = Path('/home/tiago/room315_local_training/smolvla_eval/v1_pretrained_compact32')
DEFAULT_ACTION_SPACE = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'room_315_vla'
    / 'action_space.yaml'
)
DEFAULT_DATASETS = {
    'train': {
        'repo_id': 'room315/room315_vla_train_compact32',
        'root': '/home/tiago/room315_local_training/lerobot/room315_vla_train_compact32',
    },
    'val': {
        'repo_id': 'room315/room315_vla_val_compact32',
        'root': '/home/tiago/room315_local_training/lerobot/room315_vla_val_compact32',
    },
    'test': {
        'repo_id': 'room315/room315_vla_test_compact32',
        'root': '/home/tiago/room315_local_training/lerobot/room315_vla_test_compact32',
    },
}
SPEED_FIELD = 'speed_mps'
SAMPLE_FIELDS = (
    'sample_index',
    'dataset_index',
    'episode_index',
    'frame_index',
    'task',
    'true_primitive_id',
    'pred_primitive_id',
    'true_side_id',
    'pred_side_id',
    'true_target_id',
    'pred_target_id',
    'true_speed_mps',
    'pred_speed_mps',
    'exact_discrete_action',
    'exact_action',
    'action_mae',
)


def _pretty_json(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True)


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


def action_vector_fields(action_space: dict[str, Any]) -> list[str]:
    return [str(field) for field in action_space['action_vector_fields']]


def quantize_action(
    values: np.ndarray,
    fields: list[str],
    ranges: dict[str, tuple[int, int]],
) -> np.ndarray:
    quantized = np.asarray(values, dtype=np.float32).copy()
    for index, field in enumerate(fields):
        if field == SPEED_FIELD:
            continue
        lower, upper = ranges.get(field, (0, 1))
        quantized[index] = min(max(round(float(quantized[index])), lower), upper)
    return quantized


def field_accuracy(
    true_values: np.ndarray,
    pred_values: np.ndarray,
    fields: list[str],
    field: str,
) -> float | None:
    if field not in fields or len(true_values) == 0:
        return None
    index = fields.index(field)
    return float(np.mean(true_values[:, index] == pred_values[:, index]))


def grouped_field_accuracy(
    true_values: np.ndarray,
    pred_values: np.ndarray,
    fields: list[str],
    prefix: str,
) -> float | None:
    indexes = [index for index, field in enumerate(fields) if field.startswith(prefix)]
    if not indexes or len(true_values) == 0:
        return None
    return float(np.mean(true_values[:, indexes] == pred_values[:, indexes]))


def summarise_predictions(
    records: list[dict[str, Any]],
    fields: list[str],
    *,
    speed_tolerance: float,
) -> dict[str, Any]:
    if not records:
        raise ValueError('cannot summarise an empty evaluation record set')

    true_quantized = np.asarray([record['true_quantized'] for record in records], dtype=np.float32)
    pred_quantized = np.asarray([record['pred_quantized'] for record in records], dtype=np.float32)
    true_raw = np.asarray([record['true_raw'] for record in records], dtype=np.float32)
    pred_raw = np.asarray([record['pred_raw'] for record in records], dtype=np.float32)

    discrete_indexes = [index for index, field in enumerate(fields) if field != SPEED_FIELD]
    speed_index = fields.index(SPEED_FIELD)
    discrete_matches = pred_quantized[:, discrete_indexes] == true_quantized[:, discrete_indexes]
    exact_discrete = np.all(discrete_matches, axis=1)
    speed_abs_error = np.abs(pred_raw[:, speed_index] - true_raw[:, speed_index])
    exact_action = exact_discrete & (speed_abs_error <= speed_tolerance)
    abs_error = np.abs(pred_raw - true_raw)

    per_field = {}
    for index, field in enumerate(fields):
        metric: dict[str, Any] = {'mae': float(np.mean(abs_error[:, index]))}
        if field != SPEED_FIELD:
            metric['accuracy'] = float(np.mean(pred_quantized[:, index] == true_quantized[:, index]))
        per_field[field] = metric

    primitive_confusion = Counter(
        f'{_safe_int(true)}->{_safe_int(pred)}'
        for true, pred in zip(
            true_quantized[:, fields.index('primitive_id')],
            pred_quantized[:, fields.index('primitive_id')],
        )
    )

    return {
        'samples': len(records),
        'action_mae': float(np.mean(abs_error)),
        'action_rmse': float(np.sqrt(np.mean(np.square(pred_raw - true_raw)))),
        'discrete_field_accuracy': float(np.mean(discrete_matches)),
        'exact_discrete_action_accuracy': float(np.mean(exact_discrete)),
        'exact_action_accuracy': float(np.mean(exact_action)),
        'speed_mae': float(np.mean(speed_abs_error)),
        'speed_within_tolerance_accuracy': float(np.mean(speed_abs_error <= speed_tolerance)),
        'primitive_accuracy': field_accuracy(true_quantized, pred_quantized, fields, 'primitive_id'),
        'side_accuracy': field_accuracy(true_quantized, pred_quantized, fields, 'side_id'),
        'shuttle_index_accuracy': field_accuracy(true_quantized, pred_quantized, fields, 'shuttle_index'),
        'switch_mask_accuracy': grouped_field_accuracy(true_quantized, pred_quantized, fields, 'switch_mask_'),
        'switch_value_accuracy': grouped_field_accuracy(true_quantized, pred_quantized, fields, 'switch_value_'),
        'stopper_mask_accuracy': grouped_field_accuracy(true_quantized, pred_quantized, fields, 'stopper_mask_'),
        'stopper_value_accuracy': grouped_field_accuracy(true_quantized, pred_quantized, fields, 'stopper_value_'),
        'wait_condition_accuracy': field_accuracy(
            true_quantized, pred_quantized, fields, 'wait_condition_id'
        ),
        'target_id_accuracy': field_accuracy(true_quantized, pred_quantized, fields, 'target_id'),
        'reason_id_accuracy': field_accuracy(true_quantized, pred_quantized, fields, 'reason_id'),
        'coordination_mode_accuracy': field_accuracy(
            true_quantized, pred_quantized, fields, 'coordination_mode'
        ),
        'per_field': per_field,
        'primitive_confusion': dict(primitive_confusion.most_common()),
    }


def _require_lerobot():
    try:
        import torch
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.policies.factory import make_pre_post_processors
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
    except ImportError as exc:
        raise SystemExit(
            'LeRobot/SmolVLA dependencies are required for evaluation.\n'
            'Activate the local training environment first:\n'
            '  source /home/tiago/room315_local_training/venv/bin/activate'
        ) from exc
    return torch, LeRobotDataset, SmolVLAPolicy, make_pre_post_processors


def _sample_indexes(dataset_size: int, *, max_samples: int | None, stride: int) -> list[int]:
    if stride < 1:
        raise ValueError('--stride must be >= 1')
    indexes = list(range(0, dataset_size, stride))
    if max_samples is not None and max_samples > 0:
        indexes = indexes[:max_samples]
    return indexes


def _make_policy_batch(torch: Any, item: dict[str, Any]) -> dict[str, Any]:
    batch: dict[str, Any] = {}
    for key, value in item.items():
        if key == 'action':
            continue
        if torch.is_tensor(value):
            batch[key] = value.unsqueeze(0)
        elif key == 'task':
            batch[key] = [str(value)]
    return batch


def _tensor_to_numpy(value: Any) -> np.ndarray:
    return value.detach().cpu().numpy().astype(np.float32)


def evaluate_smolvla(
    *,
    checkpoint: Path,
    dataset_repo_id: str,
    dataset_root: Path,
    output_dir: Path,
    split_name: str,
    action_space_path: Path = DEFAULT_ACTION_SPACE,
    max_samples: int | None = None,
    stride: int = 1,
    seed: int = 13,
    speed_tolerance: float = 0.015,
    progress_every: int = 25,
) -> dict[str, Any]:
    checkpoint = checkpoint.expanduser().resolve()
    dataset_root = dataset_root.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    if not checkpoint.exists():
        raise FileNotFoundError(f'checkpoint not found: {checkpoint}')
    if not dataset_root.exists():
        raise FileNotFoundError(f'dataset root not found: {dataset_root}')

    action_space = load_action_space(action_space_path)
    fields = action_vector_fields(action_space)
    ranges = action_field_ranges(action_space)
    torch, LeRobotDataset, SmolVLAPolicy, make_pre_post_processors = _require_lerobot()

    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)

    started = time.perf_counter()
    dataset = LeRobotDataset(dataset_repo_id, root=dataset_root)
    indexes = _sample_indexes(len(dataset), max_samples=max_samples, stride=stride)
    if not indexes:
        raise ValueError('no samples selected for evaluation')

    policy = SmolVLAPolicy.from_pretrained(checkpoint)
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(policy.config, pretrained_path=str(checkpoint))

    records: list[dict[str, Any]] = []
    task_counts: Counter[str] = Counter()
    per_task_records: defaultdict[str, list[dict[str, Any]]] = defaultdict(list)
    with torch.no_grad():
        for sample_index, dataset_index in enumerate(indexes):
            item = dataset[dataset_index]
            task = str(item.get('task', ''))
            task_counts[task] += 1
            true_raw = _tensor_to_numpy(item['action']).reshape(-1)
            batch = preprocessor(_make_policy_batch(torch, item))
            policy.reset()
            predicted = policy.select_action(dict(batch))
            predicted = postprocessor(predicted)
            pred_raw = _tensor_to_numpy(predicted).reshape(-1)[: len(fields)]
            true_raw = true_raw[: len(fields)]
            true_quantized = quantize_action(true_raw, fields, ranges)
            pred_quantized = quantize_action(pred_raw, fields, ranges)
            record = {
                'sample_index': sample_index,
                'dataset_index': dataset_index,
                'episode_index': _safe_int(_tensor_to_numpy(item['episode_index']).item()),
                'frame_index': _safe_int(_tensor_to_numpy(item['frame_index']).item()),
                'task': task,
                'true_raw': true_raw,
                'pred_raw': pred_raw,
                'true_quantized': true_quantized,
                'pred_quantized': pred_quantized,
            }
            records.append(record)
            per_task_records[task].append(record)
            if progress_every > 0 and (sample_index + 1) % progress_every == 0:
                print(f'evaluated {sample_index + 1}/{len(indexes)} samples', flush=True)

    summary_metrics = summarise_predictions(records, fields, speed_tolerance=speed_tolerance)
    task_metrics = {
        task: summarise_predictions(task_records, fields, speed_tolerance=speed_tolerance)
        for task, task_records in sorted(per_task_records.items())
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / f'{split_name}_smolvla_eval.json'
    samples_path = output_dir / f'{split_name}_smolvla_predictions.csv'
    speed_index = fields.index(SPEED_FIELD)
    primitive_index = fields.index('primitive_id')
    side_index = fields.index('side_id')
    target_index = fields.index('target_id')
    with samples_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=SAMPLE_FIELDS)
        writer.writeheader()
        for record in records:
            true_q = record['true_quantized']
            pred_q = record['pred_quantized']
            true_raw = record['true_raw']
            pred_raw = record['pred_raw']
            discrete_indexes = [index for index, field in enumerate(fields) if field != SPEED_FIELD]
            exact_discrete = bool(np.all(true_q[discrete_indexes] == pred_q[discrete_indexes]))
            exact_action = exact_discrete and abs(
                float(true_raw[speed_index]) - float(pred_raw[speed_index])
            ) <= speed_tolerance
            writer.writerow({
                'sample_index': record['sample_index'],
                'dataset_index': record['dataset_index'],
                'episode_index': record['episode_index'],
                'frame_index': record['frame_index'],
                'task': record['task'],
                'true_primitive_id': int(true_q[primitive_index]),
                'pred_primitive_id': int(pred_q[primitive_index]),
                'true_side_id': int(true_q[side_index]),
                'pred_side_id': int(pred_q[side_index]),
                'true_target_id': int(true_q[target_index]),
                'pred_target_id': int(pred_q[target_index]),
                'true_speed_mps': round(float(true_raw[speed_index]), 5),
                'pred_speed_mps': round(float(pred_raw[speed_index]), 5),
                'exact_discrete_action': exact_discrete,
                'exact_action': exact_action,
                'action_mae': round(float(np.mean(np.abs(pred_raw - true_raw))), 6),
            })

    summary = {
        'checkpoint': str(checkpoint),
        'dataset_repo_id': dataset_repo_id,
        'dataset_root': str(dataset_root),
        'split': split_name,
        'dataset_frames': len(dataset),
        'dataset_episodes': dataset.num_episodes,
        'evaluated_samples': len(records),
        'stride': stride,
        'seed': seed,
        'speed_tolerance': speed_tolerance,
        'elapsed_seconds': round(time.perf_counter() - started, 3),
        'action_vector_fields': fields,
        'task_counts': dict(task_counts),
        'metrics': summary_metrics,
        'task_metrics': task_metrics,
        'summary_json': str(summary_path),
        'predictions_csv': str(samples_path),
    }
    summary_path.write_text(_pretty_json(summary) + '\n', encoding='utf-8')
    print(_pretty_json(summary))
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Evaluate a local Room 315 SmolVLA checkpoint on a LeRobot split.'
    )
    parser.add_argument('--checkpoint', type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument('--split', choices=sorted(DEFAULT_DATASETS), default='val')
    parser.add_argument('--dataset-repo-id', default=None)
    parser.add_argument('--dataset-root', type=Path, default=None)
    parser.add_argument('--output-dir', type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument('--action-space', type=Path, default=DEFAULT_ACTION_SPACE)
    parser.add_argument('--max-samples', type=int, default=None)
    parser.add_argument('--stride', type=int, default=1)
    parser.add_argument('--seed', type=int, default=13)
    parser.add_argument('--speed-tolerance', type=float, default=0.015)
    parser.add_argument('--progress-every', type=int, default=25)
    args = parser.parse_args()

    defaults = DEFAULT_DATASETS[args.split]
    evaluate_smolvla(
        checkpoint=args.checkpoint,
        dataset_repo_id=args.dataset_repo_id or defaults['repo_id'],
        dataset_root=args.dataset_root or Path(defaults['root']),
        output_dir=args.output_dir,
        split_name=args.split,
        action_space_path=args.action_space,
        max_samples=args.max_samples,
        stride=args.stride,
        seed=args.seed,
        speed_tolerance=args.speed_tolerance,
        progress_every=args.progress_every,
    )


if __name__ == '__main__':
    main()
