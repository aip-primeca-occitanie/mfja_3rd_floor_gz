#!/usr/bin/env python3
"""Read-only post-pilot error analysis for the Room 315 visual-state model."""

import argparse
import csv
import hashlib
import importlib
import json
import math
import random
import statistics
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Callable, Iterable

import numpy as np
from PIL import Image, ImageDraw, ImageFont


EXPECTED_CHECKPOINT_SHA256 = (
    '61acabfeb75ca29e4612e51ccdcf233723d9b22c3600f396d9a5cf50c8487f73'
)
DEFAULT_PILOT_RESULTS = Path('/home/tiago/Downloads/kairos_room315_h200_pilot_results')
DEFAULT_PACKAGE_ROOT = Path('/home/tiago/kairos_room315_h200_pilot_seed31520260726')
DEFAULT_OUTPUT_NAME = f'error_analysis_baseline_{EXPECTED_CHECKPOINT_SHA256[:12]}'
SPLITS = ('val', 'test')
CAMERAS = ('left_rail_rgb', 'right_rail_rgb')
CLASSIFICATION_TASKS = (
    'loaded_state',
    'rail_side',
    'block_segment',
    'full_location',
    'identity_slot_conditioned',
    'identity_global_structurally_constrained',
)
WORST_CASE_CATEGORIES = ('s_m', 's_ratio', 'bbox_iou', 'location', 'loaded_state')


class ErrorAnalysisError(RuntimeError):
    """Raised when a frozen-pilot audit invariant is violated."""


def pretty_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def file_fingerprint(path: Path) -> dict[str, Any]:
    return {
        'path': str(path.resolve()),
        'bytes': path.stat().st_size,
        'sha256': sha256_file(path),
    }


def read_json(path: Path) -> dict[str, Any]:
    parsed = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(parsed, dict):
        raise ErrorAnalysisError(f'{path} does not contain a JSON object')
    return parsed


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            parsed = json.loads(line)
            if not isinstance(parsed, dict):
                raise ErrorAnalysisError(f'{path}:{line_number} is not a JSON object')
            rows.append(parsed)
    return rows


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(pretty_json(value) + '\n', encoding='utf-8')


def write_jsonl(path: Path, records: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        for record in records:
            stream.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + '\n')


def _csv_value(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return json.dumps(value, ensure_ascii=False, sort_keys=True)


def write_records_csv(path: Path, records: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = sorted({key for record in records for key in record})
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for record in records:
            writer.writerow({key: _csv_value(record.get(key)) for key in fieldnames})


def snapshot_existing_files(
    roots: dict[str, Path],
    *,
    excluded_roots: Iterable[Path] = (),
) -> dict[str, Any]:
    exclusions = [path.resolve() for path in excluded_roots]
    files: dict[str, Any] = {}
    for root_name, root in sorted(roots.items()):
        resolved_root = root.resolve()
        if not resolved_root.exists():
            raise ErrorAnalysisError(f'baseline artifact root does not exist: {resolved_root}')
        for path in sorted(item for item in resolved_root.rglob('*') if item.is_file()):
            resolved_path = path.resolve()
            if any(exclusion == resolved_path or exclusion in resolved_path.parents for exclusion in exclusions):
                continue
            key = f'{root_name}/{path.relative_to(resolved_root).as_posix()}'
            files[key] = file_fingerprint(path)
    return {
        'roots': {name: str(path.resolve()) for name, path in sorted(roots.items())},
        'file_count': len(files),
        'files': files,
    }


def verify_snapshot_unchanged(snapshot: dict[str, Any]) -> dict[str, Any]:
    missing = []
    changed = []
    for key, original in snapshot.get('files', {}).items():
        path = Path(original['path'])
        if not path.is_file():
            missing.append(key)
            continue
        current = file_fingerprint(path)
        if (
            current['bytes'] != original['bytes']
            or current['sha256'] != original['sha256']
        ):
            changed.append({
                'key': key,
                'before': original,
                'after': current,
            })
    return {
        'status': 'PASS' if not missing and not changed else 'FAIL',
        'audited_file_count': len(snapshot.get('files', {})),
        'missing_files': missing,
        'changed_files': changed,
    }


def _identity_suffix(identity: str) -> int | None:
    text = str(identity or '').strip().upper()
    if len(text) == 2 and text[0] in {'L', 'R'} and text[1].isdigit():
        return int(text[1])
    return None


def _canonical(value: Any) -> str | None:
    text = str(value or '').strip()
    return text.upper() if text else None


def _category_from_name(name: str) -> str:
    return name.split('==', 1)[1].strip().upper()


def _categorical_indexes(names: list[str], base: str) -> list[int]:
    prefix = f'{base}=='
    return [index for index, name in enumerate(names) if name.startswith(prefix)]


def _numeric_index(names: list[str], name: str) -> int:
    try:
        return names.index(name)
    except ValueError as exc:
        raise ErrorAnalysisError(f'visual vector lacks required target {name!r}') from exc


def categorical_scores(
    prediction: np.ndarray,
    names: list[str],
    base: str,
) -> dict[str, float]:
    indexes = _categorical_indexes(names, base)
    if not indexes:
        raise ErrorAnalysisError(f'visual vector lacks categorical target {base!r}')
    return {
        _category_from_name(names[index]): float(prediction[index])
        for index in indexes
    }


def score_decision(scores: dict[str, float]) -> tuple[str, float, list[dict[str, Any]]]:
    ordered = sorted(scores.items(), key=lambda item: (-item[1], item[0]))
    predicted = ordered[0][0]
    margin = ordered[0][1] - ordered[1][1] if len(ordered) >= 2 else math.inf
    return (
        predicted,
        float(margin),
        [
            {'class': name, 'score': float(score)}
            for name, score in ordered[:2]
        ],
    )


def bbox_iou_xywh(first: list[float], second: list[float]) -> float:
    if len(first) != 4 or len(second) != 4:
        raise ValueError('bbox IoU requires [x, y, width, height] boxes')
    ax, ay, aw, ah = (float(value) for value in first)
    bx, by, bw, bh = (float(value) for value in second)
    aw = max(0.0, aw)
    ah = max(0.0, ah)
    bw = max(0.0, bw)
    bh = max(0.0, bh)
    intersection_width = max(0.0, min(ax + aw, bx + bw) - max(ax, bx))
    intersection_height = max(0.0, min(ay + ah, by + bh) - max(ay, by))
    intersection = intersection_width * intersection_height
    union = aw * ah + bw * bh - intersection
    return float(intersection / union) if union > 0.0 else 0.0


def bbox_errors(
    ground_truth: list[float],
    prediction: list[float],
) -> dict[str, Any]:
    coordinate_errors = [
        float(predicted) - float(actual)
        for actual, predicted in zip(ground_truth, prediction)
    ]
    gt_center = (
        float(ground_truth[0]) + float(ground_truth[2]) / 2.0,
        float(ground_truth[1]) + float(ground_truth[3]) / 2.0,
    )
    pred_center = (
        float(prediction[0]) + float(prediction[2]) / 2.0,
        float(prediction[1]) + float(prediction[3]) / 2.0,
    )
    width_error = float(prediction[2]) - float(ground_truth[2])
    height_error = float(prediction[3]) - float(ground_truth[3])
    return {
        'bbox_coordinate_errors': coordinate_errors,
        'bbox_absolute_coordinate_errors': [abs(value) for value in coordinate_errors],
        'bbox_center_error': math.hypot(
            pred_center[0] - gt_center[0],
            pred_center[1] - gt_center[1],
        ),
        'bbox_size_error': math.hypot(width_error, height_error),
        'bbox_width_error': width_error,
        'bbox_height_error': height_error,
        'bbox_iou': bbox_iou_xywh(ground_truth, prediction),
    }


def scenario_type_from_episode(episode_id: str) -> str:
    parts = str(episode_id).split('_')
    if len(parts) >= 4 and parts[0] == 'visual' and parts[1].isdigit():
        return '_'.join(parts[2:-1])
    return 'unknown'


def fixed_entry_indexes(label_names: list[str]) -> tuple[int, ...]:
    """Derive shuttle entries from serialized schema targets, never a literal 3."""
    indexes = {
        int(parts[1])
        for name in label_names
        for parts in (name.split('.', 2),)
        if len(parts) >= 3
        and parts[0] == 'shuttles'
        and parts[1].isdigit()
    }
    if not indexes:
        raise ErrorAnalysisError('vectorizer contains no shuttle entry targets')
    expected = set(range(max(indexes) + 1))
    if indexes != expected:
        raise ErrorAnalysisError(
            f'vectorizer shuttle entries are not contiguous: {sorted(indexes)}'
        )
    return tuple(sorted(indexes))


def audit_identity_semantics(
    split_labels: dict[str, list[dict[str, Any]]],
    label_names: list[str],
) -> dict[str, Any]:
    vector_classes: dict[str, list[str]] = {}
    observed: dict[str, Any] = {}
    violations = []
    order_patterns: dict[str, Counter[tuple[str, ...]]] = {}
    for slot in fixed_entry_indexes(label_names):
        base = f'shuttles.{slot}.visually_available_identity'
        vector_classes[str(slot)] = [
            _category_from_name(label_names[index])
            for index in _categorical_indexes(label_names, base)
        ]
    for split_name, labels in split_labels.items():
        slot_counts: dict[str, Counter[str]] = defaultdict(Counter)
        patterns: Counter[tuple[str, ...]] = Counter()
        for scenario_index, label in enumerate(labels):
            identities = tuple(
                _canonical(shuttle.get('visually_available_identity')) or 'UNKNOWN'
                for shuttle in label.get('shuttles', [])
            )
            patterns[identities] += 1
            for slot, identity in enumerate(identities):
                slot_counts[str(slot)][identity] += 1
                suffix = _identity_suffix(identity)
                if suffix != slot + 1:
                    violations.append({
                        'split': split_name,
                        'scenario_index': scenario_index,
                        'slot': slot,
                        'identity': identity,
                    })
        order_patterns[split_name] = patterns
        observed[split_name] = {
            'slot_identity_counts': {
                slot: dict(sorted(counts.items()))
                for slot, counts in sorted(slot_counts.items())
            },
            'ordered_identity_patterns': {
                '|'.join(pattern): count
                for pattern, count in sorted(patterns.items())
            },
        }
    vector_slot_constrained = all(
        set(classes).issubset({f'L{slot + 1}', f'R{slot + 1}'})
        and set(classes)
        for slot, classes in ((int(key), value) for key, value in vector_classes.items())
    )
    report = {
        'status': 'PASS',
        'metric_name_audited': 'identity_accuracy',
        'current_metric_semantics': (
            'mean accuracy across present shuttle slots; each slot performs a binary '
            'left-versus-right decision for a fixed numeric shuttle suffix'
        ),
        'is_true_unconstrained_six_class_metric': False,
        'is_slot_conditioned_left_right_metric': True,
        'metric_name_is_potentially_misleading': True,
        'recommended_metric_name': 'slot_conditioned_identity_accuracy',
        'vector_classes_by_slot': vector_classes,
        'vector_schema_is_slot_constrained': vector_slot_constrained,
        'observed_dataset_assignments': observed,
        'slot_assignment_violations': violations,
        'slot_assignment_rule': (
            'labels are normalized by sorting shuttle id; in this dataset, identity '
            'numeric suffix n always occupies output slot n-1'
        ),
        'slot_order_depends_on_ground_truth_identity': not violations,
        'global_identity_report_supported': True,
        'global_identity_report_limitation': (
            'global L1..R3 labels can be reconstructed from slot plus the slot-specific '
            'left/right output, but cross-numeric-suffix confusions are impossible by construction'
        ),
        'data_leakage_demonstrated': False,
        'data_leakage_statement': (
            'This audit demonstrates target-schema conditioning, not image/label leakage. '
            'No claim of data leakage is made.'
        ),
    }
    if not vector_slot_constrained or violations:
        report['status'] = 'FAIL'
    return report


def identity_audit_markdown(audit: dict[str, Any]) -> str:
    lines = [
        '# Room 315 identity-semantics audit',
        '',
        f"Status: **{audit['status']}**",
        '',
        'The reported `identity_accuracy` is not an unconstrained six-class shuttle-ID metric. '
        'It averages one binary decision for each present output slot:',
        '',
    ]
    for slot, classes in sorted(audit['vector_classes_by_slot'].items()):
        lines.append(f"- Slot {slot}: {' versus '.join(classes)}")
    lines.extend([
        '',
        'The label normalizer sorts shuttles by ground-truth ID, and the audited splits '
        'always place suffix `n` in slot `n-1`. Consequently, a six-class report can be '
        'reconstructed, but the output cannot confuse L1/R1 with L2/R2 or L3/R3. The '
        'recommended name for the existing score is `slot_conditioned_identity_accuracy`.',
        '',
        'This is evidence of a structurally constrained target schema. It is **not**, by '
        'itself, evidence of image/label data leakage.',
        '',
        '## Other metric-semantics findings',
        '',
        '- Legacy `location_accuracy` pools side and block decisions and is not '
        'full-location accuracy.',
        '- Legacy `bbox_mae` is a coordinate MAE and does not measure overlap.',
        '- Legacy `label_mae` combines unlike units and is not interpreted as a task metric.',
        '',
    ])
    return '\n'.join(lines)


def load_training_runtime(package_root: Path):
    scripts = package_root.resolve() / 'scripts'
    if str(scripts) not in sys.path:
        sys.path.insert(0, str(scripts))
    trainer = importlib.import_module('room_315_visual_state_train_local')
    dataset_module = importlib.import_module('room_315_visual_state_dataset')
    return trainer, dataset_module


def load_frozen_model(
    pilot_results: Path,
    package_root: Path,
    *,
    checkpoint_path: Path,
    expected_sha256: str,
    device_request: str,
    seed: int,
):
    actual_hash = sha256_file(checkpoint_path)
    if actual_hash != expected_sha256:
        raise ErrorAnalysisError(
            f'frozen checkpoint SHA-256 mismatch: expected {expected_sha256}, got {actual_hash}'
        )
    trainer, dataset_module = load_training_runtime(package_root)
    torch = trainer._require_torch()
    checkpoint = trainer._torch_load_checkpoint(torch, checkpoint_path)
    config = dict(checkpoint.get('config') or read_json(pilot_results / 'training_config.json'))
    vectorizer = dataset_module.VisualStateLabelVectorizer.from_json(
        read_json(pilot_results / 'visual_label_vectorizer.json')
    )
    target_stats = read_json(pilot_results / 'target_stats.json')
    target_mean = np.asarray(target_stats['mean'], dtype=np.float32)
    target_std = np.asarray(target_stats['std'], dtype=np.float32)
    trainer._set_seed(torch, seed)
    device = trainer._choose_device(torch, device_request)
    model, model_metadata = trainer._visual_model_from_config(
        torch,
        config=config,
        output_dim=vectorizer.dim,
    )
    model.load_state_dict(checkpoint['model_state_dict'], strict=True)
    model = model.to(device)
    model.eval()
    return {
        'torch': torch,
        'trainer': trainer,
        'dataset_module': dataset_module,
        'model': model,
        'model_metadata': model_metadata,
        'checkpoint': checkpoint,
        'checkpoint_sha256': actual_hash,
        'config': config,
        'vectorizer': vectorizer,
        'target_mean': target_mean,
        'target_std': target_std,
        'device': device,
    }


def _load_split(
    package_root: Path,
    split_name: str,
    dataset_module: Any,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    splits = package_root / 'dataset' / 'splits'
    rows = read_jsonl(splits / f'{split_name}.jsonl')
    label_rows = read_jsonl(splits / f'{split_name}_visual_labels.jsonl')
    if len(rows) != len(label_rows):
        raise ErrorAnalysisError(f'{split_name} row/label count mismatch')
    if [row.get('sample_id') for row in rows] != [
        row.get('sample_id') for row in label_rows
    ]:
        raise ErrorAnalysisError(f'{split_name} row/label order mismatch')
    labels = [
        dataset_module.normalize_visual_state_labels(label_row)
        for label_row in label_rows
    ]
    return rows, labels


def _image_paths(row: dict[str, Any], dataset_root: Path) -> dict[str, str]:
    refs = row.get('model_input', {}).get('overhead_images', {})
    result = {}
    for camera in CAMERAS:
        relative = Path(str(refs.get(camera) or ''))
        if relative.is_absolute() or '..' in relative.parts:
            raise ErrorAnalysisError(f'unsafe image reference: {relative}')
        path = (dataset_root / relative).resolve()
        if not path.is_file():
            raise ErrorAnalysisError(f'missing image: {path}')
        result[camera] = str(path)
    return result


def _bbox_from_vector(vector: np.ndarray, names: list[str], slot: int) -> list[float]:
    return [
        float(vector[_numeric_index(names, f'shuttles.{slot}.bbox.{coordinate}')])
        for coordinate in range(4)
    ]


def prediction_record(
    *,
    split_name: str,
    row: dict[str, Any],
    label: dict[str, Any],
    prediction: np.ndarray,
    label_names: list[str],
    slot: int,
    dataset_root: Path,
) -> dict[str, Any]:
    shuttles = label.get('shuttles', [])
    present = (
        slot < len(shuttles)
        and bool(shuttles[slot].get('presence'))
    )
    present_shuttle_count = sum(
        bool(shuttle.get('presence'))
        for shuttle in shuttles
    )
    image_paths = _image_paths(row, dataset_root)
    base_record: dict[str, Any] = {
        'split': split_name,
        'test_usage': 'consumed_read_only_existing_pilot' if split_name == 'test' else 'validation_analysis',
        'episode_id': str(row.get('episode_id') or ''),
        'scenario_family': str(row.get('scenario_family') or ''),
        'scenario_type': scenario_type_from_episode(str(row.get('episode_id') or '')),
        'shuttle_slot': slot,
        'presence_mask': int(present),
        'shuttle_count': present_shuttle_count,
        'left_image_path': image_paths['left_rail_rgb'],
        'right_image_path': image_paths['right_rail_rgb'],
    }
    if not present:
        base_record.update({
            'ground_truth_identity': None,
            'predicted_identity': None,
            'raw_identity_scores': None,
            'identity_score_margin': None,
            'ground_truth_loaded_state': None,
            'predicted_loaded_state': None,
            'raw_loaded_state_scores': None,
            'loaded_state_score_margin': None,
            'ground_truth_side': None,
            'predicted_side': None,
            'raw_side_scores': None,
            'side_score_margin': None,
            'ground_truth_block': None,
            'predicted_block': None,
            'raw_block_scores': None,
            'top2_predicted_blocks': None,
            'block_score_margin': None,
            'ground_truth_bbox': None,
            'predicted_bbox': None,
            'bbox_coordinate_errors': None,
            'bbox_absolute_coordinate_errors': None,
            'bbox_center_error': None,
            'bbox_size_error': None,
            'bbox_width_error': None,
            'bbox_height_error': None,
            'bbox_iou': None,
            'ground_truth_s_m': None,
            'predicted_s_m': None,
            'absolute_s_m_error': None,
            'ground_truth_s_ratio': None,
            'predicted_s_ratio': None,
            'absolute_s_ratio_error': None,
            'segment_length_m': None,
            'predicted_segment_length_m': None,
            'identity_correct': None,
            'loaded_state_correct': None,
            'side_correct': None,
            'block_correct': None,
            'full_location_correct': None,
        })
        return base_record

    shuttle = shuttles[slot]
    identity_scores = categorical_scores(
        prediction,
        label_names,
        f'shuttles.{slot}.visually_available_identity',
    )
    loaded_scores = categorical_scores(
        prediction,
        label_names,
        f'shuttles.{slot}.loaded_state',
    )
    side_scores = categorical_scores(
        prediction,
        label_names,
        f'shuttles.{slot}.location.side',
    )
    block_scores = categorical_scores(
        prediction,
        label_names,
        f'shuttles.{slot}.location.block',
    )
    pred_identity, identity_margin, _ = score_decision(identity_scores)
    pred_loaded, loaded_margin, _ = score_decision(loaded_scores)
    pred_side, side_margin, _ = score_decision(side_scores)
    pred_block, block_margin, top2_blocks = score_decision(block_scores)
    gt_identity = _canonical(shuttle.get('visually_available_identity'))
    gt_loaded = _canonical(shuttle.get('loaded_state'))
    gt_side = _canonical(shuttle.get('location', {}).get('side'))
    gt_block = _canonical(shuttle.get('location', {}).get('block'))
    gt_bbox = [float(value) for value in shuttle.get('bbox', [])]
    pred_bbox = _bbox_from_vector(prediction, label_names, slot)
    rail_position = shuttle.get('rail_position', {})
    gt_s_m = float(rail_position.get('s_m'))
    gt_s_ratio = float(rail_position.get('s_ratio'))
    segment_length = float(rail_position.get('segment_length_m'))
    pred_s_m = float(
        prediction[_numeric_index(label_names, f'shuttles.{slot}.rail_position.s_m')]
    )
    pred_s_ratio = float(
        prediction[_numeric_index(label_names, f'shuttles.{slot}.rail_position.s_ratio')]
    )
    pred_segment_length = float(
        prediction[
            _numeric_index(
                label_names,
                f'shuttles.{slot}.rail_position.segment_length_m',
            )
        ]
    )
    errors = bbox_errors(gt_bbox, pred_bbox)
    base_record.update({
        'ground_truth_identity': gt_identity,
        'predicted_identity': pred_identity,
        'raw_identity_scores': identity_scores,
        'identity_score_margin': identity_margin,
        'ground_truth_loaded_state': gt_loaded,
        'predicted_loaded_state': pred_loaded,
        'raw_loaded_state_scores': loaded_scores,
        'loaded_state_score_margin': loaded_margin,
        'ground_truth_side': gt_side,
        'predicted_side': pred_side,
        'raw_side_scores': side_scores,
        'side_score_margin': side_margin,
        'ground_truth_block': gt_block,
        'predicted_block': pred_block,
        'raw_block_scores': block_scores,
        'top2_predicted_blocks': top2_blocks,
        'block_score_margin': block_margin,
        'ground_truth_bbox': gt_bbox,
        'predicted_bbox': pred_bbox,
        **errors,
        'ground_truth_s_m': gt_s_m,
        'predicted_s_m': pred_s_m,
        'absolute_s_m_error': abs(pred_s_m - gt_s_m),
        'ground_truth_s_ratio': gt_s_ratio,
        'predicted_s_ratio': pred_s_ratio,
        'absolute_s_ratio_error': abs(pred_s_ratio - gt_s_ratio),
        'segment_length_m': segment_length,
        'predicted_segment_length_m': pred_segment_length,
        'identity_correct': pred_identity == gt_identity,
        'loaded_state_correct': pred_loaded == gt_loaded,
        'side_correct': pred_side == gt_side,
        'block_correct': pred_block == gt_block,
        'full_location_correct': pred_side == gt_side and pred_block == gt_block,
    })
    return base_record


def infer_split(
    runtime: dict[str, Any],
    package_root: Path,
    split_name: str,
    rows: list[dict[str, Any]],
    labels: list[dict[str, Any]],
    *,
    batch_size: int = 32,
) -> list[dict[str, Any]]:
    torch = runtime['torch']
    trainer = runtime['trainer']
    model = runtime['model']
    device = runtime['device']
    config = runtime['config']
    mean_tensor = torch.as_tensor(
        runtime['target_mean'],
        dtype=torch.float32,
        device=device,
    )
    std_tensor = torch.as_tensor(
        runtime['target_std'],
        dtype=torch.float32,
        device=device,
    )
    dataset_root = package_root / 'dataset'
    predictions: list[np.ndarray] = []
    with torch.no_grad():
        for start in range(0, len(rows), batch_size):
            batch_rows = rows[start:start + batch_size]
            images = np.stack([
                trainer.load_paired_images(
                    row,
                    dataset_root,
                    width=int(config.get('image_width', 224)),
                    height=int(config.get('image_height', 224)),
                    allow_blank_images=False,
                    normalization_mean=trainer.VISUAL_BACKBONE_NORMALIZATION_MEAN,
                    normalization_std=trainer.VISUAL_BACKBONE_NORMALIZATION_STD,
                )
                for row in batch_rows
            ])
            tensor = torch.as_tensor(images, dtype=torch.float32, device=device)
            predicted_normalized = model(tensor)
            predicted_raw = (
                predicted_normalized * std_tensor + mean_tensor
            ).detach().cpu().numpy()
            predictions.extend(predicted_raw)
    records = []
    label_names = runtime['vectorizer'].names
    for row, label, prediction in zip(rows, labels, predictions):
        for slot in fixed_entry_indexes(label_names):
            records.append(prediction_record(
                split_name=split_name,
                row=row,
                label=label,
                prediction=np.asarray(prediction, dtype=np.float64),
                label_names=label_names,
                slot=slot,
                dataset_root=dataset_root,
            ))
    return records


def present_records(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [record for record in records if int(record.get('presence_mask', 0)) == 1]


def _identity_side(identity: Any) -> str | None:
    text = _canonical(identity)
    return text[0] if text and text[0] in {'L', 'R'} else None


def task_truth_prediction(
    record: dict[str, Any],
    task: str,
) -> tuple[str | None, str | None]:
    if task == 'loaded_state':
        return record['ground_truth_loaded_state'], record['predicted_loaded_state']
    if task == 'rail_side':
        return record['ground_truth_side'], record['predicted_side']
    if task == 'block_segment':
        return record['ground_truth_block'], record['predicted_block']
    if task == 'full_location':
        return (
            f"{record['ground_truth_side']}:{record['ground_truth_block']}",
            f"{record['predicted_side']}:{record['predicted_block']}",
        )
    if task == 'identity_slot_conditioned':
        return (
            _identity_side(record['ground_truth_identity']),
            _identity_side(record['predicted_identity']),
        )
    if task == 'identity_global_structurally_constrained':
        return record['ground_truth_identity'], record['predicted_identity']
    raise ValueError(f'unsupported classification task: {task}')


def confusion_report(
    pairs: list[tuple[str, str]],
    *,
    classes: list[str] | None = None,
) -> dict[str, Any]:
    clean_pairs = [(str(true), str(pred)) for true, pred in pairs if true and pred]
    class_names = classes or sorted({value for pair in clean_pairs for value in pair})
    index = {name: position for position, name in enumerate(class_names)}
    matrix = [[0 for _ in class_names] for _ in class_names]
    for true, pred in clean_pairs:
        if true not in index or pred not in index:
            raise ValueError(f'class order excludes observed pair {true!r}/{pred!r}')
        matrix[index[true]][index[pred]] += 1
    total = sum(sum(row) for row in matrix)
    per_class = {}
    recalls = []
    precisions = []
    f1_values = []
    for class_index, class_name in enumerate(class_names):
        true_positive = matrix[class_index][class_index]
        support = sum(matrix[class_index])
        predicted_count = sum(row[class_index] for row in matrix)
        recall = true_positive / support if support else None
        precision = true_positive / predicted_count if predicted_count else None
        if precision is None or recall is None or precision + recall == 0.0:
            f1 = 0.0 if support else None
        else:
            f1 = 2.0 * precision * recall / (precision + recall)
        if recall is not None:
            recalls.append(recall)
        if precision is not None:
            precisions.append(precision)
        if f1 is not None:
            f1_values.append(f1)
        per_class[class_name] = {
            'precision': precision,
            'recall': recall,
            'f1': f1,
            'support': support,
            'predicted_count': predicted_count,
            'predicted_distribution': predicted_count / total if total else None,
        }
    accuracy = (
        sum(matrix[position][position] for position in range(len(class_names))) / total
        if total
        else None
    )
    return {
        'classes': class_names,
        'matrix': matrix,
        'samples': total,
        'accuracy': accuracy,
        'balanced_accuracy': statistics.fmean(recalls) if recalls else None,
        'macro_precision': statistics.fmean(precisions) if precisions else None,
        'macro_recall': statistics.fmean(recalls) if recalls else None,
        'macro_f1': statistics.fmean(f1_values) if f1_values else None,
        'class_support': {
            name: values['support']
            for name, values in per_class.items()
        },
        'predicted_class_distribution': {
            name: values['predicted_distribution']
            for name, values in per_class.items()
        },
        'per_class': per_class,
    }


def top2_block_accuracy(records: list[dict[str, Any]]) -> float | None:
    considered = present_records(records)
    if not considered:
        return None
    correct = sum(
        record['ground_truth_block'] in {
            item['class']
            for item in record.get('top2_predicted_blocks') or []
        }
        for record in considered
    )
    return correct / len(considered)


def classification_reports(records: list[dict[str, Any]]) -> dict[str, Any]:
    present = present_records(records)
    reports = {}
    for task in CLASSIFICATION_TASKS:
        pairs = [task_truth_prediction(record, task) for record in present]
        report = confusion_report(pairs)
        if task == 'block_segment':
            report['top2_accuracy'] = top2_block_accuracy(present)
        if task == 'identity_global_structurally_constrained':
            report['semantics'] = (
                'global label reconstructed from fixed slot plus slot-specific left/right output; '
                'cross-slot identity errors are impossible'
            )
        reports[task] = report
    return reports


def _font(size: int = 14):
    try:
        return ImageFont.truetype('DejaVuSans.ttf', size)
    except OSError:
        return ImageFont.load_default()


def write_confusion_csvs(
    output_dir: Path,
    name: str,
    report: dict[str, Any],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    matrix_path = output_dir / f'{name}_confusion.csv'
    with matrix_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['ground_truth\\predicted', *report['classes']])
        for class_name, row in zip(report['classes'], report['matrix']):
            writer.writerow([class_name, *row])
    metrics_path = output_dir / f'{name}_metrics.csv'
    with metrics_path.open('w', encoding='utf-8', newline='') as stream:
        fieldnames = (
            'class',
            'precision',
            'recall',
            'f1',
            'support',
            'predicted_count',
            'predicted_distribution',
        )
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for class_name in report['classes']:
            writer.writerow({
                'class': class_name,
                **report['per_class'][class_name],
            })
    summary_path = output_dir / f'{name}_summary.csv'
    with summary_path.open('w', encoding='utf-8', newline='') as stream:
        fieldnames = (
            'samples',
            'accuracy',
            'balanced_accuracy',
            'macro_precision',
            'macro_recall',
            'macro_f1',
            'top2_accuracy',
        )
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow({
            key: report.get(key)
            for key in fieldnames
        })


def write_confusion_png(path: Path, title: str, report: dict[str, Any]) -> None:
    classes = report['classes']
    matrix = report['matrix']
    cell = 52
    left = 160
    top = 100
    width = max(520, left + cell * len(classes) + 20)
    height = max(320, top + cell * len(classes) + 40)
    image = Image.new('RGB', (width, height), 'white')
    draw = ImageDraw.Draw(image)
    font = _font(13)
    title_font = _font(18)
    draw.text((16, 12), title, fill='black', font=title_font)
    draw.text(
        (16, 42),
        f"accuracy={report['accuracy']:.4f}  balanced={report['balanced_accuracy']:.4f}",
        fill='black',
        font=font,
    )
    maximum = max((value for row in matrix for value in row), default=1) or 1
    for index, class_name in enumerate(classes):
        draw.text((left + index * cell + 3, top - 24), class_name[:7], fill='black', font=font)
        draw.text((8, top + index * cell + 16), class_name[:18], fill='black', font=font)
    for row_index, row in enumerate(matrix):
        for column_index, value in enumerate(row):
            intensity = int(245 - 180 * value / maximum)
            color = (intensity, intensity, 255)
            box = (
                left + column_index * cell,
                top + row_index * cell,
                left + (column_index + 1) * cell,
                top + (row_index + 1) * cell,
            )
            draw.rectangle(box, fill=color, outline=(80, 80, 80))
            draw.text(
                (box[0] + 18, box[1] + 16),
                str(value),
                fill='black',
                font=font,
            )
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)


def distribution(values: Iterable[float]) -> dict[str, Any]:
    clean = [
        float(value)
        for value in values
        if value is not None and math.isfinite(float(value))
    ]
    if not clean:
        return {
            'count': 0,
            'mean': None,
            'median': None,
            'p90': None,
            'p95': None,
            'minimum': None,
            'maximum': None,
        }
    array = np.asarray(clean, dtype=np.float64)
    return {
        'count': len(clean),
        'mean': float(array.mean()),
        'median': float(np.median(array)),
        'p90': float(np.percentile(array, 90)),
        'p95': float(np.percentile(array, 95)),
        'minimum': float(array.min()),
        'maximum': float(array.max()),
    }


def _pearson(first: list[float], second: list[float]) -> float | None:
    if len(first) < 2 or len(first) != len(second):
        return None
    first_array = np.asarray(first, dtype=np.float64)
    second_array = np.asarray(second, dtype=np.float64)
    if float(first_array.std()) == 0.0 or float(second_array.std()) == 0.0:
        return None
    return float(np.corrcoef(first_array, second_array)[0, 1])


def regression_report(records: list[dict[str, Any]]) -> dict[str, Any]:
    present = present_records(records)
    coordinate_errors = [
        abs(float(error))
        for record in present
        for error in record['bbox_coordinate_errors']
    ]
    ious = [float(record['bbox_iou']) for record in present]
    gt_s_m = [float(record['ground_truth_s_m']) for record in present]
    gt_s_ratio = [float(record['ground_truth_s_ratio']) for record in present]
    lengths = [float(record['segment_length_m']) for record in present]
    pred_s_m = [float(record['predicted_s_m']) for record in present]
    pred_s_ratio = [float(record['predicted_s_ratio']) for record in present]
    pred_lengths = [float(record['predicted_segment_length_m']) for record in present]
    gt_consistency = [
        abs(s_m - s_ratio * length)
        for s_m, s_ratio, length in zip(gt_s_m, gt_s_ratio, lengths)
    ]
    pred_consistency_gt_length = [
        abs(s_m - s_ratio * length)
        for s_m, s_ratio, length in zip(pred_s_m, pred_s_ratio, lengths)
    ]
    pred_consistency_pred_length = [
        abs(s_m - s_ratio * length)
        for s_m, s_ratio, length in zip(pred_s_m, pred_s_ratio, pred_lengths)
    ]
    return {
        'samples': len(present),
        'bbox': {
            'absolute_coordinate_error': distribution(coordinate_errors),
            'mean_iou': statistics.fmean(ious) if ious else None,
            'median_iou': statistics.median(ious) if ious else None,
            'iou_at_0_5': (
                sum(value >= 0.5 for value in ious) / len(ious)
                if ious
                else None
            ),
            'center_error': distribution(record['bbox_center_error'] for record in present),
            'size_error': distribution(record['bbox_size_error'] for record in present),
            'absolute_width_error': distribution(
                abs(record['bbox_width_error']) for record in present
            ),
            'absolute_height_error': distribution(
                abs(record['bbox_height_error']) for record in present
            ),
            'iou': distribution(ious),
        },
        's_m_absolute_error': distribution(record['absolute_s_m_error'] for record in present),
        's_ratio_absolute_error': distribution(
            record['absolute_s_ratio_error'] for record in present
        ),
        'coordinate_consistency': {
            'dataset_convention': 's_ratio approximately equals s_m / segment_length_m',
            'ground_truth_s_m_s_ratio_correlation': _pearson(gt_s_m, gt_s_ratio),
            'predicted_s_m_s_ratio_correlation': _pearson(pred_s_m, pred_s_ratio),
            'ground_truth_abs_residual_s_m_minus_ratio_times_length': distribution(gt_consistency),
            'predicted_abs_residual_using_ground_truth_length': distribution(
                pred_consistency_gt_length
            ),
            'predicted_abs_residual_using_predicted_length': distribution(
                pred_consistency_pred_length
            ),
            'segment_length_absolute_error': distribution(
                abs(predicted - actual)
                for predicted, actual in zip(pred_lengths, lengths)
            ),
        },
    }


def compact_group_metrics(records: list[dict[str, Any]]) -> dict[str, Any]:
    present = present_records(records)
    if not present:
        return {'samples': 0}

    def accuracy(key: str) -> float:
        return sum(bool(record[key]) for record in present) / len(present)

    return {
        'samples': len(present),
        'scenario_count': len({record['episode_id'] for record in present}),
        'identity_accuracy': accuracy('identity_correct'),
        'loaded_state_accuracy': accuracy('loaded_state_correct'),
        'side_accuracy': accuracy('side_correct'),
        'block_accuracy': accuracy('block_correct'),
        'full_location_accuracy': accuracy('full_location_correct'),
        'top2_block_accuracy': top2_block_accuracy(present),
        'bbox_mean_iou': statistics.fmean(record['bbox_iou'] for record in present),
        'bbox_center_error': distribution(record['bbox_center_error'] for record in present),
        's_m_absolute_error': distribution(record['absolute_s_m_error'] for record in present),
        's_ratio_absolute_error': distribution(
            record['absolute_s_ratio_error'] for record in present
        ),
    }


def grouped_metrics(records: list[dict[str, Any]]) -> dict[str, Any]:
    present = present_records(records)
    grouping_values: dict[str, Callable[[dict[str, Any]], str]] = {
        'shuttle_identity': lambda record: str(record['ground_truth_identity']),
        'shuttle_slot': lambda record: str(record['shuttle_slot']),
        'rail_side': lambda record: str(record['ground_truth_side']),
        'block_segment': lambda record: str(record['ground_truth_block']),
        'loaded_state': lambda record: str(record['ground_truth_loaded_state']),
        'segment_length_m': lambda record: f"{float(record['segment_length_m']):.6f}",
        'shuttle_count': lambda record: str(record['shuttle_count']),
        'scenario_type': lambda record: str(record['scenario_type']),
    }
    result = {}
    for group_name, getter in grouping_values.items():
        buckets: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for record in present:
            buckets[getter(record)].append(record)
        result[group_name] = {
            value: compact_group_metrics(bucket)
            for value, bucket in sorted(buckets.items())
        }
    return result


def audit_metric_semantics(
    validation_records: list[dict[str, Any]],
    test_records: list[dict[str, Any]],
    *,
    pilot_results: Path,
) -> dict[str, Any]:
    metrics = read_json(pilot_results / 'metrics.json')
    best_epoch = int(metrics.get('best_epoch', 0))
    best_history = next(
        (
            row
            for row in metrics.get('history', [])
            if int(row.get('epoch', -1)) == best_epoch
        ),
        {},
    )
    existing_test = read_json(
        pilot_results / 'evaluation_test' / 'visual_state_eval.json'
    ).get('splits', {}).get('test', {}).get('metrics', {})

    def split_audit(
        records: list[dict[str, Any]],
        *,
        reported_location_accuracy: float | None,
    ) -> dict[str, Any]:
        present = present_records(records)
        unrepresentable = []
        legacy_block_correct = 0
        for record in present:
            scores = record.get('raw_block_scores') or {}
            ground_truth = record.get('ground_truth_block')
            representable = ground_truth in scores
            if not representable:
                unrepresentable.append({
                    'episode_id': record['episode_id'],
                    'scenario_family': record['scenario_family'],
                    'shuttle_slot': record['shuttle_slot'],
                    'ground_truth_block': ground_truth,
                    'representable_block_classes': sorted(scores),
                })
            legacy_ground_truth = (
                ground_truth
                if representable
                else sorted(scores)[0]
            )
            legacy_block_correct += int(
                record.get('predicted_block') == legacy_ground_truth
            )
        side_correct = sum(bool(record['side_correct']) for record in present)
        block_correct = sum(bool(record['block_correct']) for record in present)
        full_correct = sum(bool(record['full_location_correct']) for record in present)
        legacy_location = (
            (side_correct + legacy_block_correct) / (2.0 * len(present))
            if present
            else None
        )
        corrected_pooled = (
            (side_correct + block_correct) / (2.0 * len(present))
            if present
            else None
        )
        return {
            'present_shuttles': len(present),
            'reported_location_accuracy': reported_location_accuracy,
            'legacy_metric_reproduced_from_vectorized_targets': legacy_location,
            'corrected_pooled_side_and_block_accuracy': corrected_pooled,
            'rail_side_accuracy': side_correct / len(present) if present else None,
            'block_accuracy': block_correct / len(present) if present else None,
            'full_location_accuracy': full_correct / len(present) if present else None,
            'unrepresentable_ground_truth_block_count': len(unrepresentable),
            'unrepresentable_ground_truth_blocks': unrepresentable,
            'phantom_legacy_block_correct_count': legacy_block_correct - block_correct,
        }

    validation = split_audit(
        validation_records,
        reported_location_accuracy=best_history.get('val_location_accuracy'),
    )
    test = split_audit(
        test_records,
        reported_location_accuracy=existing_test.get('location_accuracy'),
    )
    return {
        'status': 'PASS',
        'identity_accuracy': {
            'finding': (
                'slot-conditioned left/right identity accuracy, not an '
                'unconstrained six-class metric'
            ),
        },
        'location_accuracy': {
            'finding': (
                'the legacy metric pools one side classification and one block '
                'classification per present slot; it is not full-location accuracy'
            ),
            'validation': validation,
            'test_read_only': test,
        },
        'bbox_mae': {
            'finding': (
                'mean absolute x/y/width/height coordinate error does not measure '
                'box overlap; IoU, center, width, and height errors are required'
            ),
        },
        'label_mae': {
            'finding': (
                'heterogeneous categorical logits, pixel coordinates, metres, and '
                'ratios are mixed and should not be treated as an interpretable task metric'
            ),
        },
        'target_coverage': {
            'validation_has_unrepresentable_slot_conditioned_targets': (
                validation['unrepresentable_ground_truth_block_count'] > 0
            ),
            'test_has_unrepresentable_slot_conditioned_targets': (
                test['unrepresentable_ground_truth_block_count'] > 0
            ),
            'interpretation': (
                'categorical vocabularies are fitted per output slot from training only; '
                'an unseen validation class in a slot cannot be predicted by that slot'
            ),
        },
    }


def scenario_groups(records: list[dict[str, Any]]) -> list[list[dict[str, Any]]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in present_records(records):
        grouped[str(record['episode_id'])].append(record)
    return [grouped[key] for key in sorted(grouped)]


def scenario_bootstrap_ci(
    records: list[dict[str, Any]],
    metric: Callable[[list[dict[str, Any]]], float | None],
    *,
    replicates: int,
    seed: int,
) -> dict[str, Any]:
    groups = scenario_groups(records)
    if not groups:
        return {
            'point_estimate': None,
            'ci_95': [None, None],
            'replicates': 0,
            'scenario_count': 0,
        }
    rng = random.Random(seed)
    estimates = []
    for _ in range(replicates):
        sample = []
        for _ in range(len(groups)):
            sample.extend(groups[rng.randrange(len(groups))])
        value = metric(sample)
        if value is not None and math.isfinite(float(value)):
            estimates.append(float(value))
    point = metric(present_records(records))
    if not estimates:
        interval = [None, None]
    else:
        interval = [
            float(np.percentile(estimates, 2.5)),
            float(np.percentile(estimates, 97.5)),
        ]
    return {
        'point_estimate': float(point) if point is not None else None,
        'ci_95': interval,
        'replicates': len(estimates),
        'scenario_count': len(groups),
        'resampling_unit': 'episode_id/scenario; all present shuttles retained together',
    }


def bootstrap_report(
    records: list[dict[str, Any]],
    *,
    replicates: int,
    seed: int,
) -> dict[str, Any]:
    def classification_accuracy(key: str):
        return lambda sample: (
            sum(bool(record[key]) for record in sample) / len(sample)
            if sample
            else None
        )

    metrics: dict[str, Callable[[list[dict[str, Any]]], float | None]] = {
        'slot_conditioned_identity_accuracy': classification_accuracy('identity_correct'),
        'loaded_state_accuracy': classification_accuracy('loaded_state_correct'),
        'rail_side_accuracy': classification_accuracy('side_correct'),
        'block_accuracy': classification_accuracy('block_correct'),
        'full_location_accuracy': classification_accuracy('full_location_correct'),
        'top2_block_accuracy': top2_block_accuracy,
        'mean_absolute_s_m_error': lambda sample: (
            statistics.fmean(record['absolute_s_m_error'] for record in sample)
            if sample
            else None
        ),
        'mean_absolute_s_ratio_error': lambda sample: (
            statistics.fmean(record['absolute_s_ratio_error'] for record in sample)
            if sample
            else None
        ),
        'mean_bbox_iou': lambda sample: (
            statistics.fmean(record['bbox_iou'] for record in sample)
            if sample
            else None
        ),
    }
    return {
        name: scenario_bootstrap_ci(
            records,
            function,
            replicates=replicates,
            seed=seed + index * 7919,
        )
        for index, (name, function) in enumerate(metrics.items())
    }


def _ground_truth_score(record: dict[str, Any], task: str) -> float:
    if task == 'location':
        scores = record.get('raw_block_scores') or {}
        return float(scores.get(record.get('ground_truth_block'), -math.inf))
    scores = record.get('raw_loaded_state_scores') or {}
    return float(scores.get(record.get('ground_truth_loaded_state'), -math.inf))


def select_worst_cases(
    records: list[dict[str, Any]],
    *,
    limit: int = 20,
) -> dict[str, list[dict[str, Any]]]:
    present = present_records(records)
    orderings = {
        's_m': sorted(
            present,
            key=lambda record: (
                -record['absolute_s_m_error'],
                record['episode_id'],
                record['shuttle_slot'],
            ),
        ),
        's_ratio': sorted(
            present,
            key=lambda record: (
                -record['absolute_s_ratio_error'],
                record['episode_id'],
                record['shuttle_slot'],
            ),
        ),
        'bbox_iou': sorted(
            present,
            key=lambda record: (
                record['bbox_iou'],
                record['episode_id'],
                record['shuttle_slot'],
            ),
        ),
        'location': sorted(
            present,
            key=lambda record: (
                bool(record['full_location_correct']),
                _ground_truth_score(record, 'location'),
                record['episode_id'],
                record['shuttle_slot'],
            ),
        ),
        'loaded_state': sorted(
            present,
            key=lambda record: (
                bool(record['loaded_state_correct']),
                _ground_truth_score(record, 'loaded_state'),
                record['episode_id'],
                record['shuttle_slot'],
            ),
        ),
    }
    keep = (
        'episode_id',
        'scenario_family',
        'scenario_type',
        'shuttle_slot',
        'shuttle_count',
        'ground_truth_identity',
        'predicted_identity',
        'ground_truth_loaded_state',
        'predicted_loaded_state',
        'ground_truth_side',
        'predicted_side',
        'ground_truth_block',
        'predicted_block',
        'ground_truth_bbox',
        'predicted_bbox',
        'bbox_iou',
        'bbox_center_error',
        'ground_truth_s_m',
        'predicted_s_m',
        'absolute_s_m_error',
        'ground_truth_s_ratio',
        'predicted_s_ratio',
        'absolute_s_ratio_error',
        'segment_length_m',
        'identity_correct',
        'loaded_state_correct',
        'side_correct',
        'block_correct',
        'full_location_correct',
        'left_image_path',
        'right_image_path',
    )
    return {
        category: [
            {
                'rank': rank,
                **{key: record.get(key) for key in keep},
            }
            for rank, record in enumerate(ordered[:limit], start=1)
        ]
        for category, ordered in orderings.items()
    }


def _draw_box(
    draw: ImageDraw.ImageDraw,
    bbox: list[float],
    *,
    offset_x: int,
    offset_y: int,
    color: tuple[int, int, int],
    label: str,
) -> None:
    x, y, width, height = (float(value) for value in bbox)
    coordinates = (
        offset_x + x,
        offset_y + y,
        offset_x + x + max(0.0, width),
        offset_y + y + max(0.0, height),
    )
    draw.rectangle(coordinates, outline=color, width=3)
    draw.text(
        (coordinates[0] + 2, max(offset_y, coordinates[1] - 16)),
        label,
        fill=color,
        font=_font(13),
    )


def write_worst_case_overlay(
    path: Path,
    record: dict[str, Any],
    *,
    category: str,
    rank: int,
) -> None:
    with Image.open(record['left_image_path']) as left_source:
        left = left_source.convert('RGB')
    with Image.open(record['right_image_path']) as right_source:
        right = right_source.convert('RGB')
    header = 112
    width = left.width + right.width
    height = header + max(left.height, right.height)
    canvas = Image.new('RGB', (width, height), (245, 245, 245))
    canvas.paste(left, (0, header))
    canvas.paste(right, (left.width, header))
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (10, 8),
        f'{category} rank {rank} | {record["episode_id"]} | slot {record["shuttle_slot"]}',
        fill='black',
        font=_font(17),
    )
    draw.text(
        (10, 34),
        (
            f'GT {record["ground_truth_identity"]} {record["ground_truth_loaded_state"]} '
            f'{record["ground_truth_side"]}:{record["ground_truth_block"]} '
            f's={record["ground_truth_s_m"]:.3f} r={record["ground_truth_s_ratio"]:.3f}'
        ),
        fill=(0, 120, 0),
        font=_font(14),
    )
    draw.text(
        (10, 58),
        (
            f'PRED {record["predicted_identity"]} {record["predicted_loaded_state"]} '
            f'{record["predicted_side"]}:{record["predicted_block"]} '
            f's={record["predicted_s_m"]:.3f} r={record["predicted_s_ratio"]:.3f}'
        ),
        fill=(200, 0, 0),
        font=_font(14),
    )
    draw.text(
        (10, 82),
        (
            f'IoU={record["bbox_iou"]:.3f} center_err={record["bbox_center_error"]:.2f}px '
            f'| green=ground truth, red=prediction'
        ),
        fill='black',
        font=_font(14),
    )
    side_offsets = {
        'LEFT': 0,
        'RIGHT': left.width,
    }
    gt_offset = side_offsets.get(record['ground_truth_side'])
    pred_offset = side_offsets.get(record['predicted_side'])
    if gt_offset is not None:
        _draw_box(
            draw,
            record['ground_truth_bbox'],
            offset_x=gt_offset,
            offset_y=header,
            color=(0, 220, 0),
            label=f"GT {record['ground_truth_identity']}",
        )
    if pred_offset is not None:
        _draw_box(
            draw,
            record['predicted_bbox'],
            offset_x=pred_offset,
            offset_y=header,
            color=(255, 30, 30),
            label=f"PRED {record['predicted_identity']}",
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(path)


def latency_statistics(seconds: list[float]) -> dict[str, Any]:
    summary = distribution(seconds)
    return {
        'mean_seconds': summary['mean'],
        'median_seconds': summary['median'],
        'p95_seconds': summary['p95'],
        'minimum_seconds': summary['minimum'],
        'maximum_seconds': summary['maximum'],
        'sample_count': summary['count'],
    }


def latency_protocol(
    inference: Callable[[], Any],
    *,
    synchronize: Callable[[], None],
    warmup_iterations: int,
    timed_iterations: int,
    clock: Callable[[], float] = time.perf_counter,
) -> dict[str, Any]:
    if warmup_iterations < 20:
        raise ValueError('latency protocol requires at least 20 warm-up iterations')
    if timed_iterations <= 0:
        raise ValueError('latency protocol requires positive timed iterations')
    synchronize()
    cold_start = clock()
    inference()
    synchronize()
    cold_latency = clock() - cold_start
    for _ in range(warmup_iterations):
        inference()
        synchronize()
    timings = []
    for _ in range(timed_iterations):
        synchronize()
        started = clock()
        inference()
        synchronize()
        timings.append(clock() - started)
    return {
        'cold_start_latency_seconds': cold_latency,
        'warmup_iterations_excluded': warmup_iterations,
        'steady_state': latency_statistics(timings),
    }


def benchmark_latency(
    runtime: dict[str, Any],
    package_root: Path,
    rows: list[dict[str, Any]],
    *,
    warmup_iterations: int,
    timed_iterations: int,
) -> dict[str, Any]:
    torch = runtime['torch']
    device = runtime['device']
    if device != 'cuda' or not torch.cuda.is_available():
        return {
            'status': 'SKIPPED',
            'reason': 'CUDA is required for the corrected benchmark protocol',
            'device': device,
        }
    trainer = runtime['trainer']
    config = runtime['config']
    dataset_root = package_root / 'dataset'

    def image_tensor(row_batch: list[dict[str, Any]]):
        arrays = np.stack([
            trainer.load_paired_images(
                row,
                dataset_root,
                width=int(config.get('image_width', 224)),
                height=int(config.get('image_height', 224)),
                allow_blank_images=False,
                normalization_mean=trainer.VISUAL_BACKBONE_NORMALIZATION_MEAN,
                normalization_std=trainer.VISUAL_BACKBONE_NORMALIZATION_STD,
            )
            for row in row_batch
        ])
        return torch.as_tensor(arrays, dtype=torch.float32, device=device)

    batch_one = image_tensor(rows[:1])
    batch_32 = image_tensor((rows * math.ceil(32 / len(rows)))[:32])
    model = runtime['model']
    synchronize = torch.cuda.synchronize
    with torch.no_grad():
        batch_one_report = latency_protocol(
            lambda: model(batch_one),
            synchronize=synchronize,
            warmup_iterations=warmup_iterations,
            timed_iterations=timed_iterations,
        )
        batch_32_report = latency_protocol(
            lambda: model(batch_32),
            synchronize=synchronize,
            warmup_iterations=warmup_iterations,
            timed_iterations=timed_iterations,
        )
    batch_one_report['batch_size'] = 1
    batch_one_report['throughput_samples_per_second'] = (
        1.0 / batch_one_report['steady_state']['mean_seconds']
    )
    batch_32_report['batch_size'] = 32
    batch_32_report['throughput_samples_per_second'] = (
        32.0 / batch_32_report['steady_state']['mean_seconds']
    )
    return {
        'status': 'PASS',
        'device': device,
        'gpu_name': torch.cuda.get_device_name(0),
        'cuda_version': str(torch.version.cuda),
        'protocol': {
            'image_loading_included': False,
            'torch_cuda_synchronize_before_and_after_timing': True,
            'minimum_warmup_iterations': 20,
            'warmup_iterations': warmup_iterations,
            'timed_iterations': timed_iterations,
        },
        'batch_size_1': batch_one_report,
        'batch_size_32': batch_32_report,
    }


def _validation_findings(
    classification: dict[str, Any],
    regression: dict[str, Any],
    grouped: dict[str, Any],
    metric_semantics: dict[str, Any],
) -> list[str]:
    findings = []
    loaded = classification['loaded_state']
    block = classification['block_segment']
    findings.append(
        f"Loaded-state accuracy is {loaded['accuracy']:.3f} "
        f"(balanced accuracy {loaded['balanced_accuracy']:.3f})."
    )
    findings.append(
        f"Block accuracy is {block['accuracy']:.3f}; top-2 block accuracy is "
        f"{block['top2_accuracy']:.3f}."
    )
    location_audit = metric_semantics['location_accuracy']['validation']
    findings.append(
        f"The reported legacy location_accuracy={location_audit['reported_location_accuracy']:.3f} "
        f"pools side and block decisions; actual full-location accuracy is "
        f"{location_audit['full_location_accuracy']:.3f}."
    )
    if location_audit['unrepresentable_ground_truth_block_count']:
        findings.append(
            f"{location_audit['unrepresentable_ground_truth_block_count']} validation block "
            'targets are absent from their slot-conditioned training vocabulary and are '
            'therefore impossible for those slots to predict.'
        )
    bbox = regression['bbox']
    findings.append(
        f"Bounding-box mean IoU is {bbox['mean_iou']:.3f}, with "
        f"IoU@0.5={bbox['iou_at_0_5']:.3f}."
    )
    findings.append(
        f"s_m MAE is {regression['s_m_absolute_error']['mean']:.3f} m "
        f"(p95 {regression['s_m_absolute_error']['p95']:.3f} m); "
        f"s_ratio MAE is {regression['s_ratio_absolute_error']['mean']:.3f} "
        f"(p95 {regression['s_ratio_absolute_error']['p95']:.3f})."
    )
    loaded_groups = grouped.get('loaded_state', {})
    if loaded_groups:
        weakest = min(
            loaded_groups.items(),
            key=lambda item: item[1].get('loaded_state_accuracy', 1.0),
        )
        findings.append(
            f"Weakest loaded-state class group is {weakest[0]} at "
            f"{weakest[1]['loaded_state_accuracy']:.3f} accuracy."
        )
    block_groups = grouped.get('block_segment', {})
    supported = [
        item
        for item in block_groups.items()
        if item[1].get('samples', 0) >= 3
    ]
    if supported:
        weakest_block = min(
            supported,
            key=lambda item: item[1].get('block_accuracy', 1.0),
        )
        findings.append(
            f"Weakest block with at least three shuttles is {weakest_block[0]} at "
            f"{weakest_block[1]['block_accuracy']:.3f} accuracy "
            f"(n={weakest_block[1]['samples']})."
        )
    scenario_groups = grouped.get('scenario_type', {})
    if scenario_groups:
        weakest_loaded_scenario = min(
            scenario_groups.items(),
            key=lambda item: item[1].get('loaded_state_accuracy', 1.0),
        )
        weakest_block_scenario = min(
            scenario_groups.items(),
            key=lambda item: item[1].get('block_accuracy', 1.0),
        )
        weakest_bbox_scenario = min(
            scenario_groups.items(),
            key=lambda item: item[1].get('bbox_mean_iou', 1.0),
        )
        findings.extend([
            (
                f"Loaded-state performance is weakest for "
                f"{weakest_loaded_scenario[0]} at "
                f"{weakest_loaded_scenario[1]['loaded_state_accuracy']:.3f}."
            ),
            (
                f"Block performance is weakest for {weakest_block_scenario[0]} "
                f"at {weakest_block_scenario[1]['block_accuracy']:.3f}."
            ),
            (
                f"Bounding-box overlap is weakest for {weakest_bbox_scenario[0]} "
                f"with mean IoU {weakest_bbox_scenario[1]['bbox_mean_iou']:.3f}."
            ),
        ])
    return findings


def _ablation_plan(findings: list[str]) -> list[dict[str, Any]]:
    return [
        {
            'order': 1,
            'name': 'bbox representation and overlap-objective ablation',
            'validation_only_decision': True,
            'change_one_factor_at_a_time': [
                'current normalized xywh Smooth L1 baseline',
                'normalized center-x/center-y/width/height parameterization only',
                'auxiliary IoU-family loss only',
            ],
            'selection_metrics': [
                'validation mean/median IoU',
                'IoU@0.5',
                'bbox center-error p95',
                'total weighted validation loss',
            ],
        },
        {
            'order': 2,
            'name': 'loaded-state head weighting and crop-resolution ablation',
            'validation_only_decision': True,
            'change_one_factor_at_a_time': [
                'baseline',
                'loaded-state head weight only',
                'shuttle-centered auxiliary crop only',
            ],
            'selection_metrics': [
                'validation loaded-state balanced accuracy',
                'per-class recall',
                'total weighted validation loss',
            ],
        },
        {
            'order': 3,
            'name': 'location representation ablation',
            'validation_only_decision': True,
            'change_one_factor_at_a_time': [
                'current per-slot training-fitted block vocabulary',
                'global union block vocabulary available to every present slot',
                'hierarchical side-then-valid-block masking',
                'joint full-location classification',
            ],
            'selection_metrics': [
                'validation full-location balanced accuracy',
                'top-2 block accuracy',
                'per-block recall',
            ],
        },
        {
            'order': 4,
            'name': 'coordinate-consistency regularization ablation',
            'validation_only_decision': True,
            'change_one_factor_at_a_time': [
                'current independent s_m/s_ratio predictions',
                'derive s_ratio from s_m and known segment length',
                'add consistency penalty without changing inference schema',
            ],
            'selection_metrics': [
                'validation s_m MAE/p95',
                's_ratio MAE/p95',
                'coordinate consistency residual',
            ],
        },
        {
            'order': 5,
            'name': 'set-based identity/output assignment study',
            'validation_only_decision': True,
            'purpose': (
                'separate true visual identity recognition from the current '
                'ground-truth-ID-conditioned slot structure'
            ),
            'candidate': 'permutation-invariant matching with a global six-class identity head',
            'selection_metrics': [
                'validation unconstrained global identity confusion matrix',
                'detection matching accuracy',
            ],
        },
    ]


def error_analysis_report_markdown(summary: dict[str, Any]) -> str:
    validation = summary['validation']
    test = summary['test_read_only']
    identity = summary['identity_semantics']
    lines = [
        '# Room 315 post-pilot error analysis',
        '',
        f"Verdict: **{summary['verdict']}**",
        '',
        '## Frozen baseline',
        '',
        f"- Checkpoint: `{summary['baseline']['checkpoint']}`",
        f"- SHA-256: `{summary['baseline']['checkpoint_sha256']}`",
        f"- Best epoch: {summary['baseline']['best_epoch']}",
        '- Architecture, loss, splits, and baseline outputs were not changed.',
        '',
        '## Identity semantics',
        '',
        identity['current_metric_semantics'] + '.',
        '',
        'The existing metric should be read as `slot_conditioned_identity_accuracy`, '
        'not as an unconstrained six-class result. A structurally constrained global '
        'six-label matrix is also exported. No data leakage was demonstrated.',
        '',
        '## Metric-semantics audit',
        '',
        'The legacy `location_accuracy` is a pooled accuracy over side and block '
        'classification groups, not the probability that the full location is correct. '
        'Validation also contains slot-conditioned block targets that are absent from '
        'the fitted training vocabulary; these cases are explicitly listed in '
        '`metric_semantics_audit.json`.',
        '',
        'The legacy `bbox_mae` measures absolute coordinate error rather than overlap, '
        'and `label_mae` combines unrelated units. This report therefore emphasizes '
        'IoU/center/size metrics and task-specific classification results.',
        '',
        '## Validation-only findings',
        '',
    ]
    lines.extend(f'- {finding}' for finding in validation['important_failure_patterns'])
    lines.extend([
        '',
        'All recommendations below use validation analysis only. Scenario families are '
        'one sample each and are not treated as strong standalone statistics; grouped '
        'attributes and scenario-cluster bootstrap confidence intervals are reported.',
        '',
        '## Existing test result (consumed, read-only)',
        '',
        f"- Present-shuttle records: {test['present_shuttles']}",
        f"- Loaded-state accuracy: {test['classification']['loaded_state']['accuracy']:.4f}",
        f"- Block accuracy: {test['classification']['block_segment']['accuracy']:.4f}",
        f"- Mean bbox IoU: {test['regression']['bbox']['mean_iou']:.4f}",
        '- Test data was not used for thresholds, configuration selection, or recommendations.',
        '',
        '## Corrected inference benchmark',
        '',
        f"- Status: {summary['latency']['status']}",
    ])
    if summary['latency']['status'] == 'PASS':
        one = summary['latency']['batch_size_1']
        thirty_two = summary['latency']['batch_size_32']
        lines.extend([
            f"- Batch-1 median: {one['steady_state']['median_seconds'] * 1000.0:.3f} ms",
            f"- Batch-1 p95: {one['steady_state']['p95_seconds'] * 1000.0:.3f} ms",
            f"- Batch-32 throughput: {thirty_two['throughput_samples_per_second']:.2f} samples/s",
        ])
    lines.extend([
        '',
        '## Controlled ablation plan (not implemented)',
        '',
    ])
    for item in summary['recommended_controlled_ablation_plan']:
        lines.append(f"{item['order']}. **{item['name']}** — validation-only comparison.")
    lines.extend([
        '',
        '## Artifact immutability',
        '',
        f"- Status: {summary['artifact_immutability']['status']}",
        f"- Existing files checked: {summary['artifact_immutability']['audited_file_count']}",
        '',
    ])
    return '\n'.join(lines)


def run_analysis(args: argparse.Namespace) -> dict[str, Any]:
    pilot_results = args.pilot_results.expanduser().resolve()
    package_root = args.package_root.expanduser().resolve()
    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir is not None
        else pilot_results / DEFAULT_OUTPUT_NAME
    )
    if output_dir.exists():
        raise ErrorAnalysisError(f'refusing to overwrite analysis output: {output_dir}')
    checkpoint_path = (
        args.checkpoint.expanduser().resolve()
        if args.checkpoint is not None
        else pilot_results / 'best.pt'
    )
    baseline_snapshot = snapshot_existing_files(
        {
            'downloaded_pilot_results': pilot_results,
            'local_pilot_package': package_root,
        },
        excluded_roots=(output_dir,),
    )
    output_dir.mkdir(parents=True)
    write_json(output_dir / 'baseline_artifact_fingerprints_before.json', baseline_snapshot)

    runtime = load_frozen_model(
        pilot_results,
        package_root,
        checkpoint_path=checkpoint_path,
        expected_sha256=args.expected_checkpoint_sha256,
        device_request=args.device,
        seed=args.seed,
    )
    split_rows: dict[str, list[dict[str, Any]]] = {}
    split_labels: dict[str, list[dict[str, Any]]] = {}
    for split_name in SPLITS:
        split_rows[split_name], split_labels[split_name] = _load_split(
            package_root,
            split_name,
            runtime['dataset_module'],
        )

    identity_audit = audit_identity_semantics(
        split_labels,
        runtime['vectorizer'].names,
    )
    write_json(output_dir / 'identity_semantics_audit.json', identity_audit)
    (output_dir / 'identity_semantics_audit.md').write_text(
        identity_audit_markdown(identity_audit),
        encoding='utf-8',
    )

    prediction_exports: dict[str, list[dict[str, Any]]] = {}
    for split_name in SPLITS:
        prediction_exports[split_name] = infer_split(
            runtime,
            package_root,
            split_name,
            split_rows[split_name],
            split_labels[split_name],
            batch_size=args.inference_batch_size,
        )
    validation_predictions = prediction_exports['val']
    test_predictions = prediction_exports['test']
    write_jsonl(output_dir / 'validation_predictions.jsonl', validation_predictions)
    write_records_csv(output_dir / 'validation_predictions.csv', validation_predictions)
    write_jsonl(output_dir / 'test_predictions_read_only.jsonl', test_predictions)
    write_records_csv(output_dir / 'test_predictions_read_only.csv', test_predictions)

    metric_semantics = audit_metric_semantics(
        validation_predictions,
        test_predictions,
        pilot_results=pilot_results,
    )
    write_json(output_dir / 'metric_semantics_audit.json', metric_semantics)
    validation_classification = classification_reports(validation_predictions)
    test_classification = classification_reports(test_predictions)
    confusion_root = output_dir / 'confusion_matrices'
    for split_name, reports in (
        ('validation', validation_classification),
        ('test_read_only', test_classification),
    ):
        for task, report in reports.items():
            artifact_name = f'{split_name}_{task}'
            write_confusion_csvs(confusion_root, artifact_name, report)
            write_confusion_png(
                confusion_root / f'{artifact_name}_confusion.png',
                artifact_name.replace('_', ' '),
                report,
            )

    validation_regression = regression_report(validation_predictions)
    test_regression = regression_report(test_predictions)
    validation_grouped = grouped_metrics(validation_predictions)
    test_grouped = grouped_metrics(test_predictions)
    grouped_report = {
        'policy': {
            'scenario_family_accuracy_not_used_as_strong_statistic': True,
            'meaningful_grouping_attributes': [
                'scenario_type',
                'shuttle_count',
                'shuttle_identity',
                'slot',
                'rail_side',
                'block',
                'loaded_state',
                'segment_length',
            ],
            'test_is_consumed_read_only': True,
        },
        'validation': validation_grouped,
        'test_read_only': test_grouped,
        'scenario_bootstrap_confidence_intervals': {
            'validation': bootstrap_report(
                validation_predictions,
                replicates=args.bootstrap_replicates,
                seed=args.seed,
            ),
            'test_read_only_reporting_only': bootstrap_report(
                test_predictions,
                replicates=args.bootstrap_replicates,
                seed=args.seed + 104729,
            ),
        },
    }
    write_json(output_dir / 'grouped_metrics.json', grouped_report)

    worst_cases = select_worst_cases(
        validation_predictions,
        limit=args.worst_case_limit,
    )
    write_json(output_dir / 'worst_cases.json', {
        'selection_split': 'validation',
        'limit_per_category': args.worst_case_limit,
        'categories': worst_cases,
    })
    overlays_root = output_dir / 'worst_case_overlays'
    for category, cases in worst_cases.items():
        for case in cases:
            filename = (
                f"{category}_{case['rank']:02d}_"
                f"{case['episode_id']}_slot{case['shuttle_slot']}.png"
            )
            write_worst_case_overlay(
                overlays_root / category / filename,
                case,
                category=category,
                rank=case['rank'],
            )

    latency = benchmark_latency(
        runtime,
        package_root,
        split_rows['val'],
        warmup_iterations=args.latency_warmup_iterations,
        timed_iterations=args.latency_timed_iterations,
    )
    write_json(output_dir / 'latency_benchmark.json', latency)
    validation_findings = _validation_findings(
        validation_classification,
        validation_regression,
        validation_grouped,
        metric_semantics,
    )
    ablation_plan = _ablation_plan(validation_findings)
    metrics = read_json(pilot_results / 'metrics.json')
    summary: dict[str, Any] = {
        'verdict': 'PASS',
        'analysis_kind': 'read_only_post_pilot_error_analysis',
        'output_dir': str(output_dir),
        'baseline': {
            'checkpoint': str(checkpoint_path),
            'checkpoint_sha256': runtime['checkpoint_sha256'],
            'expected_checkpoint_sha256': args.expected_checkpoint_sha256,
            'best_epoch': metrics.get('best_epoch'),
            'architecture_unchanged': True,
            'loss_unchanged': True,
            'dataset_splits_unchanged': True,
            'existing_pilot_outputs_unchanged': None,
        },
        'identity_semantics': identity_audit,
        'metric_semantics_audit': metric_semantics,
        'validation': {
            'scenario_count': len(split_rows['val']),
            'present_shuttles': len(present_records(validation_predictions)),
            'classification': validation_classification,
            'regression': validation_regression,
            'important_failure_patterns': validation_findings,
            'recommendation_source': True,
        },
        'test_read_only': {
            'label': 'already-reported pilot test; consumed and read-only',
            'scenario_count': len(split_rows['test']),
            'present_shuttles': len(present_records(test_predictions)),
            'classification': test_classification,
            'regression': test_regression,
            'used_for_threshold_tuning': False,
            'used_for_configuration_selection': False,
            'used_for_recommendations': False,
        },
        'grouped_metrics_path': str(output_dir / 'grouped_metrics.json'),
        'worst_cases_path': str(output_dir / 'worst_cases.json'),
        'latency': latency,
        'recommended_controlled_ablation_plan': ablation_plan,
        'implemented_ablations': [],
        'artifact_immutability': None,
        'created_files': [],
    }
    write_json(output_dir / 'error_analysis_summary.json', summary)
    (output_dir / 'error_analysis_report.md').write_text(
        error_analysis_report_markdown({
            **summary,
            'artifact_immutability': {
                'status': 'PENDING',
                'audited_file_count': baseline_snapshot['file_count'],
            },
        }),
        encoding='utf-8',
    )

    immutability = verify_snapshot_unchanged(baseline_snapshot)
    write_json(output_dir / 'artifact_immutability_audit.json', immutability)
    if immutability['status'] != 'PASS':
        summary['verdict'] = 'FAIL'
    summary['artifact_immutability'] = immutability
    summary['baseline']['existing_pilot_outputs_unchanged'] = immutability['status'] == 'PASS'
    summary['created_files'] = sorted(
        str(path.relative_to(output_dir))
        for path in output_dir.rglob('*')
        if path.is_file()
    )
    write_json(output_dir / 'error_analysis_summary.json', summary)
    (output_dir / 'error_analysis_report.md').write_text(
        error_analysis_report_markdown(summary),
        encoding='utf-8',
    )
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Read-only post-pilot error analysis for Room 315 visual state.'
    )
    parser.add_argument('--pilot-results', type=Path, default=DEFAULT_PILOT_RESULTS)
    parser.add_argument('--package-root', type=Path, default=DEFAULT_PACKAGE_ROOT)
    parser.add_argument('--checkpoint', type=Path, default=None)
    parser.add_argument('--output-dir', type=Path, default=None)
    parser.add_argument(
        '--expected-checkpoint-sha256',
        default=EXPECTED_CHECKPOINT_SHA256,
    )
    parser.add_argument('--device', default='auto')
    parser.add_argument('--seed', type=int, default=31520260726)
    parser.add_argument('--inference-batch-size', type=int, default=32)
    parser.add_argument('--bootstrap-replicates', type=int, default=2000)
    parser.add_argument('--worst-case-limit', type=int, default=20)
    parser.add_argument('--latency-warmup-iterations', type=int, default=20)
    parser.add_argument('--latency-timed-iterations', type=int, default=100)
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    summary = run_analysis(args)
    print(pretty_json({
        'verdict': summary['verdict'],
        'output_dir': summary['output_dir'],
        'checkpoint_sha256': summary['baseline']['checkpoint_sha256'],
        'created_file_count': len(summary['created_files']),
    }))


if __name__ == '__main__':
    main()
