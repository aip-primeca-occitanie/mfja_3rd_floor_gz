#!/usr/bin/env python3

import argparse
import json
import os
import random
import re
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_state_dataset import DATASET_MODES
from room_315_visual_state_dataset import DATASET_MODE_VISUAL_STATE
from room_315_visual_state_dataset import IMAGE_KEYS
from room_315_visual_state_dataset import VISUAL_LABEL_SUFFIX
from room_315_visual_state_dataset import file_fingerprint as _file_fingerprint
from room_315_visual_state_dataset import iter_jsonl as _iter_jsonl
from room_315_visual_state_dataset import load_visual_labels_for_rows
from room_315_visual_state_dataset import pretty_json as _pretty_json
from room_315_visual_state_dataset import raw_visual_image_refs as _model_input_image_refs
from room_315_visual_state_dataset import resolve_image_path as _resolve_image_path
from room_315_visual_state_dataset import rows_fingerprint as _row_fingerprint
from room_315_visual_state_dataset import sanitized_visual_state_row
from room_315_visual_state_dataset import scenario_family_from_row
from room_315_visual_state_dataset import validate_visual_state_rows
from room_315_visual_state_dataset import visual_label_path_for_split
from room_315_visual_state_dataset import visual_state_class_balance


def _env_path(name: str, fallback: str) -> Path:
    return Path(os.environ.get(name, fallback))


DEFAULT_INPUT = _env_path('ROOM315_VLA_DATASET_ROOT', 'room315_payload_dataset')
DEFAULT_OUTPUT_DIR = _env_path('ROOM315_VLA_SPLITS_DIR', 'room315_local_training/splits')
SPLIT_FILENAMES = {
    'train': 'train.jsonl',
    'val': 'val.jsonl',
    'test': 'test.jsonl',
}
def _camera_completeness_report(
    rows: list[dict[str, Any]],
    dataset_root: Path,
) -> dict[str, Any]:
    per_camera = {
        camera: {
            'referenced_rows': 0,
            'missing_ref_rows': 0,
            'existing_files': 0,
            'missing_files': 0,
        }
        for camera in IMAGE_KEYS
    }
    complete_rows = 0
    rows_with_any_camera = 0
    missing_examples: list[dict[str, Any]] = []
    for row_index, row in enumerate(rows):
        refs = _model_input_image_refs(row)
        row_complete = True
        if refs:
            rows_with_any_camera += 1
        for camera in IMAGE_KEYS:
            ref = refs.get(camera, '')
            stats = per_camera[camera]
            if not ref:
                stats['missing_ref_rows'] += 1
                row_complete = False
                if len(missing_examples) < 10:
                    missing_examples.append({
                        'row_index': row_index,
                        'episode_id': str(row.get('episode_id') or ''),
                        'camera': camera,
                        'reason': 'missing_ref',
                    })
                continue
            stats['referenced_rows'] += 1
            image_path = _resolve_image_path(dataset_root, ref)
            if image_path.exists() and image_path.is_file():
                stats['existing_files'] += 1
            else:
                stats['missing_files'] += 1
                row_complete = False
                if len(missing_examples) < 10:
                    missing_examples.append({
                        'row_index': row_index,
                        'episode_id': str(row.get('episode_id') or ''),
                        'camera': camera,
                        'reason': 'missing_file',
                        'ref': ref,
                    })
        if row_complete:
            complete_rows += 1
    total = len(rows)
    return {
        'required_cameras': list(IMAGE_KEYS),
        'total_rows': total,
        'rows_with_any_camera': rows_with_any_camera,
        'complete_rows': complete_rows,
        'complete_row_rate': round(complete_rows / max(1, total), 6),
        'per_camera': per_camera,
        'missing_examples': missing_examples,
        'source': 'model_input.overhead_images',
    }


def _resolve_dataset_input(input_path: Path) -> tuple[Path, Path]:
    input_path = input_path.expanduser()
    if input_path.is_dir():
        dataset_root = input_path.resolve()
        event_file = dataset_root / 'meta' / 'training_events.jsonl'
    else:
        event_file = input_path.resolve()
        dataset_root = event_file.parent.parent if event_file.parent.name == 'meta' else event_file.parent
    if not event_file.exists():
        raise FileNotFoundError(f'training events file not found: {event_file}')
    return dataset_root, event_file


def _episode_number(episode_id: str) -> int:
    match = re.search(r'episode_(\d+)', episode_id)
    return int(match.group(1)) if match else 0


def _episode_validation_path(dataset_root: Path, episode_id: str) -> Path:
    return dataset_root / 'episodes' / episode_id / 'validation.json'


def _read_validation(dataset_root: Path, episode_id: str) -> dict[str, Any] | None:
    validation_path = _episode_validation_path(dataset_root, episode_id)
    if not validation_path.exists():
        return None
    with validation_path.open('r', encoding='utf-8') as stream:
        parsed = json.load(stream)
    if not isinstance(parsed, dict):
        raise ValueError(f'validation file must contain an object: {validation_path}')
    return parsed


def _approved_validation(validation: dict[str, Any] | None) -> bool:
    if validation is None:
        return False
    status = str(validation.get('validation_status') or '').strip().lower()
    visual_capture_approved = (
        validation.get('capture_complete') is True
        and validation.get('labels_valid') is True
    )
    return (
        validation.get('approved_for_training') is True
        and visual_capture_approved
        and status in {'', 'approved'}
    )


def _build_episode_index(
    rows: list[dict[str, Any]],
    dataset_root: Path,
    *,
    allow_unvalidated: bool,
) -> dict[str, dict[str, Any]]:
    episodes: dict[str, dict[str, Any]] = {}
    for row_index, row in enumerate(rows):
        episode_id = str(row.get('episode_id') or '').strip()
        if not episode_id:
            raise ValueError(f'row {row_index} is missing episode_id')
        entry = episodes.setdefault(
            episode_id,
            {
                'episode_id': episode_id,
                'row_indices': [],
                'families': set(),
            },
        )
        entry['row_indices'].append(row_index)
        entry['families'].add(scenario_family_from_row(row))

    for episode_id, entry in episodes.items():
        families = entry['families']
        if not families:
            raise ValueError(f'episode {episode_id!r} has no scenario family')
        if len(families) > 1:
            formatted = ', '.join(sorted(families))
            raise ValueError(
                f'episode {episode_id!r} has multiple scenario families: {formatted}'
            )
        entry['scenario_family'] = next(iter(families))
        validation = _read_validation(dataset_root, episode_id)
        entry['validation'] = validation or {}
        if not allow_unvalidated and not _approved_validation(validation):
            validation_path = _episode_validation_path(dataset_root, episode_id)
            raise ValueError(f'episode {episode_id!r} is not approved for training: {validation_path}')
    return episodes


def _split_family_ids(
    family_ids: list[str],
    *,
    seed: int,
    val_families: int,
    test_families: int,
) -> dict[str, set[str]]:
    if val_families < 0 or test_families < 0:
        raise ValueError('val/test family counts must be non-negative')
    if val_families + test_families >= len(family_ids):
        raise ValueError(
            'val_families + test_families must leave at least one train family'
        )
    shuffled = sorted(family_ids)
    random.Random(seed).shuffle(shuffled)
    test_ids = set(shuffled[:test_families])
    val_ids = set(shuffled[test_families:test_families + val_families])
    train_ids = set(shuffled[test_families + val_families:])
    return {
        'train': train_ids,
        'val': val_ids,
        'test': test_ids,
    }


def _split_rows(
    rows: list[dict[str, Any]],
    episodes: dict[str, dict[str, Any]],
    family_splits: dict[str, set[str]],
) -> dict[str, list[tuple[int, dict[str, Any]]]]:
    episode_to_split: dict[str, str] = {}
    for split_name, family_ids in family_splits.items():
        for episode_id, entry in episodes.items():
            if entry['scenario_family'] in family_ids:
                episode_to_split[episode_id] = split_name
    missing = sorted(set(episodes) - set(episode_to_split))
    if missing:
        raise ValueError(f'episodes did not receive a split: {missing[:5]}')

    split_rows: dict[str, list[tuple[int, dict[str, Any]]]] = {
        split_name: []
        for split_name in SPLIT_FILENAMES
    }
    for row_index, row in enumerate(rows):
        split_name = episode_to_split[str(row.get('episode_id'))]
        split_rows[split_name].append((row_index, row))
    return {
        split_name: sorted(items, key=lambda item: item[0])
        for split_name, items in split_rows.items()
    }


def _write_visual_state_jsonl(
    path: Path,
    label_path: Path,
    indexed_rows: list[tuple[int, dict[str, Any]]],
) -> dict[str, Any]:
    model_rows: list[dict[str, Any]] = []
    label_rows: list[dict[str, Any]] = []
    for row_index, row in indexed_rows:
        model_row, label_row = sanitized_visual_state_row(row, row_index)
        model_rows.append(model_row)
        label_rows.append(label_row)
    with path.open('w', encoding='utf-8') as stream:
        for row in model_rows:
            stream.write(json.dumps(row, ensure_ascii=False, separators=(',', ':')) + '\n')
    with label_path.open('w', encoding='utf-8') as stream:
        for row in label_rows:
            stream.write(json.dumps(row, ensure_ascii=False, separators=(',', ':')) + '\n')
    return {
        'model_rows': model_rows,
        'label_rows': label_rows,
    }


def _manifest_for_split(
    split_name: str,
    episodes: dict[str, dict[str, Any]],
    family_ids: set[str],
    indexed_rows: list[tuple[int, dict[str, Any]]],
    dataset_root: Path,
    dataset_mode: str,
    visual_labels: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    split_episode_ids = sorted(
        [
            episode_id
            for episode_id, entry in episodes.items()
            if entry['scenario_family'] in family_ids
        ],
        key=lambda episode_id: (_episode_number(episode_id), episode_id),
    )
    manifest = {
        'name': split_name,
        'file': SPLIT_FILENAMES[split_name],
        'dataset_mode': dataset_mode,
        'row_count': len(indexed_rows),
        'episode_count': len(split_episode_ids),
        'family_count': len(family_ids),
        'families': sorted(family_ids),
        'camera_completeness': _camera_completeness_report(
            [row for _, row in indexed_rows],
            dataset_root,
        ),
        'episodes': [
            {
                'episode_id': episode_id,
                'scenario_family': episodes[episode_id]['scenario_family'],
                'row_count': len(episodes[episode_id]['row_indices']),
            }
            for episode_id in split_episode_ids
        ],
    }
    if dataset_mode == DATASET_MODE_VISUAL_STATE:
        manifest['label_file'] = visual_label_path_for_split(Path(SPLIT_FILENAMES[split_name])).name
        manifest['oracle_labels_physically_separate'] = True
        manifest['visual_state_class_balance'] = visual_state_class_balance(visual_labels or [])
    return manifest


def _check_outputs(output_dir: Path, *, overwrite: bool, dataset_mode: str) -> None:
    wanted = [output_dir / name for name in SPLIT_FILENAMES.values()]
    wanted.append(output_dir / 'split_manifest.json')
    if dataset_mode == DATASET_MODE_VISUAL_STATE:
        wanted.extend(visual_label_path_for_split(output_dir / name) for name in SPLIT_FILENAMES.values())
    existing = [path for path in wanted if path.exists()]
    if existing and not overwrite:
        formatted = ', '.join(str(path) for path in existing)
        raise FileExistsError(f'split output already exists; pass --overwrite: {formatted}')


def _split_integrity_report(
    rows: list[dict[str, Any]],
    episodes: dict[str, dict[str, Any]],
    split_rows: dict[str, list[tuple[int, dict[str, Any]]]],
    family_splits: dict[str, set[str]],
) -> dict[str, Any]:
    family_to_splits: dict[str, list[str]] = {}
    for split_name, family_ids in family_splits.items():
        for family_id in family_ids:
            family_to_splits.setdefault(family_id, []).append(split_name)
    overlapping_families = {
        family_id: splits
        for family_id, splits in sorted(family_to_splits.items())
        if len(splits) > 1
    }
    assigned_row_indexes = [
        row_index
        for indexed_rows in split_rows.values()
        for row_index, _ in indexed_rows
    ]
    assigned_episode_ids = {
        str(row.get('episode_id') or '')
        for indexed_rows in split_rows.values()
        for _, row in indexed_rows
    }
    source_episode_ids = set(episodes)
    return {
        'source_rows': len(rows),
        'assigned_rows': len(assigned_row_indexes),
        'row_count_matches': len(assigned_row_indexes) == len(rows),
        'duplicate_row_assignments': (
            len(assigned_row_indexes) - len(set(assigned_row_indexes))
        ),
        'missing_episode_assignments': sorted(source_episode_ids - assigned_episode_ids),
        'unexpected_episode_assignments': sorted(assigned_episode_ids - source_episode_ids),
        'families_disjoint': not overlapping_families,
        'overlapping_families': overlapping_families,
        'episode_variants_grouped_by_family': not overlapping_families,
    }


def split_dataset(
    input_path: Path,
    output_dir: Path,
    *,
    seed: int = 13,
    val_families: int = 4,
    test_families: int = 4,
    dataset_mode: str = DATASET_MODE_VISUAL_STATE,
    allow_unvalidated: bool = False,
    overwrite: bool = False,
) -> dict[str, Any]:
    if dataset_mode not in DATASET_MODES:
        raise ValueError(f'unknown dataset mode: {dataset_mode}')
    dataset_root, event_file = _resolve_dataset_input(input_path)
    rows = list(_iter_jsonl(event_file))
    if not rows:
        raise ValueError(f'no rows found in {event_file}')

    load_visual_labels_for_rows(rows)

    episodes = _build_episode_index(
        rows,
        dataset_root,
        allow_unvalidated=allow_unvalidated,
    )
    family_ids = sorted({entry['scenario_family'] for entry in episodes.values()})
    family_splits = _split_family_ids(
        family_ids,
        seed=seed,
        val_families=val_families,
        test_families=test_families,
    )
    split_rows = _split_rows(rows, episodes, family_splits)

    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    _check_outputs(output_dir, overwrite=overwrite, dataset_mode=dataset_mode)
    visual_split_labels: dict[str, list[dict[str, Any]]] = {}
    visual_split_rows: dict[str, list[dict[str, Any]]] = {}
    for split_name, filename in SPLIT_FILENAMES.items():
        split_path = output_dir / filename
        written = _write_visual_state_jsonl(
            split_path,
            visual_label_path_for_split(split_path),
            split_rows[split_name],
        )
        visual_split_rows[split_name] = written['model_rows']
        visual_split_labels[split_name] = [
            row['visual_state_labels']
            for row in written['label_rows']
        ]

    sanitized_rows = [
        row
        for rows_for_split in visual_split_rows.values()
        for row in rows_for_split
    ]
    visual_input_integrity: dict[str, Any] = {
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'rows_checked': len(sanitized_rows),
        'labels_checked': sum(len(labels) for labels in visual_split_labels.values()),
        'allowed_model_input_fields': ['overhead_images'],
        'production_feature_source': 'model_input.overhead_images only',
        'oracle_label_source': 'split_sidecar_jsonl',
        'oracle_labels_physically_separate': True,
        'row_level_metadata_used_as_features': [],
    }
    for split_name, filename in SPLIT_FILENAMES.items():
        validate_visual_state_rows(
            visual_split_rows[split_name],
            visual_label_path_for_split(output_dir / filename),
        )

    manifest = {
        'dataset_mode': dataset_mode,
        'dataset_root': str(dataset_root),
        'source': str(event_file),
        'source_fingerprint': _file_fingerprint(event_file),
        'row_fingerprint': _row_fingerprint(rows),
        'output_dir': str(output_dir),
        'seed': seed,
        'rows': len(rows),
        'episodes': len(episodes),
        'families': len(family_ids),
        'val_families': val_families,
        'test_families': test_families,
        'allow_unvalidated': allow_unvalidated,
        'camera_completeness': _camera_completeness_report(rows, dataset_root),
        'visual_state_model_input_integrity': visual_input_integrity,
        'visual_state_label_sidecars': {
            split_name: str(visual_label_path_for_split(output_dir / filename))
            for split_name, filename in SPLIT_FILENAMES.items()
        },
        'visual_state_class_balance': visual_state_class_balance(load_visual_labels_for_rows(rows)),
        'split_integrity': _split_integrity_report(
            rows,
            episodes,
            split_rows,
            family_splits,
        ),
        'splits': {
            split_name: _manifest_for_split(
                split_name,
                episodes,
                family_splits[split_name],
                split_rows[split_name],
                dataset_root,
                dataset_mode,
                visual_split_labels.get(split_name),
            )
            for split_name in SPLIT_FILENAMES
        },
    }
    manifest['row_fingerprint'] = _row_fingerprint(
        [
            row
            for rows_for_split in visual_split_rows.values()
            for row in rows_for_split
        ]
    )
    manifest['oracle_label_fingerprint'] = _row_fingerprint(
        [
            {'visual_state_labels': label}
            for labels in visual_split_labels.values()
            for label in labels
        ]
    )
    manifest_path = output_dir / 'split_manifest.json'
    manifest['manifest'] = str(manifest_path)
    manifest_path.write_text(_pretty_json(manifest) + '\n', encoding='utf-8')
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Create deterministic train/val/test splits for Room 315 VLA event data.'
    )
    parser.add_argument(
        'input',
        nargs='?',
        type=Path,
        default=DEFAULT_INPUT,
        help=(
            'Dataset directory or meta/training_events.jsonl. Defaults to '
            'ROOM315_VLA_DATASET_ROOT or room315_payload_dataset.'
        ),
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=(
            'Directory that will receive train.jsonl, val.jsonl, test.jsonl, '
            'and split_manifest.json. Defaults to ROOM315_VLA_SPLITS_DIR or '
            'room315_local_training/splits.'
        ),
    )
    parser.add_argument('--seed', type=int, default=13)
    parser.add_argument(
        '--val-families',
        type=int,
        default=4,
        help='Number of base case families reserved for validation.',
    )
    parser.add_argument(
        '--test-families',
        type=int,
        default=4,
        help='Number of base case families reserved for final test.',
    )
    parser.add_argument(
        '--dataset-mode',
        choices=DATASET_MODES,
        default=DATASET_MODE_VISUAL_STATE,
        help=(
            f'{DATASET_MODE_VISUAL_STATE} writes image-only model inputs plus '
            f'physically separate *{VISUAL_LABEL_SUFFIX} oracle label sidecars.'
        ),
    )
    parser.add_argument(
        '--allow-unvalidated',
        action='store_true',
        help='Allow episodes without approved validation.json. Intended only for synthetic tests/debugging.',
    )
    parser.add_argument(
        '--overwrite',
        action='store_true',
        help='Replace existing split files in --output-dir.',
    )
    args = parser.parse_args()
    summary = split_dataset(
        args.input,
        args.output_dir,
        seed=args.seed,
        val_families=args.val_families,
        test_families=args.test_families,
        dataset_mode=args.dataset_mode,
        allow_unvalidated=args.allow_unvalidated,
        overwrite=args.overwrite,
    )
    print(_pretty_json(summary))


if __name__ == '__main__':
    main()
