#!/usr/bin/env python3

import argparse
import json
import os
import re
import shutil
import sys
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_state_dataset import DATASET_MODES
from room_315_visual_state_dataset import DATASET_MODE_VISUAL_STATE
from room_315_visual_state_dataset import IMAGE_KEYS
from room_315_visual_state_dataset import VISUAL_LABEL_SUFFIX
from room_315_visual_state_dataset import VISUAL_STATE_SCHEMA_VERSION
from room_315_visual_state_dataset import VisualStateLabelVectorizer
from room_315_visual_state_dataset import file_fingerprint as _file_fingerprint
from room_315_visual_state_dataset import image_integrity_report
from room_315_visual_state_dataset import iter_jsonl as _iter_jsonl
from room_315_visual_state_dataset import load_visual_labels_for_rows
from room_315_visual_state_dataset import missing_image_error as _missing_image_error
from room_315_visual_state_dataset import pretty_json as _pretty_json
from room_315_visual_state_dataset import resolve_image_path as _resolve_image_path
from room_315_visual_state_dataset import rows_fingerprint as _rows_fingerprint
from room_315_visual_state_dataset import safe_int as _safe_int
from room_315_visual_state_dataset import validate_visual_state_rows
from room_315_visual_state_dataset import visual_label_path_for_split
from room_315_visual_state_dataset import visual_model_input_image_refs
from room_315_visual_state_dataset import visual_state_class_balance
from room_315_visual_state_dataset import write_jsonl as write_visual_jsonl


def _env_path(name: str, fallback: str) -> Path:
    return Path(os.environ.get(name, fallback))


DEFAULT_DATASET_ROOT = _env_path('ROOM315_VISUAL_DATASET_ROOT', 'room315_payload_dataset')
DEFAULT_SPLITS_DIR = _env_path('ROOM315_VISUAL_SPLITS_DIR', 'room315_local_training/splits')
DEFAULT_OUTPUT_ROOT = _env_path('ROOM315_LEROBOT_OUTPUT_ROOT', 'room315_local_training/lerobot')
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


def build_lerobot_features(
    *,
    visual_target_names: list[str],
    image_height: int,
    image_width: int,
    use_videos: bool,
    dataset_mode: str = DATASET_MODE_VISUAL_STATE,
) -> dict[str, dict[str, Any]]:
    image_dtype = 'video' if use_videos else 'image'
    visual_target_feature = {
        'dtype': 'float32',
        'shape': (len(visual_target_names),),
        'names': visual_target_names,
        'output_semantics': 'visual_state_labels_not_rail_commands',
    }
    features: dict[str, dict[str, Any]] = {
        'action': visual_target_feature,
    }
    for image_key in IMAGE_KEYS:
        features[f'observation.images.{image_key}'] = {
            'dtype': image_dtype,
            'shape': (image_height, image_width, 3),
            'names': ['height', 'width', 'channels'],
        }
    return features


def _image_refs(row: dict[str, Any], *, dataset_mode: str = DATASET_MODE_VISUAL_STATE) -> dict[str, str]:
    _ = dataset_mode
    return visual_model_input_image_refs(row)


def _blank_image(*, image_width: int, image_height: int) -> Image.Image:
    return Image.new('RGB', (image_width, image_height), color=(0, 0, 0))


def load_image(
    row: dict[str, Any],
    dataset_root: Path,
    image_key: str,
    *,
    image_width: int,
    image_height: int,
    allow_blank_images: bool = False,
    dataset_mode: str = DATASET_MODE_VISUAL_STATE,
) -> Image.Image:
    image_ref = _image_refs(row, dataset_mode=dataset_mode).get(image_key, '')
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

def load_visual_label_vectorizer(path: Path) -> VisualStateLabelVectorizer:
    with path.expanduser().open('r', encoding='utf-8') as stream:
        parsed = json.load(stream)
    if not isinstance(parsed, dict):
        raise ValueError(f'visual label vectorizer must be a JSON object: {path}')
    return VisualStateLabelVectorizer.from_json(parsed)


def save_visual_label_vectorizer(vectorizer: VisualStateLabelVectorizer, path: Path) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(_pretty_json(vectorizer.to_json()) + '\n', encoding='utf-8')


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
    image_width: int,
    image_height: int,
    fps: int,
    max_episodes: int | None = None,
    max_rows: int | None = None,
    use_videos: bool = False,
    allow_blank_images: bool = False,
    overwrite: bool = False,
    dataset_mode: str = DATASET_MODE_VISUAL_STATE,
    visual_labels_path: Path | None = None,
    visual_label_vectorizer_path: Path | None = None,
    visual_label_vectorizer_out: Path | None = None,
) -> dict[str, Any]:
    if dataset_mode not in DATASET_MODES:
        raise ValueError(f'unknown dataset mode: {dataset_mode}')
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

    if visual_labels_path is None:
        candidate = visual_label_path_for_split(input_file)
        visual_labels_path = candidate if candidate.exists() else None

    labels = load_visual_labels_for_rows(rows, visual_labels_path)
    model_input_integrity = validate_visual_state_rows(rows, visual_labels_path)
    image_integrity = image_integrity_report(
        rows,
        dataset_root,
        allow_blank_images=allow_blank_images,
        dataset_mode=dataset_mode,
    )

    if visual_label_vectorizer_path is not None:
        label_vectorizer = load_visual_label_vectorizer(visual_label_vectorizer_path)
        fitted_label_vectorizer = False
    else:
        label_vectorizer = VisualStateLabelVectorizer.fit(labels)
        fitted_label_vectorizer = True
    visual_target_names = label_vectorizer.names
    features = build_lerobot_features(
        visual_target_names=visual_target_names,
        image_height=image_height,
        image_width=image_width,
        use_videos=use_videos,
        dataset_mode=dataset_mode,
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
    lerobot_task = 'visual_state_perception'
    labels_by_sample = {
        str(row.get('sample_id') or f'{row.get("episode_id", "")}:step:{row.get("step_index", "")}'): label
        for row, label in zip(rows, labels)
    }
    for episode_id, episode_rows in episodes:
        for row in episode_rows:
            sample_id = str(row.get('sample_id') or f'{row.get("episode_id", "")}:step:{row.get("step_index", "")}')
            target = np.asarray(label_vectorizer.transform(labels_by_sample[sample_id]), dtype=np.float32)
            frame = {
                'task': lerobot_task,
                'action': target,
            }
            for image_key in IMAGE_KEYS:
                frame[f'observation.images.{image_key}'] = load_image(
                    row,
                    dataset_root,
                    image_key,
                    image_width=image_width,
                    image_height=image_height,
                    allow_blank_images=allow_blank_images,
                    dataset_mode=dataset_mode,
                )
            dataset.add_frame(frame)
            frame_count += 1
        dataset.save_episode()
        print(f'converted {episode_id}: {len(episode_rows)} frames', flush=True)
    dataset.finalize()

    if fitted_label_vectorizer:
        if visual_label_vectorizer_out is None:
            visual_label_vectorizer_out = output_root / 'room315_visual_state_label_vectorizer.json'
        save_visual_label_vectorizer(label_vectorizer, visual_label_vectorizer_out)
    structured_labels_path = root / 'room315_visual_state_labels.jsonl'
    write_visual_jsonl(
        structured_labels_path,
        [
            {
                'sample_id': str(row.get('sample_id') or ''),
                'episode_id': str(row.get('episode_id') or ''),
                'step_index': row.get('step_index', row.get('event_index')),
                'visual_state_labels': label,
                'label_source': 'oracle',
                'model_input_exposure': 'excluded',
            }
            for row, label in zip(rows, labels)
        ],
    )

    summary = {
        'tool': 'room_315_visual_state_to_lerobot',
        'dataset_mode': dataset_mode,
        'purpose': 'visual-state label dataset conversion; labels are not rail commands',
        'source': str(input_file),
        'source_fingerprint': _file_fingerprint(input_file),
        'row_fingerprint': _rows_fingerprint(rows),
        'dataset_root': str(dataset_root.expanduser().resolve()),
        'output_root': str(root),
        'repo_id': repo_id,
        'name': name,
        'frames': frame_count,
        'episodes': len(episodes),
        'lerobot_task': lerobot_task,
        'fps': fps,
        'use_videos': use_videos,
        'image_width': image_width,
        'image_height': image_height,
        'visual_target_dim': len(visual_target_names),
        'visual_label_vectorizer': str(
            visual_label_vectorizer_path.expanduser().resolve()
            if visual_label_vectorizer_path is not None
            else (visual_label_vectorizer_out or output_root / 'room315_visual_state_label_vectorizer.json').expanduser().resolve()
        ),
        'visual_state_label_schema_version': VISUAL_STATE_SCHEMA_VERSION,
        'structured_visual_labels': str(structured_labels_path),
        'oracle_label_file': str(visual_labels_path) if visual_labels_path is not None else None,
        'oracle_label_fingerprint': _rows_fingerprint(
            [{'visual_state_labels': label} for label in labels]
        ),
        'visual_label_vectorizer_fitted_from_input': fitted_label_vectorizer,
        'feature_keys': sorted(features),
        'model_input_integrity': model_input_integrity,
        'camera_completeness': image_integrity,
        'class_balance': visual_state_class_balance(labels),
        'feature_purity': {
            'production_features_from': 'model_input.overhead_images',
            'row_level_metadata_used_as_features': [],
            'debug_or_ablation_features': [],
            'oracle_labels_physically_separate_from_model_input': True,
        },
    }
    (root / 'room315_conversion.json').write_text(_pretty_json(summary) + '\n', encoding='utf-8')
    print(_pretty_json(summary))
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Convert Room 315 visual-state event JSONL data into a local LeRobot dataset.'
    )
    parser.add_argument(
        'input',
        nargs='?',
        type=Path,
        default=DEFAULT_SPLITS_DIR / 'train.jsonl',
        help=(
            'Room 315 JSONL split, full training_events.jsonl, or dataset directory. '
            'Defaults to ROOM315_VISUAL_SPLITS_DIR/train.jsonl or '
            'room315_local_training/splits/train.jsonl.'
        ),
    )
    parser.add_argument(
        '--dataset-root',
        type=Path,
        default=DEFAULT_DATASET_ROOT,
        help='Dataset root for relative image refs. Defaults to ROOM315_VISUAL_DATASET_ROOT.',
    )
    parser.add_argument(
        '--output-root',
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
        help='LeRobot output root. Defaults to ROOM315_LEROBOT_OUTPUT_ROOT.',
    )
    parser.add_argument('--name', default='room315_visual_state_train')
    parser.add_argument('--repo-id', default='room315/room315_visual_state_train')
    parser.add_argument(
        '--dataset-mode',
        choices=DATASET_MODES,
        default=DATASET_MODE_VISUAL_STATE,
        help=(
            f'{DATASET_MODE_VISUAL_STATE} converts image-only rows plus '
            f'a *{VISUAL_LABEL_SUFFIX} oracle label sidecar.'
        ),
    )
    parser.add_argument(
        '--visual-labels',
        type=Path,
        default=None,
        help='Visual-state oracle label JSONL. Defaults to sibling *_visual_labels.jsonl.',
    )
    parser.add_argument(
        '--visual-label-vectorizer',
        type=Path,
        default=None,
        help='Existing visual-state label vectorizer JSON. Use this for val/test conversions.',
    )
    parser.add_argument(
        '--visual-label-vectorizer-out',
        type=Path,
        default=None,
        help='Where to write a fitted visual-state label vectorizer.',
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

    convert_room315_to_lerobot(
        args.input,
        dataset_root=args.dataset_root.expanduser().resolve(),
        output_root=args.output_root,
        name=args.name,
        repo_id=args.repo_id,
        image_width=args.image_width,
        image_height=args.image_height,
        fps=args.fps,
        max_episodes=args.max_episodes,
        max_rows=args.max_rows,
        use_videos=args.use_videos,
        allow_blank_images=args.allow_blank_images,
        overwrite=args.overwrite,
        dataset_mode=args.dataset_mode,
        visual_labels_path=args.visual_labels,
        visual_label_vectorizer_path=args.visual_label_vectorizer,
        visual_label_vectorizer_out=args.visual_label_vectorizer_out,
    )


if __name__ == '__main__':
    main()
