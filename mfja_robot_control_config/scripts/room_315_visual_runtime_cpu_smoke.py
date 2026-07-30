#!/usr/bin/env python3
"""One-pair CPU inference smoke using an explicitly supplied validation split."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_json_io import iter_jsonl_objects
from room_315_presence_provider import PRESENCE_ABSENT
from room_315_presence_provider import PRESENCE_PRESENT
from room_315_presence_provider import PresenceEntry
from room_315_presence_provider import PresenceSnapshot
from room_315_visual_runtime import ArtifactHashes
from room_315_visual_runtime import ArtifactPaths
from room_315_visual_runtime import FIXED_IDENTITY_ORDER
from room_315_visual_runtime import Room315VisualModelRuntime
from room_315_visual_runtime import decode_active_slots
from room_315_visual_runtime import verify_artifacts
from room_315_visual_runtime_validation import validate_prediction
from room_315_visual_state_dataset import normalize_visual_state_labels
from room_315_visual_state_dataset import visual_model_input_image_refs


def _reject_locked_test_path(path: Path) -> None:
    name = path.name.lower()
    role_tokens = {
        part.lower()
        for part in path.parts
    }
    if name.startswith('test') or role_tokens & {'test', 'locked_test'}:
        raise ValueError(f'locked Test split access is forbidden: {path}')


def _load_first(path: Path) -> dict:
    _reject_locked_test_path(path)
    rows = iter_jsonl_objects(path, error_type=ValueError, require_object=True)
    try:
        return next(rows)
    except StopIteration as exc:
        raise ValueError(f'validation file is empty: {path}') from exc


def _find_label(path: Path, scenario_id: str) -> dict:
    _reject_locked_test_path(path)
    for row in iter_jsonl_objects(path, error_type=ValueError, require_object=True):
        row_id = str(
            row.get('scenario_id')
            or row.get('episode_id')
            or (row.get('traceability_metadata') or {}).get('scenario_id')
            or ''
        )
        if row_id == scenario_id:
            labels = row.get('visual_state_labels')
            return labels if isinstance(labels, dict) else row
    raise ValueError(f'no validation label for scenario {scenario_id!r}')


def _image(root: Path, reference: str) -> np.ndarray:
    path = Path(reference)
    if not path.is_absolute():
        path = root / path
    with Image.open(path) as image:
        return np.asarray(image.convert('RGB'))


def run(args: argparse.Namespace) -> dict:
    split = args.validation_split.expanduser().resolve()
    labels = args.validation_labels.expanduser().resolve()
    _reject_locked_test_path(split)
    _reject_locked_test_path(labels)
    row = _load_first(split)
    scenario_id = str(
        row.get('scenario_id')
        or row.get('episode_id')
        or (row.get('traceability_metadata') or {}).get('scenario_id')
        or ''
    )
    if not scenario_id:
        raise ValueError('validation row is missing scenario_id')
    label = normalize_visual_state_labels(_find_label(labels, scenario_id))
    refs = visual_model_input_image_refs(row)
    dataset_root = args.dataset_root.expanduser().resolve()
    left = _image(dataset_root, refs['left_rail_rgb'])
    right = _image(dataset_root, refs['right_rail_rgb'])

    sidecars = args.sidecar_directory.expanduser().resolve()
    artifacts = verify_artifacts(
        ArtifactPaths(args.checkpoint.expanduser().resolve(), sidecars),
        ArtifactHashes(
            checkpoint=args.checkpoint_sha256,
            target_stats=args.target_stats_sha256,
            vectorizer=args.vectorizer_sha256,
            training_config=args.training_config_sha256,
            run_metadata=args.run_metadata_sha256,
        ),
    )
    runtime = Room315VisualModelRuntime(artifacts, device='cpu')
    runtime.load()
    raw, timings = runtime.infer(left, right)
    entries = []
    for item in label['shuttles']:
        entries.append(PresenceEntry(
            identity=item['id'],
            side='left' if item['id'].startswith('L') else 'right',
            state=(
                PRESENCE_PRESENT
                if bool(item.get('presence'))
                else PRESENCE_ABSENT
            ),
        ))
    presence = PresenceSnapshot(
        timestamp_s=1.0,
        ready=True,
        entries=tuple(entries),
        reasons=(),
        initialized_sides=('left', 'right'),
        stale_sides=(),
        source='validation_fixture_presence_gate_not_model_input',
    )
    prediction = decode_active_slots(
        raw,
        vectorizer=artifacts.vectorizer,
        presence=presence,
        timestamp_s=1.0,
        left_image_stamp_s=1.0,
        right_image_stamp_s=1.0,
        left_image_size=(left.shape[1], left.shape[0]),
        right_image_size=(right.shape[1], right.shape[0]),
    )
    validation = validate_prediction(prediction, presence, now_s=1.0)
    return {
        'schema_version': 'room315.visual_runtime_cpu_smoke.v1',
        'passed': (
            raw.shape == (200,)
            and bool(np.all(np.isfinite(raw)))
            and validation is not None
        ),
        'validation_accepted': validation.accepted,
        'validation_reasons': list(validation.reasons),
        'scenario_id': scenario_id,
        'split_role': 'validation',
        'locked_test_accessed': False,
        'input_shape': [1, 6, 224, 224],
        'output_dimension': int(raw.size),
        'output_finite': bool(np.all(np.isfinite(raw))),
        'identity_order': list(FIXED_IDENTITY_ORDER),
        'active_identities': list(prediction.active_identities),
        'absent_identities': list(prediction.absent_identities),
        'device': runtime.device,
        'model_load_duration_ms': runtime.model_load_duration_ms,
        'timings_ms': {
            'preprocessing': timings.preprocessing_ms,
            'inference': timings.inference_ms,
            'decode': timings.decode_ms,
            'complete_cycle': timings.complete_cycle_ms,
        },
        'checkpoint_sha256': artifacts.hashes['best.pt'],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--sidecar-directory', type=Path, required=True)
    parser.add_argument('--checkpoint-sha256', required=True)
    parser.add_argument('--target-stats-sha256', required=True)
    parser.add_argument('--vectorizer-sha256', required=True)
    parser.add_argument('--training-config-sha256', required=True)
    parser.add_argument('--run-metadata-sha256', required=True)
    parser.add_argument('--validation-split', type=Path, required=True)
    parser.add_argument('--validation-labels', type=Path, required=True)
    parser.add_argument('--dataset-root', type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> None:
    result = run(build_parser().parse_args(argv))
    print(json.dumps(result, indent=2, sort_keys=True))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
