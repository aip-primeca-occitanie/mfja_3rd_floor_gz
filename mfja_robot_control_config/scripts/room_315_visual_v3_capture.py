#!/usr/bin/env python3
"""Guarded capture/resume/finalisation for Room 315 hard-case visual V3."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_scenario_runner import _episode_exists
from room_315_visual_state_dataset import sanitized_visual_state_row
from room_315_visual_state_dataset import validate_visual_state_rows
from room_315_visual_v3_common import CAMERAS
from room_315_visual_v3_common import DEFAULT_CANARY_ROOT
from room_315_visual_v3_common import DEFAULT_CAPTURE_ROOT
from room_315_visual_v3_common import DEFAULT_GUARD_ROOT
from room_315_visual_v3_common import PACKAGE_SCHEMA
from room_315_visual_v3_common import VisualV3Error
from room_315_visual_v3_common import atomic_json
from room_315_visual_v3_common import atomic_jsonl
from room_315_visual_v3_common import image_valid
from room_315_visual_v3_common import read_jsonl
from room_315_visual_v3_common import sha256_file


PROFILE_COUNTS = {'train': 4000, 'validation': 512, 'canary': 256}


def _paths(
    profile: str,
    *,
    capture_root: Path,
    canary_root: Path,
    guard_root: Path,
) -> tuple[Path, Path, Path]:
    if profile == 'smoke':
        root = guard_root / 'smoke'
        return root, root / 'scenario_manifest.jsonl', root / 'dataset'
    if profile in {'train', 'validation'}:
        return (
            capture_root,
            capture_root / 'manifests' / f'{profile}_scenarios.jsonl',
            capture_root / 'dataset',
        )
    if profile == 'canary':
        return (
            canary_root,
            canary_root / 'manifests' / 'canary_scenarios.jsonl',
            canary_root / 'dataset',
        )
    raise VisualV3Error(f'unsupported capture profile: {profile}')


def capture_status(manifest: Path, dataset_root: Path) -> dict[str, Any]:
    scenarios = read_jsonl(manifest)
    complete = []
    incomplete = []
    for scenario in scenarios:
        try:
            if _episode_exists(dataset_root, scenario):
                complete.append(scenario['scenario_id'])
            else:
                incomplete.append(scenario['scenario_id'])
        except ValueError as exc:
            incomplete.append(scenario['scenario_id'])
    return {
        'schema_version': PACKAGE_SCHEMA,
        'manifest': str(manifest),
        'manifest_sha256': sha256_file(manifest),
        'dataset_root': str(dataset_root),
        'scenario_count': len(scenarios),
        'complete_count': len(complete),
        'incomplete_count': len(incomplete),
        'first_incomplete_scenario_id': incomplete[0] if incomplete else None,
        'complete': complete,
        'incomplete': incomplete,
        'capture_complete': not incomplete,
    }


def _runner_command(
    manifest: Path,
    dataset_root: Path,
    *,
    resume: bool,
    keep_going: bool,
    limit: int | None,
    gui: bool,
) -> list[str]:
    command = [
        sys.executable,
        str(SCRIPT_DIR / 'room_315_visual_scenario_runner.py'),
        '--scenario-manifest',
        str(manifest),
        '--output-dataset',
        str(dataset_root),
        '--readiness-timeout-seconds',
        '45',
        '--capture-timeout-seconds',
        '30',
        '--max-camera-skew-seconds',
        '0.15',
    ]
    if resume:
        command.append('--resume')
    if keep_going:
        command.append('--keep-going')
    if gui:
        command.append('--gui')
    if limit is not None:
        command.extend(('--limit', str(limit)))
    return command


def _verify_manifest_lock(
    profile: str,
    output_root: Path,
    manifest: Path,
) -> None:
    lock_path = output_root / 'manifest_lock.json'
    if not lock_path.is_file():
        raise VisualV3Error(f'missing immutable manifest lock: {lock_path}')
    lock = json.loads(lock_path.read_text(encoding='utf-8'))
    expected = (
        lock
        if profile == 'smoke'
        else (lock.get('profiles') or {}).get(profile)
    )
    if not isinstance(expected, dict):
        raise VisualV3Error(f'manifest lock does not contain profile {profile}')
    actual_hash = sha256_file(manifest)
    if (
        int(expected.get('scenario_count', -1)) != len(read_jsonl(manifest))
        or expected.get('scenario_manifest_sha256') != actual_hash
    ):
        raise VisualV3Error(
            f'{profile} manifest differs from immutable lock: {manifest}'
        )


def finalize_profile(
    profile: str,
    manifest: Path,
    dataset_root: Path,
    output_root: Path,
) -> dict[str, Any]:
    scenarios = read_jsonl(manifest)
    status = capture_status(manifest, dataset_root)
    if not status['capture_complete']:
        raise VisualV3Error(
            f'cannot finalize {profile}: {status["incomplete_count"]} scenarios incomplete'
        )
    raw_path = dataset_root / 'meta' / 'training_events.jsonl'
    raw_rows = read_jsonl(raw_path)
    raw_by_episode = {row['episode_id']: row for row in raw_rows}
    model_rows = []
    label_rows = []
    image_hashes = {}
    image_pair_hashes = {}
    for index, scenario in enumerate(scenarios):
        scenario_id = scenario['scenario_id']
        if scenario_id not in raw_by_episode:
            raise VisualV3Error(f'{scenario_id}: missing captured event')
        model_row, label_row = sanitized_visual_state_row(
            raw_by_episode[scenario_id],
            index,
        )
        trace = {
            'scenario_id': scenario_id,
            'spec_id': scenario['spec_id'],
            'generation_index': scenario['generation_index'],
            'dataset_partition': profile,
            'configuration_family_id': scenario['configuration_family_id'],
            'configuration_core_family_id': scenario[
                'configuration_core_family_id'
            ],
            'geometry_fingerprint': scenario['geometry_fingerprint'],
            'capture_configuration_fingerprint': scenario[
                'capture_configuration_fingerprint'
            ],
            'active_identities': scenario['active_identities'],
            'loaded_identities': scenario['loaded_identities'],
            'identity_to_block': scenario['identity_to_block'],
            'identity_to_position_bin': scenario['identity_to_position_bin'],
            'identity_to_s_m': scenario['identity_to_s_m'],
            'identity_to_s_ratio': scenario['identity_to_s_ratio'],
            'identity_to_segment_length_m': scenario[
                'identity_to_segment_length_m'
            ],
            'identity_to_zone': scenario['identity_to_zone'],
            'relation_family': scenario['relation_family'],
            'target_identity': scenario['target_identity'],
            'target_zone': scenario['target_zone'],
            'target_offset_bucket': scenario['target_offset_bucket'],
            'target_offset': scenario.get('target_offset'),
            'target_ratio': scenario.get('target_ratio'),
            'operational_target_name': scenario.get(
                'operational_target_name'
            ),
            'operational_target_segment': scenario.get(
                'operational_target_segment'
            ),
            'presence_class': scenario.get('presence_class'),
            'configuration_variant': scenario.get(
                'configuration_variant'
            ),
            'occlusion_class': scenario['occlusion_class'],
            'render_bucket': scenario['render_variation']['bucket'],
            'canary_family': scenario['canary_family'],
            'matched_pair_id': scenario.get('matched_pair_id'),
            'matched_pair_role': scenario.get('matched_pair_role'),
            'hard_case_tags': scenario['hard_case_tags'],
            'source_profile': scenario.get('source_profile'),
            'imported_from_v3': bool(
                scenario.get('imported_from_v3', False)
            ),
            'source_scenario_id': scenario.get('source_scenario_id'),
            'source_manifest_sha256': scenario.get(
                'source_manifest_sha256'
            ),
            'v3r1_manifest_revision': scenario.get(
                'v3r1_manifest_revision'
            ),
        }
        model_row['traceability_metadata'] = trace
        label_row['traceability_metadata'] = trace
        model_rows.append(model_row)
        label_rows.append(label_row)
        for camera in CAMERAS:
            image_ref = model_row['model_input']['overhead_images'][camera]
            image_path = dataset_root / image_ref
            verification = image_valid(image_path, expected_size=(640, 480))
            image_hashes[f'{scenario_id}:{camera}'] = verification['sha256']
        validation_record = json.loads(
            (
                dataset_root
                / 'episodes'
                / scenario_id
                / 'validation.json'
            ).read_text(encoding='utf-8')
        )
        image_pair_hashes[scenario_id] = validation_record['image_pair_sha256']
    data_dir = output_root / 'finalized'
    rows_path = data_dir / f'{profile}.jsonl'
    labels_path = data_dir / f'{profile}_visual_labels.jsonl'
    atomic_jsonl(rows_path, model_rows)
    atomic_jsonl(labels_path, label_rows)
    validation = validate_visual_state_rows(model_rows, labels_path)
    if validation.get('issues'):
        raise VisualV3Error(f'{profile} finalized label validation failed')
    report = {
        'schema_version': PACKAGE_SCHEMA,
        'profile': profile,
        'scenario_count': len(model_rows),
        'image_count': len(image_hashes),
        'rows_path': str(rows_path),
        'rows_sha256': sha256_file(rows_path),
        'labels_path': str(labels_path),
        'labels_sha256': sha256_file(labels_path),
        'image_hashes': image_hashes,
        'individual_image_hash_unique': (
            len(image_hashes) == len(set(image_hashes.values()))
        ),
        'image_pair_hashes': image_pair_hashes,
        'image_pair_hash_unique': (
            len(image_pair_hashes) == len(set(image_pair_hashes.values()))
        ),
        'visual_state_validation': validation,
        'passed': (
            len(image_hashes) == len(model_rows) * 2
            and len(image_pair_hashes) == len(set(image_pair_hashes.values()))
            and not validation.get('issues')
        ),
    }
    atomic_json(data_dir / f'{profile}_finalization.json', report)
    return report


def run_capture(
    profile: str,
    *,
    capture_root: Path,
    canary_root: Path,
    guard_root: Path,
    resume: bool,
    keep_going: bool,
    limit: int | None,
    gui: bool,
    dry_run: bool,
) -> dict[str, Any]:
    output_root, manifest, dataset_root = _paths(
        profile,
        capture_root=capture_root,
        canary_root=canary_root,
        guard_root=guard_root,
    )
    if not (output_root / 'generation_configuration.json').is_file():
        raise VisualV3Error(f'uninitialized V3 output root: {output_root}')
    if not manifest.is_file():
        raise VisualV3Error(f'missing V3 manifest: {manifest}')
    _verify_manifest_lock(profile, output_root, manifest)
    command = _runner_command(
        manifest,
        dataset_root,
        resume=resume,
        keep_going=keep_going,
        limit=limit,
        gui=gui,
    )
    if dry_run:
        return {'dry_run': True, 'command': command}
    completed = subprocess.run(command, check=False)
    status = capture_status(manifest, dataset_root)
    status['runner_returncode'] = completed.returncode
    status_path = output_root / f'{profile}_capture_state.json'
    atomic_json(status_path, status)
    if completed.returncode != 0:
        raise VisualV3Error(
            f'{profile} capture failed; inspect {dataset_root / "meta" / "last_scenario_run.json"}'
        )
    if limit is None and status['capture_complete']:
        status['finalization'] = finalize_profile(
            profile,
            manifest,
            dataset_root,
            output_root,
        )
        atomic_json(status_path, status)
    return status


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'command',
        choices=('capture', 'status', 'finalize'),
    )
    parser.add_argument(
        '--profile',
        choices=('smoke', 'train', 'validation', 'canary'),
        required=True,
    )
    parser.add_argument('--capture-root', type=Path, default=DEFAULT_CAPTURE_ROOT)
    parser.add_argument('--canary-root', type=Path, default=DEFAULT_CANARY_ROOT)
    parser.add_argument('--guard-root', type=Path, default=DEFAULT_GUARD_ROOT)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--keep-going', action='store_true')
    parser.add_argument('--limit', type=int)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    output_root, manifest, dataset_root = _paths(
        args.profile,
        capture_root=args.capture_root,
        canary_root=args.canary_root,
        guard_root=args.guard_root,
    )
    if args.command == 'status':
        result = capture_status(manifest, dataset_root)
    elif args.command == 'finalize':
        result = finalize_profile(
            args.profile,
            manifest,
            dataset_root,
            output_root,
        )
    else:
        result = run_capture(
            args.profile,
            capture_root=args.capture_root,
            canary_root=args.canary_root,
            guard_root=args.guard_root,
            resume=args.resume,
            keep_going=args.keep_going,
            limit=args.limit,
            gui=args.gui,
            dry_run=args.dry_run,
        )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, VisualV3Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
