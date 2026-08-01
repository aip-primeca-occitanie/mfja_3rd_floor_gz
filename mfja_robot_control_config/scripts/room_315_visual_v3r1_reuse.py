#!/usr/bin/env python3
"""Validate and immutably import completed V3 episodes into V3R1."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from collections import Counter
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_scenario_runner import _episode_exists
from room_315_visual_state_dataset import normalize_visual_state_labels
from room_315_visual_v3_common import VisualV3Error
from room_315_visual_v3_common import atomic_json
from room_315_visual_v3_common import atomic_jsonl
from room_315_visual_v3_common import atomic_text
from room_315_visual_v3_common import image_valid
from room_315_visual_v3_common import read_jsonl
from room_315_visual_v3_common import sha256_file
from room_315_visual_v3_common import value_sha256
from room_315_visual_v3r1_common import DEFAULT_CAPTURE_ROOT
from room_315_visual_v3r1_common import DEFAULT_GUARD_ROOT
from room_315_visual_v3r1_common import DEFAULT_V3_CAPTURE_ROOT
from room_315_visual_v3r1_common import DEFAULT_V3_GUARD_ROOT
from room_315_visual_v3r1_common import PACKAGE_SCHEMA


CAMERAS = ('left_rail_rgb', 'right_rail_rgb')


def _source_paths(
    source_capture_root: Path,
    source_guard_root: Path,
) -> dict[str, Path]:
    return {
        'scenario_manifest': (
            source_capture_root / 'manifests' / 'train_scenarios.jsonl'
        ),
        'training_events': (
            source_capture_root / 'dataset' / 'meta' / 'training_events.jsonl'
        ),
        'capture_fingerprints': (
            source_capture_root
            / 'dataset'
            / 'meta'
            / 'capture_fingerprints.jsonl'
        ),
        'pipeline_state': source_guard_root / 'full_pipeline_state.json',
    }


def _hashes(paths: dict[str, Path]) -> dict[str, dict[str, Any]]:
    result = {}
    for name, path in paths.items():
        if not path.is_file():
            raise VisualV3Error(f'missing historical V3 artifact: {path}')
        result[name] = {
            'path': str(path),
            'bytes': path.stat().st_size,
            'sha256': sha256_file(path),
        }
    return result


def _validate_episode(
    scenario: dict[str, Any],
    event: dict[str, Any],
    *,
    dataset_root: Path,
) -> dict[str, Any]:
    scenario_id = scenario['scenario_id']
    if event.get('episode_id') != scenario_id:
        raise VisualV3Error(f'{scenario_id}: event episode ID mismatch')
    if event.get('scenario_family') != scenario.get('scenario_family'):
        raise VisualV3Error(f'{scenario_id}: scenario family mismatch')
    if not str(scenario.get('configuration_family_id') or ''):
        raise VisualV3Error(f'{scenario_id}: missing configuration family')
    if not _episode_exists(dataset_root, scenario):
        raise VisualV3Error(f'{scenario_id}: episode is not complete')
    episode = dataset_root / 'episodes' / scenario_id
    validation_path = episode / 'validation.json'
    event_path = episode / 'event.json'
    disk_event = json.loads(event_path.read_text(encoding='utf-8'))
    if value_sha256(disk_event) != value_sha256(event):
        raise VisualV3Error(f'{scenario_id}: event JSONL/disk row mismatch')
    validation = json.loads(validation_path.read_text(encoding='utf-8'))
    if (
        validation.get('scenario_id') != scenario_id
        or validation.get('scenario_family') != scenario['scenario_family']
    ):
        raise VisualV3Error(f'{scenario_id}: validation identity mismatch')
    labels = normalize_visual_state_labels(event)
    present = {
        shuttle['id']: shuttle
        for shuttle in labels['shuttles']
        if shuttle['presence']
    }
    expected = {
        identity: (
            'loaded'
            if identity in set(scenario['loaded_identities'])
            else 'empty'
        )
        for identity in scenario['active_identities']
    }
    if set(present) != set(expected):
        raise VisualV3Error(f'{scenario_id}: present identity mismatch')
    for identity, expected_payload in expected.items():
        shuttle = present[identity]
        if shuttle['loaded_state'] != expected_payload:
            raise VisualV3Error(f'{scenario_id}:{identity}: payload mismatch')
        if (
            shuttle['location']['block']
            != scenario['identity_to_block'][identity]
        ):
            raise VisualV3Error(f'{scenario_id}:{identity}: block mismatch')
        rail = shuttle['rail_position']
        if (
            not rail['available']
            or abs(
                float(rail['s_ratio'])
                - float(scenario['identity_to_s_ratio'][identity])
            ) > 1e-5
            or abs(
                float(rail['s_m'])
                - float(rail['s_ratio'])
                * float(rail['segment_length_m'])
            ) > 1e-5
        ):
            raise VisualV3Error(
                f'{scenario_id}:{identity}: continuous position mismatch'
            )
        if not shuttle.get('camera_observations'):
            raise VisualV3Error(
                f'{scenario_id}:{identity}: camera labels missing'
            )
    files = {
        'event': event_path,
        'validation': validation_path,
    }
    image_records = {}
    for camera in CAMERAS:
        path = (
            episode
            / 'images'
            / camera
            / 'frame_000000.jpg'
        )
        image_records[camera] = image_valid(
            path,
            expected_size=(640, 480),
        )
        files[f'image:{camera}'] = path
    return {
        'scenario_id': scenario_id,
        'configuration_family_id': scenario['configuration_family_id'],
        'configuration_core_family_id': scenario[
            'configuration_core_family_id'
        ],
        'scenario_family': scenario['scenario_family'],
        'row_sha256': value_sha256(event),
        'files': {
            name: {
                'path': str(path),
                'sha256': sha256_file(path),
                'bytes': path.stat().st_size,
            }
            for name, path in files.items()
        },
        'image_pair_sha256': validation['image_pair_sha256'],
        'image_records': image_records,
    }


def scan_reusable(
    *,
    source_capture_root: Path = DEFAULT_V3_CAPTURE_ROOT,
    source_guard_root: Path = DEFAULT_V3_GUARD_ROOT,
) -> dict[str, Any]:
    source_paths = _source_paths(source_capture_root, source_guard_root)
    before = _hashes(source_paths)
    scenarios = read_jsonl(source_paths['scenario_manifest'])
    events = read_jsonl(source_paths['training_events'])
    fingerprints = read_jsonl(source_paths['capture_fingerprints'])
    event_counts = Counter(str(row.get('episode_id')) for row in events)
    event_by_id = {str(row.get('episode_id')): row for row in events}
    fingerprint_by_sample = {
        str(row.get('sample_id')): row for row in fingerprints
    }
    reusable = []
    rejected = []
    dataset_root = source_capture_root / 'dataset'
    for scenario in scenarios:
        scenario_id = str(scenario['scenario_id'])
        if event_counts[scenario_id] != 1:
            rejected.append({
                'scenario_id': scenario_id,
                'reason': (
                    'missing_promoted_event'
                    if event_counts[scenario_id] == 0
                    else 'duplicate_promoted_event'
                ),
            })
            continue
        try:
            record = _validate_episode(
                scenario,
                event_by_id[scenario_id],
                dataset_root=dataset_root,
            )
            sample_id = str(event_by_id[scenario_id]['sample_id'])
            fingerprint = fingerprint_by_sample.get(sample_id)
            if not fingerprint:
                raise VisualV3Error(
                    f'{scenario_id}: capture fingerprint row missing'
                )
            if (
                fingerprint.get('image_pair_sha256')
                != record['image_pair_sha256']
            ):
                raise VisualV3Error(
                    f'{scenario_id}: image-pair fingerprint mismatch'
                )
            record['source_generation_index'] = scenario['generation_index']
            record['source_sample_id'] = sample_id
            reusable.append(record)
        except (OSError, ValueError, VisualV3Error) as exc:
            rejected.append({
                'scenario_id': scenario_id,
                'reason': f'validation_failed:{exc}',
            })
    after = _hashes(source_paths)
    preserved = before == after
    return {
        'schema_version': PACKAGE_SCHEMA,
        'source_capture_root': str(source_capture_root),
        'source_guard_root': str(source_guard_root),
        'source_manifest_sha256': before['scenario_manifest']['sha256'],
        'source_scenario_count': len(scenarios),
        'promoted_event_count': len(events),
        'promoted_fingerprint_count': len(fingerprints),
        'reusable_scenario_count': len(reusable),
        'rejected_scenario_count': len(rejected),
        'rejected_incomplete_scenario_count': sum(
            row['reason'] == 'missing_promoted_event'
            for row in rejected
        ),
        'rejected_validation_failure_count': sum(
            row['reason'].startswith('validation_failed:')
            for row in rejected
        ),
        'reusable': reusable,
        'rejected': rejected,
        'source_hashes_before': before,
        'source_hashes_after': after,
        'source_preserved': preserved,
        'passed': (
            preserved
            and len(reusable) == len(events)
            and len(reusable) == len(fingerprints)
            and not any(
                row['reason'].startswith('validation_failed:')
                for row in rejected
            )
        ),
    }


def _link_or_copy(source: Path, destination: Path) -> str:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        if sha256_file(destination) != sha256_file(source):
            raise VisualV3Error(
                f'import destination hash mismatch: {destination}'
            )
        return 'existing_verified'
    try:
        os.link(source, destination)
        method = 'hard_link'
    except OSError:
        shutil.copy2(source, destination)
        method = 'verified_copy'
    if sha256_file(destination) != sha256_file(source):
        raise VisualV3Error(f'import hash mismatch: {destination}')
    return method


def import_reusable(
    scan: dict[str, Any],
    *,
    source_capture_root: Path = DEFAULT_V3_CAPTURE_ROOT,
    destination_capture_root: Path = DEFAULT_CAPTURE_ROOT,
    guard_root: Path = DEFAULT_GUARD_ROOT,
) -> dict[str, Any]:
    if scan.get('passed') is not True:
        raise VisualV3Error('refusing to import from a failed V3 reuse scan')
    source_manifest = read_jsonl(
        source_capture_root / 'manifests' / 'train_scenarios.jsonl'
    )
    source_by_id = {row['scenario_id']: row for row in source_manifest}
    source_events = read_jsonl(
        source_capture_root / 'dataset' / 'meta' / 'training_events.jsonl'
    )
    source_event_by_id = {row['episode_id']: row for row in source_events}
    source_fingerprints = read_jsonl(
        source_capture_root
        / 'dataset'
        / 'meta'
        / 'capture_fingerprints.jsonl'
    )
    fingerprint_by_sample = {
        row['sample_id']: row for row in source_fingerprints
    }
    destination_dataset = destination_capture_root / 'dataset'
    imported_events = []
    imported_fingerprints = []
    provenance = []
    methods = Counter()
    for record in scan['reusable']:
        scenario_id = record['scenario_id']
        source_episode = (
            source_capture_root / 'dataset' / 'episodes' / scenario_id
        )
        destination_episode = (
            destination_dataset / 'episodes' / scenario_id
        )
        file_map = {}
        for relative in (
            Path('event.json'),
            Path('validation.json'),
            Path('images/left_rail_rgb/frame_000000.jpg'),
            Path('images/right_rail_rgb/frame_000000.jpg'),
        ):
            source = source_episode / relative
            destination = destination_episode / relative
            method = _link_or_copy(source, destination)
            methods[method] += 1
            file_map[str(relative)] = {
                'source': str(source),
                'destination': str(destination),
                'method': method,
                'source_sha256': sha256_file(source),
                'destination_sha256': sha256_file(destination),
            }
        event = source_event_by_id[scenario_id]
        imported_events.append(event)
        imported_fingerprints.append(
            fingerprint_by_sample[event['sample_id']]
        )
        source_scenario = source_by_id[scenario_id]
        provenance.append({
            'source_profile': 'train',
            'imported_from_v3': True,
            'source_scenario_id': scenario_id,
            'source_manifest_sha256': scan['source_manifest_sha256'],
            'v3r1_manifest_revision': 'V3R1',
            'configuration_family_id': source_scenario[
                'configuration_family_id'
            ],
            'files': file_map,
        })
    meta = destination_dataset / 'meta'
    atomic_jsonl(meta / 'training_events.jsonl', imported_events)
    atomic_jsonl(
        meta / 'capture_fingerprints.jsonl',
        imported_fingerprints,
    )
    destination_hashes = {
        'training_events': {
            'path': str(meta / 'training_events.jsonl'),
            'sha256': sha256_file(meta / 'training_events.jsonl'),
            'rows': len(imported_events),
        },
        'capture_fingerprints': {
            'path': str(meta / 'capture_fingerprints.jsonl'),
            'sha256': sha256_file(meta / 'capture_fingerprints.jsonl'),
            'rows': len(imported_fingerprints),
        },
    }
    final_scan = scan_reusable(
        source_capture_root=source_capture_root,
        source_guard_root=Path(scan['source_guard_root']),
    )
    source_preserved = (
        final_scan['source_hashes_after'] == scan['source_hashes_before']
    )
    report = {
        **scan,
        'destination_capture_root': str(destination_capture_root),
        'imported_scenario_count': len(imported_events),
        'imported_image_count': len(imported_events) * len(CAMERAS),
        'imported_jsonl_record_count': (
            len(imported_events) + len(imported_fingerprints)
        ),
        'imported_training_event_count': len(imported_events),
        'imported_fingerprint_count': len(imported_fingerprints),
        'import_methods': dict(methods),
        'v3r1_hashes': destination_hashes,
        'provenance_mapping': provenance,
        'source_preserved_after_import': source_preserved,
        'passed': scan['passed'] and source_preserved,
    }
    guard_root.mkdir(parents=True, exist_ok=True)
    json_path = guard_root / 'v3_to_v3r1_reuse_audit.json'
    md_path = guard_root / 'v3_to_v3r1_reuse_audit.md'
    atomic_json(json_path, report)
    atomic_text(
        md_path,
        '\n'.join([
            '# Room 315 V3 to V3R1 reuse audit',
            '',
            f'- Result: **{"PASS" if report["passed"] else "FAIL"}**',
            f'- V3 manifest scenarios: {report["source_scenario_count"]}',
            f'- Promoted V3 rows: {report["promoted_event_count"]}',
            f'- Reusable scenarios: {report["reusable_scenario_count"]}',
            (
                '- Rejected uncaptured scenarios: '
                f'{report["rejected_incomplete_scenario_count"]}'
            ),
            (
                '- Rejected validation failures: '
                f'{report["rejected_validation_failure_count"]}'
            ),
            f'- Imported images: {report["imported_image_count"]}',
            (
                '- Imported JSONL records: '
                f'{report["imported_jsonl_record_count"]}'
            ),
            f'- Import methods: `{dict(methods)}`',
            f'- Historical V3 preserved: {source_preserved}',
            '',
        ]),
    )
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-capture-root', type=Path, default=DEFAULT_V3_CAPTURE_ROOT)
    parser.add_argument('--source-guard-root', type=Path, default=DEFAULT_V3_GUARD_ROOT)
    parser.add_argument('--destination-capture-root', type=Path, default=DEFAULT_CAPTURE_ROOT)
    parser.add_argument('--guard-root', type=Path, default=DEFAULT_GUARD_ROOT)
    parser.add_argument('--import', dest='perform_import', action='store_true')
    args = parser.parse_args(argv)
    scan = scan_reusable(
        source_capture_root=args.source_capture_root,
        source_guard_root=args.source_guard_root,
    )
    result = (
        import_reusable(
            scan,
            source_capture_root=args.source_capture_root,
            destination_capture_root=args.destination_capture_root,
            guard_root=args.guard_root,
        )
        if args.perform_import
        else scan
    )
    print(json.dumps({
        'passed': result['passed'],
        'reusable_scenario_count': result['reusable_scenario_count'],
        'rejected_scenario_count': result['rejected_scenario_count'],
        'imported_scenario_count': result.get('imported_scenario_count', 0),
    }, indent=2, sort_keys=True))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, VisualV3Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
