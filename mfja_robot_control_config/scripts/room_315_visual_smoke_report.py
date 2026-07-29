#!/usr/bin/env python3
"""Create a manual-inspection index for a Room 315 visual smoke package."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_scenario_generator import REQUIRED_CAMERAS
from room_315_visual_scenario_generator import _read_manifest
from room_315_visual_state_dataset import image_integrity_report
from room_315_visual_state_dataset import iter_jsonl


class VisualSmokeReportError(ValueError):
    """Raised when a smoke package cannot be indexed safely."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def build_smoke_inspection_report(
    package_root: Path,
    *,
    require_captured: bool = False,
) -> dict[str, Any]:
    package_root = package_root.expanduser().resolve()
    manifest_path = package_root / 'scenario_manifest.jsonl'
    scenarios = _read_manifest(manifest_path)
    dataset_root = package_root / 'dataset'
    events_path = dataset_root / 'meta' / 'training_events.jsonl'
    event_rows = iter_jsonl(events_path) if events_path.is_file() else []
    events = {
        str(row.get('episode_id') or ''): row
        for row in event_rows
    }
    entries = []
    missing_images = []
    missing_oracle_rows = []
    for scenario in scenarios:
        scenario_id = scenario['scenario_id']
        probe = scenario['relation_probe']
        shuttles = [
            {
                'identity': shuttle['id'],
                'side': side,
                'segment': shuttle['start_position']['segment'],
                's_ratio': shuttle['start_position']['s_ratio'],
                'position_zone': shuttle['start_position']['position_zone'],
                'loaded_state': shuttle['loaded_state'],
            }
            for side in ('left', 'right')
            for shuttle in scenario['scene']['rails'][side]['shuttles']
        ]
        image_paths = {
            camera: (
                dataset_root
                / 'episodes'
                / scenario_id
                / 'images'
                / camera
                / 'frame_000000.jpg'
            )
            for camera in REQUIRED_CAMERAS
        }
        for camera, path in image_paths.items():
            if not path.is_file():
                missing_images.append(f'{scenario_id}:{camera}')
        event = events.get(scenario_id)
        if event is None:
            missing_oracle_rows.append(scenario_id)
        entries.append({
            'scenario_id': scenario_id,
            'scenario_family': scenario['scenario_family'],
            'scene_type': scenario['scene_type'],
            'rail_scope': scenario['rail_scope'],
            'active_identities': [row['identity'] for row in shuttles],
            'target': probe['target_shuttle_id'],
            'relations': probe['relations'],
            'relation_neutral_shuttle_ids': probe[
                'relation_neutral_shuttle_ids'
            ],
            'opposite_rail_neutral_shuttle_ids': probe[
                'opposite_rail_neutral_shuttle_ids'
            ],
            'shuttles': shuttles,
            'images': {
                camera: {
                    'path': str(path),
                    'exists': path.is_file(),
                    'sha256': _sha256(path) if path.is_file() else None,
                }
                for camera, path in image_paths.items()
            },
            'oracle_row_present': event is not None,
            'oracle_schema_version': (
                str((event or {}).get('visual_state_labels', {}).get('schema_version') or '')
            ),
        })
    if require_captured and (missing_images or missing_oracle_rows):
        raise VisualSmokeReportError(
            'captured smoke package is incomplete; '
            f'missing_images={missing_images[:10]}, '
            f'missing_oracle_rows={missing_oracle_rows[:10]}'
        )
    image_integrity = (
        image_integrity_report(
            event_rows,
            dataset_root,
            split_name='fixed-eight-smoke',
            operation='manual inspection report',
            allow_blank_images=False,
        )
        if event_rows
        else {
            'total_rows': 0,
            'complete_rows': 0,
            'complete_row_rate': 0.0,
        }
    )
    return {
        'package_root': str(package_root),
        'scenario_manifest': str(manifest_path),
        'scenario_count': len(scenarios),
        'captured_event_count': len(event_rows),
        'expected_image_count': len(scenarios) * len(REQUIRED_CAMERAS),
        'existing_image_count': (
            len(scenarios) * len(REQUIRED_CAMERAS) - len(missing_images)
        ),
        'missing_images': missing_images,
        'missing_oracle_rows': missing_oracle_rows,
        'capture_complete': not missing_images and not missing_oracle_rows,
        'image_integrity': image_integrity,
        'scenarios': entries,
    }


def _markdown(report: dict[str, Any]) -> str:
    lines = [
        '# Room 315 fixed-eight smoke manual inspection',
        '',
        f'- Scenarios: {report["scenario_count"]}',
        f'- Captured oracle rows: {report["captured_event_count"]}',
        f'- Images: {report["existing_image_count"]}/{report["expected_image_count"]}',
        f'- Capture complete: {report["capture_complete"]}',
        '',
    ]
    for entry in report['scenarios']:
        lines.extend([
            f'## {entry["scenario_id"]}',
            '',
            f'- Family: `{entry["scene_type"]}`',
            f'- Rail scope: `{entry["rail_scope"]}`',
            f'- Target: `{entry["target"]}`',
            f'- Active identities: `{", ".join(entry["active_identities"])}`',
            f'- Relations: `{json.dumps(entry["relations"], sort_keys=True)}`',
            f'- Relation-neutral: '
            f'`{", ".join(entry["relation_neutral_shuttle_ids"])}`',
            f'- Opposite-rail neutral: '
            f'`{", ".join(entry["opposite_rail_neutral_shuttle_ids"])}`',
            '',
            '| Identity | Side | Segment | s_ratio | Zone | Payload |',
            '|---|---|---|---:|---|---|',
        ])
        for shuttle in entry['shuttles']:
            lines.append(
                f'| {shuttle["identity"]} | {shuttle["side"]} | '
                f'{shuttle["segment"]} | {shuttle["s_ratio"]:.6f} | '
                f'{shuttle["position_zone"]} | {shuttle["loaded_state"]} |'
            )
        lines.extend(['', 'Images:', ''])
        for camera, image in entry['images'].items():
            lines.append(
                f'- `{camera}`: `{image["path"]}` '
                f'(`exists={image["exists"]}`)'
            )
        lines.extend([
            f'- Oracle row present: `{entry["oracle_row_present"]}`',
            '',
        ])
    return '\n'.join(lines).rstrip() + '\n'


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument('--package-root', type=Path, required=True)
    parser.add_argument('--require-captured', action='store_true')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = build_smoke_inspection_report(
        args.package_root,
        require_captured=args.require_captured,
    )
    root = args.package_root.expanduser().resolve()
    json_path = root / 'manual_inspection_report.json'
    markdown_path = root / 'manual_inspection_report.md'
    json_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    markdown_path.write_text(_markdown(report), encoding='utf-8')
    print(json.dumps({
        'json': str(json_path),
        'markdown': str(markdown_path),
        'capture_complete': report['capture_complete'],
    }, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, VisualSmokeReportError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
