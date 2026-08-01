#!/usr/bin/env python3
"""Create/verify train+validation-only grouped Room 315 visual V3 splits."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_state_dataset import normalize_visual_state_labels
from room_315_visual_v3_common import DEFAULT_CANARY_ROOT
from room_315_visual_v3_common import DEFAULT_CAPTURE_ROOT
from room_315_visual_v3_common import DEFAULT_OLD_TRAIN
from room_315_visual_v3_common import DEFAULT_OLD_TRAIN_LABELS
from room_315_visual_v3_common import DEFAULT_SPLIT_ROOT
from room_315_visual_v3_common import IDENTITIES
from room_315_visual_v3_common import PACKAGE_SCHEMA
from room_315_visual_v3_common import SEED
from room_315_visual_v3_common import VisualV3Error
from room_315_visual_v3_common import assert_allowed_input
from room_315_visual_v3_common import atomic_json
from room_315_visual_v3_common import atomic_jsonl
from room_315_visual_v3_common import atomic_text
from room_315_visual_v3_common import configuration_family_id
from room_315_visual_v3_common import position_bin
from room_315_visual_v3_common import prepare_output_root
from room_315_visual_v3_common import read_jsonl
from room_315_visual_v3_common import sha256_file
from room_315_visual_v3_common import value_sha256


def _old_replay_core_families(
    rows_path: Path,
    labels_path: Path,
) -> set[str]:
    rows_path = assert_allowed_input(rows_path)
    labels_path = assert_allowed_input(labels_path)
    rows = read_jsonl(rows_path)
    labels = read_jsonl(labels_path)
    labels_by_sample = {row['sample_id']: row for row in labels}
    families = set()
    for row in rows:
        label_row = labels_by_sample.get(row.get('sample_id'))
        if label_row is None:
            raise VisualV3Error(f'old replay label missing: {row.get("sample_id")}')
        visual = normalize_visual_state_labels(label_row)
        present = [
            shuttle for shuttle in visual['shuttles']
            if shuttle['presence']
        ]
        metadata = row.get('stratification_metadata') or {}
        record = {
            'active_identities': [
                identity for identity in IDENTITIES
                if any(shuttle['id'] == identity for shuttle in present)
            ],
            'loaded_identities': [
                identity for identity in IDENTITIES
                if any(
                    shuttle['id'] == identity
                    and shuttle['loaded_state'] == 'loaded'
                    for shuttle in present
                )
            ],
            'identity_to_block': {
                shuttle['id']: shuttle['location']['block']
                for shuttle in present
            },
            'identity_to_position_bin': {
                shuttle['id']: position_bin(shuttle['rail_position']['s_ratio'])
                for shuttle in present
            },
            'relation_family': str(
                metadata.get('relation_family') or 'no_relation_observation'
            ),
            'target_identity': str(metadata.get('target_identity') or 'unspecified'),
            'target_zone': str(metadata.get('target_zone') or 'ordinary'),
            'target_offset_bucket': 'not_operational_target',
            'approach_direction': 'unspecified',
            'occlusion_class': (
                'partial_risk'
                if metadata.get('partial_occlusion_risk')
                else 'clear'
            ),
            'render_bucket': 'legacy',
        }
        for approach in ('increasing_s', 'decreasing_s', 'unspecified'):
            families.add(configuration_family_id(
                {**record, 'approach_direction': approach},
                include_render=False,
            ))
    return families


def _sets(rows: list[dict[str, Any]]) -> dict[str, set[str]]:
    return {
        'scenario_ids': {
            str((row.get('traceability_metadata') or {}).get('scenario_id'))
            for row in rows
        },
        'sample_ids': {str(row.get('sample_id')) for row in rows},
        'families': {
            str((row.get('traceability_metadata') or {}).get(
                'configuration_family_id'
            ))
            for row in rows
        },
        'core_families': {
            str((row.get('traceability_metadata') or {}).get(
                'configuration_core_family_id'
            ))
            for row in rows
        },
        'row_hashes': {value_sha256(row) for row in rows},
    }


def _image_hashes(finalization_path: Path) -> set[str]:
    value = json.loads(finalization_path.read_text(encoding='utf-8'))
    return set(value['image_hashes'].values())


def overlap_audit(
    train_rows: list[dict[str, Any]],
    validation_rows: list[dict[str, Any]],
    canary_rows: list[dict[str, Any]],
    *,
    old_replay_core: set[str],
    image_hashes: dict[str, set[str]],
) -> dict[str, Any]:
    by_name = {
        'train': _sets(train_rows),
        'validation': _sets(validation_rows),
        'canary': _sets(canary_rows),
    }
    pairwise: dict[str, Any] = {}
    for first, second in (
        ('train', 'validation'),
        ('train', 'canary'),
        ('validation', 'canary'),
    ):
        name = f'{first}_vs_{second}'
        pairwise[name] = {
            'scenario_id_overlap': sorted(
                by_name[first]['scenario_ids'] & by_name[second]['scenario_ids']
            ),
            'sample_id_overlap': sorted(
                by_name[first]['sample_ids'] & by_name[second]['sample_ids']
            ),
            'configuration_family_overlap': sorted(
                by_name[first]['families'] & by_name[second]['families']
            ),
            'row_hash_overlap': sorted(
                by_name[first]['row_hashes'] & by_name[second]['row_hashes']
            ),
            'image_sha256_overlap': sorted(
                image_hashes[first] & image_hashes[second]
            ),
        }
    validation_old_overlap = sorted(
        by_name['validation']['core_families'] & old_replay_core
    )
    violations = []
    for name, checks in pairwise.items():
        for check, values in checks.items():
            if values:
                violations.append(f'{name}:{check}:{len(values)}')
    if validation_old_overlap:
        violations.append(
            f'validation_vs_old_replay_core_family:{len(validation_old_overlap)}'
        )
    return {
        'schema_version': PACKAGE_SCHEMA,
        'split_roles': ['train', 'validation'],
        'canary_role': 'development_regression_only',
        'final_test_created': False,
        'counts': {
            'train': len(train_rows),
            'validation': len(validation_rows),
            'canary': len(canary_rows),
        },
        'family_counts': {
            name: len(sets['families'])
            for name, sets in by_name.items()
        },
        'pairwise': pairwise,
        'old_replay_training_core_family_count': len(old_replay_core),
        'validation_vs_old_replay_core_family_overlap': validation_old_overlap,
        'violations': violations,
        'passed': not violations,
    }


def _markdown(audit: dict[str, Any]) -> str:
    lines = [
        '# Room 315 visual V3 split-overlap audit',
        '',
        f'- Result: **{"PASS" if audit["passed"] else "FAIL"}**',
        '- Split roles: train and validation only',
        '- Canary: development regression only',
        '- Final Test created: no',
        '',
        '| Package | Scenarios | Configuration families |',
        '|---|---:|---:|',
    ]
    for name in ('train', 'validation', 'canary'):
        lines.append(
            f'| {name} | {audit["counts"][name]} | '
            f'{audit["family_counts"][name]} |'
        )
    lines.extend([
        '',
        '## Pairwise overlaps',
        '',
    ])
    for name, checks in audit['pairwise'].items():
        lines.append(f'### {name}')
        lines.append('')
        for check, values in checks.items():
            lines.append(f'- {check}: {len(values)}')
        lines.append('')
    lines.extend([
        '## Old replay isolation',
        '',
        (
            '- Validation versus old replay training core-family overlap: '
            f'{len(audit["validation_vs_old_replay_core_family_overlap"])}'
        ),
        '',
        'The consumed legacy Test was not read, hashed, copied, or referenced.',
        '',
    ])
    return '\n'.join(lines)


def create_split_package(
    *,
    capture_root: Path,
    canary_root: Path,
    output_root: Path,
    old_train: Path,
    old_train_labels: Path,
    resume: bool,
) -> dict[str, Any]:
    configuration = {
        'schema_version': PACKAGE_SCHEMA,
        'seed': SEED,
        'roles': ['train', 'validation'],
        'canary_role': 'development_regression_only',
        'capture_root': str(capture_root.resolve()),
        'canary_root': str(canary_root.resolve()),
        'old_replay_train': str(assert_allowed_input(old_train).resolve()),
        'old_replay_train_labels': str(
            assert_allowed_input(old_train_labels).resolve()
        ),
        'legacy_test_prohibited': True,
    }
    config_sha = prepare_output_root(output_root, configuration, resume=resume)
    source_paths = {
        'train': capture_root / 'finalized' / 'train.jsonl',
        'train_labels': capture_root / 'finalized' / 'train_visual_labels.jsonl',
        'validation': capture_root / 'finalized' / 'validation.jsonl',
        'validation_labels': (
            capture_root / 'finalized' / 'validation_visual_labels.jsonl'
        ),
        'canary': canary_root / 'finalized' / 'canary.jsonl',
        'canary_labels': canary_root / 'finalized' / 'canary_visual_labels.jsonl',
    }
    for name, path in source_paths.items():
        if not path.is_file():
            raise VisualV3Error(f'missing finalized {name}: {path}')
    train_rows = read_jsonl(source_paths['train'])
    validation_rows = read_jsonl(source_paths['validation'])
    canary_rows = read_jsonl(source_paths['canary'])
    if (len(train_rows), len(validation_rows), len(canary_rows)) != (4000, 512, 256):
        raise VisualV3Error('finalized split counts must be 4000/512/256')
    old_core = _old_replay_core_families(old_train, old_train_labels)
    image_hashes = {
        'train': _image_hashes(
            capture_root / 'finalized' / 'train_finalization.json'
        ),
        'validation': _image_hashes(
            capture_root / 'finalized' / 'validation_finalization.json'
        ),
        'canary': _image_hashes(
            canary_root / 'finalized' / 'canary_finalization.json'
        ),
    }
    audit = overlap_audit(
        train_rows,
        validation_rows,
        canary_rows,
        old_replay_core=old_core,
        image_hashes=image_hashes,
    )
    if not audit['passed']:
        raise VisualV3Error(f'grouped overlap audit failed: {audit["violations"]}')
    for name in ('train', 'train_labels', 'validation', 'validation_labels'):
        source = source_paths[name]
        destination = output_root / source.name
        if destination.exists():
            if not resume or sha256_file(destination) != sha256_file(source):
                raise VisualV3Error(f'refusing to overwrite split artifact: {destination}')
        else:
            shutil.copy2(source, destination)
    atomic_json(output_root / 'split_overlap_audit.json', audit)
    atomic_text(output_root / 'split_overlap_audit.md', _markdown(audit))
    manifest = {
        'schema_version': PACKAGE_SCHEMA,
        'configuration_sha256': config_sha,
        'seed': SEED,
        'splits': {
            'train': {
                'rows': str(output_root / 'train.jsonl'),
                'labels': str(output_root / 'train_visual_labels.jsonl'),
                'count': 4000,
                'rows_sha256': sha256_file(output_root / 'train.jsonl'),
                'labels_sha256': sha256_file(
                    output_root / 'train_visual_labels.jsonl'
                ),
                'dataset_root': str(capture_root / 'dataset'),
            },
            'validation': {
                'rows': str(output_root / 'validation.jsonl'),
                'labels': str(output_root / 'validation_visual_labels.jsonl'),
                'count': 512,
                'rows_sha256': sha256_file(output_root / 'validation.jsonl'),
                'labels_sha256': sha256_file(
                    output_root / 'validation_visual_labels.jsonl'
                ),
                'dataset_root': str(capture_root / 'dataset'),
            },
        },
        'canary': {
            'rows': str(source_paths['canary']),
            'labels': str(source_paths['canary_labels']),
            'count': 256,
            'role': 'development_regression_only',
            'dataset_root': str(canary_root / 'dataset'),
        },
        'old_replay': {
            'rows': str(old_train),
            'labels': str(old_train_labels),
            'role': 'training_replay_only',
        },
        'legacy_test_prohibited': True,
        'final_test_created': False,
        'overlap_audit': str(output_root / 'split_overlap_audit.json'),
    }
    atomic_json(output_root / 'package_manifest.json', manifest)
    return manifest


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture-root', type=Path, default=DEFAULT_CAPTURE_ROOT)
    parser.add_argument('--canary-root', type=Path, default=DEFAULT_CANARY_ROOT)
    parser.add_argument('--output-root', type=Path, default=DEFAULT_SPLIT_ROOT)
    parser.add_argument('--old-train', type=Path, default=DEFAULT_OLD_TRAIN)
    parser.add_argument(
        '--old-train-labels',
        type=Path,
        default=DEFAULT_OLD_TRAIN_LABELS,
    )
    parser.add_argument('--resume', action='store_true')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    result = create_split_package(
        capture_root=args.capture_root,
        canary_root=args.canary_root,
        output_root=args.output_root,
        old_train=args.old_train,
        old_train_labels=args.old_train_labels,
        resume=args.resume,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, VisualV3Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
