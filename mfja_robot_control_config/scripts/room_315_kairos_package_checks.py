#!/usr/bin/env python3
"""Fail-closed fixed-eight dataset checks for future Kairos packages."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_fleet import FIXED_VISUAL_SHUTTLE_IDENTITIES
from room_315_visual_state_dataset import VisualStateLabelVectorizer
from room_315_visual_state_dataset import iter_jsonl
from room_315_visual_state_dataset import normalize_visual_state_labels
from room_315_visual_state_dataset import sample_id_for_row


class KairosPackageValidationError(ValueError):
    """Raised when a future training package is not fixed-eight complete."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise KairosPackageValidationError(message)


def _label_paths(package_root: Path) -> list[Path]:
    dataset_dir = package_root / 'dataset'
    return sorted(dataset_dir.glob('*_visual_labels.jsonl'))


def validate_kairos_package(package_root: Path) -> dict[str, Any]:
    package_root = package_root.expanduser().resolve()
    _require(package_root.is_dir(), f'package root is missing: {package_root}')
    label_paths = _label_paths(package_root)
    _require(
        bool(label_paths),
        'Kairos package contains no *_visual_labels.jsonl files',
    )
    all_labels = []
    present_counts = Counter()
    file_reports = {}
    sample_ids = set()
    for path in label_paths:
        rows = iter_jsonl(path)
        _require(bool(rows), f'Kairos label split is empty: {path}')
        file_present = Counter()
        for index, row in enumerate(rows):
            sample_id = sample_id_for_row(row, index)
            _require(
                sample_id not in sample_ids,
                f'duplicate Kairos label sample_id: {sample_id}',
            )
            sample_ids.add(sample_id)
            label = normalize_visual_state_labels(
                row,
                context=f'{path}:{index + 1}',
            )
            identities = tuple(item['id'] for item in label['shuttles'])
            _require(
                identities == FIXED_VISUAL_SHUTTLE_IDENTITIES,
                f'{path}:{index + 1} does not contain the fixed eight-entry schema',
            )
            for shuttle in label['shuttles']:
                if shuttle['presence']:
                    file_present[shuttle['id']] += 1
                    present_counts[shuttle['id']] += 1
            all_labels.append(label)
        missing_in_file = sorted(
            set(FIXED_VISUAL_SHUTTLE_IDENTITIES) - set(file_present)
        )
        _require(
            not missing_in_file,
            f'{path} has no present samples for identities: {missing_in_file}',
        )
        file_reports[path.name] = {
            'rows': len(rows),
            'present_identity_counts': dict(sorted(file_present.items())),
        }
    missing = sorted(
        set(FIXED_VISUAL_SHUTTLE_IDENTITIES) - set(present_counts)
    )
    _require(not missing, f'Kairos package is missing identities: {missing}')
    vectorizer = VisualStateLabelVectorizer.fit(all_labels)
    _require(
        vectorizer.to_json()['capacity_inferred_from_dataset'] is False,
        'Kairos vectorizer capacity was inferred from a dataset subset',
    )
    _require(
        vectorizer.to_json()['fixed_identity_order']
        == list(FIXED_VISUAL_SHUTTLE_IDENTITIES),
        'Kairos vectorizer fixed identity order is invalid',
    )
    return {
        'passed': True,
        'package_root': str(package_root),
        'label_files': file_reports,
        'fixed_identity_order': list(FIXED_VISUAL_SHUTTLE_IDENTITIES),
        'present_identity_counts': dict(sorted(present_counts.items())),
        'vectorizer_output_dim': vectorizer.dim,
        'capacity_inferred_from_dataset': False,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Validate fixed-eight completeness of a future Room 315 Kairos package.'
    )
    parser.add_argument('--package-root', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = validate_kairos_package(args.package_root)
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.report is not None:
        report_path = args.report.expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(rendered + '\n', encoding='utf-8')
    print(rendered)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (KairosPackageValidationError, OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
