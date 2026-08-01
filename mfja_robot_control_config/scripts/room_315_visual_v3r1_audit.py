#!/usr/bin/env python3
"""Static, smoke, and final audits for Room 315 visual V3R1."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_state_dataset import normalize_visual_state_labels
from room_315_visual_v3_audit import run_full_audit as run_v3_full_audit
from room_315_visual_v3_common import TARGET_OFFSETS
from room_315_visual_v3_common import VisualV3Error
from room_315_visual_v3_common import atomic_json
from room_315_visual_v3_common import atomic_text
from room_315_visual_v3_common import read_jsonl
from room_315_visual_v3_common import sha256_file
from room_315_visual_v3_common import target_offset_bucket
from room_315_visual_v3_generator import scenario_physical_conflicts
from room_315_visual_v3r1_common import CANARY_EXPLICIT_COUNT
from room_315_visual_v3r1_common import DEFAULT_CANARY_ROOT
from room_315_visual_v3r1_common import DEFAULT_CAPTURE_ROOT
from room_315_visual_v3r1_common import DEFAULT_GUARD_ROOT
from room_315_visual_v3r1_common import DEFAULT_SPLIT_ROOT
from room_315_visual_v3r1_common import OPERATIONAL_TARGET_RATIO
from room_315_visual_v3r1_common import PACKAGE_SCHEMA
from room_315_visual_v3r1_common import SMOKE_COUNT
from room_315_visual_v3r1_common import TRAIN_EXPLICIT_COUNT
from room_315_visual_v3r1_common import VALIDATION_EXPLICIT_COUNT
from room_315_visual_v3r1_common import is_deliberate_offset
from room_315_visual_v3r1_common import presence_class
from room_315_visual_v3r1_common import quota_plan_path


def _offset_detail(rows: list[dict[str, Any]]) -> dict[str, Any]:
    buckets: dict[str, dict[str, Any]] = {}
    expected_buckets = [
        target_offset_bucket(offset) for offset in TARGET_OFFSETS
    ]
    for bucket in expected_buckets:
        selected = [
            row for row in rows
            if is_deliberate_offset(row)
            and row['target_offset_bucket'] == bucket
        ]
        buckets[bucket] = {
            'count': len(selected),
            'payload': dict(Counter(
                row['payload_assignment']['R4'] for row in selected
            )),
            'presence_class': dict(Counter(
                row.get('presence_class')
                or presence_class(len(row['active_identities']))
                for row in selected
            )),
            'relation_family': dict(Counter(
                row['relation_family'] for row in selected
            )),
            'loaded_identity_set': dict(Counter(
                '+'.join(row['loaded_identities']) or 'none'
                for row in selected
            )),
            'configuration_families': sorted({
                row['configuration_family_id'] for row in selected
            }),
        }
    deliberate = [row for row in rows if is_deliberate_offset(row)]
    incidental = []
    for row in rows:
        if is_deliberate_offset(row):
            continue
        if (
            'R4' in row['active_identities']
            and row['identity_to_block'].get('R4') == 'A34E'
            and abs(
                float(row['identity_to_s_ratio']['R4'])
                - OPERATIONAL_TARGET_RATIO
            ) <= 0.15
        ):
            incidental.append(row)
    return {
        'deliberate_exact_offset_count': len(deliberate),
        'incidental_nearby_count': len(incidental),
        'by_offset': buckets,
        'payload': dict(Counter(
            row['payload_assignment']['R4'] for row in deliberate
        )),
        'presence_class': dict(Counter(
            row.get('presence_class')
            or presence_class(len(row['active_identities']))
            for row in deliberate
        )),
        'relation_family': dict(Counter(
            row['relation_family'] for row in deliberate
        )),
        'configuration_family_count': len({
            row['configuration_family_id'] for row in deliberate
        }),
    }


def _manifest_paths(
    capture_root: Path,
    canary_root: Path,
    guard_root: Path,
) -> dict[str, Path]:
    return {
        'train': capture_root / 'manifests' / 'train_scenarios.jsonl',
        'validation': (
            capture_root / 'manifests' / 'validation_scenarios.jsonl'
        ),
        'canary': canary_root / 'manifests' / 'canary_scenarios.jsonl',
        'smoke': guard_root / 'smoke' / 'scenario_manifest.jsonl',
    }


def static_audit(
    *,
    capture_root: Path,
    canary_root: Path,
    guard_root: Path,
) -> dict[str, Any]:
    paths = _manifest_paths(capture_root, canary_root, guard_root)
    profiles = {
        name: read_jsonl(path)
        for name, path in paths.items()
        if path.is_file()
    }
    required = {'train', 'validation', 'canary'}
    if not required <= set(profiles):
        raise VisualV3Error('V3R1 static manifests are incomplete')
    offsets = {
        profile: _offset_detail(rows)
        for profile, rows in profiles.items()
    }
    expected = {
        'train': TRAIN_EXPLICIT_COUNT,
        'validation': VALIDATION_EXPLICIT_COUNT,
        'canary': CANARY_EXPLICIT_COUNT,
    }
    issues = [
        f'{profile}: deliberate count mismatch'
        for profile, count in expected.items()
        if offsets[profile]['deliberate_exact_offset_count'] != count
    ]
    family_sets = {
        profile: {
            row['configuration_family_id'] for row in rows
        }
        for profile, rows in profiles.items()
    }
    family_overlap = {}
    for first, second in (
        ('train', 'validation'),
        ('train', 'canary'),
        ('validation', 'canary'),
    ):
        overlap = sorted(family_sets[first] & family_sets[second])
        family_overlap[f'{first}_vs_{second}'] = overlap
        if overlap:
            issues.append(
                f'{first}/{second}: {len(overlap)} family overlaps'
            )
    report = {
        'schema_version': PACKAGE_SCHEMA,
        'scenario_counts': {
            name: len(rows) for name, rows in profiles.items()
        },
        'offset_coverage': offsets,
        'configuration_family_overlap': family_overlap,
        'issues': issues,
        'legacy_test_accessed': False,
        'passed': not issues,
    }
    atomic_json(guard_root / 'v3r1_static_audit.json', report)
    return report


def smoke_audit(
    *,
    capture_root: Path,
    canary_root: Path,
    guard_root: Path,
) -> dict[str, Any]:
    static = static_audit(
        capture_root=capture_root,
        canary_root=canary_root,
        guard_root=guard_root,
    )
    smoke_root = guard_root / 'smoke'
    manifest = read_jsonl(smoke_root / 'scenario_manifest.jsonl')
    rows = read_jsonl(smoke_root / 'finalized' / 'smoke.jsonl')
    labels = read_jsonl(
        smoke_root / 'finalized' / 'smoke_visual_labels.jsonl'
    )
    finalization = json.loads(
        (
            smoke_root / 'finalized' / 'smoke_finalization.json'
        ).read_text(encoding='utf-8')
    )
    if len(rows) != len(labels):
        raise VisualV3Error('V3R1 smoke rows/labels mismatch')
    scenario_by_id = {row['scenario_id']: row for row in manifest}
    payload_issues = []
    for row, label in zip(rows, labels):
        trace = row['traceability_metadata']
        scenario = scenario_by_id[trace['scenario_id']]
        normalized = normalize_visual_state_labels(label)
        shuttle = next(
            item for item in normalized['shuttles']
            if item['id'] == 'R4'
        )
        if (
            not shuttle['presence']
            or shuttle['loaded_state']
            != scenario['payload_assignment']['R4']
            or shuttle['location']['block'] != 'A34E'
            or abs(
                float(shuttle['rail_position']['s_ratio'])
                - (
                    OPERATIONAL_TARGET_RATIO
                    + float(scenario['target_offset'])
                )
            ) > 1e-5
        ):
            payload_issues.append(trace['scenario_id'])
    pairs: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for scenario in manifest:
        pairs[str(scenario['matched_pair_id'])].append(scenario)
    complete_pairs = [
        pair for pair in pairs.values()
        if (
            len(pair) == 2
            and pair[0]['geometry_fingerprint']
            == pair[1]['geometry_fingerprint']
            and pair[0]['render_variation'] == pair[1]['render_variation']
            and {
                row['payload_assignment']['R4'] for row in pair
            } == {'empty', 'loaded'}
        )
    ]
    image_hashes = finalization['image_hashes']
    image_visible_pairs = sum(
        image_hashes[
            f'{pair[0]["scenario_id"]}:right_rail_rgb'
        ]
        != image_hashes[
            f'{pair[1]["scenario_id"]}:right_rail_rgb'
        ]
        for pair in complete_pairs
    )
    smoke_family = {
        row['configuration_family_id'] for row in manifest
    }
    overlap = {}
    for profile, path in _manifest_paths(
        capture_root, canary_root, guard_root
    ).items():
        if profile == 'smoke' or not path.is_file():
            continue
        profile_families = {
            row['configuration_family_id'] for row in read_jsonl(path)
        }
        overlap[profile] = sorted(smoke_family & profile_families)
    offset = _offset_detail(manifest)
    requirements = {
        'exact_count': len(manifest) == SMOKE_COUNT == len(rows),
        'paired_images': finalization.get('image_count') == SMOKE_COUNT * 2,
        'all_offsets': len(offset['by_offset']) == 9 and all(
            detail['count'] == 4
            for detail in offset['by_offset'].values()
        ),
        'loaded_empty_each_offset': all(
            set(detail['payload']) == {'empty', 'loaded'}
            for detail in offset['by_offset'].values()
        ),
        'sparse_dense_each_offset': all(
            set(detail['presence_class']) == {'sparse', 'dense'}
            for detail in offset['by_offset'].values()
        ),
        'relation_diversity': len(offset['relation_family']) >= 4,
        'unique_families': len(smoke_family) == SMOKE_COUNT,
        'matched_pairs': len(complete_pairs) == 18,
        'payload_visible_in_images': image_visible_pairs == 18,
        'payload_labels': not payload_issues,
        'no_physical_conflicts': not any(
            scenario_physical_conflicts(row) for row in manifest
        ),
        'no_profile_overlap': not any(overlap.values()),
        'static_audit': static['passed'],
        'no_legacy_test_access': True,
    }
    report = {
        'schema_version': PACKAGE_SCHEMA,
        'scenario_count': len(rows),
        'image_count': finalization.get('image_count'),
        'offset_coverage': offset,
        'complete_matched_pair_count': len(complete_pairs),
        'image_visible_matched_pair_count': image_visible_pairs,
        'payload_label_issues': payload_issues,
        'family_overlap': overlap,
        'requirements': requirements,
        'legacy_test_accessed': False,
        'passed': all(requirements.values()),
    }
    atomic_json(
        guard_root / 'v3r1_position_smoke_report.json',
        report,
    )
    atomic_text(
        guard_root / 'v3r1_position_smoke_report.md',
        '\n'.join([
            '# Room 315 V3R1 position-correction smoke',
            '',
            f'- Result: **{"PASS" if report["passed"] else "FAIL"}**',
            f'- Scenarios: {len(rows)}',
            f'- Images: {finalization.get("image_count")}',
            f'- Complete payload-matched pairs: {len(complete_pairs)}',
            f'- Image-visible payload pairs: {image_visible_pairs}',
            f'- Relation families: {len(offset["relation_family"])}',
            f'- Profile family overlap: {sum(map(len, overlap.values()))}',
            '',
        ]),
    )
    return report


def full_audit(
    *,
    capture_root: Path,
    split_root: Path,
    canary_root: Path,
    guard_root: Path,
) -> dict[str, Any]:
    static = static_audit(
        capture_root=capture_root,
        canary_root=canary_root,
        guard_root=guard_root,
    )
    base = run_v3_full_audit(
        split_root=split_root,
        capture_root=capture_root,
        canary_root=canary_root,
        guard_root=guard_root,
        quota_plan_path=quota_plan_path(guard_root),
        experiment_config_path=(
            Path(__file__).resolve().parents[2]
            / 'config'
            / 'room_315_vla'
            / 'visual_state_experiment_a_dataset_v3r1.yaml'
        ),
        output_package_schema=PACKAGE_SCHEMA,
        command_revision='v3r1',
    )
    report = {
        'schema_version': PACKAGE_SCHEMA,
        'static': static,
        'captured_dataset_audit': base,
        'legacy_test_accessed': False,
        'passed': static['passed'] and base['passed'],
    }
    atomic_json(guard_root / 'dataset_v3r1_audit.json', report)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--mode',
        choices=('static', 'smoke', 'full'),
        required=True,
    )
    parser.add_argument('--capture-root', type=Path, default=DEFAULT_CAPTURE_ROOT)
    parser.add_argument('--split-root', type=Path, default=DEFAULT_SPLIT_ROOT)
    parser.add_argument('--canary-root', type=Path, default=DEFAULT_CANARY_ROOT)
    parser.add_argument('--guard-root', type=Path, default=DEFAULT_GUARD_ROOT)
    args = parser.parse_args(argv)
    if args.mode == 'static':
        result = static_audit(
            capture_root=args.capture_root,
            canary_root=args.canary_root,
            guard_root=args.guard_root,
        )
    elif args.mode == 'smoke':
        result = smoke_audit(
            capture_root=args.capture_root,
            canary_root=args.canary_root,
            guard_root=args.guard_root,
        )
    else:
        result = full_audit(
            capture_root=args.capture_root,
            split_root=args.split_root,
            canary_root=args.canary_root,
            guard_root=args.guard_root,
        )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, VisualV3Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
