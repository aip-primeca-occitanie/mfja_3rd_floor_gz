#!/usr/bin/env python3
"""Build immutable corrected Room 315 visual V3R1 manifests."""

from __future__ import annotations

import argparse
import copy
import json
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_scenario_generator import validate_scenario
from room_315_visual_scenario_generator import scenario_physical_conflicts
from room_315_visual_v3_common import BLOCKS
from room_315_visual_v3_common import DEFAULT_OLD_TRAIN
from room_315_visual_v3_common import DEFAULT_OLD_TRAIN_LABELS
from room_315_visual_v3_common import IDENTITIES
from room_315_visual_v3_common import POSITION_BINS
from room_315_visual_v3_common import SEED
from room_315_visual_v3_common import TARGET_OFFSETS
from room_315_visual_v3_common import VisualV3Error
from room_315_visual_v3_common import atomic_json
from room_315_visual_v3_common import atomic_jsonl
from room_315_visual_v3_common import prepare_output_root
from room_315_visual_v3_common import read_jsonl
from room_315_visual_v3_common import sha256_file
from room_315_visual_v3_common import target_offset_bucket
from room_315_visual_v3_common import value_sha256
from room_315_visual_v3_generator import _repository_state
from room_315_visual_v3_generator import _write_manifest_lock
from room_315_visual_v3_generator import materialize_specs
from room_315_visual_v3_generator import materialize_specs_avoiding
from room_315_visual_v3_generator import static_manifest_audit as v3_static_audit
from room_315_visual_v3_splitter import _old_replay_core_families
from room_315_visual_v3r1_common import CANARY_COUNT
from room_315_visual_v3r1_common import CANARY_EXPLICIT_COUNT
from room_315_visual_v3r1_common import DEFAULT_CANARY_ROOT
from room_315_visual_v3r1_common import DEFAULT_CAPTURE_ROOT
from room_315_visual_v3r1_common import DEFAULT_GUARD_ROOT
from room_315_visual_v3r1_common import DEFAULT_V3_CAPTURE_ROOT
from room_315_visual_v3r1_common import DEFAULT_V3_GUARD_ROOT
from room_315_visual_v3r1_common import GENERATOR_VERSION
from room_315_visual_v3r1_common import MANIFEST_REVISION
from room_315_visual_v3r1_common import OPERATIONAL_TARGET_IDENTITY
from room_315_visual_v3r1_common import OPERATIONAL_TARGET_NAME
from room_315_visual_v3r1_common import OPERATIONAL_TARGET_RATIO
from room_315_visual_v3r1_common import OPERATIONAL_TARGET_SEGMENT
from room_315_visual_v3r1_common import PACKAGE_SCHEMA
from room_315_visual_v3r1_common import SMOKE_COUNT
from room_315_visual_v3r1_common import TRAIN_COUNT
from room_315_visual_v3r1_common import TRAIN_EXPLICIT_COUNT
from room_315_visual_v3r1_common import VALIDATION_COUNT
from room_315_visual_v3r1_common import VALIDATION_EXPLICIT_COUNT
from room_315_visual_v3r1_common import is_deliberate_offset
from room_315_visual_v3r1_common import presence_class
from room_315_visual_v3r1_common import v3r1_family_id
from room_315_visual_v3r1_common import quota_plan_path
from room_315_visual_v3r1_quota_planner import explicit_specs
from room_315_visual_v3r1_quota_planner import generic_specs
from room_315_visual_v3r1_quota_planner import quota_plan
from room_315_visual_v3r1_quota_planner import smoke_specs
from room_315_visual_v3r1_reuse import import_reusable
from room_315_visual_v3r1_reuse import scan_reusable


def _configuration(mode: str) -> dict[str, Any]:
    return {
        'schema_version': PACKAGE_SCHEMA,
        'generator_version': GENERATOR_VERSION,
        'visual_schema': 'room315.visual_state.v3',
        'scenario_schema': 'room315.visual_capture_scenario.v1',
        'seed': SEED,
        'manifest_revision': MANIFEST_REVISION,
        'mode': mode,
        'fixed_identity_order': list(IDENTITIES),
        'public_blocks': list(BLOCKS),
        'operational_target': {
            'name': OPERATIONAL_TARGET_NAME,
            'identity': OPERATIONAL_TARGET_IDENTITY,
            'segment': OPERATIONAL_TARGET_SEGMENT,
            'ratio': OPERATIONAL_TARGET_RATIO,
            'offsets': list(TARGET_OFFSETS),
        },
        'capture': {
            'cameras': ['left_rail_rgb', 'right_rail_rgb'],
            'frames': 1,
            'settle_seconds': 1.5,
            'frame_interval_seconds': 0.25,
        },
    }


def _revisionize(
    row: dict[str, Any],
    spec: dict[str, Any],
) -> dict[str, Any]:
    result = copy.deepcopy(row)
    result.update({
        'manifest_profile_schema': PACKAGE_SCHEMA,
        'generator_version': GENERATOR_VERSION,
        'scenario_id': (
            f'hard_v3r1_{spec["profile"]}_'
            f'{int(spec["generation_index"]) + 1:04d}_'
            f'{row["geometry_fingerprint"][:12]}'
        ),
        'scenario_family': (
            'hard_v3r1_capture_'
            + value_sha256({
                'capture_configuration_fingerprint': row[
                    'capture_configuration_fingerprint'
                ],
                'spec_id': spec['spec_id'],
                'revision': MANIFEST_REVISION,
            })
        ),
        'source_profile': spec.get('source_profile', spec['profile']),
        'imported_from_v3': False,
        'source_scenario_id': None,
        'source_manifest_sha256': None,
        'v3r1_manifest_revision': MANIFEST_REVISION,
        'target_ratio': spec.get('target_ratio'),
        'operational_target_name': spec.get('operational_target_name'),
        'operational_target_segment': spec.get(
            'operational_target_segment'
        ),
        'presence_class': spec.get('presence_class'),
        'configuration_variant': spec.get('configuration_variant'),
    })
    result['configuration_family_id'] = v3r1_family_id(result)
    result['configuration_core_family_id'] = v3r1_family_id(
        result,
        include_render=False,
    )
    return result


def _revisionize_many(
    specs: list[dict[str, Any]],
    rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    if len(specs) != len(rows):
        raise VisualV3Error('V3R1 spec/scenario count mismatch')
    return [_revisionize(row, spec) for spec, row in zip(specs, rows)]


def _imported_scenarios(
    scan: dict[str, Any],
    *,
    source_capture_root: Path,
) -> list[dict[str, Any]]:
    reusable = {row['scenario_id'] for row in scan['reusable']}
    source_manifest_path = (
        source_capture_root / 'manifests' / 'train_scenarios.jsonl'
    )
    source_hash = sha256_file(source_manifest_path)
    rows = []
    for source in read_jsonl(source_manifest_path):
        if source['scenario_id'] not in reusable:
            continue
        row = copy.deepcopy(source)
        row.update({
            'manifest_profile_schema': PACKAGE_SCHEMA,
            'source_profile': 'train',
            'imported_from_v3': True,
            'source_scenario_id': source['scenario_id'],
            'source_manifest_sha256': source_hash,
            'v3r1_manifest_revision': MANIFEST_REVISION,
            'target_ratio': None,
            'operational_target_name': None,
            'operational_target_segment': None,
            'presence_class': presence_class(
                len(source['active_identities'])
            ),
            'configuration_variant': None,
        })
        rows.append(row)
    if len(rows) != scan['reusable_scenario_count']:
        raise VisualV3Error('V3R1 imported manifest count mismatch')
    return rows


def _manifest_integrity(rows: list[dict[str, Any]]) -> None:
    checks = {
        'scenario_id': [row['scenario_id'] for row in rows],
        'scenario_family': [row['scenario_family'] for row in rows],
        'capture_configuration_fingerprint': [
            row['capture_configuration_fingerprint'] for row in rows
        ],
    }
    for name, values in checks.items():
        if len(values) != len(set(values)):
            duplicates = [
                value for value, count in Counter(values).items()
                if count > 1
            ]
            raise VisualV3Error(
                f'V3R1 duplicate {name}: {duplicates[:3]}'
            )


def _deliberate_table(rows: list[dict[str, Any]]) -> dict[str, Any]:
    deliberate = [row for row in rows if is_deliberate_offset(row)]
    by_offset = Counter()
    by_payload = Counter()
    by_presence = Counter()
    by_relation = Counter()
    by_cell = Counter()
    for row in deliberate:
        bucket = row['target_offset_bucket']
        payload = row['payload_assignment']['R4']
        presence = row['presence_class']
        relation = row['relation_family']
        by_offset[bucket] += 1
        by_payload[payload] += 1
        by_presence[presence] += 1
        by_relation[relation] += 1
        by_cell[(bucket, payload, presence, relation)] += 1
    return {
        'total': len(deliberate),
        'by_offset': dict(sorted(by_offset.items())),
        'by_payload': dict(sorted(by_payload.items())),
        'by_presence_class': dict(sorted(by_presence.items())),
        'by_relation_family': dict(sorted(by_relation.items())),
        'configuration_family_count': len({
            row['configuration_family_id'] for row in deliberate
        }),
        'cell_counts': {
            '|'.join(key): value
            for key, value in sorted(by_cell.items())
        },
    }


def static_manifest_audit(
    profile: str,
    rows: list[dict[str, Any]],
) -> dict[str, Any]:
    base = v3_static_audit(rows)
    errors = list(base.get('errors') or [])
    deliberate = _deliberate_table(rows)
    expected_total = {
        'train': TRAIN_EXPLICIT_COUNT,
        'validation': VALIDATION_EXPLICIT_COUNT,
        'canary': CANARY_EXPLICIT_COUNT,
        'smoke': SMOKE_COUNT,
    }[profile]
    if deliberate['total'] != expected_total:
        errors.append(
            f'deliberate offset count {deliberate["total"]} '
            f'!= {expected_total}'
        )
    expected_buckets = {target_offset_bucket(value) for value in TARGET_OFFSETS}
    if set(deliberate['by_offset']) != expected_buckets:
        errors.append('not all nine operational offset buckets are present')
    if set(deliberate['by_payload']) != {'empty', 'loaded'}:
        errors.append('R4 loaded/empty offset balance is incomplete')
    expected_presence = {
        'train': {'sparse', 'medium', 'dense'},
        'validation': {'sparse', 'medium', 'dense'},
        'canary': {'sparse', 'dense'},
        'smoke': {'sparse', 'dense'},
    }[profile]
    if set(deliberate['by_presence_class']) != expected_presence:
        errors.append('required offset presence classes are incomplete')
    for row in rows:
        try:
            validate_scenario(row)
        except ValueError as exc:
            errors.append(f'{row.get("scenario_id")}: {exc}')
        if scenario_physical_conflicts(row):
            errors.append(f'{row.get("scenario_id")}: physical conflict')
        if is_deliberate_offset(row):
            expected_ratio = (
                OPERATIONAL_TARGET_RATIO + float(row['target_offset'])
            )
            if abs(
                float(row['identity_to_s_ratio']['R4']) - expected_ratio
            ) > 1e-8:
                errors.append(
                    f'{row["scenario_id"]}: exact target ratio mismatch'
                )
    if profile == 'train':
        cells = Counter()
        for row in rows:
            loaded = set(row['loaded_identities'])
            for identity in row['active_identities']:
                cells[(
                    identity,
                    'loaded' if identity in loaded else 'empty',
                    row['identity_to_block'][identity],
                    row['identity_to_position_bin'][identity],
                )] += 1
        valid_cells = [
            (identity, state, block, position)
            for identity in IDENTITIES
            for state in ('empty', 'loaded')
            for block in BLOCKS
            for position in POSITION_BINS
        ]
        missing = [cell for cell in valid_cells if not cells[cell]]
        if missing:
            errors.append(
                f'{len(missing)} original conditional cells are empty'
            )
    return {
        **base,
        'schema_version': PACKAGE_SCHEMA,
        'profile': profile,
        'deliberate_exact_offset_coverage': deliberate,
        'errors': errors,
        'passed': not errors,
    }


def _write_profile(
    profile: str,
    specs: list[dict[str, Any]],
    rows: list[dict[str, Any]],
    *,
    root: Path,
    result: dict[str, Any],
) -> None:
    _manifest_integrity(rows)
    manifest_dir = root / 'manifests'
    spec_path = manifest_dir / f'{profile}_specs.jsonl'
    manifest_path = manifest_dir / f'{profile}_scenarios.jsonl'
    atomic_jsonl(spec_path, specs)
    atomic_jsonl(manifest_path, rows)
    audit = static_manifest_audit(profile, rows)
    audit_path = manifest_dir / f'{profile}_static_audit.json'
    atomic_json(audit_path, audit)
    if not audit['passed']:
        raise VisualV3Error(
            f'{profile} static audit failed: {audit["errors"][:3]}'
        )
    result['manifests'][profile] = {
        'scenario_count': len(rows),
        'expected_image_count': len(rows) * 2,
        'spec_path': str(spec_path),
        'spec_sha256': sha256_file(spec_path),
        'scenario_manifest': str(manifest_path),
        'scenario_manifest_sha256': sha256_file(manifest_path),
        'static_audit': str(audit_path),
        'static_audit_sha256': sha256_file(audit_path),
    }


def _family_overlap(
    profiles: dict[str, list[dict[str, Any]]],
) -> dict[str, Any]:
    sets = {
        name: {
            'family': {row['configuration_family_id'] for row in rows},
            'core': {
                row['configuration_core_family_id'] for row in rows
            },
            'capture': {
                row['capture_configuration_fingerprint'] for row in rows
            },
        }
        for name, rows in profiles.items()
    }
    pairs = {}
    violations = []
    for first, second in (
        ('train', 'validation'),
        ('train', 'canary'),
        ('validation', 'canary'),
    ):
        key = f'{first}_vs_{second}'
        pairs[key] = {}
        for field in ('family', 'core', 'capture'):
            overlap = sorted(sets[first][field] & sets[second][field])
            pairs[key][f'{field}_overlap'] = overlap
            if overlap:
                violations.append(f'{key}:{field}:{len(overlap)}')
    return {
        'schema_version': PACKAGE_SCHEMA,
        'pairwise': pairs,
        'violations': violations,
        'passed': not violations,
    }


def prepare_manifests(
    *,
    mode: str,
    capture_root: Path,
    canary_root: Path,
    guard_root: Path,
    source_capture_root: Path,
    source_guard_root: Path,
    resume: bool,
) -> dict[str, Any]:
    if mode not in {'smoke', 'full'}:
        raise VisualV3Error(f'unsupported V3R1 mode: {mode}')
    if mode == 'smoke':
        smoke_root = guard_root / 'smoke'
        config_sha = prepare_output_root(
            smoke_root,
            _configuration('smoke'),
            resume=resume,
        )
        specs = smoke_specs()
        rows = _revisionize_many(specs, materialize_specs(specs))
        result = {
            'mode': 'smoke',
            'configuration_sha256': config_sha,
            'root': str(smoke_root),
            'manifests': {},
        }
        _write_profile('smoke', specs, rows, root=smoke_root, result=result)
        # V3 capture expects this conventional smoke manifest name.
        conventional = smoke_root / 'scenario_manifest.jsonl'
        atomic_jsonl(conventional, rows)
        result['scenario_manifest'] = str(conventional)
        result['scenario_manifest_sha256'] = sha256_file(conventional)
        _write_manifest_lock(
            smoke_root,
            {
                'schema_version': PACKAGE_SCHEMA,
                'seed': SEED,
                'profile': 'smoke',
                'scenario_count': len(rows),
                'scenario_manifest': str(conventional),
                'scenario_manifest_sha256': sha256_file(conventional),
            },
            resume=resume,
        )
        atomic_json(smoke_root / 'package_manifest.json', result)
        return result

    scan = scan_reusable(
        source_capture_root=source_capture_root,
        source_guard_root=source_guard_root,
    )
    if not scan['passed']:
        raise VisualV3Error('historical V3 reuse audit failed')
    reuse_count = int(scan['reusable_scenario_count'])
    plan = quota_plan(reuse_count)
    configuration = {
        **_configuration('full'),
        'v3_source_manifest_sha256': scan['source_manifest_sha256'],
        'v3_reuse_count': reuse_count,
    }
    config_sha = prepare_output_root(
        capture_root,
        configuration,
        resume=resume,
    )
    canary_sha = prepare_output_root(
        canary_root,
        {**configuration, 'mode': 'canary'},
        resume=resume,
    )
    guard_sha = prepare_output_root(
        guard_root,
        {**configuration, 'mode': 'guard'},
        resume=resume,
    )
    atomic_json(
        quota_plan_path(guard_root),
        plan,
    )
    imported = _imported_scenarios(
        scan,
        source_capture_root=source_capture_root,
    )
    train_offset_specs = explicit_specs(
        'train',
        start_index=reuse_count,
    )
    train_offset_v3 = materialize_specs(train_offset_specs)
    train_offsets = _revisionize_many(
        train_offset_specs,
        train_offset_v3,
    )
    train_generic_count = (
        TRAIN_COUNT - reuse_count - TRAIN_EXPLICIT_COUNT
    )
    train_generic_specs = generic_specs(
        'train',
        train_generic_count,
        start_index=reuse_count + TRAIN_EXPLICIT_COUNT,
        source_start=reuse_count,
    )
    train_generic_v3 = materialize_specs(train_generic_specs)
    train_generic = _revisionize_many(
        train_generic_specs,
        train_generic_v3,
    )
    train = imported + train_offsets + train_generic
    if len(train) != TRAIN_COUNT:
        raise VisualV3Error('V3R1 train count mismatch')

    train_v3_core = {
        row['configuration_core_family_id']
        for row in imported + train_offset_v3 + train_generic_v3
    }
    train_capture = {
        row['capture_configuration_fingerprint'] for row in train
    }
    old_replay_core = _old_replay_core_families(
        DEFAULT_OLD_TRAIN,
        DEFAULT_OLD_TRAIN_LABELS,
    )
    validation_offset_specs = explicit_specs(
        'validation',
        start_index=0,
    )
    validation_offset_v3 = materialize_specs(validation_offset_specs)
    validation_offsets = _revisionize_many(
        validation_offset_specs,
        validation_offset_v3,
    )
    validation_generic_specs = generic_specs(
        'validation',
        VALIDATION_COUNT - VALIDATION_EXPLICIT_COUNT,
        start_index=VALIDATION_EXPLICIT_COUNT,
    )
    (
        validation_generic_specs,
        validation_generic_v3,
    ) = materialize_specs_avoiding(
        validation_generic_specs,
        forbidden_core_families=train_v3_core | old_replay_core,
        forbidden_capture_configurations=train_capture | {
            row['capture_configuration_fingerprint']
            for row in validation_offset_v3
        },
    )
    validation_generic = _revisionize_many(
        validation_generic_specs,
        validation_generic_v3,
    )
    validation = validation_offsets + validation_generic
    if len(validation) != VALIDATION_COUNT:
        raise VisualV3Error('V3R1 validation count mismatch')

    canary_offset_specs = explicit_specs('canary', start_index=0)
    canary_offset_v3 = materialize_specs(canary_offset_specs)
    canary_offsets = _revisionize_many(
        canary_offset_specs,
        canary_offset_v3,
    )
    canary_generic_specs = generic_specs(
        'canary',
        CANARY_COUNT - CANARY_EXPLICIT_COUNT,
        start_index=CANARY_EXPLICIT_COUNT,
    )
    canary_forbidden_v3_core = (
        train_v3_core
        | {
            row['configuration_core_family_id']
            for row in validation_offset_v3 + validation_generic_v3
        }
    )
    canary_forbidden_capture = {
        row['capture_configuration_fingerprint']
        for row in train + validation + canary_offset_v3
    }
    (
        canary_generic_specs,
        canary_generic_v3,
    ) = materialize_specs_avoiding(
        canary_generic_specs,
        forbidden_core_families=canary_forbidden_v3_core,
        forbidden_capture_configurations=canary_forbidden_capture,
    )
    canary_generic = _revisionize_many(
        canary_generic_specs,
        canary_generic_v3,
    )
    canary = canary_offsets + canary_generic
    if len(canary) != CANARY_COUNT:
        raise VisualV3Error('V3R1 canary count mismatch')

    overlap = _family_overlap({
        'train': train,
        'validation': validation,
        'canary': canary,
    })
    if not overlap['passed']:
        raise VisualV3Error(
            f'V3R1 static family overlap: {overlap["violations"]}'
        )
    result: dict[str, Any] = {
        'schema_version': PACKAGE_SCHEMA,
        'mode': 'full',
        'configuration_sha256': config_sha,
        'canary_configuration_sha256': canary_sha,
        'guard_configuration_sha256': guard_sha,
        'quota_plan_sha256': plan['quota_plan_sha256'],
        'v3_source_manifest_sha256': scan['source_manifest_sha256'],
        'v3_reuse_count': reuse_count,
        'repository': _repository_state(),
        'manifests': {},
    }
    imported_specs = [
        {
            'profile': 'train',
            'generation_index': row['generation_index'],
            'spec_id': row['spec_id'],
            'imported_from_v3': True,
            'source_scenario_id': row['scenario_id'],
            'source_manifest_sha256': scan['source_manifest_sha256'],
            'v3r1_manifest_revision': MANIFEST_REVISION,
        }
        for row in imported
    ]
    _write_profile(
        'train',
        imported_specs + train_offset_specs + train_generic_specs,
        train,
        root=capture_root,
        result=result,
    )
    _write_profile(
        'validation',
        validation_offset_specs + validation_generic_specs,
        validation,
        root=capture_root,
        result=result,
    )
    _write_profile(
        'canary',
        canary_offset_specs + canary_generic_specs,
        canary,
        root=canary_root,
        result=result,
    )
    atomic_json(
        guard_root / 'static_family_overlap_audit.json',
        overlap,
    )
    _write_manifest_lock(
        capture_root,
        {
            'schema_version': PACKAGE_SCHEMA,
            'seed': SEED,
            'v3_source_manifest_sha256': scan[
                'source_manifest_sha256'
            ],
            'profiles': {
                name: result['manifests'][name]
                for name in ('train', 'validation')
            },
        },
        resume=resume,
    )
    _write_manifest_lock(
        canary_root,
        {
            'schema_version': PACKAGE_SCHEMA,
            'seed': SEED,
            'profiles': {
                'canary': result['manifests']['canary']
            },
        },
        resume=resume,
    )
    reuse_report = import_reusable(
        scan,
        source_capture_root=source_capture_root,
        destination_capture_root=capture_root,
        guard_root=guard_root,
    )
    if not reuse_report['passed']:
        raise VisualV3Error('V3 to V3R1 import audit failed')
    result['reuse_audit'] = str(
        guard_root / 'v3_to_v3r1_reuse_audit.json'
    )
    result['reuse_audit_sha256'] = sha256_file(
        guard_root / 'v3_to_v3r1_reuse_audit.json'
    )
    atomic_json(guard_root / 'static_package_manifest.json', result)
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('smoke', 'full'), required=True)
    parser.add_argument('--capture-root', type=Path, default=DEFAULT_CAPTURE_ROOT)
    parser.add_argument('--canary-root', type=Path, default=DEFAULT_CANARY_ROOT)
    parser.add_argument('--guard-root', type=Path, default=DEFAULT_GUARD_ROOT)
    parser.add_argument(
        '--source-capture-root',
        type=Path,
        default=DEFAULT_V3_CAPTURE_ROOT,
    )
    parser.add_argument(
        '--source-guard-root',
        type=Path,
        default=DEFAULT_V3_GUARD_ROOT,
    )
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args(argv)
    result = prepare_manifests(
        mode=args.mode,
        capture_root=args.capture_root,
        canary_root=args.canary_root,
        guard_root=args.guard_root,
        source_capture_root=args.source_capture_root,
        source_guard_root=args.source_guard_root,
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
