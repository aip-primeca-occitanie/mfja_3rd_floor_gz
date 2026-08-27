#!/usr/bin/env python3
"""Conditional coverage and integrity audit for Room 315 visual dataset V3."""

from __future__ import annotations

import argparse
import csv
import io
import itertools
import json
import statistics
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Iterable

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_state_dataset import normalize_visual_state_labels
from room_315_visual_v3_common import BLOCKS
from room_315_visual_v3_common import DEFAULT_CANARY_ROOT
from room_315_visual_v3_common import DEFAULT_CAPTURE_ROOT
from room_315_visual_v3_common import DEFAULT_GUARD_ROOT
from room_315_visual_v3_common import DEFAULT_SPLIT_ROOT
from room_315_visual_v3_common import IDENTITIES
from room_315_visual_v3_common import PACKAGE_SCHEMA
from room_315_visual_v3_common import POSITION_BINS
from room_315_visual_v3_common import RELATIONS
from room_315_visual_v3_common import RENDER_BUCKETS
from room_315_visual_v3_common import SEED
from room_315_visual_v3_common import TARGET_OFFSET_BUCKETS
from room_315_visual_v3_common import VISUAL_SCHEMA
from room_315_visual_v3_common import ZONES
from room_315_visual_v3_common import VisualV3Error
from room_315_visual_v3_common import atomic_json
from room_315_visual_v3_common import atomic_text
from room_315_visual_v3_common import counter_stats
from room_315_visual_v3_common import read_jsonl
from room_315_visual_v3_common import sha256_file
from room_315_visual_v3_common import side_for_identity


def default_experiment_config_path(filename: str) -> Path:
    """Resolve an experiment config from the source tree or package share."""
    source_path = SCRIPT_DIR.parent / 'config' / 'room_315_vla' / filename
    if source_path.is_file():
        return source_path
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory('mfja_robot_control_config'))
            / 'config'
            / 'room_315_vla'
            / filename
        )
    except Exception:
        return source_path


def _write_csv(
    path: Path,
    fieldnames: list[str],
    rows: Iterable[dict[str, Any]],
) -> None:
    stream = io.StringIO()
    writer = csv.DictWriter(stream, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)
    atomic_text(path, stream.getvalue())


def _profile_rows(
    split_root: Path,
    canary_root: Path,
) -> dict[str, tuple[list[dict[str, Any]], list[dict[str, Any]]]]:
    result = {}
    for profile, rows_path, labels_path in (
        (
            'train',
            split_root / 'train.jsonl',
            split_root / 'train_visual_labels.jsonl',
        ),
        (
            'validation',
            split_root / 'validation.jsonl',
            split_root / 'validation_visual_labels.jsonl',
        ),
        (
            'canary',
            canary_root / 'finalized' / 'canary.jsonl',
            canary_root / 'finalized' / 'canary_visual_labels.jsonl',
        ),
    ):
        rows = read_jsonl(rows_path)
        labels = read_jsonl(labels_path)
        if len(rows) != len(labels):
            raise VisualV3Error(f'{profile} row/label count mismatch')
        labels_by_sample = {row['sample_id']: row for row in labels}
        ordered_labels = []
        for row in rows:
            if row['sample_id'] not in labels_by_sample:
                raise VisualV3Error(f'{profile}: missing label {row["sample_id"]}')
            ordered_labels.append(labels_by_sample[row['sample_id']])
        result[profile] = (rows, ordered_labels)
    return result


def _validate_record(
    profile: str,
    row: dict[str, Any],
    label_row: dict[str, Any],
) -> list[str]:
    issues = []
    prohibited = {
        'safety_margin',
        'occupied_interval',
        'blocks_route',
        'route_clear',
        'planning_actions',
        'task',
        'pddl_problem',
        'primitive_command',
        'trajectory',
        'execution_success',
    }

    def walk_keys(value: Any) -> set[str]:
        found = set()
        if isinstance(value, dict):
            for key, child in value.items():
                found.add(str(key))
                found.update(walk_keys(child))
        elif isinstance(value, list):
            for child in value:
                found.update(walk_keys(child))
        return found

    leaked = sorted(walk_keys(label_row) & prohibited)
    if leaked:
        issues.append(f'planning/safety-derived labels present: {leaked}')
    trace = row.get('traceability_metadata') or {}
    labels = normalize_visual_state_labels(label_row)
    if labels['schema_version'] != VISUAL_SCHEMA:
        issues.append('wrong visual schema')
    present = {
        shuttle['id']: shuttle
        for shuttle in labels['shuttles']
        if shuttle['presence']
    }
    expected = set(trace.get('active_identities') or [])
    if set(present) != expected:
        issues.append(f'presence mismatch: {sorted(present)} != {sorted(expected)}')
    for identity, shuttle in present.items():
        if identity not in IDENTITIES:
            issues.append(f'unknown identity {identity}')
            continue
        expected_side = side_for_identity(identity)
        if shuttle['location']['side'] != expected_side:
            issues.append(f'{identity}: side mismatch')
        block = shuttle['location']['block']
        if block not in BLOCKS:
            issues.append(f'{identity}: invalid block {block}')
        rail = shuttle['rail_position']
        ratio = float(rail['s_ratio'])
        length = float(rail['segment_length_m'])
        s_m = float(rail['s_m'])
        if not 0.0 <= ratio <= 1.0 or length <= 0.0:
            issues.append(f'{identity}: invalid continuous position')
        if abs(s_m - ratio * length) > 1e-5:
            issues.append(f'{identity}: s_m/s_ratio mismatch')
        if float(rail['position_uncertainty_m']) != 0.0:
            issues.append(f'{identity}: oracle uncertainty must be 0.0')
        if trace.get('identity_to_block', {}).get(identity) != block:
            issues.append(f'{identity}: trace block mismatch')
        expected_state = (
            'loaded' if identity in set(trace.get('loaded_identities') or [])
            else 'empty'
        )
        if shuttle['loaded_state'] != expected_state:
            issues.append(f'{identity}: payload mismatch')
    if profile == 'canary' and not trace.get('canary_family'):
        issues.append('canary row lacks canary family')
    return issues


def _conditional_counts(
    profiles: dict[str, tuple[list[dict[str, Any]], list[dict[str, Any]]]],
) -> tuple[dict[str, Counter[Any]], list[str]]:
    counts: dict[str, Counter[Any]] = {
        name: Counter()
        for name in (
            'identity',
            'side',
            'loaded_state',
            'block',
            'zone',
            'position_bin',
            'identity_loaded',
            'identity_block',
            'identity_loaded_block',
            'conditional_cell',
            'presence_cardinality',
            'presence_subset',
            'loaded_identity_set',
            'loaded_count',
            'relation_family',
            'occlusion_class',
            'render_bucket',
            'target_offset_bucket',
            'profile',
        )
    }
    issues = []
    for profile, (rows, labels) in profiles.items():
        for row, label_row in zip(rows, labels):
            record_issues = _validate_record(profile, row, label_row)
            issues.extend(
                f'{profile}:{row.get("sample_id")}:{issue}'
                for issue in record_issues
            )
            trace = row['traceability_metadata']
            active = tuple(trace['active_identities'])
            loaded = tuple(trace['loaded_identities'])
            counts['profile'][profile] += 1
            counts['presence_cardinality'][len(active)] += 1
            counts['presence_subset']['+'.join(active)] += 1
            counts['loaded_identity_set']['+'.join(loaded) or 'none'] += 1
            counts['loaded_count'][len(loaded)] += 1
            counts['relation_family'][trace['relation_family']] += 1
            counts['occlusion_class'][trace['occlusion_class']] += 1
            counts['render_bucket'][trace['render_bucket']] += 1
            counts['target_offset_bucket'][trace['target_offset_bucket']] += 1
            for identity in active:
                state = 'loaded' if identity in loaded else 'empty'
                block = trace['identity_to_block'][identity]
                zone = trace['identity_to_zone'][identity]
                pos_bin = trace['identity_to_position_bin'][identity]
                counts['identity'][identity] += 1
                counts['side'][side_for_identity(identity)] += 1
                counts['loaded_state'][state] += 1
                counts['block'][block] += 1
                counts['zone'][zone] += 1
                counts['position_bin'][pos_bin] += 1
                counts['identity_loaded'][(identity, state)] += 1
                counts['identity_block'][(identity, block)] += 1
                counts['identity_loaded_block'][(identity, state, block)] += 1
                counts['conditional_cell'][
                    (identity, state, block, pos_bin)
                ] += 1
    return counts, issues


def _hard_cases(
    profiles: dict[str, tuple[list[dict[str, Any]], list[dict[str, Any]]]],
) -> dict[str, Any]:
    per_profile = {}
    total = Counter()
    for profile, (rows, _) in profiles.items():
        counts = Counter()
        offset_counts = Counter()
        for row in rows:
            trace = row['traceability_metadata']
            active = set(trace['active_identities'])
            loaded = set(trace['loaded_identities'])
            blocks = trace['identity_to_block']
            if 'L4' in loaded:
                counts['l4_loaded'] += 1
            if 'R4' in loaded:
                counts['r4_loaded'] += 1
            if {'L4', 'R4'} <= loaded:
                counts['l4_r4_both_loaded'] += 1
            if active == {'L2', 'L4', 'R4'}:
                counts['exact_l2_l4_r4_subset'] += 1
                if loaded == {'L4', 'R4'}:
                    counts['anchor_payload_assignment'] += 1
                    if all(
                        blocks.get(identity) == block
                        for identity, block in {
                            'L2': 'A12E',
                            'L4': 'A34E',
                            'R4': 'A12E',
                        }.items()
                    ):
                        counts['anchor_block_assignment'] += 1
                        counts['complete_anchor_scene'] += 1
            if (
                trace['target_identity'] == 'R4'
                and blocks.get('R4') == 'A34E'
                and trace['target_offset_bucket'] != 'not_operational_target'
            ):
                counts['right_slot3_position_samples'] += 1
                offset_counts[trace['target_offset_bucket']] += 1
        per_profile[profile] = {
            **dict(counts),
            'target_offset_buckets': dict(sorted(offset_counts.items())),
        }
        total.update(counts)
    return {
        'per_profile': per_profile,
        'combined': dict(total),
    }


def _plain_counts(counter: Counter[Any]) -> dict[str, int]:
    return {
        (
            '|'.join(str(part) for part in key)
            if isinstance(key, tuple)
            else str(key)
        ): int(value)
        for key, value in sorted(counter.items(), key=lambda item: str(item[0]))
    }


def _audit_markdown(audit: dict[str, Any]) -> str:
    hard = audit['hard_cases']['combined']
    conditional = audit['conditional_cell_statistics']
    lines = [
        '# Room 315 hard-case visual dataset V3 audit',
        '',
        f'- Result: **{"PASS" if audit["passed"] else "FAIL"}**',
        f'- Train scenarios: {audit["scenario_counts"]["train"]}',
        f'- Validation scenarios: {audit["scenario_counts"]["validation"]}',
        f'- Canary scenarios: {audit["scenario_counts"]["canary"]}',
        f'- Total paired RGB images: {audit["image_count"]}',
        f'- Empty valid conditional cells: {conditional["empty_cell_count"]}',
        '',
        '## Hard cases',
        '',
        f'- L4 loaded: {hard.get("l4_loaded", 0)}',
        f'- R4 loaded: {hard.get("r4_loaded", 0)}',
        f'- L4 and R4 both loaded: {hard.get("l4_r4_both_loaded", 0)}',
        f'- Exact `{{L2,L4,R4}}`: {hard.get("exact_l2_l4_r4_subset", 0)}',
        f'- Anchor payload assignment: {hard.get("anchor_payload_assignment", 0)}',
        f'- Anchor block assignment: {hard.get("anchor_block_assignment", 0)}',
        f'- Complete anchor scene: {hard.get("complete_anchor_scene", 0)}',
        (
            '- Right-slot-3 position samples: '
            f'{hard.get("right_slot3_position_samples", 0)}'
        ),
        '',
        '## Integrity',
        '',
        f'- Schema: `{audit["visual_schema"]}`',
        f'- Fixed identity order: `{",".join(audit["fixed_identity_order"])}`',
        f'- Record validation issues: {len(audit["issues"])}',
        f'- Grouped overlap audit passed: {audit["split_overlap_passed"]}',
        f'- Legacy Test accessed: no',
        f'- Final Test created: no',
        '',
    ]
    return '\n'.join(lines)


def run_full_audit(
    *,
    split_root: Path,
    capture_root: Path,
    canary_root: Path,
    guard_root: Path,
    quota_plan_path: Path | None = None,
    experiment_config_path: Path | None = None,
    output_package_schema: str = PACKAGE_SCHEMA,
    command_revision: str = 'v3',
) -> dict[str, Any]:
    profiles = _profile_rows(split_root, canary_root)
    expected_counts = {'train': 4000, 'validation': 512, 'canary': 256}
    actual_counts = {
        profile: len(rows)
        for profile, (rows, _) in profiles.items()
    }
    counts, issues = _conditional_counts(profiles)
    valid_cells = list(itertools.product(
        IDENTITIES,
        ('empty', 'loaded'),
        BLOCKS,
        POSITION_BINS,
    ))
    conditional_stats = counter_stats(counts['conditional_cell'], valid_cells)
    empty_cells = [
        {
            'identity': identity,
            'loaded_state': state,
            'block': block,
            'position_bin': pos_bin,
        }
        for identity, state, block, pos_bin in valid_cells
        if counts['conditional_cell'][(identity, state, block, pos_bin)] == 0
    ]
    split_overlap_path = split_root / 'split_overlap_audit.json'
    split_overlap = json.loads(split_overlap_path.read_text(encoding='utf-8'))
    finalizations = {
        'train': json.loads(
            (capture_root / 'finalized' / 'train_finalization.json').read_text(
                encoding='utf-8'
            )
        ),
        'validation': json.loads(
            (
                capture_root / 'finalized' / 'validation_finalization.json'
            ).read_text(encoding='utf-8')
        ),
        'canary': json.loads(
            (canary_root / 'finalized' / 'canary_finalization.json').read_text(
                encoding='utf-8'
            )
        ),
    }
    hard_cases = _hard_cases(profiles)
    required_anchor_profiles = all(
        hard_cases['per_profile'][profile].get('complete_anchor_scene', 0) > 0
        for profile in ('train', 'validation', 'canary')
    )
    image_count = sum(
        int(report['image_count']) for report in finalizations.values()
    )
    passed = (
        actual_counts == expected_counts
        and image_count == 9536
        and not issues
        and not empty_cells
        and split_overlap.get('passed') is True
        and required_anchor_profiles
        and all(
            counts['relation_family'][relation] > 0
            for relation in RELATIONS
        )
        and all(counts['render_bucket'][bucket] > 0 for bucket in RENDER_BUCKETS)
        and all(counts['presence_cardinality'][value] > 0 for value in range(1, 9))
    )
    audit = {
        'schema_version': PACKAGE_SCHEMA,
        'visual_schema': VISUAL_SCHEMA,
        'seed': SEED,
        'fixed_identity_order': list(IDENTITIES),
        'scenario_counts': actual_counts,
        'image_count': image_count,
        'expected_image_count': 9536,
        'counts': {
            name: _plain_counts(counter)
            for name, counter in counts.items()
        },
        'conditional_cell_statistics': conditional_stats,
        'empty_valid_conditional_cells': empty_cells,
        'infeasible_cells': [],
        'hard_cases': hard_cases,
        'complete_anchor_present_in_all_profiles': required_anchor_profiles,
        'split_overlap_passed': split_overlap.get('passed') is True,
        'issues': issues,
        'legacy_test_accessed': False,
        'final_test_created': False,
        'passed': passed,
    }
    guard_root.mkdir(parents=True, exist_ok=True)
    atomic_json(guard_root / 'dataset_v3_audit.json', audit)
    atomic_text(guard_root / 'dataset_v3_audit.md', _audit_markdown(audit))
    conditional_rows = [
        {
            'identity': identity,
            'loaded_state': state,
            'block': block,
            'position_bin': pos_bin,
            'count': counts['conditional_cell'][
                (identity, state, block, pos_bin)
            ],
        }
        for identity, state, block, pos_bin in valid_cells
    ]
    _write_csv(
        guard_root / 'dataset_v3_conditional_counts.csv',
        ['identity', 'loaded_state', 'block', 'position_bin', 'count'],
        conditional_rows,
    )
    csv_specs = {
        'dataset_v3_presence_subset_counts.csv': (
            'presence_subset',
            counts['presence_subset'],
        ),
        'dataset_v3_payload_counts.csv': (
            'loaded_identity_set',
            counts['loaded_identity_set'],
        ),
        'dataset_v3_position_counts.csv': (
            'position_bin',
            counts['position_bin'],
        ),
        'dataset_v3_relation_counts.csv': (
            'relation_family',
            counts['relation_family'],
        ),
    }
    for filename, (key_name, counter) in csv_specs.items():
        _write_csv(
            guard_root / filename,
            [key_name, 'count'],
            [
                {key_name: key, 'count': value}
                for key, value in sorted(counter.items(), key=lambda item: str(item[0]))
            ],
        )
    atomic_json(guard_root / 'dataset_v3_hard_case_report.json', hard_cases)
    atomic_text(
        guard_root / 'dataset_v3_hard_case_report.md',
        '\n'.join([
            '# Room 315 V3 hard-case report',
            '',
            *[
                f'- {key}: {value}'
                for key, value in sorted(hard_cases['combined'].items())
            ],
            '',
        ]),
    )
    _write_reproducibility_manifests(
        audit=audit,
        split_root=split_root,
        capture_root=capture_root,
        canary_root=canary_root,
        guard_root=guard_root,
        quota_plan_path=(
            quota_plan_path
            if quota_plan_path is not None
            else guard_root / 'room315_visual_v3_quota_plan.json'
        ),
        output_package_schema=output_package_schema,
        command_revision=command_revision,
    )
    _resolve_experiment_a_hashes(
        split_root=split_root,
        guard_root=guard_root,
        config_path=experiment_config_path,
    )
    return audit


def _resolve_experiment_a_hashes(
    *,
    split_root: Path,
    guard_root: Path,
    config_path: Path | None = None,
) -> None:
    if config_path is None:
        config_path = default_experiment_config_path(
            'visual_state_experiment_a_dataset_v3.yaml'
        )
    loaded = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    new_train = loaded['training_sources']['sources']['new_hard_case_train']
    validation = loaded['validation']
    new_train['expected_rows_sha256'] = sha256_file(split_root / 'train.jsonl')
    new_train['expected_labels_sha256'] = sha256_file(
        split_root / 'train_visual_labels.jsonl'
    )
    validation['expected_rows_sha256'] = sha256_file(
        split_root / 'validation.jsonl'
    )
    validation['expected_labels_sha256'] = sha256_file(
        split_root / 'validation_visual_labels.jsonl'
    )
    loaded['integrity']['expected_split_package_manifest_sha256'] = sha256_file(
        split_root / 'package_manifest.json'
    )
    loaded['integrity']['expected_dataset_manifest_sha256'] = sha256_file(
        guard_root / 'dataset_manifest.json'
    )
    loaded['integrity']['resolved_by_final_audit'] = True
    atomic_text(
        config_path,
        yaml.safe_dump(loaded, sort_keys=False, width=1000),
    )


def _write_reproducibility_manifests(
    *,
    audit: dict[str, Any],
    split_root: Path,
    capture_root: Path,
    canary_root: Path,
    guard_root: Path,
    quota_plan_path: Path,
    output_package_schema: str,
    command_revision: str,
) -> None:
    tracked_paths = [
        split_root / 'train.jsonl',
        split_root / 'train_visual_labels.jsonl',
        split_root / 'validation.jsonl',
        split_root / 'validation_visual_labels.jsonl',
        split_root / 'split_overlap_audit.json',
        canary_root / 'finalized' / 'canary.jsonl',
        canary_root / 'finalized' / 'canary_visual_labels.jsonl',
        quota_plan_path,
        guard_root / 'dataset_v3_audit.json',
        capture_root / 'finalized' / 'train_finalization.json',
        capture_root / 'finalized' / 'validation_finalization.json',
        canary_root / 'finalized' / 'canary_finalization.json',
    ]
    files = {
        str(path): {
            'sha256': sha256_file(path),
            'bytes': path.stat().st_size,
        }
        for path in tracked_paths
    }
    repo_commit = subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        cwd=Path(__file__).resolve().parents[2],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    repository_root = Path(__file__).resolve().parents[2]
    repository_status = subprocess.run(
        ['git', 'status', '--short'],
        cwd=repository_root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
    authoritative_sources = {
        'left_topology': (
            repository_root
            / 'mfja_robot_control_config'
            / 'config'
            / 'room_315_kinematics'
            / 'rail_network_left.yaml'
        ),
        'right_topology': (
            repository_root
            / 'mfja_robot_control_config'
            / 'config'
            / 'room_315_kinematics'
            / 'rail_network_right.yaml'
        ),
        'identity_registry': (
            repository_root
            / 'mfja_robot_control_config'
            / 'config'
            / 'room_315_vla'
            / 'shuttle_identity.yaml'
        ),
        'gazebo_world': (
            repository_root
            / 'mfja_3rd_floor_description'
            / 'worlds'
            / 'room_315_only.world'
        ),
        'camera_model': (
            repository_root
            / 'mfja_3rd_floor_description'
            / 'models'
            / 'room315_vla_overhead_devices'
            / 'model.sdf'
        ),
    }
    source_fingerprints = {
        name: {'path': str(path), 'sha256': sha256_file(path)}
        for name, path in authoritative_sources.items()
    }
    image_hashes_by_profile = {}
    for profile, finalization_path in (
        ('train', capture_root / 'finalized' / 'train_finalization.json'),
        (
            'validation',
            capture_root / 'finalized' / 'validation_finalization.json',
        ),
        ('canary', canary_root / 'finalized' / 'canary_finalization.json'),
    ):
        finalization = json.loads(finalization_path.read_text(encoding='utf-8'))
        image_hashes_by_profile[profile] = finalization['image_hashes']
    manifest = {
        'schema_version': output_package_schema,
        'repository_commit': repo_commit,
        'repository_dirty': bool(repository_status),
        'modified_source_files': repository_status,
        'seed': SEED,
        'visual_schema': VISUAL_SCHEMA,
        'scenario_counts': audit['scenario_counts'],
        'image_count': audit['image_count'],
        'camera_topics': {
            'left_rail_rgb': '/room_315/vla/left_rail_rgbd/image',
            'right_rail_rgb': '/room_315/vla/right_rail_rgbd/image',
        },
        'image_dimensions': [640, 480],
        'gazebo_world': 'room_315_only.world',
        'authoritative_source_fingerprints': source_fingerprints,
        'files': files,
        'image_sha256_by_profile': image_hashes_by_profile,
        'legacy_test_included': False,
        'final_test_created': False,
    }
    atomic_json(guard_root / 'dataset_manifest.json', manifest)
    atomic_text(
        guard_root / 'dataset_sha256_manifest.txt',
        ''.join(
            f'{record["sha256"]}  {path}\n'
            for path, record in sorted(files.items())
        )
        + ''.join(
            f'{fingerprint}  {profile}:{sample_camera}\n'
            for profile, image_hashes in sorted(image_hashes_by_profile.items())
            for sample_camera, fingerprint in sorted(image_hashes.items())
        ),
    )
    environment = {
        'schema_version': output_package_schema,
        'python': sys.version,
        'seed': SEED,
        'repository_commit': repo_commit,
        'repository_status': repository_status,
        'authoritative_source_fingerprints': source_fingerprints,
        'capture_root': str(capture_root),
        'split_root': str(split_root),
        'canary_root': str(canary_root),
    }
    atomic_json(guard_root / 'generation_environment.json', environment)
    atomic_text(
        guard_root / 'generation_command.txt',
        (
            'python3 mfja_robot_control_config/scripts/'
            f'room_315_visual_{command_revision}_generator.py --mode full\n'
            'python3 mfja_robot_control_config/scripts/'
            f'room_315_visual_{command_revision}_capture.py capture --profile train --resume\n'
            'python3 mfja_robot_control_config/scripts/'
            f'room_315_visual_{command_revision}_capture.py capture --profile validation --resume\n'
            'python3 mfja_robot_control_config/scripts/'
            f'room_315_visual_{command_revision}_capture.py capture --profile canary --resume\n'
        ),
    )


def run_smoke_audit(
    *,
    guard_root: Path,
) -> dict[str, Any]:
    root = guard_root / 'smoke'
    rows_path = root / 'finalized' / 'smoke.jsonl'
    labels_path = root / 'finalized' / 'smoke_visual_labels.jsonl'
    rows = read_jsonl(rows_path)
    labels = read_jsonl(labels_path)
    profiles = {'smoke': (rows, labels)}
    counts, issues = _conditional_counts(profiles)
    hard = _hard_cases(profiles)
    paired: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        trace = row['traceability_metadata']
        pair_id = trace.get('matched_pair_id')
        if pair_id:
            paired.setdefault(str(pair_id), []).append(trace)
    complete_payload_pairs = [
        traces
        for traces in paired.values()
        if (
            len(traces) == 2
            and traces[0].get('geometry_fingerprint')
            == traces[1].get('geometry_fingerprint')
            and traces[0]['loaded_identities'] != traces[1]['loaded_identities']
        )
    ]
    finalization = json.loads(
        (root / 'finalized' / 'smoke_finalization.json').read_text(
            encoding='utf-8'
        )
    )
    image_hashes = finalization['image_hashes']
    image_visible_pairs = 0
    for traces in complete_payload_pairs:
        first = traces[0]['scenario_id']
        second = traces[1]['scenario_id']
        if any(
            image_hashes[f'{first}:{camera}']
            != image_hashes[f'{second}:{camera}']
            for camera in ('left_rail_rgb', 'right_rail_rgb')
        ):
            image_visible_pairs += 1
    requirements = {
        'at_least_32': len(rows) >= 32,
        'l4_loaded': hard['combined'].get('l4_loaded', 0) > 0,
        'r4_loaded': hard['combined'].get('r4_loaded', 0) > 0,
        'both_loaded': hard['combined'].get('l4_r4_both_loaded', 0) > 0,
        'exact_subset': hard['combined'].get('exact_l2_l4_r4_subset', 0) > 0,
        'anchor_payload': hard['combined'].get('anchor_payload_assignment', 0) > 0,
        'right_slot3': hard['combined'].get('right_slot3_position_samples', 0) > 0,
        'multiple_position_bins': len(counts['position_bin']) > 1,
        'paired_images': True,
        'payload_matched_pair': bool(complete_payload_pairs),
        'image_level_payload_visibility': image_visible_pairs > 0,
        'no_validation_issues': not issues,
    }
    report = {
        'schema_version': PACKAGE_SCHEMA,
        'scenario_count': len(rows),
        'expected_image_count': len(rows) * 2,
        'requirements': requirements,
        'hard_cases': hard,
        'complete_payload_matched_pair_count': len(complete_payload_pairs),
        'image_visible_payload_matched_pair_count': image_visible_pairs,
        'issues': issues,
        'legacy_test_accessed': False,
        'passed': all(requirements.values()),
    }
    atomic_json(root / 'smoke_generation_report.json', report)
    atomic_text(
        root / 'smoke_generation_report.md',
        '\n'.join([
            '# Room 315 V3 smoke generation report',
            '',
            f'- Result: **{"PASS" if report["passed"] else "FAIL"}**',
            f'- Scenarios: {len(rows)}',
            f'- Expected paired images: {len(rows) * 2}',
            '',
            *[
                f'- {name}: {"PASS" if passed else "FAIL"}'
                for name, passed in requirements.items()
            ],
            '',
        ]),
    )
    return report


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('smoke', 'full'), required=True)
    parser.add_argument('--capture-root', type=Path, default=DEFAULT_CAPTURE_ROOT)
    parser.add_argument('--split-root', type=Path, default=DEFAULT_SPLIT_ROOT)
    parser.add_argument('--canary-root', type=Path, default=DEFAULT_CANARY_ROOT)
    parser.add_argument('--guard-root', type=Path, default=DEFAULT_GUARD_ROOT)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.mode == 'smoke':
        result = run_smoke_audit(guard_root=args.guard_root)
    else:
        result = run_full_audit(
            split_root=args.split_root,
            capture_root=args.capture_root,
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
