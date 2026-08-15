#!/usr/bin/env python3
"""Build a post-hoc cross-protocol overlap audit for Room 315 datasets.

This tool is deliberately separate from the preregistered Final-Test audit.  It
opens already materialized datasets after the experimental work is complete and
compares the legacy grouped split with the later V3R1/V4 partitions.  Its output
is provenance documentation, not a selection, calibration, or acceptance gate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
from dataclasses import dataclass
from itertools import combinations
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_DATA_ROOT = Path.home()

DEFAULT_LEGACY_ROOT = (
    DEFAULT_DATA_ROOT
    / 'room315_arbitrary_subset_visual_splits_v1_seed31520260730'
)
DEFAULT_LEGACY_DATASET_ROOT = (
    DEFAULT_DATA_ROOT
    / 'room315_arbitrary_subset_visual_2040_capture_v2_seed31520260729'
    / 'dataset'
)
DEFAULT_HARD_ROOT = (
    DEFAULT_DATA_ROOT
    / 'room315_hard_case_visual_v3r1_splits_seed31520260730'
)
DEFAULT_HARD_DATASET_ROOT = (
    DEFAULT_DATA_ROOT
    / 'room315_hard_case_visual_v3r1_capture_seed31520260730'
    / 'dataset'
)
DEFAULT_CANARY_ROOT = (
    DEFAULT_DATA_ROOT
    / 'room315_hard_case_visual_v3r1_canary_seed31520260730'
    / 'finalized'
)
DEFAULT_CANARY_DATASET_ROOT = (
    DEFAULT_DATA_ROOT
    / 'room315_hard_case_visual_v3r1_canary_seed31520260730'
    / 'dataset'
)
DEFAULT_FINAL_ROOT = (
    DEFAULT_DATA_ROOT
    / 'room315_visual_v4_final_test_seed3152026081101_coveragecompat'
    / 'finalized'
)
DEFAULT_FINAL_DATASET_ROOT = (
    DEFAULT_DATA_ROOT
    / 'room315_visual_v4_final_test_seed3152026081101_coveragecompat'
    / 'dataset'
)
DEFAULT_OUTPUT = (
    REPO_ROOT
    / 'report'
    / 'evidence'
    / 'room315_dataset_partition_overlap_post_hoc_2026-08-12'
    / 'overlap_audit.json'
)

SHUTTLE_ORDER = ('L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4')

EXACT_METRICS = (
    'sample_ids',
    'episode_ids',
    'scenario_ids',
    'source_scenario_ids',
    'individual_image_sha256',
    'image_pair_sha256',
    'full_visual_state_label_sha256',
    'full_label_row_sha256',
    'scenario_families',
    'geometry_fingerprints',
    'capture_configuration_fingerprints',
    'configuration_family_ids',
    'configuration_core_family_ids',
    'generated_variant_sha256',
    'trajectory_sha256',
    'semantic_state_sha256',
)


class AuditError(RuntimeError):
    """Raised when an input is incomplete or internally inconsistent."""


@dataclass(frozen=True)
class PartitionSpec:
    """Files and provenance needed to index one materialized partition."""

    name: str
    protocol: str
    role: str
    rows_path: Path
    labels_path: Path
    dataset_root: Path
    seeds: tuple[int, ...]


@dataclass(frozen=True)
class PartitionIndex:
    """Canonical values collected from one partition."""

    spec: PartitionSpec
    row_count: int
    label_count: int
    present_target_count: int
    metric_values: Mapping[str, frozenset[str]]
    metric_observation_counts: Mapping[str, int]
    presence_configurations: frozenset[str]
    rows_sha256: str
    labels_sha256: str


def _canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(',', ':'),
        ensure_ascii=False,
        allow_nan=False,
    )


def _json_sha256(value: Any) -> str:
    return hashlib.sha256(_canonical_json(value).encode('utf-8')).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _portable_path(path: Path) -> str:
    """Represent home-relative inputs without publishing a local username."""

    resolved = path.expanduser().resolve()
    try:
        return str(Path('~') / resolved.relative_to(Path.home().resolve()))
    except ValueError:
        return str(resolved)


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        raise AuditError(f'missing JSONL input: {path}')
    rows: list[dict[str, Any]] = []
    with path.open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                value = json.loads(stripped)
            except json.JSONDecodeError as exc:
                raise AuditError(
                    f'invalid JSON in {path}:{line_number}: {exc}'
                ) from exc
            if not isinstance(value, dict):
                raise AuditError(
                    f'{path}:{line_number} must contain a JSON object'
                )
            rows.append(value)
    return rows


def _required_string(
    row: Mapping[str, Any], key: str, context: str
) -> str:
    value = str(row.get(key) or '').strip()
    if not value:
        raise AuditError(f'{context} is missing {key}')
    return value


def _trace(row: Mapping[str, Any]) -> Mapping[str, Any]:
    value = row.get('traceability_metadata')
    return value if isinstance(value, Mapping) else {}


def _stratification(row: Mapping[str, Any]) -> Mapping[str, Any]:
    value = row.get('stratification_metadata')
    return value if isinstance(value, Mapping) else {}


def _visual_labels(label: Mapping[str, Any], context: str) -> Mapping[str, Any]:
    value = label.get('visual_state_labels')
    if not isinstance(value, Mapping):
        raise AuditError(f'{context} is missing visual_state_labels')
    shuttles = value.get('shuttles')
    if not isinstance(shuttles, list):
        raise AuditError(f'{context} is missing visual_state_labels.shuttles')
    return value


def _present_shuttles(
    visual: Mapping[str, Any], context: str
) -> list[Mapping[str, Any]]:
    shuttles = visual['shuttles']
    result = [
        shuttle
        for shuttle in shuttles
        if isinstance(shuttle, Mapping) and bool(shuttle.get('presence'))
    ]
    identities = [str(shuttle.get('id') or '') for shuttle in result]
    if any(identity not in SHUTTLE_ORDER for identity in identities):
        raise AuditError(f'{context} contains an unknown shuttle identity')
    if len(identities) != len(set(identities)):
        raise AuditError(f'{context} contains duplicate shuttle identities')
    return result


def _presence_configuration(present: Sequence[Mapping[str, Any]]) -> str:
    active = {str(shuttle['id']) for shuttle in present}
    bitmask = sum(
        1 << index
        for index, identity in enumerate(SHUTTLE_ORDER)
        if identity in active
    )
    return f'presence_{bitmask:03d}'


def _trajectory_payload(
    present: Sequence[Mapping[str, Any]], *, include_loaded: bool
) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for shuttle in sorted(present, key=lambda item: SHUTTLE_ORDER.index(
        str(item['id'])
    )):
        location = shuttle.get('location')
        rail = shuttle.get('rail_position')
        if not isinstance(location, Mapping) or not isinstance(rail, Mapping):
            raise AuditError('present shuttle lacks location or rail_position')
        item: dict[str, Any] = {
            'id': str(shuttle['id']),
            'block': str(location.get('block') or ''),
            's_ratio': rail.get('s_ratio'),
        }
        if include_loaded:
            item['loaded_state'] = str(shuttle.get('loaded_state') or '')
        result.append(item)
    return result


def _generated_variant_payload(row: Mapping[str, Any]) -> dict[str, Any]:
    trace = _trace(row)
    keys = (
        'v2_plan_id',
        'spec_id',
        'configuration_family_id',
        'configuration_core_family_id',
        'configuration_variant',
        'generation_index',
        'source_scenario_id',
    )
    payload = {key: trace[key] for key in keys if trace.get(key) is not None}
    if not payload:
        payload = {
            'scenario_family': row.get('scenario_family'),
            'episode_id': row.get('episode_id'),
        }
    return payload


def _resolve_image(
    row: Mapping[str, Any], camera: str, reference: str, dataset_root: Path
) -> tuple[Path, str | None]:
    trace_images = _trace(row).get('source_images')
    source = (
        trace_images.get(camera)
        if isinstance(trace_images, Mapping)
        else None
    )
    declared: str | None = None
    candidates: list[Path] = []
    if isinstance(source, Mapping):
        declared_value = str(source.get('sha256') or '').strip().lower()
        declared = declared_value or None
        absolute = str(source.get('absolute_path') or '').strip()
        relative = str(source.get('path') or '').strip()
        if absolute:
            candidates.append(Path(absolute).expanduser())
        if relative:
            candidates.append(dataset_root / relative)
    candidates.append(dataset_root / reference)
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve(), declared
    raise AuditError(
        f'cannot resolve {camera} image for '
        f'{row.get("sample_id")}: {candidates}'
    )


def _image_hashes(
    row: Mapping[str, Any], dataset_root: Path
) -> tuple[dict[str, str], str]:
    model_input = row.get('model_input')
    overhead = (
        model_input.get('overhead_images')
        if isinstance(model_input, Mapping)
        else None
    )
    if not isinstance(overhead, Mapping) or not overhead:
        raise AuditError(f'{row.get("sample_id")} has no overhead images')
    hashes: dict[str, str] = {}
    for camera, value in sorted(overhead.items()):
        reference = str(value or '').strip()
        if not reference:
            raise AuditError(
                f'{row.get("sample_id")} has an empty {camera} image path'
            )
        image_path, declared = _resolve_image(
            row, str(camera), reference, dataset_root
        )
        actual = _sha256_file(image_path)
        if declared is not None and actual != declared:
            raise AuditError(
                f'image digest mismatch for {image_path}: '
                f'declared={declared}, actual={actual}'
            )
        hashes[str(camera)] = actual
    return hashes, _json_sha256(hashes)


def _optional_metadata_value(
    row: Mapping[str, Any], key: str
) -> str | None:
    # The legacy grouped schema stores geometry in stratification_metadata,
    # whereas the later V3R1/V4 schemas store it in traceability_metadata.
    # A top-level fallback also permits canonicalized relocated fixtures.
    for container in (_trace(row), _stratification(row), row):
        value = container.get(key)
        if value is not None:
            normalized = str(value).strip()
            if normalized:
                return normalized
    return None


def build_partition_index(spec: PartitionSpec) -> PartitionIndex:
    """Read, validate, and canonically index a materialized partition."""

    rows = _read_jsonl(spec.rows_path)
    labels = _read_jsonl(spec.labels_path)
    labels_by_sample: dict[str, dict[str, Any]] = {}
    for index, label in enumerate(labels):
        sample_id = _required_string(
            label, 'sample_id', f'{spec.name} labels[{index}]'
        )
        if sample_id in labels_by_sample:
            raise AuditError(f'{spec.name} has duplicate label {sample_id}')
        labels_by_sample[sample_id] = label

    metric_values: dict[str, set[str]] = {
        name: set() for name in EXACT_METRICS
    }
    metric_observation_counts: dict[str, int] = {
        name: 0 for name in EXACT_METRICS
    }
    presence_configurations: set[str] = set()
    seen_samples: set[str] = set()
    present_target_count = 0

    for index, row in enumerate(rows):
        context = f'{spec.name} rows[{index}]'
        sample_id = _required_string(row, 'sample_id', context)
        if sample_id in seen_samples:
            raise AuditError(f'{spec.name} has duplicate row {sample_id}')
        seen_samples.add(sample_id)
        label = labels_by_sample.get(sample_id)
        if label is None:
            raise AuditError(f'{spec.name} has no label for {sample_id}')

        episode_id = _required_string(row, 'episode_id', context)
        scenario_id = _optional_metadata_value(row, 'scenario_id') or episode_id
        visual = _visual_labels(label, f'{spec.name}:{sample_id}')
        present = _present_shuttles(visual, f'{spec.name}:{sample_id}')
        present_target_count += len(present)
        presence_configurations.add(_presence_configuration(present))

        image_hashes, pair_hash = _image_hashes(row, spec.dataset_root)
        metric_values['sample_ids'].add(sample_id)
        metric_observation_counts['sample_ids'] += 1
        metric_values['episode_ids'].add(episode_id)
        metric_observation_counts['episode_ids'] += 1
        metric_values['scenario_ids'].add(scenario_id)
        metric_observation_counts['scenario_ids'] += 1
        metric_values['individual_image_sha256'].update(
            image_hashes.values()
        )
        metric_observation_counts['individual_image_sha256'] += len(
            image_hashes
        )
        metric_values['image_pair_sha256'].add(pair_hash)
        metric_observation_counts['image_pair_sha256'] += 1
        metric_values['full_visual_state_label_sha256'].add(
            _json_sha256(visual)
        )
        metric_observation_counts['full_visual_state_label_sha256'] += 1
        metric_values['full_label_row_sha256'].add(_json_sha256(label))
        metric_observation_counts['full_label_row_sha256'] += 1
        metric_values['trajectory_sha256'].add(
            _json_sha256(_trajectory_payload(present, include_loaded=False))
        )
        metric_observation_counts['trajectory_sha256'] += 1
        metric_values['semantic_state_sha256'].add(
            _json_sha256(_trajectory_payload(present, include_loaded=True))
        )
        metric_observation_counts['semantic_state_sha256'] += 1
        metric_values['generated_variant_sha256'].add(
            _json_sha256(_generated_variant_payload(row))
        )
        metric_observation_counts['generated_variant_sha256'] += 1

        scenario_family = str(row.get('scenario_family') or '').strip()
        if scenario_family:
            metric_values['scenario_families'].add(scenario_family)
            metric_observation_counts['scenario_families'] += 1
        for key, metric in (
            ('source_scenario_id', 'source_scenario_ids'),
            ('geometry_fingerprint', 'geometry_fingerprints'),
            (
                'capture_configuration_fingerprint',
                'capture_configuration_fingerprints',
            ),
            ('configuration_family_id', 'configuration_family_ids'),
            (
                'configuration_core_family_id',
                'configuration_core_family_ids',
            ),
        ):
            value = _optional_metadata_value(row, key)
            if value is not None:
                metric_values[metric].add(value)
                metric_observation_counts[metric] += 1

    if seen_samples != set(labels_by_sample):
        extra = sorted(set(labels_by_sample) - seen_samples)[:5]
        raise AuditError(
            f'{spec.name} has labels without rows; examples={extra}'
        )

    return PartitionIndex(
        spec=spec,
        row_count=len(rows),
        label_count=len(labels),
        present_target_count=present_target_count,
        metric_values={
            name: frozenset(values)
            for name, values in metric_values.items()
        },
        metric_observation_counts=metric_observation_counts,
        presence_configurations=frozenset(presence_configurations),
        rows_sha256=_sha256_file(spec.rows_path),
        labels_sha256=_sha256_file(spec.labels_path),
    )


def _examples(values: Iterable[str], maximum: int) -> list[str]:
    return sorted(values)[:maximum]


def compare_partitions(
    left: PartitionIndex, right: PartitionIndex, *, max_examples: int = 8
) -> dict[str, Any]:
    """Compare exact artefacts separately from abstract factor coverage."""

    exact_counts: dict[str, int | None] = {}
    exact_examples: dict[str, list[str]] = {}
    exact_coverage: dict[str, dict[str, Any]] = {}
    for metric in EXACT_METRICS:
        left_count = left.metric_observation_counts[metric]
        right_count = right.metric_observation_counts[metric]
        comparable = left_count > 0 and right_count > 0
        exact_coverage[metric] = {
            'left_extracted_value_count': left_count,
            'right_extracted_value_count': right_count,
            'left_row_count': left.row_count,
            'right_row_count': right.row_count,
            'comparison_status': (
                'comparable' if comparable else 'not_comparable_missing_field'
            ),
        }
        if not comparable:
            exact_counts[metric] = None
            continue
        overlap = (
            left.metric_values[metric] & right.metric_values[metric]
        )
        exact_counts[metric] = len(overlap)
        if overlap:
            exact_examples[metric] = _examples(overlap, max_examples)

    presence_overlap = (
        left.presence_configurations & right.presence_configurations
    )
    seed_overlap = set(left.spec.seeds) & set(right.spec.seeds)
    return {
        'left': left.spec.name,
        'right': right.spec.name,
        'exact_overlap_counts': exact_counts,
        'exact_metric_coverage': exact_coverage,
        'exact_overlap_examples': exact_examples,
        'abstract_presence_configuration_overlap': {
            'left_unique_count': len(left.presence_configurations),
            'right_unique_count': len(right.presence_configurations),
            'overlap_count': len(presence_overlap),
            'examples': _examples(presence_overlap, max_examples),
            'interpretation': (
                'factor-level support overlap; it is not sample leakage by '
                'itself'
            ),
        },
        'numeric_seed_overlap': {
            'overlap_count': len(seed_overlap),
            'values': sorted(seed_overlap),
            'interpretation': (
                'numeric seed reuse only; generator namespaces and exact '
                'artefacts are audited separately'
            ),
        },
    }


def build_audit(
    specs: Sequence[PartitionSpec], *, max_examples: int = 8
) -> dict[str, Any]:
    """Build the complete seven-partition post-hoc audit."""

    names = [spec.name for spec in specs]
    if len(names) != len(set(names)):
        raise AuditError('partition names must be unique')
    indexes = [build_partition_index(spec) for spec in specs]
    comparisons = [
        compare_partitions(left, right, max_examples=max_examples)
        for left, right in combinations(indexes, 2)
    ]

    metric_pair_counts = {
        metric: sum(
            (comparison['exact_overlap_counts'][metric] or 0) > 0
            for comparison in comparisons
        )
        for metric in EXACT_METRICS
    }
    metric_unavailable_pair_counts = {
        metric: sum(
            comparison['exact_overlap_counts'][metric] is None
            for comparison in comparisons
        )
        for metric in EXACT_METRICS
    }
    presence_pairs = sum(
        comparison['abstract_presence_configuration_overlap'][
            'overlap_count'
        ] > 0
        for comparison in comparisons
    )
    seed_pairs = sum(
        comparison['numeric_seed_overlap']['overlap_count'] > 0
        for comparison in comparisons
    )

    return {
        'schema_version': (
            'room315.dataset_partition_overlap.post_hoc.v1'
        ),
        'audit_classification': 'post_hoc_supplemental_provenance',
        'generator': {
            'repository_path': (
                'mfja_robot_control_config/scripts/'
                'room_315_dataset_partition_overlap_post_hoc.py'
            ),
            'script_sha256': _sha256_file(Path(__file__).resolve()),
        },
        'experimental_effect': {
            'model_training': False,
            'checkpoint_selection': False,
            'early_stopping': False,
            'hyperparameter_tuning': False,
            'temperature_calibration': False,
            'confidence_threshold_selection': False,
            'acceptance_gate': False,
            'final_evidence': False,
        },
        'sealed_evidence_policy': {
            'modifies_preregistered_or_historical_evidence': False,
            'relationship': (
                'supplemental comparison performed after all listed '
                'partitions had already been materialized and the current '
                'Final Test had been evaluated'
            ),
        },
        'definitions': {
            'exact_json': (
                'SHA-256 of canonical JSON with sorted keys and compact '
                'separators'
            ),
            'full_visual_state_label_sha256': (
                'canonical hash of the complete visual_state_labels target '
                'payload, excluding provenance wrapper fields; equality can '
                'mean target/state support recurrence across otherwise '
                'different samples and images'
            ),
            'full_label_row_sha256': (
                'canonical hash of the complete label JSON object including '
                'provenance wrapper fields'
            ),
            'generated_variant_sha256': (
                'canonical hash of available v2 plan/spec/configuration '
                'family/variant/generation/source identifiers'
            ),
            'presence_configuration': (
                'presence_NNN bitmask derived only from the eight label '
                'presence booleans in L1..L4,R1..R4 order'
            ),
            'image_pair_sha256': (
                'canonical hash of the camera-name to image-content-SHA-256 '
                'mapping'
            ),
        },
        'input_partitions': {
            index.spec.name: {
                'protocol': index.spec.protocol,
                'role': index.spec.role,
                'rows': {
                    'path': _portable_path(index.spec.rows_path),
                    'sha256': index.rows_sha256,
                    'count': index.row_count,
                },
                'labels': {
                    'path': _portable_path(index.spec.labels_path),
                    'sha256': index.labels_sha256,
                    'count': index.label_count,
                },
                'dataset_root': _portable_path(index.spec.dataset_root),
                'seeds': list(index.spec.seeds),
                'present_target_count': index.present_target_count,
                'presence_configuration_count': len(
                    index.presence_configurations
                ),
                'metric_unique_counts': {
                    metric: len(index.metric_values[metric])
                    for metric in EXACT_METRICS
                },
                'metric_observation_counts': dict(
                    index.metric_observation_counts
                ),
            }
            for index in indexes
        },
        'pairwise_comparisons': comparisons,
        'summary': {
            'partition_count': len(indexes),
            'pairwise_comparison_count': len(comparisons),
            'pairs_with_overlap_by_exact_metric': metric_pair_counts,
            'pairs_not_comparable_by_metric': (
                metric_unavailable_pair_counts
            ),
            'pairs_with_abstract_presence_configuration_overlap': (
                presence_pairs
            ),
            'pairs_with_numeric_seed_overlap': seed_pairs,
            'interpretation': (
                'Exact artefact/identifier/variant overlap, abstract '
                'presence-factor overlap, and numeric seed reuse are separate '
                'claims and must not be conflated.'
            ),
        },
    }


def _partition_specs(args: argparse.Namespace) -> list[PartitionSpec]:
    legacy_root = args.legacy_root.expanduser().resolve()
    hard_root = args.hard_root.expanduser().resolve()
    canary_root = args.canary_root.expanduser().resolve()
    final_root = args.final_root.expanduser().resolve()
    legacy_seeds = (args.legacy_capture_seed, args.legacy_split_seed)
    hard_seeds = (args.hard_generation_seed,)
    return [
        PartitionSpec(
            name='legacy_train',
            protocol='legacy_grouped_v1',
            role='historical_training_replay',
            rows_path=legacy_root / 'train.jsonl',
            labels_path=legacy_root / 'train_visual_labels.jsonl',
            dataset_root=args.legacy_dataset_root,
            seeds=legacy_seeds,
        ),
        PartitionSpec(
            name='legacy_grouped_validation',
            protocol='legacy_grouped_v1',
            role='historical_predecessor_checkpoint_selection',
            rows_path=legacy_root / 'validation.jsonl',
            labels_path=legacy_root / 'validation_visual_labels.jsonl',
            dataset_root=args.legacy_dataset_root,
            seeds=legacy_seeds,
        ),
        PartitionSpec(
            name='legacy_consumed_test',
            protocol='legacy_grouped_v1',
            role='historical_test_consumed_once',
            rows_path=legacy_root / 'test.jsonl',
            labels_path=legacy_root / 'test_visual_labels.jsonl',
            dataset_root=args.legacy_dataset_root,
            seeds=legacy_seeds,
        ),
        PartitionSpec(
            name='hard_case_train',
            protocol='v3r1_current_development',
            role='current_training',
            rows_path=hard_root / 'train.jsonl',
            labels_path=hard_root / 'train_visual_labels.jsonl',
            dataset_root=args.hard_dataset_root,
            seeds=hard_seeds,
        ),
        PartitionSpec(
            name='hard_case_validation',
            protocol='v3r1_v4_current_development',
            role='current_checkpoint_selection_calibration_thresholds',
            rows_path=hard_root / 'validation.jsonl',
            labels_path=hard_root / 'validation_visual_labels.jsonl',
            dataset_root=args.hard_dataset_root,
            seeds=hard_seeds,
        ),
        PartitionSpec(
            name='development_canary',
            protocol='v3r1_v4_current_development',
            role='post_selection_regression_and_manual_promotion',
            rows_path=canary_root / 'canary.jsonl',
            labels_path=canary_root / 'canary_visual_labels.jsonl',
            dataset_root=args.canary_dataset_root,
            seeds=hard_seeds,
        ),
        PartitionSpec(
            name='preregistered_final_test',
            protocol='v4_final_evidence',
            role='one_shot_final_acceptance_evidence',
            rows_path=final_root / 'final_test.jsonl',
            labels_path=final_root / 'final_test_visual_labels.jsonl',
            dataset_root=args.final_dataset_root,
            seeds=(args.final_generation_seed,),
        ),
    ]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--legacy-root', type=Path, default=DEFAULT_LEGACY_ROOT)
    parser.add_argument(
        '--legacy-dataset-root',
        type=Path,
        default=DEFAULT_LEGACY_DATASET_ROOT,
    )
    parser.add_argument('--hard-root', type=Path, default=DEFAULT_HARD_ROOT)
    parser.add_argument(
        '--hard-dataset-root', type=Path, default=DEFAULT_HARD_DATASET_ROOT
    )
    parser.add_argument('--canary-root', type=Path, default=DEFAULT_CANARY_ROOT)
    parser.add_argument(
        '--canary-dataset-root',
        type=Path,
        default=DEFAULT_CANARY_DATASET_ROOT,
    )
    parser.add_argument('--final-root', type=Path, default=DEFAULT_FINAL_ROOT)
    parser.add_argument(
        '--final-dataset-root',
        type=Path,
        default=DEFAULT_FINAL_DATASET_ROOT,
    )
    parser.add_argument('--legacy-capture-seed', type=int, default=31520260729)
    parser.add_argument('--legacy-split-seed', type=int, default=31520260730)
    parser.add_argument('--hard-generation-seed', type=int, default=31520260730)
    parser.add_argument('--final-generation-seed', type=int, default=3152026081101)
    parser.add_argument('--max-examples', type=int, default=8)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    return parser


def _atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, indent=2, sort_keys=True) + '\n'
    with tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        dir=path.parent,
        prefix=f'.{path.name}.',
        suffix='.tmp',
        delete=False,
    ) as stream:
        temporary = Path(stream.name)
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.max_examples < 0:
        raise AuditError('--max-examples must be non-negative')
    audit = build_audit(
        _partition_specs(args), max_examples=args.max_examples
    )
    output = args.output.expanduser().resolve()
    _atomic_json(output, audit)
    print(output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
