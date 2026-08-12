#!/usr/bin/env python3
"""Fail-closed trainer and evaluator for the isolated Room 315 visual V4.

This entry point is intentionally independent from the approved V3 runtime and
report.  Training and checkpoint selection use only configured train sources
and validation.  Canary data is loaded only by the explicit
``evaluate-canary`` command, after a completed training run has selected a
checkpoint on validation.

The module imports Torch lazily.  Configuration, data-contract, target, image,
and sampler preflight can therefore run on hosts that do not have a training
stack installed.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import random
import re
import sys
import time
from collections import Counter, defaultdict
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
from PIL import Image, ImageEnhance, ImageFilter


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_contract_v4 import (  # noqa: E402
    FIXED_IDENTITIES,
    SEGMENT_CLASSES,
    SIDES,
    derive_side,
    load_authoritative_public_segment_length_contract,
)
from room_315_visual_model_v4 import (  # noqa: E402
    V4_MODEL_KIND,
    V4_OUTPUT_KEYS,
    V4_SLOT_ORDER,
    build_visual_state_model_v4,
    initialize_v4_backbone_from_v3_model_state_dict,
)
from room_315_visual_state_dataset import (  # noqa: E402
    normalize_visual_state_labels,
    validate_visual_model_input,
)


CONFIG_SCHEMA_VERSION = 'room315.visual_training.v4.pilot_config.v1'
CHECKPOINT_SCHEMA_VERSION = 'room315.visual_training.v4.checkpoint.v1'
RUN_SCHEMA_VERSION = 'room315.visual_training.v4.run.v1'
V3_CANARY_BASELINE_SCHEMA_VERSION = 'room315.visual_v3.canary_baseline.v1'
CANARY_ATTEMPT_SCHEMA_VERSION = 'room315.visual_v4.canary_attempt.v1'
CANARY_COVERAGE_SCHEMA_VERSION = 'room315.visual_v4.canary_coverage.v1'
V3_PREPROCESSING_CONTRACT = (
    'v3_direct_bilinear_224_imagenet_normalization_target_stats_decode'
)
DEFAULT_CONFIG_PATH = (
    SCRIPT_DIR.parent / 'config' / 'room_315_vla' / 'visual_state_training_v4.json'
)
DEFAULT_ORIGINAL_IMAGE_SIZE = (640, 480)
TARGET_IMAGE_SIZE = (320, 240)
CAMERAS = ('left_rail_rgb', 'right_rail_rgb')
LOADED_CLASSES = ('empty', 'loaded')
SIDE_INDEX = {'left': 0, 'right': 1}
ENVIRONMENT_PATTERN = re.compile(
    r'\$(?:\{([A-Za-z_][A-Za-z0-9_]*)\}|([A-Za-z_][A-Za-z0-9_]*))'
)
PATH_FIELD_NAMES = frozenset({
    'dataset_root', 'rows', 'labels', 'checkpoint', 'output_root',
})
LOSS_SCALAR_KEYS = (
    'loss', 'segment_ce', 'loaded_ce', 's_ratio', 'bbox', 'bbox_l1',
    'bbox_giou', 'topology',
)


class V4TrainerError(RuntimeError):
    """Raised whenever an input cannot satisfy the isolated V4 contract."""


@dataclass(frozen=True)
class PairedRecord:
    sample_id: str
    source: str
    role: str
    dataset_root: Path
    row: dict[str, Any]
    label: dict[str, Any]
    normalized_label: dict[str, Any]
    image_paths: dict[str, Path]
    trace: dict[str, Any]
    draw_index: int = 0
    resample_occurrence: int = 0


@dataclass(frozen=True)
class PreflightBundle:
    config_path: Path
    config: dict[str, Any]
    train_by_source: dict[str, tuple[PairedRecord, ...]]
    validation: tuple[PairedRecord, ...]
    report: dict[str, Any]


def canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    )


def sha256_file(path: Path | str) -> str:
    candidate = Path(path).expanduser().resolve()
    assert_not_test_path(candidate, context='fingerprinted artifact')
    if not candidate.is_file():
        raise V4TrainerError(f'artifact is not a file: {candidate}')
    digest = hashlib.sha256()
    with candidate.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def file_fingerprint(path: Path | str) -> dict[str, Any]:
    candidate = Path(path).expanduser().resolve()
    return {
        'path': str(candidate),
        'sha256': sha256_file(candidate),
        'bytes': candidate.stat().st_size,
    }


def assert_not_test_path(path: Path | str, *, context: str = 'path') -> Path:
    """Reject a basename beginning with ``test`` (case-insensitive).

    Only the basename is checked, as required by the data lock.  This avoids
    treating an unrelated parent such as pytest's temporary directory as a
    data split while still rejecting ``test.jsonl``, ``test_images`` and all
    equivalent configured artifacts.
    """

    candidate = Path(path).expanduser()
    basename = candidate.name.casefold()
    if basename.startswith('test'):
        raise V4TrainerError(
            f'{context} basename is forbidden during V4 development: '
            f'{candidate.name!r}'
        )
    return candidate


def _read_json_object(path: Path) -> dict[str, Any]:
    assert_not_test_path(path, context='JSON configuration')
    try:
        parsed = json.loads(path.read_text(encoding='utf-8'))
    except FileNotFoundError as exc:
        raise V4TrainerError(f'JSON file does not exist: {path}') from exc
    except json.JSONDecodeError as exc:
        raise V4TrainerError(f'invalid JSON at {path}:{exc.lineno}') from exc
    if not isinstance(parsed, dict):
        raise V4TrainerError(f'expected a JSON object: {path}')
    return parsed


def _expand_environment_string(value: str, environment: Mapping[str, str]) -> str:
    def replacement(match: re.Match[str]) -> str:
        name = match.group(1) or match.group(2)
        resolved = environment.get(name)
        if resolved is None or str(resolved) == '':
            raise V4TrainerError(f'unresolved or empty environment variable: {name}')
        return str(resolved)

    expanded = ENVIRONMENT_PATTERN.sub(replacement, value)
    if ENVIRONMENT_PATTERN.search(expanded):
        raise V4TrainerError(f'unresolved environment expression: {expanded!r}')
    return expanded


def _expand_environment_value(value: Any, environment: Mapping[str, str]) -> Any:
    if isinstance(value, str):
        return _expand_environment_string(value, environment)
    if isinstance(value, list):
        return [_expand_environment_value(item, environment) for item in value]
    if isinstance(value, dict):
        return {
            str(key): _expand_environment_value(child, environment)
            for key, child in value.items()
        }
    return value


def _resolve_config_paths(config: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(config)

    def visit(value: Any, trail: tuple[str, ...]) -> Any:
        if isinstance(value, dict):
            return {
                key: visit(child, trail + (str(key),))
                for key, child in value.items()
            }
        if trail and trail[-1] in PATH_FIELD_NAMES:
            if not isinstance(value, str) or not value.strip():
                raise V4TrainerError(
                    f'configured path {".".join(trail)} must be a non-empty string'
                )
            candidate = assert_not_test_path(
                Path(value).expanduser(),
                context='configured path',
            ).resolve()
            return str(candidate)
        return value

    return visit(result, ())


def load_v4_config(
    path: Path | str = DEFAULT_CONFIG_PATH,
    *,
    environ: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Load, expand, resolve and validate the V4 JSON configuration."""

    config_path = assert_not_test_path(path, context='configuration path').resolve()
    raw = _read_json_object(config_path)
    defaults = raw.get('environment_defaults')
    if not isinstance(defaults, dict):
        raise V4TrainerError('environment_defaults must be a JSON object')
    effective_environment = {
        str(key): str(value)
        for key, value in defaults.items()
        if isinstance(key, str) and isinstance(value, (str, int, float))
    }
    if len(effective_environment) != len(defaults):
        raise V4TrainerError('environment_defaults must contain scalar string keys')
    supplied_environment = os.environ if environ is None else environ
    effective_environment.update({
        str(key): str(value)
        for key, value in supplied_environment.items()
    })
    expanded = _expand_environment_value(raw, effective_environment)
    config = _resolve_config_paths(expanded)
    _validate_config(config)
    return config


def _positive_int(value: Any, context: str) -> int:
    if isinstance(value, bool):
        raise V4TrainerError(f'{context} must be a positive integer')
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise V4TrainerError(f'{context} must be a positive integer') from exc
    if parsed <= 0 or float(value) != float(parsed):
        raise V4TrainerError(f'{context} must be a positive integer')
    return parsed


def _finite_float(value: Any, context: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise V4TrainerError(f'{context} must be numeric') from exc
    if not math.isfinite(parsed):
        raise V4TrainerError(f'{context} must be finite')
    return parsed


def _validate_source_spec(spec: Any, context: str) -> None:
    if not isinstance(spec, dict):
        raise V4TrainerError(f'{context} must be an object')
    for name in ('name', 'dataset_root', 'rows', 'labels', 'expected_rows'):
        if name not in spec:
            raise V4TrainerError(f'{context} is missing {name}')
    _positive_int(spec['expected_rows'], f'{context}.expected_rows')
    for name in ('dataset_root', 'rows', 'labels'):
        assert_not_test_path(spec[name], context=f'{context}.{name}')


def _validate_config(config: dict[str, Any]) -> None:
    if config.get('schema_version') != CONFIG_SCHEMA_VERSION:
        raise V4TrainerError(
            f'configuration schema must be {CONFIG_SCHEMA_VERSION!r}'
        )
    data = config.get('data')
    if not isinstance(data, dict):
        raise V4TrainerError('data must be an object')
    train_sources = data.get('train_sources')
    if not isinstance(train_sources, list) or not train_sources:
        raise V4TrainerError('data.train_sources must be a non-empty list')
    enabled_names: set[str] = set()
    fractions = []
    for index, source in enumerate(train_sources):
        context = f'data.train_sources[{index}]'
        _validate_source_spec(source, context)
        name = str(source['name']).strip()
        if not name or name in enabled_names:
            raise V4TrainerError(f'duplicate or empty training source name: {name!r}')
        if bool(source.get('enabled', True)):
            enabled_names.add(name)
            fraction = _finite_float(source.get('epoch_fraction'), f'{context}.epoch_fraction')
            if fraction <= 0.0:
                raise V4TrainerError('enabled source epoch_fraction must be positive')
            fractions.append(fraction)
    if not fractions or not math.isclose(sum(fractions), 1.0, abs_tol=1e-9):
        raise V4TrainerError('enabled training epoch fractions must sum exactly to one')
    _validate_source_spec(data.get('validation'), 'data.validation')
    _validate_source_spec(data.get('canary'), 'data.canary')

    roles = config.get('data_roles')
    if not isinstance(roles, dict) or roles.get('checkpoint_selection') != 'validation_only':
        raise V4TrainerError('checkpoint selection must be validation_only')
    if 'test' in data:
        raise V4TrainerError('test data is forbidden and must be absent')

    model = config.get('model')
    if not isinstance(model, dict) or model.get('kind') != V4_MODEL_KIND:
        raise V4TrainerError(f'model.kind must be {V4_MODEL_KIND!r}')
    if tuple(model.get('slot_order') or ()) != tuple(V4_SLOT_ORDER):
        raise V4TrainerError('model.slot_order does not match the fixed V4 fleet')
    if bool(model.get('cross_camera_feature_path')):
        raise V4TrainerError('cross_camera_feature_path must remain false')

    preprocessing = config.get('image_preprocessing')
    if not isinstance(preprocessing, dict):
        raise V4TrainerError('image_preprocessing must be an object')
    width = _positive_int(preprocessing.get('width'), 'image_preprocessing.width')
    height = _positive_int(preprocessing.get('height'), 'image_preprocessing.height')
    if (width, height) != TARGET_IMAGE_SIZE or width * 3 != height * 4:
        raise V4TrainerError('V4 preprocessing must resize to aspect-preserving 320x240')
    augmentations = preprocessing.get('train_augmentations')
    if not isinstance(augmentations, dict):
        raise V4TrainerError('train_augmentations must be an object')
    if augmentations.get('horizontal_flip') is not False:
        raise V4TrainerError('horizontal flip is forbidden for side-isolated V4')
    if augmentations.get('camera_swap') is not False:
        raise V4TrainerError('camera swap is forbidden for side-isolated V4')

    initialization = config.get('initialization')
    if not isinstance(initialization, dict):
        raise V4TrainerError('initialization must be an object')
    if initialization.get('kind') != 'verified_v3_backbone_only':
        raise V4TrainerError('initialization must be verified_v3_backbone_only')
    digest = str(initialization.get('checkpoint_sha256') or '').casefold()
    if not re.fullmatch(r'[0-9a-f]{64}', digest):
        raise V4TrainerError('initialization checkpoint_sha256 must be a SHA-256')
    assert_not_test_path(initialization.get('checkpoint', ''), context='initial checkpoint')
    if bool(initialization.get('copy_v3_joint_head')):
        raise V4TrainerError('the V3 joint head must never enter V4')

    baseline = config.get('approved_v3_validation_baseline')
    if not isinstance(baseline, dict):
        raise V4TrainerError('approved_v3_validation_baseline must be an object')
    if baseline.get('checkpoint_sha256') != digest:
        raise V4TrainerError(
            'approved V3 validation baseline checkpoint differs from initialization'
        )
    for name in ('validation_rows_sha256', 'validation_labels_sha256'):
        if not re.fullmatch(r'[0-9a-f]{64}', str(baseline.get(name) or '').casefold()):
            raise V4TrainerError(f'approved V3 baseline {name} must be a SHA-256')
    sample_count = _positive_int(
        baseline.get('sample_count'), 'approved V3 baseline sample_count'
    )
    visible_count = _positive_int(
        baseline.get('visible_count'), 'approved V3 baseline visible_count'
    )
    loaded_correct_count = _positive_int(
        baseline.get('loaded_correct_count'),
        'approved V3 baseline loaded_correct_count',
    )
    if loaded_correct_count > visible_count:
        raise V4TrainerError('approved V3 loaded correct count exceeds visible count')
    loaded_accuracy = _finite_float(
        baseline.get('loaded_accuracy'), 'approved V3 baseline loaded_accuracy'
    )
    if not math.isclose(
        loaded_accuracy,
        loaded_correct_count / visible_count,
        rel_tol=0.0,
        abs_tol=1.0e-15,
    ):
        raise V4TrainerError('approved V3 loaded accuracy conflicts with exact counts')
    if sample_count != int(config['data']['validation']['expected_rows']):
        raise V4TrainerError('approved V3 baseline sample count differs from validation')
    if baseline.get('data_role') != 'validation_only':
        raise V4TrainerError('approved V3 baseline must be validation_only')

    topology_contract = config.get('topology_contract')
    if not isinstance(topology_contract, dict):
        raise V4TrainerError('topology_contract must be an object')
    expected_topology_contract = {
        'segment_name_domain': 'public_ros_label',
        'length_loader': 'room_315_rail_defaults.public_rail_segment_lengths',
        'forbid_internal_rail_segment_lengths': True,
        'store_length_mapping_fingerprint_in_checkpoint': True,
        'verify_length_mapping_fingerprint_at_runtime': True,
    }
    for name, expected_value in expected_topology_contract.items():
        if topology_contract.get(name) != expected_value:
            raise V4TrainerError(
                f'topology_contract.{name} must be {expected_value!r}'
            )

    training = config.get('training')
    if not isinstance(training, dict):
        raise V4TrainerError('training must be an object')
    for name in ('batch_size', 'epochs', 'gradient_accumulation_steps'):
        _positive_int(training.get(name), f'training.{name}')
    if _finite_float(training.get('amp_initial_scale'), 'training.amp_initial_scale') <= 0.0:
        raise V4TrainerError('training.amp_initial_scale must be positive')
    _positive_int(training.get('early_stopping_patience'), 'training.early_stopping_patience')
    selection = training.get('checkpoint_selection')
    expected_selection = [
        'highest_validation_planning_score',
        'highest_validation_worst_side_x_segment_recall',
        'lowest_validation_correct_segment_s_m_p95',
    ]
    if selection != expected_selection:
        raise V4TrainerError('checkpoint selection must use the frozen lexicographic order')
    sampling = config.get('sampling')
    if not isinstance(sampling, dict):
        raise V4TrainerError('sampling must be an object')
    _positive_int(
        sampling.get('records_per_source_per_epoch'),
        'sampling.records_per_source_per_epoch',
    )
    evaluation = config.get('evaluation')
    if not isinstance(evaluation, dict):
        raise V4TrainerError('evaluation must be an object')
    expected_planning_components = {
        'segment_macro_f1',
        'minimum_side_top1',
        'worst_side_x_segment_recall',
        'joint_segment_and_ratio_005',
        'switch_zone_segment_top1',
        'boundary_zone_segment_top1',
    }
    raw_planning_weights = evaluation.get('planning_score_weights')
    if not isinstance(raw_planning_weights, dict) or set(raw_planning_weights) != expected_planning_components:
        raise V4TrainerError(
            'evaluation.planning_score_weights must declare the frozen planning components'
        )
    planning_weights = {
        name: _finite_float(value, f'evaluation.planning_score_weights.{name}')
        for name, value in raw_planning_weights.items()
    }
    if any(value < 0.0 for value in planning_weights.values()) or not math.isclose(
        sum(planning_weights.values()), 1.0, abs_tol=1.0e-9
    ):
        raise V4TrainerError(
            'evaluation.planning_score_weights must be non-negative and sum to one'
        )
    if evaluation.get('calibrate_segment_temperature_on_validation') is not True:
        raise V4TrainerError('validation segment calibration must remain enabled')
    if evaluation.get('report_selective_accuracy_and_coverage') is not True:
        raise V4TrainerError('selective accuracy/coverage reporting must remain enabled')
    calibration = evaluation.get('segment_calibration')
    if not isinstance(calibration, dict):
        raise V4TrainerError('evaluation.segment_calibration must be an object')
    coverage_targets = calibration.get('coverage_targets')
    if not isinstance(coverage_targets, list) or not coverage_targets:
        raise V4TrainerError('segment calibration coverage_targets must be non-empty')
    parsed_coverages = [
        _finite_float(value, 'segment calibration coverage target')
        for value in coverage_targets
    ]
    if any(not 0.0 < value <= 1.0 for value in parsed_coverages):
        raise V4TrainerError('segment calibration coverages must be in (0, 1]')
    _positive_int(calibration.get('ece_bins'), 'segment calibration ece_bins')
    _positive_int(calibration.get('grid_size'), 'segment calibration grid_size')
    if int(calibration.get('grid_size')) < 3:
        raise V4TrainerError('segment calibration grid_size must be at least three')
    minimum_temperature = _finite_float(
        calibration.get('minimum_temperature'), 'minimum calibration temperature'
    )
    maximum_temperature = _finite_float(
        calibration.get('maximum_temperature'), 'maximum calibration temperature'
    )
    if not (
        0.0 < minimum_temperature <= 1.0 <= maximum_temperature
        and minimum_temperature < maximum_temperature
    ):
        raise V4TrainerError('segment calibration bounds must contain 1.0')
    refinements = calibration.get('refinement_steps')
    if isinstance(refinements, bool) or not isinstance(refinements, int) or refinements < 0:
        raise V4TrainerError('segment calibration refinement_steps must be non-negative')
    gates = config.get('pilot_acceptance_gates')
    if not isinstance(gates, dict):
        raise V4TrainerError('pilot_acceptance_gates must be an object')
    if 'maximum_s_m_mae_m' in gates:
        raise V4TrainerError(
            'cross-segment s_m MAE is not a valid acceptance gate'
        )
    for name in (
        'maximum_correct_segment_s_m_mae_m',
        'maximum_correct_segment_s_m_mae_m_each_side',
        'minimum_correct_segment_s_m_coverage_each_side',
        'minimum_joint_segment_and_ratio_005_accuracy',
        'minimum_joint_segment_and_ratio_005_accuracy_each_side',
        'minimum_segment_top1_each_side',
        'minimum_segment_macro_recall_each_side',
        'minimum_worst_side_x_segment_recall',
        'minimum_identity_zone_switch_segment_top1',
        'minimum_identity_zone_boundary_segment_top1',
        'minimum_position_bin_segment_top1',
        'minimum_position_bin_joint_segment_and_ratio_005_accuracy',
        'minimum_scene_occlusion_segment_top1',
        'minimum_scene_occlusion_joint_segment_and_ratio_005_accuracy',
        'minimum_scene_presence_density_segment_top1',
        'minimum_scene_presence_density_joint_segment_and_ratio_005_accuracy',
        'minimum_loaded_accuracy',
        'maximum_loaded_accuracy_drop',
        'cross_side_output_change_tolerance',
        'minimum_camera_order_swap_segment_top1_drop',
    ):
        if name not in gates:
            raise V4TrainerError(f'pilot_acceptance_gates is missing {name}')
        _finite_float(gates[name], f'pilot_acceptance_gates.{name}')
    for name in (
        'zero_supported_segment_classes_with_zero_recall',
        'zero_supported_side_x_segment_cells_with_zero_recall',
    ):
        if gates.get(name) is not True:
            raise V4TrainerError(f'pilot_acceptance_gates.{name} must be true')
    if gates.get('automatic_runtime_switch') is not False:
        raise V4TrainerError('automatic runtime switch must remain disabled')
    assert_not_test_path(config.get('output_root', ''), context='output_root')


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    assert_not_test_path(path, context='JSONL artifact')
    if not path.is_file():
        raise V4TrainerError(f'JSONL artifact does not exist: {path}')
    rows: list[dict[str, Any]] = []
    with path.open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                raise V4TrainerError(f'blank JSONL row at {path}:{line_number}')
            try:
                parsed = json.loads(line)
            except json.JSONDecodeError as exc:
                raise V4TrainerError(f'invalid JSONL at {path}:{line_number}') from exc
            if not isinstance(parsed, dict):
                raise V4TrainerError(f'non-object JSONL row at {path}:{line_number}')
            rows.append(parsed)
    return rows


def _explicit_sample_id(row: Mapping[str, Any], context: str) -> str:
    sample_id = str(row.get('sample_id') or '').strip()
    if not sample_id:
        raise V4TrainerError(f'{context} is missing explicit sample_id')
    return sample_id


def _resolved_image_path(dataset_root: Path, reference: Any, context: str) -> Path:
    text = str(reference or '').strip()
    if not text:
        raise V4TrainerError(f'{context} image reference is empty')
    raw = Path(text).expanduser()
    candidate = (raw if raw.is_absolute() else dataset_root / raw).resolve()
    assert_not_test_path(candidate, context=f'{context} image')
    try:
        candidate.relative_to(dataset_root)
    except ValueError as exc:
        raise V4TrainerError(f'{context} image escapes dataset_root: {candidate}') from exc
    if not candidate.is_file():
        raise V4TrainerError(f'{context} image does not exist: {candidate}')
    return candidate


def _validate_image_size(
    size: Sequence[int],
    *,
    path: Path,
    context: str,
) -> tuple[int, int]:
    width, height = int(size[0]), int(size[1])
    if width <= 0 or height <= 0 or width * 3 != height * 4:
        raise V4TrainerError(
            f'{context} image must have a positive 4:3 size: {path} is {width}x{height}'
        )
    return width, height


def _validate_image_file(path: Path, context: str) -> tuple[int, int]:
    try:
        with Image.open(path) as opened:
            opened.load()
            size = opened.size
    except Exception as exc:
        raise V4TrainerError(f'{context} image cannot be decoded: {path}') from exc
    return _validate_image_size(size, path=path, context=context)


def load_paired_source(
    spec: Mapping[str, Any],
    *,
    role: str,
    decode_images: bool = False,
) -> tuple[list[PairedRecord], dict[str, Any]]:
    """Pair one configured row/label source by exact ``sample_id`` sets."""

    if role.casefold().startswith('test'):
        raise V4TrainerError('test role is forbidden')
    _validate_source_spec(spec, f'data.{role}')
    source = str(spec['name']).strip()
    dataset_root = assert_not_test_path(
        spec['dataset_root'], context=f'{source}.dataset_root'
    ).resolve()
    rows_path = assert_not_test_path(spec['rows'], context=f'{source}.rows').resolve()
    labels_path = assert_not_test_path(spec['labels'], context=f'{source}.labels').resolve()
    if not dataset_root.is_dir():
        raise V4TrainerError(f'dataset_root is not a directory: {dataset_root}')
    rows = _read_jsonl(rows_path)
    labels = _read_jsonl(labels_path)
    expected = _positive_int(spec['expected_rows'], f'{source}.expected_rows')
    if len(rows) != expected or len(labels) != expected:
        raise V4TrainerError(
            f'{source} expected {expected} rows and labels; '
            f'found rows={len(rows)}, labels={len(labels)}'
        )

    row_by_id: dict[str, dict[str, Any]] = {}
    for index, row in enumerate(rows):
        sample_id = _explicit_sample_id(row, f'{source}.rows[{index}]')
        if sample_id in row_by_id:
            raise V4TrainerError(f'duplicate row sample_id in {source}: {sample_id}')
        row_by_id[sample_id] = row
    label_by_id: dict[str, dict[str, Any]] = {}
    for index, label in enumerate(labels):
        sample_id = _explicit_sample_id(label, f'{source}.labels[{index}]')
        if sample_id in label_by_id:
            raise V4TrainerError(f'duplicate label sample_id in {source}: {sample_id}')
        label_by_id[sample_id] = label
    if set(row_by_id) != set(label_by_id):
        missing = sorted(set(row_by_id) - set(label_by_id))[:5]
        extra = sorted(set(label_by_id) - set(row_by_id))[:5]
        raise V4TrainerError(
            f'{source} row/label sample_id sets differ; missing={missing}, extra={extra}'
        )

    result: list[PairedRecord] = []
    decoded = 0
    for index, row in enumerate(rows):
        sample_id = _explicit_sample_id(row, f'{source}.rows[{index}]')
        try:
            model_input = validate_visual_model_input(
                row, context=f'{source}:{sample_id}'
            )
        except Exception as exc:
            raise V4TrainerError(
                f'{source}:{sample_id} has invalid camera-only model input: {exc}'
            ) from exc
        references = model_input.get('overhead_images')
        if not isinstance(references, dict) or set(references) != set(CAMERAS):
            raise V4TrainerError(
                f'{source}:{sample_id} must contain exactly the left/right camera pair'
            )
        image_paths = {
            camera: _resolved_image_path(
                dataset_root,
                references[camera],
                f'{source}:{sample_id}:{camera}',
            )
            for camera in CAMERAS
        }
        if decode_images:
            for camera, image_path in image_paths.items():
                _validate_image_file(image_path, f'{source}:{sample_id}:{camera}')
                decoded += 1
        label = label_by_id[sample_id]
        try:
            normalized = normalize_visual_state_labels(
                label,
                context=f'{source}:{sample_id}',
            )
        except Exception as exc:
            raise V4TrainerError(
                f'{source}:{sample_id} visual label is invalid: {exc}'
            ) from exc
        result.append(PairedRecord(
            sample_id=sample_id,
            source=source,
            role=role,
            dataset_root=dataset_root,
            row=row,
            label=label,
            normalized_label=normalized,
            image_paths=image_paths,
            trace=dict(row.get('traceability_metadata') or {}),
        ))
    return result, {
        'name': source,
        'role': role,
        'expected_records': expected,
        'paired_records': len(result),
        'image_references': len(result) * len(CAMERAS),
        'decoded_images': decoded,
        'rows': file_fingerprint(rows_path),
        'labels': file_fingerprint(labels_path),
        'dataset_root': str(dataset_root),
    }


def _image_sizes_by_camera(
    value: Sequence[int] | Mapping[str, Sequence[int]],
) -> dict[str, tuple[int, int]]:
    if isinstance(value, Mapping):
        raw = {camera: value.get(camera) for camera in CAMERAS}
    else:
        raw = {camera: value for camera in CAMERAS}
    parsed: dict[str, tuple[int, int]] = {}
    for camera, size in raw.items():
        if (
            not isinstance(size, Sequence)
            or isinstance(size, (str, bytes))
            or len(size) != 2
        ):
            raise V4TrainerError(f'{camera} original image size must be (width, height)')
        width = _positive_int(size[0], f'{camera}.width')
        height = _positive_int(size[1], f'{camera}.height')
        if width * 3 != height * 4:
            raise V4TrainerError(f'{camera} original image size must have 4:3 aspect')
        parsed[camera] = (width, height)
    return parsed


def extract_structured_targets(
    label: Mapping[str, Any],
    *,
    image_sizes: Sequence[int] | Mapping[str, Sequence[int]] = DEFAULT_ORIGINAL_IMAGE_SIZE,
    already_normalized: bool = False,
) -> dict[str, np.ndarray]:
    """Create fixed-order structured V4 targets and masks directly from labels."""

    try:
        normalized = (
            dict(label)
            if already_normalized
            else normalize_visual_state_labels(dict(label), context='V4 target')
        )
    except Exception as exc:
        raise V4TrainerError(f'cannot normalize V4 visual targets: {exc}') from exc
    shuttles = normalized.get('shuttles')
    if not isinstance(shuttles, list) or len(shuttles) != len(FIXED_IDENTITIES):
        raise V4TrainerError('normalized labels must contain the fixed eight shuttle slots')
    sizes = _image_sizes_by_camera(image_sizes)
    segment = np.full((8,), -100, dtype=np.int64)
    loaded = np.full((8,), -100, dtype=np.int64)
    bbox = np.zeros((8, 4), dtype=np.float32)
    s_ratio = np.zeros((8, 1), dtype=np.float32)
    s_m = np.zeros((8, 1), dtype=np.float32)
    segment_length_m = np.zeros((8, 1), dtype=np.float32)
    segment_mask = np.zeros((8,), dtype=np.bool_)
    loaded_mask = np.zeros((8,), dtype=np.bool_)
    bbox_mask = np.zeros((8, 4), dtype=np.bool_)
    s_ratio_mask = np.zeros((8,), dtype=np.bool_)

    for slot, (identity, shuttle) in enumerate(zip(FIXED_IDENTITIES, shuttles)):
        actual_identity = str(shuttle.get('id') or '').strip().upper()
        if actual_identity != identity:
            raise V4TrainerError(
                f'normalized shuttle order mismatch at slot {slot}: '
                f'{actual_identity!r} != {identity!r}'
            )
        visible = bool(
            shuttle.get('presence') and shuttle.get('visually_available')
        )
        if not visible:
            continue
        side = derive_side(identity)
        location = shuttle.get('location') or {}
        if str(location.get('side') or '').strip().lower() != side:
            raise V4TrainerError(f'{identity} label side conflicts with fixed identity')
        segment_name = str(location.get('block') or '').strip().upper()
        loaded_name = str(shuttle.get('loaded_state') or '').strip().lower()
        if segment_name not in SEGMENT_CLASSES:
            raise V4TrainerError(f'{identity} has unsupported segment {segment_name!r}')
        if loaded_name not in LOADED_CLASSES:
            raise V4TrainerError(f'{identity} has unsupported loaded state {loaded_name!r}')
        camera = f'{side}_rail_rgb'
        width, height = sizes[camera]
        raw_bbox = np.asarray(shuttle.get('bbox'), dtype=np.float64)
        if raw_bbox.shape != (4,) or not np.isfinite(raw_bbox).all():
            raise V4TrainerError(f'{identity} bbox is not four finite values')
        normalized_bbox = raw_bbox / np.asarray(
            [width, height, width, height], dtype=np.float64
        )
        x, y, box_width, box_height = normalized_bbox.tolist()
        tolerance = 1.0e-6
        if (
            x < -tolerance
            or y < -tolerance
            or box_width <= 0.0
            or box_height <= 0.0
            or x + box_width > 1.0 + tolerance
            or y + box_height > 1.0 + tolerance
        ):
            raise V4TrainerError(
                f'{identity} bbox falls outside its {width}x{height} source image'
            )
        position = shuttle.get('rail_position') or {}
        ratio = _finite_float(position.get('s_ratio'), f'{identity}.s_ratio')
        length = _finite_float(
            position.get('segment_length_m'), f'{identity}.segment_length_m'
        )
        distance = _finite_float(position.get('s_m'), f'{identity}.s_m')
        if not 0.0 <= ratio <= 1.0 or length <= 0.0:
            raise V4TrainerError(f'{identity} rail position target is invalid')

        segment[slot] = SEGMENT_CLASSES.index(segment_name)
        loaded[slot] = LOADED_CLASSES.index(loaded_name)
        bbox[slot] = np.clip(normalized_bbox, 0.0, 1.0).astype(np.float32)
        s_ratio[slot, 0] = np.float32(ratio)
        segment_length_m[slot, 0] = np.float32(length)
        s_m[slot, 0] = np.float32(distance)
        segment_mask[slot] = True
        loaded_mask[slot] = True
        bbox_mask[slot, :] = True
        s_ratio_mask[slot] = True

    visibility_mask = (
        segment_mask & loaded_mask & bbox_mask.all(axis=-1) & s_ratio_mask
    )
    return {
        'segment': segment,
        'loaded': loaded,
        'bbox': bbox,
        's_ratio': s_ratio,
        'visibility_mask': visibility_mask,
        'segment_mask': segment_mask,
        'loaded_mask': loaded_mask,
        'bbox_mask': bbox_mask,
        's_ratio_mask': s_ratio_mask,
        'segment_length_m': segment_length_m,
        's_m': s_m,
    }


def _visible_side_segments(record: PairedRecord) -> tuple[tuple[str, str], ...]:
    values = []
    for identity, shuttle in zip(FIXED_IDENTITIES, record.normalized_label['shuttles']):
        if shuttle.get('presence') and shuttle.get('visually_available'):
            segment = str((shuttle.get('location') or {}).get('block') or '').upper()
            if segment not in SEGMENT_CLASSES:
                raise V4TrainerError(
                    f'{record.source}:{record.sample_id}:{identity} has invalid segment'
                )
            values.append((derive_side(identity), segment))
    return tuple(values)


def _trace_weight_multiplier(record: PairedRecord, sampling: Mapping[str, Any]) -> float:
    trace = record.trace
    zones = {
        str(value).strip().lower()
        for value in (trace.get('identity_to_zone') or {}).values()
    }
    target_zone = str(trace.get('target_zone') or '').strip().lower()
    tags = {str(value).strip().lower() for value in trace.get('hard_case_tags') or []}
    relation = str(trace.get('relation_family') or '').strip().lower()
    multiplier = 1.0
    if target_zone == 'boundary' or 'boundary' in zones:
        multiplier *= _finite_float(
            sampling.get('boundary_multiplier', 1.0), 'sampling.boundary_multiplier'
        )
    if (
        target_zone == 'switch'
        or 'switch' in zones
        or any('switch' in tag for tag in tags)
    ):
        multiplier *= _finite_float(
            sampling.get('switch_multiplier', 1.0), 'sampling.switch_multiplier'
        )
    if relation == 'multi_blocker' or any('multi_blocker' in tag for tag in tags):
        multiplier *= _finite_float(
            sampling.get('multi_blocker_multiplier', 1.0),
            'sampling.multi_blocker_multiplier',
        )
    return multiplier


def compute_sampling_weights(
    records: Sequence[PairedRecord],
    sampling: Mapping[str, Any],
) -> dict[str, float]:
    """Compute source-local side x segment and trace-aware sample weights."""

    if not records:
        raise V4TrainerError('cannot weight an empty source')
    identifiers = [record.sample_id for record in records]
    if len(set(identifiers)) != len(identifiers):
        raise V4TrainerError('sampling records contain duplicate sample_id values')
    pairs_by_id = {
        record.sample_id: _visible_side_segments(record) for record in records
    }
    frequencies = Counter(
        pair for pairs in pairs_by_id.values() for pair in pairs
    )
    maximum = max(frequencies.values(), default=1)
    raw_balance: dict[str, float] = {}
    for record in records:
        pairs = pairs_by_id[record.sample_id]
        raw_balance[record.sample_id] = (
            sum(math.sqrt(maximum / frequencies[pair]) for pair in pairs) / len(pairs)
            if pairs
            else 1.0
        )
    mean_balance = sum(raw_balance.values()) / len(raw_balance)
    minimum = _finite_float(sampling.get('sample_weight_min'), 'sample_weight_min')
    maximum_weight = _finite_float(
        sampling.get('sample_weight_max'), 'sample_weight_max'
    )
    if minimum <= 0.0 or maximum_weight < minimum:
        raise V4TrainerError('sample weight bounds are invalid')
    weights = {}
    for record in records:
        value = raw_balance[record.sample_id] / mean_balance
        value *= _trace_weight_multiplier(record, sampling)
        weights[record.sample_id] = min(maximum_weight, max(minimum, value))
    return weights


def _source_quotas(
    fractions: Mapping[str, float],
    *,
    records_per_source: int,
) -> dict[str, int]:
    names = sorted(fractions)
    if not names:
        raise V4TrainerError('source fractions are empty')
    total_fraction = sum(fractions.values())
    if not math.isclose(total_fraction, 1.0, abs_tol=1e-9):
        raise V4TrainerError('source fractions must sum to one')
    epoch_size = records_per_source * len(names)
    exact = {name: epoch_size * fractions[name] for name in names}
    quotas = {name: int(math.floor(value)) for name, value in exact.items()}
    remainder = epoch_size - sum(quotas.values())
    order = sorted(names, key=lambda name: (-(exact[name] - quotas[name]), name))
    for name in order[:remainder]:
        quotas[name] += 1
    return quotas


def build_epoch_selection(
    records_by_source: Mapping[str, Sequence[PairedRecord]],
    train_source_specs: Sequence[Mapping[str, Any]],
    sampling: Mapping[str, Any],
    *,
    seed: int,
    epoch: int,
) -> list[dict[str, Any]]:
    """Return a deterministic source-balanced weighted-resampling plan."""

    if epoch <= 0:
        raise V4TrainerError('epoch must be positive')
    fractions = {
        str(spec['name']): _finite_float(spec['epoch_fraction'], 'epoch_fraction')
        for spec in train_source_specs
        if bool(spec.get('enabled', True))
    }
    if set(fractions) != set(records_by_source):
        raise V4TrainerError('enabled source specs and loaded train sources differ')
    quotas = _source_quotas(
        fractions,
        records_per_source=_positive_int(
            sampling.get('records_per_source_per_epoch'),
            'records_per_source_per_epoch',
        ),
    )
    plan: list[dict[str, Any]] = []
    for source in sorted(records_by_source):
        records = list(records_by_source[source])
        if not records:
            raise V4TrainerError(f'training source is empty: {source}')
        weights = compute_sampling_weights(records, sampling)
        digest = hashlib.sha256(f'{seed}:{epoch}:{source}'.encode()).digest()
        source_seed = int.from_bytes(digest[:8], 'big')
        rng = random.Random(source_seed)
        indexes = rng.choices(
            range(len(records)),
            weights=[weights[record.sample_id] for record in records],
            k=quotas[source],
        )
        occurrence_by_sample: Counter[str] = Counter()
        for draw_index, source_index in enumerate(indexes):
            record = records[source_index]
            occurrence = occurrence_by_sample[record.sample_id]
            occurrence_by_sample[record.sample_id] += 1
            plan.append({
                'source': source,
                'sample_id': record.sample_id,
                'source_index': int(source_index),
                'draw_index': draw_index,
                'resample_occurrence': occurrence,
                'sample_weight': weights[record.sample_id],
            })
    shuffle_seed = int.from_bytes(
        hashlib.sha256(f'{seed}:{epoch}:interleave'.encode()).digest()[:8],
        'big',
    )
    random.Random(shuffle_seed).shuffle(plan)
    return plan


def compute_train_class_counts(
    records_by_source: Mapping[str, Sequence[PairedRecord]],
) -> dict[str, Any]:
    """Count segment classes from train records only, separated by rail side."""

    counts = np.zeros((2, len(SEGMENT_CLASSES)), dtype=np.int64)
    sample_count = 0
    for source, records in records_by_source.items():
        if source.casefold().startswith(('validation', 'canary', 'test')):
            raise V4TrainerError(f'non-train source entered class counts: {source}')
        for record in records:
            if record.role != 'train':
                raise V4TrainerError(
                    f'non-train record entered class counts: {record.role}'
                )
            sample_count += 1
            for side, segment in _visible_side_segments(record):
                counts[SIDE_INDEX[side], SEGMENT_CLASSES.index(segment)] += 1
    if sample_count == 0 or int(counts.sum()) == 0:
        raise V4TrainerError('train-only class counts are empty')
    missing = [
        f'{side}.{segment}'
        for side, side_index in SIDE_INDEX.items()
        for segment, count in zip(SEGMENT_CLASSES, counts[side_index])
        if int(count) == 0
    ]
    if missing:
        raise V4TrainerError(
            f'train side x segment coverage contains zero-support cells: {missing}'
        )
    return {
        'provenance': 'train_only',
        'train_sample_count': sample_count,
        'slot_target_count': int(counts.sum()),
        'segment_order': list(SEGMENT_CLASSES),
        'by_side': {
            side: counts[index].tolist() for side, index in SIDE_INDEX.items()
        },
        'aggregate': counts.sum(axis=0).tolist(),
        'zero_support_side_x_segment': [],
        'all_side_x_segment_cells_supported': True,
    }


def segment_support_report(
    records: Sequence[PairedRecord],
    *,
    role: str,
) -> dict[str, Any]:
    counts = np.zeros((2, len(SEGMENT_CLASSES)), dtype=np.int64)
    for record in records:
        if record.role != role:
            raise V4TrainerError(
                f'{role} support audit received record role {record.role!r}'
            )
        for side, segment in _visible_side_segments(record):
            counts[SIDE_INDEX[side], SEGMENT_CLASSES.index(segment)] += 1
    missing = [
        f'{side}.{segment}'
        for side, side_index in SIDE_INDEX.items()
        for segment, count in zip(SEGMENT_CLASSES, counts[side_index])
        if int(count) == 0
    ]
    return {
        'role': role,
        'segment_order': list(SEGMENT_CLASSES),
        'by_side': {
            side: counts[index].tolist() for side, index in SIDE_INDEX.items()
        },
        'zero_support_side_x_segment': missing,
        'all_side_x_segment_cells_supported': not missing,
        'macro_gate_interpretation_valid': not missing,
    }


def require_full_segment_support(report: Mapping[str, Any]) -> None:
    """Reject evaluation splits that cannot support side-by-class gates."""

    missing = list(report.get('zero_support_side_x_segment') or ())
    if missing or report.get('all_side_x_segment_cells_supported') is not True:
        role = str(report.get('role') or 'evaluation')
        raise V4TrainerError(
            f'{role} side x segment coverage contains zero-support cells: {missing}'
        )


def verify_approved_v3_validation_baseline(
    config: Mapping[str, Any],
    validation_fingerprint: Mapping[str, Any],
    validation_support: Mapping[str, Any],
) -> dict[str, Any]:
    """Bind the V3 comparison number to the exact locked validation inputs."""

    baseline = config['approved_v3_validation_baseline']
    checks = {
        'checkpoint_sha256': (
            baseline['checkpoint_sha256']
            == config['initialization']['checkpoint_sha256']
        ),
        'validation_rows_sha256': (
            baseline['validation_rows_sha256']
            == validation_fingerprint['rows']['sha256']
        ),
        'validation_labels_sha256': (
            baseline['validation_labels_sha256']
            == validation_fingerprint['labels']['sha256']
        ),
        'sample_count': (
            int(baseline['sample_count'])
            == int(validation_fingerprint['paired_records'])
        ),
        'visible_count': (
            int(baseline['visible_count'])
            == sum(
                int(count)
                for counts in validation_support['by_side'].values()
                for count in counts
            )
        ),
        'loaded_accuracy_exact_counts': math.isclose(
            float(baseline['loaded_accuracy']),
            int(baseline['loaded_correct_count']) / int(baseline['visible_count']),
            rel_tol=0.0,
            abs_tol=1.0e-15,
        ),
    }
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise V4TrainerError(
            f'approved V3 validation baseline provenance mismatch: {failed}'
        )
    return {
        **dict(baseline),
        'provenance_checks': checks,
        'verified': True,
    }


def load_and_verify_approved_v3_canary_baseline(
    baseline_path: Path | str,
    expected_sha256: str,
    config: Mapping[str, Any],
    canary_fingerprint: Mapping[str, Any],
    canary_support: Mapping[str, Any],
) -> dict[str, Any]:
    """Verify an immutable V3 Canary baseline without changing train config."""

    path = assert_not_test_path(
        baseline_path, context='approved V3 Canary baseline'
    ).resolve()
    expected_digest = str(expected_sha256).strip().casefold()
    if not re.fullmatch(r'[0-9a-f]{64}', expected_digest):
        raise V4TrainerError('approved V3 Canary baseline SHA-256 is required')
    actual_digest = sha256_file(path)
    if actual_digest != expected_digest:
        raise V4TrainerError(
            'approved V3 Canary baseline artifact SHA-256 mismatch: '
            f'{actual_digest} != {expected_digest}'
        )
    baseline = _read_json_object(path)
    baseline_checkpoint_path = assert_not_test_path(
        baseline.get('checkpoint_path', ''),
        context='approved V3 Canary checkpoint',
    ).resolve()
    baseline_checkpoint_sha = sha256_file(baseline_checkpoint_path)
    prior_baseline_path = assert_not_test_path(
        baseline.get('prior_canary_artifact_path', ''),
        context='V3 baseline prior Canary artifact',
    ).resolve()
    prior_baseline_sha = sha256_file(prior_baseline_path)
    by_side = canary_support.get('by_side')
    if not isinstance(by_side, Mapping):
        raise V4TrainerError('Canary support report lacks by_side counts')
    side_visible = {
        side: sum(int(count) for count in by_side.get(side, ()))
        for side in SIDES
    }
    visible_count = sum(side_visible.values())

    def matches_integer(name: str, expected: int) -> bool:
        try:
            value = baseline[name]
            return (
                isinstance(value, int)
                and not isinstance(value, bool)
                and value == expected
            )
        except KeyError:
            return False

    checks = {
        'schema_version': (
            baseline.get('schema_version') == V3_CANARY_BASELINE_SCHEMA_VERSION
        ),
        'checkpoint_sha256': (
            str(baseline.get('checkpoint_sha256') or '').casefold()
            == str(config['initialization']['checkpoint_sha256']).casefold()
        ),
        'checkpoint_path_sha256': (
            str(baseline.get('checkpoint_sha256') or '').casefold()
            == baseline_checkpoint_sha
        ),
        'canary_rows_sha256': (
            str(baseline.get('canary_rows_sha256') or '').casefold()
            == str(canary_fingerprint['rows']['sha256']).casefold()
        ),
        'canary_labels_sha256': (
            str(baseline.get('canary_labels_sha256') or '').casefold()
            == str(canary_fingerprint['labels']['sha256']).casefold()
        ),
        'sample_count': matches_integer(
            'sample_count', int(canary_fingerprint['paired_records'])
        ),
        'visible_count': matches_integer('visible_count', visible_count),
        'visible_left_count': matches_integer(
            'visible_left_count', side_visible['left']
        ),
        'visible_right_count': matches_integer(
            'visible_right_count', side_visible['right']
        ),
        'data_role': (
            baseline.get('data_role')
            == 'post_training_development_canary_baseline_only'
        ),
        'preprocessing_contract': (
            baseline.get('preprocessing_contract') == V3_PREPROCESSING_CONTRACT
        ),
        'used_for_selection': baseline.get('used_for_selection') is False,
        'test_loaded': baseline.get('test_loaded') is False,
        'prior_exposure_acknowledged': (
            baseline.get('prior_exposure_acknowledged') is True
        ),
        'prior_canary_artifact_sha256': (
            str(baseline.get('prior_canary_artifact_sha256') or '').casefold()
            == prior_baseline_sha
        ),
        'camera_order': baseline.get('camera_order') == 'left_then_right',
        'training_performed': baseline.get('training_performed') is False,
        'validation_reproduction_guard_passed': (
            baseline.get('validation_reproduction_guard_passed') is True
        ),
        'automatic_deployment_approval': (
            baseline.get('automatic_deployment_approval') is False
        ),
    }

    exact_ratios = (
        ('loaded_accuracy', 'loaded_correct_count', 'visible_count'),
        ('loaded_accuracy_left', 'loaded_correct_left', 'visible_left_count'),
        ('loaded_accuracy_right', 'loaded_correct_right', 'visible_right_count'),
        ('segment_top1_accuracy', 'segment_correct_count', 'visible_count'),
        ('segment_top1_left', 'segment_correct_left', 'visible_left_count'),
        ('segment_top1_right', 'segment_correct_right', 'visible_right_count'),
    )
    for accuracy_name, correct_name, count_name in exact_ratios:
        try:
            raw_correct = baseline[correct_name]
            raw_count = baseline[count_name]
            if (
                not isinstance(raw_correct, int)
                or isinstance(raw_correct, bool)
                or not isinstance(raw_count, int)
                or isinstance(raw_count, bool)
            ):
                raise TypeError('counts must be strict JSON integers')
            correct = raw_correct
            count = raw_count
            accuracy = float(baseline[accuracy_name])
        except (KeyError, TypeError, ValueError):
            checks[f'{accuracy_name}_exact_counts'] = False
            continue
        checks[f'{accuracy_name}_exact_counts'] = (
            count > 0
            and 0 <= correct <= count
            and math.isfinite(accuracy)
            and math.isclose(
                accuracy,
                correct / count,
                rel_tol=0.0,
                abs_tol=1.0e-15,
            )
        )
    try:
        checks['loaded_side_counts_sum'] = (
            int(baseline['loaded_correct_left'])
            + int(baseline['loaded_correct_right'])
            == int(baseline['loaded_correct_count'])
        )
        checks['segment_side_counts_sum'] = (
            int(baseline['segment_correct_left'])
            + int(baseline['segment_correct_right'])
            == int(baseline['segment_correct_count'])
        )
    except (KeyError, TypeError, ValueError):
        checks['loaded_side_counts_sum'] = False
        checks['segment_side_counts_sum'] = False
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise V4TrainerError(
            f'approved V3 Canary baseline provenance mismatch: {failed}'
        )
    return {
        **baseline,
        'artifact': file_fingerprint(path),
        'checkpoint_artifact': file_fingerprint(baseline_checkpoint_path),
        'prior_canary_artifact': file_fingerprint(prior_baseline_path),
        'provenance_checks': checks,
        'verified': True,
    }


def load_and_verify_canary_coverage_contract(
    contract_path: Path | str,
    expected_sha256: str,
    config: Mapping[str, Any],
    canary_fingerprint: Mapping[str, Any],
    training_run_root: Path,
) -> dict[str, Any]:
    """Verify the role-specific Canary coverage contract before inference."""

    path = assert_not_test_path(
        contract_path, context='Canary coverage contract'
    ).resolve()
    expected_digest = str(expected_sha256).strip().casefold()
    if not re.fullmatch(r'[0-9a-f]{64}', expected_digest):
        raise V4TrainerError('Canary coverage contract SHA-256 is required')
    actual_digest = sha256_file(path)
    if actual_digest != expected_digest:
        raise V4TrainerError(
            'Canary coverage contract artifact SHA-256 mismatch: '
            f'{actual_digest} != {expected_digest}'
        )
    contract = _read_json_object(path)
    finalization_path = assert_not_test_path(
        contract.get('canary_finalization_path', ''),
        context='Canary finalization artifact',
    ).resolve()
    finalization_sha = sha256_file(finalization_path)
    finalization = _read_json_object(finalization_path)
    prior_artifact_path = assert_not_test_path(
        contract.get('prior_canary_artifact_path', ''),
        context='prior Canary artifact',
    ).resolve()
    prior_artifact_sha = sha256_file(prior_artifact_path)
    validation_acceptance_path = (
        training_run_root.resolve() / 'validation_acceptance.json'
    )
    validation_acceptance_sha = sha256_file(validation_acceptance_path)
    validation_acceptance = _read_json_object(validation_acceptance_path)
    validation_per_gate = validation_acceptance.get('per_gate')
    if not isinstance(validation_per_gate, Mapping):
        validation_per_gate = {}
    expected_records = int(canary_fingerprint['paired_records'])
    expected_images = expected_records * len(CAMERAS)
    raw_frozen_image_hashes = finalization.get('image_hashes')
    frozen_image_hashes = (
        dict(raw_frozen_image_hashes)
        if isinstance(raw_frozen_image_hashes, Mapping)
        else {}
    )
    canary_dataset_fingerprint = hashlib.sha256(
        canonical_json({
            'rows_sha256': str(canary_fingerprint['rows']['sha256']),
            'labels_sha256': str(canary_fingerprint['labels']['sha256']),
            'sample_count': expected_records,
            'image_hashes': dict(sorted(frozen_image_hashes.items())),
        }).encode('utf-8')
    ).hexdigest()
    raw_presence_counts = contract.get('scene_presence_record_counts')
    try:
        presence_counts = {
            name: int(raw_presence_counts[name])
            for name in ('sparse', 'medium', 'dense')
        }
    except (KeyError, TypeError, ValueError):
        presence_counts = {}
    required_presence = contract.get('required_scene_presence_densities')
    raw_references = contract.get('historical_disjoint_reference_artifacts')
    historical_disjoint_audit: dict[str, Any] = {}
    reference_roles: set[str] = set()
    if isinstance(raw_references, list):
        for reference in raw_references:
            if not isinstance(reference, Mapping):
                continue
            role = str(reference.get('role') or '').strip()
            reference_path = assert_not_test_path(
                reference.get('path', ''),
                context=f'historical disjoint reference {role}',
            ).resolve()
            reference_sha = sha256_file(reference_path)
            hash_field = str(reference.get('hash_field') or '')
            parsed_reference = _read_json_object(reference_path)
            historical_hashes = parsed_reference.get(hash_field)
            expected_reference_sha = str(
                reference.get('sha256') or ''
            ).casefold()
            if (
                role in reference_roles
                or not isinstance(historical_hashes, Mapping)
                or reference_sha != expected_reference_sha
            ):
                continue
            reference_roles.add(role)
            overlap = sorted(
                set(str(value) for value in historical_hashes.values())
                & set(str(value) for value in frozen_image_hashes.values())
            )
            historical_disjoint_audit[role] = {
                'artifact': file_fingerprint(reference_path),
                'hash_field': hash_field,
                'historical_image_hash_count': len(historical_hashes),
                'canary_image_hash_count': len(frozen_image_hashes),
                'overlap_count': len(overlap),
                'overlap_examples': overlap[:5],
                'rows_sha256': parsed_reference.get('rows_sha256'),
                'labels_sha256': parsed_reference.get('labels_sha256'),
            }
    checks = {
        'schema_version': (
            contract.get('schema_version') == CANARY_COVERAGE_SCHEMA_VERSION
        ),
        'data_role': (
            contract.get('data_role')
            == 'post_training_development_regression_only'
        ),
        'canary_rows_sha256': (
            str(contract.get('canary_rows_sha256') or '').casefold()
            == str(canary_fingerprint['rows']['sha256']).casefold()
        ),
        'canary_labels_sha256': (
            str(contract.get('canary_labels_sha256') or '').casefold()
            == str(canary_fingerprint['labels']['sha256']).casefold()
        ),
        'sample_count': contract.get('sample_count') == expected_records,
        'canary_dataset_fingerprint_sha256': (
            str(contract.get('canary_dataset_fingerprint_sha256') or '').casefold()
            == canary_dataset_fingerprint
        ),
        'historical_disjoint_reference_roles': (
            reference_roles
            == {'old_replay_superset', 'v3r1_train', 'v3r1_validation'}
        ),
        'historical_image_hash_overlap_zero': (
            len(historical_disjoint_audit) == 3
            and all(
                item['overlap_count'] == 0
                for item in historical_disjoint_audit.values()
            )
        ),
        'configured_sample_count': (
            int(config['data']['canary']['expected_rows']) == expected_records
        ),
        'required_scene_presence_densities': (
            required_presence == ['sparse', 'dense']
        ),
        'scene_presence_record_counts': (
            set(presence_counts) == {'sparse', 'medium', 'dense'}
            and all(value >= 0 for value in presence_counts.values())
            and sum(presence_counts.values()) == expected_records
            and [
                name for name in ('sparse', 'medium', 'dense')
                if presence_counts[name] > 0
            ] == required_presence
        ),
        'validation_carries_medium_presence': (
            contract.get('validation_carries_medium_presence') is True
        ),
        'prior_exposure_acknowledged': (
            contract.get('prior_exposure_acknowledged') is True
        ),
        'prior_canary_artifact_sha256': (
            str(contract.get('prior_canary_artifact_sha256') or '').casefold()
            == prior_artifact_sha
        ),
        'canary_finalization_sha256': (
            str(contract.get('canary_finalization_sha256') or '').casefold()
            == finalization_sha
        ),
        'finalization_schema': (
            finalization.get('schema_version')
            == 'room315.hard_case_visual_v3r1.v1'
        ),
        'finalization_passed': finalization.get('passed') is True,
        'finalization_profile': finalization.get('profile') == 'canary',
        'finalization_scenario_count': (
            finalization.get('scenario_count') == expected_records
        ),
        'finalization_image_count': (
            finalization.get('image_count') == expected_images
        ),
        'finalization_image_hash_count': (
            isinstance(finalization.get('image_hashes'), Mapping)
            and len(finalization['image_hashes']) == expected_images
        ),
        'finalization_pair_hash_count': (
            isinstance(finalization.get('image_pair_hashes'), Mapping)
            and len(finalization['image_pair_hashes']) == expected_records
        ),
        'finalization_pair_hash_unique': (
            finalization.get('image_pair_hash_unique') is True
        ),
        'finalization_rows_sha256': (
            str(finalization.get('rows_sha256') or '').casefold()
            == str(canary_fingerprint['rows']['sha256']).casefold()
        ),
        'finalization_labels_sha256': (
            str(finalization.get('labels_sha256') or '').casefold()
            == str(canary_fingerprint['labels']['sha256']).casefold()
        ),
        'validation_acceptance_sha256': (
            str(contract.get('validation_acceptance_sha256') or '').casefold()
            == validation_acceptance_sha
        ),
        'validation_acceptance_passed': (
            validation_acceptance.get('status') == 'passed'
        ),
        'validation_medium_segment_gate_passed': (
            isinstance(
                validation_per_gate.get(
                    'scene_presence_density.segment_top1.medium'
                ),
                Mapping,
            )
            and validation_per_gate[
                'scene_presence_density.segment_top1.medium'
            ].get('status') == 'passed'
        ),
        'validation_medium_joint_gate_passed': (
            isinstance(
                validation_per_gate.get(
                    'scene_presence_density.joint_005.medium'
                ),
                Mapping,
            )
            and validation_per_gate[
                'scene_presence_density.joint_005.medium'
            ].get('status') == 'passed'
        ),
        'used_for_selection': contract.get('used_for_selection') is False,
        'test_loaded': contract.get('test_loaded') is False,
    }
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise V4TrainerError(f'Canary coverage contract mismatch: {failed}')
    return {
        **contract,
        'canary_dataset_fingerprint_sha256': canary_dataset_fingerprint,
        'artifact': file_fingerprint(path),
        'canary_finalization_artifact': file_fingerprint(finalization_path),
        'prior_canary_artifact': file_fingerprint(prior_artifact_path),
        'historical_disjoint_audit': historical_disjoint_audit,
        'validation_acceptance_artifact': file_fingerprint(
            validation_acceptance_path
        ),
        'provenance_checks': checks,
        'verified': True,
    }


def record_groups_disjoint_audit(
    left_records: Sequence[PairedRecord],
    right_records: Sequence[PairedRecord],
    *,
    left_role: str,
    right_role: str,
) -> dict[str, Any]:
    """Fail on cross-role overlap across IDs, families, and image content."""

    def values(records: Sequence[PairedRecord]) -> dict[str, set[str]]:
        result: dict[str, set[str]] = {
            'sample_id': set(),
            'episode_id': set(),
            'scenario_family': set(),
            'image_realpath': set(),
            'image_content_sha256': set(),
            'configuration_family_id': set(),
            'configuration_core_family_id': set(),
        }
        for record in records:
            result['sample_id'].add(record.sample_id)
            for field in ('episode_id', 'scenario_family'):
                value = str(record.row.get(field) or '').strip()
                if value:
                    result[field].add(value)
            for image_path in record.image_paths.values():
                result['image_realpath'].add(str(image_path.resolve()))
                result['image_content_sha256'].add(sha256_file(image_path))
            for field in ('configuration_family_id', 'configuration_core_family_id'):
                value = str(record.trace.get(field) or '').strip()
                if value:
                    result[field].add(value)
        return result

    left_values = values(left_records)
    right_values = values(right_records)
    dimensions: dict[str, Any] = {}
    overlap_found = False
    for field in left_values:
        overlap = sorted(left_values[field] & right_values[field])
        dimensions[field] = {
            f'{left_role}_unique': len(left_values[field]),
            f'{right_role}_unique': len(right_values[field]),
            'overlap_count': len(overlap),
            'overlap_examples': overlap[:10],
        }
        overlap_found = overlap_found or bool(overlap)
    if overlap_found:
        offenders = {
            field: report['overlap_examples']
            for field, report in dimensions.items()
            if report['overlap_count']
        }
        raise V4TrainerError(
            f'{left_role}/{right_role} split overlap detected: {offenders}'
        )
    return {
        'left_role': left_role,
        'right_role': right_role,
        'disjoint': True,
        'dimensions': dimensions,
    }


def split_disjoint_audit(
    train_by_source: Mapping[str, Sequence[PairedRecord]],
    validation: Sequence[PairedRecord],
) -> dict[str, Any]:
    """Fail on train/validation overlap across IDs, families, and real images."""

    train_records = [record for values in train_by_source.values() for record in values]
    report = record_groups_disjoint_audit(
        train_records,
        validation,
        left_role='train',
        right_role='validation',
    )
    return {
        'train_role': 'train_only',
        'validation_role': 'validation_only',
        'disjoint': report['disjoint'],
        'dimensions': report['dimensions'],
    }


def image_pair_manifest_fingerprint(
    records: Sequence[PairedRecord],
) -> dict[str, Any]:
    """Bind an evaluation split to the exact ordered left/right image bytes."""

    entries = []
    for record in records:
        entries.append({
            'sample_id': record.sample_id,
            'left_sha256': sha256_file(record.image_paths['left_rail_rgb']),
            'right_sha256': sha256_file(record.image_paths['right_rail_rgb']),
        })
    entries.sort(key=lambda item: item['sample_id'])
    pair_hashes = [
        hashlib.sha256(canonical_json(item).encode('utf-8')).hexdigest()
        for item in entries
    ]
    return {
        'schema_version': 'room315.visual_v4.image_pair_manifest.v1',
        'paired_records': len(entries),
        'image_count': len(entries) * len(CAMERAS),
        'unique_pair_hashes': len(set(pair_hashes)),
        'manifest_sha256': hashlib.sha256(
            canonical_json(entries).encode('utf-8')
        ).hexdigest(),
    }


def verify_images_against_finalization(
    records: Sequence[PairedRecord],
    finalization_path: Path | str,
) -> dict[str, Any]:
    """Require every Canary image byte hash to match frozen finalization."""

    finalization = _read_json_object(Path(finalization_path).resolve())
    expected = finalization.get('image_hashes')
    if not isinstance(expected, Mapping):
        raise V4TrainerError('Canary finalization lacks image_hashes mapping')
    observed: dict[str, str] = {}
    for record in records:
        episode_id = str(record.row.get('episode_id') or '').strip()
        if not episode_id:
            raise V4TrainerError(
                f'Canary record {record.sample_id} lacks frozen episode_id'
            )
        for camera, path in record.image_paths.items():
            key = f'{episode_id}:{camera}'
            if key in observed:
                raise V4TrainerError(
                    f'duplicate Canary finalization image key: {key}'
                )
            observed[key] = sha256_file(path)
    if observed != dict(expected):
        missing = sorted(set(expected) - set(observed))[:5]
        extra = sorted(set(observed) - set(expected))[:5]
        changed = sorted(
            key
            for key in set(observed) & set(expected)
            if observed[key] != expected[key]
        )[:5]
        raise V4TrainerError(
            'Canary image hashes differ from frozen finalization: '
            f'missing={missing}, extra={extra}, changed={changed}'
        )
    return {
        **image_pair_manifest_fingerprint(records),
        'matches_frozen_finalization': True,
        'frozen_image_hash_count': len(expected),
        'frozen_image_hash_mapping_sha256': hashlib.sha256(
            canonical_json(dict(sorted(expected.items()))).encode('utf-8')
        ).hexdigest(),
    }


def verify_canary_trace_coverage(
    records: Sequence[PairedRecord],
    coverage_contract: Mapping[str, Any],
) -> dict[str, Any]:
    """Match observed record-level presence strata to the predeclared contract."""

    counts = Counter(
        str(record.trace.get('presence_class') or '').strip().casefold()
        for record in records
    )
    if '' in counts:
        raise V4TrainerError('Canary record lacks trace presence_class')
    expected = {
        str(name): int(value)
        for name, value in coverage_contract['scene_presence_record_counts'].items()
    }
    if dict(counts) != {name: count for name, count in expected.items() if count > 0}:
        raise V4TrainerError(
            'Canary presence coverage differs from the predeclared contract: '
            f'{dict(counts)} != {expected}'
        )
    required = list(coverage_contract['required_scene_presence_densities'])
    observed_categories = [
        name for name in ('sparse', 'medium', 'dense') if counts[name] > 0
    ]
    if observed_categories != required:
        raise V4TrainerError(
            'Canary required presence categories differ from observed coverage'
        )
    return {
        'record_count': len(records),
        'scene_presence_record_counts': expected,
        'required_scene_presence_densities': required,
        'matches_predeclared_contract': True,
    }


def verify_loaded_sources_against_training_fingerprints(
    training_run_root: Path,
    train_fingerprints: Mapping[str, Mapping[str, Any]],
    validation_fingerprint: Mapping[str, Any],
) -> dict[str, Any]:
    """Bind the disjoint audit to the exact rows/labels used for training."""

    artifact_path = training_run_root.resolve() / 'input_fingerprints.json'
    frozen = _read_json_object(artifact_path)
    frozen_train = frozen.get('train_sources')
    frozen_validation = frozen.get('validation')
    if not isinstance(frozen_train, Mapping) or not isinstance(
        frozen_validation, Mapping
    ):
        raise V4TrainerError('training input fingerprint artifact is incomplete')
    if set(frozen_train) != set(train_fingerprints):
        raise V4TrainerError('current train sources differ from frozen training inputs')

    def compare(
        current: Mapping[str, Any],
        expected: Mapping[str, Any],
        role: str,
    ) -> dict[str, bool]:
        checks = {
            'paired_records': (
                int(current['paired_records']) == int(expected['paired_records'])
            ),
            'dataset_root': (
                str(Path(current['dataset_root']).resolve())
                == str(Path(expected['dataset_root']).resolve())
            ),
        }
        for name in ('rows', 'labels'):
            checks[f'{name}_sha256'] = (
                current[name]['sha256'] == expected[name]['sha256']
            )
            checks[f'{name}_bytes'] = (
                int(current[name]['bytes']) == int(expected[name]['bytes'])
            )
        failed = sorted(name for name, passed in checks.items() if not passed)
        if failed:
            raise V4TrainerError(
                f'{role} differs from frozen training input fingerprints: {failed}'
            )
        return checks

    train_checks = {
        name: compare(train_fingerprints[name], frozen_train[name], f'train.{name}')
        for name in sorted(train_fingerprints)
    }
    validation_checks = compare(
        validation_fingerprint,
        frozen_validation,
        'validation',
    )
    return {
        'training_input_fingerprint_artifact': file_fingerprint(artifact_path),
        'train_sources': train_checks,
        'validation': validation_checks,
        'all_current_rows_and_labels_match_training_time': True,
    }


def verify_current_images_against_historical_references(
    train_by_source: Mapping[str, Sequence[PairedRecord]],
    validation_records: Sequence[PairedRecord],
    coverage_contract: Mapping[str, Any],
) -> dict[str, Any]:
    """Bind current train/validation image bytes to frozen capture artifacts."""

    records_by_role = {
        'old_replay_superset': list(train_by_source['old_replay']),
        'v3r1_train': list(train_by_source['v3r1_hard_case']),
        'v3r1_validation': list(validation_records),
    }
    references = {
        str(item['role']): item
        for item in coverage_contract['historical_disjoint_reference_artifacts']
    }
    report: dict[str, Any] = {}
    for role, records in records_by_role.items():
        reference = references[role]
        artifact = _read_json_object(Path(reference['path']).resolve())
        frozen_mapping = artifact.get(reference['hash_field'])
        if not isinstance(frozen_mapping, Mapping):
            raise V4TrainerError(f'historical image mapping missing for {role}')
        frozen_hashes = set(str(value) for value in frozen_mapping.values())
        current_hashes = {
            sha256_file(path)
            for record in records
            for path in record.image_paths.values()
        }
        exact_required = role != 'old_replay_superset'
        matches = (
            current_hashes == frozen_hashes
            if exact_required
            else current_hashes <= frozen_hashes
        )
        if not matches:
            raise V4TrainerError(
                f'current {role} images differ from frozen historical reference'
            )
        report[role] = {
            'comparison': 'exact_set' if exact_required else 'subset_of_superset',
            'current_unique_image_hashes': len(current_hashes),
            'historical_unique_image_hashes': len(frozen_hashes),
            'matches_historical_reference': True,
            'artifact': file_fingerprint(reference['path']),
        }
    return {
        'roles': report,
        'all_current_images_match_training_time_references': True,
    }


def class_weights_by_side(
    class_counts: Mapping[str, Any],
    loss_config: Mapping[str, Any],
) -> np.ndarray:
    segment_config = loss_config.get('segment') or {}
    minimum = _finite_float(segment_config.get('class_weight_min'), 'class_weight_min')
    maximum = _finite_float(segment_config.get('class_weight_max'), 'class_weight_max')
    if minimum <= 0.0 or maximum < minimum:
        raise V4TrainerError('segment class-weight bounds are invalid')
    output = []
    for side in SIDES:
        counts = np.asarray(class_counts['by_side'][side], dtype=np.float64)
        if counts.shape != (len(SEGMENT_CLASSES),) or np.any(counts < 0):
            raise V4TrainerError(f'invalid train class counts for {side}')
        reference = max(1.0, float(counts.max(initial=0.0)))
        weights = np.sqrt(reference / np.maximum(counts, 1.0))
        weights /= max(float(weights.mean()), np.finfo(np.float64).eps)
        output.append(np.clip(weights, minimum, maximum))
    return np.asarray(output, dtype=np.float32)


def load_public_topology_contract(
    config: Mapping[str, Any],
) -> dict[str, Any]:
    """Load the canonical contract through the compatibility module's API."""

    topology_config = config.get('topology_contract')
    if not isinstance(topology_config, Mapping):
        raise V4TrainerError('topology_contract is unavailable')
    if (
        topology_config.get('length_loader')
        != 'room_315_rail_defaults.public_rail_segment_lengths'
        or topology_config.get('segment_name_domain') != 'public_ros_label'
        or topology_config.get('forbid_internal_rail_segment_lengths') is not True
    ):
        raise V4TrainerError('only the public ROS-label segment-length loader is allowed')
    try:
        contract = load_authoritative_public_segment_length_contract()
    except Exception as exc:
        raise V4TrainerError(
            f'cannot load authoritative public segment-length contract: {exc}'
        ) from exc
    if contract.authoritative is not True or contract.source != topology_config['length_loader']:
        raise V4TrainerError('segment-length contract is not authoritative production data')
    metadata = contract.canonical_metadata()
    lengths_by_side = {
        side: {
            segment: float(contract.length_m(side, segment))
            for segment in SEGMENT_CLASSES
        }
        for side in SIDES
    }
    return {
        **metadata,
        'segment_name_domain': 'public_ros_label',
        'loader': contract.source,
        'lengths_by_side': lengths_by_side,
    }


def audit_labels_against_public_topology(
    record_groups: Iterable[Sequence[PairedRecord]],
    topology_contract: Mapping[str, Any],
) -> dict[str, Any]:
    """Verify rounded label lengths agree with authoritative public mapping."""

    lengths = topology_contract.get('lengths_by_side')
    if not isinstance(lengths, Mapping):
        raise V4TrainerError('public topology contract lacks lengths_by_side')
    checked = 0
    maximum_absolute_error = 0.0
    for records in record_groups:
        for record in records:
            for identity, shuttle in zip(
                FIXED_IDENTITIES, record.normalized_label['shuttles']
            ):
                if not (shuttle.get('presence') and shuttle.get('visually_available')):
                    continue
                side = derive_side(identity)
                segment = str(shuttle['location']['block']).upper()
                observed = _finite_float(
                    shuttle['rail_position']['segment_length_m'],
                    f'{record.sample_id}:{identity}.segment_length_m',
                )
                expected = float(lengths[side][segment])
                error = abs(observed - expected)
                maximum_absolute_error = max(maximum_absolute_error, error)
                if error > max(1.0e-5, expected * 1.0e-5):
                    raise V4TrainerError(
                        f'{record.sample_id}:{identity} label length uses a '
                        f'non-public mapping for {side}.{segment}: '
                        f'{observed} != {expected}'
                    )
                checked += 1
    return {
        'checked_visible_targets': checked,
        'maximum_absolute_error_m': maximum_absolute_error,
        'matches_public_mapping': True,
        'mapping_fingerprint_sha256': topology_contract['fingerprint_sha256'],
    }


def topology_distance_matrix() -> np.ndarray:
    """Return normalized directed shortest-path distance between rail segments."""

    transitions = {
        'A23': ('A3E', 'A3I'),
        'A3E': ('A34E',),
        'A3I': ('A34I',),
        'A34E': ('A4E',),
        'A34I': ('A4I',),
        'A4E': ('A14',),
        'A4I': ('A14',),
        'A14': ('A1E', 'A1I'),
        'A1E': ('A12E',),
        'A1I': ('A12I',),
        'A12E': ('A2E',),
        'A12I': ('A2I',),
        'A2E': ('A23',),
        'A2I': ('A23',),
    }
    count = len(SEGMENT_CLASSES)
    distance = np.full((count, count), np.inf, dtype=np.float32)
    np.fill_diagonal(distance, 0.0)
    for source, targets in transitions.items():
        for target in targets:
            distance[SEGMENT_CLASSES.index(source), SEGMENT_CLASSES.index(target)] = 1.0
    for pivot in range(count):
        distance = np.minimum(
            distance,
            distance[:, pivot, None] + distance[None, pivot, :],
        )
    if not np.isfinite(distance).all():
        raise AssertionError('declared Room 315 segment graph is disconnected')
    maximum = float(distance.max())
    return distance / max(1.0, maximum)


def _photometric_augment(
    image: Image.Image,
    augmentations: Mapping[str, Any],
    *,
    random_generator: random.Random,
    numpy_generator: np.random.Generator,
) -> Image.Image:
    if augmentations.get('horizontal_flip') is not False:
        raise V4TrainerError('horizontal flip is forbidden')
    if augmentations.get('camera_swap') is not False:
        raise V4TrainerError('camera swap is forbidden')
    result = image
    for name, enhancer in (
        ('brightness', ImageEnhance.Brightness),
        ('contrast', ImageEnhance.Contrast),
        ('saturation', ImageEnhance.Color),
    ):
        magnitude = _finite_float(augmentations.get(name, 0.0), f'augmentation.{name}')
        if magnitude < 0.0:
            raise V4TrainerError(f'augmentation.{name} must be non-negative')
        if magnitude:
            factor = random_generator.uniform(1.0 - magnitude, 1.0 + magnitude)
            result = enhancer(result).enhance(factor)
    gamma_min = _finite_float(augmentations.get('gamma_min', 1.0), 'gamma_min')
    gamma_max = _finite_float(augmentations.get('gamma_max', 1.0), 'gamma_max')
    if gamma_min <= 0.0 or gamma_max < gamma_min:
        raise V4TrainerError('gamma augmentation bounds are invalid')
    gamma = random_generator.uniform(gamma_min, gamma_max)
    if not math.isclose(gamma, 1.0, abs_tol=1e-12):
        array = np.asarray(result, dtype=np.float32) / 255.0
        array = np.power(np.clip(array, 0.0, 1.0), gamma)
        result = Image.fromarray(np.rint(array * 255.0).astype(np.uint8), mode='RGB')
    blur_probability = _finite_float(
        augmentations.get('gaussian_blur_probability', 0.0),
        'gaussian_blur_probability',
    )
    if not 0.0 <= blur_probability <= 1.0:
        raise V4TrainerError('gaussian_blur_probability must be in [0, 1]')
    if random_generator.random() < blur_probability:
        result = result.filter(ImageFilter.GaussianBlur(radius=0.75))
    noise_probability = _finite_float(
        augmentations.get('gaussian_noise_probability', 0.0),
        'gaussian_noise_probability',
    )
    noise_std = _finite_float(
        augmentations.get('gaussian_noise_std', 0.0), 'gaussian_noise_std'
    )
    if not 0.0 <= noise_probability <= 1.0 or noise_std < 0.0:
        raise V4TrainerError('gaussian noise augmentation is invalid')
    if random_generator.random() < noise_probability and noise_std:
        array = np.asarray(result, dtype=np.float32) / 255.0
        array += numpy_generator.normal(0.0, noise_std, size=array.shape).astype(np.float32)
        result = Image.fromarray(
            np.rint(np.clip(array, 0.0, 1.0) * 255.0).astype(np.uint8), mode='RGB'
        )
    return result


def load_paired_image_arrays(
    record: PairedRecord,
    preprocessing: Mapping[str, Any],
    *,
    training: bool,
    seed: int,
    epoch: int,
) -> tuple[np.ndarray, dict[str, tuple[int, int]]]:
    """Decode a fixed-order pair and resize 4:3 images without spatial augmentation."""

    width = _positive_int(preprocessing.get('width'), 'image_preprocessing.width')
    height = _positive_int(preprocessing.get('height'), 'image_preprocessing.height')
    if (width, height) != TARGET_IMAGE_SIZE or width * 3 != height * 4:
        raise V4TrainerError('paired image target must be 320x240 with 4:3 aspect')
    mean = np.asarray(preprocessing.get('normalization_mean'), dtype=np.float32)
    std = np.asarray(preprocessing.get('normalization_std'), dtype=np.float32)
    if mean.shape != (3,) or std.shape != (3,) or np.any(std <= 0.0):
        raise V4TrainerError('image normalization mean/std must contain three valid values')
    augmentations = preprocessing.get('train_augmentations') or {}
    arrays = []
    sizes: dict[str, tuple[int, int]] = {}
    for camera in CAMERAS:
        path = record.image_paths[camera]
        context = f'{record.sample_id}:{camera}'
        try:
            with Image.open(path) as opened:
                opened.load()
                rgb = opened.convert('RGB')
        except Exception as exc:
            raise V4TrainerError(f'{context} image cannot be decoded: {path}') from exc
        sizes[camera] = _validate_image_size(
            rgb.size,
            path=path,
            context=context,
        )
        digest = hashlib.sha256(
            (
                f'{seed}:{epoch}:{record.source}:{record.sample_id}:'
                f'{record.draw_index}:{record.resample_occurrence}:{camera}'
            ).encode()
        ).digest()
        if training:
            rgb = _photometric_augment(
                rgb,
                augmentations,
                random_generator=random.Random(int.from_bytes(digest[:8], 'big')),
                numpy_generator=np.random.default_rng(int.from_bytes(digest[8:16], 'big')),
            )
        resized = rgb.resize((width, height), Image.Resampling.BILINEAR)
        array = np.asarray(resized, dtype=np.float32) / 255.0
        array = np.transpose(array, (2, 0, 1))
        arrays.append((array - mean[:, None, None]) / std[:, None, None])
    return np.concatenate(arrays, axis=0).astype(np.float32), sizes


class V4PairedImageDataset:
    def __init__(
        self,
        records: Sequence[PairedRecord],
        preprocessing: Mapping[str, Any],
        *,
        torch_module: Any,
        training: bool,
        seed: int,
        epoch: int,
    ) -> None:
        self.records = tuple(records)
        self.preprocessing = dict(preprocessing)
        self.torch = torch_module
        self.training = bool(training)
        self.seed = int(seed)
        self.epoch = int(epoch)

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, index: int) -> dict[str, Any]:
        record = self.records[index]
        image, image_sizes = load_paired_image_arrays(
            record,
            self.preprocessing,
            training=self.training,
            seed=self.seed,
            epoch=self.epoch,
        )
        target = extract_structured_targets(
            record.normalized_label,
            image_sizes=image_sizes,
            already_normalized=True,
        )
        result: dict[str, Any] = {
            'image': self.torch.as_tensor(image, dtype=self.torch.float32),
            'sample_id': record.sample_id,
            'source': record.source,
            'trace_json': canonical_json(record.trace),
        }
        for name, value in target.items():
            result[name] = self.torch.as_tensor(value)
        return result


def preflight_configuration(
    config_path: Path | str = DEFAULT_CONFIG_PATH,
    *,
    environ: Mapping[str, str] | None = None,
    decode_images: bool = False,
) -> PreflightBundle:
    """Load train/validation only and return their fail-closed audit bundle."""

    resolved_config_path = assert_not_test_path(
        config_path, context='configuration path'
    ).resolve()
    config = load_v4_config(resolved_config_path, environ=environ)
    train_by_source: dict[str, tuple[PairedRecord, ...]] = {}
    source_reports: dict[str, Any] = {}
    enabled_specs = [
        spec for spec in config['data']['train_sources']
        if bool(spec.get('enabled', True))
    ]
    for spec in enabled_specs:
        name = str(spec['name'])
        records, report = load_paired_source(
            spec,
            role='train',
            decode_images=decode_images,
        )
        train_by_source[name] = tuple(records)
        source_reports[name] = report
    validation_records, validation_report = load_paired_source(
        config['data']['validation'],
        role='validation',
        decode_images=decode_images,
    )
    disjoint_report = split_disjoint_audit(train_by_source, validation_records)
    class_counts = compute_train_class_counts(train_by_source)
    validation_support = segment_support_report(
        validation_records,
        role='validation',
    )
    require_full_segment_support(validation_support)
    approved_v3_validation_baseline = verify_approved_v3_validation_baseline(
        config,
        validation_report,
        validation_support,
    )
    class_weights = class_weights_by_side(class_counts, config['loss'])
    public_topology = load_public_topology_contract(config)
    topology_label_audit = audit_labels_against_public_topology(
        [*train_by_source.values(), validation_records],
        public_topology,
    )

    checkpoint_path = Path(config['initialization']['checkpoint'])
    expected_checkpoint_sha = str(
        config['initialization']['checkpoint_sha256']
    ).casefold()
    actual_checkpoint_sha = sha256_file(checkpoint_path)
    if actual_checkpoint_sha != expected_checkpoint_sha:
        raise V4TrainerError(
            'initial V3 checkpoint SHA-256 mismatch: '
            f'{actual_checkpoint_sha} != {expected_checkpoint_sha}'
        )
    sampling_probe = build_epoch_selection(
        train_by_source,
        enabled_specs,
        config['sampling'],
        seed=int(config['training']['seed']),
        epoch=1,
    )
    source_selection_counts = Counter(item['source'] for item in sampling_probe)
    report = {
        'schema_version': RUN_SCHEMA_VERSION,
        'status': 'passed',
        'canary_loaded': False,
        'test_loaded': False,
        'configuration': file_fingerprint(resolved_config_path),
        'effective_config_sha256': hashlib.sha256(
            canonical_json(config).encode('utf-8')
        ).hexdigest(),
        'train_sources': source_reports,
        'validation': validation_report,
        'split_disjoint_audit': disjoint_report,
        'train_sample_count': sum(len(values) for values in train_by_source.values()),
        'validation_sample_count': len(validation_records),
        'class_counts': class_counts,
        'class_weights_by_side': {
            side: class_weights[index].tolist()
            for side, index in SIDE_INDEX.items()
        },
        'class_count_provenance': 'train_only',
        'validation_support': validation_support,
        'approved_v3_validation_baseline': approved_v3_validation_baseline,
        'public_topology_contract': public_topology,
        'topology_lengths_by_side': public_topology['lengths_by_side'],
        'topology_length_provenance': (
            'room_315_rail_defaults.public_rail_segment_lengths'
        ),
        'topology_label_audit': topology_label_audit,
        'sampler_epoch_one': {
            'selection_count': len(sampling_probe),
            'by_source': dict(sorted(source_selection_counts.items())),
            'selection_sha256': hashlib.sha256(
                canonical_json(sampling_probe).encode('utf-8')
            ).hexdigest(),
        },
        'initialization_checkpoint': {
            **file_fingerprint(checkpoint_path),
            'expected_sha256': expected_checkpoint_sha,
            'verified': True,
        },
        'image_contract': {
            'camera_order': list(CAMERAS),
            'resize': list(TARGET_IMAGE_SIZE),
            'aspect_ratio': '4:3',
            'photometric_only_augmentation': True,
            'horizontal_flip': False,
            'camera_swap': False,
            'decoded_during_preflight': bool(decode_images),
        },
    }
    return PreflightBundle(
        config_path=resolved_config_path,
        config=config,
        train_by_source=train_by_source,
        validation=tuple(validation_records),
        report=report,
    )


def require_training_stack() -> tuple[Any, Any, Any]:
    try:
        import torch
        import torchvision
        import room_315_visual_training_v4 as training_api
    except Exception as exc:
        raise V4TrainerError(
            f'Torch, TorchVision and the V4 loss module are required: {exc}'
        ) from exc
    return torch, torchvision, training_api


def set_deterministic(torch_module: Any, seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed % (2**32))
    torch_module.manual_seed(seed % (2**63))
    if hasattr(torch_module, 'cuda'):
        torch_module.cuda.manual_seed_all(seed % (2**63))
    if hasattr(torch_module.backends, 'cudnn'):
        torch_module.backends.cudnn.benchmark = False
        torch_module.backends.cudnn.deterministic = True
    cuda_backend = getattr(torch_module.backends, 'cuda', None)
    if cuda_backend is not None:
        for name, enabled in (
            ('enable_flash_sdp', False),
            ('enable_mem_efficient_sdp', False),
            ('enable_math_sdp', True),
        ):
            function = getattr(cuda_backend, name, None)
            if callable(function):
                function(enabled)
    try:
        torch_module.use_deterministic_algorithms(True, warn_only=False)
    except (AttributeError, TypeError):
        pass


def build_configured_model(
    config: Mapping[str, Any],
    torch_module: Any,
    torchvision_module: Any,
) -> Any:
    model_config = config['model']
    return build_visual_state_model_v4(
        torch_module,
        torchvision_module,
        head_type=str(model_config['head_type']),
        hidden_dim=int(model_config['hidden_dim']),
        attention_heads=int(model_config['attention_heads']),
        dropout=float(model_config['dropout']),
    )


def _torch_load(torch_module: Any, path: Path, *, map_location: Any) -> Any:
    try:
        return torch_module.load(
            path,
            map_location=map_location,
            weights_only=False,
        )
    except TypeError:
        return torch_module.load(path, map_location=map_location)


def initialize_verified_v3_backbone(
    model: Any,
    config: Mapping[str, Any],
    torch_module: Any,
) -> dict[str, Any]:
    initialization = config['initialization']
    checkpoint_path = Path(initialization['checkpoint'])
    expected = str(initialization['checkpoint_sha256']).casefold()
    actual = sha256_file(checkpoint_path)
    if actual != expected:
        raise V4TrainerError(
            f'initial V3 checkpoint SHA-256 mismatch: {actual} != {expected}'
        )
    checkpoint = _torch_load(
        torch_module,
        checkpoint_path,
        map_location='cpu',
    )
    if not isinstance(checkpoint, Mapping):
        raise V4TrainerError('initial V3 checkpoint must contain an object')
    state = checkpoint.get('model_state_dict')
    if not isinstance(state, Mapping):
        raise V4TrainerError('initial V3 checkpoint lacks model_state_dict')
    try:
        migration = initialize_v4_backbone_from_v3_model_state_dict(model, state)
    except Exception as exc:
        raise V4TrainerError(f'V3 backbone-only migration failed: {exc}') from exc
    return {
        'checkpoint_path': str(checkpoint_path),
        'checkpoint_sha256': actual,
        'verified_before_load': True,
        'backbone_only': True,
        'migration': migration,
    }


def _module_parameters(module: Any) -> tuple[Any, ...]:
    return tuple(module.parameters())


def configure_training_stage(
    model: Any,
    training_config: Mapping[str, Any],
    *,
    epoch: int,
    torch_module: Any,
) -> dict[str, Any]:
    """Apply staged trainability while always freezing shared BN statistics."""

    if epoch <= 0:
        raise V4TrainerError('training stage epoch must be positive')
    layer4_open = epoch >= int(training_config['unfreeze_layer4_at_epoch'])
    layer3_open = epoch >= int(training_config['unfreeze_shared_layer3_at_epoch'])
    declared_heads_only = int(training_config['heads_only_through_epoch'])
    if epoch <= declared_heads_only and (layer4_open or layer3_open):
        raise V4TrainerError('stage schedule violates heads-only epoch declaration')

    model.train()
    for parameter in model.shared_stem.parameters():
        parameter.requires_grad = False
    shared_layer3 = getattr(model.shared_stem, 'layer3', None)
    if shared_layer3 is None:
        raise V4TrainerError('V4 shared_stem is missing layer3')
    for parameter in shared_layer3.parameters():
        parameter.requires_grad = layer3_open
    for branch in (model.left_layer4, model.right_layer4):
        for parameter in branch.parameters():
            parameter.requires_grad = layer4_open
        branch.train(layer4_open)
    for head in (model.left_head, model.right_head):
        for parameter in head.parameters():
            parameter.requires_grad = True
        head.train(True)

    # Shared camera BatchNorm running statistics remain frozen forever.  When
    # layer3 opens, its convolution and BN affine tensors may learn, but every
    # shared BN module is kept in eval mode to prevent cross-source drift.
    shared_batch_norms = [
        module for module in model.shared_stem.modules()
        if isinstance(module, torch_module.nn.modules.batchnorm._BatchNorm)
    ]
    for module in shared_batch_norms:
        module.eval()
    if any(module.training for module in shared_batch_norms):
        raise V4TrainerError('shared_stem BatchNorm running statistics are not frozen')

    early_shared_parameters = [
        parameter
        for name, parameter in model.shared_stem.named_parameters()
        if not name.startswith('layer3.')
    ]
    if any(parameter.requires_grad for parameter in early_shared_parameters):
        raise V4TrainerError('shared stem before layer3 must remain frozen')
    if any(parameter.requires_grad != layer3_open for parameter in shared_layer3.parameters()):
        raise V4TrainerError('shared layer3 trainability does not match its stage')
    if any(
        parameter.requires_grad != layer4_open
        for branch in (model.left_layer4, model.right_layer4)
        for parameter in branch.parameters()
    ):
        raise V4TrainerError('rail-local layer4 trainability does not match its stage')

    def summary(name: str, modules: Iterable[Any]) -> dict[str, Any]:
        parameters = [parameter for module in modules for parameter in module.parameters()]
        return {
            'name': name,
            'parameter_tensors': len(parameters),
            'parameter_count': sum(int(parameter.numel()) for parameter in parameters),
            'trainable_tensors': sum(int(parameter.requires_grad) for parameter in parameters),
            'trainable_parameter_count': sum(
                int(parameter.numel()) for parameter in parameters if parameter.requires_grad
            ),
        }

    left_layer4_bn = [
        module for module in model.left_layer4.modules()
        if isinstance(module, torch_module.nn.modules.batchnorm._BatchNorm)
    ]
    right_layer4_bn = [
        module for module in model.right_layer4.modules()
        if isinstance(module, torch_module.nn.modules.batchnorm._BatchNorm)
    ]
    if layer4_open and not all(
        module.training for module in left_layer4_bn + right_layer4_bn
    ):
        raise V4TrainerError('rail-local layer4 BatchNorm must train after layer4 opens')
    if not layer4_open and any(
        module.training for module in left_layer4_bn + right_layer4_bn
    ):
        raise V4TrainerError('rail-local layer4 BatchNorm opened too early')
    return {
        'epoch': epoch,
        'stage': (
            'heads_shared_layer3_layer4'
            if layer3_open
            else 'heads_layer4'
            if layer4_open
            else 'heads_only'
        ),
        'heads_open': True,
        'layer4_open': layer4_open,
        'shared_layer3_open': layer3_open,
        'shared_batch_norm_count': len(shared_batch_norms),
        'shared_batch_norm_running_stats_frozen': True,
        'shared_layer3_batch_norm_affine_trainable': layer3_open,
        'rail_layer4_batch_norm_training': layer4_open,
        'groups': {
            'heads': summary('heads', (model.left_head, model.right_head)),
            'layer4': summary('layer4', (model.left_layer4, model.right_layer4)),
            'shared_layer3': summary('shared_layer3', (shared_layer3,)),
        },
        'assertions_passed': True,
    }


def build_staged_optimizer(
    model: Any,
    training_config: Mapping[str, Any],
    torch_module: Any,
) -> Any:
    shared_layer3 = getattr(model.shared_stem, 'layer3')
    groups = [
        {
            'name': 'heads',
            'params': list(model.left_head.parameters()) + list(model.right_head.parameters()),
            'lr': float(training_config['heads_learning_rate']),
        },
        {
            'name': 'layer4',
            'params': list(model.left_layer4.parameters()) + list(model.right_layer4.parameters()),
            'lr': float(training_config['layer4_learning_rate']),
        },
        {
            'name': 'shared_layer3',
            'params': list(shared_layer3.parameters()),
            'lr': float(training_config['shared_layer3_learning_rate']),
        },
    ]
    parameter_ids = [id(parameter) for group in groups for parameter in group['params']]
    if len(parameter_ids) != len(set(parameter_ids)):
        raise V4TrainerError('staged optimizer parameter groups overlap')
    for group in groups:
        if not group['params'] or group['lr'] <= 0.0:
            raise V4TrainerError(f'invalid staged optimizer group: {group["name"]}')
    return torch_module.optim.AdamW(
        groups,
        weight_decay=float(training_config['weight_decay']),
    )


def _loss_required_tensors(
    predictions: Mapping[str, Any],
    targets: Mapping[str, Any],
) -> tuple[Any, ...]:
    required_predictions = ('segment_logits', 'loaded_logits', 'bbox', 's_ratio')
    required_targets = ('segment', 'loaded', 'bbox', 's_ratio', 'visibility_mask')
    try:
        values = tuple(predictions[name] for name in required_predictions) + tuple(
            targets[name] for name in required_targets
        )
    except KeyError as exc:
        raise V4TrainerError(f'loss tensor contract is missing {exc.args[0]}') from exc
    return values


def compute_configured_loss(
    predictions: Mapping[str, Any],
    targets: Mapping[str, Any],
    *,
    class_weights: Any,
    topology_distances: Any,
    loss_config: Mapping[str, Any],
    torch_module: Any,
    training_api: Any,
) -> dict[str, Any]:
    """Pass configured values to the separately tested V4 loss API."""

    _loss_required_tensors(predictions, targets)
    del torch_module  # The tested loss module owns tensor/device validation.
    segment_config = loss_config['segment']
    bbox_config = loss_config['bbox']
    return training_api.compute_v4_loss(
        predictions,
        targets,
        segment_class_weights=class_weights,
        topology_distance_matrix=topology_distances,
        loss_weights={
            'segment': float(segment_config['weight']),
            's_ratio': float(loss_config['s_ratio']['weight']),
            'loaded': float(loss_config['loaded_state']['weight']),
            'bbox': float(bbox_config['weight']),
            'topology': float(loss_config['topology']['weight']),
        },
        bbox_image_size=None,
        segment_label_smoothing=float(
            segment_config.get('label_smoothing', 0.0)
        ),
        loaded_label_smoothing=float(
            loss_config['loaded_state'].get('label_smoothing', 0.0)
        ),
        s_ratio_beta=float(loss_config['s_ratio'].get('beta', 1.0)),
        bbox_l1_weight=float(bbox_config.get('l1_weight', 1.0)),
        bbox_giou_weight=float(bbox_config.get('giou_weight', 1.0)),
    )


def checkpoint_selection_key(metrics: Mapping[str, Any]) -> tuple[float, float, float]:
    """Planning-first, validation-only lexicographic checkpoint selection key."""

    return (
        _finite_float(metrics['planning_selection_score'], 'planning_selection_score'),
        _finite_float(
            metrics['planning_metrics']['worst_side_x_segment_recall']['recall'],
            'worst_side_x_segment_recall',
        ),
        -_finite_float(
            metrics['correct_segment_s_m_p95_m'],
            'correct_segment_s_m_p95_m',
        ),
    )


def _target_device_dict(batch: Mapping[str, Any], device: Any) -> dict[str, Any]:
    return {
        name: batch[name].to(device, non_blocking=True)
        for name in (
            'segment', 'loaded', 'bbox', 's_ratio', 'visibility_mask',
            'segment_length_m', 's_m',
        )
    }


def make_loader(
    records: Sequence[PairedRecord],
    config: Mapping[str, Any],
    *,
    torch_module: Any,
    training: bool,
    epoch: int,
    batch_size_override: int | None = None,
) -> Any:
    training_config = config['training']
    workers = int(training_config.get('num_workers', 0))
    batch_size = int(batch_size_override or training_config['batch_size'])
    dataset = V4PairedImageDataset(
        records,
        config['image_preprocessing'],
        torch_module=torch_module,
        training=training,
        seed=int(training_config['seed']),
        epoch=epoch,
    )
    generator = torch_module.Generator()
    generator.manual_seed(int(training_config['seed']) + epoch)
    return torch_module.utils.data.DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=workers,
        pin_memory=bool(training_config.get('pin_memory', False)),
        persistent_workers=(
            workers > 0 and bool(training_config.get('persistent_workers', False))
        ),
        generator=generator,
        drop_last=False,
    )


def _autocast_context(torch_module: Any, device: Any, enabled: bool) -> Any:
    return torch_module.autocast(
        device_type=str(device.type),
        dtype=torch_module.float16,
        enabled=bool(enabled),
    )


def _metric_subset(
    predictions: Mapping[str, Any],
    targets: Mapping[str, Any],
    subset_mask: Any,
    training_api: Any,
    segment_lengths_by_side: Sequence[Sequence[float]],
) -> dict[str, Any]:
    subset_targets = dict(targets)
    subset_targets['visibility_mask'] = targets['visibility_mask'] & subset_mask
    raw = training_api.compute_visual_training_v4_metrics(
        predictions,
        subset_targets,
        segment_class_names=SEGMENT_CLASSES,
        segment_lengths_by_side=segment_lengths_by_side,
    )
    supported = [
        item for item in raw['segment_per_class'] if int(item['support']) > 0
    ]
    result = {
        'visible_count': int(raw['visible_count']),
        'supported_segment_class_count': len(supported),
        'supported_class_macro_recall': (
            sum(float(item['recall']) for item in supported) / len(supported)
            if supported else 0.0
        ),
        'supported_class_macro_f1': (
            sum(float(item['f1']) for item in supported) / len(supported)
            if supported else 0.0
        ),
        'segment_top1_accuracy': float(raw['segment_top1_accuracy']),
        'segment_top2_accuracy': float(raw['segment_top2_accuracy']),
        'supported_zero_recall_classes': list(raw['segment_zero_recall_classes']),
        'loaded_accuracy': float(raw['loaded_accuracy']),
        's_ratio_mae': float(raw['s_ratio_mae']),
        's_ratio_p95': float(raw['s_ratio_p95']),
        'bbox_iou': float(raw['bbox_iou']),
        'joint_localization_accuracy_s_ratio_0_05': float(
            raw['joint_localization_accuracy_s_ratio_0_05']
        ),
        'joint_localization_accuracy_s_ratio_0_12': float(
            raw['joint_localization_accuracy_s_ratio_0_12']
        ),
        'correct_segment_s_m_mae_m': float(raw['correct_segment_s_m_mae_m']),
        'correct_segment_s_m_p95_m': float(raw['correct_segment_s_m_p95_m']),
        'correct_segment_s_m_count': int(raw['correct_segment_s_m_count']),
        'correct_segment_s_m_coverage': float(raw['correct_segment_s_m_coverage']),
    }
    return result


def evaluate_model(
    model: Any,
    records: Sequence[PairedRecord],
    config: Mapping[str, Any],
    *,
    torch_module: Any,
    training_api: Any,
    device: Any,
    class_weights: Any,
    topology_lengths: Mapping[str, Mapping[str, float]],
    epoch: int = 0,
    require_full_side_segment_support: bool = True,
) -> dict[str, Any]:
    """Aggregate tensors once, then compute V4 metrics and metadata breakdowns."""

    if not records:
        raise V4TrainerError('evaluation records are empty')
    loader = make_loader(
        records,
        config,
        torch_module=torch_module,
        training=False,
        epoch=max(0, epoch),
    )
    amp_enabled = bool(
        config['training'].get('automatic_mixed_precision', False)
        and device.type == 'cuda'
    )
    prediction_parts: dict[str, list[Any]] = defaultdict(list)
    target_parts: dict[str, list[Any]] = defaultdict(list)
    traces: list[dict[str, Any]] = []
    evaluated_samples = 0
    model.eval()
    with torch_module.inference_mode():
        for batch in loader:
            image = batch['image'].to(device, non_blocking=True)
            targets = _target_device_dict(batch, device)
            with _autocast_context(torch_module, device, amp_enabled):
                predictions = model(image)
            batch_size = int(image.shape[0])
            evaluated_samples += batch_size
            for name, value in predictions.items():
                prediction_parts[name].append(value.detach().float().cpu())
            for name, value in targets.items():
                target_parts[name].append(value.detach().cpu())
            traces.extend(json.loads(value) for value in batch['trace_json'])
    aggregated_predictions = {
        name: torch_module.cat(parts, dim=0)
        for name, parts in prediction_parts.items()
    }
    aggregated_targets = {
        name: torch_module.cat(parts, dim=0)
        for name, parts in target_parts.items()
    }
    aggregate_losses = compute_configured_loss(
        aggregated_predictions,
        aggregated_targets,
        class_weights=class_weights,
        topology_distances=torch_module.as_tensor(
            topology_distance_matrix(), dtype=torch_module.float32
        ),
        loss_config=config['loss'],
        torch_module=torch_module,
        training_api=training_api,
    )
    length_matrix = [
        [topology_lengths[side][segment] for segment in SEGMENT_CLASSES]
        for side in SIDES
    ]
    metrics = training_api.compute_visual_training_v4_metrics(
        aggregated_predictions,
        aggregated_targets,
        segment_class_names=SEGMENT_CLASSES,
        segment_lengths_by_side=length_matrix,
    )
    visibility = aggregated_targets['visibility_mask'].to(dtype=torch_module.bool)
    if len(traces) != evaluated_samples:
        raise V4TrainerError('evaluation trace count differs from sample count')

    def subset_metrics(mask: Any) -> dict[str, Any]:
        return _metric_subset(
            aggregated_predictions,
            aggregated_targets,
            mask,
            training_api,
            length_matrix,
        )

    slot_indexes = torch_module.arange(len(FIXED_IDENTITIES)).unsqueeze(0)
    identity_breakdown: dict[str, Any] = {}
    for slot, identity in enumerate(FIXED_IDENTITIES):
        subset = torch_module.zeros_like(visibility)
        subset[:, slot] = True
        identity_breakdown[identity] = subset_metrics(subset)

    side_breakdown = {
        side: subset_metrics(
            (slot_indexes < 4).expand_as(visibility)
            if side == 'left'
            else (slot_indexes >= 4).expand_as(visibility)
        )
        for side in SIDES
    }
    segment_breakdown = {
        segment: subset_metrics(aggregated_targets['segment'] == segment_index)
        for segment_index, segment in enumerate(SEGMENT_CLASSES)
    }
    side_x_segment_breakdown = {
        side: {
            segment: subset_metrics(
                (aggregated_targets['segment'] == segment_index)
                & (
                    (slot_indexes < 4).expand_as(visibility)
                    if side == 'left'
                    else (slot_indexes >= 4).expand_as(visibility)
                )
            )
            for segment_index, segment in enumerate(SEGMENT_CLASSES)
        }
        for side in SIDES
    }

    target_zone_masks: dict[str, Any] = {}
    identity_zone_masks: dict[str, Any] = {}
    position_bin_masks: dict[str, Any] = {}
    sample_category_masks: dict[str, dict[str, Any]] = {
        'scene_occlusion_class': {},
        'scene_presence_density': {},
    }
    sample_category_trace_field = {
        'scene_occlusion_class': 'occlusion_class',
        'scene_presence_density': 'presence_class',
    }
    identity_slot = {identity: slot for slot, identity in enumerate(FIXED_IDENTITIES)}
    for sample_index, trace in enumerate(traces):
        target_identity = str(trace.get('target_identity') or '').strip().upper()
        target_zone = str(trace.get('target_zone') or '').strip().lower()
        if target_identity in identity_slot and target_zone:
            target_zone_masks.setdefault(
                target_zone, torch_module.zeros_like(visibility)
            )[sample_index, identity_slot[target_identity]] = True

        identity_to_zone = trace.get('identity_to_zone') or {}
        if not isinstance(identity_to_zone, Mapping):
            raise V4TrainerError('identity_to_zone trace value must be an object')
        for raw_identity, raw_zone in identity_to_zone.items():
            identity = str(raw_identity).strip().upper()
            zone = str(raw_zone).strip().lower()
            if identity in identity_slot and zone:
                identity_zone_masks.setdefault(
                    zone, torch_module.zeros_like(visibility)
                )[sample_index, identity_slot[identity]] = True

        identity_to_position_bin = trace.get('identity_to_position_bin') or {}
        if not isinstance(identity_to_position_bin, Mapping):
            raise V4TrainerError(
                'identity_to_position_bin trace value must be an object'
            )
        for raw_identity, raw_bin in identity_to_position_bin.items():
            identity = str(raw_identity).strip().upper()
            position_bin = str(raw_bin).strip().lower()
            if identity in identity_slot and position_bin:
                position_bin_masks.setdefault(
                    position_bin, torch_module.zeros_like(visibility)
                )[sample_index, identity_slot[identity]] = True

        for output_name, masks in sample_category_masks.items():
            value = str(
                trace.get(sample_category_trace_field[output_name]) or ''
            ).strip().lower()
            if value:
                masks.setdefault(value, torch_module.zeros_like(visibility))[
                    sample_index, :
                ] = True

    target_zone_breakdown = {
        zone: subset_metrics(zone_mask)
        for zone, zone_mask in sorted(target_zone_masks.items())
    }
    identity_zone_breakdown = {
        zone: subset_metrics(zone_mask)
        for zone, zone_mask in sorted(identity_zone_masks.items())
    }
    position_bin_breakdown = {
        position_bin: subset_metrics(position_mask)
        for position_bin, position_mask in sorted(position_bin_masks.items())
    }
    categorical_breakdowns = {
        field: {
            value: subset_metrics(mask)
            for value, mask in sorted(masks.items())
        }
        for field, masks in sample_category_masks.items()
    }

    side_segment_cells = [
        {
            'side': side,
            'segment': str(item['class_name']),
            'support': int(item['support']),
            'recall': float(item['recall']),
            'f1': float(item['f1']),
        }
        for side in SIDES
        for item in metrics['per_side'][side]['per_class']
        if int(item['support']) > 0
    ]
    side_segment_support_complete = (
        len(side_segment_cells) == len(SIDES) * len(SEGMENT_CLASSES)
    )
    if require_full_side_segment_support and not side_segment_support_complete:
        raise V4TrainerError(
            'evaluation lacks support for one or more side x segment cells'
        )
    if not side_segment_cells:
        raise V4TrainerError('evaluation has no visible side x segment targets')
    worst_side_segment = min(
        side_segment_cells,
        key=lambda item: (item['recall'], item['f1'], item['side'], item['segment']),
    )
    minimum_side_top1 = min(
        float(metrics['per_side'][side]['top1_accuracy']) for side in SIDES
    )
    switch_zone_top1 = float(
        identity_zone_breakdown.get('switch', {}).get('segment_top1_accuracy', 0.0)
    )
    boundary_zone_top1 = float(
        identity_zone_breakdown.get('boundary', {}).get('segment_top1_accuracy', 0.0)
    )
    planning_components = {
        'segment_macro_f1': float(metrics['segment_macro_f1']),
        'minimum_side_top1': minimum_side_top1,
        'worst_side_x_segment_recall': float(worst_side_segment['recall']),
        'joint_segment_and_ratio_005': float(
            metrics['joint_localization_accuracy_s_ratio_0_05']
        ),
        'switch_zone_segment_top1': switch_zone_top1,
        'boundary_zone_segment_top1': boundary_zone_top1,
    }
    planning_weights = {
        name: float(value)
        for name, value in config['evaluation']['planning_score_weights'].items()
    }
    planning_score = sum(
        planning_components[name] * planning_weights[name]
        for name in planning_weights
    )
    physical_position_diagnostics = {
        'correct_segment_only': dict(metrics['s_m_metrics']['correct_segment_only']),
        'oracle_true_segment_ratio_diagnostic': dict(
            metrics['s_m_metrics']['oracle_true_segment_ratio_diagnostic']
        ),
        'cross_segment_local_coordinate_metric': 'forbidden',
    }
    metrics.update({
        'sample_count': evaluated_samples,
        'losses': {
            name: float(aggregate_losses[name].detach().cpu())
            for name in LOSS_SCALAR_KEYS
        },
        'loss_aggregation': 'single_reduction_over_all_visible_validation_slots',
        'physical_position_diagnostics': physical_position_diagnostics,
        'breakdowns': {
            'side': side_breakdown,
            'identity': identity_breakdown,
            'segment': segment_breakdown,
            'side_x_segment': side_x_segment_breakdown,
            'target_zone': target_zone_breakdown,
            'zone_by_identity': identity_zone_breakdown,
            'position_bin': position_bin_breakdown,
            **categorical_breakdowns,
        },
        'planning_selection_score': planning_score,
        'planning_metrics': {
            'score': planning_score,
            'weights': planning_weights,
            'components': planning_components,
            'worst_side_x_segment_recall': worst_side_segment,
            'all_side_x_segment_cells_supported': side_segment_support_complete,
        },
        'selection_role': 'validation_only' if records[0].role == 'validation' else 'none',
        'acceptance_gates_evaluated': False,
        'pending_evaluations': [
            'segment_temperature_calibration',
            'selective_accuracy_coverage',
            'blank_opposite_camera_counterfactual',
            'shuffle_opposite_camera_counterfactual',
            'camera_order_swap_canary',
            'pilot_acceptance_gates',
        ],
    })
    metrics['selection_key'] = list(checkpoint_selection_key(metrics))
    return metrics


def evaluate_camera_counterfactuals(
    model: Any,
    records: Sequence[PairedRecord],
    config: Mapping[str, Any],
    *,
    torch_module: Any,
    training_api: Any,
    device: Any,
    topology_lengths: Mapping[str, Mapping[str, float]],
    epoch: int = 0,
) -> dict[str, Any]:
    """Measure rail isolation and camera-order sensitivity on an evaluation split."""

    if not records:
        raise V4TrainerError('counterfactual records are empty')
    loader = make_loader(
        records,
        config,
        torch_module=torch_module,
        training=False,
        epoch=max(0, epoch),
    )
    amp_enabled = bool(
        config['training'].get('automatic_mixed_precision', False)
        and device.type == 'cuda'
    )
    preprocessing = config['image_preprocessing']
    mean = torch_module.as_tensor(
        preprocessing['normalization_mean'],
        dtype=torch_module.float32,
        device=device,
    ).reshape(1, 3, 1, 1)
    std = torch_module.as_tensor(
        preprocessing['normalization_std'],
        dtype=torch_module.float32,
        device=device,
    ).reshape(1, 3, 1, 1)
    black = -mean / std
    changes = {
        'blank_opposite_camera': {
            'left': {name: 0.0 for name in V4_OUTPUT_KEYS},
            'right': {name: 0.0 for name in V4_OUTPUT_KEYS},
        },
        'shuffle_opposite_camera': {
            'left': {name: 0.0 for name in V4_OUTPUT_KEYS},
            'right': {name: 0.0 for name in V4_OUTPUT_KEYS},
        },
    }
    baseline_parts: dict[str, list[Any]] = defaultdict(list)
    swapped_parts: dict[str, list[Any]] = defaultdict(list)
    target_parts: dict[str, list[Any]] = defaultdict(list)
    effective_shuffle_batches = 0
    model.eval()
    with torch_module.inference_mode():
        for batch in loader:
            image = batch['image'].to(device, non_blocking=True)
            targets = _target_device_dict(batch, device)
            blank_right = image.clone()
            blank_right[:, 3:] = black
            blank_left = image.clone()
            blank_left[:, :3] = black
            shuffle_right = image.clone()
            shuffle_left = image.clone()
            if int(image.shape[0]) > 1:
                shuffle_right[:, 3:] = image[:, 3:].roll(1, dims=0)
                shuffle_left[:, :3] = image[:, :3].roll(1, dims=0)
                effective_shuffle_batches += 1
            swapped_image = torch_module.cat((image[:, 3:], image[:, :3]), dim=1)
            with _autocast_context(torch_module, device, amp_enabled):
                baseline = model(image)
                blank_right_outputs = model(blank_right)
                blank_left_outputs = model(blank_left)
                shuffle_right_outputs = model(shuffle_right)
                shuffle_left_outputs = model(shuffle_left)
                swapped = model(swapped_image)

            for name in V4_OUTPUT_KEYS:
                left_blank_change = float(
                    (baseline[name][:, :4] - blank_right_outputs[name][:, :4])
                    .abs().max().detach().cpu()
                )
                right_blank_change = float(
                    (baseline[name][:, 4:] - blank_left_outputs[name][:, 4:])
                    .abs().max().detach().cpu()
                )
                left_shuffle_change = float(
                    (baseline[name][:, :4] - shuffle_right_outputs[name][:, :4])
                    .abs().max().detach().cpu()
                )
                right_shuffle_change = float(
                    (baseline[name][:, 4:] - shuffle_left_outputs[name][:, 4:])
                    .abs().max().detach().cpu()
                )
                changes['blank_opposite_camera']['left'][name] = max(
                    changes['blank_opposite_camera']['left'][name],
                    left_blank_change,
                )
                changes['blank_opposite_camera']['right'][name] = max(
                    changes['blank_opposite_camera']['right'][name],
                    right_blank_change,
                )
                changes['shuffle_opposite_camera']['left'][name] = max(
                    changes['shuffle_opposite_camera']['left'][name],
                    left_shuffle_change,
                )
                changes['shuffle_opposite_camera']['right'][name] = max(
                    changes['shuffle_opposite_camera']['right'][name],
                    right_shuffle_change,
                )
                baseline_parts[name].append(baseline[name].detach().float().cpu())
                swapped_parts[name].append(swapped[name].detach().float().cpu())
            for name, value in targets.items():
                target_parts[name].append(value.detach().cpu())

    baseline_predictions = {
        name: torch_module.cat(parts, dim=0) for name, parts in baseline_parts.items()
    }
    swapped_predictions = {
        name: torch_module.cat(parts, dim=0) for name, parts in swapped_parts.items()
    }
    aggregated_targets = {
        name: torch_module.cat(parts, dim=0) for name, parts in target_parts.items()
    }
    length_matrix = [
        [topology_lengths[side][segment] for segment in SEGMENT_CLASSES]
        for side in SIDES
    ]
    baseline_metrics = training_api.compute_visual_training_v4_metrics(
        baseline_predictions,
        aggregated_targets,
        segment_class_names=SEGMENT_CLASSES,
        segment_lengths_by_side=length_matrix,
    )
    swapped_metrics = training_api.compute_visual_training_v4_metrics(
        swapped_predictions,
        aggregated_targets,
        segment_class_names=SEGMENT_CLASSES,
        segment_lengths_by_side=length_matrix,
    )
    for scenario in ('blank_opposite_camera', 'shuffle_opposite_camera'):
        scenario_values = [
            value
            for side in SIDES
            for value in changes[scenario][side].values()
        ]
        changes[scenario]['maximum_own_side_output_change'] = max(
            scenario_values, default=0.0
        )
    maximum_cross_side_change = max(
        float(changes[scenario]['maximum_own_side_output_change'])
        for scenario in ('blank_opposite_camera', 'shuffle_opposite_camera')
    )
    return {
        'sample_count': len(records),
        **changes,
        'maximum_cross_side_output_change': maximum_cross_side_change,
        'effective_shuffle_batches': effective_shuffle_batches,
        'camera_order_swap': {
            'baseline_segment_top1': float(
                baseline_metrics['segment_top1_accuracy']
            ),
            'swapped_segment_top1': float(
                swapped_metrics['segment_top1_accuracy']
            ),
            'segment_top1_drop': float(
                baseline_metrics['segment_top1_accuracy']
                - swapped_metrics['segment_top1_accuracy']
            ),
            'baseline_joint_segment_and_ratio_005': float(
                baseline_metrics['joint_localization_accuracy_s_ratio_0_05']
            ),
            'swapped_joint_segment_and_ratio_005': float(
                swapped_metrics['joint_localization_accuracy_s_ratio_0_05']
            ),
        },
    }


def _collect_segment_calibration_tensors(
    model: Any,
    records: Sequence[PairedRecord],
    config: Mapping[str, Any],
    *,
    torch_module: Any,
    device: Any,
    epoch: int = 0,
) -> tuple[Any, Any, Any]:
    if not records:
        raise V4TrainerError('segment calibration records are empty')
    loader = make_loader(
        records,
        config,
        torch_module=torch_module,
        training=False,
        epoch=max(0, epoch),
    )
    amp_enabled = bool(
        config['training'].get('automatic_mixed_precision', False)
        and device.type == 'cuda'
    )
    logits_parts: list[Any] = []
    target_parts: list[Any] = []
    mask_parts: list[Any] = []
    model.eval()
    with torch_module.inference_mode():
        for batch in loader:
            image = batch['image'].to(device, non_blocking=True)
            with _autocast_context(torch_module, device, amp_enabled):
                logits = model(image)['segment_logits']
            logits_parts.append(logits.detach().float().cpu())
            target_parts.append(batch['segment'].detach().cpu())
            mask_parts.append(batch['visibility_mask'].detach().cpu())
    return (
        torch_module.cat(logits_parts, dim=0),
        torch_module.cat(target_parts, dim=0),
        torch_module.cat(mask_parts, dim=0),
    )


def fit_validation_segment_calibration(
    model: Any,
    records: Sequence[PairedRecord],
    config: Mapping[str, Any],
    *,
    torch_module: Any,
    device: Any,
    epoch: int = 0,
) -> dict[str, Any]:
    """Fit one validation-only segment temperature and selective curve."""

    if not records or any(record.role != 'validation' for record in records):
        raise V4TrainerError('segment temperature fitting requires validation records')
    try:
        import room_315_visual_calibration_v4 as calibration_api
    except Exception as exc:
        raise V4TrainerError(f'cannot import V4 calibration API: {exc}') from exc
    logits, targets, visibility = _collect_segment_calibration_tensors(
        model,
        records,
        config,
        torch_module=torch_module,
        device=device,
        epoch=epoch,
    )
    calibration_config = config['evaluation']['segment_calibration']
    try:
        return calibration_api.compute_segment_calibration_report(
            logits,
            targets,
            visibility,
            coverages=calibration_config['coverage_targets'],
            ece_bins=int(calibration_config['ece_bins']),
            min_temperature=float(calibration_config['minimum_temperature']),
            max_temperature=float(calibration_config['maximum_temperature']),
            grid_size=int(calibration_config['grid_size']),
            refinement_steps=int(calibration_config['refinement_steps']),
            data_role='validation',
        )
    except Exception as exc:
        raise V4TrainerError(f'validation segment calibration failed: {exc}') from exc


def evaluate_fixed_segment_calibration(
    model: Any,
    records: Sequence[PairedRecord],
    config: Mapping[str, Any],
    *,
    temperature: float,
    data_role: str,
    torch_module: Any,
    device: Any,
    epoch: int = 0,
) -> dict[str, Any]:
    """Evaluate a validation-fitted temperature without refitting it."""

    if data_role not in {'validation', 'canary'}:
        raise V4TrainerError('fixed calibration evaluation allows validation/canary only')
    if not records or any(record.role != data_role for record in records):
        raise V4TrainerError(f'fixed calibration records must all have role {data_role}')
    try:
        import room_315_visual_calibration_v4 as calibration_api
    except Exception as exc:
        raise V4TrainerError(f'cannot import V4 calibration API: {exc}') from exc
    logits, targets, visibility = _collect_segment_calibration_tensors(
        model,
        records,
        config,
        torch_module=torch_module,
        device=device,
        epoch=epoch,
    )
    calibration_config = config['evaluation']['segment_calibration']
    try:
        return calibration_api.evaluate_segment_calibration_at_temperature(
            logits,
            targets,
            visibility,
            temperature=float(temperature),
            coverages=calibration_config['coverage_targets'],
            ece_bins=int(calibration_config['ece_bins']),
            data_role=data_role,
            source_temperature_role='validation',
        )
    except Exception as exc:
        raise V4TrainerError(f'fixed segment calibration evaluation failed: {exc}') from exc


def _write_json_exclusive(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open('x', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write('\n')
    except FileExistsError as exc:
        raise V4TrainerError(f'refusing to overwrite immutable artifact: {path}') from exc


def _replace_json_atomically(path: Path, value: Any) -> None:
    """Replace an existing JSON state file without exposing a partial write."""

    if not path.is_file():
        raise V4TrainerError(f'cannot replace missing JSON state file: {path}')
    temporary = path.with_name(
        f'.{path.name}.{os.getpid()}.{time.time_ns()}.tmp'
    )
    try:
        with temporary.open('x', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _finalize_run_metadata(
    path: Path,
    initial_metadata: Mapping[str, Any],
    final_report: Mapping[str, Any],
) -> dict[str, Any]:
    """Move run metadata from running to completed after the final report exists."""

    current = _read_json_object(path)
    if current != dict(initial_metadata) or current.get('status') != 'running':
        raise V4TrainerError(
            'run metadata changed unexpectedly before completion; refusing replacement'
        )
    selected = final_report.get('selected_checkpoint')
    if final_report.get('status') != 'completed' or not isinstance(selected, Mapping):
        raise V4TrainerError('cannot finalize metadata without a completed selected run')
    completed = {
        **current,
        'status': 'completed',
        'completed_unix_s': time.time(),
        'runtime_s': float(final_report['runtime_s']),
        'epochs_completed': int(final_report['epochs_completed']),
        'selected_checkpoint': {
            'epoch': int(selected['epoch']),
            'filename': str(selected['filename']),
            'sha256': str(selected['sha256']),
        },
        'validation_acceptance_status': str(
            final_report['validation_acceptance_status']
        ),
        'promotion_status': str(final_report['promotion_status']),
        'canary_loaded': bool(final_report['canary_loaded']),
        'test_loaded': bool(final_report['test_loaded']),
        'automatic_runtime_switch': bool(
            final_report['automatic_runtime_switch']
        ),
    }
    _replace_json_atomically(path, completed)
    return completed


def _reserve_canary_attempt(
    config: Mapping[str, Any],
    completed_run: Mapping[str, Any],
    checkpoint_path: Path,
    checkpoint_sha256: str,
    output_path: Path,
    baseline_path: Path,
    baseline_sha256: str,
    coverage_contract: Mapping[str, Any],
) -> dict[str, Any]:
    """Reserve the only V4 attempt for this checkpoint/Canary pair."""

    dataset_fingerprint = str(
        coverage_contract.get('canary_dataset_fingerprint_sha256') or ''
    ).casefold()
    if not re.fullmatch(r'[0-9a-f]{64}', dataset_fingerprint):
        raise V4TrainerError('verified canonical Canary fingerprint is unavailable')
    key_payload = {
        'checkpoint_sha256': checkpoint_sha256,
        'canary_dataset_fingerprint_sha256': dataset_fingerprint,
    }
    attempt_key = hashlib.sha256(
        canonical_json(key_payload).encode('utf-8')
    ).hexdigest()
    output_root = assert_not_test_path(
        config['output_root'], context='V4 output root'
    ).resolve()
    expected_output = output_root / f'canary_v4_{attempt_key[:16]}_attempt1'
    if output_path.resolve() != expected_output:
        raise V4TrainerError(
            'Canary output must be derived from the immutable attempt key: '
            f'{expected_output}'
        )
    training_run_root = checkpoint_path.resolve().parent.parent
    source_files = {
        name: file_fingerprint(SCRIPT_DIR / name)
        for name in (
            'room_315_vla_train_v4.py',
            'room_315_visual_acceptance_v4.py',
            'room_315_visual_calibration_v4.py',
            'room_315_visual_contract_v4.py',
            'room_315_visual_model_v4.py',
            'room_315_visual_training_v4.py',
        )
    }
    reservation = {
        'schema_version': CANARY_ATTEMPT_SCHEMA_VERSION,
        'state': 'reserved_immutable',
        'attempt_key': attempt_key,
        'reserved_unix_s': time.time(),
        'checkpoint': file_fingerprint(checkpoint_path),
        'checkpoint_sha256': checkpoint_sha256,
        'canary_dataset_fingerprint_sha256': dataset_fingerprint,
        'canary_finalization_sha256': coverage_contract[
            'canary_finalization_sha256'
        ],
        'baseline_artifact': {
            **file_fingerprint(baseline_path),
            'expected_sha256': baseline_sha256,
        },
        'coverage_contract_artifact': coverage_contract['artifact'],
        'source_training_final_report': file_fingerprint(
            training_run_root / 'final_report.json'
        ),
        'effective_config_sha256': completed_run['effective_config_sha256'],
        'evaluator_sources': source_files,
        'output': str(output_path.resolve()),
        'prior_exposure_acknowledged': (
            coverage_contract.get('prior_exposure_acknowledged') is True
        ),
        'prior_canary_artifact_sha256': coverage_contract.get(
            'prior_canary_artifact_sha256'
        ),
        'canary_used_for_checkpoint_selection': False,
        'test_loaded': False,
        'automatic_runtime_switch': False,
    }
    if reservation['prior_exposure_acknowledged'] is not True:
        raise V4TrainerError('Canary prior exposure must be explicitly acknowledged')
    ledger_root = output_root / 'canary_attempt_ledger_v1'
    reservation_path = ledger_root / f'{attempt_key}.reserved.json'
    completion_path = ledger_root / f'{attempt_key}.completed.json'
    if completion_path.exists():
        raise V4TrainerError(
            f'Canary attempt already completed for immutable key {attempt_key}'
        )
    _write_json_exclusive(reservation_path, reservation)
    return {
        'attempt_key': attempt_key,
        'reservation_path': str(reservation_path),
        'reservation_sha256': sha256_file(reservation_path),
        'completion_path': str(completion_path),
        'expected_output': str(expected_output),
    }


def _complete_canary_attempt(
    reservation: Mapping[str, Any],
    output: Path,
) -> dict[str, Any]:
    """Write the immutable success event and bind every Canary artifact."""

    artifact_names = (
        'approved_v3_canary_baseline.json',
        'canary_acceptance.json',
        'canary_attempt_started.json',
        'canary_camera_counterfactuals.json',
        'canary_coverage_audit.json',
        'canary_coverage_contract.json',
        'canary_disjoint_audit.json',
        'canary_image_pair_manifest.json',
        'canary_input_fingerprint.json',
        'canary_metrics.json',
        'canary_segment_calibration.json',
        'effective_config.json',
        'final_report.json',
    )
    artifacts = {
        name: file_fingerprint(output / name) for name in artifact_names
    }
    completed = {
        'schema_version': CANARY_ATTEMPT_SCHEMA_VERSION,
        'state': 'completed_immutable',
        'attempt_key': reservation['attempt_key'],
        'completed_unix_s': time.time(),
        'reservation': {
            'path': reservation['reservation_path'],
            'sha256': reservation['reservation_sha256'],
        },
        'output': str(output.resolve()),
        'artifacts': artifacts,
        'canary_used_for_checkpoint_selection': False,
        'test_loaded': False,
        'automatic_runtime_switch': False,
    }
    completion_path = Path(str(reservation['completion_path']))
    _write_json_exclusive(completion_path, completed)
    return {
        'path': str(completion_path),
        'sha256': sha256_file(completion_path),
        'attempt_key': reservation['attempt_key'],
    }


def verify_completed_canary_handoff(
    final_report_path: Path | str,
) -> dict[str, Any]:
    """Accept a Canary handoff only when its external completion ledger binds it."""

    report_path = assert_not_test_path(
        final_report_path, context='Canary final report'
    ).resolve()
    report = _read_json_object(report_path)
    attempt = report.get('canary_attempt')
    if not isinstance(attempt, Mapping):
        raise V4TrainerError('Canary final report lacks attempt metadata')
    completion_path = assert_not_test_path(
        attempt.get('completion_path', ''), context='Canary completion ledger'
    ).resolve()
    completion = _read_json_object(completion_path)
    final_artifact = (
        completion.get('artifacts', {}).get('final_report.json')
        if isinstance(completion.get('artifacts'), Mapping)
        else None
    )
    checks = {
        'report_completed': report.get('status') == 'completed',
        'acceptance_passed': report.get('acceptance_status') == 'passed',
        'promotion_manual_only': (
            report.get('promotion_status') == 'eligible_for_manual_runtime_review'
        ),
        'completion_required': (
            attempt.get('completion_ledger_required_for_trust') is True
        ),
        'completion_state': completion.get('state') == 'completed_immutable',
        'attempt_key': (
            completion.get('attempt_key') == attempt.get('attempt_key')
        ),
        'final_report_sha256': (
            isinstance(final_artifact, Mapping)
            and final_artifact.get('sha256') == sha256_file(report_path)
        ),
        'canary_loaded': report.get('canary_loaded') is True,
        'selection_not_performed': (
            report.get('checkpoint_selection_performed') is False
            and report.get('canary_used_for_checkpoint_selection') is False
        ),
        'test_not_loaded': (
            report.get('test_loaded') is False
            and completion.get('test_loaded') is False
        ),
        'runtime_not_switched': (
            report.get('automatic_runtime_switch') is False
            and completion.get('automatic_runtime_switch') is False
        ),
    }
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise V4TrainerError(f'Canary handoff is not trustworthy: {failed}')
    return {
        'trusted': True,
        'checks': checks,
        'final_report': file_fingerprint(report_path),
        'completion_ledger': file_fingerprint(completion_path),
        'attempt_key': attempt['attempt_key'],
    }


def _append_jsonl(path: Path, value: Any) -> None:
    with path.open('a', encoding='utf-8') as stream:
        stream.write(canonical_json(value) + '\n')


def _prepare_output(path: Path | str) -> Path:
    output = assert_not_test_path(path, context='output directory').resolve()
    if output.exists():
        raise V4TrainerError(f'refusing to overwrite immutable output: {output}')
    output.parent.mkdir(parents=True, exist_ok=True)
    return output


def _resolve_device(torch_module: Any, configured: str, override: str | None) -> Any:
    name = str(override or configured).strip().lower()
    device = torch_module.device(name)
    if device.type == 'cuda' and not torch_module.cuda.is_available():
        raise V4TrainerError('configuration requires CUDA but CUDA is unavailable')
    return device


def _make_grad_scaler(
    torch_module: Any,
    enabled: bool,
    initial_scale: float,
) -> Any:
    try:
        return torch_module.amp.GradScaler(
            'cuda',
            enabled=enabled,
            init_scale=float(initial_scale),
        )
    except (AttributeError, TypeError):
        return torch_module.cuda.amp.GradScaler(
            enabled=enabled,
            init_scale=float(initial_scale),
        )


def _save_torch_exclusive(torch_module: Any, value: Any, path: Path) -> None:
    if path.exists():
        raise V4TrainerError(f'refusing to overwrite checkpoint: {path}')
    temporary = path.with_name(path.name + '.partial')
    if temporary.exists():
        raise V4TrainerError(f'stale partial checkpoint exists: {temporary}')
    torch_module.save(value, temporary)
    temporary.rename(path)


def train_v4(
    config_path: Path | str,
    output_path: Path | str,
    *,
    device_override: str | None = None,
    decode_images_preflight: bool = False,
) -> dict[str, Any]:
    """Train V4 with immutable epoch checkpoints and validation-only selection."""

    output = _prepare_output(output_path)
    bundle = preflight_configuration(
        config_path,
        decode_images=decode_images_preflight,
    )
    config = bundle.config
    torch_module, torchvision_module, training_api = require_training_stack()
    seed = int(config['training']['seed'])
    set_deterministic(torch_module, seed)
    device = _resolve_device(
        torch_module,
        str(config['training']['device']),
        device_override,
    )
    model = build_configured_model(config, torch_module, torchvision_module)
    initialization_report = initialize_verified_v3_backbone(
        model, config, torch_module
    )
    model.to(device)
    first_stage = configure_training_stage(
        model,
        config['training'],
        epoch=1,
        torch_module=torch_module,
    )
    optimizer = build_staged_optimizer(model, config['training'], torch_module)

    output.mkdir()
    (output / 'checkpoints').mkdir()
    (output / 'selection_traces').mkdir()
    _write_json_exclusive(output / 'effective_config.json', config)
    _write_json_exclusive(output / 'input_fingerprints.json', bundle.report)
    class_counts = bundle.report['class_counts']
    class_weights = class_weights_by_side(class_counts, config['loss'])
    topology_lengths = bundle.report['topology_lengths_by_side']
    _write_json_exclusive(output / 'class_counts_train_only.json', class_counts)
    _write_json_exclusive(
        output / 'public_topology_contract.json',
        bundle.report['public_topology_contract'],
    )
    _write_json_exclusive(output / 'initialization_report.json', initialization_report)
    run_metadata = {
        'schema_version': RUN_SCHEMA_VERSION,
        'status': 'running',
        'started_unix_s': time.time(),
        'seed': seed,
        'device': str(device),
        'torch': str(torch_module.__version__),
        'torchvision': str(torchvision_module.__version__),
        'automatic_mixed_precision': bool(
            config['training']['automatic_mixed_precision'] and device.type == 'cuda'
        ),
        'canary_loaded': False,
        'test_loaded': False,
        'initial_stage': first_stage,
    }
    _write_json_exclusive(output / 'run_metadata.json', run_metadata)

    optimizer.zero_grad(set_to_none=True)
    amp_enabled = bool(
        config['training']['automatic_mixed_precision'] and device.type == 'cuda'
    )
    scaler = _make_grad_scaler(
        torch_module,
        amp_enabled,
        float(config['training']['amp_initial_scale']),
    )
    topology_distances = torch_module.as_tensor(
        topology_distance_matrix(), dtype=torch_module.float32, device=device
    )
    accumulation_steps = int(config['training']['gradient_accumulation_steps'])
    clip_norm = float(config['training']['gradient_clip_norm'])
    enabled_specs = [
        spec for spec in config['data']['train_sources']
        if bool(spec.get('enabled', True))
    ]
    best_key: tuple[float, float, float] | None = None
    best_checkpoint: dict[str, Any] | None = None
    patience = 0
    history: list[dict[str, Any]] = []
    start = time.time()

    for epoch in range(1, int(config['training']['epochs']) + 1):
        stage_report = configure_training_stage(
            model,
            config['training'],
            epoch=epoch,
            torch_module=torch_module,
        )
        plan = build_epoch_selection(
            bundle.train_by_source,
            enabled_specs,
            config['sampling'],
            seed=seed,
            epoch=epoch,
        )
        selected_records = [
            replace(
                bundle.train_by_source[item['source']][int(item['source_index'])],
                draw_index=int(item['draw_index']),
                resample_occurrence=int(item['resample_occurrence']),
            )
            for item in plan
        ]
        selection_report = {
            'epoch': epoch,
            'by_source': dict(sorted(Counter(item['source'] for item in plan).items())),
            'selection_sha256': hashlib.sha256(
                canonical_json(plan).encode('utf-8')
            ).hexdigest(),
            'selection': plan,
        }
        _write_json_exclusive(
            output / 'selection_traces' / f'epoch_{epoch:03d}.json',
            selection_report,
        )
        loader = make_loader(
            selected_records,
            config,
            torch_module=torch_module,
            training=True,
            epoch=epoch,
        )
        loss_accumulator: defaultdict[str, float] = defaultdict(float)
        visible_weight_total = 0
        skipped_nonfinite_amp_steps = 0
        optimizer.zero_grad(set_to_none=True)
        epoch_start = time.time()
        for batch_index, batch in enumerate(loader, 1):
            image = batch['image'].to(device, non_blocking=True)
            targets = _target_device_dict(batch, device)
            with _autocast_context(torch_module, device, amp_enabled):
                predictions = model(image)
                losses = compute_configured_loss(
                    predictions,
                    targets,
                    class_weights=class_weights,
                    topology_distances=topology_distances,
                    loss_config=config['loss'],
                    torch_module=torch_module,
                    training_api=training_api,
                )
                if not bool(torch_module.isfinite(losses['loss']).all()):
                    raise V4TrainerError(
                        f'non-finite training loss at epoch {epoch}, batch {batch_index}'
                    )
                group_start = ((batch_index - 1) // accumulation_steps) * accumulation_steps
                current_group_size = min(
                    accumulation_steps,
                    len(loader) - group_start,
                )
                backward_loss = losses['loss'] / float(current_group_size)
            scaler.scale(backward_loss).backward()
            should_step = (
                batch_index % accumulation_steps == 0
                or batch_index == len(loader)
            )
            if should_step:
                scaler.unscale_(optimizer)
                trainable_parameters = [
                    parameter
                    for parameter in model.parameters()
                    if parameter.requires_grad and parameter.grad is not None
                ]
                gradients_finite = all(
                    bool(torch_module.isfinite(parameter.grad).all())
                    for parameter in trainable_parameters
                )
                if gradients_finite:
                    torch_module.nn.utils.clip_grad_norm_(
                        trainable_parameters,
                        max_norm=clip_norm,
                        error_if_nonfinite=True,
                    )
                elif amp_enabled:
                    # GradScaler recorded the overflow during unscale_ and will
                    # skip optimizer.step while reducing its scale in update().
                    skipped_nonfinite_amp_steps += 1
                else:
                    raise V4TrainerError(
                        f'non-finite gradients at epoch {epoch}, batch {batch_index}'
                    )
                scaler.step(optimizer)
                scaler.update()
                optimizer.zero_grad(set_to_none=True)
            visible_count = int(losses['visible_count'])
            if visible_count <= 0:
                raise V4TrainerError(
                    f'training batch has no visible targets at epoch {epoch}, '
                    f'batch {batch_index}'
                )
            for name in LOSS_SCALAR_KEYS:
                loss_accumulator[name] += (
                    float(losses[name].detach().cpu()) * visible_count
                )
            visible_weight_total += visible_count

        validation_metrics = evaluate_model(
            model,
            bundle.validation,
            config,
            torch_module=torch_module,
            training_api=training_api,
            device=device,
            class_weights=class_weights,
            topology_lengths=topology_lengths,
            epoch=epoch,
        )
        current_key = checkpoint_selection_key(validation_metrics)
        checkpoint_name = f'epoch_{epoch:03d}.pt'
        checkpoint_path = output / 'checkpoints' / checkpoint_name
        state = {
            'schema_version': CHECKPOINT_SCHEMA_VERSION,
            'model_kind': V4_MODEL_KIND,
            'slot_order': list(FIXED_IDENTITIES),
            'segment_order': list(SEGMENT_CLASSES),
            'epoch': epoch,
            'model_state_dict': model.state_dict(),
            'training_resume_supported': False,
            'validation_metrics': validation_metrics,
            'validation_selection_key': list(current_key),
            'checkpoint_selection_role': 'validation_only',
            'class_counts_train_only': class_counts,
            'class_weights_by_side': class_weights.tolist(),
            'topology_lengths_by_side': topology_lengths,
            'public_topology_contract': bundle.report['public_topology_contract'],
            'topology_length_mapping_fingerprint_sha256': (
                bundle.report['public_topology_contract']['fingerprint_sha256']
            ),
            'initialization_checkpoint_sha256': initialization_report['checkpoint_sha256'],
            'config_file_sha256': bundle.report['configuration']['sha256'],
            'effective_config_sha256': bundle.report['effective_config_sha256'],
            'stage': stage_report,
            'canary_seen': False,
            'test_seen': False,
        }
        _save_torch_exclusive(torch_module, state, checkpoint_path)
        checkpoint_sha = sha256_file(checkpoint_path)
        improved = best_key is None or current_key > best_key
        if improved:
            best_key = current_key
            patience = 0
            best_checkpoint = {
                'filename': checkpoint_name,
                'path': str(checkpoint_path),
                'sha256': checkpoint_sha,
                'epoch': epoch,
                'validation_selection_key': list(current_key),
            }
        else:
            patience += 1
        entry = {
            'epoch': epoch,
            'stage': stage_report,
            'train_losses': {
                name: value / max(1, visible_weight_total)
                for name, value in sorted(loss_accumulator.items())
            },
            'train_loss_aggregation': 'visible-slot-weighted_batch_means',
            'train_visible_target_count': visible_weight_total,
            'skipped_nonfinite_amp_optimizer_steps': skipped_nonfinite_amp_steps,
            'validation': validation_metrics,
            'selection_key': list(current_key),
            'selected_as_best': improved,
            'early_stopping_patience_used': patience,
            'checkpoint': {
                'filename': checkpoint_name,
                'sha256': checkpoint_sha,
            },
            'epoch_runtime_s': time.time() - epoch_start,
        }
        history.append(entry)
        _append_jsonl(output / 'history.jsonl', entry)
        if patience >= int(config['training']['early_stopping_patience']):
            break

    if best_checkpoint is None:
        raise V4TrainerError('training completed without a selected checkpoint')
    selected_path = output / 'checkpoints' / best_checkpoint['filename']
    if sha256_file(selected_path) != best_checkpoint['sha256']:
        raise V4TrainerError('selected checkpoint changed before final evaluation')
    selected_state = _torch_load(torch_module, selected_path, map_location='cpu')
    model.load_state_dict(selected_state['model_state_dict'], strict=True)
    final_validation = evaluate_model(
        model,
        bundle.validation,
        config,
        torch_module=torch_module,
        training_api=training_api,
        device=device,
        class_weights=class_weights,
        topology_lengths=topology_lengths,
        epoch=int(best_checkpoint['epoch']),
    )
    reproduced_key = checkpoint_selection_key(final_validation)
    recorded_key = tuple(best_checkpoint['validation_selection_key'])
    if len(recorded_key) != len(reproduced_key) or any(
        not math.isclose(actual, expected, rel_tol=1.0e-9, abs_tol=1.0e-12)
        for actual, expected in zip(reproduced_key, recorded_key)
    ):
        raise V4TrainerError(
            'selected checkpoint validation metrics are not reproducible '
            f'within tolerance: {reproduced_key} != {recorded_key}'
        )
    validation_counterfactuals = evaluate_camera_counterfactuals(
        model,
        bundle.validation,
        config,
        torch_module=torch_module,
        training_api=training_api,
        device=device,
        topology_lengths=topology_lengths,
        epoch=int(best_checkpoint['epoch']),
    )
    validation_calibration = fit_validation_segment_calibration(
        model,
        bundle.validation,
        config,
        torch_module=torch_module,
        device=device,
        epoch=int(best_checkpoint['epoch']),
    )
    try:
        import room_315_visual_acceptance_v4 as acceptance_api
    except Exception as exc:
        raise V4TrainerError(f'cannot import V4 acceptance API: {exc}') from exc
    approved_validation_baseline = bundle.report.get(
        'approved_v3_validation_baseline'
    )
    baseline_loaded_accuracy = (
        approved_validation_baseline.get('loaded_accuracy')
        if isinstance(approved_validation_baseline, Mapping)
        else None
    )
    try:
        validation_acceptance = acceptance_api.evaluate_visual_acceptance_v4(
            final_validation,
            config['pilot_acceptance_gates'],
            counterfactual_report=validation_counterfactuals,
            approved_v3_loaded_accuracy=baseline_loaded_accuracy,
        )
    except Exception as exc:
        raise V4TrainerError(f'validation acceptance evaluation failed: {exc}') from exc
    pending_validation_gates = sorted(
        gate_id
        for gate_id, item in validation_acceptance['per_gate'].items()
        if item['status'] == 'pending'
    )
    final_validation['acceptance_gates_evaluated'] = True
    final_validation['pending_evaluations'] = pending_validation_gates
    _write_json_exclusive(output / 'history.json', history)
    _write_json_exclusive(output / 'final_validation_metrics.json', final_validation)
    _write_json_exclusive(
        output / 'validation_camera_counterfactuals.json',
        validation_counterfactuals,
    )
    _write_json_exclusive(
        output / 'validation_segment_calibration.json',
        validation_calibration,
    )
    _write_json_exclusive(
        output / 'validation_acceptance.json',
        validation_acceptance,
    )
    validation_acceptance_status = str(validation_acceptance['status'])
    promotion_status = (
        'pending_post_selection_canary'
        if validation_acceptance_status == 'passed'
        else 'failed_validation_acceptance'
        if validation_acceptance_status == 'failed'
        else 'pending_validation_evidence'
    )
    final_report = {
        'schema_version': RUN_SCHEMA_VERSION,
        'status': 'completed',
        'runtime_s': time.time() - start,
        'epochs_completed': len(history),
        'selected_checkpoint': best_checkpoint,
        'checkpoint_selection': {
            'role': 'validation_only',
            'lexicographic_metrics': [
                'planning_selection_score',
                'worst_side_x_segment_recall',
                '-correct_segment_s_m_p95_m',
            ],
            'key': list(checkpoint_selection_key(final_validation)),
        },
        'final_validation_metrics': final_validation,
        'validation_camera_counterfactuals': validation_counterfactuals,
        'validation_segment_calibration': validation_calibration,
        'validation_acceptance': validation_acceptance,
        'approved_v3_validation_baseline': approved_validation_baseline,
        'public_topology_contract': bundle.report['public_topology_contract'],
        'topology_length_mapping_fingerprint_sha256': (
            bundle.report['public_topology_contract']['fingerprint_sha256']
        ),
        'config_file_sha256': bundle.report['configuration']['sha256'],
        'effective_config_sha256': bundle.report['effective_config_sha256'],
        'support_audit': {
            'train': class_counts,
            'validation': bundle.report['validation_support'],
        },
        'canary_loaded': False,
        'canary_used_for_selection': False,
        'test_loaded': False,
        'automatic_runtime_switch': False,
        'validation_acceptance_status': validation_acceptance_status,
        'promotion_status': promotion_status,
        'acceptance_status': promotion_status,
        'pending_evaluations': [
            *pending_validation_gates,
            'explicit_post_selection_canary_evaluation',
        ],
    }
    _write_json_exclusive(output / 'final_report.json', final_report)
    _finalize_run_metadata(
        output / 'run_metadata.json',
        run_metadata,
        final_report,
    )
    return final_report


def _verify_completed_selected_checkpoint(
    checkpoint_path: Path,
    expected_sha256: str,
) -> tuple[dict[str, Any], str]:
    checkpoint = assert_not_test_path(
        checkpoint_path, context='Canary checkpoint'
    ).resolve()
    expected = str(expected_sha256).strip().casefold()
    if not re.fullmatch(r'[0-9a-f]{64}', expected):
        raise V4TrainerError('Canary checkpoint expected SHA-256 is required')
    actual = sha256_file(checkpoint)
    if actual != expected:
        raise V4TrainerError(
            f'Canary checkpoint SHA-256 mismatch: {actual} != {expected}'
        )
    run_root = checkpoint.parent.parent
    final_report_path = run_root / 'final_report.json'
    if not final_report_path.is_file():
        raise V4TrainerError(
            'Canary evaluation requires a completed V4 training final_report.json'
        )
    report = _read_json_object(final_report_path)
    selected = report.get('selected_checkpoint')
    if report.get('status') != 'completed' or not isinstance(selected, dict):
        raise V4TrainerError('V4 training run is not marked completed')
    if (
        str(selected.get('filename')) != checkpoint.name
        or str(selected.get('sha256')).casefold() != actual
    ):
        raise V4TrainerError(
            'Canary checkpoint is not the validation-selected checkpoint '
            'declared by the completed training run'
        )
    selection = report.get('checkpoint_selection') or {}
    if selection.get('role') != 'validation_only':
        raise V4TrainerError('completed run did not select using validation only')
    if report.get('canary_used_for_selection') is not False:
        raise V4TrainerError('completed run does not prove Canary isolation')
    if report.get('validation_acceptance_status') != 'passed':
        raise V4TrainerError(
            'Canary remains locked until validation acceptance status is passed'
        )
    calibration = report.get('validation_segment_calibration')
    if not isinstance(calibration, Mapping):
        raise V4TrainerError('completed run lacks validation segment calibration')
    temperature = _finite_float(
        calibration.get('temperature'),
        'validation segment temperature',
    )
    if temperature <= 0.0:
        raise V4TrainerError('validation segment temperature must be positive')
    return report, actual


def evaluate_canary_v4(
    config_path: Path | str,
    checkpoint_path: Path | str,
    checkpoint_sha256: str,
    output_path: Path | str,
    *,
    approved_v3_canary_baseline_path: Path | str,
    approved_v3_canary_baseline_sha256: str,
    canary_coverage_contract_path: Path | str,
    canary_coverage_contract_sha256: str,
    device_override: str | None = None,
    decode_images: bool = False,
) -> dict[str, Any]:
    """Evaluate Canary explicitly; never use its metrics for model selection."""

    output = _prepare_output(output_path)
    config = load_v4_config(config_path)
    checkpoint_candidate = Path(checkpoint_path).resolve()
    completed_run, verified_sha = _verify_completed_selected_checkpoint(
        checkpoint_candidate, checkpoint_sha256
    )
    training_run_root = checkpoint_candidate.parent.parent
    current_effective_config_sha = hashlib.sha256(
        canonical_json(config).encode('utf-8')
    ).hexdigest()
    if current_effective_config_sha != completed_run.get('effective_config_sha256'):
        raise V4TrainerError(
            'current effective configuration hash differs from the completed run'
        )
    source_effective_config = _read_json_object(
        training_run_root / 'effective_config.json'
    )
    if source_effective_config != config:
        raise V4TrainerError(
            'current effective configuration differs from the immutable '
            'training-run configuration snapshot'
        )
    canary_spec = config['data']['canary']
    preliminary_canary_fingerprint = {
        'name': canary_spec['name'],
        'role': 'canary',
        'expected_records': int(canary_spec['expected_rows']),
        'paired_records': int(canary_spec['expected_rows']),
        'image_references': int(canary_spec['expected_rows']) * len(CAMERAS),
        'rows': file_fingerprint(canary_spec['rows']),
        'labels': file_fingerprint(canary_spec['labels']),
        'dataset_root': str(Path(canary_spec['dataset_root']).resolve()),
    }
    canary_coverage_contract = load_and_verify_canary_coverage_contract(
        canary_coverage_contract_path,
        canary_coverage_contract_sha256,
        config,
        preliminary_canary_fingerprint,
        training_run_root,
    )
    baseline_path = assert_not_test_path(
        approved_v3_canary_baseline_path,
        context='approved V3 Canary baseline',
    ).resolve()
    expected_baseline_sha = str(
        approved_v3_canary_baseline_sha256
    ).strip().casefold()
    if (
        not re.fullmatch(r'[0-9a-f]{64}', expected_baseline_sha)
        or sha256_file(baseline_path) != expected_baseline_sha
    ):
        raise V4TrainerError('approved V3 Canary baseline pre-lock SHA mismatch')
    reservation = _reserve_canary_attempt(
        config,
        completed_run,
        checkpoint_candidate,
        verified_sha,
        output,
        baseline_path,
        expected_baseline_sha,
        canary_coverage_contract,
    )
    output.mkdir()
    _write_json_exclusive(
        output / 'canary_attempt_started.json',
        {
            'schema_version': CANARY_ATTEMPT_SCHEMA_VERSION,
            'state': 'inference_started',
            'attempt_key': reservation['attempt_key'],
            'reservation': {
                'path': reservation['reservation_path'],
                'sha256': reservation['reservation_sha256'],
            },
            'started_unix_s': time.time(),
            'canary_loaded': False,
            'canary_used_for_checkpoint_selection': False,
            'test_loaded': False,
            'automatic_runtime_switch': False,
        },
    )
    canary_records, canary_fingerprint = load_paired_source(
        canary_spec,
        role='canary',
        decode_images=decode_images,
    )
    for name in ('rows', 'labels'):
        if (
            canary_fingerprint[name]['sha256']
            != preliminary_canary_fingerprint[name]['sha256']
        ):
            raise V4TrainerError(f'Canary {name} changed after attempt reservation')
    if (
        int(canary_fingerprint['paired_records'])
        != int(preliminary_canary_fingerprint['paired_records'])
    ):
        raise V4TrainerError('Canary record count changed after attempt reservation')
    canary_support = segment_support_report(canary_records, role='canary')
    require_full_segment_support(canary_support)
    approved_canary_baseline = load_and_verify_approved_v3_canary_baseline(
        baseline_path,
        expected_baseline_sha,
        config,
        canary_fingerprint,
        canary_support,
    )
    if (
        approved_canary_baseline['prior_canary_artifact']['sha256']
        != canary_coverage_contract['prior_canary_artifact']['sha256']
    ):
        raise V4TrainerError(
            'V3 baseline and Canary coverage contract cite different prior exposure'
        )
    train_by_source: dict[str, list[PairedRecord]] = {}
    train_fingerprints: dict[str, dict[str, Any]] = {}
    for spec in config['data']['train_sources']:
        if not bool(spec.get('enabled', True)):
            continue
        records, fingerprint = load_paired_source(
            spec, role='train', decode_images=False
        )
        train_by_source[str(spec['name'])] = records
        train_fingerprints[str(spec['name'])] = fingerprint
    validation_records, validation_fingerprint = load_paired_source(
        config['data']['validation'],
        role='validation',
        decode_images=False,
    )
    source_fingerprint_binding = verify_loaded_sources_against_training_fingerprints(
        training_run_root,
        train_fingerprints,
        validation_fingerprint,
    )
    historical_source_image_binding = verify_current_images_against_historical_references(
        train_by_source,
        validation_records,
        canary_coverage_contract,
    )
    train_records = [
        record for records in train_by_source.values() for record in records
    ]
    canary_disjoint_audit = {
        'source_fingerprint_binding': source_fingerprint_binding,
        'historical_source_image_binding': historical_source_image_binding,
        'train_vs_canary': record_groups_disjoint_audit(
            train_records,
            canary_records,
            left_role='train',
            right_role='canary',
        ),
        'validation_vs_canary': record_groups_disjoint_audit(
            validation_records,
            canary_records,
            left_role='validation',
            right_role='canary',
        ),
    }
    canary_image_pair_manifest = verify_images_against_finalization(
        canary_records,
        canary_coverage_contract['canary_finalization_artifact']['path'],
    )
    canary_coverage_audit = verify_canary_trace_coverage(
        canary_records,
        canary_coverage_contract,
    )
    torch_module, torchvision_module, training_api = require_training_stack()
    seed = int(config['training']['seed'])
    set_deterministic(torch_module, seed)
    device = _resolve_device(
        torch_module,
        str(config['training']['device']),
        device_override,
    )
    model = build_configured_model(config, torch_module, torchvision_module).to(device)
    checkpoint = _torch_load(
        torch_module,
        Path(checkpoint_path).resolve(),
        map_location='cpu',
    )
    if not isinstance(checkpoint, Mapping):
        raise V4TrainerError('V4 checkpoint must be an object')
    if checkpoint.get('schema_version') != CHECKPOINT_SCHEMA_VERSION:
        raise V4TrainerError('V4 checkpoint schema is incompatible')
    if checkpoint.get('model_kind') != V4_MODEL_KIND:
        raise V4TrainerError('V4 checkpoint model kind is incompatible')
    if tuple(checkpoint.get('slot_order') or ()) != tuple(FIXED_IDENTITIES):
        raise V4TrainerError('V4 checkpoint fixed slot order is incompatible')
    if tuple(checkpoint.get('segment_order') or ()) != tuple(SEGMENT_CLASSES):
        raise V4TrainerError('V4 checkpoint segment order is incompatible')
    if checkpoint.get('checkpoint_selection_role') != 'validation_only':
        raise V4TrainerError('V4 checkpoint was not selected on validation')
    if checkpoint.get('canary_seen') is not False or checkpoint.get('test_seen') is not False:
        raise V4TrainerError('V4 checkpoint does not prove data isolation')
    current_effective_config_sha = hashlib.sha256(
        canonical_json(config).encode('utf-8')
    ).hexdigest()
    if checkpoint.get('effective_config_sha256') != current_effective_config_sha:
        raise V4TrainerError(
            'current effective preprocessing/model/loss configuration differs '
            'from the selected checkpoint configuration'
        )
    source_effective_config_path = (
        Path(checkpoint_path).resolve().parent.parent / 'effective_config.json'
    )
    source_effective_config = _read_json_object(source_effective_config_path)
    if source_effective_config != config:
        raise V4TrainerError(
            'current effective configuration differs from the immutable '
            'training-run configuration snapshot'
        )
    state = checkpoint.get('model_state_dict')
    if not isinstance(state, Mapping):
        raise V4TrainerError('V4 checkpoint lacks model_state_dict')
    model.load_state_dict(state, strict=True)
    class_weights = checkpoint.get('class_weights_by_side')
    topology_lengths = checkpoint.get('topology_lengths_by_side')
    if class_weights is None or not isinstance(topology_lengths, Mapping):
        raise V4TrainerError('V4 checkpoint lacks train-only statistics')
    current_public_topology = load_public_topology_contract(config)
    canary_topology_label_audit = audit_labels_against_public_topology(
        [canary_records], current_public_topology
    )
    checkpoint_topology_fingerprint = str(
        checkpoint.get('topology_length_mapping_fingerprint_sha256') or ''
    ).casefold()
    if checkpoint_topology_fingerprint != current_public_topology['fingerprint_sha256']:
        raise V4TrainerError(
            'public topology length fingerprint changed since V4 training'
        )
    if checkpoint.get('public_topology_contract') != current_public_topology:
        raise V4TrainerError(
            'checkpoint public topology metadata differs from the canonical contract'
        )
    if topology_lengths != current_public_topology['lengths_by_side']:
        raise V4TrainerError(
            'checkpoint topology mapping differs from the canonical public mapping'
        )
    metrics = evaluate_model(
        model,
        canary_records,
        config,
        torch_module=torch_module,
        training_api=training_api,
        device=device,
        class_weights=class_weights,
        topology_lengths=topology_lengths,
        epoch=int(checkpoint.get('epoch', 0)),
    )
    metrics.pop('selection_key', None)
    metrics['selection_role'] = 'none'
    metrics['used_for_checkpoint_selection'] = False
    canary_counterfactuals = evaluate_camera_counterfactuals(
        model,
        canary_records,
        config,
        torch_module=torch_module,
        training_api=training_api,
        device=device,
        topology_lengths=topology_lengths,
        epoch=int(checkpoint.get('epoch', 0)),
    )
    validation_temperature = float(
        completed_run['validation_segment_calibration']['temperature']
    )
    canary_calibration = evaluate_fixed_segment_calibration(
        model,
        canary_records,
        config,
        temperature=validation_temperature,
        data_role='canary',
        torch_module=torch_module,
        device=device,
        epoch=int(checkpoint.get('epoch', 0)),
    )
    if (
        canary_calibration.get('fit_performed') is not False
        or canary_calibration.get('source_temperature_role') != 'validation'
        or not math.isclose(
            float(canary_calibration.get('temperature')),
            validation_temperature,
            rel_tol=0.0,
            abs_tol=0.0,
        )
    ):
        raise V4TrainerError(
            'Canary calibration must reuse the exact validation temperature '
            'without refitting'
        )
    try:
        import room_315_visual_acceptance_v4 as acceptance_api
    except Exception as exc:
        raise V4TrainerError(f'cannot import V4 acceptance API: {exc}') from exc
    try:
        canary_acceptance = acceptance_api.evaluate_visual_acceptance_v4(
            metrics,
            config['pilot_acceptance_gates'],
            counterfactual_report=canary_counterfactuals,
            approved_v3_loaded_accuracy=float(
                approved_canary_baseline['loaded_accuracy']
            ),
            required_scene_presence_densities=tuple(
                canary_coverage_contract['required_scene_presence_densities']
            ),
        )
    except Exception as exc:
        raise V4TrainerError(f'Canary acceptance evaluation failed: {exc}') from exc
    pending_canary_gates = sorted(
        gate_id
        for gate_id, item in canary_acceptance['per_gate'].items()
        if item['status'] == 'pending'
    )
    metrics['acceptance_gates_evaluated'] = True
    metrics['pending_evaluations'] = pending_canary_gates

    _write_json_exclusive(output / 'effective_config.json', config)
    _write_json_exclusive(output / 'canary_input_fingerprint.json', canary_fingerprint)
    _write_json_exclusive(
        output / 'approved_v3_canary_baseline.json',
        approved_canary_baseline,
    )
    _write_json_exclusive(
        output / 'canary_coverage_contract.json',
        canary_coverage_contract,
    )
    _write_json_exclusive(
        output / 'canary_coverage_audit.json',
        canary_coverage_audit,
    )
    _write_json_exclusive(
        output / 'canary_disjoint_audit.json',
        canary_disjoint_audit,
    )
    _write_json_exclusive(
        output / 'canary_image_pair_manifest.json',
        canary_image_pair_manifest,
    )
    _write_json_exclusive(output / 'canary_metrics.json', metrics)
    _write_json_exclusive(
        output / 'canary_camera_counterfactuals.json',
        canary_counterfactuals,
    )
    _write_json_exclusive(
        output / 'canary_segment_calibration.json',
        canary_calibration,
    )
    _write_json_exclusive(output / 'canary_acceptance.json', canary_acceptance)
    canary_acceptance_status = str(canary_acceptance['status'])
    report = {
        'schema_version': RUN_SCHEMA_VERSION,
        'status': 'completed',
        'mode': 'explicit_post_training_canary_evaluation',
        'checkpoint': {
            'path': str(Path(checkpoint_path).resolve()),
            'sha256': verified_sha,
            'validation_selected': True,
            'source_training_report': str(
                Path(checkpoint_path).resolve().parent.parent / 'final_report.json'
            ),
            'source_selected_epoch': completed_run['selected_checkpoint']['epoch'],
        },
        'metrics': metrics,
        'camera_counterfactuals': canary_counterfactuals,
        'segment_calibration': canary_calibration,
        'acceptance': canary_acceptance,
        'approved_v3_canary_baseline': approved_canary_baseline,
        'canary_coverage_contract': canary_coverage_contract,
        'canary_coverage_audit': canary_coverage_audit,
        'canary_disjoint_audit': canary_disjoint_audit,
        'canary_image_pair_manifest': canary_image_pair_manifest,
        'canary_attempt': {
            'attempt_key': reservation['attempt_key'],
            'reservation_path': reservation['reservation_path'],
            'reservation_sha256': reservation['reservation_sha256'],
            'completion_path': reservation['completion_path'],
            'completion_ledger_required_for_trust': True,
            'one_shot': True,
        },
        'support_audit': {'canary': canary_support},
        'topology_label_audit': canary_topology_label_audit,
        'public_topology_contract': current_public_topology,
        'topology_length_mapping_fingerprint_sha256': (
            current_public_topology['fingerprint_sha256']
        ),
        'canary_used_for_checkpoint_selection': False,
        'canary_loaded': True,
        'training_performed': False,
        'checkpoint_selection_performed': False,
        'calibration_refit_performed': False,
        'calibration_temperature_source': 'validation_only',
        'prior_exposure_acknowledged': True,
        'test_loaded': False,
        'automatic_runtime_switch': False,
        'acceptance_status': canary_acceptance_status,
        'promotion_status': (
            'eligible_for_manual_runtime_review'
            if canary_acceptance_status == 'passed'
            else 'failed_canary_acceptance'
            if canary_acceptance_status == 'failed'
            else 'pending_canary_evidence'
        ),
        'pending_evaluations': pending_canary_gates,
    }
    _write_json_exclusive(output / 'final_report.json', report)
    _complete_canary_attempt(reservation, output)
    return report


def smoke_v4(
    config_path: Path | str,
    output_path: Path | str,
    *,
    device_override: str | None = None,
) -> dict[str, Any]:
    """Run one real train step and a tiny validation pass without Canary."""

    output = _prepare_output(output_path)
    bundle = preflight_configuration(config_path, decode_images=False)
    config = copy.deepcopy(bundle.config)
    config['training']['num_workers'] = 0
    config['training']['persistent_workers'] = False
    torch_module, torchvision_module, training_api = require_training_stack()
    seed = int(config['training']['seed'])
    set_deterministic(torch_module, seed)
    device = _resolve_device(
        torch_module,
        str(config['training']['device']),
        device_override,
    )
    model = build_configured_model(config, torch_module, torchvision_module)
    initialization_report = initialize_verified_v3_backbone(
        model, config, torch_module
    )
    model.to(device)
    stage = configure_training_stage(
        model,
        config['training'],
        epoch=1,
        torch_module=torch_module,
    )
    optimizer = build_staged_optimizer(model, config['training'], torch_module)
    selected = [records[0] for records in bundle.train_by_source.values()]
    loader = make_loader(
        selected,
        config,
        torch_module=torch_module,
        training=True,
        epoch=1,
        batch_size_override=len(selected),
    )
    class_weights = class_weights_by_side(bundle.report['class_counts'], config['loss'])
    topology_distances = torch_module.as_tensor(
        topology_distance_matrix(), dtype=torch_module.float32, device=device
    )
    batch = next(iter(loader))
    image = batch['image'].to(device)
    targets = _target_device_dict(batch, device)
    amp_enabled = bool(
        config['training']['automatic_mixed_precision'] and device.type == 'cuda'
    )
    scaler = _make_grad_scaler(
        torch_module,
        amp_enabled,
        float(config['training']['amp_initial_scale']),
    )
    initial_amp_scale = float(scaler.get_scale())
    skipped_nonfinite_amp_steps = 0
    optimizer_step_completed = False
    losses: dict[str, Any] = {}
    for attempt in range(1, 9):
        optimizer.zero_grad(set_to_none=True)
        with _autocast_context(torch_module, device, amp_enabled):
            predictions = model(image)
            losses = compute_configured_loss(
                predictions,
                targets,
                class_weights=class_weights,
                topology_distances=topology_distances,
                loss_config=config['loss'],
                torch_module=torch_module,
                training_api=training_api,
            )
        if not bool(torch_module.isfinite(losses['loss']).all()):
            raise V4TrainerError('one-step smoke produced a non-finite loss')
        scaler.scale(losses['loss']).backward()
        scaler.unscale_(optimizer)
        trainable_parameters = [
            parameter
            for parameter in model.parameters()
            if parameter.requires_grad and parameter.grad is not None
        ]
        gradients_finite = all(
            bool(torch_module.isfinite(parameter.grad).all())
            for parameter in trainable_parameters
        )
        if gradients_finite:
            torch_module.nn.utils.clip_grad_norm_(
                trainable_parameters,
                max_norm=float(config['training']['gradient_clip_norm']),
                error_if_nonfinite=True,
            )
        elif amp_enabled:
            skipped_nonfinite_amp_steps += 1
        else:
            raise V4TrainerError('one-step smoke produced non-finite gradients')
        scaler.step(optimizer)
        scaler.update()
        if gradients_finite:
            optimizer_step_completed = True
            break
    if not optimizer_step_completed:
        raise V4TrainerError(
            'smoke could not complete a finite AMP optimizer step in eight attempts'
        )
    validation_records = bundle.validation[:max(1, len(selected))]
    validation = evaluate_model(
        model,
        validation_records,
        config,
        torch_module=torch_module,
        training_api=training_api,
        device=device,
        class_weights=class_weights,
        topology_lengths=bundle.report['topology_lengths_by_side'],
        epoch=1,
        require_full_side_segment_support=False,
    )
    output.mkdir()
    report = {
        'schema_version': RUN_SCHEMA_VERSION,
        'status': 'passed',
        'mode': 'one_step_smoke',
        'training_samples': [record.sample_id for record in selected],
        'validation_samples': [record.sample_id for record in validation_records],
        'train_loss': {
            **{
                name: float(losses[name].detach().cpu())
                for name in LOSS_SCALAR_KEYS
            },
            'visible_count': int(losses['visible_count']),
        },
        'validation': validation,
        'stage': stage,
        'initialization': initialization_report,
        'amp': {
            'enabled': amp_enabled,
            'initial_scale': initial_amp_scale,
            'final_scale': float(scaler.get_scale()),
            'attempts': attempt,
            'skipped_nonfinite_optimizer_steps': skipped_nonfinite_amp_steps,
            'finite_optimizer_step_completed': optimizer_step_completed,
        },
        'canary_loaded': False,
        'test_loaded': False,
        'quality_evaluated': False,
        'acceptance_status': 'not_evaluated_by_one_step_smoke',
    }
    _write_json_exclusive(output / 'effective_config.json', config)
    _write_json_exclusive(output / 'preflight.json', bundle.report)
    _write_json_exclusive(output / 'smoke_report.json', report)
    return report


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Train/evaluate the isolated Room 315 visual model V4.'
    )
    subparsers = parser.add_subparsers(dest='mode', required=True)

    preflight_parser = subparsers.add_parser('preflight')
    preflight_parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG_PATH)
    preflight_parser.add_argument('--output', type=Path)
    preflight_parser.add_argument('--decode-images', action='store_true')

    train_parser = subparsers.add_parser('train')
    train_parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG_PATH)
    train_parser.add_argument('--output', type=Path, required=True)
    train_parser.add_argument('--device')
    train_parser.add_argument('--decode-images-preflight', action='store_true')

    canary_parser = subparsers.add_parser('evaluate-canary')
    canary_parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG_PATH)
    canary_parser.add_argument('--checkpoint', type=Path, required=True)
    canary_parser.add_argument('--checkpoint-sha256', required=True)
    canary_parser.add_argument('--output', type=Path, required=True)
    canary_parser.add_argument(
        '--approved-v3-canary-baseline', type=Path, required=True
    )
    canary_parser.add_argument(
        '--approved-v3-canary-baseline-sha256', required=True
    )
    canary_parser.add_argument(
        '--canary-coverage-contract', type=Path, required=True
    )
    canary_parser.add_argument(
        '--canary-coverage-contract-sha256', required=True
    )
    canary_parser.add_argument('--device')
    canary_parser.add_argument('--decode-images', action='store_true')

    smoke_parser = subparsers.add_parser('smoke')
    smoke_parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG_PATH)
    smoke_parser.add_argument('--output', type=Path, required=True)
    smoke_parser.add_argument('--device')
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        if args.mode == 'preflight':
            bundle = preflight_configuration(
                args.config,
                decode_images=bool(args.decode_images),
            )
            report = bundle.report
            if args.output is not None:
                output = assert_not_test_path(
                    args.output, context='preflight output'
                ).resolve()
                _write_json_exclusive(output, report)
        elif args.mode == 'train':
            report = train_v4(
                args.config,
                args.output,
                device_override=args.device,
                decode_images_preflight=bool(args.decode_images_preflight),
            )
        elif args.mode == 'evaluate-canary':
            report = evaluate_canary_v4(
                args.config,
                args.checkpoint,
                args.checkpoint_sha256,
                args.output,
                approved_v3_canary_baseline_path=(
                    args.approved_v3_canary_baseline
                ),
                approved_v3_canary_baseline_sha256=(
                    args.approved_v3_canary_baseline_sha256
                ),
                canary_coverage_contract_path=args.canary_coverage_contract,
                canary_coverage_contract_sha256=(
                    args.canary_coverage_contract_sha256
                ),
                device_override=args.device,
                decode_images=bool(args.decode_images),
            )
        elif args.mode == 'smoke':
            report = smoke_v4(
                args.config,
                args.output,
                device_override=args.device,
            )
        else:
            raise AssertionError(f'unhandled mode: {args.mode}')
    except V4TrainerError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
