#!/usr/bin/env python3
"""One-shot, fail-closed evaluator for the sealed Room 315 visual V4 Test.

This program deliberately lives outside the V4 trainer.  It cannot train,
select a checkpoint, fit calibration, tune a threshold, or promote a runtime.
The only public operation is evaluation of one pre-registered, newly generated
and finalized Test dataset with an already frozen V4 candidate.

The ordering is part of the safety contract:

1. verify control artifacts and their caller-supplied hashes;
2. reserve a global immutable attempt key;
3. only then open Test rows, labels, or images;
4. evaluate with the validation-selected checkpoint, saved validation
   temperature, saved runtime thresholds, and frozen acceptance gates;
5. bind every output in an external immutable completion ledger.

An interrupted or failed reserved attempt is consumed.  Choosing another
output directory cannot create another attempt because the ledger location and
output name are derived from the immutable attempt key.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import math
import os
import platform
import re
import stat
import sys
from collections import Counter
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_state_train_v4 as trainer  # noqa: E402
from room_315_rail_defaults import default_rail_network_path  # noqa: E402
from room_315_visual_contract_v4 import (  # noqa: E402
    FIXED_IDENTITIES,
    SEGMENT_CLASSES,
    SIDES,
    derive_side,
)
from room_315_visual_model_v4 import (  # noqa: E402
    V4_MODEL_KIND,
    V4_SLOT_ORDER,
)
from room_315_visual_state_dataset import (  # noqa: E402
    normalize_visual_state_labels,
    validate_visual_model_input,
)
from room_315_visual_v3_common import position_bin  # noqa: E402


CONTRACT_SCHEMA_VERSION = 'room315.visual_v4.final_test_contract.v1'
EVALUATION_PROTOCOL_LOCK_SCHEMA_VERSION = (
    'room315.visual_v4.final_test_evaluation_protocol_lock.v1'
)
FINAL_TEST_CONFIG_SCHEMA_VERSION = 'room315.visual_v4.final_test_config.v1'
FINALIZATION_SCHEMA_VERSION = 'room315.visual_v4.final_test_finalization.v1'
PREREGISTRATION_SCHEMA_VERSION = 'room315.visual_v4.final_test_preregistration.v1'
PLAN_LOCK_SCHEMA_VERSION = 'room315.visual_v4.final_test_plan_lock.v1'
ATTEMPT_SCHEMA_VERSION = 'room315.visual_v4.final_test_attempt.v1'
REPORT_SCHEMA_VERSION = 'room315.visual_v4.final_test_report.v1'
CALIBRATION_REPORT_SCHEMA_VERSION = (
    'room315.visual_v4.final_test_fixed_calibration.v1'
)
DATASET_ROLE = 'sealed_final_test_only'
CHECKPOINT_SCHEMA_VERSION = 'room315.visual_training.v4.checkpoint.v1'
VALIDATION_CALIBRATION_SCHEMA_VERSION = (
    'room315.visual_segment_calibration.v4.v1'
)
RUNTIME_MANIFEST_SCHEMA_VERSION = 'room315.visual_runtime_promotion.v4.v1'
TOPOLOGY_SCHEMA_VERSION = 'room315.public_segment_length_contract.v1'
CAMERAS = ('left_rail_rgb', 'right_rail_rgb')
POSITION_BINS = ('p05', 'p15', 'p25', 'p40', 'p50', 'p60', 'p75', 'p85', 'p95')
SCENE_OCCLUSION_CLASSES = ('clear', 'partial_risk')
SCENE_PRESENCE_DENSITIES = ('sparse', 'medium', 'dense')
TARGET_ZONES = ('boundary', 'switch', 'slot', 'merge_conflict', 'buffer', 'ordinary')
IDENTITY_ZONES = (
    'adjacent_branch',
    'ahead_region',
    'behind_region',
    'boundary',
    'buffer',
    'intermediate_route',
    'merge_conflict',
    'ordinary',
    'relation_neutral',
    'slot',
    'switch',
)
EVALUATION_DEVICE = 'cuda'
RAW_SEGMENT_CSV_NAMES = (
    'A12E.csv', 'A12I.csv', 'A14.csv', 'A1E.csv', 'A1I.csv', 'A23.csv',
    'A2E.csv', 'A2I.csv', 'A34E.csv', 'A34I.csv', 'A3E.csv', 'A3I.csv',
    'A4E.csv', 'A4I.csv',
)

DEFAULT_GLOBAL_LEDGER_ROOT = Path(
    '/home/tiago/room315_visual_v4_final_test_attempt_ledger_v1'
)
DEFAULT_CONTRACT_OUTPUT_ROOT = Path(
    '/home/tiago/room315_visual_v4_final_test_outputs'
)
FRESH_DATASET_ROOT_PATTERN = re.compile(
    r'^room315_visual_v4_final_test_seed[0-9]{10,}(?:_[a-z0-9]+)*$'
)
SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')

# These roots contain an old, development, previously opened, or otherwise
# non-independent Test split.  A copy under one of them can never be the V4
# final Test.  The content hashes below also reject the historically evaluated
# V3 Test even if its two JSONL files are copied to a fresh-looking directory.
FORBIDDEN_EXPOSED_TEST_ROOTS = tuple(
    Path(value).resolve()
    for value in (
        '/home/tiago/room315_test_evaluation_approved_archive_seed31520260730',
        '/home/tiago/room315_kairos_visual_state_test_evaluation_guard_seed31520260730',
        '/home/tiago/room315_kairos_visual_state_training_v1_seed31520260730',
        '/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730',
        '/home/tiago/room315_visual_state_v4_blockers',
        '/home/tiago/room315_visual_state_visual_v2_320_clean_seed31520260726',
        '/home/tiago/kairos_room315_h200_pilot_seed31520260726',
        '/home/tiago/kairos_room315_h200_smoke_seed31520260726',
        '/home/tiago/room315_local_training',
        '/home/tiago/Downloads/kairos_room315_h200_pilot_results',
    )
)
FORBIDDEN_EXPOSED_TEST_FILE_SHA256 = frozenset({
    # The one historically authorized/evaluated V3 Test rows and labels.
    '2fcf78c0034fe290c39b2816e12076300decf5f7818538357fae072231b9b502',
    '1dc97b0836f40c53810306e9a09874967fa7e1067cd5de315cba0e00570277e3',
})
PINNED_OLD_REPLAY_IMAGE_AUDIT = Path(
    '/home/tiago/room315_arbitrary_subset_visual_2040_capture_v2_seed31520260729/'
    'captured_production_audit.json'
).resolve()
PINNED_OLD_REPLAY_IMAGE_AUDIT_SHA256 = (
    'f380a9cb5c5a49e6dfa4a858c99620f7886d32e8e0fa262b4dcec83ae8fb0028'
)


class FinalTestV4Error(RuntimeError):
    """Raised whenever the sealed final-Test contract cannot be proved."""


@dataclass(frozen=True)
class Artifact:
    path: Path
    sha256: str
    bytes: int

    def as_dict(self) -> dict[str, Any]:
        return {
            'path': str(self.path),
            'sha256': self.sha256,
            'bytes': self.bytes,
        }


@dataclass(frozen=True)
class ControlBundle:
    contract_path: Path
    contract_sha256: str
    contract: dict[str, Any]
    evaluation_protocol_lock: dict[str, Any]
    dataset_config: dict[str, Any]
    plan_lock: dict[str, Any]
    preregistration: dict[str, Any]
    effective_config: dict[str, Any]
    training_report: dict[str, Any]
    validation_acceptance: dict[str, Any]
    validation_calibration: dict[str, Any]
    topology_contract: dict[str, Any]
    runtime_manifest: dict[str, Any]
    artifacts: dict[str, Artifact]
    attempt_key: str
    output_path: Path


@dataclass(frozen=True)
class ReservedAttempt:
    bundle: ControlBundle
    ledger_root: Path
    reservation_path: Path
    reservation_sha256: str
    completion_path: Path


def canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    )


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _sha256_canonical(value: Any) -> str:
    return _sha256_bytes(canonical_json(value).encode('utf-8'))


def final_test_attempt_key(
    checkpoint_sha256: Any,
    dataset_fingerprint_sha256: Any,
) -> str:
    """Return the path/serialization-independent global one-shot identity."""

    return _sha256_canonical({
        'checkpoint_sha256': _validated_sha256(
            checkpoint_sha256, 'attempt checkpoint SHA-256'
        ),
        'dataset_fingerprint_sha256': _validated_sha256(
            dataset_fingerprint_sha256, 'attempt dataset fingerprint SHA-256'
        ),
    })


def _validated_sha256(value: Any, context: str) -> str:
    digest = str(value or '').strip().casefold()
    if not SHA256_PATTERN.fullmatch(digest):
        raise FinalTestV4Error(f'{context} must be an explicit SHA-256')
    return digest


def _sha256_file(path: Path | str) -> str:
    candidate = Path(path).expanduser().resolve()
    if not candidate.is_file():
        raise FinalTestV4Error(f'artifact is not a file: {candidate}')
    digest = hashlib.sha256()
    with candidate.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _fingerprint(path: Path | str, expected_sha256: Any, context: str) -> Artifact:
    candidate = Path(path).expanduser().resolve()
    expected = _validated_sha256(expected_sha256, f'{context} expected hash')
    actual = _sha256_file(candidate)
    if actual != expected:
        raise FinalTestV4Error(
            f'{context} SHA-256 mismatch: {actual} != {expected}'
        )
    return Artifact(candidate, actual, candidate.stat().st_size)


def _read_json_object(path: Path | str, context: str) -> dict[str, Any]:
    candidate = Path(path).expanduser().resolve()
    try:
        parsed = json.loads(candidate.read_text(encoding='utf-8'))
    except FileNotFoundError as exc:
        raise FinalTestV4Error(f'{context} does not exist: {candidate}') from exc
    except json.JSONDecodeError as exc:
        raise FinalTestV4Error(
            f'{context} is invalid JSON at line {exc.lineno}: {candidate}'
        ) from exc
    if not isinstance(parsed, dict):
        raise FinalTestV4Error(f'{context} must be a JSON object: {candidate}')
    return parsed


def _required_mapping(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise FinalTestV4Error(f'{context} must be a JSON object')
    return value


def _required_bool(value: Any, expected: bool, context: str) -> None:
    if value is not expected:
        raise FinalTestV4Error(f'{context} must be {str(expected).lower()}')


def _positive_int(value: Any, context: str, *, minimum: int = 1) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise FinalTestV4Error(f'{context} must be an integer >= {minimum}')
    return value


def _finite_float(value: Any, context: str) -> float:
    if isinstance(value, bool):
        raise FinalTestV4Error(f'{context} must be numeric')
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise FinalTestV4Error(f'{context} must be numeric') from exc
    if not math.isfinite(result):
        raise FinalTestV4Error(f'{context} must be finite')
    return result


def _parse_utc(value: Any, context: str) -> datetime:
    text = str(value or '').strip()
    if not text:
        raise FinalTestV4Error(f'{context} must be a UTC timestamp')
    try:
        parsed = datetime.fromisoformat(text.replace('Z', '+00:00'))
    except ValueError as exc:
        raise FinalTestV4Error(f'{context} is not ISO-8601: {text!r}') from exc
    if parsed.tzinfo is None or parsed.utcoffset() != timezone.utc.utcoffset(parsed):
        raise FinalTestV4Error(f'{context} must carry an explicit UTC offset')
    return parsed.astimezone(timezone.utc)


def _is_relative_to(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _assert_source_tree_execution() -> tuple[Path, Path]:
    """Return source roots or reject unsupported installed-layout execution."""

    evaluator_path = Path(__file__).resolve()
    package_root = evaluator_path.parent.parent
    repository_root = package_root.parent
    expected = package_root / 'scripts' / 'room_315_visual_final_test_v4.py'
    anchors = (
        package_root / 'CMakeLists.txt',
        package_root / 'package.xml',
        repository_root / 'mfja_3rd_floor_description' / 'worlds'
        / 'room_315_only.world',
    )
    if (
        evaluator_path != expected
        or package_root.name != 'mfja_robot_control_config'
        or not all(path.is_file() for path in anchors)
    ):
        raise FinalTestV4Error(
            'final-Test V4 evaluator supports source-tree execution only; '
            'installed/ros2-run layout is intentionally refused before reservation'
        )
    return package_root, repository_root


def assert_fresh_final_test_path(path: Path | str, *, context: str) -> Path:
    """Reject every known historical/exposed Test tree."""

    candidate = Path(path).expanduser().resolve()
    for root in FORBIDDEN_EXPOSED_TEST_ROOTS:
        if _is_relative_to(candidate, root):
            raise FinalTestV4Error(
                f'{context} points into a historical/exposed Test root: {candidate}'
            )
    return candidate


def _require_within(path: Path, root: Path, context: str) -> None:
    if not _is_relative_to(path, root):
        raise FinalTestV4Error(f'{context} escapes fresh dataset root: {path}')


def _artifact_spec(
    root: Mapping[str, Any], name: str, *, context: str
) -> tuple[Path, str]:
    raw = _required_mapping(root.get(name), f'{context}.{name}')
    path = Path(str(raw.get('path') or '')).expanduser().resolve()
    digest = _validated_sha256(raw.get('sha256'), f'{context}.{name}.sha256')
    return path, digest


def _verify_named_json_artifact(
    root: Mapping[str, Any],
    name: str,
    *,
    context: str,
) -> tuple[dict[str, Any], Artifact]:
    path, digest = _artifact_spec(root, name, context=context)
    artifact = _fingerprint(path, digest, f'{context}.{name}')
    return _read_json_object(path, f'{context}.{name}'), artifact


def _validate_fresh_dataset_declaration(
    contract: Mapping[str, Any],
) -> tuple[Path, dict[str, tuple[Path, str]]]:
    dataset = _required_mapping(contract.get('dataset'), 'contract.dataset')
    root = assert_fresh_final_test_path(
        dataset.get('root', ''), context='final Test dataset root'
    )
    if not FRESH_DATASET_ROOT_PATTERN.fullmatch(root.name):
        raise FinalTestV4Error(
            'fresh final Test root must use the dedicated '
            'room315_visual_v4_final_test_seed<seed> name'
        )
    image_root = assert_fresh_final_test_path(
        dataset.get('image_root', ''), context='final Test image root'
    )
    _require_within(image_root, root, 'final Test image root')
    if image_root != root / 'dataset':
        raise FinalTestV4Error('final Test image_root must be <dataset root>/dataset')
    expected_names = {
        'rows': 'final_test.jsonl',
        'labels': 'final_test_visual_labels.jsonl',
        'finalization': 'final_test_finalization.json',
    }
    declared: dict[str, tuple[Path, str]] = {}
    for name, basename in expected_names.items():
        path, digest = _artifact_spec(dataset, name, context='contract.dataset')
        path = assert_fresh_final_test_path(path, context=f'final Test {name}')
        _require_within(path, root, f'final Test {name}')
        if path.name != basename:
            raise FinalTestV4Error(
                f'final Test {name} must be named {basename!r}'
            )
        declared[name] = (path, digest)
    sample_count = _positive_int(
        dataset.get('sample_count'), 'contract.dataset.sample_count', minimum=512
    )
    image_count = _positive_int(
        dataset.get('image_count'), 'contract.dataset.image_count', minimum=1024
    )
    if image_count != sample_count * len(CAMERAS):
        raise FinalTestV4Error('final Test image_count must equal 2 * sample_count')
    image_manifest_sha256 = _validated_sha256(
        dataset.get('image_manifest_sha256'),
        'contract.dataset.image_manifest_sha256',
    )
    if declared['rows'][1] in FORBIDDEN_EXPOSED_TEST_FILE_SHA256:
        raise FinalTestV4Error('final Test rows reuse a historically exposed Test file')
    if declared['labels'][1] in FORBIDDEN_EXPOSED_TEST_FILE_SHA256:
        raise FinalTestV4Error('final Test labels reuse a historically exposed Test file')
    expected_fingerprint = _sha256_canonical({
        'rows_sha256': declared['rows'][1],
        'labels_sha256': declared['labels'][1],
        'image_manifest_sha256': image_manifest_sha256,
        'sample_count': sample_count,
        'image_count': image_count,
    })
    if _validated_sha256(
        dataset.get('dataset_fingerprint_sha256'),
        'contract.dataset.dataset_fingerprint_sha256',
    ) != expected_fingerprint:
        raise FinalTestV4Error('declared final Test dataset fingerprint is inconsistent')
    return root, declared


def _validate_dataset_config(
    value: Mapping[str, Any], artifact: Artifact, contract: Mapping[str, Any]
) -> None:
    if value.get('schema_version') != FINAL_TEST_CONFIG_SCHEMA_VERSION:
        raise FinalTestV4Error('final Test dataset config schema is incompatible')
    if value.get('dataset_role') != DATASET_ROLE:
        raise FinalTestV4Error(
            f'final Test dataset config role must be {DATASET_ROLE!r}'
        )
    _positive_int(value.get('scenario_count'), 'dataset config scenario_count', minimum=512)
    dataset = _required_mapping(contract.get('dataset'), 'contract.dataset')
    if int(value['scenario_count']) != int(dataset['sample_count']):
        raise FinalTestV4Error('dataset config scenario_count differs from contract')
    configured_root = assert_fresh_final_test_path(
        value.get('output_root', ''), context='dataset config output_root'
    )
    if configured_root != Path(str(dataset['root'])).expanduser().resolve():
        raise FinalTestV4Error('dataset config output_root differs from contract')
    frozen_model = _required_mapping(value.get('frozen_model'), 'dataset config frozen_model')
    candidate = _required_mapping(contract.get('frozen_candidate'), 'frozen_candidate')
    checkpoint_path, checkpoint_sha = _artifact_spec(
        candidate, 'checkpoint', context='frozen_candidate'
    )
    if Path(str(frozen_model.get('checkpoint') or '')).resolve() != checkpoint_path:
        raise FinalTestV4Error('dataset config checkpoint path differs from contract')
    if _validated_sha256(
        frozen_model.get('sha256'), 'dataset config checkpoint SHA-256'
    ) != checkpoint_sha:
        raise FinalTestV4Error('dataset config checkpoint SHA differs from contract')
    prohibitions = _required_mapping(value.get('prohibitions'), 'dataset config prohibitions')
    for name in (
        'historical_test_access',
        'training_use',
        'checkpoint_selection',
        'calibration_or_threshold_selection',
        'automatic_runtime_promotion',
        'more_than_one_final_inference',
    ):
        _required_bool(prohibitions.get(name), True, f'dataset config prohibitions.{name}')
    config_spec = _required_mapping(
        contract.get('dataset_config'), 'contract.dataset_config'
    )
    if _validated_sha256(
        config_spec.get('sha256'), 'contract.dataset_config.sha256'
    ) != artifact.sha256:
        raise FinalTestV4Error('dataset config artifact binding failed')


def _validate_protocol_artifacts(
    contract: Mapping[str, Any],
    dataset_config_artifact: Artifact,
    preregistration: Mapping[str, Any],
    preregistration_artifact: Artifact,
    plan_lock: Mapping[str, Any],
    plan_lock_artifact: Artifact,
    finalization_artifact: Artifact,
) -> None:
    if preregistration.get('schema_version') != PREREGISTRATION_SCHEMA_VERSION:
        raise FinalTestV4Error('final Test preregistration schema is incompatible')
    if plan_lock.get('schema_version') != PLAN_LOCK_SCHEMA_VERSION:
        raise FinalTestV4Error('final Test plan-lock schema is incompatible')
    candidate = _required_mapping(contract.get('frozen_candidate'), 'frozen_candidate')
    candidate_checkpoint_path, candidate_checkpoint_sha = _artifact_spec(
        candidate, 'checkpoint', context='frozen_candidate'
    )
    for name, value in (
        ('preregistration.dataset_role', preregistration.get('dataset_role')),
        ('plan_lock.dataset_role', plan_lock.get('dataset_role')),
    ):
        if value != DATASET_ROLE:
            raise FinalTestV4Error(f'{name} must be {DATASET_ROLE!r}')
    for name, value in (
        ('preregistration', preregistration),
        ('plan_lock', plan_lock),
    ):
        if (
            value.get('inference_status') != 'not_run'
            or value.get('inference_count') != 0
            or value.get('capture_started') is not False
        ):
            raise FinalTestV4Error(
                f'{name} does not prove zero inference before the sealed attempt'
            )
        frozen_model = _required_mapping(
            value.get('frozen_model'), f'{name}.frozen_model'
        )
        if (
            Path(str(frozen_model.get('checkpoint') or '')).expanduser().resolve()
            != candidate_checkpoint_path
            or _validated_sha256(
                frozen_model.get('sha256'), f'{name}.frozen_model.sha256'
            ) != candidate_checkpoint_sha
        ):
            raise FinalTestV4Error(
                f'{name} does not bind the exact frozen checkpoint'
            )
    _required_bool(
        preregistration.get('model_frozen_before_preregistration'),
        True,
        'preregistration.model_frozen_before_preregistration',
    )
    _required_bool(
        plan_lock.get('model_frozen_before_lock'),
        True,
        'plan_lock.model_frozen_before_lock',
    )
    lock_configuration = _required_mapping(
        plan_lock.get('configuration'), 'plan_lock.configuration'
    )
    lock_config_sha = _validated_sha256(
        lock_configuration.get('sha256'), 'plan_lock.configuration.sha256'
    )
    if lock_config_sha != dataset_config_artifact.sha256:
        raise FinalTestV4Error('plan lock does not bind the exact dataset config')
    lock_artifacts = _required_mapping(
        plan_lock.get('artifacts'), 'plan_lock.artifacts'
    )
    prereg_sha = _validated_sha256(
        lock_artifacts.get('preregistration.json'),
        'plan_lock.artifacts.preregistration.json',
    )
    if prereg_sha != preregistration_artifact.sha256:
        raise FinalTestV4Error('plan lock does not bind the exact preregistration')
    frozen = _required_mapping(contract.get('frozen_contract'), 'frozen_contract')
    if _validated_sha256(
        frozen.get('dataset_config_sha256'), 'frozen_contract.dataset_config_sha256'
    ) != dataset_config_artifact.sha256:
        raise FinalTestV4Error('contract does not bind the dataset config')
    if _validated_sha256(
        frozen.get('preregistration_sha256'),
        'frozen_contract.preregistration_sha256',
    ) != preregistration_artifact.sha256:
        raise FinalTestV4Error('contract does not bind the preregistration')
    if _validated_sha256(
        frozen.get('plan_lock_sha256'), 'frozen_contract.plan_lock_sha256'
    ) != plan_lock_artifact.sha256:
        raise FinalTestV4Error('contract does not bind the plan lock')
    if _validated_sha256(
        frozen.get('finalization_sha256'),
        'frozen_contract.finalization_sha256',
    ) != finalization_artifact.sha256:
        raise FinalTestV4Error('contract does not bind the dataset finalization')
    model_frozen = _parse_utc(
        _required_mapping(
            _required_mapping(
                _read_json_object(dataset_config_artifact.path, 'dataset config'),
                'dataset config',
            ).get('frozen_model'),
            'dataset config frozen_model',
        ).get('frozen_at_utc'),
        'dataset config frozen_at_utc',
    )
    preregistered = _parse_utc(
        preregistration.get('preregistered_at_utc'),
        'preregistration.preregistered_at_utc',
    )
    generation_started = _parse_utc(
        plan_lock.get('locked_at_utc'),
        'plan_lock.locked_at_utc',
    )
    if not model_frozen < preregistered <= generation_started:
        raise FinalTestV4Error(
            'final Test preregistration/generation did not occur after model freeze'
        )


def _validate_coverage_contract(value: Any) -> dict[str, Any]:
    coverage = dict(_required_mapping(value, 'contract.coverage_contract'))
    canonical_lists = {
        'required_identities': FIXED_IDENTITIES,
        'required_sides': SIDES,
        'required_segments': SEGMENT_CLASSES,
        'required_position_bins': POSITION_BINS,
        'required_scene_occlusion_classes': SCENE_OCCLUSION_CLASSES,
        'required_scene_presence_densities': SCENE_PRESENCE_DENSITIES,
        'required_target_zones': TARGET_ZONES,
        'required_identity_zones': IDENTITY_ZONES,
    }
    for name, expected in canonical_lists.items():
        if tuple(coverage.get(name) or ()) != tuple(expected):
            raise FinalTestV4Error(
                f'coverage_contract.{name} must equal the canonical order'
            )
    minimums = {
        'minimum_sample_count': 512,
        'minimum_visible_total': 512,
        'minimum_visible_per_identity': 32,
        'minimum_visible_per_side_x_segment': 8,
        'minimum_visible_per_position_bin': 16,
        'minimum_records_per_scene_occlusion_class': 32,
        'minimum_records_per_scene_presence_density': 32,
        'minimum_visible_per_target_zone': 8,
        'minimum_visible_per_identity_zone': 8,
    }
    for name, hard_floor in minimums.items():
        _positive_int(
            coverage.get(name),
            f'coverage_contract.{name}',
            minimum=hard_floor,
        )
    runtime = _required_mapping(
        coverage.get('runtime_threshold_gates'),
        'coverage_contract.runtime_threshold_gates',
    )
    floors = {
        'minimum_segment_confidence_coverage': 0.90,
        'minimum_segment_selective_accuracy': 0.95,
        'minimum_loaded_confidence_coverage': 0.95,
        'minimum_joint_confidence_coverage': 0.90,
    }
    for name, floor in floors.items():
        value = _finite_float(runtime.get(name), f'runtime_threshold_gates.{name}')
        if not floor <= value <= 1.0:
            raise FinalTestV4Error(
                f'runtime_threshold_gates.{name} must be in [{floor}, 1]'
            )
    return coverage


def _validate_historical_reference_declarations(
    contract: Mapping[str, Any],
    dataset_config: Mapping[str, Any],
) -> None:
    """Bind hash-only references to the already frozen dataset configuration."""

    sources = dataset_config.get('reference_sources')
    if not isinstance(sources, list):
        raise FinalTestV4Error('dataset config reference_sources must be a list')
    source_by_name = {
        str(item.get('name') or ''): item
        for item in sources
        if isinstance(item, Mapping)
    }
    expected: dict[str, tuple[Path, str, list[str]]] = {
        'old_replay_superset': (
            PINNED_OLD_REPLAY_IMAGE_AUDIT,
            PINNED_OLD_REPLAY_IMAGE_AUDIT_SHA256,
            ['source_image_hashes'],
        ),
    }
    for source_name, role in (
        ('v3r1_train', 'v3r1_train'),
        ('v3r1_validation', 'v3r1_validation'),
        ('v3r1_canary', 'v3r1_canary'),
    ):
        source = source_by_name.get(source_name)
        if not isinstance(source, Mapping):
            raise FinalTestV4Error(
                f'dataset config lacks reference source {source_name}'
            )
        expected[role] = (
            Path(str(source.get('image_hash_manifest') or '')).expanduser().resolve(),
            _validated_sha256(
                source.get('image_hash_manifest_sha256'),
                f'{source_name} image-hash manifest SHA-256',
            ),
            ['image_hashes'],
        )

    declarations = contract.get('historical_image_references')
    if not isinstance(declarations, list):
        raise FinalTestV4Error('historical_image_references must be a list')
    actual: dict[str, Mapping[str, Any]] = {}
    for raw in declarations:
        item = _required_mapping(raw, 'historical image reference declaration')
        role = str(item.get('role') or '').strip()
        if role in actual:
            raise FinalTestV4Error(
                f'duplicate historical image reference role: {role!r}'
            )
        actual[role] = item
    if set(actual) != set(expected):
        raise FinalTestV4Error(
            'historical image references differ from the frozen four-role set'
        )
    for role, (expected_path, expected_sha, expected_field) in expected.items():
        item = actual[role]
        if (
            Path(str(item.get('path') or '')).expanduser().resolve()
            != expected_path
            or _validated_sha256(
                item.get('sha256'), f'historical reference {role} SHA-256'
            ) != expected_sha
            or item.get('hash_field_path') != expected_field
        ):
            raise FinalTestV4Error(
                f'historical image reference {role} differs from its frozen pin'
            )


def _validate_candidate_control(
    contract: Mapping[str, Any],
    artifacts: dict[str, Artifact],
) -> tuple[
    dict[str, Any],
    dict[str, Any],
    dict[str, Any],
    dict[str, Any],
    dict[str, Any],
    dict[str, Any],
]:
    candidate = _required_mapping(contract.get('frozen_candidate'), 'frozen_candidate')
    loaded: dict[str, dict[str, Any]] = {}
    for name in (
        'effective_config',
        'training_final_report',
        'validation_acceptance',
        'validation_segment_calibration',
        'public_topology_contract',
        'runtime_promotion_manifest',
    ):
        value, artifact = _verify_named_json_artifact(
            candidate, name, context='frozen_candidate'
        )
        loaded[name] = value
        artifacts[name] = artifact
    checkpoint_path, checkpoint_sha = _artifact_spec(
        candidate, 'checkpoint', context='frozen_candidate'
    )
    checkpoint_path = assert_fresh_final_test_path(
        checkpoint_path, context='frozen V4 checkpoint'
    )
    artifacts['checkpoint'] = _fingerprint(
        checkpoint_path, checkpoint_sha, 'frozen V4 checkpoint'
    )

    config = loaded['effective_config']
    report = loaded['training_final_report']
    acceptance = loaded['validation_acceptance']
    calibration = loaded['validation_segment_calibration']
    topology = loaded['public_topology_contract']
    runtime = loaded['runtime_promotion_manifest']

    if config.get('schema_version') != trainer.CONFIG_SCHEMA_VERSION:
        raise FinalTestV4Error('effective V4 training config schema is incompatible')
    training = _required_mapping(config.get('training'), 'effective config training')
    if str(training.get('device') or '').strip().casefold() != EVALUATION_DEVICE:
        raise FinalTestV4Error('frozen V4 evaluation device must be CUDA')
    model = _required_mapping(config.get('model'), 'effective config model')
    if (
        model.get('kind') != V4_MODEL_KIND
        or tuple(model.get('slot_order') or ()) != tuple(V4_SLOT_ORDER)
        or model.get('cross_camera_feature_path') is not False
    ):
        raise FinalTestV4Error('effective V4 model contract is incompatible')
    preprocessing = _required_mapping(
        config.get('image_preprocessing'), 'effective config preprocessing'
    )
    if (
        preprocessing.get('width') != 320
        or preprocessing.get('height') != 240
        or preprocessing.get('resize') != 'aspect_preserving_bilinear_4_by_3'
    ):
        raise FinalTestV4Error('effective V4 preprocessing contract is incompatible')
    roles = _required_mapping(config.get('data_roles'), 'effective config data_roles')
    if roles.get('checkpoint_selection') != 'validation_only':
        raise FinalTestV4Error('V4 checkpoint selection was not validation-only')
    if 'test' in _required_mapping(config.get('data'), 'effective config data'):
        raise FinalTestV4Error('training configuration unexpectedly contains Test data')

    effective_canonical_sha = _sha256_canonical(config)
    selected = _required_mapping(report.get('selected_checkpoint'), 'selected checkpoint')
    if (
        report.get('status') != 'completed'
        or report.get('validation_acceptance_status') != 'passed'
        or report.get('canary_used_for_selection') is not False
        or report.get('test_loaded') is not False
        or selected.get('sha256') != checkpoint_sha
        or report.get('effective_config_sha256') != effective_canonical_sha
    ):
        raise FinalTestV4Error('training report does not prove a frozen isolated checkpoint')
    selection = _required_mapping(report.get('checkpoint_selection'), 'checkpoint selection')
    if selection.get('role') != 'validation_only':
        raise FinalTestV4Error('training report checkpoint selection is not validation-only')
    if (
        acceptance.get('status') != 'passed'
        or acceptance.get('accepted') is not True
        or acceptance.get('automatic_runtime_switch') is not False
    ):
        raise FinalTestV4Error('frozen validation acceptance did not pass safely')
    summary = _required_mapping(acceptance.get('summary'), 'validation acceptance summary')
    if int(summary.get('failed', -1)) != 0 or int(summary.get('pending', -1)) != 0:
        raise FinalTestV4Error('frozen validation acceptance has failed/pending gates')

    temperature = _finite_float(calibration.get('temperature'), 'validation temperature')
    if (
        temperature <= 0.0
        or calibration.get('schema_version') != VALIDATION_CALIBRATION_SCHEMA_VERSION
        or calibration.get('data_role') != 'validation'
        or calibration.get('fit_scope') != 'validation_only'
    ):
        raise FinalTestV4Error('validation calibration contract is incompatible')
    if (
        topology.get('schema_version') != TOPOLOGY_SCHEMA_VERSION
        or tuple(topology.get('segment_order') or ()) != tuple(SEGMENT_CLASSES)
        or tuple(topology.get('side_order') or ()) != tuple(SIDES)
    ):
        raise FinalTestV4Error('public topology contract is incompatible')

    if runtime.get('schema_version') != RUNTIME_MANIFEST_SCHEMA_VERSION:
        raise FinalTestV4Error('runtime promotion manifest schema is incompatible')
    if runtime.get('automatic_promotion_allowed') is not False:
        raise FinalTestV4Error('runtime manifest permits automatic promotion')
    manifest_model = _required_mapping(runtime.get('model_contract'), 'runtime model contract')
    manifest_topology = _required_mapping(
        runtime.get('topology_contract'), 'runtime topology contract'
    )
    manifest_calibration = _required_mapping(
        runtime.get('calibration_contract'), 'runtime calibration contract'
    )
    thresholds = _required_mapping(
        runtime.get('acceptance_thresholds'), 'runtime acceptance thresholds'
    )
    runtime_artifacts = _required_mapping(
        runtime.get('artifacts'), 'runtime manifest artifacts'
    )
    for manifest_name, bundle_name in (
        ('checkpoint', 'checkpoint'),
        ('effective_config', 'effective_config'),
        ('training_final_report', 'training_final_report'),
        ('validation_acceptance', 'validation_acceptance'),
        ('validation_segment_calibration', 'validation_segment_calibration'),
        ('public_topology_contract', 'public_topology_contract'),
    ):
        manifest_artifact = _required_mapping(
            runtime_artifacts.get(manifest_name),
            f'runtime manifest artifact {manifest_name}',
        )
        if _validated_sha256(
            manifest_artifact.get('sha256'),
            f'runtime manifest artifact {manifest_name} SHA-256',
        ) != artifacts[bundle_name].sha256:
            raise FinalTestV4Error(
                f'runtime manifest does not bind frozen artifact {manifest_name}'
            )
    if (
        manifest_model.get('model_kind') != V4_MODEL_KIND
        or tuple(manifest_model.get('slot_order') or ()) != tuple(V4_SLOT_ORDER)
        or tuple(manifest_model.get('segment_order') or ()) != tuple(SEGMENT_CLASSES)
        or manifest_model.get('cross_camera_feature_path') is not False
        or manifest_model.get('effective_config_fingerprint_sha256')
        != effective_canonical_sha
    ):
        raise FinalTestV4Error('runtime manifest model contract differs from V4 freeze')
    if manifest_topology.get('fingerprint_sha256') != topology.get('fingerprint_sha256'):
        raise FinalTestV4Error('runtime/public topology fingerprints differ')
    if (
        manifest_calibration.get('data_role') != 'validation'
        or manifest_calibration.get('fit_scope') != 'validation_only'
        or _finite_float(
            manifest_calibration.get('temperature'), 'runtime calibration temperature'
        ) != temperature
    ):
        raise FinalTestV4Error('runtime manifest does not reuse validation calibration')
    segment_threshold = _finite_float(
        thresholds.get('minimum_segment_confidence'),
        'runtime minimum_segment_confidence',
    )
    loaded_threshold = _finite_float(
        thresholds.get('minimum_loaded_confidence'),
        'runtime minimum_loaded_confidence',
    )
    if not 0.0 <= segment_threshold <= 1.0 or not 0.0 <= loaded_threshold <= 1.0:
        raise FinalTestV4Error('runtime confidence thresholds must be probabilities')
    segment_contract = _required_mapping(
        thresholds.get('segment_confidence'), 'runtime segment confidence contract'
    )
    if (
        segment_contract.get('calibrated') is not True
        or segment_contract.get('derivation')
        != 'validation_selective_curve_100_percent_coverage_floor'
        or _finite_float(segment_contract.get('temperature'), 'threshold temperature')
        != temperature
    ):
        raise FinalTestV4Error('runtime segment threshold is not validation-frozen')
    validation_full_coverage = [
        item for item in calibration.get('selective_curve', [])
        if isinstance(item, Mapping)
        and _finite_float(item.get('requested_coverage'), 'coverage target') == 1.0
    ]
    if (
        len(validation_full_coverage) != 1
        or _finite_float(
            validation_full_coverage[0].get('confidence_threshold'),
            'validation full-coverage threshold',
        ) != segment_threshold
    ):
        raise FinalTestV4Error('runtime segment threshold differs from validation curve')

    frozen = _required_mapping(contract.get('frozen_contract'), 'frozen_contract')
    exact = {
        'checkpoint_sha256': checkpoint_sha,
        'effective_config_canonical_sha256': effective_canonical_sha,
        'model_contract_sha256': _sha256_canonical(dict(manifest_model)),
        'topology_fingerprint_sha256': str(topology.get('fingerprint_sha256')),
        'validation_calibration_sha256': artifacts['validation_segment_calibration'].sha256,
        'runtime_manifest_sha256': artifacts['runtime_promotion_manifest'].sha256,
        'acceptance_gates_sha256': _sha256_canonical(config.get('pilot_acceptance_gates')),
    }
    for name, expected in exact.items():
        if _validated_sha256(frozen.get(name), f'frozen_contract.{name}') != expected:
            raise FinalTestV4Error(f'frozen_contract.{name} binding failed')
    if _finite_float(
        frozen.get('validation_temperature'), 'frozen validation temperature'
    ) != temperature:
        raise FinalTestV4Error('contract validation temperature differs from artifact')
    frozen_thresholds = _required_mapping(
        frozen.get('runtime_thresholds'), 'frozen_contract.runtime_thresholds'
    )
    if (
        _finite_float(
            frozen_thresholds.get('minimum_segment_confidence'),
            'frozen segment confidence threshold',
        ) != segment_threshold
        or _finite_float(
            frozen_thresholds.get('minimum_loaded_confidence'),
            'frozen loaded confidence threshold',
        ) != loaded_threshold
    ):
        raise FinalTestV4Error('contract runtime thresholds differ from manifest')
    return config, report, acceptance, calibration, topology, runtime


def load_control_bundle(
    contract_path: Path | str,
    expected_contract_sha256: str,
) -> ControlBundle:
    """Verify control files only; do not read Test rows, labels, or images."""

    _assert_source_tree_execution()
    path = Path(contract_path).expanduser().resolve()
    contract_artifact = _fingerprint(
        path, expected_contract_sha256, 'final Test evaluation contract'
    )
    contract = _read_json_object(path, 'final Test evaluation contract')
    if contract.get('schema_version') != CONTRACT_SCHEMA_VERSION:
        raise FinalTestV4Error('final Test evaluation contract schema is incompatible')
    if contract.get('dataset_role') != DATASET_ROLE:
        raise FinalTestV4Error(f'contract.dataset_role must be {DATASET_ROLE!r}')
    _required_bool(contract.get('prior_exposure'), False, 'contract.prior_exposure')
    _required_bool(
        contract.get('generated_after_model_freeze'),
        True,
        'contract.generated_after_model_freeze',
    )
    dataset_root, dataset_declaration = _validate_fresh_dataset_declaration(contract)
    del dataset_root  # The post-reservation loader resolves it again.

    lock_path, lock_sha = _artifact_spec(
        contract, 'evaluation_protocol_lock', context='contract'
    )
    evaluation_protocol_lock, protocol_lock_artifact = (
        load_evaluation_protocol_lock(lock_path, lock_sha)
    )
    artifacts: dict[str, Artifact] = {
        'contract': contract_artifact,
        'evaluation_protocol_lock': protocol_lock_artifact,
    }
    dataset_config, artifacts['dataset_config'] = _verify_named_json_artifact(
        contract, 'dataset_config', context='contract'
    )
    preregistration, artifacts['preregistration'] = _verify_named_json_artifact(
        contract, 'preregistration', context='contract'
    )
    plan_lock, artifacts['plan_lock'] = _verify_named_json_artifact(
        contract, 'plan_lock', context='contract'
    )
    # Hash the finalization control artifact, but intentionally do not parse it
    # until after the global attempt has been reserved.
    finalization_path, finalization_sha = dataset_declaration['finalization']
    artifacts['finalization'] = _fingerprint(
        finalization_path, finalization_sha, 'final Test finalization'
    )
    _validate_dataset_config(
        dataset_config, artifacts['dataset_config'], contract
    )
    _validate_historical_reference_declarations(contract, dataset_config)
    _validate_protocol_artifacts(
        contract,
        artifacts['dataset_config'],
        preregistration,
        artifacts['preregistration'],
        plan_lock,
        artifacts['plan_lock'],
        artifacts['finalization'],
    )
    _validate_coverage_contract(contract.get('coverage_contract'))
    (
        effective_config,
        training_report,
        validation_acceptance,
        validation_calibration,
        topology_contract,
        runtime_manifest,
    ) = _validate_candidate_control(contract, artifacts)

    policy = _required_mapping(contract.get('execution_policy'), 'execution_policy')
    expected_policy = _default_execution_policy()
    for name, expected in expected_policy.items():
        _required_bool(policy.get(name), expected, f'execution_policy.{name}')
    _validate_protocol_lock_contract_bindings(
        evaluation_protocol_lock, contract, artifacts
    )

    # Attempt identity intentionally excludes paths, output roots, timestamps,
    # serialization and policy prose.  Repackaging the same dataset/candidate
    # must collide with the same global ledger entry rather than manufacture a
    # retry.  The richer artifact hashes remain recorded as provenance.
    attempt_key = final_test_attempt_key(
        artifacts['checkpoint'].sha256,
        contract['dataset']['dataset_fingerprint_sha256'],
    )
    output_root = Path(
        str(contract.get('output_root') or DEFAULT_CONTRACT_OUTPUT_ROOT)
    ).expanduser().resolve()
    output_path = output_root / f'final_test_v4_{attempt_key[:16]}_attempt1'
    if output_path.exists():
        raise FinalTestV4Error(f'refusing existing immutable output: {output_path}')
    return ControlBundle(
        contract_path=path,
        contract_sha256=contract_artifact.sha256,
        contract=dict(contract),
        evaluation_protocol_lock=evaluation_protocol_lock,
        dataset_config=dataset_config,
        plan_lock=plan_lock,
        preregistration=preregistration,
        effective_config=effective_config,
        training_report=training_report,
        validation_acceptance=validation_acceptance,
        validation_calibration=validation_calibration,
        topology_contract=topology_contract,
        runtime_manifest=runtime_manifest,
        artifacts=artifacts,
        attempt_key=attempt_key,
        output_path=output_path,
    )


def _artifact_declaration(path: Path | str) -> dict[str, str]:
    candidate = Path(path).expanduser().resolve()
    return {
        'path': str(candidate),
        'sha256': _sha256_file(candidate),
    }


def _default_coverage_contract() -> dict[str, Any]:
    """Return the predeclared V4 final-Test support and runtime floors."""

    return {
        'required_identities': list(FIXED_IDENTITIES),
        'required_sides': list(SIDES),
        'required_segments': list(SEGMENT_CLASSES),
        'required_position_bins': list(POSITION_BINS),
        'required_scene_occlusion_classes': list(SCENE_OCCLUSION_CLASSES),
        'required_scene_presence_densities': list(SCENE_PRESENCE_DENSITIES),
        'required_target_zones': list(TARGET_ZONES),
        'required_identity_zones': list(IDENTITY_ZONES),
        'minimum_sample_count': 512,
        'minimum_visible_total': 512,
        'minimum_visible_per_identity': 32,
        'minimum_visible_per_side_x_segment': 8,
        'minimum_visible_per_position_bin': 16,
        'minimum_records_per_scene_occlusion_class': 32,
        'minimum_records_per_scene_presence_density': 32,
        'minimum_visible_per_target_zone': 8,
        'minimum_visible_per_identity_zone': 8,
        'runtime_threshold_gates': {
            'minimum_segment_confidence_coverage': 0.90,
            'minimum_segment_selective_accuracy': 0.95,
            'minimum_loaded_confidence_coverage': 0.95,
            'minimum_joint_confidence_coverage': 0.90,
        },
    }


def _default_execution_policy() -> dict[str, bool]:
    return {
        'training_performed': False,
        'checkpoint_selection_performed': False,
        'calibration_refit_performed': False,
        'threshold_selection_performed': False,
        'canary_opened': False,
        'historical_test_opened': False,
        'automatic_runtime_switch': False,
        'plansys_updates_enabled': False,
        'actuation_enabled': False,
        'one_shot': True,
    }


def _implementation_artifact_paths() -> dict[str, Path]:
    """Return every first-party implementation file frozen by the protocol."""

    return {
        'acceptance': SCRIPT_DIR / 'room_315_visual_acceptance_v4.py',
        'calibration': SCRIPT_DIR / 'room_315_visual_calibration_v4.py',
        'dataset_loader': SCRIPT_DIR / 'room_315_visual_state_dataset.py',
        'evaluator': Path(__file__).resolve(),
        'json_io': SCRIPT_DIR / 'room_315_json_io.py',
        'kinematic_shuttle': SCRIPT_DIR / 'room_315_kinematic_shuttle.py',
        'model': SCRIPT_DIR / 'room_315_visual_model_v4.py',
        'multi_shuttle': SCRIPT_DIR / 'room_315_multi_shuttle.py',
        'protocol_tests': (
            SCRIPT_DIR.parent / 'test' / 'test_room315_visual_final_test_v4.py'
        ),
        'rail_defaults': SCRIPT_DIR / 'room_315_rail_defaults.py',
        'topology_contract': SCRIPT_DIR / 'room_315_visual_contract_v4.py',
        'trainer_evaluation_api': SCRIPT_DIR / 'room_315_visual_state_train_v4.py',
        'training_runtime': SCRIPT_DIR / 'room_315_visual_training_v4.py',
        'v3_common_dependency': SCRIPT_DIR / 'room_315_visual_v3_common.py',
        'visual_fleet': SCRIPT_DIR / 'room_315_visual_fleet.py',
    }


def _referenced_raw_segment_csv_names(network_path: Path) -> tuple[str, ...]:
    names = tuple(sorted(set(re.findall(
        r'^\s*csv:\s*raw_segments/([^\s#]+\.csv)\s*$',
        network_path.read_text(encoding='utf-8'),
        flags=re.MULTILINE,
    ))))
    if names != tuple(sorted(RAW_SEGMENT_CSV_NAMES)):
        raise FinalTestV4Error(
            f'rail network raw-segment CSV set is not canonical: {network_path}'
        )
    return names


def _implementation_config_artifact_paths() -> dict[str, Path]:
    package_root = SCRIPT_DIR.parent
    repository_root = package_root.parent
    kinematics = package_root / 'config' / 'room_315_kinematics'
    left_network = kinematics / 'rail_network_left.yaml'
    right_network = kinematics / 'rail_network_right.yaml'
    for side, expected in (('left', left_network), ('right', right_network)):
        resolved_default = default_rail_network_path(side).resolve()
        if resolved_default != expected.resolve():
            raise FinalTestV4Error(
                f'{side} default rail network resolves outside the pinned '
                f'source tree (stale ament share?): {resolved_default}'
            )
    left_csv = _referenced_raw_segment_csv_names(left_network)
    right_csv = _referenced_raw_segment_csv_names(right_network)
    if left_csv != right_csv:
        raise FinalTestV4Error('left/right rail networks reference different CSV sets')
    result = {
        'rail_network_left': left_network,
        'rail_network_right': right_network,
        'shuttle_identity': (
            package_root
            / 'config'
            / 'room_315_shuttle_identity'
            / 'shuttle_identity.yaml'
        ),
        'simulation_world': (
            repository_root / 'mfja_3rd_floor_description' / 'worlds'
            / 'room_315_only.world'
        ),
    }
    for name in left_csv:
        result[f'raw_segment_{Path(name).stem}'] = (
            kinematics / 'raw_segments' / name
        )
    return result


def _environment_snapshot() -> dict[str, Any]:
    """Capture the execution environment that must remain exact at reservation."""

    try:
        import torch
    except Exception as exc:
        raise FinalTestV4Error(
            f'cannot fingerprint the evaluator PyTorch environment: {exc}'
        ) from exc
    cuda_available = bool(torch.cuda.is_available())
    cuda_devices = []
    if cuda_available:
        for index in range(torch.cuda.device_count()):
            properties = torch.cuda.get_device_properties(index)
            cuda_devices.append({
                'index': index,
                'name': str(properties.name),
                'compute_capability': [
                    int(properties.major), int(properties.minor)
                ],
                'total_memory_bytes': int(properties.total_memory),
            })
    packages = {}
    for name in ('numpy', 'pillow', 'pyyaml', 'torch', 'torchvision'):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = None
    return {
        'python_executable': str(Path(sys.executable).absolute()),
        'python_prefix': str(Path(sys.prefix).resolve()),
        'python_base_prefix': str(Path(sys.base_prefix).resolve()),
        'python_implementation': platform.python_implementation(),
        'python_version': platform.python_version(),
        'platform_system': platform.system(),
        'platform_release': platform.release(),
        'platform_machine': platform.machine(),
        'byteorder': sys.byteorder,
        'torch_version': str(torch.__version__),
        'torch_cuda_build': str(torch.version.cuda),
        'cudnn_version': (
            None if torch.backends.cudnn.version() is None
            else int(torch.backends.cudnn.version())
        ),
        'cuda_available': cuda_available,
        'cuda_device_count': len(cuda_devices),
        'cuda_devices': cuda_devices,
        'package_versions': packages,
    }


def _implementation_aggregate_sha256(
    declarations: Mapping[str, Any],
) -> str:
    normalized: dict[str, str] = {}
    for group_name, raw_group in declarations.items():
        group = _required_mapping(
            raw_group, f'implementation_artifacts.{group_name}'
        )
        for name, raw in group.items():
            item = _required_mapping(
                raw, f'implementation_artifacts.{group_name}.{name}'
            )
            normalized[f'{group_name}/{name}'] = _validated_sha256(
                item.get('sha256'),
                f'implementation_artifacts.{group_name}.{name}.sha256',
            )
    return _sha256_canonical(dict(sorted(normalized.items())))


def _protocol_frozen_sha256(value: Mapping[str, Any]) -> str:
    return _sha256_canonical({
        'execution_layout': value.get('execution_layout'),
        'source_tree': value.get('source_tree'),
        'dataset_root': value.get('dataset_root'),
        'design_artifacts': value.get('design_artifacts'),
        'candidate_artifacts': value.get('candidate_artifacts'),
        'historical_reference_artifacts': value.get(
            'historical_reference_artifacts'
        ),
        'coverage_contract_sha256': value.get('coverage_contract_sha256'),
        'execution_policy_sha256': value.get('execution_policy_sha256'),
        'one_shot': value.get('one_shot'),
        'implementation_aggregate_sha256': value.get(
            'implementation_aggregate_sha256'
        ),
        'environment_sha256': value.get('environment_sha256'),
    })


def build_evaluation_protocol_lock(
    *,
    dataset_config_path: Path | str,
    dataset_root: Path | str,
    candidate_root: Path | str,
    old_replay_image_audit_path: Path | str,
) -> dict[str, Any]:
    """Build the pre-inference lock without opening Test rows, labels or images."""

    package_root, repository_root = _assert_source_tree_execution()
    config_path = Path(dataset_config_path).expanduser().resolve()
    root = assert_fresh_final_test_path(
        dataset_root, context='protocol-lock dataset root'
    )
    candidate = Path(candidate_root).expanduser().resolve()
    old_replay = Path(old_replay_image_audit_path).expanduser().resolve()
    if old_replay != PINNED_OLD_REPLAY_IMAGE_AUDIT:
        raise FinalTestV4Error('old replay image audit differs from the frozen pin')
    if _sha256_file(old_replay) != PINNED_OLD_REPLAY_IMAGE_AUDIT_SHA256:
        raise FinalTestV4Error('pinned old replay image audit changed')

    dataset_config = _read_json_object(config_path, 'final Test dataset config')
    if dataset_config.get('dataset_role') != DATASET_ROLE:
        raise FinalTestV4Error('protocol-lock dataset role is incompatible')
    if Path(str(dataset_config.get('output_root') or '')).resolve() != root:
        raise FinalTestV4Error('protocol-lock root differs from dataset config')
    frozen_model = _required_mapping(
        dataset_config.get('frozen_model'), 'dataset config frozen_model'
    )
    checkpoint_path = Path(
        str(frozen_model.get('checkpoint') or '')
    ).expanduser().resolve()
    checkpoint_sha = _validated_sha256(
        frozen_model.get('sha256'), 'frozen checkpoint SHA-256'
    )
    if _sha256_file(checkpoint_path) != checkpoint_sha:
        raise FinalTestV4Error('frozen checkpoint changed before protocol lock')

    design_paths = {
        'dataset_config': config_path,
        'preregistration': root / 'preregistration.json',
        'plan_lock': root / 'plan_lock.json',
        'scenario_manifest': root / 'scenario_manifest.jsonl',
        'scenario_summary': root / 'scenario_summary.json',
    }
    candidate_paths = {
        'checkpoint': checkpoint_path,
        'effective_config': candidate / 'effective_config.json',
        'training_final_report': candidate / 'training_final_report.json',
        'validation_acceptance': candidate / 'validation_acceptance.json',
        'validation_segment_calibration': (
            candidate / 'validation_segment_calibration.json'
        ),
        'public_topology_contract': candidate / 'public_topology_contract.json',
        'runtime_promotion_manifest': candidate / 'runtime_promotion_manifest.json',
    }
    historical_paths = {'old_replay_superset': old_replay}
    for item in dataset_config.get('reference_sources', []):
        if not isinstance(item, Mapping):
            continue
        name = str(item.get('name') or '')
        if name in {'v3r1_train', 'v3r1_validation', 'v3r1_canary'}:
            path = Path(
                str(item.get('image_hash_manifest') or '')
            ).expanduser().resolve()
            declared = _validated_sha256(
                item.get('image_hash_manifest_sha256'),
                f'{name} image-hash manifest SHA-256',
            )
            if _sha256_file(path) != declared:
                raise FinalTestV4Error(f'historical reference changed: {name}')
            historical_paths[name] = path
    if set(historical_paths) != {
        'old_replay_superset', 'v3r1_train', 'v3r1_validation', 'v3r1_canary'
    }:
        raise FinalTestV4Error('protocol lock lacks the frozen historical references')

    implementation = {
        'code': {
            name: _artifact_declaration(path)
            for name, path in sorted(_implementation_artifact_paths().items())
        },
        'config': {
            name: _artifact_declaration(path)
            for name, path in sorted(
                _implementation_config_artifact_paths().items()
            )
        },
    }
    environment = _environment_snapshot()
    coverage = _default_coverage_contract()
    policy = _default_execution_policy()
    value: dict[str, Any] = {
        'schema_version': EVALUATION_PROTOCOL_LOCK_SCHEMA_VERSION,
        'dataset_role': DATASET_ROLE,
        'execution_layout': 'source_tree_only',
        'source_tree': {
            'package_root': str(package_root),
            'repository_root': str(repository_root),
            'anchors': {
                'cmake': _artifact_declaration(package_root / 'CMakeLists.txt'),
                'package_manifest': _artifact_declaration(
                    package_root / 'package.xml'
                ),
            },
        },
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'inference_status': 'not_run',
        'inference_count': 0,
        'test_rows_opened': False,
        'test_labels_opened': False,
        'test_images_opened': False,
        'dataset_root': str(root),
        'design_artifacts': {
            name: _artifact_declaration(path)
            for name, path in sorted(design_paths.items())
        },
        'candidate_artifacts': {
            name: _artifact_declaration(path)
            for name, path in sorted(candidate_paths.items())
        },
        'historical_reference_artifacts': {
            name: _artifact_declaration(path)
            for name, path in sorted(historical_paths.items())
        },
        'coverage_contract': coverage,
        'coverage_contract_sha256': _sha256_canonical(coverage),
        'execution_policy': policy,
        'execution_policy_sha256': _sha256_canonical(policy),
        'one_shot': {
            'enabled': True,
            'evaluation_device': EVALUATION_DEVICE,
            'ledger_override_allowed': False,
            'global_ledger_root': str(DEFAULT_GLOBAL_LEDGER_ROOT.resolve()),
            'attempt_identity': 'checkpoint_sha256+dataset_fingerprint_sha256',
            'failed_or_interrupted_attempt_is_consumed': True,
        },
        'implementation_artifacts': implementation,
        'implementation_aggregate_sha256': (
            _implementation_aggregate_sha256(implementation)
        ),
        'environment': environment,
        'environment_sha256': _sha256_canonical(environment),
    }
    value['protocol_frozen_sha256'] = _protocol_frozen_sha256(value)
    return value


def load_evaluation_protocol_lock(
    lock_path: Path | str,
    expected_lock_sha256: str,
) -> tuple[dict[str, Any], Artifact]:
    """Verify an immutable protocol lock and the current code/environment."""

    package_root, repository_root = _assert_source_tree_execution()
    artifact = _fingerprint(
        lock_path, expected_lock_sha256, 'evaluation protocol lock'
    )
    if stat.S_IMODE(artifact.path.stat().st_mode) != 0o444:
        raise FinalTestV4Error('evaluation protocol lock must have mode 0444')
    value = _read_json_object(artifact.path, 'evaluation protocol lock')
    if value.get('schema_version') != EVALUATION_PROTOCOL_LOCK_SCHEMA_VERSION:
        raise FinalTestV4Error('evaluation protocol lock schema is incompatible')
    if value.get('dataset_role') != DATASET_ROLE:
        raise FinalTestV4Error('evaluation protocol lock dataset role is incompatible')
    if value.get('execution_layout') != 'source_tree_only':
        raise FinalTestV4Error('evaluation protocol lock must be source-tree-only')
    source_tree = _required_mapping(
        value.get('source_tree'), 'protocol lock source_tree'
    )
    if (
        Path(str(source_tree.get('package_root') or '')).resolve() != package_root
        or Path(str(source_tree.get('repository_root') or '')).resolve()
        != repository_root
    ):
        raise FinalTestV4Error('protocol lock source-tree root changed')
    anchors = _required_mapping(
        source_tree.get('anchors'), 'protocol lock source_tree.anchors'
    )
    expected_anchors = {
        'cmake': package_root / 'CMakeLists.txt',
        'package_manifest': package_root / 'package.xml',
    }
    if set(anchors) != set(expected_anchors):
        raise FinalTestV4Error('protocol lock source-tree anchor set changed')
    for name, expected_path in expected_anchors.items():
        path, digest = _artifact_spec(
            anchors, name, context='protocol lock source_tree.anchors'
        )
        if path != expected_path:
            raise FinalTestV4Error(
                f'protocol lock source-tree anchor path changed: {name}'
            )
        _fingerprint(path, digest, f'protocol lock source-tree anchor {name}')
    _parse_utc(value.get('created_at_utc'), 'protocol lock created_at_utc')
    if value.get('inference_status') != 'not_run' or value.get('inference_count') != 0:
        raise FinalTestV4Error('evaluation protocol lock is not pre-inference')
    for name in ('test_rows_opened', 'test_labels_opened', 'test_images_opened'):
        _required_bool(value.get(name), False, f'protocol lock {name}')

    for group_name in (
        'design_artifacts', 'candidate_artifacts',
        'historical_reference_artifacts',
    ):
        group = _required_mapping(value.get(group_name), f'protocol lock {group_name}')
        for name, raw in group.items():
            item = _required_mapping(raw, f'protocol lock {group_name}.{name}')
            _fingerprint(
                item.get('path', ''), item.get('sha256'),
                f'protocol lock {group_name}.{name}',
            )

    expected_implementation_groups = {
        'code': {
            name: path.resolve()
            for name, path in _implementation_artifact_paths().items()
        },
        'config': {
            name: path.resolve()
            for name, path in _implementation_config_artifact_paths().items()
        },
    }
    declared_implementation = _required_mapping(
        value.get('implementation_artifacts'),
        'protocol lock implementation_artifacts',
    )
    if set(declared_implementation) != set(expected_implementation_groups):
        raise FinalTestV4Error('protocol lock implementation groups changed')
    for group_name, expected_paths in expected_implementation_groups.items():
        declared_group = _required_mapping(
            declared_implementation.get(group_name),
            f'protocol lock implementation_artifacts.{group_name}',
        )
        if set(declared_group) != set(expected_paths):
            raise FinalTestV4Error(
                f'protocol lock implementation {group_name} set changed'
            )
        for name, expected_path in expected_paths.items():
            path, digest = _artifact_spec(
                declared_group, name,
                context=f'protocol lock implementation_artifacts.{group_name}',
            )
            if path != expected_path:
                raise FinalTestV4Error(
                    f'protocol lock implementation path changed: '
                    f'{group_name}/{name}'
                )
            _fingerprint(
                path, digest,
                f'protocol lock implementation {group_name}/{name}',
            )
    aggregate = _implementation_aggregate_sha256(declared_implementation)
    if _validated_sha256(
        value.get('implementation_aggregate_sha256'),
        'protocol lock implementation aggregate SHA-256',
    ) != aggregate:
        raise FinalTestV4Error('protocol lock implementation aggregate changed')
    environment = _environment_snapshot()
    if value.get('environment') != environment:
        raise FinalTestV4Error('evaluation environment differs from protocol lock')
    if _validated_sha256(
        value.get('environment_sha256'), 'protocol lock environment SHA-256'
    ) != _sha256_canonical(environment):
        raise FinalTestV4Error('protocol lock environment hash is inconsistent')
    coverage = _required_mapping(
        value.get('coverage_contract'), 'protocol lock coverage_contract'
    )
    _validate_coverage_contract(coverage)
    if _validated_sha256(
        value.get('coverage_contract_sha256'),
        'protocol lock coverage contract SHA-256',
    ) != _sha256_canonical(coverage):
        raise FinalTestV4Error('protocol lock coverage hash is inconsistent')
    policy = _required_mapping(
        value.get('execution_policy'), 'protocol lock execution_policy'
    )
    if dict(policy) != _default_execution_policy():
        raise FinalTestV4Error('protocol lock execution policy changed')
    if _validated_sha256(
        value.get('execution_policy_sha256'),
        'protocol lock execution policy SHA-256',
    ) != _sha256_canonical(policy):
        raise FinalTestV4Error('protocol lock execution policy hash is inconsistent')
    one_shot = _required_mapping(value.get('one_shot'), 'protocol lock one_shot')
    if (
        one_shot.get('enabled') is not True
        or one_shot.get('evaluation_device') != EVALUATION_DEVICE
        or one_shot.get('ledger_override_allowed') is not False
        or Path(str(one_shot.get('global_ledger_root') or '')).resolve()
        != DEFAULT_GLOBAL_LEDGER_ROOT.resolve()
        or one_shot.get('attempt_identity')
        != 'checkpoint_sha256+dataset_fingerprint_sha256'
        or one_shot.get('failed_or_interrupted_attempt_is_consumed') is not True
    ):
        raise FinalTestV4Error('protocol lock one-shot policy changed')
    if (
        environment.get('cuda_available') is not True
        or int(environment.get('cuda_device_count', 0)) < 1
    ):
        raise FinalTestV4Error('protocol lock requires an available CUDA device')
    if _validated_sha256(
        value.get('protocol_frozen_sha256'),
        'protocol lock frozen SHA-256',
    ) != _protocol_frozen_sha256(value):
        raise FinalTestV4Error('protocol lock frozen hash is inconsistent')
    return value, artifact


def _validate_protocol_lock_contract_bindings(
    lock: Mapping[str, Any],
    contract: Mapping[str, Any],
    artifacts: Mapping[str, Artifact],
) -> None:
    """Cross-bind the independent lock to every frozen contract component."""

    root = Path(str(contract['dataset']['root'])).expanduser().resolve()
    if Path(str(lock.get('dataset_root') or '')).resolve() != root:
        raise FinalTestV4Error('protocol lock dataset root differs from contract')
    design = _required_mapping(
        lock.get('design_artifacts'), 'protocol lock design_artifacts'
    )
    for lock_name, artifact_name in (
        ('dataset_config', 'dataset_config'),
        ('preregistration', 'preregistration'),
        ('plan_lock', 'plan_lock'),
    ):
        path, digest = _artifact_spec(
            design, lock_name, context='protocol lock design_artifacts'
        )
        artifact = artifacts[artifact_name]
        if path != artifact.path or digest != artifact.sha256:
            raise FinalTestV4Error(
                f'protocol lock does not bind contract {artifact_name}'
            )
    for name, expected_path in (
        ('scenario_manifest', root / 'scenario_manifest.jsonl'),
        ('scenario_summary', root / 'scenario_summary.json'),
    ):
        path, _ = _artifact_spec(
            design, name, context='protocol lock design_artifacts'
        )
        if path != expected_path:
            raise FinalTestV4Error(f'protocol lock {name} path is inconsistent')

    candidate = _required_mapping(
        lock.get('candidate_artifacts'), 'protocol lock candidate_artifacts'
    )
    for name in (
        'checkpoint', 'effective_config', 'training_final_report',
        'validation_acceptance', 'validation_segment_calibration',
        'public_topology_contract', 'runtime_promotion_manifest',
    ):
        path, digest = _artifact_spec(
            candidate, name, context='protocol lock candidate_artifacts'
        )
        artifact = artifacts[name]
        if path != artifact.path or digest != artifact.sha256:
            raise FinalTestV4Error(
                f'protocol lock does not bind frozen candidate {name}'
            )
    coverage = _required_mapping(
        contract.get('coverage_contract'), 'contract.coverage_contract'
    )
    policy = _required_mapping(contract.get('execution_policy'), 'execution_policy')
    if lock.get('coverage_contract') != coverage:
        raise FinalTestV4Error('protocol lock coverage differs from contract')
    if lock.get('execution_policy') != policy:
        raise FinalTestV4Error('protocol lock execution policy differs from contract')
    frozen = _required_mapping(contract.get('frozen_contract'), 'frozen_contract')
    exact = {
        'evaluation_protocol_lock_sha256': artifacts[
            'evaluation_protocol_lock'
        ].sha256,
        'protocol_frozen_sha256': _validated_sha256(
            lock.get('protocol_frozen_sha256'), 'protocol lock frozen SHA-256'
        ),
        'implementation_aggregate_sha256': _validated_sha256(
            lock.get('implementation_aggregate_sha256'),
            'protocol lock implementation aggregate SHA-256',
        ),
    }
    for name, expected in exact.items():
        if _validated_sha256(frozen.get(name), f'frozen_contract.{name}') != expected:
            raise FinalTestV4Error(f'frozen_contract.{name} binding failed')

    historical = _required_mapping(
        lock.get('historical_reference_artifacts'),
        'protocol lock historical_reference_artifacts',
    )
    contract_historical = {
        str(item.get('role') or ''): item
        for item in contract.get('historical_image_references', [])
        if isinstance(item, Mapping)
    }
    role_to_lock = {
        'old_replay_superset': 'old_replay_superset',
        'v3r1_train': 'v3r1_train',
        'v3r1_validation': 'v3r1_validation',
        'v3r1_canary': 'v3r1_canary',
    }
    for role, lock_name in role_to_lock.items():
        contract_item = _required_mapping(
            contract_historical.get(role), f'historical reference {role}'
        )
        lock_path, lock_digest = _artifact_spec(
            historical, lock_name,
            context='protocol lock historical_reference_artifacts',
        )
        if (
            lock_path != Path(str(contract_item.get('path') or '')).resolve()
            or lock_digest != _validated_sha256(
                contract_item.get('sha256'), f'historical reference {role} SHA-256'
            )
        ):
            raise FinalTestV4Error(
                f'protocol lock does not bind historical reference {role}'
            )


def materialize_evaluation_protocol_lock(
    destination: Path | str,
    *,
    dataset_config_path: Path | str,
    dataset_root: Path | str,
    candidate_root: Path | str,
    old_replay_image_audit_path: Path | str,
) -> Artifact:
    """Write an immutable 0444 pre-inference protocol lock."""

    path = Path(destination).expanduser().resolve()
    value = build_evaluation_protocol_lock(
        dataset_config_path=dataset_config_path,
        dataset_root=dataset_root,
        candidate_root=candidate_root,
        old_replay_image_audit_path=old_replay_image_audit_path,
    )
    _write_json_exclusive(path, value, read_only=True)
    artifact = Artifact(path, _sha256_file(path), path.stat().st_size)
    load_evaluation_protocol_lock(path, artifact.sha256)
    return artifact


def build_final_test_contract(
    *,
    dataset_config_path: Path | str,
    dataset_root: Path | str,
    candidate_root: Path | str,
    old_replay_image_audit_path: Path | str,
    evaluation_protocol_lock_path: Path | str,
    evaluation_protocol_lock_sha256: str,
    output_root: Path | str = DEFAULT_CONTRACT_OUTPUT_ROOT,
) -> dict[str, Any]:
    """Build (but do not write or reserve) the sealed evaluation contract.

    This helper reads only control/fingerprint JSON.  In particular, it never
    opens final-Test rows, labels, images, Canary rows, or historical Test rows.
    The returned object still has to pass :func:`load_control_bundle` after it
    is serialized and caller-pinned by its own SHA-256.
    """

    config_path = Path(dataset_config_path).expanduser().resolve()
    root = assert_fresh_final_test_path(
        dataset_root, context='contract materialization dataset root'
    )
    candidate = Path(candidate_root).expanduser().resolve()
    old_replay_audit = Path(old_replay_image_audit_path).expanduser().resolve()
    if old_replay_audit != PINNED_OLD_REPLAY_IMAGE_AUDIT:
        raise FinalTestV4Error('old replay image audit differs from the frozen pin')
    protocol_lock, protocol_lock_artifact = load_evaluation_protocol_lock(
        evaluation_protocol_lock_path, evaluation_protocol_lock_sha256
    )
    finalization_path = root / 'finalized' / 'final_test_finalization.json'
    preregistration_path = root / 'preregistration.json'
    plan_lock_path = root / 'plan_lock.json'

    dataset_config = _read_json_object(config_path, 'final Test dataset config')
    finalization = _read_json_object(finalization_path, 'final Test finalization')
    runtime_path = candidate / 'runtime_promotion_manifest.json'
    runtime = _read_json_object(runtime_path, 'frozen runtime promotion manifest')
    effective_config_path = candidate / 'effective_config.json'
    effective_config = _read_json_object(
        effective_config_path, 'frozen effective V4 config'
    )
    calibration_path = candidate / 'validation_segment_calibration.json'
    calibration = _read_json_object(
        calibration_path, 'frozen validation segment calibration'
    )
    topology_path = candidate / 'public_topology_contract.json'
    topology = _read_json_object(topology_path, 'frozen public topology contract')

    if dataset_config.get('dataset_role') != DATASET_ROLE:
        raise FinalTestV4Error('contract materialization dataset role is incompatible')
    if Path(str(dataset_config.get('output_root') or '')).resolve() != root:
        raise FinalTestV4Error('contract materialization root differs from dataset config')
    if (
        finalization.get('schema_version') != FINALIZATION_SCHEMA_VERSION
        or finalization.get('dataset_role') != DATASET_ROLE
        or finalization.get('passed') is not True
        or finalization.get('inference_count') != 0
        or finalization.get('inference_status') != 'not_run'
    ):
        raise FinalTestV4Error('dataset is not a sealed, never-evaluated final Test')

    images = _required_mapping(finalization.get('images'), 'finalization.images')
    individual_hashes = dict(_required_mapping(
        images.get('individual_sha256_by_episode_camera'),
        'finalization individual image hashes',
    ))
    pair_hashes = dict(_required_mapping(
        images.get('pair_sha256_by_sample_id'),
        'finalization pair image hashes',
    ))
    image_manifest_sha256 = _sha256_canonical({
        'individual_sha256_by_episode_camera': dict(sorted(individual_hashes.items())),
        'pair_sha256_by_sample_id': dict(sorted(pair_hashes.items())),
    })
    rows = _required_mapping(finalization.get('rows'), 'finalization.rows')
    labels = _required_mapping(finalization.get('labels'), 'finalization.labels')
    sample_count = _positive_int(
        finalization.get('scenario_count'), 'finalization.scenario_count', minimum=512
    )
    image_count = _positive_int(
        finalization.get('image_count'), 'finalization.image_count', minimum=1024
    )
    rows_sha = _validated_sha256(rows.get('sha256'), 'finalization.rows.sha256')
    labels_sha = _validated_sha256(
        labels.get('sha256'), 'finalization.labels.sha256'
    )
    dataset_fingerprint_sha256 = _sha256_canonical({
        'rows_sha256': rows_sha,
        'labels_sha256': labels_sha,
        'image_manifest_sha256': image_manifest_sha256,
        'sample_count': sample_count,
        'image_count': image_count,
    })

    frozen_model = _required_mapping(
        dataset_config.get('frozen_model'), 'dataset config frozen_model'
    )
    runtime_model = _required_mapping(
        runtime.get('model_contract'), 'runtime model contract'
    )
    runtime_thresholds = _required_mapping(
        runtime.get('acceptance_thresholds'), 'runtime acceptance thresholds'
    )
    reference_by_name = {
        str(item.get('name') or ''): _required_mapping(
            item, 'dataset config reference source'
        )
        for item in dataset_config.get('reference_sources', [])
        if isinstance(item, Mapping)
    }
    historical_references = [{
        'role': 'old_replay_superset',
        **_artifact_declaration(old_replay_audit),
        'hash_field_path': ['source_image_hashes'],
    }]
    for source_name, role in (
        ('v3r1_train', 'v3r1_train'),
        ('v3r1_validation', 'v3r1_validation'),
        ('v3r1_canary', 'v3r1_canary'),
    ):
        source = reference_by_name.get(source_name)
        if source is None:
            raise FinalTestV4Error(
                f'dataset config lacks historical reference {source_name}'
            )
        manifest_path = Path(
            str(source.get('image_hash_manifest') or '')
        ).expanduser().resolve()
        declared_sha = _validated_sha256(
            source.get('image_hash_manifest_sha256'),
            f'{source_name} image hash manifest SHA-256',
        )
        if _sha256_file(manifest_path) != declared_sha:
            raise FinalTestV4Error(
                f'historical image manifest changed for {source_name}'
            )
        historical_references.append({
            'role': role,
            'path': str(manifest_path),
            'sha256': declared_sha,
            'hash_field_path': ['image_hashes'],
        })

    frozen_candidate = {
        'checkpoint': {
            'path': str(Path(str(frozen_model.get('checkpoint') or '')).resolve()),
            'sha256': _validated_sha256(
                frozen_model.get('sha256'), 'frozen checkpoint SHA-256'
            ),
        },
        'effective_config': _artifact_declaration(effective_config_path),
        'training_final_report': _artifact_declaration(
            candidate / 'training_final_report.json'
        ),
        'validation_acceptance': _artifact_declaration(
            candidate / 'validation_acceptance.json'
        ),
        'validation_segment_calibration': _artifact_declaration(calibration_path),
        'public_topology_contract': _artifact_declaration(topology_path),
        'runtime_promotion_manifest': _artifact_declaration(runtime_path),
    }
    contract = {
        'schema_version': CONTRACT_SCHEMA_VERSION,
        'dataset_role': DATASET_ROLE,
        'prior_exposure': False,
        'generated_after_model_freeze': True,
        'dataset': {
            'root': str(root),
            'image_root': str(root / 'dataset'),
            'sample_count': sample_count,
            'image_count': image_count,
            'rows': {
                'path': str(Path(str(rows.get('path') or '')).resolve()),
                'sha256': rows_sha,
            },
            'labels': {
                'path': str(Path(str(labels.get('path') or '')).resolve()),
                'sha256': labels_sha,
            },
            'finalization': _artifact_declaration(finalization_path),
            'image_manifest_sha256': image_manifest_sha256,
            'dataset_fingerprint_sha256': dataset_fingerprint_sha256,
        },
        'dataset_config': _artifact_declaration(config_path),
        'preregistration': _artifact_declaration(preregistration_path),
        'plan_lock': _artifact_declaration(plan_lock_path),
        'evaluation_protocol_lock': protocol_lock_artifact.as_dict(),
        'frozen_candidate': frozen_candidate,
        'frozen_contract': {
            'dataset_config_sha256': _sha256_file(config_path),
            'preregistration_sha256': _sha256_file(preregistration_path),
            'plan_lock_sha256': _sha256_file(plan_lock_path),
            'finalization_sha256': _sha256_file(finalization_path),
            'evaluation_protocol_lock_sha256': protocol_lock_artifact.sha256,
            'protocol_frozen_sha256': _validated_sha256(
                protocol_lock.get('protocol_frozen_sha256'),
                'evaluation protocol frozen SHA-256',
            ),
            'implementation_aggregate_sha256': _validated_sha256(
                protocol_lock.get('implementation_aggregate_sha256'),
                'implementation aggregate SHA-256',
            ),
            'checkpoint_sha256': frozen_candidate['checkpoint']['sha256'],
            'effective_config_canonical_sha256': _sha256_canonical(effective_config),
            'model_contract_sha256': _sha256_canonical(dict(runtime_model)),
            'topology_fingerprint_sha256': _validated_sha256(
                topology.get('fingerprint_sha256'), 'topology fingerprint'
            ),
            'validation_calibration_sha256': _sha256_file(calibration_path),
            'runtime_manifest_sha256': _sha256_file(runtime_path),
            'acceptance_gates_sha256': _sha256_canonical(
                effective_config.get('pilot_acceptance_gates')
            ),
            'validation_temperature': _finite_float(
                calibration.get('temperature'), 'validation temperature'
            ),
            'runtime_thresholds': {
                'minimum_segment_confidence': _finite_float(
                    runtime_thresholds.get('minimum_segment_confidence'),
                    'runtime minimum segment confidence',
                ),
                'minimum_loaded_confidence': _finite_float(
                    runtime_thresholds.get('minimum_loaded_confidence'),
                    'runtime minimum loaded confidence',
                ),
            },
        },
        'coverage_contract': _default_coverage_contract(),
        'historical_image_references': historical_references,
        'execution_policy': _default_execution_policy(),
        'output_root': str(Path(output_root).expanduser().resolve()),
    }
    if Path(str(protocol_lock.get('dataset_root') or '')).resolve() != root:
        raise FinalTestV4Error('protocol lock dataset root differs from contract')
    if protocol_lock.get('coverage_contract') != contract['coverage_contract']:
        raise FinalTestV4Error('protocol lock coverage differs from contract')
    if protocol_lock.get('execution_policy') != contract['execution_policy']:
        raise FinalTestV4Error('protocol lock execution policy differs from contract')
    _validate_coverage_contract(contract['coverage_contract'])
    return contract


def materialize_final_test_contract(
    destination: Path | str,
    *,
    dataset_config_path: Path | str,
    dataset_root: Path | str,
    candidate_root: Path | str,
    old_replay_image_audit_path: Path | str,
    evaluation_protocol_lock_path: Path | str,
    evaluation_protocol_lock_sha256: str,
    output_root: Path | str = DEFAULT_CONTRACT_OUTPUT_ROOT,
) -> Artifact:
    """Write and control-validate one immutable contract without Test access."""

    path = Path(destination).expanduser().resolve()
    contract = build_final_test_contract(
        dataset_config_path=dataset_config_path,
        dataset_root=dataset_root,
        candidate_root=candidate_root,
        old_replay_image_audit_path=old_replay_image_audit_path,
        evaluation_protocol_lock_path=evaluation_protocol_lock_path,
        evaluation_protocol_lock_sha256=evaluation_protocol_lock_sha256,
        output_root=output_root,
    )
    _write_json_exclusive(path, contract, read_only=True)
    artifact = Artifact(path, _sha256_file(path), path.stat().st_size)
    load_control_bundle(path, artifact.sha256)
    return artifact


def _write_json_exclusive(path: Path, value: Any, *, read_only: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open('x', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
    except FileExistsError as exc:
        raise FinalTestV4Error(f'refusing to overwrite immutable artifact: {path}') from exc
    if read_only:
        path.chmod(0o444)


def reserve_final_test_attempt(
    contract_path: Path | str,
    expected_contract_sha256: str,
) -> ReservedAttempt:
    """Reserve the global attempt before any Test row, label, or image access."""

    bundle = load_control_bundle(contract_path, expected_contract_sha256)
    # The ledger is deliberately resolved from the module-level global at call
    # time.  Production callers have no path override; tests may monkeypatch
    # the constant without weakening the public API.
    ledger = Path(DEFAULT_GLOBAL_LEDGER_ROOT).expanduser().resolve()
    reservation_path = ledger / f'{bundle.attempt_key}.reserved.json'
    completion_path = ledger / f'{bundle.attempt_key}.completed.json'
    if reservation_path.exists() or completion_path.exists():
        raise FinalTestV4Error(
            'the immutable final Test attempt is already reserved or completed: '
            f'{bundle.attempt_key}'
        )
    reservation = {
        'schema_version': ATTEMPT_SCHEMA_VERSION,
        'state': 'reserved_immutable',
        'attempt_key': bundle.attempt_key,
        'reserved_at_utc': datetime.now(timezone.utc).isoformat(),
        'contract': bundle.artifacts['contract'].as_dict(),
        'evaluation_protocol_lock': bundle.artifacts[
            'evaluation_protocol_lock'
        ].as_dict(),
        'protocol_frozen_sha256': bundle.evaluation_protocol_lock[
            'protocol_frozen_sha256'
        ],
        'implementation_aggregate_sha256': bundle.evaluation_protocol_lock[
            'implementation_aggregate_sha256'
        ],
        'dataset_config': bundle.artifacts['dataset_config'].as_dict(),
        'preregistration': bundle.artifacts['preregistration'].as_dict(),
        'plan_lock': bundle.artifacts['plan_lock'].as_dict(),
        'dataset_finalization': bundle.artifacts['finalization'].as_dict(),
        'checkpoint': bundle.artifacts['checkpoint'].as_dict(),
        'effective_config_canonical_sha256': _sha256_canonical(
            bundle.effective_config
        ),
        'model_contract_sha256': _sha256_canonical(
            bundle.runtime_manifest['model_contract']
        ),
        'topology_fingerprint_sha256': bundle.topology_contract[
            'fingerprint_sha256'
        ],
        'validation_calibration': bundle.artifacts[
            'validation_segment_calibration'
        ].as_dict(),
        'runtime_manifest': bundle.artifacts['runtime_promotion_manifest'].as_dict(),
        'acceptance_gates_sha256': _sha256_canonical(
            bundle.effective_config['pilot_acceptance_gates']
        ),
        'output': str(bundle.output_path),
        'rows_opened': False,
        'labels_opened': False,
        'images_opened': False,
        'training_performed': False,
        'checkpoint_selection_performed': False,
        'calibration_refit_performed': False,
        'threshold_selection_performed': False,
        'canary_opened': False,
        'historical_test_opened': False,
        'automatic_runtime_switch': False,
    }
    _write_json_exclusive(reservation_path, reservation, read_only=True)
    return ReservedAttempt(
        bundle=bundle,
        ledger_root=ledger,
        reservation_path=reservation_path,
        reservation_sha256=_sha256_file(reservation_path),
        completion_path=completion_path,
    )


def _assert_reservation_intact(reserved: ReservedAttempt) -> None:
    if not reserved.reservation_path.is_file():
        raise FinalTestV4Error('global final Test reservation disappeared')
    if _sha256_file(reserved.reservation_path) != reserved.reservation_sha256:
        raise FinalTestV4Error('global final Test reservation changed after creation')
    reservation = _read_json_object(
        reserved.reservation_path, 'global final Test reservation'
    )
    if (
        reservation.get('state') != 'reserved_immutable'
        or reservation.get('attempt_key') != reserved.bundle.attempt_key
        or reservation.get('rows_opened') is not False
        or reservation.get('labels_opened') is not False
        or reservation.get('images_opened') is not False
    ):
        raise FinalTestV4Error('global final Test reservation contract is invalid')


def _read_jsonl_reserved(
    reserved: ReservedAttempt,
    path: Path,
    expected_sha256: str,
    context: str,
) -> tuple[list[dict[str, Any]], Artifact]:
    _assert_reservation_intact(reserved)
    artifact = _fingerprint(path, expected_sha256, context)
    if artifact.sha256 in FORBIDDEN_EXPOSED_TEST_FILE_SHA256:
        raise FinalTestV4Error(f'{context} is a historically exposed Test artifact')
    rows: list[dict[str, Any]] = []
    with artifact.path.open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                raise FinalTestV4Error(
                    f'{context} contains a blank line at {line_number}'
                )
            try:
                value = json.loads(line)
            except json.JSONDecodeError as exc:
                raise FinalTestV4Error(
                    f'{context} contains invalid JSON at line {line_number}'
                ) from exc
            if not isinstance(value, dict):
                raise FinalTestV4Error(
                    f'{context} line {line_number} is not a JSON object'
                )
            rows.append(value)
    return rows, artifact


def _resolve_test_image(
    image_root: Path,
    reference: Any,
    *,
    context: str,
) -> Path:
    text = str(reference or '').strip()
    if not text:
        raise FinalTestV4Error(f'{context} image reference is empty')
    raw = Path(text).expanduser()
    candidate = (raw if raw.is_absolute() else image_root / raw).resolve()
    candidate = assert_fresh_final_test_path(candidate, context=f'{context} image')
    _require_within(candidate, image_root, f'{context} image')
    if not candidate.is_file():
        raise FinalTestV4Error(f'{context} image is missing: {candidate}')
    return candidate


def _sample_id(value: Mapping[str, Any], context: str) -> str:
    sample_id = str(value.get('sample_id') or '').strip()
    if not sample_id:
        raise FinalTestV4Error(f'{context} lacks sample_id')
    return sample_id


def _verify_fresh_row_metadata(
    row: Mapping[str, Any],
    label: Mapping[str, Any],
    *,
    sample_id: str,
) -> dict[str, Any]:
    if not sample_id.startswith('v4_final_test_'):
        raise FinalTestV4Error(
            f'final Test sample_id has a non-fresh prefix: {sample_id!r}'
        )
    episode_id = str(row.get('episode_id') or '').strip()
    if (
        not episode_id.startswith('v4_final_test_')
        or str(label.get('episode_id') or '').strip() != episode_id
    ):
        raise FinalTestV4Error(f'{sample_id} has a non-fresh/mismatched episode_id')
    scenario_family = str(row.get('scenario_family') or '').strip()
    if (
        not scenario_family.startswith('v4_final_test_family_')
        or str(label.get('scenario_family') or '').strip() != scenario_family
    ):
        raise FinalTestV4Error(
            f'{sample_id} has a non-fresh/mismatched scenario_family'
        )
    row_trace = _required_mapping(
        row.get('traceability_metadata'), f'{sample_id} row traceability'
    )
    label_trace = _required_mapping(
        label.get('traceability_metadata'), f'{sample_id} label traceability'
    )
    if dict(row_trace) != dict(label_trace):
        raise FinalTestV4Error(f'{sample_id} row/label traceability differs')
    for name in ('dataset_partition', 'source_profile'):
        if row_trace.get(name) != 'final_test':
            raise FinalTestV4Error(f'{sample_id} trace {name} is not final_test')
    if row_trace.get('imported_from_v3') is not False:
        raise FinalTestV4Error(f'{sample_id} is marked imported_from_v3')
    for name in ('configuration_family_id', 'configuration_core_family_id'):
        value = str(row_trace.get(name) or '').strip()
        if not value.startswith('v4_final_test_family_'):
            raise FinalTestV4Error(f'{sample_id} has non-fresh trace {name}')
    if row.get('dataset_mode') != 'visual_state' or label.get('dataset_mode') != 'visual_state':
        raise FinalTestV4Error(f'{sample_id} is not a visual_state sample')
    if (
        label.get('label_source') != 'oracle'
        or label.get('model_input_exposure') != 'excluded'
    ):
        raise FinalTestV4Error(f'{sample_id} oracle-label isolation is invalid')
    return dict(row_trace)


def _load_reserved_records(
    reserved: ReservedAttempt,
) -> tuple[
    list[trainer.PairedRecord],
    dict[str, Artifact],
    dict[str, str],
    dict[str, str],
]:
    """Open and validate Test rows/labels/images after reservation only."""

    _assert_reservation_intact(reserved)
    contract = reserved.bundle.contract
    dataset = _required_mapping(contract.get('dataset'), 'contract.dataset')
    dataset_root = assert_fresh_final_test_path(
        dataset.get('root', ''), context='final Test dataset root'
    )
    image_root = assert_fresh_final_test_path(
        dataset.get('image_root', ''), context='final Test image root'
    )
    if not dataset_root.is_dir():
        raise FinalTestV4Error(f'final Test dataset root is missing: {dataset_root}')
    rows_path, rows_sha = _artifact_spec(dataset, 'rows', context='contract.dataset')
    labels_path, labels_sha = _artifact_spec(dataset, 'labels', context='contract.dataset')
    rows, rows_artifact = _read_jsonl_reserved(
        reserved, rows_path, rows_sha, 'final Test rows'
    )
    labels, labels_artifact = _read_jsonl_reserved(
        reserved, labels_path, labels_sha, 'final Test labels'
    )
    expected = int(dataset['sample_count'])
    if len(rows) != expected or len(labels) != expected:
        raise FinalTestV4Error(
            f'final Test count mismatch: rows={len(rows)}, labels={len(labels)}, '
            f'expected={expected}'
        )
    label_by_id: dict[str, dict[str, Any]] = {}
    for index, label in enumerate(labels):
        sample_id = _sample_id(label, f'final Test labels[{index}]')
        if sample_id in label_by_id:
            raise FinalTestV4Error(f'duplicate final Test label sample_id: {sample_id}')
        label_by_id[sample_id] = label
    row_ids: set[str] = set()
    records: list[trainer.PairedRecord] = []
    image_hashes: dict[str, str] = {}
    pair_hashes: dict[str, str] = {}
    for index, row in enumerate(rows):
        sample_id = _sample_id(row, f'final Test rows[{index}]')
        if sample_id in row_ids:
            raise FinalTestV4Error(f'duplicate final Test row sample_id: {sample_id}')
        row_ids.add(sample_id)
        label = label_by_id.get(sample_id)
        if label is None:
            raise FinalTestV4Error(f'final Test row lacks paired label: {sample_id}')
        trace = _verify_fresh_row_metadata(row, label, sample_id=sample_id)
        try:
            model_input = validate_visual_model_input(
                dict(row), context=f'final Test {sample_id}'
            )
            normalized = normalize_visual_state_labels(
                dict(label), context=f'final Test {sample_id}'
            )
        except Exception as exc:
            raise FinalTestV4Error(
                f'{sample_id} violates the V4 camera/label contract: {exc}'
            ) from exc
        references = _required_mapping(
            model_input.get('overhead_images'), f'{sample_id} overhead_images'
        )
        if set(references) != set(CAMERAS):
            raise FinalTestV4Error(
                f'{sample_id} must contain exactly the two rail cameras'
            )
        image_paths = {
            camera: _resolve_test_image(
                image_root,
                references[camera],
                context=f'{sample_id}:{camera}',
            )
            for camera in CAMERAS
        }
        episode_id = str(row['episode_id'])
        pair_entry = {'sample_id': sample_id}
        for camera, image_path in image_paths.items():
            try:
                size = trainer._validate_image_file(  # noqa: SLF001
                    image_path, f'final Test {sample_id}:{camera}'
                )
            except Exception as exc:
                raise FinalTestV4Error(str(exc)) from exc
            if size[0] * 3 != size[1] * 4:
                raise FinalTestV4Error(f'{sample_id}:{camera} is not 4:3')
            key = f'{episode_id}:{camera}'
            if key in image_hashes:
                raise FinalTestV4Error(f'duplicate final Test image key: {key}')
            digest = _sha256_file(image_path)
            image_hashes[key] = digest
            pair_entry['left_sha256' if camera == CAMERAS[0] else 'right_sha256'] = digest
        pair_hashes[sample_id] = _sha256_canonical(pair_entry)
        records.append(trainer.PairedRecord(
            sample_id=sample_id,
            source='sealed_final_test_v4',
            role='final_test',
            dataset_root=image_root,
            row=dict(row),
            label=dict(label),
            normalized_label=dict(normalized),
            image_paths=image_paths,
            trace=trace,
        ))
    if row_ids != set(label_by_id):
        extras = sorted(set(label_by_id) - row_ids)[:5]
        raise FinalTestV4Error(f'final Test has unpaired label IDs: {extras}')
    return (
        records,
        {'rows': rows_artifact, 'labels': labels_artifact},
        image_hashes,
        pair_hashes,
    )


def _nested_counter() -> dict[str, Counter[str]]:
    return {side: Counter() for side in SIDES}


def _presence_density(cardinality: int) -> str:
    if 1 <= cardinality <= 3:
        return 'sparse'
    if cardinality == 4:
        return 'medium'
    if 5 <= cardinality <= 8:
        return 'dense'
    raise FinalTestV4Error(f'invalid final Test presence cardinality: {cardinality}')


def _compute_support_summary(
    records: Sequence[trainer.PairedRecord],
) -> dict[str, Any]:
    visible_by_identity: Counter[str] = Counter()
    visible_by_side_x_segment = _nested_counter()
    visible_by_position_bin: Counter[str] = Counter()
    records_by_occlusion: Counter[str] = Counter()
    records_by_presence: Counter[str] = Counter()
    visible_by_target_zone: Counter[str] = Counter()
    visible_by_identity_zone: Counter[tuple[str, str]] = Counter()
    visible_total = 0
    for record in records:
        trace = record.trace
        occlusion = str(trace.get('occlusion_class') or '').strip().casefold()
        trace_presence = str(trace.get('presence_class') or '').strip().casefold()
        records_by_occlusion[occlusion] += 1
        position_map = _required_mapping(
            trace.get('identity_to_position_bin'),
            f'{record.sample_id} identity_to_position_bin',
        )
        zone_map = _required_mapping(
            trace.get('identity_to_zone'),
            f'{record.sample_id} identity_to_zone',
        )
        target_identity = str(trace.get('target_identity') or '').strip().upper()
        target_zone = str(trace.get('target_zone') or '').strip().casefold()
        shuttles = record.normalized_label.get('shuttles')
        if not isinstance(shuttles, list) or len(shuttles) != len(FIXED_IDENTITIES):
            raise FinalTestV4Error(
                f'{record.sample_id} normalized labels do not contain eight slots'
            )
        present_count = sum(
            bool(shuttle.get('presence'))
            for shuttle in shuttles
            if isinstance(shuttle, Mapping)
        )
        presence = _presence_density(present_count)
        if trace_presence != presence:
            raise FinalTestV4Error(
                f'{record.sample_id} trace presence density differs from labels'
            )
        records_by_presence[presence] += 1
        for shuttle in shuttles:
            if not isinstance(shuttle, Mapping):
                raise FinalTestV4Error(f'{record.sample_id} has invalid shuttle label')
            identity = str(shuttle.get('id') or '').strip().upper()
            if identity not in FIXED_IDENTITIES:
                raise FinalTestV4Error(
                    f'{record.sample_id} has invalid shuttle identity {identity!r}'
                )
            visible = bool(shuttle.get('visually_available'))
            if not visible:
                continue
            visible_total += 1
            visible_by_identity[identity] += 1
            location = _required_mapping(
                shuttle.get('location'), f'{record.sample_id}:{identity} location'
            )
            segment = str(location.get('block') or '').strip().upper()
            side = derive_side(identity)
            if segment not in SEGMENT_CLASSES:
                raise FinalTestV4Error(
                    f'{record.sample_id}:{identity} has invalid segment {segment!r}'
                )
            visible_by_side_x_segment[side][segment] += 1
            trace_position_bin = str(
                position_map.get(identity) or ''
            ).strip().casefold()
            identity_zone = str(zone_map.get(identity) or '').strip().casefold()
            rail_position = _required_mapping(
                shuttle.get('rail_position'),
                f'{record.sample_id}:{identity} rail_position',
            )
            observed_position_bin = position_bin(float(rail_position.get('s_ratio')))
            if (
                trace_position_bin not in POSITION_BINS
                or trace_position_bin != observed_position_bin
            ):
                raise FinalTestV4Error(
                    f'{record.sample_id}:{identity} position bin differs from labels'
                )
            if identity_zone not in IDENTITY_ZONES:
                raise FinalTestV4Error(
                    f'{record.sample_id}:{identity} lacks canonical identity zone'
                )
            visible_by_position_bin[observed_position_bin] += 1
            visible_by_identity_zone[(identity, identity_zone)] += 1
            if identity == target_identity:
                if target_zone not in TARGET_ZONES:
                    raise FinalTestV4Error(
                        f'{record.sample_id} lacks canonical target zone'
                    )
                visible_by_target_zone[target_zone] += 1
    return {
        'sample_count': len(records),
        'visible_total': visible_total,
        'visible_by_identity': {
            name: visible_by_identity[name] for name in FIXED_IDENTITIES
        },
        'visible_by_side_x_segment': {
            side: {
                segment: visible_by_side_x_segment[side][segment]
                for segment in SEGMENT_CLASSES
            }
            for side in SIDES
        },
        'visible_by_position_bin': {
            name: visible_by_position_bin[name] for name in POSITION_BINS
        },
        'records_by_occlusion_class': {
            name: records_by_occlusion[name] for name in SCENE_OCCLUSION_CLASSES
        },
        'records_by_presence_density': {
            name: records_by_presence[name] for name in SCENE_PRESENCE_DENSITIES
        },
        'visible_by_target_zone': {
            name: visible_by_target_zone[name] for name in TARGET_ZONES
        },
        'visible_by_identity_zone': {
            identity: {
                zone: visible_by_identity_zone[(identity, zone)]
                for zone in IDENTITY_ZONES
                if visible_by_identity_zone[(identity, zone)]
            }
            for identity in FIXED_IDENTITIES
        },
        'occlusion_claim_scope': (
            'static calibrated-projection risk estimate; not a measured '
            'pixel-occlusion percentage'
        ),
    }


def _validate_support_coverage(
    support: Mapping[str, Any],
    coverage: Mapping[str, Any],
) -> dict[str, Any]:
    checks: dict[str, bool] = {
        'sample_count': int(support['sample_count']) >= int(coverage['minimum_sample_count']),
        'visible_total': int(support['visible_total']) >= int(coverage['minimum_visible_total']),
    }
    identity_counts = _required_mapping(
        support.get('visible_by_identity'), 'support.visible_by_identity'
    )
    for identity in FIXED_IDENTITIES:
        checks[f'identity.{identity}'] = (
            int(identity_counts.get(identity, 0))
            >= int(coverage['minimum_visible_per_identity'])
        )
    cells = _required_mapping(
        support.get('visible_by_side_x_segment'),
        'support.visible_by_side_x_segment',
    )
    for side in SIDES:
        side_cells = _required_mapping(cells.get(side), f'support side {side}')
        for segment in SEGMENT_CLASSES:
            checks[f'side_x_segment.{side}.{segment}'] = (
                int(side_cells.get(segment, 0))
                >= int(coverage['minimum_visible_per_side_x_segment'])
            )
    categories = (
        (
            'position_bin',
            'visible_by_position_bin',
            POSITION_BINS,
            'minimum_visible_per_position_bin',
        ),
        (
            'scene_occlusion',
            'records_by_occlusion_class',
            SCENE_OCCLUSION_CLASSES,
            'minimum_records_per_scene_occlusion_class',
        ),
        (
            'scene_presence',
            'records_by_presence_density',
            SCENE_PRESENCE_DENSITIES,
            'minimum_records_per_scene_presence_density',
        ),
        (
            'target_zone',
            'visible_by_target_zone',
            TARGET_ZONES,
            'minimum_visible_per_target_zone',
        ),
    )
    for prefix, support_key, names, minimum_key in categories:
        values = _required_mapping(support.get(support_key), f'support.{support_key}')
        for name in names:
            checks[f'{prefix}.{name}'] = (
                int(values.get(name, 0)) >= int(coverage[minimum_key])
            )
    by_identity_zone = _required_mapping(
        support.get('visible_by_identity_zone'),
        'support.visible_by_identity_zone',
    )
    zone_totals = Counter()
    for identity in FIXED_IDENTITIES:
        by_zone = _required_mapping(
            by_identity_zone.get(identity),
            f'support.visible_by_identity_zone.{identity}',
        )
        for zone, count in by_zone.items():
            if zone not in IDENTITY_ZONES:
                raise FinalTestV4Error(
                    f'unsupported final Test identity zone: {zone!r}'
                )
            zone_totals[zone] += int(count)
    for zone in IDENTITY_ZONES:
        checks[f'identity_zone.{zone}'] = (
            zone_totals[zone]
            >= int(coverage['minimum_visible_per_identity_zone'])
        )
    failed = sorted(name for name, passed in checks.items() if not passed)
    return {
        'schema_version': 'room315.visual_v4.final_test_coverage_audit.v1',
        'passed': not failed,
        'checks': checks,
        'failed_checks': failed,
        'coverage_contract': dict(coverage),
        'observed_support': dict(support),
    }


def _verify_finalization_after_reservation(
    reserved: ReservedAttempt,
    records: Sequence[trainer.PairedRecord],
    rows_artifact: Artifact,
    labels_artifact: Artifact,
    image_hashes: Mapping[str, str],
    pair_hashes: Mapping[str, str],
    support: Mapping[str, Any],
) -> dict[str, Any]:
    _assert_reservation_intact(reserved)
    finalization_artifact = reserved.bundle.artifacts['finalization']
    finalization = _read_json_object(
        finalization_artifact.path, 'final Test finalization'
    )
    if finalization.get('schema_version') != FINALIZATION_SCHEMA_VERSION:
        raise FinalTestV4Error('final Test finalization schema is incompatible')
    if finalization.get('dataset_role') != DATASET_ROLE:
        raise FinalTestV4Error(
            f'finalization.dataset_role must be {DATASET_ROLE!r}'
        )
    if finalization.get('passed') is not True:
        raise FinalTestV4Error('final Test dataset did not pass finalization')
    if (
        finalization.get('inference_status') != 'not_run'
        or finalization.get('inference_count') != 0
        or finalization.get('historical_test_accessed') is not False
    ):
        raise FinalTestV4Error('final Test inference was already exposed before reservation')
    if (
        int(finalization.get('scenario_count', -1)) != len(records)
        or int(finalization.get('image_count', -1)) != len(image_hashes)
    ):
        raise FinalTestV4Error('final Test finalization counts differ from loaded data')
    for name, artifact in (('rows', rows_artifact), ('labels', labels_artifact)):
        value = _required_mapping(finalization.get(name), f'finalization.{name}')
        if (
            Path(str(value.get('path') or '')).expanduser().resolve() != artifact.path
            or _validated_sha256(
                value.get('sha256'), f'finalization.{name}.sha256'
            ) != artifact.sha256
            or int(value.get('bytes', artifact.bytes)) != artifact.bytes
        ):
            raise FinalTestV4Error(f'finalization.{name} binding failed')
    images = _required_mapping(finalization.get('images'), 'finalization.images')
    frozen_individual = _required_mapping(
        images.get('individual_sha256_by_episode_camera'),
        'finalization individual image hashes',
    )
    frozen_pairs = _required_mapping(
        images.get('pair_sha256_by_sample_id'),
        'finalization pair hashes',
    )
    if dict(frozen_individual) != dict(image_hashes):
        raise FinalTestV4Error('loaded image bytes differ from frozen finalization')
    if dict(frozen_pairs) != dict(pair_hashes):
        raise FinalTestV4Error('loaded image pairs differ from frozen finalization')
    observed_image_manifest_sha256 = _sha256_canonical({
        'individual_sha256_by_episode_camera': dict(sorted(image_hashes.items())),
        'pair_sha256_by_sample_id': dict(sorted(pair_hashes.items())),
    })
    declared_image_manifest_sha256 = _validated_sha256(
        _required_mapping(
            reserved.bundle.contract.get('dataset'), 'contract.dataset'
        ).get('image_manifest_sha256'),
        'contract.dataset.image_manifest_sha256',
    )
    if observed_image_manifest_sha256 != declared_image_manifest_sha256:
        raise FinalTestV4Error('final Test image-manifest fingerprint differs from contract')
    if (
        images.get('individual_unique') is not True
        or images.get('pair_unique') is not True
        or images.get('pair_content_unique') is not True
        or len(set(image_hashes.values())) != len(image_hashes)
        or len(set(pair_hashes.values())) != len(pair_hashes)
    ):
        raise FinalTestV4Error('final Test images/pairs are not globally unique')
    frozen_support = _required_mapping(
        finalization.get('support_summary'), 'finalization.support_summary'
    )
    support_keys = (
        'visible_by_identity',
        'visible_by_side_x_segment',
        'visible_by_position_bin',
        'records_by_occlusion_class',
        'records_by_presence_density',
        'visible_by_target_zone',
        'visible_by_identity_zone',
        'occlusion_claim_scope',
    )
    if set(frozen_support) != set(support_keys):
        raise FinalTestV4Error('finalization support summary keys are incompatible')
    for name in support_keys:
        if frozen_support.get(name) != support.get(name):
            raise FinalTestV4Error(
                f'finalization support summary differs for {name}'
            )
    finalized_at = _parse_utc(
        finalization.get('capture_completed_at_utc'),
        'finalization capture completion',
    )
    created_at = _parse_utc(
        finalization.get('created_at_utc'), 'finalization creation'
    )
    declared_generation_started = _parse_utc(
        finalization.get('generation_started_at_utc'),
        'finalization generation start',
    )
    generation_started = _parse_utc(
        reserved.bundle.plan_lock.get('locked_at_utc'),
        'plan_lock locked_at_utc',
    )
    preregistered_at = _parse_utc(
        reserved.bundle.preregistration.get('preregistered_at_utc'),
        'preregistration preregistered_at_utc',
    )
    if declared_generation_started != preregistered_at:
        raise FinalTestV4Error(
            'finalization generation start differs from preregistration'
        )
    if created_at != finalized_at or finalized_at <= generation_started:
        raise FinalTestV4Error('final Test finalization predates generation start')

    frozen_configuration = _required_mapping(
        finalization.get('configuration'), 'finalization.configuration'
    )
    locked_configuration = _required_mapping(
        reserved.bundle.plan_lock.get('configuration'), 'plan_lock.configuration'
    )
    if dict(frozen_configuration) != dict(locked_configuration):
        raise FinalTestV4Error(
            'finalization configuration differs from the literal plan-lock copy'
        )
    for artifact_name, finalization_name in (
        ('dataset_config', 'configuration'),
        ('preregistration', 'preregistration'),
        ('plan_lock', 'plan_lock'),
    ):
        value = _required_mapping(
            finalization.get(finalization_name), f'finalization.{finalization_name}'
        )
        expected_artifact = reserved.bundle.artifacts[artifact_name]
        if (
            Path(str(value.get('path') or '')).expanduser().resolve()
            != expected_artifact.path
            or _validated_sha256(
                value.get('sha256'), f'finalization.{artifact_name}.sha256'
            ) != expected_artifact.sha256
        ):
            raise FinalTestV4Error(
                f'finalization does not bind the exact {artifact_name}'
            )

    dataset_root = Path(
        str(reserved.bundle.contract['dataset']['root'])
    ).expanduser().resolve()
    scenario_manifest = _required_mapping(
        finalization.get('scenario_manifest'), 'finalization.scenario_manifest'
    )
    scenario_manifest_path = Path(
        str(scenario_manifest.get('path') or '')
    ).expanduser().resolve()
    if scenario_manifest_path != dataset_root / 'scenario_manifest.jsonl':
        raise FinalTestV4Error('finalization scenario manifest path is not canonical')
    plan_artifacts = _required_mapping(
        reserved.bundle.plan_lock.get('artifacts'), 'plan_lock.artifacts'
    )
    scenario_manifest_sha = _validated_sha256(
        scenario_manifest.get('sha256'), 'finalization.scenario_manifest.sha256'
    )
    if scenario_manifest_sha != _validated_sha256(
        plan_artifacts.get('scenario_manifest.jsonl'),
        'plan_lock.artifacts.scenario_manifest.jsonl',
    ):
        raise FinalTestV4Error(
            'finalization scenario manifest differs from the locked manifest'
        )
    scenario_manifest_artifact = _fingerprint(
        scenario_manifest_path,
        scenario_manifest_sha,
        'locked final Test scenario manifest',
    )
    plan_verification = _required_mapping(
        finalization.get('plan_verification'), 'finalization.plan_verification'
    )
    if (
        plan_verification.get('passed') is not True
        or _validated_sha256(
            plan_verification.get('manifest_sha256'),
            'finalization.plan_verification.manifest_sha256',
        ) != scenario_manifest_artifact.sha256
    ):
        raise FinalTestV4Error('finalization plan verification did not bind the manifest')
    visual_validation = _required_mapping(
        finalization.get('visual_state_validation'),
        'finalization.visual_state_validation',
    )
    if visual_validation.get('issues') not in ([], None):
        raise FinalTestV4Error('finalization visual-state validation has issues')

    disjoint = _required_mapping(
        finalization.get('disjoint_audit'), 'finalization.disjoint_audit'
    )
    disjoint_path = Path(str(disjoint.get('path') or '')).expanduser().resolve()
    if disjoint_path != dataset_root / 'finalized' / 'final_test_disjoint_audit.json':
        raise FinalTestV4Error('finalization disjoint-audit path is not canonical')
    return {
        **finalization,
        'artifact': finalization_artifact.as_dict(),
        'scenario_manifest_artifact': scenario_manifest_artifact.as_dict(),
        'image_manifest_sha256': observed_image_manifest_sha256,
        'verified_after_global_reservation': True,
    }


def _mapping_at_path(root: Mapping[str, Any], path: Sequence[str]) -> Mapping[str, Any]:
    current: Any = root
    for name in path:
        if not isinstance(current, Mapping):
            raise FinalTestV4Error(
                f'historical image hash field is not an object: {".".join(path)}'
            )
        current = current.get(name)
    return _required_mapping(current, f'historical hash field {".".join(path)}')


def _verify_historical_disjointness(
    reserved: ReservedAttempt,
    image_hashes: Mapping[str, str],
    rows_artifact: Artifact,
    labels_artifact: Artifact,
) -> dict[str, Any]:
    references = reserved.bundle.contract.get('historical_image_references')
    if not isinstance(references, list):
        raise FinalTestV4Error('historical_image_references must be a list')
    required_roles = {
        'old_replay_superset',
        'v3r1_train',
        'v3r1_validation',
        'v3r1_canary',
    }
    observed_roles: set[str] = set()
    details: dict[str, Any] = {}
    final_hashes = set(image_hashes.values())
    for raw in references:
        reference = _required_mapping(raw, 'historical image reference')
        role = str(reference.get('role') or '').strip()
        if role in observed_roles or role not in required_roles:
            raise FinalTestV4Error(f'invalid/duplicate historical reference role: {role!r}')
        observed_roles.add(role)
        artifact = _fingerprint(
            reference.get('path', ''),
            reference.get('sha256'),
            f'historical image reference {role}',
        )
        parsed = _read_json_object(artifact.path, f'historical reference {role}')
        field = reference.get('hash_field_path')
        if not isinstance(field, list) or not field or any(
            not isinstance(name, str) or not name for name in field
        ):
            raise FinalTestV4Error(
                f'historical reference {role} needs hash_field_path'
            )
        historical_map = _mapping_at_path(parsed, field)
        historical_hashes = {
            _validated_sha256(value, f'historical {role} image hash')
            for value in historical_map.values()
        }
        overlap = sorted(historical_hashes & final_hashes)
        if overlap:
            raise FinalTestV4Error(
                f'fresh final Test reuses {role} image bytes: {overlap[:5]}'
            )
        details[role] = {
            'artifact': artifact.as_dict(),
            'hash_field_path': field,
            'historical_image_hash_count': len(historical_hashes),
            'final_test_image_hash_count': len(final_hashes),
            'overlap_count': 0,
        }
    if observed_roles != required_roles:
        raise FinalTestV4Error(
            f'historical references are incomplete: {sorted(required_roles - observed_roles)}'
        )
    historical_data_hashes: set[str] = set(FORBIDDEN_EXPOSED_TEST_FILE_SHA256)
    reference_sources = reserved.bundle.dataset_config.get('reference_sources')
    if not isinstance(reference_sources, list) or not reference_sources:
        raise FinalTestV4Error('dataset config reference_sources is missing')
    for source in reference_sources:
        item = _required_mapping(source, 'dataset config reference source')
        historical_data_hashes.add(
            _validated_sha256(item.get('rows_sha256'), 'historical rows SHA')
        )
        historical_data_hashes.add(
            _validated_sha256(item.get('labels_sha256'), 'historical labels SHA')
        )
    if rows_artifact.sha256 in historical_data_hashes:
        raise FinalTestV4Error('final Test rows reuse a historical data artifact')
    if labels_artifact.sha256 in historical_data_hashes:
        raise FinalTestV4Error('final Test labels reuse a historical data artifact')
    return {
        'schema_version': 'room315.visual_v4.final_test_disjoint_audit.v1',
        'passed': True,
        'historical_test_paths_refused': True,
        'historical_exposed_test_hashes_refused': True,
        'fresh_rows_sha256': rows_artifact.sha256,
        'fresh_labels_sha256': labels_artifact.sha256,
        'fresh_image_hash_count': len(final_hashes),
        'references': details,
    }


def _load_frozen_model(
    reserved: ReservedAttempt,
    records: Sequence[trainer.PairedRecord],
) -> tuple[Any, Any, Any, Any, Mapping[str, Any], Mapping[str, Any], dict[str, Any]]:
    """Strictly load the already-selected checkpoint; never modify it."""

    torch_module, torchvision_module, training_api = trainer.require_training_stack()
    seed = int(reserved.bundle.effective_config['training']['seed'])
    trainer.set_deterministic(torch_module, seed)
    device = trainer._resolve_device(  # noqa: SLF001
        torch_module,
        EVALUATION_DEVICE,
        None,
    )
    model = trainer.build_configured_model(
        reserved.bundle.effective_config,
        torch_module,
        torchvision_module,
    ).to(device)
    checkpoint = trainer._torch_load(  # noqa: SLF001
        torch_module,
        reserved.bundle.artifacts['checkpoint'].path,
        map_location='cpu',
    )
    if not isinstance(checkpoint, Mapping):
        raise FinalTestV4Error('V4 checkpoint is not an object')
    checks = {
        'schema_version': checkpoint.get('schema_version') == CHECKPOINT_SCHEMA_VERSION,
        'model_kind': checkpoint.get('model_kind') == V4_MODEL_KIND,
        'slot_order': tuple(checkpoint.get('slot_order') or ()) == tuple(FIXED_IDENTITIES),
        'segment_order': tuple(checkpoint.get('segment_order') or ())
        == tuple(SEGMENT_CLASSES),
        'selection_role': checkpoint.get('checkpoint_selection_role')
        == 'validation_only',
        'canary_unseen_at_selection': checkpoint.get('canary_seen') is False,
        'test_unseen_at_selection': checkpoint.get('test_seen') is False,
        'effective_config': checkpoint.get('effective_config_sha256')
        == _sha256_canonical(reserved.bundle.effective_config),
        'topology_fingerprint': checkpoint.get(
            'topology_length_mapping_fingerprint_sha256'
        )
        == reserved.bundle.topology_contract.get('fingerprint_sha256'),
        'topology_contract': checkpoint.get('public_topology_contract')
        == reserved.bundle.topology_contract,
    }
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise FinalTestV4Error(f'frozen V4 checkpoint contract failed: {failed}')
    state = checkpoint.get('model_state_dict')
    if not isinstance(state, Mapping):
        raise FinalTestV4Error('V4 checkpoint lacks model_state_dict')
    try:
        model.load_state_dict(state, strict=True)
    except Exception as exc:
        raise FinalTestV4Error(f'V4 checkpoint strict load failed: {exc}') from exc
    class_weights = checkpoint.get('class_weights_by_side')
    topology_lengths = checkpoint.get('topology_lengths_by_side')
    if class_weights is None or not isinstance(topology_lengths, Mapping):
        raise FinalTestV4Error('V4 checkpoint lacks frozen train-only statistics')
    current_topology = trainer.load_public_topology_contract(
        reserved.bundle.effective_config
    )
    if current_topology != reserved.bundle.topology_contract:
        raise FinalTestV4Error('authoritative public topology changed after freeze')
    if topology_lengths != current_topology.get('lengths_by_side'):
        raise FinalTestV4Error('checkpoint topology lengths differ from public topology')
    topology_label_audit = trainer.audit_labels_against_public_topology(
        [records], current_topology
    )
    return (
        torch_module,
        training_api,
        device,
        model,
        checkpoint,
        topology_lengths,
        {
            'checkpoint_contract_checks': checks,
            'topology_label_audit': topology_label_audit,
            'device': str(device),
            'strict_state_dict_load': True,
            'training_performed': False,
        },
    )


def _collect_confidence_tensors(
    model: Any,
    records: Sequence[trainer.PairedRecord],
    config: Mapping[str, Any],
    *,
    torch_module: Any,
    device: Any,
    epoch: int,
) -> dict[str, Any]:
    loader = trainer.make_loader(
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
    parts: dict[str, list[Any]] = {
        'segment_logits': [],
        'loaded_logits': [],
        'segment_targets': [],
        'loaded_targets': [],
        'visibility_mask': [],
    }
    model.eval()
    with torch_module.inference_mode():
        for batch in loader:
            image = batch['image'].to(device, non_blocking=True)
            with trainer._autocast_context(  # noqa: SLF001
                torch_module, device, amp_enabled
            ):
                outputs = model(image)
            parts['segment_logits'].append(
                outputs['segment_logits'].detach().float().cpu()
            )
            parts['loaded_logits'].append(
                outputs['loaded_logits'].detach().float().cpu()
            )
            parts['segment_targets'].append(batch['segment'].detach().cpu())
            parts['loaded_targets'].append(batch['loaded'].detach().cpu())
            parts['visibility_mask'].append(
                batch['visibility_mask'].detach().cpu()
            )
    if any(not values for values in parts.values()):
        raise FinalTestV4Error('confidence tensor collection produced no batches')
    return {
        name: torch_module.cat(values, dim=0)
        for name, values in parts.items()
    }


def _calibration_view(
    logits: Any,
    targets: Any,
    mask: Any,
    *,
    temperature: float,
    coverages: Sequence[float],
    ece_bins: int,
    calibration_api: Any,
    torch_module: Any,
) -> dict[str, Any]:
    visible_count = int(mask.sum())
    if visible_count <= 0:
        return {
            'available': False,
            'visible_count': 0,
            'temperature': temperature,
            'nll': None,
            'ece': None,
            'accuracy': None,
            'mean_confidence': None,
            'selective_curve': [],
        }
    nll = calibration_api.segment_negative_log_likelihood(
        logits, targets, mask, temperature=temperature
    )
    ece = calibration_api.segment_expected_calibration_error(
        logits,
        targets,
        mask,
        temperature=temperature,
        ece_bins=ece_bins,
    )
    curve = calibration_api.segment_selective_accuracy_curve(
        logits,
        targets,
        mask,
        temperature=temperature,
        coverages=coverages,
    )
    probabilities = torch_module.softmax(
        logits.detach().to(dtype=torch_module.float64) / temperature,
        dim=-1,
    )
    confidence, prediction = probabilities.max(dim=-1)
    visible_confidence = confidence[mask]
    visible_correct = prediction[mask].eq(targets[mask])
    return {
        'available': True,
        'visible_count': visible_count,
        'temperature': temperature,
        'nll': float(nll),
        'ece': float(ece),
        'accuracy': float(visible_correct.to(dtype=torch_module.float64).mean()),
        'mean_confidence': float(visible_confidence.mean()),
        'selective_curve': curve,
    }


def _fixed_final_test_calibration(
    tensors: Mapping[str, Any],
    bundle: ControlBundle,
    *,
    torch_module: Any,
) -> dict[str, Any]:
    """Evaluate only the saved validation temperature; fitting is impossible."""

    try:
        import room_315_visual_calibration_v4 as calibration_api
    except Exception as exc:
        raise FinalTestV4Error(f'cannot import V4 calibration metrics: {exc}') from exc
    temperature = float(bundle.validation_calibration['temperature'])
    coverages = tuple(bundle.validation_calibration['coverage_targets'])
    ece_bins = int(bundle.validation_calibration['ece_bins'])
    logits = tensors['segment_logits']
    targets = tensors['segment_targets']
    visibility = tensors['visibility_mask'].to(dtype=torch_module.bool)
    slot_indexes = torch_module.arange(len(FIXED_IDENTITIES)).unsqueeze(0)
    per_side: dict[str, Any] = {}
    for side in SIDES:
        side_mask = visibility & (
            (slot_indexes < 4).expand_as(visibility)
            if side == 'left'
            else (slot_indexes >= 4).expand_as(visibility)
        )
        per_side[side] = _calibration_view(
            logits,
            targets,
            side_mask,
            temperature=temperature,
            coverages=coverages,
            ece_bins=ece_bins,
            calibration_api=calibration_api,
            torch_module=torch_module,
        )
    global_view = _calibration_view(
        logits,
        targets,
        visibility,
        temperature=temperature,
        coverages=coverages,
        ece_bins=ece_bins,
        calibration_api=calibration_api,
        torch_module=torch_module,
    )
    return {
        'schema_version': CALIBRATION_REPORT_SCHEMA_VERSION,
        'data_role': DATASET_ROLE,
        'source_temperature_role': 'validation',
        'source_artifact': bundle.artifacts[
            'validation_segment_calibration'
        ].as_dict(),
        'temperature': temperature,
        'fit_performed': False,
        'threshold_selection_performed': False,
        'coverage_targets': list(coverages),
        'ece_bins': ece_bins,
        **global_view,
        'per_side': per_side,
    }


def _threshold_view(
    confidence: Any,
    prediction: Any,
    target: Any,
    mask: Any,
    threshold: float,
    *,
    torch_module: Any,
) -> dict[str, Any]:
    visible = mask.to(dtype=torch_module.bool)
    accepted = visible & confidence.ge(threshold)
    visible_count = int(visible.sum())
    accepted_count = int(accepted.sum())
    correct = prediction.eq(target)
    return {
        'threshold': threshold,
        'visible_count': visible_count,
        'accepted_count': accepted_count,
        'coverage': accepted_count / visible_count if visible_count else 0.0,
        'selective_accuracy': (
            float(correct[accepted].to(dtype=torch_module.float64).mean())
            if accepted_count else None
        ),
        'correct_and_accepted_rate': (
            float((correct & accepted).sum()) / visible_count
            if visible_count else 0.0
        ),
    }


def _runtime_threshold_report(
    tensors: Mapping[str, Any],
    bundle: ControlBundle,
    *,
    torch_module: Any,
) -> dict[str, Any]:
    thresholds = bundle.runtime_manifest['acceptance_thresholds']
    temperature = float(bundle.validation_calibration['temperature'])
    segment_threshold = float(thresholds['minimum_segment_confidence'])
    loaded_threshold = float(thresholds['minimum_loaded_confidence'])
    segment_probability = torch_module.softmax(
        tensors['segment_logits'].to(dtype=torch_module.float64) / temperature,
        dim=-1,
    )
    loaded_probability = torch_module.softmax(
        tensors['loaded_logits'].to(dtype=torch_module.float64), dim=-1
    )
    segment_confidence, segment_prediction = segment_probability.max(dim=-1)
    loaded_confidence, loaded_prediction = loaded_probability.max(dim=-1)
    visibility = tensors['visibility_mask'].to(dtype=torch_module.bool)
    segment_view = _threshold_view(
        segment_confidence,
        segment_prediction,
        tensors['segment_targets'],
        visibility,
        segment_threshold,
        torch_module=torch_module,
    )
    loaded_view = _threshold_view(
        loaded_confidence,
        loaded_prediction,
        tensors['loaded_targets'],
        visibility,
        loaded_threshold,
        torch_module=torch_module,
    )
    joint_mask = visibility & segment_confidence.ge(segment_threshold) & loaded_confidence.ge(
        loaded_threshold
    )
    visible_count = int(visibility.sum())
    joint_count = int(joint_mask.sum())
    both_correct = segment_prediction.eq(tensors['segment_targets']) & loaded_prediction.eq(
        tensors['loaded_targets']
    )
    per_side: dict[str, Any] = {}
    slots = torch_module.arange(len(FIXED_IDENTITIES)).unsqueeze(0)
    for side in SIDES:
        side_mask = visibility & (
            (slots < 4).expand_as(visibility)
            if side == 'left'
            else (slots >= 4).expand_as(visibility)
        )
        per_side[side] = {
            'segment': _threshold_view(
                segment_confidence,
                segment_prediction,
                tensors['segment_targets'],
                side_mask,
                segment_threshold,
                torch_module=torch_module,
            ),
            'loaded': _threshold_view(
                loaded_confidence,
                loaded_prediction,
                tensors['loaded_targets'],
                side_mask,
                loaded_threshold,
                torch_module=torch_module,
            ),
        }
    return {
        'schema_version': 'room315.visual_v4.final_test_runtime_thresholds.v1',
        'data_role': DATASET_ROLE,
        'threshold_source': 'frozen_validation_and_runtime_manifest_only',
        'runtime_manifest': bundle.artifacts['runtime_promotion_manifest'].as_dict(),
        'validation_temperature': temperature,
        'segment': segment_view,
        'loaded': loaded_view,
        'joint': {
            'visible_count': visible_count,
            'accepted_count': joint_count,
            'coverage': joint_count / visible_count if visible_count else 0.0,
            'selective_both_correct_accuracy': (
                float(both_correct[joint_mask].to(dtype=torch_module.float64).mean())
                if joint_count else None
            ),
        },
        'per_side': per_side,
        'calibration_refit_performed': False,
        'threshold_selection_performed': False,
    }


def _combine_final_test_acceptance(
    base_acceptance: Mapping[str, Any],
    runtime_thresholds: Mapping[str, Any],
    coverage_audit: Mapping[str, Any],
    coverage_contract: Mapping[str, Any],
) -> dict[str, Any]:
    runtime_contract = _required_mapping(
        coverage_contract.get('runtime_threshold_gates'),
        'coverage_contract.runtime_threshold_gates',
    )
    observations = {
        'minimum_segment_confidence_coverage': float(
            runtime_thresholds['segment']['coverage']
        ),
        'minimum_segment_selective_accuracy': runtime_thresholds['segment'][
            'selective_accuracy'
        ],
        'minimum_loaded_confidence_coverage': float(
            runtime_thresholds['loaded']['coverage']
        ),
        'minimum_joint_confidence_coverage': float(
            runtime_thresholds['joint']['coverage']
        ),
    }
    per_gate: dict[str, Any] = {}
    for name, observed_raw in observations.items():
        threshold = float(runtime_contract[name])
        observed = None if observed_raw is None else float(observed_raw)
        per_gate[f'runtime_threshold.{name}'] = {
            'status': (
                'pending'
                if observed is None or not math.isfinite(observed)
                else 'passed'
                if observed >= threshold
                else 'failed'
            ),
            'required': True,
            'comparison': '>=',
            'observed': observed,
            'threshold': threshold,
            'threshold_source': 'preregistered_final_test_coverage_contract',
        }
    base_status = str(base_acceptance.get('status') or 'pending')
    runtime_statuses = [item['status'] for item in per_gate.values()]
    if coverage_audit.get('passed') is not True or 'failed' in runtime_statuses:
        status = 'failed'
    elif base_status == 'failed':
        status = 'failed'
    elif base_status != 'passed' or 'pending' in runtime_statuses:
        status = 'pending'
    else:
        status = 'passed'
    return {
        'schema_version': 'room315.visual_v4.final_test_acceptance.v1',
        'data_role': DATASET_ROLE,
        'status': status,
        'accepted': status == 'passed',
        'base_v4_acceptance': dict(base_acceptance),
        'test_specific_runtime_gates': per_gate,
        'dataset_coverage_passed': coverage_audit.get('passed') is True,
        'automatic_runtime_switch': False,
        'human_review_required': True,
    }


FINAL_ARTIFACT_NAMES = (
    'attempt_started.json',
    'evaluation_contract.json',
    'evaluation_protocol_lock.json',
    'dataset_config.json',
    'preregistration.json',
    'plan_lock.json',
    'final_test_finalization.json',
    'final_test_input_fingerprint.json',
    'final_test_image_pair_manifest.json',
    'final_test_coverage_audit.json',
    'final_test_disjoint_audit.json',
    'final_test_metrics.json',
    'final_test_camera_counterfactuals.json',
    'final_test_segment_calibration.json',
    'final_test_runtime_thresholds.json',
    'final_test_acceptance.json',
    'final_report.json',
)


def _complete_attempt(
    reserved: ReservedAttempt,
    *,
    state: str,
    failure: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    if state not in {'completed_immutable', 'failed_immutable'}:
        raise FinalTestV4Error(f'invalid final Test completion state: {state}')
    output = reserved.bundle.output_path
    artifacts: dict[str, Any] = {}
    if output.is_dir():
        for path in sorted(output.iterdir()):
            if path.is_file():
                artifacts[path.name] = Artifact(
                    path.resolve(), _sha256_file(path), path.stat().st_size
                ).as_dict()
    if state == 'completed_immutable':
        missing = sorted(set(FINAL_ARTIFACT_NAMES) - set(artifacts))
        if missing:
            raise FinalTestV4Error(
                f'cannot complete final Test; output artifacts missing: {missing}'
            )
    completion = {
        'schema_version': ATTEMPT_SCHEMA_VERSION,
        'state': state,
        'attempt_key': reserved.bundle.attempt_key,
        'completed_at_utc': datetime.now(timezone.utc).isoformat(),
        'reservation': {
            'path': str(reserved.reservation_path),
            'sha256': reserved.reservation_sha256,
        },
        'contract_sha256': reserved.bundle.contract_sha256,
        'evaluation_protocol_lock_sha256': reserved.bundle.artifacts[
            'evaluation_protocol_lock'
        ].sha256,
        'protocol_frozen_sha256': reserved.bundle.evaluation_protocol_lock[
            'protocol_frozen_sha256'
        ],
        'implementation_aggregate_sha256': reserved.bundle.evaluation_protocol_lock[
            'implementation_aggregate_sha256'
        ],
        'checkpoint_sha256': reserved.bundle.artifacts['checkpoint'].sha256,
        'dataset_fingerprint_sha256': reserved.bundle.contract['dataset'][
            'dataset_fingerprint_sha256'
        ],
        'output': str(output),
        'artifacts': artifacts,
        'failure': dict(failure or {}),
        'test_used_for_training': False,
        'test_used_for_checkpoint_selection': False,
        'calibration_refit_performed': False,
        'threshold_selection_performed': False,
        'automatic_runtime_switch': False,
        'plansys_updates_enabled': False,
        'actuation_enabled': False,
    }
    _write_json_exclusive(reserved.completion_path, completion, read_only=True)
    if output.is_dir():
        for path in output.iterdir():
            if path.is_file():
                path.chmod(0o444)
        output.chmod(0o555)
    return {
        'path': str(reserved.completion_path),
        'sha256': _sha256_file(reserved.completion_path),
        'state': state,
        'attempt_key': reserved.bundle.attempt_key,
    }


def _evaluate_reserved_attempt(
    reserved: ReservedAttempt,
) -> dict[str, Any]:
    _assert_reservation_intact(reserved)
    output = reserved.bundle.output_path
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        output.mkdir()
    except FileExistsError as exc:
        raise FinalTestV4Error(f'refusing existing final Test output: {output}') from exc
    _write_json_exclusive(output / 'attempt_started.json', {
        'schema_version': ATTEMPT_SCHEMA_VERSION,
        'state': 'inference_started_after_global_reservation',
        'attempt_key': reserved.bundle.attempt_key,
        'reservation': {
            'path': str(reserved.reservation_path),
            'sha256': reserved.reservation_sha256,
        },
        'rows_opened_before_reservation': False,
        'labels_opened_before_reservation': False,
        'images_opened_before_reservation': False,
        'started_at_utc': datetime.now(timezone.utc).isoformat(),
    })
    _write_json_exclusive(
        output / 'evaluation_contract.json', reserved.bundle.contract
    )
    _write_json_exclusive(
        output / 'evaluation_protocol_lock.json',
        reserved.bundle.evaluation_protocol_lock,
    )
    _write_json_exclusive(output / 'dataset_config.json', reserved.bundle.dataset_config)
    _write_json_exclusive(output / 'preregistration.json', reserved.bundle.preregistration)
    _write_json_exclusive(output / 'plan_lock.json', reserved.bundle.plan_lock)

    records, input_artifacts, image_hashes, pair_hashes = _load_reserved_records(
        reserved
    )
    support = _compute_support_summary(records)
    coverage_contract = _validate_coverage_contract(
        reserved.bundle.contract['coverage_contract']
    )
    coverage_audit = _validate_support_coverage(support, coverage_contract)
    if coverage_audit['passed'] is not True:
        raise FinalTestV4Error(
            f'final Test coverage contract failed: {coverage_audit["failed_checks"]}'
        )
    finalization = _verify_finalization_after_reservation(
        reserved,
        records,
        input_artifacts['rows'],
        input_artifacts['labels'],
        image_hashes,
        pair_hashes,
        support,
    )
    disjoint = _verify_historical_disjointness(
        reserved,
        image_hashes,
        input_artifacts['rows'],
        input_artifacts['labels'],
    )
    finalization_disjoint = _required_mapping(
        finalization.get('disjoint_audit'), 'finalization.disjoint_audit'
    )
    if finalization_disjoint.get('passed') is not True:
        raise FinalTestV4Error('dataset finalization disjoint audit did not pass')
    disjoint_artifact = _fingerprint(
        finalization_disjoint.get('path', ''),
        finalization_disjoint.get('sha256'),
        'dataset-generation disjoint audit',
    )
    generation_disjoint = _read_json_object(
        disjoint_artifact.path, 'dataset-generation disjoint audit'
    )
    generation_overlaps = _required_mapping(
        generation_disjoint.get('overlap_counts'),
        'dataset-generation disjoint overlap counts',
    )
    if (
        generation_disjoint.get('schema_version')
        != 'room315.visual_v4.final_test_disjoint_audit.v1'
        or generation_disjoint.get('dataset_role') != DATASET_ROLE
        or generation_disjoint.get('passed') is not True
        or generation_disjoint.get('historical_test_accessed') is not False
        or any(int(value) != 0 for value in generation_overlaps.values())
        or dict(finalization_disjoint.get('overlap_counts') or {})
        != dict(generation_overlaps)
    ):
        raise FinalTestV4Error(
            'dataset-generation disjoint audit is incomplete or reports overlap'
        )
    disjoint['dataset_generation_audit'] = disjoint_artifact.as_dict()
    disjoint['dataset_generation_overlap_counts'] = dict(generation_overlaps)

    _write_json_exclusive(output / 'final_test_finalization.json', finalization)
    input_fingerprint = {
        'schema_version': 'room315.visual_v4.final_test_input_fingerprint.v1',
        'dataset_role': DATASET_ROLE,
        'sample_count': len(records),
        'image_count': len(image_hashes),
        'rows': input_artifacts['rows'].as_dict(),
        'labels': input_artifacts['labels'].as_dict(),
        'dataset_fingerprint_sha256': reserved.bundle.contract['dataset'][
            'dataset_fingerprint_sha256'
        ],
        'finalization': reserved.bundle.artifacts['finalization'].as_dict(),
        'opened_only_after_global_reservation': True,
    }
    image_manifest = {
        'schema_version': 'room315.visual_v4.final_test_image_manifest.v1',
        'image_count': len(image_hashes),
        'pair_count': len(pair_hashes),
        'unique_image_hash_count': len(set(image_hashes.values())),
        'unique_pair_hash_count': len(set(pair_hashes.values())),
        'image_manifest_sha256': finalization['image_manifest_sha256'],
        'matches_frozen_finalization': True,
        # Do not duplicate individual image paths or Test rows in results.
        'individual_hash_mapping_in_frozen_finalization': True,
    }
    _write_json_exclusive(output / 'final_test_input_fingerprint.json', input_fingerprint)
    _write_json_exclusive(output / 'final_test_image_pair_manifest.json', image_manifest)
    _write_json_exclusive(output / 'final_test_coverage_audit.json', coverage_audit)
    _write_json_exclusive(output / 'final_test_disjoint_audit.json', disjoint)

    (
        torch_module,
        training_api,
        device,
        model,
        checkpoint,
        topology_lengths,
        model_audit,
    ) = _load_frozen_model(
        reserved,
        records,
    )
    epoch = int(checkpoint.get('epoch', 0))
    metrics = trainer.evaluate_model(
        model,
        records,
        reserved.bundle.effective_config,
        torch_module=torch_module,
        training_api=training_api,
        device=device,
        class_weights=checkpoint['class_weights_by_side'],
        topology_lengths=topology_lengths,
        epoch=epoch,
        require_full_side_segment_support=True,
    )
    metrics.pop('selection_key', None)
    metrics['loss_aggregation'] = 'single_reduction_over_all_visible_final_test_slots'
    metrics['selection_role'] = 'none'
    metrics['used_for_checkpoint_selection'] = False
    metrics['planning_selection_score_role'] = 'diagnostic_only_on_final_test'
    counterfactuals = trainer.evaluate_camera_counterfactuals(
        model,
        records,
        reserved.bundle.effective_config,
        torch_module=torch_module,
        training_api=training_api,
        device=device,
        topology_lengths=topology_lengths,
        epoch=epoch,
    )
    tensors = _collect_confidence_tensors(
        model,
        records,
        reserved.bundle.effective_config,
        torch_module=torch_module,
        device=device,
        epoch=epoch,
    )
    calibration = _fixed_final_test_calibration(
        tensors, reserved.bundle, torch_module=torch_module
    )
    runtime_thresholds = _runtime_threshold_report(
        tensors, reserved.bundle, torch_module=torch_module
    )
    try:
        import room_315_visual_acceptance_v4 as acceptance_api
    except Exception as exc:
        raise FinalTestV4Error(f'cannot import V4 acceptance API: {exc}') from exc
    approved_baseline = _required_mapping(
        reserved.bundle.training_report.get('approved_v3_validation_baseline'),
        'frozen approved V3 validation baseline',
    )
    base_acceptance = acceptance_api.evaluate_visual_acceptance_v4(
        metrics,
        reserved.bundle.effective_config['pilot_acceptance_gates'],
        counterfactual_report=counterfactuals,
        approved_v3_loaded_accuracy=float(approved_baseline['loaded_accuracy']),
        required_scene_presence_densities=SCENE_PRESENCE_DENSITIES,
    )
    acceptance = _combine_final_test_acceptance(
        base_acceptance,
        runtime_thresholds,
        coverage_audit,
        coverage_contract,
    )
    metrics['acceptance_gates_evaluated'] = True
    metrics['pending_evaluations'] = [
        gate_id
        for gate_id, item in base_acceptance['per_gate'].items()
        if item['status'] == 'pending'
    ]
    _write_json_exclusive(output / 'final_test_metrics.json', metrics)
    _write_json_exclusive(
        output / 'final_test_camera_counterfactuals.json', counterfactuals
    )
    _write_json_exclusive(
        output / 'final_test_segment_calibration.json', calibration
    )
    _write_json_exclusive(
        output / 'final_test_runtime_thresholds.json', runtime_thresholds
    )
    _write_json_exclusive(output / 'final_test_acceptance.json', acceptance)

    report = {
        'schema_version': REPORT_SCHEMA_VERSION,
        'status': 'completed',
        'mode': 'one_shot_post_freeze_final_test_evaluation',
        'dataset_role': DATASET_ROLE,
        'attempt': {
            'attempt_key': reserved.bundle.attempt_key,
            'one_shot': True,
            'reservation_path': str(reserved.reservation_path),
            'reservation_sha256': reserved.reservation_sha256,
            'completion_path': str(reserved.completion_path),
            'completion_ledger_required_for_trust': True,
            'evaluation_protocol_lock_sha256': reserved.bundle.artifacts[
                'evaluation_protocol_lock'
            ].sha256,
            'protocol_frozen_sha256': reserved.bundle.evaluation_protocol_lock[
                'protocol_frozen_sha256'
            ],
            'implementation_aggregate_sha256': (
                reserved.bundle.evaluation_protocol_lock[
                    'implementation_aggregate_sha256'
                ]
            ),
        },
        'checkpoint': reserved.bundle.artifacts['checkpoint'].as_dict(),
        'dataset': input_fingerprint,
        'model_audit': model_audit,
        'metrics': metrics,
        'camera_counterfactuals': counterfactuals,
        'segment_calibration': calibration,
        'runtime_thresholds': runtime_thresholds,
        'acceptance': acceptance,
        'acceptance_status': acceptance['status'],
        'promotion_status': 'human_review_only_no_automatic_transition',
        'test_loaded': True,
        'test_used_for_training': False,
        'test_used_for_checkpoint_selection': False,
        'test_used_for_calibration': False,
        'test_used_for_threshold_selection': False,
        'training_performed': False,
        'checkpoint_selection_performed': False,
        'calibration_refit_performed': False,
        'threshold_selection_performed': False,
        'canary_opened': False,
        'historical_test_opened': False,
        'automatic_runtime_switch': False,
        'plansys_updates_enabled': False,
        'actuation_enabled': False,
    }
    _write_json_exclusive(output / 'final_report.json', report)
    return report


def evaluate_final_test_v4(
    contract_path: Path | str,
    contract_sha256: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Reserve and consume exactly one final-Test attempt."""

    reserved = reserve_final_test_attempt(
        contract_path,
        contract_sha256,
    )
    try:
        report = _evaluate_reserved_attempt(reserved)
        completion = _complete_attempt(reserved, state='completed_immutable')
        return report, completion
    except BaseException as exc:
        # A reserved attempt is consumed even when evaluation fails.  Record a
        # compact immutable failure rather than silently allowing a retry.
        try:
            _complete_attempt(
                reserved,
                state='failed_immutable',
                failure={
                    'exception_type': type(exc).__name__,
                    'message': str(exc),
                },
            )
        except Exception:
            # Preserve the original error.  The immutable reservation itself
            # still blocks a retry if failure-ledger creation is interrupted.
            pass
        raise


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Evaluate one newly generated, sealed Room 315 visual V4 final '
            'Test after reserving its global immutable attempt.'
        )
    )
    subparsers = parser.add_subparsers(dest='mode', required=True)
    evaluate = subparsers.add_parser('evaluate')
    evaluate.add_argument('--contract', type=Path, required=True)
    evaluate.add_argument('--contract-sha256', required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.mode != 'evaluate':
        raise FinalTestV4Error(f'unsupported mode: {args.mode}')
    report, completion = evaluate_final_test_v4(
        args.contract,
        args.contract_sha256,
    )
    print(json.dumps({
        'status': report['status'],
        'acceptance_status': report['acceptance_status'],
        'attempt_key': completion['attempt_key'],
        'completion_ledger': completion,
    }, indent=2, sort_keys=True))
    return 0 if report['acceptance_status'] == 'passed' else 2


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except FinalTestV4Error as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        raise SystemExit(2)
