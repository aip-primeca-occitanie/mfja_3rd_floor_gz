#!/usr/bin/env python3
"""Project the sealed V2 coverage extension into evaluator-canonical controls.

This is a compatibility layer, not a new dataset design.  It copies the exact
1,040-row V2 manifest (whose first 1,024 rows are the byte-identical V1
prefix), keeps the V2 per-row provenance, and emits the canonical V1 control
schemas required by the already-frozen evaluator.  No model is imported and
there is deliberately no evaluate command.

When ``finalize`` is invoked on a planned compatibility root with no dataset,
the wrapper makes an isolated byte copy of the already sealed V2 capture and
runs the original finalizer under the canonical schemas.  It never recaptures
a scene and never modifies the V1 or V2 source artifacts.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import re
import shutil
import sys
import tempfile
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator, Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_v4_final_test as canonical
import room_315_visual_v4_final_test_coverage_extension as coverage_v2


CONFIG_SCHEMA = canonical.CONFIG_SCHEMA
PREREGISTRATION_SCHEMA = canonical.PREREGISTRATION_SCHEMA
PLAN_LOCK_SCHEMA = canonical.PLAN_LOCK_SCHEMA
PLAN_SUMMARY_SCHEMA = canonical.PLAN_SUMMARY_SCHEMA
FINALIZATION_SCHEMA = canonical.FINALIZATION_SCHEMA
DISJOINT_AUDIT_SCHEMA = canonical.DISJOINT_AUDIT_SCHEMA
SEAL_SCHEMA = canonical.SEAL_SCHEMA
GENERATOR_VERSION = 'room315.visual_v4.final_test_coverage_compat.v1'

SEED = coverage_v2.SEED
SCENARIO_COUNT = coverage_v2.SCENARIO_COUNT
LATTICE_COUNT = coverage_v2.LATTICE_COUNT
STRESS_COUNT = coverage_v2.STRESS_COUNT
V1_SCENARIO_COUNT = coverage_v2.V1_SCENARIO_COUNT
V2_ROOT = Path(
    '/home/tiago/room315_visual_v4_final_test_coverage_v2_seed3152026081101'
)
V2_MANIFEST_SHA256 = (
    '590dd942f60cdeadb40238b2db95e54f8eb2022005985fed13ca4f76b28f1d7f'
)
ROOT_NAME_PATTERN = re.compile(
    r'^room315_visual_v4_final_test_seed[0-9]{10,}(?:_[a-z0-9]+)*$'
)

DEFAULT_CONFIG = (
    SCRIPT_DIR.parent
    / 'config'
    / 'room_315_visual_state'
    / 'visual_state_final_test_v4_coverage_compat_v1.json'
)
DEFAULT_ROOT = Path(
    '/home/tiago/room315_visual_v4_final_test_seed3152026081101_coveragecompat'
)

V2_ARTIFACT_PINS = {
    'source_wrapper': (
        SCRIPT_DIR / 'room_315_visual_v4_final_test_coverage_extension.py',
        'b2d00e036f6bf3a46af3acd2fe295115ed919d77c1249828ab9469dc4f1eec4b',
    ),
    'source_config': (
        SCRIPT_DIR.parent
        / 'config'
        / 'room_315_visual_state'
        / 'visual_state_final_test_v4_coverage_extension.json',
        '42fd32c7ee1fa7cbb02dd28fd38a3411d4a8602cda903364754fcdc9e8fb29ec',
    ),
    'scenario_manifest': (
        V2_ROOT / 'scenario_manifest.jsonl',
        V2_MANIFEST_SHA256,
    ),
    'scenario_summary': (
        V2_ROOT / 'scenario_summary.json',
        '17c45911c2d69f80e37d66fcebd273c945a2298a9fdbf63abe7be978a1432b4a',
    ),
    'preregistration': (
        V2_ROOT / 'preregistration.json',
        '3cb3d59271f56569ef760aa705f8229040c0bef844e460c6a8cc7b7e24515b0f',
    ),
    'plan_lock': (
        V2_ROOT / 'plan_lock.json',
        'ef279fdbacf80c7e756c6d307d557a4a207ac3466e7863e9b5e7a7cff8e2d0fd',
    ),
    'finalization': (
        V2_ROOT / 'finalized' / 'final_test_finalization.json',
        'b9d276daf77bd061cdf3536f3234d68d8c7f1e89eb8d22749e344dc7d5e347b3',
    ),
    'disjoint_audit': (
        V2_ROOT / 'finalized' / 'final_test_disjoint_audit.json',
        'b8608f08bae3984638f7352f570262be5bb47cdda1e5c974233486fd34b38c13',
    ),
    'seal': (
        V2_ROOT / 'finalized' / 'final_test_seal.json',
        'cc2964bf28e523c19c57d39e302ee70ba46e07685669ad5a3557c2c45be6a43c',
    ),
    'evaluation_protocol_lock': (
        Path(
            '/home/tiago/room315_visual_v4_final_test_'
            'evaluation_protocol_lock_coverage_v2_seed3152026081101.json'
        ),
        'f0fe2738d5a4caabf0300d59e8c892569a9e186dc8ff424cef7aaac2903f152e',
    ),
}


_BASE_LOAD_CONFIG = canonical._load_config
_BASE_CREATE_PLAN = canonical.create_plan
_BASE_VERIFY_PLAN = canonical.verify_plan
_BASE_CAPTURE_STATUS = canonical.capture_status
_BASE_FINALIZE_CAPTURE = canonical.finalize_capture
_BASE_VERIFY_SEAL = canonical.verify_seal


def _rows_bytes(rows: list[dict[str, Any]]) -> bytes:
    return ''.join(
        canonical.canonical_json(row) + '\n' for row in rows
    ).encode('utf-8')


def _v2_rows() -> list[dict[str, Any]]:
    manifest, expected = V2_ARTIFACT_PINS['scenario_manifest']
    if not manifest.is_file() or canonical.sha256_file(manifest) != expected:
        raise canonical.FinalTestError('the sealed V2 manifest pin changed')
    rows = canonical.read_jsonl(manifest)
    if len(rows) != SCENARIO_COUNT or _rows_bytes(rows) != manifest.read_bytes():
        raise canonical.FinalTestError('the sealed V2 manifest is not canonical')
    v1_manifest = coverage_v2.V1_CONTROL_PINS['scenario_manifest'][0]
    if _rows_bytes(rows[:V1_SCENARIO_COUNT]) != v1_manifest.read_bytes():
        raise canonical.FinalTestError('the V2 manifest lost its byte-identical V1 prefix')
    return rows


def _validate_v2_source(config: Mapping[str, Any]) -> None:
    declared = config.get('coverage_v2_compatibility_source') or {}
    artifacts = declared.get('artifacts') or {}
    for name, (expected_path, expected_sha256) in V2_ARTIFACT_PINS.items():
        item = artifacts.get(name) or {}
        if (
            Path(str(item.get('path') or '')).expanduser().resolve()
            != expected_path.resolve()
            or item.get('sha256') != expected_sha256
            or not expected_path.is_file()
            or canonical.sha256_file(expected_path) != expected_sha256
        ):
            raise canonical.FinalTestError(f'sealed V2 compatibility pin mismatch: {name}')

    summary = canonical.read_json(V2_ARTIFACT_PINS['scenario_summary'][0])
    preregistration = canonical.read_json(V2_ARTIFACT_PINS['preregistration'][0])
    plan_lock = canonical.read_json(V2_ARTIFACT_PINS['plan_lock'][0])
    finalization = canonical.read_json(V2_ARTIFACT_PINS['finalization'][0])
    seal = canonical.read_json(V2_ARTIFACT_PINS['seal'][0])
    protocol = canonical.read_json(V2_ARTIFACT_PINS['evaluation_protocol_lock'][0])
    for name, value in (
        ('scenario summary', summary),
        ('preregistration', preregistration),
        ('plan lock', plan_lock),
        ('finalization', finalization),
        ('seal', seal),
        ('evaluation protocol lock', protocol),
    ):
        if (
            value.get('inference_status') != 'not_run'
            or int(value.get('inference_count', -1)) != 0
        ):
            raise canonical.FinalTestError(f'{name} no longer proves zero inference')
    if any(
        int(value.get('scenario_count', SCENARIO_COUNT)) != SCENARIO_COUNT
        for value in (summary, plan_lock, finalization, seal)
    ):
        raise canonical.FinalTestError('sealed V2 control scenario count changed')
    if finalization.get('passed') is not True or seal.get('passed') is not True:
        raise canonical.FinalTestError('the V2 source is not a passed sealed capture')
    if any(protocol.get(name) is not False for name in (
        'test_rows_opened', 'test_labels_opened', 'test_images_opened'
    )):
        raise canonical.FinalTestError('V2 protocol records Test-data exposure')
    if any(path.exists() for path in coverage_v2.FORBIDDEN_PRIOR_INFERENCE_ARTIFACTS):
        raise canonical.FinalTestError('an official V4 inference attempt already exists')

    if (
        declared.get('projection_type')
        != 'schema_only_pre_inference_compatibility_projection'
        or declared.get('manifest_copy_policy') != 'byte_identical'
        or declared.get('source_inference_status') != 'not_run'
        or int(declared.get('source_inference_count', -1)) != 0
        or declared.get('source_rows_labels_images_opened_for_design') is not False
        or declared.get('preserve_v2_row_provenance') is not True
    ):
        raise canonical.FinalTestError('V2 compatibility provenance is incomplete')
    _v2_rows()


def _load_config_in_context(path: Path) -> dict[str, Any]:
    config = _BASE_LOAD_CONFIG(path)
    root = Path(str(config.get('output_root') or '')).expanduser().resolve()
    if not ROOT_NAME_PATTERN.fullmatch(root.name):
        raise canonical.FinalTestError('compatibility root is not evaluator-canonical')
    if root in {
        V2_ROOT.resolve(),
        coverage_v2.V1_CONTROL_PINS['scenario_manifest'][0].parent.resolve(),
    }:
        raise canonical.FinalTestError('compatibility root must be a new root')
    composition = config.get('composition') or {}
    if (
        int(config.get('scenario_count', -1)) != SCENARIO_COUNT
        or int(composition.get('lattice_scenarios', -1)) != LATTICE_COUNT
        or int(composition.get('stress_scenarios', -1)) != STRESS_COUNT
        or int(composition.get('v2_byte_identical_manifest_scenarios', -1))
        != SCENARIO_COUNT
    ):
        raise canonical.FinalTestError('compatibility configuration count changed')
    schemas = composition.get('evaluator_control_schemas') or {}
    expected_schemas = {
        'config': CONFIG_SCHEMA,
        'preregistration': PREREGISTRATION_SCHEMA,
        'plan_summary': PLAN_SUMMARY_SCHEMA,
        'plan_lock': PLAN_LOCK_SCHEMA,
        'finalization': FINALIZATION_SCHEMA,
        'disjoint_audit': DISJOINT_AUDIT_SCHEMA,
        'seal': SEAL_SCHEMA,
    }
    if schemas != expected_schemas:
        raise canonical.FinalTestError('canonical evaluator schema declaration changed')
    _validate_v2_source(config)
    return config


def load_config(path: Path = DEFAULT_CONFIG) -> dict[str, Any]:
    with _compatibility_contract():
        return _load_config_in_context(path)


def _materialize_plan_in_context(
    config: Mapping[str, Any], references: Mapping[str, Any]
) -> list[dict[str, Any]]:
    _validate_v2_source(config)
    rows = _v2_rows()
    support = canonical.plan_support_summary(rows)
    uniqueness = canonical._uniqueness_summary(rows)
    disjoint = canonical._static_disjoint_audit(rows, references)
    coverage_v2._assert_plan_contract_in_context(
        rows, support, uniqueness, disjoint
    )
    return rows


def materialize_plan(
    config: Mapping[str, Any], references: Mapping[str, Any]
) -> list[dict[str, Any]]:
    with _compatibility_contract():
        return _materialize_plan_in_context(config, references)


def _assert_plan_contract_in_context(
    rows: list[dict[str, Any]],
    support: Mapping[str, Any],
    uniqueness: Mapping[str, Any],
    static_disjoint: Mapping[str, Any],
) -> None:
    coverage_v2._assert_plan_contract_in_context(
        rows, support, uniqueness, static_disjoint
    )
    if _rows_bytes(rows) != V2_ARTIFACT_PINS['scenario_manifest'][0].read_bytes():
        raise canonical.FinalTestError(
            'compatibility manifest is not byte-identical to sealed V2'
        )


@contextmanager
def _compatibility_contract() -> Iterator[None]:
    replacements = {
        'CONFIG_SCHEMA': CONFIG_SCHEMA,
        'PREREGISTRATION_SCHEMA': PREREGISTRATION_SCHEMA,
        'PLAN_LOCK_SCHEMA': PLAN_LOCK_SCHEMA,
        'PLAN_SUMMARY_SCHEMA': PLAN_SUMMARY_SCHEMA,
        'FINALIZATION_SCHEMA': FINALIZATION_SCHEMA,
        'DISJOINT_AUDIT_SCHEMA': DISJOINT_AUDIT_SCHEMA,
        'SEAL_SCHEMA': SEAL_SCHEMA,
        'GENERATOR_VERSION': GENERATOR_VERSION,
        'SCENARIO_COUNT': SCENARIO_COUNT,
        'LATTICE_COUNT': LATTICE_COUNT,
        'STRESS_COUNT': STRESS_COUNT,
        '_load_config': _load_config_in_context,
        'materialize_plan': _materialize_plan_in_context,
        '_assert_plan_contract': _assert_plan_contract_in_context,
    }
    previous = {name: getattr(canonical, name) for name in replacements}
    try:
        for name, value in replacements.items():
            setattr(canonical, name, value)
        yield
    finally:
        for name, value in previous.items():
            setattr(canonical, name, value)


def create_plan(config_path: Path, *, output_root: Path | None = None) -> dict[str, Any]:
    with _compatibility_contract():
        return _BASE_CREATE_PLAN(config_path, output_root=output_root)


def verify_plan(root: Path, config_path: Path, *, regenerate: bool = True) -> dict[str, Any]:
    with _compatibility_contract():
        result = _BASE_VERIFY_PLAN(root, config_path, regenerate=regenerate)
    if result['manifest_sha256'] != V2_MANIFEST_SHA256:
        raise canonical.FinalTestError('verified compatibility manifest hash changed')
    return result


def capture_status(root: Path, config_path: Path) -> dict[str, Any]:
    with _compatibility_contract():
        result = _BASE_CAPTURE_STATUS(root, config_path)
    result.update({
        'schema_version': 'room315.visual_v4.final_test_capture_status.v1',
        'capture_mode': 'sealed_v2_byte_copy_only_no_recapture',
        'sealed_v2_source_available': True,
        'sealed_v2_source_manifest_sha256': V2_MANIFEST_SHA256,
    })
    return result


def _stage_sealed_v2_capture(root: Path) -> bool:
    root = root.expanduser().resolve()
    target = root / 'dataset'
    if target.exists():
        return False
    source = V2_ROOT / 'dataset'
    if not source.is_dir():
        raise canonical.FinalTestError('sealed V2 dataset source is missing')
    temporary_parent = Path(tempfile.mkdtemp(prefix='.compat-dataset.', dir=root))
    staged = temporary_parent / 'dataset'
    try:
        shutil.copytree(source, staged, copy_function=shutil.copy2)
        os.replace(staged, target)
    except BaseException:
        if temporary_parent.exists():
            shutil.rmtree(temporary_parent)
        raise
    if temporary_parent.exists():
        temporary_parent.rmdir()
    return True


def _assert_finalization_preserves_v2_capture(
    finalization: Mapping[str, Any],
) -> None:
    source = canonical.read_json(V2_ARTIFACT_PINS['finalization'][0])
    checks = {
        'scenario_count': finalization.get('scenario_count') == SCENARIO_COUNT,
        'image_count': finalization.get('image_count') == SCENARIO_COUNT * 2,
        'rows_sha256': finalization.get('rows', {}).get('sha256')
        == source.get('rows', {}).get('sha256'),
        'labels_sha256': finalization.get('labels', {}).get('sha256')
        == source.get('labels', {}).get('sha256'),
        'images': finalization.get('images') == source.get('images'),
        'manifest': finalization.get('scenario_manifest', {}).get('sha256')
        == V2_MANIFEST_SHA256,
        'schema': finalization.get('schema_version') == FINALIZATION_SCHEMA,
        'not_run': finalization.get('inference_status') == 'not_run'
        and int(finalization.get('inference_count', -1)) == 0,
    }
    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise canonical.FinalTestError(
            f'compatibility finalization changed sealed V2 capture: {failed}'
        )


def finalize_capture(root: Path, config_path: Path) -> dict[str, Any]:
    root = root.expanduser().resolve()
    verify_plan(root, config_path, regenerate=True)
    _stage_sealed_v2_capture(root)
    with _compatibility_contract():
        result = _BASE_FINALIZE_CAPTURE(root, config_path)
    _assert_finalization_preserves_v2_capture(result)
    return result


def verify_seal(root: Path, config_path: Path) -> dict[str, Any]:
    with _compatibility_contract():
        result = _BASE_VERIFY_SEAL(root, config_path)
    finalization = canonical.read_json(
        root.expanduser().resolve() / 'finalized' / 'final_test_finalization.json'
    )
    _assert_finalization_preserves_v2_capture(finalization)
    return result


def build_control_fixture_finalization(
    root: Path,
    config_path: Path,
) -> dict[str, Any]:
    """Build an unsealed control-only fixture; never opens rows, labels, or images."""
    root = root.expanduser().resolve()
    config_path = config_path.expanduser().resolve()
    source = canonical.read_json(V2_ARTIFACT_PINS['finalization'][0])
    value = copy.deepcopy(source)
    value.update({
        'schema_version': FINALIZATION_SCHEMA,
        'configuration': {
            'path': str(config_path),
            'sha256': canonical.sha256_file(config_path),
        },
        'compatibility_provenance_v2': {
            'projection_type': 'control_only_schema_compatibility_fixture',
            'source_root': str(V2_ROOT),
            'source_finalization_sha256': V2_ARTIFACT_PINS['finalization'][1],
            'source_manifest_sha256': V2_MANIFEST_SHA256,
            'source_inference_status': 'not_run',
            'source_inference_count': 0,
            'rows_labels_images_opened': False,
        },
    })
    value['rows']['path'] = str(root / 'finalized' / 'final_test.jsonl')
    value['labels']['path'] = str(
        root / 'finalized' / 'final_test_visual_labels.jsonl'
    )
    value['scenario_manifest'] = {
        'path': str(root / 'scenario_manifest.jsonl'),
        'sha256': canonical.sha256_file(root / 'scenario_manifest.jsonl'),
    }
    value['preregistration'] = {
        'path': str(root / 'preregistration.json'),
        'sha256': canonical.sha256_file(root / 'preregistration.json'),
    }
    value['plan_lock'] = {
        'path': str(root / 'plan_lock.json'),
        'sha256': canonical.sha256_file(root / 'plan_lock.json'),
    }
    value['disjoint_audit']['path'] = str(
        root / 'finalized' / 'final_test_disjoint_audit.json'
    )
    value['plan_verification'] = {
        'manifest_sha256': V2_MANIFEST_SHA256,
        'passed': True,
    }
    return value


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('plan')
    verify = commands.add_parser('verify-plan')
    verify.add_argument('--skip-deterministic-regeneration', action='store_true')
    commands.add_parser('status')
    commands.add_parser('finalize')
    commands.add_parser('verify-seal')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == 'plan':
        result: Any = create_plan(args.config, output_root=args.root)
    elif args.command == 'verify-plan':
        result = verify_plan(
            args.root,
            args.config,
            regenerate=not args.skip_deterministic_regeneration,
        )
    elif args.command == 'status':
        result = capture_status(args.root, args.config)
    elif args.command == 'finalize':
        result = finalize_capture(args.root, args.config)
    else:
        result = verify_seal(args.root, args.config)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (canonical.FinalTestError, OSError, canonical.VisualV3Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
