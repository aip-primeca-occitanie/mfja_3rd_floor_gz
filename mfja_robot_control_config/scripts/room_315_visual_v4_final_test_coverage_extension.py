#!/usr/bin/env python3
"""Prepare the pre-inference Room 315 V4 final-Test coverage extension.

The immutable V1 plan was captured and sealed, but it was never inferred.  A
support-only audit of its preregistered manifest found that three required
identity zones had aggregate planned support below eight.  Before reserving an
inference attempt, this wrapper preserves the 1,024 V1 manifest rows byte for
byte and appends 16 preregistered stress scenes.  It never opens Test images,
Test rows, Test labels, predictions, or model metrics.

This module deliberately wraps ``room_315_visual_v4_final_test.py`` instead of
changing it: the V1 source, configuration, plan, and evaluator remain frozen.
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
from collections import Counter
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator, Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_v4_final_test as v1


CONFIG_SCHEMA = 'room315.visual_v4.final_test_coverage_extension_config.v2'
PREREGISTRATION_SCHEMA = (
    'room315.visual_v4.final_test_coverage_extension_preregistration.v2'
)
PLAN_LOCK_SCHEMA = 'room315.visual_v4.final_test_coverage_extension_plan_lock.v2'
PLAN_SUMMARY_SCHEMA = (
    'room315.visual_v4.final_test_coverage_extension_plan_summary.v2'
)
FINALIZATION_SCHEMA = (
    'room315.visual_v4.final_test_coverage_extension_finalization.v2'
)
DISJOINT_AUDIT_SCHEMA = (
    'room315.visual_v4.final_test_coverage_extension_disjoint_audit.v2'
)
SEAL_SCHEMA = 'room315.visual_v4.final_test_coverage_extension_seal.v2'
GENERATOR_VERSION = 'room315.visual_v4.final_test_coverage_extension.v2'

SEED = v1.SEED
V1_SCENARIO_COUNT = 1024
SCENARIO_COUNT = 1040
LATTICE_COUNT = 1008
V1_STRESS_COUNT = 16
STRESS_COUNT = 32
ADDED_STRESS_COUNT = 16
MINIMUM_AGGREGATE_IDENTITY_ZONE_SUPPORT = 8
V1_MANIFEST_SHA256 = (
    'c2147e28d3116798648acfbc3da1488c615e5131c3835deef38f8f552239a282'
)

DEFAULT_CONFIG = (
    SCRIPT_DIR.parent
    / 'config'
    / 'room_315_vla'
    / 'visual_state_final_test_v4_coverage_extension.json'
)
DEFAULT_ROOT = Path(
    '/home/tiago/room315_visual_v4_final_test_coverage_v2_seed3152026081101'
)

V1_CONTROL_PINS = {
    'scenario_manifest': (
        Path('/home/tiago/room315_visual_v4_final_test_seed3152026081101/scenario_manifest.jsonl'),
        V1_MANIFEST_SHA256,
    ),
    'scenario_summary': (
        Path('/home/tiago/room315_visual_v4_final_test_seed3152026081101/scenario_summary.json'),
        'c83dba51ad37d812bcf548c115bc7fa289925edac1841740296fefee0bc63478',
    ),
    'preregistration': (
        Path('/home/tiago/room315_visual_v4_final_test_seed3152026081101/preregistration.json'),
        'a0e8ec9ad97a13b82ffa73a03c1ccd20915f82fae449ed2a913e12c290b8fd49',
    ),
    'plan_lock': (
        Path('/home/tiago/room315_visual_v4_final_test_seed3152026081101/plan_lock.json'),
        '39e12180961a96046e0d3f3cc5698a8c268069891778731ca8db831a150d221e',
    ),
    'finalization': (
        Path('/home/tiago/room315_visual_v4_final_test_seed3152026081101/finalized/final_test_finalization.json'),
        'c4c655b60cf68d505b3f654486819354a9bc31a69972531e3ee9ece9221aa2c3',
    ),
    'seal': (
        Path('/home/tiago/room315_visual_v4_final_test_seed3152026081101/finalized/final_test_seal.json'),
        '50fdf28ec3220f0d701febad610a59ddea5ab6f6f4059b6d5921af96c1af2ff8',
    ),
    'evaluation_protocol_lock': (
        Path('/home/tiago/room315_visual_v4_final_test_evaluation_protocol_lock_seed3152026081101.json'),
        '71310756cdb624387267ef47f2efd03d1b38a126ad84f584e8ca2a13dd72f7ec',
    ),
    'unexecuted_evaluation_contract': (
        Path('/home/tiago/room315_visual_v4_final_test_contract_seed3152026081101.json'),
        'c189830e6154c4cfa772ab0b6a372420e92dc6ffbd350e3a2ed325c40c06ca50',
    ),
}
FORBIDDEN_PRIOR_INFERENCE_ARTIFACTS = (
    Path('/home/tiago/room315_visual_v4_final_test_attempt_ledger_v1'),
    Path('/home/tiago/room315_visual_v4_final_test_outputs'),
)

ADDED_RELATIONS = (
    v1.NO_RELATION,
    v1.NO_RELATION,
    'nonblocker_adjacent_branch',
    'nonblocker_adjacent_branch',
    'nonblocker_behind_same_segment',
    'nonblocker_behind_same_segment',
    'blocker_intermediate_segment',
    'blocker_intermediate_segment',
    'nonblocker_adjacent_branch',
    'nonblocker_adjacent_branch',
    'nonblocker_behind_same_segment',
    'nonblocker_behind_same_segment',
    'nonblocker_adjacent_branch',
    'nonblocker_adjacent_branch',
    'nonblocker_behind_same_segment',
    'nonblocker_behind_same_segment',
)
ADDED_RELATION_COUNTS = {
    'blocker_intermediate_segment': 2,
    'no_relation_observation': 2,
    'nonblocker_adjacent_branch': 6,
    'nonblocker_behind_same_segment': 6,
}
EXPECTED_RELATION_COUNTS = {
    'blocker_ahead_same_segment': 4,
    'blocker_intermediate_segment': 4,
    'multi_blocker': 4,
    'no_relation_observation': 1012,
    'nonblocker_adjacent_branch': 8,
    'nonblocker_behind_same_segment': 8,
}
TARGET_ZONE_DELTAS = {
    'adjacent_branch': 6,
    'behind_region': 6,
    'intermediate_route': 2,
}


_V1_BUILD_SPECS = v1.build_specs
_V1_LOAD_CONFIG = v1._load_config
_V1_CREATE_PLAN = v1.create_plan
_V1_VERIFY_PLAN = v1.verify_plan
_V1_CAPTURE_COMMAND = v1.capture_command
_V1_CAPTURE_STATUS = v1.capture_status
_V1_FINALIZE_CAPTURE = v1.finalize_capture
_V1_VERIFY_SEAL = v1.verify_seal


def _rows_bytes(rows: list[dict[str, Any]]) -> bytes:
    return ''.join(v1.canonical_json(row) + '\n' for row in rows).encode('utf-8')


def _v1_rows() -> list[dict[str, Any]]:
    path, expected = V1_CONTROL_PINS['scenario_manifest']
    if not path.is_file() or v1.sha256_file(path) != expected:
        raise v1.FinalTestError('the immutable V1 manifest pin changed')
    rows = v1.read_jsonl(path)
    if len(rows) != V1_SCENARIO_COUNT or _rows_bytes(rows) != path.read_bytes():
        raise v1.FinalTestError('the V1 manifest is not the pinned canonical prefix')
    return rows


def _validate_v1_control_evidence(config: Mapping[str, Any]) -> None:
    declared = config.get('v1_pre_inference_prefix') or {}
    artifacts = declared.get('artifacts') or {}
    for name, (expected_path, expected_sha256) in V1_CONTROL_PINS.items():
        item = artifacts.get(name) or {}
        if (
            Path(str(item.get('path') or '')).expanduser().resolve()
            != expected_path.resolve()
            or item.get('sha256') != expected_sha256
            or not expected_path.is_file()
            or v1.sha256_file(expected_path) != expected_sha256
        ):
            raise v1.FinalTestError(f'V1 control pin mismatch: {name}')

    summary = v1.read_json(V1_CONTROL_PINS['scenario_summary'][0])
    preregistration = v1.read_json(V1_CONTROL_PINS['preregistration'][0])
    lock = v1.read_json(V1_CONTROL_PINS['plan_lock'][0])
    finalization = v1.read_json(V1_CONTROL_PINS['finalization'][0])
    seal = v1.read_json(V1_CONTROL_PINS['seal'][0])
    protocol = v1.read_json(V1_CONTROL_PINS['evaluation_protocol_lock'][0])
    contract = v1.read_json(V1_CONTROL_PINS['unexecuted_evaluation_contract'][0])
    status_objects = (summary, preregistration, lock, finalization, seal, protocol)
    if any(
        item.get('inference_status') != 'not_run'
        or int(item.get('inference_count', -1)) != 0
        for item in status_objects
    ):
        raise v1.FinalTestError('V1 is no longer eligible for a pre-inference extension')
    if any(
        int(item.get('scenario_count', V1_SCENARIO_COUNT)) != V1_SCENARIO_COUNT
        for item in (summary, lock, finalization, seal)
    ):
        raise v1.FinalTestError('V1 control evidence has an unexpected scenario count')
    if any(protocol.get(key) is not False for key in (
        'test_images_opened', 'test_labels_opened', 'test_rows_opened'
    )):
        raise v1.FinalTestError('the V1 protocol records Test-data exposure')
    if contract.get('prior_exposure') is not False:
        raise v1.FinalTestError('the unexecuted V1 contract records prior exposure')
    if any(path.exists() for path in FORBIDDEN_PRIOR_INFERENCE_ARTIFACTS):
        raise v1.FinalTestError('a V1 inference attempt ledger or output already exists')
    if (
        declared.get('inference_status') != 'not_run'
        or int(declared.get('inference_count', -1)) != 0
        or declared.get('decision_basis')
        != 'static_manifest_support_counts_only_before_inference'
        or declared.get('test_images_opened_for_decision') is not False
        or declared.get('test_rows_or_labels_opened_for_decision') is not False
        or declared.get('predictions_or_metrics_opened_for_decision') is not False
    ):
        raise v1.FinalTestError('the pre-inference extension declaration is incomplete')


def _load_config_in_context(path: Path) -> dict[str, Any]:
    config = _V1_LOAD_CONFIG(path)
    composition = config.get('composition') or {}
    if composition.get('v1_byte_identical_prefix_scenarios') != V1_SCENARIO_COUNT:
        raise v1.FinalTestError('configuration must pin the 1,024-row V1 prefix')
    if composition.get('added_pre_inference_stress_scenarios') != ADDED_STRESS_COUNT:
        raise v1.FinalTestError('configuration must add exactly 16 stress scenes')
    if composition.get('presence_cardinality_counts') != {
        str(value): 130 for value in range(1, 9)
    }:
        raise v1.FinalTestError('configuration must preregister 130 of each cardinality')
    if composition.get('added_relation_family_counts') != ADDED_RELATION_COUNTS:
        raise v1.FinalTestError('configuration relation extension does not match source')
    if composition.get('planned_identity_zone_deltas') != TARGET_ZONE_DELTAS:
        raise v1.FinalTestError('configuration identity-zone deltas do not match source')
    if int(composition.get('minimum_aggregate_identity_zone_support', -1)) != 8:
        raise v1.FinalTestError('minimum aggregate identity-zone support must be eight')
    if not all((config.get('prohibitions') or {}).values()):
        raise v1.FinalTestError('all final-Test prohibitions must remain enabled')
    _validate_v1_control_evidence(config)
    return config


def load_config(path: Path = DEFAULT_CONFIG) -> dict[str, Any]:
    with _extended_contract():
        return _load_config_in_context(path)


def build_specs(seed: int = SEED) -> list[dict[str, Any]]:
    """Return the exact V1 specs followed by 16 support-targeted stress specs."""
    if seed != SEED:
        raise v1.FinalTestError(f'final-Test seed is frozen at {SEED}')
    old_count, old_stress = v1.SCENARIO_COUNT, v1.STRESS_COUNT
    try:
        v1.SCENARIO_COUNT = V1_SCENARIO_COUNT
        v1.STRESS_COUNT = V1_STRESS_COUNT
        specs = _V1_BUILD_SPECS(seed)
    finally:
        v1.SCENARIO_COUNT, v1.STRESS_COUNT = old_count, old_stress

    with v1._hard_case_seed(seed):
        for added_index, relation in enumerate(ADDED_RELATIONS):
            index = V1_SCENARIO_COUNT + added_index
            offset = -0.0091 if added_index % 2 == 0 else 0.0091
            spec = v1._base_spec(
                index=index,
                cardinality=added_index // 2 + 1,
                target=v1.IDENTITIES[added_index % len(v1.IDENTITIES)],
                target_state=v1.LOADED_STATES[added_index // 8],
                segment=v1.BLOCKS[(added_index * 3 + 2) % len(v1.BLOCKS)],
                ratio=(
                    v1.POSITION_RATIOS[
                        (added_index * 5 + 2) % len(v1.POSITION_RATIOS)
                    ] + offset
                ),
                zone=v1.TARGET_ZONES[added_index % len(v1.TARGET_ZONES)],
                relation=relation,
                offset_bucket=f'pre_inference_coverage_extension_{added_index + 1:02d}',
                offset=offset,
                stress=True,
            )
            spec['hard_case_tags'].extend([
                'pre_inference_coverage_extension_v2',
                'v1_never_inferred_before_extension',
            ])
            specs.append(spec)
    if len(specs) != SCENARIO_COUNT:
        raise v1.FinalTestError('coverage-extension spec count mismatch')
    return specs


def _fingerprints(row: Mapping[str, Any]) -> dict[str, str | None]:
    return {
        'scenario_ids': str(row['scenario_id']),
        'scenario_family_digests': v1._family_digest(row['scenario_family']),
        'configuration_family_digests': v1._family_digest(
            row['configuration_family_id']
        ),
        'configuration_core_family_digests': v1._family_digest(
            row['configuration_core_family_id']
        ),
        'capture_configuration_fingerprints': str(
            row['capture_configuration_fingerprint']
        ),
        'geometry_fingerprints': str(row['geometry_fingerprint']),
        'trajectory_fingerprints': v1._scenario_trajectory_fingerprint(row),
        'semantic_fingerprints': v1._scenario_semantic_fingerprint(row),
    }


def _materialize_plan_in_context(
    config: Mapping[str, Any],
    references: Mapping[str, Any],
) -> list[dict[str, Any]]:
    _validate_v1_control_evidence(config)
    prefix = _v1_rows()
    local = {name: set() for name in _fingerprints(prefix[0])}
    for row in prefix:
        for name, value in _fingerprints(row).items():
            if value is None or value in local[name]:
                raise v1.FinalTestError(f'V1 prefix is not unique for {name}')
            local[name].add(str(value))

    cameras = v1.hard_generator.load_camera_projections(
        v1.hard_generator._default_camera_model_path()
    )
    identity_contract = v1.hard_generator._identity_visual_contract()
    added_rows = []
    with v1._hard_case_seed(int(config['seed'])):
        for source in build_specs(int(config['seed']))[V1_SCENARIO_COUNT:]:
            last_reason = 'no attempt made'
            for attempt in range(256):
                candidate = v1._retry_spec(source, attempt)
                try:
                    raw = v1.hard_generator.materialize_spec(
                        candidate,
                        cameras=cameras,
                        identity_contract=identity_contract,
                    )
                except v1.VisualV3Error as exc:
                    last_reason = str(exc)
                    continue
                row = v1._v4ize_scenario(
                    raw, candidate, int(candidate['generation_index']) + 1
                )
                values = _fingerprints(row)
                collision = next((
                    name
                    for name, value in values.items()
                    if value is None
                    or value in references[name]
                    or value in local[name]
                ), None)
                if collision:
                    last_reason = f'{collision} collision'
                    continue
                row.update({
                    'coverage_extension_version': GENERATOR_VERSION,
                    'coverage_extension_basis': (
                        'static_manifest_support_counts_only_before_inference'
                    ),
                    'v1_prefix_manifest_sha256': V1_MANIFEST_SHA256,
                    'v1_inference_status_before_extension': 'not_run',
                    'v1_inference_count_before_extension': 0,
                })
                for name, value in values.items():
                    local[name].add(str(value))
                added_rows.append(row)
                break
            else:
                raise v1.FinalTestError(
                    f'{source["spec_id"]}: could not isolate extension scene: '
                    f'{last_reason}'
                )
    rows = prefix + added_rows
    v1.validate_scenarios(rows)
    if len(rows) != SCENARIO_COUNT or _rows_bytes(rows[:1024]) != (
        V1_CONTROL_PINS['scenario_manifest'][0].read_bytes()
    ):
        raise v1.FinalTestError('materialized plan does not preserve V1 byte prefix')
    return rows


def materialize_plan(
    config: Mapping[str, Any], references: Mapping[str, Any]
) -> list[dict[str, Any]]:
    with _extended_contract():
        return _materialize_plan_in_context(config, references)


def _identity_zone_counts(rows: list[dict[str, Any]]) -> Counter[str]:
    return Counter(
        zone for row in rows for zone in row['identity_to_zone'].values()
    )


def _assert_plan_contract_in_context(
    rows: list[dict[str, Any]],
    support: Mapping[str, Any],
    uniqueness: Mapping[str, Any],
    static_disjoint: Mapping[str, Any],
) -> None:
    issues = []
    prefix, added = rows[:V1_SCENARIO_COUNT], rows[V1_SCENARIO_COUNT:]
    if len(rows) != SCENARIO_COUNT:
        issues.append('scenario count is not 1040')
    if _rows_bytes(prefix) != V1_CONTROL_PINS['scenario_manifest'][0].read_bytes():
        issues.append('the first 1024 manifest rows are not byte-identical V1')
    if support['lattice_scenario_count'] != LATTICE_COUNT:
        issues.append('lattice scenario count is not 1008')
    if support['stress_scenario_count'] != STRESS_COUNT:
        issues.append('stress scenario count is not 32')
    expected_cardinality = {str(value): 130 for value in range(1, 9)}
    if support['presence_cardinality_counts'] != expected_cardinality:
        issues.append('presence cardinalities are not exactly 130 each')
    if Counter(row['relation_family'] for row in added) != ADDED_RELATION_COUNTS:
        issues.append('the added relation-family counts changed')
    if support['records_by_relation_family'] != EXPECTED_RELATION_COUNTS:
        issues.append('the aggregate relation-family counts changed')

    prefix_zones = _identity_zone_counts(prefix)
    added_zones = _identity_zone_counts(added)
    aggregate_zones = _identity_zone_counts(rows)
    if any(added_zones[zone] != delta for zone, delta in TARGET_ZONE_DELTAS.items()):
        issues.append('the targeted identity-zone deltas changed')
    if any(
        aggregate_zones[zone] - prefix_zones[zone] != delta
        for zone, delta in TARGET_ZONE_DELTAS.items()
    ):
        issues.append('aggregate identity-zone deltas do not match preregistration')
    below_minimum = {
        zone: aggregate_zones[zone]
        for zone in v1.IDENTITY_ZONES
        if aggregate_zones[zone] < MINIMUM_AGGREGATE_IDENTITY_ZONE_SUPPORT
    }
    if below_minimum:
        issues.append(f'aggregate identity-zone support is below eight: {below_minimum}')

    if set(support['target_by_identity']) != set(v1.IDENTITIES):
        issues.append('not all identities are targets')
    for side in v1.SIDES:
        side_support = support['target_by_side_x_segment'].get(side, {})
        if set(side_support) != set(v1.BLOCKS) or any(
            count < 36 for count in side_support.values()
        ):
            issues.append(f'{side} target segment support is incomplete')
    if set(support['target_by_position_bin']) != set(v1.POSITION_BINS):
        issues.append('target position-bin support is incomplete')
    if set(support['target_by_loaded_state']) != set(v1.LOADED_STATES):
        issues.append('target loaded-state support is incomplete')
    if set(support['target_by_zone']) != set(v1.TARGET_ZONES):
        issues.append('target-zone support is incomplete')
    if set(support['records_by_estimated_occlusion_class']) != {
        'clear', 'partial_risk'
    }:
        issues.append('occlusion-class support is incomplete')
    if any(not result['unique'] for result in uniqueness.values()):
        issues.append('one or more required plan fingerprints are not unique')
    if not static_disjoint['passed']:
        issues.append('static train/validation/canary disjoint audit failed')
    if any(
        row.get('coverage_extension_basis')
        != 'static_manifest_support_counts_only_before_inference'
        or row.get('v1_inference_status_before_extension') != 'not_run'
        or int(row.get('v1_inference_count_before_extension', -1)) != 0
        for row in added
    ):
        issues.append('added rows lack pre-inference provenance')
    if issues:
        raise v1.FinalTestError(
            'coverage-extension plan contract failed: ' + '; '.join(issues)
        )


@contextmanager
def _extended_contract() -> Iterator[None]:
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
        'build_specs': build_specs,
        'materialize_plan': _materialize_plan_in_context,
        '_assert_plan_contract': _assert_plan_contract_in_context,
    }
    previous = {name: getattr(v1, name) for name in replacements}
    try:
        for name, value in replacements.items():
            setattr(v1, name, value)
        yield
    finally:
        for name, value in previous.items():
            setattr(v1, name, value)


def create_plan(config_path: Path, *, output_root: Path | None = None) -> dict[str, Any]:
    with _extended_contract():
        return _V1_CREATE_PLAN(config_path, output_root=output_root)


def verify_plan(root: Path, config_path: Path, *, regenerate: bool = True) -> dict[str, Any]:
    with _extended_contract():
        return _V1_VERIFY_PLAN(root, config_path, regenerate=regenerate)


def capture_command(root: Path, config_path: Path) -> list[str]:
    with _extended_contract():
        return _V1_CAPTURE_COMMAND(root, config_path)


def capture_status(root: Path, config_path: Path) -> dict[str, Any]:
    with _extended_contract():
        result = _V1_CAPTURE_STATUS(root, config_path)
    result['schema_version'] = (
        'room315.visual_v4.final_test_coverage_extension_capture_status.v2'
    )
    return result


def finalize_capture(root: Path, config_path: Path) -> dict[str, Any]:
    with _extended_contract():
        return _V1_FINALIZE_CAPTURE(root, config_path)


def verify_seal(root: Path, config_path: Path) -> dict[str, Any]:
    with _extended_contract():
        return _V1_VERIFY_SEAL(root, config_path)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('plan')
    verify = commands.add_parser('verify-plan')
    verify.add_argument('--skip-deterministic-regeneration', action='store_true')
    commands.add_parser('capture-command')
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
    elif args.command == 'capture-command':
        result = {
            'dataset_role': v1.DATASET_ROLE,
            'command': capture_command(args.root, args.config),
            'note': 'Printed only; no capture or inference was started.',
            'inference_status': 'not_run',
        }
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
    except (v1.FinalTestError, OSError, v1.VisualV3Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
