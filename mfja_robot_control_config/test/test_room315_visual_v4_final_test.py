#!/usr/bin/env python3
"""Contract tests for the preregistered Room 315 V4 final Test."""

from __future__ import annotations

import hashlib
import json
import sys
from collections import Counter
from pathlib import Path

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_v4_final_test as final_test


CONFIG = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'room_315_vla'
    / 'visual_state_final_test_v4.json'
)


def test_preregistration_configuration_is_exact_and_one_shot() -> None:
    config = json.loads(CONFIG.read_text(encoding='utf-8'))
    assert config['schema_version'] == final_test.CONFIG_SCHEMA
    assert config['dataset_role'] == 'sealed_final_test_only'
    assert config['seed'] == 3152026081101
    assert config['scenario_count'] == 1024
    assert config['composition']['lattice_scenarios'] == 1008
    assert config['composition']['stress_scenarios'] == 16
    assert all(config['prohibitions'].values())
    assert {
        source['role'] for source in config['reference_sources']
    } == {'train', 'validation', 'canary'}
    assert {
        source['name'] for source in config['reference_sources']
    } == final_test.EXPECTED_REFERENCE_NAMES


def test_specs_are_deterministic_and_have_exact_preregistered_support() -> None:
    first = final_test.build_specs()
    second = final_test.build_specs()
    assert final_test.canonical_json(first) == final_test.canonical_json(second)
    assert len(first) == 1024
    assert sum(
        'v4_final_test_lattice' in row['hard_case_tags'] for row in first
    ) == 1008
    assert sum(
        'v4_final_test_stress' in row['hard_case_tags'] for row in first
    ) == 16
    assert Counter(len(row['active_identities']) for row in first) == {
        cardinality: 128 for cardinality in range(1, 9)
    }
    assert {row['target_identity'] for row in first} == set(final_test.IDENTITIES)
    assert {row['target_position_bin'] for row in first} == set(
        final_test.POSITION_BINS
    )
    assert {row['target_loaded_state'] for row in first} == {'empty', 'loaded'}
    assert {row['target_zone'] for row in first} == set(final_test.TARGET_ZONES)
    assert {row['relation_family'] for row in first} == set(final_test.RELATIONS)


def test_lattice_is_exact_side_segment_bin_state_replicate_product() -> None:
    specs = final_test.build_specs()[: final_test.LATTICE_COUNT]
    cells = Counter()
    for spec in specs:
        cells[(
            final_test.identity_side(spec['target_identity']),
            spec['target_block'],
            spec['target_position_bin'],
            spec['target_loaded_state'],
        )] += 1
        assert spec['relation_family'] == final_test.NO_RELATION
    expected = {
        (side, segment, position, loaded): 2
        for side in final_test.SIDES
        for segment in final_test.BLOCKS
        for position in final_test.POSITION_BINS
        for loaded in final_test.LOADED_STATES
    }
    assert cells == expected
    assert Counter(
        (
            final_test.identity_side(spec['target_identity']),
            spec['target_block'],
        )
        for spec in specs
    ) == {
        (side, segment): 36
        for side in final_test.SIDES
        for segment in final_test.BLOCKS
    }
    assert Counter(spec['target_position_bin'] for spec in specs) == {
        position: 112 for position in final_test.POSITION_BINS
    }
    assert Counter(spec['target_loaded_state'] for spec in specs) == {
        state: 504 for state in final_test.LOADED_STATES
    }
    identity_counts = Counter(spec['target_identity'] for spec in specs)
    assert min(identity_counts.values()) == 125
    assert max(identity_counts.values()) == 127


def test_unlisted_reference_is_rejected_before_hashing(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    prohibited = tmp_path / 'unexpected.jsonl'
    prohibited.write_text('{"must_not_be_opened":true}\n', encoding='utf-8')
    opened = False

    def forbidden_hash(_path: Path) -> str:
        nonlocal opened
        opened = True
        raise AssertionError('unlisted reference was hashed')

    monkeypatch.setattr(final_test, 'sha256_file', forbidden_hash)
    with pytest.raises(final_test.FinalTestError, match='not in the explicit'):
        final_test._allowed_reference(prohibited, '0' * 64)
    assert not opened


def test_reference_allowlist_contains_no_historical_test_path() -> None:
    paths = [str(path).lower() for path in final_test.REFERENCE_FILE_ALLOWLIST]
    assert all(not path.endswith('/test.jsonl') for path in paths)
    assert all(not path.endswith('/test_visual_labels.jsonl') for path in paths)
    assert all('/final_test' not in path for path in paths)


def test_historical_test_config_is_rejected_before_opening(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    opened = False

    def forbidden_read(_path: Path) -> dict:
        nonlocal opened
        opened = True
        raise AssertionError('historical Test was opened as configuration')

    monkeypatch.setattr(final_test, 'read_json', forbidden_read)
    with pytest.raises(final_test.FinalTestError, match='historical Test'):
        final_test._load_config(next(iter(final_test.HISTORICAL_TEST_PATHS)))
    assert not opened


def test_contract_pair_hash_definition_is_exact() -> None:
    payload = {
        'sample_id': 'v4_final_test_0001:step:0',
        'left_sha256': '1' * 64,
        'right_sha256': '2' * 64,
    }
    expected = hashlib.sha256(
        json.dumps(
            payload,
            sort_keys=True,
            separators=(',', ':'),
            ensure_ascii=False,
        ).encode('utf-8')
    ).hexdigest()
    assert final_test.contract_pair_sha(
        payload['sample_id'], payload['left_sha256'], payload['right_sha256']
    ) == expected


def test_v4_freshness_prefixes_replace_materializer_transport_names() -> None:
    raw = {
        'configuration_family_id': f'v3_family_{"a" * 64}',
        'configuration_core_family_id': f'v3_family_{"b" * 64}',
        'capture_configuration_fingerprint': 'c' * 64,
        'geometry_fingerprint': 'd' * 64,
    }
    spec = {'spec_id': 'v4_final_test_lattice_0001'}
    row = final_test._v4ize_scenario(raw, spec, 1)
    assert row['scenario_id'].startswith('v4_final_test_')
    assert row['scenario_family'].startswith('v4_final_test_family_')
    assert row['configuration_family_id'].startswith('v4_final_test_family_')
    assert row['configuration_core_family_id'].startswith(
        'v4_final_test_family_'
    )
    assert row['dataset_partition'] == 'final_test'
    assert row['source_profile'] == 'final_test'
    assert row['imported_from_v3'] is False
    assert row['inference_exposure'] == 'never_evaluated_at_plan_time'


def test_cli_has_no_inference_or_evaluation_command() -> None:
    parser = final_test._parser()
    help_text = parser.format_help()
    source = Path(final_test.__file__).read_text(encoding='utf-8')
    assert 'plan' in source
    assert 'verify-plan' in source
    assert 'capture-command' in source
    assert 'finalize' in source
    assert 'verify-seal' in source
    assert 'torch' not in source
    assert 'room_315_visual_model' not in source
    assert 'evaluate' not in help_text.lower()
    subparser_action = next(
        action
        for action in parser._actions
        if getattr(action, 'choices', None)
    )
    assert set(subparser_action.choices) == {
        'plan',
        'verify-plan',
        'capture-command',
        'status',
        'finalize',
        'verify-seal',
    }
