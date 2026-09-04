#!/usr/bin/env python3
"""Focused contracts for the pre-inference V4 final-Test extension."""

from __future__ import annotations

import copy
import json
import sys
from collections import Counter
from pathlib import Path

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_v4_final_test as v1
import room_315_visual_v4_final_test_coverage_extension as extension


CONFIG = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'room_315_visual_state'
    / 'visual_state_final_test_v4_coverage_extension.json'
)


@pytest.fixture(scope='module', autouse=True)
def isolated_prior_inference_artifacts(
    tmp_path_factory: pytest.TempPathFactory,
):
    """Keep pre-inference unit contracts independent of production ledgers."""
    isolated = tmp_path_factory.mktemp('coverage-extension-inference-state')
    patch = pytest.MonkeyPatch()
    patch.setattr(
        extension,
        'FORBIDDEN_PRIOR_INFERENCE_ARTIFACTS',
        (isolated / 'attempt_ledger', isolated / 'outputs'),
    )
    yield
    patch.undo()


@pytest.fixture(scope='module')
def materialized(
    isolated_prior_inference_artifacts: None,
) -> tuple[dict, dict, list[dict]]:
    config = extension.load_config(CONFIG)
    references = v1._reference_index(config)
    rows = extension.materialize_plan(config, references)
    return config, references, rows


def test_prior_inference_guard_remains_fail_closed(tmp_path, monkeypatch) -> None:
    consumed = tmp_path / 'attempt_ledger'
    consumed.mkdir()
    monkeypatch.setattr(
        extension,
        'FORBIDDEN_PRIOR_INFERENCE_ARTIFACTS',
        (consumed,),
    )
    config = json.loads(CONFIG.read_text(encoding='utf-8'))
    with pytest.raises(v1.FinalTestError, match='inference attempt'):
        extension._validate_v1_control_evidence(config)


def test_configuration_records_pre_inference_reason_and_exact_counts() -> None:
    config = json.loads(CONFIG.read_text(encoding='utf-8'))
    assert config['schema_version'] == extension.CONFIG_SCHEMA
    assert config['scenario_count'] == 1040
    assert config['composition']['lattice_scenarios'] == 1008
    assert config['composition']['stress_scenarios'] == 32
    assert config['composition']['v1_byte_identical_prefix_scenarios'] == 1024
    assert config['composition']['added_pre_inference_stress_scenarios'] == 16
    assert config['composition']['presence_cardinality_counts'] == {
        str(value): 130 for value in range(1, 9)
    }
    provenance = config['v1_pre_inference_prefix']
    assert provenance['decision_basis'] == (
        'static_manifest_support_counts_only_before_inference'
    )
    assert provenance['inference_status'] == 'not_run'
    assert provenance['inference_count'] == 0
    assert provenance['test_images_opened_for_decision'] is False
    assert provenance['test_rows_or_labels_opened_for_decision'] is False
    assert provenance['predictions_or_metrics_opened_for_decision'] is False


def test_specs_add_two_scenes_per_cardinality_and_exact_relations() -> None:
    first = extension.build_specs()
    second = extension.build_specs()
    assert v1.canonical_json(first) == v1.canonical_json(second)
    assert len(first) == 1040
    added = first[1024:]
    assert Counter(len(row['active_identities']) for row in first) == {
        cardinality: 130 for cardinality in range(1, 9)
    }
    assert Counter(len(row['active_identities']) for row in added) == {
        cardinality: 2 for cardinality in range(1, 9)
    }
    assert Counter(row['relation_family'] for row in added) == (
        extension.ADDED_RELATION_COUNTS
    )
    assert all(
        'pre_inference_coverage_extension_v2' in row['hard_case_tags']
        for row in added
    )


def test_materialized_prefix_relations_cardinalities_and_zone_minima_are_exact(
    materialized: tuple[dict, dict, list[dict]],
) -> None:
    _config, references, rows = materialized
    prefix_path = extension.V1_CONTROL_PINS['scenario_manifest'][0]
    assert extension._rows_bytes(rows[:1024]) == prefix_path.read_bytes()
    assert v1.sha256_file(prefix_path) == extension.V1_MANIFEST_SHA256

    assert Counter(len(row['active_identities']) for row in rows) == {
        cardinality: 130 for cardinality in range(1, 9)
    }
    assert Counter(row['relation_family'] for row in rows) == (
        extension.EXPECTED_RELATION_COUNTS
    )
    added = rows[1024:]
    assert Counter(row['relation_family'] for row in added) == (
        extension.ADDED_RELATION_COUNTS
    )

    prefix_zones = extension._identity_zone_counts(rows[:1024])
    added_zones = extension._identity_zone_counts(added)
    aggregate_zones = extension._identity_zone_counts(rows)
    assert {
        zone: added_zones[zone]
        for zone in extension.TARGET_ZONE_DELTAS
    } == extension.TARGET_ZONE_DELTAS
    assert {
        zone: aggregate_zones[zone] - prefix_zones[zone]
        for zone in extension.TARGET_ZONE_DELTAS
    } == extension.TARGET_ZONE_DELTAS
    assert {zone: aggregate_zones[zone] for zone in (
        'adjacent_branch', 'behind_region', 'intermediate_route', 'ahead_region'
    )} == {
        'adjacent_branch': 8,
        'behind_region': 8,
        'intermediate_route': 8,
        'ahead_region': 8,
    }
    assert min(aggregate_zones[zone] for zone in v1.IDENTITY_ZONES) == 8

    support = v1.plan_support_summary(rows)
    uniqueness = v1._uniqueness_summary(rows)
    disjoint = v1._static_disjoint_audit(rows, references)
    with extension._extended_contract():
        extension._assert_plan_contract_in_context(
            rows, support, uniqueness, disjoint
        )


def test_plan_contract_fails_if_any_required_aggregate_zone_drops_below_eight(
    materialized: tuple[dict, dict, list[dict]],
) -> None:
    _config, references, rows = materialized
    damaged = copy.deepcopy(rows)
    for row in damaged[1024:]:
        for identity, zone in tuple(row['identity_to_zone'].items()):
            if zone == 'adjacent_branch':
                row['identity_to_zone'][identity] = 'relation_neutral'
    support = v1.plan_support_summary(damaged)
    uniqueness = v1._uniqueness_summary(damaged)
    disjoint = v1._static_disjoint_audit(damaged, references)
    with pytest.raises(v1.FinalTestError, match='below eight'):
        with extension._extended_contract():
            extension._assert_plan_contract_in_context(
                damaged, support, uniqueness, disjoint
            )


def test_cli_defaults_are_new_and_has_no_evaluation_command() -> None:
    parser = extension._parser()
    defaults = parser.parse_args(['status'])
    assert defaults.config == extension.DEFAULT_CONFIG
    assert defaults.root == extension.DEFAULT_ROOT
    action = next(
        item for item in parser._actions if getattr(item, 'choices', None)
    )
    assert set(action.choices) == {
        'plan', 'verify-plan', 'capture-command', 'status', 'finalize', 'verify-seal'
    }
    assert 'evaluate' not in parser.format_help().lower()
