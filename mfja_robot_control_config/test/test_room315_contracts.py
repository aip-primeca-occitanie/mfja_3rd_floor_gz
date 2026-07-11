#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_contracts.py'


def _load_contracts():
    spec = importlib.util.spec_from_file_location('room_315_contracts', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _visual_fact(contracts, **overrides):
    payload = {
        'fact_id': 'vf-1',
        'subject': 'room315_right_shuttle_1',
        'predicate': 'visible_slot',
        'value': {'slot': 'DZI3R'},
        'source': 'visual_model',
        'timestamp': 10.5,
        'confidence': 0.92,
        'status': 'known',
    }
    payload.update(overrides)
    return contracts.ObservedFact(**payload)


def _fused_fact(contracts, **overrides):
    payload = {
        'fact_id': 'ff-1',
        'subject': 'room315_right_shuttle_1',
        'predicate': 'at_slot',
        'value': 'DZI3R',
        'source': 'state_fuser',
        'timestamp': 10.6,
        'confidence': 0.88,
        'status': 'known',
    }
    payload.update(overrides)
    return contracts.ObservedFact(**payload)


def test_contracts_round_trip_all_closed_loop_boundaries():
    contracts = _load_contracts()
    visual_fact = _visual_fact(contracts)
    fused_fact = _fused_fact(contracts)
    state = contracts.ObservedState(
        state_id='state-1',
        timestamp=10.7,
        stale_after_s=0.5,
        visual_model_inputs=[visual_fact],
        fused_planner_state=[fused_fact],
    )
    goal = contracts.TaskGoal(
        goal_id='goal-1',
        description='deliver target shuttle to the right-side Staubli slot',
        source='learned_task_goal',
        timestamp=10.8,
        confidence=0.74,
        constraints={
            'side': 'right',
            'target_slot': 'staubli',
            'target_shuttle': 'room315_right_shuttle_1',
            'payload_required': True,
            'max_plan_steps': 12,
        },
    )
    step = contracts.PlanStep(
        step_id='step-1',
        plan_id='plan-1',
        index=0,
        action_name='move-shuttle',
        arguments={'side': 'right', 'from_slot': 'DZI3R', 'to_slot': 'DZI2R'},
        source='plansys2',
        timestamp=10.9,
        preconditions=[fused_fact],
        expected_effects=[
            _fused_fact(
                contracts,
                fact_id='ff-2',
                value='DZI2R',
                timestamp=11.0,
            )
        ],
    )
    command = contracts.PrimitiveCommand(
        command_id='cmd-1',
        plan_step_id='step-1',
        primitive='SHUTTLE_ON',
        parameters={
            'side': 'right',
            'target_shuttle': 'room315_right_shuttle_1',
            'speed_mps': 0.08,
        },
        source='plan_translator',
        timestamp=11.1,
        legacy_action_vector={
            'baseline_id': contracts.LEGACY_DIRECT_ACTION_BASELINE,
            'schema_version': contracts.ACTION_VECTOR_SCHEMA_VERSION,
            'enabled': False,
            'vector': [0.0] * 24,
        },
    )
    result = contracts.StepResult(
        result_id='result-1',
        command_id='cmd-1',
        plan_step_id='step-1',
        status='executed',
        source='executor',
        timestamp=11.2,
        reason='ok',
        observed_state_id='state-2',
        facts=[_fused_fact(contracts, fact_id='ff-3', value='DZI2R', timestamp=11.2)],
    )

    for contract in (visual_fact, state, goal, step, command, result):
        payload = contract.to_dict()
        restored = contracts.contract_from_dict(payload)
        assert restored.to_dict() == payload


@pytest.mark.parametrize('status', ['known', 'unknown', 'stale', 'conflicting'])
def test_observed_fact_accepts_all_declared_statuses(status):
    contracts = _load_contracts()
    fact = _visual_fact(contracts, status=status)
    assert fact.status == status


@pytest.mark.parametrize(
    'overrides',
    [
        {'status': 'maybe'},
        {'timestamp': -1.0},
        {'confidence': 1.1},
        {'source': 'model_payload'},
    ],
)
def test_observed_fact_rejects_invalid_source_timestamp_confidence_and_status(overrides):
    contracts = _load_contracts()
    with pytest.raises(contracts.ContractValidationError):
        _visual_fact(contracts, **overrides)


def test_observed_state_separates_visual_inputs_from_fused_planner_state():
    contracts = _load_contracts()
    sensor_fact = _fused_fact(contracts, source='sensor')
    visual_fact = _visual_fact(contracts)

    with pytest.raises(contracts.ContractValidationError, match='visual_model_inputs'):
        contracts.ObservedState(
            state_id='state-bad-visual',
            timestamp=1.0,
            visual_model_inputs=[sensor_fact],
            fused_planner_state=[],
        )

    with pytest.raises(contracts.ContractValidationError, match='fused'):
        contracts.ObservedState(
            state_id='state-bad-fused',
            timestamp=1.0,
            visual_model_inputs=[],
            fused_planner_state=[visual_fact],
        )

    known_a = _fused_fact(contracts, fact_id='ff-a', value='DZI1R')
    known_b = _fused_fact(contracts, fact_id='ff-b', value='DZI2R')
    with pytest.raises(contracts.ContractValidationError, match='conflicting'):
        contracts.ObservedState(
            state_id='state-conflict-unmarked',
            timestamp=1.0,
            fused_planner_state=[known_a, known_b],
        )

    conflicting_b = _fused_fact(
        contracts,
        fact_id='ff-c',
        value='DZI2R',
        status='conflicting',
    )
    state = contracts.ObservedState(
        state_id='state-conflict-marked',
        timestamp=1.0,
        fused_planner_state=[known_a, conflicting_b],
    )
    assert state.fused_planner_state[1].status == 'conflicting'


def test_learned_components_can_output_only_visual_facts_or_constrained_goals():
    contracts = _load_contracts()
    visual_fact = _visual_fact(contracts)
    learned_goal = contracts.TaskGoal(
        goal_id='goal-learned',
        description='deliver target shuttle to the requested slot',
        source='learned_task_goal',
        timestamp=2.0,
        confidence=0.71,
        constraints={'side': 'right', 'target_slot': 'staubli'},
    )
    command = contracts.PrimitiveCommand(
        command_id='cmd-bad',
        plan_step_id='step-1',
        primitive='WAIT',
        parameters={'duration_s': 0.1},
        source='plan_translator',
        timestamp=2.1,
    )

    assert contracts.validate_learned_component_output(visual_fact.to_dict()) == visual_fact
    assert contracts.validate_learned_component_output(learned_goal.to_dict()) == learned_goal
    with pytest.raises(contracts.ContractValidationError, match='learned components'):
        contracts.validate_learned_component_output(command.to_dict())

    with pytest.raises(contracts.ContractValidationError, match='unsupported TaskGoal'):
        contracts.TaskGoal(
            goal_id='goal-command-leak',
            description='bad learned goal',
            source='learned_task_goal',
            timestamp=2.2,
            confidence=0.8,
            constraints={'primitive': 'SHUTTLE_ON'},
        )


@pytest.mark.parametrize(
    'leaky_value',
    [
        {'pddl_problem': 'room315-right-yaskawa-to-staubli'},
        {'payload_present': True},
        {'action_vector': [0.0] * 24},
        {'primitive': 'SHUTTLE_ON'},
        {'nested': {'step_index': 4}},
    ],
)
def test_learned_visual_facts_reject_privileged_data_and_command_leakage(leaky_value):
    contracts = _load_contracts()
    with pytest.raises(contracts.ContractValidationError, match='privileged|command-like'):
        _visual_fact(contracts, value=leaky_value)


def test_privileged_data_is_rejected_from_planner_and_command_payloads():
    contracts = _load_contracts()
    with pytest.raises(contracts.ContractValidationError, match='privileged'):
        contracts.PlanStep(
            step_id='step-leak',
            plan_id='plan-1',
            index=0,
            action_name='move-shuttle',
            arguments={'pddl_problem': 'room315-right-yaskawa-to-staubli'},
            source='plansys2',
            timestamp=3.0,
        )
    with pytest.raises(contracts.ContractValidationError, match='privileged'):
        contracts.PrimitiveCommand(
            command_id='cmd-leak',
            plan_step_id='step-1',
            primitive='WAIT',
            parameters={'symbolic_plan': ['move-shuttle']},
            source='plan_translator',
            timestamp=3.1,
        )


def test_schema_v3_action_vectors_are_disabled_legacy_baseline_only():
    contracts = _load_contracts()
    good_command = contracts.PrimitiveCommand(
        command_id='cmd-legacy',
        plan_step_id='step-1',
        primitive='WAIT',
        parameters={'duration_s': 0.25},
        source='plan_translator',
        timestamp=4.0,
        legacy_action_vector={
            'baseline_id': contracts.LEGACY_DIRECT_ACTION_BASELINE,
            'schema_version': contracts.ACTION_VECTOR_SCHEMA_VERSION,
            'enabled': False,
            'vector': [0.0] * 24,
        },
    )
    assert good_command.legacy_action_vector['enabled'] is False

    with pytest.raises(contracts.ContractValidationError, match='disabled'):
        contracts.PrimitiveCommand(
            command_id='cmd-enabled-legacy',
            plan_step_id='step-1',
            primitive='WAIT',
            parameters={'duration_s': 0.25},
            source='plan_translator',
            timestamp=4.1,
            legacy_action_vector={
                'baseline_id': contracts.LEGACY_DIRECT_ACTION_BASELINE,
                'schema_version': contracts.ACTION_VECTOR_SCHEMA_VERSION,
                'enabled': True,
                'vector': [0.0] * 24,
            },
        )

    payload = good_command.to_dict()
    payload['action_vector_schema_version'] = contracts.ACTION_VECTOR_SCHEMA_VERSION
    payload['action_vector'] = [0.0] * 24
    with pytest.raises(contracts.ContractValidationError, match='legacy_action_vector'):
        contracts.PrimitiveCommand.from_dict(payload)


def test_contract_payload_requires_matching_schema_and_contract_type():
    contracts = _load_contracts()
    payload = _visual_fact(contracts).to_dict()

    payload['schema_version'] = contracts.CONTRACT_SCHEMA_VERSION + 1
    with pytest.raises(contracts.ContractValidationError, match='schema_version'):
        contracts.contract_from_dict(payload)

    payload = _visual_fact(contracts).to_dict()
    payload['contract_type'] = 'UnknownContract'
    with pytest.raises(contracts.ContractValidationError, match='unsupported contract_type'):
        contracts.contract_from_dict(payload)
