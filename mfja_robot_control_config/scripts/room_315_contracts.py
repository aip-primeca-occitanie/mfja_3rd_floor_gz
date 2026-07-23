#!/usr/bin/env python3
"""Versioned Room 315 neuro-symbolic closed-loop contracts.

The contracts in this module describe the boundary between perception,
symbolic planning, safety supervision, and execution. They are intentionally
free of ROS imports so they can be validated in CI and reused by data tooling.
"""

from __future__ import annotations

import copy
import json
import math
from dataclasses import asdict
from dataclasses import dataclass
from dataclasses import field
from typing import Any


CONTRACT_SCHEMA_VERSION = 1

FACT_STATUSES = frozenset({'known', 'unknown', 'stale', 'conflicting'})
FACT_SOURCES = frozenset({
    'sensor',
    'visual_model',
    'state_fuser',
    'planner',
    'plansys2',
    'supervisor',
    'executor',
    'manual',
    'oracle',
    'trusted_device',
})
LEARNED_OUTPUT_SOURCES = frozenset({'visual_model', 'learned_task_goal'})
TASK_GOAL_SOURCES = frozenset({'human', 'learned_task_goal', 'planner', 'supervisor'})
PLAN_STEP_SOURCES = frozenset({'plansys2', 'planner'})
COMMAND_SOURCES = frozenset({'plan_translator', 'supervisor', 'manual'})
STEP_RESULT_SOURCES = frozenset({'supervisor', 'executor', 'simulator'})
PRIMITIVES = frozenset({
    'WAIT',
    'DONE',
    'SET_SWITCHES',
    'SET_STOPPERS',
    'SHUTTLE_ON',
    'STOP_NOW',
    'EMERGENCY_STOP',
})
STEP_RESULT_STATUSES = frozenset({
    'accepted',
    'rejected',
    'executed',
    'failed',
    'timed_out',
    'stale',
})
TASK_GOAL_CONSTRAINT_KEYS = frozenset({
    'goal_type',
    'inspection_subject',
    'payload_filter',
    'shuttle_selection',
    'selection_strategy',
    'side',
    'target_slot',
    'target_kind',
    'target_station',
    'target_shuttle',
    'payload_required',
    'max_plan_steps',
    'deadline_s',
})

PRIVILEGED_FIELD_NAMES = frozenset({
    'action_vector',
    'action_vector_schema_version',
    'event_index',
    'gazebo_pose',
    'model_input_exposure',
    'payload_condition',
    'payload_present',
    'pddl_goal',
    'pddl_problem',
    'plan_step_index',
    'privileged_eval',
    'step_index',
    'symbolic_plan',
})
COMMAND_FIELD_NAMES = frozenset({
    'action',
    'action_name',
    'command',
    'parameters',
    'primitive',
    'rail_command',
    'speed_mps',
    'stopper_mask',
    'stopper_values',
    'switch_mask',
    'switch_values',
})


class ContractValidationError(ValueError):
    """Raised when a Room 315 contract payload violates the boundary."""


def _deepcopy_jsonable(value: Any) -> Any:
    return json.loads(json.dumps(value, sort_keys=True))


def _require_mapping(data: Any, context: str) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise ContractValidationError(f'{context} must be an object')
    return data


def _require_schema(data: dict[str, Any], contract_type: str) -> None:
    if data.get('schema_version') != CONTRACT_SCHEMA_VERSION:
        raise ContractValidationError(
            f'{contract_type} schema_version must be {CONTRACT_SCHEMA_VERSION}'
        )
    if data.get('contract_type') != contract_type:
        raise ContractValidationError(f'contract_type must be {contract_type!r}')


def _require_string(value: Any, name: str, *, allow_empty: bool = False) -> str:
    if not isinstance(value, str):
        raise ContractValidationError(f'{name} must be a string')
    text = value.strip()
    if not text and not allow_empty:
        raise ContractValidationError(f'{name} must be non-empty')
    return text


def _require_timestamp(value: Any, name: str = 'timestamp') -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ContractValidationError(f'{name} must be a numeric timestamp') from exc
    if not math.isfinite(result) or result < 0.0:
        raise ContractValidationError(f'{name} must be finite and non-negative')
    return result


def _require_confidence(value: Any) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ContractValidationError('confidence must be numeric') from exc
    if not math.isfinite(result) or result < 0.0 or result > 1.0:
        raise ContractValidationError('confidence must be in [0, 1]')
    return result


def _normalise_status(value: Any) -> str:
    status = _require_string(value, 'status')
    if status not in FACT_STATUSES:
        raise ContractValidationError(
            f'status must be one of {sorted(FACT_STATUSES)}'
        )
    return status


def _normalise_source(value: Any, allowed: frozenset[str], name: str = 'source') -> str:
    source = _require_string(value, name)
    if source not in allowed:
        raise ContractValidationError(f'{name} must be one of {sorted(allowed)}')
    return source


def _scan_for_keys(value: Any, blocked: frozenset[str], path: str = '$') -> list[str]:
    leaks: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            key_text = str(key)
            child_path = f'{path}.{key_text}'
            if key_text in blocked:
                leaks.append(child_path)
            leaks.extend(_scan_for_keys(child, blocked, child_path))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            leaks.extend(_scan_for_keys(child, blocked, f'{path}[{index}]'))
    return leaks


def _reject_privileged_keys(value: Any, context: str) -> None:
    leaks = _scan_for_keys(value, PRIVILEGED_FIELD_NAMES)
    if leaks:
        raise ContractValidationError(
            f'{context} contains privileged fields: {leaks[:5]}'
        )


def _reject_learned_privileged_or_command_keys(value: Any, context: str) -> None:
    leaks = _scan_for_keys(value, PRIVILEGED_FIELD_NAMES | COMMAND_FIELD_NAMES)
    if leaks:
        raise ContractValidationError(
            f'{context} contains privileged or command-like fields: {leaks[:5]}'
        )


def _fact_key(fact: 'ObservedFact') -> tuple[str, str, str]:
    return (fact.subject, fact.predicate, fact.frame_id)


@dataclass(frozen=True)
class ObservedFact:
    fact_id: str
    subject: str
    predicate: str
    value: Any
    source: str
    timestamp: float
    confidence: float
    status: str
    frame_id: str = 'room_315'
    metadata: dict[str, Any] = field(default_factory=dict)
    schema_version: int = CONTRACT_SCHEMA_VERSION
    contract_type: str = 'ObservedFact'

    def __post_init__(self) -> None:
        _require_string(self.fact_id, 'fact_id')
        _require_string(self.subject, 'subject')
        _require_string(self.predicate, 'predicate')
        _normalise_source(self.source, FACT_SOURCES)
        _require_timestamp(self.timestamp)
        _require_confidence(self.confidence)
        _normalise_status(self.status)
        _require_string(self.frame_id, 'frame_id')
        if not isinstance(self.metadata, dict):
            raise ContractValidationError('metadata must be an object')
        _require_schema(asdict(self), 'ObservedFact')
        if self.source in LEARNED_OUTPUT_SOURCES:
            _reject_learned_privileged_or_command_keys({
                'subject': self.subject,
                'predicate': self.predicate,
                'value': self.value,
                'metadata': self.metadata,
            }, 'learned ObservedFact')

    def to_dict(self) -> dict[str, Any]:
        return _deepcopy_jsonable(asdict(self))

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> 'ObservedFact':
        payload = _require_mapping(data, 'ObservedFact')
        _require_schema(payload, 'ObservedFact')
        return cls(
            fact_id=payload.get('fact_id'),
            subject=payload.get('subject'),
            predicate=payload.get('predicate'),
            value=copy.deepcopy(payload.get('value')),
            source=payload.get('source'),
            timestamp=payload.get('timestamp'),
            confidence=payload.get('confidence'),
            status=payload.get('status'),
            frame_id=payload.get('frame_id', 'room_315'),
            metadata=copy.deepcopy(payload.get('metadata') or {}),
        )


@dataclass(frozen=True)
class ObservedState:
    state_id: str
    timestamp: float
    visual_model_inputs: list[ObservedFact] = field(default_factory=list)
    fused_planner_state: list[ObservedFact] = field(default_factory=list)
    stale_after_s: float = 1.0
    schema_version: int = CONTRACT_SCHEMA_VERSION
    contract_type: str = 'ObservedState'

    def __post_init__(self) -> None:
        _require_string(self.state_id, 'state_id')
        _require_timestamp(self.timestamp)
        _require_timestamp(self.stale_after_s, 'stale_after_s')
        _require_schema(asdict(self), 'ObservedState')
        for fact in self.visual_model_inputs:
            if not isinstance(fact, ObservedFact):
                raise ContractValidationError('visual_model_inputs must contain ObservedFact')
            if fact.source != 'visual_model':
                raise ContractValidationError('visual_model_inputs require source visual_model')
        seen: dict[tuple[str, str, str], ObservedFact] = {}
        for fact in self.fused_planner_state:
            if not isinstance(fact, ObservedFact):
                raise ContractValidationError('fused_planner_state must contain ObservedFact')
            if fact.source == 'visual_model':
                raise ContractValidationError(
                    'visual_model facts must be fused before entering fused_planner_state'
                )
            existing = seen.get(_fact_key(fact))
            if existing is not None and existing.value != fact.value:
                if fact.status != 'conflicting' and existing.status != 'conflicting':
                    raise ContractValidationError(
                        f'fused planner state has unmarked conflicting facts: {_fact_key(fact)}'
                    )
            seen[_fact_key(fact)] = fact

    def to_dict(self) -> dict[str, Any]:
        return {
            'schema_version': self.schema_version,
            'contract_type': self.contract_type,
            'state_id': self.state_id,
            'timestamp': self.timestamp,
            'stale_after_s': self.stale_after_s,
            'visual_model_inputs': [fact.to_dict() for fact in self.visual_model_inputs],
            'fused_planner_state': [fact.to_dict() for fact in self.fused_planner_state],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> 'ObservedState':
        payload = _require_mapping(data, 'ObservedState')
        _require_schema(payload, 'ObservedState')
        return cls(
            state_id=payload.get('state_id'),
            timestamp=payload.get('timestamp'),
            stale_after_s=payload.get('stale_after_s', 1.0),
            visual_model_inputs=[
                ObservedFact.from_dict(item)
                for item in payload.get('visual_model_inputs') or []
            ],
            fused_planner_state=[
                ObservedFact.from_dict(item)
                for item in payload.get('fused_planner_state') or []
            ],
        )


@dataclass(frozen=True)
class TaskGoal:
    goal_id: str
    description: str
    source: str
    timestamp: float
    confidence: float
    constraints: dict[str, Any] = field(default_factory=dict)
    schema_version: int = CONTRACT_SCHEMA_VERSION
    contract_type: str = 'TaskGoal'

    def __post_init__(self) -> None:
        _require_string(self.goal_id, 'goal_id')
        _require_string(self.description, 'description')
        _normalise_source(self.source, TASK_GOAL_SOURCES)
        _require_timestamp(self.timestamp)
        _require_confidence(self.confidence)
        if not isinstance(self.constraints, dict):
            raise ContractValidationError('constraints must be an object')
        unexpected = sorted(set(self.constraints) - TASK_GOAL_CONSTRAINT_KEYS)
        if unexpected:
            raise ContractValidationError(f'unsupported TaskGoal constraints: {unexpected}')
        _require_schema(asdict(self), 'TaskGoal')
        if self.source in LEARNED_OUTPUT_SOURCES:
            _reject_learned_privileged_or_command_keys({
                'description': self.description,
                'constraints': self.constraints,
            }, 'learned TaskGoal')

    def to_dict(self) -> dict[str, Any]:
        return _deepcopy_jsonable(asdict(self))

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> 'TaskGoal':
        payload = _require_mapping(data, 'TaskGoal')
        _require_schema(payload, 'TaskGoal')
        return cls(
            goal_id=payload.get('goal_id'),
            description=payload.get('description'),
            source=payload.get('source'),
            timestamp=payload.get('timestamp'),
            confidence=payload.get('confidence'),
            constraints=copy.deepcopy(payload.get('constraints') or {}),
        )


@dataclass(frozen=True)
class PlanStep:
    step_id: str
    plan_id: str
    index: int
    action_name: str
    arguments: dict[str, Any]
    source: str
    timestamp: float
    preconditions: list[ObservedFact] = field(default_factory=list)
    expected_effects: list[ObservedFact] = field(default_factory=list)
    schema_version: int = CONTRACT_SCHEMA_VERSION
    contract_type: str = 'PlanStep'

    def __post_init__(self) -> None:
        _require_string(self.step_id, 'step_id')
        _require_string(self.plan_id, 'plan_id')
        if not isinstance(self.index, int) or self.index < 0:
            raise ContractValidationError('index must be a non-negative integer')
        _require_string(self.action_name, 'action_name')
        if not isinstance(self.arguments, dict):
            raise ContractValidationError('arguments must be an object')
        _normalise_source(self.source, PLAN_STEP_SOURCES)
        _require_timestamp(self.timestamp)
        _require_schema(asdict(self), 'PlanStep')
        _reject_privileged_keys(self.arguments, 'PlanStep arguments')
        for fact in [*self.preconditions, *self.expected_effects]:
            if not isinstance(fact, ObservedFact):
                raise ContractValidationError('PlanStep facts must be ObservedFact')

    def to_dict(self) -> dict[str, Any]:
        return {
            'schema_version': self.schema_version,
            'contract_type': self.contract_type,
            'step_id': self.step_id,
            'plan_id': self.plan_id,
            'index': self.index,
            'action_name': self.action_name,
            'arguments': _deepcopy_jsonable(self.arguments),
            'source': self.source,
            'timestamp': self.timestamp,
            'preconditions': [fact.to_dict() for fact in self.preconditions],
            'expected_effects': [fact.to_dict() for fact in self.expected_effects],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> 'PlanStep':
        payload = _require_mapping(data, 'PlanStep')
        _require_schema(payload, 'PlanStep')
        return cls(
            step_id=payload.get('step_id'),
            plan_id=payload.get('plan_id'),
            index=payload.get('index'),
            action_name=payload.get('action_name'),
            arguments=copy.deepcopy(payload.get('arguments') or {}),
            source=payload.get('source'),
            timestamp=payload.get('timestamp'),
            preconditions=[
                ObservedFact.from_dict(item)
                for item in payload.get('preconditions') or []
            ],
            expected_effects=[
                ObservedFact.from_dict(item)
                for item in payload.get('expected_effects') or []
            ],
        )


@dataclass(frozen=True)
class PrimitiveCommand:
    command_id: str
    plan_step_id: str
    primitive: str
    parameters: dict[str, Any]
    source: str
    timestamp: float
    schema_version: int = CONTRACT_SCHEMA_VERSION
    contract_type: str = 'PrimitiveCommand'

    def __post_init__(self) -> None:
        _require_string(self.command_id, 'command_id')
        _require_string(self.plan_step_id, 'plan_step_id')
        primitive = _require_string(self.primitive, 'primitive')
        if primitive not in PRIMITIVES:
            raise ContractValidationError(f'primitive must be one of {sorted(PRIMITIVES)}')
        if not isinstance(self.parameters, dict):
            raise ContractValidationError('parameters must be an object')
        _normalise_source(self.source, COMMAND_SOURCES)
        _require_timestamp(self.timestamp)
        _require_schema(asdict(self), 'PrimitiveCommand')
        _reject_privileged_keys(self.parameters, 'PrimitiveCommand parameters')

    def to_dict(self) -> dict[str, Any]:
        return _deepcopy_jsonable(asdict(self))

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> 'PrimitiveCommand':
        payload = _require_mapping(data, 'PrimitiveCommand')
        _require_schema(payload, 'PrimitiveCommand')
        forbidden = {'action_vector', 'action_vector_schema_version'} & set(payload)
        if forbidden:
            raise ContractValidationError(
                f'PrimitiveCommand no longer accepts removed direct-control fields: {sorted(forbidden)}'
            )
        allowed = {
            'command_id',
            'plan_step_id',
            'primitive',
            'parameters',
            'source',
            'timestamp',
            'schema_version',
            'contract_type',
        }
        extras = sorted(set(payload) - allowed)
        if extras:
            raise ContractValidationError(
                f'PrimitiveCommand contains unsupported fields: {extras}'
            )
        return cls(
            command_id=payload.get('command_id'),
            plan_step_id=payload.get('plan_step_id'),
            primitive=payload.get('primitive'),
            parameters=copy.deepcopy(payload.get('parameters') or {}),
            source=payload.get('source'),
            timestamp=payload.get('timestamp'),
        )


@dataclass(frozen=True)
class StepResult:
    result_id: str
    command_id: str
    plan_step_id: str
    status: str
    source: str
    timestamp: float
    reason: str = ''
    observed_state_id: str = ''
    facts: list[ObservedFact] = field(default_factory=list)
    schema_version: int = CONTRACT_SCHEMA_VERSION
    contract_type: str = 'StepResult'

    def __post_init__(self) -> None:
        _require_string(self.result_id, 'result_id')
        _require_string(self.command_id, 'command_id')
        _require_string(self.plan_step_id, 'plan_step_id')
        status = _require_string(self.status, 'status')
        if status not in STEP_RESULT_STATUSES:
            raise ContractValidationError(
                f'StepResult status must be one of {sorted(STEP_RESULT_STATUSES)}'
            )
        _normalise_source(self.source, STEP_RESULT_SOURCES)
        _require_timestamp(self.timestamp)
        _require_string(self.reason, 'reason', allow_empty=True)
        _require_string(self.observed_state_id, 'observed_state_id', allow_empty=True)
        _require_schema(asdict(self), 'StepResult')
        for fact in self.facts:
            if not isinstance(fact, ObservedFact):
                raise ContractValidationError('facts must contain ObservedFact')

    def to_dict(self) -> dict[str, Any]:
        return {
            'schema_version': self.schema_version,
            'contract_type': self.contract_type,
            'result_id': self.result_id,
            'command_id': self.command_id,
            'plan_step_id': self.plan_step_id,
            'status': self.status,
            'source': self.source,
            'timestamp': self.timestamp,
            'reason': self.reason,
            'observed_state_id': self.observed_state_id,
            'facts': [fact.to_dict() for fact in self.facts],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> 'StepResult':
        payload = _require_mapping(data, 'StepResult')
        _require_schema(payload, 'StepResult')
        return cls(
            result_id=payload.get('result_id'),
            command_id=payload.get('command_id'),
            plan_step_id=payload.get('plan_step_id'),
            status=payload.get('status'),
            source=payload.get('source'),
            timestamp=payload.get('timestamp'),
            reason=payload.get('reason', ''),
            observed_state_id=payload.get('observed_state_id', ''),
            facts=[
                ObservedFact.from_dict(item)
                for item in payload.get('facts') or []
            ],
        )


CONTRACT_TYPES = {
    'ObservedFact': ObservedFact,
    'ObservedState': ObservedState,
    'TaskGoal': TaskGoal,
    'PlanStep': PlanStep,
    'PrimitiveCommand': PrimitiveCommand,
    'StepResult': StepResult,
}


def contract_from_dict(data: dict[str, Any]) -> Any:
    payload = _require_mapping(data, 'contract')
    contract_type = payload.get('contract_type')
    cls = CONTRACT_TYPES.get(contract_type)
    if cls is None:
        raise ContractValidationError(f'unsupported contract_type {contract_type!r}')
    return cls.from_dict(payload)


def validate_learned_component_output(data: dict[str, Any]) -> Any:
    contract = contract_from_dict(data)
    if isinstance(contract, ObservedFact):
        if contract.source != 'visual_model':
            raise ContractValidationError('learned ObservedFact output must use source visual_model')
        return contract
    if isinstance(contract, TaskGoal):
        if contract.source != 'learned_task_goal':
            raise ContractValidationError('learned TaskGoal output must use source learned_task_goal')
        return contract
    raise ContractValidationError(
        'learned components may output only visual ObservedFact or constrained TaskGoal'
    )
