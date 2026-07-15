#!/usr/bin/env python3
"""Strict offline TaskGoal draft schema for Room 315 goal understanding."""

from __future__ import annotations

import copy
import hashlib
import json
import math
import re
from dataclasses import dataclass
from dataclasses import field as dataclass_field
from typing import Any

from room_315_contracts import COMMAND_FIELD_NAMES
from room_315_contracts import CONTRACT_SCHEMA_VERSION
from room_315_contracts import PRIVILEGED_FIELD_NAMES


TASK_GOAL_DRAFT_CONTRACT_TYPE = 'TaskGoalDraft'
GOAL_TYPES = ('transport', 'inspection')
SELECTION_STRATEGIES = ('nearest', 'explicit', 'any')
PAYLOAD_FILTERS = ('loaded', 'empty', 'any')
SIDES = ('right', 'left')
SLOTS = ('1', '2', '3', '4')
TARGET_KINDS = ('station', 'slot', 'shuttle', 'shuttle_selection', 'rail')
STATIONS_BY_SIDE = {
    'right': ('yaskawa', 'staubli'),
    'left': ('yaskawa', 'kuka'),
}
STATION_ALIASES = {
    'yaskawa': 'yaskawa',
    'yaskawa_station': 'yaskawa',
    'staubli': 'staubli',
    'staubli_station': 'staubli',
    'kuka': 'kuka',
    'kuka_station': 'kuka',
}

DRAFT_FIELDS = frozenset({
    'schema_version',
    'contract_type',
    'draft_id',
    'goal_type',
    'selection_strategy',
    'payload_filter',
    'side',
    'target_kind',
    'target_station',
    'target_slot',
    'target_shuttle',
    'inspection_subject',
    'confidence',
    'source',
    'language',
    'raw',
})

MODEL_DRAFT_FIELDS = frozenset({
    'schema_version',
    'contract_type',
    'goal_type',
    'selection_strategy',
    'payload_filter',
    'side',
    'target_kind',
    'target_station',
    'target_slot',
    'target_shuttle',
    'inspection_subject',
    'confidence',
})

BLOCKED_DRAFT_KEYS = (
    PRIVILEGED_FIELD_NAMES
    | COMMAND_FIELD_NAMES
    | frozenset({
        'actions',
        'device_command',
        'device_commands',
        'editable_safety_constraints',
        'pddl',
        'pddl_domain',
        'pddl_goal',
        'pddl_problem',
        'plan',
        'plan_steps',
        'plans',
        'primitive_command',
        'primitive_commands',
        'rail_command',
        'safety_constraint',
        'safety_constraints',
        'steps',
    })
)


@dataclass(frozen=True)
class GoalIssue:
    code: str
    message: str
    field: str = ''
    options: tuple[str, ...] = ()
    details: dict[str, Any] = dataclass_field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            'code': self.code,
            'message': self.message,
        }
        if self.field:
            payload['field'] = self.field
        if self.options:
            payload['options'] = list(self.options)
        if self.details:
            payload['details'] = copy.deepcopy(self.details)
        return payload


@dataclass(frozen=True)
class TaskGoalDraft:
    goal_type: str | None = None
    selection_strategy: str | None = None
    payload_filter: str | None = None
    side: str | None = None
    target_kind: str | None = None
    target_station: str | None = None
    target_slot: str | None = None
    target_shuttle: str | None = None
    inspection_subject: str | None = None
    confidence: float | None = None
    source: str = 'human'
    language: str | None = None
    raw: dict[str, Any] = dataclass_field(default_factory=dict)
    draft_id: str = ''
    schema_version: int = CONTRACT_SCHEMA_VERSION
    contract_type: str = TASK_GOAL_DRAFT_CONTRACT_TYPE

    def __post_init__(self) -> None:
        if self.schema_version != CONTRACT_SCHEMA_VERSION:
            raise ValueError(f'TaskGoalDraft schema_version must be {CONTRACT_SCHEMA_VERSION}')
        if self.contract_type != TASK_GOAL_DRAFT_CONTRACT_TYPE:
            raise ValueError(f'contract_type must be {TASK_GOAL_DRAFT_CONTRACT_TYPE!r}')
        _validate_choice_or_none(self.goal_type, GOAL_TYPES, 'goal_type')
        _validate_choice_or_none(self.selection_strategy, SELECTION_STRATEGIES, 'selection_strategy')
        _validate_choice_or_none(self.payload_filter, PAYLOAD_FILTERS, 'payload_filter')
        _validate_choice_or_none(self.side, SIDES, 'side')
        _validate_choice_or_none(self.target_kind, TARGET_KINDS, 'target_kind')
        if self.target_slot is not None and self.target_slot not in SLOTS:
            raise ValueError('target_slot must be one of 1, 2, 3, or 4')
        if self.target_station is not None and self.target_station not in set(STATION_ALIASES.values()):
            raise ValueError('target_station must be yaskawa, staubli, or kuka')
        if self.confidence is not None:
            confidence = float(self.confidence)
            if not math.isfinite(confidence) or confidence < 0.0 or confidence > 1.0:
                raise ValueError('confidence must be in [0, 1]')
        if not isinstance(self.raw, dict):
            raise ValueError('raw must be an object')
        if not self.draft_id:
            object.__setattr__(self, 'draft_id', stable_draft_id(self.to_dict(include_id=False)))

    def merge(self, **updates: Any) -> 'TaskGoalDraft':
        payload = self.to_dict(include_nulls=True)
        payload.update(updates)
        payload.pop('draft_id', None)
        return TaskGoalDraft.from_dict(payload, strict=False)

    def to_dict(self, *, include_nulls: bool = True, include_id: bool = True) -> dict[str, Any]:
        payload: dict[str, Any] = {
            'schema_version': self.schema_version,
            'contract_type': self.contract_type,
            'goal_type': self.goal_type,
            'selection_strategy': self.selection_strategy,
            'payload_filter': self.payload_filter,
            'side': self.side,
            'target_kind': self.target_kind,
            'target_station': self.target_station,
            'target_slot': self.target_slot,
            'target_shuttle': self.target_shuttle,
            'inspection_subject': self.inspection_subject,
            'confidence': self.confidence,
            'source': self.source,
            'language': self.language,
            'raw': copy.deepcopy(self.raw),
        }
        if include_id:
            payload['draft_id'] = self.draft_id
        if not include_nulls:
            payload = {key: value for key, value in payload.items() if value is not None}
        return payload

    @classmethod
    def from_dict(cls, payload: dict[str, Any], *, strict: bool = True) -> 'TaskGoalDraft':
        if not isinstance(payload, dict):
            raise ValueError('TaskGoalDraft payload must be an object')
        allowed = DRAFT_FIELDS if strict else DRAFT_FIELDS | {'constraints'}
        unknown = sorted(set(payload) - allowed)
        if unknown:
            raise ValueError(f'unknown TaskGoalDraft fields: {unknown}')
        return cls(
            draft_id=_optional_text(payload.get('draft_id')),
            goal_type=normalize_goal_type(payload.get('goal_type')),
            selection_strategy=normalize_selection_strategy(payload.get('selection_strategy')),
            payload_filter=normalize_payload_filter(payload.get('payload_filter')),
            side=normalize_side_symbol(payload.get('side')),
            target_kind=normalize_target_kind(payload.get('target_kind')),
            target_station=normalize_station_symbol(payload.get('target_station')),
            target_slot=normalize_slot_symbol(payload.get('target_slot')),
            target_shuttle=_optional_text(payload.get('target_shuttle')) or None,
            inspection_subject=_optional_text(payload.get('inspection_subject')) or None,
            confidence=payload.get('confidence'),
            source=_optional_text(payload.get('source')) or 'human',
            language=_optional_text(payload.get('language')) or None,
            raw=copy.deepcopy(payload.get('raw') or {}),
            schema_version=int(payload.get('schema_version', CONTRACT_SCHEMA_VERSION)),
            contract_type=_optional_text(payload.get('contract_type')) or TASK_GOAL_DRAFT_CONTRACT_TYPE,
        )


@dataclass(frozen=True)
class DraftParseResult:
    status: str
    draft: TaskGoalDraft | None = None
    issues: tuple[GoalIssue, ...] = ()
    parser_name: str = ''
    raw_output: Any = None

    @property
    def ok(self) -> bool:
        return self.status == 'ok' and self.draft is not None

    def to_dict(self) -> dict[str, Any]:
        return {
            'status': self.status,
            'ok': self.ok,
            'parser_name': self.parser_name,
            'draft': self.draft.to_dict() if self.draft else None,
            'issues': [issue.to_dict() for issue in self.issues],
            'raw_output': copy.deepcopy(self.raw_output),
        }


def strict_model_draft_from_json(model_output: str | bytes) -> DraftParseResult:
    try:
        payload = json.loads(model_output)
    except json.JSONDecodeError as exc:
        return DraftParseResult(
            status='error',
            issues=(GoalIssue(
                code='invalid_json',
                message=f'Model output must be strict draft JSON: {exc.msg}',
                field='model_output',
                details={'line': exc.lineno, 'column': exc.colno},
            ),),
            parser_name='local_semantic_model',
        )
    if not isinstance(payload, dict):
        return DraftParseResult(
            status='error',
            issues=(GoalIssue('invalid_json_type', 'Model output must be a JSON object', 'model_output'),),
            parser_name='local_semantic_model',
            raw_output=payload,
        )
    blocked = blocked_paths(payload)
    if blocked:
        return DraftParseResult(
            status='error',
            issues=(GoalIssue(
                code='forbidden_model_field',
                message='Local semantic model may output TaskGoalDraft JSON only',
                field=blocked[0],
                details={'blocked_paths': blocked[:10]},
            ),),
            parser_name='local_semantic_model',
            raw_output=payload,
        )
    unknown = sorted(set(payload) - MODEL_DRAFT_FIELDS)
    if unknown:
        return DraftParseResult(
            status='error',
            issues=(GoalIssue(
                code='unknown_model_field',
                message='Model draft JSON contains fields outside the strict draft schema',
                field=unknown[0],
                details={'unknown_fields': unknown},
            ),),
            parser_name='local_semantic_model',
            raw_output=payload,
        )
    if payload.get('contract_type') not in (None, TASK_GOAL_DRAFT_CONTRACT_TYPE):
        return DraftParseResult(
            status='error',
            issues=(GoalIssue(
                code='unsupported_contract_type',
                message='Model JSON may only declare contract_type TaskGoalDraft',
                field='contract_type',
            ),),
            parser_name='local_semantic_model',
            raw_output=payload,
        )
    if 'schema_version' in payload and payload.get('schema_version') != CONTRACT_SCHEMA_VERSION:
        return DraftParseResult(
            status='error',
            issues=(GoalIssue(
                code='unsupported_schema_version',
                message=f'Model JSON schema_version must be {CONTRACT_SCHEMA_VERSION}',
                field='schema_version',
            ),),
            parser_name='local_semantic_model',
            raw_output=payload,
        )
    try:
        draft = TaskGoalDraft.from_dict({
            **payload,
            'source': 'learned_task_goal',
            'raw': {'model_output': payload},
        }, strict=False)
    except ValueError as exc:
        return DraftParseResult(
            status='error',
            issues=(GoalIssue('invalid_draft', str(exc), 'model_output'),),
            parser_name='local_semantic_model',
            raw_output=payload,
        )
    return DraftParseResult(status='ok', draft=draft, parser_name='local_semantic_model', raw_output=payload)


def stable_draft_id(payload: dict[str, Any]) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(',', ':'), default=str)
    return f'room315-task-goal-draft-{hashlib.sha256(encoded.encode("utf-8")).hexdigest()[:12]}'


def blocked_paths(value: Any, path: str = '$') -> list[str]:
    paths: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            key_text = str(key)
            child_path = f'{path}.{key_text}'
            if key_text in BLOCKED_DRAFT_KEYS:
                paths.append(child_path)
            paths.extend(blocked_paths(child, child_path))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            paths.extend(blocked_paths(child, f'{path}[{index}]'))
    return paths


def normalize_goal_type(value: Any) -> str | None:
    text = slug_text(value)
    if not text:
        return None
    if text in {'transport', 'move', 'send', 'route', 'bring', 'deliver', 'transfer'}:
        return 'transport'
    if text in {'inspection', 'inspect', 'check', 'observe', 'look', 'look_at'}:
        return 'inspection'
    return text


def normalize_selection_strategy(value: Any) -> str | None:
    text = slug_text(value)
    if not text:
        return None
    if text in {'nearest', 'closest'}:
        return 'nearest'
    if text in {'explicit', 'named', 'specific', 'shuttle'}:
        return 'explicit'
    if text in {'any', 'whatever', 'unspecified'}:
        return 'any'
    return text


def normalize_payload_filter(value: Any) -> str | None:
    if value is True:
        return 'loaded'
    if value is False:
        return 'empty'
    text = slug_text(value)
    if not text:
        return None
    if text in {'loaded', 'carrying', 'with_payload', 'with_load', 'with_part'}:
        return 'loaded'
    if text in {'empty', 'unloaded', 'without_payload', 'without_load', 'without_part'}:
        return 'empty'
    if text in {'any', 'either', 'unspecified'}:
        return 'any'
    return text


def normalize_side_symbol(value: Any) -> str | None:
    text = slug_text(value)
    if not text:
        return None
    if text in {'right', 'r'}:
        return 'right'
    if text in {'left', 'l'}:
        return 'left'
    return text


def normalize_target_kind(value: Any) -> str | None:
    text = slug_text(value)
    if not text:
        return None
    if text in {'station', 'slot', 'shuttle', 'shuttle_selection', 'rail'}:
        return text
    if text in {'track', 'line'}:
        return 'rail'
    return text


def normalize_station_symbol(value: Any) -> str | None:
    text = slug_text(value)
    if not text:
        return None
    return STATION_ALIASES.get(text, text)


def normalize_slot_symbol(value: Any) -> str | None:
    text = slug_text(value)
    if not text:
        return None
    if text.startswith('slot_'):
        text = text.removeprefix('slot_')
    elif text.startswith('slot'):
        text = text.removeprefix('slot')
    return text


def slug_text(value: Any) -> str:
    return re.sub(r'[^a-z0-9_]+', '_', str(value or '').strip().casefold()).strip('_')


def _optional_text(value: Any) -> str:
    if value is None:
        return ''
    return str(value).strip()


def _validate_choice_or_none(value: str | None, allowed: tuple[str, ...], name: str) -> None:
    if value is not None and value not in allowed:
        raise ValueError(f'{name} must be one of {allowed}')


__all__ = [
    'GOAL_TYPES',
    'GoalIssue',
    'PAYLOAD_FILTERS',
    'SELECTION_STRATEGIES',
    'SIDES',
    'SLOTS',
    'STATIONS_BY_SIDE',
    'TASK_GOAL_DRAFT_CONTRACT_TYPE',
    'TaskGoalDraft',
    'DraftParseResult',
    'blocked_paths',
    'strict_model_draft_from_json',
]
