#!/usr/bin/env python3
"""Deterministic Room 315 TaskGoal builder and strict JSON parser."""

from __future__ import annotations

import copy
import hashlib
import json
import re
import sys
from dataclasses import dataclass
from dataclasses import field as dataclass_field
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_contracts import COMMAND_FIELD_NAMES
from room_315_contracts import CONTRACT_SCHEMA_VERSION
from room_315_contracts import PRIVILEGED_FIELD_NAMES
from room_315_contracts import ContractValidationError
from room_315_contracts import TaskGoal
from room_315_multi_shuttle import SIDES
from room_315_multi_shuttle import normalize_shuttle_ref
from room_315_multi_shuttle import normalize_side
from room_315_task_goal_dialogue import TaskGoalDialogueManager
from room_315_task_goal_dialogue import TaskGoalDialogueState
from room_315_task_goal_parsers import ConversationalIntentGatewayParser
from room_315_task_goal_parsers import DeterministicEnglishParser
from room_315_task_goal_parsers import LocalSemanticModelAdapter
from room_315_task_goal_parsers import ParserPipeline
from room_315_task_goal_parsers import RegexFallbackParser
from room_315_task_goal_parsers import StructuredFormParser
from room_315_task_goal_parsers import parse_task_goal_draft
from room_315_task_goal_schema import PAYLOAD_FILTERS
from room_315_task_goal_schema import SELECTION_STRATEGIES
from room_315_task_goal_schema import GOAL_TYPES
from room_315_task_goal_schema import SLOTS
from room_315_task_goal_schema import STATIONS_BY_SIDE
from room_315_task_goal_schema import STATION_ALIASES
from room_315_task_goal_schema import TASK_GOAL_DRAFT_CONTRACT_TYPE
from room_315_task_goal_schema import TaskGoalDraft
from room_315_task_goal_schema import optional_text as _optional_text
from room_315_task_goal_schema import slug_text as _slug_text
from room_315_task_goal_schema import strict_model_draft_from_json
from room_315_task_goal_validation import Room315DomainValidator


SELECTIONS = ('loaded', 'empty', 'nearest', 'explicit')
SELECTION_STRATEGIES_COMPAT = SELECTION_STRATEGIES
PAYLOAD_FILTERS_COMPAT = PAYLOAD_FILTERS
MODEL_ALLOWED_TOP_LEVEL_KEYS = frozenset({
    'schema_version',
    'contract_type',
    'goal_id',
    'description',
    'confidence',
    'goal_type',
    'task',
    'intent',
    'side',
    'shuttle_selection',
    'selection',
    'selector',
    'shuttle',
    'shuttle_id',
    'target_shuttle',
    'payload_required',
    'target',
    'target_kind',
    'target_station',
    'station',
    'target_slot',
    'slot',
    'inspection_subject',
    'constraints',
})
MODEL_ALLOWED_REQUEST_KEYS = frozenset({
    'goal_type',
    'task',
    'intent',
    'side',
    'shuttle_selection',
    'selection',
    'selector',
    'shuttle',
    'shuttle_id',
    'target_shuttle',
    'payload_required',
    'target',
    'target_kind',
    'target_station',
    'station',
    'target_slot',
    'slot',
    'inspection_subject',
})
MODEL_ALLOWED_TARGET_KEYS = frozenset({
    'id',
    'kind',
    'name',
    'slot',
    'station',
    'target_kind',
    'target_slot',
    'target_station',
    'type',
})
MODEL_BLOCKED_KEYS = (
    PRIVILEGED_FIELD_NAMES
    | COMMAND_FIELD_NAMES
    | frozenset({
        'deadline_s',
        'editable_safety_constraints',
        'min_headway_blocks',
        'pddl',
        'plan',
        'plan_steps',
        'plans',
        'primitive_command',
        'primitive_commands',
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
        payload = {
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
class TaskGoalBuildResult:
    status: str
    task_goal: TaskGoal | None = None
    errors: tuple[GoalIssue, ...] = ()
    clarifications: tuple[GoalIssue, ...] = ()
    normalized_request: dict[str, Any] = dataclass_field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.status == 'ok' and self.task_goal is not None

    def to_dict(self) -> dict[str, Any]:
        payload = {
            'status': self.status,
            'ok': self.ok,
            'errors': [issue.to_dict() for issue in self.errors],
            'clarifications': [issue.to_dict() for issue in self.clarifications],
            'normalized_request': copy.deepcopy(self.normalized_request),
            'task_goal': self.task_goal.to_dict() if self.task_goal is not None else None,
        }
        return payload


def build_task_goal(
    request: str | dict[str, Any],
    *,
    timestamp: float = 0.0,
    source: str = 'human',
    confidence: float | None = None,
    goal_id: str | None = None,
) -> TaskGoalBuildResult:
    """Build a high-level TaskGoal from deterministic Room 315 request fields."""

    parsed = parse_task_goal_draft(request)
    if not parsed.ok:
        return _result_for_schema_issues(
            parsed.issues,
            normalized_request={'parser': parsed.parser_name},
        )
    draft = parsed.draft
    if confidence is not None:
        draft = draft.merge(confidence=confidence)
    validation = Room315DomainValidator().validate(
        draft,
        timestamp=timestamp,
        source=source,
        goal_id=goal_id,
    )
    return _result_from_validation(validation, normalized_request={
        'parser': parsed.parser_name,
        'draft': draft.to_dict(),
        'parse_trace': copy.deepcopy(parsed.raw_output.get('trace')) if isinstance(parsed.raw_output, dict) else None,
    })


def parse_model_task_goal_json(
    model_output: str | bytes,
    *,
    timestamp: float = 0.0,
    confidence: float | None = None,
    goal_id: str | None = None,
) -> TaskGoalBuildResult:
    """Parse strict local-model TaskGoalDraft JSON into a constrained TaskGoal."""

    parsed = strict_model_draft_from_json(model_output)
    if not parsed.ok:
        return _result_for_schema_issues(
            parsed.issues,
            normalized_request={'parser': parsed.parser_name},
        )
    draft = parsed.draft
    if confidence is not None:
        draft = draft.merge(confidence=confidence)
    validation = Room315DomainValidator().validate(
        draft,
        timestamp=timestamp,
        source='learned_task_goal',
        goal_id=goal_id,
    )
    return _result_from_validation(validation, normalized_request={
        'parser': parsed.parser_name,
        'draft': draft.to_dict(),
        'raw_model_output': copy.deepcopy(parsed.raw_output),
    })


def _build_transport_goal(
    normalized: dict[str, Any],
    *,
    timestamp: float,
    source: str,
    confidence: float | None,
    goal_id: str | None,
) -> TaskGoalBuildResult:
    issues: list[GoalIssue] = []
    side = _normalize_optional_side(normalized.get('side'), issues)
    shuttle = _normalize_optional_shuttle(normalized.get('target_shuttle'), side=side, issues=issues)
    if shuttle is not None:
        side = shuttle['side']
    target_kind, target_value = _target_from_normalized(normalized, issues)
    if target_kind == 'station':
        station_side = _side_for_station(target_value, side=side, issues=issues)
        if station_side:
            side = station_side
    elif target_kind == 'slot' and side is None:
        issues.append(GoalIssue(
            code='missing_side',
            message='Slot transport goals need a rail side unless a shuttle is explicitly named',
            field='side',
            options=SIDES,
        ))

    selection = _normalize_selection(normalized, explicit=shuttle is not None, issues=issues)
    if selection is None and shuttle is None:
        issues.append(GoalIssue(
            code='missing_shuttle_selection',
            message='Choose loaded, empty, nearest, or an explicitly named shuttle',
            field='shuttle_selection',
            options=SELECTIONS,
        ))
    if selection == 'explicit' and shuttle is None:
        issues.append(GoalIssue(
            code='missing_shuttle',
            message='Explicit shuttle selection needs a grounded Room 315 shuttle id',
            field='target_shuttle',
        ))
    if side is None:
        issues.append(GoalIssue(
            code='missing_side',
            message='Transport goals need a grounded Room 315 rail side',
            field='side',
            options=SIDES,
        ))

    if issues:
        return _result_for_issues(issues, normalized_request=normalized)

    constraints: dict[str, Any] = {
        'goal_type': 'transport',
        'side': side,
        'target_kind': target_kind,
        'shuttle_selection': selection,
    }
    if target_kind == 'station':
        constraints['target_station'] = target_value
    elif target_kind == 'slot':
        constraints['target_slot'] = target_value
    if shuttle is not None:
        constraints['target_shuttle'] = shuttle['entity']
    payload_required = _payload_requirement(normalized, selection)
    if payload_required is not None:
        constraints['payload_required'] = payload_required

    description = _transport_description(constraints)
    return _make_goal_result(
        constraints,
        description=description,
        timestamp=timestamp,
        source=source,
        confidence=confidence,
        goal_id=goal_id,
        normalized_request=normalized,
    )


def _build_inspection_goal(
    normalized: dict[str, Any],
    *,
    timestamp: float,
    source: str,
    confidence: float | None,
    goal_id: str | None,
) -> TaskGoalBuildResult:
    issues: list[GoalIssue] = []
    side = _normalize_optional_side(normalized.get('side'), issues)
    shuttle = _normalize_optional_shuttle(normalized.get('target_shuttle'), side=side, issues=issues)
    if shuttle is not None:
        side = shuttle['side']
    selection = _normalize_selection(normalized, explicit=shuttle is not None, issues=issues)
    target_kind, target_value = _inspection_target_from_normalized(
        normalized,
        side=side,
        shuttle=shuttle,
        selection=selection,
        issues=issues,
    )
    if target_kind == 'station':
        station_side = _side_for_station(target_value, side=side, issues=issues)
        if station_side:
            side = station_side
    elif target_kind == 'slot' and side is None:
        issues.append(GoalIssue(
            code='missing_side',
            message='Slot inspection goals need a rail side unless a shuttle is explicitly named',
            field='side',
            options=SIDES,
        ))
    elif target_kind == 'rail' and side is None:
        issues.append(GoalIssue(
            code='missing_side',
            message='Rail inspection goals need a side',
            field='side',
            options=SIDES,
        ))

    if target_kind == 'shuttle_selection' and side is None:
        issues.append(GoalIssue(
            code='missing_side',
            message='Selected-shuttle inspection goals need a side',
            field='side',
            options=SIDES,
        ))
    if target_kind is None:
        issues.append(GoalIssue(
            code='missing_inspection_subject',
            message='Inspection goals need a grounded shuttle, station, slot, rail, or shuttle selection',
            field='inspection_subject',
        ))
    if issues:
        return _result_for_issues(issues, normalized_request=normalized)

    constraints: dict[str, Any] = {
        'goal_type': 'inspection',
        'target_kind': target_kind,
    }
    if side is not None:
        constraints['side'] = side
    if shuttle is not None:
        constraints['target_shuttle'] = shuttle['entity']
    if target_kind == 'station':
        constraints['target_station'] = target_value
        constraints['inspection_subject'] = f'{side}:station:{target_value}'
    elif target_kind == 'slot':
        constraints['target_slot'] = target_value
        constraints['inspection_subject'] = f'{side}:slot:{target_value}'
    elif target_kind == 'rail':
        constraints['inspection_subject'] = f'{side}:rail'
    elif target_kind == 'shuttle':
        constraints['inspection_subject'] = shuttle['entity']
    elif target_kind == 'shuttle_selection':
        constraints['shuttle_selection'] = selection
        constraints['inspection_subject'] = f'{side}:shuttle_selection:{selection}'
    payload_required = _payload_requirement(normalized, selection)
    if payload_required is not None:
        constraints['payload_required'] = payload_required

    return _make_goal_result(
        constraints,
        description=_inspection_description(constraints),
        timestamp=timestamp,
        source=source,
        confidence=confidence,
        goal_id=goal_id,
        normalized_request=normalized,
    )


def _request_to_normalized(request: str | dict[str, Any]) -> tuple[dict[str, Any], list[GoalIssue]]:
    if isinstance(request, str):
        return _request_from_text(request), []
    if isinstance(request, dict):
        return _request_from_mapping(request), []
    return {}, [
        GoalIssue(
            code='invalid_request_type',
            message='TaskGoal request must be a string or object',
            field='request',
        )
    ]


def _request_from_mapping(request: dict[str, Any]) -> dict[str, Any]:
    payload = copy.deepcopy(request)
    if isinstance(payload.get('constraints'), dict):
        constraints = copy.deepcopy(payload['constraints'])
        for key, value in constraints.items():
            payload.setdefault(key, value)
    target = payload.get('target')
    if isinstance(target, dict):
        target_kind = target.get('kind') or target.get('type') or target.get('target_kind')
        if target_kind:
            payload.setdefault('target_kind', target_kind)
        for key in ('station', 'target_station'):
            if key in target:
                payload.setdefault('target_station', target[key])
        for key in ('slot', 'target_slot'):
            if key in target:
                payload.setdefault('target_slot', target[key])
        if 'id' in target or 'name' in target:
            target_id = target.get('id') or target.get('name')
            if _slot_symbol(target_id):
                payload.setdefault('target_slot', target_id)
            elif _station_symbol(target_id):
                payload.setdefault('target_station', target_id)
    elif isinstance(target, str):
        if _slot_symbol(target):
            payload.setdefault('target_kind', 'slot')
            payload.setdefault('target_slot', target)
        else:
            station = _station_symbol(target)
            if station:
                payload.setdefault('target_kind', 'station')
                payload.setdefault('target_station', station)

    return {
        'goal_type': payload.get('goal_type') or payload.get('task') or payload.get('intent'),
        'side': payload.get('side'),
        'target_shuttle': (
            payload.get('target_shuttle')
            or payload.get('shuttle_id')
            or payload.get('shuttle')
        ),
        'shuttle_selection': (
            payload.get('shuttle_selection')
            or payload.get('selection')
            or payload.get('selector')
        ),
        'payload_required': payload.get('payload_required'),
        'target_kind': payload.get('target_kind'),
        'target_station': payload.get('target_station') or payload.get('station'),
        'target_slot': payload.get('target_slot') or payload.get('slot'),
        'inspection_subject': payload.get('inspection_subject'),
        '_raw': payload,
    }


def _request_from_text(text: str) -> dict[str, Any]:
    normalized = _normalize_text(text)
    result: dict[str, Any] = {
        'goal_type': _goal_type_from_text(normalized),
        'side': _side_from_text(normalized),
        'target_shuttle': _shuttle_from_text(normalized),
        'shuttle_selection': _selection_from_text(normalized),
        'payload_required': _payload_from_text(normalized),
        'target_kind': '',
        'target_station': '',
        'target_slot': '',
        'inspection_subject': '',
        '_raw': {'language': text},
    }
    slot_side, slot = _slot_from_text(normalized)
    if slot:
        result['target_kind'] = 'slot'
        result['target_slot'] = slot
        if slot_side and not result['side']:
            result['side'] = slot_side
    station = _station_from_text(normalized)
    if station and not slot:
        result['target_kind'] = 'station'
        result['target_station'] = station
    if result['goal_type'] == 'inspection' and not result['target_kind']:
        if re.search(r'\b(?:rail|line|track)\b', normalized):
            result['target_kind'] = 'rail'
        elif result['target_shuttle']:
            result['target_kind'] = 'shuttle'
        elif result['shuttle_selection']:
            result['target_kind'] = 'shuttle_selection'
    return result


def _model_payload_to_request(payload: dict[str, Any]) -> dict[str, Any]:
    if isinstance(payload.get('constraints'), dict):
        request = copy.deepcopy(payload['constraints'])
        for key in ('goal_type', 'description', 'side', 'target_shuttle'):
            if key in payload:
                request.setdefault(key, payload[key])
        return request
    return copy.deepcopy(payload)


def _normalize_goal_type(value: Any) -> str:
    text = _slug_text(value)
    if text in {'transport', 'move', 'send', 'route', 'bring', 'deliver', 'transfer'}:
        return 'transport'
    if text in {'inspection', 'inspect', 'check', 'observe', 'look', 'look_at'}:
        return 'inspection'
    return ''


def _normalize_optional_side(value: Any, issues: list[GoalIssue]) -> str | None:
    text = _optional_text(value)
    if not text:
        return None
    try:
        return normalize_side(text)
    except ValueError:
        issues.append(GoalIssue(
            code='unknown_side',
            message=f'Unknown Room 315 rail side {text!r}',
            field='side',
            options=SIDES,
        ))
        return None


def _normalize_optional_shuttle(
    value: Any,
    *,
    side: str | None,
    issues: list[GoalIssue],
) -> dict[str, str] | None:
    text = _optional_text(value)
    if not text:
        return None
    spec = normalize_shuttle_ref(text, side=side)
    if spec is None:
        issues.append(GoalIssue(
            code='unknown_shuttle',
            message=f'Unknown Room 315 shuttle {text!r}',
            field='target_shuttle',
            options=tuple(spec.gazebo_entity_name for spec in _all_shuttle_specs()),
        ))
        return None
    return {
        'side': spec.side,
        'short_id': spec.short_id,
        'entity': spec.gazebo_entity_name,
        'shuttle_id': spec.shuttle_id,
    }


def _normalize_selection(
    normalized: dict[str, Any],
    *,
    explicit: bool,
    issues: list[GoalIssue],
) -> str | None:
    raw = normalized.get('shuttle_selection')
    text = _slug_text(raw)
    if not text and explicit:
        return 'explicit'
    if not text:
        payload = normalized.get('payload_required')
        if payload is True:
            return 'loaded'
        if payload is False:
            return 'empty'
        return None
    if text in {'loaded', 'carrying', 'with_payload', 'with_load'}:
        return 'loaded'
    if text in {'empty', 'unloaded', 'without_payload', 'without_load'}:
        return 'empty'
    if text in {'nearest', 'closest'}:
        return 'nearest'
    if text in {'explicit', 'named', 'shuttle'}:
        return 'explicit'
    issues.append(GoalIssue(
        code='unknown_shuttle_selection',
        message=f'Unsupported shuttle selection {raw!r}',
        field='shuttle_selection',
        options=SELECTIONS,
    ))
    return None


def _target_from_normalized(
    normalized: dict[str, Any],
    issues: list[GoalIssue],
) -> tuple[str, str]:
    target_kind = _slug_text(normalized.get('target_kind'))
    slot = _slot_symbol(normalized.get('target_slot'))
    station = _station_symbol(normalized.get('target_station'))
    if slot and station:
        issues.append(GoalIssue(
            code='ambiguous_target',
            message='Choose either a station or slot target, not both',
            field='target',
        ))
        return '', ''
    if target_kind in {'slot', 'station'}:
        if target_kind == 'slot':
            if not slot:
                issues.append(GoalIssue(
                    code='missing_target_slot',
                    message='Slot transport goals need target_slot 1, 2, 3, or 4',
                    field='target_slot',
                    options=SLOTS,
                ))
            return 'slot', slot
        if not station:
            issues.append(GoalIssue(
                code='missing_target_station',
                message='Station transport goals need yaskawa, staubli, or kuka',
                field='target_station',
                options=tuple(sorted(STATION_ALIASES.values())),
            ))
        return 'station', station
    if slot:
        return 'slot', slot
    if station:
        return 'station', station
    issues.append(GoalIssue(
        code='missing_target',
        message='Transport goals need a grounded Room 315 station or slot target',
        field='target',
    ))
    return '', ''


def _inspection_target_from_normalized(
    normalized: dict[str, Any],
    *,
    side: str | None,
    shuttle: dict[str, str] | None,
    selection: str | None,
    issues: list[GoalIssue],
) -> tuple[str | None, str]:
    target_kind = _slug_text(normalized.get('target_kind'))
    subject = _optional_text(normalized.get('inspection_subject'))
    if subject:
        subject_shuttle = normalize_shuttle_ref(subject, side=side)
        if subject_shuttle is not None:
            return 'shuttle', subject_shuttle.gazebo_entity_name
        slot = _slot_symbol(subject)
        if slot:
            return 'slot', slot
        station = _station_symbol(subject)
        if station:
            return 'station', station
        if _slug_text(subject) in {'rail', 'track', 'line'}:
            return 'rail', ''
        issues.append(GoalIssue(
            code='unknown_inspection_subject',
            message=f'Unknown Room 315 inspection subject {subject!r}',
            field='inspection_subject',
        ))
        return None, ''
    if shuttle is not None:
        return 'shuttle', shuttle['entity']
    if target_kind == 'rail':
        return 'rail', ''
    if target_kind == 'slot' or normalized.get('target_slot'):
        slot = _slot_symbol(normalized.get('target_slot'))
        if not slot:
            issues.append(GoalIssue(
                code='missing_target_slot',
                message='Slot inspection goals need target_slot 1, 2, 3, or 4',
                field='target_slot',
                options=SLOTS,
            ))
        return 'slot', slot
    if target_kind == 'station' or normalized.get('target_station'):
        station = _station_symbol(normalized.get('target_station'))
        if not station:
            issues.append(GoalIssue(
                code='missing_target_station',
                message='Station inspection goals need yaskawa, staubli, or kuka',
                field='target_station',
                options=tuple(sorted(STATION_ALIASES.values())),
            ))
        return 'station', station
    if selection in {'loaded', 'empty', 'nearest'}:
        return 'shuttle_selection', selection
    return None, ''


def _side_for_station(
    station: str,
    *,
    side: str | None,
    issues: list[GoalIssue],
) -> str | None:
    if not station:
        return side
    valid_sides = tuple(
        candidate
        for candidate in SIDES
        if station in STATIONS_BY_SIDE[candidate]
    )
    if not valid_sides:
        issues.append(GoalIssue(
            code='unknown_station',
            message=f'Unknown Room 315 station {station!r}',
            field='target_station',
            options=tuple(sorted(STATION_ALIASES.values())),
        ))
        return side
    if side is not None:
        if side not in valid_sides:
            issues.append(GoalIssue(
                code='station_side_mismatch',
                message=f'Station {station!r} is not grounded on the {side!r} rail',
                field='target_station',
                options=valid_sides,
            ))
        return side
    if len(valid_sides) == 1:
        return valid_sides[0]
    issues.append(GoalIssue(
        code='ambiguous_station_side',
        message=f'Station {station!r} exists on multiple Room 315 rails; specify a side',
        field='side',
        options=valid_sides,
    ))
    return None


def _payload_requirement(normalized: dict[str, Any], selection: str | None) -> bool | None:
    if normalized.get('payload_required') is True:
        return True
    if normalized.get('payload_required') is False:
        return False
    if selection == 'loaded':
        return True
    if selection == 'empty':
        return False
    return None


def _make_goal_result(
    constraints: dict[str, Any],
    *,
    description: str,
    timestamp: float,
    source: str,
    confidence: float | None,
    goal_id: str | None,
    normalized_request: dict[str, Any],
) -> TaskGoalBuildResult:
    filtered_constraints = {
        key: value
        for key, value in constraints.items()
        if value is not None and value != ''
    }
    try:
        task_goal = TaskGoal(
            goal_id=goal_id or _stable_goal_id(filtered_constraints),
            description=description,
            source=source,
            timestamp=timestamp,
            confidence=1.0 if confidence is None else confidence,
            constraints=filtered_constraints,
        )
    except ContractValidationError as exc:
        return _error_result(
            'invalid_task_goal',
            str(exc),
            normalized_request=normalized_request,
        )
    return TaskGoalBuildResult(
        status='ok',
        task_goal=task_goal,
        normalized_request=copy.deepcopy(normalized_request),
    )


def _transport_description(constraints: dict[str, Any]) -> str:
    selection = constraints.get('target_shuttle') or f'{constraints["shuttle_selection"]} shuttle'
    side = constraints['side']
    if constraints['target_kind'] == 'station':
        target = f'station {constraints["target_station"]}'
    else:
        target = f'slot {constraints["target_slot"]}'
    return f'transport {selection} on {side} rail to {target}'


def _inspection_description(constraints: dict[str, Any]) -> str:
    subject = constraints.get('inspection_subject', '')
    return f'inspect {subject}'


def _stable_goal_id(constraints: dict[str, Any]) -> str:
    encoded = json.dumps(constraints, sort_keys=True, separators=(',', ':')).encode('utf-8')
    digest = hashlib.sha256(encoded).hexdigest()[:12]
    return f'room315-task-goal-{digest}'


def _result_for_issues(
    issues: list[GoalIssue],
    *,
    normalized_request: dict[str, Any],
) -> TaskGoalBuildResult:
    errors = tuple(issue for issue in issues if not issue.code.startswith('missing_') and 'ambiguous' not in issue.code)
    clarifications = tuple(issue for issue in issues if issue not in errors)
    status = 'error' if errors else 'clarification_required'
    return TaskGoalBuildResult(
        status=status,
        errors=errors,
        clarifications=clarifications,
        normalized_request=copy.deepcopy(normalized_request),
    )


def _result_for_schema_issues(
    issues: tuple[Any, ...],
    *,
    normalized_request: dict[str, Any],
) -> TaskGoalBuildResult:
    converted = [
        GoalIssue(
            code=str(issue.code),
            message=str(issue.message),
            field=str(getattr(issue, 'field', '') or ''),
            options=tuple(getattr(issue, 'options', ()) or ()),
            details=copy.deepcopy(getattr(issue, 'details', {}) or {}),
        )
        for issue in issues
    ]
    return _result_for_issues(converted, normalized_request=normalized_request)


def _result_from_validation(
    validation: Any,
    *,
    normalized_request: dict[str, Any],
) -> TaskGoalBuildResult:
    if validation.status == 'ok':
        return TaskGoalBuildResult(
            status='ok',
            task_goal=validation.task_goal,
            normalized_request=copy.deepcopy({
                **normalized_request,
                'constraints': validation.constraints,
                'risk_level': validation.risk_level,
            }),
        )
    if validation.status == 'confirmation_required':
        issue = GoalIssue(
            'confirmation_required',
            validation.confirmation_prompt,
            'confirmation',
            details={'risk_level': validation.risk_level},
        )
        return TaskGoalBuildResult(
            status='confirmation_required',
            clarifications=(issue,),
            normalized_request=copy.deepcopy({
                **normalized_request,
                'constraints': validation.constraints,
                'risk_level': validation.risk_level,
            }),
        )
    errors = [
        GoalIssue(
            code=str(issue.code),
            message=str(issue.message),
            field=str(getattr(issue, 'field', '') or ''),
            options=tuple(getattr(issue, 'options', ()) or ()),
            details=copy.deepcopy(getattr(issue, 'details', {}) or {}),
        )
        for issue in validation.errors
    ]
    clarifications = [
        GoalIssue(
            code=str(issue.code),
            message=str(issue.message),
            field=str(getattr(issue, 'field', '') or ''),
            options=tuple(getattr(issue, 'options', ()) or ()),
            details=copy.deepcopy(getattr(issue, 'details', {}) or {}),
        )
        for issue in validation.clarifications
    ]
    return TaskGoalBuildResult(
        status='error' if errors else 'clarification_required',
        errors=tuple(errors),
        clarifications=tuple(clarifications),
        normalized_request=copy.deepcopy(normalized_request),
    )


def _error_result(
    code: str,
    message: str,
    *,
    field: str = '',
    options: tuple[str, ...] = (),
    normalized_request: dict[str, Any] | None = None,
    details: dict[str, Any] | None = None,
) -> TaskGoalBuildResult:
    return TaskGoalBuildResult(
        status='error',
        errors=(GoalIssue(code, message, field, options, details or {}),),
        normalized_request=copy.deepcopy(normalized_request or {}),
    )


def _clarification_result(
    code: str,
    message: str,
    *,
    field: str = '',
    options: tuple[str, ...] = (),
    normalized_request: dict[str, Any] | None = None,
) -> TaskGoalBuildResult:
    return TaskGoalBuildResult(
        status='clarification_required',
        clarifications=(GoalIssue(code, message, field, options),),
        normalized_request=copy.deepcopy(normalized_request or {}),
    )


def _validate_model_request_shape(payload: dict[str, Any]) -> TaskGoalBuildResult | None:
    request_payloads: list[tuple[str, dict[str, Any]]] = []
    if isinstance(payload.get('constraints'), dict):
        request_payloads.append(('constraints', payload['constraints']))
    else:
        request_payloads.append(('model_output', payload))
    for context, request in request_payloads:
        allowed = MODEL_ALLOWED_REQUEST_KEYS
        if context == 'model_output':
            allowed = MODEL_ALLOWED_TOP_LEVEL_KEYS
        unknown = sorted(set(request) - allowed)
        if unknown:
            return _error_result(
                'unknown_model_field',
                'Model JSON contains fields outside the strict TaskGoal request schema',
                field=f'{context}.{unknown[0]}',
                details={'unknown_fields': unknown},
            )
        target = request.get('target')
        if isinstance(target, dict):
            target_unknown = sorted(set(target) - MODEL_ALLOWED_TARGET_KEYS)
            if target_unknown:
                return _error_result(
                    'unknown_model_field',
                    'Model JSON target contains fields outside the strict target schema',
                    field=f'{context}.target.{target_unknown[0]}',
                    details={'unknown_fields': target_unknown},
                )
    return None


def _blocked_paths(value: Any, blocked: frozenset[str], path: str = '$') -> list[str]:
    paths: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            key_text = str(key)
            child_path = f'{path}.{key_text}'
            if key_text in blocked:
                paths.append(child_path)
            paths.extend(_blocked_paths(child, blocked, child_path))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            paths.extend(_blocked_paths(child, blocked, f'{path}[{index}]'))
    return paths


def _all_shuttle_specs() -> tuple[Any, ...]:
    from room_315_multi_shuttle import all_shuttle_specs

    return tuple(all_shuttle_specs())


def _goal_type_from_text(normalized: str) -> str:
    if re.search(r'\b(?:inspect|check|observe|look\s+at)\b', normalized):
        return 'inspection'
    if re.search(r'\b(?:move|send|route|bring|deliver|transport|transfer)\b', normalized):
        return 'transport'
    return ''


def _side_from_text(normalized: str) -> str:
    if re.search(r'\bright\b', normalized):
        return 'right'
    if re.search(r'\bleft\b', normalized):
        return 'left'
    return ''


def _selection_from_text(normalized: str) -> str:
    if re.search(r'\b(?:nearest|closest)\b', normalized):
        return 'nearest'
    if re.search(r'\b(?:loaded|carrying|with\s+(?:a\s+)?(?:payload|part|load))\b', normalized):
        return 'loaded'
    if re.search(r'\b(?:empty|unloaded|without\s+(?:a\s+)?(?:payload|part|load))\b', normalized):
        return 'empty'
    return ''


def _payload_from_text(normalized: str) -> bool | None:
    selection = _selection_from_text(normalized)
    if selection == 'loaded':
        return True
    if selection == 'empty':
        return False
    return None


def _shuttle_from_text(normalized: str) -> str:
    match = re.search(
        r'\b(?P<shuttle>(?:room315_)?(?:right|left)_shuttle_?[1-4]|[rl][1-4])\b',
        normalized,
    )
    return match.group('shuttle') if match else ''


def _slot_from_text(normalized: str) -> tuple[str, str]:
    match = re.search(
        r'\b(?:(?P<side_a>right|left)\s+)?slot\s*[_-]?\s*(?P<slot>[1-4])'
        r'(?:\s+(?:on|for)\s+(?P<side_b>right|left))?\b',
        normalized,
    )
    if not match:
        return '', ''
    return match.group('side_a') or match.group('side_b') or '', match.group('slot')


def _station_from_text(normalized: str) -> str:
    for token in ('staubli', 'kuka', 'yaskawa'):
        if re.search(rf'\b{token}\b', normalized):
            return token
    return ''


def _normalize_text(text: str) -> str:
    lowered = str(text or '').casefold()
    lowered = lowered.replace('-', '_')
    lowered = re.sub(r'[^\w\s:]', ' ', lowered)
    return re.sub(r'\s+', ' ', lowered).strip()


def _station_symbol(value: Any) -> str:
    text = _slug_text(value)
    return STATION_ALIASES.get(text, '')


def _slot_symbol(value: Any) -> str:
    text = _slug_text(value)
    if text.startswith('slot_'):
        text = text.removeprefix('slot_')
    elif text.startswith('slot'):
        text = text.removeprefix('slot')
    return text if text in SLOTS else ''


__all__ = [
    'ConversationalIntentGatewayParser',
    'DeterministicEnglishParser',
    'GOAL_TYPES',
    'GoalIssue',
    'LocalSemanticModelAdapter',
    'PAYLOAD_FILTERS',
    'PAYLOAD_FILTERS_COMPAT',
    'ParserPipeline',
    'RegexFallbackParser',
    'SELECTION_STRATEGIES',
    'SELECTION_STRATEGIES_COMPAT',
    'SELECTIONS',
    'SLOTS',
    'STATIONS_BY_SIDE',
    'StructuredFormParser',
    'TASK_GOAL_DRAFT_CONTRACT_TYPE',
    'TaskGoalBuildResult',
    'TaskGoalDialogueManager',
    'TaskGoalDialogueState',
    'TaskGoalDraft',
    'build_task_goal',
    'parse_task_goal_draft',
    'parse_model_task_goal_json',
]
