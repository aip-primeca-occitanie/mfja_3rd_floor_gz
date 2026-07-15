#!/usr/bin/env python3
"""Deterministic Room 315 domain validation and TaskGoal construction."""

from __future__ import annotations

import copy
import hashlib
import json
from dataclasses import dataclass
from dataclasses import field as dataclass_field
from typing import Any

from room_315_contracts import ContractValidationError
from room_315_contracts import TaskGoal
from room_315_multi_shuttle import all_shuttle_specs
from room_315_multi_shuttle import normalize_shuttle_ref

from room_315_task_goal_schema import GOAL_TYPES
from room_315_task_goal_schema import PAYLOAD_FILTERS
from room_315_task_goal_schema import SELECTION_STRATEGIES
from room_315_task_goal_schema import SIDES
from room_315_task_goal_schema import SLOTS
from room_315_task_goal_schema import STATIONS_BY_SIDE
from room_315_task_goal_schema import GoalIssue
from room_315_task_goal_schema import TaskGoalDraft
from room_315_task_goal_schema import first_slot_for_station
from room_315_task_goal_schema import slots_for_station
from room_315_task_goal_schema import station_for_slot


@dataclass(frozen=True)
class DomainValidationResult:
    status: str
    draft: TaskGoalDraft
    errors: tuple[GoalIssue, ...] = ()
    clarifications: tuple[GoalIssue, ...] = ()
    constraints: dict[str, Any] = dataclass_field(default_factory=dict)
    description: str = ''
    risk_level: str = 'low'
    confirmation_required: bool = False
    confirmation_prompt: str = ''
    task_goal: TaskGoal | None = None

    @property
    def ok(self) -> bool:
        return self.status == 'ok' and self.task_goal is not None

    def to_dict(self) -> dict[str, Any]:
        return {
            'status': self.status,
            'ok': self.ok,
            'draft': self.draft.to_dict(),
            'errors': [issue.to_dict() for issue in self.errors],
            'clarifications': [issue.to_dict() for issue in self.clarifications],
            'constraints': copy.deepcopy(self.constraints),
            'description': self.description,
            'risk_level': self.risk_level,
            'confirmation_required': self.confirmation_required,
            'confirmation_prompt': self.confirmation_prompt,
            'task_goal': self.task_goal.to_dict() if self.task_goal else None,
        }


class Room315DomainValidator:
    """Fail-closed deterministic validator for TaskGoalDraft values."""

    def validate(
        self,
        draft: TaskGoalDraft,
        *,
        timestamp: float = 0.0,
        source: str | None = None,
        goal_id: str | None = None,
        require_confirmation: bool = False,
        confirmed: bool = False,
    ) -> DomainValidationResult:
        errors: list[GoalIssue] = []
        clarifications: list[GoalIssue] = []

        goal_type = draft.goal_type
        if goal_type is None:
            clarifications.append(GoalIssue(
                'missing_goal_type',
                'Specify whether this is a transport or inspection goal.',
                'goal_type',
                GOAL_TYPES,
            ))
        elif goal_type not in GOAL_TYPES:
            errors.append(GoalIssue(
                'unsupported_goal_type',
                f'Unsupported Room 315 goal type {goal_type!r}.',
                'goal_type',
                GOAL_TYPES,
            ))

        side = draft.side
        if side is not None and side not in SIDES:
            errors.append(GoalIssue('unknown_side', f'Unknown Room 315 rail side {side!r}.', 'side', SIDES))

        shuttle = None
        if draft.target_shuttle:
            shuttle = normalize_shuttle_ref(draft.target_shuttle)
            if shuttle is None:
                errors.append(GoalIssue(
                    'unknown_shuttle',
                    f'Unknown Room 315 shuttle {draft.target_shuttle!r}.',
                    'target_shuttle',
                    tuple(spec.gazebo_entity_name for spec in all_shuttle_specs()),
                ))
            elif side is not None and shuttle.side != side:
                errors.append(GoalIssue(
                    'shuttle_side_conflict',
                    f'Shuttle {draft.target_shuttle!r} is on {shuttle.side!r}, not {side!r}.',
                    'target_shuttle',
                    (shuttle.side,),
                ))
            elif side is None:
                side = shuttle.side

        target_slot = draft.target_slot
        if target_slot is not None and target_slot not in SLOTS:
            errors.append(GoalIssue('invalid_slot', 'Room 315 target_slot must be 1, 2, 3, or 4.', 'target_slot', SLOTS))

        if draft.target_station is not None:
            valid_sides = tuple(
                candidate for candidate, stations in STATIONS_BY_SIDE.items()
                if draft.target_station in stations
            )
            if not valid_sides:
                errors.append(GoalIssue(
                    'unknown_station',
                    f'Unknown Room 315 station {draft.target_station!r}.',
                    'target_station',
                    tuple(sorted({station for stations in STATIONS_BY_SIDE.values() for station in stations})),
                ))
            elif side is not None and side not in valid_sides:
                errors.append(GoalIssue(
                    'station_side_mismatch',
                    f'Station {draft.target_station!r} is not on the {side!r} rail.',
                    'target_station',
                    valid_sides,
                ))
            elif side is None and len(valid_sides) == 1:
                side = valid_sides[0]
            elif side is None:
                clarifications.append(GoalIssue(
                    'ambiguous_station_side',
                    f'Station {draft.target_station!r} exists on multiple Room 315 rails; specify a side.',
                    'side',
                    valid_sides,
                ))
            if side is not None and side in valid_sides:
                station_slots = slots_for_station(side, draft.target_station)
                if target_slot in SLOTS:
                    slot_station = station_for_slot(side, target_slot)
                    if slot_station != draft.target_station:
                        errors.append(GoalIssue(
                            'station_slot_mismatch',
                            (
                                f'Slot {target_slot!r} on the {side!r} rail belongs to '
                                f'{slot_station!r}, not {draft.target_station!r}.'
                            ),
                            'target_slot',
                            station_slots,
                        ))
                elif goal_type == 'transport':
                    target_slot = first_slot_for_station(side, draft.target_station) or target_slot

        selection_strategy = draft.selection_strategy
        payload_filter = draft.payload_filter
        if shuttle is not None:
            selection_strategy = 'explicit'
            if payload_filter is None:
                payload_filter = 'any'
        if selection_strategy is not None and selection_strategy not in SELECTION_STRATEGIES:
            errors.append(GoalIssue(
                'unknown_selection_strategy',
                f'Unsupported selection_strategy {selection_strategy!r}.',
                'selection_strategy',
                SELECTION_STRATEGIES,
            ))
        if payload_filter is not None and payload_filter not in PAYLOAD_FILTERS:
            errors.append(GoalIssue(
                'unknown_payload_filter',
                f'Unsupported payload_filter {payload_filter!r}.',
                'payload_filter',
                PAYLOAD_FILTERS,
            ))

        if goal_type == 'transport':
            self._validate_transport(
                draft,
                side=side,
                target_slot=target_slot,
                shuttle=shuttle,
                selection_strategy=selection_strategy,
                payload_filter=payload_filter,
                errors=errors,
                clarifications=clarifications,
            )
        elif goal_type == 'inspection':
            self._validate_inspection(
                draft,
                side=side,
                target_slot=target_slot,
                shuttle=shuttle,
                selection_strategy=selection_strategy,
                payload_filter=payload_filter,
                errors=errors,
                clarifications=clarifications,
            )

        if errors:
            return DomainValidationResult(
                status='error',
                draft=draft,
                errors=tuple(errors),
                clarifications=tuple(clarifications),
            )
        if clarifications:
            return DomainValidationResult(
                status='clarification_required',
                draft=draft,
                clarifications=tuple(clarifications),
            )

        constraints = self._constraints(
            draft,
            side=side,
            target_slot=target_slot,
            shuttle=shuttle,
            selection_strategy=selection_strategy,
            payload_filter=payload_filter,
        )
        description = self._description(constraints)
        risk_level = self.risk_level(constraints)
        confirmation_needed = require_confirmation and risk_level in {'medium', 'high'}
        confirmation_prompt = self.confirmation_prompt(constraints, risk_level) if confirmation_needed else ''
        if confirmation_needed and not confirmed:
            return DomainValidationResult(
                status='confirmation_required',
                draft=draft,
                constraints=constraints,
                description=description,
                risk_level=risk_level,
                confirmation_required=True,
                confirmation_prompt=confirmation_prompt,
            )

        try:
            task_goal = TaskGoal(
                goal_id=goal_id or stable_goal_id(constraints),
                description=description,
                source=source or draft.source,
                timestamp=timestamp,
                confidence=1.0 if draft.confidence is None else draft.confidence,
                constraints=constraints,
            )
        except ContractValidationError as exc:
            return DomainValidationResult(
                status='error',
                draft=draft,
                errors=(GoalIssue('invalid_task_goal', str(exc), 'constraints'),),
                constraints=constraints,
                description=description,
            )
        return DomainValidationResult(
            status='ok',
            draft=draft,
            constraints=constraints,
            description=description,
            risk_level=risk_level,
            task_goal=task_goal,
        )

    def _validate_transport(
        self,
        draft: TaskGoalDraft,
        *,
        side: str | None,
        target_slot: str | None,
        shuttle: Any,
        selection_strategy: str | None,
        payload_filter: str | None,
        errors: list[GoalIssue],
        clarifications: list[GoalIssue],
    ) -> None:
        if side is None:
            clarifications.append(GoalIssue('missing_side', 'Transport goals need a Room 315 rail side.', 'side', SIDES))
        if draft.target_kind not in {'station', 'slot'}:
            if draft.target_kind is None:
                clarifications.append(GoalIssue(
                    'missing_target',
                    'Transport goals need a station or slot target.',
                    'target',
                ))
            else:
                errors.append(GoalIssue(
                    'unsupported_transport_target',
                    'Transport goals support station or slot targets only.',
                    'target_kind',
                    ('station', 'slot'),
                ))
        if draft.target_kind == 'station' and not draft.target_station:
            clarifications.append(GoalIssue(
                'missing_target_station',
                'Station transport goals need yaskawa, staubli, or kuka.',
                'target_station',
                ('yaskawa', 'staubli', 'kuka'),
            ))
        if draft.target_kind == 'slot' and not target_slot:
            clarifications.append(GoalIssue('missing_target_slot', 'Slot transport goals need slot 1, 2, 3, or 4.', 'target_slot', SLOTS))
        if shuttle is None and selection_strategy is None and payload_filter is None:
            clarifications.append(GoalIssue(
                'missing_selection_strategy',
                'Specify nearest, explicitly named shuttle, or any shuttle selection.',
                'selection_strategy',
                SELECTION_STRATEGIES,
            ))
            clarifications.append(GoalIssue(
                'missing_payload_filter',
                'Specify loaded, empty, or any payload filter.',
                'payload_filter',
                PAYLOAD_FILTERS,
            ))
        elif shuttle is None and selection_strategy == 'any' and payload_filter is None:
            clarifications.append(GoalIssue(
                'missing_payload_filter',
                'Specify loaded, empty, or any payload filter.',
                'payload_filter',
                PAYLOAD_FILTERS,
            ))
        if selection_strategy == 'explicit' and shuttle is None:
            clarifications.append(GoalIssue(
                'missing_shuttle',
                'Explicit selection requires a grounded Room 315 shuttle id.',
                'target_shuttle',
            ))

    def _validate_inspection(
        self,
        draft: TaskGoalDraft,
        *,
        side: str | None,
        target_slot: str | None,
        shuttle: Any,
        selection_strategy: str | None,
        payload_filter: str | None,
        errors: list[GoalIssue],
        clarifications: list[GoalIssue],
    ) -> None:
        if draft.target_kind is None:
            if shuttle is None:
                clarifications.append(GoalIssue(
                    'missing_inspection_subject',
                    'Inspection goals need a shuttle, station, slot, rail, or shuttle selection.',
                    'inspection_subject',
                ))
            return
        if draft.target_kind == 'slot' and target_slot is None:
            clarifications.append(GoalIssue('missing_target_slot', 'Slot inspection goals need slot 1, 2, 3, or 4.', 'target_slot', SLOTS))
        if draft.target_kind == 'station' and draft.target_station is None:
            clarifications.append(GoalIssue(
                'missing_target_station',
                'Station inspection goals need yaskawa, staubli, or kuka.',
                'target_station',
                ('yaskawa', 'staubli', 'kuka'),
            ))
        if draft.target_kind in {'slot', 'rail', 'shuttle_selection'} and side is None:
            clarifications.append(GoalIssue('missing_side', 'This inspection target needs a Room 315 side.', 'side', SIDES))
        if draft.target_kind == 'shuttle_selection' and selection_strategy is None and payload_filter is None:
            clarifications.append(GoalIssue(
                'missing_selection_strategy',
                'Selected-shuttle inspection needs nearest, explicit, or any selection.',
                'selection_strategy',
                SELECTION_STRATEGIES,
            ))
        if draft.target_kind not in {'station', 'slot', 'shuttle', 'shuttle_selection', 'rail'}:
            errors.append(GoalIssue(
                'unsupported_inspection_target',
                'Unsupported Room 315 inspection target.',
                'target_kind',
                ('station', 'slot', 'shuttle', 'shuttle_selection', 'rail'),
            ))

    def _constraints(
        self,
        draft: TaskGoalDraft,
        *,
        side: str | None,
        target_slot: str | None,
        shuttle: Any,
        selection_strategy: str | None,
        payload_filter: str | None,
    ) -> dict[str, Any]:
        goal_type = draft.goal_type or 'transport'
        selection_strategy = selection_strategy or 'any'
        payload_filter = payload_filter or 'any'
        constraints: dict[str, Any] = {
            'goal_type': goal_type,
            'selection_strategy': selection_strategy,
            'payload_filter': payload_filter,
        }
        if side:
            constraints['side'] = side
        if draft.target_kind:
            constraints['target_kind'] = draft.target_kind
        if draft.target_station:
            constraints['target_station'] = draft.target_station
        if target_slot:
            constraints['target_slot'] = target_slot
        if shuttle is not None:
            constraints['target_shuttle'] = shuttle.gazebo_entity_name
        if goal_type == 'inspection':
            subject = self._inspection_subject(draft, side=side, shuttle=shuttle, selection_strategy=selection_strategy, payload_filter=payload_filter)
            if subject:
                constraints['inspection_subject'] = subject
        constraints.update(_legacy_compatibility(selection_strategy, payload_filter, shuttle is not None))
        return {key: value for key, value in constraints.items() if value not in (None, '')}

    def _inspection_subject(
        self,
        draft: TaskGoalDraft,
        *,
        side: str | None,
        shuttle: Any,
        selection_strategy: str,
        payload_filter: str,
    ) -> str:
        if draft.inspection_subject:
            return draft.inspection_subject
        if shuttle is not None:
            return shuttle.gazebo_entity_name
        if draft.target_kind == 'station' and side and draft.target_station:
            return f'{side}:station:{draft.target_station}'
        if draft.target_kind == 'slot' and side and draft.target_slot:
            return f'{side}:slot:{draft.target_slot}'
        if draft.target_kind == 'rail' and side:
            return f'{side}:rail'
        if draft.target_kind == 'shuttle_selection' and side:
            return f'{side}:shuttle_selection:{selection_strategy}:{payload_filter}'
        return ''

    def _description(self, constraints: dict[str, Any]) -> str:
        if constraints.get('goal_type') == 'inspection':
            return f'inspect {constraints.get("inspection_subject", "room315_system")}'
        side = constraints.get('side')
        target = (
            (
                f'station {constraints["target_station"]} / slot {constraints["target_slot"]}'
                if constraints.get('target_slot')
                else f'station {constraints["target_station"]}'
            )
            if constraints.get('target_kind') == 'station'
            else f'slot {constraints.get("target_slot")}'
        )
        if constraints.get('target_shuttle'):
            selection = constraints['target_shuttle']
        else:
            selection = f'{constraints.get("selection_strategy", "any")} / {constraints.get("payload_filter", "any")} shuttle'
        return f'transport {selection} on {side} rail to {target}'

    def risk_level(self, constraints: dict[str, Any]) -> str:
        if constraints.get('goal_type') == 'transport':
            return 'high'
        if constraints.get('target_kind') in {'shuttle', 'shuttle_selection', 'rail'}:
            return 'medium'
        return 'low'

    def confirmation_prompt(self, constraints: dict[str, Any], risk_level: str) -> str:
        return (
            f'Confirm {risk_level}-risk Room 315 goal: '
            f'{self._description(constraints)}. Reply yes to finalize.'
        )


def _legacy_compatibility(
    selection_strategy: str,
    payload_filter: str,
    explicit: bool,
) -> dict[str, Any]:
    if explicit:
        shuttle_selection = 'explicit'
    elif selection_strategy == 'nearest':
        shuttle_selection = 'nearest'
    elif payload_filter == 'loaded':
        shuttle_selection = 'loaded'
    elif payload_filter == 'empty':
        shuttle_selection = 'empty'
    else:
        shuttle_selection = 'any'
    payload_required: Any = None
    if payload_filter == 'loaded':
        payload_required = True
    elif payload_filter == 'empty':
        payload_required = False
    result = {'shuttle_selection': shuttle_selection}
    if payload_required is not None:
        result['payload_required'] = payload_required
    return result


def stable_goal_id(constraints: dict[str, Any]) -> str:
    encoded = json.dumps(constraints, sort_keys=True, separators=(',', ':')).encode('utf-8')
    return f'room315-task-goal-{hashlib.sha256(encoded).hexdigest()[:12]}'


__all__ = [
    'DomainValidationResult',
    'Room315DomainValidator',
    'stable_goal_id',
]
