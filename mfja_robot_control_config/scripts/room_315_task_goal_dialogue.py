#!/usr/bin/env python3
"""Stateful clarification and confirmation for Room 315 TaskGoalDrafts."""

from __future__ import annotations

import copy
import re
from dataclasses import dataclass
from dataclasses import field as dataclass_field
from typing import Any

from room_315_task_goal_parsers import ParserPipeline
from room_315_task_goal_schema import PAYLOAD_FILTERS
from room_315_task_goal_schema import SELECTION_STRATEGIES
from room_315_task_goal_schema import SIDES
from room_315_task_goal_schema import SLOTS
from room_315_task_goal_schema import GoalIssue
from room_315_task_goal_schema import TaskGoalDraft
from room_315_task_goal_schema import normalize_payload_filter
from room_315_task_goal_schema import normalize_selection_strategy
from room_315_task_goal_schema import normalize_side_symbol
from room_315_task_goal_schema import normalize_slot_symbol
from room_315_task_goal_schema import normalize_station_symbol
from room_315_task_goal_validation import DomainValidationResult
from room_315_task_goal_validation import Room315DomainValidator


YES_WORDS = {'yes', 'y', 'confirm', 'confirmed', 'correct', 'proceed', 'go ahead'}
NO_WORDS = {'no', 'n', 'cancel', 'stop', 'revise', 'change'}


@dataclass(frozen=True)
class DialogueTurnResult:
    status: str
    state: 'TaskGoalDialogueState'
    task_goal: Any = None
    draft: TaskGoalDraft | None = None
    errors: tuple[GoalIssue, ...] = ()
    clarifications: tuple[GoalIssue, ...] = ()
    questions: tuple[str, ...] = ()
    confirmation_required: bool = False
    confirmation_prompt: str = ''

    @property
    def ok(self) -> bool:
        return self.status == 'ok' and self.task_goal is not None

    def to_dict(self) -> dict[str, Any]:
        return {
            'status': self.status,
            'ok': self.ok,
            'task_goal': self.task_goal.to_dict() if self.task_goal is not None else None,
            'draft': self.draft.to_dict() if self.draft else None,
            'errors': [issue.to_dict() for issue in self.errors],
            'clarifications': [issue.to_dict() for issue in self.clarifications],
            'questions': list(self.questions),
            'confirmation_required': self.confirmation_required,
            'confirmation_prompt': self.confirmation_prompt,
            'state': self.state.to_dict(),
        }


@dataclass(frozen=True)
class TaskGoalDialogueState:
    pending_draft: TaskGoalDraft | None = None
    attempts: int = 0
    max_attempts: int = 3
    awaiting_confirmation: bool = False
    confirmation_prompt: str = ''
    history: tuple[dict[str, Any], ...] = dataclass_field(default_factory=tuple)

    def to_dict(self) -> dict[str, Any]:
        return {
            'pending_draft': self.pending_draft.to_dict() if self.pending_draft else None,
            'attempts': self.attempts,
            'max_attempts': self.max_attempts,
            'awaiting_confirmation': self.awaiting_confirmation,
            'confirmation_prompt': self.confirmation_prompt,
            'history': [copy.deepcopy(item) for item in self.history],
        }

    def with_update(self, **updates: Any) -> 'TaskGoalDialogueState':
        payload = self.to_dict()
        payload.update(updates)
        pending = payload.get('pending_draft')
        if isinstance(pending, dict):
            pending = TaskGoalDraft.from_dict(pending, strict=False)
        return TaskGoalDialogueState(
            pending_draft=pending,
            attempts=int(payload.get('attempts', self.attempts)),
            max_attempts=int(payload.get('max_attempts', self.max_attempts)),
            awaiting_confirmation=bool(payload.get('awaiting_confirmation', self.awaiting_confirmation)),
            confirmation_prompt=str(payload.get('confirmation_prompt', self.confirmation_prompt) or ''),
            history=tuple(payload.get('history') or ()),
        )


class TaskGoalDialogueManager:
    def __init__(
        self,
        *,
        parser: ParserPipeline | None = None,
        validator: Room315DomainValidator | None = None,
        max_attempts: int = 3,
    ) -> None:
        self.parser = parser or ParserPipeline()
        self.validator = validator or Room315DomainValidator()
        self.max_attempts = max_attempts

    def handle(
        self,
        utterance: str | dict[str, Any],
        *,
        state: TaskGoalDialogueState | None = None,
        timestamp: float = 0.0,
    ) -> DialogueTurnResult:
        state = state or TaskGoalDialogueState(max_attempts=self.max_attempts)
        if state.awaiting_confirmation:
            return self._handle_confirmation(utterance, state=state, timestamp=timestamp)

        if state.pending_draft is not None and isinstance(utterance, str):
            merge_result = merge_clarification_answer(state.pending_draft, utterance)
            if merge_result.status == 'error':
                next_state = _append_history(state, utterance, merge_result.to_dict())
                return DialogueTurnResult(
                    status='error',
                    state=next_state,
                    draft=state.pending_draft,
                    errors=merge_result.issues,
                )
            if merge_result.draft is not None:
                draft = merge_result.draft
            else:
                parsed = self.parser.parse(utterance)
                if not parsed.ok:
                    next_state = _append_history(state, utterance, parsed.to_dict())
                    return DialogueTurnResult(status='error', state=next_state, errors=parsed.issues)
                draft = _merge_drafts(state.pending_draft, parsed.draft)
        else:
            parsed = self.parser.parse(utterance)
            if not parsed.ok:
                next_state = _append_history(state, utterance, parsed.to_dict())
                return DialogueTurnResult(status='error', state=next_state, errors=parsed.issues)
            draft = parsed.draft

        validation = self.validator.validate(
            draft,
            timestamp=timestamp,
            require_confirmation=True,
        )
        return self._result_from_validation(utterance, state, validation)

    def _handle_confirmation(
        self,
        utterance: str | dict[str, Any],
        *,
        state: TaskGoalDialogueState,
        timestamp: float,
    ) -> DialogueTurnResult:
        if not isinstance(utterance, str):
            issue = GoalIssue('confirmation_requires_text', 'Reply yes to confirm or no to revise.', 'confirmation')
            return DialogueTurnResult(status='clarification_required', state=state, clarifications=(issue,), questions=(issue.message,))
        text = _clean(utterance)
        if text in YES_WORDS:
            validation = self.validator.validate(
                state.pending_draft,
                timestamp=timestamp,
                require_confirmation=True,
                confirmed=True,
            )
            cleared = state.with_update(
                pending_draft=None,
                awaiting_confirmation=False,
                confirmation_prompt='',
                attempts=0,
            )
            final_state = _append_history(cleared, utterance, validation.to_dict())
            return DialogueTurnResult(
                status=validation.status,
                state=final_state,
                task_goal=validation.task_goal,
                draft=validation.draft,
                errors=validation.errors,
                clarifications=validation.clarifications,
            )
        if text in NO_WORDS:
            issue = GoalIssue(
                'confirmation_declined',
                'Goal was not finalized. Provide corrected Room 315 goal details.',
                'confirmation',
            )
            next_state = state.with_update(awaiting_confirmation=False, confirmation_prompt='')
            next_state = _append_history(next_state, utterance, {'status': 'confirmation_declined'})
            return DialogueTurnResult(
                status='clarification_required',
                state=next_state,
                draft=state.pending_draft,
                clarifications=(issue,),
                questions=(issue.message,),
            )
        issue = GoalIssue('confirmation_required', 'Reply yes to finalize this goal, or no to revise it.', 'confirmation')
        return DialogueTurnResult(
            status='confirmation_required',
            state=state,
            draft=state.pending_draft,
            clarifications=(issue,),
            questions=(issue.message,),
            confirmation_required=True,
            confirmation_prompt=state.confirmation_prompt,
        )

    def _result_from_validation(
        self,
        utterance: str | dict[str, Any],
        state: TaskGoalDialogueState,
        validation: DomainValidationResult,
    ) -> DialogueTurnResult:
        if validation.status == 'clarification_required':
            attempts = state.attempts + 1
            if attempts > state.max_attempts:
                issue = GoalIssue(
                    'clarification_attempt_limit',
                    'Clarification attempt limit reached; no TaskGoal was finalized.',
                    'dialogue',
                )
                next_state = state.with_update(pending_draft=validation.draft, attempts=attempts)
                next_state = _append_history(next_state, utterance, validation.to_dict())
                return DialogueTurnResult(status='error', state=next_state, draft=validation.draft, errors=(issue,))
            questions = tuple(question_for_issue(issue) for issue in validation.clarifications)
            next_state = state.with_update(
                pending_draft=validation.draft,
                attempts=attempts,
                awaiting_confirmation=False,
                confirmation_prompt='',
            )
            next_state = _append_history(next_state, utterance, validation.to_dict())
            return DialogueTurnResult(
                status='clarification_required',
                state=next_state,
                draft=validation.draft,
                clarifications=validation.clarifications,
                questions=questions,
            )
        if validation.status == 'confirmation_required':
            next_state = state.with_update(
                pending_draft=validation.draft,
                awaiting_confirmation=True,
                confirmation_prompt=validation.confirmation_prompt,
            )
            next_state = _append_history(next_state, utterance, validation.to_dict())
            return DialogueTurnResult(
                status='confirmation_required',
                state=next_state,
                draft=validation.draft,
                confirmation_required=True,
                confirmation_prompt=validation.confirmation_prompt,
                questions=(validation.confirmation_prompt,),
            )
        next_state = state.with_update(pending_draft=None, attempts=0, awaiting_confirmation=False, confirmation_prompt='')
        next_state = _append_history(next_state, utterance, validation.to_dict())
        return DialogueTurnResult(
            status=validation.status,
            state=next_state,
            task_goal=validation.task_goal,
            draft=validation.draft,
            errors=validation.errors,
            clarifications=validation.clarifications,
        )


def merge_clarification_answer(pending: TaskGoalDraft, answer: str) -> Any:
    text = _clean(answer)
    updates: dict[str, Any] = {}
    if text in SIDES:
        updates['side'] = text
    elif text in SELECTION_STRATEGIES:
        updates['selection_strategy'] = text
    elif text in PAYLOAD_FILTERS:
        updates['payload_filter'] = text
    elif text in {'yaskawa', 'staubli', 'kuka'}:
        updates['target_kind'] = pending.target_kind or 'station'
        updates['target_station'] = text
    else:
        slot_match = re.fullmatch(r'(?:slot\s*)?([1-4])', text)
        shuttle_match = re.fullmatch(r'((?:right|left)_shuttle_?[1-4]|[rl][1-4])', text)
        if slot_match:
            updates['target_kind'] = pending.target_kind or 'slot'
            updates['target_slot'] = slot_match.group(1)
        elif shuttle_match:
            updates['target_shuttle'] = shuttle_match.group(1)
            updates['selection_strategy'] = pending.selection_strategy or 'explicit'
            updates['payload_filter'] = pending.payload_filter or 'any'
    if not updates:
        return _MergeResult(status='no_match', draft=None)
    conflicts = []
    for key, value in updates.items():
        existing = getattr(pending, key)
        if existing not in (None, '', value):
            conflicts.append(key)
    if conflicts:
        return _MergeResult(
            status='error',
            issues=(GoalIssue(
                'contradictory_clarification',
                'Clarification contradicts the pending TaskGoalDraft.',
                conflicts[0],
                details={'conflicts': conflicts},
            ),),
        )
    try:
        return _MergeResult(status='ok', draft=pending.merge(**updates))
    except ValueError as exc:
        return _MergeResult(status='error', issues=(GoalIssue('invalid_clarification', str(exc), 'clarification'),))


@dataclass(frozen=True)
class _MergeResult:
    status: str
    draft: TaskGoalDraft | None = None
    issues: tuple[GoalIssue, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {
            'status': self.status,
            'draft': self.draft.to_dict() if self.draft else None,
            'issues': [issue.to_dict() for issue in self.issues],
        }


def question_for_issue(issue: GoalIssue) -> str:
    field = issue.field
    if field == 'side':
        return 'Which Room 315 rail side should be used: right or left?'
    if field == 'selection_strategy':
        return 'Which shuttle selection strategy should be used: nearest, explicit, or any?'
    if field == 'payload_filter':
        return 'Which payload filter should be used: loaded, empty, or any?'
    if field == 'target_shuttle':
        return 'Which exact Room 315 shuttle should be used, for example R1, R2, L1, or L2?'
    if field == 'target_slot':
        return 'Which target slot should be used: 1, 2, 3, or 4?'
    if field == 'target_station':
        return 'Which target station should be used: yaskawa, staubli, or kuka?'
    if field == 'target':
        return 'What is the target: a station or slot?'
    if field == 'goal_type':
        return 'Is this a transport goal or an inspection goal?'
    return issue.message


def _merge_drafts(base: TaskGoalDraft, update: TaskGoalDraft) -> TaskGoalDraft:
    updates = {
        key: value
        for key, value in update.to_dict(include_nulls=True).items()
        if key in {
            'goal_type',
            'selection_strategy',
            'payload_filter',
            'side',
            'target_kind',
            'target_station',
            'target_slot',
            'target_shuttle',
            'inspection_subject',
        } and value is not None
    }
    return base.merge(**updates)


def _append_history(
    state: TaskGoalDialogueState,
    utterance: str | dict[str, Any],
    result: dict[str, Any],
) -> TaskGoalDialogueState:
    entry = {'utterance': copy.deepcopy(utterance), 'result': copy.deepcopy(result)}
    return state.with_update(history=tuple(state.history) + (entry,))


def _clean(text: str) -> str:
    return re.sub(r'\s+', ' ', str(text or '').strip().casefold())


__all__ = [
    'DialogueTurnResult',
    'TaskGoalDialogueManager',
    'TaskGoalDialogueState',
    'merge_clarification_answer',
    'question_for_issue',
]
