#!/usr/bin/env python3
"""Offline parser interfaces for Room 315 TaskGoalDraft understanding."""

from __future__ import annotations

import copy
import re
from typing import Any
from typing import Callable
from typing import Protocol

from room_315_task_goal_fusion import EvidenceAwareFusionResolver
from room_315_task_goal_fusion import ExplicitFactExtractor
from room_315_task_goal_fusion import uses_confirmed_context
from room_315_task_goal_semantic import LocalSemanticBackend
from room_315_task_goal_semantic import LocalSemanticModelConfig
from room_315_task_goal_semantic import SemanticParseEnvelope
from room_315_task_goal_semantic import get_default_semantic_backend
from room_315_task_goal_semantic import strict_semantic_envelope_from_json
from room_315_task_goal_schema import CONTRACT_SCHEMA_VERSION
from room_315_task_goal_schema import PAYLOAD_FILTERS
from room_315_task_goal_schema import SELECTION_STRATEGIES
from room_315_task_goal_schema import SIDES
from room_315_task_goal_schema import SLOTS
from room_315_task_goal_schema import STATION_ALIASES
from room_315_task_goal_schema import TASK_GOAL_DRAFT_CONTRACT_TYPE
from room_315_task_goal_schema import DraftParseResult
from room_315_task_goal_schema import GoalIssue
from room_315_task_goal_schema import TaskGoalDraft
from room_315_task_goal_schema import blocked_paths
from room_315_task_goal_schema import normalize_payload_filter
from room_315_task_goal_schema import normalize_goal_type as _goal_type
from room_315_task_goal_schema import normalize_selection_strategy
from room_315_task_goal_schema import normalize_side_symbol
from room_315_task_goal_schema import normalize_slot_symbol
from room_315_task_goal_schema import normalize_station_symbol
from room_315_task_goal_schema import normalize_target_kind
from room_315_task_goal_schema import optional_text as _optional_text
from room_315_task_goal_schema import slug_text
from room_315_task_goal_schema import strict_model_draft_from_json

TERMINAL_GATEWAY_ISSUES = {
    'ambiguous_explicit_conflict',
    'missing_reference_context',
    'unsupported_language',
}


class TaskGoalDraftParser(Protocol):
    parser_name: str

    def parse(self, request: Any) -> DraftParseResult:
        ...


class StructuredFormParser:
    parser_name = 'structured_form'

    _allowed = frozenset({
        'schema_version',
        'contract_type',
        'goal_type',
        'task',
        'intent',
        'selection_strategy',
        'payload_filter',
        'shuttle_selection',
        'selection',
        'selector',
        'payload_required',
        'side',
        'target',
        'target_kind',
        'target_station',
        'station',
        'target_slot',
        'slot',
        'target_shuttle',
        'shuttle',
        'shuttle_id',
        'inspection_subject',
        'confidence',
        'constraints',
    })

    def parse(self, request: Any) -> DraftParseResult:
        if not isinstance(request, dict):
            return DraftParseResult(
                status='error',
                issues=(GoalIssue('invalid_structured_form', 'Structured parser requires a mapping.', 'request'),),
                parser_name=self.parser_name,
            )
        blocked = blocked_paths(request)
        if blocked:
            return DraftParseResult(
                status='error',
                issues=(GoalIssue(
                    'forbidden_field',
                    'Structured TaskGoal form may not include commands, PDDL, plans, or safety constraints.',
                    blocked[0],
                    details={'blocked_paths': blocked[:10]},
                ),),
                parser_name=self.parser_name,
                raw_output=request,
            )
        payload = copy.deepcopy(request)
        if isinstance(payload.get('constraints'), dict):
            constraints = copy.deepcopy(payload['constraints'])
            blocked_constraints = blocked_paths(constraints)
            if blocked_constraints:
                return DraftParseResult(
                    status='error',
                    issues=(GoalIssue(
                        'forbidden_field',
                        'TaskGoal constraints may not include commands, PDDL, plans, or safety constraints.',
                        f'constraints{blocked_constraints[0][1:]}',
                        details={'blocked_paths': blocked_constraints[:10]},
                    ),),
                    parser_name=self.parser_name,
                    raw_output=request,
                )
            payload = {**constraints, **{key: value for key, value in payload.items() if key != 'constraints'}}
        unknown = sorted(set(payload) - self._allowed)
        if unknown:
            return DraftParseResult(
                status='error',
                issues=(GoalIssue(
                    'unknown_structured_field',
                    'Structured TaskGoal form contains fields outside the offline parser schema.',
                    unknown[0],
                    details={'unknown_fields': unknown},
                ),),
                parser_name=self.parser_name,
                raw_output=request,
            )

        target = payload.get('target')
        if isinstance(target, dict):
            payload.setdefault('target_kind', target.get('kind') or target.get('target_kind') or target.get('type'))
            payload.setdefault('target_station', target.get('station') or target.get('target_station'))
            payload.setdefault('target_slot', target.get('slot') or target.get('target_slot'))
            payload.setdefault('target_shuttle', target.get('shuttle') or target.get('target_shuttle'))
        elif isinstance(target, str):
            slot = normalize_slot_symbol(target)
            station = normalize_station_symbol(target)
            if slot in SLOTS:
                payload.setdefault('target_kind', 'slot')
                payload.setdefault('target_slot', slot)
            elif station in STATION_ALIASES.values():
                payload.setdefault('target_kind', 'station')
                payload.setdefault('target_station', station)
        subject = _optional_text(payload.get('inspection_subject'))
        canonicalize_subject = False
        if subject and not payload.get('target_kind'):
            subject_slot = normalize_slot_symbol(subject)
            subject_station = normalize_station_symbol(subject)
            if subject_slot in SLOTS:
                payload.setdefault('target_kind', 'slot')
                payload.setdefault('target_slot', subject_slot)
                canonicalize_subject = True
            elif subject_station in STATION_ALIASES.values():
                payload.setdefault('target_kind', 'station')
                payload.setdefault('target_station', subject_station)
                canonicalize_subject = True
            elif slug_text(subject) in {'rail', 'track', 'line'}:
                payload.setdefault('target_kind', 'rail')
                canonicalize_subject = True
            elif re.fullmatch(r'(?:room315_)?(?:right|left)_shuttle_?[1-4]|[rl][1-4]', subject.casefold()):
                payload.setdefault('target_kind', 'shuttle')
                payload.setdefault('target_shuttle', subject)
                canonicalize_subject = True

        if not payload.get('target_kind'):
            if payload.get('target_station') or payload.get('station'):
                payload.setdefault('target_kind', 'station')
            elif payload.get('target_slot') or payload.get('slot'):
                payload.setdefault('target_kind', 'slot')
            elif payload.get('target_shuttle') or payload.get('shuttle_id') or payload.get('shuttle'):
                payload.setdefault('target_kind', 'shuttle')

        selection_strategy = normalize_selection_strategy(payload.get('selection_strategy'))
        payload_filter = normalize_payload_filter(payload.get('payload_filter'))
        legacy_selection = slug_text(payload.get('shuttle_selection') or payload.get('selection') or payload.get('selector'))
        if legacy_selection:
            if legacy_selection in {'nearest', 'closest'}:
                selection_strategy = selection_strategy or 'nearest'
            elif legacy_selection in {'explicit', 'named', 'shuttle'}:
                selection_strategy = selection_strategy or 'explicit'
            elif legacy_selection in {'loaded', 'carrying', 'with_payload'}:
                payload_filter = payload_filter or 'loaded'
                selection_strategy = selection_strategy or 'any'
            elif legacy_selection in {'empty', 'unloaded', 'without_payload'}:
                payload_filter = payload_filter or 'empty'
                selection_strategy = selection_strategy or 'any'
            elif legacy_selection == 'any':
                selection_strategy = selection_strategy or 'any'
        if payload.get('payload_required') is True:
            payload_filter = payload_filter or 'loaded'
        elif payload.get('payload_required') is False:
            payload_filter = payload_filter or 'empty'
        target_shuttle = _optional_text(
            payload.get('target_shuttle') or payload.get('shuttle_id') or payload.get('shuttle')
        )
        if target_shuttle:
            selection_strategy = selection_strategy or 'explicit'
            payload_filter = payload_filter or 'any'
        slot = normalize_slot_symbol(payload.get('target_slot') or payload.get('slot'))
        if slot is not None and slot not in SLOTS:
            return DraftParseResult(
                status='error',
                issues=(GoalIssue('invalid_slot', 'Room 315 slot must be 1, 2, 3, or 4.', 'target_slot', SLOTS),),
                parser_name=self.parser_name,
                raw_output=request,
            )
        try:
            draft = TaskGoalDraft(
                goal_type=_goal_type(payload.get('goal_type') or payload.get('task') or payload.get('intent')),
                selection_strategy=selection_strategy,
                payload_filter=payload_filter,
                side=normalize_side_symbol(payload.get('side')),
                target_kind=normalize_target_kind(payload.get('target_kind')),
                target_station=normalize_station_symbol(payload.get('target_station') or payload.get('station')),
                target_slot=slot,
                target_shuttle=target_shuttle or None,
                inspection_subject=None if canonicalize_subject else (_optional_text(payload.get('inspection_subject')) or None),
                confidence=payload.get('confidence'),
                raw={'structured_form': request},
            )
        except ValueError as exc:
            return DraftParseResult(
                status='error',
                issues=(GoalIssue('invalid_draft', str(exc), 'request'),),
                parser_name=self.parser_name,
                raw_output=request,
            )
        return DraftParseResult(status='ok', draft=draft, parser_name=self.parser_name, raw_output=request)


class DeterministicEnglishParser:
    parser_name = 'deterministic_english'

    def parse(self, request: Any) -> DraftParseResult:
        if not isinstance(request, str):
            return DraftParseResult(
                status='error',
                issues=(GoalIssue('invalid_text_request', 'English parser requires text.', 'request'),),
                parser_name=self.parser_name,
            )
        if _has_non_english_letters(request):
            return DraftParseResult(
                status='error',
                issues=(GoalIssue(
                    'unsupported_language',
                    'Task-goal understanding is English-only in production.',
                    'request',
                ),),
                parser_name=self.parser_name,
                raw_output=request,
            )
        text = _normalize_text(request)
        if not text:
            return DraftParseResult(
                status='error',
                issues=(GoalIssue('empty_request', 'Task-goal request is empty.', 'request'),),
                parser_name=self.parser_name,
                raw_output=request,
            )
        goal_type = _goal_type_from_text(text)
        side = _side_from_text(text)
        shuttle = _shuttle_from_text(text)
        selection_strategy = _selection_strategy_from_text(text, shuttle=bool(shuttle))
        payload_filter = _payload_filter_from_text(text)
        target_kind = None
        target_station = _station_from_text(text)
        slot_issue, slot_side, target_slot = _slot_from_text(text)
        if slot_issue is not None:
            return DraftParseResult(
                status='error',
                issues=(slot_issue,),
                parser_name=self.parser_name,
                raw_output=request,
            )
        if slot_side and not side:
            side = slot_side
        if goal_type == 'inspection':
            if shuttle:
                target_kind = 'shuttle'
            elif target_slot:
                target_kind = 'slot'
            elif target_station:
                target_kind = 'station'
            elif re.search(r'\b(?:rail|track|line)\b', text):
                target_kind = 'rail'
            elif selection_strategy or payload_filter:
                target_kind = 'shuttle_selection'
        else:
            if target_slot:
                target_kind = 'slot'
            elif target_station:
                target_kind = 'station'
        try:
            draft = TaskGoalDraft(
                goal_type=goal_type,
                selection_strategy=selection_strategy,
                payload_filter=payload_filter,
                side=side,
                target_kind=target_kind,
                target_station=target_station,
                target_slot=target_slot,
                target_shuttle=shuttle,
                language='en',
                raw={'request': request},
            )
        except ValueError as exc:
            return DraftParseResult(
                status='error',
                issues=(GoalIssue('invalid_draft', str(exc), 'request'),),
                parser_name=self.parser_name,
                raw_output=request,
            )
        if not any([draft.goal_type, draft.target_station, draft.target_slot, draft.target_shuttle, draft.selection_strategy, draft.payload_filter]):
            return DraftParseResult(
                status='error',
                issues=(GoalIssue('parse_no_match', 'No grounded Room 315 task-goal fields were found.', 'request'),),
                parser_name=self.parser_name,
                raw_output=request,
            )
        return DraftParseResult(status='ok', draft=draft, parser_name=self.parser_name, raw_output=request)


class RegexFallbackParser(DeterministicEnglishParser):
    parser_name = 'regex_fallback'


class LocalSemanticModelAdapter:
    parser_name = 'local_semantic_model'

    def __init__(self, model: Callable[[str], str] | None = None) -> None:
        self.model = model

    def parse(self, request: Any) -> DraftParseResult:
        if not isinstance(request, str):
            return DraftParseResult(
                status='error',
                issues=(GoalIssue('invalid_model_request', 'Local semantic model adapter requires text.', 'request'),),
                parser_name=self.parser_name,
            )
        if self.model is None:
            return DraftParseResult(
                status='error',
                issues=(GoalIssue(
                    'local_model_unavailable',
                    'No local semantic model callable is configured.',
                    'model',
                ),),
                parser_name=self.parser_name,
            )
        try:
            output = self.model(request)
        except Exception as exc:  # pragma: no cover - defensive adapter boundary
            return DraftParseResult(
                status='error',
                issues=(GoalIssue('local_model_error', str(exc), 'model'),),
                parser_name=self.parser_name,
            )
        result = strict_model_draft_from_json(output)
        return DraftParseResult(
            status=result.status,
            draft=result.draft,
            issues=result.issues,
            parser_name=self.parser_name,
            raw_output=result.raw_output,
        )


class ConversationalIntentGatewayParser:
    parser_name = 'conversational_intent_gateway'

    def __init__(
        self,
        *,
        backend: LocalSemanticBackend | None = None,
        config: LocalSemanticModelConfig | None = None,
        config_path: str | None = None,
        extractor: ExplicitFactExtractor | None = None,
        resolver: EvidenceAwareFusionResolver | None = None,
        shadow_mode: bool | None = None,
        deterministic_only: bool | None = None,
    ) -> None:
        self.config = config or LocalSemanticModelConfig.from_file(config_path)
        self.backend = backend or get_default_semantic_backend(config_path)
        self.extractor = extractor or ExplicitFactExtractor()
        self.resolver = resolver or EvidenceAwareFusionResolver()
        self.shadow_mode = self.config.shadow_mode if shadow_mode is None else shadow_mode
        self.deterministic_only = self.config.deterministic_only if deterministic_only is None else deterministic_only

    def parse(
        self,
        request: Any,
        *,
        confirmed_draft: TaskGoalDraft | None = None,
    ) -> DraftParseResult:
        if not isinstance(request, str):
            return DraftParseResult(
                status='error',
                issues=(GoalIssue('invalid_text_request', 'Conversational parser requires English text.', 'request'),),
                parser_name=self.parser_name,
            )
        if _has_non_english_letters(request):
            return DraftParseResult(
                status='error',
                issues=(GoalIssue(
                    'unsupported_language',
                    'Task-goal understanding is English-only in production.',
                    'request',
                ),),
                parser_name=self.parser_name,
                raw_output=request,
            )
        if not _normalize_text(request):
            return DraftParseResult(
                status='error',
                issues=(GoalIssue('empty_request', 'Task-goal request is empty.', 'request'),),
                parser_name=self.parser_name,
                raw_output=request,
            )

        explicit_facts = self.extractor.extract(request)
        if self.extractor.contains_unresolved_reference(request, has_context=confirmed_draft is not None):
            trace = self.resolver.resolve(
                request_text=request,
                explicit_facts=explicit_facts,
                semantic_envelope=None,
                backend_result=None,
                confirmed_draft=confirmed_draft,
                fallback_reason='unresolved_reference',
                prompt_schema_version=self.config.prompt_schema_version,
            ).trace
            return DraftParseResult(
                status='error',
                issues=(GoalIssue(
                    'missing_reference_context',
                    'The request uses a reference such as it, there, or the same shuttle without confirmed context.',
                    'reference',
                ),),
                parser_name=self.parser_name,
                raw_output={'trace': trace.to_dict() if trace else {}},
            )

        semantic_envelope: SemanticParseEnvelope | None = None
        backend_result = None
        fallback_reason = ''
        if self.deterministic_only:
            fallback_reason = 'deterministic_only'
        else:
            semantic_context = (
                confirmed_draft
                if uses_confirmed_context(request, confirmed_draft)
                else None
            )
            backend_result = self.backend.infer(
                request,
                confirmed_context=(
                    semantic_context.to_dict()
                    if semantic_context is not None
                    else None
                ),
            )
            if backend_result.ok:
                envelope_result = strict_semantic_envelope_from_json(backend_result.text)
                if envelope_result.ok:
                    semantic_envelope = envelope_result.envelope
                else:
                    fallback_reason = envelope_result.issues[0].code if envelope_result.issues else 'invalid_semantic_envelope'
            else:
                fallback_reason = backend_result.fallback_reason or backend_result.status

        fusion = self.resolver.resolve(
            request_text=request,
            explicit_facts=explicit_facts,
            semantic_envelope=semantic_envelope,
            backend_result=backend_result,
            confirmed_draft=confirmed_draft,
            shadow_mode=self.shadow_mode,
            fallback_reason=fallback_reason,
            prompt_schema_version=self.config.prompt_schema_version,
        )
        raw_output = {
            'trace': fusion.trace.to_dict() if fusion.trace else {},
            'semantic_envelope': semantic_envelope.to_dict() if semantic_envelope else None,
            'backend_result': backend_result.to_dict() if backend_result else None,
        }
        if fusion.ok:
            return DraftParseResult(
                status='ok',
                draft=fusion.draft,
                parser_name=self.parser_name,
                raw_output=raw_output,
            )
        return DraftParseResult(
            status='error',
            issues=fusion.issues,
            parser_name=self.parser_name,
            raw_output=raw_output,
        )


class ParserPipeline:
    def __init__(
        self,
        parsers: list[TaskGoalDraftParser] | None = None,
        *,
        semantic_backend: LocalSemanticBackend | None = None,
        semantic_config: LocalSemanticModelConfig | None = None,
    ) -> None:
        self.parsers = parsers or [
            StructuredFormParser(),
            ConversationalIntentGatewayParser(backend=semantic_backend, config=semantic_config),
            RegexFallbackParser(),
        ]

    def parse(
        self,
        request: Any,
        *,
        confirmed_draft: TaskGoalDraft | None = None,
    ) -> DraftParseResult:
        errors: list[GoalIssue] = []
        for parser in self.parsers:
            if not isinstance(request, dict) and getattr(parser, 'parser_name', '') == StructuredFormParser.parser_name:
                continue
            try:
                result = parser.parse(request, confirmed_draft=confirmed_draft)  # type: ignore[call-arg]
            except TypeError:
                result = parser.parse(request)
            if result.ok:
                return result
            errors.extend(result.issues)
            if getattr(parser, 'parser_name', '') == ConversationalIntentGatewayParser.parser_name:
                if any(issue.code in TERMINAL_GATEWAY_ISSUES for issue in result.issues):
                    return DraftParseResult(
                        status='error',
                        issues=tuple(errors),
                        parser_name=parser.parser_name,
                        raw_output=result.raw_output,
                    )
            if isinstance(request, dict):
                break
        return DraftParseResult(status='error', issues=tuple(errors), parser_name='parser_pipeline')


def parse_task_goal_draft(request: Any) -> DraftParseResult:
    return ParserPipeline().parse(request)


def _goal_type_from_text(text: str) -> str | None:
    if re.search(r'\b(?:inspect|check|observe|look\s+at)\b', text):
        return 'inspection'
    if re.search(r'\b(?:move|send|route|bring|deliver|transport|transfer)\b', text):
        return 'transport'
    return None


def _side_from_text(text: str) -> str | None:
    if re.search(r'\bright\b', text):
        return 'right'
    if re.search(r'\bleft\b', text):
        return 'left'
    return None


def _selection_strategy_from_text(text: str, *, shuttle: bool) -> str | None:
    if re.search(r'\b(?:nearest|closest)\b', text):
        return 'nearest'
    if shuttle:
        return 'explicit'
    if re.search(r'\b(?:any\s+)?(?:shuttle|carrier)\b', text):
        return 'any'
    return None


def _payload_filter_from_text(text: str) -> str | None:
    if re.search(
        r'\b(?:loaded|carrying(?:\s+(?:a\s+)?(?:payload|part|load|component))?|holding(?:\s+(?:a\s+)?(?:payload|part|load|component))?|with\s+(?:a\s+)?(?:payload|part|load|component))\b',
        text,
    ):
        return 'loaded'
    if re.search(r'\b(?:empty|unloaded|without\s+(?:a\s+)?(?:payload|part|load|component))\b', text):
        return 'empty'
    if re.search(r'\bany\s+(?:payload|load|shuttle)\b', text):
        return 'any'
    return None


def _shuttle_from_text(text: str) -> str | None:
    match = re.search(
        r'\b(?P<shuttle>(?:room315_)?(?:right|left)_shuttle_?[1-4]|[rl][1-4])\b',
        text,
    )
    return match.group('shuttle') if match else None


def _slot_from_text(text: str) -> tuple[GoalIssue | None, str | None, str | None]:
    match = re.search(
        r'\b(?:(?P<side_a>right|left)\s+)?slot\s*[_-]?\s*(?P<slot>\d+)'
        r'(?:\s+(?:on|for)\s+(?P<side_b>right|left))?\b',
        text,
    )
    if not match:
        return None, None, None
    slot = match.group('slot')
    if slot not in SLOTS:
        return GoalIssue('invalid_slot', 'Room 315 slot must be 1, 2, 3, or 4.', 'target_slot', SLOTS), None, None
    return None, match.group('side_a') or match.group('side_b') or None, slot


def _station_from_text(text: str) -> str | None:
    for token in ('staubli', 'kuka', 'yaskawa'):
        if re.search(rf'\b{token}\b', text):
            return token
    return None


def _normalize_text(text: str) -> str:
    lowered = str(text or '').casefold().replace('-', '_')
    lowered = re.sub(r'[^\w\s:]', ' ', lowered)
    return re.sub(r'\s+', ' ', lowered).strip()


def _has_non_english_letters(text: str) -> bool:
    return bool(re.search(r'[^\x00-\x7f]', text))


__all__ = [
    'ConversationalIntentGatewayParser',
    'DeterministicEnglishParser',
    'LocalSemanticModelAdapter',
    'ParserPipeline',
    'RegexFallbackParser',
    'StructuredFormParser',
    'TaskGoalDraftParser',
    'parse_task_goal_draft',
]
