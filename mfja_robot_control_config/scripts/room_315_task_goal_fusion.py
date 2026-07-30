#!/usr/bin/env python3
"""Evidence-aware fusion for Room 315 task-goal parsing."""

from __future__ import annotations

import copy
import re
import time
from dataclasses import dataclass
from dataclasses import field as dataclass_field
from typing import Any

from room_315_task_goal_schema import PAYLOAD_FILTERS
from room_315_task_goal_schema import SELECTION_STRATEGIES
from room_315_task_goal_schema import SIDES
from room_315_task_goal_schema import SLOTS
from room_315_task_goal_schema import STATION_ALIASES
from room_315_task_goal_schema import GoalIssue
from room_315_task_goal_schema import TaskGoalDraft
from room_315_task_goal_schema import normalize_station_symbol
from room_315_task_goal_schema import slug_text
from room_315_task_goal_semantic import SemanticBackendResult
from room_315_task_goal_semantic import SemanticParseEnvelope


DRAFT_FIELDS = (
    'goal_type',
    'selection_strategy',
    'payload_filter',
    'side',
    'target_kind',
    'target_station',
    'target_slot',
    'target_shuttle',
    'inspection_subject',
)
CORRECTION_TERMS = re.compile(r'\b(?:actually|instead|correction|change|make it|use|switch to)\b', re.I)
REFERENCE_TERMS = re.compile(r'\b(?:it|that one|there|same shuttle|same one)\b', re.I)
GENERIC_SHUTTLE_TERMS = re.compile(r'\b(?:shuttle|carrier)\b', re.I)
NEAREST_TERMS = re.compile(r'\b(?:nearest|closest)\b', re.I)
LOADED_TERMS = re.compile(
    r'\b(?:loaded|carrying(?:\s+(?:a\s+)?(?:payload|part|load|component))?|holding(?:\s+(?:a\s+)?(?:payload|part|load|component))?|with\s+(?:a\s+)?(?:payload|part|load|component))\b',
    re.I,
)
EMPTY_TERMS = re.compile(r'\b(?:empty|unloaded|without\s+(?:a\s+)?(?:payload|part|load|component))\b', re.I)
SLOT_TARGET_TERMS = re.compile(r'\b(?:slot|position|place)\b', re.I)


@dataclass(frozen=True)
class SourceSpan:
    text: str
    start: int
    end: int

    def to_dict(self) -> dict[str, Any]:
        return {'text': self.text, 'start': self.start, 'end': self.end}


@dataclass(frozen=True)
class ExplicitFact:
    field: str
    value: Any
    span: SourceSpan
    provenance: str = 'explicit_text'

    def to_dict(self) -> dict[str, Any]:
        return {
            'field': self.field,
            'value': copy.deepcopy(self.value),
            'span': self.span.to_dict(),
            'provenance': self.provenance,
        }


@dataclass(frozen=True)
class ParseTrace:
    parser_name: str
    source_spans: tuple[ExplicitFact, ...] = ()
    model_evidence: dict[str, str] = dataclass_field(default_factory=dict)
    field_provenance: dict[str, str] = dataclass_field(default_factory=dict)
    parser_disagreements: tuple[dict[str, Any], ...] = ()
    fallback_reason: str = ''
    model_fingerprint: str = ''
    model_backend: str = ''
    model_ready: bool = False
    semantic_model_invoked: bool = False
    fallback_used: bool = False
    prompt_schema_version: int = 0
    envelope_schema_version: int = 0
    latency_s: float = 0.0
    validation_status: str = ''
    final_decision: str = ''
    shadow_mode: bool = False
    dialogue_act: str = 'new_goal'

    def to_dict(self) -> dict[str, Any]:
        return {
            'parser_name': self.parser_name,
            'source_spans': [fact.to_dict() for fact in self.source_spans],
            'model_evidence': copy.deepcopy(self.model_evidence),
            'field_provenance': copy.deepcopy(self.field_provenance),
            'parser_disagreements': [copy.deepcopy(item) for item in self.parser_disagreements],
            'fallback_reason': self.fallback_reason,
            'model_fingerprint': self.model_fingerprint,
            'model_backend': self.model_backend,
            'model_ready': self.model_ready,
            'semantic_model_invoked': self.semantic_model_invoked,
            'fallback_used': self.fallback_used,
            'prompt_schema_version': self.prompt_schema_version,
            'envelope_schema_version': self.envelope_schema_version,
            'latency_s': self.latency_s,
            'validation_status': self.validation_status,
            'final_decision': self.final_decision,
            'shadow_mode': self.shadow_mode,
            'dialogue_act': self.dialogue_act,
        }


@dataclass(frozen=True)
class FusionResult:
    status: str
    draft: TaskGoalDraft | None = None
    issues: tuple[GoalIssue, ...] = ()
    trace: ParseTrace | None = None

    @property
    def ok(self) -> bool:
        return self.status == 'ok' and self.draft is not None


class ExplicitFactExtractor:
    """High-precision extractor for exact Room 315 tokens.

    This intentionally does not try to infer broad semantics. It captures exact
    grounded text and leaves indirect meaning to the semantic model.
    """

    def extract(self, text: str) -> tuple[ExplicitFact, ...]:
        facts: list[ExplicitFact] = []
        provenance = 'explicit_correction' if CORRECTION_TERMS.search(text) else 'explicit_text'
        facts.extend(self._extract_shuttles(text, provenance))
        facts.extend(self._extract_sides(text, provenance))
        facts.extend(self._extract_slots(text, provenance))
        facts.extend(self._extract_stations(text, provenance))
        facts.extend(self._extract_payloads(text, provenance))
        facts.extend(self._extract_selection(text, provenance))
        facts.extend(self._extract_goal_types(text, provenance))
        return tuple(sorted(facts, key=lambda fact: (fact.span.start, fact.field)))

    def contains_unresolved_reference(self, text: str, *, has_context: bool) -> bool:
        return bool(REFERENCE_TERMS.search(text)) and not has_context

    def _extract_shuttles(self, text: str, provenance: str) -> list[ExplicitFact]:
        facts = []
        pattern = re.compile(r'\b(?P<value>(?:room315_)?(?:right|left)_shuttle_?[1-4]|[RLrl][1-4])\b')
        for match in pattern.finditer(text):
            facts.append(_fact('target_shuttle', match.group('value'), match, provenance))
            facts.append(_fact('selection_strategy', 'explicit', match, provenance))
        return facts

    def _extract_sides(self, text: str, provenance: str) -> list[ExplicitFact]:
        facts = []
        for match in re.finditer(r'\b(right|left)\b', text, flags=re.I):
            facts.append(_fact('side', match.group(1).casefold(), match, provenance))
        return facts

    def _extract_slots(self, text: str, provenance: str) -> list[ExplicitFact]:
        facts = []
        for match in re.finditer(r'\bslot\s*[_-]?\s*(?P<value>\d+)\b', text, flags=re.I):
            facts.append(_fact('target_slot', match.group('value'), match, provenance))
            facts.append(_fact('target_kind', 'slot', match, provenance))
        return facts

    def _extract_stations(self, text: str, provenance: str) -> list[ExplicitFact]:
        facts = []
        for match in re.finditer(r'\b(yaskawa|staubli|kuka)\b', text, flags=re.I):
            station = normalize_station_symbol(match.group(1))
            facts.append(_fact('target_station', station, match, provenance))
            facts.append(_fact('target_kind', 'station', match, provenance))
        return facts

    def _extract_payloads(self, text: str, provenance: str) -> list[ExplicitFact]:
        facts = []
        loaded = r'\b(?:loaded|carrying(?:\s+(?:a\s+)?(?:payload|part|load|component))?|holding(?:\s+(?:a\s+)?(?:payload|part|load|component))?|with\s+(?:a\s+)?(?:payload|part|load|component))\b'
        empty = r'\b(?:empty|unloaded|without\s+(?:a\s+)?(?:payload|part|load|component))\b'
        for match in re.finditer(loaded, text, flags=re.I):
            facts.append(_fact('payload_filter', 'loaded', match, provenance))
        for match in re.finditer(empty, text, flags=re.I):
            facts.append(_fact('payload_filter', 'empty', match, provenance))
        return facts

    def _extract_selection(self, text: str, provenance: str) -> list[ExplicitFact]:
        facts = []
        for match in re.finditer(r'\b(?:nearest|closest)\b', text, flags=re.I):
            facts.append(_fact('selection_strategy', 'nearest', match, provenance))
        return facts

    def _extract_goal_types(self, text: str, provenance: str) -> list[ExplicitFact]:
        facts = []
        transport = r'\b(?:move|send|route|bring|deliver|transport|transfer)\b'
        inspection = r'\b(?:inspect|inspection|check|observe|look\s+at)\b'
        for match in re.finditer(transport, text, flags=re.I):
            facts.append(_fact('goal_type', 'transport', match, provenance))
        for match in re.finditer(inspection, text, flags=re.I):
            facts.append(_fact('goal_type', 'inspection', match, provenance))
        return facts


class EvidenceAwareFusionResolver:
    """Merge structured, explicit, confirmed, and semantic evidence."""

    def resolve(
        self,
        *,
        request_text: str = '',
        explicit_facts: tuple[ExplicitFact, ...],
        semantic_envelope: SemanticParseEnvelope | None,
        backend_result: SemanticBackendResult | None,
        confirmed_draft: TaskGoalDraft | None = None,
        structured_draft: TaskGoalDraft | None = None,
        shadow_mode: bool = False,
        fallback_reason: str = '',
        prompt_schema_version: int = 0,
    ) -> FusionResult:
        started = time.monotonic()
        explicit_conflicts = _explicit_conflicts(explicit_facts)
        if explicit_conflicts:
            trace = self._trace(
                explicit_facts=explicit_facts,
                semantic_envelope=semantic_envelope,
                backend_result=backend_result,
                field_provenance={},
                disagreements=(),
                fallback_reason=fallback_reason,
                prompt_schema_version=prompt_schema_version,
                latency_s=time.monotonic() - started,
                final_decision='clarification_required',
                shadow_mode=shadow_mode,
            )
            return FusionResult(
                status='error',
                issues=(GoalIssue(
                    'ambiguous_explicit_conflict',
                    'The current text contains conflicting explicit Room 315 facts.',
                    explicit_conflicts[0],
                    details={'conflicts': explicit_conflicts},
                ),),
                trace=trace,
            )

        values: dict[str, Any] = {}
        provenance: dict[str, str] = {}
        disagreements: list[dict[str, Any]] = []

        _merge_draft(values, provenance, structured_draft, 'structured_form')
        explicit_values = _facts_to_values(explicit_facts)
        _merge_values(values, provenance, explicit_values, 'explicit_text')
        confirmed_context = (
            confirmed_draft
            if uses_confirmed_context(request_text, confirmed_draft)
            else None
        )
        _merge_draft(values, provenance, confirmed_context, 'confirmed_context', only_missing=True)

        semantic_patch = semantic_envelope.draft_patch if semantic_envelope and semantic_envelope.draft_patch else {}
        semantic_patch = _sanitize_semantic_patch(
            request_text,
            semantic_patch,
            explicit_values=explicit_values,
            confirmed_draft=confirmed_context,
        )
        if semantic_patch and not shadow_mode:
            for field, value in semantic_patch.items():
                if field not in DRAFT_FIELDS and field != 'confidence':
                    continue
                if value in (None, ''):
                    continue
                if field in values and values[field] != value:
                    disagreements.append({
                        'field': field,
                        'authoritative': values[field],
                        'semantic': value,
                        'decision': 'kept_authoritative_value',
                    })
                    continue
                values[field] = value
                provenance.setdefault(field, semantic_envelope.provenance.get(field, 'semantic_inference'))
        elif semantic_patch and shadow_mode:
            for field, value in semantic_patch.items():
                if field in values and values[field] != value:
                    disagreements.append({
                        'field': field,
                        'authoritative': values[field],
                        'semantic': value,
                        'decision': 'shadow_only_disagreement',
                    })

        self._prefer_explicit_inspection_subject(values, provenance, explicit_values)
        self._drop_ungrounded_target_kind(values, provenance)
        self._infer_target_kind(values)
        if values.get('target_shuttle') and not values.get('selection_strategy'):
            values['selection_strategy'] = 'explicit'
            provenance.setdefault('selection_strategy', provenance.get('target_shuttle', 'explicit_text'))
        if (
            not values.get('target_shuttle')
            and not values.get('selection_strategy')
            and _has_generic_shuttle_reference(request_text)
        ):
            values['selection_strategy'] = 'any'
            provenance.setdefault('selection_strategy', 'explicit_text')

        try:
            draft = TaskGoalDraft(
                goal_type=values.get('goal_type'),
                selection_strategy=values.get('selection_strategy'),
                payload_filter=values.get('payload_filter'),
                side=values.get('side'),
                target_kind=values.get('target_kind'),
                target_station=values.get('target_station'),
                target_slot=values.get('target_slot'),
                target_shuttle=values.get('target_shuttle'),
                inspection_subject=values.get('inspection_subject'),
                confidence=values.get('confidence'),
                source='human',
                language='en',
                raw={
                    'request': request_text,
                    'fusion_values': copy.deepcopy(values),
                },
            )
        except ValueError as exc:
            trace = self._trace(
                explicit_facts=explicit_facts,
                semantic_envelope=semantic_envelope,
                backend_result=backend_result,
                field_provenance=provenance,
                disagreements=tuple(disagreements),
                fallback_reason=fallback_reason,
                prompt_schema_version=prompt_schema_version,
                latency_s=time.monotonic() - started,
                final_decision='error',
                shadow_mode=shadow_mode,
            )
            return FusionResult(
                status='error',
                issues=(GoalIssue('invalid_fused_draft', str(exc), 'draft'),),
                trace=trace,
            )

        if not any(getattr(draft, field) is not None for field in DRAFT_FIELDS):
            trace = self._trace(
                explicit_facts=explicit_facts,
                semantic_envelope=semantic_envelope,
                backend_result=backend_result,
                field_provenance=provenance,
                disagreements=tuple(disagreements),
                fallback_reason=fallback_reason or 'no_grounded_fields',
                prompt_schema_version=prompt_schema_version,
                latency_s=time.monotonic() - started,
                final_decision='error',
                shadow_mode=shadow_mode,
            )
            return FusionResult(
                status='error',
                issues=(GoalIssue('parse_no_match', 'No grounded Room 315 task-goal fields were found.', 'request'),),
                trace=trace,
            )

        trace = self._trace(
            explicit_facts=explicit_facts,
            semantic_envelope=semantic_envelope,
            backend_result=backend_result,
            field_provenance=provenance,
            disagreements=tuple(disagreements),
            fallback_reason=fallback_reason,
            prompt_schema_version=prompt_schema_version,
            latency_s=time.monotonic() - started,
            final_decision='draft',
            shadow_mode=shadow_mode,
        )
        return FusionResult(status='ok', draft=draft, trace=trace)

    def _prefer_explicit_inspection_subject(
        self,
        values: dict[str, Any],
        provenance: dict[str, str],
        explicit_values: dict[str, Any],
    ) -> None:
        if values.get('goal_type') != 'inspection':
            return
        if not explicit_values.get('target_shuttle'):
            return
        if explicit_values.get('target_slot') or explicit_values.get('target_station'):
            return
        values['target_kind'] = 'shuttle'
        values['target_shuttle'] = explicit_values['target_shuttle']
        provenance['target_kind'] = provenance.get('target_shuttle', 'explicit_text')
        values.pop('target_slot', None)
        values.pop('target_station', None)
        provenance.pop('target_slot', None)
        provenance.pop('target_station', None)

    def _drop_ungrounded_target_kind(
        self,
        values: dict[str, Any],
        provenance: dict[str, str],
    ) -> None:
        target_kind = values.get('target_kind')
        if target_kind == 'shuttle' and not values.get('target_shuttle'):
            values.pop('target_kind', None)
            provenance.pop('target_kind', None)
        elif target_kind == 'slot' and not values.get('target_slot'):
            values.pop('target_kind', None)
            provenance.pop('target_kind', None)
        elif target_kind == 'station' and not values.get('target_station'):
            values.pop('target_kind', None)
            provenance.pop('target_kind', None)
        elif target_kind == 'shuttle_selection' and not (
            values.get('selection_strategy') or values.get('payload_filter')
        ):
            values.pop('target_kind', None)
            provenance.pop('target_kind', None)

    def _infer_target_kind(self, values: dict[str, Any]) -> None:
        if values.get('target_kind'):
            return
        if values.get('target_slot'):
            values['target_kind'] = 'slot'
        elif values.get('target_station'):
            values['target_kind'] = 'station'
        elif values.get('goal_type') == 'inspection' and values.get('target_shuttle'):
            values['target_kind'] = 'shuttle'
        elif values.get('goal_type') == 'inspection' and (
            values.get('selection_strategy') or values.get('payload_filter')
        ):
            values['target_kind'] = 'shuttle_selection'

    def _trace(
        self,
        *,
        explicit_facts: tuple[ExplicitFact, ...],
        semantic_envelope: SemanticParseEnvelope | None,
        backend_result: SemanticBackendResult | None,
        field_provenance: dict[str, str],
        disagreements: tuple[dict[str, Any], ...],
        fallback_reason: str,
        prompt_schema_version: int,
        latency_s: float,
        final_decision: str,
        shadow_mode: bool,
    ) -> ParseTrace:
        return ParseTrace(
            parser_name='conversational_intent_gateway',
            source_spans=explicit_facts,
            model_evidence=semantic_envelope.evidence if semantic_envelope else {},
            field_provenance=copy.deepcopy(field_provenance),
            parser_disagreements=tuple(copy.deepcopy(item) for item in disagreements),
            fallback_reason=fallback_reason,
            model_fingerprint=backend_result.model_fingerprint if backend_result else '',
            model_backend=backend_result.backend if backend_result else '',
            model_ready=bool(backend_result.model_ready) if backend_result else False,
            semantic_model_invoked=bool(backend_result and backend_result.ok),
            fallback_used=bool(fallback_reason),
            prompt_schema_version=prompt_schema_version,
            envelope_schema_version=semantic_envelope.schema_version if semantic_envelope else 0,
            latency_s=latency_s + (backend_result.latency_s if backend_result else 0.0),
            final_decision=final_decision,
            shadow_mode=shadow_mode,
            dialogue_act=semantic_envelope.dialogue_act if semantic_envelope else 'new_goal',
        )


def _fact(field: str, value: Any, match: re.Match[str], provenance: str) -> ExplicitFact:
    return ExplicitFact(
        field=field,
        value=value,
        span=SourceSpan(match.group(0), match.start(), match.end()),
        provenance=provenance,
    )


def _facts_to_values(facts: tuple[ExplicitFact, ...]) -> dict[str, Any]:
    values: dict[str, Any] = {}
    for fact in facts:
        if fact.field == 'target_slot' and str(fact.value) not in SLOTS:
            values[fact.field] = str(fact.value)
        elif fact.field == 'target_station':
            values[fact.field] = normalize_station_symbol(fact.value)
        elif fact.field == 'side' and fact.value in SIDES:
            values[fact.field] = fact.value
        elif fact.field == 'selection_strategy' and fact.value in SELECTION_STRATEGIES:
            values[fact.field] = fact.value
        elif fact.field == 'payload_filter' and fact.value in PAYLOAD_FILTERS:
            values[fact.field] = fact.value
        else:
            values[fact.field] = fact.value
    return values


def _sanitize_semantic_patch(
    request_text: str,
    patch: dict[str, Any],
    *,
    explicit_values: dict[str, Any],
    confirmed_draft: TaskGoalDraft | None,
) -> dict[str, Any]:
    sanitized: dict[str, Any] = {}
    for field, value in (patch or {}).items():
        if value in (None, ''):
            continue
        if field == 'side' and not _has_authority('side', explicit_values, confirmed_draft):
            continue
        if field == 'target_shuttle' and not _has_authority('target_shuttle', explicit_values, confirmed_draft):
            continue
        if field == 'target_station' and not _has_authority(
            'target_station',
            explicit_values,
            confirmed_draft,
        ):
            continue
        if field == 'target_slot' and not (
            explicit_values.get('target_slot')
            or (confirmed_draft is not None and confirmed_draft.target_slot)
            or SLOT_TARGET_TERMS.search(request_text)
        ):
            continue
        if field == 'selection_strategy':
            if value == 'nearest' and not (
                explicit_values.get('selection_strategy') == 'nearest'
                or (confirmed_draft is not None and confirmed_draft.selection_strategy == 'nearest')
                or NEAREST_TERMS.search(request_text)
            ):
                continue
            if value == 'explicit' and not (
                explicit_values.get('target_shuttle')
                or (confirmed_draft is not None and confirmed_draft.target_shuttle)
            ):
                continue
        if field == 'payload_filter':
            if value == 'loaded' and not (
                explicit_values.get('payload_filter') == 'loaded'
                or (confirmed_draft is not None and confirmed_draft.payload_filter == 'loaded')
                or LOADED_TERMS.search(request_text)
            ):
                continue
            if value == 'empty' and not (
                explicit_values.get('payload_filter') == 'empty'
                or (confirmed_draft is not None and confirmed_draft.payload_filter == 'empty')
                or EMPTY_TERMS.search(request_text)
            ):
                continue
        sanitized[field] = value
    return sanitized


def _has_authority(field: str, explicit_values: dict[str, Any], confirmed_draft: TaskGoalDraft | None) -> bool:
    if explicit_values.get(field):
        return True
    return confirmed_draft is not None and getattr(confirmed_draft, field, None) not in (None, '')


def _has_generic_shuttle_reference(text: str) -> bool:
    return bool(GENERIC_SHUTTLE_TERMS.search(text))


def uses_confirmed_context(
    text: str,
    confirmed_draft: TaskGoalDraft | None,
) -> bool:
    """Return whether this utterance explicitly refers to confirmed context."""
    return confirmed_draft is not None and bool(REFERENCE_TERMS.search(text))


def _explicit_conflicts(facts: tuple[ExplicitFact, ...]) -> list[str]:
    values: dict[str, set[Any]] = {}
    for fact in facts:
        if fact.field == 'target_kind':
            continue
        values.setdefault(fact.field, set()).add(slug_text(fact.value))
    return sorted(field for field, field_values in values.items() if len(field_values) > 1)


def _merge_draft(
    values: dict[str, Any],
    provenance: dict[str, str],
    draft: TaskGoalDraft | None,
    source: str,
    *,
    only_missing: bool = False,
) -> None:
    if draft is None:
        return
    payload = draft.to_dict(include_nulls=True)
    for field in DRAFT_FIELDS:
        value = payload.get(field)
        if value in (None, ''):
            continue
        if only_missing and field in values:
            continue
        values[field] = value
        provenance[field] = source


def _merge_values(
    values: dict[str, Any],
    provenance: dict[str, str],
    updates: dict[str, Any],
    source: str,
) -> None:
    for field, value in updates.items():
        if value in (None, ''):
            continue
        values[field] = value
        provenance[field] = source


__all__ = [
    'EvidenceAwareFusionResolver',
    'ExplicitFact',
    'ExplicitFactExtractor',
    'FusionResult',
    'ParseTrace',
    'SourceSpan',
]
