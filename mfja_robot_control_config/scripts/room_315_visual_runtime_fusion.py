#!/usr/bin/env python3
"""Deterministic Room 315 visual-fact fusion and PlanSys2 boundary."""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_contracts import ObservedFact
from room_315_contracts import ObservedState
from room_315_multi_shuttle import normalize_fleet_block_id
from room_315_multi_shuttle import normalize_shuttle_ref
from room_315_observed_state_provider import fuse_observed_facts
from room_315_presence_provider import PRESENCE_ABSENT
from room_315_presence_provider import PRESENCE_PRESENT
from room_315_presence_provider import PresenceSnapshot
from room_315_visual_runtime_validation import ValidationResult


@dataclass(frozen=True)
class StateFusionResult:
    ready: bool
    reasons: tuple[str, ...]
    observed_state: ObservedState | None
    field_sources: dict[str, str]


@dataclass(frozen=True)
class PlanSysPredicateUpdate:
    accepted: bool
    reasons: tuple[str, ...]
    add_predicates: tuple[str, ...]
    remove_predicates: tuple[str, ...]


def fuse_validated_visual_state(
    validation: ValidationResult,
    presence: PresenceSnapshot,
    *,
    checkpoint_sha256: str,
    schema_version: str,
    stale_after_s: float,
    state_id: str,
    additional_trusted_facts: list[ObservedFact] | None = None,
) -> StateFusionResult:
    """Convert only accepted active-slot outputs to existing ObservedState."""

    reasons: list[str] = []
    if not validation.accepted or validation.prediction is None:
        reasons.extend(validation.reasons or ('visual_validation_failed',))
    if not presence.ready:
        reasons.extend(presence.reasons or ('presence_registry_not_ready',))
    if reasons:
        return StateFusionResult(
            ready=False,
            reasons=tuple(dict.fromkeys(reasons)),
            observed_state=None,
            field_sources={},
        )

    prediction = validation.prediction
    presence_facts: list[ObservedFact] = []
    visual_facts: list[ObservedFact] = []
    field_sources: dict[str, str] = {}
    common_metadata = {
        'checkpoint_sha256': checkpoint_sha256,
        'visual_schema_version': schema_version,
        'validation_status': 'accepted',
        'validation_reasons': list(validation.reasons),
        'source_image_timestamps': {
            'left': prediction.left_image_stamp_s,
            'right': prediction.right_image_stamp_s,
        },
        'model_ready': True,
        'input_ready': True,
        'state_fusion_ready': True,
        'stale': False,
    }
    presence_by_id = presence.by_identity()
    for identity, entry in presence_by_id.items():
        spec = normalize_shuttle_ref(identity)
        if spec is None:
            return StateFusionResult(
                ready=False,
                reasons=(f'authoritative_identity_mapping_failed:{identity}',),
                observed_state=None,
                field_sources={},
            )
        if entry.state not in {PRESENCE_PRESENT, PRESENCE_ABSENT}:
            return StateFusionResult(
                ready=False,
                reasons=(f'presence_unknown:{identity}',),
                observed_state=None,
                field_sources={},
            )
        subject = spec.gazebo_entity_name
        value = entry.state == PRESENCE_PRESENT
        presence_facts.append(ObservedFact(
            fact_id=f'presence-{identity}',
            subject=subject,
            predicate='present',
            value=value,
            source='trusted_device',
            timestamp=prediction.timestamp_s,
            confidence=1.0,
            status='known',
            metadata={
                **common_metadata,
                'field_owner': 'deterministic_controller_presence',
                'presence_source': presence.source,
                'identity': identity,
                'side': spec.side,
            },
        ))
        field_sources[f'{identity}.present'] = 'deterministic_sensor'

    for shuttle in prediction.shuttles:
        spec = normalize_shuttle_ref(shuttle.identity)
        if spec is None:
            return StateFusionResult(
                ready=False,
                reasons=(f'visual_identity_mapping_failed:{shuttle.identity}',),
                observed_state=None,
                field_sources={},
            )
        if presence_by_id[shuttle.identity].state != PRESENCE_PRESENT:
            return StateFusionResult(
                ready=False,
                reasons=(f'visual_fact_for_absent_identity:{shuttle.identity}',),
                observed_state=None,
                field_sources={},
            )
        subject = spec.gazebo_entity_name
        block_id = normalize_fleet_block_id(shuttle.block, side=shuttle.side)
        if not block_id:
            return StateFusionResult(
                ready=False,
                reasons=(f'cannot_normalize_visual_block:{shuttle.identity}',),
                observed_state=None,
                field_sources={},
            )
        values = {
            'rail_side': shuttle.side,
            'location_block': block_id,
            'rail_position': {
                'available': True,
                'side': shuttle.side,
                'segment': shuttle.block,
                's_m': shuttle.s_m,
                's_ratio': shuttle.s_ratio,
                'segment_length_m': shuttle.segment_length_m,
            },
            'loaded': shuttle.loaded_state == 'loaded',
            'visual_bbox': {
                'camera': (
                    'left_rail_rgb'
                    if shuttle.side == 'left'
                    else 'right_rail_rgb'
                ),
                'bbox_xywh': list(shuttle.bbox_xywh),
            },
        }
        for predicate, value in values.items():
            visual_facts.append(ObservedFact(
                fact_id=f'visual-{shuttle.identity}-{predicate}',
                subject=subject,
                predicate=predicate,
                value=value,
                source='visual_model',
                timestamp=prediction.timestamp_s,
                # The model has no calibrated confidence output.  Zero is the
                # explicit contract sentinel; deterministic validation status
                # is carried in metadata instead.
                confidence=0.0,
                status='known',
                metadata={
                    **common_metadata,
                    'field_owner': 'visual_model',
                    'identity': shuttle.identity,
                    'confidence_available': False,
                    'confidence_semantics': 'unsupported_by_approved_model',
                },
            ))
            field_sources[f'{shuttle.identity}.{predicate}'] = 'visual_model'

    all_facts = [
        *presence_facts,
        *(additional_trusted_facts or []),
        *visual_facts,
    ]
    fused = fuse_observed_facts(
        all_facts,
        timestamp=prediction.timestamp_s,
    )
    state = ObservedState(
        state_id=state_id,
        timestamp=prediction.timestamp_s,
        stale_after_s=stale_after_s,
        visual_model_inputs=visual_facts,
        fused_planner_state=fused,
    )
    return StateFusionResult(
        ready=True,
        reasons=(),
        observed_state=state,
        field_sources=field_sources,
    )


class DeterministicPlanSys2FactGate:
    """Create an idempotent predicate delta; never plans or executes."""

    def __init__(self) -> None:
        self._previous: set[str] = set()

    def reset(self) -> None:
        self._previous.clear()

    def build_update(
        self,
        fusion: StateFusionResult,
        *,
        model_ready: bool,
        input_ready: bool,
        safety_ready: bool,
        enabled: bool,
    ) -> PlanSysPredicateUpdate:
        reasons: list[str] = []
        if not enabled:
            reasons.append('plansys2_updates_disabled')
        if not model_ready:
            reasons.append('model_not_ready')
        if not input_ready:
            reasons.append('input_not_ready')
        if not safety_ready:
            reasons.append('safety_supervisor_not_ready')
        if not fusion.ready or fusion.observed_state is None:
            reasons.extend(fusion.reasons or ('state_fusion_not_ready',))
        if reasons:
            return PlanSysPredicateUpdate(
                accepted=False,
                reasons=tuple(dict.fromkeys(reasons)),
                add_predicates=(),
                remove_predicates=(),
            )

        current = _existing_visual_predicates(fusion.observed_state)
        add = tuple(sorted(current - self._previous))
        remove = tuple(sorted(self._previous - current))
        self._previous = current
        return PlanSysPredicateUpdate(
            accepted=True,
            reasons=(),
            add_predicates=add,
            remove_predicates=remove,
        )


def _existing_visual_predicates(state: ObservedState) -> set[str]:
    """Map only visual facts to predicates already declared in the domain."""

    by_subject: dict[str, dict[str, ObservedFact]] = {}
    for fact in state.fused_planner_state:
        if fact.status != 'known':
            continue
        by_subject.setdefault(fact.subject, {})[fact.predicate] = fact

    predicates: set[str] = set()
    occupancy: dict[str, str] = {}
    for subject, facts in by_subject.items():
        spec = normalize_shuttle_ref(subject)
        if spec is None:
            continue
        present = facts.get('present')
        if present is None or not bool(present.value):
            continue
        side = spec.side
        shuttle = _pddl_symbol(spec.short_id)
        predicates.add(f'(shuttle_on_side {shuttle} {side})')
        loaded = facts.get('loaded')
        if loaded is not None:
            predicates.add(
                f'({"loaded" if bool(loaded.value) else "empty"} {shuttle})'
            )
        block = facts.get('location_block')
        if block is not None and block.value:
            block_symbol = _pddl_symbol(block.value)
            predicates.add(f'(shuttle_in_block {shuttle} {block_symbol})')
            existing = occupancy.get(block_symbol)
            if existing and existing != shuttle:
                raise ValueError(
                    f'validated visual state has duplicate block occupancy: '
                    f'{block_symbol}'
                )
            occupancy[block_symbol] = shuttle
    for block, shuttle in occupancy.items():
        predicates.add(f'(block_occupied_by {block} {shuttle})')
    return predicates


def _pddl_symbol(value: Any) -> str:
    result = re.sub(r'[^a-z0-9_-]+', '_', str(value).strip().lower())
    return result.strip('_') or 'unknown'
