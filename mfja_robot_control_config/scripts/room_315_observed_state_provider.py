#!/usr/bin/env python3
"""ObservedState providers for the Room 315 closed-loop planner boundary."""

from __future__ import annotations

import copy
import math
import re
import sys
from abc import ABC
from abc import abstractmethod
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_contracts import ObservedFact
from room_315_contracts import ObservedState
from room_315_multi_shuttle import DEVICE_NAMES
from room_315_multi_shuttle import SIDES
from room_315_multi_shuttle import all_shuttle_specs
from room_315_multi_shuttle import fleet_safety_state_from_rails
from room_315_multi_shuttle import normalize_fleet_block_id
from room_315_multi_shuttle import normalize_shuttle_ref


SLOTS = ('1', '2', '3', '4')
DEFAULT_SLOT_SENSOR_BY_SIDE = {
    'right': {
        '1': 'DZI1R',
        '2': 'DZI2R',
        '3': 'DZI3R',
        '4': 'DZI4R',
    },
    'left': {
        '1': 'DZI1L',
        '2': 'DZI2L',
        '3': 'DZI3L',
        '4': 'DZI4L',
    },
}
DEFAULT_SOURCE_PRIORITY = ('trusted_device', 'oracle', 'visual_model')


class ObservedStateProvider(ABC):
    """Interface for components that produce validated planner observations."""

    @abstractmethod
    def observe(self, *, timestamp: float | None = None) -> ObservedState:
        """Return a versioned observed state at the requested planner time."""


class OracleObservedStateProvider(ObservedStateProvider):
    """Build oracle-only ObservedState facts from simulator/supervisor truth."""

    def __init__(
        self,
        status_snapshot: dict[str, Any] | None = None,
        *,
        rails: dict[str, Any] | None = None,
        slot_sensor_by_side: dict[str, dict[str, str]] | None = None,
        source_timestamps: dict[str, float] | None = None,
        stale_after_s: float = 1.0,
        state_id: str = 'room315-oracle-state',
    ) -> None:
        self.status_snapshot = copy.deepcopy(status_snapshot or {})
        if rails is not None:
            self.status_snapshot['rails'] = copy.deepcopy(rails)
        self.slot_sensor_by_side = _normalise_slot_sensor_map(slot_sensor_by_side)
        self.source_timestamps = dict(source_timestamps or {})
        self.stale_after_s = float(stale_after_s)
        self.state_id = state_id

    def observe(self, *, timestamp: float | None = None) -> ObservedState:
        observed_at = _timestamp_or_zero(timestamp)
        facts = _facts_from_status_snapshot(
            self.status_snapshot,
            source='oracle',
            observed_at=observed_at,
            stale_after_s=self.stale_after_s,
            slot_sensor_by_side=self.slot_sensor_by_side,
            source_timestamps=self.source_timestamps,
            include_oracle_truth=True,
        )
        return ObservedState(
            state_id=self.state_id,
            timestamp=observed_at,
            stale_after_s=self.stale_after_s,
            visual_model_inputs=[],
            fused_planner_state=facts,
        )


class FusedObservedStateProvider(ObservedStateProvider):
    """Merge visual-model facts with trusted device/status observations."""

    def __init__(
        self,
        trusted_status_snapshot: dict[str, Any] | None = None,
        *,
        visual_facts: list[ObservedFact] | None = None,
        slot_sensor_by_side: dict[str, dict[str, str]] | None = None,
        source_timestamps: dict[str, float] | None = None,
        stale_after_s: float = 1.0,
        source_priority: tuple[str, ...] = DEFAULT_SOURCE_PRIORITY,
        state_id: str = 'room315-fused-state',
    ) -> None:
        self.trusted_status_snapshot = copy.deepcopy(trusted_status_snapshot or {})
        self.visual_facts = list(visual_facts or [])
        self.slot_sensor_by_side = _normalise_slot_sensor_map(slot_sensor_by_side)
        self.source_timestamps = dict(source_timestamps or {})
        self.stale_after_s = float(stale_after_s)
        self.source_priority = tuple(source_priority)
        self.state_id = state_id

    def observe(self, *, timestamp: float | None = None) -> ObservedState:
        observed_at = _timestamp_or_zero(timestamp)
        visual_inputs = [_ensure_visual_fact(fact) for fact in self.visual_facts]
        trusted_facts = _facts_from_status_snapshot(
            self.trusted_status_snapshot,
            source='trusted_device',
            observed_at=observed_at,
            stale_after_s=self.stale_after_s,
            slot_sensor_by_side=self.slot_sensor_by_side,
            source_timestamps=self.source_timestamps,
            include_oracle_truth=False,
        )
        fused_facts = fuse_observed_facts(
            [*trusted_facts, *visual_inputs],
            timestamp=observed_at,
            source_priority=self.source_priority,
        )
        return ObservedState(
            state_id=self.state_id,
            timestamp=observed_at,
            stale_after_s=self.stale_after_s,
            visual_model_inputs=visual_inputs,
            fused_planner_state=fused_facts,
        )


def fuse_observed_facts(
    facts: list[ObservedFact],
    *,
    timestamp: float,
    source_priority: tuple[str, ...] = DEFAULT_SOURCE_PRIORITY,
) -> list[ObservedFact]:
    grouped: dict[tuple[str, str, str], list[ObservedFact]] = {}
    for fact in facts:
        grouped.setdefault((fact.subject, fact.predicate, fact.frame_id), []).append(fact)

    priority_index = {source: index for index, source in enumerate(source_priority)}
    fused: list[ObservedFact] = []
    for key in sorted(grouped):
        candidates = grouped[key]
        ordered = sorted(
            candidates,
            key=lambda fact: (
                _status_rank(fact.status),
                priority_index.get(fact.source, len(priority_index)),
                -float(fact.confidence),
                fact.fact_id,
            ),
        )
        selected = ordered[0]
        comparable = [
            fact
            for fact in ordered
            if fact.status not in {'unknown', 'stale'}
        ]
        conflict_values = []
        for fact in comparable:
            if fact.value != selected.value:
                conflict_values.append({
                    'source': fact.source,
                    'fact_id': fact.fact_id,
                    'value': copy.deepcopy(fact.value),
                })
        status = selected.status
        if conflict_values:
            status = 'conflicting'
        metadata = {
            'selected_source': selected.source,
            'candidate_sources': [fact.source for fact in ordered],
            'source_priority': list(source_priority),
            'fusion_rule': 'freshness_then_source_priority_then_confidence',
        }
        if conflict_values:
            metadata['conflicts'] = [
                {
                    'source': selected.source,
                    'fact_id': selected.fact_id,
                    'value': copy.deepcopy(selected.value),
                },
                *conflict_values,
            ]
        fused.append(ObservedFact(
            fact_id=f'fused-{_slug(key[0])}-{_slug(key[1])}',
            subject=selected.subject,
            predicate=selected.predicate,
            value=copy.deepcopy(selected.value),
            source='state_fuser',
            timestamp=timestamp,
            confidence=float(selected.confidence),
            status=status,
            frame_id=selected.frame_id,
            metadata=metadata,
        ))
    return fused


def _facts_from_status_snapshot(
    status_snapshot: dict[str, Any],
    *,
    source: str,
    observed_at: float,
    stale_after_s: float,
    slot_sensor_by_side: dict[str, dict[str, str]],
    source_timestamps: dict[str, float],
    include_oracle_truth: bool,
) -> list[ObservedFact]:
    status = status_snapshot if isinstance(status_snapshot, dict) else {}
    rails = status.get('rails', {})
    if not isinstance(rails, dict):
        rails = {}
    source_timestamp = _source_timestamp(status, source, observed_at, source_timestamps)
    source_is_stale = _is_stale(source_timestamp, observed_at, stale_after_s)
    source_meta = _source_metadata(
        source=source,
        source_timestamp=source_timestamp,
        observed_at=observed_at,
        stale_after_s=stale_after_s,
        include_oracle_truth=include_oracle_truth,
    )
    facts: list[ObservedFact] = [
        _fact(
            source=source,
            subject=f'{source}:snapshot',
            predicate='freshness',
            value={
                'observed_at': source_timestamp,
                'age_s': round(max(0.0, observed_at - source_timestamp), 6),
                'stale_after_s': stale_after_s,
                'fresh': not source_is_stale,
            },
            timestamp=source_timestamp,
            confidence=1.0,
            status='stale' if source_is_stale else 'known',
            metadata=source_meta,
        )
    ]

    slot_occupancy, shuttle_slot_by_entity, shuttle_block_by_entity = _device_locations(
        rails,
        slot_sensor_by_side=slot_sensor_by_side,
        include_oracle_truth=include_oracle_truth,
    )
    payloads = _payload_entries(status, rails)
    obstacles = _obstacle_entries(status, rails)

    if include_oracle_truth:
        fleet_state = fleet_safety_state_from_rails(
            rails,
            block_reservations=_nested_dict(status, 'safety_decoder', 'fleet_state', 'block_reservations'),
            station_slot_targets=_nested_dict(status, 'safety_decoder', 'fleet_state', 'station_slot_targets'),
            min_headway_blocks=_nested_value(status, 'safety_decoder', 'fleet_state', 'min_headway_blocks') or 1,
        )
        for spec in all_shuttle_specs():
            block = fleet_state.shuttle_blocks.get(spec.short_id)
            if block:
                shuttle_block_by_entity[spec.gazebo_entity_name] = block

    for spec in all_shuttle_specs():
        side_rail = _side_rail(rails, spec.side)
        shuttle_state = _mapping(side_rail.get('shuttles')).get(spec.gazebo_entity_name, {})
        if not isinstance(shuttle_state, dict):
            shuttle_state = {}
        payload = payloads.get(spec.gazebo_entity_name)
        present_known = bool(shuttle_state) if include_oracle_truth else (
            spec.gazebo_entity_name in shuttle_slot_by_entity
            or spec.gazebo_entity_name in shuttle_block_by_entity
            or payload is not None
        )
        facts.append(_fact(
            source=source,
            subject=spec.gazebo_entity_name,
            predicate='present',
            value=True if present_known else None,
            timestamp=source_timestamp,
            confidence=1.0 if present_known else 0.0,
            status=_fact_status(known=present_known, stale=source_is_stale),
            metadata={**source_meta, 'side': spec.side, 'short_id': spec.short_id},
        ))
        mode = shuttle_state.get('mode') if include_oracle_truth else None
        facts.append(_fact(
            source=source,
            subject=spec.gazebo_entity_name,
            predicate='motion_mode',
            value=str(mode).upper() if mode else None,
            timestamp=source_timestamp,
            confidence=1.0 if mode else 0.0,
            status=_fact_status(known=bool(mode), stale=source_is_stale),
            metadata={**source_meta, 'side': spec.side, 'short_id': spec.short_id},
        ))
        slot_value = shuttle_slot_by_entity.get(spec.gazebo_entity_name)
        facts.append(_fact(
            source=source,
            subject=spec.gazebo_entity_name,
            predicate='location_slot',
            value=slot_value,
            timestamp=source_timestamp,
            confidence=0.95 if slot_value else 0.0,
            status=_fact_status(known=bool(slot_value), stale=source_is_stale),
            metadata={**source_meta, 'side': spec.side, 'short_id': spec.short_id},
        ))
        block_value = shuttle_block_by_entity.get(spec.gazebo_entity_name)
        facts.append(_fact(
            source=source,
            subject=spec.gazebo_entity_name,
            predicate='location_block',
            value=block_value,
            timestamp=source_timestamp,
            confidence=0.95 if block_value else 0.0,
            status=_fact_status(known=bool(block_value), stale=source_is_stale),
            metadata={**source_meta, 'side': spec.side, 'short_id': spec.short_id},
        ))
        loaded_known = isinstance(payload, dict) and 'loaded' in payload
        facts.append(_fact(
            source=source,
            subject=spec.gazebo_entity_name,
            predicate='loaded',
            value=bool(payload.get('loaded')) if loaded_known else None,
            timestamp=source_timestamp,
            confidence=0.95 if loaded_known else 0.0,
            status=_fact_status(known=loaded_known, stale=source_is_stale),
            metadata={**source_meta, 'side': spec.side, 'short_id': spec.short_id},
        ))

    for side in SIDES:
        for slot in SLOTS:
            subject = _slot_id(side, slot)
            value = slot_occupancy.get(subject)
            facts.append(_fact(
                source=source,
                subject=subject,
                predicate='occupancy',
                value=value,
                timestamp=source_timestamp,
                confidence=0.95 if value is not None else 0.0,
                status=_fact_status(known=value is not None, stale=source_is_stale),
                metadata={**source_meta, 'side': side, 'slot': slot},
            ))
        side_rail = _side_rail(rails, side)
        for device in DEVICE_NAMES:
            switch_value = _normalise_switch_state(_mapping(side_rail.get('switches')).get(device))
            facts.append(_fact(
                source=source,
                subject=f'{side}:switch:{device}',
                predicate='state',
                value=switch_value,
                timestamp=source_timestamp,
                confidence=0.95 if switch_value else 0.0,
                status=_fact_status(known=bool(switch_value), stale=source_is_stale),
                metadata={**source_meta, 'side': side, 'device': device},
            ))
            stopper_value = _normalise_stopper_state(_mapping(side_rail.get('stoppers')).get(device))
            facts.append(_fact(
                source=source,
                subject=f'{side}:stopper:{device}',
                predicate='state',
                value=stopper_value,
                timestamp=source_timestamp,
                confidence=0.95 if stopper_value else 0.0,
                status=_fact_status(known=bool(stopper_value), stale=source_is_stale),
                metadata={**source_meta, 'side': side, 'device': device},
            ))
        side_obstacles = obstacles.get(side)
        facts.append(_fact(
            source=source,
            subject=f'{side}:obstacles',
            predicate='present_obstacles',
            value=sorted(side_obstacles) if side_obstacles is not None else None,
            timestamp=source_timestamp,
            confidence=0.9 if side_obstacles is not None else 0.0,
            status=_fact_status(known=side_obstacles is not None, stale=source_is_stale),
            metadata={**source_meta, 'side': side},
        ))

    if include_oracle_truth:
        fleet_state = fleet_safety_state_from_rails(rails)
        for block_id, occupant in sorted(fleet_state.block_occupancy.items()):
            facts.append(_fact(
                source=source,
                subject=block_id,
                predicate='occupancy',
                value=_entity_from_owner(occupant, block_id),
                timestamp=source_timestamp,
                confidence=1.0,
                status='stale' if source_is_stale else 'known',
                metadata={**source_meta, 'block_id': block_id},
            ))
    else:
        for entity_name, block_id in sorted(shuttle_block_by_entity.items()):
            facts.append(_fact(
                source=source,
                subject=block_id,
                predicate='occupancy',
                value=entity_name,
                timestamp=source_timestamp,
                confidence=0.9,
                status='stale' if source_is_stale else 'known',
                metadata={**source_meta, 'block_id': block_id},
            ))

    return facts


def _device_locations(
    rails: dict[str, Any],
    *,
    slot_sensor_by_side: dict[str, dict[str, str]],
    include_oracle_truth: bool,
) -> tuple[dict[str, dict[str, Any] | None], dict[str, str], dict[str, str]]:
    slot_occupancy: dict[str, dict[str, Any] | None] = {
        _slot_id(side, slot): None
        for side in SIDES
        for slot in SLOTS
    }
    shuttle_slot_by_entity: dict[str, str] = {}
    shuttle_block_by_entity: dict[str, str] = {}
    for side in SIDES:
        sensor_to_slot = {
            sensor.casefold(): slot
            for slot, sensor in slot_sensor_by_side[side].items()
        }
        side_rail = _side_rail(rails, side)
        if _has_sensor_snapshot(side_rail):
            for slot in SLOTS:
                slot_occupancy[_slot_id(side, slot)] = {
                    'occupied': False,
                    'shuttle': None,
                    'sensor': slot_sensor_by_side[side][slot],
                }
        for reading in _active_sensor_readings(side_rail):
            sensor_name = str(reading.get('name') or '').strip()
            slot = sensor_to_slot.get(sensor_name.casefold())
            shuttle_name = _normalise_shuttle_name(reading.get('shuttle'), side=side)
            raw_segment = reading.get('segment')
            if shuttle_name and raw_segment:
                block_id = normalize_fleet_block_id(raw_segment, side=side)
                if block_id:
                    shuttle_block_by_entity[shuttle_name] = block_id
            if not slot:
                continue
            subject = _slot_id(side, slot)
            slot_occupancy[subject] = {
                'occupied': True,
                'shuttle': shuttle_name or None,
                'sensor': sensor_name,
            }
            if shuttle_name:
                shuttle_slot_by_entity[shuttle_name] = subject
        if include_oracle_truth:
            for raw_name, state in _mapping(side_rail.get('shuttles')).items():
                if not isinstance(state, dict):
                    continue
                entity_name = _normalise_shuttle_name(raw_name, side=side)
                raw_slot = state.get('slot') or state.get('current_slot')
                if raw_slot and str(raw_slot) in SLOTS:
                    shuttle_slot_by_entity[entity_name] = _slot_id(side, str(raw_slot))
                raw_block = (
                    state.get('block')
                    or state.get('current_block')
                    or state.get('segment')
                    or state.get('current_segment')
                )
                if raw_block:
                    block_id = normalize_fleet_block_id(raw_block, side=side)
                    if block_id:
                        shuttle_block_by_entity[entity_name] = block_id
    return slot_occupancy, shuttle_slot_by_entity, shuttle_block_by_entity


def _active_sensor_readings(side_rail: dict[str, Any]) -> list[dict[str, Any]]:
    readings: list[dict[str, Any]] = []
    for key in ('active_position_sensors', 'active_sensors'):
        raw_items = side_rail.get(key, [])
        if not isinstance(raw_items, list):
            continue
        for item in raw_items:
            if isinstance(item, dict):
                readings.append(item)
    return readings


def _has_sensor_snapshot(side_rail: dict[str, Any]) -> bool:
    return any(isinstance(side_rail.get(key), list) for key in ('active_position_sensors', 'active_sensors'))


def _payload_entries(status: dict[str, Any], rails: dict[str, Any]) -> dict[str, dict[str, Any]]:
    entries: dict[str, dict[str, Any]] = {}
    payload_state = status.get('payload_state')
    by_shuttle = payload_state.get('by_shuttle', {}) if isinstance(payload_state, dict) else {}
    if isinstance(by_shuttle, dict):
        for entity_name, entry in by_shuttle.items():
            if isinstance(entry, dict):
                entries[str(entity_name)] = entry
    for side in SIDES:
        payloads = _mapping(_side_rail(rails, side).get('payloads'))
        for entity_name, entry in payloads.items():
            if isinstance(entry, dict):
                entries[str(entity_name)] = entry
    return entries


def _obstacle_entries(status: dict[str, Any], rails: dict[str, Any]) -> dict[str, set[str] | None]:
    result: dict[str, set[str] | None] = {side: None for side in SIDES}
    raw = status.get('obstacles')
    if isinstance(raw, dict):
        for side in SIDES:
            if side in raw:
                result[side] = _obstacle_names(raw.get(side))
    for side in SIDES:
        side_obstacles = _side_rail(rails, side).get('obstacles')
        if side_obstacles is not None:
            result[side] = _obstacle_names(side_obstacles)
    return result


def _obstacle_names(raw: Any) -> set[str]:
    if raw is None:
        return set()
    if isinstance(raw, dict):
        names = set()
        for name, value in raw.items():
            if isinstance(value, dict):
                present = bool(value.get('present', True))
            else:
                present = bool(value)
            if present:
                names.add(str(name))
        return names
    if isinstance(raw, list):
        names = set()
        for item in raw:
            if isinstance(item, dict):
                if bool(item.get('present', True)):
                    names.add(str(item.get('name') or item.get('id') or 'obstacle'))
            elif item:
                names.add(str(item))
        return names
    if raw:
        return {'obstacle'}
    return set()


def _fact(
    *,
    source: str,
    subject: str,
    predicate: str,
    value: Any,
    timestamp: float,
    confidence: float,
    status: str,
    metadata: dict[str, Any],
) -> ObservedFact:
    return ObservedFact(
        fact_id=f'{source}-{_slug(subject)}-{_slug(predicate)}',
        subject=subject,
        predicate=predicate,
        value=copy.deepcopy(value),
        source=source,
        timestamp=timestamp,
        confidence=max(0.0, min(1.0, float(confidence))),
        status=status,
        metadata=copy.deepcopy(metadata),
    )


def _ensure_visual_fact(fact: ObservedFact) -> ObservedFact:
    if not isinstance(fact, ObservedFact):
        raise TypeError('visual_facts must contain ObservedFact')
    if fact.source != 'visual_model':
        raise ValueError('visual_facts must use source visual_model')
    return fact


def _source_metadata(
    *,
    source: str,
    source_timestamp: float,
    observed_at: float,
    stale_after_s: float,
    include_oracle_truth: bool,
) -> dict[str, Any]:
    metadata = {
        'source_timestamp': source_timestamp,
        'age_s': round(max(0.0, observed_at - source_timestamp), 6),
        'stale_after_s': stale_after_s,
    }
    if source == 'oracle':
        metadata.update({
            'oracle_only': True,
            'model_input_exposure': 'excluded',
            'input_stream': 'gazebo_truth',
        })
    elif source == 'trusted_device':
        metadata.update({
            'oracle_only': False,
            'model_input_exposure': 'fused_planner_state_only',
            'input_stream': (
                'trusted_device_status'
                if not include_oracle_truth
                else 'trusted_device_status_with_oracle'
            ),
        })
    return metadata


def _source_timestamp(
    status: dict[str, Any],
    source: str,
    observed_at: float,
    source_timestamps: dict[str, float],
) -> float:
    if source in source_timestamps:
        return _timestamp_or_zero(source_timestamps[source])
    for key in ('observed_at', 'timestamp', 'stamp', 'updated_at'):
        if key in status:
            return _timestamp_or_zero(status.get(key))
    return observed_at


def _timestamp_or_zero(value: Any) -> float:
    if value is None:
        return 0.0
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(parsed) or parsed < 0.0:
        return 0.0
    return parsed


def _is_stale(source_timestamp: float, observed_at: float, stale_after_s: float) -> bool:
    return observed_at - source_timestamp > max(0.0, stale_after_s)


def _fact_status(*, known: bool, stale: bool) -> str:
    if stale:
        return 'stale'
    return 'known' if known else 'unknown'


def _status_rank(status: str) -> int:
    if status in {'known', 'conflicting'}:
        return 0
    if status == 'unknown':
        return 1
    return 2


def _normalise_slot_sensor_map(
    raw: dict[str, dict[str, str]] | None,
) -> dict[str, dict[str, str]]:
    mapping = copy.deepcopy(DEFAULT_SLOT_SENSOR_BY_SIDE)
    if not isinstance(raw, dict):
        return mapping
    for side in SIDES:
        side_mapping = raw.get(side)
        if not isinstance(side_mapping, dict):
            continue
        for slot in SLOTS:
            sensor = side_mapping.get(slot)
            if sensor:
                mapping[side][slot] = str(sensor)
    return mapping


def _side_rail(rails: dict[str, Any], side: str) -> dict[str, Any]:
    rail = rails.get(side, {})
    return rail if isinstance(rail, dict) else {}


def _mapping(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _nested_dict(data: dict[str, Any], *keys: str) -> dict[str, Any]:
    value = _nested_value(data, *keys)
    return value if isinstance(value, dict) else {}


def _nested_value(data: dict[str, Any], *keys: str) -> Any:
    value: Any = data
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _normalise_shuttle_name(raw: Any, *, side: str) -> str:
    spec = normalize_shuttle_ref(raw, side=side)
    if spec is not None:
        return spec.gazebo_entity_name
    return str(raw or '').strip()


def _entity_from_owner(owner: str, block_id: str) -> str:
    side = block_id.split(':', 1)[0] if ':' in block_id else ''
    spec = normalize_shuttle_ref(owner, side=side or None)
    if spec is not None:
        return spec.gazebo_entity_name
    return str(owner)


def _normalise_switch_state(value: Any) -> str | None:
    text = str(value or '').strip().casefold()
    if text in {'e', 'exterior', 'external'}:
        return 'EXTERIOR'
    if text in {'i', 'interior', 'internal'}:
        return 'INTERIOR'
    return str(value).strip() if value not in {None, ''} else None


def _normalise_stopper_state(value: Any) -> str | None:
    text = str(value or '').strip().casefold()
    if text in {'0', 'open', 'opened', 'false'}:
        return 'open'
    if text in {'1', 'closed', 'close', 'true'}:
        return 'closed'
    return str(value).strip() if value not in {None, ''} else None


def _slot_id(side: str, slot: str) -> str:
    return f'{side}:slot:{slot}'


def _slug(value: str) -> str:
    return re.sub(r'[^A-Za-z0-9_.-]+', '-', str(value)).strip('-') or 'fact'
