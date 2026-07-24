#!/usr/bin/env python3
"""Room 315 multi-shuttle research utilities.

This module contains deterministic helpers shared by the VLA supervisor,
dataset tooling, PlanSys scenario generation, and tests. It is intentionally
free of ROS imports so it can be used in CI without a running simulator.
"""

from __future__ import annotations

import json
import re
from dataclasses import asdict
from dataclasses import dataclass
from dataclasses import field
from pathlib import Path
from typing import Any

import yaml


SIDES = ('right', 'left')
MAX_SHUTTLES_PER_SIDE = 4
GLOBAL_SHUTTLE_INDEX = -1
DEVICE_NAMES = ('A1', 'A2', 'A3', 'A4')

SIDE_IDS = {'right': 0, 'left': 1}

PRIMITIVE_IDS = {
    'WAIT': 0,
    'DONE': 1,
    'SET_SWITCHES': 2,
    'SET_STOPPERS': 3,
    'SHUTTLE_ON': 4,
    'STOP_NOW': 5,
    'EMERGENCY_STOP': 6,
}

SWITCH_VALUE_IDS = {'UNCHANGED': 0, 'EXTERIOR': 1, 'INTERIOR': 2}
STOPPER_VALUE_IDS = {'UNCHANGED': 0, 'open': 1, 'closed': 2}

WAIT_CONDITION_IDS = {
    'none': 0,
    'switch_state_match': 1,
    'stopper_state_match': 2,
    'shuttle_command_applied': 3,
    'task_terminal_status': 4,
    'reserved_wait_5': 5,
    'terminal': 6,
    'target_sensor_active': 7,
    'block_clearance': 8,
    'headway_clearance': 9,
}

TARGET_IDS = {
    'none': 0,
    'A1': 1,
    'A2': 2,
    'A3': 3,
    'A4': 4,
    'ALL_SWITCHES': 5,
    'ALL_STOPPERS': 6,
    'MULTIPLE_DEVICES': 7,
    'right_shuttle': 8,
    'left_shuttle': 9,
    'reserved_target_10': 10,
    'reserved_target_11': 11,
    'reserved_target_12': 12,
    'reserved_target_13': 13,
    'reserved_target_14': 14,
    'reserved_target_15': 15,
    'reserved_target_16': 16,
    'terminal': 17,
    'DZI1R': 18,
    'DZI2R': 19,
    'DZI3R': 20,
    'DZI4R': 21,
    'DZI1L': 22,
    'DZI2L': 23,
    'DZI3L': 24,
    'DZI4L': 25,
    'DA3IR': 26,
    'DA3IL': 27,
}
for _side in SIDES:
    for _index in range(1, MAX_SHUTTLES_PER_SIDE + 1):
        TARGET_IDS[f'{_side}_shuttle_{_index}'] = len(TARGET_IDS)
REASON_IDS = {
    'none': 0,
    'command_event': 1,
    'reserved_reason_2': 2,
    'reserved_reason_3': 3,
    'task_succeeded': 4,
    'task_failed': 5,
    'episode_stopped': 6,
    'episode_discarded': 7,
    'switch_update': 8,
    'stopper_update': 9,
    'shuttle_start': 10,
    'shuttle_stop': 11,
    'emergency': 12,
    'unsupported_command': 13,
    'target_station_route': 14,
    'wait_for_block_clearance': 15,
    'maintain_headway': 16,
    'avoid_collision': 17,
    'avoid_deadlock': 18,
    'obstacle_stop': 19,
    'wrong_shuttle_rejected': 20,
    'fleet_coordination': 21,
}

COORDINATION_MODE_IDS = {
    'normal': 0,
    'guarded_motion': 1,
    'wait_for_clearance': 2,
    'reservation_based_move': 3,
    'fleet_coordination': 4,
    'emergency': 5,
}

MODEL_INPUT_SCHEMA_VERSION = 3
DEFAULT_SHUTTLE_LENGTH_M = 0.36
DEFAULT_ROUTE_SAFETY_MARGIN_M = 0.05


@dataclass(frozen=True)
class ShuttleSpec:
    shuttle_id: str
    short_id: str
    side: str
    shuttle_index: int
    gazebo_entity_name: str


def validate_shuttle_count(count: Any) -> int:
    parsed = int(count)
    if parsed < 0 or parsed > MAX_SHUTTLES_PER_SIDE:
        raise ValueError(f'shuttle_count must be in 0..{MAX_SHUTTLES_PER_SIDE}, got {count!r}')
    return parsed


def shuttle_id(side: str, shuttle_index: int) -> str:
    side = normalize_side(side)
    index = validate_shuttle_index(shuttle_index)
    return f'{side}_shuttle_{index}'


def short_shuttle_id(side: str, shuttle_index: int) -> str:
    side = normalize_side(side)
    index = validate_shuttle_index(shuttle_index)
    return f'{"R" if side == "right" else "L"}{index}'


def gazebo_entity_name(side: str, shuttle_index: int) -> str:
    return f'room315_{shuttle_id(side, shuttle_index)}'


def shuttle_specs_for_side(side: str, count: Any) -> list[ShuttleSpec]:
    count = validate_shuttle_count(count)
    return [
        ShuttleSpec(
            shuttle_id=shuttle_id(side, index),
            short_id=short_shuttle_id(side, index),
            side=normalize_side(side),
            shuttle_index=index - 1,
            gazebo_entity_name=gazebo_entity_name(side, index),
        )
        for index in range(1, count + 1)
    ]


def all_shuttle_specs() -> list[ShuttleSpec]:
    return [
        spec
        for side in SIDES
        for spec in shuttle_specs_for_side(side, MAX_SHUTTLES_PER_SIDE)
    ]


def normalize_side(raw: Any) -> str:
    text = str(raw or '').strip().casefold()
    if text in {'right', 'r', 'droit', 'droite'}:
        return 'right'
    if text in {'left', 'l', 'gauche'}:
        return 'left'
    raise ValueError(f'invalid rail side {raw!r}; expected right or left')


def validate_shuttle_index(index: Any) -> int:
    parsed = int(index)
    if parsed < 1 or parsed > MAX_SHUTTLES_PER_SIDE:
        raise ValueError(f'shuttle index must be in 1..{MAX_SHUTTLES_PER_SIDE}, got {index!r}')
    return parsed


def parse_start_slots(raw: Any, *, count: Any, default_slot: Any = '2') -> list[str]:
    count = validate_shuttle_count(count)
    if count == 0:
        return []
    text = str(raw or '').strip()
    if text:
        slots = [part.strip() for part in text.split(',') if part.strip()]
    elif count == 1:
        slots = [str(default_slot)]
    else:
        slots = [str(slot) for slot in range(1, count + 1)]
    if len(slots) != count:
        raise ValueError(f'{count} shuttle(s) require exactly {count} start slot(s), got {slots}')
    invalid = [slot for slot in slots if slot not in {'1', '2', '3', '4'}]
    if invalid:
        raise ValueError(f'invalid Room 315 start slot(s): {invalid}')
    if len(set(slots)) != len(slots):
        raise ValueError(f'duplicate Room 315 start slots are not allowed: {slots}')
    return slots


def normalize_shuttle_ref(raw: Any, *, side: str | None = None) -> ShuttleSpec | None:
    text = str(raw or '').strip()
    if not text:
        return None
    lowered = text.casefold()
    match = re.fullmatch(r'([rl])([1-4])', lowered)
    if match:
        ref_side = 'right' if match.group(1) == 'r' else 'left'
        return shuttle_specs_for_side(ref_side, int(match.group(2)))[-1]
    match = re.fullmatch(r'(?:room315_)?(right|left)_shuttle_?([1-4])', lowered)
    if match:
        return shuttle_specs_for_side(match.group(1), int(match.group(2)))[-1]
    match = re.fullmatch(r'(right|left)_shuttle', lowered)
    if match and side:
        return shuttle_specs_for_side(side, 1)[0]
    return None


class ShuttleRegistry:
    def __init__(self, *, right_count: int = 1, left_count: int = 1) -> None:
        self.specs = {
            spec.shuttle_id: spec
            for side, count in (('right', right_count), ('left', left_count))
            for spec in shuttle_specs_for_side(side, count)
        }
        self._by_short = {spec.short_id: spec for spec in self.specs.values()}

    @classmethod
    def from_rails(cls, rails: dict[str, Any]) -> 'ShuttleRegistry':
        counts = {}
        for side in SIDES:
            shuttles = (rails.get(side, {}) or {}).get('shuttles', {})
            counts[side] = len(shuttles) if isinstance(shuttles, dict) else 0
        return cls(right_count=counts.get('right', 0), left_count=counts.get('left', 0))

    def resolve(self, raw: Any, *, side: str | None = None) -> ShuttleSpec | None:
        spec = normalize_shuttle_ref(raw, side=side)
        if spec and spec.shuttle_id in self.specs:
            return self.specs[spec.shuttle_id]
        text = str(raw or '').strip().upper()
        return self._by_short.get(text)

    def count_on_side(self, side: str) -> int:
        side = normalize_side(side)
        return sum(1 for spec in self.specs.values() if spec.side == side)

    def is_multi_shuttle_side(self, side: str) -> bool:
        return self.count_on_side(side) > 1


@dataclass
class ShuttleTaskState:
    shuttle_id: str
    active_task_id: str = ''
    target_slot: str = ''
    target_block: str = ''
    wait_time_s: float = 0.0


@dataclass
class FleetSafetyState:
    block_occupancy: dict[str, str]
    block_reservations: dict[str, str]
    station_slot_targets: dict[str, str]
    min_headway_blocks: int = 1
    shuttle_blocks: dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class RailSlotLocation:
    slot: str
    segment: str
    s_ratio: float


@dataclass(frozen=True)
class RailRouteBlock:
    side: str
    segment: str
    start_s_ratio: float
    end_s_ratio: float

    @property
    def block_id(self) -> str:
        return normalize_fleet_block_id(self.segment, side=self.side)

    def contains(self, segment: Any, s_ratio: Any = None) -> bool:
        if _segment_name(segment) != self.segment:
            return False
        ratio = _optional_float(s_ratio)
        if ratio is None:
            return True
        low = min(self.start_s_ratio, self.end_s_ratio)
        high = max(self.start_s_ratio, self.end_s_ratio)
        return low <= ratio <= high

    def overlaps(
        self,
        segment: Any,
        start_s_ratio: Any,
        end_s_ratio: Any,
    ) -> bool:
        if _segment_name(segment) != self.segment:
            return False
        start = _optional_float(start_s_ratio)
        end = _optional_float(end_s_ratio)
        if start is None or end is None:
            return True
        route_low = min(self.start_s_ratio, self.end_s_ratio)
        route_high = max(self.start_s_ratio, self.end_s_ratio)
        occupied_low = min(start, end)
        occupied_high = max(start, end)
        return occupied_high >= route_low and occupied_low <= route_high


@dataclass(frozen=True)
class RailRouteBlocker:
    shuttle_id: str
    owner: str
    side: str
    segment: str
    s_ratio: float | None
    block_id: str
    reason: str
    occupancy_start_s_ratio: float | None = None
    occupancy_end_s_ratio: float | None = None


@dataclass(frozen=True)
class RailTopology:
    side: str
    routing_table: dict[str, dict[str, Any]]
    fixed_transitions: dict[str, str]
    slots: dict[str, RailSlotLocation]
    default_switch_state: str = 'E'


class BlockReservationTable:
    def __init__(self, reservations: dict[str, str] | None = None) -> None:
        self.reservations = dict(reservations or {})

    def reserve(self, block_id: str, shuttle_id_value: str) -> None:
        owner = self.reservations.get(block_id)
        if owner and owner != shuttle_id_value:
            raise ValueError(f'block {block_id} is already reserved by {owner}')
        self.reservations[block_id] = shuttle_id_value

    def owner(self, block_id: str) -> str:
        return self.reservations.get(block_id, '')

    def release(self, block_id: str, shuttle_id_value: str) -> None:
        if self.reservations.get(block_id) == shuttle_id_value:
            self.reservations.pop(block_id, None)


class RailBlockOccupancyModel:
    def __init__(self, occupancy: dict[str, str] | None = None) -> None:
        self.occupancy = dict(occupancy or {})

    def occupant(self, block_id: str) -> str:
        return self.occupancy.get(str(block_id), '')


@dataclass
class FleetSafetyMetrics:
    wrong_shuttle_command_rate: float = 0.0
    shuttle_id_accuracy: float | None = None
    headway_violation_count: int = 0
    block_occupancy_violation_count: int = 0
    block_reservation_rejection_count: int = 0
    deadlock_detected_count: int = 0
    deadlock_avoided_count: int = 0
    multi_shuttle_task_success: float | None = None
    per_shuttle_task_success: dict[str, float] | None = None
    fleet_throughput_tasks_per_minute: float | None = None
    average_wait_time_by_shuttle: dict[str, float] | None = None

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


class DeadlockDetector:
    def detect(self, state: FleetSafetyState) -> bool:
        for block, owner in state.block_reservations.items():
            occupant = state.block_occupancy.get(block)
            if occupant and occupant != owner:
                return True
        return False


def empty_fleet_safety_metrics() -> dict[str, Any]:
    return FleetSafetyMetrics(per_shuttle_task_success={}, average_wait_time_by_shuttle={}).as_dict()


def fleet_safety_state_from_rails(
    rails: dict[str, Any],
    *,
    block_reservations: dict[str, str] | None = None,
    station_slot_targets: dict[str, str] | None = None,
    min_headway_blocks: int = 1,
) -> FleetSafetyState:
    """Build privileged runtime fleet-safety state from supervisor rail status.

    This state is for safety/evaluation only. It must stay outside model_input.
    Blocks are represented by the rail segment when a richer block graph is not
    available yet.
    """

    occupancy: dict[str, str] = {}
    shuttle_blocks: dict[str, str] = {}
    for side in SIDES:
        rail = rails.get(side, {}) if isinstance(rails.get(side), dict) else {}
        shuttles = rail.get('shuttles', {}) if isinstance(rail.get('shuttles'), dict) else {}
        for raw_name, shuttle_state in shuttles.items():
            if not isinstance(shuttle_state, dict):
                continue
            owner = _short_owner(raw_name, side=side) or str(raw_name)
            raw_block = (
                shuttle_state.get('block')
                or shuttle_state.get('current_block')
                or shuttle_state.get('segment')
                or shuttle_state.get('current_segment')
                or ''
            )
            block_id = normalize_fleet_block_id(raw_block, side=side)
            if not block_id:
                continue
            occupancy[block_id] = owner
            shuttle_blocks[owner] = block_id
    return FleetSafetyState(
        block_occupancy=occupancy,
        block_reservations=dict(block_reservations or {}),
        station_slot_targets=dict(station_slot_targets or {}),
        min_headway_blocks=max(int(min_headway_blocks), 0),
        shuttle_blocks=shuttle_blocks,
    )


def normalize_fleet_block_id(raw: Any, *, side: str) -> str:
    text = str(raw or '').strip()
    if not text:
        return ''
    side = normalize_side(side)
    if ':' in text:
        return text
    if text.startswith(f'{side}_'):
        text = text.removeprefix(f'{side}_')
    return f'{side}:{text}'


def normalize_fleet_slot_id(raw: Any, *, side: str) -> str:
    text = str(raw or '').strip()
    if not text:
        return ''
    side = normalize_side(side)
    if ':' in text:
        return text
    if text.startswith(f'{side}_'):
        text = text.removeprefix(f'{side}_')
    return f'{side}:slot:{text}'


def load_rail_topology(
    network_path: Path | str,
    devices_path: Path | str,
    *,
    side: str | None = None,
    default_switch_state: str = 'E',
) -> RailTopology:
    """Load the directed Room 315 rail graph and slot positions.

    The returned topology is deliberately small: it contains enough structure to
    answer "which physical segments does this slot-to-slot move occupy?" without
    pulling in ROS or the kinematic simulator.
    """

    network = yaml.safe_load(Path(network_path).expanduser().read_text(encoding='utf-8')) or {}
    devices = yaml.safe_load(Path(devices_path).expanduser().read_text(encoding='utf-8')) or {}
    if not isinstance(network, dict):
        raise ValueError(f'{network_path} must contain a YAML mapping')
    if not isinstance(devices, dict):
        raise ValueError(f'{devices_path} must contain a YAML mapping')

    rail_side = normalize_side(side or devices.get('rail_side') or 'right')
    routing_table = {
        _segment_name(segment): dict(entry or {})
        for segment, entry in (network.get('routing_table') or {}).items()
        if _segment_name(segment)
    }
    fixed_transitions = {
        _segment_name(segment): _segment_name(next_segment)
        for segment, next_segment in (network.get('fixed_transitions') or {}).items()
        if _segment_name(segment) and _segment_name(next_segment)
    }
    slots: dict[str, RailSlotLocation] = {}
    for entry in devices.get('slots') or []:
        if not isinstance(entry, dict):
            continue
        slot = _slot_symbol(entry.get('name'))
        segment = _segment_name(entry.get('segment'))
        s_ratio = _optional_float(entry.get('s_ratio'))
        if not slot or not segment or s_ratio is None:
            raise ValueError(f'invalid slot entry in {devices_path}: {entry!r}')
        if not 0.0 <= s_ratio <= 1.0:
            raise ValueError(f'slot {entry.get("name")!r} s_ratio must be in [0.0, 1.0]')
        slots[slot] = RailSlotLocation(slot=slot, segment=segment, s_ratio=s_ratio)
    return RailTopology(
        side=rail_side,
        routing_table=routing_table,
        fixed_transitions=fixed_transitions,
        slots=slots,
        default_switch_state=str(default_switch_state or 'E').strip().upper(),
    )


def route_blocks_between_slots(
    topology: RailTopology,
    source_slot: Any,
    target_slot: Any,
    *,
    switch_states: dict[str, Any] | None = None,
    max_segments: int = 32,
) -> list[RailRouteBlock]:
    source = _slot_location(topology, source_slot)
    target = _slot_location(topology, target_slot)
    if source.segment == target.segment and source.s_ratio <= target.s_ratio:
        return [
            RailRouteBlock(
                side=topology.side,
                segment=source.segment,
                start_s_ratio=source.s_ratio,
                end_s_ratio=target.s_ratio,
            )
        ]

    blocks: list[RailRouteBlock] = []
    current_segment = source.segment
    start_ratio = source.s_ratio
    first_segment = True
    for _step in range(max_segments):
        if (
            current_segment == target.segment
            and not (
                first_segment
                and source.segment == target.segment
                and source.s_ratio > target.s_ratio
            )
        ):
            blocks.append(
                RailRouteBlock(
                    side=topology.side,
                    segment=current_segment,
                    start_s_ratio=start_ratio,
                    end_s_ratio=target.s_ratio,
                )
            )
            return blocks
        blocks.append(
            RailRouteBlock(
                side=topology.side,
                segment=current_segment,
                start_s_ratio=start_ratio,
                end_s_ratio=1.0,
            )
        )
        next_segment = _next_route_segment(topology, current_segment, switch_states or {})
        if not next_segment:
            raise ValueError(
                f'no route from slot {source.slot} to slot {target.slot}: '
                f'{current_segment} has no valid successor'
            )
        current_segment = next_segment
        start_ratio = 0.0
        first_segment = False
    raise ValueError(
        f'no route from slot {source.slot} to slot {target.slot} within {max_segments} segments'
    )


def route_blockers_from_rails(
    rails: dict[str, Any],
    topology: RailTopology,
    source_slot: Any,
    target_slot: Any,
    *,
    selected_shuttle: Any = None,
    side: str | None = None,
    switch_states: dict[str, Any] | None = None,
) -> list[RailRouteBlocker]:
    rail_side = normalize_side(side or topology.side)
    route_blocks = route_blocks_between_slots(
        topology,
        source_slot,
        target_slot,
        switch_states=switch_states,
    )
    selected_labels = _owner_labels(selected_shuttle, side=rail_side)
    rail = rails.get(rail_side, {}) if isinstance(rails, dict) else {}
    shuttles = rail.get('shuttles', {}) if isinstance(rail, dict) else {}
    if not isinstance(shuttles, dict):
        return []

    blockers: list[RailRouteBlocker] = []
    for raw_name, shuttle_state in shuttles.items():
        if not isinstance(shuttle_state, dict):
            continue
        owner = _short_owner(raw_name, side=rail_side) or str(raw_name)
        if _owner_labels(raw_name, side=rail_side) & selected_labels:
            continue
        segment = _segment_name(
            shuttle_state.get('segment')
            or shuttle_state.get('current_segment')
            or shuttle_state.get('block')
            or shuttle_state.get('current_block')
        )
        if not segment:
            continue
        s_ratio = _shuttle_s_ratio(shuttle_state)
        occupancy_bounds = _shuttle_occupancy_ratio_bounds(shuttle_state)
        for block in route_blocks:
            overlap = (
                block.overlaps(segment, *occupancy_bounds)
                if occupancy_bounds is not None
                else block.contains(segment, s_ratio)
            )
            if overlap:
                interval_available = (
                    occupancy_bounds is not None
                    and occupancy_bounds[0] != occupancy_bounds[1]
                )
                blockers.append(
                    RailRouteBlocker(
                        shuttle_id=owner,
                        owner=owner,
                        side=rail_side,
                        segment=segment,
                        s_ratio=s_ratio,
                        block_id=block.block_id,
                        reason=(
                            'route_occupancy_interval_overlap'
                            if interval_available
                            else 'route_segment_overlap'
                            if s_ratio is not None
                            else 'route_segment_overlap_unknown_position'
                        ),
                        occupancy_start_s_ratio=(
                            occupancy_bounds[0]
                            if occupancy_bounds is not None
                            else None
                        ),
                        occupancy_end_s_ratio=(
                            occupancy_bounds[1]
                            if occupancy_bounds is not None
                            else None
                        ),
                    )
                )
                break
    return blockers


def normalize_event_action(action: dict[str, Any]) -> dict[str, Any]:
    normalized = dict(action or {})
    primitive = str(normalized.get('primitive') or 'WAIT').strip().upper()
    side = normalize_side(normalized.get('side', 'right'))
    shuttle_spec = normalize_shuttle_ref(
        normalized.get('shuttle_id') or normalized.get('shuttle') or normalized.get('target_id'),
        side=side,
    )
    shuttle_index = int(normalized.get('shuttle_index', GLOBAL_SHUTTLE_INDEX))
    if shuttle_spec is not None:
        shuttle_index = shuttle_spec.shuttle_index
        normalized['shuttle_id'] = shuttle_spec.short_id
    elif shuttle_index < 0 and primitive not in {'WAIT', 'DONE', 'EMERGENCY_STOP', 'SET_SWITCHES', 'SET_STOPPERS'}:
        raise ValueError('multi-shuttle action needs shuttle_id or shuttle_index')
    normalized['primitive'] = primitive if primitive in PRIMITIVE_IDS else 'WAIT'
    normalized['side'] = side
    normalized['shuttle_index'] = shuttle_index
    normalized['switch_mask'] = device_map(normalized.get('switch_mask'), default=0)
    normalized['switch_values'] = device_map(normalized.get('switch_values'), default='UNCHANGED')
    normalized['stopper_mask'] = device_map(normalized.get('stopper_mask'), default=0)
    normalized['stopper_values'] = device_map(normalized.get('stopper_values'), default='UNCHANGED')
    normalized['speed_mps'] = round(float(normalized.get('speed_mps') or normalized.get('speed') or 0.0), 4)
    normalized['wait_condition'] = _known(normalized.get('wait_condition'), WAIT_CONDITION_IDS, 'none')
    normalized['target_id'] = _known_target(normalized.get('target_id'), side=side, shuttle_index=shuttle_index)
    normalized['reason'] = _known(normalized.get('reason'), REASON_IDS, 'none')
    normalized['coordination_mode'] = _known(
        normalized.get('coordination_mode'),
        COORDINATION_MODE_IDS,
        'normal',
    )
    return normalized

def validate_fleet_command(
    command: dict[str, Any],
    *,
    registry: ShuttleRegistry,
    fleet_state: FleetSafetyState | None = None,
) -> tuple[bool, str]:
    action = str(command.get('action') or '').strip().lower()
    if action in {'emergency_stop', 'stop_all', 'status', 'snapshot'}:
        return True, ''
    side = normalize_side(command.get('side', 'right'))
    if action == 'shuttle' and registry.is_multi_shuttle_side(side):
        spec = registry.resolve(command.get('shuttle') or command.get('shuttle_id') or command.get('name'), side=side)
        if spec is None:
            return False, 'multi-shuttle command is ambiguous: shuttle_id/shuttle/name is required'
        command_name = str(command.get('command') or '').strip().upper()
        raw_next_block = str(command.get('next_block') or command.get('target_block') or '').strip()
        raw_target_slot = str(command.get('target_slot') or command.get('slot') or '').strip()
        next_block = normalize_fleet_block_id(raw_next_block, side=side)
        target_slot = normalize_fleet_slot_id(raw_target_slot, side=side)
        if fleet_state is not None:
            owner_labels = {spec.short_id, spec.shuttle_id, spec.gazebo_entity_name}
            if command_name == 'ON':
                deadlock_reason = _deadlock_tie_breaker_reason(fleet_state, spec.short_id)
                if deadlock_reason:
                    return False, deadlock_reason
            if command_name == 'ON' and next_block:
                occupant = _fleet_mapping_lookup(
                    fleet_state.block_occupancy,
                    next_block,
                    raw_next_block,
                )
                if occupant and occupant not in owner_labels:
                    return False, f'next block {next_block} is occupied by {occupant}'
                owner = _fleet_mapping_lookup(
                    fleet_state.block_reservations,
                    next_block,
                    raw_next_block,
                )
                if owner and owner not in owner_labels:
                    return False, f'next block {next_block} is reserved by {owner}'
            if command_name == 'ON' and target_slot:
                owner = _fleet_mapping_lookup(
                    fleet_state.station_slot_targets,
                    target_slot,
                    raw_target_slot,
                )
                if owner and owner not in owner_labels:
                    return False, f'target slot {target_slot} is already targeted by {owner}'
            if command_name == 'ON':
                if command.get('headway_clearance_ok') is False:
                    return False, 'minimum headway clearance is not available'
                headway_blocks = _optional_int(command.get('headway_blocks_ahead'))
                if (
                    headway_blocks is not None
                    and headway_blocks < int(fleet_state.min_headway_blocks)
                ):
                    return False, (
                        f'headway violation: {headway_blocks} block(s) ahead, '
                        f'minimum is {fleet_state.min_headway_blocks}'
                    )
    return True, ''


def _deadlock_tie_breaker_reason(state: FleetSafetyState, owner: str) -> str:
    if not owner or not DeadlockDetector().detect(state):
        return ''
    participants: set[str] = set()
    for block, reserved_owner in state.block_reservations.items():
        occupant = state.block_occupancy.get(block, '')
        if occupant and reserved_owner and occupant != reserved_owner:
            participants.update({occupant, reserved_owner})
    if owner not in participants or len(participants) < 2:
        return ''
    winner = sorted(participants)[0]
    if owner == winner:
        return ''
    return (
        f'deadlock detected: deterministic tie-breaker grants priority to {winner}; '
        f'{owner} must safe-stop, reobserve, and replan'
    )


def _segment_name(raw: Any) -> str:
    text = str(raw or '').strip()
    if not text:
        return ''
    if ':' in text:
        text = text.rsplit(':', 1)[-1]
    lowered = text.casefold()
    for prefix in ('right_', 'left_'):
        if lowered.startswith(prefix):
            text = text[len(prefix):]
            break
    text = text.strip().upper()
    if re.fullmatch(r'A(?:14|23)[EI]', text):
        return text[:-1]
    return text


def _slot_symbol(raw: Any) -> str:
    text = str(raw or '').strip().casefold()
    if not text:
        return ''
    if ':' in text:
        text = text.rsplit(':', 1)[-1]
    text = text.replace('-', '_')
    if text.startswith('slot_'):
        text = text.removeprefix('slot_')
    elif text.startswith('slot'):
        text = text.removeprefix('slot')
    return text if text in {'1', '2', '3', '4'} else ''


def _slot_location(topology: RailTopology, raw_slot: Any) -> RailSlotLocation:
    slot = _slot_symbol(raw_slot)
    if not slot or slot not in topology.slots:
        raise ValueError(f'unknown Room 315 slot {raw_slot!r} on {topology.side} rail')
    return topology.slots[slot]


def _next_route_segment(
    topology: RailTopology,
    segment: Any,
    switch_states: dict[str, Any],
) -> str:
    segment_name = _segment_name(segment)
    entry = topology.routing_table.get(segment_name)
    raw_next = ''
    if isinstance(entry, dict):
        route_type = str(entry.get('type') or '').strip().casefold()
        if route_type == 'fixed':
            raw_next = entry.get('next_segment', '')
        elif route_type in {'switch_select', 'switch_guard'}:
            switch_name = str(entry.get('switch') or '').strip().upper()
            state = str(
                switch_states.get(switch_name, topology.default_switch_state)
                if switch_name
                else topology.default_switch_state
            ).strip().upper()
            by_state = entry.get('by_state') if isinstance(entry.get('by_state'), dict) else {}
            raw_next = by_state.get(state, entry.get('on_unknown_state', ''))
        else:
            raw_next = entry.get('next_segment', '')
    if not raw_next:
        raw_next = topology.fixed_transitions.get(segment_name, '')
    next_segment = _segment_name(raw_next)
    return '' if next_segment == 'FALLING' else next_segment


def _owner_labels(raw_name: Any, *, side: str) -> set[str]:
    text = str(raw_name or '').strip()
    labels = {text} if text else set()
    spec = normalize_shuttle_ref(text, side=side)
    if spec is not None:
        labels.update({spec.short_id, spec.shuttle_id, spec.gazebo_entity_name})
    return {label for label in labels if label}


def _shuttle_s_ratio(shuttle_state: dict[str, Any]) -> float | None:
    rail_position = shuttle_state.get('rail_position')
    if isinstance(rail_position, dict):
        ratio = _optional_float(rail_position.get('s_ratio'))
        if ratio is not None and bool(rail_position.get('available', True)):
            return max(0.0, min(1.0, ratio))
    for key in ('s_ratio', 'position_ratio', 'normalized_position', 'progress_ratio'):
        ratio = _optional_float(shuttle_state.get(key))
        if ratio is not None:
            return max(0.0, min(1.0, ratio))
    return None


def _shuttle_occupancy_ratio_bounds(
    shuttle_state: dict[str, Any],
    *,
    shuttle_length_m: float = DEFAULT_SHUTTLE_LENGTH_M,
    safety_margin_m: float = DEFAULT_ROUTE_SAFETY_MARGIN_M,
) -> tuple[float, float] | None:
    explicit_start = _optional_float(shuttle_state.get('occupancy_start_s_ratio'))
    explicit_end = _optional_float(shuttle_state.get('occupancy_end_s_ratio'))
    if explicit_start is not None and explicit_end is not None:
        return (
            max(0.0, min(1.0, explicit_start)),
            max(0.0, min(1.0, explicit_end)),
        )
    ratio = _shuttle_s_ratio(shuttle_state)
    if ratio is None:
        return None
    rail_position = shuttle_state.get('rail_position')
    segment_length_m = None
    uncertainty_m = 0.0
    if isinstance(rail_position, dict):
        segment_length_m = _optional_float(rail_position.get('segment_length_m'))
        uncertainty_m = _optional_float(
            rail_position.get('position_uncertainty_m')
        ) or 0.0
    if segment_length_m is None:
        segment_length_m = _optional_float(shuttle_state.get('segment_length_m'))
    if segment_length_m is None or segment_length_m <= 0.0:
        return ratio, ratio
    half_extent_ratio = (
        float(shuttle_length_m) / 2.0
        + float(safety_margin_m)
        + max(0.0, uncertainty_m)
    ) / segment_length_m
    return (
        max(0.0, ratio - half_extent_ratio),
        min(1.0, ratio + half_extent_ratio),
    )


def _optional_float(value: Any) -> float | None:
    if value is None or value == '':
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _optional_int(value: Any) -> int | None:
    if value is None or value == '':
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _fleet_mapping_lookup(mapping: dict[str, str], normalized: str, raw: str) -> str:
    if normalized in mapping:
        return mapping.get(normalized, '')
    if raw in mapping:
        return mapping.get(raw, '')
    legacy = normalized.replace(':', '_')
    return mapping.get(legacy, '')


def _short_owner(raw_name: Any, *, side: str) -> str:
    spec = normalize_shuttle_ref(raw_name, side=side)
    return '' if spec is None else spec.short_id


def marker_id_for(side: str, shuttle_index: int, role_index: int) -> int:
    base = 100 if normalize_side(side) == 'right' else 200
    return base + ((validate_shuttle_index(shuttle_index) - 1) * 10) + int(role_index)


def default_identity_config() -> dict[str, Any]:
    roles = ('front_left', 'front_right', 'rear_left', 'rear_right')
    shuttles = []
    colors = {
        'R1': 'red',
        'R2': 'blue',
        'R3': 'green',
        'R4': 'yellow',
        'L1': 'cyan',
        'L2': 'magenta',
        'L3': 'orange',
        'L4': 'white',
    }
    for spec in all_shuttle_specs():
        tag_ids = {
            role: marker_id_for(spec.side, spec.shuttle_index + 1, role_index)
            for role_index, role in enumerate(roles, start=1)
        }
        shuttles.append({
            'shuttle_id': spec.shuttle_id,
            'side': spec.side,
            'shuttle_index': spec.shuttle_index,
            'tag_ids': tag_ids,
            'marker_roles': list(roles),
            'label_text': spec.short_id,
            'color_name': colors[spec.short_id],
            'expected_marker_locations_on_body': {
                'front_left': {'x': 0.12, 'y': 0.08, 'z': 0.03},
                'front_right': {'x': 0.12, 'y': -0.08, 'z': 0.03},
                'rear_left': {'x': -0.12, 'y': 0.08, 'z': 0.03},
                'rear_right': {'x': -0.12, 'y': -0.08, 'z': 0.03},
            },
            'payload_keepout_zone': {
                'center_zone_m': {'x_half_width': 0.08, 'y_half_width': 0.055},
                'identity_safe_perimeter_m': 0.025,
            },
        })
    return {
        'version': 1,
        'description': 'Room 315 shuttle perimeter identity frame mapping.',
        'marker_family': 'tag36h11',
        'shuttles': shuttles,
    }


def load_identity_config(path: Path | str) -> dict[str, Any]:
    loaded = yaml.safe_load(Path(path).expanduser().read_text(encoding='utf-8')) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f'{path} must contain a YAML mapping')
    return loaded


def validate_identity_config(config: dict[str, Any]) -> None:
    shuttles = config.get('shuttles')
    if not isinstance(shuttles, list):
        raise ValueError('identity config requires a shuttles list')
    seen_tags: set[int] = set()
    seen_shuttles: set[str] = set()
    for entry in shuttles:
        if not isinstance(entry, dict):
            raise ValueError('identity config shuttle entries must be mappings')
        spec = normalize_shuttle_ref(entry.get('shuttle_id'), side=entry.get('side'))
        if spec is None:
            raise ValueError(f'invalid shuttle_id {entry.get("shuttle_id")!r}')
        seen_shuttles.add(spec.shuttle_id)
        tags = entry.get('tag_ids')
        if not isinstance(tags, dict) or len(tags) < 2:
            raise ValueError(f'{spec.shuttle_id} needs at least two perimeter tag IDs')
        for tag_id in tags.values():
            parsed = int(tag_id)
            if parsed in seen_tags:
                raise ValueError(f'duplicate tag ID {parsed}')
            seen_tags.add(parsed)
        label = str(entry.get('label_text') or '').strip().upper()
        if label != spec.short_id:
            raise ValueError(f'{spec.shuttle_id} label_text should be {spec.short_id}, got {label!r}')
    expected = {spec.shuttle_id for spec in all_shuttle_specs()}
    missing = expected - seen_shuttles
    if missing:
        raise ValueError(f'identity config missing shuttle(s): {sorted(missing)}')


def identity_tracks_from_marker_detections(
    detections: list[dict[str, Any]],
    identity_config: dict[str, Any],
) -> list[dict[str, Any]]:
    tag_to_entry: dict[int, dict[str, Any]] = {}
    for entry in identity_config.get('shuttles', []):
        for role, tag_id in (entry.get('tag_ids') or {}).items():
            mapped = dict(entry)
            mapped['marker_role'] = role
            tag_to_entry[int(tag_id)] = mapped
    grouped: dict[str, dict[str, Any]] = {}
    for detection in detections:
        tag_id = int(detection.get('tag_id'))
        entry = tag_to_entry.get(tag_id)
        if not entry:
            continue
        shuttle = str(entry['shuttle_id'])
        track = grouped.setdefault(
            shuttle,
            {
                'shuttle_id': shuttle,
                'side': entry['side'],
                'shuttle_index': entry['shuttle_index'],
                'visible_marker_ids': [],
                'visible_marker_count': 0,
                'bbox': detection.get('bbox', []),
                'confidence': 0.0,
                'visibility_state': 'lost',
            },
        )
        track['visible_marker_ids'].append(tag_id)
        track['visible_marker_count'] = len(track['visible_marker_ids'])
        track['confidence'] = min(1.0, 0.35 + 0.2 * track['visible_marker_count'])
        track['visibility_state'] = 'visible' if track['visible_marker_count'] >= 2 else 'partially_occluded'
    return sorted(grouped.values(), key=lambda item: item['shuttle_id'])


def model_input_is_clean(model_input: dict[str, Any]) -> bool:
    if set(model_input) != {'language', 'overhead_images', 'last_command', 'observable_state'}:
        return False
    if not _observable_state_is_clean(model_input.get('observable_state')):
        return False
    forbidden_keys = {
        'active_position_sensors',
        'active_sensors',
        'binary_sensor_bits',
        'current_segment',
        'distance_to_switch',
        'expert_sensor_state',
        'gazebo_pose',
        'loaded',
        'normalized_position',
        'normalized_rail_position',
        'payload',
        'payload_condition',
        'payload_present',
        'payload_state',
        'payload_type',
        'raw_shuttle_states',
        's',
        'segment',
        'shuttle_identity_tracks',
        'status',
        'stopper_states',
        'structured_rail_state',
        'supervisor_status',
        'switch_states',
        'target_shuttle_id',
        'true_shuttle_segment',
        'x',
        'y',
        'yaw',
        'z',
    }
    if _contains_forbidden_model_input_key(model_input, forbidden_keys):
        return False
    serialized = json.dumps(model_input, sort_keys=True)
    forbidden = (
        'pddl',
        'symbolic_plan',
        'planner',
        'tag_id',
        'apriltag',
        'shuttle_track',
        'target_shuttle_id',
        'structured_rail_state',
        'privileged_eval',
        'binary_sensor',
        'gazebo',
        'arc_length',
    )
    return not any(token in serialized for token in forbidden)


def _observable_state_is_clean(value: Any) -> bool:
    if not isinstance(value, dict):
        return False
    for side, side_state in value.items():
        if str(side) not in SIDES or not isinstance(side_state, dict):
            return False
        if set(side_state) != {'sensors', 'switches', 'stoppers'}:
            return False
        sensors = side_state.get('sensors')
        switches = side_state.get('switches')
        stoppers = side_state.get('stoppers')
        if (
            not isinstance(sensors, dict)
            or not isinstance(switches, dict)
            or not isinstance(stoppers, dict)
        ):
            return False
        for sensor_name, sensor_value in sensors.items():
            if not isinstance(sensor_name, str):
                return False
            try:
                if int(sensor_value) not in {0, 1}:
                    return False
            except (TypeError, ValueError):
                return False
        if set(switches) - set(DEVICE_NAMES) or set(stoppers) - set(DEVICE_NAMES):
            return False
        if any(value not in {'EXTERIOR', 'INTERIOR', 'UNKNOWN'} for value in switches.values()):
            return False
        if any(value not in {'open', 'closed', 'unknown'} for value in stoppers.values()):
            return False
    return True


def _contains_forbidden_model_input_key(value: Any, forbidden_keys: set[str]) -> bool:
    if isinstance(value, dict):
        for key, child in value.items():
            if str(key).strip().casefold() in forbidden_keys:
                return True
            if _contains_forbidden_model_input_key(child, forbidden_keys):
                return True
    elif isinstance(value, list):
        return any(_contains_forbidden_model_input_key(item, forbidden_keys) for item in value)
    return False


def round_int(value: Any) -> int:
    try:
        return int(round(float(value)))
    except (TypeError, ValueError):
        return 0


def safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def device_map(raw: Any, *, default: Any) -> dict[str, Any]:
    values = {name: default for name in DEVICE_NAMES}
    if isinstance(raw, dict):
        for name in DEVICE_NAMES:
            if name in raw:
                values[name] = raw[name]
    return values


def _switch_value(raw: Any) -> str:
    text = str(raw or '').strip().upper()
    if text in {'E', 'EXTERIOR'}:
        return 'EXTERIOR'
    if text in {'I', 'INTERIOR'}:
        return 'INTERIOR'
    return 'UNCHANGED'


def _stopper_value(raw: Any) -> str:
    text = str(raw or '').strip().lower()
    if text in {'0', 'open', 'opened', 'release', 'released', 'off', 'false'}:
        return 'open'
    if text in {'1', 'closed', 'close', 'stop', 'blocked', 'on', 'true'}:
        return 'closed'
    return 'UNCHANGED'


def _known(raw: Any, lookup: dict[str, int], default: str) -> str:
    text = str(raw or default).strip()
    return text if text in lookup else default


def _known_target(raw: Any, *, side: str, shuttle_index: int) -> str:
    text = str(raw or '').strip()
    if text in TARGET_IDS:
        return text
    if 0 <= shuttle_index < MAX_SHUTTLES_PER_SIDE:
        target = f'{side}_shuttle_{shuttle_index + 1}'
        if target in TARGET_IDS:
            return target
    legacy = f'{side}_shuttle'
    return legacy if legacy in TARGET_IDS else 'none'
