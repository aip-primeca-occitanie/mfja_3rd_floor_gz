#!/usr/bin/env python3
"""ROS-independent state and transport core for Room 315 task execution.

The visual observation owns shuttle location and payload facts.  Controller
state is admitted only for presence (upstream of this module), switch/stopper
state, safety decisions, and confirmation that an OFF command took effect.
In particular, supervisor ShuttleState position fields are never read here.
"""

from __future__ import annotations

import copy
import itertools
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from typing import Callable


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in __import__('sys').path:
    __import__('sys').path.insert(0, str(SCRIPT_DIR))

from room_315_contracts import ObservedFact
from room_315_contracts import ObservedState
from room_315_contracts import TaskGoal
from room_315_multi_shuttle import DEVICE_NAMES
from room_315_multi_shuttle import all_shuttle_specs
from room_315_multi_shuttle import normalize_fleet_block_id
from room_315_multi_shuttle import normalize_shuttle_ref
from room_315_observed_state_provider import ObservedStateProvider
from room_315_observed_state_provider import fuse_observed_facts
from room_315_pddl_scenario_generator import RAIL_DEVICES_PATH_BY_SIDE
from room_315_pddl_scenario_generator import RAIL_NETWORK_PATH_BY_SIDE
from room_315_pddl_scenario_generator import ScenarioTransport
from room_315_pddl_scenario_generator import SLOT_SENSOR_BY_SIDE_AND_SLOT
from room_315_multi_shuttle import load_rail_topology
from room_315_rail_defaults import LEFT_PUBLIC_SEGMENT_NAME_MAP


PRESENCE_STATES = frozenset({'present', 'absent', 'unknown'})
TERMINAL_SUPERVISOR_DECISIONS = frozenset({
    'accepted',
    'approved',
    'executed',
    'failed',
    'rejected',
    'blocked',
})
STOPPED_MOTION_VALUES = frozenset({
    'DISABLED',
    'FALLING',
    'HALTED',
    'IDLE',
    'OFF',
    'STOPPED',
    'WAITING',
})


class TaskExecutionStateError(RuntimeError):
    """Raised when live state cannot safely cross the planner boundary."""


@dataclass(frozen=True)
class LiveStateConfig:
    observation_timeout_s: float = 1.5
    supervisor_status_timeout_s: float = 1.5
    slot_sensor_state_timeout_s: float = 1.0
    planning_slot_tolerance_ratio: float = 0.12
    target_arrival_tolerance_ratio: float = 0.05
    position_consistency_tolerance_m: float = 0.08
    observation_wait_s: float = 2.0
    external_obstacles_disabled: bool = True

    def __post_init__(self) -> None:
        for name in (
            'observation_timeout_s',
            'supervisor_status_timeout_s',
            'slot_sensor_state_timeout_s',
            'planning_slot_tolerance_ratio',
            'target_arrival_tolerance_ratio',
            'position_consistency_tolerance_m',
            'observation_wait_s',
        ):
            value = float(getattr(self, name))
            if value <= 0.0:
                raise ValueError(f'{name} must be greater than zero')
        for name in (
            'planning_slot_tolerance_ratio',
            'target_arrival_tolerance_ratio',
        ):
            if float(getattr(self, name)) > 1.0:
                raise ValueError(f'{name} must be no greater than one')


@dataclass(frozen=True)
class LiveVisualSnapshot:
    observation: dict[str, Any]
    observation_receive_s: float
    supervisor_status: dict[str, Any]
    supervisor_receive_s: float


class VisualSlotMatcher:
    """Map learned segment/position outputs to authoritative slot locations."""

    def __init__(self, *, tolerance_ratio: float) -> None:
        if not 0.0 < float(tolerance_ratio) <= 1.0:
            raise ValueError('slot tolerance must be in (0, 1]')
        self.tolerance_ratio = float(tolerance_ratio)
        self._topologies = {
            side: load_rail_topology(
                RAIL_NETWORK_PATH_BY_SIDE[side],
                RAIL_DEVICES_PATH_BY_SIDE[side],
                side=side,
            )
            for side in ('left', 'right')
        }

    def slot_for_shuttle(
        self,
        shuttle: dict[str, Any],
        *,
        tolerance_ratio: float | None = None,
    ) -> str:
        side = _side(shuttle.get('side'))
        segment = str(shuttle.get('block') or '').strip().upper()
        match_tolerance = (
            self.tolerance_ratio
            if tolerance_ratio is None
            else float(tolerance_ratio)
        )
        if not 0.0 < match_tolerance <= 1.0:
            raise ValueError('slot tolerance must be in (0, 1]')
        if side == 'left':
            slots = {
                slot: (
                    LEFT_PUBLIC_SEGMENT_NAME_MAP.get(location.segment, location.segment),
                    float(location.s_ratio),
                )
                for slot, location in self._topologies[side].slots.items()
            }
        else:
            slots = {
                slot: (location.segment, float(location.s_ratio))
                for slot, location in self._topologies[side].slots.items()
            }
        try:
            s_ratio = float(shuttle.get('s_ratio'))
        except (TypeError, ValueError) as exc:
            raise TaskExecutionStateError(
                f'visual shuttle {shuttle.get("identity")!r} has invalid position'
            ) from exc
        if not 0.0 <= s_ratio <= 1.0:
            raise TaskExecutionStateError(
                f'visual shuttle {shuttle.get("identity")!r} has invalid s_ratio'
            )
        candidates = []
        for slot, (slot_segment, slot_ratio) in slots.items():
            if segment != str(slot_segment).upper():
                continue
            distance_ratio = abs(s_ratio - slot_ratio)
            if distance_ratio <= match_tolerance:
                candidates.append((distance_ratio, slot))
        if not candidates:
            return ''
        candidates.sort()
        if (
            len(candidates) > 1
            and abs(candidates[0][0] - candidates[1][0]) < 1e-9
        ):
            raise TaskExecutionStateError(
                f'visual shuttle {shuttle.get("identity")!r} is ambiguous between slots'
            )
        return candidates[0][1]

    def assign_slots(
        self,
        shuttles: list[dict[str, Any]],
        *,
        competition_tolerance_ratio: float = 0.20,
        minimum_assignment_margin_ratio: float = 0.02,
    ) -> dict[str, str]:
        """Return a fail-closed one-to-one assignment for the whole rail.

        The expanded tolerance is used only when multiple shuttles compete for
        physical slots on the same public segment.  A globally minimum unique
        assignment can therefore resolve a shared model bias without allowing
        two shuttles to claim one slot.  A tied optimum remains unknown.
        """

        expanded = float(competition_tolerance_ratio)
        minimum_margin = float(minimum_assignment_margin_ratio)
        if not self.tolerance_ratio <= expanded <= 1.0:
            raise ValueError(
                'competition_tolerance_ratio must be between the normal '
                'slot tolerance and one'
            )
        if not 0.0 <= minimum_margin <= 1.0:
            raise ValueError('minimum_assignment_margin_ratio must be in [0, 1]')
        assignments: dict[str, str] = {}
        grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
        for shuttle in shuttles:
            identity = str(shuttle.get('identity') or '').strip().upper()
            if not identity:
                raise TaskExecutionStateError('visual shuttle has no identity')
            side = _side(shuttle.get('side'))
            segment = str(shuttle.get('block') or '').strip().upper()
            grouped.setdefault((side, segment), []).append(shuttle)

        for (side, segment), items in grouped.items():
            slots = self._slot_ratios(side=side, segment=segment)
            if not slots:
                continue
            tolerance = self.tolerance_ratio if len(items) == 1 else expanded
            identities = [
                str(item.get('identity') or '').strip().upper()
                for item in items
            ]
            ratios: list[float] = []
            for item in items:
                try:
                    ratio = float(item.get('s_ratio'))
                except (TypeError, ValueError) as exc:
                    raise TaskExecutionStateError(
                        f'visual shuttle {item.get("identity")!r} has invalid position'
                    ) from exc
                if not 0.0 <= ratio <= 1.0:
                    raise TaskExecutionStateError(
                        f'visual shuttle {item.get("identity")!r} has invalid s_ratio'
                    )
                ratios.append(ratio)

            choices = [None, *sorted(slots)]
            candidates: list[
                tuple[int, float, tuple[str | None, ...]]
            ] = []
            for selected in itertools.product(choices, repeat=len(items)):
                occupied = [slot for slot in selected if slot is not None]
                if len(occupied) != len(set(occupied)):
                    continue
                cost = 0.0
                valid = True
                for ratio, slot in zip(ratios, selected):
                    if slot is None:
                        continue
                    distance = abs(ratio - slots[slot])
                    if distance > tolerance:
                        valid = False
                        break
                    cost += distance
                if valid:
                    candidates.append((-len(occupied), cost, selected))
            candidates.sort(key=lambda item: (item[0], item[1]))
            if not candidates:
                continue
            best = candidates[0]
            if (
                len(candidates) > 1
                and candidates[1][0] == best[0]
                and candidates[1][2] != best[2]
                and candidates[1][1] - best[1] < minimum_margin
            ):
                raise TaskExecutionStateError(
                    f'visual slot assignment ambiguous:{side}:{segment}:'
                    f'{",".join(identities)}'
                )
            for identity, slot in zip(identities, best[2]):
                if slot is not None:
                    assignments[identity] = slot
        return assignments

    def _slot_ratios(self, *, side: str, segment: str) -> dict[str, float]:
        if side == 'left':
            return {
                slot: float(location.s_ratio)
                for slot, location in self._topologies[side].slots.items()
                if str(
                    LEFT_PUBLIC_SEGMENT_NAME_MAP.get(
                        location.segment,
                        location.segment,
                    )
                ).upper() == segment
            }
        return {
            slot: float(location.s_ratio)
            for slot, location in self._topologies[side].slots.items()
            if str(location.segment).upper() == segment
        }


class VisualObservedStateBuilder:
    """Build the fail-closed planner state from accepted visual observations."""

    def __init__(self, config: LiveStateConfig | None = None) -> None:
        self.config = config or LiveStateConfig()
        self.slot_matcher = VisualSlotMatcher(
            tolerance_ratio=self.config.planning_slot_tolerance_ratio,
        )
        self.identity_order = tuple(spec.short_id for spec in all_shuttle_specs())

    def build(
        self,
        snapshot: LiveVisualSnapshot,
        *,
        now_s: float,
        runtime_clearance_certificates: dict[str, dict[str, Any]] | None = None,
        slot_sensor_anchors: dict[str, dict[str, Any]] | None = None,
    ) -> ObservedState:
        observation = copy.deepcopy(snapshot.observation)
        supervisor = copy.deepcopy(snapshot.supervisor_status)
        self._validate_freshness(snapshot, now_s=now_s)
        self._validate_observation_envelope(observation)
        self._validate_supervisor(supervisor)
        clearance_certificates = self._clearance_certificates_by_identity(
            runtime_clearance_certificates
        )
        sensor_anchors = self._validated_slot_sensor_anchors(
            slot_sensor_anchors
        )

        timestamp_s = float(observation.get('timestamp_s') or now_s)
        visual_facts: list[ObservedFact] = []
        trusted_facts: list[ObservedFact] = []
        slot_occupants = {
            side: {slot: '' for slot in ('1', '2', '3', '4')}
            for side in ('left', 'right')
        }
        shuttles_by_identity = self._shuttles_by_identity(observation)

        assignable_items: list[dict[str, Any]] = []
        for spec in all_shuttle_specs():
            item = shuttles_by_identity[spec.short_id]
            if str(item.get('presence_state') or '').strip().lower() == 'absent':
                continue
            self._validate_present_visual_item(
                item,
                spec.short_id,
                spec.side,
                consistency_tolerance_m=(
                    self.config.position_consistency_tolerance_m
                ),
            )
            if (
                spec.short_id not in clearance_certificates
                and spec.short_id not in sensor_anchors
            ):
                assignable_items.append(item)
        slot_assignments = self.slot_matcher.assign_slots(assignable_items)

        for spec in all_shuttle_specs():
            item = shuttles_by_identity[spec.short_id]
            presence_state = str(item.get('presence_state') or '').strip().lower()
            trusted_facts.append(_fact(
                source='trusted_device',
                subject=spec.gazebo_entity_name,
                predicate='present',
                value=presence_state == 'present',
                timestamp_s=timestamp_s,
                metadata={
                    'field_owner': 'deterministic_controller_presence',
                    'identity': spec.short_id,
                    'side': spec.side,
                    'allowed_source_fields': [
                        'ShuttleState.name',
                        'ShuttleState.header.stamp',
                        'ROS receive time',
                    ],
                },
            ))
            if presence_state == 'absent':
                continue

            self._validate_present_visual_item(
                item,
                spec.short_id,
                spec.side,
                consistency_tolerance_m=(
                    self.config.position_consistency_tolerance_m
                ),
            )
            block_id = normalize_fleet_block_id(item['block'], side=spec.side)
            if not block_id:
                raise TaskExecutionStateError(
                    f'cannot normalize visual block for {spec.short_id}'
                )
            clearance_certificate = clearance_certificates.get(spec.short_id)
            sensor_anchor = sensor_anchors.get(spec.short_id)
            # A sensor-certified interior relocation means the shuttle cannot
            # physically occupy an exterior slot. Preserve the learned
            # block/position facts below, but suppress only their derived slot
            # claim so a parallel-branch classification error cannot invent a
            # collision with the selected shuttle.
            slot = (
                str(sensor_anchor['slot'])
                if sensor_anchor is not None
                else slot_assignments.get(spec.short_id, '')
            )
            if slot:
                previous = slot_occupants[spec.side][slot]
                if previous:
                    raise TaskExecutionStateError(
                        f'visual slot conflict:{spec.side}:slot:{slot}:'
                        f'{previous},{spec.short_id}'
                    )
                slot_occupants[spec.side][slot] = spec.gazebo_entity_name

            if clearance_certificate is not None:
                trusted_facts.append(_fact(
                    source='executor',
                    subject=spec.gazebo_entity_name,
                    predicate='runtime_route_clearance',
                    value=clearance_certificate,
                    timestamp_s=timestamp_s,
                    metadata={
                        'field_owner': 'sensor_certified_execution_effect',
                        'model_prediction_replaced': False,
                        'controller_position_fields_used_for_localization': False,
                        'slot_derivation_suppressed': True,
                    },
                ))

            if sensor_anchor is not None:
                trusted_facts.append(_fact(
                    source='trusted_device',
                    subject=spec.gazebo_entity_name,
                    predicate='location_slot',
                    value=f'{spec.side}:slot:{slot}',
                    timestamp_s=timestamp_s,
                    metadata={
                        'field_owner': 'deterministic_binary_slot_sensor',
                        'identity': spec.short_id,
                        'side': spec.side,
                        'slot': slot,
                        'sensor': sensor_anchor['sensor'],
                        'allowed_source_fields': [
                            'SensorReading.name',
                            'SensorReading.active',
                            'SensorReading.shuttle_name',
                            'SensorFeedback.header.stamp',
                            'ROS receive time',
                        ],
                        'forbidden_source_fields_ignored': [
                            'SensorReading.segment',
                            'SensorReading.s',
                            'SensorReading.s_ratio',
                            'ShuttleState.current_segment',
                            'ShuttleState.s',
                            'ShuttleState.x',
                            'ShuttleState.y',
                            'ShuttleState.z',
                            'ShuttleState.yaw',
                        ],
                        'raw_visual_location_replaced': False,
                    },
                ))

            common = {
                'field_owner': 'visual_model',
                'identity': spec.short_id,
                'side': spec.side,
                'checkpoint_sha256': observation.get('checkpoint_sha256', ''),
                'schema_version': observation.get('schema_version', ''),
            }
            visual_values = {
                'loaded': str(item['loaded_state']).lower() == 'loaded',
                'location_block': block_id,
                'rail_position': {
                    'available': True,
                    'side': spec.side,
                    'segment': str(item['block']).upper(),
                    's_m': float(item['s_m']),
                    's_ratio': float(item['s_ratio']),
                    'segment_length_m': float(item['segment_length_m']),
                    # This is not a model target. It records that no extra
                    # oracle uncertainty was injected at runtime.
                    'position_uncertainty_m': 0.0,
                },
                'visual_bbox': {
                    'bbox_xywh': [float(value) for value in item['bbox_xywh']],
                    'camera': (
                        'left_rail_rgb'
                        if spec.side == 'left'
                        else 'right_rail_rgb'
                    ),
                },
            }
            if slot and sensor_anchor is None:
                visual_values['location_slot'] = f'{spec.side}:slot:{slot}'
            for predicate, value in visual_values.items():
                visual_facts.append(_fact(
                    source='visual_model',
                    subject=spec.gazebo_entity_name,
                    predicate=predicate,
                    value=value,
                    timestamp_s=timestamp_s,
                    confidence=0.0,
                    metadata={
                        **common,
                        'confidence_available': False,
                    },
                ))

        for side in ('left', 'right'):
            for slot in ('1', '2', '3', '4'):
                occupant = slot_occupants[side][slot]
                trusted_facts.append(_fact(
                    source='state_fuser',
                    subject=f'{side}:slot:{slot}',
                    predicate='occupancy',
                    value={
                        'occupied': bool(occupant),
                        'shuttle': occupant or None,
                        'derived_from': 'accepted_visual_segment_and_position',
                    },
                    timestamp_s=timestamp_s,
                    metadata={
                        'field_owner': 'visual_location_slot_derivation',
                        'side': side,
                        'slot': slot,
                        'planning_slot_tolerance_ratio': (
                            self.config.planning_slot_tolerance_ratio
                        ),
                    },
                ))

        trusted_facts.extend(
            self._device_and_obstacle_facts(supervisor, timestamp_s=timestamp_s)
        )
        fused = fuse_observed_facts(
            [*trusted_facts, *visual_facts],
            timestamp=timestamp_s,
        )
        return ObservedState(
            state_id=str(
                observation.get('state_id')
                or f'room315-live-visual-{int(timestamp_s * 1_000_000)}'
            ),
            timestamp=timestamp_s,
            stale_after_s=self.config.observation_timeout_s,
            visual_model_inputs=visual_facts,
            fused_planner_state=fused,
        )

    @staticmethod
    def _validated_slot_sensor_anchors(
        raw: dict[str, dict[str, Any]] | None,
    ) -> dict[str, dict[str, Any]]:
        expected_by_sensor = {
            sensor.upper(): (side, slot)
            for (side, slot), sensor in SLOT_SENSOR_BY_SIDE_AND_SLOT.items()
        }
        anchors: dict[str, dict[str, Any]] = {}
        occupied: dict[tuple[str, str], str] = {}
        for raw_identity, raw_anchor in dict(raw or {}).items():
            if not isinstance(raw_anchor, dict):
                raise TaskExecutionStateError(
                    'slot sensor anchor must be an object'
                )
            spec = normalize_shuttle_ref(raw_identity)
            if spec is None:
                spec = normalize_shuttle_ref(raw_anchor.get('identity'))
            if spec is None:
                raise TaskExecutionStateError(
                    f'unknown slot sensor identity:{raw_identity}'
                )
            sensor = str(raw_anchor.get('sensor') or '').strip().upper()
            expected = expected_by_sensor.get(sensor)
            if expected is None:
                raise TaskExecutionStateError(
                    f'unknown slot sensor anchor:{sensor or "empty"}'
                )
            side, slot = expected
            if side != spec.side:
                raise TaskExecutionStateError(
                    f'slot sensor identity side conflict:{sensor}:{spec.short_id}'
                )
            key = (side, slot)
            previous = occupied.get(key)
            if previous and previous != spec.short_id:
                raise TaskExecutionStateError(
                    f'duplicate slot sensor occupancy:{side}:slot:{slot}:'
                    f'{previous},{spec.short_id}'
                )
            if spec.short_id in anchors:
                raise TaskExecutionStateError(
                    f'duplicate slot sensor identity:{spec.short_id}'
                )
            occupied[key] = spec.short_id
            anchors[spec.short_id] = {
                'identity': spec.short_id,
                'side': side,
                'slot': slot,
                'sensor': sensor,
            }
        return anchors

    @staticmethod
    def _clearance_certificates_by_identity(
        raw: dict[str, dict[str, Any]] | None,
    ) -> dict[str, dict[str, Any]]:
        certificates: dict[str, dict[str, Any]] = {}
        for raw_identity, raw_certificate in dict(raw or {}).items():
            if not isinstance(raw_certificate, dict):
                raise TaskExecutionStateError(
                    'runtime clearance certificate must be an object'
                )
            spec = normalize_shuttle_ref(raw_identity)
            if spec is None:
                spec = normalize_shuttle_ref(raw_certificate.get('identity'))
            if spec is None:
                raise TaskExecutionStateError(
                    f'unknown runtime clearance identity:{raw_identity}'
                )
            certificate = copy.deepcopy(raw_certificate)
            side = _side(certificate.get('side') or spec.side)
            if side != spec.side:
                raise TaskExecutionStateError(
                    f'runtime clearance side conflict:{spec.short_id}:{side}'
                )
            if not bool(certificate.get('entry_sensor_identity_confirmed')):
                raise TaskExecutionStateError(
                    f'runtime clearance lacks entry sensor proof:{spec.short_id}'
                )
            if not bool(certificate.get('controller_stop_confirmed')):
                raise TaskExecutionStateError(
                    f'runtime clearance lacks stop proof:{spec.short_id}'
                )
            if certificate.get('matched_by') != (
                'interior_entry_sensor_plus_bounded_travel_time'
            ):
                raise TaskExecutionStateError(
                    'runtime clearance lacks bounded-motion proof:'
                    f'{spec.short_id}'
                )
            if not bool(certificate.get('bounded_commanded_motion_completed')):
                raise TaskExecutionStateError(
                    'runtime clearance lacks completed bounded motion:'
                    f'{spec.short_id}'
                )
            if (
                certificate.get('clearance_mode_held') is not True
                or certificate.get('normal_route_restored') is not False
            ):
                raise TaskExecutionStateError(
                    'runtime clearance lacks held-route proof:'
                    f'{spec.short_id}'
                )
            if (
                certificate.get(
                    'controller_position_fields_used_for_localization'
                )
                is not False
            ):
                raise TaskExecutionStateError(
                    'runtime clearance used forbidden controller position:'
                    f'{spec.short_id}'
                )
            certificate['identity'] = spec.short_id
            certificate['side'] = side
            certificates[spec.short_id] = certificate
        return certificates

    def target_reached(
        self,
        observation: dict[str, Any],
        *,
        shuttle: str,
        side: str,
        target_slot: str,
    ) -> bool:
        if not bool(observation.get('accepted', False)):
            return False
        spec = normalize_shuttle_ref(shuttle, side=side)
        if spec is None:
            return False
        for item in observation.get('shuttles') or []:
            if str(item.get('identity') or '').upper() != spec.short_id:
                continue
            if str(item.get('presence_state') or '').lower() != 'present':
                return False
            if not bool(item.get('visual_facts_valid', False)):
                return False
            return self.slot_matcher.slot_for_shuttle(
                item,
                tolerance_ratio=self.config.target_arrival_tolerance_ratio,
            ) == str(target_slot)
        return False

    def _validate_freshness(
        self,
        snapshot: LiveVisualSnapshot,
        *,
        now_s: float,
    ) -> None:
        if float(snapshot.observation_receive_s) < 0.0:
            raise TaskExecutionStateError(
                'accepted_visual_observation_unavailable'
            )
        if float(snapshot.supervisor_receive_s) < 0.0:
            raise TaskExecutionStateError('supervisor_status_unavailable')
        visual_age = now_s - float(snapshot.observation_receive_s)
        supervisor_age = now_s - float(snapshot.supervisor_receive_s)
        if visual_age < 0.0 or visual_age > self.config.observation_timeout_s:
            raise TaskExecutionStateError(
                f'accepted_visual_observation_stale:age_s={visual_age:.3f}'
            )
        if (
            supervisor_age < 0.0
            or supervisor_age > self.config.supervisor_status_timeout_s
        ):
            raise TaskExecutionStateError(
                f'supervisor_status_stale:age_s={supervisor_age:.3f}'
            )

    @staticmethod
    def _validate_observation_envelope(observation: dict[str, Any]) -> None:
        required_true = (
            'accepted',
            'model_ready',
            'input_ready',
            'presence_ready',
            'state_fusion_ready',
        )
        false_fields = [name for name in required_true if not observation.get(name)]
        if false_fields:
            raise TaskExecutionStateError(
                f'visual_observation_not_ready:{",".join(false_fields)}'
            )
        if observation.get('stale'):
            raise TaskExecutionStateError('visual_observation_marked_stale')
        if observation.get('stage') != 'fused_observed_state':
            raise TaskExecutionStateError(
                f'unsupported_visual_stage:{observation.get("stage")!r}'
            )

    @staticmethod
    def _validate_supervisor(supervisor: dict[str, Any]) -> None:
        if not supervisor:
            raise TaskExecutionStateError('supervisor_status_unavailable')
        if bool(supervisor.get('emergency_stop', True)):
            raise TaskExecutionStateError('supervisor_emergency_stop_active')
        rails = supervisor.get('rails')
        if not isinstance(rails, dict):
            raise TaskExecutionStateError('supervisor_rail_state_unavailable')

    def _shuttles_by_identity(
        self,
        observation: dict[str, Any],
    ) -> dict[str, dict[str, Any]]:
        result: dict[str, dict[str, Any]] = {}
        for raw in observation.get('shuttles') or []:
            if not isinstance(raw, dict):
                raise TaskExecutionStateError('visual shuttle entry must be an object')
            identity = str(raw.get('identity') or '').strip().upper()
            if identity not in self.identity_order:
                raise TaskExecutionStateError(
                    f'unknown_visual_identity:{identity or "empty"}'
                )
            if identity in result:
                raise TaskExecutionStateError(f'duplicate_visual_identity:{identity}')
            state = str(raw.get('presence_state') or '').strip().lower()
            if state not in PRESENCE_STATES:
                raise TaskExecutionStateError(
                    f'invalid_presence_state:{identity}:{state or "empty"}'
                )
            if state == 'unknown':
                raise TaskExecutionStateError(f'unknown_presence:{identity}')
            result[identity] = raw
        missing = [identity for identity in self.identity_order if identity not in result]
        if missing:
            raise TaskExecutionStateError(
                f'missing_visual_identity_slots:{",".join(missing)}'
            )
        return result

    @staticmethod
    def _validate_present_visual_item(
        item: dict[str, Any],
        identity: str,
        expected_side: str,
        *,
        consistency_tolerance_m: float,
    ) -> None:
        if not bool(item.get('visual_facts_valid', False)):
            raise TaskExecutionStateError(
                f'present_identity_without_visual_facts:{identity}'
            )
        if _side(item.get('side')) != expected_side:
            raise TaskExecutionStateError(
                f'visual_identity_side_conflict:{identity}'
            )
        if str(item.get('loaded_state') or '').lower() not in {'loaded', 'empty'}:
            raise TaskExecutionStateError(f'invalid_loaded_state:{identity}')
        bbox = item.get('bbox_xywh')
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            raise TaskExecutionStateError(f'invalid_visual_bbox:{identity}')
        try:
            s_m = float(item.get('s_m'))
            ratio = float(item.get('s_ratio'))
            length = float(item.get('segment_length_m'))
        except (TypeError, ValueError) as exc:
            raise TaskExecutionStateError(
                f'invalid_visual_position:{identity}'
            ) from exc
        if length <= 0.0 or not 0.0 <= ratio <= 1.0:
            raise TaskExecutionStateError(f'invalid_visual_position:{identity}')
        if (
            abs(s_m - ratio * length)
            > float(consistency_tolerance_m)
        ):
            raise TaskExecutionStateError(
                f'inconsistent_visual_position:{identity}'
            )

    def _device_and_obstacle_facts(
        self,
        supervisor: dict[str, Any],
        *,
        timestamp_s: float,
    ) -> list[ObservedFact]:
        facts: list[ObservedFact] = []
        rails = supervisor['rails']
        for side in ('left', 'right'):
            rail = rails.get(side)
            if not isinstance(rail, dict):
                raise TaskExecutionStateError(
                    f'missing_supervisor_rail_state:{side}'
                )
            switches = rail.get('switches')
            stoppers = rail.get('stoppers')
            if not isinstance(switches, dict) or not isinstance(stoppers, dict):
                raise TaskExecutionStateError(
                    f'missing_supervisor_device_state:{side}'
                )
            for device in DEVICE_NAMES:
                if device not in switches or device not in stoppers:
                    raise TaskExecutionStateError(
                        f'incomplete_supervisor_device_state:{side}:{device}'
                    )
                facts.extend([
                    _fact(
                        source='trusted_device',
                        subject=f'{side}:switch:{device}',
                        predicate='state',
                        # The controller publishes compact E/I values while
                        # plan postconditions use EXTERIOR/INTERIOR. Keep one
                        # canonical fact vocabulary at the fusion boundary.
                        value=_normalize_switch(switches[device]),
                        timestamp_s=timestamp_s,
                        metadata={
                            'field_owner': 'deterministic_switch_controller',
                            'side': side,
                            'device': device,
                        },
                    ),
                    _fact(
                        source='trusted_device',
                        subject=f'{side}:stopper:{device}',
                        predicate='state',
                        # Likewise, normalize the controller's 0/1 encoding
                        # before the executive compares physical effects.
                        value=_normalize_stopper(stoppers[device]).lower(),
                        timestamp_s=timestamp_s,
                        metadata={
                            'field_owner': 'deterministic_stopper_controller',
                            'side': side,
                            'device': device,
                        },
                    ),
                ])
            if not self.config.external_obstacles_disabled:
                raise TaskExecutionStateError(
                    'external_obstacle_perception_not_configured'
                )
            facts.append(_fact(
                source='trusted_device',
                subject=f'{side}:obstacles',
                predicate='present_obstacles',
                value=[],
                timestamp_s=timestamp_s,
                metadata={
                    'field_owner': 'deployment_configuration',
                    'external_obstacles_disabled': True,
                    'side': side,
                },
            ))
        return facts


class LatestVisualObservedStateProvider(ObservedStateProvider):
    """Thread-safe latest accepted observation provider for the executive."""

    def __init__(
        self,
        builder: VisualObservedStateBuilder,
        *,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.builder = builder
        self.monotonic = monotonic
        self._condition = threading.Condition()
        self._observation: dict[str, Any] = {}
        self._observation_receive_s = -1.0
        self._supervisor_status: dict[str, Any] = {}
        self._supervisor_receive_s = -1.0
        self._runtime_clearance_certificates: dict[str, dict[str, Any]] = {}
        self._slot_sensor_anchors_by_side: dict[
            str,
            dict[str, dict[str, Any]],
        ] = {'left': {}, 'right': {}}
        self._slot_sensor_receive_s = {'left': -1.0, 'right': -1.0}
        self._slot_sensor_error = {'left': '', 'right': ''}

    def update_observation(
        self,
        observation: dict[str, Any],
        *,
        receive_s: float | None = None,
    ) -> None:
        with self._condition:
            self._observation = copy.deepcopy(observation)
            self._observation_receive_s = (
                self.monotonic() if receive_s is None else float(receive_s)
            )
            self._condition.notify_all()

    def update_supervisor(
        self,
        status: dict[str, Any],
        *,
        receive_s: float | None = None,
    ) -> None:
        with self._condition:
            self._supervisor_status = copy.deepcopy(status)
            self._supervisor_receive_s = (
                self.monotonic() if receive_s is None else float(receive_s)
            )
            self._condition.notify_all()

    def snapshot(self) -> LiveVisualSnapshot:
        with self._condition:
            return LiveVisualSnapshot(
                observation=copy.deepcopy(self._observation),
                observation_receive_s=self._observation_receive_s,
                supervisor_status=copy.deepcopy(self._supervisor_status),
                supervisor_receive_s=self._supervisor_receive_s,
            )

    def set_runtime_clearance_certificate(
        self,
        certificate: dict[str, Any],
    ) -> None:
        spec = normalize_shuttle_ref(certificate.get('identity'))
        if spec is None:
            raise TaskExecutionStateError('unknown runtime clearance identity')
        with self._condition:
            self._runtime_clearance_certificates[spec.short_id] = copy.deepcopy(
                certificate
            )
            self._condition.notify_all()

    def clear_runtime_clearance_certificate(
        self,
        shuttle: str,
        *,
        side: str | None = None,
    ) -> None:
        spec = normalize_shuttle_ref(shuttle, side=side)
        if spec is None:
            return
        with self._condition:
            self._runtime_clearance_certificates.pop(spec.short_id, None)
            self._condition.notify_all()

    def runtime_clearance_certificates(self) -> dict[str, dict[str, Any]]:
        with self._condition:
            return copy.deepcopy(self._runtime_clearance_certificates)

    def update_slot_sensor_feedback(
        self,
        side: str,
        readings: list[dict[str, Any]],
        *,
        receive_s: float | None = None,
    ) -> None:
        """Store fresh exact-slot anchors from binary sensor fields only."""

        rail_side = _side(side)
        slot_by_sensor = {
            sensor.upper(): slot
            for (sensor_side, slot), sensor in (
                SLOT_SENSOR_BY_SIDE_AND_SLOT.items()
            )
            if sensor_side == rail_side
        }
        anchors: dict[str, dict[str, Any]] = {}
        occupied_sensors: set[str] = set()
        error = ''
        for reading in readings:
            if not isinstance(reading, dict) or not bool(reading.get('active')):
                continue
            sensor = str(reading.get('name') or '').strip().upper()
            if sensor not in slot_by_sensor:
                continue
            shuttle = str(
                reading.get('shuttle')
                or reading.get('shuttle_name')
                or ''
            ).strip()
            spec = normalize_shuttle_ref(shuttle, side=rail_side)
            if spec is None:
                error = f'active slot sensor has unknown shuttle:{sensor}:{shuttle}'
                break
            if spec.side != rail_side:
                error = (
                    f'active slot sensor side conflict:{sensor}:{spec.short_id}'
                )
                break
            if sensor in occupied_sensors:
                error = f'duplicate active slot sensor:{sensor}'
                break
            if spec.short_id in anchors:
                error = f'duplicate active slot identity:{spec.short_id}'
                break
            occupied_sensors.add(sensor)
            anchors[spec.short_id] = {
                'identity': spec.short_id,
                'side': rail_side,
                'slot': slot_by_sensor[sensor],
                'sensor': sensor,
            }
        with self._condition:
            self._slot_sensor_anchors_by_side[rail_side] = anchors
            self._slot_sensor_receive_s[rail_side] = (
                self.monotonic() if receive_s is None else float(receive_s)
            )
            self._slot_sensor_error[rail_side] = error
            self._condition.notify_all()

    def slot_sensor_anchors(self, *, now_s: float | None = None) -> dict[
        str,
        dict[str, Any],
    ]:
        current = self.monotonic() if now_s is None else float(now_s)
        result: dict[str, dict[str, Any]] = {}
        with self._condition:
            for side in ('left', 'right'):
                age = current - self._slot_sensor_receive_s[side]
                if age < 0.0 or age > self.builder.config.slot_sensor_state_timeout_s:
                    continue
                error = self._slot_sensor_error[side]
                if error:
                    raise TaskExecutionStateError(error)
                for identity, anchor in self._slot_sensor_anchors_by_side[
                    side
                ].items():
                    if identity in result:
                        raise TaskExecutionStateError(
                            f'duplicate cross-rail slot sensor identity:{identity}'
                        )
                    result[identity] = copy.deepcopy(anchor)
        return result

    def ready(self) -> tuple[bool, str]:
        try:
            now_s = self.monotonic()
            self.builder.build(
                self.snapshot(),
                now_s=now_s,
                runtime_clearance_certificates=(
                    self.runtime_clearance_certificates()
                ),
                slot_sensor_anchors=self.slot_sensor_anchors(now_s=now_s),
            )
        except Exception as exc:  # noqa: BLE001 - readiness explanation
            return False, str(exc)
        return True, ''

    def observe(self, *, timestamp: float | None = None) -> ObservedState:
        deadline = self.monotonic() + self.builder.config.observation_wait_s
        last_error = 'live visual observation is unavailable'
        while self.monotonic() <= deadline:
            try:
                now_s = self.monotonic()
                return self.builder.build(
                    self.snapshot(),
                    now_s=now_s,
                    runtime_clearance_certificates=(
                        self.runtime_clearance_certificates()
                    ),
                    slot_sensor_anchors=self.slot_sensor_anchors(now_s=now_s),
                )
            except Exception as exc:  # noqa: BLE001 - bounded retry boundary
                last_error = str(exc)
            with self._condition:
                remaining = max(0.0, deadline - self.monotonic())
                self._condition.wait(timeout=min(remaining, 0.1))
        raise TaskExecutionStateError(last_error)


class VisualSupervisorTransport(ScenarioTransport):
    """Supervisor transport with deterministic final slot confirmation.

    Learned vision owns block and continuous position localization. Binary rail
    feedback may additionally anchor exact slot occupancy using only the sensor
    name, active bit, and occupying shuttle identity; its position-like fields
    are never consumed. The same binary proof stops and verifies a commanded
    slot move.
    """

    def __init__(
        self,
        *,
        provider: LatestVisualObservedStateProvider,
        publish_callback: Callable[[dict[str, Any]], None],
        slot_sensor_confirmation_frames: int | None = None,
        arrival_confirmation_frames: int | None = None,
        controller_stop_timeout_s: float = 3.0,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.provider = provider
        self.publish_callback = publish_callback
        confirmation_frames = (
            slot_sensor_confirmation_frames
            if slot_sensor_confirmation_frames is not None
            else arrival_confirmation_frames
        )
        if confirmation_frames is None:
            confirmation_frames = 2
        self.slot_sensor_confirmation_frames = max(int(confirmation_frames), 1)
        self.controller_stop_timeout_s = max(float(controller_stop_timeout_s), 0.1)
        self.monotonic = monotonic
        self._condition = threading.Condition()
        self._supervisor_status: dict[str, Any] = {}
        self._supervisor_sequence = 0
        self._visual_observation: dict[str, Any] = {}
        self._visual_sequence = 0
        self._sensor_feedback = {'left': [], 'right': []}
        self._sensor_sequence = {'left': 0, 'right': 0}

    def update_supervisor(self, status: dict[str, Any]) -> None:
        with self._condition:
            self._supervisor_status = copy.deepcopy(status)
            self._supervisor_sequence += 1
            self._condition.notify_all()

    def update_observation(self, observation: dict[str, Any]) -> None:
        with self._condition:
            self._visual_observation = copy.deepcopy(observation)
            self._visual_sequence += 1
            self._condition.notify_all()

    def visual_observation_count(self) -> int:
        with self._condition:
            return self._visual_sequence

    def wait_for_fresh_visual_observation(
        self,
        *,
        previous_count: int,
        timeout_s: float,
    ) -> dict[str, Any]:
        deadline = self.monotonic() + max(float(timeout_s), 0.0)
        while self.monotonic() <= deadline:
            with self._condition:
                if self._visual_sequence > int(previous_count):
                    return {
                        'ready': True,
                        'reason': '',
                        'visual_sequence': self._visual_sequence,
                    }
                self._condition.wait(
                    timeout=min(
                        0.1,
                        max(0.0, deadline - self.monotonic()),
                    )
                )
        return {
            'ready': False,
            'reason': 'timeout waiting for post-restoration visual frame',
            'visual_sequence': self.visual_observation_count(),
        }

    def update_sensor_feedback(
        self,
        side: str,
        readings: list[dict[str, Any]],
    ) -> None:
        """Accept only binary-active sensor name/identity observations."""

        rail_side = _side(side)
        sanitized = []
        for reading in readings:
            if not isinstance(reading, dict) or not bool(reading.get('active', True)):
                continue
            sanitized.append({
                'name': str(reading.get('name') or '').strip(),
                'shuttle': str(reading.get('shuttle') or '').strip(),
            })
        slot_sensors = {
            sensor.upper()
            for sensor in SLOT_SENSOR_BY_SIDE_AND_SLOT.values()
        }
        for reading in sanitized:
            if reading['name'].upper() in slot_sensors and reading['shuttle']:
                self.provider.clear_runtime_clearance_certificate(
                    reading['shuttle'],
                    side=rail_side,
                )
        with self._condition:
            self._sensor_feedback[rail_side] = sanitized
            self._sensor_sequence[rail_side] += 1
            self._condition.notify_all()

    def publish_command(self, command: dict[str, Any]) -> None:
        self.publish_callback(copy.deepcopy(command))

    def supervisor_decision_count(self) -> int:
        with self._condition:
            metrics = _nested_dict(
                self._supervisor_status,
                'safety_decoder',
                'metrics',
            )
            return int(metrics.get('total_proposed_actions') or 0)

    def wait_for_supervisor_decision(
        self,
        *,
        previous_count: int,
        timeout_s: float,
    ) -> dict[str, Any] | None:
        deadline = self.monotonic() + max(float(timeout_s), 0.0)
        while self.monotonic() <= deadline:
            with self._condition:
                metrics = _nested_dict(
                    self._supervisor_status,
                    'safety_decoder',
                    'metrics',
                )
                count = int(metrics.get('total_proposed_actions') or 0)
                if count > int(previous_count):
                    decision = _nested_dict(
                        self._supervisor_status,
                        'safety_decoder',
                        'last_decision',
                    )
                    return copy.deepcopy(decision or {
                        'accepted': True,
                        'status': 'accepted',
                    })
                self._condition.wait(
                    timeout=min(0.1, max(0.0, deadline - self.monotonic()))
                )
        return None

    def wait_for_switch_state(
        self,
        *,
        side: str,
        switches: dict[str, Any],
        timeout_s: float,
    ) -> dict[str, Any]:
        expected = {
            device: str(value).strip().upper()
            for device, value in _expand_devices(switches).items()
        }
        return self._wait_device_state(
            side=side,
            kind='switches',
            expected=expected,
            timeout_s=timeout_s,
            normalize=lambda value: _normalize_switch(value),
        )

    def wait_for_stopper_state(
        self,
        *,
        side: str,
        stoppers: dict[str, Any],
        timeout_s: float,
    ) -> dict[str, Any]:
        expected = {
            device: _normalize_stopper(value)
            for device, value in _expand_devices(stoppers).items()
        }
        return self._wait_device_state(
            side=side,
            kind='stoppers',
            expected=expected,
            timeout_s=timeout_s,
            normalize=_normalize_stopper,
        )

    def wait_for_target_arrival(
        self,
        *,
        side: str,
        target_sensors: list[str],
        shuttle: str,
        timeout_s: float,
        target_slot: str = '',
        target_station: str = '',
        target_segment: str = '',
        target_s: float | None = None,
        target_tolerance_m: float | None = None,
    ) -> dict[str, Any]:
        rail_side = _side(side)
        if not target_slot:
            return {
                'arrived': False,
                'reason': 'slot-sensor arrival requires an explicit target_slot',
            }
        spec = normalize_shuttle_ref(shuttle, side=rail_side)
        if spec is None or spec.side != rail_side:
            return {
                'arrived': False,
                'reason': f'unknown or side-conflicting shuttle {shuttle!r}',
            }
        expected_sensor = SLOT_SENSOR_BY_SIDE_AND_SLOT.get(
            (rail_side, str(target_slot)),
            '',
        )
        wanted = {
            str(sensor or '').strip().upper()
            for sensor in target_sensors
            if str(sensor or '').strip()
        }
        if not expected_sensor or wanted != {expected_sensor.upper()}:
            return {
                'arrived': False,
                'reason': (
                    'target sensor contract mismatch: expected '
                    f'{expected_sensor or "unconfigured"}, received '
                    f'{sorted(wanted)}'
                ),
                'target_sensors': sorted(wanted),
            }
        deadline = self.monotonic() + max(float(timeout_s), 0.0)
        confirmed = 0
        last_sequence = -1
        while self.monotonic() <= deadline:
            with self._condition:
                sequence = self._sensor_sequence[rail_side]
                readings = copy.deepcopy(self._sensor_feedback[rail_side])
                if sequence == last_sequence:
                    self._condition.wait(
                        timeout=min(0.1, max(0.0, deadline - self.monotonic()))
                    )
                    continue
                last_sequence = sequence
            sensor_match = _exact_target_sensor_match(
                readings,
                sensor=expected_sensor,
                shuttle=spec.gazebo_entity_name,
                side=rail_side,
            )
            if sensor_match['error']:
                return {
                    'arrived': False,
                    'reason': sensor_match['error'],
                    'matched_by': 'deterministic_slot_sensor',
                    'target_sensor': expected_sensor,
                }
            confirmed = confirmed + 1 if sensor_match['matched'] else 0
            if confirmed < self.slot_sensor_confirmation_frames:
                continue
            stop_result = self._stop_after_sensor_arrival(
                side=rail_side,
                shuttle=shuttle,
                target_slot=target_slot,
                target_sensor=expected_sensor,
            )
            if not stop_result['ready']:
                return {
                    'arrived': False,
                    'reason': stop_result['reason'],
                    'matched_by': 'deterministic_slot_sensor',
                }
            with self._condition:
                stopped_readings = copy.deepcopy(
                    self._sensor_feedback[rail_side]
                )
            stopped_match = _exact_target_sensor_match(
                stopped_readings,
                sensor=expected_sensor,
                shuttle=spec.gazebo_entity_name,
                side=rail_side,
            )
            if not stopped_match['matched'] or stopped_match['error']:
                return {
                    'arrived': False,
                    'reason': (
                        stopped_match['error']
                        or 'target slot sensor cleared before stop confirmation'
                    ),
                    'matched_by': 'deterministic_slot_sensor',
                }
            return {
                'arrived': True,
                'reason': '',
                'side': rail_side,
                'shuttle': spec.gazebo_entity_name,
                'target_slot': target_slot,
                'target_station': target_station,
                'target_sensor': expected_sensor,
                'matched_sensors': [expected_sensor],
                'matched_by': 'deterministic_slot_sensor',
                'sensor_identity_confirmed': True,
                'sensor_confirmation_frames': confirmed,
                'controller_stop_confirmed': True,
                'controller_target_slot_confirmed': True,
                'controller_position_fields_used_for_localization': False,
                'visual_localization_used_for_final_stop': False,
            }
        return {
            'arrived': False,
            'reason': (
                f'timeout waiting for {expected_sensor} to report '
                f'{spec.gazebo_entity_name} at {rail_side} slot {target_slot}'
            ),
            'matched_by': 'deterministic_slot_sensor',
        }

    def wait_for_shuttle_stopped(
        self,
        *,
        side: str,
        shuttle: str,
        timeout_s: float,
    ) -> dict[str, Any]:
        return self._wait_controller_stopped(
            side=side,
            shuttle=shuttle,
            timeout_s=timeout_s,
        )

    def wait_for_visual_position_and_stop(
        self,
        *,
        side: str,
        shuttle: str,
        target_segment: str,
        target_s_m: float,
        tolerance_m: float,
        entry_sensor: str = '',
        minimum_clearance_delay_s: float = 0.0,
        timeout_s: float,
    ) -> dict[str, Any]:
        """Stop an interior relocation with an identity-bearing sensor guard.

        The accepted visual state remains the planner-localization source. The
        binary branch-entry sensor plus a bounded travel time owns this safety
        stop, so neither a false positive nor a false negative visual block can
        move the shuttle into FALLING. A disagreeing raw visual block/position
        is preserved for audit and is never rewritten by this execution-effect
        proof. Controller mode is used only for FALLING detection and OFF
        confirmation; controller segment/s fields are never read.
        """

        rail_side = _side(side)
        spec = normalize_shuttle_ref(shuttle, side=rail_side)
        if spec is None or spec.side != rail_side:
            return {'arrived': False, 'reason': f'unknown shuttle {shuttle!r}'}
        wanted_segment = str(target_segment or '').strip().upper()
        wanted_sensor = str(entry_sensor or '').strip().upper()
        tolerance = max(float(tolerance_m), 0.0)
        clearance_delay_s = max(float(minimum_clearance_delay_s), 0.0)
        deadline = self.monotonic() + max(float(timeout_s), 0.0)
        last_visual_sequence = -1
        last_sensor_sequence = -1
        last_position: dict[str, Any] = {}
        entry_detected_at: float | None = None
        entry_confirmation: dict[str, Any] = {}

        def visual_position_match(
            observation: dict[str, Any],
        ) -> tuple[bool, dict[str, Any]]:
            item = next(
                (
                    raw
                    for raw in observation.get('shuttles', [])
                    if isinstance(raw, dict)
                    and normalize_shuttle_ref(raw.get('identity')) == spec
                ),
                None,
            )
            if not isinstance(item, dict):
                return False, {'reason': 'identity_missing_from_visual_observation'}
            if (
                str(item.get('presence_state') or '').strip().lower() != 'present'
                or not bool(item.get('visual_facts_valid', False))
            ):
                return False, {'reason': 'visual_identity_not_valid_and_present'}
            segment = str(item.get('block') or '').strip().upper()
            try:
                observed_s_m = float(item['s_m'])
            except (KeyError, TypeError, ValueError):
                return False, {'reason': 'visual_s_m_invalid'}
            error_m = abs(observed_s_m - float(target_s_m))
            position = {
                'segment': segment,
                's_m': observed_s_m,
                'absolute_error_m': error_m,
            }
            return (
                segment == wanted_segment and error_m <= tolerance,
                position,
            )

        def publish_off(trigger: str) -> dict[str, Any]:
            previous_count = self.supervisor_decision_count()
            self.publish_command({
                'action': 'shuttle',
                'side': rail_side,
                'shuttle': shuttle,
                'command': 'OFF',
                'closed_loop_executive': {
                    'mode': 'guarded_interior_clearance_stop',
                    'target_segment': wanted_segment,
                    'target_s_m': float(target_s_m),
                    'tolerance_m': tolerance,
                    'entry_sensor': wanted_sensor,
                    'minimum_clearance_delay_s': clearance_delay_s,
                    'stop_trigger': trigger,
                    'controller_position_fields_used_for_localization': False,
                },
            })
            decision = self.wait_for_supervisor_decision(
                previous_count=previous_count,
                timeout_s=self.controller_stop_timeout_s,
            )
            if decision is None:
                return {'ready': False, 'reason': 'interior-clearance OFF timed out'}
            if not _decision_accepted(decision):
                return {
                    'ready': False,
                    'reason': (
                        'interior-clearance OFF rejected: '
                        f'{decision.get("reason", "unknown")}'
                    ),
                }
            return self._wait_controller_stopped(
                side=rail_side,
                shuttle=shuttle,
                timeout_s=self.controller_stop_timeout_s,
            )

        stop_trigger = ''
        while self.monotonic() <= deadline:
            now = self.monotonic()
            with self._condition:
                visual_sequence = self._visual_sequence
                observation = copy.deepcopy(self._visual_observation)
                sensor_sequence = self._sensor_sequence[rail_side]
                readings = copy.deepcopy(self._sensor_feedback[rail_side])
                supervisor = copy.deepcopy(self._supervisor_status)

            if visual_sequence != last_visual_sequence:
                last_visual_sequence = visual_sequence
                _visual_matched, last_position = visual_position_match(observation)

            if wanted_sensor and sensor_sequence != last_sensor_sequence:
                last_sensor_sequence = sensor_sequence
                sensor_match = _exact_target_sensor_match(
                    readings,
                    sensor=wanted_sensor,
                    shuttle=spec.gazebo_entity_name,
                    side=rail_side,
                )
                if sensor_match['error']:
                    stopped = publish_off('entry_sensor_identity_error')
                    return {
                        'arrived': False,
                        'reason': (
                            f'{sensor_match["error"]}; '
                            f'guard_stop={stopped.get("reason") or "confirmed"}'
                        ),
                    }
                if sensor_match['matched'] and entry_detected_at is None:
                    entry_detected_at = now
                    entry_confirmation = dict(sensor_match)

            if (
                not stop_trigger
                and entry_detected_at is not None
                and now - entry_detected_at >= clearance_delay_s
            ):
                stop_trigger = 'interior_entry_sensor_plus_bounded_travel_time'

            controller_state = _nested_dict(
                supervisor,
                'rails',
                rail_side,
            ).get('shuttles', {})
            controller_state = (
                controller_state.get(spec.gazebo_entity_name, {})
                if isinstance(controller_state, dict)
                else {}
            )
            if str(controller_state.get('mode') or '').strip().upper() == 'FALLING':
                stopped = publish_off('falling_mode_emergency_guard')
                return {
                    'arrived': False,
                    'reason': (
                        'controller reported FALLING before clearance stop; '
                        f'guard_stop={stopped.get("reason") or "confirmed"}'
                    ),
                    'controller_position_fields_used_for_localization': False,
                }
            if stop_trigger:
                break

            wait_s = min(0.05, max(0.0, deadline - self.monotonic()))
            if entry_detected_at is not None:
                wait_s = min(
                    wait_s,
                    max(
                        0.0,
                        clearance_delay_s
                        - (self.monotonic() - entry_detected_at),
                    ),
                )
            with self._condition:
                self._condition.wait(timeout=wait_s)
        else:
            stopped = publish_off('clearance_timeout_guard')
            return {
                'arrived': False,
                'reason': (
                    f'timeout waiting for {spec.short_id} interior clearance; '
                    f'last_visual={last_position}; '
                    f'entry_sensor_seen={entry_detected_at is not None}; '
                    f'guard_stop={stopped.get("reason") or "confirmed"}'
                ),
                'controller_position_fields_used_for_localization': False,
            }

        stopped = publish_off(stop_trigger)
        if not stopped.get('ready'):
            return {'arrived': False, 'reason': stopped.get('reason', 'stop unconfirmed')}
        post_stop_deadline = self.monotonic() + self.controller_stop_timeout_s
        post_stop_position: dict[str, Any] = {}
        post_stop_frame_received = False
        while self.monotonic() <= post_stop_deadline:
            with self._condition:
                sequence = self._visual_sequence
                post_stop_observation = copy.deepcopy(self._visual_observation)
                if sequence == last_visual_sequence:
                    self._condition.wait(timeout=min(
                        0.1,
                        max(0.0, post_stop_deadline - self.monotonic()),
                    ))
                    continue
                last_visual_sequence = sequence
            post_stop_frame_received = True
            confirmed, post_stop_position = visual_position_match(
                post_stop_observation
            )
            break
        if not post_stop_frame_received:
            return {
                'arrived': False,
                'reason': (
                    'controller stopped but no fresh accepted visual frame '
                    'arrived for clearance verification'
                ),
                'controller_position_fields_used_for_localization': False,
            }
        return {
            'arrived': True,
            'reason': '',
            'side': rail_side,
            'shuttle': spec.gazebo_entity_name,
            'target_segment': wanted_segment,
            'target_s_m': float(target_s_m),
            'observed_segment': post_stop_position.get('segment', ''),
            'observed_s_m': post_stop_position.get('s_m'),
            'absolute_error_m': post_stop_position.get('absolute_error_m'),
            'tolerance_m': tolerance,
            'matched_by': stop_trigger,
            'entry_sensor': wanted_sensor,
            'entry_sensor_identity_confirmed': bool(entry_confirmation),
            'post_stop_visual_frame_received': True,
            'post_stop_visual_confirmation': bool(confirmed),
            'controller_stop_confirmed': True,
            'controller_position_fields_used_for_localization': False,
        }

    def _stop_after_sensor_arrival(
        self,
        *,
        side: str,
        shuttle: str,
        target_slot: str,
        target_sensor: str,
    ) -> dict[str, Any]:
        setpoint_result = self._wait_controller_stopped(
            side=side,
            shuttle=shuttle,
            timeout_s=self.controller_stop_timeout_s,
            target_slot=target_slot,
        )
        if not setpoint_result['ready']:
            guard_result = self._publish_arrival_off(
                side=side,
                shuttle=shuttle,
                target_slot=target_slot,
                target_sensor=target_sensor,
                mode='slot_sensor_setpoint_timeout_guard',
            )
            return {
                'ready': False,
                'reason': (
                    f'{setpoint_result["reason"]}; '
                    f'guard_off={guard_result["reason"] or "confirmed"}'
                ),
            }
        finalize_result = self._publish_arrival_off(
            side=side,
            shuttle=shuttle,
            target_slot=target_slot,
            target_sensor=target_sensor,
            mode='slot_sensor_target_arrival_finalize',
        )
        if not finalize_result['ready']:
            return finalize_result
        return {
            **finalize_result,
            'controller_target_slot_confirmed': True,
        }

    def _publish_arrival_off(
        self,
        *,
        side: str,
        shuttle: str,
        target_slot: str,
        target_sensor: str,
        mode: str,
    ) -> dict[str, Any]:
        previous_count = self.supervisor_decision_count()
        self.publish_command({
            'action': 'shuttle',
            'side': side,
            'shuttle': shuttle,
            'command': 'OFF',
            'closed_loop_executive': {
                'mode': str(mode),
                'target_slot': str(target_slot),
                'target_sensor': str(target_sensor),
                'final_arrival_source': 'deterministic_slot_sensor',
                'planner_localization_source': 'accepted_visual_state',
            },
        })
        decision = self.wait_for_supervisor_decision(
            previous_count=previous_count,
            timeout_s=self.controller_stop_timeout_s,
        )
        if decision is None:
            return {'ready': False, 'reason': 'slot-sensor arrival OFF command timed out'}
        if not _decision_accepted(decision):
            return {
                'ready': False,
                'reason': (
                    'slot-sensor arrival OFF command rejected: '
                    f'{decision.get("reason", "unknown")}'
                ),
            }
        return self._wait_controller_stopped(
            side=side,
            shuttle=shuttle,
            timeout_s=self.controller_stop_timeout_s,
            target_slot=(
                target_slot
                if mode == 'slot_sensor_target_arrival_finalize'
                else ''
            ),
        )

    def _wait_controller_stopped(
        self,
        *,
        side: str,
        shuttle: str,
        timeout_s: float,
        target_slot: str = '',
    ) -> dict[str, Any]:
        spec = normalize_shuttle_ref(shuttle, side=side)
        if spec is None:
            return {'ready': False, 'reason': f'unknown shuttle {shuttle!r}'}
        deadline = self.monotonic() + max(float(timeout_s), 0.0)
        while self.monotonic() <= deadline:
            with self._condition:
                rail = _nested_dict(self._supervisor_status, 'rails', side)
                shuttles = rail.get('shuttles')
                state = (
                    shuttles.get(spec.gazebo_entity_name, {})
                    if isinstance(shuttles, dict)
                    else {}
                )
                mode = str(state.get('mode') or '').strip().upper()
                reached_target_slot = str(
                    state.get('reached_target_slot') or ''
                ).strip()
                if (
                    mode in STOPPED_MOTION_VALUES
                    and (
                        not target_slot
                        or reached_target_slot == str(target_slot)
                    )
                ):
                    return {
                        'ready': True,
                        'reason': '',
                        'mode': mode,
                        'confirmation_source': 'controller_execution_feedback',
                        'localization_source': 'not_used',
                        'reached_target_slot': reached_target_slot,
                    }
                self._condition.wait(
                    timeout=min(0.1, max(0.0, deadline - self.monotonic()))
                )
        return {
            'ready': False,
            'reason': (
                f'timeout confirming controller stop for {spec.short_id}'
                + (
                    f' at target_slot {target_slot}'
                    if target_slot
                    else ''
                )
            ),
        }

    def _wait_device_state(
        self,
        *,
        side: str,
        kind: str,
        expected: dict[str, str],
        timeout_s: float,
        normalize: Callable[[Any], str],
    ) -> dict[str, Any]:
        deadline = self.monotonic() + max(float(timeout_s), 0.0)
        while self.monotonic() <= deadline:
            with self._condition:
                values = _nested_dict(
                    self._supervisor_status,
                    'rails',
                    side,
                ).get(kind)
                if isinstance(values, dict) and all(
                    normalize(values.get(device)) == wanted
                    for device, wanted in expected.items()
                ):
                    return {'ready': True, 'reason': ''}
                self._condition.wait(
                    timeout=min(0.1, max(0.0, deadline - self.monotonic()))
                )
        return {
            'ready': False,
            'reason': f'timeout waiting for {side} {kind} state {expected}',
        }


def ground_transport_task_goal(
    task_goal: TaskGoal,
    observed_state: ObservedState,
) -> TaskGoal:
    """Ground nearest/any language selection from accepted visual facts.

    This is deterministic goal grounding, not action planning. Location and
    payload eligibility come from the learned visual state; PlanSys2 still
    owns route/action planning after an explicit candidate is selected.
    """

    constraints = dict(task_goal.constraints or {})
    if str(constraints.get('goal_type') or '').lower() != 'transport':
        raise TaskExecutionStateError('only transport TaskGoals can be grounded')
    side = _side(constraints.get('side'))
    existing = constraints.get('target_shuttle')
    if existing:
        spec = normalize_shuttle_ref(existing, side=side)
        if spec is None or spec.side != side:
            raise TaskExecutionStateError(
                f'invalid explicit target shuttle:{existing!r}'
            )
        constraints['target_shuttle'] = spec.gazebo_entity_name
        constraints['selection_strategy'] = 'explicit'
        constraints['shuttle_selection'] = 'explicit'
        return TaskGoal(
            goal_id=task_goal.goal_id,
            description=task_goal.description,
            source=task_goal.source,
            timestamp=task_goal.timestamp,
            confidence=task_goal.confidence,
            constraints=constraints,
        )

    selection = str(
        constraints.get('selection_strategy')
        or constraints.get('shuttle_selection')
        or 'any'
    ).lower()
    payload_filter = str(
        constraints.get('payload_filter')
        or constraints.get('payload_required')
        or 'any'
    ).lower()
    target_slot = str(constraints.get('target_slot') or '')
    if selection == 'nearest' and target_slot not in {'1', '2', '3', '4'}:
        raise TaskExecutionStateError(
            'nearest selection requires target slot 1, 2, 3, or 4'
        )
    facts = {
        (fact.subject, fact.predicate): fact
        for fact in observed_state.fused_planner_state
        if fact.status == 'known'
    }
    candidates = []
    for spec in all_shuttle_specs():
        if spec.side != side:
            continue
        present = facts.get((spec.gazebo_entity_name, 'present'))
        if present is None or not bool(present.value):
            continue
        loaded = facts.get((spec.gazebo_entity_name, 'loaded'))
        if loaded is None:
            continue
        if payload_filter == 'loaded' and not bool(loaded.value):
            continue
        if payload_filter == 'empty' and bool(loaded.value):
            continue
        slot_fact = facts.get((spec.gazebo_entity_name, 'location_slot'))
        slot = _slot_number(slot_fact.value if slot_fact is not None else '')
        if selection == 'nearest' and not slot:
            continue
        distance = (
            abs(int(slot) - int(target_slot))
            if selection == 'nearest'
            else 0
        )
        candidates.append((distance, spec.short_id, spec))
    if not candidates:
        raise TaskExecutionStateError(
            f'no visual candidate for selection={selection}, '
            f'payload_filter={payload_filter}, side={side}'
        )
    candidates.sort(key=lambda item: (item[0], item[1]))
    selected = candidates[0][2]
    constraints['target_shuttle'] = selected.gazebo_entity_name
    constraints['selection_strategy'] = 'explicit'
    constraints['shuttle_selection'] = 'explicit'
    return TaskGoal(
        goal_id=task_goal.goal_id,
        description=task_goal.description,
        source=task_goal.source,
        timestamp=task_goal.timestamp,
        confidence=task_goal.confidence,
        constraints=constraints,
    )


def _fact(
    *,
    source: str,
    subject: str,
    predicate: str,
    value: Any,
    timestamp_s: float,
    metadata: dict[str, Any],
    confidence: float = 1.0,
) -> ObservedFact:
    return ObservedFact(
        fact_id=(
            f'{source}-{subject}-{predicate}'
            .replace(':', '-')
            .replace('_', '-')
        ),
        subject=subject,
        predicate=predicate,
        value=copy.deepcopy(value),
        source=source,
        timestamp=float(timestamp_s),
        confidence=float(confidence),
        status='known',
        metadata=copy.deepcopy(metadata),
    )


def _side(value: Any) -> str:
    text = str(value or '').strip().lower()
    if text not in {'left', 'right'}:
        raise TaskExecutionStateError(f'invalid rail side:{value!r}')
    return text


def _exact_target_sensor_match(
    readings: list[dict[str, Any]],
    *,
    sensor: str,
    shuttle: str,
    side: str,
) -> dict[str, Any]:
    """Match a binary slot sensor without reading any position fields."""

    wanted_sensor = str(sensor).strip().upper()
    expected = normalize_shuttle_ref(shuttle)
    if expected is None or expected.side != side:
        return {
            'matched': False,
            'error': f'invalid expected shuttle identity {shuttle!r}',
        }
    matching = [
        reading
        for reading in readings
        if str(reading.get('name') or '').strip().upper() == wanted_sensor
    ]
    if not matching:
        return {'matched': False, 'error': ''}

    raw_identities = {
        str(reading.get('shuttle') or '').strip()
        for reading in matching
    }
    if '' in raw_identities:
        return {
            'matched': False,
            'error': f'{wanted_sensor} is active with unknown shuttle identity',
        }
    normalized = []
    for raw_identity in sorted(raw_identities):
        observed = normalize_shuttle_ref(raw_identity)
        if observed is None:
            return {
                'matched': False,
                'error': (
                    f'{wanted_sensor} reported unmapped shuttle '
                    f'{raw_identity!r}'
                ),
            }
        if observed.side != side:
            return {
                'matched': False,
                'error': (
                    f'{wanted_sensor} reported {observed.short_id} on the '
                    f'wrong rail side {side}'
                ),
            }
        normalized.append(observed.gazebo_entity_name)
    identities = set(normalized)
    if len(identities) != 1:
        return {
            'matched': False,
            'error': (
                f'{wanted_sensor} reported duplicate identities '
                f'{sorted(identities)}'
            ),
        }
    observed_identity = next(iter(identities))
    if observed_identity != expected.gazebo_entity_name:
        return {
            'matched': False,
            'error': (
                f'{wanted_sensor} is occupied by {observed_identity}, '
                f'expected {expected.gazebo_entity_name}'
            ),
        }
    return {
        'matched': True,
        'error': '',
        'shuttle': observed_identity,
        'sensor': wanted_sensor,
    }


def _nested_dict(root: Any, *keys: str) -> dict[str, Any]:
    value = root
    for key in keys:
        if not isinstance(value, dict):
            return {}
        value = value.get(key)
    return value if isinstance(value, dict) else {}


def _expand_devices(assignments: dict[str, Any]) -> dict[str, Any]:
    if 'ALL' in assignments:
        return {device: assignments['ALL'] for device in DEVICE_NAMES}
    return {
        str(device).strip().upper(): value
        for device, value in assignments.items()
        if str(device).strip().upper() in DEVICE_NAMES
    }


def _normalize_switch(value: Any) -> str:
    text = str(value or '').strip().upper()
    if text in {'E', 'EXTERIOR'}:
        return 'EXTERIOR'
    if text in {'I', 'INTERIOR'}:
        return 'INTERIOR'
    return text


def _normalize_stopper(value: Any) -> str:
    text = str(value or '').strip().upper()
    if text in {'0', 'OFF', 'OPEN', 'PASS', 'RELEASE'}:
        return 'OPEN'
    if text in {'1', 'ON', 'CLOSED', 'STOP', 'BLOCKED'}:
        return 'CLOSED'
    return text


def _decision_accepted(decision: dict[str, Any]) -> bool:
    if decision.get('accepted') is False:
        return False
    status = str(
        decision.get('status')
        or decision.get('decision')
        or ''
    ).strip().lower()
    if status in {'rejected', 'failed', 'blocked', 'timed_out', 'timeout'}:
        return False
    return bool(decision.get('accepted', status in {'accepted', 'approved', 'executed'}))


def _slot_number(value: Any) -> str:
    text = str(value or '').strip()
    if text and text[-1] in {'1', '2', '3', '4'}:
        return text[-1]
    return ''
