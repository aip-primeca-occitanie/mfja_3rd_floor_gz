#!/usr/bin/env python3
"""ROS-independent state and transport core for Room 315 task execution.

The visual observation owns shuttle location and payload facts.  Controller
state is admitted only for presence (upstream of this module), switch/stopper
state, safety decisions, and a fresh explicit ``DISABLED`` confirmation that
an OFF command took effect.
In particular, supervisor ShuttleState position fields are never read here.
"""

from __future__ import annotations

import copy
import itertools
import math
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
from room_315_multi_shuttle import route_candidates_from_position_to_slot
from room_315_observed_state_provider import ObservedStateProvider
from room_315_observed_state_provider import fuse_observed_facts
from room_315_presence_provider import PRESENCE_STATES
from room_315_pddl_scenario_generator import RAIL_DEVICES_PATH_BY_SIDE
from room_315_pddl_scenario_generator import RAIL_NETWORK_PATH_BY_SIDE
from room_315_pddl_scenario_generator import ScenarioTransport
from room_315_pddl_scenario_generator import SLOT_SENSOR_BY_SIDE_AND_SLOT
from room_315_pddl_scenario_generator import SLOT_STATION_BY_SIDE_AND_SLOT
from room_315_multi_shuttle import load_rail_topology
from room_315_rail_defaults import LEFT_PUBLIC_SEGMENT_NAME_MAP
from room_315_rail_defaults import rail_segment_lengths
from room_315_runtime_contracts import CONTROLLER_DISABLED_MODE
from room_315_runtime_contracts import MIN_PAYLOAD_CONFIRMATION_FRAMES
from room_315_runtime_contracts import create_runtime_payload_grounding
from room_315_runtime_contracts import create_visual_payload_confirmation
from room_315_runtime_contracts import normalize_runtime_clearance_certificate
from room_315_runtime_contracts import supervisor_decision_accepted
from room_315_runtime_contracts import supervisor_decision_is_terminal


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
            if not math.isfinite(value) or value <= 0.0:
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
        verified_slot_arrival_certificates: (
            dict[str, dict[str, Any]] | None
        ) = None,
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
        verified_slot_arrivals = (
            self._verified_slot_arrivals_by_identity(
                verified_slot_arrival_certificates,
                sensor_anchors=sensor_anchors,
            )
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
                if spec.short_id in verified_slot_arrivals:
                    raise TaskExecutionStateError(
                        'verified slot arrival references absent shuttle:'
                        f'{spec.short_id}'
                    )
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
            verified_slot_arrival = verified_slot_arrivals.get(spec.short_id)
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

            if verified_slot_arrival is not None:
                trusted_facts.append(_fact(
                    source='executor',
                    subject=spec.gazebo_entity_name,
                    predicate='verified_slot_arrival',
                    value=verified_slot_arrival,
                    timestamp_s=timestamp_s,
                    metadata={
                        'field_owner': 'verified_deterministic_slot_arrival',
                        'identity': spec.short_id,
                        'side': spec.side,
                        'slot': slot,
                        'sensor': sensor_anchor['sensor'],
                        'categorical_location_scope_only': True,
                        'model_prediction_replaced': False,
                        'controller_position_fields_used_for_localization': False,
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
    def _verified_slot_arrivals_by_identity(
        raw: dict[str, dict[str, Any]] | None,
        *,
        sensor_anchors: dict[str, dict[str, Any]],
    ) -> dict[str, dict[str, Any]]:
        """Validate executor slot proof without controller localization.

        A raw active DZI reading is deliberately insufficient.  The
        certificate is created by the arrival transport, or reconstructed
        after a runtime restart, only after the same identity-bearing DZI
        stays active for the configured confirmation count and fresh
        controller status proves the shuttle is disabled.  A simulator-start
        shuttle has no commanded ``reached_target_slot`` yet, so its stable
        stopped DZI is admitted under a distinct initial-occupancy proof.
        """

        certificates: dict[str, dict[str, Any]] = {}
        for raw_identity, raw_certificate in dict(raw or {}).items():
            if not isinstance(raw_certificate, dict):
                raise TaskExecutionStateError(
                    'verified slot arrival certificate must be an object'
                )
            certificate = copy.deepcopy(raw_certificate)
            spec = normalize_shuttle_ref(raw_identity)
            certificate_spec = normalize_shuttle_ref(
                certificate.get('identity')
            )
            shuttle_spec = normalize_shuttle_ref(
                certificate.get('shuttle')
            )
            if spec is None or certificate_spec is None or shuttle_spec is None:
                raise TaskExecutionStateError(
                    f'unknown verified slot arrival identity:{raw_identity}'
                )
            if not (
                spec.short_id
                == certificate_spec.short_id
                == shuttle_spec.short_id
            ):
                raise TaskExecutionStateError(
                    f'verified slot arrival identity conflict:{raw_identity}'
                )
            side = _side(certificate.get('side'))
            slot = str(certificate.get('slot') or '').strip()
            sensor = str(certificate.get('sensor') or '').strip().upper()
            expected_sensor = SLOT_SENSOR_BY_SIDE_AND_SLOT.get((side, slot), '')
            if side != spec.side:
                raise TaskExecutionStateError(
                    f'verified slot arrival side conflict:{spec.short_id}:{side}'
                )
            if not expected_sensor or sensor != expected_sensor.upper():
                raise TaskExecutionStateError(
                    f'verified slot arrival sensor conflict:{spec.short_id}:'
                    f'{sensor or "missing"}'
                )
            anchor = sensor_anchors.get(spec.short_id)
            if not isinstance(anchor, dict) or (
                anchor.get('side') != side
                or str(anchor.get('slot') or '') != slot
                or str(anchor.get('sensor') or '').upper() != sensor
            ):
                raise TaskExecutionStateError(
                    'verified slot arrival lacks matching fresh DZI anchor:'
                    f'{spec.short_id}'
                )
            proof_mode = str(certificate.get('proof_mode') or '')
            reached_target_slot = str(
                certificate.get('reached_target_slot') or ''
            ).strip()
            initial_occupancy_proof = (
                proof_mode == 'stable_stopped_dzi_initial_occupancy'
                and certificate.get('controller_target_slot_confirmed')
                is False
                and not reached_target_slot
            )
            commanded_arrival_proof = (
                proof_mode in {
                    'supervised_command_arrival',
                    'stable_stopped_dzi_runtime_recovery',
                }
                and certificate.get('controller_target_slot_confirmed')
                is True
                and reached_target_slot == slot
            )
            if (
                certificate.get('matched_by') != 'deterministic_slot_sensor'
                or not (
                    initial_occupancy_proof
                    or commanded_arrival_proof
                )
                or certificate.get('sensor_identity_confirmed') is not True
                or certificate.get('controller_stop_confirmed') is not True
                or certificate.get('controller_mode') != CONTROLLER_DISABLED_MODE
                or certificate.get('model_prediction_replaced') is not False
                or certificate.get(
                    'controller_position_fields_used_for_localization'
                ) is not False
            ):
                raise TaskExecutionStateError(
                    f'invalid verified slot arrival proof:{spec.short_id}'
                )
            try:
                confirmation_frames = int(
                    certificate.get('sensor_confirmation_frames')
                )
                sensor_sequence = int(certificate.get('sensor_sequence'))
                supervisor_sequence = int(certificate.get('supervisor_sequence'))
                motion_epoch = int(certificate.get('motion_epoch'))
            except (TypeError, ValueError) as exc:
                raise TaskExecutionStateError(
                    f'invalid verified slot arrival sequence:{spec.short_id}'
                ) from exc
            if min(
                confirmation_frames,
                sensor_sequence,
                supervisor_sequence,
            ) < 1:
                raise TaskExecutionStateError(
                    f'invalid verified slot arrival sequence:{spec.short_id}'
                )
            if motion_epoch < 0:
                raise TaskExecutionStateError(
                    f'invalid verified slot arrival motion epoch:{spec.short_id}'
                )
            if spec.short_id in certificates:
                raise TaskExecutionStateError(
                    f'duplicate verified slot arrival identity:{spec.short_id}'
                )
            certificate.update({
                'identity': spec.short_id,
                'shuttle': spec.gazebo_entity_name,
                'side': side,
                'slot': slot,
                'sensor': sensor,
            })
            certificates[spec.short_id] = certificate
        return certificates

    @staticmethod
    def _clearance_certificates_by_identity(
        raw: dict[str, dict[str, Any]] | None,
    ) -> dict[str, dict[str, Any]]:
        if raw is not None and not isinstance(raw, dict):
            raise TaskExecutionStateError(
                'runtime clearance certificates must be a mapping'
            )
        certificates: dict[str, dict[str, Any]] = {}
        for raw_identity, raw_certificate in dict(raw or {}).items():
            try:
                certificate = normalize_runtime_clearance_certificate(
                    raw_identity,
                    raw_certificate,
                )
            except ValueError as exc:
                raise TaskExecutionStateError(str(exc)) from exc
            spec = normalize_shuttle_ref(certificate['identity'])
            assert spec is not None  # guaranteed by shared normalization
            if spec.short_id in certificates:
                raise TaskExecutionStateError(
                    f'duplicate runtime clearance identity:{spec.short_id}'
                )
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
            bbox_values = tuple(float(value) for value in bbox)
        except (TypeError, ValueError) as exc:
            raise TaskExecutionStateError(
                f'invalid_visual_bbox:{identity}'
            ) from exc
        try:
            s_m = float(item.get('s_m'))
            ratio = float(item.get('s_ratio'))
            length = float(item.get('segment_length_m'))
        except (TypeError, ValueError) as exc:
            raise TaskExecutionStateError(
                f'invalid_visual_position:{identity}'
            ) from exc
        if (
            not all(math.isfinite(value) for value in bbox_values)
            # The runtime schema carries camera-pixel xywh, not normalized
            # coordinates. Image dimensions are not part of this boundary,
            # and an accepted detection may be clipped at an image edge, so
            # only finiteness and positive extents can be validated here;
            # image intersection is validated upstream.
            or bbox_values[2] <= 0.0
            or bbox_values[3] <= 0.0
        ):
            raise TaskExecutionStateError(f'invalid_visual_bbox:{identity}')
        if (
            not all(math.isfinite(value) for value in (s_m, ratio, length))
            or length <= 0.0
            or not 0.0 <= s_m <= length
            or not 0.0 <= ratio <= 1.0
        ):
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
        slot_sensor_confirmation_frames: int = 2,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.builder = builder
        self.slot_sensor_confirmation_frames = max(
            int(slot_sensor_confirmation_frames),
            1,
        )
        self.monotonic = monotonic
        self._condition = threading.Condition()
        self._observation: dict[str, Any] = {}
        self._observation_receive_s = -1.0
        self._supervisor_status: dict[str, Any] = {}
        self._supervisor_receive_s = -1.0
        self._supervisor_sequence = 0
        self._runtime_clearance_certificates: dict[str, dict[str, Any]] = {}
        self._verified_slot_arrival_certificates: dict[
            str,
            dict[str, Any],
        ] = {}
        # Monotonic per-shuttle command generation.  An arrival proof is
        # consumable only in the same generation in which its verification
        # started.  This closes the interval between publishing a later
        # motion command and receiving the first MOVING controller frame.
        self._verified_slot_motion_epoch_by_identity: dict[str, int] = {}
        self._verified_slot_bootstrap_suppressed: set[str] = set()
        self._slot_sensor_anchors_by_side: dict[
            str,
            dict[str, dict[str, Any]],
        ] = {'left': {}, 'right': {}}
        self._slot_sensor_receive_s = {'left': -1.0, 'right': -1.0}
        self._slot_sensor_sequence = {'left': 0, 'right': 0}
        self._slot_sensor_stability_by_side: dict[
            str,
            dict[str, dict[str, Any]],
        ] = {'left': {}, 'right': {}}
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
            presence_by_identity = {
                str(item.get('identity') or '').strip().upper(): str(
                    item.get('presence_state') or ''
                ).strip().lower()
                for item in observation.get('shuttles', [])
                if isinstance(item, dict)
            }
            for identity in list(self._verified_slot_arrival_certificates):
                if presence_by_identity.get(identity) != 'present':
                    self._verified_slot_arrival_certificates.pop(identity, None)
            self._condition.notify_all()

    def update_supervisor(
        self,
        status: dict[str, Any],
        *,
        receive_s: float | None = None,
    ) -> None:
        with self._condition:
            current_receive_s = (
                self.monotonic() if receive_s is None else float(receive_s)
            )
            previous_age = current_receive_s - self._supervisor_receive_s
            if (
                self._supervisor_receive_s >= 0.0
                and previous_age > self.builder.config.supervisor_status_timeout_s
            ):
                # A proof may not bridge a controller telemetry outage, even
                # if the first frame after the outage repeats the old value.
                self._verified_slot_arrival_certificates.clear()
                self._slot_sensor_stability_by_side = {
                    'left': {},
                    'right': {},
                }
            self._supervisor_status = copy.deepcopy(status)
            self._supervisor_receive_s = current_receive_s
            self._supervisor_sequence += 1
            for identity, certificate in list(
                self._verified_slot_arrival_certificates.items()
            ):
                if not self._verified_slot_arrival_is_current_locked(
                    certificate,
                    now_s=current_receive_s,
                ):
                    self._verified_slot_arrival_certificates.pop(identity, None)
            self._bootstrap_verified_slot_arrivals_locked(
                now_s=current_receive_s
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
        raw_identity = (
            certificate.get('identity')
            if isinstance(certificate, dict)
            else None
        )
        try:
            normalized = normalize_runtime_clearance_certificate(
                raw_identity,
                certificate,
            )
        except ValueError as exc:
            raise TaskExecutionStateError(str(exc)) from exc
        with self._condition:
            self._runtime_clearance_certificates[
                normalized['identity']
            ] = normalized
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

    def set_verified_slot_arrival_certificate(
        self,
        certificate: dict[str, Any],
    ) -> None:
        spec = normalize_shuttle_ref(certificate.get('identity'))
        if spec is None:
            raise TaskExecutionStateError('unknown verified slot arrival identity')
        with self._condition:
            if not self._verified_slot_arrival_is_current_locked(
                certificate,
                now_s=self.monotonic(),
            ):
                raise TaskExecutionStateError(
                    'verified slot arrival proof is not current in provider:'
                    f'{spec.short_id}'
                )
            self._verified_slot_arrival_certificates[
                spec.short_id
            ] = copy.deepcopy(certificate)
            # A stopped, identity-bearing DZI arrival is the authoritative
            # end of an earlier interior-staging effect.  Consume that effect
            # only after the complete arrival proof is current; a single raw
            # sensor frame is not sufficient to erase it.
            self._runtime_clearance_certificates.pop(spec.short_id, None)
            self._verified_slot_bootstrap_suppressed.discard(spec.short_id)
            self._condition.notify_all()

    def verified_slot_motion_epoch(
        self,
        shuttle: str,
        *,
        side: str | None = None,
    ) -> int:
        """Return the current local motion-command generation."""

        spec = normalize_shuttle_ref(shuttle, side=side)
        if spec is None:
            raise TaskExecutionStateError(
                f'unknown verified slot arrival identity:{shuttle}'
            )
        with self._condition:
            return self._verified_slot_motion_epoch_by_identity.get(
                spec.short_id,
                0,
            )

    def invalidate_verified_slot_arrival_for_motion(
        self,
        shuttle: str,
        *,
        side: str | None = None,
    ) -> int:
        """Atomically consume proof and advance its command generation."""

        spec = normalize_shuttle_ref(shuttle, side=side)
        if spec is None:
            raise TaskExecutionStateError(
                f'unknown motion-capable shuttle identity:{shuttle}'
            )
        with self._condition:
            next_epoch = (
                self._verified_slot_motion_epoch_by_identity.get(
                    spec.short_id,
                    0,
                )
                + 1
            )
            self._verified_slot_motion_epoch_by_identity[
                spec.short_id
            ] = next_epoch
            self._verified_slot_arrival_certificates.pop(spec.short_id, None)
            self._verified_slot_bootstrap_suppressed.add(spec.short_id)
            self._condition.notify_all()
            return next_epoch

    def clear_verified_slot_arrival_certificate(
        self,
        shuttle: str,
        *,
        side: str | None = None,
    ) -> None:
        spec = normalize_shuttle_ref(shuttle, side=side)
        if spec is None:
            return
        with self._condition:
            self._verified_slot_arrival_certificates.pop(spec.short_id, None)
            self._verified_slot_bootstrap_suppressed.add(spec.short_id)
            self._condition.notify_all()

    def verified_slot_arrival_certificates(
        self,
        *,
        now_s: float | None = None,
    ) -> dict[str, dict[str, Any]]:
        """Return proof still bound to fresh matching DZI/controller state."""

        current = self.monotonic() if now_s is None else float(now_s)
        with self._condition:
            for identity, certificate in list(
                self._verified_slot_arrival_certificates.items()
            ):
                if not self._verified_slot_arrival_is_current_locked(
                    certificate,
                    now_s=current,
                ):
                    self._verified_slot_arrival_certificates.pop(identity, None)
            return copy.deepcopy(self._verified_slot_arrival_certificates)

    def _verified_slot_arrival_is_current_locked(
        self,
        certificate: dict[str, Any],
        *,
        now_s: float,
    ) -> bool:
        """Check both independent telemetry sources under the provider lock."""

        spec = normalize_shuttle_ref(certificate.get('identity'))
        if spec is None:
            return False
        try:
            certificate_motion_epoch = int(certificate.get('motion_epoch'))
        except (TypeError, ValueError):
            return False
        if certificate_motion_epoch != (
            self._verified_slot_motion_epoch_by_identity.get(spec.short_id, 0)
        ):
            return False
        side = str(certificate.get('side') or '').strip().casefold()
        slot = str(certificate.get('slot') or '').strip()
        sensor = str(certificate.get('sensor') or '').strip().upper()
        if side != spec.side or not slot or not sensor:
            return False
        sensor_age = now_s - self._slot_sensor_receive_s.get(side, -1.0)
        if (
            sensor_age < 0.0
            or sensor_age > self.builder.config.slot_sensor_state_timeout_s
            or self._slot_sensor_error.get(side)
        ):
            return False
        anchor = self._slot_sensor_anchors_by_side.get(side, {}).get(
            spec.short_id
        )
        if not isinstance(anchor, dict) or (
            str(anchor.get('slot') or '') != slot
            or str(anchor.get('sensor') or '').upper() != sensor
        ):
            return False
        supervisor_age = now_s - self._supervisor_receive_s
        if (
            supervisor_age < 0.0
            or supervisor_age
            > self.builder.config.supervisor_status_timeout_s
        ):
            return False
        shuttles = _nested_dict(
            self._supervisor_status,
            'rails',
            side,
        ).get('shuttles')
        controller_state = (
            shuttles.get(spec.gazebo_entity_name, {})
            if isinstance(shuttles, dict)
            else {}
        )
        controller_mode = str(
            controller_state.get('mode') or ''
        ).strip().upper()
        controller_reached_slot = str(
            controller_state.get('reached_target_slot') or ''
        ).strip()
        proof_mode = str(certificate.get('proof_mode') or '')
        if proof_mode == 'stable_stopped_dzi_initial_occupancy':
            target_evidence_is_current = bool(
                certificate.get('controller_target_slot_confirmed') is False
                and not str(
                    certificate.get('reached_target_slot') or ''
                ).strip()
                and not controller_reached_slot
            )
        elif proof_mode in {
            'supervised_command_arrival',
            'stable_stopped_dzi_runtime_recovery',
        }:
            target_evidence_is_current = bool(
                certificate.get('controller_target_slot_confirmed') is True
                and str(
                    certificate.get('reached_target_slot') or ''
                ).strip() == slot
                and controller_reached_slot == slot
            )
        else:
            target_evidence_is_current = False
        return bool(
            controller_mode == CONTROLLER_DISABLED_MODE
            and target_evidence_is_current
        )

    def _bootstrap_verified_slot_arrivals_locked(
        self,
        *,
        now_s: float,
    ) -> None:
        """Recover proof after a runtime restart from stable stopped sources.

        This never uses controller position fields. Recovery requires the same
        unique identity-bearing DZI for the full confirmation count plus fresh
        ``DISABLED`` feedback. A matching ``reached_target_slot`` reconstructs
        a commanded arrival. An empty value reconstructs only initial stopped
        occupancy; a non-empty mismatch is rejected. A locally issued motion
        command suppresses both forms until the arrival transport registers
        the new supervised result.
        """

        for side in ('left', 'right'):
            for identity, stable in self._slot_sensor_stability_by_side[
                side
            ].items():
                if (
                    identity in self._verified_slot_arrival_certificates
                    or identity in self._verified_slot_bootstrap_suppressed
                    or int(stable.get('count') or 0)
                    < self.slot_sensor_confirmation_frames
                ):
                    continue
                spec = normalize_shuttle_ref(identity)
                anchor = stable.get('anchor')
                if spec is None or not isinstance(anchor, dict):
                    continue
                slot = str(anchor.get('slot') or '')
                shuttles = _nested_dict(
                    self._supervisor_status,
                    'rails',
                    side,
                ).get('shuttles')
                controller_state = (
                    shuttles.get(spec.gazebo_entity_name, {})
                    if isinstance(shuttles, dict)
                    else {}
                )
                controller_mode = str(
                    controller_state.get('mode') or ''
                ).strip().upper()
                reached_target_slot = str(
                    controller_state.get('reached_target_slot') or ''
                ).strip()
                if (
                    controller_mode != CONTROLLER_DISABLED_MODE
                    or reached_target_slot not in {'', slot}
                ):
                    continue
                controller_target_slot_confirmed = (
                    reached_target_slot == slot
                )
                certificate = {
                    'identity': spec.short_id,
                    'shuttle': spec.gazebo_entity_name,
                    'side': side,
                    'slot': slot,
                    'sensor': str(anchor.get('sensor') or '').upper(),
                    'matched_by': 'deterministic_slot_sensor',
                    'proof_mode': (
                        'stable_stopped_dzi_runtime_recovery'
                        if controller_target_slot_confirmed
                        else 'stable_stopped_dzi_initial_occupancy'
                    ),
                    'motion_epoch': (
                        self._verified_slot_motion_epoch_by_identity.get(
                            spec.short_id,
                            0,
                        )
                    ),
                    'sensor_identity_confirmed': True,
                    'sensor_confirmation_frames': int(stable['count']),
                    'sensor_sequence': self._slot_sensor_sequence[side],
                    'controller_stop_confirmed': True,
                    'controller_mode': CONTROLLER_DISABLED_MODE,
                    'controller_target_slot_confirmed': (
                        controller_target_slot_confirmed
                    ),
                    'reached_target_slot': reached_target_slot,
                    'supervisor_sequence': self._supervisor_sequence,
                    'model_prediction_replaced': False,
                    'controller_position_fields_used_for_localization': False,
                }
                if self._verified_slot_arrival_is_current_locked(
                    certificate,
                    now_s=now_s,
                ):
                    self._verified_slot_arrival_certificates[
                        identity
                    ] = certificate
                    self._runtime_clearance_certificates.pop(identity, None)

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
            current_receive_s = (
                self.monotonic() if receive_s is None else float(receive_s)
            )
            previous_age = (
                current_receive_s - self._slot_sensor_receive_s[rail_side]
            )
            source_gap = (
                self._slot_sensor_receive_s[rail_side] >= 0.0
                and previous_age > self.builder.config.slot_sensor_state_timeout_s
            )
            if source_gap:
                # Do not let one repeated post-outage frame resurrect an old
                # multi-frame arrival certificate.
                for identity, certificate in list(
                    self._verified_slot_arrival_certificates.items()
                ):
                    if certificate.get('side') == rail_side:
                        self._verified_slot_arrival_certificates.pop(
                            identity,
                            None,
                        )
                self._slot_sensor_stability_by_side[rail_side] = {}
            previous_stability = self._slot_sensor_stability_by_side[rail_side]
            next_stability: dict[str, dict[str, Any]] = {}
            if not error:
                for identity, anchor in anchors.items():
                    previous = previous_stability.get(identity, {})
                    same_anchor = previous.get('anchor') == anchor
                    next_stability[identity] = {
                        'anchor': copy.deepcopy(anchor),
                        'count': (
                            int(previous.get('count') or 0) + 1
                            if same_anchor and not source_gap
                            else 1
                        ),
                    }
            self._slot_sensor_stability_by_side[
                rail_side
            ] = next_stability
            self._slot_sensor_anchors_by_side[rail_side] = anchors
            self._slot_sensor_receive_s[rail_side] = current_receive_s
            self._slot_sensor_sequence[rail_side] += 1
            self._slot_sensor_error[rail_side] = error
            for identity, certificate in list(
                self._verified_slot_arrival_certificates.items()
            ):
                if certificate.get('side') != rail_side:
                    continue
                anchor = anchors.get(identity)
                if error or not isinstance(anchor, dict) or (
                    str(anchor.get('slot') or '')
                    != str(certificate.get('slot') or '')
                    or str(anchor.get('sensor') or '').upper()
                    != str(certificate.get('sensor') or '').upper()
                ):
                    self._verified_slot_arrival_certificates.pop(identity, None)
            self._bootstrap_verified_slot_arrivals_locked(
                now_s=current_receive_s
            )
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
            self._build_current_state()
        except Exception as exc:  # noqa: BLE001 - readiness explanation
            return False, str(exc)
        return True, ''

    def _build_current_state(self) -> ObservedState:
        """Build one state from a consistently validated provider snapshot."""

        now_s = self.monotonic()
        return self.builder.build(
            self.snapshot(),
            now_s=now_s,
            runtime_clearance_certificates=(
                self.runtime_clearance_certificates()
            ),
            slot_sensor_anchors=self.slot_sensor_anchors(now_s=now_s),
            verified_slot_arrival_certificates=(
                self.verified_slot_arrival_certificates(now_s=now_s)
            ),
        )

    def observe(self, *, timestamp: float | None = None) -> ObservedState:
        deadline = self.monotonic() + self.builder.config.observation_wait_s
        last_error = 'live visual observation is unavailable'
        while self.monotonic() <= deadline:
            try:
                return self._build_current_state()
            except Exception as exc:  # noqa: BLE001 - bounded retry boundary
                last_error = str(exc)
            with self._condition:
                remaining = max(0.0, deadline - self.monotonic())
                self._condition.wait(timeout=min(remaining, 0.1))
        raise TaskExecutionStateError(last_error)

    def observe_fresh_after(
        self,
        state_id: str,
        *,
        timestamp: float | None = None,
    ) -> ObservedState:
        """Wait for a valid observation whose visual state ID has advanced.

        A PDDL input-consistency failure is recoverable only if another visual
        inference result is actually available.  Rebuilding the same cached
        snapshot several times creates fake retries and can exhaust recovery
        in a few milliseconds.  Supervisor/status updates may wake this wait,
        but only a different accepted visual ``state_id`` satisfies it.
        """

        previous_state_id = str(state_id or '').strip()
        if not previous_state_id:
            return self.observe(timestamp=timestamp)
        deadline = self.monotonic() + self.builder.config.observation_wait_s
        last_error = (
            'accepted visual observation did not advance beyond '
            f'{previous_state_id}'
        )
        while self.monotonic() <= deadline:
            try:
                state = self._build_current_state()
                if state.state_id != previous_state_id:
                    return state
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
        self._sensor_receive_s = {'left': -1.0, 'right': -1.0}

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
            if not isinstance(reading, dict) or reading.get('active') is not True:
                continue
            sanitized.append({
                'name': str(reading.get('name') or '').strip(),
                'shuttle': str(reading.get('shuttle') or '').strip(),
            })
        with self._condition:
            self._sensor_feedback[rail_side] = sanitized
            self._sensor_sequence[rail_side] += 1
            self._sensor_receive_s[rail_side] = self.monotonic()
            self._condition.notify_all()

    def publish_command(self, command: dict[str, Any]) -> None:
        action = str(command.get('action') or '').strip().casefold()
        command_name = str(command.get('command') or '').strip().upper()
        shuttle = str(
            command.get('shuttle')
            or command.get('shuttle_id')
            or command.get('name')
            or ''
        ).strip()
        if action == 'shuttle' and command_name == 'ON' and shuttle:
            # A bounded-stop certificate proves only the completed motion that
            # created it. Any later ON command invalidates that proof before
            # the command leaves this runtime boundary.
            self.provider.clear_runtime_clearance_certificate(
                shuttle,
                side=str(command.get('side') or '').strip() or None,
            )
        if (
            action == 'shuttle'
            and command_name in {'ON', 'RESET', 'REMOVE', 'ADD_MOVING'}
            and shuttle
        ):
            # The exact-slot proof is valid only while the same shuttle stays
            # stopped on the same active DZI. Invalidate before any command
            # that can move, reset, or remove it; never wait for a later
            # visual disagreement to discover stale categorical state.
            self.provider.invalidate_verified_slot_arrival_for_motion(
                shuttle,
                side=str(command.get('side') or '').strip() or None,
            )
        self.publish_callback(copy.deepcopy(command))

    def supervisor_decision_count(self) -> int:
        with self._condition:
            metrics = _nested_dict(
                self._supervisor_status,
                'safety_decoder',
                'metrics',
            )
            return int(metrics.get('total_proposed_actions') or 0)

    def supervisor_state_count(self) -> int:
        """Return the received controller/supervisor snapshot sequence."""

        with self._condition:
            return self._supervisor_sequence

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
                    if supervisor_decision_is_terminal(decision):
                        return copy.deepcopy(decision)
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
        # The ON command has already crossed publish_command(), which
        # atomically advanced this generation.  Any later motion-capable
        # command advances it again and makes this in-flight proof stale,
        # even before controller telemetry changes from DISABLED to MOVING.
        arrival_motion_epoch = self.provider.verified_slot_motion_epoch(
            spec.short_id,
            side=rail_side,
        )
        deadline = self.monotonic() + max(float(timeout_s), 0.0)
        confirmed = 0
        last_sequence = -1
        last_sensor_receive_s = -1.0
        while self.monotonic() <= deadline:
            with self._condition:
                sequence = self._sensor_sequence[rail_side]
                readings = copy.deepcopy(self._sensor_feedback[rail_side])
                sensor_receive_s = self._sensor_receive_s[rail_side]
                if sequence == last_sequence:
                    self._condition.wait(
                        timeout=min(0.1, max(0.0, deadline - self.monotonic()))
                    )
                    continue
                last_sequence = sequence
            if (
                last_sensor_receive_s >= 0.0
                and sensor_receive_s - last_sensor_receive_s
                > self.provider.builder.config.slot_sensor_state_timeout_s
            ):
                confirmed = 0
            last_sensor_receive_s = sensor_receive_s
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
            certificate = {
                'identity': spec.short_id,
                'shuttle': spec.gazebo_entity_name,
                'side': rail_side,
                'slot': str(target_slot),
                'sensor': expected_sensor.upper(),
                'matched_by': 'deterministic_slot_sensor',
                'proof_mode': 'supervised_command_arrival',
                'motion_epoch': arrival_motion_epoch,
                'sensor_identity_confirmed': True,
                'sensor_confirmation_frames': confirmed,
                'sensor_sequence': last_sequence,
                'controller_stop_confirmed': True,
                'controller_mode': str(
                    stop_result.get('mode') or ''
                ).strip().upper(),
                'controller_target_slot_confirmed': bool(
                    stop_result.get('controller_target_slot_confirmed')
                ),
                'reached_target_slot': str(
                    stop_result.get('reached_target_slot') or ''
                ).strip(),
                'supervisor_sequence': int(
                    stop_result.get('supervisor_sequence') or 0
                ),
                'model_prediction_replaced': False,
                'controller_position_fields_used_for_localization': False,
            }
            try:
                self.provider.set_verified_slot_arrival_certificate(certificate)
            except TaskExecutionStateError as exc:
                return {
                    'arrived': False,
                    'reason': (
                        'slot arrival proof became stale before registration: '
                        f'{exc}'
                    ),
                    'matched_by': 'deterministic_slot_sensor',
                    'controller_position_fields_used_for_localization': False,
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
                'verified_slot_arrival_certificate': copy.deepcopy(
                    certificate
                ),
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
        motion_origin_s_m: float | None = None,
        timeout_s: float,
    ) -> dict[str, Any]:
        """Stop an interior relocation with an identity-bearing sensor guard.

        The accepted visual state remains the planner-localization source. The
        binary branch-entry sensor plus a bounded travel time owns this safety
        stop, so neither a false positive nor a false negative visual block can
        move the shuttle into FALLING. A disagreeing raw visual block/position
        is preserved for audit and is never rewritten by this execution-effect
        proof. Controller mode is used only for FALLING detection and fresh
        explicit DISABLED confirmation after OFF; controller segment/s fields
        are never read.
        """

        rail_side = _side(side)
        spec = normalize_shuttle_ref(shuttle, side=rail_side)
        if spec is None or spec.side != rail_side:
            return {'arrived': False, 'reason': f'unknown shuttle {shuttle!r}'}
        wanted_segment = str(target_segment or '').strip().upper()
        wanted_sensor = str(entry_sensor or '').strip().upper()
        tolerance = max(float(tolerance_m), 0.0)
        clearance_delay_s = max(float(minimum_clearance_delay_s), 0.0)
        started_at = self.monotonic()
        deadline = started_at + max(float(timeout_s), 0.0)
        advance_from_certified_origin = motion_origin_s_m is not None
        certified_origin_s_m: float | None = None
        if advance_from_certified_origin:
            try:
                certified_origin_s_m = float(motion_origin_s_m)
            except (TypeError, ValueError):
                return {
                    'arrived': False,
                    'reason': 'certified interior motion origin is invalid',
                }
            if (
                not math.isfinite(certified_origin_s_m)
                or float(target_s_m) <= certified_origin_s_m
                or clearance_delay_s <= 0.0
            ):
                return {
                    'arrived': False,
                    'reason': (
                        'certified interior advance requires a finite forward '
                        'origin and positive bounded travel time'
                    ),
                }
        last_visual_sequence = -1
        last_sensor_sequence = -1
        last_position: dict[str, Any] = {}
        entry_detected_at: float | None = (
            started_at if advance_from_certified_origin else None
        )
        entry_confirmation: dict[str, Any] = (
            {
                'matched': True,
                'source': 'validated_runtime_clearance_origin_certificate',
            }
            if advance_from_certified_origin
            else {}
        )

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
            previous_state_sequence = self.supervisor_state_count()
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
                    'motion_origin_s_m': certified_origin_s_m,
                    'advance_from_certified_interior_origin': (
                        advance_from_certified_origin
                    ),
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
                after_supervisor_sequence=previous_state_sequence,
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
                stop_trigger = (
                    'certified_interior_origin_plus_bounded_travel_time'
                    if advance_from_certified_origin
                    else 'interior_entry_sensor_plus_bounded_travel_time'
                )

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
            'interior_advance_origin_certified': (
                advance_from_certified_origin
            ),
            'motion_origin_s_m': certified_origin_s_m,
            'bounded_motion_distance_m': (
                float(target_s_m) - certified_origin_s_m
                if certified_origin_s_m is not None
                else float(target_s_m)
            ),
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
        previous_state_sequence = self.supervisor_state_count()
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
            after_supervisor_sequence=previous_state_sequence,
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
        after_supervisor_sequence: int | None = None,
    ) -> dict[str, Any]:
        spec = normalize_shuttle_ref(shuttle, side=side)
        if spec is None:
            return {'ready': False, 'reason': f'unknown shuttle {shuttle!r}'}
        deadline = self.monotonic() + max(float(timeout_s), 0.0)
        while self.monotonic() <= deadline:
            with self._condition:
                if (
                    after_supervisor_sequence is not None
                    and self._supervisor_sequence <= after_supervisor_sequence
                ):
                    self._condition.wait(
                        timeout=min(
                            0.1,
                            max(0.0, deadline - self.monotonic()),
                        )
                    )
                    continue
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
                    mode == CONTROLLER_DISABLED_MODE
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
                        'supervisor_sequence': self._supervisor_sequence,
                        'localization_source': 'not_used',
                        'reached_target_slot': reached_target_slot,
                    }
                self._condition.wait(
                    timeout=min(0.1, max(0.0, deadline - self.monotonic()))
                )
        return {
            'ready': False,
            'reason': (
                f'timeout confirming fresh controller DISABLED state for {spec.short_id}'
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
    """Ground shuttle selection from accepted visual facts.

    This is deterministic goal grounding, not action planning. Location and
    payload eligibility come from the learned visual state; PlanSys2 still
    owns route/action planning after an explicit candidate is selected.
    """

    constraints = dict(task_goal.constraints or {})
    goal_type = str(constraints.get('goal_type') or '').strip().lower()
    facts = {
        (fact.subject, fact.predicate): fact
        for fact in observed_state.fused_planner_state
        if fact.status == 'known'
    }
    if goal_type == 'inspection':
        return _ground_inspection_task_goal(
            task_goal,
            constraints=constraints,
            facts=facts,
        )
    if goal_type != 'transport':
        raise TaskExecutionStateError('only transport TaskGoals can be grounded')
    side = _side(constraints.get('side'))
    selection, payload_filter = _normalize_live_selection_contract(
        constraints,
    )
    existing = constraints.get('target_shuttle')
    if existing:
        spec = normalize_shuttle_ref(existing, side=side)
        if spec is None or spec.side != side:
            raise TaskExecutionStateError(
                f'invalid explicit target shuttle:{existing!r}'
            )
        present = facts.get((spec.gazebo_entity_name, 'present'))
        if present is None or not bool(present.value):
            raise TaskExecutionStateError(
                f'explicit target shuttle is absent or presence is unknown:'
                f'{spec.short_id}'
            )
        loaded = facts.get((spec.gazebo_entity_name, 'loaded'))
        if payload_filter in {'loaded', 'empty'}:
            if loaded is None:
                raise TaskExecutionStateError(
                    f'explicit target shuttle payload is unknown:{spec.short_id}'
                )
            if payload_filter == 'loaded' and not bool(loaded.value):
                raise TaskExecutionStateError(
                    f'explicit target shuttle is not loaded:{spec.short_id}'
                )
            if payload_filter == 'empty' and bool(loaded.value):
                raise TaskExecutionStateError(
                    f'explicit target shuttle is not empty:{spec.short_id}'
                )
        constraints['target_shuttle'] = spec.gazebo_entity_name
        constraints['selection_strategy'] = 'explicit'
        constraints['shuttle_selection'] = 'explicit'
        _ground_station_destination_for_selected_shuttle(
            constraints,
            facts=facts,
            selected_spec=spec,
            side=side,
        )
        return TaskGoal(
            goal_id=task_goal.goal_id,
            description=task_goal.description,
            source=task_goal.source,
            timestamp=task_goal.timestamp,
            confidence=task_goal.confidence,
            constraints=constraints,
        )

    target_slot = str(constraints.get('target_slot') or '')
    target_station = _station_name(constraints.get('target_station'), side=side)
    station_slots = _station_slots(side=side, station=target_station)
    if (
        selection == 'nearest'
        and target_slot not in {'1', '2', '3', '4'}
        and not station_slots
    ):
        raise TaskExecutionStateError(
            'nearest selection requires target slot 1, 2, 3, or 4, or a '
            'valid target station on the selected rail'
        )
    candidates = []
    for spec in all_shuttle_specs():
        if spec.side != side:
            continue
        present = facts.get((spec.gazebo_entity_name, 'present'))
        if present is None or not bool(present.value):
            continue
        loaded = facts.get((spec.gazebo_entity_name, 'loaded'))
        if not _payload_eligible(loaded, payload_filter=payload_filter):
            continue
        slot_fact = facts.get((spec.gazebo_entity_name, 'location_slot'))
        slot = _slot_number(slot_fact.value if slot_fact is not None else '')
        selected_station_slot = ''
        occupancy_penalty = 0.0
        already_satisfied = False
        if target_slot:
            distance = _visual_route_distance_m(
                facts,
                spec=spec,
                target_slot=target_slot,
            )
            if distance == float('inf'):
                continue
            already_satisfied = slot == target_slot
            occupancy_penalty = _slot_occupancy_penalty(
                facts,
                side=side,
                slot=target_slot,
                spec=spec,
            )
            if occupancy_penalty == float('inf'):
                continue
        elif station_slots:
            route_candidates = [
                (
                    _slot_occupancy_penalty(
                        facts,
                        side=side,
                        slot=station_slot,
                        spec=spec,
                    ),
                    _visual_route_distance_m(
                        facts,
                        spec=spec,
                        target_slot=station_slot,
                    ),
                    station_slot,
                )
                for station_slot in station_slots
            ]
            route_candidates = [
                item
                for item in route_candidates
                if item[0] != float('inf') and item[1] != float('inf')
            ]
            if not route_candidates:
                continue
            _occupancy_penalty, distance, selected_station_slot = min(
                route_candidates,
                key=lambda item: (item[0], item[1], int(item[2])),
            )
            occupancy_penalty = _occupancy_penalty
            already_satisfied = slot == selected_station_slot
        else:
            distance = 0
        # ``any`` is deterministic but not arbitrary: do no work when an
        # eligible shuttle already satisfies the exact destination, otherwise
        # prefer the least-cost feasible visual/topology route. ``nearest``
        # uses the same authoritative route distance without requiring a
        # derived exact-slot label at the source.
        satisfied_rank = 0 if selection == 'any' and already_satisfied else 1
        candidates.append((
            satisfied_rank,
            occupancy_penalty,
            distance,
            spec.short_id,
            selected_station_slot,
            spec,
        ))
    if not candidates:
        raise TaskExecutionStateError(
            f'no visual candidate for selection={selection}, '
            f'payload_filter={payload_filter}, side={side}'
        )
    candidates.sort(key=lambda item: item[:4])
    selected_station_slot = candidates[0][4]
    selected = candidates[0][5]
    constraints['target_shuttle'] = selected.gazebo_entity_name
    constraints['selection_strategy'] = 'explicit'
    constraints['shuttle_selection'] = 'explicit'
    if selected_station_slot:
        constraints['target_kind'] = 'slot'
        constraints['target_slot'] = selected_station_slot
        constraints['target_station'] = target_station
    else:
        _ground_station_destination_for_selected_shuttle(
            constraints,
            facts=facts,
            selected_spec=selected,
            side=side,
        )
    return TaskGoal(
        goal_id=task_goal.goal_id,
        description=task_goal.description,
        source=task_goal.source,
        timestamp=task_goal.timestamp,
        confidence=task_goal.confidence,
        constraints=constraints,
    )


@dataclass(frozen=True)
class StableTaskGoalGrounding:
    task_goal: TaskGoal
    observed_state: ObservedState
    payload_confirmation: dict[str, Any]


def ground_transport_task_goal_stably(
    task_goal: TaskGoal,
    initial_state: ObservedState,
    *,
    observe_fresh_after: Callable[[str], ObservedState],
    confirmation_frames: int = 5,
    max_observations: int = 15,
) -> StableTaskGoalGrounding:
    """Require consecutive fresh visual agreement for payload selection."""

    constraints = dict(task_goal.constraints or {})
    if str(constraints.get('goal_type') or '').casefold() != 'transport':
        return StableTaskGoalGrounding(
            task_goal=ground_transport_task_goal(task_goal, initial_state),
            observed_state=initial_state,
            payload_confirmation={'required': False},
        )
    _selection, payload_filter = _normalize_live_selection_contract(
        constraints,
    )
    if payload_filter not in {'loaded', 'empty'}:
        return StableTaskGoalGrounding(
            task_goal=ground_transport_task_goal(task_goal, initial_state),
            observed_state=initial_state,
            payload_confirmation={'required': False},
        )

    required_frames = max(
        int(confirmation_frames),
        MIN_PAYLOAD_CONFIRMATION_FRAMES,
    )
    observation_limit = max(int(max_observations), required_frames)
    current = initial_state
    previous_state_id = ''
    seen_state_ids: set[str] = set()
    streak_target = ''
    streak_state_ids: list[str] = []
    latest_grounded: TaskGoal | None = None
    last_error = 'no accepted visual payload candidate'

    for observation_index in range(observation_limit):
        state_id = str(current.state_id or '').strip()
        if not state_id or state_id in seen_state_ids:
            raise TaskExecutionStateError(
                'payload grounding requires distinct fresh visual state IDs'
            )
        seen_state_ids.add(state_id)
        try:
            grounded = ground_transport_task_goal(task_goal, current)
        except TaskExecutionStateError as exc:
            last_error = str(exc)
            streak_target = ''
            streak_state_ids = []
            latest_grounded = None
        else:
            spec = normalize_shuttle_ref(
                grounded.constraints.get('target_shuttle'),
                side=constraints.get('side'),
            )
            if spec is None:
                raise TaskExecutionStateError(
                    'payload grounding produced no authoritative identity'
                )
            target = spec.shuttle_id
            if target != streak_target:
                streak_target = target
                streak_state_ids = []
            streak_state_ids.append(state_id)
            latest_grounded = grounded
            if len(streak_state_ids) >= required_frames:
                return StableTaskGoalGrounding(
                    task_goal=grounded,
                    observed_state=current,
                    payload_confirmation=create_visual_payload_confirmation(
                        selected_shuttle=target,
                        payload_filter=payload_filter,
                        state_ids=list(
                            streak_state_ids[-required_frames:]
                        ),
                        observations_examined=observation_index + 1,
                    ),
                )
        if observation_index + 1 >= observation_limit:
            break
        previous_state_id = state_id
        current = observe_fresh_after(previous_state_id)

    candidate = (
        str(latest_grounded.constraints.get('target_shuttle') or '')
        if latest_grounded is not None
        else 'none'
    )
    raise TaskExecutionStateError(
        'visual payload selection did not reach consecutive consensus:'
        f'filter={payload_filter},candidate={candidate},'
        f'confirmed={len(streak_state_ids)}/{required_frames},'
        f'observations={observation_limit},last_error={last_error}'
    )


def build_runtime_payload_grounding(
    task_goal: TaskGoal,
    observed_state: ObservedState,
    *,
    payload_confirmation: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Create trusted selection-time evidence outside the TaskGoal contract."""

    constraints = dict(task_goal.constraints or {})
    if str(constraints.get('goal_type') or '').casefold() != 'transport':
        return {}
    side = _side(constraints.get('side'))
    _selection, payload_filter = _normalize_live_selection_contract(
        constraints,
    )
    if payload_filter not in {'loaded', 'empty'}:
        return {}
    spec = normalize_shuttle_ref(
        constraints.get('target_shuttle'),
        side=side,
    )
    if spec is None or spec.side != side:
        raise TaskExecutionStateError(
            'payload grounding requires an explicit authoritative target'
        )
    facts = {
        (fact.subject, fact.predicate): fact
        for fact in observed_state.fused_planner_state
        if fact.status == 'known'
    }
    loaded = facts.get((spec.gazebo_entity_name, 'loaded'))
    if loaded is None:
        raise TaskExecutionStateError(
            f'payload grounding source is unknown:{spec.short_id}'
        )
    predicted = 'loaded' if bool(loaded.value) else 'empty'
    if predicted != payload_filter:
        raise TaskExecutionStateError(
            f'payload grounding source does not satisfy {payload_filter}:'
            f'{spec.short_id}:{predicted}'
        )
    try:
        return create_runtime_payload_grounding(
            selected_shuttle=spec.shuttle_id,
            payload_filter=payload_filter,
            initial_visual_prediction=predicted,
            source_state_id=str(observed_state.state_id),
            confirmation=dict(payload_confirmation or {}),
        )
    except ValueError as exc:
        raise TaskExecutionStateError(str(exc)) from exc


def _normalize_live_selection_contract(
    constraints: dict[str, Any],
) -> tuple[str, str]:
    """Canonicalize current and legacy selection fields before grounding."""

    raw_payload = constraints.get('payload_filter')
    payload_is_explicit = raw_payload is not None and str(raw_payload).strip() != ''
    if payload_is_explicit:
        payload_filter = str(raw_payload).strip().casefold()
    else:
        legacy_payload = constraints.get('payload_required')
        legacy_selection = str(
            constraints.get('shuttle_selection') or ''
        ).strip().casefold()
        if legacy_selection in {'loaded', 'empty'}:
            payload_filter = legacy_selection
        elif isinstance(legacy_payload, bool):
            payload_filter = 'loaded' if legacy_payload else 'empty'
        else:
            legacy_text = str(
                legacy_payload if legacy_payload is not None else ''
            ).strip().casefold()
            payload_filter = {
                'true': 'loaded',
                'yes': 'loaded',
                '1': 'loaded',
                'false': 'empty',
                'no': 'empty',
                '0': 'empty',
            }.get(legacy_text, legacy_text or 'any')
    if payload_filter not in {'loaded', 'empty', 'any'}:
        raise TaskExecutionStateError(
            f'invalid visual payload filter:{payload_filter!r}'
        )

    raw_selection = str(
        constraints.get('selection_strategy')
        or constraints.get('shuttle_selection')
        or 'any'
    ).strip().casefold()
    selection = 'any' if raw_selection in {'loaded', 'empty'} else raw_selection
    if constraints.get('target_shuttle'):
        selection = 'explicit'
    if selection not in {'explicit', 'nearest', 'any'}:
        raise TaskExecutionStateError(
            f'invalid shuttle selection strategy:{selection!r}'
        )

    constraints['payload_filter'] = payload_filter
    constraints['selection_strategy'] = selection
    constraints['shuttle_selection'] = selection
    return selection, payload_filter


def _payload_eligible(
    loaded_fact: ObservedFact | None,
    *,
    payload_filter: str,
) -> bool:
    if payload_filter == 'any':
        return True
    if loaded_fact is None:
        return False
    loaded = bool(loaded_fact.value)
    return loaded if payload_filter == 'loaded' else not loaded


def _ground_inspection_task_goal(
    task_goal: TaskGoal,
    *,
    constraints: dict[str, Any],
    facts: dict[tuple[str, str], ObservedFact],
) -> TaskGoal:
    """Resolve shuttle inspection subjects against the fresh presence state."""

    target_kind = str(constraints.get('target_kind') or '').strip().casefold()
    raw_subject = constraints.get('target_shuttle') or constraints.get(
        'inspection_subject'
    )
    explicit_spec = normalize_shuttle_ref(raw_subject)
    needs_selection = target_kind == 'shuttle_selection'
    if explicit_spec is None and not needs_selection:
        # Room, rail, slot and station inspection subjects are already grounded
        # and do not depend on live shuttle presence.
        return task_goal

    if explicit_spec is not None:
        _selection, payload_filter = _normalize_live_selection_contract(
            constraints,
        )
        supplied_side = str(constraints.get('side') or '').strip().casefold()
        if supplied_side and _side(supplied_side) != explicit_spec.side:
            raise TaskExecutionStateError(
                f'inspection shuttle side conflicts with authoritative identity:'
                f'{explicit_spec.short_id}'
            )
        present = facts.get((explicit_spec.gazebo_entity_name, 'present'))
        if present is None or not bool(present.value):
            raise TaskExecutionStateError(
                f'explicit inspection shuttle is absent or presence is unknown:'
                f'{explicit_spec.short_id}'
            )
        loaded = facts.get((explicit_spec.gazebo_entity_name, 'loaded'))
        if not _payload_eligible(loaded, payload_filter=payload_filter):
            state = (
                'unknown'
                if loaded is None
                else ('loaded' if bool(loaded.value) else 'empty')
            )
            raise TaskExecutionStateError(
                f'explicit inspection shuttle payload does not satisfy '
                f'{payload_filter}:{explicit_spec.short_id}:{state}'
            )
        constraints['side'] = explicit_spec.side
        constraints['target_kind'] = 'shuttle'
        constraints['target_shuttle'] = explicit_spec.gazebo_entity_name
        constraints['inspection_subject'] = explicit_spec.gazebo_entity_name
        constraints['payload_filter'] = payload_filter
        constraints['selection_strategy'] = 'explicit'
        constraints['shuttle_selection'] = 'explicit'
        return _copy_task_goal(task_goal, constraints=constraints)

    side = _side(constraints.get('side'))
    selection, payload_filter = _normalize_live_selection_contract(constraints)
    if selection == 'explicit':
        raise TaskExecutionStateError(
            'explicit shuttle inspection requires a grounded shuttle identity'
        )
    target_slot = str(constraints.get('target_slot') or '')
    target_station = _station_name(constraints.get('target_station'), side=side)
    if selection == 'nearest' and not (
        target_slot in {'1', '2', '3', '4'}
        or _station_slots(side=side, station=target_station)
    ):
        raise TaskExecutionStateError(
            'nearest shuttle inspection requires an exact slot or station '
            'reference; ask the user to clarify the reference'
        )

    synthetic_constraints = dict(constraints)
    synthetic_constraints['goal_type'] = 'transport'
    synthetic = TaskGoal(
        goal_id=task_goal.goal_id,
        description=task_goal.description,
        source=task_goal.source,
        timestamp=task_goal.timestamp,
        confidence=task_goal.confidence,
        constraints=synthetic_constraints,
    )
    grounded = ground_transport_task_goal(
        synthetic,
        ObservedState(
            state_id='inspection-grounding-view',
            timestamp=task_goal.timestamp,
            fused_planner_state=list(facts.values()),
        ),
    )
    selected = str(grounded.constraints['target_shuttle'])
    constraints.update({
        'target_kind': 'shuttle',
        'target_shuttle': selected,
        'inspection_subject': selected,
        'payload_filter': grounded.constraints['payload_filter'],
        'selection_strategy': 'explicit',
        'shuttle_selection': 'explicit',
    })
    return _copy_task_goal(task_goal, constraints=constraints)


def _copy_task_goal(
    task_goal: TaskGoal,
    *,
    constraints: dict[str, Any],
) -> TaskGoal:
    return TaskGoal(
        goal_id=task_goal.goal_id,
        description=task_goal.description,
        source=task_goal.source,
        timestamp=task_goal.timestamp,
        confidence=task_goal.confidence,
        constraints=constraints,
    )


def _ground_station_destination_for_selected_shuttle(
    constraints: dict[str, Any],
    *,
    facts: dict[tuple[str, str], ObservedFact],
    selected_spec: Any,
    side: str,
) -> None:
    """Convert a station-only request into a deterministic sensor-backed slot."""

    if str(constraints.get('target_slot') or '') in {'1', '2', '3', '4'}:
        return
    station = _station_name(constraints.get('target_station'), side=side)
    slots = _station_slots(side=side, station=station)
    if not slots:
        return
    candidates = [
        (
            _slot_occupancy_penalty(
                facts,
                side=side,
                slot=slot,
                spec=selected_spec,
            ),
            _visual_route_distance_m(
                facts,
                spec=selected_spec,
                target_slot=slot,
            ),
            slot,
        )
        for slot in slots
    ]
    candidates = [
        item
        for item in candidates
        if item[0] != float('inf') and item[1] != float('inf')
    ]
    if not candidates:
        raise TaskExecutionStateError(
            f'no reachable sensor-backed slot at target station:{side}:{station}'
        )
    _occupancy_penalty, _distance, slot = min(
        candidates,
        key=lambda item: (item[0], item[1], int(item[2])),
    )
    constraints['target_kind'] = 'slot'
    constraints['target_slot'] = slot
    constraints['target_station'] = station


def _station_name(value: Any, *, side: str) -> str:
    text = str(value or '').strip().lower().replace(':', '_')
    if text.startswith(f'{side}_'):
        text = text[len(side) + 1:]
    allowed = {
        station
        for (station_side, _slot), station in SLOT_STATION_BY_SIDE_AND_SLOT.items()
        if station_side == side
    }
    return text if text in allowed else ''


def _station_slots(*, side: str, station: str) -> tuple[str, ...]:
    if not station:
        return ()
    return tuple(
        slot
        for (slot_side, slot), slot_station in sorted(
            SLOT_STATION_BY_SIDE_AND_SLOT.items()
        )
        if slot_side == side and slot_station == station
    )


def _slot_occupancy_penalty(
    facts: dict[tuple[str, str], ObservedFact],
    *,
    side: str,
    slot: str,
    spec: Any,
) -> float:
    occupancy = facts.get((f'{side}:slot:{slot}', 'occupancy'))
    if occupancy is None or not isinstance(occupancy.value, dict):
        return float('inf')
    if not bool(occupancy.value.get('occupied')):
        return 0.0
    occupant = normalize_shuttle_ref(
        occupancy.value.get('shuttle') or occupancy.value.get('occupant'),
        side=side,
    )
    if occupant is None:
        return float('inf')
    return 0.0 if occupant.short_id == spec.short_id else 1.0


def _visual_route_distance_m(
    facts: dict[tuple[str, str], ObservedFact],
    *,
    spec: Any,
    target_slot: str,
) -> float:
    position = facts.get((spec.gazebo_entity_name, 'rail_position'))
    try:
        topology = load_rail_topology(
            RAIL_NETWORK_PATH_BY_SIDE[spec.side],
            RAIL_DEVICES_PATH_BY_SIDE[spec.side],
            side=spec.side,
        )
        if position is not None and isinstance(position.value, dict):
            raw = position.value
            segment = str(raw.get('segment') or '').strip().upper()
            if spec.side == 'left':
                segment = LEFT_PUBLIC_SEGMENT_NAME_MAP.get(segment, segment)
            source_ratio = raw.get('s_ratio')
        else:
            # Static scenario states and deterministic slot-sensor anchors may
            # legitimately omit a free-form rail_position. An exact accepted
            # slot is still a safe topology source, but never substitute a
            # guessed slot for a segment-only visual position.
            slot_fact = facts.get((spec.gazebo_entity_name, 'location_slot'))
            source_slot = _slot_number(
                slot_fact.value if slot_fact is not None else ''
            )
            if not source_slot:
                return float('inf')
            source = topology.slots[source_slot]
            segment = source.segment
            source_ratio = source.s_ratio
        routes = route_candidates_from_position_to_slot(
            topology,
            segment,
            source_ratio,
            target_slot,
        )
        if not routes:
            return float('inf')
        lengths = rail_segment_lengths(spec.side)
        return min(
            sum(
                abs(float(block.end_s_ratio) - float(block.start_s_ratio))
                * float(lengths[block.segment])
                for block in route.blocks
            )
            for route in routes
        )
    except (KeyError, TypeError, ValueError):
        return float('inf')


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
    return supervisor_decision_accepted(decision)


def _slot_number(value: Any) -> str:
    text = str(value or '').strip()
    if text and text[-1] in {'1', '2', '3', '4'}:
        return text[-1]
    return ''
