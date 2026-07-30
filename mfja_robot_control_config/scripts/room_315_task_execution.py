#!/usr/bin/env python3
"""ROS-independent state and transport core for Room 315 task execution.

The visual observation owns shuttle location and payload facts.  Controller
state is admitted only for presence (upstream of this module), switch/stopper
state, safety decisions, and confirmation that an OFF command took effect.
In particular, supervisor ShuttleState position fields are never read here.
"""

from __future__ import annotations

import copy
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
    planning_slot_tolerance_ratio: float = 0.12
    target_arrival_tolerance_ratio: float = 0.05
    position_consistency_tolerance_m: float = 0.08
    observation_wait_s: float = 2.0
    external_obstacles_disabled: bool = True

    def __post_init__(self) -> None:
        for name in (
            'observation_timeout_s',
            'supervisor_status_timeout_s',
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
    ) -> ObservedState:
        observation = copy.deepcopy(snapshot.observation)
        supervisor = copy.deepcopy(snapshot.supervisor_status)
        self._validate_freshness(snapshot, now_s=now_s)
        self._validate_observation_envelope(observation)
        self._validate_supervisor(supervisor)

        timestamp_s = float(observation.get('timestamp_s') or now_s)
        visual_facts: list[ObservedFact] = []
        trusted_facts: list[ObservedFact] = []
        slot_occupants = {
            side: {slot: '' for slot in ('1', '2', '3', '4')}
            for side in ('left', 'right')
        }
        shuttles_by_identity = self._shuttles_by_identity(observation)

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
            slot = self.slot_matcher.slot_for_shuttle(item)
            if slot:
                previous = slot_occupants[spec.side][slot]
                if previous:
                    raise TaskExecutionStateError(
                        f'visual slot conflict:{spec.side}:slot:{slot}:'
                        f'{previous},{spec.short_id}'
                    )
                slot_occupants[spec.side][slot] = spec.gazebo_entity_name

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
            if slot:
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
                        value=str(switches[device]),
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
                        value=str(stoppers[device]),
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

    def ready(self) -> tuple[bool, str]:
        try:
            self.builder.build(self.snapshot(), now_s=self.monotonic())
        except Exception as exc:  # noqa: BLE001 - readiness explanation
            return False, str(exc)
        return True, ''

    def observe(self, *, timestamp: float | None = None) -> ObservedState:
        deadline = self.monotonic() + self.builder.config.observation_wait_s
        last_error = 'live visual observation is unavailable'
        while self.monotonic() <= deadline:
            try:
                return self.builder.build(
                    self.snapshot(),
                    now_s=self.monotonic(),
                )
            except Exception as exc:  # noqa: BLE001 - bounded retry boundary
                last_error = str(exc)
            with self._condition:
                remaining = max(0.0, deadline - self.monotonic())
                self._condition.wait(timeout=min(remaining, 0.1))
        raise TaskExecutionStateError(last_error)


class VisualSupervisorTransport(ScenarioTransport):
    """Supervisor transport with visual-only target-arrival localization."""

    def __init__(
        self,
        *,
        provider: LatestVisualObservedStateProvider,
        publish_callback: Callable[[dict[str, Any]], None],
        arrival_confirmation_frames: int = 3,
        controller_stop_timeout_s: float = 3.0,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.provider = provider
        self.publish_callback = publish_callback
        self.arrival_confirmation_frames = max(int(arrival_confirmation_frames), 1)
        self.controller_stop_timeout_s = max(float(controller_stop_timeout_s), 0.1)
        self.monotonic = monotonic
        self._condition = threading.Condition()
        self._supervisor_status: dict[str, Any] = {}
        self._supervisor_sequence = 0
        self._visual_observation: dict[str, Any] = {}
        self._visual_sequence = 0

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
        if not target_slot:
            return {
                'arrived': False,
                'reason': 'visual target arrival requires an explicit target_slot',
            }
        deadline = self.monotonic() + max(float(timeout_s), 0.0)
        confirmed = 0
        last_sequence = -1
        while self.monotonic() <= deadline:
            with self._condition:
                sequence = self._visual_sequence
                observation = copy.deepcopy(self._visual_observation)
                if sequence == last_sequence:
                    self._condition.wait(
                        timeout=min(0.1, max(0.0, deadline - self.monotonic()))
                    )
                    continue
                last_sequence = sequence
            try:
                reached = self.provider.builder.target_reached(
                    observation,
                    shuttle=shuttle,
                    side=side,
                    target_slot=target_slot,
                )
            except TaskExecutionStateError:
                reached = False
            confirmed = confirmed + 1 if reached else 0
            if confirmed < self.arrival_confirmation_frames:
                continue
            stop_result = self._stop_after_visual_arrival(
                side=side,
                shuttle=shuttle,
                target_slot=target_slot,
            )
            if not stop_result['ready']:
                return {
                    'arrived': False,
                    'reason': stop_result['reason'],
                    'matched_by': 'accepted_visual_state',
                }
            return {
                'arrived': True,
                'reason': '',
                'side': side,
                'shuttle': shuttle,
                'target_slot': target_slot,
                'target_station': target_station,
                'matched_by': 'accepted_visual_state',
                'visual_confirmation_frames': confirmed,
                'controller_stop_confirmed': True,
            }
        return {
            'arrived': False,
            'reason': (
                f'timeout waiting for accepted visual localization of '
                f'{shuttle} at {side} slot {target_slot}'
            ),
            'matched_by': 'accepted_visual_state',
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

    def _stop_after_visual_arrival(
        self,
        *,
        side: str,
        shuttle: str,
        target_slot: str,
    ) -> dict[str, Any]:
        previous_count = self.supervisor_decision_count()
        self.publish_command({
            'action': 'shuttle',
            'side': side,
            'shuttle': shuttle,
            'command': 'OFF',
            'closed_loop_executive': {
                'mode': 'visual_target_arrival_stop',
                'target_slot': str(target_slot),
                'location_source': 'accepted_visual_state',
            },
        })
        decision = self.wait_for_supervisor_decision(
            previous_count=previous_count,
            timeout_s=self.controller_stop_timeout_s,
        )
        if decision is None:
            return {'ready': False, 'reason': 'visual arrival OFF command timed out'}
        if not _decision_accepted(decision):
            return {
                'ready': False,
                'reason': (
                    'visual arrival OFF command rejected: '
                    f'{decision.get("reason", "unknown")}'
                ),
            }
        return self._wait_controller_stopped(
            side=side,
            shuttle=shuttle,
            timeout_s=self.controller_stop_timeout_s,
        )

    def _wait_controller_stopped(
        self,
        *,
        side: str,
        shuttle: str,
        timeout_s: float,
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
                if mode in STOPPED_MOTION_VALUES:
                    return {
                        'ready': True,
                        'reason': '',
                        'mode': mode,
                        'confirmation_source': 'controller_execution_feedback',
                        'localization_source': 'not_used',
                    }
                self._condition.wait(
                    timeout=min(0.1, max(0.0, deadline - self.monotonic()))
                )
        return {
            'ready': False,
            'reason': f'timeout confirming OFF for {spec.short_id}',
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
