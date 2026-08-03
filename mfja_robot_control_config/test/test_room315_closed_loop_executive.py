#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_closed_loop_executive import ClosedLoopExecutive
from room_315_closed_loop_executive import ClosedLoopExecutiveConfig
from room_315_closed_loop_executive import PostconditionCheck
from room_315_closed_loop_executive import _canonical_stopper_effect_state
from room_315_closed_loop_executive import _canonical_switch_effect_state
from room_315_closed_loop_executive import _verify_device_state
from room_315_closed_loop_executive import _verify_shuttle_stopped
from room_315_contracts import ObservedState
from room_315_contracts import TaskGoal
from room_315_observed_state_provider import ObservedStateProvider
from room_315_pddl_scenario_generator import BasePlannerBackend
from room_315_pddl_scenario_generator import PddlProblemBuildError
from room_315_pddl_scenario_generator import ScenarioSpec
from room_315_pddl_scenario_generator import ScenarioTransport
from room_315_pddl_scenario_generator import _observed_state_from_scenario_spec
from room_315_pddl_scenario_generator import _planner_fact
from room_315_pddl_scenario_generator import build_pddl_problem_from_observed_state_task_goal
from room_315_pddl_plan_translator import translate_plan
from room_315_rail_defaults import public_rail_segment_lengths
from room_315_task_execution import ground_transport_task_goal


class SequenceObservedStateProvider(ObservedStateProvider):
    def __init__(self, states: list[ObservedState]) -> None:
        self.states = list(states)
        self.calls = 0

    def observe(self, *, timestamp: float | None = None) -> ObservedState:
        state = self.states[min(self.calls, len(self.states) - 1)]
        self.calls += 1
        return state


class PersistedCertificateObservedStateProvider(
    SequenceObservedStateProvider
):
    """Provider fixture for certificates retained across TaskGoal instances."""

    def __init__(
        self,
        states: list[ObservedState],
        certificates: dict[str, dict],
    ) -> None:
        super().__init__(states)
        self.certificates = {
            identity: dict(certificate)
            for identity, certificate in certificates.items()
        }
        self.certificate_reads = 0

    def runtime_clearance_certificates(self) -> dict[str, dict]:
        self.certificate_reads += 1
        return {
            identity: dict(certificate)
            for identity, certificate in self.certificates.items()
        }


class SequencePlanner(BasePlannerBackend):
    def __init__(self, plans: list[list[str]]) -> None:
        self.plans = [list(plan) for plan in plans]
        self.calls = 0
        self.problems = []

    def plan(self, goal_or_problem, *, speed: float) -> list[str]:
        self.problems.append(goal_or_problem)
        plan = self.plans[min(self.calls, len(self.plans) - 1)]
        self.calls += 1
        return list(plan)


class RecordingTransport(ScenarioTransport):
    def __init__(
        self,
        *,
        deterministic_arrival_proof: bool = False,
        clearance_observed_segment: str = '',
        arrival_shuttle: str = 'room315_right_shuttle_1',
        arrival_sensor: str = 'DZI3R',
        arrival_sensor_by_shuttle: dict[str, str] | None = None,
    ) -> None:
        self.commands: list[dict] = []
        self.waits: list[tuple[str, dict]] = []
        self._decision_count = 0
        self._visual_count = 0
        self.deterministic_arrival_proof = deterministic_arrival_proof
        self.clearance_observed_segment = clearance_observed_segment
        self.arrival_shuttle = arrival_shuttle
        self.arrival_sensor = arrival_sensor
        self.arrival_sensor_by_shuttle = dict(
            arrival_sensor_by_shuttle or {}
        )

    def publish_command(self, command: dict) -> None:
        self.commands.append(dict(command))
        if command.get('action') != 'stop_all':
            self._decision_count += 1

    def supervisor_decision_count(self) -> int:
        return self._decision_count

    def wait_for_supervisor_decision(
        self,
        *,
        previous_count: int,
        timeout_s: float,
    ) -> dict | None:
        assert self._decision_count > previous_count
        return {'accepted': True, 'status': 'accepted'}

    def wait_for_switch_state(self, *, side: str, switches: dict, timeout_s: float) -> dict:
        self.waits.append(('switches', {'side': side, 'switches': dict(switches)}))
        return {'ready': True, 'reason': ''}

    def wait_for_stopper_state(self, *, side: str, stoppers: dict, timeout_s: float) -> dict:
        self.waits.append(('stoppers', {'side': side, 'stoppers': dict(stoppers)}))
        return {'ready': True, 'reason': ''}

    def visual_observation_count(self) -> int:
        return self._visual_count

    def wait_for_fresh_visual_observation(
        self,
        *,
        previous_count: int,
        timeout_s: float,
    ) -> dict:
        self._visual_count = max(self._visual_count, previous_count) + 1
        self.waits.append((
            'fresh_visual_observation',
            {'previous_count': previous_count},
        ))
        return {
            'ready': True,
            'reason': '',
            'visual_sequence': self._visual_count,
        }

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
    ) -> dict:
        self.waits.append((
            'target_arrival',
            {
                'side': side,
                'target_sensors': list(target_sensors),
                'shuttle': shuttle,
                'target_slot': target_slot,
                'target_station': target_station,
                'timeout_s': timeout_s,
            },
        ))
        result = {'arrived': True, 'reason': ''}
        if self.deterministic_arrival_proof:
            proof_sensor = self.arrival_sensor_by_shuttle.get(
                shuttle,
                self.arrival_sensor,
            )
            proof_shuttle = (
                shuttle
                if shuttle in self.arrival_sensor_by_shuttle
                else self.arrival_shuttle
            )
            result.update({
                'side': side,
                'shuttle': proof_shuttle,
                'target_slot': target_slot,
                'target_sensor': proof_sensor,
                'matched_by': 'deterministic_slot_sensor',
                'sensor_identity_confirmed': True,
                'controller_stop_confirmed': True,
                'controller_target_slot_confirmed': True,
                'controller_position_fields_used_for_localization': False,
            })
        return result

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
    ) -> dict:
        self.waits.append((
            'visual_position_and_stop',
            {
                'side': side,
                'shuttle': shuttle,
                'target_segment': target_segment,
                'target_s_m': target_s_m,
                'tolerance_m': tolerance_m,
                'entry_sensor': entry_sensor,
                'minimum_clearance_delay_s': minimum_clearance_delay_s,
                'motion_origin_s_m': motion_origin_s_m,
            },
        ))
        self.commands.append({
            'action': 'shuttle',
            'side': side,
            'shuttle': shuttle,
            'command': 'OFF',
            'test_transport_owned_stop': True,
        })
        return {
            'arrived': True,
            'reason': '',
            'matched_by': (
                'certified_interior_origin_plus_bounded_travel_time'
                if motion_origin_s_m is not None
                else 'interior_entry_sensor_plus_bounded_travel_time'
            ),
            'target_segment': target_segment,
            'target_s_m': target_s_m,
            'observed_segment': (
                self.clearance_observed_segment or target_segment
            ),
            'observed_s_m': target_s_m,
            'absolute_error_m': 0.0,
            'entry_sensor': entry_sensor,
            'entry_sensor_identity_confirmed': True,
            'interior_advance_origin_certified': (
                motion_origin_s_m is not None
            ),
            'motion_origin_s_m': motion_origin_s_m,
            'bounded_motion_distance_m': (
                target_s_m - motion_origin_s_m
                if motion_origin_s_m is not None
                else target_s_m
            ),
            'post_stop_visual_frame_received': True,
            'post_stop_visual_confirmation': True,
            'controller_stop_confirmed': True,
            'controller_position_fields_used_for_localization': False,
        }


def _base_state(state_id: str = 'base') -> ObservedState:
    state = _observed_state_from_scenario_spec(
        ScenarioSpec(
            goal_id=state_id,
            side='right',
            shuttle='right_shuttle_1',
            source='yaskawa',
            target='staubli',
            target_slot='3',
            payload_condition='loaded',
            loaded_shuttles=('right_shuttle_1',),
            start_slots_by_shuttle=(('right_shuttle_1', '1'),),
        )
    )
    return replace(state, state_id=state_id)


def _inspection_goal() -> TaskGoal:
    return TaskGoal(
        goal_id='inspect',
        description='Inspect Room 315 state',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'inspection',
            'side': 'right',
            'inspection_subject': 'room315_system',
        },
    )


def _transport_goal() -> TaskGoal:
    return TaskGoal(
        goal_id='move-r1-slot3',
        description='Move R1 to right slot 3',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '3',
            'target_shuttle': 'right_shuttle_1',
            'payload_required': 'loaded',
        },
    )


def _with_fact(state: ObservedState, subject: str, predicate: str, **updates) -> ObservedState:
    facts = [
        replace(fact, **updates)
        if fact.subject == subject and fact.predicate == predicate
        else fact
        for fact in state.fused_planner_state
    ]
    return replace(state, fused_planner_state=facts)


def _state_with_r1_at_slot3(state_id: str = 'r1-slot3') -> ObservedState:
    state = _base_state(state_id)
    facts = []
    for fact in state.fused_planner_state:
        if fact.subject == 'right:slot:1' and fact.predicate == 'occupancy':
            value = {'occupied': False, 'shuttle': None, 'sensor': 'DZI1R'}
            facts.append(replace(fact, value=value))
        elif fact.subject == 'right:slot:3' and fact.predicate == 'occupancy':
            value = {
                'occupied': True,
                'shuttle': 'room315_right_shuttle_1',
                'sensor': 'DZI3R',
            }
            facts.append(replace(fact, value=value))
        elif fact.subject == 'room315_right_shuttle_1' and fact.predicate == 'location_slot':
            facts.append(replace(fact, value='right:slot:3'))
        else:
            facts.append(fact)
    return replace(state, state_id=state_id, fused_planner_state=facts)


def _r4_blocked_state(state_id: str = 'r4-blocked') -> ObservedState:
    return replace(
        _observed_state_from_scenario_spec(
            ScenarioSpec(
                goal_id=state_id,
                side='right',
                shuttle='right_shuttle_4',
                source='staubli',
                target='yaskawa',
                target_slot='2',
                payload_condition='empty',
                start_slots_by_shuttle=(
                    ('right_shuttle_4', '4'),
                    ('right_shuttle_2', '2'),
                ),
            )
        ),
        state_id=state_id,
    )


def _r4_goal() -> TaskGoal:
    return TaskGoal(
        goal_id='move-r4-slot2',
        description='Move R4 to right slot 2',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '2',
            'target_shuttle': 'right_shuttle_4',
            'payload_filter': 'any',
        },
    )


def _any_loaded_right_slot3_goal() -> TaskGoal:
    return TaskGoal(
        goal_id='loaded-r4-slot3-after-r2-topology-route',
        description='Move any loaded right shuttle to slot 3',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '3',
            'selection_strategy': 'any',
            'shuttle_selection': 'loaded',
            'payload_filter': 'loaded',
            'payload_required': True,
        },
    )


def _r2_slot1_goal() -> TaskGoal:
    return TaskGoal(
        goal_id='move-r2-a34i-slot1',
        description='Move R2 from A34I to right slot 1',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '1',
            'target_shuttle': 'right_shuttle_2',
            'payload_filter': 'any',
        },
    )


def _with_r2_interior(
    state: ObservedState,
    state_id: str,
    *,
    visual_s_m: float = 0.7083,
    visual_segment: str = 'A34I',
) -> ObservedState:
    facts = []
    replaced_position = False
    for fact in state.fused_planner_state:
        if fact.subject == 'right:slot:2' and fact.predicate == 'occupancy':
            facts.append(replace(fact, value={'occupied': False, 'shuttle': None}))
        elif (
            fact.subject == 'room315_right_shuttle_2'
            and fact.predicate == 'location_slot'
        ):
            continue
        elif (
            fact.subject == 'room315_right_shuttle_2'
            and fact.predicate == 'rail_position'
        ):
            replaced_position = True
            facts.append(replace(fact, value={
                'side': 'right',
                'segment': visual_segment,
                's_m': visual_s_m,
                's_ratio': visual_s_m / 1.4166,
                'segment_length_m': 1.4166,
                'position_uncertainty_m': 0.0,
            }))
        else:
            facts.append(fact)
    if not replaced_position:
        facts.append(_planner_fact(
            'room315_right_shuttle_2',
            'rail_position',
            {
                'side': 'right',
                'segment': visual_segment,
                's_m': visual_s_m,
                's_ratio': visual_s_m / 1.4166,
                'segment_length_m': 1.4166,
                'position_uncertainty_m': 0.0,
            },
            timestamp=state.timestamp,
            metadata={'source': 'accepted_visual_state'},
        ))
    return replace(state, state_id=state_id, fused_planner_state=facts)


def _with_r4_at_slot2(state: ObservedState, state_id: str) -> ObservedState:
    facts = []
    for fact in state.fused_planner_state:
        if fact.subject == 'right:slot:2' and fact.predicate == 'occupancy':
            facts.append(replace(fact, value={
                'occupied': True,
                'shuttle': 'room315_right_shuttle_4',
            }))
        elif fact.subject == 'right:slot:4' and fact.predicate == 'occupancy':
            facts.append(replace(fact, value={'occupied': False, 'shuttle': None}))
        elif (
            fact.subject == 'room315_right_shuttle_4'
            and fact.predicate == 'location_slot'
        ):
            facts.append(replace(fact, value='right:slot:2'))
        else:
            facts.append(fact)
    return replace(state, state_id=state_id, fused_planner_state=facts)


def _with_clearance_device_mode(
    state: ObservedState,
    state_id: str,
) -> ObservedState:
    facts = []
    for fact in state.fused_planner_state:
        if fact.subject in {
            'right:switch:A3',
            'right:switch:A4',
        } and fact.predicate == 'state':
            facts.append(replace(fact, value='INTERIOR'))
        elif (
            fact.subject == 'right:stopper:A4'
            and fact.predicate == 'state'
        ):
            facts.append(replace(fact, value='closed'))
        else:
            facts.append(fact)
    return replace(state, state_id=state_id, fused_planner_state=facts)


def _persisted_r2_clearance_certificate() -> dict:
    return {
        'identity': 'R2',
        'shuttle': 'right_shuttle_2',
        'side': 'right',
        'target_segment': 'A34I',
        'target_s_m': 0.7083,
        'observed_segment': 'A34E',
        'observed_s_m': 0.7083,
        'entry_sensor': 'DA3IR',
        'matched_by': 'interior_entry_sensor_plus_bounded_travel_time',
        'entry_sensor_identity_confirmed': True,
        'controller_stop_confirmed': True,
        'post_stop_visual_frame_received': True,
        'bounded_commanded_motion_completed': True,
        'clearance_mode_held': True,
        'normal_route_restored': False,
        'model_prediction_replaced': False,
        'controller_position_fields_used_for_localization': False,
    }


def _persisted_r1_clearance_certificate() -> dict:
    certificate = _persisted_r2_clearance_certificate()
    certificate.update({
        'identity': 'R1',
        'shuttle': 'right_shuttle_1',
        'target_s_m': 0.35,
        'observed_segment': 'A34I',
        'observed_s_m': 0.35,
    })
    return certificate


def _loaded_r4_slot3_recovery_state(state_id: str) -> ObservedState:
    """Exact accepted state after R2 returned from A34I to right slot 1."""

    state = _observed_state_from_scenario_spec(ScenarioSpec(
        goal_id=state_id,
        side='right',
        shuttle='right_shuttle_4',
        source='yaskawa',
        target='staubli',
        target_slot='3',
        payload_condition='loaded',
        loaded_shuttles=('right_shuttle_4',),
        start_slots_by_shuttle=(
            ('right_shuttle_1', '4'),
            ('right_shuttle_2', '1'),
            ('right_shuttle_3', '3'),
            ('right_shuttle_4', '2'),
        ),
    ))
    facts = []
    for fact in state.fused_planner_state:
        if (
            fact.subject == 'room315_right_shuttle_1'
            and fact.predicate in {'location_slot', 'location_block'}
        ):
            continue
        if (
            fact.subject == 'right:slot:4'
            and fact.predicate == 'occupancy'
        ):
            facts.append(replace(fact, value={
                'occupied': False,
                'shuttle': None,
                'sensor': 'DZI4R',
            }))
        else:
            facts.append(fact)
    length = public_rail_segment_lengths('right')['A34I']
    facts.append(_planner_fact(
        'room315_right_shuttle_1',
        'rail_position',
        {
            'side': 'right',
            'segment': 'A34I',
            's_m': 0.35,
            's_ratio': 0.35 / length,
            'segment_length_m': length,
            'position_uncertainty_m': 0.0,
        },
        timestamp=state.timestamp,
        metadata={'source': 'accepted_visual_state'},
    ))
    return replace(
        state,
        state_id=state_id,
        fused_planner_state=facts,
    )


def _with_shuttle_moved_between_slots(
    state: ObservedState,
    *,
    shuttle_number: str,
    from_slot: str,
    to_slot: str,
    state_id: str,
) -> ObservedState:
    entity = f'room315_right_shuttle_{shuttle_number}'
    facts = []
    for fact in state.fused_planner_state:
        if (
            fact.subject == f'right:slot:{from_slot}'
            and fact.predicate == 'occupancy'
        ):
            facts.append(replace(fact, value={
                'occupied': False,
                'shuttle': None,
                'sensor': f'DZI{from_slot}R',
            }))
        elif (
            fact.subject == f'right:slot:{to_slot}'
            and fact.predicate == 'occupancy'
        ):
            facts.append(replace(fact, value={
                'occupied': True,
                'shuttle': entity,
                'sensor': f'DZI{to_slot}R',
            }))
        elif fact.subject == entity and fact.predicate == 'location_slot':
            facts.append(replace(fact, value=f'right:slot:{to_slot}'))
        elif fact.subject == entity and fact.predicate == 'location_block':
            facts.append(replace(fact, value=f'right:block:slot:{to_slot}'))
        else:
            facts.append(fact)
    return replace(state, state_id=state_id, fused_planner_state=facts)


def test_supervisor_rejection_reason_is_preserved_at_execution_boundary():
    class RejectingTransport(RecordingTransport):
        def wait_for_supervisor_decision(
            self,
            *,
            previous_count: int,
            timeout_s: float,
        ) -> dict | None:
            assert self._decision_count > previous_count
            return {
                'accepted': False,
                'status': 'rejected',
                'reason': 'certificate set does not match interior occupants',
            }

    executive = ClosedLoopExecutive(
        observed_state_provider=None,
        planner=None,
        transport=RejectingTransport(),
    )
    status = executive._publish_supervised_macro_command(
        {'action': 'switches', 'side': 'right'},
        {'mode': 'restore_normal_route_before_slot_motion'},
    )
    check = executive._supervisor_status_check(
        label='clearance_restore_switches',
        status=status,
    )

    assert status == 'rejected'
    assert check.reason == (
        'clearance_restore_switches_supervisor_rejected:'
        'certificate set does not match interior occupants'
    )
    assert check.details['supervisor_reason'] == (
        'certificate set does not match interior occupants'
    )


def test_executes_only_first_atomic_step_from_validated_plan():
    state = _base_state('inspect-state')
    fresh_state = replace(
        state,
        state_id='inspect-state-fresh',
        timestamp=state.timestamp + 0.1,
    )
    provider = SequenceObservedStateProvider([state, fresh_state])
    planner = SequencePlanner([
        ['inspect_state room315_system', 'prepare_switches right switch=A1 state=INTERIOR'],
    ])
    transport = RecordingTransport()

    result = ClosedLoopExecutive(
        observed_state_provider=provider,
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(max_steps=4),
    ).run(_inspection_goal())

    assert result.succeeded
    assert result.plan_attempts == 1
    assert len(result.executed_steps) == 1
    assert result.executed_steps[0].plan_length == 2
    assert result.executed_steps[0].symbolic_step == 'inspect_state room315_system'
    assert transport.commands == []
    assert result.executed_steps[0].postcondition.reason == (
        'fresh_validated_observation_inspected'
    )


def test_inspection_rejects_replayed_observation_as_not_fresh():
    state = _base_state('inspect-replayed-state')
    provider = SequenceObservedStateProvider([state, state])
    planner = SequencePlanner([['inspect_state room315_system']])
    transport = RecordingTransport()

    result = ClosedLoopExecutive(
        observed_state_provider=provider,
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(max_replans=0),
    ).run(_inspection_goal())

    assert result.status == 'aborted'
    assert result.reason == (
        'postcondition_unknown:inspection_requires_fresh_observation'
    )
    assert result.executed_steps[0].postcondition.details == {
        'before_state_id': state.state_id,
        'after_state_id': state.state_id,
        'before_timestamp': state.timestamp,
        'after_timestamp': state.timestamp,
        'supervisor_command_published': False,
    }
    assert transport.commands == [
        {
            'action': 'stop_all',
            'reason': 'postcondition_unknown',
            'closed_loop_executive': {
                'mode': 'safe_abort',
                'reason': 'postcondition_unknown',
            },
        }
    ]


def test_rejects_wrong_shuttle_plan_before_any_motion_command():
    state = _base_state('wrong-shuttle-plan')
    provider = SequenceObservedStateProvider([state])
    planner = SequencePlanner([[
        'move_shuttle_to_slot right_shuttle_2 right right_yaskawa '
        'right_staubli right_slot_1 right_slot_3 speed=0.2'
    ]])
    transport = RecordingTransport()

    result = ClosedLoopExecutive(
        observed_state_provider=provider,
        planner=planner,
        transport=transport,
    ).run(_transport_goal())

    assert result.status == 'aborted'
    assert result.reason.startswith('planned_action_contract_violation:')
    assert [command['action'] for command in transport.commands] == [
        'stop_all'
    ]


@pytest.mark.parametrize(
    'symbolic_step,expected_error',
    [
        (
            'move_shuttle_to_slot right_shuttle_1 right right_yaskawa '
            'right_staubli right_slot_2 right_slot_3',
            'missing_frozen_precondition:'
            'shuttle_at_slot:right_shuttle_1:right_slot_2',
        ),
        (
            'move_shuttle_via_topology_to_slot right_shuttle_1 right '
            'right_slot_1 right_topology_a12i right_yaskawa right_staubli '
            'right_slot_3',
            'missing_frozen_precondition:'
            'shuttle_at_topology_block:right_shuttle_1:right_topology_a12i',
        ),
    ],
)
def test_motion_action_must_match_frozen_source_and_topology(
    symbolic_step,
    expected_error,
):
    problem = build_pddl_problem_from_observed_state_task_goal(
        _base_state('frozen-motion-contract'),
        _transport_goal(),
    )
    translated = translate_plan([symbolic_step])[0]

    error = ClosedLoopExecutive._first_action_contract_error(
        first_step=translated.pddl_step,
        translated_step=translated,
        problem=problem,
        task_goal=_transport_goal(),
    )

    assert error == expected_error


def test_goal_atom_cannot_satisfy_first_action_init_precondition():
    problem = build_pddl_problem_from_observed_state_task_goal(
        _r4_blocked_state('goal-atom-is-not-init'),
        _r4_goal(),
    )
    translated = translate_plan([
        'begin_route_clearance right_shuttle_4 right '
        'right_slot_2 right_slot_2'
    ])[0]

    # The target goal contains R4/right_slot_2, but the accepted initial state
    # has R4 at right_slot_4.  Only :init may satisfy action preconditions.
    assert '(shuttle_at_slot right_shuttle_4 right_slot_2)' in problem.goal_text
    error = ClosedLoopExecutive._first_action_contract_error(
        first_step=translated.pddl_step,
        translated_step=translated,
        problem=problem,
        task_goal=_r4_goal(),
    )

    assert error == (
        'missing_frozen_precondition:'
        'shuttle_at_slot:right_shuttle_4:right_slot_2'
    )


def test_inspection_requires_frozen_init_authorization():
    problem = build_pddl_problem_from_observed_state_task_goal(
        _base_state('inspection-init-authorization'),
        _inspection_goal(),
    )
    problem = replace(
        problem,
        problem_text=problem.problem_text.replace(
            '    (inspection_required room315_system)\n',
            '',
        ),
    )
    translated = translate_plan(['inspect_state room315_system'])[0]

    error = ClosedLoopExecutive._first_action_contract_error(
        first_step=translated.pddl_step,
        translated_step=translated,
        problem=problem,
        task_goal=_inspection_goal(),
    )

    assert error == 'missing_frozen_inspection_target:room315_system'


def test_topology_motion_requires_frozen_configured_route():
    source = _with_r2_interior(
        _r4_blocked_state('unconfigured-topology-motion'),
        'unconfigured-topology-motion',
        visual_s_m=1.4166 * 0.95,
    )
    problem = build_pddl_problem_from_observed_state_task_goal(
        source,
        _r2_slot1_goal(),
    )
    translated = translate_plan([
        'move_shuttle_from_segment_to_slot right_shuttle_2 right '
        'right_topology_a34i right_yaskawa right_slot_1'
    ])[0]

    error = ClosedLoopExecutive._first_action_contract_error(
        first_step=translated.pddl_step,
        translated_step=translated,
        problem=problem,
        task_goal=_r2_slot1_goal(),
    )

    assert error == (
        'missing_frozen_precondition:topology_route_configured:'
        'right_shuttle_2:right_topology_a34i:right_slot_1'
    )


def test_missing_motion_mode_is_not_stop_proof():
    check = _verify_shuttle_stopped(
        _base_state('missing-motion-mode'),
        'right_shuttle_1',
        'right',
    )

    assert check.status == 'unknown'
    assert check.reason == 'missing_shuttle_motion_mode_stop_proof'


def test_unsatisfied_transport_terminal_is_not_sent_as_done_or_accepted():
    state = _base_state('false-terminal-plan')
    provider = SequenceObservedStateProvider([state, state])
    planner = SequencePlanner([[
        'finish_task right_shuttle_1 right_staubli'
    ]])
    transport = RecordingTransport()

    result = ClosedLoopExecutive(
        observed_state_provider=provider,
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(max_replans=0),
    ).run(_transport_goal())

    assert result.status == 'aborted'
    assert result.reason == (
        'postcondition_mismatch:'
        'terminal_plan_claimed_unsatisfied_transport_goal'
    )
    assert all(command.get('action') != 'DONE' for command in transport.commands)


def test_retries_recoverable_unknown_before_planning():
    unknown = _with_fact(
        _base_state('unknown-slot'),
        'right:slot:3',
        'occupancy',
        status='unknown',
    )
    known = _base_state('known-slot')
    fresh_known = replace(
        known,
        state_id='known-slot-fresh',
        timestamp=known.timestamp + 0.1,
    )
    provider = SequenceObservedStateProvider([unknown, known, fresh_known])
    planner = SequencePlanner([['inspect_state room315_system']])
    transport = RecordingTransport()

    result = ClosedLoopExecutive(
        observed_state_provider=provider,
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(max_unknown_retries=2),
    ).run(_inspection_goal())

    assert result.succeeded
    assert result.unknown_retries == 1
    assert result.plan_attempts == 1
    assert provider.calls >= 3
    assert transport.commands == []


def test_postcondition_mismatch_replans_before_next_command():
    source = _base_state('r1-source')
    target = _state_with_r1_at_slot3('r1-target')
    provider = SequenceObservedStateProvider([source, source, source, target])
    planner = SequencePlanner([
        [
            'move_shuttle_to_slot right_shuttle_1 right right_yaskawa '
            'right_staubli right_slot_1 right_slot_3',
            'finish_candidate_task right_shuttle_1 right_staubli right_slot_3',
        ],
        [
            'move_shuttle_to_slot right_shuttle_1 right right_yaskawa '
            'right_staubli right_slot_1 right_slot_3'
        ],
    ])
    transport = RecordingTransport()

    result = ClosedLoopExecutive(
        observed_state_provider=provider,
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(max_replans=2, max_steps=4),
    ).run(_transport_goal())

    assert result.succeeded
    assert result.plan_attempts == 2
    assert result.replans == 1
    assert any('target_slot_expected' in reason for reason in result.replan_reasons)
    assert [command['action'] for command in transport.commands] == ['shuttle', 'shuttle']
    assert all(
        command['target_slot'] == '3'
        for command in transport.commands
    )
    assert all(record.plan_length >= 1 for record in result.executed_steps)


def test_exact_sensor_and_controller_stop_override_visual_arrival_bias():
    source = _base_state('r1-source')
    provider = SequenceObservedStateProvider([source, source])
    planner = SequencePlanner([[
        'move_shuttle_to_slot right_shuttle_1 right right_yaskawa '
        'right_staubli right_slot_1 right_slot_3',
    ]])
    transport = RecordingTransport(deterministic_arrival_proof=True)

    result = ClosedLoopExecutive(
        observed_state_provider=provider,
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(max_replans=1, max_steps=2),
    ).run(_transport_goal())

    assert result.succeeded
    assert result.plan_attempts == 1
    assert result.replans == 0
    postcondition = result.executed_steps[0].postcondition
    assert postcondition.reason == 'target_slot_sensor_identity_and_stop_verified'
    assert postcondition.details['visual_disagreement'] is True
    assert (
        postcondition.details['arrival_verification']['matched_by']
        == 'deterministic_slot_sensor'
    )


def test_r2_visual_a34i_to_slot1_executes_exact_authoritative_topology_route():
    source = _with_r2_interior(
        _r4_blocked_state('r2-a34i-source'),
        'r2-a34i-source',
        visual_s_m=1.4166 * 0.95,
    )
    configured = _with_fact(
        source,
        'right:switch:A4',
        'state',
        value='INTERIOR',
    )
    provider = SequenceObservedStateProvider([
        source,
        configured,
        configured,
        configured,
    ])
    planner = SequencePlanner([
        [
            'prepare_topology_route right_shuttle_2 right '
            'right_topology_a34i right_slot_1 right_switch_group',
            'move_shuttle_from_segment_to_slot right_shuttle_2 right '
            'right_topology_a34i right_yaskawa right_slot_1 speed=0.2',
        ],
        [
            'move_shuttle_from_segment_to_slot right_shuttle_2 right '
            'right_topology_a34i right_yaskawa right_slot_1 speed=0.2',
        ],
    ])
    transport = RecordingTransport(
        deterministic_arrival_proof=True,
        arrival_shuttle='room315_right_shuttle_2',
        arrival_sensor='DZI1R',
    )

    result = ClosedLoopExecutive(
        observed_state_provider=provider,
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(max_steps=4),
    ).run(_r2_slot1_goal())

    assert result.succeeded
    assert result.reason == 'task_goal_satisfied'
    assert result.plan_attempts == 2
    assert [command['action'] for command in transport.commands] == [
        'switches',
        'stoppers',
        'shuttle',
    ]
    assert transport.commands[0]['switches'] == {
        'A1': 'EXTERIOR',
        'A2': 'EXTERIOR',
        'A3': 'EXTERIOR',
        'A4': 'INTERIOR',
    }
    assert transport.commands[1]['stoppers'] == {
        device: '0'
        for device in ('A1', 'A2', 'A3', 'A4')
    }
    moving_commands = [
        command
        for command in transport.commands
        if command.get('action') == 'shuttle'
        and command.get('command') == 'ON'
    ]
    assert len(moving_commands) == 1
    assert moving_commands[0]['shuttle'] == 'right_shuttle_2'
    assert moving_commands[0]['target_slot'] == '1'
    target_wait = next(wait for wait in transport.waits if wait[0] == 'target_arrival')
    assert target_wait[1]['target_sensors'] == ['DZI1R']
    assert target_wait[1]['shuttle'] == 'right_shuttle_2'
    assert target_wait[1]['target_slot'] == '1'
    final_check = result.executed_steps[-1].postcondition
    assert final_check.reason == 'target_slot_sensor_identity_and_stop_verified'
    arrival = final_check.details['arrival_verification']
    assert arrival['target_sensor'] == 'DZI1R'
    assert arrival['shuttle_contract'] == 'right_shuttle_2'
    assert arrival['sensor_identity_confirmed'] is True
    assert arrival['controller_stop_confirmed'] is True
    assert arrival['controller_position_fields_used_for_localization'] is False


def test_topology_route_accepts_persisted_motion_effect_when_visual_branch_is_wrong():
    """A fresh A12I misclassification cannot erase a proved A34I motion."""

    certificate = _persisted_r2_clearance_certificate()
    source = _with_r2_interior(
        _r4_blocked_state('persisted-a34i-visual-a12i'),
        'persisted-a34i-visual-a12i',
        visual_s_m=0.70,
        visual_segment='A12I',
    )
    problem = build_pddl_problem_from_observed_state_task_goal(
        source,
        _r2_slot1_goal(),
        runtime_clearance_certificates={'R2': certificate},
    )
    route = problem.provenance['topology_routes']['routes'][0]
    consistency = route['runtime_clearance_visual_consistency']
    assert route['source_segment'] == 'A34I'
    assert consistency['satisfied'] is False
    assert consistency['accepted_visual_internal_segment'] == 'A12I'
    assert consistency['certificate_used_as_persisted_execution_effect'] is True
    assert consistency['raw_visual_prediction_preserved'] is True

    translated = translate_plan([
        'prepare_topology_route right_shuttle_2 right '
        'right_topology_a34i right_slot_1 right_switch_group'
    ])[0]
    transport = RecordingTransport()
    executive = ClosedLoopExecutive(
        observed_state_provider=SequenceObservedStateProvider([source]),
        planner=SequencePlanner([[]]),
        transport=transport,
    )
    executive._runtime_clearance_certificates = {'R2': certificate}

    status, check = executive._prepare_topology_route(
        translated_step=translated,
        problem=problem,
        plan_length=2,
        step_index=0,
    )

    assert status == 'accepted'
    assert check.status == 'satisfied'
    assert check.reason == 'authoritative_topology_route_configured'
    origin_proof = transport.commands[0]['closed_loop_executive'][
        'persisted_topology_route_origin_proof'
    ]
    assert origin_proof['validated'] is True
    assert origin_proof['source_segment'] == 'A34I'
    assert origin_proof['raw_visual_prediction_preserved'] is True
    assert origin_proof['model_prediction_replaced'] is False
    assert origin_proof['controller_position_fields_used_for_localization'] is False


def test_topology_route_rejects_visual_disagreement_without_complete_motion_effect():
    certificate = _persisted_r2_clearance_certificate()
    source = _with_r2_interior(
        _r4_blocked_state('invalid-persisted-a34i-effect'),
        'invalid-persisted-a34i-effect',
        visual_s_m=0.70,
        visual_segment='A12I',
    )
    problem = build_pddl_problem_from_observed_state_task_goal(
        source,
        _r2_slot1_goal(),
        runtime_clearance_certificates={'R2': certificate},
    )
    translated = translate_plan([
        'prepare_topology_route right_shuttle_2 right '
        'right_topology_a34i right_slot_1 right_switch_group'
    ])[0]
    invalid_certificate = dict(certificate)
    invalid_certificate['controller_stop_confirmed'] = False
    transport = RecordingTransport()
    executive = ClosedLoopExecutive(
        observed_state_provider=SequenceObservedStateProvider([source]),
        planner=SequencePlanner([[]]),
        transport=transport,
    )
    executive._runtime_clearance_certificates = {
        'R2': invalid_certificate,
    }

    status, check = executive._prepare_topology_route(
        translated_step=translated,
        problem=problem,
        plan_length=2,
        step_index=0,
    )

    assert status == 'rejected'
    assert check.reason == (
        'topology_route_clearance_certificate_visual_'
        'consistency_failed_without_valid_persisted_effect'
    )
    assert transport.commands == []


def test_live_r3_advanced_a34i_effect_survives_visual_a12i_misclassification():
    """Replay the exact proof preceding the loaded-R4-to-slot-2 rejection."""

    certificate = {
        **_persisted_r2_clearance_certificate(),
        'identity': 'R3',
        'shuttle': 'right_shuttle_3',
        'target_s_m': 1.066772,
        'observed_segment': 'A12I',
        'observed_s_m': 0.792615532875061,
        'matched_by': 'certified_interior_origin_plus_bounded_travel_time',
        'interior_advance_origin_certified': True,
        'motion_origin_s_m': 0.36,
        'bounded_motion_distance_m': 0.706772,
        'origin_clearance_proof': {
            'identity': 'R3',
            'target_segment': 'A34I',
            'target_s_m': 0.36,
            'entry_sensor': 'DA3IR',
            'entry_sensor_identity_confirmed': True,
            'controller_stop_confirmed': True,
            'bounded_commanded_motion_completed': True,
            'controller_position_fields_used_for_localization': False,
        },
    }
    route = {
        'shuttle': 'right_shuttle_3',
        'side': 'right',
        'source_segment': 'A34I',
        'source_s_ratio': 1.066772 / 1.416772043341839,
    }
    consistency = {
        'required': True,
        'satisfied': False,
        'certificate_target_public_segment': 'A34I',
        'certificate_target_internal_segment': 'A34I',
        'accepted_visual_internal_segment': 'A12I',
        'certificate_used_as_localization': False,
        'certificate_used_as_persisted_execution_effect': True,
        'planning_origin_segment': 'A34I',
        'raw_visual_prediction_preserved': True,
    }
    executive = ClosedLoopExecutive(
        observed_state_provider=None,
        planner=None,
        transport=None,
    )
    executive._runtime_clearance_certificates = {'R3': certificate}

    proof = executive._validated_persisted_topology_route_origin(
        route=route,
        consistency=consistency,
    )

    assert proof['validated'] is True
    assert proof['identity'] == 'R3'
    assert proof['source_segment'] == 'A34I'
    assert proof['matched_by'] == (
        'certified_interior_origin_plus_bounded_travel_time'
    )
    assert proof['raw_visual_prediction_preserved'] is True
    assert proof['model_prediction_replaced'] is False


def test_new_executive_imports_persisted_clearance_before_first_problem():
    blocked = _r4_blocked_state('persisted-certificate-source')
    # The learned state still reports R2 on the parallel exterior branch. The
    # persisted effect certificate proves only that the prior TaskGoal moved
    # R2 into A34I; it does not replace the model prediction.
    cleared = _with_r2_interior(
        blocked,
        'persisted-certificate-cleared',
        visual_s_m=1.13328,
        visual_segment='A34E',
    )
    arrived = _with_r4_at_slot2(cleared, 'persisted-certificate-arrived')
    certificate = _persisted_r2_clearance_certificate()

    # Without the persisted certificate, the accepted visual branch error
    # makes R2 an exterior route blocker. Dual-branch recovery can now move
    # that apparent blocker safely through A1, but it cannot manufacture a
    # direct R4 route. This establishes that the first planning problem below
    # can be direct only if the new executive synchronizes its provider-owned
    # certificate before building that problem.
    uncertified = build_pddl_problem_from_observed_state_task_goal(
        cleared,
        _r4_goal(),
    )
    uncertified_clearance = uncertified.provenance[
        'target_blocker_clearance_plan'
    ]
    assert uncertified_clearance['required'] is True
    assert uncertified_clearance['ordered_relocations'][0]['shuttle'] == (
        'right_shuttle_2'
    )
    assert uncertified_clearance['ordered_relocations'][0]['destination'][
        'target_segment'
    ] == 'A12I'

    provider = PersistedCertificateObservedStateProvider(
        [cleared, arrived],
        {'R2': certificate},
    )
    planner = SequencePlanner([[
        'move_shuttle_to_slot right_shuttle_4 right right_staubli '
        'right_yaskawa right_slot_4 right_slot_2',
    ]])
    executive = ClosedLoopExecutive(
        observed_state_provider=provider,
        planner=planner,
        transport=RecordingTransport(),
        config=ClosedLoopExecutiveConfig(max_steps=2),
    )

    result = executive.run(_r4_goal())

    assert result.succeeded
    assert provider.certificate_reads >= 1
    assert len(planner.problems) == 1
    first_problem = planner.problems[0]
    assert first_problem.provenance[
        'target_blocker_clearance_plan'
    ]['required'] is False
    certified = first_problem.provenance['route_clearance'][
        'sensor_certified_interior_clearances'
    ]
    assert certified == [{
        'shuttle': 'right_shuttle_2',
        'target_s_m': 0.7083,
        'entry_sensor': 'DA3IR',
        'controller_position_fields_used_for_localization': False,
    }]
    assert executive._runtime_clearance_certificates['R2'] == certificate


def test_blocked_r4_goal_holds_clearance_mode_until_all_relocations_finish():
    blocked = _r4_blocked_state()
    cleared = _with_r2_interior(
        blocked,
        'r2-cleared',
        visual_segment='A34I',
    )
    arrived = _with_r4_at_slot2(cleared, 'r4-arrived')
    clearance_blocked = _with_clearance_device_mode(
        blocked,
        'clearance-blocked',
    )
    clearance_cleared = _with_clearance_device_mode(
        cleared,
        'clearance-cleared',
    )
    provider = SequenceObservedStateProvider([
        blocked,
        clearance_blocked,
        clearance_blocked,
        clearance_cleared,
        clearance_cleared,
        cleared,
        cleared,
        arrived,
    ])
    planner = SequencePlanner([
        [
            'begin_route_clearance right_shuttle_4 right '
            'right_slot_4 right_slot_2'
        ],
        [
            'relocate_blocker_to_interior right_shuttle_2 right_shuttle_4 '
            'right right_slot_4 right_slot_2'
        ],
        [
            'finish_route_clearance right_shuttle_4 right '
            'right_slot_4 right_slot_2'
        ],
        [
            'move_shuttle_to_slot right_shuttle_4 right right_staubli '
            'right_yaskawa right_slot_4 right_slot_2'
        ],
    ])
    transport = RecordingTransport(clearance_observed_segment='A34I')

    result = ClosedLoopExecutive(
        observed_state_provider=provider,
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(max_steps=8),
    ).run(_r4_goal())

    assert result.succeeded
    assert result.plan_attempts == 4
    clearance_postcondition = result.executed_steps[1].postcondition
    assert clearance_postcondition.satisfied
    assert clearance_postcondition.reason == (
        'blocker_sensor_motion_certified_interior_clearance'
    )
    assert clearance_postcondition.details['target_segment'] == 'A34I'
    assert clearance_postcondition.details['observed_segment'] == 'A34I'
    assert planner.problems[0].goal_text == (
        '(clearance_relocated right_shuttle_2)'
    )
    assert planner.problems[0].provenance['planning_phase'] == (
        'clear_blocker_to_interior_loop'
    )
    assert planner.problems[1].goal_text == (
        '(clearance_relocated right_shuttle_2)'
    )
    assert planner.problems[1].provenance['planning_phase'] == (
        'clear_blocker_to_interior_loop'
    )
    assert planner.problems[2].goal_text == (
        '(and (task_done right_shuttle_4 right_yaskawa) '
        '(shuttle_at_slot right_shuttle_4 right_slot_2))'
    )
    assert [command['action'] for command in transport.commands[:3]] == [
        'switches',
        'stoppers',
        'shuttle',
    ]
    assert transport.commands[0]['switches'] == {
        'A1': 'EXTERIOR',
        'A2': 'EXTERIOR',
        'A3': 'INTERIOR',
        'A4': 'INTERIOR',
    }
    assert transport.commands[1]['stoppers'] == {
        'A1': '0',
        'A2': '0',
        'A3': '0',
        'A4': '1',
    }
    assert transport.commands[2]['shuttle'] == 'right_shuttle_2'
    assert transport.commands[2].get('target_slot', '') == ''
    assert transport.commands[2]['target_stopper'] == 'A4'
    visual_wait = next(item for item in transport.waits if item[0] == 'visual_position_and_stop')
    assert visual_wait[1]['target_segment'] == 'A34I'
    assert visual_wait[1]['target_s_m'] == pytest.approx(
        public_rail_segment_lengths('right')['A34I'] - 0.35
    )
    assert visual_wait[1]['entry_sensor'] == 'DA3IR'
    restoration = [
        command
        for command in transport.commands
        if command.get('closed_loop_executive', {}).get('mode')
        == 'restore_normal_route_after_interior_clearance'
    ]
    assert [command['action'] for command in restoration] == [
        'switches',
        'stoppers',
    ]


    relocation_index = next(
        index
        for index, command in enumerate(transport.commands)
        if command.get('shuttle') == 'right_shuttle_2'
        and command.get('command') == 'ON'
    )
    restore_index = transport.commands.index(restoration[0])
    assert restore_index > relocation_index
    assert restoration[0]['switches'] == {
        device: 'EXTERIOR'
        for device in ('A1', 'A2', 'A3', 'A4')
    }
    restoration_metadata = restoration[0]['closed_loop_executive']
    assert restoration_metadata['route_normalization_proof'][
        'clearance_pause_safe'
    ] is True
    assert restoration_metadata['route_normalization_proof'][
        'certified_stopped_interior_shuttles'
    ] == ['right_shuttle_2']
    assert restoration_metadata['runtime_clearance_certificates'][
        'R2'
    ]['entry_sensor'] == 'DA3IR'
    assert restoration[1]['stoppers'] == {
        device: '0'
        for device in ('A1', 'A2', 'A3', 'A4')
    }
    assert any(
        wait[0] == 'fresh_visual_observation'
        for wait in transport.waits
    )


@pytest.mark.parametrize('side', ('right', 'left'))
def test_a12i_clearance_mode_uses_a1_a2_devices_from_provenance(side):
    selected = f'{side}_shuttle_2'
    spec = ScenarioSpec(
        goal_id=f'{side}-a12i-executive-branch',
        side=side,
        shuttle=selected,
        source='yaskawa',
        target='staubli' if side == 'right' else 'kuka',
        target_slot='4',
        payload_condition='empty',
        start_slots_by_shuttle=tuple(
            (f'{side}_shuttle_{identity}', str(identity))
            for identity in range(1, 5)
        ),
    )
    state = _observed_state_from_scenario_spec(spec)
    goal = TaskGoal(
        goal_id=spec.goal_id,
        description='Move slot 2 shuttle to slot 4',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': side,
            'target_kind': 'slot',
            'target_slot': '4',
            'target_shuttle': selected,
            'payload_filter': 'any',
        },
    )
    parent = build_pddl_problem_from_observed_state_task_goal(state, goal)
    problem = ClosedLoopExecutive._next_planning_problem(parent)
    transport = RecordingTransport()
    executive = ClosedLoopExecutive(
        observed_state_provider=SequenceObservedStateProvider([state]),
        planner=SequencePlanner([[]]),
        transport=transport,
    )

    status, check = executive._enter_route_clearance_mode(
        side=side,
        problem=problem,
        plan_length=2,
        step_index=0,
        symbolic_step=(
            f'begin_route_clearance {selected} {side} '
            f'{side}_slot_2 {side}_slot_4'
        ),
    )

    assert status == 'accepted'
    assert check.satisfied
    assert transport.commands[0]['switches'] == {
        'A1': 'INTERIOR',
        'A2': 'INTERIOR',
        'A3': 'EXTERIOR',
        'A4': 'EXTERIOR',
    }
    assert transport.commands[1]['stoppers'] == {
        'A1': '0',
        'A2': '1',
        'A3': '0',
        'A4': '0',
    }
    assert check.details['gate_switch'] == 'A1'
    assert check.details['exit_switch'] == 'A2'
    assert check.details['target_segment'] == 'A12I'

    # The real executive rebuilds the problem from device feedback after the
    # begin action. Mirror that fresh active-clearance snapshot here before
    # testing the subsequent shuttle motion.
    provenance = dict(problem.provenance)
    route_normalization = dict(provenance['route_normalization'])
    by_side = dict(route_normalization['by_side'])
    side_normalization = dict(by_side[side])
    side_normalization.update({
        'switches': {
            'A1': 'interior', 'A2': 'interior',
            'A3': 'exterior', 'A4': 'exterior',
        },
        'stoppers': {
            'A1': 'open', 'A2': 'closed',
            'A3': 'open', 'A4': 'open',
        },
    })
    by_side[side] = side_normalization
    route_normalization['by_side'] = by_side
    provenance['route_normalization'] = route_normalization
    problem = replace(problem, provenance=provenance)

    relocation = problem.provenance['clearance_relocation']
    blocker = relocation['shuttle']
    translated = translate_plan([(
        f'relocate_blocker_to_interior {blocker} {selected} {side} '
        f'{side}_slot_2 {side}_slot_4 speed=0.2'
    )])[0]
    relocation_status, relocation_check = (
        executive._execute_interior_clearance(
            translated_step=translated,
            problem=problem,
            plan_length=1,
            step_index=1,
        )
    )

    assert relocation_status == 'accepted'
    assert relocation_check.satisfied
    shuttle_on = next(
        command
        for command in transport.commands
        if command.get('command') == 'ON'
    )
    assert shuttle_on['shuttle'] == blocker
    assert shuttle_on['target_stopper'] == 'A2'
    interior_wait = next(
        details
        for name, details in transport.waits
        if name == 'visual_position_and_stop'
    )
    assert interior_wait['target_segment'] == 'A12I'
    assert interior_wait['entry_sensor'] == (
        'DA1IR' if side == 'right' else 'DA1IL'
    )


def test_cross_branch_clearance_uses_complete_route_specific_switch_assignment():
    """The target hold retains the certified source-interior exit switch."""

    spec = ScenarioSpec(
        goal_id='cross-branch-route-specific-switches',
        side='right',
        shuttle='right_shuttle_2',
        source='yaskawa',
        target='staubli',
        target_slot='4',
        payload_condition='empty',
        start_slots_by_shuttle=tuple(
            (f'right_shuttle_{identity}', str(identity))
            for identity in range(1, 5)
        ),
    )
    state = _observed_state_from_scenario_spec(spec)
    goal = TaskGoal(
        goal_id=spec.goal_id,
        description='Move a blocker from A12I into A34I',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '4',
            'target_shuttle': 'right_shuttle_2',
            'payload_filter': 'any',
        },
    )
    parent = build_pddl_problem_from_observed_state_task_goal(state, goal)
    problem = ClosedLoopExecutive._next_planning_problem(parent)
    provenance = dict(problem.provenance)
    relocation = dict(provenance['clearance_relocation'])
    destination = dict(relocation['destination'])
    proof = dict(destination['interior_entry_route_proof'])
    proof['required_switches'] = {
        'A1': 'E',
        'A2': 'I',
        'A3': 'I',
        'A4': 'I',
    }
    destination.update({
        'gate_switch': 'A3',
        'exit_switch': 'A4',
        'target_segment': 'A34I',
        'interior_entry_route_proof': proof,
    })
    relocation['destination'] = destination
    provenance['clearance_relocation'] = relocation
    problem = replace(problem, provenance=provenance)
    transport = RecordingTransport()
    executive = ClosedLoopExecutive(
        observed_state_provider=SequenceObservedStateProvider([state]),
        planner=SequencePlanner([[]]),
        transport=transport,
    )

    status, check = executive._enter_route_clearance_mode(
        side='right',
        problem=problem,
        plan_length=2,
        step_index=0,
        symbolic_step=(
            'begin_segment_route_clearance right_shuttle_2 right '
            'right_topology_a12i right_slot_4'
        ),
    )

    assert status == 'accepted'
    assert check.satisfied
    assert transport.commands[0]['switches'] == {
        'A1': 'EXTERIOR',
        'A2': 'INTERIOR',
        'A3': 'INTERIOR',
        'A4': 'INTERIOR',
    }
    assert transport.commands[1]['stoppers'] == {
        'A1': '0',
        'A2': '0',
        'A3': '0',
        'A4': '1',
    }
    assert check.details['route_specific_switch_assignment'] is True
    assert check.details['authoritative_required_switches'] == {
        'A1': 'E',
        'A2': 'I',
        'A3': 'I',
        'A4': 'I',
    }

    blocker = relocation['shuttle']
    translated = translate_plan([(
        f'relocate_segment_blocker_to_interior {blocker} '
        'right_shuttle_2 right right_topology_a12i right_slot_4 speed=0.2'
    )])[0]
    command_count_before = len(transport.commands)

    relocation_status, relocation_check = (
        executive._execute_interior_clearance(
            translated_step=translated,
            problem=problem,
            plan_length=1,
            step_index=1,
        )
    )

    assert relocation_status == 'rejected'
    assert 'clearance_motion_route_assignment_not_active' in (
        relocation_check.reason
    )
    assert len(transport.commands) == command_count_before


def test_duplicate_finish_clearance_is_rejected_after_one_normalization():
    blocked = _r4_blocked_state('finish-proof-blocked')
    visual_mismatch = _with_r2_interior(
        blocked,
        'finish-proof-visual-mismatch',
        visual_segment='A34E',
    )
    clearance_blocked = _with_clearance_device_mode(
        blocked,
        'finish-proof-clearance-blocked',
    )
    clearance_mismatch = _with_clearance_device_mode(
        visual_mismatch,
        'finish-proof-clearance-mismatch',
    )
    transport = RecordingTransport(clearance_observed_segment='A34E')
    planner = SequencePlanner([
        [
            'begin_route_clearance right_shuttle_4 right '
            'right_slot_4 right_slot_2'
        ],
        [
            'relocate_blocker_to_interior right_shuttle_2 right_shuttle_4 '
            'right right_slot_4 right_slot_2'
        ],
        [
            'finish_route_clearance right_shuttle_4 right '
            'right_slot_4 right_slot_2'
        ],
    ])

    result = ClosedLoopExecutive(
        observed_state_provider=SequenceObservedStateProvider([
            blocked,
            clearance_blocked,
            clearance_blocked,
            clearance_mismatch,
            clearance_mismatch,
        ]),
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(max_steps=5),
    ).run(_r4_goal())

    assert result.status == 'aborted'
    assert result.reason == (
        'clearance_phase_plan_violation:clearance_not_started,'
        'action=finish_route_clearance'
    )
    normalization_commands = [
        command
        for command in transport.commands
        if (
            command.get('closed_loop_executive', {}).get('mode')
            == 'restore_normal_route_after_interior_clearance'
        )
    ]
    assert [command['action'] for command in normalization_commands] == [
        'switches',
        'stoppers',
    ]


def test_startup_latch_recovery_accepts_certified_visual_disagreement():
    blocked = _r4_blocked_state('restart-clearance-blocked')
    visually_mismatched = _with_r2_interior(
        blocked,
        'restart-clearance-visual-mismatch',
        visual_segment='A34E',
    )
    active_state = _with_clearance_device_mode(
        visually_mismatched,
        'restart-clearance-active',
    )
    problem = build_pddl_problem_from_observed_state_task_goal(
        active_state,
        _r4_goal(),
        runtime_clearance_certificates={
            'R2': _persisted_r2_clearance_certificate(),
        },
    )
    normalization = problem.provenance['route_normalization']['by_side'][
        'right'
    ]
    assert normalization['certificate_segment_mismatches'] == [
        'right_shuttle_2'
    ]
    assert normalization[
        'clearance_lifecycle_visual_disagreements'
    ] == ['right_shuttle_2']
    translated = translate_plan([
        'finish_route_clearance right_shuttle_4 right '
        'right_slot_4 right_slot_2'
    ])[0]
    executive = ClosedLoopExecutive(
        observed_state_provider=None,
        planner=None,
        transport=None,
    )

    executive._resume_clearance_phase_from_problem(
        problem=problem,
        first_step=translated.pddl_step,
    )

    assert executive._route_clearance_active_side == 'right'
    assert executive._first_action_contract_error(
        first_step=translated.pddl_step,
        translated_step=translated,
        problem=problem,
        task_goal=_r4_goal(),
    ) == ''


def test_mixed_topology_is_normalized_before_slot_motion():
    normal = _base_state('normal-after-route-recovery')
    mixed = _with_fact(
        replace(normal, state_id='mixed-before-route-recovery'),
        'right:switch:A4',
        'state',
        value='I',
    )
    mixed_problem = build_pddl_problem_from_observed_state_task_goal(
        mixed,
        _transport_goal(),
    )
    arrived = _state_with_r1_at_slot3('arrived-after-route-recovery')
    provider = SequenceObservedStateProvider([
        mixed,
        normal,
        normal,
        arrived,
        arrived,
    ])
    planner = SequencePlanner([
        [
            'move_shuttle_to_slot right_shuttle_1 right right_yaskawa '
            'right_staubli right_slot_1 right_slot_3'
        ],
    ])
    transport = RecordingTransport(
        deterministic_arrival_proof=True,
        arrival_shuttle='room315_right_shuttle_1',
        arrival_sensor='DZI3R',
    )

    result = ClosedLoopExecutive(
        observed_state_provider=provider,
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(max_steps=4),
    ).run(_transport_goal())

    assert result.succeeded
    assert result.plan_attempts == 2
    assert planner.calls == 1
    assert len(planner.problems) == 1
    normalization = mixed_problem.provenance[
        'route_normalization'
    ]['by_side']['right']
    assert normalization['reconfiguration_required'] is True
    assert normalization['reconfiguration_safe'] is True
    planned_normalization = planner.problems[0].provenance[
        'route_normalization'
    ]['by_side']['right']
    assert planned_normalization['normal_route'] is True
    assert planned_normalization['reconfiguration_required'] is False
    assert [command['action'] for command in transport.commands] == [
        'switches',
        'stoppers',
        'shuttle',
    ]
    assert transport.commands[0]['switches'] == {
        device: 'EXTERIOR'
        for device in ('A1', 'A2', 'A3', 'A4')
    }
    assert transport.commands[1]['stoppers'] == {
        device: '0'
        for device in ('A1', 'A2', 'A3', 'A4')
    }
    assert transport.commands[0]['closed_loop_executive']['mode'] == (
        'restore_normal_route_before_slot_motion'
    )
    assert result.executed_steps[0].postcondition.reason == (
        'normal_route_restored_before_slot_motion'
    )
    assert any(
        wait[0] == 'fresh_visual_observation'
        for wait in transport.waits
    )


def test_exact_loaded_r4_goal_restores_route_parks_r3_then_reaches_slot3():
    normal = _loaded_r4_slot3_recovery_state('loaded-r4-normal-route')
    mixed = _with_fact(
        replace(normal, state_id='loaded-r4-mixed-route'),
        'right:switch:A4',
        'state',
        value='I',
    )
    r3_parked = _with_shuttle_moved_between_slots(
        normal,
        shuttle_number='3',
        from_slot='3',
        to_slot='4',
        state_id='r3-parked-at-slot4',
    )
    r4_arrived = _with_shuttle_moved_between_slots(
        r3_parked,
        shuttle_number='4',
        from_slot='2',
        to_slot='3',
        state_id='loaded-r4-arrived-at-slot3',
    )
    provider = PersistedCertificateObservedStateProvider(
        [
            mixed,
            normal,
            normal,
            r3_parked,
            r3_parked,
            r4_arrived,
            r4_arrived,
        ],
        {'R1': _persisted_r1_clearance_certificate()},
    )
    planner = SequencePlanner([
        [
            'move_shuttle_to_slot right_shuttle_3 right right_staubli '
            'right_staubli right_slot_3 right_slot_4'
        ],
        [
            'move_shuttle_to_slot right_shuttle_4 right right_yaskawa '
            'right_staubli right_slot_2 right_slot_3'
        ],
    ])
    transport = RecordingTransport(
        deterministic_arrival_proof=True,
        arrival_sensor_by_shuttle={
            'right_shuttle_3': 'DZI4R',
            'right_shuttle_4': 'DZI3R',
        },
    )

    grounded_goal = ground_transport_task_goal(
        _any_loaded_right_slot3_goal(),
        mixed,
    )
    assert grounded_goal.constraints['target_shuttle'] == (
        'room315_right_shuttle_4'
    )

    result = ClosedLoopExecutive(
        observed_state_provider=provider,
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(max_steps=6),
    ).run(grounded_goal)

    assert result.succeeded
    assert result.reason == 'task_goal_satisfied'
    assert result.plan_attempts == 3
    assert planner.calls == 2
    assert [problem.selected_shuttle for problem in planner.problems] == [
        'right_shuttle_3',
        'right_shuttle_4',
    ]
    assert planner.problems[0].goal_text == (
        '(shuttle_at_slot right_shuttle_3 right_slot_4)'
    )
    assert planner.problems[1].goal_text == (
        '(and (task_done right_shuttle_4 right_staubli) '
        '(shuttle_at_slot right_shuttle_4 right_slot_3))'
    )
    assert [command['action'] for command in transport.commands] == [
        'switches',
        'stoppers',
        'shuttle',
        'shuttle',
    ]
    assert [
        command['shuttle']
        for command in transport.commands
        if command['action'] == 'shuttle'
    ] == ['right_shuttle_3', 'right_shuttle_4']
    assert [record.postcondition.reason for record in result.executed_steps] == [
        'normal_route_restored_before_slot_motion',
        'target_slot_sensor_identity_and_stop_verified',
        'target_slot_sensor_identity_and_stop_verified',
    ]


def test_mixed_topology_normalization_fails_closed_without_stop_proof():
    mixed = _with_fact(
        _base_state('uncertified-interior-route'),
        'right:switch:A4',
        'state',
        value='I',
    )
    mixed = replace(
        mixed,
        fused_planner_state=[
            *mixed.fused_planner_state,
            _planner_fact(
                'room315_right_shuttle_1',
                'rail_position',
                {
                    'side': 'right',
                    'segment': 'A34I',
                    's_m': 0.35,
                    's_ratio': 0.25,
                    'segment_length_m': 1.4,
                    'position_uncertainty_m': 0.0,
                },
                timestamp=mixed.timestamp,
                metadata={'source': 'accepted_visual_state'},
            ),
        ],
    )
    planner = SequencePlanner([
        ['restore_normal_route right right_yaskawa right_staubli'],
    ])
    transport = RecordingTransport()

    result = ClosedLoopExecutive(
        observed_state_provider=SequenceObservedStateProvider([mixed]),
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(
            max_steps=1,
            max_unknown_retries=0,
        ),
    ).run(_transport_goal())

    assert planner.problems == []
    assert result.status == 'aborted'
    assert result.reason.startswith(
        'persistent_uncertainty:mixed rail route requires normalization '
        'but no certified safe normalization'
    )
    assert transport.commands == [{
        'action': 'stop_all',
        'reason': 'persistent_uncertainty',
        'closed_loop_executive': {
            'mode': 'safe_abort',
            'reason': 'persistent_uncertainty',
        },
    }]


@pytest.mark.parametrize(
    'malformed_step,expected_reason',
    [
        (
            'restore_normal_route left left_yaskawa left_kuka',
            'planned_action_contract_violation:'
            'side_mismatch:problem=right,plan=left',
        ),
        (
            'restore_normal_route right left_yaskawa right_staubli',
            'planned_action_contract_violation:'
            'missing_frozen_precondition:'
            'connected:right:left_yaskawa:right_staubli',
        ),
    ],
)
def test_route_normalization_rejects_plan_argument_mismatch(
    malformed_step,
    expected_reason,
):
    normal = _base_state('normal-for-malformed-route-plan')
    mixed = _with_fact(
        replace(normal, state_id='mixed-for-malformed-route-plan'),
        'right:switch:A4',
        'state',
        value='I',
    )
    problem = build_pddl_problem_from_observed_state_task_goal(
        mixed,
        _transport_goal(),
    )
    translated = translate_plan([malformed_step])[0]
    contract_error = ClosedLoopExecutive._first_action_contract_error(
        first_step=translated.pddl_step,
        translated_step=translated,
        problem=problem,
        task_goal=_transport_goal(),
    )

    assert (
        f'planned_action_contract_violation:{contract_error}'
        == expected_reason
    )


def test_clearance_latch_rejects_normal_route_action_before_any_shuttle_moves():
    blocked = _r4_blocked_state()
    clearance_blocked = _with_clearance_device_mode(
        blocked,
        'clearance-blocked',
    )
    provider = SequenceObservedStateProvider([
        blocked,
        clearance_blocked,
        clearance_blocked,
    ])
    planner = SequencePlanner([
        [
            'begin_route_clearance right_shuttle_4 right '
            'right_slot_4 right_slot_2'
        ],
        ['prepare_switches right staubli yaskawa'],
    ])
    transport = RecordingTransport()

    result = ClosedLoopExecutive(
        observed_state_provider=provider,
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(max_steps=4),
    ).run(_r4_goal())

    assert result.status == 'aborted'
    assert result.reason == (
        'clearance_phase_plan_violation:'
        'active_side=right,forbidden_action=prepare_switches'
    )
    assert [command['action'] for command in transport.commands] == [
        'switches',
        'stoppers',
        'stop_all',
    ]
    assert not any(
        command.get('switches') == {
            device: 'EXTERIOR'
            for device in ('A1', 'A2', 'A3', 'A4')
        }
        for command in transport.commands[2:]
    )


def test_clearance_latch_rejects_route_normalization_action():
    blocked = _r4_blocked_state()
    clearance_blocked = _with_clearance_device_mode(
        blocked,
        'clearance-blocked-before-illegal-normalization',
    )
    planner = SequencePlanner([
        [
            'begin_route_clearance right_shuttle_4 right '
            'right_slot_4 right_slot_2'
        ],
        ['restore_normal_route right right_staubli right_yaskawa'],
    ])
    transport = RecordingTransport()

    result = ClosedLoopExecutive(
        observed_state_provider=SequenceObservedStateProvider([
            blocked,
            clearance_blocked,
            clearance_blocked,
        ]),
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(max_steps=4),
    ).run(_r4_goal())

    assert result.status == 'aborted'
    assert result.reason == (
        'clearance_phase_plan_violation:'
        'active_side=right,forbidden_action=restore_normal_route'
    )
    assert [command['action'] for command in transport.commands] == [
        'switches',
        'stoppers',
        'stop_all',
    ]


def test_interior_clearance_accepts_visual_s_outlier_only_with_complete_sensor_proof():
    blocked = _r4_blocked_state()
    problem = build_pddl_problem_from_observed_state_task_goal(
        blocked,
        _r4_goal(),
    )
    translated = translate_plan([
        'relocate_blocker_to_interior right_shuttle_2 right_shuttle_4 '
        'right right_slot_4 right_slot_2 speed=0.2'
    ])[0]
    executive = ClosedLoopExecutive(
        observed_state_provider=SequenceObservedStateProvider([blocked]),
        planner=SequencePlanner([[]]),
        transport=RecordingTransport(),
    )
    target_s_m = problem.provenance[
        'target_blocker_clearance_plan'
    ]['ordered_relocations'][0]['destination']['target_s_m']
    visual_s_m = target_s_m - 0.18
    proof = PostconditionCheck(
        status='satisfied',
        reason='guarded_interior_stop_and_fresh_visual_frame_satisfied',
        details={
            'matched_by': (
                'interior_entry_sensor_plus_bounded_travel_time'
            ),
            'entry_sensor': 'DA3IR',
            'entry_sensor_identity_confirmed': True,
            'controller_stop_confirmed': True,
            'post_stop_visual_frame_received': True,
            'post_stop_visual_confirmation': False,
            'controller_position_fields_used_for_localization': False,
            'clearance_mode_held': True,
            'normal_route_restored': False,
            'observed_segment': 'A34I',
            'observed_s_m': visual_s_m,
        },
    )

    certificate = executive._interior_clearance_certificate(
        wait_check=proof,
        problem=problem,
        translated_step=translated,
    )

    assert certificate is not None
    assert certificate['visual_s_within_tolerance'] is False
    assert certificate['observed_s_m'] == visual_s_m
    assert certificate['absolute_error_m'] == pytest.approx(0.18)
    assert certificate['model_prediction_replaced'] is False
    assert certificate['controller_position_fields_used_for_localization'] is False

    check = executive._verify_interior_clearance(
        after_state=_with_r2_interior(
            blocked,
            'visual-s-outlier',
            visual_s_m=visual_s_m,
        ),
        problem=problem,
        translated_step=translated,
        clearance_certificate=certificate,
    )
    assert check.satisfied
    assert check.details['visual_longitudinal_disagreement'] is True

    disagreeing_branch = replace(
        proof,
        details={**proof.details, 'observed_segment': 'A34E'},
    )
    disagreement_certificate = executive._interior_clearance_certificate(
        wait_check=disagreeing_branch,
        problem=problem,
        translated_step=translated,
    )
    assert disagreement_certificate is not None
    assert disagreement_certificate['observed_segment'] == 'A34E'
    assert disagreement_certificate['target_segment'] == 'A34I'
    assert disagreement_certificate['model_segment_disagreement'] is True
    assert disagreement_certificate['visual_segment_prediction_preserved'] is True
    assert disagreement_certificate['model_prediction_replaced'] is False

    disagreeing_check = executive._verify_interior_clearance(
        after_state=_with_r2_interior(
            blocked,
            'visual-branch-disagreement',
            visual_s_m=visual_s_m,
            visual_segment='A34E',
        ),
        problem=problem,
        translated_step=translated,
        clearance_certificate=disagreement_certificate,
    )
    assert disagreeing_check.satisfied
    assert disagreeing_check.details['visual_segment_disagreement'] is True
    assert disagreeing_check.details['model_prediction_replaced'] is False

    incomplete_proofs = (
        {'entry_sensor': 'DA4IR'},
        {'entry_sensor_identity_confirmed': False},
        {'matched_by': 'visual_position_only'},
        {'controller_stop_confirmed': False},
        {'post_stop_visual_frame_received': False},
        {'controller_position_fields_used_for_localization': True},
        {'clearance_mode_held': False},
        {'normal_route_restored': True},
    )
    for overrides in incomplete_proofs:
        incomplete = replace(
            proof,
            details={**proof.details, **overrides},
        )
        assert executive._interior_clearance_certificate(
            wait_check=incomplete,
            problem=problem,
            translated_step=translated,
        ) is None


def test_device_postconditions_treat_wire_and_canonical_states_as_equivalent():
    state = _with_fact(
        _with_fact(
            _base_state('compact-device-states'),
            'right:switch:A1',
            'state',
            value='E',
        ),
        'right:stopper:A1',
        'state',
        value='0',
    )

    switch_check = _verify_device_state(
        state,
        side='right',
        device_kind='switch',
        assignments={'A1': 'EXTERIOR'},
        command_to_state=_canonical_switch_effect_state,
    )
    stopper_check = _verify_device_state(
        state,
        side='right',
        device_kind='stopper',
        assignments={'A1': 'open'},
        command_to_state=_canonical_stopper_effect_state,
    )

    assert switch_check.satisfied
    assert stopper_check.satisfied


def test_persistent_uncertainty_fails_closed_with_safe_abort():
    unknown = _with_fact(
        _base_state('always-unknown'),
        'right:slot:3',
        'occupancy',
        status='unknown',
    )
    provider = SequenceObservedStateProvider([unknown, unknown, unknown])
    planner = SequencePlanner([['inspect_state room315_system']])
    transport = RecordingTransport()

    result = ClosedLoopExecutive(
        observed_state_provider=provider,
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(max_unknown_retries=1),
    ).run(_inspection_goal())

    assert result.status == 'aborted'
    assert result.safe_abort_sent
    assert result.plan_attempts == 0
    assert result.unknown_retries == 1
    assert provider.calls == 2
    assert result.reason == (
        'persistent_uncertainty:'
        'fresh_visual_observation_unavailable_after:always-unknown:'
        'provider_replayed_same_state_id'
    )
    assert result.replan_reasons == (
        'recoverable_unknown:slot occupancy fact '
        "'right:slot:3'/'occupancy' is unknown; observation or recovery "
        'is required before planning',
    )
    assert transport.commands == [
        {
            'action': 'stop_all',
            'reason': 'persistent_uncertainty',
            'closed_loop_executive': {
                'mode': 'safe_abort',
                'reason': 'persistent_uncertainty',
            },
        }
    ]
