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
from room_315_contracts import ObservedState
from room_315_contracts import TaskGoal
from room_315_observed_state_provider import ObservedStateProvider
from room_315_pddl_scenario_generator import BasePlannerBackend
from room_315_pddl_scenario_generator import ScenarioSpec
from room_315_pddl_scenario_generator import ScenarioTransport
from room_315_pddl_scenario_generator import _observed_state_from_scenario_spec
from room_315_pddl_scenario_generator import _planner_fact
from room_315_pddl_scenario_generator import build_pddl_problem_from_observed_state_task_goal
from room_315_pddl_plan_translator import translate_plan


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
    ) -> None:
        self.commands: list[dict] = []
        self.waits: list[tuple[str, dict]] = []
        self._decision_count = 0
        self._visual_count = 0
        self.deterministic_arrival_proof = deterministic_arrival_proof
        self.clearance_observed_segment = clearance_observed_segment
        self.arrival_shuttle = arrival_shuttle
        self.arrival_sensor = arrival_sensor

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
            },
        ))
        result = {'arrived': True, 'reason': ''}
        if self.deterministic_arrival_proof:
            result.update({
                'side': side,
                'shuttle': self.arrival_shuttle,
                'target_slot': target_slot,
                'target_sensor': self.arrival_sensor,
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
            'matched_by': 'interior_entry_sensor_plus_bounded_travel_time',
            'target_segment': target_segment,
            'target_s_m': target_s_m,
            'observed_segment': (
                self.clearance_observed_segment or target_segment
            ),
            'observed_s_m': target_s_m,
            'absolute_error_m': 0.0,
            'entry_sensor': entry_sensor,
            'entry_sensor_identity_confirmed': True,
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


def test_executes_only_first_atomic_step_from_validated_plan():
    state = _base_state('inspect-state')
    provider = SequenceObservedStateProvider([state, state])
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
    assert [command['action'] for command in transport.commands] == ['DONE']


def test_retries_recoverable_unknown_before_planning():
    unknown = _with_fact(
        _base_state('unknown-slot'),
        'right:slot:3',
        'occupancy',
        status='unknown',
    )
    known = _base_state('known-slot')
    provider = SequenceObservedStateProvider([unknown, known, known])
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
    assert [command['action'] for command in transport.commands] == ['DONE']


def test_postcondition_mismatch_replans_before_next_command():
    source = _base_state('r1-source')
    target = _state_with_r1_at_slot3('r1-target')
    provider = SequenceObservedStateProvider([source, source, source, target])
    planner = SequencePlanner([
        [
            'move_shuttle_to_slot right right_shuttle_1 right_slot_1 right_slot_3',
            'finish_candidate_task right_shuttle_1 staubli',
        ],
        ['move_shuttle_to_slot right right_shuttle_1 right_slot_1 right_slot_3'],
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
        'move_shuttle_to_slot right right_shuttle_1 right_slot_1 right_slot_3',
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
    provider = SequenceObservedStateProvider([source, source, source, source])
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
    # makes R2 a route blocker. This establishes that the first planning
    # problem below can be direct only if the new executive synchronizes its
    # provider-owned certificate before building that problem.
    uncertified = build_pddl_problem_from_observed_state_task_goal(
        cleared,
        _r4_goal(),
    )
    assert uncertified.provenance[
        'target_blocker_clearance_plan'
    ]['required'] is True

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
        visual_segment='A34E',
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
    transport = RecordingTransport(clearance_observed_segment='A34E')

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
    assert (
        clearance_postcondition.details['visual_segment_disagreement']
        is True
    )
    assert clearance_postcondition.details['model_prediction_replaced'] is False
    assert planner.problems[0].goal_text == (
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
    assert visual_wait[1]['target_s_m'] == 0.7083
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
    assert restoration[1]['stoppers'] == {
        device: '0'
        for device in ('A1', 'A2', 'A3', 'A4')
    }
    assert any(
        wait[0] == 'fresh_visual_observation'
        for wait in transport.waits
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
