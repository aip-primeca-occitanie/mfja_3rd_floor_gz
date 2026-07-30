#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_closed_loop_executive import ClosedLoopExecutive
from room_315_closed_loop_executive import ClosedLoopExecutiveConfig
from room_315_contracts import ObservedState
from room_315_contracts import TaskGoal
from room_315_observed_state_provider import ObservedStateProvider
from room_315_pddl_scenario_generator import BasePlannerBackend
from room_315_pddl_scenario_generator import ScenarioSpec
from room_315_pddl_scenario_generator import ScenarioTransport
from room_315_pddl_scenario_generator import _observed_state_from_scenario_spec


class SequenceObservedStateProvider(ObservedStateProvider):
    def __init__(self, states: list[ObservedState]) -> None:
        self.states = list(states)
        self.calls = 0

    def observe(self, *, timestamp: float | None = None) -> ObservedState:
        state = self.states[min(self.calls, len(self.states) - 1)]
        self.calls += 1
        return state


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
    def __init__(self) -> None:
        self.commands: list[dict] = []
        self.waits: list[tuple[str, dict]] = []
        self._decision_count = 0

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
        return {'arrived': True, 'reason': ''}


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
