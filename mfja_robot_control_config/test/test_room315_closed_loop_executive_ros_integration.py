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


class ReplayStateProvider(ObservedStateProvider):
    def __init__(self, states: list[ObservedState]) -> None:
        self.states = list(states)
        self.calls = 0

    def observe(self, *, timestamp: float | None = None) -> ObservedState:
        state = self.states[min(self.calls, len(self.states) - 1)]
        self.calls += 1
        return state


class ReplayPlanner(BasePlannerBackend):
    def __init__(self) -> None:
        self.calls = 0

    def plan(self, goal_or_problem, *, speed: float) -> list[str]:
        self.calls += 1
        if self.calls == 1:
            return [
                'prepare_switches right switch=A1 state=INTERIOR',
                'open_stoppers right',
            ]
        return ['inspect_state room315_system']


class RosSupervisorLoopTransport(ScenarioTransport):
    """Deterministic stand-in for the ROS supervisor command/decision loop."""

    def __init__(self) -> None:
        self.commands: list[dict] = []
        self.decision_count = 0
        self.switch_waits: list[dict] = []

    def publish_command(self, command: dict) -> None:
        self.commands.append(dict(command))
        if command.get('action') != 'stop_all':
            self.decision_count += 1

    def supervisor_decision_count(self) -> int:
        return self.decision_count

    def wait_for_supervisor_decision(
        self,
        *,
        previous_count: int,
        timeout_s: float,
    ) -> dict | None:
        assert self.decision_count == previous_count + 1
        return {'accepted': True, 'status': 'accepted', 'source': 'supervisor'}

    def wait_for_switch_state(self, *, side: str, switches: dict, timeout_s: float) -> dict:
        self.switch_waits.append({'side': side, 'switches': dict(switches)})
        return {'ready': True, 'source': 'trusted_device'}


def _state(state_id: str, *, a1: str = 'EXTERIOR') -> ObservedState:
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
    facts = [
        replace(fact, value=a1)
        if fact.subject == 'right:switch:A1' and fact.predicate == 'state'
        else fact
        for fact in state.fused_planner_state
    ]
    return replace(state, state_id=state_id, fused_planner_state=facts)


def test_closed_loop_uses_supervisor_decision_and_wait_hooks_between_plans():
    provider = ReplayStateProvider([
        _state('before-switch', a1='EXTERIOR'),
        _state('after-switch', a1='INTERIOR'),
        _state('before-inspect', a1='INTERIOR'),
    ])
    planner = ReplayPlanner()
    transport = RosSupervisorLoopTransport()
    goal = TaskGoal(
        goal_id='inspect-after-switch',
        description='Inspect Room 315 after a supervised switch command',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'inspection',
            'side': 'right',
            'inspection_subject': 'room315_system',
        },
    )

    result = ClosedLoopExecutive(
        observed_state_provider=provider,
        planner=planner,
        transport=transport,
        config=ClosedLoopExecutiveConfig(max_steps=4),
    ).run(goal)

    assert result.succeeded
    assert planner.calls == 2
    assert [command['action'] for command in transport.commands] == ['switches', 'DONE']
    assert transport.commands[0]['closed_loop_executive']['plan_length'] == 2
    assert transport.commands[0]['closed_loop_executive']['symbolic_step'].startswith(
        'prepare_switches'
    )
    assert transport.switch_waits == [
        {'side': 'right', 'switches': {'A1': 'INTERIOR'}},
    ]
