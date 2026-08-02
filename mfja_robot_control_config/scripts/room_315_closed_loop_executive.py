#!/usr/bin/env python3
"""Closed-loop Room 315 symbolic executive.

This module is the production loop around the existing boundaries:
ObservedState -> PDDL problem -> PlanSys2 plan -> one translated primitive
command -> supervisor -> re-observe. It intentionally executes only the first
atomic symbolic step from each validated plan and then replans from fresh state.
"""

from __future__ import annotations

import re
import sys
import time
from dataclasses import dataclass
from dataclasses import field
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_contracts import ObservedFact
from room_315_contracts import ObservedState
from room_315_contracts import TaskGoal
from room_315_multi_shuttle import DEVICE_NAMES
from room_315_multi_shuttle import normalize_shuttle_ref
from room_315_observed_state_provider import ObservedStateProvider
from room_315_pddl_plan_translator import PddlPlanStep
from room_315_pddl_plan_translator import TranslatedPlanStep
from room_315_pddl_plan_translator import translate_plan
from room_315_pddl_plan_translator import translate_step
from room_315_pddl_scenario_generator import BasePlannerBackend
from room_315_pddl_scenario_generator import INTERIOR_HOLDING_BRANCH_BY_GATE
from room_315_pddl_scenario_generator import INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M
from room_315_pddl_scenario_generator import INTERIOR_LOOP_ENTRY_SENSOR_BY_SIDE_AND_GATE
from room_315_pddl_scenario_generator import PddlProblemBuildError
from room_315_pddl_scenario_generator import Room315PddlProblem
from room_315_pddl_scenario_generator import RUNTIME_SYMBOLIC_ACTIONS
from room_315_pddl_scenario_generator import ScenarioTransport
from room_315_pddl_scenario_generator import SLOT_SENSOR_BY_SIDE_AND_SLOT
from room_315_pddl_scenario_generator import SLOT_STATION_BY_SIDE_AND_SLOT
from room_315_pddl_scenario_generator import build_clearance_pause_problem
from room_315_pddl_scenario_generator import build_first_blocker_clearance_problem
from room_315_pddl_scenario_generator import build_intermediate_selected_advance_problem
from room_315_pddl_scenario_generator import build_pddl_problem_from_observed_state_task_goal


TERMINAL_SUCCESS_STEPS = {'finish_task', 'finish_candidate_task', 'inspect_state'}
RECOVERABLE_STATE_STATUSES = {'unknown', 'stale', 'conflicting'}
STOPPED_MOTION_VALUES = {'STOPPED', 'OFF', 'IDLE', 'HALTED', 'stopped', 'off', 'idle', 'halted'}
CLEARANCE_BEGIN_ACTIONS = frozenset({
    'begin_route_clearance',
    'begin_segment_route_clearance',
})
CLEARANCE_RELOCATE_ACTIONS = frozenset({
    'relocate_blocker_to_interior',
    'relocate_segment_blocker_to_interior',
    'stage_selected_to_interior',
    'stage_selected_segment_to_interior',
})
CLEARANCE_FINISH_ACTIONS = frozenset({
    'finish_route_clearance',
    'finish_segment_route_clearance',
})
CLEARANCE_PHASE_ACTIONS = frozenset({
    *CLEARANCE_BEGIN_ACTIONS,
    *CLEARANCE_RELOCATE_ACTIONS,
    *CLEARANCE_FINISH_ACTIONS,
    'pause_route_clearance',
})


class _FreshObservationUnavailable(RuntimeError):
    """A recovery retry could not obtain a newer accepted visual state."""


def _pddl_contract_symbol(value: Any) -> str:
    text = re.sub(r'[^a-zA-Z0-9_]+', '_', str(value or '').strip())
    return re.sub(r'_+', '_', text).strip('_').casefold()


def _problem_has_atom(
    problem_text: str,
    predicate: str,
    *arguments: str,
) -> bool:
    """Return whether an exact grounded atom exists in the frozen problem."""

    tokens = (predicate, *arguments)
    expression = r'\(\s*' + r'\s+'.join(
        re.escape(_pddl_contract_symbol(token)) for token in tokens
    ) + r'\s*\)'
    return re.search(expression, _problem_init_section(problem_text)) is not None


def _problem_init_section(problem_text: str) -> str:
    """Extract the balanced PDDL ``:init`` form for applicability checks."""

    text = str(problem_text or '').casefold()
    start = text.find('(:init')
    if start < 0:
        return ''
    depth = 0
    for index in range(start, len(text)):
        if text[index] == '(':
            depth += 1
        elif text[index] == ')':
            depth -= 1
            if depth == 0:
                return text[start:index + 1]
    return ''


def _problem_numeric_value(
    problem_text: str,
    function_name: str,
    *arguments: str,
) -> float | None:
    function = r'\(\s*' + r'\s+'.join(
        re.escape(_pddl_contract_symbol(token))
        for token in (function_name, *arguments)
    ) + r'\s*\)'
    match = re.search(
        r'\(\s*=\s*' + function + r'\s+([-+]?[0-9]+(?:\.[0-9]+)?)\s*\)',
        _problem_init_section(problem_text),
    )
    return float(match.group(1)) if match else None


def _frozen_precondition_error(
    step: PddlPlanStep,
    problem: Room315PddlProblem,
) -> str:
    """Recheck every state-bearing first action before physical execution.

    PlanSys2 normally returns an applicable action.  This independent boundary
    prevents malformed, stale, mocked, or compromised planner output from
    reaching the supervisor merely because it has a recognized action name.
    """

    args = tuple(_pddl_contract_symbol(value) for value in step.args)
    text = problem.problem_text
    side = problem.side

    required_atoms: list[tuple[str, ...]] = [('validated_state',)]
    pending_expectation: str | None = None

    if step.name == 'prepare_switches':
        if len(args) != 4:
            return 'prepare_switches_arity'
        planned_side, source, target, switch_group = args
        required_atoms.extend([
            ('connected', planned_side, source, target),
            ('switch_group_on_side', switch_group, planned_side),
            ('normal_route', planned_side),
        ])
    elif step.name == 'open_stoppers':
        if len(args) != 4:
            return 'open_stoppers_arity'
        planned_side, source, target, stopper_group = args
        required_atoms.extend([
            ('connected', planned_side, source, target),
            ('stopper_group_on_side', stopper_group, planned_side),
            ('switches_ready', planned_side),
            ('normal_route', planned_side),
        ])
    elif step.name == 'move_shuttle_to_slot':
        if len(args) != 6:
            return 'move_shuttle_to_slot_arity'
        shuttle, planned_side, source, target, source_slot, target_slot = args
        required_atoms.extend([
            ('shuttle_on_side', shuttle, planned_side),
            ('slot_on_side', source_slot, planned_side),
            ('slot_on_side', target_slot, planned_side),
            ('slot_at_station', source_slot, source),
            ('slot_at_station', target_slot, target),
            ('shuttle_at', shuttle, source),
            ('shuttle_at_slot', shuttle, source_slot),
            ('slot_occupied_by', source_slot, shuttle),
            ('connected', planned_side, source, target),
            ('path_ready', planned_side, source, target),
            ('switches_ready', planned_side),
            ('stoppers_open', planned_side),
            ('normal_route', planned_side),
            ('route_clear_between', source_slot, target_slot),
            ('slot_free', target_slot),
        ])
        pending_expectation = 'zero'
    elif step.name == 'prepare_topology_route':
        if len(args) != 5:
            return 'prepare_topology_route_arity'
        shuttle, planned_side, source_block, target_slot, switch_group = args
        required_atoms.extend([
            ('shuttle_on_side', shuttle, planned_side),
            ('segment_only_location', shuttle),
            ('shuttle_at_topology_block', shuttle, source_block),
            ('block_on_side', source_block, planned_side),
            ('slot_on_side', target_slot, planned_side),
            ('topology_route_available', shuttle, source_block, target_slot),
            ('topology_route_clear', shuttle, source_block, target_slot),
            ('switch_group_on_side', switch_group, planned_side),
            ('slot_free', target_slot),
            ('normal_route', planned_side),
        ])
        pending_expectation = 'zero'
    elif step.name == 'move_shuttle_from_segment_to_slot':
        if len(args) != 5:
            return 'move_shuttle_from_segment_to_slot_arity'
        shuttle, planned_side, source_block, target, target_slot = args
        required_atoms.extend([
            ('shuttle_on_side', shuttle, planned_side),
            ('segment_only_location', shuttle),
            ('shuttle_at_topology_block', shuttle, source_block),
            ('block_on_side', source_block, planned_side),
            ('slot_on_side', target_slot, planned_side),
            ('slot_at_station', target_slot, target),
            ('topology_route_available', shuttle, source_block, target_slot),
            ('topology_route_clear', shuttle, source_block, target_slot),
            ('topology_route_configured', shuttle, source_block, target_slot),
            ('slot_free', target_slot),
        ])
        pending_expectation = 'zero'
    elif step.name == 'prepare_slot_topology_route':
        if len(args) != 6:
            return 'prepare_slot_topology_route_arity'
        (
            shuttle,
            planned_side,
            source_slot,
            source_block,
            target_slot,
            switch_group,
        ) = args
        required_atoms.extend([
            ('shuttle_on_side', shuttle, planned_side),
            ('slot_on_side', source_slot, planned_side),
            ('slot_on_side', target_slot, planned_side),
            ('shuttle_at_slot', shuttle, source_slot),
            ('slot_occupied_by', source_slot, shuttle),
            ('shuttle_at_topology_block', shuttle, source_block),
            ('block_on_side', source_block, planned_side),
            ('topology_route_available', shuttle, source_block, target_slot),
            ('topology_route_clear', shuttle, source_block, target_slot),
            ('switch_group_on_side', switch_group, planned_side),
            ('slot_free', target_slot),
            ('normal_route', planned_side),
        ])
        pending_expectation = 'zero'
    elif step.name == 'move_shuttle_via_topology_to_slot':
        if len(args) != 7:
            return 'move_shuttle_via_topology_to_slot_arity'
        (
            shuttle,
            planned_side,
            source_slot,
            source_block,
            source,
            target,
            target_slot,
        ) = args
        required_atoms.extend([
            ('shuttle_on_side', shuttle, planned_side),
            ('slot_on_side', source_slot, planned_side),
            ('slot_on_side', target_slot, planned_side),
            ('slot_at_station', source_slot, source),
            ('slot_at_station', target_slot, target),
            ('shuttle_at_slot', shuttle, source_slot),
            ('slot_occupied_by', source_slot, shuttle),
            ('shuttle_at', shuttle, source),
            ('shuttle_at_topology_block', shuttle, source_block),
            ('block_on_side', source_block, planned_side),
            ('topology_route_available', shuttle, source_block, target_slot),
            ('topology_route_clear', shuttle, source_block, target_slot),
            ('topology_route_configured', shuttle, source_block, target_slot),
            ('slot_free', target_slot),
        ])
        pending_expectation = 'zero'
    elif step.name == 'begin_route_clearance':
        if len(args) != 4:
            return 'begin_route_clearance_arity'
        selected, planned_side, source_slot, target_slot = args
        required_atoms.extend([
            ('normal_route', planned_side),
            ('shuttle_on_side', selected, planned_side),
            ('goal_candidate', selected),
            ('shuttle_at_slot', selected, source_slot),
            ('target_slot_for_goal', target_slot),
        ])
        pending_expectation = 'positive'
    elif step.name == 'relocate_blocker_to_interior':
        if len(args) != 5:
            return 'relocate_blocker_to_interior_arity'
        blocker, selected, planned_side, source_slot, target_slot = args
        required_atoms.extend([
            ('shuttle_on_side', blocker, planned_side),
            ('shuttle_on_side', selected, planned_side),
            ('shuttle_at_slot', selected, source_slot),
            ('target_slot_for_goal', target_slot),
            ('clearance_mode', planned_side),
            ('clearance_precedes', blocker, selected),
            ('route_blocked_by', source_slot, target_slot, blocker),
            ('clearance_destination_ready', blocker),
            ('interior_entry_route_clear', blocker),
        ])
        pending_expectation = 'positive'
        order = _problem_numeric_value(text, 'clearance_order', blocker)
        cursor = _problem_numeric_value(text, 'clearance_cursor', planned_side)
        if order is None or cursor is None or order != cursor:
            return 'clearance_order_cursor_mismatch'
    elif step.name == 'stage_selected_to_interior':
        if len(args) != 4:
            return 'stage_selected_to_interior_arity'
        selected, planned_side, source_slot, target_slot = args
        required_atoms.extend([
            ('shuttle_on_side', selected, planned_side),
            ('goal_candidate', selected),
            ('shuttle_at_slot', selected, source_slot),
            ('target_slot_for_goal', target_slot),
            ('clearance_mode', planned_side),
            ('clearance_destination_ready', selected),
            ('interior_entry_route_clear', selected),
        ])
        pending_expectation = 'positive'
        order = _problem_numeric_value(text, 'clearance_order', selected)
        cursor = _problem_numeric_value(text, 'clearance_cursor', planned_side)
        if order is None or cursor is None or order != cursor:
            return 'clearance_order_cursor_mismatch'
    elif step.name == 'finish_route_clearance':
        if len(args) != 4:
            return 'finish_route_clearance_arity'
        selected, planned_side, source_slot, target_slot = args
        required_atoms.extend([
            ('clearance_mode', planned_side),
            ('shuttle_on_side', selected, planned_side),
            ('goal_candidate', selected),
            ('shuttle_at_slot', selected, source_slot),
            ('target_slot_for_goal', target_slot),
            ('clearance_pause_safe', planned_side),
        ])
        pending_expectation = 'zero'
    elif step.name == 'begin_segment_route_clearance':
        if len(args) != 4:
            return 'begin_segment_route_clearance_arity'
        selected, planned_side, source_block, target_slot = args
        required_atoms.extend([
            ('normal_route', planned_side),
            ('shuttle_on_side', selected, planned_side),
            ('goal_candidate', selected),
            ('segment_only_location', selected),
            ('shuttle_at_topology_block', selected, source_block),
            ('block_on_side', source_block, planned_side),
            ('slot_on_side', target_slot, planned_side),
            ('target_slot_for_goal', target_slot),
            ('topology_route_available', selected, source_block, target_slot),
        ])
        pending_expectation = 'positive'
    elif step.name == 'relocate_segment_blocker_to_interior':
        if len(args) != 5:
            return 'relocate_segment_blocker_to_interior_arity'
        blocker, selected, planned_side, source_block, target_slot = args
        required_atoms.extend([
            ('shuttle_on_side', blocker, planned_side),
            ('shuttle_on_side', selected, planned_side),
            ('segment_only_location', selected),
            ('shuttle_at_topology_block', selected, source_block),
            ('block_on_side', source_block, planned_side),
            ('slot_on_side', target_slot, planned_side),
            ('target_slot_for_goal', target_slot),
            ('clearance_mode', planned_side),
            ('clearance_precedes', blocker, selected),
            (
                'topology_route_blocked_by',
                selected,
                source_block,
                target_slot,
                blocker,
            ),
            ('clearance_destination_ready', blocker),
            ('interior_entry_route_clear', blocker),
        ])
        pending_expectation = 'positive'
        order = _problem_numeric_value(text, 'clearance_order', blocker)
        cursor = _problem_numeric_value(text, 'clearance_cursor', planned_side)
        if order is None or cursor is None or order != cursor:
            return 'clearance_order_cursor_mismatch'
    elif step.name == 'stage_selected_segment_to_interior':
        if len(args) != 4:
            return 'stage_selected_segment_to_interior_arity'
        selected, planned_side, source_block, target_slot = args
        required_atoms.extend([
            ('shuttle_on_side', selected, planned_side),
            ('goal_candidate', selected),
            ('segment_only_location', selected),
            ('shuttle_at_topology_block', selected, source_block),
            ('block_on_side', source_block, planned_side),
            ('target_slot_for_goal', target_slot),
            ('clearance_mode', planned_side),
            ('clearance_destination_ready', selected),
            ('interior_entry_route_clear', selected),
        ])
        pending_expectation = 'positive'
        order = _problem_numeric_value(text, 'clearance_order', selected)
        cursor = _problem_numeric_value(text, 'clearance_cursor', planned_side)
        if order is None or cursor is None or order != cursor:
            return 'clearance_order_cursor_mismatch'
    elif step.name == 'finish_segment_route_clearance':
        if len(args) != 4:
            return 'finish_segment_route_clearance_arity'
        selected, planned_side, source_block, target_slot = args
        required_atoms.extend([
            ('clearance_mode', planned_side),
            ('shuttle_on_side', selected, planned_side),
            ('goal_candidate', selected),
            ('segment_only_location', selected),
            ('shuttle_at_topology_block', selected, source_block),
            ('block_on_side', source_block, planned_side),
            ('slot_on_side', target_slot, planned_side),
            ('target_slot_for_goal', target_slot),
            ('topology_route_available', selected, source_block, target_slot),
            ('clearance_pause_safe', planned_side),
        ])
        pending_expectation = 'zero'
    elif step.name == 'pause_route_clearance':
        if len(args) != 1:
            return 'pause_route_clearance_arity'
        planned_side = args[0]
        required_atoms.extend([
            ('clearance_mode', planned_side),
            ('clearance_pause_safe', planned_side),
        ])
        pending_expectation = 'positive'
    elif step.name == 'restore_normal_route':
        if len(args) != 3:
            return 'restore_normal_route_arity'
        planned_side, source, target = args
        required_atoms.extend([
            ('connected', planned_side, source, target),
            ('route_reconfiguration_required', planned_side),
            ('route_reconfiguration_safe', planned_side),
        ])
        pending_expectation = 'zero'
    elif step.name == 'stop_shuttle':
        if len(args) != 4:
            return 'stop_shuttle_arity'
        shuttle, planned_side, source, target = args
        required_atoms.extend([
            ('shuttle_on_side', shuttle, planned_side),
            ('shuttle_at', shuttle, target),
            ('path_ready', planned_side, source, target),
        ])
    else:
        return ''

    if 'planned_side' in locals() and planned_side != side:
        return f'action_side_mismatch:problem={side},plan={planned_side}'
    for atom in required_atoms:
        if not _problem_has_atom(text, *atom):
            return 'missing_frozen_precondition:' + ':'.join(atom)
    if pending_expectation:
        pending = _problem_numeric_value(text, 'pending_clearances', side)
        if pending is None:
            return 'missing_pending_clearances'
        if pending_expectation == 'zero' and pending != 0:
            return f'pending_clearances_not_zero:{pending:g}'
        if pending_expectation == 'positive' and pending <= 0:
            return f'pending_clearances_not_positive:{pending:g}'
    return ''


@dataclass(frozen=True)
class ClosedLoopExecutiveConfig:
    """Runtime limits for the closed-loop executive."""

    speed_mps: float = 0.3
    max_steps: int = 32
    max_replans: int = 8
    max_unknown_retries: int = 3
    supervisor_timeout_s: float = 10.0
    effect_timeout_s: float = 20.0
    clearance_effect_timeout_s: float = 60.0
    planning_timeout_s: float = 10.0
    safe_abort_command: str = 'stop_all'


@dataclass(frozen=True)
class PostconditionCheck:
    status: str
    reason: str
    details: dict[str, Any] = field(default_factory=dict)

    @property
    def satisfied(self) -> bool:
        return self.status == 'satisfied'

    @property
    def recoverable(self) -> bool:
        return self.status in {'mismatch', 'unknown', 'obstacle'}


@dataclass(frozen=True)
class ClosedLoopStepRecord:
    step_index: int
    observed_state_id: str
    problem_name: str
    plan_length: int
    symbolic_step: str
    primitive: str
    supervisor_status: str
    postcondition: PostconditionCheck
    occupancy_changed: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            'step_index': self.step_index,
            'observed_state_id': self.observed_state_id,
            'problem_name': self.problem_name,
            'plan_length': self.plan_length,
            'symbolic_step': self.symbolic_step,
            'primitive': self.primitive,
            'supervisor_status': self.supervisor_status,
            'postcondition': {
                'status': self.postcondition.status,
                'reason': self.postcondition.reason,
                'details': dict(self.postcondition.details),
            },
            'occupancy_changed': self.occupancy_changed,
        }


@dataclass(frozen=True)
class ClosedLoopExecutiveResult:
    status: str
    reason: str
    executed_steps: tuple[ClosedLoopStepRecord, ...]
    plan_attempts: int
    observations: int
    replans: int
    unknown_retries: int
    safe_abort_sent: bool
    final_state_id: str = ''
    replan_reasons: tuple[str, ...] = ()

    @property
    def succeeded(self) -> bool:
        return self.status == 'succeeded'

    def to_dict(self) -> dict[str, Any]:
        return {
            'status': self.status,
            'reason': self.reason,
            'executed_steps': [step.to_dict() for step in self.executed_steps],
            'plan_attempts': self.plan_attempts,
            'observations': self.observations,
            'replans': self.replans,
            'unknown_retries': self.unknown_retries,
            'safe_abort_sent': self.safe_abort_sent,
            'final_state_id': self.final_state_id,
            'replan_reasons': list(self.replan_reasons),
        }


class ClosedLoopExecutive:
    """Observe, plan one PlanSys2 step, execute through supervisor, re-observe."""

    def __init__(
        self,
        *,
        observed_state_provider: ObservedStateProvider,
        planner: BasePlannerBackend,
        transport: ScenarioTransport,
        config: ClosedLoopExecutiveConfig | None = None,
    ) -> None:
        self.observed_state_provider = observed_state_provider
        self.planner = planner
        self.transport = transport
        self.config = config or ClosedLoopExecutiveConfig()
        self._observations = 0
        self._plan_attempts = 0
        self._safe_abort_sent = False
        self._records: list[ClosedLoopStepRecord] = []
        self._replan_reasons: list[str] = []
        self._runtime_clearance_certificates: dict[str, dict[str, Any]] = {}
        # Symbolic effects are otherwise lost when the problem is rebuilt
        # after each atomic action. Keep an executor-owned safety latch as a
        # second guard around the physically observed clearance mode.
        self._route_clearance_active_side = ''

    def run(self, task_goal: TaskGoal) -> ClosedLoopExecutiveResult:
        """Execute a TaskGoal by replanning after every atomic supervised step."""

        if not isinstance(task_goal, TaskGoal):
            raise TypeError('task_goal must be a TaskGoal contract')

        unknown_retries = 0
        replans = 0
        final_state_id = ''
        require_fresh_after_state_id = ''

        for step_index in range(self.config.max_steps):
            try:
                observed_state = self._observe(
                    after_state_id=require_fresh_after_state_id,
                )
            except _FreshObservationUnavailable as exc:
                self._safe_abort('persistent_uncertainty')
                return self._result(
                    status='aborted',
                    reason=f'persistent_uncertainty:{exc}',
                    replans=replans,
                    unknown_retries=unknown_retries,
                    final_state_id=final_state_id,
                )
            require_fresh_after_state_id = ''
            final_state_id = observed_state.state_id
            self._refresh_runtime_clearance_certificates()

            if self._task_goal_satisfied(observed_state, task_goal):
                return self._result(
                    status='succeeded',
                    reason=(
                        'task_goal_satisfied'
                        if self._records
                        else 'task_goal_already_satisfied'
                    ),
                    replans=replans,
                    unknown_retries=unknown_retries,
                    final_state_id=final_state_id,
                )

            obstacle_reason = self._obstacle_reason(observed_state)
            if obstacle_reason:
                replans += 1
                self._replan_reasons.append(f'obstacle:{obstacle_reason}')
                if replans > self.config.max_replans:
                    self._safe_abort('persistent_obstacle')
                    return self._result(
                        status='aborted',
                        reason='persistent_obstacle',
                        replans=replans,
                        unknown_retries=unknown_retries,
                        final_state_id=final_state_id,
                    )
                continue

            try:
                problem = build_pddl_problem_from_observed_state_task_goal(
                    observed_state,
                    task_goal,
                    problem_name=f'closed-loop-{task_goal.goal_id}-{step_index}',
                    runtime_clearance_certificates=(
                        self._runtime_clearance_certificates
                    ),
                )
                problem = self._next_planning_problem(problem)
            except PddlProblemBuildError as exc:
                unknown_retries += 1
                self._replan_reasons.append(f'recoverable_unknown:{exc}')
                if unknown_retries > self.config.max_unknown_retries:
                    self._safe_abort('persistent_uncertainty')
                    return self._result(
                        status='aborted',
                        reason=f'persistent_uncertainty:{exc}',
                        replans=replans,
                        unknown_retries=unknown_retries,
                        final_state_id=final_state_id,
                    )
                require_fresh_after_state_id = observed_state.state_id
                continue

            try:
                plan, translated_plan = self._request_and_validate_plan(problem)
            except Exception as exc:  # noqa: BLE001 - fail closed at execution boundary
                self._safe_abort('planner_failure')
                return self._result(
                    status='aborted',
                    reason=f'planner_failure:{exc}',
                    replans=replans,
                    unknown_retries=unknown_retries,
                    final_state_id=final_state_id,
                )

            if not plan:
                self._safe_abort('empty_plan')
                return self._result(
                    status='aborted',
                    reason='empty_plan',
                    replans=replans,
                    unknown_retries=unknown_retries,
                    final_state_id=final_state_id,
                )

            first_step = _as_plan_step(plan[0])
            translated_step = translated_plan[0]
            before_occupancy = _occupancy_snapshot(observed_state)

            self._resume_clearance_phase_from_problem(
                problem=problem,
                first_step=first_step,
            )

            phase_error = self._clearance_phase_plan_error(
                first_step=first_step,
                side=problem.side,
            )
            if phase_error:
                self._safe_abort('clearance_phase_plan_violation')
                return self._result(
                    status='aborted',
                    reason=f'clearance_phase_plan_violation:{phase_error}',
                    replans=replans,
                    unknown_retries=unknown_retries,
                    final_state_id=final_state_id,
                )

            action_contract_error = self._first_action_contract_error(
                first_step=first_step,
                translated_step=translated_step,
                problem=problem,
                task_goal=task_goal,
            )
            if action_contract_error:
                detailed_reason = (
                    'planned_action_contract_violation:'
                    f'{action_contract_error}'
                )
                abort_reason = detailed_reason
                if first_step.name == 'restore_normal_route':
                    abort_reason = (
                        'route_normalization_rejected:'
                        f'{detailed_reason}'
                    )
                self._safe_abort(abort_reason)
                return self._result(
                    status='aborted',
                    reason=detailed_reason,
                    replans=replans,
                    unknown_retries=unknown_retries,
                    final_state_id=final_state_id,
                )

            if first_step.name in CLEARANCE_BEGIN_ACTIONS:
                supervisor_status, wait_check = self._enter_route_clearance_mode(
                    side=problem.side,
                    problem=problem,
                    plan_length=len(plan),
                    step_index=step_index,
                    symbolic_step=_step_text(first_step),
                )
            elif first_step.name in CLEARANCE_RELOCATE_ACTIONS:
                supervisor_status, wait_check = self._execute_interior_clearance(
                    translated_step=translated_step,
                    problem=problem,
                    plan_length=len(plan),
                    step_index=step_index,
                )
            elif first_step.name in CLEARANCE_FINISH_ACTIONS:
                supervisor_status, wait_check = self._restore_normal_route_after_clearance(
                    side=problem.side,
                    common=self._clearance_phase_metadata(
                        mode='finish_route_clearance_after_all_blockers',
                        problem=problem,
                        plan_length=len(plan),
                        step_index=step_index,
                        symbolic_step=_step_text(first_step),
                    ),
                )
            elif first_step.name == 'pause_route_clearance':
                supervisor_status, wait_check = self._restore_normal_route_after_clearance(
                    side=problem.side,
                    common=self._clearance_phase_metadata(
                        mode='pause_clearance_for_exterior_choreography',
                        problem=problem,
                        plan_length=len(plan),
                        step_index=step_index,
                        symbolic_step=_step_text(first_step),
                    ),
                    restore_mode=(
                        'pause_clearance_after_interior_capacity_exhausted'
                    ),
                    success_reason=(
                        'clearance_paused_for_exterior_slot_choreography'
                    ),
                )
            elif first_step.name == 'restore_normal_route':
                supervisor_status, wait_check = (
                    self._restore_normal_route_from_mixed_topology(
                        problem=problem,
                        translated_step=translated_step,
                        plan_length=len(plan),
                        step_index=step_index,
                        symbolic_step=_step_text(first_step),
                    )
                )
            elif first_step.name in {
                'prepare_topology_route',
                'prepare_slot_topology_route',
            }:
                supervisor_status, wait_check = self._prepare_topology_route(
                    translated_step=translated_step,
                    problem=problem,
                    plan_length=len(plan),
                    step_index=step_index,
                )
            elif first_step.name in TERMINAL_SUCCESS_STEPS:
                # DONE is a symbolic completion marker, not a physical
                # supervisor command.  The real supervisor intentionally has
                # no DONE actuator action.  Completion is proved from the
                # fresh observation below.
                supervisor_status = 'accepted'
                wait_check = PostconditionCheck(
                    status='satisfied',
                    reason='non_actuating_terminal_marker',
                    details={'supervisor_command_published': False},
                )
            else:
                supervisor_status = self._execute_first_atomic_step(
                    translated_step=translated_step,
                    problem=problem,
                    task_goal=task_goal,
                    plan_length=len(plan),
                    step_index=step_index,
                )
                wait_check = PostconditionCheck(
                    status='satisfied',
                    reason='execution_accepted',
                )
            if supervisor_status != 'accepted':
                rejection_reason = f'supervisor_{supervisor_status}'
                if (
                    first_step.name in {
                        'restore_normal_route',
                        'pause_route_clearance',
                    }
                    and wait_check.reason
                ):
                    rejection_reason = (
                        'route_normalization_rejected:'
                        f'{wait_check.reason}'
                    )
                self._safe_abort(rejection_reason)
                return self._result(
                    status='aborted',
                    reason=rejection_reason,
                    replans=replans,
                    unknown_retries=unknown_retries,
                    final_state_id=final_state_id,
                )

            if first_step.name not in {
                *CLEARANCE_PHASE_ACTIONS,
                'restore_normal_route',
                'prepare_topology_route',
                'prepare_slot_topology_route',
                *TERMINAL_SUCCESS_STEPS,
            }:
                wait_check = self._wait_for_expected_effects(
                    step=first_step,
                    translated_step=translated_step,
                    task_goal=task_goal,
                    problem=problem,
                )
            if wait_check.status == 'timed_out':
                self._safe_abort(f'effect_timeout:{wait_check.reason}')
                return self._result(
                    status='aborted',
                    reason=f'effect_timeout:{wait_check.reason}',
                    replans=replans,
                    unknown_retries=unknown_retries,
                    final_state_id=final_state_id,
                )

            if first_step.name in CLEARANCE_BEGIN_ACTIONS and wait_check.satisfied:
                self._route_clearance_active_side = problem.side
            elif (
                first_step.name in CLEARANCE_FINISH_ACTIONS | {
                    'pause_route_clearance',
                }
                and wait_check.satisfied
            ):
                self._route_clearance_active_side = ''

            clearance_certificate: dict[str, Any] | None = None
            if first_step.name in CLEARANCE_RELOCATE_ACTIONS:
                (
                    clearance_certificate,
                    clearance_proof_failure,
                ) = self._interior_clearance_certificate_result(
                    wait_check=wait_check,
                    problem=problem,
                    translated_step=translated_step,
                )
                if clearance_certificate is None:
                    self._safe_abort('clearance_proof_incomplete')
                    return self._result(
                        status='aborted',
                        reason=(
                            'clearance_proof_incomplete:'
                            f'{clearance_proof_failure}'
                        ),
                        replans=replans,
                        unknown_retries=unknown_retries,
                        final_state_id=final_state_id,
                    )
                certificate_identity = str(
                    clearance_certificate['identity']
                )
                self._runtime_clearance_certificates[
                    certificate_identity
                ] = clearance_certificate
                register = getattr(
                    self.observed_state_provider,
                    'set_runtime_clearance_certificate',
                    None,
                )
                if callable(register):
                    register(clearance_certificate)

            after_state = self._observe()
            final_state_id = after_state.state_id
            after_occupancy = _occupancy_snapshot(after_state)
            occupancy_changed = before_occupancy != after_occupancy
            if first_step.name in CLEARANCE_RELOCATE_ACTIONS:
                postcondition = self._verify_interior_clearance(
                    after_state=after_state,
                    problem=problem,
                    translated_step=translated_step,
                    clearance_certificate=clearance_certificate,
                )
            elif first_step.name in {
                *CLEARANCE_BEGIN_ACTIONS,
                *CLEARANCE_FINISH_ACTIONS,
                'pause_route_clearance',
                'restore_normal_route',
                'prepare_topology_route',
                'prepare_slot_topology_route',
            }:
                postcondition = wait_check
            elif first_step.name in TERMINAL_SUCCESS_STEPS:
                postcondition = self._verify_terminal_completion(
                    before_state=observed_state,
                    observed_state=after_state,
                    step=first_step,
                    task_goal=task_goal,
                    problem=problem,
                )
            else:
                postcondition = self._verify_expected_effects(
                    before_state=observed_state,
                    after_state=after_state,
                    step=first_step,
                    translated_step=translated_step,
                    task_goal=task_goal,
                    problem=problem,
                )
            sensor_arrival = _verified_sensor_arrival_for_step(
                wait_check,
                step=first_step,
                translated_step=translated_step,
                task_goal=task_goal,
                problem=problem,
            )
            if sensor_arrival:
                visual_postcondition = postcondition
                postcondition = PostconditionCheck(
                    status='satisfied',
                    reason='target_slot_sensor_identity_and_stop_verified',
                    details={
                        'arrival_verification': sensor_arrival,
                        'visual_postcondition': {
                            'status': visual_postcondition.status,
                            'reason': visual_postcondition.reason,
                            'details': dict(visual_postcondition.details),
                        },
                        'visual_disagreement': not visual_postcondition.satisfied,
                    },
                )
            if wait_check.status != 'satisfied' and postcondition.satisfied:
                postcondition = PostconditionCheck(
                    status='satisfied',
                    reason=f'{postcondition.reason}; wait={wait_check.reason}',
                    details=postcondition.details,
                )

            self._records.append(
                ClosedLoopStepRecord(
                    step_index=step_index,
                    observed_state_id=observed_state.state_id,
                    problem_name=problem.problem_name,
                    plan_length=len(plan),
                    symbolic_step=_step_text(first_step),
                    primitive=str(translated_step.event_action.get('primitive') or ''),
                    supervisor_status=supervisor_status,
                    postcondition=postcondition,
                    occupancy_changed=occupancy_changed,
                )
            )

            if (
                first_step.name in CLEARANCE_RELOCATE_ACTIONS
                and not postcondition.satisfied
            ):
                # The binary entry sensor may have stopped the blocker safely,
                # but learned vision must independently confirm the new block
                # before the planner is allowed to continue. Never command the
                # same blocker again from a contradictory visual state.
                self._safe_abort('clearance_visual_verification_failed')
                return self._result(
                    status='aborted',
                    reason=(
                        'clearance_visual_verification_failed:'
                        f'{postcondition.reason}'
                    ),
                    replans=replans,
                    unknown_retries=unknown_retries,
                    final_state_id=final_state_id,
                )

            if postcondition.satisfied:
                if (
                    first_step.name in TERMINAL_SUCCESS_STEPS
                    or _sensor_arrival_satisfies_goal(
                        sensor_arrival,
                        task_goal=task_goal,
                        problem=problem,
                    )
                    or self._task_goal_satisfied(after_state, task_goal)
                ):
                    return self._result(
                        status='succeeded',
                        reason='task_goal_satisfied',
                        replans=replans,
                        unknown_retries=unknown_retries,
                        final_state_id=final_state_id,
                    )
                if occupancy_changed:
                    replans += 1
                    self._replan_reasons.append('changed_occupancy')
                    if replans > self.config.max_replans:
                        self._safe_abort('persistent_changed_occupancy')
                        return self._result(
                            status='aborted',
                            reason='persistent_changed_occupancy',
                            replans=replans,
                            unknown_retries=unknown_retries,
                            final_state_id=final_state_id,
                        )
                continue

            replans += 1
            reason = postcondition.reason or postcondition.status
            self._replan_reasons.append(reason)
            if replans > self.config.max_replans or not postcondition.recoverable:
                self._safe_abort(f'postcondition_{postcondition.status}')
                return self._result(
                    status='aborted',
                    reason=f'postcondition_{postcondition.status}:{reason}',
                    replans=replans,
                    unknown_retries=unknown_retries,
                    final_state_id=final_state_id,
                )

        self._safe_abort('max_steps_exceeded')
        return self._result(
            status='aborted',
            reason='max_steps_exceeded',
            replans=replans,
            unknown_retries=unknown_retries,
            final_state_id=final_state_id,
        )

    def _clearance_phase_plan_error(
        self,
        *,
        first_step: PddlPlanStep,
        side: str,
    ) -> str:
        """Reject any plan that could tear down an active clearance route."""

        phase_actions = CLEARANCE_PHASE_ACTIONS
        active_side = self._route_clearance_active_side
        if active_side:
            if side != active_side:
                return f'active_side={active_side},planned_side={side}'
            if first_step.name not in {
                *CLEARANCE_RELOCATE_ACTIONS,
                *CLEARANCE_FINISH_ACTIONS,
                'pause_route_clearance',
            }:
                return (
                    f'active_side={active_side},forbidden_action='
                    f'{first_step.name}'
                )
            return ''
        if first_step.name in phase_actions - CLEARANCE_BEGIN_ACTIONS:
            return f'clearance_not_started,action={first_step.name}'
        return ''

    def _resume_clearance_phase_from_problem(
        self,
        *,
        problem: Room315PddlProblem,
        first_step: PddlPlanStep,
    ) -> None:
        """Recover the executor latch from fail-closed observed provenance.

        A new TaskGoal creates a new executive instance, while a prior verified
        goal may have left the physical rail in clearance mode.  The latch is
        therefore reconstructable only when the accepted device snapshot says
        clearance mode is active and every visually/certifiably interior
        shuttle has a consistent validated stop proof.
        """

        if self._route_clearance_active_side:
            return
        if first_step.name not in {
            *CLEARANCE_RELOCATE_ACTIONS,
            *CLEARANCE_FINISH_ACTIONS,
            'pause_route_clearance',
        }:
            return
        snapshot = dict(
            (problem.provenance or {})
            .get('route_normalization', {})
            .get('by_side', {})
            .get(problem.side, {})
        )
        if not snapshot.get('clearance_mode'):
            return
        unsafe = (
            list(snapshot.get('uncertified_interior_shuttles') or [])
            + list(snapshot.get('certificate_segment_mismatches') or [])
        )
        if unsafe:
            return
        clearance = dict(
            (problem.provenance or {}).get(
                'target_blocker_clearance_plan'
            ) or {}
        )
        if first_step.name in CLEARANCE_RELOCATE_ACTIONS and not (
            clearance.get('required')
            and clearance.get('ordered_relocations')
        ):
            return
        self._route_clearance_active_side = problem.side

    @staticmethod
    def _first_action_contract_error(
        *,
        first_step: PddlPlanStep,
        translated_step: TranslatedPlanStep,
        problem: Room315PddlProblem,
        task_goal: TaskGoal,
    ) -> str:
        """Bind the first PlanSys action to the frozen problem and TaskGoal."""

        command = dict(translated_step.command or {})
        command_side = str(command.get('side') or '').strip().casefold()
        if command_side and command_side != problem.side:
            return f'side_mismatch:problem={problem.side},plan={command_side}'
        if first_step.name not in RUNTIME_SYMBOLIC_ACTIONS:
            return f'unsupported_runtime_action:{first_step.name or "missing"}'

        applicability_error = _frozen_precondition_error(first_step, problem)
        if applicability_error:
            return applicability_error

        if first_step.name == 'inspect_state':
            expected = _pddl_contract_symbol(
                (task_goal.constraints or {}).get('inspection_subject')
                or 'room315_system'
            )
            planned = _pddl_contract_symbol(
                first_step.args[0] if first_step.args else ''
            )
            if planned != expected:
                return (
                    f'inspection_subject_mismatch:expected={expected},'
                    f'planned={planned}'
                )
            if not _problem_has_atom(
                problem.problem_text,
                'inspection_required',
                planned,
            ):
                return f'missing_frozen_inspection_target:{planned}'
            return ''

        shuttle_bound_actions = {
            'move_shuttle_to_slot',
            'prepare_topology_route',
            'prepare_slot_topology_route',
            'move_shuttle_from_segment_to_slot',
            'move_shuttle_via_topology_to_slot',
            *CLEARANCE_BEGIN_ACTIONS,
            *CLEARANCE_RELOCATE_ACTIONS,
            *CLEARANCE_FINISH_ACTIONS,
            'stop_shuttle',
        }
        provenance = dict(problem.provenance or {})
        if first_step.name in CLEARANCE_FINISH_ACTIONS | {
            'pause_route_clearance',
        }:
            normalization = dict(
                provenance.get('route_normalization', {})
                .get('by_side', {})
                .get(problem.side, {})
            )
            unsafe = (
                list(normalization.get('uncertified_interior_shuttles') or [])
                + list(
                    normalization.get('certificate_segment_mismatches') or []
                )
                + list(normalization.get('external_obstacles') or [])
            )
            if not normalization.get('clearance_mode'):
                return 'clearance_restoration_without_active_mode'
            if not normalization.get('clearance_pause_safe'):
                return 'clearance_restoration_not_certified_safe'
            if unsafe:
                return f'clearance_restoration_unsafe:{sorted(set(unsafe))}'
            if normalization.get(
                'controller_position_fields_used_for_localization'
            ) is not False:
                return 'clearance_restoration_used_controller_localization'
        if first_step.name not in shuttle_bound_actions:
            return ''
        planned_shuttle = _canonical_shuttle_id(
            first_step.args[0] if first_step.args else '',
            side=problem.side,
        )
        planning_phase = provenance.get('planning_phase')
        if (
            planning_phase == 'clear_blocker_to_interior_loop'
            and first_step.name in CLEARANCE_BEGIN_ACTIONS
        ):
            # The phase subgoal belongs to the blocker, but entering the held
            # route is explicitly anchored to the user's selected shuttle and
            # its frozen source/target pair.  Only the following relocation
            # action is blocker-bound.
            expected_shuttles = {
                _canonical_shuttle_id(
                    str(
                        provenance.get('target_blocker_clearance_plan', {})
                        .get('selected_shuttle')
                        or ''
                    ),
                    side=problem.side,
                )
            }
        elif planning_phase in {
            'clear_blocker_to_slot',
            'clear_blocker_to_interior_loop',
        }:
            relocation = dict(provenance.get('clearance_relocation') or {})
            expected_shuttles = {
                _canonical_shuttle_id(
                    str(relocation.get('shuttle') or ''),
                    side=problem.side,
                )
            }
        elif first_step.name in CLEARANCE_RELOCATE_ACTIONS:
            expected_shuttles = {
                _canonical_shuttle_id(
                    str(relocation.get('shuttle') or ''),
                    side=problem.side,
                )
                for relocation in (
                    provenance.get('target_blocker_clearance_plan', {})
                    .get('ordered_relocations', [])
                )
            }
        else:
            explicit = _canonical_shuttle_id(
                str((task_goal.constraints or {}).get('target_shuttle') or ''),
                side=problem.side,
            )
            expected_shuttles = {explicit} if explicit else {
                _canonical_shuttle_id(str(candidate), side=problem.side)
                for candidate in provenance.get('candidate_shuttles', [])
            }
            if not expected_shuttles and problem.selected_shuttle:
                expected_shuttles = {
                    _canonical_shuttle_id(
                        problem.selected_shuttle,
                        side=problem.side,
                    )
                }
        expected_shuttles.discard('')
        if not planned_shuttle or planned_shuttle not in expected_shuttles:
            return (
                f'shuttle_mismatch:allowed={sorted(expected_shuttles)},'
                f'planned={planned_shuttle or "unknown"}'
            )

        return ''

    def _observe(self, *, after_state_id: str = '') -> ObservedState:
        previous_state_id = str(after_state_id or '').strip()
        if previous_state_id:
            observe_fresh_after = getattr(
                self.observed_state_provider,
                'observe_fresh_after',
                None,
            )
            try:
                if callable(observe_fresh_after):
                    state = observe_fresh_after(
                        previous_state_id,
                        timestamp=time.time(),
                    )
                else:
                    state = self.observed_state_provider.observe(
                        timestamp=time.time()
                    )
            except Exception as exc:  # noqa: BLE001 - fail closed on no fresh state
                raise _FreshObservationUnavailable(
                    'fresh_visual_observation_unavailable_after:'
                    f'{previous_state_id}:{exc}'
                ) from exc
        else:
            state = self.observed_state_provider.observe(timestamp=time.time())
        if not isinstance(state, ObservedState):
            raise TypeError('ObservedStateProvider.observe() must return ObservedState')
        if previous_state_id and state.state_id == previous_state_id:
            raise _FreshObservationUnavailable(
                'fresh_visual_observation_unavailable_after:'
                f'{previous_state_id}:provider_replayed_same_state_id'
            )
        self._observations += 1
        return state

    def _refresh_runtime_clearance_certificates(self) -> None:
        """Synchronize executor-owned staging proofs across sequential goals."""

        getter = getattr(
            self.observed_state_provider,
            'runtime_clearance_certificates',
            None,
        )
        if not callable(getter):
            return
        raw = getter()
        if not isinstance(raw, dict):
            raise TypeError(
                'runtime_clearance_certificates() must return a dictionary'
            )
        self._runtime_clearance_certificates = {
            str(identity): dict(certificate)
            for identity, certificate in raw.items()
            if isinstance(certificate, dict)
        }

    @staticmethod
    def _next_planning_problem(
        problem: Room315PddlProblem,
    ) -> Room315PddlProblem:
        """Select one proved receding-horizon choreography subproblem."""

        clearance = dict(
            (problem.provenance or {}).get(
                'target_blocker_clearance_plan'
            ) or {}
        )
        if dict(clearance.get('intermediate_selected_advance') or {}).get(
            'required'
        ):
            return build_intermediate_selected_advance_problem(problem)
        relocations = list(clearance.get('ordered_relocations') or [])
        if not relocations:
            return problem
        destination = dict(relocations[0].get('destination') or {})
        destination_kind = str(
            destination.get('kind') or ''
        ).strip().casefold()
        if destination_kind == 'unavailable':
            normalization = dict(
                (problem.provenance or {})
                .get('route_normalization', {})
                .get('by_side', {})
                .get(problem.side, {})
            )
            if normalization.get('clearance_pause_safe'):
                return build_clearance_pause_problem(problem)
            raise PddlProblemBuildError(
                'no safe blocker destination and clearance cannot be paused: '
                f'{destination.get("reason") or "unknown"}'
            )
        if destination_kind in {'slot', 'interior_loop'}:
            # Clearance is deliberately planned as a one-relocation
            # receding-horizon subgoal.  A monolithic transport plan cannot
            # soundly assume that the visual move, controller stop, and fresh
            # effect verification will succeed.  Isolating the proved next
            # relocation lets PlanSys2 produce ``begin``/``relocate`` now;
            # after execution the complete transport problem is rebuilt from
            # a new accepted observation before route restoration or target
            # motion is considered.
            return build_first_blocker_clearance_problem(problem)
        raise PddlProblemBuildError(
            'unsupported proved blocker destination for closed-loop '
            f'planning: {destination_kind or "missing"}'
        )

    def _request_and_validate_plan(
        self,
        problem: Room315PddlProblem,
    ) -> tuple[list[str | PddlPlanStep], list[TranslatedPlanStep]]:
        self._plan_attempts += 1
        plan = self.planner.plan(problem, speed=self.config.speed_mps)
        if not isinstance(plan, list):
            raise RuntimeError('PlanSys2 planner returned a non-list plan')
        translated = translate_plan(plan)
        if len(translated) != len(plan):
            raise RuntimeError('translated plan length does not match symbolic plan')
        return plan, translated

    def _execute_first_atomic_step(
        self,
        *,
        translated_step: TranslatedPlanStep,
        problem: Room315PddlProblem,
        task_goal: TaskGoal,
        plan_length: int,
        step_index: int,
    ) -> str:
        previous_count = self.transport.supervisor_decision_count()
        command = dict(translated_step.command)
        target_slot = _target_slot_for_step(
            translated_step.pddl_step,
            task_goal,
            problem,
        )
        if (
            str(command.get('action') or '').strip() == 'shuttle'
            and str(command.get('command') or '').strip().upper() == 'ON'
            and target_slot
        ):
            target_side, target_slot_number = _split_slot_id(
                target_slot,
                default_side=problem.side,
            )
            command['side'] = target_side
            command['target_slot'] = target_slot_number
        command['closed_loop_executive'] = {
            'mode': 'plansys2_first_atomic_step',
            'problem_name': problem.problem_name,
            'plan_length': plan_length,
            'step_index': step_index,
            'symbolic_step': _step_text(translated_step.pddl_step),
        }
        self.transport.publish_command(command)
        decision = self.transport.wait_for_supervisor_decision(
            previous_count=previous_count,
            timeout_s=self.config.supervisor_timeout_s,
        )
        if decision is None:
            return 'timed_out'
        if not _decision_accepted(decision):
            return 'rejected'
        return 'accepted'

    def _execute_interior_clearance(
        self,
        *,
        translated_step: TranslatedPlanStep,
        problem: Room315PddlProblem,
        plan_length: int,
        step_index: int,
    ) -> tuple[str, PostconditionCheck]:
        """Execute one PlanSys2 relocation as guarded supervisor commands."""

        relocation = self._clearance_relocation_for_step(
            problem=problem,
            translated_step=translated_step,
        )
        destination = dict(relocation.get('destination') or {})
        if str(destination.get('kind') or '') != 'interior_loop':
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='invalid_interior_clearance_provenance',
            )
        side = str(translated_step.command.get('side') or problem.side)
        shuttle = str(translated_step.command.get('shuttle') or '')
        expected = _canonical_shuttle_id(
            str(relocation.get('shuttle') or ''),
            side=side,
        )
        if not expected or _canonical_shuttle_id(shuttle, side=side) != expected:
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='clearance_plan_and_provenance_shuttle_mismatch',
            )
        gate = str(destination.get('gate_switch') or '').strip().upper()
        target_segment = str(destination.get('target_segment') or '').strip().upper()
        try:
            target_s_m = float(destination['target_s_m'])
        except (KeyError, TypeError, ValueError):
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='clearance_destination_has_no_valid_target_s_m',
            )
        branch = INTERIOR_HOLDING_BRANCH_BY_GATE.get(gate)
        if (
            not branch
            or target_segment
            != str(branch.get('target_segment') or '').strip().upper()
        ):
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason=(
                    'clearance_destination_is_not_an_authoritative_'
                    'interior_branch'
                ),
            )

        motion_mode = str(
            destination.get('motion_mode') or 'enter_interior_branch'
        ).strip().casefold()
        if motion_mode not in {
            'enter_interior_branch',
            'advance_within_interior_branch',
        }:
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='unsupported_interior_clearance_motion_mode',
            )
        motion_origin_s_m: float | None = None
        bounded_motion_distance_m = target_s_m
        if motion_mode == 'advance_within_interior_branch':
            origin_proof = dict(
                destination.get('origin_clearance_proof') or {}
            )
            origin_spec = normalize_shuttle_ref(origin_proof.get('identity'))
            expected_sensor = INTERIOR_LOOP_ENTRY_SENSOR_BY_SIDE_AND_GATE.get(
                (side, gate),
                '',
            )
            try:
                motion_origin_s_m = float(destination['motion_origin_s_m'])
                bounded_motion_distance_m = float(
                    destination['bounded_motion_distance_m']
                )
                proof_origin_s_m = float(origin_proof['target_s_m'])
            except (KeyError, TypeError, ValueError):
                return 'rejected', PostconditionCheck(
                    status='unknown',
                    reason='invalid_certified_interior_advance_origin',
                )
            if (
                origin_spec is None
                or origin_spec != normalize_shuttle_ref(expected)
                or origin_proof.get('target_segment') != target_segment
                or abs(proof_origin_s_m - motion_origin_s_m) > 1e-9
                or str(origin_proof.get('entry_sensor') or '').upper()
                != expected_sensor
                or origin_proof.get('entry_sensor_identity_confirmed')
                is not True
                or origin_proof.get('controller_stop_confirmed') is not True
                or origin_proof.get('bounded_commanded_motion_completed')
                is not True
                or origin_proof.get(
                    'controller_position_fields_used_for_localization'
                )
                is not False
                or target_s_m <= motion_origin_s_m
                or abs(
                    bounded_motion_distance_m
                    - (target_s_m - motion_origin_s_m)
                )
                > 1e-9
            ):
                return 'rejected', PostconditionCheck(
                    status='unknown',
                    reason='incomplete_certified_interior_advance_origin',
                )

        common = {
            'mode': (
                'plansys2_supervised_interior_advance'
                if motion_origin_s_m is not None
                else 'plansys2_supervised_interior_clearance'
            ),
            'problem_name': problem.problem_name,
            'plan_length': plan_length,
            'step_index': step_index,
            'symbolic_step': _step_text(translated_step.pddl_step),
            'localization_source': 'accepted_visual_state',
            'controller_position_fields_used_for_localization': False,
        }
        start_command = {
            'action': 'shuttle',
            'side': side,
            'shuttle': shuttle,
            'command': 'ON',
            'speed': float(translated_step.command.get('speed') or self.config.speed_mps),
            'target_stopper': str(branch['exit_switch']),
        }
        status = self._publish_supervised_macro_command(start_command, common)
        if status != 'accepted':
            return status, PostconditionCheck(
                status='unknown',
                reason=f'clearance_shuttle_start_supervisor_{status}',
            )
        wait_method = getattr(
            self.transport,
            'wait_for_visual_position_and_stop',
            None,
        )
        if not callable(wait_method):
            return 'accepted', PostconditionCheck(
                status='timed_out',
                reason='transport_has_no_visual_interior_stop_contract',
            )
        result = wait_method(
            side=side,
            shuttle=shuttle,
            target_segment=target_segment,
            target_s_m=target_s_m,
            tolerance_m=INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M,
            entry_sensor=INTERIOR_LOOP_ENTRY_SENSOR_BY_SIDE_AND_GATE[
                (side, gate)
            ],
            minimum_clearance_delay_s=(
                bounded_motion_distance_m
                / float(
                    translated_step.command.get('speed')
                    or self.config.speed_mps
                )
            ),
            motion_origin_s_m=motion_origin_s_m,
            timeout_s=self.config.clearance_effect_timeout_s,
        )
        clearance_wait = _wait_result_check(
            result,
            ready_key='arrived',
            label='guarded_interior_stop',
        )
        if not clearance_wait.satisfied:
            return 'accepted', clearance_wait

        return 'accepted', PostconditionCheck(
            status='satisfied',
            reason='guarded_interior_stop_and_fresh_visual_frame_satisfied',
            details={
                **dict(clearance_wait.details),
                'clearance_mode_held': True,
                'normal_route_restored': False,
            },
        )

    def _prepare_topology_route(
        self,
        *,
        translated_step: TranslatedPlanStep,
        problem: Room315PddlProblem,
        plan_length: int,
        step_index: int,
    ) -> tuple[str, PostconditionCheck]:
        """Apply and verify the exact topology-derived mixed route state."""

        command = dict(translated_step.command)
        side = str(command.get('side') or problem.side).strip().casefold()
        shuttle = _canonical_shuttle_id(
            str(command.get('shuttle') or ''),
            side=side,
        )
        source_block = str(command.get('source_block') or '').strip().casefold()
        raw_target_slot = str(command.get('target_slot') or '').strip()
        try:
            target_side, target_number = _split_slot_id(
                raw_target_slot,
                default_side=side,
            )
        except ValueError:
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='topology_route_has_invalid_target_slot',
            )
        target_slot = _contract_slot_id(target_side, target_number).replace(':', '_')
        route = next(
            (
                item
                for item in (
                    (problem.provenance or {})
                    .get('topology_routes', {})
                    .get('routes', [])
                )
                if _canonical_shuttle_id(
                    str(item.get('shuttle') or ''),
                    side=side,
                ) == shuttle
                and str(item.get('source_block') or '').casefold() == source_block
                and str(item.get('target_slot_object') or '').casefold()
                == target_slot
            ),
            None,
        )
        if route is None:
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='topology_route_missing_from_audited_problem_provenance',
            )
        raw_source_slot = str(command.get('source_slot') or '').strip()
        if raw_source_slot:
            try:
                source_side, source_number = _split_slot_id(
                    raw_source_slot,
                    default_side=side,
                )
            except ValueError:
                return 'rejected', PostconditionCheck(
                    status='unknown',
                    reason='slot_topology_route_has_invalid_source_slot',
                )
            audited_source = str(
                route.get('source_slot_object') or ''
            ).strip().casefold()
            planned_source = _contract_slot_id(
                source_side,
                source_number,
            ).replace(':', '_')
            if planned_source != audited_source:
                return 'rejected', PostconditionCheck(
                    status='unknown',
                    reason='slot_topology_route_source_slot_mismatch',
                    details={
                        'planned': planned_source,
                        'audited': audited_source,
                    },
                )
        clearance_consistency = dict(
            route.get('runtime_clearance_visual_consistency') or {}
        )
        if (
            clearance_consistency.get('required')
            and not clearance_consistency.get('satisfied')
        ):
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason=(
                    'topology_route_clearance_certificate_visual_'
                    'consistency_failed'
                ),
                details=clearance_consistency,
            )
        if not bool(route.get('route_clear')) or route.get('blockers'):
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='topology_route_is_not_clear',
                details={'blockers': list(route.get('blockers') or [])},
            )
        if route.get('controller_position_fields_used_for_localization') is not False:
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='topology_route_used_forbidden_controller_position',
            )
        raw_switches = dict(route.get('required_switches') or {})
        if set(raw_switches) != set(DEVICE_NAMES):
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='topology_route_switch_assignment_is_incomplete',
            )
        switches = {
            device: (
                'INTERIOR'
                if str(raw_switches[device]).strip().upper() == 'I'
                else 'EXTERIOR'
            )
            for device in DEVICE_NAMES
        }
        if any(
            str(raw_switches[device]).strip().upper() not in {'E', 'I'}
            for device in DEVICE_NAMES
        ):
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='topology_route_switch_assignment_is_invalid',
            )
        stoppers = {device: '0' for device in DEVICE_NAMES}
        common = {
            'mode': 'plansys2_authoritative_topology_route_setup',
            'problem_name': problem.problem_name,
            'plan_length': plan_length,
            'step_index': step_index,
            'symbolic_step': _step_text(translated_step.pddl_step),
            'shuttle': shuttle,
            'source_block': source_block,
            'target_slot': _contract_slot_id(target_side, target_number),
            'topology_network_source': (
                (problem.provenance or {})
                .get('topology_routes', {})
                .get('network_sources', {})
                .get(side, '')
            ),
            'localization_source': 'accepted_visual_state',
            'controller_position_fields_used_for_localization': False,
        }
        operations = (
            (
                {'action': 'switches', 'side': side, 'switches': switches},
                lambda: self.transport.wait_for_switch_state(
                    side=side,
                    switches=switches,
                    timeout_s=self.config.effect_timeout_s,
                ),
                'topology_route_switches',
            ),
            (
                {'action': 'stoppers', 'side': side, 'stoppers': stoppers},
                lambda: self.transport.wait_for_stopper_state(
                    side=side,
                    stoppers=stoppers,
                    timeout_s=self.config.effect_timeout_s,
                ),
                'topology_route_stoppers',
            ),
        )
        for physical_command, wait, label in operations:
            status = self._publish_supervised_macro_command(
                physical_command,
                common,
            )
            if status != 'accepted':
                return status, PostconditionCheck(
                    status='unknown',
                    reason=f'{label}_supervisor_{status}',
                )
            check = _wait_result_check(wait(), ready_key='ready', label=label)
            if not check.satisfied:
                return 'accepted', check
        return 'accepted', PostconditionCheck(
            status='satisfied',
            reason='authoritative_topology_route_configured',
            details={
                **common,
                'switches': switches,
                'stoppers': stoppers,
                'route_blocks': list(route.get('route_blocks') or []),
            },
        )

    def _enter_route_clearance_mode(
        self,
        *,
        side: str,
        problem: Room315PddlProblem,
        plan_length: int,
        step_index: int,
        symbolic_step: str,
    ) -> tuple[str, PostconditionCheck]:
        """Hold the interior route until PDDL finishes every relocation."""

        provenance = dict(problem.provenance or {})
        relocation = dict(provenance.get('clearance_relocation') or {})
        if not relocation:
            clearance = dict(
                provenance.get('target_blocker_clearance_plan') or {}
            )
            relocations = list(clearance.get('ordered_relocations') or [])
            relocation = dict(relocations[0]) if len(relocations) == 1 else {}
        destination = dict(relocation.get('destination') or {})
        gate = str(destination.get('gate_switch') or '').strip().upper()
        target_segment = str(
            destination.get('target_segment') or ''
        ).strip().upper()
        branch = INTERIOR_HOLDING_BRANCH_BY_GATE.get(gate)
        if (
            not branch
            or target_segment
            != str(branch.get('target_segment') or '').strip().upper()
        ):
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='clearance_problem_has_no_authoritative_branch',
            )
        switches = {
            device: ('INTERIOR' if state == 'I' else 'EXTERIOR')
            for device, state in dict(branch['switches']).items()
        }
        exit_switch = str(branch['exit_switch'])
        stoppers = {
            device: ('1' if device == exit_switch else '0')
            for device in DEVICE_NAMES
        }
        common = self._clearance_phase_metadata(
            mode='begin_route_clearance_hold_interior',
            problem=problem,
            plan_length=plan_length,
            step_index=step_index,
            symbolic_step=symbolic_step,
        )
        commands_and_waits = (
            (
                {'action': 'switches', 'side': side, 'switches': switches},
                lambda: self.transport.wait_for_switch_state(
                    side=side,
                    switches=switches,
                    timeout_s=self.config.effect_timeout_s,
                ),
                'clearance_begin_switches',
            ),
            (
                {'action': 'stoppers', 'side': side, 'stoppers': stoppers},
                lambda: self.transport.wait_for_stopper_state(
                    side=side,
                    stoppers=stoppers,
                    timeout_s=self.config.effect_timeout_s,
                ),
                'clearance_begin_stoppers',
            ),
        )
        for command, wait, label in commands_and_waits:
            status = self._publish_supervised_macro_command(command, common)
            if status != 'accepted':
                return status, PostconditionCheck(
                    status='unknown',
                    reason=f'{label}_supervisor_{status}',
                )
            check = _wait_result_check(wait(), ready_key='ready', label=label)
            if not check.satisfied:
                return 'accepted', check
        return 'accepted', PostconditionCheck(
            status='satisfied',
            reason='route_clearance_mode_held_for_pending_blockers',
            details={
                'switches': switches,
                'stoppers': stoppers,
                'gate_switch': gate,
                'exit_switch': exit_switch,
                'target_segment': target_segment,
                'restore_deferred_to_finish_route_clearance': True,
            },
        )

    def _clearance_phase_metadata(
        self,
        *,
        mode: str,
        problem: Room315PddlProblem,
        plan_length: int,
        step_index: int,
        symbolic_step: str,
    ) -> dict[str, Any]:
        normalization = dict(
            (problem.provenance or {})
            .get('route_normalization', {})
            .get('by_side', {})
            .get(problem.side, {})
        )
        certificates = {
            str(identity): dict(certificate)
            for identity, certificate in
            self._runtime_clearance_certificates.items()
            if isinstance(certificate, dict)
            and str(certificate.get('side') or '').strip().casefold()
            == problem.side
        }
        return {
            'mode': mode,
            'problem_name': problem.problem_name,
            'plan_length': plan_length,
            'step_index': step_index,
            'symbolic_step': symbolic_step,
            'localization_source': 'accepted_visual_state',
            'controller_position_fields_used_for_localization': False,
            'route_normalization_proof': normalization,
            'runtime_clearance_certificates': certificates,
        }

    @staticmethod
    def _clearance_relocation_for_step(
        *,
        problem: Room315PddlProblem,
        translated_step: TranslatedPlanStep,
    ) -> dict[str, Any]:
        provenance = dict(problem.provenance or {})
        legacy = dict(provenance.get('clearance_relocation') or {})
        shuttle = str(translated_step.command.get('shuttle') or '')
        expected = _canonical_shuttle_id(shuttle, side=problem.side)
        if legacy and _canonical_shuttle_id(
            str(legacy.get('shuttle') or ''),
            side=problem.side,
        ) == expected:
            return legacy
        clearance = dict(
            provenance.get('target_blocker_clearance_plan') or {}
        )
        matches = [
            dict(relocation)
            for relocation in clearance.get('ordered_relocations') or []
            if _canonical_shuttle_id(
                str(relocation.get('shuttle') or ''),
                side=problem.side,
            ) == expected
        ]
        return matches[0] if len(matches) == 1 else {}

    def _restore_normal_route_after_clearance(
        self,
        *,
        side: str,
        common: dict[str, Any],
        restore_mode: str = 'restore_normal_route_after_interior_clearance',
        success_reason: str = 'normal_route_restored_after_interior_clearance',
    ) -> tuple[str, PostconditionCheck]:
        """Restore canonical exterior/open routing before re-observation."""

        exterior_switches = {
            device: 'EXTERIOR'
            for device in ('A1', 'A2', 'A3', 'A4')
        }
        open_stoppers = {
            device: '0'
            for device in ('A1', 'A2', 'A3', 'A4')
        }
        restoration_steps = (
            (
                {
                    'action': 'switches',
                    'side': side,
                    'switches': exterior_switches,
                },
                lambda: self.transport.wait_for_switch_state(
                    side=side,
                    switches=exterior_switches,
                    timeout_s=self.config.effect_timeout_s,
                ),
                'clearance_restore_switches',
            ),
            (
                {
                    'action': 'stoppers',
                    'side': side,
                    'stoppers': open_stoppers,
                },
                lambda: self.transport.wait_for_stopper_state(
                    side=side,
                    stoppers=open_stoppers,
                    timeout_s=self.config.effect_timeout_s,
                ),
                'clearance_restore_stoppers',
            ),
        )
        restore_metadata = {
            **common,
            'mode': restore_mode,
        }
        for command, wait, label in restoration_steps:
            status = self._publish_supervised_macro_command(
                command,
                restore_metadata,
            )
            if status != 'accepted':
                return status, PostconditionCheck(
                    status='unknown',
                    reason=f'{label}_supervisor_{status}',
                )
            check = _wait_result_check(
                wait(),
                ready_key='ready',
                label=label,
            )
            if not check.satisfied:
                return 'accepted', check
        observation_count = getattr(
            self.transport,
            'visual_observation_count',
            None,
        )
        wait_for_visual = getattr(
            self.transport,
            'wait_for_fresh_visual_observation',
            None,
        )
        if not callable(observation_count) or not callable(wait_for_visual):
            return 'accepted', PostconditionCheck(
                status='unknown',
                reason='transport_has_no_post_restoration_visual_barrier',
            )
        previous_visual_count = int(observation_count())
        fresh_visual = _wait_result_check(
            wait_for_visual(
                previous_count=previous_visual_count,
                timeout_s=self.config.effect_timeout_s,
            ),
            ready_key='ready',
            label='post_restoration_visual_frame',
        )
        if not fresh_visual.satisfied:
            return 'accepted', fresh_visual
        return 'accepted', PostconditionCheck(
            status='satisfied',
            reason=success_reason,
            details={
                'switches': exterior_switches,
                'stoppers': open_stoppers,
                'fresh_visual_frame': dict(fresh_visual.details),
            },
        )

    def _restore_normal_route_from_mixed_topology(
        self,
        *,
        problem: Room315PddlProblem,
        translated_step: TranslatedPlanStep,
        plan_length: int,
        step_index: int,
        symbolic_step: str,
    ) -> tuple[str, PostconditionCheck]:
        """Execute only a state-derived, certificate-safe route normalization."""

        command = dict(translated_step.command or {})
        planned_side = str(command.get('side') or '').strip().casefold()
        if planned_side != problem.side:
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason=(
                    'route_normalization_side_mismatch:'
                    f'problem={problem.side},plan={planned_side or "missing"}'
                ),
            )
        valid_stations = {
            f'{problem.side}_{station}'
            for station in (
                ('yaskawa', 'staubli')
                if problem.side == 'right'
                else ('yaskawa', 'kuka')
            )
        }
        source_station = str(command.get('source_station') or '').strip()
        target_station = str(command.get('target_station') or '').strip()
        if (
            source_station not in valid_stations
            or target_station not in valid_stations
        ):
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason=(
                    'route_normalization_station_mismatch:'
                    f'source={source_station or "missing"},'
                    f'target={target_station or "missing"},'
                    f'side={problem.side}'
                ),
            )
        normalization = (
            (problem.provenance or {})
            .get('route_normalization', {})
            .get('by_side', {})
            .get(problem.side, {})
        )
        if not normalization.get('reconfiguration_required'):
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='normal_route_reconfiguration_not_required',
            )
        if not normalization.get('reconfiguration_safe'):
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='normal_route_reconfiguration_not_proven_safe',
                details={
                    'reason': normalization.get('reason', ''),
                    'uncertified_interior_shuttles': list(
                        normalization.get('uncertified_interior_shuttles') or []
                    ),
                    'external_obstacles': list(
                        normalization.get('external_obstacles') or []
                    ),
                },
            )
        if normalization.get('clearance_mode'):
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='active_clearance_must_finish_before_route_normalization',
            )
        if (
            normalization.get(
                'controller_position_fields_used_for_localization'
            )
            is not False
        ):
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='route_normalization_used_forbidden_controller_position',
            )
        common = self._clearance_phase_metadata(
            mode='restore_normal_route_before_slot_motion',
            problem=problem,
            plan_length=plan_length,
            step_index=step_index,
            symbolic_step=symbolic_step,
        )
        common.update({
            'route_normalization_proof': dict(normalization),
            'controller_position_fields_used_for_localization': False,
        })
        return self._restore_normal_route_after_clearance(
            side=problem.side,
            common=common,
            restore_mode='restore_normal_route_before_slot_motion',
            success_reason='normal_route_restored_before_slot_motion',
        )

    def _publish_supervised_macro_command(
        self,
        command: dict[str, Any],
        metadata: dict[str, Any],
    ) -> str:
        previous_count = self.transport.supervisor_decision_count()
        payload = dict(command)
        payload['closed_loop_executive'] = dict(metadata)
        self.transport.publish_command(payload)
        decision = self.transport.wait_for_supervisor_decision(
            previous_count=previous_count,
            timeout_s=self.config.supervisor_timeout_s,
        )
        if decision is None:
            return 'timed_out'
        return 'accepted' if _decision_accepted(decision) else 'rejected'

    def _interior_clearance_certificate(
        self,
        *,
        wait_check: PostconditionCheck,
        problem: Room315PddlProblem,
        translated_step: TranslatedPlanStep,
    ) -> dict[str, Any] | None:
        """Create a narrow route-clearance proof without controller position."""

        certificate, _reason = self._interior_clearance_certificate_result(
            wait_check=wait_check,
            problem=problem,
            translated_step=translated_step,
        )
        return certificate

    def _interior_clearance_certificate_result(
        self,
        *,
        wait_check: PostconditionCheck,
        problem: Room315PddlProblem,
        translated_step: TranslatedPlanStep,
    ) -> tuple[dict[str, Any] | None, str]:
        """Build the certificate and retain an exact fail-closed reason."""

        if not wait_check.satisfied:
            return None, f'wait_not_satisfied:{wait_check.reason}'
        relocation = self._clearance_relocation_for_step(
            problem=problem,
            translated_step=translated_step,
        )
        destination = dict(relocation.get('destination') or {})
        spec = normalize_shuttle_ref(
            relocation.get('shuttle'),
            side=problem.side,
        )
        details = dict(wait_check.details or {})
        if spec is None:
            return None, 'unknown_relocation_identity'
        expected_sensor = INTERIOR_LOOP_ENTRY_SENSOR_BY_SIDE_AND_GATE.get(
            (spec.side, str(destination.get('gate_switch') or '').upper()),
            '',
        )
        motion_mode = str(
            destination.get('motion_mode') or 'enter_interior_branch'
        ).strip().casefold()
        advance_within_interior = (
            motion_mode == 'advance_within_interior_branch'
        )
        required_true = (
            'controller_stop_confirmed',
            'post_stop_visual_frame_received',
        )
        for name in required_true:
            if not bool(details.get(name)):
                return None, f'missing_{name}'
        if (
            not advance_within_interior
            and not bool(details.get('entry_sensor_identity_confirmed'))
        ):
            return None, 'missing_entry_sensor_identity_confirmed'
        if str(details.get('entry_sensor') or '').upper() != expected_sensor:
            return None, (
                'wrong_entry_sensor:'
                f'expected={expected_sensor},observed='
                f'{str(details.get("entry_sensor") or "").upper()}'
            )
        expected_match = (
            'certified_interior_origin_plus_bounded_travel_time'
            if advance_within_interior
            else 'interior_entry_sensor_plus_bounded_travel_time'
        )
        if details.get('matched_by') != expected_match:
            return None, (
                'wrong_stop_trigger:'
                f'{details.get("matched_by") or "missing"}'
            )
        if details.get('controller_position_fields_used_for_localization') is not False:
            return None, 'forbidden_controller_position_fields'
        if details.get('clearance_mode_held') is not True:
            return None, 'clearance_mode_not_held'
        if details.get('normal_route_restored') is not False:
            return None, 'normal_route_restored_before_clearance_finished'
        try:
            target_s_m = float(destination['target_s_m'])
            observed_s_m = float(details['observed_s_m'])
        except (KeyError, TypeError, ValueError):
            return None, 'missing_or_invalid_raw_visual_s_m'
        error_m = abs(observed_s_m - target_s_m)
        target_segment = str(destination.get('target_segment') or '').upper()
        observed_segment = str(details.get('observed_segment') or '').upper()
        if not target_segment:
            return None, 'missing_target_segment'
        if not observed_segment:
            return None, 'missing_raw_visual_segment'
        model_segment_disagreement = observed_segment != target_segment
        origin_proof: dict[str, Any] = {}
        motion_origin_s_m: float | None = None
        bounded_motion_distance_m = target_s_m
        if advance_within_interior:
            origin_proof = dict(
                destination.get('origin_clearance_proof') or {}
            )
            try:
                motion_origin_s_m = float(destination['motion_origin_s_m'])
                bounded_motion_distance_m = float(
                    destination['bounded_motion_distance_m']
                )
                detail_origin_s_m = float(details['motion_origin_s_m'])
                detail_distance_m = float(
                    details['bounded_motion_distance_m']
                )
            except (KeyError, TypeError, ValueError):
                return None, 'missing_or_invalid_interior_advance_distance'
            if (
                details.get('interior_advance_origin_certified') is not True
                or abs(detail_origin_s_m - motion_origin_s_m) > 1e-9
                or abs(detail_distance_m - bounded_motion_distance_m) > 1e-9
                or origin_proof.get('entry_sensor_identity_confirmed')
                is not True
                or origin_proof.get('controller_stop_confirmed') is not True
                or origin_proof.get('bounded_commanded_motion_completed')
                is not True
                or origin_proof.get(
                    'controller_position_fields_used_for_localization'
                )
                is not False
                or target_s_m <= motion_origin_s_m
                or abs(
                    bounded_motion_distance_m
                    - (target_s_m - motion_origin_s_m)
                )
                > 1e-9
            ):
                return None, 'incomplete_interior_advance_origin_proof'
        certificate = {
            'identity': spec.short_id,
            'shuttle': spec.shuttle_id,
            'side': spec.side,
            'target_segment': target_segment,
            'target_s_m': target_s_m,
            'observed_segment': observed_segment,
            'observed_s_m': observed_s_m,
            'absolute_error_m': error_m,
            'tolerance_m': INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M,
            'visual_s_within_tolerance': (
                error_m <= INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M
            ),
            'entry_sensor': expected_sensor,
            'matched_by': details['matched_by'],
            'entry_sensor_identity_confirmed': True,
            'interior_advance_origin_certified': advance_within_interior,
            'motion_origin_s_m': motion_origin_s_m,
            'bounded_motion_distance_m': bounded_motion_distance_m,
            'origin_clearance_proof': origin_proof,
            'controller_stop_confirmed': True,
            'post_stop_visual_frame_received': True,
            'post_stop_visual_confirmation': bool(
                details.get('post_stop_visual_confirmation')
            ),
            'model_segment_disagreement': model_segment_disagreement,
            'visual_prediction_disagreement': (
                model_segment_disagreement
                or error_m > INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M
            ),
            'visual_longitudinal_prediction_preserved': True,
            'visual_segment_prediction_preserved': True,
            'clearance_mode_held': True,
            'normal_route_restored': False,
            'bounded_commanded_motion_completed': True,
            'segment_effect_verification_source': (
                'identity_bearing_binary_interior_entry_sensor_and_held_route'
            ),
            'longitudinal_effect_verification_source': (
                'binary_entry_sensor_plus_bounded_commanded_motion_and_stop'
            ),
            'model_prediction_replaced': False,
            'controller_position_fields_used_for_localization': False,
            'proof': (
                'certified_interior_origin+held_clearance_route+'
                'bounded_forward_motion+controller_stop+'
                'fresh_visual_prediction_preserved'
                if advance_within_interior
                else
                'interior_entry_sensor_identity+held_clearance_route+'
                'bounded_commanded_motion+controller_stop+'
                'fresh_visual_prediction_preserved'
            ),
        }
        return certificate, ''

    def _verify_interior_clearance(
        self,
        *,
        after_state: ObservedState,
        problem: Room315PddlProblem,
        translated_step: TranslatedPlanStep,
        clearance_certificate: dict[str, Any] | None = None,
    ) -> PostconditionCheck:
        relocation = self._clearance_relocation_for_step(
            problem=problem,
            translated_step=translated_step,
        )
        destination = dict(relocation.get('destination') or {})
        shuttle = str(relocation.get('shuttle') or '')
        spec = normalize_shuttle_ref(shuttle, side=problem.side)
        if spec is None:
            return PostconditionCheck(status='unknown', reason='unknown_clearance_shuttle')
        position = _first_fact(
            after_state,
            subjects=(
                shuttle,
                spec.short_id,
                spec.shuttle_id,
                spec.gazebo_entity_name,
            ),
            predicate='rail_position',
        )
        if position is None or position.status != 'known' or not isinstance(position.value, dict):
            return PostconditionCheck(
                status='unknown',
                reason='missing_fresh_visual_clearance_position',
            )
        target_segment = str(destination.get('target_segment') or '').strip().upper()
        observed_segment = str(position.value.get('segment') or '').strip().upper()
        try:
            target_s_m = float(destination['target_s_m'])
            observed_s_m = float(position.value['s_m'])
        except (KeyError, TypeError, ValueError):
            return PostconditionCheck(status='unknown', reason='invalid_visual_clearance_position')
        error_m = abs(observed_s_m - target_s_m)
        details = {
            'shuttle': spec.shuttle_id,
            'target_segment': target_segment,
            'observed_segment': observed_segment,
            'target_s_m': target_s_m,
            'observed_s_m': observed_s_m,
            'absolute_error_m': error_m,
            'tolerance_m': INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M,
            'localization_source': 'accepted_visual_state',
            'controller_position_fields_used_for_localization': False,
        }
        if (
            observed_segment == target_segment
            and error_m <= INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M
        ):
            return PostconditionCheck(
                status='satisfied',
                reason='blocker_visual_interior_clearance_verified',
                details=details,
            )
        certificate = dict(clearance_certificate or {})
        expected_sensor = INTERIOR_LOOP_ENTRY_SENSOR_BY_SIDE_AND_GATE.get(
            (
                spec.side,
                str(destination.get('gate_switch') or '').upper(),
            ),
            '',
        )
        try:
            certified_target_s_m = float(certificate['target_s_m'])
        except (KeyError, TypeError, ValueError):
            certified_target_s_m = float('nan')
        if (
            certificate.get('identity') == spec.short_id
            and certificate.get('side') == spec.side
            and certificate.get('target_segment') == target_segment
            and abs(certified_target_s_m - target_s_m) <= 1e-9
            and certificate.get('entry_sensor') == expected_sensor
            and bool(certificate.get('entry_sensor_identity_confirmed'))
            and bool(certificate.get('controller_stop_confirmed'))
            and bool(certificate.get('post_stop_visual_frame_received'))
            and bool(certificate.get('bounded_commanded_motion_completed'))
            and certificate.get('clearance_mode_held') is True
            and certificate.get('normal_route_restored') is False
            and certificate.get('matched_by') in {
                'interior_entry_sensor_plus_bounded_travel_time',
                'certified_interior_origin_plus_bounded_travel_time',
            }
            and (
                certificate.get('matched_by')
                != 'certified_interior_origin_plus_bounded_travel_time'
                or certificate.get('interior_advance_origin_certified')
                is True
            )
            and certificate.get(
                'controller_position_fields_used_for_localization'
            ) is False
        ):
            details.update({
                'route_clearance_certificate': certificate,
                'visual_segment_disagreement': (
                    observed_segment != target_segment
                ),
                'visual_longitudinal_disagreement': (
                    error_m > INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M
                ),
                'segment_identity_source': 'accepted_visual_state',
                'raw_visual_longitudinal_position_source': (
                    'accepted_visual_state'
                ),
                'segment_effect_verification_source': (
                    'identity_bearing_binary_interior_entry_sensor_and_held_route'
                ),
                'longitudinal_effect_verification_source': (
                    'binary_entry_sensor_plus_bounded_commanded_motion_and_stop'
                ),
                'model_prediction_replaced': False,
            })
            return PostconditionCheck(
                status='satisfied',
                reason=(
                    'blocker_sensor_motion_certified_interior_clearance'
                ),
                details=details,
            )
        return PostconditionCheck(
            status='mismatch',
            reason='blocker_not_at_visual_interior_clearance_pose',
            details=details,
        )

    def _wait_for_expected_effects(
        self,
        *,
        step: PddlPlanStep,
        translated_step: TranslatedPlanStep,
        task_goal: TaskGoal,
        problem: Room315PddlProblem,
    ) -> PostconditionCheck:
        command = translated_step.command
        action = str(command.get('action') or '').strip()
        if action == 'switches':
            result = self.transport.wait_for_switch_state(
                side=str(command.get('side') or problem.side),
                switches=dict(command.get('switches') or {}),
                timeout_s=self.config.effect_timeout_s,
            )
            return _wait_result_check(result, ready_key='ready', label='switch_state')
        if action == 'stoppers':
            result = self.transport.wait_for_stopper_state(
                side=str(command.get('side') or problem.side),
                stoppers=dict(command.get('stoppers') or {}),
                timeout_s=self.config.effect_timeout_s,
            )
            return _wait_result_check(result, ready_key='ready', label='stopper_state')
        if action == 'shuttle' and str(command.get('command') or '').upper() == 'OFF':
            result = self.transport.wait_for_shuttle_stopped(
                side=str(command.get('side') or problem.side),
                shuttle=str(command.get('shuttle') or ''),
                timeout_s=self.config.effect_timeout_s,
            )
            return _wait_result_check(result, ready_key='ready', label='shuttle_stopped')
        if action == 'shuttle' and str(command.get('command') or '').upper() == 'ON':
            target_slot = _target_slot_for_step(step, task_goal, problem)
            if target_slot:
                side, slot_number = _split_slot_id(target_slot, default_side=problem.side)
                target_sensor = SLOT_SENSOR_BY_SIDE_AND_SLOT.get((side, slot_number))
                target_station = (
                    SLOT_STATION_BY_SIDE_AND_SLOT.get((side, slot_number), '')
                    or problem.target_station
                )
                result = self.transport.wait_for_target_arrival(
                    side=side,
                    target_sensors=[target_sensor] if target_sensor else [],
                    shuttle=str(command.get('shuttle') or ''),
                    timeout_s=self.config.effect_timeout_s,
                    target_slot=slot_number,
                    target_station=target_station,
                )
                return _wait_result_check(result, ready_key='arrived', label='target_arrival')
        return PostconditionCheck(status='satisfied', reason='no_wait_required')

    def _verify_expected_effects(
        self,
        *,
        before_state: ObservedState,
        after_state: ObservedState,
        step: PddlPlanStep,
        translated_step: TranslatedPlanStep,
        task_goal: TaskGoal,
        problem: Room315PddlProblem,
    ) -> PostconditionCheck:
        obstacle_reason = self._obstacle_reason(after_state)
        if obstacle_reason:
            return PostconditionCheck(status='obstacle', reason=obstacle_reason)

        command = translated_step.command
        action = str(command.get('action') or '').strip()
        if action == 'switches':
            return _verify_device_state(
                after_state,
                side=str(command.get('side') or problem.side),
                device_kind='switch',
                assignments=dict(command.get('switches') or {}),
                command_to_state=_canonical_switch_effect_state,
            )
        if action == 'stoppers':
            return _verify_device_state(
                after_state,
                side=str(command.get('side') or problem.side),
                device_kind='stopper',
                assignments=dict(command.get('stoppers') or {}),
                command_to_state=_canonical_stopper_effect_state,
            )
        if action == 'shuttle':
            shuttle = str(command.get('shuttle') or '')
            if str(command.get('command') or '').upper() == 'OFF':
                return _verify_shuttle_stopped(after_state, shuttle, problem.side)
            target_slot = _target_slot_for_step(step, task_goal, problem)
            if target_slot:
                selected = _canonical_shuttle_id(shuttle, side=problem.side)
                return _verify_slot_occupancy(after_state, target_slot, selected)
            return PostconditionCheck(status='satisfied', reason='shuttle_command_accepted')
        if action == 'DONE' and step.name in TERMINAL_SUCCESS_STEPS:
            return PostconditionCheck(
                status='mismatch',
                reason='terminal_marker_requires_fresh_goal_verification',
            )

        if before_state.state_id == after_state.state_id:
            return PostconditionCheck(status='satisfied', reason='accepted_no_state_effect')
        return PostconditionCheck(status='satisfied', reason='accepted_and_reobserved')

    def _verify_terminal_completion(
        self,
        *,
        before_state: ObservedState,
        observed_state: ObservedState,
        step: PddlPlanStep,
        task_goal: TaskGoal,
        problem: Room315PddlProblem,
    ) -> PostconditionCheck:
        """Verify a symbolic terminal marker without publishing an actuator command."""

        goal_type = str(
            (task_goal.constraints or {}).get('goal_type') or ''
        ).strip().casefold()
        if step.name == 'inspect_state':
            if goal_type != 'inspection' or problem.goal_type != 'inspection':
                return PostconditionCheck(
                    status='mismatch',
                    reason='inspection_terminal_does_not_match_task_goal',
                )
            if (
                observed_state.state_id == before_state.state_id
                or float(observed_state.timestamp)
                <= float(before_state.timestamp)
            ):
                return PostconditionCheck(
                    status='unknown',
                    reason='inspection_requires_fresh_observation',
                    details={
                        'before_state_id': before_state.state_id,
                        'after_state_id': observed_state.state_id,
                        'before_timestamp': float(before_state.timestamp),
                        'after_timestamp': float(observed_state.timestamp),
                        'supervisor_command_published': False,
                    },
                )
            return PostconditionCheck(
                status='satisfied',
                reason='fresh_validated_observation_inspected',
                details={
                    'state_id': observed_state.state_id,
                    'inspection_subject': (
                        (task_goal.constraints or {}).get(
                            'inspection_subject'
                        )
                        or 'room315_system'
                    ),
                    'supervisor_command_published': False,
                },
            )
        if goal_type != 'transport':
            return PostconditionCheck(
                status='mismatch',
                reason='transport_terminal_does_not_match_task_goal',
            )
        if not self._task_goal_satisfied(observed_state, task_goal):
            return PostconditionCheck(
                status='mismatch',
                reason='terminal_plan_claimed_unsatisfied_transport_goal',
                details={
                    'state_id': observed_state.state_id,
                    'supervisor_command_published': False,
                },
            )
        return PostconditionCheck(
            status='satisfied',
            reason='fresh_observation_proves_transport_goal',
            details={
                'state_id': observed_state.state_id,
                'supervisor_command_published': False,
            },
        )

    def _task_goal_satisfied(self, observed_state: ObservedState, task_goal: TaskGoal) -> bool:
        constraints = dict(task_goal.constraints or {})
        if str(constraints.get('goal_type') or '').casefold() != 'transport':
            return False
        target_slot = constraints.get('target_slot')
        if target_slot:
            side = str(constraints.get('side') or 'right').casefold()
            canonical_slot = _contract_slot_id(side, target_slot)
            target_shuttle = constraints.get('target_shuttle') or ''
            selected = _canonical_shuttle_id(target_shuttle, side=side) if target_shuttle else ''
            fact = _fact(observed_state, canonical_slot, 'occupancy')
            if fact is None or fact.status != 'known':
                return False
            occupant = _occupancy_shuttle(fact.value, side=side)
            if not occupant:
                return False
            if selected and occupant != selected:
                return False
            required_payload = str(
                constraints.get('payload_filter')
                or constraints.get('payload_required')
                or 'any'
            ).casefold()
            if required_payload in {'loaded', 'empty'}:
                loaded = _loaded_state(observed_state, occupant, side=side)
                if loaded is None:
                    return False
                if required_payload == 'loaded' and not loaded:
                    return False
                if required_payload == 'empty' and loaded:
                    return False
            return True
        target_station = str(
            constraints.get('target_station') or ''
        ).strip().casefold()
        if target_station:
            side = str(constraints.get('side') or 'right').casefold()
            station_slots = [
                slot
                for (slot_side, slot), station in
                SLOT_STATION_BY_SIDE_AND_SLOT.items()
                if slot_side == side and station == target_station
            ]
            target_shuttle = constraints.get('target_shuttle') or ''
            selected = (
                _canonical_shuttle_id(target_shuttle, side=side)
                if target_shuttle
                else ''
            )
            required_payload = str(
                constraints.get('payload_filter')
                or constraints.get('payload_required')
                or 'any'
            ).casefold()
            for slot in station_slots:
                fact = _fact(
                    observed_state,
                    _contract_slot_id(side, slot),
                    'occupancy',
                )
                if fact is None or fact.status != 'known':
                    continue
                occupant = _occupancy_shuttle(fact.value, side=side)
                if not occupant or (selected and occupant != selected):
                    continue
                loaded = _loaded_state(observed_state, occupant, side=side)
                if required_payload == 'loaded' and loaded is not True:
                    continue
                if required_payload == 'empty' and loaded is not False:
                    continue
                return True
        return False

    def _obstacle_reason(self, observed_state: ObservedState) -> str:
        for fact in observed_state.fused_planner_state:
            if fact.predicate != 'present_obstacles':
                continue
            if fact.status in RECOVERABLE_STATE_STATUSES:
                return f'{fact.subject}_{fact.status}'
            if fact.status == 'known' and fact.value:
                return f'{fact.subject}:{fact.value}'
        return ''

    def _safe_abort(self, reason: str) -> None:
        if self._safe_abort_sent:
            return
        self._safe_abort_sent = True
        self.transport.publish_command({
            'action': self.config.safe_abort_command,
            'reason': str(reason),
            'closed_loop_executive': {
                'mode': 'safe_abort',
                'reason': str(reason),
            },
        })

    def _result(
        self,
        *,
        status: str,
        reason: str,
        replans: int,
        unknown_retries: int,
        final_state_id: str,
    ) -> ClosedLoopExecutiveResult:
        return ClosedLoopExecutiveResult(
            status=status,
            reason=reason,
            executed_steps=tuple(self._records),
            plan_attempts=self._plan_attempts,
            observations=self._observations,
            replans=replans,
            unknown_retries=unknown_retries,
            safe_abort_sent=self._safe_abort_sent,
            final_state_id=final_state_id,
            replan_reasons=tuple(self._replan_reasons),
        )


def _as_plan_step(step: str | PddlPlanStep) -> PddlPlanStep:
    if isinstance(step, PddlPlanStep):
        return step
    return PddlPlanStep.from_text(str(step))


def _step_text(step: str | PddlPlanStep) -> str:
    parsed = _as_plan_step(step)
    if parsed.raw:
        return parsed.raw
    return ' '.join([parsed.name, *parsed.args]).strip()


def _decision_accepted(decision: dict[str, Any]) -> bool:
    if not isinstance(decision, dict):
        return False
    if decision.get('accepted') is False:
        return False
    status = str(decision.get('status') or decision.get('decision') or '').casefold()
    if status in {'rejected', 'failed', 'blocked', 'timed_out', 'timeout'}:
        return False
    if status in {'accepted', 'executed', 'ok', 'approved'}:
        return True
    return bool(decision.get('accepted', True))


def _wait_result_check(result: dict[str, Any], *, ready_key: str, label: str) -> PostconditionCheck:
    if not isinstance(result, dict):
        return PostconditionCheck(status='timed_out', reason=f'{label}_invalid_wait_result')
    if bool(result.get(ready_key)):
        return PostconditionCheck(
            status='satisfied',
            reason=f'{label}_wait_satisfied',
            details=dict(result),
        )
    reason = str(result.get('reason') or f'{label}_timeout')
    return PostconditionCheck(status='timed_out', reason=reason, details=dict(result))


def _verified_sensor_arrival_for_step(
    wait_check: PostconditionCheck,
    *,
    step: PddlPlanStep,
    translated_step: TranslatedPlanStep,
    task_goal: TaskGoal,
    problem: Room315PddlProblem,
) -> dict[str, Any]:
    """Validate the narrow deterministic proof allowed for final stopping."""

    details = dict(wait_check.details or {})
    if (
        not wait_check.satisfied
        or details.get('matched_by') != 'deterministic_slot_sensor'
        or details.get('sensor_identity_confirmed') is not True
        or details.get('controller_stop_confirmed') is not True
        or details.get('controller_target_slot_confirmed') is not True
        or details.get('controller_position_fields_used_for_localization') is not False
    ):
        return {}
    expected_slot = _target_slot_for_step(step, task_goal, problem)
    if not expected_slot:
        return {}
    side, slot_number = _split_slot_id(expected_slot, default_side=problem.side)
    expected_sensor = SLOT_SENSOR_BY_SIDE_AND_SLOT.get((side, slot_number), '')
    expected_shuttle = _canonical_shuttle_id(
        str(translated_step.command.get('shuttle') or ''),
        side=side,
    )
    observed_shuttle = _canonical_shuttle_id(
        str(details.get('shuttle') or ''),
        side=side,
    )
    if (
        str(details.get('side') or '') != side
        or str(details.get('target_slot') or '') != slot_number
        or str(details.get('target_sensor') or '').upper() != expected_sensor.upper()
        or not expected_shuttle
        or observed_shuttle != expected_shuttle
    ):
        return {}
    return {
        **details,
        'target_slot_contract': _contract_slot_id(side, slot_number),
        'shuttle_contract': expected_shuttle,
        'ground_truth_scope': 'final_actuation_verification_only',
        'planner_localization_unchanged': True,
    }


def _sensor_arrival_satisfies_goal(
    arrival: dict[str, Any],
    *,
    task_goal: TaskGoal,
    problem: Room315PddlProblem,
) -> bool:
    if not arrival:
        return False
    constraints = dict(task_goal.constraints or {})
    if str(constraints.get('goal_type') or '').casefold() != 'transport':
        return False
    side = str(constraints.get('side') or problem.side).casefold()
    target_slot = _contract_slot_id(side, constraints.get('target_slot') or '')
    target_shuttle = _canonical_shuttle_id(
        constraints.get('target_shuttle') or '',
        side=side,
    )
    return bool(
        target_slot
        and target_shuttle
        and arrival.get('target_slot_contract') == target_slot
        and arrival.get('shuttle_contract') == target_shuttle
    )


def _verify_device_state(
    observed_state: ObservedState,
    *,
    side: str,
    device_kind: str,
    assignments: dict[str, Any],
    command_to_state: Any,
) -> PostconditionCheck:
    expanded = _expand_device_assignments(assignments)
    for device, raw_value in expanded.items():
        expected = command_to_state(raw_value)
        subject = f'{side}:{device_kind}:{device}'
        fact = _fact(observed_state, subject, 'state')
        if fact is None:
            return PostconditionCheck(
                status='unknown',
                reason=f'missing_{device_kind}_{device}_state',
                details={'subject': subject},
            )
        if fact.status != 'known':
            return PostconditionCheck(
                status='unknown',
                reason=f'{device_kind}_{device}_{fact.status}',
                details={'subject': subject},
            )
        observed_raw = str(fact.value or '').strip()
        observed = command_to_state(observed_raw)
        if observed.casefold() != str(expected).casefold():
            return PostconditionCheck(
                status='mismatch',
                reason=f'{device_kind}_{device}_expected_{expected}_observed_{observed}',
                details={
                    'subject': subject,
                    'expected': expected,
                    'observed': observed,
                    'observed_raw': observed_raw,
                },
            )
    return PostconditionCheck(
        status='satisfied',
        reason=f'{device_kind}_state_verified',
        details={'assignments': expanded},
    )


def _expand_device_assignments(assignments: dict[str, Any]) -> dict[str, Any]:
    if 'ALL' in assignments:
        return {device: assignments['ALL'] for device in DEVICE_NAMES}
    return {
        str(device).strip().upper(): value
        for device, value in assignments.items()
        if str(device).strip().upper() in DEVICE_NAMES
    }


def _canonical_switch_effect_state(value: Any) -> str:
    text = str(value or '').strip().upper()
    if text in {'E', 'EXTERIOR', 'EXTERNAL'}:
        return 'EXTERIOR'
    if text in {'I', 'INTERIOR', 'INTERNAL'}:
        return 'INTERIOR'
    return text


def _canonical_stopper_effect_state(value: Any) -> str:
    text = str(value or '').strip().casefold()
    if text in {'0', 'off', 'open', 'opened', 'release', 'released', 'false'}:
        return 'open'
    if text in {'1', 'on', 'close', 'closed', 'stop', 'blocked', 'true'}:
        return 'closed'
    return text


def _verify_shuttle_stopped(
    observed_state: ObservedState,
    shuttle: str,
    side: str,
) -> PostconditionCheck:
    selected = _canonical_shuttle_id(shuttle, side=side)
    fact = _first_fact(
        observed_state,
        subjects=(selected, _gazebo_entity_for_shuttle(selected), shuttle),
        predicate='motion_mode',
    )
    if fact is None:
        return PostconditionCheck(
            status='unknown',
            reason='missing_shuttle_motion_mode_stop_proof',
        )
    if fact.status != 'known':
        return PostconditionCheck(status='unknown', reason=f'shuttle_motion_{fact.status}')
    if str(fact.value) in STOPPED_MOTION_VALUES:
        return PostconditionCheck(status='satisfied', reason='shuttle_stopped_verified')
    return PostconditionCheck(
        status='mismatch',
        reason=f'shuttle_expected_stopped_observed_{fact.value}',
    )


def _verify_slot_occupancy(
    observed_state: ObservedState,
    target_slot: str,
    expected_shuttle: str,
) -> PostconditionCheck:
    side, slot_number = _split_slot_id(target_slot)
    subject = _contract_slot_id(side, slot_number)
    fact = _fact(observed_state, subject, 'occupancy')
    if fact is None:
        return PostconditionCheck(
            status='unknown',
            reason=f'missing_slot_{target_slot}_occupancy',
            details={'subject': subject},
        )
    if fact.status != 'known':
        return PostconditionCheck(
            status='unknown',
            reason=f'slot_{target_slot}_occupancy_{fact.status}',
            details={'subject': subject},
        )
    occupant = _occupancy_shuttle(fact.value, side=side)
    if occupant == expected_shuttle:
        return PostconditionCheck(
            status='satisfied',
            reason='target_slot_occupancy_verified',
            details={'target_slot': target_slot, 'shuttle': expected_shuttle},
        )
    return PostconditionCheck(
        status='mismatch',
        reason=f'target_slot_expected_{expected_shuttle}_observed_{occupant or "empty"}',
        details={'target_slot': target_slot, 'expected': expected_shuttle, 'observed': occupant},
    )


def _target_slot_for_step(
    step: PddlPlanStep,
    task_goal: TaskGoal,
    problem: Room315PddlProblem,
) -> str:
    provenance = dict(problem.provenance or {})
    if provenance.get('planning_phase') == 'clear_blocker_to_slot':
        relocation = dict(provenance.get('clearance_relocation') or {})
        destination = dict(relocation.get('destination') or {})
        raw_parking_slot = str(destination.get('target_slot') or '').strip()
        if not raw_parking_slot:
            raise RuntimeError(
                'isolated blocker parking problem has no audited target slot'
            )
        parking_side, parking_slot = _split_slot_id(
            raw_parking_slot,
            default_side=problem.side,
        )
        audited_target = _contract_slot_id(parking_side, parking_slot)
        if step.name in {
            'move_shuttle_to_slot',
            'move_shuttle_from_segment_to_slot',
            'move_shuttle_via_topology_to_slot',
        } and len(step.args) >= 4:
            planned_target = _contract_slot_id(problem.side, step.args[-1])
            if planned_target != audited_target:
                raise RuntimeError(
                    'PlanSys2 blocker parking target does not match audited '
                    f'clearance destination: planned={planned_target}, '
                    f'audited={audited_target}'
                )
        # PlanSysPlannerBackend canonicalizes move_shuttle_to_slot to the
        # legacy move_shuttle spelling, which omits slot arguments.  In this
        # isolated phase the frozen clearance provenance, not the parent
        # TaskGoal destination, is therefore authoritative for stopping.
        return audited_target
    if step.name in {
        'move_shuttle_to_slot',
        'move_shuttle_from_segment_to_slot',
        'move_shuttle_via_topology_to_slot',
    } and len(step.args) >= 4:
        return _contract_slot_id(problem.side, step.args[-1])
    constraints = dict(task_goal.constraints or {})
    if constraints.get('target_slot'):
        return _contract_slot_id(
            str(constraints.get('side') or problem.side),
            constraints['target_slot'],
        )
    if problem.target_slot:
        return _contract_slot_id(problem.side, problem.target_slot)
    return ''


def _task_goal_target_shuttle(task_goal: TaskGoal, *, side: str) -> str:
    constraints = dict(task_goal.constraints or {})
    raw = constraints.get('target_shuttle') or ''
    return _canonical_shuttle_id(raw, side=side) if raw else ''


def _occupancy_snapshot(observed_state: ObservedState) -> dict[str, tuple[str, str]]:
    snapshot: dict[str, tuple[str, str]] = {}
    for fact in observed_state.fused_planner_state:
        if fact.predicate != 'occupancy' or ':slot:' not in fact.subject:
            continue
        side = _side_from_subject(fact.subject)
        occupant = _occupancy_shuttle(fact.value, side=side) if fact.status == 'known' else ''
        snapshot[fact.subject] = (fact.status, occupant)
    return snapshot


def _fact(observed_state: ObservedState, subject: str, predicate: str) -> ObservedFact | None:
    for fact in observed_state.fused_planner_state:
        if fact.subject == subject and fact.predicate == predicate:
            return fact
    return None


def _first_fact(
    observed_state: ObservedState,
    *,
    subjects: tuple[str, ...],
    predicate: str,
) -> ObservedFact | None:
    subject_set = {subject for subject in subjects if subject}
    for fact in observed_state.fused_planner_state:
        if fact.subject in subject_set and fact.predicate == predicate:
            return fact
    return None


def _loaded_state(observed_state: ObservedState, shuttle: str, *, side: str) -> bool | None:
    selected = _canonical_shuttle_id(shuttle, side=side)
    fact = _first_fact(
        observed_state,
        subjects=(selected, _gazebo_entity_for_shuttle(selected), shuttle),
        predicate='loaded',
    )
    if fact is None or fact.status != 'known':
        return None
    return bool(fact.value)


def _occupancy_shuttle(value: Any, *, side: str) -> str:
    raw = None
    if isinstance(value, dict):
        raw = value.get('shuttle')
    elif value:
        raw = value
    if not raw:
        return ''
    return _canonical_shuttle_id(str(raw), side=side)


def _canonical_shuttle_id(value: str, *, side: str) -> str:
    spec = normalize_shuttle_ref(value, side=side)
    if spec is not None:
        return spec.shuttle_id
    text = str(value or '').strip().upper()
    lowered = text.casefold()
    for prefix, prefix_side in (
        ('room315_right_shuttle_', 'right'),
        ('room315_left_shuttle_', 'left'),
    ):
        if lowered.startswith(prefix):
            index_text = lowered.rsplit('_', 1)[-1]
            if index_text.isdigit():
                spec = normalize_shuttle_ref(
                    f'{"R" if prefix_side == "right" else "L"}{index_text}',
                    side=prefix_side,
                )
                if spec is not None:
                    return spec.shuttle_id
    if len(text) == 2 and text[0] in {'R', 'L'} and text[1].isdigit():
        prefix_side = 'right' if text[0] == 'R' else 'left'
        spec = normalize_shuttle_ref(text, side=prefix_side)
        if spec is not None:
            return spec.shuttle_id
    return str(value or '').strip()


def _gazebo_entity_for_shuttle(shuttle_id: str) -> str:
    text = str(shuttle_id or '').strip()
    lowered = text.casefold()
    if lowered.startswith(('right_shuttle_', 'left_shuttle_')):
        side = 'right' if lowered.startswith('right_') else 'left'
        index = lowered.rsplit('_', 1)[-1]
        if index.isdigit():
            return f'room315_{side}_shuttle_{index}'
    if len(text) == 2 and text[0] in {'R', 'L'} and text[1].isdigit():
        side = 'right' if text[0] == 'R' else 'left'
        index = int(text[1])
        return f'room315_{side}_shuttle_{index}'
    return text


def _contract_slot_id(side: str, slot: Any) -> str:
    side = str(side or '').strip().casefold()
    text = str(slot or '').strip().casefold()
    if ':slot:' in text:
        return text
    if text.startswith(('right_slot_', 'left_slot_')):
        parts = text.split('_')
        return f'{parts[0]}:slot:{parts[-1]}'
    if text.startswith('slot_'):
        return f'{side}:slot:{text.split("_")[-1]}'
    if text.isdigit():
        return f'{side}:slot:{text}'
    return text.replace('_', ':')


def _split_slot_id(slot_id: str, *, default_side: str = 'right') -> tuple[str, str]:
    text = _contract_slot_id(default_side, slot_id)
    if ':slot:' in text:
        side, slot_number = text.split(':slot:', 1)
        return side, slot_number
    if text.startswith(('right_slot_', 'left_slot_')):
        parts = text.split('_')
        return parts[0], parts[-1]
    return default_side, text


def _side_from_subject(subject: str) -> str:
    text = str(subject or '').strip().casefold()
    if text.startswith('left'):
        return 'left'
    return 'right'
