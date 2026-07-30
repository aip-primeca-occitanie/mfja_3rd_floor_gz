#!/usr/bin/env python3
"""Closed-loop Room 315 symbolic executive.

This module is the production loop around the existing boundaries:
ObservedState -> PDDL problem -> PlanSys2 plan -> one translated primitive
command -> supervisor -> re-observe. It intentionally executes only the first
atomic symbolic step from each validated plan and then replans from fresh state.
"""

from __future__ import annotations

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
from room_315_pddl_scenario_generator import PddlProblemBuildError
from room_315_pddl_scenario_generator import Room315PddlProblem
from room_315_pddl_scenario_generator import ScenarioTransport
from room_315_pddl_scenario_generator import SLOT_SENSOR_BY_SIDE_AND_SLOT
from room_315_pddl_scenario_generator import SLOT_STATION_BY_SIDE_AND_SLOT
from room_315_pddl_scenario_generator import build_pddl_problem_from_observed_state_task_goal


TERMINAL_SUCCESS_STEPS = {'finish_task', 'finish_candidate_task', 'inspect_state'}
RECOVERABLE_STATE_STATUSES = {'unknown', 'stale', 'conflicting'}
STOPPED_MOTION_VALUES = {'STOPPED', 'OFF', 'IDLE', 'HALTED', 'stopped', 'off', 'idle', 'halted'}


@dataclass(frozen=True)
class ClosedLoopExecutiveConfig:
    """Runtime limits for the closed-loop executive."""

    speed_mps: float = 0.3
    max_steps: int = 32
    max_replans: int = 8
    max_unknown_retries: int = 3
    supervisor_timeout_s: float = 10.0
    effect_timeout_s: float = 20.0
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

    def run(self, task_goal: TaskGoal) -> ClosedLoopExecutiveResult:
        """Execute a TaskGoal by replanning after every atomic supervised step."""

        if not isinstance(task_goal, TaskGoal):
            raise TypeError('task_goal must be a TaskGoal contract')

        unknown_retries = 0
        replans = 0
        final_state_id = ''

        for step_index in range(self.config.max_steps):
            observed_state = self._observe()
            final_state_id = observed_state.state_id

            if self._task_goal_satisfied(observed_state, task_goal):
                return self._result(
                    status='succeeded',
                    reason='task_goal_already_satisfied',
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
                )
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

            supervisor_status = self._execute_first_atomic_step(
                translated_step=translated_step,
                problem=problem,
                task_goal=task_goal,
                plan_length=len(plan),
                step_index=step_index,
            )
            if supervisor_status != 'accepted':
                self._safe_abort(f'supervisor_{supervisor_status}')
                return self._result(
                    status='aborted',
                    reason=f'supervisor_{supervisor_status}',
                    replans=replans,
                    unknown_retries=unknown_retries,
                    final_state_id=final_state_id,
                )

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

            after_state = self._observe()
            final_state_id = after_state.state_id
            after_occupancy = _occupancy_snapshot(after_state)
            occupancy_changed = before_occupancy != after_occupancy
            postcondition = self._verify_expected_effects(
                before_state=observed_state,
                after_state=after_state,
                step=first_step,
                translated_step=translated_step,
                task_goal=task_goal,
                problem=problem,
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

            if postcondition.satisfied:
                if first_step.name in TERMINAL_SUCCESS_STEPS or self._task_goal_satisfied(
                    after_state,
                    task_goal,
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

    def _observe(self) -> ObservedState:
        state = self.observed_state_provider.observe(timestamp=time.time())
        if not isinstance(state, ObservedState):
            raise TypeError('ObservedStateProvider.observe() must return ObservedState')
        self._observations += 1
        return state

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
                    problem.target_station
                    or SLOT_STATION_BY_SIDE_AND_SLOT.get((side, slot_number), '')
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
                command_to_state=lambda value: str(value or '').strip().upper(),
            )
        if action == 'stoppers':
            return _verify_device_state(
                after_state,
                side=str(command.get('side') or problem.side),
                device_kind='stopper',
                assignments=dict(command.get('stoppers') or {}),
                command_to_state=lambda value: 'closed' if str(value) == '1' else 'open',
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
            return PostconditionCheck(status='satisfied', reason=f'{step.name}_terminal')

        if before_state.state_id == after_state.state_id:
            return PostconditionCheck(status='satisfied', reason='accepted_no_state_effect')
        return PostconditionCheck(status='satisfied', reason='accepted_and_reobserved')

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
            required_payload = str(constraints.get('payload_required') or '').casefold()
            if required_payload in {'loaded', 'empty'}:
                loaded = _loaded_state(observed_state, occupant, side=side)
                if loaded is None:
                    return False
                if required_payload == 'loaded' and not loaded:
                    return False
                if required_payload == 'empty' and loaded:
                    return False
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
        return PostconditionCheck(status='satisfied', reason=f'{label}_wait_satisfied')
    reason = str(result.get('reason') or f'{label}_timeout')
    return PostconditionCheck(status='timed_out', reason=reason, details=dict(result))


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
        observed = str(fact.value or '').strip()
        if observed.casefold() != str(expected).casefold():
            return PostconditionCheck(
                status='mismatch',
                reason=f'{device_kind}_{device}_expected_{expected}_observed_{observed}',
                details={'subject': subject, 'expected': expected, 'observed': observed},
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
        return PostconditionCheck(status='satisfied', reason='shuttle_stop_wait_verified')
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
    if step.name == 'move_shuttle_to_slot' and len(step.args) >= 4:
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
