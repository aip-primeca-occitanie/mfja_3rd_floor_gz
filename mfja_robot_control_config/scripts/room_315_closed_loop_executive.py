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
from room_315_pddl_scenario_generator import INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M
from room_315_pddl_scenario_generator import INTERIOR_LOOP_ENTRY_SENSOR_BY_SIDE_AND_GATE
from room_315_pddl_scenario_generator import PddlProblemBuildError
from room_315_pddl_scenario_generator import Room315PddlProblem
from room_315_pddl_scenario_generator import ScenarioTransport
from room_315_pddl_scenario_generator import SLOT_SENSOR_BY_SIDE_AND_SLOT
from room_315_pddl_scenario_generator import SLOT_STATION_BY_SIDE_AND_SLOT
from room_315_pddl_scenario_generator import build_first_blocker_clearance_problem
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

        for step_index in range(self.config.max_steps):
            observed_state = self._observe()
            final_state_id = observed_state.state_id
            self._refresh_runtime_clearance_certificates()

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

            if first_step.name == 'begin_route_clearance':
                supervisor_status, wait_check = self._enter_route_clearance_mode(
                    side=problem.side,
                    problem=problem,
                    plan_length=len(plan),
                    step_index=step_index,
                    symbolic_step=_step_text(first_step),
                )
            elif first_step.name == 'relocate_blocker_to_interior':
                supervisor_status, wait_check = self._execute_interior_clearance(
                    translated_step=translated_step,
                    problem=problem,
                    plan_length=len(plan),
                    step_index=step_index,
                )
            elif first_step.name == 'finish_route_clearance':
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
            elif first_step.name == 'prepare_topology_route':
                supervisor_status, wait_check = self._prepare_topology_route(
                    translated_step=translated_step,
                    problem=problem,
                    plan_length=len(plan),
                    step_index=step_index,
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
                self._safe_abort(f'supervisor_{supervisor_status}')
                return self._result(
                    status='aborted',
                    reason=f'supervisor_{supervisor_status}',
                    replans=replans,
                    unknown_retries=unknown_retries,
                    final_state_id=final_state_id,
                )

            if first_step.name not in {
                'begin_route_clearance',
                'relocate_blocker_to_interior',
                'finish_route_clearance',
                'prepare_topology_route',
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

            if first_step.name == 'begin_route_clearance' and wait_check.satisfied:
                self._route_clearance_active_side = problem.side
            elif (
                first_step.name == 'finish_route_clearance'
                and wait_check.satisfied
            ):
                self._route_clearance_active_side = ''

            clearance_certificate: dict[str, Any] | None = None
            if first_step.name == 'relocate_blocker_to_interior':
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
            if first_step.name == 'relocate_blocker_to_interior':
                postcondition = self._verify_interior_clearance(
                    after_state=after_state,
                    problem=problem,
                    translated_step=translated_step,
                    clearance_certificate=clearance_certificate,
                )
            elif first_step.name in {
                'begin_route_clearance',
                'finish_route_clearance',
                'prepare_topology_route',
            }:
                postcondition = wait_check
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
                first_step.name == 'relocate_blocker_to_interior'
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

        phase_actions = {
            'begin_route_clearance',
            'relocate_blocker_to_interior',
            'finish_route_clearance',
        }
        active_side = self._route_clearance_active_side
        if active_side:
            if side != active_side:
                return f'active_side={active_side},planned_side={side}'
            if first_step.name not in {
                'relocate_blocker_to_interior',
                'finish_route_clearance',
            }:
                return (
                    f'active_side={active_side},forbidden_action='
                    f'{first_step.name}'
                )
            return ''
        if first_step.name in phase_actions - {'begin_route_clearance'}:
            return f'clearance_not_started,action={first_step.name}'
        return ''

    def _observe(self) -> ObservedState:
        state = self.observed_state_provider.observe(timestamp=time.time())
        if not isinstance(state, ObservedState):
            raise TypeError('ObservedStateProvider.observe() must return ObservedState')
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
        """Isolate a free-slot blocker move before the parent transport."""

        clearance = dict(
            (problem.provenance or {}).get(
                'target_blocker_clearance_plan'
            ) or {}
        )
        relocations = list(clearance.get('ordered_relocations') or [])
        if not relocations:
            return problem
        destination = dict(relocations[0].get('destination') or {})
        if str(destination.get('kind') or '').strip().casefold() != 'slot':
            return problem
        return build_first_blocker_clearance_problem(problem)

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
        if gate != 'A3' or not target_segment:
            return 'rejected', PostconditionCheck(
                status='unknown',
                reason='clearance_destination_is_not_the_authoritative_a3_loop',
            )

        common = {
            'mode': 'plansys2_supervised_interior_clearance',
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
            'target_stopper': 'A4',
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
                target_s_m
                / float(
                    translated_step.command.get('speed')
                    or self.config.speed_mps
                )
            ),
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

        switches = {
            'A1': 'EXTERIOR',
            'A2': 'EXTERIOR',
            'A3': 'INTERIOR',
            'A4': 'INTERIOR',
        }
        stoppers = {'A1': '0', 'A2': '0', 'A3': '0', 'A4': '1'}
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
                'restore_deferred_to_finish_route_clearance': True,
            },
        )

    @staticmethod
    def _clearance_phase_metadata(
        *,
        mode: str,
        problem: Room315PddlProblem,
        plan_length: int,
        step_index: int,
        symbolic_step: str,
    ) -> dict[str, Any]:
        return {
            'mode': mode,
            'problem_name': problem.problem_name,
            'plan_length': plan_length,
            'step_index': step_index,
            'symbolic_step': symbolic_step,
            'localization_source': 'accepted_visual_state',
            'controller_position_fields_used_for_localization': False,
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
            'mode': 'restore_normal_route_after_interior_clearance',
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
            reason='normal_route_restored_after_interior_clearance',
            details={
                'switches': exterior_switches,
                'stoppers': open_stoppers,
                'fresh_visual_frame': dict(fresh_visual.details),
            },
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
        required_true = (
            'entry_sensor_identity_confirmed',
            'controller_stop_confirmed',
            'post_stop_visual_frame_received',
        )
        for name in required_true:
            if not bool(details.get(name)):
                return None, f'missing_{name}'
        if str(details.get('entry_sensor') or '').upper() != expected_sensor:
            return None, (
                'wrong_entry_sensor:'
                f'expected={expected_sensor},observed='
                f'{str(details.get("entry_sensor") or "").upper()}'
            )
        if details.get('matched_by') != (
            'interior_entry_sensor_plus_bounded_travel_time'
        ):
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
            and certificate.get('matched_by') == (
                'interior_entry_sensor_plus_bounded_travel_time'
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
