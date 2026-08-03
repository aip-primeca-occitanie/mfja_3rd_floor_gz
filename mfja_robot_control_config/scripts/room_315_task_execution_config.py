#!/usr/bin/env python3
"""Authoritative defaults and validation for Room 315 task execution."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from room_315_runtime_contracts import MIN_PAYLOAD_CONFIRMATION_FRAMES


SCRIPT_DIR = Path(__file__).resolve().parent
RUNTIME_PDDL_DOMAIN_PATH = (
    SCRIPT_DIR.parent
    / 'config'
    / 'room_315_vla'
    / 'pddl'
    / 'domain_room315_runtime.pddl'
)
PLANSYS2_RUNTIME_SOLVER_TIMEOUT_S = 15.0
DEFAULT_PLANNER_CLIENT_TIMEOUT_S = 20.0
DEFAULT_PAYLOAD_GROUNDING_CONFIRMATION_FRAMES = 5
DEFAULT_PAYLOAD_GROUNDING_MAX_OBSERVATIONS = 15

# ``arrival_confirmation_frames`` is accepted only so an older launch override
# does not fail parameter declaration. Runtime arrival is sensor-confirmed.
TASK_EXECUTION_COMPATIBILITY_PARAMETER_DEFAULTS = {
    'arrival_confirmation_frames': 3,
}

TASK_EXECUTION_ACTIVE_PARAMETER_DEFAULTS = {
    'use_sim_time': True,
    'execution_enabled': False,
    'task_goal_topic': '/room_315/task_goal',
    'task_status_topic': '/room_315/task_goal/status',
    'accepted_observed_state_topic': '/room_315/visual_state/observed_state',
    'supervisor_command_topic': '/room_315/vla/command',
    'supervisor_status_topic': '/room_315/vla/status',
    'left_sensor_feedback_topic': '/room_315/rails/left/sensors/feedback',
    'right_sensor_feedback_topic': '/room_315/rails/right/sensors/feedback',
    'diagnostics_topic': '/diagnostics',
    'planner_service': '/planner/get_plan',
    'planner_domain_path': str(RUNTIME_PDDL_DOMAIN_PATH),
    'planner_timeout_s': DEFAULT_PLANNER_CLIENT_TIMEOUT_S,
    'observation_timeout_s': 1.5,
    'supervisor_status_timeout_s': 1.5,
    'slot_sensor_state_timeout_s': 1.0,
    'observation_wait_s': 2.0,
    'planning_slot_tolerance_ratio': 0.12,
    'target_arrival_tolerance_ratio': 0.05,
    'position_consistency_tolerance_m': 0.08,
    'slot_sensor_confirmation_frames': 2,
    'payload_grounding_confirmation_frames': (
        DEFAULT_PAYLOAD_GROUNDING_CONFIRMATION_FRAMES
    ),
    'payload_grounding_max_observations': (
        DEFAULT_PAYLOAD_GROUNDING_MAX_OBSERVATIONS
    ),
    'controller_stop_timeout_s': 3.0,
    'speed_mps': 0.2,
    'max_steps': 32,
    'max_replans': 8,
    'max_unknown_retries': 3,
    'supervisor_timeout_s': 5.0,
    'effect_timeout_s': 30.0,
    'clearance_effect_timeout_s': 60.0,
    # Final arrival remains sensor-gated, but its deadline must cover the
    # authoritative forward-only route rather than assuming every move is an
    # adjacent-slot move.
    'route_arrival_timeout_scale': 1.25,
    'route_arrival_timeout_margin_s': 5.0,
    'external_obstacles_disabled': True,
    'diagnostic_period_s': 1.0,
}

TASK_EXECUTION_PARAMETER_DEFAULTS = {
    **TASK_EXECUTION_ACTIVE_PARAMETER_DEFAULTS,
    **TASK_EXECUTION_COMPATIBILITY_PARAMETER_DEFAULTS,
}

_POSITIVE_FLOAT_PARAMETERS = frozenset({
    'planner_timeout_s',
    'observation_timeout_s',
    'supervisor_status_timeout_s',
    'slot_sensor_state_timeout_s',
    'observation_wait_s',
    'planning_slot_tolerance_ratio',
    'target_arrival_tolerance_ratio',
    'position_consistency_tolerance_m',
    'controller_stop_timeout_s',
    'speed_mps',
    'supervisor_timeout_s',
    'effect_timeout_s',
    'clearance_effect_timeout_s',
    'route_arrival_timeout_scale',
    'route_arrival_timeout_margin_s',
    'diagnostic_period_s',
})
_NON_NEGATIVE_INTEGER_PARAMETERS = frozenset({
    'max_replans',
    'max_unknown_retries',
})
_POSITIVE_INTEGER_PARAMETERS = frozenset({
    'max_steps',
    'slot_sensor_confirmation_frames',
    'payload_grounding_confirmation_frames',
    'payload_grounding_max_observations',
})


def validate_task_execution_parameters(parameters: dict[str, Any]) -> None:
    """Reject inconsistent runtime overrides before subscriptions start."""

    missing = sorted(
        name
        for name in TASK_EXECUTION_ACTIVE_PARAMETER_DEFAULTS
        if name not in parameters
    )
    if missing:
        raise ValueError(f'missing task-execution parameters:{missing}')
    for name in _POSITIVE_FLOAT_PARAMETERS:
        value = float(parameters[name])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
    for name in _NON_NEGATIVE_INTEGER_PARAMETERS:
        if int(parameters[name]) < 0:
            raise ValueError(f'{name} must be non-negative')
    for name in _POSITIVE_INTEGER_PARAMETERS:
        if int(parameters[name]) < 1:
            raise ValueError(f'{name} must be at least one')
    for name in (
        'planning_slot_tolerance_ratio',
        'target_arrival_tolerance_ratio',
    ):
        if float(parameters[name]) > 1.0:
            raise ValueError(f'{name} must be no greater than one')
    if float(parameters['planner_timeout_s']) <= (
        PLANSYS2_RUNTIME_SOLVER_TIMEOUT_S
    ):
        raise ValueError(
            'planner_timeout_s must exceed the PlanSys2 solver timeout '
            f'({PLANSYS2_RUNTIME_SOLVER_TIMEOUT_S:.1f}s)'
        )
    confirmation_frames = int(
        parameters['payload_grounding_confirmation_frames']
    )
    if confirmation_frames < MIN_PAYLOAD_CONFIRMATION_FRAMES:
        raise ValueError(
            'payload_grounding_confirmation_frames must be at least '
            f'{MIN_PAYLOAD_CONFIRMATION_FRAMES}'
        )
    if int(parameters['payload_grounding_max_observations']) < (
        confirmation_frames
    ):
        raise ValueError(
            'payload_grounding_max_observations must be no smaller than '
            'payload_grounding_confirmation_frames'
        )
    if float(parameters['clearance_effect_timeout_s']) < float(
        parameters['effect_timeout_s']
    ):
        raise ValueError(
            'clearance_effect_timeout_s must be no smaller than '
            'effect_timeout_s'
        )
    if float(parameters['route_arrival_timeout_scale']) < 1.0:
        raise ValueError(
            'route_arrival_timeout_scale must be at least one'
        )
