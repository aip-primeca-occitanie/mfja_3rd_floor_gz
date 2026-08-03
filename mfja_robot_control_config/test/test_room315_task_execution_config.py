#!/usr/bin/env python3

from __future__ import annotations

import copy
import sys
from pathlib import Path

import pytest
import yaml


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_task_execution_config import RUNTIME_PDDL_DOMAIN_PATH
from room_315_task_execution_config import TASK_EXECUTION_PARAMETER_DEFAULTS
from room_315_task_execution_config import validate_task_execution_parameters
from room_315_closed_loop_executive import ClosedLoopExecutiveConfig


CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'room_315_vla'
    / 'task_execution_runtime.yaml'
)


def test_runtime_parameter_defaults_are_valid_and_domain_exists():
    validate_task_execution_parameters(
        dict(TASK_EXECUTION_PARAMETER_DEFAULTS)
    )
    assert RUNTIME_PDDL_DOMAIN_PATH.is_file()


def test_runtime_yaml_and_node_fallback_defaults_cannot_drift():
    loaded = yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))
    configured = loaded['room_315_task_execution_node']['ros__parameters']
    defaults = dict(TASK_EXECUTION_PARAMETER_DEFAULTS)

    assert set(configured) == set(defaults) - {'planner_domain_path'}
    for name, value in configured.items():
        assert value == defaults[name], name


def test_executive_defaults_match_authoritative_runtime_defaults():
    executive = ClosedLoopExecutiveConfig()
    defaults = TASK_EXECUTION_PARAMETER_DEFAULTS

    for name in (
        'speed_mps',
        'max_steps',
        'max_replans',
        'max_unknown_retries',
        'supervisor_timeout_s',
        'effect_timeout_s',
        'clearance_effect_timeout_s',
        'route_arrival_timeout_scale',
        'route_arrival_timeout_margin_s',
    ):
        assert getattr(executive, name) == defaults[name], name
    assert not hasattr(executive, 'planning_timeout_s')


@pytest.mark.parametrize(
    ('name', 'value', 'message'),
    (
        (
            'planner_timeout_s',
            15.0,
            'must exceed the PlanSys2 solver timeout',
        ),
        (
            'payload_grounding_confirmation_frames',
            2,
            'must be at least 3',
        ),
        (
            'payload_grounding_max_observations',
            4,
            'must be no smaller',
        ),
        (
            'clearance_effect_timeout_s',
            20.0,
            'must be no smaller',
        ),
        (
            'route_arrival_timeout_scale',
            0.99,
            'must be at least one',
        ),
        ('speed_mps', 0.0, 'must be greater than zero'),
        ('speed_mps', float('nan'), 'must be greater than zero'),
    ),
)
def test_runtime_parameter_validation_fails_closed(name, value, message):
    parameters = copy.deepcopy(TASK_EXECUTION_PARAMETER_DEFAULTS)
    parameters[name] = value

    with pytest.raises(ValueError, match=message):
        validate_task_execution_parameters(parameters)
