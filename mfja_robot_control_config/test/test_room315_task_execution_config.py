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
from room_315_task_execution_config import (
    TASK_EXECUTION_AUTHORIZATION_PARAMETER_DEFAULTS,
)
from room_315_task_execution_config import TASK_EXECUTION_PARAMETER_DEFAULTS
from room_315_task_execution_config import validate_task_execution_parameters
from room_315_closed_loop_executive import ClosedLoopExecutiveConfig


CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'room_315_task_execution'
    / 'task_execution_runtime.yaml'
)
ACTIVE_RUNTIME_BUNDLE = Path(
    '/home/tiago/room315_visual_runtime_candidate_v4_seed31520260811_'
    'epoch11_869d6404_closed_loop_runtime_attempt1'
)
ACTIVE_RUNTIME_MANIFEST_SHA256 = (
    '506cae0511cf1675fdd666103ce7fc0b5980eb5e68d4cbadf0af99d9ee9560da'
)
ACTIVE_RUNTIME_STATE_SHA256 = (
    '14cedafe28c999786a66934a523db5757e1ccdd7ae34705d5a2df58488fc8df1'
)
V4_CHECKPOINT_SHA256 = (
    '869d64049b0092c37d21a4c8b910dc6b91954527e0e49c5694fa82dce570f40d'
)


def test_runtime_parameter_defaults_are_valid_and_domain_exists():
    validate_task_execution_parameters(
        dict(TASK_EXECUTION_PARAMETER_DEFAULTS)
    )
    assert RUNTIME_PDDL_DOMAIN_PATH.is_file()


def test_runtime_yaml_and_node_defaults_cannot_drift():
    loaded = yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))
    configured = loaded['room_315_task_execution_node']['ros__parameters']
    defaults = dict(TASK_EXECUTION_PARAMETER_DEFAULTS)
    authorization_names = set(TASK_EXECUTION_AUTHORIZATION_PARAMETER_DEFAULTS)
    configured_without_authorization = {
        name: value
        for name, value in configured.items()
        if name not in authorization_names
    }

    assert set(configured_without_authorization) == (
        set(defaults) - {'planner_domain_path'} - authorization_names
    )
    for name, value in configured_without_authorization.items():
        assert value == defaults[name], name
    assert set(configured) & authorization_names == authorization_names


def test_active_runtime_yaml_is_v4_pinned_disabled_and_authorized():
    loaded = yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))
    configured = loaded['room_315_task_execution_node']['ros__parameters']
    manifest_path = ACTIVE_RUNTIME_BUNDLE / 'runtime_promotion_manifest.json'
    state_path = ACTIVE_RUNTIME_BUNDLE / 'candidate_state.json'

    assert configured['execution_enabled'] is False
    assert configured['allowed_visual_schema_version'] == (
        'room315.visual_state.v4'
    )
    assert configured['allowed_visual_checkpoint_sha256'] == (
        V4_CHECKPOINT_SHA256
    )
    assert configured['task_execution_authorization_path'] == str(state_path)
    assert configured['task_execution_authorization_sha256'] == (
        ACTIVE_RUNTIME_STATE_SHA256
    )
    assert configured['task_execution_promotion_manifest_path'] == str(
        manifest_path
    )

    enabled = dict(TASK_EXECUTION_PARAMETER_DEFAULTS)
    enabled.update(configured)
    enabled['execution_enabled'] = True
    verified = validate_task_execution_parameters(enabled)
    assert verified is not None
    assert verified['authorization_scope'] == (
        'gazebo_v4_closed_loop_runtime_only'
    )
    assert verified['sha256'] == ACTIVE_RUNTIME_STATE_SHA256
    assert verified['promotion_manifest_sha256'] == (
        ACTIVE_RUNTIME_MANIFEST_SHA256
    )


def test_v3_visual_allowlist_is_rejected_even_when_execution_is_disabled():
    configured = dict(TASK_EXECUTION_PARAMETER_DEFAULTS)
    configured['allowed_visual_schema_version'] = 'room315.visual_state.v3'

    with pytest.raises(
        ValueError,
        match='allowed_visual_schema_version must be room315.visual_state.v4',
    ):
        validate_task_execution_parameters(configured)


def test_non_authorized_visual_hash_is_rejected_when_execution_is_disabled():
    configured = dict(TASK_EXECUTION_PARAMETER_DEFAULTS)
    configured['allowed_visual_checkpoint_sha256'] = (
        '8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d'
    )

    with pytest.raises(
        ValueError,
        match='must be the exact authorized V4 checkpoint',
    ):
        validate_task_execution_parameters(configured)


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
