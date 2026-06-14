#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path
from types import MethodType
from types import SimpleNamespace

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_real_vla_agent.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_real_vla_agent', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _multi_status():
    return {
        'rails': {
            'right': {
                'shuttles': {
                    'room315_right_shuttle_1': {'segment': 'A12E'},
                    'room315_right_shuttle_2': {'segment': 'A23E'},
                },
                'switches': {},
                'stoppers': {},
                'active_sensors': [],
                'active_position_sensors': [],
            },
            'left': {'shuttles': {}, 'switches': {}, 'stoppers': {}},
        }
    }


def _valid_v3_vector(agent):
    vector = [0.0] * len(agent.EVENT_ACTION_VECTOR_FIELDS)
    vector[agent.EVENT_ACTION_VECTOR_FIELDS.index('primitive_id')] = 4
    vector[agent.EVENT_ACTION_VECTOR_FIELDS.index('side_id')] = 0
    vector[agent.EVENT_ACTION_VECTOR_FIELDS.index('shuttle_index')] = 1
    vector[agent.EVENT_ACTION_VECTOR_FIELDS.index('speed_mps')] = 0.25
    vector[agent.EVENT_ACTION_VECTOR_FIELDS.index('wait_condition_id')] = 3
    vector[agent.EVENT_ACTION_VECTOR_FIELDS.index('target_id')] = agent.TARGET_IDS['right_shuttle_2']
    vector[agent.EVENT_ACTION_VECTOR_FIELDS.index('reason_id')] = agent.REASON_IDS['target_station_route']
    vector[agent.EVENT_ACTION_VECTOR_FIELDS.index('coordination_mode')] = (
        agent.COORDINATION_MODE_IDS['guarded_motion']
    )
    return vector


def test_agent_accepts_valid_schema_v3_multi_shuttle_vector():
    agent = _load_module()
    vector = _valid_v3_vector(agent)

    parsed = agent._parse_command_payload(
        {'action_vector_schema_version': 3, 'action_vector': vector},
        multi_shuttle_active=True,
    )

    assert parsed['action_vector_schema_version'] == 3
    assert parsed['action_vector'] == [float(value) for value in vector]


def test_agent_rejects_multi_shuttle_vector_missing_shuttle_index():
    agent = _load_module()
    vector = _valid_v3_vector(agent)
    vector[agent.EVENT_ACTION_VECTOR_FIELDS.index('shuttle_index')] = -1

    with pytest.raises(ValueError, match='requires shuttle_index'):
        agent._parse_command_payload(
            {'action_vector_schema_version': 3, 'action_vector': vector},
            multi_shuttle_active=True,
        )


def test_agent_rejects_v2_vector_when_multi_shuttle_active():
    agent = _load_module()
    vector = [0.0] * len(agent.EVENT_ACTION_VECTOR_V2_FIELDS)
    vector[agent.EVENT_ACTION_VECTOR_V2_FIELDS.index('primitive_id')] = 4

    with pytest.raises(ValueError, match='schema-v3'):
        agent._parse_command_payload(vector, multi_shuttle_active=True)


def test_agent_rejects_ambiguous_multi_shuttle_json_command():
    agent = _load_module()

    with pytest.raises(ValueError, match='requires shuttle_id'):
        agent._parse_command_payload(
            {'action': 'shuttle', 'side': 'right', 'command': 'ON', 'speed': 0.2},
            multi_shuttle_active=True,
        )

    assert agent._parse_command_payload(
        {'action': 'shuttle', 'side': 'right', 'shuttle_id': 'R2', 'command': 'ON', 'speed': 0.2},
        multi_shuttle_active=True,
    )['shuttle_id'] == 'R2'


def test_agent_rejects_forbidden_privileged_output_fields():
    agent = _load_module()

    with pytest.raises(ValueError, match='forbidden privileged field'):
        agent._parse_command_payload(
            {
                'action': 'status',
                'target_shuttle_id': 'R2',
                'structured_rail_state': {'right': {}},
            },
            multi_shuttle_active=True,
        )


def test_http_plan_requests_schema_v3_when_multiple_shuttles_are_active():
    agent_module = _load_module()
    agent = agent_module.Room315RealVlaAgent.__new__(agent_module.Room315RealVlaAgent)
    agent.latest_status = _multi_status()
    agent.last_command = {'action': 'status'}
    agent.last_sensor_event_time_by_side = {'right': None, 'left': None}
    vector = _valid_v3_vector(agent_module)
    captured = {}

    def get_parameter(_self, name):
        return SimpleNamespace(value={'http_endpoint': 'http://example.test/vla'}[name])

    def post_json(_self, url, payload, headers):
        captured['payload'] = payload
        return {'action_vector_schema_version': 3, 'action_vector': vector}

    agent.get_parameter = MethodType(get_parameter, agent)
    agent._post_json = MethodType(post_json, agent)

    parsed = agent_module.Room315RealVlaAgent._http_plan(
        agent,
        'move R2 to Staubli',
        {'right_rail_rgb': 'right-b64', 'left_rail_rgb': 'left-b64'},
    )

    payload = captured['payload']
    assert payload['action_vector_schema_version'] == 3
    assert payload['multi_shuttle_active'] is True
    assert payload['shuttle_index_mapping']['right']['R2'] == 1
    assert payload['event_action_vector_fields'] == agent_module.EVENT_ACTION_VECTOR_FIELDS
    assert 'route_template' not in payload['allowed_actions']
    assert parsed['action_vector_schema_version'] == 3
