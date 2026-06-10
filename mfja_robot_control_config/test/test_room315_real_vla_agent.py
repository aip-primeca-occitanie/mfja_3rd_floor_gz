#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path
from types import MethodType
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_real_vla_agent.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_real_vla_agent', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _status():
    return {
        'rails': {
            'right': {
                'shuttles': {
                    'room315_right_shuttle_1': {
                        'segment': 'A12E',
                        's': 0.25,
                        'x': 1.0,
                        'y': 2.0,
                        'z': 0.3,
                        'yaw': 1.57,
                        'distance_to_switch': 0.1,
                        'normalized_position': 0.25,
                    }
                },
                'switches': {'A1': 'E', 'A2': 'I', 'A3': 'EXTERIOR', 'A4': 'interior'},
                'stoppers': {'A1': '0', 'A2': '1', 'A3': 'open', 'A4': 'closed'},
                'active_sensors': [],
                'active_position_sensors': [{'name': 'DZI2R'}],
            },
            'left': {
                'shuttles': {},
                'switches': {'A1': 'unknown', 'A2': '', 'A3': None, 'A4': 'E'},
                'stoppers': {'A1': '0', 'A2': '0', 'A3': '0', 'A4': '0'},
                'active_sensors': [],
                'active_position_sensors': [{'name': 'DA3IL'}],
            },
        },
        'last_primitive_command': {
            'action': 'shuttle',
            'side': 'right',
            'shuttle': 'room315_right_shuttle_1',
            'command': 'ON',
        },
    }


def test_agent_model_input_schema_v3_excludes_sensor_and_privileged_state():
    agent_module = _load_module()

    model_input = agent_module._model_input_from_status(
        _status(),
        language='move the right shuttle from Yaskawa to Staubli',
        overhead_images={
            'right_rail_rgb': 'right-jpeg-b64',
            'left_rail_rgb': 'left-jpeg-b64',
            'legacy_primary_rgb': 'legacy-should-not-be-sent',
        },
        last_command={
            'action': 'switches',
            'side': 'right',
            'switches': {'A3': 'I'},
            'supervisor_status': _status(),
            'segment': 'A12E',
        },
        sensor_event_times={'right': 10.0, 'left': None},
        now_s=12.5,
    )

    assert agent_module.MODEL_INPUT_SCHEMA_VERSION == 3
    assert set(model_input) == set(agent_module.MODEL_INPUT_FIELDS)
    assert set(model_input) == {'language', 'overhead_images', 'last_command'}
    assert set(model_input['overhead_images']) == {'right_rail_rgb', 'left_rail_rgb'}
    assert model_input['last_command'] == {
        'action': 'switches',
        'side': 'right',
        'switches': {'A3': 'INTERIOR'},
    }

    serialized = json.dumps(model_input, sort_keys=True)
    for sensor_shortcut in (
        'binary_sensor_bits',
        'switch_states',
        'stopper_states',
        'shuttle_command_state',
        'time_since_last_sensor_event',
        'DZI2R',
        'DZI3R',
        'DA3IL',
    ):
        assert sensor_shortcut not in serialized
    assert 'supervisor_status' not in serialized
    assert 'A12E' not in serialized
    assert 'distance_to_switch' not in serialized
    assert 'normalized_position' not in serialized
    for privileged_key in ('segment', '"x"', '"y"', '"z"', 'yaw', '"s"'):
        assert privileged_key not in serialized


def test_http_plan_sends_only_schema_v3_model_input_to_provider():
    agent_module = _load_module()
    agent = agent_module.Room315RealVlaAgent.__new__(agent_module.Room315RealVlaAgent)
    agent.latest_status = _status()
    agent.last_command = {'action': 'status'}
    agent.last_sensor_event_time_by_side = {'right': 20.0, 'left': 21.0}

    captured = {}

    def get_parameter(_self, name):
        values = {'http_endpoint': 'http://example.test/vla'}
        return SimpleNamespace(value=values[name])

    def post_json(_self, url, payload, headers):
        captured['url'] = url
        captured['payload'] = payload
        captured['headers'] = headers
        return {'action': 'status'}

    agent.get_parameter = MethodType(get_parameter, agent)
    agent._post_json = MethodType(post_json, agent)

    command = agent_module.Room315RealVlaAgent._http_plan(
        agent,
        'make the right shuttle circulate on the interior loop',
        {
            'right_rail_rgb': 'right-jpeg-b64',
            'left_rail_rgb': 'left-jpeg-b64',
            'legacy_primary_rgb': 'legacy-should-not-be-sent',
        },
    )

    payload = captured['payload']
    assert command == {'action': 'status'}
    assert captured['url'] == 'http://example.test/vla'
    assert 'supervisor_status' not in payload
    assert 'image_jpeg_b64' not in payload
    assert 'images_jpeg_b64' not in payload
    assert payload['model_input_schema_version'] == 3
    assert payload['allowed_actions'] == agent_module.MODEL_OUTPUT_ACTIONS
    assert 'route_template' not in payload['allowed_actions']
    assert payload['preferred_model_output'] == 'action_vector'
    assert 'action_vector' in payload['allowed_output_formats']
    assert payload['event_action_vector_fields'] == agent_module.EVENT_ACTION_VECTOR_FIELDS
    assert payload['event_primitive_ids']['SET_SWITCHES'] == 2
    assert set(payload['model_input']) == set(agent_module.MODEL_INPUT_FIELDS)
    assert set(payload['model_input']['overhead_images']) == {'right_rail_rgb', 'left_rail_rgb'}
    serialized = json.dumps(payload['model_input'], sort_keys=True)
    assert 'A12E' not in serialized
    assert 'DZI2R' not in serialized
    assert 'binary_sensor_bits' not in serialized


def test_agent_accepts_event_level_action_vector_response():
    agent_module = _load_module()
    action_vector = [
        2, 0,
        0, 0, 1, 0,
        0, 0, 2, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0.0, 1, 3, 8,
    ]

    assert agent_module._parse_command_payload(action_vector) == {
        'action_vector': [float(value) for value in action_vector],
    }
    assert agent_module._parse_command_payload({'action_vector': action_vector}) == {
        'action_vector': [float(value) for value in action_vector],
    }
