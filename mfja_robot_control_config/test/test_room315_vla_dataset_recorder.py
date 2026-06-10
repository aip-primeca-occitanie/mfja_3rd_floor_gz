#!/usr/bin/env python3

import importlib.util
import json
from io import StringIO
from pathlib import Path
from types import MethodType


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_vla_dataset_recorder.py'
ACTION_SPACE_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'config'
    / 'room_315_vla'
    / 'action_space.yaml'
)


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_vla_dataset_recorder', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _status():
    return {
        'emergency_stop': False,
        'vision': {'image_frames': 1},
        'active_tasks': {},
        'completed_tasks': [],
        'safety_decoder': {
            'metrics': {
                'total_proposed_actions': 2,
                'accepted_actions': 1,
                'rejected_actions': 1,
                'illegal_proposal_rate': 0.5,
                'rejection_reasons': {'unsafe switch change': 1},
            }
        },
        'rails': {
            side: {
                'shuttles': {},
                'switches': {'A1': 'EXTERIOR', 'A2': 'EXTERIOR', 'A3': 'EXTERIOR', 'A4': 'EXTERIOR'},
                'stoppers': {'A1': '0', 'A2': '0', 'A3': '0', 'A4': '0'},
                'active_sensors': [],
                'active_position_sensors': [],
            }
            for side in ('right', 'left')
        },
    }


def _fake_recorder(module):
    recorder = module.Room315VlaDatasetRecorder.__new__(module.Room315VlaDatasetRecorder)
    recorder.active = True
    recorder.episode_index = 1
    recorder.episode_id = 'episode_000001_test'
    recorder.frame_index = 0
    recorder.event_index = 0
    recorder.latest_goal = 'test task'
    recorder.latest_task_index = 0
    recorder.latest_command = {'action': 'status'}
    recorder.latest_status = _status()
    recorder.latest_images = {'right_rail_rgb': object()}
    recorder.image_dirs = {'right_rail_rgb': Path('/tmp/right_rail_rgb')}
    recorder.data_stream = StringIO()
    recorder.event_stream = StringIO()
    recorder.last_event_signature = ''
    recorder.last_primitive_signature = ''
    recorder.previous_event_command = {'action': 'START'}
    recorder.last_task_phase_by_id = {}
    recorder.completed_task_signatures = set()
    recorder.last_sensor_signature_by_side = {side: '' for side in ('right', 'left')}
    recorder.last_sensor_event_time_by_side = {side: None for side in ('right', 'left')}
    recorder.last_error = ''

    def write_image(self, camera_name, _image):
        return f'episodes/{self.episode_id}/images/{camera_name}/{self.frame_index:06d}.jpg'

    recorder._write_image = MethodType(write_image, recorder)
    recorder._publish_status = MethodType(lambda self, _state: None, recorder)
    recorder._now_seconds = MethodType(lambda self: 1000.0 + self.frame_index, recorder)
    return recorder


def _jsonl_rows(stream):
    return [
        json.loads(line)
        for line in stream.getvalue().splitlines()
        if line.strip()
    ]


def _feature_value(module, status, field):
    values = module._encode_state(status)
    return values[module.OBSERVATION_STATE_FIELDS.index(field)]


def _action_value(module, action_vector, field):
    return action_vector[module.ACTION_VECTOR_FIELDS.index(field)]


def _blank_event_action(module, *, primitive='WAIT', side='right', wait_condition='none', target_id='none', reason='none'):
    return {
        'primitive': primitive,
        'side': side,
        'switch_mask': {name: 0 for name in module.DEVICE_NAMES},
        'switch_values': {name: 'UNCHANGED' for name in module.DEVICE_NAMES},
        'stopper_mask': {name: 0 for name in module.DEVICE_NAMES},
        'stopper_values': {name: 'UNCHANGED' for name in module.DEVICE_NAMES},
        'speed_mps': 0.0,
        'wait_condition': wait_condition,
        'target_id': target_id,
        'reason': reason,
    }


def test_event_action_v2_switch_json_vector_roundtrip():
    recorder = _load_module()
    action = _blank_event_action(
        recorder,
        primitive='SET_SWITCHES',
        side='right',
        wait_condition='switch_state_match',
        target_id='A3',
        reason='switch_update',
    )
    action['switch_mask']['A3'] = 1
    action['switch_values']['A3'] = 'INTERIOR'

    encoded = recorder._encode_action(action)
    decoded = recorder._decode_action(encoded)

    assert recorder.ACTION_VECTOR_FIELDS[:2] == ['primitive_id', 'side_id']
    assert encoded[0] == recorder.PRIMITIVE_IDS['SET_SWITCHES']
    assert encoded[1] == recorder.SIDE_IDS['right']
    assert _action_value(recorder, encoded, 'switch_mask_A1') == 0.0
    assert _action_value(recorder, encoded, 'switch_mask_A3') == 1.0
    assert _action_value(recorder, encoded, 'switch_value_A3') == recorder.SWITCH_VALUE_IDS['INTERIOR']
    assert _action_value(recorder, encoded, 'wait_condition_id') == recorder.WAIT_CONDITION_IDS['switch_state_match']
    assert _action_value(recorder, encoded, 'target_id') == recorder.TARGET_IDS['A3']
    assert decoded == action


def test_event_action_v2_stopper_json_vector_roundtrip():
    recorder = _load_module()
    action = _blank_event_action(
        recorder,
        primitive='SET_STOPPERS',
        side='left',
        wait_condition='stopper_state_match',
        target_id='A4',
        reason='stopper_update',
    )
    action['stopper_mask']['A4'] = 1
    action['stopper_values']['A4'] = 'closed'

    encoded = recorder._encode_action(action)
    decoded = recorder._decode_action(encoded)

    assert encoded[0] == recorder.PRIMITIVE_IDS['SET_STOPPERS']
    assert encoded[1] == recorder.SIDE_IDS['left']
    assert _action_value(recorder, encoded, 'stopper_mask_A4') == 1.0
    assert _action_value(recorder, encoded, 'stopper_value_A4') == recorder.STOPPER_VALUE_IDS['closed']
    assert decoded == action


def test_event_action_v2_shuttle_wait_done_roundtrips():
    recorder = _load_module()
    actions = [
        _blank_event_action(
            recorder,
            primitive='SHUTTLE_ON',
            side='right',
            wait_condition='shuttle_command_applied',
            target_id='right_shuttle',
            reason='shuttle_start',
        ),
        _blank_event_action(
            recorder,
            primitive='SHUTTLE_ON',
            side='left',
            wait_condition='shuttle_command_applied',
            target_id='left_shuttle',
            reason='shuttle_start',
        ),
        _blank_event_action(
            recorder,
            primitive='STOP_NOW',
            side='right',
            wait_condition='shuttle_command_applied',
            target_id='right_shuttle',
            reason='shuttle_stop',
        ),
        _blank_event_action(
            recorder,
            primitive='WAIT',
            side='left',
            wait_condition='target_sensor_active',
            target_id='DA3IL',
            reason='task_phase',
        ),
        _blank_event_action(
            recorder,
            primitive='DONE',
            side='right',
            wait_condition='terminal',
            target_id='terminal',
            reason='task_succeeded',
        ),
        _blank_event_action(
            recorder,
            primitive='EMERGENCY_STOP',
            side='left',
            wait_condition='none',
            target_id='none',
            reason='emergency',
        ),
    ]
    actions[0]['speed_mps'] = 0.45
    actions[1]['speed_mps'] = 0.12

    for action in actions:
        assert recorder._decode_action(recorder._encode_action(action)) == action


def test_unmasked_device_values_do_not_decode_to_changes():
    recorder = _load_module()
    action = _blank_event_action(recorder, primitive='SET_SWITCHES', side='right')
    action['switch_mask']['A3'] = 1
    action['switch_values']['A3'] = 'INTERIOR'
    encoded = recorder._encode_action(action)
    encoded[recorder.ACTION_VECTOR_FIELDS.index('switch_value_A2')] = recorder.SWITCH_VALUE_IDS['INTERIOR']

    decoded = recorder._decode_action(encoded)

    assert decoded['switch_mask']['A2'] == 0
    assert decoded['switch_values']['A2'] == 'UNCHANGED'
    assert decoded['switch_mask']['A3'] == 1
    assert decoded['switch_values']['A3'] == 'INTERIOR'

    encoded[recorder.ACTION_VECTOR_FIELDS.index('switch_mask_A2')] = 1.0
    encoded[recorder.ACTION_VECTOR_FIELDS.index('switch_value_A2')] = recorder.SWITCH_VALUE_IDS['UNCHANGED']
    try:
        recorder._decode_action(encoded)
    except ValueError as exc:
        assert 'switch_mask_A2 selected but value is UNCHANGED' in str(exc)
    else:
        raise AssertionError('expected selected UNCHANGED switch value to be rejected')


def test_dataset_recorder_extracts_task_context_and_primitives_from_status():
    recorder = _load_module()
    status = {
        'active_tasks': {
            'task_000001': {
                'task_id': 'task_000001',
                'template': 'right_enter_interior_loop',
                'phase': 'wait_target_switches',
                'status': 'running',
                'primitive_commands': [
                    {'action': 'switches', 'side': 'right', 'switches': {'ALL': 'INTERIOR'}},
                ],
            }
        },
        'completed_tasks': [],
    }

    context = recorder._task_context_from_status(
        status,
        {'action': 'route_template', 'template': 'right_enter_interior_loop'},
    )

    assert context['task_id'] == 'task_000001'
    assert context['phase'] == 'wait_target_switches'
    assert context['primitive_commands'][0]['action'] == 'switches'


def test_observation_uses_sensor_identity_not_only_counts():
    recorder = _load_module()
    status_dzi2 = _status()
    status_dzi3 = _status()
    status_dzi2['rails']['right']['active_position_sensors'] = [
        {'name': 'DZI2R', 'shuttle': 'room315_right_shuttle_1'},
    ]
    status_dzi3['rails']['right']['active_position_sensors'] = [
        {'name': 'DZI3R', 'shuttle': 'room315_right_shuttle_1'},
    ]

    assert _feature_value(recorder, status_dzi2, 'right_sensor_DZI2R') == 1.0
    assert _feature_value(recorder, status_dzi2, 'right_sensor_DZI3R') == 0.0
    assert _feature_value(recorder, status_dzi3, 'right_sensor_DZI2R') == 0.0
    assert _feature_value(recorder, status_dzi3, 'right_sensor_DZI3R') == 1.0
    assert recorder._encode_state(status_dzi2) != recorder._encode_state(status_dzi3)


def test_switch_state_encoder_normalizes_short_and_long_values():
    recorder = _load_module()
    status = _status()
    status['rails']['right']['switches'] = {
        'A1': 'E',
        'A2': 'I',
        'A3': 'exterior',
        'A4': 'interior',
    }
    status['rails']['left']['switches'] = {
        'A1': '',
        'A2': None,
        'A3': 'UNKNOWN',
        'A4': 'weird',
    }

    assert _feature_value(recorder, status, 'right_switch_A1') == 1.0
    assert _feature_value(recorder, status, 'right_switch_A2') == 2.0
    assert _feature_value(recorder, status, 'right_switch_A3') == 1.0
    assert _feature_value(recorder, status, 'right_switch_A4') == 2.0
    assert _feature_value(recorder, status, 'left_switch_A1') == 0.0
    assert _feature_value(recorder, status, 'left_switch_A2') == 0.0
    assert _feature_value(recorder, status, 'left_switch_A3') == 0.0
    assert _feature_value(recorder, status, 'left_switch_A4') == 0.0


def test_model_input_schema_v3_is_visual_policy_input_only():
    recorder = _load_module()
    status = _status()
    status['rails']['right']['switches'] = {
        'A1': 'E',
        'A2': 'I',
        'A3': 'exterior',
        'A4': 'interior',
    }
    status['rails']['right']['active_position_sensors'] = [
        {'name': 'DZI2R', 'shuttle': 'room315_right_shuttle_1'},
    ]
    status['rails']['right']['shuttles'] = {
        'room315_right_shuttle_1': {
            'segment': 'A12E',
            's': 0.42,
            'x': 1.0,
            'y': 2.0,
            'z': 0.3,
            'yaw': 0.5,
            'distance_to_switch': 0.12,
            'normalized_position': 0.42,
        }
    }
    status['last_primitive_command'] = {
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'room315_right_shuttle_1',
        'command': 'ON',
    }

    model_input = recorder._model_input_from_status(
        status,
        language='move the right shuttle from Yaskawa to Staubli',
        overhead_images={
            'right_rail_rgb': 'episodes/e/images/right_rail_rgb/000001.jpg',
            'left_rail_rgb': 'episodes/e/images/left_rail_rgb/000001.jpg',
            'legacy_primary_rgb': 'should_not_be_used.jpg',
        },
        last_command={'action': 'switches', 'side': 'right', 'switches': {'A3': 'INTERIOR'}},
        sensor_event_times={'right': 95.0, 'left': None},
        now_s=100.0,
    )

    assert recorder.MODEL_INPUT_SCHEMA_VERSION == 3
    assert set(model_input) == set(recorder.MODEL_INPUT_FIELDS)
    assert set(model_input) == {
        'language',
        'overhead_images',
        'last_command',
    }
    assert set(model_input['overhead_images']) == {'right_rail_rgb', 'left_rail_rgb'}
    assert model_input['last_command']['action'] == 'switches'

    serialized = json.dumps(model_input, sort_keys=True)
    for expert_shortcut in (
        'binary_sensor_bits',
        'switch_states',
        'stopper_states',
        'shuttle_command_state',
        'time_since_last_sensor_event',
        'DZI2R',
        'DZI3R',
    ):
        assert expert_shortcut not in serialized
    assert 'A12E' not in serialized
    assert 'distance_to_switch' not in serialized
    assert 'normalized_position' not in serialized
    for privileged_key in ('segment', '"x"', '"y"', '"z"', 'yaw', '"s"'):
        assert privileged_key not in serialized

    privileged = recorder._privileged_eval_from_status(
        status,
        sensor_event_times={'right': 95.0, 'left': None},
        now_s=100.0,
    )
    expert_state = privileged['expert_sensor_state']
    assert expert_state['binary_sensor_bits']['right']['DZI2R'] == 1
    assert expert_state['binary_sensor_bits']['right']['DZI3R'] == 0
    assert expert_state['switch_states']['right'] == {
        'A1': 'EXTERIOR',
        'A2': 'INTERIOR',
        'A3': 'EXTERIOR',
        'A4': 'INTERIOR',
    }
    assert expert_state['stopper_states']['right']['A1'] == 'open'
    assert expert_state['shuttle_command_state']['right']['last_command'] == 'ON'
    assert expert_state['time_since_last_sensor_event']['right'] == 5.0
    assert expert_state['time_since_last_sensor_event']['left'] is None
    assert expert_state['model_input_exposure'] == 'excluded'
    assert privileged['raw_shuttle_states']['right']['room315_right_shuttle_1']['segment'] == 'A12E'


def test_visual_eval_markers_are_privileged_labels_not_observation_features():
    recorder = _load_module()
    status = _status()
    status['rails']['right']['active_position_sensors'] = [
        {'name': 'DZI3R', 'shuttle': 'room315_right_shuttle_1'},
    ]

    marker_ids = set()
    for marker_group in recorder.VISUAL_EVAL_MARKERS.values():
        if isinstance(marker_group, list):
            marker_ids.update(str(item.get('id')) for item in marker_group)
        elif isinstance(marker_group, dict):
            marker_ids.add(str(marker_group.get('entity')))
            entities = marker_group.get('entities', {})
            if isinstance(entities, dict):
                marker_ids.update(str(name) for name in entities.values())
            marker_ids.update(str(name) for name in marker_group.get('visual_ids', []))

    model_input = recorder._model_input_from_status(
        status,
        language='inspect the station marker',
        overhead_images={'right_rail_rgb': 'right.jpg', 'left_rail_rgb': 'left.jpg'},
        last_command={'action': 'status'},
    )
    observation_schema = json.dumps(recorder.OBSERVATION_STATE_FIELDS)
    model_input_json = json.dumps(model_input, sort_keys=True)

    for marker_id in marker_ids:
        assert marker_id not in observation_schema
        assert marker_id not in model_input_json

    privileged = recorder._privileged_eval_from_status(status)
    labels = privileged['visual_eval_labels']
    assert labels['model_input_exposure'] == 'excluded'
    assert labels['policy_visibility'] == 'visual_input_only'
    assert 'colored_station_markers' not in labels['marker_definitions']
    assert 'inspection_markers' not in labels['marker_definitions']
    assert 'station_status_markers' not in labels['marker_definitions']
    assert labels['station_occupancy']['right']['staubli_tx2']['label'] == 'occupied'
    assert labels['station_occupancy']['right']['yaskawa_hc10dt']['label'] == 'empty'
    assert (
        labels['station_occupancy']['right']['staubli_tx2']['model_task']
        == 'infer visual occupancy from shuttle-over-slot-fiducials'
    )
    assert (
        labels['marker_definitions']['removable_obstacle_marker']['entities']['right']
        == 'room315_vla_right_obstacle_marker'
    )
    assert (
        labels['marker_definitions']['removable_obstacle_marker']['entities']['left']
        == 'room315_vla_left_obstacle_marker'
    )
    for removed_marker in (
        'right_yaskawa_station_marker',
        'right_staubli_station_marker',
        'left_yaskawa_station_marker',
        'left_kuka_station_marker',
        'right_green_inspection_marker',
        'left_green_inspection_marker',
        'right_station_empty_marker',
        'right_station_occupied_marker',
        'left_station_empty_marker',
        'left_station_occupied_marker',
    ):
        assert removed_marker not in json.dumps(labels['marker_definitions'])


def test_structured_rail_state_is_sensor_and_device_only():
    recorder = _load_module()
    status = _status()
    status['rails']['left']['active_position_sensors'] = [{'name': 'DA3IL'}]
    status['rails']['left']['shuttles'] = {
        'room315_left_shuttle_1': {'segment': 'A12I', 's': 0.9, 'x': -1.0}
    }

    structured = recorder._structured_rail_state(status)

    assert structured['rails']['left']['sensor_multi_hot']['DA3IL'] == 1
    assert 'shuttles' not in structured['rails']['left']
    assert 'active_position_sensors' not in structured['rails']['left']
    assert 'A12I' not in json.dumps(structured)


def test_action_space_observation_schema_matches_recorder():
    import yaml

    recorder = _load_module()
    config = yaml.safe_load(ACTION_SPACE_PATH.read_text(encoding='utf-8'))

    assert config['schema_version'] == 2
    assert config['action_vector_fields'] == recorder.ACTION_VECTOR_FIELDS
    assert config['symbolic_action_fields'] == recorder.EVENT_ACTION_FIELDS
    assert config['observation_state_fields'] == recorder.OBSERVATION_STATE_FIELDS
    assert config['primitive_ids'] == recorder.PRIMITIVE_IDS
    assert config['wait_condition_ids'] == recorder.WAIT_CONDITION_IDS
    assert config['target_ids'] == recorder.TARGET_IDS
    assert config['reason_ids'] == recorder.REASON_IDS
    assert 'action_ids' not in config
    assert 'template_ids' not in config
    assert 'switch_all_state_id' not in config['action_vector_fields']
    assert 'switch_mask_A3' in config['action_vector_fields']
    assert 'stopper_value_A4' in config['action_vector_fields']
    assert 'speed_mps' in config['action_vector_fields']
    assert 'right_sensor_DZI2R' in config['observation_state_fields']
    assert 'right_active_sensors' not in config['observation_state_fields']
    assert 'right_active_sensors_count' in config['debug_observation_fields']
    assert config['model_input_schema_version'] == recorder.MODEL_INPUT_SCHEMA_VERSION
    assert config['model_input_fields'] == recorder.MODEL_INPUT_FIELDS
    assert config['privileged_eval_fields'] == recorder.PRIVILEGED_EVAL_FIELDS


def test_event_labels_do_not_repeat_long_shuttle_on_command():
    recorder_module = _load_module()
    recorder = _fake_recorder(recorder_module)

    on_command = {
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'room315_right_shuttle_1',
        'command': 'ON',
        'speed': 0.2,
    }
    recorder._record_command_event(on_command)
    recorder._record_command_event(on_command)

    rows = _jsonl_rows(recorder.event_stream)
    assert len(rows) == 1
    assert rows[0]['legacy_next_action']['command'] == 'ON'
    assert rows[0]['next_action']['primitive'] == 'SHUTTLE_ON'
    assert rows[0]['next_action']['speed_mps'] == 0.2
    assert rows[0]['next_action']['reason'] == 'shuttle_start'

    recorder._record_command_event({**on_command, 'command': 'OFF'})
    rows = _jsonl_rows(recorder.event_stream)
    assert len(rows) == 2
    assert rows[1]['legacy_next_action']['command'] == 'OFF'
    assert rows[1]['next_action']['primitive'] == 'STOP_NOW'


def test_event_model_input_last_command_is_previous_event_not_current_label():
    recorder_module = _load_module()
    recorder = _fake_recorder(recorder_module)

    commands = [
        {'action': 'shuttle', 'side': 'right', 'command': 'OFF'},
        {'action': 'switches', 'side': 'right', 'switches': {'A3': 'INTERIOR'}},
        {'action': 'stoppers', 'side': 'right', 'stoppers': {'A3': 'closed'}},
        {'action': 'shuttle', 'side': 'right', 'command': 'ON', 'speed': 0.2},
        {'action': 'shuttle', 'side': 'right', 'command': 'OFF'},
    ]
    for command in commands:
        recorder._record_command_event(command)
    recorder._record_event(
        {'action': 'DONE', 'status': 'success'},
        original_command={'action': 'DONE', 'status': 'success'},
        event_type='episode_terminal',
        status_text='success',
    )

    rows = _jsonl_rows(recorder.event_stream)

    assert [row['action']['primitive'] for row in rows] == [
        'STOP_NOW',
        'SET_SWITCHES',
        'SET_STOPPERS',
        'SHUTTLE_ON',
        'STOP_NOW',
        'DONE',
    ]
    assert rows[0]['model_input']['last_command'] == {'action': 'START'}
    assert rows[1]['model_input']['last_command'] == rows[0]['action']
    assert rows[2]['model_input']['last_command'] == rows[1]['action']
    assert rows[3]['model_input']['last_command'] == rows[2]['action']
    assert rows[4]['model_input']['last_command'] == rows[3]['action']
    assert rows[5]['model_input']['last_command'] == rows[4]['action']
    for row in rows:
        assert set(row['model_input']) == set(recorder_module.MODEL_INPUT_FIELDS)
        assert row['model_input']['last_command'] != row['legacy_next_action']
        assert row['model_input']['last_command'] != row['action']
        assert row['action_vector'] == recorder_module._encode_action(row['action'])
        serialized_model_input = json.dumps(row['model_input'], sort_keys=True)
        for forbidden in (
            'binary_sensor_bits',
            'switch_states',
            'stopper_states',
            'raw_shuttle_states',
            'privileged_eval',
            'segment',
            '"x"',
            '"s"',
        ):
            assert forbidden not in serialized_model_input


def test_switch_and_stopper_commands_create_separate_events():
    recorder_module = _load_module()
    recorder = _fake_recorder(recorder_module)

    recorder._record_command_event({
        'action': 'switches',
        'side': 'left',
        'switches': {'A1': 'INTERIOR'},
    })
    recorder._record_command_event({
        'action': 'stoppers',
        'side': 'left',
        'stoppers': {'A1': '1'},
    })

    rows = _jsonl_rows(recorder.event_stream)
    assert [row['legacy_next_action']['action'] for row in rows] == ['switches', 'stoppers']
    assert [row['next_action']['primitive'] for row in rows] == ['SET_SWITCHES', 'SET_STOPPERS']
    assert rows[0]['next_action']['switch_mask']['A1'] == 1
    assert rows[1]['next_action']['stopper_mask']['A1'] == 1
    assert rows[0]['wait_condition']['type'] == 'switch_state_match'
    assert rows[1]['wait_condition']['type'] == 'stopper_state_match'


def test_redundant_switch_command_is_skipped_when_state_already_matches():
    recorder_module = _load_module()
    recorder = _fake_recorder(recorder_module)

    recorder._record_command_event({
        'action': 'switches',
        'side': 'right',
        'switches': {'ALL': 'EXTERIOR'},
    })

    rows = _jsonl_rows(recorder.event_stream)
    metrics = recorder._event_generation_metrics()
    assert rows == []
    assert recorder.previous_event_command == {'action': 'START'}
    assert metrics['event_candidate_count'] == 1
    assert metrics['skipped_redundant_event_count'] == 1
    assert metrics['redundant_action_rate'] == 1.0
    assert metrics['noop_action_rate'] == 1.0
    assert metrics['effective_action_rate'] == 0.0


def test_switch_event_records_only_devices_that_need_changes():
    recorder_module = _load_module()
    recorder = _fake_recorder(recorder_module)
    recorder.latest_status['rails']['right']['switches']['A3'] = 'INTERIOR'

    recorder._record_command_event({
        'action': 'switches',
        'side': 'right',
        'switches': {'ALL': 'EXTERIOR'},
    })

    rows = _jsonl_rows(recorder.event_stream)
    assert len(rows) == 1
    row = rows[0]
    assert row['legacy_next_action']['switches'] == {'A3': 'EXTERIOR'}
    assert row['action']['primitive'] == 'SET_SWITCHES'
    assert row['action']['switch_mask'] == {'A1': 0, 'A2': 0, 'A3': 1, 'A4': 0}
    assert row['action']['switch_values']['A3'] == 'EXTERIOR'
    assert row['action_vector'] == recorder_module._encode_action(row['action'])
    assert row['model_input']['last_command'] == {'action': 'START'}
    assert set(row['model_input']) == set(recorder_module.MODEL_INPUT_FIELDS)
    assert row['minimal_recording']['requested'] == {
        'A1': 'EXTERIOR',
        'A2': 'EXTERIOR',
        'A3': 'EXTERIOR',
        'A4': 'EXTERIOR',
    }
    assert row['minimal_recording']['needed'] == {'A3': 'EXTERIOR'}
    assert row['minimal_recording']['redundant'] == {
        'A1': 'EXTERIOR',
        'A2': 'EXTERIOR',
        'A4': 'EXTERIOR',
    }
    assert row['auxiliary_targets']['switch_states']['right']['A3'] == 'INTERIOR'
    assert 'auxiliary_targets' not in row['model_input']
    assert 'privileged_eval' not in row['model_input']
    assert row['event_generation_metrics']['effective_action_rate'] == 1.0


def test_stopper_events_use_same_minimal_recording_logic():
    recorder_module = _load_module()
    recorder = _fake_recorder(recorder_module)

    recorder._record_command_event({
        'action': 'stoppers',
        'side': 'right',
        'stoppers': {'ALL': 'open'},
    })
    assert _jsonl_rows(recorder.event_stream) == []
    assert recorder._event_generation_metrics()['skipped_redundant_event_count'] == 1

    recorder.latest_status['rails']['right']['stoppers']['A2'] = '1'
    recorder._record_command_event({
        'action': 'stoppers',
        'side': 'right',
        'stoppers': {'ALL': 'open'},
    })

    rows = _jsonl_rows(recorder.event_stream)
    assert len(rows) == 1
    row = rows[0]
    assert row['legacy_next_action']['stoppers'] == {'A2': 'open'}
    assert row['action']['primitive'] == 'SET_STOPPERS'
    assert row['action']['stopper_mask'] == {'A1': 0, 'A2': 1, 'A3': 0, 'A4': 0}
    assert row['action']['stopper_values']['A2'] == 'open'
    assert row['action_vector'] == recorder_module._encode_action(row['action'])
    assert row['model_input']['last_command'] == {'action': 'START'}
    assert row['auxiliary_targets']['stopper_states']['right']['A2'] == 'closed'
    assert set(row['model_input']) == {'language', 'overhead_images', 'last_command'}


def test_route_template_phase_change_creates_one_event_per_phase():
    recorder_module = _load_module()
    recorder = _fake_recorder(recorder_module)
    recorder.latest_status['active_tasks'] = {
        'task_000001': {
            'task_id': 'task_000001',
            'template': 'right_yaskawa_to_staubli',
            'phase': 'wait_target',
            'status': 'running',
        }
    }

    recorder._record_status_events()
    recorder._record_status_events()

    rows = _jsonl_rows(recorder.event_stream)
    assert len(rows) == 1
    assert rows[0]['legacy_next_action']['action'] == 'route_template_phase'
    assert rows[0]['legacy_next_action']['phase'] == 'wait_target'
    assert rows[0]['next_action']['primitive'] == 'WAIT'
    assert rows[0]['next_action']['wait_condition'] == 'task_phase_observed'
    assert rows[0]['next_action']['target_id'] == 'task_phase'


def test_raw_framewise_recording_still_writes_replay_rows():
    recorder_module = _load_module()
    recorder = _fake_recorder(recorder_module)

    sample = recorder._write_raw_sample({
        'action': 'shuttle',
        'side': 'right',
        'command': 'ON',
    })

    rows = _jsonl_rows(recorder.data_stream)
    assert sample['frame_index'] == 0
    assert len(rows) == 1
    assert rows[0]['raw_replay_only'] is True
    assert rows[0]['command']['action'] == 'shuttle'
    assert rows[0]['model_input_schema_version'] == recorder_module.MODEL_INPUT_SCHEMA_VERSION
    assert set(rows[0]['model_input']) == set(recorder_module.MODEL_INPUT_FIELDS)
    assert rows[0]['model_input']['last_command'] == {'action': 'START'}
    assert 'supervisor_status' not in rows[0]
    assert 'supervisor_status' in rows[0]['privileged_eval']
    assert rows[0]['safety_decoder_metrics']['illegal_proposal_rate'] == 0.5
    assert rows[0]['observation.images.right_rail_rgb'].endswith('000000.jpg')
