#!/usr/bin/env python3

import importlib.util
import json
from io import StringIO
from pathlib import Path
from types import MethodType


REPO_ROOT = Path(__file__).resolve().parents[2]
RECORDER_PATH = (
    REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_vla_dataset_recorder.py'
)
EVAL_PATH = (
    REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_vla_baseline_eval.py'
)
SUPERVISOR_PATH = (
    REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_vla_supervisor.py'
)


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load_recorder():
    return _load_module('room_315_vla_dataset_recorder_smoke', RECORDER_PATH)


def _load_eval():
    return _load_module('room_315_vla_baseline_eval_smoke', EVAL_PATH)


def _load_supervisor():
    return _load_module('room_315_vla_supervisor_smoke', SUPERVISOR_PATH)


def _status():
    return {
        'emergency_stop': False,
        'vision': {'image_frames': 2},
        'active_tasks': {},
        'completed_tasks': [],
        'last_primitive_command': {
            'action': 'shuttle',
            'side': 'right',
            'shuttle': 'room315_right_shuttle_1',
            'command': 'OFF',
        },
        'rails': {
            'right': {
                'shuttles': {
                    'room315_right_shuttle_1': {
                        'mode': 'STOPPED',
                        'segment': 'A12E',
                        's': 0.42,
                        'x': 1.0,
                        'y': 2.0,
                        'z': 0.3,
                        'yaw': 0.5,
                        'distance_to_switch': 0.12,
                        'normalized_position': 0.42,
                    },
                },
                'switches': {'A1': 'E', 'A2': 'I', 'A3': 'EXTERIOR', 'A4': 'INTERIOR'},
                'stoppers': {'A1': '0', 'A2': '0', 'A3': '0', 'A4': '0'},
                'active_sensors': [],
                'active_position_sensors': [
                    {'name': 'DZI2R', 'shuttle': 'room315_right_shuttle_1'},
                ],
            },
            'left': {
                'shuttles': {},
                'switches': {'A1': 'EXTERIOR', 'A2': 'EXTERIOR', 'A3': 'EXTERIOR', 'A4': 'EXTERIOR'},
                'stoppers': {'A1': '0', 'A2': '0', 'A3': '0', 'A4': '0'},
                'active_sensors': [],
                'active_position_sensors': [{'name': 'DA3IL'}],
            },
        },
    }


def _fake_recorder(module):
    recorder = module.Room315VlaDatasetRecorder.__new__(module.Room315VlaDatasetRecorder)
    recorder.active = True
    recorder.episode_index = 1
    recorder.episode_id = 'episode_000001_smoke'
    recorder.frame_index = 0
    recorder.event_index = 0
    recorder.latest_goal = 'left_slot3_kuka_then_slot2'
    recorder.latest_task_index = 0
    recorder.latest_command = {'action': 'status'}
    recorder.latest_status = _status()
    recorder.latest_images = {
        'right_rail_rgb': object(),
        'left_rail_rgb': object(),
    }
    recorder.image_dirs = {
        'right_rail_rgb': Path('/tmp/right_rail_rgb'),
        'left_rail_rgb': Path('/tmp/left_rail_rgb'),
    }
    recorder.data_stream = StringIO()
    recorder.event_stream = StringIO()
    recorder.last_event_signature = ''
    recorder.last_primitive_signature = ''
    recorder.last_task_phase_by_id = {}
    recorder.completed_task_signatures = set()
    recorder.last_sensor_signature_by_side = {side: '' for side in ('right', 'left')}
    recorder.last_sensor_event_time_by_side = {'right': 95.0, 'left': None}
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


def _blank_action(module, primitive: str, side: str = 'right') -> dict:
    return {
        'primitive': primitive,
        'side': side,
        'switch_mask': {name: 0 for name in module.DEVICE_NAMES},
        'switch_values': {name: 'UNCHANGED' for name in module.DEVICE_NAMES},
        'stopper_mask': {name: 0 for name in module.DEVICE_NAMES},
        'stopper_values': {name: 'UNCHANGED' for name in module.DEVICE_NAMES},
        'speed_mps': 0.0,
        'wait_condition': 'none',
        'target_id': 'none',
        'reason': 'none',
    }


def _event_vector(
    module,
    *,
    primitive: str,
    side: str = 'right',
    speed_mps: float = 0.0,
    switch_values: dict[str, str] | None = None,
    stopper_values: dict[str, str] | None = None,
    wait_condition: str = 'none',
    target_id: str = 'none',
    reason: str = 'none',
) -> list[float]:
    switch_values = switch_values or {}
    stopper_values = stopper_values or {}
    primitive_ids = {value: key for key, value in module.EVENT_PRIMITIVE_BY_ID.items()}
    side_ids = {value: key for key, value in module.EVENT_SIDE_BY_ID.items()}
    wait_ids = {value: key for key, value in module.EVENT_WAIT_CONDITION_BY_ID.items()}
    target_ids = {value: key for key, value in module.EVENT_TARGET_BY_ID.items()}
    reason_ids = {value: key for key, value in module.EVENT_REASON_BY_ID.items()}
    values = [0.0 for _field in module.EVENT_ACTION_VECTOR_FIELDS]

    def set_field(name: str, value: float) -> None:
        values[module.EVENT_ACTION_VECTOR_FIELDS.index(name)] = float(value)

    set_field('primitive_id', primitive_ids[primitive])
    set_field('side_id', side_ids[side])
    set_field('speed_mps', speed_mps)
    for switch_name, state in switch_values.items():
        set_field(f'switch_mask_{switch_name}', 1)
        set_field(f'switch_value_{switch_name}', {'EXTERIOR': 1, 'INTERIOR': 2}[state])
    for stopper_name, state in stopper_values.items():
        set_field(f'stopper_mask_{stopper_name}', 1)
        set_field(f'stopper_value_{stopper_name}', {'open': 1, 'closed': 2, '0': 1, '1': 2}[state])
    set_field('wait_condition_id', wait_ids[wait_condition])
    set_field('target_id', target_ids[target_id])
    set_field('reason_id', reason_ids[reason])
    return values


def _done_action() -> dict:
    return {
        'primitive': 'DONE',
        'side': 'right',
        'switch_mask': {'A1': 0, 'A2': 0, 'A3': 0, 'A4': 0},
        'switch_values': {
            'A1': 'UNCHANGED',
            'A2': 'UNCHANGED',
            'A3': 'UNCHANGED',
            'A4': 'UNCHANGED',
        },
        'stopper_mask': {'A1': 0, 'A2': 0, 'A3': 0, 'A4': 0},
        'stopper_values': {
            'A1': 'UNCHANGED',
            'A2': 'UNCHANGED',
            'A3': 'UNCHANGED',
            'A4': 'UNCHANGED',
        },
        'speed_mps': 0.0,
        'wait_condition': 'terminal',
        'target_id': 'terminal',
        'reason': 'task_succeeded',
    }


def _rails_for_safety():
    return {
        'right': {
            'shuttles': {
                'room315_right_shuttle_1': {
                    'mode': 'STOPPED',
                    'segment': 'A34E',
                    'speed': 0.0,
                },
            },
            'switches': {'A1': 'E', 'A2': 'E', 'A3': 'E', 'A4': 'E'},
            'stoppers': {'A1': '0', 'A2': '0', 'A3': '0', 'A4': '0'},
            'active_sensors': [],
            'active_position_sensors': [],
        },
        'left': {
            'shuttles': {},
            'switches': {'A1': 'E', 'A2': 'E', 'A3': 'E', 'A4': 'E'},
            'stoppers': {'A1': '0', 'A2': '0', 'A3': '0', 'A4': '0'},
            'active_sensors': [],
            'active_position_sensors': [],
        },
    }


def test_smoke_has_one_episode_for_each_evaluator_task_family():
    evaluator = _load_eval()
    examples = {
        'visual_target': 'left_slot3_kuka_then_slot2',
        'obstacle_stop': 'right_obstacle_aware_route',
        'loop_entry': 'right_enter_interior_loop',
        'transport': 'right_yaskawa_to_staubli',
        'station_navigation': 'center at station',
        'stopper': 'stop at A2 stopper',
        'exterior_loop': 'complete one exterior loop',
        'emergency': 'emergency stop_all',
    }
    rows = [
        {
            'episode_id': f'episode_{index:06d}_{family}',
            'task': task,
            'template': task if '_' in task else '',
            'phase': '',
            'event_type': 'task_terminal',
            'timestamp': float(index),
            'action': _done_action(),
        }
        for index, (family, task) in enumerate(examples.items(), start=1)
    ]

    observed_families = {evaluator.task_family(row) for row in rows}
    metrics_by_family = {
        row['task_family']: row
        for row in evaluator.task_family_metrics(rows)
    }

    assert observed_families == set(examples)
    for family in examples:
        assert metrics_by_family[family]['episodes'] == 1
        assert metrics_by_family[family]['tasks'] == 1
        assert metrics_by_family[family]['task_success'] == 1.0


def test_smoke_model_input_has_no_sensor_or_exact_pose_leaks():
    recorder = _load_recorder()

    model_input = recorder._model_input_from_status(
        _status(),
        language='move to the visual target',
        overhead_images={'right_rail_rgb': 'right.jpg', 'left_rail_rgb': 'left.jpg'},
        last_command={'action': 'status'},
        sensor_event_times={'right': 95.0, 'left': None},
        now_s=100.0,
    )
    serialized = json.dumps(model_input, sort_keys=True)

    assert set(model_input) == set(recorder.MODEL_INPUT_FIELDS)
    for forbidden in (
        'binary_sensor_bits',
        'switch_states',
        'stopper_states',
        'shuttle_command_state',
        'time_since_last_sensor_event',
        'DZI2R',
        'DA3IL',
        'EXTERIOR',
        'INTERIOR',
        'A12E',
        'segment',
        '"x"',
        '"y"',
        '"z"',
        'yaw',
        '"s"',
        'distance_to_switch',
        'normalized_position',
    ):
        assert forbidden not in serialized

    privileged = recorder._privileged_eval_from_status(
        _status(),
        sensor_event_times={'right': 95.0, 'left': None},
        now_s=100.0,
    )
    assert privileged['expert_sensor_state']['binary_sensor_bits']['right']['DZI2R'] == 1
    assert privileged['expert_sensor_state']['switch_states']['right']['A2'] == 'INTERIOR'
    assert privileged['raw_shuttle_states']['right']['room315_right_shuttle_1']['segment'] == 'A12E'


def test_smoke_event_row_includes_images_binary_state_and_action():
    recorder_module = _load_recorder()
    recorder = _fake_recorder(recorder_module)

    recorder._record_command_event({
        'action': 'switches',
        'side': 'right',
        'switches': {'A3': 'INTERIOR'},
    })
    rows = _jsonl_rows(recorder.event_stream)

    assert len(rows) == 1
    row = rows[0]
    assert row['observation.images.right_rail_rgb'].endswith('/right_rail_rgb/000000.jpg')
    assert row['observation.images.left_rail_rgb'].endswith('/left_rail_rgb/000000.jpg')
    assert 'binary_sensor_bits' not in row['model_input']
    assert 'auxiliary_targets' not in row['model_input']
    assert row['auxiliary_targets']['switch_states']['right']['A2'] == 'INTERIOR'
    assert row['auxiliary_targets']['stopper_states']['right']['A1'] == 'open'
    assert row['auxiliary_targets']['shuttle_visual_region']['right']['region'] == 'yaskawa_hc10dt'
    assert row['privileged_eval']['expert_sensor_state']['binary_sensor_bits']['right']['DZI2R'] == 1
    assert row['privileged_eval']['expert_sensor_state']['binary_sensor_bits']['left']['DA3IL'] == 1
    assert row['structured_rail_state']['rails']['right']['sensor_multi_hot']['DZI2R'] == 1
    assert row['action']['primitive'] == 'SET_SWITCHES'
    assert row['action']['switch_mask']['A3'] == 1
    assert row['action_vector'] == recorder_module._encode_action(row['action'])


def test_smoke_action_json_vector_roundtrip_for_event_v2_primitives():
    recorder = _load_recorder()
    actions = [
        _blank_action(recorder, 'WAIT'),
        _blank_action(recorder, 'DONE'),
        _blank_action(recorder, 'EMERGENCY_STOP', side='left'),
        _blank_action(recorder, 'STOP_NOW'),
        _blank_action(recorder, 'SHUTTLE_ON'),
        _blank_action(recorder, 'SHUTTLE_ON', side='left'),
        _blank_action(recorder, 'SET_SWITCHES'),
        _blank_action(recorder, 'SET_STOPPERS', side='left'),
    ]
    actions[0]['wait_condition'] = 'target_sensor_active'
    actions[0]['target_id'] = 'DZI2R'
    actions[1]['wait_condition'] = 'terminal'
    actions[1]['target_id'] = 'terminal'
    actions[1]['reason'] = 'task_succeeded'
    actions[2]['reason'] = 'emergency'
    actions[4]['speed_mps'] = 0.45
    actions[5]['speed_mps'] = 0.12
    for action in actions[3:6]:
        action['wait_condition'] = 'shuttle_command_applied'
        action['target_id'] = f'{action["side"]}_shuttle'
        action['reason'] = 'shuttle_start' if action['primitive'].startswith('SHUTTLE_ON') else 'shuttle_stop'
    actions[6]['switch_mask']['A3'] = 1
    actions[6]['switch_values']['A3'] = 'INTERIOR'
    actions[6]['wait_condition'] = 'switch_state_match'
    actions[6]['target_id'] = 'A3'
    actions[6]['reason'] = 'switch_update'
    actions[7]['stopper_mask']['A4'] = 1
    actions[7]['stopper_values']['A4'] = 'closed'
    actions[7]['wait_condition'] = 'stopper_state_match'
    actions[7]['target_id'] = 'A4'
    actions[7]['reason'] = 'stopper_update'

    for action in actions:
        encoded = recorder._encode_action(action)
        assert recorder._decode_action(encoded) == action


def test_smoke_safety_decoder_rejects_invalid_switch_and_move_combos():
    supervisor = _load_supervisor()
    rails = _rails_for_safety()
    common = {
        'rails': rails,
        'route_templates': {},
        'emergency_stop': False,
        'active_tasks': {},
        'slot_sensor_by_side': {
            'right': {'1': 'DZI1R', '2': 'DZI2R', '3': 'DZI3R', '4': 'DZI4R'},
            'left': {'1': 'DZI1L', '2': 'DZI2L', '3': 'DZI3L', '4': 'DZI4L'},
        },
        'default_shuttle_name_by_side': {
            'right': 'room315_right_shuttle_1',
            'left': 'room315_left_shuttle_1',
        },
    }
    unsafe_switch_vector = _event_vector(
        supervisor,
        primitive='SET_SWITCHES',
        side='right',
        switch_values={'A3': 'INTERIOR'},
        wait_condition='switch_state_match',
        target_id='A3',
        reason='switch_update',
    )
    unsafe_move_vector = _event_vector(
        supervisor,
        primitive='SHUTTLE_ON',
        side='right',
        speed_mps=0.4,
        wait_condition='none',
        target_id='none',
        reason='shuttle_start',
    )
    blocked_move_vector = _event_vector(
        supervisor,
        primitive='SHUTTLE_ON',
        side='right',
        speed_mps=0.4,
        wait_condition='shuttle_command_applied',
        target_id='right_shuttle',
        reason='shuttle_start',
    )

    switch_decision = supervisor.decode_and_validate(unsafe_switch_vector, **common)
    move_decision = supervisor.decode_and_validate(unsafe_move_vector, **common)
    rails['right']['stoppers']['A2'] = '1'
    blocked_decision = supervisor.decode_and_validate(blocked_move_vector, **common)

    assert switch_decision['accepted'] is False
    assert 'guarded segment' in switch_decision['reason']
    assert switch_decision['rejected_action'] == unsafe_switch_vector
    assert move_decision['accepted'] is False
    assert 'missing wait_condition or target_id' in move_decision['reason']
    assert blocked_decision['accepted'] is False
    assert 'path blocked by closed stopper' in blocked_decision['reason']
