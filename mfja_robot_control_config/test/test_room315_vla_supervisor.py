#!/usr/bin/env python3

import importlib.util
from pathlib import Path
from types import MethodType


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_vla_supervisor.py'


def _load_supervisor_module():
    spec = importlib.util.spec_from_file_location('room_315_vla_supervisor', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _fake_supervisor(module):
    supervisor = module.Room315VlaSupervisor.__new__(module.Room315VlaSupervisor)
    supervisor.defaults = {'speed': 0.2}
    supervisor.slot_sensor_by_side = {
        'right': {'1': 'DZI1R', '2': 'DZI2R', '3': 'DZI3R', '4': 'DZI4R'},
        'left': {'1': 'DZI1L', '2': 'DZI2L', '3': 'DZI3L', '4': 'DZI4L'},
    }
    supervisor.rails = {
        'right': {
            'shuttles': {
                'room315_right_shuttle_1': {'mode': 'STOPPED', 'segment': 'A12E'}
            },
            'switches': {'A1': 'E', 'A2': 'E', 'A3': 'E', 'A4': 'E'},
            'stoppers': {'A1': '0', 'A2': '0', 'A3': '0', 'A4': '0'},
            'payloads': {},
            'active_sensors': [],
            'active_position_sensors': [
                {'name': 'DZI1R', 'shuttle': 'room315_right_shuttle_1'},
            ],
        },
        'left': {
            'shuttles': {},
            'switches': {'A1': 'E', 'A2': 'E', 'A3': 'E', 'A4': 'E'},
            'stoppers': {'A1': '0', 'A2': '0', 'A3': '0', 'A4': '0'},
            'payloads': {},
            'active_sensors': [],
            'active_position_sensors': [],
        },
    }
    supervisor.active_tasks = {}
    supervisor.completed_tasks = []
    supervisor.completed_task_limit = 3
    supervisor.task_counter = 0
    supervisor.emergency_stop = False
    supervisor.last_result = ''
    supervisor.last_primitive_command = None
    supervisor.safety_metrics = module._empty_safety_metrics()
    supervisor.block_reservations = {}
    supervisor.station_slot_targets = {}
    supervisor.min_headway_blocks = 1
    supervisor.max_recovery_retries = 1
    supervisor.safety_recovery = {
        'phase': 'idle',
        'retry_count': 0,
        'reason': '',
        'next_step': '',
        'model_input_exposure': 'excluded',
    }
    supervisor.last_safety_decision = None
    supervisor.safety_decisions = []
    supervisor.safety_decision_log_limit = 3

    def set_result(self, result):
        self.last_result = result

    def publish_switches(self, side, assignments, *, task_id=''):
        self._record_primitive_command(task_id, 'switches', side, {'switches': assignments})

    def publish_stoppers(self, side, assignments, *, task_id=''):
        self._record_primitive_command(task_id, 'stoppers', side, {'stoppers': assignments})

    def publish_shuttle(
        self,
        side,
        name,
        command,
        *,
        start_slot='',
        target_slot='',
        speed=None,
        task_id='',
    ):
        self._record_primitive_command(
            task_id,
            'shuttle',
            side,
            {
                'shuttle': str(name),
                'command': str(command).upper(),
                'start_slot': start_slot,
                'target_slot': target_slot,
                'speed': float(speed or self._default_speed()),
            },
        )

    supervisor._set_result = MethodType(set_result, supervisor)
    supervisor._publish_switches = MethodType(publish_switches, supervisor)
    supervisor._publish_stoppers = MethodType(publish_stoppers, supervisor)
    supervisor._publish_shuttle_command = MethodType(publish_shuttle, supervisor)
    return supervisor


def test_supervisor_ingests_payload_state_as_privileged_snapshot():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    message = module.String()
    message.data = (
        '{"shuttles": ['
        '{"entity_name": "room315_right_shuttle_2", "shuttle_id": "right_shuttle_2", '
        '"side": "right", "loaded": true, "payload_type": "box"}'
        ']}'
    )

    supervisor._on_payload_state('right', message)

    payloads = supervisor.rails['right']['payloads']
    assert payloads['room315_right_shuttle_2']['loaded'] is True
    assert payloads['room315_right_shuttle_2']['payload_type'] == 'box'
    snapshot = supervisor._payload_state_snapshot()
    assert snapshot['by_shuttle']['room315_right_shuttle_2']['model_input_exposure'] == 'excluded'


def test_payload_language_does_not_map_to_direct_route_command():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)

    command = supervisor._parse_text_command('move R2 carrying a part to Staubli')

    assert command == {'action': 'status'}


def test_supervisor_primitive_commands_remain_backward_compatible():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    called = {}

    def execute_switches(self, command):
        called['switches'] = command

    supervisor._execute_switches = MethodType(execute_switches, supervisor)

    supervisor._execute({
        'action': 'switches',
        'side': 'right',
        'switches': {'ALL': 'EXTERIOR'},
    })

    assert called['switches']['switches'] == {'ALL': 'EXTERIOR'}


def test_supervisor_propagates_target_slot_to_low_level_controller():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)

    supervisor._execute_shuttle_command({
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'room315_right_shuttle_1',
        'command': 'ON',
        'speed': 0.2,
        'target_slot': '3',
    })

    assert supervisor.last_primitive_command['target_slot'] == '3'


def test_safety_decoder_accepts_safe_partial_switch_action():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)

    decision = supervisor._safety_decode_command({
        'action': 'switch',
        'side': 'right',
        'switches': {'A3': 'I'},
    })

    assert decision['accepted'] is True
    assert decision['safe_correction'] is True
    assert decision['corrected_action'] == {
        'action': 'switches',
        'side': 'right',
        'switches': {'A3': 'INTERIOR'},
    }


def test_safety_decoder_rejects_unsafe_switch_change_near_moving_shuttle():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['shuttles']['room315_right_shuttle_1'] = {
        'mode': 'MOVING',
        'segment': 'A34E',
        'speed': 0.2,
    }

    decision = supervisor._safety_decode_command({
        'action': 'switches',
        'side': 'right',
        'switches': {'A3': 'INTERIOR'},
    })

    assert decision['accepted'] is False
    assert 'unsafe switch change' in decision['reason']


def test_safety_decoder_rejects_unsafe_stopper_close_near_moving_shuttle():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['shuttles']['room315_right_shuttle_1'] = {
        'mode': 'MOVING',
        'segment': 'A34E',
        'speed': 0.2,
    }

    decision = supervisor._safety_decode_command({
        'action': 'stoppers',
        'side': 'right',
        'stoppers': {'A4': '1'},
    })

    assert decision['accepted'] is False
    assert 'unsafe stopper close' in decision['reason']


def test_safety_decoder_rejects_shuttle_on_when_path_blocked():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['stoppers']['A2'] = '1'

    decision = supervisor._safety_decode_command({
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'room315_right_shuttle_1',
        'command': 'ON',
    })

    assert decision['accepted'] is False
    assert 'path blocked by closed stopper' in decision['reason']


def test_safety_decoder_rejects_unknown_localization_before_motion():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['shuttles']['room315_right_shuttle_1'] = {
        'mode': 'STOPPED',
        'speed': 0.0,
    }
    supervisor.rails['right']['active_position_sensors'] = []

    decision = supervisor._safety_decode_command({
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'room315_right_shuttle_1',
        'command': 'ON',
    })

    assert decision['accepted'] is False
    assert 'unknown localization' in decision['reason']


def test_safety_decoder_rejects_obstacle_appearance_but_allows_stop():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['obstacles'] = ['box_on_track']

    rejected = supervisor._safety_decode_command({
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'room315_right_shuttle_1',
        'command': 'ON',
    })
    stopped = supervisor._safety_decode_command({
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'room315_right_shuttle_1',
        'command': 'OFF',
    })

    assert rejected['accepted'] is False
    assert 'obstacle appearance' in rejected['reason']
    assert stopped['accepted'] is True


def test_safety_decoder_rejects_occupied_target_slot_from_trusted_sensors():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['shuttles']['room315_right_shuttle_2'] = {
        'mode': 'STOPPED',
        'segment': 'A34E',
        'speed': 0.0,
    }
    supervisor.rails['right']['active_position_sensors'] = [
        {'name': 'DZI1R', 'shuttle': 'room315_right_shuttle_1'},
        {'name': 'DZI3R', 'shuttle': 'room315_right_shuttle_2'},
    ]

    decision = supervisor._safety_decode_command({
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'room315_right_shuttle_1',
        'command': 'ON',
        'target_slot': '3',
    })

    assert decision['accepted'] is False
    assert 'target slot right:slot:3 is occupied by room315_right_shuttle_2' in decision['reason']


def test_safety_decoder_rejects_stale_conflicting_dropout_and_timeout_state():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)

    supervisor.rails['right']['state_status'] = 'stale'
    stale = supervisor._safety_decode_command({
        'action': 'switches',
        'side': 'right',
        'switches': {'A1': 'INTERIOR'},
    })
    assert stale['accepted'] is False
    assert 'stale' in stale['reason']

    supervisor.rails['right'].pop('state_status')
    supervisor.rails['right']['shuttles']['room315_right_shuttle_1']['status'] = 'conflicting'
    conflicting = supervisor._safety_decode_command({
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'room315_right_shuttle_1',
        'command': 'ON',
    })
    assert conflicting['accepted'] is False
    assert 'conflicting' in conflicting['reason']

    supervisor.rails['right']['shuttles']['room315_right_shuttle_1'].pop('status')
    supervisor.rails['right']['sensor_dropout'] = True
    dropout = supervisor._safety_decode_command({
        'action': 'stoppers',
        'side': 'right',
        'stoppers': {'A1': '1'},
    })
    assert dropout['accepted'] is False
    assert 'sensor dropout' in dropout['reason']

    supervisor.rails['right'].pop('sensor_dropout')
    supervisor.rails['right']['timed_out'] = True
    timeout = supervisor._safety_decode_command({
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'room315_right_shuttle_1',
        'command': 'ON',
    })
    assert timeout['accepted'] is False
    assert 'timeout' in timeout['reason']


def test_safety_decoder_allows_explicit_target_stopper_stop():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['stoppers']['A2'] = '1'

    decision = supervisor._safety_decode_command({
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'room315_right_shuttle_1',
        'command': 'ON',
        'target_stopper': 'A2',
    })

    assert decision['accepted'] is True
    assert decision['corrected_action']['target_stopper'] == 'A2'


def test_removed_action_vector_payload_is_rejected_even_with_primitive_context():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['stoppers']['A4'] = '1'

    decision = supervisor._decode_and_record_safety({
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'right_shuttle_1',
        'command': 'ON',
        'speed': 0.3,
        'target_stopper': 'A4',
        'action_vector': [0.0] * 24,
    })

    assert decision['accepted'] is False
    assert decision['illegal_proposal'] is True
    assert 'removed action_vector commands are not supported' in decision['reason']
    assert supervisor.last_safety_decision['rejected_action']['action_vector'] == [0.0] * 24


def test_safety_decoder_rejects_emergency_and_falling_states():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.emergency_stop = True

    emergency_decision = supervisor._safety_decode_command({
        'action': 'switches',
        'side': 'right',
        'switches': {'A1': 'INTERIOR'},
    })
    assert emergency_decision['accepted'] is False
    assert 'emergency stop is active' in emergency_decision['reason']

    supervisor.emergency_stop = False
    supervisor.rails['right']['shuttles']['room315_right_shuttle_1']['mode'] = 'FALLING'
    falling_decision = supervisor._safety_decode_command({
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'room315_right_shuttle_1',
        'command': 'ON',
    })
    assert falling_decision['accepted'] is False
    assert 'falling state rejection' in falling_decision['reason']


def test_recoverable_safety_rejection_safe_stops_then_fail_safe_aborts():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    rejected = {
        'accepted': False,
        'reason': 'obstacle appearance on right rail: box',
    }

    recovered = supervisor._handle_recoverable_safety_rejection(rejected)

    assert recovered is True
    assert supervisor.safety_recovery['phase'] == 'safe_stop_reobserve_replan'
    assert supervisor.safety_recovery['next_step'] == 'reacquire_observations_then_request_new_plan'
    assert supervisor.safety_recovery['model_input_exposure'] == 'excluded'
    assert supervisor.safety_metrics['safety_recovery_count'] == 1
    assert supervisor.last_primitive_command['action'] == 'shuttle'
    assert supervisor.last_primitive_command['command'] == 'OFF'

    aborted = supervisor._handle_recoverable_safety_rejection(rejected)

    assert aborted is True
    assert supervisor.emergency_stop is True
    assert supervisor.safety_recovery['phase'] == 'fail_safe_abort'
    assert supervisor.safety_metrics['fail_safe_abort_count'] == 1
    assert supervisor.last_primitive_command['action'] == 'stoppers'


def test_safety_decoder_metrics_track_illegal_proposal_rate():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)

    accepted = supervisor._safety_decode_command({'action': 'status'})
    rejected = supervisor._safety_decode_command({'action': 'switches', 'side': 'middle'})
    supervisor._record_safety_decision(accepted)
    supervisor._record_safety_decision(rejected)

    assert supervisor.safety_metrics['total_proposed_actions'] == 2
    assert supervisor.safety_metrics['accepted_actions'] == 1
    assert supervisor.safety_metrics['rejected_actions'] == 1
    assert supervisor.safety_metrics['illegal_proposal_rate'] == 0.5
    assert supervisor.safety_metrics['rejected_action_rate'] == 0.5
    assert supervisor.safety_metrics['rejection_reasons'][rejected['reason']] == 1


def test_numeric_vector_command_is_rejected_and_logged():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    action_vector = [0.0] * 24

    decision = supervisor._decode_and_record_safety(action_vector)

    assert decision['accepted'] is False
    assert decision['executed_action'] is None
    assert supervisor.last_safety_decision['raw_action'] == action_vector
    assert supervisor.last_safety_decision['illegal_proposal'] is True
    assert supervisor.last_safety_decision['rejected_action'] == action_vector
    assert supervisor.safety_metrics['rejected_actions'] == 1
    assert 'removed action_vector commands are not supported' in decision['reason']


def test_primitive_command_json_remains_the_only_executable_supervisor_payload():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)

    decision = supervisor._decode_and_record_safety({
        'action': 'switches',
        'side': 'right',
        'switches': {'A3': 'INTERIOR'},
    })

    assert decision['accepted'] is True
    assert decision['executed_action'] == {
        'action': 'switches',
        'side': 'right',
        'switches': {'A3': 'INTERIOR'},
    }
    assert supervisor.safety_metrics['accepted_actions'] == 1
