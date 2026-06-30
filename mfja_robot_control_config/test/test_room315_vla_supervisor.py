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
    supervisor.last_safety_decision = None
    supervisor.safety_decisions = []
    supervisor.safety_decision_log_limit = 3

    def set_result(self, result):
        self.last_result = result

    def publish_switches(self, side, assignments, *, task_id=''):
        self._record_primitive_command(task_id, 'switches', side, {'switches': assignments})

    def publish_stoppers(self, side, assignments, *, task_id=''):
        self._record_primitive_command(task_id, 'stoppers', side, {'stoppers': assignments})

    def publish_shuttle(self, side, name, command, *, start_slot='', speed=None, task_id=''):
        self._record_primitive_command(
            task_id,
            'shuttle',
            side,
            {
                'shuttle': str(name),
                'command': str(command).upper(),
                'start_slot': start_slot,
                'speed': float(speed or self._default_speed()),
            },
        )

    supervisor._set_result = MethodType(set_result, supervisor)
    supervisor._publish_switches = MethodType(publish_switches, supervisor)
    supervisor._publish_stoppers = MethodType(publish_stoppers, supervisor)
    supervisor._publish_shuttle_command = MethodType(publish_shuttle, supervisor)
    return supervisor


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

    def set_field(name, value):
        values[module.EVENT_ACTION_VECTOR_FIELDS.index(name)] = float(value)

    set_field('primitive_id', primitive_ids[primitive])
    set_field('side_id', side_ids[side])
    set_field('speed_mps', speed_mps)
    for switch_name, state in switch_values.items():
        set_field(f'switch_mask_{switch_name}', 1)
        set_field(f'switch_value_{switch_name}', {'EXTERIOR': 1, 'INTERIOR': 2}[state])
    for stopper_name, state in stopper_values.items():
        set_field(f'stopper_mask_{stopper_name}', 1)
        set_field(f'stopper_value_{stopper_name}', {'0': 1, '1': 2, 'open': 1, 'closed': 2}[state])
    set_field('wait_condition_id', wait_ids[wait_condition])
    set_field('target_id', target_ids[target_id])
    set_field('reason_id', reason_ids[reason])
    return values


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


def test_payload_language_no_longer_maps_to_legacy_route_command():
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


def test_action_vector_command_preserves_target_stopper_context():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['stoppers']['A4'] = '1'
    vector = _event_vector(
        module,
        primitive='SHUTTLE_ON',
        side='right',
        speed_mps=0.3,
        wait_condition='shuttle_command_applied',
        target_id='right_shuttle',
        reason='shuttle_start',
    )

    decision = supervisor._decode_and_record_safety({
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'right_shuttle_1',
        'command': 'ON',
        'speed': 0.3,
        'target_stopper': 'A4',
        'action_vector': vector,
    })

    assert decision['accepted'] is True
    assert decision['corrected_action']['target_stopper'] == 'A4'
    assert decision['raw_action'] == vector
    assert decision['original_action']['target_stopper'] == 'A4'


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


def test_action_vector_decode_and_validate_accepts_safe_switch_command():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    action_vector = _event_vector(
        module,
        primitive='SET_SWITCHES',
        side='right',
        switch_values={'A3': 'INTERIOR'},
        wait_condition='switch_state_match',
        target_id='A3',
        reason='switch_update',
    )

    decision = supervisor.decode_and_validate(action_vector)

    assert decision['accepted'] is True
    assert decision['raw_action'] == action_vector
    assert decision['illegal_proposal'] is False
    assert decision['rejected_action'] is None
    assert decision['executed_action'] == {
        'action': 'switches',
        'side': 'right',
        'switches': {'A3': 'INTERIOR'},
    }


def test_action_vector_rejects_switch_near_occupied_guarded_segment():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['shuttles']['room315_right_shuttle_1'] = {
        'mode': 'STOPPED',
        'segment': 'A34E',
        'speed': 0.0,
    }
    action_vector = _event_vector(
        module,
        primitive='SET_SWITCHES',
        side='right',
        switch_values={'A3': 'INTERIOR'},
        wait_condition='switch_state_match',
        target_id='A3',
        reason='switch_update',
    )

    decision = supervisor.decode_and_validate(action_vector)

    assert decision['accepted'] is False
    assert decision['illegal_proposal'] is True
    assert decision['rejected_action'] == action_vector
    assert decision['executed_action'] is None
    assert 'guarded segment' in decision['reason']


def test_action_vector_accepts_single_gate_switch_when_shuttle_is_staged_at_gate():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['shuttles']['room315_right_shuttle_1'] = {
        'mode': 'WAITING',
        'segment': 'A23',
        'speed': 0.08,
    }
    action_vector = _event_vector(
        module,
        primitive='SET_SWITCHES',
        side='right',
        switch_values={'A3': 'INTERIOR'},
        wait_condition='switch_state_match',
        target_id='A3',
        reason='switch_update',
    )

    decision = supervisor.decode_and_validate(action_vector)

    assert decision['accepted'] is True
    assert decision['executed_action']['switches'] == {'A3': 'INTERIOR'}


def test_action_vector_accepts_left_a3_switch_at_left_gate_context():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['left']['shuttles']['room315_left_shuttle_2'] = {
        'mode': 'STOPPED',
        'segment': 'A23',
        's': 1.069,
        'speed': 0.0,
    }
    action_vector = _event_vector(
        module,
        primitive='SET_SWITCHES',
        side='left',
        switch_values={'A3': 'INTERIOR'},
        wait_condition='switch_state_match',
        target_id='A3',
        reason='switch_update',
    )

    decision = supervisor.decode_and_validate(action_vector)

    assert decision['accepted'] is True
    assert decision['executed_action']['switches'] == {'A3': 'INTERIOR'}


def test_action_vector_accepts_single_gate_restore_after_blocker_clears_switch():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['shuttles']['room315_right_shuttle_2'] = {
        'mode': 'WAITING',
        'segment': 'A34I',
        's': 0.7083,
        'speed': 0.08,
    }
    action_vector = _event_vector(
        module,
        primitive='SET_SWITCHES',
        side='right',
        switch_values={'A3': 'EXTERIOR'},
        wait_condition='switch_state_match',
        target_id='A3',
        reason='switch_update',
    )

    decision = supervisor.decode_and_validate(action_vector)

    assert decision['accepted'] is True
    assert decision['executed_action']['switches'] == {'A3': 'EXTERIOR'}


def test_action_vector_loop_transition_requires_stop_and_side_specific_gate():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    action_vector = _event_vector(
        module,
        primitive='SET_SWITCHES',
        side='right',
        switch_values={
            'A1': 'INTERIOR',
            'A2': 'INTERIOR',
            'A3': 'INTERIOR',
            'A4': 'INTERIOR',
        },
        wait_condition='switch_state_match',
        target_id='ALL_SWITCHES',
        reason='switch_update',
    )

    supervisor.rails['right']['shuttles']['room315_right_shuttle_1'] = {
        'mode': 'MOVING',
        'segment': 'A23',
        'speed': 0.2,
    }
    moving_decision = supervisor.decode_and_validate(action_vector)
    assert moving_decision['accepted'] is False
    assert 'must STOP before switching loop mode' in moving_decision['reason']

    supervisor.rails['right']['shuttles']['room315_right_shuttle_1'] = {
        'mode': 'STOPPED',
        'segment': 'A12E',
        'speed': 0.0,
    }
    wrong_gate_decision = supervisor.decode_and_validate(action_vector)
    assert wrong_gate_decision['accepted'] is False
    assert 'side-specific gate A3' in wrong_gate_decision['reason']

    supervisor.rails['right']['shuttles']['room315_right_shuttle_1'] = {
        'mode': 'STOPPED',
        'segment': 'A34E',
        'speed': 0.0,
    }
    staged_decision = supervisor.decode_and_validate(action_vector)
    assert staged_decision['accepted'] is True
    assert staged_decision['executed_action']['switches'] == {
        'A1': 'INTERIOR',
        'A2': 'INTERIOR',
        'A3': 'INTERIOR',
        'A4': 'INTERIOR',
    }


def test_action_vector_all_switch_noop_does_not_require_loop_gate_staging():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    action_vector = _event_vector(
        module,
        primitive='SET_SWITCHES',
        side='right',
        switch_values={
            'A1': 'EXTERIOR',
            'A2': 'EXTERIOR',
            'A3': 'EXTERIOR',
            'A4': 'EXTERIOR',
        },
        wait_condition='switch_state_match',
        target_id='ALL_SWITCHES',
        reason='switch_update',
    )

    decision = supervisor.decode_and_validate(action_vector)

    assert decision['accepted'] is True
    assert decision['executed_action']['switches'] == {
        'A1': 'EXTERIOR',
        'A2': 'EXTERIOR',
        'A3': 'EXTERIOR',
        'A4': 'EXTERIOR',
    }


def test_action_vector_loop_transition_uses_left_gate():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['left']['shuttles'] = {
        'room315_left_shuttle_1': {'mode': 'STOPPED', 'segment': 'A12E', 'speed': 0.0}
    }
    action_vector = _event_vector(
        module,
        primitive='SET_SWITCHES',
        side='left',
        switch_values={
            'A1': 'INTERIOR',
            'A2': 'INTERIOR',
            'A3': 'INTERIOR',
            'A4': 'INTERIOR',
        },
        wait_condition='switch_state_match',
        target_id='ALL_SWITCHES',
        reason='switch_update',
    )

    wrong_gate_decision = supervisor.decode_and_validate(action_vector)
    assert wrong_gate_decision['accepted'] is False
    assert 'side-specific gate A3' in wrong_gate_decision['reason']

    supervisor.rails['left']['shuttles']['room315_left_shuttle_1'] = {
        'mode': 'STOPPED',
        'segment': 'A34E',
        'speed': 0.0,
    }
    staged_decision = supervisor.decode_and_validate(action_vector)
    assert staged_decision['accepted'] is True
    assert staged_decision['executed_action']['side'] == 'left'


def test_action_vector_rejects_shuttle_on_without_wait_or_target():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    unsafe_vector = _event_vector(
        module,
        primitive='SHUTTLE_ON',
        side='right',
        speed_mps=0.35,
        wait_condition='none',
        target_id='none',
        reason='shuttle_start',
    )

    rejected = supervisor.decode_and_validate(unsafe_vector)

    assert rejected['accepted'] is False
    assert 'missing wait_condition or target_id' in rejected['reason']

    safe_vector = _event_vector(
        module,
        primitive='SHUTTLE_ON',
        side='right',
        speed_mps=0.35,
        wait_condition='shuttle_command_applied',
        target_id='right_shuttle',
        reason='shuttle_start',
    )
    accepted = supervisor.decode_and_validate(safe_vector)

    assert accepted['accepted'] is True
    assert accepted['executed_action']['command'] == 'ON'
    assert accepted['executed_action']['speed'] == 0.35


def test_action_vector_emergency_stop_is_allowed_and_logged():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.emergency_stop = True
    action_vector = _event_vector(
        module,
        primitive='EMERGENCY_STOP',
        side='left',
        reason='emergency',
    )

    decision = supervisor._decode_and_record_safety({'action_vector': action_vector})

    assert decision['accepted'] is True
    assert decision['executed_action'] == {'action': 'emergency_stop'}
    assert supervisor.last_safety_decision['raw_action'] == action_vector
    assert supervisor.last_safety_decision['illegal_proposal'] is False
    assert supervisor.safety_metrics['accepted_actions'] == 1


def test_action_vector_rejection_log_contains_required_fields():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    action_vector = _event_vector(
        module,
        primitive='SHUTTLE_ON',
        side='left',
        speed_mps=0.1,
        wait_condition='none',
        target_id='none',
        reason='shuttle_start',
    )

    decision = supervisor._decode_and_record_safety(action_vector)

    assert decision['accepted'] is False
    assert supervisor.last_safety_decision['raw_action'] == action_vector
    assert supervisor.last_safety_decision['illegal_proposal'] is True
    assert supervisor.last_safety_decision['rejected_action'] == action_vector
    assert supervisor.last_safety_decision['executed_action'] is None
    assert supervisor.safety_metrics['rejected_actions'] == 1
