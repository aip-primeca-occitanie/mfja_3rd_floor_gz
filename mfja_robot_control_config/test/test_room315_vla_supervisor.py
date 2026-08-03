#!/usr/bin/env python3

import copy
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
                'room315_right_shuttle_1': {
                    'mode': 'STOPPED',
                    'segment': 'A12E',
                    's': 0.917,
                    'speed': 0.0,
                }
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


def _guarded_normalization_command(side='right', public_segment='A34I'):
    short_id = 'R1' if side == 'right' else 'L1'
    shuttle_id = f'{side}_shuttle_1'
    gate = 'A1' if public_segment == 'A12I' else 'A3'
    exit_gate = 'A2' if gate == 'A1' else 'A4'
    internal_segment = (
        public_segment
        if side == 'right'
        else {'A12I': 'A34I', 'A34I': 'A12I'}[public_segment]
    )
    entry_sensor = f'DA{gate[-1]}I{"R" if side == "right" else "L"}'
    switches = {
        'A1': 'interior' if gate == 'A1' else 'exterior',
        'A2': 'interior' if gate == 'A1' else 'exterior',
        'A3': 'interior' if gate == 'A3' else 'exterior',
        'A4': 'interior' if gate == 'A3' else 'exterior',
    }
    stoppers = {
        device: 'closed' if device == exit_gate else 'open'
        for device in ('A1', 'A2', 'A3', 'A4')
    }
    proof = {
        'side': side,
        'switches': switches,
        'stoppers': stoppers,
        'normal_route': False,
        'clearance_mode': True,
        'all_stoppers_open': False,
        'reconfiguration_required': False,
        'reconfiguration_safe': False,
        'clearance_pause_safe': True,
        'interior_shuttles': [shuttle_id],
        'visually_interior_shuttles': [shuttle_id],
        'certified_interior_shuttles': [shuttle_id],
        'certified_stopped_interior_shuttles': [shuttle_id],
        'uncertified_interior_shuttles': [],
        'certificate_segment_mismatches': [],
        'clearance_lifecycle_certified_stopped_interior_shuttles': [
            shuttle_id
        ],
        'clearance_lifecycle_uncertified_interior_shuttles': [],
        'clearance_lifecycle_visual_disagreements': [],
        'clearance_lifecycle_visual_prediction_preserved': True,
        'clearance_lifecycle_certificate_used_as_localization': False,
        'certificate_segment_consistency': {
            shuttle_id: {
                'required': True,
                'satisfied': True,
                'certificate_target_public_segment': public_segment,
                'certificate_target_internal_segment': internal_segment,
                'accepted_visual_internal_segment': internal_segment,
                'certificate_used_as_localization': False,
            },
        },
        'external_obstacles': [],
        'controller_position_fields_used_for_localization': False,
    }
    certificate = {
        'identity': short_id,
        'shuttle': shuttle_id,
        'side': side,
        'target_segment': public_segment,
        'target_s_m': 0.75,
        'entry_sensor': entry_sensor,
        'entry_sensor_identity_confirmed': True,
        'controller_stop_confirmed': True,
        'post_stop_visual_frame_received': True,
        'bounded_commanded_motion_completed': True,
        'clearance_mode_held': True,
        'normal_route_restored': False,
        'matched_by': 'interior_entry_sensor_plus_bounded_travel_time',
        'model_prediction_replaced': False,
        'controller_position_fields_used_for_localization': False,
    }
    return {
        'action': 'switches',
        'side': side,
        'switches': {
            'A1': 'EXTERIOR',
            'A2': 'EXTERIOR',
            'A3': 'EXTERIOR',
            'A4': 'EXTERIOR',
        },
        'closed_loop_executive': {
            'mode': 'restore_normal_route_after_interior_clearance',
            'problem_name': f'closed-loop-{side}-normalization',
            'plan_length': 1,
            'step_index': 0,
            'symbolic_step': (
                f'finish_route_clearance {shuttle_id} {side} '
                f'{side}_slot_4 {side}_slot_2'
            ),
            'localization_source': 'accepted_visual_state',
            'controller_position_fields_used_for_localization': False,
            'route_normalization_proof': proof,
            'runtime_clearance_certificates': {short_id: certificate},
        },
    }


def _stage_guarded_interior_shuttle(
    supervisor,
    *,
    side='right',
    public_segment='A34I',
):
    entity = f'room315_{side}_shuttle_1'
    segment = (
        public_segment
        if side == 'right'
        else {'A12I': 'A34I', 'A34I': 'A12I'}[public_segment]
    )
    gate = 'A1' if public_segment == 'A12I' else 'A3'
    exit_gate = 'A2' if gate == 'A1' else 'A4'
    supervisor.rails[side]['shuttles'] = {
        entity: {
            'mode': 'DISABLED',
            'segment': segment,
            's': 0.75,
            # The Gazebo ShuttleState contract retains the configured travel
            # speed after OFF. It is not an instantaneous velocity.
            'speed': 0.2,
        },
    }
    supervisor.rails[side]['switches'] = {
        'A1': 'I' if gate == 'A1' else 'E',
        'A2': 'I' if gate == 'A1' else 'E',
        'A3': 'I' if gate == 'A3' else 'E',
        'A4': 'I' if gate == 'A3' else 'E',
    }
    supervisor.rails[side]['stoppers'] = {
        device: '1' if device == exit_gate else '0'
        for device in ('A1', 'A2', 'A3', 'A4')
    }
    supervisor.rails[side]['active_sensors'] = []
    supervisor.rails[side]['active_position_sensors'] = []


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
        's': 0.1,
        'speed': 0.2,
    }

    decision = supervisor._safety_decode_command({
        'action': 'switches',
        'side': 'right',
        'switches': {'A3': 'INTERIOR'},
    })

    assert decision['accepted'] is False
    assert 'unsafe switch change' in decision['reason']


def test_safety_decoder_rejects_switch_change_on_stopped_guarded_segment():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    _stage_guarded_interior_shuttle(supervisor)
    supervisor.rails['right']['shuttles'][
        'room315_right_shuttle_1'
    ]['s'] = 1.4

    decision = supervisor._safety_decode_command({
        'action': 'switches',
        'side': 'right',
        'switches': {'ALL': 'EXTERIOR'},
    })

    assert decision['accepted'] is False
    assert 'guarded segment is occupied' in decision['reason']
    assert 'missing closed-loop route-normalization proof' in decision['reason']


def test_safety_decoder_accepts_both_interior_branches_on_both_rails():
    module = _load_supervisor_module()
    for side in ('right', 'left'):
        for public_segment in ('A12I', 'A34I'):
            supervisor = _fake_supervisor(module)
            _stage_guarded_interior_shuttle(
                supervisor,
                side=side,
                public_segment=public_segment,
            )
            command = _guarded_normalization_command(
                side,
                public_segment,
            )

            decision = supervisor._safety_decode_command(command)

            assert decision['accepted'] is True
            assert decision['corrected_action']['switches'] == {
                'A1': 'EXTERIOR',
                'A2': 'EXTERIOR',
                'A3': 'EXTERIOR',
                'A4': 'EXTERIOR',
            }
            assert (
                decision['corrected_action']['closed_loop_executive'][
                    'controller_position_fields_used_for_localization'
                ]
                is False
            )


def test_active_clearance_finish_accepts_executor_proved_visual_disagreement():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    _stage_guarded_interior_shuttle(supervisor)
    command = _guarded_normalization_command()
    proof = command['closed_loop_executive']['route_normalization_proof']
    shuttle = 'right_shuttle_1'
    proof.update({
        'visually_interior_shuttles': [],
        'certified_stopped_interior_shuttles': [],
        'uncertified_interior_shuttles': [shuttle],
        'certificate_segment_mismatches': [shuttle],
        'clearance_lifecycle_visual_disagreements': [shuttle],
    })
    proof['certificate_segment_consistency'][shuttle].update({
        'satisfied': False,
        'accepted_visual_internal_segment': 'A34E',
        'certificate_used_as_persisted_execution_effect': True,
        'planning_origin_segment': 'A34I',
        'raw_visual_prediction_preserved': True,
        'reason': 'certificate_and_visual_segment_disagree',
    })

    decision = supervisor._safety_decode_command(command)

    assert decision['accepted'] is True
    corrected = decision['corrected_action']['closed_loop_executive']
    assert corrected['route_normalization_proof'][
        'clearance_lifecycle_visual_prediction_preserved'
    ] is True
    assert corrected['controller_position_fields_used_for_localization'] is False


def test_active_clearance_finish_accepts_dual_branch_live_replay():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    _stage_guarded_interior_shuttle(supervisor)
    supervisor.rails['right']['shuttles']['room315_right_shuttle_4'] = {
        'mode': 'DISABLED',
        'segment': 'A12I',
        's': 0.8411016464233398,
        'speed': 0.2,
    }
    command = _guarded_normalization_command()
    metadata = command['closed_loop_executive']
    proof = metadata['route_normalization_proof']
    red = 'right_shuttle_1'
    yellow = 'right_shuttle_4'
    proof.update({
        'interior_shuttles': [red, yellow],
        'visually_interior_shuttles': [yellow],
        'certified_interior_shuttles': [red, yellow],
        'certified_stopped_interior_shuttles': [yellow],
        'uncertified_interior_shuttles': [red],
        'certificate_segment_mismatches': [red],
        'clearance_lifecycle_certified_stopped_interior_shuttles': [
            red,
            yellow,
        ],
        'clearance_lifecycle_visual_disagreements': [red],
    })
    proof['certificate_segment_consistency'][red].update({
        'satisfied': False,
        'accepted_visual_internal_segment': 'A34E',
        'certificate_used_as_persisted_execution_effect': True,
        'planning_origin_segment': 'A34I',
        'raw_visual_prediction_preserved': True,
        'reason': 'certificate_and_visual_segment_disagree',
    })
    proof['certificate_segment_consistency'][yellow] = {
        'required': True,
        'satisfied': True,
        'certificate_target_public_segment': 'A12I',
        'certificate_target_internal_segment': 'A12I',
        'accepted_visual_internal_segment': 'A12I',
        'certificate_used_as_localization': False,
    }
    metadata['runtime_clearance_certificates']['R4'] = {
        'identity': 'R4',
        'shuttle': yellow,
        'side': 'right',
        'target_segment': 'A12I',
        'target_s_m': 1.060396,
        'entry_sensor': 'DA1IR',
        'entry_sensor_identity_confirmed': True,
        'controller_stop_confirmed': True,
        'post_stop_visual_frame_received': True,
        'bounded_commanded_motion_completed': True,
        'clearance_mode_held': True,
        'normal_route_restored': False,
        'matched_by': 'interior_entry_sensor_plus_bounded_travel_time',
        'model_prediction_replaced': False,
        'controller_position_fields_used_for_localization': False,
    }

    decision = supervisor._safety_decode_command(command)

    assert decision['accepted'] is True
    assert decision['corrected_action']['switches'] == {
        'A1': 'EXTERIOR',
        'A2': 'EXTERIOR',
        'A3': 'EXTERIOR',
        'A4': 'EXTERIOR',
    }


def test_safety_decoder_requires_disabled_mode_not_retained_speed_for_stop_proof():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    _stage_guarded_interior_shuttle(supervisor)

    stopped = supervisor._safety_decode_command(
        _guarded_normalization_command()
    )

    assert stopped['accepted'] is True
    assert (
        supervisor.rails['right']['shuttles'][
            'room315_right_shuttle_1'
        ]['speed']
        == 0.2
    )

    for mode in ('WAITING', 'MOVING', ''):
        supervisor.rails['right']['shuttles'][
            'room315_right_shuttle_1'
        ]['mode'] = mode
        occupants, reason = module._current_interior_safety_occupants(
            supervisor.rails,
            side='right',
        )
        not_disabled = supervisor._safety_decode_command(
            _guarded_normalization_command()
        )

        assert occupants == {}
        assert 'no explicit disabled controller mode' in reason
        assert not_disabled['accepted'] is False


def test_safety_decoder_accepts_two_stopped_interior_shuttles_with_retained_speed():
    module = _load_supervisor_module()
    for side in ('right', 'left'):
        supervisor = _fake_supervisor(module)
        _stage_guarded_interior_shuttle(supervisor, side=side)
        prefix = 'R' if side == 'right' else 'L'
        segment = 'A34I' if side == 'right' else 'A12I'
        first_entity = f'room315_{side}_shuttle_1'
        second_entity = f'room315_{side}_shuttle_2'
        supervisor.rails[side]['shuttles'][first_entity]['s'] = 0.35
        supervisor.rails[side]['shuttles'][second_entity] = {
            'mode': 'DISABLED',
            'segment': segment,
            's': 0.95,
            'speed': 0.2,
        }

        command = _guarded_normalization_command(side)
        metadata = command['closed_loop_executive']
        proof = metadata['route_normalization_proof']
        metadata['runtime_clearance_certificates'][f'{prefix}1'][
            'target_s_m'
        ] = 0.35
        for field in (
            'interior_shuttles',
            'visually_interior_shuttles',
            'certified_interior_shuttles',
            'certified_stopped_interior_shuttles',
            'clearance_lifecycle_certified_stopped_interior_shuttles',
        ):
            proof[field].append(f'{side}_shuttle_2')
        consistency = proof['certificate_segment_consistency']
        consistency[f'{side}_shuttle_2'] = copy.deepcopy(
            consistency[f'{side}_shuttle_1']
        )
        second_certificate = copy.deepcopy(
            metadata['runtime_clearance_certificates'][f'{prefix}1']
        )
        second_certificate.update({
            'identity': f'{prefix}2',
            'shuttle': f'{side}_shuttle_2',
            'target_s_m': 0.95,
        })
        metadata['runtime_clearance_certificates'][f'{prefix}2'] = second_certificate

        decision = supervisor._safety_decode_command(command)

        assert decision['accepted'] is True
        assert decision['corrected_action']['switches'] == {
            'A1': 'EXTERIOR',
            'A2': 'EXTERIOR',
            'A3': 'EXTERIOR',
            'A4': 'EXTERIOR',
        }
        for state in supervisor.rails[side]['shuttles'].values():
            for switch_name in ('A1', 'A2', 'A3', 'A4'):
                distance, reason = module._shuttle_switch_distance_m(
                    state,
                    switch_name,
                    side=side,
                )
                assert reason == ''
                assert distance > module.SWITCH_CLEAR_DISTANCE_M

        supervisor.rails[side]['shuttles'][second_entity]['mode'] = 'WAITING'
        enabled_waiting = supervisor._safety_decode_command(command)
        assert enabled_waiting['accepted'] is False

        supervisor.rails[side]['shuttles'][second_entity]['mode'] = 'DISABLED'
        metadata['runtime_clearance_certificates'].pop(f'{prefix}2')
        missing_certificate = supervisor._safety_decode_command(command)
        assert missing_certificate['accepted'] is False
        assert (
            'certificates do not match interior occupants'
            in missing_certificate['reason']
        )


def test_safety_decoder_accepts_r4_slot4_clearance_begin_switches():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['shuttles'] = {
        'room315_right_shuttle_4': {
            'mode': 'STOPPED',
            'segment': 'A34E',
            's': 1.523,
            'speed': 0.0,
        },
    }
    supervisor.rails['right']['active_position_sensors'] = [{
        'name': 'DZI4R',
        'shuttle': 'room315_right_shuttle_4',
    }]

    decision = supervisor._safety_decode_command({
        'action': 'switches',
        'side': 'right',
        'switches': {
            'A1': 'EXTERIOR',
            'A2': 'EXTERIOR',
            'A3': 'INTERIOR',
            'A4': 'INTERIOR',
        },
    })

    assert decision['accepted'] is True


def test_safety_decoder_accepts_r1_slot1_topology_switches():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)

    decision = supervisor._safety_decode_command({
        'action': 'switches',
        'side': 'right',
        'switches': {
            'A1': 'INTERIOR',
            'A2': 'INTERIOR',
            'A3': 'EXTERIOR',
            'A4': 'EXTERIOR',
        },
    })

    assert decision['accepted'] is True


def test_safety_decoder_rejects_shuttle_at_exact_switch_clearance_boundary():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    geometry = module._rail_switch_distance_geometry('right')
    connector_length = geometry['segments']['A3E'][2]
    boundary_s = module.SWITCH_CLEAR_DISTANCE_M - connector_length
    supervisor.rails['right']['shuttles'] = {
        'room315_right_shuttle_1': {
            'mode': 'STOPPED',
            'segment': 'A34E',
            's': boundary_s,
            'speed': 0.0,
        },
    }
    supervisor.rails['right']['active_position_sensors'] = []

    decision = supervisor._safety_decode_command({
        'action': 'switches',
        'side': 'right',
        'switches': {'A3': 'INTERIOR'},
    })

    assert decision['accepted'] is False
    assert 'guarded segment is occupied within 0.350m' in decision['reason']


def test_safety_decoder_rejects_missing_and_invalid_controller_s():
    module = _load_supervisor_module()
    for invalid_s in ('missing', float('nan'), -0.1):
        supervisor = _fake_supervisor(module)
        state = supervisor.rails['right']['shuttles'][
            'room315_right_shuttle_1'
        ]
        if invalid_s == 'missing':
            state.pop('s')
        else:
            state['s'] = invalid_s

        decision = supervisor._safety_decode_command({
            'action': 'switches',
            'side': 'right',
            'switches': {'A3': 'INTERIOR'},
        })

        assert decision['accepted'] is False
        assert 'cannot prove controller safety distance' in decision['reason']


def test_safety_decoder_uses_mirrored_left_switch_geometry():
    module = _load_supervisor_module()
    right_distance, right_reason = module._shuttle_switch_distance_m(
        {'segment': 'A34E', 's': 0.2},
        'A3',
        side='right',
    )
    left_distance, left_reason = module._shuttle_switch_distance_m(
        {'segment': 'A12E', 's': 0.2},
        'A1',
        side='left',
    )
    assert right_reason == left_reason == ''
    assert abs(right_distance - left_distance) < 1e-12
    assert left_distance > module.SWITCH_CLEAR_DISTANCE_M

    supervisor = _fake_supervisor(module)
    supervisor.rails['left']['shuttles'] = {
        'room315_left_shuttle_1': {
            'mode': 'STOPPED',
            'segment': 'A12E',
            's': 0.2,
            'speed': 0.0,
        },
    }
    decision = supervisor._safety_decode_command({
        'action': 'switches',
        'side': 'left',
        'switches': {'A1': 'INTERIOR'},
    })

    assert decision['accepted'] is True


def test_safety_decoder_rejects_unidentified_active_switch_sensor():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['active_position_sensors'] = [{
        'name': 'DZI1R',
        'shuttle': '',
    }]

    decision = supervisor._safety_decode_command({
        'action': 'switches',
        'side': 'right',
        'switches': {'A1': 'INTERIOR'},
    })

    assert decision['accepted'] is False
    assert 'unknown shuttle identity' in decision['reason']


def test_safety_decoder_accepts_strict_mixed_route_normalization_proof():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    _stage_guarded_interior_shuttle(supervisor)
    supervisor.rails['right']['stoppers']['A4'] = '0'
    command = copy.deepcopy(_guarded_normalization_command())
    metadata = command['closed_loop_executive']
    metadata['mode'] = 'restore_normal_route_before_slot_motion'
    metadata['symbolic_step'] = (
        'restore_normal_route right right_staubli right_yaskawa'
    )
    proof = metadata['route_normalization_proof']
    proof.update({
        'stoppers': {
            'A1': 'open',
            'A2': 'open',
            'A3': 'open',
            'A4': 'open',
        },
        'clearance_mode': False,
        'all_stoppers_open': True,
        'reconfiguration_required': True,
        'reconfiguration_safe': True,
        'clearance_pause_safe': False,
    })

    decision = supervisor._safety_decode_command(command)

    assert decision['accepted'] is True
    assert decision['corrected_action']['closed_loop_executive']['mode'] == (
        'restore_normal_route_before_slot_motion'
    )


def _proved_cross_branch_clearance_command():
    command = copy.deepcopy(_guarded_normalization_command())
    command['switches'] = {
        'A1': 'EXTERIOR',
        'A2': 'INTERIOR',
        'A3': 'INTERIOR',
        'A4': 'INTERIOR',
    }
    metadata = command['closed_loop_executive']
    metadata.update({
        'mode': 'begin_route_clearance_hold_interior',
        'symbolic_step': (
            'begin_route_clearance right_shuttle_1 right '
            'right_slot_4 right_slot_2'
        ),
        'clearance_route_switch_proof': {
            'side': 'right',
            'target_segment': 'A34I',
            'gate_switch': 'A3',
            'exit_switch': 'A4',
            'required_switches': dict(command['switches']),
            'route_specific_switch_assignment': True,
            'controller_position_fields_used_for_localization': False,
        },
    })
    metadata['route_normalization_proof'].update({
        'switches': {name: 'exterior' for name in ('A1', 'A2', 'A3', 'A4')},
        'stoppers': {name: 'open' for name in ('A1', 'A2', 'A3', 'A4')},
        'normal_route': True,
        'clearance_mode': False,
        'all_stoppers_open': True,
        'reconfiguration_required': False,
        'reconfiguration_safe': False,
        'clearance_pause_safe': False,
    })
    return command


def test_safety_decoder_accepts_proved_cross_branch_clearance_switches():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    _stage_guarded_interior_shuttle(supervisor)
    supervisor.rails['right']['switches'] = {
        name: 'E' for name in ('A1', 'A2', 'A3', 'A4')
    }
    supervisor.rails['right']['stoppers'] = {
        name: '0' for name in ('A1', 'A2', 'A3', 'A4')
    }
    supervisor.rails['right']['active_sensors'] = []
    supervisor.rails['right']['active_position_sensors'] = []

    decision = supervisor._safety_decode_command(
        _proved_cross_branch_clearance_command()
    )

    assert decision['accepted'] is True
    assert decision['corrected_action']['switches'] == {
        'A1': 'EXTERIOR',
        'A2': 'INTERIOR',
        'A3': 'INTERIOR',
        'A4': 'INTERIOR',
    }


def test_safety_decoder_preserves_ordinary_clearance_with_no_interior_shuttle():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['shuttles'] = {}
    supervisor.rails['right']['switches'] = {
        name: 'E' for name in ('A1', 'A2', 'A3', 'A4')
    }
    supervisor.rails['right']['stoppers'] = {
        name: '0' for name in ('A1', 'A2', 'A3', 'A4')
    }
    supervisor.rails['right']['active_sensors'] = []
    supervisor.rails['right']['active_position_sensors'] = []
    command = _proved_cross_branch_clearance_command()
    command['switches'] = {
        'A1': 'INTERIOR',
        'A2': 'INTERIOR',
        'A3': 'EXTERIOR',
        'A4': 'EXTERIOR',
    }
    command['closed_loop_executive']['clearance_route_switch_proof'].update({
        'target_segment': 'A12I',
        'gate_switch': 'A1',
        'exit_switch': 'A2',
        'required_switches': dict(command['switches']),
    })
    command['closed_loop_executive']['route_normalization_proof'].update({
        'interior_shuttles': [],
        'visually_interior_shuttles': [],
        'certified_interior_shuttles': [],
        'certified_stopped_interior_shuttles': [],
        'uncertified_interior_shuttles': [],
        'clearance_lifecycle_certified_stopped_interior_shuttles': [],
        'certificate_segment_consistency': {},
    })
    command['closed_loop_executive']['runtime_clearance_certificates'] = {}

    decision = supervisor._safety_decode_command(command)

    assert decision['accepted'] is True


def test_safety_decoder_rejects_cross_branch_switches_that_differ_from_proof():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    _stage_guarded_interior_shuttle(supervisor)
    supervisor.rails['right']['switches'] = {
        name: 'E' for name in ('A1', 'A2', 'A3', 'A4')
    }
    supervisor.rails['right']['stoppers'] = {
        name: '0' for name in ('A1', 'A2', 'A3', 'A4')
    }
    command = _proved_cross_branch_clearance_command()
    command['switches']['A2'] = 'EXTERIOR'

    decision = supervisor._safety_decode_command(command)

    assert decision['accepted'] is False
    assert 'does not match the proved clearance route' in decision['reason']


def _mixed_route_visual_disagreement_command():
    command = copy.deepcopy(_guarded_normalization_command())
    metadata = command['closed_loop_executive']
    metadata['mode'] = 'restore_normal_route_before_slot_motion'
    metadata['symbolic_step'] = (
        'restore_normal_route right right_staubli right_yaskawa'
    )
    proof = metadata['route_normalization_proof']
    shuttle = 'right_shuttle_1'
    proof.update({
        'stoppers': {
            'A1': 'open',
            'A2': 'open',
            'A3': 'open',
            'A4': 'open',
        },
        'clearance_mode': False,
        'all_stoppers_open': True,
        'reconfiguration_required': True,
        'reconfiguration_safe': True,
        'clearance_pause_safe': False,
        'visually_interior_shuttles': [],
        'certified_stopped_interior_shuttles': [],
        'uncertified_interior_shuttles': [shuttle],
        'certificate_segment_mismatches': [shuttle],
        'clearance_lifecycle_visual_disagreements': [shuttle],
    })
    proof['certificate_segment_consistency'][shuttle].update({
        'satisfied': False,
        'accepted_visual_internal_segment': 'A34E',
        'certificate_used_as_persisted_execution_effect': True,
        'planning_origin_segment': 'A34I',
        'raw_visual_prediction_preserved': True,
        'reason': 'certificate_and_visual_segment_disagree',
    })
    return command


def test_mixed_route_normalization_accepts_proved_visual_disagreement():
    """Replay the live post-topology-setup normalization rejection."""

    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    _stage_guarded_interior_shuttle(supervisor)
    supervisor.rails['right']['stoppers']['A4'] = '0'

    decision = supervisor._safety_decode_command(
        _mixed_route_visual_disagreement_command()
    )

    assert decision['accepted'] is True
    assert decision['corrected_action']['switches'] == {
        'A1': 'EXTERIOR',
        'A2': 'EXTERIOR',
        'A3': 'EXTERIOR',
        'A4': 'EXTERIOR',
    }


def test_mixed_route_visual_disagreement_requires_persisted_effect_proof():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    _stage_guarded_interior_shuttle(supervisor)
    supervisor.rails['right']['stoppers']['A4'] = '0'
    command = _mixed_route_visual_disagreement_command()
    consistency = command['closed_loop_executive'][
        'route_normalization_proof'
    ]['certificate_segment_consistency']['right_shuttle_1']
    consistency.pop('certificate_used_as_persisted_execution_effect')

    decision = supervisor._safety_decode_command(command)

    assert decision['accepted'] is False
    assert 'visual/certificate segment proof is invalid' in decision['reason']


def test_safety_decoder_accepts_strict_capacity_pause_proof():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    _stage_guarded_interior_shuttle(supervisor)
    command = copy.deepcopy(_guarded_normalization_command())
    metadata = command['closed_loop_executive']
    metadata['mode'] = 'pause_clearance_after_interior_capacity_exhausted'
    metadata['symbolic_step'] = 'pause_route_clearance right'

    decision = supervisor._safety_decode_command(command)

    assert decision['accepted'] is True


def test_safety_decoder_rejects_forged_guarded_normalization_certificate():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    _stage_guarded_interior_shuttle(supervisor)
    command = copy.deepcopy(_guarded_normalization_command())
    certificate = command['closed_loop_executive'][
        'runtime_clearance_certificates'
    ]['R1']
    certificate['identity'] = 'R2'
    certificate['controller_position_fields_used_for_localization'] = True

    decision = supervisor._safety_decode_command(command)

    assert decision['accepted'] is False
    assert 'runtime clearance certificate is invalid' in decision['reason']
    assert decision['corrected_action'] is None


def test_safety_decoder_rejects_controller_position_as_normalization_localization():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    _stage_guarded_interior_shuttle(supervisor)
    command = copy.deepcopy(_guarded_normalization_command())
    command['closed_loop_executive'][
        'controller_position_fields_used_for_localization'
    ] = True

    decision = supervisor._safety_decode_command(command)

    assert decision['accepted'] is False
    assert 'accepted-visual localization provenance' in decision['reason']
    assert decision['corrected_action'] is None


def test_safety_decoder_rejects_unbound_active_guard_sensor_with_valid_proof():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    _stage_guarded_interior_shuttle(supervisor)
    supervisor.rails['right']['active_position_sensors'] = [{
        'name': 'DA4IR',
        'shuttle': 'room315_right_shuttle_2',
    }]

    decision = supervisor._safety_decode_command(
        _guarded_normalization_command()
    )

    assert decision['accepted'] is False
    assert 'active A4 guard sensor is not identity-bound' in decision['reason']
    assert decision['corrected_action'] is None


def test_safety_decoder_rejects_partial_switch_change_despite_valid_proof():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    _stage_guarded_interior_shuttle(supervisor)
    command = _guarded_normalization_command()
    command['switches'] = {'A4': 'EXTERIOR'}

    decision = supervisor._safety_decode_command(command)

    assert decision['accepted'] is False
    assert 'permits only all-switch EXTERIOR normalization' in decision['reason']


def test_safety_decoder_rejects_unsafe_stopper_close_near_moving_shuttle():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['shuttles']['room315_right_shuttle_1'] = {
        'mode': 'MOVING',
        'segment': 'A34E',
        's': 2.1,
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


def _interior_cross_branch_motion_command():
    return {
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'room315_right_shuttle_1',
        'command': 'ON',
        'target_stopper': 'A2',
        'closed_loop_executive': {
            'mode': 'plansys2_supervised_interior_clearance',
            'problem_name': 'live-loaded-a34i-to-a12i',
            'localization_source': 'accepted_visual_state',
            'controller_position_fields_used_for_localization': False,
            'clearance_motion_route_proof': {
                'side': 'right',
                'target_segment': 'A12I',
                'gate_switch': 'A1',
                'exit_switch': 'A2',
                'required_switches': {
                    'A1': 'INTERIOR',
                    'A2': 'INTERIOR',
                    'A3': 'EXTERIOR',
                    'A4': 'INTERIOR',
                },
                'required_stoppers': {
                    'A1': '0', 'A2': '1', 'A3': '0', 'A4': '0',
                },
                'route_specific_switch_assignment': True,
                'controller_position_fields_used_for_localization': False,
            },
        },
    }


def test_safety_decoder_rejects_live_falling_cross_branch_assignment():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['shuttles'][
        'room315_right_shuttle_1'
    ].update({'mode': 'DISABLED', 'segment': 'A34I', 's': 0.369})
    supervisor.rails['right']['active_position_sensors'] = []
    supervisor.rails['right']['switches'] = {
        'A1': 'I', 'A2': 'I', 'A3': 'E', 'A4': 'E',
    }
    supervisor.rails['right']['stoppers']['A2'] = '1'

    decision = supervisor._safety_decode_command(
        _interior_cross_branch_motion_command()
    )

    assert decision['accepted'] is False
    assert 'required switch assignment is not active' in decision['reason']
    assert "'A4': 'INTERIOR'" in decision['reason']
    assert "'A4': 'EXTERIOR'" in decision['reason']


def test_safety_decoder_accepts_cross_branch_motion_only_after_full_route_setup():
    module = _load_supervisor_module()
    supervisor = _fake_supervisor(module)
    supervisor.rails['right']['shuttles'][
        'room315_right_shuttle_1'
    ].update({'mode': 'DISABLED', 'segment': 'A34I', 's': 0.369})
    supervisor.rails['right']['active_position_sensors'] = []
    supervisor.rails['right']['switches'] = {
        'A1': 'I', 'A2': 'I', 'A3': 'E', 'A4': 'I',
    }
    supervisor.rails['right']['stoppers']['A2'] = '1'

    decision = supervisor._safety_decode_command(
        _interior_cross_branch_motion_command()
    )

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


def test_typed_off_command_does_not_mislabel_retained_speed_as_stop_speed():
    module = _load_supervisor_module()

    assert module._shuttle_command_speed('ON', None, 0.2) == 0.2
    assert module._shuttle_command_speed('ON', 0.35, 0.2) == 0.35
    assert module._shuttle_command_speed('OFF', 0.2, 0.2) == 0.0
    assert module._shuttle_command_speed('RESET', 0.2, 0.2) == 0.0


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
