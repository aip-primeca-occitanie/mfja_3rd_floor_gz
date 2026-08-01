#!/usr/bin/env python3

import importlib.util
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'scripts'
    / 'room_315_pddl_plan_translator.py'
)


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_pddl_plan_translator', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_prepare_switches_translates_to_set_switches_compatible_command():
    translator = _load_module()

    translated = translator.translate_step('prepare_switches right yaskawa staubli')

    assert translated.command == {
        'action': 'switches',
        'side': 'right',
        'switches': {'ALL': 'EXTERIOR'},
    }
    assert translated.event_action['primitive'] == 'SET_SWITCHES'
    assert translated.event_action['side'] == 'right'
    assert translated.event_action['switch_mask'] == {
        'A1': 1,
        'A2': 1,
        'A3': 1,
        'A4': 1,
    }
    assert translated.event_action['switch_values']['A3'] == 'EXTERIOR'
    assert translated.event_action['target_id'] == 'ALL_SWITCHES'


def test_prepare_switches_can_target_one_switch():
    translator = _load_module()

    translated = translator.translate_step(
        'prepare_switches right yaskawa yaskawa switch=A3 state=INTERIOR'
    )

    assert translated.command == {
        'action': 'switches',
        'side': 'right',
        'switches': {'A3': 'INTERIOR'},
    }
    assert translated.event_action['target_id'] == 'A3'
    assert translated.event_action['switch_mask'] == {
        'A1': 0,
        'A2': 0,
        'A3': 1,
        'A4': 0,
    }
    assert translated.event_action['switch_values']['A1'] == 'UNCHANGED'
    assert translated.event_action['switch_values']['A3'] == 'INTERIOR'


def test_open_stoppers_translates_to_set_stoppers_compatible_command():
    translator = _load_module()

    translated = translator.translate_step('open_stoppers right')

    assert translated.command == {
        'action': 'stoppers',
        'side': 'right',
        'stoppers': {'ALL': '0'},
    }
    assert translated.event_action['primitive'] == 'SET_STOPPERS'
    assert translated.event_action['stopper_mask'] == {
        'A1': 1,
        'A2': 1,
        'A3': 1,
        'A4': 1,
    }
    assert translated.event_action['stopper_values']['A4'] == 'open'
    assert translated.event_action['target_id'] == 'ALL_STOPPERS'


def test_set_stoppers_can_close_single_stopper_and_open_the_rest():
    translator = _load_module()

    translated = translator.translate_step('set_stoppers right A4 closed')

    assert translated.command == {
        'action': 'stoppers',
        'side': 'right',
        'stoppers': {'ALL': '0', 'A4': '1'},
    }
    assert translated.event_action['primitive'] == 'SET_STOPPERS'
    assert translated.event_action['target_id'] == 'A4'
    assert translated.event_action['stopper_mask'] == {
        'A1': 1,
        'A2': 1,
        'A3': 1,
        'A4': 1,
    }
    assert translated.event_action['stopper_values'] == {
        'A1': 'open',
        'A2': 'open',
        'A3': 'open',
        'A4': 'closed',
    }


def test_move_shuttle_translates_to_shuttle_on_with_speed():
    translator = _load_module()

    translated = translator.translate_step(
        'move_shuttle right right_shuttle yaskawa staubli speed=0.3'
    )

    assert translated.command == {
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'right_shuttle',
        'command': 'ON',
        'speed': 0.3,
    }
    assert translated.event_action['primitive'] == 'SHUTTLE_ON'
    assert translated.event_action['speed_mps'] == 0.3
    assert translated.event_action['wait_condition'] == 'shuttle_command_applied'
    assert translated.event_action['target_id'] == 'right_shuttle_1'
    assert translated.event_action['shuttle_id'] == 'R1'
    assert translated.event_action['shuttle_index'] == 0
    assert translated.event_action['reason'] == 'shuttle_start'


def test_move_shuttle_with_payload_selected_identity_maps_to_r2_schema_fields():
    translator = _load_module()

    translated = translator.translate_step(
        'move_shuttle right right_shuttle_2 yaskawa staubli speed=0.3'
    )

    assert translated.command['shuttle'] == 'right_shuttle_2'
    assert translated.event_action['target_id'] == 'right_shuttle_2'
    assert translated.event_action['shuttle_id'] == 'R2'
    assert translated.event_action['shuttle_index'] == 1


def test_move_shuttle_preserves_target_stopper_for_guarded_stopper_motion():
    translator = _load_module()

    translated = translator.translate_step(
        'move_shuttle right right_shuttle_2 staubli staubli speed=0.3 target_stopper=A4'
    )

    assert translated.command == {
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'right_shuttle_2',
        'command': 'ON',
        'speed': 0.3,
        'target_stopper': 'A4',
    }
    assert translated.event_action['primitive'] == 'SHUTTLE_ON'
    assert translated.event_action['target_id'] == 'right_shuttle_2'


def test_prepare_topology_route_preserves_audited_route_endpoints():
    translator = _load_module()

    translated = translator.translate_step(
        '(prepare_topology_route right_shuttle_2 right '
        'right_topology_a34i right_slot_1 right_switch_group)'
    )

    assert translator.SYMBOLIC_ACTION_PRIMITIVE_MAP[
        'prepare_topology_route'
    ] == 'SET_SWITCHES'
    assert translated.command == {
        'action': 'topology_route',
        'side': 'right',
        'shuttle': 'right_shuttle_2',
        'source_block': 'right_topology_a34i',
        'target_slot': 'right_slot_1',
        'deterministic_macro': (
            'authoritative_topology_switches_and_open_stoppers'
        ),
    }
    assert translated.event_action['primitive'] == 'SET_SWITCHES'
    assert translated.event_action['wait_condition'] == 'switch_state_match'
    assert translated.event_action['target_id'] == 'ALL_SWITCHES'
    assert translated.event_action['coordination_mode'] == (
        'reservation_based_move'
    )
    assert translated.event_action['shuttle_id'] == 'R2'
    assert translated.event_action['shuttle_index'] == 1


def test_segment_origin_move_preserves_block_station_slot_and_speed():
    translator = _load_module()

    translated = translator.translate_step(
        '(move_shuttle_from_segment_to_slot right_shuttle_2 right '
        'right_topology_a34i right_yaskawa right_slot_1 speed=0.2)'
    )

    assert translator.SYMBOLIC_ACTION_PRIMITIVE_MAP[
        'move_shuttle_from_segment_to_slot'
    ] == 'SHUTTLE_ON'
    assert translated.command == {
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'right_shuttle_2',
        'command': 'ON',
        'speed': 0.2,
        'source_block': 'right_topology_a34i',
        'target_station': 'right_yaskawa',
        'target_slot': 'right_slot_1',
        'topology_route_move': True,
    }
    assert translated.event_action['primitive'] == 'SHUTTLE_ON'
    assert translated.event_action['wait_condition'] == 'target_sensor_active'
    assert translated.event_action['target_id'] == 'right_shuttle_2'
    assert translated.event_action['coordination_mode'] == (
        'reservation_based_move'
    )
    assert translated.event_action['shuttle_id'] == 'R2'
    assert translated.event_action['shuttle_index'] == 1
    assert translated.event_action['speed_mps'] == 0.2


@pytest.mark.parametrize(
    'step',
    [
        'prepare_topology_route right_shuttle_2 right right_topology_a34i',
        (
            'move_shuttle_from_segment_to_slot right_shuttle_2 right '
            'right_topology_a34i right_yaskawa'
        ),
    ],
)
def test_topology_actions_reject_missing_route_endpoints(step):
    translator = _load_module()

    with pytest.raises(ValueError, match='requires'):
        translator.translate_step(step)


def test_stop_shuttle_translates_to_stop_now():
    translator = _load_module()

    translated = translator.translate_step('stop_shuttle right right_shuttle')

    assert translated.command == {
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'right_shuttle',
        'command': 'OFF',
    }
    assert translated.event_action['primitive'] == 'STOP_NOW'
    assert translated.event_action['target_id'] == 'right_shuttle_1'
    assert translated.event_action['shuttle_id'] == 'R1'
    assert translated.event_action['shuttle_index'] == 0
    assert translated.event_action['reason'] == 'shuttle_stop'


def test_finish_task_translates_to_done():
    translator = _load_module()

    translated = translator.translate_step('finish_task right_shuttle staubli')

    assert translated.command == {
        'action': 'DONE',
        'status': 'success',
        'shuttle': 'right_shuttle',
        'station': 'staubli',
    }
    assert translated.event_action['primitive'] == 'DONE'
    assert translated.event_action['wait_condition'] == 'terminal'
    assert translated.event_action['target_id'] == 'terminal'
    assert translated.event_action['reason'] == 'task_succeeded'


def test_full_high_level_plan_translates_to_ordered_event_sequence():
    translator = _load_module()
    plan = [
        '(prepare_switches right yaskawa staubli)',
        '(open_stoppers right yaskawa staubli)',
        '(move_shuttle right right_shuttle yaskawa staubli speed=0.3)',
        '(stop_shuttle right right_shuttle)',
        '(finish_task right_shuttle staubli)',
    ]

    translated = translator.translate_plan(plan)

    assert [row.command['action'] for row in translated] == [
        'switches',
        'stoppers',
        'shuttle',
        'shuttle',
        'DONE',
    ]
    assert [row.event_action['primitive'] for row in translated] == [
        'SET_SWITCHES',
        'SET_STOPPERS',
        'SHUTTLE_ON',
        'STOP_NOW',
        'DONE',
    ]


def test_translator_accepts_domain_order_for_move_and_stop_steps():
    translator = _load_module()

    move = translator.translate_step('(move_shuttle right_shuttle right yaskawa staubli)')
    stop = translator.translate_step('(stop_shuttle right_shuttle right yaskawa staubli)')

    assert move.command['side'] == 'right'
    assert move.command['shuttle'] == 'right_shuttle'
    assert stop.command['side'] == 'right'
    assert stop.command['shuttle'] == 'right_shuttle'


def test_costed_slot_move_and_candidate_finish_have_primitive_mappings():
    translator = _load_module()

    move = translator.translate_step(
        '(move_shuttle_to_slot right_shuttle_2 right right_yaskawa '
        'right_staubli right_slot_2 right_slot_3)'
    )
    finish = translator.translate_step(
        '(finish_candidate_task right_shuttle_2 right_staubli right_slot_3)'
    )

    assert translator.SYMBOLIC_ACTION_PRIMITIVE_MAP['move_shuttle_to_slot'] == 'SHUTTLE_ON'
    assert move.command['action'] == 'shuttle'
    assert move.event_action['primitive'] == 'SHUTTLE_ON'
    assert finish.command['action'] == 'DONE'
    assert finish.event_action['primitive'] == 'DONE'


def test_wait_for_clearance_is_safe_stop_macro():
    translator = _load_module()

    translated = translator.translate_step(
        '(wait_for_clearance right_shuttle_2 right_slot_3)'
    )

    assert translated.command['action'] == 'shuttle'
    assert translated.command['command'] == 'OFF'
    assert translated.command['deterministic_macro'] == 'wait_for_clearance'
    assert translated.event_action['primitive'] == 'STOP_NOW'
    assert translated.event_action['wait_condition'] == 'block_clearance'
    assert translated.event_action['reason'] == 'wait_for_block_clearance'


def test_interior_blocker_relocation_translates_to_executive_guarded_macro():
    translator = _load_module()

    translated = translator.translate_step(
        '(relocate_blocker_to_interior right_shuttle_2 right_shuttle_4 '
        'right right_slot_4 right_slot_2)'
    )

    assert translated.command == {
        'action': 'clearance_relocation',
        'side': 'right',
        'shuttle': 'right_shuttle_2',
        'selected_shuttle': 'right_shuttle_4',
        'command': 'ON',
        'speed': 0.3,
        'deterministic_macro': 'supervised_a3_interior_visual_stop',
    }
    assert translated.event_action['primitive'] == 'SHUTTLE_ON'
    assert translated.event_action['wait_condition'] == (
        'accepted_visual_interior_pose_then_controller_stop'
    )


def test_clearance_phase_boundaries_translate_without_per_blocker_restore():
    translator = _load_module()

    begin = translator.translate_step(
        '(begin_route_clearance right_shuttle_4 right '
        'right_slot_4 right_slot_2)'
    )
    finish = translator.translate_step(
        '(finish_route_clearance right_shuttle_4 right '
        'right_slot_4 right_slot_2)'
    )

    assert begin.command['switches'] == {
        'A1': 'EXTERIOR',
        'A2': 'EXTERIOR',
        'A3': 'INTERIOR',
        'A4': 'INTERIOR',
    }
    assert begin.command['deterministic_macro'] == (
        'hold_interior_route_for_all_blockers'
    )
    assert finish.command['switches'] == {'ALL': 'EXTERIOR'}
    assert finish.command['deterministic_macro'] == 'restore_normal_route_once'
