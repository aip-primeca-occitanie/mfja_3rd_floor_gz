#!/usr/bin/env python3

import importlib.util
from pathlib import Path


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


def _action_value(module, action_vector, field):
    return action_vector[module.ACTION_VECTOR_FIELDS.index(field)]


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
    assert _action_value(translator, translated.action_vector, 'primitive_id') == 2.0
    assert _action_value(translator, translated.action_vector, 'switch_mask_A3') == 1.0
    assert _action_value(translator, translated.action_vector, 'switch_value_A3') == 1.0


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
    assert _action_value(translator, translated.action_vector, 'primitive_id') == 3.0
    assert _action_value(translator, translated.action_vector, 'stopper_mask_A4') == 1.0
    assert _action_value(translator, translated.action_vector, 'stopper_value_A4') == 1.0


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
    assert _action_value(translator, translated.action_vector, 'primitive_id') == 4.0
    assert _action_value(translator, translated.action_vector, 'speed_mps') == 0.3


def test_move_shuttle_with_payload_selected_identity_maps_to_r2_schema_fields():
    translator = _load_module()

    translated = translator.translate_step(
        'move_shuttle right right_shuttle_2 yaskawa staubli speed=0.3'
    )

    assert translated.command['shuttle'] == 'right_shuttle_2'
    assert translated.event_action['target_id'] == 'right_shuttle_2'
    assert translated.event_action['shuttle_id'] == 'R2'
    assert translated.event_action['shuttle_index'] == 1
    assert _action_value(translator, translated.action_vector, 'shuttle_index') == 1.0


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
    assert _action_value(translator, translated.action_vector, 'primitive_id') == 5.0


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
    assert _action_value(translator, translated.action_vector, 'primitive_id') == 1.0


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
    assert [row.action_vector[0] for row in translated] == [
        translator.PRIMITIVE_IDS['SET_SWITCHES'],
        translator.PRIMITIVE_IDS['SET_STOPPERS'],
        translator.PRIMITIVE_IDS['SHUTTLE_ON'],
        translator.PRIMITIVE_IDS['STOP_NOW'],
        translator.PRIMITIVE_IDS['DONE'],
    ]


def test_translator_accepts_domain_order_for_move_and_stop_steps():
    translator = _load_module()

    move = translator.translate_step('(move_shuttle right_shuttle right yaskawa staubli)')
    stop = translator.translate_step('(stop_shuttle right_shuttle right yaskawa staubli)')

    assert move.command['side'] == 'right'
    assert move.command['shuttle'] == 'right_shuttle'
    assert stop.command['side'] == 'right'
    assert stop.command['shuttle'] == 'right_shuttle'
