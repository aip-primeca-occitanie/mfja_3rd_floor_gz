#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'scripts'
    / 'room_315_pddl_language_generator.py'
)


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_pddl_language_generator', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_same_seed_gives_same_language():
    generator = _load_module()

    first = generator.generate_language(
        pddl_goal='right_shuttle at staubli',
        seed=7,
    )
    second = generator.generate_language(
        pddl_goal='right_shuttle at staubli',
        seed=7,
    )

    assert first.language == second.language
    assert first.template_id == second.template_id


def test_different_template_ids_produce_different_paraphrases():
    generator = _load_module()

    move = generator.generate_language(
        pddl_goal='left_shuttle at kuka',
        template_id='move_from_to',
    )
    send = generator.generate_language(
        pddl_goal='left_shuttle at kuka',
        template_id='send_to_station',
    )

    assert move.language == 'move the left shuttle from Yaskawa to KUKA'
    assert send.language == 'send the left shuttle to the KUKA station'
    assert move.language != send.language
    assert len(generator.generate_language_variants(pddl_goal='left_shuttle at kuka')) >= 2


def test_generated_language_excludes_raw_pddl_syntax_by_default():
    generator = _load_module()

    generated = generator.generate_language(
        pddl_goal='(:goal (and (task_done right_shuttle right_staubli)))',
        template_id='send_to_station',
    )
    debug = generator.generate_language(
        pddl_goal='(:goal (and (task_done right_shuttle right_staubli)))',
        template_id='send_to_station',
        include_raw_pddl=True,
    )

    assert generated.language == 'send the right shuttle to the Staubli station'
    forbidden = ('(', ')', ':goal', 'task_done', 'right_shuttle', 'right_staubli')
    for token in forbidden:
        assert token not in generated.language
    assert 'task_done' in debug.language


def test_language_can_be_attached_to_planned_scenario_metadata_object():
    generator = _load_module()
    generated = generator.generate_language(
        pddl_goal='right_shuttle at staubli',
        pddl_problem='room315-right-yaskawa-to-staubli',
        symbolic_plan=[
            'prepare_switches right yaskawa staubli',
            'open_stoppers right',
            'move_shuttle right right_shuttle yaskawa staubli speed=0.3',
            'stop_shuttle right right_shuttle',
            'finish_task right_shuttle staubli',
        ],
        template_id='move_from_to',
    )
    scenario = {
        'scenario_id': 'planned_right_transport',
        'model_input': {
            'overhead_images': {},
            'last_command': {'action': 'START'},
            'observable_state': {},
        },
    }

    updated = generator.attach_language_to_scenario_metadata(scenario, generated)

    assert updated['model_input']['language'] == 'move the right shuttle from Yaskawa to Staubli'
    assert updated['pddl_goal'] == 'right_shuttle at staubli'
    assert updated['pddl_problem'] == 'room315-right-yaskawa-to-staubli'
    assert updated['generated_language_template_id'] == 'move_from_to'
    assert updated['symbolic_plan'][2].startswith('move_shuttle right')
    assert 'pddl_goal' not in updated['model_input']
    assert 'pddl_problem' not in updated['model_input']
    assert 'generated_language_template_id' not in updated['model_input']
    assert 'symbolic_plan' not in updated['model_input']
    assert set(updated['model_input']) == {
        'language',
        'overhead_images',
        'last_command',
        'observable_state',
    }


def test_action_sequence_can_generate_future_slot_sequence_language():
    generator = _load_module()

    generated = generator.generate_language(
        action_sequence='send left shuttle to slot 3 and then to slot 2',
        template_id='send_slot_sequence',
    )

    assert generated.language == 'send the left shuttle to slot 3 and then to slot 2'
    assert 'slot 3' in generated.language
    assert 'slot 2' in generated.language
    assert 'pddl' not in json.dumps({'language': generated.language}).casefold()


def test_loaded_slot_language_stays_in_task_language_boundary():
    generator = _load_module()

    generated = generator.generate_language(
        action_sequence='move the loaded right shuttle to slot 3',
        template_id='loaded_shuttle_to_slot',
    )

    assert generated.language == 'move the loaded right shuttle to slot 3'
    assert generated.metadata['payload_condition'] == 'loaded'
    assert generated.metadata['target_slot'] == '3'
    assert 'pddl' not in json.dumps({'language': generated.language}).casefold()


def test_identity_aware_language_templates_use_visual_shuttle_labels():
    generator = _load_module()

    explicit = generator.generate_language(
        pddl_goal='right_shuttle_2 at staubli',
        template_id='explicit_id_to_station',
    )
    labeled = generator.generate_language(
        action_sequence='send the shuttle labeled L3 to KUKA',
        template_id='labeled_id_to_station',
    )

    assert explicit.language == 'move R2 to the Staubli station'
    assert labeled.language == 'move the shuttle labeled L3 to KUKA'
    assert 'right_shuttle_2' not in explicit.language
    assert 'pddl' not in explicit.language.casefold()


def test_relational_language_remains_task_language_only():
    generator = _load_module()

    generated = generator.generate_language(
        action_sequence='move R4 to staubli',
        template_id='loaded_id_to_station',
    )

    assert generated.language == 'move R4 to Staubli even though it is carrying a part'
    assert generated.metadata['generated_language_template_id'] == 'loaded_id_to_station'
    assert 'symbolic_plan' not in generated.language


def test_payload_language_templates_parse_loaded_and_empty_tasks():
    generator = _load_module()

    loaded = generator.generate_language(
        action_sequence='move R2 carrying a part to Staubli',
        template_id='carrying_part_id_to_station',
    )
    empty = generator.generate_language(
        action_sequence='move the empty shuttle to Yaskawa',
        template_id='empty_shuttle_to_station',
    )

    assert loaded.language == 'move R2 carrying a part to Staubli'
    assert loaded.metadata['payload_condition'] == 'loaded'
    assert empty.language == 'move the empty right shuttle to Yaskawa'
    assert empty.metadata['payload_condition'] == 'empty'


def test_payload_pddl_goal_generates_loaded_shuttle_language():
    generator = _load_module()

    generated = generator.generate_language(
        pddl_goal='(:goal (and (loaded right_shuttle_2) (task_done right_shuttle_2 right_staubli)))',
        template_id='loaded_shuttle_to_station',
    )

    assert generated.language == 'move the loaded right shuttle to Staubli'
    assert generated.metadata['payload_condition'] == 'loaded'


def test_payload_condition_from_pddl_goal_enriches_symbolic_plan_language():
    generator = _load_module()

    generated = generator.generate_language(
        pddl_goal='(:goal (and (loaded right_shuttle_2) (task_done right_shuttle_2 right_staubli)))',
        symbolic_plan=['move_shuttle right right_shuttle_2 yaskawa staubli speed=0.3'],
        template_id='loaded_shuttle_to_station',
    )

    assert generated.language == 'move the loaded right shuttle to Staubli'
    assert generated.metadata['payload_condition'] == 'loaded'
