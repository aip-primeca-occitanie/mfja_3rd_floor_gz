#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PDDL_DOMAIN = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'config'
    / 'room_315_vla'
    / 'pddl'
    / 'domain_room315.pddl'
)
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
TRANSLATOR_PATH = SCRIPT_DIR / 'room_315_pddl_plan_translator.py'


def _load_translator():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    spec = importlib.util.spec_from_file_location('room_315_pddl_plan_translator', TRANSLATOR_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_multi_shuttle_pddl_domain_contains_fleet_predicates_and_actions():
    text = PDDL_DOMAIN.read_text(encoding='utf-8')

    for required in (
        'shuttle_on_side',
        'shuttle_at_slot',
        'slot_reserved_by',
        'block_free',
        'block_reserved_by',
        'block_occupied_by',
        'move_shuttle_to_slot',
        'finish_candidate_task',
        'route_cost',
        'inspection_done',
        'front_of',
        'waiting_for_clearance',
        'assign_task',
        'prepare_switches_for_shuttle',
        'open_stoppers_for_shuttle',
        'stop_shuttle_at_slot',
        'wait_for_clearance',
    ):
        assert required in text
    for removed in (
        'shuttle_at_block',
        'reserve_next_block',
        'release_block',
        'move_shuttle_to_block',
        'carrying_payload',
        'payload_on_shuttle',
        'transfer_payload_if_applicable',
    ):
        assert removed not in text


def test_multi_shuttle_plan_step_translates_to_targeted_primitive_command():
    translator = _load_translator()

    translated = translator.translate_step(
        'move_shuttle right right_shuttle_2 yaskawa staubli speed=0.31'
    )

    assert translated.command['shuttle'] == 'right_shuttle_2'
    assert translated.event_action['shuttle_id'] == 'R2'
    assert translated.event_action['shuttle_index'] == 1
    assert translated.event_action['target_id'] == 'right_shuttle_2'
