#!/usr/bin/env python3

import re
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PDDL_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'config' / 'room_315_vla' / 'pddl'
DOC_PATH = REPO_ROOT / 'docs' / 'ROOM315_PDDL_PLANNING.md'

EXPECTED_PDDL_FILES = {
    'domain_room315.pddl',
}

EXPECTED_ACTIONS = {
    'prepare_switches',
    'open_stoppers',
    'move_shuttle_to_slot',
    'stop_shuttle',
    'finish_task',
}

RUNTIME_DOMAIN_PATH = PDDL_DIR / 'domain_room315_runtime.pddl'
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_pddl_plan_translator import SYMBOLIC_ACTION_PRIMITIVE_MAP
from room_315_pddl_scenario_generator import RUNTIME_SYMBOLIC_ACTIONS


def _without_comments(text: str) -> str:
    return '\n'.join(line.split(';', 1)[0] for line in text.splitlines())


def _balanced_parentheses(text: str) -> bool:
    depth = 0
    for char in _without_comments(text):
        if char == '(':
            depth += 1
        elif char == ')':
            depth -= 1
            if depth < 0:
                return False
    return depth == 0


def test_room315_pddl_files_exist():
    for filename in EXPECTED_PDDL_FILES:
        path = PDDL_DIR / filename
        assert path.exists(), f'missing {path}'
        assert path.read_text(encoding='utf-8').strip()

    assert DOC_PATH.exists()
    assert DOC_PATH.read_text(encoding='utf-8').strip()


def test_room315_pddl_domain_contains_expected_actions():
    text = (PDDL_DIR / 'domain_room315.pddl').read_text(encoding='utf-8').casefold()

    assert _balanced_parentheses(text)
    for action in EXPECTED_ACTIONS:
        assert re.search(rf'\(:action\s+{re.escape(action)}\b', text), action


def test_room315_pddl_declares_negative_precondition_requirement():
    text = (PDDL_DIR / 'domain_room315.pddl').read_text(encoding='utf-8').casefold()

    assert '(not ' in text
    assert re.search(r'\(:requirements[^)]*:negative-preconditions', text)


def test_room315_pddl_finish_requires_stopped_shuttle():
    text = _without_comments(
        (PDDL_DIR / 'domain_room315.pddl').read_text(encoding='utf-8').casefold()
    )

    assert '(shuttle_stopped_at ?s - shuttle ?station - station)' in text
    finish_body = text.split('(:action finish_task', 1)[1].split('(:action assign_task', 1)[0]
    assert '(shuttle_at ?s ?station)' in finish_body
    assert '(shuttle_stopped_at ?s ?station)' in finish_body


def test_room315_static_pddl_problems_are_not_committed():
    assert sorted(PDDL_DIR.glob('problem_' + '*.pddl')) == []


def test_runtime_slot_goal_cannot_finish_before_exact_slot_is_reached():
    text = _without_comments(RUNTIME_DOMAIN_PATH.read_text(encoding='utf-8').casefold())

    assert _balanced_parentheses(text)
    finish_body = text.split('(:action finish_task', 1)[1].split(
        '(:action finish_candidate_task', 1
    )[0]
    assert '(station_only_goal)' in finish_body
    assert '(target_station_for_goal ?station)' in finish_body

    slot_finish_body = text.split('(:action finish_candidate_task', 1)[1]
    assert '(target_slot_for_goal ?slot)' in slot_finish_body
    assert '(shuttle_at_slot ?s ?slot)' in slot_finish_body
    assert '(shuttle_stopped_at ?s ?station)' in slot_finish_body


def test_runtime_domain_supports_non_actuating_inspection():
    text = _without_comments(RUNTIME_DOMAIN_PATH.read_text(encoding='utf-8').casefold())

    assert _balanced_parentheses(text)
    inspection_body = text.split('(:action inspect_state', 1)[1]
    assert '?target - inspectable' in inspection_body
    assert '(validated_state)' in inspection_body
    assert '(inspection_required ?target)' in inspection_body
    assert '(inspection_done ?target)' in inspection_body
    assert '(shuttle_at ' not in inspection_body
    assert '(switches_ready ' not in inspection_body
    assert '(stoppers_open ' not in inspection_body


def test_both_domains_define_slot_origin_topology_route_with_exact_occupancy_transfer():
    for domain_path in (
        PDDL_DIR / 'domain_room315.pddl',
        RUNTIME_DOMAIN_PATH,
    ):
        text = _without_comments(domain_path.read_text(encoding='utf-8').casefold())
        assert _balanced_parentheses(text)

        prepare = text.split('(:action prepare_slot_topology_route', 1)[1].split(
            '(:action move_shuttle_via_topology_to_slot', 1
        )[0]
        assert '(shuttle_at_slot ?s ?from_slot)' in prepare
        assert '(slot_occupied_by ?from_slot ?s)' in prepare
        assert '(shuttle_at_topology_block ?s ?from_block)' in prepare
        assert '(topology_route_available ?s ?from_block ?to_slot)' in prepare
        assert '(topology_route_clear ?s ?from_block ?to_slot)' in prepare
        assert '(topology_route_configured ?s ?from_block ?to_slot)' in prepare

        move = text.split('(:action move_shuttle_via_topology_to_slot', 1)[1].split(
            '(:action begin_route_clearance', 1
        )[0]
        assert '(slot_at_station ?to_slot ?to)' in move
        assert '(not (shuttle_at_slot ?s ?from_slot))' in move
        assert '(not (slot_occupied_by ?from_slot ?s))' in move
        assert '(slot_free ?from_slot)' in move
        assert '(not (slot_free ?to_slot))' in move
        assert '(slot_occupied_by ?to_slot ?s)' in move
        assert '(shuttle_at_slot ?s ?to_slot)' in move
        assert '(shuttle_at ?s ?to)' in move
        assert '(shuttle_stopped_at ?s ?to)' in move


def test_segment_origin_actions_cannot_consume_exact_slot_locations():
    for domain_path in (
        PDDL_DIR / 'domain_room315.pddl',
        RUNTIME_DOMAIN_PATH,
    ):
        text = _without_comments(domain_path.read_text(encoding='utf-8').casefold())
        prepare = text.split('(:action prepare_topology_route', 1)[1].split(
            '(:action move_shuttle_from_segment_to_slot', 1
        )[0]
        move = text.split('(:action move_shuttle_from_segment_to_slot', 1)[1].split(
            '(:action prepare_slot_topology_route', 1
        )[0]

        assert '(segment_only_location ?s)' in prepare
        assert '(segment_only_location ?s)' in move


def test_runtime_domain_action_surface_matches_declared_runtime_contract():
    text = _without_comments(
        RUNTIME_DOMAIN_PATH.read_text(encoding='utf-8').casefold()
    )
    domain_actions = frozenset(re.findall(r'\(:action\s+([a-z0-9_]+)', text))

    assert domain_actions == RUNTIME_SYMBOLIC_ACTIONS
    assert domain_actions <= SYMBOLIC_ACTION_PRIMITIVE_MAP.keys()


def test_expert_and_runtime_domains_share_one_executable_action_surface():
    expert = _without_comments(
        (PDDL_DIR / 'domain_room315.pddl').read_text(encoding='utf-8').casefold()
    )
    runtime = _without_comments(
        RUNTIME_DOMAIN_PATH.read_text(encoding='utf-8').casefold()
    )
    expert_actions = frozenset(
        re.findall(r'\(:action\s+([a-z0-9_]+)', expert)
    )
    runtime_actions = frozenset(
        re.findall(r'\(:action\s+([a-z0-9_]+)', runtime)
    )

    assert expert_actions == runtime_actions == RUNTIME_SYMBOLIC_ACTIONS
    assert 'move_shuttle' not in expert_actions
    assert 'wait_for_clearance' not in expert_actions


def test_expert_and_runtime_domains_are_symbolically_identical():
    """Comments may differ, but offline and live planning semantics may not."""

    def pddl_tokens(path):
        text = _without_comments(path.read_text(encoding='utf-8').casefold())
        return re.findall(r'[()]|[^()\s]+', text)

    assert pddl_tokens(PDDL_DIR / 'domain_room315.pddl') == pddl_tokens(
        RUNTIME_DOMAIN_PATH
    )


def test_clearance_dependency_actions_do_not_require_direct_blocker_atoms():
    """A proved capacity mover need not directly overlap the user's route."""

    for path in (PDDL_DIR / 'domain_room315.pddl', RUNTIME_DOMAIN_PATH):
        text = _without_comments(path.read_text(encoding='utf-8').casefold())
        slot_action = text.split(
            '(:action relocate_blocker_to_interior',
            1,
        )[1].split('(:action stage_selected_to_interior', 1)[0]
        segment_action = text.split(
            '(:action relocate_segment_blocker_to_interior',
            1,
        )[1].split('(:action stage_selected_segment_to_interior', 1)[0]

        assert '(clearance_precedes ?blocker ?selected)' in slot_action
        assert '(clearance_destination_ready ?blocker)' in slot_action
        assert '(interior_entry_route_clear ?blocker)' in slot_action
        assert '(route_blocked_by ?from_slot ?to_slot ?blocker)' not in (
            slot_action
        )

        assert '(clearance_precedes ?blocker ?selected)' in segment_action
        assert '(clearance_destination_ready ?blocker)' in segment_action
        assert '(interior_entry_route_clear ?blocker)' in segment_action
        assert (
            '(topology_route_blocked_by ?selected ?from_block ?to_slot '
            '?blocker)'
        ) not in segment_action


def test_topology_setup_groups_are_bound_to_the_planned_rail_side():
    for path in (PDDL_DIR / 'domain_room315.pddl', RUNTIME_DOMAIN_PATH):
        text = _without_comments(path.read_text(encoding='utf-8').casefold())
        assert '(switch_group_on_side ?group - switch_group ?side - rail_side)' in text
        assert '(stopper_group_on_side ?group - stopper_group ?side - rail_side)' in text
        for action_name in ('prepare_topology_route', 'prepare_slot_topology_route'):
            body = text.split(f'(:action {action_name}', 1)[1].split(
                '\n  (:action ', 1
            )[0]
            assert '(switch_group_on_side ?switches ?side)' in body
