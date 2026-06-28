#!/usr/bin/env python3

import re
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
    'move_shuttle',
    'stop_shuttle',
    'finish_task',
}


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
