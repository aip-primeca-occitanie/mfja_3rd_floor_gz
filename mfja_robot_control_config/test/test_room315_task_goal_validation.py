#!/usr/bin/env python3

from __future__ import annotations

import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_task_goal_schema import TaskGoalDraft
from room_315_task_goal_dialogue import merge_clarification_answer
from room_315_task_goal_validation import Room315DomainValidator


def _validate(**draft_fields):
    return Room315DomainValidator().validate(TaskGoalDraft(
        goal_type='inspection',
        **draft_fields,
    ))


def test_shuttle_inspection_requires_one_concrete_target_shuttle():
    result = _validate(target_kind='shuttle')

    assert result.status == 'clarification_required'
    assert result.task_goal is None
    issue = next(issue for issue in result.clarifications if issue.code == 'missing_shuttle')
    assert issue.field == 'target_shuttle'
    assert len(issue.options) == 8
    assert 'room315_left_shuttle_4' in issue.options
    assert 'room315_right_shuttle_4' in issue.options


def test_explicit_subject_cannot_replace_a_structured_inspection_target():
    result = _validate(inspection_subject='room315_system')

    assert result.status == 'error'
    assert result.task_goal is None
    assert {issue.code for issue in result.errors} == {'ungrounded_inspection_subject'}
    assert {issue.code for issue in result.clarifications} == {'missing_inspection_subject'}


def test_shuttle_inspection_never_falls_back_to_room315_system():
    missing = _validate(
        target_kind='shuttle',
        inspection_subject='room315_system',
    )
    mismatch = _validate(
        target_kind='shuttle',
        target_shuttle='L4',
        inspection_subject='room315_system',
    )

    assert missing.task_goal is None
    assert missing.status == 'clarification_required'
    assert mismatch.task_goal is None
    assert mismatch.status == 'error'
    assert mismatch.errors[0].code == 'inspection_subject_mismatch'
    assert mismatch.errors[0].options == ('room315_left_shuttle_4',)


@pytest.mark.parametrize(
    ('draft_fields', 'canonical_subject'),
    (
        ({'target_kind': 'shuttle', 'target_shuttle': 'R4'}, 'room315_right_shuttle_4'),
        ({'target_kind': 'slot', 'side': 'left', 'target_slot': '2'}, 'left:slot:2'),
        ({'target_kind': 'station', 'side': 'right', 'target_station': 'staubli'}, 'right:station:staubli'),
        ({'target_kind': 'rail', 'side': 'right'}, 'right:rail'),
        (
            {
                'target_kind': 'shuttle_selection',
                'side': 'left',
                'selection_strategy': 'any',
                'payload_filter': 'loaded',
            },
            'left:shuttle_selection:any:loaded',
        ),
        ({'target_kind': 'system'}, 'room315_system'),
    ),
)
def test_canonical_inspection_subject_whitelist_is_derived_from_structured_target(
    draft_fields,
    canonical_subject,
):
    derived = _validate(**draft_fields)
    redundant = _validate(**draft_fields, inspection_subject=canonical_subject)

    for result in (derived, redundant):
        assert result.ok, result.to_dict()
        assert result.task_goal.constraints['inspection_subject'] == canonical_subject
        assert result.description == f'inspect {canonical_subject}'
        if canonical_subject != 'room315_system':
            assert 'room315_system' not in result.description


def test_noncanonical_or_conflicting_explicit_inspection_subject_fails_closed():
    alias = _validate(
        target_kind='shuttle',
        target_shuttle='L4',
        inspection_subject='L4',
    )
    wrong_slot = _validate(
        target_kind='slot',
        side='right',
        target_slot='2',
        inspection_subject='right:slot:3',
    )

    for result in (alias, wrong_slot):
        assert result.status == 'error'
        assert result.task_goal is None
        assert result.errors[0].code == 'inspection_subject_mismatch'


@pytest.mark.parametrize(
    'draft_fields',
    (
        {'target_kind': 'shuttle', 'target_shuttle': 'L2', 'target_slot': '1'},
        {'target_kind': 'slot', 'side': 'left', 'target_slot': '1', 'target_station': 'yaskawa'},
        {'target_kind': 'station', 'side': 'left', 'target_station': 'kuka', 'target_shuttle': 'L1'},
        {'target_kind': 'rail', 'side': 'right', 'target_shuttle': 'R1'},
        {'target_kind': 'shuttle_selection', 'side': 'right', 'target_slot': '4'},
    ),
)
def test_inspection_target_is_atomic_and_rejects_conflicting_target_fields(draft_fields):
    result = _validate(**draft_fields)

    assert result.status == 'error'
    assert result.task_goal is None
    assert any(issue.code == 'ambiguous_inspection_target' for issue in result.errors)


@pytest.mark.parametrize('side', ('left', 'right'))
@pytest.mark.parametrize('payload_filter', ('loaded', 'empty', 'any'))
def test_nearest_shuttle_inspection_fails_before_confirmation(
    side,
    payload_filter,
):
    result = _validate(
        target_kind='shuttle_selection',
        side=side,
        selection_strategy='nearest',
        payload_filter=payload_filter,
    )

    assert result.status == 'error'
    assert result.task_goal is None
    assert {issue.code for issue in result.errors} == {
        'unsupported_nearest_inspection_reference',
    }


@pytest.mark.parametrize('payload_filter', ('loaded', 'empty', 'any'))
def test_explicit_shuttle_selection_requires_a_concrete_identity(payload_filter):
    result = _validate(
        target_kind='shuttle_selection',
        side='right',
        selection_strategy='explicit',
        payload_filter=payload_filter,
    )

    assert result.status == 'clarification_required'
    assert result.task_goal is None
    issue = next(
        issue for issue in result.clarifications
        if issue.code == 'missing_shuttle'
    )
    assert issue.field == 'target_shuttle'
    assert len(issue.options) == 8


def test_explicit_inspection_identity_clarification_becomes_a_shuttle_target():
    pending = TaskGoalDraft(
        goal_type='inspection',
        target_kind='shuttle_selection',
        side='right',
        selection_strategy='explicit',
        payload_filter='loaded',
    )

    merged = merge_clarification_answer(pending, 'R4')

    assert merged.status == 'ok'
    assert merged.draft.target_kind == 'shuttle'
    assert merged.draft.target_shuttle == 'r4'
    result = Room315DomainValidator().validate(merged.draft)
    assert result.ok, result.to_dict()
    assert result.task_goal.constraints['target_shuttle'] == (
        'room315_right_shuttle_4'
    )


@pytest.mark.parametrize('target_kind', ('system', 'rail', 'slot', 'station'))
def test_non_shuttle_inspection_rejects_irrelevant_selection_and_payload(
    target_kind,
):
    fields = {
        'target_kind': target_kind,
        'selection_strategy': 'any',
        'payload_filter': 'loaded',
    }
    if target_kind == 'rail':
        fields['side'] = 'right'
    elif target_kind == 'slot':
        fields.update(side='right', target_slot='2')
    elif target_kind == 'station':
        fields.update(side='right', target_station='staubli')

    result = _validate(**fields)

    assert result.status == 'error'
    assert result.task_goal is None
    assert {issue.code for issue in result.errors} == {
        'irrelevant_inspection_payload_filter',
        'irrelevant_inspection_selection',
    }


def test_system_inspection_has_one_canonical_subject_and_no_selector_defaults():
    result = _validate(target_kind='system')

    assert result.ok, result.to_dict()
    assert result.task_goal.constraints == {
        'goal_type': 'inspection',
        'inspection_subject': 'room315_system',
        'target_kind': 'system',
    }


def test_system_inspection_rejects_a_rail_side():
    result = _validate(target_kind='system', side='left')

    assert result.status == 'error'
    assert {issue.code for issue in result.errors} == {
        'system_inspection_side_conflict',
    }
