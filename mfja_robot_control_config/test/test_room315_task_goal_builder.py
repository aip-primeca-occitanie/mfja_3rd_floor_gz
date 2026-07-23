#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
SCRIPT_PATH = SCRIPT_DIR / 'room_315_task_goal_builder.py'


def _load_builder():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    spec = importlib.util.spec_from_file_location('room_315_task_goal_builder', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _constraints(result):
    assert result.ok, result.to_dict()
    assert result.task_goal.contract_type == 'TaskGoal'
    return result.task_goal.constraints


def test_builds_loaded_empty_nearest_and_explicit_transport_goals():
    builder = _load_builder()

    loaded = _constraints(builder.build_task_goal(
        {
            'goal_type': 'transport',
            'selection_strategy': 'any',
            'payload_filter': 'loaded',
            'side': 'right',
            'target_station': 'staubli',
        },
        timestamp=10.0,
    ))
    assert loaded['goal_type'] == 'transport'
    assert loaded['side'] == 'right'
    assert loaded['target_kind'] == 'station'
    assert loaded['target_station'] == 'staubli'
    assert loaded['selection_strategy'] == 'any'
    assert loaded['payload_filter'] == 'loaded'
    assert loaded['shuttle_selection'] == 'loaded'
    assert loaded['payload_required'] is True

    empty = _constraints(builder.build_task_goal(
        'send the empty left shuttle to slot 4',
        timestamp=11.0,
    ))
    assert empty['side'] == 'left'
    assert empty['target_kind'] == 'slot'
    assert empty['target_slot'] == '4'
    assert empty['shuttle_selection'] == 'empty'
    assert empty['selection_strategy'] == 'any'
    assert empty['payload_filter'] == 'empty'
    assert empty['payload_required'] is False

    nearest_result = builder.build_task_goal(
        'route the nearest right shuttle to slot 2',
        timestamp=12.0,
    )
    assert nearest_result.normalized_request['draft']['language'] == 'en'
    assert nearest_result.normalized_request['draft']['raw']['request'] == 'route the nearest right shuttle to slot 2'
    nearest = _constraints(nearest_result)
    assert nearest['side'] == 'right'
    assert nearest['target_slot'] == '2'
    assert nearest['shuttle_selection'] == 'nearest'
    assert nearest['selection_strategy'] == 'nearest'
    assert nearest['payload_filter'] == 'any'
    assert 'target_shuttle' not in nearest

    explicit = _constraints(builder.build_task_goal(
        'move R3 to the Staubli station',
        timestamp=13.0,
    ))
    assert explicit['side'] == 'right'
    assert explicit['target_shuttle'] == 'room315_right_shuttle_3'
    assert explicit['shuttle_selection'] == 'explicit'
    assert explicit['selection_strategy'] == 'explicit'
    assert explicit['payload_filter'] == 'any'
    assert explicit['target_station'] == 'staubli'

    nearest_loaded = _constraints(builder.build_task_goal(
        'move the nearest loaded right shuttle to slot 3',
        timestamp=14.0,
    ))
    assert nearest_loaded['selection_strategy'] == 'nearest'
    assert nearest_loaded['payload_filter'] == 'loaded'
    assert nearest_loaded['shuttle_selection'] == 'nearest'
    assert nearest_loaded['payload_required'] is True


def test_builds_inspection_goals_for_grounded_entities():
    builder = _load_builder()

    shuttle = _constraints(builder.build_task_goal('inspect L2', timestamp=20.0))
    assert shuttle['goal_type'] == 'inspection'
    assert shuttle['target_kind'] == 'shuttle'
    assert shuttle['side'] == 'left'
    assert shuttle['target_shuttle'] == 'room315_left_shuttle_2'
    assert shuttle['inspection_subject'] == 'room315_left_shuttle_2'

    slot = _constraints(builder.build_task_goal('check the right slot 3', timestamp=21.0))
    assert slot['target_kind'] == 'slot'
    assert slot['inspection_subject'] == 'right:slot:3'

    station = _constraints(builder.build_task_goal(
        {'goal_type': 'inspection', 'target_station': 'kuka'},
        timestamp=22.0,
    ))
    assert station['side'] == 'left'
    assert station['target_kind'] == 'station'
    assert station['inspection_subject'] == 'left:station:kuka'

    selected = _constraints(builder.build_task_goal(
        'inspect the loaded right shuttle',
        timestamp=23.0,
    ))
    assert selected['target_kind'] == 'shuttle_selection'
    assert selected['selection_strategy'] == 'any'
    assert selected['payload_filter'] == 'loaded'
    assert selected['shuttle_selection'] == 'loaded'
    assert selected['payload_required'] is True

    structured_slot = _constraints(builder.build_task_goal(
        {'goal_type': 'inspection', 'side': 'right', 'inspection_subject': 'slot 2'},
        timestamp=24.0,
    ))
    assert structured_slot['target_kind'] == 'slot'
    assert structured_slot['inspection_subject'] == 'right:slot:2'


def test_paraphrases_normalize_to_same_task_goal_id():
    builder = _load_builder()
    first = builder.build_task_goal('move the loaded right shuttle to Staubli', timestamp=1.0)
    second = builder.build_task_goal('bring a right shuttle carrying a part to the staubli station', timestamp=2.0)

    assert first.ok, first.to_dict()
    assert second.ok, second.to_dict()
    assert first.task_goal.goal_id == second.task_goal.goal_id
    assert first.task_goal.constraints == second.task_goal.constraints


def test_returns_structured_clarifications_for_incomplete_or_ambiguous_requests():
    builder = _load_builder()

    incomplete = builder.build_task_goal('move shuttle to slot 3', timestamp=1.0)
    assert incomplete.status == 'clarification_required'
    codes = {issue.code for issue in incomplete.clarifications}
    assert {'missing_side', 'missing_payload_filter'} <= codes
    assert 'missing_selection_strategy' not in codes
    assert incomplete.to_dict()['clarifications'][0]['code']

    ambiguous = builder.build_task_goal('send the nearest shuttle to yaskawa', timestamp=1.0)
    assert ambiguous.status == 'clarification_required'
    assert any(issue.code == 'ambiguous_station_side' for issue in ambiguous.clarifications)


def test_strict_json_model_parser_outputs_task_goal_only():
    builder = _load_builder()
    result = builder.parse_model_task_goal_json(
        json.dumps({
            'contract_type': 'TaskGoalDraft',
            'goal_type': 'transport',
            'selection_strategy': 'nearest',
            'payload_filter': 'loaded',
            'side': 'left',
            'target_kind': 'slot',
            'target_slot': '1',
            'confidence': 0.82,
        }),
        timestamp=30.0,
    )

    constraints = _constraints(result)
    assert result.task_goal.source == 'learned_task_goal'
    assert result.task_goal.confidence == 0.82
    assert constraints['goal_type'] == 'transport'
    assert constraints['selection_strategy'] == 'nearest'
    assert constraints['payload_filter'] == 'loaded'
    assert constraints['shuttle_selection'] == 'nearest'
    assert constraints['target_slot'] == '1'
    assert 'pddl_goal' not in result.task_goal.to_dict()
    assert 'action_vector' not in result.task_goal.to_dict()


def test_strict_json_parser_rejects_adversarial_non_goal_outputs():
    builder = _load_builder()
    bad_payloads = [
        {'pddl_goal': '(:goal (and (task_done right_shuttle right_staubli)))'},
        {'rail_command': {'action': 'shuttle', 'command': 'ON'}},
        {'plan': ['move_shuttle right_shuttle yaskawa staubli']},
        {'primitive': 'SHUTTLE_ON', 'speed_mps': 0.08},
        {'goal_type': 'transport', 'side': 'right', 'target_station': 'staubli', 'deadline_s': 4.0},
        {'contract_type': 'PrimitiveCommand', 'primitive': 'WAIT'},
        {'contract_type': 'TaskGoalDraft', 'goal_type': 'inspection', 'target_kind': 'slot', 'target_slot': '1', 'pddl': 'x'},
        {'contract_type': 'TaskGoalDraft', 'goal_type': 'inspection', 'target_kind': 'slot', 'target_slot': '1', 'extra': 'x'},
    ]

    for payload in bad_payloads:
        result = builder.parse_model_task_goal_json(json.dumps(payload), timestamp=40.0)
        assert result.status == 'error', payload
        assert result.task_goal is None


def test_strict_json_parser_rejects_invalid_or_unknown_entities():
    builder = _load_builder()

    unknown = builder.parse_model_task_goal_json(
        json.dumps({
            'contract_type': 'TaskGoalDraft',
            'goal_type': 'transport',
            'selection_strategy': 'any',
            'payload_filter': 'loaded',
            'side': 'right',
            'target_kind': 'station',
            'target_station': 'kuka',
        }),
        timestamp=50.0,
    )
    assert unknown.status == 'error'
    assert unknown.errors[0].code == 'station_side_mismatch'

    malformed = builder.parse_model_task_goal_json('move R1 to staubli', timestamp=50.0)
    assert malformed.status == 'error'
    assert malformed.errors[0].code == 'invalid_json'


def test_structured_form_parser_rejects_invalid_slots_and_conflicts():
    builder = _load_builder()

    invalid = builder.build_task_goal({
        'goal_type': 'transport',
        'selection_strategy': 'any',
        'payload_filter': 'loaded',
        'side': 'right',
        'target_kind': 'slot',
        'target_slot': '5',
    })
    assert invalid.status == 'error'
    assert invalid.errors[0].code == 'invalid_slot'

    conflict = builder.build_task_goal({
        'goal_type': 'transport',
        'selection_strategy': 'explicit',
        'payload_filter': 'any',
        'side': 'left',
        'target_shuttle': 'R2',
        'target_kind': 'slot',
        'target_slot': '3',
    })
    assert conflict.status == 'error'
    assert conflict.errors[0].code == 'shuttle_side_conflict'


def test_parser_pipeline_uses_regex_fallback_baseline():
    builder = _load_builder()

    class FailingParser:
        parser_name = 'forced_failure'

        def parse(self, request):
            return builder.parse_task_goal_draft({'unexpected': 'shape'})

    pipeline = builder.ParserPipeline([
        FailingParser(),
        builder.RegexFallbackParser(),
    ])
    result = pipeline.parse('move the nearest right shuttle to slot 2')
    assert result.ok
    assert result.parser_name == 'regex_fallback'
    assert result.draft.selection_strategy == 'nearest'


def test_stateful_clarification_requires_confirmation_before_final_task_goal():
    builder = _load_builder()
    manager = builder.TaskGoalDialogueManager(max_attempts=3)

    first = manager.handle('move the loaded shuttle to slot 3', timestamp=1.0)
    assert first.status == 'clarification_required'
    assert first.task_goal is None
    assert any(issue.field == 'side' for issue in first.clarifications)

    second = manager.handle('right', state=first.state, timestamp=2.0)
    assert second.status == 'confirmation_required'
    assert second.task_goal is None
    assert second.confirmation_required is True
    assert 'Confirm' in second.confirmation_prompt

    final = manager.handle('yes', state=second.state, timestamp=3.0)
    assert final.ok
    assert final.task_goal.constraints['side'] == 'right'
    assert final.task_goal.constraints['payload_filter'] == 'loaded'


def test_multi_turn_clarification_limits_attempts_and_avoids_unsafe_resolution():
    builder = _load_builder()
    manager = builder.TaskGoalDialogueManager(max_attempts=1)

    first = manager.handle('move shuttle to slot 3', timestamp=1.0)
    assert first.status == 'clarification_required'
    assert first.task_goal is None

    second = manager.handle('maybe later', state=first.state, timestamp=2.0)
    assert second.status == 'error'
    assert second.task_goal is None
    assert second.errors[0].code in {'parse_no_match', 'clarification_attempt_limit'}


def test_confirmation_decline_does_not_finalize_goal():
    builder = _load_builder()
    manager = builder.TaskGoalDialogueManager()

    first = manager.handle('move the nearest loaded right shuttle to slot 3', timestamp=1.0)
    assert first.status == 'confirmation_required'
    declined = manager.handle('no', state=first.state, timestamp=2.0)
    assert declined.status == 'clarification_required'
    assert declined.task_goal is None


def test_deterministic_ids_for_same_draft_and_final_goal():
    builder = _load_builder()
    first = builder.build_task_goal('move the nearest loaded right shuttle to slot 3', timestamp=1.0)
    second = builder.build_task_goal('transport nearest loaded right shuttle to slot 3', timestamp=99.0)

    assert first.ok
    assert second.ok
    assert first.task_goal.goal_id == second.task_goal.goal_id

    draft_a = builder.TaskGoalDraft(
        goal_type='transport',
        selection_strategy='nearest',
        payload_filter='loaded',
        side='right',
        target_kind='slot',
        target_slot='3',
    )
    draft_b = builder.TaskGoalDraft(
        goal_type='transport',
        selection_strategy='nearest',
        payload_filter='loaded',
        side='right',
        target_kind='slot',
        target_slot='3',
    )
    assert draft_a.draft_id == draft_b.draft_id
