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
            'shuttle_selection': 'loaded',
            'side': 'right',
            'target_station': 'staubli',
        },
        timestamp=10.0,
    ))
    assert loaded == {
        'goal_type': 'transport',
        'side': 'right',
        'target_kind': 'station',
        'target_station': 'staubli',
        'shuttle_selection': 'loaded',
        'payload_required': True,
    }

    empty = _constraints(builder.build_task_goal(
        'send the empty left shuttle to slot 4',
        timestamp=11.0,
    ))
    assert empty['side'] == 'left'
    assert empty['target_kind'] == 'slot'
    assert empty['target_slot'] == '4'
    assert empty['shuttle_selection'] == 'empty'
    assert empty['payload_required'] is False

    nearest = _constraints(builder.build_task_goal(
        'route the nearest right shuttle to slot 2',
        timestamp=12.0,
    ))
    assert nearest['side'] == 'right'
    assert nearest['target_slot'] == '2'
    assert nearest['shuttle_selection'] == 'nearest'
    assert 'target_shuttle' not in nearest

    explicit = _constraints(builder.build_task_goal(
        'move R3 to the Staubli station',
        timestamp=13.0,
    ))
    assert explicit['side'] == 'right'
    assert explicit['target_shuttle'] == 'room315_right_shuttle_3'
    assert explicit['shuttle_selection'] == 'explicit'
    assert explicit['target_station'] == 'staubli'


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
    assert {'missing_side', 'missing_shuttle_selection'} <= codes
    assert incomplete.to_dict()['clarifications'][0]['code']

    ambiguous = builder.build_task_goal('send the nearest shuttle to yaskawa', timestamp=1.0)
    assert ambiguous.status == 'clarification_required'
    assert any(issue.code == 'ambiguous_station_side' for issue in ambiguous.clarifications)


def test_strict_json_model_parser_outputs_task_goal_only():
    builder = _load_builder()
    result = builder.parse_model_task_goal_json(
        json.dumps({
            'goal_type': 'transport',
            'selection': 'nearest',
            'side': 'left',
            'target': {'kind': 'slot', 'slot': '1'},
            'confidence': 0.82,
        }),
        timestamp=30.0,
    )

    constraints = _constraints(result)
    assert result.task_goal.source == 'learned_task_goal'
    assert result.task_goal.confidence == 0.82
    assert constraints['goal_type'] == 'transport'
    assert constraints['shuttle_selection'] == 'nearest'
    assert constraints['target_slot'] == '1'
    assert 'pddl_goal' not in result.task_goal.to_dict()
    assert 'action_vector' not in result.task_goal.to_dict()


def test_strict_json_parser_rejects_adversarial_non_goal_outputs():
    builder = _load_builder()
    bad_payloads = [
        {'pddl_goal': '(:goal (and (task_done right_shuttle right_staubli)))'},
        {'action_vector_schema_version': 3, 'action_vector': [0.0] * 24},
        {'plan': ['move_shuttle right_shuttle yaskawa staubli']},
        {'primitive': 'SHUTTLE_ON', 'speed_mps': 0.08},
        {'goal_type': 'transport', 'side': 'right', 'target_station': 'staubli', 'deadline_s': 4.0},
        {'contract_type': 'PrimitiveCommand', 'primitive': 'WAIT'},
        {'constraints': {'goal_type': 'inspection', 'target': {'kind': 'slot', 'slot': '1', 'pddl': 'x'}}},
        {'constraints': {'goal_type': 'inspection', 'target': {'kind': 'slot', 'slot': '1', 'extra': 'x'}}},
    ]

    for payload in bad_payloads:
        result = builder.parse_model_task_goal_json(json.dumps(payload), timestamp=40.0)
        assert result.status == 'error', payload
        assert result.task_goal is None


def test_strict_json_parser_rejects_invalid_or_unknown_entities():
    builder = _load_builder()

    unknown = builder.parse_model_task_goal_json(
        json.dumps({
            'goal_type': 'transport',
            'selection': 'loaded',
            'side': 'right',
            'target_station': 'kuka',
        }),
        timestamp=50.0,
    )
    assert unknown.status == 'error'
    assert unknown.errors[0].code == 'station_side_mismatch'

    malformed = builder.parse_model_task_goal_json('move R1 to staubli', timestamp=50.0)
    assert malformed.status == 'error'
    assert malformed.errors[0].code == 'invalid_json'
