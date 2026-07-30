#!/usr/bin/env python3

from __future__ import annotations

import io
import sys
from pathlib import Path

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_contracts import TaskGoal
from room_315_multi_shuttle import all_shuttle_specs
from room_315_pddl_scenario_generator import _planning_rail_topology
from room_315_pddl_scenario_generator import build_pddl_problem_from_observed_state_task_goal
from room_315_rail_defaults import LEFT_PUBLIC_SEGMENT_NAME_MAP
from room_315_rail_defaults import public_rail_segment_lengths
from room_315_task_execution import LiveStateConfig
from room_315_task_execution import LiveVisualSnapshot
from room_315_task_execution import TaskExecutionStateError
from room_315_task_execution import VisualObservedStateBuilder
from room_315_task_execution import VisualSupervisorTransport
from room_315_task_execution import LatestVisualObservedStateProvider
from room_315_task_execution import ground_transport_task_goal
from room_315_task_goal_cli import _print_turn_result
from room_315_task_goal_dialogue import DialogueTurnResult
from room_315_task_goal_dialogue import TaskGoalDialogueState


def _visual_item(identity: str, *, slot: str) -> dict:
    spec = next(item for item in all_shuttle_specs() if item.short_id == identity)
    location = _planning_rail_topology(spec.side).slots[str(slot)]
    segment = location.segment
    if spec.side == 'left':
        segment = LEFT_PUBLIC_SEGMENT_NAME_MAP.get(segment, segment)
    length = public_rail_segment_lengths(spec.side)[segment]
    return {
        'identity': identity,
        'presence_state': 'present',
        'visual_facts_valid': True,
        'side': spec.side,
        'block': segment,
        'bbox_xywh': [0.1, 0.2, 0.12, 0.08],
        's_m': location.s_ratio * length,
        's_ratio': location.s_ratio,
        'segment_length_m': length,
        'loaded_state': 'loaded' if identity == 'R4' else 'empty',
    }


def _observation(
    *,
    present: dict[str, str] | None = None,
    unknown: str = '',
) -> dict:
    present = present or {'R4': '2'}
    items = []
    for spec in all_shuttle_specs():
        if spec.short_id in present:
            item = _visual_item(spec.short_id, slot=present[spec.short_id])
        else:
            item = {
                'identity': spec.short_id,
                'presence_state': 'absent',
                'visual_facts_valid': False,
                'side': '',
                'block': '',
                'bbox_xywh': [0.0, 0.0, 0.0, 0.0],
                's_m': 0.0,
                's_ratio': 0.0,
                'segment_length_m': 0.0,
                'loaded_state': '',
            }
        if spec.short_id == unknown:
            item['presence_state'] = 'unknown'
        items.append(item)
    return {
        'timestamp_s': 10.0,
        'state_id': 'accepted-visual-10',
        'schema_version': 'room315.visual_state.v3',
        'checkpoint_sha256': 'a' * 64,
        'stage': 'fused_observed_state',
        'accepted': True,
        'stabilized': False,
        'stale': False,
        'model_ready': True,
        'input_ready': True,
        'presence_ready': True,
        'state_fusion_ready': True,
        'validation_reasons': [],
        'shuttles': items,
    }


def _supervisor(
    *,
    decision_count: int = 0,
    mode: str = 'WAITING',
) -> dict:
    rails = {}
    for side in ('left', 'right'):
        rails[side] = {
            'switches': {
                device: 'E'
                for device in ('A1', 'A2', 'A3', 'A4')
            },
            'stoppers': {
                device: '0'
                for device in ('A1', 'A2', 'A3', 'A4')
            },
            # Position-like controller fields are deliberately wrong. They
            # must not affect learned localization or slot derivation.
            'shuttles': {
                'room315_right_shuttle_4': {
                    'mode': mode,
                    'segment': 'CONTROLLER_POSITION_MUST_NOT_BE_USED',
                    's': -999.0,
                    'x': -999.0,
                    'y': -999.0,
                },
            } if side == 'right' else {},
        }
    return {
        'emergency_stop': False,
        'rails': rails,
        'safety_decoder': {
            'metrics': {'total_proposed_actions': decision_count},
            'last_decision': {
                'accepted': True,
                'status': 'accepted',
                'reason': '',
            },
        },
    }


def _snapshot(
    observation: dict | None = None,
    supervisor: dict | None = None,
) -> LiveVisualSnapshot:
    return LiveVisualSnapshot(
        observation=observation or _observation(),
        observation_receive_s=100.0,
        supervisor_status=supervisor or _supervisor(),
        supervisor_receive_s=100.0,
    )


def _fact(state, subject: str, predicate: str):
    return next(
        item
        for item in state.fused_planner_state
        if item.subject == subject and item.predicate == predicate
    )


def _transport_goal() -> TaskGoal:
    return TaskGoal(
        goal_id='move-r4-slot3',
        description='Move R4 to slot 3 on the right rail',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'payload_filter': 'any',
            'selection_strategy': 'explicit',
            'shuttle_selection': 'explicit',
            'side': 'right',
            'target_kind': 'slot',
            'target_shuttle': 'room315_right_shuttle_4',
            'target_slot': '3',
        },
    )


def test_builder_uses_visual_location_and_masks_absent_slots():
    builder = VisualObservedStateBuilder()
    state = builder.build(_snapshot(), now_s=100.1)

    assert _fact(
        state,
        'room315_right_shuttle_4',
        'location_slot',
    ).value == 'right:slot:2'
    assert _fact(
        state,
        'right:slot:2',
        'occupancy',
    ).value['shuttle'] == 'room315_right_shuttle_4'
    assert _fact(
        state,
        'right:slot:3',
        'occupancy',
    ).value['occupied'] is False
    assert not any(
        fact.subject == 'room315_right_shuttle_1'
        for fact in state.visual_model_inputs
    )
    assert _fact(
        state,
        'room315_right_shuttle_1',
        'present',
    ).value is False
    assert _fact(
        state,
        'room315_right_shuttle_4',
        'location_block',
    ).value != 'CONTROLLER_POSITION_MUST_NOT_BE_USED'


def test_unknown_presence_fails_closed():
    builder = VisualObservedStateBuilder()
    with pytest.raises(TaskExecutionStateError, match='unknown_presence:R4'):
        builder.build(
            _snapshot(observation=_observation(unknown='R4')),
            now_s=100.1,
        )


def test_stale_visual_or_supervisor_state_fails_closed():
    builder = VisualObservedStateBuilder()
    with pytest.raises(TaskExecutionStateError, match='accepted_visual_observation_stale'):
        builder.build(_snapshot(), now_s=102.0)
    stale_supervisor = LiveVisualSnapshot(
        observation=_observation(),
        observation_receive_s=100.0,
        supervisor_status=_supervisor(),
        supervisor_receive_s=98.0,
    )
    with pytest.raises(TaskExecutionStateError, match='supervisor_status_stale'):
        builder.build(stale_supervisor, now_s=100.1)


def test_planner_builder_matches_visual_validator_position_tolerance():
    builder = VisualObservedStateBuilder()
    accepted = _observation()
    r4 = next(
        item for item in accepted['shuttles']
        if item['identity'] == 'R4'
    )
    r4['s_ratio'] = (
        r4['s_m'] - 0.079
    ) / r4['segment_length_m']

    builder.build(_snapshot(observation=accepted), now_s=100.1)

    rejected = _observation()
    r4 = next(
        item for item in rejected['shuttles']
        if item['identity'] == 'R4'
    )
    r4['s_ratio'] = (
        r4['s_m'] - 0.081
    ) / r4['segment_length_m']
    with pytest.raises(
        TaskExecutionStateError,
        match='inconsistent_visual_position:R4',
    ):
        builder.build(_snapshot(observation=rejected), now_s=100.1)


def test_planning_slot_and_target_arrival_use_separate_ratio_tolerances():
    builder = VisualObservedStateBuilder()
    observation = _observation(present={'R4': '3'})
    r4 = next(
        item for item in observation['shuttles']
        if item['identity'] == 'R4'
    )
    target_ratio = float(r4['s_ratio'])

    # The planning view can associate a mildly noisy visual prediction with
    # the nearest slot, while the motion stop condition remains strict.
    r4['s_ratio'] = target_ratio - 0.10
    r4['s_m'] = r4['s_ratio'] * r4['segment_length_m']
    state = builder.build(
        _snapshot(observation=observation),
        now_s=100.1,
    )
    assert _fact(
        state,
        'room315_right_shuttle_4',
        'location_slot',
    ).value == 'right:slot:3'
    assert not builder.target_reached(
        observation,
        shuttle='right_shuttle_4',
        side='right',
        target_slot='3',
    )

    r4['s_ratio'] = target_ratio - 0.04
    r4['s_m'] = r4['s_ratio'] * r4['segment_length_m']
    assert builder.target_reached(
        observation,
        shuttle='right_shuttle_4',
        side='right',
        target_slot='3',
    )


def test_live_early_stop_observation_is_not_slot3_arrival():
    builder = VisualObservedStateBuilder()
    observation = _observation(present={'R4': '3'})
    r4 = next(
        item for item in observation['shuttles']
        if item['identity'] == 'R4'
    )
    # Captured from the accepted visual state at the premature stop.
    r4.update({
        'block': 'A34E',
        's_m': 0.2670303285121918,
        's_ratio': 0.2860526441138427,
        'segment_length_m': 0.9335006475448608,
    })

    assert not builder.target_reached(
        observation,
        shuttle='right_shuttle_4',
        side='right',
        target_slot='3',
    )


def test_pddl_replan_state_persists_observed_device_readiness():
    builder = VisualObservedStateBuilder()
    state = builder.build(_snapshot(), now_s=100.1)
    problem = build_pddl_problem_from_observed_state_task_goal(
        state,
        _transport_goal(),
    )

    assert '(switches_ready right)' in problem.problem_text
    assert '(stoppers_open right)' in problem.problem_text
    assert '(path_ready right right_yaskawa right_staubli)' in problem.problem_text
    assert problem.selected_shuttle == 'right_shuttle_4'


def test_nearest_loaded_goal_is_grounded_from_visual_facts():
    builder = VisualObservedStateBuilder()
    observation = _observation(present={'R2': '1', 'R4': '2'})
    for item in observation['shuttles']:
        if item['identity'] == 'R2':
            item['loaded_state'] = 'loaded'
    state = builder.build(
        _snapshot(observation=observation),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='nearest-loaded-to-slot3',
        description='Move the nearest loaded right shuttle to slot 3',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'payload_filter': 'loaded',
            'selection_strategy': 'nearest',
            'shuttle_selection': 'nearest',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '3',
        },
    )

    grounded = ground_transport_task_goal(goal, state)

    assert (
        grounded.constraints['target_shuttle']
        == 'room315_right_shuttle_4'
    )
    assert grounded.constraints['selection_strategy'] == 'explicit'


def test_grounding_respects_visual_payload_filter():
    builder = VisualObservedStateBuilder()
    observation = _observation(present={'R2': '2', 'R4': '1'})
    for item in observation['shuttles']:
        if item['identity'] == 'R2':
            item['loaded_state'] = 'empty'
    state = builder.build(
        _snapshot(observation=observation),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='any-empty-right',
        description='Move an empty right shuttle to slot 3',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'payload_filter': 'empty',
            'selection_strategy': 'any',
            'shuttle_selection': 'any',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '3',
        },
    )

    grounded = ground_transport_task_goal(goal, state)

    assert (
        grounded.constraints['target_shuttle']
        == 'room315_right_shuttle_2'
    )


def test_visual_target_arrival_sends_supervised_off_and_confirms_stop():
    builder = VisualObservedStateBuilder()
    provider = LatestVisualObservedStateProvider(builder)
    commands: list[dict] = []
    transport = None

    def publish(command: dict) -> None:
        commands.append(command)
        if command.get('command') == 'OFF':
            stopped = _supervisor(decision_count=1, mode='WAITING')
            transport.update_supervisor(stopped)
            provider.update_supervisor(stopped)

    transport = VisualSupervisorTransport(
        provider=provider,
        publish_callback=publish,
        arrival_confirmation_frames=1,
        controller_stop_timeout_s=1.0,
    )
    moving = _supervisor(decision_count=0, mode='MOVING')
    target = _observation(present={'R4': '3'})
    provider.update_supervisor(moving)
    provider.update_observation(target)
    transport.update_supervisor(moving)
    transport.update_observation(target)

    result = transport.wait_for_target_arrival(
        side='right',
        target_sensors=[],
        shuttle='right_shuttle_4',
        timeout_s=1.0,
        target_slot='3',
    )

    assert result['arrived']
    assert result['matched_by'] == 'accepted_visual_state'
    assert commands == [{
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'right_shuttle_4',
        'command': 'OFF',
        'closed_loop_executive': {
            'mode': 'visual_target_arrival_stop',
            'target_slot': '3',
            'location_source': 'accepted_visual_state',
        },
    }]


def test_confirmed_cli_goal_invokes_execution_handler():
    goal = _transport_goal()
    result = DialogueTurnResult(
        status='ok',
        state=TaskGoalDialogueState(),
        task_goal=goal,
    )
    received = []
    output = io.StringIO()

    _print_turn_result(
        result,
        output,
        on_task_goal=lambda item: (
            received.append(item)
            or {'status': 'published', 'goal_id': item.goal_id}
        ),
    )

    assert received == [goal]
    assert 'Final validated TaskGoal:' in output.getvalue()
    assert 'Task execution response:' in output.getvalue()
    assert '"status": "published"' in output.getvalue()
