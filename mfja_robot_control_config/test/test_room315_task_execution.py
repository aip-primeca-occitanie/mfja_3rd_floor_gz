#!/usr/bin/env python3

from __future__ import annotations

import io
import itertools
import sys
from dataclasses import replace
from pathlib import Path

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_contracts import TaskGoal
from room_315_closed_loop_executive import ClosedLoopExecutive
from room_315_closed_loop_executive import _target_slot_for_step
from room_315_multi_shuttle import all_shuttle_specs
from room_315_multi_shuttle import DEFAULT_ROUTE_SAFETY_MARGIN_M
from room_315_multi_shuttle import DEFAULT_SHUTTLE_LENGTH_M
from room_315_pddl_plan_translator import PddlPlanStep
from room_315_pddl_plan_translator import translate_plan
from room_315_pddl_scenario_generator import _planning_rail_topology
from room_315_pddl_scenario_generator import PddlProblemBuildError
from room_315_pddl_scenario_generator import build_first_blocker_clearance_problem
from room_315_pddl_scenario_generator import build_clearance_pause_problem
from room_315_pddl_scenario_generator import build_intermediate_selected_advance_problem
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
from room_315_task_goal_schema import STATIONS_BY_SIDE
from room_315_task_goal_schema import TaskGoalDraft
from room_315_task_goal_validation import Room315DomainValidator


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
    reached_target_slot: str = '',
    shuttle_identity: str = 'R4',
) -> dict:
    shuttle_spec = next(
        item
        for item in all_shuttle_specs()
        if item.short_id == shuttle_identity
    )
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
                shuttle_spec.gazebo_entity_name: {
                    'mode': mode,
                    'segment': 'CONTROLLER_POSITION_MUST_NOT_BE_USED',
                    's': -999.0,
                    'x': -999.0,
                    'y': -999.0,
                    'reached_target_slot': reached_target_slot,
                },
            } if side == shuttle_spec.side else {},
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


def _clearance_certificate(
    identity: str = 'R2',
    *,
    target_s_m: float = 0.95,
) -> dict:
    return {
        'identity': identity,
        'shuttle': f'right_shuttle_{identity[-1]}',
        'side': 'right',
        'target_segment': 'A34I',
        'target_s_m': target_s_m,
        'observed_segment': 'A34E',
        'observed_s_m': target_s_m,
        'absolute_error_m': 0.0,
        'entry_sensor': 'DA3IR',
        'matched_by': 'interior_entry_sensor_plus_bounded_travel_time',
        'entry_sensor_identity_confirmed': True,
        'controller_stop_confirmed': True,
        'post_stop_visual_frame_received': True,
        'post_stop_visual_confirmation': False,
        'bounded_commanded_motion_completed': True,
        'clearance_mode_held': True,
        'normal_route_restored': False,
        'model_prediction_replaced': False,
        'controller_position_fields_used_for_localization': False,
    }


def _right_interior_slot_state(
    *,
    interior_s_m_by_identity: dict[str, float],
    slot_by_identity: dict[str, str],
):
    """Build an identity-agnostic right-rail topology/slot arrangement."""

    expected = {'R1', 'R2', 'R3', 'R4'}
    if set(interior_s_m_by_identity) & set(slot_by_identity):
        raise AssertionError('one shuttle cannot be both interior and in a slot')
    if set(interior_s_m_by_identity) | set(slot_by_identity) != expected:
        raise AssertionError('the helper requires all four right shuttles')
    present = {
        **{identity: '1' for identity in interior_s_m_by_identity},
        **slot_by_identity,
    }
    observation = _observation(present=present)
    interior_length = public_rail_segment_lengths('right')['A34I']
    for item in observation['shuttles']:
        target_s_m = interior_s_m_by_identity.get(item['identity'])
        if target_s_m is None:
            continue
        item.update({
            'block': 'A34I',
            's_m': target_s_m,
            's_ratio': target_s_m / interior_length,
            'segment_length_m': interior_length,
        })
    certificates = {
        identity: {
            **_clearance_certificate(identity, target_s_m=target_s_m),
            'observed_segment': 'A34I',
            'observed_s_m': target_s_m,
        }
        for identity, target_s_m in interior_s_m_by_identity.items()
    }
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=observation),
        now_s=100.1,
        runtime_clearance_certificates=certificates,
        slot_sensor_anchors={
            identity: {
                'identity': identity,
                'side': 'right',
                'slot': slot,
                'sensor': f'DZI{slot}R',
            }
            for identity, slot in slot_by_identity.items()
        },
    )
    return state, certificates


def _verified_slot_arrival_certificate(
    identity: str = 'R2',
    *,
    slot: str = '1',
    sensor_sequence: int = 7,
    supervisor_sequence: int = 11,
    motion_epoch: int = 0,
) -> dict:
    spec = next(item for item in all_shuttle_specs() if item.short_id == identity)
    sensor = f'DZI{slot}{"R" if spec.side == "right" else "L"}'
    return {
        'identity': identity,
        'shuttle': spec.gazebo_entity_name,
        'side': spec.side,
        'slot': slot,
        'sensor': sensor,
        'matched_by': 'deterministic_slot_sensor',
        'proof_mode': 'supervised_command_arrival',
        'motion_epoch': motion_epoch,
        'sensor_identity_confirmed': True,
        'sensor_confirmation_frames': 2,
        'sensor_sequence': sensor_sequence,
        'controller_stop_confirmed': True,
        'controller_mode': 'DISABLED',
        'controller_target_slot_confirmed': True,
        'reached_target_slot': slot,
        'supervisor_sequence': supervisor_sequence,
        'model_prediction_replaced': False,
        'controller_position_fields_used_for_localization': False,
    }


def _sequential_cutoff_state_and_certificates(
    *,
    include_unrelated_left_anchor_mismatch: bool = False,
):
    """Reproduce the accepted state before the live R3 slot-1 -> slot-2 failure."""

    present = {'R1': '3', 'R2': '4', 'R3': '1', 'R4': '2'}
    if include_unrelated_left_anchor_mismatch:
        present['L2'] = '2'
    observation = _observation(
        present=present
    )
    segment_length_m = public_rail_segment_lengths('right')['A34I']
    staged_positions_m = {'R1': 0.35, 'R2': 0.95}
    for item in observation['shuttles']:
        target_s_m = staged_positions_m.get(item['identity'])
        if (
            include_unrelated_left_anchor_mismatch
            and item['identity'] == 'L2'
        ):
            # Reproduce the exact live discrepancy.  L2 has a trusted DZI2L
            # anchor but the learned longitudinal ratio is just outside the
            # consistency tolerance.  It must remain a diagnostic for a
            # right-rail task, not veto R3/R4 route planning.
            item['s_ratio'] += 0.121590135
            item['s_m'] = item['s_ratio'] * item['segment_length_m']
        if target_s_m is None:
            continue
        item.update({
            'block': 'A34I',
            's_m': target_s_m,
            's_ratio': target_s_m / segment_length_m,
            'segment_length_m': segment_length_m,
        })

    certificates = {
        identity: _clearance_certificate(identity, target_s_m=target_s_m)
        for identity, target_s_m in staged_positions_m.items()
    }
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=observation),
        now_s=100.1,
        runtime_clearance_certificates=certificates,
        slot_sensor_anchors=(
            {
                'L2': {
                    'identity': 'L2',
                    'side': 'left',
                    'slot': '2',
                    'sensor': 'DZI2L',
                },
            }
            if include_unrelated_left_anchor_mismatch
            else None
        ),
    )
    goal = TaskGoal(
        goal_id='sequential-r3-slot1-to-slot2',
        description='Move R3 from right slot 1 to right slot 2',
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
            'target_shuttle': 'room315_right_shuttle_3',
            'target_slot': '2',
        },
    )
    return state, certificates, goal


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


def test_builder_canonicalizes_compact_controller_device_states():
    supervisor = _supervisor()
    supervisor['rails']['right']['switches'] = {
        'A1': 'E',
        'A2': 'EXTERIOR',
        'A3': 'I',
        'A4': 'INTERIOR',
    }
    supervisor['rails']['right']['stoppers'] = {
        'A1': '0',
        'A2': 'OPEN',
        'A3': '1',
        'A4': 'CLOSED',
    }

    state = VisualObservedStateBuilder().build(
        _snapshot(supervisor=supervisor),
        now_s=100.1,
    )

    assert _fact(state, 'right:switch:A1', 'state').value == 'EXTERIOR'
    assert _fact(state, 'right:switch:A2', 'state').value == 'EXTERIOR'
    assert _fact(state, 'right:switch:A3', 'state').value == 'INTERIOR'
    assert _fact(state, 'right:switch:A4', 'state').value == 'INTERIOR'
    assert _fact(state, 'right:stopper:A1', 'state').value == 'open'
    assert _fact(state, 'right:stopper:A2', 'state').value == 'open'
    assert _fact(state, 'right:stopper:A3', 'state').value == 'closed'
    assert _fact(state, 'right:stopper:A4', 'state').value == 'closed'


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


def test_never_received_visual_or_supervisor_state_is_reported_as_unavailable():
    builder = VisualObservedStateBuilder()
    missing_visual = LiveVisualSnapshot(
        observation={},
        observation_receive_s=-1.0,
        supervisor_status=_supervisor(),
        supervisor_receive_s=100.0,
    )
    with pytest.raises(
        TaskExecutionStateError,
        match='accepted_visual_observation_unavailable',
    ):
        builder.build(missing_visual, now_s=100.1)

    missing_supervisor = LiveVisualSnapshot(
        observation=_observation(),
        observation_receive_s=100.0,
        supervisor_status={},
        supervisor_receive_s=-1.0,
    )
    with pytest.raises(
        TaskExecutionStateError,
        match='supervisor_status_unavailable',
    ):
        builder.build(missing_supervisor, now_s=100.1)


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


def test_sensor_certified_interior_relocation_suppresses_only_false_slot_claim():
    builder = VisualObservedStateBuilder()
    observation = _observation(
        present={'R1': '1', 'R2': '2', 'R3': '3', 'R4': '4'}
    )
    r2 = next(
        item for item in observation['shuttles']
        if item['identity'] == 'R2'
    )
    # Reproduce the live model error after R2 physically reached A34I@0.95:
    # identity and longitudinal position are accurate, parallel branch is not.
    r2.update({
        'block': 'A34E',
        's_ratio': 0.5688492991615494,
        'segment_length_m': 1.6587773561477661,
    })
    r2['s_m'] = r2['s_ratio'] * r2['segment_length_m']

    uncorrected = builder.build(
        _snapshot(observation=observation),
        now_s=100.1,
    )
    assert not any(
        fact.subject == 'room315_right_shuttle_2'
        and fact.predicate == 'location_slot'
        for fact in uncorrected.fused_planner_state
    )

    certificate = _clearance_certificate()
    state = builder.build(
        _snapshot(observation=observation),
        now_s=100.1,
        runtime_clearance_certificates={'R2': certificate},
    )

    assert _fact(
        state,
        'right:slot:4',
        'occupancy',
    ).value['shuttle'] == 'room315_right_shuttle_4'
    assert not any(
        fact.subject == 'room315_right_shuttle_2'
        and fact.predicate == 'location_slot'
        for fact in state.fused_planner_state
    )
    assert _fact(
        state,
        'room315_right_shuttle_2',
        'location_block',
    ).value == 'right:A34E'
    clearance = _fact(
        state,
        'room315_right_shuttle_2',
        'runtime_route_clearance',
    )
    assert clearance.value['target_segment'] == 'A34I'
    assert clearance.metadata['selected_source'] == 'executor'


def test_global_slot_assignment_resolves_live_r3_r4_shared_slot_bias():
    builder = VisualObservedStateBuilder()
    observation = _observation(present={'R3': '3', 'R4': '4'})
    values = {
        'R3': (0.804, 0.499),
        'R4': (1.325, 0.547),
    }
    for item in observation['shuttles']:
        if item['identity'] not in values:
            continue
        s_m, s_ratio = values[item['identity']]
        item.update({
            'block': 'A34E',
            's_m': s_m,
            's_ratio': s_ratio,
            'segment_length_m': s_m / s_ratio,
        })

    state = builder.build(_snapshot(observation=observation), now_s=100.1)

    assert _fact(
        state,
        'right:slot:3',
        'occupancy',
    ).value['shuttle'] == 'room315_right_shuttle_3'
    assert _fact(
        state,
        'right:slot:4',
        'occupancy',
    ).value['shuttle'] == 'room315_right_shuttle_4'


def test_live_full_rail_r4_slot4_to_slot2_separates_from_rear_margin():
    """Replay the post-dual-branch regression from the running Gazebo state.

    R1 is visually behind R2 on A12E, but the learned centres are close enough
    that R1's protected spacing interval crosses R2's route origin.  Moving R2
    forward toward A3 increases that separation and must not create the false
    R2 -> R1 -> R2 dependency cycle that previously aborted before PlanSys2.
    """

    live_positions = {
        'L1': ('A12E', 0.5124400854110718),
        'L2': ('A12E', 0.6684333086013794),
        'L3': ('A34E', 0.4723213550810661),
        'L4': ('A34E', 0.6253717077315208),
        'R1': ('A12E', 0.4512688832257739),
        'R2': ('A12E', 0.6158399252122138),
        'R3': ('A34E', 0.5004833340644836),
        'R4': ('A34E', 0.6487650275230408),
    }
    observation = _observation(
        present={identity: '1' for identity in live_positions}
    )
    for item in observation['shuttles']:
        segment, ratio = live_positions[item['identity']]
        length = public_rail_segment_lengths(item['side'])[segment]
        item.update({
            'block': segment,
            's_m': ratio * length,
            's_ratio': ratio,
            'segment_length_m': length,
            'loaded_state': (
                'loaded' if item['identity'] == 'R4' else 'empty'
            ),
        })

    state = VisualObservedStateBuilder().build(
        _snapshot(observation=observation),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='live-r4-slot4-to-slot2-rear-margin-regression',
        description='Move the yellow shuttle to slot 2',
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
            'target_slot': '2',
        },
    )

    problem = build_pddl_problem_from_observed_state_task_goal(state, goal)
    clearance = problem.provenance['target_blocker_clearance_plan']
    assert clearance['source_slot'] == 'right_slot_4'
    assert clearance['target_slot'] == 'right_slot_2'
    assert clearance['observed_blockers'] == [
        'right_shuttle_2',
        'right_shuttle_1',
    ]
    relocation = clearance['ordered_relocations'][0]
    assert relocation['shuttle'] == 'right_shuttle_2'
    assert relocation['destination']['kind'] == 'interior_loop'
    assert relocation['destination']['gate_switch'] == 'A3'
    assert relocation['destination']['target_segment'] == 'A34I'

    a3_result = next(
        result
        for result in clearance['clearance_branch_search']['results']
        if result['gate_switch'] == 'A3'
    )
    assert a3_result['resolved'] is True
    assert a3_result['first_movable_shuttle'] == 'right_shuttle_2'
    assert a3_result['dependency_chain'] == ['right_shuttle_2']
    entry = a3_result['route_entries'][0]
    assert entry['blockers'] == []
    assert entry['ignored_rear_separation_overlaps'] == [{
        'shuttle': 'right_shuttle_1',
        'source_segment': 'A12E',
        'mover_s_ratio': pytest.approx(0.615839925),
        'rear_shuttle_s_ratio': pytest.approx(0.451268883),
        'rear_occupancy_start_s_ratio': pytest.approx(0.267231324),
        'rear_occupancy_end_s_ratio': pytest.approx(0.635306443),
        'overlap_route_block_indices': [0],
        'proof': 'forward_motion_monotonically_increases_rear_spacing',
        'controller_position_fields_used_for_localization': False,
    }]
    assert entry['route_candidates'][0]['raw_interval_blockers'] == [
        'right_shuttle_1'
    ]

    subproblem = ClosedLoopExecutive._next_planning_problem(problem)
    assert subproblem.provenance['planning_phase'] == (
        'clear_blocker_to_interior_loop'
    )
    assert subproblem.selected_shuttle == 'right_shuttle_2'
    assert subproblem.goal_text == '(clearance_relocated right_shuttle_2)'


@pytest.mark.parametrize(
    (
        'selected_identity',
        'other_interior_identity',
        'target_occupant_identity',
        'vacancy_dependency_identity',
    ),
    list(itertools.permutations(('R1', 'R2', 'R3', 'R4'))),
)
def test_segment_origin_vacancy_rotation_is_identity_agnostic(
    selected_identity,
    other_interior_identity,
    target_occupant_identity,
    vacancy_dependency_identity,
):
    """Any identity distribution uses slot 3 -> 4 before freeing slot 2."""

    state, certificates = _right_interior_slot_state(
        interior_s_m_by_identity={
            selected_identity: 0.95,
            other_interior_identity: 0.35,
        },
        slot_by_identity={
            target_occupant_identity: '2',
            vacancy_dependency_identity: '3',
        },
    )
    goal = TaskGoal(
        goal_id=(
            'identity-agnostic-segment-vacancy-'
            f'{selected_identity}-{target_occupant_identity}'
        ),
        description=f'Move {selected_identity} to right slot 2',
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
            'target_shuttle': f'room315_right_shuttle_{selected_identity[-1]}',
            'target_slot': '2',
        },
    )

    problem = build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
        runtime_clearance_certificates=certificates,
    )
    clearance = problem.provenance['target_blocker_clearance_plan']
    relocation = clearance['ordered_relocations'][0]
    dependency = f'right_shuttle_{vacancy_dependency_identity[-1]}'
    occupant = f'right_shuttle_{target_occupant_identity[-1]}'
    assert relocation['shuttle'] == dependency
    assert relocation['reason'] == 'blocks_target_occupant_relocation'
    assert relocation['destination'] == {
        'kind': 'slot',
        'source_slot': 'right_slot_3',
        'target_slot': 'right_slot_4',
        'target_sensor': 'DZI4R',
        'selection_policy': 'nearest_forward_vacancy_dependency_step',
    }
    assert relocation['dependency_for_shuttle'] == occupant
    resolution = clearance['vacancy_dependency_resolution']
    assert resolution['selected_source_slot'] == ''
    assert resolution['selected_source_kind'] == (
        'accepted_visual_continuous_position'
    )
    assert resolution['first_safe_move'] == {
        'shuttle': dependency,
        'source_slot': 'right_slot_3',
        'target_slot': 'right_slot_4',
    }
    assert resolution['policy'] == (
        'propagate_nearest_forward_vacancy_then_reobserve'
    )

    subproblem = ClosedLoopExecutive._next_planning_problem(problem)
    assert subproblem.provenance['planning_phase'] == 'clear_blocker_to_slot'
    assert subproblem.selected_shuttle == dependency
    assert subproblem.target_slot == '4'
    assert subproblem.goal_text == f'(shuttle_at_slot {dependency} right_slot_4)'
    assert '(= (pending_clearances right) 0)' in subproblem.problem_text


def test_live_blue_a34i_to_slot2_uses_complete_exterior_vacancy_rotation():
    """Replay R2@A34I, R4@2, R3@3 and prove the full three-step chain."""

    goal = TaskGoal(
        goal_id='live-blue-a34i-to-slot2-vacancy-rotation',
        description='Move blue R2 to right slot 2',
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
            'target_shuttle': 'room315_right_shuttle_2',
            'target_slot': '2',
        },
    )
    interior = {'R1': 0.49, 'R2': 1.066772}

    initial, certificates = _right_interior_slot_state(
        interior_s_m_by_identity=interior,
        slot_by_identity={'R3': '3', 'R4': '2'},
    )
    first_problem = build_pddl_problem_from_observed_state_task_goal(
        initial,
        goal,
        runtime_clearance_certificates=certificates,
    )
    first = first_problem.provenance['target_blocker_clearance_plan'][
        'ordered_relocations'
    ][0]
    assert first['shuttle'] == 'right_shuttle_3'
    assert first['destination']['source_slot'] == 'right_slot_3'
    assert first['destination']['target_slot'] == 'right_slot_4'

    after_green, certificates = _right_interior_slot_state(
        interior_s_m_by_identity=interior,
        slot_by_identity={'R3': '4', 'R4': '2'},
    )
    second_problem = build_pddl_problem_from_observed_state_task_goal(
        after_green,
        goal,
        runtime_clearance_certificates=certificates,
    )
    second = second_problem.provenance['target_blocker_clearance_plan'][
        'ordered_relocations'
    ][0]
    assert second['shuttle'] == 'right_shuttle_4'
    assert second['destination']['kind'] == 'slot'
    assert second['destination']['source_slot'] == 'right_slot_2'
    assert second['destination']['target_slot'] == 'right_slot_3'

    after_yellow, certificates = _right_interior_slot_state(
        interior_s_m_by_identity=interior,
        slot_by_identity={'R3': '4', 'R4': '3'},
    )
    final_problem = build_pddl_problem_from_observed_state_task_goal(
        after_yellow,
        goal,
        runtime_clearance_certificates=certificates,
    )
    final_clearance = final_problem.provenance[
        'target_blocker_clearance_plan'
    ]
    assert final_clearance['required'] is False
    assert final_clearance['ordered_relocations'] == []
    final_route = final_problem.provenance['topology_routes']['routes'][0]
    assert final_route['shuttle'] == 'right_shuttle_2'
    assert final_route['target_slot_object'] == 'right_slot_2'
    assert final_route['blockers'] == []
    assert final_route['route_clear'] is True


def test_binary_slot_sensor_anchor_recovers_live_r4_position_outlier():
    builder = VisualObservedStateBuilder()
    observation = _observation(present={'R3': '3', 'R4': '4'})
    r4 = next(
        item for item in observation['shuttles']
        if item['identity'] == 'R4'
    )
    # Live accepted prediction that correctly identified R4/A34E but was too
    # far from either slot to cross the visual-only planning boundary.
    r4.update({
        'block': 'A34E',
        's_m': 0.11299154162406921,
        's_ratio': 0.18221604203357064,
        'segment_length_m': 0.6200965642929077,
    })

    without_anchor = builder.build(
        _snapshot(observation=observation),
        now_s=100.1,
    )
    assert not any(
        fact.subject == 'room315_right_shuttle_4'
        and fact.predicate == 'location_slot'
        for fact in without_anchor.fused_planner_state
    )

    state = builder.build(
        _snapshot(observation=observation),
        now_s=100.1,
        slot_sensor_anchors={
            'R4': {
                'identity': 'R4',
                'sensor': 'DZI4R',
                # Deliberately ignored position-like fields.
                'segment': 'FORBIDDEN',
                's': -999.0,
                's_ratio': -999.0,
            },
        },
    )

    slot = _fact(state, 'room315_right_shuttle_4', 'location_slot')
    assert slot.value == 'right:slot:4'
    assert slot.source == 'state_fuser'
    assert slot.metadata['selected_source'] == 'trusted_device'
    assert _fact(
        state,
        'room315_right_shuttle_4',
        'location_block',
    ).value == 'right:A34E'
    position = _fact(
        state,
        'room315_right_shuttle_4',
        'rail_position',
    )
    assert position.value['s_m'] == pytest.approx(0.11299154162406921)
    assert position.source == 'state_fuser'
    assert position.metadata['selected_source'] == 'visual_model'


def test_slot_sensor_anchor_is_fresh_bounded_and_uses_no_position_fields():
    clock = [100.0]
    provider = LatestVisualObservedStateProvider(
        VisualObservedStateBuilder(),
        monotonic=lambda: clock[0],
    )
    provider.update_slot_sensor_feedback(
        'right',
        [{
            'active': True,
            'name': 'DZI4R',
            'shuttle': 'room315_right_shuttle_4',
            'segment': 'IGNORED',
            's': -999.0,
            's_ratio': -999.0,
        }],
        receive_s=clock[0],
    )
    assert provider.slot_sensor_anchors(now_s=100.5)['R4']['slot'] == '4'
    clock[0] = 101.1
    assert provider.slot_sensor_anchors(now_s=clock[0]) == {}


def test_verified_arrival_replays_live_r2_ratio_bias_from_canonical_slot_origin():
    """Regression for the R2 slot-1 -> slot-2 0.121863467 mismatch."""

    builder = VisualObservedStateBuilder()
    observation = _observation(
        present={'R1': '3', 'R2': '1', 'R3': '3', 'R4': '4'}
    )
    # Exact accepted state after the prior step moved green R3 out of slot 2:
    # red R1 is safely stopped in A34I, blue R2 is physically on DZI1R,
    # green R3 is at slot 3, and yellow R4 is at slot 4.
    r1 = next(
        item for item in observation['shuttles']
        if item['identity'] == 'R1'
    )
    interior_length = public_rail_segment_lengths('right')['A34I']
    r1.update({
        'block': 'A34I',
        's_m': 0.35,
        's_ratio': 0.35 / interior_length,
        'segment_length_m': interior_length,
    })
    r2 = next(
        item for item in observation['shuttles']
        if item['identity'] == 'R2'
    )
    slot = _planning_rail_topology('right').slots['1']
    biased_ratio = slot.s_ratio + 0.121863467
    r2.update({
        'block': slot.segment,
        's_ratio': biased_ratio,
        's_m': biased_ratio * r2['segment_length_m'],
    })
    certificate = _verified_slot_arrival_certificate('R2', slot='1')
    clearance_certificates = {
        'R1': _clearance_certificate('R1', target_s_m=0.35),
    }
    state = builder.build(
        _snapshot(observation=observation),
        now_s=100.1,
        runtime_clearance_certificates=clearance_certificates,
        slot_sensor_anchors={
            'R2': {
                'identity': 'R2',
                'side': 'right',
                'slot': '1',
                'sensor': 'DZI1R',
            },
        },
        verified_slot_arrival_certificates={'R2': certificate},
    )

    raw_position = _fact(
        state,
        'room315_right_shuttle_2',
        'rail_position',
    )
    assert raw_position.value['s_ratio'] == pytest.approx(biased_ratio)
    assert raw_position.metadata['selected_source'] == 'visual_model'
    verified = _fact(
        state,
        'room315_right_shuttle_2',
        'verified_slot_arrival',
    )
    assert verified.metadata['selected_source'] == 'executor'
    assert verified.value['model_prediction_replaced'] is False

    goal = TaskGoal(
        goal_id='replay-r2-slot1-to-slot2-after-verified-arrival',
        description='Move R2 from right slot 1 to right slot 2',
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
            'target_shuttle': 'room315_right_shuttle_2',
            'target_slot': '2',
        },
    )
    problem = build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
        runtime_clearance_certificates=clearance_certificates,
    )

    assert '(shuttle_at_slot right_shuttle_2 right_slot_1)' in problem.problem_text
    assert '(segment_only_location right_shuttle_2)' not in problem.problem_text
    assert not any(
        route['shuttle'] == 'right_shuttle_2'
        for route in problem.provenance['topology_routes']['routes']
    )
    clearance = problem.provenance['target_blocker_clearance_plan']
    assert clearance['source_kind'] == 'slot'
    assert clearance['required'] is False
    assert clearance['ordered_relocations'] == []
    diagnostic = problem.provenance['planning_scope'][
        'exact_slot_anchor_visual_diagnostics'
    ]['right_shuttle_2']
    assert diagnostic['absolute_ratio_error'] == pytest.approx(0.121863467)
    assert diagnostic[
        'accepted_under_verified_arrival_certificate'
    ] is True
    anchor = next(
        item
        for item in problem.provenance['route_clearance'][
            'verified_slot_arrival_anchors'
        ]
        if item['shuttle'] == 'right_shuttle_2'
    )
    assert anchor['raw_visual_s_ratio'] == pytest.approx(biased_ratio)
    assert anchor['canonical_slot_s_ratio'] == pytest.approx(slot.s_ratio)
    required_spacing_ratio = (
        DEFAULT_SHUTTLE_LENGTH_M + DEFAULT_ROUTE_SAFETY_MARGIN_M
    ) / r2['segment_length_m']
    assert anchor['occupancy_start_s_ratio'] == pytest.approx(
        max(0.0, slot.s_ratio - required_spacing_ratio)
    )
    assert anchor['occupancy_end_s_ratio'] == pytest.approx(
        min(1.0, slot.s_ratio + required_spacing_ratio)
    )
    assert anchor['occupancy_source'] == (
        'verified_identity_bearing_slot_sensor'
    )
    assert anchor['raw_visual_s_ratio_used_for_occupancy'] is False
    assert anchor['raw_visual_position_replaced'] is False

    # The following user request in the captured sequence also used to fail
    # before planning on the same R2 ratio mismatch.  It must now identify R2
    # as the exact slot-1 occupant and produce its first safe relocation.
    red_goal = TaskGoal(
        goal_id='replay-r1-a34i-to-slot1-after-r2-arrival',
        description='Move R1 from A34I to right slot 1',
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
            'target_shuttle': 'room315_right_shuttle_1',
            'target_slot': '1',
        },
    )
    red_problem = build_pddl_problem_from_observed_state_task_goal(
        state,
        red_goal,
        runtime_clearance_certificates=clearance_certificates,
    )
    red_clearance = red_problem.provenance[
        'target_blocker_clearance_plan'
    ]
    assert red_clearance['required'] is True
    assert red_clearance['ordered_relocations'][0]['shuttle'] == (
        'right_shuttle_2'
    )
    assert red_clearance['ordered_relocations'][0]['reason'] == (
        'occupies_goal_target_slot'
    )
    assert red_clearance['ordered_relocations'][0]['destination'][
        'target_slot'
    ] == 'right_slot_2'


def test_verified_arrival_never_overrides_visual_segment_disagreement():
    builder = VisualObservedStateBuilder()
    observation = _observation(present={'R2': '1'})
    r2 = next(
        item for item in observation['shuttles']
        if item['identity'] == 'R2'
    )
    r2.update({
        'block': 'A34E',
        's_ratio': 0.45,
        's_m': 0.45 * public_rail_segment_lengths('right')['A34E'],
        'segment_length_m': public_rail_segment_lengths('right')['A34E'],
    })
    state = builder.build(
        _snapshot(observation=observation),
        now_s=100.1,
        slot_sensor_anchors={
            'R2': {'identity': 'R2', 'sensor': 'DZI1R'},
        },
        verified_slot_arrival_certificates={
            'R2': _verified_slot_arrival_certificate('R2', slot='1'),
        },
    )
    goal = TaskGoal(
        goal_id='segment-mismatch-stays-fail-closed',
        description='Move R2 to right slot 2',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_shuttle': 'room315_right_shuttle_2',
            'target_slot': '2',
            'payload_filter': 'any',
        },
    )

    with pytest.raises(
        PddlProblemBuildError,
        match='exact slot anchor and visual segment disagree',
    ):
        build_pddl_problem_from_observed_state_task_goal(state, goal)


def test_live_provider_recovery_requires_visual_state_id_advance():
    provider = LatestVisualObservedStateProvider(
        VisualObservedStateBuilder(LiveStateConfig(
            observation_wait_s=0.02,
        )),
    )
    first_observation = _observation()
    provider.update_observation(first_observation)
    provider.update_supervisor(_supervisor())
    first_state = provider.observe()

    with pytest.raises(
        TaskExecutionStateError,
        match=(
            'accepted visual observation did not advance beyond '
            'accepted-visual-10'
        ),
    ):
        provider.observe_fresh_after(first_state.state_id)

    second_observation = _observation()
    second_observation['state_id'] = 'accepted-visual-11'
    second_observation['timestamp_s'] = 10.1
    provider.update_observation(second_observation)
    second_state = provider.observe_fresh_after(first_state.state_id)

    assert second_state.state_id == 'accepted-visual-11'


def test_certified_first_blocker_is_excluded_and_second_staging_is_separated():
    builder = VisualObservedStateBuilder()
    observation = _observation(
        present={'R1': '1', 'R2': '2', 'R3': '3', 'R4': '4'}
    )
    r2 = next(
        item for item in observation['shuttles']
        if item['identity'] == 'R2'
    )
    r2.update({
        'block': 'A34E',
        's_ratio': 0.5688492991615494,
        'segment_length_m': 1.6587773561477661,
    })
    r2['s_m'] = r2['s_ratio'] * r2['segment_length_m']
    certificate = _clearance_certificate(target_s_m=0.95)
    state = builder.build(
        _snapshot(observation=observation),
        now_s=100.1,
        runtime_clearance_certificates={'R2': certificate},
    )
    goal = TaskGoal(
        goal_id='move-r4-slot2-after-r2-clearance',
        description='Move R4 to slot 2',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '2',
            'target_shuttle': 'room315_right_shuttle_4',
            'payload_filter': 'any',
        },
    )

    problem = build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
        runtime_clearance_certificates={'R2': certificate},
    )
    clearance = problem.provenance['target_blocker_clearance_plan']

    assert [
        relocation['shuttle']
        for relocation in clearance['ordered_relocations']
    ] == ['right_shuttle_1']
    destination = clearance['ordered_relocations'][0]['destination']
    assert destination['target_s_m'] == pytest.approx(0.38)
    assert abs(destination['target_s_m'] - 0.95) >= (
        destination['required_center_spacing_m']
    )


def test_r4_slot4_to_slot2_receding_horizon_preserves_two_blocker_capacity():
    """Regression for sequential holding capacity and stale visual geometry."""

    goal = TaskGoal(
        goal_id='live-r4-slot4-to-slot2-two-blocker-capacity',
        description='Move R4 to right slot 2',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '2',
            'target_shuttle': 'room315_right_shuttle_4',
            'selection_strategy': 'explicit',
            'payload_filter': 'any',
        },
    )
    initial_state = VisualObservedStateBuilder().build(
        _snapshot(observation=_observation(
            present={'R1': '1', 'R2': '2', 'R3': '3', 'R4': '4'}
        )),
        now_s=100.1,
    )

    first_problem = build_pddl_problem_from_observed_state_task_goal(
        initial_state,
        goal,
    )
    first_clearance = first_problem.provenance[
        'target_blocker_clearance_plan'
    ]
    first_relocation = first_clearance['ordered_relocations'][0]

    assert first_clearance['observed_blockers'] == [
        'right_shuttle_2',
        'right_shuttle_1',
    ]
    assert first_clearance['deferred_blockers_require_fresh_reobservation'] == [
        'right_shuttle_1',
    ]
    assert first_relocation['shuttle'] == 'right_shuttle_2'
    assert first_relocation['destination']['target_s_m'] == pytest.approx(
        public_rail_segment_lengths('right')['A34I'] - 0.35
    )

    # Reproduce the accepted frame after the first supervised relocation.  A
    # sensor certificate may retain A34I even when the learned visual segment
    # still disagrees as A34E, exactly as in the live failure report.
    after_first_observation = _observation(
        present={'R1': '1', 'R2': '2', 'R3': '3', 'R4': '4'}
    )
    r2_after_first = next(
        item
        for item in after_first_observation['shuttles']
        if item['identity'] == 'R2'
    )
    exterior_length = public_rail_segment_lengths('right')['A34E']
    r2_after_first.update({
        'block': 'A34E',
        's_m': 0.7369174957275391,
        's_ratio': 0.7369174957275391 / exterior_length,
        'segment_length_m': exterior_length,
    })
    supervisor = _supervisor()
    supervisor['rails']['right']['switches'].update({
        'A3': 'I',
        'A4': 'I',
    })
    supervisor['rails']['right']['stoppers']['A4'] = '1'
    first_target_s_m = first_relocation['destination']['target_s_m']
    certificate = _clearance_certificate(
        'R2',
        target_s_m=first_target_s_m,
    )
    after_first_state = VisualObservedStateBuilder().build(
        _snapshot(
            observation=after_first_observation,
            supervisor=supervisor,
        ),
        now_s=100.1,
        runtime_clearance_certificates={'R2': certificate},
    )

    second_problem = build_pddl_problem_from_observed_state_task_goal(
        after_first_state,
        goal,
        runtime_clearance_certificates={'R2': certificate},
    )
    second_relocation = second_problem.provenance[
        'target_blocker_clearance_plan'
    ]['ordered_relocations'][0]
    second_destination = second_relocation['destination']

    assert second_relocation['shuttle'] == 'right_shuttle_1'
    assert second_destination['kind'] == 'interior_loop'
    # The raw learned label still says A34E, but its segment-local ratio must
    # not be projected onto the certificate-proven A34I occupancy interval.
    assert r2_after_first['block'] == 'A34E'
    assert second_destination['target_s_m'] < first_target_s_m
    assert abs(
        second_destination['target_s_m']
        - first_relocation['destination']['target_s_m']
    ) >= second_destination['required_center_spacing_m']

    # A process that already staged the first shuttle at the legacy midpoint
    # must remain fail-closed.  Source changes cannot manufacture physical
    # clearance around an already occupied pose.
    legacy_certificate = _clearance_certificate('R2', target_s_m=0.7083)
    legacy_state = VisualObservedStateBuilder().build(
        _snapshot(
            observation=after_first_observation,
            supervisor=supervisor,
        ),
        now_s=100.1,
        runtime_clearance_certificates={'R2': legacy_certificate},
    )
    with pytest.raises(
        PddlProblemBuildError,
        match='no_reachable_physically_separated_interior_holding_pose',
    ):
        build_pddl_problem_from_observed_state_task_goal(
            legacy_state,
            goal,
            runtime_clearance_certificates={'R2': legacy_certificate},
        )


def test_sequential_cutoff_parks_target_slot_blocker_in_free_exterior_slot():
    state, certificates, goal = _sequential_cutoff_state_and_certificates()

    # The two already-staged shuttles remain learned visual facts. Their
    # deterministic effect certificates suppress false exterior occupancy;
    # they do not replace localization with controller position fields.
    for identity, target_s_m in {'R1': 0.35, 'R2': 0.95}.items():
        entity = f'room315_right_shuttle_{identity[-1]}'
        position = _fact(state, entity, 'rail_position')
        assert position.source == 'state_fuser'
        assert position.metadata['selected_source'] == 'visual_model'
        assert position.value['segment'] == 'A34I'
        assert position.value['s_m'] == pytest.approx(target_s_m)
        certificate = _fact(state, entity, 'runtime_route_clearance')
        assert certificate.metadata['selected_source'] == 'executor'
        assert certificate.value['model_prediction_replaced'] is False
        assert (
            certificate.value[
                'controller_position_fields_used_for_localization'
            ]
            is False
        )
    assert all(
        'CONTROLLER_POSITION_MUST_NOT_BE_USED' not in str(fact.value)
        for fact in state.fused_planner_state
    )
    assert _fact(state, 'right:slot:3', 'occupancy').value['occupied'] is False
    assert _fact(state, 'right:slot:4', 'occupancy').value['occupied'] is False

    # This used to fail before planning because all three A34I staging poses
    # were exhausted. With slots 3/4 free, R4 (the slot-2 occupant) must be
    # parked outside instead. The recovery-aware policy deliberately keeps
    # slot 3 free for subsequent staged-shuttle recovery.
    problem = build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
        runtime_clearance_certificates=certificates,
    )
    clearance = problem.provenance['target_blocker_clearance_plan']
    assert clearance['required'] is True
    assert clearance['unsupported_if_more_than_two_blockers'] is False
    assert clearance['exterior_slot_relocations_precede_interior_clearance'] is True
    assert len(clearance['ordered_relocations']) == 1
    relocation = clearance['ordered_relocations'][0]
    assert relocation['shuttle'] == 'right_shuttle_4'
    assert relocation['reason'] == 'occupies_goal_target_slot'
    assert relocation['destination'] == {
        'kind': 'slot',
        'source_slot': 'right_slot_2',
        'target_slot': 'right_slot_4',
        'target_sensor': 'DZI4R',
        'selection_policy': (
            'recovery_aware_highest_reachable_free_exterior_slot'
        ),
    }
    assert all(
        relocation['destination'].get('kind') != 'unavailable'
        for relocation in clearance['ordered_relocations']
    )
    certified = problem.provenance['route_clearance'][
        'sensor_certified_interior_clearances'
    ]
    assert {item['shuttle'] for item in certified} == {
        'right_shuttle_1',
        'right_shuttle_2',
    }
    assert all(
        item['controller_position_fields_used_for_localization'] is False
        for item in certified
    )


def test_live_r3_slot1_to_slot2_uses_sensor_anchored_occupancy_not_visual_hull():
    """Replay the post-arrival bias that falsely exhausted A34I capacity."""

    goal = _sequential_cutoff_state_and_certificates()[2]
    interior_length = public_rail_segment_lengths('right')['A34I']
    exterior_length = public_rail_segment_lengths('right')['A12E']
    slot_1 = _planning_rail_topology('right').slots['1']
    slot_2 = _planning_rail_topology('right').slots['2']
    raw_r3_ratio = slot_1.s_ratio + 0.10
    clearance_certificates = {
        identity: _clearance_certificate(identity, target_s_m=target_s_m)
        for identity, target_s_m in {'R1': 0.35, 'R2': 0.95}.items()
    }

    def accepted_state(*, r4_slot: str):
        observation = _observation(present={
            'R1': '3',
            'R2': '4',
            'R3': '1',
            'R4': r4_slot,
        })
        for item in observation['shuttles']:
            if item['identity'] in {'R1', 'R2'}:
                target_s_m = {'R1': 0.35, 'R2': 0.95}[item['identity']]
                item.update({
                    'block': 'A34I',
                    's_m': target_s_m,
                    's_ratio': target_s_m / interior_length,
                    'segment_length_m': interior_length,
                })
            elif item['identity'] == 'R3':
                # The learned value remains within the accepted same-segment
                # consistency window, but extending a safety hull through it
                # reaches the slot-2 route and recreates the reported abort.
                item.update({
                    'block': 'A12E',
                    's_m': raw_r3_ratio * exterior_length,
                    's_ratio': raw_r3_ratio,
                    'segment_length_m': exterior_length,
                })
        anchored_slots = {'R3': '1', 'R4': r4_slot}
        return VisualObservedStateBuilder().build(
            _snapshot(observation=observation),
            now_s=100.1,
            runtime_clearance_certificates=clearance_certificates,
            slot_sensor_anchors={
                identity: {
                    'identity': identity,
                    'side': 'right',
                    'slot': slot,
                    'sensor': f'DZI{slot}R',
                }
                for identity, slot in anchored_slots.items()
            },
            verified_slot_arrival_certificates={
                identity: _verified_slot_arrival_certificate(
                    identity,
                    slot=slot,
                )
                for identity, slot in anchored_slots.items()
            },
        )

    blocked_state = accepted_state(r4_slot='2')
    raw_position = _fact(
        blocked_state,
        'room315_right_shuttle_3',
        'rail_position',
    )
    assert raw_position.value['s_ratio'] == pytest.approx(raw_r3_ratio)
    first_problem = build_pddl_problem_from_observed_state_task_goal(
        blocked_state,
        goal,
        runtime_clearance_certificates=clearance_certificates,
    )
    clearance = first_problem.provenance['target_blocker_clearance_plan']
    assert clearance['ordered_relocations'][0]['shuttle'] == (
        'right_shuttle_4'
    )
    assert clearance['ordered_relocations'][0]['destination'] == {
        'kind': 'slot',
        'source_slot': 'right_slot_2',
        'target_slot': 'right_slot_4',
        'target_sensor': 'DZI4R',
        'selection_policy': (
            'recovery_aware_highest_reachable_free_exterior_slot'
        ),
    }
    r3_anchor = next(
        item
        for item in first_problem.provenance['route_clearance'][
            'verified_slot_arrival_anchors'
        ]
        if item['shuttle'] == 'right_shuttle_3'
    )
    assert r3_anchor['raw_visual_s_ratio'] == pytest.approx(raw_r3_ratio)
    assert r3_anchor['canonical_slot_s_ratio'] == pytest.approx(
        slot_1.s_ratio
    )
    assert r3_anchor['occupancy_end_s_ratio'] < slot_2.s_ratio
    assert r3_anchor['occupancy_source'] == (
        'verified_identity_bearing_slot_sensor'
    )
    assert r3_anchor['raw_visual_s_ratio_used_for_occupancy'] is False

    # After the supervised R4 slot-2 -> slot-4 move and fresh observation,
    # R3's exact slot-1 -> slot-2 route must be directly clear; no third A34I
    # pose is requested and the user's original goal can complete.
    parked_state = accepted_state(r4_slot='4')
    final_problem = build_pddl_problem_from_observed_state_task_goal(
        parked_state,
        goal,
        runtime_clearance_certificates=clearance_certificates,
    )
    final_clearance = final_problem.provenance[
        'target_blocker_clearance_plan'
    ]
    assert final_clearance['required'] is False
    assert final_clearance['ordered_relocations'] == []


def test_live_green_slot1_goal_moves_blue_before_yellow_without_collision():
    """Replay R4@1, R2@2, R3@3: vacancy must propagate safely."""

    goal = TaskGoal(
        goal_id='live-green-to-right-slot1-blue-before-yellow',
        description='Move green R3 to right slot 1',
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
            'target_shuttle': 'room315_right_shuttle_3',
            'target_slot': '1',
        },
    )
    interior_length = public_rail_segment_lengths('right')['A34I']
    clearance_certificates = {
        'R1': _clearance_certificate('R1', target_s_m=0.35),
    }

    def accepted_state(*, r2_slot: str, r3_slot: str, r4_slot: str):
        observation = _observation(present={
            'R1': '3',
            'R2': r2_slot,
            'R3': r3_slot,
            'R4': r4_slot,
        })
        r1 = next(
            item
            for item in observation['shuttles']
            if item['identity'] == 'R1'
        )
        r1.update({
            'block': 'A34I',
            's_m': 0.35,
            's_ratio': 0.35 / interior_length,
            'segment_length_m': interior_length,
        })
        anchored_slots = {
            'R2': r2_slot,
            'R3': r3_slot,
            'R4': r4_slot,
        }
        return VisualObservedStateBuilder().build(
            _snapshot(observation=observation),
            now_s=100.1,
            runtime_clearance_certificates=clearance_certificates,
            slot_sensor_anchors={
                identity: {
                    'identity': identity,
                    'side': 'right',
                    'slot': slot,
                    'sensor': f'DZI{slot}R',
                }
                for identity, slot in anchored_slots.items()
            },
            verified_slot_arrival_certificates={
                identity: _verified_slot_arrival_certificate(
                    identity,
                    slot=slot,
                )
                for identity, slot in anchored_slots.items()
            },
        )

    # First, the selected green shuttle may safely advance from slot 3 to the
    # free slot 4 while the final slot remains occupied.
    initial = accepted_state(r2_slot='2', r3_slot='3', r4_slot='1')
    initial_problem = build_pddl_problem_from_observed_state_task_goal(
        initial,
        goal,
        runtime_clearance_certificates=clearance_certificates,
    )
    advance = initial_problem.provenance['target_blocker_clearance_plan'][
        'intermediate_selected_advance'
    ]
    assert advance['source_slot'] == 'right_slot_3'
    assert advance['target_slot'] == 'right_slot_4'

    # Exact failure state from the report. Yellow R4 cannot be sent from slot
    # 1 toward A3 because blue R2 occupies slot 2. The first clearance action
    # must propagate the free slot 3 backward by moving blue into it.
    after_green_advance = accepted_state(
        r2_slot='2',
        r3_slot='4',
        r4_slot='1',
    )
    dependency_problem = build_pddl_problem_from_observed_state_task_goal(
        after_green_advance,
        goal,
        runtime_clearance_certificates=clearance_certificates,
    )
    dependency_clearance = dependency_problem.provenance[
        'target_blocker_clearance_plan'
    ]
    first_relocation = dependency_clearance['ordered_relocations'][0]
    assert first_relocation['shuttle'] == 'right_shuttle_2'
    assert first_relocation['reason'] == 'blocks_target_occupant_relocation'
    assert first_relocation['destination'] == {
        'kind': 'slot',
        'source_slot': 'right_slot_2',
        'target_slot': 'right_slot_3',
        'target_sensor': 'DZI3R',
        'selection_policy': 'nearest_forward_vacancy_dependency_step',
    }
    assert dependency_clearance['vacancy_dependency_resolution'][
        'interior_relocation_of_goal_occupant_prevented'
    ] is True
    dependency_subproblem = build_first_blocker_clearance_problem(
        dependency_problem
    )
    assert dependency_subproblem.goal_text == (
        '(shuttle_at_slot right_shuttle_2 right_slot_3)'
    )

    # Re-observation after blue reaches slot 3 exposes only the minimum
    # yellow move: slot 1 -> newly free slot 2. No interior motion is needed.
    after_blue = accepted_state(r2_slot='3', r3_slot='4', r4_slot='1')
    yellow_problem = build_pddl_problem_from_observed_state_task_goal(
        after_blue,
        goal,
        runtime_clearance_certificates=clearance_certificates,
    )
    yellow_relocation = yellow_problem.provenance[
        'target_blocker_clearance_plan'
    ]['ordered_relocations'][0]
    assert yellow_relocation['shuttle'] == 'right_shuttle_4'
    assert yellow_relocation['destination']['kind'] == 'slot'
    assert yellow_relocation['destination']['source_slot'] == 'right_slot_1'
    assert yellow_relocation['destination']['target_slot'] == 'right_slot_2'

    # Once yellow is just far enough away, the user's green-to-slot-1 goal is
    # clear and receives no extra blocker relocation.
    after_yellow = accepted_state(r2_slot='3', r3_slot='4', r4_slot='2')
    final_problem = build_pddl_problem_from_observed_state_task_goal(
        after_yellow,
        goal,
        runtime_clearance_certificates=clearance_certificates,
    )
    final_clearance = final_problem.provenance[
        'target_blocker_clearance_plan'
    ]
    assert final_clearance['required'] is False
    assert final_clearance['ordered_relocations'] == []
    all_destinations = [
        first_relocation['destination'],
        yellow_relocation['destination'],
    ]
    assert all(
        destination['kind'] == 'slot'
        for destination in all_destinations
    )


@pytest.mark.parametrize('side', ('right', 'left'))
def test_full_rail_moves_clearance_dependency_before_goal_slot_blocker(side):
    """Use topology to move the shuttle ahead when no exterior slot is free."""

    state = VisualObservedStateBuilder().build(
        _snapshot(observation=_observation(present={
            f'{side[0].upper()}1': '3',
            f'{side[0].upper()}2': '2',
            f'{side[0].upper()}3': '4',
            f'{side[0].upper()}4': '1',
        })),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='no-unsafe-yellow-through-blue',
        description=f'Move shuttle 3 to {side} slot 1',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'payload_filter': 'any',
            'selection_strategy': 'explicit',
            'shuttle_selection': 'explicit',
            'side': side,
            'target_kind': 'slot',
            'target_shuttle': f'room315_{side}_shuttle_3',
            'target_slot': '1',
        },
    )

    problem = build_pddl_problem_from_observed_state_task_goal(state, goal)
    relocation = problem.provenance['target_blocker_clearance_plan'][
        'ordered_relocations'
    ][0]
    assert relocation['shuttle'] == f'{side}_shuttle_2'
    assert relocation['reason'] == 'blocks_clearance_dependency'
    assert relocation['destination']['kind'] == 'interior_loop'
    route_proof = relocation['destination']['interior_entry_route_proof']
    assert route_proof['status'] == 'clear'
    assert route_proof['method'] == (
        'authoritative_topology_dependency_search_v2'
    )
    assert route_proof['dependency_chain'] == [
        f'{side}_shuttle_4',
        f'{side}_shuttle_2',
    ]
    assert route_proof['blocking_shuttles'] == []
    assert route_proof['required_switches'] == {
        'A1': 'E', 'A2': 'E', 'A3': 'I', 'A4': 'I',
    }
    assert (
        f'(interior_entry_route_clear {side}_shuttle_2)'
        in problem.problem_text
    )
    clearance_problem = ClosedLoopExecutive._next_planning_problem(problem)
    assert clearance_problem.goal_text == (
        f'(clearance_relocated {side}_shuttle_2)'
    )
    resolution = problem.provenance['target_blocker_clearance_plan'][
        'clearance_dependency_resolution'
    ]
    assert resolution['first_action_only'] is True
    assert resolution['fresh_reobservation_required_after_action'] is True
    assert resolution['controller_position_fields_used_for_localization'] is False


@pytest.mark.parametrize('side', ('right', 'left'))
def test_full_slot2_to_slot4_clears_slot4_then_slot3_through_a12i(side):
    """Use the complete opposite interior branch for two forward blockers."""

    prefix = side[0].upper()
    goal = TaskGoal(
        goal_id=f'{side}-full-slot2-to-slot4',
        description=f'Move shuttle 2 to {side} slot 4',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'payload_filter': 'any',
            'selection_strategy': 'explicit',
            'shuttle_selection': 'explicit',
            'side': side,
            'target_kind': 'slot',
            'target_shuttle': f'room315_{side}_shuttle_2',
            'target_slot': '4',
        },
    )
    present = {f'{prefix}{identity}': str(identity) for identity in range(1, 5)}

    def certificate(identity: int, target_s_m: float) -> dict:
        suffix = 'R' if side == 'right' else 'L'
        return {
            'identity': f'{prefix}{identity}',
            'shuttle': f'{side}_shuttle_{identity}',
            'side': side,
            'target_segment': 'A12I',
            'target_s_m': target_s_m,
            'observed_segment': 'A12I',
            'observed_s_m': target_s_m,
            'absolute_error_m': 0.0,
            'entry_sensor': f'DA1I{suffix}',
            'matched_by': 'interior_entry_sensor_plus_bounded_travel_time',
            'entry_sensor_identity_confirmed': True,
            'controller_stop_confirmed': True,
            'post_stop_visual_frame_received': True,
            'post_stop_visual_confirmation': True,
            'bounded_commanded_motion_completed': True,
            'clearance_mode_held': True,
            'normal_route_restored': False,
            'model_prediction_replaced': False,
            'controller_position_fields_used_for_localization': False,
        }

    def accepted_state(staged: dict[int, float]):
        observation = _observation(present=present)
        length_m = public_rail_segment_lengths(side)['A12I']
        for identity, target_s_m in staged.items():
            item = next(
                shuttle
                for shuttle in observation['shuttles']
                if shuttle['identity'] == f'{prefix}{identity}'
            )
            item.update({
                'block': 'A12I',
                's_m': target_s_m,
                's_ratio': target_s_m / length_m,
                'segment_length_m': length_m,
            })
        supervisor = _supervisor()
        supervisor['rails'][side]['switches'] = {
            'A1': 'I', 'A2': 'I', 'A3': 'E', 'A4': 'E',
        }
        supervisor['rails'][side]['stoppers'] = {
            'A1': '0', 'A2': '1', 'A3': '0', 'A4': '0',
        }
        certificates = {
            f'{prefix}{identity}': certificate(identity, target_s_m)
            for identity, target_s_m in staged.items()
        }
        state = VisualObservedStateBuilder().build(
            _snapshot(observation=observation, supervisor=supervisor),
            now_s=100.1,
            runtime_clearance_certificates=certificates,
        )
        return state, certificates

    initial = VisualObservedStateBuilder().build(
        _snapshot(observation=_observation(present=present)),
        now_s=100.1,
    )
    first_problem = build_pddl_problem_from_observed_state_task_goal(
        initial,
        goal,
    )
    first_clearance = first_problem.provenance[
        'target_blocker_clearance_plan'
    ]
    first = first_clearance['ordered_relocations'][0]
    assert first_clearance['observed_blockers'] == [
        f'{side}_shuttle_4',
        f'{side}_shuttle_3',
    ]
    assert first['shuttle'] == f'{side}_shuttle_4'
    assert first['destination']['gate_switch'] == 'A1'
    assert first['destination']['exit_switch'] == 'A2'
    assert first['destination']['target_segment'] == 'A12I'
    assert first['destination']['interior_entry_route_proof'][
        'required_switches'
    ] == {'A1': 'I', 'A2': 'I', 'A3': 'E', 'A4': 'E'}
    assert first_clearance['clearance_branch_search']['selected_gate'] == 'A1'

    first_s_m = float(first['destination']['target_s_m'])
    after_first, first_certificates = accepted_state({4: first_s_m})
    second_problem = build_pddl_problem_from_observed_state_task_goal(
        after_first,
        goal,
        runtime_clearance_certificates=first_certificates,
    )
    second = second_problem.provenance['target_blocker_clearance_plan'][
        'ordered_relocations'
    ][0]
    assert second['shuttle'] == f'{side}_shuttle_3'
    assert second['destination']['gate_switch'] == 'A1'
    assert second['destination']['target_segment'] == 'A12I'
    second_s_m = float(second['destination']['target_s_m'])
    assert first_s_m - second_s_m >= second['destination'][
        'required_center_spacing_m'
    ]

    after_second, both_certificates = accepted_state({
        3: second_s_m,
        4: first_s_m,
    })
    final_problem = build_pddl_problem_from_observed_state_task_goal(
        after_second,
        goal,
        runtime_clearance_certificates=both_certificates,
    )
    final_clearance = final_problem.provenance[
        'target_blocker_clearance_plan'
    ]
    assert final_clearance['required'] is False
    assert final_clearance['ordered_relocations'] == []
    assert '(clearance_mode ' + side + ')' in final_problem.problem_text
    assert '(normal_route ' + side + ')' not in final_problem.problem_text


def test_live_sequential_cutoff_routes_around_unrelated_left_anchor_mismatch():
    """Replay the complete attached state that previously died pre-planning."""

    state, certificates, goal = _sequential_cutoff_state_and_certificates(
        include_unrelated_left_anchor_mismatch=True,
    )

    problem = build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
        runtime_clearance_certificates=certificates,
    )
    scope = problem.provenance['planning_scope']
    assert scope['goal_side'] == 'right'
    assert scope['deferred_out_of_scope_location_issues'] == [{
        'shuttle': 'left_shuttle_2',
        'side': 'left',
        'reason': (
            'exact slot anchor and visual s_ratio disagree for '
            "'left_shuttle_2': error 0.121590135 exceeds 0.120000000"
        ),
    }]

    clearance = problem.provenance['target_blocker_clearance_plan']
    assert clearance['required'] is True
    assert clearance['ordered_relocations'][0] == {
        'order': 1,
        'shuttle': 'right_shuttle_4',
        'reason': 'occupies_goal_target_slot',
        'current_segment': 'A12E',
        'current_s_ratio': 0.653074,
        'destination': {
            'kind': 'slot',
            'source_slot': 'right_slot_2',
            'target_slot': 'right_slot_4',
            'target_sensor': 'DZI4R',
            'selection_policy': (
                'recovery_aware_highest_reachable_free_exterior_slot'
            ),
        },
    }
    isolated = build_first_blocker_clearance_problem(problem)
    assert isolated.provenance['planning_phase'] == 'clear_blocker_to_slot'
    assert isolated.selected_shuttle == 'right_shuttle_4'
    assert isolated.goal_text == '(shuttle_at_slot right_shuttle_4 right_slot_4)'
    assert isolated.provenance['parking_target_free_fact_retained'] == (
        'right_slot_4'
    )


def test_live_any_loaded_slot3_replay_builds_normalize_then_parking_subgoal():
    """Regression for the attached post-A34I ``any loaded`` Plan-not-found."""

    observation = _observation(
        present={'R1': '3', 'R2': '1', 'R3': '3', 'R4': '2'}
    )
    length = public_rail_segment_lengths('right')['A34I']
    r1 = next(
        item for item in observation['shuttles']
        if item['identity'] == 'R1'
    )
    r1.update({
        'block': 'A34I',
        's_m': 0.35,
        's_ratio': 0.35 / length,
        'segment_length_m': length,
    })
    supervisor = _supervisor()
    supervisor['rails']['right']['switches']['A4'] = 'I'
    certificates = {
        'R1': _clearance_certificate('R1', target_s_m=0.35),
    }
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=observation, supervisor=supervisor),
        now_s=100.1,
        runtime_clearance_certificates=certificates,
    )
    goal = TaskGoal(
        goal_id='replay-any-loaded-right-slot3',
        description='Move any loaded shuttle to right slot 3',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '3',
            'selection_strategy': 'any',
            'shuttle_selection': 'loaded',
            'payload_filter': 'loaded',
            'payload_required': True,
        },
    )

    parent = build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
        runtime_clearance_certificates=certificates,
    )
    clearance = parent.provenance['target_blocker_clearance_plan']
    normalization = parent.provenance['route_normalization']['by_side'][
        'right'
    ]
    isolated = build_first_blocker_clearance_problem(parent)

    assert parent.selected_shuttle == 'right_shuttle_4'
    assert parent.provenance['eligible_candidate_shuttles'] == [
        'right_shuttle_4'
    ]
    assert normalization['reconfiguration_required'] is True
    assert normalization['reconfiguration_safe'] is True
    assert normalization['certified_stopped_interior_shuttles'] == [
        'right_shuttle_1'
    ]
    assert clearance['ordered_relocations'] == [{
        'order': 1,
        'shuttle': 'right_shuttle_3',
        'reason': 'occupies_goal_target_slot',
        'current_segment': 'A34E',
        'current_s_ratio': pytest.approx(0.447469),
        'destination': {
            'kind': 'slot',
            'source_slot': 'right_slot_3',
            'target_slot': 'right_slot_4',
            'target_sensor': 'DZI4R',
            'selection_policy': (
                'recovery_aware_highest_reachable_free_exterior_slot'
            ),
        },
    }]
    assert isolated.goal_text == (
        '(shuttle_at_slot right_shuttle_3 right_slot_4)'
    )
    assert '(route_reconfiguration_required right)' in isolated.problem_text
    assert '(route_reconfiguration_safe right)' in isolated.problem_text
    assert '(slot_free right_slot_4)' in isolated.problem_text


def test_segment_clearance_must_finish_before_topology_route_setup():
    """A fresh post-relocation plan cannot bypass the clearance lifecycle."""

    observation = _observation(present={'R2': '4', 'R4': '1'})
    for item in observation['shuttles']:
        if item['identity'] == 'R4':
            length = public_rail_segment_lengths('right')['A23']
            item.update({
                'block': 'A23',
                's_m': 0.2 * length,
                's_ratio': 0.2,
                'segment_length_m': length,
            })
        elif item['identity'] == 'R2':
            length = public_rail_segment_lengths('right')['A34I']
            item.update({
                'block': 'A34I',
                's_m': 0.7083,
                's_ratio': 0.7083 / length,
                'segment_length_m': length,
            })
    supervisor = _supervisor()
    supervisor['rails']['right']['switches'].update({
        'A3': 'I',
        'A4': 'I',
    })
    supervisor['rails']['right']['stoppers']['A4'] = '1'
    certificates = {
        'R2': _clearance_certificate('R2', target_s_m=0.7083),
    }
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=observation, supervisor=supervisor),
        now_s=100.1,
        runtime_clearance_certificates=certificates,
    )
    goal = TaskGoal(
        goal_id='finish-segment-clearance-before-r4-slot2',
        description='Move R4 from A23 to right slot 2',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '2',
            'target_shuttle': 'room315_right_shuttle_4',
            'selection_strategy': 'explicit',
            'payload_filter': 'any',
        },
    )
    problem = build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
        runtime_clearance_certificates=certificates,
    )

    prepare = translate_plan([
        'prepare_topology_route right_shuttle_4 right '
        'right_topology_a23 right_slot_2 right_switch_group'
    ])[0]
    finish = translate_plan([
        'finish_segment_route_clearance right_shuttle_4 right '
        'right_topology_a23 right_slot_2'
    ])[0]

    assert '(clearance_mode right)' in problem.problem_text
    assert '(clearance_pause_safe right)' in problem.problem_text
    assert '(normal_route right)' not in problem.problem_text
    assert ClosedLoopExecutive._first_action_contract_error(
        first_step=prepare.pddl_step,
        translated_step=prepare,
        problem=problem,
        task_goal=goal,
    ) == 'missing_frozen_precondition:normal_route:right'
    assert ClosedLoopExecutive._first_action_contract_error(
        first_step=finish.pddl_step,
        translated_step=finish,
        problem=problem,
        task_goal=goal,
    ) == ''


def test_certified_r2_a34i_builds_authoritative_topology_route_to_slot_1():
    observation = _observation(
        present={'R1': '3', 'R2': '4', 'R3': '2', 'R4': '4'}
    )
    length = public_rail_segment_lengths('right')['A34I']
    certificates = {}
    for identity, s_m in {'R1': 0.35, 'R2': 0.95}.items():
        item = next(
            shuttle for shuttle in observation['shuttles']
            if shuttle['identity'] == identity
        )
        item.update({
            'block': 'A34I',
            's_m': s_m,
            's_ratio': s_m / length,
            'segment_length_m': length,
        })
        certificates[identity] = _clearance_certificate(
            identity,
            target_s_m=s_m,
        )
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=observation),
        now_s=100.1,
        runtime_clearance_certificates=certificates,
    )
    goal = TaskGoal(
        goal_id='recover-r2-a34i-slot1',
        description='Move R2 from A34I to right slot 1',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '1',
            'target_shuttle': 'room315_right_shuttle_2',
            'payload_filter': 'any',
        },
    )

    problem = build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
        runtime_clearance_certificates=certificates,
    )

    assert not any(
        fact.subject == 'room315_right_shuttle_2'
        and fact.predicate == 'location_slot'
        for fact in state.fused_planner_state
    )
    route = problem.provenance['topology_routes']['routes'][0]
    assert route['shuttle'] == 'right_shuttle_2'
    assert route['source_kind'] == 'accepted_visual_continuous_position'
    assert route['source_public_segment'] == 'A34I'
    assert route['target_slot_object'] == 'right_slot_1'
    assert route['target_sensor'] == 'DZI1R'
    assert route['required_switches'] == {
        'A1': 'E',
        'A2': 'E',
        'A3': 'E',
        'A4': 'I',
    }
    assert [block['public_segment'] for block in route['route_blocks']] == [
        'A34I',
        'A4I',
        'A14',
        'A1E',
        'A12E',
    ]
    assert route['blockers'] == []
    assert route['route_clear'] is True
    assert route['controller_position_fields_used_for_localization'] is False
    assert route['runtime_clearance_visual_consistency'] == {
        'required': True,
        'satisfied': True,
        'certificate_target_public_segment': 'A34I',
        'certificate_target_internal_segment': 'A34I',
        'accepted_visual_internal_segment': 'A34I',
        'certificate_used_as_localization': False,
    }
    assert (
        '(shuttle_at_topology_block right_shuttle_2 right_topology_a34i)'
        in problem.problem_text
    )
    assert (
        '(topology_route_clear right_shuttle_2 '
        'right_topology_a34i right_slot_1)'
        in problem.problem_text
    )


def test_certified_interior_effect_recovers_from_visual_segment_disagreement():
    observation = _observation(present={'R2': '4'})
    item = next(
        shuttle for shuttle in observation['shuttles']
        if shuttle['identity'] == 'R2'
    )
    length = public_rail_segment_lengths('right')['A34E']
    item.update({
        'block': 'A34E',
        's_m': 0.95,
        's_ratio': 0.95 / length,
        'segment_length_m': length,
    })
    certificate = _clearance_certificate('R2', target_s_m=0.95)
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=observation),
        now_s=100.1,
        runtime_clearance_certificates={'R2': certificate},
    )
    goal = TaskGoal(
        goal_id='reject-r2-a34e-certificate-disagreement',
        description='Move R2 to right slot 1',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '1',
            'target_shuttle': 'room315_right_shuttle_2',
            'payload_filter': 'any',
        },
    )

    problem = build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
        runtime_clearance_certificates={'R2': certificate},
    )
    route = problem.provenance['topology_routes']['routes'][0]

    assert route['source_public_segment'] == 'A34I'
    assert route['route_clear'] is True
    assert route['runtime_clearance_visual_consistency'] == {
        'required': True,
        'satisfied': False,
        'certificate_target_public_segment': 'A34I',
        'certificate_target_internal_segment': 'A34I',
        'accepted_visual_internal_segment': 'A34E',
        'certificate_used_as_localization': False,
        'certificate_used_as_persisted_execution_effect': True,
        'planning_origin_segment': 'A34I',
        'raw_visual_prediction_preserved': True,
        'reason': 'certificate_and_visual_segment_disagree',
    }
    raw = _fact(state, 'room315_right_shuttle_2', 'rail_position')
    assert raw.value['segment'] == 'A34E'
    assert raw.value['s_ratio'] == pytest.approx(0.95 / length)


def test_live_blue_a34i_to_slot1_does_not_request_third_interior_pose():
    """Replay the exact state after the successful yellow-to-slot-2 goal."""

    observation = _observation(present={
        'R1': '1',
        'R2': '2',
        'R3': '3',
        'R4': '2',
    })
    interior_length = public_rail_segment_lengths('right')['A34I']
    exterior_length = public_rail_segment_lengths('right')['A34E']
    for item in observation['shuttles']:
        if item['identity'] == 'R1':
            item.update({
                'block': 'A34I',
                's_m': 0.4400593936443329,
                's_ratio': 0.4400593936443329 / interior_length,
                'segment_length_m': interior_length,
            })
        elif item['identity'] == 'R2':
            # The accepted model frame retained the parallel A34E class after
            # the entry-sensor/bounded-motion effect put R2 at A34I@0.95.
            item.update({
                'block': 'A34E',
                's_m': 0.9196587800979614,
                's_ratio': 0.9196587800979614 / exterior_length,
                'segment_length_m': exterior_length,
            })
    certificates = {
        'R1': _clearance_certificate('R1', target_s_m=0.35),
        'R2': _clearance_certificate('R2', target_s_m=0.95),
    }
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=observation),
        now_s=100.1,
        runtime_clearance_certificates=certificates,
        slot_sensor_anchors={
            'R4': {
                'identity': 'R4',
                'side': 'right',
                'slot': '2',
                'sensor': 'DZI2R',
            },
        },
        verified_slot_arrival_certificates={
            'R4': _verified_slot_arrival_certificate('R4', slot='2'),
        },
    )
    goal = TaskGoal(
        goal_id='live-blue-a34i-to-slot1-after-yellow-slot2',
        description='Move blue R2 to right slot 1',
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
            'target_shuttle': 'room315_right_shuttle_2',
            'target_slot': '1',
        },
    )

    problem = build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
        runtime_clearance_certificates=certificates,
    )
    clearance = problem.provenance['target_blocker_clearance_plan']
    route = problem.provenance['topology_routes']['routes'][0]

    assert clearance['required'] is False
    assert clearance['ordered_relocations'] == []
    assert route['source_public_segment'] == 'A34I'
    assert route['source_s_ratio'] == pytest.approx(0.95 / interior_length)
    assert route['blockers'] == []
    assert route['route_clear'] is True
    assert route['runtime_clearance_visual_consistency']['satisfied'] is False
    assert (
        route['runtime_clearance_visual_consistency'][
            'certificate_used_as_persisted_execution_effect'
        ]
        is True
    )
    assert (
        '(topology_route_clear right_shuttle_2 '
        'right_topology_a34i right_slot_1)'
        in problem.problem_text
    )
    raw = _fact(state, 'room315_right_shuttle_2', 'rail_position')
    assert raw.value['segment'] == 'A34E'
    assert raw.value['s_m'] == pytest.approx(0.9196587800979614)


def test_public_left_a34i_maps_through_authoritative_topology_to_slot_1():
    observation = _observation(present={'L2': '4'})
    item = next(
        shuttle for shuttle in observation['shuttles']
        if shuttle['identity'] == 'L2'
    )
    length = public_rail_segment_lengths('left')['A34I']
    item.update({
        'block': 'A34I',
        's_m': 0.95,
        's_ratio': 0.95 / length,
        'segment_length_m': length,
    })
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=observation),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='recover-l2-a34i-slot1',
        description='Move L2 from A34I to left slot 1',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'left',
            'target_kind': 'slot',
            'target_slot': '1',
            'target_shuttle': 'room315_left_shuttle_2',
            'payload_filter': 'any',
        },
    )

    problem = build_pddl_problem_from_observed_state_task_goal(state, goal)
    route = problem.provenance['topology_routes']['routes'][0]

    assert route['source_public_segment'] == 'A34I'
    assert route['source_segment'] == 'A12I'
    assert route['required_switches'] == {
        'A1': 'E',
        'A2': 'E',
        'A3': 'E',
        'A4': 'I',
    }
    assert [block['public_segment'] for block in route['route_blocks']] == [
        'A34I',
        'A4I',
        'A14',
        'A1E',
        'A12E',
    ]
    assert route['target_sensor'] == 'DZI1L'


def test_a34i_ahead_blocker_gets_topology_parking_problem_first():
    observation = _observation(present={'R1': '3', 'R2': '4'})
    length = public_rail_segment_lengths('right')['A34I']
    certificates = {}
    for identity, s_m in {'R1': 0.35, 'R2': 0.95}.items():
        item = next(
            shuttle for shuttle in observation['shuttles']
            if shuttle['identity'] == identity
        )
        item.update({
            'block': 'A34I',
            's_m': s_m,
            's_ratio': s_m / length,
            'segment_length_m': length,
        })
        certificates[identity] = _clearance_certificate(
            identity,
            target_s_m=s_m,
        )
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=observation),
        now_s=100.1,
        runtime_clearance_certificates=certificates,
    )
    goal = TaskGoal(
        goal_id='r1-behind-r2-to-slot1',
        description='Move R1 to right slot 1',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '1',
            'target_shuttle': 'room315_right_shuttle_1',
            'payload_filter': 'any',
        },
    )

    parent = build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
        runtime_clearance_certificates=certificates,
    )
    clearance = parent.provenance['target_blocker_clearance_plan']
    isolated = build_first_blocker_clearance_problem(parent)

    assert clearance['required'] is True
    assert len(clearance['ordered_relocations']) == 1
    relocation = clearance['ordered_relocations'][0]
    assert relocation['shuttle'] == 'right_shuttle_2'
    assert relocation['destination']['kind'] == 'slot'
    assert relocation['destination']['target_slot'] == 'right_slot_2'
    assert relocation['destination']['source_kind'] == (
        'accepted_visual_continuous_position'
    )
    assert isolated.selected_shuttle == 'right_shuttle_2'
    assert isolated.target_slot == '2'
    assert isolated.provenance['planning_phase'] == 'clear_blocker_to_slot'
    assert (
        '(topology_route_clear right_shuttle_2 '
        'right_topology_a34i right_slot_2)'
        in isolated.problem_text
    )
    assert _target_slot_for_step(
        PddlPlanStep.from_text(
            'move_shuttle_from_segment_to_slot right_shuttle_2 right '
            'right_topology_a34i right_yaskawa right_slot_2'
        ),
        goal,
        isolated,
    ) == 'right:slot:2'


def test_interior_blocker_isolated_before_final_transport_plan():
    """Never make the initial plan assume a future visual relocation proof."""

    state = VisualObservedStateBuilder().build(
        _snapshot(observation=_observation(present={'R2': '2', 'R4': '4'})),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='receding-horizon-r4-slot2',
        description='Move R4 to right slot 2',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '2',
            'target_shuttle': 'room315_right_shuttle_4',
            'payload_filter': 'any',
        },
    )

    parent = build_pddl_problem_from_observed_state_task_goal(state, goal)
    relocation = parent.provenance['target_blocker_clearance_plan'][
        'ordered_relocations'
    ][0]
    assert relocation['destination']['kind'] == 'interior_loop'

    isolated = ClosedLoopExecutive._next_planning_problem(parent)

    assert isolated.goal_text == '(clearance_relocated right_shuttle_2)'
    assert isolated.provenance['planning_phase'] == (
        'clear_blocker_to_interior_loop'
    )
    assert isolated.provenance['parent_problem_name'] == parent.problem_name


def test_unknown_blocker_destination_fails_closed_before_planning():
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=_observation(present={'R2': '2', 'R4': '4'})),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='unknown-clearance-destination',
        description='Move R4 to right slot 2',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '2',
            'target_shuttle': 'room315_right_shuttle_4',
            'payload_filter': 'any',
        },
    )
    parent = build_pddl_problem_from_observed_state_task_goal(state, goal)
    provenance = dict(parent.provenance)
    clearance = dict(provenance['target_blocker_clearance_plan'])
    relocations = [dict(item) for item in clearance['ordered_relocations']]
    relocations[0]['destination'] = {'kind': 'teleport'}
    clearance['ordered_relocations'] = relocations
    provenance['target_blocker_clearance_plan'] = clearance

    with pytest.raises(
        PddlProblemBuildError,
        match='unsupported proved blocker destination.*teleport',
    ):
        ClosedLoopExecutive._next_planning_problem(
            replace(parent, provenance=provenance)
        )


def test_loaded_any_slot_goal_recovers_mixed_topology_before_blocker_move():
    """Replay the exact R2-topology -> loaded-R4 live failure."""

    observation = _observation(
        present={'R1': '4', 'R2': '1', 'R3': '3', 'R4': '2'}
    )
    length = public_rail_segment_lengths('right')['A34I']
    r1 = next(
        shuttle for shuttle in observation['shuttles']
        if shuttle['identity'] == 'R1'
    )
    r1.update({
        'block': 'A34I',
        's_m': 0.5079,
        's_ratio': 0.5079 / length,
        'segment_length_m': length,
    })
    certificate = _clearance_certificate('R1', target_s_m=0.35)
    supervisor = _supervisor()
    supervisor['rails']['right']['switches']['A4'] = 'I'
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=observation, supervisor=supervisor),
        now_s=100.1,
        runtime_clearance_certificates={'R1': certificate},
    )
    goal = TaskGoal(
        goal_id='loaded-r4-slot3-after-r2-topology-route',
        description='Move any loaded right shuttle to slot 3',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '3',
            'selection_strategy': 'any',
            'shuttle_selection': 'loaded',
            'payload_filter': 'loaded',
            'payload_required': True,
        },
    )

    parent = build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
        runtime_clearance_certificates={'R1': certificate},
    )
    isolated = build_first_blocker_clearance_problem(parent)
    normalization = isolated.provenance['route_normalization'][
        'by_side'
    ]['right']

    assert parent.selected_shuttle == 'right_shuttle_4'
    relocation = parent.provenance['target_blocker_clearance_plan'][
        'ordered_relocations'
    ][0]
    assert relocation['shuttle'] == 'right_shuttle_3'
    assert relocation['destination']['target_slot'] == 'right_slot_4'
    assert isolated.goal_text == '(shuttle_at_slot right_shuttle_3 right_slot_4)'
    assert normalization['switches']['A4'] == 'interior'
    assert normalization['normal_route'] is False
    assert normalization['reconfiguration_required'] is True
    assert normalization['reconfiguration_safe'] is True
    assert normalization['certified_stopped_interior_shuttles'] == [
        'right_shuttle_1'
    ]
    assert normalization['controller_position_fields_used_for_localization'] is False
    assert '(route_reconfiguration_required right)' in isolated.problem_text
    assert '(route_reconfiguration_safe right)' in isolated.problem_text
    assert '(normal_route right)' not in isolated.problem_text
    domain = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'room_315_vla'
        / 'pddl'
        / 'domain_room315_runtime.pddl'
    ).read_text(encoding='utf-8')
    action = domain[domain.index('(:action restore_normal_route'):]
    action = action[:action.index('\n  (:action ', 1)]
    assert '(route_reconfiguration_required ?side)' in action
    assert '(route_reconfiguration_safe ?side)' in action
    assert '(= (pending_clearances ?side) 0)' in action

    with pytest.raises(
        PddlProblemBuildError,
        match=(
            'mixed rail route requires normalization.*'
            'interior_shuttle_has_no_validated_stop_certificate'
        ),
    ):
        build_pddl_problem_from_observed_state_task_goal(
            state,
            goal,
            runtime_clearance_certificates={},
        )


    mismatched_certificate = dict(certificate)
    mismatched_certificate['target_segment'] = 'A23'
    with pytest.raises(
        PddlProblemBuildError,
        match='target segment invalid.*expected one of.*A23',
    ):
        build_pddl_problem_from_observed_state_task_goal(
            state,
            goal,
            runtime_clearance_certificates={'R1': mismatched_certificate},
        )

    visual_disagreement_observation = _observation(
        present={'R1': '4', 'R2': '1', 'R3': '3', 'R4': '2'}
    )
    visual_r1 = next(
        shuttle
        for shuttle in visual_disagreement_observation['shuttles']
        if shuttle['identity'] == 'R1'
    )
    exterior_length = public_rail_segment_lengths('right')['A34E']
    visual_r1.update({
        'block': 'A34E',
        's_m': 0.5079,
        's_ratio': 0.5079 / exterior_length,
        'segment_length_m': exterior_length,
    })
    visual_disagreement_state = VisualObservedStateBuilder().build(
        _snapshot(
            observation=visual_disagreement_observation,
            supervisor=supervisor,
        ),
        now_s=100.1,
        runtime_clearance_certificates={'R1': certificate},
    )
    with pytest.raises(
        PddlProblemBuildError,
        match=(
            'mixed rail route requires normalization.*'
            'interior_stop_certificate_segment_mismatch'
        ),
    ):
        build_pddl_problem_from_observed_state_task_goal(
            visual_disagreement_state,
            goal,
            runtime_clearance_certificates={'R1': certificate},
        )

    blocked_stopper_supervisor = _supervisor()
    blocked_stopper_supervisor['rails']['right']['switches']['A4'] = 'I'
    blocked_stopper_supervisor['rails']['right']['stoppers']['A2'] = '1'
    blocked_stopper_state = VisualObservedStateBuilder().build(
        _snapshot(
            observation=observation,
            supervisor=blocked_stopper_supervisor,
        ),
        now_s=100.1,
        runtime_clearance_certificates={'R1': certificate},
    )
    with pytest.raises(
        PddlProblemBuildError,
        match=(
            'mixed rail route requires normalization.*'
            'mixed_route_has_non_open_stopper_state'
        ),
    ):
        build_pddl_problem_from_observed_state_task_goal(
            blocked_stopper_state,
            goal,
            runtime_clearance_certificates={'R1': certificate},
        )


def test_full_rail_three_blocker_choreography_has_safe_capacity_pause_and_advance():
    """A four-shuttle rotation remains solvable after A34I reaches capacity."""

    def staged_state(*, r1_slot: str, r2_slot: str, clearance_mode: bool):
        observation = _observation(
            present={'R1': r1_slot, 'R2': r2_slot, 'R3': '3', 'R4': '4'}
        )
        length = public_rail_segment_lengths('right')['A34I']
        staged = {'R3': 0.35, 'R4': 0.95}
        for item in observation['shuttles']:
            if item['identity'] not in staged:
                continue
            s_m = staged[item['identity']]
            item.update({
                'block': 'A34I',
                's_m': s_m,
                's_ratio': s_m / length,
                'segment_length_m': length,
            })
        supervisor = _supervisor()
        if clearance_mode:
            supervisor['rails']['right']['switches'].update({
                'A1': 'E', 'A2': 'E', 'A3': 'I', 'A4': 'I',
            })
            supervisor['rails']['right']['stoppers']['A4'] = '1'
        certificates = {
            identity: _clearance_certificate(identity, target_s_m=s_m)
            for identity, s_m in staged.items()
        }
        return (
            VisualObservedStateBuilder().build(
                _snapshot(observation=observation, supervisor=supervisor),
                now_s=100.1,
                runtime_clearance_certificates=certificates,
            ),
            certificates,
        )

    goal = TaskGoal(
        goal_id='full-rail-r1-slot1-to-slot4',
        description='Move R1 to right slot 4',
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
            'target_shuttle': 'room315_right_shuttle_1',
            'target_slot': '4',
        },
    )

    active, certificates = staged_state(
        r1_slot='1', r2_slot='2', clearance_mode=True
    )
    active_parent = build_pddl_problem_from_observed_state_task_goal(
        active,
        goal,
        runtime_clearance_certificates=certificates,
    )
    active_clearance = active_parent.provenance[
        'target_blocker_clearance_plan'
    ]
    unavailable_destination = active_clearance[
        'ordered_relocations'
    ][0]['destination']
    assert unavailable_destination['kind'] == 'unavailable'
    assert unavailable_destination['reason'] == (
        'no_reachable_physically_separated_interior_holding_pose'
    )
    assert unavailable_destination['gate_switch'] == 'A3'
    assert unavailable_destination['target_segment'] == 'A34I'
    assert unavailable_destination['required_center_spacing_m'] == 0.57
    assert unavailable_destination['topology_dependency_search'][
        'resolved'
    ] is False
    assert active_parent.provenance['route_normalization']['by_side']['right'][
        'clearance_pause_safe'
    ] is True
    pause = ClosedLoopExecutive._next_planning_problem(active_parent)
    assert pause.goal_text == '(normal_route right)'
    assert pause.provenance['planning_phase'] == (
        'pause_clearance_for_exterior_choreography'
    )
    assert '(clearance_pause_safe right)' in pause.problem_text
    assert build_clearance_pause_problem(active_parent) == pause

    normal, certificates = staged_state(
        r1_slot='1', r2_slot='2', clearance_mode=False
    )
    park_parent = build_pddl_problem_from_observed_state_task_goal(
        normal,
        goal,
        runtime_clearance_certificates=certificates,
    )
    park = ClosedLoopExecutive._next_planning_problem(park_parent)
    assert park.provenance['planning_phase'] == 'clear_blocker_to_slot'
    assert park.selected_shuttle == 'right_shuttle_2'
    assert park.target_slot == '3'

    advanced_source, certificates = staged_state(
        r1_slot='1', r2_slot='3', clearance_mode=False
    )
    advance_parent = build_pddl_problem_from_observed_state_task_goal(
        advanced_source,
        goal,
        runtime_clearance_certificates=certificates,
    )
    advance_proof = advance_parent.provenance[
        'target_blocker_clearance_plan'
    ]['intermediate_selected_advance']
    assert advance_proof['source_slot'] == 'right_slot_1'
    assert advance_proof['target_slot'] == 'right_slot_2'
    assert advance_proof['final_target_slot'] == 'right_slot_4'
    advance = ClosedLoopExecutive._next_planning_problem(advance_parent)
    assert advance.goal_text == (
        '(shuttle_at_slot right_shuttle_1 right_slot_2)'
    )
    assert advance.provenance['parent_final_target_slot'] == '4'
    assert build_intermediate_selected_advance_problem(advance_parent) == advance


def test_left_public_interior_certificate_maps_for_route_normalization():
    observation = _observation(present={'L1': '4'})
    length = public_rail_segment_lengths('left')['A34I']
    l1 = next(
        shuttle for shuttle in observation['shuttles']
        if shuttle['identity'] == 'L1'
    )
    l1.update({
        'block': 'A34I',
        's_m': 0.35,
        's_ratio': 0.35 / length,
        'segment_length_m': length,
    })
    certificate = _clearance_certificate('R1', target_s_m=0.35)
    certificate.update({
        'identity': 'L1',
        'shuttle': 'left_shuttle_1',
        'side': 'left',
        'entry_sensor': 'DA3IL',
    })
    supervisor = _supervisor()
    supervisor['rails']['left']['switches']['A4'] = 'I'
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=observation, supervisor=supervisor),
        now_s=100.1,
        runtime_clearance_certificates={'L1': certificate},
    )
    goal = TaskGoal(
        goal_id='left-public-certificate-route-normalization',
        description='Move L1 to left slot 1',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'left',
            'target_kind': 'slot',
            'target_slot': '1',
            'target_shuttle': 'room315_left_shuttle_1',
            'payload_filter': 'any',
        },
    )

    problem = build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
        runtime_clearance_certificates={'L1': certificate},
    )
    normalization = problem.provenance[
        'route_normalization'
    ]['by_side']['left']

    assert normalization['interior_shuttles'] == ['left_shuttle_1']
    assert normalization['certified_stopped_interior_shuttles'] == [
        'left_shuttle_1'
    ]
    assert normalization['certificate_segment_mismatches'] == []
    consistency = normalization['certificate_segment_consistency'][
        'left_shuttle_1'
    ]
    assert consistency['certificate_target_public_segment'] == 'A34I'
    assert consistency['certificate_target_internal_segment'] == 'A12I'
    assert consistency['accepted_visual_internal_segment'] == 'A12I'
    assert consistency['satisfied'] is True
    assert normalization['reconfiguration_safe'] is True
    assert '(route_reconfiguration_safe left)' in problem.problem_text


def test_sequential_cutoff_isolates_one_r4_slot_parking_subproblem():
    state, certificates, goal = _sequential_cutoff_state_and_certificates()
    parent = build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
        runtime_clearance_certificates=certificates,
    )

    isolated = build_first_blocker_clearance_problem(parent)

    assert '(shuttle_at_slot right_shuttle_3 right_slot_2)' in parent.goal_text
    assert '(= (pending_clearances right) 1)' in parent.problem_text
    assert isolated.goal_text == '(shuttle_at_slot right_shuttle_4 right_slot_4)'
    assert '(:goal\n    (shuttle_at_slot right_shuttle_4 right_slot_4)\n  )' in (
        isolated.problem_text
    )
    assert '(= (pending_clearances right) 0)' in isolated.problem_text
    assert '(= (pending_clearances right) 1)' not in isolated.problem_text
    assert '(slot_free right_slot_4)' in isolated.problem_text
    assert '(slot_free right_slot_3)' not in isolated.problem_text
    assert isolated.provenance['planning_phase'] == 'clear_blocker_to_slot'
    assert isolated.provenance['parent_pending_clearances_suspended'] is True
    assert isolated.provenance['first_action_destination_constrained'] is True
    assert isolated.provenance['parking_target_free_fact_retained'] == (
        'right_slot_4'
    )
    assert isolated.provenance['temporarily_withheld_known_free_slots'] == [
        'right_slot_3'
    ]
    assert isolated.provenance['parent_problem_name'] == parent.problem_name
    assert isolated.target_slot == '4'
    assert isolated.target_station == 'staubli'
    assert isolated.selected_shuttle == 'right_shuttle_4'
    assert isolated.provenance['clearance_relocation']['shuttle'] == (
        'right_shuttle_4'
    )
    assert isolated.provenance['clearance_relocation']['destination'][
        'target_slot'
    ] == 'right_slot_4'
    # Historical recorded plans may use this legacy spelling. The isolated
    # provenance must still override the user's R3 -> slot-2 destination for
    # replay compatibility; the live PDDL domains no longer expose that
    # legacy action.
    assert _target_slot_for_step(
        PddlPlanStep(
            name='move_shuttle',
            args=('right', 'right_shuttle_4', 'yaskawa', 'staubli'),
            kwargs={'speed': '0.2'},
            raw='move_shuttle right right_shuttle_4 yaskawa staubli speed=0.2',
        ),
        goal,
        isolated,
    ) == 'right:slot:4'


def test_clearance_certificate_fails_closed_and_expires_only_after_proved_slot_arrival():
    builder = VisualObservedStateBuilder()
    invalid_cases = (
        (
            {'controller_position_fields_used_for_localization': True},
            'used forbidden controller position:R2',
        ),
        (
            {'matched_by': 'visual_position_only'},
            'lacks bounded-motion proof:R2',
        ),
        (
            {'bounded_commanded_motion_completed': False},
            'lacks completed bounded motion:R2',
        ),
        (
            {'clearance_mode_held': False},
            'lacks held-route proof:R2',
        ),
        (
            {'normal_route_restored': True},
            'lacks held-route proof:R2',
        ),
    )
    for overrides, expected_error in invalid_cases:
        invalid = {**_clearance_certificate(), **overrides}
        with pytest.raises(TaskExecutionStateError, match=expected_error):
            builder.build(
                _snapshot(),
                now_s=100.1,
                runtime_clearance_certificates={'R2': invalid},
            )

    provider = LatestVisualObservedStateProvider(builder)
    provider.set_runtime_clearance_certificate(_clearance_certificate())
    transport = VisualSupervisorTransport(
        provider=provider,
        publish_callback=lambda _command: None,
    )
    assert 'R2' in provider.runtime_clearance_certificates()

    transport.update_sensor_feedback('right', [{
        'active': True,
        'name': 'DZI2R',
        'shuttle': 'room315_right_shuttle_2',
    }])

    # One raw DZI frame must not erase a previously proved interior effect.
    assert 'R2' in provider.runtime_clearance_certificates()

    provider.update_supervisor(_supervisor(
        mode='DISABLED',
        reached_target_slot='2',
        shuttle_identity='R2',
    ))
    readings = [{
        'active': True,
        'name': 'DZI2R',
        'shuttle': 'room315_right_shuttle_2',
    }]
    provider.update_slot_sensor_feedback('right', readings)
    provider.update_slot_sensor_feedback('right', readings)

    assert provider.runtime_clearance_certificates() == {}
    assert provider.verified_slot_arrival_certificates()['R2']['slot'] == '2'


def test_new_shuttle_motion_invalidates_persisted_clearance_certificate():
    builder = VisualObservedStateBuilder()
    provider = LatestVisualObservedStateProvider(builder)
    provider.set_runtime_clearance_certificate(_clearance_certificate())
    published: list[dict] = []
    transport = VisualSupervisorTransport(
        provider=provider,
        publish_callback=published.append,
    )

    transport.publish_command({
        'action': 'shuttle',
        'command': 'ON',
        'side': 'right',
        'shuttle': 'right_shuttle_2',
    })

    assert provider.runtime_clearance_certificates() == {}
    assert published == [{
        'action': 'shuttle',
        'command': 'ON',
        'side': 'right',
        'shuttle': 'right_shuttle_2',
    }]


@pytest.mark.parametrize(
    ('command_name', 'shuttle_field'),
    (
        ('ON', 'shuttle'),
        ('RESET', 'shuttle'),
        ('REMOVE', 'shuttle'),
        ('ADD_MOVING', 'shuttle'),
        ('ON', 'shuttle_id'),
    ),
)
def test_motion_capable_command_invalidates_verified_slot_arrival_before_publish(
    command_name,
    shuttle_field,
):
    provider = LatestVisualObservedStateProvider(VisualObservedStateBuilder())
    provider.update_slot_sensor_feedback('right', [{
        'active': True,
        'name': 'DZI1R',
        'shuttle': 'room315_right_shuttle_2',
    }])
    provider.update_supervisor(_supervisor(
        mode='DISABLED',
        reached_target_slot='1',
        shuttle_identity='R2',
    ))
    provider.set_verified_slot_arrival_certificate(
        _verified_slot_arrival_certificate('R2', slot='1')
    )
    published: list[dict] = []

    def publish(command):
        assert provider.verified_slot_arrival_certificates() == {}
        published.append(command)

    transport = VisualSupervisorTransport(
        provider=provider,
        publish_callback=publish,
    )

    transport.publish_command({
        'action': 'shuttle',
        'command': command_name,
        'side': 'right',
        shuttle_field: 'right_shuttle_2',
    })

    assert provider.verified_slot_arrival_certificates() == {}
    assert published[0]['command'] == command_name


def test_late_pre_command_arrival_proof_cannot_cross_motion_epoch():
    """A new ON must reject proof finishing against old stopped telemetry."""

    provider = LatestVisualObservedStateProvider(VisualObservedStateBuilder())
    reading = {
        'active': True,
        'name': 'DZI1R',
        'shuttle': 'room315_right_shuttle_2',
    }
    provider.update_slot_sensor_feedback('right', [reading])
    provider.update_supervisor(_supervisor(
        mode='DISABLED',
        reached_target_slot='1',
        shuttle_identity='R2',
    ))
    pre_command_proof = _verified_slot_arrival_certificate(
        'R2',
        slot='1',
        motion_epoch=provider.verified_slot_motion_epoch('R2'),
    )
    provider.set_verified_slot_arrival_certificate(pre_command_proof)

    transport = VisualSupervisorTransport(
        provider=provider,
        publish_callback=lambda _command: None,
    )
    transport.publish_command({
        'action': 'shuttle',
        'command': 'ON',
        'side': 'right',
        'shuttle': 'right_shuttle_2',
    })

    assert provider.verified_slot_motion_epoch('R2') == 1
    assert provider.verified_slot_arrival_certificates() == {}
    # The controller and DZI still expose the old stopped levels here.  The
    # generation check, not a fortunate telemetry scheduling order, rejects
    # this late setter.
    with pytest.raises(
        TaskExecutionStateError,
        match='proof is not current in provider:R2',
    ):
        provider.set_verified_slot_arrival_certificate(pre_command_proof)
    assert provider.verified_slot_arrival_certificates() == {}

    # A proof whose verification started after that ON belongs to generation
    # 1 and may be registered once the same supervised arrival completes.
    current_proof = _verified_slot_arrival_certificate(
        'R2',
        slot='1',
        motion_epoch=provider.verified_slot_motion_epoch('R2'),
    )
    provider.set_verified_slot_arrival_certificate(current_proof)
    assert provider.verified_slot_arrival_certificates()['R2'] == current_proof


def test_verified_slot_arrival_expires_on_sensor_controller_or_time_change():
    clock = [100.0]
    provider = LatestVisualObservedStateProvider(
        VisualObservedStateBuilder(),
        monotonic=lambda: clock[0],
    )

    def stopped_status(mode='DISABLED', reached='1'):
        status = _supervisor()
        status['rails']['right']['shuttles'][
            'room315_right_shuttle_2'
        ] = {
            'mode': mode,
            'reached_target_slot': reached,
        }
        return status

    reading = {
        'active': True,
        'name': 'DZI1R',
        'shuttle': 'room315_right_shuttle_2',
    }
    certificate = _verified_slot_arrival_certificate('R2', slot='1')
    provider.update_supervisor(stopped_status(), receive_s=clock[0])
    provider.update_slot_sensor_feedback('right', [reading], receive_s=clock[0])
    provider.set_verified_slot_arrival_certificate(certificate)
    assert 'R2' in provider.verified_slot_arrival_certificates()

    provider.update_supervisor(
        stopped_status(mode='MOVING'),
        receive_s=clock[0],
    )
    assert provider.verified_slot_arrival_certificates() == {}

    provider.update_supervisor(stopped_status(), receive_s=clock[0])
    provider.update_slot_sensor_feedback('right', [reading], receive_s=clock[0])
    provider.set_verified_slot_arrival_certificate(certificate)
    provider.update_slot_sensor_feedback('right', [], receive_s=clock[0])
    assert provider.verified_slot_arrival_certificates() == {}

    # Re-adding the same physical reading does not resurrect a consumed proof.
    provider.update_slot_sensor_feedback('right', [reading], receive_s=clock[0])
    assert provider.verified_slot_arrival_certificates() == {}

    provider.set_verified_slot_arrival_certificate(certificate)
    clock[0] += 1.1
    assert provider.verified_slot_arrival_certificates(now_s=clock[0]) == {}


def test_verified_slot_arrival_cannot_bridge_telemetry_outage_or_moving_race():
    clock = [100.0]
    config = LiveStateConfig(
        slot_sensor_state_timeout_s=1.0,
        supervisor_status_timeout_s=1.5,
    )
    provider = LatestVisualObservedStateProvider(
        VisualObservedStateBuilder(config),
        monotonic=lambda: clock[0],
    )
    reading = {
        'active': True,
        'name': 'DZI1R',
        'shuttle': 'room315_right_shuttle_2',
    }
    certificate = _verified_slot_arrival_certificate('R2', slot='1')
    stopped = _supervisor(
        mode='DISABLED',
        reached_target_slot='1',
        shuttle_identity='R2',
    )
    provider.update_supervisor(stopped, receive_s=clock[0])
    provider.update_slot_sensor_feedback('right', [reading], receive_s=clock[0])
    provider.set_verified_slot_arrival_certificate(certificate)

    # No getter runs during the outage. One repeated sensor frame after the
    # timeout must not resurrect the earlier multi-frame arrival proof.
    clock[0] = 101.1
    provider.update_slot_sensor_feedback('right', [reading], receive_s=clock[0])
    assert provider.verified_slot_arrival_certificates() == {}

    # A new proof may be registered while both sources are fresh and stopped,
    # but an equivalent supervisor frame after its own outage consumes it.
    provider.set_verified_slot_arrival_certificate(certificate)
    clock[0] = 101.6
    provider.update_supervisor(stopped, receive_s=clock[0])
    assert provider.verified_slot_arrival_certificates() == {}

    # Recreate fresh DZI data, then reproduce the setter TOCTOU ordering:
    # MOVING arrives before certificate insertion. The setter must reject it.
    provider.update_slot_sensor_feedback('right', [reading], receive_s=clock[0])
    provider.update_supervisor(
        _supervisor(
            mode='MOVING',
            reached_target_slot='1',
            shuttle_identity='R2',
        ),
        receive_s=clock[0],
    )
    with pytest.raises(
        TaskExecutionStateError,
        match='proof is not current in provider:R2',
    ):
        provider.set_verified_slot_arrival_certificate(certificate)


def test_runtime_restart_recovers_only_after_stable_stopped_dzi_proof():
    clock = [100.0]
    provider = LatestVisualObservedStateProvider(
        VisualObservedStateBuilder(),
        slot_sensor_confirmation_frames=2,
        monotonic=lambda: clock[0],
    )
    reading = {
        'active': True,
        'name': 'DZI1R',
        'shuttle': 'room315_right_shuttle_2',
    }
    provider.update_supervisor(
        _supervisor(
            mode='DISABLED',
            reached_target_slot='1',
            shuttle_identity='R2',
        ),
        receive_s=clock[0],
    )

    provider.update_slot_sensor_feedback('right', [reading], receive_s=clock[0])
    assert provider.verified_slot_arrival_certificates() == {}
    clock[0] += 0.1
    provider.update_slot_sensor_feedback('right', [reading], receive_s=clock[0])

    recovered = provider.verified_slot_arrival_certificates()['R2']
    assert recovered['proof_mode'] == 'stable_stopped_dzi_runtime_recovery'
    assert recovered['sensor_confirmation_frames'] == 2
    assert recovered['controller_mode'] == 'DISABLED'
    assert recovered['reached_target_slot'] == '1'
    assert recovered['controller_position_fields_used_for_localization'] is False

    # A locally issued motion command consumes and suppresses restart
    # recovery, even if old stopped telemetry repeats before motion begins.
    transport = VisualSupervisorTransport(
        provider=provider,
        publish_callback=lambda _command: None,
    )
    transport.publish_command({
        'action': 'shuttle',
        'command': 'ON',
        'side': 'right',
        'shuttle': 'right_shuttle_2',
    })
    clock[0] += 0.1
    provider.update_slot_sensor_feedback('right', [reading], receive_s=clock[0])
    clock[0] += 0.1
    provider.update_slot_sensor_feedback('right', [reading], receive_s=clock[0])
    assert provider.verified_slot_arrival_certificates() == {}


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


def test_nearest_exact_slot_uses_forward_topology_from_segment_only_position():
    observation = _observation(present={'R4': '3'})
    r4 = next(
        item for item in observation['shuttles']
        if item['identity'] == 'R4'
    )
    # This is past both A34E slot anchors. Reaching slot 3 requires a directed
    # wraparound route, and the position deliberately has no exact-slot label.
    r4['s_ratio'] = 0.9
    r4['s_m'] = r4['segment_length_m'] * r4['s_ratio']
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=observation),
        now_s=100.1,
    )
    assert not any(
        fact.subject == 'room315_right_shuttle_4'
        and fact.predicate == 'location_slot'
        for fact in state.fused_planner_state
    )
    goal = TaskGoal(
        goal_id='nearest-segment-only-wrap-to-slot3',
        description='Move the nearest right shuttle to slot 3',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'payload_filter': 'any',
            'selection_strategy': 'nearest',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '3',
        },
    )

    grounded = ground_transport_task_goal(goal, state)

    assert grounded.constraints['target_shuttle'] == 'room315_right_shuttle_4'


def test_complete_atomic_transport_selection_matrix_is_groundable():
    """Persist the finite user-request matrix instead of relying on examples."""

    placements = {
        **{f'L{index}': str(index) for index in range(1, 5)},
        **{f'R{index}': str(index) for index in range(1, 5)},
    }
    validator = Room315DomainValidator()

    def state_with_payload(
        *,
        identity: str = '',
        payload_filter: str = 'any',
    ):
        observation = _observation(present=placements)
        for item in observation['shuttles']:
            item['loaded_state'] = (
                'loaded' if item['identity'] in {'L4', 'R4'} else 'empty'
            )
            if item['identity'] == identity:
                item['loaded_state'] = (
                    'loaded' if payload_filter == 'loaded' else 'empty'
                )
        return VisualObservedStateBuilder().build(
            _snapshot(observation=observation),
            now_s=100.1,
        )

    generic_state = state_with_payload()

    explicit_cases = 0
    for spec in all_shuttle_specs():
        destinations = (
            [('slot', slot) for slot in ('1', '2', '3', '4')]
            + [
                ('station', station)
                for station in STATIONS_BY_SIDE[spec.side]
            ]
        )
        for payload_filter in ('loaded', 'empty', 'any'):
            state = state_with_payload(
                identity=spec.short_id,
                payload_filter=payload_filter,
            )
            for target_kind, target_value in destinations:
                target_fields = {
                    'target_slot': target_value,
                } if target_kind == 'slot' else {
                    'target_station': target_value,
                }
                validation = validator.validate(TaskGoalDraft(
                    goal_type='transport',
                    payload_filter=payload_filter,
                    selection_strategy='explicit',
                    side=spec.side,
                    target_kind=target_kind,
                    target_shuttle=spec.gazebo_entity_name,
                    **target_fields,
                ))
                assert validation.ok, validation.to_dict()
                grounded = ground_transport_task_goal(
                    validation.task_goal,
                    state,
                )
                assert grounded.constraints['target_shuttle'] == (
                    spec.gazebo_entity_name
                )
                assert grounded.constraints['payload_filter'] == payload_filter
                if target_kind == 'station':
                    assert grounded.constraints['target_kind'] == 'slot'
                    assert grounded.constraints['target_station'] == target_value
                    assert grounded.constraints['target_slot'] in {
                        '1', '2', '3', '4',
                    }
                explicit_cases += 1

    selection_cases = 0
    for side in ('left', 'right'):
        destinations = (
            [('slot', slot) for slot in ('1', '2', '3', '4')]
            + [
                ('station', station)
                for station in STATIONS_BY_SIDE[side]
            ]
        )
        for selection in ('any', 'nearest'):
            for payload_filter in ('any', 'loaded', 'empty'):
                for target_kind, target_value in destinations:
                    target_fields = {
                        'target_slot': target_value,
                    } if target_kind == 'slot' else {
                        'target_station': target_value,
                    }
                    validation = validator.validate(TaskGoalDraft(
                        goal_type='transport',
                        payload_filter=payload_filter,
                        selection_strategy=selection,
                        side=side,
                        target_kind=target_kind,
                        **target_fields,
                    ))
                    assert validation.ok, validation.to_dict()
                    grounded = ground_transport_task_goal(
                        validation.task_goal,
                        generic_state,
                    )
                    selected = grounded.constraints['target_shuttle']
                    assert selected.startswith(f'room315_{side}_shuttle_')
                    if payload_filter == 'loaded':
                        assert selected.endswith('_4')
                    elif payload_filter == 'empty':
                        assert not selected.endswith('_4')
                    if target_kind == 'station':
                        assert grounded.constraints['target_kind'] == 'slot'
                        assert grounded.constraints['target_station'] == target_value
                    selection_cases += 1

    assert explicit_cases == 144
    assert selection_cases == 72


def test_any_prefers_eligible_shuttle_already_at_exact_target_slot():
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=_observation(present={'R1': '1', 'R2': '3'})),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='any-right-already-slot3',
        description='Move any right shuttle to slot 3',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'payload_filter': 'any',
            'selection_strategy': 'any',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '3',
        },
    )

    grounded = ground_transport_task_goal(goal, state)

    assert grounded.constraints['target_shuttle'] == 'room315_right_shuttle_2'


def test_any_prefers_lower_cost_feasible_route_before_stable_identity():
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=_observation(present={'R1': '1', 'R2': '2'})),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='any-right-low-cost-slot3',
        description='Move any right shuttle to slot 3',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'payload_filter': 'any',
            'selection_strategy': 'any',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '3',
        },
    )

    grounded = ground_transport_task_goal(goal, state)

    assert grounded.constraints['target_shuttle'] == 'room315_right_shuttle_2'


@pytest.mark.parametrize(
    ('legacy_constraints', 'expected_payload', 'expected_identity'),
    [
        ({'shuttle_selection': 'loaded'}, 'loaded', 'R4'),
        ({'shuttle_selection': 'any', 'payload_required': False}, 'empty', 'R2'),
    ],
)
def test_legacy_payload_selection_is_canonicalized_before_enumeration(
    legacy_constraints,
    expected_payload,
    expected_identity,
):
    observation = _observation(present={'R2': '2', 'R4': '1'})
    for item in observation['shuttles']:
        if item['identity'] == 'R2':
            item['loaded_state'] = 'empty'
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=observation),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id=f'legacy-{expected_payload}-right-slot3',
        description=f'Move a legacy {expected_payload} shuttle to slot 3',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '3',
            **legacy_constraints,
        },
    )

    grounded = ground_transport_task_goal(goal, state)

    assert grounded.constraints['payload_filter'] == expected_payload
    assert grounded.constraints['selection_strategy'] == 'explicit'
    assert grounded.constraints['target_shuttle'].endswith(
        f'_shuttle_{expected_identity[-1]}'
    )


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


def test_rail_inspection_passes_through_without_transport_fields():
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=_observation()),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='inspect-right-rail-runtime',
        description='Inspect the right rail',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'inspection',
            'target_kind': 'rail',
            'side': 'right',
            'inspection_subject': 'right:rail',
        },
    )

    assert ground_transport_task_goal(goal, state) is goal


def test_explicit_inspection_rejects_absent_shuttle():
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=_observation(present={'R4': '2'})),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='inspect-absent-r2',
        description='Inspect R2',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'inspection',
            'target_kind': 'shuttle',
            'target_shuttle': 'R2',
            'inspection_subject': 'room315_right_shuttle_2',
            'side': 'right',
        },
    )

    with pytest.raises(
        TaskExecutionStateError,
        match='explicit inspection shuttle is absent or presence is unknown:R2',
    ):
        ground_transport_task_goal(goal, state)


def test_inspection_loaded_selection_is_live_grounded_to_present_identity():
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=_observation(present={'R2': '1', 'R4': '2'})),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='inspect-loaded-right',
        description='Inspect the loaded right shuttle',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'inspection',
            'target_kind': 'shuttle_selection',
            'side': 'right',
            'selection_strategy': 'any',
            'shuttle_selection': 'loaded',
            'payload_required': True,
            'inspection_subject': 'right:shuttle_selection:any:loaded',
        },
    )

    grounded = ground_transport_task_goal(goal, state)

    assert grounded.constraints['target_kind'] == 'shuttle'
    assert grounded.constraints['target_shuttle'] == 'room315_right_shuttle_4'
    assert grounded.constraints['inspection_subject'] == (
        'room315_right_shuttle_4'
    )
    assert grounded.constraints['payload_filter'] == 'loaded'


def test_complete_validator_accepted_inspection_matrix_is_groundable():
    placements = {
        **{f'L{index}': str(index) for index in range(1, 5)},
        **{f'R{index}': str(index) for index in range(1, 5)},
    }
    validator = Room315DomainValidator()

    def state_with_payload(
        *,
        identity: str = '',
        payload_filter: str = 'any',
    ):
        observation = _observation(present=placements)
        for item in observation['shuttles']:
            item['loaded_state'] = (
                'loaded' if item['identity'] in {'L4', 'R4'} else 'empty'
            )
            if item['identity'] == identity:
                item['loaded_state'] = (
                    'loaded' if payload_filter == 'loaded' else 'empty'
                )
        return VisualObservedStateBuilder().build(
            _snapshot(observation=observation),
            now_s=100.1,
        )

    generic_state = state_with_payload()
    grounded_cases = 0

    for spec in all_shuttle_specs():
        for payload_filter in ('loaded', 'empty', 'any'):
            validation = validator.validate(TaskGoalDraft(
                goal_type='inspection',
                target_kind='shuttle',
                target_shuttle=spec.gazebo_entity_name,
                payload_filter=payload_filter,
            ))
            assert validation.ok, validation.to_dict()
            grounded = ground_transport_task_goal(
                validation.task_goal,
                state_with_payload(
                    identity=spec.short_id,
                    payload_filter=payload_filter,
                ),
            )
            assert grounded.constraints['target_shuttle'] == (
                spec.gazebo_entity_name
            )
            grounded_cases += 1

    for side in ('left', 'right'):
        for payload_filter in ('loaded', 'empty', 'any'):
            validation = validator.validate(TaskGoalDraft(
                goal_type='inspection',
                target_kind='shuttle_selection',
                side=side,
                selection_strategy='any',
                payload_filter=payload_filter,
            ))
            assert validation.ok, validation.to_dict()
            grounded = ground_transport_task_goal(
                validation.task_goal,
                generic_state,
            )
            assert grounded.constraints['target_kind'] == 'shuttle'
            assert grounded.constraints['side'] == side
            grounded_cases += 1

    non_shuttle_drafts = [
        TaskGoalDraft(goal_type='inspection', target_kind='system'),
    ]
    for side in ('left', 'right'):
        non_shuttle_drafts.append(TaskGoalDraft(
            goal_type='inspection',
            target_kind='rail',
            side=side,
        ))
        non_shuttle_drafts.extend(
            TaskGoalDraft(
                goal_type='inspection',
                target_kind='slot',
                side=side,
                target_slot=slot,
            )
            for slot in ('1', '2', '3', '4')
        )
        non_shuttle_drafts.extend(
            TaskGoalDraft(
                goal_type='inspection',
                target_kind='station',
                side=side,
                target_station=station,
            )
            for station in STATIONS_BY_SIDE[side]
        )
    for draft in non_shuttle_drafts:
        validation = validator.validate(draft)
        assert validation.ok, validation.to_dict()
        assert ground_transport_task_goal(
            validation.task_goal,
            generic_state,
        ) is validation.task_goal
        grounded_cases += 1

    assert grounded_cases == 45


def test_nearest_inspection_without_reference_fails_closed_for_clarification():
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=_observation(present={'R2': '1', 'R4': '2'})),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='inspect-nearest-right-without-reference',
        description='Inspect the nearest right shuttle',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'inspection',
            'target_kind': 'shuttle_selection',
            'side': 'right',
            'selection_strategy': 'nearest',
            'payload_filter': 'any',
            'inspection_subject': 'right:shuttle_selection:nearest:any',
        },
    )

    with pytest.raises(
        TaskExecutionStateError,
        match='nearest shuttle inspection requires an exact slot or station',
    ):
        ground_transport_task_goal(goal, state)


def test_grounding_rejects_explicit_absent_shuttle():
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=_observation(present={'R4': '2'})),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='absent-r2-to-slot3',
        description='Move R2 to slot 3',
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
            'target_slot': '3',
            'target_shuttle': 'R2',
        },
    )

    with pytest.raises(
        TaskExecutionStateError,
        match='explicit target shuttle is absent or presence is unknown:R2',
    ):
        ground_transport_task_goal(goal, state)


def test_grounding_rejects_explicit_shuttle_with_wrong_payload_state():
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=_observation(present={'R2': '1'})),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='loaded-r2-but-r2-is-empty',
        description='Move loaded R2 to slot 3',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'payload_filter': 'loaded',
            'selection_strategy': 'explicit',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '3',
            'target_shuttle': 'R2',
        },
    )

    with pytest.raises(
        TaskExecutionStateError,
        match='explicit target shuttle is not loaded:R2',
    ):
        ground_transport_task_goal(goal, state)


def test_nearest_station_goal_uses_present_visual_shuttle_and_sensor_slot():
    observation = _observation(present={'R2': '1', 'R4': '4'})
    state = VisualObservedStateBuilder().build(
        _snapshot(observation=observation),
        now_s=100.1,
    )
    goal = TaskGoal(
        goal_id='nearest-right-yaskawa',
        description='Move the nearest right shuttle to Yaskawa',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'payload_filter': 'any',
            'selection_strategy': 'nearest',
            'shuttle_selection': 'nearest',
            'side': 'right',
            'target_kind': 'station',
            'target_station': 'right:yaskawa',
        },
    )

    grounded = ground_transport_task_goal(goal, state)

    assert grounded.constraints['target_shuttle'] == 'room315_right_shuttle_2'
    assert grounded.constraints['selection_strategy'] == 'explicit'
    assert grounded.constraints['target_kind'] == 'slot'
    assert grounded.constraints['target_station'] == 'yaskawa'
    assert grounded.constraints['target_slot'] == '1'


def test_slot_sensor_arrival_sends_supervised_off_and_confirms_stop():
    builder = VisualObservedStateBuilder()
    provider = LatestVisualObservedStateProvider(builder)
    commands: list[dict] = []
    transport = None

    def publish(command: dict) -> None:
        commands.append(command)
        if command.get('command') == 'OFF':
            stopped = _supervisor(
                decision_count=1,
                mode='DISABLED',
                reached_target_slot='3',
            )
            transport.update_supervisor(stopped)
            provider.update_supervisor(stopped)

    transport = VisualSupervisorTransport(
        provider=provider,
        publish_callback=publish,
        slot_sensor_confirmation_frames=1,
        controller_stop_timeout_s=1.0,
    )
    moving = _supervisor(
        decision_count=0,
        mode='DISABLED',
        reached_target_slot='3',
    )
    target = _observation(present={'R4': '3'})
    provider.update_supervisor(moving)
    provider.update_observation(target)
    transport.update_supervisor(moving)
    transport.update_observation(target)
    target_readings = [{
        'active': True,
        'name': 'DZI3R',
        'shuttle': 'room315_right_shuttle_4',
        # Position-like fields must be ignored by the runtime gate.
        'segment': 'WRONG',
        's': -999.0,
        's_ratio': -999.0,
    }]
    transport.update_sensor_feedback('right', target_readings)
    provider.update_slot_sensor_feedback('right', target_readings)

    result = transport.wait_for_target_arrival(
        side='right',
        target_sensors=['DZI3R'],
        shuttle='right_shuttle_4',
        timeout_s=1.0,
        target_slot='3',
    )

    assert result['arrived']
    assert result['matched_by'] == 'deterministic_slot_sensor'
    assert result['sensor_identity_confirmed'] is True
    assert result['controller_target_slot_confirmed'] is True
    assert result['controller_position_fields_used_for_localization'] is False
    certificate = result['verified_slot_arrival_certificate']
    assert certificate['identity'] == 'R4'
    assert certificate['slot'] == '3'
    assert certificate['sensor'] == 'DZI3R'
    assert certificate['controller_mode'] == 'DISABLED'
    assert certificate['reached_target_slot'] == '3'
    assert certificate['model_prediction_replaced'] is False
    assert (
        provider.verified_slot_arrival_certificates()['R4']
        == certificate
    )
    assert commands == [{
        'action': 'shuttle',
        'side': 'right',
        'shuttle': 'right_shuttle_4',
        'command': 'OFF',
        'closed_loop_executive': {
            'mode': 'slot_sensor_target_arrival_finalize',
            'target_slot': '3',
            'target_sensor': 'DZI3R',
            'final_arrival_source': 'deterministic_slot_sensor',
            'planner_localization_source': 'accepted_visual_state',
        },
    }]


def test_visual_target_prediction_cannot_stop_without_slot_sensor():
    builder = VisualObservedStateBuilder()
    provider = LatestVisualObservedStateProvider(builder)
    commands: list[dict] = []
    transport = VisualSupervisorTransport(
        provider=provider,
        publish_callback=commands.append,
        slot_sensor_confirmation_frames=1,
        controller_stop_timeout_s=0.1,
    )
    transport.update_supervisor(_supervisor(mode='MOVING'))
    transport.update_observation(_observation(present={'R4': '3'}))
    transport.update_sensor_feedback('right', [])

    result = transport.wait_for_target_arrival(
        side='right',
        target_sensors=['DZI3R'],
        shuttle='right_shuttle_4',
        timeout_s=0.02,
        target_slot='3',
    )

    assert not result['arrived']
    assert result['matched_by'] == 'deterministic_slot_sensor'
    assert commands == []


def test_interior_clearance_sensor_stop_is_confirmed_by_fresh_visual_state():
    builder = VisualObservedStateBuilder()
    provider = LatestVisualObservedStateProvider(builder)
    commands: list[dict] = []
    transport = None

    def publish(command: dict) -> None:
        commands.append(command)
        transport.update_supervisor(
            _supervisor(decision_count=1, mode='DISABLED')
        )
        transport.update_observation(observation)

    transport = VisualSupervisorTransport(
        provider=provider,
        publish_callback=publish,
        controller_stop_timeout_s=1.0,
    )
    observation = _observation(present={'R4': '3'})
    r4 = next(item for item in observation['shuttles'] if item['identity'] == 'R4')
    length = public_rail_segment_lengths('right')['A34I']
    r4.update({
        'block': 'A34I',
        's_m': 0.7083,
        's_ratio': 0.7083 / length,
        'segment_length_m': length,
    })
    # Deliberately wrong controller position fields prove they are not used.
    transport.update_supervisor(_supervisor(mode='MOVING'))
    transport.update_observation(observation)
    transport.update_sensor_feedback('right', [{
        'active': True,
        'name': 'DA3IR',
        'shuttle': 'room315_right_shuttle_4',
    }])

    result = transport.wait_for_visual_position_and_stop(
        side='right',
        shuttle='right_shuttle_4',
        target_segment='A34I',
        target_s_m=0.7083,
        tolerance_m=0.08,
        entry_sensor='DA3IR',
        minimum_clearance_delay_s=0.0,
        timeout_s=1.0,
    )

    assert result['arrived'] is True
    assert result['matched_by'] == (
        'interior_entry_sensor_plus_bounded_travel_time'
    )
    assert result['controller_stop_confirmed'] is True
    assert result['post_stop_visual_confirmation'] is True
    assert result['controller_position_fields_used_for_localization'] is False
    assert commands[0]['command'] == 'OFF'
    assert commands[0]['closed_loop_executive']['stop_trigger'] == (
        'interior_entry_sensor_plus_bounded_travel_time'
    )


def test_wrong_visual_block_cannot_bypass_interior_entry_sensor_stop():
    builder = VisualObservedStateBuilder()
    provider = LatestVisualObservedStateProvider(builder)
    commands: list[dict] = []
    transport = None
    observation = _observation(present={'R4': '3'})

    def publish(command: dict) -> None:
        commands.append(command)
        transport.update_supervisor(
            _supervisor(decision_count=1, mode='DISABLED')
        )
        # A fresh but still-wrong model classification must not prevent OFF.
        transport.update_observation(observation)

    transport = VisualSupervisorTransport(
        provider=provider,
        publish_callback=publish,
        controller_stop_timeout_s=1.0,
    )
    transport.update_supervisor(_supervisor(mode='MOVING'))
    transport.update_observation(observation)
    transport.update_sensor_feedback('right', [{
        'active': True,
        'name': 'DA3IR',
        'shuttle': 'room315_right_shuttle_4',
    }])

    result = transport.wait_for_visual_position_and_stop(
        side='right',
        shuttle='right_shuttle_4',
        target_segment='A34I',
        target_s_m=0.95,
        tolerance_m=0.08,
        entry_sensor='DA3IR',
        minimum_clearance_delay_s=0.0,
        timeout_s=1.0,
    )

    assert result['arrived'] is True
    assert result['matched_by'] == (
        'interior_entry_sensor_plus_bounded_travel_time'
    )
    assert result['entry_sensor_identity_confirmed'] is True
    assert result['post_stop_visual_confirmation'] is False
    assert commands[0]['command'] == 'OFF'
    assert commands[0]['closed_loop_executive']['stop_trigger'] == (
        'interior_entry_sensor_plus_bounded_travel_time'
    )


def test_visual_clearance_prediction_alone_cannot_authorize_interior_motion():
    builder = VisualObservedStateBuilder()
    provider = LatestVisualObservedStateProvider(builder)
    commands: list[dict] = []
    transport = None
    observation = _observation(present={'R4': '3'})
    r4 = next(item for item in observation['shuttles'] if item['identity'] == 'R4')
    length = public_rail_segment_lengths('right')['A34I']
    r4.update({
        'block': 'A34I',
        's_m': 0.7083,
        's_ratio': 0.7083 / length,
        'segment_length_m': length,
    })

    def publish(command: dict) -> None:
        commands.append(command)
        transport.update_supervisor(
            _supervisor(decision_count=1, mode='DISABLED')
        )

    transport = VisualSupervisorTransport(
        provider=provider,
        publish_callback=publish,
        controller_stop_timeout_s=1.0,
    )
    transport.update_supervisor(_supervisor(mode='MOVING'))
    transport.update_observation(observation)

    result = transport.wait_for_visual_position_and_stop(
        side='right',
        shuttle='right_shuttle_4',
        target_segment='A34I',
        target_s_m=0.7083,
        tolerance_m=0.08,
        entry_sensor='DA3IR',
        minimum_clearance_delay_s=0.0,
        timeout_s=0.02,
    )

    assert result['arrived'] is False
    assert 'entry_sensor_seen=False' in result['reason']
    assert commands[0]['closed_loop_executive']['stop_trigger'] == (
        'clearance_timeout_guard'
    )


def test_sensor_zone_entry_without_controller_setpoint_completion_guard_stops():
    builder = VisualObservedStateBuilder()
    provider = LatestVisualObservedStateProvider(builder)
    commands: list[dict] = []
    transport = None

    def publish(command: dict) -> None:
        commands.append(command)
        stopped = _supervisor(decision_count=1, mode='WAITING')
        transport.update_supervisor(stopped)

    transport = VisualSupervisorTransport(
        provider=provider,
        publish_callback=publish,
        slot_sensor_confirmation_frames=1,
        controller_stop_timeout_s=0.1,
    )
    transport.update_supervisor(_supervisor(mode='MOVING'))
    transport.update_sensor_feedback('right', [{
        'active': True,
        'name': 'DZI3R',
        'shuttle': 'room315_right_shuttle_4',
    }])

    result = transport.wait_for_target_arrival(
        side='right',
        target_sensors=['DZI3R'],
        shuttle='right_shuttle_4',
        timeout_s=1.0,
        target_slot='3',
    )

    assert not result['arrived']
    assert 'target_slot 3' in result['reason']
    assert 'fresh controller DISABLED state' in result['reason']
    assert commands[0]['closed_loop_executive']['mode'] == (
        'slot_sensor_setpoint_timeout_guard'
    )


def test_controller_stop_confirmation_requires_fresh_disabled_snapshot():
    builder = VisualObservedStateBuilder()
    provider = LatestVisualObservedStateProvider(builder)
    transport = VisualSupervisorTransport(
        provider=provider,
        publish_callback=lambda _command: None,
        controller_stop_timeout_s=0.1,
    )

    transport.update_supervisor(_supervisor(mode='DISABLED'))
    stale_sequence = transport.supervisor_state_count()
    stale = transport._wait_controller_stopped(
        side='right',
        shuttle='R4',
        timeout_s=0.01,
        after_supervisor_sequence=stale_sequence,
    )
    assert stale['ready'] is False
    assert 'fresh controller DISABLED state' in stale['reason']

    transport.update_supervisor(_supervisor(mode='WAITING'))
    enabled_waiting = transport._wait_controller_stopped(
        side='right',
        shuttle='R4',
        timeout_s=0.01,
        after_supervisor_sequence=stale_sequence,
    )
    assert enabled_waiting['ready'] is False

    transport.update_supervisor(_supervisor(mode='DISABLED'))
    disabled = transport._wait_controller_stopped(
        side='right',
        shuttle='R4',
        timeout_s=0.1,
        after_supervisor_sequence=stale_sequence,
    )
    assert disabled['ready'] is True
    assert disabled['mode'] == 'DISABLED'
    assert disabled['supervisor_sequence'] > stale_sequence


@pytest.mark.parametrize(
    ('reading', 'reason'),
    [
        ({'active': True, 'name': 'DZI3R', 'shuttle': ''}, 'unknown shuttle'),
        (
            {
                'active': True,
                'name': 'DZI3R',
                'shuttle': 'room315_right_shuttle_2',
            },
            'occupied by room315_right_shuttle_2',
        ),
        (
            {
                'active': True,
                'name': 'DZI3R',
                'shuttle': 'room315_left_shuttle_4',
            },
            'wrong rail side',
        ),
    ],
)
def test_slot_sensor_identity_mismatch_fails_closed(reading, reason):
    builder = VisualObservedStateBuilder()
    provider = LatestVisualObservedStateProvider(builder)
    transport = VisualSupervisorTransport(
        provider=provider,
        publish_callback=lambda _command: None,
        slot_sensor_confirmation_frames=1,
    )
    transport.update_sensor_feedback('right', [reading])

    result = transport.wait_for_target_arrival(
        side='right',
        target_sensors=['DZI3R'],
        shuttle='right_shuttle_4',
        timeout_s=0.1,
        target_slot='3',
    )

    assert not result['arrived']
    assert reason in result['reason']


def test_slot_sensor_contract_mismatch_fails_before_motion_stop():
    builder = VisualObservedStateBuilder()
    provider = LatestVisualObservedStateProvider(builder)
    commands: list[dict] = []
    transport = VisualSupervisorTransport(
        provider=provider,
        publish_callback=commands.append,
    )

    result = transport.wait_for_target_arrival(
        side='right',
        target_sensors=['DZI2R'],
        shuttle='right_shuttle_4',
        timeout_s=0.1,
        target_slot='3',
    )

    assert not result['arrived']
    assert 'contract mismatch' in result['reason']
    assert commands == []


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
