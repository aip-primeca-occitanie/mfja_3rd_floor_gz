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
from room_315_closed_loop_executive import _target_slot_for_step
from room_315_multi_shuttle import all_shuttle_specs
from room_315_pddl_plan_translator import PddlPlanStep
from room_315_pddl_scenario_generator import _planning_rail_topology
from room_315_pddl_scenario_generator import PddlProblemBuildError
from room_315_pddl_scenario_generator import build_first_blocker_clearance_problem
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
    reached_target_slot: str = '',
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
                    'reached_target_slot': reached_target_slot,
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


def _sequential_cutoff_state_and_certificates():
    """Reproduce the accepted state before the live R3 slot-1 -> slot-2 failure."""

    observation = _observation(
        present={'R1': '3', 'R2': '4', 'R3': '1', 'R4': '2'}
    )
    segment_length_m = public_rail_segment_lengths('right')['A34I']
    staged_positions_m = {'R1': 0.35, 'R2': 0.95}
    for item in observation['shuttles']:
        target_s_m = staged_positions_m.get(item['identity'])
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
    assert destination['target_s_m'] == 0.35
    assert abs(destination['target_s_m'] - 0.95) >= (
        destination['required_center_spacing_m']
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


def test_certified_interior_segment_disagreement_rejects_topology_motion():
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

    with pytest.raises(
        PddlProblemBuildError,
        match=(
            'runtime clearance and accepted visual segment disagree.*'
            'certificate=A34I, visual=A34E'
        ),
    ):
        build_pddl_problem_from_observed_state_task_goal(
            state,
            goal,
            runtime_clearance_certificates={'R2': certificate},
        )


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
    # The real PlanSys2 backend canonicalizes move_shuttle_to_slot to this
    # legacy spelling and drops the slot arguments. The isolated provenance
    # must still override the user's R3 -> slot-2 destination for this step.
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


def test_clearance_certificate_fails_closed_and_expires_at_a_slot_sensor():
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

    assert provider.runtime_clearance_certificates() == {}


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
                mode='WAITING',
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
        mode='WAITING',
        reached_target_slot='3',
    )
    target = _observation(present={'R4': '3'})
    provider.update_supervisor(moving)
    provider.update_observation(target)
    transport.update_supervisor(moving)
    transport.update_observation(target)
    transport.update_sensor_feedback('right', [{
        'active': True,
        'name': 'DZI3R',
        'shuttle': 'room315_right_shuttle_4',
        # Position-like fields must be ignored by the runtime gate.
        'segment': 'WRONG',
        's': -999.0,
        's_ratio': -999.0,
    }])

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
            _supervisor(decision_count=1, mode='WAITING')
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
            _supervisor(decision_count=1, mode='WAITING')
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
            _supervisor(decision_count=1, mode='WAITING')
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
    assert commands[0]['closed_loop_executive']['mode'] == (
        'slot_sensor_setpoint_timeout_guard'
    )


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
