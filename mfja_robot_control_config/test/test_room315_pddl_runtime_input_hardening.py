#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import room_315_pddl_scenario_generator as generator
from room_315_multi_shuttle import normalize_shuttle_ref


def _spec(*, starts=(('right_shuttle_1', '1'),)):
    return generator.ScenarioSpec(
        goal_id='runtime-input-hardening',
        side='right',
        shuttle='right_shuttle_1',
        source='yaskawa',
        target='staubli',
        target_slot='3',
        payload_condition='empty',
        start_slots_by_shuttle=starts,
    )


def _state_and_goal(*, starts=(('right_shuttle_1', '1'),)):
    spec = _spec(starts=starts)
    return (
        generator._observed_state_from_scenario_spec(spec),
        generator._task_goal_from_scenario_spec(spec),
    )


def _certificate(
    identity='R1',
    *,
    target_s_m=0.35,
    side=None,
):
    spec = normalize_shuttle_ref(identity)
    assert spec is not None
    rail_side = side or spec.side
    return {
        'identity': spec.short_id,
        'shuttle': spec.shuttle_id,
        'side': rail_side,
        'target_segment': 'A34I',
        'target_s_m': target_s_m,
        'observed_segment': 'A34I',
        'observed_s_m': target_s_m,
        'absolute_error_m': 0.0,
        'entry_sensor': 'DA3IR' if rail_side == 'right' else 'DA3IL',
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


def _with_exact_r1_slot_anchor(state, *, segment, s_ratio):
    facts = []
    for fact in state.fused_planner_state:
        if (
            fact.subject == 'room315_right_shuttle_1'
            and fact.predicate == 'location_slot'
        ):
            facts.append(replace(
                fact,
                source='state_fuser',
                metadata={
                    'selected_source': 'trusted_device',
                },
            ))
        else:
            facts.append(fact)
    facts.append(replace(
        generator._planner_fact(
            'room315_right_shuttle_1',
            'rail_position',
            {
                'available': True,
                'side': 'right',
                'segment': segment,
                's_ratio': s_ratio,
            },
            timestamp=state.timestamp,
            metadata={'field_owner': 'visual_model'},
        ),
        source='state_fuser',
        metadata={'selected_source': 'visual_model'},
    ))
    return replace(state, fused_planner_state=facts)


def _with_exact_l2_slot_anchor(state, *, segment, s_ratio):
    facts = []
    for fact in state.fused_planner_state:
        if (
            fact.subject == 'room315_left_shuttle_2'
            and fact.predicate == 'location_slot'
        ):
            facts.append(replace(
                fact,
                source='state_fuser',
                metadata={'selected_source': 'trusted_device'},
            ))
        else:
            facts.append(fact)
    facts.append(replace(
        generator._planner_fact(
            'room315_left_shuttle_2',
            'rail_position',
            {
                'available': True,
                'side': 'left',
                'segment': segment,
                's_ratio': s_ratio,
            },
            timestamp=state.timestamp,
            metadata={'field_owner': 'visual_model'},
        ),
        source='state_fuser',
        metadata={'selected_source': 'visual_model'},
    ))
    return replace(state, fused_planner_state=facts)


def test_missing_presence_fact_fails_closed_instead_of_defaulting_present():
    state, goal = _state_and_goal()
    facts = [
        fact for fact in state.fused_planner_state
        if not (
            fact.subject == 'room315_right_shuttle_4'
            and fact.predicate == 'present'
        )
    ]

    with pytest.raises(
        generator.PddlProblemBuildError,
        match='missing required presence fact.*right_shuttle_4',
    ):
        generator.build_pddl_problem_from_observed_state_task_goal(
            replace(state, fused_planner_state=facts),
            goal,
        )


def test_right_transport_still_requires_complete_left_presence_registry():
    state, goal = _state_and_goal()
    facts = [
        fact for fact in state.fused_planner_state
        if not (
            fact.subject == 'room315_left_shuttle_4'
            and fact.predicate == 'present'
        )
    ]

    with pytest.raises(
        generator.PddlProblemBuildError,
        match='missing required presence fact.*left_shuttle_4',
    ):
        generator.build_pddl_problem_from_observed_state_task_goal(
            replace(state, fused_planner_state=facts),
            goal,
        )


def test_presence_fact_requires_an_explicit_boolean():
    state, goal = _state_and_goal()
    facts = [
        replace(fact, value='present')
        if (
            fact.subject == 'room315_right_shuttle_1'
            and fact.predicate == 'present'
        )
        else fact
        for fact in state.fused_planner_state
    ]

    with pytest.raises(
        generator.PddlProblemBuildError,
        match='presence fact.*explicit boolean',
    ):
        generator.build_pddl_problem_from_observed_state_task_goal(
            replace(state, fused_planner_state=facts),
            goal,
        )


def test_exact_slot_anchor_accepts_consistent_visual_segment_and_ratio():
    state, goal = _state_and_goal()
    slot = generator._planning_rail_topology('right').slots['1']

    problem = generator.build_pddl_problem_from_observed_state_task_goal(
        _with_exact_r1_slot_anchor(
            state,
            segment=slot.segment,
            s_ratio=slot.s_ratio,
        ),
        goal,
    )

    assert '(shuttle_at_slot right_shuttle_1 right_slot_1)' in (
        problem.problem_text
    )


def test_exact_slot_anchor_rejects_visual_segment_disagreement():
    state, goal = _state_and_goal()

    with pytest.raises(
        generator.PddlProblemBuildError,
        match='exact slot anchor and visual segment disagree',
    ):
        generator.build_pddl_problem_from_observed_state_task_goal(
            _with_exact_r1_slot_anchor(
                state,
                segment='A34E',
                s_ratio=0.45,
            ),
            goal,
        )


def test_exact_slot_anchor_rejects_visual_ratio_disagreement():
    state, goal = _state_and_goal()
    slot = generator._planning_rail_topology('right').slots['1']

    with pytest.raises(
        generator.PddlProblemBuildError,
        match='exact slot anchor and visual s_ratio disagree',
    ):
        generator.build_pddl_problem_from_observed_state_task_goal(
            _with_exact_r1_slot_anchor(
                state,
                segment=slot.segment,
                s_ratio=min(1.0, slot.s_ratio + 0.121),
            ),
            goal,
        )


def test_right_transport_defers_unrelated_left_anchor_ratio_disagreement():
    state, goal = _state_and_goal(starts=(
        ('right_shuttle_1', '1'),
        ('left_shuttle_2', '2'),
    ))
    slot = generator._planning_rail_topology('left').slots['2']

    problem = generator.build_pddl_problem_from_observed_state_task_goal(
        _with_exact_l2_slot_anchor(
            state,
            # Left visual labels use the public mirror name; the planner maps
            # A12E back to the authoritative internal A34E segment.
            segment='A12E',
            s_ratio=slot.s_ratio + 0.121590135,
        ),
        goal,
    )

    scope = problem.provenance['planning_scope']
    assert scope['goal_side'] == 'right'
    assert scope['exact_slot_anchor_visual_consistency'] == 'right'
    assert scope['deferred_out_of_scope_location_issues'] == [{
        'shuttle': 'left_shuttle_2',
        'side': 'left',
        'reason': (
            'exact slot anchor and visual s_ratio disagree for '
            "'left_shuttle_2': error 0.121590135 exceeds 0.120000000"
        ),
    }]
    # Global registry and occupancy integrity remain in the frozen problem;
    # only the unrelated learned-vs-anchor consistency veto is deferred.
    assert '(shuttle_at_slot left_shuttle_2 left_slot_2)' in problem.problem_text
    assert '(slot_occupied_by left_slot_2 left_shuttle_2)' in problem.problem_text


def test_left_transport_still_rejects_same_left_anchor_ratio_disagreement():
    state, _ = _state_and_goal(starts=(
        ('right_shuttle_1', '1'),
        ('left_shuttle_2', '2'),
    ))
    slot = generator._planning_rail_topology('left').slots['2']
    left_goal = replace(
        generator._task_goal_from_scenario_spec(generator.ScenarioSpec(
            goal_id='left-anchor-route-relevant',
            side='left',
            shuttle='left_shuttle_2',
            source='yaskawa',
            target='kuka',
            target_slot='3',
            payload_condition='empty',
            start_slots_by_shuttle=(('left_shuttle_2', '2'),),
        )),
        goal_id='left-anchor-route-relevant',
    )

    with pytest.raises(
        generator.PddlProblemBuildError,
        match='exact slot anchor and visual s_ratio disagree.*left_shuttle_2',
    ):
        generator.build_pddl_problem_from_observed_state_task_goal(
            _with_exact_l2_slot_anchor(
                state,
                segment='A12E',
                s_ratio=slot.s_ratio + 0.121590135,
            ),
            left_goal,
        )


@pytest.mark.parametrize(
    ('override', 'match'),
    (
        ({'identity': 'R2'}, 'identity conflict'),
        ({'shuttle': 'right_shuttle_2'}, 'identity conflict'),
        ({'side': 'left'}, 'side conflict'),
        ({'target_segment': 'A34E'}, 'target segment invalid'),
        ({'entry_sensor': 'DA3IL'}, 'entry sensor invalid'),
        ({'entry_sensor_identity_confirmed': 'true'}, 'entry sensor proof'),
        ({'controller_stop_confirmed': 1}, 'stop proof'),
        ({'bounded_commanded_motion_completed': 'yes'}, 'completed bounded motion'),
        ({'post_stop_visual_frame_received': False}, 'post-stop visual proof'),
        ({'model_prediction_replaced': True}, 'replaced model prediction'),
    ),
)
def test_runtime_clearance_certificate_rejects_wrong_provenance(
    override,
    match,
):
    certificate = {**_certificate(), **override}

    with pytest.raises(generator.PddlProblemBuildError, match=match):
        generator._validated_runtime_clearance_certificates(
            {'R1': certificate},
            present_by_shuttle={'right_shuttle_1': True},
        )


@pytest.mark.parametrize('target_s_m', (float('nan'), float('inf'), -0.01, 99.0))
def test_runtime_clearance_certificate_rejects_nonfinite_or_out_of_bounds_s_m(
    target_s_m,
):
    with pytest.raises(
        generator.PddlProblemBuildError,
        match='target_s_m out of bounds',
    ):
        generator._validated_runtime_clearance_certificates(
            {'R1': _certificate(target_s_m=target_s_m)},
            present_by_shuttle={'right_shuttle_1': True},
        )


def test_runtime_clearance_certificate_requires_explicit_presence():
    with pytest.raises(
        generator.PddlProblemBuildError,
        match='not explicitly present: R1',
    ):
        generator._validated_runtime_clearance_certificates(
            {'R1': _certificate()},
            present_by_shuttle={'right_shuttle_1': False},
        )


def test_runtime_clearance_certificates_reject_physical_overlap():
    certificates = {
        'R1': _certificate('R1', target_s_m=0.35),
        'R2': _certificate('R2', target_s_m=0.50),
    }

    with pytest.raises(
        generator.PddlProblemBuildError,
        match='physical spacing violation.*right_shuttle_1,right_shuttle_2',
    ):
        generator._validated_runtime_clearance_certificates(
            certificates,
            present_by_shuttle={
                'right_shuttle_1': True,
                'right_shuttle_2': True,
            },
        )


def test_overlapping_certificates_fail_before_clearance_pause_can_be_asserted():
    state, goal = _state_and_goal(starts=(
        ('right_shuttle_1', '1'),
        ('right_shuttle_2', '2'),
    ))
    certificates = {
        'R1': _certificate('R1', target_s_m=0.35),
        'R2': _certificate('R2', target_s_m=0.50),
    }

    with pytest.raises(
        generator.PddlProblemBuildError,
        match='runtime clearance physical spacing violation',
    ):
        generator.build_pddl_problem_from_observed_state_task_goal(
            state,
            goal,
            runtime_clearance_certificates=certificates,
        )


def test_physically_separated_left_and_right_certificates_are_valid():
    validated = generator._validated_runtime_clearance_certificates(
        {
            'R1': _certificate('R1', target_s_m=0.35),
            'R2': _certificate('R2', target_s_m=0.95),
            'L1': _certificate('L1', target_s_m=0.35),
            'L2': _certificate('L2', target_s_m=0.95),
        },
        present_by_shuttle={
            'right_shuttle_1': True,
            'right_shuttle_2': True,
            'left_shuttle_1': True,
            'left_shuttle_2': True,
        },
    )

    assert set(validated) == {
        'right_shuttle_1',
        'right_shuttle_2',
        'left_shuttle_1',
        'left_shuttle_2',
    }
    assert validated['right_shuttle_1']['entry_sensor'] == 'DA3IR'
    assert validated['left_shuttle_1']['entry_sensor'] == 'DA3IL'
