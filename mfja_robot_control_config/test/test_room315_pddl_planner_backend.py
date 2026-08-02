#!/usr/bin/env python3

import importlib.util
import itertools
import subprocess
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'scripts'
    / 'room_315_pddl_scenario_generator.py'
)
RIGHT_CASE = 'right_loaded_r1_s1_to_slot3_no_blocker_speed008'
LEFT_CASE = 'left_loaded_l1_s1_to_slot3_no_blocker_speed008'


class FakePlanSysClient:
    def __init__(self, actions):
        self.actions = list(actions)
        self.calls = []
        self.closed = False

    def get_plan(self, *, domain, problem):
        self.calls.append({'domain': domain, 'problem': problem})
        return SimpleNamespace(
            items=[
                SimpleNamespace(action=action, time=float(index), duration=1.0)
                for index, action in enumerate(self.actions)
            ]
        )

    def close(self):
        self.closed = True


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_pddl_scenario_generator', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _right_plansys_actions():
    return [
        '(prepare_switches right right_yaskawa right_staubli right_switch_group)',
        '(open_stoppers right right_yaskawa right_staubli right_stopper_group)',
        '(move_shuttle_to_slot right_shuttle_1 right right_yaskawa '
        'right_staubli right_slot_1 right_slot_3)',
        '(stop_shuttle right_shuttle_1 right right_yaskawa right_staubli)',
        '(finish_candidate_task right_shuttle_1 right_staubli right_slot_3)',
    ]


def test_default_backend_is_plansys_only():
    generator = _load_module()

    backend = generator.create_planner_backend()

    assert isinstance(backend, generator.PlanSysPlannerBackend)


def test_fallback_backend_is_not_supported():
    generator = _load_module()

    with pytest.raises(ValueError, match='fallback.*no longer supported.*PlanSys2'):
        generator.create_planner_backend('fallback')

    assert not hasattr(generator, 'FallbackRoom315PlannerBackend')


def test_external_pddl_backend_is_not_supported():
    generator = _load_module()

    with pytest.raises(ValueError, match='external-pddl.*no longer supported.*PlanSys2'):
        generator.create_planner_backend('external-pddl')

    assert not hasattr(generator, 'ExternalPDDLPlannerBackend')


def test_missing_plansys_package_gives_clear_error(monkeypatch):
    generator = _load_module()
    spec = generator.scenario_spec_from_case(RIGHT_CASE)
    original_import_module = generator.importlib.import_module

    def fake_import_module(name, *args, **kwargs):
        if name == 'plansys2_msgs.srv':
            raise ModuleNotFoundError("No module named 'plansys2_msgs'")
        return original_import_module(name, *args, **kwargs)

    monkeypatch.setattr(generator.importlib, 'import_module', fake_import_module)
    backend = generator.PlanSysPlannerBackend()

    with pytest.raises(RuntimeError, match='PlanSys2.*(required|service).*fall.*back'):
        backend.plan(spec, speed=0.3)


def test_plansys_backend_converts_plan_items_to_internal_symbolic_plan():
    generator = _load_module()
    spec = generator.scenario_spec_from_case(RIGHT_CASE)
    client = FakePlanSysClient(_right_plansys_actions())
    backend = generator.PlanSysPlannerBackend(planner_client=client)

    plan = backend.plan(spec, speed=0.41)

    assert plan == [
        'prepare_switches right right_yaskawa right_staubli right_switch_group',
        'open_stoppers right right_yaskawa right_staubli right_stopper_group',
        'move_shuttle_to_slot right_shuttle_1 right right_yaskawa '
        'right_staubli right_slot_1 right_slot_3 speed=0.41',
        'stop_shuttle right_shuttle_1 right right_yaskawa right_staubli',
        'finish_candidate_task right_shuttle_1 right_staubli right_slot_3',
    ]


def test_plansys_backend_canonicalizes_costed_slot_plan_actions():
    generator = _load_module()
    spec = generator.scenario_spec_from_case(RIGHT_CASE)
    client = FakePlanSysClient([
        '(prepare_switches right right_yaskawa right_staubli right_switch_group)',
        '(open_stoppers right right_yaskawa right_staubli right_stopper_group)',
        (
            '(move_shuttle_to_slot right_shuttle_1 right right_yaskawa '
            'right_staubli right_slot_1 right_slot_3)'
        ),
        '(stop_shuttle right_shuttle_1 right right_yaskawa right_staubli)',
        '(finish_candidate_task right_shuttle_1 right_staubli right_slot_3)',
    ])
    backend = generator.PlanSysPlannerBackend(planner_client=client)

    plan = backend.plan(spec, speed=0.29)

    assert plan == [
        'prepare_switches right right_yaskawa right_staubli right_switch_group',
        'open_stoppers right right_yaskawa right_staubli right_stopper_group',
        'move_shuttle_to_slot right_shuttle_1 right right_yaskawa '
        'right_staubli right_slot_1 right_slot_3 speed=0.29',
        'stop_shuttle right_shuttle_1 right right_yaskawa right_staubli',
        'finish_candidate_task right_shuttle_1 right_staubli right_slot_3',
    ]


def test_plansys_canonicalization_preserves_topology_route_and_move_endpoints():
    generator = _load_module()
    spec = generator.scenario_spec_from_case(RIGHT_CASE)
    problem = generator.build_pddl_problem_from_spec(spec)
    plan_msg = SimpleNamespace(items=[
        SimpleNamespace(action=(
            '(prepare_topology_route right_shuttle_2 right '
            'right_topology_a34i right_slot_1 right_switch_group)'
        )),
        SimpleNamespace(action=(
            '(move_shuttle_from_segment_to_slot right_shuttle_2 right '
            'right_topology_a34i right_yaskawa right_slot_1)'
        )),
    ])

    plan = generator._symbolic_plan_from_plansys_plan(
        plan_msg,
        spec=None,
        problem=problem,
        speed=0.2,
    )

    assert plan == [
        'prepare_topology_route right_shuttle_2 right '
        'right_topology_a34i right_slot_1 right_switch_group',
        'move_shuttle_from_segment_to_slot right_shuttle_2 right '
        'right_topology_a34i right_yaskawa right_slot_1 speed=0.2',
    ]
    translated = generator.translate_plan(plan)
    assert [step.pddl_step.name for step in translated] == [
        'prepare_topology_route',
        'move_shuttle_from_segment_to_slot',
    ]
    assert translated[0].command['source_block'] == 'right_topology_a34i'
    assert translated[0].command['target_slot'] == 'right_slot_1'
    assert translated[1].command['source_block'] == 'right_topology_a34i'
    assert translated[1].command['target_station'] == 'right_yaskawa'
    assert translated[1].command['target_slot'] == 'right_slot_1'
    assert translated[1].command['speed'] == 0.2


def test_known_slot_alternate_route_preserves_source_occupancy_and_action_family():
    generator = _load_module()
    spec = generator.ScenarioSpec(
        goal_id='known-slot-alternate-route',
        side='right',
        shuttle='right_shuttle_1',
        source='yaskawa',
        target='yaskawa',
        target_slot='1',
        payload_condition='loaded',
        start_slots_by_shuttle=(
            ('right_shuttle_1', '2'),
            ('right_shuttle_2', '3'),
        ),
    )
    problem = generator.build_pddl_problem_from_observed_state_task_goal(
        generator._observed_state_from_scenario_spec(spec),
        generator._task_goal_from_scenario_spec(spec),
    )
    clearance = problem.provenance['target_blocker_clearance_plan']

    assert clearance['topology_alternative_selected'] is True
    assert '(shuttle_at_slot right_shuttle_1 right_slot_2)' in problem.problem_text
    assert '(segment_only_location right_shuttle_1)' not in problem.problem_text
    assert '(switch_group_on_side right_switch_group right)' in problem.problem_text

    plan_msg = SimpleNamespace(items=[
        SimpleNamespace(action=(
            '(prepare_slot_topology_route right_shuttle_1 right '
            'right_slot_2 right_topology_a12e right_slot_1 '
            'right_switch_group)'
        )),
        SimpleNamespace(action=(
            '(move_shuttle_via_topology_to_slot right_shuttle_1 right '
            'right_slot_2 right_topology_a12e right_yaskawa right_yaskawa '
            'right_slot_1)'
        )),
    ])
    plan = generator._symbolic_plan_from_plansys_plan(
        plan_msg,
        spec=None,
        problem=problem,
        speed=0.2,
    )

    assert plan == [
        'prepare_slot_topology_route right_shuttle_1 right right_slot_2 '
        'right_topology_a12e right_slot_1 right_switch_group',
        'move_shuttle_via_topology_to_slot right_shuttle_1 right '
        'right_slot_2 right_topology_a12e right_yaskawa right_yaskawa '
        'right_slot_1 '
        'speed=0.2',
    ]


def test_plansys_canonicalization_preserves_route_normalization_endpoints():
    generator = _load_module()
    problem = generator.build_pddl_problem_from_spec(
        generator.scenario_spec_from_case(RIGHT_CASE)
    )
    plan_msg = SimpleNamespace(items=[SimpleNamespace(action=(
        '(restore_normal_route right right_staubli right_staubli)'
    ))])

    plan = generator._symbolic_plan_from_plansys_plan(
        plan_msg,
        spec=None,
        problem=problem,
        speed=0.2,
    )

    assert plan == [
        'restore_normal_route right right_staubli right_staubli'
    ]
    translated = generator.translate_plan(plan)[0]
    assert translated.command['action'] == 'restore_normal_route'
    assert translated.command['source_station'] == 'right_staubli'
    assert translated.command['target_station'] == 'right_staubli'


@pytest.mark.parametrize(
    'malformed_action',
    [
        (
            '(prepare_topology_route right_shuttle_2 right '
            'right_topology_a34i right_slot_1)'
        ),
        (
            '(move_shuttle_from_segment_to_slot right_shuttle_2 right '
            'right_topology_a34i right_yaskawa)'
        ),
    ],
)
def test_plansys_topology_plan_fails_closed_instead_of_eliding_malformed_step(
    malformed_action,
):
    generator = _load_module()
    problem = generator.build_pddl_problem_from_spec(
        generator.scenario_spec_from_case(RIGHT_CASE)
    )
    plan_msg = SimpleNamespace(items=[
        SimpleNamespace(action=malformed_action),
        SimpleNamespace(action=(
            '(move_shuttle_from_segment_to_slot right_shuttle_2 right '
            'right_topology_a34i right_yaskawa right_slot_1)'
        )),
    ])

    with pytest.raises(RuntimeError, match='unsupported|canonical'):
        generator._symbolic_plan_from_plansys_plan(
            plan_msg,
            spec=None,
            problem=problem,
            speed=0.2,
        )


@pytest.mark.parametrize(
    'legacy_action',
    [
        '(move_shuttle right_shuttle_1 right right_yaskawa right_staubli)',
        '(set_stoppers right A1 open)',
        '(wait_for_clearance right_shuttle_1 right_slot_3)',
    ],
)
def test_plansys_runtime_rejects_legacy_actions_without_exact_contract(
    legacy_action,
):
    generator = _load_module()
    problem = generator.build_pddl_problem_from_spec(
        generator.scenario_spec_from_case(RIGHT_CASE)
    )

    with pytest.raises(RuntimeError, match='unsupported or malformed'):
        generator._symbolic_plan_from_plansys_plan(
            SimpleNamespace(items=[SimpleNamespace(action=legacy_action)]),
            spec=None,
            problem=problem,
            speed=0.2,
        )


def test_plansys_backend_sends_room315_domain_and_problem_to_plan_service():
    generator = _load_module()
    spec = generator.scenario_spec_from_case(RIGHT_CASE)
    client = FakePlanSysClient(_right_plansys_actions())
    backend = generator.PlanSysPlannerBackend(planner_client=client)

    backend.plan(spec, speed=0.3)

    assert len(client.calls) == 1
    call = client.calls[0]
    assert '(domain room315-shuttle)' in call['domain']
    assert f'(problem room315-{RIGHT_CASE})' in call['problem']
    assert '(goal_candidate right_shuttle_1)' in call['problem']
    assert '(task_done right_shuttle_1 right_staubli)' in call['problem']
    assert '(shuttle_at_slot right_shuttle_1 right_slot_3)' in call['problem']
    assert '(= (route_cost right_slot_1 right_slot_3) 2)' in call['problem']
    assert 'right_shuttle_1 right_shuttle_2 right_shuttle_3 right_shuttle_4' in call['problem']
    assert '(slot_occupied_by right_slot_1 right_shuttle_1)' in call['problem']
    assert '(block_free right_block_slot_3)' in call['problem']


@pytest.mark.parametrize(
    'selection,expected',
    [('any', 'right_shuttle_1'), ('nearest', 'right_shuttle_2')],
)
def test_non_explicit_goal_is_identity_bound_before_plansys(
    selection,
    expected,
):
    generator = _load_module()
    spec = generator.ScenarioSpec(
        goal_id=f'identity-bound-{selection}',
        side='right',
        shuttle='right_shuttle_1',
        source='yaskawa',
        target='staubli',
        target_slot='3',
        payload_condition='loaded',
        loaded_shuttles=('right_shuttle_1', 'right_shuttle_2'),
        start_slots_by_shuttle=(
            ('right_shuttle_1', '1'),
            ('right_shuttle_2', '2'),
        ),
    )
    state = generator._observed_state_from_scenario_spec(spec)
    goal = generator.TaskGoal(
        goal_id=f'identity-bound-{selection}',
        description='Move a loaded shuttle to right slot 3',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '3',
            'selection_strategy': selection,
            'payload_filter': 'loaded',
        },
    )

    problem = generator.build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
    )

    assert problem.selected_shuttle == expected
    assert problem.provenance['candidate_shuttles'] == [expected]
    assert problem.provenance['eligible_candidate_shuttles'] == [
        'right_shuttle_1',
        'right_shuttle_2',
    ] if selection == 'any' else [
        'right_shuttle_2',
        'right_shuttle_1',
    ]
    assert problem.provenance['selection_owner'] == (
        'deterministic accepted-visual grounding before PlanSys2'
    )
    assert problem.problem_text.count('(goal_candidate ') == 1
    assert f'(goal_candidate {expected})' in problem.problem_text


def test_problem_builder_fails_closed_on_unknown_required_facts():
    generator = _load_module()
    spec = generator.scenario_spec_from_case(RIGHT_CASE)
    state = generator._observed_state_from_scenario_spec(spec)
    facts = []
    for fact in state.fused_planner_state:
        if fact.subject == 'right:slot:3' and fact.predicate == 'occupancy':
            facts.append(generator.ObservedFact(
                fact_id=fact.fact_id,
                subject=fact.subject,
                predicate=fact.predicate,
                value=fact.value,
                source=fact.source,
                timestamp=fact.timestamp,
                confidence=fact.confidence,
                status='unknown',
                metadata=fact.metadata,
            ))
        else:
            facts.append(fact)
    unsafe_state = generator.ObservedState(
        state_id='unsafe-right-slot3-unknown',
        timestamp=state.timestamp,
        stale_after_s=state.stale_after_s,
        visual_model_inputs=[],
        fused_planner_state=facts,
    )
    task_goal = generator._task_goal_from_scenario_spec(spec)

    with pytest.raises(generator.PddlProblemBuildError, match='observation or recovery'):
        generator.build_pddl_problem_from_observed_state_task_goal(unsafe_state, task_goal)


def _with_continuous_r2(generator, state, *, segment, s_ratio):
    facts = [
        replace(fact, value=True)
        if (
            fact.subject == 'room315_right_shuttle_2'
            and fact.predicate == 'present'
        )
        else fact
        for fact in state.fused_planner_state
    ]
    facts.append(generator._planner_fact(
        'room315_right_shuttle_2',
        'rail_position',
        {
            'side': 'right',
            'segment': segment,
            's_ratio': s_ratio,
            'position_uncertainty_m': 0.01,
        },
        timestamp=state.timestamp,
        metadata={'model_input_exposure': 'excluded'},
    ))
    return replace(state, fused_planner_state=facts)


def _segment_only_two_shuttle_state(
    generator,
    *,
    side,
    selected_segment,
    blocker_segment,
):
    selected = f'{side}_shuttle_4'
    blocker = f'{side}_shuttle_2'
    selected_entity = f'room315_{side}_shuttle_4'
    blocker_entity = f'room315_{side}_shuttle_2'
    far_station = 'staubli' if side == 'right' else 'kuka'
    spec = generator.ScenarioSpec(
        goal_id='segment-only-clearance-seed',
        side=side,
        shuttle=selected,
        source='yaskawa',
        target=far_station,
        target_slot='3',
        payload_condition='loaded',
        loaded_shuttles=(selected,),
        start_slots_by_shuttle=((selected, '1'), (blocker, '3')),
    )
    state = generator._observed_state_from_scenario_spec(spec)
    facts = []
    for fact in state.fused_planner_state:
        if (
            fact.subject in {selected_entity, blocker_entity}
            and fact.predicate in {'location_slot', 'location_block'}
        ):
            facts.append(replace(fact, value=None))
            continue
        if (
            fact.predicate == 'occupancy'
            and (
                ':slot:' in fact.subject
                or ':block:slot:' in fact.subject
            )
        ):
            value = (
                {
                    'occupied': False,
                    'shuttle': None,
                    'sensor': fact.value.get('sensor'),
                }
                if isinstance(fact.value, dict)
                else None
            )
            facts.append(replace(fact, value=value))
            continue
        facts.append(fact)
    for entity, segment, ratio in (
        (selected_entity, selected_segment, 0.20),
        (blocker_entity, blocker_segment, 0.75),
    ):
        facts.append(generator._planner_fact(
            entity,
            'rail_position',
            {
                'side': side,
                'segment': segment,
                's_ratio': ratio,
                'position_uncertainty_m': 0.0,
            },
            timestamp=state.timestamp,
            metadata={'model_input_exposure': 'excluded'},
        ))
    return replace(state, fused_planner_state=facts), selected, blocker


@pytest.mark.parametrize(
    'side,selected_segment,blocker_segment,target_slot,source_block',
    [
        ('right', 'A23', 'A12E', '2', 'right_topology_a23'),
        ('right', 'A2E', 'A12E', '2', 'right_topology_a2e'),
        ('right', 'A14', 'A34E', '4', 'right_topology_a14'),
        ('right', 'A4E', 'A34E', '4', 'right_topology_a4e'),
        ('left', 'A23', 'A12E', '2', 'left_topology_a14'),
        ('left', 'A2E', 'A12E', '2', 'left_topology_a4e'),
        ('left', 'A14', 'A34E', '4', 'left_topology_a23'),
        ('left', 'A4E', 'A34E', '4', 'left_topology_a2e'),
    ],
)
def test_segment_origin_blocker_clearance_is_representable_for_both_rails(
    side,
    selected_segment,
    blocker_segment,
    target_slot,
    source_block,
):
    generator = _load_module()
    state, selected, blocker = _segment_only_two_shuttle_state(
        generator,
        side=side,
        selected_segment=selected_segment,
        blocker_segment=blocker_segment,
    )
    goal = generator.TaskGoal(
        goal_id=(
            f'{side}-{selected_segment}-{blocker_segment}-slot-{target_slot}'
        ),
        description='Move a segment-bound shuttle after clearing its blocker',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': side,
            'target_kind': 'slot',
            'target_slot': target_slot,
            'target_shuttle': selected,
            'selection_strategy': 'explicit',
            'payload_filter': 'any',
        },
    )

    parent = generator.build_pddl_problem_from_observed_state_task_goal(
        state,
        goal,
    )
    clearance = parent.provenance['target_blocker_clearance_plan']
    isolated = generator.build_first_blocker_clearance_problem(parent)

    assert clearance['source_kind'] == 'accepted_visual_continuous_position'
    assert clearance['observed_blockers'] == [blocker]
    assert clearance['ordered_relocations'][0]['destination']['kind'] == (
        'interior_loop'
    )
    assert f'(segment_only_location {selected})' in parent.problem_text
    init_text = parent.problem_text.split('(:goal', 1)[0]
    assert f'(shuttle_at_slot {selected} ' not in init_text
    assert isolated.goal_text == f'(clearance_relocated {blocker})'
    assert (
        f'(topology_route_blocked_by {selected} {source_block} '
        f'{side}_slot_{target_slot} {blocker})'
        in isolated.problem_text
    )

    plan_message = SimpleNamespace(items=[
        SimpleNamespace(action=(
            f'(begin_segment_route_clearance {selected} {side} '
            f'{source_block} {side}_slot_{target_slot})'
        )),
        SimpleNamespace(action=(
            f'(relocate_segment_blocker_to_interior {blocker} {selected} '
            f'{side} {source_block} {side}_slot_{target_slot})'
        )),
    ])
    symbolic = generator._symbolic_plan_from_plansys_plan(
        plan_message,
        spec=None,
        problem=isolated,
        speed=0.2,
    )
    translated = generator.translate_plan(symbolic)

    assert symbolic == [
        f'begin_segment_route_clearance {selected} {side} '
        f'{source_block} {side}_slot_{target_slot}',
        f'relocate_segment_blocker_to_interior {blocker} {selected} {side} '
        f'{source_block} {side}_slot_{target_slot} speed=0.2',
    ]
    assert [item.command['action'] for item in translated] == [
        'switches',
        'clearance_relocation',
    ]


def test_problem_builder_removes_route_clear_and_emits_order_for_blocker_ahead():
    generator = _load_module()
    spec = generator.scenario_spec_from_case(RIGHT_CASE)
    state = _with_continuous_r2(
        generator,
        generator._observed_state_from_scenario_spec(spec),
        segment='A12E',
        s_ratio=0.72,
    )

    problem = generator.build_pddl_problem_from_observed_state_task_goal(
        state,
        generator._task_goal_from_scenario_spec(spec),
    )

    assert '(route_clear_between right_slot_1 right_slot_3)' not in problem.problem_text
    assert (
        '(route_blocked_by right_slot_1 right_slot_3 right_shuttle_2)'
        in problem.problem_text
    )
    assert (
        '(clearance_precedes right_shuttle_2 right_shuttle_1)'
        in problem.problem_text
    )
    assert '(front_of right_shuttle_2 right_shuttle_1)' in problem.problem_text
    assert '(behind right_shuttle_1 right_shuttle_2)' in problem.problem_text
    route_pair = next(
        pair
        for pair in problem.provenance['route_clearance']['pairs']
        if pair['from_slot'] == 'right_slot_1'
        and pair['to_slot'] == 'right_slot_3'
    )
    assert route_pair['blocker_shuttles'] == ['right_shuttle_2']
    assert route_pair['clear'] is False
    clearance = problem.provenance['target_blocker_clearance_plan']
    assert clearance['required'] is True
    assert clearance['ordered_relocations'] == [
        {
            'order': 1,
            'shuttle': 'right_shuttle_2',
            'reason': 'blocks_selected_shuttle_route',
                'current_segment': 'A12E',
                'current_s_ratio': 0.72,
                'destination': {
                    'kind': 'interior_loop',
                    'gate_switch': 'A3',
                    'target_segment': 'A34I',
                    'target_s_m': 0.7083,
                    'required_center_spacing_m': 0.57,
                },
            },
        ]
    assert '(normal_route right)' in problem.problem_text
    assert '(= (pending_clearances right) 1)' in problem.problem_text
    assert '(= (clearance_order right_shuttle_2) 1)' in problem.problem_text
    assert '(clearance_destination_ready right_shuttle_2)' in problem.problem_text

    clearance_problem = generator.build_first_blocker_clearance_problem(problem)
    assert clearance_problem.goal_text == (
        '(clearance_relocated right_shuttle_2)'
    )
    assert clearance_problem.provenance['planning_phase'] == (
        'clear_blocker_to_interior_loop'
    )


def test_interior_clearance_problem_has_one_plansys_relocation_goal():
    generator = _load_module()
    spec = generator.scenario_spec_from_case(RIGHT_CASE)
    state = _with_continuous_r2(
        generator,
        generator._observed_state_from_scenario_spec(spec),
        segment='A12E',
        s_ratio=0.72,
    )
    problem = generator.build_pddl_problem_from_observed_state_task_goal(
        state,
        generator._task_goal_from_scenario_spec(spec),
    )
    provenance = dict(problem.provenance)
    clearance = dict(provenance['target_blocker_clearance_plan'])
    relocation = dict(clearance['ordered_relocations'][0])
    relocation['destination'] = {
        'kind': 'interior_loop',
        'gate_switch': 'A3',
        'target_segment': 'A34I',
        'target_s_m': 0.7083,
    }
    clearance['ordered_relocations'] = [relocation]
    provenance['target_blocker_clearance_plan'] = clearance

    clearance_problem = generator.build_first_blocker_clearance_problem(
        replace(problem, provenance=provenance)
    )

    assert clearance_problem.goal_text == '(clearance_relocated right_shuttle_2)'
    assert '(:goal\n    (clearance_relocated right_shuttle_2)' in (
        clearance_problem.problem_text
    )
    assert clearance_problem.provenance['planning_phase'] == (
        'clear_blocker_to_interior_loop'
    )
    assert '(:action relocate_blocker_to_interior' in (
        generator.PDDL_DOMAIN_PATH.read_text(encoding='utf-8')
    )


@pytest.mark.parametrize(
    ('side', 'far_station'),
    (('right', 'staubli'), ('left', 'kuka')),
)
def test_full_rail_clearance_plans_one_verified_relocation_per_observation(
    side,
    far_station,
):
    generator = _load_module()
    spec = generator.ScenarioSpec(
        goal_id=f'{side}-shuttle4-slot4-to-slot2-all-present',
        side=side,
        shuttle=f'{side}_shuttle_4',
        source=far_station,
        target='yaskawa',
        target_slot='2',
        payload_condition='empty',
        start_slots_by_shuttle=tuple(
            (f'{side}_shuttle_{index}', str(index))
            for index in range(1, 5)
        ),
    )

    problem = generator.build_pddl_problem_from_observed_state_task_goal(
        generator._observed_state_from_scenario_spec(spec),
        generator._task_goal_from_scenario_spec(spec),
    )
    relocations = problem.provenance[
        'target_blocker_clearance_plan'
    ]['ordered_relocations']
    clearance = problem.provenance['target_blocker_clearance_plan']

    assert len(relocations) == 1
    destination = relocations[0]['destination']
    # The second blocker is deliberately deferred until a fresh observation,
    # but its known existence must still preserve enough A34I capacity.  The
    # middle single-blocker pose would make both endpoint poses unreachable on
    # the following replan.
    assert destination['target_s_m'] == pytest.approx(0.95)
    assert abs(destination['target_s_m'] - 0.35) >= (
        destination['required_center_spacing_m']
    )
    assert clearance['observed_blocker_count'] == 2
    assert len(clearance['deferred_blockers_require_fresh_reobservation']) == 1
    assert clearance['receding_horizon_clearance'] is True
    assert clearance['stale_multi_destination_preallocation_used'] is False
    assert f'(= (pending_clearances {side}) 1)' in problem.problem_text
    assert f'(= (clearance_order {side}_shuttle_2) 1)' in problem.problem_text
    assert f'(= (clearance_order {side}_shuttle_1) 2)' not in problem.problem_text
    domain = generator.PDDL_DOMAIN_PATH.read_text(encoding='utf-8')
    assert '(:action begin_route_clearance' in domain
    assert '(:action finish_route_clearance' in domain
    assert '(= (pending_clearances ?side) 0)' in domain


@pytest.mark.parametrize('side', ('right', 'left'))
def test_every_full_rail_public_slot_request_builds_without_stale_capacity_failure(
    side,
):
    """Cover all 24 placements x 4 identities x 4 target slots per side."""

    generator = _load_module()
    far_station = 'staubli' if side == 'right' else 'kuka'
    cases = 0
    for placement in itertools.permutations(('1', '2', '3', '4')):
        starts = tuple(
            (f'{side}_shuttle_{identity}', slot)
            for identity, slot in enumerate(placement, start=1)
        )
        for identity in range(1, 5):
            source_slot = placement[identity - 1]
            for target_slot in ('1', '2', '3', '4'):
                spec = generator.ScenarioSpec(
                    goal_id=(
                        f'complete-{side}-{placement}-{identity}-{target_slot}'
                    ),
                    side=side,
                    shuttle=f'{side}_shuttle_{identity}',
                    source=(
                        'yaskawa' if source_slot in {'1', '2'} else far_station
                    ),
                    target=(
                        'yaskawa' if target_slot in {'1', '2'} else far_station
                    ),
                    target_slot=target_slot,
                    payload_condition='empty',
                    start_slots_by_shuttle=starts,
                )
                problem = (
                    generator.build_pddl_problem_from_observed_state_task_goal(
                        generator._observed_state_from_scenario_spec(spec),
                        generator._task_goal_from_scenario_spec(spec),
                    )
                )
                clearance = problem.provenance[
                    'target_blocker_clearance_plan'
                ]
                assert len(clearance['ordered_relocations']) <= 1
                assert clearance['unsupported_if_more_than_two_blockers'] is False
                cases += 1

    assert cases == 384


def test_normal_route_preparation_requires_positive_normal_route_latch():
    generator = _load_module()
    domains_and_actions = {
        generator.PDDL_DOMAIN_PATH: (
            'prepare_switches',
            'open_stoppers',
        ),
        generator.PDDL_DIR / 'domain_room315_runtime.pddl': (
            'prepare_switches',
            'open_stoppers',
        ),
    }

    for domain_path, actions in domains_and_actions.items():
        domain = domain_path.read_text(encoding='utf-8')
        for action in actions:
            start = domain.index(f'(:action {action}')
            end = domain.find('\n  (:action ', start + 1)
            action_text = domain[start:end if end >= 0 else len(domain)]
            assert '(normal_route ?side)' in action_text, (
                domain_path,
                action,
            )


def test_plansys_interior_relocation_is_preserved_by_canonicalization():
    generator = _load_module()
    spec = generator.ScenarioSpec(
        goal_id='canonical-clearance',
        side='right',
        shuttle='right_shuttle_4',
        source='staubli',
        target='yaskawa',
        target_slot='2',
        payload_condition='empty',
        start_slots_by_shuttle=(
            ('right_shuttle_2', '2'),
            ('right_shuttle_4', '4'),
        ),
    )
    problem = generator.build_pddl_problem_from_observed_state_task_goal(
        generator._observed_state_from_scenario_spec(spec),
        generator._task_goal_from_scenario_spec(spec),
    )
    problem = generator.build_first_blocker_clearance_problem(problem)
    plan_msg = SimpleNamespace(items=[SimpleNamespace(
        action=(
            '(relocate_blocker_to_interior right_shuttle_2 '
            'right_shuttle_4 right right_slot_4 right_slot_2)'
        ),
    )])

    plan = generator._symbolic_plan_from_plansys_plan(
        plan_msg,
        spec=None,
        problem=problem,
        speed=0.2,
    )

    assert plan == [
        'relocate_blocker_to_interior right_shuttle_2 right_shuttle_4 '
        'right right_slot_4 right_slot_2 speed=0.2'
    ]


@pytest.mark.parametrize(
    ('segment', 's_ratio'),
    [
        ('A12E', 0.12),
        ('A12I', 0.72),
    ],
)
def test_problem_builder_keeps_route_clear_for_nonblocking_hard_negatives(
    segment,
    s_ratio,
):
    generator = _load_module()
    spec = generator.scenario_spec_from_case(RIGHT_CASE)
    state = _with_continuous_r2(
        generator,
        generator._observed_state_from_scenario_spec(spec),
        segment=segment,
        s_ratio=s_ratio,
    )

    problem = generator.build_pddl_problem_from_observed_state_task_goal(
        state,
        generator._task_goal_from_scenario_spec(spec),
    )

    assert '(route_clear_between right_slot_1 right_slot_3)' in problem.problem_text


def test_problem_builder_maps_left_public_segment_names_to_topology():
    generator = _load_module()
    spec = generator.scenario_spec_from_case(LEFT_CASE)
    state = generator._observed_state_from_scenario_spec(spec)
    facts = [
        replace(fact, value=True)
        if (
            fact.subject == 'room315_left_shuttle_2'
            and fact.predicate == 'present'
        )
        else fact
        for fact in state.fused_planner_state
    ]
    facts.append(generator._planner_fact(
        'room315_left_shuttle_2',
        'rail_position',
        {
            'side': 'left',
            'segment': 'A12E',
            's_ratio': 0.72,
        },
        timestamp=state.timestamp,
        metadata={'segment_naming': 'published_public'},
    ))

    problem = generator.build_pddl_problem_from_observed_state_task_goal(
        replace(state, fused_planner_state=facts),
        generator._task_goal_from_scenario_spec(spec),
    )

    assert '(route_clear_between left_slot_1 left_slot_3)' not in problem.problem_text
    assert (
        '(route_blocked_by left_slot_1 left_slot_3 left_shuttle_2)'
        in problem.problem_text
    )


def test_left_slot1_to_slot2_problem_disables_early_station_completion():
    generator = _load_module()
    spec = replace(
        generator.scenario_spec_from_case(LEFT_CASE),
        goal_id='left-l1-slot1-to-slot2',
        target='yaskawa',
        target_slot='2',
    )
    state = generator._observed_state_from_scenario_spec(spec)
    task_goal = generator.TaskGoal(
        goal_id='left-l1-slot1-to-slot2',
        description='move L1 from left slot 1 to left slot 2',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'selection_strategy': 'explicit',
            'payload_filter': 'any',
            'side': 'left',
            'target_kind': 'slot',
            'target_slot': '2',
            'target_shuttle': 'room315_left_shuttle_1',
        },
    )

    problem = generator.build_pddl_problem_from_observed_state_task_goal(
        state,
        task_goal,
    )

    assert '(shuttle_at_slot left_shuttle_1 left_slot_1)' in problem.problem_text
    assert '(target_station_for_goal left_yaskawa)' in problem.problem_text
    assert '(target_slot_for_goal left_slot_2)' in problem.problem_text
    assert '(station_only_goal)' not in problem.problem_text
    assert '(task_done left_shuttle_1 left_yaskawa)' in problem.goal_text
    assert '(shuttle_at_slot left_shuttle_1 left_slot_2)' in problem.goal_text


def test_problem_builder_supports_inspection_goal_from_validated_state():
    generator = _load_module()
    spec = generator.scenario_spec_from_case(RIGHT_CASE)
    state = generator._observed_state_from_scenario_spec(spec)
    task_goal = generator.TaskGoal(
        goal_id='inspect-right-slot-3',
        description='inspect right slot 3',
        source='human',
        timestamp=0.0,
        confidence=1.0,
        constraints={
            'goal_type': 'inspection',
            'side': 'right',
            'inspection_subject': 'right_slot_3',
        },
    )

    problem = generator.build_pddl_problem_from_observed_state_task_goal(state, task_goal)

    assert '(inspection_required right_slot_3)' in problem.problem_text
    assert problem.goal_text == '(inspection_done right_slot_3)'
    assert '(switch_state_known right_switch_a1)' in problem.problem_text
    assert '(stopper_open right_stopper_a1)' in problem.problem_text


def test_plansys_output_translates_to_primitive_commands_and_event_targets():
    generator = _load_module()
    client = FakePlanSysClient(_right_plansys_actions())
    backend = generator.PlanSysPlannerBackend(planner_client=client)

    scenario = generator.generate_scenario(
        case_id=RIGHT_CASE,
        speed=0.35,
        planner=backend,
    )

    assert [command['action'] for command in scenario['primitive_commands']] == [
        'switches',
        'stoppers',
        'shuttle',
        'shuttle',
        'DONE',
    ]
    assert scenario['primitive_commands'][2]['speed'] == 0.35
    assert [target['primitive'] for target in scenario['expected_event_targets']] == [
        'SET_SWITCHES',
        'SET_STOPPERS',
        'SHUTTLE_ON',
        'STOP_NOW',
        'DONE',
    ]
    assert 'action_vectors' not in scenario


def test_cli_rejects_fallback_backend_before_planning():
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            '--case-id',
            RIGHT_CASE,
            '--planner-backend',
            'fallback',
            '--dry-run',
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert 'invalid choice' in result.stderr
    assert 'fallback' in result.stderr
