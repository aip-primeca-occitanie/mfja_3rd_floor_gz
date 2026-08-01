#!/usr/bin/env python3

import importlib.util
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
        '(move_shuttle right_shuttle_1 right right_yaskawa right_staubli)',
        '(stop_shuttle right_shuttle_1 right right_yaskawa right_staubli)',
        '(finish_task right_shuttle_1 right_staubli)',
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
        'prepare_switches right yaskawa staubli',
        'open_stoppers right yaskawa staubli',
        'move_shuttle right right_shuttle_1 yaskawa staubli speed=0.41',
        'stop_shuttle right right_shuttle_1',
        'finish_task right_shuttle_1 staubli',
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
        'prepare_switches right yaskawa staubli',
        'open_stoppers right yaskawa staubli',
        'move_shuttle right right_shuttle_1 yaskawa staubli speed=0.29',
        'stop_shuttle right right_shuttle_1',
        'finish_task right_shuttle_1 staubli',
    ]


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
    assert '(transport_goal_done right_staubli)' in call['problem']
    assert '(goal_slot_reached right_slot_3)' in call['problem']
    assert '(= (route_cost right_slot_1 right_slot_3) 2)' in call['problem']
    assert 'right_shuttle_1 right_shuttle_2 right_shuttle_3 right_shuttle_4' in call['problem']
    assert '(slot_occupied_by right_slot_1 right_shuttle_1)' in call['problem']
    assert '(block_free right_block_slot_3)' in call['problem']


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


def test_two_interior_blockers_receive_physically_separated_staging_poses():
    generator = _load_module()
    spec = generator.ScenarioSpec(
        goal_id='r4-slot4-to-slot2-all-right-present',
        side='right',
        shuttle='right_shuttle_4',
        source='staubli',
        target='yaskawa',
        target_slot='2',
        payload_condition='empty',
        start_slots_by_shuttle=tuple(
            (f'right_shuttle_{index}', str(index))
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
    targets = [item['destination']['target_s_m'] for item in relocations]
    required_spacing = relocations[0]['destination'][
        'required_center_spacing_m'
    ]

    assert targets == [0.95, 0.35]
    assert abs(targets[0] - targets[1]) >= required_spacing
    assert '(= (pending_clearances right) 2)' in problem.problem_text
    assert '(= (clearance_order right_shuttle_2) 1)' in problem.problem_text
    assert '(= (clearance_order right_shuttle_1) 2)' in problem.problem_text
    domain = generator.PDDL_DOMAIN_PATH.read_text(encoding='utf-8')
    assert '(:action begin_route_clearance' in domain
    assert '(:action finish_route_clearance' in domain
    assert '(= (pending_clearances ?side) 0)' in domain


def test_normal_route_preparation_requires_positive_normal_route_latch():
    generator = _load_module()
    domains_and_actions = {
        generator.PDDL_DOMAIN_PATH: (
            'prepare_switches',
            'open_stoppers',
            'prepare_switches_for_shuttle',
            'open_stoppers_for_shuttle',
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
