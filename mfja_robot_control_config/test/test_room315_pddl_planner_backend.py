#!/usr/bin/env python3

import importlib.util
import subprocess
import sys
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
    assert '(task_done right_shuttle_1 right_staubli)' in call['problem']


def test_plansys_output_translates_to_primitive_commands_and_action_vectors():
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
    assert len(scenario['action_vectors']) == 5
    assert scenario['action_vectors'][2][0] == 4.0


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
