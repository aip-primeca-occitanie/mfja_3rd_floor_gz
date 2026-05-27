#!/usr/bin/env python3

import importlib.util
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'config' / 'room_315_vla'
SCRIPTS_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'

TASK_TEMPLATES = {
    'right_yaskawa_to_staubli',
    'right_staubli_to_yaskawa',
    'left_yaskawa_to_kuka',
    'left_kuka_to_yaskawa',
    'right_enter_interior_loop',
    'left_enter_interior_loop',
}


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_vla_supervisor_yaml_contains_only_task_templates():
    config_path = CONFIG_DIR / 'vla_supervisor.yaml'
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    templates = config['route_templates']

    assert set(templates) == TASK_TEMPLATES
    for template in templates.values():
        assert not any(key == 'target' + '_stopper' for key in template)
        assert not any('deprecated' in key for key in template)
        assert not any(key == 'alias' + '_for' for key in template)

    aliases = config['task_aliases']
    assert aliases['move the right shuttle from yaskawa to stäubli'] == 'right_yaskawa_to_staubli'
    assert aliases['make the right shuttle circulate on the interior loop'] == 'right_enter_interior_loop'

    for template_name in ('right_enter_interior_loop', 'left_enter_interior_loop'):
        loop_template = templates[template_name]
        assert loop_template['initial_switches'] == {
            'A1': 'EXTERIOR',
            'A2': 'EXTERIOR',
            'A3': 'INTERIOR',
            'A4': 'INTERIOR',
        }
        assert loop_template['final_switches'] == {
            'A1': 'INTERIOR',
            'A2': 'INTERIOR',
        }
        assert loop_template['keepalive_duration_s'] > 0.0

    assert templates['right_enter_interior_loop']['trigger_sensors'] == ['DA3IR']
    assert templates['right_enter_interior_loop']['trigger_segments'] == ['A34I']
    assert templates['left_enter_interior_loop']['trigger_sensors'] == ['DA3IL', 'DA2L']
    assert templates['left_enter_interior_loop']['trigger_segments'] == ['A23', 'A34I']


def test_action_space_exposes_only_task_template_labels():
    action_space_path = CONFIG_DIR / 'action_space.yaml'
    action_space = yaml.safe_load(action_space_path.read_text(encoding='utf-8'))
    template_ids = action_space['template_ids']

    assert set(template_ids) == {'none', *TASK_TEMPLATES}
    assert not any(template_name.endswith('_demo') for template_name in template_ids)


def test_supervisor_text_parser_returns_task_templates():
    supervisor = _load_module('room_315_vla_supervisor', SCRIPTS_DIR / 'room_315_vla_supervisor.py')
    node = object.__new__(supervisor.Room315VlaSupervisor)
    config = yaml.safe_load((CONFIG_DIR / 'vla_supervisor.yaml').read_text(encoding='utf-8'))
    node.task_aliases = config['task_aliases']

    examples = {
        'move the right shuttle from Yaskawa to Stäubli': 'right_yaskawa_to_staubli',
        'move the right shuttle from Stäubli to Yaskawa': 'right_staubli_to_yaskawa',
        'move the left shuttle from Yaskawa to KUKA': 'left_yaskawa_to_kuka',
        'move the left shuttle from KUKA to Yaskawa': 'left_kuka_to_yaskawa',
        'make the right shuttle circulate on the interior loop': 'right_enter_interior_loop',
        'make the left shuttle circulate on the interior loop': 'left_enter_interior_loop',
    }
    for text, expected_template in examples.items():
        command = node._parse_text_command(text)
        assert command == {
            'action': 'route_template',
            'template': expected_template,
        }


def test_mock_agent_and_recorder_map_natural_language_to_task_labels():
    agent = _load_module('room_315_real_vla_agent', SCRIPTS_DIR / 'room_315_real_vla_agent.py')
    recorder = _load_module(
        'room_315_vla_dataset_recorder',
        SCRIPTS_DIR / 'room_315_vla_dataset_recorder.py',
    )

    assert set(agent.ROUTE_TEMPLATES) == TASK_TEMPLATES
    assert set(recorder.TEMPLATE_IDS) == {'none', *TASK_TEMPLATES}

    text = 'move the right shuttle from Yaskawa to Staubli'.casefold()
    assert agent._task_template_for_text(text, 'right', 'exterior') == 'right_yaskawa_to_staubli'
    assert recorder._task_template_for_text(text) == 'right_yaskawa_to_staubli'

    loop_text = 'make right shuttle circulate on the interior loop'.casefold()
    assert agent._task_template_for_text(loop_text, 'right', 'interior') == 'right_enter_interior_loop'
    assert recorder._task_template_for_text(loop_text) == 'right_enter_interior_loop'

    assert not any(template_name.endswith('_demo') for template_name in agent.ROUTE_TEMPLATES)
    assert not any(template_name.endswith('_demo') for template_name in recorder.TEMPLATE_IDS)
