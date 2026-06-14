#!/usr/bin/env python3

import ast
from collections import Counter
import importlib.util
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'scripts'
    / 'room_315_pddl_scenario_generator.py'
)
PDDL_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'config' / 'room_315_vla' / 'pddl'
BATCH_CONFIG_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'config'
    / 'room_315_vla'
    / 'pddl_scenario_batch.yaml'
)


class FakeTransport:
    def __init__(self, decisions=None):
        self.decisions = list(decisions or [])
        self.command_messages = []
        self.episode_controls = []
        self.count = 0

    def publish_episode_control(self, command):
        self.episode_controls.append(command)

    def publish_command(self, command):
        self.command_messages.append(command)

    def supervisor_decision_count(self):
        return self.count

    def wait_for_supervisor_decision(self, *, previous_count, timeout_s):
        self.count = max(self.count, previous_count) + 1
        if self.decisions:
            return self.decisions.pop(0)
        return {'accepted': True, 'reason': ''}


class NotReadyTransport(FakeTransport):
    def wait_until_ready(self, *, timeout_s):
        return {'ready': False, 'reason': 'no supervisor subscriber on /room_315/vla/command'}


class ArrivalTrackingTransport(FakeTransport):
    def __init__(self, arrival_result=None):
        super().__init__()
        self.arrival_result = arrival_result or {'arrived': True, 'matched_sensors': ['DZI3R']}
        self.arrival_waits = []
        self.event_order = []

    def publish_command(self, command):
        super().publish_command(command)
        self.event_order.append(('command', command.get('action'), command.get('command')))

    def wait_for_target_arrival(self, *, side, target_sensors, shuttle, timeout_s):
        self.arrival_waits.append({
            'side': side,
            'target_sensors': list(target_sensors),
            'shuttle': shuttle,
            'timeout_s': timeout_s,
        })
        self.event_order.append(('arrival_wait', list(target_sensors)))
        return dict(self.arrival_result)


class FakePlanSysBackend:
    def __init__(self):
        self.calls = []

    def plan(self, spec, *, speed):
        self.calls.append({'goal_id': spec.goal_id, 'speed': float(speed)})
        return [
            f'prepare_switches {spec.side} {spec.source} {spec.target}',
            f'open_stoppers {spec.side} {spec.source} {spec.target}',
            (
                f'move_shuttle {spec.side} {spec.shuttle} '
                f'{spec.source} {spec.target} speed={float(speed):.4g}'
            ),
            f'stop_shuttle {spec.side} {spec.shuttle}',
            f'finish_task {spec.shuttle} {spec.target}',
        ]


class FakePlanSysClient:
    def __init__(self, actions):
        self.actions = list(actions)

    def get_plan(self, *, domain, problem):
        class Item:
            def __init__(self, action):
                self.action = action

        class Plan:
            pass

        plan = Plan()
        plan.items = [Item(action) for action in self.actions]
        return plan


def _fake_backend():
    return FakePlanSysBackend()


def _domain_order_right_plan():
    return [
        '(prepare_switches right right_yaskawa right_staubli right_switch_group)',
        '(open_stoppers right right_yaskawa right_staubli right_stopper_group)',
        '(move_shuttle right_shuttle right right_yaskawa right_staubli)',
        '(stop_shuttle right_shuttle right right_yaskawa right_staubli)',
        '(finish_task right_shuttle right_staubli)',
    ]


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_pddl_scenario_generator', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_dry_run_right_yaskawa_to_staubli_produces_ordered_plan():
    generator = _load_module()

    scenario = generator.generate_scenario(
        goal='right_yaskawa_to_staubli',
        language_seed=42,
        planner=_fake_backend(),
    )

    assert scenario['scenario_id'] == 'right_yaskawa_to_staubli'
    assert scenario['pddl_goal'] == 'right_shuttle at staubli'
    assert scenario['symbolic_plan'] == [
        'prepare_switches right yaskawa staubli',
        'open_stoppers right yaskawa staubli',
        'move_shuttle right right_shuttle yaskawa staubli speed=0.3',
        'stop_shuttle right right_shuttle',
        'finish_task right_shuttle staubli',
    ]
    assert [event['primitive'] for event in scenario['expected_event_targets']] == [
        'SET_SWITCHES',
        'SET_STOPPERS',
        'SHUTTLE_ON',
        'STOP_NOW',
        'DONE',
    ]


def test_dry_run_left_yaskawa_to_kuka_produces_left_side_commands():
    generator = _load_module()

    scenario = generator.generate_scenario(
        problem=PDDL_DIR / 'problem_left_yaskawa_to_kuka.pddl',
        language_seed=1,
        planner=_fake_backend(),
    )

    assert scenario['scenario_id'] == 'left_yaskawa_to_kuka'
    assert 'problem_left_yaskawa_to_kuka.pddl' in scenario['pddl_problem']
    assert all(
        command.get('side') == 'left'
        for command in scenario['primitive_commands']
        if command['action'] != 'DONE'
    )
    assert scenario['primitive_commands'][2]['shuttle'] == 'left_shuttle'
    assert scenario['expected_event_targets'][2]['target_id'] == 'left_shuttle'


def test_generated_plan_includes_language():
    generator = _load_module()

    scenario = generator.generate_scenario(
        goal='left_yaskawa_to_kuka',
        language_template_id='move_from_to',
        planner=_fake_backend(),
    )

    assert scenario['language'] == 'move the left shuttle from Yaskawa to KUKA'
    assert scenario['generated_language_template_id'] == 'move_from_to'
    assert 'pddl' not in scenario['language'].casefold()
    assert '(' not in scenario['language']


def test_generated_plan_includes_primitive_commands():
    generator = _load_module()

    scenario = generator.generate_scenario(
        goal='right_staubli_to_yaskawa',
        planner=_fake_backend(),
    )

    assert [command['action'] for command in scenario['primitive_commands']] == [
        'switches',
        'stoppers',
        'shuttle',
        'shuttle',
        'DONE',
    ]
    assert scenario['primitive_commands'][2]['command'] == 'ON'
    assert scenario['primitive_commands'][3]['command'] == 'OFF'
    assert len(scenario['action_vectors']) == len(scenario['expected_event_targets'])


def test_plansys_domain_order_plan_generates_primitive_commands_and_vectors():
    generator = _load_module()
    backend = generator.PlanSysPlannerBackend(
        planner_client=FakePlanSysClient(_domain_order_right_plan())
    )

    scenario = generator.generate_scenario(
        goal='right_yaskawa_to_staubli',
        speed=0.44,
        planner=backend,
    )

    assert scenario['symbolic_plan'] == [
        'prepare_switches right yaskawa staubli',
        'open_stoppers right yaskawa staubli',
        'move_shuttle right right_shuttle yaskawa staubli speed=0.44',
        'stop_shuttle right right_shuttle',
        'finish_task right_shuttle staubli',
    ]
    assert scenario['primitive_commands'][2]['speed'] == 0.44
    assert [target['primitive'] for target in scenario['expected_event_targets']] == [
        'SET_SWITCHES',
        'SET_STOPPERS',
        'SHUTTLE_ON',
        'STOP_NOW',
        'DONE',
    ]
    assert scenario['action_vectors'][2][0] == 4.0


def test_dry_run_imports_ros_lazily_and_can_use_mocked_plansys():
    text = SCRIPT_PATH.read_text(encoding='utf-8')
    module = ast.parse(text)
    imported_modules = set()
    for node in ast.walk(module):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name.split('.', 1)[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_modules.add(node.module.split('.', 1)[0])

    assert 'rclpy' not in imported_modules
    scenario = _load_module().generate_scenario(
        goal='right_yaskawa_to_staubli',
        planner=_fake_backend(),
    )
    assert scenario['primitive_commands'][0]['action'] == 'switches'


def test_dry_run_does_not_modify_files_unless_output_is_provided(tmp_path):
    before = sorted(path.name for path in tmp_path.iterdir())
    generator = _load_module()
    scenario = generator.generate_scenario(
        goal='right_yaskawa_to_staubli',
        language_seed=42,
        planner=_fake_backend(),
    )
    after = sorted(path.name for path in tmp_path.iterdir())

    assert before == []
    assert after == []
    assert scenario['scenario_id'] == 'right_yaskawa_to_staubli'

    output = tmp_path / 'planned_episode.json'
    generator.write_scenario(output, scenario)

    assert sorted(path.name for path in tmp_path.iterdir()) == ['planned_episode.json']
    assert json.loads(output.read_text(encoding='utf-8'))['scenario_id'] == (
        'right_yaskawa_to_staubli'
    )


def test_execute_mode_publishes_episode_start():
    generator = _load_module()
    scenario = generator.generate_scenario(
        goal='right_yaskawa_to_staubli',
        language_template_id='move_from_to',
        planner=_fake_backend(),
    )
    transport = FakeTransport()

    result = generator.execute_scenario(scenario, transport)

    assert result['success'] is True
    assert transport.episode_controls[0] == 'start move the right shuttle from Yaskawa to Staubli'


def test_execute_mode_publishes_commands_in_plan_order():
    generator = _load_module()
    scenario = generator.generate_scenario(
        goal='right_yaskawa_to_staubli',
        planner=_fake_backend(),
    )
    transport = FakeTransport()

    generator.execute_scenario(scenario, transport)

    assert [message.get('action') for message in transport.command_messages[:-1]] == [
        'switches',
        'stoppers',
        'shuttle',
        'shuttle',
    ]
    assert all('action_vector' in message for message in transport.command_messages)
    assert transport.command_messages[-1]['action'] == 'DONE'
    assert 'action_vector' in transport.command_messages[-1]
    assert [message['plan_step_index'] for message in transport.command_messages] == [0, 1, 2, 3, 4]
    assert transport.command_messages[0]['planning_source'] == 'pddl'
    assert transport.command_messages[2]['command'] == 'ON'
    assert transport.command_messages[3]['command'] == 'OFF'


def test_execute_mode_waits_for_target_sensor_before_stop_command():
    generator = _load_module()
    scenario = generator.generate_scenario(
        goal='right_yaskawa_to_staubli',
        planner=_fake_backend(),
    )
    transport = ArrivalTrackingTransport()

    result = generator.execute_scenario(
        scenario,
        transport,
        arrival_timeout_s=17.0,
    )

    assert result['success'] is True
    assert transport.arrival_waits == [{
        'side': 'right',
        'target_sensors': ['DZI3R', 'DZI4R'],
        'shuttle': 'right_shuttle',
        'timeout_s': 17.0,
    }]
    on_index = next(
        index
        for index, event in enumerate(transport.event_order)
        if event == ('command', 'shuttle', 'ON')
    )
    wait_index = next(
        index
        for index, event in enumerate(transport.event_order)
        if event[0] == 'arrival_wait'
    )
    off_index = next(
        index
        for index, event in enumerate(transport.event_order)
        if event == ('command', 'shuttle', 'OFF')
    )
    assert on_index < wait_index < off_index


def test_execute_mode_target_arrival_timeout_stops_before_off_command():
    generator = _load_module()
    scenario = generator.generate_scenario(
        goal='right_yaskawa_to_staubli',
        planner=_fake_backend(),
    )
    transport = ArrivalTrackingTransport(
        arrival_result={'arrived': False, 'reason': 'timeout waiting for DZI3R'}
    )

    result = generator.execute_scenario(scenario, transport)

    assert result['success'] is False
    assert result['failed_step_index'] == 2
    assert result['failure_reason'] == 'timeout waiting for DZI3R'
    assert transport.episode_controls[-1] == 'stop failure'
    assert [message.get('action') for message in transport.command_messages] == [
        'switches',
        'stoppers',
        'shuttle',
    ]
    assert transport.command_messages[-1]['command'] == 'ON'


def test_rejected_supervisor_response_stops_episode_with_failure():
    generator = _load_module()
    scenario = generator.generate_scenario(
        goal='right_yaskawa_to_staubli',
        planner=_fake_backend(),
    )
    transport = FakeTransport(decisions=[
        {'accepted': True},
        {'accepted': False, 'reason': 'unsafe switch change'},
        {'accepted': True},
    ])

    result = generator.execute_scenario(scenario, transport)

    assert result['success'] is False
    assert result['failed_step_index'] == 1
    assert result['failure_reason'] == 'unsafe switch change'
    assert transport.episode_controls[-1] == 'stop failure'
    assert len(transport.command_messages) == 2


def test_execute_mode_fails_clearly_when_supervisor_not_ready():
    generator = _load_module()
    scenario = generator.generate_scenario(
        goal='right_yaskawa_to_staubli',
        planner=_fake_backend(),
    )
    transport = NotReadyTransport()

    result = generator.execute_scenario(scenario, transport)

    assert result['success'] is False
    assert result['failed_step_index'] is None
    assert 'no supervisor subscriber' in result['failure_reason']
    assert transport.episode_controls == []
    assert transport.command_messages == []


def test_success_path_stops_episode_with_success():
    generator = _load_module()
    scenario = generator.generate_scenario(
        goal='left_yaskawa_to_kuka',
        planner=_fake_backend(),
    )
    transport = FakeTransport()

    result = generator.execute_scenario(scenario, transport)

    assert result['success'] is True
    assert transport.episode_controls[-1] == 'stop success'
    assert len(transport.command_messages) == len(scenario['symbolic_plan'])


def test_batch_dry_run_creates_n_planned_episodes():
    generator = _load_module()

    batch = generator.generate_batch_scenarios(
        generator.load_batch_config(BATCH_CONFIG_PATH),
        planner=_fake_backend(),
    )

    assert batch['planned_episode_count'] == 12
    assert Counter(episode['scenario_id'] for episode in batch['episodes']) == {
        'right_yaskawa_to_staubli': 3,
        'right_staubli_to_yaskawa': 3,
        'left_yaskawa_to_kuka': 3,
        'left_kuka_to_yaskawa': 3,
    }
    assert all(episode['symbolic_plan'] for episode in batch['episodes'])


def test_batch_respects_repetitions_per_goal():
    generator = _load_module()

    batch = generator.generate_batch_scenarios({
        'goals': ['right_yaskawa_to_staubli', 'left_yaskawa_to_kuka'],
        'repetitions_per_goal': 3,
        'speed_values': [0.3],
    }, planner=_fake_backend())

    assert batch['planned_episode_count'] == 6
    assert Counter(episode['scenario_id'] for episode in batch['episodes']) == {
        'right_yaskawa_to_staubli': 3,
        'left_yaskawa_to_kuka': 3,
    }


def test_batch_produces_language_paraphrases():
    generator = _load_module()

    batch = generator.generate_batch_scenarios({
        'goals': ['right_yaskawa_to_staubli'],
        'repetitions_per_goal': 4,
        'language_seed': 0,
        'speed_values': [0.3],
    }, planner=_fake_backend())
    languages = [episode['language'] for episode in batch['episodes']]

    assert len(set(languages)) == 4
    assert [episode['generated_language_template_id'] for episode in batch['episodes']] == [
        'move_from_to',
        'send_to_station',
        'route_between_stations',
        'bring_to_station',
    ]
    assert all('pddl' not in language.casefold() for language in languages)


def test_batch_speed_values_propagate_to_shuttle_on_commands():
    generator = _load_module()

    batch = generator.generate_batch_scenarios({
        'goals': ['right_yaskawa_to_staubli'],
        'repetitions_per_goal': 3,
        'speed_values': [0.2, 0.3, 0.5],
    }, planner=_fake_backend())
    speeds = []
    for episode in batch['episodes']:
        shuttle_on = [
            command
            for command in episode['primitive_commands']
            if command.get('action') == 'shuttle' and command.get('command') == 'ON'
        ]
        speeds.append(shuttle_on[0]['speed'])

    assert speeds == [0.2, 0.3, 0.5]


def test_batch_keeps_model_input_free_of_speed_and_pddl_internals():
    generator = _load_module()

    batch = generator.generate_batch_scenarios({
        'goals': ['right_yaskawa_to_staubli'],
        'repetitions_per_goal': 2,
        'speed_values': [0.2, 0.5],
    }, planner=_fake_backend())

    forbidden = {
        'speed',
        'speed_values',
        'batch_speed_mps',
        'pddl_domain',
        'pddl_problem',
        'pddl_goal',
        'symbolic_plan',
        'plan_step_index',
    }
    for episode in batch['episodes']:
        assert 'model_input' not in episode
        for payload in generator.command_payloads_for_execution(episode):
            assert 'model_input' not in payload
        model_input = {
            'language': episode['language'],
            'overhead_images': {},
            'last_command': {'action': 'START'},
        }
        assert set(model_input) == {'language', 'overhead_images', 'last_command'}
        serialized_model_input = json.dumps(model_input, sort_keys=True)
        for field in forbidden:
            assert field not in model_input
            assert field not in serialized_model_input


def test_batch_execute_uses_supervisor_command_path():
    generator = _load_module()
    batch = generator.generate_batch_scenarios({
        'goals': ['right_yaskawa_to_staubli', 'left_yaskawa_to_kuka'],
        'repetitions_per_goal': 1,
        'speed_values': [0.3],
    }, planner=_fake_backend())
    transport = FakeTransport()

    result = generator.execute_batch_scenarios(batch, transport)

    assert result['success'] is True
    assert result['completed_episode_count'] == 2
    assert len(transport.command_messages) == 10
    assert transport.episode_controls[0].startswith('start ')
    assert transport.episode_controls[1] == 'stop success'
    assert transport.episode_controls[2].startswith('start ')
    assert transport.episode_controls[3] == 'stop success'
    assert all(message['planning_source'] == 'pddl' for message in transport.command_messages)
    assert all('action_vector' in message for message in transport.command_messages)
    assert [message['action'] for message in transport.command_messages[:5]] == [
        'switches',
        'stoppers',
        'shuttle',
        'shuttle',
        'DONE',
    ]
