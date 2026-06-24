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

    def shutdown(self):
        pass


class NotReadyTransport(FakeTransport):
    def wait_until_ready(self, *, timeout_s):
        return {'ready': False, 'reason': 'no supervisor subscriber on /room_315/vla/command'}


class InitialStateTransport(FakeTransport):
    def __init__(self, initial_state):
        super().__init__()
        self.initial_state = dict(initial_state)

    def wait_until_ready(self, *, timeout_s):
        return {'ready': True, 'reason': ''}

    def wait_for_initial_scenario_state(self, *, scenario, timeout_s):
        return dict(self.initial_state)


class TimeoutTransport(FakeTransport):
    def wait_for_supervisor_decision(self, *, previous_count, timeout_s):
        return None


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


class RecorderAckTransport(FakeTransport):
    def __init__(self):
        super().__init__()
        self.recorder_waits = []

    def wait_for_episode_started(self, *, goal, timeout_s):
        self.recorder_waits.append(('started', goal, timeout_s))
        return {'ready': True, 'reason': '', 'observed': True}

    def wait_for_episode_stopped(self, *, timeout_s):
        self.recorder_waits.append(('stopped', timeout_s))
        return {'ready': True, 'reason': '', 'observed': True}


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


class _Endpoint:
    def __init__(self, node_name):
        self.node_name = node_name


class _FakePublisher:
    def __init__(self, subscription_count):
        self.subscription_count = subscription_count

    def get_subscription_count(self):
        return self.subscription_count


class _FakeRclpy:
    def spin_once(self, node, timeout_sec=0.0):
        return None


class _FakeNode:
    def __init__(self, endpoints, publisher_endpoints=None):
        self.endpoints = list(endpoints)
        self.publisher_endpoints = list(publisher_endpoints or [])

    def get_subscriptions_info_by_topic(self, topic):
        return list(self.endpoints)

    def get_publishers_info_by_topic(self, topic):
        return list(self.publisher_endpoints)


def _ros_transport_shell(
    generator,
    *,
    endpoints,
    subscription_count=1,
    latest_status=None,
    publisher_endpoints=None,
):
    transport = generator.RosScenarioTransport.__new__(generator.RosScenarioTransport)
    transport.command_topic = '/room_315/vla/command'
    transport.dataset_status_topic = '/room_315/vla/dataset_status'
    transport.status_topic = '/room_315/vla/status'
    transport.command_pub = _FakePublisher(subscription_count)
    transport.rclpy = _FakeRclpy()
    transport.node = _FakeNode(endpoints, publisher_endpoints=publisher_endpoints)
    transport.latest_status = latest_status or {'last_result': 'initialized'}
    transport.latest_dataset_status = {}
    return transport


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


def test_loaded_payload_goal_selects_r2_and_records_metadata_outside_model_input():
    generator = _load_module()

    scenario = generator.generate_scenario(
        goal='right_loaded_r2_to_staubli',
        language_template_id='carrying_part_id_to_station',
        planner=_fake_backend(),
    )

    assert scenario['scenario_id'] == 'right_loaded_r2_to_staubli'
    assert scenario['pddl_goal'] == 'loaded right_shuttle_2 at staubli'
    assert scenario['payload_condition'] == 'loaded'
    assert scenario['target_shuttle_id'] == 'right_shuttle_2'
    assert scenario['symbolic_plan'][2] == (
        'move_shuttle right right_shuttle_2 yaskawa staubli speed=0.3'
    )
    assert scenario['primitive_commands'][2]['shuttle'] == 'right_shuttle_2'
    assert scenario['expected_event_targets'][2]['target_id'] == 'right_shuttle_2'
    assert scenario['expected_event_targets'][2]['shuttle_id'] == 'R2'
    assert scenario['expected_event_targets'][2]['shuttle_index'] == 1
    assert scenario['payload_state']['by_shuttle']['right_shuttle_2']['loaded'] is True
    assert scenario['payload_state']['model_input_exposure'] == 'excluded'
    assert 'model_input' not in scenario

    payloads = generator.command_payloads_for_execution(scenario)
    assert payloads[2]['payload_condition'] == 'loaded'
    assert payloads[2]['payload_present'] is True
    assert payloads[2]['payload_type'] == 'box'
    assert payloads[2]['target_shuttle_id'] == 'right_shuttle_2'


def test_nearest_loaded_slot_goal_selects_closest_loaded_shuttle():
    generator = _load_module()

    scenario = generator.generate_scenario(
        goal='right_loaded_to_slot3',
        planner=_fake_backend(),
    )

    assert scenario['scenario_id'] == 'right_loaded_to_slot3'
    assert scenario['language'] == 'move the loaded right shuttle to slot 3'
    assert scenario['generated_language_template_id'] == 'loaded_shuttle_to_slot'
    assert scenario['target_slot'] == '3'
    assert scenario['target_shuttle_id'] == 'right_shuttle_2'
    assert scenario['pddl_goal'] == 'loaded right_shuttle_2 at staubli'
    assert scenario['symbolic_plan'][2] == (
        'move_shuttle right right_shuttle_2 yaskawa staubli speed=0.3'
    )

    candidates = scenario['selection_candidates']
    assert [candidate['shuttle_id'] for candidate in candidates] == [
        'right_shuttle_2',
        'right_shuttle_1',
    ]
    assert candidates[0]['selected'] is True
    assert candidates[0]['distance_to_target_slot'] == 1
    assert candidates[1]['distance_to_target_slot'] == 2
    assert scenario['payload_state']['by_shuttle']['right_shuttle_1']['loaded'] is True
    assert scenario['payload_state']['by_shuttle']['right_shuttle_2']['loaded'] is True
    assert scenario['payload_state']['by_shuttle']['right_shuttle_1']['start_slot'] == '1'
    assert scenario['payload_state']['by_shuttle']['right_shuttle_2']['start_slot'] == '2'
    assert scenario['payload_state']['model_input_exposure'] == 'excluded'
    assert 'model_input' not in scenario

    payloads = generator.command_payloads_for_execution(scenario)
    assert payloads[2]['target_slot'] == '3'
    assert payloads[2]['selection_policy'] == 'nearest_loaded_to_target_slot_then_lowest_id'
    assert payloads[2]['selection_candidates'][0]['shuttle_id'] == 'right_shuttle_2'
    assert 'model_input' not in payloads[2]


def test_nearest_loaded_slot_goal_waits_for_slot_sensor_only():
    generator = _load_module()
    scenario = generator.generate_scenario(
        goal='right_loaded_to_slot3',
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
        'target_sensors': ['DZI3R'],
        'shuttle': 'right_shuttle_2',
        'timeout_s': 17.0,
    }]


def test_blocker_clear_goal_moves_empty_blocker_before_loaded_shuttle():
    generator = _load_module()

    scenario = generator.generate_scenario(
        goal='right_loaded_to_slot3_clear_blocker',
        planner=_fake_backend(),
    )

    assert scenario['scenario_id'] == 'right_loaded_to_slot3_clear_blocker'
    assert scenario['language'] == 'move the loaded right shuttle to slot 3'
    assert scenario['target_shuttle_id'] == 'right_shuttle_2'
    assert scenario['target_slot'] == '3'
    assert scenario['blocker_clearance'] == {
        'strategy': 'clear_blocker_move_loaded_then_restore_blocker_to_free_slot',
        'phase': 'clear_blocker_move_selected_restore_blocker',
        'blocker_shuttle_id': 'right_shuttle_1',
        'blocker_start_slot': '3',
        'blocker_clear_slot': '1',
        'blocker_restore_slot': '2',
        'blocker_final_slot': '2',
        'blocker_restore_policy': 'selected_source_slot_then_nearest_free_slot',
        'blocker_restore_slot_source': 'selected_source_slot',
        'blocker_restore_candidate_slots': ['2', '1', '3', '4'],
        'selected_shuttle_id': 'right_shuttle_2',
        'selected_target_slot': '3',
        'restore_deferred': False,
        'model_input_exposure': 'excluded',
    }
    assert scenario['symbolic_plan'] == [
        'prepare_switches right staubli yaskawa',
        'open_stoppers right staubli yaskawa',
        'move_shuttle right right_shuttle_1 staubli yaskawa speed=0.3',
        'stop_shuttle right right_shuttle_1',
        'prepare_switches right yaskawa staubli',
        'open_stoppers right yaskawa staubli',
        'move_shuttle right right_shuttle_2 yaskawa staubli speed=0.3',
        'stop_shuttle right right_shuttle_2',
        'prepare_switches right yaskawa yaskawa',
        'open_stoppers right yaskawa yaskawa',
        'move_shuttle right right_shuttle_1 yaskawa yaskawa speed=0.3',
        'stop_shuttle right right_shuttle_1',
        'finish_task right_shuttle_2 staubli',
    ]
    assert scenario['payload_state']['by_shuttle']['right_shuttle_1']['loaded'] is False
    assert scenario['payload_state']['by_shuttle']['right_shuttle_2']['loaded'] is True
    assert scenario['payload_state']['by_shuttle']['right_shuttle_1']['start_slot'] == '3'
    assert scenario['payload_state']['by_shuttle']['right_shuttle_2']['start_slot'] == '2'

    payloads = generator.command_payloads_for_execution(scenario)
    assert payloads[2]['shuttle'] == 'right_shuttle_1'
    assert payloads[2]['coordination_phase'] == 'clear_blocker'
    assert payloads[2]['target_slot'] == '1'
    assert payloads[2]['payload_condition'] == 'empty'
    assert payloads[2]['payload_present'] is False
    assert payloads[6]['shuttle'] == 'right_shuttle_2'
    assert payloads[6]['coordination_phase'] == 'move_selected_loaded'
    assert payloads[6]['target_slot'] == '3'
    assert payloads[6]['payload_condition'] == 'loaded'
    assert payloads[6]['payload_present'] is True
    assert payloads[10]['shuttle'] == 'right_shuttle_1'
    assert payloads[10]['coordination_phase'] == 'restore_blocker'
    assert payloads[10]['target_slot'] == '2'
    assert payloads[10]['payload_condition'] == 'empty'
    assert payloads[10]['payload_present'] is False
    assert payloads[10]['blocker_clearance']['blocker_restore_policy'] == (
        'selected_source_slot_then_nearest_free_slot'
    )
    assert payloads[10]['blocker_clearance']['blocker_restore_slot_source'] == (
        'selected_source_slot'
    )
    assert 'model_input' not in scenario
    assert all('model_input' not in payload for payload in payloads)


def test_blocker_restore_policy_falls_back_to_nearest_free_slot():
    generator = _load_module()

    restore = generator._resolve_blocker_restore_slot(
        side='right',
        selected_shuttle='right_shuttle_2',
        selected_start_slot='1',
        target_slot='3',
        start_slots_by_shuttle={
            'right_shuttle_1': '3',
            'right_shuttle_2': '1',
            'right_shuttle_3': '2',
        },
        data={
            'blocker_shuttle': 'right_shuttle_1',
            'blocker_clear_slot': '1',
            'blocker_restore_slot': 'auto',
            'blocker_restore_policy': 'selected_source_slot_then_nearest_free_slot',
        },
    )

    assert restore['blocker_restore_slot'] == '4'
    assert restore['blocker_restore_slot_source'] == 'nearest_free_slot'
    assert restore['blocker_restore_candidate_slots'] == ('1', '2', '3', '4')


def test_blocker_clear_goal_waits_for_blocker_slot_then_loaded_target_slot():
    generator = _load_module()
    scenario = generator.generate_scenario(
        goal='right_loaded_to_slot3_clear_blocker',
        planner=_fake_backend(),
    )
    transport = ArrivalTrackingTransport()

    result = generator.execute_scenario(
        scenario,
        transport,
        arrival_timeout_s=19.0,
    )

    assert result['success'] is True
    assert transport.arrival_waits == [
        {
            'side': 'right',
            'target_sensors': ['DZI1R'],
            'shuttle': 'right_shuttle_1',
            'timeout_s': 19.0,
        },
        {
            'side': 'right',
            'target_sensors': ['DZI3R'],
            'shuttle': 'right_shuttle_2',
            'timeout_s': 19.0,
        },
        {
            'side': 'right',
            'target_sensors': ['DZI2R'],
            'shuttle': 'right_shuttle_1',
            'timeout_s': 19.0,
        },
    ]


def test_payload_problem_file_round_trips_to_specific_goal_spec():
    generator = _load_module()

    spec = generator.scenario_spec_from_problem(PDDL_DIR / 'problem_right_loaded_r2_to_staubli.pddl')
    problem_text = generator._problem_text_from_goal_spec(spec)

    assert spec.goal_id == 'right_loaded_r2_to_staubli'
    assert spec.shuttle == 'right_shuttle_2'
    assert spec.payload_condition == 'loaded'
    assert '(loaded right_shuttle_2)' in problem_text
    assert '(carrying_payload right_shuttle_2)' in problem_text


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
    assert scenario['expected_event_targets'][2]['target_id'] == 'left_shuttle_1'
    assert scenario['expected_event_targets'][2]['shuttle_id'] == 'L1'
    assert scenario['expected_event_targets'][2]['shuttle_index'] == 0


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


def test_ros_ready_requires_the_supervisor_command_subscriber():
    generator = _load_module()
    transport = _ros_transport_shell(
        generator,
        endpoints=[_Endpoint('room_315_vla_dataset_recorder')],
        subscription_count=1,
    )

    result = generator.RosScenarioTransport.wait_until_ready(transport, timeout_s=0.01)

    assert result['ready'] is False
    assert 'no room_315_vla_supervisor subscriber' in result['reason']
    assert '1 total subscriber' in result['reason']


def test_ros_ready_accepts_the_supervisor_command_subscriber():
    generator = _load_module()
    transport = _ros_transport_shell(
        generator,
        endpoints=[
            _Endpoint('room_315_vla_dataset_recorder'),
            _Endpoint('room_315_vla_supervisor'),
        ],
        subscription_count=2,
    )

    result = generator.RosScenarioTransport.wait_until_ready(transport, timeout_s=0.01)

    assert result == {'ready': True, 'reason': ''}


def test_ros_initial_state_wait_accepts_loaded_target_shuttle():
    generator = _load_module()
    scenario = generator.generate_scenario(
        goal='right_loaded_r2_to_staubli',
        planner=_fake_backend(),
    )
    transport = _ros_transport_shell(
        generator,
        endpoints=[_Endpoint('room_315_vla_supervisor')],
        latest_status={
            'rails': {
                'right': {
                    'shuttles': {
                        'room315_right_shuttle_2': {'mode': 'STOPPED'},
                    },
                    'payloads': {
                        'room315_right_shuttle_2': {
                            'loaded': True,
                            'payload_type': 'box',
                        },
                    },
                },
            },
            'payload_state': {
                'by_shuttle': {
                    'room315_right_shuttle_2': {
                        'loaded': True,
                        'payload_type': 'box',
                    },
                },
            },
        },
    )

    result = generator.RosScenarioTransport.wait_for_initial_scenario_state(
        transport,
        scenario=scenario,
        timeout_s=0.01,
    )

    assert result['ready'] is True
    assert result['target_shuttle'] == 'room315_right_shuttle_2'
    assert result['payload_condition'] == 'loaded'


def test_ros_initial_state_wait_reports_missing_target_shuttle_before_execute():
    generator = _load_module()
    scenario = generator.generate_scenario(
        goal='right_loaded_r2_to_staubli',
        planner=_fake_backend(),
    )
    transport = _ros_transport_shell(
        generator,
        endpoints=[_Endpoint('room_315_vla_supervisor')],
        latest_status={
            'rails': {
                'right': {
                    'shuttles': {
                        'room315_right_shuttle_1': {'mode': 'STOPPED'},
                    },
                    'payloads': {},
                },
            },
            'payload_state': {'by_shuttle': {}},
        },
    )

    result = generator.RosScenarioTransport.wait_for_initial_scenario_state(
        transport,
        scenario=scenario,
        timeout_s=0.01,
    )

    assert result['ready'] is False
    assert "missing shuttle 'room315_right_shuttle_2' on right rail" in result['reason']
    assert 'wait for preflight READY' in result['reason']


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


def test_execute_mode_waits_for_recorder_start_and_stop_ack():
    generator = _load_module()
    scenario = generator.generate_scenario(
        goal='right_yaskawa_to_staubli',
        language_template_id='move_from_to',
        planner=_fake_backend(),
    )
    transport = RecorderAckTransport()

    result = generator.execute_scenario(
        scenario,
        transport,
        command_timeout_s=7.0,
    )

    assert result['success'] is True
    assert transport.recorder_waits[0] == (
        'started',
        'move the right shuttle from Yaskawa to Staubli',
        7.0,
    )
    assert transport.recorder_waits[-1] == ('stopped', 7.0)
    assert transport.episode_controls == [
        'start move the right shuttle from Yaskawa to Staubli',
        'stop success',
    ]


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


def test_preflight_mode_reports_ready_line_without_publishing_commands():
    generator = _load_module()
    scenario = generator.generate_scenario(
        goal='right_loaded_r2_to_staubli',
        planner=_fake_backend(),
    )
    transport = InitialStateTransport({
        'ready': True,
        'side': 'right',
        'target_shuttle': 'room315_right_shuttle_2',
        'payload_condition': 'loaded',
    })

    scenario['preflight'] = generator.preflight_scenario(scenario, transport)

    assert scenario['preflight']['ready'] is True
    assert generator._preflight_ready_line(scenario) == (
        'READY room315_right_shuttle_2 loaded on right rail'
    )
    assert transport.episode_controls == []
    assert transport.command_messages == []


def test_preflight_mode_reports_missing_initial_state():
    generator = _load_module()
    scenario = generator.generate_scenario(
        goal='right_loaded_r2_to_staubli',
        planner=_fake_backend(),
    )
    transport = InitialStateTransport({
        'ready': False,
        'reason': (
            "initial scenario state is not ready: missing shuttle "
            "'room315_right_shuttle_2' on right rail; available: room315_right_shuttle_1. "
            "Restart the Room 315 launch with the matching right_shuttle_count/start_slots "
            "and wait for preflight READY."
        ),
    })

    scenario['preflight'] = generator.preflight_scenario(scenario, transport)

    assert scenario['preflight']['ready'] is False
    assert generator._preflight_ready_line(scenario) == (
        "NOT READY: initial scenario state is not ready: missing shuttle "
        "'room315_right_shuttle_2' on right rail; available: room315_right_shuttle_1. "
        "Restart the Room 315 launch with the matching right_shuttle_count/start_slots "
        "and wait for preflight READY."
    )


def test_main_execute_returns_nonzero_when_execution_fails(monkeypatch, capsys):
    generator = _load_module()
    transport = TimeoutTransport()
    monkeypatch.setattr(generator, 'create_planner_backend', lambda *args, **kwargs: _fake_backend())
    monkeypatch.setattr(generator, 'RosScenarioTransport', lambda **kwargs: transport)

    rc = generator.main(['--goal', 'right_yaskawa_to_staubli', '--execute'])
    captured = capsys.readouterr()

    assert rc == 1
    assert '"success": false' in captured.out
    assert transport.episode_controls[-1] == 'stop failure'


def test_main_execute_quiet_suppresses_success_json(monkeypatch, capsys):
    generator = _load_module()
    transport = FakeTransport()
    monkeypatch.setattr(generator, 'create_planner_backend', lambda *args, **kwargs: _fake_backend())
    monkeypatch.setattr(generator, 'RosScenarioTransport', lambda **kwargs: transport)

    rc = generator.main(['--goal', 'right_yaskawa_to_staubli', '--execute', '--quiet'])
    captured = capsys.readouterr()

    assert rc == 0
    assert captured.out == ''
    assert captured.err == ''
    assert transport.episode_controls[-1] == 'stop success'


def test_main_execute_quiet_reports_compact_failure(monkeypatch, capsys):
    generator = _load_module()
    transport = TimeoutTransport()
    monkeypatch.setattr(generator, 'create_planner_backend', lambda *args, **kwargs: _fake_backend())
    monkeypatch.setattr(generator, 'RosScenarioTransport', lambda **kwargs: transport)

    rc = generator.main(['--goal', 'right_yaskawa_to_staubli', '--execute', '--quiet'])
    captured = capsys.readouterr()

    assert rc == 1
    assert captured.out == ''
    assert 'FAILED: timeout waiting for supervisor decision at plan step 0' in captured.err


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
