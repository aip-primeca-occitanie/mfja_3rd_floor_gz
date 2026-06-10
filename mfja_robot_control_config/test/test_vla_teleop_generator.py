#!/usr/bin/env python3

import ast
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'vla_teleop_generator.py'


def _script_text() -> str:
    return SCRIPT_PATH.read_text(encoding='utf-8')


def test_teleop_generator_subscribes_to_real_and_compat_sensor_feedback_topics():
    module = ast.parse(_script_text())
    constants = {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in module.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
    }

    assert constants['SENSOR_FEEDBACK_TOPICS'] == ('feedback', 'position_feedback')
    assert "/sensors/{topic_suffix}" in _script_text()


def test_teleop_generator_stops_shuttle_after_station_timeout():
    text = _script_text()

    assert 'try:\n            self.on(side)' in text
    assert 'finally:\n            self.off(side)' in text
    assert 'slot wait diagnostics' in text


def test_teleop_generator_does_not_require_dataset_recorder_subscription():
    text = _script_text()

    assert 'if self.cmd_pub.get_subscription_count() > 0:' in text
    assert 'and self.ctrl_pub.get_subscription_count() > 0' not in text
    assert 'running without dataset episodes' in text
    assert 'VLA command subscriber connected' in text


def test_teleop_generator_resets_shuttle_after_each_scenario():
    text = _script_text()

    assert "self.declare_parameter('reset_after_each_scenario', True)" in text
    assert 'self.current_scenario_sides: set[str] = set()' in text
    assert 'def reset_after_scenario(self) -> None:' in text
    assert "self.reset_after_scenario()" in text
    assert "'command': 'RESET'" in text
    assert "{'ALL': 'EXTERIOR'}" in text


def test_teleop_generator_keeps_reset_outside_dataset_episode():
    text = _script_text()

    assert "self.declare_parameter('recorder_status_topic', '/room_315/vla/dataset_status')" in text
    assert 'self._on_recorder_status' in text
    assert 'stop_sent_at = time.monotonic()' in text
    assert 'self.wait_for_recorder_stopped_before_reset(stop_sent_at)' in text
    assert 'def recorder_has_stopped_since(self, stop_sent_at: float) -> bool:' in text
    assert 'self.recorder_status_time_s < stop_sent_at' in text
    assert "'dataset recorder to stop before reset'" in text
    assert 'skipping post-scenario reset because the dataset recorder did not' in text


def test_teleop_generator_can_select_specific_scenario_by_code_or_name():
    text = _script_text()

    assert "('r02', 'move_right_shuttle_from_yaskawa_to_staubli', self.r02)" in text
    assert "('m10', 'right_obstacle_aware_route', self.m10)" in text
    assert 'def select_scenarios(' in text
    assert 'self._scenario_key(code)' in text
    assert 'self._scenario_key(name)' in text
    assert "self._scenario_key(getattr(scenario, '__name__', ''))" in text
    assert "self.get_logger().error('no VLA teleop scenarios selected')" in text


def test_station_to_station_uses_station_inference_before_source_staging():
    text = _script_text()
    body = text.split('def move_station_to_station', 1)[1].split(
        '\n    def full_exterior_loop',
        1,
    )[0]

    assert 'if not self.wait_for_state(side):' in body
    assert 'if not self.wait_for_sensor_feedback(side):' in body
    assert 'current = self.current_station(side)' in body
    assert 'if current != source:' in body
    assert 'if not self.active_slot(side, source_slots):' not in body
    assert "segment.startswith('A12')" in text
    assert "return 'yaskawa'" in text


def test_stopper_wait_accepts_real_stopper_sensor_feedback():
    module = ast.parse(_script_text())
    constants = {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in module.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
    }
    text = _script_text()

    assert constants['STOPPER_SENSOR_NAMES']['A1'] == 'A1_STOPPER_SENSOR'
    assert 'if self.active_sensor(side, (sensor_name,)):' in text
    assert 'return True\n            if self.segment(side) not in target_segments:' in text


def test_segment_leave_wait_uses_requested_timeout_for_slow_loops():
    text = _script_text()

    assert 'min(timeout_s, 20.0)' not in text
    assert "f'{side} shuttle to leave segments {sorted(normalized)}'" in text
    assert 'return \'\'' in text


def test_a4_transition_stages_interior_approach_before_switching_a4():
    module = ast.parse(_script_text())
    constants = {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in module.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
    }
    text = _script_text()

    assert constants['A4_INTERIOR_APPROACH_SEGMENT'] == {
        'right': 'A34I',
        'left': 'A34I',
    }
    assert constants['A4_INTERIOR_EXIT_SEGMENTS'] == {
        'right': {'A14'},
        'left': {'A14'},
    }
    assert constants['A4_INTERIOR_PASS_PLANS']['left'] == {
        'approach_segment': 'A34I',
        'exit_segments': {'A14'},
        'pass_switches': {'A4': 'INTERIOR'},
        'stage_from_stopper': 'A3',
        'stage_switches': {'A3': 'INTERIOR', 'A4': 'EXTERIOR'},
        'stage_stopper': 'A4',
    }
    assert 'def stage_a4_interior_approach' in text
    assert "self.sw_i(side, plan['stage_switches'])" in text
    assert "self.sw_i(side, plan['pass_switches'])" in text


def test_interior_segments_exit_with_side_specific_guard_switches():
    module = ast.parse(_script_text())
    constants = {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in module.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
    }

    plans = constants['INTERIOR_EXTERIOR_EXIT_PLANS']

    assert {
        'segments': {'A1I', 'A12I', 'A2I'},
        'switches': {'A4': 'INTERIOR'},
        'stopper': 'A1',
    } in plans['left']
    assert {
        'segments': {'A1I', 'A12I', 'A2I'},
        'switches': {'A2': 'INTERIOR'},
        'stopper': 'A3',
    } in plans['right']


def test_switch_transition_episode_labels_are_task_level():
    text = _script_text()

    assert 'stop at A3 then switch A3 interior' not in text
    assert 'stop at A4 then switch A4 interior' not in text
    assert 'route right shuttle through A3 into the interior branch' in text
    assert 'pass right shuttle through A4 from the interior approach' in text
    assert 'route left shuttle through A3 into the interior branch' in text
    assert 'pass left shuttle through A4 from the interior approach' in text


def test_loop_mode_changes_stop_at_gate_and_wait_for_switch_feedback():
    module = ast.parse(_script_text())
    constants = {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in module.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
    }
    text = _script_text()

    assert constants['MODE_CHANGE_STOPPER'] == {
        'right': 'A3',
        'left': 'A1',
    }
    assert 'from mfja_rail_interfaces.msg import SwitchState as RailSwitchState' in text
    assert 'f\'{prefix}/switches/state\'' in text
    assert 'def wait_for_all_switches' in text
    assert "self._stop_before_mode_change_gate(side, approach_state)" in text
    assert "self._stop_before_mode_change_gate(side, 'INTERIOR')" in text
    assert "self._set_all_switches_before_continuing(side, 'INTERIOR')" in text
    assert "self._set_all_switches_before_continuing(side, 'EXTERIOR')" in text
    assert "if not self.force_exterior(side):" in text


def test_visual_recovery_training_scenarios_are_registered():
    module = ast.parse(_script_text())
    constants = {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in module.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
    }
    text = _script_text()

    expected = (
        'left_slot3_kuka_then_slot2',
        'right_obstacle_aware_route',
        'left_obstacle_aware_route',
    )

    assert constants['VISUAL_TRAINING_SCENARIOS'] == expected
    expected_codes = {
        'left_slot3_kuka_then_slot2': 'm08',
        'right_obstacle_aware_route': 'm10',
        'left_obstacle_aware_route': 'm11',
    }
    for scenario_name, scenario_code in expected_codes.items():
        assert f'def {scenario_code}(self):' in text
        assert f"self.begin('{scenario_name}')" in text
        assert f'self.{scenario_code}' in text
    removed_scenarios = {
        'm01': 'close all right stoppers then open them',
        'm03': 'unknown_position_recovery',
        'm07': 'sensor_dropout_route',
        f'm{9:02d}': 'visual_' + 'obstacle_stop',
    }
    for scenario_code, scenario_name in removed_scenarios.items():
        assert f'def {scenario_code}(self):' not in text
        assert f"self.begin('{scenario_name}')" not in text
        assert f"('{scenario_code}'," not in text
    assert "self.begin('visual_stop_before_A3')" not in text
    assert "self.begin('visual_stop_before_A4')" not in text
    assert "self.begin('visual_center_at_station')" not in text


def test_visual_research_scenarios_keep_only_nontrivial_visual_tasks():
    text = _script_text()
    manual_obstacle_body = text.split('def stop_before_manual_obstacle', 1)[1].split(
        '\n    def stop_at_stopper',
        1,
    )[0]

    assert 'def reacquire_unknown_position' not in text
    assert 'def stage_unknown_position_start' not in text
    assert 'def route_with_sensor_dropout_fallback' not in text
    assert 'unknown-position recovery' not in text
    assert 'sensor-dropout route' not in text
    assert 'def ' + 'visual_' + 'obstacle_stop' not in text
    assert 'from trajectory_msgs.msg import JointTrajectory' in text
    assert 'from trajectory_msgs.msg import JointTrajectoryPoint' in text
    assert "self.kuka_trajectory_pub = self.create_publisher(" in text
    assert "self.begin('left_slot3_kuka_then_slot2')" in text
    assert "self.go_to_slot('left', '3', require_leave=True)" in text
    assert "self.command_kuka_joint_pose(KUKA_SLOT3_INTERLOCK_POSITIONS_RAD)" in text
    assert "self.go_to_slot('left', '2', require_leave=True)" in text
    assert 'def stop_before_manual_obstacle' in text
    assert 'self.set_obstacle_pose(' not in text
    assert 'scenario will not move it' in text
    assert 'def obstacle_decision' in text
    assert 'def _distance_to_obstacle_path_xy' in text
    assert 'def _project_to_exterior_loop_xy' in text
    assert 'def _loop_distance_ahead' in text
    assert 'def _gazebo_to_rail_xy' in text
    assert 'RAIL_TO_GAZEBO_CALIBRATION' in text
    assert 'EXTERIOR_LOOP_RAW_SEGMENT_CSVS' in text
    assert 'csv.DictReader' in text
    assert 'def wait_until_before_manual_obstacle' in text
    assert 'FALLBACK_EXTERIOR_ROUTE_SEGMENTS_XY' in text
    assert 'blocks_path' in text
    assert 'clear of the exterior loop; completing one big loop' in text
    assert 'return self.full_exterior_loop(side, speed=speed)' in text
    assert 'def full_exterior_loop(self, side: str, speed: float = 0.3) -> bool' in text
    assert "self.declare_parameter('manual_obstacle_pose_file'" in text
    assert "self.declare_parameter(f'{side}_manual_obstacle_use_pose_file', True)" in text
    assert "f'{side}_manual_obstacle_path_threshold_m'," in text
    assert "f'{side}_manual_obstacle_stop_before_m'," in text
    assert 'rail_xy=({decision["obstacle_rail_x"]:.3f}' in text
    assert 'stop_before={stop_before_m:.3f}m, route=EXTERIOR_LOOP' in text
    assert "self.sw(side, 'EXTERIOR')" in text
    assert "self.st(side, '0')" in text
    assert 'self.on(side, speed)' in text
    assert 'target_stopper=stopper_name' not in manual_obstacle_body
    assert 'self.wait_until_before_manual_obstacle(' in text
    assert 'shuttle stopped directly before visual obstacle' in text
    assert 'def prepare_shortest_obstacle_route' not in text
    assert 'def _shortest_route_mode_to_obstacle' not in text
    assert 'INTERIOR_SHORTEST_OBSTACLE_STOPPERS' not in text
    assert "self.begin('right_obstacle_aware_route')" in text
    assert "self.begin('left_obstacle_aware_route')" in text
    assert "self.declare_parameter(f'{side}_manual_obstacle_x'" in text
    assert "self.get_parameter(f'{side}_manual_obstacle_x')" in text


def test_left_slot3_kuka_slot2_scenario_uses_requested_pose():
    module = ast.parse(_script_text())
    constants = {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in module.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
    }

    assert constants['KUKA_JOINT_NAMES'] == (
        'joint_a1',
        'joint_a2',
        'joint_a3',
        'joint_a4',
        'joint_a5',
        'joint_a6',
    )
    assert constants['KUKA_SLOT3_INTERLOCK_POSITIONS_RAD'] == (
        1.57079632679,
        -0.52359877560,
        1.91986217719,
        0.69813170080,
        -0.03490658504,
        0.0,
    )
    assert constants['KUKA_SLOT3_INTERLOCK_DURATION_S'] == 4.0


def test_teleop_generator_documents_privileged_state_not_model_input():
    text = _script_text()

    assert 'language, overhead images, and the' in text
    assert 'Binary rail sensors, segment, and arc-length values are used' in text
    assert 'only by the deterministic expert' in text
    assert 'keeps those privileged values out of model_input' in text
