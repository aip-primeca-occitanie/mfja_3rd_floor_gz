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
        'unknown_position_recovery',
        'visual_stop_before_A3',
        'visual_stop_before_A4',
        'visual_center_at_station',
        'sensor_dropout_route',
        'visual_marker_target',
        'visual_obstacle_stop',
    )

    assert constants['VISUAL_TRAINING_SCENARIOS'] == expected
    for index, scenario_name in enumerate(expected, 3):
        assert f'def m{index:02d}(self):' in text
        assert f"self.begin('{scenario_name}')" in text
        assert f'self.m{index:02d}' in text


def test_visual_recovery_scenarios_are_sensor_driven_and_safe():
    text = _script_text()

    assert 'def reacquire_unknown_position' in text
    assert 'tuple(SLOT_SENSORS[side])' in text
    assert 'leave_first=bool(start_slot)' in text
    assert 'def route_with_sensor_dropout_fallback' in text
    assert 'checkpoint stopper {fallback_stopper}' in text
    assert 'self.wait_for_stopper_stop(side, fallback_stopper, 90.0)' in text
    assert 'self.wait_for_slot(side, target_slots, 30.0)' in text
    assert 'def visual_obstacle_stop' in text
    assert "self.st_i(side, {'ALL': '0', stopper_name: '1'})" in text
    assert "self.go_to_slot('left', '4', require_leave=True)" in text


def test_teleop_generator_documents_privileged_state_not_model_input():
    text = _script_text()

    assert 'overhead images plus binary rail' in text
    assert 'used here only by' in text
    assert 'keeps those privileged values out of model_input' in text
