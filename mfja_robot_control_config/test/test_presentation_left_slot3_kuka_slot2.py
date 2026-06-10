#!/usr/bin/env python3

import ast
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'scripts'
    / 'presentation_left_slot3_kuka_slot2.py'
)
VIDEO_PAGE_PATH = REPO_ROOT / 'rail_robot_video_scenarios.html'


def _script_text() -> str:
    return SCRIPT_PATH.read_text(encoding='utf-8')


def test_presentation_script_uses_direct_rail_and_kuka_topics_without_vla():
    text = _script_text()

    assert '/room_315/vla' not in text
    assert "LEFT_RAIL_PREFIX = '/room_315/rails/left'" in text
    assert "'2': 'DZI2L'" in text
    assert "'3': 'DZI3L'" in text
    assert "'/kuka1/joint_trajectory'" in text
    assert "self.declare_parameter('slot3_overshoot_m', 0.15)" in text
    assert "overshoot_m / speed_mps" in text
    assert "self._overshoot_after_slot_if_needed(slot)" in text
    assert 'target_active_at_start = self._active_slot(slot)' in text
    assert "f'left shuttle to leave slot {slot}'" in text
    assert "self._move_to_slot('3')" in text
    assert 'self._command_kuka()' in text
    assert 'self._command_kuka_initial()' in text
    assert "self._move_to_slot('2')" in text
    assert (
        text.index("self._move_to_slot('3')")
        < text.index('self._command_kuka()')
        < text.index('self._command_kuka_initial()')
        < text.index("self._move_to_slot('2')")
    )


def test_presentation_kuka_pose_matches_requested_degrees():
    module = ast.parse(_script_text())
    constants = {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in module.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
    }

    assert constants['KUKA_PRESENTATION_POSITIONS_RAD'] == (
        1.57079632679,
        -0.52359877560,
        1.91986217719,
        0.69813170080,
        -0.03490658504,
        0.0,
    )
    assert constants['KUKA_INITIAL_POSITIONS_RAD'] == (
        0.0,
        -1.57079632679,
        1.91986217719,
        0.0,
        -0.03490658504,
        0.0,
    )


def test_video_scenario_08_uses_presentation_script_not_vla():
    text = VIDEO_PAGE_PATH.read_text(encoding='utf-8')

    assert 'presentation_left_slot3_kuka_slot2.py' in text
    assert 'enable_room315_vla:=false' in text
    assert 'room315_left_start_slot:=2' in text
    assert 'room315_sensor_publish_rate_hz:=30.0' in text
    assert 'room315_sync_sensor_feedback_to_motion_tick:=true' in text
    assert 'room315_gazebo_set_pose_rate_hz:=30.0' in text
    assert '-p slot3_overshoot_m:=0.15' in text
    assert 'KUKA يرجع إلى الوضع الابتدائي' in text
    assert 'vla_teleop_generator.py' not in text
