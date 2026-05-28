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
