#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
LAUNCH_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'launch'
BRINGUP_LAUNCH_DIR = REPO_ROOT / 'mfja_3rd_floor_bringup' / 'launch'


def _load_module(name: str, path: Path):
    if str(SCRIPTS_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_shuttle_visual_debug_colors_can_keep_falling_shuttle_black():
    shuttle_node = _load_module(
        'room_315_kinematic_shuttle_node',
        SCRIPTS_DIR / 'room_315_kinematic_shuttle_node.py',
    )
    node = object.__new__(shuttle_node.Room315KinematicShuttleNode)
    shuttle = SimpleNamespace(
        core=SimpleNamespace(
            state=SimpleNamespace(
                mode=shuttle_node.FALLING,
            )
        )
    )

    node.visual_debug_colors = True
    assert node._desired_shuttle_visual_state(shuttle) == shuttle_node.SHUTTLE_VISUAL_FALLING

    node.visual_debug_colors = False
    assert node._desired_shuttle_visual_state(shuttle) == shuttle_node.SHUTTLE_VISUAL_NORMAL
    assert node._shuttle_visual_rgba(shuttle_node.SHUTTLE_VISUAL_NORMAL) == (0.01, 0.01, 0.01, 1.0)


def test_switch_visual_debug_colors_can_use_neutral_rail_color():
    controller = _load_module(
        'conveyor_loop_mode_controller',
        SCRIPTS_DIR / 'conveyor_loop_mode_controller.py',
    )
    node = object.__new__(controller.ConveyorLoopModeController)

    node.visual_debug_colors = True
    assert node._switch_colors_for_mode('interior') == controller.SWITCH_MODE_COLORS['interior']
    assert node._switch_colors_for_mode('exterior') == controller.SWITCH_MODE_COLORS['exterior']

    node.visual_debug_colors = False
    assert node._switch_colors_for_mode('interior') == controller.SWITCH_NEUTRAL_COLORS
    assert node._switch_colors_for_mode('exterior') == controller.SWITCH_NEUTRAL_COLORS


def test_visual_debug_color_launch_argument_is_threaded_to_room315_nodes():
    files = [
        LAUNCH_DIR / 'multi_robot_sim.launch.py',
        LAUNCH_DIR / 'room_315_dual_kinematic_shuttles.launch.py',
        BRINGUP_LAUNCH_DIR / 'room_315_only.launch.py',
        BRINGUP_LAUNCH_DIR / 'full_floor.launch.py',
    ]
    for path in files:
        text = path.read_text(encoding='utf-8')
        assert 'visual_debug_colors' in text or 'room315_visual_debug_colors' in text
