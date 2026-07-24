#!/usr/bin/env python3

import ast
import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
MULTI_SIM_LAUNCH = REPO_ROOT / 'mfja_robot_control_config' / 'launch' / 'multi_robot_sim.launch.py'
CONTROL_LAUNCH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'launch'
    / 'room_315_dual_kinematic_shuttles.launch.py'
)
ROOM_ONLY_LAUNCH = REPO_ROOT / 'mfja_3rd_floor_bringup' / 'launch' / 'room_315_only.launch.py'
FULL_FLOOR_LAUNCH = REPO_ROOT / 'mfja_3rd_floor_bringup' / 'launch' / 'full_floor.launch.py'
FLOOR_COMMON_LAUNCH = (
    REPO_ROOT / 'mfja_3rd_floor_bringup' / 'launch' / 'room_315_floor_common.py'
)
KINEMATIC_NODE = (
    REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_kinematic_shuttle_node.py'
)
ROOM315_WORLD = REPO_ROOT / 'mfja_3rd_floor_description' / 'worlds' / 'room_315_only.world'


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _launch_argument_block(text: str, argument_name: str) -> str:
    start = text.index(f"DeclareLaunchArgument(\n            '{argument_name}'")
    return text[start:text.index('),', start)]


def _bringup_launch_text(path: Path) -> str:
    return (
        FLOOR_COMMON_LAUNCH.read_text(encoding='utf-8')
        + '\n'
        + path.read_text(encoding='utf-8')
    )


def _floor_profiles() -> dict:
    tree = ast.parse(FLOOR_COMMON_LAUNCH.read_text(encoding='utf-8'))
    assignment = next(
        node
        for node in tree.body
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == 'FLOOR_PROFILES'
            for target in node.targets
        )
    )
    return ast.literal_eval(assignment.value)


def test_multi_shuttle_launch_arguments_are_exposed_and_forwarded():
    control = CONTROL_LAUNCH.read_text(encoding='utf-8')
    room_only = _bringup_launch_text(ROOM_ONLY_LAUNCH)
    full_floor = _bringup_launch_text(FULL_FLOOR_LAUNCH)

    for text in (control, room_only, full_floor):
        assert 'right_start_slots' in text
        assert 'left_start_slots' in text
        assert 'right_start_positions' in text
        assert 'left_start_positions' in text
        assert 'right_shuttle_count' in text
        assert 'left_shuttle_count' in text
        assert 'falling_stop_offset_m' in text
        assert 'shuttle_collision_distance_m' in text
        assert 'enable_payload_visuals' in text or 'room315_enable_payload_visuals' in text
        assert 'loaded_shuttles' in text or 'room315_right_loaded_shuttles' in text
        assert 'payload_pose_x_offset_m' in text or 'room315_payload_pose_x_offset_m' in text

    assert 'room315_right_start_slots' in room_only
    assert 'room315_left_start_slots' in room_only
    assert 'room315_right_start_positions' in room_only
    assert 'room315_left_start_positions' in room_only
    assert 'room315_falling_stop_offset_m' in room_only
    assert 'room315_shuttle_collision_distance_m' in room_only
    assert 'room315_enable_payload_visuals' in room_only
    assert 'room315_payload_pose_x_offset_m' in room_only
    assert 'room315_right_loaded_shuttles' in room_only
    assert 'room315_left_loaded_shuttles' in room_only
    assert 'room315_right_start_slots' in full_floor
    assert 'room315_left_start_slots' in full_floor
    assert 'room315_right_start_positions' in full_floor
    assert 'room315_left_start_positions' in full_floor
    assert 'room315_falling_stop_offset_m' in full_floor
    assert 'room315_shuttle_collision_distance_m' in full_floor
    assert 'room315_enable_payload_visuals' in full_floor
    assert 'room315_payload_pose_x_offset_m' in full_floor
    assert 'room315_right_loaded_shuttles' in full_floor
    assert 'room315_left_loaded_shuttles' in full_floor


def test_payload_x_offset_defaults_center_the_payload():
    control = CONTROL_LAUNCH.read_text(encoding='utf-8')
    room_only = _bringup_launch_text(ROOM_ONLY_LAUNCH)
    full_floor = _bringup_launch_text(FULL_FLOOR_LAUNCH)
    node = KINEMATIC_NODE.read_text(encoding='utf-8')

    assert "'payload_pose_x_offset_m', -0.08" in node
    assert "'payload_pose_x_offset_m',\n            default_value='-0.08'" in control
    assert "'room315_payload_pose_x_offset_m',\n            default_value='-0.08'" in room_only
    assert "'room315_payload_pose_x_offset_m',\n            default_value='-0.08'" in full_floor


def test_room315_vla_obstacle_launch_argument_defaults_disabled_and_is_forwarded():
    multi_sim = MULTI_SIM_LAUNCH.read_text(encoding='utf-8')
    room_only = _bringup_launch_text(ROOM_ONLY_LAUNCH)
    full_floor = _bringup_launch_text(FULL_FLOOR_LAUNCH)

    for text in (multi_sim, room_only, full_floor):
        block = _launch_argument_block(text, 'enable_room315_vla_obstacles')
        assert "default_value='false'" in block
        assert "choices=['true', 'false']" in block
        assert 'without obstacles unless explicitly requested' in block

    assert "'enable_room315_vla_obstacles': LaunchConfiguration" in room_only
    assert "'enable_room315_vla_obstacles': LaunchConfiguration" in full_floor
    assert 'LaunchConfiguration(\'enable_room315_vla_obstacles\').perform(context)' in multi_sim


def test_room315_vla_obstacle_world_materializer_removes_only_obstacle_markers():
    launch_module = _load_module('multi_robot_sim_launch', MULTI_SIM_LAUNCH)
    materialized_world = Path(
        launch_module._materialize_world_without_room315_vla_obstacles(str(ROOM315_WORLD))
    )

    assert materialized_world != ROOM315_WORLD
    root = ET.parse(materialized_world).getroot()
    includes = {
        include.findtext('name', default='').strip(): include.findtext('uri', default='').strip()
        for include in root.findall('.//include')
    }

    assert 'room315_vla_right_obstacle_marker' not in includes
    assert 'room315_vla_left_obstacle_marker' not in includes
    assert includes['room315_vla_overhead_devices_1'] == 'model://room315_vla_overhead_devices'


def test_room315_only_starts_unpaused_by_default_for_visible_shuttles():
    profile = _floor_profiles()['room_315_only']

    assert profile['start_paused'] == 'false'
    assert 'shuttle timers publish visible poses' in profile['start_paused_description']


def test_kinematic_node_rejects_more_than_four_shuttles_per_side():
    text = KINEMATIC_NODE.read_text(encoding='utf-8')

    assert 'shuttle_count > 4' in text
    assert 'supports at most 4 shuttles per rail side' in text
