#!/usr/bin/env python3

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
CONTROL_LAUNCH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'launch'
    / 'room_315_dual_kinematic_shuttles.launch.py'
)
ROOM_ONLY_LAUNCH = REPO_ROOT / 'mfja_3rd_floor_bringup' / 'launch' / 'room_315_only.launch.py'
FULL_FLOOR_LAUNCH = REPO_ROOT / 'mfja_3rd_floor_bringup' / 'launch' / 'full_floor.launch.py'
KINEMATIC_NODE = (
    REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_kinematic_shuttle_node.py'
)


def test_multi_shuttle_launch_arguments_are_exposed_and_forwarded():
    control = CONTROL_LAUNCH.read_text(encoding='utf-8')
    room_only = ROOM_ONLY_LAUNCH.read_text(encoding='utf-8')
    full_floor = FULL_FLOOR_LAUNCH.read_text(encoding='utf-8')

    for text in (control, room_only, full_floor):
        assert 'right_start_slots' in text
        assert 'left_start_slots' in text
        assert 'right_shuttle_count' in text
        assert 'left_shuttle_count' in text
        assert 'falling_stop_offset_m' in text
        assert 'shuttle_collision_distance_m' in text
        assert 'enable_payload_visuals' in text or 'room315_enable_payload_visuals' in text
        assert 'loaded_shuttles' in text or 'room315_right_loaded_shuttles' in text
        assert 'payload_pose_x_offset_m' in text or 'room315_payload_pose_x_offset_m' in text

    assert 'room315_right_start_slots' in room_only
    assert 'room315_left_start_slots' in room_only
    assert 'room315_falling_stop_offset_m' in room_only
    assert 'room315_shuttle_collision_distance_m' in room_only
    assert 'room315_enable_payload_visuals' in room_only
    assert 'room315_payload_pose_x_offset_m' in room_only
    assert 'room315_right_loaded_shuttles' in room_only
    assert 'room315_left_loaded_shuttles' in room_only
    assert 'room315_right_start_slots' in full_floor
    assert 'room315_left_start_slots' in full_floor
    assert 'room315_falling_stop_offset_m' in full_floor
    assert 'room315_shuttle_collision_distance_m' in full_floor
    assert 'room315_enable_payload_visuals' in full_floor
    assert 'room315_payload_pose_x_offset_m' in full_floor
    assert 'room315_right_loaded_shuttles' in full_floor
    assert 'room315_left_loaded_shuttles' in full_floor


def test_payload_x_offset_defaults_center_the_payload():
    control = CONTROL_LAUNCH.read_text(encoding='utf-8')
    room_only = ROOM_ONLY_LAUNCH.read_text(encoding='utf-8')
    full_floor = FULL_FLOOR_LAUNCH.read_text(encoding='utf-8')
    node = KINEMATIC_NODE.read_text(encoding='utf-8')

    assert "'payload_pose_x_offset_m', -0.08" in node
    assert "'payload_pose_x_offset_m',\n            default_value='-0.08'" in control
    assert "'room315_payload_pose_x_offset_m',\n            default_value='-0.08'" in room_only
    assert "'room315_payload_pose_x_offset_m',\n            default_value='-0.08'" in full_floor


def test_room315_only_starts_unpaused_by_default_for_visible_shuttles():
    text = ROOM_ONLY_LAUNCH.read_text(encoding='utf-8')
    start = text.index("'start_paused'")
    block = text[start:text.index('),', start)]

    assert "default_value='false'" in block
    assert 'shuttle timers publish visible poses' in block


def test_kinematic_node_rejects_more_than_four_shuttles_per_side():
    text = KINEMATIC_NODE.read_text(encoding='utf-8')

    assert 'shuttle_count > 4' in text
    assert 'supports at most 4 shuttles per rail side' in text
