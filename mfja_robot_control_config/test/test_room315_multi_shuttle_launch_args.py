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

    assert 'room315_right_start_slots' in room_only
    assert 'room315_left_start_slots' in room_only
    assert 'room315_right_start_slots' in full_floor
    assert 'room315_left_start_slots' in full_floor


def test_kinematic_node_rejects_more_than_four_shuttles_per_side():
    text = KINEMATIC_NODE.read_text(encoding='utf-8')

    assert 'shuttle_count > 4' in text
    assert 'supports at most 4 shuttles per rail side' in text
