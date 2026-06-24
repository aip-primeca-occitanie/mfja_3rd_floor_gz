#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'


def _load_module():
    if str(SCRIPTS_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPTS_DIR))
    path = SCRIPTS_DIR / 'room_315_kinematic_shuttle.py'
    spec = importlib.util.spec_from_file_location('room_315_kinematic_shuttle_core_test', path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _single_segment_network(shuttle):
    segment = shuttle.SegmentGeometry(
        name='A',
        points=[
            shuttle.Point3D(0.0, 0.0, 0.0),
            shuttle.Point3D(1.0, 0.0, 0.0),
        ],
    )
    return shuttle.RailNetwork(
        network_path=Path('test.yaml'),
        config={
            'routing_table': {
                'A': {
                    'type': 'fixed',
                    'next_segment': shuttle.FALLING,
                },
            },
            'switch_state_space': {'values': ['E', 'I']},
        },
        segments={'A': segment},
    )


def test_falling_stop_offset_latches_pose_before_invalid_endpoint():
    shuttle = _load_module()
    network = _single_segment_network(shuttle)
    core = shuttle.KinematicShuttleCore(
        network=network,
        initial_state=shuttle.ShuttleState(current_segment='A', s=0.0, speed=1.0),
        falling_stop_offset_m=0.1,
    )

    pose = core.step(2.0)

    assert pose.mode == shuttle.FALLING
    assert pose.s == pytest.approx(0.9)


def test_default_falling_pose_stays_at_invalid_endpoint():
    shuttle = _load_module()
    network = _single_segment_network(shuttle)
    core = shuttle.KinematicShuttleCore(
        network=network,
        initial_state=shuttle.ShuttleState(current_segment='A', s=0.0, speed=1.0),
    )

    pose = core.step(2.0)

    assert pose.mode == shuttle.FALLING
    assert pose.s == pytest.approx(1.0)
