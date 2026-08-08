#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
FLOOR_COMMON_LAUNCH = (
    REPO_ROOT
    / 'mfja_3rd_floor_bringup'
    / 'launch'
    / 'room_315_floor_common.py'
)


def _load_floor_common():
    module_name = 'room_315_floor_common_singleton_test'
    spec = importlib.util.spec_from_file_location(
        module_name, FLOOR_COMMON_LAUNCH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def test_runtime_lock_rejects_a_second_room315_launch_and_is_reusable(tmp_path):
    floor_common = _load_floor_common()
    lock_path = tmp_path / 'room315-runtime.lock'

    first_descriptor = floor_common.acquire_room315_runtime_instance_lock(
        lock_path
    )
    try:
        with pytest.raises(
            floor_common.Room315RuntimeAlreadyRunning,
            match='another Room 315 simulation launch is already running',
        ):
            floor_common.acquire_room315_runtime_instance_lock(lock_path)
    finally:
        floor_common.release_room315_runtime_instance_lock(first_descriptor)

    replacement_descriptor = (
        floor_common.acquire_room315_runtime_instance_lock(lock_path)
    )
    floor_common.release_room315_runtime_instance_lock(replacement_descriptor)


def test_runtime_lock_wraps_the_complete_floor_launch_lifetime():
    source = FLOOR_COMMON_LAUNCH.read_text(encoding='utf-8')

    acquire_index = source.index(
        'OpaqueFunction(function=_acquire_room315_runtime_lock)'
    )
    base_launch_index = source.index(
        'IncludeLaunchDescription(\n            '
        'PythonLaunchDescriptionSource(base_launch)'
    )

    assert acquire_index < base_launch_index
    assert 'OnShutdown(' in source
    assert 'OpaqueFunction(function=_release_room315_runtime_lock)' in source
