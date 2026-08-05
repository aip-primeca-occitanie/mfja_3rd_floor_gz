#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
import sys

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_task_execution_node import TaskExecutionInstanceAlreadyRunning
from room_315_task_execution_node import acquire_task_execution_instance_lock
from room_315_task_execution_node import release_task_execution_instance_lock


def test_task_execution_instance_lock_rejects_duplicate_and_releases(tmp_path):
    lock_path = tmp_path / 'room315-task-execution.lock'
    first = acquire_task_execution_instance_lock(lock_path)
    try:
        with pytest.raises(
            TaskExecutionInstanceAlreadyRunning,
            match='duplicate command subscriber',
        ):
            acquire_task_execution_instance_lock(lock_path)
    finally:
        release_task_execution_instance_lock(first)

    replacement = acquire_task_execution_instance_lock(lock_path)
    release_task_execution_instance_lock(replacement)
