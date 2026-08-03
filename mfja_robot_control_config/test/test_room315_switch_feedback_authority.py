#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'scripts'
    / 'room_315_kinematic_shuttle_node.py'
)


def _load_module():
    scripts_dir = str(SCRIPT_PATH.parent)
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    spec = importlib.util.spec_from_file_location(
        'room_315_kinematic_shuttle_node_switch_feedback_test',
        SCRIPT_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_stale_visual_feedback_cannot_reverse_latest_typed_switch_command():
    module = _load_module()
    node = module.Room315KinematicShuttleNode.__new__(
        module.Room315KinematicShuttleNode
    )
    node.switch_states = {'A1': module.SWITCH_EXTERIOR_STATE}
    node.pending_switch_state_updates = {}
    node.visual_switch_feedback_targets = {
        'A1': module.SWITCH_EXTERIOR_STATE,
    }
    logger = _Logger()
    node.get_logger = lambda: logger
    scheduled = []
    published_states = []
    parsed = {'A1': module.SWITCH_INTERIOR_STATE}
    node._parse_visual_switch_state_summary = lambda _raw: dict(parsed)
    node._schedule_switch_state_updates = (
        lambda updates, *, source: scheduled.append((dict(updates), source)) or {}
    )
    node._publish_switch_state = lambda: published_states.append(True)

    node._on_visual_switch_state(SimpleNamespace(data='stale interior feedback'))

    assert node.switch_states == {'A1': module.SWITCH_EXTERIOR_STATE}
    assert scheduled == []
    assert published_states == []
    assert node.visual_switch_feedback_targets == {
        'A1': module.SWITCH_EXTERIOR_STATE,
    }
    assert any('Ignored stale visual switch feedback' in message
               for message in logger.messages)

    parsed['A1'] = module.SWITCH_EXTERIOR_STATE
    node._on_visual_switch_state(SimpleNamespace(data='matching acknowledgement'))

    assert node.visual_switch_feedback_targets == {}
    assert scheduled == []

    parsed['A1'] = module.SWITCH_INTERIOR_STATE
    node._on_visual_switch_state(SimpleNamespace(data='new external request'))

    assert scheduled == [
        ({'A1': module.SWITCH_INTERIOR_STATE}, 'visual state sync'),
    ]
    assert published_states == [True]


def test_published_visual_switch_command_registers_feedback_target():
    module = _load_module()
    node = module.Room315KinematicShuttleNode.__new__(
        module.Room315KinematicShuttleNode
    )
    node.publish_visual_switch_commands = True
    node.sync_from_visual_switch_states = True
    node.visual_switch_feedback_targets = {}
    node.visual_switch_publisher = _Publisher()
    logger = _Logger()
    node.get_logger = lambda: logger
    node._public_switch_state_map = lambda states: dict(states)
    node._visual_selector_for_selector = lambda name: f'{name}R'

    node._publish_visual_switch_actual_updates(
        {'A1': module.SWITCH_EXTERIOR_STATE},
        source='motion delay',
    )

    assert node.visual_switch_feedback_targets == {
        'A1': module.SWITCH_EXTERIOR_STATE,
    }
    assert len(node.visual_switch_publisher.messages) == 1
    assert node.visual_switch_publisher.messages[0].data == 'A1R=EXTERIOR'
