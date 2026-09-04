#!/usr/bin/env python3

import importlib.util
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
SCRIPT_PATH = SCRIPTS_DIR / 'room_315_kinematic_shuttle_node.py'


def _load_module():
    if str(SCRIPTS_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location('room_315_kinematic_shuttle_node_test', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(('info', message))

    def warn(self, message):
        self.messages.append(('warn', message))

    def error(self, message):
        self.messages.append(('error', message))


class _FakeFuture:
    def done(self):
        return False


class _FakeClient:
    def __init__(self):
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return _FakeFuture()


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _FakeStamp:
    def to_msg(self):
        return self


class _FakeClock:
    def now(self):
        return _FakeStamp()


class _FakeCore:
    def __init__(self, pose):
        self._pose = pose
        self.state = SimpleNamespace(speed=0.0)

    def pose(self):
        return self._pose


def _node_shell(module):
    node = module.Room315KinematicShuttleNode.__new__(module.Room315KinematicShuttleNode)
    node.rail_side = 'right'
    node.payload_model_sdf = module._default_payload_model_sdf_path()
    node.payload_pose_x_offset_m = -0.08
    node.payload_pose_z_offset_m = 0.0
    node.payload_type = 'box'
    node.enable_payload_visuals = True
    node.enable_gazebo_spawn = True
    node.enable_gazebo_delete = True
    node.enable_gazebo_pose_transform = False
    node.spawn_client = _FakeClient()
    node.delete_client = _FakeClient()
    node.shuttles = []
    node.state_publisher = _FakePublisher()
    node.frame_id = 'world'
    node.get_clock = lambda: _FakeClock()
    node.payload_state_topic = '/room_315/rails/right/shuttles/payload_state'
    node.get_logger = _FakeLogger
    return node


def _managed_shuttle(module, name, *, loaded=False, deployed=True):
    pose = module.ShuttlePose(
        x=1.25,
        y=-0.5,
        z=0.42,
        yaw=0.7,
        current_segment='A12E',
        s=0.35,
        mode='STOPPED',
    )
    return module.ManagedShuttle(
        entity_name=name,
        start_slot='2',
        start_pose=None,
        start_snap_distance_m=0.0,
        start_segment='A12E',
        start_s=0.0,
        core=_FakeCore(pose),
        pose_publisher=None,
        payload_loaded=loaded,
        payload_type='box' if loaded else 'none',
        payload_entity_name=f'{name}_payload',
        deployed=deployed,
    )


def test_default_carried_payload_model_exists_and_is_collisionless():
    module = _load_module()

    payload_path = module._default_payload_model_sdf_path()
    sdf = payload_path.read_text(encoding='utf-8')

    assert payload_path.name == 'model.sdf'
    assert payload_path.parent.name == 'room315_payload_small_box'
    assert '<collide_bitmask>0x0000</collide_bitmask>' in sdf


def test_payload_spawn_factory_places_box_on_shuttle_pose():
    module = _load_module()
    node = _node_shell(module)
    shuttle = _managed_shuttle(module, 'room315_right_shuttle_2', loaded=True)

    factory = node._make_payload_spawn_entity_factory(shuttle)

    assert factory.name == 'room315_right_shuttle_2_payload'
    assert factory.sdf_filename == str(node.payload_model_sdf)
    assert factory.relative_to == 'world'
    assert factory.pose.position.x == pytest.approx(1.25 - 0.08 * math.cos(0.7))
    assert factory.pose.position.y == pytest.approx(-0.5 - 0.08 * math.sin(0.7))
    assert factory.pose.position.z == pytest.approx(0.42)


def test_payload_pose_tracks_shuttle_motion_without_changing_identity_regions():
    module = _load_module()
    node = _node_shell(module)
    node.payload_pose_x_offset_m = 0.1
    node.payload_pose_z_offset_m = 0.015
    pose = module.ShuttlePose(
        x=2.0,
        y=3.0,
        z=0.4,
        yaw=1.2,
        current_segment='A34E',
        s=0.75,
        mode='MOVING',
    )

    payload_pose = node._payload_pose_from_shuttle_pose(pose)

    assert payload_pose.x == pytest.approx(pose.x + 0.1 * math.cos(pose.yaw))
    assert payload_pose.y == pytest.approx(pose.y + 0.1 * math.sin(pose.yaw))
    assert payload_pose.yaw == pytest.approx(pose.yaw)
    assert payload_pose.current_segment == pose.current_segment
    assert payload_pose.s == pytest.approx(pose.s)
    assert payload_pose.z == pytest.approx(0.415)


def test_payload_local_x_offset_stays_fixed_when_shuttle_yaws():
    module = _load_module()
    node = _node_shell(module)
    node.payload_pose_x_offset_m = 0.1
    pose = module.ShuttlePose(
        x=2.0,
        y=3.0,
        z=0.4,
        yaw=math.pi / 2.0,
        current_segment='A34E',
        s=0.75,
        mode='MOVING',
    )

    payload_pose = node._payload_pose_from_shuttle_pose(pose)

    assert payload_pose.x == pytest.approx(2.0, abs=1e-9)
    assert payload_pose.y == pytest.approx(3.1)
    assert payload_pose.yaw == pytest.approx(pose.yaw)


def test_payload_state_reports_loaded_and_empty_shuttles_outside_model_input():
    module = _load_module()
    node = _node_shell(module)
    loaded = _managed_shuttle(module, 'room315_right_shuttle_2', loaded=True)
    empty = _managed_shuttle(module, 'room315_right_shuttle_3', loaded=False)
    node.shuttles = [loaded, empty]

    state = node._payload_state_payload()
    by_name = {entry['entity_name']: entry for entry in state['shuttles']}

    assert by_name['room315_right_shuttle_2']['loaded'] is True
    assert by_name['room315_right_shuttle_2']['payload_type'] == 'box'
    assert by_name['room315_right_shuttle_3']['loaded'] is False
    assert by_name['room315_right_shuttle_3']['payload_type'] == 'none'
    assert state['model_input_exposure'] == 'excluded'
    assert all(entry['model_input_exposure'] == 'excluded' for entry in state['shuttles'])


def test_shuttle_state_publishes_every_managed_shuttle_for_supervisor():
    module = _load_module()
    node = _node_shell(module)
    r1 = _managed_shuttle(module, 'room315_right_shuttle_1', loaded=False)
    r2 = _managed_shuttle(module, 'room315_right_shuttle_2', loaded=True)
    node.shuttles = [r1, r2]

    raw_poses = [r1.core.pose(), r2.core.pose()]
    gazebo_poses = [r1.core.pose(), r2.core.pose()]

    node._publish_state(raw_poses, gazebo_poses)

    assert [message.name for message in node.state_publisher.messages] == [
        'room315_right_shuttle_1',
        'room315_right_shuttle_2',
    ]
    assert all(message.current_segment == 'A12E' for message in node.state_publisher.messages)


def test_loaded_payload_requests_spawn_and_empty_payload_requests_delete():
    module = _load_module()
    node = _node_shell(module)
    shuttle = _managed_shuttle(module, 'room315_right_shuttle_2', loaded=False)

    node._set_shuttle_payload_loaded(shuttle, True)

    assert shuttle.payload_loaded is True
    assert shuttle.payload_type == 'box'
    assert shuttle.pending_payload_spawn is not None
    assert len(node.spawn_client.requests) == 1
    assert node.spawn_client.requests[0].entity_factory.name == 'room315_right_shuttle_2_payload'

    shuttle.pending_payload_spawn = None
    shuttle.payload_gazebo_spawned = True
    node._set_shuttle_payload_loaded(shuttle, False)

    assert shuttle.payload_loaded is False
    assert shuttle.payload_type == 'none'
    assert shuttle.pending_payload_delete is not None
    assert len(node.delete_client.requests) == 1
    assert node.delete_client.requests[0].entity.name == 'room315_right_shuttle_2_payload'
