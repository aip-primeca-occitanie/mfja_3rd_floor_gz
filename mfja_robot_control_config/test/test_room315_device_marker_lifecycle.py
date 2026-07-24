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
        'room_315_kinematic_shuttle_node',
        SCRIPT_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _DoneFuture:
    def __init__(self, response):
        self._response = response

    def done(self):
        return True

    def result(self):
        return self._response


class _ReadyClient:
    def service_is_ready(self):
        return True


class _ReadySpawnClient:
    def __init__(self):
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return 'pending_spawn'


class _Logger:
    def warn(self, _message):
        pass


def _marker(module, **overrides):
    values = {
        'entity_name': 'marker_right_DA3R',
        'device_type': 'position_sensor',
        'device_name': 'DA3R',
        'segment': 'A23',
        'pose': None,
        'sdf': '',
        'visual_state': module.MARKER_VISUAL_INACTIVE,
    }
    values.update(overrides)
    return module.DeviceMarker(**values)


def test_marker_spawn_rejection_does_not_delete_unknown_gazebo_entity():
    module = _load_module()
    marker = _marker(module)
    marker.pending_spawn = _DoneFuture(SimpleNamespace(success=False))
    marker.pending_spawn_visual_state = module.MARKER_VISUAL_INACTIVE
    marker.spawn_attempts = 1
    delete_calls = []
    fake_node = SimpleNamespace(
        device_markers=[marker],
        device_marker_retry_interval_s=0.5,
        _marker_spawn_attempts_exhausted=lambda _marker: False,
        _request_device_marker_delete=lambda marker, reason: delete_calls.append(
            (marker.entity_name, reason)
        ),
        get_logger=lambda: _Logger(),
    )

    module.Room315KinematicShuttleNode._process_device_marker_futures(fake_node)

    assert delete_calls == []
    assert marker.pending_spawn is None
    assert marker.pending_spawn_visual_state is None
    assert marker.next_spawn_attempt_time > 0.0


def test_continuous_start_positions_parse_public_segment_and_ratio():
    module = _load_module()
    parser = module.Room315KinematicShuttleNode._resolve_start_position_overrides
    splitter = module.Room315KinematicShuttleNode._split_list_parameter
    right = SimpleNamespace(
        rail_side='right',
        network=SimpleNamespace(segments={
            'A12E': SimpleNamespace(length=2.0),
            'A23': SimpleNamespace(length=0.3),
        }),
        _split_list_parameter=splitter,
    )
    left = SimpleNamespace(
        rail_side='left',
        network=SimpleNamespace(segments={
            'A34E': SimpleNamespace(length=2.4),
        }),
        _split_list_parameter=splitter,
    )

    assert parser(right, 'A12E@0.25,A23@0.5', shuttle_count=2) == [
        ('A12E', 0.5),
        ('A23', 0.15),
    ]
    assert parser(left, 'A12E@0.25', shuttle_count=1) == [
        ('A34E', 0.6),
    ]


def test_marker_spawn_warning_state_does_not_block_future_retries(monkeypatch):
    module = _load_module()
    marker = _marker(
        module,
        spawn_failure_logged=True,
        spawn_attempts=8,
        next_spawn_attempt_time=10.0,
    )
    spawn_client = _ReadySpawnClient()
    fake_node = SimpleNamespace(
        device_markers=[marker],
        spawn_client=spawn_client,
        next_device_marker_spawn_time=0.0,
        device_marker_spawn_interval_s=0.05,
        _make_device_marker_factory=lambda _marker: module.EntityFactory(),
    )

    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.1)
    module.Room315KinematicShuttleNode._request_device_marker_spawns(fake_node)

    assert len(spawn_client.requests) == 1
    assert marker.pending_spawn == 'pending_spawn'
    assert marker.pending_spawn_visual_state == marker.visual_state
    assert marker.spawn_attempts == 9
    assert fake_node.next_device_marker_spawn_time == 10.15


def test_position_sensor_marker_refresh_never_deletes_gazebo_entity(monkeypatch):
    module = _load_module()
    marker = _marker(
        module,
        spawned=True,
        spawned_visual_state=module.MARKER_VISUAL_INACTIVE,
        visual_state=module.MARKER_VISUAL_ACTIVE,
        last_spawn_success_time=10.0,
    )
    delete_calls = []
    fake_node = SimpleNamespace(
        device_markers=[marker],
        delete_client=_ReadyClient(),
        device_marker_dynamic_refresh=True,
        device_marker_refresh_grace_period_s=0.5,
        _request_device_marker_delete=lambda marker, reason: delete_calls.append(
            (marker.entity_name, reason)
        ),
    )

    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.2)
    module.Room315KinematicShuttleNode._request_device_marker_refreshes(fake_node)
    assert delete_calls == []

    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.6)
    module.Room315KinematicShuttleNode._request_device_marker_refreshes(fake_node)
    assert delete_calls == []


def test_marker_refresh_is_disabled_by_default_to_avoid_gazebo_remove_errors():
    module = _load_module()
    marker = _marker(
        module,
        spawned=True,
        spawned_visual_state=module.MARKER_VISUAL_INACTIVE,
        visual_state=module.MARKER_VISUAL_ACTIVE,
    )
    delete_calls = []
    fake_node = SimpleNamespace(
        device_markers=[marker],
        delete_client=_ReadyClient(),
        device_marker_dynamic_refresh=False,
        device_marker_refresh_grace_period_s=0.0,
        _request_device_marker_delete=lambda marker, reason: delete_calls.append(
            (marker.entity_name, reason)
        ),
    )

    module.Room315KinematicShuttleNode._request_device_marker_refreshes(fake_node)

    assert delete_calls == []


def test_position_sensor_startup_markers_spawn_single_base_and_hidden_active_overlay():
    module = _load_module()
    node = object.__new__(module.Room315KinematicShuttleNode)
    node.rail_side = 'left'
    node.device_marker_z_offset_m = 0.0
    node.device_marker_scale = 1.0
    node.stopper_states = {}
    node.rail_devices = module.RailDeviceSet(
        path=Path('rail_devices_left.yaml'),
        slots={},
        position_sensors={
            'DZI2L': (
                module.RailDevice(
                    name='DZI2L',
                    device_type='position_sensors',
                    segment='A12E',
                    s_ratio=0.5,
                    s=1.5,
                    x=1.0,
                    y=2.0,
                    z=0.0,
                    yaw=0.0,
                    radius_m=0.08,
                ),
            )
        },
        stoppers={},
    )
    node._to_gazebo_pose = lambda pose: pose

    markers = module.Room315KinematicShuttleNode._make_device_markers(node)

    assert [marker.entity_name for marker in markers] == [
        'marker_left_position_sensors',
        'marker_left_DZI2L_active',
    ]
    base_marker = markers[0]
    assert base_marker.device_type == 'position_sensor_base'
    assert base_marker.active_overlay is False
    assert 'marker_left_DZI2L' in base_marker.sdf

    active_marker = markers[1]
    assert active_marker.active_overlay is True
    assert active_marker.visual_state == module.MARKER_VISUAL_ACTIVE
    assert '<radius>0.054000</radius>' in active_marker.sdf
    assert active_marker.pose.z == -10.0
    assert active_marker.desired_visible is False
    assert active_marker.visible is False


def test_stopper_startup_markers_spawn_base_and_hidden_active_overlay():
    module = _load_module()
    node = object.__new__(module.Room315KinematicShuttleNode)
    node.rail_side = 'right'
    node.device_marker_z_offset_m = 0.0
    node.device_marker_scale = 1.0
    node.stopper_states = {'A1': module.STOPPER_PASS_STATE}
    node.rail_devices = module.RailDeviceSet(
        path=Path('rail_devices_right.yaml'),
        slots={},
        position_sensors={},
        stoppers={
            'A1': (
                module.RailDevice(
                    name='A1',
                    device_type='stoppers',
                    segment='A14',
                    s_ratio=0.5,
                    s=1.5,
                    x=1.0,
                    y=2.0,
                    z=0.0,
                    yaw=0.0,
                    default_state=module.STOPPER_PASS_STATE,
                ),
            )
        },
    )
    node._to_gazebo_pose = lambda pose: pose

    markers = module.Room315KinematicShuttleNode._make_device_markers(node)

    assert [marker.entity_name for marker in markers] == [
        'marker_right_stopper_A1',
        'marker_right_stopper_A1_active',
    ]
    base_marker = markers[0]
    assert base_marker.device_type == 'stopper_base'
    assert base_marker.active_overlay is False
    assert '1.000 0.720 0.080 0.900' in base_marker.sdf

    active_marker = markers[1]
    assert active_marker.device_type == 'stopper'
    assert active_marker.active_overlay is True
    assert active_marker.visual_state == module.MARKER_VISUAL_ACTIVE
    assert '<radius>0.056250</radius>' in active_marker.sdf
    assert active_marker.pose.z == -10.0
    assert active_marker.visible_pose.z == 0.015
    assert active_marker.desired_visible is False
    assert active_marker.visible is False


def test_stopper_active_overlay_visibility_follows_stopper_state():
    module = _load_module()
    marker = _marker(
        module,
        entity_name='marker_right_stopper_A1_active',
        device_type='stopper',
        device_name='A1',
        active_overlay=True,
        desired_visible=False,
        visible=False,
        visual_state=module.MARKER_VISUAL_ACTIVE,
    )
    pose_updates = []
    refreshes = []
    fake_node = SimpleNamespace(
        enable_device_markers=True,
        device_markers=[marker],
        stopper_states={'A1': module.STOPPER_STOP_STATE},
    )
    fake_node._set_device_marker_visibility = (
        lambda marker, visible:
        module.Room315KinematicShuttleNode._set_device_marker_visibility(
            fake_node,
            marker,
            visible,
        )
    )
    fake_node._request_device_marker_pose_updates = (
        lambda: pose_updates.append('requested')
    )
    fake_node._request_device_marker_refreshes = (
        lambda: refreshes.append('requested')
    )

    module.Room315KinematicShuttleNode._update_stopper_marker_states(fake_node)

    assert marker.desired_visible is True
    assert pose_updates == ['requested']
    assert refreshes == []

    fake_node.stopper_states['A1'] = module.STOPPER_PASS_STATE
    module.Room315KinematicShuttleNode._update_stopper_marker_states(fake_node)

    assert marker.desired_visible is False
    assert pose_updates == ['requested', 'requested']
    assert refreshes == []


def test_sensor_feedback_timer_is_disabled_when_syncing_to_motion_tick():
    module = _load_module()
    calls = []
    fake_node = SimpleNamespace(
        sensor_timer='existing_timer',
        sensor_publish_rate_hz=5.0,
        sync_sensor_feedback_to_motion_tick=True,
        destroy_timer=lambda timer: calls.append(('destroy', timer)),
        create_timer=lambda period, callback: calls.append(('create', period, callback)),
        _publish_all_sensor_feedback=lambda: None,
    )

    module.Room315KinematicShuttleNode._reset_sensor_timer(fake_node)

    assert fake_node.sensor_timer is None
    assert calls == [('destroy', 'existing_timer')]


def test_sensor_feedback_timer_is_created_only_when_motion_sync_is_disabled():
    module = _load_module()
    calls = []

    def create_timer(period, callback):
        calls.append(('create', period, callback))
        return 'new_timer'

    fake_node = SimpleNamespace(
        sensor_timer=None,
        sensor_publish_rate_hz=20.0,
        sync_sensor_feedback_to_motion_tick=False,
        destroy_timer=lambda timer: calls.append(('destroy', timer)),
        create_timer=create_timer,
        _publish_all_sensor_feedback=lambda: None,
    )

    module.Room315KinematicShuttleNode._reset_sensor_timer(fake_node)

    assert fake_node.sensor_timer == 'new_timer'
    assert len(calls) == 1
    assert calls[0][0] == 'create'
    assert calls[0][1] == 0.05


def _shuttle(
    module,
    *,
    segment='A12E',
    s=1.5,
    armed=False,
    mode=None,
    previous_segment=None,
    previous_s=None,
):
    if mode is None:
        mode = module.WAITING
    return SimpleNamespace(
        entity_name='room315_left_shuttle_1',
        deployed=True,
        start_segment='A12E',
        start_s=1.5,
        sensor_markers_armed=armed,
        previous_sensor_segment=previous_segment,
        previous_sensor_s=previous_s,
        core=SimpleNamespace(
            state=module.ShuttleState(
                current_segment=segment,
                s=s,
                speed=0.16,
                mode=mode,
            )
        ),
    )


def test_parked_start_shuttle_does_not_light_position_sensor_marker():
    module = _load_module()
    shuttle = _shuttle(module)
    fake_node = SimpleNamespace(shuttles=[shuttle])
    fake_node._shuttle_on_sensor = (
        lambda segment, sensor_s, radius_m:
        module.Room315KinematicShuttleNode._shuttle_on_sensor(
            fake_node,
            segment,
            sensor_s,
            radius_m,
        )
    )
    fake_node._sensor_markers_should_arm = (
        module.Room315KinematicShuttleNode._sensor_markers_should_arm
    )

    raw_occupancy = module.Room315KinematicShuttleNode._shuttle_on_sensor(
        fake_node,
        'A12E',
        1.5,
        0.08,
    )
    active_for_marker = module.Room315KinematicShuttleNode._shuttle_on_sensor_for_marker(
        fake_node,
        'A12E',
        1.5,
        0.08,
    )

    assert raw_occupancy is shuttle
    assert active_for_marker is None
    assert not shuttle.sensor_markers_armed


def test_parked_start_shuttle_does_not_publish_active_sensor_feedback():
    module = _load_module()
    shuttle = _shuttle(module)
    point = module.PositionSensorPoint(
        segment='A12E',
        sensor_s=1.5,
        radius_m=0.08,
    )
    fake_node = SimpleNamespace(
        shuttles=[shuttle],
        position_sensor_configs={
            'DZI2L': module.PositionSensorConfig(
                name='DZI2L',
                points=(point,),
            )
        },
        network=SimpleNamespace(
            segments={'A12E': SimpleNamespace(length=3.0)}
        ),
        _public_sensor_name=lambda name: name,
        _public_segment_name=lambda name: name,
    )
    fake_node._shuttle_on_sensor = (
        lambda segment, sensor_s, radius_m:
        module.Room315KinematicShuttleNode._shuttle_on_sensor(
            fake_node,
            segment,
            sensor_s,
            radius_m,
        )
    )
    fake_node._shuttle_on_sensor_for_marker = (
        lambda segment, sensor_s, radius_m:
        module.Room315KinematicShuttleNode._shuttle_on_sensor_for_marker(
            fake_node,
            segment,
            sensor_s,
            radius_m,
        )
    )
    fake_node._sensor_markers_should_arm = (
        module.Room315KinematicShuttleNode._sensor_markers_should_arm
    )

    readings = module.Room315KinematicShuttleNode._position_sensor_readings(
        fake_node
    )

    assert readings[0].name == 'DZI2L'
    assert readings[0].active == 0
    assert readings[0].shuttle_name == ''
    assert not shuttle.sensor_markers_armed


def test_position_sensor_marker_lights_after_shuttle_has_moved():
    module = _load_module()
    shuttle = _shuttle(module, s=1.55, mode=module.MOVING)
    fake_node = SimpleNamespace(shuttles=[shuttle])
    fake_node._shuttle_on_sensor = (
        lambda segment, sensor_s, radius_m:
        module.Room315KinematicShuttleNode._shuttle_on_sensor(
            fake_node,
            segment,
            sensor_s,
            radius_m,
        )
    )
    fake_node._sensor_markers_should_arm = (
        module.Room315KinematicShuttleNode._sensor_markers_should_arm
    )

    active_for_marker = module.Room315KinematicShuttleNode._shuttle_on_sensor_for_marker(
        fake_node,
        'A12E',
        1.55,
        0.08,
    )

    assert active_for_marker is shuttle
    assert shuttle.sensor_markers_armed


def test_position_sensor_marker_lights_when_tick_crosses_sensor_near_segment_end():
    module = _load_module()
    shuttle = _shuttle(
        module,
        segment='A23',
        s=0.01,
        mode=module.MOVING,
        previous_segment='A2I',
        previous_s=0.10,
    )
    fake_node = SimpleNamespace(
        shuttles=[shuttle],
        network=SimpleNamespace(
            segments={'A2I': SimpleNamespace(length=0.242834)}
        ),
    )
    fake_node._shuttle_on_sensor = (
        lambda segment, sensor_s, radius_m:
        module.Room315KinematicShuttleNode._shuttle_on_sensor(
            fake_node,
            segment,
            sensor_s,
            radius_m,
        )
    )
    fake_node._shuttle_crossed_sensor_since_last_tick = (
        lambda segment, sensor_s, radius_m:
        module.Room315KinematicShuttleNode._shuttle_crossed_sensor_since_last_tick(
            fake_node,
            segment,
            sensor_s,
            radius_m,
        )
    )
    fake_node._sensor_markers_should_arm = (
        module.Room315KinematicShuttleNode._sensor_markers_should_arm
    )

    active_for_marker = module.Room315KinematicShuttleNode._shuttle_on_sensor_for_marker(
        fake_node,
        'A2I',
        0.2347,
        0.07,
    )

    assert active_for_marker is shuttle
    assert shuttle.sensor_markers_armed


def test_left_da4l_feedback_lights_when_shuttle_crosses_exterior_connector():
    module = _load_module()
    shuttle = _shuttle(
        module,
        segment='A23',
        s=0.01,
        mode=module.MOVING,
        previous_segment='A2E',
        previous_s=0.10,
    )
    fake_node = SimpleNamespace(
        shuttles=[shuttle],
        position_sensor_configs={
            'DA4L': module.PositionSensorConfig(
                name='DA4L',
                points=(
                    module.PositionSensorPoint(
                        segment='A2E',
                        sensor_s=0.1812,
                        radius_m=0.07,
                    ),
                    module.PositionSensorPoint(
                        segment='A2I',
                        sensor_s=0.2347,
                        radius_m=0.07,
                    ),
                ),
            )
        },
        network=SimpleNamespace(
            segments={
                'A2E': SimpleNamespace(length=0.187484),
                'A2I': SimpleNamespace(length=0.242834),
            }
        ),
        _public_sensor_name=lambda name: name,
        _public_segment_name=lambda name: name,
    )
    fake_node._shuttle_on_sensor = (
        lambda segment, sensor_s, radius_m:
        module.Room315KinematicShuttleNode._shuttle_on_sensor(
            fake_node,
            segment,
            sensor_s,
            radius_m,
        )
    )
    fake_node._shuttle_on_sensor_for_marker = (
        lambda segment, sensor_s, radius_m:
        module.Room315KinematicShuttleNode._shuttle_on_sensor_for_marker(
            fake_node,
            segment,
            sensor_s,
            radius_m,
        )
    )
    fake_node._shuttle_crossed_sensor_since_last_tick = (
        lambda segment, sensor_s, radius_m:
        module.Room315KinematicShuttleNode._shuttle_crossed_sensor_since_last_tick(
            fake_node,
            segment,
            sensor_s,
            radius_m,
        )
    )
    fake_node._sensor_markers_should_arm = (
        module.Room315KinematicShuttleNode._sensor_markers_should_arm
    )

    readings = module.Room315KinematicShuttleNode._position_sensor_readings(
        fake_node
    )

    assert readings[0].name == 'DA4L'
    assert readings[0].active == 1
    assert readings[0].shuttle_name
    assert readings[0].segment == 'A2E'


def test_position_sensor_active_overlay_visibility_follows_shuttle_crossing():
    module = _load_module()
    shuttle = _shuttle(module, s=1.55, armed=True)
    marker = _marker(
        module,
        active_overlay=True,
        segment='A12E',
        sensor_s=1.55,
        sensor_radius_m=0.08,
        desired_visible=False,
        visible=False,
        visual_state=module.MARKER_VISUAL_ACTIVE,
    )
    pose_updates = []
    fake_node = SimpleNamespace(
        enable_device_markers=True,
        device_markers=[marker],
        shuttles=[shuttle],
    )
    fake_node._shuttle_on_sensor = (
        lambda segment, sensor_s, radius_m:
        module.Room315KinematicShuttleNode._shuttle_on_sensor(
            fake_node,
            segment,
            sensor_s,
            radius_m,
        )
    )
    fake_node._shuttle_on_sensor_for_marker = (
        lambda segment, sensor_s, radius_m:
        module.Room315KinematicShuttleNode._shuttle_on_sensor_for_marker(
            fake_node,
            segment,
            sensor_s,
            radius_m,
        )
    )
    fake_node._shuttle_crossed_sensor_since_last_tick = (
        lambda segment, sensor_s, radius_m:
        module.Room315KinematicShuttleNode._shuttle_crossed_sensor_since_last_tick(
            fake_node,
            segment,
            sensor_s,
            radius_m,
        )
    )
    fake_node._sensor_markers_should_arm = (
        module.Room315KinematicShuttleNode._sensor_markers_should_arm
    )
    fake_node._set_device_marker_visibility = (
        lambda marker, visible:
        module.Room315KinematicShuttleNode._set_device_marker_visibility(
            fake_node,
            marker,
            visible,
        )
    )
    fake_node._request_device_marker_pose_updates = (
        lambda: pose_updates.append('requested')
    )

    module.Room315KinematicShuttleNode._update_sensor_marker_states(fake_node)

    assert marker.desired_visible is True
    assert pose_updates == ['requested']

    shuttle.core.state.s = 1.75
    module.Room315KinematicShuttleNode._update_sensor_marker_states(fake_node)

    assert marker.desired_visible is False
    assert pose_updates == ['requested', 'requested']


def test_position_sensor_active_overlay_stays_visible_for_visual_hold():
    module = _load_module()
    shuttle = _shuttle(module, s=1.55, armed=True)
    marker = _marker(
        module,
        active_overlay=True,
        segment='A12E',
        sensor_s=1.55,
        sensor_radius_m=0.08,
        desired_visible=False,
        visible=False,
        visual_state=module.MARKER_VISUAL_ACTIVE,
    )
    pose_updates = []
    now_s = [10.0]
    fake_node = SimpleNamespace(
        enable_device_markers=True,
        device_markers=[marker],
        shuttles=[shuttle],
        sensor_marker_visual_hold_s=0.35,
        _state_update_time_s=lambda: now_s[0],
    )
    fake_node._shuttle_on_sensor = (
        lambda segment, sensor_s, radius_m:
        module.Room315KinematicShuttleNode._shuttle_on_sensor(
            fake_node,
            segment,
            sensor_s,
            radius_m,
        )
    )
    fake_node._shuttle_on_sensor_for_marker = (
        lambda segment, sensor_s, radius_m:
        module.Room315KinematicShuttleNode._shuttle_on_sensor_for_marker(
            fake_node,
            segment,
            sensor_s,
            radius_m,
        )
    )
    fake_node._shuttle_crossed_sensor_since_last_tick = (
        lambda segment, sensor_s, radius_m:
        module.Room315KinematicShuttleNode._shuttle_crossed_sensor_since_last_tick(
            fake_node,
            segment,
            sensor_s,
            radius_m,
        )
    )
    fake_node._sensor_markers_should_arm = (
        module.Room315KinematicShuttleNode._sensor_markers_should_arm
    )
    fake_node._set_device_marker_visibility = (
        lambda marker, visible:
        module.Room315KinematicShuttleNode._set_device_marker_visibility(
            fake_node,
            marker,
            visible,
        )
    )
    fake_node._request_device_marker_pose_updates = (
        lambda: pose_updates.append('requested')
    )

    module.Room315KinematicShuttleNode._update_sensor_marker_states(fake_node)
    assert marker.desired_visible is True
    assert abs(marker.visual_hold_until_s - 10.35) < 1e-9
    assert pose_updates == ['requested']

    shuttle.core.state.s = 1.75
    now_s[0] = 10.1
    module.Room315KinematicShuttleNode._update_sensor_marker_states(fake_node)
    assert marker.desired_visible is True
    assert pose_updates == ['requested']

    now_s[0] = 10.4
    module.Room315KinematicShuttleNode._update_sensor_marker_states(fake_node)
    assert marker.desired_visible is False
    assert pose_updates == ['requested', 'requested']
