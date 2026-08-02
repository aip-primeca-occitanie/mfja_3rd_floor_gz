#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
CONFIG_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'config'
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


class _Logger:
    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(message)


class _PendingFuture:
    def done(self):
        return False


class _ReadySetPoseClient:
    def __init__(self):
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return _PendingFuture()


class _Clock:
    def now(self):
        return object()


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


def test_shuttle_on_command_speed_updates_existing_shuttle():
    shuttle_node = _load_module(
        'room_315_kinematic_shuttle_node',
        SCRIPTS_DIR / 'room_315_kinematic_shuttle_node.py',
    )
    node = object.__new__(shuttle_node.Room315KinematicShuttleNode)
    shuttle = SimpleNamespace(
        core=SimpleNamespace(
            state=SimpleNamespace(
                mode=shuttle_node.WAITING,
                speed=0.25,
            )
        ),
        deployed=True,
        blocked_by='other_shuttle',
        collision_distance_m=0.1,
        stopped_by='DISABLED',
        stopper_distance_m=0.0,
    )

    node._apply_shuttle_action(shuttle, 'ENABLE', 0.6)

    assert shuttle.enabled is True
    assert shuttle.core.state.speed == 0.6
    assert shuttle.core.state.mode == shuttle_node.MOVING
    assert shuttle.blocked_by is None
    assert shuttle.collision_distance_m is None
    assert shuttle.stopped_by is None


def test_shuttle_off_retains_travel_speed_but_reports_disabled_mode():
    shuttle_node = _load_module(
        'room_315_kinematic_shuttle_node',
        SCRIPTS_DIR / 'room_315_kinematic_shuttle_node.py',
    )
    node = object.__new__(shuttle_node.Room315KinematicShuttleNode)
    shuttle = SimpleNamespace(
        core=SimpleNamespace(
            state=SimpleNamespace(
                mode=shuttle_node.MOVING,
                speed=0.2,
            )
        ),
        enabled=True,
        deployed=True,
        blocked_by=None,
        collision_distance_m=None,
        motion_target_slot='2',
        motion_target_segment='A34E',
        motion_target_s=0.5,
        stopped_by=None,
        stopper_distance_m=None,
    )

    node._apply_shuttle_action(shuttle, 'DISABLE')

    assert shuttle.enabled is False
    assert shuttle.core.state.mode == shuttle_node.DISABLED
    assert shuttle.core.state.speed == 0.2
    assert shuttle.stopped_by == 'DISABLED'

    node._fill_header = lambda _message: None
    message = node._make_shuttle_state_message({
        'entity_name': 'room315_right_shuttle_1',
        'mode': shuttle.core.state.mode,
        'speed': shuttle.core.state.speed,
    })
    assert message.mode == 'DISABLED'
    assert message.speed == 0.2


def test_generated_room315_shuttle_visual_has_no_gazebo_contact_collisions():
    shuttle_node = _load_module(
        'room_315_kinematic_shuttle_node',
        SCRIPTS_DIR / 'room_315_kinematic_shuttle_node.py',
    )
    node = object.__new__(shuttle_node.Room315KinematicShuttleNode)

    sdf = node._shuttle_visual_sdf(
        'room315_right_shuttle_1',
        shuttle_node.SHUTTLE_VISUAL_NORMAL,
    )

    assert '<collide_bitmask>0x0000</collide_bitmask>' in sdf
    assert '<collide_bitmask>0x0002</collide_bitmask>' not in sdf


def test_stale_gazebo_set_pose_request_does_not_freeze_visual_updates(monkeypatch):
    shuttle_node = _load_module(
        'room_315_kinematic_shuttle_node',
        SCRIPTS_DIR / 'room_315_kinematic_shuttle_node.py',
    )
    node = object.__new__(shuttle_node.Room315KinematicShuttleNode)
    logger = _Logger()
    client = _ReadySetPoseClient()
    shuttle = SimpleNamespace(
        entity_name='room315_right_shuttle_2',
        pending_set_pose=_PendingFuture(),
        pending_set_pose_wall_time=10.0,
        last_gazebo_set_pose_time=None,
        set_pose_timeout_warning_logged=False,
    )
    pose_message = shuttle_node.PoseStamped()
    pose_message.pose.position.x = 1.0

    node.enable_gazebo_set_pose = True
    node.set_pose_client = client
    node.gazebo_set_pose_timeout_s = 0.5
    node.get_clock = lambda: _Clock()
    node.get_logger = lambda: logger
    monkeypatch.setattr(shuttle_node.time, 'monotonic', lambda: 10.75)

    shuttle_node.Room315KinematicShuttleNode._send_gazebo_pose(
        node,
        shuttle,
        pose_message,
    )

    assert len(client.requests) == 1
    assert client.requests[0].entity.name == 'room315_right_shuttle_2'
    assert shuttle.pending_set_pose is not None
    assert shuttle.pending_set_pose_wall_time == 10.75
    assert shuttle.set_pose_timeout_warning_logged is True
    assert any('sending newest pose' in warning for warning in logger.warnings)


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
    assert controller.SWITCH_NEUTRAL_COLORS['diffuse'] == (0.38, 0.40, 0.43, 1.0)


def test_visual_debug_color_launch_argument_is_threaded_to_room315_nodes():
    files = [
        LAUNCH_DIR / 'multi_robot_sim.launch.py',
        LAUNCH_DIR / 'room_315_dual_kinematic_shuttles.launch.py',
        BRINGUP_LAUNCH_DIR / 'room_315_only.launch.py',
        BRINGUP_LAUNCH_DIR / 'full_floor.launch.py',
    ]
    for path in files:
        text = path.read_text(encoding='utf-8')
        if path.parent == BRINGUP_LAUNCH_DIR:
            text += (
                BRINGUP_LAUNCH_DIR / 'room_315_floor_common.py'
            ).read_text(encoding='utf-8')
        assert 'visual_debug_colors' in text or 'room315_visual_debug_colors' in text


def test_room315_sensor_and_visible_pose_rates_are_video_synchronized():
    launch_files = [
        LAUNCH_DIR / 'room_315_dual_kinematic_shuttles.launch.py',
        BRINGUP_LAUNCH_DIR / 'room_315_only.launch.py',
        BRINGUP_LAUNCH_DIR / 'full_floor.launch.py',
    ]
    for path in launch_files:
        text = path.read_text(encoding='utf-8')
        if path.parent == BRINGUP_LAUNCH_DIR:
            text += (
                BRINGUP_LAUNCH_DIR / 'room_315_floor_common.py'
            ).read_text(encoding='utf-8')
        assert 'sync_sensor_feedback_to_motion_tick' in text
        assert 'gazebo_set_pose_rate_hz' in text
        assert 'sensor_marker_visual_hold_s' in text
        assert "default_value='30.0'" in text


def test_light_gui_keeps_entity_selection_inspection_tools():
    text = (CONFIG_DIR / 'mfja_light.gui.config').read_text(encoding='utf-8')

    assert 'SelectEntities' in text
    assert 'EntityContextMenuPlugin' in text
    assert 'ComponentInspector' in text
    assert 'EntityTree' in text


def test_room315_runtime_gui_excludes_crashing_entity_context_menu():
    text = (
        CONFIG_DIR / 'room315_runtime_safe.gui.config'
    ).read_text(encoding='utf-8')
    common = (
        BRINGUP_LAUNCH_DIR / 'room_315_floor_common.py'
    ).read_text(encoding='utf-8')

    assert 'MinimalScene' in text
    assert 'GzSceneManager' in text
    assert 'InteractiveViewControl' in text
    assert 'WorldControl' in text
    assert 'EntityContextMenuPlugin' not in text.split('-->')[-1]
    assert "'gui_config': 'config/room315_runtime_safe.gui.config'" in common
