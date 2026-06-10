#!/usr/bin/env python3
"""
Room 315 VLA teleoperation data generator.

The generator intentionally records high-level language goals while issuing
deterministic low-level VLA JSON commands. Unlike the first draft, station and
loop tasks are now feedback-driven: transport tasks stop on slot sensors, full
loops return to the starting slot/segment, and loop mode changes stop at a
guarded gate before switching every turnout to the target mode.

The model-facing dataset must learn from language, overhead images, and the
previous command. Binary rail sensors, segment, and arc-length values are used
here only by the deterministic expert for safe routing, reset, and evaluation;
the dataset recorder keeps those privileged values out of model_input.
"""

import json
import math
import csv
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from mfja_rail_interfaces.msg import SensorFeedback
from mfja_rail_interfaces.msg import ShuttleState
from mfja_rail_interfaces.msg import SwitchState as RailSwitchState


SIDES = ('right', 'left')
DEFAULT_SHUTTLE = {
    'right': 'room315_right_shuttle_1',
    'left': 'room315_left_shuttle_1',
}
DEFAULT_OBSTACLE_POSE_FILE = '~/.ros/room315_vla_obstacles.json'
DEFAULT_MANUAL_OBSTACLE_POSE = {
    'right': {
        'x': -14.18,
        'y': -4.68,
        'z': 0.856,
        'yaw': 0.0,
    },
    'left': {
        'x': -9.86,
        'y': -4.68,
        'z': 0.856,
        'yaw': 0.0,
    },
}
OBSTACLE_PATH_THRESHOLD_M = 0.45
OBSTACLE_HIDDEN_Z_THRESHOLD_M = 0.2
OBSTACLE_STOP_BEFORE_M = 0.20
FALLBACK_EXTERIOR_ROUTE_SEGMENTS_XY = (
    ((-12.417, -3.779), (-12.415, -3.950)),
    ((-12.415, -3.950), (-12.412, -4.210)),
    ((-12.412, -4.210), (-12.411, -4.378)),
    ((-12.411, -4.378), (-14.171, -4.462)),
    ((-14.171, -4.462), (-14.298, -4.315)),
    ((-14.298, -4.315), (-14.296, -3.842)),
    ((-14.296, -3.842), (-14.170, -3.696)),
    ((-14.170, -3.696), (-12.417, -3.779)),
)
EXTERIOR_LOOP_RAW_SEGMENT_CSVS = (
    'A14.csv',
    'A1E.csv',
    'A12E.csv',
    'A2E.csv',
    'A23.csv',
    'A3E.csv',
    'A34E.csv',
    'A4E.csv',
)
RAIL_TO_GAZEBO_CALIBRATION = {
    'right': {
        'a': -0.893249246800,
        'b': 0.005839516878,
        'tx': -26.921427375871,
        'c': 0.001889497475,
        'd': 1.308619216904,
        'ty': 0.666926143808,
        'scale_x': 1.0,
        'scale_y': 1.0,
        'scale_origin_x': -15.855195431322,
        'scale_origin_y': -4.525523413467,
        'rotation_deg': 0.0,
        'rotation_origin_x': -15.855195431322,
        'rotation_origin_y': -4.525523413467,
        'offset_x': 0.0,
        'offset_y': 0.0,
    },
    'left': {
        'a': -0.8938584503560025,
        'b': 0.005001975618640809,
        'tx': -22.47198317328330,
        'c': 0.001348127530438647,
        'd': 1.255463611604302,
        'ty': 0.4431777232193935,
        'scale_x': 0.98,
        'scale_y': 1.041,
        'scale_origin_x': -10.6365565,
        'scale_origin_y': -4.6995835,
        'rotation_deg': 180.0,
        'rotation_origin_x': -10.6365565,
        'rotation_origin_y': -4.6995835,
        'offset_x': 0.14,
        'offset_y': 0.0,
    },
}
SLOT_SENSORS = {
    'right': {
        '1': 'DZI1R',
        '2': 'DZI2R',
        '3': 'DZI3R',
        '4': 'DZI4R',
    },
    'left': {
        '1': 'DZI1L',
        '2': 'DZI2L',
        '3': 'DZI3L',
        '4': 'DZI4L',
    },
}
STATION_SLOTS = {
    'right': {
        'yaskawa': ('1', '2'),
        'staubli': ('3', '4'),
    },
    'left': {
        'yaskawa': ('1', '2'),
        'kuka': ('3', '4'),
    },
}
INTERIOR_TRIGGER_SENSOR = {
    'right': ('DA3IR',),
    'left': ('DA3IL', 'DA2L'),
}
A34_EXTERIOR_SEGMENTS = {'A34E', 'A4E'}
A34_INTERIOR_SEGMENTS = {'A34I', 'A4I'}
AFTER_A4_SEGMENTS = {'A14', 'A1E', 'A1I', 'A12E', 'A12I'}
INTERIOR_LOOP_SEGMENTS = {'A3I', 'A34I', 'A4I', 'A1I', 'A12I', 'A2I'}
MODE_CHANGE_STOPPER = {
    'right': 'A3',
    'left': 'A1',
}
A4_INTERIOR_APPROACH_SEGMENT = {
    'right': 'A34I',
    'left': 'A34I',
}
A4_INTERIOR_EXIT_SEGMENTS = {
    'right': {'A14'},
    'left': {'A14'},
}
A4_INTERIOR_PASS_PLANS = {
    'right': {
        'approach_segment': 'A34I',
        'exit_segments': {'A14'},
        'pass_switches': {'A4': 'INTERIOR'},
        'stage_from_stopper': 'A3',
        'stage_switches': {'A3': 'INTERIOR', 'A4': 'EXTERIOR'},
        'stage_stopper': 'A4',
    },
    'left': {
        'approach_segment': 'A34I',
        'exit_segments': {'A14'},
        'pass_switches': {'A4': 'INTERIOR'},
        'stage_from_stopper': 'A3',
        'stage_switches': {'A3': 'INTERIOR', 'A4': 'EXTERIOR'},
        'stage_stopper': 'A4',
    },
}
INTERIOR_EXTERIOR_EXIT_PLANS = {
    'right': (
        {
            'segments': {'A3I', 'A34I', 'A4I'},
            'switches': {'A4': 'INTERIOR'},
            'stopper': 'A1',
        },
        {
            'segments': {'A1I', 'A12I', 'A2I'},
            'switches': {'A2': 'INTERIOR'},
            'stopper': 'A3',
        },
    ),
    'left': (
        {
            'segments': {'A3I', 'A34I', 'A4I'},
            'switches': {'A2': 'INTERIOR'},
            'stopper': 'A3',
        },
        {
            'segments': {'A1I', 'A12I', 'A2I'},
            'switches': {'A4': 'INTERIOR'},
            'stopper': 'A1',
        },
    ),
}
STOPPER_SEGMENTS = {
    'right': {
        'A1': {'A14'},
        'A2': {'A12E', 'A12I'},
        'A3': {'A23'},
        'A4': {'A34E', 'A34I'},
    },
    'left': {
        'A1': {'A23'},
        'A2': {'A34E', 'A34I'},
        'A3': {'A14'},
        'A4': {'A12E', 'A12I'},
    },
}
STOPPER_SENSOR_NAMES = {
    'A1': 'A1_STOPPER_SENSOR',
    'A2': 'A2_STOPPER_SENSOR',
    'A3': 'A3_STOPPER_SENSOR',
    'A4': 'A4_STOPPER_SENSOR',
}
SENSOR_FEEDBACK_TOPICS = ('feedback', 'position_feedback')
VISUAL_TRAINING_SCENARIOS = (
    'left_slot3_kuka_then_slot2',
    'right_obstacle_aware_route',
    'left_obstacle_aware_route',
)
KUKA_JOINT_NAMES = (
    'joint_a1',
    'joint_a2',
    'joint_a3',
    'joint_a4',
    'joint_a5',
    'joint_a6',
)
KUKA_SLOT3_INTERLOCK_POSITIONS_RAD = (
    1.57079632679,
    -0.52359877560,
    1.91986217719,
    0.69813170080,
    -0.03490658504,
    0.0,
)
KUKA_SLOT3_INTERLOCK_DURATION_S = 4.0


class VLATeleopGenerator(Node):
    def __init__(self):
        super().__init__('vla_teleop_generator')
        self.declare_parameter('scenario_names', '')
        self.declare_parameter('reset_after_each_scenario', True)
        self.declare_parameter('recorder_status_topic', '/room_315/vla/dataset_status')
        self.declare_parameter('manual_obstacle_pose_file', DEFAULT_OBSTACLE_POSE_FILE)
        for side, defaults in DEFAULT_MANUAL_OBSTACLE_POSE.items():
            self.declare_parameter(f'{side}_manual_obstacle_x', defaults['x'])
            self.declare_parameter(f'{side}_manual_obstacle_y', defaults['y'])
            self.declare_parameter(f'{side}_manual_obstacle_z', defaults['z'])
            self.declare_parameter(f'{side}_manual_obstacle_yaw', defaults['yaw'])
            self.declare_parameter(f'{side}_manual_obstacle_use_pose_file', True)
            self.declare_parameter(
                f'{side}_manual_obstacle_path_threshold_m',
                OBSTACLE_PATH_THRESHOLD_M,
            )
            self.declare_parameter(
                f'{side}_manual_obstacle_hidden_z_threshold_m',
                OBSTACLE_HIDDEN_Z_THRESHOLD_M,
            )
            self.declare_parameter(
                f'{side}_manual_obstacle_stop_before_m',
                OBSTACLE_STOP_BEFORE_M,
            )

        self.cmd_pub = self.create_publisher(String, '/room_315/vla/command', 10)
        self.ctrl_pub = self.create_publisher(String, '/room_315/vla/episode_control', 10)
        self.kuka_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/kuka1/joint_trajectory',
            10,
        )
        self.shuttle_states: dict[str, dict[str, dict[str, Any]]] = {
            side: {} for side in SIDES
        }
        self.position_sensors: dict[str, dict[str, dict[str, Any]]] = {
            side: {} for side in SIDES
        }
        self.switch_states: dict[str, dict[str, str]] = {side: {} for side in SIDES}
        self.sensor_feedback_times: dict[str, float] = {side: 0.0 for side in SIDES}
        self.switch_state_times: dict[str, float] = {side: 0.0 for side in SIDES}
        self.loop_mode_by_side: dict[str, str] = {side: '' for side in SIDES}
        self.current_scenario_sides: set[str] = set()
        self.latest_recorder_status: dict[str, Any] = {}
        self.recorder_status_time_s = 0.0

        for side in SIDES:
            prefix = f'/room_315/rails/{side}'
            self.create_subscription(
                ShuttleState,
                f'{prefix}/shuttles/state',
                lambda msg, rail_side=side: self._on_shuttle_state(rail_side, msg),
                10,
            )
            self.create_subscription(
                RailSwitchState,
                f'{prefix}/switches/state',
                lambda msg, rail_side=side: self._on_switch_state(rail_side, msg),
                10,
            )
            for topic_suffix in SENSOR_FEEDBACK_TOPICS:
                self.create_subscription(
                    SensorFeedback,
                    f'{prefix}/sensors/{topic_suffix}',
                    lambda msg, rail_side=side: self._on_sensor_feedback(rail_side, msg),
                    10,
                )

        self.create_subscription(
            String,
            str(self.get_parameter('recorder_status_topic').value),
            self._on_recorder_status,
            10,
        )

    def _on_shuttle_state(self, side: str, msg: ShuttleState) -> None:
        self.shuttle_states[side][msg.name] = {
            'mode': msg.mode,
            'segment': msg.current_segment,
            's': float(msg.s),
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(msg.z),
            'yaw': float(msg.yaw),
            'speed': float(msg.speed),
        }

    def _on_sensor_feedback(self, side: str, msg: SensorFeedback) -> None:
        active = {}
        for reading in msg.readings:
            if not reading.active:
                continue
            active[reading.name] = {
                'shuttle': reading.shuttle_name,
                'segment': reading.segment,
                's': float(reading.s),
                's_ratio': float(reading.s_ratio),
            }
        self.position_sensors[side] = active
        self.sensor_feedback_times[side] = time.monotonic()

    def _on_switch_state(self, side: str, msg: RailSwitchState) -> None:
        states = {}
        for switch_state in msg.switches:
            name = str(switch_state.name).strip().upper()
            if not name:
                continue
            states[name] = self._canonical_switch_mode(switch_state.state)
        if not states:
            return
        self.switch_states[side] = states
        self.switch_state_times[side] = time.monotonic()
        self._remember_switch_assignments(side, states)

    def _on_recorder_status(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(parsed, dict):
            self.latest_recorder_status = parsed
            self.recorder_status_time_s = time.monotonic()

    def slp(self, seconds: float) -> None:
        time.sleep(max(float(seconds), 0.0))

    def ctrl(self, command: str) -> None:
        self.ctrl_pub.publish(String(data=command))
        self.slp(0.8)

    def cmd(self, data: dict[str, Any], wait_s: float = 0.4) -> None:
        self.cmd_pub.publish(String(data=json.dumps(data, sort_keys=True)))
        self.slp(wait_s)

    def _touch_side(self, side: str) -> None:
        normalized = str(side).strip().lower()
        if normalized in SIDES:
            self.current_scenario_sides.add(normalized)

    def command_kuka_joint_pose(
        self,
        positions_rad: tuple[float, ...],
        *,
        duration_s: float = KUKA_SLOT3_INTERLOCK_DURATION_S,
        wait_timeout_s: float = 5.0,
    ) -> bool:
        if len(positions_rad) != len(KUKA_JOINT_NAMES):
            self.get_logger().warning(
                f'KUKA command expects {len(KUKA_JOINT_NAMES)} joints, '
                f'got {len(positions_rad)}'
            )
            return False

        if self.kuka_trajectory_pub.get_subscription_count() == 0:
            self.wait_until(
                lambda: self.kuka_trajectory_pub.get_subscription_count() > 0,
                wait_timeout_s,
                'KUKA joint trajectory subscriber',
                period_s=0.1,
            )
        if self.kuka_trajectory_pub.get_subscription_count() == 0:
            self.get_logger().warning('no subscriber matched /kuka1/joint_trajectory')
            return False

        whole_seconds = int(duration_s)
        nanoseconds = int(round((float(duration_s) - whole_seconds) * 1_000_000_000))
        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000

        point = JointTrajectoryPoint()
        point.positions = list(positions_rad)
        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = nanoseconds

        message = JointTrajectory()
        message.joint_names = list(KUKA_JOINT_NAMES)
        message.points = [point]

        self.get_logger().info(
            'moving KUKA to slot-3 interlock pose '
            f'positions_rad={list(positions_rad)} duration_s={duration_s:.1f}'
        )
        self.kuka_trajectory_pub.publish(message)
        self.slp(float(duration_s) + 0.5)
        return True

    def sw(self, side: str, state: str) -> None:
        self._touch_side(side)
        self.cmd({'action': 'switches', 'side': side, 'switches': {'ALL': state}})
        self._remember_switch_mode(side, state)

    def sw_i(self, side: str, assignments: dict[str, str]) -> None:
        self._touch_side(side)
        self.cmd({'action': 'switches', 'side': side, 'switches': assignments})
        self._remember_switch_assignments(side, assignments)

    def st(self, side: str, state: str) -> None:
        self._touch_side(side)
        self.cmd({'action': 'stoppers', 'side': side, 'stoppers': {'ALL': state}})

    def st_i(self, side: str, assignments: dict[str, str]) -> None:
        self._touch_side(side)
        self.cmd({'action': 'stoppers', 'side': side, 'stoppers': assignments})

    def on(self, side: str, speed: float = 0.3, *, target_stopper: str | None = None) -> None:
        self._touch_side(side)
        command = {'action': 'shuttle', 'side': side, 'command': 'ON', 'speed': speed}
        if target_stopper:
            command['target_stopper'] = target_stopper
        self.cmd(command)

    def off(self, side: str) -> None:
        self._touch_side(side)
        self.cmd({'action': 'shuttle', 'side': side, 'command': 'OFF'})

    def _remember_switch_mode(self, side: str, state: str) -> None:
        normalized = self._canonical_switch_mode(state)
        if normalized in {'INTERIOR', 'EXTERIOR'}:
            self.loop_mode_by_side[side] = normalized

    def _remember_switch_assignments(self, side: str, assignments: dict[str, str]) -> None:
        normalized = {
            str(name).strip().upper(): self._canonical_switch_mode(state)
            for name, state in assignments.items()
        }
        if normalized.get('ALL') in {'INTERIOR', 'EXTERIOR'}:
            self.loop_mode_by_side[side] = normalized['ALL']
            return
        explicit_states = {
            normalized.get(name)
            for name in ('A1', 'A2', 'A3', 'A4')
        }
        if explicit_states == {'INTERIOR'}:
            self.loop_mode_by_side[side] = 'INTERIOR'
        elif explicit_states == {'EXTERIOR'}:
            self.loop_mode_by_side[side] = 'EXTERIOR'
        elif any(state in {'INTERIOR', 'EXTERIOR'} for state in normalized.values()):
            self.loop_mode_by_side[side] = 'MIXED'

    @staticmethod
    def _canonical_switch_mode(state: str) -> str:
        normalized = str(state).strip().upper()
        if normalized in {'E', 'EXTERIOR'}:
            return 'EXTERIOR'
        if normalized in {'I', 'INTERIOR'}:
            return 'INTERIOR'
        return normalized

    @staticmethod
    def _project_to_segment_xy(
        x: float,
        y: float,
        start: tuple[float, float],
        end: tuple[float, float],
    ) -> tuple[float, float, float]:
        sx, sy = start
        ex, ey = end
        dx = ex - sx
        dy = ey - sy
        segment_length = math.hypot(dx, dy)
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            return 0.0, math.hypot(x - sx, y - sy), 0.0
        u = max(0.0, min(1.0, ((x - sx) * dx + (y - sy) * dy) / length_sq))
        px = sx + u * dx
        py = sy + u * dy
        return u * segment_length, math.hypot(x - px, y - py), segment_length

    @classmethod
    def _room315_kinematics_path(cls) -> Path:
        try:
            from ament_index_python.packages import get_package_share_directory

            return (
                Path(get_package_share_directory('mfja_robot_control_config'))
                / 'config'
                / 'room_315_kinematics'
            )
        except Exception:
            return (
                Path(__file__).resolve().parents[1]
                / 'config'
                / 'room_315_kinematics'
            )

    @classmethod
    def _read_raw_segment_xy(cls, filename: str) -> list[tuple[float, float]]:
        path = cls._room315_kinematics_path() / 'raw_segments' / filename
        points: list[tuple[float, float]] = []
        with path.open(newline='', encoding='utf-8') as handle:
            for row in csv.DictReader(handle):
                points.append((float(row['x']), float(row['y'])))
        return points

    @classmethod
    def _exterior_loop_polyline_xy(cls) -> tuple[tuple[float, float], ...]:
        cached = getattr(cls, '_cached_exterior_loop_polyline_xy', None)
        if cached:
            return cached
        points: list[tuple[float, float]] = []
        try:
            for filename in EXTERIOR_LOOP_RAW_SEGMENT_CSVS:
                segment_points = cls._read_raw_segment_xy(filename)
                if points and segment_points and points[-1] == segment_points[0]:
                    points.extend(segment_points[1:])
                else:
                    points.extend(segment_points)
        except (OSError, KeyError, ValueError) as exc:
            # Keep teleop usable from a partial install, but log-worthy callers still
            # get a geometrically valid fallback.
            points = [FALLBACK_EXTERIOR_ROUTE_SEGMENTS_XY[0][0]]
            points.extend(end for _start, end in FALLBACK_EXTERIOR_ROUTE_SEGMENTS_XY)
        cached = tuple(points)
        setattr(cls, '_cached_exterior_loop_polyline_xy', cached)
        return cached

    @classmethod
    def _exterior_loop_segments_xy(cls) -> tuple[tuple[tuple[float, float], tuple[float, float]], ...]:
        points = cls._exterior_loop_polyline_xy()
        return tuple(zip(points, points[1:]))

    @staticmethod
    def _inverse_planar_rotation(
        x: float,
        y: float,
        *,
        origin_x: float,
        origin_y: float,
        rotation_deg: float,
    ) -> tuple[float, float]:
        theta = -math.radians(rotation_deg)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        dx = x - origin_x
        dy = y - origin_y
        return (
            origin_x + cos_theta * dx - sin_theta * dy,
            origin_y + sin_theta * dx + cos_theta * dy,
        )

    @classmethod
    def _gazebo_to_rail_xy(cls, side: str, x: float, y: float) -> tuple[float, float]:
        calibration = RAIL_TO_GAZEBO_CALIBRATION[side]
        unoffset_x = x - calibration['offset_x']
        unoffset_y = y - calibration['offset_y']
        unrotated_x, unrotated_y = cls._inverse_planar_rotation(
            unoffset_x,
            unoffset_y,
            origin_x=calibration['rotation_origin_x'],
            origin_y=calibration['rotation_origin_y'],
            rotation_deg=calibration['rotation_deg'],
        )
        base_x = (
            calibration['scale_origin_x']
            + (unrotated_x - calibration['scale_origin_x']) / calibration['scale_x']
        )
        base_y = (
            calibration['scale_origin_y']
            + (unrotated_y - calibration['scale_origin_y']) / calibration['scale_y']
        )
        affine_x = base_x - calibration['tx']
        affine_y = base_y - calibration['ty']
        determinant = calibration['a'] * calibration['d'] - calibration['b'] * calibration['c']
        if abs(determinant) <= 1e-12:
            raise ValueError(f'{side} rail/Gazebo calibration is singular')
        rail_x = (calibration['d'] * affine_x - calibration['b'] * affine_y) / determinant
        rail_y = (-calibration['c'] * affine_x + calibration['a'] * affine_y) / determinant
        return rail_x, rail_y

    @classmethod
    def _project_to_exterior_loop_xy(cls, x: float, y: float) -> dict[str, float]:
        best_s = 0.0
        best_distance = float('inf')
        loop_length = 0.0
        for start, end in cls._exterior_loop_segments_xy():
            segment_s, distance, segment_length = cls._project_to_segment_xy(x, y, start, end)
            if distance < best_distance:
                best_s = loop_length + segment_s
                best_distance = distance
            loop_length += segment_length
        return {
            's_m': best_s,
            'path_distance_m': best_distance,
            'loop_length_m': loop_length,
        }

    @staticmethod
    def _loop_distance_ahead(from_s: float, to_s: float, loop_length: float) -> float:
        if loop_length <= 1e-9:
            return 0.0
        return (float(to_s) - float(from_s)) % float(loop_length)

    @classmethod
    def _distance_to_obstacle_path_xy(cls, x: float, y: float) -> float:
        return cls._project_to_exterior_loop_xy(x, y)['path_distance_m']

    def _pose_file_path(self) -> Path:
        return Path(str(self.get_parameter('manual_obstacle_pose_file').value)).expanduser()

    def _cached_obstacle_pose(self, side: str) -> dict[str, Any]:
        if not self._as_bool(self.get_parameter(f'{side}_manual_obstacle_use_pose_file').value):
            return {}
        path = self._pose_file_path()
        if not path.exists():
            return {}
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'could not read obstacle pose cache {path}: {exc}')
            return {}
        if not isinstance(data, dict) or not isinstance(data.get(side), dict):
            return {}
        pose = data[side]
        if not all(name in pose for name in ('x', 'y')):
            return {}
        return pose

    def manual_obstacle_pose(self, side: str) -> dict[str, Any]:
        pose = {
            'x': float(self.get_parameter(f'{side}_manual_obstacle_x').value),
            'y': float(self.get_parameter(f'{side}_manual_obstacle_y').value),
            'z': float(self.get_parameter(f'{side}_manual_obstacle_z').value),
            'yaw': float(self.get_parameter(f'{side}_manual_obstacle_yaw').value),
            'source': 'ros_parameters',
        }
        cached = self._cached_obstacle_pose(side)
        if cached:
            for name in ('x', 'y', 'z', 'yaw'):
                if name in cached:
                    pose[name] = float(cached[name])
            pose['source'] = f'pose_file:{self._pose_file_path()}'
        return pose

    def obstacle_decision(self, side: str, pose: dict[str, Any]) -> dict[str, Any]:
        x = float(pose['x'])
        y = float(pose['y'])
        z = float(pose['z'])
        threshold = float(self.get_parameter(f'{side}_manual_obstacle_path_threshold_m').value)
        hidden_z = float(self.get_parameter(f'{side}_manual_obstacle_hidden_z_threshold_m').value)
        rail_x, rail_y = self._gazebo_to_rail_xy(side, x, y)
        obstacle_projection = self._project_to_exterior_loop_xy(rail_x, rail_y)
        path_distance = obstacle_projection['path_distance_m']
        if z < hidden_z:
            return {
                'blocks_path': False,
                'reason': f'obstacle hidden below z threshold ({z:.3f} < {hidden_z:.3f})',
                'path_distance_m': path_distance,
                'path_threshold_m': threshold,
                'obstacle_rail_x': rail_x,
                'obstacle_rail_y': rail_y,
                'obstacle_s_m': obstacle_projection['s_m'],
                'loop_length_m': obstacle_projection['loop_length_m'],
            }
        return {
            'blocks_path': path_distance <= threshold,
            'reason': 'near rail path' if path_distance <= threshold else 'clear of rail path',
            'path_distance_m': path_distance,
            'path_threshold_m': threshold,
            'obstacle_rail_x': rail_x,
            'obstacle_rail_y': rail_y,
            'obstacle_s_m': obstacle_projection['s_m'],
            'loop_length_m': obstacle_projection['loop_length_m'],
        }

    def begin(self, goal: str) -> None:
        self.current_scenario_sides = set()
        self.get_logger().info(f'=== {goal} ===')
        self.ctrl(f'start {goal}')

    def end(self) -> None:
        stop_sent_at = time.monotonic()
        self.ctrl('stop success')
        if self.wait_for_recorder_stopped_before_reset(stop_sent_at):
            self.reset_after_scenario()
        else:
            self.get_logger().warning(
                'skipping post-scenario reset because the dataset recorder did not '
                'confirm stop; this avoids recording reset frames or reset commands'
            )

    @staticmethod
    def _as_bool(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}

    def reset_after_scenario(self) -> None:
        if not self._as_bool(self.get_parameter('reset_after_each_scenario').value):
            return
        sides = (
            sorted(self.current_scenario_sides)
            if self.current_scenario_sides
            else list(SIDES)
        )
        self.get_logger().info(f'resetting shuttle(s) after scenario on sides={sides}')
        for side in sides:
            self.reset_side_after_scenario(side)
        self.current_scenario_sides = set()

    def reset_side_after_scenario(self, side: str) -> None:
        shuttle = self.shuttle_name(side)
        self.cmd(
            {'action': 'shuttle', 'side': side, 'shuttle': shuttle, 'command': 'OFF'},
            wait_s=0.4,
        )
        self.cmd(
            {'action': 'shuttle', 'side': side, 'shuttle': shuttle, 'command': 'RESET'},
            wait_s=1.0,
        )
        self.wait_until(
            lambda: self.mode(side) != 'FALLING' and bool(self.segment(side)),
            5.0,
            f'{side} shuttle reset pose',
            period_s=0.1,
        )
        self.cmd({'action': 'stoppers', 'side': side, 'stoppers': {'ALL': '0'}}, wait_s=0.4)
        self.cmd(
            {'action': 'switches', 'side': side, 'switches': {'ALL': 'EXTERIOR'}},
            wait_s=0.4,
        )
        self._remember_switch_mode(side, 'EXTERIOR')

    def recorder_has_stopped_since(self, stop_sent_at: float) -> bool:
        if self.ctrl_pub.get_subscription_count() == 0:
            return True
        if self.recorder_status_time_s < stop_sent_at:
            return False
        status = self.latest_recorder_status
        if not status:
            return False
        if status.get('active') is False:
            return True
        return str(status.get('state') or '').strip().lower() in {
            'idle',
            'ready',
            'stopped',
            'discarded',
        }

    def wait_for_recorder_stopped_before_reset(
        self,
        stop_sent_at: float,
        timeout_s: float = 10.0,
    ) -> bool:
        if self.ctrl_pub.get_subscription_count() == 0:
            return True
        return self.wait_until(
            lambda: self.recorder_has_stopped_since(stop_sent_at),
            timeout_s,
            'dataset recorder to stop before reset',
            period_s=0.1,
        )

    def wait_subs(self) -> None:
        while rclpy.ok():
            if self.cmd_pub.get_subscription_count() > 0:
                break
            self.slp(0.25)
        if self.ctrl_pub.get_subscription_count() == 0:
            self.get_logger().warning(
                'No /room_315/vla/episode_control subscriber; running without dataset episodes.'
            )
        self.get_logger().info('VLA command subscriber connected')

    def wait_until(
        self,
        predicate: Callable[[], bool],
        timeout_s: float,
        label: str,
        period_s: float = 0.1,
    ) -> bool:
        deadline = time.monotonic() + float(timeout_s)
        while rclpy.ok() and time.monotonic() <= deadline:
            if predicate():
                return True
            self.slp(period_s)
        self.get_logger().warning(f'timeout waiting for {label}')
        return False

    def wait_for_sensor_feedback(self, side: str, timeout_s: float = 5.0) -> bool:
        self._touch_side(side)
        if self.sensor_feedback_times[side] > 0.0:
            return True
        ok = self.wait_until(
            lambda: self.sensor_feedback_times[side] > 0.0,
            timeout_s,
            f'{side} sensor feedback',
        )
        if not ok:
            self.get_logger().warning(
                f'no {side} sensor feedback received; expected '
                f'/room_315/rails/{side}/sensors/feedback'
            )
        return ok

    def shuttle_name(self, side: str) -> str:
        if self.shuttle_states[side]:
            return sorted(self.shuttle_states[side])[0]
        return DEFAULT_SHUTTLE[side]

    def shuttle_state(self, side: str) -> dict[str, Any]:
        name = self.shuttle_name(side)
        return self.shuttle_states[side].get(name, {})

    def segment(self, side: str) -> str:
        return str(self.shuttle_state(side).get('segment') or '').upper()

    def mode(self, side: str) -> str:
        return str(self.shuttle_state(side).get('mode') or '').upper()

    def s_position(self, side: str) -> float:
        try:
            return float(self.shuttle_state(side).get('s') or 0.0)
        except (TypeError, ValueError):
            return 0.0

    def active_sensor(self, side: str, sensor_names: tuple[str, ...]) -> str:
        active_names = {
            name.casefold(): name
            for name in self.position_sensors[side]
        }
        for sensor_name in sensor_names:
            active_name = active_names.get(sensor_name.casefold())
            if active_name:
                return active_name
        return ''

    def active_slot(self, side: str, slots: tuple[str, ...] | None = None) -> str:
        wanted_slots = slots or tuple(SLOT_SENSORS[side])
        active_names = {
            name.casefold()
            for name in self.position_sensors[side]
        }
        for slot in wanted_slots:
            sensor_name = SLOT_SENSORS[side][slot]
            if sensor_name.casefold() in active_names:
                return slot
        return ''

    def active_sensor_summary(self, side: str) -> str:
        active_names = sorted(self.position_sensors[side])
        return ', '.join(active_names) if active_names else 'none'

    def switch_state_summary(self, side: str) -> str:
        states = self.switch_states[side]
        if not states:
            return 'none'
        return ', '.join(
            f'{name}={states[name]}'
            for name in sorted(states)
        )

    def wait_for_all_switches(self, side: str, state: str, timeout_s: float = 5.0) -> bool:
        target_state = self._canonical_switch_mode(state)
        if not self.switch_states[side]:
            self.wait_until(
                lambda: bool(self.switch_states[side]),
                timeout_s,
                f'{side} switch state feedback',
            )

        switch_names = ('A1', 'A2', 'A3', 'A4')

        def all_switches_match() -> bool:
            states = self.switch_states[side]
            return all(states.get(name) == target_state for name in switch_names)

        ok = self.wait_until(
            all_switches_match,
            timeout_s,
            f'{side} switches to become {target_state}',
        )
        if ok:
            self.get_logger().info(f'{side} switches are all {target_state}')
        else:
            self.get_logger().warning(
                f'{side} switch wait diagnostics: states={self.switch_state_summary(side)}'
            )
        return ok

    def current_station(self, side: str) -> str:
        for station, slots in STATION_SLOTS[side].items():
            if self.active_slot(side, slots):
                return station
        segment = self.segment(side)
        if segment.startswith('A12'):
            return 'yaskawa'
        if segment.startswith('A34'):
            return 'staubli' if side == 'right' else 'kuka'
        return ''

    def wait_for_state(self, side: str, timeout_s: float = 10.0) -> bool:
        self._touch_side(side)
        return self.wait_until(
            lambda: bool(self.shuttle_states[side]),
            timeout_s,
            f'{side} shuttle state',
        )

    def wait_for_slot(
        self,
        side: str,
        slots: tuple[str, ...],
        timeout_s: float,
        *,
        leave_first: bool = False,
    ) -> str:
        if not self.wait_for_sensor_feedback(side):
            return ''
        if leave_first and self.active_slot(side, slots):
            if not self.wait_until(
                lambda: not self.active_slot(side, slots),
                timeout_s,
                f'{side} shuttle to leave slots {slots}',
            ):
                return ''
        hit = ''
        self.wait_until(
            lambda: bool(self.active_slot(side, slots)),
            timeout_s,
            f'{side} shuttle to reach slots {slots}',
        )
        hit = self.active_slot(side, slots)
        if hit:
            self.get_logger().info(f'{side} shuttle reached slot {hit}')
        else:
            self.get_logger().warning(
                f'{side} slot wait diagnostics: active_sensors='
                f'{self.active_sensor_summary(side)}, segment={self.segment(side) or "-"}'
            )
        return hit

    def wait_for_segment(
        self,
        side: str,
        segments: set[str],
        timeout_s: float,
        *,
        leave_first: bool = False,
    ) -> str:
        normalized = {segment.upper() for segment in segments}
        if leave_first and self.segment(side) in normalized:
            if not self.wait_until(
                lambda: self.segment(side) not in normalized,
                timeout_s,
                f'{side} shuttle to leave segments {sorted(normalized)}',
            ):
                return ''
        self.wait_until(
            lambda: self.segment(side) in normalized,
            timeout_s,
            f'{side} shuttle to reach segments {sorted(normalized)}',
        )
        return self.segment(side)

    def wait_for_sensor_or_segment(
        self,
        side: str,
        sensor_names: tuple[str, ...],
        segments: set[str],
        timeout_s: float,
    ) -> bool:
        wanted_sensors = {name.casefold() for name in sensor_names}
        wanted_segments = {segment.upper() for segment in segments}

        def triggered() -> bool:
            active_names = {name.casefold() for name in self.position_sensors[side]}
            return bool(active_names & wanted_sensors) or self.segment(side) in wanted_segments

        return self.wait_until(
            triggered,
            timeout_s,
            f'{side} trigger sensors {sensor_names} or segments {sorted(wanted_segments)}',
        )

    def wait_for_stopper_stop(
        self,
        side: str,
        stopper_name: str,
        timeout_s: float = 90.0,
    ) -> bool:
        # NOTE: We only check mode == WAITING, not the stopper sensor.
        # The stopper sensor radius_m (0.08) is smaller than
        # before_stopper_m (0.1), so the shuttle at the stopper point
        # is already outside the sensor detection range.
        was_moving = False
        sensor_name = STOPPER_SENSOR_NAMES[stopper_name]
        target_segments = STOPPER_SEGMENTS[side][stopper_name]

        def stopped_at_target() -> bool:
            nonlocal was_moving
            if self.mode(side) == 'MOVING':
                was_moving = True
            if self.mode(side) != 'WAITING':
                return False
            if self.active_sensor(side, (sensor_name,)):
                return True
            if self.segment(side) not in target_segments:
                return False
            return was_moving or self.s_position(side) > 0.0

        ok = self.wait_until(
            stopped_at_target,
            timeout_s,
            f'{side} shuttle to stop at stopper {stopper_name}',
        )
        if ok:
            self.get_logger().info(f'{side} shuttle stopped at stopper {stopper_name}')
        else:
            self.get_logger().warning(
                f'{side} stopper wait diagnostics: '
                f'active_sensors={self.active_sensor_summary(side)}, '
                f'segment={self.segment(side) or "-"}, mode={self.mode(side) or "-"}'
            )
        return ok

    def recover_if_falling(self, side: str) -> bool:
        if self.mode(side) != 'FALLING':
            return True
        self.get_logger().warning(f'{side} shuttle is in FALLING mode; resetting before scenario')
        self.cmd({'action': 'shuttle', 'side': side, 'command': 'RESET'}, wait_s=1.0)
        self.off(side)
        return self.wait_until(
            lambda: self.mode(side) != 'FALLING' and bool(self.segment(side)),
            10.0,
            f'{side} shuttle to recover from FALLING',
        )

    def wait_for_segment_progress(
        self,
        side: str,
        start_segment: str,
        start_s: float,
        min_delta_m: float,
        timeout_s: float,
    ) -> bool:
        target_segment = start_segment.upper()

        def moved_safely() -> bool:
            if self.mode(side) == 'FALLING':
                return True
            if self.segment(side) != target_segment:
                return True
            return (self.s_position(side) - start_s) >= min_delta_m

        ok = self.wait_until(
            moved_safely,
            timeout_s,
            f'{side} shuttle to move {min_delta_m:.2f}m on {target_segment}',
            period_s=0.05,
        )
        if self.mode(side) == 'FALLING':
            self.get_logger().warning(
                f'{side} shuttle entered FALLING during guarded progress on {target_segment}'
            )
            return False
        return ok

    def go_to_station(self, side: str, station: str, *, require_leave: bool = False) -> bool:
        if not self.wait_for_state(side):
            return False
        if not self.wait_for_sensor_feedback(side):
            return False
        slots = STATION_SLOTS[side][station]
        if self.active_slot(side, slots) and not require_leave:
            self.get_logger().info(f'{side} shuttle is already on a {station} slot')
            self.off(side)
            return True

        self.get_logger().info(f'moving {side} shuttle to {station} slots {slots}')
        if not self.force_exterior(side):
            return False
        self.st(side, '0')
        hit = ''
        try:
            self.on(side)
            hit = self.wait_for_slot(side, slots, 90.0, leave_first=require_leave)
        finally:
            self.off(side)
        return bool(hit)

    def go_to_slot(self, side: str, slot: str, *, require_leave: bool = False) -> bool:
        if slot not in SLOT_SENSORS[side]:
            self.get_logger().warning(f'unsupported {side} slot target {slot!r}')
            return False
        if not self.wait_for_state(side):
            return False
        if not self.wait_for_sensor_feedback(side):
            return False
        if not self.force_exterior(side):
            return False

        if self.active_slot(side, (slot,)) and not require_leave:
            self.get_logger().info(f'{side} shuttle is already centered on slot {slot}')
            self.off(side)
            return True

        self.get_logger().info(
            f'moving {side} shuttle to visual slot marker {slot} '
            f'({SLOT_SENSORS[side][slot]})'
        )
        self.st(side, '0')
        hit = ''
        try:
            self.on(side, 0.2)
            hit = self.wait_for_slot(
                side,
                (slot,),
                90.0,
                leave_first=require_leave,
            )
        finally:
            self.off(side)
        return bool(hit)

    def center_at_station(
        self,
        side: str,
        station: str,
        *,
        require_leave: bool = True,
    ) -> bool:
        self.get_logger().info(f'centering {side} shuttle at {station} station using slot sensors')
        return self.go_to_station(side, station, require_leave=require_leave)

    def wait_until_before_manual_obstacle(
        self,
        side: str,
        decision: dict[str, Any],
        *,
        stop_before_m: float,
        timeout_s: float = 120.0,
    ) -> bool:
        obstacle_s = float(decision['obstacle_s_m'])
        loop_length = float(decision['loop_length_m'])
        last_remaining = float('inf')
        last_path_distance = float('inf')

        def reached_stop_point() -> bool:
            nonlocal last_remaining, last_path_distance
            state = self.shuttle_state(side)
            projection = self._project_to_exterior_loop_xy(
                float(state.get('x') or 0.0),
                float(state.get('y') or 0.0),
            )
            last_path_distance = projection['path_distance_m']
            last_remaining = self._loop_distance_ahead(
                projection['s_m'],
                obstacle_s,
                loop_length,
            )
            return last_remaining <= float(stop_before_m)

        ok = self.wait_until(
            reached_stop_point,
            timeout_s,
            f'{side} shuttle to reach {stop_before_m:.2f}m before obstacle',
            period_s=0.03,
        )
        if ok:
            self.get_logger().info(
                f'{side} reached manual-obstacle stop point: '
                f'distance_ahead={last_remaining:.3f}m, '
                f'path_offset={last_path_distance:.3f}m'
            )
        else:
            self.get_logger().warning(
                f'{side} manual-obstacle stop diagnostics: '
                f'distance_ahead={last_remaining:.3f}m, '
                f'path_offset={last_path_distance:.3f}m, '
                f'segment={self.segment(side) or "-"}, mode={self.mode(side) or "-"}'
            )
        return ok

    def stop_before_manual_obstacle(
        self,
        side: str,
        *,
        speed: float = 0.3,
    ) -> bool:
        """Run the exterior loop and stop before a visible obstacle if it blocks it."""
        pose = self.manual_obstacle_pose(side)
        self.get_logger().info(
            f'{side} obstacle pose x={pose["x"]:.3f}, y={pose["y"]:.3f}, '
            f'z={pose["z"]:.3f}, source={pose["source"]}; scenario will not move it'
        )

        decision = self.obstacle_decision(side, pose)
        stop_before_m = float(self.get_parameter(f'{side}_manual_obstacle_stop_before_m').value)
        self.get_logger().info(
            f'{side} obstacle decision: blocks_path={decision["blocks_path"]}, '
            f'reason={decision["reason"]}, distance={decision["path_distance_m"]:.3f}m, '
            f'threshold={decision["path_threshold_m"]:.3f}m, '
            f'rail_xy=({decision["obstacle_rail_x"]:.3f}, {decision["obstacle_rail_y"]:.3f}), '
            f'stop_before={stop_before_m:.3f}m, route=EXTERIOR_LOOP'
        )

        if not decision['blocks_path']:
            self.get_logger().info(
                f'{side} obstacle is clear of the exterior loop; completing one big loop'
            )
            return self.full_exterior_loop(side, speed=speed)

        if not self.wait_for_state(side):
            return False
        if not self.wait_for_sensor_feedback(side):
            return False
        if not self.force_exterior(side):
            return False
        self.sw(side, 'EXTERIOR')
        if not self.wait_for_all_switches(side, 'EXTERIOR'):
            return False
        self.st(side, '0')
        stopped = False
        try:
            self.on(side, speed)
            stopped = self.wait_until_before_manual_obstacle(
                side,
                decision,
                stop_before_m=stop_before_m,
            )
        finally:
            self.off(side)
        if stopped:
            self.get_logger().info(
                f'{side} shuttle stopped directly before visual obstacle'
            )
        return stopped

    def stop_at_stopper(self, side: str, stopper_name: str) -> bool:
        if not self.wait_for_state(side):
            return False
        if not self.wait_for_sensor_feedback(side):
            return False

        if not self.force_exterior(side):
            return False
        self.st_i(side, {'ALL': '0', stopper_name: '1'})
        hit = False
        try:
            self.on(side, target_stopper=stopper_name)
            hit = self.wait_for_stopper_stop(side, stopper_name)
        finally:
            self.off(side)
        return hit

    def move_station_to_station(self, side: str, source: str, target: str) -> None:
        if not self.wait_for_state(side):
            return
        if not self.wait_for_sensor_feedback(side):
            return
        current = self.current_station(side)
        if current != source:
            self.get_logger().info(
                f'{side} shuttle is at {current or "unknown station"}, not {source}; '
                f'routing to source station first'
            )
            if not self.go_to_station(side, source):
                self.get_logger().warning(f'could not stage {side} shuttle at {source}')
                return
            self.slp(1.0)
        self.go_to_station(side, target, require_leave=True)

    def full_exterior_loop(self, side: str, speed: float = 0.3) -> bool:
        if not self.wait_for_state(side):
            return False
        if not self.wait_for_sensor_feedback(side):
            return False
        if not self.force_exterior(side):
            return False
        self.sw(side, 'EXTERIOR')
        if not self.wait_for_all_switches(side, 'EXTERIOR'):
            return False
        self.st(side, '0')
        start_slot = self.active_slot(side)
        start_segment = self.segment(side)

        self.get_logger().info(
            f'{side} exterior loop start: slot={start_slot or "-"} '
            f'segment={start_segment or "-"}'
        )
        try:
            self.on(side, speed)
            if start_slot:
                return bool(self.wait_for_slot(side, (start_slot,), 120.0, leave_first=True))
            elif start_segment:
                return self.wait_for_segment(side, {start_segment}, 120.0, leave_first=True)
            self.get_logger().warning('no start slot/segment known; using timed fallback')
            self.slp(60)
            return True
        finally:
            self.off(side)

    def stop_then_resume_to_stopper(self, side: str, first: str, second: str) -> None:
        if not self.stop_at_stopper(side, first):
            return
        self.slp(1.0)
        self.st_i(side, {first: '0', second: '1'})
        try:
            self.on(side, target_stopper=second)
            self.wait_for_stopper_stop(side, second, 90.0)
        finally:
            self.off(side)

    def run_interior_loop(self, side: str, duration_s: float = 35.0) -> None:
        self.enter_interior_loop(side)
        self.sw(side, 'INTERIOR')
        if not self.wait_for_all_switches(side, 'INTERIOR'):
            return
        self.st(side, '0')
        try:
            self.on(side)
            self.slp(duration_s)
        finally:
            self.off(side)

    def _in_interior_loop_context(self, side: str) -> bool:
        return (
            self.segment(side) in INTERIOR_LOOP_SEGMENTS
            or self.loop_mode_by_side.get(side) == 'INTERIOR'
        )

    def _stop_before_mode_change_gate(
        self,
        side: str,
        approach_switch_state: str,
        timeout_s: float = 120.0,
    ) -> bool:
        gate = MODE_CHANGE_STOPPER[side]
        approach_state = self._canonical_switch_mode(approach_switch_state)
        self.get_logger().info(
            f'{side} mode change: stopping before {gate} with switches {approach_state}'
        )
        self.off(side)
        self.sw(side, approach_state)
        if not self.wait_for_all_switches(side, approach_state):
            return False
        self.st_i(side, {'ALL': '0', gate: '1'})
        try:
            self.on(side, target_stopper=gate)
            return self.wait_for_stopper_stop(side, gate, timeout_s)
        finally:
            self.off(side)

    def _set_all_switches_before_continuing(self, side: str, target_state: str) -> bool:
        gate = MODE_CHANGE_STOPPER[side]
        target = self._canonical_switch_mode(target_state)
        self.get_logger().info(
            f'{side} mode change: shuttle is stopped before {gate}; '
            f'waiting for all switches to become {target}'
        )
        self.sw(side, target)
        return self.wait_for_all_switches(side, target)

    def force_exterior(self, side: str) -> bool:
        if not self.wait_for_state(side):
            return False
        if not self.wait_for_sensor_feedback(side):
            return False
        self.off(side)
        if not self.recover_if_falling(side):
            return False

        if self._in_interior_loop_context(side):
            if not self._stop_before_mode_change_gate(side, 'INTERIOR'):
                return False
            if not self._set_all_switches_before_continuing(side, 'EXTERIOR'):
                return False
            self.st(side, '0')
            return True

        segment = self.segment(side)
        for plan in INTERIOR_EXTERIOR_EXIT_PLANS[side]:
            if segment not in plan['segments']:
                continue
            self.get_logger().info(
                f'{side} shuttle is on interior segment {segment}; '
                f'exiting safely via stopper {plan["stopper"]}'
            )
            self.sw_i(side, plan['switches'])
            self.st_i(side, {'ALL': '0', plan['stopper']: '1'})
            try:
                self.on(side, target_stopper=plan['stopper'])
                self.wait_for_stopper_stop(side, plan['stopper'], 60.0)
            finally:
                self.off(side)
            break

        segment = self.segment(side)
        safe_segments = {'A14', 'A23', 'A12E', 'A34E', 'A1E', 'A2E', 'A3E', 'A4E'}
        if not segment or segment in safe_segments:
            self.sw(side, 'EXTERIOR')
            if not self.wait_for_all_switches(side, 'EXTERIOR'):
                return False
            self.st(side, '0')
            return True
        self.get_logger().warning(
            f'{side} shuttle is on unexpected segment {segment}; leaving switches unchanged'
        )
        return False

    def enter_interior_loop(self, side: str) -> None:
        if not self.wait_for_state(side):
            return
        if not self.wait_for_sensor_feedback(side):
            return
        if not self.recover_if_falling(side):
            return
        segment = self.segment(side)
        self.get_logger().info(f'{side} interior entry starts from segment {segment or "-"}')
        approach_state = 'INTERIOR' if self._in_interior_loop_context(side) else 'EXTERIOR'
        if not self._stop_before_mode_change_gate(side, approach_state):
            return
        if not self._set_all_switches_before_continuing(side, 'INTERIOR'):
            return
        self.st(side, '0')
        try:
            self.on(side)
            self.slp(20.0)
        finally:
            self.off(side)

    def route_through_a3_into_interior_branch(self, side: str) -> None:
        if not self.stop_at_stopper(side, 'A3'):
            return
        self.slp(1.0)

        # Task semantics: stage on the exterior approach to A3, choose the
        # interior branch at A3, then stop before the next guarded switch.
        self.sw_i(side, {'A3': 'INTERIOR'})
        self.st_i(side, {'ALL': '0', 'A4': '1'})
        try:
            self.on(side, 0.15, target_stopper='A4')
            self.wait_for_stopper_stop(side, 'A4', 30.0)
        finally:
            self.off(side)

    def pass_a4_from_interior_approach(self, side: str) -> None:
        if not self.stage_a4_interior_approach(side):
            return
        self.slp(1.0)

        plan = A4_INTERIOR_PASS_PLANS[side]
        approach_segment = plan['approach_segment']
        exit_segments = plan['exit_segments']
        self.get_logger().info(
            f'{side} A4 interior-pass task: shuttle staged on {approach_segment}; '
            f'now selecting {plan["pass_switches"]} and releasing the staging stopper'
        )
        self.sw_i(side, plan['pass_switches'])
        self.st_i(side, {plan['stage_stopper']: '0'})
        cleared_a4 = False
        try:
            self.on(side, 0.15)
            self.wait_for_segment(
                side,
                exit_segments,
                30.0,
                leave_first=False,
            )
            cleared_a4 = self.segment(side) in exit_segments
            self.slp(1.5)
        finally:
            self.off(side)

        if cleared_a4:
            # Restore the exterior loop only after the shuttle has cleared A4.
            restore_switches = {
                switch_name: 'EXTERIOR'
                for switch_name in {
                    *plan['stage_switches'],
                    *plan['pass_switches'],
                }
            }
            self.sw_i(side, restore_switches)
            self.st(side, '0')
        else:
            self.get_logger().warning(
                f'{side} A4 interior-pass task did not clear into {sorted(exit_segments)}; '
                'leaving A4 INTERIOR to avoid creating an unsafe guarded segment'
            )

    def stage_a4_interior_approach(self, side: str) -> bool:
        if not self.wait_for_state(side):
            return False
        self.off(side)
        if not self.recover_if_falling(side):
            return False

        plan = A4_INTERIOR_PASS_PLANS[side]
        approach_segment = plan['approach_segment']
        if self.segment(side) == approach_segment:
            self.get_logger().info(
                f'{side} shuttle already staged on {approach_segment} before A4; '
                f'closing stopper {plan["stage_stopper"]} and keeping the switch state safe'
            )
            self.st_i(side, {'ALL': '0', plan['stage_stopper']: '1'})
            return True

        self.get_logger().info(
            f'staging {side} shuttle on {approach_segment} before A4; '
            f'{plan["pass_switches"]} will be selected only after staging'
        )
        if not self.stop_at_stopper(side, plan['stage_from_stopper']):
            return False
        self.slp(1.0)

        self.sw_i(side, plan['stage_switches'])
        self.st_i(side, {'ALL': '0', plan['stage_stopper']: '1'})
        try:
            self.on(side, 0.15, target_stopper=plan['stage_stopper'])
            if not self.wait_for_stopper_stop(side, plan['stage_stopper'], 45.0):
                return False
        finally:
            self.off(side)

        if self.segment(side) != approach_segment:
            self.get_logger().warning(
                f'{side} A4 staging expected {approach_segment}, '
                f'but shuttle is on {self.segment(side) or "-"}'
            )
            return False
        return True

    # ------------------------------------------------------------------
    # RIGHT RAIL - transport and station scenarios
    # ------------------------------------------------------------------
    def r01(self):
        self.begin('move right shuttle full exterior loop')
        self.full_exterior_loop('right')
        self.end()

    def r02(self):
        self.begin('move right shuttle from yaskawa to staubli')
        self.move_station_to_station('right', 'yaskawa', 'staubli')
        self.end()

    def r03(self):
        self.begin('move right shuttle from staubli to yaskawa')
        self.move_station_to_station('right', 'staubli', 'yaskawa')
        self.end()

    def r04(self):
        self.begin('go to staubli on right rail')
        self.go_to_station('right', 'staubli')
        self.end()

    def r05(self):
        self.begin('go to yaskawa on right rail')
        self.go_to_station('right', 'yaskawa')
        self.end()

    # ------------------------------------------------------------------
    # RIGHT RAIL - stopper-specific scenarios
    # ------------------------------------------------------------------
    def r06(self):
        self.begin('stop right shuttle at stopper A1')
        self.stop_at_stopper('right', 'A1')
        self.end()

    def r07(self):
        self.begin('stop right shuttle at stopper A2')
        self.stop_at_stopper('right', 'A2')
        self.end()

    def r08(self):
        self.begin('stop right shuttle at stopper A3')
        self.stop_at_stopper('right', 'A3')
        self.end()

    def r09(self):
        self.begin('stop right shuttle at stopper A4')
        self.stop_at_stopper('right', 'A4')
        self.end()

    # ------------------------------------------------------------------
    # RIGHT RAIL - interior loop
    # ------------------------------------------------------------------
    def r10(self):
        self.begin('right shuttle enter interior loop from exterior')
        self.enter_interior_loop('right')
        self.end()

    def r11(self):
        self.begin('move right shuttle on interior loop')
        self.run_interior_loop('right')
        self.end()

    # ------------------------------------------------------------------
    # RIGHT RAIL - switch transition scenarios
    # ------------------------------------------------------------------
    def r12(self):
        self.begin('route right shuttle through A3 into the interior branch')
        self.route_through_a3_into_interior_branch('right')
        self.end()

    def r13(self):
        self.begin('pass right shuttle through A4 from the interior approach')
        self.pass_a4_from_interior_approach('right')
        self.end()

    # ------------------------------------------------------------------
    # RIGHT RAIL - multi-stop and speed variations
    # ------------------------------------------------------------------
    def r14(self):
        self.begin('right shuttle stop at A3 then resume and stop at A4')
        self.stop_then_resume_to_stopper('right', 'A3', 'A4')
        self.end()

    def r15(self):
        self.begin('right shuttle stop at A2 then resume and stop at A4')
        self.stop_then_resume_to_stopper('right', 'A2', 'A4')
        self.end()

    def r16(self):
        self.begin('complete one fast right exterior loop')
        self.full_exterior_loop('right', speed=0.6)
        self.end()

    def r17(self):
        self.begin('complete one slow right exterior loop')
        self.full_exterior_loop('right', speed=0.1)
        self.end()

    # ------------------------------------------------------------------
    # LEFT RAIL - transport and station scenarios
    # ------------------------------------------------------------------
    def l01(self):
        self.begin('move left shuttle full exterior loop')
        self.full_exterior_loop('left')
        self.end()

    def l02(self):
        self.begin('move left shuttle from yaskawa to kuka')
        self.move_station_to_station('left', 'yaskawa', 'kuka')
        self.end()

    def l03(self):
        self.begin('move left shuttle from kuka to yaskawa')
        self.move_station_to_station('left', 'kuka', 'yaskawa')
        self.end()

    def l04(self):
        self.begin('go to kuka on left rail')
        self.go_to_station('left', 'kuka')
        self.end()

    def l05(self):
        self.begin('go to yaskawa on left rail')
        self.go_to_station('left', 'yaskawa')
        self.end()

    # ------------------------------------------------------------------
    # LEFT RAIL - stopper-specific scenarios
    # ------------------------------------------------------------------
    def l06(self):
        self.begin('stop left shuttle at stopper A1')
        self.stop_at_stopper('left', 'A1')
        self.end()

    def l07(self):
        self.begin('stop left shuttle at stopper A2')
        self.stop_at_stopper('left', 'A2')
        self.end()

    def l08(self):
        self.begin('stop left shuttle at stopper A3')
        self.stop_at_stopper('left', 'A3')
        self.end()

    def l09(self):
        self.begin('stop left shuttle at stopper A4')
        self.stop_at_stopper('left', 'A4')
        self.end()

    # ------------------------------------------------------------------
    # LEFT RAIL - interior and transition scenarios
    # ------------------------------------------------------------------
    def l10(self):
        self.begin('left shuttle enter interior loop from exterior')
        self.enter_interior_loop('left')
        self.end()

    def l11(self):
        self.begin('move left shuttle on interior loop')
        self.run_interior_loop('left')
        self.end()

    def l12(self):
        self.begin('route left shuttle through A3 into the interior branch')
        self.route_through_a3_into_interior_branch('left')
        self.end()

    def l13(self):
        self.begin('pass left shuttle through A4 from the interior approach')
        self.pass_a4_from_interior_approach('left')
        self.end()

    # ------------------------------------------------------------------
    # MISC
    # ------------------------------------------------------------------
    def m02(self):
        self.begin('emergency stop all')
        if not self.force_exterior('right'):
            self.end()
            return
        self.st('right', '0')
        self.on('right')
        self.slp(5)
        self.cmd({'action': 'stop_all'})
        self.slp(3)
        self.end()

    # ------------------------------------------------------------------
    # VLA perception, recovery, and robustness scenarios
    # ------------------------------------------------------------------
    def m08(self):
        self.begin('left_slot3_kuka_then_slot2')
        if not self.go_to_slot('left', '3', require_leave=True):
            self.end()
            return
        if not self.command_kuka_joint_pose(KUKA_SLOT3_INTERLOCK_POSITIONS_RAD):
            self.end()
            return
        self.go_to_slot('left', '2', require_leave=True)
        self.end()

    def m10(self):
        self.begin('right_obstacle_aware_route')
        self.stop_before_manual_obstacle('right')
        self.end()

    def m11(self):
        self.begin('left_obstacle_aware_route')
        self.stop_before_manual_obstacle('left')
        self.end()

    @staticmethod
    def _scenario_key(value: Any) -> str:
        normalized = str(value or '').strip().replace('-', '_')
        return '_'.join(normalized.split()).casefold()

    def select_scenarios(
        self,
        scenarios: list[tuple[str, str, Callable[[], None]]],
        raw_filter: str,
    ) -> list[tuple[str, str, Callable[[], None]]]:
        text = str(raw_filter or '').strip()
        if not text:
            return scenarios

        wanted_by_key = {
            self._scenario_key(token): token.strip()
            for token in text.replace(';', ',').split(',')
            if token.strip()
        }
        if not wanted_by_key or set(wanted_by_key) == {'all'}:
            return scenarios

        selected: list[tuple[str, str, Callable[[], None]]] = []
        matched_keys: set[str] = set()
        wanted_keys = set(wanted_by_key)
        for code, name, scenario in scenarios:
            aliases = {
                self._scenario_key(code),
                self._scenario_key(name),
                self._scenario_key(getattr(scenario, '__name__', '')),
            }
            if aliases & wanted_keys:
                selected.append((code, name, scenario))
                matched_keys.update(aliases & wanted_keys)

        missing = [
            wanted_by_key[key]
            for key in sorted(wanted_by_key)
            if key not in matched_keys
        ]
        if missing:
            allowed = ', '.join(
                f'{code}:{name}'
                for code, name, _scenario in scenarios
            )
            self.get_logger().warning(
                f'unknown scenario_names entries: {missing}; allowed: {allowed}'
            )
        return selected

    def run_all(self):
        self.wait_subs()
        self.slp(5)
        scenarios = [
            ('r01', 'move_right_shuttle_full_exterior_loop', self.r01),
            ('r02', 'move_right_shuttle_from_yaskawa_to_staubli', self.r02),
            ('r03', 'move_right_shuttle_from_staubli_to_yaskawa', self.r03),
            ('r04', 'go_to_staubli_on_right_rail', self.r04),
            ('r05', 'go_to_yaskawa_on_right_rail', self.r05),
            ('r06', 'stop_right_shuttle_at_stopper_A1', self.r06),
            ('r07', 'stop_right_shuttle_at_stopper_A2', self.r07),
            ('r08', 'stop_right_shuttle_at_stopper_A3', self.r08),
            ('r09', 'stop_right_shuttle_at_stopper_A4', self.r09),
            ('r10', 'right_shuttle_enter_interior_loop_from_exterior', self.r10),
            ('r11', 'move_right_shuttle_on_interior_loop', self.r11),
            ('r12', 'route_right_shuttle_through_A3_into_the_interior_branch', self.r12),
            ('r13', 'pass_right_shuttle_through_A4_from_the_interior_approach', self.r13),
            ('r14', 'right_shuttle_stop_at_A3_then_resume_and_stop_at_A4', self.r14),
            ('r15', 'right_shuttle_stop_at_A2_then_resume_and_stop_at_A4', self.r15),
            ('r16', 'complete_one_fast_right_exterior_loop', self.r16),
            ('r17', 'complete_one_slow_right_exterior_loop', self.r17),
            ('l01', 'move_left_shuttle_full_exterior_loop', self.l01),
            ('l02', 'move_left_shuttle_from_yaskawa_to_kuka', self.l02),
            ('l03', 'move_left_shuttle_from_kuka_to_yaskawa', self.l03),
            ('l04', 'go_to_kuka_on_left_rail', self.l04),
            ('l05', 'go_to_yaskawa_on_left_rail', self.l05),
            ('l06', 'stop_left_shuttle_at_stopper_A1', self.l06),
            ('l07', 'stop_left_shuttle_at_stopper_A2', self.l07),
            ('l08', 'stop_left_shuttle_at_stopper_A3', self.l08),
            ('l09', 'stop_left_shuttle_at_stopper_A4', self.l09),
            ('l10', 'left_shuttle_enter_interior_loop_from_exterior', self.l10),
            ('l11', 'move_left_shuttle_on_interior_loop', self.l11),
            ('l12', 'route_left_shuttle_through_A3_into_the_interior_branch', self.l12),
            ('l13', 'pass_left_shuttle_through_A4_from_the_interior_approach', self.l13),
            ('m02', 'emergency_stop_all', self.m02),
            ('m08', 'left_slot3_kuka_then_slot2', self.m08),
            ('m10', 'right_obstacle_aware_route', self.m10),
            ('m11', 'left_obstacle_aware_route', self.m11),
        ]
        scenarios = self.select_scenarios(
            scenarios,
            str(self.get_parameter('scenario_names').value),
        )
        if not scenarios:
            self.get_logger().error('no VLA teleop scenarios selected')
            return
        self.get_logger().info(f'====== {len(scenarios)} scenarios ======')
        for i, (scenario_code, scenario_name, scenario) in enumerate(scenarios, 1):
            self.get_logger().info(f'--- {i}/{len(scenarios)} ---')
            self.get_logger().info(f'scenario_code={scenario_code}')
            self.get_logger().info(f'scenario_name={scenario_name}')
            scenario()
            self.slp(3)
        self.get_logger().info('====== ALL DONE ======')


def main(args=None):
    rclpy.init(args=args)
    node = VLATeleopGenerator()
    threading.Thread(target=node.run_all, daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
