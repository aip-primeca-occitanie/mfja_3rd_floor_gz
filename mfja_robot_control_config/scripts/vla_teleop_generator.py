#!/usr/bin/env python3
"""
Room 315 VLA teleoperation data generator.

The generator intentionally records high-level language goals while issuing
deterministic low-level VLA JSON commands. Unlike the first draft, station and
loop tasks are now feedback-driven: transport tasks stop on slot sensors, full
loops return to the starting slot/segment, and interior-loop entry avoids
changing A4 to INTERIOR while the shuttle is still on A34E.
"""

import json
import threading
import time
from collections.abc import Callable
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from mfja_rail_interfaces.msg import SensorFeedback
from mfja_rail_interfaces.msg import ShuttleState


SIDES = ('right', 'left')
DEFAULT_SHUTTLE = {
    'right': 'room315_right_shuttle_1',
    'left': 'room315_left_shuttle_1',
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
STOPPER_SENSORS = {
    'A1': 'A1_STOPPER_SENSOR',
    'A2': 'A2_STOPPER_SENSOR',
    'A3': 'A3_STOPPER_SENSOR',
    'A4': 'A4_STOPPER_SENSOR',
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
SENSOR_FEEDBACK_TOPICS = ('feedback', 'position_feedback')


class VLATeleopGenerator(Node):
    def __init__(self):
        super().__init__('vla_teleop_generator')
        self.cmd_pub = self.create_publisher(String, '/room_315/vla/command', 10)
        self.ctrl_pub = self.create_publisher(String, '/room_315/vla/episode_control', 10)
        self.shuttle_states: dict[str, dict[str, dict[str, Any]]] = {
            side: {} for side in SIDES
        }
        self.position_sensors: dict[str, dict[str, dict[str, Any]]] = {
            side: {} for side in SIDES
        }
        self.sensor_feedback_times: dict[str, float] = {side: 0.0 for side in SIDES}

        for side in SIDES:
            prefix = f'/room_315/rails/{side}'
            self.create_subscription(
                ShuttleState,
                f'{prefix}/shuttles/state',
                lambda msg, rail_side=side: self._on_shuttle_state(rail_side, msg),
                10,
            )
            for topic_suffix in SENSOR_FEEDBACK_TOPICS:
                self.create_subscription(
                    SensorFeedback,
                    f'{prefix}/sensors/{topic_suffix}',
                    lambda msg, rail_side=side: self._on_sensor_feedback(rail_side, msg),
                    10,
                )

    def _on_shuttle_state(self, side: str, msg: ShuttleState) -> None:
        self.shuttle_states[side][msg.name] = {
            'mode': msg.mode,
            'segment': msg.current_segment,
            's': float(msg.s),
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

    def slp(self, seconds: float) -> None:
        time.sleep(max(float(seconds), 0.0))

    def ctrl(self, command: str) -> None:
        self.ctrl_pub.publish(String(data=command))
        self.slp(0.8)

    def cmd(self, data: dict[str, Any], wait_s: float = 0.4) -> None:
        self.cmd_pub.publish(String(data=json.dumps(data, sort_keys=True)))
        self.slp(wait_s)

    def sw(self, side: str, state: str) -> None:
        self.cmd({'action': 'switches', 'side': side, 'switches': {'ALL': state}})

    def sw_i(self, side: str, assignments: dict[str, str]) -> None:
        self.cmd({'action': 'switches', 'side': side, 'switches': assignments})

    def st(self, side: str, state: str) -> None:
        self.cmd({'action': 'stoppers', 'side': side, 'stoppers': {'ALL': state}})

    def st_i(self, side: str, assignments: dict[str, str]) -> None:
        self.cmd({'action': 'stoppers', 'side': side, 'stoppers': assignments})

    def on(self, side: str, speed: float = 0.3) -> None:
        self.cmd({'action': 'shuttle', 'side': side, 'command': 'ON', 'speed': speed})

    def off(self, side: str) -> None:
        self.cmd({'action': 'shuttle', 'side': side, 'command': 'OFF'})

    def begin(self, goal: str) -> None:
        self.get_logger().info(f'=== {goal} ===')
        self.ctrl(f'start {goal}')

    def end(self) -> None:
        self.ctrl('stop success')
        self.slp(1.5)

    def wait_subs(self) -> None:
        while rclpy.ok():
            if self.cmd_pub.get_subscription_count() > 0 and self.ctrl_pub.get_subscription_count() > 0:
                break
            self.slp(0.25)
        self.get_logger().info('Subscribers connected')

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
                min(timeout_s, 20.0),
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
            self.wait_until(
                lambda: self.segment(side) not in normalized,
                min(timeout_s, 20.0),
                f'{side} shuttle to leave segments {sorted(normalized)}',
            )
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
        sensor_name = STOPPER_SENSORS[stopper_name]

        def stopped_at_target() -> bool:
            return bool(self.active_sensor(side, (sensor_name,))) and self.mode(side) == 'WAITING'

        ok = self.wait_until(
            stopped_at_target,
            timeout_s,
            f'{side} shuttle to stop at stopper {stopper_name}',
        )
        if ok:
            self.get_logger().info(f'{side} shuttle stopped at stopper {stopper_name}')
        else:
            self.get_logger().warning(
                f'{side} stopper wait diagnostics: target_sensor={sensor_name}, '
                f'active_sensors={self.active_sensor_summary(side)}, '
                f'segment={self.segment(side) or "-"}, mode={self.mode(side) or "-"}'
            )
        return ok

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
        self.sw(side, 'EXTERIOR')
        self.st(side, '0')
        hit = ''
        try:
            self.on(side)
            hit = self.wait_for_slot(side, slots, 90.0, leave_first=require_leave)
        finally:
            self.off(side)
        return bool(hit)

    def stop_at_stopper(self, side: str, stopper_name: str) -> bool:
        if not self.wait_for_state(side):
            return False
        if not self.wait_for_sensor_feedback(side):
            return False
        self.sw(side, 'EXTERIOR')
        self.st_i(side, {'ALL': '0', stopper_name: '1'})
        hit = False
        try:
            self.on(side)
            hit = self.wait_for_stopper_stop(side, stopper_name)
        finally:
            self.off(side)
        return hit

    def move_station_to_station(self, side: str, source: str, target: str) -> None:
        source_slots = STATION_SLOTS[side][source]
        if not self.active_slot(side, source_slots):
            self.get_logger().info(
                f'{side} shuttle is not on a {source} slot; routing to source station first'
            )
            if not self.go_to_station(side, source):
                self.get_logger().warning(f'could not stage {side} shuttle at {source}')
                return
            self.slp(1.0)
        self.go_to_station(side, target, require_leave=True)

    def full_exterior_loop(self, side: str) -> None:
        self.wait_for_state(side)
        self.wait_for_sensor_feedback(side)
        self.sw(side, 'EXTERIOR')
        self.st(side, '0')
        start_slot = self.active_slot(side)
        start_segment = self.segment(side)

        self.get_logger().info(
            f'{side} exterior loop start: slot={start_slot or "-"} '
            f'segment={start_segment or "-"}'
        )
        self.on(side)
        if start_slot:
            self.wait_for_slot(side, (start_slot,), 120.0, leave_first=True)
        elif start_segment:
            self.wait_for_segment(side, {start_segment}, 120.0, leave_first=True)
        else:
            self.get_logger().warning('no start slot/segment known; using timed fallback')
            self.slp(60)
        self.off(side)

    def enter_interior_loop(self, side: str) -> None:
        self.wait_for_state(side)
        self.st(side, '0')
        segment = self.segment(side)
        self.get_logger().info(f'{side} interior entry starts from segment {segment or "-"}')

        if segment == 'A34E':
            # The shuttle is already on the exterior A34 branch. Setting A4=INTERIOR
            # now conflicts with the incoming A34E segment and can cause falling mode.
            self.sw_i(
                side,
                {
                    'A1': 'INTERIOR',
                    'A2': 'INTERIOR',
                    'A3': 'INTERIOR',
                    'A4': 'EXTERIOR',
                },
            )
            self.on(side)
            self.wait_for_segment(side, AFTER_A4_SEGMENTS, 30.0)
            self.sw_i(side, {'A4': 'INTERIOR'})
        elif segment in A34_INTERIOR_SEGMENTS:
            self.sw_i(
                side,
                {
                    'A1': 'INTERIOR',
                    'A2': 'INTERIOR',
                    'A3': 'INTERIOR',
                    'A4': 'INTERIOR',
                },
            )
            self.on(side)
        elif segment in A34_EXTERIOR_SEGMENTS:
            self.sw_i(side, {'A1': 'INTERIOR', 'A2': 'INTERIOR', 'A4': 'EXTERIOR'})
            self.on(side)
            self.wait_for_segment(side, AFTER_A4_SEGMENTS, 30.0)
            self.sw_i(side, {'A3': 'INTERIOR', 'A4': 'INTERIOR'})
        elif segment in AFTER_A4_SEGMENTS:
            self.sw_i(
                side,
                {
                    'A1': 'INTERIOR',
                    'A2': 'INTERIOR',
                    'A3': 'INTERIOR',
                    'A4': 'INTERIOR',
                },
            )
            self.on(side)
        else:
            self.sw_i(
                side,
                {
                    'A1': 'EXTERIOR',
                    'A2': 'EXTERIOR',
                    'A3': 'INTERIOR',
                    'A4': 'INTERIOR',
                },
            )
            self.on(side)
            self.wait_for_sensor_or_segment(
                side,
                INTERIOR_TRIGGER_SENSOR[side],
                {'A34I'},
                30.0,
            )
            self.sw_i(side, {'A1': 'INTERIOR', 'A2': 'INTERIOR'})

        self.slp(20.0)
        self.off(side)

    def safe_a3_interior_transition(self, side: str) -> None:
        if not self.stop_at_stopper(side, 'A3'):
            return
        self.slp(1.0)

        # This scenario is intentionally about switch A3 only. A4 stays
        # unchanged, so the shuttle is stopped shortly after taking A3's
        # interior branch instead of being allowed to reach the unsafe A4 gate.
        self.sw_i(side, {'A3': 'INTERIOR'})
        self.st(side, '0')
        try:
            self.on(side, 0.15)
            self.wait_for_sensor_or_segment(
                side,
                INTERIOR_TRIGGER_SENSOR[side],
                {'A3I', 'A34I'},
                20.0,
            )
            self.slp(1.5)
        finally:
            self.off(side)

    def safe_a4_interior_transition(self, side: str) -> None:
        if not self.stop_at_stopper(side, 'A4'):
            return
        self.slp(1.0)

        # This scenario is intentionally about switch A4 only. Since A3 remains
        # exterior, moving far through A4 in INTERIOR would be unsafe; move just
        # enough to show the guarded transition, then stop before the gate.
        self.sw_i(side, {'A4': 'INTERIOR'})
        self.st(side, '0')
        start_segment = self.segment(side)
        start_s = self.s_position(side)
        try:
            self.on(side, 0.1)
            self.wait_for_segment_progress(side, start_segment, start_s, 0.04, 5.0)
        finally:
            self.off(side)

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
        self.sw('right', 'INTERIOR')
        self.st('right', '0')
        self.on('right')
        self.slp(35)
        self.off('right')
        self.end()

    # ------------------------------------------------------------------
    # RIGHT RAIL - switch transition scenarios
    # ------------------------------------------------------------------
    def r12(self):
        self.begin('right shuttle stop at A3 then switch A3 interior and continue')
        self.safe_a3_interior_transition('right')
        self.end()

    def r13(self):
        self.begin('right shuttle stop at A4 then switch A4 interior and continue')
        self.safe_a4_interior_transition('right')
        self.end()

    # ------------------------------------------------------------------
    # RIGHT RAIL - multi-stop and speed variations
    # ------------------------------------------------------------------
    def r14(self):
        self.begin('right shuttle stop at A3 then resume and stop at A4')
        self.sw('right', 'EXTERIOR')
        self.st_i('right', {'ALL': '0', 'A3': '1'})
        self.on('right')
        self.slp(30)
        self.off('right')
        self.slp(2)
        self.st_i('right', {'A3': '0', 'A4': '1'})
        self.on('right')
        self.slp(20)
        self.off('right')
        self.end()

    def r15(self):
        self.begin('right shuttle stop at A2 then resume and stop at A4')
        self.sw('right', 'EXTERIOR')
        self.st_i('right', {'ALL': '0', 'A2': '1'})
        self.on('right')
        self.slp(30)
        self.off('right')
        self.slp(2)
        self.st_i('right', {'A2': '0', 'A4': '1'})
        self.on('right')
        self.slp(30)
        self.off('right')
        self.end()

    def r16(self):
        self.begin('move right shuttle fast exterior loop')
        self.sw('right', 'EXTERIOR')
        self.st('right', '0')
        self.on('right', 0.6)
        self.slp(25)
        self.off('right')
        self.end()

    def r17(self):
        self.begin('move right shuttle slowly exterior loop')
        self.sw('right', 'EXTERIOR')
        self.st('right', '0')
        self.on('right', 0.1)
        self.slp(50)
        self.off('right')
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
        self.sw('left', 'INTERIOR')
        self.st('left', '0')
        self.on('left')
        self.slp(35)
        self.off('left')
        self.end()

    def l12(self):
        self.begin('left shuttle stop at A3 then switch A3 interior and continue')
        self.safe_a3_interior_transition('left')
        self.end()

    def l13(self):
        self.begin('left shuttle stop at A4 then switch A4 interior and continue')
        self.safe_a4_interior_transition('left')
        self.end()

    # ------------------------------------------------------------------
    # MISC
    # ------------------------------------------------------------------
    def m01(self):
        self.begin('close all right stoppers then open them')
        self.st('right', '1')
        self.slp(3)
        self.st('right', '0')
        self.end()

    def m02(self):
        self.begin('emergency stop all')
        self.sw('right', 'EXTERIOR')
        self.st('right', '0')
        self.on('right')
        self.slp(5)
        self.cmd({'action': 'stop_all'})
        self.slp(3)
        self.end()

    def run_all(self):
        self.wait_subs()
        self.slp(5)
        scenarios = [
            self.r01, self.r02, self.r03, self.r04, self.r05,
            self.r06, self.r07, self.r08, self.r09,
            self.r10, self.r11, self.r12, self.r13, self.r14, self.r15,
            self.r16, self.r17,
            self.l01, self.l02, self.l03, self.l04, self.l05,
            self.l06, self.l07, self.l08, self.l09,
            self.l10, self.l11, self.l12, self.l13,
            self.m01, self.m02,
        ]
        self.get_logger().info(f'====== {len(scenarios)} scenarios ======')
        for i, scenario in enumerate(scenarios, 1):
            self.get_logger().info(f'--- {i}/{len(scenarios)} ---')
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
