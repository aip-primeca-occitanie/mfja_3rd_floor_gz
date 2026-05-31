#!/usr/bin/env python3
"""
Room 315 VLA teleoperation data generator.

The generator intentionally records high-level language goals while issuing
deterministic low-level VLA JSON commands. Unlike the first draft, station and
loop tasks are now feedback-driven: transport tasks stop on slot sensors, full
loops return to the starting slot/segment, and loop mode changes stop at a
guarded gate before switching every turnout to the target mode.
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
from mfja_rail_interfaces.msg import SwitchState as RailSwitchState


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
        self.switch_states: dict[str, dict[str, str]] = {side: {} for side in SIDES}
        self.sensor_feedback_times: dict[str, float] = {side: 0.0 for side in SIDES}
        self.switch_state_times: dict[str, float] = {side: 0.0 for side in SIDES}
        self.loop_mode_by_side: dict[str, str] = {side: '' for side in SIDES}

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
        self._remember_switch_mode(side, state)

    def sw_i(self, side: str, assignments: dict[str, str]) -> None:
        self.cmd({'action': 'switches', 'side': side, 'switches': assignments})
        self._remember_switch_assignments(side, assignments)

    def st(self, side: str, state: str) -> None:
        self.cmd({'action': 'stoppers', 'side': side, 'stoppers': {'ALL': state}})

    def st_i(self, side: str, assignments: dict[str, str]) -> None:
        self.cmd({'action': 'stoppers', 'side': side, 'stoppers': assignments})

    def on(self, side: str, speed: float = 0.3) -> None:
        self.cmd({'action': 'shuttle', 'side': side, 'command': 'ON', 'speed': speed})

    def off(self, side: str) -> None:
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

    def full_exterior_loop(self, side: str, speed: float = 0.3) -> None:
        self.wait_for_state(side)
        self.wait_for_sensor_feedback(side)
        if not self.force_exterior(side):
            return
        self.sw(side, 'EXTERIOR')
        if not self.wait_for_all_switches(side, 'EXTERIOR'):
            return
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
                self.wait_for_slot(side, (start_slot,), 120.0, leave_first=True)
            elif start_segment:
                self.wait_for_segment(side, {start_segment}, 120.0, leave_first=True)
            else:
                self.get_logger().warning('no start slot/segment known; using timed fallback')
                self.slp(60)
        finally:
            self.off(side)

    def stop_then_resume_to_stopper(self, side: str, first: str, second: str) -> None:
        if not self.stop_at_stopper(side, first):
            return
        self.slp(1.0)
        self.st_i(side, {first: '0', second: '1'})
        try:
            self.on(side)
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
            self.on(side)
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
                self.on(side)
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
            self.on(side, 0.15)
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
            self.on(side, 0.15)
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
    def m01(self):
        self.begin('close all right stoppers then open them')
        self.st('right', '1')
        self.slp(3)
        self.st('right', '0')
        self.end()

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
