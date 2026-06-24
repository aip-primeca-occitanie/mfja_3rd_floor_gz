#!/usr/bin/env python3
"""Room 315 PlanSys2-backed PDDL scenario generator.

Dry-run mode produces planned episode JSON only. Execute mode publishes only to
the existing VLA supervisor and dataset-control topics; it does not bypass the
supervisor, execute Gazebo directly, or modify model_input.
"""

import argparse
import importlib
import json
import random
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_pddl_language_generator import generate_language
from room_315_pddl_plan_translator import parse_plan_text
from room_315_pddl_plan_translator import translate_plan
from room_315_pddl_validation_gate import build_validation_result
from room_315_pddl_validation_gate import runtime_failure_reason
from room_315_pddl_validation_gate import validate_candidate_scenario
from room_315_pddl_validation_gate import write_validation_result


REPO_ROOT = SCRIPT_DIR.parents[1]
PDDL_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'config' / 'room_315_vla' / 'pddl'
PDDL_DOMAIN_PATH = PDDL_DIR / 'domain_room315.pddl'
DEFAULT_PLANSYS_GET_PLAN_SERVICE = '/planner/get_plan'
DEFAULT_PLANSYS_TIMEOUT_S = 10.0
DEFAULT_SUPERVISOR_NODE_NAME = 'room_315_vla_supervisor'

SUPPORTED_GOALS = {
    'right_yaskawa_to_staubli': {
        'side': 'right',
        'shuttle': 'right_shuttle',
        'source': 'yaskawa',
        'target': 'staubli',
        'problem_name': 'room315-right-yaskawa-to-staubli',
        'problem_file': 'problem_right_yaskawa_to_staubli.pddl',
    },
    'right_staubli_to_yaskawa': {
        'side': 'right',
        'shuttle': 'right_shuttle',
        'source': 'staubli',
        'target': 'yaskawa',
        'problem_name': 'room315-right-staubli-to-yaskawa',
        'problem_file': 'problem_right_staubli_to_yaskawa.pddl',
    },
    'left_yaskawa_to_kuka': {
        'side': 'left',
        'shuttle': 'left_shuttle',
        'source': 'yaskawa',
        'target': 'kuka',
        'problem_name': 'room315-left-yaskawa-to-kuka',
        'problem_file': 'problem_left_yaskawa_to_kuka.pddl',
    },
    'left_kuka_to_yaskawa': {
        'side': 'left',
        'shuttle': 'left_shuttle',
        'source': 'kuka',
        'target': 'yaskawa',
        'problem_name': 'room315-left-kuka-to-yaskawa',
        'problem_file': 'problem_left_kuka_to_yaskawa.pddl',
    },
    'right_loaded_r2_to_staubli': {
        'side': 'right',
        'shuttle': 'right_shuttle_2',
        'source': 'yaskawa',
        'target': 'staubli',
        'payload_condition': 'loaded',
        'problem_name': 'room315-right-loaded-r2-to-staubli',
        'problem_file': 'problem_right_loaded_r2_to_staubli.pddl',
    },
    'right_loaded_to_slot3': {
        'side': 'right',
        'target_slot': '3',
        'payload_condition': 'loaded',
        'selection_policy': 'nearest_loaded_to_target_slot_then_lowest_id',
        'loaded_shuttles': ('right_shuttle_1', 'right_shuttle_2'),
        'start_slots_by_shuttle': {
            'right_shuttle_1': '1',
            'right_shuttle_2': '2',
        },
        'problem_name': 'room315-right-loaded-to-slot3',
    },
    'right_loaded_to_slot3_clear_blocker': {
        'side': 'right',
        'target_slot': '3',
        'payload_condition': 'loaded',
        'selection_policy': 'nearest_loaded_to_target_slot_then_lowest_id',
        'loaded_shuttles': ('right_shuttle_2',),
        'start_slots_by_shuttle': {
            'right_shuttle_1': '3',
            'right_shuttle_2': '2',
        },
        'blocker_shuttle': 'right_shuttle_1',
        'blocker_start_slot': '3',
        'blocker_clear_slot': '1',
        'blocker_restore_slot': 'auto',
        'blocker_restore_policy': 'selected_source_slot_then_nearest_free_slot',
        'clearance_strategy': 'clear_blocker_move_loaded_then_restore_blocker_to_free_slot',
        'problem_name': 'room315-right-loaded-to-slot3-clear-blocker',
    },
    'right_empty_r1_to_yaskawa': {
        'side': 'right',
        'shuttle': 'right_shuttle_1',
        'source': 'staubli',
        'target': 'yaskawa',
        'payload_condition': 'empty',
        'problem_name': 'room315-right-empty-r1-to-yaskawa',
        'problem_file': 'problem_right_empty_r1_to_yaskawa.pddl',
    },
    'left_loaded_l2_to_kuka': {
        'side': 'left',
        'shuttle': 'left_shuttle_2',
        'source': 'yaskawa',
        'target': 'kuka',
        'payload_condition': 'loaded',
        'problem_name': 'room315-left-loaded-l2-to-kuka',
        'problem_file': 'problem_left_loaded_l2_to_kuka.pddl',
    },
}
LANGUAGE_TEMPLATE_SEQUENCE = (
    'move_from_to',
    'send_to_station',
    'route_between_stations',
    'bring_to_station',
)
SUPPORTED_SYMBOLIC_ACTIONS = {
    'prepare_switches',
    'open_stoppers',
    'move_shuttle',
    'stop_shuttle',
    'finish_task',
}
TARGET_SENSORS_BY_SIDE_AND_STATION = {
    ('right', 'yaskawa'): ('DZI1R', 'DZI2R'),
    ('right', 'staubli'): ('DZI3R', 'DZI4R'),
    ('left', 'yaskawa'): ('DZI1L', 'DZI2L'),
    ('left', 'kuka'): ('DZI3L', 'DZI4L'),
}
SLOT_STATION_BY_SIDE_AND_SLOT = {
    ('right', '1'): 'yaskawa',
    ('right', '2'): 'yaskawa',
    ('right', '3'): 'staubli',
    ('right', '4'): 'staubli',
    ('left', '1'): 'yaskawa',
    ('left', '2'): 'yaskawa',
    ('left', '3'): 'kuka',
    ('left', '4'): 'kuka',
}
SLOT_SENSOR_BY_SIDE_AND_SLOT = {
    ('right', '1'): 'DZI1R',
    ('right', '2'): 'DZI2R',
    ('right', '3'): 'DZI3R',
    ('right', '4'): 'DZI4R',
    ('left', '1'): 'DZI1L',
    ('left', '2'): 'DZI2L',
    ('left', '3'): 'DZI3L',
    ('left', '4'): 'DZI4L',
}


@dataclass(frozen=True)
class ScenarioSpec:
    goal_id: str
    side: str
    shuttle: str
    source: str
    target: str
    pddl_problem: str = ''
    pddl_goal: str = ''
    pddl_problem_file: str = ''
    payload_condition: str = ''
    target_slot: str = ''
    selection_policy: str = ''
    loaded_shuttles: tuple[str, ...] = ()
    start_slots_by_shuttle: tuple[tuple[str, str], ...] = ()
    selection_candidates: tuple[dict[str, Any], ...] = ()
    blocker_shuttle: str = ''
    blocker_start_slot: str = ''
    blocker_clear_slot: str = ''
    blocker_restore_slot: str = ''
    blocker_restore_policy: str = ''
    blocker_restore_slot_source: str = ''
    blocker_restore_candidate_slots: tuple[str, ...] = ()
    clearance_strategy: str = ''


class BasePlannerBackend:
    """Planner interface boundary for PlanSys2-backed symbolic planning."""

    def plan(self, goal_or_problem: ScenarioSpec, *, speed: float) -> list[str]:
        raise NotImplementedError


class PlanSys2GetPlanClient:
    """Small rclpy client for PlanSys2 planner/get_plan.

    The PlanSys2 planner service accepts raw PDDL domain and problem strings and
    returns a timed Plan. Keeping this adapter tiny makes it easy to mock in
    tests while production code still requires real PlanSys2 services.
    """

    def __init__(
        self,
        *,
        service_name: str = DEFAULT_PLANSYS_GET_PLAN_SERVICE,
        timeout_s: float = DEFAULT_PLANSYS_TIMEOUT_S,
        ros_args: list[str] | None = None,
    ) -> None:
        self.service_name = str(service_name or DEFAULT_PLANSYS_GET_PLAN_SERVICE)
        self.timeout_s = float(timeout_s)
        self._closed = False
        try:
            self.rclpy = importlib.import_module('rclpy')
            srv_module = importlib.import_module('plansys2_msgs.srv')
        except ModuleNotFoundError as exc:
            missing = str(exc).split("'")
            missing_name = missing[1] if len(missing) > 1 else str(exc)
            raise RuntimeError(
                'PlanSys2 is required for Room 315 PDDL planning, but '
                f'{missing_name!r} is not importable. Install the ROS 2 PlanSys2 '
                'packages for this distro and source the workspace, then start '
                'the PlanSys2 planner service. This pipeline no longer falls '
                'back to a deterministic local planner.'
            ) from exc
        self.GetPlan = getattr(srv_module, 'GetPlan')
        self._shutdown_on_close = not self.rclpy.ok()
        if self._shutdown_on_close:
            self.rclpy.init(args=ros_args)
        self.node = self.rclpy.create_node('room_315_plansys2_planner_client')
        self.client = self.node.create_client(self.GetPlan, self.service_name)
        if not self.client.wait_for_service(timeout_sec=self.timeout_s):
            self.close()
            raise RuntimeError(
                f'PlanSys2 planner service {self.service_name!r} is not available. '
                'Start the PlanSys2 planner node that exposes /planner/get_plan, '
                'or launch the Room 315 planning stack before running the PDDL '
                'scenario generator. No fallback planner is used.'
            )

    def get_plan(self, *, domain: str, problem: str) -> Any:
        request = self.GetPlan.Request()
        request.domain = str(domain)
        request.problem = str(problem)
        future = self.client.call_async(request)
        self.rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=self.timeout_s,
        )
        if not future.done():
            raise RuntimeError(
                f'timed out after {self.timeout_s:.1f}s waiting for PlanSys2 '
                f'planner service {self.service_name!r}'
            )
        response = future.result()
        if response is None:
            raise RuntimeError(
                f'PlanSys2 planner service {self.service_name!r} returned no response'
            )
        if not bool(getattr(response, 'success', False)):
            detail = str(getattr(response, 'error_info', '') or '').strip()
            if not detail:
                detail = 'no error_info provided by PlanSys2'
            raise RuntimeError(f'PlanSys2 failed to generate a plan: {detail}')
        return getattr(response, 'plan', None)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.node.destroy_node()
        finally:
            if self._shutdown_on_close:
                self.rclpy.try_shutdown()


class PlanSysPlannerBackend(BasePlannerBackend):
    """Plan Room 315 scenarios through a real PlanSys2 planner service."""

    def __init__(
        self,
        *,
        domain_path: Path | str = PDDL_DOMAIN_PATH,
        planner_client: Any | None = None,
        planner_service: str = DEFAULT_PLANSYS_GET_PLAN_SERVICE,
        timeout_s: float = DEFAULT_PLANSYS_TIMEOUT_S,
        ros_args: list[str] | None = None,
    ) -> None:
        self.domain_path = Path(domain_path).expanduser()
        self.planner_client = planner_client
        self.planner_service = planner_service
        self.timeout_s = float(timeout_s)
        self.ros_args = ros_args

    def plan(self, goal_or_problem: ScenarioSpec, *, speed: float) -> list[str]:
        spec = goal_or_problem
        domain_text = self._domain_text()
        problem_text = _problem_text_for_spec(spec)
        client = self.planner_client or PlanSys2GetPlanClient(
            service_name=self.planner_service,
            timeout_s=self.timeout_s,
            ros_args=self.ros_args,
        )
        close = getattr(client, 'close', None) if self.planner_client is None else None
        try:
            plan_msg = client.get_plan(domain=domain_text, problem=problem_text)
        finally:
            if close is not None:
                close()
        return _symbolic_plan_from_plansys_plan(plan_msg, spec=spec, speed=float(speed))

    def _domain_text(self) -> str:
        if not self.domain_path.exists():
            raise RuntimeError(f'Room 315 PDDL domain file is missing: {self.domain_path}')
        return self.domain_path.read_text(encoding='utf-8')


PlannerBackend = BasePlannerBackend


class ScenarioTransport:
    """Transport interface used by the execution loop and mocked in tests."""

    def wait_until_ready(self, *, timeout_s: float) -> dict[str, Any]:
        return {'ready': True, 'reason': ''}

    def publish_episode_control(self, command: str) -> None:
        raise NotImplementedError

    def wait_for_episode_started(self, *, goal: str, timeout_s: float) -> dict[str, Any]:
        return {'ready': True, 'reason': '', 'observed': False}

    def wait_for_episode_stopped(self, *, timeout_s: float) -> dict[str, Any]:
        return {'ready': True, 'reason': '', 'observed': False}

    def publish_command(self, command: dict[str, Any]) -> None:
        raise NotImplementedError

    def supervisor_decision_count(self) -> int:
        return 0

    def wait_for_supervisor_decision(
        self,
        *,
        previous_count: int,
        timeout_s: float,
    ) -> dict[str, Any] | None:
        raise NotImplementedError

    def wait_for_target_arrival(
        self,
        *,
        side: str,
        target_sensors: list[str],
        shuttle: str,
        timeout_s: float,
    ) -> dict[str, Any]:
        return {'arrived': True, 'reason': ''}


class RosScenarioTransport(ScenarioTransport):
    """ROS 2 execution transport loaded only for supervisor command publishing."""

    def __init__(
        self,
        *,
        command_topic: str,
        episode_control_topic: str,
        status_topic: str,
        dataset_status_topic: str,
        ros_args: list[str] | None = None,
    ) -> None:
        self.command_topic = command_topic
        self.episode_control_topic = episode_control_topic
        self.status_topic = status_topic
        self.dataset_status_topic = dataset_status_topic
        self.rclpy = importlib.import_module('rclpy')
        std_msgs = importlib.import_module('std_msgs.msg')
        self.String = std_msgs.String
        self.rclpy.init(args=ros_args)
        self.node = self.rclpy.create_node('room_315_pddl_scenario_generator')
        self.command_pub = self.node.create_publisher(self.String, command_topic, 10)
        self.episode_control_pub = self.node.create_publisher(
            self.String,
            episode_control_topic,
            10,
        )
        self.latest_status: dict[str, Any] = {}
        self.latest_dataset_status: dict[str, Any] = {}
        self.last_dataset_dir = ''
        self.last_episode_id = ''
        self.node.create_subscription(
            self.String,
            status_topic,
            self._on_status,
            10,
        )
        self.node.create_subscription(
            self.String,
            dataset_status_topic,
            self._on_dataset_status,
            10,
        )

    def wait_until_ready(self, *, timeout_s: float) -> dict[str, Any]:
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        command_subscribers = 0
        supervisor_command_subscribers: int | None = None
        status_seen = False
        while time.monotonic() <= deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            command_subscribers = int(self.command_pub.get_subscription_count())
            supervisor_command_subscribers = self._supervisor_command_subscription_count()
            status_seen = bool(self.latest_status)
            if supervisor_command_subscribers is None:
                command_path_ready = command_subscribers > 0
            else:
                command_path_ready = supervisor_command_subscribers > 0
            if command_path_ready and status_seen:
                return {'ready': True, 'reason': ''}

        missing = []
        if supervisor_command_subscribers is None:
            if command_subscribers <= 0:
                missing.append(f'no subscriber on {self.command_topic}')
        elif supervisor_command_subscribers <= 0:
            missing.append(
                f'no {DEFAULT_SUPERVISOR_NODE_NAME} subscriber on {self.command_topic} '
                f'({command_subscribers} total subscriber(s) discovered)'
            )
        if not status_seen:
            missing.append(f'no supervisor status received on {self.status_topic}')
        return {
            'ready': False,
            'reason': (
                'Room 315 VLA supervisor is not ready: '
                + '; '.join(missing or ['readiness check timed out'])
            ),
        }

    def wait_for_initial_scenario_state(
        self,
        *,
        scenario: dict[str, Any],
        timeout_s: float,
    ) -> dict[str, Any]:
        side = _side_from_scenario(scenario)
        target_shuttle = _target_shuttle_entity_from_scenario(scenario, side)
        payload_condition = _payload_condition_from_scenario(scenario)
        if not side or not target_shuttle:
            return {'ready': True, 'reason': '', 'observed': False}

        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        reason = ''
        while time.monotonic() <= deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            status = self.latest_status if isinstance(self.latest_status, dict) else {}
            ready, reason = _scenario_initial_state_ready(
                status=status,
                side=side,
                target_shuttle=target_shuttle,
                payload_condition=payload_condition,
            )
            if ready:
                ready, reason = _scenario_payload_state_ready(
                    status=status,
                    side=side,
                    scenario=scenario,
                )
            if ready:
                return {
                    'ready': True,
                    'reason': '',
                    'observed': True,
                    'side': side,
                    'target_shuttle': target_shuttle,
                    'payload_condition': payload_condition,
                }

        return {
            'ready': False,
            'reason': reason or 'initial scenario state was not observed',
            'side': side,
            'target_shuttle': target_shuttle,
            'payload_condition': payload_condition,
        }

    def _supervisor_command_subscription_count(self) -> int | None:
        endpoint_info = getattr(self.node, 'get_subscriptions_info_by_topic', None)
        if not callable(endpoint_info):
            return None
        try:
            infos = endpoint_info(self.command_topic)
        except Exception:
            return None
        count = 0
        for info in infos:
            node_name = str(getattr(info, 'node_name', '') or '').strip().lstrip('/')
            if node_name == DEFAULT_SUPERVISOR_NODE_NAME:
                count += 1
        return count

    def _on_status(self, msg: Any) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(parsed, dict):
            self.latest_status = parsed

    def _on_dataset_status(self, msg: Any) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(parsed, dict):
            self.latest_dataset_status = parsed
            dataset_dir = str(parsed.get('dataset_dir') or '').strip()
            episode_id = str(parsed.get('episode_id') or '').strip()
            if dataset_dir:
                self.last_dataset_dir = dataset_dir
            if episode_id:
                self.last_episode_id = episode_id

    def publish_episode_control(self, command: str) -> None:
        msg = self.String()
        msg.data = str(command)
        self.episode_control_pub.publish(msg)

    def wait_for_episode_started(self, *, goal: str, timeout_s: float) -> dict[str, Any]:
        if not self._dataset_status_publisher_count():
            return {'ready': True, 'reason': '', 'observed': False}
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        wanted_goal = str(goal or '').strip()
        latest: dict[str, Any] = {}
        while time.monotonic() <= deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            latest = dict(self.latest_dataset_status)
            if not bool(latest.get('active', False)):
                continue
            if wanted_goal and str(latest.get('task') or '').strip() != wanted_goal:
                continue
            if str(latest.get('episode_id') or '').strip():
                return {'ready': True, 'reason': '', 'observed': True}
        return {
            'ready': False,
            'reason': (
                'dataset recorder did not acknowledge episode start'
                + (f' for {wanted_goal!r}' if wanted_goal else '')
            ),
            'latest_dataset_status': latest,
            'observed': True,
        }

    def wait_for_episode_stopped(self, *, timeout_s: float) -> dict[str, Any]:
        if not self._dataset_status_publisher_count():
            return {'ready': True, 'reason': '', 'observed': False}
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        latest: dict[str, Any] = {}
        while time.monotonic() <= deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            latest = dict(self.latest_dataset_status)
            if latest and not bool(latest.get('active', False)):
                return {'ready': True, 'reason': '', 'observed': True}
        return {
            'ready': False,
            'reason': 'dataset recorder did not acknowledge episode stop',
            'latest_dataset_status': latest,
            'observed': True,
        }

    def _dataset_status_publisher_count(self) -> int | None:
        endpoint_info = getattr(self.node, 'get_publishers_info_by_topic', None)
        if not callable(endpoint_info):
            return None
        try:
            return len(endpoint_info(self.dataset_status_topic))
        except Exception:
            return None

    def publish_command(self, command: dict[str, Any]) -> None:
        msg = self.String()
        msg.data = _json_dumps(command)
        self.command_pub.publish(msg)

    def supervisor_decision_count(self) -> int:
        metrics = _safety_metrics_from_status(self.latest_status)
        return int(metrics.get('total_proposed_actions') or 0)

    def wait_for_supervisor_decision(
        self,
        *,
        previous_count: int,
        timeout_s: float,
    ) -> dict[str, Any] | None:
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        while time.monotonic() <= deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.supervisor_decision_count() > int(previous_count):
                safety = self.latest_status.get('safety_decoder', {})
                if isinstance(safety, dict) and isinstance(safety.get('last_decision'), dict):
                    return dict(safety['last_decision'])
                return {'accepted': True, 'reason': 'supervisor decision observed'}
        return None

    def wait_for_target_arrival(
        self,
        *,
        side: str,
        target_sensors: list[str],
        shuttle: str,
        timeout_s: float,
    ) -> dict[str, Any]:
        wanted = {
            str(sensor or '').strip().upper()
            for sensor in target_sensors
            if str(sensor or '').strip()
        }
        if not wanted:
            return {
                'arrived': False,
                'reason': 'no target sensors were configured for this PDDL move_shuttle step',
            }

        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        active: set[str] = set()
        while time.monotonic() <= deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            active = _active_sensor_names_from_status(self.latest_status, side)
            matched = sorted(active & wanted)
            if matched:
                return {
                    'arrived': True,
                    'reason': '',
                    'matched_sensors': matched,
                    'target_sensors': sorted(wanted),
                    'side': side,
                    'shuttle': shuttle,
                }

        return {
            'arrived': False,
            'reason': (
                f'timeout waiting for {shuttle or "shuttle"} on {side} target '
                f'sensor(s): {", ".join(sorted(wanted))}'
            ),
            'active_sensors': sorted(active),
            'target_sensors': sorted(wanted),
            'side': side,
            'shuttle': shuttle,
        }

    def verify_final_goal(self, *, scenario: dict[str, Any], timeout_s: float) -> dict[str, Any]:
        side = _side_from_scenario(scenario)
        target = _target_station_from_scenario(scenario)
        wanted = set(_target_sensors_for_scenario(scenario, side=side, target_station=target))
        if not side or not target or not wanted:
            return {
                'known': False,
                'satisfied': False,
                'reason': 'final goal target station/sensor could not be inferred',
                'side': side,
                'target_station': target,
                'target_slot': _target_slot_from_scenario(scenario),
                'target_sensors': sorted(wanted),
            }

        target_shuttle = _target_shuttle_entity_from_scenario(scenario, side)
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        active: set[str] = set()
        identity_mismatch = ''
        while time.monotonic() <= deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            active = _active_sensor_names_from_status(self.latest_status, side)
            matched = sorted(active & wanted)
            if matched:
                matched_readings = _active_sensor_readings_for_names(
                    self.latest_status,
                    side,
                    set(matched),
                )
                if target_shuttle:
                    reading_shuttles = {
                        str(reading.get('shuttle') or '').strip()
                        for reading in matched_readings
                        if str(reading.get('shuttle') or '').strip()
                    }
                    if reading_shuttles and target_shuttle not in reading_shuttles:
                        identity_mismatch = (
                            f'final goal sensor active for {sorted(reading_shuttles)}, '
                            f'expected {target_shuttle}'
                        )
                        continue
                return {
                    'known': True,
                    'satisfied': True,
                    'reason': '',
                    'side': side,
                    'target_station': target,
                    'target_slot': _target_slot_from_scenario(scenario),
                    'target_sensors': sorted(wanted),
                    'matched_sensors': matched,
                    'target_shuttle': target_shuttle,
                }
        return {
            'known': True,
            'satisfied': False,
            'reason': identity_mismatch or (
                f'final goal not satisfied: no {side} target sensor active for '
                f'{target}; expected {", ".join(sorted(wanted))}'
            ),
            'side': side,
            'target_station': target,
            'target_slot': _target_slot_from_scenario(scenario),
            'target_sensors': sorted(wanted),
            'active_sensors': sorted(active),
        }

    def validation_episode_ref(self) -> dict[str, str]:
        return {
            'dataset_dir': self.last_dataset_dir,
            'episode_id': self.last_episode_id,
        }

    def shutdown(self) -> None:
        self.node.destroy_node()
        self.rclpy.try_shutdown()


def generate_scenario(
    *,
    goal: str = '',
    problem: Path | str | None = None,
    language_seed: int | None = None,
    language_template_id: str = '',
    speed: float = 0.3,
    planner: BasePlannerBackend | None = None,
) -> dict[str, Any]:
    """Build a dry-run planned episode structure."""

    spec = scenario_spec_from_inputs(goal=goal, problem=problem)
    if spec.blocker_shuttle:
        plan = _blocker_clear_symbolic_plan_for_spec(spec, speed=float(speed))
    else:
        planner = planner or PlanSysPlannerBackend()
        plan = planner.plan(spec, speed=float(speed))

    translated = translate_plan(plan)
    default_template_id = _default_language_template_id_for_spec(spec)
    generated = generate_language(
        pddl_goal=spec.pddl_goal,
        pddl_problem=spec.pddl_problem,
        symbolic_plan=plan,
        action_sequence=_language_action_sequence_for_spec(spec),
        template_id=language_template_id or default_template_id,
        seed=language_seed,
    )
    scenario = {
        'scenario_id': spec.goal_id,
        'pddl_problem': spec.pddl_problem,
        'pddl_goal': spec.pddl_goal,
        'target_shuttle_id': spec.shuttle,
        'payload_condition': spec.payload_condition,
        'payload_state': _payload_state_for_spec(spec),
        'language': generated.language,
        'generated_language_template_id': generated.template_id,
        'planning_source': 'pddl',
        'planner_backend': 'plansys',
        'pddl_domain': str(PDDL_DOMAIN_PATH.name),
        'symbolic_plan': plan,
        'primitive_commands': [step.command for step in translated],
        'expected_event_targets': [step.event_action for step in translated],
        'action_vectors': [step.action_vector for step in translated],
    }
    if spec.target_slot:
        scenario['target_slot'] = spec.target_slot
    if spec.selection_policy:
        scenario['selection_policy'] = spec.selection_policy
        scenario['selection_candidates'] = [dict(item) for item in spec.selection_candidates]
    if spec.blocker_shuttle:
        scenario['blocker_clearance'] = _blocker_clearance_metadata_for_spec(spec)
        scenario['plan_step_metadata'] = _blocker_clear_plan_step_metadata(spec)
    scenario['validation'] = build_validation_result(scenario, execution_result=None)
    return scenario


def execute_scenario(
    scenario: dict[str, Any],
    transport: ScenarioTransport,
    *,
    command_timeout_s: float = 5.0,
    arrival_timeout_s: float = 120.0,
) -> dict[str, Any]:
    """Execute a generated scenario through the supervisor transport."""

    static_validation = validate_candidate_scenario(scenario)
    if not static_validation['valid']:
        result = {
            'success': False,
            'failed_step_index': None,
            'failure_reason': static_validation['failure_reasons'][0],
            'published_commands': [],
            'executed_command_count': 0,
        }
        return _finalize_execution_result(scenario, transport, result)

    readiness_check = getattr(transport, 'wait_until_ready', None)
    if callable(readiness_check):
        readiness = readiness_check(timeout_s=command_timeout_s)
        if not bool(readiness.get('ready', True)):
            return _finalize_execution_result(scenario, transport, {
                'success': False,
                'failed_step_index': None,
                'failure_reason': str(readiness.get('reason') or 'supervisor is not ready'),
                'published_commands': [],
                'executed_command_count': 0,
            })

    initial_state_check = getattr(transport, 'wait_for_initial_scenario_state', None)
    if callable(initial_state_check):
        initial_state = initial_state_check(
            scenario=scenario,
            timeout_s=command_timeout_s,
        )
        if not bool(initial_state.get('ready', True)):
            return _finalize_execution_result(scenario, transport, {
                'success': False,
                'failed_step_index': None,
                'failure_reason': str(
                    initial_state.get('reason')
                    or 'initial scenario state is not ready'
                ),
                'initial_state_wait': initial_state,
                'published_commands': [],
                'executed_command_count': 0,
            })

    language = str(scenario.get('language') or scenario.get('scenario_id') or 'Room 315 PDDL task')
    transport.publish_episode_control(f'start {language}')
    episode_started = _wait_for_episode_started(
        transport,
        goal=language,
        timeout_s=command_timeout_s,
    )
    if not bool(episode_started.get('ready', True)):
        return _finalize_execution_result(scenario, transport, {
            'success': False,
            'failed_step_index': None,
            'failure_reason': str(
                episode_started.get('reason')
                or 'dataset recorder did not acknowledge episode start'
            ),
            'dataset_recorder_wait': episode_started,
            'published_commands': [],
            'executed_command_count': 0,
        })

    published_commands: list[dict[str, Any]] = []
    for step_index, payload in enumerate(command_payloads_for_execution(scenario)):
        previous_count = transport.supervisor_decision_count()
        transport.publish_command(payload)
        published_commands.append(payload)
        decision = transport.wait_for_supervisor_decision(
            previous_count=previous_count,
            timeout_s=command_timeout_s,
        )
        if decision is None:
            reason = f'timeout waiting for supervisor decision at plan step {step_index}'
            _publish_episode_stop_and_wait(
                transport,
                'stop failure',
                timeout_s=command_timeout_s,
            )
            return _finalize_execution_result(scenario, transport, {
                'success': False,
                'failed_step_index': step_index,
                'failure_reason': reason,
                'published_commands': published_commands,
                'executed_command_count': len(published_commands),
            })
        if not bool(decision.get('accepted', False)):
            reason = str(decision.get('reason') or 'supervisor rejected command')
            _publish_episode_stop_and_wait(
                transport,
                'stop failure',
                timeout_s=command_timeout_s,
            )
            return _finalize_execution_result(scenario, transport, {
                'success': False,
                'failed_step_index': step_index,
                'failure_reason': reason,
                'supervisor_decision': decision,
                'published_commands': published_commands,
                'executed_command_count': len(published_commands),
            })

        runtime_failure = _runtime_failure_from_transport(transport)
        if runtime_failure:
            _publish_episode_stop_and_wait(
                transport,
                'stop failure',
                timeout_s=command_timeout_s,
            )
            return _finalize_execution_result(scenario, transport, {
                'success': False,
                'failed_step_index': step_index,
                'failure_reason': runtime_failure,
                'published_commands': published_commands,
                'executed_command_count': len(published_commands),
            })

        arrival_wait = _target_arrival_wait_for_payload(payload, scenario)
        wait_for_arrival = getattr(transport, 'wait_for_target_arrival', None)
        if arrival_wait is not None and callable(wait_for_arrival):
            arrival_result = wait_for_arrival(
                side=arrival_wait['side'],
                target_sensors=arrival_wait['target_sensors'],
                shuttle=arrival_wait['shuttle'],
                timeout_s=arrival_timeout_s,
            )
            if not bool(arrival_result.get('arrived', False)):
                reason = str(arrival_result.get('reason') or 'target arrival wait failed')
                _publish_episode_stop_and_wait(
                    transport,
                    'stop failure',
                    timeout_s=command_timeout_s,
                )
                return _finalize_execution_result(scenario, transport, {
                    'success': False,
                    'failed_step_index': step_index,
                    'failure_reason': reason,
                    'arrival_wait': arrival_result,
                    'arrival_timeout': 'timeout' in reason.casefold(),
                    'published_commands': published_commands,
                    'executed_command_count': len(published_commands),
                })

    final_goal_verification = None
    verify_final_goal = getattr(transport, 'verify_final_goal', None)
    if callable(verify_final_goal):
        final_goal_verification = verify_final_goal(
            scenario=scenario,
            timeout_s=command_timeout_s,
        )
        if not bool(final_goal_verification.get('satisfied', False)):
            reason = str(final_goal_verification.get('reason') or 'final goal was not satisfied')
            _publish_episode_stop_and_wait(
                transport,
                'stop failure',
                timeout_s=command_timeout_s,
            )
            return _finalize_execution_result(scenario, transport, {
                'success': False,
                'failed_step_index': None,
                'failure_reason': reason,
                'final_goal_satisfied': False,
                'final_goal_verification': final_goal_verification,
                'published_commands': published_commands,
                'executed_command_count': len(published_commands),
            })

    episode_stopped = _publish_episode_stop_and_wait(
        transport,
        'stop success',
        timeout_s=command_timeout_s,
    )
    return _finalize_execution_result(scenario, transport, {
        'success': True,
        'failed_step_index': None,
        'failure_reason': '',
        'task_success': True,
        'final_goal_satisfied': (
            bool(final_goal_verification.get('satisfied', False))
            if isinstance(final_goal_verification, dict)
            else None
        ),
        'final_goal_verification': final_goal_verification,
        'dataset_recorder_stop_wait': episode_stopped,
        'published_commands': published_commands,
        'executed_command_count': len(published_commands),
    })


def preflight_scenario(
    scenario: dict[str, Any],
    transport: ScenarioTransport,
    *,
    command_timeout_s: float = 5.0,
) -> dict[str, Any]:
    """Check that the supervisor and initial shuttle/payload state are ready."""

    readiness_check = getattr(transport, 'wait_until_ready', None)
    if callable(readiness_check):
        readiness = readiness_check(timeout_s=command_timeout_s)
        if not bool(readiness.get('ready', True)):
            return {
                'ready': False,
                'reason': str(readiness.get('reason') or 'supervisor is not ready'),
                'supervisor_ready': readiness,
            }

    initial_state_check = getattr(transport, 'wait_for_initial_scenario_state', None)
    if callable(initial_state_check):
        initial_state = initial_state_check(
            scenario=scenario,
            timeout_s=command_timeout_s,
        )
        if not bool(initial_state.get('ready', True)):
            return {
                'ready': False,
                'reason': str(
                    initial_state.get('reason')
                    or 'initial scenario state is not ready'
                ),
                'initial_state_wait': initial_state,
            }
        return {
            'ready': True,
            'reason': '',
            'initial_state_wait': initial_state,
        }

    return {'ready': True, 'reason': '', 'observed': False}


def _preflight_ready_line(scenario: dict[str, Any]) -> str:
    preflight = scenario.get('preflight')
    if not isinstance(preflight, dict):
        return 'NOT READY: preflight result is missing'
    if not bool(preflight.get('ready', False)):
        return f"NOT READY: {preflight.get('reason') or 'preflight failed'}"

    initial_state = preflight.get('initial_state_wait')
    if not isinstance(initial_state, dict):
        return 'READY'
    side = str(initial_state.get('side') or '').strip()
    target_shuttle = str(initial_state.get('target_shuttle') or '').strip()
    payload_condition = str(initial_state.get('payload_condition') or '').strip()
    suffix = ' '.join(
        token
        for token in (target_shuttle, payload_condition, f'on {side} rail' if side else '')
        if token
    )
    return f'READY {suffix}'.strip()


def _wait_for_episode_started(
    transport: ScenarioTransport,
    *,
    goal: str,
    timeout_s: float,
) -> dict[str, Any]:
    wait_for_start = getattr(transport, 'wait_for_episode_started', None)
    if callable(wait_for_start):
        return wait_for_start(goal=goal, timeout_s=timeout_s)
    return {'ready': True, 'reason': '', 'observed': False}


def _publish_episode_stop_and_wait(
    transport: ScenarioTransport,
    command: str,
    *,
    timeout_s: float,
) -> dict[str, Any]:
    transport.publish_episode_control(command)
    wait_for_stop = getattr(transport, 'wait_for_episode_stopped', None)
    if callable(wait_for_stop):
        return wait_for_stop(timeout_s=timeout_s)
    return {'ready': True, 'reason': '', 'observed': False}


def command_payloads_for_execution(scenario: dict[str, Any]) -> list[dict[str, Any]]:
    commands = list(scenario.get('primitive_commands') or [])
    action_vectors = list(scenario.get('action_vectors') or [])
    symbolic_plan = list(scenario.get('symbolic_plan') or [])
    payloads = []
    for index, command in enumerate(commands):
        action_vector = action_vectors[index] if index < len(action_vectors) else None
        metadata = _planning_metadata_for_step(scenario, index)
        payload = dict(command)
        if action_vector is not None:
            payload['action_vector'] = action_vector
        payload.update(metadata)
        if index < len(symbolic_plan):
            payload.setdefault('symbolic_step', symbolic_plan[index])
        payloads.append(payload)
    return payloads


def _target_arrival_wait_for_payload(
    payload: dict[str, Any],
    scenario: dict[str, Any],
) -> dict[str, Any] | None:
    if str(payload.get('action') or '').casefold() != 'shuttle':
        return None
    if str(payload.get('command') or '').casefold() != 'on':
        return None

    symbolic_step = str(payload.get('symbolic_step') or '').strip()
    if not symbolic_step:
        return None
    try:
        parsed_steps = parse_plan_text(symbolic_step)
    except ValueError:
        return None
    if not parsed_steps or parsed_steps[0].name != 'move_shuttle':
        return None

    step = parsed_steps[0]
    side = _side_from_symbols(step.args, default=str(payload.get('side') or 'right'))
    target = _target_station_from_move_step(step, scenario)
    target_slot = _slot_symbol_or_empty(payload.get('target_slot')) or _target_slot_from_scenario(scenario)
    sensors = _target_sensors_for_scenario(
        scenario,
        side=side,
        target_station=target,
        target_slot=target_slot,
    )
    return {
        'side': side,
        'target_station': target,
        'target_slot': target_slot,
        'target_sensors': sensors,
        'shuttle': str(payload.get('shuttle') or ''),
    }


def _target_station_from_move_step(step: Any, scenario: dict[str, Any]) -> str:
    stations = _stations_from_symbols(step.args)
    if len(stations) >= 2:
        return stations[-1]
    goal_text = str(scenario.get('pddl_goal') or '')
    match = re.search(r'\bat\s+([A-Za-z0-9_-]+)\b', goal_text, re.IGNORECASE)
    if match:
        try:
            return _station_symbol(match.group(1))
        except ValueError:
            pass
    return ''


def _target_sensors_for_scenario(
    scenario: dict[str, Any],
    *,
    side: str,
    target_station: str,
    target_slot: str = '',
) -> list[str]:
    target_slot = _slot_symbol_or_empty(target_slot) or _target_slot_from_scenario(scenario)
    if target_slot:
        slot_sensor = SLOT_SENSOR_BY_SIDE_AND_SLOT.get((side, target_slot), '')
        if slot_sensor:
            return [slot_sensor]
    return list(TARGET_SENSORS_BY_SIDE_AND_STATION.get((side, target_station), ()))


def _target_slot_from_scenario(scenario: dict[str, Any]) -> str:
    return _slot_symbol_or_empty(scenario.get('target_slot'))


def _default_language_template_id_for_spec(spec: ScenarioSpec) -> str:
    if not spec.target_slot:
        return ''
    if spec.payload_condition == 'loaded':
        return 'loaded_shuttle_to_slot'
    if spec.payload_condition == 'empty':
        return 'empty_shuttle_to_slot'
    return 'move_to_slot'


def _language_action_sequence_for_spec(spec: ScenarioSpec) -> str:
    if not spec.target_slot:
        return ''
    condition = f'{spec.payload_condition} ' if spec.payload_condition else ''
    return f'move the {condition}{spec.side} shuttle to slot {spec.target_slot}'


def _blocker_clear_symbolic_plan_for_spec(spec: ScenarioSpec, *, speed: float) -> list[str]:
    if not spec.blocker_shuttle:
        return []
    blocker_source = _station_for_slot(spec.side, spec.blocker_start_slot)
    blocker_target = _station_for_slot(spec.side, spec.blocker_clear_slot)
    selected_source = spec.source
    selected_target = spec.target
    speed_text = f'{float(speed):.4g}'
    plan = [
        f'prepare_switches {spec.side} {blocker_source} {blocker_target}',
        f'open_stoppers {spec.side} {blocker_source} {blocker_target}',
        (
            f'move_shuttle {spec.side} {spec.blocker_shuttle} '
            f'{blocker_source} {blocker_target} speed={speed_text}'
        ),
        f'stop_shuttle {spec.side} {spec.blocker_shuttle}',
        f'prepare_switches {spec.side} {selected_source} {selected_target}',
        f'open_stoppers {spec.side} {selected_source} {selected_target}',
        (
            f'move_shuttle {spec.side} {spec.shuttle} '
            f'{selected_source} {selected_target} speed={speed_text}'
        ),
        f'stop_shuttle {spec.side} {spec.shuttle}',
    ]
    if spec.blocker_restore_slot:
        restore_source = _station_for_slot(spec.side, spec.blocker_clear_slot)
        restore_target = _station_for_slot(spec.side, spec.blocker_restore_slot)
        plan.extend([
            f'prepare_switches {spec.side} {restore_source} {restore_target}',
            f'open_stoppers {spec.side} {restore_source} {restore_target}',
            (
                f'move_shuttle {spec.side} {spec.blocker_shuttle} '
                f'{restore_source} {restore_target} speed={speed_text}'
            ),
            f'stop_shuttle {spec.side} {spec.blocker_shuttle}',
        ])
    plan.append(f'finish_task {spec.shuttle} {selected_target}')
    return plan


def _blocker_clearance_metadata_for_spec(spec: ScenarioSpec) -> dict[str, Any]:
    return {
        'strategy': spec.clearance_strategy,
        'phase': (
            'clear_blocker_move_selected_restore_blocker'
            if spec.blocker_restore_slot
            else 'clear_blocker_then_move_selected'
        ),
        'blocker_shuttle_id': spec.blocker_shuttle,
        'blocker_start_slot': spec.blocker_start_slot,
        'blocker_clear_slot': spec.blocker_clear_slot,
        'blocker_restore_slot': spec.blocker_restore_slot,
        'blocker_final_slot': spec.blocker_restore_slot or spec.blocker_clear_slot,
        'blocker_restore_policy': spec.blocker_restore_policy,
        'blocker_restore_slot_source': spec.blocker_restore_slot_source,
        'blocker_restore_candidate_slots': list(spec.blocker_restore_candidate_slots),
        'selected_shuttle_id': spec.shuttle,
        'selected_target_slot': spec.target_slot,
        'restore_deferred': not bool(spec.blocker_restore_slot),
        'model_input_exposure': 'excluded',
    }


def _blocker_clear_plan_step_metadata(spec: ScenarioSpec) -> list[dict[str, Any]]:
    blocker_route = {
        'coordination_phase': 'clear_blocker',
        'plan_step_target_shuttle_id': spec.blocker_shuttle,
        'blocker_shuttle_id': spec.blocker_shuttle,
        'target_slot': spec.blocker_clear_slot,
        'target_station': _station_for_slot(spec.side, spec.blocker_clear_slot),
        'clearance_strategy': spec.clearance_strategy,
    }
    selected_route = {
        'coordination_phase': 'move_selected_loaded',
        'plan_step_target_shuttle_id': spec.shuttle,
        'selected_shuttle_id': spec.shuttle,
        'target_slot': spec.target_slot,
        'target_station': spec.target,
        'clearance_strategy': spec.clearance_strategy,
    }
    steps = [
        dict(blocker_route),
        dict(blocker_route),
        dict(blocker_route),
        dict(blocker_route),
        dict(selected_route),
        dict(selected_route),
        dict(selected_route),
        dict(selected_route),
    ]
    if spec.blocker_restore_slot:
        restore_route = {
            'coordination_phase': 'restore_blocker',
            'plan_step_target_shuttle_id': spec.blocker_shuttle,
            'blocker_shuttle_id': spec.blocker_shuttle,
            'target_slot': spec.blocker_restore_slot,
            'target_station': _station_for_slot(spec.side, spec.blocker_restore_slot),
            'clearance_strategy': spec.clearance_strategy,
        }
        steps.extend([
            dict(restore_route),
            dict(restore_route),
            dict(restore_route),
            dict(restore_route),
        ])
    steps.append({
        'coordination_phase': 'complete_selected_loaded_task',
        'plan_step_target_shuttle_id': spec.shuttle,
        'selected_shuttle_id': spec.shuttle,
        'target_slot': spec.target_slot,
        'target_station': spec.target,
        'clearance_strategy': spec.clearance_strategy,
    })
    return steps


def load_batch_config(path: Path | str) -> dict[str, Any]:
    config_path = Path(path).expanduser()
    loaded = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f'batch config {config_path} must contain a YAML mapping')
    loaded.setdefault('batch_config_path', str(config_path))
    return loaded


def generate_batch_scenarios(
    config: dict[str, Any],
    *,
    language_seed: int | None = None,
    language_template_id: str = '',
    planner: BasePlannerBackend | None = None,
) -> dict[str, Any]:
    """Generate a deterministic batch of planned Room 315 episodes."""

    normalized = _normalize_batch_config(config)
    if language_seed is not None:
        normalized['language_seed'] = int(language_seed)
    goals = list(normalized['goals'])
    repetitions = int(normalized['repetitions_per_goal'])
    speed_values = list(normalized['speed_values'])
    base_seed = int(normalized['language_seed'])
    episodes = []
    unshuffled_index = 0
    for goal_index, goal in enumerate(goals):
        for repetition_index in range(repetitions):
            speed = float(speed_values[repetition_index % len(speed_values)])
            template_id = language_template_id or _batch_language_template_id(
                base_seed=base_seed,
                goal_index=goal_index,
                repetition_index=repetition_index,
            )
            scenario = generate_scenario(
                goal=goal,
                language_seed=base_seed + unshuffled_index,
                language_template_id=template_id,
                speed=speed,
                planner=planner or PlanSysPlannerBackend(),
            )
            scenario.update({
                'batch_id': normalized['batch_id'],
                'batch_index': unshuffled_index,
                'batch_goal_index': goal_index,
                'batch_repetition_index': repetition_index,
                'batch_speed_mps': speed,
                'output_dataset_dir': normalized['output_dataset_dir'],
            })
            episodes.append(scenario)
            unshuffled_index += 1

    if normalized['shuffle']:
        random.Random(base_seed).shuffle(episodes)
        for batch_index, scenario in enumerate(episodes):
            scenario['batch_index'] = batch_index

    return {
        'batch_id': normalized['batch_id'],
        'batch_config_path': normalized['batch_config_path'],
        'goals': goals,
        'optional_later_goals': list(normalized['optional_later_goals']),
        'repetitions_per_goal': repetitions,
        'language_seed': base_seed,
        'speed_values': speed_values,
        'output_dataset_dir': normalized['output_dataset_dir'],
        'dry_run': normalized['dry_run'],
        'execute': normalized['execute'],
        'shuffle': normalized['shuffle'],
        'planned_episode_count': len(episodes),
        'episodes': episodes,
    }


def execute_batch_scenarios(
    batch: dict[str, Any],
    transport: ScenarioTransport,
    *,
    command_timeout_s: float = 5.0,
    arrival_timeout_s: float = 120.0,
) -> dict[str, Any]:
    """Execute planned episodes through the same supervisor command path."""

    results = []
    for scenario in list(batch.get('episodes') or []):
        result = execute_scenario(
            scenario,
            transport,
            command_timeout_s=command_timeout_s,
            arrival_timeout_s=arrival_timeout_s,
        )
        result['scenario_id'] = scenario.get('scenario_id', '')
        result['batch_index'] = scenario.get('batch_index')
        results.append(result)
        if not result.get('success'):
            break
    completed = len(results)
    successes = sum(1 for result in results if result.get('success'))
    return {
        'success': completed == len(batch.get('episodes') or []) and successes == completed,
        'completed_episode_count': completed,
        'successes': successes,
        'failures': completed - successes,
        'results': results,
    }


def create_planner_backend(
    backend_name: str = 'plansys',
    *,
    planner_service: str = DEFAULT_PLANSYS_GET_PLAN_SERVICE,
    planner_timeout_s: float = DEFAULT_PLANSYS_TIMEOUT_S,
    ros_args: list[str] | None = None,
) -> BasePlannerBackend:
    name = str(backend_name or 'plansys').strip().casefold().replace('_', '-')
    if name in {'plansys', 'plansys2'}:
        return PlanSysPlannerBackend(
            planner_service=planner_service,
            timeout_s=planner_timeout_s,
            ros_args=ros_args,
        )
    if name in {'fallback', 'deterministic', 'deterministic-fallback'}:
        raise ValueError(
            'fallback planner backend is no longer supported. Room 315 PDDL '
            'scenario generation requires PlanSys2; use --planner-backend plansys.'
        )
    if name in {'external-pddl', 'external'}:
        raise ValueError(
            'external-pddl planner backend is no longer supported. Room 315 PDDL '
            'scenario generation requires PlanSys2; use --planner-backend plansys.'
        )
    raise ValueError(
        f'unknown planner backend {backend_name!r}; allowed: plansys'
    )


def _symbolic_plan_from_plansys_plan(
    plan_msg: Any,
    *,
    spec: ScenarioSpec,
    speed: float,
) -> list[str]:
    plan = [
        _canonical_symbolic_step(step, spec=spec, speed=speed)
        for step in _plansys_action_strings(plan_msg)
    ]
    plan = [step for step in plan if step]
    if not plan:
        raise RuntimeError(
            'PlanSys2 generated no supported Room 315 symbolic plan steps. '
            'Check that the Room 315 domain/problem use the expected actions: '
            f'{", ".join(sorted(SUPPORTED_SYMBOLIC_ACTIONS))}.'
        )
    return plan


def _plansys_action_strings(plan_msg: Any) -> list[str]:
    if plan_msg is None:
        return []
    if isinstance(plan_msg, str):
        return [line.strip() for line in plan_msg.splitlines() if line.strip()]
    if isinstance(plan_msg, (list, tuple)):
        return [_action_string_from_plan_item(item) for item in plan_msg]
    items = getattr(plan_msg, 'items', None)
    if items is not None:
        return [_action_string_from_plan_item(item) for item in items]
    action = getattr(plan_msg, 'action', None)
    if action is not None:
        return [str(action)]
    return []


def _action_string_from_plan_item(item: Any) -> str:
    if isinstance(item, str):
        return item
    if isinstance(item, dict):
        return str(item.get('action') or item.get('name') or '')
    return str(getattr(item, 'action', '') or '')


def _canonical_symbolic_step(raw_action: Any, *, spec: ScenarioSpec, speed: float) -> str:
    text = str(raw_action or '').strip()
    if not text:
        return ''
    try:
        parsed_steps = parse_plan_text(text)
    except ValueError:
        return ''
    if not parsed_steps:
        return ''
    step = parsed_steps[0]
    if step.name not in SUPPORTED_SYMBOLIC_ACTIONS:
        return ''
    side, shuttle, source, target = _route_parts_from_step(step, spec)
    if step.name == 'prepare_switches':
        return f'prepare_switches {side} {source} {target}'
    if step.name == 'open_stoppers':
        return f'open_stoppers {side} {source} {target}'
    if step.name == 'move_shuttle':
        step_speed = step.kwargs.get('speed') or step.kwargs.get('speed_mps')
        speed_text = step_speed if step_speed is not None else f'{float(speed):.4g}'
        return f'move_shuttle {side} {shuttle} {source} {target} speed={speed_text}'
    if step.name == 'stop_shuttle':
        return f'stop_shuttle {side} {shuttle}'
    if step.name == 'finish_task':
        return f'finish_task {shuttle} {target}'
    return ''


def _route_parts_from_step(step: Any, spec: ScenarioSpec) -> tuple[str, str, str, str]:
    side = _side_from_symbols(step.args, default=spec.side)
    shuttle = _shuttle_from_symbols(step.args, default=spec.shuttle)
    stations = _stations_from_symbols(step.args)
    source = stations[0] if len(stations) > 0 else spec.source
    target = stations[1] if len(stations) > 1 else spec.target
    if step.name == 'finish_task' and len(stations) == 1:
        source = spec.source
        target = stations[0]
    return side, shuttle, source, target


def _side_from_symbols(symbols: tuple[str, ...], *, default: str) -> str:
    for symbol in symbols:
        text = _clean_symbol(symbol).lower()
        if text in {'right', 'left'}:
            return text
        if text.startswith('right_'):
            return 'right'
        if text.startswith('left_'):
            return 'left'
    return default


def _shuttle_from_symbols(symbols: tuple[str, ...], *, default: str) -> str:
    for symbol in symbols:
        text = _clean_symbol(symbol).lower()
        if re.fullmatch(r'[rl][1-4]', text):
            return f'{"right" if text.startswith("r") else "left"}_shuttle_{text[1:]}'
        if (
            re.fullmatch(r'(?:right|left)_shuttle(?:_[1-4])?', text)
            or text in {'right_shuttle', 'left_shuttle'}
        ):
            return text
    return default


def _payload_state_for_spec(spec: ScenarioSpec) -> dict[str, Any]:
    start_slots = dict(spec.start_slots_by_shuttle)
    loaded_shuttles = set(spec.loaded_shuttles)
    if loaded_shuttles or start_slots:
        shuttle_ids = sorted(
            set(start_slots)
            | loaded_shuttles
            | ({spec.shuttle} if spec.shuttle else set()),
            key=_shuttle_sort_key,
        )
        records = []
        by_shuttle = {}
        for shuttle_id in shuttle_ids:
            loaded = shuttle_id in loaded_shuttles
            record = {
                'shuttle_id': shuttle_id,
                'side': spec.side,
                'loaded': loaded,
                'payload_type': 'box' if loaded else 'none',
                'model_input_exposure': 'excluded',
            }
            if start_slots.get(shuttle_id):
                record['start_slot'] = start_slots[shuttle_id]
            records.append(record)
            by_shuttle[shuttle_id] = dict(record)
        return {
            'shuttles': records,
            'by_shuttle': by_shuttle,
            'model_input_exposure': 'excluded',
        }

    loaded = spec.payload_condition == 'loaded'
    return {
        'shuttles': [
            {
                'shuttle_id': spec.shuttle,
                'side': spec.side,
                'loaded': loaded,
                'payload_type': 'box' if loaded else 'none',
                'model_input_exposure': 'excluded',
            }
        ],
        'by_shuttle': {
            spec.shuttle: {
                'shuttle_id': spec.shuttle,
                'side': spec.side,
                'loaded': loaded,
                'payload_type': 'box' if loaded else 'none',
                'model_input_exposure': 'excluded',
            }
        },
        'model_input_exposure': 'excluded',
    } if spec.payload_condition else {}


def _payload_init_facts_for_spec(spec: ScenarioSpec) -> str:
    if spec.loaded_shuttles:
        if spec.shuttle in set(spec.loaded_shuttles):
            return (
                f'    (loaded {spec.shuttle})\n'
                f'    (carrying_payload {spec.shuttle})'
            )
        return f'    (empty {spec.shuttle})'
    if spec.payload_condition == 'loaded':
        return (
            f'    (loaded {spec.shuttle})\n'
            f'    (carrying_payload {spec.shuttle})'
        )
    if spec.payload_condition == 'empty':
        return f'    (empty {spec.shuttle})'
    return f'    (empty {spec.shuttle})'


def _payload_condition_for_problem(text: str, shuttle: str) -> str:
    shuttle_symbol = re.escape(_clean_symbol(shuttle).lower())
    normalized = text.casefold().replace('-', '_')
    if re.search(rf'\(loaded\s+{shuttle_symbol}\)', normalized):
        return 'loaded'
    if re.search(rf'\(empty\s+{shuttle_symbol}\)', normalized):
        return 'empty'
    if re.search(rf'\(carrying_payload\s+{shuttle_symbol}\)', normalized):
        return 'loaded'
    return ''


def _payload_goal_id(
    *,
    side: str,
    shuttle: str,
    target: str,
    payload_condition: str,
) -> str:
    if payload_condition not in {'loaded', 'empty'}:
        return ''
    token = _short_shuttle_token(shuttle)
    if not token:
        return ''
    return f'{side}_{payload_condition}_{token}_to_{target}'


def _short_shuttle_token(shuttle: str) -> str:
    text = _clean_symbol(shuttle).lower()
    match = re.fullmatch(r'([rl])([1-4])', text)
    if match:
        return f'{match.group(1)}{match.group(2)}'
    match = re.fullmatch(r'(right|left)_shuttle_([1-4])', text)
    if match:
        return f'{"r" if match.group(1) == "right" else "l"}{match.group(2)}'
    return ''


def _shuttle_sort_key(shuttle: Any) -> tuple[int, int, str]:
    text = _clean_symbol(shuttle).lower()
    side_rank = 0 if text.startswith(('r', 'right')) else 1
    match = re.search(r'([1-4])$', text)
    number = int(match.group(1)) if match else 99
    return side_rank, number, text


def _stations_from_symbols(symbols: tuple[str, ...]) -> list[str]:
    stations = []
    for symbol in symbols:
        try:
            station = _station_symbol(symbol)
        except ValueError:
            continue
        if station not in stations:
            stations.append(station)
    return stations


def _normalize_batch_config(config: dict[str, Any]) -> dict[str, Any]:
    goals = _as_string_list(config.get('goals'))
    if not goals:
        raise ValueError('batch config needs at least one goal')
    goals = [_canonical_goal_id(goal) for goal in goals]
    unsupported = [goal for goal in goals if goal not in SUPPORTED_GOALS]
    if unsupported:
        allowed = ', '.join(sorted(SUPPORTED_GOALS))
        raise ValueError(f'unsupported batch goal(s) {unsupported!r}; allowed: {allowed}')

    repetitions = max(int(config.get('repetitions_per_goal', 1) or 1), 1)
    speed_values = [float(value) for value in _as_list(config.get('speed_values', [0.3]))]
    if not speed_values:
        speed_values = [0.3]
    for speed in speed_values:
        if speed <= 0.0:
            raise ValueError(f'batch speed values must be positive, got {speed!r}')

    execute = _as_bool(config.get('execute', False))
    dry_run = _as_bool(config.get('dry_run', not execute))
    if execute and dry_run:
        raise ValueError('batch config cannot enable both dry_run and execute')

    return {
        'batch_id': str(config.get('batch_id') or 'room315_pddl_batch'),
        'batch_config_path': str(config.get('batch_config_path') or ''),
        'goals': goals,
        'optional_later_goals': _as_string_list(config.get('optional_later_goals')),
        'repetitions_per_goal': repetitions,
        'language_seed': int(config.get('language_seed', 0) or 0),
        'speed_values': speed_values,
        'output_dataset_dir': str(config.get('output_dataset_dir') or ''),
        'dry_run': dry_run,
        'execute': execute,
        'shuffle': _as_bool(config.get('shuffle', False)),
    }


def _batch_language_template_id(
    *,
    base_seed: int,
    goal_index: int,
    repetition_index: int,
) -> str:
    index = (
        int(base_seed)
        + int(goal_index)
        + int(repetition_index)
    ) % len(LANGUAGE_TEMPLATE_SEQUENCE)
    return LANGUAGE_TEMPLATE_SEQUENCE[index]


def _as_list(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return list(value)
    if isinstance(value, tuple):
        return list(value)
    return [value]


def _as_string_list(value: Any) -> list[str]:
    return [str(item).strip() for item in _as_list(value) if str(item).strip()]


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).strip().casefold() in {'1', 'true', 'yes', 'on'}


def _batch_execute_enabled(args: argparse.Namespace, config: dict[str, Any]) -> bool:
    if bool(getattr(args, 'execute', False)):
        return True
    if bool(getattr(args, 'dry_run', False)):
        return False
    normalized = _normalize_batch_config(config)
    return bool(normalized['execute'])


def _planning_metadata_for_step(scenario: dict[str, Any], step_index: int) -> dict[str, Any]:
    step_metadata = _scenario_plan_step_metadata(scenario, step_index)
    metadata = {
        'planning_source': 'pddl',
        'pddl_domain': str(scenario.get('pddl_domain') or 'domain_room315.pddl'),
        'pddl_problem': scenario.get('pddl_problem', ''),
        'pddl_goal': scenario.get('pddl_goal', ''),
        'symbolic_plan': list(scenario.get('symbolic_plan') or []),
        'plan_step_index': int(step_index),
        'generated_language': scenario.get('language', ''),
        'language_template_id': scenario.get('generated_language_template_id', ''),
        'target_shuttle_id': scenario.get('target_shuttle_id', ''),
        'payload_condition': scenario.get('payload_condition', ''),
        'payload_present': scenario.get('payload_condition') == 'loaded',
        'payload_type': 'box' if scenario.get('payload_condition') == 'loaded' else 'none',
    }
    metadata.update(step_metadata)
    step_shuttle = str(step_metadata.get('plan_step_target_shuttle_id') or '').strip()
    if step_shuttle:
        step_loaded = _payload_loaded_for_shuttle(scenario, step_shuttle)
        metadata['plan_step_target_shuttle_id'] = step_shuttle
        metadata['payload_condition'] = 'loaded' if step_loaded else 'empty'
        metadata['payload_present'] = step_loaded
        metadata['payload_type'] = 'box' if step_loaded else 'none'
    target_slot = str(step_metadata.get('target_slot') or scenario.get('target_slot') or '')
    if target_slot:
        metadata['target_slot'] = target_slot
    if scenario.get('selection_policy'):
        metadata['selection_policy'] = str(scenario.get('selection_policy') or '')
        metadata['selection_candidates'] = [
            dict(item)
            for item in scenario.get('selection_candidates') or []
            if isinstance(item, dict)
        ]
    if isinstance(scenario.get('blocker_clearance'), dict):
        metadata['blocker_clearance'] = dict(scenario['blocker_clearance'])
    return metadata


def _scenario_plan_step_metadata(scenario: dict[str, Any], step_index: int) -> dict[str, Any]:
    entries = scenario.get('plan_step_metadata') or []
    if isinstance(entries, list) and 0 <= int(step_index) < len(entries):
        entry = entries[int(step_index)]
        if isinstance(entry, dict):
            return dict(entry)
    return {}


def _payload_loaded_for_shuttle(scenario: dict[str, Any], shuttle_id: str) -> bool:
    payload_state = scenario.get('payload_state', {}) if isinstance(scenario, dict) else {}
    if not isinstance(payload_state, dict):
        return bool(scenario.get('payload_condition') == 'loaded')
    by_shuttle = payload_state.get('by_shuttle', {})
    if isinstance(by_shuttle, dict):
        entry = by_shuttle.get(shuttle_id)
        if not isinstance(entry, dict):
            for key, value in by_shuttle.items():
                if not isinstance(value, dict):
                    continue
                if _clean_symbol(key).lower() == _clean_symbol(shuttle_id).lower():
                    entry = value
                    break
        if isinstance(entry, dict):
            return bool(entry.get('loaded', False))
    return bool(scenario.get('payload_condition') == 'loaded')


def _problem_text_for_spec(spec: ScenarioSpec) -> str:
    if spec.pddl_problem_file:
        problem_path = Path(spec.pddl_problem_file).expanduser()
        if problem_path.exists():
            return problem_path.read_text(encoding='utf-8')
    problem_filename = SUPPORTED_GOALS.get(spec.goal_id, {}).get('problem_file', '')
    candidate = PDDL_DIR / problem_filename if problem_filename else Path()
    if problem_filename and candidate.exists():
        return candidate.read_text(encoding='utf-8')
    return _problem_text_from_goal_spec(spec)


def _problem_text_from_goal_spec(spec: ScenarioSpec) -> str:
    side_station_prefix = f'{spec.side}_'
    source_station = f'{side_station_prefix}{spec.source}'
    target_station = f'{side_station_prefix}{spec.target}'
    switch_group = f'{spec.side}_switch_group'
    stopper_group = f'{spec.side}_stopper_group'
    problem_name = spec.pddl_problem or SUPPORTED_GOALS[spec.goal_id]['problem_name']
    payload_facts = _payload_init_facts_for_spec(spec)
    return f"""(define (problem {problem_name})
  (:domain room315-shuttle)

  (:objects
    {spec.side} - rail_side
    {spec.shuttle} - shuttle
    {source_station} {target_station} - station
    {switch_group} - switch_group
    {stopper_group} - stopper_group
  )

    (:init
    (shuttle_at {spec.shuttle} {source_station})
    (shuttle_stopped_at {spec.shuttle} {source_station})
    (shuttle_on_side {spec.shuttle} {spec.side})
{payload_facts}
    (connected {spec.side} {source_station} {target_station})
  )

  (:goal
    (and
      (task_done {spec.shuttle} {target_station})
    )
  )
)
"""


def _resolved_goal_data(goal_id: str) -> dict[str, Any]:
    data = dict(SUPPORTED_GOALS[goal_id])
    policy = str(data.get('selection_policy') or '').strip()
    if policy == 'nearest_loaded_to_target_slot_then_lowest_id':
        return _resolve_nearest_loaded_goal_data(goal_id, data)
    return data


def _resolve_nearest_loaded_goal_data(goal_id: str, data: dict[str, Any]) -> dict[str, Any]:
    side = str(data.get('side') or '').strip().casefold()
    target_slot = _slot_symbol_or_empty(data.get('target_slot'))
    if side not in {'right', 'left'} or not target_slot:
        raise ValueError(f'goal {goal_id!r} needs side and target_slot for nearest-loaded selection')

    start_slots_by_shuttle = {
        _clean_symbol(shuttle).lower(): _slot_symbol_or_empty(slot)
        for shuttle, slot in dict(data.get('start_slots_by_shuttle') or {}).items()
    }
    loaded_shuttles = tuple(
        _clean_symbol(shuttle).lower()
        for shuttle in data.get('loaded_shuttles') or ()
        if _clean_symbol(shuttle)
    )
    if not loaded_shuttles:
        raise ValueError(f'goal {goal_id!r} needs at least one loaded shuttle candidate')

    target_index = int(target_slot)
    candidates: list[dict[str, Any]] = []
    for shuttle in loaded_shuttles:
        start_slot = start_slots_by_shuttle.get(shuttle, '')
        if not start_slot:
            raise ValueError(f'goal {goal_id!r} is missing start slot for {shuttle!r}')
        distance = abs(int(start_slot) - target_index)
        candidates.append({
            'shuttle_id': shuttle,
            'side': side,
            'loaded': True,
            'start_slot': start_slot,
            'target_slot': target_slot,
            'distance_to_target_slot': distance,
            'selection_policy': data['selection_policy'],
        })

    selected = min(
        candidates,
        key=lambda item: (
            int(item['distance_to_target_slot']),
            _shuttle_sort_key(item['shuttle_id']),
        ),
    )
    selected_shuttle = str(selected['shuttle_id'])
    selected_slot = str(selected['start_slot'])
    source = _station_for_slot(side, selected_slot)
    target = _station_for_slot(side, target_slot)
    if not source or not target:
        raise ValueError(
            f'goal {goal_id!r} could not map slots {selected_slot!r}->{target_slot!r} '
            f'on {side!r} rail to stations'
        )
    if source == target:
        raise ValueError(
            f'goal {goal_id!r} selected {selected_shuttle!r} already in target station '
            f'for slot {target_slot}'
        )

    ranked = []
    for rank, candidate in enumerate(
        sorted(
            candidates,
            key=lambda item: (
                int(item['distance_to_target_slot']),
                _shuttle_sort_key(item['shuttle_id']),
            ),
        ),
        start=1,
    ):
        entry = dict(candidate)
        entry['selection_rank'] = rank
        entry['selected'] = entry['shuttle_id'] == selected_shuttle
        ranked.append(entry)

    resolved = dict(data)
    restore = _resolve_blocker_restore_slot(
        side=side,
        selected_shuttle=selected_shuttle,
        selected_start_slot=selected_slot,
        target_slot=target_slot,
        start_slots_by_shuttle=start_slots_by_shuttle,
        data=data,
    )
    resolved.update({
        'shuttle': selected_shuttle,
        'source': source,
        'target': target,
        'target_slot': target_slot,
        'loaded_shuttles': loaded_shuttles,
        'start_slots_by_shuttle': start_slots_by_shuttle,
        'selection_candidates': tuple(ranked),
        **restore,
    })
    return resolved


def _resolve_blocker_restore_slot(
    *,
    side: str,
    selected_shuttle: str,
    selected_start_slot: str,
    target_slot: str,
    start_slots_by_shuttle: dict[str, str],
    data: dict[str, Any],
) -> dict[str, Any]:
    if not data.get('blocker_shuttle'):
        return {}
    policy = str(data.get('blocker_restore_policy') or '').strip()
    raw_restore_slot = str(data.get('blocker_restore_slot') or '').strip()
    if raw_restore_slot and raw_restore_slot != 'auto':
        return {
            'blocker_restore_slot': _slot_symbol_or_empty(raw_restore_slot),
            'blocker_restore_policy': policy or 'explicit_slot',
            'blocker_restore_slot_source': 'explicit_slot',
            'blocker_restore_candidate_slots': (),
        }
    if not policy:
        policy = 'selected_source_slot_then_nearest_free_slot'

    blocker_shuttle = _clean_symbol(data.get('blocker_shuttle')).lower()
    blocker_clear_slot = _slot_symbol_or_empty(data.get('blocker_clear_slot'))
    selected_start_slot = _slot_symbol_or_empty(selected_start_slot)
    target_slot = _slot_symbol_or_empty(target_slot)
    occupied_after_selected_move = {
        slot
        for shuttle, slot in start_slots_by_shuttle.items()
        if (
            shuttle not in {selected_shuttle, blocker_shuttle}
            and _slot_symbol_or_empty(slot)
        )
    }
    if target_slot:
        occupied_after_selected_move.add(target_slot)
    if blocker_clear_slot:
        occupied_after_selected_move.add(blocker_clear_slot)

    candidate_slots = []
    if selected_start_slot:
        candidate_slots.append(selected_start_slot)
    all_slots = ['1', '2', '3', '4']
    sorted_fallbacks = sorted(
        all_slots,
        key=lambda slot: (
            0 if _station_for_slot(side, slot) == _station_for_slot(side, blocker_clear_slot) else 1,
            abs(int(slot) - int(blocker_clear_slot or slot)),
            int(slot),
        ),
    )
    for slot in sorted_fallbacks:
        if slot not in candidate_slots:
            candidate_slots.append(slot)

    for slot in candidate_slots:
        if slot in occupied_after_selected_move:
            continue
        return {
            'blocker_restore_slot': slot,
            'blocker_restore_policy': policy,
            'blocker_restore_slot_source': (
                'selected_source_slot'
                if slot == selected_start_slot
                else 'nearest_free_slot'
            ),
            'blocker_restore_candidate_slots': tuple(candidate_slots),
        }

    return {
        'blocker_restore_slot': '',
        'blocker_restore_policy': policy,
        'blocker_restore_slot_source': 'none_available',
        'blocker_restore_candidate_slots': tuple(candidate_slots),
    }


def _station_for_slot(side: str, slot: str) -> str:
    return SLOT_STATION_BY_SIDE_AND_SLOT.get((side, _slot_symbol_or_empty(slot)), '')


def scenario_spec_from_inputs(*, goal: str = '', problem: Path | str | None = None) -> ScenarioSpec:
    if problem:
        return scenario_spec_from_problem(Path(problem))
    if goal:
        return scenario_spec_from_goal(goal)
    raise ValueError('provide --goal or --problem')


def scenario_spec_from_goal(goal: str) -> ScenarioSpec:
    goal_id = _canonical_goal_id(goal)
    if goal_id not in SUPPORTED_GOALS:
        allowed = ', '.join(sorted(SUPPORTED_GOALS))
        raise ValueError(f'unsupported Room 315 PDDL goal {goal!r}; allowed: {allowed}')
    data = _resolved_goal_data(goal_id)
    payload_condition = str(data.get('payload_condition') or '')
    pddl_goal = f'{data["shuttle"]} at {data["target"]}'
    if payload_condition:
        pddl_goal = f'{payload_condition} {pddl_goal}'
    problem_file = str(data.get('problem_file') or '')
    return ScenarioSpec(
        goal_id=goal_id,
        side=data['side'],
        shuttle=data['shuttle'],
        source=data['source'],
        target=data['target'],
        pddl_problem=data['problem_name'],
        pddl_goal=pddl_goal,
        pddl_problem_file=str(PDDL_DIR / problem_file) if problem_file else '',
        payload_condition=payload_condition,
        target_slot=str(data.get('target_slot') or ''),
        selection_policy=str(data.get('selection_policy') or ''),
        loaded_shuttles=tuple(data.get('loaded_shuttles') or ()),
        start_slots_by_shuttle=tuple(
            (str(shuttle), str(slot))
            for shuttle, slot in dict(data.get('start_slots_by_shuttle') or {}).items()
        ),
        selection_candidates=tuple(data.get('selection_candidates') or ()),
        blocker_shuttle=str(data.get('blocker_shuttle') or ''),
        blocker_start_slot=str(data.get('blocker_start_slot') or ''),
        blocker_clear_slot=str(data.get('blocker_clear_slot') or ''),
        blocker_restore_slot=str(data.get('blocker_restore_slot') or ''),
        blocker_restore_policy=str(data.get('blocker_restore_policy') or ''),
        blocker_restore_slot_source=str(data.get('blocker_restore_slot_source') or ''),
        blocker_restore_candidate_slots=tuple(data.get('blocker_restore_candidate_slots') or ()),
        clearance_strategy=str(data.get('clearance_strategy') or ''),
    )


def scenario_spec_from_problem(path: Path) -> ScenarioSpec:
    problem_path = path.expanduser()
    text = problem_path.read_text(encoding='utf-8')
    problem_name = _match_first(r'\(problem\s+([^) \t\n]+)', text)
    shuttle, source = _parse_shuttle_at(text)
    goal_shuttle, target = _parse_task_done_goal(text)
    shuttle = goal_shuttle or shuttle
    side = _infer_side(shuttle, source, target)
    payload_condition = _payload_condition_for_problem(text, shuttle)
    base_goal_id = _goal_id(side=side, source=source, target=target)
    goal_id = _payload_goal_id(
        side=side,
        shuttle=shuttle,
        target=target,
        payload_condition=payload_condition,
    )
    if goal_id not in SUPPORTED_GOALS:
        goal_id = base_goal_id
    if goal_id not in SUPPORTED_GOALS:
        allowed = ', '.join(sorted(SUPPORTED_GOALS))
        raise ValueError(f'unsupported Room 315 PDDL problem route {goal_id!r}; allowed: {allowed}')
    data = SUPPORTED_GOALS[goal_id]
    pddl_goal = f'{shuttle} at {target}'
    if payload_condition:
        pddl_goal = f'{payload_condition} {pddl_goal}'
    return ScenarioSpec(
        goal_id=goal_id,
        side=data['side'],
        shuttle=shuttle,
        source=data['source'],
        target=data['target'],
        pddl_problem=str(problem_path),
        pddl_goal=pddl_goal,
        pddl_problem_file=str(problem_path),
        payload_condition=payload_condition,
    )


def write_scenario(path: Path, scenario: dict[str, Any]) -> None:
    path.expanduser().parent.mkdir(parents=True, exist_ok=True)
    path.expanduser().write_text(_json_dumps(scenario) + '\n', encoding='utf-8')


def _parse_shuttle_at(text: str) -> tuple[str, str]:
    match = re.search(r'\(shuttle_at\s+([^) \t\n]+)\s+([^) \t\n]+)\)', text, re.IGNORECASE)
    if not match:
        raise ValueError('PDDL problem is missing initial shuttle_at fact')
    return _clean_symbol(match.group(1)), _station_symbol(match.group(2))


def _parse_task_done_goal(text: str) -> tuple[str, str]:
    match = re.search(r'\(task_done\s+([^) \t\n]+)\s+([^) \t\n]+)\)', text, re.IGNORECASE)
    if not match:
        raise ValueError('PDDL problem is missing task_done goal')
    return _clean_symbol(match.group(1)), _station_symbol(match.group(2))


def _canonical_goal_id(goal: str) -> str:
    text = _clean_symbol(goal).lower()
    aliases = {
        'right_shuttle_at_staubli': 'right_yaskawa_to_staubli',
        'right_shuttle_at_yaskawa': 'right_staubli_to_yaskawa',
        'left_shuttle_at_kuka': 'left_yaskawa_to_kuka',
        'left_shuttle_at_yaskawa': 'left_kuka_to_yaskawa',
        'right_loaded_shuttle_to_slot3': 'right_loaded_to_slot3',
        'right_loaded_shuttle_to_slot_3': 'right_loaded_to_slot3',
        'right_loaded_to_slot_3': 'right_loaded_to_slot3',
        'loaded_right_shuttle_to_slot3': 'right_loaded_to_slot3',
        'right_loaded_to_slot3_with_blocker': 'right_loaded_to_slot3_clear_blocker',
        'right_loaded_shuttle_to_slot3_clear_blocker': 'right_loaded_to_slot3_clear_blocker',
        'loaded_right_shuttle_to_slot3_clear_blocker': 'right_loaded_to_slot3_clear_blocker',
    }
    return aliases.get(text, text)


def _goal_id(*, side: str, source: str, target: str) -> str:
    return f'{side}_{source}_to_{target}'


def _infer_side(*values: str) -> str:
    for value in values:
        text = str(value or '').casefold()
        if text.startswith('right') or '_right_' in text:
            return 'right'
        if text.startswith('left') or '_left_' in text:
            return 'left'
    raise ValueError(f'could not infer side from {values!r}')


def _station_symbol(value: str) -> str:
    symbol = _clean_symbol(value).lower()
    if symbol.startswith('right_') or symbol.startswith('left_'):
        symbol = symbol.split('_', 1)[1]
    if symbol not in {'yaskawa', 'staubli', 'kuka'}:
        raise ValueError(f'unsupported Room 315 station {value!r}')
    return symbol


def _slot_symbol_or_empty(value: Any) -> str:
    match = re.search(r'[1-4]', str(value or ''))
    return match.group(0) if match else ''


def _clean_symbol(value: str) -> str:
    return str(value or '').strip().strip('()[]{}:,').replace('-', '_')


def _match_first(pattern: str, text: str) -> str:
    match = re.search(pattern, text, re.IGNORECASE)
    return _clean_symbol(match.group(1)) if match else ''


def _json_dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, sort_keys=True)


def _safety_metrics_from_status(status: dict[str, Any]) -> dict[str, Any]:
    safety = status.get('safety_decoder', {})
    if isinstance(safety, dict) and isinstance(safety.get('metrics'), dict):
        return dict(safety['metrics'])
    metrics = status.get('safety_decoder_metrics', {})
    return dict(metrics) if isinstance(metrics, dict) else {}


def _finalize_execution_result(
    scenario: dict[str, Any],
    transport: ScenarioTransport,
    result: dict[str, Any],
) -> dict[str, Any]:
    status = getattr(transport, 'latest_status', {})
    dataset_status = getattr(transport, 'latest_dataset_status', {})
    supervisor_metrics = _safety_metrics_from_status(status if isinstance(status, dict) else {})
    validation = build_validation_result(
        scenario,
        execution_result=result,
        supervisor_metrics=supervisor_metrics,
        dataset_status=dataset_status if isinstance(dataset_status, dict) else {},
        status=status if isinstance(status, dict) else {},
    )
    result['validation'] = validation
    validation_path = _write_validation_for_transport(transport, validation)
    if validation_path:
        result['validation_path'] = validation_path
    return result


def _write_validation_for_transport(
    transport: ScenarioTransport,
    validation: dict[str, Any],
) -> str:
    ref: dict[str, Any] = {}
    episode_ref = getattr(transport, 'validation_episode_ref', None)
    if callable(episode_ref):
        maybe_ref = episode_ref()
        if isinstance(maybe_ref, dict):
            ref.update(maybe_ref)
    dataset_status = getattr(transport, 'latest_dataset_status', {})
    if isinstance(dataset_status, dict):
        ref.setdefault('dataset_dir', dataset_status.get('dataset_dir'))
        ref.setdefault('episode_id', dataset_status.get('episode_id'))
    dataset_dir = str(ref.get('dataset_dir') or '').strip()
    episode_id = str(ref.get('episode_id') or '').strip()
    if not dataset_dir or not episode_id:
        return ''
    try:
        return str(write_validation_result(dataset_dir, episode_id, validation))
    except OSError:
        return ''


def _runtime_failure_from_transport(transport: ScenarioTransport) -> str:
    status = getattr(transport, 'latest_status', {})
    metrics = _safety_metrics_from_status(status if isinstance(status, dict) else {})
    return runtime_failure_reason(
        status=status if isinstance(status, dict) else {},
        supervisor_metrics=metrics,
    )


def _scenario_initial_state_ready(
    *,
    status: dict[str, Any],
    side: str,
    target_shuttle: str,
    payload_condition: str,
) -> tuple[bool, str]:
    shuttles = _rail_shuttles_from_status(status, side)
    target_present = target_shuttle in shuttles or any(
        _shuttle_entity_name(name, side) == target_shuttle
        for name in shuttles
    )
    if not target_present:
        available = ', '.join(sorted(shuttles)) or 'none'
        return (
            False,
            (
                f"initial scenario state is not ready: missing shuttle "
                f"'{target_shuttle}' on {side} rail; available: {available}. "
                f"Restart the Room 315 launch with the matching "
                f"{side}_shuttle_count/start_slots and wait for preflight READY."
            ),
        )

    if payload_condition not in {'loaded', 'empty'}:
        return True, ''

    payload_entry = _payload_entry_from_status(status, side, target_shuttle)
    if not payload_entry:
        return (
            False,
            (
                f"initial scenario state is not ready: no payload state for "
                f"'{target_shuttle}' on {side} rail"
            ),
        )

    loaded = bool(payload_entry.get('loaded', False))
    if payload_condition == 'loaded' and not loaded:
        return (
            False,
            (
                f"initial scenario state is not ready: '{target_shuttle}' is "
                'empty but the scenario requires loaded'
            ),
        )
    if payload_condition == 'empty' and loaded:
        return (
            False,
            (
                f"initial scenario state is not ready: '{target_shuttle}' is "
                'loaded but the scenario requires empty'
            ),
        )
    return True, ''


def _scenario_payload_state_ready(
    *,
    status: dict[str, Any],
    side: str,
    scenario: dict[str, Any],
) -> tuple[bool, str]:
    required = _required_payload_states_from_scenario(scenario)
    if not required:
        return True, ''
    for shuttle, expected_loaded in required.items():
        entity = _shuttle_entity_name(shuttle, side) or shuttle
        payload_entry = _payload_entry_from_status(status, side, entity)
        if not payload_entry:
            return (
                False,
                (
                    f"initial scenario state is not ready: no payload state for "
                    f"'{entity}' on {side} rail"
                ),
            )
        actual_loaded = bool(payload_entry.get('loaded', False))
        if actual_loaded != expected_loaded:
            expected = 'loaded' if expected_loaded else 'empty'
            actual = 'loaded' if actual_loaded else 'empty'
            return (
                False,
                (
                    f"initial scenario state is not ready: '{entity}' is "
                    f'{actual} but the scenario requires {expected}'
                ),
            )
    return True, ''


def _required_payload_states_from_scenario(scenario: dict[str, Any]) -> dict[str, bool]:
    payload_state = scenario.get('payload_state', {}) if isinstance(scenario, dict) else {}
    if not isinstance(payload_state, dict):
        return {}
    by_shuttle = payload_state.get('by_shuttle', {})
    if not isinstance(by_shuttle, dict):
        return {}
    required: dict[str, bool] = {}
    for shuttle, entry in by_shuttle.items():
        if not isinstance(entry, dict) or 'loaded' not in entry:
            continue
        shuttle_id = str(entry.get('shuttle_id') or shuttle or '').strip()
        if shuttle_id:
            required[shuttle_id] = bool(entry.get('loaded', False))
    return required


def _rail_shuttles_from_status(status: dict[str, Any], side: str) -> dict[str, Any]:
    rails = status.get('rails', {}) if isinstance(status, dict) else {}
    rail = rails.get(side, {}) if isinstance(rails, dict) else {}
    shuttles = rail.get('shuttles', {}) if isinstance(rail, dict) else {}
    return shuttles if isinstance(shuttles, dict) else {}


def _payload_entry_from_status(
    status: dict[str, Any],
    side: str,
    target_shuttle: str,
) -> dict[str, Any]:
    rails = status.get('rails', {}) if isinstance(status, dict) else {}
    rail = rails.get(side, {}) if isinstance(rails, dict) else {}
    payloads = rail.get('payloads', {}) if isinstance(rail, dict) else {}
    if isinstance(payloads, dict):
        entry = payloads.get(target_shuttle)
        if isinstance(entry, dict):
            return entry
        for key, value in payloads.items():
            if isinstance(value, dict) and _payload_entry_matches_shuttle(
                key,
                value,
                side,
                target_shuttle,
            ):
                return value

    payload_state = status.get('payload_state', {}) if isinstance(status, dict) else {}
    if not isinstance(payload_state, dict):
        return {}

    by_shuttle = payload_state.get('by_shuttle', {})
    if isinstance(by_shuttle, dict):
        entry = by_shuttle.get(target_shuttle)
        if isinstance(entry, dict):
            return entry
        for key, value in by_shuttle.items():
            if isinstance(value, dict) and _payload_entry_matches_shuttle(
                key,
                value,
                side,
                target_shuttle,
            ):
                return value

    shuttles = payload_state.get('shuttles', [])
    if isinstance(shuttles, list):
        for entry in shuttles:
            if isinstance(entry, dict) and _payload_entry_matches_shuttle(
                entry.get('entity_name') or entry.get('shuttle_id'),
                entry,
                side,
                target_shuttle,
            ):
                return entry
    return {}


def _payload_entry_matches_shuttle(
    key: Any,
    entry: dict[str, Any],
    side: str,
    target_shuttle: str,
) -> bool:
    candidates = [
        key,
        entry.get('entity_name'),
        entry.get('name'),
        entry.get('shuttle'),
        entry.get('shuttle_id'),
        entry.get('short_id'),
    ]
    return any(_shuttle_entity_name(candidate, side) == target_shuttle for candidate in candidates)


def _payload_condition_from_scenario(scenario: dict[str, Any]) -> str:
    condition = str(scenario.get('payload_condition') or '').strip().casefold()
    if condition in {'loaded', 'empty'}:
        return condition
    return ''


def _active_sensor_names_from_status(status: dict[str, Any], side: str) -> set[str]:
    rails = status.get('rails', {}) if isinstance(status, dict) else {}
    rail = rails.get(side, {}) if isinstance(rails, dict) else {}
    if not isinstance(rail, dict):
        return set()
    names: set[str] = set()
    for key in ('active_position_sensors', 'active_sensors'):
        readings = rail.get(key, [])
        if not isinstance(readings, list):
            continue
        for reading in readings:
            name = _sensor_name_from_reading(reading)
            if name:
                names.add(name.upper())
    return names


def _active_sensor_readings_for_names(
    status: dict[str, Any],
    side: str,
    names: set[str],
) -> list[dict[str, Any]]:
    rails = status.get('rails', {}) if isinstance(status, dict) else {}
    rail = rails.get(side, {}) if isinstance(rails, dict) else {}
    if not isinstance(rail, dict):
        return []
    wanted = {name.upper() for name in names}
    readings: list[dict[str, Any]] = []
    for key in ('active_position_sensors', 'active_sensors'):
        raw_readings = rail.get(key, [])
        if not isinstance(raw_readings, list):
            continue
        for reading in raw_readings:
            if not isinstance(reading, dict):
                continue
            name = _sensor_name_from_reading(reading).upper()
            if name in wanted:
                readings.append(reading)
    return readings


def _sensor_name_from_reading(reading: Any) -> str:
    if isinstance(reading, dict):
        return str(reading.get('name') or reading.get('sensor') or '').strip()
    return str(reading or '').strip()


def _side_from_scenario(scenario: dict[str, Any]) -> str:
    for target in scenario.get('expected_event_targets') or []:
        if isinstance(target, dict) and target.get('side'):
            side = str(target.get('side') or '').strip().casefold()
            if side in {'right', 'left'}:
                return side
    for key in ('scenario_id', 'goal_id', 'pddl_goal'):
        text = str(scenario.get(key) or '').casefold()
        if 'right' in text:
            return 'right'
        if 'left' in text:
            return 'left'
    return ''


def _target_station_from_scenario(scenario: dict[str, Any]) -> str:
    goal_text = str(scenario.get('pddl_goal') or '')
    match = re.search(r'\bat\s+([A-Za-z0-9_-]+)\b', goal_text, re.IGNORECASE)
    if match:
        try:
            return _station_symbol(match.group(1))
        except ValueError:
            return ''
    return ''


def _target_shuttle_entity_from_scenario(scenario: dict[str, Any], side: str) -> str:
    for target in scenario.get('expected_event_targets') or []:
        if not isinstance(target, dict):
            continue
        if str(target.get('primitive') or '') != 'SHUTTLE_ON':
            continue
        for value in (target.get('target_id'), target.get('shuttle_id')):
            entity = _shuttle_entity_name(value, side)
            if entity:
                return entity
    return ''


def _shuttle_entity_name(value: Any, side: str) -> str:
    text = str(value or '').strip()
    if not text:
        return ''
    lowered = text.casefold()
    match = re.fullmatch(r'([rl])([1-4])', lowered)
    if match:
        side_name = 'right' if match.group(1) == 'r' else 'left'
        return f'room315_{side_name}_shuttle_{match.group(2)}'
    match = re.fullmatch(r'(?:room315_)?(right|left)_shuttle_?([1-4])', lowered)
    if match:
        return f'room315_{match.group(1)}_shuttle_{match.group(2)}'
    match = re.fullmatch(r'(right|left)_shuttle', lowered)
    if match and side:
        return f'room315_{side}_shuttle_1'
    return ''


def _strip_ros_args(argv: list[str]) -> tuple[list[str], list[str]]:
    if '--ros-args' not in argv:
        return argv, []
    try:
        utilities = importlib.import_module('rclpy.utilities')
        return utilities.remove_ros_args(args=argv), argv
    except Exception:
        index = argv.index('--ros-args')
        return argv[:index], argv[index:]


def main(argv: list[str] | None = None) -> int:
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    parser_argv, ros_argv = _strip_ros_args(raw_argv)
    parser = argparse.ArgumentParser(
        description='Generate or execute Room 315 PDDL-style VLA scenarios.'
    )
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument('--problem', type=Path, help='PDDL problem file to generate.')
    input_group.add_argument('--goal', help='Supported symbolic goal id.')
    input_group.add_argument('--batch-config', type=Path, help='YAML batch scenario config.')
    parser.add_argument(
        '--language-seed',
        type=int,
        default=None,
        help='Deterministic seed for paraphrase selection.',
    )
    parser.add_argument(
        '--language-template-id',
        default='',
        help='Explicit deterministic language template id.',
    )
    parser.add_argument(
        '--planner-backend',
        choices=['plansys'],
        default='plansys',
        help='Symbolic planner backend. PlanSys2 is required; no fallback backend is used.',
    )
    parser.add_argument(
        '--planner-service',
        default=DEFAULT_PLANSYS_GET_PLAN_SERVICE,
        help='PlanSys2 GetPlan service name.',
    )
    parser.add_argument(
        '--planner-timeout-s',
        type=float,
        default=DEFAULT_PLANSYS_TIMEOUT_S,
        help='Timeout for PlanSys2 planner service availability and responses.',
    )
    parser.add_argument('--speed', type=float, default=0.3, help='move_shuttle speed annotation.')
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        '--dry-run',
        action='store_true',
        help='Generate JSON without supervisor/Gazebo execution; still uses PlanSys2 planning.',
    )
    mode_group.add_argument(
        '--preflight-only',
        action='store_true',
        help='Check supervisor readiness and required initial shuttle/payload state without execution.',
    )
    mode_group.add_argument('--execute', action='store_true', help='Publish generated commands through the VLA supervisor.')
    parser.add_argument(
        '--ready-line',
        action='store_true',
        help='With --preflight-only, print one READY/NOT READY line instead of full scenario JSON.',
    )
    parser.add_argument(
        '--quiet',
        action='store_true',
        help='Suppress full scenario JSON on stdout; failures still return nonzero.',
    )
    parser.add_argument('--output', type=Path, default=None, help='Optional JSON output path.')
    parser.add_argument('--command-topic', default='/room_315/vla/command')
    parser.add_argument('--episode-control-topic', default='/room_315/vla/episode_control')
    parser.add_argument('--status-topic', default='/room_315/vla/status')
    parser.add_argument('--dataset-status-topic', default='/room_315/vla/dataset_status')
    parser.add_argument('--command-timeout-s', type=float, default=5.0)
    parser.add_argument(
        '--arrival-timeout-s',
        type=float,
        default=120.0,
        help='Timeout for waiting until the shuttle reaches the target station sensor before STOP.',
    )
    args = parser.parse_args(parser_argv)
    planner = create_planner_backend(
        args.planner_backend,
        planner_service=args.planner_service,
        planner_timeout_s=args.planner_timeout_s,
        ros_args=ros_argv or None,
    )

    if args.batch_config is not None:
        batch_config = load_batch_config(args.batch_config)
        if args.execute:
            batch_config['execute'] = True
            batch_config['dry_run'] = False
        elif args.dry_run or args.preflight_only:
            batch_config['execute'] = False
            batch_config['dry_run'] = True
        batch = generate_batch_scenarios(
            batch_config,
            language_seed=args.language_seed,
            language_template_id=args.language_template_id,
            planner=planner,
        )
        if _batch_execute_enabled(args, batch_config):
            transport = RosScenarioTransport(
                command_topic=args.command_topic,
                episode_control_topic=args.episode_control_topic,
                status_topic=args.status_topic,
                dataset_status_topic=args.dataset_status_topic,
                ros_args=ros_argv or None,
            )
            try:
                batch['execution'] = execute_batch_scenarios(
                    batch,
                    transport,
                    command_timeout_s=args.command_timeout_s,
                    arrival_timeout_s=args.arrival_timeout_s,
                )
            finally:
                transport.shutdown()
        if args.output is not None:
            write_scenario(args.output, batch)
        if not args.quiet:
            print(_json_dumps(batch))
        execution = batch.get('execution')
        if isinstance(execution, dict) and not bool(execution.get('success', False)):
            if args.quiet:
                print(
                    f"FAILED: {execution.get('failure_reason') or 'batch execution failed'}",
                    file=sys.stderr,
                )
            return 1
        return 0

    scenario = generate_scenario(
        goal=args.goal or '',
        problem=args.problem,
        language_seed=args.language_seed,
        language_template_id=args.language_template_id,
        speed=args.speed,
        planner=planner,
    )
    if args.preflight_only or args.execute:
        transport = RosScenarioTransport(
            command_topic=args.command_topic,
            episode_control_topic=args.episode_control_topic,
            status_topic=args.status_topic,
            dataset_status_topic=args.dataset_status_topic,
            ros_args=ros_argv or None,
        )
        try:
            if args.preflight_only:
                scenario['preflight'] = preflight_scenario(
                    scenario,
                    transport,
                    command_timeout_s=args.command_timeout_s,
                )
            else:
                scenario['execution'] = execute_scenario(
                    scenario,
                    transport,
                    command_timeout_s=args.command_timeout_s,
                    arrival_timeout_s=args.arrival_timeout_s,
                )
        finally:
            transport.shutdown()
    if args.output is not None:
        write_scenario(args.output, scenario)
    if args.preflight_only and args.ready_line:
        print(_preflight_ready_line(scenario))
    elif not args.quiet:
        print(_json_dumps(scenario))
    preflight = scenario.get('preflight')
    if isinstance(preflight, dict) and not bool(preflight.get('ready', False)):
        if args.quiet and not args.ready_line:
            print(
                f"FAILED: {preflight.get('reason') or 'preflight failed'}",
                file=sys.stderr,
            )
        return 1
    execution = scenario.get('execution')
    if isinstance(execution, dict) and not bool(execution.get('success', False)):
        if args.quiet:
            print(
                f"FAILED: {execution.get('failure_reason') or 'execution failed'}",
                file=sys.stderr,
            )
        return 1
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
