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


REPO_ROOT = SCRIPT_DIR.parents[1]
PDDL_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'config' / 'room_315_vla' / 'pddl'
PDDL_DOMAIN_PATH = PDDL_DIR / 'domain_room315.pddl'
DEFAULT_PLANSYS_GET_PLAN_SERVICE = '/planner/get_plan'
DEFAULT_PLANSYS_TIMEOUT_S = 10.0

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
        status_seen = False
        while time.monotonic() <= deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            command_subscribers = int(self.command_pub.get_subscription_count())
            status_seen = bool(self.latest_status)
            if command_subscribers > 0 and status_seen:
                return {'ready': True, 'reason': ''}

        missing = []
        if command_subscribers <= 0:
            missing.append(f'no supervisor subscriber on {self.command_topic}')
        if not status_seen:
            missing.append(f'no supervisor status received on {self.status_topic}')
        return {
            'ready': False,
            'reason': (
                'Room 315 VLA supervisor is not ready: '
                + '; '.join(missing or ['readiness check timed out'])
            ),
        }

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

    def publish_episode_control(self, command: str) -> None:
        msg = self.String()
        msg.data = str(command)
        self.episode_control_pub.publish(msg)

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
    planner = planner or PlanSysPlannerBackend()
    plan = planner.plan(spec, speed=float(speed))

    translated = translate_plan(plan)
    generated = generate_language(
        pddl_goal=spec.pddl_goal,
        pddl_problem=spec.pddl_problem,
        symbolic_plan=plan,
        template_id=language_template_id,
        seed=language_seed,
    )
    return {
        'scenario_id': spec.goal_id,
        'pddl_problem': spec.pddl_problem,
        'pddl_goal': spec.pddl_goal,
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


def execute_scenario(
    scenario: dict[str, Any],
    transport: ScenarioTransport,
    *,
    command_timeout_s: float = 5.0,
    arrival_timeout_s: float = 120.0,
) -> dict[str, Any]:
    """Execute a generated scenario through the supervisor transport."""

    readiness_check = getattr(transport, 'wait_until_ready', None)
    if callable(readiness_check):
        readiness = readiness_check(timeout_s=command_timeout_s)
        if not bool(readiness.get('ready', True)):
            return {
                'success': False,
                'failed_step_index': None,
                'failure_reason': str(readiness.get('reason') or 'supervisor is not ready'),
                'published_commands': [],
            }

    language = str(scenario.get('language') or scenario.get('scenario_id') or 'Room 315 PDDL task')
    transport.publish_episode_control(f'start {language}')

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
            transport.publish_episode_control('stop failure')
            return {
                'success': False,
                'failed_step_index': step_index,
                'failure_reason': reason,
                'published_commands': published_commands,
            }
        if not bool(decision.get('accepted', False)):
            reason = str(decision.get('reason') or 'supervisor rejected command')
            transport.publish_episode_control('stop failure')
            return {
                'success': False,
                'failed_step_index': step_index,
                'failure_reason': reason,
                'supervisor_decision': decision,
                'published_commands': published_commands,
            }

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
                transport.publish_episode_control('stop failure')
                return {
                    'success': False,
                    'failed_step_index': step_index,
                    'failure_reason': reason,
                    'arrival_wait': arrival_result,
                    'published_commands': published_commands,
                }

    transport.publish_episode_control('stop success')
    return {
        'success': True,
        'failed_step_index': None,
        'failure_reason': '',
        'published_commands': published_commands,
    }


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
    sensors = list(TARGET_SENSORS_BY_SIDE_AND_STATION.get((side, target), ()))
    return {
        'side': side,
        'target_station': target,
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
        if text.endswith('_shuttle') or text in {'right_shuttle', 'left_shuttle'}:
            return text
    return default


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
    return {
        'planning_source': 'pddl',
        'pddl_domain': str(scenario.get('pddl_domain') or 'domain_room315.pddl'),
        'pddl_problem': scenario.get('pddl_problem', ''),
        'pddl_goal': scenario.get('pddl_goal', ''),
        'symbolic_plan': list(scenario.get('symbolic_plan') or []),
        'plan_step_index': int(step_index),
        'generated_language': scenario.get('language', ''),
        'language_template_id': scenario.get('generated_language_template_id', ''),
    }


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
    (connected {spec.side} {source_station} {target_station})
  )

  (:goal
    (and
      (task_done {spec.shuttle} {target_station})
    )
  )
)
"""


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
    data = SUPPORTED_GOALS[goal_id]
    return ScenarioSpec(
        goal_id=goal_id,
        side=data['side'],
        shuttle=data['shuttle'],
        source=data['source'],
        target=data['target'],
        pddl_problem=data['problem_name'],
        pddl_goal=f'{data["shuttle"]} at {data["target"]}',
        pddl_problem_file=str(PDDL_DIR / data['problem_file']),
    )


def scenario_spec_from_problem(path: Path) -> ScenarioSpec:
    problem_path = path.expanduser()
    text = problem_path.read_text(encoding='utf-8')
    problem_name = _match_first(r'\(problem\s+([^) \t\n]+)', text)
    shuttle, source = _parse_shuttle_at(text)
    goal_shuttle, target = _parse_task_done_goal(text)
    shuttle = goal_shuttle or shuttle
    side = _infer_side(shuttle, source, target)
    goal_id = _goal_id(side=side, source=source, target=target)
    if goal_id not in SUPPORTED_GOALS:
        allowed = ', '.join(sorted(SUPPORTED_GOALS))
        raise ValueError(f'unsupported Room 315 PDDL problem route {goal_id!r}; allowed: {allowed}')
    data = SUPPORTED_GOALS[goal_id]
    return ScenarioSpec(
        goal_id=goal_id,
        side=data['side'],
        shuttle=data['shuttle'],
        source=data['source'],
        target=data['target'],
        pddl_problem=str(problem_path),
        pddl_goal=f'{data["shuttle"]} at {data["target"]}',
        pddl_problem_file=str(problem_path),
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


def _sensor_name_from_reading(reading: Any) -> str:
    if isinstance(reading, dict):
        return str(reading.get('name') or reading.get('sensor') or '').strip()
    return str(reading or '').strip()


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
    mode_group.add_argument('--execute', action='store_true', help='Publish generated commands through the VLA supervisor.')
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
        elif args.dry_run:
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
        print(_json_dumps(batch))
        return 0

    scenario = generate_scenario(
        goal=args.goal or '',
        problem=args.problem,
        language_seed=args.language_seed,
        language_template_id=args.language_template_id,
        speed=args.speed,
        planner=planner,
    )
    if args.execute:
        transport = RosScenarioTransport(
            command_topic=args.command_topic,
            episode_control_topic=args.episode_control_topic,
            status_topic=args.status_topic,
            dataset_status_topic=args.dataset_status_topic,
            ros_args=ros_argv or None,
        )
        try:
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
    print(_json_dumps(scenario))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
