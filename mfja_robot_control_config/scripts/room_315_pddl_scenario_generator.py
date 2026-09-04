#!/usr/bin/env python3
"""Room 315 PlanSys2-backed PDDL scenario generator.

Dry-run mode produces planned episode JSON only. Execute mode publishes only to
the existing rail-safety supervisor and dataset-control topics; it does not bypass the
supervisor, execute Gazebo directly, or modify model_input.
"""

import argparse
import copy
import importlib
import json
import math
import re
import sys
import time
from dataclasses import dataclass
from dataclasses import replace
from functools import lru_cache
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
from room_315_pddl_validation_gate import validate_generic_execution_boundary
from room_315_pddl_validation_gate import write_validation_result
from room_315_contracts import ObservedFact
from room_315_contracts import ObservedState
from room_315_contracts import TaskGoal
from room_315_multi_shuttle import DEVICE_NAMES
from room_315_multi_shuttle import DEFAULT_ROUTE_SAFETY_MARGIN_M
from room_315_multi_shuttle import DEFAULT_SHUTTLE_LENGTH_M
from room_315_multi_shuttle import SIDES
from room_315_multi_shuttle import all_shuttle_specs
from room_315_multi_shuttle import load_rail_topology
from room_315_multi_shuttle import load_slot_sensor_map
from room_315_multi_shuttle import normalize_shuttle_ref
from room_315_multi_shuttle import occupancy_aware_route_candidates_from_position_to_slot
from room_315_multi_shuttle import route_blockers_from_rails
from room_315_multi_shuttle import route_blocks_between_slots
from room_315_multi_shuttle import route_plan_from_position_to_slot
from room_315_rail_defaults import internal_rail_segment_name_to_public
from room_315_rail_defaults import public_rail_segment_lengths
from room_315_rail_defaults import public_rail_segment_name_to_internal
from room_315_rail_defaults import rail_segment_lengths
from room_315_runtime_contracts import normalize_runtime_clearance_certificate
from room_315_runtime_contracts import runtime_payload_grounding_matches


REPO_ROOT = SCRIPT_DIR.parents[1]
PACKAGE_NAME = 'mfja_robot_control_config'


def _package_root_for_script(script_dir: Path) -> Path:
    """Resolve this package's root from either source or a colcon install.

    Scripts are installed in ``<prefix>/lib/<package>``, while package data is
    installed in ``<prefix>/share/<package>``.  Resolving relative to the
    script directory alone only works from a source checkout and made the
    task-execution node exit during import after a clean colcon build.
    """

    source_package_root = script_dir.parent
    if (source_package_root / 'config').is_dir():
        return source_package_root

    install_prefix = script_dir.parents[1]
    return install_prefix / 'share' / PACKAGE_NAME


PACKAGE_ROOT = _package_root_for_script(SCRIPT_DIR)
PDDL_DIR = PACKAGE_ROOT / 'config' / 'room_315_planning' / 'pddl'
PDDL_DOMAIN_PATH = PDDL_DIR / 'domain_room315.pddl'
KINEMATICS_DIR = PACKAGE_ROOT / 'config' / 'room_315_kinematics'
DEFAULT_PAYLOAD_TRAINING_CASES_PATH = (
    PACKAGE_ROOT
    / 'config'
    / 'room_315_payload_cases'
    / 'payload_training_cases_expanded_160_speed_sweep.yaml'
)
DEFAULT_PLANSYS_GET_PLAN_SERVICE = '/planner/get_plan'
DEFAULT_PLANSYS_TIMEOUT_S = 10.0
DEFAULT_SUPERVISOR_NODE_NAME = 'room_315_rail_safety_supervisor'
DEFAULT_SHUTTLE_SPEED_MPS = 0.3
RAIL_NETWORK_PATH_BY_SIDE = {
    'right': KINEMATICS_DIR / 'rail_network_right.yaml',
    'left': KINEMATICS_DIR / 'rail_network_left.yaml',
}
RAIL_DEVICES_PATH_BY_SIDE = {
    'right': KINEMATICS_DIR / 'rail_devices_right.yaml',
    'left': KINEMATICS_DIR / 'rail_devices_left.yaml',
}


@lru_cache(maxsize=2)
def _planning_rail_topology(side: str) -> Any:
    side = _normalise_planning_side(side)
    return load_rail_topology(
        RAIL_NETWORK_PATH_BY_SIDE[side],
        RAIL_DEVICES_PATH_BY_SIDE[side],
        side=side,
    )

SUPPORTED_SYMBOLIC_ACTIONS = {
    'prepare_switches',
    'open_stoppers',
    'restore_normal_route',
    'set_stoppers',
    'move_shuttle',
    'move_shuttle_to_slot',
    'prepare_topology_route',
    'prepare_slot_topology_route',
    'move_shuttle_from_segment_to_slot',
    'move_shuttle_via_topology_to_slot',
    'begin_route_clearance',
    'relocate_blocker_to_interior',
    'stage_selected_to_interior',
    'finish_route_clearance',
    'begin_segment_route_clearance',
    'relocate_segment_blocker_to_interior',
    'stage_selected_segment_to_interior',
    'finish_segment_route_clearance',
    'pause_route_clearance',
    'stop_shuttle',
    'finish_task',
    'finish_candidate_task',
    'inspect_state',
    'wait_for_clearance',
}
RUNTIME_SYMBOLIC_ACTIONS = frozenset({
    'prepare_switches',
    'open_stoppers',
    'restore_normal_route',
    'move_shuttle_to_slot',
    'prepare_topology_route',
    'prepare_slot_topology_route',
    'move_shuttle_from_segment_to_slot',
    'move_shuttle_via_topology_to_slot',
    'begin_route_clearance',
    'relocate_blocker_to_interior',
    'stage_selected_to_interior',
    'finish_route_clearance',
    'begin_segment_route_clearance',
    'relocate_segment_blocker_to_interior',
    'stage_selected_segment_to_interior',
    'finish_segment_route_clearance',
    'pause_route_clearance',
    'stop_shuttle',
    'finish_task',
    'finish_candidate_task',
    'inspect_state',
})
PDDL_ACTION_TRANSLATION_PROVENANCE = {
    'prepare_switches': 'primitive:SET_SWITCHES',
    'open_stoppers': 'primitive:SET_STOPPERS',
    'restore_normal_route': (
        'supervised_macro:verified all-exterior switches and open stoppers '
        'before slot motion resumes'
    ),
    'set_stoppers': 'primitive:SET_STOPPERS',
    'move_shuttle': 'primitive:SHUTTLE_ON',
    'move_shuttle_to_slot': 'deterministic_macro:move_shuttle with slot metadata',
    'prepare_topology_route': (
        'supervised_macro:authoritative topology switch configuration and '
        'open stoppers'
    ),
    'prepare_slot_topology_route': (
        'supervised_macro:authoritative alternate topology switch '
        'configuration from a known source slot'
    ),
    'move_shuttle_from_segment_to_slot': (
        'deterministic_macro:visual segment origin to identity-bearing slot sensor'
    ),
    'move_shuttle_via_topology_to_slot': (
        'deterministic_macro:known source slot through audited alternate '
        'topology route to identity-bearing slot sensor'
    ),
    'begin_route_clearance': (
        'supervised_macro:hold A3/A4 on the interior route and close A4 stopper'
    ),
    'relocate_blocker_to_interior': (
        'supervised_macro:A3 sensor-certified interior entry with '
        'accepted-visual longitudinal stop confirmation'
    ),
    'stage_selected_to_interior': (
        'supervised_macro:topology-selected user-shuttle vacancy seed with A3 '
        'sensor-certified interior stop'
    ),
    'finish_route_clearance': (
        'supervised_macro:restore exterior switches/open stoppers once after '
        'all blocker relocations'
    ),
    'begin_segment_route_clearance': (
        'supervised_macro:hold A3/A4 interior route for a selected shuttle '
        'bound to an accepted visual topology block'
    ),
    'relocate_segment_blocker_to_interior': (
        'supervised_macro:A3 sensor-certified interior entry with '
        'accepted-visual longitudinal stop confirmation for a frozen '
        'topology-route blocker'
    ),
    'stage_selected_segment_to_interior': (
        'supervised_macro:segment-origin user-shuttle vacancy seed with A3 '
        'sensor-certified interior stop'
    ),
    'finish_segment_route_clearance': (
        'supervised_macro:restore exterior switches/open stoppers and certify '
        'the selected segment-origin topology route clear'
    ),
    'pause_route_clearance': (
        'supervised_macro:capacity-safe clearance pause with certified stopped '
        'interior shuttles before exterior-slot choreography'
    ),
    'stop_shuttle': 'primitive:STOP_NOW',
    'finish_task': 'primitive:DONE',
    'finish_candidate_task': 'deterministic_macro:finish_task for selected candidate',
    'inspect_state': 'deterministic_macro:DONE after validated observation',
    'wait_for_clearance': 'primitive:STOP_NOW while waiting for block/slot clearance',
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
    (side, slot): sensor
    for side in SIDES
    for slot, sensor in load_slot_sensor_map(
        RAIL_DEVICES_PATH_BY_SIDE[side],
        side=side,
    ).items()
}
SLOT_POSE_BY_SIDE_AND_SLOT = {
    ('right', '1'): ('A12E', 0.9172),
    ('right', '2'): ('A12E', 1.4544),
    ('right', '3'): ('A34E', 0.9960),
    ('right', '4'): ('A34E', 1.5219),
    ('left', '1'): ('A12E', 0.9534),
    ('left', '2'): ('A12E', 1.5011),
    ('left', '3'): ('A34E', 0.9522),
    ('left', '4'): ('A34E', 1.4886),
}
SLOT_POSE_ARRIVAL_TOLERANCE_M = 0.08
EXACT_SLOT_ANCHOR_VISUAL_TOLERANCE_RATIO = 0.12
INTERIOR_LOOP_CLEAR_POSE_BY_SIDE_AND_GATE = {
    ('right', 'A1'): ('A12I', 0.7052),
    ('right', 'A3'): ('A34I', 0.7083),
    ('left', 'A1'): ('A12I', 0.7083),
    ('left', 'A3'): ('A34I', 0.7083),
}
# Both physical interior branches are valid bounded holding areas.  Public
# segment and switch names are identical on the two ROS rail interfaces; the
# mirrored left topology is converted to its internal segment names only at
# the topology boundary.
INTERIOR_HOLDING_BRANCH_BY_GATE = {
    'A1': {
        'gate_switch': 'A1',
        'exit_switch': 'A2',
        'target_segment': 'A12I',
        'switches': {'A1': 'I', 'A2': 'I', 'A3': 'E', 'A4': 'E'},
    },
    'A3': {
        'gate_switch': 'A3',
        'exit_switch': 'A4',
        'target_segment': 'A34I',
        'switches': {'A1': 'E', 'A2': 'E', 'A3': 'I', 'A4': 'I'},
    },
}
INTERIOR_LOOP_GATE_BY_PUBLIC_SEGMENT = {
    branch['target_segment']: gate
    for gate, branch in INTERIOR_HOLDING_BRANCH_BY_GATE.items()
}
INTERIOR_LOOP_GATE_BY_SIDE = {
    'right': 'A3',
    'left': 'A3',
}
INTERIOR_LOOP_ENTRY_SENSOR_BY_SIDE_AND_GATE = {
    ('right', 'A1'): 'DA1IR',
    ('right', 'A3'): 'DA3IR',
    ('left', 'A1'): 'DA1IL',
    ('left', 'A3'): 'DA3IL',
}
INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M = 0.08


@dataclass(frozen=True)
class ScenarioSpec:
    goal_id: str
    side: str
    shuttle: str
    source: str
    target: str
    pddl_problem: str = ''
    pddl_goal: str = ''
    payload_condition: str = ''
    target_slot: str = ''
    selection_policy: str = ''
    loaded_shuttles: tuple[str, ...] = ()
    start_slots_by_shuttle: tuple[tuple[str, str], ...] = ()
    selection_candidates: tuple[dict[str, Any], ...] = ()
    blocker_shuttle: str = ''
    blocker_start_slot: str = ''
    blocker_clear_slot: str = ''
    blocker_clear_target: str = ''
    blocker_clear_sensor: str = ''
    blocker_clear_stopper: str = ''
    blocker_restore_slot: str = ''
    blocker_restore_policy: str = ''
    blocker_restore_slot_source: str = ''
    blocker_restore_candidate_slots: tuple[str, ...] = ()
    clearance_strategy: str = ''
    clearance_steps: tuple[dict[str, Any], ...] = ()


@dataclass(frozen=True)
class Room315PddlProblem:
    """Validated PlanSys2 problem built from ObservedState + TaskGoal."""

    problem_name: str
    problem_text: str
    goal_text: str
    goal_type: str
    side: str
    target_station: str = ''
    target_slot: str = ''
    selected_shuttle: str = ''
    selection_policy: str = ''
    provenance: dict[str, Any] | None = None


class PddlProblemBuildError(RuntimeError):
    """Raised when state/goal contracts are not safe enough for PDDL planning."""


class BasePlannerBackend:
    """Planner interface boundary for PlanSys2-backed symbolic planning."""

    def plan(
        self,
        goal_or_problem: ScenarioSpec | Room315PddlProblem,
        *,
        speed: float,
    ) -> list[str]:
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
            executors_module = importlib.import_module('rclpy.executors')
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
        # This adapter can run from a worker thread while the owning ROS node
        # is spinning. It must not share rclpy's global executor, and launch
        # remaps such as ``__node:=...`` must not rename this private client
        # to the owning gateway's name.
        self.node = self.rclpy.create_node(
            'room_315_plansys2_planner_client',
            use_global_arguments=False,
        )
        self.executor = executors_module.SingleThreadedExecutor(
            context=self.node.context,
        )
        self.executor.add_node(self.node)
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
        self.executor.spin_until_future_complete(
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
            self.executor.remove_node(self.node)
            self.node.destroy_node()
        finally:
            try:
                self.executor.shutdown(timeout_sec=1.0)
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

    def plan(
        self,
        goal_or_problem: ScenarioSpec | Room315PddlProblem,
        *,
        speed: float,
    ) -> list[str]:
        spec = goal_or_problem if isinstance(goal_or_problem, ScenarioSpec) else None
        problem = (
            build_pddl_problem_from_spec(goal_or_problem)
            if isinstance(goal_or_problem, ScenarioSpec)
            else goal_or_problem
        )
        domain_text = self._domain_text()
        client = self.planner_client or PlanSys2GetPlanClient(
            service_name=self.planner_service,
            timeout_s=self.timeout_s,
            ros_args=self.ros_args,
        )
        close = getattr(client, 'close', None) if self.planner_client is None else None
        try:
            plan_msg = client.get_plan(domain=domain_text, problem=problem.problem_text)
        finally:
            if close is not None:
                close()
        return _symbolic_plan_from_plansys_plan(
            plan_msg,
            spec=spec,
            problem=problem,
            speed=float(speed),
        )

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
        target_slot: str = '',
        target_station: str = '',
        target_segment: str = '',
        target_s: float | None = None,
        target_tolerance_m: float | None = None,
    ) -> dict[str, Any]:
        return {'arrived': True, 'reason': ''}

    def wait_for_visual_position_and_stop(
        self,
        *,
        side: str,
        shuttle: str,
        target_segment: str,
        target_s_m: float,
        tolerance_m: float,
        entry_sensor: str = '',
        minimum_clearance_delay_s: float = 0.0,
        motion_origin_s_m: float | None = None,
        timeout_s: float,
    ) -> dict[str, Any]:
        return {
            'arrived': False,
            'reason': 'visual position stop is not implemented by this transport',
        }

    def wait_for_stopper_state(
        self,
        *,
        side: str,
        stoppers: dict[str, Any],
        timeout_s: float,
    ) -> dict[str, Any]:
        return {'ready': True, 'reason': ''}

    def wait_for_switch_state(
        self,
        *,
        side: str,
        switches: dict[str, Any],
        timeout_s: float,
    ) -> dict[str, Any]:
        return {'ready': True, 'reason': ''}

    def wait_for_shuttle_stopped(
        self,
        *,
        side: str,
        shuttle: str,
        timeout_s: float,
    ) -> dict[str, Any]:
        return {'ready': True, 'reason': ''}


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
        require_dataset_recorder: bool = False,
    ) -> None:
        self.command_topic = command_topic
        self.episode_control_topic = episode_control_topic
        self.status_topic = status_topic
        self.dataset_status_topic = dataset_status_topic
        self.require_dataset_recorder = bool(require_dataset_recorder)
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
                'Room 315 rail-safety supervisor is not ready: '
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

    def publish_episode_start_and_wait(self, *, goal: str, timeout_s: float) -> dict[str, Any]:
        if not self.require_dataset_recorder and not self._dataset_status_publisher_count():
            return {'ready': True, 'reason': '', 'observed': False}
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        wanted_goal = str(goal or '').strip()
        start_command = f'start {wanted_goal}' if wanted_goal else 'start'
        latest: dict[str, Any] = {}
        ignored_episode_ids = self._current_dataset_episode_ids()
        last_publish_s = 0.0
        published_start = False
        dataset_status_publishers: int | None = None
        episode_control_subscribers: int | None = None
        while time.monotonic() <= deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            latest = dict(self.latest_dataset_status)
            if not published_start:
                episode_id = str(latest.get('episode_id') or '').strip()
                if episode_id:
                    ignored_episode_ids.add(episode_id)
            if published_start and self._dataset_status_matches_started_episode(
                latest,
                wanted_goal,
                ignored_episode_ids=ignored_episode_ids,
            ):
                return {
                    'ready': True,
                    'reason': '',
                    'observed': True,
                    'latest_dataset_status': latest,
                }

            dataset_status_publishers = self._dataset_status_publisher_count()
            episode_control_subscribers = self._episode_control_subscriber_count()
            if episode_control_subscribers == 0:
                continue

            now_s = time.monotonic()
            if now_s - last_publish_s >= 0.5:
                self.publish_episode_control(start_command)
                last_publish_s = now_s
                published_start = True

        return {
            'ready': False,
            'reason': (
                'dataset recorder did not acknowledge episode start'
                + (f' for {wanted_goal!r}' if wanted_goal else '')
            ),
            'latest_dataset_status': latest,
            'dataset_status_publishers': dataset_status_publishers,
            'episode_control_subscribers': episode_control_subscribers,
            'observed': True,
        }

    def wait_for_episode_started(self, *, goal: str, timeout_s: float) -> dict[str, Any]:
        if not self.require_dataset_recorder and not self._dataset_status_publisher_count():
            return {'ready': True, 'reason': '', 'observed': False}
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        wanted_goal = str(goal or '').strip()
        latest: dict[str, Any] = {}
        while time.monotonic() <= deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            latest = dict(self.latest_dataset_status)
            if self._dataset_status_matches_started_episode(latest, wanted_goal):
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
        if not self.require_dataset_recorder and not self._dataset_status_publisher_count():
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

    def _episode_control_subscriber_count(self) -> int | None:
        endpoint_info = getattr(self.node, 'get_subscriptions_info_by_topic', None)
        if callable(endpoint_info):
            try:
                infos = endpoint_info(self.episode_control_topic)
            except Exception:
                infos = []
            recorder_count = 0
            for info in infos:
                node_name = str(getattr(info, 'node_name', '') or '').strip().lstrip('/')
                if node_name == 'room_315_visual_state_dataset_recorder':
                    recorder_count += 1
            if recorder_count:
                return recorder_count
            if infos:
                return len(infos)
        count_fn = getattr(self.episode_control_pub, 'get_subscription_count', None)
        if callable(count_fn):
            try:
                return int(count_fn())
            except Exception:
                return None
        return None

    @staticmethod
    def _dataset_status_matches_started_episode(
        latest: dict[str, Any],
        wanted_goal: str,
        *,
        ignored_episode_ids: set[str] | None = None,
    ) -> bool:
        if not bool(latest.get('active', False)):
            return False
        if wanted_goal and str(latest.get('task') or '').strip() != wanted_goal:
            return False
        episode_id = str(latest.get('episode_id') or '').strip()
        if not episode_id:
            return False
        return episode_id not in (ignored_episode_ids or set())

    def _current_dataset_episode_ids(self) -> set[str]:
        episode_ids = set()
        latest = self.latest_dataset_status if isinstance(self.latest_dataset_status, dict) else {}
        for episode_id in (self.last_episode_id, latest.get('episode_id')):
            normalized = str(episode_id or '').strip()
            if normalized:
                episode_ids.add(normalized)
        return episode_ids

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

    def wait_for_stopper_state(
        self,
        *,
        side: str,
        stoppers: dict[str, Any],
        timeout_s: float,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        expected = _expanded_stopper_assignments(stoppers)
        while time.monotonic() <= deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if _stoppers_match_status(self.latest_status, side, expected):
                return {
                    'ready': True,
                    'reason': '',
                    'side': side,
                    'stoppers': expected,
                }
        return {
            'ready': False,
            'reason': (
                f'timeout waiting for {side} stopper state '
                f'{_json_dumps(expected)}'
            ),
            'side': side,
            'stoppers': expected,
        }

    def wait_for_switch_state(
        self,
        *,
        side: str,
        switches: dict[str, Any],
        timeout_s: float,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        expected = _expanded_switch_assignments(switches)
        while time.monotonic() <= deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if _switches_match_status(self.latest_status, side, expected):
                return {
                    'ready': True,
                    'reason': '',
                    'side': side,
                    'switches': expected,
                }
        return {
            'ready': False,
            'reason': (
                f'timeout waiting for {side} switch state '
                f'{_json_dumps(expected)}'
            ),
            'side': side,
            'switches': expected,
        }

    def wait_for_shuttle_stopped(
        self,
        *,
        side: str,
        shuttle: str,
        timeout_s: float,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        last_state: dict[str, Any] = {}
        while time.monotonic() <= deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            last_state = _shuttle_state_from_status(
                self.latest_status,
                side=side,
                shuttle=shuttle,
            )
            if _shuttle_state_is_stopped(last_state):
                return {
                    'ready': True,
                    'reason': '',
                    'side': side,
                    'shuttle': shuttle,
                    'mode': str(last_state.get('mode') or '').strip(),
                    'speed': last_state.get('speed'),
                }
            if _shuttle_state_is_falling(last_state):
                return {
                    'ready': False,
                    'reason': f'{side} shuttle {shuttle} is in falling mode',
                    'side': side,
                    'shuttle': shuttle,
                    'last_state': last_state,
                }
        return {
            'ready': False,
            'reason': (
                f'timeout waiting for {side} shuttle {shuttle} to stop '
                f'after OFF command'
            ),
            'side': side,
            'shuttle': shuttle,
            'last_state': last_state,
        }

    def wait_for_target_arrival(
        self,
        *,
        side: str,
        target_sensors: list[str],
        shuttle: str,
        timeout_s: float,
        target_slot: str = '',
        target_station: str = '',
        target_segment: str = '',
        target_s: float | None = None,
        target_tolerance_m: float | None = None,
    ) -> dict[str, Any]:
        wanted = {
            str(sensor or '').strip().upper()
            for sensor in target_sensors
            if str(sensor or '').strip()
        }
        has_pose_target = bool(str(target_segment or '').strip()) and target_s is not None
        if not wanted and not has_pose_target:
            return {
                'arrived': False,
                'reason': (
                    'no target sensors or target pose were configured for this '
                    'PDDL move_shuttle step'
                ),
            }

        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        active: set[str] = set()
        last_slot_match: dict[str, Any] = {}
        last_pose_match: dict[str, Any] = {}
        while time.monotonic() <= deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            active = _active_sensor_names_from_status(self.latest_status, side)
            if wanted:
                matched = _matched_target_sensor_names_from_status(
                    self.latest_status,
                    side=side,
                    wanted=wanted,
                    shuttle=shuttle,
                )
                if matched:
                    return {
                        'arrived': True,
                        'reason': '',
                        'matched_sensors': matched,
                        'target_sensors': sorted(wanted),
                        'side': side,
                        'shuttle': shuttle,
                        'target_slot': target_slot,
                        'target_station': target_station,
                    }
            if has_pose_target:
                last_pose_match = _target_pose_match_from_status(
                    self.latest_status,
                    side=side,
                    shuttle=shuttle,
                    target_segment=target_segment,
                    target_s=float(target_s),
                    tolerance_m=target_tolerance_m,
                )
                if bool(last_pose_match.get('arrived', False)):
                    return {
                        'arrived': True,
                        'reason': str(last_pose_match.get('reason') or ''),
                        'matched_sensors': [],
                        'target_sensors': sorted(wanted),
                        'side': side,
                        'shuttle': shuttle,
                        'target_slot': target_slot,
                        'target_station': target_station,
                        'target_segment': target_segment,
                        'target_s': target_s,
                        'matched_by': 'shuttle_pose',
                        'slot_pose': last_pose_match,
                    }
            last_slot_match = _target_slot_pose_match_from_status(
                self.latest_status,
                side=side,
                target_slot=target_slot,
                shuttle=shuttle,
            )
            if bool(last_slot_match.get('arrived', False)):
                return {
                    'arrived': True,
                    'reason': str(last_slot_match.get('reason') or ''),
                    'matched_sensors': [],
                    'target_sensors': sorted(wanted),
                    'side': side,
                    'shuttle': shuttle,
                    'target_slot': target_slot,
                    'target_station': target_station,
                    'matched_by': 'shuttle_pose',
                    'slot_pose': last_slot_match,
                }

        wanted_text = ', '.join(sorted(wanted)) if wanted else 'none'
        return {
            'arrived': False,
            'reason': (
                f'timeout waiting for {shuttle or "shuttle"} on {side} target '
                f'sensor(s): {wanted_text}'
            ),
            'active_sensors': sorted(active),
            'target_sensors': sorted(wanted),
            'side': side,
            'shuttle': shuttle,
            'target_slot': target_slot,
            'target_station': target_station,
            'target_segment': target_segment,
            'target_s': target_s,
            'last_slot_pose': last_slot_match,
            'last_target_pose': last_pose_match,
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
    case_id: str = '',
    case_config: Path | str | None = None,
    language_seed: int | None = None,
    language_template_id: str = '',
    speed: float = 0.3,
    planner: BasePlannerBackend | None = None,
) -> dict[str, Any]:
    """Build a dry-run planned episode structure."""

    spec = scenario_spec_from_inputs(
        case_id=case_id,
        case_config=case_config,
    )
    problem = build_pddl_problem_from_spec(spec)
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
        'planner_provenance': problem.provenance or {},
        'symbolic_plan': plan,
        'primitive_commands': [step.command for step in translated],
        'expected_event_targets': [step.event_action for step in translated],
    }
    if spec.target_slot:
        scenario['target_slot'] = spec.target_slot
    route_topology = _route_topology_metadata_for_spec(spec)
    if route_topology:
        scenario['route_topology'] = route_topology
    if spec.selection_policy:
        scenario['selection_policy'] = spec.selection_policy
        scenario['selection_candidates'] = [dict(item) for item in spec.selection_candidates]
    if spec.target_slot and not spec.blocker_shuttle and not spec.clearance_steps:
        scenario['plan_step_metadata'] = _slot_target_plan_step_metadata(spec, plan)
    if spec.clearance_steps:
        scenario['blocker_clearance'] = _multi_blocker_clearance_metadata_for_spec(spec)
        scenario['plan_step_metadata'] = _multi_blocker_clear_plan_step_metadata(spec)
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

    # Audit the exact enriched payloads before touching transport readiness,
    # dataset episode control, or command publishers. Topology/clearance
    # macros are valid static PDDL fixtures but require the closed-loop
    # executive's one-step, re-observe, effect-verify, and replan protocol.
    execution_payloads = command_payloads_for_execution(scenario)
    execution_boundary = validate_generic_execution_boundary(
        scenario,
        command_payloads=execution_payloads,
    )
    if not execution_boundary['valid']:
        result = {
            'success': False,
            'failed_step_index': None,
            'failure_reason': execution_boundary['failure_reasons'][0],
            'direct_execution_validation': execution_boundary,
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
    episode_started = _publish_episode_start_and_wait(
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
    for step_index, payload in enumerate(execution_payloads):
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

        switch_wait = _switch_state_wait_for_payload(payload)
        wait_for_switches = getattr(transport, 'wait_for_switch_state', None)
        if switch_wait is not None and callable(wait_for_switches):
            switch_result = wait_for_switches(
                side=switch_wait['side'],
                switches=switch_wait['switches'],
                timeout_s=command_timeout_s,
            )
            if not bool(switch_result.get('ready', False)):
                reason = str(switch_result.get('reason') or 'switch state wait failed')
                _publish_episode_stop_and_wait(
                    transport,
                    'stop failure',
                    timeout_s=command_timeout_s,
                )
                return _finalize_execution_result(scenario, transport, {
                    'success': False,
                    'failed_step_index': step_index,
                    'failure_reason': reason,
                    'switch_wait': switch_result,
                    'published_commands': published_commands,
                    'executed_command_count': len(published_commands),
                })

        stopper_wait = _stopper_state_wait_for_payload(payload)
        wait_for_stoppers = getattr(transport, 'wait_for_stopper_state', None)
        if stopper_wait is not None and callable(wait_for_stoppers):
            stopper_result = wait_for_stoppers(
                side=stopper_wait['side'],
                stoppers=stopper_wait['stoppers'],
                timeout_s=command_timeout_s,
            )
            if not bool(stopper_result.get('ready', False)):
                reason = str(stopper_result.get('reason') or 'stopper state wait failed')
                _publish_episode_stop_and_wait(
                    transport,
                    'stop failure',
                    timeout_s=command_timeout_s,
                )
                return _finalize_execution_result(scenario, transport, {
                    'success': False,
                    'failed_step_index': step_index,
                    'failure_reason': reason,
                    'stopper_wait': stopper_result,
                    'published_commands': published_commands,
                    'executed_command_count': len(published_commands),
                })

        shuttle_stop_wait = _shuttle_stop_wait_for_payload(payload)
        wait_for_shuttle_stopped = getattr(transport, 'wait_for_shuttle_stopped', None)
        if shuttle_stop_wait is not None and callable(wait_for_shuttle_stopped):
            shuttle_stop_result = wait_for_shuttle_stopped(
                side=shuttle_stop_wait['side'],
                shuttle=shuttle_stop_wait['shuttle'],
                timeout_s=command_timeout_s,
            )
            if not bool(shuttle_stop_result.get('ready', False)):
                reason = str(
                    shuttle_stop_result.get('reason')
                    or 'shuttle stop wait failed'
                )
                _publish_episode_stop_and_wait(
                    transport,
                    'stop failure',
                    timeout_s=command_timeout_s,
                )
                return _finalize_execution_result(scenario, transport, {
                    'success': False,
                    'failed_step_index': step_index,
                    'failure_reason': reason,
                    'shuttle_stop_wait': shuttle_stop_result,
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
                target_slot=arrival_wait.get('target_slot', ''),
                target_station=arrival_wait.get('target_station', ''),
                target_segment=arrival_wait.get('target_segment', ''),
                target_s=arrival_wait.get('target_s'),
                target_tolerance_m=arrival_wait.get('target_tolerance_m'),
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


def _publish_episode_start_and_wait(
    transport: ScenarioTransport,
    *,
    goal: str,
    timeout_s: float,
) -> dict[str, Any]:
    start_and_wait = getattr(transport, 'publish_episode_start_and_wait', None)
    if callable(start_and_wait):
        return start_and_wait(goal=goal, timeout_s=timeout_s)
    start_command = f'start {goal}' if str(goal or '').strip() else 'start'
    transport.publish_episode_control(start_command)
    return _wait_for_episode_started(
        transport,
        goal=goal,
        timeout_s=timeout_s,
    )


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
    symbolic_plan = list(scenario.get('symbolic_plan') or [])
    payloads = []
    for index, command in enumerate(commands):
        metadata = _planning_metadata_for_step(scenario, index)
        payload = dict(command)
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
    if not parsed_steps or parsed_steps[0].name not in {
        'move_shuttle',
        'move_shuttle_to_slot',
        'move_shuttle_from_segment_to_slot',
        'move_shuttle_via_topology_to_slot',
    }:
        return None

    step = parsed_steps[0]
    side = _side_from_symbols(step.args, default=str(payload.get('side') or 'right'))
    target = str(payload.get('target_station') or '').strip()
    if not target:
        target = _target_station_from_move_step(step, scenario)
    sensors = _as_string_list(payload.get('target_sensors'))
    target_slot = _slot_symbol_or_empty(payload.get('target_slot'))
    target_segment = str(payload.get('target_segment') or '').strip().upper()
    target_s = _optional_float(payload.get('target_s'))
    target_tolerance_m = _optional_float(payload.get('target_tolerance_m'))
    if not sensors and not target_slot and not target_segment:
        target_slot = _target_slot_from_scenario(scenario)
    if not sensors and not target_segment:
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
        'target_segment': target_segment,
        'target_s': target_s,
        'target_tolerance_m': target_tolerance_m,
        'shuttle': str(payload.get('shuttle') or ''),
    }


def _stopper_state_wait_for_payload(payload: dict[str, Any]) -> dict[str, Any] | None:
    if str(payload.get('action') or '').casefold() != 'stoppers':
        return None
    stoppers = payload.get('stoppers')
    if not isinstance(stoppers, dict) or not stoppers:
        return None
    side = _side_from_symbols((), default=str(payload.get('side') or 'right'))
    return {
        'side': side,
        'stoppers': dict(stoppers),
    }


def _switch_state_wait_for_payload(payload: dict[str, Any]) -> dict[str, Any] | None:
    if str(payload.get('action') or '').casefold() != 'switches':
        return None
    switches = payload.get('switches')
    if not isinstance(switches, dict) or not switches:
        return None
    side = _side_from_symbols((), default=str(payload.get('side') or 'right'))
    return {
        'side': side,
        'switches': dict(switches),
    }


def _shuttle_stop_wait_for_payload(payload: dict[str, Any]) -> dict[str, Any] | None:
    if str(payload.get('action') or '').casefold() != 'shuttle':
        return None
    if str(payload.get('command') or '').casefold() != 'off':
        return None
    side = _side_from_symbols((), default=str(payload.get('side') or 'right'))
    shuttle = str(payload.get('shuttle') or payload.get('shuttle_id') or '').strip()
    if not shuttle:
        return None
    return {
        'side': side,
        'shuttle': shuttle,
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


def oracle_blocker_clear_symbolic_plan_for_spec(
    spec: ScenarioSpec,
    *,
    speed: float,
) -> list[str]:
    """Oracle-only fixture plan retained for tests; production uses PlanSys2."""
    if not spec.blocker_shuttle:
        return []
    if _blocker_enters_interior_loop(spec):
        return oracle_interior_loop_clear_symbolic_plan_for_spec(spec, speed=speed)
    blocker_source = _station_for_slot(spec.side, spec.blocker_start_slot)
    blocker_target = _station_for_slot(spec.side, spec.blocker_clear_slot)
    selected_source = spec.source
    selected_target = spec.target
    speed_text = f'{float(speed):.4g}'
    blocker_move_kwargs = f'speed={speed_text}'
    if spec.blocker_clear_stopper:
        blocker_move_kwargs = (
            f'{blocker_move_kwargs} target_stopper={spec.blocker_clear_stopper}'
        )
    plan = [
        f'prepare_switches {spec.side} {blocker_source} {blocker_target}',
    ]
    if spec.blocker_clear_stopper:
        plan.append(f'set_stoppers {spec.side} {spec.blocker_clear_stopper} closed')
    else:
        plan.append(f'open_stoppers {spec.side} {blocker_source} {blocker_target}')
    plan.extend([
        (
            f'move_shuttle {spec.side} {spec.blocker_shuttle} '
            f'{blocker_source} {blocker_target} {blocker_move_kwargs}'
        ),
        f'stop_shuttle {spec.side} {spec.blocker_shuttle}',
        f'prepare_switches {spec.side} {selected_source} {selected_target}',
        f'open_stoppers {spec.side} {selected_source} {selected_target}',
        (
            f'move_shuttle {spec.side} {spec.shuttle} '
            f'{selected_source} {selected_target} speed={speed_text}'
        ),
        f'stop_shuttle {spec.side} {spec.shuttle}',
    ])
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


def oracle_multi_blocker_clear_symbolic_plan_for_spec(
    spec: ScenarioSpec,
    *,
    speed: float,
) -> list[str]:
    """Oracle-only fixture plan retained for tests; production uses PlanSys2."""

    if not spec.clearance_steps:
        return []
    speed_text = f'{float(speed):.4g}'
    plan: list[str] = []
    for step in spec.clearance_steps:
        if _clearance_step_enters_interior_loop(step):
            plan.extend(
                oracle_interior_loop_clear_symbolic_steps_for_clearance_step(
                    spec,
                    step,
                    speed_text=speed_text,
                )
            )
            continue
        plan.extend(
            oracle_slot_clear_symbolic_steps_for_clearance_step(
                spec,
                step,
                speed_text=speed_text,
            )
        )

    plan.extend([
        f'prepare_switches {spec.side} {spec.source} {spec.target}',
        f'open_stoppers {spec.side} {spec.source} {spec.target}',
        (
            f'move_shuttle {spec.side} {spec.shuttle} '
            f'{spec.source} {spec.target} speed={speed_text}'
        ),
        f'stop_shuttle {spec.side} {spec.shuttle}',
        f'finish_task {spec.shuttle} {spec.target}',
    ])
    return plan


def oracle_station_route_symbolic_plan_for_spec(
    spec: ScenarioSpec,
    *,
    speed: float,
) -> list[str]:
    """Oracle-only fixture plan retained for tests; production uses PlanSys2."""

    speed_text = f'{float(speed):.4g}'
    return [
        f'prepare_switches {spec.side} {spec.source} {spec.target}',
        f'open_stoppers {spec.side} {spec.source} {spec.target}',
        (
            f'move_shuttle {spec.side} {spec.shuttle} '
            f'{spec.source} {spec.target} speed={speed_text}'
        ),
        f'stop_shuttle {spec.side} {spec.shuttle}',
        f'finish_task {spec.shuttle} {spec.target}',
    ]


def _blocker_enters_interior_loop(spec: ScenarioSpec) -> bool:
    return (
        str(spec.blocker_clear_target or '').strip().casefold() == 'interior_loop'
        or str(spec.clearance_strategy or '').strip().casefold()
        == 'clear_blocker_to_interior_loop_then_move_loaded'
    )


def _clearance_step_enters_interior_loop(step: dict[str, Any]) -> bool:
    return str(step.get('clear_target') or step.get('target') or '').strip().casefold() in {
        'interior_loop',
        'interior',
        'loop',
    }


def _clearance_step_shuttle(step: dict[str, Any]) -> str:
    shuttle = _clean_symbol(step.get('shuttle') or step.get('blocker_shuttle')).lower()
    _require_clearance_step_value(shuttle, step=step, field='shuttle')
    return shuttle


def _clearance_step_start_slot(
    spec: ScenarioSpec,
    step: dict[str, Any],
    *,
    shuttle: str,
) -> str:
    raw_slot = step.get('start_slot') or step.get('blocker_start_slot')
    slot = _slot_symbol_or_empty(raw_slot)
    if slot:
        return slot
    start_slots = {
        _clean_symbol(raw_shuttle).lower(): _slot_symbol_or_empty(raw_start_slot)
        for raw_shuttle, raw_start_slot in spec.start_slots_by_shuttle
    }
    slot = start_slots.get(shuttle, '')
    _require_clearance_step_value(slot, step=step, field='start_slot')
    return slot


def _clearance_step_gate_stopper(spec: ScenarioSpec, step: dict[str, Any]) -> str:
    raw_stopper = _stopper_symbol_or_empty(step.get('clear_stopper') or step.get('gate_stopper'))
    if _clearance_step_enters_interior_loop(step):
        return _interior_loop_gate_stopper(spec.side, raw_stopper)
    return raw_stopper or 'A3'


def _clearance_step_clear_sensor(spec: ScenarioSpec, step: dict[str, Any]) -> str:
    if _clearance_step_enters_interior_loop(step):
        gate_stopper = _clearance_step_gate_stopper(spec, step)
        return _interior_loop_entry_sensor(spec.side, gate_stopper)
    return str(
        step.get('clear_sensor')
        or ('DA3IR' if spec.side == 'right' else 'DA3IL')
    ).strip()


def _interior_loop_gate_stopper(side: str, raw_stopper: Any = '') -> str:
    default_stopper = INTERIOR_LOOP_GATE_BY_SIDE.get(side, 'A3')
    stopper = _stopper_symbol_or_empty(raw_stopper)
    if not stopper:
        return default_stopper
    if (side, stopper) in INTERIOR_LOOP_CLEAR_POSE_BY_SIDE_AND_GATE:
        return stopper
    return default_stopper


def _interior_loop_entry_sensor(side: str, gate_stopper: str) -> str:
    return INTERIOR_LOOP_ENTRY_SENSOR_BY_SIDE_AND_GATE.get(
        (side, gate_stopper),
        'DA3IR' if side == 'right' else 'DA3IL',
    )


def _interior_loop_clear_pose(side: str, gate_stopper: str) -> tuple[str, float]:
    return INTERIOR_LOOP_CLEAR_POSE_BY_SIDE_AND_GATE.get(
        (side, gate_stopper),
        ('A34I' if side == 'right' else 'A34I', 0.7083),
    )


def _require_clearance_step_value(
    value: Any,
    *,
    step: dict[str, Any],
    field: str,
    message: str = '',
) -> None:
    if str(value or '').strip():
        return
    shuttle = step.get('shuttle') or step.get('blocker_shuttle') or '<unknown>'
    detail = message or f'missing required clearance step field {field!r}'
    raise ValueError(f'clearance step for {shuttle!r}: {detail}')


def oracle_interior_loop_clear_symbolic_plan_for_spec(
    spec: ScenarioSpec,
    *,
    speed: float,
) -> list[str]:
    """Oracle-only interior-loop fixture plan retained for tests."""

    blocker_source = _station_for_slot(spec.side, spec.blocker_start_slot)
    selected_source = spec.source
    selected_target = spec.target
    gate_stopper = _interior_loop_gate_stopper(spec.side, spec.blocker_clear_stopper)
    speed_text = f'{float(speed):.4g}'
    return [
        f'set_stoppers {spec.side} {gate_stopper} closed',
        (
            f'move_shuttle {spec.side} {spec.blocker_shuttle} '
            f'{blocker_source} {blocker_source} speed={speed_text} '
            f'target_stopper={gate_stopper}'
        ),
        f'stop_shuttle {spec.side} {spec.blocker_shuttle}',
        (
            f'prepare_switches {spec.side} {blocker_source} {blocker_source} '
            f'switch={gate_stopper} state=INTERIOR'
        ),
        f'open_stoppers {spec.side} {blocker_source} {blocker_source}',
        (
            f'move_shuttle {spec.side} {spec.blocker_shuttle} '
            f'{blocker_source} {blocker_source} speed={speed_text}'
        ),
        f'stop_shuttle {spec.side} {spec.blocker_shuttle}',
        (
            f'prepare_switches {spec.side} {selected_source} {selected_target} '
            f'switch={gate_stopper} state=EXTERIOR'
        ),
        f'open_stoppers {spec.side} {selected_source} {selected_target}',
        (
            f'move_shuttle {spec.side} {spec.shuttle} '
            f'{selected_source} {selected_target} speed={speed_text}'
        ),
        f'stop_shuttle {spec.side} {spec.shuttle}',
        f'finish_task {spec.shuttle} {selected_target}',
    ]


def oracle_interior_loop_clear_symbolic_steps_for_clearance_step(
    spec: ScenarioSpec,
    step: dict[str, Any],
    *,
    speed_text: str,
) -> list[str]:
    """Oracle-only interior-loop clearance fixture steps retained for tests."""

    shuttle = _clearance_step_shuttle(step)
    source_slot = _clearance_step_start_slot(spec, step, shuttle=shuttle)
    source = _station_for_slot(spec.side, source_slot)
    gate_stopper = _clearance_step_gate_stopper(spec, step)
    _require_clearance_step_value(
        source,
        step=step,
        field='start_slot',
        message=f'could not map slot {source_slot!r} to a station on {spec.side!r} rail',
    )
    return [
        f'set_stoppers {spec.side} {gate_stopper} closed',
        (
            f'move_shuttle {spec.side} {shuttle} {source} {source} '
            f'speed={speed_text} target_stopper={gate_stopper}'
        ),
        f'stop_shuttle {spec.side} {shuttle}',
        (
            f'prepare_switches {spec.side} {source} {source} '
            f'switch={gate_stopper} state=INTERIOR'
        ),
        f'open_stoppers {spec.side} {source} {source}',
        f'move_shuttle {spec.side} {shuttle} {source} {source} speed={speed_text}',
        f'stop_shuttle {spec.side} {shuttle}',
        (
            f'prepare_switches {spec.side} {source} {source} '
            f'switch={gate_stopper} state=EXTERIOR'
        ),
    ]


def oracle_slot_clear_symbolic_steps_for_clearance_step(
    spec: ScenarioSpec,
    step: dict[str, Any],
    *,
    speed_text: str,
) -> list[str]:
    """Oracle-only slot-clearance fixture steps retained for tests."""

    shuttle = _clearance_step_shuttle(step)
    source_slot = _clearance_step_start_slot(spec, step, shuttle=shuttle)
    clear_slot = _slot_symbol_or_empty(step.get('clear_slot') or step.get('target_slot'))
    _require_clearance_step_value(clear_slot, step=step, field='clear_slot')
    source = _station_for_slot(spec.side, source_slot)
    target = _station_for_slot(spec.side, clear_slot)
    _require_clearance_step_value(
        source,
        step=step,
        field='start_slot',
        message=f'could not map slot {source_slot!r} to a station on {spec.side!r} rail',
    )
    _require_clearance_step_value(
        target,
        step=step,
        field='clear_slot',
        message=f'could not map slot {clear_slot!r} to a station on {spec.side!r} rail',
    )
    target_stopper = _stopper_symbol_or_empty(step.get('clear_stopper'))
    move_kwargs = f'speed={speed_text}'
    if target_stopper:
        move_kwargs = f'{move_kwargs} target_stopper={target_stopper}'
        stopper_step = f'set_stoppers {spec.side} {target_stopper} closed'
    else:
        stopper_step = f'open_stoppers {spec.side} {source} {target}'
    return [
        f'prepare_switches {spec.side} {source} {target}',
        stopper_step,
        f'move_shuttle {spec.side} {shuttle} {source} {target} {move_kwargs}',
        f'stop_shuttle {spec.side} {shuttle}',
    ]

def _slot_target_plan_step_metadata(spec: ScenarioSpec, plan: list[str]) -> list[dict[str, Any]]:
    metadata = []
    for step in plan:
        text = str(step or '').strip().casefold()
        if text.startswith('move_shuttle '):
            metadata.append({
                'coordination_phase': 'move_selected_loaded',
                'plan_step_target_shuttle_id': spec.shuttle,
                'selected_shuttle_id': spec.shuttle,
                'target_slot': spec.target_slot,
                'target_station': spec.target,
                'selection_policy': spec.selection_policy,
            })
        elif text.startswith('stop_shuttle '):
            metadata.append({
                'coordination_phase': 'stop_selected_loaded',
                'plan_step_target_shuttle_id': spec.shuttle,
                'selected_shuttle_id': spec.shuttle,
                'target_slot': spec.target_slot,
                'target_station': spec.target,
                'selection_policy': spec.selection_policy,
            })
        else:
            metadata.append({
                'coordination_phase': 'prepare_selected_loaded'
                if not text.startswith('finish_task ')
                else 'complete_selected_loaded_task',
                'plan_step_target_shuttle_id': spec.shuttle,
                'selected_shuttle_id': spec.shuttle,
                'target_station': spec.target,
                'selection_policy': spec.selection_policy,
            })
    return metadata


def _blocker_clearance_metadata_for_spec(spec: ScenarioSpec) -> dict[str, Any]:
    restore_requested = spec.blocker_restore_policy not in {
        'none',
        'no_restore',
        'skip_restore',
        'clear_only',
    }
    blocker_clear_target = spec.blocker_clear_target
    blocker_clear_sensor = spec.blocker_clear_sensor
    blocker_clear_stopper = spec.blocker_clear_stopper
    if _blocker_enters_interior_loop(spec):
        blocker_clear_target = blocker_clear_target or 'interior_loop'
        blocker_clear_stopper = _interior_loop_gate_stopper(
            spec.side,
            spec.blocker_clear_stopper,
        )
        blocker_clear_sensor = _interior_loop_entry_sensor(spec.side, blocker_clear_stopper)
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
        'blocker_clear_target': blocker_clear_target,
        'blocker_clear_sensor': blocker_clear_sensor,
        'blocker_clear_stopper': blocker_clear_stopper,
        'blocker_restore_slot': spec.blocker_restore_slot,
        'blocker_final_slot': spec.blocker_restore_slot or spec.blocker_clear_slot,
        'blocker_final_target': (
            spec.blocker_restore_slot
            or spec.blocker_clear_slot
            or blocker_clear_target
        ),
        'blocker_restore_policy': spec.blocker_restore_policy,
        'blocker_restore_slot_source': spec.blocker_restore_slot_source,
        'blocker_restore_candidate_slots': list(spec.blocker_restore_candidate_slots),
        'selected_shuttle_id': spec.shuttle,
        'selected_target_slot': spec.target_slot,
        'restore_deferred': restore_requested and not bool(spec.blocker_restore_slot),
        'model_input_exposure': 'excluded',
    }


def _blocker_clear_plan_step_metadata(spec: ScenarioSpec) -> list[dict[str, Any]]:
    if _blocker_enters_interior_loop(spec):
        return _interior_loop_clear_plan_step_metadata(spec)

    blocker_prepare = {
        'coordination_phase': 'clear_blocker',
        'plan_step_target_shuttle_id': spec.blocker_shuttle,
        'blocker_shuttle_id': spec.blocker_shuttle,
        'target_station': _station_for_slot(spec.side, spec.blocker_clear_slot),
        'clearance_strategy': spec.clearance_strategy,
    }
    if spec.blocker_clear_sensor:
        blocker_prepare['target_sensors'] = [spec.blocker_clear_sensor]
    if spec.blocker_clear_stopper:
        blocker_prepare['target_stopper'] = spec.blocker_clear_stopper
    blocker_move = dict(blocker_prepare)
    if not spec.blocker_clear_sensor:
        blocker_move['target_slot'] = spec.blocker_clear_slot
    selected_route = {
        'coordination_phase': 'move_selected_loaded',
        'plan_step_target_shuttle_id': spec.shuttle,
        'selected_shuttle_id': spec.shuttle,
        'target_station': spec.target,
        'clearance_strategy': spec.clearance_strategy,
    }
    selected_move = dict(selected_route)
    selected_move['target_slot'] = spec.target_slot
    steps = [
        dict(blocker_prepare),
        dict(blocker_prepare),
        dict(blocker_move),
        dict(blocker_move),
        dict(selected_route),
        dict(selected_route),
        dict(selected_move),
        dict(selected_move),
    ]
    if spec.blocker_restore_slot:
        restore_prepare = {
            'coordination_phase': 'restore_blocker',
            'plan_step_target_shuttle_id': spec.blocker_shuttle,
            'blocker_shuttle_id': spec.blocker_shuttle,
            'target_station': _station_for_slot(spec.side, spec.blocker_restore_slot),
            'clearance_strategy': spec.clearance_strategy,
        }
        restore_move = dict(restore_prepare)
        restore_move['target_slot'] = spec.blocker_restore_slot
        steps.extend([
            dict(restore_prepare),
            dict(restore_prepare),
            dict(restore_move),
            dict(restore_move),
        ])
    steps.append({
        'coordination_phase': 'complete_selected_loaded_task',
        'plan_step_target_shuttle_id': spec.shuttle,
        'selected_shuttle_id': spec.shuttle,
        'target_station': spec.target,
        'clearance_strategy': spec.clearance_strategy,
    })
    return steps


def _interior_loop_clear_plan_step_metadata(spec: ScenarioSpec) -> list[dict[str, Any]]:
    gate_stopper = _interior_loop_gate_stopper(spec.side, spec.blocker_clear_stopper)
    gate_sensor = f'{gate_stopper}_STOPPER_SENSOR'
    interior_sensor = _interior_loop_entry_sensor(spec.side, gate_stopper)
    blocker_source = _station_for_slot(spec.side, spec.blocker_start_slot)
    clear_segment, clear_s = _interior_loop_clear_pose(spec.side, gate_stopper)
    gate_route = {
        'coordination_phase': 'clear_blocker_to_gate',
        'plan_step_target_shuttle_id': spec.blocker_shuttle,
        'blocker_shuttle_id': spec.blocker_shuttle,
        'target_station': blocker_source,
        'target_sensors': [gate_sensor],
        'target_stopper': gate_stopper,
        'blocker_clear_target': 'interior_loop',
        'clearance_strategy': spec.clearance_strategy,
    }
    interior_route = {
        'coordination_phase': 'clear_blocker_to_interior_loop',
        'plan_step_target_shuttle_id': spec.blocker_shuttle,
        'blocker_shuttle_id': spec.blocker_shuttle,
        'target_station': 'interior_loop',
        'target_segment': clear_segment,
        'target_s': clear_s,
        'target_tolerance_m': INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M,
        'entry_sensor': interior_sensor,
        'blocker_clear_target': 'interior_loop',
        'clearance_strategy': spec.clearance_strategy,
    }
    selected_route = {
        'coordination_phase': 'move_selected_loaded',
        'plan_step_target_shuttle_id': spec.shuttle,
        'selected_shuttle_id': spec.shuttle,
        'target_station': spec.target,
        'clearance_strategy': spec.clearance_strategy,
    }
    selected_move = dict(selected_route)
    selected_move['target_slot'] = spec.target_slot
    return [
        dict(gate_route),
        dict(gate_route),
        dict(gate_route),
        dict(interior_route),
        dict(interior_route),
        dict(interior_route),
        dict(interior_route),
        dict(selected_route),
        dict(selected_route),
        dict(selected_move),
        dict(selected_move),
        {
            'coordination_phase': 'complete_selected_loaded_task',
            'plan_step_target_shuttle_id': spec.shuttle,
            'selected_shuttle_id': spec.shuttle,
            'target_station': spec.target,
            'clearance_strategy': spec.clearance_strategy,
        },
    ]


def _multi_blocker_clearance_metadata_for_spec(spec: ScenarioSpec) -> dict[str, Any]:
    clearance_steps = []
    for index, step in enumerate(spec.clearance_steps, start=1):
        shuttle = _clearance_step_shuttle(step)
        start_slot = _clearance_step_start_slot(spec, step, shuttle=shuttle)
        if _clearance_step_enters_interior_loop(step):
            clear_target = 'interior_loop'
            clear_slot = ''
            final_target = clear_target
            final_slot = ''
            clear_stopper = _clearance_step_gate_stopper(spec, step)
            clear_sensor = _clearance_step_clear_sensor(spec, step)
        else:
            clear_target = ''
            clear_slot = _slot_symbol_or_empty(step.get('clear_slot') or step.get('target_slot'))
            final_target = clear_slot
            final_slot = clear_slot
            clear_stopper = _stopper_symbol_or_empty(step.get('clear_stopper'))
            clear_sensor = str(step.get('clear_sensor') or '').strip()
        clearance_steps.append({
            'step_index': index,
            'blocker_shuttle_id': shuttle,
            'blocker_start_slot': start_slot,
            'blocker_clear_slot': clear_slot,
            'blocker_clear_target': clear_target,
            'blocker_clear_sensor': clear_sensor,
            'blocker_clear_stopper': clear_stopper,
            'blocker_final_slot': final_slot,
            'blocker_final_target': final_target,
        })

    return {
        'strategy': spec.clearance_strategy,
        'phase': 'clear_multiple_blockers_then_move_selected',
        'clearance_step_count': len(clearance_steps),
        'clearance_steps': clearance_steps,
        'selected_shuttle_id': spec.shuttle,
        'selected_target_slot': spec.target_slot,
        'blocker_restore_policy': 'none',
        'model_input_exposure': 'excluded',
    }


def _multi_blocker_clear_plan_step_metadata(spec: ScenarioSpec) -> list[dict[str, Any]]:
    steps: list[dict[str, Any]] = []
    for index, step in enumerate(spec.clearance_steps, start=1):
        if _clearance_step_enters_interior_loop(step):
            steps.extend(_multi_interior_step_metadata(spec, step, step_index=index))
        else:
            steps.extend(_multi_slot_clear_step_metadata(spec, step, step_index=index))

    selected_route = {
        'coordination_phase': 'move_selected_loaded',
        'plan_step_target_shuttle_id': spec.shuttle,
        'selected_shuttle_id': spec.shuttle,
        'target_station': spec.target,
        'clearance_strategy': spec.clearance_strategy,
    }
    selected_move = dict(selected_route)
    selected_move['target_slot'] = spec.target_slot
    steps.extend([
        dict(selected_route),
        dict(selected_route),
        dict(selected_move),
        dict(selected_move),
        {
            'coordination_phase': 'complete_selected_loaded_task',
            'plan_step_target_shuttle_id': spec.shuttle,
            'selected_shuttle_id': spec.shuttle,
            'target_station': spec.target,
            'clearance_strategy': spec.clearance_strategy,
        },
    ])
    return steps


def _multi_interior_step_metadata(
    spec: ScenarioSpec,
    step: dict[str, Any],
    *,
    step_index: int,
) -> list[dict[str, Any]]:
    shuttle = _clearance_step_shuttle(step)
    source_slot = _clearance_step_start_slot(spec, step, shuttle=shuttle)
    source = _station_for_slot(spec.side, source_slot)
    gate_stopper = _clearance_step_gate_stopper(spec, step)
    gate_sensor = f'{gate_stopper}_STOPPER_SENSOR'
    interior_sensor = _clearance_step_clear_sensor(spec, step)
    clear_segment, clear_s = _interior_loop_clear_pose(spec.side, gate_stopper)
    gate_route = {
        'coordination_phase': 'clear_blocker_to_gate',
        'clearance_step_index': step_index,
        'plan_step_target_shuttle_id': shuttle,
        'blocker_shuttle_id': shuttle,
        'blocker_start_slot': source_slot,
        'target_station': source,
        'target_sensors': [gate_sensor],
        'target_stopper': gate_stopper,
        'blocker_clear_target': 'interior_loop',
        'clearance_strategy': spec.clearance_strategy,
    }
    interior_route = {
        'coordination_phase': 'clear_blocker_to_interior_loop',
        'clearance_step_index': step_index,
        'plan_step_target_shuttle_id': shuttle,
        'blocker_shuttle_id': shuttle,
        'blocker_start_slot': source_slot,
        'target_station': 'interior_loop',
        'target_segment': clear_segment,
        'target_s': clear_s,
        'target_tolerance_m': INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M,
        'entry_sensor': interior_sensor,
        'blocker_clear_target': 'interior_loop',
        'clearance_strategy': spec.clearance_strategy,
    }
    reset_switch = {
        'coordination_phase': 'restore_switch_after_interior_clear',
        'clearance_step_index': step_index,
        'plan_step_target_shuttle_id': shuttle,
        'blocker_shuttle_id': shuttle,
        'target_station': source,
        'blocker_clear_target': 'interior_loop',
        'clearance_strategy': spec.clearance_strategy,
    }
    return [
        dict(gate_route),
        dict(gate_route),
        dict(gate_route),
        dict(interior_route),
        dict(interior_route),
        dict(interior_route),
        dict(interior_route),
        reset_switch,
    ]


def _multi_slot_clear_step_metadata(
    spec: ScenarioSpec,
    step: dict[str, Any],
    *,
    step_index: int,
) -> list[dict[str, Any]]:
    shuttle = _clearance_step_shuttle(step)
    source_slot = _clearance_step_start_slot(spec, step, shuttle=shuttle)
    clear_slot = _slot_symbol_or_empty(step.get('clear_slot') or step.get('target_slot'))
    prepare = {
        'coordination_phase': 'clear_blocker_to_slot',
        'clearance_step_index': step_index,
        'plan_step_target_shuttle_id': shuttle,
        'blocker_shuttle_id': shuttle,
        'blocker_start_slot': source_slot,
        'target_station': _station_for_slot(spec.side, clear_slot),
        'clearance_strategy': spec.clearance_strategy,
    }
    clear_sensor = str(step.get('clear_sensor') or '').strip()
    clear_stopper = _stopper_symbol_or_empty(step.get('clear_stopper'))
    if clear_sensor:
        prepare['target_sensors'] = [clear_sensor]
    if clear_stopper:
        prepare['target_stopper'] = clear_stopper
    move = dict(prepare)
    if not clear_sensor:
        move['target_slot'] = clear_slot
    return [
        dict(prepare),
        dict(prepare),
        dict(move),
        dict(move),
    ]


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
    spec: ScenarioSpec | None,
    problem: Room315PddlProblem,
    speed: float,
) -> list[str]:
    raw_actions = _plansys_action_strings(plan_msg)
    plan = [
        _canonical_symbolic_step(step, spec=spec, problem=problem, speed=speed)
        for step in raw_actions
    ]
    rejected = [
        str(raw).strip() or '<empty>'
        for raw, canonical in zip(raw_actions, plan)
        if not canonical
    ]
    if rejected:
        raise RuntimeError(
            'PlanSys2 returned unsupported or malformed Room 315 actions; '
            'the complete plan is rejected fail-closed: '
            + '; '.join(rejected)
        )
    if not plan:
        raise RuntimeError(
            'PlanSys2 generated no supported Room 315 symbolic plan steps. '
            'Check that the Room 315 domain/problem use the expected actions: '
            f'{", ".join(sorted(RUNTIME_SYMBOLIC_ACTIONS))}.'
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


def _canonical_symbolic_step(
    raw_action: Any,
    *,
    spec: ScenarioSpec | None,
    problem: Room315PddlProblem,
    speed: float,
) -> str:
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
    # PlanSys2 is an online execution boundary.  Legacy fixture spellings
    # remain supported by the offline dataset helpers, but they must never be
    # accepted from the runtime planner because they omit the exact source
    # slot/topology terms needed for applicability and safety checks.
    if step.name not in RUNTIME_SYMBOLIC_ACTIONS:
        return ''
    if step.name == 'set_stoppers':
        side = _side_from_symbols(step.args, default=_problem_side(problem, spec))
        stopper = _stopper_from_symbols(step.args, default='ALL')
        state = _stopper_state_from_step(step)
        return f'set_stoppers {side} {stopper} {state}'
    if step.name == 'wait_for_clearance':
        side = _side_from_symbols(step.args, default=_problem_side(problem, spec))
        shuttle = _shuttle_from_symbols(step.args, default=_problem_shuttle(problem, spec))
        return f'stop_shuttle {side} {shuttle}'
    if step.name == 'inspect_state':
        target = step.args[0] if step.args else 'room315'
        return f'inspect_state {target}'
    if step.name == 'restore_normal_route':
        if len(step.args) != 3:
            return ''
        side, source, target = step.args
        if side not in SIDES:
            return ''
        valid_stations = {
            _station_object(side, station)
            for station in (
                ('yaskawa', 'staubli')
                if side == 'right'
                else ('yaskawa', 'kuka')
            )
        }
        if source not in valid_stations or target not in valid_stations:
            return ''
        return f'restore_normal_route {side} {source} {target}'
    if step.name in {'begin_route_clearance', 'finish_route_clearance'}:
        if len(step.args) != 4:
            return ''
        selected, side, from_slot, to_slot = step.args
        if side not in SIDES:
            return ''
        if normalize_shuttle_ref(selected, side=side) is None:
            return ''
        return (
            f'{step.name} {selected} {side} {from_slot} {to_slot}'
        )
    if step.name in {
        'begin_segment_route_clearance',
        'finish_segment_route_clearance',
    }:
        if len(step.args) != 4:
            return ''
        selected, side, source_block, target_slot = step.args
        if side not in SIDES:
            return ''
        if normalize_shuttle_ref(selected, side=side) is None:
            return ''
        return (
            f'{step.name} {selected} {side} {source_block} {target_slot}'
        )
    if step.name == 'pause_route_clearance':
        if len(step.args) != 1 or step.args[0] not in SIDES:
            return ''
        return f'pause_route_clearance {step.args[0]}'
    if step.name == 'relocate_blocker_to_interior':
        if len(step.args) != 5:
            return ''
        blocker, selected, side, from_slot, to_slot = step.args
        if side not in SIDES:
            return ''
        if normalize_shuttle_ref(blocker, side=side) is None:
            return ''
        if normalize_shuttle_ref(selected, side=side) is None:
            return ''
        speed_text = (
            step.kwargs.get('speed')
            or step.kwargs.get('speed_mps')
            or f'{float(speed):.4g}'
        )
        return (
            f'relocate_blocker_to_interior {blocker} {selected} {side} '
            f'{from_slot} {to_slot} speed={speed_text}'
        )
    if step.name == 'stage_selected_to_interior':
        if len(step.args) != 4:
            return ''
        selected, side, from_slot, to_slot = step.args
        if side not in SIDES:
            return ''
        if normalize_shuttle_ref(selected, side=side) is None:
            return ''
        speed_text = (
            step.kwargs.get('speed')
            or step.kwargs.get('speed_mps')
            or f'{float(speed):.4g}'
        )
        return (
            f'stage_selected_to_interior {selected} {side} '
            f'{from_slot} {to_slot} speed={speed_text}'
        )
    if step.name == 'relocate_segment_blocker_to_interior':
        if len(step.args) != 5:
            return ''
        blocker, selected, side, source_block, target_slot = step.args
        if side not in SIDES:
            return ''
        if normalize_shuttle_ref(blocker, side=side) is None:
            return ''
        if normalize_shuttle_ref(selected, side=side) is None:
            return ''
        speed_text = (
            step.kwargs.get('speed')
            or step.kwargs.get('speed_mps')
            or f'{float(speed):.4g}'
        )
        return (
            'relocate_segment_blocker_to_interior '
            f'{blocker} {selected} {side} {source_block} {target_slot} '
            f'speed={speed_text}'
        )
    if step.name == 'stage_selected_segment_to_interior':
        if len(step.args) != 4:
            return ''
        selected, side, source_block, target_slot = step.args
        if side not in SIDES:
            return ''
        if normalize_shuttle_ref(selected, side=side) is None:
            return ''
        speed_text = (
            step.kwargs.get('speed')
            or step.kwargs.get('speed_mps')
            or f'{float(speed):.4g}'
        )
        return (
            'stage_selected_segment_to_interior '
            f'{selected} {side} {source_block} {target_slot} '
            f'speed={speed_text}'
        )
    if step.name == 'prepare_topology_route':
        if len(step.args) != 5:
            return ''
        shuttle, side, source_block, target_slot, switch_group = step.args
        if side not in SIDES or normalize_shuttle_ref(shuttle, side=side) is None:
            return ''
        return (
            f'prepare_topology_route {shuttle} {side} '
            f'{source_block} {target_slot} {switch_group}'
        )
    if step.name == 'prepare_slot_topology_route':
        if len(step.args) != 6:
            return ''
        (
            shuttle,
            side,
            source_slot,
            source_block,
            target_slot,
            switch_group,
        ) = step.args
        if side not in SIDES or normalize_shuttle_ref(shuttle, side=side) is None:
            return ''
        return (
            f'prepare_slot_topology_route {shuttle} {side} {source_slot} '
            f'{source_block} {target_slot} {switch_group}'
        )
    if step.name == 'move_shuttle_from_segment_to_slot':
        if len(step.args) != 5:
            return ''
        shuttle, side, source_block, target_station, target_slot = step.args
        if side not in SIDES or normalize_shuttle_ref(shuttle, side=side) is None:
            return ''
        speed_text = (
            step.kwargs.get('speed')
            or step.kwargs.get('speed_mps')
            or f'{float(speed):.4g}'
        )
        return (
            f'move_shuttle_from_segment_to_slot {shuttle} {side} '
            f'{source_block} {target_station} {target_slot} '
            f'speed={speed_text}'
        )
    if step.name == 'move_shuttle_via_topology_to_slot':
        if len(step.args) != 7:
            return ''
        (
            shuttle,
            side,
            source_slot,
            source_block,
            source_station,
            target_station,
            target_slot,
        ) = step.args
        if side not in SIDES or normalize_shuttle_ref(shuttle, side=side) is None:
            return ''
        speed_text = (
            step.kwargs.get('speed')
            or step.kwargs.get('speed_mps')
            or f'{float(speed):.4g}'
        )
        return (
            f'move_shuttle_via_topology_to_slot {shuttle} {side} '
            f'{source_slot} {source_block} {source_station} {target_station} '
            f'{target_slot} '
            f'speed={speed_text}'
        )
    if step.name == 'move_shuttle_to_slot':
        if len(step.args) != 6:
            return ''
        shuttle, side, source, target, source_slot, target_slot = step.args
        if side not in SIDES or normalize_shuttle_ref(shuttle, side=side) is None:
            return ''
        speed_text = (
            step.kwargs.get('speed')
            or step.kwargs.get('speed_mps')
            or f'{float(speed):.4g}'
        )
        return (
            f'move_shuttle_to_slot {shuttle} {side} {source} {target} '
            f'{source_slot} {target_slot} speed={speed_text}'
        )
    if step.name == 'prepare_switches':
        if len(step.args) != 4:
            return ''
        return f'prepare_switches {" ".join(step.args)}'
    if step.name == 'open_stoppers':
        if len(step.args) != 4:
            return ''
        return f'open_stoppers {" ".join(step.args)}'
    if step.name == 'stop_shuttle':
        if len(step.args) != 4:
            return ''
        return f'stop_shuttle {" ".join(step.args)}'
    if step.name == 'finish_task':
        if len(step.args) != 2:
            return ''
        return f'finish_task {" ".join(step.args)}'
    if step.name == 'finish_candidate_task':
        if len(step.args) != 3:
            return ''
        return f'finish_candidate_task {" ".join(step.args)}'
    side, shuttle, source, target = _route_parts_from_step(step, spec, problem)
    if step.name == 'move_shuttle':
        step_speed = step.kwargs.get('speed') or step.kwargs.get('speed_mps')
        speed_text = step_speed if step_speed is not None else f'{float(speed):.4g}'
        target_stopper = _stopper_symbol_or_empty(
            step.kwargs.get('target_stopper') or step.kwargs.get('stopper_target')
        )
        stopper_text = f' target_stopper={target_stopper}' if target_stopper else ''
        return f'move_shuttle {side} {shuttle} {source} {target} speed={speed_text}{stopper_text}'
    return ''


def _route_parts_from_step(
    step: Any,
    spec: ScenarioSpec | None,
    problem: Room315PddlProblem,
) -> tuple[str, str, str, str]:
    side = _side_from_symbols(step.args, default=_problem_side(problem, spec))
    shuttle = _shuttle_from_symbols(step.args, default=_problem_shuttle(problem, spec))
    stations = _stations_from_symbols(step.args)
    source = stations[0] if len(stations) > 0 else _problem_source_station(problem, spec)
    target = stations[1] if len(stations) > 1 else _problem_target_station(problem, spec)
    if step.name in {'finish_task', 'finish_candidate_task'} and len(stations) == 1:
        source = _problem_source_station(problem, spec)
        target = stations[0]
    return side, shuttle, source, target


def _problem_side(problem: Room315PddlProblem, spec: ScenarioSpec | None) -> str:
    return problem.side or (spec.side if spec is not None else 'right')


def _problem_shuttle(problem: Room315PddlProblem, spec: ScenarioSpec | None) -> str:
    return problem.selected_shuttle or (spec.shuttle if spec is not None else 'right_shuttle_1')


def _problem_source_station(problem: Room315PddlProblem, spec: ScenarioSpec | None) -> str:
    return spec.source if spec is not None else ''


def _problem_target_station(problem: Room315PddlProblem, spec: ScenarioSpec | None) -> str:
    if problem.target_station:
        return _station_symbol(problem.target_station)
    return spec.target if spec is not None else ''


def _stopper_from_symbols(symbols: tuple[str, ...], *, default: str) -> str:
    for symbol in symbols:
        stopper = _stopper_symbol_or_empty(symbol)
        if stopper:
            return stopper
    return default


def _stopper_state_from_step(step: Any) -> str:
    for key in ('state', 'stopper_state', 'value'):
        if key in step.kwargs:
            return _stopper_state_symbol(step.kwargs[key])
    for symbol in step.args:
        if _side_from_symbols((symbol,), default='') or _stopper_symbol_or_empty(symbol):
            continue
        state = _stopper_state_symbol(symbol, default='')
        if state:
            return state
    return 'open'


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
        if text in {'right_shuttle', 'left_shuttle'}:
            return f'{text}_1'
        if re.fullmatch(r'(?:right|left)_shuttle_[1-4]', text):
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


def _route_topology_metadata_for_spec(spec: ScenarioSpec) -> dict[str, Any]:
    target_slot = _slot_symbol_or_empty(spec.target_slot)
    start_slots = {
        _canonical_planning_shuttle_id(shuttle, side=spec.side): _slot_symbol_or_empty(slot)
        for shuttle, slot in spec.start_slots_by_shuttle
    }
    source_slot = start_slots.get(_canonical_planning_shuttle_id(spec.shuttle, side=spec.side), '')
    if not target_slot or not source_slot:
        return {}

    topology = load_rail_topology(
        RAIL_NETWORK_PATH_BY_SIDE[spec.side],
        RAIL_DEVICES_PATH_BY_SIDE[spec.side],
        side=spec.side,
    )
    route_blocks = route_blocks_between_slots(
        topology,
        source_slot,
        target_slot,
    )
    rails = _synthetic_rails_from_start_slots(spec, topology)
    blockers = route_blockers_from_rails(
        rails,
        topology,
        source_slot,
        target_slot,
        selected_shuttle=spec.shuttle,
    )
    return {
        'source': 'room_315_kinematics_topology',
        'side': spec.side,
        'selected_shuttle_id': _canonical_planning_shuttle_id(spec.shuttle, side=spec.side),
        'source_slot': source_slot,
        'target_slot': target_slot,
        'default_switch_state': topology.default_switch_state,
        'route_blocks': [
            {
                'block_id': block.block_id,
                'segment': block.segment,
                'start_s_ratio': round(float(block.start_s_ratio), 6),
                'end_s_ratio': round(float(block.end_s_ratio), 6),
            }
            for block in route_blocks
        ],
        'route_blocker_count': len(blockers),
        'route_blockers': [
            _route_blocker_metadata(blocker, side=spec.side, start_slots=start_slots)
            for blocker in blockers
        ],
        'model_input_exposure': 'excluded',
    }


def _synthetic_rails_from_start_slots(spec: ScenarioSpec, topology: Any) -> dict[str, Any]:
    shuttles = {}
    for raw_shuttle, raw_slot in spec.start_slots_by_shuttle:
        shuttle_id = _canonical_planning_shuttle_id(raw_shuttle, side=spec.side)
        slot = _slot_symbol_or_empty(raw_slot)
        location = topology.slots.get(slot)
        if not shuttle_id or location is None:
            continue
        shuttles[_gazebo_entity_for_planning_shuttle(shuttle_id)] = {
            'segment': location.segment,
            's_ratio': location.s_ratio,
            'start_slot': slot,
        }
    return {spec.side: {'shuttles': shuttles}}


def _route_blocker_metadata(blocker: Any, *, side: str, start_slots: dict[str, str]) -> dict[str, Any]:
    shuttle_id = _canonical_planning_shuttle_id(blocker.shuttle_id, side=side)
    entry = {
        'shuttle_id': shuttle_id,
        'side': side,
        'segment': blocker.segment,
        's_ratio': (
            round(float(blocker.s_ratio), 6)
            if blocker.s_ratio is not None
            else None
        ),
        'block_id': blocker.block_id,
        'reason': blocker.reason,
        'occupancy_start_s_ratio': (
            round(float(blocker.occupancy_start_s_ratio), 6)
            if blocker.occupancy_start_s_ratio is not None
            else None
        ),
        'occupancy_end_s_ratio': (
            round(float(blocker.occupancy_end_s_ratio), 6)
            if blocker.occupancy_end_s_ratio is not None
            else None
        ),
    }
    if blocker.shuttle_id != shuttle_id:
        entry['short_shuttle_id'] = blocker.shuttle_id
    start_slot = start_slots.get(shuttle_id, '')
    if start_slot:
        entry['start_slot'] = start_slot
    return entry


def _canonical_planning_shuttle_id(raw: Any, *, side: str) -> str:
    ref = normalize_shuttle_ref(raw, side=side)
    if ref is not None:
        return ref.shuttle_id
    return _clean_symbol(str(raw or '')).lower()


def _gazebo_entity_for_planning_shuttle(shuttle_id: str) -> str:
    text = str(shuttle_id or '').strip()
    return text if text.startswith('room315_') else f'room315_{text}'


def _payload_init_facts_for_spec(spec: ScenarioSpec) -> str:
    if spec.loaded_shuttles:
        if spec.shuttle in set(spec.loaded_shuttles):
            return f'    (loaded {spec.shuttle})'
        return f'    (empty {spec.shuttle})'
    if spec.payload_condition == 'loaded':
        return f'    (loaded {spec.shuttle})'
    if spec.payload_condition == 'empty':
        return f'    (empty {spec.shuttle})'
    return f'    (empty {spec.shuttle})'


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


def _optional_float(value: Any) -> float | None:
    if value is None or str(value).strip() == '':
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


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
    has_step_metadata = bool(scenario.get('plan_step_metadata'))
    target_slot = str(
        step_metadata.get('target_slot')
        or ('' if has_step_metadata else scenario.get('target_slot'))
        or ''
    )
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
    return build_pddl_problem_from_spec(spec).problem_text


def build_pddl_problem_from_spec(spec: ScenarioSpec) -> Room315PddlProblem:
    """Build a PlanSys2 problem from a ScenarioSpec via validated contracts."""

    observed_state = _observed_state_from_scenario_spec(spec)
    task_goal = _task_goal_from_scenario_spec(spec)
    return build_pddl_problem_from_observed_state_task_goal(
        observed_state,
        task_goal,
        problem_name=spec.pddl_problem or f'room315-{_clean_symbol(spec.goal_id)}',
    )


def build_pddl_problem_from_observed_state_task_goal(
    observed_state: ObservedState,
    task_goal: TaskGoal,
    *,
    problem_name: str | None = None,
    runtime_clearance_certificates: dict[str, dict[str, Any]] | None = None,
    runtime_payload_grounding: dict[str, Any] | None = None,
) -> Room315PddlProblem:
    """Convert validated planner state and a high-level goal into PDDL.

    The builder is intentionally fail-closed. Missing, unknown, stale, or
    conflicting facts that affect planning raise PddlProblemBuildError instead
    of being emitted as false PDDL predicates.
    """

    if not isinstance(observed_state, ObservedState):
        raise PddlProblemBuildError('observed_state must be an ObservedState contract')
    if not isinstance(task_goal, TaskGoal):
        raise PddlProblemBuildError('task_goal must be a TaskGoal contract')

    constraints = dict(task_goal.constraints or {})
    goal_type = str(constraints.get('goal_type') or '').strip().casefold()
    if goal_type not in {'transport', 'inspection'}:
        raise PddlProblemBuildError(
            f'unsupported TaskGoal goal_type {goal_type!r}; expected transport or inspection'
        )
    side = _normalise_planning_side(constraints.get('side') or 'right')
    problem_name = _pddl_problem_name(
        problem_name
        or f'room315-{_clean_symbol(task_goal.goal_id or observed_state.state_id)}'
    )

    fact_index = _observed_fact_index(observed_state)
    fleet = _fleet_snapshot_from_observed_state(
        fact_index,
        # The two Room 315 rails have independent, side-qualified topology,
        # devices, slots, and shuttle commands.  A transport problem must
        # therefore validate exact-anchor/learned-position agreement on the
        # rail that can contribute actions to this goal, without allowing a
        # transient disagreement on the physically independent rail to veto
        # planning.  Presence, identity, occupancy, and device integrity stay
        # global below.  System inspection deliberately retains the former
        # all-rail validation scope.
        planning_side=side if goal_type == 'transport' else None,
        runtime_clearance_certificates=runtime_clearance_certificates,
    )
    devices = _device_snapshot_from_observed_state(fact_index)
    obstacles = _obstacle_snapshot_from_observed_state(fact_index)
    blocks = _block_snapshot_from_observed_state(fact_index, fleet)
    goal_data = _goal_data_from_task_goal(
        constraints,
        side=side,
        goal_type=goal_type,
        fleet=fleet,
        runtime_payload_grounding=runtime_payload_grounding,
    )
    route_clearance = _route_clearance_snapshot(fleet=fleet)
    route_clearance['normalization'] = _route_normalization_snapshot(
        fleet=fleet,
        devices=devices,
        obstacles=obstacles,
    )
    side_normalization = dict(
        route_clearance['normalization']
        .get('by_side', {})
        .get(side, {})
    )
    route_clearance['topology_routes'] = _topology_route_snapshot(
        fleet=fleet,
        devices=devices,
        obstacles=obstacles,
        goal_data=goal_data,
    )
    configured_clear_goal_route_bindings = [
        binding
        for route in route_clearance['topology_routes'].get('routes', [])
        if (
            binding := _configured_clear_segment_route_binding(
                route,
                side=side,
                purpose='goal_motion',
            )
        ) is not None
    ]
    configured_goal_route = bool(configured_clear_goal_route_bindings)
    # A mixed switch state is not necessarily unfinished clearance.  It can
    # be the exact authoritative route just configured for a segment-origin
    # goal.  Preserve the general normalization requirement for safety
    # auditing, while explicitly proving when normalization must be deferred
    # until after the selected goal motion consumes this route.
    side_normalization.update({
        'configured_clear_goal_route': configured_goal_route,
        'normalization_required_before_goal_motion': bool(
            side_normalization.get('reconfiguration_required')
            and not configured_goal_route
        ),
        'configured_clear_goal_route_bindings': (
            configured_clear_goal_route_bindings
        ),
        'configured_clear_clearance_route': False,
        'configured_clear_clearance_route_bindings': [],
        'configured_clear_motion_route': configured_goal_route,
    })
    route_clearance['normalization']['by_side'][side] = side_normalization
    route_clearance['target_clearance_plan'] = _target_blocker_clearance_plan(
        fleet=fleet,
        devices=devices,
        obstacles=obstacles,
        goal_data=goal_data,
        route_clearance=route_clearance,
    )
    route_candidate_exclusions: set[int] = set()
    progress_route_audit: list[dict[str, Any]] = []
    while True:
        selected_goal_route = (
            route_clearance.get('topology_routes', {})
            .get('by_shuttle_and_slot', {})
            .get((
                goal_data['selected_shuttle'],
                _slot_object(side, goal_data['target_slot']),
            ))
        )
        nonprogress = _direct_blocker_slot_reoccupies_goal_route(
            side=side,
            goal_route=selected_goal_route,
            target_clearance=route_clearance['target_clearance_plan'],
        )
        if nonprogress is None:
            break
        selected_candidate_index = int(
            selected_goal_route['selected_route_candidate_index']
        )
        route_candidate_exclusions.add(selected_candidate_index)
        progress_route_audit.append({
            **nonprogress,
            'excluded_route_candidate_index': selected_candidate_index,
            'policy': (
                'reject_direct_blocker_parking_that_reoccupies_the_same_'
                'selected_goal_route'
            ),
            'controller_position_fields_used_for_localization': False,
        })
        try:
            alternate_routes = _topology_route_snapshot(
                fleet=fleet,
                devices=devices,
                obstacles=obstacles,
                goal_data=goal_data,
                excluded_candidate_indices_by_shuttle={
                    goal_data['selected_shuttle']: set(
                        route_candidate_exclusions
                    ),
                },
            )
        except PddlProblemBuildError as exc:
            raise PddlProblemBuildError(
                'every authoritative topology candidate would relocate a '
                'direct blocker back onto the selected goal route: '
                f'{exc}'
            ) from exc
        route_clearance['topology_routes'] = alternate_routes
        route_clearance['target_clearance_plan'] = (
            _target_blocker_clearance_plan(
                fleet=fleet,
                devices=devices,
                obstacles=obstacles,
                goal_data=goal_data,
                route_clearance=route_clearance,
            )
        )
    if progress_route_audit:
        route_clearance['topology_routes']['provenance'][
            'progress_preserving_route_selection'
        ] = progress_route_audit
        route_clearance['target_clearance_plan'][
            'progress_preserving_route_selection'
        ] = copy.deepcopy(progress_route_audit)

    # Candidate rejection above can change the final goal route. Recompute
    # its configured binding before deciding whether a mixed route must be
    # normalized.
    configured_clear_goal_route_bindings = [
        binding
        for route in route_clearance['topology_routes'].get('routes', [])
        if (
            binding := _configured_clear_segment_route_binding(
                route,
                side=side,
                purpose='goal_motion',
            )
        ) is not None
    ]
    configured_goal_route = bool(configured_clear_goal_route_bindings)
    side_normalization.update({
        'configured_clear_goal_route': configured_goal_route,
        'configured_clear_goal_route_bindings': (
            configured_clear_goal_route_bindings
        ),
    })
    _append_topology_clearance_routes(route_clearance)
    target_clearance = route_clearance['target_clearance_plan']
    configured_clearance_route_bindings = []
    for relocation in list(
        target_clearance.get('ordered_relocations') or []
    )[:1]:
        destination = dict(relocation.get('destination') or {})
        binding = _configured_clear_segment_route_binding(
            destination.get('topology_route'),
            side=side,
            purpose='blocker_clearance_motion',
        )
        if binding is not None:
            configured_clearance_route_bindings.append({
                **binding,
                'relocation_order': int(relocation.get('order') or 1),
            })
    configured_clearance_route = bool(
        configured_clearance_route_bindings
    )
    configured_motion_route = bool(
        configured_goal_route or configured_clearance_route
    )
    side_normalization.update({
        'configured_clear_clearance_route': configured_clearance_route,
        'configured_clear_clearance_route_bindings': (
            configured_clearance_route_bindings
        ),
        'configured_clear_motion_route': configured_motion_route,
        'normalization_required_before_goal_motion': bool(
            side_normalization.get('reconfiguration_required')
            and not configured_motion_route
        ),
        'normalization_required_before_planned_motion': bool(
            side_normalization.get('reconfiguration_required')
            and not configured_motion_route
        ),
    })
    if (
        side_normalization.get('reconfiguration_required')
        and not side_normalization.get('reconfiguration_safe')
        and not configured_motion_route
    ):
        raise PddlProblemBuildError(
            'mixed rail route requires normalization but no certified safe '
            'normalization or already-configured clear planned-motion route '
            'is available: '
            f'{side_normalization.get("reason") or "unknown"}'
        )
    if target_clearance.get('unsupported_if_more_than_two_blockers'):
        raise PddlProblemBuildError(
            'more than two route blockers exceed the physically separated '
            'interior holding-branch capacity'
        )
    unavailable = [
        relocation
        for relocation in target_clearance.get('ordered_relocations') or []
        if str((relocation.get('destination') or {}).get('kind') or '')
        == 'unavailable'
    ]
    if unavailable:
        pause_safe = bool(
            route_clearance.get('normalization', {})
            .get('by_side', {})
            .get(side, {})
            .get('clearance_pause_safe')
        )
        if pause_safe:
            target_clearance['capacity_pause_required'] = True
            target_clearance['capacity_pause_policy'] = (
                'restore certified stopped interior route, reobserve, and '
                'continue with exterior-slot token rotation'
            )
            unavailable = []
    if unavailable:
        unavailable_reasons = sorted({
            str((item.get('destination') or {}).get('reason') or 'unknown')
            for item in unavailable
        })
        raise PddlProblemBuildError(
            'no safe reachable relocation destination is available for all '
            'route blockers: ' + ','.join(unavailable_reasons)
        )

    init_facts = _pddl_init_facts(
        fleet=fleet,
        devices=devices,
        obstacles=obstacles,
        blocks=blocks,
        goal_data=goal_data,
        route_clearance=route_clearance,
    )
    objects = _pddl_objects(
        fleet=fleet,
        devices=devices,
        obstacles=obstacles,
        blocks=blocks,
        goal_data=goal_data,
    )
    goal_text = _pddl_goal_text(goal_data)
    problem_text = _format_pddl_problem(
        problem_name=problem_name,
        objects=objects,
        init_facts=init_facts,
        goal_text=goal_text,
    )
    provenance = {
        'planner': 'PlanSys2',
        'problem_builder': 'observed_state_task_goal_v1',
        'observed_state_id': observed_state.state_id,
        'task_goal_id': task_goal.goal_id,
        'task_goal_source': task_goal.source,
        'unknown_fact_policy': 'fail_closed_or_request_observation_before_planning',
        'planning_scope': {
            'goal_side': side,
            'exact_slot_anchor_visual_consistency': (
                side if goal_type == 'transport' else 'all_rails'
            ),
            'global_integrity_checks_retained': [
                'presence',
                'identity',
                'slot_occupancy',
                'device_state',
                'obstacles',
            ],
            'deferred_out_of_scope_location_issues': list(
                fleet.get('deferred_out_of_scope_location_issues') or []
            ),
            'verified_slot_arrival_certificates': [
                {
                    'shuttle': shuttle,
                    'side': certificate['side'],
                    'slot': certificate['slot'],
                    'sensor': certificate['sensor'],
                    'proof_mode': certificate.get('proof_mode', ''),
                    'categorical_location_scope_only': True,
                    'raw_visual_position_replaced': False,
                    'controller_position_fields_used_for_localization': False,
                }
                for shuttle, certificate in sorted(
                    (
                        fleet.get('verified_slot_arrival_by_shuttle')
                        or {}
                    ).items()
                )
            ],
            'exact_slot_anchor_visual_diagnostics': dict(
                fleet.get('exact_slot_anchor_visual_consistency') or {}
            ),
            'deferred_issue_policy': (
                'diagnostic_only_for_independent_rail;fail_closed_when_that_rail_is_planned'
            ),
        },
        'model_input_exposure': 'excluded',
        'oracle_test_fixture_used': False,
        'symbolic_action_mapping': dict(PDDL_ACTION_TRANSLATION_PROVENANCE),
        'route_clearance': route_clearance['provenance'],
        'route_normalization': route_clearance['normalization'],
        'topology_routes': route_clearance['topology_routes']['provenance'],
        'target_blocker_clearance_plan': route_clearance['target_clearance_plan'],
    }
    if goal_data['selection_policy']:
        provenance['selection_policy'] = goal_data['selection_policy']
        provenance['candidate_shuttles'] = list(goal_data['candidate_shuttles'])
        provenance['eligible_candidate_shuttles'] = list(
            goal_data.get('eligible_candidate_shuttles')
            or goal_data['candidate_shuttles']
        )
        provenance['selection_owner'] = (
            'PlanSys2 via goal_candidate facts and route_cost metric'
            if goal_data['planner_selects_candidate']
            else (
                'TaskGoal explicit grounding'
                if goal_data['selection_policy'] == 'explicit'
                else 'deterministic accepted-visual grounding before PlanSys2'
            )
        )
        provenance['payload_selection_contract'] = {
            'payload_filter': goal_data['payload_filter'],
            'semantics': goal_data['payload_filter_semantics'],
            'selected_shuttle': goal_data['selected_shuttle'],
            'current_visual_prediction': goal_data[
                'selected_payload_prediction'
            ],
            'current_visual_prediction_matches_selection': goal_data[
                'selected_payload_prediction_matches_filter'
            ],
            'raw_visual_prediction_preserved': True,
            'model_prediction_replaced': False,
            'controller_payload_state_used': False,
            'grounding_proof': dict(
                goal_data.get('payload_grounding') or {}
            ),
        }
    if goal_data.get('station_target_resolution'):
        provenance['station_target_resolution'] = dict(
            goal_data['station_target_resolution']
        )
    return Room315PddlProblem(
        problem_name=problem_name,
        problem_text=problem_text,
        goal_text=goal_text,
        goal_type=goal_type,
        side=side,
        target_station=goal_data.get('target_station', ''),
        target_slot=goal_data.get('target_slot', ''),
        selected_shuttle=goal_data.get('selected_shuttle', ''),
        selection_policy=goal_data.get('selection_policy', ''),
        provenance=provenance,
    )


def _replace_problem_goal(
    problem: Room315PddlProblem,
    *,
    problem_name: str,
    goal_text: str,
    error_context: str,
) -> str:
    """Replace one frozen problem name and goal, or fail closed."""

    old_goal = f'  (:goal\n    {problem.goal_text}\n  )'
    new_goal = f'  (:goal\n    {goal_text}\n  )'
    old_name = f'(problem {problem.problem_name})'
    if (
        problem.problem_text.count(old_goal) != 1
        or problem.problem_text.count(old_name) != 1
    ):
        raise PddlProblemBuildError(
            f'could not isolate {error_context} PDDL problem safely'
        )
    return problem.problem_text.replace(old_goal, new_goal, 1).replace(
        old_name,
        f'(problem {problem_name})',
        1,
    )


def _reset_pending_clearances(
    problem_text: str,
    *,
    side: str,
    error_context: str,
) -> str:
    """Suspend a parent route count in a one-action subproblem."""

    pending_pattern = re.compile(
        rf'\(= \(pending_clearances {re.escape(side)}\) \d+\)'
    )
    updated, replacements = pending_pattern.subn(
        f'(= (pending_clearances {side}) 0)',
        problem_text,
        count=1,
    )
    if replacements != 1:
        raise PddlProblemBuildError(
            f'could not isolate {error_context} pending count safely'
        )
    return updated


def _retain_only_target_free_slot(
    problem_text: str,
    *,
    side: str,
    target_slot: str,
    error_context: str,
) -> tuple[str, list[str]]:
    """Withhold alternate waypoints from a one-action parking problem."""

    free_slot_pattern = re.compile(
        rf'(?m)^\s*\(slot_free ({re.escape(side)}_slot_\d+)\)\s*$'
    )
    exposed = free_slot_pattern.findall(problem_text)
    if target_slot not in exposed:
        raise PddlProblemBuildError(
            f'{error_context} target {target_slot!r} is not known free'
        )
    withheld = sorted(slot for slot in exposed if slot != target_slot)
    for withheld_slot in withheld:
        problem_text, replacements = re.subn(
            rf'(?m)^\s*\(slot_free {re.escape(withheld_slot)}\)\s*\n?',
            '',
            problem_text,
            count=1,
        )
        if replacements != 1:
            raise PddlProblemBuildError(
                f'could not withhold {error_context} alternate slot '
                f'{withheld_slot!r}'
            )
    return problem_text, withheld


def build_first_blocker_clearance_problem(
    problem: Room315PddlProblem,
) -> Room315PddlProblem:
    """Build the next one-relocation PlanSys2 problem from frozen provenance.

    The normal transport goal remains untouched.  The closed-loop executive
    invokes this helper only when the state-derived route analysis says that a
    blocker must move first.  Exactly one relocation is planned, followed by a
    fresh visual observation and a complete rebuild of the transport problem.
    """

    provenance = dict(problem.provenance or {})
    clearance = dict(provenance.get('target_blocker_clearance_plan') or {})
    relocations = clearance.get('ordered_relocations') or []
    if not clearance.get('required') or not relocations:
        raise PddlProblemBuildError('blocker-clearance problem requested without a blocker')
    if clearance.get('unsupported_if_more_than_two_blockers'):
        raise PddlProblemBuildError(
            'more than two route blockers require operator recovery'
        )

    relocation = dict(relocations[0])
    raw_blocker = str(relocation.get('shuttle') or '').strip()
    if not raw_blocker:
        raise PddlProblemBuildError('clearance relocation has no blocker identity')
    blocker = _pddl_symbol(raw_blocker)
    destination = dict(relocation.get('destination') or {})
    destination_kind = str(destination.get('kind') or '').strip().casefold()
    if destination_kind == 'slot':
        raw_target_slot = str(destination.get('target_slot') or '').strip()
        if not raw_target_slot:
            raise PddlProblemBuildError('slot clearance relocation has no target slot')
        target_slot = _pddl_symbol(raw_target_slot)
        goal_text = f'(shuttle_at_slot {blocker} {target_slot})'
        phase = 'clear_blocker_to_slot'
    elif destination_kind == 'interior_loop':
        interior_route_proof = dict(
            destination.get('interior_entry_route_proof') or {}
        )
        if (
            interior_route_proof.get('status') != 'clear'
            or interior_route_proof.get('blocking_shuttles')
        ):
            raise PddlProblemBuildError(
                'interior blocker relocation lacks a clear authoritative '
                'entry-route dependency proof'
            )
        goal_text = f'(clearance_relocated {blocker})'
        phase = 'clear_blocker_to_interior_loop'
    else:
        raise PddlProblemBuildError(
            f'unsupported blocker-clearance destination {destination_kind!r}'
        )

    normalization = dict(
        provenance.get('route_normalization', {})
        .get('by_side', {})
        .get(problem.side, {})
    )
    if destination_kind == 'slot' and normalization.get('clearance_mode'):
        raise PddlProblemBuildError(
            'cannot build a normal-route slot relocation while clearance '
            'mode is active; build and execute the certified clearance-pause '
            'subproblem first'
        )
    normalize_before_clearance = bool(
        normalization.get('reconfiguration_required')
        and normalization.get('reconfiguration_safe')
        and not normalization.get('normal_route')
        and not normalization.get('configured_clear_motion_route')
    )
    deferred_clearance_goal = goal_text
    if normalize_before_clearance:
        # The executive executes one atomic action and re-observes. Asking POPF
        # to normalize a mixed topology and solve the future relocation in one
        # search creates an avoidable numeric deadlock/search explosion because
        # pending_clearances belongs to the parent route. Isolate the safe
        # normalization action, suspend the future count, then rebuild every
        # clearance fact from the next accepted observation.
        goal_text = f'(normal_route {problem.side})'
        phase = 'normalize_route_before_blocker_clearance'

    clearance_provenance = dict(provenance)
    clearance_provenance.update({
        'planning_phase': phase,
        'clearance_relocation': relocation,
        'parent_problem_name': problem.problem_name,
    })
    if normalize_before_clearance:
        clearance_provenance.update({
            'deferred_clearance_goal': deferred_clearance_goal,
            'normalization_before_clearance': True,
            'fresh_reobservation_required_after_normalization': True,
        })
    clearance_problem_name = (
        f'{problem.problem_name}-normalization-before-clearance'
        if normalize_before_clearance
        else (
            f'{problem.problem_name}-clearance-'
            f'{int(relocation.get("order") or 1)}'
        )
    )
    problem_text = _replace_problem_goal(
        problem,
        problem_name=clearance_problem_name,
        goal_text=goal_text,
        error_context='blocker-clearance',
    )
    if normalize_before_clearance:
        problem_text = _reset_pending_clearances(
            problem_text,
            side=problem.side,
            error_context='pre-clearance normalization',
        )
        clearance_provenance['parent_pending_clearances_suspended'] = True
    elif destination_kind == 'slot':
        # pending_clearances describes the parent selected-shuttle route. A
        # free-slot parking subgoal is itself a normal-route move and must not
        # be blocked by that parent's pending count. The parent problem is
        # rebuilt from a fresh observation immediately after this one move.
        problem_text = _reset_pending_clearances(
            problem_text,
            side=problem.side,
            error_context='slot-clearance',
        )
        clearance_provenance['parent_pending_clearances_suspended'] = True

        # The executive deliberately executes only the first action from each
        # plan and then re-observes.  If other known-free slots remain exposed,
        # POPF may legally use one of them as an intermediate waypoint instead
        # of moving directly to the audited parking destination.  Withhold
        # those alternatives from this one-action subproblem so its first
        # action has the same destination as the frozen clearance provenance.
        # Withholding a free fact is fail-closed; no occupied slot is presented
        # as free, and the complete state is rebuilt after the move.
        problem_text, withheld_free_slots = _retain_only_target_free_slot(
            problem_text,
            side=problem.side,
            target_slot=target_slot,
            error_context='clearance parking',
        )
        clearance_provenance.update({
            'first_action_destination_constrained': True,
            'parking_target_free_fact_retained': target_slot,
            'temporarily_withheld_known_free_slots': withheld_free_slots,
            'free_fact_policy': (
                'fail_closed_one_action_subproblem_then_fresh_reobservation'
            ),
        })
    return replace(
        problem,
        problem_name=clearance_problem_name,
        problem_text=problem_text,
        goal_text=goal_text,
        target_station=(
            problem.target_station
            if normalize_before_clearance
            else
            SLOT_STATION_BY_SIDE_AND_SLOT[
                (problem.side, _slot_number_from_object(target_slot))
            ]
            if destination_kind == 'slot'
            else problem.target_station
        ),
        target_slot=(
            problem.target_slot
            if normalize_before_clearance
            else
            _slot_number_from_object(target_slot)
            if destination_kind == 'slot'
            else problem.target_slot
        ),
        selected_shuttle=(
            problem.selected_shuttle
            if normalize_before_clearance
            else blocker
        ),
        provenance=clearance_provenance,
    )


def build_intermediate_selected_advance_problem(
    problem: Room315PddlProblem,
) -> Room315PddlProblem:
    """Isolate one safe selected-shuttle advance toward the final slot.

    A full four-shuttle rail sometimes needs a token-rotation sequence: park
    an ahead shuttle, advance the selected shuttle into the newly freed slot,
    then park the remaining blocker behind it.  The user goal remains the
    final slot; this subproblem authorizes exactly one sensor-backed advance
    and the executive rebuilds the final problem from a fresh observation.
    """

    provenance = dict(problem.provenance or {})
    clearance = dict(provenance.get('target_blocker_clearance_plan') or {})
    advance = dict(clearance.get('intermediate_selected_advance') or {})
    if not advance.get('required'):
        raise PddlProblemBuildError(
            'intermediate selected-shuttle advance requested without proof'
        )
    selected = _pddl_symbol(
        str(advance.get('shuttle') or problem.selected_shuttle)
    )
    target_slot = _pddl_symbol(str(advance.get('target_slot') or ''))
    if not selected or not target_slot:
        raise PddlProblemBuildError(
            'intermediate selected-shuttle advance has incomplete endpoints'
        )
    goal_text = f'(shuttle_at_slot {selected} {target_slot})'
    name = f'{problem.problem_name}-advance-{_slot_number_from_object(target_slot)}'
    problem_text = _replace_problem_goal(
        problem,
        problem_name=name,
        goal_text=goal_text,
        error_context='intermediate selected-shuttle advance',
    )
    problem_text = _reset_pending_clearances(
        problem_text,
        side=problem.side,
        error_context='intermediate-advance',
    )
    # No other free destination may be used as an unreviewed waypoint in this
    # one-action subproblem.  The complete facts are restored after reobserve.
    problem_text, withheld = _retain_only_target_free_slot(
        problem_text,
        side=problem.side,
        target_slot=target_slot,
        error_context='intermediate advance',
    )
    provenance.update({
        'planning_phase': 'advance_selected_to_intermediate_slot',
        'intermediate_selected_advance': advance,
        'parent_problem_name': problem.problem_name,
        'parent_final_target_slot': problem.target_slot,
        'temporarily_withheld_known_free_slots': withheld,
    })
    slot_number = _slot_number_from_object(target_slot)
    return replace(
        problem,
        problem_name=name,
        problem_text=problem_text,
        goal_text=goal_text,
        target_station=SLOT_STATION_BY_SIDE_AND_SLOT[(problem.side, slot_number)],
        target_slot=slot_number,
        selected_shuttle=selected,
        provenance=provenance,
    )


def build_clearance_pause_problem(
    problem: Room315PddlProblem,
) -> Room315PddlProblem:
    """Isolate a certified safe pause when the interior buffer is full."""

    provenance = dict(problem.provenance or {})
    normalization = dict(
        provenance.get('route_normalization', {})
        .get('by_side', {})
        .get(problem.side, {})
    )
    if not normalization.get('clearance_mode') or not normalization.get(
        'clearance_pause_safe'
    ):
        raise PddlProblemBuildError(
            'clearance pause requested without a certified safe active phase'
        )
    goal_text = f'(normal_route {problem.side})'
    name = f'{problem.problem_name}-pause-clearance'
    problem_text = _replace_problem_goal(
        problem,
        problem_name=name,
        goal_text=goal_text,
        error_context='clearance pause',
    )
    provenance.update({
        'planning_phase': 'pause_clearance_for_exterior_choreography',
        'parent_problem_name': problem.problem_name,
        'clearance_pause': {
            'reason': 'interior_staging_capacity_exhausted',
            'certified_stopped_interior_shuttles': list(
                normalization.get('certified_stopped_interior_shuttles') or []
            ),
            'controller_position_fields_used_for_localization': False,
        },
    })
    return replace(
        problem,
        problem_name=name,
        problem_text=problem_text,
        goal_text=goal_text,
        provenance=provenance,
    )


def _observed_state_from_scenario_spec(spec: ScenarioSpec) -> ObservedState:
    timestamp = 0.0
    facts: list[ObservedFact] = []
    side = _normalise_planning_side(spec.side)
    start_slots = {
        _canonical_planning_shuttle_id(shuttle, side=side): _slot_symbol_or_empty(slot)
        for shuttle, slot in spec.start_slots_by_shuttle
    }
    if spec.shuttle and spec.shuttle not in start_slots:
        source_slot = _slot_for_station_default(side, spec.source)
        if source_slot:
            start_slots[_canonical_planning_shuttle_id(spec.shuttle, side=side)] = source_slot
    if spec.blocker_shuttle and spec.blocker_start_slot:
        start_slots[_canonical_planning_shuttle_id(spec.blocker_shuttle, side=side)] = (
            _slot_symbol_or_empty(spec.blocker_start_slot)
        )
    for step in spec.clearance_steps:
        shuttle = _canonical_planning_shuttle_id(_clearance_step_shuttle(step), side=side)
        start_slots.setdefault(
            shuttle,
            _clearance_step_start_slot(spec, step, shuttle=shuttle),
        )

    loaded_shuttles = {
        _canonical_planning_shuttle_id(shuttle, side=side)
        for shuttle in spec.loaded_shuttles
    }
    if not loaded_shuttles and spec.payload_condition == 'loaded' and spec.shuttle:
        loaded_shuttles.add(_canonical_planning_shuttle_id(spec.shuttle, side=side))

    for shuttle_spec in all_shuttle_specs():
        shuttle_id = shuttle_spec.shuttle_id
        slot = start_slots.get(shuttle_id, '')
        present = bool(slot)
        loaded = shuttle_id in loaded_shuttles
        metadata = {
            'side': shuttle_spec.side,
            'short_id': shuttle_spec.short_id,
            'model_input_exposure': 'excluded',
            'synthetic_from_case_spec': True,
        }
        facts.extend([
            _planner_fact(
                shuttle_spec.gazebo_entity_name,
                'present',
                present,
                timestamp=timestamp,
                metadata=metadata,
            ),
            _planner_fact(
                shuttle_spec.gazebo_entity_name,
                'loaded',
                loaded,
                timestamp=timestamp,
                metadata=metadata,
            ),
            _planner_fact(
                shuttle_spec.gazebo_entity_name,
                'location_slot',
                _contract_slot_id(shuttle_spec.side, slot) if slot else None,
                timestamp=timestamp,
                metadata=metadata,
            ),
            _planner_fact(
                shuttle_spec.gazebo_entity_name,
                'location_block',
                _block_id_for_slot(shuttle_spec.side, slot) if slot else None,
                timestamp=timestamp,
                metadata=metadata,
            ),
        ])

    for rail_side in SIDES:
        occupied_by_slot = {
            slot: shuttle
            for shuttle, slot in start_slots.items()
            if _shuttle_side(shuttle) == rail_side and slot
        }
        for slot in ('1', '2', '3', '4'):
            shuttle = occupied_by_slot.get(slot, '')
            facts.append(_planner_fact(
                _contract_slot_id(rail_side, slot),
                'occupancy',
                {
                    'occupied': bool(shuttle),
                    'shuttle': _gazebo_entity_for_planning_shuttle(shuttle) if shuttle else None,
                    'sensor': SLOT_SENSOR_BY_SIDE_AND_SLOT[(rail_side, slot)],
                },
                timestamp=timestamp,
                metadata={
                    'side': rail_side,
                    'slot': slot,
                    'model_input_exposure': 'excluded',
                    'synthetic_from_case_spec': True,
                },
            ))
            block_id = _block_id_for_slot(rail_side, slot)
            facts.append(_planner_fact(
                block_id,
                'occupancy',
                _gazebo_entity_for_planning_shuttle(shuttle) if shuttle else None,
                timestamp=timestamp,
                metadata={
                    'side': rail_side,
                    'slot': slot,
                    'block_id': block_id,
                    'model_input_exposure': 'excluded',
                    'synthetic_from_case_spec': True,
                },
            ))
            facts.append(_planner_fact(
                block_id,
                'reservation',
                None,
                timestamp=timestamp,
                metadata={
                    'side': rail_side,
                    'slot': slot,
                    'block_id': block_id,
                    'model_input_exposure': 'excluded',
                    'synthetic_from_case_spec': True,
                },
            ))
        for device in DEVICE_NAMES:
            facts.append(_planner_fact(
                f'{rail_side}:switch:{device}',
                'state',
                'EXTERIOR',
                timestamp=timestamp,
                metadata={'side': rail_side, 'device': device, 'model_input_exposure': 'excluded'},
            ))
            facts.append(_planner_fact(
                f'{rail_side}:stopper:{device}',
                'state',
                'open',
                timestamp=timestamp,
                metadata={'side': rail_side, 'device': device, 'model_input_exposure': 'excluded'},
            ))
        facts.append(_planner_fact(
            f'{rail_side}:obstacles',
            'present_obstacles',
            [],
            timestamp=timestamp,
            metadata={'side': rail_side, 'model_input_exposure': 'excluded'},
        ))

    return ObservedState(
        state_id=f'room315-scenario-{_clean_symbol(spec.goal_id)}',
        timestamp=timestamp,
        stale_after_s=1.0,
        visual_model_inputs=[],
        fused_planner_state=facts,
    )


def _task_goal_from_scenario_spec(spec: ScenarioSpec) -> TaskGoal:
    constraints: dict[str, Any] = {
        'goal_type': 'transport',
        'side': spec.side,
        'target_kind': 'slot' if spec.target_slot else 'station',
        'target_station': spec.target,
        'shuttle_selection': 'explicit',
        'payload_required': spec.payload_condition,
    }
    if spec.target_slot:
        constraints['target_slot'] = spec.target_slot
    if spec.selection_policy:
        constraints['shuttle_selection'] = (
            'nearest'
            if 'nearest' in spec.selection_policy
            else spec.payload_condition or 'loaded'
        )
    else:
        constraints['target_shuttle'] = spec.shuttle
    return TaskGoal(
        goal_id=spec.goal_id,
        description=spec.pddl_goal or _language_action_sequence_for_spec(spec),
        source='planner',
        timestamp=0.0,
        confidence=1.0,
        constraints=constraints,
    )


def _planner_fact(
    subject: str,
    predicate: str,
    value: Any,
    *,
    timestamp: float,
    metadata: dict[str, Any],
) -> ObservedFact:
    return ObservedFact(
        fact_id=f'planner-{_pddl_symbol(subject)}-{_pddl_symbol(predicate)}',
        subject=subject,
        predicate=predicate,
        value=value,
        source='planner',
        timestamp=timestamp,
        confidence=1.0,
        status='known',
        metadata=metadata,
    )


def _observed_fact_index(observed_state: ObservedState) -> dict[tuple[str, str], ObservedFact]:
    index: dict[tuple[str, str], ObservedFact] = {}
    for fact in observed_state.fused_planner_state:
        key = (str(fact.subject), str(fact.predicate))
        if key in index and index[key].value != fact.value:
            raise PddlProblemBuildError(f'conflicting duplicate fact for {key!r}')
        index[key] = fact
    return index


def _known_fact(
    index: dict[tuple[str, str], ObservedFact],
    subjects: tuple[str, ...],
    predicate: str,
    *,
    context: str,
    required: bool = True,
) -> ObservedFact | None:
    for subject in subjects:
        fact = index.get((subject, predicate))
        if fact is None:
            continue
        if fact.status != 'known':
            raise PddlProblemBuildError(
                f'{context} fact {subject!r}/{predicate!r} is {fact.status}; '
                'observation or recovery is required before planning'
            )
        return fact
    if required:
        raise PddlProblemBuildError(
            f'missing required {context} fact for {subjects[0]!r}/{predicate!r}; '
            'observation or recovery is required before planning'
        )
    return None


def _fleet_snapshot_from_observed_state(
    index: dict[tuple[str, str], ObservedFact],
    *,
    planning_side: str | None = None,
    runtime_clearance_certificates: dict[str, dict[str, Any]] | None = None,
) -> dict[str, Any]:
    validated_planning_side = (
        _normalise_planning_side(planning_side)
        if planning_side is not None
        else None
    )
    loaded_by_shuttle: dict[str, bool] = {}
    present_by_shuttle: dict[str, bool] = {}
    location_slot_by_shuttle: dict[str, str] = {}
    location_block_by_shuttle: dict[str, str] = {}
    rail_position_by_shuttle: dict[str, dict[str, Any]] = {}
    planning_rail_position_by_shuttle: dict[str, dict[str, Any]] = {}
    rail_position_facts: dict[str, ObservedFact] = {}
    location_slot_facts: dict[str, ObservedFact] = {}
    verified_slot_arrival_facts: dict[str, ObservedFact] = {}
    verified_slot_arrival_by_shuttle: dict[str, dict[str, Any]] = {}
    exact_slot_anchor_visual_consistency: dict[str, dict[str, Any]] = {}
    slot_occupancy: dict[str, str] = {}
    deferred_out_of_scope_location_issues: list[dict[str, str]] = []

    for shuttle_spec in all_shuttle_specs():
        shuttle_id = shuttle_spec.shuttle_id
        subjects = (shuttle_spec.gazebo_entity_name, shuttle_id)
        present_fact = _known_fact(
            index,
            subjects,
            'present',
            context='presence',
        )
        if not isinstance(present_fact.value, bool):
            raise PddlProblemBuildError(
                f'presence fact for {shuttle_id!r} must be an explicit boolean'
            )
        present_by_shuttle[shuttle_id] = present_fact.value
        if not present_by_shuttle[shuttle_id]:
            # Presence is the runtime gate.  An explicitly absent shuttle has
            # no accepted visual payload/location facts and must not require
            # them merely to build the planner snapshot.
            loaded_by_shuttle[shuttle_id] = False
            continue
        loaded_fact = _known_fact(index, subjects, 'loaded', context='payload')
        loaded_by_shuttle[shuttle_id] = bool(loaded_fact.value)
        slot_fact = _known_fact(index, subjects, 'location_slot', context='slot location', required=False)
        if slot_fact is not None and slot_fact.value:
            slot_object = _slot_symbol_from_observation(
                slot_fact.value,
                default_side=shuttle_spec.side,
            )
            if not slot_object.startswith(f'{shuttle_spec.side}_slot_'):
                raise PddlProblemBuildError(
                    f'slot location side conflict for {shuttle_id!r}: '
                    f'{slot_object!r}'
                )
            location_slot_by_shuttle[shuttle_id] = slot_object
            location_slot_facts[shuttle_id] = slot_fact
        block_fact = _known_fact(index, subjects, 'location_block', context='block location', required=False)
        if block_fact is not None and block_fact.value:
            location_block_by_shuttle[shuttle_id] = _pddl_symbol(block_fact.value)
        rail_position_fact = _known_fact(
            index,
            subjects,
            'rail_position',
            context='continuous rail position',
            required=False,
        )
        if rail_position_fact is not None:
            rail_position_facts[shuttle_id] = rail_position_fact
        verified_arrival_fact = _known_fact(
            index,
            subjects,
            'verified_slot_arrival',
            context='verified slot arrival',
            required=False,
        )
        if verified_arrival_fact is not None:
            verified_slot_arrival_facts[shuttle_id] = verified_arrival_fact

    for rail_side in SIDES:
        for slot in ('1', '2', '3', '4'):
            slot_id = _contract_slot_id(rail_side, slot)
            slot_symbol = _slot_object(rail_side, slot)
            fact = _known_fact(
                index,
                (slot_id, slot_symbol),
                'occupancy',
                context='slot occupancy',
            )
            occupant = _occupancy_shuttle_from_value(fact.value, side=rail_side, slot=slot)
            slot_occupancy[slot_symbol] = occupant
            if occupant:
                if not present_by_shuttle.get(occupant, False):
                    raise PddlProblemBuildError(
                        f'slot {slot_symbol!r} reports absent shuttle '
                        f'{occupant!r} as its occupant'
                    )
                location_slot_by_shuttle.setdefault(occupant, slot_symbol)

    validated_runtime_clearances = _validated_runtime_clearance_certificates(
        runtime_clearance_certificates,
        present_by_shuttle=present_by_shuttle,
    )
    topologies = {side: _planning_rail_topology(side) for side in SIDES}
    lengths_by_side = {
        side: rail_segment_lengths(side)
        for side in SIDES
    }
    for shuttle_spec in all_shuttle_specs():
        shuttle_id = shuttle_spec.shuttle_id
        if not present_by_shuttle[shuttle_id]:
            continue
        raw_fact = rail_position_facts.get(shuttle_id)
        if raw_fact is not None:
            position = _rail_position_from_fact(
                raw_fact,
                shuttle_id=shuttle_id,
                expected_side=shuttle_spec.side,
                segment_lengths=lengths_by_side[shuttle_spec.side],
            )
            slot_fact = location_slot_facts.get(shuttle_id)
            verified_slot_arrival: dict[str, Any] | None = None
            verified_fact = verified_slot_arrival_facts.get(shuttle_id)
            if verified_fact is not None:
                if (
                    slot_fact is None
                    or not _is_exact_slot_anchor_fact(slot_fact)
                    or shuttle_id not in location_slot_by_shuttle
                ):
                    raise PddlProblemBuildError(
                        f'verified slot arrival for {shuttle_id!r} lacks an '
                        'exact trusted-device slot anchor'
                    )
                verified_slot_arrival = _verified_slot_arrival_from_fact(
                    verified_fact,
                    shuttle_id=shuttle_id,
                    side=shuttle_spec.side,
                    slot_object=location_slot_by_shuttle[shuttle_id],
                )
                verified_slot_arrival_by_shuttle[
                    shuttle_id
                ] = verified_slot_arrival
            if slot_fact is not None and _is_exact_slot_anchor_fact(slot_fact):
                try:
                    consistency = _validate_exact_slot_anchor_visual_consistency(
                        shuttle_id=shuttle_id,
                        side=shuttle_spec.side,
                        slot_object=location_slot_by_shuttle[shuttle_id],
                        rail_position_fact=raw_fact,
                        position=position,
                        topology=topologies[shuttle_spec.side],
                        verified_slot_arrival=verified_slot_arrival,
                    )
                    exact_slot_anchor_visual_consistency[
                        shuttle_id
                    ] = consistency
                except PddlProblemBuildError as exc:
                    if (
                        validated_planning_side is None
                        or shuttle_spec.side == validated_planning_side
                    ):
                        raise
                    # Keep the disagreement visible in provenance; it is not
                    # reclassified as valid or used to localize the goal-side
                    # route.  A later task for this rail rebuilds with this
                    # side in scope and fails closed on the same evidence.
                    deferred_out_of_scope_location_issues.append({
                        'shuttle': shuttle_id,
                        'side': shuttle_spec.side,
                        'reason': str(exc),
                    })
            rail_position_by_shuttle[shuttle_id] = position
            planning_position = dict(position)
            if verified_slot_arrival is not None:
                if shuttle_id in validated_runtime_clearances:
                    raise PddlProblemBuildError(
                        'shuttle cannot have simultaneous interior-clearance '
                        f'and verified-slot-arrival proofs: {shuttle_id}'
                    )
                planning_position = _verified_slot_planning_position(
                    visual_position=position,
                    slot_object=location_slot_by_shuttle[shuttle_id],
                    topology=topologies[shuttle_spec.side],
                )
            elif shuttle_id in validated_runtime_clearances:
                planning_position = _runtime_clearance_planning_position(
                    visual_position=position,
                    certificate=validated_runtime_clearances[shuttle_id],
                )
            planning_rail_position_by_shuttle[
                shuttle_id
            ] = planning_position
            continue
        slot_object = location_slot_by_shuttle.get(shuttle_id)
        if slot_object:
            slot_fact = location_slot_facts.get(shuttle_id)
            if slot_fact is not None and _is_exact_slot_anchor_fact(slot_fact):
                raise PddlProblemBuildError(
                    f'exact slot anchor for {shuttle_id!r} has no accepted '
                    'visual segment/ratio for consistency validation'
                )
            slot_number = _slot_number_from_object(slot_object)
            slot_location = topologies[shuttle_spec.side].slots[slot_number]
            rail_position_by_shuttle[shuttle_id] = {
                'side': shuttle_spec.side,
                'segment': slot_location.segment,
                's_ratio': slot_location.s_ratio,
                'segment_length_m': lengths_by_side[shuttle_spec.side].get(
                    slot_location.segment
                ),
                'position_uncertainty_m': 0.0,
                'source': 'derived_from_known_slot',
            }
            planning_rail_position_by_shuttle[shuttle_id] = dict(
                rail_position_by_shuttle[shuttle_id]
            )
            continue
        raise PddlProblemBuildError(
            f'present shuttle {shuttle_id!r} has neither a known continuous '
            'rail position nor a known slot; observation is required before planning'
        )

    return {
        'loaded_by_shuttle': loaded_by_shuttle,
        'present_by_shuttle': present_by_shuttle,
        'location_slot_by_shuttle': location_slot_by_shuttle,
        'location_block_by_shuttle': location_block_by_shuttle,
        'rail_position_by_shuttle': rail_position_by_shuttle,
        'planning_rail_position_by_shuttle': (
            planning_rail_position_by_shuttle
        ),
        'verified_slot_arrival_by_shuttle': (
            verified_slot_arrival_by_shuttle
        ),
        'exact_slot_anchor_visual_consistency': (
            exact_slot_anchor_visual_consistency
        ),
        'slot_occupancy': slot_occupancy,
        'planning_side': validated_planning_side or 'all_rails',
        'deferred_out_of_scope_location_issues': (
            deferred_out_of_scope_location_issues
        ),
        'runtime_clearance_certificates': (
            validated_runtime_clearances
        ),
    }


def _is_exact_slot_anchor_fact(fact: ObservedFact) -> bool:
    metadata = fact.metadata if isinstance(fact.metadata, dict) else {}
    return (
        fact.source == 'state_fuser'
        and metadata.get('selected_source') == 'trusted_device'
    )


def _validate_exact_slot_anchor_visual_consistency(
    *,
    shuttle_id: str,
    side: str,
    slot_object: str,
    rail_position_fact: ObservedFact,
    position: dict[str, Any],
    topology: Any,
    verified_slot_arrival: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Bind raw learned location to a DZI anchor without rewriting either.

    A raw active DZI still requires visual segment and bounded-ratio
    agreement.  Only an executor certificate created after multi-frame
    identity confirmation, a supervised OFF command, and a fresh
    stopped-at-target controller effect may turn either disagreement into a
    diagnostic.  The learned location remains unchanged in the raw facts;
    only the separate planning geometry is anchored to the certified slot.
    """

    position_metadata = (
        rail_position_fact.metadata
        if isinstance(rail_position_fact.metadata, dict)
        else {}
    )
    if not (
        rail_position_fact.source == 'state_fuser'
        and position_metadata.get('selected_source') == 'visual_model'
    ):
        raise PddlProblemBuildError(
            f'exact slot anchor for {shuttle_id!r} lacks an accepted visual '
            'segment/ratio fact'
        )
    slot = _slot_number_from_object(slot_object)
    # Fusion intentionally retains only the selected source, not the original
    # trusted-device metadata. Identity/side/slot are therefore validated from
    # the canonical fact subject/value; no unsupported sensor provenance is
    # invented at this boundary.
    expected_location = topology.slots[slot]
    observed_segment = str(position.get('segment') or '').strip().upper()
    expected_segment = str(expected_location.segment).strip().upper()
    segment_agrees = observed_segment == expected_segment
    if not segment_agrees and verified_slot_arrival is None:
        raise PddlProblemBuildError(
            f'exact slot anchor and visual segment disagree for {shuttle_id!r}: '
            f'expected {expected_segment}, observed '
            f'{observed_segment or "missing"}'
        )
    observed_ratio = float(position['s_ratio'])
    expected_ratio = float(expected_location.s_ratio)
    ratio_error = abs(observed_ratio - expected_ratio)
    ratio_comparison_applicable = segment_agrees
    tolerance_exceeded = bool(
        ratio_comparison_applicable
        and ratio_error > EXACT_SLOT_ANCHOR_VISUAL_TOLERANCE_RATIO
    )
    if tolerance_exceeded and verified_slot_arrival is None:
        raise PddlProblemBuildError(
            f'exact slot anchor and visual s_ratio disagree for {shuttle_id!r}: '
            f'error {ratio_error:.9f} exceeds '
            f'{EXACT_SLOT_ANCHOR_VISUAL_TOLERANCE_RATIO:.9f}'
        )
    return {
        'shuttle': shuttle_id,
        'side': side,
        'slot': slot,
        'segment': expected_segment,
        'raw_visual_segment': observed_segment,
        'canonical_slot_segment': expected_segment,
        'visual_segment_agrees': segment_agrees,
        'visual_segment_disagreement': not segment_agrees,
        'raw_visual_s_ratio': observed_ratio,
        'canonical_slot_s_ratio': expected_ratio,
        'absolute_ratio_error': ratio_error,
        'ratio_comparison_applicable': ratio_comparison_applicable,
        'legacy_ratio_tolerance': (
            EXACT_SLOT_ANCHOR_VISUAL_TOLERANCE_RATIO
        ),
        'legacy_ratio_tolerance_exceeded': tolerance_exceeded,
        'accepted_segment_disagreement_under_verified_arrival_certificate': (
            bool(not segment_agrees and verified_slot_arrival is not None)
        ),
        'accepted_ratio_disagreement_under_verified_arrival_certificate': (
            bool(tolerance_exceeded and verified_slot_arrival is not None)
        ),
        'accepted_under_verified_arrival_certificate': bool(
            verified_slot_arrival is not None
            and (not segment_agrees or tolerance_exceeded)
        ),
        'raw_visual_position_replaced': False,
        'controller_position_fields_used_for_localization': False,
    }


def _verified_slot_arrival_from_fact(
    fact: ObservedFact,
    *,
    shuttle_id: str,
    side: str,
    slot_object: str,
) -> dict[str, Any]:
    """Validate the narrow executor-owned slot proof admitted by the planner."""

    metadata = fact.metadata if isinstance(fact.metadata, dict) else {}
    if not (
        fact.source == 'state_fuser'
        and metadata.get('selected_source') == 'executor'
        and isinstance(fact.value, dict)
    ):
        raise PddlProblemBuildError(
            f'verified slot arrival for {shuttle_id!r} has invalid provenance'
        )
    certificate = dict(fact.value)
    expected_spec = normalize_shuttle_ref(shuttle_id, side=side)
    identity_spec = normalize_shuttle_ref(certificate.get('identity'))
    shuttle_spec = normalize_shuttle_ref(
        certificate.get('shuttle'),
        side=side,
    )
    if (
        expected_spec is None
        or identity_spec is None
        or shuttle_spec is None
        or not (
            expected_spec.short_id
            == identity_spec.short_id
            == shuttle_spec.short_id
        )
    ):
        raise PddlProblemBuildError(
            f'verified slot arrival identity conflict for {shuttle_id!r}'
        )
    slot = _slot_number_from_object(slot_object)
    sensor = SLOT_SENSOR_BY_SIDE_AND_SLOT[(side, slot)].upper()
    if (
        str(certificate.get('side') or '').strip().casefold() != side
        or str(certificate.get('slot') or '').strip() != slot
        or str(certificate.get('sensor') or '').strip().upper() != sensor
    ):
        raise PddlProblemBuildError(
            f'verified slot arrival slot/sensor conflict for {shuttle_id!r}'
        )
    proof_mode = str(certificate.get('proof_mode') or '')
    reached_target_slot = str(
        certificate.get('reached_target_slot') or ''
    ).strip()
    initial_occupancy_proof = (
        proof_mode == 'stable_stopped_dzi_initial_occupancy'
        and certificate.get('controller_target_slot_confirmed') is False
        and not reached_target_slot
    )
    commanded_arrival_proof = (
        proof_mode in {
            'supervised_command_arrival',
            'stable_stopped_dzi_runtime_recovery',
        }
        and certificate.get('controller_target_slot_confirmed') is True
        and reached_target_slot == slot
    )
    if (
        certificate.get('matched_by') != 'deterministic_slot_sensor'
        or not (initial_occupancy_proof or commanded_arrival_proof)
        or certificate.get('sensor_identity_confirmed') is not True
        or certificate.get('controller_stop_confirmed') is not True
        or str(certificate.get('controller_mode') or '').strip().upper()
        != 'DISABLED'
        or certificate.get('model_prediction_replaced') is not False
        or certificate.get(
            'controller_position_fields_used_for_localization'
        ) is not False
    ):
        raise PddlProblemBuildError(
            f'verified slot arrival proof incomplete for {shuttle_id!r}'
        )
    try:
        counts = (
            int(certificate.get('sensor_confirmation_frames')),
            int(certificate.get('sensor_sequence')),
            int(certificate.get('supervisor_sequence')),
        )
        motion_epoch = int(certificate.get('motion_epoch'))
    except (TypeError, ValueError) as exc:
        raise PddlProblemBuildError(
            f'verified slot arrival sequence invalid for {shuttle_id!r}'
        ) from exc
    if min(counts) < 1:
        raise PddlProblemBuildError(
            f'verified slot arrival sequence invalid for {shuttle_id!r}'
        )
    if motion_epoch < 0:
        raise PddlProblemBuildError(
            f'verified slot arrival motion epoch invalid for {shuttle_id!r}'
        )
    certificate.update({
        'identity': expected_spec.short_id,
        'shuttle': expected_spec.gazebo_entity_name,
        'side': side,
        'slot': slot,
        'sensor': sensor,
    })
    return certificate


def _verified_slot_planning_position(
    *,
    visual_position: dict[str, Any],
    slot_object: str,
    topology: Any,
) -> dict[str, Any]:
    """Create sensor-anchored planning geometry while retaining raw vision.

    A verified slot proof combines a stable identity-bearing DZI with a
    disabled controller. It can represent either a supervised arrival or
    initial stopped occupancy before any target was commanded. That proof owns
    the stopped shuttle's planning centre; a biased learned ratio must not
    stretch its body into the next slot and make an otherwise safe forward
    move impossible. The learned ratio remains unchanged in the raw fact and
    provenance. The protected interval still includes both complete shuttle
    half-bodies plus the route margin, so canonical occupancy is not reduced
    below the physical separation contract.
    """

    slot = _slot_number_from_object(slot_object)
    canonical = topology.slots[slot]
    side = _normalise_planning_side(visual_position['side'])
    canonical_segment = str(canonical.segment).strip().upper()
    canonical_segment_length_m = float(
        rail_segment_lengths(side)[canonical_segment]
    )
    segment_length_m = max(canonical_segment_length_m, 1e-9)
    uncertainty_m = float(
        visual_position.get('position_uncertainty_m') or 0.0
    )
    protected_center_extent_ratio = (
        # Route blocks describe the commanded shuttle centreline.  Protect
        # both complete bodies: half the stopped shuttle plus half the moving
        # shuttle equals one full shuttle length.
        DEFAULT_SHUTTLE_LENGTH_M
        + DEFAULT_ROUTE_SAFETY_MARGIN_M
        + uncertainty_m
    ) / segment_length_m
    raw_ratio = float(visual_position['s_ratio'])
    canonical_ratio = float(canonical.s_ratio)
    return {
        **visual_position,
        'segment': canonical_segment,
        's_ratio': canonical_ratio,
        'segment_length_m': canonical_segment_length_m,
        'source': 'verified_slot_arrival_categorical_anchor',
        'raw_visual_segment': str(
            visual_position.get('segment') or ''
        ).strip().upper(),
        'raw_visual_segment_length_m': float(
            visual_position['segment_length_m']
        ),
        'raw_visual_s_ratio': raw_ratio,
        'canonical_slot_s_ratio': canonical_ratio,
        'occupancy_start_s_ratio': max(
            0.0,
            canonical_ratio - protected_center_extent_ratio,
        ),
        'occupancy_end_s_ratio': min(
            1.0,
            canonical_ratio + protected_center_extent_ratio,
        ),
        'occupancy_source': 'verified_identity_bearing_slot_sensor',
        'raw_visual_s_ratio_used_for_occupancy': False,
        'raw_visual_position_replaced': False,
        'controller_position_fields_used_for_localization': False,
    }


def _runtime_clearance_planning_position(
    *,
    visual_position: dict[str, Any],
    certificate: dict[str, Any],
) -> dict[str, Any]:
    """Retain a supervised interior-relocation effect for later planning.

    The learned block and ratio remain untouched in ``rail_position_by_shuttle``.
    This separate planning view records the already verified effect of an
    executor-owned command: identity crossed the A3 interior-entry sensor,
    completed bounded commanded motion, and stopped.  It never reads controller
    position fields and is invalidated before any later motion command.
    """

    side = _normalise_planning_side(certificate['side'])
    public_segment = str(certificate['target_segment']).strip().upper()
    internal_segment = public_rail_segment_name_to_internal(side, public_segment)
    internal_lengths = rail_segment_lengths(side)
    if internal_segment not in internal_lengths:
        raise PddlProblemBuildError(
            'runtime clearance planning segment is absent from authoritative '
            f'{side} topology: {internal_segment}'
        )
    segment_length_m = float(internal_lengths[internal_segment])
    target_s_m = float(certificate['target_s_m'])
    if not 0.0 <= target_s_m <= segment_length_m:
        raise PddlProblemBuildError(
            'runtime clearance planning position is outside authoritative '
            f'{internal_segment}: {target_s_m}'
        )
    return {
        **visual_position,
        'side': side,
        'segment': internal_segment,
        's_ratio': target_s_m / segment_length_m,
        'segment_length_m': segment_length_m,
        'position_uncertainty_m': 0.0,
        'source': 'sensor_certified_execution_effect',
        'raw_visual_segment': visual_position['segment'],
        'raw_visual_s_ratio': visual_position['s_ratio'],
        'raw_visual_position_replaced': False,
        'controller_position_fields_used_for_localization': False,
        'effect_certificate_target_s_m': target_s_m,
        'effect_certificate_invalidated_before_next_motion': True,
    }


def _validated_runtime_clearance_certificates(
    raw: dict[str, dict[str, Any]] | None,
    *,
    present_by_shuttle: dict[str, bool] | None = None,
) -> dict[str, dict[str, Any]]:
    """Validate task-runtime clearance proof without replacing visual facts."""

    if raw is not None and not isinstance(raw, dict):
        raise PddlProblemBuildError(
            'runtime clearance certificates must be a mapping'
        )
    certificates: dict[str, dict[str, Any]] = {}
    target_positions_by_branch: dict[
        tuple[str, str], list[tuple[float, str]]
    ] = {
        (side, segment): []
        for side in SIDES
        for segment in INTERIOR_LOOP_GATE_BY_PUBLIC_SEGMENT
    }
    for raw_identity, raw_certificate in dict(raw or {}).items():
        try:
            certificate = normalize_runtime_clearance_certificate(
                raw_identity,
                raw_certificate,
            )
        except ValueError as exc:
            raise PddlProblemBuildError(str(exc)) from exc
        spec = normalize_shuttle_ref(certificate['identity'])
        assert spec is not None  # guaranteed by shared normalization
        if spec.shuttle_id in certificates:
            raise PddlProblemBuildError(
                f'duplicate runtime clearance identity for {spec.short_id}'
            )
        if (
            present_by_shuttle is not None
            and present_by_shuttle.get(spec.shuttle_id) is not True
        ):
            raise PddlProblemBuildError(
                f'runtime clearance identity is not explicitly present: '
                f'{spec.short_id}'
            )
        side = certificate['side']
        target_segment = str(
            certificate.get('target_segment') or ''
        ).strip().upper()
        gate = INTERIOR_LOOP_GATE_BY_PUBLIC_SEGMENT.get(target_segment, '')
        if not gate:
            raise PddlProblemBuildError(
                f'runtime clearance target segment invalid for '
                f'{spec.short_id}: expected one of '
                f'{sorted(INTERIOR_LOOP_GATE_BY_PUBLIC_SEGMENT)}, observed '
                f'{target_segment or "missing"}'
            )
        expected_sensor = INTERIOR_LOOP_ENTRY_SENSOR_BY_SIDE_AND_GATE[
            (side, gate)
        ]
        entry_sensor = str(
            certificate.get('entry_sensor') or ''
        ).strip().upper()
        if entry_sensor != expected_sensor:
            raise PddlProblemBuildError(
                f'runtime clearance entry sensor invalid for '
                f'{spec.short_id}: expected {expected_sensor}, observed '
                f'{entry_sensor or "missing"}'
            )
        target_s_m = certificate['target_s_m']
        segment_length_m = float(
            public_rail_segment_lengths(side)[target_segment]
        )
        if (
            not math.isfinite(target_s_m)
            or target_s_m < 0.0
            or target_s_m > segment_length_m
        ):
            raise PddlProblemBuildError(
                f'runtime clearance target_s_m out of bounds for '
                f'{spec.short_id}: {target_s_m!r} not in '
                f'[0.0, {segment_length_m}]'
            )
        certificate.update({
            'identity': spec.short_id,
            'shuttle': spec.shuttle_id,
            'side': side,
            'target_segment': target_segment,
            'gate_switch': gate,
            'target_s_m': target_s_m,
            'entry_sensor': entry_sensor,
            'segment_length_m': segment_length_m,
        })
        certificates[spec.shuttle_id] = certificate
        target_positions_by_branch[(side, target_segment)].append(
            (target_s_m, spec.shuttle_id)
        )

    required_spacing_m = (
        DEFAULT_SHUTTLE_LENGTH_M
        + DEFAULT_ROUTE_SAFETY_MARGIN_M
        + 2.0 * INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M
    )
    for (side, target_segment), positions in target_positions_by_branch.items():
        positions.sort()
        for (first_s_m, first), (second_s_m, second) in zip(
            positions,
            positions[1:],
        ):
            separation_m = second_s_m - first_s_m
            if separation_m + 1e-12 < required_spacing_m:
                raise PddlProblemBuildError(
                    'runtime clearance physical spacing violation on '
                    f'{side}:{target_segment}: '
                    f'{first},{second} separation {separation_m:.9f} m is '
                    f'below required {required_spacing_m:.9f} m'
                )
    return certificates


def _rail_position_from_fact(
    fact: ObservedFact,
    *,
    shuttle_id: str,
    expected_side: str,
    segment_lengths: dict[str, float],
) -> dict[str, Any]:
    raw = fact.value if isinstance(fact.value, dict) else {}
    metadata = fact.metadata if isinstance(fact.metadata, dict) else {}
    if raw.get('available') is False:
        raise PddlProblemBuildError(
            f'continuous rail position for {shuttle_id!r} is unavailable'
        )
    raw_side = raw.get('side') or metadata.get('side') or expected_side
    side = _normalise_planning_side(raw_side)
    if side != expected_side:
        raise PddlProblemBuildError(
            f'continuous rail position for {shuttle_id!r} reports side '
            f'{side!r}, expected {expected_side!r}'
        )
    segment = str(
        raw.get('segment')
        or metadata.get('segment')
        or ''
    ).strip().upper()
    segment = public_rail_segment_name_to_internal(expected_side, segment)
    if segment not in segment_lengths:
        raise PddlProblemBuildError(
            f'continuous rail position for {shuttle_id!r} has unknown '
            f'segment {segment!r}'
        )
    try:
        s_ratio = float(
            raw.get('s_ratio')
            if raw.get('s_ratio') is not None
            else metadata.get('s_ratio')
        )
    except (TypeError, ValueError) as exc:
        raise PddlProblemBuildError(
            f'continuous rail position for {shuttle_id!r} requires numeric s_ratio'
        ) from exc
    if not 0.0 <= s_ratio <= 1.0:
        raise PddlProblemBuildError(
            f'continuous rail position for {shuttle_id!r} has s_ratio '
            f'{s_ratio!r} outside [0.0, 1.0]'
        )
    raw_uncertainty = raw.get(
        'position_uncertainty_m',
        metadata.get('position_uncertainty_m', 0.0),
    )
    try:
        uncertainty_m = float(raw_uncertainty or 0.0)
    except (TypeError, ValueError) as exc:
        raise PddlProblemBuildError(
            f'continuous rail position for {shuttle_id!r} has invalid uncertainty'
        ) from exc
    if uncertainty_m < 0.0:
        raise PddlProblemBuildError(
            f'continuous rail position for {shuttle_id!r} has negative uncertainty'
        )
    return {
        'side': side,
        'segment': segment,
        's_ratio': s_ratio,
        'segment_length_m': float(segment_lengths[segment]),
        'position_uncertainty_m': uncertainty_m,
        'source': fact.source,
    }


def _device_snapshot_from_observed_state(
    index: dict[tuple[str, str], ObservedFact],
) -> dict[str, dict[str, str]]:
    devices = {'switches': {}, 'stoppers': {}}
    for side in SIDES:
        for device in DEVICE_NAMES:
            switch_subject = f'{side}:switch:{device}'
            stopper_subject = f'{side}:stopper:{device}'
            switch = _known_fact(index, (switch_subject,), 'state', context='switch state')
            stopper = _known_fact(index, (stopper_subject,), 'state', context='stopper state')
            devices['switches'][_switch_object(side, device)] = _switch_state_for_pddl(switch.value)
            devices['stoppers'][_stopper_object(side, device)] = _stopper_state_symbol(stopper.value)
    return devices


def _obstacle_snapshot_from_observed_state(
    index: dict[tuple[str, str], ObservedFact],
) -> dict[str, list[str]]:
    obstacles: dict[str, list[str]] = {}
    for side in SIDES:
        fact = _known_fact(
            index,
            (f'{side}:obstacles',),
            'present_obstacles',
            context='obstacle observation',
        )
        values = fact.value if isinstance(fact.value, (list, tuple, set)) else []
        obstacles[side] = [_pddl_symbol(f'{side}_{value}') for value in values if str(value).strip()]
    return obstacles


def _block_snapshot_from_observed_state(
    index: dict[tuple[str, str], ObservedFact],
    fleet: dict[str, Any],
) -> dict[str, Any]:
    block_occupancy: dict[str, str] = {}
    block_reservations: dict[str, str] = {}
    blocks = {
        _block_object_for_slot(side, slot): {'side': side, 'slot': slot}
        for side in SIDES
        for slot in ('1', '2', '3', '4')
    }
    for side in SIDES:
        topology = _planning_rail_topology(side)
        topology_segments = (
            set(topology.routing_table)
            | set(topology.fixed_transitions)
            | set(topology.fixed_transitions.values())
            | {location.segment for location in topology.slots.values()}
        )
        for segment in sorted(topology_segments):
            if segment and str(segment).upper() != 'FALLING':
                blocks[_topology_block_object(side, segment)] = {
                    'side': side,
                    'slot': '',
                    'topology_segment': str(segment).upper(),
                }
    for shuttle, block in fleet['location_block_by_shuttle'].items():
        block_symbol = _pddl_symbol(block)
        blocks.setdefault(block_symbol, {'side': _shuttle_side(shuttle), 'slot': ''})
        block_occupancy[block_symbol] = shuttle
    for side in SIDES:
        for slot in ('1', '2', '3', '4'):
            raw_block = _block_id_for_slot(side, slot)
            block_symbol = _block_object_for_slot(side, slot)
            occupancy = _known_fact(
                index,
                (raw_block, block_symbol),
                'occupancy',
                context='block occupancy',
                required=False,
            )
            if occupancy is not None and occupancy.value:
                block_occupancy[block_symbol] = _occupancy_shuttle_from_value(
                    occupancy.value,
                    side=side,
                    slot=slot,
                )
            reservation = _known_fact(
                index,
                (raw_block, block_symbol),
                'reservation',
                context='block reservation',
                required=False,
            )
            if reservation is not None and reservation.value:
                block_reservations[block_symbol] = _occupancy_shuttle_from_value(
                    reservation.value,
                    side=side,
                    slot=slot,
                )
    return {
        'blocks': blocks,
        'block_occupancy': block_occupancy,
        'block_reservations': block_reservations,
    }


def _goal_data_from_task_goal(
    constraints: dict[str, Any],
    *,
    side: str,
    goal_type: str,
    fleet: dict[str, Any],
    runtime_payload_grounding: dict[str, Any] | None = None,
) -> dict[str, Any]:
    if goal_type == 'inspection':
        subject = _pddl_symbol(constraints.get('inspection_subject') or 'room315_system')
        return {
            'goal_type': goal_type,
            'side': side,
            'inspection_subject': subject,
            'target_station': '',
            'target_slot': '',
            'selected_shuttle': '',
            'candidate_shuttles': (),
            'selection_policy': '',
            'planner_selects_candidate': False,
        }

    target_slot = _slot_symbol_or_empty(constraints.get('target_slot'))
    target_kind = str(constraints.get('target_kind') or '').strip().casefold()
    raw_station = constraints.get('target_station')
    target_station = _station_symbol(raw_station) if raw_station else ''
    if target_kind == 'slot' or target_slot:
        if not target_slot:
            raise PddlProblemBuildError('transport TaskGoal target_kind slot requires target_slot')
        target_station = _station_for_slot(side, target_slot)
    if not target_station:
        raise PddlProblemBuildError('transport TaskGoal requires target_station or target_slot')

    selection = str(
        constraints.get('selection_strategy')
        or constraints.get('shuttle_selection')
        or ''
    ).strip().casefold()
    target_shuttle = _task_goal_target_shuttle(constraints, side=side)
    payload_filter = str(
        constraints.get('payload_filter')
        or constraints.get('payload_required')
        or ''
    ).strip().casefold()
    if payload_filter == 'true':
        payload_filter = 'loaded'
    elif payload_filter == 'false':
        payload_filter = 'empty'
    if target_shuttle:
        selection = 'explicit'
    if selection in {'loaded', 'empty'} and payload_filter in {'', 'any'}:
        payload_filter = selection
        selection = 'any'
    if selection not in {'explicit', 'nearest', 'any'}:
        selection = 'explicit' if target_shuttle else 'any'
    if payload_filter not in {'loaded', 'empty', 'any'}:
        payload_filter = 'any'

    raw_payload_grounding = runtime_payload_grounding
    if raw_payload_grounding is not None and not isinstance(
        raw_payload_grounding,
        dict,
    ):
        raise PddlProblemBuildError(
            'runtime payload grounding must be a mapping'
        )
    payload_grounding = dict(raw_payload_grounding or {})
    if payload_grounding and not runtime_payload_grounding_matches(
        payload_grounding,
        selected_shuttle=target_shuttle,
        payload_filter=payload_filter,
    ):
        raise PddlProblemBuildError(
            'runtime payload grounding does not match the explicit target '
            'and payload filter'
        )

    side_shuttles = [
        spec.shuttle_id
        for spec in all_shuttle_specs()
        if spec.side == side
        and bool(fleet['present_by_shuttle'].get(spec.shuttle_id))
    ]
    if selection == 'explicit':
        if not target_shuttle:
            raise PddlProblemBuildError('explicit transport TaskGoal requires target_shuttle')
        if target_shuttle not in side_shuttles:
            raise PddlProblemBuildError(
                f'explicit transport target {target_shuttle!r} is not present '
                f'on the authoritative {side} presence registry'
            )
        # ``payload_filter`` is a candidate-selection qualifier.  The runtime
        # gateway validates it against an accepted visual state before it
        # grounds the TaskGoal to one explicit identity.  Reapplying the
        # qualifier after every atomic motion made the selected identity
        # unstable: a later occluded frame could flip loaded/empty and abort a
        # task whose payload cannot physically change in this transport-only
        # plan.  Keep the later raw visual prediction in the PDDL facts, but
        # accept a mismatch only with the gateway-created frozen proof.
        explicit_loaded = bool(fleet['loaded_by_shuttle'][target_shuttle])
        payload_matches = (
            payload_filter == 'any'
            or (payload_filter == 'loaded' and explicit_loaded)
            or (payload_filter == 'empty' and not explicit_loaded)
        )
        if not payload_matches and not payload_grounding:
            raise PddlProblemBuildError(
                f'explicit transport target {target_shuttle!r} does not '
                f'satisfy payload_filter={payload_filter}'
            )
        candidates = (target_shuttle,)
        selected = target_shuttle
        planner_selects_candidate = False
    else:
        wants_loaded = payload_filter == 'loaded'
        wants_empty = payload_filter == 'empty'
        candidates_list = []
        for shuttle in side_shuttles:
            loaded = bool(fleet['loaded_by_shuttle'][shuttle])
            if wants_loaded and not loaded:
                continue
            if wants_empty and loaded:
                continue
            _require_goal_candidate_location(fleet, shuttle)
            candidates_list.append(shuttle)
        if not candidates_list:
            raise PddlProblemBuildError(
                f'no {selection} shuttle candidates with known state on {side} rail'
            )
        if selection == 'nearest' and target_slot:
            candidates_list = sorted(
                candidates_list,
                key=lambda shuttle: (
                    _goal_candidate_route_cost(
                        fleet,
                        shuttle=shuttle,
                        target_slot=target_slot,
                    ),
                    _shuttle_sort_key(shuttle),
                ),
            )
        else:
            candidates_list = sorted(candidates_list, key=_shuttle_sort_key)
        candidates = tuple(candidates_list)
        selected = candidates[0]
        planner_selects_candidate = True

    station_resolution: dict[str, Any] = {}
    if not target_slot:
        station_slots = sorted(
            slot
            for (slot_side, slot), station in
            SLOT_STATION_BY_SIDE_AND_SLOT.items()
            if slot_side == side and station == target_station
        )
        if not station_slots:
            raise PddlProblemBuildError(
                f'target station {target_station!r} has no authoritative '
                f'sensor-backed slot on the {side} rail'
            )

        def station_slot_score(slot: str) -> tuple[float, ...]:
            slot_object = _slot_object(side, slot)
            occupant = fleet['slot_occupancy'].get(slot_object, '')
            already_satisfied = occupant in candidates
            occupancy_penalty = 0 if not occupant or already_satisfied else 1
            route_costs = [
                _goal_candidate_route_cost(
                    fleet,
                    shuttle=shuttle,
                    target_slot=slot,
                )
                for shuttle in candidates
            ]
            finite_costs = [cost for cost in route_costs if math.isfinite(cost)]
            best_cost = min(finite_costs) if finite_costs else math.inf
            return (
                0 if already_satisfied else 1,
                occupancy_penalty,
                best_cost,
                int(slot),
            )

        target_slot = min(station_slots, key=station_slot_score)
        station_resolution = {
            'requested_target_kind': 'station',
            'requested_target_station': target_station,
            'eligible_sensor_backed_slots': station_slots,
            'resolved_target_slot': target_slot,
            'policy': (
                'already_satisfied_then_free_then_shortest_authoritative_'
                'forward_route'
            ),
            'silent_first_slot_default_used': False,
        }

        if selection == 'nearest':
            candidates = tuple(sorted(
                candidates,
                key=lambda shuttle: (
                    _goal_candidate_route_cost(
                        fleet,
                        shuttle=shuttle,
                        target_slot=target_slot,
                    ),
                    _shuttle_sort_key(shuttle),
                ),
            ))
            selected = candidates[0]

    for shuttle in candidates:
        _require_goal_candidate_location(fleet, shuttle)

    eligible_candidates = tuple(candidates)
    if selection != 'explicit':
        if target_slot:
            target_slot_object = _slot_object(side, target_slot)

            def candidate_score(shuttle: str) -> tuple[float, ...]:
                already_at_target = (
                    fleet['location_slot_by_shuttle'].get(shuttle)
                    == target_slot_object
                )
                route_cost = _goal_candidate_route_cost(
                    fleet,
                    shuttle=shuttle,
                    target_slot=target_slot,
                )
                if selection == 'nearest':
                    return (
                        0 if already_at_target else 1,
                        route_cost,
                        *_shuttle_sort_key(shuttle),
                    )
                return (
                    0 if already_at_target else 1,
                    0 if math.isfinite(route_cost) else 1,
                    *_shuttle_sort_key(shuttle),
                )

            eligible_candidates = tuple(sorted(
                eligible_candidates,
                key=candidate_score,
            ))
        selected = eligible_candidates[0]
        # PDDL receives one identity-bound goal.  Exposing several candidates
        # while clearance provenance was frozen for only one of them could
        # let the planner move a different shuttle under the wrong blocker
        # proof.  Keep the full eligible set only as audit provenance.
        candidates = (selected,)
        planner_selects_candidate = False

    return {
        'goal_type': goal_type,
        'side': side,
        'target_station': target_station,
        'target_slot': target_slot,
        'selected_shuttle': selected,
        'candidate_shuttles': candidates,
        'eligible_candidate_shuttles': eligible_candidates,
        'selection_policy': selection,
        'planner_selects_candidate': planner_selects_candidate,
        'station_target_resolution': station_resolution,
        'payload_filter': payload_filter,
        'payload_filter_semantics': (
            'selection_time_only_after_explicit_grounding'
            if payload_grounding
            else (
                'live_visual_explicit_validation'
                if selection == 'explicit'
                else 'live_visual_candidate_filter_before_grounding'
            )
        ),
        'payload_grounding': payload_grounding,
        'selected_payload_prediction': (
            'loaded'
            if bool(fleet['loaded_by_shuttle'].get(selected))
            else 'empty'
        ),
        'selected_payload_prediction_matches_filter': (
            payload_filter == 'any'
            or (
                payload_filter == 'loaded'
                and bool(fleet['loaded_by_shuttle'].get(selected))
            )
            or (
                payload_filter == 'empty'
                and not bool(fleet['loaded_by_shuttle'].get(selected))
            )
        ),
    }


def _pddl_objects(
    *,
    fleet: dict[str, Any],
    devices: dict[str, dict[str, str]],
    obstacles: dict[str, list[str]],
    blocks: dict[str, Any],
    goal_data: dict[str, Any],
) -> dict[str, list[str]]:
    obstacle_objects = sorted({item for values in obstacles.values() for item in values})
    typed_objects = {
        'rail_side': list(SIDES),
        'shuttle': [spec.shuttle_id for spec in all_shuttle_specs()],
        'station': [
            'right_yaskawa',
            'right_staubli',
            'left_yaskawa',
            'left_kuka',
        ],
        'slot': [_slot_object(side, slot) for side in SIDES for slot in ('1', '2', '3', '4')],
        'block': sorted(blocks['blocks']),
        'switch_device': sorted(devices['switches']),
        'stopper_device': sorted(devices['stoppers']),
        'obstacle': obstacle_objects or ['no_obstacle'],
        'switch_group': [f'{side}_switch_group' for side in SIDES],
        'stopper_group': [f'{side}_stopper_group' for side in SIDES],
    }
    declared = {name for names in typed_objects.values() for name in names}
    inspection_targets = ['room315_system']
    if (
        goal_data['goal_type'] == 'inspection'
        and goal_data['inspection_subject'] not in declared
    ):
        inspection_targets.append(goal_data['inspection_subject'])
    typed_objects['inspection_target'] = sorted(set(inspection_targets) - declared)
    return typed_objects


def _route_clearance_snapshot(
    *,
    fleet: dict[str, Any],
) -> dict[str, Any]:
    rails = {
        side: {'shuttles': {}}
        for side in SIDES
    }
    certified_clearances = dict(
        fleet.get('runtime_clearance_certificates') or {}
    )
    planning_positions = (
        fleet.get('planning_rail_position_by_shuttle')
        or fleet['rail_position_by_shuttle']
    )
    for shuttle, position in planning_positions.items():
        if shuttle in certified_clearances:
            # The exact A3 interior-entry sensor and confirmed OFF effect prove
            # that this shuttle left the exterior route. Its raw visual
            # position is retained elsewhere; it is not rewritten here.
            continue
        side = position['side']
        rails[side]['shuttles'][shuttle] = {
            'current_segment': position['segment'],
            'rail_position': {
                'available': True,
                's_ratio': position['s_ratio'],
                'segment_length_m': position['segment_length_m'],
                'position_uncertainty_m': position['position_uncertainty_m'],
            },
        }
        if (
            position.get('occupancy_start_s_ratio') is not None
            and position.get('occupancy_end_s_ratio') is not None
        ):
            rails[side]['shuttles'][shuttle].update({
                'occupancy_start_s_ratio': float(
                    position['occupancy_start_s_ratio']
                ),
                'occupancy_end_s_ratio': float(
                    position['occupancy_end_s_ratio']
                ),
            })

    clear_pairs: set[tuple[str, str]] = set()
    blockers_by_pair: dict[tuple[str, str], tuple[str, ...]] = {}
    pair_provenance: list[dict[str, Any]] = []
    exterior_switches = {device: 'E' for device in DEVICE_NAMES}
    for side in SIDES:
        topology = _planning_rail_topology(side)
        for from_slot in ('1', '2', '3', '4'):
            from_object = _slot_object(side, from_slot)
            selected_shuttle = fleet['slot_occupancy'].get(from_object, '')
            for to_slot in ('1', '2', '3', '4'):
                to_object = _slot_object(side, to_slot)
                pair = (from_object, to_object)
                try:
                    blockers = route_blockers_from_rails(
                        rails,
                        topology,
                        from_slot,
                        to_slot,
                        selected_shuttle=selected_shuttle,
                        side=side,
                        switch_states=exterior_switches,
                    )
                    route_blocks = route_blocks_between_slots(
                        topology,
                        from_slot,
                        to_slot,
                        switch_states=exterior_switches,
                    )
                    error = ''
                except ValueError as exc:
                    blockers = []
                    route_blocks = []
                    error = str(exc)
                blocker_ids = tuple(
                    sorted({
                        _canonical_planning_shuttle_id(
                            blocker.shuttle_id,
                            side=side,
                        )
                        for blocker in blockers
                    })
                )
                blockers_by_pair[pair] = blocker_ids
                if not blocker_ids and not error:
                    clear_pairs.add(pair)
                pair_provenance.append({
                    'side': side,
                    'from_slot': from_object,
                    'to_slot': to_object,
                    'clear': pair in clear_pairs,
                    'selected_shuttle': selected_shuttle,
                    'blocker_shuttles': list(blocker_ids),
                    'route_blocks': [
                        {
                            'segment': block.segment,
                            'start_s_ratio': round(block.start_s_ratio, 6),
                            'end_s_ratio': round(block.end_s_ratio, 6),
                        }
                        for block in route_blocks
                    ],
                    'error': error,
                })

    ordering_pairs: set[tuple[str, str]] = set()
    for side in SIDES:
        by_segment: dict[str, list[tuple[str, float, float, float]]] = {}
        for shuttle, position in planning_positions.items():
            if shuttle in certified_clearances:
                continue
            if position['side'] != side:
                continue
            length_m = max(float(position['segment_length_m']), 1e-9)
            ratio = float(position['s_ratio'])
            occupancy_start = position.get('occupancy_start_s_ratio')
            occupancy_end = position.get('occupancy_end_s_ratio')
            if occupancy_start is None or occupancy_end is None:
                half_extent = (
                    DEFAULT_SHUTTLE_LENGTH_M / 2.0
                    + DEFAULT_ROUTE_SAFETY_MARGIN_M
                    + float(position['position_uncertainty_m'])
                ) / length_m
                occupancy_start = max(0.0, ratio - half_extent)
                occupancy_end = min(1.0, ratio + half_extent)
            by_segment.setdefault(position['segment'], []).append((
                shuttle,
                ratio,
                float(occupancy_start),
                float(occupancy_end),
            ))
        for segment_positions in by_segment.values():
            ordered = sorted(segment_positions, key=lambda item: (item[1], item[0]))
            for rear_index, rear in enumerate(ordered):
                for front in ordered[rear_index + 1:]:
                    if rear[3] < front[2]:
                        ordering_pairs.add((front[0], rear[0]))

    return {
        'clear_pairs': clear_pairs,
        'blockers_by_pair': blockers_by_pair,
        'ordering_pairs': ordering_pairs,
        'provenance': {
            'method': 'continuous_segment_occupancy_intervals_v1',
            'planned_switch_configuration': 'all_exterior',
            'sensor_certified_interior_clearances': [
                {
                    'shuttle': shuttle,
                    'target_s_m': certificate['target_s_m'],
                    'entry_sensor': certificate.get('entry_sensor', ''),
                    'controller_position_fields_used_for_localization': False,
                }
                for shuttle, certificate in sorted(
                    certified_clearances.items()
                )
            ],
            'verified_slot_arrival_anchors': [
                {
                    'shuttle': shuttle,
                    'slot': certificate['slot'],
                    'sensor': certificate['sensor'],
                    'proof_mode': certificate.get('proof_mode', ''),
                    'raw_visual_segment': (
                        fleet['rail_position_by_shuttle'][shuttle]['segment']
                    ),
                    'canonical_slot_segment': (
                        planning_positions[shuttle]['segment']
                    ),
                    'raw_visual_segment_length_m': (
                        fleet['rail_position_by_shuttle'][shuttle][
                            'segment_length_m'
                        ]
                    ),
                    'canonical_slot_segment_length_m': (
                        planning_positions[shuttle]['segment_length_m']
                    ),
                    'raw_visual_s_ratio': (
                        fleet['rail_position_by_shuttle'][shuttle]['s_ratio']
                    ),
                    'canonical_slot_s_ratio': (
                        planning_positions[shuttle]['s_ratio']
                    ),
                    'occupancy_start_s_ratio': planning_positions[shuttle].get(
                        'occupancy_start_s_ratio'
                    ),
                    'occupancy_end_s_ratio': planning_positions[shuttle].get(
                        'occupancy_end_s_ratio'
                    ),
                    'occupancy_source': planning_positions[shuttle].get(
                        'occupancy_source'
                    ),
                    'raw_visual_s_ratio_used_for_occupancy': (
                        planning_positions[shuttle].get(
                            'raw_visual_s_ratio_used_for_occupancy'
                        )
                    ),
                    'raw_visual_position_replaced': False,
                    'controller_position_fields_used_for_localization': False,
                }
                for shuttle, certificate in sorted(
                    (
                        fleet.get('verified_slot_arrival_by_shuttle')
                        or {}
                    ).items()
                )
            ],
            'shuttle_length_m': DEFAULT_SHUTTLE_LENGTH_M,
            'safety_margin_m': DEFAULT_ROUTE_SAFETY_MARGIN_M,
            'pairs': pair_provenance,
            'ordering': [
                {'front': front, 'rear': rear}
                for front, rear in sorted(ordering_pairs)
            ],
        },
    }


def _topology_route_snapshot(
    *,
    fleet: dict[str, Any],
    devices: dict[str, dict[str, str]],
    obstacles: dict[str, list[str]],
    goal_data: dict[str, Any],
    excluded_candidate_indices_by_shuttle: (
        dict[str, set[int]] | None
    ) = None,
) -> dict[str, Any]:
    """Compile non-slot visual locations through the authoritative rail graph."""

    entries: list[dict[str, Any]] = []
    if (
        goal_data.get('goal_type') == 'transport'
        and goal_data.get('target_slot')
    ):
        for shuttle in goal_data.get('candidate_shuttles') or ():
            if (
                shuttle in (
                    fleet.get('verified_slot_arrival_by_shuttle') or {}
                )
                and fleet['location_slot_by_shuttle'].get(shuttle)
            ):
                # A verified stopped arrival must start from its canonical
                # slot action family. Do not offer a ratio-derived topology
                # alternative for the same shuttle.
                continue
            entries.append(_topology_route_entry(
                fleet=fleet,
                devices=devices,
                obstacles=obstacles,
                shuttle=shuttle,
                target_slot=str(goal_data['target_slot']),
                excluded_candidate_indices=set(
                    (excluded_candidate_indices_by_shuttle or {}).get(
                        shuttle,
                        set(),
                    )
                ),
            ))
    return {
        'routes': entries,
        'by_shuttle_and_slot': {
            (entry['shuttle'], entry['target_slot_object']): entry
            for entry in entries
        },
        'provenance': {
            'method': 'authoritative_position_to_slot_topology_v2_all_origins',
            'network_sources': {
                side: str(RAIL_NETWORK_PATH_BY_SIDE[side]) for side in SIDES
            },
            'device_sources': {
                side: str(RAIL_DEVICES_PATH_BY_SIDE[side]) for side in SIDES
            },
            'controller_position_fields_used_for_localization': False,
            'routes': [
                {
                    key: value
                    for key, value in entry.items()
                    if key not in {'configured'}
                }
                for entry in entries
            ],
        },
    }


def _configured_clear_segment_route_binding(
    route: Any,
    *,
    side: str,
    purpose: str,
) -> dict[str, Any] | None:
    """Return proof that one configured mixed route must be consumed now.

    Receding-horizon planning rebuilds the parent TaskGoal after every atomic
    action.  A route prepared for the first blocker relocation is therefore
    just as authoritative as a route prepared for the final goal shuttle.  If
    only final-goal routes are retained, the next rebuild restores the normal
    route, then prepares the same blocker route again forever.  This binding
    is deliberately limited to a clear, configured, segment-origin route;
    exact-slot moves retain the canonical normal-route lifecycle.
    """

    if not isinstance(route, dict):
        return None
    if (
        route.get('side') != side
        or not route.get('configured')
        or not route.get('route_clear')
        or route.get('blockers')
        or route.get('source_slot_object')
    ):
        return None
    return {
        'purpose': str(purpose),
        'shuttle': route['shuttle'],
        'source_block': route['source_block'],
        'target_slot': route['target_slot_object'],
        'required_switches': dict(route['required_switches']),
        'required_stoppers': dict(route['required_stoppers']),
        'route_clear': True,
        'controller_position_fields_used_for_localization': False,
    }


def _direct_blocker_slot_reoccupies_goal_route(
    *,
    side: str,
    goal_route: Any,
    target_clearance: dict[str, Any],
) -> dict[str, Any] | None:
    """Detect a one-step relocation that cannot clear its own goal route."""

    if not isinstance(goal_route, dict):
        return None
    relocations = list(target_clearance.get('ordered_relocations') or [])
    if not relocations:
        return None
    relocation = dict(relocations[0])
    blocker = str(relocation.get('shuttle') or '')
    if blocker not in list(goal_route.get('blockers') or []):
        # A dependency move can legitimately enter a branch behind the goal
        # shuttle. Only the direct blocker itself is subject to this test.
        return None
    destination = dict(relocation.get('destination') or {})
    if str(destination.get('kind') or '').strip().casefold() != 'slot':
        return None
    if str(destination.get('source_slot') or '').strip():
        # A sensor-anchored slot-to-slot move can be a deliberate staging
        # transition that opens a later interior entry (for example slot 4 ->
        # slot 1 -> A34I).  The non-progress defect is specific to exporting a
        # segment-origin blocker into an exterior slot on the same goal trace.
        return None
    if (
        str(destination.get('source_kind') or '').strip().casefold()
        != 'accepted_visual_continuous_position'
    ):
        return None
    target_slot_object = str(destination.get('target_slot') or '')
    try:
        target_slot = _slot_number_from_object(target_slot_object)
        slot_position = _planning_rail_topology(side).slots[target_slot]
    except (KeyError, ValueError):
        return None
    for block_index, block in enumerate(goal_route.get('route_blocks') or []):
        if (
            str(block.get('segment') or '').strip().upper()
            == str(slot_position.segment).strip().upper()
            and float(block['start_s_ratio']) - 1e-9
            <= float(slot_position.s_ratio)
            <= float(block['end_s_ratio']) + 1e-9
        ):
            return {
                'reason': 'direct_blocker_parking_reoccupies_goal_route',
                'blocker': blocker,
                'parking_slot': target_slot_object,
                'overlap_segment': str(slot_position.segment).strip().upper(),
                'overlap_s_ratio': round(float(slot_position.s_ratio), 9),
                'overlap_route_block_index': block_index,
            }
    return None


def _clearance_certificate_visual_segment_consistency(
    *,
    side: str,
    certificate: dict[str, Any],
    position: dict[str, Any],
) -> dict[str, Any]:
    """Bind a persisted stop proof to the current accepted visual segment."""

    certified_public_segment = str(
        certificate.get('target_segment') or ''
    ).strip().upper()
    certified_internal_segment = public_rail_segment_name_to_internal(
        side,
        certified_public_segment,
    )
    visual_segment = str(
        position.get('raw_visual_segment')
        or position.get('segment')
        or ''
    ).strip().upper()
    satisfied = bool(
        certified_public_segment
        and visual_segment
        and visual_segment == certified_internal_segment
    )
    return {
        'required': True,
        'satisfied': satisfied,
        'certificate_target_public_segment': certified_public_segment,
        'certificate_target_internal_segment': certified_internal_segment,
        'accepted_visual_internal_segment': visual_segment,
        'certificate_used_as_localization': False,
        **(
            {
                'certificate_used_as_persisted_execution_effect': True,
                'planning_origin_segment': str(
                    position.get('segment') or ''
                ).strip().upper(),
                'raw_visual_prediction_preserved': True,
            }
            if not satisfied
            and position.get('source') == 'sensor_certified_execution_effect'
            else {}
        ),
        'reason': (
            'certificate_and_visual_segment_match'
            if satisfied
            else 'certificate_and_visual_segment_disagree'
        ),
    }


def _topology_route_entry(
    *,
    fleet: dict[str, Any],
    devices: dict[str, dict[str, str]],
    obstacles: dict[str, list[str]],
    shuttle: str,
    target_slot: str,
    target_position: tuple[str, float] | None = None,
    required_switch_states: dict[str, str] | None = None,
    excluded_candidate_indices: set[int] | None = None,
) -> dict[str, Any]:
    planning_positions = (
        fleet.get('planning_rail_position_by_shuttle')
        or fleet['rail_position_by_shuttle']
    )
    position = planning_positions.get(shuttle)
    if not isinstance(position, dict):
        raise PddlProblemBuildError(
            f'goal candidate {shuttle!r} has no accepted visual rail position'
        )
    side = _shuttle_side(shuttle)
    if position.get('side') != side:
        raise PddlProblemBuildError(
            f'goal candidate {shuttle!r} has a side-conflicting rail position'
        )
    clearance_certificate = (
        fleet.get('runtime_clearance_certificates') or {}
    ).get(shuttle)
    certificate_consistency: dict[str, Any] = {
        'required': False,
        'satisfied': True,
    }
    if clearance_certificate is not None:
        certificate_consistency = (
            _clearance_certificate_visual_segment_consistency(
                side=side,
                certificate=clearance_certificate,
                position=position,
            )
        )
        certified_public_segment = certificate_consistency[
            'certificate_target_public_segment'
        ]
        if not certified_public_segment:
            raise PddlProblemBuildError(
                f'runtime clearance for {shuttle!r} has no target segment'
            )
        if (
            not certificate_consistency['satisfied']
            and position.get('source')
            != 'sensor_certified_execution_effect'
        ):
            raise PddlProblemBuildError(
                'runtime clearance and accepted visual segment disagree for '
                f'{shuttle!r}: certificate={certified_public_segment}, '
                'visual='
                f'{certificate_consistency["accepted_visual_internal_segment"]}; '
                're-observation is required before '
                'topology motion'
            )
        if certificate_consistency['satisfied']:
            certificate_consistency.pop('reason', None)
    topology = _planning_rail_topology(side)
    route_target_description = f'slot {target_slot}'
    if target_position is not None:
        target_segment = str(target_position[0] or '').strip().upper()
        try:
            target_s_ratio = float(target_position[1])
        except (TypeError, ValueError) as exc:
            raise PddlProblemBuildError(
                f'invalid topology holding-position ratio {target_position[1]!r}'
            ) from exc
        if (
            not target_segment
            or not math.isfinite(target_s_ratio)
            or not 0.0 <= target_s_ratio <= 1.0
        ):
            raise PddlProblemBuildError(
                'topology holding position requires a known segment and a '
                'finite ratio in [0, 1]'
            )
        slot_key = _slot_number_from_object(target_slot)
        if slot_key not in topology.slots:
            raise PddlProblemBuildError(
                f'cannot bind topology holding position to unknown slot {target_slot!r}'
            )
        topology = replace(
            topology,
            slots={
                **topology.slots,
                slot_key: replace(
                    topology.slots[slot_key],
                    segment=target_segment,
                    s_ratio=target_s_ratio,
                ),
            },
        )
        route_target_description = (
            f'holding position {target_segment}@{target_s_ratio:.6f}'
        )
    rails = {side: {'shuttles': {}}}
    selected_uncertainty_m = float(
        position.get('position_uncertainty_m') or 0.0
    )
    for other, other_position in planning_positions.items():
        if other == shuttle or other_position.get('side') != side:
            continue
        segment_length_m = max(
            float(other_position['segment_length_m']),
            1e-9,
        )
        ratio = float(other_position['s_ratio'])
        # Compare shuttle centres while accounting for both complete bodies,
        # both localization uncertainties, and the configured margin.  The
        # old route-as-a-point check omitted half of the selected shuttle and
        # could therefore label a near-contact route as clear.
        required_center_spacing_m = (
            DEFAULT_SHUTTLE_LENGTH_M
            + DEFAULT_ROUTE_SAFETY_MARGIN_M
            + selected_uncertainty_m
            + float(other_position.get('position_uncertainty_m') or 0.0)
        )
        half_extent_ratio = required_center_spacing_m / segment_length_m
        centres = [ratio]
        # A runtime effect certificate can prove that a shuttle entered A34I
        # while the newest learned label still names its previous exterior
        # segment. Preserve that raw label for audit, but never project its
        # segment-local ratio onto a different topology segment: doing so can
        # stretch one certified interior occupant across the whole holding
        # loop and manufacture a collision that does not physically exist.
        raw_visual_segment = str(
            other_position.get('raw_visual_segment') or ''
        ).strip().upper()
        if (
            other_position.get('raw_visual_s_ratio') is not None
            and raw_visual_segment == str(
                other_position['segment']
            ).strip().upper()
        ):
            centres.append(float(other_position['raw_visual_s_ratio']))
        rails[side]['shuttles'][other] = {
            'current_segment': other_position['segment'],
            'rail_position': {
                'available': True,
                's_ratio': ratio,
                'segment_length_m': segment_length_m,
                'position_uncertainty_m': 0.0,
            },
            'occupancy_start_s_ratio': max(
                0.0,
                min(centres) - half_extent_ratio,
            ),
            'occupancy_end_s_ratio': min(
                1.0,
                max(centres) + half_extent_ratio,
            ),
        }
    try:
        route_candidates = occupancy_aware_route_candidates_from_position_to_slot(
            rails,
            topology,
            position['segment'],
            position['s_ratio'],
            target_slot,
            selected_shuttle=shuttle,
            side=side,
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise PddlProblemBuildError(
            f'no safe authoritative topology route for {shuttle!r}: {exc}'
        ) from exc
    if not route_candidates:
        raise PddlProblemBuildError(
            f'no non-FALLING static topology route exists for {shuttle!r} '
            f'to {route_target_description}'
        )
    normalized_required_switches = {
        str(device).strip().upper(): str(state).strip().upper()
        for device, state in (required_switch_states or {}).items()
    }
    if normalized_required_switches:
        route_candidates = tuple(
            candidate
            for candidate in route_candidates
            if all(
                candidate.route.switch_states.get(device) == state
                for device, state in normalized_required_switches.items()
            )
        )
        if not route_candidates:
            raise PddlProblemBuildError(
                f'no non-FALLING authoritative route from {shuttle!r} to '
                f'{route_target_description} uses required switch assignment '
                f'{normalized_required_switches}'
            )
    segment_lengths = rail_segment_lengths(side)
    observed_switches = {
        device: devices['switches'].get(_switch_object(side, device), '')
        for device in DEVICE_NAMES
    }

    def candidate_blocker_partition(
        candidate: Any,
    ) -> tuple[tuple[Any, ...], tuple[dict[str, Any], ...]]:
        """Separate forward blockers from safe rear-separation overlaps.

        Occupancy intervals include one complete shuttle length plus the
        configured route margin.  When two stopped shuttles are close, the
        *rear* shuttle's protected interval can therefore extend across the
        leading shuttle's route origin even though the commanded forward
        motion immediately increases their separation.  Treating that
        origin-only overlap as a forward blocker creates a false dependency
        cycle: the leader waits for the follower while the follower correctly
        waits for the leader.

        The exception is deliberately narrow.  The other shuttle's accepted
        visual centre must be strictly behind the mover on the same source
        segment, its interval may overlap only the first monotonically-forward
        route block, and it must not overlap any later visit to that segment.
        All shuttles ahead, on later blocks, or on a wrapped revisit remain
        blockers.  No controller position field participates in this proof.
        """

        route = candidate.route
        if not route.blocks:
            return tuple(candidate.blockers), ()
        first_block = route.blocks[0]
        source_segment = str(position['segment']).strip().upper()
        source_ratio = float(position['s_ratio'])
        if (
            first_block.segment != source_segment
            or float(first_block.end_s_ratio)
            <= float(first_block.start_s_ratio)
            or not math.isclose(
                float(first_block.start_s_ratio),
                source_ratio,
                abs_tol=1e-9,
            )
        ):
            return tuple(candidate.blockers), ()

        effective: list[Any] = []
        ignored: list[dict[str, Any]] = []
        for blocker in candidate.blockers:
            canonical = _canonical_planning_shuttle_id(
                blocker.shuttle_id,
                side=side,
            )
            blocker_position = planning_positions.get(canonical) or {}
            blocker_segment = str(
                blocker_position.get('segment') or ''
            ).strip().upper()
            blocker_ratio = blocker_position.get('s_ratio')
            interval_start = blocker.occupancy_start_s_ratio
            interval_end = blocker.occupancy_end_s_ratio
            overlap_indices = [
                index
                for index, block in enumerate(route.blocks)
                if block.overlaps(
                    blocker.segment,
                    interval_start,
                    interval_end,
                )
            ] if interval_start is not None and interval_end is not None else []
            separates_from_rear = (
                blocker.reason == 'route_occupancy_interval_overlap'
                and blocker_segment == source_segment
                and blocker_ratio is not None
                and float(blocker_ratio) < source_ratio
                and overlap_indices == [0]
            )
            if not separates_from_rear:
                effective.append(blocker)
                continue
            ignored.append({
                'shuttle': canonical,
                'source_segment': source_segment,
                'mover_s_ratio': round(source_ratio, 9),
                'rear_shuttle_s_ratio': round(float(blocker_ratio), 9),
                'rear_occupancy_start_s_ratio': round(
                    float(interval_start),
                    9,
                ),
                'rear_occupancy_end_s_ratio': round(
                    float(interval_end),
                    9,
                ),
                'overlap_route_block_indices': overlap_indices,
                'proof': (
                    'forward_motion_monotonically_increases_rear_spacing'
                ),
                'controller_position_fields_used_for_localization': False,
            })
        return tuple(effective), tuple(ignored)

    def candidate_score(index_and_candidate: tuple[int, Any]) -> tuple[Any, ...]:
        index, candidate = index_and_candidate
        effective_blockers, _ignored = candidate_blocker_partition(candidate)
        route_length_m = sum(
            max(0.0, block.end_s_ratio - block.start_s_ratio)
            * float(segment_lengths[block.segment])
            for block in candidate.route.blocks
        )
        switch_changes = sum(
            observed_switches.get(device)
            != ('interior' if state == 'I' else 'exterior')
            for device, state in candidate.route.switch_states.items()
        )
        return (
            0 if not effective_blockers else 1,
            len(effective_blockers),
            round(route_length_m, 9),
            switch_changes,
            index,
        )

    excluded_candidate_indices = set(excluded_candidate_indices or set())
    eligible_indexed_candidates = [
        (index, candidate)
        for index, candidate in enumerate(route_candidates)
        if index not in excluded_candidate_indices
    ]
    if not eligible_indexed_candidates:
        raise PddlProblemBuildError(
            f'no progress-preserving topology candidate remains for '
            f'{shuttle!r} to {route_target_description}; excluded='
            f'{sorted(excluded_candidate_indices)}'
        )
    selected_index, selected_candidate = min(
        eligible_indexed_candidates,
        key=candidate_score,
    )
    route = selected_candidate.route
    selected_effective_blockers, selected_ignored_rear_overlaps = (
        candidate_blocker_partition(selected_candidate)
    )
    # Preserve route order rather than collapsing repeated segments into one
    # dictionary index.  A wrapped path can encounter the target segment twice.
    ordered_candidate_blockers = sorted(
        selected_effective_blockers,
        key=lambda blocker: (
            next(
                (
                    route_index
                    for route_index, block in enumerate(route.blocks)
                    if block.overlaps(
                        blocker.segment,
                        blocker.occupancy_start_s_ratio,
                        blocker.occupancy_end_s_ratio,
                    )
                ),
                len(route.blocks),
            ),
            float(blocker.s_ratio or 0.0),
            blocker.shuttle_id,
        ),
    )
    blocker_ids = []
    for blocker in ordered_candidate_blockers:
        canonical = _canonical_planning_shuttle_id(
            blocker.shuttle_id,
            side=side,
        )
        if canonical not in blocker_ids:
            blocker_ids.append(canonical)
    target_object = _slot_object(side, target_slot)
    target_occupant = (
        fleet['slot_occupancy'].get(target_object, '')
        if target_position is None
        else ''
    )
    if target_occupant and target_occupant != shuttle and target_occupant not in blocker_ids:
        blocker_ids.append(target_occupant)

    expected_switches = dict(route.switch_states)
    switches_match = all(
        devices['switches'].get(_switch_object(side, device))
        == ('interior' if state == 'I' else 'exterior')
        for device, state in expected_switches.items()
    )
    stoppers_open = all(
        devices['stoppers'].get(_stopper_object(side, device)) == 'open'
        for device in DEVICE_NAMES
    )
    public_segment = internal_rail_segment_name_to_public(
        side,
        route.source_segment,
    )
    return {
        'shuttle': shuttle,
        'side': side,
        'source_kind': 'accepted_visual_continuous_position',
        'source_segment': route.source_segment,
        'source_public_segment': public_segment,
        'source_s_ratio': round(float(route.source_s_ratio), 9),
        'source_block': _topology_block_object(side, route.source_segment),
        'source_slot_object': fleet['location_slot_by_shuttle'].get(
            shuttle,
            '',
        ),
        'target_slot': route.target_slot,
        'target_slot_object': target_object,
        'target_station': _station_for_slot(side, route.target_slot),
        'target_sensor': SLOT_SENSOR_BY_SIDE_AND_SLOT[(side, route.target_slot)],
        'target_position': (
            {
                'segment': route.target_segment,
                's_ratio': round(float(route.target_s_ratio), 9),
            }
            if target_position is not None
            else None
        ),
        'required_switches': expected_switches,
        'required_stoppers': {device: '0' for device in DEVICE_NAMES},
        'route_blocks': [
            {
                'segment': block.segment,
                'public_segment': internal_rail_segment_name_to_public(
                    side,
                    block.segment,
                ),
                'start_s_ratio': round(float(block.start_s_ratio), 9),
                'end_s_ratio': round(float(block.end_s_ratio), 9),
            }
            for block in route.blocks
        ],
        'route_candidate_count': len(route_candidates),
        'selected_route_candidate_index': selected_index,
        'route_selection_policy': (
            'clear_before_blocked_then_fewest_blockers_then_shortest_'
            'authoritative_route_then_fewest_switch_changes_with_'
            'nonprogressing_candidate_exclusion'
        ),
        'excluded_route_candidate_indices': sorted(
            excluded_candidate_indices
        ),
        'route_candidates': [
            {
                'index': index,
                'clear': not candidate_blocker_partition(candidate)[0],
                'blockers': [
                    _canonical_planning_shuttle_id(
                        blocker.shuttle_id,
                        side=side,
                    )
                    for blocker in candidate_blocker_partition(candidate)[0]
                ],
                'raw_interval_blockers': [
                    _canonical_planning_shuttle_id(
                        blocker.shuttle_id,
                        side=side,
                    )
                    for blocker in candidate.blockers
                ],
                'ignored_rear_separation_overlaps': list(
                    candidate_blocker_partition(candidate)[1]
                ),
                'required_switches': dict(candidate.route.switch_states),
                'route_segments': [
                    block.segment for block in candidate.route.blocks
                ],
            }
            for index, candidate in enumerate(route_candidates)
        ],
        'ignored_rear_separation_overlaps': list(
            selected_ignored_rear_overlaps
        ),
        'blockers': blocker_ids,
        'route_clear': not blocker_ids and not bool(obstacles.get(side)),
        'configured': switches_match and stoppers_open,
        'switches_match': switches_match,
        'stoppers_open': stoppers_open,
        'controller_position_fields_used_for_localization': False,
        'runtime_clearance_visual_consistency': certificate_consistency,
    }


_INTERIOR_HOLDING_END_MARGIN_M = 0.35
_INTERIOR_HOLDING_GRID_STEP_M = 0.01


def _interior_internal_segment(side: str, public_segment: str) -> str:
    return public_rail_segment_name_to_internal(side, public_segment)


def _public_segment_for_side(side: str, internal_segment: Any) -> str:
    return internal_rail_segment_name_to_public(side, internal_segment)


def _is_interior_holding_position(
    *,
    side: str,
    position: dict[str, Any],
    public_segment: str | None = None,
) -> bool:
    observed_public = _public_segment_for_side(
        side,
        position.get('segment'),
    )
    if public_segment is not None:
        return observed_public == public_segment
    return observed_public in INTERIOR_LOOP_GATE_BY_PUBLIC_SEGMENT


def _clearance_device_states_for_branch(
    branch: dict[str, Any],
) -> tuple[dict[str, str], dict[str, str]]:
    switches = {
        device: ('interior' if state == 'I' else 'exterior')
        for device, state in dict(branch['switches']).items()
    }
    exit_switch = str(branch['exit_switch']).strip().upper()
    stoppers = {
        device: ('closed' if device == exit_switch else 'open')
        for device in DEVICE_NAMES
    }
    return switches, stoppers


def _interior_route_switches_for_source(
    *,
    side: str,
    branch: dict[str, Any],
    source_position: dict[str, Any],
) -> dict[str, str]:
    """Complete a holding-route assignment for an interior source.

    A branch's canonical switch assignment describes entry from the exterior
    loop.  It is incomplete when the mover already occupies the *other*
    interior branch: that shuttle must first traverse its current converging
    gate before it can reach the requested diverging gate.  Leaving the source
    exit at ``E`` manufactures a FALLING route and makes a physically feasible
    cross-branch capacity transfer appear impossible.

    The returned assignment is derived only from the authoritative branch
    table and the accepted/certified source segment.  Identity, colour,
    payload, and shuttle number are deliberately absent.
    """

    required = {
        str(device).strip().upper(): str(state).strip().upper()
        for device, state in dict(branch['switches']).items()
    }
    source_public_segment = _public_segment_for_side(
        side,
        source_position.get('segment'),
    )
    source_gate = INTERIOR_LOOP_GATE_BY_PUBLIC_SEGMENT.get(
        source_public_segment,
        '',
    )
    if source_gate:
        source_branch = INTERIOR_HOLDING_BRANCH_BY_GATE[source_gate]
        required[str(source_branch['exit_switch']).strip().upper()] = 'I'
    return required


def _required_switch_words(
    required_switches: dict[str, Any],
) -> dict[str, str]:
    """Normalize a complete route assignment for audit comparisons."""

    return {
        str(device).strip().upper(): (
            'interior'
            if str(state).strip().upper() in {'I', 'INTERIOR'}
            else 'exterior'
        )
        for device, state in required_switches.items()
    }


def _active_clearance_assignment_differs(
    *,
    side: str,
    devices: dict[str, dict[str, str]],
    required_switches: dict[str, Any],
) -> bool:
    required = _required_switch_words(required_switches)
    return bool(
        set(required) != set(DEVICE_NAMES)
        or any(
            devices['switches'].get(_switch_object(side, device)) != state
            for device, state in required.items()
        )
    )


def _topology_route_length_m(
    *,
    side: str,
    route_entry: dict[str, Any],
) -> float:
    """Return the travelled length of one authoritative topology route."""

    lengths = rail_segment_lengths(side)
    return sum(
        float(lengths[block['segment']])
        * abs(
            float(block['end_s_ratio'])
            - float(block['start_s_ratio'])
        )
        for block in route_entry.get('route_blocks') or []
    )


def _hypothetical_verified_slot_relocation(
    *,
    fleet: dict[str, Any],
    shuttle: str,
    source_slot: str,
    target_slot: str,
) -> dict[str, Any]:
    """Return a planning-only fleet after one sensor-verifiable slot move.

    This is used only to prove that a proposed first vacancy-rotation move can
    unlock the next receding-horizon action.  It does not assert that either
    hypothetical move happened, and none of these positions are emitted as
    observed facts.  The real executor still performs one supervised action,
    waits for its identity-bearing slot sensor, and rebuilds from vision.
    """

    future = copy.deepcopy(fleet)
    if future['slot_occupancy'].get(source_slot) != shuttle:
        raise PddlProblemBuildError(
            f'cannot simulate {shuttle!r} leaving unoccupied slot '
            f'{source_slot!r}'
        )
    if future['slot_occupancy'].get(target_slot):
        raise PddlProblemBuildError(
            f'cannot simulate {shuttle!r} entering occupied slot '
            f'{target_slot!r}'
        )
    side = _shuttle_side(shuttle)
    if not target_slot.startswith(f'{side}_slot_'):
        raise PddlProblemBuildError(
            f'cannot simulate cross-side slot relocation for {shuttle!r}'
        )
    future['slot_occupancy'][source_slot] = ''
    future['slot_occupancy'][target_slot] = shuttle
    future['location_slot_by_shuttle'][shuttle] = target_slot
    raw_position = future['rail_position_by_shuttle'].get(shuttle)
    if not isinstance(raw_position, dict):
        raise PddlProblemBuildError(
            f'cannot simulate slot relocation without visual geometry for '
            f'{shuttle!r}'
        )
    future['planning_rail_position_by_shuttle'][shuttle] = {
        **_verified_slot_planning_position(
            visual_position=raw_position,
            slot_object=target_slot,
            topology=_planning_rail_topology(side),
        ),
        'source': 'conditional_verified_slot_relocation_search',
        'hypothetical_for_search_only': True,
    }
    return future


def _conditional_exterior_vacancy_release(
    *,
    fleet: dict[str, Any],
    devices: dict[str, dict[str, str]],
    obstacles: dict[str, list[str]],
    route_clearance: dict[str, Any],
    side: str,
    selected_shuttle: str,
    selected_target_slot: str,
    primary_blocker: str,
    target_occupant: str,
    observed_blockers: list[str],
) -> dict[str, Any] | None:
    """Find a non-target exterior move that causally releases the goal route.

    A segment-origin target must not be sent around the opposite interior loop
    merely because the first exact-slot blocker cannot move immediately.  This
    search considers the other *observed route blockers*, simulates one clear
    slot move, and requires proof that the primary blocker can then park in a
    non-goal slot and leave the selected shuttle's topology route clear.  Only
    the first move is returned; every later step remains conditional on a fresh
    observation.
    """

    if (
        obstacles.get(side)
        or not primary_blocker
        or primary_blocker == target_occupant
        or primary_blocker == selected_shuttle
    ):
        return None
    runtime_clearances = dict(
        fleet.get('runtime_clearance_certificates') or {}
    )
    primary_source = fleet['location_slot_by_shuttle'].get(
        primary_blocker,
        '',
    )
    if not primary_source or primary_blocker in runtime_clearances:
        return None

    current_pairs = {
        (entry['from_slot'], entry['to_slot']): entry
        for entry in route_clearance['provenance']['pairs']
        if entry.get('side') == side
    }
    free_slots = sorted(
        slot
        for slot, occupant in fleet['slot_occupancy'].items()
        if (
            slot.startswith(f'{side}_slot_')
            and slot != selected_target_slot
            and not occupant
        )
    )
    candidates: list[dict[str, Any]] = []
    for dependency in observed_blockers[1:]:
        if (
            dependency in {selected_shuttle, primary_blocker}
            or dependency in runtime_clearances
        ):
            continue
        dependency_source = fleet['location_slot_by_shuttle'].get(
            dependency,
            '',
        )
        if not dependency_source:
            continue
        for dependency_target in free_slots:
            dependency_pair = (dependency_source, dependency_target)
            if dependency_pair not in route_clearance['clear_pairs']:
                continue
            try:
                after_dependency = _hypothetical_verified_slot_relocation(
                    fleet=fleet,
                    shuttle=dependency,
                    source_slot=dependency_source,
                    target_slot=dependency_target,
                )
            except PddlProblemBuildError:
                continue
            after_dependency_clearance = _route_clearance_snapshot(
                fleet=after_dependency
            )
            for primary_target, occupant in sorted(
                after_dependency['slot_occupancy'].items()
            ):
                if (
                    occupant
                    or not primary_target.startswith(f'{side}_slot_')
                    or primary_target == selected_target_slot
                    or (
                        primary_source,
                        primary_target,
                    ) not in after_dependency_clearance['clear_pairs']
                ):
                    continue
                try:
                    after_primary = _hypothetical_verified_slot_relocation(
                        fleet=after_dependency,
                        shuttle=primary_blocker,
                        source_slot=primary_source,
                        target_slot=primary_target,
                    )
                    selected_route = _topology_route_entry(
                        fleet=after_primary,
                        devices=devices,
                        obstacles=obstacles,
                        shuttle=selected_shuttle,
                        target_slot=_slot_number_from_object(
                            selected_target_slot
                        ),
                    )
                except PddlProblemBuildError:
                    continue
                if not selected_route.get('route_clear'):
                    continue
                future_pairs = {
                    (entry['from_slot'], entry['to_slot']): entry
                    for entry in after_dependency_clearance[
                        'provenance'
                    ]['pairs']
                    if entry.get('side') == side
                }
                primary_pair = (primary_source, primary_target)
                dependency_route = current_pairs.get(dependency_pair) or {}
                primary_route = future_pairs.get(primary_pair) or {}
                dependency_length_m = _topology_route_length_m(
                    side=side,
                    route_entry=dependency_route,
                )
                primary_length_m = _topology_route_length_m(
                    side=side,
                    route_entry=primary_route,
                )
                candidates.append({
                    'shuttle': dependency,
                    'source_slot': dependency_source,
                    'target_slot': dependency_target,
                    'dependency_route_length_m': dependency_length_m,
                    'future_primary_blocker': primary_blocker,
                    'future_primary_source_slot': primary_source,
                    'future_primary_target_slot': primary_target,
                    'future_primary_route_length_m': primary_length_m,
                    'future_selected_route': selected_route,
                    'total_conditional_release_length_m': (
                        dependency_length_m + primary_length_m
                    ),
                })
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda candidate: (
            float(candidate['total_conditional_release_length_m']),
            float(candidate['dependency_route_length_m']),
            str(candidate['target_slot']),
            str(candidate['shuttle']),
        ),
    )


def _active_interior_clearance_gate(
    *,
    side: str,
    devices: dict[str, dict[str, str]],
) -> str:
    observed_switches = {
        device: devices['switches'].get(_switch_object(side, device))
        for device in DEVICE_NAMES
    }
    observed_stoppers = {
        device: devices['stoppers'].get(_stopper_object(side, device))
        for device in DEVICE_NAMES
    }
    for gate, branch in INTERIOR_HOLDING_BRANCH_BY_GATE.items():
        target_gate = str(branch['gate_switch']).strip().upper()
        target_exit = str(branch['exit_switch']).strip().upper()
        # A cross-branch transfer may additionally keep the *source* merge
        # switch interior.  The target diverger, target merge, and unique
        # closed target stopper are the authoritative clearance-mode
        # signature; unrelated switches need not match the exterior-entry
        # template exactly.
        if (
            observed_switches.get(target_gate) == 'interior'
            and observed_switches.get(target_exit) == 'interior'
            and observed_stoppers.get(target_exit) == 'closed'
            and all(
                state == 'open'
                for device, state in observed_stoppers.items()
                if device != target_exit
            )
        ):
            return gate
    return ''


def _interior_holding_pose_candidates(
    *,
    side: str,
    public_segment: str,
    occupied_s_m: list[float],
    required_center_spacing_m: float,
) -> list[float]:
    """Enumerate safe holding centres from one authoritative interior branch.

    The rail is forward-only, so the farthest reachable free pose is preferred.
    This leaves the entry-side portion available for a later dependency shuttle
    and avoids the old middle pose that made both remaining holding regions
    unusable.  The grid is a planning search only; execution remains guarded by
    the identity-bearing A3 interior sensor, bounded motion, OFF confirmation,
    and a fresh visual frame.
    """

    segment_length_m = float(
        public_rail_segment_lengths(side)[public_segment]
    )
    lower = _INTERIOR_HOLDING_END_MARGIN_M
    upper = segment_length_m - _INTERIOR_HOLDING_END_MARGIN_M
    if upper < lower:
        return []
    steps = int(math.floor((upper - lower) / _INTERIOR_HOLDING_GRID_STEP_M))
    candidates = {
        round(lower + index * _INTERIOR_HOLDING_GRID_STEP_M, 6)
        for index in range(steps + 1)
    }
    candidates.update({round(lower, 6), round(upper, 6)})
    return [
        candidate
        for candidate in sorted(candidates, reverse=True)
        if all(
            abs(candidate - float(occupied)) >= required_center_spacing_m
            for occupied in occupied_s_m
        )
    ]


def _resolve_selected_interior_buffer_choreography(
    *,
    fleet: dict[str, Any],
    devices: dict[str, dict[str, str]],
    obstacles: dict[str, list[str]],
    primary_blocker: str,
    selected_shuttle: str,
    branch: dict[str, Any],
    required_center_spacing_m: float,
) -> dict[str, Any]:
    """Open a second holding pose behind an interior goal shuttle.

    On a full four-shuttle rail the user's shuttle can already be stopped in
    A12I/A34I while the goal-slot occupant waits at the branch entrance.  The
    physically shortest safe choreography is then:

    1. advance the selected shuttle to the nearest forward holding pose;
    2. reobserve;
    3. move the goal-slot occupant into the newly opened rear holding pose;
    4. reobserve and let the selected shuttle continue to the requested slot.

    This search uses only authoritative topology, accepted visual positions,
    and a validated executor-owned interior stop certificate.  Identity,
    colour, payload, and shuttle number do not participate in the choice.
    """

    side = _shuttle_side(selected_shuttle)
    if _shuttle_side(primary_blocker) != side or obstacles.get(side):
        return {'resolved': False, 'reason': 'side_or_obstacle_conflict'}
    public_segment = str(branch['target_segment']).strip().upper()
    internal_segment = _interior_internal_segment(side, public_segment)
    planning_positions = (
        fleet.get('planning_rail_position_by_shuttle')
        or fleet['rail_position_by_shuttle']
    )
    selected_position = planning_positions.get(selected_shuttle) or {}
    selected_required_switches = _interior_route_switches_for_source(
        side=side,
        branch=branch,
        source_position=selected_position,
    )
    if not _is_interior_holding_position(
        side=side,
        position=selected_position,
        public_segment=public_segment,
    ):
        return {'resolved': False, 'reason': 'selected_not_on_target_branch'}
    origin_certificate = dict(
        (fleet.get('runtime_clearance_certificates') or {}).get(
            selected_shuttle
        )
        or {}
    )
    if (
        origin_certificate.get('target_segment') != public_segment
        or origin_certificate.get('controller_stop_confirmed') is not True
        or origin_certificate.get('bounded_commanded_motion_completed')
        is not True
        or origin_certificate.get(
            'controller_position_fields_used_for_localization'
        )
        is not False
    ):
        return {
            'resolved': False,
            'reason': 'selected_interior_origin_is_not_certified_stopped',
        }
    segment_length_m = float(rail_segment_lengths(side)[internal_segment])
    selected_origin_s_m = float(origin_certificate['target_s_m'])
    occupied_other_s_m = [
        float(position['s_ratio']) * float(position['segment_length_m'])
        for shuttle, position in planning_positions.items()
        if shuttle not in {selected_shuttle, primary_blocker}
        and position.get('side') == side
        and _is_interior_holding_position(
            side=side,
            position=position,
            public_segment=public_segment,
        )
    ]

    # First prefer the blocker itself when it can already use the rear pose.
    direct_targets = _interior_holding_pose_candidates(
        side=side,
        public_segment=public_segment,
        occupied_s_m=[*occupied_other_s_m, selected_origin_s_m],
        required_center_spacing_m=required_center_spacing_m,
    )
    direct = _resolve_interior_clearance_dependency(
        fleet=fleet,
        devices=devices,
        obstacles=obstacles,
        primary_blocker=primary_blocker,
        selected_shuttle=selected_shuttle,
        branch=branch,
        target_s_m_candidates=direct_targets,
    )
    if (
        direct.get('resolved')
        and direct.get('first_movable_shuttle') == primary_blocker
    ):
        return {
            **direct,
            'motion_mode': 'enter_interior_branch',
            'selected_origin_s_m': selected_origin_s_m,
            'selection_policy': (
                'direct_rear_holding_pose_before_selected_goal_completion'
            ),
        }

    # The blocker cannot enter yet. Find the *nearest* forward pose for the
    # selected shuttle that leaves one physically separated rear pose.  The
    # future route is checked on a copied state because execution still stops
    # after the first move and requires a fresh accepted observation.
    selected_targets = sorted(_interior_holding_pose_candidates(
        side=side,
        public_segment=public_segment,
        occupied_s_m=occupied_other_s_m,
        required_center_spacing_m=required_center_spacing_m,
    ))
    attempts: list[dict[str, Any]] = []
    for selected_target_s_m in selected_targets:
        if (
            selected_target_s_m
            <= selected_origin_s_m + INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M
        ):
            continue
        primary_targets = [
            value
            for value in _interior_holding_pose_candidates(
                side=side,
                public_segment=public_segment,
                occupied_s_m=[*occupied_other_s_m, selected_target_s_m],
                required_center_spacing_m=required_center_spacing_m,
            )
            if value < selected_target_s_m
        ]
        if not primary_targets:
            attempts.append({
                'selected_target_s_m': selected_target_s_m,
                'reason': 'no_separated_rear_holding_pose',
            })
            continue
        try:
            selected_route = _topology_route_entry(
                fleet=fleet,
                devices=devices,
                obstacles=obstacles,
                shuttle=selected_shuttle,
                target_slot='4',
                target_position=(
                    internal_segment,
                    selected_target_s_m / segment_length_m,
                ),
                required_switch_states=selected_required_switches,
            )
        except PddlProblemBuildError as exc:
            attempts.append({
                'selected_target_s_m': selected_target_s_m,
                'reason': str(exc),
            })
            continue
        if selected_route.get('blockers'):
            attempts.append({
                'selected_target_s_m': selected_target_s_m,
                'reason': 'selected_advance_route_blocked',
                'blockers': list(selected_route['blockers']),
            })
            continue

        for primary_target_s_m in primary_targets:
            future_fleet = copy.deepcopy(fleet)
            future_position = future_fleet[
                'planning_rail_position_by_shuttle'
            ][selected_shuttle]
            future_ratio = selected_target_s_m / segment_length_m
            future_position.update({
                'segment': internal_segment,
                's_ratio': future_ratio,
                'segment_length_m': segment_length_m,
                'effect_certificate_target_s_m': selected_target_s_m,
                # This is a conditional topology proof, not a prediction. The
                # actual next problem is rebuilt from a fresh visual frame.
                'raw_visual_segment': internal_segment,
                'raw_visual_s_ratio': future_ratio,
            })
            future_certificate = future_fleet[
                'runtime_clearance_certificates'
            ][selected_shuttle]
            future_certificate.update({
                'target_s_m': selected_target_s_m,
                'observed_segment': public_segment,
                'observed_s_m': selected_target_s_m,
            })
            primary_required_switches = _interior_route_switches_for_source(
                side=side,
                branch=branch,
                source_position=(
                    future_fleet['planning_rail_position_by_shuttle'].get(
                        primary_blocker
                    )
                    or {}
                ),
            )
            try:
                primary_route = _topology_route_entry(
                    fleet=future_fleet,
                    devices=devices,
                    obstacles=obstacles,
                    shuttle=primary_blocker,
                    target_slot='4',
                    target_position=(
                        internal_segment,
                        primary_target_s_m / segment_length_m,
                    ),
                    required_switch_states=primary_required_switches,
                )
            except PddlProblemBuildError:
                continue
            if primary_route.get('blockers'):
                continue
            return {
                'resolved': True,
                'primary_blocker': primary_blocker,
                'first_movable_shuttle': selected_shuttle,
                'target_s_m': selected_target_s_m,
                'target_s_ratio': future_ratio,
                'dependency_chain': [primary_blocker, selected_shuttle],
                'route_entries': [selected_route],
                'future_primary_route': primary_route,
                'future_primary_target_s_m': primary_target_s_m,
                'motion_mode': 'advance_within_interior_branch',
                'motion_origin_s_m': selected_origin_s_m,
                'bounded_motion_distance_m': (
                    selected_target_s_m - selected_origin_s_m
                ),
                'origin_clearance_proof': {
                    'identity': origin_certificate['identity'],
                    'target_segment': public_segment,
                    'target_s_m': selected_origin_s_m,
                    'entry_sensor': origin_certificate['entry_sensor'],
                    'entry_sensor_identity_confirmed': True,
                    'controller_stop_confirmed': True,
                    'bounded_commanded_motion_completed': True,
                    'controller_position_fields_used_for_localization': False,
                },
                'gate_switch': str(branch['gate_switch']),
                'target_segment': public_segment,
                'required_switches': selected_required_switches,
                'future_primary_required_switches': (
                    primary_required_switches
                ),
                'selection_policy': (
                    'nearest_forward_selected_pose_that_opens_a_separated_'
                    'rear_holding_pose'
                ),
                'controller_position_fields_used_for_localization': False,
            }
    return {
        'resolved': False,
        'primary_blocker': primary_blocker,
        'first_movable_shuttle': '',
        'dependency_chain': [primary_blocker],
        'attempts': attempts,
        'gate_switch': str(branch['gate_switch']),
        'target_segment': public_segment,
        'reason': 'no_safe_selected_interior_advance',
        'controller_position_fields_used_for_localization': False,
    }


def _resolve_interior_clearance_dependency(
    *,
    fleet: dict[str, Any],
    devices: dict[str, dict[str, str]],
    obstacles: dict[str, list[str]],
    primary_blocker: str,
    selected_shuttle: str,
    branch: dict[str, Any],
    target_s_m_candidates: list[float],
    preferred_dependency_shuttles: list[str] | None = None,
) -> dict[str, Any]:
    """Find the first causally movable blocker for one interior branch.

    This follows the complete authoritative topology under the held clearance
    switch assignment. If the primary blocker cannot reach the branch because a
    second shuttle is ahead, that second shuttle becomes the next dependency;
    the search continues until it finds a shuttle whose path is clear.  No
    colour, numeric identity, or fixed slot ordering participates in the
    choice. Cycles, external obstacles, and an occupied one-way target branch
    approach remain fail-closed.
    """

    side = _shuttle_side(primary_blocker)
    gate = str(branch['gate_switch']).strip().upper()
    public_segment = str(branch['target_segment']).strip().upper()
    internal_segment = _interior_internal_segment(side, public_segment)
    internal_length_m = float(rail_segment_lengths(side)[internal_segment])
    planning_positions = (
        fleet.get('planning_rail_position_by_shuttle')
        or fleet['rail_position_by_shuttle']
    )
    attempts: list[dict[str, Any]] = []
    preferred_dependencies = [
        shuttle
        for shuttle in (preferred_dependency_shuttles or [])
        if shuttle not in {primary_blocker, selected_shuttle}
    ]
    for target_s_m in target_s_m_candidates:
        target_s_ratio = float(target_s_m) / max(internal_length_m, 1e-9)
        mover = primary_blocker
        dependency_chain = [primary_blocker]
        route_entries: list[dict[str, Any]] = []
        rejected_reason = ''
        while True:
            mover_required_switches = _interior_route_switches_for_source(
                side=side,
                branch=branch,
                source_position=planning_positions.get(mover) or {},
            )
            try:
                entry = _topology_route_entry(
                    fleet=fleet,
                    devices=devices,
                    obstacles=obstacles,
                    shuttle=mover,
                    target_slot='4',
                    target_position=(internal_segment, target_s_ratio),
                    required_switch_states=mover_required_switches,
                )
            except PddlProblemBuildError as exc:
                rejected_reason = str(exc)
                # A blocker already downstream of this gate cannot enter its
                # holding branch under
                # one held switch assignment without first completing a loop.
                # On a full rail, create the first vacancy by staging another
                # same-side shuttle whose topology route to A34I is directly
                # clear. Prefer shuttles already proven to block the user's
                # route, then consider the remaining non-selected fleet.
                if mover == primary_blocker and len(dependency_chain) == 1:
                    fallback_candidates = preferred_dependencies + sorted(
                        shuttle
                        for shuttle, candidate_position in planning_positions.items()
                        if candidate_position.get('side') == side
                        and shuttle not in {
                            primary_blocker,
                            selected_shuttle,
                            *preferred_dependencies,
                        }
                        and not _is_interior_holding_position(
                            side=side,
                            position=candidate_position,
                            public_segment=public_segment,
                        )
                    )
                    selected_position = planning_positions.get(
                        selected_shuttle
                    ) or {}
                    if (
                        selected_shuttle not in fallback_candidates
                        and selected_position.get('side') == side
                        and not _is_interior_holding_position(
                            side=side,
                            position=selected_position,
                            public_segment=public_segment,
                        )
                    ):
                        fallback_candidates.append(selected_shuttle)
                    for fallback in fallback_candidates:
                        fallback_required_switches = (
                            _interior_route_switches_for_source(
                                side=side,
                                branch=branch,
                                source_position=(
                                    planning_positions.get(fallback) or {}
                                ),
                            )
                        )
                        try:
                            fallback_entry = _topology_route_entry(
                                fleet=fleet,
                                devices=devices,
                                obstacles=obstacles,
                                shuttle=fallback,
                                target_slot='4',
                                target_position=(
                                    internal_segment,
                                    target_s_ratio,
                                ),
                                required_switch_states=(
                                    fallback_required_switches
                                ),
                            )
                        except PddlProblemBuildError:
                            continue
                        if fallback_entry.get('blockers') or obstacles.get(side):
                            continue
                        return {
                            'resolved': True,
                            'primary_blocker': primary_blocker,
                            'first_movable_shuttle': fallback,
                            'target_s_m': float(target_s_m),
                            'target_s_ratio': target_s_ratio,
                            'dependency_chain': [primary_blocker, fallback],
                            'route_entries': [fallback_entry],
                            'dependency_mode': (
                                'topology_vacancy_seed_for_downstream_blocker'
                            ),
                            'gate_switch': gate,
                            'target_segment': public_segment,
                            'required_switches': (
                                fallback_required_switches
                            ),
                            'selection_policy': (
                                'authoritative_topology_dependency_chain_then_'
                                'farthest_capacity_preserving_holding_pose'
                            ),
                            'controller_position_fields_used_for_localization': False,
                        }
                break
            route_entries.append(entry)
            blockers = [
                blocker
                for blocker in entry.get('blockers') or []
                if blocker != mover
            ]
            if obstacles.get(side):
                rejected_reason = 'external_obstacle_blocks_clearance_route'
                break
            if not blockers:
                return {
                    'resolved': True,
                    'primary_blocker': primary_blocker,
                    'first_movable_shuttle': mover,
                    'target_s_m': float(target_s_m),
                    'target_s_ratio': target_s_ratio,
                    'dependency_chain': dependency_chain,
                    'route_entries': route_entries,
                    'gate_switch': gate,
                    'target_segment': public_segment,
                    'required_switches': mover_required_switches,
                    'selection_policy': (
                        'authoritative_topology_dependency_chain_then_'
                        'farthest_capacity_preserving_holding_pose'
                    ),
                    'controller_position_fields_used_for_localization': False,
                }
            next_blocker = blockers[0]
            next_position = planning_positions.get(next_blocker) or {}
            if next_blocker in dependency_chain:
                rejected_reason = 'cyclic_clearance_dependency'
                break
            if _is_interior_holding_position(
                side=side,
                position=next_position,
                public_segment=public_segment,
            ):
                rejected_reason = (
                    'one_way_interior_occupant_blocks_requested_holding_pose'
                )
                break
            dependency_chain.append(next_blocker)
            mover = next_blocker
            if len(dependency_chain) > len(planning_positions):
                rejected_reason = 'clearance_dependency_depth_exceeds_fleet'
                break
        attempts.append({
            'target_s_m': float(target_s_m),
            'dependency_chain': dependency_chain,
            'reason': rejected_reason or 'unresolved_clearance_dependency',
        })
    return {
        'resolved': False,
        'primary_blocker': primary_blocker,
        'first_movable_shuttle': '',
        'target_s_m': None,
        'dependency_chain': [primary_blocker],
        'attempts': attempts,
        'gate_switch': gate,
        'target_segment': public_segment,
        'required_switches': dict(branch['switches']),
        'selection_policy': (
            'authoritative_topology_dependency_chain_then_'
            'farthest_capacity_preserving_holding_pose'
        ),
        'controller_position_fields_used_for_localization': False,
    }


def _resolve_shortest_direct_interior_blocker_release(
    *,
    fleet: dict[str, Any],
    devices: dict[str, dict[str, str]],
    obstacles: dict[str, list[str]],
    primary_blocker: str,
    selected_shuttle: str,
    gates: tuple[str, ...],
    required_center_spacing_m: float,
) -> dict[str, Any]:
    """Choose the shortest one-move release of a proved route blocker.

    The blocker is tested against every currently admissible
    interior branch and every capacity-preserving holding pose.  A result is
    accepted only when that blocker itself can move now; dependency moves are
    deliberately excluded because they do not release the user's route in one
    action. Identity, colour, payload, and exterior slot cardinality are absent
    from the ranking.
    """

    side = _shuttle_side(selected_shuttle)
    positions = (
        fleet.get('planning_rail_position_by_shuttle')
        or fleet['rail_position_by_shuttle']
    )
    runtime_clearances = dict(
        fleet.get('runtime_clearance_certificates') or {}
    )
    branch_search: list[dict[str, Any]] = []
    for gate in gates:
        branch = dict(INTERIOR_HOLDING_BRANCH_BY_GATE[gate])
        public_segment = str(branch['target_segment']).strip().upper()
        occupied_s_m = [
            float(certificate['target_s_m'])
            for identity, certificate in runtime_clearances.items()
            if identity != primary_blocker
            and certificate.get('side') == side
            and str(certificate.get('target_segment') or '').upper()
            == public_segment
        ] + [
            float(position['s_ratio'])
            * float(position['segment_length_m'])
            for shuttle, position in positions.items()
            if shuttle not in runtime_clearances
            and shuttle != primary_blocker
            and position.get('side') == side
            and _is_interior_holding_position(
                side=side,
                position=position,
                public_segment=public_segment,
            )
        ]
        pose_candidates = _interior_holding_pose_candidates(
            side=side,
            public_segment=public_segment,
            occupied_s_m=occupied_s_m,
            required_center_spacing_m=required_center_spacing_m,
        )
        internal_segment = _interior_internal_segment(
            side,
            public_segment,
        )
        internal_length_m = float(
            rail_segment_lengths(side)[internal_segment]
        )
        attempts: list[dict[str, Any]] = []
        resolution: dict[str, Any] | None = None
        required_switches = _interior_route_switches_for_source(
            side=side,
            branch=branch,
            source_position=positions.get(primary_blocker) or {},
        )
        for target_s_m in pose_candidates:
            target_s_ratio = float(target_s_m) / max(
                internal_length_m,
                1e-9,
            )
            try:
                route_entry = _topology_route_entry(
                    fleet=fleet,
                    devices=devices,
                    obstacles=obstacles,
                    shuttle=primary_blocker,
                    target_slot='4',
                    target_position=(internal_segment, target_s_ratio),
                    required_switch_states=required_switches,
                )
            except PddlProblemBuildError as exc:
                attempts.append({
                    'target_s_m': float(target_s_m),
                    'reason': str(exc),
                })
                continue
            blockers = [
                blocker
                for blocker in route_entry.get('blockers') or []
                if blocker != primary_blocker
            ]
            if obstacles.get(side) or blockers:
                attempts.append({
                    'target_s_m': float(target_s_m),
                    'reason': (
                        'external_obstacle_blocks_clearance_route'
                        if obstacles.get(side)
                        else 'direct_interior_route_blocked'
                    ),
                    'blockers': blockers,
                })
                continue
            resolution = {
                'resolved': True,
                'primary_blocker': primary_blocker,
                'first_movable_shuttle': primary_blocker,
                'target_s_m': float(target_s_m),
                'target_s_ratio': target_s_ratio,
                'dependency_chain': [primary_blocker],
                'route_entries': [route_entry],
                'gate_switch': str(branch['gate_switch']),
                'target_segment': public_segment,
                'required_switches': required_switches,
                'controller_position_fields_used_for_localization': False,
            }
            break
        if resolution is None:
            resolution = {
                'resolved': False,
                'primary_blocker': primary_blocker,
                'first_movable_shuttle': '',
                'target_s_m': None,
                'dependency_chain': [primary_blocker],
                'attempts': attempts,
                'gate_switch': str(branch['gate_switch']),
                'target_segment': public_segment,
                'required_switches': required_switches,
                'controller_position_fields_used_for_localization': False,
            }
        elif _is_interior_holding_position(
            side=side,
            position=positions.get(primary_blocker) or {},
            public_segment=public_segment,
        ):
            # The blocker is already stopped on this one-way branch. Treating
            # the next forward holding pose as a fresh branch entry makes the
            # receding-horizon problem unstable: after ``begin clearance`` it
            # can choose an exterior relocation, finish clearance, and then
            # rediscover this shorter interior route forever. Bind the move to
            # the persisted entry/stop effect and advance monotonically within
            # the branch instead.
            try:
                origin = normalize_runtime_clearance_certificate(
                    primary_blocker,
                    runtime_clearances.get(primary_blocker),
                )
            except ValueError as exc:
                resolution = {
                    **resolution,
                    'resolved': False,
                    'reason': (
                        'existing_interior_blocker_has_no_valid_origin_proof:'
                        f'{exc}'
                    ),
                }
            else:
                origin_segment = str(
                    origin.get('target_segment') or ''
                ).strip().upper()
                origin_s_m = float(origin['target_s_m'])
                target_s_m = float(resolution['target_s_m'])
                if (
                    origin_segment != public_segment
                    or target_s_m
                    <= origin_s_m + INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M
                ):
                    resolution = {
                        **resolution,
                        'resolved': False,
                        'reason': (
                            'existing_interior_blocker_has_no_forward_'
                            'separated_holding_pose'
                        ),
                    }
                else:
                    resolution.update({
                        'motion_mode': 'advance_within_interior_branch',
                        'motion_origin_s_m': origin_s_m,
                        'bounded_motion_distance_m': (
                            target_s_m - origin_s_m
                        ),
                        'origin_clearance_proof': {
                            'identity': origin['identity'],
                            'target_segment': origin_segment,
                            'target_s_m': origin_s_m,
                            'entry_sensor': origin['entry_sensor'],
                            'entry_sensor_identity_confirmed': True,
                            'controller_stop_confirmed': True,
                            'bounded_commanded_motion_completed': True,
                            'controller_position_fields_used_for_localization': (
                                False
                            ),
                        },
                    })
        elif resolution.get('resolved'):
            resolution['motion_mode'] = 'enter_interior_branch'
        route_entries = list(resolution.get('route_entries') or [])
        resolution.update({
            'branch': branch,
            'occupied_target_s_m': sorted(occupied_s_m),
            'candidate_pose_count': len(pose_candidates),
            'first_mover_route_length_m': _topology_route_length_m(
                side=side,
                route_entry=(route_entries[-1] if route_entries else {}),
            ),
            'direct_blocker_release': bool(resolution['resolved']),
        })
        branch_search.append(resolution)

    direct_results = [
        result
        for result in branch_search
        if result['direct_blocker_release']
    ]
    if not direct_results:
        return {
            'resolved': False,
            'branch_search': branch_search,
            'reason': 'no_direct_interior_blocker_release',
            'controller_position_fields_used_for_localization': False,
        }
    selected = min(
        direct_results,
        key=lambda result: (
            float(result['first_mover_route_length_m']),
            len(result.get('dependency_chain') or []),
            str(result.get('gate_switch') or ''),
        ),
    )
    return {
        **selected,
        'branch_search': branch_search,
        'selection_policy': (
            'fewest_blocker_release_actions_then_shortest_'
            'authoritative_interior_route'
        ),
        'controller_position_fields_used_for_localization': False,
    }


def _target_blocker_clearance_plan(
    *,
    fleet: dict[str, Any],
    devices: dict[str, dict[str, str]],
    obstacles: dict[str, list[str]],
    goal_data: dict[str, Any],
    route_clearance: dict[str, Any],
) -> dict[str, Any]:
    base = {
        'required': False,
        'selected_shuttle': goal_data.get('selected_shuttle', ''),
        'source_slot': '',
        'target_slot': '',
        'ordered_relocations': [],
        'execution_policy': (
            'enter clearance mode once, execute supervised relocations with '
            'fresh re-observation after each, restore the normal route once '
            'after the pending-clearance count reaches zero'
        ),
        'continuous_motion_owner': 'supervisor_and_kinematic_safety_layer',
        'unsupported_if_more_than_two_blockers': False,
        'stale_multi_destination_preallocation_used': False,
        'receding_horizon_clearance': False,
    }
    if goal_data.get('goal_type') != 'transport' or not goal_data.get('target_slot'):
        return base
    if goal_data.get('planner_selects_candidate'):
        base['selection_deferred_to_plansys2'] = True
        base['candidate_specific_route_facts_emitted'] = True
        base['clearance_plan_is_provisional_until_candidate_selection'] = True
    side = goal_data['side']
    selected = goal_data['selected_shuttle']
    source = fleet['location_slot_by_shuttle'].get(selected, '')
    target = _slot_object(side, goal_data['target_slot'])
    base['source_slot'] = source
    base['target_slot'] = target
    topology_route = (
        route_clearance.get('topology_routes', {})
        .get('by_shuttle_and_slot', {})
        .get((selected, target))
    )
    if source:
        exterior_blocker_ids = list(
            route_clearance['blockers_by_pair'].get((source, target), ())
        )
        exterior_route_pair = next(
            (
                pair
                for pair in route_clearance['provenance']['pairs']
                if pair['from_slot'] == source and pair['to_slot'] == target
            ),
            {},
        )
        topology_blocker_ids = list(
            (topology_route or {}).get('blockers') or []
        )
        # A known-slot topology alternative is valid only when it is already
        # clear.  Selecting a merely "less blocked" alternative would enter
        # the exact-slot clearance actions and later assert exterior
        # route_clear_between, which is not a proof for that alternate route.
        # Blocked alternatives therefore stay out of this action family; the
        # normal exact-slot route and its existing clearance semantics remain
        # authoritative.
        if (
            topology_route
            and not topology_blocker_ids
            and len(topology_blocker_ids) < len(exterior_blocker_ids)
        ):
            blocker_ids = topology_blocker_ids
            route_pair = topology_route
            base.update({
                'source_kind': 'slot_with_authoritative_topology_alternative',
                'source_block': topology_route['source_block'],
                'source_segment': topology_route['source_segment'],
                'source_public_segment': topology_route[
                    'source_public_segment'
                ],
                'source_s_ratio': topology_route['source_s_ratio'],
                'normal_exterior_blockers': exterior_blocker_ids,
                'selected_topology_blockers': topology_blocker_ids,
                'topology_alternative_selected': True,
            })
        else:
            blocker_ids = exterior_blocker_ids
            route_pair = exterior_route_pair
            base['source_kind'] = 'slot'
            base['topology_alternative_selected'] = False
    elif topology_route:
        blocker_ids = list(topology_route.get('blockers') or [])
        route_pair = topology_route
        base.update({
            'source_kind': 'accepted_visual_continuous_position',
            'source_block': topology_route['source_block'],
            'source_segment': topology_route['source_segment'],
            'source_public_segment': topology_route['source_public_segment'],
            'source_s_ratio': topology_route['source_s_ratio'],
        })
    else:
        blocker_ids = []
        route_pair = {}
    if not blocker_ids:
        return base
    positions = (
        fleet.get('planning_rail_position_by_shuttle')
        or fleet['rail_position_by_shuttle']
    )
    if source:
        route_occurrences: dict[str, list[int]] = {}
        for index, block in enumerate(route_pair.get('route_blocks') or []):
            route_occurrences.setdefault(block['segment'], []).append(index)
        blocker_ids.sort(
            key=lambda shuttle: (
                max(route_occurrences.get(
                    positions[shuttle]['segment'],
                    [-1],
                )),
                float(positions[shuttle]['s_ratio']),
                shuttle,
            ),
            reverse=True,
        )
    # Topology-origin entries already preserve the first exact overlap along
    # the selected candidate trace.  Do not reorder them through a lossy
    # segment->index map: wrapped routes may visit one segment more than once.
    # The executive executes one atomic relocation and then rebuilds the
    # complete problem from a fresh visual observation.  Freezing destinations
    # for every currently observed blocker was both stale and unnecessarily
    # incomplete: with four shuttles it tried to place all three blockers in
    # A34I before executing the first move, even though that first move frees
    # an exterior slot that the next fresh problem can reuse.  Plan only the
    # farthest-ahead obstruction in this observation.  The remaining blockers
    # stay explicit in the audit metadata and are reconsidered after the
    # verified move; no route is declared clear early.
    observed_blocker_ids = list(blocker_ids)
    runtime_clearances = dict(
        fleet.get('runtime_clearance_certificates') or {}
    )
    staged_interior_present = any(
        certificate.get('side') == side
        for certificate in runtime_clearances.values()
    ) or any(
        position.get('side') == side
        and _is_interior_holding_position(
            side=side,
            position=position,
        )
        for position in positions.values()
    )
    normal_device_route = all(
        devices['switches'].get(_switch_object(side, device)) == 'exterior'
        and devices['stoppers'].get(_stopper_object(side, device)) == 'open'
        for device in DEVICE_NAMES
    )
    active_clearance_gate = _active_interior_clearance_gate(
        side=side,
        devices=devices,
    )
    clearance_mode_active = bool(active_clearance_gate)
    if source and staged_interior_present:
        source_number = int(_slot_number_from_object(source))
        target_number = int(_slot_number_from_object(target))
        final_distance = (target_number - source_number) % 4
        clear_advances = []
        for distance in range(1, final_distance):
            number = ((source_number - 1 + distance) % 4) + 1
            candidate = _slot_object(side, str(number))
            if (
                not fleet['slot_occupancy'].get(candidate)
                and (source, candidate) in route_clearance['clear_pairs']
            ):
                clear_advances.append((distance, candidate))
        if clear_advances and clearance_mode_active:
            distance, intermediate = max(clear_advances)
            base.update({
                'required': False,
                'observed_blockers': observed_blocker_ids,
                'observed_blocker_count': len(observed_blocker_ids),
                'planned_relocations_this_observation': 0,
                'deferred_blockers_require_fresh_reobservation': (
                    observed_blocker_ids
                ),
                'receding_horizon_clearance': True,
                'clearance_mode_active': True,
                'clearance_pause_for_exterior_progress': {
                    'required': True,
                    'active_gate': active_clearance_gate,
                    'shuttle': selected,
                    'source_slot': source,
                    'next_reachable_slot': intermediate,
                    'final_target_slot': target,
                    'forward_slot_steps': distance,
                    'reason': (
                        'staged_interior_move_opened_shorter_exterior_'
                        'progress_for_selected_shuttle'
                    ),
                    'policy': (
                        'pause_clearance_then_advance_selected_and_'
                        'reobserve'
                    ),
                },
            })
            return base
        if clear_advances and normal_device_route:
            distance, intermediate = max(clear_advances)
            base.update({
                'required': False,
                'observed_blockers': observed_blocker_ids,
                'observed_blocker_count': len(observed_blocker_ids),
                'planned_relocations_this_observation': 0,
                'deferred_blockers_require_fresh_reobservation': (
                    observed_blocker_ids
                ),
                'receding_horizon_clearance': True,
                'intermediate_selected_advance': {
                    'required': True,
                    'shuttle': selected,
                    'source_slot': source,
                    'target_slot': intermediate,
                    'final_target_slot': target,
                    'forward_slot_steps': distance,
                    'reason': (
                        'advance_selected_into_farthest_clear_intermediate_'
                        'slot_before_remaining_blocker'
                    ),
                    'effect_verification': 'identity_bearing_slot_sensor',
                },
            })
            return base
    blocker_ids = blocker_ids[:1]
    base.update({
        'observed_blockers': observed_blocker_ids,
        'observed_blocker_count': len(observed_blocker_ids),
        'planned_relocations_this_observation': len(blocker_ids),
        'deferred_blockers_require_fresh_reobservation': observed_blocker_ids[1:],
        'receding_horizon_clearance': True,
    })
    # Reuse a verified free exterior slot when prior sequential goals already
    # left shuttles in A34I. A target-slot occupant can then park outside and
    # must not be forced into an already-full interior buffer. In a fresh task
    # with no staged interior shuttle, retain the established A34I clearance
    # strategy. Only routes clear in the accepted state are eligible.
    target_occupant = fleet['slot_occupancy'].get(target, '')
    clearance_spacing_m = (
        DEFAULT_SHUTTLE_LENGTH_M
        + DEFAULT_ROUTE_SAFETY_MARGIN_M
        + 2.0 * INTERIOR_LOOP_CLEAR_POSE_TOLERANCE_M
    )

    # An interior-origin goal can use either branch as a one-move holding
    # destination for a route blocker or the goal-slot occupant. This is useful
    # even when only
    # two exterior slots are occupied: the low slot count may mean that other
    # shuttles are already staged inside, not that an exterior vacancy cascade
    # is cheaper.  Never gate this proof on exterior shuttle cardinality.
    # Either move the goal-slot occupant directly into a separated rear pose
    # or advance the selected shuttle by the minimum safe distance that creates
    # such a pose. Execute only that first proved move, then reobserve. This is
    # topology/geometry based and independent of identity, colour, or payload.
    primary_blocker = blocker_ids[0] if blocker_ids else ''
    selected_position = positions.get(selected) or {}
    selected_public_segment = _public_segment_for_side(
        side,
        selected_position.get('segment'),
    )
    selected_gate = INTERIOR_LOOP_GATE_BY_PUBLIC_SEGMENT.get(
        selected_public_segment,
        '',
    )
    direct_exterior_release_by_slot: dict[str, dict[str, Any]] = {}
    primary_source_slot = fleet['location_slot_by_shuttle'].get(
        primary_blocker,
        '',
    )
    normalization = dict(
        route_clearance.get('normalization', {})
        .get('by_side', {})
        .get(side, {})
    )
    exterior_route_setup_safe = bool(
        normal_device_route or normalization.get('reconfiguration_safe')
    )
    conditional_exterior_release: dict[str, Any] | None = None
    if (
        staged_interior_present
        and (
            normal_device_route
            or normalization.get('clearance_pause_safe')
        )
    ):
        conditional_exterior_release = (
            _conditional_exterior_vacancy_release(
                fleet=fleet,
                devices=devices,
                obstacles=obstacles,
                route_clearance=route_clearance,
                side=side,
                selected_shuttle=selected,
                selected_target_slot=target,
                primary_blocker=primary_blocker,
                target_occupant=target_occupant,
                observed_blockers=observed_blocker_ids,
            )
        )
    if conditional_exterior_release is not None:
        mover = str(conditional_exterior_release['shuttle'])
        mover_position = positions[mover]
        relocation = {
            'order': 1,
            'shuttle': mover,
            'reason': 'opens_exterior_vacancy_chain_for_route_blocker',
            'current_segment': mover_position['segment'],
            'current_s_ratio': round(
                float(mover_position['s_ratio']),
                6,
            ),
            'destination': {
                'kind': 'slot',
                'source_slot': conditional_exterior_release[
                    'source_slot'
                ],
                'target_slot': conditional_exterior_release[
                    'target_slot'
                ],
                'target_sensor': SLOT_SENSOR_BY_SIDE_AND_SLOT[
                    (
                        side,
                        _slot_number_from_object(
                            conditional_exterior_release['target_slot']
                        ),
                    )
                ],
                'selection_policy': (
                    'conditional_exterior_vacancy_release_before_'
                    'selected_interior_detour'
                ),
            },
            'dependency_for_shuttle': primary_blocker,
            'effect_verification': 'identity_bearing_slot_sensor',
        }
        base.update({
            'required': True,
            'observed_blockers': observed_blocker_ids,
            'observed_blocker_count': len(observed_blocker_ids),
            'planned_relocations_this_observation': 1,
            'deferred_blockers_require_fresh_reobservation': (
                observed_blocker_ids
            ),
            'receding_horizon_clearance': True,
            'ordered_relocations': [relocation],
            'route_must_be_reobserved_after_each_relocation': True,
            'exterior_slot_relocations_precede_interior_clearance': True,
            'switch_restore_policy': 'once_after_all_relocations',
            'clearance_mode_active': clearance_mode_active,
            'conditional_exterior_vacancy_resolution': {
                'required': True,
                'first_safe_move': {
                    'shuttle': mover,
                    'source_slot': conditional_exterior_release[
                        'source_slot'
                    ],
                    'target_slot': conditional_exterior_release[
                        'target_slot'
                    ],
                },
                'conditionally_unlocked_primary_move': {
                    'shuttle': primary_blocker,
                    'source_slot': conditional_exterior_release[
                        'future_primary_source_slot'
                    ],
                    'target_slot': conditional_exterior_release[
                        'future_primary_target_slot'
                    ],
                },
                'selected_shuttle_kept_on_current_route': True,
                'selected_interior_detour_prevented': True,
                'conditional_release_length_m': round(
                    float(
                        conditional_exterior_release[
                            'total_conditional_release_length_m'
                        ]
                    ),
                    6,
                ),
                'future_actions_are_not_preallocated': True,
                'fresh_reobservation_required_after_first_move': True,
                'policy': (
                    'prove_non_target_vacancy_rotation_then_execute_only_'
                    'the_first_sensor_verified_move'
                ),
                'identity_or_colour_used': False,
                'controller_position_fields_used_for_localization': False,
            },
        })
        if clearance_mode_active:
            base['clearance_pause_for_exterior_progress'] = {
                'required': True,
                'active_gate': active_clearance_gate,
                'shuttle': selected,
                'source_slot': source,
                'final_target_slot': target,
                'reason': (
                    'proved_exterior_vacancy_rotation_requires_normal_route'
                ),
                'policy': (
                    'pause_clearance_then_execute_proved_exterior_move_and_'
                    'reobserve'
                ),
            }
        return base
    if (
        selected_gate
        and primary_blocker
        and primary_source_slot
        and exterior_route_setup_safe
        and not active_clearance_gate
    ):
        for slot_object, occupant in fleet['slot_occupancy'].items():
            if (
                occupant
                or not slot_object.startswith(f'{side}_slot_')
                or slot_object == target
                or (
                    primary_source_slot,
                    slot_object,
                ) not in route_clearance['clear_pairs']
            ):
                continue
            pair = next(
                (
                    candidate
                    for candidate in route_clearance['provenance']['pairs']
                    if candidate['from_slot'] == primary_source_slot
                    and candidate['to_slot'] == slot_object
                ),
                {},
            )
            if not pair:
                continue
            direct_exterior_release_by_slot[slot_object] = {
                'target_slot': slot_object,
                'route_length_m': _topology_route_length_m(
                    side=side,
                    route_entry=pair,
                ),
                'route_blocks': copy.deepcopy(pair['route_blocks']),
            }
    active_gate_direct_release: dict[str, Any] | None = None
    active_gate_is_best_direct_release = not bool(active_clearance_gate)
    active_gate_can_preserve_buffer_choreography = False
    if primary_blocker and selected_gate and active_clearance_gate:
        active_gate_direct_release = (
            _resolve_shortest_direct_interior_blocker_release(
                fleet=fleet,
                devices=devices,
                obstacles=obstacles,
                primary_blocker=primary_blocker,
                selected_shuttle=selected,
                gates=tuple(INTERIOR_HOLDING_BRANCH_BY_GATE),
                required_center_spacing_m=clearance_spacing_m,
            )
        )
        active_gate_is_best_direct_release = bool(
            active_gate_direct_release.get('resolved')
            and str(active_gate_direct_release.get('gate_switch') or '')
            == active_clearance_gate
        )
        # BEGIN is an atomic receding-horizon step, so the next accepted
        # observation still precedes the relocation it configured.  A direct
        # blocker release can remain globally unresolved precisely because the
        # selected interior shuttle must advance first to open a separated
        # holding pose.  Preserve that dense-buffer choreography when its
        # selected branch is the already-held clearance branch; otherwise the
        # rebuild replaces the pending selected advance with ``unavailable``,
        # pauses the unchanged route, and repeats BEGIN forever.
        active_gate_can_preserve_buffer_choreography = bool(
            not active_gate_direct_release.get('resolved')
            and active_clearance_gate == selected_gate
        )
        base['active_clearance_gate_continuation_audit'] = {
            'active_gate': active_clearance_gate,
            'best_direct_release_gate': str(
                active_gate_direct_release.get('gate_switch') or ''
            ),
            'best_direct_release_resolved': bool(
                active_gate_direct_release.get('resolved')
            ),
            'active_gate_retained': bool(
                active_gate_is_best_direct_release
                or active_gate_can_preserve_buffer_choreography
            ),
            'selected_buffer_choreography_retained': (
                active_gate_can_preserve_buffer_choreography
            ),
            'policy': (
                'retain_active_gate_for_the_fresh_global_direct_release_'
                'optimum_or_its_same_branch_selected_buffer_dependency'
            ),
            'controller_position_fields_used_for_localization': False,
        }
    if (
        primary_blocker
        and selected_gate
        and (
            active_gate_is_best_direct_release
            or active_gate_can_preserve_buffer_choreography
        )
    ):
        # Once a clearance gate is active, continue it when a fresh direct
        # comparison selects that gate or when the unresolved direct move is
        # still waiting on the selected shuttle's same-branch buffer advance.
        # This keeps a decision stable across ``begin clearance`` without
        # hijacking cases where another branch has become the proved direct
        # optimum and the active route must pause before reconfiguration.
        direct_release = (
            active_gate_direct_release
            if active_gate_direct_release is not None
            else _resolve_shortest_direct_interior_blocker_release(
            fleet=fleet,
            devices=devices,
            obstacles=obstacles,
            primary_blocker=primary_blocker,
            selected_shuttle=selected,
            gates=tuple(INTERIOR_HOLDING_BRANCH_BY_GATE),
            required_center_spacing_m=clearance_spacing_m,
            )
        )
        best_exterior_release = min(
            direct_exterior_release_by_slot.values(),
            key=lambda candidate: (
                float(candidate['route_length_m']),
                str(candidate['target_slot']),
            ),
            default=None,
        )
        direct_interior_length_m = float(
            direct_release.get('first_mover_route_length_m') or math.inf
        )
        direct_interior_preferred = bool(
            direct_release.get('resolved')
            and (
                best_exterior_release is None
                or direct_interior_length_m
                < float(best_exterior_release['route_length_m'])
            )
        )
        release_kind = (
            'goal_slot' if primary_blocker == target_occupant else 'route'
        )
        cost_comparison = {
            'objective': (
                'fewest_blocker_release_actions_then_shortest_'
                'authoritative_route'
            ),
            'blocker': primary_blocker,
            'blocker_kind': release_kind,
            'direct_interior_available': bool(
                direct_release.get('resolved')
            ),
            'direct_interior_route_length_m': (
                direct_interior_length_m
                if math.isfinite(direct_interior_length_m)
                else None
            ),
            'direct_interior_target_segment': (
                direct_release.get('target_segment') or ''
            ),
            'direct_exterior_candidates': sorted(
                direct_exterior_release_by_slot.values(),
                key=lambda candidate: str(candidate['target_slot']),
            ),
            'selected_strategy': (
                f'direct_interior_{release_kind}_blocker_release'
                if direct_interior_preferred
                else (
                    f'direct_exterior_{release_kind}_blocker_release'
                    if best_exterior_release is not None
                    else 'interior_capacity_choreography'
                )
            ),
            'identity_or_colour_used': False,
            'exterior_slot_cardinality_used': False,
        }
        base['blocker_release_cost_comparison'] = cost_comparison
        if primary_blocker == target_occupant:
            # Backward-compatible audit key retained for target-slot cases.
            base['goal_slot_release_cost_comparison'] = {
                **copy.deepcopy(cost_comparison),
                'objective': (
                    'fewest_goal_slot_release_actions_then_shortest_'
                    'authoritative_route'
                ),
                'selected_strategy': (
                    'direct_interior_goal_slot_release'
                    if direct_interior_preferred
                    else (
                        'direct_exterior_goal_slot_release'
                        if best_exterior_release is not None
                        else 'interior_capacity_choreography'
                    )
                ),
            }
        if direct_interior_preferred:
            buffer_resolution = direct_release
            selected_branch = dict(direct_release['branch'])
            selected_public_segment = str(
                selected_branch['target_segment']
            )
        elif best_exterior_release is not None:
            # The cost audit above selected a proved one-move exterior release.
            # Honour that result even when no *direct interior* route exists.
            # The former condition tested direct_release.resolved here, fell
            # into selected-shuttle capacity choreography when it was false,
            # and therefore moved the user's target despite having already
            # recorded ``direct_exterior_*_blocker_release`` as the cheaper
            # strategy.  Leave buffer resolution false so the ordinary slot
            # relocation block below emits the audited exterior move.
            buffer_resolution = {
                'resolved': False,
                'reason': 'shorter_direct_exterior_blocker_release_exists',
                'direct_release_branch_search': direct_release[
                    'branch_search'
                ],
            }
        else:
            # The same capacity choreography is required for a route blocker,
            # not only for the shuttle occupying the final goal slot.  A
            # segment-origin goal can sit at the rear holding pose while a
            # blocker is stopped on the other interior branch.  Advancing the
            # selected shuttle first opens a separated rear pose; the blocker
            # can then transfer across branches and release the selected goal
            # route after a fresh observation.  Restricting this proof to a
            # goal-slot occupant caused the live R1(A34I) -> slot 4 task to
            # reject even though this two-move topology solution existed.
            selected_branch = dict(
                INTERIOR_HOLDING_BRANCH_BY_GATE[selected_gate]
            )
            buffer_resolution = _resolve_selected_interior_buffer_choreography(
                fleet=fleet,
                devices=devices,
                obstacles=obstacles,
                primary_blocker=primary_blocker,
                selected_shuttle=selected,
                branch=selected_branch,
                required_center_spacing_m=clearance_spacing_m,
            )
            buffer_resolution['direct_release_branch_search'] = (
                direct_release['branch_search']
            )
        if buffer_resolution.get('resolved'):
            mover = str(buffer_resolution['first_movable_shuttle'])
            route_entries = list(
                buffer_resolution.get('route_entries') or []
            )
            first_route = route_entries[-1] if route_entries else {}
            first_route_length_m = _topology_route_length_m(
                side=side,
                route_entry=first_route,
            )
            target_number = int(_slot_number_from_object(target))
            exterior_vacancy_release_actions: int | None = None
            for distance in range(1, 5):
                slot_number = ((target_number - 1 + distance) % 4) + 1
                slot_object = _slot_object(side, str(slot_number))
                if not fleet['slot_occupancy'].get(slot_object):
                    exterior_vacancy_release_actions = distance
                    break
            motion_mode = str(buffer_resolution['motion_mode'])
            relocation_required_switches = dict(
                buffer_resolution.get('required_switches')
                or selected_branch['switches']
            )
            destination = {
                'kind': 'interior_loop',
                'gate_switch': str(selected_branch['gate_switch']),
                'exit_switch': str(selected_branch['exit_switch']),
                'target_segment': selected_public_segment,
                'target_s_m': float(buffer_resolution['target_s_m']),
                'motion_mode': motion_mode,
                'required_center_spacing_m': round(
                    clearance_spacing_m,
                    6,
                ),
                'interior_entry_route_proof': {
                    'status': 'clear',
                    'method': (
                        'authoritative_costed_interior_blocker_release_v3'
                    ),
                    'checked_route_blocks': list(
                        (route_entries[-1] if route_entries else {}).get(
                            'route_blocks'
                        )
                        or []
                    ),
                    'required_switches': relocation_required_switches,
                    'gate_switch': str(selected_branch['gate_switch']),
                    'target_segment': selected_public_segment,
                    'blocking_shuttles': [],
                    'dependency_chain': list(
                        buffer_resolution['dependency_chain']
                    ),
                    'primary_blocker': primary_blocker,
                    'first_movable_shuttle': mover,
                },
            }
            if motion_mode == 'advance_within_interior_branch':
                destination.update({
                    'motion_origin_s_m': float(
                        buffer_resolution['motion_origin_s_m']
                    ),
                    'bounded_motion_distance_m': float(
                        buffer_resolution['bounded_motion_distance_m']
                    ),
                    'origin_clearance_proof': copy.deepcopy(
                        buffer_resolution['origin_clearance_proof']
                    ),
                })
                # When the selected shuttle advances to open a rear pose,
                # retain that conditional future pose for the primary
                # blocker.  If the primary blocker itself is already on the
                # branch and advances now, no second hypothetical move exists
                # and inventing this field would misstate the choreography.
                if 'future_primary_target_s_m' in buffer_resolution:
                    destination['future_primary_target_s_m'] = float(
                        buffer_resolution['future_primary_target_s_m']
                    )
            relocation = {
                'order': 1,
                'shuttle': mover,
                'reason': (
                    'advance_selected_to_open_interior_entry_for_goal_occupant'
                    if mover == selected and primary_blocker == target_occupant
                    else (
                        'advance_selected_to_open_cross_branch_route_capacity'
                        if mover == selected
                        else 'move_goal_occupant_to_open_selected_goal_slot'
                        if primary_blocker == target_occupant
                        else 'move_route_blocker_to_shortest_safe_interior_branch'
                    )
                ),
                'current_segment': positions[mover]['segment'],
                'current_s_ratio': round(
                    float(positions[mover]['s_ratio']),
                    6,
                ),
                'destination': destination,
            }
            required_switch_states = _required_switch_words(
                relocation_required_switches
            )
            active_assignment_mismatch = bool(
                clearance_mode_active
                and _active_clearance_assignment_differs(
                    side=side,
                    devices=devices,
                    required_switches=relocation_required_switches,
                )
            )
            base.update({
                'required': True,
                'observed_blockers': observed_blocker_ids,
                'observed_blocker_count': len(observed_blocker_ids),
                'planned_relocations_this_observation': 1,
                'deferred_blockers_require_fresh_reobservation': (
                    observed_blocker_ids
                ),
                'receding_horizon_clearance': True,
                'ordered_relocations': [relocation],
                'route_must_be_reobserved_after_each_relocation': True,
                'exterior_slot_relocations_precede_interior_clearance': False,
                'switch_restore_policy': 'once_after_all_relocations',
                'clearance_mode_active': clearance_mode_active,
                'unsupported_if_more_than_two_blockers': False,
                'stale_multi_destination_preallocation_used': False,
                'dense_interior_buffer_choreography': {
                    **copy.deepcopy(buffer_resolution),
                    'first_action_only': True,
                    'fresh_reobservation_required_after_action': True,
                    'exterior_slot_rotation_avoided': True,
                    'exterior_slot_cardinality_gate_used': False,
                    'clearance_cost': {
                        'first_mover_route_length_m': first_route_length_m,
                        'route_blocker_release_actions': 1,
                        'goal_slot_release_actions': (
                            (1 if mover == primary_blocker else 2)
                            if primary_blocker == target_occupant else None
                        ),
                        'exterior_vacancy_release_actions': (
                            exterior_vacancy_release_actions
                        ),
                    },
                    'selection_policy': buffer_resolution.get(
                        'selection_policy',
                        'cardinality_independent_shared_branch_release_then_'
                        'fresh_reobservation',
                    ),
                },
            })
            if active_assignment_mismatch:
                # Reconfigure only through the existing certified pause.  A
                # direct switch change while an interior shuttle is stopped
                # under a different held route can connect it to FALLING.
                base['clearance_pause_for_exterior_progress'] = {
                    'required': True,
                    'active_gate': active_clearance_gate,
                    'shuttle': selected,
                    'source_slot': source,
                    'final_target_slot': target,
                    'reason': (
                        'next_cross_branch_relocation_requires_a_different_'
                        'certified_switch_assignment'
                    ),
                    'current_switches': {
                        device: devices['switches'].get(
                            _switch_object(side, device)
                        )
                        for device in DEVICE_NAMES
                    },
                    'next_required_switches': required_switch_states,
                    'policy': (
                        'pause_clearance_then_reconfigure_once_and_reobserve'
                    ),
                }
            return base
    has_staged_interior_shuttles = staged_interior_present
    topology_recovery = not bool(source)
    free_parking_slots = {
        slot
        for slot, occupant in fleet['slot_occupancy'].items()
        if (
            topology_recovery
            or (has_staged_interior_shuttles and not clearance_mode_active)
        )
        and not occupant
        and slot.startswith(f'{side}_slot_')
        and slot != target
    }
    parking_destination_by_blocker: dict[str, dict[str, Any]] = {}
    parking_order = sorted(
        blocker_ids,
        key=lambda blocker: (
            0 if blocker == target_occupant else 1,
            blocker_ids.index(blocker),
        ),
    )
    for blocker in parking_order:
        blocker_slot = fleet['location_slot_by_shuttle'].get(blocker, '')
        topology_candidates: dict[str, dict[str, Any]] = {}
        if blocker_slot:
            candidates = sorted(
                (
                    slot
                    for slot in free_parking_slots
                    if (blocker_slot, slot) in route_clearance['clear_pairs']
                ),
                key=lambda slot: (
                    float(
                        direct_exterior_release_by_slot.get(
                            slot,
                            {},
                        ).get('route_length_m', math.inf)
                    )
                    if blocker == primary_blocker
                    and direct_exterior_release_by_slot
                    else (
                        -int(_slot_number_from_object(slot))
                        if has_staged_interior_shuttles
                        else 0
                    ),
                    abs(
                        int(_slot_number_from_object(blocker_slot))
                        - int(_slot_number_from_object(slot))
                    ),
                    int(_slot_number_from_object(slot)),
                ),
            )
        else:
            for parking_slot in sorted(free_parking_slots):
                entry = _topology_route_entry(
                    fleet=fleet,
                    devices=devices,
                    obstacles=obstacles,
                    shuttle=blocker,
                    target_slot=_slot_number_from_object(parking_slot),
                )
                # The selected shuttle may sit safely behind the blocker on
                # the same segment. It is not a forward obstruction unless
                # its physical interval actually overlaps the blocker route.
                if entry['route_clear']:
                    topology_candidates[parking_slot] = entry
            candidates = sorted(
                topology_candidates,
                key=lambda slot: (
                    len(topology_candidates[slot]['route_blocks']),
                    int(_slot_number_from_object(slot)),
                ),
            )
        if not candidates:
            continue
        parking_slot = candidates[0]
        free_parking_slots.remove(parking_slot)
        # The blocker source becomes free after this supervised move. It is
        # intentionally not reused in this frozen planning pass because route
        # clearance is recomputed from a fresh observation after every move.
        parking_destination_by_blocker[blocker] = {
            'kind': 'slot',
            'source_slot': blocker_slot,
            'target_slot': parking_slot,
            'target_sensor': SLOT_SENSOR_BY_SIDE_AND_SLOT[
                (side, _slot_number_from_object(parking_slot))
            ],
            'selection_policy': (
                'shortest_authoritative_one_move_exterior_goal_release'
                if blocker == target_occupant
                and direct_exterior_release_by_slot
                else 'shortest_authoritative_one_move_exterior_blocker_release'
                if blocker == primary_blocker
                and direct_exterior_release_by_slot
                else 'recovery_aware_highest_reachable_free_exterior_slot'
                if has_staged_interior_shuttles
                else 'nearest_reachable_known_free_exterior_slot'
            ),
        }
        if parking_slot in topology_candidates:
            parking_destination_by_blocker[blocker].update({
                'source_kind': 'accepted_visual_continuous_position',
                'topology_route': topology_candidates[parking_slot],
            })

    # If the goal-slot occupant cannot yet reach a free parking slot, do not
    # send it toward an interior gate through another occupied slot. Propagate
    # the nearest exterior vacancy backward by one sensor-verifiable slot move,
    # then rebuild from a fresh observation. This applies equally when the
    # user's shuttle starts at an exact slot and when it starts on an accepted
    # visual topology segment such as A12I/A34I. In the latter case there is no
    # synthetic selected-source slot to protect; the continuous topology
    # occupancy proof still prevents a dependency move through the selected
    # shuttle. Example: target slot 2 is occupied, slot 3 is occupied, and slot
    # 4 is free. Move slot 3 -> 4, reobserve, then move the target occupant
    # slot 2 -> 3. This is a causal vacancy dependency, not an additional
    # blocker guessed from identity, colour, or the user's wording.
    vacancy_dependency_relocation: dict[str, Any] | None = None
    if (
        normal_device_route
        and not obstacles.get(side)
        and target_occupant
        and target_occupant in blocker_ids
        and target_occupant not in parking_destination_by_blocker
    ):
        target_number = int(_slot_number_from_object(target))
        dependency_chain = [{
            'slot': target,
            'shuttle': target_occupant,
        }]
        for forward_distance in range(1, 4):
            slot_number = ((target_number - 1 + forward_distance) % 4) + 1
            slot_object = _slot_object(side, str(slot_number))
            if source and slot_object == source:
                # Never move a dependency through the selected shuttle that
                # is waiting to enter the goal slot.
                break
            occupant = fleet['slot_occupancy'].get(slot_object, '')
            if occupant:
                dependency_chain.append({
                    'slot': slot_object,
                    'shuttle': occupant,
                })
                continue
            if slot_object not in free_parking_slots:
                break
            mover = dependency_chain[-1]
            mover_shuttle = str(mover['shuttle'])
            mover_source = str(mover['slot'])
            # A one-entry chain means the goal occupant has a direct parking
            # move; the ordinary candidate selection above owns that case.
            if mover_shuttle == target_occupant:
                break
            if (mover_source, slot_object) not in route_clearance['clear_pairs']:
                break
            mover_position = positions[mover_shuttle]
            vacancy_dependency_relocation = {
                'order': 1,
                'shuttle': mover_shuttle,
                'reason': 'blocks_target_occupant_relocation',
                'current_segment': mover_position['segment'],
                'current_s_ratio': round(
                    float(mover_position['s_ratio']),
                    6,
                ),
                'destination': {
                    'kind': 'slot',
                    'source_slot': mover_source,
                    'target_slot': slot_object,
                    'target_sensor': SLOT_SENSOR_BY_SIDE_AND_SLOT[
                        (side, str(slot_number))
                    ],
                    'selection_policy': (
                        'nearest_forward_vacancy_dependency_step'
                    ),
                },
                'dependency_for_shuttle': target_occupant,
                'dependency_chain': copy.deepcopy(dependency_chain),
                'free_slot': slot_object,
                'forward_slot_steps': 1,
                'effect_verification': 'identity_bearing_slot_sensor',
            }
            break
    if vacancy_dependency_relocation is not None:
        base.update({
            'required': True,
            'observed_blockers': observed_blocker_ids,
            'observed_blocker_count': len(observed_blocker_ids),
            'planned_relocations_this_observation': 1,
            'deferred_blockers_require_fresh_reobservation': (
                observed_blocker_ids
            ),
            'receding_horizon_clearance': True,
            'ordered_relocations': [vacancy_dependency_relocation],
            'route_must_be_reobserved_after_each_relocation': True,
            'exterior_slot_relocations_precede_interior_clearance': True,
            'switch_restore_policy': 'once_after_all_relocations',
            'clearance_mode_active': clearance_mode_active,
            'unsupported_if_more_than_two_blockers': False,
            'stale_multi_destination_preallocation_used': False,
            'vacancy_dependency_resolution': {
                'required': True,
                'goal_target_occupant': target_occupant,
                'selected_source_slot': source,
                'selected_source_kind': (
                    'exact_slot'
                    if source
                    else 'accepted_visual_continuous_position'
                ),
                'selected_target_slot': target,
                'dependency_chain': copy.deepcopy(
                    vacancy_dependency_relocation['dependency_chain']
                ),
                'first_safe_move': {
                    'shuttle': vacancy_dependency_relocation['shuttle'],
                    'source_slot': vacancy_dependency_relocation[
                        'destination'
                    ]['source_slot'],
                    'target_slot': vacancy_dependency_relocation[
                        'destination'
                    ]['target_slot'],
                },
                'interior_relocation_of_goal_occupant_prevented': True,
                'policy': (
                    'propagate_nearest_forward_vacancy_then_reobserve'
                ),
            },
        })
        return base

    interior_target_by_blocker: dict[str, float] = {}
    interior_route_proof_by_blocker: dict[str, dict[str, Any]] = {}
    unresolved_interior_search: dict[str, Any] = {}
    selected_branch: dict[str, Any] = dict(
        INTERIOR_HOLDING_BRANCH_BY_GATE['A3']
    )
    reserved_interior_s_m: list[float] = []

    # Search both complete interior branches. The target's first physical
    # blocker is preferred over moving an unrelated or selected shuttle; a
    # deeper dependency is used only when that blocker cannot enter either
    # branch directly. Once a branch clearance mode is active, retain it until
    # the executive explicitly finishes or pauses that mode.
    primary_blocker = blocker_ids[0] if blocker_ids else ''
    if (
        primary_blocker
        and primary_blocker not in parking_destination_by_blocker
    ):
        branch_search: list[dict[str, Any]] = []
        gates = (
            (active_clearance_gate,)
            if active_clearance_gate
            else tuple(INTERIOR_HOLDING_BRANCH_BY_GATE)
        )
        for gate in gates:
            branch = dict(INTERIOR_HOLDING_BRANCH_BY_GATE[gate])
            public_segment = str(branch['target_segment'])
            occupied_s_m = [
                float(certificate['target_s_m'])
                for certificate in runtime_clearances.values()
                if certificate.get('side') == side
                and str(certificate.get('target_segment') or '').upper()
                == public_segment
            ] + [
                float(position['s_ratio'])
                * float(position['segment_length_m'])
                for shuttle, position in positions.items()
                if shuttle not in runtime_clearances
                and shuttle != primary_blocker
                and position.get('side') == side
                and _is_interior_holding_position(
                    side=side,
                    position=position,
                    public_segment=public_segment,
                )
            ]
            pose_candidates = _interior_holding_pose_candidates(
                side=side,
                public_segment=public_segment,
                occupied_s_m=occupied_s_m,
                required_center_spacing_m=clearance_spacing_m,
            )
            resolution = _resolve_interior_clearance_dependency(
                fleet=fleet,
                devices=devices,
                obstacles=obstacles,
                primary_blocker=primary_blocker,
                selected_shuttle=selected,
                branch=branch,
                target_s_m_candidates=pose_candidates,
                preferred_dependency_shuttles=observed_blocker_ids[1:],
            )
            route_entries = list(resolution.get('route_entries') or [])
            route_length_m = _topology_route_length_m(
                side=side,
                route_entry=(route_entries[-1] if route_entries else {}),
            )
            resolution.update({
                'branch': branch,
                'occupied_target_s_m': sorted(occupied_s_m),
                'candidate_pose_count': len(pose_candidates),
                'first_mover_route_length_m': route_length_m,
            })
            first_mover = str(
                resolution.get('first_movable_shuttle') or ''
            )
            nearest_forward_blocker = (
                observed_blocker_ids[-1]
                if source and observed_blocker_ids
                else ''
            )
            unlocks_selected_progress = bool(
                nearest_forward_blocker
                and first_mover == nearest_forward_blocker
            )
            resolution.update({
                'nearest_forward_blocker': nearest_forward_blocker,
                'immediate_selected_forward_progress_unlocked': (
                    unlocks_selected_progress
                ),
                'clearance_cost': {
                    'first_mover_route_length_m': route_length_m,
                    'dependency_depth': len(
                        resolution.get('dependency_chain') or []
                    ),
                    'moves_primary_blocker': (
                        first_mover == primary_blocker
                    ),
                    'moves_selected_shuttle': first_mover == selected,
                    'opens_immediate_selected_progress': (
                        unlocks_selected_progress
                    ),
                },
            })
            branch_search.append(resolution)

        resolved_branches = [
            result for result in branch_search if result.get('resolved')
        ]
        dependency_resolution = (
            min(
                resolved_branches,
                key=lambda result: (
                    0
                    if result.get(
                        'immediate_selected_forward_progress_unlocked'
                    )
                    else 1,
                    0
                    if result['first_movable_shuttle'] == primary_blocker
                    else 1,
                    0
                    if result['first_movable_shuttle'] != selected
                    else 1,
                    len(result.get('dependency_chain') or []),
                    float(result.get('first_mover_route_length_m') or 0.0),
                    str(result.get('gate_switch') or ''),
                ),
            )
            if resolved_branches
            else {
                'resolved': False,
                'primary_blocker': primary_blocker,
                'branch_search': branch_search,
                'reason': 'all_authoritative_interior_branches_unavailable',
                'controller_position_fields_used_for_localization': False,
            }
        )
        base['clearance_branch_search'] = {
            'available_gates': list(gates),
            'selected_gate': (
                dependency_resolution.get('gate_switch') or ''
            ),
            'selected_target_segment': (
                dependency_resolution.get('target_segment') or ''
            ),
            'active_gate_retained': bool(active_clearance_gate),
            'results': branch_search,
            'policy': (
                'retain_active_branch_else_unlock_selected_forward_'
                'progress_then_direct_primary_blocker_then_nonselected_'
                'dependency_then_shortest_route'
            ),
        }
        if dependency_resolution['resolved']:
            selected_branch = dict(dependency_resolution['branch'])
            selected_public_segment = str(
                selected_branch['target_segment']
            )
            reserved_interior_s_m = [
                float(value)
                for value in dependency_resolution.get(
                    'occupied_target_s_m'
                )
                or []
            ]
            first_movable = str(
                dependency_resolution['first_movable_shuttle']
            )
            blocker_ids = [first_movable]
            interior_target_by_blocker[first_movable] = float(
                dependency_resolution['target_s_m']
            )
            route_entries = list(
                dependency_resolution.get('route_entries') or []
            )
            interior_route_proof_by_blocker[first_movable] = {
                'status': 'clear',
                'method': 'authoritative_topology_dependency_search_v2',
                'checked_route_blocks': list(
                    (route_entries[-1] if route_entries else {}).get(
                        'route_blocks'
                    ) or []
                ),
                'required_switches': dict(
                    dependency_resolution['required_switches']
                ),
                'gate_switch': dependency_resolution['gate_switch'],
                'target_segment': selected_public_segment,
                'blocking_shuttles': [],
                'dependency_chain': list(
                    dependency_resolution['dependency_chain']
                ),
                'primary_blocker': primary_blocker,
                'first_movable_shuttle': first_movable,
            }
            if first_movable != primary_blocker:
                deferred = list(
                    base.get(
                        'deferred_blockers_require_fresh_reobservation'
                    ) or []
                )
                if primary_blocker not in deferred:
                    deferred.insert(0, primary_blocker)
                base[
                    'deferred_blockers_require_fresh_reobservation'
                ] = deferred
                base['clearance_dependency_resolution'] = {
                    **dependency_resolution,
                    'first_action_only': True,
                    'fresh_reobservation_required_after_action': True,
                }
        else:
            unresolved_interior_search = dependency_resolution

    relocations = []
    interior_blocker_ids = [
        blocker
        for blocker in blocker_ids
        if blocker not in parking_destination_by_blocker
    ]
    ordered_blockers = [
        blocker
        for blocker in blocker_ids
        if blocker in parking_destination_by_blocker
    ] + interior_blocker_ids
    for index, blocker in enumerate(ordered_blockers):
        position = positions[blocker]
        parking_destination = parking_destination_by_blocker.get(blocker)
        if parking_destination is not None:
            relocations.append({
                'order': index + 1,
                'shuttle': blocker,
                'reason': (
                    'occupies_goal_target_slot'
                    if blocker == target_occupant
                    else 'blocks_selected_shuttle_route'
                ),
                'current_segment': position['segment'],
                'current_s_ratio': round(float(position['s_ratio']), 6),
                'destination': parking_destination,
            })
            continue
        # ROS visual labels and runtime certificates retain the public branch
        # name on both rails. The mirrored left topology is converted only at
        # the topology-search boundary.
        public_segment = str(selected_branch['target_segment'])
        gate_switch = str(selected_branch['gate_switch'])
        target_s_m = interior_target_by_blocker.get(blocker)
        if target_s_m is None and not unresolved_interior_search:
            target_s_m = next(
                iter(_interior_holding_pose_candidates(
                    side=side,
                    public_segment=public_segment,
                    occupied_s_m=reserved_interior_s_m,
                    required_center_spacing_m=clearance_spacing_m,
                )),
                None,
            )
        if target_s_m is None:
            destination = {
                'kind': 'unavailable',
                'reason': (
                    'no_reachable_physically_separated_interior_holding_pose'
                ),
                'gate_switch': gate_switch,
                'target_segment': public_segment,
                'required_center_spacing_m': round(clearance_spacing_m, 6),
                'topology_dependency_search': copy.deepcopy(
                    unresolved_interior_search
                ),
            }
        else:
            reserved_interior_s_m.append(float(target_s_m))
            route_proof = interior_route_proof_by_blocker.get(blocker)
            if route_proof is None:
                fallback_resolution = _resolve_interior_clearance_dependency(
                    fleet=fleet,
                    devices=devices,
                    obstacles=obstacles,
                    primary_blocker=blocker,
                    selected_shuttle=selected,
                    branch=selected_branch,
                    target_s_m_candidates=[float(target_s_m)],
                )
                if not fallback_resolution['resolved'] or (
                    fallback_resolution['first_movable_shuttle'] != blocker
                ):
                    destination = {
                        'kind': 'unavailable',
                        'reason': (
                            'no_reachable_physically_separated_interior_'
                            'holding_pose'
                        ),
                        'gate_switch': gate_switch,
                        'target_segment': public_segment,
                        'required_center_spacing_m': round(
                            clearance_spacing_m,
                            6,
                        ),
                        'topology_dependency_search': fallback_resolution,
                    }
                    relocations.append({
                        'order': index + 1,
                        'shuttle': blocker,
                        'reason': 'blocks_selected_shuttle_route',
                        'current_segment': position['segment'],
                        'current_s_ratio': round(
                            float(position['s_ratio']),
                            6,
                        ),
                        'destination': destination,
                    })
                    continue
                route_entries = list(
                    fallback_resolution.get('route_entries') or []
                )
                route_proof = {
                    'status': 'clear',
                    'method': 'authoritative_topology_dependency_search_v2',
                    'checked_route_blocks': list(
                        (route_entries[-1] if route_entries else {}).get(
                            'route_blocks'
                        ) or []
                    ),
                    'required_switches': dict(
                        fallback_resolution['required_switches']
                    ),
                    'gate_switch': gate_switch,
                    'target_segment': public_segment,
                    'blocking_shuttles': [],
                    'dependency_chain': [blocker],
                    'primary_blocker': blocker,
                    'first_movable_shuttle': blocker,
                }
            destination = {
                'kind': 'interior_loop',
                'gate_switch': gate_switch,
                'exit_switch': selected_branch['exit_switch'],
                'target_segment': public_segment,
                'target_s_m': float(target_s_m),
                'required_center_spacing_m': round(clearance_spacing_m, 6),
                'interior_entry_route_proof': route_proof,
            }
        relocations.append({
            'order': index + 1,
            'shuttle': blocker,
            'reason': (
                'blocks_clearance_dependency'
                if blocker != primary_blocker
                else 'blocks_selected_shuttle_route'
            ),
            'current_segment': position['segment'],
            'current_s_ratio': round(float(position['s_ratio']), 6),
            'destination': destination,
        })
    base.update({
        'required': True,
        'ordered_relocations': relocations,
        'route_must_be_reobserved_after_each_relocation': True,
        'exterior_slot_relocations_precede_interior_clearance': True,
        'switch_restore_policy': 'once_after_all_relocations',
        'clearance_mode_active': clearance_mode_active,
        'unsupported_if_more_than_two_blockers': False,
        'stale_multi_destination_preallocation_used': False,
    })
    first_relocation = relocations[0] if relocations else {}
    first_destination = dict(first_relocation.get('destination') or {})
    first_route_proof = dict(
        first_destination.get('interior_entry_route_proof') or {}
    )
    next_required_switches = dict(
        first_route_proof.get('required_switches') or {}
    )
    if (
        clearance_mode_active
        and first_destination.get('kind') == 'interior_loop'
        and next_required_switches
        and _active_clearance_assignment_differs(
            side=side,
            devices=devices,
            required_switches=next_required_switches,
        )
    ):
        # Every receding-horizon relocation owns a complete route assignment.
        # Never reuse an active branch merely because the target gate matches:
        # a mover starting on the opposite interior branch may additionally
        # require its source exit switch.  The live R4(A34I) -> A12I case
        # required A4=I; reusing A1/A2 clearance with A4=E sent it to FALLING.
        base['clearance_pause_for_exterior_progress'] = {
            'required': True,
            'active_gate': active_clearance_gate,
            'shuttle': str(first_relocation.get('shuttle') or selected),
            'source_slot': source,
            'final_target_slot': target,
            'reason': (
                'next_interior_relocation_requires_a_different_'
                'certified_switch_assignment'
            ),
            'current_switches': {
                device: devices['switches'].get(
                    _switch_object(side, device)
                )
                for device in DEVICE_NAMES
            },
            'next_required_switches': _required_switch_words(
                next_required_switches
            ),
            'policy': (
                'pause_clearance_then_reconfigure_once_and_reobserve'
            ),
        }
    return base


def _append_topology_clearance_routes(route_clearance: dict[str, Any]) -> None:
    """Expose audited segment-origin blocker parking routes to PlanSys2."""

    topology_routes = route_clearance.get('topology_routes') or {}
    routes = topology_routes.get('routes')
    by_key = topology_routes.get('by_shuttle_and_slot')
    provenance = topology_routes.get('provenance') or {}
    provenance_routes = provenance.get('routes')
    if not isinstance(routes, list) or not isinstance(by_key, dict):
        return
    if not isinstance(provenance_routes, list):
        provenance_routes = []
        provenance['routes'] = provenance_routes
    clearance = route_clearance.get('target_clearance_plan') or {}
    for relocation in clearance.get('ordered_relocations') or []:
        destination = relocation.get('destination') or {}
        entry = destination.get('topology_route')
        if not isinstance(entry, dict):
            continue
        key = (entry['shuttle'], entry['target_slot_object'])
        if key in by_key:
            continue
        routes.append(entry)
        by_key[key] = entry
        provenance_routes.append({
            field: value
            for field, value in entry.items()
            if field not in {'configured'}
        })


def _clearance_lifecycle_certificate_is_safe(
    certificate: dict[str, Any],
) -> bool:
    """Recognize an executor-owned stopped interior-motion effect proof."""

    try:
        normalized = normalize_runtime_clearance_certificate(
            certificate.get('identity'),
            certificate,
        )
    except ValueError:
        return False
    side = normalized['side']
    target_segment = str(
        normalized.get('target_segment') or ''
    ).strip().upper()
    gate = INTERIOR_LOOP_GATE_BY_PUBLIC_SEGMENT.get(target_segment, '')
    if side not in SIDES or not gate:
        return False
    expected_sensor = INTERIOR_LOOP_ENTRY_SENSOR_BY_SIDE_AND_GATE.get(
        (side, gate),
        '',
    )
    target_s_m = normalized['target_s_m']
    return bool(
        math.isfinite(target_s_m)
        and target_s_m > 0.0
        and str(normalized.get('entry_sensor') or '').strip().upper()
        == expected_sensor
    )


def _route_normalization_snapshot(
    *,
    fleet: dict[str, Any],
    devices: dict[str, dict[str, str]],
    obstacles: dict[str, list[str]],
) -> dict[str, Any]:
    """Prove when a stopped mixed route may return to exterior/open state.

    A topology-origin move can legitimately leave one or more switches in an
    interior state after the destination sensor stops the commanded shuttle.
    Later slot-to-slot actions require the canonical all-exterior route.  This
    snapshot makes that lifecycle transition explicit and fail-closed: every
    shuttle visually located on an interior segment must also have a validated
    executor certificate proving its controller stop, and an active clearance
    configuration or external obstacle forbids automatic normalization.
    """

    by_side: dict[str, dict[str, Any]] = {}
    certificates = dict(
        fleet.get('runtime_clearance_certificates') or {}
    )
    positions = fleet.get('rail_position_by_shuttle') or {}
    planning_positions = (
        fleet.get('planning_rail_position_by_shuttle') or positions
    )
    for side in SIDES:
        switches = {
            device: devices['switches'].get(_switch_object(side, device), '')
            for device in DEVICE_NAMES
        }
        stoppers = {
            device: devices['stoppers'].get(_stopper_object(side, device), '')
            for device in DEVICE_NAMES
        }
        normal_route = (
            all(state == 'exterior' for state in switches.values())
            and all(state == 'open' for state in stoppers.values())
        )
        all_stoppers_open = all(
            state == 'open' for state in stoppers.values()
        )
        active_clearance_gate = _active_interior_clearance_gate(
            side=side,
            devices=devices,
        )
        clearance_mode = bool(active_clearance_gate)
        visually_interior_shuttles = sorted(
            shuttle
            for shuttle, position in positions.items()
            if position.get('side') == side
            and str(position.get('segment') or '').strip().upper().endswith('I')
        )
        planning_interior_shuttles = sorted(
            shuttle
            for shuttle, position in planning_positions.items()
            if position.get('side') == side
            and str(position.get('segment') or '').strip().upper().endswith('I')
        )
        certified_interior_shuttles = sorted(
            shuttle
            for shuttle, certificate in certificates.items()
            if str(certificate.get('side') or '').strip().casefold() == side
            and str(certificate.get('target_segment') or '')
            .strip().upper().endswith('I')
        )
        # The physical switch-risk set follows canonical planning geometry plus
        # persisted interior-entry effects.  Planning geometry differs from raw
        # vision only under strict execution evidence: either an identity-
        # bearing verified exterior-slot arrival or an interior-entry/stop
        # certificate.  Thus an unanchored raw ``*I`` label still fails closed,
        # while a stale raw A12I prediction cannot overrule a later verified
        # DZI arrival at an exterior slot.  Raw vision remains preserved below
        # for disagreement diagnostics and is never rewritten.
        interior_shuttles = sorted(
            set(planning_interior_shuttles)
            | set(certified_interior_shuttles)
        )
        raw_interior_overridden_by_verified_exterior_slot = sorted(
            set(visually_interior_shuttles) - set(interior_shuttles)
        )
        certificate_segment_consistency = {
            shuttle: _clearance_certificate_visual_segment_consistency(
                side=side,
                certificate=certificates[shuttle],
                position=planning_positions[shuttle],
            )
            for shuttle in interior_shuttles
            if shuttle in certificates and shuttle in planning_positions
        }
        certified_stopped = sorted(
            shuttle
            for shuttle in interior_shuttles
            if shuttle in certificates
            and bool(certificates[shuttle].get('controller_stop_confirmed'))
            and bool(
                certificates[shuttle].get(
                    'bounded_commanded_motion_completed'
                )
            )
            and bool(
                certificate_segment_consistency.get(shuttle, {}).get(
                    'satisfied'
                )
            )
        )
        uncertified_interior = sorted(
            set(interior_shuttles) - set(certified_stopped)
        )
        segment_mismatches = sorted(
            shuttle
            for shuttle, consistency in certificate_segment_consistency.items()
            if not consistency['satisfied']
        )
        clearance_lifecycle_certified = sorted(
            shuttle
            for shuttle in interior_shuttles
            if shuttle in certificates
            and _clearance_lifecycle_certificate_is_safe(
                certificates[shuttle]
            )
        )
        clearance_lifecycle_uncertified = sorted(
            set(interior_shuttles) - set(clearance_lifecycle_certified)
        )
        clearance_lifecycle_visual_disagreements = sorted(
            set(segment_mismatches) & set(clearance_lifecycle_certified)
        )
        lifecycle_disagreements_proved = (
            set(segment_mismatches)
            == set(clearance_lifecycle_visual_disagreements)
        )
        reconfiguration_required = not normal_route and not clearance_mode
        reconfiguration_safe = (
            reconfiguration_required
            and all_stoppers_open
            and not bool(obstacles.get(side))
            and not clearance_lifecycle_uncertified
            and lifecycle_disagreements_proved
        )
        clearance_pause_safe = (
            clearance_mode
            and not bool(obstacles.get(side))
            and bool(interior_shuttles)
            and not clearance_lifecycle_uncertified
        )
        if normal_route:
            reason = 'already_normal'
        elif clearance_mode:
            reason = 'active_clearance_configuration_must_finish_explicitly'
        elif not all_stoppers_open:
            reason = 'mixed_route_has_non_open_stopper_state'
        elif obstacles.get(side):
            reason = 'external_obstacle_present'
        elif clearance_lifecycle_uncertified:
            reason = 'interior_shuttle_has_no_validated_stop_certificate'
        elif not lifecycle_disagreements_proved:
            reason = 'interior_stop_certificate_segment_mismatch'
        elif segment_mismatches:
            reason = (
                'mixed_topology_route_is_safe_with_certified_visual_'
                'disagreement'
            )
        else:
            reason = 'mixed_topology_route_is_safe_to_normalize'
        by_side[side] = {
            'side': side,
            'switches': switches,
            'stoppers': stoppers,
            'normal_route': normal_route,
            'clearance_mode': clearance_mode,
            'active_clearance_gate': active_clearance_gate,
            'all_stoppers_open': all_stoppers_open,
            'reconfiguration_required': reconfiguration_required,
            'reconfiguration_safe': reconfiguration_safe,
            'clearance_pause_safe': clearance_pause_safe,
            'interior_shuttles': interior_shuttles,
            'visually_interior_shuttles': visually_interior_shuttles,
            'planning_interior_shuttles': planning_interior_shuttles,
            'raw_interior_overridden_by_verified_exterior_slot': (
                raw_interior_overridden_by_verified_exterior_slot
            ),
            'raw_visual_position_preserved': True,
            'certified_interior_shuttles': certified_interior_shuttles,
            'certified_stopped_interior_shuttles': certified_stopped,
            'uncertified_interior_shuttles': uncertified_interior,
            'certificate_segment_mismatches': segment_mismatches,
            'clearance_lifecycle_certified_stopped_interior_shuttles': (
                clearance_lifecycle_certified
            ),
            'clearance_lifecycle_uncertified_interior_shuttles': (
                clearance_lifecycle_uncertified
            ),
            'clearance_lifecycle_visual_disagreements': (
                clearance_lifecycle_visual_disagreements
            ),
            'reconfiguration_visual_disagreements_proved': (
                lifecycle_disagreements_proved
            ),
            'clearance_lifecycle_visual_prediction_preserved': True,
            'clearance_lifecycle_certificate_used_as_localization': False,
            'certificate_segment_consistency': (
                certificate_segment_consistency
            ),
            'external_obstacles': list(obstacles.get(side) or []),
            'reason': reason,
            'controller_position_fields_used_for_localization': False,
        }
    return {
        'method': 'validated_mixed_route_normalization_v1',
        'canonical_switch_state': {
            device: 'exterior' for device in DEVICE_NAMES
        },
        'canonical_stopper_state': {
            device: 'open' for device in DEVICE_NAMES
        },
        'by_side': by_side,
        'controller_position_fields_used_for_localization': False,
    }


def _pddl_init_facts(
    *,
    fleet: dict[str, Any],
    devices: dict[str, dict[str, str]],
    obstacles: dict[str, list[str]],
    blocks: dict[str, Any],
    goal_data: dict[str, Any],
    route_clearance: dict[str, Any],
) -> list[str]:
    facts: list[str] = ['(= (total-cost) 0)', '(validated_state)']
    target_clearance = dict(
        route_clearance.get('target_clearance_plan') or {}
    )
    clearance_side = str(goal_data.get('side') or '')
    relocations = list(target_clearance.get('ordered_relocations') or [])
    for side in SIDES:
        pending = len(relocations) if side == clearance_side else 0
        facts.extend([
            f'(= (pending_clearances {side}) {pending})',
            f'(= (clearance_cursor {side}) 1)',
        ])
    for relocation in relocations:
        blocker = str(relocation.get('shuttle') or '')
        destination = dict(relocation.get('destination') or {})
        if destination.get('kind') != 'interior_loop':
            continue
        facts.extend([
            f'(clearance_destination_ready {blocker})',
            f'(= (clearance_order {blocker}) '
            f'{int(relocation.get("order") or 1)})',
        ])
        interior_route_proof = dict(
            destination.get('interior_entry_route_proof') or {}
        )
        if (
            interior_route_proof.get('status') == 'clear'
            and not interior_route_proof.get('blocking_shuttles')
        ):
            facts.append(f'(interior_entry_route_clear {blocker})')
    for side in SIDES:
        facts.extend([
            f'(switch_group_on_side {side}_switch_group {side})',
            f'(stopper_group_on_side {side}_stopper_group {side})',
        ])
        stations = ('yaskawa', 'staubli') if side == 'right' else ('yaskawa', 'kuka')
        for source in stations:
            for target in stations:
                facts.append(f'(connected {side} {_station_object(side, source)} {_station_object(side, target)})')
        for slot in ('1', '2', '3', '4'):
            slot_object = _slot_object(side, slot)
            station = _station_object(side, _station_for_slot(side, slot))
            block = _block_object_for_slot(side, slot)
            facts.extend([
                f'(slot_on_side {slot_object} {side})',
                f'(slot_at_station {slot_object} {station})',
                f'(slot_in_block {slot_object} {block})',
                f'(block_on_side {block} {side})',
            ])
    planning_positions = (
        fleet.get('planning_rail_position_by_shuttle')
        or fleet['rail_position_by_shuttle']
    )
    for shuttle_spec in all_shuttle_specs():
        shuttle = shuttle_spec.shuttle_id
        if not fleet['present_by_shuttle'][shuttle]:
            continue
        facts.append(f'(shuttle_on_side {shuttle} {shuttle_spec.side})')
        if fleet['loaded_by_shuttle'][shuttle]:
            facts.append(f'(loaded {shuttle})')
        else:
            facts.append(f'(empty {shuttle})')
        slot_object = fleet['location_slot_by_shuttle'].get(shuttle)
        if slot_object:
            station = _station_object(
                shuttle_spec.side,
                _station_for_slot(shuttle_spec.side, _slot_number_from_object(slot_object)),
            )
            facts.extend([
                f'(shuttle_at_slot {shuttle} {slot_object})',
                f'(shuttle_at {shuttle} {station})',
                f'(shuttle_stopped_at {shuttle} {station})',
            ])
        block_object = fleet['location_block_by_shuttle'].get(shuttle)
        if block_object:
            facts.append(f'(shuttle_in_block {shuttle} {_pddl_symbol(block_object)})')
        # Topology actions must start from the same authoritative planning
        # position used to construct their route facts.  This differs from
        # the raw learned position only after an identity-bearing slot sensor
        # or a supervised interior-motion certificate has proved an executed
        # categorical effect.  Emitting the raw visual block here while
        # topology_route_available uses that persisted effect creates an
        # internally contradictory PDDL problem (for example A34E versus
        # A34I), so POPF can never apply begin_segment_route_clearance.  The
        # learned block remains untouched in rail_position_by_shuttle and in
        # shuttle_in_block above; this fact is executor-owned planning state,
        # not a replacement visual prediction or controller localization.
        position = planning_positions.get(shuttle)
        if position:
            facts.append(
                f'(shuttle_at_topology_block {shuttle} '
                f'{_topology_block_object(shuttle_spec.side, position["segment"])})'
            )
            # Segment-origin actions intentionally omit source-slot occupancy.
            # Expose them only when perception cannot bind this shuttle to an
            # exact slot.  A slot-bound shuttle must use the slot-topology
            # actions so its source occupancy is removed atomically.
            if not slot_object:
                facts.append(f'(segment_only_location {shuttle})')
    for side in SIDES:
        side_has_obstacle = bool(obstacles.get(side))
        for from_slot in ('1', '2', '3', '4'):
            from_object = _slot_object(side, from_slot)
            for to_slot in ('1', '2', '3', '4'):
                to_object = _slot_object(side, to_slot)
                cost = abs(int(from_slot) - int(to_slot))
                if cost == 0:
                    cost = 1
                facts.append(f'(= (route_cost {from_object} {to_object}) {cost})')
                if (
                    not side_has_obstacle
                    and (from_object, to_object) in route_clearance['clear_pairs']
                ):
                    facts.append(f'(route_clear_between {from_object} {to_object})')
                for blocker in route_clearance['blockers_by_pair'].get(
                    (from_object, to_object),
                    (),
                ):
                    facts.append(
                        f'(route_blocked_by {from_object} {to_object} {blocker})'
                    )
    for front, rear in sorted(route_clearance['ordering_pairs']):
        facts.extend([
            f'(front_of {front} {rear})',
            f'(behind {rear} {front})',
        ])
    for route in (
        route_clearance.get('topology_routes', {}).get('routes') or []
    ):
        shuttle = route['shuttle']
        source_block = route['source_block']
        target_slot = route['target_slot_object']
        facts.append(
            f'(topology_route_available {shuttle} {source_block} {target_slot})'
        )
        if route.get('route_clear'):
            facts.append(
                f'(topology_route_clear {shuttle} {source_block} {target_slot})'
            )
        for blocker in route.get('blockers') or []:
            facts.append(
                f'(topology_route_blocked_by {shuttle} {source_block} '
                f'{target_slot} {blocker})'
            )
        if route.get('configured'):
            facts.append(
                f'(topology_route_configured {shuttle} {source_block} '
                f'{target_slot})'
            )
    # Bind ordering to the exact route selected by the frozen clearance plan.
    # This covers both slot routes and arbitrary segment origins without
    # manufacturing a source-slot fact for a visually segment-bound shuttle.
    clearance_selected = str(
        target_clearance.get('selected_shuttle')
        or goal_data.get('selected_shuttle')
        or ''
    )
    for relocation in relocations:
        blocker = str(relocation.get('shuttle') or '')
        if (
            clearance_selected
            and blocker
            and blocker != clearance_selected
        ):
            facts.append(
                f'(clearance_precedes {blocker} {clearance_selected})'
            )
    for slot_object, occupant in sorted(fleet['slot_occupancy'].items()):
        if occupant:
            facts.append(f'(slot_occupied_by {slot_object} {occupant})')
        else:
            facts.append(f'(slot_free {slot_object})')
    for block_object in sorted(blocks['blocks']):
        block_metadata = blocks['blocks'][block_object]
        if not block_metadata.get('slot'):
            facts.append(
                f'(block_on_side {block_object} {block_metadata["side"]})'
            )
        occupant = blocks['block_occupancy'].get(block_object, '')
        reservation = blocks['block_reservations'].get(block_object, '')
        if occupant:
            facts.append(f'(block_occupied_by {block_object} {occupant})')
        else:
            facts.append(f'(block_free {block_object})')
        if reservation:
            facts.append(f'(block_reserved_by {block_object} {reservation})')
    for switch, state in sorted(devices['switches'].items()):
        facts.append(f'(switch_state_known {switch})')
        facts.append(f'(switch_{state} {switch})')
    for stopper, state in sorted(devices['stoppers'].items()):
        facts.append(f'(stopper_state_known {stopper})')
        facts.append(f'(stopper_{state} {stopper})')
    # The executive rebuilds the PDDL problem after every supervised atomic
    # step.  Persist readiness predicates from the freshly observed physical
    # device state; otherwise every replan starts again with
    # prepare_switches/open_stoppers and can never reach a move action.
    for side in SIDES:
        side_switches = {
            _switch_object(side, device)
            for device in DEVICE_NAMES
        }
        side_stoppers = {
            _stopper_object(side, device)
            for device in DEVICE_NAMES
        }
        switches_ready = all(
            devices['switches'].get(name) == 'exterior'
            for name in side_switches
        )
        stoppers_open = all(
            devices['stoppers'].get(name) == 'open'
            for name in side_stoppers
        )
        if switches_ready:
            facts.append(f'(switches_ready {side})')
        if stoppers_open:
            facts.append(f'(stoppers_open {side})')
        if switches_ready and stoppers_open:
            facts.append(f'(normal_route {side})')
            stations = (
                ('yaskawa', 'staubli')
                if side == 'right'
                else ('yaskawa', 'kuka')
            )
            for source in stations:
                for target in stations:
                    facts.append(
                        f'(path_ready {side} '
                        f'{_station_object(side, source)} '
                        f'{_station_object(side, target)})'
                    )
        if _active_interior_clearance_gate(side=side, devices=devices):
            facts.append(f'(clearance_mode {side})')
        normalization = (
            route_clearance.get('normalization', {})
            .get('by_side', {})
            .get(side, {})
        )
        if (
            normalization.get('reconfiguration_required')
            and not normalization.get('configured_clear_motion_route')
        ):
            facts.append(f'(route_reconfiguration_required {side})')
        if normalization.get('reconfiguration_safe'):
            facts.append(f'(route_reconfiguration_safe {side})')
        if normalization.get('clearance_pause_safe'):
            facts.append(f'(clearance_pause_safe {side})')
    for side, side_obstacles in sorted(obstacles.items()):
        for obstacle in side_obstacles:
            facts.append(f'(obstacle_present {obstacle} {side})')
    if goal_data['goal_type'] == 'transport':
        facts.append(f'(active_goal_side {goal_data["side"]})')
        target_station = _station_object(goal_data['side'], goal_data['target_station'])
        facts.append(f'(target_station_for_goal {target_station})')
        if goal_data['target_slot']:
            facts.append(f'(target_slot_for_goal {_slot_object(goal_data["side"], goal_data["target_slot"])})')
        else:
            facts.append('(station_only_goal)')
        for shuttle in goal_data['candidate_shuttles']:
            facts.append(f'(goal_candidate {shuttle})')
    else:
        facts.append(f'(inspection_required {goal_data["inspection_subject"]})')
    return _unique_sorted_facts(facts)


def _pddl_goal_text(goal_data: dict[str, Any]) -> str:
    if goal_data['goal_type'] == 'inspection':
        return f'(inspection_done {goal_data["inspection_subject"]})'
    target_station = _station_object(goal_data['side'], goal_data['target_station'])
    if goal_data['planner_selects_candidate']:
        if goal_data['target_slot']:
            return (
                f'(and (transport_goal_done {target_station}) '
                f'(goal_slot_reached {_slot_object(goal_data["side"], goal_data["target_slot"])}))'
            )
        return f'(transport_goal_done {target_station})'
    selected = goal_data['selected_shuttle']
    if goal_data['target_slot']:
        return (
            f'(and (task_done {selected} {target_station}) '
            f'(shuttle_at_slot {selected} {_slot_object(goal_data["side"], goal_data["target_slot"])}))'
        )
    return f'(task_done {selected} {target_station})'


def _format_pddl_problem(
    *,
    problem_name: str,
    objects: dict[str, list[str]],
    init_facts: list[str],
    goal_text: str,
) -> str:
    object_lines = []
    for pddl_type, names in objects.items():
        object_lines.append(f'    {" ".join(sorted(names))} - {pddl_type}')
    init_lines = [f'    {fact}' for fact in init_facts]
    return (
        f'(define (problem {problem_name})\n'
        '  (:domain room315-shuttle)\n\n'
        '  (:objects\n'
        + '\n'.join(object_lines)
        + '\n  )\n\n'
        '  (:init\n'
        + '\n'.join(init_lines)
        + '\n  )\n\n'
        '  (:goal\n'
        f'    {goal_text}\n'
        '  )\n\n'
        '  (:metric minimize (total-cost))\n'
        ')\n'
    )


def _unique_sorted_facts(facts: list[str]) -> list[str]:
    return sorted(dict.fromkeys(facts))


def _normalise_planning_side(value: Any) -> str:
    text = str(value or '').strip().casefold()
    if text in {'right', 'r'}:
        return 'right'
    if text in {'left', 'l'}:
        return 'left'
    raise PddlProblemBuildError(f'invalid Room 315 side {value!r}')


def _task_goal_target_shuttle(constraints: dict[str, Any], *, side: str) -> str:
    raw = constraints.get('target_shuttle')
    if not raw:
        return ''
    shuttle = _canonical_planning_shuttle_id(raw, side=side)
    if _shuttle_side(shuttle) != side:
        raise PddlProblemBuildError(
            f'target_shuttle {raw!r} is not on requested {side!r} rail'
        )
    return shuttle


def _require_goal_candidate_location(fleet: dict[str, Any], shuttle: str) -> None:
    if (
        shuttle not in fleet['location_slot_by_shuttle']
        and shuttle not in fleet['rail_position_by_shuttle']
    ):
        raise PddlProblemBuildError(
            f'goal candidate {shuttle!r} has no known topology location; '
            'observation or recovery is required'
        )


def _goal_candidate_route_cost(
    fleet: dict[str, Any],
    *,
    shuttle: str,
    target_slot: str,
) -> float:
    """Rank candidates by authoritative forward route length, not slot index."""

    position = (
        fleet.get('planning_rail_position_by_shuttle')
        or fleet['rail_position_by_shuttle']
    ).get(shuttle)
    if not isinstance(position, dict):
        return math.inf
    side = _shuttle_side(shuttle)
    try:
        route = route_plan_from_position_to_slot(
            _planning_rail_topology(side),
            position['segment'],
            position['s_ratio'],
            target_slot,
        )
    except (KeyError, TypeError, ValueError):
        return math.inf
    segment_lengths = rail_segment_lengths(side)
    return sum(
        max(0.0, float(block.end_s_ratio) - float(block.start_s_ratio))
        * float(segment_lengths[block.segment])
        for block in route.blocks
    )


def _occupancy_shuttle_from_value(value: Any, *, side: str, slot: str) -> str:
    raw: Any = value
    occupied = False
    if isinstance(value, dict):
        occupied = bool(value.get('occupied', False))
        raw = value.get('shuttle') or value.get('occupant') or value.get('shuttle_id')
        if not occupied:
            return ''
    elif value is None or value is False:
        return ''
    else:
        occupied = True
    if not raw:
        if occupied:
            raise PddlProblemBuildError(
                f'occupied slot {side}:{slot} lacks shuttle identity; observation recovery required'
            )
        return ''
    shuttle = _canonical_planning_shuttle_id(raw, side=side)
    if _shuttle_side(shuttle) != side:
        raise PddlProblemBuildError(
            f'occupancy for {side}:{slot} references shuttle on another rail: {raw!r}'
        )
    return shuttle


def _slot_symbol_from_observation(value: Any, *, default_side: str) -> str:
    text = str(value or '').strip()
    for (side, slot), sensor in SLOT_SENSOR_BY_SIDE_AND_SLOT.items():
        if text.upper() == sensor:
            return _slot_object(side, slot)
    match = re.search(r'\b(right|left)[:_-]?slot[:_-]?([1-4])\b', text, re.IGNORECASE)
    if match:
        return _slot_object(match.group(1).lower(), match.group(2))
    match = re.search(r'\b([1-4])\b', text)
    if match:
        return _slot_object(default_side, match.group(1))
    raise PddlProblemBuildError(f'could not parse Room 315 slot observation {value!r}')


def _slot_number_from_object(slot_object: str) -> str:
    match = re.search(r'([1-4])$', str(slot_object or ''))
    if not match:
        raise PddlProblemBuildError(f'invalid slot object {slot_object!r}')
    return match.group(1)


def _slot_for_station_default(side: str, station: str) -> str:
    station = _station_symbol(station)
    for slot in ('1', '2', '3', '4'):
        if _station_for_slot(side, slot) == station:
            return slot
    return ''


def _contract_slot_id(side: str, slot: str) -> str:
    return f'{side}:slot:{_slot_symbol_or_empty(slot)}'


def _block_id_for_slot(side: str, slot: str) -> str:
    return f'{side}:block:slot:{_slot_symbol_or_empty(slot)}'


def _slot_object(side: str, slot: str) -> str:
    return f'{side}_slot_{_slot_symbol_or_empty(slot)}'


def _block_object_for_slot(side: str, slot: str) -> str:
    return f'{side}_block_slot_{_slot_symbol_or_empty(slot)}'


def _topology_block_object(side: str, segment: str) -> str:
    return f'{_normalise_planning_side(side)}_topology_{_pddl_symbol(segment)}'


def _station_object(side: str, station: str) -> str:
    return f'{side}_{_station_symbol(station)}'


def _switch_object(side: str, device: str) -> str:
    return f'{side}_switch_{str(device).upper()}'.lower()


def _stopper_object(side: str, device: str) -> str:
    return f'{side}_stopper_{str(device).upper()}'.lower()


def _switch_state_for_pddl(value: Any) -> str:
    text = str(value or '').strip().casefold()
    if text in {'interior', 'i'}:
        return 'interior'
    if text in {'exterior', 'e'}:
        return 'exterior'
    raise PddlProblemBuildError(f'invalid switch state {value!r}; expected EXTERIOR or INTERIOR')


def _shuttle_side(shuttle: str) -> str:
    text = str(shuttle or '').strip().casefold()
    if text.startswith('left_') or text.startswith('l'):
        return 'left'
    return 'right'


def _pddl_symbol(value: Any) -> str:
    text = str(value or '').strip().casefold().replace('-', '_').replace(':', '_')
    text = re.sub(r'[^a-z0-9_]+', '_', text)
    text = re.sub(r'_+', '_', text).strip('_')
    if not text:
        text = 'unnamed'
    if text[0].isdigit():
        text = f'n_{text}'
    return text


def _pddl_problem_name(value: Any) -> str:
    text = str(value or '').strip().casefold().replace(':', '_')
    text = re.sub(r'[^a-z0-9_-]+', '_', text)
    text = re.sub(r'_+', '_', text).strip('_-')
    if not text:
        text = 'room315_problem'
    if text[0].isdigit():
        text = f'n-{text}'
    return text


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
    if selected_slot == target_slot:
        raise ValueError(
            f'goal {goal_id!r} selected {selected_shuttle!r} already in target slot '
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
    if policy in {'none', 'no_restore', 'skip_restore', 'clear_only'}:
        return {
            'blocker_restore_slot': '',
            'blocker_restore_policy': policy,
            'blocker_restore_slot_source': 'not_requested',
            'blocker_restore_candidate_slots': (),
        }
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


def scenario_spec_from_inputs(
    *,
    case_id: str = '',
    case_config: Path | str | None = None,
) -> ScenarioSpec:
    if case_id:
        return scenario_spec_from_case(case_id, case_config=case_config)
    raise ValueError('provide --case-id')


def payload_training_cases_by_id(config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    cases = config.get('cases', [])
    if not isinstance(cases, list):
        raise ValueError('payload training case config needs a cases list')
    return {
        case_id: dict(case)
        for case in cases
        if isinstance(case, dict)
        and (case_id := str(case.get('case_id') or '').strip())
    }


def select_payload_training_cases(
    config: dict[str, Any],
    requested_ids: list[str],
) -> list[dict[str, Any]]:
    by_id = payload_training_cases_by_id(config)
    if not requested_ids:
        return list(by_id.values())
    unknown = [case_id for case_id in requested_ids if case_id not in by_id]
    if unknown:
        allowed = ', '.join(sorted(by_id))
        raise ValueError(f'unknown case id(s): {", ".join(unknown)}; allowed: {allowed}')
    return [dict(by_id[case_id]) for case_id in requested_ids]


def launch_bool(value: Any, *, default: bool) -> bool:
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    return str(value).strip().casefold() in {'1', 'true', 'yes', 'on'}


def scenario_spec_from_case(
    case_id: str,
    *,
    case_config: Path | str | None = None,
) -> ScenarioSpec:
    raw_case_id = str(case_id or '').strip()
    if not raw_case_id:
        raise ValueError('case_id must not be empty')
    config = load_payload_training_case_config(case_config)
    by_id = payload_training_cases_by_id(config)
    if raw_case_id not in by_id:
        allowed = ', '.join(sorted(case_id for case_id in by_id if case_id))
        raise ValueError(f'unknown payload training case {raw_case_id!r}; allowed: {allowed}')

    data = dict(by_id[raw_case_id])
    data.setdefault('payload_condition', 'loaded')
    data.setdefault('selection_policy', 'nearest_loaded_to_target_slot_then_lowest_id')
    data.setdefault('problem_name', f'room315-{_clean_symbol(raw_case_id)}')
    if data.get('selection_policy'):
        data = _resolve_nearest_loaded_goal_data(raw_case_id, data)
    return _scenario_spec_from_case_data(raw_case_id, data)


def load_payload_training_case_config(path: Path | str | None = None) -> dict[str, Any]:
    config_path = _resolve_payload_training_case_config_path(path)
    loaded = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f'payload training case config {config_path} must be a YAML mapping')
    loaded.setdefault('case_config_path', str(config_path))
    return loaded


def speed_for_payload_training_case(
    case_id: str,
    case_config: Path | str | None = None,
    *,
    fallback: float = DEFAULT_SHUTTLE_SPEED_MPS,
) -> float:
    raw_case_id = str(case_id or '').strip()
    if not raw_case_id:
        return float(fallback)
    config = load_payload_training_case_config(case_config)
    cases = config.get('cases', [])
    if not isinstance(cases, list):
        return float(fallback)
    for case in cases:
        if not isinstance(case, dict):
            continue
        if str(case.get('case_id') or '').strip() != raw_case_id:
            continue
        case_speed = _optional_float(case.get('speed', case.get('speed_mps')))
        return float(case_speed if case_speed is not None else fallback)
    return float(fallback)


def _resolve_payload_training_case_config_path(path: Path | str | None = None) -> Path:
    raw_path = Path(path).expanduser() if path else DEFAULT_PAYLOAD_TRAINING_CASES_PATH
    candidates = [raw_path]
    if not raw_path.is_absolute():
        candidates.extend([
            Path.cwd() / raw_path,
            REPO_ROOT / raw_path,
            Path.cwd() / 'src' / 'mfja_3rd_floor_gz' / raw_path,
        ])
    candidates.extend([
        REPO_ROOT
        / 'mfja_robot_control_config'
        / 'config'
        / 'room_315_payload_cases'
        / raw_path.name,
        SCRIPT_DIR.parents[1]
        / 'share'
        / 'mfja_robot_control_config'
        / 'config'
        / 'room_315_payload_cases'
        / raw_path.name,
    ])

    seen: set[str] = set()
    for candidate in candidates:
        key = str(candidate)
        if key in seen:
            continue
        seen.add(key)
        if candidate.exists():
            return candidate
    return raw_path


def _normalize_clearance_steps(value: Any) -> tuple[dict[str, Any], ...]:
    if value is None or value == '':
        return ()
    if not isinstance(value, (list, tuple)):
        raise ValueError('clearance_steps must be a list of mappings')
    normalized = []
    for index, raw_step in enumerate(value, start=1):
        if not isinstance(raw_step, dict):
            raise ValueError(f'clearance_steps[{index}] must be a mapping')
        step = dict(raw_step)
        if step.get('shuttle') or step.get('blocker_shuttle'):
            step['shuttle'] = _clean_symbol(
                step.get('shuttle') or step.get('blocker_shuttle')
            ).lower()
        if step.get('start_slot') or step.get('blocker_start_slot'):
            step['start_slot'] = _slot_symbol_or_empty(
                step.get('start_slot') or step.get('blocker_start_slot')
            )
        if step.get('clear_slot') or step.get('target_slot'):
            step['clear_slot'] = _slot_symbol_or_empty(
                step.get('clear_slot') or step.get('target_slot')
            )
        if step.get('clear_target') or step.get('target'):
            step['clear_target'] = str(
                step.get('clear_target') or step.get('target')
            ).strip()
        if step.get('clear_stopper') or step.get('gate_stopper'):
            step['clear_stopper'] = _stopper_symbol_or_empty(
                step.get('clear_stopper') or step.get('gate_stopper')
            )
        if step.get('clear_sensor'):
            step['clear_sensor'] = str(step.get('clear_sensor') or '').strip()
        normalized.append(step)
    return tuple(normalized)


def _scenario_spec_from_case_data(goal_id: str, data: dict[str, Any]) -> ScenarioSpec:
    payload_condition = str(data.get('payload_condition') or '')
    pddl_goal = f'{data["shuttle"]} at {data["target"]}'
    if payload_condition:
        pddl_goal = f'{payload_condition} {pddl_goal}'
    return ScenarioSpec(
        goal_id=goal_id,
        side=data['side'],
        shuttle=data['shuttle'],
        source=data['source'],
        target=data['target'],
        pddl_problem=data['problem_name'],
        pddl_goal=pddl_goal,
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
        blocker_clear_target=str(data.get('blocker_clear_target') or ''),
        blocker_clear_sensor=str(data.get('blocker_clear_sensor') or ''),
        blocker_clear_stopper=_stopper_symbol_or_empty(data.get('blocker_clear_stopper')),
        blocker_restore_slot=str(data.get('blocker_restore_slot') or ''),
        blocker_restore_policy=str(data.get('blocker_restore_policy') or ''),
        blocker_restore_slot_source=str(data.get('blocker_restore_slot_source') or ''),
        blocker_restore_candidate_slots=tuple(data.get('blocker_restore_candidate_slots') or ()),
        clearance_strategy=str(data.get('clearance_strategy') or ''),
        clearance_steps=_normalize_clearance_steps(data.get('clearance_steps') or ()),
    )


def write_scenario(path: Path, scenario: dict[str, Any]) -> None:
    path.expanduser().parent.mkdir(parents=True, exist_ok=True)
    path.expanduser().write_text(_json_dumps(scenario) + '\n', encoding='utf-8')


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


def _stopper_symbol_or_empty(value: Any) -> str:
    text = _clean_symbol(str(value or '')).upper()
    if text in {'A1', 'A2', 'A3', 'A4'}:
        return text
    return ''


def _stopper_state_symbol(value: Any, *, default: str = 'open') -> str:
    text = str(value or '').strip().casefold()
    if text in {'0', 'open', 'opened', 'release', 'released', 'off', 'false'}:
        return 'open'
    if text in {'1', 'closed', 'close', 'stop', 'blocked', 'on', 'true'}:
        return 'closed'
    if default:
        return default
    raise ValueError(f'invalid stopper state {value!r}; expected open or closed')


def _clean_symbol(value: str) -> str:
    return str(value or '').strip().strip('()[]{}:,').replace('-', '_')


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


def _rail_stoppers_from_status(status: dict[str, Any], side: str) -> dict[str, Any]:
    rails = status.get('rails', {}) if isinstance(status, dict) else {}
    rail = rails.get(side, {}) if isinstance(rails, dict) else {}
    stoppers = rail.get('stoppers', {}) if isinstance(rail, dict) else {}
    if not isinstance(stoppers, dict):
        return {}
    return {str(name).strip().upper(): value for name, value in stoppers.items()}


def _rail_switches_from_status(status: dict[str, Any], side: str) -> dict[str, Any]:
    rails = status.get('rails', {}) if isinstance(status, dict) else {}
    rail = rails.get(side, {}) if isinstance(rails, dict) else {}
    switches = rail.get('switches', {}) if isinstance(rail, dict) else {}
    if not isinstance(switches, dict):
        return {}
    return {str(name).strip().upper(): value for name, value in switches.items()}


def _switches_match_status(
    status: dict[str, Any],
    side: str,
    expected: dict[str, str],
) -> bool:
    if not expected:
        return True
    actual = _rail_switches_from_status(status, side)
    for name, expected_state in expected.items():
        if name not in actual:
            return False
        if _switch_state_value(actual.get(name)) != expected_state:
            return False
    return True


def _stoppers_match_status(
    status: dict[str, Any],
    side: str,
    expected: dict[str, str],
) -> bool:
    if not expected:
        return True
    actual = _rail_stoppers_from_status(status, side)
    for name, expected_state in expected.items():
        if name not in actual:
            return False
        if _stopper_state_value(actual.get(name)) != expected_state:
            return False
    return True


def _expanded_switch_assignments(assignments: dict[str, Any]) -> dict[str, str]:
    expanded: dict[str, str] = {}
    for raw_name, raw_state in assignments.items():
        name = str(raw_name or '').strip().upper()
        state = _switch_state_value(raw_state)
        if name == 'ALL':
            for switch_name in ('A1', 'A2', 'A3', 'A4'):
                expanded[switch_name] = state
        elif name in {'A1', 'A2', 'A3', 'A4'}:
            expanded[name] = state
    return expanded


def _expanded_stopper_assignments(assignments: dict[str, Any]) -> dict[str, str]:
    expanded: dict[str, str] = {}
    for raw_name, raw_state in assignments.items():
        name = str(raw_name or '').strip().upper()
        state = _stopper_state_value(raw_state)
        if name == 'ALL':
            for stopper_name in ('A1', 'A2', 'A3', 'A4'):
                expanded[stopper_name] = state
        elif name in {'A1', 'A2', 'A3', 'A4'}:
            expanded[name] = state
    return expanded


def _switch_state_value(value: Any) -> str:
    text = str(value or '').strip().upper()
    if text in {'1', 'E', 'EXTERIOR'}:
        return 'EXTERIOR'
    if text in {'2', 'I', 'INTERIOR'}:
        return 'INTERIOR'
    return text


def _stopper_state_value(value: Any) -> str:
    return '1' if _stopper_state_symbol(value, default='open') == 'closed' else '0'


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


def _matched_target_sensor_names_from_status(
    status: dict[str, Any],
    *,
    side: str,
    wanted: set[str],
    shuttle: str,
) -> list[str]:
    matched_readings = _active_sensor_readings_for_names(status, side, wanted)
    if not matched_readings:
        return []
    target_shuttle = _shuttle_entity_name(shuttle, side)
    if not target_shuttle:
        return sorted({
            _sensor_name_from_reading(reading).upper()
            for reading in matched_readings
            if _sensor_name_from_reading(reading)
        })
    readings_with_identity = [
        reading
        for reading in matched_readings
        if str(reading.get('shuttle') or '').strip()
    ]
    if not readings_with_identity:
        return sorted({
            _sensor_name_from_reading(reading).upper()
            for reading in matched_readings
            if _sensor_name_from_reading(reading)
        })
    return sorted({
        _sensor_name_from_reading(reading).upper()
        for reading in readings_with_identity
        if (
            _sensor_name_from_reading(reading)
            and _shuttle_entity_name(reading.get('shuttle'), side) == target_shuttle
        )
    })


def _target_slot_pose_match_from_status(
    status: dict[str, Any],
    *,
    side: str,
    target_slot: str,
    shuttle: str,
) -> dict[str, Any]:
    slot = _slot_symbol_or_empty(target_slot)
    if not slot:
        return {}
    expected = SLOT_POSE_BY_SIDE_AND_SLOT.get((side, slot))
    if expected is None:
        return {}
    target_segment, target_s = expected
    result = _target_pose_match_from_status(
        status,
        side=side,
        shuttle=shuttle,
        target_segment=target_segment,
        target_s=target_s,
        tolerance_m=SLOT_POSE_ARRIVAL_TOLERANCE_M,
        allow_overshot=False,
    )
    if result:
        result['target_slot'] = slot
    return result


def _target_pose_match_from_status(
    status: dict[str, Any],
    *,
    side: str,
    shuttle: str,
    target_segment: str,
    target_s: float,
    tolerance_m: float | None = None,
    allow_overshot: bool = True,
) -> dict[str, Any]:
    target_segment = str(target_segment or '').strip().upper()
    if not target_segment:
        return {}
    tolerance = (
        float(tolerance_m)
        if tolerance_m is not None
        else SLOT_POSE_ARRIVAL_TOLERANCE_M
    )
    state = _shuttle_state_from_status(status, side=side, shuttle=shuttle)
    if not state:
        return {}
    segment = str(
        state.get('segment')
        or state.get('current_segment')
        or state.get('block')
        or ''
    ).strip().upper()
    if segment != str(target_segment).upper():
        return {
            'arrived': False,
            'reason': '',
            'expected_segment': target_segment,
            'current_segment': segment,
        }
    try:
        current_s = float(state.get('s'))
    except (TypeError, ValueError):
        return {
            'arrived': False,
            'reason': '',
            'expected_segment': target_segment,
            'current_segment': segment,
        }
    distance_m = abs(current_s - float(target_s))
    arrived = distance_m <= tolerance
    overshot = current_s > float(target_s) + tolerance
    return {
        'arrived': arrived or (overshot and allow_overshot),
        'reason': (
            'target slot pose matched'
            if arrived
            else ('target slot pose overshot' if overshot and allow_overshot else '')
        ),
        'expected_segment': target_segment,
        'expected_s': round(float(target_s), 4),
        'current_segment': segment,
        'current_s': round(current_s, 4),
        'distance_m': round(distance_m, 4),
        'tolerance_m': tolerance,
        'overshot': overshot,
    }


def _shuttle_state_from_status(
    status: dict[str, Any],
    *,
    side: str,
    shuttle: str,
) -> dict[str, Any]:
    rails = status.get('rails', {}) if isinstance(status, dict) else {}
    rail = rails.get(side, {}) if isinstance(rails, dict) else {}
    shuttles = rail.get('shuttles', {}) if isinstance(rail, dict) else {}
    if not isinstance(shuttles, dict):
        return {}
    target = _shuttle_entity_name(shuttle, side)
    if not target:
        return {}
    if target and isinstance(shuttles.get(target), dict):
        return dict(shuttles[target])
    for key, value in shuttles.items():
        if not isinstance(value, dict):
            continue
        candidates = [
            key,
            value.get('entity_name'),
            value.get('name'),
            value.get('shuttle'),
            value.get('shuttle_id'),
            value.get('short_id'),
        ]
        if any(_shuttle_entity_name(candidate, side) == target for candidate in candidates):
                return dict(value)
    return {}


def _shuttle_state_mode(state: dict[str, Any]) -> str:
    if not isinstance(state, dict):
        return ''
    return str(state.get('mode') or '').strip().upper()


def _shuttle_state_is_falling(state: dict[str, Any]) -> bool:
    return _shuttle_state_mode(state) in {'FALLING', 'FALLEN'}


def _shuttle_state_is_stopped(state: dict[str, Any]) -> bool:
    if not isinstance(state, dict) or not state:
        return False
    mode = _shuttle_state_mode(state)
    if _shuttle_state_is_falling(state):
        return False
    # An enabled shuttle held by a stopper/collision also reports WAITING.
    # Only the kinematic controller's explicit post-OFF mode proves disable.
    return mode == 'DISABLED'


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
    explicit_target = _shuttle_entity_name(scenario.get('target_shuttle_id'), side)
    if explicit_target:
        return explicit_target
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
        description='Generate or execute one curated Room 315 payload training case.'
    )
    parser.add_argument(
        '--case-id',
        required=True,
        help='Payload training case id from --case-config.',
    )
    parser.add_argument(
        '--case-config',
        type=Path,
        default=DEFAULT_PAYLOAD_TRAINING_CASES_PATH,
        help='YAML payload training case matrix for --case-id.',
    )
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
    parser.add_argument(
        '--speed',
        type=float,
        default=None,
        help=(
            'move_shuttle speed annotation. With --case-id, defaults to the '
            'case YAML speed when present; otherwise defaults to 0.3.'
        ),
    )
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
    mode_group.add_argument(
        '--execute',
        action='store_true',
        help='Publish generated commands through the rail-safety supervisor.',
    )
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
    parser.add_argument('--command-topic', default='/room_315/rail_safety/primitive_command')
    parser.add_argument('--episode-control-topic', default='/room_315/visual_dataset/episode_control')
    parser.add_argument('--status-topic', default='/room_315/rail_safety/status')
    parser.add_argument('--dataset-status-topic', default='/room_315/visual_dataset/status')
    parser.add_argument(
        '--require-dataset-recorder',
        action='store_true',
        help='Fail execute mode unless the dataset recorder acknowledges episode start/stop.',
    )
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

    scenario_speed = (
        float(args.speed)
        if args.speed is not None
        else speed_for_payload_training_case(args.case_id, args.case_config)
    )
    scenario = generate_scenario(
        case_id=args.case_id or '',
        case_config=args.case_config,
        language_seed=args.language_seed,
        language_template_id=args.language_template_id,
        speed=scenario_speed,
        planner=planner,
    )
    if args.preflight_only or args.execute:
        transport = RosScenarioTransport(
            command_topic=args.command_topic,
            episode_control_topic=args.episode_control_topic,
            status_topic=args.status_topic,
            dataset_status_topic=args.dataset_status_topic,
            ros_args=ros_argv or None,
            require_dataset_recorder=args.require_dataset_recorder,
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
