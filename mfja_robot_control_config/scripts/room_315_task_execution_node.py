#!/usr/bin/env python3
"""ROS 2 gateway from confirmed Room 315 TaskGoal to closed-loop execution."""

from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from mfja_rail_interfaces.msg import SensorFeedback
from mfja_rail_interfaces.msg import VisualStateObservation
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from room_315_closed_loop_executive import ClosedLoopExecutive
from room_315_closed_loop_executive import ClosedLoopExecutiveConfig
from room_315_contracts import ContractValidationError
from room_315_contracts import TaskGoal
from room_315_pddl_scenario_generator import PlanSysPlannerBackend
from room_315_task_execution import LatestVisualObservedStateProvider
from room_315_task_execution import LiveStateConfig
from room_315_task_execution import VisualObservedStateBuilder
from room_315_task_execution import VisualSupervisorTransport
from room_315_task_execution import build_runtime_payload_grounding
from room_315_task_execution import ground_transport_task_goal_stably
from room_315_task_execution_config import TASK_EXECUTION_PARAMETER_DEFAULTS
from room_315_task_execution_config import validate_task_execution_parameters


TERMINAL_STATUSES = frozenset({'succeeded', 'aborted', 'rejected', 'failed'})


class Room315TaskExecutionNode(Node):
    """Validate TaskGoals and execute them through PlanSys2 and the supervisor."""

    def __init__(self) -> None:
        super().__init__('room_315_task_execution_node')
        self._declare_parameters()
        state_config = LiveStateConfig(
            observation_timeout_s=self._float('observation_timeout_s'),
            supervisor_status_timeout_s=self._float(
                'supervisor_status_timeout_s'
            ),
            slot_sensor_state_timeout_s=self._float(
                'slot_sensor_state_timeout_s'
            ),
            planning_slot_tolerance_ratio=self._float(
                'planning_slot_tolerance_ratio'
            ),
            target_arrival_tolerance_ratio=self._float(
                'target_arrival_tolerance_ratio'
            ),
            position_consistency_tolerance_m=self._float(
                'position_consistency_tolerance_m'
            ),
            observation_wait_s=self._float('observation_wait_s'),
            external_obstacles_disabled=self._bool(
                'external_obstacles_disabled'
            ),
        )
        self.state_builder = VisualObservedStateBuilder(state_config)
        self.state_provider = LatestVisualObservedStateProvider(
            self.state_builder,
            slot_sensor_confirmation_frames=self._int(
                'slot_sensor_confirmation_frames'
            ),
        )
        self.command_pub = self.create_publisher(
            String,
            self._string('supervisor_command_topic'),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            self._string('task_status_topic'),
            10,
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            self._string('diagnostics_topic'),
            10,
        )
        self.transport = VisualSupervisorTransport(
            provider=self.state_provider,
            publish_callback=self._publish_supervisor_command,
            slot_sensor_confirmation_frames=self._int(
                'slot_sensor_confirmation_frames'
            ),
            controller_stop_timeout_s=self._float(
                'controller_stop_timeout_s'
            ),
        )
        self.task_goal_sub = self.create_subscription(
            String,
            self._string('task_goal_topic'),
            self._on_task_goal,
            10,
        )
        self.visual_observation_sub = self.create_subscription(
            VisualStateObservation,
            self._string('accepted_observed_state_topic'),
            self._on_visual_observation,
            10,
        )
        self.supervisor_status_sub = self.create_subscription(
            String,
            self._string('supervisor_status_topic'),
            self._on_supervisor_status,
            10,
        )
        self.slot_sensor_subscriptions = [
            self.create_subscription(
                SensorFeedback,
                self._string(f'{side}_sensor_feedback_topic'),
                lambda message, rail_side=side: self._on_sensor_feedback(
                    rail_side,
                    message,
                ),
                10,
            )
            for side in ('left', 'right')
        ]

        self._lock = threading.Lock()
        self._worker: threading.Thread | None = None
        self._active_goal_id = ''
        self._last_goal_id = ''
        self._last_status = 'idle'
        self._last_reason = 'waiting_for_ready_inputs'
        self._publish_status(
            status='idle',
            reason='task execution node initialized',
        )
        self.create_timer(
            max(self._float('diagnostic_period_s'), 0.1),
            self._publish_diagnostics,
        )
        self.get_logger().info(
            'Room 315 TaskGoal execution gateway initialized. '
            'Planning localization comes from validated visual state; final '
            'slot arrival requires deterministic sensor identity plus '
            'controller-stop confirmation.'
        )

    def _declare_parameters(self) -> None:
        for name, default in TASK_EXECUTION_PARAMETER_DEFAULTS.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
        validate_task_execution_parameters({
            name: self.get_parameter(name).value
            for name in TASK_EXECUTION_PARAMETER_DEFAULTS
        })

    def _on_visual_observation(
        self,
        message: VisualStateObservation,
    ) -> None:
        payload = _visual_message_dict(message)
        receive_s = time.monotonic()
        self.state_provider.update_observation(
            payload,
            receive_s=receive_s,
        )
        self.transport.update_observation(payload)

    def _on_supervisor_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data or '{}')
        except json.JSONDecodeError as exc:
            self._last_reason = f'invalid_supervisor_status_json:{exc}'
            return
        if not isinstance(payload, dict):
            self._last_reason = 'invalid_supervisor_status_type'
            return
        receive_s = time.monotonic()
        self.state_provider.update_supervisor(
            payload,
            receive_s=receive_s,
        )
        self.transport.update_supervisor(payload)

    def _on_sensor_feedback(
        self,
        side: str,
        message: SensorFeedback,
    ) -> None:
        readings = [
            {
                'active': bool(reading.active),
                'name': str(reading.name),
                'shuttle': str(reading.shuttle_name),
            }
            for reading in message.readings
        ]
        self.transport.update_sensor_feedback(side, readings)
        self.state_provider.update_slot_sensor_feedback(
            side,
            readings,
            receive_s=time.monotonic(),
        )

    def _on_task_goal(self, message: String) -> None:
        if not self._bool('execution_enabled'):
            self._reject_goal(
                goal_id='',
                reason='task_execution_disabled',
            )
            return
        try:
            payload = json.loads(message.data or '{}')
            if not isinstance(payload, dict):
                raise ContractValidationError('TaskGoal payload must be an object')
            task_goal = TaskGoal.from_dict(payload)
        except (json.JSONDecodeError, ContractValidationError, TypeError) as exc:
            self._reject_goal(
                goal_id='',
                reason=f'invalid_task_goal:{exc}',
            )
            return

        goal_type = str(
            task_goal.constraints.get('goal_type') or ''
        ).strip().casefold()
        if goal_type not in {'transport', 'inspection'}:
            self._reject_goal(
                goal_id=task_goal.goal_id,
                reason=(
                    'runtime_supports_transport_or_inspection_goals_only'
                ),
            )
            return
        ready, reason = self.state_provider.ready()
        if not ready:
            self._reject_goal(
                goal_id=task_goal.goal_id,
                reason=f'execution_state_not_ready:{reason}',
            )
            return
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                self._reject_goal(
                    goal_id=task_goal.goal_id,
                    reason=f'executor_busy:{self._active_goal_id}',
                )
                return
            if (
                task_goal.goal_id == self._last_goal_id
                and self._last_status in TERMINAL_STATUSES
            ):
                self._reject_goal(
                    goal_id=task_goal.goal_id,
                    reason='duplicate_completed_goal_id',
                )
                return
            self._active_goal_id = task_goal.goal_id
            self._last_goal_id = task_goal.goal_id
            self._worker = threading.Thread(
                target=self._run_task,
                args=(task_goal,),
                name=f'room315-task-{task_goal.goal_id}',
                daemon=True,
            )
            worker = self._worker
        self._publish_status(
            status='accepted',
            reason='validated TaskGoal accepted for closed-loop execution',
            goal_id=task_goal.goal_id,
            task_goal=task_goal.to_dict(),
        )
        worker.start()

    def _run_task(self, task_goal: TaskGoal) -> None:
        self._publish_status(
            status='running',
            reason='observing and requesting PlanSys2 plan',
            goal_id=task_goal.goal_id,
        )
        try:
            grounded_goal, runtime_payload_grounding = (
                self._ground_task_goal(task_goal)
            )
            self._publish_status(
                status='running',
                reason=(
                    'TaskGoal validated and grounded when required from '
                    'accepted visual state; '
                    'requesting PlanSys2 plan'
                ),
                goal_id=task_goal.goal_id,
                task_goal=grounded_goal.to_dict(),
            )
            executive = self._create_executive(runtime_payload_grounding)
            result = executive.run(grounded_goal)
            payload = result.to_dict()
            self._publish_status(
                status=result.status,
                reason=result.reason,
                goal_id=task_goal.goal_id,
                result=payload,
            )
        except Exception as exc:  # noqa: BLE001 - fail-closed runtime boundary
            self._publish_supervisor_command({
                'action': 'stop_all',
                'reason': f'task_execution_exception:{exc}',
                'closed_loop_executive': {
                    'mode': 'safe_abort',
                    'goal_id': task_goal.goal_id,
                },
            })
            self._publish_status(
                status='failed',
                reason=f'task_execution_exception:{exc}',
                goal_id=task_goal.goal_id,
            )
        finally:
            with self._lock:
                self._active_goal_id = ''

    def _ground_task_goal(
        self,
        task_goal: TaskGoal,
    ) -> tuple[TaskGoal, dict[str, Any]]:
        """Ground target identity from accepted vision before planning."""

        initial_state = self.state_provider.observe()
        stable = ground_transport_task_goal_stably(
            task_goal,
            initial_state,
            observe_fresh_after=self.state_provider.observe_fresh_after,
            confirmation_frames=self._int(
                'payload_grounding_confirmation_frames'
            ),
            max_observations=self._int(
                'payload_grounding_max_observations'
            ),
        )
        proof = build_runtime_payload_grounding(
            stable.task_goal,
            stable.observed_state,
            payload_confirmation=stable.payload_confirmation,
        )
        if proof:
            confirmation = proof['temporal_confirmation']
            self.get_logger().info(
                'Payload-qualified TaskGoal grounded by fresh visual '
                'consensus: selected=%s filter=%s frames=%d/%d '
                'observations=%d source_state=%s'
                % (
                    proof['selected_shuttle'],
                    proof['payload_filter'],
                    len(confirmation['state_ids']),
                    confirmation['confirmation_frames'],
                    confirmation['observations_examined'],
                    proof['source_state_id'],
                )
            )
        return stable.task_goal, proof

    def _create_executive(
        self,
        runtime_payload_grounding: dict[str, Any],
    ) -> ClosedLoopExecutive:
        planner = PlanSysPlannerBackend(
            domain_path=Path(
                self._string('planner_domain_path')
            ).expanduser(),
            planner_service=self._string('planner_service'),
            timeout_s=self._float('planner_timeout_s'),
        )
        return ClosedLoopExecutive(
            observed_state_provider=self.state_provider,
            planner=planner,
            transport=self.transport,
            runtime_payload_grounding=runtime_payload_grounding,
            config=ClosedLoopExecutiveConfig(
                speed_mps=self._float('speed_mps'),
                max_steps=self._int('max_steps'),
                max_replans=self._int('max_replans'),
                max_unknown_retries=self._int('max_unknown_retries'),
                supervisor_timeout_s=self._float('supervisor_timeout_s'),
                effect_timeout_s=self._float('effect_timeout_s'),
                clearance_effect_timeout_s=self._float(
                    'clearance_effect_timeout_s'
                ),
                route_arrival_timeout_scale=self._float(
                    'route_arrival_timeout_scale'
                ),
                route_arrival_timeout_margin_s=self._float(
                    'route_arrival_timeout_margin_s'
                ),
            ),
        )

    def _publish_supervisor_command(
        self,
        command: dict[str, Any],
    ) -> None:
        message = String()
        message.data = json.dumps(command, sort_keys=True)
        self.command_pub.publish(message)

    def _reject_goal(self, *, goal_id: str, reason: str) -> None:
        self._publish_status(
            status='rejected',
            reason=reason,
            goal_id=goal_id,
        )

    def _publish_status(
        self,
        *,
        status: str,
        reason: str,
        goal_id: str = '',
        task_goal: dict[str, Any] | None = None,
        result: dict[str, Any] | None = None,
    ) -> None:
        payload = {
            'contract': 'room315.task_execution_status.v1',
            'timestamp_s': self.get_clock().now().nanoseconds / 1e9,
            'goal_id': goal_id or self._active_goal_id,
            'status': str(status),
            'reason': str(reason),
            'execution_enabled': self._bool('execution_enabled'),
            'location_source': 'accepted_visual_state',
            'presence_source':
                'controller_ShuttleState_name_and_timestamps_only',
            'controller_position_fields_used_for_localization': False,
        }
        if task_goal is not None:
            payload['task_goal'] = task_goal
        if result is not None:
            payload['result'] = result
        self._last_status = str(status)
        self._last_reason = str(reason)
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(message)
        log_message = (
            f'Task execution status={status} '
            f'goal={payload["goal_id"] or "none"} reason={reason}'
        )
        if status in {'failed', 'aborted'}:
            self.get_logger().error(log_message)
        elif status == 'rejected':
            self.get_logger().warning(log_message)
        else:
            self.get_logger().info(log_message)

    def _publish_diagnostics(self) -> None:
        ready, reason = self.state_provider.ready()
        status = DiagnosticStatus()
        status.name = 'room_315_task_execution'
        status.hardware_id = 'room_315'
        if not self._bool('execution_enabled'):
            status.level = DiagnosticStatus.WARN
            status.message = 'execution disabled'
        elif not ready:
            status.level = DiagnosticStatus.WARN
            status.message = 'execution state not ready'
        elif self._last_status in {'failed', 'aborted'}:
            status.level = DiagnosticStatus.ERROR
            status.message = self._last_reason
        else:
            status.level = DiagnosticStatus.OK
            status.message = self._last_status
        values = {
            'execution_enabled': self._bool('execution_enabled'),
            'state_ready': ready,
            'state_reason': reason,
            'active_goal_id': self._active_goal_id,
            'last_goal_id': self._last_goal_id,
            'last_status': self._last_status,
            'last_reason': self._last_reason,
            'planner_service': self._string('planner_service'),
            'location_source': 'accepted_visual_state',
            'presence_contract':
                'ShuttleState.name+header.stamp+receive_time_only',
        }
        status.values = [
            KeyValue(key=key, value=json.dumps(value, sort_keys=True))
            for key, value in values.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub.publish(array)

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)


def _visual_message_dict(message: VisualStateObservation) -> dict[str, Any]:
    return {
        'timestamp_s': _stamp_s(message.header.stamp),
        'state_id': (
            f'room315-live-visual-'
            f'{message.header.stamp.sec}-{message.header.stamp.nanosec}'
        ),
        'schema_version': message.schema_version,
        'checkpoint_sha256': message.checkpoint_sha256,
        'stage': message.stage,
        'accepted': bool(message.accepted),
        'stabilized': bool(message.stabilized),
        'stale': bool(message.stale),
        'model_ready': bool(message.model_ready),
        'input_ready': bool(message.input_ready),
        'presence_ready': bool(message.presence_ready),
        'state_fusion_ready': bool(message.state_fusion_ready),
        'validation_reasons': list(message.validation_reasons),
        'shuttles': [
            {
                'identity': item.identity,
                'presence_state': item.presence_state,
                'visual_facts_valid': bool(item.visual_facts_valid),
                'side': item.side,
                'block': item.block,
                'bbox_xywh': list(item.bbox_xywh),
                's_m': float(item.s_m),
                's_ratio': float(item.s_ratio),
                'segment_length_m': float(item.segment_length_m),
                'loaded_state': item.loaded_state,
            }
            for item in message.shuttles
        ],
    }


def _stamp_s(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Room315TaskExecutionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node._active_goal_id:
            node._publish_supervisor_command({
                'action': 'stop_all',
                'reason': 'task_execution_node_shutdown',
            })
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
