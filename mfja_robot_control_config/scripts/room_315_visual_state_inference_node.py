#!/usr/bin/env python3
"""ROS 2 runtime node for the approved Room 315 visual-state checkpoint."""

from __future__ import annotations

import json
import math
import re
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import message_filters
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from mfja_rail_interfaces.msg import ShuttleState
from mfja_rail_interfaces.msg import VisualShuttleState
from mfja_rail_interfaces.msg import VisualStateObservation
from plansys2_msgs.msg import Node as PlanSysNode
from plansys2_msgs.msg import Param as PlanSysParam
from plansys2_msgs.srv import AffectNode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from room_315_presence_provider import PresenceSnapshot
from room_315_presence_provider import ShuttleStatePresenceProvider
from room_315_visual_runtime import ArtifactHashes
from room_315_visual_runtime import ArtifactPaths
from room_315_visual_runtime import DecodedVisualPrediction
from room_315_visual_runtime import FIXED_IDENTITY_ORDER
from room_315_visual_runtime import InferenceTimings
from room_315_visual_runtime import MODEL_SCHEMA
from room_315_visual_runtime import Room315VisualModelRuntime
from room_315_visual_runtime import VisualRuntimeError
from room_315_visual_runtime import decode_active_slots
from room_315_visual_runtime import verify_artifacts
from room_315_visual_runtime_fusion import DeterministicPlanSys2FactGate
from room_315_visual_runtime_fusion import PlanSysPredicateUpdate
from room_315_visual_runtime_fusion import StateFusionResult
from room_315_visual_runtime_fusion import fuse_validated_visual_state
from room_315_visual_runtime_validation import DeterministicTemporalStabilizer
from room_315_visual_runtime_validation import ValidationConfig
from room_315_visual_runtime_validation import ValidationResult
from room_315_visual_runtime_validation import validate_prediction


class PlanSys2PredicateClient:
    """Problem-expert predicate updater; it cannot plan or execute actions."""

    def __init__(
        self,
        node: Node,
        *,
        add_service: str,
        remove_service: str,
    ) -> None:
        self._node = node
        self._add = node.create_client(AffectNode, add_service)
        self._remove = node.create_client(AffectNode, remove_service)
        self._pending: list[Any] = []
        self.last_error = ''

    def ready(self) -> bool:
        return bool(
            self._add.service_is_ready()
            and self._remove.service_is_ready()
        )

    def apply(self, update: PlanSysPredicateUpdate) -> bool:
        if not update.accepted:
            return False
        if not self.ready():
            self.last_error = 'PlanSys2 problem-expert predicate services are not ready'
            return False
        try:
            for predicate in update.remove_predicates:
                self._call(self._remove, predicate)
            for predicate in update.add_predicates:
                self._call(self._add, predicate)
        except ValueError as exc:
            self.last_error = str(exc)
            return False
        self.last_error = ''
        return True

    def _call(self, client: Any, predicate: str) -> None:
        request = AffectNode.Request()
        request.node = _predicate_node(predicate)
        future = client.call_async(request)
        self._pending.append(future)
        future.add_done_callback(self._on_result)

    def _on_result(self, future: Any) -> None:
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001 - diagnostics boundary
            self.last_error = f'PlanSys2 predicate service failed: {exc}'
            return
        if result is None or not bool(result.success):
            self.last_error = (
                'PlanSys2 predicate update rejected: '
                f'{getattr(result, "error_info", "no response")}'
            )


class Room315VisualStateInferenceNode(Node):
    """Synchronize paired RGB frames, infer, validate, fuse, and diagnose."""

    def __init__(self) -> None:
        super().__init__('room_315_visual_state_inference_node')
        self._declare_parameters()
        self.bridge = CvBridge()
        self.presence = ShuttleStatePresenceProvider(
            timeout_s=self._float('presence_state_timeout_s'),
            warmup_s=self._float('presence_warmup_s'),
        )
        self.validator_config = ValidationConfig(
            stale_image_timeout_s=self._float('stale_image_timeout_s'),
            maximum_timestamp_difference_s=self._float(
                'maximum_timestamp_difference_s'
            ),
            s_ratio_tolerance=self._float('s_ratio_tolerance'),
            s_m_tolerance_m=self._float('s_m_tolerance_m'),
            position_consistency_tolerance_m=self._float(
                'position_consistency_tolerance_m'
            ),
            reconcile_position_consistency=self._bool(
                'reconcile_position_consistency'
            ),
            max_position_reconciliation_error_m=self._float(
                'max_position_reconciliation_error_m'
            ),
        )
        self.stabilizer = DeterministicTemporalStabilizer(
            enabled=self._bool('temporal_filter_enabled'),
            majority_window=self._int('temporal_majority_window'),
            ema_alpha=self._float('temporal_ema_alpha'),
        )
        self.plan_gate = DeterministicPlanSys2FactGate()
        self.plan_client = PlanSys2PredicateClient(
            self,
            add_service=self._string('plansys2_add_predicate_service'),
            remove_service=self._string('plansys2_remove_predicate_service'),
        )

        self.raw_pub = self.create_publisher(
            VisualStateObservation,
            self._string('raw_observation_topic'),
            10,
        )
        self.validation_pub = self.create_publisher(
            VisualStateObservation,
            self._string('validation_topic'),
            10,
        )
        self.observed_state_pub = self.create_publisher(
            VisualStateObservation,
            self._string('accepted_observed_state_topic'),
            10,
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            self._string('diagnostics_topic'),
            10,
        )

        self.create_subscription(
            ShuttleState,
            self._string('left_presence_topic'),
            lambda message: self._on_presence('left', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ShuttleState,
            self._string('right_presence_topic'),
            lambda message: self._on_presence('right', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            self._string('safety_status_topic'),
            self._on_safety_status,
            10,
        )

        left_sub = message_filters.Subscriber(
            self,
            Image,
            self._string('left_image_topic'),
            qos_profile=qos_profile_sensor_data,
        )
        right_sub = message_filters.Subscriber(
            self,
            Image,
            self._string('right_image_topic'),
            qos_profile=qos_profile_sensor_data,
        )
        self.image_subscribers = (left_sub, right_sub)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub],
            queue_size=self._int('synchronization_queue_size'),
            slop=self._float('maximum_timestamp_difference_s'),
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self._on_image_pair)

        self.model_runtime: Room315VisualModelRuntime | None = None
        self.artifacts = None
        self.artifact_error = ''
        self.checkpoint_sha256 = self._string('expected_checkpoint_sha256')
        self.last_inference_stamp_s: float | None = None
        self.last_accepted_stamp_s: float | None = None
        self.last_pair_receive_s: float | None = None
        self.last_validation_reasons: tuple[str, ...] = ()
        self.last_timings = InferenceTimings(0.0, 0.0, 0.0, 0.0)
        self.validation_latency_ms = 0.0
        self.accepted_frames = 0
        self.rejected_frames = 0
        self.stale_frames = 0
        self.rejection_counts: dict[str, int] = {}
        self.last_safety_status_receive_s: float | None = None
        self.safety_emergency_stop = True
        self.safety_status_valid = False
        self._load_artifacts_and_model()

        period = max(0.1, self._float('diagnostic_period_s'))
        self.create_timer(period, self._publish_diagnostics)
        self.get_logger().info(
            'Room 315 visual-state runtime initialized fail-closed; '
            'no planner, executor, or command publisher is owned by this node.'
        )

    def _declare_parameters(self) -> None:
        parameters = {
            'left_image_topic': '/room_315/vla/left_rail_rgbd/image',
            'right_image_topic': '/room_315/vla/right_rail_rgbd/image',
            'left_presence_topic': '/room_315/rails/left/shuttles/state',
            'right_presence_topic': '/room_315/rails/right/shuttles/state',
            'checkpoint_path': '',
            'sidecar_directory': '',
            'expected_checkpoint_sha256': '',
            'expected_target_stats_sha256': '',
            'expected_vectorizer_sha256': '',
            'expected_training_config_sha256': '',
            'expected_run_metadata_sha256': '',
            'device': 'auto',
            'synchronization_queue_size': 10,
            'maximum_timestamp_difference_s': 0.1,
            'stale_image_timeout_s': 1.0,
            'inference_frequency_limit_hz': 5.0,
            'presence_state_timeout_s': 1.0,
            'presence_warmup_s': 0.5,
            's_ratio_tolerance': 0.02,
            's_m_tolerance_m': 0.02,
            'position_consistency_tolerance_m': 0.08,
            'reconcile_position_consistency': False,
            'max_position_reconciliation_error_m': 0.40,
            'temporal_filter_enabled': False,
            'temporal_majority_window': 3,
            'temporal_ema_alpha': 0.5,
            'dry_run_state_fusion': True,
            'plansys2_update_enabled': False,
            'plansys2_add_predicate_service': '/problem_expert/add_predicate',
            'plansys2_remove_predicate_service': '/problem_expert/remove_predicate',
            'safety_status_topic': '/room_315/vla/status',
            'safety_status_timeout_s': 1.5,
            'raw_observation_topic': '/room_315/visual_state/raw',
            'validation_topic': '/room_315/visual_state/validation',
            'accepted_observed_state_topic': '/room_315/visual_state/observed_state',
            'diagnostics_topic': '/diagnostics',
            'diagnostic_period_s': 1.0,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _load_artifacts_and_model(self) -> None:
        try:
            checkpoint = Path(self._string('checkpoint_path')).expanduser()
            sidecars = Path(self._string('sidecar_directory')).expanduser()
            if not str(checkpoint) or not self._string('checkpoint_path'):
                raise VisualRuntimeError('checkpoint_path parameter is empty')
            if not self._string('sidecar_directory'):
                raise VisualRuntimeError('sidecar_directory parameter is empty')
            self.artifacts = verify_artifacts(
                ArtifactPaths(
                    checkpoint=checkpoint.resolve(),
                    sidecar_directory=sidecars.resolve(),
                ),
                ArtifactHashes(
                    checkpoint=self._string('expected_checkpoint_sha256'),
                    target_stats=self._string('expected_target_stats_sha256'),
                    vectorizer=self._string('expected_vectorizer_sha256'),
                    training_config=self._string(
                        'expected_training_config_sha256'
                    ),
                    run_metadata=self._string('expected_run_metadata_sha256'),
                ),
            )
            runtime = Room315VisualModelRuntime(
                self.artifacts,
                device=self._string('device'),
            )
            runtime.load()
            self.model_runtime = runtime
            self.artifact_error = ''
            self.get_logger().info(
                f'Approved visual checkpoint loaded strictly on {runtime.device}; '
                f'load_ms={runtime.model_load_duration_ms:.3f}'
            )
        except Exception as exc:  # noqa: BLE001 - startup must remain diagnosable
            self.model_runtime = None
            self.artifact_error = str(exc)
            self.get_logger().error(
                f'Visual runtime not ready; artifact/model load failed: {exc}'
            )

    def _on_presence(self, side: str, message: ShuttleState) -> None:
        now_s = self._now_s()
        try:
            # Deliberately do not read mode/current_segment/s/x/y/z/yaw/speed.
            self.presence.observe(
                topic_side=side,
                entity_name=message.name,
                source_stamp_s=_stamp_s(message.header.stamp),
                receive_time_s=now_s,
            )
        except Exception as exc:  # noqa: BLE001 - fail closed in provider
            self.get_logger().error(f'Presence registry rejected state message: {exc}')

    def _on_safety_status(self, message: String) -> None:
        now_s = self._now_s()
        try:
            payload = json.loads(message.data or '{}')
        except json.JSONDecodeError:
            self.safety_status_valid = False
            return
        if not isinstance(payload, dict):
            self.safety_status_valid = False
            return
        self.last_safety_status_receive_s = now_s
        self.safety_emergency_stop = bool(payload.get('emergency_stop', True))
        self.safety_status_valid = True

    def _on_image_pair(self, left_message: Image, right_message: Image) -> None:
        cycle_started = time.perf_counter()
        now_s = self._now_s()
        self.last_pair_receive_s = now_s
        left_stamp = _stamp_s(left_message.header.stamp)
        right_stamp = _stamp_s(right_message.header.stamp)
        presence = self.presence.snapshot(now_s=now_s)
        early_reasons = self._input_rejection_reasons(
            now_s,
            left_stamp,
            right_stamp,
            presence,
        )
        if early_reasons:
            self._reject_without_prediction(
                left_message,
                right_message,
                presence,
                early_reasons,
                cycle_started,
            )
            return

        frequency = self._float('inference_frequency_limit_hz')
        if (
            frequency > 0.0
            and self.last_inference_stamp_s is not None
            and now_s - self.last_inference_stamp_s < 1.0 / frequency
        ):
            return
        self.last_inference_stamp_s = now_s

        try:
            left_rgb = np.asarray(
                self.bridge.imgmsg_to_cv2(left_message, desired_encoding='rgb8')
            )
            right_rgb = np.asarray(
                self.bridge.imgmsg_to_cv2(right_message, desired_encoding='rgb8')
            )
            if self.model_runtime is None or self.artifacts is None:
                raise VisualRuntimeError('model runtime is not ready')
            raw, timings = self.model_runtime.infer(left_rgb, right_rgb)
            self.last_timings = timings
            prediction = decode_active_slots(
                raw,
                vectorizer=self.artifacts.vectorizer,
                presence=presence,
                timestamp_s=now_s,
                left_image_stamp_s=left_stamp,
                right_image_stamp_s=right_stamp,
                left_image_size=(left_rgb.shape[1], left_rgb.shape[0]),
                right_image_size=(right_rgb.shape[1], right_rgb.shape[0]),
            )
            self.raw_pub.publish(self._observation_message(
                stage='raw',
                prediction=prediction,
                presence=presence,
                validation=None,
                stabilized=False,
                fusion=None,
                left_message=left_message,
                right_message=right_message,
            ))
        except Exception as exc:  # noqa: BLE001 - inference boundary
            self._reject_without_prediction(
                left_message,
                right_message,
                presence,
                (f'inference_failed:{exc}',),
                cycle_started,
            )
            return

        validation_started = time.perf_counter()
        result = validate_prediction(
            prediction,
            presence,
            now_s=now_s,
            config=self.validator_config,
            artifact_healthy=self.model_runtime is not None,
            input_healthy=True,
        )
        self.validation_latency_ms = (
            time.perf_counter() - validation_started
        ) * 1000.0
        result, stabilized = self.stabilizer.apply(result)
        fusion = fuse_validated_visual_state(
            result,
            presence,
            checkpoint_sha256=self.checkpoint_sha256,
            schema_version=MODEL_SCHEMA,
            stale_after_s=self._float('stale_image_timeout_s'),
            state_id=f'room315-visual-{int(now_s * 1_000_000)}',
        )
        accepted = bool(result.accepted and fusion.ready)
        if accepted:
            self.accepted_frames += 1
            self.last_accepted_stamp_s = now_s
            self.last_validation_reasons = ()
        else:
            self._record_rejection(result.reasons or fusion.reasons)

        validation_message = self._observation_message(
            stage='validated' if accepted else 'rejected',
            prediction=result.prediction,
            presence=presence,
            validation=result,
            stabilized=stabilized,
            fusion=fusion,
            left_message=left_message,
            right_message=right_message,
        )
        validation_message.cycle_latency_ms = (
            time.perf_counter() - cycle_started
        ) * 1000.0
        self.validation_pub.publish(validation_message)
        if accepted:
            observed = self._observation_message(
                stage='fused_observed_state',
                prediction=result.prediction,
                presence=presence,
                validation=result,
                stabilized=stabilized,
                fusion=fusion,
                left_message=left_message,
                right_message=right_message,
            )
            observed.cycle_latency_ms = validation_message.cycle_latency_ms
            self.observed_state_pub.publish(observed)
            self._maybe_update_plansys2(fusion)

    def _input_rejection_reasons(
        self,
        now_s: float,
        left_stamp: float,
        right_stamp: float,
        presence: PresenceSnapshot,
    ) -> tuple[str, ...]:
        reasons: list[str] = []
        if self.model_runtime is None:
            reasons.append('model_runtime_not_ready')
        if not presence.ready:
            reasons.extend(presence.reasons or ('presence_registry_not_ready',))
        stale_timeout = self._float('stale_image_timeout_s')
        for side, stamp in (('left', left_stamp), ('right', right_stamp)):
            age = now_s - stamp
            if age < 0.0:
                reasons.append(f'{side}_image_timestamp_in_future')
            elif age > stale_timeout:
                reasons.append(f'{side}_image_stale')
                self.stale_frames += 1
        if abs(left_stamp - right_stamp) > self._float(
            'maximum_timestamp_difference_s'
        ):
            reasons.append('paired_image_timestamp_skew_exceeded')
        return tuple(dict.fromkeys(reasons))

    def _reject_without_prediction(
        self,
        left_message: Image,
        right_message: Image,
        presence: PresenceSnapshot,
        reasons: tuple[str, ...],
        cycle_started: float,
    ) -> None:
        self.stabilizer.reset()
        self.plan_gate.reset()
        self._record_rejection(reasons)
        validation = ValidationResult(
            accepted=False,
            reasons=reasons,
            clamped_fields=(),
            topology_consistent=False,
            timestamp_consistent=False,
            artifact_healthy=self.model_runtime is not None,
            input_healthy=False,
            presence_ready=presence.ready,
            prediction=None,
        )
        message = self._observation_message(
            stage='rejected',
            prediction=None,
            presence=presence,
            validation=validation,
            stabilized=False,
            fusion=None,
            left_message=left_message,
            right_message=right_message,
        )
        message.cycle_latency_ms = (
            time.perf_counter() - cycle_started
        ) * 1000.0
        self.validation_pub.publish(message)

    def _record_rejection(self, reasons: tuple[str, ...]) -> None:
        self.rejected_frames += 1
        self.last_validation_reasons = tuple(reasons)
        for reason in reasons:
            self.rejection_counts[reason] = self.rejection_counts.get(reason, 0) + 1

    def _maybe_update_plansys2(self, fusion: StateFusionResult) -> None:
        enabled = self._bool('plansys2_update_enabled')
        dry_run = self._bool('dry_run_state_fusion')
        safety_ready = self._safety_ready()
        update = self.plan_gate.build_update(
            fusion,
            model_ready=self.model_runtime is not None,
            input_ready=True,
            safety_ready=safety_ready,
            enabled=enabled and not dry_run,
        )
        if update.accepted and not self.plan_client.apply(update):
            self.get_logger().error(
                f'PlanSys2 predicate update failed closed: {self.plan_client.last_error}'
            )

    def _safety_ready(self) -> bool:
        if (
            not self.safety_status_valid
            or self.last_safety_status_receive_s is None
        ):
            return False
        if self._now_s() - self.last_safety_status_receive_s > self._float(
            'safety_status_timeout_s'
        ):
            return False
        return not self.safety_emergency_stop

    def _observation_message(
        self,
        *,
        stage: str,
        prediction: DecodedVisualPrediction | None,
        presence: PresenceSnapshot,
        validation: ValidationResult | None,
        stabilized: bool,
        fusion: StateFusionResult | None,
        left_message: Image,
        right_message: Image,
    ) -> VisualStateObservation:
        message = VisualStateObservation()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'room_315'
        message.left_image_stamp = left_message.header.stamp
        message.right_image_stamp = right_message.header.stamp
        message.schema_version = MODEL_SCHEMA
        message.checkpoint_sha256 = self.checkpoint_sha256
        message.stage = stage
        message.accepted = bool(
            validation is not None
            and validation.accepted
            and (fusion is None or fusion.ready)
        )
        message.stabilized = bool(stabilized)
        message.stale = any(
            'stale' in reason
            for reason in (validation.reasons if validation else ())
        )
        message.model_ready = self.model_runtime is not None
        message.input_ready = validation.input_healthy if validation else True
        message.presence_ready = presence.ready
        message.state_fusion_ready = bool(fusion and fusion.ready)
        reasons = list(validation.reasons if validation else ())
        if fusion and not fusion.ready:
            reasons.extend(fusion.reasons)
        message.validation_reasons = list(dict.fromkeys(reasons))
        message.clamped_fields = list(
            validation.clamped_fields if validation else ()
        )
        message.preprocessing_latency_ms = self.last_timings.preprocessing_ms
        message.inference_latency_ms = self.last_timings.inference_ms
        message.validation_latency_ms = self.validation_latency_ms
        message.cycle_latency_ms = self.last_timings.complete_cycle_ms
        message.accepted_frame_count = self.accepted_frames
        message.rejected_frame_count = self.rejected_frames
        message.stale_frame_count = self.stale_frames
        message.shuttles = _shuttle_messages(prediction, presence)
        return message

    def _publish_diagnostics(self) -> None:
        now_s = self._now_s()
        presence = self.presence.snapshot(now_s=now_s)
        model_ready = self.model_runtime is not None
        input_ready = (
            self.last_pair_receive_s is not None
            and now_s - self.last_pair_receive_s
            <= self._float('stale_image_timeout_s')
        )
        level = DiagnosticStatus.OK
        summary = 'visual runtime ready'
        if not model_ready:
            level = DiagnosticStatus.ERROR
            summary = 'model/artifact runtime not ready'
        elif not presence.ready or not input_ready:
            level = DiagnosticStatus.WARN
            summary = 'visual runtime waiting for fresh inputs'
        status = DiagnosticStatus()
        status.level = level
        status.name = 'room_315_visual_state_runtime'
        status.hardware_id = self.checkpoint_sha256[:12] or 'unverified'
        status.message = summary
        values = {
            'model_ready': model_ready,
            'input_ready': input_ready,
            'presence_ready': presence.ready,
            'state_fusion_ready': (
                model_ready and input_ready and presence.ready
            ),
            'artifact_error': self.artifact_error,
            'device': (
                self.model_runtime.device
                if self.model_runtime is not None
                else self._string('device')
            ),
            'checkpoint_sha256': self.checkpoint_sha256,
            'schema_version': MODEL_SCHEMA,
            'last_inference_timestamp_s': self.last_inference_stamp_s,
            'last_accepted_observation_timestamp_s': self.last_accepted_stamp_s,
            'inference_latency_ms': self.last_timings.inference_ms,
            'complete_cycle_latency_ms': self.last_timings.complete_cycle_ms,
            'accepted_frames': self.accepted_frames,
            'rejected_frames': self.rejected_frames,
            'stale_frames': self.stale_frames,
            'last_validation_reasons': list(self.last_validation_reasons),
            'rejection_counts': self.rejection_counts,
            'presence_reasons': list(presence.reasons),
            'safety_ready': self._safety_ready(),
            'plansys2_update_enabled': self._bool('plansys2_update_enabled'),
            'dry_run_state_fusion': self._bool('dry_run_state_fusion'),
            'plansys2_last_error': self.plan_client.last_error,
        }
        status.values = [
            KeyValue(key=key, value=json.dumps(value, sort_keys=True))
            for key, value in values.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub.publish(array)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)


def _shuttle_messages(
    prediction: DecodedVisualPrediction | None,
    presence: PresenceSnapshot,
) -> list[VisualShuttleState]:
    predicted = {
        shuttle.identity: shuttle
        for shuttle in (prediction.shuttles if prediction else ())
    }
    presence_by_id = presence.by_identity()
    messages: list[VisualShuttleState] = []
    for identity in FIXED_IDENTITY_ORDER:
        item = VisualShuttleState()
        item.identity = identity
        entry = presence_by_id.get(identity)
        item.presence_state = entry.state if entry else 'unknown'
        shuttle = predicted.get(identity)
        item.visual_facts_valid = shuttle is not None
        if shuttle is not None:
            item.side = shuttle.side
            item.block = shuttle.block
            item.bbox_xywh = [float(value) for value in shuttle.bbox_xywh]
            item.s_m = float(shuttle.s_m)
            item.s_ratio = float(shuttle.s_ratio)
            item.segment_length_m = float(shuttle.segment_length_m)
            item.loaded_state = shuttle.loaded_state
        messages.append(item)
    return messages


def _stamp_s(stamp: Any) -> float:
    result = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
    if not math.isfinite(result) or result < 0.0:
        raise ValueError('ROS timestamp must be finite and non-negative')
    return result


def _predicate_node(predicate: str) -> PlanSysNode:
    match = re.fullmatch(r'\(\s*([a-zA-Z0-9_-]+)((?:\s+[a-zA-Z0-9_-]+)*)\s*\)', predicate)
    if match is None:
        raise ValueError(f'invalid deterministic predicate: {predicate!r}')
    node = PlanSysNode()
    node.node_type = PlanSysNode.PREDICATE
    node.name = match.group(1)
    node.parameters = []
    for token in match.group(2).split():
        parameter = PlanSysParam()
        parameter.name = token
        node.parameters.append(parameter)
    return node


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Room315VisualStateInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
