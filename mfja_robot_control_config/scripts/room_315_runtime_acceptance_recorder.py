#!/usr/bin/env python3
"""Record one guarded Gazebo runtime-acceptance scenario without controlling it."""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
from mfja_rail_interfaces.msg import VisualStateObservation
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_runtime_acceptance_report import REQUIRED_RECORD_FIELDS
from room_315_runtime_acceptance_report import load_json
from room_315_runtime_acceptance_report import validate_scenario_manifest


def observation_dict(message: VisualStateObservation) -> dict[str, Any]:
    return {
        'status': 'observed',
        'timestamp_s': _stamp_s(message.header.stamp),
        'schema_version': message.schema_version,
        'checkpoint_sha256': message.checkpoint_sha256,
        'stage': message.stage,
        'accepted': bool(message.accepted),
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
                'bbox_xywh': [float(value) for value in item.bbox_xywh],
                's_m': float(item.s_m),
                's_ratio': float(item.s_ratio),
                'segment_length_m': float(item.segment_length_m),
                'loaded_state': item.loaded_state,
            }
            for item in message.shuttles
        ],
    }


class Room315RuntimeAcceptanceRecorder(Node):
    def __init__(self) -> None:
        super().__init__('room_315_runtime_acceptance_recorder')
        self.declare_parameter('scenario_manifest_path', '')
        self.declare_parameter('scenario_id', '')
        self.declare_parameter('output_file', '')
        self.declare_parameter('record_duration_s', 60.0)
        self.declare_parameter('minimum_record_duration_s', 3.0)
        self.declare_parameter('minimum_accepted_observations', 3)
        self.declare_parameter('readiness_proof_path', '')
        self.declare_parameter('expected_checkpoint_sha256', '')
        self.declare_parameter('observation_only', True)
        self.declare_parameter(
            'raw_model_prediction_topic',
            '/room_315/visual_state/raw_model_prediction',
        )
        self.declare_parameter('raw_observation_topic', '/room_315/visual_state/raw')
        self.declare_parameter(
            'validation_topic',
            '/room_315/visual_state/validation',
        )
        self.declare_parameter(
            'accepted_observed_state_topic',
            '/room_315/visual_state/observed_state',
        )
        self.declare_parameter('safety_status_topic', '/room_315/vla/status')
        self.declare_parameter(
            'task_execution_status_topic',
            '/room_315/task_goal/status',
        )

        manifest_path = Path(self._string('scenario_manifest_path')).resolve()
        scenario_id = self._string('scenario_id').strip()
        output_text = self._string('output_file').strip()
        if not scenario_id or not output_text:
            raise RuntimeError('scenario_id and output_file are required')
        manifest = load_json(manifest_path)
        rows = validate_scenario_manifest(manifest)
        matches = [row for row in rows if row['scenario_id'] == scenario_id]
        if len(matches) != 1:
            raise RuntimeError(f'unknown acceptance scenario: {scenario_id}')
        self.scenario = matches[0]
        self.expected_identities = set(
            self.scenario['ground_truth']['present_identities']
        )
        self.output_file = Path(output_text).resolve()
        if self.output_file.exists():
            raise RuntimeError(
                f'refusing to overwrite acceptance event: {self.output_file}'
            )
        if not self._bool('observation_only'):
            raise RuntimeError('runtime acceptance recorder is observation-only')
        proof_text = self._string('readiness_proof_path').strip()
        if not proof_text:
            raise RuntimeError('readiness_proof_path is required')
        self.readiness_proof = load_json(Path(proof_text).resolve())
        if (
            self.readiness_proof.get('status') != 'ready'
            or self.readiness_proof.get('phase') != 'runtime'
            or self.readiness_proof.get('scenario_id') != scenario_id
            or not all(self.readiness_proof.get('checks', {}).values())
        ):
            raise RuntimeError('runtime readiness proof is not complete and valid')
        self.expected_checkpoint_sha256 = self._string(
            'expected_checkpoint_sha256'
        ).strip()
        prediction_evidence = (
            self.readiness_proof.get('evidence', {}).get('raw_prediction', {})
        )
        if (
            not self.expected_checkpoint_sha256
            or prediction_evidence.get('checkpoint_sha256')
            != self.expected_checkpoint_sha256
            or int(prediction_evidence.get('output_dimension', -1)) != 200
        ):
            raise RuntimeError('readiness proof checkpoint contract mismatch')
        self.started_monotonic = time.monotonic()
        self.finalized = False
        self.raw_model_prediction: dict[str, Any] = {'status': 'not_observed'}
        self.decoded_observed_state: dict[str, Any] = {'status': 'not_observed'}
        self.presence_provider_result: dict[str, Any] = {'status': 'not_observed'}
        self.fusion_result: dict[str, Any] = {'status': 'not_observed'}
        self.validation_result: dict[str, Any] = {'status': 'not_observed'}
        self.safety_supervisor_decision: dict[str, Any] = {'status': 'not_observed'}
        self.execution_decision: dict[str, Any] = {
            'status': 'observed',
            'allowed': False,
            'reason': 'observation_only_acceptance_contract',
        }
        self.effect_verification: dict[str, Any] = {
            'status': 'observed',
            'result': 'not_applicable_observation_only',
            'actuation_performed': False,
        }
        self.accepted_reobservations: list[dict[str, Any]] = []
        self.policy_violations: list[str] = []
        self.exit_code = 1

        self.create_subscription(
            String,
            self._string('raw_model_prediction_topic'),
            self._on_raw_prediction,
            10,
        )
        self.create_subscription(
            VisualStateObservation,
            self._string('raw_observation_topic'),
            self._on_raw_observation,
            10,
        )
        self.create_subscription(
            VisualStateObservation,
            self._string('validation_topic'),
            self._on_validation,
            10,
        )
        self.create_subscription(
            VisualStateObservation,
            self._string('accepted_observed_state_topic'),
            self._on_accepted,
            10,
        )
        self.create_subscription(
            String,
            self._string('safety_status_topic'),
            self._on_safety,
            10,
        )
        self.create_subscription(
            String,
            self._string('task_execution_status_topic'),
            self._on_execution,
            10,
        )
        self.create_timer(0.25, self._on_timer)

    def _on_raw_prediction(self, message: String) -> None:
        self.raw_model_prediction = _json_message(message)
        payload = self.raw_model_prediction.get('payload') or {}
        if (
            payload.get('checkpoint_sha256') != self.expected_checkpoint_sha256
            or int(payload.get('output_dimension', -1)) != 200
            or len(payload.get('denormalized_output') or []) != 200
        ):
            self.policy_violations.append('raw_prediction_contract_mismatch')

    def _on_raw_observation(self, message: VisualStateObservation) -> None:
        value = observation_dict(message)
        self.decoded_observed_state = value
        self.presence_provider_result = {
            'status': 'observed',
            'ready': value['presence_ready'],
            'identity_states': {
                item['identity']: item['presence_state']
                for item in value['shuttles']
            },
            'source': 'controller_ShuttleState_name_and_timestamps_only',
        }

    def _on_validation(self, message: VisualStateObservation) -> None:
        value = observation_dict(message)
        self.validation_result = {
            'status': 'observed',
            'accepted': value['accepted'],
            'reasons': value['validation_reasons'],
            'stage': value['stage'],
        }
        self.fusion_result = {
            'status': 'observed',
            'ready': value['state_fusion_ready'],
            'accepted': value['accepted'],
            'reasons': value['validation_reasons'],
        }

    def _on_accepted(self, message: VisualStateObservation) -> None:
        value = observation_dict(message)
        self.fusion_result = {
            'status': 'observed',
            'ready': value['state_fusion_ready'],
            'accepted': value['accepted'],
            'observed_state': value,
        }
        self.accepted_reobservations.append({
            'timestamp_s': value['timestamp_s'],
            'accepted': value['accepted'],
            'shuttles': value['shuttles'],
        })
        self.accepted_reobservations = self.accepted_reobservations[-20:]

    def _on_safety(self, message: String) -> None:
        payload = _json_message(message)
        value = payload.get('payload') or {}
        self.safety_supervisor_decision = {
            'status': 'observed',
            'payload': payload,
            'execution_safe': (
                not bool(value.get('emergency_stop', True))
                if payload.get('status') == 'observed'
                else False
            ),
        }

    def _on_execution(self, message: String) -> None:
        payload = _json_message(message)
        value = payload.get('payload') or {}
        status = str(value.get('status') or '')
        allowed = status not in {'rejected', 'failed', 'aborted', 'error', ''}
        received = {
            'status': 'observed',
            'allowed': allowed,
            'runtime_status': status,
            'reason': value.get('reason'),
            'payload': payload,
        }
        if allowed:
            self.policy_violations.append(
                f'observation_only_execution_status_received:{status}'
            )
        self.execution_decision = received
        if status in {'completed', 'failed', 'aborted', 'rejected', 'error'}:
            self.effect_verification = {
                'status': 'observed',
                'terminal_execution_status': status,
                'result': value.get('result'),
                'accepted_reobservations': list(self.accepted_reobservations),
            }

    def _on_timer(self) -> None:
        elapsed = time.monotonic() - self.started_monotonic
        if (
            elapsed >= self._float('minimum_record_duration_s')
            and self._evidence_complete()
        ):
            self.finalize(success=True)
            rclpy.shutdown()
        elif elapsed >= self._float('record_duration_s'):
            self.finalize(success=False)
            rclpy.shutdown()

    def _evidence_failures(self) -> list[str]:
        failures = list(dict.fromkeys(self.policy_violations))
        raw = self.raw_model_prediction.get('payload') or {}
        if self.raw_model_prediction.get('status') != 'observed':
            failures.append('raw_model_prediction_not_observed')
        elif (
            raw.get('checkpoint_sha256') != self.expected_checkpoint_sha256
            or int(raw.get('output_dimension', -1)) != 200
            or len(raw.get('denormalized_output') or []) != 200
        ):
            failures.append('raw_model_prediction_incompatible')
        if self.decoded_observed_state.get('status') != 'observed':
            failures.append('decoded_observed_state_not_observed')
        presence = self.presence_provider_result
        identity_states = presence.get('identity_states') or {}
        present = {
            identity for identity, state in identity_states.items()
            if state == 'present'
        }
        unknown = {
            identity for identity, state in identity_states.items()
            if state == 'unknown'
        }
        if (
            presence.get('status') != 'observed'
            or not presence.get('ready')
            or present != self.expected_identities
            or unknown
        ):
            failures.append('presence_inventory_not_ready_or_incorrect')
        if (
            self.validation_result.get('status') != 'observed'
            or not self.validation_result.get('accepted')
        ):
            failures.append('visual_validation_not_accepted')
        if (
            self.fusion_result.get('status') != 'observed'
            or not self.fusion_result.get('ready')
            or not self.fusion_result.get('accepted')
        ):
            failures.append('state_fusion_not_accepted')
        if self.safety_supervisor_decision.get('status') != 'observed':
            failures.append('safety_supervisor_not_observed')
        minimum = self._int('minimum_accepted_observations')
        if len(self.accepted_reobservations) < minimum:
            failures.append(
                f'insufficient_accepted_reobservations:'
                f'{len(self.accepted_reobservations)}<{minimum}'
            )
        if self.execution_decision.get('allowed') is not False:
            failures.append('observation_only_execution_not_rejected')
        if self.effect_verification.get('actuation_performed') is not False:
            failures.append('observation_only_effect_contract_invalid')
        return list(dict.fromkeys(failures))

    def _evidence_complete(self) -> bool:
        return not self._evidence_failures()

    def finalize(self, *, success: bool = False) -> None:
        if self.finalized:
            return
        self.finalized = True
        failures = self._evidence_failures()
        success = bool(success and not failures)
        self.exit_code = 0 if success else 1
        record = {
            'schema_version': 'room315.runtime_acceptance_event.v1',
            'record_status': 'complete' if success else 'failed',
            'failure_reasons': failures,
            'scenario_id': self.scenario['scenario_id'],
            'coverage': list(self.scenario.get('coverage') or []),
            'ground_truth': self.scenario['ground_truth'],
            'raw_model_prediction': self.raw_model_prediction,
            'decoded_observed_state': self.decoded_observed_state,
            'presence_provider_result': self.presence_provider_result,
            'fusion_result': self.fusion_result,
            'validation_result': self.validation_result,
            'safety_supervisor_decision': self.safety_supervisor_decision,
            'execution_decision': self.execution_decision,
            'reobservation_and_effect_verification': self.effect_verification,
            'readiness_proof': self.readiness_proof,
            'observation_only': True,
            'record_duration_s': time.monotonic() - self.started_monotonic,
        }
        missing = [field for field in REQUIRED_RECORD_FIELDS if field not in record]
        if missing:
            raise RuntimeError(f'internal acceptance record fields missing: {missing}')
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output_file.with_name(
            f'.{self.output_file.name}.tmp-{os.getpid()}'
        )
        temporary.write_text(
            json.dumps(record, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        os.replace(temporary, self.output_file)
        logger = self.get_logger().info if success else self.get_logger().error
        logger(
            f'Acceptance event status={record["record_status"]} '
            f'written={self.output_file} failures={failures}'
        )

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)


def _json_message(message: String) -> dict[str, Any]:
    try:
        value = json.loads(message.data or '{}')
    except json.JSONDecodeError as exc:
        return {'status': 'invalid_json', 'error': str(exc), 'raw': message.data}
    if not isinstance(value, dict):
        return {'status': 'invalid_type', 'value': value}
    return {'status': 'observed', 'payload': value}


def _stamp_s(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = Room315RuntimeAcceptanceRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finalize(success=False)
        exit_code = node.exit_code
        node.destroy_node()
        rclpy.try_shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
