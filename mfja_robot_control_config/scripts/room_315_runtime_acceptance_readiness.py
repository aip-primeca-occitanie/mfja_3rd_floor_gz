#!/usr/bin/env python3
"""Fail-closed readiness gates for Room 315 Gazebo runtime acceptance."""

from __future__ import annotations

import json
import math
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from mfja_rail_interfaces.msg import ShuttleState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.srv import DeleteEntity
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.srv import SpawnEntity
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import String


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_rail_defaults import public_rail_segment_lengths
from room_315_presence_provider import ShuttleStatePresenceProvider
from room_315_runtime_acceptance_report import load_json
from room_315_runtime_acceptance_report import validate_scenario_manifest
from room_315_visual_fleet import AUTHORITATIVE_VISUAL_FLEET


PHASES = ('world', 'scene', 'camera', 'runtime')
DEFAULT_FRESHNESS_S = 2.0
V3_RAW_PREDICTION_SCHEMA = 'room315.raw_model_prediction.v1'
V4_RAW_PREDICTION_SCHEMA = 'room315.visual_runtime_v4.diagnostic.v1'
SUPPORTED_RAW_PREDICTION_SCHEMAS = (
    V3_RAW_PREDICTION_SCHEMA,
    V4_RAW_PREDICTION_SCHEMA,
)


@dataclass(frozen=True)
class ExpectedShuttle:
    identity: str
    entity_name: str
    side: str
    segment: str
    s_ratio: float
    loaded_state: str


@dataclass(frozen=True)
class AcceptanceExpectation:
    scenario_id: str
    shuttles: tuple[ExpectedShuttle, ...]

    @property
    def identities(self) -> tuple[str, ...]:
        return tuple(shuttle.identity for shuttle in self.shuttles)

    def identities_for_side(self, side: str) -> tuple[str, ...]:
        return tuple(
            shuttle.identity for shuttle in self.shuttles
            if shuttle.side == side
        )


def scenario_expectation(
    manifest: dict[str, Any],
    scenario_id: str,
) -> AcceptanceExpectation:
    rows = validate_scenario_manifest(manifest)
    matches = [row for row in rows if row['scenario_id'] == scenario_id]
    if len(matches) != 1:
        raise ValueError(f'unknown or duplicate acceptance scenario: {scenario_id}')
    row = matches[0]
    truth = row['ground_truth']
    truth_shuttles = truth.get('shuttles')
    if not isinstance(truth_shuttles, list) or not truth_shuttles:
        raise ValueError(f'{scenario_id}: non-empty ground-truth shuttles are required')
    world_entities = AUTHORITATIVE_VISUAL_FLEET['world_entities']
    shuttles: list[ExpectedShuttle] = []
    seen: set[str] = set()
    for value in truth_shuttles:
        identity = str(value.get('identity') or '').strip().upper()
        side = str(value.get('side') or '').strip().lower()
        segment = str(value.get('segment') or '').strip().upper()
        loaded_state = str(value.get('loaded_state') or '').strip().lower()
        try:
            ratio = float(value['s_ratio'])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'{scenario_id}:{identity}: invalid s_ratio') from exc
        if identity in seen or identity not in world_entities:
            raise ValueError(f'{scenario_id}: invalid/duplicate identity {identity!r}')
        if side not in {'left', 'right'}:
            raise ValueError(f'{scenario_id}:{identity}: invalid side')
        if side != ('left' if identity.startswith('L') else 'right'):
            raise ValueError(f'{scenario_id}:{identity}: identity/side conflict')
        if loaded_state not in {'loaded', 'empty'}:
            raise ValueError(f'{scenario_id}:{identity}: invalid payload state')
        if not math.isfinite(ratio) or not 0.0 <= ratio <= 1.0:
            raise ValueError(f'{scenario_id}:{identity}: s_ratio outside [0,1]')
        if segment not in public_rail_segment_lengths(side):
            raise ValueError(f'{scenario_id}:{identity}: unknown public segment {segment}')
        seen.add(identity)
        shuttles.append(ExpectedShuttle(
            identity=identity,
            entity_name=str(world_entities[identity]),
            side=side,
            segment=segment,
            s_ratio=ratio,
            loaded_state=loaded_state,
        ))
    present = tuple(str(item) for item in truth.get('present_identities') or ())
    if set(present) != seen:
        raise ValueError(f'{scenario_id}: present identities disagree with shuttles')
    if not any(item.side == 'left' for item in shuttles):
        raise ValueError(f'{scenario_id}: a left source is required')
    if not any(item.side == 'right' for item in shuttles):
        raise ValueError(f'{scenario_id}: a right source is required')
    return AcceptanceExpectation(scenario_id=scenario_id, shuttles=tuple(shuttles))


def scenario_launch_arguments(
    scenario: dict[str, Any],
) -> dict[str, str]:
    setup = scenario['gazebo_setup']
    left = [str(value) for value in setup['left_active_identities']]
    right = [str(value) for value in setup['right_active_identities']]
    if not left or not right:
        raise ValueError('acceptance requires initialized left and right presence sources')
    return {
        'identity_selection_mode': 'explicit',
        'left_active_identities': ','.join(left),
        'right_active_identities': ','.join(right),
        'left_shuttle_count': str(len(left)),
        'right_shuttle_count': str(len(right)),
        'left_start_positions': ','.join(setup['left_start_positions']),
        'right_start_positions': ','.join(setup['right_start_positions']),
        'left_loaded_shuttles': ','.join(setup['left_loaded_identities']),
        'right_loaded_shuttles': ','.join(setup['right_loaded_identities']),
    }


def _balanced_blocks(text: str, token: str) -> list[str]:
    blocks: list[str] = []
    search_at = 0
    while True:
        start = text.find(token, search_at)
        if start < 0:
            return blocks
        brace = text.find('{', start)
        if brace < 0:
            return blocks
        depth = 0
        end = brace
        for end in range(brace, len(text)):
            if text[end] == '{':
                depth += 1
            elif text[end] == '}':
                depth -= 1
                if depth == 0:
                    blocks.append(text[start:end + 1])
                    search_at = end + 1
                    break
        else:
            return blocks


def gazebo_model_poses(scene_text: str) -> dict[str, dict[str, float]]:
    """Parse the static ``scene/info`` model snapshot.

    This parser is retained for offline evidence/tests only.  Gazebo's scene
    service reports the models' initial poses and therefore must not be used as
    runtime visibility evidence after ``set_pose`` moves a shuttle.
    """

    models: dict[str, dict[str, float]] = {}
    for block in _balanced_blocks(scene_text, 'model {'):
        name_match = re.search(r'\bname:\s*"([^"]+)"', block)
        if not name_match:
            continue
        pose_blocks = _balanced_blocks(block, 'pose {')
        pose = pose_blocks[0] if pose_blocks else ''
        position_blocks = _balanced_blocks(pose, 'position {')
        position = position_blocks[0] if position_blocks else ''
        coordinates = {}
        for axis in ('x', 'y', 'z'):
            match = re.search(
                rf'\b{axis}:\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)',
                position,
            )
            coordinates[axis] = float(match.group(1)) if match else 0.0
        models[name_match.group(1)] = coordinates
    return models


def gazebo_live_model_poses(pose_info_text: str) -> dict[str, dict[str, float]]:
    """Parse one live ``gz.msgs.Pose_V`` message from ``pose/info``."""

    models: dict[str, dict[str, float]] = {}
    for block in _balanced_blocks(pose_info_text, 'pose {'):
        name_match = re.search(r'\bname:\s*"([^"]+)"', block)
        if not name_match:
            continue
        position_blocks = _balanced_blocks(block, 'position {')
        if not position_blocks:
            continue
        position = position_blocks[0]
        coordinates: dict[str, float] = {}
        valid = True
        for axis in ('x', 'y', 'z'):
            match = re.search(rf'\b{axis}:\s*([^\s}}]+)', position)
            # Protobuf DebugString omits coordinates whose value is zero.
            try:
                coordinates[axis] = float(match.group(1)) if match else 0.0
            except ValueError:
                valid = False
                break
        if valid and all(math.isfinite(value) for value in coordinates.values()):
            models[name_match.group(1)] = coordinates
    return models


def query_gazebo_live_model_poses(
    world_name: str,
    *,
    environment: dict[str, str] | None = None,
    timeout_s: float = 2.5,
    runner: Any = None,
) -> tuple[dict[str, dict[str, float]], str]:
    """Read exactly one live pose message, returning no models on any failure."""

    env = os.environ.copy() if environment is None else environment.copy()
    if not str(env.get('GZ_PARTITION') or '').strip():
        return {}, 'GZ_PARTITION is empty'
    if not str(world_name or '').strip():
        return {}, 'world_name is empty'
    command = [
        'gz', 'topic', '--echo',
        '--topic', f'/world/{world_name}/pose/info',
        '--num', '1',
    ]
    run = subprocess.run if runner is None else runner
    try:
        completed = run(
            command,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
            env=env,
        )
    except subprocess.TimeoutExpired:
        return {}, f'live pose query timed out after {timeout_s:.3f}s'
    except OSError as exc:
        return {}, f'live pose query failed: {exc}'
    if completed.returncode != 0:
        detail = (
            completed.stderr or completed.stdout or 'gz pose topic query failed'
        ).strip()
        return {}, detail
    models = gazebo_live_model_poses(completed.stdout)
    if not models:
        return {}, 'live pose query returned no parseable poses'
    return models, ''


def scene_entity_checks(
    models: dict[str, dict[str, float]],
    expectation: AcceptanceExpectation,
) -> dict[str, Any]:
    missing_entities = []
    hidden_entities = []
    missing_payloads = []
    unexpected_payloads = []
    for shuttle in expectation.shuttles:
        pose = models.get(shuttle.entity_name)
        if pose is None:
            missing_entities.append(shuttle.entity_name)
        elif float(pose.get('z', -math.inf)) < 0.0:
            hidden_entities.append(shuttle.entity_name)
        payload_name = f'{shuttle.entity_name}_payload'
        if shuttle.loaded_state == 'loaded' and payload_name not in models:
            missing_payloads.append(payload_name)
        if shuttle.loaded_state == 'empty' and payload_name in models:
            unexpected_payloads.append(payload_name)
    return {
        'ready': not (
            missing_entities
            or hidden_entities
            or missing_payloads
            or unexpected_payloads
        ),
        'missing_entities': missing_entities,
        'hidden_entities': hidden_entities,
        'missing_payloads': missing_payloads,
        'unexpected_payloads': unexpected_payloads,
        'active_entity_poses': {
            shuttle.entity_name: models.get(shuttle.entity_name)
            for shuttle in expectation.shuttles
        },
    }


def validate_raw_prediction_contract(
    prediction: Any,
    *,
    expected_checkpoint_sha256: str,
    expected_schema_version: str,
) -> tuple[bool, dict[str, Any]]:
    """Validate the diagnostic-only raw prediction for V3 or V4 readiness."""

    errors: list[str] = []
    if not isinstance(prediction, dict):
        return False, {
            'schema_version': None,
            'checkpoint_sha256': None,
            'output_dimension': None,
            'output_value_count': 0,
            'control_input': None,
            'errors': ['prediction_not_object'],
        }
    if expected_schema_version not in SUPPORTED_RAW_PREDICTION_SCHEMAS:
        errors.append('unsupported_expected_schema')
    if prediction.get('schema_version') != expected_schema_version:
        errors.append('schema_version_mismatch')
    if prediction.get('checkpoint_sha256') != expected_checkpoint_sha256:
        errors.append('checkpoint_sha256_mismatch')
    output_dimension = prediction.get('output_dimension')
    if (
        isinstance(output_dimension, bool)
        or not isinstance(output_dimension, int)
        or output_dimension != 200
    ):
        errors.append('output_dimension_mismatch')
    vector = prediction.get('denormalized_output')
    if not isinstance(vector, list) or len(vector) != 200:
        errors.append('output_vector_length_mismatch')
        output_count = len(vector) if isinstance(vector, list) else 0
    else:
        output_count = len(vector)
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in vector
        ):
            errors.append('output_vector_nonfinite_or_nonnumeric')
    if prediction.get('control_input') is not False:
        errors.append('raw_prediction_not_diagnostic_only')
    if expected_schema_version == V4_RAW_PREDICTION_SCHEMA:
        if prediction.get('model_schema_version') != 'room315.visual_state.v4':
            errors.append('v4_model_schema_mismatch')
        if prediction.get('runtime_generation') != 'v4':
            errors.append('v4_runtime_generation_mismatch')
        if prediction.get('runtime_mode') not in {'shadow', 'active'}:
            errors.append('v4_runtime_mode_invalid')
        if not isinstance(prediction.get('acceptance_envelope'), dict):
            errors.append('v4_acceptance_envelope_missing')
    return not errors, {
        'schema_version': prediction.get('schema_version'),
        'checkpoint_sha256': prediction.get('checkpoint_sha256'),
        'output_dimension': output_dimension,
        'output_value_count': output_count,
        'control_input': prediction.get('control_input'),
        'runtime_generation': prediction.get('runtime_generation'),
        'runtime_mode': prediction.get('runtime_mode'),
        'errors': errors,
    }


class Room315AcceptanceReadiness(Node):
    def __init__(self) -> None:
        super().__init__('room_315_runtime_acceptance_readiness')
        for name, default in {
            'phase': '',
            'scenario_manifest_path': '',
            'scenario_id': '',
            'proof_path': '',
            'world_name': 'room_315_only',
            'timeout_s': 45.0,
            'freshness_s': DEFAULT_FRESHNESS_S,
            'position_ratio_tolerance': 0.02,
            'expected_checkpoint_sha256': '',
            'expected_raw_prediction_schema': V4_RAW_PREDICTION_SCHEMA,
            'left_state_topic': '/room_315/rails/left/shuttles/state',
            'right_state_topic': '/room_315/rails/right/shuttles/state',
            'left_payload_topic': '/room_315/rails/left/shuttles/payload_state',
            'right_payload_topic': '/room_315/rails/right/shuttles/payload_state',
            'left_image_topic': '/room_315/perception/left_rail_rgbd/image',
            'right_image_topic': '/room_315/perception/right_rail_rgbd/image',
            'raw_prediction_topic': '/room_315/visual_state/raw_model_prediction',
        }.items():
            self.declare_parameter(name, default)
        self.phase = self._string('phase').strip()
        if self.phase not in PHASES:
            raise RuntimeError(f'phase must be one of {PHASES}')
        manifest_path = Path(self._string('scenario_manifest_path')).resolve()
        self.expectation = scenario_expectation(
            load_json(manifest_path),
            self._string('scenario_id').strip(),
        )
        self.proof_path = Path(self._string('proof_path')).resolve()
        if self.proof_path.exists():
            raise RuntimeError(f'refusing to overwrite readiness proof: {self.proof_path}')
        self.world_name = self._string('world_name').strip()
        self.started = time.monotonic()
        self.exit_code = 1
        self.finished = False
        self.clock_received = False
        self.sim_time_s: float | None = None
        self.presence_provider = ShuttleStatePresenceProvider(
            timeout_s=self._float('freshness_s'),
            warmup_s=0.5,
        )
        self.states: dict[str, dict[str, tuple[ShuttleState, float]]] = {
            'left': {}, 'right': {},
        }
        self.payloads: dict[str, tuple[dict[str, Any], float]] = {}
        self.images: dict[str, tuple[dict[str, Any], float]] = {}
        self.raw_prediction: tuple[dict[str, Any], float] | None = None
        self.last_live_pose_query_s = 0.0
        self.live_pose_models: dict[str, dict[str, float]] = {}
        self.live_pose_query_error = 'not_queried'

        self.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data)
        self.create_subscription(
            ShuttleState,
            self._string('left_state_topic'),
            lambda message: self._on_state('left', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ShuttleState,
            self._string('right_state_topic'),
            lambda message: self._on_state('right', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            self._string('left_payload_topic'),
            lambda message: self._on_payload('left', message),
            10,
        )
        self.create_subscription(
            String,
            self._string('right_payload_topic'),
            lambda message: self._on_payload('right', message),
            10,
        )
        self.create_subscription(
            Image,
            self._string('left_image_topic'),
            lambda message: self._on_image('left', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self._string('right_image_topic'),
            lambda message: self._on_image('right', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            self._string('raw_prediction_topic'),
            self._on_raw_prediction,
            10,
        )
        self.service_clients = (
            self.create_client(
                SetEntityPose,
                f'/world/{self.world_name}/set_pose',
            ),
            self.create_client(
                SpawnEntity,
                f'/world/{self.world_name}/create',
            ),
            self.create_client(
                DeleteEntity,
                f'/world/{self.world_name}/remove',
            ),
        )
        self.create_timer(0.2, self._tick)

    def _on_clock(self, message: Clock) -> None:
        self.clock_received = True
        self.sim_time_s = (
            float(message.clock.sec)
            + float(message.clock.nanosec) / 1_000_000_000.0
        )

    def _on_state(self, side: str, message: ShuttleState) -> None:
        name = str(message.name or '').strip()
        if name:
            self.states[side][name] = (message, time.monotonic())
            if self.sim_time_s is not None:
                self.presence_provider.observe(
                    topic_side=side,
                    entity_name=name,
                    source_stamp_s=(
                        float(message.header.stamp.sec)
                        + float(message.header.stamp.nanosec) / 1_000_000_000.0
                    ),
                    receive_time_s=self.sim_time_s,
                )

    def _on_payload(self, side: str, message: String) -> None:
        try:
            payload = json.loads(message.data or '{}')
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self.payloads[side] = (payload, time.monotonic())

    def _on_image(self, side: str, message: Image) -> None:
        valid = bool(
            int(message.width) > 0
            and int(message.height) > 0
            and int(message.step) > 0
            and len(message.data) >= int(message.height) * int(message.step)
            and str(message.encoding or '').strip()
        )
        self.images[side] = ({
            'valid': valid,
            'width': int(message.width),
            'height': int(message.height),
            'step': int(message.step),
            'encoding': str(message.encoding),
            'data_bytes': len(message.data),
        }, time.monotonic())

    def _on_raw_prediction(self, message: String) -> None:
        try:
            payload = json.loads(message.data or '{}')
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self.raw_prediction = (payload, time.monotonic())

    def _tick(self) -> None:
        if self.finished:
            return
        now = time.monotonic()
        if now - self.last_live_pose_query_s >= 1.0 and self.phase != 'world':
            self._query_live_poses()
            # Querying is bounded but synchronous; freshness must be evaluated
            # at completion, not against the timestamp from before the read.
            now = time.monotonic()
            self.last_live_pose_query_s = now
        checks, evidence = self._checks(now)
        if all(checks.values()):
            self._finish(True, checks, evidence, ())
            return
        if now - self.started >= self._float('timeout_s'):
            missing = tuple(name for name, passed in checks.items() if not passed)
            self._finish(False, checks, evidence, missing)

    def _checks(self, now: float) -> tuple[dict[str, bool], dict[str, Any]]:
        services_ready = all(client.service_is_ready() for client in self.service_clients)
        checks = {
            'gazebo_world_services_ready': services_ready,
            'simulation_clock_received': self.clock_received,
        }
        evidence: dict[str, Any] = {
            'gz_partition': os.environ.get('GZ_PARTITION', ''),
            'expected_identities': list(self.expectation.identities),
        }
        if self.phase == 'world':
            return checks, evidence

        state_evidence = self._state_checks(now)
        payload_check, payload_evidence = self._payload_checks(now)
        scene_check = scene_entity_checks(self.live_pose_models, self.expectation)
        live_pose_ready = bool(
            not self.live_pose_query_error
            and scene_check['ready']
        )
        checks.update({
            'exact_fresh_shuttle_state_inventory': state_evidence['exact_inventory'],
            'configured_segment_and_ratio_match': state_evidence['positions_match'],
            'deterministic_presence_provider_ready': (
                state_evidence['presence_provider_ready']
            ),
            'payload_state_matches': payload_check,
            'gazebo_entities_visible_and_payloads_match': live_pose_ready,
        })
        evidence.update({
            'shuttle_state': state_evidence,
            'payload_state': payload_evidence,
            'gazebo_live_pose': {
                **scene_check,
                'ready': live_pose_ready,
                'source_topic': f'/world/{self.world_name}/pose/info',
                'static_scene_info_used': False,
                'query_error': self.live_pose_query_error,
            },
        })
        if self.phase == 'scene':
            return checks, evidence

        camera_check, camera_evidence = self._camera_checks(now)
        checks['fresh_valid_paired_camera_frames'] = camera_check
        evidence['cameras'] = camera_evidence
        if self.phase == 'camera':
            return checks, evidence

        prediction_check, prediction_evidence = self._prediction_checks(now)
        checks['candidate_checkpoint_produced_raw_prediction'] = prediction_check
        evidence['raw_prediction'] = prediction_evidence
        return checks, evidence

    def _state_checks(self, now: float) -> dict[str, Any]:
        freshness = self._float('freshness_s')
        positions_match = True
        observed: dict[str, Any] = {}
        fresh_by_side: dict[str, set[str]] = {'left': set(), 'right': set()}
        for side, records in self.states.items():
            lengths = public_rail_segment_lengths(side)
            for name, (message, received) in records.items():
                if now - received > freshness:
                    continue
                fresh_by_side[side].add(name)
                shuttle = next(
                    (item for item in self.expectation.shuttles if item.entity_name == name),
                    None,
                )
                ratio = None
                if shuttle is not None and message.current_segment == shuttle.segment:
                    ratio = float(message.s) / float(lengths[shuttle.segment])
                    if abs(ratio - shuttle.s_ratio) > self._float(
                        'position_ratio_tolerance'
                    ):
                        positions_match = False
                elif shuttle is not None:
                    positions_match = False
                observed[name] = {
                    'side': side,
                    'segment': str(message.current_segment),
                    's_m': float(message.s),
                    's_ratio_from_oracle_for_setup_verification_only': ratio,
                    'age_s': now - received,
                }
        expected_by_side = {
            side: {
                item.entity_name for item in self.expectation.shuttles
                if item.side == side
            }
            for side in ('left', 'right')
        }
        exact = all(
            fresh_by_side[side] == expected_by_side[side]
            for side in ('left', 'right')
        )
        presence_evidence: dict[str, Any]
        presence_ready = False
        if self.sim_time_s is None:
            presence_evidence = {
                'ready': False,
                'reasons': ['simulation_clock_not_received'],
                'identity_states': {},
            }
        else:
            snapshot = self.presence_provider.snapshot(now_s=self.sim_time_s)
            identity_states = {
                entry.identity: entry.state for entry in snapshot.entries
            }
            present = {
                identity for identity, state in identity_states.items()
                if state == 'present'
            }
            unknown = {
                identity for identity, state in identity_states.items()
                if state == 'unknown'
            }
            presence_ready = bool(
                snapshot.ready
                and present == set(self.expectation.identities)
                and not unknown
            )
            presence_evidence = {
                'ready': snapshot.ready,
                'acceptance_inventory_match': presence_ready,
                'source': snapshot.source,
                'reasons': list(snapshot.reasons),
                'initialized_sides': list(snapshot.initialized_sides),
                'stale_sides': list(snapshot.stale_sides),
                'identity_states': identity_states,
                'expected_present_identities': list(self.expectation.identities),
                'authorized_fields': [
                    'msg.name', 'msg.header.stamp', 'ROS receive time',
                ],
            }
        return {
            'exact_inventory': exact,
            'positions_match': positions_match,
            'presence_provider_ready': presence_ready,
            'presence_provider': presence_evidence,
            'expected_by_side': {
                side: sorted(values) for side, values in expected_by_side.items()
            },
            'fresh_by_side': {
                side: sorted(values) for side, values in fresh_by_side.items()
            },
            'observed': observed,
            'position_oracle_used_as_model_input': False,
        }

    def _payload_checks(self, now: float) -> tuple[bool, dict[str, Any]]:
        freshness = self._float('freshness_s')
        observed: dict[str, str] = {}
        source_sides: list[str] = []
        for side, value in self.payloads.items():
            payload, received = value
            if now - received > freshness:
                continue
            source_sides.append(side)
            for shuttle in payload.get('shuttles') or []:
                identity = str(shuttle.get('short_id') or '').upper()
                if identity:
                    observed[identity] = (
                        'loaded' if bool(shuttle.get('loaded')) else 'empty'
                    )
        expected = {
            item.identity: item.loaded_state for item in self.expectation.shuttles
        }
        ready = set(source_sides) == {'left', 'right'} and observed == expected
        return ready, {
            'source_sides': sorted(source_sides),
            'expected': expected,
            'observed': observed,
            'payload_oracle_used_as_model_input': False,
        }

    def _camera_checks(self, now: float) -> tuple[bool, dict[str, Any]]:
        freshness = self._float('freshness_s')
        evidence = {}
        ready = True
        for side in ('left', 'right'):
            value = self.images.get(side)
            if value is None:
                evidence[side] = {'valid': False, 'reason': 'not_received'}
                ready = False
                continue
            image, received = value
            age = now - received
            evidence[side] = {**image, 'age_s': age}
            ready = ready and bool(image['valid']) and age <= freshness
        return ready, evidence

    def _prediction_checks(self, now: float) -> tuple[bool, dict[str, Any]]:
        if self.raw_prediction is None:
            return False, {'status': 'not_received'}
        prediction, received = self.raw_prediction
        age = now - received
        expected_hash = self._string('expected_checkpoint_sha256').strip()
        expected_schema = self._string(
            'expected_raw_prediction_schema'
        ).strip()
        contract_ready, contract_evidence = validate_raw_prediction_contract(
            prediction,
            expected_checkpoint_sha256=expected_hash,
            expected_schema_version=expected_schema,
        )
        ready = bool(
            age <= self._float('freshness_s')
            and contract_ready
        )
        return ready, {
            'status': 'observed',
            'age_s': age,
            **contract_evidence,
        }

    def _query_live_poses(self) -> None:
        # Replace, rather than retain, prior evidence.  A transport failure must
        # never allow the last successful (or initial scene) snapshot to pass.
        self.live_pose_models, self.live_pose_query_error = (
            query_gazebo_live_model_poses(self.world_name)
        )

    def _finish(
        self,
        success: bool,
        checks: dict[str, bool],
        evidence: dict[str, Any],
        missing: tuple[str, ...],
    ) -> None:
        self.finished = True
        self.exit_code = 0 if success else 1
        proof = {
            'schema_version': 'room315.runtime_acceptance_readiness.v1',
            'scenario_id': self.expectation.scenario_id,
            'phase': self.phase,
            'status': 'ready' if success else 'failed',
            'checks': checks,
            'missing_checks': list(missing),
            'evidence': evidence,
            'elapsed_s': time.monotonic() - self.started,
            'observation_only': True,
        }
        self.proof_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.proof_path.with_name(
            f'.{self.proof_path.name}.tmp-{os.getpid()}'
        )
        temporary.write_text(
            json.dumps(proof, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        os.replace(temporary, self.proof_path)
        if success:
            self.get_logger().info(
                f'READINESS_PASS phase={self.phase} proof={self.proof_path}'
            )
        else:
            self.get_logger().error(
                f'READINESS_FAIL phase={self.phase} missing={list(missing)} '
                f'proof={self.proof_path}'
            )
        rclpy.shutdown()

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = Room315AcceptanceReadiness()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        rclpy.try_shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
