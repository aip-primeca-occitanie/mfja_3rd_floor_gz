#!/usr/bin/env python3
"""ROS graph smoke for the fail-closed Room 315 visual runtime."""

from __future__ import annotations

import json
import os
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest


os.environ.setdefault('ROS_DOMAIN_ID', '97')

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('message_filters')
from sensor_msgs.msg import Image

from room_315_presence_provider import PresenceEntry
from room_315_presence_provider import PresenceSnapshot
from room_315_visual_runtime import FIXED_IDENTITY_ORDER
from room_315_visual_runtime import InferenceTimings
from room_315_visual_state_inference_node import SHADOW_TOPIC_ROOT
from room_315_visual_state_inference_node import Room315VisualStateInferenceNode
from room_315_visual_state_inference_node import _v4_raw_prediction_payload


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _ready_presence(now_s):
    return PresenceSnapshot(
        timestamp_s=now_s,
        ready=True,
        entries=tuple(
            PresenceEntry(
                identity=identity,
                side='left' if identity.startswith('L') else 'right',
                state='present' if identity == 'L1' else 'absent',
            )
            for identity in FIXED_IDENTITY_ORDER
        ),
        reasons=(),
        initialized_sides=('left', 'right'),
        stale_sides=(),
        source='node_test',
    )


def _rgb_message(stamp):
    message = Image()
    message.header.stamp = stamp
    message.height = 1
    message.width = 1
    message.encoding = 'rgb8'
    message.step = 3
    message.data = bytes((0, 0, 0))
    return message


def test_v4_raw_payload_is_diagnostic_only_and_keeps_acceptance_envelope():
    diagnostic = SimpleNamespace(
        schema_version='room315.visual_runtime_v4.diagnostic.v1',
        legacy_vector=tuple(float(index) for index in range(200)),
        acceptance={
            'schema_version': 'room315.visual_acceptance.v4',
            'accepted': False,
            'reasons': ('L1:low_segment_confidence',),
            'slots': ({
                'identity': 'L1',
                'segment_confidence': 0.42,
                'loaded_confidence': 0.95,
                'accepted': False,
            },),
        },
        shuttles=(),
        control_input_permitted=False,
    )

    payload = _v4_raw_prediction_payload(
        diagnostic,
        checkpoint_sha256='a' * 64,
        promotion_manifest_sha256='b' * 64,
        runtime_mode='shadow',
        timestamp_s=3.0,
        left_image_stamp_s=2.9,
        right_image_stamp_s=2.9,
    )

    assert payload['schema_version'] == diagnostic.schema_version
    assert payload['model_schema_version'] == 'room315.visual_state.v4'
    assert payload['runtime_generation'] == 'v4'
    assert payload['runtime_mode'] == 'shadow'
    assert payload['output_dimension'] == 200
    assert len(payload['denormalized_output']) == 200
    assert not payload['control_input']
    assert not payload['acceptance_envelope']['accepted']
    assert (
        payload['acceptance_envelope']['slots'][0]['segment_confidence']
        == pytest.approx(0.42)
    )


def _fake_v4_module(*, deployment_mode='shadow', decode_error=None):
    calls = {'verify': [], 'runtime_created': 0, 'decode': 0}
    promotion = SimpleNamespace(
        manifest_sha256='b' * 64,
        checkpoint_sha256='a' * 64,
        model_kind='room315_visual_state_resnet18_split_rails_v4',
        deployment_mode=deployment_mode,
        authorization_scope='gazebo_v4_shadow_observation_only',
        runtime_guards={
            'dry_run_state_fusion': True,
            'plansys2_update_enabled': False,
        },
    )

    class Runtime:
        def __init__(self, received_promotion, *, device):
            assert received_promotion is promotion
            calls['runtime_created'] += 1
            self.device = 'cpu' if device == 'auto' else device
            self.model_load_duration_ms = 1.25
            self.ready = False

        def load(self):
            self.ready = True

        def infer(self, _left, _right):
            return object(), InferenceTimings(1.0, 2.0, 0.0, 3.0)

    def verify(path, expected_sha256):
        calls['verify'].append((path, expected_sha256))
        return promotion

    def diagnostic(
        _structured,
        *,
        promotion,
        presence,
        left_image_size,
        right_image_size,
    ):
        assert promotion is not None
        assert presence.ready
        assert left_image_size == (1, 1)
        assert right_image_size == (1, 1)
        return SimpleNamespace(
            schema_version='room315.visual_runtime_v4.diagnostic.v1',
            legacy_vector=(0.0,) * 200,
            acceptance={
                'accepted': False,
                'reasons': ('L1:low_segment_confidence',),
            },
            shuttles=(),
            control_input_permitted=False,
        )

    def decode(_structured, **_kwargs):
        calls['decode'] += 1
        if decode_error is not None:
            raise RuntimeError(decode_error)
        raise AssertionError('this fake expects a fail-closed decode')

    module = types.ModuleType('room_315_visual_runtime_v4')
    module.Room315VisualModelRuntimeV4 = Runtime
    module.build_diagnostic_legacy_output_v4 = diagnostic
    module.decode_active_slots_v4 = decode
    module.verify_v4_runtime_promotion = verify
    return module, calls


def test_ros_node_rejects_v3_runtime_generation():
    rclpy.init(args=[
        '--ros-args',
        '-p', 'use_sim_time:=false',
        '-p', 'runtime_generation:=v3',
    ])
    try:
        with pytest.raises(
            ValueError,
            match=r'runtime_generation must be one of \{v4\}',
        ):
            Room315VisualStateInferenceNode()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_unconfigured_node_defaults_to_v4_and_fails_closed(monkeypatch):
    for environment_name in (
        'ROOM315_VISUAL_V4_PROMOTION_MANIFEST_PATH',
        'ROOM315_VISUAL_V4_MANIFEST_PATH',
        'ROOM315_VISUAL_EXPECTED_V4_PROMOTION_MANIFEST_SHA256',
        'ROOM315_VISUAL_V4_EXPECTED_PROMOTION_MANIFEST_SHA256',
    ):
        monkeypatch.delenv(environment_name, raising=False)

    rclpy.init(args=[
        '--ros-args',
        '-p', 'use_sim_time:=false',
    ])
    runtime = Room315VisualStateInferenceNode()
    try:
        assert runtime.runtime_generation == 'v4'
        assert runtime.runtime_mode == 'active'
        assert runtime.model_schema == 'room315.visual_state.v4'
        assert runtime.model_runtime is None
        assert runtime.v4_promotion is None
        assert 'v4_promotion_manifest_path parameter is empty' in (
            runtime.artifact_error
        )
        assert not runtime._effective_plansys2_update_enabled()
    finally:
        runtime.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_v4_shadow_isolated_and_confidence_failure_rejects_callback(monkeypatch):
    fake_module, calls = _fake_v4_module(
        deployment_mode='shadow',
        decode_error='low_segment_confidence:L1',
    )
    monkeypatch.setitem(sys.modules, 'room_315_visual_runtime_v4', fake_module)
    for environment_name in (
        'ROOM315_VISUAL_V4_PROMOTION_MANIFEST_PATH',
        'ROOM315_VISUAL_V4_MANIFEST_PATH',
        'ROOM315_VISUAL_EXPECTED_V4_PROMOTION_MANIFEST_SHA256',
        'ROOM315_VISUAL_V4_EXPECTED_PROMOTION_MANIFEST_SHA256',
    ):
        monkeypatch.delenv(environment_name, raising=False)

    manifest = '/tmp/node-test-v4-promotion.json'
    manifest_sha256 = 'b' * 64
    rclpy.init(args=[
        '--ros-args',
        '-p', 'use_sim_time:=false',
        '-p', 'runtime_generation:=v4',
        '-p', 'runtime_mode:=shadow',
        '-p', f'v4_promotion_manifest_path:={manifest}',
        '-p', f'expected_v4_promotion_manifest_sha256:={manifest_sha256}',
        '-p', 'raw_observation_topic:=/unsafe/active/raw',
        '-p', 'raw_model_prediction_topic:=/unsafe/active/model',
        '-p', 'validation_topic:=/unsafe/active/validation',
        '-p', 'accepted_observed_state_topic:=/unsafe/active/observed',
        '-p', 'diagnostics_topic:=/unsafe/active/diagnostics',
        '-p', 'dry_run_state_fusion:=false',
        '-p', 'plansys2_update_enabled:=true',
        '-p', 'presence_warmup_s:=0.0',
        '-p', 'presence_state_timeout_s:=5.0',
    ])
    runtime = Room315VisualStateInferenceNode()
    try:
        assert runtime.runtime_generation == 'v4'
        assert runtime.runtime_mode == 'shadow'
        assert runtime.model_schema == 'room315.visual_state.v4'
        assert runtime.model_runtime is not None
        assert runtime.plan_client is None
        assert runtime._effective_dry_run_state_fusion()
        assert not runtime._effective_plansys2_update_enabled()
        assert calls['verify'] == [(Path(manifest), manifest_sha256)]
        assert calls['runtime_created'] == 1

        publishers = runtime.get_publisher_names_and_types_by_node(
            runtime.get_name(),
            runtime.get_namespace(),
        )
        visual_topics = {
            topic
            for topic, _types in publishers
            if topic.startswith('/room_315/visual_state/')
            or topic.startswith('/unsafe/active/')
        }
        assert visual_topics == {
            f'{SHADOW_TOPIC_ROOT}/raw',
            f'{SHADOW_TOPIC_ROOT}/raw_model_prediction',
            f'{SHADOW_TOPIC_ROOT}/validation',
            f'{SHADOW_TOPIC_ROOT}/observed_state',
            f'{SHADOW_TOPIC_ROOT}/diagnostics',
        }

        now_s = runtime._now_s()
        ready_presence = _ready_presence(now_s)
        runtime.presence = SimpleNamespace(
            snapshot=lambda *, now_s: ready_presence,
        )
        raw_capture = _CapturePublisher()
        validation_capture = _CapturePublisher()
        diagnostics_capture = _CapturePublisher()
        runtime.raw_model_prediction_pub = raw_capture
        runtime.validation_pub = validation_capture
        runtime.diagnostics_pub = diagnostics_capture
        stamp = runtime.get_clock().now().to_msg()

        runtime._on_image_pair(_rgb_message(stamp), _rgb_message(stamp))

        assert calls['decode'] == 1
        assert len(raw_capture.messages) == 1
        raw_payload = json.loads(raw_capture.messages[0].data)
        assert raw_payload['output_dimension'] == 200
        assert not raw_payload['control_input']
        assert not raw_payload['acceptance_envelope']['accepted']
        assert len(validation_capture.messages) == 1
        rejection = validation_capture.messages[0]
        assert not rejection.accepted
        assert any(
            'low_segment_confidence:L1' in reason
            for reason in rejection.validation_reasons
        )

        runtime._publish_diagnostics()
        assert len(diagnostics_capture.messages) == 1
        diagnostic_values = {
            item.key: json.loads(item.value)
            for item in diagnostics_capture.messages[0].status[0].values
        }
        assert diagnostic_values['runtime_generation'] == 'v4'
        assert diagnostic_values['runtime_mode'] == 'shadow'
        assert diagnostic_values['model_kind'] == runtime.model_kind
        assert diagnostic_values['promotion_manifest_sha256'] == 'b' * 64
        assert diagnostic_values['dry_run_state_fusion']
        assert not diagnostic_values['plansys2_update_enabled']
    finally:
        runtime.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_v4_shadow_manifest_cannot_be_started_as_active(monkeypatch):
    fake_module, calls = _fake_v4_module(deployment_mode='shadow')
    monkeypatch.setitem(sys.modules, 'room_315_visual_runtime_v4', fake_module)
    manifest_sha256 = 'b' * 64
    rclpy.init(args=[
        '--ros-args',
        '-p', 'use_sim_time:=false',
        '-p', 'runtime_generation:=v4',
        '-p', 'runtime_mode:=active',
        '-p', 'v4_promotion_manifest_path:=/tmp/shadow-only.json',
        '-p',
        f'expected_v4_promotion_manifest_sha256:={manifest_sha256}',
    ])
    runtime = Room315VisualStateInferenceNode()
    try:
        assert runtime.model_runtime is None
        assert calls['runtime_created'] == 0
        assert 'deployment_mode does not authorize' in runtime.artifact_error
    finally:
        runtime.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_v4_manifest_environment_values_override_ros_parameters(monkeypatch):
    fake_module, calls = _fake_v4_module(deployment_mode='shadow')
    monkeypatch.setitem(sys.modules, 'room_315_visual_runtime_v4', fake_module)
    environment_manifest = '/tmp/v4-from-environment.json'
    environment_sha256 = 'c' * 64
    parameter_sha256 = 'd' * 64
    monkeypatch.setenv(
        'ROOM315_VISUAL_V4_PROMOTION_MANIFEST_PATH',
        environment_manifest,
    )
    monkeypatch.setenv(
        'ROOM315_VISUAL_EXPECTED_V4_PROMOTION_MANIFEST_SHA256',
        environment_sha256,
    )
    monkeypatch.delenv('ROOM315_VISUAL_V4_MANIFEST_PATH', raising=False)
    monkeypatch.delenv(
        'ROOM315_VISUAL_V4_EXPECTED_PROMOTION_MANIFEST_SHA256',
        raising=False,
    )
    rclpy.init(args=[
        '--ros-args',
        '-p', 'use_sim_time:=false',
        '-p', 'runtime_generation:=v4',
        '-p', 'runtime_mode:=shadow',
        '-p', 'v4_promotion_manifest_path:=/tmp/v4-from-parameter.json',
        '-p',
        f'expected_v4_promotion_manifest_sha256:={parameter_sha256}',
    ])
    runtime = Room315VisualStateInferenceNode()
    try:
        assert runtime.model_runtime is not None
        assert calls['verify'] == [(
            Path(environment_manifest),
            environment_sha256,
        )]
    finally:
        runtime.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_v4_runtime_launch_passes_manifest_explicitly():
    launch_source = (
        ROOT / 'launch' / 'room_315_visual_state_runtime.launch.py'
    ).read_text(encoding='utf-8')

    assert (
        "('v4_promotion_manifest', 'v4_promotion_manifest_path')"
        in launch_source
    )
    assert (
        "'v4_promotion_manifest_sha256',\n"
        "            'expected_v4_promotion_manifest_sha256',"
    ) in launch_source
    assert "choices=['v4']" in launch_source
    assert (
        "'runtime_generation': LaunchConfiguration('runtime_generation')"
        in launch_source
    )
