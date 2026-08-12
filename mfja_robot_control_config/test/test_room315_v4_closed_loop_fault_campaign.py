#!/usr/bin/env python3

from __future__ import annotations

import copy
import hashlib
import inspect
import json
import stat
import sys
import time
from pathlib import Path

import pytest
import yaml


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import room_315_v4_closed_loop_fault_campaign as campaign  # noqa: E402


def _audit(
    *,
    actuating: int = 0,
    successes: int = 0,
    v3: int = 0,
    wrong: int = 0,
    estop: bool = False,
) -> dict[str, object]:
    return {
        'status': 'passed',
        'v4_observation_count': 7,
        'v3_observation_count': v3,
        'wrong_visual_provenance_count': wrong,
        'success_status_count': successes,
        'actuating_command_count': actuating,
        'emergency_stop_values': [True] if estop else [],
    }


def _stationary() -> dict[str, object]:
    return {
        'before_s_m': 1.0,
        'after_s_m': 1.0,
        'absolute_delta_s_m': 0.0,
        'final_mode': 'DISABLED',
    }


def _disabled_state() -> dict[str, object]:
    return {
        'name': 'room315_right_shuttle_4',
        'mode': 'DISABLED',
        's': 1.0,
    }


def _non_success_status(value: str = 'failed') -> dict[str, object]:
    return {
        'goal_id': 'room315-v4-fault-r4-slot1-to-slot4',
        'status': value,
        'reason': 'injected qualification fault',
    }


def _feedback_proof() -> dict[str, object]:
    topic = '/room_315/fault/F05/right_sensor_blackhole'
    unavailable = {
        'status': 'unavailable',
        'topic': topic,
        'publisher_count': 0,
        'subscription_count': 1,
    }
    return {
        'schema_version': 'room315.v4_feedback_unavailable_proof.v1',
        'status': 'passed',
        'configuration': {
            'status': 'configured',
            'topic': topic,
            'runtime_config_path': '/evidence/task_execution_runtime.yaml',
            'runtime_config_sha256': 'a' * 64,
        },
        'before_goal': copy.deepcopy(unavailable),
        'after_terminal': copy.deepcopy(unavailable),
    }


def _minimal_task_config(path: Path) -> None:
    path.write_text(
        yaml.safe_dump({
            'room_315_task_execution_node': {
                'ros__parameters': {
                    'execution_enabled': False,
                    'task_execution_authorization_path': '/bound/candidate.json',
                    'task_execution_authorization_sha256': 'a' * 64,
                    'task_execution_promotion_manifest_path': '/bound/manifest.json',
                    'right_sensor_feedback_topic': (
                        '/room_315/rails/right/sensors/feedback'
                    ),
                },
            },
        }, sort_keys=False),
        encoding='utf-8',
    )


def test_fault_matrix_is_exact_complete_and_never_expects_success():
    campaign.validate_scenarios(campaign.FAULT_SCENARIOS)

    assert [row.scenario_id for row in campaign.FAULT_SCENARIOS] == [
        'F01', 'F02', 'F03', 'F04', 'F05',
    ]
    assert [row.fault for row in campaign.FAULT_SCENARIOS] == [
        'corrupt_authorization',
        'wrong_promotion_manifest',
        'planner_unavailable',
        'emergency_stop_during_long_move',
        'right_sensor_feedback_loss',
    ]
    assert all(row.expected_terminal != 'succeeded'
               for row in campaign.FAULT_SCENARIOS)
    f04 = campaign.scenario_by_id('F04')
    f05 = campaign.scenario_by_id('F05')
    assert f04.expect_actuating_command is True
    assert f04.inject_emergency_stop is True
    assert f05.expect_actuating_command is True
    assert f05.inject_emergency_stop is False
    assert f05.sensor_feedback_available is False


def test_frozen_v4_qualification_identifiers_are_exact():
    assert campaign.AUTHORIZATION_SCOPE == (
        'gazebo_v4_closed_loop_qualification_only'
    )
    assert campaign.V4_SCHEMA == 'room315.visual_state.v4'
    assert campaign.V4_CHECKPOINT_SHA256 == (
        '869d64049b0092c37d21a4c8b910dc6b91954527e0e49c5694fa82dce570f40d'
    )
    assert campaign.V4_PROMOTION_MANIFEST_SHA256 == (
        '6f9828219c22599825f5a14e405c8f11ce017984cc0d65821a357240d6529e2a'
    )
    assert campaign.TASK_RUNTIME_CONFIG_SHA256 == (
        'f2282108cb058caf400846b5b187b85c6a0e849b936cb77623267f06929669e2'
    )


def test_commands_are_gazebo_only_v4_and_keep_problem_expert_mirror_off():
    templates = campaign.verify_command_templates()
    visual = templates['visual']
    floor = templates['floor']
    execution = templates['execution']

    assert 'runtime_generation:=v4' in visual
    assert 'runtime_mode:=active' in visual
    assert 'dry_run_state_fusion:=true' in visual
    assert 'plansys2_update_enabled:=false' in visual
    assert f'v4_promotion_manifest:={campaign.V4_PROMOTION_MANIFEST}' in visual
    assert 'gui:=false' in floor
    assert 'robots:=none' in floor
    assert 'room315_right_active_identities:=R4' in floor
    assert 'execution_enabled:=true' in execution
    assert all('physical' not in value.casefold()
               for value in [*visual, *floor, *execution])


def test_long_move_goal_is_explicit_r4_slot1_to_slot4():
    goal = campaign.long_move_task_goal()

    assert goal['contract_type'] == 'TaskGoal'
    assert goal['source'] == 'human'
    assert goal['constraints']['target_shuttle'] == 'room315_right_shuttle_4'
    assert goal['constraints']['target_slot'] == '4'
    assert goal['constraints']['side'] == 'right'


def test_emergency_actuation_capture_starts_before_task_publication():
    source = inspect.getsource(campaign.run_worker)

    assert source.index('start_actuating_command_capture(') < source.index(
        'publish_result = publish_task_goal_with_ack('
    )
    assert 'if scenario.inject_emergency_stop:' in source


class _FakeString:
    def __init__(self):
        self.data = ''


class _FakePublisher:
    def __init__(self, runtime):
        self.runtime = runtime
        self.messages = []

    def get_subscription_count(self):
        return 1

    def publish(self, message):
        self.messages.append(message.data)
        self.runtime.goal_published = True


class _FakeEndpoint:
    def __init__(self, node_name):
        self.node_name = node_name


class _FakeNode:
    def __init__(self, runtime):
        self.runtime = runtime
        self.publisher = _FakePublisher(runtime)
        self.status_callback = None
        self.destroyed = False

    def create_publisher(self, _message_type, _topic, _depth):
        return self.publisher

    def create_subscription(self, _message_type, _topic, callback, _depth):
        self.status_callback = callback
        return object()

    def count_publishers(self, _topic):
        return 1

    def get_subscriptions_info_by_topic(self, _topic):
        endpoints = [_FakeEndpoint('rosbag2_recorder')]
        if self.runtime.gateway_discovered:
            endpoints.append(_FakeEndpoint('room_315_task_execution_node'))
        return endpoints

    def get_publishers_info_by_topic(self, _topic):
        if self.runtime.gateway_discovered:
            return [_FakeEndpoint('room_315_task_execution_node')]
        return []

    def destroy_node(self):
        self.destroyed = True


class _FakeRclpy:
    def __init__(self, *, acknowledge, gateway_discovered=True):
        self.acknowledge = acknowledge
        self.gateway_discovered = gateway_discovered
        self.goal_published = False
        self.ack_sent = False
        self.initialized = False
        self.shutdown_called = False
        self.node = _FakeNode(self)

    def init(self, *, args):
        assert args == []
        self.initialized = True

    def create_node(self, name):
        assert name.startswith('room315_v4_fault_goal_publisher_')
        return self.node

    def spin_once(self, _node, *, timeout_sec):
        if self.goal_published and self.acknowledge and not self.ack_sent:
            message = _FakeString()
            message.data = json.dumps({
                'goal_id': 'room315-v4-fault-r4-slot1-to-slot4',
                'status': 'accepted',
            })
            self.node.status_callback(message)
            self.ack_sent = True
        elif not self.acknowledge:
            time.sleep(timeout_sec)

    def ok(self):
        return self.initialized and not self.shutdown_called

    def shutdown(self):
        self.shutdown_called = True


def test_persistent_task_goal_publisher_publishes_once_and_waits_for_exact_ack():
    runtime = _FakeRclpy(acknowledge=True)
    goal = campaign.long_move_task_goal()

    evidence = campaign.publish_task_goal_once_wait_ack(
        goal,
        timeout_s=0.1,
        rclpy_module=runtime,
        string_type=_FakeString,
    )

    assert evidence['status'] == 'acknowledged'
    assert evidence['acknowledgement_status'] == 'accepted'
    assert evidence['published_count'] == 1
    assert evidence['unsafe_retry_performed'] is False
    assert evidence['matched_gateway_task_subscription_count'] == 1
    assert evidence['matched_gateway_status_publisher_count'] == 1
    assert runtime.node.publisher.messages == [json.dumps(goal, sort_keys=True)]
    assert runtime.node.destroyed is True
    assert runtime.shutdown_called is True


def test_missing_task_goal_ack_fails_closed_without_duplicate_publish():
    runtime = _FakeRclpy(acknowledge=False)

    with pytest.raises(campaign.QualificationError, match='unsafe retry'):
        campaign.publish_task_goal_once_wait_ack(
            campaign.long_move_task_goal(),
            timeout_s=0.005,
            rclpy_module=runtime,
            string_type=_FakeString,
        )

    assert len(runtime.node.publisher.messages) == 1
    assert runtime.node.destroyed is True
    assert runtime.shutdown_called is True


def test_rosbag_subscription_alone_never_authorizes_task_goal_publication():
    runtime = _FakeRclpy(
        acknowledge=False,
        gateway_discovered=False,
    )

    with pytest.raises(
        campaign.QualificationError,
        match='exact task execution gateway',
    ):
        campaign.publish_task_goal_once_wait_ack(
            campaign.long_move_task_goal(),
            timeout_s=0.005,
            rclpy_module=runtime,
            string_type=_FakeString,
        )

    assert runtime.node.publisher.messages == []
    assert runtime.node.destroyed is True
    assert runtime.shutdown_called is True


def test_task_goal_acknowledgement_requires_exact_goal_and_known_status():
    goal_id = campaign.long_move_task_goal()['goal_id']
    accepted = json.dumps({'goal_id': goal_id, 'status': 'RUNNING'})

    assert campaign.parse_task_goal_acknowledgement(
        accepted, goal_id=goal_id,
    )['status'] == 'running'
    assert campaign.parse_task_goal_acknowledgement(
        json.dumps({'goal_id': 'another-goal', 'status': 'accepted'}),
        goal_id=goal_id,
    ) is None
    assert campaign.parse_task_goal_acknowledgement(
        json.dumps({'goal_id': goal_id, 'status': 'idle'}),
        goal_id=goal_id,
    ) is None


class _FakeBool:
    def __init__(self):
        self.data = False


class _FakeEstopPublisher:
    def __init__(self, runtime):
        self.runtime = runtime
        self.messages = []

    def get_subscription_count(self):
        return int(self.runtime.supervisor_discovered) + int(
            self.runtime.recorder_discovered
        )

    def publish(self, message):
        self.messages.append(message.data)
        self.runtime.estop_published = True


class _FakeEstopNode:
    def __init__(self, runtime):
        self.runtime = runtime
        self.publisher = _FakeEstopPublisher(runtime)
        self.status_callback = None
        self.destroyed = False

    def create_publisher(self, _message_type, _topic, _depth):
        return self.publisher

    def create_subscription(self, _message_type, _topic, callback, _depth):
        self.status_callback = callback
        return object()

    def get_subscriptions_info_by_topic(self, _topic):
        endpoints = []
        if self.runtime.supervisor_discovered:
            endpoints.append(_FakeEndpoint('room_315_vla_supervisor'))
        if self.runtime.recorder_discovered:
            endpoints.append(_FakeEndpoint('rosbag2_recorder'))
        return endpoints

    def get_publishers_info_by_topic(self, _topic):
        if self.runtime.supervisor_discovered:
            return [_FakeEndpoint('room_315_vla_supervisor')]
        return []

    def count_publishers(self, _topic):
        return int(self.runtime.supervisor_discovered)

    def destroy_node(self):
        self.destroyed = True


class _FakeEstopRclpy:
    def __init__(
        self,
        *,
        acknowledge,
        supervisor_discovered=True,
        recorder_discovered=True,
    ):
        self.acknowledge = acknowledge
        self.supervisor_discovered = supervisor_discovered
        self.recorder_discovered = recorder_discovered
        self.estop_published = False
        self.ack_sent = False
        self.initialized = False
        self.shutdown_called = False
        self.node = _FakeEstopNode(self)

    def init(self, *, args):
        assert args == []
        self.initialized = True

    def create_node(self, name):
        assert name.startswith('room315_v4_fault_estop_publisher_')
        return self.node

    def spin_once(self, _node, *, timeout_sec):
        if self.estop_published and self.acknowledge and not self.ack_sent:
            message = _FakeString()
            message.data = json.dumps({
                'emergency_stop': True,
                'last_result': 'external emergency stop: all shuttles OFF',
            })
            self.node.status_callback(message)
            self.ack_sent = True
        else:
            time.sleep(timeout_sec)

    def ok(self):
        return self.initialized and not self.shutdown_called

    def shutdown(self):
        self.shutdown_called = True


def test_persistent_estop_matches_supervisor_and_recorder_then_publishes_once():
    runtime = _FakeEstopRclpy(acknowledge=True)

    evidence = campaign.publish_estop_once_wait_ack(
        timeout_s=0.1,
        delivery_hold_s=0.001,
        rclpy_module=runtime,
        bool_type=_FakeBool,
        string_type=_FakeString,
    )

    assert evidence['status'] == 'acknowledged'
    assert evidence['published_count'] == 1
    assert evidence['unsafe_retry_performed'] is False
    assert evidence['matched_supervisor_subscription_count'] == 1
    assert evidence['matched_recorder_subscription_count'] == 1
    assert runtime.node.publisher.messages == [True]
    assert runtime.node.destroyed is True
    assert runtime.shutdown_called is True


def test_estop_never_publishes_until_rosbag_recorder_is_matched():
    runtime = _FakeEstopRclpy(
        acknowledge=False,
        recorder_discovered=False,
    )

    with pytest.raises(campaign.QualificationError, match='rosbag'):
        campaign.publish_estop_once_wait_ack(
            timeout_s=0.005,
            delivery_hold_s=0.0,
            rclpy_module=runtime,
            bool_type=_FakeBool,
            string_type=_FakeString,
        )

    assert runtime.node.publisher.messages == []


def test_missing_estop_ack_fails_closed_without_duplicate_publish():
    runtime = _FakeEstopRclpy(acknowledge=False)

    with pytest.raises(campaign.QualificationError, match='unsafe retry'):
        campaign.publish_estop_once_wait_ack(
            timeout_s=0.005,
            delivery_hold_s=0.0,
            rclpy_module=runtime,
            bool_type=_FakeBool,
            string_type=_FakeString,
        )

    assert runtime.node.publisher.messages == [True]


def test_estop_ack_requires_applied_supervisor_stop_all_snapshot():
    expected = {
        'emergency_stop': True,
        'last_result': 'external emergency stop: all shuttles OFF',
    }

    assert campaign.parse_emergency_stop_acknowledgement(
        json.dumps(expected)
    ) == expected
    assert campaign.parse_emergency_stop_acknowledgement(
        json.dumps({'emergency_stop': True, 'last_result': 'status requested'})
    ) is None
    assert campaign.parse_emergency_stop_acknowledgement(
        json.dumps({'emergency_stop': False, 'last_result': expected['last_result']})
    ) is None


@pytest.mark.parametrize(
    ('scenario_id', 'parameter', 'expected'),
    (
        ('F01', 'task_execution_authorization_path', 'corrupt_authorization.json'),
        ('F02', 'task_execution_promotion_manifest_path',
         'wrong_promotion_manifest.json'),
        ('F05', 'right_sensor_feedback_topic',
         '/room_315/fault/F05/right_sensor_blackhole'),
    ),
)
def test_fault_runtime_configs_are_derived_read_only_and_exact(
    tmp_path,
    monkeypatch,
    scenario_id,
    parameter,
    expected,
):
    source = tmp_path / 'task_runtime.yaml'
    _minimal_task_config(source)
    monkeypatch.setattr(campaign, 'TASK_RUNTIME_CONFIG', source)
    scenario = campaign.scenario_by_id(scenario_id)

    derived, inputs = campaign.create_fault_runtime_config(
        scenario, case_dir=tmp_path / 'case',
    )
    payload = yaml.safe_load(derived.read_text(encoding='utf-8'))
    parameters = payload['room_315_task_execution_node']['ros__parameters']

    assert parameters['execution_enabled'] is True
    assert expected in str(parameters[parameter])
    assert stat.S_IMODE(derived.stat().st_mode) == 0o444
    assert all(stat.S_IMODE(path.stat().st_mode) == 0o444 for path in inputs)


def test_fixed_artifact_contract_rejects_hash_drift(tmp_path, monkeypatch):
    artifact = tmp_path / 'artifact.bin'
    artifact.write_bytes(b'bound')
    artifact.chmod(0o444)
    monkeypatch.setattr(campaign, 'FIXED_ARTIFACTS', (
        campaign.FixedArtifact(
            'bound', artifact, len(b'bound'),
            hashlib.sha256(b'bound').hexdigest(), True,
        ),
    ))
    monkeypatch.setattr(campaign, 'verify_qualification_semantics', lambda: None)

    assert campaign.verify_fixed_artifacts()['status'] == 'passed'
    artifact.chmod(0o644)
    artifact.write_bytes(b'drift')
    artifact.chmod(0o444)
    with pytest.raises(campaign.QualificationError, match='artifact drift'):
        campaign.verify_fixed_artifacts()


def test_fixed_immutable_artifact_rejects_group_write(tmp_path, monkeypatch):
    artifact = tmp_path / 'artifact.bin'
    artifact.write_bytes(b'bound')
    artifact.chmod(0o464)
    monkeypatch.setattr(campaign, 'FIXED_ARTIFACTS', (
        campaign.FixedArtifact(
            'bound', artifact, len(b'bound'),
            hashlib.sha256(b'bound').hexdigest(), True,
        ),
    ))
    monkeypatch.setattr(campaign, 'verify_qualification_semantics', lambda: None)

    with pytest.raises(campaign.QualificationError, match='group- or world-writable'):
        campaign.verify_fixed_artifacts()


def test_source_lock_is_read_only_no_clobber_and_detects_source_drift(
    tmp_path,
    monkeypatch,
):
    relative = 'mfja_robot_control_config/scripts/runtime.py'
    source = tmp_path / relative
    source.parent.mkdir(parents=True)
    source.write_text('BOUND = True\n', encoding='utf-8')
    install_root = tmp_path / 'install'
    installed = (
        install_root / 'mfja_robot_control_config/lib/'
        'mfja_robot_control_config/runtime.py'
    )
    installed.parent.mkdir(parents=True)
    installed.write_text('BOUND = True\n', encoding='utf-8')
    monkeypatch.setattr(campaign, 'REPO', tmp_path)
    monkeypatch.setattr(campaign, 'INSTALL_ROOT', install_root)
    monkeypatch.setattr(campaign, 'CRITICAL_SOURCE_PATHS', (relative,))
    monkeypatch.setattr(campaign, 'FIXED_ARTIFACTS', ())
    monkeypatch.setattr(campaign, 'verify_qualification_semantics', lambda: None)
    lock = tmp_path / 'lock.json'

    payload = campaign.prepare_source_lock(lock)

    assert payload['source_count'] == 1
    assert stat.S_IMODE(lock.stat().st_mode) == 0o444
    assert campaign.verify_source_lock(lock)['source_lock_sha256'] == (
        campaign.sha256_file(lock)
    )
    with pytest.raises(campaign.QualificationError, match='overwrite'):
        campaign.prepare_source_lock(lock)
    source.write_text('BOUND = False\n', encoding='utf-8')
    with pytest.raises(campaign.QualificationError, match='source/install drift'):
        campaign.verify_source_lock(lock)


def test_source_lock_refuses_stale_installed_runtime(tmp_path, monkeypatch):
    relative = 'mfja_robot_control_config/scripts/runtime.py'
    source = tmp_path / relative
    source.parent.mkdir(parents=True)
    source.write_text('CURRENT = True\n', encoding='utf-8')
    install_root = tmp_path / 'install'
    installed = (
        install_root / 'mfja_robot_control_config/lib/'
        'mfja_robot_control_config/runtime.py'
    )
    installed.parent.mkdir(parents=True)
    installed.write_text('CURRENT = False\n', encoding='utf-8')
    monkeypatch.setattr(campaign, 'REPO', tmp_path)
    monkeypatch.setattr(campaign, 'INSTALL_ROOT', install_root)
    monkeypatch.setattr(campaign, 'CRITICAL_SOURCE_PATHS', (relative,))
    monkeypatch.setattr(campaign, 'FIXED_ARTIFACTS', ())
    monkeypatch.setattr(campaign, 'verify_qualification_semantics', lambda: None)

    with pytest.raises(campaign.QualificationError, match='source/install drift'):
        campaign.prepare_source_lock(tmp_path / 'lock.json')


@pytest.mark.parametrize('scenario_id', ('F01', 'F02'))
def test_bad_authority_or_manifest_refuses_gateway_and_proves_zero_motion(
    scenario_id,
):
    scenario = campaign.scenario_by_id(scenario_id)

    result = campaign.validate_fault_outcome(
        scenario,
        gateway_returncode=0,
        terminal_status=None,
        bag_audit=_audit(),
        stationary_proof=_stationary(),
        final_controller_state=_disabled_state(),
        gateway_refusal_reason='exact authorization/manifest rejection',
    )

    assert result['status'] == 'passed'
    assert result['gateway_refused'] is True
    assert result['actuating_command_count'] == 0
    assert result['zero_motion_proved'] is True


def test_corrupt_authorization_accepts_strict_field_set_rejection_evidence():
    scenario = campaign.scenario_by_id('F01')
    log_text = (
        'ValueError: task execution authorization has an incompatible field '
        "set: missing=['schema_version'], unexpected=['corrupt']\n"
    )

    reason = campaign.gateway_refusal_reason(scenario, log_text)

    assert reason == log_text.strip()


@pytest.mark.parametrize('terminal', ('failed', 'aborted', 'rejected'))
def test_planner_unavailable_is_non_success_with_zero_motion(terminal):
    scenario = campaign.scenario_by_id('F03')

    result = campaign.validate_fault_outcome(
        scenario,
        gateway_returncode=None,
        terminal_status=_non_success_status(terminal),
        bag_audit=_audit(),
        stationary_proof=_stationary(),
        final_controller_state=_disabled_state(),
    )

    assert result['status'] == 'passed'
    assert result['terminal_status'] == terminal
    assert result['false_success_count'] == 0
    assert result['actuating_command_count'] == 0


def test_feedback_loss_allows_motion_then_requires_fail_closed_abort():
    scenario = campaign.scenario_by_id('F05')

    result = campaign.validate_fault_outcome(
        scenario,
        gateway_returncode=None,
        terminal_status=_non_success_status('aborted'),
        bag_audit=_audit(actuating=1),
        stationary_proof=None,
        final_controller_state=_disabled_state(),
        feedback_unavailable_proof=_feedback_proof(),
    )

    assert result['status'] == 'passed'
    assert result['terminal_status'] == 'aborted'
    assert result['actuating_command_count'] == 1
    assert result['zero_motion_proved'] is False
    assert result['emergency_stop_injected'] is False
    assert result['feedback_unavailable_proved'] is True
    assert result['controller_final_mode'] == 'DISABLED'
    assert result['false_success_count'] == 0


def test_emergency_stop_requires_real_actuation_then_non_success_and_disabled():
    scenario = campaign.scenario_by_id('F04')

    result = campaign.validate_fault_outcome(
        scenario,
        gateway_returncode=None,
        terminal_status=_non_success_status('aborted'),
        bag_audit=_audit(actuating=1, estop=True),
        stationary_proof=None,
        final_controller_state=_disabled_state(),
    )

    assert result['status'] == 'passed'
    assert result['terminal_status'] == 'aborted'
    assert result['actuating_command_count'] == 1
    assert result['controller_final_mode'] == 'DISABLED'
    assert result['false_success_count'] == 0


@pytest.mark.parametrize(
    ('scenario_id', 'audit', 'state', 'message'),
    (
        ('F03', _audit(successes=1), _disabled_state(), 'false success'),
        ('F03', _audit(actuating=1), _disabled_state(), 'stationary'),
        ('F03', _audit(v3=1), _disabled_state(), 'V3 observation'),
        ('F03', _audit(wrong=1), _disabled_state(), 'wrong V4 provenance'),
        ('F04', _audit(actuating=1, estop=False), _disabled_state(),
         'injected e-stop'),
        ('F04', _audit(actuating=1, estop=True),
         {'mode': 'ENABLED'}, 'final mode'),
        ('F05', _audit(), _disabled_state(), 'deliberate long move'),
        ('F05', _audit(actuating=1, estop=True), _disabled_state(),
         'unexpectedly injected an e-stop'),
        ('F05', _audit(actuating=1), {'mode': 'ENABLED'}, 'final mode'),
    ),
)
def test_outcome_validator_rejects_false_or_incomplete_safety_evidence(
    scenario_id,
    audit,
    state,
    message,
):
    scenario = campaign.scenario_by_id(scenario_id)
    stationary = None if scenario.expect_actuating_command else _stationary()

    with pytest.raises(campaign.QualificationError, match=message):
        campaign.validate_fault_outcome(
            scenario,
            gateway_returncode=None,
            terminal_status=_non_success_status(),
            bag_audit=copy.deepcopy(audit),
            stationary_proof=stationary,
            final_controller_state=state,
            feedback_unavailable_proof=(
                _feedback_proof() if scenario_id == 'F05' else None
            ),
        )


def test_feedback_loss_refuses_missing_unavailability_evidence():
    with pytest.raises(
        campaign.QualificationError,
        match='feedback unavailability proof',
    ):
        campaign.validate_fault_outcome(
            campaign.scenario_by_id('F05'),
            gateway_returncode=None,
            terminal_status=_non_success_status('aborted'),
            bag_audit=_audit(actuating=1),
            stationary_proof=None,
            final_controller_state=_disabled_state(),
            feedback_unavailable_proof=None,
        )


def test_live_visual_accepts_only_fused_v4_exact_checkpoint_and_r4_presence():
    payload = {
        'accepted': True,
        'model_ready': True,
        'input_ready': True,
        'presence_ready': True,
        'state_fusion_ready': True,
        'schema_version': campaign.V4_SCHEMA,
        'checkpoint_sha256': campaign.V4_CHECKPOINT_SHA256,
        'stage': 'fused_observed_state',
        'stale': False,
        'validation_reasons': [],
        'accepted_frame_count': 3,
        'shuttles': [
            {'identity': identity,
             'presence_state': 'present' if identity == 'R4' else 'absent'}
            for identity in ('L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4')
        ],
    }

    assert campaign.validate_live_visual(payload)['present_identities'] == ['R4']
    for field, invalid in (
        ('schema_version', 'room315.visual_state.v3'),
        ('checkpoint_sha256', '0' * 64),
        ('stale', True),
    ):
        changed = copy.deepcopy(payload)
        changed[field] = invalid
        with pytest.raises(campaign.QualificationError):
            campaign.validate_live_visual(changed)


def test_stationary_proof_uses_position_and_disabled_mode():
    before = {'s': 1.25, 'mode': 'DISABLED'}
    after = {'s': 1.25, 'mode': 'DISABLED'}

    assert campaign.verify_stationary(before, after)['absolute_delta_s_m'] == 0.0
    with pytest.raises(campaign.QualificationError, match='zero-motion'):
        campaign.verify_stationary(before, {'s': 1.2501, 'mode': 'DISABLED'})
    with pytest.raises(campaign.QualificationError, match='DISABLED'):
        campaign.verify_stationary(before, {'s': 1.25, 'mode': 'ENABLED'})


def test_evidence_finalization_hashes_files_and_makes_tree_read_only(tmp_path):
    output = tmp_path / 'evidence'
    output.mkdir()
    (output / 'case.txt').write_text('evidence\n', encoding='utf-8')
    summary = {
        'schema_version': campaign.CAMPAIGN_SCHEMA,
        'status': 'passed',
        'physical_deployment': False,
    }

    campaign.finalise_evidence(output, summary)

    manifest = json.loads((output / 'manifest.json').read_text(encoding='utf-8'))
    assert manifest['campaign_status'] == 'passed'
    assert manifest['physical_deployment'] is False
    assert (output / 'SHA256SUMS').is_file()
    assert stat.S_IMODE(output.stat().st_mode) == 0o555
    assert stat.S_IMODE((output / 'summary.json').stat().st_mode) == 0o444
