import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from room_315_runtime_acceptance_readiness import gazebo_model_poses
from room_315_runtime_acceptance_readiness import gazebo_live_model_poses
from room_315_runtime_acceptance_readiness import query_gazebo_live_model_poses
from room_315_runtime_acceptance_readiness import scenario_expectation
from room_315_runtime_acceptance_readiness import scenario_launch_arguments
from room_315_runtime_acceptance_readiness import scene_entity_checks
from room_315_runtime_acceptance_readiness import V4_RAW_PREDICTION_SCHEMA
from room_315_runtime_acceptance_readiness import validate_raw_prediction_contract
from room_315_runtime_acceptance_recorder import (
    evaluate_observation_against_ground_truth,
)
from room_315_runtime_acceptance_report import REQUIRED_RECORD_FIELDS
from room_315_runtime_acceptance_report import build_report
from room_315_visual_state_inference_node import image_message_to_rgb8
from sensor_msgs.msg import Image


CANDIDATE = Path(
    '/home/tiago/room315_visual_runtime_candidate_experiment_a_full_'
    'seed31520260730_epoch24_4cb9cd88'
)


def _load_acceptance_launch_module():
    path = ROOT / 'launch/room_315_runtime_acceptance.launch.py'
    spec = importlib.util.spec_from_file_location(
        'room315_runtime_acceptance_launch_test', path
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _manifest():
    return json.loads((CANDIDATE / 'acceptance_scenarios.json').read_text())


def _scenario(manifest, scenario_id):
    return next(
        row for row in manifest['scenarios']
        if row['scenario_id'] == scenario_id
    )


def test_l4_loaded_launch_cardinality_matches_explicit_identities():
    manifest = _manifest()
    row = _scenario(manifest, 'accept_l4_loaded')
    arguments = scenario_launch_arguments(row)
    assert arguments == {
        'identity_selection_mode': 'explicit',
        'left_active_identities': 'L4',
        'right_active_identities': 'R1',
        'left_shuttle_count': '1',
        'right_shuttle_count': '1',
        'left_start_positions': 'A34E@0.200000000',
        'right_start_positions': 'A12E@0.200000000',
        'left_loaded_shuttles': 'L4',
        'right_loaded_shuttles': '',
    }


def test_l4_loaded_expectation_uses_authoritative_entities_and_payload():
    expectation = scenario_expectation(_manifest(), 'accept_l4_loaded')
    by_identity = {item.identity: item for item in expectation.shuttles}
    assert by_identity['L4'].entity_name == 'room315_left_shuttle_4'
    assert by_identity['L4'].loaded_state == 'loaded'
    assert by_identity['L4'].segment == 'A34E'
    assert by_identity['R1'].entity_name == 'room315_right_shuttle_1'
    assert by_identity['R1'].loaded_state == 'empty'


def test_empty_or_hidden_scene_cannot_pass_l4_readiness():
    expectation = scenario_expectation(_manifest(), 'accept_l4_loaded')
    empty = scene_entity_checks({}, expectation)
    assert not empty['ready']
    scene = '''
model { name: "room315_left_shuttle_4" pose { position { z: -10 } } }
model { name: "room315_right_shuttle_1" pose { position { z: -10 } } }
'''
    hidden = scene_entity_checks(gazebo_model_poses(scene), expectation)
    assert not hidden['ready']
    assert set(hidden['hidden_entities']) == {
        'room315_left_shuttle_4', 'room315_right_shuttle_1',
    }
    assert hidden['missing_payloads'] == ['room315_left_shuttle_4_payload']


def test_visible_l4_payload_and_r1_scene_passes_entity_readiness():
    expectation = scenario_expectation(_manifest(), 'accept_l4_loaded')
    scene = '''
model { name: "room315_left_shuttle_4" pose { position { x: 1.0 y: 2.0 z: 0.1 } } }
model { name: "room315_left_shuttle_4_payload" pose { position { z: 0.2 } } }
model { name: "room315_right_shuttle_1" pose { position { x: -1.0 z: 0.1 } } }
'''
    checks = scene_entity_checks(gazebo_model_poses(scene), expectation)
    assert checks['ready']
    assert not checks['missing_entities']
    assert not checks['hidden_entities']
    assert not checks['missing_payloads']
    assert not checks['unexpected_payloads']


def test_live_pose_info_parser_reads_pose_v_and_protobuf_zero_defaults():
    message = '''
header { stamp { sec: 12 nsec: 34 } }
pose {
  name: "room315_left_shuttle_4"
  id: 106
  position { x: -9.25 z: 0.5 }
  orientation { w: 1 }
}
pose {
  name: "room315_right_shuttle_1"
  id: 110
  position { y: 2.75 z: 5e-1 }
}
'''
    assert gazebo_live_model_poses(message) == {
        'room315_left_shuttle_4': {'x': -9.25, 'y': 0.0, 'z': 0.5},
        'room315_right_shuttle_1': {'x': 0.0, 'y': 2.75, 'z': 0.5},
    }
    assert gazebo_live_model_poses(
        'pose { name: "bad" position { z: nan } }',
    ) == {}


def test_live_pose_query_is_single_message_topic_read_without_shell():
    captured = {}

    def runner(command, **kwargs):
        captured['command'] = command
        captured['kwargs'] = kwargs
        return subprocess.CompletedProcess(
            command,
            0,
            stdout='pose { name: "entity" position { z: 0.5 } }',
            stderr='',
        )

    models, error = query_gazebo_live_model_poses(
        'room_315_only',
        environment={'GZ_PARTITION': 'acceptance_partition'},
        timeout_s=1.25,
        runner=runner,
    )
    assert error == ''
    assert models == {'entity': {'x': 0.0, 'y': 0.0, 'z': 0.5}}
    assert captured['command'] == [
        'gz', 'topic', '--echo',
        '--topic', '/world/room_315_only/pose/info',
        '--num', '1',
    ]
    assert captured['kwargs']['timeout'] == 1.25
    assert 'shell' not in captured['kwargs']


def test_live_pose_query_timeout_and_transport_failure_are_fail_closed():
    def timeout_runner(command, **kwargs):
        raise subprocess.TimeoutExpired(command, kwargs['timeout'])

    models, error = query_gazebo_live_model_poses(
        'room_315_only',
        environment={'GZ_PARTITION': 'acceptance_partition'},
        timeout_s=0.5,
        runner=timeout_runner,
    )
    assert models == {}
    assert error == 'live pose query timed out after 0.500s'

    def failed_runner(command, **kwargs):
        return subprocess.CompletedProcess(
            command, 1, stdout='', stderr='transport unavailable',
        )

    models, error = query_gazebo_live_model_poses(
        'room_315_only',
        environment={'GZ_PARTITION': 'acceptance_partition'},
        runner=failed_runner,
    )
    assert models == {}
    assert error == 'transport unavailable'


def test_stale_scene_info_snapshot_cannot_override_live_pose_evidence():
    expectation = scenario_expectation(_manifest(), 'accept_l4_loaded')
    stale_initial_scene = '''
model { name: "room315_left_shuttle_4" pose { position { z: 0.5 } } }
model { name: "room315_left_shuttle_4_payload" pose { position { z: 0.6 } } }
model { name: "room315_right_shuttle_1" pose { position { z: 0.5 } } }
'''
    assert scene_entity_checks(
        gazebo_model_poses(stale_initial_scene), expectation,
    )['ready']

    live_message = '''
pose { name: "room315_left_shuttle_4" position { z: -10 } }
pose { name: "room315_left_shuttle_4_payload" position { z: -9.9 } }
pose { name: "room315_right_shuttle_1" position { z: -10 } }
'''

    def runner(command, **kwargs):
        assert '/world/room_315_only/pose/info' in command
        assert not any('/scene/info' in value for value in command)
        return subprocess.CompletedProcess(
            command, 0, stdout=live_message, stderr='',
        )

    live_models, error = query_gazebo_live_model_poses(
        'room_315_only',
        environment={'GZ_PARTITION': 'acceptance_partition'},
        runner=runner,
    )
    assert error == ''
    live_checks = scene_entity_checks(live_models, expectation)
    assert not live_checks['ready']
    assert set(live_checks['hidden_entities']) == {
        'room315_left_shuttle_4', 'room315_right_shuttle_1',
    }


def test_failed_record_can_never_complete_or_approve_report():
    manifest = _manifest()
    row = _scenario(manifest, 'accept_l4_loaded')
    event = {
        'record_status': 'failed',
        'scenario_id': row['scenario_id'],
        'coverage': row['coverage'],
        'ground_truth': row['ground_truth'],
    }
    for field in REQUIRED_RECORD_FIELDS[1:]:
        event[field] = {'status': 'observed'}
    report = build_report(
        candidate_state={
            'candidate_id': 'candidate',
            'checkpoint_sha256': '4cb9cd88',
        },
        manifest=manifest,
        event_records=[event],
    )
    assert report['acceptance_status'] == 'incomplete'
    assert report['complete_scenario_count'] == 0
    assert report['failed_scenario_count'] == 1
    assert not report['automatic_deployment_approval']
    assert not report['approval']['approved']


def test_acceptance_launch_is_gated_and_never_starts_task_execution():
    source = (ROOT / 'launch/room_315_runtime_acceptance.launch.py').read_text()
    assert "'left_shuttle_count': str(len(left))" in source
    assert "'right_shuttle_count': str(len(right))" in source
    assert "readiness_node('world'" in source
    assert "readiness_node('scene'" in source
    assert "readiness_node('camera'" in source
    assert "readiness_node('runtime'" in source
    assert 'TimerAction' not in source
    assert 'room_315_task_execution.launch.py' not in source
    assert 'execution is forbidden' in source
    assert 'expected_raw_prediction_schema' in source
    assert '_require_v4_candidate_state(state)' in source
    assert "else 'room315.raw_model_prediction.v1'" not in source


def test_acceptance_launch_fails_closed_for_every_non_v4_candidate_schema():
    module = _load_acceptance_launch_module()
    accepted = module._require_v4_candidate_state({
        'schema_version': module.V4_CANDIDATE_STATE_SCHEMA,
    })
    assert accepted == module.V4_RAW_PREDICTION_SCHEMA

    invalid_states = (
        {'schema_version': 'room315.deployment_candidate_state.v3.v1'},
        {'schema_version': ''},
        {},
        [],
    )
    for state in invalid_states:
        with pytest.raises(RuntimeError):
            module._require_v4_candidate_state(state)


def test_readiness_default_raw_prediction_contract_is_v4():
    source = (
        ROOT / 'scripts/room_315_runtime_acceptance_readiness.py'
    ).read_text()
    assert V4_RAW_PREDICTION_SCHEMA == (
        'room315.visual_runtime_v4.diagnostic.v1'
    )
    assert (
        "'expected_raw_prediction_schema': V4_RAW_PREDICTION_SCHEMA"
        in source
    )


def _raw_prediction(schema_version):
    payload = {
        'schema_version': schema_version,
        'checkpoint_sha256': 'a' * 64,
        'output_dimension': 200,
        'denormalized_output': [0.0] * 200,
        'control_input': False,
    }
    if schema_version == 'room315.visual_runtime_v4.diagnostic.v1':
        payload.update({
            'model_schema_version': 'room315.visual_state.v4',
            'runtime_generation': 'v4',
            'runtime_mode': 'shadow',
            'acceptance_envelope': {'accepted': True},
        })
    return payload


def test_readiness_accepts_explicit_diagnostic_contracts_for_v3_and_v4():
    for schema in (
        'room315.raw_model_prediction.v1',
        'room315.visual_runtime_v4.diagnostic.v1',
    ):
        ready, evidence = validate_raw_prediction_contract(
            _raw_prediction(schema),
            expected_checkpoint_sha256='a' * 64,
            expected_schema_version=schema,
        )
        assert ready
        assert not evidence['errors']
        assert evidence['control_input'] is False


def test_v4_readiness_rejects_v3_schema_control_payload_and_nonfinite_vector():
    prediction = _raw_prediction('room315.visual_runtime_v4.diagnostic.v1')
    prediction['schema_version'] = 'room315.raw_model_prediction.v1'
    prediction['control_input'] = True
    prediction['denormalized_output'][17] = float('nan')
    ready, evidence = validate_raw_prediction_contract(
        prediction,
        expected_checkpoint_sha256='a' * 64,
        expected_schema_version='room315.visual_runtime_v4.diagnostic.v1',
    )
    assert not ready
    assert set(evidence['errors']) >= {
        'schema_version_mismatch',
        'raw_prediction_not_diagnostic_only',
        'output_vector_nonfinite_or_nonnumeric',
    }


def _accepted_observation_for(row):
    expected = {
        item['identity']: item
        for item in row['ground_truth']['shuttles']
    }
    return {
        'shuttles': [
            {
                'identity': identity,
                'presence_state': 'present' if identity in expected else 'absent',
                'visual_facts_valid': identity in expected,
                'side': expected.get(identity, {}).get('side', ''),
                'block': expected.get(identity, {}).get('segment', ''),
                'loaded_state': expected.get(identity, {}).get('loaded_state', ''),
                's_ratio': expected.get(identity, {}).get('s_ratio', 0.0),
            }
            for identity in ('L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4')
        ],
    }


def test_runtime_acceptance_requires_exact_segment_payload_and_position():
    row = _scenario(_manifest(), 'accept_l4_loaded')
    observation = _accepted_observation_for(row)
    passed = evaluate_observation_against_ground_truth(
        observation,
        row['ground_truth'],
        maximum_s_ratio_error=0.12,
    )
    assert passed['passed']
    assert not passed['errors']
    assert not passed['ground_truth_used_as_model_input']

    l4 = next(item for item in observation['shuttles'] if item['identity'] == 'L4')
    l4['block'] = 'A12E'
    l4['loaded_state'] = 'empty'
    l4['s_ratio'] = 0.35
    failed = evaluate_observation_against_ground_truth(
        observation,
        row['ground_truth'],
        maximum_s_ratio_error=0.12,
    )
    assert not failed['passed']
    assert any('segment_mismatch' in error for error in failed['errors'])
    assert any('loaded_state_mismatch' in error for error in failed['errors'])
    assert any('s_ratio_error' in error for error in failed['errors'])


def test_runtime_acceptance_rejects_missing_or_invented_visual_identity():
    row = _scenario(_manifest(), 'accept_sparse')
    observation = _accepted_observation_for(row)
    l3 = next(item for item in observation['shuttles'] if item['identity'] == 'L3')
    l3['visual_facts_valid'] = False
    r4 = next(item for item in observation['shuttles'] if item['identity'] == 'R4')
    r4['visual_facts_valid'] = True
    failed = evaluate_observation_against_ground_truth(
        observation,
        row['ground_truth'],
        maximum_s_ratio_error=0.12,
    )
    assert not failed['passed']
    assert any('visual_identity_set_mismatch' in error for error in failed['errors'])


def _image(*, encoding, width, height, step, data):
    message = Image()
    message.encoding = encoding
    message.width = width
    message.height = height
    message.step = step
    message.data = bytes(data)
    return message


def test_ros_rgb_image_decode_handles_row_padding_without_cv_bridge():
    message = _image(
        encoding='rgb8', width=2, height=2, step=8,
        data=[1, 2, 3, 4, 5, 6, 99, 99, 7, 8, 9, 10, 11, 12, 88, 88],
    )
    decoded = image_message_to_rgb8(message)
    assert decoded.shape == (2, 2, 3)
    assert decoded.tolist() == [
        [[1, 2, 3], [4, 5, 6]],
        [[7, 8, 9], [10, 11, 12]],
    ]


def test_ros_bgr_and_mono_image_decode_to_contiguous_rgb():
    bgr = image_message_to_rgb8(_image(
        encoding='bgr8', width=1, height=1, step=3, data=[3, 2, 1],
    ))
    mono = image_message_to_rgb8(_image(
        encoding='mono8', width=1, height=1, step=1, data=[7],
    ))
    assert bgr.tolist() == [[[1, 2, 3]]]
    assert mono.tolist() == [[[7, 7, 7]]]
    assert bgr.flags.c_contiguous
    assert mono.flags.c_contiguous
