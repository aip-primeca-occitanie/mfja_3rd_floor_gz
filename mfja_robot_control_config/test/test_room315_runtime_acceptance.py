import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from room_315_runtime_acceptance_readiness import gazebo_model_poses
from room_315_runtime_acceptance_readiness import scenario_expectation
from room_315_runtime_acceptance_readiness import scenario_launch_arguments
from room_315_runtime_acceptance_readiness import scene_entity_checks
from room_315_runtime_acceptance_report import REQUIRED_RECORD_FIELDS
from room_315_runtime_acceptance_report import build_report
from room_315_visual_state_inference_node import image_message_to_rgb8
from sensor_msgs.msg import Image


CANDIDATE = Path(
    '/home/tiago/room315_visual_runtime_candidate_experiment_a_full_'
    'seed31520260730_epoch24_4cb9cd88'
)


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
