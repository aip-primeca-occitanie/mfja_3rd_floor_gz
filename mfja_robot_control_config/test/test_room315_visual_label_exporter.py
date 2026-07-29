#!/usr/bin/env python3

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_label_exporter as exporter


def _candidate(sample_id, image_fingerprint, label_fingerprint):
    return exporter.ExportCandidate(
        row={'sample_id': sample_id},
        image_refs={},
        image_pair_fingerprint=image_fingerprint,
        label_fingerprint=label_fingerprint,
    )


def _oracle_event():
    return {
        'task': 'legacy language task that must not be exported',
        'pddl_problem': 'legacy-planning-problem',
        'scenario_family': 'visual_family',
        'model_input': {
            'overhead_images': {
                'left_rail_rgb': 'left.jpg',
                'right_rail_rgb': 'right.jpg',
            }
        },
        'privileged_eval': {
            'raw_shuttle_states': {
                'right': {
                    'room315_right_shuttle_1': {
                        'x': -13.4246,
                        'y': -3.4369,
                        'z': 0.8393,
                        'yaw': -0.0004,
                        'segment': 'A12E',
                        's': 0.9172,
                    }
                }
            },
            'payload_state': {
                'by_shuttle': {
                    'room315_right_shuttle_1': {'loaded': True},
                }
            },
            'expert_sensor_state': {
                'switch_states': {
                    'right': {'A1': 'EXTERIOR', 'A2': 'INTERIOR'},
                }
            },
        },
    }


def test_loads_overhead_camera_calibration_from_gazebo_model():
    cameras = exporter.load_camera_projections(exporter._default_camera_model_path())

    assert set(cameras) == {'left', 'right'}
    assert cameras['right'].position == pytest.approx((-14.9, -4.7, 3.95), abs=1e-5)
    assert cameras['left'].position == pytest.approx((-10.9, -4.7, 3.95), abs=1e-5)
    assert cameras['right'].width == 640
    assert cameras['right'].height == 480
    assert cameras['right'].focal_px == pytest.approx(493.792492, abs=1e-6)


def test_projects_rail_pose_into_expected_camera_bbox():
    cameras = exporter.load_camera_projections(exporter._default_camera_model_path())
    raw_state = {
        'x': -13.4246,
        'y': -3.4369,
        'z': 0.8393,
        'yaw': -0.0004,
        'segment': 'A12E',
    }

    gazebo_pose = exporter.rail_pose_to_gazebo('right', raw_state)
    bbox = exporter.shuttle_bbox(cameras['right'], gazebo_pose)

    assert gazebo_pose == pytest.approx(
        (-14.949983, -3.856033, 0.8393, 3.140063),
        abs=1e-5,
    )
    assert bbox == pytest.approx([160.288, 201.954, 50.362, 66.807], abs=0.01)


def test_shared_rail_calibration_supports_runtime_parameter_objects():
    calibration = exporter.RIGHT_CALIBRATION_DEFAULTS
    raw_pose = (-13.4246, -3.4369, 0.8393, -0.0004)

    from_mapping = exporter.apply_rail_pose_calibration(*raw_pose, calibration)
    from_runtime_object = exporter.apply_rail_pose_calibration(
        *raw_pose,
        SimpleNamespace(**calibration),
    )

    assert from_runtime_object == pytest.approx(from_mapping, abs=1e-12)


def test_builds_visual_labels_without_exposing_oracle_model_inputs():
    cameras = exporter.load_camera_projections(exporter._default_camera_model_path())
    labels = exporter.visual_labels_from_event(_oracle_event(), cameras)

    assert labels['schema_version'] == 'room315.visual_state.v3'
    assert labels['calibration_version'] == exporter.CALIBRATION_VERSION
    r1 = next(shuttle for shuttle in labels['shuttles'] if shuttle['id'] == 'R1')
    assert r1['location'] == {'block': 'A12E', 'side': 'right'}
    assert r1['loaded_state'] == 'loaded'
    assert r1['rail_position']['available'] is True
    assert r1['rail_position']['s_m'] == pytest.approx(0.9172)
    assert r1['rail_position']['s_ratio'] == pytest.approx(
        0.4118,
        abs=0.001,
    )
    assert labels['switches'] == [
        {'id': 'right:A1', 'state': 'exterior', 'confidence': 1.0},
        {'id': 'right:A2', 'state': 'interior', 'confidence': 1.0},
    ]


def test_exported_model_row_drops_legacy_task_and_planning_fields():
    cameras = exporter.load_camera_projections(exporter._default_camera_model_path())

    row, _refs = exporter._event_row(
        _oracle_event(),
        validation={'scenario_id': 'visual_family'},
        episode_id='episode_000001_visual_family',
        fallback_index=0,
        cameras=cameras,
    )

    assert set(row) == {
        'dataset_mode',
        'sample_id',
        'episode_id',
        'step_index',
        'scenario_family',
        'model_input',
        'visual_state_labels',
        'oracle_label_provenance',
    }
    assert set(row['model_input']) == {'overhead_images'}


def test_exact_pair_curation_deduplicates_and_rejects_label_conflicts():
    candidates = [
        _candidate('same:a', 'images-same', 'labels-same'),
        _candidate('same:b', 'images-same', 'labels-same'),
        _candidate('conflict:a', 'images-conflict', 'labels-a'),
        _candidate('conflict:b', 'images-conflict', 'labels-b'),
        _candidate('unique', 'images-unique', 'labels-unique'),
    ]

    retained, summary = exporter.curate_exact_image_pairs(candidates)

    assert [candidate.row['sample_id'] for candidate in retained] == ['same:a', 'unique']
    assert summary['source_candidates'] == 5
    assert summary['unique_image_pair_groups'] == 3
    assert summary['duplicate_rows_removed'] == 1
    assert summary['conflicting_groups_removed'] == 1
    assert summary['conflicting_rows_removed'] == 2
