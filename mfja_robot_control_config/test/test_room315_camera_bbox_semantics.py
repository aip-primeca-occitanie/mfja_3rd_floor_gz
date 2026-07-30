#!/usr/bin/env python3

import json
import sys
from pathlib import Path

import pytest
from PIL import Image


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_arbitrary_subset_capture_v2 as capture
from room_315_visual_state_dataset import (
    VisualStateLabelVectorizer,
    VisualStateValidationError,
    camera_observation_for_shuttle,
    normalize_visual_state_labels,
)


PACKAGE_ROOT = Path(
    '/home/tiago/'
    'room315_arbitrary_subset_visual_2040_capture_v2_seed31520260729'
)


def _labels(*identities: str):
    shuttles = []
    for index, identity in enumerate(identities):
        side = 'left' if identity.startswith('L') else 'right'
        shuttles.append({
            'id': identity,
            'presence': True,
            'visually_available': True,
            'bbox': [
                20.0 + index * 15.0,
                30.0 + index * 10.0,
                12.0,
                14.0,
            ],
            'location': {'side': side, 'block': 'A1E'},
            'rail_position': {
                'available': True,
                's_m': 0.5,
                's_ratio': 0.5,
                'segment_length_m': 1.0,
                'position_uncertainty_m': 0.0,
            },
            'loaded_state': 'loaded' if index % 2 else 'empty',
            'confidence': 1.0,
        })
    return normalize_visual_state_labels({
        'visual_state_labels': {
            'schema_version': 'room315.visual_state.v3',
            'calibration_version': 'test',
            'confidence': 1.0,
            'shuttles': shuttles,
            'switches': [],
            'obstacles': [],
        },
    })


def _by_id(labels, identity):
    return next(
        shuttle for shuttle in labels['shuttles']
        if shuttle['id'] == identity
    )


def _render(tmp_path: Path, labels, camera: str):
    source = tmp_path / f'{camera}.png'
    target = tmp_path / f'{camera}_overlay.png'
    Image.new('RGB', (240, 180), color=(30, 30, 30)).save(source)
    metadata = capture._overlay(
        source,
        labels,
        target,
        camera,
    )
    assert target.is_file()
    return metadata


def test_l3_present_has_left_bbox_and_right_mask():
    shuttle = _by_id(_labels('L3'), 'L3')

    left = camera_observation_for_shuttle(shuttle, 'left_rail_rgb')
    right = camera_observation_for_shuttle(shuttle, 'right_rail_rgb')

    assert left['visual_available']
    assert left['bbox_target_mask'] == [1.0] * 4
    assert not right['visual_available']
    assert right['bbox'] == [0.0] * 4
    assert right['bbox_target_mask'] == [0.0] * 4


def test_r4_present_has_right_bbox_and_left_mask():
    shuttle = _by_id(_labels('R4'), 'R4')

    assert camera_observation_for_shuttle(
        shuttle,
        'right_rail_rgb',
    )['bbox_target_mask'] == [1.0] * 4
    assert camera_observation_for_shuttle(
        shuttle,
        'left_rail_rgb',
    )['bbox_target_mask'] == [0.0] * 4


@pytest.mark.parametrize(
    ('identities', 'left_ids', 'right_ids'),
    [
        (('L2', 'L4'), ['L2', 'L4'], []),
        (('R1', 'R3'), [], ['R1', 'R3']),
        (
            ('L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4'),
            ['L1', 'L2', 'L3', 'L4'],
            ['R1', 'R2', 'R3', 'R4'],
        ),
    ],
)
def test_renderer_uses_only_camera_compatible_boxes(
    tmp_path,
    identities,
    left_ids,
    right_ids,
):
    labels = _labels(*identities)

    left = _render(tmp_path, labels, 'left_rail_rgb')
    right = _render(tmp_path, labels, 'right_rail_rgb')

    assert left['rendered_identities'] == left_ids
    assert right['rendered_identities'] == right_ids
    assert left['rendered_rectangle_count'] <= 4
    assert right['rendered_rectangle_count'] <= 4


def test_absent_and_masked_zero_boxes_are_not_drawn(tmp_path):
    labels = _labels('L3')

    left = _render(tmp_path, labels, 'left_rail_rgb')
    right = _render(tmp_path, labels, 'right_rail_rgb')

    assert left['rendered_identities'] == ['L3']
    assert right['rendered_identities'] == []
    assert left['identity_camera_status']['L1'] == 'absent'
    assert right['identity_camera_status']['L3'] == (
        'not_applicable_to_camera'
    )


def test_global_presence_is_separate_from_camera_availability():
    labels = _labels('L3', 'R4')
    l3 = _by_id(labels, 'L3')
    r4 = _by_id(labels, 'R4')

    assert l3['presence'] and r4['presence']
    assert not camera_observation_for_shuttle(
        l3,
        'right_rail_rgb',
    )['visual_available']
    assert not camera_observation_for_shuttle(
        r4,
        'left_rail_rgb',
    )['visual_available']


def test_bbox_loss_mask_ignores_opposite_camera_identity():
    labels = _labels('L3', 'R4')
    vectorizer = VisualStateLabelVectorizer()
    target = vectorizer.transform(labels)
    prediction = list(target)
    left_mask = vectorizer.target_mask(
        labels,
        camera='left_rail_rgb',
    )
    r4_slot = list(capture.GLOBAL_IDENTITIES).index('R4')
    r4_bbox_indexes = [
        index
        for index, name in enumerate(vectorizer.names)
        if name.startswith(f'shuttles.{r4_slot}.bbox.')
    ]
    for index in r4_bbox_indexes:
        prediction[index] += 1000.0

    masked_bbox_loss = sum(
        abs(prediction[index] - target[index]) * left_mask[index]
        for index in r4_bbox_indexes
    )

    assert len(r4_bbox_indexes) == 4
    assert all(left_mask[index] == 0.0 for index in r4_bbox_indexes)
    assert masked_bbox_loss == 0.0


def test_explicit_opposite_camera_bbox_fails_closed():
    labels = _labels('R4')
    r4 = _by_id(labels, 'R4')
    r4['camera_observations']['left_rail_rgb'] = {
        'applicable': False,
        'visual_available': True,
        'bbox': [10.0, 10.0, 12.0, 14.0],
        'bbox_target_mask': [1.0] * 4,
    }

    with pytest.raises(
        VisualStateValidationError,
        match='opposite-rail',
    ):
        normalize_visual_state_labels(labels)


def test_masked_nonzero_bbox_and_mask_fail_closed():
    labels = _labels('L3')
    l3 = _by_id(labels, 'L3')
    right = l3['camera_observations']['right_rail_rgb']
    right['bbox'] = [0.0, 0.0, 1.0, 1.0]

    with pytest.raises(
        VisualStateValidationError,
        match='fully masked',
    ):
        normalize_visual_state_labels(labels)

    labels = _labels('L3')
    l3 = _by_id(labels, 'L3')
    l3['camera_observations']['right_rail_rgb'][
        'bbox_target_mask'
    ] = [1.0] * 4
    with pytest.raises(
        VisualStateValidationError,
        match='bbox_target_mask',
    ):
        normalize_visual_state_labels(labels)


def test_legacy_valid_label_derives_camera_semantics():
    labels = _labels('L3', 'R4')
    for shuttle in labels['shuttles']:
        shuttle.pop('bbox_camera', None)
        shuttle.pop('camera_observations', None)

    migrated = normalize_visual_state_labels(labels)

    assert camera_observation_for_shuttle(
        _by_id(migrated, 'L3'),
        'left_rail_rgb',
    )['visual_available']
    assert not camera_observation_for_shuttle(
        _by_id(migrated, 'L3'),
        'right_rail_rgb',
    )['visual_available']
    assert camera_observation_for_shuttle(
        _by_id(migrated, 'R4'),
        'right_rail_rgb',
    )['visual_available']


def test_corrected_canary_audit_and_gallery_pass():
    audit = json.loads(
        (PACKAGE_ROOT / 'camera_bbox_semantics_audit.json').read_text(
            encoding='utf-8',
        )
    )

    assert audit['passed']
    assert audit['checks']['maximum_four_boxes_per_camera']
    assert audit['checks']['masked_boxes_excluded_from_training_loss']
    assert audit['raw_label_metadata'][
        'missing_camera_observation_slot_count'
    ] == 0
    assert audit['before']['opposite_rail_bbox_violation_count'] == 247
    assert audit['after']['opposite_rail_bbox_violation_count'] == 0
    assert audit['after']['empty_placeholder_count'] == 0
    assert audit['gallery_validation']['render_counts_match']
    assert audit['gallery_validation']['rendered_identities_match']
