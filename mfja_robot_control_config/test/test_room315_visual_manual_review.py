#!/usr/bin/env python3

import copy
import hashlib
import json
import sys
from pathlib import Path

import pytest
from PIL import Image
from PIL import ImageDraw


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_manual_review as manual_review


GLOBAL_IDS = ('L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4')
CAMERAS = ('left_rail_rgb', 'right_rail_rgb')


def _write_jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        ''.join(json.dumps(row, sort_keys=True) + '\n' for row in rows),
        encoding='utf-8',
    )


def _scenario(
    scenario_id,
    left,
    right,
    *,
    rail_scope='arbitrary_identity_subset',
):
    active = tuple(left) + tuple(right)
    target = active[0] if active else 'L1'
    target_side = 'left' if target.startswith('L') else 'right'
    same_rail_neutral = [
        identity
        for identity in active
        if identity != target
        and identity.startswith(target[0])
    ]
    opposite_neutral = [
        identity
        for identity in active
        if not identity.startswith(target[0])
    ]

    def shuttle(identity):
        return {
            'id': identity,
            'loaded_state': (
                'loaded' if int(identity[1]) % 2 else 'empty'
            ),
            'start_position': {
                'segment': 'A1E',
                's_ratio': 0.25,
                'position_zone': 'ordinary',
            },
        }

    value = {
        'scenario_id': scenario_id,
        'scenario_family': 'no_relation_observation',
        'scene_type': 'no_relation_observation',
        'relation_family': 'no_relation_observation',
        'rail_scope': rail_scope,
        'presence_configuration_id': f'presence_{scenario_id}',
        'scene': {
            'rails': {
                'left': {'shuttles': [shuttle(identity) for identity in left]},
                'right': {
                    'shuttles': [shuttle(identity) for identity in right]
                },
            },
        },
        'relation_probe': {
            'target_shuttle_id': target,
            'side': target_side,
            'relations': [],
            'relation_neutral_shuttle_ids': same_rail_neutral,
            'opposite_rail_neutral_shuttle_ids': opposite_neutral,
        },
    }
    if rail_scope == 'arbitrary_identity_subset':
        value['left_active_identities'] = list(left)
        value['right_active_identities'] = list(right)
    return value


def _label(identity, present):
    side = 'left' if identity.startswith('L') else 'right'
    if present:
        return {
            'id': identity,
            'presence': True,
            'visually_available': True,
            'bbox': [5.0, 6.0, 24.0, 20.0],
            'location': {'side': side, 'block': 'A1E'},
            'rail_position': {
                'available': True,
                's_m': 0.5,
                's_ratio': 0.25,
                'segment_length_m': 2.0,
                'position_uncertainty_m': 0.0,
            },
            'loaded_state': (
                'loaded' if int(identity[1]) % 2 else 'empty'
            ),
            'confidence': 1.0,
        }
    return {
        'id': identity,
        'presence': False,
        'visually_available': False,
        'bbox': [0.0, 0.0, 0.0, 0.0],
        'location': {'side': side, 'block': 'unknown'},
        'rail_position': {
            'available': False,
            's_m': 0.0,
            's_ratio': 0.0,
            'segment_length_m': 0.0,
            'position_uncertainty_m': 0.0,
        },
        'loaded_state': 'unknown',
        'confidence': 1.0,
    }


def _write_nonblank_jpeg(path, color):
    path.parent.mkdir(parents=True, exist_ok=True)
    image = Image.new('RGB', (80, 60), color)
    draw = ImageDraw.Draw(image)
    draw.rectangle((4, 4, 42, 35), fill=(255, 255, 255))
    image.save(path, format='JPEG', quality=90)


def _package(tmp_path, scenarios, *, current_approval=True):
    root = tmp_path / 'smoke'
    _write_jsonl(root / 'scenario_manifest.jsonl', scenarios)
    events = []
    for index, scenario in enumerate(scenarios):
        active = {
            shuttle['id']
            for side in ('left', 'right')
            for shuttle in scenario['scene']['rails'][side]['shuttles']
        }
        image_refs = {}
        for camera_index, camera in enumerate(CAMERAS):
            reference = (
                f'episodes/{scenario["scenario_id"]}/images/'
                f'{camera}/frame_000000.jpg'
            )
            image_refs[camera] = reference
            _write_nonblank_jpeg(
                root / 'dataset' / reference,
                (
                    25 + index % 200,
                    40 + camera_index * 80,
                    80,
                ),
            )
        events.append({
            'episode_id': scenario['scenario_id'],
            'sample_id': scenario['scenario_id'],
            'step_index': 0,
            'model_input': {'overhead_images': image_refs},
            'visual_state_labels': {
                'schema_version': 'room315.visual_state.v3',
                'calibration_version': 'test',
                'confidence': 1.0,
                'shuttles': [
                    _label(identity, identity in active)
                    for identity in GLOBAL_IDS
                ],
                'switches': [],
                'obstacles': [],
            },
        })
    _write_jsonl(
        root / 'dataset' / 'meta' / 'training_events.jsonl',
        events,
    )
    if current_approval:
        (root / 'smoke_manual_approval.json').write_text(
            json.dumps({
                'schema_version': (
                    'room315.arbitrary_subset_smoke_approval.v1'
                ),
                'approved_for_smoke_capture': True,
                'approved_after_gallery_review': False,
                'approved_for_training': False,
                'reviewer': '',
                'reviewed_at': '',
            }),
            encoding='utf-8',
        )
    return root


def _image_hashes(root):
    return {
        path.relative_to(root): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted((root / 'dataset').rglob('*.jpg'))
    }


def _approved_template(package_root, scenario_ids):
    value = manual_review._approval_template(package_root, scenario_ids)
    value['reviewer'] = 'Human Reviewer'
    value['reviewed_at'] = '2026-07-29T12:00:00+02:00'
    for review in value['scenarios'].values():
        for field in manual_review.SCENARIO_REVIEW_BOOLEAN_FIELDS:
            review[field] = True
    for field in manual_review.AGGREGATE_REVIEW_BOOLEAN_FIELDS:
        value[field] = True
    return value


@pytest.mark.parametrize('scenario_count', [20, 96])
def test_scenario_count_is_derived_dynamically(tmp_path, scenario_count):
    path = tmp_path / f'{scenario_count}.jsonl'
    _write_jsonl(path, [
        {'scenario_id': f'scenario_{index:03d}'}
        for index in range(scenario_count)
    ])

    assert len(manual_review.read_scenario_manifest(path)) == scenario_count


def test_empty_manifest_fails(tmp_path):
    path = tmp_path / 'empty.jsonl'
    path.write_text('\n\n', encoding='utf-8')

    with pytest.raises(manual_review.ManualReviewError, match='empty'):
        manual_review.read_scenario_manifest(path)


def test_duplicate_manifest_ids_fail(tmp_path):
    path = tmp_path / 'duplicate.jsonl'
    _write_jsonl(path, [
        {'scenario_id': 'duplicate'},
        {'scenario_id': 'duplicate'},
    ])

    with pytest.raises(manual_review.ManualReviewError, match='duplicate'):
        manual_review.read_scenario_manifest(path)


def test_non_object_and_missing_manifest_id_fail(tmp_path):
    non_object = tmp_path / 'non_object.jsonl'
    non_object.write_text('[]\n', encoding='utf-8')
    missing_id = tmp_path / 'missing_id.jsonl'
    _write_jsonl(missing_id, [{'wrong': 'field'}])

    with pytest.raises(manual_review.ManualReviewError, match='object'):
        manual_review.read_scenario_manifest(non_object)
    with pytest.raises(manual_review.ManualReviewError, match='missing'):
        manual_review.read_scenario_manifest(missing_id)


def test_missing_and_unexpected_captured_ids_fail():
    with pytest.raises(
        manual_review.ManualReviewError,
        match=r"missing=\['b'\]",
    ):
        manual_review.validate_manifest_event_ids(['a', 'b'], ['a'])
    with pytest.raises(
        manual_review.ManualReviewError,
        match=r"unexpected=\['c'\]",
    ):
        manual_review.validate_manifest_event_ids(['a'], ['a', 'c'])


def test_duplicate_captured_event_ids_fail(tmp_path):
    path = tmp_path / 'events.jsonl'
    _write_jsonl(path, [
        {'episode_id': 'duplicate'},
        {'episode_id': 'duplicate'},
    ])

    with pytest.raises(manual_review.ManualReviewError, match='duplicate'):
        manual_review.read_captured_events(path)


def test_legacy_20_scenario_gallery_remains_supported(tmp_path):
    scenarios = [
        _scenario(
            f'legacy_{index:03d}',
            ('L1', 'L2', 'L3', 'L4'),
            (),
            rail_scope='left_four',
        )
        for index in range(20)
    ]
    root = _package(tmp_path, scenarios, current_approval=False)

    result = manual_review.write_gallery(root)
    gallery = json.loads(
        (root / 'manual_inspection_gallery_manifest.json').read_text()
    )

    assert result['scenario_count'] == 20
    assert result['source_image_count'] == 40
    assert result['overlay_image_count'] == 40
    assert gallery['exact_subset_validation'] is True
    assert all(
        row['left_active_identities'] == ['L1', 'L2', 'L3', 'L4']
        and row['right_active_identities'] == []
        for row in gallery['scenarios']
    )
    assert result['approval_path'].endswith(
        manual_review.LEGACY_APPROVAL_NAME
    )


def test_96_scenario_arbitrary_gallery_and_source_integrity(tmp_path):
    required = [
        (('L3',), ()),
        (('L2', 'L4'), ()),
        ((), ('R4',)),
        ((), ('R1', 'R3')),
        (('L1', 'L4'), ('R2',)),
        (('L2',), ('R1', 'R4')),
        (('L2', 'L4'), ('R2', 'R3')),
        (('L1', 'L2', 'L3', 'L4'), ('R1', 'R2', 'R3', 'R4')),
    ]
    subsets = required + [
        (('L1',), ('R1',)),
        (('L1', 'L3'), ('R2', 'R4')),
    ]
    scenarios = [
        _scenario(
            f'arbitrary_{index:03d}',
            *subsets[index % len(subsets)],
        )
        for index in range(96)
    ]
    root = _package(tmp_path, scenarios)
    approval_path = root / manual_review.CURRENT_APPROVAL_NAME
    approval_before = approval_path.read_bytes()
    hashes_before = _image_hashes(root)

    result = manual_review.write_gallery(root)
    hashes_after = _image_hashes(root)
    gallery = json.loads(
        (root / 'manual_inspection_gallery_manifest.json').read_text()
    )

    assert result['scenario_count'] == 96
    assert result['source_image_count'] == 192
    assert result['overlay_image_count'] == 192
    assert hashes_after == hashes_before
    assert gallery['source_images_unchanged'] is True
    assert gallery['modified_source_images'] == []
    assert gallery['summary']['total_scenarios'] == 96
    assert gallery['summary']['total_source_images'] == 192
    assert gallery['summary']['total_overlay_images'] == 192
    assert gallery['summary']['exact_subset_validation'] is True
    assert len(gallery['scenarios']) == 96
    assert approval_path.read_bytes() == approval_before
    assert not (root / manual_review.LEGACY_APPROVAL_NAME).exists()
    approval = json.loads(approval_path.read_text())
    assert approval['approved_after_gallery_review'] is False
    assert approval['approved_for_training'] is False


def test_missing_camera_image_fails(tmp_path):
    scenario = _scenario('missing_image', ('L3',), ())
    root = _package(tmp_path, [scenario])
    missing = next(
        (root / 'dataset').rglob('right_rail_rgb/frame_000000.jpg')
    )
    missing.unlink()

    with pytest.raises(manual_review.ManualReviewError, match='missing smoke image'):
        manual_review.build_gallery(root)


def test_legacy_rail_scopes_remain_supported():
    examples = {
        'left_four': (('L1', 'L2', 'L3', 'L4'), ()),
        'right_four': ((), ('R1', 'R2', 'R3', 'R4')),
        'dual_four_plus_four': (
            ('L1', 'L2', 'L3', 'L4'),
            ('R1', 'R2', 'R3', 'R4'),
        ),
    }
    for scope, expected in examples.items():
        scenario = _scenario('legacy', *expected, rail_scope=scope)
        assert manual_review.exact_active_membership(scenario) == expected


@pytest.mark.parametrize(
    ('left', 'right'),
    [
        (('L3',), ()),
        (('L2', 'L4'), ()),
        ((), ('R4',)),
        ((), ('R1', 'R3')),
        (('L1', 'L4'), ('R2',)),
        (('L2',), ('R1', 'R4')),
        (('L2', 'L4'), ('R2', 'R3')),
    ],
)
def test_arbitrary_identity_subsets_remain_exact(left, right):
    scenario = _scenario('arbitrary', left, right)

    assert manual_review.exact_active_membership(scenario) == (left, right)


def test_wrong_side_identity_fails():
    scenario = _scenario('wrong_side', ('L3',), ())
    scenario['right_active_identities'] = ['L3']
    scenario['scene']['rails']['right']['shuttles'] = [
        scenario['scene']['rails']['left']['shuttles'][0]
    ]
    scenario['left_active_identities'] = []
    scenario['scene']['rails']['left']['shuttles'] = []

    with pytest.raises(manual_review.ManualReviewError, match='wrong-side'):
        manual_review.exact_active_membership(scenario)


def test_duplicate_identity_fails():
    scenario = _scenario('duplicate', ('L2', 'L4'), ())
    scenario['left_active_identities'] = ['L2', 'L2']

    with pytest.raises(manual_review.ManualReviewError, match='duplicate'):
        manual_review.exact_active_membership(scenario)


def test_prefix_substitution_fails():
    scenario = _scenario('substitution', ('L2', 'L4'), ())
    scenario['scene']['rails']['left']['shuttles'] = [
        {'id': 'L1'},
        {'id': 'L2'},
    ]

    with pytest.raises(
        manual_review.ManualReviewError,
        match='prefix substitution',
    ):
        manual_review.exact_active_membership(scenario)


def test_unknown_rail_scope_fails():
    scenario = _scenario('unknown', ('L3',), ())
    scenario['rail_scope'] = 'unknown_scope'

    with pytest.raises(
        manual_review.ManualReviewError,
        match='unsupported rail scope',
    ):
        manual_review.exact_active_membership(scenario)


def test_all_empty_and_inactive_target_fail():
    empty = _scenario('empty', (), ())
    inactive_target = _scenario('inactive_target', ('L3',), ())
    inactive_target['relation_probe']['target_shuttle_id'] = 'L1'

    with pytest.raises(manual_review.ManualReviewError, match='all-empty'):
        manual_review.exact_active_membership(empty)
    with pytest.raises(
        manual_review.ManualReviewError,
        match='target identity is not active',
    ):
        manual_review.exact_active_membership(inactive_target)


def test_approval_template_starts_with_every_boolean_false():
    scenario_ids = [f'visual_{index:04d}' for index in range(1, 21)]
    package_root = Path('/tmp/example-smoke')

    value = manual_review._approval_template(package_root, scenario_ids)

    assert set(value['scenarios']) == set(scenario_ids)
    assert value['reviewer'] == ''
    assert value['reviewed_at'] == ''
    for review in value['scenarios'].values():
        assert all(
            review[field] is False
            for field in manual_review.SCENARIO_REVIEW_BOOLEAN_FIELDS
        )
    assert all(
        value[field] is False
        for field in manual_review.AGGREGATE_REVIEW_BOOLEAN_FIELDS
    )
    assert value['approved_for_full_capture'] is False


def test_manual_approval_uses_dynamic_manifest_ids(tmp_path):
    package_root = tmp_path / 'smoke'
    scenario_ids = [f'visual_{index:04d}' for index in range(96)]
    _write_jsonl(package_root / 'scenario_manifest.jsonl', [
        {'scenario_id': scenario_id}
        for scenario_id in scenario_ids
    ])
    approval_path = package_root / manual_review.LEGACY_APPROVAL_NAME
    approval_path.write_text(
        json.dumps(_approved_template(package_root, scenario_ids)),
        encoding='utf-8',
    )

    result = manual_review.validate_manual_approval(package_root)

    assert result['valid'] is True
    assert result['scenario_count'] == 96
    assert result['approved_for_full_capture'] is False


def test_competing_approval_files_fail(tmp_path):
    package_root = tmp_path / 'smoke'
    package_root.mkdir()
    (package_root / manual_review.CURRENT_APPROVAL_NAME).write_text(
        '{}',
        encoding='utf-8',
    )
    (package_root / manual_review.LEGACY_APPROVAL_NAME).write_text(
        '{}',
        encoding='utf-8',
    )

    with pytest.raises(
        manual_review.ManualReviewError,
        match='competing approval',
    ):
        manual_review.authoritative_approval_path(package_root)
