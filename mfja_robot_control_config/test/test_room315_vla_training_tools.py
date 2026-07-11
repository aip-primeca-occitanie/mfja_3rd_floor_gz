#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path

import pytest
from PIL import Image


REPO_ROOT = Path(__file__).resolve().parents[2]
SPLIT_SCRIPT = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_vla_split_dataset.py'
TRAIN_SCRIPT = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_vla_train_local.py'
LEROBOT_SCRIPT = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_vla_to_lerobot.py'
METRICS_SCRIPT = (
    REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_legacy_direct_action_metrics.py'
)
SMOLVLA_EVAL_SCRIPT = (
    REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_smolvla_eval.py'
)


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _write_episode(tmp_path, *, family, speed, index):
    episode_id = f'episode_{index:06d}_{family}'
    problem = f'room315-{family}_speed{speed}'
    for camera in ('left_rail_rgb', 'right_rail_rgb'):
        image_dir = tmp_path / 'episodes' / episode_id / 'images' / camera
        image_dir.mkdir(parents=True)
        (image_dir / '000000.jpg').write_bytes(b'not-a-real-image-for-split-only')
    validation_dir = tmp_path / 'episodes' / episode_id
    validation_dir.mkdir(parents=True, exist_ok=True)
    (validation_dir / 'validation.json').write_text(
        json.dumps({
            'episode_id': episode_id,
            'approved_for_training': True,
            'task_success': True,
            'validation_status': 'approved',
        }),
        encoding='utf-8',
    )
    rows = [
        {
            'episode_id': episode_id,
            'step_index': 0,
            'task': f'move {family}',
            'pddl_problem': problem,
            'model_input': {
                'language': f'move {family}',
                'last_command': {'action': 'START'},
                'observable_state': {
                    'right': {
                        'sensors': {'DZI1R': 1},
                        'switches': {'A1': 'EXTERIOR'},
                        'stoppers': {'A1': 'open'},
                    },
                    'left': {
                        'sensors': {'DZI1L': 0},
                        'switches': {'A1': 'UNKNOWN'},
                        'stoppers': {'A1': 'unknown'},
                    },
                },
                'overhead_images': {
                    'left_rail_rgb': f'episodes/{episode_id}/images/left_rail_rgb/000000.jpg',
                    'right_rail_rgb': f'episodes/{episode_id}/images/right_rail_rgb/000000.jpg',
                },
            },
            'action_vector': [
                4.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.08,
                3.0,
                28.0,
                10.0,
                0.0,
            ],
        },
        {
            'episode_id': episode_id,
            'step_index': 1,
            'task': f'move {family}',
            'pddl_problem': None,
            'model_input': {
                'language': f'move {family}',
                'last_command': {'action': 'SHUTTLE_ON'},
                'observable_state': {'right': {'sensors': {'DZI1R': 1}}},
                'overhead_images': {},
            },
            'action_vector': [
                5.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                7.0,
                18.0,
                11.0,
                0.0,
            ],
        },
    ]
    return rows


def _make_dataset(tmp_path):
    rows = []
    index = 0
    for family in [f'case_{number}' for number in range(6)]:
        for speed in ('006', '008'):
            rows.extend(_write_episode(tmp_path, family=family, speed=speed, index=index))
            index += 1
    meta = tmp_path / 'meta'
    meta.mkdir()
    training_events = meta / 'training_events.jsonl'
    training_events.write_text(
        ''.join(json.dumps(row) + '\n' for row in rows),
        encoding='utf-8',
    )
    return training_events


def _model_input_row(*, overhead_images=None, language='move the right shuttle to slot 1'):
    return {
        'episode_id': 'episode_000001_test',
        'model_input': {
            'language': language,
            'overhead_images': overhead_images or {},
            'last_command': {},
            'observable_state': {},
        },
        'action_vector': [0.0] * 24,
    }


def _write_image(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.new('RGB', (4, 3), color=(10, 20, 30)).save(path)


def test_split_dataset_keeps_speed_variants_with_their_base_family(tmp_path):
    splitter = _load_module('room_315_vla_split_dataset', SPLIT_SCRIPT)
    training_events = _make_dataset(tmp_path)
    output_dir = tmp_path / 'splits'

    summary = splitter.split_dataset(
        training_events,
        output_dir,
        seed=5,
        val_families=1,
        test_families=1,
        overwrite=True,
    )

    assert summary['episodes'] == 12
    assert summary['families'] == 6
    assert summary['splits']['train']['episode_count'] == 8
    assert summary['splits']['val']['episode_count'] == 2
    assert summary['splits']['test']['episode_count'] == 2
    assert summary['source_fingerprint']['sha256']
    assert summary['row_fingerprint']
    assert summary['split_integrity']['row_count_matches'] is True
    assert summary['split_integrity']['families_disjoint'] is True
    assert summary['camera_completeness']['per_camera']['left_rail_rgb']['referenced_rows'] == 12
    assert summary['class_balance']['primitive_id']['4'] == 12
    family_to_split = {}
    for split_name, split_info in summary['splits'].items():
        for family in split_info['families']:
            assert family not in family_to_split
            family_to_split[family] = split_name
        assert (output_dir / split_info['file']).exists()

    for family, split_name in family_to_split.items():
        rows = [
            json.loads(line)
            for line in (output_dir / summary['splits'][split_name]['file']).read_text(
                encoding='utf-8'
            ).splitlines()
        ]
        family_rows = [
            row for row in rows
            if str(row.get('pddl_problem') or '').startswith(f'room315-{family}')
        ]
        assert len(family_rows) == 2


def test_training_helper_builds_vocab_state_vectorizer_and_target_stats():
    trainer = _load_module('room_315_vla_train_local', TRAIN_SCRIPT)
    rows = [
        {
            'task': 'move the loaded right shuttle to slot 3',
            'model_input': {
                'language': 'move the loaded right shuttle to slot 3',
                'overhead_images': {},
                'last_command': {'action': 'START'},
                'observable_state': {
                    'right': {
                        'sensors': {'DZI1R': 1},
                        'switches': {'A1': 'EXTERIOR'},
                    }
                },
            },
            'action_vector': [0.0] * 24,
        },
        {
            'task': 'move the loaded left shuttle to slot 1',
            'model_input': {
                'language': 'move the loaded left shuttle to slot 1',
                'overhead_images': {},
                'last_command': {'action': 'SHUTTLE_ON'},
                'observable_state': {
                    'left': {
                        'sensors': {'DZI1L': 1},
                        'switches': {'A1': 'INTERIOR'},
                    }
                },
            },
            'action_vector': [1.0] * 24,
        },
    ]

    vocab = trainer.build_vocab(rows, max_vocab_size=12)
    encoded = trainer.encode_text('move unknown_token', vocab, max_tokens=4)
    vectorizer = trainer.StateVectorizer.fit(rows)
    restored = trainer.StateVectorizer.from_json(vectorizer.to_json())
    state = vectorizer.transform(rows[0])
    restored_state = restored.transform(rows[0])
    mean, std = trainer.target_stats(rows)

    assert vocab['move'] > 1
    assert encoded.shape == (4,)
    assert encoded[1] == vocab[trainer.TEXT_UNK]
    assert state.shape == (vectorizer.dim,)
    assert restored_state.shape == (vectorizer.dim,)
    assert restored.numeric_keys == vectorizer.numeric_keys
    assert vectorizer.dim > 0
    assert mean.shape == (24,)
    assert std.shape == (24,)


def test_legacy_direct_action_metrics_legality_latency_and_family():
    metrics = _load_module('room_315_legacy_direct_action_metrics', METRICS_SCRIPT)
    fields = [
        'primitive_id',
        'side_id',
        'shuttle_index',
        'switch_mask_A1',
        'switch_mask_A2',
        'switch_mask_A3',
        'switch_mask_A4',
        'switch_value_A1',
        'switch_value_A2',
        'switch_value_A3',
        'switch_value_A4',
        'stopper_mask_A1',
        'stopper_mask_A2',
        'stopper_mask_A3',
        'stopper_mask_A4',
        'stopper_value_A1',
        'stopper_value_A2',
        'stopper_value_A3',
        'stopper_value_A4',
        'speed_mps',
        'wait_condition_id',
        'target_id',
        'reason_id',
        'coordination_mode',
    ]
    action = [
        4.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.08,
        7.0,
        28.0,
        10.0,
        0.0,
    ]
    decision = metrics.offline_supervisor_decision(action)
    summary = metrics.action_metrics(
        [
            {
                'true_raw': action,
                'pred_raw': action,
                'true_quantized': action,
                'pred_quantized': action,
                'task_family': 'right_loaded_case',
                'supervisor_decision': decision,
                'inference_latency_seconds': 0.002,
                'cycle_time_seconds': 0.005,
            }
        ],
        fields,
        speed_tolerance=0.015,
    )
    by_family = metrics.action_metrics_by_family(
        [
            {
                'true_raw': action,
                'pred_raw': action,
                'true_quantized': action,
                'pred_quantized': action,
                'task_family': 'right_loaded_case',
                'supervisor_decision': decision,
                'inference_latency_seconds': 0.002,
                'cycle_time_seconds': 0.005,
            }
        ],
        fields,
        speed_tolerance=0.015,
    )

    assert decision['accepted'] is True
    assert summary['action_schema_decode_success_rate'] == 1.0
    assert summary['action_schema_legality_rate'] == 1.0
    assert summary['supervisor_rejection_rate'] == 0.0
    assert summary['inference_latency_seconds']['p50'] == 0.002
    assert summary['cycle_time_seconds']['p95'] == 0.005
    assert by_family['right_loaded_case']['samples'] == 1


def test_legacy_direct_action_metrics_detects_leaked_vectorizer(tmp_path):
    metrics = _load_module('room_315_legacy_direct_action_metrics_leakage', METRICS_SCRIPT)
    vectorizer_path = tmp_path / 'state_vectorizer.json'
    vectorizer_path.write_text(
        json.dumps({'dim': 32, 'names': ['task.side_left', 'payload_present', 'step_index_norm']}),
        encoding='utf-8',
    )

    report = metrics.detect_vectorizer_leakage(vectorizer_path)

    assert report['checked'] is True
    assert report['comparison_valid'] is False
    assert report['leaked_features'] == ['payload_present', 'step_index_norm']


def test_lerobot_converter_state_vectorizer_round_trip():
    converter = _load_module('room_315_vla_to_lerobot', LEROBOT_SCRIPT)
    rows = [
        {
            'episode_id': 'episode_000002_case_b',
            'step_index': 1,
            'model_input': {
                'language': 'move right',
                'overhead_images': {},
                'last_command': {'action': 'START'},
                'observable_state': {
                    'right': {
                        'sensors': {'DZI1R': 1},
                        'switches': {'A1': 'EXTERIOR'},
                    }
                },
            },
            'action_vector': [0.0] * 24,
        },
        {
            'episode_id': 'episode_000001_case_a',
            'step_index': 0,
            'model_input': {
                'language': 'move left',
                'overhead_images': {},
                'last_command': {'action': 'SHUTTLE_ON'},
                'observable_state': {
                    'left': {
                        'sensors': {'DZI1L': 1},
                        'switches': {'A1': 'INTERIOR'},
                    }
                },
            },
            'action_vector': [1.0] * 24,
        },
    ]

    vectorizer = converter.Room315StateVectorizer.fit(rows)
    restored = converter.Room315StateVectorizer.from_json(vectorizer.to_json())
    transformed = restored.transform(rows[0])
    grouped = converter.group_rows_by_episode(rows)

    assert restored.dim == vectorizer.dim
    assert transformed.shape == (vectorizer.dim,)
    assert any(name.endswith('==exterior') for name in restored.names)
    assert [episode_id for episode_id, _ in grouped] == [
        'episode_000001_case_a',
        'episode_000002_case_b',
    ]


def test_lerobot_converter_compact32_keeps_route_blocker_signal():
    converter = _load_module('room_315_vla_to_lerobot', LEROBOT_SCRIPT)
    row = {
        'episode_id': 'episode_000001_route_blocker',
        'step_index': 5,
        'task': 'move the loaded right shuttle to slot 3',
        'payload_present': True,
        'model_input': {
            'language': 'move the loaded right shuttle to slot 3',
            'overhead_images': {},
            'last_command': {
                'primitive': 'SHUTTLE_ON',
                'side': 'right',
                'target_id': 'right_shuttle_1',
                'speed_mps': 0.08,
            },
            'observable_state': {
                'right': {
                    'sensors': {'DZI3R': 0, 'DA3IR': 1},
                    'switches': {
                        'A1': 'EXTERIOR',
                        'A2': 'EXTERIOR',
                        'A3': 'INTERIOR',
                        'A4': 'EXTERIOR',
                    },
                },
                'left': {
                    'sensors': {},
                    'switches': {'A1': 'UNKNOWN', 'A2': 'UNKNOWN', 'A3': 'UNKNOWN', 'A4': 'UNKNOWN'},
                },
            },
        },
        'action_vector': [0.0] * 24,
    }

    vectorizer = converter.fit_state_vectorizer([row], 'compact32')
    restored = converter.Room315CompactStateVectorizer.from_json(vectorizer.to_json())
    state = restored.transform(row)
    names = restored.names

    assert restored.dim == 32
    assert state.shape == (32,)
    assert state[names.index('right.route.A3.occupied')] == 1.0
    assert state[names.index('right.slot.A3.occupied')] == 0.0
    assert state[names.index('right.switch.A3.position')] == 1.0
    assert state[names.index('language.payload_hint')] == 1.0
    assert state[names.index('last.command_present')] == 1.0
    assert state[names.index('last.speed_norm')] > 0.0
    assert 'payload_present' not in names
    assert 'step_index_norm' not in names


def test_lerobot_converter_compact32_ignores_row_level_leakage_fields():
    converter = _load_module('room_315_vla_to_lerobot_leakage', LEROBOT_SCRIPT)
    row = _model_input_row(language='move the empty left shuttle to slot 1')
    row.update({
        'step_index': 99,
        'payload_present': True,
        'pddl_goal': 'loaded right_shuttle_4 at slot 4',
        'pddl_problem': 'room315-right-yaskawa-to-staubli',
    })

    vectorizer = converter.fit_state_vectorizer([row], 'compact32')
    state = vectorizer.transform(row)
    names = vectorizer.names

    assert state[names.index('task.side_left')] == 1.0
    assert state[names.index('task.target_slot_norm')] == pytest.approx(0.25)
    assert state[names.index('language.payload_hint')] == 0.0


def test_lerobot_converter_fails_on_missing_images_by_default(tmp_path):
    converter = _load_module('room_315_vla_to_lerobot_images', LEROBOT_SCRIPT)
    row = _model_input_row(overhead_images={
        'left_rail_rgb': 'missing_left.jpg',
        'right_rail_rgb': 'missing_right.jpg',
    })

    with pytest.raises(FileNotFoundError):
        converter.image_integrity_report([row], tmp_path)

    report = converter.image_integrity_report([row], tmp_path, allow_blank_images=True)
    blank = converter.load_image(
        row,
        tmp_path,
        'left_rail_rgb',
        image_width=4,
        image_height=3,
        allow_blank_images=True,
    )

    assert report['debug_blank_image_mode'] is True
    assert report['per_camera']['left_rail_rgb']['blank_substitutions'] == 1
    assert blank.size == (4, 3)


def test_local_trainer_fails_on_missing_images_by_default(tmp_path):
    trainer = _load_module('room_315_vla_train_local_images', TRAIN_SCRIPT)
    row = _model_input_row(overhead_images={
        'left_rail_rgb': 'missing_left.jpg',
        'right_rail_rgb': 'missing_right.jpg',
    })

    with pytest.raises(FileNotFoundError):
        trainer.image_integrity_report([row], tmp_path, split_name='train')
    with pytest.raises(FileNotFoundError):
        trainer.load_paired_images(row, tmp_path, width=4, height=3)

    report = trainer.image_integrity_report(
        [row],
        tmp_path,
        split_name='train',
        allow_blank_images=True,
    )
    image = trainer.load_paired_images(
        row,
        tmp_path,
        width=4,
        height=3,
        allow_blank_images=True,
    )

    assert report['debug_blank_image_mode'] is True
    assert image.shape == (6, 3, 4)
    assert image.sum() == 0.0


def test_image_reports_accept_readable_declared_images(tmp_path):
    converter = _load_module('room_315_vla_to_lerobot_readable_images', LEROBOT_SCRIPT)
    trainer = _load_module('room_315_vla_train_local_readable_images', TRAIN_SCRIPT)
    _write_image(tmp_path / 'left.jpg')
    _write_image(tmp_path / 'right.jpg')
    row = _model_input_row(overhead_images={
        'left_rail_rgb': 'left.jpg',
        'right_rail_rgb': 'right.jpg',
    })

    converter_report = converter.image_integrity_report([row], tmp_path)
    trainer_report = trainer.image_integrity_report([row], tmp_path, split_name='train')
    image = trainer.load_paired_images(row, tmp_path, width=4, height=3)

    assert converter_report['complete_rows'] == 1
    assert trainer_report['complete_rows'] == 1
    assert image.shape == (6, 3, 4)
    assert image.sum() > 0.0


def test_lerobot_converter_feature_schema_uses_room315_modalities():
    converter = _load_module('room_315_vla_to_lerobot', LEROBOT_SCRIPT)

    features = converter.build_lerobot_features(
        state_names=['sensor_a', 'switch_a==exterior'],
        action_names=['primitive_id', 'side_id'],
        image_height=240,
        image_width=320,
        use_videos=False,
    )

    assert features['observation.state']['shape'] == (2,)
    assert features['action']['shape'] == (2,)
    assert features['observation.images.left_rail_rgb']['dtype'] == 'image'
    assert features['observation.images.left_rail_rgb']['shape'] == (240, 320, 3)
    assert features['observation.images.right_rail_rgb']['dtype'] == 'image'


def test_smolvla_eval_quantizes_discrete_fields_and_summarises_metrics():
    evaluator = _load_module('room_315_smolvla_eval', SMOLVLA_EVAL_SCRIPT)
    action_space = {
        'action_vector_fields': [
            'primitive_id',
            'side_id',
            'shuttle_index',
            'speed_mps',
            'target_id',
        ],
        'primitive_ids': {'WAIT': 0, 'DONE': 1, 'SET_STOPPERS': 3, 'SHUTTLE_ON': 4},
        'side_ids': {'right': 0, 'left': 1},
        'target_ids': {'none': 0, 'A1': 1, 'right_shuttle_1': 28},
    }
    fields = evaluator.action_vector_fields(action_space)
    ranges = evaluator.action_field_ranges(action_space)
    true = [4.0, 1.0, 0.0, 0.08, 28.0]
    pred = [3.7, 0.6, -0.8, 0.071, 27.8]

    true_quantized = evaluator.quantize_action(true, fields, ranges)
    pred_quantized = evaluator.quantize_action(pred, fields, ranges)
    summary = evaluator.summarise_predictions(
        [
            {
                'true_raw': true,
                'pred_raw': pred,
                'true_quantized': true_quantized,
                'pred_quantized': pred_quantized,
            }
        ],
        fields,
        speed_tolerance=0.015,
    )

    assert pred_quantized.tolist() == pytest.approx([4.0, 1.0, -1.0, 0.071, 28.0])
    assert summary['primitive_accuracy'] == 1.0
    assert summary['side_accuracy'] == 1.0
    assert summary['target_id_accuracy'] == 1.0
    assert summary['speed_within_tolerance_accuracy'] == 1.0
    assert summary['exact_discrete_action_accuracy'] == 0.0
    assert summary['exact_action_accuracy'] == 0.0


def test_smolvla_eval_policy_batch_whitelists_model_inputs_only():
    evaluator = _load_module('room_315_smolvla_eval_batch', SMOLVLA_EVAL_SCRIPT)

    class FakeTensor:
        def __init__(self, value):
            self.value = value

        def unsqueeze(self, dim):
            return (self.value, dim)

    class FakeTorch:
        @staticmethod
        def is_tensor(value):
            return isinstance(value, FakeTensor)

    batch = evaluator._make_policy_batch(
        FakeTorch,
        {
            'task': 'move left',
            'observation.state': FakeTensor('state'),
            'observation.images.left_rail_rgb': FakeTensor('left'),
            'observation.images.right_rail_rgb': FakeTensor('right'),
            'action': FakeTensor('action'),
            'episode_index': FakeTensor('episode'),
            'frame_index': FakeTensor('frame'),
        },
    )

    assert set(batch) == {
        'task',
        'observation.state',
        'observation.images.left_rail_rgb',
        'observation.images.right_rail_rgb',
    }
    assert 'episode_index' not in batch
    assert 'frame_index' not in batch
    assert 'action' not in batch
