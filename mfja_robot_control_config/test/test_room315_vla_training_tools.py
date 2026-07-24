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
VISUAL_STATE_SCRIPT = (
    REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_visual_state_dataset.py'
)


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _write_image(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.new('RGB', (4, 3), color=(10, 20, 30)).save(path)


def _visual_labels(*, family='visual_case', shuttle='R1', loaded_state='loaded'):
    return {
        'schema_version': 'room315.visual_state.v1',
        'calibration_version': 'calib-2026-07',
        'scenario_family': family,
        'confidence': 0.95,
        'shuttles': [
            {
                'id': shuttle,
                'visually_available_identity': shuttle,
                'bbox': [1.0, 2.0, 10.0, 12.0],
                'location': {'side': 'right', 'slot': 'A1'},
                'loaded_state': loaded_state,
                'confidence': 0.9,
            }
        ],
        'switches': [
            {'id': 'right:A1', 'state': 'EXTERIOR', 'confidence': 0.8},
        ],
        'obstacles': [
            {
                'id': 'box',
                'bbox': [3.0, 4.0, 5.0, 6.0],
                'location': {'side': 'right', 'block': 'A12E'},
                'confidence': 0.7,
            }
        ],
    }


def _visual_row(tmp_path, *, episode_id, family, step=0, missing_images=False):
    left = f'episodes/{episode_id}/images/left_rail_rgb/000000.jpg'
    right = f'episodes/{episode_id}/images/right_rail_rgb/000000.jpg'
    if not missing_images:
        _write_image(tmp_path / left)
        _write_image(tmp_path / right)
    episode_dir = tmp_path / 'episodes' / episode_id
    episode_dir.mkdir(parents=True, exist_ok=True)
    (episode_dir / 'validation.json').write_text(
        json.dumps({
            'episode_id': episode_id,
            'approved_for_training': True,
            'capture_complete': True,
            'labels_valid': True,
            'validation_status': 'approved',
        }),
        encoding='utf-8',
    )
    return {
        'sample_id': f'{episode_id}:step:{step}',
        'episode_id': episode_id,
        'step_index': step,
        'scenario_family': family,
        'model_input': {
            'overhead_images': {
                'left_rail_rgb': left,
                'right_rail_rgb': right,
            },
        },
        'visual_state_labels': _visual_labels(family=family),
    }


def _write_visual_training_events(tmp_path, families=('visual_case_a', 'visual_case_b', 'visual_case_c')):
    rows = []
    index = 0
    for family in families:
        for speed in ('006', '008'):
            rows.append(
                _visual_row(
                    tmp_path,
                    episode_id=f'episode_{index:06d}_{family}_{speed}',
                    family=family,
                    step=0,
                )
            )
            index += 1
    meta = tmp_path / 'meta'
    meta.mkdir(exist_ok=True)
    training_events = meta / 'training_events.jsonl'
    training_events.write_text(
        ''.join(json.dumps(row) + '\n' for row in rows),
        encoding='utf-8',
    )
    return training_events


def test_split_dataset_writes_visual_inputs_and_separate_oracle_labels(tmp_path):
    splitter = _load_module('room_315_vla_split_dataset', SPLIT_SCRIPT)
    training_events = _write_visual_training_events(tmp_path)
    output_dir = tmp_path / 'visual_splits'

    summary = splitter.split_dataset(
        training_events,
        output_dir,
        seed=17,
        val_families=1,
        test_families=1,
        dataset_mode='visual_state',
        overwrite=True,
    )

    train_rows = [
        json.loads(line)
        for line in (output_dir / 'train.jsonl').read_text(encoding='utf-8').splitlines()
    ]
    train_labels = [
        json.loads(line)
        for line in (output_dir / 'train_visual_labels.jsonl').read_text(encoding='utf-8').splitlines()
    ]

    assert summary['dataset_mode'] == 'visual_state'
    assert summary['split_integrity']['families_disjoint'] is True
    assert summary['visual_state_model_input_integrity']['oracle_labels_physically_separate'] is True
    assert summary['camera_completeness']['complete_rows'] == 6
    assert summary['visual_state_class_balance']['loaded_state']['loaded'] == 6
    assert set(train_rows[0]) == {
        'dataset_mode',
        'sample_id',
        'episode_id',
        'step_index',
        'scenario_family',
        'model_input',
    }
    assert set(train_rows[0]['model_input']) == {'overhead_images'}
    assert 'visual_state_labels' not in train_rows[0]
    assert 'action_vector' not in json.dumps(train_rows)
    assert train_labels[0]['model_input_exposure'] == 'excluded'


def test_visual_state_rejects_model_input_label_leakage():
    visual = _load_module('room_315_visual_state_dataset', VISUAL_STATE_SCRIPT)
    row = {
        'sample_id': 'sample-1',
        'model_input': {
            'overhead_images': {},
            'visual_state_labels': {'bbox': [0, 0, 1, 1]},
        },
        'visual_state_labels': _visual_labels(),
    }

    with pytest.raises(visual.VisualStateValidationError, match='undeclared fields'):
        visual.validate_visual_model_input(row)


def test_missing_images_fail_by_default_and_blank_images_are_debug_only(tmp_path):
    converter = _load_module('room_315_vla_to_lerobot_visual_images', LEROBOT_SCRIPT)
    trainer = _load_module('room_315_vla_train_local_visual_images', TRAIN_SCRIPT)
    row = _visual_row(
        tmp_path,
        episode_id='episode_000001_missing_images',
        family='visual_missing',
        missing_images=True,
    )
    sanitized_row = {
        'sample_id': row['sample_id'],
        'episode_id': row['episode_id'],
        'model_input': {'overhead_images': row['model_input']['overhead_images']},
    }

    with pytest.raises(FileNotFoundError):
        converter.image_integrity_report([sanitized_row], tmp_path, dataset_mode='visual_state')
    with pytest.raises(FileNotFoundError):
        trainer.image_integrity_report(
            [sanitized_row],
            tmp_path,
            split_name='train',
            dataset_mode='visual_state',
        )

    report = converter.image_integrity_report(
        [sanitized_row],
        tmp_path,
        dataset_mode='visual_state',
        allow_blank_images=True,
    )
    image = trainer.load_paired_images(
        sanitized_row,
        tmp_path,
        width=4,
        height=3,
        allow_blank_images=True,
    )

    assert report['debug_blank_image_mode'] is True
    assert report['per_camera']['left_rail_rgb']['blank_substitutions'] == 1
    assert image.shape == (6, 3, 4)
    assert image.sum() == 0.0


def test_lerobot_converter_visual_state_mode_uses_label_targets_and_sidecar(tmp_path, monkeypatch):
    converter = _load_module('room_315_vla_to_lerobot_visual', LEROBOT_SCRIPT)
    row = _visual_row(
        tmp_path,
        episode_id='episode_000001_visual_case',
        family='visual_case',
        step=0,
    )
    model_row = {
        'dataset_mode': 'visual_state',
        'sample_id': row['sample_id'],
        'episode_id': row['episode_id'],
        'step_index': row['step_index'],
        'scenario_family': row['scenario_family'],
        'model_input': {'overhead_images': row['model_input']['overhead_images']},
    }
    label_row = {
        'sample_id': row['sample_id'],
        'episode_id': row['episode_id'],
        'step_index': row['step_index'],
        'visual_state_labels': row['visual_state_labels'],
        'model_input_exposure': 'excluded',
    }
    split_path = tmp_path / 'train.jsonl'
    labels_path = tmp_path / 'train_visual_labels.jsonl'
    split_path.write_text(json.dumps(model_row) + '\n', encoding='utf-8')
    labels_path.write_text(json.dumps(label_row) + '\n', encoding='utf-8')
    created = {}

    class FakeLeRobotDataset:
        @classmethod
        def create(cls, **kwargs):
            created['kwargs'] = kwargs
            return cls()

        def __init__(self):
            self.frames = []
            self.saved = 0
            self.finalized = False

        def add_frame(self, frame):
            self.frames.append(frame)

        def save_episode(self):
            self.saved += 1

        def finalize(self):
            self.finalized = True

    monkeypatch.setattr(converter, '_require_lerobot_dataset', lambda: FakeLeRobotDataset)

    summary = converter.convert_room315_to_lerobot(
        split_path,
        dataset_root=tmp_path,
        output_root=tmp_path / 'lerobot',
        name='room315_visual_state_train',
        repo_id='room315/visual_state_train',
        image_width=4,
        image_height=3,
        fps=10,
        dataset_mode='visual_state',
        visual_labels_path=labels_path,
        visual_label_vectorizer_out=tmp_path / 'visual_label_vectorizer.json',
        overwrite=True,
    )

    features = created['kwargs']['features']
    assert summary['dataset_mode'] == 'visual_state'
    assert summary['purpose'] == 'visual-state label dataset conversion; labels are not rail commands'
    assert summary['model_input_integrity']['oracle_labels_physically_separate'] is True
    assert features['action']['output_semantics'] == 'visual_state_labels_not_rail_commands'
    assert 'observation.state' not in features
    assert set(features) == {
        'action',
        'observation.images.left_rail_rgb',
        'observation.images.right_rail_rgb',
    }
    assert Path(summary['structured_visual_labels']).exists()
    assert Path(summary['visual_label_vectorizer']).exists()
    assert 'loaded_state' in json.dumps(summary['class_balance'])
    assert 'action_vector' not in json.dumps(summary)


def test_visual_state_vectorizer_round_trip_and_metrics():
    visual = _load_module('room_315_visual_state_dataset_metrics', VISUAL_STATE_SCRIPT)
    label = visual.normalize_visual_state_labels({'visual_state_labels': _visual_labels()})
    vectorizer = visual.VisualStateLabelVectorizer.fit([label])
    restored = visual.VisualStateLabelVectorizer.from_json(vectorizer.to_json())
    true_vector = restored.transform(label)
    pred_vector = list(true_vector)
    bbox_index = next(index for index, name in enumerate(restored.names) if '.bbox.0' in name)
    confidence_index = restored.names.index('confidence')
    pred_vector[bbox_index] += 2.0
    pred_vector[confidence_index] -= 0.05

    metrics = visual.visual_state_metrics(
        [{'true_raw': true_vector, 'pred_raw': pred_vector}],
        restored.names,
    )

    assert restored.dim == len(restored.names)
    assert any('loaded_state==loaded' in name for name in restored.names)
    assert metrics['dataset_mode'] == 'visual_state'
    assert metrics['bbox_mae'] > 0.0
    assert metrics['confidence_mae'] > 0.0
    assert metrics['loaded_state_accuracy'] == 1.0
    assert metrics['obstacle_metrics']['presence_accuracy'] == 1.0
    assert metrics['confidence_calibration']['mean_abs_calibration_error'] > 0.0


def test_visual_state_training_models_compare_frozen_backbone_and_lora():
    torch = pytest.importorskip('torch')
    trainer = _load_module('room_315_vla_train_local_visual_models', TRAIN_SCRIPT)

    frozen = trainer._build_visual_state_model(
        torch,
        output_dim=8,
        adaptation_mode='frozen_backbone',
        lora_rank=2,
    )
    lora = trainer._build_visual_state_model(
        torch,
        output_dim=8,
        adaptation_mode='lora',
        lora_rank=2,
    )
    image = torch.zeros((1, 6, 32, 32), dtype=torch.float32)
    assert frozen(image).shape == (1, 8)
    assert lora(image).shape == (1, 8)

    frozen_counts = trainer.parameter_report(frozen)
    lora_counts = trainer.parameter_report(lora)
    metadata = trainer.visual_state_model_metadata(
        adaptation_mode='lora',
        lora_rank=2,
        parameter_counts=lora_counts,
    )

    assert trainer.visual_adaptation_variants('compare') == ['frozen_backbone', 'lora']
    assert all(not parameter.requires_grad for parameter in frozen.backbone.parameters())
    assert all(not parameter.requires_grad for parameter in lora.backbone.parameters())
    assert lora_counts['trainable_parameters'] > frozen_counts['trainable_parameters']
    assert metadata['diffusion_policy_head'] is False
    assert metadata['direct_command_capability'] is False
    assert metadata['output_semantics'] == 'visual_facts_for_state_fusion_not_rail_commands'


def test_visual_state_compact_scene_is_provider_compatible_and_command_free():
    trainer = _load_module('room_315_vla_train_local_visual_compact', TRAIN_SCRIPT)
    provider = trainer._load_local_script_module('room_315_visual_observed_state_provider')
    scene = trainer.visual_label_to_provider_compact_scene(
        {'visual_state_labels': _visual_labels()},
        timestamp=12.0,
    )

    parsed = provider.StrictJsonCompactModelAdapter().parse(scene)
    lower = json.dumps(parsed, sort_keys=True).casefold()

    assert parsed['detections'][0]['identity'] == 'R1'
    assert 'pddl' not in lower
    assert 'rail_command' not in lower
    with pytest.raises(provider.VisualObservationError, match='forbidden key'):
        provider.StrictJsonCompactModelAdapter().parse({
            **scene,
            'detections': [
                {**scene['detections'][0], 'rail_command': {'action': 'shuttle', 'command': 'ON'}}
            ],
        })


def test_visual_state_plansys2_smoke_connects_provider_without_commands():
    trainer = _load_module('room_315_vla_train_local_visual_plansys2', TRAIN_SCRIPT)

    smoke = trainer.visual_state_plansys2_smoke()

    assert smoke['plansys2_problem_built'] is True
    assert smoke['visual_problem_uses_plansys2'] is True
    assert smoke['oracle_visual_goal_match'] is True
    assert smoke['direct_command_capability'] is False
    assert smoke['compact_command_payload_rejected'] is True
    assert smoke['command_like_visual_fact_count'] == 0
    assert smoke['published_commands'] == []
