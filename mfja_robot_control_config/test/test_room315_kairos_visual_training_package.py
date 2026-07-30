#!/usr/bin/env python3

import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_kairos_visual_training_package as package


PACKAGE_ROOT = Path(
    '/home/tiago/'
    'room315_kairos_visual_state_training_v1_seed31520260730'
)


def _load_packaged_trainer():
    scripts = PACKAGE_ROOT / 'scripts'
    if str(scripts) not in sys.path:
        sys.path.insert(0, str(scripts))
    path = scripts / 'room_315_vla_train_local.py'
    spec = importlib.util.spec_from_file_location(
        'room315_packaged_trainer_test',
        path,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_frozen_package_verifies_and_contains_all_data():
    result = package.verify_package(PACKAGE_ROOT)

    assert result['passed']
    assert result['scenario_counts'] == {
        'train': 1528,
        'validation': 256,
        'test': 256,
    }
    assert result['configuration_counts'] == {
        'train': 191,
        'validation': 32,
        'test': 32,
    }
    assert result['unique_images'] == 4080
    assert result['checkpoint_sha256'] == package.CHECKPOINT_SHA256
    assert result['opposite_camera_bbox_loss_weight_sum'] == 0.0
    assert not result['model_input_prohibited_hits']
    assert not result['prediction_target_prohibited_hits']


def test_training_commands_fail_closed_on_test_files_and_unlock():
    trainer = _load_packaged_trainer()
    normal = SimpleNamespace(
        eval_checkpoint=None,
        train_file='train.jsonl',
        val_file='validation.jsonl',
        unlock_test=False,
        eval_splits='validation',
    )
    assert trainer.validate_test_lock(normal)['test_loaded'] is False

    with pytest.raises(ValueError, match='normal training'):
        trainer.validate_test_lock(SimpleNamespace(
            **{**vars(normal), 'train_file': 'test.jsonl'}
        ))
    with pytest.raises(ValueError, match='only for explicit'):
        trainer.validate_test_lock(SimpleNamespace(
            **{**vars(normal), 'unlock_test': True}
        ))


def test_kairos_training_preflight_does_not_open_test_data(monkeypatch):
    opened = []
    original = package._read_jsonl

    def guarded(path):
        opened.append(Path(path).name)
        if Path(path).name.startswith('test'):
            raise AssertionError(f'test data opened by preflight: {path}')
        return original(path)

    monkeypatch.setattr(package, '_read_jsonl', guarded)
    result = package.verify_training_preflight(PACKAGE_ROOT)

    assert result['passed']
    assert result['scenario_counts'] == {
        'train': 1528,
        'validation': 256,
    }
    assert result['unique_images'] == 3568
    assert result['test_files_read'] == []
    assert result['test_data_touched'] is False
    assert not any(name.startswith('test') for name in opened)


def test_test_evaluation_requires_explicit_unlock_and_is_not_run():
    trainer = _load_packaged_trainer()
    locked = SimpleNamespace(
        eval_checkpoint=Path('/tmp/checkpoint.pt'),
        train_file='train.jsonl',
        val_file='validation.jsonl',
        unlock_test=False,
        eval_splits='test',
    )
    with pytest.raises(ValueError, match='test evaluation is locked'):
        trainer.validate_test_lock(locked)
    unlocked = SimpleNamespace(**{**vars(locked), 'unlock_test': True})
    report = trainer.validate_test_lock(unlocked)

    assert report['test_loaded'] is True
    assert report['test_unlocked'] is True
    assert not (PACKAGE_ROOT / 'outputs').exists()
    manifest = json.loads(
        (PACKAGE_ROOT / 'package_manifest.json').read_text(encoding='utf-8')
    )
    assert manifest['immutable_payload']


def test_training_configuration_preserves_reviewed_core():
    config = json.loads(
        (PACKAGE_ROOT / 'config/full_training.json').read_text(
            encoding='utf-8'
        )
    )
    architecture = config['architecture']

    assert config['schema'] == 'room315.visual_state.v3'
    assert config['vector_dimension'] == 200
    assert architecture['pretrained_identifier'] == (
        'ResNet18_Weights.IMAGENET1K_V1'
    )
    assert architecture['adaptation'] == 'partial_finetune'
    assert architecture['trainable_backbone_scope'] == 'layer4'
    assert architecture['output_dimension'] == 200
    assert config['loss']['opposite_camera_bbox_loss'] == 0.0
    assert config['checkpoint_policy']['test_used_for_selection'] is False
    assert config['test_access'].startswith('locked')
