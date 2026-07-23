#!/usr/bin/env python3
"""Small custom Room 315 visual-state trainer.

The learned model predicts structured visual facts for state fusion. It never
produces PDDL, plans, primitive commands, device commands, or rail commands.
"""

import argparse
import json
import math
import os
import random
import shutil
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_state_dataset import DATASET_MODES
from room_315_visual_state_dataset import DATASET_MODE_VISUAL_STATE
from room_315_visual_state_dataset import IMAGE_KEYS
from room_315_visual_state_dataset import VISUAL_LABEL_SUFFIX
from room_315_visual_state_dataset import VISUAL_STATE_NAMES
from room_315_visual_state_dataset import VISUAL_STATE_SCHEMA_VERSION
from room_315_visual_state_dataset import VisualStateLabelVectorizer
from room_315_visual_state_dataset import file_fingerprint as _file_fingerprint
from room_315_visual_state_dataset import image_integrity_report as _shared_image_integrity_report
from room_315_visual_state_dataset import iter_jsonl as _iter_jsonl
from room_315_visual_state_dataset import load_visual_labels_for_rows
from room_315_visual_state_dataset import missing_image_error as _missing_image_error
from room_315_visual_state_dataset import pretty_json as _pretty_json
from room_315_visual_state_dataset import resolve_image_path as _resolve_image_path
from room_315_visual_state_dataset import rows_fingerprint as _rows_fingerprint
from room_315_visual_state_dataset import safe_int as _safe_int
from room_315_visual_state_dataset import validate_visual_state_rows
from room_315_visual_state_dataset import visual_label_path_for_split
from room_315_visual_state_dataset import visual_model_input_image_refs
from room_315_visual_state_dataset import visual_state_class_balance
from room_315_visual_state_dataset import visual_state_metrics
from room_315_visual_state_dataset import visual_target_stats
from room_315_visual_state_smoke import load_local_script_module as _load_local_script_module
from room_315_visual_state_smoke import visual_label_to_provider_compact_scene
from room_315_visual_state_smoke import visual_state_plansys2_smoke


def _env_path(name: str, fallback: str) -> Path:
    return Path(os.environ.get(name, fallback))


DEFAULT_SPLITS_DIR = _env_path('ROOM315_VLA_SPLITS_DIR', 'room315_local_training/splits')
DEFAULT_OUTPUT_DIR = _env_path('ROOM315_VISUAL_STATE_OUTPUT_DIR', 'room315_local_training/checkpoints/visual_state')
DEFAULT_DATASET_ROOT = _env_path('ROOM315_VLA_DATASET_ROOT', 'room315_payload_dataset')
VISUAL_ADAPTATION_FROZEN_BACKBONE = 'frozen_backbone'
VISUAL_ADAPTATION_LORA = 'lora'
VISUAL_ADAPTATION_COMPARE = 'compare'
VISUAL_ADAPTATION_MODES = (
    VISUAL_ADAPTATION_FROZEN_BACKBONE,
    VISUAL_ADAPTATION_LORA,
    VISUAL_ADAPTATION_COMPARE,
)
VISUAL_MODEL_KIND = 'structured_visual_state_compact_backbone_v1'
TEXT_PAD = '<pad>'
TEXT_UNK = '<unk>'


def _require_torch():
    try:
        import torch
    except ImportError as exc:
        raise SystemExit(
            'PyTorch is required for local neural training but is not installed.\n'
            'Install it first, then rerun this command. For CUDA 12.8, start with:\n'
            '  python3 -m pip install --user torch --index-url https://download.pytorch.org/whl/cu128\n'
            'Or choose the current command from https://pytorch.org/get-started/locally/.'
        ) from exc
    return torch


def _load_manifest(splits_dir: Path) -> dict[str, Any]:
    manifest_path = splits_dir / 'split_manifest.json'
    if not manifest_path.exists():
        return {}
    with manifest_path.open('r', encoding='utf-8') as stream:
        parsed = json.load(stream)
    return parsed if isinstance(parsed, dict) else {}


def _load_json(path: Path) -> dict[str, Any]:
    with path.expanduser().open('r', encoding='utf-8') as stream:
        parsed = json.load(stream)
    return parsed if isinstance(parsed, dict) else {}


def _resolve_dataset_root(dataset_root: Path | None, splits_dir: Path) -> Path:
    if dataset_root is not None:
        return dataset_root.expanduser().resolve()
    manifest = _load_manifest(splits_dir)
    root = str(manifest.get('dataset_root') or '').strip()
    if root:
        return Path(root).expanduser().resolve()
    return DEFAULT_DATASET_ROOT.expanduser().resolve()


def _image_refs(row: dict[str, Any], *, dataset_mode: str = DATASET_MODE_VISUAL_STATE) -> dict[str, str]:
    _ = dataset_mode
    return visual_model_input_image_refs(row)


def _blank_image_tensor(*, width: int, height: int) -> np.ndarray:
    return np.zeros((3, height, width), dtype=np.float32)


def load_image_tensor(
    dataset_root: Path,
    image_ref: str,
    *,
    width: int,
    height: int,
    row: dict[str, Any] | None = None,
    image_key: str = '',
    allow_blank_images: bool = False,
) -> np.ndarray:
    if not image_ref:
        if allow_blank_images:
            return _blank_image_tensor(width=width, height=height)
        raise FileNotFoundError(
            _missing_image_error(row or {}, image_key, 'missing from model_input.overhead_images')
        )
    image_path = _resolve_image_path(dataset_root, image_ref)
    if not image_path.exists():
        if allow_blank_images:
            return _blank_image_tensor(width=width, height=height)
        raise FileNotFoundError(_missing_image_error(row or {}, image_key, 'missing on disk', image_ref))
    try:
        with Image.open(image_path) as image:
            rgb = image.convert('RGB').resize((width, height), Image.BILINEAR)
            array = np.asarray(rgb, dtype=np.float32) / 255.0
    except Exception as exc:
        if allow_blank_images:
            return _blank_image_tensor(width=width, height=height)
        raise RuntimeError(_missing_image_error(row or {}, image_key, 'unreadable', image_ref)) from exc
    return np.transpose(array, (2, 0, 1))


def load_paired_images(
    row: dict[str, Any],
    dataset_root: Path,
    *,
    width: int,
    height: int,
    allow_blank_images: bool = False,
    dataset_mode: str = DATASET_MODE_VISUAL_STATE,
) -> np.ndarray:
    refs = _image_refs(row, dataset_mode=dataset_mode)
    left = load_image_tensor(
        dataset_root,
        refs.get('left_rail_rgb', ''),
        width=width,
        height=height,
        row=row,
        image_key='left_rail_rgb',
        allow_blank_images=allow_blank_images,
    )
    right = load_image_tensor(
        dataset_root,
        refs.get('right_rail_rgb', ''),
        width=width,
        height=height,
        row=row,
        image_key='right_rail_rgb',
        allow_blank_images=allow_blank_images,
    )
    return np.concatenate([left, right], axis=0).astype(np.float32)


def image_integrity_report(
    rows: list[dict[str, Any]],
    dataset_root: Path,
    *,
    split_name: str,
    allow_blank_images: bool = False,
    dataset_mode: str = DATASET_MODE_VISUAL_STATE,
) -> dict[str, Any]:
    return _shared_image_integrity_report(
        rows,
        dataset_root,
        split_name=split_name,
        operation='training',
        allow_blank_images=allow_blank_images,
        dataset_mode=dataset_mode,
    )


def split_integrity_report(
    train_rows: list[dict[str, Any]],
    val_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    train_episodes = {str(row.get('episode_id') or '') for row in train_rows}
    val_episodes = {str(row.get('episode_id') or '') for row in val_rows}
    overlap = sorted((train_episodes & val_episodes) - {''})
    return {
        'train_rows': len(train_rows),
        'val_rows': len(val_rows),
        'train_episode_count': len(train_episodes - {''}),
        'val_episode_count': len(val_episodes - {''}),
        'episode_overlap': overlap,
        'disjoint_episodes': not overlap,
        'train_row_fingerprint': _rows_fingerprint(train_rows),
        'val_row_fingerprint': _rows_fingerprint(val_rows),
    }


class Room315VisualStateDataset:
    def __init__(
        self,
        rows: list[dict[str, Any]],
        labels: list[dict[str, Any]],
        *,
        dataset_root: Path,
        label_vectorizer: VisualStateLabelVectorizer,
        target_mean: np.ndarray,
        target_std: np.ndarray,
        max_tokens: int,
        image_width: int,
        image_height: int,
        torch_module: Any,
        allow_blank_images: bool = False,
    ) -> None:
        if len(rows) != len(labels):
            raise ValueError('visual-state rows and labels must have the same length')
        self.rows = rows
        self.labels = labels
        self.dataset_root = dataset_root
        self.label_vectorizer = label_vectorizer
        self.target_mean = target_mean
        self.target_std = target_std
        self.max_tokens = max_tokens
        self.image_width = image_width
        self.image_height = image_height
        self.torch = torch_module
        self.allow_blank_images = allow_blank_images

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int) -> dict[str, Any]:
        row = self.rows[index]
        target = np.asarray(self.label_vectorizer.transform(self.labels[index]), dtype=np.float32)
        normalized_target = (target - self.target_mean) / self.target_std
        return {
            'text': self.torch.zeros((self.max_tokens,), dtype=self.torch.long),
            'state': self.torch.zeros((len(VISUAL_STATE_NAMES),), dtype=self.torch.float32),
            'image': self.torch.as_tensor(
                load_paired_images(
                    row,
                    self.dataset_root,
                    width=self.image_width,
                    height=self.image_height,
                    allow_blank_images=self.allow_blank_images,
                    dataset_mode=DATASET_MODE_VISUAL_STATE,
                ),
                dtype=self.torch.float32,
            ),
            'target': self.torch.as_tensor(normalized_target, dtype=self.torch.float32),
            'raw_target': self.torch.as_tensor(target, dtype=self.torch.float32),
        }


def visual_adaptation_variants(mode: str) -> list[str]:
    mode = str(mode or VISUAL_ADAPTATION_FROZEN_BACKBONE).strip().lower()
    if mode == VISUAL_ADAPTATION_COMPARE:
        return [VISUAL_ADAPTATION_FROZEN_BACKBONE, VISUAL_ADAPTATION_LORA]
    if mode in {VISUAL_ADAPTATION_FROZEN_BACKBONE, VISUAL_ADAPTATION_LORA}:
        return [mode]
    raise ValueError(f'unsupported visual adaptation mode: {mode!r}')


def parameter_report(model: Any) -> dict[str, Any]:
    total = 0
    trainable = 0
    frozen = 0
    trainable_tensors: list[str] = []
    frozen_tensors: list[str] = []
    for name, parameter in model.named_parameters():
        count = int(parameter.numel())
        total += count
        if bool(parameter.requires_grad):
            trainable += count
            trainable_tensors.append(name)
        else:
            frozen += count
            frozen_tensors.append(name)
    return {
        'total_parameters': total,
        'trainable_parameters': trainable,
        'frozen_parameters': frozen,
        'trainable_fraction': round(trainable / max(1, total), 6),
        'trainable_tensors': trainable_tensors,
        'frozen_tensors': frozen_tensors,
    }


def _build_visual_state_model(
    torch_module: Any,
    *,
    output_dim: int,
    adaptation_mode: str,
    lora_rank: int,
):
    nn = torch_module.nn
    mode = visual_adaptation_variants(adaptation_mode)[0]
    rank = max(1, int(lora_rank))
    feature_dim = 128

    class VisualStateCompactBackbone(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.encoder = nn.Sequential(
                nn.Conv2d(6, 32, kernel_size=5, stride=2, padding=2),
                nn.BatchNorm2d(32),
                nn.SiLU(inplace=True),
                nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
                nn.BatchNorm2d(64),
                nn.SiLU(inplace=True),
                nn.Conv2d(64, feature_dim, kernel_size=3, stride=2, padding=1),
                nn.BatchNorm2d(feature_dim),
                nn.SiLU(inplace=True),
                nn.AdaptiveAvgPool2d((1, 1)),
                nn.Flatten(),
            )

        def forward(self, image):
            return self.encoder(image)

    class VisualStateBackboneWithOptionalLora(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.backbone = VisualStateCompactBackbone()
            for parameter in self.backbone.parameters():
                parameter.requires_grad = False
            self.adaptation_mode = mode
            if mode == VISUAL_ADAPTATION_LORA:
                self.lora_down = nn.Linear(feature_dim, rank, bias=False)
                self.lora_up = nn.Linear(rank, feature_dim, bias=False)
                nn.init.kaiming_uniform_(self.lora_down.weight, a=math.sqrt(5))
                nn.init.zeros_(self.lora_up.weight)
            else:
                self.lora_down = None
                self.lora_up = None
            self.head = nn.Sequential(
                nn.LayerNorm(feature_dim),
                nn.Linear(feature_dim, 128),
                nn.SiLU(inplace=True),
                nn.Dropout(0.05),
                nn.Linear(128, output_dim),
            )

        def forward(self, text, state, image):
            del text, state
            features = self.backbone(image)
            if self.lora_down is not None and self.lora_up is not None:
                features = features + self.lora_up(self.lora_down(features)) / float(rank)
            return self.head(features)

    return VisualStateBackboneWithOptionalLora()


def visual_state_model_metadata(
    *,
    adaptation_mode: str,
    lora_rank: int,
    parameter_counts: dict[str, Any] | None = None,
    pretrained_backbone: str | None = None,
    pretrained_backbone_report: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        'model_kind': VISUAL_MODEL_KIND,
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'adaptation_mode': adaptation_mode,
        'pretrained_backbone': pretrained_backbone or 'compact_pretrained_backbone_v1',
        'pretrained_backbone_report': pretrained_backbone_report or {},
        'backbone_trainable': False,
        'lora_rank': max(1, int(lora_rank)) if adaptation_mode == VISUAL_ADAPTATION_LORA else 0,
        'policy_head': 'linear_structured_visual_state_regression',
        'diffusion_policy_head': False,
        'output_semantics': 'visual_facts_for_state_fusion_not_rail_commands',
        'direct_command_capability': False,
        'parameter_counts': parameter_counts or {},
    }


def _load_visual_pretrained_backbone(
    torch_module: Any,
    model: Any,
    source: str | None,
) -> dict[str, Any]:
    source_text = str(source or '').strip()
    if not source_text or source_text == 'compact_pretrained_backbone_v1':
        return {
            'source': source_text or 'compact_pretrained_backbone_v1',
            'loaded': False,
            'source_kind': 'reported_compact_pretrained_backbone_label',
        }
    path = Path(source_text).expanduser()
    if not path.exists():
        if path.suffix in {'.pt', '.pth', '.ckpt'} or '/' in source_text:
            raise FileNotFoundError(f'visual pretrained backbone checkpoint not found: {path}')
        return {
            'source': source_text,
            'loaded': False,
            'source_kind': 'reported_external_backbone_label',
        }
    try:
        loaded = torch_module.load(path, map_location='cpu', weights_only=False)
    except TypeError:
        loaded = torch_module.load(path, map_location='cpu')
    raw_state = loaded
    if isinstance(loaded, dict):
        raw_state = (
            loaded.get('backbone_state_dict')
            or loaded.get('image_backbone_state_dict')
            or loaded.get('model_state_dict')
            or loaded
        )
    if not isinstance(raw_state, dict):
        raise ValueError(f'visual pretrained backbone checkpoint has no state dict: {path}')
    backbone_state: dict[str, Any] = {}
    for key, value in raw_state.items():
        key_text = str(key)
        if key_text.startswith('module.backbone.'):
            backbone_state[key_text.removeprefix('module.backbone.')] = value
        elif key_text.startswith('backbone.'):
            backbone_state[key_text.removeprefix('backbone.')] = value
        elif key_text.startswith('image_encoder.'):
            backbone_state[key_text.removeprefix('image_encoder.')] = value
        elif key_text.startswith('encoder.'):
            backbone_state[key_text] = value
    if not backbone_state:
        raise ValueError(f'visual pretrained backbone checkpoint has no backbone weights: {path}')
    missing, unexpected = model.backbone.load_state_dict(backbone_state, strict=False)
    for parameter in model.backbone.parameters():
        parameter.requires_grad = False
    return {
        'source': str(path.resolve()),
        'loaded': True,
        'source_kind': 'torch_state_dict',
        'missing_keys': list(missing),
        'unexpected_keys': list(unexpected),
    }


def _choose_device(torch_module: Any, requested: str) -> str:
    requested = str(requested or 'auto').strip().lower()
    if requested == 'auto':
        return 'cuda' if torch_module.cuda.is_available() else 'cpu'
    if requested == 'cuda' and not torch_module.cuda.is_available():
        raise SystemExit('CUDA was requested, but torch.cuda.is_available() is false.')
    return requested


def _set_seed(torch_module: Any, seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch_module.manual_seed(seed)
    if torch_module.cuda.is_available():
        torch_module.cuda.manual_seed_all(seed)


def _row_limit(rows: list[dict[str, Any]], limit: int | None) -> list[dict[str, Any]]:
    if limit is None or limit <= 0:
        return rows
    return rows[:limit]


def _evaluate_visual_state(
    torch_module: Any,
    model: Any,
    loader: Any,
    *,
    device: str,
    target_mean: np.ndarray,
    target_std: np.ndarray,
    label_names: list[str],
) -> dict[str, Any]:
    model.eval()
    loss_fn = torch_module.nn.SmoothL1Loss(reduction='sum')
    mean_tensor = torch_module.as_tensor(target_mean, dtype=torch_module.float32, device=device)
    std_tensor = torch_module.as_tensor(target_std, dtype=torch_module.float32, device=device)
    rows = 0
    loss_total = 0.0
    records: list[dict[str, Any]] = []
    with torch_module.no_grad():
        for batch in loader:
            text = batch['text'].to(device)
            state = batch['state'].to(device)
            image = batch['image'].to(device)
            target = batch['target'].to(device)
            raw_target = batch['raw_target'].to(device)
            pred_norm = model(text, state, image)
            loss_total += float(loss_fn(pred_norm, target).item())
            pred = pred_norm * std_tensor + mean_tensor
            batch_size = int(raw_target.shape[0])
            rows += batch_size
            for true_row, pred_row in zip(raw_target.detach().cpu().numpy(), pred.detach().cpu().numpy()):
                records.append({
                    'true_raw': true_row.astype(np.float32).tolist(),
                    'pred_raw': pred_row.astype(np.float32).tolist(),
                })
    metrics = visual_state_metrics(records, label_names)
    metrics['loss'] = round(loss_total / max(1, rows), 6)
    return metrics


def _save_checkpoint(
    torch_module: Any,
    path: Path,
    *,
    model: Any,
    optimizer: Any,
    epoch: int,
    metrics: dict[str, Any],
    config: dict[str, Any],
) -> None:
    torch_module.save(
        {
            'epoch': epoch,
            'model_state_dict': model.state_dict(),
            'optimizer_state_dict': optimizer.state_dict(),
            'metrics': metrics,
            'config': config,
        },
        path,
    )


def _torch_load_checkpoint(torch_module: Any, path: Path) -> dict[str, Any]:
    try:
        loaded = torch_module.load(path, map_location='cpu', weights_only=False)
    except TypeError:
        loaded = torch_module.load(path, map_location='cpu')
    if not isinstance(loaded, dict) or 'model_state_dict' not in loaded:
        raise ValueError(f'checkpoint does not contain a model_state_dict: {path}')
    return loaded


def _artifact_path(checkpoint: Path, name: str) -> Path:
    return checkpoint.expanduser().resolve().parent / name


def _load_target_stats(path: Path) -> tuple[np.ndarray, np.ndarray]:
    parsed = _load_json(path)
    mean = np.asarray(parsed.get('mean'), dtype=np.float32)
    std = np.asarray(parsed.get('std'), dtype=np.float32)
    if mean.ndim != 1 or std.ndim != 1 or mean.shape != std.shape:
        raise ValueError(f'target stats must contain same-length mean/std arrays: {path}')
    std = std.copy()
    std[std < 1e-6] = 1.0
    return mean, std


def _checkpoint_training_config(checkpoint: dict[str, Any], checkpoint_path: Path) -> dict[str, Any]:
    config = checkpoint.get('config')
    if isinstance(config, dict):
        return dict(config)
    path = _artifact_path(checkpoint_path, 'training_config.json')
    return _load_json(path) if path.exists() else {}


def _peak_gpu_memory(torch_module: Any, device: str) -> dict[str, Any]:
    if device != 'cuda' or not torch_module.cuda.is_available():
        return {
            'device': device,
            'cuda_available': bool(torch_module.cuda.is_available()),
            'allocated_bytes': None,
            'reserved_bytes': None,
        }
    return {
        'device': device,
        'cuda_available': True,
        'allocated_bytes': int(torch_module.cuda.max_memory_allocated()),
        'reserved_bytes': int(torch_module.cuda.max_memory_reserved()),
    }


def _visual_labels_path_for(splits_dir: Path, split_file: str, override: Path | None = None) -> Path:
    if override is not None:
        return override.expanduser().resolve()
    return visual_label_path_for_split(splits_dir / split_file).expanduser().resolve()


def _load_visual_split(
    splits_dir: Path,
    split_file: str,
    labels_path: Path,
    *,
    limit: int | None,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    rows = _row_limit(_iter_jsonl(splits_dir / split_file), limit)
    if not rows:
        raise ValueError(f'{split_file} split is empty')
    labels = load_visual_labels_for_rows(rows, labels_path)
    integrity = validate_visual_state_rows(rows, labels_path)
    return rows, labels, integrity


def _save_visual_label_vectorizer(vectorizer: VisualStateLabelVectorizer, path: Path) -> None:
    path.write_text(_pretty_json(vectorizer.to_json()) + '\n', encoding='utf-8')


def _write_visual_training_artifacts(
    output_dir: Path,
    *,
    dataset_report: dict[str, Any],
    vocab: dict[str, int],
    label_vectorizer: VisualStateLabelVectorizer,
    target_mean: np.ndarray,
    target_std: np.ndarray,
    config: dict[str, Any],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'dataset_report.json').write_text(
        _pretty_json(dataset_report) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'vocab.json').write_text(_pretty_json(vocab) + '\n', encoding='utf-8')
    _save_visual_label_vectorizer(label_vectorizer, output_dir / 'visual_label_vectorizer.json')
    (output_dir / 'state_vectorizer.json').write_text(
        _pretty_json({
            'kind': 'room315_visual_state_constant_state',
            'dataset_mode': DATASET_MODE_VISUAL_STATE,
            'names': list(VISUAL_STATE_NAMES),
            'dim': len(VISUAL_STATE_NAMES),
            'model_input_exposure': 'constant_placeholder_not_privileged_state',
        }) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'target_stats.json').write_text(
        _pretty_json({
            'mean': target_mean.tolist(),
            'std': target_std.tolist(),
        }) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'training_config.json').write_text(
        _pretty_json(config) + '\n',
        encoding='utf-8',
    )


def _visual_model_from_config(
    torch_module: Any,
    *,
    config: dict[str, Any],
    output_dim: int,
    vocab_size: int,
) -> tuple[Any, dict[str, Any]]:
    if config.get('visual_model_kind') != VISUAL_MODEL_KIND:
        raise ValueError(
            f'checkpoint is not a {VISUAL_MODEL_KIND} visual-state model; '
            'non-visual-state checkpoints are not supported'
        )
    adaptation_mode = str(
        config.get('visual_adaptation')
        or VISUAL_ADAPTATION_FROZEN_BACKBONE
    )
    lora_rank = _safe_int(config.get('visual_lora_rank'), 4)
    model = _build_visual_state_model(
        torch_module,
        output_dim=output_dim,
        adaptation_mode=adaptation_mode,
        lora_rank=lora_rank,
    )
    metadata = visual_state_model_metadata(
        adaptation_mode=adaptation_mode,
        lora_rank=lora_rank,
        pretrained_backbone=str(config.get('visual_pretrained_backbone') or ''),
        pretrained_backbone_report=dict(config.get('visual_pretrained_backbone_report') or {}),
    )
    return model, metadata


def _select_visual_adaptation_variant(
    variant_results: dict[str, Any],
    *,
    requested: str,
) -> str:
    requested = str(requested or 'best_val_loss').strip().lower()
    if requested in variant_results:
        return requested
    if requested != 'best_val_loss':
        raise ValueError(f'cannot select unknown visual adaptation {requested!r}')
    return min(
        variant_results,
        key=lambda name: (
            float(variant_results[name].get('best_val_loss', float('inf'))),
            name,
        ),
    )


def train_visual_state(args: argparse.Namespace) -> dict[str, Any]:
    torch_module = _require_torch()
    splits_dir = args.splits_dir.expanduser().resolve()
    dataset_root = _resolve_dataset_root(args.dataset_root, splits_dir)
    train_labels_path = _visual_labels_path_for(splits_dir, args.train_file, args.visual_train_labels)
    val_labels_path = _visual_labels_path_for(splits_dir, args.val_file, args.visual_val_labels)
    train_rows, train_labels, train_input = _load_visual_split(
        splits_dir,
        args.train_file,
        train_labels_path,
        limit=args.limit_train_rows,
    )
    val_rows, val_labels, val_input = _load_visual_split(
        splits_dir,
        args.val_file,
        val_labels_path,
        limit=args.limit_val_rows,
    )
    train_image_integrity = image_integrity_report(
        train_rows,
        dataset_root,
        split_name='train',
        allow_blank_images=args.allow_blank_images,
        dataset_mode=DATASET_MODE_VISUAL_STATE,
    )
    val_image_integrity = image_integrity_report(
        val_rows,
        dataset_root,
        split_name='val',
        allow_blank_images=args.allow_blank_images,
        dataset_mode=DATASET_MODE_VISUAL_STATE,
    )

    _set_seed(torch_module, args.seed)
    device = _choose_device(torch_module, args.device)
    label_vectorizer = VisualStateLabelVectorizer.fit(train_labels)
    target_mean_list, target_std_list = visual_target_stats(train_labels, label_vectorizer)
    target_mean = np.asarray(target_mean_list, dtype=np.float32)
    target_std = np.asarray(target_std_list, dtype=np.float32)
    output_dim = int(target_mean.shape[0])
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    vocab = {TEXT_PAD: 0, TEXT_UNK: 1}
    dataset_report = {
        'tool': 'room_315_vla_train_local',
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'purpose': 'small custom visual-state label predictor; outputs are not rail commands',
        'source_files': {
            'train': _file_fingerprint(splits_dir / args.train_file),
            'val': _file_fingerprint(splits_dir / args.val_file),
            'train_labels': _file_fingerprint(train_labels_path),
            'val_labels': _file_fingerprint(val_labels_path),
        },
        'row_fingerprint': {
            'train': _rows_fingerprint(train_rows),
            'val': _rows_fingerprint(val_rows),
            'train_labels': _rows_fingerprint([
                {'visual_state_labels': label}
                for label in train_labels
            ]),
            'val_labels': _rows_fingerprint([
                {'visual_state_labels': label}
                for label in val_labels
            ]),
        },
        'model_input_integrity': {
            'train': train_input,
            'val': val_input,
        },
        'camera_completeness': {
            'train': train_image_integrity,
            'val': val_image_integrity,
        },
        'class_balance': {
            'train': visual_state_class_balance(train_labels),
            'val': visual_state_class_balance(val_labels),
        },
        'split_integrity': split_integrity_report(train_rows, val_rows),
        'feature_purity': {
            'production_features_from': 'model_input.overhead_images',
            'oracle_labels_physically_separate_from_model_input': True,
            'row_level_metadata_used_as_features': [],
            'debug_or_ablation_features': ['allow_blank_images'] if args.allow_blank_images else [],
            'learned_output_boundary': 'visual facts only; no PDDL, plans, primitives, or rail commands',
        },
        'schema': {
            'visual_state_schema_version': VISUAL_STATE_SCHEMA_VERSION,
            'label_vector_names': label_vectorizer.names,
        },
    }
    config = {
        'tool': 'room_315_vla_train_local',
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'purpose': 'small custom visual-state label predictor; outputs are not rail commands',
        'dataset_root': str(dataset_root),
        'splits_dir': str(splits_dir),
        'train_file': args.train_file,
        'val_file': args.val_file,
        'train_labels': str(train_labels_path),
        'val_labels': str(val_labels_path),
        'seed': args.seed,
        'device': device,
        'epochs': args.epochs,
        'batch_size': args.batch_size,
        'learning_rate': args.learning_rate,
        'weight_decay': args.weight_decay,
        'image_width': args.image_width,
        'image_height': args.image_height,
        'max_tokens': args.max_tokens,
        'train_rows': len(train_rows),
        'val_rows': len(val_rows),
        'output_dim': output_dim,
        'state_dim': len(VISUAL_STATE_NAMES),
        'vocab_size': len(vocab),
        'allow_blank_images': bool(args.allow_blank_images),
        'debug_blank_image_mode': bool(args.allow_blank_images),
        'production_feature_source': 'model_input.overhead_images only',
        'visual_model_kind': VISUAL_MODEL_KIND,
        'visual_adaptation_request': args.visual_adaptation,
        'visual_lora_rank': args.visual_lora_rank,
        'visual_selected_adaptation': args.visual_selected_adaptation,
        'visual_pretrained_backbone': args.visual_pretrained_backbone,
        'diffusion_policy_head': False,
        'learned_output_boundary': 'visual facts only; cannot publish rail commands',
        'dataset_report': str(output_dir / 'dataset_report.json'),
    }

    (output_dir / 'dataset_report.json').write_text(_pretty_json(dataset_report) + '\n', encoding='utf-8')
    (output_dir / 'vocab.json').write_text(_pretty_json(vocab) + '\n', encoding='utf-8')
    (output_dir / 'visual_label_vectorizer.json').write_text(
        _pretty_json(label_vectorizer.to_json()) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'state_vectorizer.json').write_text(
        _pretty_json({
            'kind': 'room315_visual_state_constant_state',
            'dataset_mode': DATASET_MODE_VISUAL_STATE,
            'names': list(VISUAL_STATE_NAMES),
            'dim': len(VISUAL_STATE_NAMES),
        }) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'target_stats.json').write_text(
        _pretty_json({
            'mean': target_mean.tolist(),
            'std': target_std.tolist(),
        }) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'training_config.json').write_text(_pretty_json(config) + '\n', encoding='utf-8')

    train_dataset = Room315VisualStateDataset(
        train_rows,
        train_labels,
        dataset_root=dataset_root,
        label_vectorizer=label_vectorizer,
        target_mean=target_mean,
        target_std=target_std,
        max_tokens=args.max_tokens,
        image_width=args.image_width,
        image_height=args.image_height,
        torch_module=torch_module,
        allow_blank_images=args.allow_blank_images,
    )
    val_dataset = Room315VisualStateDataset(
        val_rows,
        val_labels,
        dataset_root=dataset_root,
        label_vectorizer=label_vectorizer,
        target_mean=target_mean,
        target_std=target_std,
        max_tokens=args.max_tokens,
        image_width=args.image_width,
        image_height=args.image_height,
        torch_module=torch_module,
        allow_blank_images=args.allow_blank_images,
    )
    train_loader = torch_module.utils.data.DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=(device == 'cuda'),
    )
    val_loader = torch_module.utils.data.DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=(device == 'cuda'),
    )
    variant_results: dict[str, Any] = {}
    requested_variants = visual_adaptation_variants(args.visual_adaptation)
    for adaptation_mode in requested_variants:
        run_dir = (
            output_dir / adaptation_mode
            if args.visual_adaptation == VISUAL_ADAPTATION_COMPARE
            else output_dir
        )
        variant_config = {
            **config,
            'visual_adaptation': adaptation_mode,
            'visual_model_kind': VISUAL_MODEL_KIND,
            'visual_lora_rank': args.visual_lora_rank,
            'visual_pretrained_backbone': args.visual_pretrained_backbone,
            'output_dir': str(run_dir),
        }
        model = _build_visual_state_model(
            torch_module,
            output_dim=output_dim,
            adaptation_mode=adaptation_mode,
            lora_rank=args.visual_lora_rank,
        ).to(device)
        pretrained_report = _load_visual_pretrained_backbone(
            torch_module,
            model,
            args.visual_pretrained_backbone,
        )
        variant_config['visual_pretrained_backbone_report'] = pretrained_report
        counts = parameter_report(model)
        variant_config['visual_model'] = visual_state_model_metadata(
            adaptation_mode=adaptation_mode,
            lora_rank=args.visual_lora_rank,
            parameter_counts=counts,
            pretrained_backbone=args.visual_pretrained_backbone,
            pretrained_backbone_report=pretrained_report,
        )
        _write_visual_training_artifacts(
            run_dir,
            dataset_report=dataset_report,
            vocab=vocab,
            label_vectorizer=label_vectorizer,
            target_mean=target_mean,
            target_std=target_std,
            config=variant_config,
        )
        trainable_parameters = [parameter for parameter in model.parameters() if parameter.requires_grad]
        if not trainable_parameters:
            raise ValueError(f'{adaptation_mode} visual model has no trainable parameters')
        optimizer = torch_module.optim.AdamW(
            trainable_parameters,
            lr=args.learning_rate,
            weight_decay=args.weight_decay,
        )
        loss_fn = torch_module.nn.SmoothL1Loss()
        history: list[dict[str, Any]] = []
        best_val_loss = float('inf')
        best_epoch = 0

        for epoch in range(1, args.epochs + 1):
            model.train()
            running_loss = 0.0
            seen = 0
            for batch in train_loader:
                text = batch['text'].to(device)
                state = batch['state'].to(device)
                image = batch['image'].to(device)
                target = batch['target'].to(device)
                optimizer.zero_grad(set_to_none=True)
                pred = model(text, state, image)
                loss = loss_fn(pred, target)
                loss.backward()
                optimizer.step()
                batch_size = int(target.shape[0])
                running_loss += float(loss.item()) * batch_size
                seen += batch_size
            train_loss = running_loss / max(1, seen)
            val_metrics = _evaluate_visual_state(
                torch_module,
                model,
                val_loader,
                device=device,
                target_mean=target_mean,
                target_std=target_std,
                label_names=label_vectorizer.names,
            )
            epoch_metrics = {
                'epoch': epoch,
                'visual_adaptation': adaptation_mode,
                'train_loss': round(train_loss, 6),
                'parameter_counts': counts,
                **{f'val_{key}': value for key, value in val_metrics.items()},
            }
            history.append(epoch_metrics)
            print(_pretty_json(epoch_metrics), flush=True)
            _save_checkpoint(
                torch_module,
                run_dir / 'last.pt',
                model=model,
                optimizer=optimizer,
                epoch=epoch,
                metrics=epoch_metrics,
                config=variant_config,
            )
            if float(val_metrics['loss']) < best_val_loss:
                best_val_loss = float(val_metrics['loss'])
                best_epoch = epoch
                _save_checkpoint(
                    torch_module,
                    run_dir / 'best.pt',
                    model=model,
                    optimizer=optimizer,
                    epoch=epoch,
                    metrics=epoch_metrics,
                    config=variant_config,
                )

        variant_summary = {
            'visual_adaptation': adaptation_mode,
            'output_dir': str(run_dir),
            'best_checkpoint': str(run_dir / 'best.pt'),
            'last_checkpoint': str(run_dir / 'last.pt'),
            'best_epoch': best_epoch,
            'best_val_loss': best_val_loss,
            'history': history,
            'parameter_counts': counts,
            'visual_model': variant_config['visual_model'],
            'config': variant_config,
        }
        (run_dir / 'metrics.json').write_text(_pretty_json(variant_summary) + '\n', encoding='utf-8')
        variant_results[adaptation_mode] = variant_summary

    selected_variant = _select_visual_adaptation_variant(
        variant_results,
        requested=args.visual_selected_adaptation,
    )
    selected = variant_results[selected_variant]
    selected_dir = Path(selected['output_dir'])
    if selected_dir != output_dir:
        for name in (
            'best.pt',
            'last.pt',
            'dataset_report.json',
            'vocab.json',
            'visual_label_vectorizer.json',
            'state_vectorizer.json',
            'target_stats.json',
            'training_config.json',
        ):
            shutil.copy2(selected_dir / name, output_dir / name)
    smoke = visual_state_plansys2_smoke()
    summary = {
        'tool': 'room_315_vla_train_local',
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'purpose': 'small custom visual-state label predictor; outputs are not rail commands',
        'output_dir': str(output_dir),
        'best_checkpoint': str(output_dir / 'best.pt'),
        'last_checkpoint': str(output_dir / 'last.pt'),
        'selected_visual_adaptation': selected_variant,
        'best_epoch': selected['best_epoch'],
        'best_val_loss': selected['best_val_loss'],
        'history': selected['history'],
        'variant_results': variant_results,
        'parameter_counts': selected['parameter_counts'],
        'visual_model': selected['visual_model'],
        'diffusion_policy_head': False,
        'direct_command_capability': False,
        'state_fusion_to_plansys2': smoke,
        'config': {**config, 'selected_visual_adaptation': selected_variant},
        'dataset_report': dataset_report,
    }
    (output_dir / 'metrics.json').write_text(_pretty_json(summary) + '\n', encoding='utf-8')
    return summary


def _load_visual_label_vectorizer(path: Path) -> VisualStateLabelVectorizer:
    parsed = _load_json(path)
    return VisualStateLabelVectorizer.from_json(parsed)


def evaluate_visual_state_checkpoint(args: argparse.Namespace) -> dict[str, Any]:
    torch_module = _require_torch()
    checkpoint_path = args.eval_checkpoint.expanduser().resolve()
    if not checkpoint_path.exists():
        raise FileNotFoundError(f'checkpoint not found: {checkpoint_path}')
    splits_dir = args.splits_dir.expanduser().resolve()
    dataset_root = _resolve_dataset_root(args.dataset_root, splits_dir)
    output_dir = (
        args.eval_output_dir.expanduser().resolve()
        if args.eval_output_dir is not None
        else checkpoint_path.parent / 'visual_state_eval'
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    checkpoint = _torch_load_checkpoint(torch_module, checkpoint_path)
    config = _checkpoint_training_config(checkpoint, checkpoint_path)
    vocab = _load_json(_artifact_path(checkpoint_path, 'vocab.json'))
    label_vectorizer = _load_visual_label_vectorizer(
        _artifact_path(checkpoint_path, 'visual_label_vectorizer.json')
    )
    target_mean, target_std = _load_target_stats(_artifact_path(checkpoint_path, 'target_stats.json'))
    _set_seed(torch_module, args.seed)
    device = _choose_device(torch_module, args.device)
    model, model_metadata = _visual_model_from_config(
        torch_module,
        config=config,
        output_dim=int(target_mean.shape[0]),
        vocab_size=max(2, len(vocab)),
    )
    model = model.to(device)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()
    counts = parameter_report(model)
    model_metadata['parameter_counts'] = counts
    if device == 'cuda' and torch_module.cuda.is_available():
        torch_module.cuda.reset_peak_memory_stats()

    split_names = [
        name.strip()
        for name in str(args.eval_splits or '').split(',')
        if name.strip()
    ]
    if not split_names:
        raise ValueError('--eval-splits selected no splits')

    image_width = _safe_int(config.get('image_width'), args.image_width)
    image_height = _safe_int(config.get('image_height'), args.image_height)
    max_tokens = _safe_int(config.get('max_tokens'), args.max_tokens)
    mean_tensor = torch_module.as_tensor(target_mean, dtype=torch_module.float32, device=device)
    std_tensor = torch_module.as_tensor(target_std, dtype=torch_module.float32, device=device)
    started = time.perf_counter()
    splits: dict[str, Any] = {}
    all_records: list[dict[str, Any]] = []
    family_records: dict[str, list[dict[str, Any]]] = {}

    with torch_module.no_grad():
        for split_name in split_names:
            split_file = f'{split_name}.jsonl'
            labels_path = visual_label_path_for_split(splits_dir / split_file)
            rows, labels, input_integrity = _load_visual_split(
                splits_dir,
                split_file,
                labels_path,
                limit=args.limit_eval_rows,
            )
            image_report = image_integrity_report(
                rows,
                dataset_root,
                split_name=split_name,
                allow_blank_images=args.allow_blank_images,
                dataset_mode=DATASET_MODE_VISUAL_STATE,
            )
            split_records: list[dict[str, Any]] = []
            for sample_index, (row, label) in enumerate(zip(rows, labels)):
                cycle_start = time.perf_counter()
                text = torch_module.zeros((1, max_tokens), dtype=torch_module.long, device=device)
                state = torch_module.zeros((1, len(VISUAL_STATE_NAMES)), dtype=torch_module.float32, device=device)
                image = torch_module.as_tensor(
                    load_paired_images(
                        row,
                        dataset_root,
                        width=image_width,
                        height=image_height,
                        allow_blank_images=args.allow_blank_images,
                        dataset_mode=DATASET_MODE_VISUAL_STATE,
                    ),
                    dtype=torch_module.float32,
                    device=device,
                ).unsqueeze(0)
                if device == 'cuda' and torch_module.cuda.is_available():
                    torch_module.cuda.synchronize()
                inference_start = time.perf_counter()
                pred_norm = model(text, state, image)
                if device == 'cuda' and torch_module.cuda.is_available():
                    torch_module.cuda.synchronize()
                inference_latency = time.perf_counter() - inference_start
                pred = (pred_norm[0] * std_tensor + mean_tensor).detach().cpu().numpy()
                true_raw = np.asarray(label_vectorizer.transform(label), dtype=np.float32)
                record = {
                    'split': split_name,
                    'sample_index': sample_index,
                    'episode_id': str(row.get('episode_id') or ''),
                    'task_family': str(row.get('scenario_family') or ''),
                    'true_raw': true_raw.tolist(),
                    'pred_raw': pred[: len(true_raw)].astype(np.float32).tolist(),
                    'inference_latency_seconds': inference_latency,
                    'cycle_time_seconds': time.perf_counter() - cycle_start,
                }
                split_records.append(record)
                family_key = record['task_family'] or 'unknown'
                family_records.setdefault(family_key, []).append(record)
            splits[split_name] = {
                'source_file': _file_fingerprint(splits_dir / split_file),
                'label_file': _file_fingerprint(labels_path),
                'row_fingerprint': _rows_fingerprint(rows),
                'label_fingerprint': _rows_fingerprint([
                    {'visual_state_labels': label}
                    for label in labels
                ]),
                'rows': len(rows),
                'model_input_integrity': input_integrity,
                'image_integrity': image_report,
                'class_balance': visual_state_class_balance(labels),
                'metrics': visual_state_metrics(split_records, label_vectorizer.names),
            }
            all_records.extend(split_records)

    summary_path = output_dir / 'visual_state_eval.json'
    summary = {
        'tool': 'room_315_vla_train_local',
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'purpose': 'small custom visual-state label checkpoint evaluation',
        'checkpoint': str(checkpoint_path),
        'checkpoint_epoch': checkpoint.get('epoch'),
        'checkpoint_metrics': checkpoint.get('metrics'),
        'checkpoint_training_config': config,
        'dataset_root': str(dataset_root),
        'splits_dir': str(splits_dir),
        'evaluated_splits': split_names,
        'seed': args.seed,
        'device': device,
        'allow_blank_images': bool(args.allow_blank_images),
        'debug_blank_image_mode': bool(args.allow_blank_images),
        'production_feature_source': 'model_input.overhead_images only',
        'visual_model': model_metadata,
        'parameter_counts': counts,
        'diffusion_policy_head': False,
        'direct_command_capability': False,
        'feature_purity': {
            'production_features_from': 'model_input.overhead_images',
            'oracle_labels_physically_separate_from_model_input': True,
            'row_level_metadata_used_as_features': [],
            'debug_or_ablation_features': ['allow_blank_images'] if args.allow_blank_images else [],
            'learned_output_boundary': 'visual facts only; no PDDL, plans, primitives, or rail commands',
        },
        'artifact_fingerprints': {
            name: _file_fingerprint(path)
            for name, path in {
                'checkpoint': checkpoint_path,
                'vocab': _artifact_path(checkpoint_path, 'vocab.json'),
                'visual_label_vectorizer': _artifact_path(checkpoint_path, 'visual_label_vectorizer.json'),
                'target_stats': _artifact_path(checkpoint_path, 'target_stats.json'),
                'training_config': _artifact_path(checkpoint_path, 'training_config.json'),
            }.items()
            if path.exists()
        },
        'label_names': label_vectorizer.names,
        'splits': splits,
        'aggregate_metrics': visual_state_metrics(all_records, label_vectorizer.names),
        'family_metrics': {
            family: visual_state_metrics(records, label_vectorizer.names)
            for family, records in sorted(family_records.items())
        },
        'state_fusion_to_plansys2': visual_state_plansys2_smoke(),
        'peak_gpu_memory': _peak_gpu_memory(torch_module, device),
        'elapsed_seconds': round(time.perf_counter() - started, 3),
        'summary_json': str(summary_path),
    }
    summary_path.write_text(_pretty_json(summary) + '\n', encoding='utf-8')
    return summary


def evaluate_checkpoint(args: argparse.Namespace) -> dict[str, Any]:
    return evaluate_visual_state_checkpoint(args)


def train_local(args: argparse.Namespace) -> dict[str, Any]:
    return train_visual_state(args)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Train or evaluate the Room 315 visual-state model from validated JSONL splits.'
        )
    )
    parser.add_argument(
        '--splits-dir',
        type=Path,
        default=DEFAULT_SPLITS_DIR,
        help='Split directory. Defaults to ROOM315_VLA_SPLITS_DIR or room315_local_training/splits.',
    )
    parser.add_argument(
        '--dataset-root',
        type=Path,
        default=None,
        help=(
            'Dataset root for relative image refs. Defaults to the split manifest '
            'dataset_root, ROOM315_VLA_DATASET_ROOT, or room315_payload_dataset.'
        ),
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=(
            'Output checkpoint directory. Defaults to ROOM315_VISUAL_STATE_OUTPUT_DIR '
            'or room315_local_training/checkpoints/visual_state.'
        ),
    )
    parser.add_argument(
        '--dataset-mode',
        choices=DATASET_MODES,
        default=DATASET_MODE_VISUAL_STATE,
        help=(
            f'{DATASET_MODE_VISUAL_STATE} trains/evaluates image-to-visual-state labels from '
            f'separate *{VISUAL_LABEL_SUFFIX} sidecars.'
        ),
    )
    parser.add_argument('--train-file', default='train.jsonl')
    parser.add_argument('--val-file', default='val.jsonl')
    parser.add_argument(
        '--visual-train-labels',
        type=Path,
        default=None,
        help='Visual-state train label JSONL. Defaults to sibling train_visual_labels.jsonl.',
    )
    parser.add_argument(
        '--visual-val-labels',
        type=Path,
        default=None,
        help='Visual-state val label JSONL. Defaults to sibling val_visual_labels.jsonl.',
    )
    parser.add_argument('--epochs', type=int, default=5)
    parser.add_argument('--batch-size', type=int, default=8)
    parser.add_argument('--learning-rate', type=float, default=1e-3)
    parser.add_argument('--weight-decay', type=float, default=1e-4)
    parser.add_argument('--image-width', type=int, default=160)
    parser.add_argument('--image-height', type=int, default=120)
    parser.add_argument('--max-tokens', type=int, default=32)
    parser.add_argument('--num-workers', type=int, default=2)
    parser.add_argument('--device', default='auto', help='auto, cuda, or cpu')
    parser.add_argument('--seed', type=int, default=13)
    parser.add_argument('--limit-train-rows', type=int, default=None)
    parser.add_argument('--limit-val-rows', type=int, default=None)
    parser.add_argument(
        '--allow-blank-images',
        action='store_true',
        help='Debug only: substitute zero tensors for missing/unreadable required images.',
    )
    parser.add_argument(
        '--visual-adaptation',
        choices=VISUAL_ADAPTATION_MODES,
        default=VISUAL_ADAPTATION_COMPARE,
        help=(
            'Visual-state only: train a frozen compact backbone, LoRA adaptation, '
            'or both for comparison.'
        ),
    )
    parser.add_argument(
        '--visual-lora-rank',
        type=int,
        default=4,
        help='Visual-state only: low-rank adapter rank used by --visual-adaptation lora/compare.',
    )
    parser.add_argument(
        '--visual-selected-adaptation',
        choices=('best_val_loss', VISUAL_ADAPTATION_FROZEN_BACKBONE, VISUAL_ADAPTATION_LORA),
        default='best_val_loss',
        help='Visual-state only: selected checkpoint copied to the root output directory after compare.',
    )
    parser.add_argument(
        '--visual-pretrained-backbone',
        default='compact_pretrained_backbone_v1',
        help='Visual-state only: label/path for the frozen compact pretrained backbone in reports.',
    )
    parser.add_argument(
        '--eval-checkpoint',
        type=Path,
        default=None,
        help='Evaluate an existing local checkpoint instead of training.',
    )
    parser.add_argument(
        '--eval-output-dir',
        type=Path,
        default=None,
        help='Output directory for --eval-checkpoint artifacts.',
    )
    parser.add_argument(
        '--eval-splits',
        default='train,val,test',
        help='Comma-separated JSONL split names to evaluate with --eval-checkpoint.',
    )
    parser.add_argument(
        '--limit-eval-rows',
        type=int,
        default=None,
        help='Optional debug row limit per split during --eval-checkpoint.',
    )
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_arg_parser().parse_args(argv)
    if args.eval_checkpoint is not None:
        summary = evaluate_checkpoint(args)
    else:
        summary = train_visual_state(args)
    print(_pretty_json(summary))


if __name__ == '__main__':
    main()
