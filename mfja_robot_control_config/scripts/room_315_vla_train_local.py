#!/usr/bin/env python3
"""Small custom Room 315 visual-state trainer.

The learned model predicts structured visual facts for state fusion. It never
produces PDDL, plans, primitive commands, device commands, or rail commands.
"""

import argparse
import hashlib
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
from urllib.parse import urlparse

import numpy as np
from PIL import Image

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_state_dataset import DATASET_MODES
from room_315_visual_state_dataset import DATASET_MODE_VISUAL_STATE
from room_315_visual_state_dataset import IMAGE_KEYS
from room_315_visual_state_dataset import VISUAL_LABEL_SUFFIX
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
VISUAL_ADAPTATION_PARTIAL_FINETUNE = 'partial_finetune'
VISUAL_ADAPTATION_COMPARE = 'compare'
VISUAL_ADAPTATION_MODES = (
    VISUAL_ADAPTATION_FROZEN_BACKBONE,
    VISUAL_ADAPTATION_LORA,
    VISUAL_ADAPTATION_PARTIAL_FINETUNE,
    VISUAL_ADAPTATION_COMPARE,
)
VISUAL_MODEL_KIND = 'structured_visual_state_torchvision_resnet18_fixed8_v3'
VISUAL_BACKBONE_ARCHITECTURE = 'resnet18'
VISUAL_BACKBONE_LIBRARY = 'torchvision'
VISUAL_BACKBONE_WEIGHTS_IDENTIFIER = 'ResNet18_Weights.IMAGENET1K_V1'
VISUAL_BACKBONE_SOURCE = (
    f'{VISUAL_BACKBONE_LIBRARY}:{VISUAL_BACKBONE_ARCHITECTURE}:IMAGENET1K_V1'
)
VISUAL_BACKBONE_INPUT_WIDTH = 224
VISUAL_BACKBONE_INPUT_HEIGHT = 224
VISUAL_BACKBONE_NORMALIZATION_MEAN = (0.485, 0.456, 0.406)
VISUAL_BACKBONE_NORMALIZATION_STD = (0.229, 0.224, 0.225)
VISUAL_IMAGE_RESIZE = 'direct_bilinear_resize'
VISUAL_AUGMENTATIONS: tuple[str, ...] = ()
VISUAL_LOSS_HEADS = (
    'segment_location',
    'loaded_state',
    'bbox',
    's_m',
    's_ratio',
)
VISUAL_LOSS_HEAD_WEIGHTS = {
    name: 1.0
    for name in VISUAL_LOSS_HEADS
}
VISUAL_LOSS_KIND = 'masked_per_sample_smooth_l1_equal_head_weight_v1'


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


def _require_torchvision():
    try:
        import torchvision
    except ImportError as exc:
        raise SystemExit(
            'TorchVision is required for the official pretrained visual backbone but is not '
            'installed. Install the version matching PyTorch from the official PyTorch index.'
        ) from exc
    return torchvision


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
    normalization_mean: tuple[float, float, float] | None = None,
    normalization_std: tuple[float, float, float] | None = None,
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
    paired = np.concatenate([left, right], axis=0).astype(np.float32)
    if normalization_mean is None and normalization_std is None:
        return paired
    if normalization_mean is None or normalization_std is None:
        raise ValueError('normalization mean and std must be provided together')
    mean = np.asarray(normalization_mean * 2, dtype=np.float32).reshape(6, 1, 1)
    std = np.asarray(normalization_std * 2, dtype=np.float32).reshape(6, 1, 1)
    if np.any(std <= 0.0):
        raise ValueError('normalization std values must be positive')
    return ((paired - mean) / std).astype(np.float32)


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
        image_width: int,
        image_height: int,
        torch_module: Any,
        allow_blank_images: bool = False,
        normalization_mean: tuple[float, float, float] = VISUAL_BACKBONE_NORMALIZATION_MEAN,
        normalization_std: tuple[float, float, float] = VISUAL_BACKBONE_NORMALIZATION_STD,
    ) -> None:
        if len(rows) != len(labels):
            raise ValueError('visual-state rows and labels must have the same length')
        self.rows = rows
        self.labels = labels
        self.dataset_root = dataset_root
        self.label_vectorizer = label_vectorizer
        self.target_mean = target_mean
        self.target_std = target_std
        self.image_width = image_width
        self.image_height = image_height
        self.torch = torch_module
        self.allow_blank_images = allow_blank_images
        self.normalization_mean = normalization_mean
        self.normalization_std = normalization_std

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int) -> dict[str, Any]:
        row = self.rows[index]
        target = np.asarray(self.label_vectorizer.transform(self.labels[index]), dtype=np.float32)
        target_mask = np.asarray(
            self.label_vectorizer.target_mask(self.labels[index]),
            dtype=np.float32,
        )
        normalized_target = (target - self.target_mean) / self.target_std
        return {
            'image': self.torch.as_tensor(
                load_paired_images(
                    row,
                    self.dataset_root,
                    width=self.image_width,
                    height=self.image_height,
                    allow_blank_images=self.allow_blank_images,
                    dataset_mode=DATASET_MODE_VISUAL_STATE,
                    normalization_mean=self.normalization_mean,
                    normalization_std=self.normalization_std,
                ),
                dtype=self.torch.float32,
            ),
            'target': self.torch.as_tensor(normalized_target, dtype=self.torch.float32),
            'target_mask': self.torch.as_tensor(target_mask, dtype=self.torch.float32),
            'raw_target': self.torch.as_tensor(target, dtype=self.torch.float32),
        }


def visual_adaptation_variants(mode: str) -> list[str]:
    mode = str(mode or VISUAL_ADAPTATION_FROZEN_BACKBONE).strip().lower()
    if mode == VISUAL_ADAPTATION_COMPARE:
        return [VISUAL_ADAPTATION_FROZEN_BACKBONE, VISUAL_ADAPTATION_LORA]
    if mode in {
        VISUAL_ADAPTATION_FROZEN_BACKBONE,
        VISUAL_ADAPTATION_LORA,
        VISUAL_ADAPTATION_PARTIAL_FINETUNE,
    }:
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
    torchvision_module: Any | None = None,
):
    from room_315_visual_model import build_visual_state_model

    torchvision_module = torchvision_module or _require_torchvision()
    mode = visual_adaptation_variants(adaptation_mode)[0]
    return build_visual_state_model(
        torch_module,
        torchvision_module,
        output_dim=output_dim,
        adaptation_mode=mode,
        lora_rank=lora_rank,
    )


def visual_state_model_metadata(
    *,
    adaptation_mode: str,
    lora_rank: int,
    parameter_counts: dict[str, Any] | None = None,
    pretrained_backbone: str | None = None,
    pretrained_backbone_report: dict[str, Any] | None = None,
    torchvision_version: str = '',
) -> dict[str, Any]:
    report = pretrained_backbone_report or {}
    return {
        'model_kind': VISUAL_MODEL_KIND,
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'adaptation_mode': adaptation_mode,
        'backbone_architecture': VISUAL_BACKBONE_ARCHITECTURE,
        'backbone_library': VISUAL_BACKBONE_LIBRARY,
        'backbone_library_version': torchvision_version,
        'checkpoint_source': report.get('checkpoint_source'),
        'checkpoint_identifier': report.get('checkpoint_identifier'),
        'checkpoint_sha256': report.get('checkpoint_sha256'),
        'pretrained_backbone': pretrained_backbone or VISUAL_BACKBONE_SOURCE,
        'pretrained_requested': bool(report.get('pretrained_requested')),
        'pretrained_loaded': bool(report.get('pretrained_loaded')),
        'pretrained_backbone_report': report,
        'backbone_trainable': adaptation_mode == VISUAL_ADAPTATION_PARTIAL_FINETUNE,
        'backbone_trainable_scope': (
            'layer4'
            if adaptation_mode == VISUAL_ADAPTATION_PARTIAL_FINETUNE
            else 'none'
        ),
        'lora_rank': max(1, int(lora_rank)) if adaptation_mode == VISUAL_ADAPTATION_LORA else 0,
        'parameter_efficient_method': (
            'low_rank_paired_feature_adapter'
            if adaptation_mode == VISUAL_ADAPTATION_LORA
            else None
        ),
        'policy_head': 'linear_structured_visual_state_regression',
        'diffusion_policy_head': False,
        'output_semantics': 'visual_facts_for_state_fusion_not_rail_commands',
        'direct_command_capability': False,
        'parameter_counts': parameter_counts or {},
        'image_preprocessing': {
            'input_resolution': [
                VISUAL_BACKBONE_INPUT_HEIGHT,
                VISUAL_BACKBONE_INPUT_WIDTH,
            ],
            'resize': VISUAL_IMAGE_RESIZE,
            'value_range': [0.0, 1.0],
            'normalization_mean_per_rgb_view': list(VISUAL_BACKBONE_NORMALIZATION_MEAN),
            'normalization_std_per_rgb_view': list(VISUAL_BACKBONE_NORMALIZATION_STD),
            'augmentations': list(VISUAL_AUGMENTATIONS),
        },
    }


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _pretrained_requested(source: str | None) -> bool:
    return str(source or '').strip().casefold() not in {'', 'none', 'random', 'false'}


def _load_visual_pretrained_backbone(
    torch_module: Any,
    model: Any,
    source: str | None,
    *,
    torchvision_module: Any | None = None,
) -> dict[str, Any]:
    torchvision_module = torchvision_module or _require_torchvision()
    source_text = str(source or '').strip()
    requested = _pretrained_requested(source_text)
    if not requested:
        return {
            'backbone_architecture': VISUAL_BACKBONE_ARCHITECTURE,
            'backbone_library': VISUAL_BACKBONE_LIBRARY,
            'backbone_library_version': str(torchvision_module.__version__),
            'checkpoint_source': None,
            'checkpoint_identifier': None,
            'checkpoint_path': None,
            'checkpoint_sha256': None,
            'pretrained_requested': False,
            'pretrained_loaded': False,
            'strict_load': False,
        }
    aliases = {
        VISUAL_BACKBONE_SOURCE.casefold(),
        VISUAL_BACKBONE_WEIGHTS_IDENTIFIER.casefold(),
        'resnet18_weights.imagenet1k_v1',
        'imagenet1k_v1',
    }
    if source_text.casefold() not in aliases:
        raise ValueError(
            f'unsupported pretrained backbone {source_text!r}; expected '
            f'{VISUAL_BACKBONE_SOURCE!r} or "none"'
        )
    weights = torchvision_module.models.ResNet18_Weights.IMAGENET1K_V1
    checkpoint_url = str(weights.url)
    try:
        state_dict = weights.get_state_dict(progress=True, check_hash=True)
        official_model = torchvision_module.models.resnet18(weights=None)
        official_model.load_state_dict(state_dict, strict=True)
        official_model.fc = torch_module.nn.Identity()
        model.backbone.load_state_dict(official_model.state_dict(), strict=True)
    except Exception as exc:
        raise RuntimeError(
            f'pretrained weights were requested from {checkpoint_url}, but strict loading failed'
        ) from exc
    checkpoint_name = Path(urlparse(checkpoint_url).path).name
    checkpoint_path = Path(torch_module.hub.get_dir()) / 'checkpoints' / checkpoint_name
    checkpoint_sha256 = _sha256_file(checkpoint_path) if checkpoint_path.is_file() else None
    report = {
        'backbone_architecture': VISUAL_BACKBONE_ARCHITECTURE,
        'backbone_library': VISUAL_BACKBONE_LIBRARY,
        'backbone_library_version': str(torchvision_module.__version__),
        'checkpoint_source': checkpoint_url,
        'checkpoint_identifier': VISUAL_BACKBONE_WEIGHTS_IDENTIFIER,
        'checkpoint_path': str(checkpoint_path.resolve()) if checkpoint_path.is_file() else None,
        'checkpoint_sha256': checkpoint_sha256,
        'pretrained_requested': True,
        'pretrained_loaded': True,
        'strict_load': True,
    }
    if not report['pretrained_loaded']:
        raise RuntimeError('pretrained weights were requested but were not loaded')
    return report


def _choose_device(torch_module: Any, requested: str) -> str:
    requested = str(requested or 'auto').strip().lower()
    if requested == 'auto':
        return 'cuda' if torch_module.cuda.is_available() else 'cpu'
    if requested == 'cuda' and not torch_module.cuda.is_available():
        raise SystemExit('CUDA was requested, but torch.cuda.is_available() is false.')
    return requested


def _seed_report(seed: int) -> dict[str, int]:
    requested_seed = int(seed)
    return {
        'requested': requested_seed,
        'python': requested_seed,
        'numpy': requested_seed % (2 ** 32),
        'torch': requested_seed % (2 ** 63),
    }


def _set_seed(torch_module: Any, seed: int) -> dict[str, int]:
    report = _seed_report(seed)
    random.seed(report['python'])
    np.random.seed(report['numpy'])
    torch_module.manual_seed(report['torch'])
    if torch_module.cuda.is_available():
        torch_module.cuda.manual_seed_all(report['torch'])
        torch_module.backends.cudnn.benchmark = False
        torch_module.backends.cudnn.deterministic = True
    torch_module.use_deterministic_algorithms(True, warn_only=True)
    return report


def _row_limit(rows: list[dict[str, Any]], limit: int | None) -> list[dict[str, Any]]:
    if limit is None or limit <= 0:
        return rows
    return rows[:limit]


def visual_loss_head_indexes(label_names: list[str]) -> dict[str, list[int]]:
    groups = {name: [] for name in VISUAL_LOSS_HEADS}
    for index, name in enumerate(label_names):
        if '.location.' in name or name.endswith('.rail_position.segment_length_m'):
            head = 'segment_location'
        elif '.loaded_state==' in name:
            head = 'loaded_state'
        elif '.bbox.' in name:
            head = 'bbox'
        elif name.endswith('.rail_position.s_m'):
            head = 's_m'
        elif name.endswith('.rail_position.s_ratio'):
            head = 's_ratio'
        else:
            raise ValueError(f'visual target is not assigned to a loss head: {name}')
        groups[head].append(index)
    missing = [name for name, indexes in groups.items() if not indexes]
    if missing:
        raise ValueError(f'visual loss heads have no target indexes: {missing}')
    return groups


def visual_loss_components(
    torch_module: Any,
    prediction: Any,
    target: Any,
    target_mask: Any,
    label_names: list[str],
    *,
    head_weights: dict[str, float] | None = None,
) -> dict[str, Any]:
    if prediction.shape != target.shape or prediction.shape != target_mask.shape:
        raise ValueError(
            'prediction, target, and target_mask must have identical shapes; '
            f'got {tuple(prediction.shape)}, {tuple(target.shape)}, '
            f'{tuple(target_mask.shape)}'
        )
    if prediction.ndim != 2 or prediction.shape[1] != len(label_names):
        raise ValueError('visual loss expects a batch by target-dimension tensor')
    groups = visual_loss_head_indexes(label_names)
    weights = dict(VISUAL_LOSS_HEAD_WEIGHTS)
    if head_weights is not None:
        weights.update({str(key): float(value) for key, value in head_weights.items()})
    element_loss = torch_module.nn.functional.smooth_l1_loss(
        prediction,
        target,
        reduction='none',
    )
    head_losses: dict[str, Any] = {}
    head_loss_sums: dict[str, Any] = {}
    head_sample_counts: dict[str, int] = {}
    weighted_terms: list[Any] = []
    active_weight = 0.0
    for head in VISUAL_LOSS_HEADS:
        indexes = groups[head]
        head_mask = target_mask[:, indexes]
        per_sample_elements = head_mask.sum(dim=1)
        valid_samples = per_sample_elements > 0
        sample_count = int(valid_samples.sum().item())
        if sample_count <= 0:
            raise ValueError(f'loss head {head!r} has no valid samples in this batch')
        per_sample_loss = (
            (element_loss[:, indexes] * head_mask).sum(dim=1)
            / per_sample_elements.clamp_min(1.0)
        )
        loss_sum = per_sample_loss[valid_samples].sum()
        head_loss = loss_sum / float(sample_count)
        head_losses[head] = head_loss
        head_loss_sums[head] = loss_sum
        head_sample_counts[head] = sample_count
        weight = float(weights[head])
        if weight < 0.0:
            raise ValueError(f'loss head weight must be non-negative: {head}={weight}')
        weighted_terms.append(head_loss * weight)
        active_weight += weight
    if active_weight <= 0.0:
        raise ValueError('at least one visual loss head weight must be positive')
    total_weighted_loss = sum(weighted_terms) / active_weight
    return {
        'kind': VISUAL_LOSS_KIND,
        'head_losses': head_losses,
        'head_loss_sums': head_loss_sums,
        'head_sample_counts': head_sample_counts,
        'head_weights': weights,
        'total_weighted_loss': total_weighted_loss,
    }


def _new_visual_loss_accumulator() -> dict[str, Any]:
    return {
        'head_loss_sums': {head: 0.0 for head in VISUAL_LOSS_HEADS},
        'head_sample_counts': {head: 0 for head in VISUAL_LOSS_HEADS},
    }


def _accumulate_visual_loss(accumulator: dict[str, Any], components: dict[str, Any]) -> None:
    for head in VISUAL_LOSS_HEADS:
        accumulator['head_loss_sums'][head] += float(
            components['head_loss_sums'][head].detach().item()
        )
        accumulator['head_sample_counts'][head] += int(
            components['head_sample_counts'][head]
        )


def _summarize_visual_loss(
    accumulator: dict[str, Any],
    *,
    head_weights: dict[str, float] | None = None,
) -> dict[str, Any]:
    weights = dict(VISUAL_LOSS_HEAD_WEIGHTS)
    if head_weights is not None:
        weights.update({str(key): float(value) for key, value in head_weights.items()})
    head_losses: dict[str, float] = {}
    weighted_total = 0.0
    active_weight = 0.0
    for head in VISUAL_LOSS_HEADS:
        count = int(accumulator['head_sample_counts'][head])
        if count <= 0:
            raise ValueError(f'loss accumulator has no samples for head {head!r}')
        value = float(accumulator['head_loss_sums'][head]) / count
        head_losses[head] = value
        weighted_total += value * weights[head]
        active_weight += weights[head]
    total = weighted_total / active_weight
    return {
        **{f'{head}_loss': round(value, 6) for head, value in head_losses.items()},
        'total_weighted_loss': total,
        'loss': total,
        'loss_kind': VISUAL_LOSS_KIND,
        'loss_head_weights': weights,
        'loss_sample_counts': dict(accumulator['head_sample_counts']),
    }


def _add_visual_per_head_metrics(metrics: dict[str, Any]) -> dict[str, Any]:
    metrics['per_head'] = {
        'segment_location': {
            'loss': metrics['segment_location_loss'],
            'side_accuracy': metrics.get('side_accuracy'),
            'block_accuracy': metrics.get('block_accuracy'),
            'full_location_accuracy': metrics.get('full_location_accuracy'),
            'top2_block_accuracy': metrics.get('top2_block_accuracy'),
        },
        'loaded_state': {
            'loss': metrics['loaded_state_loss'],
            'accuracy': metrics.get('loaded_state_accuracy'),
        },
        'bbox': {
            'loss': metrics['bbox_loss'],
            'mae': metrics.get('bbox_mae'),
        },
        's_m': {
            'loss': metrics['s_m_loss'],
            'mae': metrics.get('s_m_mae'),
        },
        's_ratio': {
            'loss': metrics['s_ratio_loss'],
            'mae': metrics.get('s_ratio_mae'),
        },
    }
    return metrics


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
    mean_tensor = torch_module.as_tensor(target_mean, dtype=torch_module.float32, device=device)
    std_tensor = torch_module.as_tensor(target_std, dtype=torch_module.float32, device=device)
    loss_accumulator = _new_visual_loss_accumulator()
    records: list[dict[str, Any]] = []
    with torch_module.no_grad():
        for batch in loader:
            image = batch['image'].to(device)
            target = batch['target'].to(device)
            target_mask = batch['target_mask'].to(device)
            raw_target = batch['raw_target'].to(device)
            pred_norm = model(image)
            loss_components = visual_loss_components(
                torch_module,
                pred_norm,
                target,
                target_mask,
                label_names,
            )
            _accumulate_visual_loss(loss_accumulator, loss_components)
            pred = pred_norm * std_tensor + mean_tensor
            for true_row, pred_row, mask_row in zip(
                raw_target.detach().cpu().numpy(),
                pred.detach().cpu().numpy(),
                target_mask.detach().cpu().numpy(),
            ):
                records.append({
                    'true_raw': true_row.astype(np.float32).tolist(),
                    'pred_raw': pred_row.astype(np.float32).tolist(),
                    'target_mask': mask_row.astype(np.float32).tolist(),
                })
    metrics = visual_state_metrics(records, label_names)
    metrics.update(_summarize_visual_loss(loss_accumulator))
    return _add_visual_per_head_metrics(metrics)


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
    _save_visual_label_vectorizer(label_vectorizer, output_dir / 'visual_label_vectorizer.json')
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
    run_metadata = config.get('run_metadata')
    if isinstance(run_metadata, dict):
        (output_dir / 'run_metadata.json').write_text(
            _pretty_json(run_metadata) + '\n',
            encoding='utf-8',
        )


def _visual_model_from_config(
    torch_module: Any,
    *,
    config: dict[str, Any],
    output_dim: int,
) -> tuple[Any, dict[str, Any]]:
    torchvision_module = _require_torchvision()
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
        torchvision_module=torchvision_module,
    )
    metadata = visual_state_model_metadata(
        adaptation_mode=adaptation_mode,
        lora_rank=lora_rank,
        pretrained_backbone=str(config.get('visual_pretrained_backbone') or ''),
        pretrained_backbone_report=dict(config.get('visual_pretrained_backbone_report') or {}),
        torchvision_version=str(torchvision_module.__version__),
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
    torchvision_module = _require_torchvision()
    if args.early_stopping_patience < 0:
        raise ValueError('--early-stopping-patience must be non-negative')
    splits_dir = args.splits_dir.expanduser().resolve()
    dataset_root = _resolve_dataset_root(args.dataset_root, splits_dir)
    if _pretrained_requested(args.visual_pretrained_backbone) and (
        args.image_width != VISUAL_BACKBONE_INPUT_WIDTH
        or args.image_height != VISUAL_BACKBONE_INPUT_HEIGHT
    ):
        raise ValueError(
            'the official pretrained ResNet-18 pipeline requires '
            f'{VISUAL_BACKBONE_INPUT_WIDTH}x{VISUAL_BACKBONE_INPUT_HEIGHT} input; '
            f'got {args.image_width}x{args.image_height}'
        )
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

    seed_report = _set_seed(torch_module, args.seed)
    device = _choose_device(torch_module, args.device)
    label_vectorizer = VisualStateLabelVectorizer.fit(train_labels)
    train_target_masks = [
        label_vectorizer.target_mask(label)
        for label in train_labels
    ]
    target_mean_list, target_std_list = visual_target_stats(
        train_labels,
        label_vectorizer,
        masks=train_target_masks,
    )
    target_mean = np.asarray(target_mean_list, dtype=np.float32)
    target_std = np.asarray(target_std_list, dtype=np.float32)
    output_dim = int(target_mean.shape[0])
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

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
        'target_normalization': {
            'source_split': 'train',
            'kind': 'per_target_mean_and_population_std',
            'absent_shuttle_slots_excluded': True,
            'oracle_position_uncertainty_excluded': True,
        },
        'image_preprocessing': {
            'input_resolution': [args.image_height, args.image_width],
            'resize': VISUAL_IMAGE_RESIZE,
            'value_range': [0.0, 1.0],
            'normalization_mean_per_rgb_view': list(VISUAL_BACKBONE_NORMALIZATION_MEAN),
            'normalization_std_per_rgb_view': list(VISUAL_BACKBONE_NORMALIZATION_STD),
            'augmentations': list(VISUAL_AUGMENTATIONS),
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
        'seed_report': seed_report,
        'device': device,
        'epochs': args.epochs,
        'batch_size': args.batch_size,
        'learning_rate': args.learning_rate,
        'weight_decay': args.weight_decay,
        'image_width': args.image_width,
        'image_height': args.image_height,
        'train_rows': len(train_rows),
        'val_rows': len(val_rows),
        'output_dim': output_dim,
        'allow_blank_images': bool(args.allow_blank_images),
        'debug_blank_image_mode': bool(args.allow_blank_images),
        'production_feature_source': 'model_input.overhead_images only',
        'visual_model_kind': VISUAL_MODEL_KIND,
        'visual_adaptation_request': args.visual_adaptation,
        'visual_lora_rank': args.visual_lora_rank,
        'visual_selected_adaptation': args.visual_selected_adaptation,
        'visual_pretrained_backbone': args.visual_pretrained_backbone,
        'backbone_architecture': VISUAL_BACKBONE_ARCHITECTURE,
        'backbone_library': VISUAL_BACKBONE_LIBRARY,
        'backbone_library_version': str(torchvision_module.__version__),
        'image_normalization': {
            'mean_per_rgb_view': list(VISUAL_BACKBONE_NORMALIZATION_MEAN),
            'std_per_rgb_view': list(VISUAL_BACKBONE_NORMALIZATION_STD),
        },
        'image_resize': VISUAL_IMAGE_RESIZE,
        'augmentations': list(VISUAL_AUGMENTATIONS),
        'loss': {
            'kind': VISUAL_LOSS_KIND,
            'element_loss': 'SmoothL1Loss',
            'element_reduction': 'none',
            'head_reduction': 'mean over valid elements per sample, then mean over samples',
            'sample_weighting': 'uniform',
            'head_weights': dict(VISUAL_LOSS_HEAD_WEIGHTS),
            'total_reduction': 'weighted mean over five heads',
            'absent_shuttle_slots_masked': True,
            'segment_length_m_head': 'segment_location',
            'target_normalization': (
                'train-split per-target mean/population-std over valid shuttle slots only'
            ),
            'output_decoding': 'pred_raw = pred_normalized * target_std + target_mean',
        },
        'early_stopping': {
            'patience': args.early_stopping_patience,
            'criterion': 'validation_total_weighted_loss',
            'best_checkpoint_uses_validation_only': True,
            'test_split_used_during_training': False,
        },
        'resume_checkpoint': (
            str(args.resume_checkpoint.expanduser().resolve())
            if args.resume_checkpoint is not None
            else None
        ),
        'dataset_split_paths': {
            'train': str((splits_dir / args.train_file).resolve()),
            'validation': str((splits_dir / args.val_file).resolve()),
            'test': str((splits_dir / 'test.jsonl').resolve()),
            'train_labels': str(train_labels_path),
            'validation_labels': str(val_labels_path),
            'test_labels': str(visual_label_path_for_split(splits_dir / 'test.jsonl')),
        },
        'diffusion_policy_head': False,
        'learned_output_boundary': 'visual facts only; cannot publish rail commands',
        'dataset_report': str(output_dir / 'dataset_report.json'),
    }

    train_dataset = Room315VisualStateDataset(
        train_rows,
        train_labels,
        dataset_root=dataset_root,
        label_vectorizer=label_vectorizer,
        target_mean=target_mean,
        target_std=target_std,
        image_width=args.image_width,
        image_height=args.image_height,
        torch_module=torch_module,
        allow_blank_images=args.allow_blank_images,
        normalization_mean=VISUAL_BACKBONE_NORMALIZATION_MEAN,
        normalization_std=VISUAL_BACKBONE_NORMALIZATION_STD,
    )
    val_dataset = Room315VisualStateDataset(
        val_rows,
        val_labels,
        dataset_root=dataset_root,
        label_vectorizer=label_vectorizer,
        target_mean=target_mean,
        target_std=target_std,
        image_width=args.image_width,
        image_height=args.image_height,
        torch_module=torch_module,
        allow_blank_images=args.allow_blank_images,
        normalization_mean=VISUAL_BACKBONE_NORMALIZATION_MEAN,
        normalization_std=VISUAL_BACKBONE_NORMALIZATION_STD,
    )
    train_loader = torch_module.utils.data.DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=(device == 'cuda'),
    )
    train_eval_loader = torch_module.utils.data.DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=False,
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
    if args.resume_checkpoint is not None and len(requested_variants) != 1:
        raise ValueError(
            '--resume-checkpoint requires one explicit visual adaptation'
        )
    for adaptation_mode in requested_variants:
        _set_seed(torch_module, args.seed)
        if device == 'cuda' and torch_module.cuda.is_available():
            torch_module.cuda.empty_cache()
            torch_module.cuda.reset_peak_memory_stats()
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
            torchvision_module=torchvision_module,
        )
        pretrained_report = _load_visual_pretrained_backbone(
            torch_module,
            model,
            args.visual_pretrained_backbone,
            torchvision_module=torchvision_module,
        )
        if (
            pretrained_report.get('pretrained_requested')
            and not pretrained_report.get('pretrained_loaded')
        ):
            raise RuntimeError('pretrained weights were requested but were not loaded')
        model = model.to(device)
        variant_config['visual_pretrained_backbone_report'] = pretrained_report
        counts = parameter_report(model)
        variant_config['visual_model'] = visual_state_model_metadata(
            adaptation_mode=adaptation_mode,
            lora_rank=args.visual_lora_rank,
            parameter_counts=counts,
            pretrained_backbone=args.visual_pretrained_backbone,
            pretrained_backbone_report=pretrained_report,
            torchvision_version=str(torchvision_module.__version__),
        )
        variant_config['run_metadata'] = {
            'schema_version': 'room315.visual_training_run.v2',
            'backbone': {
                'architecture': VISUAL_BACKBONE_ARCHITECTURE,
                'library': VISUAL_BACKBONE_LIBRARY,
                'library_version': str(torchvision_module.__version__),
                'checkpoint_source': pretrained_report.get('checkpoint_source'),
                'checkpoint_identifier': pretrained_report.get('checkpoint_identifier'),
                'checkpoint_path': pretrained_report.get('checkpoint_path'),
                'checkpoint_sha256': pretrained_report.get('checkpoint_sha256'),
                'pretrained_requested': bool(pretrained_report.get('pretrained_requested')),
                'pretrained_loaded': bool(pretrained_report.get('pretrained_loaded')),
            },
            'adaptation': {
                'mode': adaptation_mode,
                'lora_rank': args.visual_lora_rank if adaptation_mode == VISUAL_ADAPTATION_LORA else 0,
                'trainable_scope': variant_config['visual_model']['backbone_trainable_scope'],
            },
            'parameters': counts,
            'image_preprocessing': variant_config['visual_model']['image_preprocessing'],
            'seed': seed_report,
            'dataset_split_paths': config['dataset_split_paths'],
            'loss': config['loss'],
            'augmentations': config['augmentations'],
        }
        print(
            _pretty_json({
                'event': 'pretrained_backbone_verified',
                'visual_adaptation': adaptation_mode,
                **variant_config['run_metadata']['backbone'],
            }),
            flush=True,
        )
        _write_visual_training_artifacts(
            run_dir,
            dataset_report=dataset_report,
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
        history: list[dict[str, Any]] = []
        best_val_loss = float('inf')
        best_epoch = 0
        epochs_without_improvement = 0
        start_epoch = 1
        resume_report: dict[str, Any] = {
            'requested': False,
            'loaded': False,
        }
        if args.resume_checkpoint is not None:
            resume_path = args.resume_checkpoint.expanduser().resolve()
            if not resume_path.is_file():
                raise FileNotFoundError(
                    f'resume checkpoint not found: {resume_path}'
                )
            resume_checkpoint = _torch_load_checkpoint(
                torch_module,
                resume_path,
            )
            resume_config = _checkpoint_training_config(
                resume_checkpoint,
                resume_path,
            )
            if (
                resume_config.get('visual_model_kind')
                != VISUAL_MODEL_KIND
                or resume_config.get('visual_adaptation')
                != adaptation_mode
                or int(resume_config.get('output_dim', -1))
                != output_dim
            ):
                raise ValueError(
                    'resume checkpoint model contract does not match '
                    'the requested fixed-eight run'
                )
            model.load_state_dict(
                resume_checkpoint['model_state_dict'],
                strict=True,
            )
            optimizer.load_state_dict(
                resume_checkpoint['optimizer_state_dict'],
            )
            resumed_epoch = int(resume_checkpoint.get('epoch') or 0)
            start_epoch = resumed_epoch + 1
            prior_summary_path = resume_path.parent / 'metrics.json'
            if prior_summary_path.is_file():
                prior_summary = _load_json(prior_summary_path)
                history = list(prior_summary.get('history') or [])
                best_val_loss = float(
                    prior_summary.get('best_val_loss', float('inf'))
                )
                best_epoch = int(
                    prior_summary.get('best_epoch') or 0
                )
                prior_best = resume_path.parent / 'best.pt'
                if prior_best.is_file():
                    shutil.copy2(prior_best, run_dir / 'best.pt')
            if not (run_dir / 'best.pt').is_file():
                shutil.copy2(resume_path, run_dir / 'best.pt')
                metrics = resume_checkpoint.get('metrics') or {}
                best_val_loss = float(
                    metrics.get(
                        'val_total_weighted_loss',
                        best_val_loss,
                    )
                )
                best_epoch = resumed_epoch
            if start_epoch > args.epochs:
                raise ValueError(
                    '--epochs must exceed the resume checkpoint epoch'
                )
            resume_report = {
                'requested': True,
                'loaded': True,
                'checkpoint_path': str(resume_path),
                'checkpoint_sha256': _sha256_file(resume_path),
                'checkpoint_epoch': resumed_epoch,
                'next_epoch': start_epoch,
            }
            variant_config['resume'] = resume_report
            _write_visual_training_artifacts(
                run_dir,
                dataset_report=dataset_report,
                label_vectorizer=label_vectorizer,
                target_mean=target_mean,
                target_std=target_std,
                config=variant_config,
            )
        stopped_early = False
        training_started = time.perf_counter()
        overall_peak_gpu_memory = {
            'device': device,
            'cuda_available': bool(torch_module.cuda.is_available()),
            'allocated_bytes': None,
            'reserved_bytes': None,
        }

        for epoch in range(start_epoch, args.epochs + 1):
            epoch_started = time.perf_counter()
            if device == 'cuda' and torch_module.cuda.is_available():
                torch_module.cuda.reset_peak_memory_stats()
            model.train()
            optimization_loss_accumulator = _new_visual_loss_accumulator()
            for batch in train_loader:
                image = batch['image'].to(device)
                target = batch['target'].to(device)
                target_mask = batch['target_mask'].to(device)
                optimizer.zero_grad(set_to_none=True)
                pred = model(image)
                loss_components = visual_loss_components(
                    torch_module,
                    pred,
                    target,
                    target_mask,
                    label_vectorizer.names,
                )
                loss = loss_components['total_weighted_loss']
                loss.backward()
                optimizer.step()
                _accumulate_visual_loss(
                    optimization_loss_accumulator,
                    loss_components,
                )
            optimization_metrics = _summarize_visual_loss(
                optimization_loss_accumulator
            )
            train_metrics = _evaluate_visual_state(
                torch_module,
                model,
                train_eval_loader,
                device=device,
                target_mean=target_mean,
                target_std=target_std,
                label_names=label_vectorizer.names,
            )
            val_metrics = _evaluate_visual_state(
                torch_module,
                model,
                val_loader,
                device=device,
                target_mean=target_mean,
                target_std=target_std,
                label_names=label_vectorizer.names,
            )
            if device == 'cuda' and torch_module.cuda.is_available():
                torch_module.cuda.synchronize()
            epoch_peak_gpu_memory = _peak_gpu_memory(torch_module, device)
            for memory_key in ('allocated_bytes', 'reserved_bytes'):
                value = epoch_peak_gpu_memory.get(memory_key)
                if value is not None:
                    previous = overall_peak_gpu_memory.get(memory_key)
                    overall_peak_gpu_memory[memory_key] = max(
                        int(previous or 0),
                        int(value),
                    )
            epoch_metrics = {
                'epoch': epoch,
                'visual_adaptation': adaptation_mode,
                'epoch_runtime_seconds': round(
                    time.perf_counter() - epoch_started,
                    3,
                ),
                'learning_rates': [
                    float(group['lr'])
                    for group in optimizer.param_groups
                ],
                'epoch_peak_gpu_memory': epoch_peak_gpu_memory,
                'parameter_counts': counts,
                **{
                    f'optimization_{key}': value
                    for key, value in optimization_metrics.items()
                },
                **{f'train_{key}': value for key, value in train_metrics.items()},
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
            current_val_loss = float(val_metrics['total_weighted_loss'])
            if current_val_loss < best_val_loss:
                best_val_loss = current_val_loss
                best_epoch = epoch
                epochs_without_improvement = 0
                _save_checkpoint(
                    torch_module,
                    run_dir / 'best.pt',
                    model=model,
                    optimizer=optimizer,
                    epoch=epoch,
                    metrics=epoch_metrics,
                    config=variant_config,
                )
            else:
                epochs_without_improvement += 1
            if (
                args.early_stopping_patience > 0
                and epochs_without_improvement >= args.early_stopping_patience
            ):
                stopped_early = True
                print(
                    _pretty_json({
                        'event': 'early_stopping',
                        'epoch': epoch,
                        'patience': args.early_stopping_patience,
                        'best_epoch': best_epoch,
                        'best_validation_total_weighted_loss': best_val_loss,
                    }),
                    flush=True,
                )
                break

        checkpoint_fingerprints = {
            'best': _file_fingerprint(run_dir / 'best.pt'),
            'last': _file_fingerprint(run_dir / 'last.pt'),
        }

        variant_summary = {
            'visual_adaptation': adaptation_mode,
            'output_dir': str(run_dir),
            'best_checkpoint': str(run_dir / 'best.pt'),
            'last_checkpoint': str(run_dir / 'last.pt'),
            'checkpoint_fingerprints': checkpoint_fingerprints,
            'best_epoch': best_epoch,
            'best_val_loss': best_val_loss,
            'best_checkpoint_selection': 'validation_total_weighted_loss_only',
            'test_evaluations_during_training': 0,
            'epochs_completed': len(history),
            'early_stopping_patience': args.early_stopping_patience,
            'stopped_early': stopped_early,
            'training_runtime_seconds': round(
                time.perf_counter() - training_started,
                3,
            ),
            'history': history,
            'resume': resume_report,
            'parameter_counts': counts,
            'visual_model': variant_config['visual_model'],
            'run_metadata': variant_config['run_metadata'],
            'peak_gpu_memory': overall_peak_gpu_memory,
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
            'visual_label_vectorizer.json',
            'target_stats.json',
            'training_config.json',
            'run_metadata.json',
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
        'best_checkpoint_selection': selected['best_checkpoint_selection'],
        'test_evaluations_during_training': selected['test_evaluations_during_training'],
        'checkpoint_fingerprints': {
            'best': _file_fingerprint(output_dir / 'best.pt'),
            'last': _file_fingerprint(output_dir / 'last.pt'),
        },
        'epochs_completed': selected['epochs_completed'],
        'early_stopping_patience': selected['early_stopping_patience'],
        'stopped_early': selected['stopped_early'],
        'training_runtime_seconds': selected['training_runtime_seconds'],
        'history': selected['history'],
        'variant_results': variant_results,
        'parameter_counts': selected['parameter_counts'],
        'visual_model': selected['visual_model'],
        'run_metadata': selected['run_metadata'],
        'peak_gpu_memory': selected['peak_gpu_memory'],
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
            split_loss_accumulator = _new_visual_loss_accumulator()
            for sample_index, (row, label) in enumerate(zip(rows, labels)):
                cycle_start = time.perf_counter()
                image = torch_module.as_tensor(
                    load_paired_images(
                        row,
                        dataset_root,
                        width=image_width,
                        height=image_height,
                        allow_blank_images=args.allow_blank_images,
                        dataset_mode=DATASET_MODE_VISUAL_STATE,
                        normalization_mean=VISUAL_BACKBONE_NORMALIZATION_MEAN,
                        normalization_std=VISUAL_BACKBONE_NORMALIZATION_STD,
                    ),
                    dtype=torch_module.float32,
                    device=device,
                ).unsqueeze(0)
                if device == 'cuda' and torch_module.cuda.is_available():
                    torch_module.cuda.synchronize()
                inference_start = time.perf_counter()
                pred_norm = model(image)
                if device == 'cuda' and torch_module.cuda.is_available():
                    torch_module.cuda.synchronize()
                inference_latency = time.perf_counter() - inference_start
                pred = (pred_norm[0] * std_tensor + mean_tensor).detach().cpu().numpy()
                true_raw = np.asarray(label_vectorizer.transform(label), dtype=np.float32)
                target_mask = np.asarray(
                    label_vectorizer.target_mask(label),
                    dtype=np.float32,
                )
                true_normalized = (true_raw - target_mean) / target_std
                loss_components = visual_loss_components(
                    torch_module,
                    pred_norm,
                    torch_module.as_tensor(
                        true_normalized,
                        dtype=torch_module.float32,
                        device=device,
                    ).unsqueeze(0),
                    torch_module.as_tensor(
                        target_mask,
                        dtype=torch_module.float32,
                        device=device,
                    ).unsqueeze(0),
                    label_vectorizer.names,
                )
                _accumulate_visual_loss(
                    split_loss_accumulator,
                    loss_components,
                )
                record = {
                    'split': split_name,
                    'sample_index': sample_index,
                    'episode_id': str(row.get('episode_id') or ''),
                    'scenario_family': str(row.get('scenario_family') or ''),
                    'true_raw': true_raw.tolist(),
                    'pred_raw': pred[: len(true_raw)].astype(np.float32).tolist(),
                    'target_mask': target_mask.tolist(),
                    'inference_latency_seconds': inference_latency,
                    'cycle_time_seconds': time.perf_counter() - cycle_start,
                }
                split_records.append(record)
                family_key = record['scenario_family'] or 'unknown'
                family_records.setdefault(family_key, []).append(record)
            split_metrics = visual_state_metrics(
                split_records,
                label_vectorizer.names,
            )
            split_metrics.update(
                _summarize_visual_loss(split_loss_accumulator)
            )
            _add_visual_per_head_metrics(split_metrics)
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
                'metrics': split_metrics,
            }
            all_records.extend(split_records)

    summary_path = output_dir / 'visual_state_eval.json'
    summary = {
        'tool': 'room_315_vla_train_local',
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'purpose': 'small custom visual-state label checkpoint evaluation',
        'checkpoint': str(checkpoint_path),
        'checkpoint_fingerprint': _file_fingerprint(checkpoint_path),
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
                'visual_label_vectorizer': _artifact_path(checkpoint_path, 'visual_label_vectorizer.json'),
                'target_stats': _artifact_path(checkpoint_path, 'target_stats.json'),
                'training_config': _artifact_path(checkpoint_path, 'training_config.json'),
                'run_metadata': _artifact_path(checkpoint_path, 'run_metadata.json'),
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


def validate_test_lock(args: argparse.Namespace) -> dict[str, Any]:
    """Fail closed unless test evaluation is explicit and unlocked."""
    if args.eval_checkpoint is None:
        selected = {
            Path(str(args.train_file)).name.casefold(),
            Path(str(args.val_file)).name.casefold(),
        }
        if any(name.startswith('test') for name in selected):
            raise ValueError(
                'normal training may not load test split files'
            )
        if args.unlock_test:
            raise ValueError(
                '--unlock-test is valid only for explicit checkpoint evaluation'
            )
        return {
            'mode': 'training',
            'test_loaded': False,
            'test_unlocked': False,
        }
    evaluated = {
        name.strip().casefold()
        for name in str(args.eval_splits or '').split(',')
        if name.strip()
    }
    test_selected = 'test' in evaluated
    if test_selected and not args.unlock_test:
        raise ValueError(
            'test evaluation is locked; pass --unlock-test explicitly'
        )
    if args.unlock_test and not test_selected:
        raise ValueError(
            '--unlock-test requires test in --eval-splits'
        )
    return {
        'mode': 'evaluation',
        'test_loaded': test_selected,
        'test_unlocked': bool(args.unlock_test),
        'evaluated_splits': sorted(evaluated),
    }


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
    parser.add_argument(
        '--early-stopping-patience',
        type=int,
        default=0,
        help=(
            'Stop after this many consecutive epochs without validation total-loss '
            'improvement. Zero disables early stopping.'
        ),
    )
    parser.add_argument('--learning-rate', type=float, default=1e-3)
    parser.add_argument('--weight-decay', type=float, default=1e-4)
    parser.add_argument('--image-width', type=int, default=VISUAL_BACKBONE_INPUT_WIDTH)
    parser.add_argument('--image-height', type=int, default=VISUAL_BACKBONE_INPUT_HEIGHT)
    parser.add_argument('--num-workers', type=int, default=2)
    parser.add_argument('--device', default='auto', help='auto, cuda, or cpu')
    parser.add_argument('--seed', type=int, default=13)
    parser.add_argument('--limit-train-rows', type=int, default=None)
    parser.add_argument('--limit-val-rows', type=int, default=None)
    parser.add_argument(
        '--resume-checkpoint',
        type=Path,
        default=None,
        help=(
            'Resume one explicit adaptation from a prior last.pt/best.pt. '
            '--epochs is the maximum total epoch number.'
        ),
    )
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
            'Visual-state only: train the official pretrained backbone frozen, with a '
            'low-rank feature adapter, with ResNet layer4 trainable, or compare frozen/LoRA.'
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
        choices=(
            'best_val_loss',
            VISUAL_ADAPTATION_FROZEN_BACKBONE,
            VISUAL_ADAPTATION_LORA,
            VISUAL_ADAPTATION_PARTIAL_FINETUNE,
        ),
        default='best_val_loss',
        help='Visual-state only: selected checkpoint copied to the root output directory after compare.',
    )
    parser.add_argument(
        '--visual-pretrained-backbone',
        default=VISUAL_BACKBONE_SOURCE,
        help=(
            f'Official pretrained identifier (default: {VISUAL_BACKBONE_SOURCE}). '
            'Use "none" only for an explicit random-weight ablation.'
        ),
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
        default='val',
        help='Comma-separated JSONL split names to evaluate with --eval-checkpoint.',
    )
    parser.add_argument(
        '--unlock-test',
        action='store_true',
        help=(
            'Explicitly unlock test evaluation. This is rejected during '
            'training and is required when --eval-splits contains test.'
        ),
    )
    parser.add_argument(
        '--limit-eval-rows',
        type=int,
        default=None,
        help='Optional debug row limit per split during --eval-checkpoint.',
    )
    return parser


def main(argv: list[str] | None = None) -> None:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    try:
        test_lock = validate_test_lock(args)
    except ValueError as exc:
        parser.error(str(exc))
    if args.eval_checkpoint is not None:
        summary = evaluate_checkpoint(args)
    else:
        summary = train_visual_state(args)
    if isinstance(summary, dict):
        summary['test_lock'] = test_lock
    print(_pretty_json(summary))


if __name__ == '__main__':
    main()
