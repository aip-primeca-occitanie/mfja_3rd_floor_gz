#!/usr/bin/env python3
"""Pure runtime for the approved Room 315 paired-camera visual-state model."""

from __future__ import annotations

import hashlib
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_presence_provider import PRESENCE_ABSENT
from room_315_presence_provider import PRESENCE_PRESENT
from room_315_presence_provider import PRESENCE_UNKNOWN
from room_315_presence_provider import PresenceSnapshot
from room_315_visual_model import build_visual_state_model
from room_315_visual_state_dataset import VisualStateLabelVectorizer


MODEL_SCHEMA = 'room315.visual_state.v3'
MODEL_KIND = 'structured_visual_state_torchvision_resnet18_fixed8_v3'
OUTPUT_DIM = 200
FIXED_IDENTITY_ORDER = ('L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4')
IMAGE_SIZE = (224, 224)
IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)
REQUIRED_SIDECARS = (
    'target_stats.json',
    'visual_label_vectorizer.json',
    'training_config.json',
    'run_metadata.json',
)


class VisualRuntimeError(RuntimeError):
    """Raised when artifact, input, or inference contracts fail closed."""


@dataclass(frozen=True)
class ArtifactPaths:
    checkpoint: Path
    sidecar_directory: Path

    def path_for(self, name: str) -> Path:
        if name == 'best.pt':
            return self.checkpoint
        return self.sidecar_directory / name


@dataclass(frozen=True)
class ArtifactHashes:
    checkpoint: str
    target_stats: str
    vectorizer: str
    training_config: str
    run_metadata: str
    runtime_configuration: str = ''

    def as_filenames(self) -> dict[str, str]:
        result = {
            'best.pt': self.checkpoint,
            'target_stats.json': self.target_stats,
            'visual_label_vectorizer.json': self.vectorizer,
            'training_config.json': self.training_config,
            'run_metadata.json': self.run_metadata,
        }
        if self.runtime_configuration:
            result['runtime_configuration.json'] = self.runtime_configuration
        return result


@dataclass(frozen=True)
class VerifiedArtifacts:
    paths: ArtifactPaths
    hashes: dict[str, str]
    target_mean: np.ndarray
    target_std: np.ndarray
    vectorizer: VisualStateLabelVectorizer
    vectorizer_json: dict[str, Any]
    training_config: dict[str, Any]
    run_metadata: dict[str, Any]
    runtime_configuration: dict[str, Any]
    expected_checkpoint_epoch: int


@dataclass(frozen=True)
class DecodedShuttlePrediction:
    identity: str
    side: str
    block: str
    bbox_xywh: tuple[float, float, float, float]
    s_m: float
    s_ratio: float
    segment_length_m: float
    loaded_state: str


@dataclass(frozen=True)
class DecodedVisualPrediction:
    timestamp_s: float
    left_image_stamp_s: float
    right_image_stamp_s: float
    left_image_size: tuple[int, int]
    right_image_size: tuple[int, int]
    shuttles: tuple[DecodedShuttlePrediction, ...]
    active_identities: tuple[str, ...]
    absent_identities: tuple[str, ...]


@dataclass(frozen=True)
class InferenceTimings:
    preprocessing_ms: float
    inference_ms: float
    decode_ms: float
    complete_cycle_ms: float


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def verify_artifacts(
    paths: ArtifactPaths,
    expected_hashes: ArtifactHashes,
) -> VerifiedArtifacts:
    """Verify all non-checkpoint metadata before importing Torch."""

    expected = expected_hashes.as_filenames()
    actual: dict[str, str] = {}
    for filename, expected_digest in expected.items():
        path = paths.path_for(filename)
        if not path.is_file():
            raise VisualRuntimeError(f'required model artifact is missing: {path}')
        digest = sha256_file(path)
        actual[filename] = digest
        if digest != str(expected_digest).strip().lower():
            raise VisualRuntimeError(
                f'artifact SHA-256 mismatch for {path}: '
                f'expected {expected_digest}, got {digest}'
            )

    target_stats = _load_json(paths.path_for('target_stats.json'))
    vectorizer_json = _load_json(paths.path_for('visual_label_vectorizer.json'))
    training_config = _load_json(paths.path_for('training_config.json'))
    run_metadata = _load_json(paths.path_for('run_metadata.json'))
    runtime_configuration: dict[str, Any] = {}
    if expected_hashes.runtime_configuration:
        runtime_configuration = _load_json(
            paths.path_for('runtime_configuration.json')
        )

    vectorizer = VisualStateLabelVectorizer.from_json(vectorizer_json)
    if vectorizer_json.get('schema_version') != MODEL_SCHEMA:
        raise VisualRuntimeError('vectorizer schema version is not room315.visual_state.v3')
    if vectorizer.dim != OUTPUT_DIM:
        raise VisualRuntimeError(f'expected vector dimension 200, got {vectorizer.dim}')
    if (
        tuple(vectorizer_json.get('fixed_identity_order') or ())
        != FIXED_IDENTITY_ORDER
    ):
        raise VisualRuntimeError('fixed identity order does not match the approved contract')
    if training_config.get('visual_model_kind') != MODEL_KIND:
        raise VisualRuntimeError('training config model kind does not match approved runtime')
    if int(training_config.get('output_dim', -1)) != OUTPUT_DIM:
        raise VisualRuntimeError('training config output_dim is not 200')
    if training_config.get('image_resize') != 'direct_bilinear_resize':
        raise VisualRuntimeError('training resize contract is not direct bilinear')
    if training_config.get('augmentations') not in ([], None):
        raise VisualRuntimeError('approved runtime requires a no-augmentation model')
    if training_config.get('visual_adaptation') != 'partial_finetune':
        raise VisualRuntimeError('approved checkpoint is not partial layer4 fine-tuning')

    visual_model = training_config.get('visual_model') or {}
    if visual_model.get('model_kind') != MODEL_KIND:
        raise VisualRuntimeError('visual model metadata kind mismatch')
    if visual_model.get('backbone_architecture') != 'resnet18':
        raise VisualRuntimeError('visual model backbone is not ResNet-18')
    if tuple(visual_model.get('image_preprocessing', {}).get('input_resolution') or ()) != IMAGE_SIZE:
        raise VisualRuntimeError('visual model input resolution is not 224x224')

    preprocessing = run_metadata.get('image_preprocessing') or {}
    if tuple(preprocessing.get('normalization_mean_per_rgb_view') or ()) != IMAGENET_MEAN:
        raise VisualRuntimeError('runtime mean does not match approved ImageNet normalization')
    if tuple(preprocessing.get('normalization_std_per_rgb_view') or ()) != IMAGENET_STD:
        raise VisualRuntimeError('runtime std does not match approved ImageNet normalization')

    mean = _finite_vector(target_stats.get('mean'), 'target mean')
    std = _finite_vector(target_stats.get('std'), 'target std')
    if mean.shape != (OUTPUT_DIM,) or std.shape != (OUTPUT_DIM,):
        raise VisualRuntimeError('target mean/std must each contain exactly 200 values')
    if np.any(std <= 0.0):
        raise VisualRuntimeError('all target standard deviations must be positive')

    expected_checkpoint_epoch = 14
    if runtime_configuration:
        expected_checkpoint_epoch = _verify_runtime_configuration(
            runtime_configuration,
            actual_hashes=actual,
        )

    return VerifiedArtifacts(
        paths=paths,
        hashes=actual,
        target_mean=mean,
        target_std=std,
        vectorizer=vectorizer,
        vectorizer_json=vectorizer_json,
        training_config=training_config,
        run_metadata=run_metadata,
        runtime_configuration=runtime_configuration,
        expected_checkpoint_epoch=expected_checkpoint_epoch,
    )


def preprocess_rgb_image(
    rgb_image: np.ndarray,
    *,
    width: int = IMAGE_SIZE[0],
    height: int = IMAGE_SIZE[1],
) -> np.ndarray:
    """Match the training PIL bilinear RGB preprocessing exactly."""

    array = np.asarray(rgb_image)
    if array.ndim != 3 or array.shape[2] != 3:
        raise VisualRuntimeError(
            f'RGB image must have shape [H,W,3], got {array.shape}'
        )
    if array.shape[0] <= 0 or array.shape[1] <= 0:
        raise VisualRuntimeError('RGB image dimensions must be positive')
    if not np.all(np.isfinite(array)):
        raise VisualRuntimeError('RGB image contains non-finite values')
    if array.dtype != np.uint8:
        if np.issubdtype(array.dtype, np.floating):
            if float(array.min()) < 0.0 or float(array.max()) > 1.0:
                raise VisualRuntimeError('floating RGB image must be in [0,1]')
            array = np.rint(array * 255.0).astype(np.uint8)
        else:
            if int(array.min()) < 0 or int(array.max()) > 255:
                raise VisualRuntimeError('integer RGB image must be in [0,255]')
            array = array.astype(np.uint8)
    pil_image = Image.fromarray(array, mode='RGB')
    resized = pil_image.resize((int(width), int(height)), Image.BILINEAR)
    result = np.asarray(resized, dtype=np.float32) / 255.0
    return np.transpose(result, (2, 0, 1))


def preprocess_paired_rgb(
    left_rgb: np.ndarray,
    right_rgb: np.ndarray,
) -> np.ndarray:
    left = preprocess_rgb_image(left_rgb)
    right = preprocess_rgb_image(right_rgb)
    paired = np.concatenate([left, right], axis=0).astype(np.float32)
    mean = np.asarray(IMAGENET_MEAN * 2, dtype=np.float32).reshape(6, 1, 1)
    std = np.asarray(IMAGENET_STD * 2, dtype=np.float32).reshape(6, 1, 1)
    result = ((paired - mean) / std).astype(np.float32)
    if result.shape != (6, IMAGE_SIZE[1], IMAGE_SIZE[0]):
        raise VisualRuntimeError(f'paired preprocessing shape mismatch: {result.shape}')
    if not np.all(np.isfinite(result)):
        raise VisualRuntimeError('preprocessed paired tensor contains non-finite values')
    return result


def denormalize_output(
    normalized_output: Any,
    target_mean: np.ndarray,
    target_std: np.ndarray,
) -> np.ndarray:
    output = np.asarray(normalized_output, dtype=np.float32).reshape(-1)
    if output.shape != (OUTPUT_DIM,):
        raise VisualRuntimeError(f'model output must contain 200 values, got {output.shape}')
    if not np.all(np.isfinite(output)):
        raise VisualRuntimeError('model output contains non-finite values')
    raw = output * target_std + target_mean
    if not np.all(np.isfinite(raw)):
        raise VisualRuntimeError('denormalized model output contains non-finite values')
    return raw.astype(np.float32)


def decode_active_slots(
    raw_output: Any,
    *,
    vectorizer: VisualStateLabelVectorizer,
    presence: PresenceSnapshot,
    timestamp_s: float,
    left_image_stamp_s: float,
    right_image_stamp_s: float,
    left_image_size: tuple[int, int],
    right_image_size: tuple[int, int],
) -> DecodedVisualPrediction:
    """Decode only slots declared present by the deterministic registry."""

    raw = np.asarray(raw_output, dtype=np.float32).reshape(-1)
    if raw.shape != (OUTPUT_DIM,):
        raise VisualRuntimeError(f'decoder expected 200 values, got {raw.shape}')
    if not np.all(np.isfinite(raw)):
        raise VisualRuntimeError('decoder received non-finite model values')
    if not presence.ready:
        raise VisualRuntimeError(
            'presence registry is not ready: ' + ','.join(presence.reasons)
        )
    entries = presence.by_identity()
    if (
        len(presence.entries) != len(FIXED_IDENTITY_ORDER)
        or tuple(entries) != FIXED_IDENTITY_ORDER
    ):
        raise VisualRuntimeError('presence registry identity order is invalid')
    for identity, entry in entries.items():
        expected_side = 'left' if identity.startswith('L') else 'right'
        if entry.side != expected_side:
            raise VisualRuntimeError(
                f'presence identity side conflict: '
                f'{identity}:{entry.side}!={expected_side}'
            )
        if entry.state == PRESENCE_UNKNOWN:
            raise VisualRuntimeError(
                f'presence is unknown for active-slot gating: {identity}'
            )
        if entry.state not in {PRESENCE_PRESENT, PRESENCE_ABSENT}:
            raise VisualRuntimeError(
                f'unsupported presence state for {identity}: {entry.state}'
            )

    names = vectorizer.names
    name_to_index = {name: index for index, name in enumerate(names)}
    active: list[str] = []
    absent: list[str] = []
    predictions: list[DecodedShuttlePrediction] = []
    for slot, identity in enumerate(FIXED_IDENTITY_ORDER):
        entry = entries[identity]
        if entry.state == PRESENCE_ABSENT:
            absent.append(identity)
            continue
        active.append(identity)
        numeric = {
            field: _value_for(
                raw,
                name_to_index,
                f'shuttles.{slot}.{field}',
            )
            for field in (
                'bbox.0',
                'bbox.1',
                'bbox.2',
                'bbox.3',
                'rail_position.s_m',
                'rail_position.s_ratio',
                'rail_position.segment_length_m',
            )
        }
        side = _categorical_argmax(
            raw,
            name_to_index,
            f'shuttles.{slot}.location.side',
            vectorizer.categorical_values[
                f'shuttles.{slot}.location.side'
            ],
        )
        block = _categorical_argmax(
            raw,
            name_to_index,
            f'shuttles.{slot}.location.block',
            vectorizer.categorical_values[
                f'shuttles.{slot}.location.block'
            ],
        )
        loaded = _categorical_argmax(
            raw,
            name_to_index,
            f'shuttles.{slot}.loaded_state',
            vectorizer.categorical_values[
                f'shuttles.{slot}.loaded_state'
            ],
        )
        predictions.append(DecodedShuttlePrediction(
            identity=identity,
            side=side,
            block=block,
            bbox_xywh=tuple(
                numeric[f'bbox.{index}']
                for index in range(4)
            ),
            s_m=numeric['rail_position.s_m'],
            s_ratio=numeric['rail_position.s_ratio'],
            segment_length_m=numeric['rail_position.segment_length_m'],
            loaded_state=loaded,
        ))

    return DecodedVisualPrediction(
        timestamp_s=_finite_nonnegative(timestamp_s, 'prediction timestamp'),
        left_image_stamp_s=_finite_nonnegative(
            left_image_stamp_s,
            'left image timestamp',
        ),
        right_image_stamp_s=_finite_nonnegative(
            right_image_stamp_s,
            'right image timestamp',
        ),
        left_image_size=_image_size(left_image_size, 'left image size'),
        right_image_size=_image_size(right_image_size, 'right image size'),
        shuttles=tuple(predictions),
        active_identities=tuple(active),
        absent_identities=tuple(absent),
    )


class Room315VisualModelRuntime:
    """Strict artifact loader and deterministic paired-image inference API."""

    def __init__(
        self,
        artifacts: VerifiedArtifacts,
        *,
        device: str = 'auto',
    ) -> None:
        self.artifacts = artifacts
        self.requested_device = str(device or 'auto').strip().lower()
        self.device = ''
        self.model = None
        self.torch = None
        self.torchvision = None
        self.ready = False
        self.model_load_duration_ms: float | None = None

    def load(self) -> None:
        started = time.perf_counter()
        try:
            import torch
            import torchvision
        except ImportError as exc:
            raise VisualRuntimeError(
                'Torch and TorchVision are required to load the visual runtime'
            ) from exc

        self.torch = torch
        self.torchvision = torchvision
        self.device = _choose_device(torch, self.requested_device)
        config = self.artifacts.training_config
        model = build_visual_state_model(
            torch,
            torchvision,
            output_dim=OUTPUT_DIM,
            adaptation_mode='partial_finetune',
            lora_rank=int(config.get('visual_lora_rank', 4)),
        )
        checkpoint = _torch_load(torch, self.artifacts.paths.checkpoint)
        if int(checkpoint.get('epoch', -1)) != self.artifacts.expected_checkpoint_epoch:
            raise VisualRuntimeError(
                'checkpoint epoch does not match the signed runtime contract: '
                f'expected {self.artifacts.expected_checkpoint_epoch}, '
                f'got {checkpoint.get("epoch")!r}'
            )
        if self.artifacts.runtime_configuration:
            if checkpoint.get('label_vectorizer') != self.artifacts.vectorizer_json:
                raise VisualRuntimeError(
                    'checkpoint-embedded vectorizer does not match the signed sidecar'
                )
            if checkpoint.get('target_stats') != _load_json(
                self.artifacts.paths.path_for('target_stats.json')
            ):
                raise VisualRuntimeError(
                    'checkpoint-embedded target statistics do not match the signed sidecar'
                )
        state = checkpoint.get('model_state_dict')
        if not isinstance(state, dict):
            raise VisualRuntimeError('checkpoint does not contain model_state_dict')
        try:
            model.load_state_dict(state, strict=True)
        except Exception as exc:
            raise VisualRuntimeError('checkpoint strict model loading failed') from exc
        first_convolution = getattr(model.backbone, 'conv1', None)
        if first_convolution is None or int(first_convolution.in_channels) != 3:
            raise VisualRuntimeError('each paired-image branch must accept exactly RGB')
        final_head = model.head[-1]
        if int(getattr(final_head, 'out_features', -1)) != OUTPUT_DIM:
            raise VisualRuntimeError('strictly loaded prediction head is not dimension 200')
        for name, parameter in model.named_parameters():
            if not bool(torch.isfinite(parameter.detach()).all().item()):
                raise VisualRuntimeError(f'model parameter is non-finite: {name}')
        model.to(self.device)
        model.eval()
        self.model = model
        self.ready = True
        self.model_load_duration_ms = (time.perf_counter() - started) * 1000.0

    def infer(
        self,
        left_rgb: np.ndarray,
        right_rgb: np.ndarray,
    ) -> tuple[np.ndarray, InferenceTimings]:
        if not self.ready or self.model is None or self.torch is None:
            raise VisualRuntimeError('model runtime is not ready')
        cycle_started = time.perf_counter()
        prep_started = time.perf_counter()
        paired = preprocess_paired_rgb(left_rgb, right_rgb)
        preprocessing_ms = (time.perf_counter() - prep_started) * 1000.0

        infer_started = time.perf_counter()
        tensor = self.torch.from_numpy(paired[None]).to(self.device)
        with self.torch.inference_mode():
            prediction = self.model(tensor)
        if tuple(prediction.shape) != (1, OUTPUT_DIM):
            raise VisualRuntimeError(
                f'model returned incompatible shape: {tuple(prediction.shape)}'
            )
        normalized = prediction.detach().to('cpu').numpy()[0]
        inference_ms = (time.perf_counter() - infer_started) * 1000.0

        decode_started = time.perf_counter()
        raw = denormalize_output(
            normalized,
            self.artifacts.target_mean,
            self.artifacts.target_std,
        )
        decode_ms = (time.perf_counter() - decode_started) * 1000.0
        return raw, InferenceTimings(
            preprocessing_ms=preprocessing_ms,
            inference_ms=inference_ms,
            decode_ms=decode_ms,
            complete_cycle_ms=(time.perf_counter() - cycle_started) * 1000.0,
        )


def _load_json(path: Path) -> dict[str, Any]:
    try:
        parsed = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise VisualRuntimeError(f'cannot read JSON artifact: {path}') from exc
    if not isinstance(parsed, dict):
        raise VisualRuntimeError(f'JSON artifact must contain an object: {path}')
    return parsed


def _finite_vector(value: Any, name: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float32)
    except (TypeError, ValueError) as exc:
        raise VisualRuntimeError(f'{name} must be numeric') from exc
    if result.ndim != 1 or not np.all(np.isfinite(result)):
        raise VisualRuntimeError(f'{name} must be a finite one-dimensional vector')
    return result


def _verify_runtime_configuration(
    configuration: dict[str, Any],
    *,
    actual_hashes: dict[str, str],
) -> int:
    if configuration.get('schema_version') != 'room315.visual_runtime_candidate.v1':
        raise VisualRuntimeError('runtime configuration schema is unsupported')
    if configuration.get('deployment_state') != 'candidate':
        raise VisualRuntimeError('runtime configuration must remain a candidate')
    contract = configuration.get('model_contract') or {}
    if int(contract.get('output_dimension', -1)) != OUTPUT_DIM:
        raise VisualRuntimeError('runtime contract output dimension is not 200')
    if tuple(contract.get('identity_order') or ()) != FIXED_IDENTITY_ORDER:
        raise VisualRuntimeError('runtime contract identity order is incompatible')
    if tuple(contract.get('paired_rgb_input_shape') or ()) != ('B', 6, 224, 224):
        raise VisualRuntimeError('runtime contract input must be [B,6,224,224]')
    if contract.get('vectorizer_schema') != MODEL_SCHEMA:
        raise VisualRuntimeError('runtime contract vectorizer schema is incompatible')
    if contract.get('checkpoint_loading') != 'strict':
        raise VisualRuntimeError('runtime contract must require strict checkpoint loading')

    configured_hashes = configuration.get('artifact_sha256') or {}
    for filename, actual_digest in actual_hashes.items():
        if filename == 'runtime_configuration.json':
            continue
        if configured_hashes.get(filename) != actual_digest:
            raise VisualRuntimeError(
                f'runtime configuration hash mismatch for {filename}'
            )
    try:
        epoch = int(configuration['checkpoint']['epoch'])
    except (KeyError, TypeError, ValueError) as exc:
        raise VisualRuntimeError(
            'runtime configuration checkpoint epoch is missing or invalid'
        ) from exc
    if epoch < 0:
        raise VisualRuntimeError('runtime configuration checkpoint epoch is negative')
    return epoch


def _torch_load(torch_module: Any, path: Path) -> dict[str, Any]:
    try:
        loaded = torch_module.load(path, map_location='cpu', weights_only=False)
    except TypeError:
        loaded = torch_module.load(path, map_location='cpu')
    except Exception as exc:
        raise VisualRuntimeError(f'cannot load checkpoint: {path}') from exc
    if not isinstance(loaded, dict):
        raise VisualRuntimeError('checkpoint root must be a dictionary')
    return loaded


def _choose_device(torch_module: Any, requested: str) -> str:
    if requested not in {'auto', 'cpu', 'cuda'}:
        raise VisualRuntimeError('device must be auto, cpu, or cuda')
    if requested == 'cuda':
        if not torch_module.cuda.is_available():
            raise VisualRuntimeError('CUDA was requested but is not available')
        return 'cuda'
    if requested == 'auto':
        return 'cuda' if torch_module.cuda.is_available() else 'cpu'
    return 'cpu'


def _categorical_argmax(
    raw: np.ndarray,
    name_to_index: dict[str, int],
    base: str,
    values: list[str],
) -> str:
    if not values:
        raise VisualRuntimeError(f'categorical vocabulary is empty: {base}')
    indexes = [
        name_to_index.get(f'{base}=={value}')
        for value in values
    ]
    if any(index is None for index in indexes):
        raise VisualRuntimeError(f'categorical vector indexes are incomplete: {base}')
    best = max(
        range(len(indexes)),
        key=lambda offset: float(raw[int(indexes[offset])]),
    )
    return str(values[best])


def _value_for(
    raw: np.ndarray,
    name_to_index: dict[str, int],
    name: str,
) -> float:
    index = name_to_index.get(name)
    if index is None:
        raise VisualRuntimeError(f'output vector is missing {name}')
    return float(raw[index])


def _finite_nonnegative(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise VisualRuntimeError(f'{name} must be numeric') from exc
    if not math.isfinite(result) or result < 0.0:
        raise VisualRuntimeError(f'{name} must be finite and non-negative')
    return result


def _image_size(value: tuple[int, int], name: str) -> tuple[int, int]:
    if len(value) != 2:
        raise VisualRuntimeError(f'{name} must contain width and height')
    width, height = int(value[0]), int(value[1])
    if width <= 0 or height <= 0:
        raise VisualRuntimeError(f'{name} must be positive')
    return width, height
