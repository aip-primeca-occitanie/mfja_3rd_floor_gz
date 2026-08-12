#!/usr/bin/env python3
"""Fail-closed runtime core for the manually reviewed Room 315 V4 model.

The runtime is intentionally split into two trust phases.  Promotion metadata,
JSON evidence, and every artifact digest are verified without importing Torch.
Only a :class:`VerifiedV4RuntimePromotion` can then construct the model loader.

V4's raw 200-value compatibility vector is diagnostic-only.  Control callers
must use :func:`decode_active_slots_v4`, which applies deterministic presence
gating and rejects the complete active frame when any present slot misses an
explicit confidence threshold from the immutable promotion manifest.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import sys
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any

import numpy as np
from PIL import Image


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_presence_provider import (  # noqa: E402
    PRESENCE_ABSENT,
    PRESENCE_PRESENT,
    PRESENCE_UNKNOWN,
    PresenceSnapshot,
)
from room_315_visual_contract_v4 import (  # noqa: E402
    AcceptanceEnvelope,
    CanonicalSlotPrediction,
    LegacyCompatibilityOutput,
    PublicSegmentLengthContract,
    SEGMENT_CLASSES,
    SIDES,
    SlotAcceptance,
    StructuredSlotPrediction,
    VisualContractV4Error,
    assemble_legacy_200,
    derive_side,
    load_authoritative_public_segment_length_contract,
)
from room_315_visual_model_v4 import (  # noqa: E402
    V4_MODEL_KIND,
    V4_OUTPUT_KEYS,
    V4_SLOT_ORDER,
    build_visual_state_model_v4,
)
from room_315_visual_runtime import (  # noqa: E402
    DecodedShuttlePrediction,
    DecodedVisualPrediction,
    InferenceTimings,
    VisualRuntimeError,
)


PROMOTION_SCHEMA_VERSION = 'room315.visual_runtime_promotion.v4.v1'
CHECKPOINT_SCHEMA_VERSION = 'room315.visual_training.v4.checkpoint.v1'
TRAINING_RUN_SCHEMA_VERSION = 'room315.visual_training.v4.run.v1'
SEGMENT_CALIBRATION_SCHEMA_VERSION = (
    'room315.visual_segment_calibration.v4.v1'
)
VALIDATION_ACCEPTANCE_SCHEMA_VERSION = 'room315.visual_acceptance.v4.v1'
PUBLIC_TOPOLOGY_SCHEMA_VERSION = 'room315.public_segment_length_contract.v1'
CANARY_ATTEMPT_SCHEMA_VERSION = 'room315.visual_v4.canary_attempt.v1'
DIAGNOSTIC_SCHEMA_VERSION = 'room315.visual_runtime_v4.diagnostic.v1'
MANUAL_DECISION_SCHEMA_VERSION = (
    'room315.visual_runtime_v4.manual_decision.v1'
)

SHADOW_AUTHORIZATION_SCOPE = 'gazebo_v4_shadow_observation_only'
DRY_RUN_AUTHORIZATION_SCOPE = 'gazebo_runtime_dry_run_only'
CLOSED_LOOP_QUALIFICATION_SCOPE = (
    'gazebo_v4_closed_loop_qualification_only'
)
CLOSED_LOOP_RUNTIME_SCOPE = 'gazebo_v4_closed_loop_runtime_only'

CHECKPOINT_EPOCH = 11
IMAGE_SIZE = (320, 240)
CAMERA_ORDER = ('left_rail_rgb', 'right_rail_rgb')
CHANNEL_ORDER = ('left_rgb', 'right_rgb')
INPUT_SHAPE = ('B', 6, 240, 320)
IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)
LOADED_CLASSES = ('empty', 'loaded')
REQUIRED_ARTIFACTS = (
    'checkpoint',
    'training_final_report',
    'validation_acceptance',
    'canary_final_report',
    'canary_completion_ledger',
    'effective_config',
    'validation_segment_calibration',
    'public_topology_contract',
)
SHA256_PATTERN = re.compile(r'[0-9a-f]{64}')


class VisualRuntimeV4Error(VisualRuntimeError):
    """Raised whenever the V4 trust, input, or inference contract fails."""


@dataclass(frozen=True, slots=True)
class ArtifactReferenceV4:
    path: Path
    sha256: str


@dataclass(frozen=True, slots=True)
class V4AcceptanceThresholds:
    minimum_segment_confidence: float
    minimum_loaded_confidence: float
    active_frame_policy: str


@dataclass(frozen=True)
class VerifiedV4RuntimePromotion:
    """Metadata-only proof required before importing Torch or loading weights."""

    manifest_path: Path
    manifest_sha256: str
    deployment_mode: str
    authorization_scope: str
    runtime_guards: Mapping[str, bool]
    artifacts: Mapping[str, ArtifactReferenceV4]
    checkpoint_epoch: int
    effective_config_fingerprint_sha256: str
    head_type: str
    hidden_dim: int
    attention_heads: int
    dropout: float
    segment_temperature: float
    thresholds: V4AcceptanceThresholds
    topology: PublicSegmentLengthContract

    @property
    def checkpoint_sha256(self) -> str:
        return self.artifact('checkpoint').sha256

    @property
    def model_kind(self) -> str:
        return V4_MODEL_KIND

    def artifact(self, name: str) -> ArtifactReferenceV4:
        try:
            return self.artifacts[name]
        except KeyError as exc:
            raise VisualRuntimeV4Error(
                f'verified V4 promotion has no artifact named {name!r}'
            ) from exc


@dataclass(frozen=True)
class StructuredVisualOutputV4:
    """Finite, unbatched model outputs in the canonical eight-slot order."""

    segment_logits: np.ndarray
    loaded_logits: np.ndarray
    bbox: np.ndarray
    s_ratio: np.ndarray


@dataclass(frozen=True)
class DiagnosticLegacyOutputV4:
    """A deliberately non-control V3 adapter payload for observability only."""

    compatibility: LegacyCompatibilityOutput
    schema_version: str = field(
        default=DIAGNOSTIC_SCHEMA_VERSION,
        init=False,
    )
    control_input_permitted: bool = field(default=False, init=False)

    @property
    def legacy_vector(self) -> tuple[float, ...]:
        return self.compatibility.legacy_vector

    @property
    def acceptance(self) -> AcceptanceEnvelope:
        return self.compatibility.acceptance

    @property
    def shuttles(self) -> tuple[CanonicalSlotPrediction, ...]:
        return self.compatibility.shuttles


def sha256_file_v4(path: Path | str) -> str:
    candidate = Path(path)
    digest = hashlib.sha256()
    try:
        with candidate.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
    except OSError as exc:
        raise VisualRuntimeV4Error(
            f'cannot read V4 artifact for SHA-256: {candidate}'
        ) from exc
    return digest.hexdigest()


def verify_v4_runtime_promotion(
    manifest_path: Path | str,
    expected_manifest_sha256: str,
) -> VerifiedV4RuntimePromotion:
    """Verify a complete immutable promotion handoff without importing Torch."""

    path = Path(manifest_path).expanduser().resolve()
    expected_manifest_sha = _required_sha256(
        expected_manifest_sha256,
        'expected manifest SHA-256',
    )
    actual_manifest_sha = sha256_file_v4(path)
    if actual_manifest_sha != expected_manifest_sha:
        raise VisualRuntimeV4Error(
            'V4 promotion manifest SHA-256 mismatch: '
            f'{actual_manifest_sha} != {expected_manifest_sha}'
        )
    manifest = _load_json_object(path, 'V4 promotion manifest')
    if manifest.get('schema_version') != PROMOTION_SCHEMA_VERSION:
        raise VisualRuntimeV4Error('V4 promotion manifest schema is unsupported')
    if manifest.get('immutable') is not True:
        raise VisualRuntimeV4Error('V4 promotion manifest must be immutable')
    deployment_mode = str(manifest.get('deployment_mode') or '').strip().lower()
    if deployment_mode not in {'shadow', 'active'}:
        raise VisualRuntimeV4Error(
            'V4 deployment_mode must be shadow or active'
        )
    manual_review_approved = manifest.get('manual_review_approved')
    manual_review_status = str(
        manifest.get('manual_runtime_review_status') or ''
    ).strip().lower()
    if deployment_mode == 'shadow':
        if (
            manifest.get('shadow_execution_authorized') is not True
            or manual_review_approved is not False
            or manual_review_status != 'pending'
        ):
            raise VisualRuntimeV4Error(
                'V4 shadow execution requires explicit shadow authorization '
                'and a pending, unapproved manual runtime review'
            )
    elif (
        manual_review_approved is not True
        or manual_review_status != 'approved'
    ):
        raise VisualRuntimeV4Error(
            'active V4 execution requires an approved manual runtime review'
        )
    if manifest.get('automatic_promotion_allowed') not in (None, False):
        raise VisualRuntimeV4Error('automatic V4 promotion is forbidden')

    authorization_scope, runtime_guards = _verify_runtime_authorization(
        manifest,
        manifest_path=path,
        deployment_mode=deployment_mode,
    )

    artifact_entries = manifest.get('artifacts')
    if not isinstance(artifact_entries, Mapping):
        raise VisualRuntimeV4Error('V4 manifest artifacts must be an object')
    missing_artifacts = sorted(set(REQUIRED_ARTIFACTS) - set(artifact_entries))
    if missing_artifacts:
        raise VisualRuntimeV4Error(
            f'V4 promotion artifacts are incomplete: {missing_artifacts}'
        )
    artifacts: dict[str, ArtifactReferenceV4] = {}
    for name in REQUIRED_ARTIFACTS:
        artifacts[name] = _verify_artifact_reference(
            name,
            artifact_entries[name],
            relative_to=path.parent,
        )

    # Parse evidence only after every referenced byte stream has matched the
    # externally pinned manifest and its per-artifact digest.
    training_report = _load_json_object(
        artifacts['training_final_report'].path,
        'V4 training final report',
    )
    validation_acceptance = _load_json_object(
        artifacts['validation_acceptance'].path,
        'V4 validation acceptance',
    )
    canary_report = _load_json_object(
        artifacts['canary_final_report'].path,
        'V4 Canary final report',
    )
    canary_ledger = _load_json_object(
        artifacts['canary_completion_ledger'].path,
        'V4 Canary completion ledger',
    )
    effective_config = _load_json_object(
        artifacts['effective_config'].path,
        'V4 effective configuration',
    )
    calibration = _load_json_object(
        artifacts['validation_segment_calibration'].path,
        'V4 validation segment calibration',
    )
    topology_json = _load_json_object(
        artifacts['public_topology_contract'].path,
        'V4 public topology contract',
    )

    model_contract = _required_mapping(
        manifest.get('model_contract'),
        'V4 manifest model_contract',
    )
    preprocessing_contract = _required_mapping(
        manifest.get('preprocessing_contract'),
        'V4 manifest preprocessing_contract',
    )
    topology_manifest = _required_mapping(
        manifest.get('topology_contract'),
        'V4 manifest topology_contract',
    )
    calibration_manifest = _required_mapping(
        manifest.get('calibration_contract'),
        'V4 manifest calibration_contract',
    )
    thresholds = _verify_acceptance_thresholds(
        manifest.get('acceptance_thresholds')
    )
    model_values = _verify_model_contract(model_contract)
    _verify_preprocessing_contract(preprocessing_contract)
    effective_fingerprint = hashlib.sha256(
        _canonical_json(effective_config).encode('utf-8')
    ).hexdigest()
    if (
        model_values['effective_config_fingerprint_sha256']
        != effective_fingerprint
    ):
        raise VisualRuntimeV4Error(
            'manifest effective configuration fingerprint does not match its artifact'
        )
    _verify_effective_config(
        effective_config,
        model_contract=model_contract,
        preprocessing_contract=preprocessing_contract,
        effective_fingerprint=effective_fingerprint,
    )
    topology = _verify_topology_contract(
        topology_json,
        topology_manifest=topology_manifest,
        effective_config=effective_config,
    )
    temperature = _verify_calibration_contract(
        calibration,
        calibration_manifest=calibration_manifest,
    )
    _verify_training_report(
        training_report,
        checkpoint=artifacts['checkpoint'],
        effective_fingerprint=effective_fingerprint,
        validation_acceptance=validation_acceptance,
        calibration=calibration,
        topology_json=topology_json,
    )
    _verify_canary_handoff(
        canary_report,
        canary_ledger,
        checkpoint=artifacts['checkpoint'],
        canary_report_sha256=artifacts['canary_final_report'].sha256,
        temperature=temperature,
        topology_json=topology_json,
    )

    return VerifiedV4RuntimePromotion(
        manifest_path=path,
        manifest_sha256=actual_manifest_sha,
        deployment_mode=deployment_mode,
        authorization_scope=authorization_scope,
        runtime_guards=runtime_guards,
        artifacts=MappingProxyType(dict(artifacts)),
        checkpoint_epoch=model_values['checkpoint_epoch'],
        effective_config_fingerprint_sha256=effective_fingerprint,
        head_type=model_values['head_type'],
        hidden_dim=model_values['hidden_dim'],
        attention_heads=model_values['attention_heads'],
        dropout=model_values['dropout'],
        segment_temperature=temperature,
        thresholds=thresholds,
        topology=topology,
    )


def preprocess_rgb_image_v4(rgb_image: np.ndarray) -> np.ndarray:
    """Match V4 validation preprocessing: RGB uint8, PIL bilinear, 320x240."""

    array = np.asarray(rgb_image)
    if array.ndim != 3 or array.shape[2] != 3:
        raise VisualRuntimeV4Error(
            f'V4 RGB image must have shape [H,W,3], got {array.shape}'
        )
    if array.shape[0] <= 0 or array.shape[1] <= 0:
        raise VisualRuntimeV4Error('V4 RGB image dimensions must be positive')
    if int(array.shape[1]) * 3 != int(array.shape[0]) * 4:
        raise VisualRuntimeV4Error(
            'V4 RGB image must retain the training-time 4:3 aspect ratio'
        )
    if array.dtype != np.uint8:
        raise VisualRuntimeV4Error(
            f'V4 runtime RGB input must be uint8, got {array.dtype}'
        )
    resampling = getattr(Image, 'Resampling', Image).BILINEAR
    resized = Image.fromarray(array, mode='RGB').resize(IMAGE_SIZE, resampling)
    result = np.asarray(resized, dtype=np.float32) / np.float32(255.0)
    result = np.transpose(result, (2, 0, 1))
    mean = np.asarray(IMAGENET_MEAN, dtype=np.float32).reshape(3, 1, 1)
    std = np.asarray(IMAGENET_STD, dtype=np.float32).reshape(3, 1, 1)
    normalized = ((result - mean) / std).astype(np.float32)
    if normalized.shape != (3, IMAGE_SIZE[1], IMAGE_SIZE[0]):
        raise VisualRuntimeV4Error(
            f'V4 RGB preprocessing shape mismatch: {normalized.shape}'
        )
    if not np.isfinite(normalized).all():
        raise VisualRuntimeV4Error('V4 preprocessed RGB contains non-finite values')
    return normalized


def preprocess_paired_rgb_v4(
    left_rgb: np.ndarray,
    right_rgb: np.ndarray,
) -> np.ndarray:
    """Return fixed left-then-right ImageNet-normalized channels."""

    result = np.concatenate(
        (preprocess_rgb_image_v4(left_rgb), preprocess_rgb_image_v4(right_rgb)),
        axis=0,
    ).astype(np.float32)
    if result.shape != (6, IMAGE_SIZE[1], IMAGE_SIZE[0]):
        raise VisualRuntimeV4Error(
            f'V4 paired preprocessing shape mismatch: {result.shape}'
        )
    return result


class Room315VisualModelRuntimeV4:
    """Strict V4 checkpoint loader and deterministic paired-image inference."""

    def __init__(
        self,
        promotion: VerifiedV4RuntimePromotion,
        *,
        device: str = 'auto',
    ) -> None:
        if not isinstance(promotion, VerifiedV4RuntimePromotion):
            raise VisualRuntimeV4Error(
                'V4 runtime requires a VerifiedV4RuntimePromotion'
            )
        self.promotion = promotion
        self.requested_device = str(device or 'auto').strip().lower()
        self.device = ''
        self.model: Any | None = None
        self.torch: Any | None = None
        self.torchvision: Any | None = None
        self.ready = False
        self.model_load_duration_ms: float | None = None

    def load(self) -> None:
        started = time.perf_counter()
        try:
            import torch
            import torchvision
        except Exception as exc:
            raise VisualRuntimeV4Error(
                'Torch and TorchVision are required after V4 metadata verification'
            ) from exc
        self.device = _choose_device(torch, self.requested_device)
        model = build_visual_state_model_v4(
            torch,
            torchvision,
            head_type=self.promotion.head_type,
            hidden_dim=self.promotion.hidden_dim,
            attention_heads=self.promotion.attention_heads,
            dropout=self.promotion.dropout,
        )
        checkpoint = _torch_load(
            torch,
            self.promotion.artifact('checkpoint').path,
        )
        _verify_loaded_checkpoint_metadata(checkpoint, self.promotion)
        state = checkpoint.get('model_state_dict')
        if not isinstance(state, Mapping):
            raise VisualRuntimeV4Error('V4 checkpoint lacks model_state_dict')
        try:
            model.load_state_dict(state, strict=True)
        except Exception as exc:
            raise VisualRuntimeV4Error(
                'V4 checkpoint strict model loading failed'
            ) from exc
        if getattr(model, 'model_kind', None) != V4_MODEL_KIND:
            raise VisualRuntimeV4Error('loaded V4 model kind is incompatible')
        if tuple(getattr(model, 'slot_order', ())) != tuple(V4_SLOT_ORDER):
            raise VisualRuntimeV4Error('loaded V4 model slot order is incompatible')
        if getattr(model, 'head_type', None) != self.promotion.head_type:
            raise VisualRuntimeV4Error('loaded V4 head type is incompatible')
        for name, value in model.state_dict().items():
            try:
                finite = bool(torch.isfinite(value.detach()).all().item())
            except Exception as exc:
                raise VisualRuntimeV4Error(
                    f'cannot validate V4 state tensor: {name}'
                ) from exc
            if not finite:
                raise VisualRuntimeV4Error(
                    f'V4 model state contains non-finite values: {name}'
                )
        model.to(self.device)
        model.eval()
        self.torch = torch
        self.torchvision = torchvision
        self.model = model
        self.ready = True
        self.model_load_duration_ms = (time.perf_counter() - started) * 1000.0

    def infer(
        self,
        left_rgb: np.ndarray,
        right_rgb: np.ndarray,
    ) -> tuple[StructuredVisualOutputV4, InferenceTimings]:
        if not self.ready or self.model is None or self.torch is None:
            raise VisualRuntimeV4Error('V4 model runtime is not ready')
        cycle_started = time.perf_counter()
        prep_started = time.perf_counter()
        paired = preprocess_paired_rgb_v4(left_rgb, right_rgb)
        preprocessing_ms = (time.perf_counter() - prep_started) * 1000.0

        infer_started = time.perf_counter()
        tensor = self.torch.from_numpy(paired[None]).to(self.device)
        try:
            with self.torch.inference_mode():
                raw_output = self.model(tensor)
        except Exception as exc:
            raise VisualRuntimeV4Error('V4 model inference failed') from exc
        inference_ms = (time.perf_counter() - infer_started) * 1000.0

        decode_started = time.perf_counter()
        structured = _structured_output_from_model(raw_output)
        decode_ms = (time.perf_counter() - decode_started) * 1000.0
        return structured, InferenceTimings(
            preprocessing_ms=preprocessing_ms,
            inference_ms=inference_ms,
            decode_ms=decode_ms,
            complete_cycle_ms=(time.perf_counter() - cycle_started) * 1000.0,
        )


def build_diagnostic_legacy_output_v4(
    output: StructuredVisualOutputV4,
    *,
    promotion: VerifiedV4RuntimePromotion,
    presence: PresenceSnapshot,
    left_image_size: tuple[int, int],
    right_image_size: tuple[int, int],
) -> DiagnosticLegacyOutputV4:
    """Build a presence-aware diagnostic vector that is never a control input."""

    if not isinstance(promotion, VerifiedV4RuntimePromotion):
        raise VisualRuntimeV4Error('diagnostic adaptation requires verified promotion')
    entries = _validated_presence(presence)
    left_size = _image_size(left_image_size, 'left image size')
    right_size = _image_size(right_image_size, 'right image size')
    structured = _validated_structured_output(output)
    segment_probability = _softmax(
        structured.segment_logits,
        temperature=promotion.segment_temperature,
        context='segment logits',
    )
    loaded_probability = _softmax(
        structured.loaded_logits,
        temperature=1.0,
        context='loaded logits',
    )
    predictions: list[StructuredSlotPrediction] = []
    acceptance: dict[str, SlotAcceptance] = {}
    image_sizes = {'left': left_size, 'right': right_size}
    for index, identity in enumerate(V4_SLOT_ORDER):
        side = derive_side(identity)
        width, height = image_sizes[side]
        segment_index = int(np.argmax(segment_probability[index]))
        loaded_index = int(np.argmax(loaded_probability[index]))
        segment_confidence = float(segment_probability[index, segment_index])
        loaded_confidence = float(loaded_probability[index, loaded_index])
        normalized_bbox = structured.bbox[index]
        predictions.append(StructuredSlotPrediction(
            identity=identity,
            segment_index=segment_index,
            loaded_index=loaded_index,
            bbox_xywh=(
                float(normalized_bbox[0]) * width,
                float(normalized_bbox[1]) * height,
                float(normalized_bbox[2]) * width,
                float(normalized_bbox[3]) * height,
            ),
            s_ratio=float(structured.s_ratio[index]),
        ))
        presence_entry = entries[identity]
        if presence_entry.state == PRESENCE_ABSENT:
            acceptance[identity] = SlotAcceptance(
                identity=identity,
                accepted=False,
                required=False,
                segment_confidence=segment_confidence,
                loaded_confidence=loaded_confidence,
                reasons=('presence_absent_not_evaluated',),
            )
            continue
        reasons = []
        if segment_confidence < promotion.thresholds.minimum_segment_confidence:
            reasons.append('segment_confidence_below_threshold')
        if loaded_confidence < promotion.thresholds.minimum_loaded_confidence:
            reasons.append('loaded_confidence_below_threshold')
        acceptance[identity] = SlotAcceptance(
            identity=identity,
            accepted=not reasons,
            required=True,
            segment_confidence=segment_confidence,
            loaded_confidence=loaded_confidence,
            reasons=tuple(reasons),
        )
    try:
        compatibility = assemble_legacy_200(
            predictions,
            segment_length_contract=promotion.topology,
            image_sizes_by_side=image_sizes,
            acceptance_by_identity=acceptance,
        )
    except VisualContractV4Error as exc:
        raise VisualRuntimeV4Error(
            f'V4 diagnostic compatibility adaptation failed: {exc}'
        ) from exc
    return DiagnosticLegacyOutputV4(compatibility=compatibility)


def decode_active_slots_v4(
    output: StructuredVisualOutputV4,
    *,
    promotion: VerifiedV4RuntimePromotion,
    presence: PresenceSnapshot,
    timestamp_s: float,
    left_image_stamp_s: float,
    right_image_stamp_s: float,
    left_image_size: tuple[int, int],
    right_image_size: tuple[int, int],
) -> DecodedVisualPrediction:
    """Decode active slots or reject the entire active frame fail-closed."""

    diagnostic = build_diagnostic_legacy_output_v4(
        output,
        promotion=promotion,
        presence=presence,
        left_image_size=left_image_size,
        right_image_size=right_image_size,
    )
    if not diagnostic.acceptance.accepted:
        reasons = diagnostic.acceptance.reasons or ('active_frame_rejected',)
        raise VisualRuntimeV4Error(
            'V4 active frame rejected: ' + ','.join(reasons)
        )
    active = set(diagnostic.acceptance.required_identities)
    acceptance_by_identity = {
        slot.identity: slot for slot in diagnostic.acceptance.slots
    }
    predictions = tuple(
        DecodedShuttlePrediction(
            identity=shuttle.identity,
            side=shuttle.side,
            block=shuttle.segment,
            bbox_xywh=shuttle.bbox_xywh,
            s_m=shuttle.s_m,
            s_ratio=shuttle.s_ratio,
            segment_length_m=shuttle.segment_length_m,
            loaded_state=shuttle.loaded_state,
            segment_confidence=float(
                acceptance_by_identity[shuttle.identity].segment_confidence
            ),
            loaded_confidence=float(
                acceptance_by_identity[shuttle.identity].loaded_confidence
            ),
        )
        for shuttle in diagnostic.shuttles
        if shuttle.identity in active
    )
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
        shuttles=predictions,
        active_identities=tuple(
            identity for identity in V4_SLOT_ORDER if identity in active
        ),
        absent_identities=tuple(
            identity for identity in V4_SLOT_ORDER if identity not in active
        ),
    )


def _verify_artifact_reference(
    name: str,
    value: Any,
    *,
    relative_to: Path,
) -> ArtifactReferenceV4:
    entry = _required_mapping(value, f'V4 artifact {name}')
    raw_path = entry.get('path')
    if not isinstance(raw_path, str) or not raw_path.strip():
        raise VisualRuntimeV4Error(f'V4 artifact {name} path is missing')
    candidate = Path(raw_path).expanduser()
    if not candidate.is_absolute():
        candidate = relative_to / candidate
    candidate = candidate.resolve()
    if not candidate.is_file():
        raise VisualRuntimeV4Error(
            f'required V4 artifact is missing: {candidate}'
        )
    expected = _required_sha256(entry.get('sha256'), f'{name} SHA-256')
    actual = sha256_file_v4(candidate)
    if actual != expected:
        raise VisualRuntimeV4Error(
            f'V4 artifact SHA-256 mismatch for {name}: {actual} != {expected}'
        )
    return ArtifactReferenceV4(path=candidate, sha256=actual)


def _verify_runtime_authorization(
    manifest: Mapping[str, Any],
    *,
    manifest_path: Path,
    deployment_mode: str,
) -> tuple[str, Mapping[str, bool]]:
    """Bind runtime side effects to the immutable manual decision.

    The model manifest used to prove only that an active candidate had been
    reviewed.  That is insufficient once the same visual output can feed a
    task gateway: a dry-run approval must never be repurposed for actuation by
    launch-argument overrides.  Shadow mode has a fixed no-side-effect policy;
    active mode additionally verifies the referenced decision bytes and their
    exact Gazebo scope.
    """

    if deployment_mode == 'shadow':
        return (
            SHADOW_AUTHORIZATION_SCOPE,
            MappingProxyType({
                'dry_run_state_fusion': True,
                'plansys2_update_enabled': False,
                'actuation_enabled': False,
            }),
        )

    decision_reference = _required_mapping(
        manifest.get('manual_decision_record'),
        'active V4 manual_decision_record',
    )
    if (
        decision_reference.get('schema_version')
        != MANUAL_DECISION_SCHEMA_VERSION
    ):
        raise VisualRuntimeV4Error(
            'active V4 manual-decision schema is unsupported'
        )
    decision_artifact = _verify_artifact_reference(
        'manual_decision_record',
        decision_reference,
        relative_to=manifest_path.parent,
    )
    decision = _load_json_object(
        decision_artifact.path,
        'active V4 manual decision',
    )
    if decision.get('schema_version') != MANUAL_DECISION_SCHEMA_VERSION:
        raise VisualRuntimeV4Error(
            'active V4 manual-decision payload schema is unsupported'
        )
    if decision.get('decision') != 'approved':
        raise VisualRuntimeV4Error('active V4 manual decision is not approved')
    if decision.get('automatic_promotion') is not False:
        raise VisualRuntimeV4Error(
            'active V4 manual decision must forbid automatic promotion'
        )
    if decision.get('physical_deployment_approved') is not False:
        raise VisualRuntimeV4Error(
            'this V4 runtime authorizes Gazebo only, not physical deployment'
        )
    if not str(decision.get('reviewer') or '').strip():
        raise VisualRuntimeV4Error('active V4 manual reviewer is missing')
    if not str(decision.get('reviewed_at_utc') or '').strip():
        raise VisualRuntimeV4Error('active V4 review timestamp is missing')
    if decision.get('candidate_id') != manifest.get('candidate_id'):
        raise VisualRuntimeV4Error(
            'active V4 decision/manifest candidate ID mismatch'
        )

    artifacts = _required_mapping(
        manifest.get('artifacts'),
        'active V4 manifest artifacts',
    )
    checkpoint = _required_mapping(
        artifacts.get('checkpoint'),
        'active V4 checkpoint artifact',
    )
    if decision.get('checkpoint_sha256') != _required_sha256(
        checkpoint.get('sha256'),
        'active V4 checkpoint SHA-256',
    ):
        raise VisualRuntimeV4Error(
            'active V4 decision/checkpoint SHA-256 mismatch'
        )

    scope = str(decision.get('scope') or '').strip().lower()
    guards_by_scope = {
        DRY_RUN_AUTHORIZATION_SCOPE: {
            'dry_run_state_fusion': True,
            'plansys2_update_enabled': False,
            'actuation_enabled': False,
        },
        CLOSED_LOOP_QUALIFICATION_SCOPE: {
            # The visual node does not own the planner or actuator.  Keep its
            # optional Problem-Expert predicate mirror disabled; the separate
            # task gateway builds each PlanSys2 problem from accepted V4 state.
            'dry_run_state_fusion': True,
            'plansys2_update_enabled': False,
            'actuation_enabled': True,
        },
        CLOSED_LOOP_RUNTIME_SCOPE: {
            'dry_run_state_fusion': True,
            'plansys2_update_enabled': False,
            'actuation_enabled': True,
        },
    }
    expected_guards = guards_by_scope.get(scope)
    if expected_guards is None:
        raise VisualRuntimeV4Error(
            f'active V4 authorization scope is unsupported: {scope!r}'
        )
    guards = _required_mapping(
        decision.get('runtime_guards'),
        'active V4 runtime guards',
    )
    if dict(guards) != expected_guards:
        raise VisualRuntimeV4Error(
            'active V4 runtime guards do not match the approved scope'
        )
    return scope, MappingProxyType(dict(expected_guards))


def _verify_model_contract(contract: Mapping[str, Any]) -> dict[str, Any]:
    if contract.get('checkpoint_schema_version') != CHECKPOINT_SCHEMA_VERSION:
        raise VisualRuntimeV4Error('V4 checkpoint schema contract is incompatible')
    if contract.get('model_kind') != V4_MODEL_KIND:
        raise VisualRuntimeV4Error('V4 model kind contract is incompatible')
    if tuple(contract.get('slot_order') or ()) != tuple(V4_SLOT_ORDER):
        raise VisualRuntimeV4Error('V4 model slot order contract is incompatible')
    if tuple(contract.get('segment_order') or ()) != tuple(SEGMENT_CLASSES):
        raise VisualRuntimeV4Error('V4 segment order contract is incompatible')
    if tuple(contract.get('output_keys') or ()) != tuple(V4_OUTPUT_KEYS):
        raise VisualRuntimeV4Error('V4 output-key contract is incompatible')
    if contract.get('checkpoint_loading') != 'strict':
        raise VisualRuntimeV4Error('V4 checkpoint loading must remain strict')
    epoch = _integer(contract.get('checkpoint_epoch'), 'V4 checkpoint epoch')
    if epoch != CHECKPOINT_EPOCH:
        raise VisualRuntimeV4Error(
            f'V4 runtime is pinned to checkpoint epoch {CHECKPOINT_EPOCH}'
        )
    head_type = str(contract.get('head_type') or '').strip().lower()
    if head_type != 'spatial_query':
        raise VisualRuntimeV4Error('V4 runtime requires the spatial_query head')
    hidden_dim = _positive_integer(contract.get('hidden_dim'), 'V4 hidden_dim')
    attention_heads = _positive_integer(
        contract.get('attention_heads'),
        'V4 attention_heads',
    )
    if hidden_dim % 4 or hidden_dim % attention_heads:
        raise VisualRuntimeV4Error('V4 hidden/head dimensions are incompatible')
    dropout = _finite_float(contract.get('dropout'), 'V4 dropout')
    if not 0.0 <= dropout < 1.0:
        raise VisualRuntimeV4Error('V4 dropout must be in [0,1)')
    fingerprint = _required_sha256(
        contract.get('effective_config_fingerprint_sha256'),
        'V4 effective configuration fingerprint',
    )
    return {
        'checkpoint_epoch': epoch,
        'head_type': head_type,
        'hidden_dim': hidden_dim,
        'attention_heads': attention_heads,
        'dropout': dropout,
        'effective_config_fingerprint_sha256': fingerprint,
    }


def _verify_preprocessing_contract(contract: Mapping[str, Any]) -> None:
    checks = {
        'camera_order': tuple(contract.get('camera_order') or ()) == CAMERA_ORDER,
        'channel_order': tuple(contract.get('channel_order') or ()) == CHANNEL_ORDER,
        'input_shape': tuple(contract.get('input_shape') or ()) == INPUT_SHAPE,
        'width': _integer(contract.get('width'), 'V4 preprocessing width') == 320,
        'height': _integer(contract.get('height'), 'V4 preprocessing height') == 240,
        'resize': contract.get('resize') == 'aspect_preserving_bilinear_4_by_3',
        'resampling': contract.get('resampling') == 'bilinear',
        'mean': tuple(contract.get('normalization_mean') or ()) == IMAGENET_MEAN,
        'std': tuple(contract.get('normalization_std') or ()) == IMAGENET_STD,
        'augmentations': contract.get('runtime_augmentations') is False,
    }
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise VisualRuntimeV4Error(
            f'V4 preprocessing contract is incompatible: {failed}'
        )


def _verify_effective_config(
    config: Mapping[str, Any],
    *,
    model_contract: Mapping[str, Any],
    preprocessing_contract: Mapping[str, Any],
    effective_fingerprint: str,
) -> None:
    model = _required_mapping(config.get('model'), 'effective V4 model config')
    expected_model_keys = (
        'kind', 'head_type', 'hidden_dim', 'attention_heads', 'dropout',
    )
    manifest_key = {
        'kind': 'model_kind',
        'head_type': 'head_type',
        'hidden_dim': 'hidden_dim',
        'attention_heads': 'attention_heads',
        'dropout': 'dropout',
    }
    if any(
        model.get(key) != model_contract.get(manifest_key[key])
        for key in expected_model_keys
    ):
        raise VisualRuntimeV4Error(
            'effective V4 model configuration differs from promotion contract'
        )
    if tuple(model.get('slot_order') or ()) != tuple(V4_SLOT_ORDER):
        raise VisualRuntimeV4Error('effective V4 fixed slot order is incompatible')
    if model.get('cross_camera_feature_path') is not False:
        raise VisualRuntimeV4Error('V4 cross-camera feature path must remain disabled')

    preprocessing = _required_mapping(
        config.get('image_preprocessing'),
        'effective V4 image preprocessing',
    )
    for key in ('width', 'height', 'resize', 'normalization_mean', 'normalization_std'):
        if preprocessing.get(key) != preprocessing_contract.get(key):
            raise VisualRuntimeV4Error(
                f'effective V4 preprocessing differs for {key}'
            )
    augmentations = _required_mapping(
        preprocessing.get('train_augmentations'),
        'effective V4 train augmentations',
    )
    if (
        augmentations.get('horizontal_flip') is not False
        or augmentations.get('camera_swap') is not False
    ):
        raise VisualRuntimeV4Error(
            'effective V4 configuration permits forbidden spatial augmentation'
        )
    roles = _required_mapping(config.get('data_roles'), 'effective V4 data roles')
    if roles.get('checkpoint_selection') != 'validation_only':
        raise VisualRuntimeV4Error('V4 checkpoint selection was not validation-only')
    data = _required_mapping(config.get('data'), 'effective V4 data config')
    if 'test' in data:
        raise VisualRuntimeV4Error('V4 effective configuration contains forbidden Test data')
    computed = hashlib.sha256(_canonical_json(config).encode('utf-8')).hexdigest()
    if computed != effective_fingerprint:
        raise VisualRuntimeV4Error('V4 effective configuration changed during verification')


def _verify_topology_contract(
    topology_json: Mapping[str, Any],
    *,
    topology_manifest: Mapping[str, Any],
    effective_config: Mapping[str, Any],
) -> PublicSegmentLengthContract:
    expected_manifest = {
        'schema_version': PUBLIC_TOPOLOGY_SCHEMA_VERSION,
        'source': 'room_315_rail_defaults.public_rail_segment_lengths',
        'segment_name_domain': 'public_ros_label',
        'side_order': list(SIDES),
        'segment_order': list(SEGMENT_CLASSES),
    }
    for key, expected in expected_manifest.items():
        observed = topology_manifest.get(key)
        if isinstance(expected, list):
            observed = list(observed or ())
        if observed != expected:
            raise VisualRuntimeV4Error(
                f'V4 manifest topology contract differs for {key}'
            )
    manifest_fingerprint = _required_sha256(
        topology_manifest.get('fingerprint_sha256'),
        'V4 topology fingerprint',
    )
    if topology_json.get('schema_version') != PUBLIC_TOPOLOGY_SCHEMA_VERSION:
        raise VisualRuntimeV4Error('V4 topology artifact schema is incompatible')
    if topology_json.get('authoritative') is not True:
        raise VisualRuntimeV4Error('V4 topology artifact is not authoritative')
    if topology_json.get('source') != expected_manifest['source']:
        raise VisualRuntimeV4Error('V4 topology source is incompatible')
    if topology_json.get('segment_name_domain') != 'public_ros_label':
        raise VisualRuntimeV4Error('V4 topology names are not public ROS labels')
    if tuple(topology_json.get('side_order') or ()) != SIDES:
        raise VisualRuntimeV4Error('V4 topology side order is incompatible')
    if tuple(topology_json.get('segment_order') or ()) != SEGMENT_CLASSES:
        raise VisualRuntimeV4Error('V4 topology segment order is incompatible')
    if topology_json.get('fingerprint_sha256') != manifest_fingerprint:
        raise VisualRuntimeV4Error('V4 topology artifact fingerprint mismatch')
    effective_topology = _required_mapping(
        effective_config.get('topology_contract'),
        'effective V4 topology contract',
    )
    if (
        effective_topology.get('segment_name_domain') != 'public_ros_label'
        or effective_topology.get('length_loader')
        != 'room_315_rail_defaults.public_rail_segment_lengths'
        or effective_topology.get('forbid_internal_rail_segment_lengths') is not True
        or effective_topology.get('verify_length_mapping_fingerprint_at_runtime')
        is not True
    ):
        raise VisualRuntimeV4Error('effective V4 topology policy is incompatible')
    try:
        current = load_authoritative_public_segment_length_contract()
    except VisualContractV4Error as exc:
        raise VisualRuntimeV4Error(
            f'cannot load authoritative public V4 topology: {exc}'
        ) from exc
    if current.fingerprint_sha256 != manifest_fingerprint:
        raise VisualRuntimeV4Error(
            'authoritative public topology changed since V4 promotion'
        )
    expected_matrix = tuple(
        tuple(float(item) for item in row)
        for row in topology_json.get('lengths_m_by_side') or ()
    )
    if expected_matrix != current.as_matrix():
        raise VisualRuntimeV4Error('V4 topology length matrix is incompatible')
    expected_by_side = topology_json.get('lengths_by_side')
    if not isinstance(expected_by_side, Mapping):
        raise VisualRuntimeV4Error('V4 topology side mappings are unavailable')
    for side in SIDES:
        row = expected_by_side.get(side)
        if not isinstance(row, Mapping) or {
            segment: float(row.get(segment, math.nan))
            for segment in SEGMENT_CLASSES
        } != dict(current.as_mapping()[side]):
            raise VisualRuntimeV4Error(
                f'V4 public topology mapping differs for {side}'
            )
    return current


def _verify_calibration_contract(
    calibration: Mapping[str, Any],
    *,
    calibration_manifest: Mapping[str, Any],
) -> float:
    if calibration.get('schema_version') != SEGMENT_CALIBRATION_SCHEMA_VERSION:
        raise VisualRuntimeV4Error('V4 validation calibration schema is incompatible')
    if calibration.get('fit_scope') != 'validation_only':
        raise VisualRuntimeV4Error('V4 segment calibration was not validation-only')
    if calibration.get('data_role') != 'validation':
        raise VisualRuntimeV4Error('V4 segment calibration data role is incompatible')
    temperature = _finite_float(
        calibration.get('temperature'),
        'V4 validation segment temperature',
    )
    if temperature <= 0.0:
        raise VisualRuntimeV4Error('V4 segment temperature must be positive')
    expected = {
        'schema_version': SEGMENT_CALIBRATION_SCHEMA_VERSION,
        'fit_scope': 'validation_only',
        'data_role': 'validation',
    }
    for key, value in expected.items():
        if calibration_manifest.get(key) != value:
            raise VisualRuntimeV4Error(
                f'V4 manifest calibration differs for {key}'
            )
    manifest_temperature = _finite_float(
        calibration_manifest.get('temperature'),
        'manifest V4 segment temperature',
    )
    if manifest_temperature != temperature:
        raise VisualRuntimeV4Error('V4 manifest segment temperature mismatch')
    return temperature


def _verify_training_report(
    report: Mapping[str, Any],
    *,
    checkpoint: ArtifactReferenceV4,
    effective_fingerprint: str,
    validation_acceptance: Mapping[str, Any],
    calibration: Mapping[str, Any],
    topology_json: Mapping[str, Any],
) -> None:
    if report.get('schema_version') != TRAINING_RUN_SCHEMA_VERSION:
        raise VisualRuntimeV4Error('V4 training report schema is incompatible')
    acceptance = _required_mapping(
        report.get('validation_acceptance'),
        'V4 validation acceptance',
    )
    checks = {
        'training_completed': report.get('status') == 'completed',
        'validation_status': report.get('validation_acceptance_status') == 'passed',
        'validation_artifact_schema': (
            validation_acceptance.get('schema_version')
            == VALIDATION_ACCEPTANCE_SCHEMA_VERSION
        ),
        'validation_artifact_binding': acceptance == validation_acceptance,
        'validation_accepted': acceptance.get('accepted') is True,
        'validation_acceptance_status': acceptance.get('status') == 'passed',
        'validation_summary': _all_gates_passed(acceptance.get('summary')),
        'selection_role': (
            isinstance(report.get('checkpoint_selection'), Mapping)
            and report['checkpoint_selection'].get('role') == 'validation_only'
        ),
        'canary_not_used_for_selection': report.get('canary_used_for_selection') is False,
        'canary_not_loaded_during_training': report.get('canary_loaded') is False,
        'test_not_loaded': report.get('test_loaded') is False,
        'runtime_not_automatically_switched': report.get('automatic_runtime_switch') is False,
        'effective_config': report.get('effective_config_sha256') == effective_fingerprint,
        'calibration': report.get('validation_segment_calibration') == calibration,
        'topology': report.get('public_topology_contract') == topology_json,
        'topology_fingerprint': (
            report.get('topology_length_mapping_fingerprint_sha256')
            == topology_json.get('fingerprint_sha256')
        ),
    }
    selected = report.get('selected_checkpoint')
    checks['selected_checkpoint'] = (
        isinstance(selected, Mapping)
        and selected.get('sha256') == checkpoint.sha256
        and _integer_or_none(selected.get('epoch')) == CHECKPOINT_EPOCH
        and selected.get('filename') == 'epoch_011.pt'
    )
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise VisualRuntimeV4Error(
            f'V4 training report cannot authorize runtime: {failed}'
        )


def _verify_canary_handoff(
    report: Mapping[str, Any],
    ledger: Mapping[str, Any],
    *,
    checkpoint: ArtifactReferenceV4,
    canary_report_sha256: str,
    temperature: float,
    topology_json: Mapping[str, Any],
) -> None:
    attempt = report.get('canary_attempt')
    final_artifact = None
    artifacts = ledger.get('artifacts')
    if isinstance(artifacts, Mapping):
        final_artifact = artifacts.get('final_report.json')
    canary_checkpoint = report.get('checkpoint')
    calibration = report.get('segment_calibration')
    checks = {
        'report_schema': report.get('schema_version') == TRAINING_RUN_SCHEMA_VERSION,
        'report_completed': report.get('status') == 'completed',
        'acceptance_passed': report.get('acceptance_status') == 'passed',
        'acceptance_accepted': (
            isinstance(report.get('acceptance'), Mapping)
            and report['acceptance'].get('accepted') is True
            and _all_gates_passed(report['acceptance'].get('summary'))
        ),
        'manual_promotion_only': (
            report.get('promotion_status') == 'eligible_for_manual_runtime_review'
        ),
        'prior_exposure_acknowledged': report.get('prior_exposure_acknowledged') is True,
        'canary_loaded': report.get('canary_loaded') is True,
        'selection_not_performed': (
            report.get('checkpoint_selection_performed') is False
            and report.get('canary_used_for_checkpoint_selection') is False
        ),
        'test_not_loaded': report.get('test_loaded') is False,
        'runtime_not_switched': report.get('automatic_runtime_switch') is False,
        'calibration_not_refit': (
            report.get('calibration_refit_performed') is False
            and report.get('calibration_temperature_source') == 'validation_only'
        ),
        'calibration_temperature': (
            isinstance(calibration, Mapping)
            and calibration.get('source_temperature_role') == 'validation'
            and _finite_float_or_none(calibration.get('temperature')) == temperature
        ),
        'checkpoint': (
            isinstance(canary_checkpoint, Mapping)
            and canary_checkpoint.get('sha256') == checkpoint.sha256
            and _integer_or_none(canary_checkpoint.get('source_selected_epoch'))
            == CHECKPOINT_EPOCH
            and canary_checkpoint.get('validation_selected') is True
        ),
        'topology': report.get('public_topology_contract') == topology_json,
        'topology_fingerprint': (
            report.get('topology_length_mapping_fingerprint_sha256')
            == topology_json.get('fingerprint_sha256')
        ),
        'attempt_present': isinstance(attempt, Mapping),
        'attempt_one_shot': isinstance(attempt, Mapping) and attempt.get('one_shot') is True,
        'completion_required': (
            isinstance(attempt, Mapping)
            and attempt.get('completion_ledger_required_for_trust') is True
        ),
        'ledger_schema': ledger.get('schema_version') == CANARY_ATTEMPT_SCHEMA_VERSION,
        'ledger_state': ledger.get('state') == 'completed_immutable',
        'attempt_key': (
            isinstance(attempt, Mapping)
            and ledger.get('attempt_key') == attempt.get('attempt_key')
        ),
        'ledger_report_sha': (
            isinstance(final_artifact, Mapping)
            and final_artifact.get('sha256') == canary_report_sha256
        ),
        'ledger_test_not_loaded': ledger.get('test_loaded') is False,
        'ledger_selection_not_performed': (
            ledger.get('canary_used_for_checkpoint_selection') is False
        ),
        'ledger_runtime_not_switched': ledger.get('automatic_runtime_switch') is False,
        'reservation_binding': (
            isinstance(attempt, Mapping)
            and isinstance(ledger.get('reservation'), Mapping)
            and ledger['reservation'].get('sha256') == attempt.get('reservation_sha256')
        ),
    }
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise VisualRuntimeV4Error(
            f'V4 Canary handoff is not trustworthy: {failed}'
        )


def _verify_acceptance_thresholds(value: Any) -> V4AcceptanceThresholds:
    thresholds = _required_mapping(value, 'V4 acceptance_thresholds')
    segment = _finite_float(
        thresholds.get('minimum_segment_confidence'),
        'minimum V4 segment confidence',
    )
    loaded = _finite_float(
        thresholds.get('minimum_loaded_confidence'),
        'minimum V4 loaded confidence',
    )
    if not 0.0 < segment <= 1.0 or not 0.0 < loaded <= 1.0:
        raise VisualRuntimeV4Error('V4 confidence thresholds must be in (0,1]')
    policy = str(thresholds.get('active_frame_policy') or '').strip()
    if policy != 'all_present_slots_must_pass':
        raise VisualRuntimeV4Error('V4 active-frame policy is unsupported')
    return V4AcceptanceThresholds(
        minimum_segment_confidence=segment,
        minimum_loaded_confidence=loaded,
        active_frame_policy=policy,
    )


def _verify_loaded_checkpoint_metadata(
    checkpoint: Mapping[str, Any],
    promotion: VerifiedV4RuntimePromotion,
) -> None:
    if not isinstance(checkpoint, Mapping):
        raise VisualRuntimeV4Error('V4 checkpoint root must be an object')
    checks = {
        'schema': checkpoint.get('schema_version') == CHECKPOINT_SCHEMA_VERSION,
        'kind': checkpoint.get('model_kind') == V4_MODEL_KIND,
        'slot_order': tuple(checkpoint.get('slot_order') or ()) == tuple(V4_SLOT_ORDER),
        'segment_order': (
            tuple(checkpoint.get('segment_order') or ()) == tuple(SEGMENT_CLASSES)
        ),
        'epoch': _integer_or_none(checkpoint.get('epoch')) == promotion.checkpoint_epoch,
        'selection_role': checkpoint.get('checkpoint_selection_role') == 'validation_only',
        'canary_unseen': checkpoint.get('canary_seen') is False,
        'test_unseen': checkpoint.get('test_seen') is False,
        'config_fingerprint': (
            checkpoint.get('effective_config_sha256')
            == promotion.effective_config_fingerprint_sha256
        ),
        'topology_fingerprint': (
            checkpoint.get('topology_length_mapping_fingerprint_sha256')
            == promotion.topology.fingerprint_sha256
        ),
        'topology_contract': (
            isinstance(checkpoint.get('public_topology_contract'), Mapping)
            and checkpoint['public_topology_contract'].get('fingerprint_sha256')
            == promotion.topology.fingerprint_sha256
            and _nested_float_matrix(
                checkpoint['public_topology_contract'].get('lengths_m_by_side')
            ) == promotion.topology.as_matrix()
        ),
        'topology_lengths': _topology_lengths_match(
            checkpoint.get('topology_lengths_by_side'),
            promotion.topology,
        ),
    }
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise VisualRuntimeV4Error(
            f'V4 checkpoint metadata is incompatible: {failed}'
        )


def _structured_output_from_model(raw_output: Any) -> StructuredVisualOutputV4:
    if not isinstance(raw_output, Mapping):
        raise VisualRuntimeV4Error('V4 model output must be an object')
    if set(raw_output) != set(V4_OUTPUT_KEYS):
        raise VisualRuntimeV4Error(
            f'V4 model output keys are incompatible: {sorted(raw_output)}'
        )
    arrays: dict[str, np.ndarray] = {}
    expected_shapes = {
        'segment_logits': (1, 8, 14),
        'loaded_logits': (1, 8, 2),
        'bbox': (1, 8, 4),
        's_ratio': (1, 8, 1),
    }
    for name, shape in expected_shapes.items():
        value = raw_output[name]
        try:
            array = value.detach().to('cpu').numpy()
        except Exception as exc:
            raise VisualRuntimeV4Error(
                f'V4 model output {name} is not a tensor'
            ) from exc
        if tuple(array.shape) != shape:
            raise VisualRuntimeV4Error(
                f'V4 model output {name} shape is {array.shape}, expected {shape}'
            )
        arrays[name] = np.asarray(array[0], dtype=np.float32).copy()
    return _validated_structured_output(StructuredVisualOutputV4(
        segment_logits=arrays['segment_logits'],
        loaded_logits=arrays['loaded_logits'],
        bbox=arrays['bbox'],
        s_ratio=arrays['s_ratio'][:, 0],
    ))


def _validated_structured_output(
    output: StructuredVisualOutputV4,
) -> StructuredVisualOutputV4:
    if not isinstance(output, StructuredVisualOutputV4):
        raise VisualRuntimeV4Error('V4 structured output has an invalid type')
    expected = {
        'segment_logits': ((8, 14), output.segment_logits),
        'loaded_logits': ((8, 2), output.loaded_logits),
        'bbox': ((8, 4), output.bbox),
        's_ratio': ((8,), output.s_ratio),
    }
    arrays = {}
    for name, (shape, value) in expected.items():
        try:
            array = np.asarray(value, dtype=np.float32)
        except (TypeError, ValueError) as exc:
            raise VisualRuntimeV4Error(f'V4 {name} must be numeric') from exc
        if array.shape != shape:
            raise VisualRuntimeV4Error(
                f'V4 {name} shape is {array.shape}, expected {shape}'
            )
        if not np.isfinite(array).all():
            raise VisualRuntimeV4Error(f'V4 {name} contains non-finite values')
        arrays[name] = array
    bbox = arrays['bbox']
    tolerance = np.float32(1.0e-5)
    if (
        np.any(bbox[:, :2] < 0.0)
        or np.any(bbox[:, 2:] <= 0.0)
        or np.any(bbox[:, :2] + bbox[:, 2:] > 1.0 + tolerance)
    ):
        raise VisualRuntimeV4Error('V4 normalized bounding boxes are invalid')
    ratio = arrays['s_ratio']
    if np.any(ratio < 0.0) or np.any(ratio > 1.0):
        raise VisualRuntimeV4Error('V4 s_ratio values must be in [0,1]')
    return StructuredVisualOutputV4(
        segment_logits=arrays['segment_logits'],
        loaded_logits=arrays['loaded_logits'],
        bbox=bbox,
        s_ratio=ratio,
    )


def _validated_presence(presence: PresenceSnapshot) -> dict[str, Any]:
    if not isinstance(presence, PresenceSnapshot):
        raise VisualRuntimeV4Error('V4 presence snapshot has an invalid type')
    if not presence.ready:
        raise VisualRuntimeV4Error(
            'V4 presence registry is not ready: ' + ','.join(presence.reasons)
        )
    entries = presence.by_identity()
    if (
        len(presence.entries) != len(V4_SLOT_ORDER)
        or tuple(entries) != tuple(V4_SLOT_ORDER)
    ):
        raise VisualRuntimeV4Error('V4 presence identity order is invalid')
    for identity, entry in entries.items():
        expected_side = derive_side(identity)
        if entry.side != expected_side:
            raise VisualRuntimeV4Error(
                f'V4 presence side conflict: {identity}:{entry.side}!={expected_side}'
            )
        if entry.state == PRESENCE_UNKNOWN:
            raise VisualRuntimeV4Error(f'V4 presence is unknown for {identity}')
        if entry.state not in {PRESENCE_PRESENT, PRESENCE_ABSENT}:
            raise VisualRuntimeV4Error(
                f'unsupported V4 presence state for {identity}: {entry.state}'
            )
    return entries


def _softmax(
    logits: np.ndarray,
    *,
    temperature: float,
    context: str,
) -> np.ndarray:
    if not math.isfinite(temperature) or temperature <= 0.0:
        raise VisualRuntimeV4Error(f'{context} temperature must be positive')
    scaled = np.asarray(logits, dtype=np.float64) / temperature
    scaled -= np.max(scaled, axis=-1, keepdims=True)
    exponent = np.exp(scaled)
    denominator = exponent.sum(axis=-1, keepdims=True)
    probability = exponent / denominator
    if not np.isfinite(probability).all() or np.any(denominator <= 0.0):
        raise VisualRuntimeV4Error(f'{context} softmax is invalid')
    return probability


def _topology_lengths_match(
    value: Any,
    topology: PublicSegmentLengthContract,
) -> bool:
    if not isinstance(value, Mapping):
        return False
    expected = topology.as_mapping()
    for side in SIDES:
        row = value.get(side)
        if not isinstance(row, Mapping):
            return False
        try:
            observed = {segment: float(row[segment]) for segment in SEGMENT_CLASSES}
        except (KeyError, TypeError, ValueError):
            return False
        if set(row) != set(SEGMENT_CLASSES) or observed != dict(expected[side]):
            return False
    return set(value) == set(SIDES)


def _nested_float_matrix(value: Any) -> tuple[tuple[float, ...], ...] | None:
    try:
        return tuple(tuple(float(item) for item in row) for row in value)
    except (TypeError, ValueError):
        return None


def _all_gates_passed(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    gate_count = _integer_or_none(value.get('gate_count'))
    passed = _integer_or_none(value.get('passed'))
    failed = _integer_or_none(value.get('failed'))
    pending = _integer_or_none(value.get('pending'))
    return (
        gate_count is not None
        and gate_count > 0
        and passed == gate_count
        and failed == 0
        and pending == 0
    )


def _load_json_object(path: Path, context: str) -> dict[str, Any]:
    try:
        parsed = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise VisualRuntimeV4Error(f'cannot read {context}: {path}') from exc
    if not isinstance(parsed, dict):
        raise VisualRuntimeV4Error(f'{context} must contain a JSON object')
    return parsed


def _required_mapping(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise VisualRuntimeV4Error(f'{context} must be an object')
    return value


def _required_sha256(value: Any, context: str) -> str:
    digest = str(value or '').strip().lower()
    if not SHA256_PATTERN.fullmatch(digest):
        raise VisualRuntimeV4Error(f'{context} must be a lowercase SHA-256')
    return digest


def _canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    )


def _integer(value: Any, context: str) -> int:
    if isinstance(value, bool):
        raise VisualRuntimeV4Error(f'{context} must be an integer')
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise VisualRuntimeV4Error(f'{context} must be an integer') from exc
    if isinstance(value, float) and not value.is_integer():
        raise VisualRuntimeV4Error(f'{context} must be an integer')
    return parsed


def _positive_integer(value: Any, context: str) -> int:
    parsed = _integer(value, context)
    if parsed <= 0:
        raise VisualRuntimeV4Error(f'{context} must be positive')
    return parsed


def _integer_or_none(value: Any) -> int | None:
    try:
        return _integer(value, 'integer')
    except VisualRuntimeV4Error:
        return None


def _finite_float(value: Any, context: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise VisualRuntimeV4Error(f'{context} must be numeric') from exc
    if not math.isfinite(parsed):
        raise VisualRuntimeV4Error(f'{context} must be finite')
    return parsed


def _finite_float_or_none(value: Any) -> float | None:
    try:
        return _finite_float(value, 'numeric value')
    except VisualRuntimeV4Error:
        return None


def _finite_nonnegative(value: Any, context: str) -> float:
    parsed = _finite_float(value, context)
    if parsed < 0.0:
        raise VisualRuntimeV4Error(f'{context} must be non-negative')
    return parsed


def _image_size(value: Sequence[int], context: str) -> tuple[int, int]:
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise VisualRuntimeV4Error(f'{context} must contain width and height')
    width = _positive_integer(value[0], f'{context} width')
    height = _positive_integer(value[1], f'{context} height')
    if width * 3 != height * 4:
        raise VisualRuntimeV4Error(f'{context} must retain a 4:3 aspect ratio')
    return width, height


def _choose_device(torch_module: Any, requested: str) -> str:
    if requested not in {'auto', 'cpu', 'cuda'}:
        raise VisualRuntimeV4Error('V4 device must be auto, cpu, or cuda')
    if requested == 'cuda':
        if not torch_module.cuda.is_available():
            raise VisualRuntimeV4Error('CUDA was requested for V4 but is unavailable')
        return 'cuda'
    if requested == 'auto':
        return 'cuda' if torch_module.cuda.is_available() else 'cpu'
    return 'cpu'


def _torch_load(torch_module: Any, path: Path) -> Mapping[str, Any]:
    try:
        loaded = torch_module.load(path, map_location='cpu', weights_only=False)
    except TypeError:
        loaded = torch_module.load(path, map_location='cpu')
    except Exception as exc:
        raise VisualRuntimeV4Error(f'cannot load V4 checkpoint: {path}') from exc
    if not isinstance(loaded, Mapping):
        raise VisualRuntimeV4Error('V4 checkpoint root must be an object')
    return loaded


__all__ = [
    'ArtifactReferenceV4',
    'DiagnosticLegacyOutputV4',
    'PROMOTION_SCHEMA_VERSION',
    'Room315VisualModelRuntimeV4',
    'StructuredVisualOutputV4',
    'V4AcceptanceThresholds',
    'VerifiedV4RuntimePromotion',
    'VisualRuntimeV4Error',
    'build_diagnostic_legacy_output_v4',
    'decode_active_slots_v4',
    'preprocess_paired_rgb_v4',
    'preprocess_rgb_image_v4',
    'sha256_file_v4',
    'verify_v4_runtime_promotion',
]
