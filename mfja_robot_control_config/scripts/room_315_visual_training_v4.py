#!/usr/bin/env python3
"""Losses and metrics for the Room 315 fixed-eight visual model V4.

This module deliberately contains no model or dataset code.  Its public API
operates on the following tensor dictionaries::

    predictions = {
        "segment_logits": Tensor[B, 8, 14],
        "loaded_logits": Tensor[B, 8, 2],
        "bbox": Tensor[B, 8, 4],       # normalized x, y, width, height
        "s_ratio": Tensor[B, 8, 1],
    }
    targets = {
        "segment": LongTensor[B, 8],
        "loaded": LongTensor[B, 8],
        "bbox": Tensor[B, 8, 4],
        "s_ratio": Tensor[B, 8, 1],
        "visibility_mask": BoolTensor[B, 8],
    }

Only visible slots contribute to losses or metrics.  Slots 0--3 are the left
rail (L1--L4), and slots 4--7 are the right rail (R1--R4).
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

import torch
import torch.nn.functional as F


SHUTTLE_COUNT = 8
SEGMENT_CLASS_COUNT = 14
LOADED_CLASS_COUNT = 2
SLOT_IDENTITIES = ("L1", "L2", "L3", "L4", "R1", "R2", "R3", "R4")
DEFAULT_BBOX_IMAGE_SIZE = None
DEFAULT_SEGMENT_LABEL_SMOOTHING = 0.02
DEFAULT_LOADED_LABEL_SMOOTHING = 0.01
DEFAULT_S_RATIO_BETA = 0.05
DEFAULT_BBOX_L1_WEIGHT = 1.0
DEFAULT_BBOX_GIOU_WEIGHT = 1.0
DEFAULT_LOSS_WEIGHTS = {
    "segment": 4.0,
    "s_ratio": 2.0,
    "loaded": 1.0,
    "bbox": 1.0,
    "topology": 0.25,
}


def inverse_sqrt_class_weights(
    class_counts: Sequence[float] | torch.Tensor,
    *,
    cap: float = 4.0,
    normalize_mean: bool = True,
    dtype: torch.dtype = torch.float32,
    device: torch.device | str | None = None,
) -> torch.Tensor:
    """Return capped inverse-square-root weights for class counts.

    The most frequent observed class starts at weight 1.  Rarer classes grow
    as ``sqrt(max_count / count)`` and are capped before optional mean-one
    normalization.  A zero-count class is treated as count one and therefore
    receives the capped rare-class weight instead of infinity.
    """

    if not math.isfinite(float(cap)) or float(cap) < 1.0:
        raise ValueError("class-weight cap must be finite and at least 1.0")
    counts = torch.as_tensor(class_counts, dtype=dtype, device=device)
    if counts.ndim not in {1, 2} or counts.numel() == 0:
        raise ValueError("class_counts must have shape [C], [2, C], or [8, C]")
    if counts.ndim == 2 and counts.shape[0] not in {2, SHUTTLE_COUNT}:
        raise ValueError("two-dimensional class_counts must have 2 or 8 rows")
    if not bool(torch.isfinite(counts).all()) or bool((counts < 0).any()):
        raise ValueError("class_counts must contain finite non-negative values")

    safe_counts = counts.clamp_min(1.0)
    reference_count = counts.max(dim=-1, keepdim=True).values.clamp_min(1.0)
    weights = torch.sqrt(reference_count / safe_counts).clamp(max=float(cap))
    if normalize_mean:
        weights = weights / weights.mean(dim=-1, keepdim=True).clamp_min(
            torch.finfo(weights.dtype).eps
        )
    return weights


def _validated_unit_interval(name: str, value: float) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or not 0.0 <= parsed < 1.0:
        raise ValueError(f"{name} must be finite and in [0, 1)")
    return parsed


def _validated_positive(name: str, value: float) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ValueError(f"{name} must be finite and greater than zero")
    return parsed


def _validated_nonnegative(name: str, value: float) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return parsed


def _loss_weights(overrides: Mapping[str, float] | None) -> dict[str, float]:
    result = dict(DEFAULT_LOSS_WEIGHTS)
    if overrides is not None:
        unknown = sorted(set(overrides) - set(result))
        if unknown:
            raise ValueError(f"unknown V4 loss weights: {unknown}")
        result.update({str(key): float(value) for key, value in overrides.items()})
    for name, value in result.items():
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"loss weight {name!r} must be finite and non-negative")
    return result


def _required_tensor(mapping: Mapping[str, Any], name: str) -> torch.Tensor:
    value = mapping.get(name)
    if not isinstance(value, torch.Tensor):
        raise TypeError(f"{name!r} must be a torch.Tensor")
    return value


def _validated_contract(
    predictions: Mapping[str, Any],
    targets: Mapping[str, Any],
) -> tuple[
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
]:
    segment_logits = _required_tensor(predictions, "segment_logits")
    loaded_logits = _required_tensor(predictions, "loaded_logits")
    pred_bbox = _required_tensor(predictions, "bbox")
    pred_s_ratio = _required_tensor(predictions, "s_ratio")
    target_segment = _required_tensor(targets, "segment")
    target_loaded = _required_tensor(targets, "loaded")
    target_bbox = _required_tensor(targets, "bbox")
    target_s_ratio = _required_tensor(targets, "s_ratio")
    raw_mask = _required_tensor(targets, "visibility_mask")

    if segment_logits.ndim != 3:
        raise ValueError("segment_logits must have shape [B, 8, C]")
    batch, slots, segment_classes = segment_logits.shape
    if slots != SHUTTLE_COUNT or segment_classes != SEGMENT_CLASS_COUNT:
        raise ValueError(
            f"segment_logits must have shape [B, {SHUTTLE_COUNT}, "
            f"{SEGMENT_CLASS_COUNT}]"
        )
    expected_shapes = {
        "loaded_logits": (batch, slots, LOADED_CLASS_COUNT),
        "prediction bbox": (batch, slots, 4),
        "prediction s_ratio": (batch, slots, 1),
        "target segment": (batch, slots),
        "target loaded": (batch, slots),
        "target bbox": (batch, slots, 4),
        "target s_ratio": (batch, slots, 1),
        "visibility_mask": (batch, slots),
    }
    tensors = {
        "loaded_logits": loaded_logits,
        "prediction bbox": pred_bbox,
        "prediction s_ratio": pred_s_ratio,
        "target segment": target_segment,
        "target loaded": target_loaded,
        "target bbox": target_bbox,
        "target s_ratio": target_s_ratio,
        "visibility_mask": raw_mask,
    }
    for name, expected in expected_shapes.items():
        if tuple(tensors[name].shape) != expected:
            raise ValueError(f"{name} must have shape {list(expected)}")

    if raw_mask.dtype == torch.bool:
        mask = raw_mask
    else:
        if not bool(torch.isfinite(raw_mask).all()):
            raise ValueError("visibility_mask must be finite")
        if not bool(((raw_mask == 0) | (raw_mask == 1)).all()):
            raise ValueError("visibility_mask must be boolean or contain only 0/1")
        mask = raw_mask.to(dtype=torch.bool)

    device = segment_logits.device
    if any(
        tensor.device != device
        for tensor in (loaded_logits, pred_bbox, pred_s_ratio)
    ):
        raise ValueError("all prediction tensors must be on the same device")
    for name, tensor in (
        ("segment_logits", segment_logits),
        ("loaded_logits", loaded_logits),
        ("prediction bbox", pred_bbox),
        ("prediction s_ratio", pred_s_ratio),
    ):
        if not bool(torch.isfinite(tensor).all()):
            raise ValueError(f"{name} must contain only finite values")
    mask = mask.to(device=device)
    target_segment = target_segment.to(device=device)
    target_loaded = target_loaded.to(device=device)
    target_bbox = target_bbox.to(device=device, dtype=pred_bbox.dtype)
    target_s_ratio = target_s_ratio.to(device=device, dtype=pred_s_ratio.dtype)
    for name, tensor in (
        ("visible target bbox", target_bbox),
        ("visible target s_ratio", target_s_ratio),
    ):
        selected = tensor[mask]
        if not bool(torch.isfinite(selected).all()):
            raise ValueError(f"{name} must contain only finite values")

    target_segment = _validated_indices(
        target_segment,
        mask,
        class_count=segment_classes,
        name="segment",
    )
    target_loaded = _validated_indices(
        target_loaded,
        mask,
        class_count=LOADED_CLASS_COUNT,
        name="loaded",
    )
    return (
        segment_logits,
        loaded_logits,
        pred_bbox,
        pred_s_ratio,
        target_segment,
        target_loaded,
        target_bbox,
        target_s_ratio,
        mask,
    )


def _validated_indices(
    values: torch.Tensor,
    mask: torch.Tensor,
    *,
    class_count: int,
    name: str,
) -> torch.Tensor:
    visible = values[mask]
    if visible.is_floating_point():
        if not bool(torch.isfinite(visible).all()):
            raise ValueError(f"visible {name} targets must be finite")
        if not bool((visible == visible.round()).all()):
            raise ValueError(f"visible {name} targets must be integer class indices")
    indexes = values.to(dtype=torch.long)
    visible_indexes = indexes[mask]
    if visible_indexes.numel() and bool(
        ((visible_indexes < 0) | (visible_indexes >= class_count)).any()
    ):
        raise ValueError(
            f"visible {name} targets must be in [0, {class_count - 1}]"
        )
    return indexes


def _bbox_scale(
    reference: torch.Tensor,
    image_size: Sequence[float] | torch.Tensor | None,
) -> torch.Tensor:
    if image_size is None:
        values = [1.0, 1.0, 1.0, 1.0]
    else:
        raw = torch.as_tensor(image_size, dtype=reference.dtype, device=reference.device)
        if raw.ndim != 1 or raw.numel() not in {2, 4}:
            raise ValueError("bbox_image_size must contain (width, height) or four scales")
        if not bool(torch.isfinite(raw).all()) or bool((raw <= 0).any()):
            raise ValueError("bbox_image_size values must be finite and positive")
        values = (
            [float(raw[0]), float(raw[1]), float(raw[0]), float(raw[1])]
            if raw.numel() == 2
            else [float(value) for value in raw]
        )
    return reference.new_tensor(values)


def _xywh_to_canonical_xyxy(boxes: torch.Tensor) -> torch.Tensor:
    x, y, width, height = boxes.unbind(dim=-1)
    other_x = x + width
    other_y = y + height
    return torch.stack(
        (
            torch.minimum(x, other_x),
            torch.minimum(y, other_y),
            torch.maximum(x, other_x),
            torch.maximum(y, other_y),
        ),
        dim=-1,
    )


def elementwise_box_iou_xywh(
    first: torch.Tensor,
    second: torch.Tensor,
    *,
    eps: float = 1e-7,
) -> torch.Tensor:
    """Compute aligned IoU for equally shaped ``[..., 4]`` XYWH tensors."""

    if first.shape != second.shape or first.ndim < 1 or first.shape[-1] != 4:
        raise ValueError("aligned XYWH box tensors must have identical [..., 4] shapes")
    first_xyxy = _xywh_to_canonical_xyxy(first)
    second_xyxy = _xywh_to_canonical_xyxy(second)
    intersection_low = torch.maximum(first_xyxy[..., :2], second_xyxy[..., :2])
    intersection_high = torch.minimum(first_xyxy[..., 2:], second_xyxy[..., 2:])
    intersection_size = (intersection_high - intersection_low).clamp_min(0.0)
    intersection = intersection_size[..., 0] * intersection_size[..., 1]
    first_size = (first_xyxy[..., 2:] - first_xyxy[..., :2]).clamp_min(0.0)
    second_size = (second_xyxy[..., 2:] - second_xyxy[..., :2]).clamp_min(0.0)
    first_area = first_size[..., 0] * first_size[..., 1]
    second_area = second_size[..., 0] * second_size[..., 1]
    union = first_area + second_area - intersection
    return (intersection / union.clamp_min(float(eps))).clamp(0.0, 1.0)


def elementwise_generalized_box_iou_xywh(
    first: torch.Tensor,
    second: torch.Tensor,
    *,
    eps: float = 1e-7,
) -> torch.Tensor:
    """Compute aligned generalized IoU for equally shaped XYWH tensors."""

    if first.shape != second.shape or first.ndim < 1 or first.shape[-1] != 4:
        raise ValueError("aligned XYWH box tensors must have identical [..., 4] shapes")
    first_xyxy = _xywh_to_canonical_xyxy(first)
    second_xyxy = _xywh_to_canonical_xyxy(second)
    intersection_low = torch.maximum(first_xyxy[..., :2], second_xyxy[..., :2])
    intersection_high = torch.minimum(first_xyxy[..., 2:], second_xyxy[..., 2:])
    intersection_size = (intersection_high - intersection_low).clamp_min(0.0)
    intersection = intersection_size[..., 0] * intersection_size[..., 1]
    first_size = (first_xyxy[..., 2:] - first_xyxy[..., :2]).clamp_min(0.0)
    second_size = (second_xyxy[..., 2:] - second_xyxy[..., :2]).clamp_min(0.0)
    first_area = first_size[..., 0] * first_size[..., 1]
    second_area = second_size[..., 0] * second_size[..., 1]
    union = first_area + second_area - intersection
    iou = intersection / union.clamp_min(float(eps))

    enclosing_low = torch.minimum(first_xyxy[..., :2], second_xyxy[..., :2])
    enclosing_high = torch.maximum(first_xyxy[..., 2:], second_xyxy[..., 2:])
    enclosing_size = (enclosing_high - enclosing_low).clamp_min(0.0)
    enclosing_area = enclosing_size[..., 0] * enclosing_size[..., 1]
    penalty = (enclosing_area - union) / enclosing_area.clamp_min(float(eps))
    return (iou - penalty).clamp(-1.0, 1.0)


def _masked_mean(values: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
    selected = values[mask]
    return selected.mean() if selected.numel() else selected.sum()


def _masked_cross_entropy(
    logits: torch.Tensor,
    targets: torch.Tensor,
    mask: torch.Tensor,
    *,
    class_weights: torch.Tensor | None = None,
    label_smoothing: float = 0.0,
) -> torch.Tensor:
    selected_logits = logits[mask]
    selected_targets = targets[mask]
    smoothing = _validated_unit_interval("label_smoothing", label_smoothing)
    per_slot_weights = None
    if class_weights is not None:
        weights = class_weights.to(device=logits.device, dtype=logits.dtype)
        valid_shapes = {
            (logits.shape[-1],),
            (2, logits.shape[-1]),
            (SHUTTLE_COUNT, logits.shape[-1]),
        }
        if tuple(weights.shape) not in valid_shapes:
            raise ValueError(
                "segment_class_weights must have shape "
                f"[{logits.shape[-1]}], [2, {logits.shape[-1]}], or "
                f"[{SHUTTLE_COUNT}, {logits.shape[-1]}]"
            )
        if not bool(torch.isfinite(weights).all()) or bool((weights <= 0).any()):
            raise ValueError("segment_class_weights must be finite and positive")
        if weights.ndim == 1:
            per_slot_weights = weights.reshape(1, 1, -1).expand(
                logits.shape[0],
                logits.shape[1],
                -1,
            )
        elif weights.shape[0] == 2:
            side_by_slot = (
                torch.arange(SHUTTLE_COUNT, device=logits.device) >= 4
            ).to(dtype=torch.long)
            per_slot_weights = (
                weights.index_select(0, side_by_slot)
                .unsqueeze(0)
                .expand(logits.shape[0], -1, -1)
            )
        else:
            per_slot_weights = weights.unsqueeze(0).expand(logits.shape[0], -1, -1)

    if selected_targets.numel() == 0:
        return selected_logits.sum()

    log_probabilities = F.log_softmax(selected_logits, dim=-1)
    target_nll = -log_probabilities.gather(
        dim=-1,
        index=selected_targets.unsqueeze(-1),
    ).squeeze(-1)
    if smoothing:
        uniform_nll = -log_probabilities.mean(dim=-1)
        per_entry_loss = (1.0 - smoothing) * target_nll + smoothing * uniform_nll
    else:
        per_entry_loss = target_nll
    if per_slot_weights is None:
        return per_entry_loss.mean()
    selected_class_weights = per_slot_weights[mask].gather(
        dim=-1,
        index=selected_targets.unsqueeze(-1),
    ).squeeze(-1)
    return (per_entry_loss * selected_class_weights).sum() / selected_class_weights.sum()


def topology_expected_distance_penalty(
    segment_logits: torch.Tensor,
    target_segment: torch.Tensor,
    visibility_mask: torch.Tensor,
    distance_matrix: Sequence[Sequence[float]] | torch.Tensor,
) -> torch.Tensor:
    """Expected graph distance from the true segment under predicted softmax."""

    class_count = segment_logits.shape[-1]
    distances = torch.as_tensor(
        distance_matrix,
        dtype=segment_logits.dtype,
        device=segment_logits.device,
    )
    if tuple(distances.shape) != (class_count, class_count):
        raise ValueError(
            f"topology distance matrix must have shape [{class_count}, {class_count}]"
        )
    if not bool(torch.isfinite(distances).all()) or bool((distances < 0).any()):
        raise ValueError("topology distances must be finite and non-negative")
    selected_logits = segment_logits[visibility_mask]
    selected_targets = target_segment[visibility_mask]
    if selected_targets.numel() == 0:
        return selected_logits.sum()
    probabilities = F.softmax(selected_logits, dim=-1)
    true_distances = distances.index_select(0, selected_targets)
    return (probabilities * true_distances).sum(dim=-1).mean()


def compute_visual_training_v4_loss(
    predictions: Mapping[str, Any],
    targets: Mapping[str, Any],
    *,
    segment_class_weights: Sequence[float] | torch.Tensor | None = None,
    segment_class_counts: Sequence[float] | torch.Tensor | None = None,
    class_weight_cap: float = 4.0,
    topology_distance_matrix: Sequence[Sequence[float]] | torch.Tensor | None = None,
    loss_weights: Mapping[str, float] | None = None,
    bbox_image_size: Sequence[float] | torch.Tensor | None = DEFAULT_BBOX_IMAGE_SIZE,
    segment_label_smoothing: float = DEFAULT_SEGMENT_LABEL_SMOOTHING,
    loaded_label_smoothing: float = DEFAULT_LOADED_LABEL_SMOOTHING,
    s_ratio_beta: float = DEFAULT_S_RATIO_BETA,
    bbox_l1_weight: float = DEFAULT_BBOX_L1_WEIGHT,
    bbox_giou_weight: float = DEFAULT_BBOX_GIOU_WEIGHT,
) -> dict[str, Any]:
    """Compute the masked multi-task V4 objective.

    Segment weights/counts may be global ``[14]``, per-side ``[2, 14]``, or
    per-fixed-slot ``[8, 14]``.  ``segment_class_counts`` is a convenience
    input for generating capped inverse-square-root weights; pass train-split
    counts rather than recomputing them per mini-batch.  Bounding boxes are
    expected to be normalized.  ``bbox_image_size=(width, height)`` remains an
    explicit compatibility option when both prediction and target are pixels.
    """

    (
        segment_logits,
        loaded_logits,
        pred_bbox,
        pred_s_ratio,
        target_segment,
        target_loaded,
        target_bbox,
        target_s_ratio,
        mask,
    ) = _validated_contract(predictions, targets)
    segment_smoothing = _validated_unit_interval(
        "segment_label_smoothing",
        segment_label_smoothing,
    )
    loaded_smoothing = _validated_unit_interval(
        "loaded_label_smoothing",
        loaded_label_smoothing,
    )
    ratio_beta = _validated_positive("s_ratio_beta", s_ratio_beta)
    parsed_bbox_l1_weight = _validated_nonnegative(
        "bbox_l1_weight",
        bbox_l1_weight,
    )
    parsed_bbox_giou_weight = _validated_nonnegative(
        "bbox_giou_weight",
        bbox_giou_weight,
    )
    if parsed_bbox_l1_weight == 0.0 and parsed_bbox_giou_weight == 0.0:
        raise ValueError("bbox_l1_weight and bbox_giou_weight cannot both be zero")
    if segment_class_weights is not None and segment_class_counts is not None:
        raise ValueError("pass segment_class_weights or segment_class_counts, not both")
    if segment_class_counts is not None:
        segment_class_weights = inverse_sqrt_class_weights(
            segment_class_counts,
            cap=class_weight_cap,
            dtype=segment_logits.dtype,
            device=segment_logits.device,
        )

    segment_ce = _masked_cross_entropy(
        segment_logits,
        target_segment,
        mask,
        class_weights=(
            torch.as_tensor(segment_class_weights)
            if segment_class_weights is not None
            else None
        ),
        label_smoothing=segment_smoothing,
    )
    loaded_ce = _masked_cross_entropy(
        loaded_logits,
        target_loaded,
        mask,
        label_smoothing=loaded_smoothing,
    )

    scale = _bbox_scale(pred_bbox, bbox_image_size)
    normalized_pred_bbox = pred_bbox / scale
    normalized_target_bbox = target_bbox / scale
    bbox_l1_per_slot = (normalized_pred_bbox - normalized_target_bbox).abs().mean(dim=-1)
    bbox_giou_per_slot = 1.0 - elementwise_generalized_box_iou_xywh(
        normalized_pred_bbox,
        normalized_target_bbox,
    )
    bbox_l1 = _masked_mean(bbox_l1_per_slot, mask)
    bbox_giou = _masked_mean(bbox_giou_per_slot, mask)
    bbox_loss = (
        parsed_bbox_l1_weight * bbox_l1
        + parsed_bbox_giou_weight * bbox_giou
    )

    s_ratio_per_slot = F.smooth_l1_loss(
        pred_s_ratio.squeeze(-1),
        target_s_ratio.squeeze(-1),
        reduction="none",
        beta=ratio_beta,
    )
    s_ratio_loss = _masked_mean(s_ratio_per_slot, mask)

    if topology_distance_matrix is None:
        topology_loss = segment_logits[mask].sum() * 0.0
    else:
        topology_loss = topology_expected_distance_penalty(
            segment_logits,
            target_segment,
            mask,
            topology_distance_matrix,
        )

    weights = _loss_weights(loss_weights)
    total = (
        weights["segment"] * segment_ce
        + weights["s_ratio"] * s_ratio_loss
        + weights["loaded"] * loaded_ce
        + weights["bbox"] * bbox_loss
        + weights["topology"] * topology_loss
    )
    return {
        "loss": total,
        "total_loss": total,
        "segment_ce": segment_ce,
        "loaded_ce": loaded_ce,
        "bbox": bbox_loss,
        "bbox_l1": bbox_l1,
        "bbox_giou": bbox_giou,
        "s_ratio": s_ratio_loss,
        "topology": topology_loss,
        "visible_count": int(mask.sum().detach().cpu()),
        "loss_weights": weights,
        "head_parameters": {
            "segment_label_smoothing": segment_smoothing,
            "loaded_label_smoothing": loaded_smoothing,
            "s_ratio_beta": ratio_beta,
            "bbox_l1_weight": parsed_bbox_l1_weight,
            "bbox_giou_weight": parsed_bbox_giou_weight,
        },
        "segment_class_weights": (
            None
            if segment_class_weights is None
            else torch.as_tensor(
                segment_class_weights,
                dtype=segment_logits.dtype,
                device=segment_logits.device,
            )
        ),
    }


def _optional_ratio(numerator: int, denominator: int) -> float:
    return float(numerator / denominator) if denominator else 0.0


def _segment_metrics(
    predicted: torch.Tensor,
    top2_predicted: torch.Tensor,
    target: torch.Tensor,
    mask: torch.Tensor,
    *,
    class_count: int,
    class_names: Sequence[str] | None,
) -> dict[str, Any]:
    predicted = predicted[mask].to(dtype=torch.long).detach().cpu()
    top2_predicted = top2_predicted[mask].to(dtype=torch.long).detach().cpu()
    target = target[mask].to(dtype=torch.long).detach().cpu()
    count = int(target.numel())
    confusion = torch.zeros((class_count, class_count), dtype=torch.long)
    if count:
        flat = target * class_count + predicted
        confusion = torch.bincount(
            flat,
            minlength=class_count * class_count,
        ).reshape(class_count, class_count)

    per_class: list[dict[str, Any]] = []
    recalls: list[float] = []
    f1_values: list[float] = []
    zero_recall_classes: list[int | str] = []
    for index in range(class_count):
        true_positive = int(confusion[index, index])
        support = int(confusion[index, :].sum())
        predicted_count = int(confusion[:, index].sum())
        precision = _optional_ratio(true_positive, predicted_count)
        recall = _optional_ratio(true_positive, support)
        f1 = (
            2.0 * precision * recall / (precision + recall)
            if precision + recall > 0.0
            else 0.0
        )
        label: int | str = class_names[index] if class_names is not None else index
        if support > 0 and recall == 0.0:
            zero_recall_classes.append(label)
        per_class.append(
            {
                "class_index": index,
                "class_name": class_names[index] if class_names is not None else None,
                "support": support,
                "predicted": predicted_count,
                "true_positive": true_positive,
                "precision": precision,
                "recall": recall,
                "f1": f1,
            }
        )
        recalls.append(recall)
        f1_values.append(f1)

    correct = int(confusion.diag().sum())
    top2_correct = int(
        (top2_predicted == target.unsqueeze(-1)).any(dim=-1).sum()
    )
    return {
        "visible_count": count,
        "top1_accuracy": _optional_ratio(correct, count),
        "top2_accuracy": _optional_ratio(top2_correct, count),
        "macro_recall": float(sum(recalls) / class_count),
        "macro_f1": float(sum(f1_values) / class_count),
        "zero_recall_classes": zero_recall_classes,
        "per_class": per_class,
        "confusion_matrix": confusion.tolist(),
    }


def _absolute_error_metrics(errors: torch.Tensor) -> dict[str, float | int]:
    detached = errors.detach().float()
    count = int(detached.numel())
    if not count:
        return {"count": 0, "mae_m": 0.0, "p95_m": 0.0}
    return {
        "count": count,
        "mae_m": float(detached.mean().cpu()),
        "p95_m": float(torch.quantile(detached, 0.95).cpu()),
    }


def _validated_segment_lengths(
    segment_lengths_by_side: Sequence[Sequence[float]] | torch.Tensor,
    reference: torch.Tensor,
) -> torch.Tensor:
    lengths = torch.as_tensor(
        segment_lengths_by_side,
        dtype=reference.dtype,
        device=reference.device,
    )
    if tuple(lengths.shape) != (2, SEGMENT_CLASS_COUNT):
        raise ValueError(
            "segment_lengths_by_side must have shape "
            f"[2, {SEGMENT_CLASS_COUNT}]"
        )
    if not bool(torch.isfinite(lengths).all()) or bool((lengths <= 0).any()):
        raise ValueError("segment lengths must be finite and greater than zero")
    return lengths


@torch.no_grad()
def compute_visual_training_v4_metrics(
    predictions: Mapping[str, Any],
    targets: Mapping[str, Any],
    *,
    segment_class_names: Sequence[str] | None = None,
    segment_lengths_by_side: Sequence[Sequence[float]] | torch.Tensor | None = None,
) -> dict[str, Any]:
    """Compute masking-aware V4 evaluation metrics.

    Macro recall and F1 use all 14 configured classes with zero-division set to
    zero.  ``per_class`` and ``zero_recall_classes`` make collapsed classes
    explicit instead of hiding them inside one aggregate.  Supplying the
    physical ``[left/right, segment]`` length matrix adds ``s_m`` errors only
    when the predicted segment is correct, plus an oracle-true-segment
    diagnostic for the ratio head.  Local ``s_m`` values from different
    segments are intentionally never compared.
    """

    (
        segment_logits,
        loaded_logits,
        pred_bbox,
        pred_s_ratio,
        target_segment,
        target_loaded,
        target_bbox,
        target_s_ratio,
        mask,
    ) = _validated_contract(predictions, targets)
    if segment_class_names is not None and len(segment_class_names) != SEGMENT_CLASS_COUNT:
        raise ValueError(
            f"segment_class_names must contain {SEGMENT_CLASS_COUNT} names"
        )
    segment_prediction = segment_logits.argmax(dim=-1)
    segment_top2_prediction = segment_logits.topk(k=2, dim=-1).indices
    overall = _segment_metrics(
        segment_prediction,
        segment_top2_prediction,
        target_segment,
        mask,
        class_count=SEGMENT_CLASS_COUNT,
        class_names=segment_class_names,
    )
    side_masks = {
        "left": mask & (
            torch.arange(SHUTTLE_COUNT, device=mask.device).unsqueeze(0) < 4
        ),
        "right": mask & (
            torch.arange(SHUTTLE_COUNT, device=mask.device).unsqueeze(0) >= 4
        ),
    }
    per_side = {
        side: _segment_metrics(
            segment_prediction,
            segment_top2_prediction,
            target_segment,
            side_mask,
            class_count=SEGMENT_CLASS_COUNT,
            class_names=segment_class_names,
        )
        for side, side_mask in side_masks.items()
    }
    slot_indexes = torch.arange(SHUTTLE_COUNT, device=mask.device).unsqueeze(0)
    per_identity = {
        identity: _segment_metrics(
            segment_prediction,
            segment_top2_prediction,
            target_segment,
            mask & (slot_indexes == slot),
            class_count=SEGMENT_CLASS_COUNT,
            class_names=segment_class_names,
        )
        for slot, identity in enumerate(SLOT_IDENTITIES)
    }

    loaded_prediction = loaded_logits.argmax(dim=-1)
    visible_loaded_prediction = loaded_prediction[mask]
    visible_loaded_target = target_loaded[mask]
    loaded_count = int(visible_loaded_target.numel())
    loaded_accuracy = (
        float((visible_loaded_prediction == visible_loaded_target).float().mean().cpu())
        if loaded_count
        else 0.0
    )

    s_ratio_errors = (
        pred_s_ratio.squeeze(-1) - target_s_ratio.squeeze(-1)
    ).abs()[mask]
    if s_ratio_errors.numel():
        s_ratio_mae = float(s_ratio_errors.mean().cpu())
        s_ratio_p95 = float(torch.quantile(s_ratio_errors.float(), 0.95).cpu())
    else:
        s_ratio_mae = 0.0
        s_ratio_p95 = 0.0

    bbox_ious = elementwise_box_iou_xywh(pred_bbox, target_bbox)[mask]
    bbox_iou = float(bbox_ious.mean().cpu()) if bbox_ious.numel() else 0.0
    visible_segment_correct = (segment_prediction == target_segment)[mask]
    joint_localization = {
        "s_ratio_tolerance_0_05": _optional_ratio(
            int((visible_segment_correct & (s_ratio_errors <= 0.05)).sum()),
            int(s_ratio_errors.numel()),
        ),
        "s_ratio_tolerance_0_12": _optional_ratio(
            int((visible_segment_correct & (s_ratio_errors <= 0.12)).sum()),
            int(s_ratio_errors.numel()),
        ),
    }
    result = {
        "visible_count": overall["visible_count"],
        "segment_top1_accuracy": overall["top1_accuracy"],
        "segment_top2_accuracy": overall["top2_accuracy"],
        "segment_macro_recall": overall["macro_recall"],
        "segment_macro_f1": overall["macro_f1"],
        "segment_zero_recall_classes": overall["zero_recall_classes"],
        "segment_per_class": overall["per_class"],
        "segment_confusion_matrix": overall["confusion_matrix"],
        "per_side": per_side,
        "per_identity": per_identity,
        "loaded_accuracy": loaded_accuracy,
        "s_ratio_mae": s_ratio_mae,
        "s_ratio_p95": s_ratio_p95,
        "bbox_iou": bbox_iou,
        "joint_localization_accuracy": joint_localization,
        "joint_localization_accuracy_s_ratio_0_05": joint_localization[
            "s_ratio_tolerance_0_05"
        ],
        "joint_localization_accuracy_s_ratio_0_12": joint_localization[
            "s_ratio_tolerance_0_12"
        ],
    }
    if segment_lengths_by_side is None:
        return result

    lengths = _validated_segment_lengths(segment_lengths_by_side, pred_s_ratio)
    side_indexes = (slot_indexes >= 4).to(dtype=torch.long).expand_as(mask)
    visible_sides = side_indexes[mask]
    visible_predicted_segments = segment_prediction[mask]
    visible_true_segments = target_segment[mask]
    visible_predicted_ratios = pred_s_ratio.squeeze(-1)[mask]
    visible_true_ratios = target_s_ratio.squeeze(-1)[mask]
    true_lengths = lengths[visible_sides, visible_true_segments]
    true_s_m = visible_true_ratios * true_lengths
    oracle_true_segment_predicted_s_m = visible_predicted_ratios * true_lengths
    oracle_true_segment_errors = (
        oracle_true_segment_predicted_s_m - true_s_m
    ).abs()
    correct_segment_mask = visible_predicted_segments == visible_true_segments
    correct_segment_errors = oracle_true_segment_errors[correct_segment_mask]
    correct_segment_metrics = _absolute_error_metrics(correct_segment_errors)
    correct_segment_metrics["coverage"] = _optional_ratio(
        int(correct_segment_mask.sum()),
        int(correct_segment_mask.numel()),
    )
    s_m_metrics = {
        "correct_segment_only": correct_segment_metrics,
        "oracle_true_segment_ratio_diagnostic": _absolute_error_metrics(
            oracle_true_segment_errors
        ),
    }
    result.update(
        {
            "s_m_metrics": s_m_metrics,
            "correct_segment_s_m_mae_m": correct_segment_metrics["mae_m"],
            "correct_segment_s_m_p95_m": correct_segment_metrics["p95_m"],
            "correct_segment_s_m_count": correct_segment_metrics["count"],
            "correct_segment_s_m_coverage": correct_segment_metrics["coverage"],
            "oracle_true_segment_ratio_s_m_mae_m": s_m_metrics[
                "oracle_true_segment_ratio_diagnostic"
            ]["mae_m"],
            "oracle_true_segment_ratio_s_m_p95_m": s_m_metrics[
                "oracle_true_segment_ratio_diagnostic"
            ]["p95_m"],
        }
    )
    return result


# Concise aliases for training loops that already namespace this module as V4.
compute_v4_loss = compute_visual_training_v4_loss
compute_v4_metrics = compute_visual_training_v4_metrics
