#!/usr/bin/env python3
"""Validation-only segment calibration for the Room 315 V4 visual model.

The public API is intentionally independent of the trainer and dataset code.
It accepts segment logits with shape ``[B, 8, 14]``, segment class indices with
shape ``[B, 8]``, and a boolean visibility mask with shape ``[B, 8]``.

A single positive scalar temperature is fitted on visible validation targets.
The fitted temperature is shared by both rails; per-side results are diagnostic
views of that same calibrator, not independently fitted temperatures.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Any

import torch
import torch.nn.functional as F


SHUTTLE_COUNT = 8
SEGMENT_CLASS_COUNT = 14
LEFT_SLOT_COUNT = 4
DEFAULT_COVERAGES = (1.0, 0.95, 0.90, 0.80, 0.70)
DEFAULT_ECE_BINS = 15
DEFAULT_MIN_TEMPERATURE = 0.05
DEFAULT_MAX_TEMPERATURE = 20.0
DEFAULT_GRID_SIZE = 129
DEFAULT_REFINEMENT_STEPS = 4
REPORT_SCHEMA_VERSION = "room315.visual_segment_calibration.v4.v1"
EVALUATION_REPORT_SCHEMA_VERSION = (
    "room315.visual_segment_calibration_evaluation.v4.v1"
)


def _required_tensor(name: str, value: Any) -> torch.Tensor:
    if not isinstance(value, torch.Tensor):
        raise TypeError(f"{name} must be a torch.Tensor")
    return value


def _validated_inputs(
    segment_logits: torch.Tensor,
    segment_targets: torch.Tensor,
    visibility_mask: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Validate and flatten visible inputs onto deterministic CPU float64."""

    logits = _required_tensor("segment_logits", segment_logits)
    targets = _required_tensor("segment_targets", segment_targets)
    mask = _required_tensor("visibility_mask", visibility_mask)

    if logits.ndim != 3 or tuple(logits.shape[1:]) != (
        SHUTTLE_COUNT,
        SEGMENT_CLASS_COUNT,
    ):
        raise ValueError(
            "segment_logits must have shape [B, 8, 14]"
        )
    batch_size = int(logits.shape[0])
    if tuple(targets.shape) != (batch_size, SHUTTLE_COUNT):
        raise ValueError("segment_targets must have shape [B, 8]")
    if tuple(mask.shape) != (batch_size, SHUTTLE_COUNT):
        raise ValueError("visibility_mask must have shape [B, 8]")
    if mask.dtype != torch.bool:
        raise TypeError("visibility_mask must have boolean dtype")
    if not logits.is_floating_point() or logits.is_complex():
        raise TypeError("segment_logits must have a real floating-point dtype")
    if not bool(torch.isfinite(logits).all()):
        raise ValueError("segment_logits must contain only finite values")

    if targets.dtype == torch.bool or targets.is_complex():
        raise TypeError("segment_targets must contain integer class indices")
    if targets.is_floating_point():
        if not bool(torch.isfinite(targets).all()):
            raise ValueError("segment_targets must contain only finite values")
        if not bool((targets == targets.round()).all()):
            raise ValueError("segment_targets must contain integer class indices")

    cpu_logits = logits.detach().to(device="cpu", dtype=torch.float64)
    cpu_targets = targets.detach().to(device="cpu", dtype=torch.long)
    cpu_mask = mask.detach().to(device="cpu")
    slot_indices = torch.arange(SHUTTLE_COUNT, dtype=torch.long).expand(
        batch_size, SHUTTLE_COUNT
    )

    visible_logits = cpu_logits[cpu_mask]
    visible_targets = cpu_targets[cpu_mask]
    visible_slots = slot_indices[cpu_mask]
    if visible_targets.numel() == 0:
        raise ValueError("visibility_mask must select at least one segment target")
    if bool(
        ((visible_targets < 0) | (visible_targets >= SEGMENT_CLASS_COUNT)).any()
    ):
        raise ValueError("visible segment targets must be class indices in [0, 14)")
    return visible_logits, visible_targets, visible_slots


def _validated_temperature(value: float) -> float:
    temperature = float(value)
    if not math.isfinite(temperature) or temperature <= 0.0:
        raise ValueError("temperature must be finite and greater than zero")
    return temperature


def _validated_temperature_search(
    min_temperature: float,
    max_temperature: float,
    grid_size: int,
    refinement_steps: int,
) -> tuple[float, float, int, int]:
    lower = _validated_temperature(min_temperature)
    upper = _validated_temperature(max_temperature)
    if lower >= upper:
        raise ValueError("min_temperature must be less than max_temperature")
    if not lower <= 1.0 <= upper:
        raise ValueError("temperature search bounds must contain 1.0")
    if isinstance(grid_size, bool) or not isinstance(grid_size, int) or grid_size < 3:
        raise ValueError("grid_size must be an integer of at least 3")
    if (
        isinstance(refinement_steps, bool)
        or not isinstance(refinement_steps, int)
        or refinement_steps < 0
    ):
        raise ValueError("refinement_steps must be a non-negative integer")
    return lower, upper, grid_size, refinement_steps


def _validated_ece_bins(value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError("ece_bins must be a positive integer")
    return value


def _validated_coverages(coverages: Sequence[float]) -> tuple[float, ...]:
    if isinstance(coverages, (str, bytes)):
        raise TypeError("coverages must be a sequence of numeric values")
    try:
        raw_values = list(coverages)
    except TypeError as exc:
        raise TypeError("coverages must be a sequence of numeric values") from exc
    if not raw_values:
        raise ValueError("coverages must not be empty")
    values: list[float] = []
    for raw_value in raw_values:
        if isinstance(raw_value, bool):
            raise ValueError("coverage values must be finite and in (0, 1]")
        value = float(raw_value)
        if not math.isfinite(value) or not 0.0 < value <= 1.0:
            raise ValueError("coverage values must be finite and in (0, 1]")
        values.append(value)
    if len(set(values)) != len(values):
        raise ValueError("coverage values must be unique")
    return tuple(values)


def _validated_data_role(data_role: str) -> str:
    if data_role != "validation":
        raise ValueError("temperature fitting is validation-only")
    return data_role


def _validated_evaluation_role(data_role: str) -> str:
    if data_role not in {"validation", "canary"}:
        raise ValueError(
            "saved-temperature evaluation permits only validation or canary data"
        )
    return data_role


def _validated_temperature_source_role(source_temperature_role: str) -> str:
    if source_temperature_role != "validation":
        raise ValueError("saved temperature must originate from validation")
    return source_temperature_role


def _probability_outputs(
    logits: torch.Tensor,
    targets: torch.Tensor,
    temperature: float,
) -> tuple[float, torch.Tensor, torch.Tensor]:
    scaled_logits = logits / temperature
    if not bool(torch.isfinite(scaled_logits).all()):
        raise ValueError("temperature scaling produced non-finite logits")
    log_probabilities = F.log_softmax(scaled_logits, dim=-1)
    nll_tensor = F.nll_loss(log_probabilities, targets, reduction="mean")
    probabilities = log_probabilities.exp()
    confidence, prediction = probabilities.max(dim=-1)
    correct = prediction.eq(targets)
    nll = float(nll_tensor)
    if not math.isfinite(nll) or not bool(torch.isfinite(confidence).all()):
        raise ValueError("calibration metrics became non-finite")
    return nll, confidence, correct


def _ece(
    confidence: torch.Tensor,
    correct: torch.Tensor,
    ece_bins: int,
) -> float:
    count = int(confidence.numel())
    if count == 0:
        raise ValueError("ECE requires at least one observation")
    # Bins are (lower, upper], except that zero is included in the first bin.
    bin_index = torch.ceil(confidence * ece_bins).to(dtype=torch.long) - 1
    bin_index = bin_index.clamp(min=0, max=ece_bins - 1)
    correct_float = correct.to(dtype=torch.float64)
    result = torch.zeros((), dtype=torch.float64)
    for index in range(ece_bins):
        selected = bin_index == index
        selected_count = int(selected.sum())
        if selected_count == 0:
            continue
        bin_accuracy = correct_float[selected].mean()
        bin_confidence = confidence[selected].mean()
        result += (selected_count / count) * (bin_accuracy - bin_confidence).abs()
    value = float(result)
    if not math.isfinite(value):
        raise ValueError("ECE became non-finite")
    return value


def _evaluation(
    logits: torch.Tensor,
    targets: torch.Tensor,
    temperature: float,
    ece_bins: int,
) -> dict[str, float | int]:
    nll, confidence, correct = _probability_outputs(
        logits, targets, temperature
    )
    return {
        "visible_count": int(targets.numel()),
        "nll": nll,
        "ece": _ece(confidence, correct, ece_bins),
        "accuracy": float(correct.to(dtype=torch.float64).mean()),
        "mean_confidence": float(confidence.mean()),
    }


def segment_negative_log_likelihood(
    segment_logits: torch.Tensor,
    segment_targets: torch.Tensor,
    visibility_mask: torch.Tensor,
    *,
    temperature: float = 1.0,
) -> float:
    """Return masked segment NLL at a positive scalar temperature."""

    logits, targets, _ = _validated_inputs(
        segment_logits, segment_targets, visibility_mask
    )
    value, _, _ = _probability_outputs(
        logits, targets, _validated_temperature(temperature)
    )
    return value


def segment_expected_calibration_error(
    segment_logits: torch.Tensor,
    segment_targets: torch.Tensor,
    visibility_mask: torch.Tensor,
    *,
    temperature: float = 1.0,
    ece_bins: int = DEFAULT_ECE_BINS,
) -> float:
    """Return masked top-label expected calibration error."""

    logits, targets, _ = _validated_inputs(
        segment_logits, segment_targets, visibility_mask
    )
    _, confidence, correct = _probability_outputs(
        logits, targets, _validated_temperature(temperature)
    )
    return _ece(confidence, correct, _validated_ece_bins(ece_bins))


def _nll_for_temperature(
    logits: torch.Tensor,
    targets: torch.Tensor,
    temperature: float,
) -> float:
    scaled_logits = logits / temperature
    if not bool(torch.isfinite(scaled_logits).all()):
        return math.inf
    value = float(F.cross_entropy(scaled_logits, targets, reduction="mean"))
    return value if math.isfinite(value) else math.inf


def _log_grid(lower: float, upper: float, grid_size: int) -> list[float]:
    log_values = torch.linspace(
        math.log(lower),
        math.log(upper),
        steps=grid_size,
        dtype=torch.float64,
    )
    values = [float(value.exp()) for value in log_values]
    if lower <= 1.0 <= upper:
        values.append(1.0)
    return sorted(set(values))


def _fit_flat_temperature(
    logits: torch.Tensor,
    targets: torch.Tensor,
    *,
    min_temperature: float,
    max_temperature: float,
    grid_size: int,
    refinement_steps: int,
) -> tuple[float, float, int]:
    search_lower = min_temperature
    search_upper = max_temperature
    base_nll = _nll_for_temperature(logits, targets, 1.0)
    if not math.isfinite(base_nll):
        raise ValueError("uncalibrated NLL is non-finite")

    best_temperature = 1.0
    best_nll = base_nll
    evaluation_count = 0
    for _ in range(refinement_steps + 1):
        candidates = _log_grid(search_lower, search_upper, grid_size)
        candidate_nll = [
            _nll_for_temperature(logits, targets, temperature)
            for temperature in candidates
        ]
        evaluation_count += len(candidates)
        local_index = min(
            range(len(candidates)),
            key=lambda index: (
                candidate_nll[index],
                abs(math.log(candidates[index])),
                candidates[index],
            ),
        )
        local_temperature = candidates[local_index]
        local_nll = candidate_nll[local_index]
        if (local_nll, abs(math.log(local_temperature))) < (
            best_nll,
            abs(math.log(best_temperature)),
        ):
            best_temperature = local_temperature
            best_nll = local_nll

        left_index = max(0, local_index - 1)
        right_index = min(len(candidates) - 1, local_index + 1)
        next_lower = candidates[left_index]
        next_upper = candidates[right_index]
        if next_lower >= next_upper:
            break
        search_lower, search_upper = next_lower, next_upper

    improvement_tolerance = max(1.0e-12, abs(base_nll) * 1.0e-12)
    if best_nll >= base_nll - improvement_tolerance:
        return 1.0, base_nll, evaluation_count
    bounded_temperature = min(
        max(best_temperature, min_temperature), max_temperature
    )
    return bounded_temperature, best_nll, evaluation_count


def fit_segment_temperature(
    segment_logits: torch.Tensor,
    segment_targets: torch.Tensor,
    visibility_mask: torch.Tensor,
    *,
    min_temperature: float = DEFAULT_MIN_TEMPERATURE,
    max_temperature: float = DEFAULT_MAX_TEMPERATURE,
    grid_size: int = DEFAULT_GRID_SIZE,
    refinement_steps: int = DEFAULT_REFINEMENT_STEPS,
    data_role: str = "validation",
) -> float:
    """Fit a deterministic bounded scalar temperature on validation targets."""

    _validated_data_role(data_role)
    lower, upper, size, refinements = _validated_temperature_search(
        min_temperature, max_temperature, grid_size, refinement_steps
    )
    logits, targets, _ = _validated_inputs(
        segment_logits, segment_targets, visibility_mask
    )
    temperature, _, _ = _fit_flat_temperature(
        logits,
        targets,
        min_temperature=lower,
        max_temperature=upper,
        grid_size=size,
        refinement_steps=refinements,
    )
    return temperature


def _selective_curve(
    logits: torch.Tensor,
    targets: torch.Tensor,
    *,
    temperature: float,
    coverages: tuple[float, ...],
) -> list[dict[str, float | int]]:
    _, confidence, correct = _probability_outputs(
        logits, targets, temperature
    )
    count = int(targets.numel())
    order = torch.tensor(
        sorted(
            range(count),
            key=lambda index: (-float(confidence[index]), index),
        ),
        dtype=torch.long,
    )
    sorted_confidence = confidence[order]
    sorted_correct = correct[order].to(dtype=torch.float64)
    result: list[dict[str, float | int]] = []
    for requested_coverage in coverages:
        retained_count = max(
            1,
            min(
                count,
                int(math.ceil(requested_coverage * count - 1.0e-12)),
            ),
        )
        retained_confidence = sorted_confidence[:retained_count]
        retained_correct = sorted_correct[:retained_count]
        result.append({
            "requested_coverage": requested_coverage,
            "achieved_coverage": retained_count / count,
            "retained_count": retained_count,
            "total_count": count,
            "confidence_threshold": float(retained_confidence[-1]),
            "mean_confidence": float(retained_confidence.mean()),
            "accuracy": float(retained_correct.mean()),
        })
    return result


def segment_selective_accuracy_curve(
    segment_logits: torch.Tensor,
    segment_targets: torch.Tensor,
    visibility_mask: torch.Tensor,
    *,
    temperature: float = 1.0,
    coverages: Sequence[float] = DEFAULT_COVERAGES,
) -> list[dict[str, float | int]]:
    """Return confidence-ranked top-1 accuracy at requested coverages."""

    logits, targets, _ = _validated_inputs(
        segment_logits, segment_targets, visibility_mask
    )
    return _selective_curve(
        logits,
        targets,
        temperature=_validated_temperature(temperature),
        coverages=_validated_coverages(coverages),
    )


def _empty_side_report(
    slot_indices: list[int],
) -> dict[str, Any]:
    return {
        "slot_indices": slot_indices,
        "visible_count": 0,
        "available": False,
        "uncalibrated_nll": None,
        "calibrated_nll": None,
        "uncalibrated_ece": None,
        "calibrated_ece": None,
        "uncalibrated": None,
        "calibrated": None,
        "selective_curve": [],
    }


def _side_report(
    logits: torch.Tensor,
    targets: torch.Tensor,
    *,
    slot_indices: list[int],
    temperature: float,
    ece_bins: int,
    coverages: tuple[float, ...],
) -> dict[str, Any]:
    if targets.numel() == 0:
        return _empty_side_report(slot_indices)
    uncalibrated = _evaluation(logits, targets, 1.0, ece_bins)
    calibrated = _evaluation(logits, targets, temperature, ece_bins)
    return {
        "slot_indices": slot_indices,
        "visible_count": int(targets.numel()),
        "available": True,
        "uncalibrated_nll": uncalibrated["nll"],
        "calibrated_nll": calibrated["nll"],
        "uncalibrated_ece": uncalibrated["ece"],
        "calibrated_ece": calibrated["ece"],
        "uncalibrated": uncalibrated,
        "calibrated": calibrated,
        "selective_curve": _selective_curve(
            logits,
            targets,
            temperature=temperature,
            coverages=coverages,
        ),
    }


def _temperature_evaluation_side_report(
    logits: torch.Tensor,
    targets: torch.Tensor,
    *,
    slot_indices: list[int],
    temperature: float,
    ece_bins: int,
    coverages: tuple[float, ...],
) -> dict[str, Any]:
    if targets.numel() == 0:
        return {
            "slot_indices": slot_indices,
            "visible_count": 0,
            "available": False,
            "temperature": temperature,
            "nll": None,
            "ece": None,
            "accuracy": None,
            "mean_confidence": None,
            "calibrated_nll": None,
            "calibrated_ece": None,
            "calibrated": None,
            "selective_curve": [],
        }
    evaluated = _evaluation(logits, targets, temperature, ece_bins)
    return {
        "slot_indices": slot_indices,
        "visible_count": int(targets.numel()),
        "available": True,
        "temperature": temperature,
        "nll": evaluated["nll"],
        "ece": evaluated["ece"],
        "accuracy": evaluated["accuracy"],
        "mean_confidence": evaluated["mean_confidence"],
        "calibrated_nll": evaluated["nll"],
        "calibrated_ece": evaluated["ece"],
        "calibrated": evaluated,
        "selective_curve": _selective_curve(
            logits,
            targets,
            temperature=temperature,
            coverages=coverages,
        ),
    }


def compute_segment_calibration_report(
    segment_logits: torch.Tensor,
    segment_targets: torch.Tensor,
    visibility_mask: torch.Tensor,
    *,
    coverages: Sequence[float] = DEFAULT_COVERAGES,
    ece_bins: int = DEFAULT_ECE_BINS,
    min_temperature: float = DEFAULT_MIN_TEMPERATURE,
    max_temperature: float = DEFAULT_MAX_TEMPERATURE,
    grid_size: int = DEFAULT_GRID_SIZE,
    refinement_steps: int = DEFAULT_REFINEMENT_STEPS,
    data_role: str = "validation",
) -> dict[str, Any]:
    """Fit temperature and return calibration/selective validation metrics."""

    role = _validated_data_role(data_role)
    coverage_values = _validated_coverages(coverages)
    bin_count = _validated_ece_bins(ece_bins)
    lower, upper, size, refinements = _validated_temperature_search(
        min_temperature, max_temperature, grid_size, refinement_steps
    )
    logits, targets, slots = _validated_inputs(
        segment_logits, segment_targets, visibility_mask
    )
    temperature, fitted_nll, fit_evaluations = _fit_flat_temperature(
        logits,
        targets,
        min_temperature=lower,
        max_temperature=upper,
        grid_size=size,
        refinement_steps=refinements,
    )
    uncalibrated = _evaluation(logits, targets, 1.0, bin_count)
    calibrated = _evaluation(logits, targets, temperature, bin_count)

    left = slots < LEFT_SLOT_COUNT
    right = ~left
    per_side = {
        "left": _side_report(
            logits[left],
            targets[left],
            slot_indices=list(range(LEFT_SLOT_COUNT)),
            temperature=temperature,
            ece_bins=bin_count,
            coverages=coverage_values,
        ),
        "right": _side_report(
            logits[right],
            targets[right],
            slot_indices=list(range(LEFT_SLOT_COUNT, SHUTTLE_COUNT)),
            temperature=temperature,
            ece_bins=bin_count,
            coverages=coverage_values,
        ),
    }
    return {
        "schema_version": REPORT_SCHEMA_VERSION,
        "data_role": role,
        "fit_scope": "validation_only",
        "visible_count": int(targets.numel()),
        "temperature": temperature,
        "temperature_bounds": {
            "minimum": lower,
            "maximum": upper,
        },
        "uncalibrated_nll": uncalibrated["nll"],
        "calibrated_nll": calibrated["nll"],
        "nll_improvement": uncalibrated["nll"] - calibrated["nll"],
        "uncalibrated_ece": uncalibrated["ece"],
        "calibrated_ece": calibrated["ece"],
        "ece_improvement": uncalibrated["ece"] - calibrated["ece"],
        "uncalibrated": uncalibrated,
        "calibrated": calibrated,
        "coverage_targets": list(coverage_values),
        "selective_curve": _selective_curve(
            logits,
            targets,
            temperature=temperature,
            coverages=coverage_values,
        ),
        "per_side": per_side,
        "ece_bins": bin_count,
        "fit": {
            "method": "bounded_log_grid_refinement",
            "grid_size": size,
            "refinement_steps": refinements,
            "objective": "visible_validation_segment_nll",
            "objective_nll": fitted_nll,
            "candidate_evaluations": fit_evaluations,
        },
    }


def evaluate_segment_calibration_at_temperature(
    segment_logits: torch.Tensor,
    segment_targets: torch.Tensor,
    visibility_mask: torch.Tensor,
    *,
    temperature: float,
    coverages: Sequence[float] = DEFAULT_COVERAGES,
    ece_bins: int = DEFAULT_ECE_BINS,
    data_role: str = "canary",
    source_temperature_role: str = "validation",
) -> dict[str, Any]:
    """Evaluate validation/canary logits using a saved validation temperature.

    This function never fits or changes ``temperature``.  It is the safe path
    for post-selection canary evaluation using the scalar stored by a completed
    validation-only calibration run.
    """

    role = _validated_evaluation_role(data_role)
    source_role = _validated_temperature_source_role(source_temperature_role)
    applied_temperature = _validated_temperature(temperature)
    coverage_values = _validated_coverages(coverages)
    bin_count = _validated_ece_bins(ece_bins)
    logits, targets, slots = _validated_inputs(
        segment_logits, segment_targets, visibility_mask
    )
    evaluated = _evaluation(
        logits, targets, applied_temperature, bin_count
    )
    left = slots < LEFT_SLOT_COUNT
    right = ~left
    per_side = {
        "left": _temperature_evaluation_side_report(
            logits[left],
            targets[left],
            slot_indices=list(range(LEFT_SLOT_COUNT)),
            temperature=applied_temperature,
            ece_bins=bin_count,
            coverages=coverage_values,
        ),
        "right": _temperature_evaluation_side_report(
            logits[right],
            targets[right],
            slot_indices=list(range(LEFT_SLOT_COUNT, SHUTTLE_COUNT)),
            temperature=applied_temperature,
            ece_bins=bin_count,
            coverages=coverage_values,
        ),
    }
    return {
        "schema_version": EVALUATION_REPORT_SCHEMA_VERSION,
        "data_role": role,
        "source_temperature_role": source_role,
        "fit_performed": False,
        "temperature": applied_temperature,
        "visible_count": int(targets.numel()),
        "nll": evaluated["nll"],
        "ece": evaluated["ece"],
        "accuracy": evaluated["accuracy"],
        "mean_confidence": evaluated["mean_confidence"],
        "calibrated_nll": evaluated["nll"],
        "calibrated_ece": evaluated["ece"],
        "calibrated": evaluated,
        "coverage_targets": list(coverage_values),
        "selective_curve": _selective_curve(
            logits,
            targets,
            temperature=applied_temperature,
            coverages=coverage_values,
        ),
        "per_side": per_side,
        "ece_bins": bin_count,
    }


# Concise aliases for callers that already establish the segment context.
fit_temperature = fit_segment_temperature
selective_accuracy_curve = segment_selective_accuracy_curve
build_calibration_report = compute_segment_calibration_report
evaluate_at_temperature = evaluate_segment_calibration_at_temperature


__all__ = [
    "DEFAULT_COVERAGES",
    "DEFAULT_ECE_BINS",
    "DEFAULT_GRID_SIZE",
    "DEFAULT_MAX_TEMPERATURE",
    "DEFAULT_MIN_TEMPERATURE",
    "DEFAULT_REFINEMENT_STEPS",
    "EVALUATION_REPORT_SCHEMA_VERSION",
    "REPORT_SCHEMA_VERSION",
    "build_calibration_report",
    "compute_segment_calibration_report",
    "evaluate_at_temperature",
    "evaluate_segment_calibration_at_temperature",
    "fit_segment_temperature",
    "fit_temperature",
    "segment_expected_calibration_error",
    "segment_negative_log_likelihood",
    "segment_selective_accuracy_curve",
    "selective_accuracy_curve",
]
