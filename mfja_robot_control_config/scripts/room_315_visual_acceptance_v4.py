#!/usr/bin/env python3
"""Pure, fail-closed acceptance gates for Room 315 visual V4.

The evaluator consumes already-computed validation metrics.  It performs no
file I/O, model loading, data loading, or runtime switching.  A gate can pass
only when its configured threshold, metric, and required support evidence are
all present and finite.  Missing or malformed evidence is ``pending`` rather
than being silently treated as a pass.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

from room_315_visual_contract_v4 import SEGMENT_CLASSES, SIDES


ACCEPTANCE_SCHEMA_VERSION = "room315.visual_acceptance.v4.v1"
POSITION_BINS = ("p05", "p15", "p25", "p40", "p50", "p60", "p75", "p85", "p95")
SCENE_OCCLUSION_CLASSES = ("clear", "partial_risk")
SCENE_PRESENCE_DENSITIES = ("sparse", "medium", "dense")

_MISSING = object()


def _is_sequence(value: Any) -> bool:
    return isinstance(value, Sequence) and not isinstance(
        value, (str, bytes, bytearray)
    )


def _finite_number(
    value: Any,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
) -> tuple[float | None, str | None]:
    if isinstance(value, bool):
        return None, "boolean_is_not_numeric"
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None, "not_numeric"
    if not math.isfinite(parsed):
        return None, "not_finite"
    if minimum is not None and parsed < minimum:
        return None, f"below_minimum_{minimum}"
    if maximum is not None and parsed > maximum:
        return None, f"above_maximum_{maximum}"
    return parsed, None


def _path_text(path: Sequence[str]) -> str:
    return ".".join(path)


def _raw_at(root: Any, path: Sequence[str]) -> Any:
    current = root
    for name in path:
        if not isinstance(current, Mapping) or name not in current:
            return _MISSING
        current = current[name]
    return current


def _number_at(
    root: Any,
    paths: Sequence[Sequence[str]],
    *,
    minimum: float | None = None,
    maximum: float | None = None,
) -> tuple[float | None, str | None, str]:
    expected = " | ".join(_path_text(path) for path in paths)
    for path in paths:
        raw = _raw_at(root, path)
        if raw is _MISSING:
            continue
        value, issue = _finite_number(raw, minimum=minimum, maximum=maximum)
        if issue is not None:
            return None, f"invalid_metric:{issue}", _path_text(path)
        return value, None, _path_text(path)
    return None, "missing_metric", expected


def _positive_count_at(
    root: Any,
    paths: Sequence[Sequence[str]],
) -> tuple[int | None, str | None, str]:
    value, issue, path = _number_at(root, paths, minimum=0.0)
    if issue is not None:
        return None, issue, path
    assert value is not None
    if not value.is_integer():
        return None, "invalid_metric:count_is_not_integral", path
    if value <= 0.0:
        return None, "insufficient_support", path
    return int(value), None, path


def _threshold(
    gates: Mapping[str, Any],
    names: Sequence[str],
    *,
    minimum: float | None = None,
    maximum: float | None = None,
) -> tuple[float | None, str | None, str]:
    expected = " | ".join(names)
    for name in names:
        if name not in gates:
            continue
        value, issue = _finite_number(
            gates[name], minimum=minimum, maximum=maximum
        )
        if issue is not None:
            return None, f"invalid_threshold:{issue}", name
        return value, None, name
    return None, "missing_threshold", expected


def _comparison_passes(observed: float, threshold: float, comparison: str) -> bool:
    if comparison == ">=":
        return observed >= threshold
    if comparison == "<=":
        return observed <= threshold
    if comparison == ">":
        return observed > threshold
    if comparison == "==":
        return observed == threshold
    raise ValueError(f"unsupported comparison {comparison!r}")


class _GateCollector:
    def __init__(self) -> None:
        self.per_gate: dict[str, dict[str, Any]] = {}

    def add_pending(
        self,
        gate_id: str,
        *,
        reason: str,
        metric_path: str | None = None,
        threshold_key: str | None = None,
        evidence: Mapping[str, Any] | None = None,
    ) -> None:
        item: dict[str, Any] = {
            "status": "pending",
            "required": True,
            "reason": reason,
            "evidence": dict(evidence or {}),
        }
        if metric_path is not None:
            item["metric_path"] = metric_path
        if threshold_key is not None:
            item["threshold_key"] = threshold_key
        self._store(gate_id, item)

    def add_numeric(
        self,
        gate_id: str,
        *,
        observed: float | None,
        observed_issue: str | None,
        metric_path: str,
        threshold: float | None,
        threshold_issue: str | None,
        threshold_key: str,
        comparison: str,
        evidence: Mapping[str, Any] | None = None,
    ) -> None:
        details = dict(evidence or {})
        if threshold_issue is not None:
            self.add_pending(
                gate_id,
                reason=threshold_issue,
                metric_path=metric_path,
                threshold_key=threshold_key,
                evidence=details,
            )
            return
        if observed_issue is not None:
            self.add_pending(
                gate_id,
                reason=observed_issue,
                metric_path=metric_path,
                threshold_key=threshold_key,
                evidence=details,
            )
            return
        assert observed is not None and threshold is not None
        passed = _comparison_passes(observed, threshold, comparison)
        self._store(
            gate_id,
            {
                "status": "passed" if passed else "failed",
                "required": True,
                "comparison": comparison,
                "observed": observed,
                "threshold": threshold,
                "metric_path": metric_path,
                "threshold_key": threshold_key,
                "evidence": details,
            },
        )

    def _store(self, gate_id: str, item: dict[str, Any]) -> None:
        if gate_id in self.per_gate:
            raise ValueError(f"duplicate acceptance gate id {gate_id!r}")
        self.per_gate[gate_id] = item


def _class_entries(
    raw: Any,
) -> tuple[dict[str, dict[str, Any]], dict[str, str]]:
    """Parse canonical per-class evidence without accepting missing support."""

    if not _is_sequence(raw):
        return {}, {segment: "missing_per_class_sequence" for segment in SEGMENT_CLASSES}
    entries: dict[str, dict[str, Any]] = {}
    duplicate_names: set[str] = set()
    for item in raw:
        if not isinstance(item, Mapping):
            continue
        name = str(item.get("class_name") or "").strip().upper()
        if name not in SEGMENT_CLASSES:
            continue
        if name in entries:
            duplicate_names.add(name)
            continue
        support_value, support_issue = _finite_number(item.get("support"), minimum=0.0)
        recall, recall_issue = _finite_number(
            item.get("recall"), minimum=0.0, maximum=1.0
        )
        if support_issue is not None:
            entries[name] = {"issue": f"invalid_support:{support_issue}"}
            continue
        assert support_value is not None
        if not support_value.is_integer():
            entries[name] = {"issue": "invalid_support:not_integral"}
            continue
        if support_value <= 0.0:
            entries[name] = {"issue": "unsupported_cell", "support": int(support_value)}
            continue
        if recall_issue is not None:
            entries[name] = {
                "issue": f"invalid_recall:{recall_issue}",
                "support": int(support_value),
            }
            continue
        entries[name] = {
            "support": int(support_value),
            "recall": recall,
        }
    issues: dict[str, str] = {}
    for segment in SEGMENT_CLASSES:
        if segment in duplicate_names:
            issues[segment] = "duplicate_class_entry"
        elif segment not in entries:
            issues[segment] = "missing_class_entry"
        elif "issue" in entries[segment]:
            issues[segment] = str(entries[segment]["issue"])
    return entries, issues


def _side_segment_gates(
    collector: _GateCollector,
    metrics: Mapping[str, Any],
    gates: Mapping[str, Any],
) -> None:
    top1_threshold = _threshold(
        gates, ("minimum_segment_top1_each_side",), minimum=0.0, maximum=1.0
    )
    macro_threshold = _threshold(
        gates,
        (
            "minimum_segment_macro_recall_each_side",
            "minimum_segment_macro_recall",
        ),
        minimum=0.0,
        maximum=1.0,
    )
    worst_threshold = _threshold(
        gates,
        ("minimum_worst_side_x_segment_recall",),
        minimum=0.0,
        maximum=1.0,
    )
    zero_policy = gates.get(
        "zero_supported_side_x_segment_cells_with_zero_recall", _MISSING
    )
    zero_policy_issue = None if zero_policy is True else (
        "missing_threshold"
        if zero_policy is _MISSING
        else "required_zero_recall_policy_not_enabled"
    )
    per_side = metrics.get("per_side")
    per_side_mapping = per_side if isinstance(per_side, Mapping) else {}
    parsed_cells: dict[tuple[str, str], dict[str, Any]] = {}
    cell_issues: dict[tuple[str, str], str] = {}

    for side in SIDES:
        side_metrics = per_side_mapping.get(side)
        side_mapping = side_metrics if isinstance(side_metrics, Mapping) else {}
        observed, issue, metric_path = _number_at(
            side_mapping,
            (("top1_accuracy",),),
            minimum=0.0,
            maximum=1.0,
        )
        collector.add_numeric(
            f"side_segment_top1.{side}",
            observed=observed,
            observed_issue=("missing_side_metrics" if not side_mapping else issue),
            metric_path=f"per_side.{side}.{metric_path}",
            threshold=top1_threshold[0],
            threshold_issue=top1_threshold[1],
            threshold_key=top1_threshold[2],
            comparison=">=",
            evidence={"side": side},
        )
        observed, issue, metric_path = _number_at(
            side_mapping,
            (("macro_recall",),),
            minimum=0.0,
            maximum=1.0,
        )
        collector.add_numeric(
            f"side_segment_macro_recall.{side}",
            observed=observed,
            observed_issue=("missing_side_metrics" if not side_mapping else issue),
            metric_path=f"per_side.{side}.{metric_path}",
            threshold=macro_threshold[0],
            threshold_issue=macro_threshold[1],
            threshold_key=macro_threshold[2],
            comparison=">=",
            evidence={"side": side},
        )

        entries, issues = _class_entries(side_mapping.get("per_class"))
        for segment in SEGMENT_CLASSES:
            cell = entries.get(segment, {})
            cell_issue = issues.get(segment)
            if cell_issue is not None:
                cell_issues[(side, segment)] = cell_issue
            else:
                parsed_cells[(side, segment)] = cell
            gate_id = f"side_segment_nonzero_recall.{side}.{segment}"
            evidence = {
                "side": side,
                "segment": segment,
                **({"support": cell.get("support")} if cell else {}),
            }
            if zero_policy_issue is not None:
                collector.add_pending(
                    gate_id,
                    reason=zero_policy_issue,
                    metric_path=f"per_side.{side}.per_class[{segment}].recall",
                    threshold_key=(
                        "zero_supported_side_x_segment_cells_with_zero_recall"
                    ),
                    evidence=evidence,
                )
            elif cell_issue is not None:
                collector.add_pending(
                    gate_id,
                    reason=cell_issue,
                    metric_path=f"per_side.{side}.per_class[{segment}].recall",
                    threshold_key=(
                        "zero_supported_side_x_segment_cells_with_zero_recall"
                    ),
                    evidence=evidence,
                )
            else:
                recall = float(cell["recall"])
                collector.add_numeric(
                    gate_id,
                    observed=recall,
                    observed_issue=None,
                    metric_path=f"per_side.{side}.per_class[{segment}].recall",
                    threshold=0.0,
                    threshold_issue=None,
                    threshold_key=(
                        "zero_supported_side_x_segment_cells_with_zero_recall"
                    ),
                    comparison=">",
                    evidence=evidence,
                )

    missing_cells = [
        f"{side}.{segment}"
        for side in SIDES
        for segment in SEGMENT_CLASSES
        if (side, segment) in cell_issues
    ]
    if missing_cells:
        collector.add_numeric(
            "worst_side_x_segment_recall",
            observed=None,
            observed_issue="incomplete_side_x_segment_support",
            metric_path="per_side.<side>.per_class[*].recall",
            threshold=worst_threshold[0],
            threshold_issue=worst_threshold[1],
            threshold_key=worst_threshold[2],
            comparison=">=",
            evidence={
                "missing_or_invalid_cells": missing_cells,
                "cell_issues": {
                    f"{side}.{segment}": issue
                    for (side, segment), issue in sorted(cell_issues.items())
                },
            },
        )
    else:
        worst_key = min(
            parsed_cells,
            key=lambda key: (float(parsed_cells[key]["recall"]), key[0], key[1]),
        )
        worst = parsed_cells[worst_key]
        collector.add_numeric(
            "worst_side_x_segment_recall",
            observed=float(worst["recall"]),
            observed_issue=None,
            metric_path=(
                f"per_side.{worst_key[0]}.per_class[{worst_key[1]}].recall"
            ),
            threshold=worst_threshold[0],
            threshold_issue=worst_threshold[1],
            threshold_key=worst_threshold[2],
            comparison=">=",
            evidence={
                "side": worst_key[0],
                "segment": worst_key[1],
                "support": worst["support"],
                "evaluated_cell_count": len(parsed_cells),
            },
        )


def _global_zero_recall_gate(
    collector: _GateCollector,
    metrics: Mapping[str, Any],
    gates: Mapping[str, Any],
) -> None:
    if "zero_supported_segment_classes_with_zero_recall" not in gates:
        return
    if gates.get("zero_supported_segment_classes_with_zero_recall") is not True:
        collector.add_pending(
            "global_supported_segment_zero_recall_count",
            reason="required_zero_recall_policy_not_enabled",
            metric_path="segment_per_class[*].recall",
            threshold_key="zero_supported_segment_classes_with_zero_recall",
        )
        return
    entries, issues = _class_entries(metrics.get("segment_per_class"))
    if issues:
        collector.add_pending(
            "global_supported_segment_zero_recall_count",
            reason="incomplete_segment_support",
            metric_path="segment_per_class[*].recall",
            threshold_key="zero_supported_segment_classes_with_zero_recall",
            evidence={"class_issues": dict(sorted(issues.items()))},
        )
        return
    zero_recall = [
        segment for segment in SEGMENT_CLASSES if float(entries[segment]["recall"]) == 0.0
    ]
    collector.add_numeric(
        "global_supported_segment_zero_recall_count",
        observed=float(len(zero_recall)),
        observed_issue=None,
        metric_path="segment_per_class[*].recall",
        threshold=0.0,
        threshold_issue=None,
        threshold_key="zero_supported_segment_classes_with_zero_recall",
        comparison="==",
        evidence={"zero_recall_classes": zero_recall},
    )


def _physical_and_loaded_gates(
    collector: _GateCollector,
    metrics: Mapping[str, Any],
    gates: Mapping[str, Any],
    approved_v3_loaded_accuracy: Any,
) -> None:
    breakdowns = metrics.get("breakdowns")
    breakdown_mapping = breakdowns if isinstance(breakdowns, Mapping) else {}
    side_breakdown = breakdown_mapping.get("side")
    side_mapping = side_breakdown if isinstance(side_breakdown, Mapping) else {}

    side_mae_threshold = _threshold(
        gates,
        ("maximum_correct_segment_s_m_mae_m_each_side",),
        minimum=0.0,
    )
    side_coverage_threshold = _threshold(
        gates,
        ("minimum_correct_segment_s_m_coverage_each_side",),
        minimum=0.0,
        maximum=1.0,
    )
    side_joint_threshold = _threshold(
        gates,
        ("minimum_joint_segment_and_ratio_005_accuracy_each_side",),
        minimum=0.0,
        maximum=1.0,
    )
    for side in SIDES:
        side_item = side_mapping.get(side)
        item = side_item if isinstance(side_item, Mapping) else {}
        count, count_issue, count_path = _positive_count_at(
            item, (("correct_segment_s_m_count",),)
        )
        mae, mae_issue, mae_path = _number_at(
            item,
            (("correct_segment_s_m_mae_m",),),
            minimum=0.0,
        )
        support_issue = (
            "missing_side_breakdown" if not item else count_issue
        )
        collector.add_numeric(
            f"correct_segment_s_m_mae.{side}",
            observed=mae,
            observed_issue=support_issue or mae_issue,
            metric_path=f"breakdowns.side.{side}.{mae_path}",
            threshold=side_mae_threshold[0],
            threshold_issue=side_mae_threshold[1],
            threshold_key=side_mae_threshold[2],
            comparison="<=",
            evidence={
                "side": side,
                "correct_segment_count": count,
                "support_metric_path": f"breakdowns.side.{side}.{count_path}",
            },
        )
        coverage, coverage_issue, coverage_path = _number_at(
            item,
            (("correct_segment_s_m_coverage",),),
            minimum=0.0,
            maximum=1.0,
        )
        collector.add_numeric(
            f"correct_segment_s_m_coverage.{side}",
            observed=coverage,
            observed_issue=(
                "missing_side_breakdown"
                if not item
                else count_issue or coverage_issue
            ),
            metric_path=f"breakdowns.side.{side}.{coverage_path}",
            threshold=side_coverage_threshold[0],
            threshold_issue=side_coverage_threshold[1],
            threshold_key=side_coverage_threshold[2],
            comparison=">=",
            evidence={"side": side, "correct_segment_count": count},
        )
        joint, joint_issue, joint_path = _number_at(
            item,
            (("joint_localization_accuracy_s_ratio_0_05",),),
            minimum=0.0,
            maximum=1.0,
        )
        collector.add_numeric(
            f"joint_segment_and_ratio_005.{side}",
            observed=joint,
            observed_issue=("missing_side_breakdown" if not item else joint_issue),
            metric_path=f"breakdowns.side.{side}.{joint_path}",
            threshold=side_joint_threshold[0],
            threshold_issue=side_joint_threshold[1],
            threshold_key=side_joint_threshold[2],
            comparison=">=",
            evidence={"side": side},
        )

    global_mae_threshold = _threshold(
        gates,
        ("maximum_correct_segment_s_m_mae_m",),
        minimum=0.0,
    )
    global_count, global_count_issue, global_count_path = _positive_count_at(
        metrics, (("correct_segment_s_m_count",),)
    )
    global_mae, global_mae_issue, global_mae_path = _number_at(
        metrics,
        (("correct_segment_s_m_mae_m",),),
        minimum=0.0,
    )
    collector.add_numeric(
        "correct_segment_s_m_mae.global",
        observed=global_mae,
        observed_issue=global_count_issue or global_mae_issue,
        metric_path=global_mae_path,
        threshold=global_mae_threshold[0],
        threshold_issue=global_mae_threshold[1],
        threshold_key=global_mae_threshold[2],
        comparison="<=",
        evidence={
            "correct_segment_count": global_count,
            "support_metric_path": global_count_path,
        },
    )

    global_joint_threshold = _threshold(
        gates,
        ("minimum_joint_segment_and_ratio_005_accuracy",),
        minimum=0.0,
        maximum=1.0,
    )
    global_joint, global_joint_issue, global_joint_path = _number_at(
        metrics,
        (("joint_localization_accuracy_s_ratio_0_05",),),
        minimum=0.0,
        maximum=1.0,
    )
    collector.add_numeric(
        "joint_segment_and_ratio_005.global",
        observed=global_joint,
        observed_issue=global_joint_issue,
        metric_path=global_joint_path,
        threshold=global_joint_threshold[0],
        threshold_issue=global_joint_threshold[1],
        threshold_key=global_joint_threshold[2],
        comparison=">=",
    )

    loaded_threshold = _threshold(
        gates,
        ("minimum_loaded_accuracy",),
        minimum=0.0,
        maximum=1.0,
    )
    loaded, loaded_issue, loaded_path = _number_at(
        metrics,
        (("loaded_accuracy",),),
        minimum=0.0,
        maximum=1.0,
    )
    collector.add_numeric(
        "loaded_accuracy.minimum",
        observed=loaded,
        observed_issue=loaded_issue,
        metric_path=loaded_path,
        threshold=loaded_threshold[0],
        threshold_issue=loaded_threshold[1],
        threshold_key=loaded_threshold[2],
        comparison=">=",
    )

    if "maximum_loaded_accuracy_drop" in gates:
        drop_threshold = _threshold(
            gates,
            ("maximum_loaded_accuracy_drop",),
            minimum=0.0,
            maximum=1.0,
        )
        baseline, baseline_issue = _finite_number(
            approved_v3_loaded_accuracy, minimum=0.0, maximum=1.0
        )
        if approved_v3_loaded_accuracy is None:
            baseline_issue = "missing_approved_v3_loaded_accuracy"
        drop = None
        drop_issue = loaded_issue or baseline_issue
        if drop_issue is None:
            assert baseline is not None and loaded is not None
            drop = baseline - loaded
        collector.add_numeric(
            "loaded_accuracy.maximum_approved_v3_drop",
            observed=drop,
            observed_issue=drop_issue,
            metric_path="approved_v3_loaded_accuracy - loaded_accuracy",
            threshold=drop_threshold[0],
            threshold_issue=drop_threshold[1],
            threshold_key=drop_threshold[2],
            comparison="<=",
            evidence={
                "approved_v3_loaded_accuracy": baseline,
                "v4_loaded_accuracy": loaded,
            },
        )


def _mapping_at_first(
    root: Any,
    paths: Sequence[Sequence[str]],
) -> tuple[Mapping[str, Any] | None, str]:
    expected = " | ".join(_path_text(path) for path in paths)
    for path in paths:
        raw = _raw_at(root, path)
        if raw is _MISSING:
            continue
        if isinstance(raw, Mapping):
            return raw, _path_text(path)
        return None, _path_text(path)
    return None, expected


def _category_metric_gates(
    collector: _GateCollector,
    metrics: Mapping[str, Any],
    gates: Mapping[str, Any],
    *,
    group_name: str,
    breakdown_paths: Sequence[Sequence[str]],
    expected_categories: Sequence[str],
    threshold_specs: Sequence[tuple[str, str, Sequence[str]]],
) -> None:
    configured_specs = [
        (metric_label, metric_field, threshold_names)
        for metric_label, metric_field, threshold_names in threshold_specs
        if any(name in gates for name in threshold_names)
    ]
    if not configured_specs:
        return
    breakdown, breakdown_path = _mapping_at_first(metrics, breakdown_paths)
    actual_categories = [] if breakdown is None else [str(name) for name in breakdown]
    categories = list(dict.fromkeys([*expected_categories, *sorted(actual_categories)]))
    for metric_label, metric_field, threshold_names in configured_specs:
        threshold = _threshold(
            gates, threshold_names, minimum=0.0, maximum=1.0
        )
        for category in categories:
            raw_item = None if breakdown is None else breakdown.get(category)
            item = raw_item if isinstance(raw_item, Mapping) else {}
            count, count_issue, count_path = _positive_count_at(
                item, (("visible_count",),)
            )
            observed, observed_issue, observed_path = _number_at(
                item,
                ((metric_field,),),
                minimum=0.0,
                maximum=1.0,
            )
            if breakdown is None:
                issue = "missing_breakdown"
            elif not item:
                issue = "missing_category"
            else:
                issue = count_issue or observed_issue
            collector.add_numeric(
                f"{group_name}.{metric_label}.{category}",
                observed=observed,
                observed_issue=issue,
                metric_path=f"{breakdown_path}.{category}.{observed_path}",
                threshold=threshold[0],
                threshold_issue=threshold[1],
                threshold_key=threshold[2],
                comparison=">=",
                evidence={
                    "category": category,
                    "visible_count": count,
                    "support_metric_path": (
                        f"{breakdown_path}.{category}.{count_path}"
                    ),
                },
            )


def _breakdown_gates(
    collector: _GateCollector,
    metrics: Mapping[str, Any],
    gates: Mapping[str, Any],
    *,
    required_scene_presence_densities: Sequence[str],
) -> None:
    zone_specs = (
        (
            "identity_zone",
            ("breakdowns", "zone_by_identity"),
            "switch",
            "minimum_identity_zone_switch_segment_top1",
        ),
        (
            "identity_zone",
            ("breakdowns", "zone_by_identity"),
            "boundary",
            "minimum_identity_zone_boundary_segment_top1",
        ),
        (
            "target_zone",
            ("breakdowns", "target_zone"),
            "switch",
            "minimum_target_zone_switch_segment_top1",
        ),
        (
            "target_zone",
            ("breakdowns", "target_zone"),
            "boundary",
            "minimum_target_zone_boundary_segment_top1",
        ),
    )
    for label, path, zone, threshold_name in zone_specs:
        if threshold_name not in gates:
            continue
        threshold = _threshold(
            gates, (threshold_name,), minimum=0.0, maximum=1.0
        )
        alternate_paths = [path]
        if label == "identity_zone":
            alternate_paths.append(("breakdowns", "identity_zone"))
        breakdown, breakdown_path = _mapping_at_first(metrics, alternate_paths)
        raw_item = None if breakdown is None else breakdown.get(zone)
        item = raw_item if isinstance(raw_item, Mapping) else {}
        count, count_issue, count_path = _positive_count_at(
            item, (("visible_count",),)
        )
        observed, observed_issue, observed_path = _number_at(
            item,
            (("segment_top1_accuracy",),),
            minimum=0.0,
            maximum=1.0,
        )
        if breakdown is None:
            issue = "missing_breakdown"
        elif not item:
            issue = "missing_zone"
        else:
            issue = count_issue or observed_issue
        collector.add_numeric(
            f"{label}.segment_top1.{zone}",
            observed=observed,
            observed_issue=issue,
            metric_path=f"{breakdown_path}.{zone}.{observed_path}",
            threshold=threshold[0],
            threshold_issue=threshold[1],
            threshold_key=threshold[2],
            comparison=">=",
            evidence={
                "zone": zone,
                "visible_count": count,
                "support_metric_path": f"{breakdown_path}.{zone}.{count_path}",
            },
        )

    _category_metric_gates(
        collector,
        metrics,
        gates,
        group_name="position_bin",
        breakdown_paths=(("breakdowns", "position_bin"),),
        expected_categories=POSITION_BINS,
        threshold_specs=(
            (
                "segment_top1",
                "segment_top1_accuracy",
                ("minimum_position_bin_segment_top1",),
            ),
            (
                "joint_005",
                "joint_localization_accuracy_s_ratio_0_05",
                ("minimum_position_bin_joint_segment_and_ratio_005_accuracy",),
            ),
        ),
    )
    _category_metric_gates(
        collector,
        metrics,
        gates,
        group_name="scene_occlusion",
        breakdown_paths=(
            ("breakdowns", "scene_occlusion_class"),
            ("breakdowns", "occlusion_class"),
        ),
        expected_categories=SCENE_OCCLUSION_CLASSES,
        threshold_specs=(
            (
                "segment_top1",
                "segment_top1_accuracy",
                ("minimum_scene_occlusion_segment_top1",),
            ),
            (
                "joint_005",
                "joint_localization_accuracy_s_ratio_0_05",
                ("minimum_scene_occlusion_joint_segment_and_ratio_005_accuracy",),
            ),
        ),
    )
    _category_metric_gates(
        collector,
        metrics,
        gates,
        group_name="scene_presence_density",
        breakdown_paths=(
            ("breakdowns", "scene_presence_density"),
            ("breakdowns", "presence_class"),
        ),
        expected_categories=required_scene_presence_densities,
        threshold_specs=(
            (
                "segment_top1",
                "segment_top1_accuracy",
                ("minimum_scene_presence_density_segment_top1",),
            ),
            (
                "joint_005",
                "joint_localization_accuracy_s_ratio_0_05",
                (
                    "minimum_scene_presence_density_joint_segment_and_ratio_005_accuracy",
                ),
            ),
        ),
    )


def _counterfactual_gate(
    collector: _GateCollector,
    gates: Mapping[str, Any],
    counterfactual_report: Any,
) -> None:
    cross_side_configured = "cross_side_output_change_tolerance" in gates
    swap_configured = "minimum_camera_order_swap_segment_top1_drop" in gates
    if not cross_side_configured and not swap_configured:
        return
    report = counterfactual_report if isinstance(counterfactual_report, Mapping) else {}
    evidence: dict[str, Any] = {}
    for name in ("blank_opposite_camera", "shuffle_opposite_camera"):
        value, value_issue, path = _number_at(
            report,
            ((name, "maximum_own_side_output_change"),),
            minimum=0.0,
        )
        if value_issue is None:
            evidence[name] = {
                "maximum_own_side_output_change": value,
                "metric_path": path,
            }
    evidence["camera_order_swap_diagnostics_present"] = isinstance(
        report.get("camera_order_swap"), Mapping
    ) or isinstance(report.get("camera_order_swap_canary"), Mapping)
    report_issue = None
    if counterfactual_report is None:
        report_issue = "missing_counterfactual_report"
    elif not isinstance(counterfactual_report, Mapping):
        report_issue = "invalid_counterfactual_report"

    if cross_side_configured:
        threshold = _threshold(
            gates,
            ("cross_side_output_change_tolerance",),
            minimum=0.0,
        )
        observed, issue, metric_path = _number_at(
            report,
            (
                ("maximum_cross_side_output_change",),
                ("cross_side_max_output_change",),
                ("max_cross_side_output_change",),
                ("summary", "maximum_cross_side_output_change"),
            ),
            minimum=0.0,
        )
        collector.add_numeric(
            "counterfactual.maximum_cross_side_output_change",
            observed=observed,
            observed_issue=report_issue or issue,
            metric_path=metric_path,
            threshold=threshold[0],
            threshold_issue=threshold[1],
            threshold_key=threshold[2],
            comparison="<=",
            evidence=evidence,
        )

    if swap_configured:
        threshold = _threshold(
            gates,
            ("minimum_camera_order_swap_segment_top1_drop",),
            minimum=0.0,
            maximum=1.0,
        )
        observed, issue, metric_path = _number_at(
            report,
            (
                ("camera_order_swap", "segment_top1_drop"),
                ("camera_order_swap_canary", "segment_top1_drop"),
            ),
            minimum=0.0,
            maximum=1.0,
        )
        collector.add_numeric(
            "counterfactual.camera_order_swap_segment_top1_drop",
            observed=observed,
            observed_issue=report_issue or issue,
            metric_path=metric_path,
            threshold=threshold[0],
            threshold_issue=threshold[1],
            threshold_key=threshold[2],
            comparison=">=",
            evidence={
                "interpretation": (
                    "camera-order swap must cause at least the configured "
                    "segment top1 degradation"
                )
            },
        )


def evaluate_visual_acceptance_v4(
    validation_metrics: Mapping[str, Any] | None,
    gates_config: Mapping[str, Any] | None,
    *,
    counterfactual_report: Mapping[str, Any] | None = None,
    approved_v3_loaded_accuracy: float | None = None,
    required_scene_presence_densities: Sequence[str] | None = None,
) -> dict[str, Any]:
    """Evaluate V4 acceptance without side effects.

    ``failed`` takes precedence when at least one complete gate has a definite
    threshold violation.  Otherwise any missing/invalid required evidence
    yields ``pending``.  Only a complete set of passing gates returns
    ``passed``.
    """

    metrics = validation_metrics if isinstance(validation_metrics, Mapping) else {}
    gates = gates_config if isinstance(gates_config, Mapping) else {}
    if required_scene_presence_densities is None:
        presence_densities = SCENE_PRESENCE_DENSITIES
    else:
        if not _is_sequence(required_scene_presence_densities):
            raise ValueError("required scene presence densities must be a sequence")
        presence_densities = tuple(
            str(value).strip().casefold()
            for value in required_scene_presence_densities
        )
        if (
            not presence_densities
            or len(set(presence_densities)) != len(presence_densities)
            or any(value not in SCENE_PRESENCE_DENSITIES for value in presence_densities)
        ):
            raise ValueError(
                "required scene presence densities must be a non-empty unique "
                "subset of the canonical densities"
            )
    collector = _GateCollector()

    _side_segment_gates(collector, metrics, gates)
    _global_zero_recall_gate(collector, metrics, gates)
    _physical_and_loaded_gates(
        collector,
        metrics,
        gates,
        approved_v3_loaded_accuracy,
    )
    _breakdown_gates(
        collector,
        metrics,
        gates,
        required_scene_presence_densities=presence_densities,
    )
    _counterfactual_gate(collector, gates, counterfactual_report)

    counts = {
        status: sum(
            item["status"] == status for item in collector.per_gate.values()
        )
        for status in ("passed", "failed", "pending")
    }
    if counts["failed"]:
        status = "failed"
    elif counts["pending"] or not collector.per_gate:
        status = "pending"
    else:
        status = "passed"
    return {
        "schema_version": ACCEPTANCE_SCHEMA_VERSION,
        "status": status,
        "accepted": status == "passed",
        "automatic_runtime_switch": False,
        "summary": {
            "gate_count": len(collector.per_gate),
            **counts,
        },
        "inputs": {
            "validation_metrics_mapping": isinstance(validation_metrics, Mapping),
            "gates_config_mapping": isinstance(gates_config, Mapping),
            "counterfactual_report_provided": counterfactual_report is not None,
            "approved_v3_loaded_accuracy_provided": (
                approved_v3_loaded_accuracy is not None
            ),
            "required_scene_presence_densities": list(presence_densities),
        },
        "per_gate": collector.per_gate,
    }


def evaluate_acceptance_v4(
    validation_metrics: Mapping[str, Any] | None,
    gates_config: Mapping[str, Any] | None,
    *,
    counterfactual_report: Mapping[str, Any] | None = None,
    approved_v3_loaded_accuracy: float | None = None,
    required_scene_presence_densities: Sequence[str] | None = None,
) -> dict[str, Any]:
    """Short public alias for :func:`evaluate_visual_acceptance_v4`."""

    return evaluate_visual_acceptance_v4(
        validation_metrics,
        gates_config,
        counterfactual_report=counterfactual_report,
        approved_v3_loaded_accuracy=approved_v3_loaded_accuracy,
        required_scene_presence_densities=required_scene_presence_densities,
    )


__all__ = (
    "ACCEPTANCE_SCHEMA_VERSION",
    "POSITION_BINS",
    "SCENE_OCCLUSION_CLASSES",
    "SCENE_PRESENCE_DENSITIES",
    "evaluate_acceptance_v4",
    "evaluate_visual_acceptance_v4",
)
