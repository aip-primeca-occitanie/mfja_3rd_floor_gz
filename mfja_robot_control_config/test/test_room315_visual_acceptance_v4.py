#!/usr/bin/env python3
"""Tests for the pure, fail-closed Room 315 V4 acceptance API."""

from __future__ import annotations

import copy
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / "mfja_robot_control_config" / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_acceptance_v4 as acceptance  # noqa: E402
from room_315_visual_contract_v4 import SEGMENT_CLASSES, SIDES  # noqa: E402


def _per_class(recall=0.82, support=20):
    return [
        {
            "class_name": segment,
            "support": support,
            "recall": recall,
            "f1": recall,
        }
        for segment in SEGMENT_CLASSES
    ]


def _subset(*, top1=0.84, joint=0.78, count=80, coverage=0.84, mae=0.06):
    return {
        "visible_count": 100,
        "segment_top1_accuracy": top1,
        "joint_localization_accuracy_s_ratio_0_05": joint,
        "correct_segment_s_m_mae_m": mae,
        "correct_segment_s_m_count": count,
        "correct_segment_s_m_coverage": coverage,
    }


def _validation_metrics():
    position = {
        name: _subset(top1=0.82, joint=0.72)
        for name in acceptance.POSITION_BINS
    }
    occlusion = {
        name: _subset(top1=0.83, joint=0.71)
        for name in acceptance.SCENE_OCCLUSION_CLASSES
    }
    presence = {
        name: _subset(top1=0.81, joint=0.70)
        for name in acceptance.SCENE_PRESENCE_DENSITIES
    }
    return {
        "segment_per_class": _per_class(recall=0.84, support=40),
        "per_side": {
            side: {
                "top1_accuracy": 0.86,
                "macro_recall": 0.84,
                "per_class": _per_class(),
            }
            for side in SIDES
        },
        "correct_segment_s_m_mae_m": 0.055,
        "correct_segment_s_m_count": 170,
        "correct_segment_s_m_coverage": 0.85,
        "joint_localization_accuracy_s_ratio_0_05": 0.79,
        "loaded_accuracy": 0.97,
        "breakdowns": {
            "side": {side: _subset() for side in SIDES},
            "zone_by_identity": {
                "switch": _subset(top1=0.82),
                "boundary": _subset(top1=0.81),
            },
            "target_zone": {
                "switch": _subset(top1=0.80),
                "boundary": _subset(top1=0.79),
            },
            "position_bin": position,
            "scene_occlusion_class": occlusion,
            "scene_presence_density": presence,
        },
    }


def _gates():
    return {
        "minimum_segment_top1_each_side": 0.75,
        "minimum_segment_macro_recall_each_side": 0.75,
        "zero_supported_segment_classes_with_zero_recall": True,
        "zero_supported_side_x_segment_cells_with_zero_recall": True,
        "minimum_worst_side_x_segment_recall": 0.50,
        "maximum_correct_segment_s_m_mae_m": 0.12,
        "maximum_correct_segment_s_m_mae_m_each_side": 0.12,
        "minimum_correct_segment_s_m_coverage_each_side": 0.75,
        "minimum_joint_segment_and_ratio_005_accuracy": 0.70,
        "minimum_joint_segment_and_ratio_005_accuracy_each_side": 0.70,
        "minimum_identity_zone_switch_segment_top1": 0.75,
        "minimum_identity_zone_boundary_segment_top1": 0.75,
        "minimum_position_bin_segment_top1": 0.70,
        "minimum_position_bin_joint_segment_and_ratio_005_accuracy": 0.60,
        "minimum_scene_occlusion_segment_top1": 0.70,
        "minimum_scene_occlusion_joint_segment_and_ratio_005_accuracy": 0.60,
        "minimum_scene_presence_density_segment_top1": 0.70,
        "minimum_scene_presence_density_joint_segment_and_ratio_005_accuracy": 0.60,
        "minimum_loaded_accuracy": 0.95,
        "maximum_loaded_accuracy_drop": 0.01,
        "cross_side_output_change_tolerance": 1.0e-6,
        "minimum_camera_order_swap_segment_top1_drop": 0.30,
        "automatic_runtime_switch": False,
    }


def _counterfactual_report():
    return {
        "maximum_cross_side_output_change": 5.0e-7,
        "blank_opposite_camera": {
            "maximum_own_side_output_change": 0.40,
        },
        "shuffle_opposite_camera": {
            "maximum_own_side_output_change": 0.35,
        },
        "camera_order_swap": {
            "segment_top1_drop": 0.35,
        },
    }


def _evaluate(metrics=None, gates=None, counterfactual=None, baseline=0.975):
    return acceptance.evaluate_visual_acceptance_v4(
        _validation_metrics() if metrics is None else metrics,
        _gates() if gates is None else gates,
        counterfactual_report=(
            _counterfactual_report() if counterfactual is None else counterfactual
        ),
        approved_v3_loaded_accuracy=baseline,
    )


def test_complete_evidence_passes_every_gate_without_switching_runtime():
    report = _evaluate()

    assert report["status"] == "passed"
    assert report["accepted"] is True
    assert report["automatic_runtime_switch"] is False
    assert report["summary"]["failed"] == 0
    assert report["summary"]["pending"] == 0
    assert report["summary"]["passed"] == report["summary"]["gate_count"]
    assert all(item["status"] == "passed" for item in report["per_gate"].values())
    assert (
        report["per_gate"]["counterfactual.maximum_cross_side_output_change"][
            "metric_path"
        ]
        == "maximum_cross_side_output_change"
    )


def test_side_zero_recall_and_worst_cell_fail_with_exact_evidence():
    metrics = _validation_metrics()
    left_a12e = metrics["per_side"]["left"]["per_class"][0]
    left_a12e["recall"] = 0.0

    report = _evaluate(metrics=metrics)

    assert report["status"] == "failed"
    cell = report["per_gate"]["side_segment_nonzero_recall.left.A12E"]
    assert cell["status"] == "failed"
    assert cell["observed"] == 0.0
    worst = report["per_gate"]["worst_side_x_segment_recall"]
    assert worst["status"] == "failed"
    assert worst["evidence"]["side"] == "left"
    assert worst["evidence"]["segment"] == "A12E"


def test_missing_side_cell_support_is_pending_not_a_fake_zero_recall_failure():
    metrics = _validation_metrics()
    metrics["per_side"]["right"]["per_class"][3]["support"] = 0

    report = _evaluate(metrics=metrics)

    assert report["status"] == "pending"
    cell = report["per_gate"]["side_segment_nonzero_recall.right.A1E"]
    assert cell["status"] == "pending"
    assert cell["reason"] == "unsupported_cell"
    worst = report["per_gate"]["worst_side_x_segment_recall"]
    assert worst["status"] == "pending"
    assert "right.A1E" in worst["evidence"]["missing_or_invalid_cells"]


def test_side_physical_gate_requires_correct_segment_metric_and_positive_count():
    metrics = _validation_metrics()
    left = metrics["breakdowns"]["side"]["left"]
    left.pop("correct_segment_s_m_mae_m")
    left["s_m_mae_m"] = 0.0  # Invalid cross-segment alias must never substitute.

    report = _evaluate(metrics=metrics)

    gate = report["per_gate"]["correct_segment_s_m_mae.left"]
    assert gate["status"] == "pending"
    assert gate["reason"] == "missing_metric"
    assert report["accepted"] is False

    metrics = _validation_metrics()
    metrics["breakdowns"]["side"]["left"]["correct_segment_s_m_count"] = 0
    report = _evaluate(metrics=metrics)
    assert report["per_gate"]["correct_segment_s_m_mae.left"]["reason"] == (
        "insufficient_support"
    )
    assert report["per_gate"]["correct_segment_s_m_coverage.left"]["reason"] == (
        "insufficient_support"
    )


def test_missing_optional_runtime_evidence_is_pending_when_gate_is_configured():
    report = acceptance.evaluate_visual_acceptance_v4(
        _validation_metrics(),
        _gates(),
        counterfactual_report=None,
        approved_v3_loaded_accuracy=None,
    )

    assert report["status"] == "pending"
    assert report["accepted"] is False
    assert report["per_gate"]["loaded_accuracy.maximum_approved_v3_drop"] == {
        "status": "pending",
        "required": True,
        "reason": "missing_approved_v3_loaded_accuracy",
        "evidence": {
            "approved_v3_loaded_accuracy": None,
            "v4_loaded_accuracy": 0.97,
        },
        "metric_path": "approved_v3_loaded_accuracy - loaded_accuracy",
        "threshold_key": "maximum_loaded_accuracy_drop",
    }
    assert report["per_gate"]["counterfactual.maximum_cross_side_output_change"][
        "reason"
    ] == "missing_counterfactual_report"
    assert report["per_gate"]["counterfactual.camera_order_swap_segment_top1_drop"][
        "reason"
    ] == "missing_counterfactual_report"


def test_global_loaded_floor_and_approved_v3_drop_are_independent_failures():
    metrics = _validation_metrics()
    metrics["loaded_accuracy"] = 0.94

    report = _evaluate(metrics=metrics)

    assert report["status"] == "failed"
    assert report["per_gate"]["loaded_accuracy.minimum"]["status"] == "failed"
    drop = report["per_gate"]["loaded_accuracy.maximum_approved_v3_drop"]
    assert drop["status"] == "failed"
    assert abs(drop["observed"] - 0.035) < 1.0e-12


def test_zone_position_and_scene_thresholds_gate_each_supported_breakdown():
    metrics = _validation_metrics()
    metrics["breakdowns"]["zone_by_identity"]["boundary"][
        "segment_top1_accuracy"
    ] = 0.74
    metrics["breakdowns"]["position_bin"]["p05"][
        "joint_localization_accuracy_s_ratio_0_05"
    ] = 0.59
    metrics["breakdowns"]["scene_occlusion_class"]["partial_risk"][
        "segment_top1_accuracy"
    ] = 0.69

    report = _evaluate(metrics=metrics)

    assert report["per_gate"]["identity_zone.segment_top1.boundary"]["status"] == (
        "failed"
    )
    assert report["per_gate"]["position_bin.joint_005.p05"]["status"] == "failed"
    assert report["per_gate"][
        "scene_occlusion.segment_top1.partial_risk"
    ]["status"] == "failed"


def test_target_zone_gates_are_created_only_when_configured():
    gates = _gates()
    gates["minimum_target_zone_switch_segment_top1"] = 0.78
    gates["minimum_target_zone_boundary_segment_top1"] = 0.78

    report = _evaluate(gates=gates)

    assert report["per_gate"]["target_zone.segment_top1.switch"]["status"] == "passed"
    assert report["per_gate"]["target_zone.segment_top1.boundary"]["status"] == (
        "passed"
    )

    gates = _gates()
    report = _evaluate(gates=gates)
    assert not any(name.startswith("target_zone.") for name in report["per_gate"])


def test_scene_breakdown_legacy_aliases_remain_accepted():
    metrics = _validation_metrics()
    breakdowns = metrics["breakdowns"]
    breakdowns["occlusion_class"] = breakdowns.pop("scene_occlusion_class")
    breakdowns["presence_class"] = breakdowns.pop("scene_presence_density")

    report = _evaluate(metrics=metrics)

    assert report["status"] == "passed"
    gate = report["per_gate"]["scene_presence_density.segment_top1.dense"]
    assert gate["metric_path"].startswith("breakdowns.presence_class.dense")


def test_role_specific_presence_contract_does_not_invent_missing_canary_medium():
    metrics = _validation_metrics()
    metrics["breakdowns"]["scene_presence_density"].pop("medium")

    default_report = _evaluate(metrics=metrics)
    assert default_report["status"] == "pending"
    assert default_report["per_gate"][
        "scene_presence_density.segment_top1.medium"
    ]["reason"] == "missing_category"

    canary_report = acceptance.evaluate_visual_acceptance_v4(
        metrics,
        _gates(),
        counterfactual_report=_counterfactual_report(),
        approved_v3_loaded_accuracy=0.975,
        required_scene_presence_densities=("sparse", "dense"),
    )
    assert canary_report["status"] == "passed"
    assert canary_report["inputs"]["required_scene_presence_densities"] == [
        "sparse",
        "dense",
    ]
    assert not any(
        name.endswith(".medium") for name in canary_report["per_gate"]
    )


def test_role_specific_presence_contract_rejects_invalid_or_duplicate_categories():
    for categories in ((), ("sparse", "sparse"), ("unknown",)):
        with pytest.raises(ValueError, match="canonical densities"):
            acceptance.evaluate_visual_acceptance_v4(
                _validation_metrics(),
                _gates(),
                required_scene_presence_densities=categories,
            )


def test_camera_order_swap_drop_has_minimum_direction_and_missing_is_pending():
    counterfactual = _counterfactual_report()
    counterfactual["camera_order_swap"]["segment_top1_drop"] = 0.29
    report = _evaluate(counterfactual=counterfactual)
    gate = report["per_gate"]["counterfactual.camera_order_swap_segment_top1_drop"]
    assert gate["status"] == "failed"
    assert gate["comparison"] == ">="

    counterfactual = _counterfactual_report()
    counterfactual.pop("camera_order_swap")
    report = _evaluate(counterfactual=counterfactual)
    assert report["per_gate"][
        "counterfactual.camera_order_swap_segment_top1_drop"
    ]["status"] == "pending"


def test_invalid_nan_metric_is_pending_and_inputs_are_not_mutated():
    metrics = _validation_metrics()
    gates = _gates()
    counterfactual = _counterfactual_report()
    metrics["per_side"]["left"]["top1_accuracy"] = float("nan")
    before_metrics = copy.deepcopy(metrics)
    before_gates = copy.deepcopy(gates)
    before_counterfactual = copy.deepcopy(counterfactual)

    report = _evaluate(
        metrics=metrics,
        gates=gates,
        counterfactual=counterfactual,
    )

    gate = report["per_gate"]["side_segment_top1.left"]
    assert gate["status"] == "pending"
    assert gate["reason"] == "invalid_metric:not_finite"
    assert metrics == before_metrics
    assert gates == before_gates
    assert counterfactual == before_counterfactual


def test_missing_required_threshold_is_pending_not_implicitly_disabled():
    gates = _gates()
    gates.pop("minimum_worst_side_x_segment_recall")

    report = _evaluate(gates=gates)

    gate = report["per_gate"]["worst_side_x_segment_recall"]
    assert gate["status"] == "pending"
    assert gate["reason"] == "missing_threshold"
    assert report["accepted"] is False


def test_missing_top_level_required_inputs_returns_pending_report_not_exception():
    report = acceptance.evaluate_visual_acceptance_v4(None, None)

    assert report["status"] == "pending"
    assert report["accepted"] is False
    assert report["summary"]["failed"] == 0
    assert report["summary"]["pending"] > 0
    assert report["inputs"]["validation_metrics_mapping"] is False
    assert report["inputs"]["gates_config_mapping"] is False


def test_absent_optional_category_thresholds_do_not_require_category_breakdowns():
    metrics = _validation_metrics()
    gates = _gates()
    for name in tuple(gates):
        if any(
            marker in name
            for marker in ("position_bin", "scene_occlusion", "scene_presence_density")
        ):
            gates.pop(name)
    metrics["breakdowns"].pop("position_bin")
    metrics["breakdowns"].pop("scene_occlusion_class")
    metrics["breakdowns"].pop("scene_presence_density")

    report = _evaluate(metrics=metrics, gates=gates)

    assert report["status"] == "passed"
    assert not any(
        name.startswith(("position_bin.", "scene_occlusion.", "scene_presence_density."))
        for name in report["per_gate"]
    )
