#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path

import pytest


torch = pytest.importorskip("torch")

REPO_ROOT = Path(__file__).resolve().parents[2]
CALIBRATION_SCRIPT = (
    REPO_ROOT
    / "mfja_robot_control_config"
    / "scripts"
    / "room_315_visual_calibration_v4.py"
)


def _load_calibration(name="room315_visual_calibration_v4_test"):
    spec = importlib.util.spec_from_file_location(name, CALIBRATION_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _contract(batch=1):
    return (
        torch.zeros(batch, 8, 14, dtype=torch.float32),
        torch.full((batch, 8), -100, dtype=torch.long),
        torch.zeros(batch, 8, dtype=torch.bool),
    )


def test_overconfident_wrong_predictions_fit_temperature_above_one_and_improve_nll():
    calibration = _load_calibration("room315_calibration_overconfident")
    logits, targets, visibility = _contract()
    visibility[0, [0, 4]] = True
    targets[0, [0, 4]] = 1
    logits[0, [0, 4], 0] = 12.0

    report = calibration.compute_segment_calibration_report(
        logits,
        targets,
        visibility,
        min_temperature=0.25,
        max_temperature=8.0,
        grid_size=65,
        refinement_steps=3,
    )

    assert 1.0 < report["temperature"] <= 8.0
    assert report["calibrated_nll"] < report["uncalibrated_nll"]
    assert report["nll_improvement"] > 0.0
    assert report["fit"]["objective_nll"] == pytest.approx(
        report["calibrated_nll"]
    )
    assert report["data_role"] == "validation"


def test_mask_bounds_per_side_and_selective_counts_are_deterministic_and_finite():
    calibration = _load_calibration("room315_calibration_mask_side")
    logits, targets, visibility = _contract(batch=2)
    visible = [(0, 0, 2), (0, 4, 3), (1, 7, 5)]
    for batch, slot, target in visible:
        visibility[batch, slot] = True
        targets[batch, slot] = target
        logits[batch, slot, target] = 3.0
    logits[~visibility] = 999.0

    kwargs = {
        "coverages": (1.0, 0.5),
        "min_temperature": 0.5,
        "max_temperature": 3.0,
        "grid_size": 33,
        "refinement_steps": 2,
    }
    first = calibration.compute_segment_calibration_report(
        logits, targets, visibility, **kwargs
    )
    second = calibration.compute_segment_calibration_report(
        logits, targets, visibility, **kwargs
    )

    assert first["temperature"] == second["temperature"]
    assert 0.5 <= first["temperature"] <= 3.0
    assert first["visible_count"] == 3
    assert first["per_side"]["left"]["visible_count"] == 1
    assert first["per_side"]["right"]["visible_count"] == 2
    assert [point["retained_count"] for point in first["selective_curve"]] == [
        3,
        2,
    ]
    assert [
        point["retained_count"]
        for point in first["per_side"]["right"]["selective_curve"]
    ] == [2, 1]
    for key in (
        "temperature",
        "uncalibrated_nll",
        "calibrated_nll",
        "uncalibrated_ece",
        "calibrated_ece",
    ):
        assert math.isfinite(first[key])
    for point in first["selective_curve"]:
        assert 0.0 <= point["accuracy"] <= 1.0
        assert 0.0 <= point["confidence_threshold"] <= 1.0
        assert 0.0 <= point["mean_confidence"] <= 1.0


def test_invisible_finite_values_and_absent_targets_do_not_affect_fit():
    calibration = _load_calibration("room315_calibration_mask_invariance")
    logits, targets, visibility = _contract()
    visibility[0, 0] = True
    targets[0, 0] = 4
    logits[0, 0, 4] = 2.0

    baseline = calibration.compute_segment_calibration_report(
        logits,
        targets,
        visibility,
        coverages=(1.0,),
        grid_size=17,
        refinement_steps=1,
    )
    changed = logits.clone()
    changed[0, 1:] = torch.linspace(
        -1000.0, 1000.0, steps=7 * 14
    ).reshape(7, 14)
    repeated = calibration.compute_segment_calibration_report(
        changed,
        targets,
        visibility,
        coverages=(1.0,),
        grid_size=17,
        refinement_steps=1,
    )

    assert repeated["temperature"] == pytest.approx(baseline["temperature"])
    assert repeated["uncalibrated_nll"] == pytest.approx(
        baseline["uncalibrated_nll"]
    )
    assert repeated["calibrated_ece"] == pytest.approx(
        baseline["calibrated_ece"]
    )


def test_empty_side_is_explicit_in_per_side_report():
    calibration = _load_calibration("room315_calibration_empty_side")
    logits, targets, visibility = _contract()
    visibility[0, 0] = True
    targets[0, 0] = 0

    report = calibration.compute_segment_calibration_report(
        logits,
        targets,
        visibility,
        coverages=(1.0,),
        grid_size=9,
        refinement_steps=0,
    )

    assert report["per_side"]["left"]["available"] is True
    assert report["per_side"]["right"] == {
        "slot_indices": [4, 5, 6, 7],
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


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_logits_are_rejected_even_when_invisible(bad_value):
    calibration = _load_calibration("room315_calibration_nonfinite")
    logits, targets, visibility = _contract()
    visibility[0, 0] = True
    targets[0, 0] = 0
    logits[0, 7, 0] = bad_value

    with pytest.raises(ValueError, match="finite"):
        calibration.compute_segment_calibration_report(
            logits, targets, visibility
        )


def test_contract_and_validation_only_fit_are_enforced():
    calibration = _load_calibration("room315_calibration_contract")
    logits, targets, visibility = _contract()
    visibility[0, 0] = True
    targets[0, 0] = 0

    with pytest.raises(TypeError, match="boolean"):
        calibration.fit_segment_temperature(
            logits, targets, visibility.to(dtype=torch.int64)
        )
    with pytest.raises(ValueError, match="validation-only"):
        calibration.fit_segment_temperature(
            logits, targets, visibility, data_role="test"
        )
    with pytest.raises(ValueError, match="at least one"):
        calibration.fit_segment_temperature(
            logits, targets, torch.zeros_like(visibility)
        )
    with pytest.raises(ValueError, match="contain 1.0"):
        calibration.fit_segment_temperature(
            logits,
            targets,
            visibility,
            min_temperature=2.0,
            max_temperature=3.0,
        )


def test_saved_temperature_evaluation_does_not_refit_and_allows_canary():
    calibration = _load_calibration("room315_calibration_saved_temperature")
    logits, targets, visibility = _contract()
    visibility[0, [0, 4]] = True
    targets[0, [0, 4]] = torch.tensor([1, 2])
    logits[0, 0, 0] = 8.0
    logits[0, 4, 2] = 3.0
    saved_temperature = 2.25

    def forbidden_fit(*args, **kwargs):
        raise AssertionError("saved-temperature evaluation must not refit")

    calibration._fit_flat_temperature = forbidden_fit
    evaluated = calibration.evaluate_segment_calibration_at_temperature(
        logits,
        targets,
        visibility,
        temperature=saved_temperature,
        coverages=(1.0, 0.5),
        data_role="canary",
    )

    assert evaluated["temperature"] == saved_temperature
    assert evaluated["data_role"] == "canary"
    assert evaluated["source_temperature_role"] == "validation"
    assert evaluated["fit_performed"] is False
    assert evaluated["visible_count"] == 2
    assert evaluated["per_side"]["left"]["visible_count"] == 1
    assert evaluated["per_side"]["right"]["visible_count"] == 1


def test_saved_temperature_metrics_match_fitted_report_at_identical_temperature():
    calibration = _load_calibration("room315_calibration_saved_matches_fit")
    logits, targets, visibility = _contract(batch=2)
    visible = [(0, 0, 1), (0, 4, 2), (1, 1, 3), (1, 7, 4)]
    for batch, slot, target in visible:
        visibility[batch, slot] = True
        targets[batch, slot] = target
    logits[0, 0, 0] = 7.0
    logits[0, 4, 2] = 4.0
    logits[1, 1, 3] = 2.0
    logits[1, 7, 5] = 6.0
    kwargs = {
        "coverages": (1.0, 0.75, 0.5),
        "ece_bins": 7,
    }
    fitted = calibration.compute_segment_calibration_report(
        logits,
        targets,
        visibility,
        min_temperature=0.5,
        max_temperature=6.0,
        grid_size=33,
        refinement_steps=2,
        **kwargs,
    )
    evaluated = calibration.evaluate_segment_calibration_at_temperature(
        logits,
        targets,
        visibility,
        temperature=fitted["temperature"],
        data_role="validation",
        **kwargs,
    )

    assert evaluated["calibrated_nll"] == pytest.approx(
        fitted["calibrated_nll"]
    )
    assert evaluated["calibrated_ece"] == pytest.approx(
        fitted["calibrated_ece"]
    )
    assert evaluated["calibrated"] == fitted["calibrated"]
    assert evaluated["selective_curve"] == fitted["selective_curve"]
    for side in ("left", "right"):
        assert evaluated["per_side"][side]["calibrated"] == (
            fitted["per_side"][side]["calibrated"]
        )
        assert evaluated["per_side"][side]["selective_curve"] == (
            fitted["per_side"][side]["selective_curve"]
        )


@pytest.mark.parametrize("role", ["test", "train", "development"])
def test_saved_temperature_evaluation_rejects_forbidden_data_roles(role):
    calibration = _load_calibration("room315_calibration_saved_role")
    logits, targets, visibility = _contract()
    visibility[0, 0] = True
    targets[0, 0] = 0

    with pytest.raises(ValueError, match="validation or canary"):
        calibration.evaluate_segment_calibration_at_temperature(
            logits,
            targets,
            visibility,
            temperature=1.0,
            data_role=role,
        )
    with pytest.raises(ValueError, match="originate from validation"):
        calibration.evaluate_segment_calibration_at_temperature(
            logits,
            targets,
            visibility,
            temperature=1.0,
            data_role="canary",
            source_temperature_role="canary",
        )
