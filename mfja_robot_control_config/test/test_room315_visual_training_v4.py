#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path

import pytest


torch = pytest.importorskip("torch")

REPO_ROOT = Path(__file__).resolve().parents[2]
V4_SCRIPT = (
    REPO_ROOT
    / "mfja_robot_control_config"
    / "scripts"
    / "room_315_visual_training_v4.py"
)


def _load_v4(name="room315_visual_training_v4_test"):
    spec = importlib.util.spec_from_file_location(name, V4_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _contract(batch=1, *, requires_grad=False):
    segment_logits = torch.zeros(batch, 8, 14, dtype=torch.float32)
    loaded_logits = torch.zeros(batch, 8, 2, dtype=torch.float32)
    bbox = torch.zeros(batch, 8, 4, dtype=torch.float32)
    bbox[..., 0] = 100.0
    bbox[..., 1] = 80.0
    bbox[..., 2] = 40.0
    bbox[..., 3] = 30.0
    s_ratio = torch.full((batch, 8, 1), 0.25, dtype=torch.float32)
    if requires_grad:
        segment_logits.requires_grad_()
        loaded_logits.requires_grad_()
        bbox.requires_grad_()
        s_ratio.requires_grad_()
    predictions = {
        "segment_logits": segment_logits,
        "loaded_logits": loaded_logits,
        "bbox": bbox,
        "s_ratio": s_ratio,
    }
    targets = {
        "segment": torch.full((batch, 8), -100, dtype=torch.long),
        "loaded": torch.full((batch, 8), -100, dtype=torch.long),
        "bbox": bbox.detach().clone(),
        "s_ratio": s_ratio.detach().clone(),
        "visibility_mask": torch.zeros(batch, 8, dtype=torch.bool),
    }
    return predictions, targets


def test_inverse_sqrt_class_weights_are_finite_mean_one_and_capped():
    v4 = _load_v4("room315_v4_class_weights")

    weights = v4.inverse_sqrt_class_weights([100.0, 25.0, 1.0, 0.0], cap=2.0)

    assert torch.isfinite(weights).all()
    assert weights.mean().item() == pytest.approx(1.0)
    assert (weights.max() / weights.min()).item() == pytest.approx(2.0)
    assert weights[1] > weights[0]
    assert weights[2] == pytest.approx(weights[3])


@pytest.mark.parametrize("rows", [2, 8])
def test_inverse_sqrt_class_weights_support_side_and_slot_counts(rows):
    v4 = _load_v4(f"room315_v4_class_weight_rows_{rows}")
    counts = torch.arange(1, rows * 14 + 1, dtype=torch.float32).reshape(rows, 14)

    weights = v4.inverse_sqrt_class_weights(counts, cap=2.5)

    assert weights.shape == (rows, 14)
    assert torch.isfinite(weights).all()
    assert torch.allclose(weights.mean(dim=-1), torch.ones(rows))
    assert torch.all(weights.max(dim=-1).values / weights.min(dim=-1).values <= 2.5)


def test_masked_loss_matches_expected_components_and_ignores_absent_labels():
    v4 = _load_v4("room315_v4_masked_numerics")
    predictions, targets = _contract()
    targets["visibility_mask"][0, 0] = True
    targets["segment"][0, 0] = 0
    targets["loaded"][0, 0] = 1
    predictions["s_ratio"][0, 0, 0] = 0.3
    targets["s_ratio"][0, 0, 0] = 0.1
    distances = torch.arange(14, dtype=torch.float32).sub(
        torch.arange(14, dtype=torch.float32).unsqueeze(1)
    ).abs()

    result = v4.compute_v4_loss(
        predictions,
        targets,
        topology_distance_matrix=distances,
    )

    assert result["visible_count"] == 1
    assert result["segment_ce"].item() == pytest.approx(math.log(14.0))
    assert result["loaded_ce"].item() == pytest.approx(math.log(2.0))
    assert result["bbox_l1"].item() == pytest.approx(0.0)
    assert result["bbox_giou"].item() == pytest.approx(0.0, abs=1e-6)
    assert result["s_ratio"].item() == pytest.approx(0.2 - 0.5 * 0.05)
    assert result["topology"].item() == pytest.approx(6.5)
    expected = (
        4.0 * math.log(14.0)
        + math.log(2.0)
        + 2.0 * 0.175
        + 0.25 * 6.5
    )
    assert result["loss"].item() == pytest.approx(expected)
    assert result["head_parameters"] == {
        "segment_label_smoothing": 0.02,
        "loaded_label_smoothing": 0.01,
        "s_ratio_beta": 0.05,
        "bbox_l1_weight": 1.0,
        "bbox_giou_weight": 1.0,
    }


def test_bbox_loss_combines_normalized_l1_and_generalized_iou():
    v4 = _load_v4("room315_v4_bbox_numerics")
    predictions, targets = _contract()
    targets["visibility_mask"][0, 0] = True
    targets["segment"][0, 0] = 0
    targets["loaded"][0, 0] = 0
    targets["bbox"][0, 0] = torch.tensor([0.0, 0.0, 0.1, 0.1])
    predictions["bbox"][0, 0] = torch.tensor([0.1, 0.0, 0.1, 0.1])

    result = v4.compute_v4_loss(
        predictions,
        targets,
        bbox_l1_weight=2.0,
        bbox_giou_weight=0.5,
    )

    assert result["bbox_l1"].item() == pytest.approx(0.025)
    assert result["bbox_giou"].item() == pytest.approx(1.0)
    assert result["bbox"].item() == pytest.approx(0.55)


def test_invisible_slots_do_not_change_loss_or_receive_gradients():
    v4 = _load_v4("room315_v4_mask_gradient")
    predictions, targets = _contract(requires_grad=True)
    targets["visibility_mask"][0, 0] = True
    targets["segment"][0, 0] = 3
    targets["loaded"][0, 0] = 1

    with torch.no_grad():
        predictions["segment_logits"][0, 1:] = 1.0e6
        predictions["loaded_logits"][0, 1:] = -1.0e6
        predictions["bbox"][0, 1:] = 1.0e6
        predictions["s_ratio"][0, 1:] = -1.0e6
    result = v4.compute_v4_loss(predictions, targets)
    result["loss"].backward()

    for tensor in predictions.values():
        assert tensor.grad is not None
        assert torch.isfinite(tensor.grad).all()
        assert torch.count_nonzero(tensor.grad[0, 1:]).item() == 0

    clean_predictions, clean_targets = _contract()
    clean_targets["visibility_mask"][0, 0] = True
    clean_targets["segment"][0, 0] = 3
    clean_targets["loaded"][0, 0] = 1
    clean = v4.compute_v4_loss(clean_predictions, clean_targets)
    assert result["loss"].item() == pytest.approx(clean["loss"].item())


def test_empty_visibility_mask_returns_differentiable_zero():
    v4 = _load_v4("room315_v4_empty_mask")
    predictions, targets = _contract(requires_grad=True)

    result = v4.compute_v4_loss(predictions, targets)
    result["loss"].backward()

    assert result["visible_count"] == 0
    assert result["loss"].item() == pytest.approx(0.0)
    for tensor in predictions.values():
        assert tensor.grad is not None
        assert torch.count_nonzero(tensor.grad).item() == 0


def test_segment_class_weights_change_masked_cross_entropy_as_expected():
    v4 = _load_v4("room315_v4_weighted_ce")
    predictions, targets = _contract()
    targets["visibility_mask"][0, :2] = True
    targets["segment"][0, :2] = torch.tensor([0, 1])
    targets["loaded"][0, :2] = 0
    predictions["segment_logits"][0, 0, 0] = 5.0
    predictions["segment_logits"][0, 1, 0] = 5.0

    uniform = v4.compute_v4_loss(predictions, targets)
    weights = torch.ones(14)
    weights[1] = 4.0
    weighted = v4.compute_v4_loss(
        predictions,
        targets,
        segment_class_weights=weights,
    )

    assert weighted["segment_ce"] > uniform["segment_ce"]


@pytest.mark.parametrize("weight_rows", [2, 8])
def test_segment_cross_entropy_applies_side_or_slot_specific_weights(weight_rows):
    v4 = _load_v4(f"room315_v4_spatial_weights_{weight_rows}")
    predictions, targets = _contract()
    targets["visibility_mask"][0, [0, 4]] = True
    targets["segment"][0, [0, 4]] = 0
    targets["loaded"][0, [0, 4]] = 0
    predictions["segment_logits"][0, 0, 0] = 4.0
    predictions["segment_logits"][0, 4, 1] = 4.0
    class_weights = torch.ones(weight_rows, 14)
    class_weights[-1 if weight_rows == 2 else 4, 0] = 4.0

    result = v4.compute_v4_loss(
        predictions,
        targets,
        segment_class_weights=class_weights,
        segment_label_smoothing=0.0,
    )

    left_nll = -torch.log_softmax(predictions["segment_logits"][0, 0], dim=-1)[0]
    right_nll = -torch.log_softmax(predictions["segment_logits"][0, 4], dim=-1)[0]
    expected = (left_nll + 4.0 * right_nll) / 5.0
    assert result["segment_ce"].item() == pytest.approx(expected.item())


@pytest.mark.parametrize(
    "overrides",
    [
        {"segment_label_smoothing": -0.1},
        {"loaded_label_smoothing": 1.0},
        {"s_ratio_beta": 0.0},
        {"bbox_l1_weight": -1.0},
        {"bbox_l1_weight": 0.0, "bbox_giou_weight": 0.0},
    ],
)
def test_v4_head_hyperparameters_are_validated(overrides):
    v4 = _load_v4("room315_v4_hyperparameter_validation")
    predictions, targets = _contract()
    targets["visibility_mask"][0, 0] = True
    targets["segment"][0, 0] = 0
    targets["loaded"][0, 0] = 0

    with pytest.raises(ValueError):
        v4.compute_v4_loss(predictions, targets, **overrides)


def test_metrics_expose_zero_recall_classes_per_side_and_regression_quality():
    v4 = _load_v4("room315_v4_metrics")
    predictions, targets = _contract()
    visible_slots = [0, 1, 4]
    targets["visibility_mask"][0, visible_slots] = True
    targets["segment"][0, visible_slots] = torch.tensor([0, 1, 2])
    targets["loaded"][0, visible_slots] = torch.tensor([0, 1, 1])
    predictions["segment_logits"][0, 0, 0] = 5.0
    predictions["segment_logits"][0, 1, 0] = 5.0
    predictions["segment_logits"][0, 1, 1] = 4.0
    predictions["segment_logits"][0, 4, 2] = 5.0
    predictions["loaded_logits"][0, 0, 0] = 5.0
    predictions["loaded_logits"][0, 1, 0] = 5.0
    predictions["loaded_logits"][0, 4, 1] = 5.0
    targets["s_ratio"][0, visible_slots, 0] = torch.tensor([0.15, 0.45, 0.55])
    predictions["s_ratio"][0, visible_slots, 0] = torch.tensor([0.25, 0.25, 0.25])
    names = [f"S{index}" for index in range(14)]

    metrics = v4.compute_v4_metrics(
        predictions,
        targets,
        segment_class_names=names,
    )

    assert metrics["visible_count"] == 3
    assert metrics["segment_top1_accuracy"] == pytest.approx(2.0 / 3.0)
    assert metrics["segment_top2_accuracy"] == pytest.approx(1.0)
    assert metrics["segment_macro_recall"] == pytest.approx(2.0 / 14.0)
    assert metrics["segment_zero_recall_classes"] == ["S1"]
    assert metrics["segment_per_class"][1]["support"] == 1
    assert metrics["segment_per_class"][1]["recall"] == 0.0
    assert metrics["per_side"]["left"]["top1_accuracy"] == pytest.approx(0.5)
    assert metrics["per_side"]["left"]["top2_accuracy"] == pytest.approx(1.0)
    assert metrics["per_side"]["right"]["top1_accuracy"] == pytest.approx(1.0)
    assert metrics["per_identity"]["L1"]["top1_accuracy"] == pytest.approx(1.0)
    assert metrics["per_identity"]["L2"]["top1_accuracy"] == pytest.approx(0.0)
    assert metrics["per_identity"]["L2"]["top2_accuracy"] == pytest.approx(1.0)
    assert metrics["per_identity"]["R1"]["top1_accuracy"] == pytest.approx(1.0)
    assert metrics["loaded_accuracy"] == pytest.approx(2.0 / 3.0)
    assert metrics["s_ratio_mae"] == pytest.approx(0.2)
    assert metrics["s_ratio_p95"] == pytest.approx(0.29)
    assert metrics["bbox_iou"] == pytest.approx(1.0)
    assert metrics["joint_localization_accuracy_s_ratio_0_05"] == 0.0
    assert metrics["joint_localization_accuracy_s_ratio_0_12"] == pytest.approx(
        1.0 / 3.0
    )


def test_metrics_report_only_physically_valid_s_m_errors_and_joint_accuracy():
    v4 = _load_v4("room315_v4_s_m_metrics")
    predictions, targets = _contract()
    visible_slots = [0, 1, 4, 5]
    targets["visibility_mask"][0, visible_slots] = True
    targets["segment"][0, visible_slots] = torch.tensor([0, 3, 2, 5])
    targets["loaded"][0, visible_slots] = 0
    predictions["segment_logits"][0, 0, 1] = 5.0
    predictions["segment_logits"][0, 1, 3] = 5.0
    predictions["segment_logits"][0, 4, 2] = 5.0
    predictions["segment_logits"][0, 5, 5] = 5.0
    targets["s_ratio"][0, visible_slots, 0] = 0.5
    predictions["s_ratio"][0, visible_slots, 0] = torch.tensor(
        [0.5, 0.54, 0.60, 0.63]
    )
    lengths = torch.ones(2, 14)
    lengths[0, 0] = 2.0
    lengths[0, 1] = 4.0
    lengths[0, 3] = 2.0
    lengths[1, 2] = 4.0
    lengths[1, 5] = 10.0

    metrics = v4.compute_v4_metrics(
        predictions,
        targets,
        segment_lengths_by_side=lengths,
    )

    assert "s_m_mae_m" not in metrics
    assert "s_m_p95_m" not in metrics
    assert "joint" not in metrics["s_m_metrics"]
    assert metrics["correct_segment_s_m_mae_m"] == pytest.approx(1.78 / 3.0)
    assert metrics["correct_segment_s_m_p95_m"] == pytest.approx(1.21)
    assert metrics["correct_segment_s_m_count"] == 3
    assert metrics["correct_segment_s_m_coverage"] == pytest.approx(0.75)
    assert metrics["oracle_true_segment_ratio_s_m_mae_m"] == pytest.approx(
        1.78 / 4.0
    )
    assert metrics["oracle_true_segment_ratio_s_m_p95_m"] == pytest.approx(1.165)
    assert metrics["s_m_metrics"]["correct_segment_only"] == {
        "count": 3,
        "mae_m": pytest.approx(1.78 / 3.0),
        "p95_m": pytest.approx(1.21),
        "coverage": pytest.approx(0.75),
    }
    assert metrics["s_m_metrics"]["oracle_true_segment_ratio_diagnostic"][
        "count"
    ] == 4
    assert metrics["joint_localization_accuracy"] == {
        "s_ratio_tolerance_0_05": pytest.approx(0.25),
        "s_ratio_tolerance_0_12": pytest.approx(0.5),
    }


@pytest.mark.parametrize(
    "prediction_name",
    ["segment_logits", "loaded_logits", "bbox", "s_ratio"],
)
def test_loss_and_metrics_reject_nonfinite_predictions(prediction_name):
    v4 = _load_v4(f"room315_v4_nonfinite_{prediction_name}")
    predictions, targets = _contract()
    targets["visibility_mask"][0, 0] = True
    targets["segment"][0, 0] = 0
    targets["loaded"][0, 0] = 0
    predictions[prediction_name].reshape(-1)[0] = float("nan")

    with pytest.raises(ValueError, match="finite"):
        v4.compute_v4_loss(predictions, targets)
    with pytest.raises(ValueError, match="finite"):
        v4.compute_v4_metrics(predictions, targets)


@pytest.mark.parametrize("target_name", ["bbox", "s_ratio"])
def test_loss_and_metrics_reject_nonfinite_visible_regression_targets(target_name):
    v4 = _load_v4(f"room315_v4_nonfinite_target_{target_name}")
    predictions, targets = _contract()
    targets["visibility_mask"][0, 0] = True
    targets["segment"][0, 0] = 0
    targets["loaded"][0, 0] = 0
    targets[target_name].reshape(-1)[0] = float("inf")

    with pytest.raises(ValueError, match="finite"):
        v4.compute_v4_loss(predictions, targets)
    with pytest.raises(ValueError, match="finite"):
        v4.compute_v4_metrics(predictions, targets)


def test_all_v4_loss_heads_have_finite_gradients_with_topology_penalty():
    v4 = _load_v4("room315_v4_finite_gradients")
    generator = torch.Generator().manual_seed(315)
    segment_logits = torch.randn(2, 8, 14, generator=generator, requires_grad=True)
    loaded_logits = torch.randn(2, 8, 2, generator=generator, requires_grad=True)
    bbox_data = torch.rand(2, 8, 4, generator=generator)
    bbox_data[..., 0] *= 500.0
    bbox_data[..., 1] *= 350.0
    bbox_data[..., 2] = 20.0 + bbox_data[..., 2] * 80.0
    bbox_data[..., 3] = 20.0 + bbox_data[..., 3] * 80.0
    bbox = bbox_data.requires_grad_()
    s_ratio = torch.rand(2, 8, 1, generator=generator, requires_grad=True)
    predictions = {
        "segment_logits": segment_logits,
        "loaded_logits": loaded_logits,
        "bbox": bbox,
        "s_ratio": s_ratio,
    }
    targets = {
        "segment": torch.randint(0, 14, (2, 8), generator=generator),
        "loaded": torch.randint(0, 2, (2, 8), generator=generator),
        "bbox": bbox_data.roll(1, dims=1),
        "s_ratio": torch.rand(2, 8, 1, generator=generator),
        "visibility_mask": torch.tensor(
            [
                [1, 1, 0, 1, 0, 1, 1, 0],
                [1, 0, 1, 1, 1, 0, 1, 1],
            ],
            dtype=torch.bool,
        ),
    }
    indexes = torch.arange(14, dtype=torch.float32)
    distances = (indexes[:, None] - indexes[None, :]).abs()

    result = v4.compute_v4_loss(
        predictions,
        targets,
        segment_class_counts=torch.arange(1, 15, dtype=torch.float32),
        topology_distance_matrix=distances,
    )
    result["loss"].backward()

    assert torch.isfinite(result["loss"])
    for tensor in predictions.values():
        assert tensor.grad is not None
        assert torch.isfinite(tensor.grad).all()
        assert torch.count_nonzero(tensor.grad).item() > 0
