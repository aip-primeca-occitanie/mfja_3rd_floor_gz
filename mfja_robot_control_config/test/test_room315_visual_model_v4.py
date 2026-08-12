#!/usr/bin/env python3

import sys
from pathlib import Path

import pytest


torch = pytest.importorskip('torch')
torchvision = pytest.importorskip('torchvision')

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_model_v4 import V4_HEAD_GLOBAL
from room_315_visual_model_v4 import V4_HEAD_SPATIAL_QUERY
from room_315_visual_model_v4 import V4_OUTPUT_KEYS
from room_315_visual_model_v4 import V4_SLOT_ORDER
from room_315_visual_model_v4 import V4BackboneInitializationError
from room_315_visual_model_v4 import build_visual_state_model_v4
from room_315_visual_model_v4 import initialize_v4_backbone_from_v3_model_state_dict


@pytest.fixture(scope='module')
def spatial_model():
    torch.manual_seed(315)
    model = build_visual_state_model_v4(
        torch,
        torchvision,
        head_type=V4_HEAD_SPATIAL_QUERY,
        hidden_dim=64,
        attention_heads=4,
    )
    model.eval()
    return model


def _assert_structured_shapes(output, batch_size):
    assert tuple(output) == V4_OUTPUT_KEYS
    assert output['segment_logits'].shape == (batch_size, 8, 14)
    assert output['loaded_logits'].shape == (batch_size, 8, 2)
    assert output['bbox'].shape == (batch_size, 8, 4)
    assert output['s_ratio'].shape == (batch_size, 8, 1)
    assert torch.isfinite(output['segment_logits']).all()
    assert torch.isfinite(output['loaded_logits']).all()
    assert torch.all((output['bbox'] > 0.0) & (output['bbox'] < 1.0))
    x, y, width, height = output['bbox'].unbind(dim=-1)
    assert torch.all(x + width <= 1.0)
    assert torch.all(y + height <= 1.0)
    assert torch.all((output['s_ratio'] > 0.0) & (output['s_ratio'] < 1.0))


def _changed_tensor(value, offset):
    changed = value.detach().clone()
    if changed.is_floating_point() or changed.is_complex():
        return changed + float(offset)
    return changed + int(offset)


def _synthetic_v3_model_state_dict(model):
    state = {}
    for key, value in model.shared_stem.state_dict().items():
        state[f'backbone.{key}'] = _changed_tensor(value, 1)
    for key, value in model.left_layer4.state_dict().items():
        state[f'backbone.layer4.{key}'] = _changed_tensor(value, 2)
    # V3 head values are legal but must never be loaded into either V4 head.
    state['head.synthetic_sentinel'] = torch.tensor([315.0])
    return state


def _cloned_state(module):
    return {
        key: value.detach().clone()
        for key, value in module.state_dict().items()
    }


def test_v4_structured_shape_range_and_independent_late_branches(spatial_model):
    image = torch.randn(2, 6, 64, 64)
    with torch.inference_mode():
        output = spatial_model(image)

    _assert_structured_shapes(output, batch_size=2)
    assert spatial_model.slot_order == V4_SLOT_ORDER
    assert spatial_model.left_layer4 is not spatial_model.right_layer4
    left_bn = next(
        module
        for module in spatial_model.left_layer4.modules()
        if isinstance(module, torch.nn.BatchNorm2d)
    )
    right_bn = next(
        module
        for module in spatial_model.right_layer4.modules()
        if isinstance(module, torch.nn.BatchNorm2d)
    )
    assert left_bn is not right_bn
    assert left_bn.weight.data_ptr() != right_bn.weight.data_ptr()
    assert left_bn.running_mean.data_ptr() != right_bn.running_mean.data_ptr()


def test_v4_training_freezes_shared_bn_stats_but_trains_rail_local_bn():
    torch.manual_seed(319)
    model = build_visual_state_model_v4(
        torch,
        torchvision,
        hidden_dim=32,
        attention_heads=4,
    )
    model.train()
    shared_batch_norms = [
        module
        for module in model.shared_stem.modules()
        if isinstance(module, torch.nn.modules.batchnorm._BatchNorm)
    ]
    left_batch_norms = [
        module
        for module in model.left_layer4.modules()
        if isinstance(module, torch.nn.modules.batchnorm._BatchNorm)
    ]
    right_batch_norms = [
        module
        for module in model.right_layer4.modules()
        if isinstance(module, torch.nn.modules.batchnorm._BatchNorm)
    ]
    assert shared_batch_norms
    assert left_batch_norms
    assert right_batch_norms
    assert all(not module.training for module in shared_batch_norms)
    assert all(module.training for module in left_batch_norms)
    assert all(module.training for module in right_batch_norms)
    assert all(module.weight.requires_grad for module in shared_batch_norms)
    assert all(
        parameter.requires_grad
        for parameter in model.shared_stem.layer3.parameters()
    )

    shared_state_before = [
        (
            module.running_mean.detach().clone(),
            module.running_var.detach().clone(),
            module.num_batches_tracked.detach().clone(),
        )
        for module in shared_batch_norms
    ]
    left_batches_before = [
        module.num_batches_tracked.detach().clone()
        for module in left_batch_norms
    ]
    right_batches_before = [
        module.num_batches_tracked.detach().clone()
        for module in right_batch_norms
    ]
    base = torch.randn(2, 6, 64, 64)
    changed_right = base.clone()
    changed_right[:, 3:] = torch.randn_like(changed_right[:, 3:]) * 5.0
    with torch.no_grad():
        model(base)
        model(changed_right)

    for module, before in zip(shared_batch_norms, shared_state_before):
        assert torch.equal(module.running_mean, before[0])
        assert torch.equal(module.running_var, before[1])
        assert torch.equal(module.num_batches_tracked, before[2])
    assert all(
        module.num_batches_tracked > before
        for module, before in zip(left_batch_norms, left_batches_before)
    )
    assert all(
        module.num_batches_tracked > before
        for module, before in zip(right_batch_norms, right_batches_before)
    )

    model.zero_grad(set_to_none=True)
    gradient_output = model(base)['segment_logits'].square().mean()
    gradient_output.backward()
    shared_layer3_convolution = next(
        parameter
        for parameter in model.shared_stem.layer3.parameters()
        if parameter.ndim == 4
    )
    shared_bn_affine = shared_batch_norms[-1].weight
    assert shared_layer3_convolution.grad is not None
    assert torch.count_nonzero(shared_layer3_convolution.grad).item() > 0
    assert shared_bn_affine.grad is not None
    assert torch.count_nonzero(shared_bn_affine.grad).item() > 0
    for module, before in zip(shared_batch_norms, shared_state_before):
        assert torch.equal(module.running_mean, before[0])
        assert torch.equal(module.running_var, before[1])
        assert torch.equal(module.num_batches_tracked, before[2])


def test_v4_global_head_is_available_as_an_offline_ablation():
    model = build_visual_state_model_v4(
        torch,
        torchvision,
        head_type=V4_HEAD_GLOBAL,
        hidden_dim=32,
        attention_heads=4,
    ).eval()
    with torch.inference_mode():
        output = model(torch.zeros(1, 6, 64, 64))

    _assert_structured_shapes(output, batch_size=1)
    assert model.head_type == V4_HEAD_GLOBAL


def test_v4_outputs_are_invariant_to_the_opposite_camera(spatial_model):
    torch.manual_seed(316)
    base = torch.randn(1, 6, 64, 64)
    changed_right = base.clone()
    changed_right[:, 3:] = torch.randn_like(changed_right[:, 3:]) * 4.0
    changed_left = base.clone()
    changed_left[:, :3] = torch.randn_like(changed_left[:, :3]) * 4.0

    with torch.inference_mode():
        reference = spatial_model(base)
        right_changed = spatial_model(changed_right)
        left_changed = spatial_model(changed_left)

    for key in V4_OUTPUT_KEYS:
        torch.testing.assert_close(
            reference[key][:, :4],
            right_changed[key][:, :4],
            rtol=0.0,
            atol=0.0,
        )
        torch.testing.assert_close(
            reference[key][:, 4:],
            left_changed[key][:, 4:],
            rtol=0.0,
            atol=0.0,
        )
    assert not torch.equal(
        reference['segment_logits'][:, 4:],
        right_changed['segment_logits'][:, 4:],
    )
    assert not torch.equal(
        reference['segment_logits'][:, :4],
        left_changed['segment_logits'][:, :4],
    )


def test_v4_cross_side_input_gradients_are_exactly_zero(spatial_model):
    torch.manual_seed(317)
    image = torch.randn(1, 6, 64, 64, requires_grad=True)
    output = spatial_model(image)

    left_score = (
        output['segment_logits'][:, :4].square().sum()
        + output['loaded_logits'][:, :4].square().sum()
        + output['bbox'][:, :4].sum()
        + output['s_ratio'][:, :4].sum()
    )
    left_gradient, left_to_right_branch = torch.autograd.grad(
        left_score,
        (image, next(spatial_model.right_layer4.parameters())),
        allow_unused=True,
        retain_graph=True,
    )
    assert left_to_right_branch is None or (
        torch.count_nonzero(left_to_right_branch).item() == 0
    )
    assert torch.count_nonzero(left_gradient[:, 3:]).item() == 0
    assert torch.count_nonzero(left_gradient[:, :3]).item() > 0

    right_score = (
        output['segment_logits'][:, 4:].square().sum()
        + output['loaded_logits'][:, 4:].square().sum()
        + output['bbox'][:, 4:].sum()
        + output['s_ratio'][:, 4:].sum()
    )
    right_gradient, right_to_left_branch = torch.autograd.grad(
        right_score,
        (image, next(spatial_model.left_layer4.parameters())),
        allow_unused=True,
    )
    assert right_to_left_branch is None or (
        torch.count_nonzero(right_to_left_branch).item() == 0
    )
    assert torch.count_nonzero(right_gradient[:, :3]).item() == 0
    assert torch.count_nonzero(right_gradient[:, 3:]).item() > 0


def test_v4_strictly_migrates_v3_backbone_without_touching_heads():
    torch.manual_seed(318)
    model = build_visual_state_model_v4(
        torch,
        torchvision,
        hidden_dim=32,
        attention_heads=4,
    )
    source = _synthetic_v3_model_state_dict(model)
    left_head_before = _cloned_state(model.left_head)
    right_head_before = _cloned_state(model.right_head)

    report = initialize_v4_backbone_from_v3_model_state_dict(model, source)

    for key, value in model.shared_stem.state_dict().items():
        assert torch.equal(value, source[f'backbone.{key}'])
    for branch in (model.left_layer4, model.right_layer4):
        for key, value in branch.state_dict().items():
            assert torch.equal(value, source[f'backbone.layer4.{key}'])
    assert all(
        torch.equal(value, left_head_before[key])
        for key, value in model.left_head.state_dict().items()
    )
    assert all(
        torch.equal(value, right_head_before[key])
        for key, value in model.right_head.state_dict().items()
    )
    assert report['strict'] is True
    assert report['missing_source_keys'] == []
    assert report['unexpected_source_keys'] == []
    assert report['prediction_heads_touched'] is False
    assert report['prediction_heads_unchanged'] is True
    assert report['shared_stem']['all_tensors_equal_source'] is True
    assert report['layer4']['left_all_tensors_equal_source'] is True
    assert report['layer4']['right_all_tensors_equal_source'] is True
    assert report['ignored_non_backbone_keys'] == ['head.synthetic_sentinel']


def test_v4_v3_backbone_migration_rejects_missing_and_unexpected_tensors():
    model = build_visual_state_model_v4(
        torch,
        torchvision,
        hidden_dim=32,
        attention_heads=4,
    )
    complete = _synthetic_v3_model_state_dict(model)
    original_shared_stem = _cloned_state(model.shared_stem)

    missing = dict(complete)
    missing_key = next(
        key
        for key in missing
        if key.startswith('backbone.layer3.')
    )
    missing.pop(missing_key)
    with pytest.raises(V4BackboneInitializationError, match='missing='):
        initialize_v4_backbone_from_v3_model_state_dict(model, missing)

    unexpected = dict(complete)
    unexpected['backbone.fc.synthetic'] = torch.zeros(1)
    with pytest.raises(V4BackboneInitializationError, match='unexpected='):
        initialize_v4_backbone_from_v3_model_state_dict(model, unexpected)

    # Key-set failures are checked before the first model tensor is changed.
    assert all(
        torch.equal(value, original_shared_stem[key])
        for key, value in model.shared_stem.state_dict().items()
    )
