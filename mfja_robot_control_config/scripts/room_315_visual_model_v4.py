#!/usr/bin/env python3
"""Experimental Room 315 V4 paired-camera visual-state model.

V4 deliberately remains separate from the approved V3 model and runtime.  It
shares only the low-level ResNet-18 stem between cameras; every rail-specific
feature and prediction module is independent.  The default head retains the
spatial feature map and lets four fixed identity queries attend to it.

The builder always constructs ResNet-18 with ``weights=None``.  A future V4
trainer may explicitly migrate verified weights into the returned modules,
but constructing or testing this model never downloads a checkpoint.
"""

from __future__ import annotations

import copy
import math
from collections import OrderedDict
from collections.abc import Mapping
from typing import Any


V4_MODEL_KIND = 'room315_visual_state_resnet18_split_rails_v4'
V4_HEAD_SPATIAL_QUERY = 'spatial_query'
V4_HEAD_GLOBAL = 'global'
V4_HEAD_TYPES = frozenset({V4_HEAD_SPATIAL_QUERY, V4_HEAD_GLOBAL})
V4_SLOT_ORDER = ('L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4')
V4_OUTPUT_KEYS = ('segment_logits', 'loaded_logits', 'bbox', 's_ratio')
V4_SLOTS_PER_SIDE = 4
V4_SEGMENT_CLASS_COUNT = 14
V4_LOADED_CLASS_COUNT = 2
V4_V3_BACKBONE_MIGRATION_SCHEMA = (
    'room315.visual_model_v4.v3_backbone_migration.v1'
)


class V4BackboneInitializationError(ValueError):
    """Raised before V4 can accept an incomplete V3 backbone migration."""


def initialize_v4_backbone_from_v3_model_state_dict(
    model: Any,
    v3_model_state_dict: Mapping[str, Any],
) -> dict[str, Any]:
    """Strictly migrate a V3 ``backbone.*`` state into a V4 model.

    V3 ``conv1`` through ``layer3`` are copied once into ``shared_stem``.
    V3 ``layer4`` is copied independently into both rail branches.  Prediction
    heads and all non-backbone V3 tensors are intentionally ignored.

    The complete relevant source key set, tensor shapes, and dtypes are
    validated before any model tensor is mutated.  Extra ``backbone.*`` keys
    therefore fail just like missing keys instead of being silently ignored.
    """

    if getattr(model, 'model_kind', None) != V4_MODEL_KIND:
        raise V4BackboneInitializationError(
            f'model must declare model_kind={V4_MODEL_KIND!r}'
        )
    if not isinstance(v3_model_state_dict, Mapping):
        raise V4BackboneInitializationError(
            'v3_model_state_dict must be a tensor mapping'
        )
    required_modules = (
        'shared_stem',
        'left_layer4',
        'right_layer4',
        'left_head',
        'right_head',
    )
    missing_modules = [name for name in required_modules if not hasattr(model, name)]
    if missing_modules:
        raise V4BackboneInitializationError(
            f'V4 model is missing required modules: {missing_modules}'
        )

    shared_target_state = model.shared_stem.state_dict()
    left_layer4_target_state = model.left_layer4.state_dict()
    right_layer4_target_state = model.right_layer4.state_dict()
    if tuple(left_layer4_target_state) != tuple(right_layer4_target_state):
        raise V4BackboneInitializationError(
            'left and right layer4 state layouts do not match'
        )

    shared_source_by_target = {
        target_key: f'backbone.{target_key}'
        for target_key in shared_target_state
    }
    layer4_source_by_target = {
        target_key: f'backbone.layer4.{target_key}'
        for target_key in left_layer4_target_state
    }
    expected_source_keys = set(shared_source_by_target.values()) | set(
        layer4_source_by_target.values()
    )
    actual_backbone_keys = {
        key
        for key in v3_model_state_dict
        if isinstance(key, str) and key.startswith('backbone.')
    }
    missing_source_keys = sorted(expected_source_keys - actual_backbone_keys)
    unexpected_source_keys = sorted(actual_backbone_keys - expected_source_keys)
    if missing_source_keys or unexpected_source_keys:
        raise V4BackboneInitializationError(
            'V3 backbone key set is incompatible; '
            f'missing={missing_source_keys}, unexpected={unexpected_source_keys}'
        )

    def validated_source_tensor(source_key: str, target_tensor: Any):
        source_tensor = v3_model_state_dict[source_key]
        if not all(
            hasattr(source_tensor, attribute)
            for attribute in ('shape', 'dtype', 'detach')
        ):
            raise V4BackboneInitializationError(
                f'V3 backbone value is not a tensor: {source_key}'
            )
        if tuple(source_tensor.shape) != tuple(target_tensor.shape):
            raise V4BackboneInitializationError(
                f'V3 backbone tensor shape mismatch for {source_key}: '
                f'{tuple(source_tensor.shape)} != {tuple(target_tensor.shape)}'
            )
        if source_tensor.dtype != target_tensor.dtype:
            raise V4BackboneInitializationError(
                f'V3 backbone tensor dtype mismatch for {source_key}: '
                f'{source_tensor.dtype} != {target_tensor.dtype}'
            )
        return source_tensor

    shared_load_state = {
        target_key: validated_source_tensor(source_key, shared_target_state[target_key])
        for target_key, source_key in shared_source_by_target.items()
    }
    layer4_load_state = {
        target_key: validated_source_tensor(
            source_key,
            left_layer4_target_state[target_key],
        )
        for target_key, source_key in layer4_source_by_target.items()
    }
    for target_key, right_tensor in right_layer4_target_state.items():
        source_key = layer4_source_by_target[target_key]
        validated_source_tensor(source_key, right_tensor)

    head_state_before = {
        'left': {
            key: value.detach().clone()
            for key, value in model.left_head.state_dict().items()
        },
        'right': {
            key: value.detach().clone()
            for key, value in model.right_head.state_dict().items()
        },
    }

    model.shared_stem.load_state_dict(shared_load_state, strict=True)
    model.left_layer4.load_state_dict(layer4_load_state, strict=True)
    model.right_layer4.load_state_dict(layer4_load_state, strict=True)

    def tensors_equal(first: Any, second: Any) -> bool:
        return bool(first.detach().cpu().equal(second.detach().cpu()))

    loaded_shared_state = model.shared_stem.state_dict()
    loaded_left_layer4_state = model.left_layer4.state_dict()
    loaded_right_layer4_state = model.right_layer4.state_dict()
    shared_matches = all(
        tensors_equal(
            loaded_shared_state[target_key],
            v3_model_state_dict[source_key],
        )
        for target_key, source_key in shared_source_by_target.items()
    )
    left_layer4_matches = all(
        tensors_equal(
            loaded_left_layer4_state[target_key],
            v3_model_state_dict[source_key],
        )
        for target_key, source_key in layer4_source_by_target.items()
    )
    right_layer4_matches = all(
        tensors_equal(
            loaded_right_layer4_state[target_key],
            v3_model_state_dict[source_key],
        )
        for target_key, source_key in layer4_source_by_target.items()
    )
    heads_unchanged = all(
        tensors_equal(current, head_state_before[side][key])
        for side, head in (
            ('left', model.left_head),
            ('right', model.right_head),
        )
        for key, current in head.state_dict().items()
    )
    if not (
        shared_matches
        and left_layer4_matches
        and right_layer4_matches
        and heads_unchanged
    ):
        raise V4BackboneInitializationError(
            'post-load V3 backbone equality verification failed'
        )

    ignored_non_backbone_keys = sorted(
        key
        for key in v3_model_state_dict
        if isinstance(key, str) and not key.startswith('backbone.')
    )
    return {
        'schema_version': V4_V3_BACKBONE_MIGRATION_SCHEMA,
        'strict': True,
        'source_prefix': 'backbone.',
        'source_backbone_key_count': len(actual_backbone_keys),
        'loaded_source_keys': sorted(expected_source_keys),
        'ignored_non_backbone_keys': ignored_non_backbone_keys,
        'missing_source_keys': missing_source_keys,
        'unexpected_source_keys': unexpected_source_keys,
        'shared_stem': {
            'source_keys': sorted(shared_source_by_target.values()),
            'target_keys': list(shared_source_by_target),
            'tensor_count': len(shared_source_by_target),
            'all_tensors_equal_source': shared_matches,
        },
        'layer4': {
            'source_keys': sorted(layer4_source_by_target.values()),
            'left_target_keys': list(layer4_source_by_target),
            'right_target_keys': list(layer4_source_by_target),
            'tensor_count_per_side': len(layer4_source_by_target),
            'left_all_tensors_equal_source': left_layer4_matches,
            'right_all_tensors_equal_source': right_layer4_matches,
            'rail_branch_storage_shared': False,
        },
        'prediction_heads_touched': False,
        'prediction_heads_unchanged': heads_unchanged,
    }


def build_visual_state_model_v4(
    torch_module: Any,
    torchvision_module: Any,
    *,
    head_type: str = V4_HEAD_SPATIAL_QUERY,
    hidden_dim: int = 128,
    attention_heads: int = 4,
    dropout: float = 0.0,
):
    """Build the isolated V4 structured visual-state model.

    Input is ``[B, 6, H, W]`` with left RGB in channels ``0:3`` and right RGB
    in channels ``3:6``.  The returned dictionary has the fixed slot order
    ``L1..L4,R1..R4`` and these shapes:

    * ``segment_logits``: ``[B, 8, 14]``
    * ``loaded_logits``: ``[B, 8, 2]``
    * ``bbox``: ``[B, 8, 4]`` normalized ``(x, y, width, height)``
    * ``s_ratio``: ``[B, 8, 1]`` in the open interval ``(0, 1)``
    """

    normalized_head_type = str(head_type or '').strip().lower()
    if normalized_head_type not in V4_HEAD_TYPES:
        raise ValueError(
            f'unsupported V4 head_type {normalized_head_type!r}; '
            f'expected one of {sorted(V4_HEAD_TYPES)}'
        )
    model_dim = int(hidden_dim)
    number_of_heads = int(attention_heads)
    if model_dim <= 0 or model_dim % 4:
        raise ValueError('hidden_dim must be positive and divisible by four')
    if number_of_heads <= 0 or model_dim % number_of_heads:
        raise ValueError(
            'attention_heads must be positive and divide hidden_dim exactly'
        )
    dropout_probability = float(dropout)
    if not math.isfinite(dropout_probability) or not 0.0 <= dropout_probability < 1.0:
        raise ValueError('dropout must be finite and in [0, 1)')

    nn = torch_module.nn

    def sine_position_encoding(feature_map):
        """Return deterministic two-dimensional Fourier positions as [1, HW, D]."""

        height, width = int(feature_map.shape[-2]), int(feature_map.shape[-1])
        dtype = feature_map.dtype
        device = feature_map.device
        quarter_dim = model_dim // 4
        y_coordinates = torch_module.linspace(
            0.0,
            1.0,
            height,
            dtype=dtype,
            device=device,
        )
        x_coordinates = torch_module.linspace(
            0.0,
            1.0,
            width,
            dtype=dtype,
            device=device,
        )
        y_grid, x_grid = torch_module.meshgrid(
            y_coordinates,
            x_coordinates,
            indexing='ij',
        )
        frequency_indexes = torch_module.arange(
            quarter_dim,
            dtype=dtype,
            device=device,
        )
        frequency_scale = torch_module.pow(
            torch_module.as_tensor(10_000.0, dtype=dtype, device=device),
            frequency_indexes / float(max(1, quarter_dim - 1)),
        )
        x_phase = (2.0 * math.pi * x_grid[..., None]) / frequency_scale
        y_phase = (2.0 * math.pi * y_grid[..., None]) / frequency_scale
        encoding = torch_module.cat(
            (
                torch_module.sin(x_phase),
                torch_module.cos(x_phase),
                torch_module.sin(y_phase),
                torch_module.cos(y_phase),
            ),
            dim=-1,
        )
        return encoding.reshape(1, height * width, model_dim)

    class StructuredOutputProjection(nn.Module):
        """Project four rail-local slot embeddings to the fixed V4 contract."""

        def __init__(self) -> None:
            super().__init__()
            self.segment_projection = nn.Linear(
                model_dim,
                V4_SEGMENT_CLASS_COUNT,
            )
            self.loaded_projection = nn.Linear(
                model_dim,
                V4_LOADED_CLASS_COUNT,
            )
            self.bbox_projection = nn.Linear(model_dim, 4)
            self.s_ratio_projection = nn.Linear(model_dim, 1)

        def forward(self, slots):
            # Treat each axis as (leading margin, extent, trailing margin).
            # Mixing softmax partitions with epsilon keeps every component
            # positive and guarantees x + width <= 1 and y + height <= 1.
            epsilon = 1.0e-4
            raw_bbox = self.bbox_projection(slots)
            zero_margin_logit = torch_module.zeros_like(raw_bbox[..., :1])
            horizontal = torch_module.softmax(
                torch_module.cat(
                    (raw_bbox[..., 0:2], zero_margin_logit),
                    dim=-1,
                ),
                dim=-1,
            )
            vertical = torch_module.softmax(
                torch_module.cat(
                    (raw_bbox[..., 2:4], zero_margin_logit),
                    dim=-1,
                ),
                dim=-1,
            )
            horizontal = epsilon + (1.0 - 3.0 * epsilon) * horizontal
            vertical = epsilon + (1.0 - 3.0 * epsilon) * vertical
            bbox = torch_module.stack(
                (
                    horizontal[..., 0],
                    vertical[..., 0],
                    horizontal[..., 1],
                    vertical[..., 1],
                ),
                dim=-1,
            )
            s_ratio = torch_module.sigmoid(self.s_ratio_projection(slots))
            return {
                'segment_logits': self.segment_projection(slots),
                'loaded_logits': self.loaded_projection(slots),
                'bbox': bbox,
                's_ratio': epsilon + (1.0 - 2.0 * epsilon) * s_ratio,
            }

    class SpatialIdentityQueryHead(nn.Module):
        """Decode four fixed identities by cross-attending to spatial tokens."""

        def __init__(self, input_channels: int) -> None:
            super().__init__()
            self.input_projection = nn.Conv2d(
                int(input_channels),
                model_dim,
                kernel_size=1,
            )
            self.identity_queries = nn.Parameter(
                torch_module.empty(V4_SLOTS_PER_SIDE, model_dim)
            )
            nn.init.normal_(self.identity_queries, mean=0.0, std=0.02)
            self.cross_attention = nn.MultiheadAttention(
                model_dim,
                number_of_heads,
                dropout=dropout_probability,
                batch_first=True,
            )
            self.attention_norm = nn.LayerNorm(model_dim)
            self.feed_forward = nn.Sequential(
                nn.Linear(model_dim, model_dim * 4),
                nn.GELU(),
                nn.Dropout(dropout_probability),
                nn.Linear(model_dim * 4, model_dim),
            )
            self.output_norm = nn.LayerNorm(model_dim)
            self.output_projection = StructuredOutputProjection()

        def forward(self, feature_map):
            projected = self.input_projection(feature_map)
            spatial_tokens = projected.flatten(2).transpose(1, 2)
            spatial_tokens = spatial_tokens + sine_position_encoding(projected)
            queries = self.identity_queries.unsqueeze(0).expand(
                int(projected.shape[0]),
                -1,
                -1,
            )
            attended, _ = self.cross_attention(
                queries,
                spatial_tokens,
                spatial_tokens,
                need_weights=False,
            )
            slots = self.attention_norm(queries + attended)
            slots = self.output_norm(slots + self.feed_forward(slots))
            return self.output_projection(slots)

    class GlobalIdentityHead(nn.Module):
        """Non-spatial four-slot head retained solely as a controlled ablation."""

        def __init__(self, input_channels: int) -> None:
            super().__init__()
            self.pool = nn.AdaptiveAvgPool2d((1, 1))
            self.feature_projection = nn.Linear(int(input_channels), model_dim)
            self.identity_queries = nn.Parameter(
                torch_module.empty(V4_SLOTS_PER_SIDE, model_dim)
            )
            nn.init.normal_(self.identity_queries, mean=0.0, std=0.02)
            self.slot_mlp = nn.Sequential(
                nn.LayerNorm(model_dim),
                nn.Linear(model_dim, model_dim * 2),
                nn.GELU(),
                nn.Dropout(dropout_probability),
                nn.Linear(model_dim * 2, model_dim),
                nn.LayerNorm(model_dim),
            )
            self.output_projection = StructuredOutputProjection()

        def forward(self, feature_map):
            pooled = self.pool(feature_map).flatten(1)
            shared_feature = self.feature_projection(pooled).unsqueeze(1)
            slots = shared_feature + self.identity_queries.unsqueeze(0)
            return self.output_projection(self.slot_mlp(slots))

    def make_head(input_channels: int):
        if normalized_head_type == V4_HEAD_SPATIAL_QUERY:
            return SpatialIdentityQueryHead(input_channels)
        return GlobalIdentityHead(input_channels)

    class Room315VisualStateModelV4(nn.Module):
        """Shared low-level encoder with strictly rail-local late computation."""

        def __init__(self) -> None:
            super().__init__()
            # weights=None is intentional: construction must be offline-safe.
            resnet = torchvision_module.models.resnet18(weights=None)
            feature_channels = int(resnet.fc.in_features)
            self.shared_stem = nn.Sequential(OrderedDict((
                ('conv1', resnet.conv1),
                ('bn1', resnet.bn1),
                ('relu', resnet.relu),
                ('maxpool', resnet.maxpool),
                ('layer1', resnet.layer1),
                ('layer2', resnet.layer2),
                ('layer3', resnet.layer3),
            )))
            # Deep copies include distinct affine parameters and running BN
            # buffers.  Neither branch is called from the other branch.
            self.left_layer4 = copy.deepcopy(resnet.layer4)
            self.right_layer4 = copy.deepcopy(resnet.layer4)
            self.left_head = make_head(feature_channels)
            self.right_head = make_head(feature_channels)
            self.model_kind = V4_MODEL_KIND
            self.head_type = normalized_head_type
            self.slot_order = V4_SLOT_ORDER
            # Apply the V4 BatchNorm policy immediately; nn.Module defaults to
            # training mode during construction.
            self.train(True)

        def _rail_features(self, rgb, rail_layer4):
            return rail_layer4(self.shared_stem(rgb))

        def train(self, train_mode: bool = True):
            """Keep shared BN statistics frozen while training rail-local BN.

            ``eval()`` changes only BatchNorm behavior here.  It does not
            change ``requires_grad``, so shared affine parameters and all
            shared convolution/layer3 parameters remain trainable when their
            optimizer group enables them.
            """

            super().train(train_mode)
            for module in self.shared_stem.modules():
                if isinstance(module, nn.modules.batchnorm._BatchNorm):
                    module.eval()
            return self

        def forward(self, image):
            if image.ndim != 4 or int(image.shape[1]) != 6:
                raise ValueError(
                    'V4 expects paired RGB input shaped [B, 6, H, W] with '
                    f'left/right channel order; got {tuple(image.shape)}'
                )
            left_features = self._rail_features(
                image[:, :3],
                self.left_layer4,
            )
            right_features = self._rail_features(
                image[:, 3:],
                self.right_layer4,
            )
            left = self.left_head(left_features)
            right = self.right_head(right_features)
            return {
                key: torch_module.cat((left[key], right[key]), dim=1)
                for key in V4_OUTPUT_KEYS
            }

    return Room315VisualStateModelV4()
