#!/usr/bin/env python3
"""Shared Room 315 paired-camera ResNet-18 model definition.

This module has no ROS or training CLI dependency.  Both training and runtime
construct the exact same architecture through ``build_visual_state_model``.
"""

from __future__ import annotations

import math
from typing import Any


VISUAL_ADAPTATION_FROZEN_BACKBONE = 'frozen_backbone'
VISUAL_ADAPTATION_LORA = 'lora'
VISUAL_ADAPTATION_PARTIAL_FINETUNE = 'partial_finetune'
SUPPORTED_VISUAL_ADAPTATIONS = frozenset({
    VISUAL_ADAPTATION_FROZEN_BACKBONE,
    VISUAL_ADAPTATION_LORA,
    VISUAL_ADAPTATION_PARTIAL_FINETUNE,
})


def build_visual_state_model(
    torch_module: Any,
    torchvision_module: Any,
    *,
    output_dim: int,
    adaptation_mode: str,
    lora_rank: int = 4,
):
    """Build the architecture used by the approved fixed-eight checkpoint."""

    mode = str(adaptation_mode or '').strip().lower()
    if mode not in SUPPORTED_VISUAL_ADAPTATIONS:
        raise ValueError(f'unsupported visual adaptation mode: {mode!r}')
    if int(output_dim) <= 0:
        raise ValueError('output_dim must be positive')

    nn = torch_module.nn
    rank = max(1, int(lora_rank))
    per_camera_feature_dim = 512
    paired_feature_dim = per_camera_feature_dim * 2

    class VisualStateResNet18WithAdaptation(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.backbone = torchvision_module.models.resnet18(weights=None)
            self.backbone.fc = nn.Identity()
            for parameter in self.backbone.parameters():
                parameter.requires_grad = False
            self.adaptation_mode = mode
            if mode == VISUAL_ADAPTATION_LORA:
                self.lora_down = nn.Linear(paired_feature_dim, rank, bias=False)
                self.lora_up = nn.Linear(rank, paired_feature_dim, bias=False)
                nn.init.kaiming_uniform_(self.lora_down.weight, a=math.sqrt(5))
                nn.init.zeros_(self.lora_up.weight)
            else:
                self.lora_down = None
                self.lora_up = None
            if mode == VISUAL_ADAPTATION_PARTIAL_FINETUNE:
                for parameter in self.backbone.layer4.parameters():
                    parameter.requires_grad = True
            self.head = nn.Sequential(
                nn.LayerNorm(paired_feature_dim),
                nn.Linear(paired_feature_dim, 128),
                nn.SiLU(inplace=True),
                nn.Dropout(0.05),
                nn.Linear(128, int(output_dim)),
            )

        def forward(self, image):
            if image.ndim != 4 or image.shape[1] != 6:
                raise ValueError(
                    'expected paired RGB input shaped (N, 6, H, W), '
                    f'got {tuple(image.shape)}'
                )
            left_features = self.backbone(image[:, :3])
            right_features = self.backbone(image[:, 3:])
            features = torch_module.cat([left_features, right_features], dim=1)
            if self.lora_down is not None and self.lora_up is not None:
                features = (
                    features
                    + self.lora_up(self.lora_down(features)) / float(rank)
                )
            return self.head(features)

        def train(self, train_mode: bool = True):
            super().train(train_mode)
            if train_mode and self.adaptation_mode in {
                VISUAL_ADAPTATION_FROZEN_BACKBONE,
                VISUAL_ADAPTATION_LORA,
            }:
                self.backbone.eval()
            elif (
                train_mode
                and self.adaptation_mode == VISUAL_ADAPTATION_PARTIAL_FINETUNE
            ):
                self.backbone.eval()
                self.backbone.layer4.train()
            return self

    return VisualStateResNet18WithAdaptation()
