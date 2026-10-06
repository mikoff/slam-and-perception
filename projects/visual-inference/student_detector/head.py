"""Shared class-agnostic FCOS-style prediction head."""

from __future__ import annotations

import math
from typing import NamedTuple

import torch
from torch import Tensor, nn

from .neck import DepthwiseSeparableConv


class DetectorOutput(NamedTuple):
    """Raw fixed-shape outputs ordered by configured pyramid stride."""

    objectness: tuple[Tensor, ...]
    box_distances: tuple[Tensor, ...]
    centerness: tuple[Tensor, ...]


class QuadDetectorOutput(NamedTuple):
    """Raw fixed-shape outputs for the class-agnostic quad detector."""

    quality: tuple[Tensor, ...]
    corner_offsets: tuple[Tensor, ...]


class SharedDetectionHead(nn.Module):
    """One shared object tower and one shared regression tower."""

    def __init__(
        self,
        channels: int = 96,
        strides: tuple[int, ...] = (8, 16, 32),
        prior_probability: float = 0.01,
        use_dfl: bool = False,
        reg_max: int = 16,
    ) -> None:
        super().__init__()
        self.use_dfl = use_dfl
        self.reg_max = reg_max
        self.object_tower = DepthwiseSeparableConv(channels, channels)
        # Stage-0 diagnostics isolated localization as the primary recall
        # ceiling. Keep the shared, quantization-friendly design but give the
        # regression path one additional lightweight nonlinear refinement.
        self.regression_tower = nn.Sequential(
            DepthwiseSeparableConv(channels, channels),
            DepthwiseSeparableConv(channels, channels),
        )
        self.objectness = nn.Conv2d(channels, 1, kernel_size=1)
        reg_channels = 4 * reg_max if use_dfl else 4
        self.box_regression = nn.Conv2d(channels, reg_channels, kernel_size=1)
        self.centerness = nn.Conv2d(channels, 1, kernel_size=1)
        self.scales = nn.Parameter(torch.ones(len(strides)))
        self.strides = strides

        objectness_bias = -math.log((1.0 - prior_probability) / prior_probability)
        nn.init.constant_(self.objectness.bias, objectness_bias)
        # ReLU distance decoding needs a positive starting point. Unlike FCOS's
        # exponential transform, a default near-zero logit can create zero-area
        # boxes and block the gradient on negative regression outputs.
        if not use_dfl:
            nn.init.constant_(self.box_regression.bias, 1.0)
        else:
            nn.init.zeros_(self.box_regression.bias)
        nn.init.zeros_(self.centerness.bias)

    def _forward_level(
        self, feature: Tensor, level: int
    ) -> tuple[Tensor, Tensor, Tensor]:
        object_feature = self.object_tower(feature)
        regression_feature = self.regression_tower(feature)
        objectness = self.objectness(object_feature)
        raw_regression = self.box_regression(regression_feature)
        if self.use_dfl:
            b, _, h, w = raw_regression.shape
            prob = torch.softmax(
                raw_regression.view(b, 4, self.reg_max, h, w), dim=2
            )
            bins = torch.arange(
                self.reg_max, device=feature.device, dtype=feature.dtype
            ).view(1, 1, self.reg_max, 1, 1)
            expected = (prob * bins).sum(dim=2)
            distance = expected * self.scales[level] * self.strides[level]
        else:
            distance = (
                torch.relu(self.scales[level] * raw_regression) * self.strides[level]
            )
        centerness = self.centerness(regression_feature)
        return objectness, distance, centerness

    def forward(self, features: tuple[Tensor, ...]) -> DetectorOutput:
        if len(features) != len(self.strides):
            raise ValueError("feature count must match configured strides")
        levels = tuple(
            self._forward_level(feature, level)
            for level, feature in enumerate(features)
        )
        return DetectorOutput(
            objectness=tuple(level[0] for level in levels),
            box_distances=tuple(level[1] for level in levels),
            centerness=tuple(level[2] for level in levels),
        )


class SharedQuadProposalHead(nn.Module):
    """One shared quality tower and one unmasked eight-coordinate tower."""

    def __init__(
        self,
        channels: int = 96,
        strides: tuple[int, ...] = (8, 16, 32),
        prior_probability: float = 0.01,
    ) -> None:
        super().__init__()
        self.quality_tower = DepthwiseSeparableConv(channels, channels)
        self.geometry_tower = nn.Sequential(
            DepthwiseSeparableConv(channels, channels),
            DepthwiseSeparableConv(channels, channels),
        )
        self.quality = nn.Conv2d(channels, 1, kernel_size=1)
        self.corner_offsets = nn.Conv2d(channels, 8, kernel_size=1)
        self.strides = strides
        self.scales = nn.Parameter(torch.ones(len(strides)))
        quality_bias = -math.log((1.0 - prior_probability) / prior_probability)
        nn.init.constant_(self.quality.bias, quality_bias)
        # Initialize corner offset bias with explicit HBB grid prior:
        # P0=(-1,-1), P1=(+1,-1), P2=(+1,+1), P3=(-1,+1) in stride units
        hbb_prior_bias = torch.tensor([-1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0])
        self.corner_offsets.bias.data.copy_(hbb_prior_bias)
        nn.init.zeros_(self.corner_offsets.weight)

    def _forward_level(self, feature: Tensor, level: int) -> tuple[Tensor, Tensor]:
        quality = self.quality(self.quality_tower(feature))
        # Direct signed stride-normalized corner offsets relative to grid point scaled per level.
        offsets = self.corner_offsets(self.geometry_tower(feature)) * self.scales[level]
        return quality, offsets

    def forward(self, features: tuple[Tensor, ...]) -> QuadDetectorOutput:
        if len(features) != len(self.strides):
            raise ValueError("feature count must match configured strides")
        levels = tuple(
            self._forward_level(feature, level)
            for level, feature in enumerate(features)
        )
        return QuadDetectorOutput(
            quality=tuple(level[0] for level in levels),
            corner_offsets=tuple(level[1] for level in levels),
        )
