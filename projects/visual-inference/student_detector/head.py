"""Shared class-agnostic FCOS-style prediction head."""

from __future__ import annotations

import math
from typing import NamedTuple

import torch
from torch import Tensor, nn

from .neck import DepthwiseSeparableConv


class DetectorOutput(NamedTuple):
    """Raw, fixed-shape outputs ordered as P3, P4 and P5."""

    objectness: tuple[Tensor, Tensor, Tensor]
    box_distances: tuple[Tensor, Tensor, Tensor]
    centerness: tuple[Tensor, Tensor, Tensor]


class QuadDetectorOutput(NamedTuple):
    """Raw fixed-shape outputs for the class-agnostic quad detector."""

    quality: tuple[Tensor, Tensor, Tensor]
    corner_offsets: tuple[Tensor, Tensor, Tensor]


class SharedDetectionHead(nn.Module):
    """One shared object tower and one shared regression tower."""

    def __init__(
        self,
        channels: int = 96,
        strides: tuple[int, int, int] = (8, 16, 32),
        prior_probability: float = 0.01,
    ) -> None:
        super().__init__()
        self.object_tower = DepthwiseSeparableConv(channels, channels)
        # Stage-0 diagnostics isolated localization as the primary recall
        # ceiling. Keep the shared, quantization-friendly design but give the
        # regression path one additional lightweight nonlinear refinement.
        self.regression_tower = nn.Sequential(
            DepthwiseSeparableConv(channels, channels),
            DepthwiseSeparableConv(channels, channels),
        )
        self.objectness = nn.Conv2d(channels, 1, kernel_size=1)
        self.box_regression = nn.Conv2d(channels, 4, kernel_size=1)
        self.centerness = nn.Conv2d(channels, 1, kernel_size=1)
        self.scales = nn.Parameter(torch.ones(len(strides)))
        self.strides = strides

        objectness_bias = -math.log((1.0 - prior_probability) / prior_probability)
        nn.init.constant_(self.objectness.bias, objectness_bias)
        # ReLU distance decoding needs a positive starting point. Unlike FCOS's
        # exponential transform, a default near-zero logit can create zero-area
        # boxes and block the gradient on negative regression outputs.
        nn.init.constant_(self.box_regression.bias, 1.0)
        nn.init.zeros_(self.centerness.bias)

    def _forward_level(
        self, feature: Tensor, level: int
    ) -> tuple[Tensor, Tensor, Tensor]:
        object_feature = self.object_tower(feature)
        regression_feature = self.regression_tower(feature)
        objectness = self.objectness(object_feature)
        raw_distance = self.box_regression(regression_feature)
        distance = torch.relu(self.scales[level] * raw_distance) * self.strides[level]
        centerness = self.centerness(regression_feature)
        return objectness, distance, centerness

    def forward(self, features: tuple[Tensor, Tensor, Tensor]) -> DetectorOutput:
        level3 = self._forward_level(features[0], 0)
        level4 = self._forward_level(features[1], 1)
        level5 = self._forward_level(features[2], 2)
        return DetectorOutput(
            objectness=(level3[0], level4[0], level5[0]),
            box_distances=(level3[1], level4[1], level5[1]),
            centerness=(level3[2], level4[2], level5[2]),
        )


class SharedQuadProposalHead(nn.Module):
    """One shared quality tower and one unmasked eight-coordinate tower."""

    def __init__(
        self,
        channels: int = 96,
        strides: tuple[int, int, int] = (8, 16, 32),
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
        quality_bias = -math.log((1.0 - prior_probability) / prior_probability)
        nn.init.constant_(self.quality.bias, quality_bias)
        # Initialize corner offset bias with explicit HBB grid prior:
        # P0=(-1,-1), P1=(+1,-1), P2=(+1,+1), P3=(-1,+1) in stride units
        hbb_prior_bias = torch.tensor([-1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0])
        self.corner_offsets.bias.data.copy_(hbb_prior_bias)
        nn.init.zeros_(self.corner_offsets.weight)

    def _forward_level(self, feature: Tensor) -> tuple[Tensor, Tensor]:
        quality = self.quality(self.quality_tower(feature))
        # Direct signed stride-normalized corner offsets relative to grid point.
        offsets = self.corner_offsets(self.geometry_tower(feature))
        return quality, offsets

    def forward(self, features: tuple[Tensor, Tensor, Tensor]) -> QuadDetectorOutput:
        levels = tuple(self._forward_level(feature) for feature in features)
        return QuadDetectorOutput(
            quality=tuple(level[0] for level in levels),
            corner_offsets=tuple(level[1] for level in levels),
        )


