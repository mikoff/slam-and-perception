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
        self.regression_tower = DepthwiseSeparableConv(channels, channels)
        self.objectness = nn.Conv2d(channels, 1, kernel_size=1)
        self.box_regression = nn.Conv2d(channels, 4, kernel_size=1)
        self.centerness = nn.Conv2d(channels, 1, kernel_size=1)
        self.scales = nn.Parameter(torch.ones(len(strides)))
        self.strides = strides

        objectness_bias = -math.log((1.0 - prior_probability) / prior_probability)
        nn.init.constant_(self.objectness.bias, objectness_bias)

    def _forward_level(self, feature: Tensor, level: int) -> tuple[Tensor, Tensor, Tensor]:
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
