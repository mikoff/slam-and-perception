"""A three-level, quantization-friendly feature pyramid."""

from __future__ import annotations

import torch.nn.functional as functional
from torch import Tensor, nn


class DepthwiseSeparableConv(nn.Sequential):
    """Depthwise 3x3 followed by pointwise Conv-BN-ReLU."""

    def __init__(self, in_channels: int, out_channels: int) -> None:
        super().__init__(
            nn.Conv2d(
                in_channels,
                in_channels,
                kernel_size=3,
                padding=1,
                groups=in_channels,
                bias=False,
            ),
            nn.Conv2d(in_channels, out_channels, kernel_size=1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
        )


class LiteFPN(nn.Module):
    """Top-down C3-C5 pyramid with 96-channel outputs."""

    def __init__(
        self,
        in_channels: tuple[int, int, int] = (80, 160, 256),
        out_channels: int = 96,
    ) -> None:
        super().__init__()
        self.lateral3 = nn.Conv2d(in_channels[0], out_channels, kernel_size=1)
        self.lateral4 = nn.Conv2d(in_channels[1], out_channels, kernel_size=1)
        self.lateral5 = nn.Conv2d(in_channels[2], out_channels, kernel_size=1)
        self.output3 = DepthwiseSeparableConv(out_channels, out_channels)
        self.output4 = DepthwiseSeparableConv(out_channels, out_channels)
        self.output5 = DepthwiseSeparableConv(out_channels, out_channels)

    def forward(
        self, features: tuple[Tensor, Tensor, Tensor]
    ) -> tuple[Tensor, Tensor, Tensor]:
        c3, c4, c5 = features
        lateral5 = self.lateral5(c5)
        lateral4 = self.lateral4(c4) + functional.interpolate(
            lateral5, size=c4.shape[-2:], mode="nearest"
        )
        lateral3 = self.lateral3(c3) + functional.interpolate(
            lateral4, size=c3.shape[-2:], mode="nearest"
        )
        return (
            self.output3(lateral3),
            self.output4(lateral4),
            self.output5(lateral5),
        )
