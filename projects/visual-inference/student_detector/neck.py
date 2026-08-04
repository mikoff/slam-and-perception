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


class AttnResLiteFPN(nn.Module):
    """Three-level feature pyramid with dynamic Softmax depth-selection weights across levels."""

    def __init__(
        self,
        in_channels: tuple[int, int, int] = (80, 160, 256),
        out_channels: int = 96,
    ) -> None:
        super().__init__()
        self.lateral3 = nn.Conv2d(in_channels[0], out_channels, kernel_size=1)
        self.lateral4 = nn.Conv2d(in_channels[1], out_channels, kernel_size=1)
        self.lateral5 = nn.Conv2d(in_channels[2], out_channels, kernel_size=1)
        
        # 1x1 Conv depth projections for spatial dynamic weighting
        self.attn3 = nn.Conv2d(out_channels, 3, kernel_size=1)
        self.attn4 = nn.Conv2d(out_channels, 3, kernel_size=1)
        self.attn5 = nn.Conv2d(out_channels, 3, kernel_size=1)

        self.output3 = DepthwiseSeparableConv(out_channels, out_channels)
        self.output4 = DepthwiseSeparableConv(out_channels, out_channels)
        self.output5 = DepthwiseSeparableConv(out_channels, out_channels)

    def _fuse_level(
        self,
        attn_conv: nn.Conv2d,
        l3: Tensor,
        l4: Tensor,
        l5: Tensor,
        target_size: tuple[int, int],
    ) -> Tensor:
        # Resize all lateral maps to target spatial size
        m3 = l3 if l3.shape[-2:] == target_size else functional.interpolate(l3, size=target_size, mode="nearest")
        m4 = l4 if l4.shape[-2:] == target_size else functional.interpolate(l4, size=target_size, mode="nearest")
        m5 = l5 if l5.shape[-2:] == target_size else functional.interpolate(l5, size=target_size, mode="nearest")

        # Dynamic Softmax depth weights per spatial cell
        weights = functional.softmax(attn_conv(m3 + m4 + m5), dim=1)  # [B, 3, H, W]
        w3, w4, w5 = weights.unbind(dim=1)
        fused = w3.unsqueeze(1) * m3 + w4.unsqueeze(1) * m4 + w5.unsqueeze(1) * m5
        return fused

    def forward(
        self, features: tuple[Tensor, Tensor, Tensor]
    ) -> tuple[Tensor, Tensor, Tensor]:
        c3, c4, c5 = features
        l3 = self.lateral3(c3)
        l4 = self.lateral4(c4)
        l5 = self.lateral5(c5)

        f3 = self._fuse_level(self.attn3, l3, l4, l5, l3.shape[-2:])
        f4 = self._fuse_level(self.attn4, l3, l4, l5, l4.shape[-2:])
        f5 = self._fuse_level(self.attn5, l3, l4, l5, l5.shape[-2:])

        return (
            self.output3(f3),
            self.output4(f4),
            self.output5(f5),
        )

