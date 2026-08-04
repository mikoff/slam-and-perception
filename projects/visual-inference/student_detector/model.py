"""End-to-end raw student detector network."""

from __future__ import annotations

from torch import Tensor, nn

from .backbone import MobileNetV4Backbone
from .head import (
    DetectorOutput,
    QuadDetectorOutput,
    SharedDetectionHead,
    SharedQuadProposalHead,
)
from .neck import AttnResLiteFPN, LiteFPN


class StudentDetector(nn.Module):
    """MobileNetV4 + Lite FPN + shared class-agnostic detector head."""

    def __init__(
        self,
        *,
        pretrained_backbone: bool = True,
        backbone: nn.Module | None = None,
        fpn_channels: int = 96,
        neck_type: str = "lite",
    ) -> None:
        super().__init__()
        if backbone is None:
            backbone = MobileNetV4Backbone(pretrained=pretrained_backbone)
        if not hasattr(backbone, "out_channels"):
            raise TypeError("backbone must expose an out_channels tuple")

        self.backbone = backbone
        if neck_type == "attn_res":
            self.fpn = AttnResLiteFPN(tuple(backbone.out_channels), fpn_channels)  # type: ignore[arg-type]
        elif neck_type == "lite":
            self.fpn = LiteFPN(tuple(backbone.out_channels), fpn_channels)  # type: ignore[arg-type]
        else:
            raise ValueError(f"Unknown neck_type: {neck_type}. Must be 'lite' or 'attn_res'")
        self.head = SharedDetectionHead(fpn_channels)

    def forward_features(self, images: Tensor) -> tuple[Tensor, Tensor, Tensor]:
        return self.fpn(self.backbone(images))  # type: ignore[arg-type]

    def forward(self, images: Tensor) -> DetectorOutput:
        return self.head(self.forward_features(images))


class QuadProposalDetector(nn.Module):
    """MobileNetV4 + Lite FPN + class-agnostic quadrilateral proposal head."""

    def __init__(
        self,
        *,
        pretrained_backbone: bool = True,
        backbone: nn.Module | None = None,
        fpn_channels: int = 96,
        neck_type: str = "lite",
    ) -> None:
        super().__init__()
        if backbone is None:
            backbone = MobileNetV4Backbone(pretrained=pretrained_backbone)
        if not hasattr(backbone, "out_channels"):
            raise TypeError("backbone must expose an out_channels tuple")
        self.backbone = backbone
        if neck_type == "attn_res":
            self.fpn = AttnResLiteFPN(tuple(backbone.out_channels), fpn_channels)  # type: ignore[arg-type]
        elif neck_type == "lite":
            self.fpn = LiteFPN(tuple(backbone.out_channels), fpn_channels)  # type: ignore[arg-type]
        else:
            raise ValueError(f"Unknown neck_type: {neck_type}. Must be 'lite' or 'attn_res'")
        self.head = SharedQuadProposalHead(fpn_channels)

    def forward_features(self, images: Tensor) -> tuple[Tensor, Tensor, Tensor]:
        return self.fpn(self.backbone(images))  # type: ignore[arg-type]

    def forward(self, images: Tensor) -> QuadDetectorOutput:
        return self.head(self.forward_features(images))


QuadStudentDetector = QuadProposalDetector
QuadDetector = QuadProposalDetector


__all__ = [
    "DetectorOutput",
    "QuadDetector",
    "QuadDetectorOutput",
    "QuadProposalDetector",
    "QuadStudentDetector",
    "StudentDetector",
]
