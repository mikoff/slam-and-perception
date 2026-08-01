"""End-to-end raw student detector network."""

from __future__ import annotations

from torch import Tensor, nn

from .backbone import MobileNetV4Backbone
from .head import DetectorOutput, SharedDetectionHead
from .neck import LiteFPN


class StudentDetector(nn.Module):
    """MobileNetV4 + Lite FPN + shared class-agnostic detector head."""

    def __init__(
        self,
        *,
        pretrained_backbone: bool = True,
        backbone: nn.Module | None = None,
        fpn_channels: int = 96,
    ) -> None:
        super().__init__()
        if backbone is None:
            backbone = MobileNetV4Backbone(pretrained=pretrained_backbone)
        if not hasattr(backbone, "out_channels"):
            raise TypeError("backbone must expose an out_channels tuple")

        self.backbone = backbone
        self.fpn = LiteFPN(tuple(backbone.out_channels), fpn_channels)  # type: ignore[arg-type]
        self.head = SharedDetectionHead(fpn_channels)

    def forward_features(self, images: Tensor) -> tuple[Tensor, Tensor, Tensor]:
        return self.fpn(self.backbone(images))  # type: ignore[arg-type]

    def forward(self, images: Tensor) -> DetectorOutput:
        return self.head(self.forward_features(images))


__all__ = ["DetectorOutput", "StudentDetector"]
