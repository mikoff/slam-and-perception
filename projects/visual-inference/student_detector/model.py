"""End-to-end raw student detector network."""

from __future__ import annotations

from typing import Protocol

from torch import Tensor, nn

from .backbone import MobileNetV4Backbone
from .head import DetectorOutput, SharedDetectionHead
from .neck import LiteFPN


class SemanticHead(Protocol):
    """Future Phase-3 interface; implementations produce 256-D vectors."""

    def __call__(
        self, p3: Tensor, p4: Tensor, p5: Tensor
    ) -> tuple[Tensor, Tensor, Tensor]: ...


class StudentDetector(nn.Module):
    """MobileNetV4 + Lite FPN + shared class-agnostic detector head."""

    semantic_head: nn.Module | None

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
        # Reserved extension point. It deliberately does not participate in forward
        # until Phase 3, keeping the Phase-2 export signature stable.
        self.semantic_head = None

    def forward_features(self, images: Tensor) -> tuple[Tensor, Tensor, Tensor]:
        return self.fpn(self.backbone(images))  # type: ignore[arg-type]

    def forward_semantics(
        self, pyramid: tuple[Tensor, Tensor, Tensor]
    ) -> tuple[Tensor, Tensor, Tensor] | None:
        """Run a future 256-D semantic head without changing backbone/FPN APIs."""
        if self.semantic_head is None:
            return None
        return self.semantic_head(*pyramid)  # type: ignore[no-any-return]

    def forward(self, images: Tensor) -> DetectorOutput:
        return self.head(self.forward_features(images))


__all__ = ["DetectorOutput", "SemanticHead", "StudentDetector"]
