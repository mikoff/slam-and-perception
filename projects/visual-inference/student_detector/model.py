"""End-to-end raw student detector network."""

from __future__ import annotations

import torch
from torch import Tensor, nn

from .backbone import MobileNetV4Backbone
from .head import (
    DetectorOutput,
    QuadDetectorOutput,
    SharedDetectionHead,
    SharedQuadProposalHead,
)
from .neck import AttnResLiteFPN, LiteFPN, P2LiteFPN, P2LitePAN


class StudentDetector(nn.Module):
    """MobileNetV4 + Lite FPN/PAN + shared class-agnostic detector head."""

    def __init__(
        self,
        *,
        pretrained_backbone: bool = True,
        backbone: nn.Module | None = None,
        fpn_channels: int = 96,
        neck_type: str = "lite",
        strides: tuple[int, ...] = (8, 16, 32),
        head_seed: int | None = None,
        dual_head: bool = False,
        use_dfl: bool = False,
        reg_max: int = 16,
        use_3dmf: bool = False,
    ) -> None:
        super().__init__()
        if backbone is None:
            backbone = MobileNetV4Backbone(
                pretrained=pretrained_backbone, desired_reductions=strides
            )
        if not hasattr(backbone, "out_channels"):
            raise TypeError("backbone must expose an out_channels tuple")

        self.backbone = backbone
        self.dual_head = dual_head
        self.fpn_channels = fpn_channels
        self.neck_type = neck_type
        self.strides = strides
        self.use_3dmf = use_3dmf
        if tuple(backbone.reductions) != strides:
            raise ValueError("backbone reductions must match detector strides")
        if neck_type == "pan" and strides in ((8, 16, 32), (4, 8, 16, 32)):
            self.fpn: nn.Module = P2LitePAN(tuple(backbone.out_channels), fpn_channels)
        elif strides == (4, 8, 16, 32) and neck_type == "lite":
            self.fpn = P2LiteFPN(tuple(backbone.out_channels), fpn_channels)  # type: ignore[arg-type]
        elif neck_type == "attn_res":
            if strides != (8, 16, 32):
                raise ValueError("attn_res supports only P3-P5")
            self.fpn = AttnResLiteFPN(tuple(backbone.out_channels), fpn_channels)  # type: ignore[arg-type]
        elif neck_type == "lite" and strides == (8, 16, 32):
            self.fpn = LiteFPN(tuple(backbone.out_channels), fpn_channels)  # type: ignore[arg-type]
        else:
            raise ValueError(
                f"unsupported neck/stride combination: {neck_type}/{strides}"
            )
        if head_seed is None:
            self.head = SharedDetectionHead(
                fpn_channels,
                strides=strides,
                use_dfl=use_dfl,
                reg_max=reg_max,
                use_3dmf=use_3dmf,
            )
            self.aux_head = (
                SharedDetectionHead(
                    fpn_channels,
                    strides=strides,
                    use_dfl=use_dfl,
                    reg_max=reg_max,
                    use_3dmf=False,
                )
                if dual_head
                else None
            )
        else:
            with torch.random.fork_rng():
                torch.manual_seed(head_seed)
                self.head = SharedDetectionHead(
                    fpn_channels,
                    strides=strides,
                    use_dfl=use_dfl,
                    reg_max=reg_max,
                    use_3dmf=use_3dmf,
                )
                self.aux_head = (
                    SharedDetectionHead(
                        fpn_channels,
                        strides=strides,
                        use_dfl=use_dfl,
                        reg_max=reg_max,
                        use_3dmf=False,
                    )
                    if dual_head
                    else None
                )

    def forward_features(self, images: Tensor) -> tuple[Tensor, ...]:
        return self.fpn(self.backbone(images))  # type: ignore[arg-type]

    def forward_eval_head(
        self, features: tuple[Tensor, ...], aux_weight: float = 0.0
    ) -> DetectorOutput:
        """Evaluate primary 1-to-1 + 3DMF head (optionally fused with auxiliary objectness)."""
        primary = self.head(features)
        if not self.dual_head or self.aux_head is None or aux_weight <= 0.0:
            return primary
        aux = self.aux_head(features)
        w_p = 1.0 - aux_weight
        blended_obj = tuple(
            w_p * p_o + aux_weight * a_o
            for p_o, a_o in zip(primary.objectness, aux.objectness, strict=True)
        )
        return DetectorOutput(
            objectness=blended_obj,
            box_distances=primary.box_distances,
            centerness=primary.centerness,
        )

    def forward(
        self, images: Tensor
    ) -> DetectorOutput | tuple[DetectorOutput, DetectorOutput]:
        features = self.forward_features(images)
        primary = self.head(features)
        if self.training and self.dual_head and self.aux_head is not None:
            return primary, self.aux_head(features)
        return primary


class QuadProposalDetector(nn.Module):
    """MobileNetV4 + Lite FPN/PAN + class-agnostic quadrilateral proposal head."""

    def __init__(
        self,
        *,
        pretrained_backbone: bool = True,
        backbone: nn.Module | None = None,
        fpn_channels: int = 96,
        neck_type: str = "lite",
        strides: tuple[int, ...] = (8, 16, 32),
        dual_head: bool = False,
        use_3dmf: bool = False,
    ) -> None:
        super().__init__()
        if backbone is None:
            backbone = MobileNetV4Backbone(
                pretrained=pretrained_backbone, desired_reductions=strides
            )
        if not hasattr(backbone, "out_channels"):
            raise TypeError("backbone must expose an out_channels tuple")
        self.backbone = backbone
        self.dual_head = dual_head
        self.fpn_channels = fpn_channels
        self.neck_type = neck_type
        self.strides = strides
        self.use_3dmf = use_3dmf
        if tuple(backbone.reductions) != strides:
            raise ValueError("backbone reductions must match detector strides")
        if neck_type == "pan" and strides in ((8, 16, 32), (4, 8, 16, 32)):
            self.fpn: nn.Module = P2LitePAN(tuple(backbone.out_channels), fpn_channels)
        elif strides == (4, 8, 16, 32) and neck_type == "lite":
            self.fpn = P2LiteFPN(tuple(backbone.out_channels), fpn_channels)  # type: ignore[arg-type]
        elif neck_type == "attn_res":
            if strides != (8, 16, 32):
                raise ValueError("attn_res supports only P3-P5")
            self.fpn = AttnResLiteFPN(tuple(backbone.out_channels), fpn_channels)  # type: ignore[arg-type]
        elif neck_type == "lite" and strides == (8, 16, 32):
            self.fpn = LiteFPN(tuple(backbone.out_channels), fpn_channels)  # type: ignore[arg-type]
        else:
            raise ValueError(
                f"unsupported neck/stride combination: {neck_type}/{strides}"
            )
        self.head = SharedQuadProposalHead(
            fpn_channels, strides=strides, use_3dmf=use_3dmf
        )
        self.aux_head = (
            SharedQuadProposalHead(fpn_channels, strides=strides, use_3dmf=False)
            if dual_head
            else None
        )

    def forward_features(self, images: Tensor) -> tuple[Tensor, ...]:
        return self.fpn(self.backbone(images))  # type: ignore[arg-type]

    def forward_eval_head(
        self, features: tuple[Tensor, ...], aux_weight: float = 0.0
    ) -> QuadDetectorOutput:
        """Evaluate primary 1-to-1 + 3DMF quad head (optionally fused with auxiliary quality)."""
        primary = self.head(features)
        if not self.dual_head or self.aux_head is None or aux_weight <= 0.0:
            return primary
        aux = self.aux_head(features)
        w_p = 1.0 - aux_weight
        blended_q = tuple(
            w_p * p_q + aux_weight * a_q
            for p_q, a_q in zip(primary.quality, aux.quality, strict=True)
        )
        return QuadDetectorOutput(
            quality=blended_q,
            corner_offsets=primary.corner_offsets,
        )

    def forward(
        self, images: Tensor
    ) -> QuadDetectorOutput | tuple[QuadDetectorOutput, QuadDetectorOutput]:
        features = self.forward_features(images)
        primary = self.head(features)
        if self.training and self.dual_head and self.aux_head is not None:
            return primary, self.aux_head(features)
        return primary


__all__ = [
    "DetectorOutput",
    "QuadDetectorOutput",
    "QuadProposalDetector",
    "StudentDetector",
]
