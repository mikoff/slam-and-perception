"""Lightweight region embedding head for open-vocabulary distillation and edge inference."""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import torch
from torch import Tensor, nn
import torch.nn.functional as F
from torchvision.ops import MultiScaleRoIAlign

from .crop_extractor import _quad_to_bbox


class RegionEmbeddingHead(nn.Module):
    """Multi-scale RoI feature extractor and projector into SigLIP visual embedding space.

    /// @brief Lightweight RoI projector mapping multi-scale FPN features and box geometry to unit L2 embeddings.
    /// @param in_channels Number of channels in FPN feature maps (default 96).
    /// @param embedding_dim Target vision-language embedding dimension (default 768).
    /// @param output_size Spatial resolution for RoIAlign grid (default 3x3).
    /// @param hidden_dim Intermediate projection MLP dimension (default 512).
    /// @param strides Feature pyramid strides for mapping levels (default (4, 8, 16, 32)).
    /// @param focal_margin Fractional box expansion per side for the focal view (default 0.15).
    /// @param context_margin Fractional box expansion per side for context view (default 0.45).
    /// @param use_context_roi Whether to concatenate focal and context RoI features (default True).
    /// @param use_box_geometry Whether to inject 6-D geometric scale and position features (default True).
    /// @param geom_dim Intermediate dimension for geometric coordinate embedding (default 64).
    """

    def __init__(
        self,
        in_channels: int = 96,
        embedding_dim: int = 768,
        output_size: int = 3,
        hidden_dim: int = 512,
        strides: tuple[int, ...] = (4, 8, 16, 32),
        focal_margin: float = 0.15,
        context_margin: float = 0.45,
        use_context_roi: bool = True,
        use_box_geometry: bool = True,
        geom_dim: int = 64,
    ) -> None:
        super().__init__()
        self.in_channels = in_channels
        self.embedding_dim = embedding_dim
        self.output_size = output_size
        self.hidden_dim = hidden_dim
        self.strides = strides
        self.focal_margin = focal_margin
        self.context_margin = context_margin
        self.use_context_roi = use_context_roi
        self.use_box_geometry = use_box_geometry
        self.geom_dim = geom_dim

        featmap_names = [f"p{i}" for i in range(len(strides))]
        self.featmap_names = featmap_names

        self.roi_align = MultiScaleRoIAlign(
            featmap_names=featmap_names,
            output_size=output_size,
            sampling_ratio=2,
        )

        base_feat_dim = in_channels * output_size * output_size
        visual_dim = base_feat_dim * 2 if use_context_roi else base_feat_dim

        if use_box_geometry:
            self.geom_encoder: nn.Module | None = nn.Sequential(
                nn.Linear(6, geom_dim),
                nn.LayerNorm(geom_dim),
                nn.GELU(),
            )
            total_in = visual_dim + geom_dim
        else:
            self.geom_encoder = None
            total_in = visual_dim

        self.projector = nn.Sequential(
            nn.Linear(total_in, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.GELU(),
            nn.Linear(hidden_dim, embedding_dim),
        )

    def get_config(self) -> dict[str, Any]:
        """Return configuration dictionary for serialization.

        /// @return Dictionary containing complete constructor hyperparameters.
        """
        return {
            "in_channels": self.in_channels,
            "embedding_dim": self.embedding_dim,
            "output_size": self.output_size,
            "hidden_dim": self.hidden_dim,
            "strides": self.strides,
            "focal_margin": self.focal_margin,
            "context_margin": self.context_margin,
            "use_context_roi": self.use_context_roi,
            "use_box_geometry": self.use_box_geometry,
            "geom_dim": self.geom_dim,
        }

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> RegionEmbeddingHead:
        """Instantiate RegionEmbeddingHead from configuration dictionary.

        /// @param config Dictionary matching get_config() output.
        /// @return Instantiated RegionEmbeddingHead.
        """
        return cls(**config)

    @classmethod
    def load_from_checkpoint(
        cls, ckpt_path: str | Path, device: torch.device | str = "cpu"
    ) -> tuple[RegionEmbeddingHead, dict[str, Any]]:
        """Load RegionEmbeddingHead from saved checkpoint supporting v1-v4 formats.

        /// @param ckpt_path Path to the saved checkpoint file.
        /// @param device Target torch device for the loaded weights.
        /// @return Tuple of (loaded_head, full_checkpoint_dict).
        """
        ckpt = torch.load(ckpt_path, map_location=device)
        state_dict = ckpt["region_head_state_dict"]

        if "region_head_config" in ckpt:
            head = cls.from_config(ckpt["region_head_config"])
            head.load_state_dict(state_dict)
        else:
            # Detect legacy checkpoint configuration (v1 - v3)
            has_geom = any(k.startswith("geom_encoder") for k in state_dict)
            legacy_keys = [k for k in state_dict if k.startswith("projector.1.")]
            legacy_in_features = 864
            if legacy_keys:
                legacy_in_features = state_dict[legacy_keys[0]].shape[1]
            use_ctx = legacy_in_features > 864

            head = cls(
                use_context_roi=use_ctx,
                use_box_geometry=has_geom,
                context_margin=float(ckpt.get("context_margin", 0.15)),
            )

            # Map legacy flattened projector keys if necessary
            adapted_dict: dict[str, Tensor] = {}
            for k, v in state_dict.items():
                if k.startswith("projector.1."):
                    adapted_dict[k.replace("projector.1.", "projector.0.")] = v
                elif k.startswith("projector.2."):
                    adapted_dict[k.replace("projector.2.", "projector.1.")] = v
                elif k.startswith("projector.4."):
                    adapted_dict[k.replace("projector.4.", "projector.3.")] = v
                else:
                    adapted_dict[k] = v
            head.load_state_dict(adapted_dict)

        head.to(device)
        head.eval()
        return head, ckpt

    def forward(
        self,
        features: tuple[Tensor, ...] | list[Tensor],
        boxes: list[Tensor] | Tensor,
        image_shapes: list[tuple[int, int]] | None = None,
    ) -> Tensor:
        """Extract multi-scale pooled region embeddings and project to unit L2 sphere.

        /// @brief Runs RoIAlign over pyramid features, encodes geometry, and projects to L2 normalized embeddings.
        /// @param features Multi-level FPN feature maps corresponding to self.strides.
        /// @param boxes List of [M, 4] box tensors (one per batch item) or a single [N, 4] tensor.
        /// @param image_shapes Optional list of (height, width) shapes for RoI canonical scaling.
        /// @return Float32 tensor of shape [N, embedding_dim] with unit L2 norm.
        """
        if len(features) != len(self.featmap_names):
            raise ValueError(
                f"Feature map count ({len(features)}) does not match expected levels ({len(self.featmap_names)})"
            )

        feat_dict = OrderedDict(
            (name, feat) for name, feat in zip(self.featmap_names, features, strict=True)
        )

        # Normalize boxes to list of [M, 4] tensors
        if isinstance(boxes, Tensor):
            if boxes.ndim == 2:
                box_list = [boxes]
            elif boxes.ndim == 3:
                box_list = [b for b in boxes]
            else:
                raise ValueError(f"Unsupported box tensor shape: {boxes.shape}")
        else:
            box_list = list(boxes)

        # Default image shapes to (384, 384) if omitted
        if image_shapes is None:
            batch_size = len(box_list)
            image_shapes = [(384, 384)] * batch_size

        total_boxes = sum(b.shape[0] for b in box_list)
        if total_boxes == 0:
            first_device = features[0].device
            return torch.empty((0, self.embedding_dim), dtype=torch.float32, device=first_device)

        # 1. Focal RoI extraction (tight crop matching teacher view)
        focal_box_list = [
            self.expand_boxes(b, self.focal_margin, shape)
            for b, shape in zip(box_list, image_shapes, strict=True)
        ]
        pooled_focal = self.roi_align(feat_dict, focal_box_list, image_shapes=image_shapes).flatten(
            start_dim=1
        )

        # 2. Context RoI extraction (wider crop capturing environment)
        if self.use_context_roi:
            context_box_list = [
                self.expand_boxes(b, self.context_margin, shape)
                for b, shape in zip(box_list, image_shapes, strict=True)
            ]
            pooled_context = self.roi_align(
                feat_dict, context_box_list, image_shapes=image_shapes
            ).flatten(start_dim=1)
            visual_feats = torch.cat([pooled_focal, pooled_context], dim=-1)
        else:
            visual_feats = pooled_focal

        # 3. Geometric box conditioning (normalized coordinates, scale, and aspect ratio)
        if self.use_box_geometry and self.geom_encoder is not None:
            geom_list = [
                self.compute_box_geometry(b, shape)
                for b, shape in zip(box_list, image_shapes, strict=True)
            ]
            geom_all = torch.cat(geom_list, dim=0)
            geom_feats = self.geom_encoder(geom_all)
            fused_feats = torch.cat([visual_feats, geom_feats], dim=-1)
        else:
            fused_feats = visual_feats

        # 4. Projection into vision-language embedding space
        projected = self.projector(fused_feats)
        return F.normalize(projected, p=2.0, dim=-1)

    @staticmethod
    def compute_box_geometry(boxes: Tensor, image_shape: tuple[int, int]) -> Tensor:
        """Compute 6-D normalized geometric scale and position features for boxes.

        /// @brief Computes [xc/W, yc/H, w/W, h/H, log(w/h), sqrt(area_norm)] for each box.
        /// @param boxes Float tensor of shape [M, 4] in canvas xyxy coordinates.
        /// @param image_shape (height, width) of the canvas.
        /// @return Float tensor of shape [M, 6] with normalized geometric features.
        """
        if boxes.numel() == 0:
            return torch.empty((0, 6), dtype=torch.float32, device=boxes.device)
        h_img, w_img = image_shape
        w = (boxes[:, 2] - boxes[:, 0]).clamp(min=1.0)
        h = (boxes[:, 3] - boxes[:, 1]).clamp(min=1.0)
        xc = (boxes[:, 0] + boxes[:, 2]) * 0.5
        yc = (boxes[:, 1] + boxes[:, 3]) * 0.5

        norm_xc = xc / float(w_img)
        norm_yc = yc / float(h_img)
        norm_w = w / float(w_img)
        norm_h = h / float(h_img)
        aspect_log = torch.log(norm_w / norm_h).clamp(min=-3.0, max=3.0)
        area_norm = torch.sqrt(norm_w * norm_h)

        return torch.stack([norm_xc, norm_yc, norm_w, norm_h, aspect_log, area_norm], dim=-1)

    @staticmethod
    def expand_boxes(boxes: Tensor, margin: float, image_shape: tuple[int, int]) -> Tensor:
        """Expand [M, 4] xyxy boxes by `margin` of their size per side, clamped to the image.

        /// @brief Mirrors the teacher's context-margin crop so pooled regions match crops.
        /// @param boxes Float tensor [M, 4] in canvas pixels.
        /// @param margin Fractional expansion per side (0 disables).
        /// @param image_shape (height, width) of the canvas used for clamping.
        /// @return Expanded float tensor [M, 4].
        """
        if margin <= 0.0 or boxes.numel() == 0:
            return boxes
        h, w = image_shape
        pad = (boxes[:, 2:] - boxes[:, :2]) * margin
        x1y1 = (boxes[:, :2] - pad).clamp(min=0.0)
        x2 = (boxes[:, 2] + pad[:, 0]).clamp(max=float(w))
        y2 = (boxes[:, 3] + pad[:, 1]).clamp(max=float(h))
        return torch.cat([x1y1, x2[:, None], y2[:, None]], dim=1)

    @classmethod
    def boxes_from_quads(cls, quads: Tensor | Sequence[Sequence[Sequence[float]]]) -> Tensor:
        """Helper to derive enclosing axis-aligned envelopes from 4-point quads."""
        if isinstance(quads, Tensor):
            pts = quads.reshape(-1, 4, 2)
            x1 = pts[..., 0].min(dim=-1).values
            y1 = pts[..., 1].min(dim=-1).values
            x2 = pts[..., 0].max(dim=-1).values
            y2 = pts[..., 1].max(dim=-1).values
            return torch.stack([x1, y1, x2, y2], dim=-1)
        res = [_quad_to_bbox(q) for q in quads]
        return torch.tensor(res, dtype=torch.float32)


__all__ = [
    "RegionEmbeddingHead",
]
