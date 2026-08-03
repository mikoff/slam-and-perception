"""Post-processing for fixed-shape quad proposal outputs."""

from __future__ import annotations

from dataclasses import dataclass

import torch
from torch import Tensor

from .geometry import make_grid_points
from .head import QuadDetectorOutput
from .quad_geometry import (
    canonicalize_quads,
    decode_quad_offsets,
    polygon_nms,
    quad_validity,
)


@dataclass(frozen=True)
class QuadDetection:
    quads: Tensor
    scores: Tensor
    pre_nms_quads: Tensor | None = None
    pre_nms_scores: Tensor | None = None
    candidate_count: int = 0
    invalid_candidate_count: int = 0

    @property
    def boxes(self) -> Tensor:
        """Compatibility envelope for callers that only need an HBB."""
        if self.quads.numel() == 0:
            return self.quads.new_empty((0, 4))
        return torch.cat((self.quads[..., 0].amin(dim=1, keepdim=True),
                          self.quads[..., 1].amin(dim=1, keepdim=True),
                          self.quads[..., 0].amax(dim=1, keepdim=True),
                          self.quads[..., 1].amax(dim=1, keepdim=True)), dim=1)


def class_agnostic_polygon_nms(quads: Tensor, scores: Tensor, iou_threshold: float) -> Tensor:
    return polygon_nms(quads, scores, iou_threshold)


def decode_dense_quad_output(
    output: QuadDetectorOutput,
    strides: tuple[int, int, int] = (8, 16, 32),
    centerness_alpha: float = 0.3,
) -> tuple[Tensor, Tensor]:
    """Decode all locations without ranking, validation, clipping, or NMS."""
    feature_shapes = tuple((item.shape[-2], item.shape[-1]) for item in output.quality)
    points, level_slices = make_grid_points(
        feature_shapes,
        strides,
        device=output.quality[0].device,
        dtype=output.corner_offsets[0].dtype,
    )
    scores = torch.cat(
        [item.sigmoid().flatten(start_dim=1) for item in output.quality], dim=1
    )
    offsets = torch.cat([
        item.permute(0, 2, 3, 1).reshape(item.shape[0], -1, 8)
        for item in output.corner_offsets
    ], dim=1)
    point_strides = offsets.new_empty(points.shape[0])
    for level_slice, stride in zip(level_slices, strides, strict=True):
        point_strides[level_slice] = stride
    decoded = decode_quad_offsets(
        points.unsqueeze(0), offsets, point_strides.reshape(1, -1, 1)
    )
    if centerness_alpha > 0:
        px = decoded[..., 0]
        py = decoded[..., 1]
        pts = points.unsqueeze(0)
        l = (pts[..., 0] - px.min(dim=-1).values).clamp(min=1e-4)
        r = (px.max(dim=-1).values - pts[..., 0]).clamp(min=1e-4)
        t = (pts[..., 1] - py.min(dim=-1).values).clamp(min=1e-4)
        b = (py.max(dim=-1).values - pts[..., 1]).clamp(min=1e-4)
        centerness = torch.sqrt(
            (torch.minimum(l, r) / torch.maximum(l, r)) *
            (torch.minimum(t, b) / torch.maximum(t, b))
        ).clamp(min=1e-4)
        scores = (scores ** (1.0 - centerness_alpha)) * (centerness ** centerness_alpha)
    return decoded, scores



class QuadInferenceDecoder:
    """Decode, validate, rank, polygon-NMS, and cap candidate quads."""

    def __init__(
        self,
        *,
        strides: tuple[int, int, int] = (8, 16, 32),
        pre_nms_top_k: int = 300,
        nms_iou_threshold: float = 0.9,
        max_proposals: int = 100,
    ) -> None:
        if pre_nms_top_k < max_proposals:
            raise ValueError("pre_nms_top_k must be at least max_proposals")
        self.strides = strides
        self.pre_nms_top_k = pre_nms_top_k
        self.nms_iou_threshold = nms_iou_threshold
        self.max_proposals = max_proposals

    def __call__(
        self,
        output: QuadDetectorOutput,
        image_size: tuple[int, int],
        valid_point_masks: tuple[Tensor, Tensor, Tensor] | None = None,
    ) -> list[QuadDetection]:
        height, width = image_size
        decoded_all, scores = decode_dense_quad_output(output, self.strides)
        if valid_point_masks is not None:
            masks = torch.cat([mask.reshape(mask.shape[0], -1) for mask in valid_point_masks], dim=1)
            if masks.shape != scores.shape:
                raise ValueError("valid_point_masks must match P3-P5 output shapes")
            scores = scores.masked_fill(~masks.to(device=scores.device, dtype=torch.bool), -torch.inf)

        detections: list[QuadDetection] = []
        for batch in range(scores.shape[0]):
            finite = torch.isfinite(scores[batch])
            count = min(self.pre_nms_top_k, int(finite.sum().item()))
            if count == 0:
                detections.append(QuadDetection(
                    decoded_all.new_empty((0, 4, 2)), scores.new_empty((0,)),
                    decoded_all.new_empty((0, 4, 2)), scores.new_empty((0,)),
                ))
                continue
            selected_scores, selected_indices = torch.topk(scores[batch], count, sorted=True)
            decoded = decoded_all[batch, selected_indices]
            in_bounds = (
                (decoded[..., 0] >= 0).all(dim=1)
                & (decoded[..., 0] <= width).all(dim=1)
                & (decoded[..., 1] >= 0).all(dim=1)
                & (decoded[..., 1] <= height).all(dim=1)
            )
            valid = quad_validity(decoded) & in_bounds
            invalid_count = int((~valid).sum().item())
            decoded = decoded[valid]
            selected_scores = selected_scores[valid]
            if decoded.numel() == 0:
                detections.append(QuadDetection(
                    decoded, selected_scores, decoded, selected_scores,
                    candidate_count=count,
                    invalid_candidate_count=invalid_count,
                ))
                continue
            canonical = canonicalize_quads(decoded)
            keep = polygon_nms(
                canonical,
                selected_scores,
                self.nms_iou_threshold,
                max_output=self.max_proposals,
            )
            detections.append(QuadDetection(
                canonical[keep], selected_scores[keep], canonical, selected_scores,
                candidate_count=count,
                invalid_candidate_count=invalid_count,
            ))
        return detections


InferenceQuadDecoder = QuadInferenceDecoder
