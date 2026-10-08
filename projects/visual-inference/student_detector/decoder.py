"""Post-processing kept outside the quantized detector graph."""

from __future__ import annotations

from dataclasses import dataclass

import torch
from torch import Tensor
from torchvision.ops import nms as torchvision_nms

from .geometry import decode_ltrb, make_grid_points
from .head import DetectorOutput


@dataclass(frozen=True)
class Detection:
    boxes: Tensor
    scores: Tensor
    levels: Tensor | None = None
    location_indices: Tensor | None = None
    pre_nms_boxes: Tensor | None = None
    pre_nms_scores: Tensor | None = None
    candidate_count: int = 0
    invalid_candidate_count: int = 0


def class_agnostic_nms(
    boxes: Tensor,
    scores: Tensor,
    iou_threshold: float,
    *,
    nms_mode: str = "hard",
    soft_sigma: float = 0.5,
    part_scale_ratio: float = 0.4,
    part_max_iou: float = 0.65,
    score_threshold: float = 0.05,
    max_output: int | None = None,
    return_scores: bool = False,
) -> Tensor | tuple[Tensor, Tensor]:
    """Class-agnostic NMS supporting hard NMS, Gaussian Soft-NMS, and part-aware suppression."""
    if boxes.numel() == 0:
        empty = torch.empty((0,), dtype=torch.long, device=boxes.device)
        empty_scores = torch.empty((0,), dtype=scores.dtype, device=scores.device)
        return (empty, empty_scores) if return_scores else empty

    if nms_mode == "nms_free":
        order = torch.argsort(scores, descending=True, stable=True)
        if max_output is not None:
            order = order[:max_output]
        return (order, scores[order]) if return_scores else order

    if nms_mode == "hard":
        order = torch.argsort(scores, descending=True, stable=True)
        # NMS only needs a strict processing order. Unique synthetic priorities
        # preserve stable score/index ordering even when model scores are equal.
        priorities = torch.arange(
            order.numel(), 0, -1, dtype=torch.float32, device=scores.device
        )
        kept_order = torchvision_nms(boxes[order].float(), priorities, iou_threshold)
        keep = order[kept_order]
        if max_output is not None:
            keep = keep[:max_output]
        return (keep, scores[keep]) if return_scores else keep

    boxes_f = boxes.float()
    scores_f = scores.clone().float()
    num_boxes = boxes.shape[0]

    widths = (boxes_f[:, 2] - boxes_f[:, 0]).clamp(min=0)
    heights = (boxes_f[:, 3] - boxes_f[:, 1]).clamp(min=0)
    areas = widths * heights

    order = torch.argsort(scores_f, descending=True, stable=True)
    boxes_sorted = boxes_f[order]
    areas_sorted = areas[order]

    from torchvision.ops import box_iou as tv_box_iou

    ious = tv_box_iou(boxes_sorted, boxes_sorted)
    scale_ratios = torch.minimum(
        areas_sorted[:, None], areas_sorted[None, :]
    ) / torch.maximum(areas_sorted[:, None], areas_sorted[None, :]).clamp(min=1e-6)

    kept_local: list[int] = []
    final_scores: list[float] = []
    suppressed = torch.zeros(num_boxes, dtype=torch.bool, device=boxes.device)
    current_scores = scores_f[order].clone()

    for i in range(num_boxes):
        if suppressed[i] or current_scores[i] < score_threshold:
            continue
        kept_local.append(i)
        final_scores.append(float(current_scores[i].item()))
        if max_output is not None and len(kept_local) >= max_output:
            break

        remaining = torch.arange(i + 1, num_boxes, device=boxes.device)
        active_remaining = remaining[~suppressed[remaining]]
        if active_remaining.numel() == 0:
            continue

        pair_ious = ious[i, active_remaining]
        pair_ratios = scale_ratios[i, active_remaining]

        if nms_mode == "part_aware":
            to_suppress = (pair_ious > part_max_iou) | (
                (pair_ious > iou_threshold) & (pair_ratios > part_scale_ratio)
            )
            suppressed[active_remaining[to_suppress]] = True
        elif nms_mode == "soft":
            decay = torch.exp(-(pair_ious**2) / soft_sigma)
            current_scores[active_remaining] *= decay
            dropped = current_scores[active_remaining] < score_threshold
            suppressed[active_remaining[dropped]] = True
        elif nms_mode == "part_aware_soft":
            decay = torch.where(
                (pair_ratios <= part_scale_ratio) & (pair_ious <= part_max_iou),
                torch.ones_like(pair_ious),
                torch.exp(-(pair_ious**2) / soft_sigma),
            )
            current_scores[active_remaining] *= decay
            dropped = current_scores[active_remaining] < score_threshold
            suppressed[active_remaining[dropped]] = True
        else:
            raise ValueError(f"Unknown nms_mode: {nms_mode}")

    keep = order[torch.tensor(kept_local, dtype=torch.long, device=boxes.device)]
    kept_scores = torch.tensor(final_scores, dtype=scores.dtype, device=scores.device)
    return (keep, kept_scores) if return_scores else keep


class InferenceDecoder:
    """Score, top-K, decode, clip and NMS raw detector outputs."""

    def __init__(
        self,
        *,
        strides: tuple[int, ...] = (8, 16, 32),
        top_k: int = 1000,
        nms_iou_threshold: float = 0.50,
        max_detections: int = 100,
        score_mode: str = "objectness",
        nms_mode: str = "hard",
        soft_sigma: float = 0.5,
        part_scale_ratio: float = 0.4,
        part_max_iou: float = 0.65,
        centerness_alpha: float = 1.0,
        score_threshold: float = 0.05,
    ) -> None:
        if score_mode not in {"objectness", "objectness_x_centerness"}:
            raise ValueError(
                "score_mode must be 'objectness' or 'objectness_x_centerness'"
            )
        if not (0.0 <= centerness_alpha <= 1.0):
            raise ValueError("centerness_alpha must be in [0.0, 1.0]")
        if nms_mode not in {
            "hard",
            "soft",
            "part_aware",
            "part_aware_soft",
            "nms_free",
        }:
            raise ValueError(
                "nms_mode must be 'hard', 'soft', 'part_aware', 'part_aware_soft', or 'nms_free'"
            )
        self.strides = strides
        self.top_k = top_k
        self.nms_iou_threshold = nms_iou_threshold
        self.max_detections = max_detections
        self.score_mode = score_mode
        self.centerness_alpha = centerness_alpha
        self.nms_mode = nms_mode
        self.soft_sigma = soft_sigma
        self.part_scale_ratio = part_scale_ratio
        self.part_max_iou = part_max_iou
        self.score_threshold = score_threshold

    def __call__(
        self,
        output: DetectorOutput,
        image_size: tuple[int, int],
        valid_point_masks: tuple[Tensor, ...] | None = None,
    ) -> list[Detection]:
        height, width = image_size
        feature_shapes = tuple(
            (tensor.shape[-2], tensor.shape[-1]) for tensor in output.objectness
        )
        points, level_slices = make_grid_points(
            feature_shapes,
            self.strides,
            device=output.objectness[0].device,
            dtype=output.box_distances[0].dtype,
        )
        if self.score_mode == "objectness" or self.centerness_alpha == 0.0:
            scores = torch.cat(
                [torch.sigmoid(obj).flatten(start_dim=1) for obj in output.objectness],
                dim=1,
            )
        elif self.centerness_alpha == 1.0:
            scores = torch.cat(
                [
                    torch.sigmoid(obj).flatten(start_dim=1)
                    * torch.sigmoid(center).flatten(start_dim=1)
                    for obj, center in zip(
                        output.objectness, output.centerness, strict=True
                    )
                ],
                dim=1,
            )
        else:
            scores = torch.cat(
                [
                    torch.sigmoid(obj).flatten(start_dim=1)
                    * (
                        torch.sigmoid(center).flatten(start_dim=1)
                        ** self.centerness_alpha
                    )
                    for obj, center in zip(
                        output.objectness, output.centerness, strict=True
                    )
                ],
                dim=1,
            )
        if valid_point_masks is not None:
            flattened_masks = torch.cat(
                [mask.reshape(mask.shape[0], -1) for mask in valid_point_masks],
                dim=1,
            ).to(device=scores.device, dtype=torch.bool)
            if flattened_masks.shape != scores.shape:
                raise ValueError(
                    "valid_point_masks must match the batch and pyramid shapes"
                )
            scores = scores.masked_fill(~flattened_masks, -torch.inf)
        distances = torch.cat(
            [
                distance.permute(0, 2, 3, 1).reshape(distance.shape[0], -1, 4)
                for distance in output.box_distances
            ],
            dim=1,
        )
        point_levels = torch.cat(
            [
                torch.full(
                    (level_slice.stop - level_slice.start,),
                    stride.bit_length() - 1,
                    dtype=torch.int64,
                    device=points.device,
                )
                for level_slice, stride in zip(level_slices, self.strides, strict=True)
            ]
        )
        point_locations = torch.cat(
            [
                torch.arange(
                    level_slice.stop - level_slice.start,
                    dtype=torch.int64,
                    device=points.device,
                )
                for level_slice in level_slices
            ]
        )
        detections: list[Detection] = []
        for batch_index in range(scores.shape[0]):
            count = min(
                self.top_k,
                int(torch.isfinite(scores[batch_index]).sum().item()),
            )
            if count == 0:
                detections.append(
                    Detection(
                        torch.empty(
                            (0, 4),
                            dtype=distances.dtype,
                            device=distances.device,
                        ),
                        torch.empty((0,), dtype=scores.dtype, device=scores.device),
                        torch.empty((0,), dtype=torch.int64, device=scores.device),
                        torch.empty((0,), dtype=torch.int64, device=scores.device),
                    )
                )
                continue
            selected_indices = torch.argsort(
                scores[batch_index], descending=True, stable=True
            )[:count]
            selected_scores = scores[batch_index, selected_indices]
            selected_boxes = decode_ltrb(
                points[selected_indices], distances[batch_index, selected_indices]
            )
            selected_boxes[:, 0::2].clamp_(min=0, max=width)
            selected_boxes[:, 1::2].clamp_(min=0, max=height)
            valid = (
                torch.isfinite(selected_boxes).all(dim=1)
                & torch.isfinite(selected_scores)
                & (selected_boxes[:, 2] > selected_boxes[:, 0])
                & (selected_boxes[:, 3] > selected_boxes[:, 1])
            )
            invalid_count = int((~valid).sum().item())
            selected_boxes = selected_boxes[valid]
            selected_scores = selected_scores[valid]
            selected_levels = point_levels[selected_indices][valid]
            selected_locations = point_locations[selected_indices][valid]
            keep, kept_scores = class_agnostic_nms(
                selected_boxes,
                selected_scores,
                self.nms_iou_threshold,
                nms_mode=self.nms_mode,
                soft_sigma=self.soft_sigma,
                part_scale_ratio=self.part_scale_ratio,
                part_max_iou=self.part_max_iou,
                score_threshold=self.score_threshold,
                max_output=self.max_detections,
                return_scores=True,
            )
            detections.append(
                Detection(
                    selected_boxes[keep],
                    kept_scores,
                    selected_levels[keep],
                    selected_locations[keep],
                    selected_boxes,
                    selected_scores,
                    count,
                    invalid_count,
                )
            )
        return detections
