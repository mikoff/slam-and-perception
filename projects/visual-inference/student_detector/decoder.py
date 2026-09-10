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


def class_agnostic_nms(boxes: Tensor, scores: Tensor, iou_threshold: float) -> Tensor:
    """Use the compiled torchvision kernel outside the exported model graph."""
    if boxes.numel() == 0:
        return torch.empty((0,), dtype=torch.long, device=boxes.device)
    order = torch.argsort(scores, descending=True, stable=True)
    # NMS only needs a strict processing order. Unique synthetic priorities
    # preserve stable score/index ordering even when model scores are equal.
    priorities = torch.arange(
        order.numel(), 0, -1, dtype=torch.float32, device=scores.device
    )
    kept_order = torchvision_nms(boxes[order].float(), priorities, iou_threshold)
    return order[kept_order]


class InferenceDecoder:
    """Score, top-K, decode, clip and NMS raw detector outputs."""

    def __init__(
        self,
        *,
        strides: tuple[int, ...] = (8, 16, 32),
        top_k: int = 300,
        nms_iou_threshold: float = 0.6,
        max_detections: int = 100,
        score_mode: str = "objectness_x_centerness",
    ) -> None:
        if score_mode not in {"objectness", "objectness_x_centerness"}:
            raise ValueError(
                "score_mode must be 'objectness' or 'objectness_x_centerness'"
            )
        self.strides = strides
        self.top_k = top_k
        self.nms_iou_threshold = nms_iou_threshold
        self.max_detections = max_detections
        self.score_mode = score_mode

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
        if self.score_mode == "objectness":
            scores = torch.cat(
                [torch.sigmoid(obj).flatten(start_dim=1) for obj in output.objectness],
                dim=1,
            )
        else:
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
            keep = class_agnostic_nms(
                selected_boxes, selected_scores, self.nms_iou_threshold
            )[: self.max_detections]
            detections.append(
                Detection(
                    selected_boxes[keep],
                    selected_scores[keep],
                    selected_levels[keep],
                    selected_locations[keep],
                    selected_boxes,
                    selected_scores,
                    count,
                    invalid_count,
                )
            )
        return detections
