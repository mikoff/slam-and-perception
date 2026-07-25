"""Post-processing kept outside the quantized detector graph."""

from __future__ import annotations

from dataclasses import dataclass

import torch
from torch import Tensor

from .geometry import box_iou, decode_ltrb, make_grid_points
from .head import DetectorOutput


@dataclass(frozen=True)
class Detection:
    boxes: Tensor
    scores: Tensor


def class_agnostic_nms(boxes: Tensor, scores: Tensor, iou_threshold: float) -> Tensor:
    """Pure-PyTorch greedy NMS for the small preselected proposal set."""
    order = scores.argsort(descending=True)
    kept: list[Tensor] = []
    while order.numel() > 0:
        current = order[0]
        kept.append(current)
        if order.numel() == 1:
            break
        remaining = order[1:]
        overlaps = box_iou(boxes[current].unsqueeze(0), boxes[remaining]).squeeze(0)
        order = remaining[overlaps <= iou_threshold]
    if not kept:
        return torch.empty((0,), dtype=torch.long, device=boxes.device)
    return torch.stack(kept)


class InferenceDecoder:
    """Score, top-K, decode, clip and NMS raw detector outputs."""

    def __init__(
        self,
        *,
        strides: tuple[int, int, int] = (8, 16, 32),
        top_k: int = 300,
        nms_iou_threshold: float = 0.6,
        max_detections: int = 100,
    ) -> None:
        self.strides = strides
        self.top_k = top_k
        self.nms_iou_threshold = nms_iou_threshold
        self.max_detections = max_detections

    def __call__(
        self, output: DetectorOutput, image_size: tuple[int, int]
    ) -> list[Detection]:
        height, width = image_size
        feature_shapes = tuple(
            (tensor.shape[-2], tensor.shape[-1]) for tensor in output.objectness
        )
        points, _ = make_grid_points(
            feature_shapes,
            self.strides,
            device=output.objectness[0].device,
            dtype=output.box_distances[0].dtype,
        )
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
        distances = torch.cat(
            [
                distance.permute(0, 2, 3, 1).reshape(distance.shape[0], -1, 4)
                for distance in output.box_distances
            ],
            dim=1,
        )

        detections: list[Detection] = []
        for batch_index in range(scores.shape[0]):
            count = min(self.top_k, scores.shape[1])
            selected_scores, selected_indices = torch.topk(
                scores[batch_index], k=count, largest=True, sorted=True
            )
            selected_boxes = decode_ltrb(
                points[selected_indices], distances[batch_index, selected_indices]
            )
            selected_boxes[:, 0::2].clamp_(min=0, max=width)
            selected_boxes[:, 1::2].clamp_(min=0, max=height)
            keep = class_agnostic_nms(
                selected_boxes, selected_scores, self.nms_iou_threshold
            )[: self.max_detections]
            detections.append(Detection(selected_boxes[keep], selected_scores[keep]))
        return detections
