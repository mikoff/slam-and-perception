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


def class_agnostic_nms(boxes: Tensor, scores: Tensor, iou_threshold: float) -> Tensor:
    """Use the compiled torchvision kernel outside the exported model graph."""
    if boxes.numel() == 0:
        return torch.empty((0,), dtype=torch.long, device=boxes.device)
    return torchvision_nms(boxes.float(), scores.float(), iou_threshold)


class InferenceDecoder:
    """Score, top-K, decode, clip and NMS raw detector outputs."""

    def __init__(
        self,
        *,
        strides: tuple[int, int, int] = (8, 16, 32),
        top_k: int = 300,
        nms_iou_threshold: float = 0.6,
        max_detections: int = 100,
        score_mode: str = "objectness_x_centerness",
    ) -> None:
        if score_mode not in {
            "objectness", "objectness_x_centerness"
        }:
            raise ValueError(
                "score_mode must be 'objectness' or "
                "'objectness_x_centerness'"
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
        valid_point_masks: tuple[Tensor, Tensor, Tensor] | None = None,
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
        if self.score_mode == "objectness":
            scores = torch.cat(
                [
                    torch.sigmoid(obj).flatten(start_dim=1)
                    for obj in output.objectness
                ],
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
                [
                    mask.reshape(mask.shape[0], -1)
                    for mask in valid_point_masks
                ],
                dim=1,
            ).to(device=scores.device, dtype=torch.bool)
            if flattened_masks.shape != scores.shape:
                raise ValueError(
                    "valid_point_masks must match the batch and P3-P5 shapes"
                )
            scores = scores.masked_fill(~flattened_masks, -torch.inf)
        distances = torch.cat(
            [
                distance.permute(0, 2, 3, 1).reshape(distance.shape[0], -1, 4)
                for distance in output.box_distances
            ],
            dim=1,
        )

        detections: list[Detection] = []
        for batch_index in range(scores.shape[0]):
            count = min(
                self.top_k,
                int(torch.isfinite(scores[batch_index]).sum().item()),
            )
            if count == 0:
                detections.append(Detection(
                    torch.empty(
                        (0, 4),
                        dtype=distances.dtype,
                        device=distances.device,
                    ),
                    torch.empty(
                        (0,), dtype=scores.dtype, device=scores.device
                    ),
                ))
                continue
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
