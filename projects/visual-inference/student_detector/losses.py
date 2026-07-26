"""Phase-3 quality, overlap, LTRB, and FCOS centerness losses."""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch
import torch.distributed as distributed
import torch.nn.functional as functional
from torch import Tensor, nn

from .geometry import decode_ltrb, make_grid_points
from .head import DetectorOutput
from .targets import TrainingTargets


@dataclass(frozen=True)
class LossOutput:
    total: Tensor
    objectness: Tensor
    box_ciou: Tensor
    box_ltrb: Tensor
    centerness: Tensor
    number_positive: Tensor


def flatten_detector_output(
    output: DetectorOutput,
) -> tuple[Tensor, Tensor, Tensor]:
    objectness = torch.cat(
        [tensor.flatten(start_dim=1) for tensor in output.objectness], dim=1
    )
    distances = torch.cat(
        [
            tensor.permute(0, 2, 3, 1).reshape(tensor.shape[0], -1, 4)
            for tensor in output.box_distances
        ],
        dim=1,
    )
    centerness = torch.cat(
        [tensor.flatten(start_dim=1) for tensor in output.centerness], dim=1
    )
    return objectness, distances, centerness


def sigmoid_focal_loss(
    logits: Tensor,
    targets: Tensor,
    *,
    alpha: float,
    gamma: float,
) -> Tensor:
    cross_entropy = functional.binary_cross_entropy_with_logits(
        logits, targets, reduction="none"
    )
    probability = torch.sigmoid(logits)
    target_probability = probability * targets + (1 - probability) * (1 - targets)
    alpha_factor = alpha * targets + (1 - alpha) * (1 - targets)
    return alpha_factor * (1 - target_probability).pow(gamma) * cross_entropy


def quality_focal_loss(
    logits: Tensor,
    targets: Tensor,
    *,
    beta: float = 2.0,
) -> Tensor:
    """Quality Focal Loss for continuous IoU targets in ``[0, 1]``."""
    cross_entropy = functional.binary_cross_entropy_with_logits(
        logits, targets, reduction="none"
    )
    modulation = (targets - torch.sigmoid(logits)).abs().pow(beta)
    return modulation * cross_entropy


def aligned_iou(boxes1: Tensor, boxes2: Tensor) -> Tensor:
    """IoU for aligned xyxy pairs."""
    intersection_top_left = torch.maximum(boxes1[:, :2], boxes2[:, :2])
    intersection_bottom_right = torch.minimum(boxes1[:, 2:], boxes2[:, 2:])
    intersection = (
        intersection_bottom_right - intersection_top_left
    ).clamp(min=0).prod(dim=1)
    size1 = (boxes1[:, 2:] - boxes1[:, :2]).clamp(min=0)
    size2 = (boxes2[:, 2:] - boxes2[:, :2]).clamp(min=0)
    union = size1.prod(dim=1) + size2.prod(dim=1) - intersection
    return intersection / union.clamp(
        min=torch.finfo(boxes1.dtype).eps
    )


def aligned_giou_loss(boxes1: Tensor, boxes2: Tensor) -> Tensor:
    """Generalized-IoU loss for aligned xyxy box pairs."""
    iou = aligned_iou(boxes1, boxes2)
    top_left = torch.minimum(boxes1[:, :2], boxes2[:, :2])
    bottom_right = torch.maximum(boxes1[:, 2:], boxes2[:, 2:])
    enclosing_area = (bottom_right - top_left).clamp(min=0).prod(dim=1)

    size1 = (boxes1[:, 2:] - boxes1[:, :2]).clamp(min=0)
    size2 = (boxes2[:, 2:] - boxes2[:, :2]).clamp(min=0)
    intersection_top_left = torch.maximum(boxes1[:, :2], boxes2[:, :2])
    intersection_bottom_right = torch.minimum(boxes1[:, 2:], boxes2[:, 2:])
    intersection = (
        intersection_bottom_right - intersection_top_left
    ).clamp(min=0).prod(dim=1)
    union = size1.prod(dim=1) + size2.prod(dim=1) - intersection
    giou = iou - (enclosing_area - union) / enclosing_area.clamp(
        min=torch.finfo(boxes1.dtype).eps
    )
    return 1 - giou


def aligned_ciou_loss(boxes1: Tensor, boxes2: Tensor) -> Tensor:
    """Complete-IoU loss for aligned xyxy box pairs."""
    intersection_top_left = torch.maximum(boxes1[:, :2], boxes2[:, :2])
    intersection_bottom_right = torch.minimum(boxes1[:, 2:], boxes2[:, 2:])
    intersection = (
        intersection_bottom_right - intersection_top_left
    ).clamp(min=0).prod(dim=1)
    size1 = (boxes1[:, 2:] - boxes1[:, :2]).clamp(min=0)
    size2 = (boxes2[:, 2:] - boxes2[:, :2]).clamp(min=0)
    union = size1.prod(dim=1) + size2.prod(dim=1) - intersection
    iou = intersection / union.clamp(min=torch.finfo(boxes1.dtype).eps)

    center1 = (boxes1[:, :2] + boxes1[:, 2:]) * 0.5
    center2 = (boxes2[:, :2] + boxes2[:, 2:]) * 0.5
    center_distance = ((center1 - center2) ** 2).sum(dim=1)
    enclosing_top_left = torch.minimum(boxes1[:, :2], boxes2[:, :2])
    enclosing_bottom_right = torch.maximum(boxes1[:, 2:], boxes2[:, 2:])
    diagonal = (
        (enclosing_bottom_right - enclosing_top_left) ** 2
    ).sum(dim=1).clamp(min=torch.finfo(boxes1.dtype).eps)

    angle = (
        torch.atan(size2[:, 0] / size2[:, 1].clamp(min=1e-7))
        - torch.atan(size1[:, 0] / size1[:, 1].clamp(min=1e-7))
    )
    aspect = 4.0 / math.pi**2 * angle.pow(2)
    with torch.no_grad():
        aspect_weight = aspect / (1 - iou + aspect).clamp(min=1e-7)
    ciou = iou - center_distance / diagonal - aspect_weight * aspect
    return 1 - ciou


def _distributed_average_normalizer(value: Tensor) -> Tensor:
    value = value.detach().clone()
    if distributed.is_available() and distributed.is_initialized():
        distributed.all_reduce(value)
        value /= distributed.get_world_size()
    return value.clamp(min=1.0)


class ProposalLoss(nn.Module):
    """Weighted Phase-3 objective with explicit masks and safe empty batches."""

    def __init__(
        self,
        *,
        strides: tuple[int, int, int] = (8, 16, 32),
        objectness_weight: float = 1.0,
        box_weight: float = 2.0,
        ltrb_weight: float = 0.0,
        centerness_weight: float = 1.0,
        objectness_loss: str = "focal",
        quality_focal_beta: float = 2.0,
        box_loss: str = "ciou",
        box_weighting: str = "uniform",
        focal_alpha: float = 0.25,
        focal_gamma: float = 2.0,
    ) -> None:
        super().__init__()
        if objectness_loss not in {"focal", "quality_focal"}:
            raise ValueError(
                "objectness_loss must be 'focal' or 'quality_focal'"
            )
        if box_loss not in {"ciou", "giou"}:
            raise ValueError("box_loss must be 'ciou' or 'giou'")
        if box_weighting not in {"uniform", "centerness"}:
            raise ValueError(
                "box_weighting must be 'uniform' or 'centerness'"
            )
        if ltrb_weight < 0:
            raise ValueError("ltrb_weight must be non-negative")
        self.strides = strides
        self.objectness_weight = objectness_weight
        self.box_weight = box_weight
        self.ltrb_weight = ltrb_weight
        self.centerness_weight = centerness_weight
        self.objectness_loss = objectness_loss
        self.quality_focal_beta = quality_focal_beta
        self.box_loss = box_loss
        self.box_weighting = box_weighting
        self.focal_alpha = focal_alpha
        self.focal_gamma = focal_gamma

    def forward(
        self, output: DetectorOutput, targets: TrainingTargets
    ) -> LossOutput:
        objectness_logits, predicted_distances, centerness_logits = (
            flatten_detector_output(output)
        )
        positive_count = targets.positive_mask.sum().to(
            dtype=objectness_logits.dtype
        )
        positive_normalizer = _distributed_average_normalizer(positive_count)

        positive = targets.positive_mask
        if torch.any(positive):
            feature_shapes = tuple(
                (tensor.shape[-2], tensor.shape[-1])
                for tensor in output.objectness
            )
            points, level_slices = make_grid_points(
                feature_shapes,
                self.strides,
                device=predicted_distances.device,
                dtype=predicted_distances.dtype,
            )
            batch_points = points.unsqueeze(0).expand(
                predicted_distances.shape[0], -1, -1
            )
            predicted_boxes = decode_ltrb(
                batch_points[positive], predicted_distances[positive]
            ).float()
            target_boxes = decode_ltrb(
                batch_points[positive], targets.box_distances[positive]
            ).float()
            center_weights = targets.centerness[positive].float()
            box_losses = (
                aligned_ciou_loss(predicted_boxes, target_boxes)
                if self.box_loss == "ciou"
                else aligned_giou_loss(predicted_boxes, target_boxes)
            )
            if self.box_weighting == "centerness":
                box_normalizer = _distributed_average_normalizer(
                    center_weights.sum()
                )
                box_loss = (
                    box_losses * center_weights
                ).sum() / box_normalizer
            else:
                box_loss = box_losses.sum() / positive_normalizer

            if self.ltrb_weight:
                location_strides = predicted_distances.new_empty(
                    points.shape[0]
                )
                for level_slice, stride in zip(
                    level_slices, self.strides, strict=True
                ):
                    location_strides[level_slice] = stride
                batch_strides = location_strides.unsqueeze(0).expand(
                    predicted_distances.shape[0], -1
                )
                positive_strides = batch_strides[positive].float().unsqueeze(1)
                ltrb_loss = functional.smooth_l1_loss(
                    predicted_distances[positive].float() / positive_strides,
                    targets.box_distances[positive].float() / positive_strides,
                    reduction="sum",
                ) / (positive_normalizer * 4)
            else:
                ltrb_loss = predicted_distances.sum() * 0

            if self.centerness_weight:
                centerness_loss = (
                    functional.binary_cross_entropy_with_logits(
                        centerness_logits[positive],
                        center_weights,
                        reduction="sum",
                    )
                    / positive_normalizer
                )
            else:
                centerness_loss = objectness_logits.sum() * 0

            quality_targets = torch.zeros_like(targets.objectness)
            quality_targets[positive] = aligned_iou(
                predicted_boxes, target_boxes
            ).detach().to(dtype=quality_targets.dtype)
        else:
            box_loss = predicted_distances.sum() * 0
            ltrb_loss = predicted_distances.sum() * 0
            centerness_loss = objectness_logits.sum() * 0
            quality_targets = torch.zeros_like(targets.objectness)

        if self.objectness_loss == "quality_focal":
            classification_losses = quality_focal_loss(
                objectness_logits,
                quality_targets,
                beta=self.quality_focal_beta,
            )
        else:
            classification_losses = sigmoid_focal_loss(
                objectness_logits,
                targets.objectness,
                alpha=self.focal_alpha,
                gamma=self.focal_gamma,
            )
        objectness_loss = (
            classification_losses * targets.objectness_weights
        ).sum() / positive_normalizer

        total = (
            self.objectness_weight * objectness_loss
            + self.box_weight * box_loss
            + self.ltrb_weight * ltrb_loss
            + self.centerness_weight * centerness_loss
        )
        return LossOutput(
            total=total,
            objectness=objectness_loss,
            box_ciou=box_loss,
            box_ltrb=ltrb_loss,
            centerness=centerness_loss,
            number_positive=positive_count.detach(),
        )
