"""Phase-3 quality, overlap, LTRB, and FCOS centerness losses."""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch
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
    diagnostics: dict[str, Tensor]
    family_terms: dict[str, Tensor]


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
        (intersection_bottom_right - intersection_top_left).clamp(min=0).prod(dim=1)
    )
    size1 = (boxes1[:, 2:] - boxes1[:, :2]).clamp(min=0)
    size2 = (boxes2[:, 2:] - boxes2[:, :2]).clamp(min=0)
    union = size1.prod(dim=1) + size2.prod(dim=1) - intersection
    return intersection / union.clamp(min=torch.finfo(boxes1.dtype).eps)


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
        (intersection_bottom_right - intersection_top_left).clamp(min=0).prod(dim=1)
    )
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
        (intersection_bottom_right - intersection_top_left).clamp(min=0).prod(dim=1)
    )
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
        ((enclosing_bottom_right - enclosing_top_left) ** 2)
        .sum(dim=1)
        .clamp(min=torch.finfo(boxes1.dtype).eps)
    )

    angle = torch.atan(size2[:, 0] / size2[:, 1].clamp(min=1e-7)) - torch.atan(
        size1[:, 0] / size1[:, 1].clamp(min=1e-7)
    )
    aspect = 4.0 / math.pi**2 * angle.pow(2)
    with torch.no_grad():
        aspect_weight = aspect / (1 - iou + aspect).clamp(min=1e-7)
    ciou = iou - center_distance / diagonal - aspect_weight * aspect
    return 1 - ciou


def _per_image_weighted_mean(
    values: Tensor, weights: Tensor
) -> tuple[Tensor, Tensor, Tensor]:
    """Reduce locations within each image using an explicit safe denominator."""
    weight_sums = weights.sum(dim=1)
    active = weight_sums > 0
    reduced = (values * weights).sum(dim=1) / weight_sums.clamp(min=1)
    return reduced, weight_sums.detach(), active.detach()


class ProposalLoss(nn.Module):
    """Weighted Phase-3 objective with explicit masks and safe empty batches."""

    def __init__(
        self,
        *,
        strides: tuple[int, ...] = (8, 16, 32),
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
            raise ValueError("objectness_loss must be 'focal' or 'quality_focal'")
        if box_loss not in {"ciou", "giou"}:
            raise ValueError("box_loss must be 'ciou' or 'giou'")
        if box_weighting not in {"uniform", "centerness"}:
            raise ValueError("box_weighting must be 'uniform' or 'centerness'")
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

    def forward(self, output: DetectorOutput, targets: TrainingTargets) -> LossOutput:
        (
            objectness_logits,
            predicted_distances,
            centerness_logits,
        ) = flatten_detector_output(output)
        positive_count = targets.positive_mask.sum().to(dtype=objectness_logits.dtype)
        # Keep the loss graph shape independent of the number of positives.
        # Dynamic boolean indexing is harmless on CUDA but causes recompilation
        # and synchronization on PyTorch/XLA. Computing aligned losses for every
        # location and applying a dense mask is mathematically equivalent.
        positive = targets.positive_mask
        positive_float = positive.to(dtype=torch.float32)
        feature_shapes = tuple(
            (tensor.shape[-2], tensor.shape[-1]) for tensor in output.objectness
        )
        points, level_slices = make_grid_points(
            feature_shapes,
            self.strides,
            device=predicted_distances.device,
            dtype=predicted_distances.dtype,
        )
        batch_points = points.unsqueeze(0).expand(predicted_distances.shape[0], -1, -1)
        flat_predicted_boxes = decode_ltrb(
            batch_points.reshape(-1, 2),
            predicted_distances.reshape(-1, 4),
        ).float()
        flat_target_boxes = decode_ltrb(
            batch_points.reshape(-1, 2),
            targets.box_distances.reshape(-1, 4),
        ).float()
        box_losses = (
            aligned_ciou_loss(flat_predicted_boxes, flat_target_boxes)
            if self.box_loss == "ciou"
            else aligned_giou_loss(flat_predicted_boxes, flat_target_boxes)
        ).reshape_as(positive_float)
        if self.box_weighting == "centerness":
            center_weights = targets.centerness.float() * positive_float
            box_reduction_weights = center_weights
        else:
            box_reduction_weights = positive_float
        box_loss_per_image, box_weight_sums, box_active = _per_image_weighted_mean(
            box_losses, box_reduction_weights
        )
        box_loss = box_loss_per_image.mean()

        if self.ltrb_weight:
            location_strides = predicted_distances.new_empty(points.shape[0])
            for level_slice, stride in zip(level_slices, self.strides, strict=True):
                location_strides[level_slice] = stride
            batch_strides = location_strides.view(1, -1, 1)
            component_losses = functional.smooth_l1_loss(
                predicted_distances.float() / batch_strides,
                targets.box_distances.float() / batch_strides,
                reduction="none",
            )
            ltrb_location_losses = component_losses.mean(dim=2)
            (
                ltrb_loss_per_image,
                ltrb_weight_sums,
                ltrb_active,
            ) = _per_image_weighted_mean(ltrb_location_losses, positive_float)
            ltrb_loss = ltrb_loss_per_image.mean()
        else:
            # Reduce in FP32: an FP16 sum over all dense locations can overflow
            # to inf, and ``inf * 0`` is NaN even for a disabled branch.
            ltrb_loss = predicted_distances.float().mean() * 0
            ltrb_loss_per_image = positive_float.sum(dim=1) * 0
            ltrb_weight_sums = positive_float.sum(dim=1).detach()
            ltrb_active = (ltrb_weight_sums > 0).detach()

        if self.centerness_weight:
            centerness_losses = functional.binary_cross_entropy_with_logits(
                centerness_logits,
                targets.centerness.float(),
                reduction="none",
            )
            (
                centerness_loss_per_image,
                centerness_weight_sums,
                centerness_active,
            ) = _per_image_weighted_mean(centerness_losses, positive_float)
            centerness_loss = centerness_loss_per_image.mean()
        else:
            centerness_loss = objectness_logits.float().mean() * 0
            centerness_loss_per_image = positive_float.sum(dim=1) * 0
            centerness_weight_sums = positive_float.sum(dim=1).detach()
            centerness_active = (centerness_weight_sums > 0).detach()

        quality_targets = (
            aligned_iou(flat_predicted_boxes, flat_target_boxes)
            .reshape_as(targets.objectness)
            .detach()
            .to(dtype=targets.objectness.dtype)
            * positive_float
        )

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
        detached_quality = classification_losses.detach()
        detached_weights = targets.objectness_weights.detach()
        state_masks = {
            "positive": targets.positive_mask,
            "trusted_background": targets.trusted_background_mask,
            "weak_background": targets.weak_background_mask,
            "ignore": targets.ignore_mask,
        }
        state_raw: dict[str, Tensor] = {}
        state_effective: dict[str, Tensor] = {}
        state_counts: dict[str, Tensor] = {}
        state_weight_sums: dict[str, Tensor] = {}
        state_present: dict[str, Tensor] = {}
        for state, mask in state_masks.items():
            mask_float = mask.to(dtype=detached_quality.dtype)
            raw, counts, active = _per_image_weighted_mean(
                classification_losses, mask_float
            )
            weighted_mask = targets.objectness_weights * mask_float
            weight_sums = weighted_mask.sum(dim=1).detach()
            # Preserve weak-region coefficients: normalize by point count, not
            # by their weighted count. Ignore has zero effective weight.
            effective = (classification_losses * weighted_mask).sum(
                dim=1
            ) / counts.clamp(min=1)
            state_raw[state] = raw.detach()
            state_effective[state] = effective
            state_counts[state] = counts
            state_weight_sums[state] = weight_sums
            state_present[state] = active

        objectness_per_image = sum(
            state_effective[state]
            for state in ("positive", "trusted_background", "weak_background")
        )
        objectness_loss = objectness_per_image.mean()
        batch_size = max(objectness_logits.shape[0], 1)
        diagnostics = {
            "component/objectness/raw": objectness_per_image.detach() / batch_size,
            "component/box_ciou/raw": box_loss_per_image.detach() / batch_size,
            "component/box_ltrb/raw": ltrb_loss_per_image.detach() / batch_size,
            "component/centerness/raw": centerness_loss_per_image.detach() / batch_size,
            "component/objectness/points": targets.objectness_mask.sum(dim=1).detach(),
            "component/objectness/total_weight": detached_weights.sum(dim=1),
            "component/objectness/hard_negative_focus_points": (
                targets.hard_negative_focus_mask.sum(dim=1).detach()
            ),
            "component/objectness/hard_negative_focus_extra_weight": (
                (detached_weights - torch.ones_like(detached_weights)).clamp_min(0)
                * targets.hard_negative_focus_mask
            ).sum(dim=1),
            "component/box_ciou/points": targets.positive_mask.sum(dim=1).detach(),
            "component/box_ciou/total_weight": box_weight_sums,
            "component/box_ltrb/points": targets.positive_mask.sum(dim=1).detach(),
            "component/box_ltrb/total_weight": ltrb_weight_sums,
            "component/centerness/points": targets.positive_mask.sum(dim=1).detach(),
            "component/centerness/total_weight": centerness_weight_sums,
        }
        for name, active in (
            ("objectness", targets.objectness_mask.any(dim=1)),
            ("box_ciou", box_active),
            ("box_ltrb", ltrb_active),
            ("centerness", centerness_active),
        ):
            diagnostics[f"component/{name}/active_images"] = active.detach()
            diagnostics[f"component/{name}/empty_images"] = (~active).detach()
        component_weights = {
            "objectness": self.objectness_weight,
            "box_ciou": self.box_weight,
            "box_ltrb": self.ltrb_weight,
            "centerness": self.centerness_weight,
        }
        for name, weight in component_weights.items():
            diagnostics[f"component/{name}/weighted"] = (
                diagnostics[f"component/{name}/raw"] * weight
            )
        for state in state_masks:
            effective = state_effective[state] / batch_size
            diagnostics[f"state/{state}/objectness_raw"] = state_raw[state] / batch_size
            diagnostics[f"state/{state}/objectness_effective"] = effective
            diagnostics[f"state/{state}/objectness_weighted"] = (
                effective * self.objectness_weight
            )
            diagnostics[f"state/{state}/points"] = state_counts[state]
            diagnostics[f"state/{state}/total_weight"] = state_weight_sums[state]
            diagnostics[f"state/{state}/present_images"] = state_present[state]
            diagnostics[f"state/{state}/empty_images"] = ~state_present[state]
            diagnostics[f"state/{state}/active_images"] = state_weight_sums[state] > 0

        total = (
            self.objectness_weight * objectness_loss
            + self.box_weight * box_loss
            + self.ltrb_weight * ltrb_loss
            + self.centerness_weight * centerness_loss
        )
        family_terms = {
            "quality/unweighted": objectness_loss + centerness_loss,
            "quality/weighted": (
                self.objectness_weight * objectness_loss
                + self.centerness_weight * centerness_loss
            ),
            "localization/unweighted": box_loss + ltrb_loss,
            "localization/weighted": (
                self.box_weight * box_loss + self.ltrb_weight * ltrb_loss
            ),
        }
        return LossOutput(
            total=total,
            objectness=objectness_loss,
            box_ciou=box_loss,
            box_ltrb=ltrb_loss,
            centerness=centerness_loss,
            number_positive=positive_count.detach(),
            diagnostics=diagnostics,
            family_terms=family_terms,
        )
