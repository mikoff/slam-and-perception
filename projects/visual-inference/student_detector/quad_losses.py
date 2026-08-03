"""Direct-corner and quality losses for quadrilateral proposals."""

from __future__ import annotations

from dataclasses import dataclass

import torch
import torch.nn.functional as functional
from torch import Tensor, nn

from .geometry import make_grid_points
from .head import QuadDetectorOutput
from .quad_geometry import (
    aligned_quad_iou,
    decode_quad_offsets,
    quad_signed_area,
)
from .quad_targets import QuadTrainingTargets


@dataclass(frozen=True)
class QuadLossOutput:
    total: Tensor
    quality: Tensor
    corner: Tensor
    gwd: Tensor
    validity: Tensor
    quality_positive: Tensor
    quality_trusted_background: Tensor
    quality_weak_background: Tensor
    number_positive: Tensor
    quality_target_mean: Tensor


def flatten_quad_output(output: QuadDetectorOutput) -> tuple[Tensor, Tensor]:
    quality = torch.cat([item.flatten(start_dim=1) for item in output.quality], dim=1)
    offsets = torch.cat([
        item.permute(0, 2, 3, 1).reshape(item.shape[0], -1, 8)
        for item in output.corner_offsets
    ], dim=1)
    return quality, offsets


def _quality_focal_loss(logits: Tensor, targets: Tensor, beta: float) -> Tensor:
    cross_entropy = functional.binary_cross_entropy_with_logits(logits, targets, reduction="none")
    return (targets - logits.sigmoid()).abs().pow(beta) * cross_entropy


class QuadProposalLoss(nn.Module):
    """Min-over-eight direct-corner loss with a detached quality curriculum."""

    def __init__(
        self,
        *,
        strides: tuple[int, int, int] = (8, 16, 32),
        quality_weight: float = 1.0,
        corner_weight: float = 2.0,
        corner_smooth_l1_beta: float = 1.0,
        gwd_weight: float = 0.0,
        validity_weight: float = 0.05,
        quality_focal_beta: float = 2.0,
        quality_target_mode: str = "centerness",
        quality_blend: float = 0.0,
        geometry_quality_target: str = "corner_proxy",
    ) -> None:
        super().__init__()
        if quality_target_mode not in {"centerness", "blend", "iou"}:
            raise ValueError("quality_target_mode must be centerness, blend, or iou")
        if not 0 <= quality_blend <= 1:
            raise ValueError("quality_blend must be in [0, 1]")
        if geometry_quality_target not in {"exact_iou", "corner_proxy"}:
            raise ValueError(
                "geometry_quality_target must be exact_iou or corner_proxy"
            )
        self.strides = strides
        self.quality_weight = quality_weight
        self.corner_weight = corner_weight
        self.corner_smooth_l1_beta = corner_smooth_l1_beta
        self.gwd_weight = gwd_weight
        self.validity_weight = validity_weight
        self.quality_focal_beta = quality_focal_beta
        self.quality_target_mode = quality_target_mode
        self.quality_blend = quality_blend
        self.geometry_quality_target = geometry_quality_target

    def _decoded_quads(self, offsets: Tensor, feature_shapes: tuple[tuple[int, int], ...]) -> Tensor:
        points, level_slices = make_grid_points(
            feature_shapes, self.strides, device=offsets.device, dtype=offsets.dtype
        )
        point_strides = offsets.new_empty(points.shape[0])
        for level_slice, stride in zip(level_slices, self.strides, strict=True):
            point_strides[level_slice] = stride
        return torch.stack([
            decode_quad_offsets(points, offsets[batch], point_strides[:, None])
            for batch in range(offsets.shape[0])
        ])

    def _corner_loss(self, predicted: Tensor, target: Tensor, positive: Tensor) -> Tensor:
        if not positive.any():
            return predicted.float().mean() * 0
        # Stride-normalized offsets preserve the same cyclic/reversed vertex
        # equivalence as image-space coordinates. Assignment canonicalizes
        # targets once, so generate all eight traversals directly here without
        # per-positive angle sorting (which is a major GPU training bottleneck).
        target_quads = target[positive].reshape(-1, 4, 2)
        forward = torch.stack([torch.roll(target_quads, -index, dims=1) for index in range(4)], dim=1)
        reverse = torch.flip(target_quads, dims=(1,))
        backward = torch.stack([torch.roll(reverse, -index, dims=1) for index in range(4)], dim=1)
        traversals = torch.cat((forward, backward), dim=1).reshape(-1, 8, 8)
        errors = functional.smooth_l1_loss(
            predicted[positive].float().unsqueeze(1).expand(-1, 8, -1),
            traversals.float(), reduction="none", beta=self.corner_smooth_l1_beta
        ).mean(dim=2)
        min_errors = errors.min(dim=1).values

        # Scale loss inverse to GT diagonal length (in stride units) to balance
        # gradients between small and large objects.
        xmin = target_quads[..., 0].amin(dim=1)
        xmax = target_quads[..., 0].amax(dim=1)
        ymin = target_quads[..., 1].amin(dim=1)
        ymax = target_quads[..., 1].amax(dim=1)
        gt_diag = torch.sqrt((xmax - xmin).square() + (ymax - ymin).square()).clamp(min=1.0)
        # Normalize weights so mean weight stays close to 1.0
        diag_weights = (gt_diag.median() / gt_diag).clamp(0.2, 5.0)
        return (min_errors * diag_weights).mean()


    def _validity_loss(self, quads: Tensor, positive: Tensor) -> Tensor:
        if not positive.any():
            return quads.float().mean() * 0
        selected = quads[positive].float()
        edges = torch.roll(selected, -1, dims=1) - selected
        edge_lengths = torch.linalg.vector_norm(edges, dim=2)
        edge_penalty = functional.relu(0.25 - edge_lengths).square().mean(dim=1)
        area_penalty = functional.relu(0.25 - quad_signed_area(selected).abs()).square()
        turns = (
            edges[..., 0] * torch.roll(edges[..., 1], -1, dims=1)
            - edges[..., 1] * torch.roll(edges[..., 0], -1, dims=1)
        )
        clockwise = functional.relu(0.01 - turns).mean(dim=1)
        counter_clockwise = functional.relu(0.01 + turns).mean(dim=1)
        convexity_penalty = torch.minimum(clockwise, counter_clockwise)
        return (edge_penalty + area_penalty + convexity_penalty).mean()

    @staticmethod
    def _gwd_loss(predicted: Tensor, target: Tensor, positive: Tensor) -> Tensor:
        """Return a stable, scale-normalized Gaussian Wasserstein auxiliary.

        Each quad is represented by the mean and covariance of its four
        vertices.  The closed-form 2-D Bures term avoids matrix square roots,
        while target-trace normalization keeps the auxiliary comparable across
        FPN levels.  This term is deliberately permutation invariant and cannot
        replace the vertex-correspondence corner loss.
        """
        if not positive.any():
            return predicted.float().mean() * 0
        # AMP otherwise recasts covariance products to float16 and dispatches
        # through an unsupported/fragile LU determinant path on CUDA.
        with torch.autocast(device_type=predicted.device.type, enabled=False):
            selected_pred = predicted[positive].float()
            selected_target = target[positive].float()
            pred_center = selected_pred.mean(dim=1)
            target_center = selected_target.mean(dim=1)
            pred_centered = selected_pred - pred_center[:, None]
            target_centered = selected_target - target_center[:, None]
            pred_covariance = pred_centered.transpose(1, 2) @ pred_centered / 4.0
            target_covariance = target_centered.transpose(1, 2) @ target_centered / 4.0

            pred_trace = pred_covariance.diagonal(dim1=-2, dim2=-1).sum(dim=-1)
            target_trace = target_covariance.diagonal(dim1=-2, dim2=-1).sum(dim=-1)
            scale = target_trace.clamp(min=1e-4)
            center_distance = (pred_center - target_center).square().sum(dim=-1) / scale

            covariance_product_trace = (
                pred_covariance * target_covariance.transpose(-1, -2)
            ).sum(dim=(-2, -1))
            pred_determinant = (
                pred_covariance[:, 0, 0] * pred_covariance[:, 1, 1]
                - pred_covariance[:, 0, 1] * pred_covariance[:, 1, 0]
            ).clamp(min=0)
            target_determinant = (
                target_covariance[:, 0, 0] * target_covariance[:, 1, 1]
                - target_covariance[:, 0, 1] * target_covariance[:, 1, 0]
            ).clamp(min=0)
            inner = (
                covariance_product_trace
                + 2.0 * torch.sqrt(pred_determinant * target_determinant + 1e-12)
            ).clamp(min=0)
            covariance_distance = (
                pred_trace + target_trace - 2.0 * torch.sqrt(inner + 1e-12)
            ).clamp(min=0) / scale
            normalized_distance = center_distance + covariance_distance
            return (normalized_distance / (1.0 + normalized_distance)).mean()

    @staticmethod
    def _group_mean(values: Tensor, mask: Tensor) -> Tensor:
        selected = values[mask]
        return selected.mean() if selected.numel() else values.mean() * 0

    @staticmethod
    def _corner_quality_proxy(predicted: Tensor, target: Tensor) -> Tensor:
        """Return an aspect-aware cyclic/reversed corner quality in [0, 1]."""
        forward = torch.stack(
            [torch.roll(target, -index, dims=1) for index in range(4)], dim=1
        )
        reverse = torch.flip(target, dims=(1,))
        backward = torch.stack(
            [torch.roll(reverse, -index, dims=1) for index in range(4)], dim=1
        )
        traversals = torch.cat((forward, backward), dim=1)
        centered = target - target.mean(dim=1, keepdim=True)
        covariance = centered.transpose(1, 2) @ centered / 4.0
        _, eigenvectors = torch.linalg.eigh(covariance)
        # Columns are the minor and major principal axes. Project errors into
        # that local frame and normalize each direction independently so a
        # one-pixel error across a thin object is not hidden by its long axis.
        local_target = centered @ eigenvectors
        half_extents = local_target.abs().amax(dim=1).clamp(min=1e-4)
        local_error = (predicted[:, None] - traversals) @ eigenvectors[:, None]
        normalized_vertex_error = torch.linalg.vector_norm(
            local_error / half_extents[:, None, None], dim=-1
        )
        normalized = normalized_vertex_error.mean(dim=-1).amin(dim=1)
        return torch.exp(-2.0 * normalized).clamp(0, 1)

    def forward(self, output: QuadDetectorOutput, targets: QuadTrainingTargets) -> QuadLossOutput:
        quality_logits, predicted_offsets = flatten_quad_output(output)
        positive = targets.positive_mask
        positive_count = positive.sum().to(dtype=quality_logits.dtype)
        feature_shapes = tuple((item.shape[-2], item.shape[-1]) for item in output.quality)
        predicted_quads = self._decoded_quads(predicted_offsets, feature_shapes)
        target_quads = self._decoded_quads(targets.corner_offsets, feature_shapes)
        corner = self._corner_loss(predicted_offsets.reshape(-1, 8), targets.corner_offsets.reshape(-1, 8), positive.reshape(-1))
        # Validity thresholds are defined in stride-normalized coordinates so
        # the same shape receives the same penalty on every FPN level.
        validity = self._validity_loss(predicted_offsets.reshape(-1, 4, 2), positive.reshape(-1))
        if self.gwd_weight > 0:
            gwd = self._gwd_loss(
                predicted_offsets.reshape(-1, 4, 2),
                targets.corner_offsets.reshape(-1, 4, 2),
                positive.reshape(-1),
            )
        else:
            gwd = predicted_offsets.float().mean() * 0

        quality_target = targets.quality.reshape(-1).float()
        if self.quality_target_mode in {"blend", "iou"} and positive.any():
            selected_pred = predicted_quads.reshape(-1, 4, 2)[positive.reshape(-1)].detach()
            selected_target = target_quads.reshape(-1, 4, 2)[positive.reshape(-1)].detach()
            # CUDA eigensolvers used by the proxy and reference polygon geometry
            # are float32-only; explicitly leave the surrounding AMP region.
            with torch.no_grad(), torch.autocast(
                device_type=selected_pred.device.type, enabled=False
            ):
                if self.geometry_quality_target == "exact_iou":
                    geometry_quality = aligned_quad_iou(
                        selected_pred.float(), selected_target.float()
                    )
                else:
                    geometry_quality = self._corner_quality_proxy(
                        selected_pred.float(), selected_target.float()
                    )
            if self.quality_target_mode == "iou":
                quality_target = quality_target.clone()
                quality_target[positive.reshape(-1)] = geometry_quality
            else:
                quality_target = quality_target.clone()
                quality_target[positive.reshape(-1)] = (
                    (1 - self.quality_blend) * quality_target[positive.reshape(-1)]
                    + self.quality_blend * geometry_quality
                )
        per_location_quality = _quality_focal_loss(
            quality_logits.reshape(-1), quality_target.to(quality_logits.dtype), self.quality_focal_beta
        )
        positive_quality = self._group_mean(per_location_quality, positive.reshape(-1))
        trusted_quality = self._group_mean(
            per_location_quality, targets.trusted_background_mask.reshape(-1)
        )
        weak_quality = self._group_mean(
            per_location_quality, targets.weak_background_mask.reshape(-1)
        )
        quality_loss = (
            positive_quality
            + trusted_quality
            + targets.weak_negative_weight * weak_quality
        )
        total = (
            self.quality_weight * quality_loss
            + self.corner_weight * corner
            + self.gwd_weight * gwd
            + self.validity_weight * validity
        )
        return QuadLossOutput(
            total=total,
            quality=quality_loss,
            corner=corner,
            gwd=gwd,
            validity=validity,
            quality_positive=positive_quality,
            quality_trusted_background=trusted_quality,
            quality_weak_background=weak_quality,
            number_positive=positive_count.detach(),
            quality_target_mean=quality_target[positive.reshape(-1)].mean() if positive.any() else quality_target.new_zeros(()),
        )
