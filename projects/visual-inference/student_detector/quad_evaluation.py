"""Fixed-budget polygon recall metrics for quad proposals."""

from __future__ import annotations

from dataclasses import dataclass

import torch
from torch import Tensor

from .quad_decoder import QuadDetection
from .quad_geometry import aligned_quad_iou


@dataclass(frozen=True)
class QuadEvaluationImage:
    image_id: int
    domain: str
    camera_type: str
    image_size: tuple[int, int]
    ground_truth: Tensor
    ignore_quads: Tensor
    detection: QuadDetection
    pre_nms_detection: QuadDetection | None = None
    geometry_tiers: tuple[str, ...] = ()


def _best_overlaps(gt: Tensor, proposals: Tensor) -> Tensor:
    if gt.numel() == 0 or proposals.numel() == 0:
        return gt.new_zeros((gt.shape[0],))
    return torch.stack([
        aligned_quad_iou(proposals, target.expand_as(proposals)).max()
        for target in gt
    ])


def _recall(images: list[QuadEvaluationImage], top_k: int, threshold: float) -> tuple[int, int]:
    recalled = total = 0
    for image in images:
        if image.ground_truth.numel() == 0:
            continue
        total += image.ground_truth.shape[0]
        best = _best_overlaps(image.ground_truth, image.detection.quads[:top_k])
        recalled += int((best >= threshold).sum())
    return recalled, total


def evaluate_quad_proposals(images: list[QuadEvaluationImage]) -> dict[str, float]:
    """Compute AR over IoU .50:.95 and required fixed-budget slices."""
    metrics: dict[str, float] = {}
    thresholds = [0.50 + 0.05 * index for index in range(10)]
    for top_k in (50, 100, 300):
        for threshold in (0.50, 0.75):
            hit, total = _recall(images, top_k, threshold)
            metrics[f"recall/{top_k}@{threshold:.2f}"] = hit / total if total else 0.0
    values = []
    for threshold in thresholds:
        hit, total = _recall(images, 100, threshold)
        values.append(hit / total if total else 0.0)
    metrics["ar/100"] = sum(values) / len(values)

    domains = sorted({image.domain for image in images})
    for domain in domains:
        subset = [image for image in images if image.domain == domain]
        hit, total = _recall(subset, 100, 0.50)
        metrics[f"recall/domain_{domain}/100@0.50"] = hit / total if total else 0.0

    matched: list[Tensor] = []
    pre_nms_loss: list[Tensor] = []
    for image in images:
        if image.ground_truth.numel() == 0:
            continue
        matched.append(_best_overlaps(image.ground_truth, image.detection.quads[:100]))
        if image.pre_nms_detection is not None:
            pre_nms_loss.append(
                _best_overlaps(image.ground_truth, image.pre_nms_detection.quads[:100])
                - matched[-1]
            )
    metrics["matched_iou/median"] = float(torch.cat(matched).median()) if matched else 0.0
    metrics["nms/matched_iou_delta_mean"] = float(torch.cat(pre_nms_loss).mean()) if pre_nms_loss else 0.0
    return metrics
