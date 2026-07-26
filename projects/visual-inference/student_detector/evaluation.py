"""Class-agnostic proposal metrics used for Phase-3 model selection."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass

import torch
from torch import Tensor

from .decoder import Detection
from .geometry import box_iou


@dataclass(frozen=True)
class EvaluationImage:
    image_id: int
    domain: str
    camera_type: str
    image_size: tuple[int, int]
    ground_truth: Tensor
    ignore_boxes: Tensor
    detection: Detection


def _average_precision(
    images: list[EvaluationImage], iou_threshold: float
) -> float:
    total_gt = sum(image.ground_truth.shape[0] for image in images)
    if total_gt == 0:
        return 0.0
    all_scores: list[Tensor] = []
    all_true_positives: list[Tensor] = []
    all_false_positives: list[Tensor] = []
    for image in images:
        scores = image.detection.scores
        boxes = image.detection.boxes
        if scores.numel() == 0:
            continue
        keep = torch.ones(scores.shape[0], dtype=torch.bool)
        if image.ignore_boxes.numel():
            keep &= (
                box_iou(boxes, image.ignore_boxes).max(dim=1).values
                < iou_threshold
            )
        scores, boxes = scores[keep], boxes[keep]
        true_positive = torch.zeros(scores.shape[0])
        false_positive = torch.zeros(scores.shape[0])
        matched = torch.zeros(image.ground_truth.shape[0], dtype=torch.bool)
        for detection_index, box in enumerate(boxes):
            if image.ground_truth.numel() == 0:
                false_positive[detection_index] = 1
                continue
            overlaps = box_iou(
                box.unsqueeze(0), image.ground_truth
            ).squeeze(0)
            overlap, gt_index = overlaps.max(dim=0)
            if overlap >= iou_threshold and not matched[gt_index]:
                matched[gt_index] = True
                true_positive[detection_index] = 1
            else:
                false_positive[detection_index] = 1
        all_scores.append(scores)
        all_true_positives.append(true_positive)
        all_false_positives.append(false_positive)
    if not all_scores:
        return 0.0
    scores = torch.cat(all_scores)
    order = scores.argsort(descending=True)
    tp = torch.cat(all_true_positives)[order].cumsum(0)
    fp = torch.cat(all_false_positives)[order].cumsum(0)
    recall = tp / total_gt
    precision = tp / (tp + fp).clamp(min=1)
    samples = torch.linspace(0, 1, 101)
    interpolated = torch.stack([
        precision[recall >= threshold].max()
        if torch.any(recall >= threshold)
        else precision.new_zeros(())
        for threshold in samples
    ])
    return float(interpolated.mean())


def _recall(
    images: list[EvaluationImage],
    *,
    top_k: int,
    iou_threshold: float,
    domain: str | None = None,
    size_range: tuple[float, float] | None = None,
    radial_range: tuple[float, float] | None = None,
) -> tuple[int, int]:
    recalled = total = 0
    for image in images:
        if domain is not None and image.domain != domain:
            continue
        gt = image.ground_truth
        if gt.numel() == 0:
            continue
        keep = torch.ones(gt.shape[0], dtype=torch.bool, device=gt.device)
        if size_range is not None:
            area = (gt[:, 2] - gt[:, 0]) * (gt[:, 3] - gt[:, 1])
            keep &= (area >= size_range[0]) & (area < size_range[1])
        if radial_range is not None:
            height, width = image.image_size
            center = (gt[:, :2] + gt[:, 2:]) * 0.5
            normalized_radius = torch.sqrt(
                ((center[:, 0] - width / 2) / (width / 2)) ** 2
                + ((center[:, 1] - height / 2) / (height / 2)) ** 2
            ) / 2**0.5
            keep &= (
                (normalized_radius >= radial_range[0])
                & (normalized_radius < radial_range[1])
            )
        gt = gt[keep]
        total += gt.shape[0]
        if gt.numel() == 0 or image.detection.boxes.numel() == 0:
            continue
        proposals = image.detection.boxes[:top_k]
        best = box_iou(gt, proposals).max(dim=1).values
        recalled += int((best >= iou_threshold).sum())
    return recalled, total


def evaluate_proposals(images: list[EvaluationImage]) -> dict[str, float]:
    """Compute fixed-decoder AP and recall slices without category labels."""
    metrics: dict[str, float] = {
        "ap/50": _average_precision(images, 0.50),
        "ap/75": _average_precision(images, 0.75),
    }
    counts: dict[str, list[int]] = defaultdict(lambda: [0, 0])
    size_ranges = {
        "edge_small": (100, 256),
        "small": (256, 1_024),
        "medium": (1_024, 9_216),
        "large": (9_216, float("inf")),
    }
    radial_ranges = {
        "center": (0.0, 0.33),
        "middle": (0.33, 0.66),
        "border": (0.66, 1.01),
    }
    matched_ious: list[Tensor] = []
    for image in images:
        gt = image.ground_truth
        if gt.numel() == 0:
            continue
        proposals = image.detection.boxes[:100]
        overlaps = (
            box_iou(gt, proposals)
            if proposals.numel()
            else torch.empty((gt.shape[0], 0))
        )
        best_by_k = {
            top_k: (
                overlaps[:, :top_k].max(dim=1).values
                if overlaps.shape[1]
                else torch.zeros(gt.shape[0])
            )
            for top_k in (10, 50, 100)
        }
        matched_ious.append(best_by_k[100])
        for threshold in (0.50, 0.75):
            for top_k in (10, 50, 100):
                key = f"recall/{top_k}@{int(threshold * 100)}"
                counts[key][0] += int(
                    (best_by_k[top_k] >= threshold).sum()
                )
                counts[key][1] += gt.shape[0]
        domain_key = f"recall/{image.domain}/100@50"
        domain_hits = best_by_k[100] >= 0.50
        counts[domain_key][0] += int(domain_hits.sum())
        counts[domain_key][1] += gt.shape[0]

        area = (gt[:, 2] - gt[:, 0]) * (gt[:, 3] - gt[:, 1])
        for name, (minimum, maximum) in size_ranges.items():
            mask = (area >= minimum) & (area < maximum)
            key = f"recall/size_{name}/100@50"
            counts[key][0] += int((domain_hits & mask).sum())
            counts[key][1] += int(mask.sum())
        if image.camera_type == "fisheye":
            height, width = image.image_size
            centers = (gt[:, :2] + gt[:, 2:]) * 0.5
            radius = torch.sqrt(
                ((centers[:, 0] - width / 2) / (width / 2)) ** 2
                + ((centers[:, 1] - height / 2) / (height / 2)) ** 2
            ) / 2**0.5
            for name, (minimum, maximum) in radial_ranges.items():
                mask = (radius >= minimum) & (radius < maximum)
                key = f"recall/fisheye_{name}/100@50"
                counts[key][0] += int((domain_hits & mask).sum())
                counts[key][1] += int(mask.sum())
    for key, (recalled, total) in counts.items():
        metrics[key] = recalled / total if total else 0.0
    for name in size_ranges:
        metrics.setdefault(f"recall/size_{name}/100@50", 0.0)
    for name in radial_ranges:
        metrics.setdefault(f"recall/fisheye_{name}/100@50", 0.0)
    domains = sorted({image.domain for image in images})
    domain_values = [
        metrics[f"recall/{domain}/100@50"]
        for domain in domains
        if f"recall/{domain}/100@50" in metrics
    ]
    metrics["selection_score"] = (
        sum(domain_values) / len(domain_values) if domain_values else 0.0
    )
    metrics["matched_iou/median"] = (
        float(torch.cat(matched_ious).median()) if matched_ious else 0.0
    )
    return metrics
