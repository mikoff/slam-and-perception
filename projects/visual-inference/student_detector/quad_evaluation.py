"""Fixed-budget polygon recall metrics for quad proposals."""

from __future__ import annotations

from dataclasses import dataclass

import torch
from torch import Tensor

from .quad_decoder import QuadDetection
from .quad_geometry import (
    aligned_quad_iou,
    equivalent_quad_traversals,
    pairwise_quad_iou,
    quad_area,
    quad_validity,
)


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
    object_conditions: tuple[str, ...] = ()
    seen_statuses: tuple[str, ...] = ()
    size_bins: tuple[str, ...] = ()
    aspect_bins: tuple[str, ...] = ()
    radial_bins: tuple[str, ...] = ()
    dense_detection: QuadDetection | None = None
    assigned_detection: QuadDetection | None = None
    assigned_gt_indices: Tensor | None = None
    positive_scores: Tensor | None = None
    trusted_background_scores: Tensor | None = None
    weak_background_scores: Tensor | None = None


def _best_overlaps(gt: Tensor, proposals: Tensor) -> Tensor:
    if gt.numel() == 0 or proposals.numel() == 0:
        return gt.new_zeros((gt.shape[0],))
    device = gt.device
    return pairwise_quad_iou(gt.to(device), proposals.to(device)).max(dim=1).values.to(gt.device)


def _overlap_matrix(gt: Tensor, proposals: Tensor) -> Tensor:
    if gt.numel() == 0 or proposals.numel() == 0:
        return gt.new_zeros((gt.shape[0], proposals.shape[0]))
    device = gt.device
    return pairwise_quad_iou(gt.to(device), proposals.to(device)).to(gt.device)


def _cached_best(matrix: Tensor, top_k: int) -> Tensor:
    if matrix.shape[1] == 0:
        return matrix.new_zeros((matrix.shape[0],))
    return matrix[:, :top_k].max(dim=1).values


def _recall(
    images: list[QuadEvaluationImage],
    top_k: int,
    threshold: float,
    overlap_cache: dict[int, Tensor] | None = None,
) -> tuple[int, int]:
    recalled = total = 0
    for image in images:
        if image.ground_truth.numel() == 0:
            continue
        total += image.ground_truth.shape[0]
        best = (
            _cached_best(overlap_cache[image.image_id], top_k)
            if overlap_cache is not None
            else _best_overlaps(image.ground_truth, image.detection.quads[:top_k])
        )
        recalled += int((best >= threshold).sum())
    return recalled, total


def _sliced_recall(
    images: list[QuadEvaluationImage],
    labels_name: str,
    label: str,
    top_k: int,
    threshold: float,
    overlap_cache: dict[int, Tensor] | None = None,
) -> tuple[int, int]:
    recalled = total = 0
    for image in images:
        labels = getattr(image, labels_name)
        if not labels:
            continue
        mask = torch.tensor([value == label for value in labels], dtype=torch.bool)
        if not mask.any():
            continue
        gt = image.ground_truth[mask]
        total += gt.shape[0]
        if overlap_cache is not None:
            best = _cached_best(overlap_cache[image.image_id][mask], top_k)
        else:
            best = _best_overlaps(gt, image.detection.quads[:top_k])
        recalled += int((best >= threshold).sum())
    return recalled, total


def _assigned_best(image: QuadEvaluationImage) -> Tensor:
    result = image.ground_truth.new_zeros((image.ground_truth.shape[0],))
    if image.assigned_detection is None or image.assigned_gt_indices is None:
        return result
    device = result.device
    for gt_index, target in enumerate(image.ground_truth):
        owned = image.assigned_gt_indices == gt_index
        if owned.any():
            proposals = image.assigned_detection.quads[owned]
            result[gt_index] = aligned_quad_iou(
                proposals.to(device), target.to(device).expand_as(proposals).to(device)
            ).max().to(result.device)
    return result


def _normalized_corner_errors(image: QuadEvaluationImage) -> Tensor:
    errors = image.ground_truth.new_full(
        (image.ground_truth.shape[0],), float("inf")
    )
    if image.assigned_detection is None or image.assigned_gt_indices is None:
        return errors
    device = errors.device
    for gt_index, target in enumerate(image.ground_truth):
        owned = image.assigned_gt_indices == gt_index
        if not owned.any():
            continue
        predictions = image.assigned_detection.quads[owned].to(device)
        traversals = equivalent_quad_traversals(target.to(device))
        distances = torch.linalg.vector_norm(
            predictions[:, None] - traversals[None], dim=-1
        ).mean(dim=-1)
        errors[gt_index] = distances.amin().to(errors.device) / torch.sqrt(quad_area(target.to(device)).clamp(min=1e-7)).to(errors.device)
    return errors


def _quantiles(values: list[Tensor], prefix: str, metrics: dict[str, float]) -> None:
    nonempty = [
        value.flatten() for value in values if value is not None and value.numel()
    ]
    if not nonempty:
        metrics[f"score/{prefix}/count"] = 0.0
        return
    joined = torch.cat(nonempty).float()
    metrics[f"score/{prefix}/count"] = float(joined.numel())
    for probability, name in ((0.1, "p10"), (0.5, "p50"), (0.9, "p90")):
        metrics[f"score/{prefix}/{name}"] = float(torch.quantile(joined, probability))


from datetime import datetime


@torch.no_grad()
def evaluate_quad_proposals(
    images: list[QuadEvaluationImage],
    *,
    log_interval: int = 500,
) -> dict[str, float]:
    """Compute AR over IoU .50:.95 and required fixed-budget slices."""
    metrics: dict[str, float] = {}
    post_cache: dict[int, Tensor] = {}
    for index, image in enumerate(images):
        post_cache[image.image_id] = _overlap_matrix(image.ground_truth, image.detection.quads)
        if log_interval and ((index + 1) % log_interval == 0 or index + 1 == len(images)):
            ts = datetime.now().strftime("%H:%M:%S")
            print(f"[{ts}] [Evaluation Math] Computed overlap matrix for {index + 1}/{len(images)} images", flush=True)

    pre_cache: dict[int, Tensor] = {}
    for index, image in enumerate(images):
        if image.pre_nms_detection is not None:
            pre_cache[image.image_id] = _overlap_matrix(
                image.ground_truth, image.pre_nms_detection.quads
            )
            if log_interval and ((index + 1) % log_interval == 0 or index + 1 == len(images)):
                ts = datetime.now().strftime("%H:%M:%S")
                print(f"[{ts}] [Evaluation Math] Computed pre-NMS overlap matrix for {index + 1}/{len(images)} images", flush=True)

    thresholds = [0.50 + 0.05 * index for index in range(10)]
    for top_k in (50, 100, 300):
        for threshold in (0.50, 0.75):
            hit, total = _recall(images, top_k, threshold, post_cache)
            metrics[f"recall/{top_k}@{threshold:.2f}"] = hit / total if total else 0.0
    values = []
    for threshold in thresholds:
        hit, total = _recall(images, 100, threshold, post_cache)
        values.append(hit / total if total else 0.0)
    metrics["ar/100"] = sum(values) / len(values)

    domains = sorted({image.domain for image in images})
    for domain in domains:
        subset = [image for image in images if image.domain == domain]
        hit, total = _recall(subset, 100, 0.50, post_cache)
        metrics[f"recall/domain_{domain}/100@0.50"] = hit / total if total else 0.0

    for labels_name, prefix in (
        ("geometry_tiers", "geometry_tier"),
        ("object_conditions", "condition"),
        ("seen_statuses", "seen_status"),
        ("size_bins", "size"),
        ("aspect_bins", "aspect"),
        ("radial_bins", "radial"),
    ):
        labels = sorted({
            label for image in images for label in getattr(image, labels_name)
        })
        for label in labels:
            for threshold in (0.50, 0.75):
                hit, total = _sliced_recall(
                    images, labels_name, label, 100, threshold, post_cache
                )
                metrics[f"recall/{prefix}_{label}/100@{threshold:.2f}"] = (
                    hit / total if total else 0.0
                )

    matched: list[Tensor] = []
    matched_scores: list[Tensor] = []
    pre_nms_loss: list[Tensor] = []
    assigned_oracle: list[Tensor] = []
    dense_oracle: list[Tensor] = []
    corner_errors: list[Tensor] = []
    for image in images:
        if image.ground_truth.numel() == 0:
            continue
        matched.append(_cached_best(post_cache[image.image_id], 100))
        if image.detection.quads.numel():
            overlaps = post_cache[image.image_id][:, :100]
            for row in overlaps:
                matched_scores.append(
                    image.detection.scores[:100][row.argmax()].reshape(1)
                )
        else:
            matched_scores.append(
                image.ground_truth.new_zeros((image.ground_truth.shape[0],))
            )
        if image.assigned_detection is not None:
            assigned_oracle.append(_assigned_best(image))
            corner_errors.append(_normalized_corner_errors(image))
        if image.dense_detection is not None:
            dense_oracle.append(
                _best_overlaps(image.ground_truth, image.dense_detection.quads)
            )
        if image.pre_nms_detection is not None:
            pre_nms_loss.append(
                _cached_best(pre_cache[image.image_id], 100)
                - matched[-1]
            )
    metrics["matched_iou/median"] = float(torch.cat(matched).median()) if matched else 0.0
    metrics["matched_score/median"] = (
        float(torch.cat(matched_scores).median()) if matched_scores else 0.0
    )
    metrics["nms/matched_iou_delta_mean"] = (
        float(torch.cat(pre_nms_loss).mean()) if pre_nms_loss else 0.0
    )
    for threshold in (0.50, 0.75):
        post_hit, total = _recall(images, 100, threshold, post_cache)
        pre_hit = 0
        for image in images:
            if image.pre_nms_detection is None or image.ground_truth.numel() == 0:
                continue
            pre_hit += int((
                _cached_best(pre_cache[image.image_id], 100) >= threshold
            ).sum())
        metrics[f"nms/recall_delta/100@{threshold:.2f}"] = (
            (pre_hit - post_hit) / total if total else 0.0
        )
    if assigned_oracle:
        joined = torch.cat(assigned_oracle)
        metrics["oracle/assigned_recall@0.50"] = float(
            (joined >= 0.50).float().mean()
        )
        metrics["oracle/assigned_recall@0.75"] = float(
            (joined >= 0.75).float().mean()
        )
        metrics["oracle/assigned_iou_median"] = float(joined.median())
    if dense_oracle:
        joined = torch.cat(dense_oracle)
        metrics["oracle/dense_recall@0.50"] = float((joined >= 0.50).float().mean())
        metrics["oracle/dense_recall@0.75"] = float((joined >= 0.75).float().mean())
        metrics["oracle/dense_iou_median"] = float(joined.median())
    if corner_errors:
        finite_errors = torch.cat(corner_errors)
        finite_errors = finite_errors[torch.isfinite(finite_errors)]
        metrics["corner_error/normalized_median"] = (
            float(finite_errors.median()) if finite_errors.numel() else 0.0
        )
    candidates = sum(
        getattr(image.detection, "candidate_count", 0) for image in images
    )
    invalid = sum(
        getattr(image.detection, "invalid_candidate_count", 0) for image in images
    )
    metrics["decoder/topk_invalid_rate"] = invalid / candidates if candidates else 0.0
    metrics["decoder/post_nms_invalid_count"] = float(sum(
        int((~quad_validity(image.detection.quads)).sum())
        for image in images
        if image.detection.quads.numel()
    ))
    _quantiles([image.positive_scores for image in images], "positive", metrics)
    _quantiles(
        [image.trusted_background_scores for image in images],
        "trusted_background",
        metrics,
    )
    _quantiles([image.weak_background_scores for image in images], "weak_background", metrics)
    return metrics
