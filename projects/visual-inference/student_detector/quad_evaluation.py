"""Fixed-budget polygon recall metrics for quad proposals."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Iterable
from dataclasses import dataclass
from datetime import datetime
import math

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
    return (
        pairwise_quad_iou(gt.to(device), proposals.to(device))
        .max(dim=1)
        .values.to(gt.device)
    )


def _overlap_matrix(gt: Tensor, proposals: Tensor) -> Tensor:
    if gt.numel() == 0 or proposals.numel() == 0:
        return gt.new_zeros((gt.shape[0], proposals.shape[0]))
    device = gt.device
    return pairwise_quad_iou(gt.to(device), proposals.to(device)).to(gt.device)


def _cached_best(matrix: Tensor, top_k: int) -> Tensor:
    if matrix.shape[1] == 0:
        return matrix.new_zeros((matrix.shape[0],))
    return matrix[:, :top_k].max(dim=1).values


def _assigned_best(image: QuadEvaluationImage) -> Tensor:
    result = image.ground_truth.new_zeros((image.ground_truth.shape[0],))
    if image.assigned_detection is None or image.assigned_gt_indices is None:
        return result
    device = result.device
    for gt_index, target in enumerate(image.ground_truth):
        owned = image.assigned_gt_indices == gt_index
        if owned.any():
            proposals = image.assigned_detection.quads[owned]
            result[gt_index] = (
                aligned_quad_iou(
                    proposals.to(device),
                    target.to(device).expand_as(proposals).to(device),
                )
                .max()
                .to(result.device)
            )
    return result


def _normalized_corner_errors(image: QuadEvaluationImage) -> Tensor:
    errors = image.ground_truth.new_full((image.ground_truth.shape[0],), float("inf"))
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
        errors[gt_index] = distances.amin().to(errors.device) / torch.sqrt(
            quad_area(target.to(device)).clamp(min=1e-7)
        ).to(errors.device)
    return errors


class _ScoreHistogram:
    """Constant-memory quantiles for sigmoid scores in the closed unit interval."""

    _bins = 65_536

    def __init__(self) -> None:
        self.counts = torch.zeros(self._bins, dtype=torch.int64)
        self.total = 0

    def update(self, values: Tensor) -> None:
        scores = values.detach().flatten().float().cpu()
        if not scores.numel():
            return
        if not torch.isfinite(scores).all():
            raise FloatingPointError("non-finite validation quality score")
        if bool((scores < 0).any() or (scores > 1).any()):
            raise ValueError("validation quality scores must be in [0, 1]")
        indices = (scores * self._bins).to(torch.int64).clamp(max=self._bins - 1)
        self.counts += torch.bincount(indices, minlength=self._bins)
        self.total += scores.numel()

    def write_metrics(self, prefix: str, metrics: dict[str, float]) -> None:
        metrics[f"score/{prefix}/count"] = float(self.total)
        if not self.total:
            return
        cumulative = self.counts.cumsum(dim=0)
        for probability, name in ((0.1, "p10"), (0.5, "p50"), (0.9, "p90")):
            rank = probability * (self.total - 1)
            lower_rank = math.floor(rank)
            upper_rank = math.ceil(rank)
            lower_index = int(
                torch.searchsorted(cumulative, lower_rank + 1).item()
            )
            upper_index = int(
                torch.searchsorted(cumulative, upper_rank + 1).item()
            )
            lower = (lower_index + 0.5) / self._bins
            upper = (upper_index + 0.5) / self._bins
            metrics[f"score/{prefix}/{name}"] = lower + (rank - lower_rank) * (
                upper - lower
            )


class QuadEvaluationAccumulator:
    """Aggregate quad metrics while releasing per-image GPU tensors promptly."""

    _thresholds = tuple(0.50 + 0.05 * index for index in range(10))
    _slices = (
        ("geometry_tiers", "geometry_tier"),
        ("object_conditions", "condition"),
        ("seen_statuses", "seen_status"),
        ("size_bins", "size"),
        ("aspect_bins", "aspect"),
        ("radial_bins", "radial"),
    )

    def __init__(self, *, log_interval: int = 500, state: str = "model") -> None:
        self.log_interval = log_interval
        self.state = state
        self.image_count = 0
        self.total_gt = 0
        self.hits: dict[tuple[int, float], int] = defaultdict(int)
        self.domain_counts: dict[str, list[int]] = defaultdict(lambda: [0, 0])
        self.slice_counts: dict[tuple[str, str, float], list[int]] = defaultdict(
            lambda: [0, 0]
        )
        self.pre_hits: dict[float, int] = defaultdict(int)
        self.values: dict[str, list[Tensor]] = defaultdict(list)
        self.score_histograms = {
            name: _ScoreHistogram()
            for name in ("positive", "trusted_background", "weak_background")
        }
        self.candidates = 0
        self.invalid_candidates = 0
        self.post_nms_invalid = 0

    @staticmethod
    def _keep_cpu(values: Tensor) -> Tensor:
        return values.detach().flatten().cpu()

    @torch.no_grad()
    def update(self, images: Iterable[QuadEvaluationImage]) -> None:
        """Consume images and retain only counters and one-dimensional CPU values."""
        score_buffers: dict[str, list[Tensor]] = defaultdict(list)
        score_buffer_sizes: dict[str, int] = defaultdict(int)

        def flush_scores(name: str) -> None:
            if score_buffers[name]:
                self.score_histograms[name].update(torch.cat(score_buffers[name]))
                score_buffers[name].clear()
                score_buffer_sizes[name] = 0

        for image in images:
            self._update_image(image)
            for name in ("positive", "trusted_background", "weak_background"):
                scores = getattr(image, f"{name}_scores")
                if scores is None or not scores.numel():
                    continue
                score_buffers[name].append(scores.detach().flatten())
                score_buffer_sizes[name] += scores.numel()
                if score_buffer_sizes[name] >= 1_000_000:
                    flush_scores(name)
        for name in score_buffers:
            flush_scores(name)

    def _update_image(self, image: QuadEvaluationImage) -> None:
        self.image_count += 1
        post = _overlap_matrix(image.ground_truth, image.detection.quads)
        gt_count = image.ground_truth.shape[0]
        self.total_gt += gt_count

        best_by_k = {top_k: _cached_best(post, top_k) for top_k in (50, 100, 300)}
        for top_k in (50, 100, 300):
            for threshold in (0.50, 0.75):
                self.hits[top_k, threshold] += int(
                    (best_by_k[top_k] >= threshold).sum()
                )
        for threshold in self._thresholds:
            if threshold not in (0.50, 0.75):
                self.hits[100, threshold] += int((best_by_k[100] >= threshold).sum())

        domain = self.domain_counts[image.domain]
        domain[0] += int((best_by_k[100] >= 0.50).sum())
        domain[1] += gt_count
        for labels_name, prefix in self._slices:
            labels = getattr(image, labels_name)
            for label in set(labels):
                mask = torch.tensor(
                    [value == label for value in labels],
                    dtype=torch.bool,
                    device=image.ground_truth.device,
                )
                for threshold in (0.50, 0.75):
                    counts = self.slice_counts[prefix, label, threshold]
                    counts[0] += int((best_by_k[100][mask] >= threshold).sum())
                    counts[1] += int(mask.sum())

        if gt_count:
            matched = best_by_k[100]
            self.values["matched"].append(self._keep_cpu(matched))
            if image.detection.quads.numel():
                overlaps = post[:, :100]
                scores = image.detection.scores[:100]
                self.values["matched_scores"].append(
                    self._keep_cpu(scores[overlaps.argmax(dim=1)])
                )
            else:
                self.values["matched_scores"].append(torch.zeros(gt_count))
            if image.assigned_detection is not None:
                self.values["assigned"].append(self._keep_cpu(_assigned_best(image)))
                self.values["corner_errors"].append(
                    self._keep_cpu(_normalized_corner_errors(image))
                )
            if image.dense_detection is not None:
                self.values["dense"].append(
                    self._keep_cpu(
                        _best_overlaps(image.ground_truth, image.dense_detection.quads)
                    )
                )
            if image.pre_nms_detection is not None:
                pre = _overlap_matrix(image.ground_truth, image.pre_nms_detection.quads)
                pre_best = _cached_best(pre, 100)
                self.values["pre_nms_loss"].append(self._keep_cpu(pre_best - matched))
                for threshold in (0.50, 0.75):
                    self.pre_hits[threshold] += int((pre_best >= threshold).sum())

        self.candidates += getattr(image.detection, "candidate_count", 0)
        self.invalid_candidates += getattr(
            image.detection, "invalid_candidate_count", 0
        )
        if image.detection.quads.numel():
            self.post_nms_invalid += int((~quad_validity(image.detection.quads)).sum())
        if self.log_interval and self.image_count % self.log_interval == 0:
            timestamp = datetime.now().strftime("%H:%M:%S")
            print(
                f"[{timestamp}] [Evaluation Math] state={self.state} "
                f"aggregated={self.image_count} images",
                flush=True,
            )

    def _joined(self, name: str) -> Tensor | None:
        chunks = self.values[name]
        return torch.cat(chunks) if chunks else None

    def compute(self) -> dict[str, float]:
        """Finalize metrics without revisiting model outputs or proposal tensors."""
        metrics: dict[str, float] = {}
        for top_k in (50, 100, 300):
            for threshold in (0.50, 0.75):
                metrics[f"recall/{top_k}@{threshold:.2f}"] = (
                    self.hits[top_k, threshold] / self.total_gt
                    if self.total_gt
                    else 0.0
                )
        metrics["ar/100"] = sum(
            self.hits[100, threshold] / self.total_gt if self.total_gt else 0.0
            for threshold in self._thresholds
        ) / len(self._thresholds)
        for domain, (hit, total) in sorted(self.domain_counts.items()):
            metrics[f"recall/domain_{domain}/100@0.50"] = hit / total if total else 0.0
        for (prefix, label, threshold), (hit, total) in sorted(
            self.slice_counts.items()
        ):
            metrics[f"recall/{prefix}_{label}/100@{threshold:.2f}"] = (
                hit / total if total else 0.0
            )

        matched = self._joined("matched")
        matched_scores = self._joined("matched_scores")
        pre_nms_loss = self._joined("pre_nms_loss")
        metrics["matched_iou/median"] = (
            float(matched.median()) if matched is not None else 0.0
        )
        metrics["matched_score/median"] = (
            float(matched_scores.median()) if matched_scores is not None else 0.0
        )
        metrics["nms/matched_iou_delta_mean"] = (
            float(pre_nms_loss.mean()) if pre_nms_loss is not None else 0.0
        )
        for threshold in (0.50, 0.75):
            metrics[f"nms/recall_delta/100@{threshold:.2f}"] = (
                (self.pre_hits[threshold] - self.hits[100, threshold]) / self.total_gt
                if self.total_gt
                else 0.0
            )
        for name in ("assigned", "dense"):
            joined = self._joined(name)
            if joined is not None:
                metrics[f"oracle/{name}_recall@0.50"] = float(
                    (joined >= 0.50).float().mean()
                )
                metrics[f"oracle/{name}_recall@0.75"] = float(
                    (joined >= 0.75).float().mean()
                )
                metrics[f"oracle/{name}_iou_median"] = float(joined.median())
        corner_errors = self._joined("corner_errors")
        if corner_errors is not None:
            finite = corner_errors[torch.isfinite(corner_errors)]
            metrics["corner_error/normalized_median"] = (
                float(finite.median()) if finite.numel() else 0.0
            )
        metrics["decoder/topk_invalid_rate"] = (
            self.invalid_candidates / self.candidates if self.candidates else 0.0
        )
        metrics["decoder/post_nms_invalid_count"] = float(self.post_nms_invalid)
        for name in ("positive", "trusted_background", "weak_background"):
            self.score_histograms[name].write_metrics(name, metrics)
        return metrics


@torch.no_grad()
def evaluate_quad_proposals(
    images: list[QuadEvaluationImage],
    *,
    log_interval: int = 500,
) -> dict[str, float]:
    """Compute fixed-budget metrics through the streaming accumulator."""
    accumulator = QuadEvaluationAccumulator(log_interval=log_interval)
    accumulator.update(images)
    return accumulator.compute()
