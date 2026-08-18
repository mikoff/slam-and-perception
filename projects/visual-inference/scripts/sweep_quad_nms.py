"""Sweep polygon-NMS thresholds on one quad validation inference pass."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
from dataclasses import dataclass, field
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time
from typing import Any

import torch
from torch import Tensor
from torch.utils.data import DataLoader

from student_detector.checkpoints import load_model_state_strict
from student_detector.config import load_phase3_config
from student_detector.head import QuadDetectorOutput
from student_detector.model import QuadProposalDetector
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_data import (
    QuadProposalDataset,
    collate_quad_proposal_samples,
)
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_geometry import pairwise_quad_iou, warmup_compiled_quad_iou
from student_detector.quad_targets import QuadTargetBuilder


_IOU_THRESHOLDS = tuple(0.50 + 0.05 * index for index in range(10))
_RECALL_BUDGETS = (50, 100)
_RECALL_IOUS = (0.50, 0.75)


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument(
        "--thresholds",
        default="0.5,0.6,0.7,0.8,0.85,0.9",
        help="Comma-separated polygon-NMS IoU thresholds",
    )
    parser.add_argument(
        "--state",
        choices=("auto", "raw", "ema"),
        default="auto",
        help="Weights to evaluate; auto honors selected_state when present",
    )
    parser.add_argument("--device", default="auto")
    parser.add_argument(
        "--batch-size",
        type=int,
        help="Defaults to data.batch_size from the config",
    )
    parser.add_argument(
        "--workers", type=int, help="Defaults to data.workers from the config"
    )
    parser.add_argument(
        "--amp",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use CUDA FP16 autocast for model inference (default: enabled)",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        help="Optional prefix for a smoke test; omit for the full validation set",
    )
    parser.add_argument("--log-interval", type=int, default=500)
    parser.add_argument(
        "--output", type=Path, default=Path("artifacts/phase3/nms_sweep.json")
    )
    return parser.parse_args()


def _parse_thresholds(value: str) -> tuple[float, ...]:
    try:
        thresholds = tuple(sorted({float(item.strip()) for item in value.split(",")}))
    except ValueError as error:
        raise ValueError("thresholds must be comma-separated numbers") from error
    if not thresholds or any(not 0 <= item <= 1 for item in thresholds):
        raise ValueError("NMS thresholds must be unique values in [0, 1]")
    return thresholds


def _checkpoint_state_key(checkpoint: dict[str, Any], requested: str) -> str:
    if requested == "raw":
        key = "model"
    elif requested == "ema":
        key = "ema_model"
    else:
        selected = checkpoint.get("selected_state")
        if selected in {"model", "ema_model"}:
            key = str(selected)
        else:
            key = "ema_model" if "ema_model" in checkpoint else "model"
    if key not in checkpoint:
        raise ValueError(f"checkpoint does not contain requested state {key!r}")
    return key


def _geometry_output_fp32(output: QuadDetectorOutput) -> QuadDetectorOutput:
    """Keep AMP in the network while making coordinate geometry overflow-safe."""
    return QuadDetectorOutput(
        quality=tuple(tensor.float() for tensor in output.quality),
        corner_offsets=tuple(tensor.float() for tensor in output.corner_offsets),
    )


def _nms_keep_from_overlaps(
    overlaps: Tensor,
    scores: Tensor,
    iou_threshold: float,
    *,
    max_output: int,
) -> Tensor:
    """Replay production's greedy NMS using a precomputed exact IoU matrix."""
    if overlaps.ndim != 2 or overlaps.shape[0] != overlaps.shape[1]:
        raise ValueError("NMS overlaps must be a square matrix")
    if scores.ndim != 1 or scores.shape[0] != overlaps.shape[0]:
        raise ValueError("NMS scores must match the overlap matrix")
    order = torch.argsort(scores.detach().cpu(), descending=True, stable=True)
    overlaps_cpu = overlaps.detach().cpu()
    suppressed = torch.zeros(overlaps.shape[0], dtype=torch.bool)
    kept: list[int] = []
    for index in order.tolist():
        if suppressed[index]:
            continue
        kept.append(index)
        suppressed |= overlaps_cpu[index] > iou_threshold
        if len(kept) >= max_output:
            break
    return torch.tensor(kept, dtype=torch.long)


@dataclass
class _SweepMetrics:
    total_gt: int = 0
    image_count: int = 0
    proposal_count: int = 0
    hits: dict[tuple[int, float], int] = field(default_factory=dict)

    def update(self, gt_overlaps: Tensor, keep: Tensor) -> None:
        self.image_count += 1
        self.total_gt += gt_overlaps.shape[0]
        self.proposal_count += keep.numel()
        for budget in _RECALL_BUDGETS:
            selected = keep[:budget]
            if gt_overlaps.shape[0] == 0 or selected.numel() == 0:
                best = gt_overlaps.new_zeros((gt_overlaps.shape[0],))
            else:
                best = gt_overlaps[:, selected].max(dim=1).values
            thresholds = _IOU_THRESHOLDS if budget == 100 else _RECALL_IOUS
            for threshold in thresholds:
                key = (budget, threshold)
                self.hits[key] = self.hits.get(key, 0) + int(
                    (best >= threshold).sum().item()
                )

    def compute(self) -> dict[str, float | int]:
        denominator = self.total_gt

        def recall(budget: int, threshold: float) -> float:
            return (
                self.hits.get((budget, threshold), 0) / denominator
                if denominator
                else 0.0
            )

        result: dict[str, float | int] = {
            "images": self.image_count,
            "ground_truth": self.total_gt,
            "proposals_per_image": (
                self.proposal_count / self.image_count if self.image_count else 0.0
            ),
            "ar/100": sum(recall(100, threshold) for threshold in _IOU_THRESHOLDS)
            / len(_IOU_THRESHOLDS),
        }
        for budget in _RECALL_BUDGETS:
            for threshold in _RECALL_IOUS:
                result[f"recall/{budget}@{threshold:.2f}"] = recall(budget, threshold)
        return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _print_table(results: dict[float, dict[str, float | int]]) -> None:
    print("\nNMS sweep results")
    print("threshold  AR@100   R@50/.50  R@50/.75  R@100/.50 R@100/.75 proposals/image")
    for threshold, metrics in results.items():
        print(
            f"{threshold:>9.2f}  {metrics['ar/100']:>7.4f}  "
            f"{metrics['recall/50@0.50']:>9.4f}  "
            f"{metrics['recall/50@0.75']:>9.4f}  "
            f"{metrics['recall/100@0.50']:>10.4f} "
            f"{metrics['recall/100@0.75']:>10.4f} "
            f"{metrics['proposals_per_image']:>15.1f}"
        )


@torch.inference_mode()
def main() -> None:
    args = _args()
    thresholds = _parse_thresholds(args.thresholds)
    if args.batch_size is not None and args.batch_size < 1:
        raise ValueError("batch size must be positive")
    if args.workers is not None and args.workers < 0:
        raise ValueError("workers cannot be negative")
    if args.max_images is not None and args.max_images < 1:
        raise ValueError("max-images must be positive")
    if args.log_interval < 0:
        raise ValueError("log-interval cannot be negative")

    config_path = args.config.resolve()
    checkpoint_path = args.checkpoint.resolve()
    config = load_phase3_config(config_path)
    if args.device == "auto":
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    else:
        device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is not available")

    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    state_key = _checkpoint_state_key(checkpoint, args.state)
    model = QuadProposalDetector(
        pretrained_backbone=False,
        neck_type=config.neck_type,
    ).to(device)
    load_model_state_strict(
        model,
        checkpoint,
        kind="quad",
        neck_type=config.neck_type,
        state_key=state_key,
    )
    model.eval()

    annotations = config.data.quad_val_annotations or config.data.val_annotations
    dataset = QuadProposalDataset(
        annotations,
        config.data.image_root,
        config.data.index_dir / "quad_val.sqlite",
        config.data,
        config.augmentation,
        training=False,
    )
    workers = config.data.workers if args.workers is None else args.workers
    loader = DataLoader(
        dataset,
        batch_size=args.batch_size or config.data.batch_size,
        shuffle=False,
        num_workers=workers,
        pin_memory=device.type == "cuda",
        persistent_workers=workers > 0,
        collate_fn=collate_quad_proposal_samples,
    )
    target_builder = QuadTargetBuilder(
        QuadAssigner(
            strides=config.assignment.strides,
            top_k=config.quad.top_k,
            gamma=config.quad.gamma,
            scale_sigma=config.quad.scale_sigma,
            eligible_levels=config.quad.eligible_levels,
            scale_measure=config.quad.scale_measure,
        )
    )
    # IoU=1 suppresses nothing. This still runs the production decoder's exact
    # ranking, validity, bounds, and canonicalization path once per image.
    pre_nms_decoder = QuadInferenceDecoder(
        strides=config.assignment.strides,
        pre_nms_top_k=config.inference.pre_nms_top_k,
        nms_iou_threshold=1.0,
        max_proposals=config.inference.pre_nms_top_k,
    )
    accumulators = {threshold: _SweepMetrics() for threshold in thresholds}

    print(
        f"Evaluating state={state_key} device={device} images={len(dataset)} "
        f"thresholds={','.join(f'{value:g}' for value in thresholds)}",
        flush=True,
    )
    if device.type == "cuda":
        print("Warming exact quadrilateral IoU kernel", flush=True)
        warmup_compiled_quad_iou(device)
    started = time.monotonic()
    seen = 0
    stop = False
    for images, samples in loader:
        images_device = images.to(device, non_blocking=device.type == "cuda")
        autocast = (
            torch.autocast(device_type="cuda", dtype=torch.float16)
            if device.type == "cuda" and args.amp
            else nullcontext()
        )
        with autocast:
            output = model(images_device)
        output = _geometry_output_fp32(output)
        shapes = tuple((level.shape[-2], level.shape[-1]) for level in output.quality)
        targets = target_builder(samples, shapes, device=device)
        detections = pre_nms_decoder(
            output,
            (images.shape[-2], images.shape[-1]),
            targets.valid_point_masks,
        )
        for sample, detection in zip(samples, detections, strict=True):
            if args.max_images is not None and seen >= args.max_images:
                stop = True
                break
            if detection.pre_nms_quads is None or detection.pre_nms_scores is None:
                raise RuntimeError(
                    "production decoder did not return pre-NMS candidates"
                )
            proposals = detection.pre_nms_quads
            scores = detection.pre_nms_scores
            proposal_overlaps = pairwise_quad_iou(
                proposals,
                proposals,
                minimum_iou=min(thresholds),
            ).cpu()
            ground_truth = sample.quads.to(device)
            gt_overlaps = pairwise_quad_iou(ground_truth, proposals).cpu()
            for threshold, accumulator in accumulators.items():
                keep = _nms_keep_from_overlaps(
                    proposal_overlaps,
                    scores,
                    threshold,
                    max_output=config.inference.max_proposals,
                )
                accumulator.update(gt_overlaps, keep)
            seen += 1
            if args.log_interval and seen % args.log_interval == 0:
                elapsed = time.monotonic() - started
                rate = seen / elapsed if elapsed else 0.0
                total = min(len(dataset), args.max_images or len(dataset))
                remaining = (total - seen) / rate if rate else 0.0
                timestamp = datetime.now().strftime("%H:%M:%S")
                print(
                    f"[{timestamp}] NMS sweep: {seen}/{total} images "
                    f"({rate:.1f} images/s, ETA {remaining / 60:.1f} min)",
                    flush=True,
                )
        if stop:
            break

    elapsed = time.monotonic() - started
    results = {
        threshold: accumulator.compute()
        for threshold, accumulator in accumulators.items()
    }
    report = {
        "schema_version": "visual-inference-quad-nms-sweep.v1",
        "config": str(config_path),
        "config_sha256": _sha256(config_path),
        "checkpoint": str(checkpoint_path),
        "checkpoint_sha256": _sha256(checkpoint_path),
        "checkpoint_state": state_key,
        "device": str(device),
        "amp": bool(device.type == "cuda" and args.amp),
        "pre_nms_top_k": config.inference.pre_nms_top_k,
        "max_proposals": config.inference.max_proposals,
        "partial": args.max_images is not None and seen < len(dataset),
        "elapsed_seconds": elapsed,
        "results": {
            f"{threshold:g}": metrics for threshold, metrics in results.items()
        },
    }
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    _print_table(results)
    print(f"\nWrote {output}")


if __name__ == "__main__":
    main()
