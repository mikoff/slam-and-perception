"""Sweep matched HBB/quad score and NMS thresholds on cached candidates."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import sqlite3
import time
from typing import Any

import torch
from torch.utils.data import DataLoader, Subset
from torchvision.ops import box_iou

from student_detector.checkpoints import checkpoint_neck_type, load_model_state_strict
from student_detector.config import load_phase3_config
from student_detector.data import select_source_mixture_indices
from student_detector.decoder import Detection, InferenceDecoder
from student_detector.head import QuadDetectorOutput
from student_detector.model import QuadProposalDetector, StudentDetector
from student_detector.proposal_utility import (
    ProposalUtilityAccumulator,
    ScoreReliabilityAccumulator,
    proposal_region_fraction,
)
from student_detector.quad_data import (
    QuadProposalDataset,
    collate_quad_proposal_samples,
)
from student_detector.quad_decoder import QuadDetection, QuadInferenceDecoder
from student_detector.quad_geometry import (
    canonicalize_quads,
    pairwise_quad_iou,
    quad_validity,
    warmup_compiled_quad_iou,
)
from student_detector.quad_targets import point_validity_from_pixel_mask
from student_detector.suppression import (
    SuppressionSweepAccumulator,
    greedy_nms_from_overlaps,
)


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--hbb-checkpoint", type=Path, required=True)
    parser.add_argument("--quad-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--validation-images", type=int, default=800)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--device", default="cuda")
    parser.add_argument(
        "--checkpoint-state", choices=("model", "ema_model"), default="ema_model"
    )
    parser.add_argument("--nms-thresholds", default="0.5,0.6,0.7,0.8,0.9")
    parser.add_argument("--score-thresholds", default="0,0.2,0.3,0.4,0.5,0.6")
    parser.add_argument("--log-interval", type=int, default=100)
    return parser.parse_args()


def _thresholds(value: str, name: str) -> tuple[float, ...]:
    try:
        result = tuple(sorted({float(item.strip()) for item in value.split(",")}))
    except ValueError as error:
        raise ValueError(f"{name} thresholds must be comma-separated numbers") from error
    if not result or any(not 0 <= item <= 1 for item in result):
        raise ValueError(f"{name} thresholds must be in [0, 1]")
    return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _box_quads(boxes: torch.Tensor) -> torch.Tensor:
    if not boxes.numel():
        return boxes.new_empty((0, 4, 2))
    x1, y1, x2, y2 = boxes.unbind(dim=1)
    return torch.stack(
        (
            torch.stack((x1, y1), dim=1),
            torch.stack((x2, y1), dim=1),
            torch.stack((x2, y2), dim=1),
            torch.stack((x1, y2), dim=1),
        ),
        dim=1,
    )


def _sync(device: torch.device) -> None:
    if device.type == "cuda":
        torch.cuda.synchronize(device)


def _ego_source_quads(
    index_path: Path, image_ids: list[int]
) -> dict[int, torch.Tensor]:
    result: dict[int, list[Any]] = defaultdict(list)
    placeholders = ",".join("?" for _ in image_ids)
    with sqlite3.connect(
        f"file:{index_path}?mode=ro&immutable=1", uri=True
    ) as connection:
        rows = connection.execute(
            f"""
            SELECT image_id, quad_json FROM annotations
            WHERE category_name='ego_platform_bodywork'
              AND image_id IN ({placeholders})
            """,
            image_ids,
        )
    for image_id, encoded in rows:
        result[int(image_id)].append(json.loads(encoded))
    return {
        image_id: torch.tensor(quads, dtype=torch.float32).reshape(-1, 4, 2)
        for image_id, quads in result.items()
    }


def _transform_ego(
    source_quads: torch.Tensor | None,
    transform: tuple[float, float, float],
    input_size: int,
    device: torch.device,
) -> torch.Tensor:
    if source_quads is None or not source_quads.numel():
        return torch.empty((0, 4, 2), dtype=torch.float32, device=device)
    scale, offset_x, offset_y = transform
    quads = source_quads.to(device) * scale
    quads[..., 0] += offset_x
    quads[..., 1] += offset_y
    quads.clamp_(0, input_size)
    quads = canonicalize_quads(quads)
    return quads[quad_validity(quads)]


def _source_size_bands(quads: torch.Tensor, scale: float) -> tuple[str, ...]:
    extent = (quads.amax(dim=1) - quads.amin(dim=1)) / scale
    short = extent.amin(dim=1)
    edges = (0, 4, 8, 16, 32, 64, 128, 256, float("inf"))
    return tuple(
        next(
            f"[{lower:g},{upper:g})"
            for lower, upper in zip(edges, edges[1:], strict=True)
            if lower <= value < upper
        )
        for value in short.tolist()
    )


def _merge_metrics(
    utility: ProposalUtilityAccumulator, suppression: SuppressionSweepAccumulator
) -> dict[str, float | int]:
    result = utility.compute()
    for key, value in suppression.compute().items():
        if key not in result:
            result[key] = value
    return result


def _valid_levels(
    valid_mask: torch.Tensor,
    output_levels: tuple[torch.Tensor, ...],
    strides: tuple[int, ...],
    device: torch.device,
) -> tuple[torch.Tensor, ...]:
    shapes = tuple((item.shape[-2], item.shape[-1]) for item in output_levels)
    levels = point_validity_from_pixel_mask(valid_mask.to(device), shapes, strides)[1]
    return tuple(level.unsqueeze(0) for level in levels)


def _load_model(
    kind: str, checkpoint_path: Path, state: str, device: torch.device
) -> torch.nn.Module:
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    neck_type = checkpoint_neck_type(checkpoint)
    model: torch.nn.Module
    if kind == "hbb":
        model = StudentDetector(pretrained_backbone=False, neck_type=neck_type)
    else:
        model = QuadProposalDetector(pretrained_backbone=False, neck_type=neck_type)
    load_model_state_strict(
        model,
        checkpoint,
        kind=kind,
        neck_type=neck_type,
        state_key=state,
    )
    return model.to(device).eval()


def _candidate_tensors(
    kind: str, detection: Detection | QuadDetection
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    if kind == "hbb":
        if not isinstance(detection, Detection):
            raise TypeError("HBB sweep received a non-HBB detection")
        if detection.pre_nms_boxes is None or detection.pre_nms_scores is None:
            raise RuntimeError("HBB decoder did not retain pre-NMS candidates")
        boxes = detection.pre_nms_boxes.float()
        return boxes, _box_quads(boxes), detection.pre_nms_scores.float()
    if not isinstance(detection, QuadDetection):
        raise TypeError("quad sweep received a non-quad detection")
    if detection.pre_nms_quads is None or detection.pre_nms_scores is None:
        raise RuntimeError("quad decoder did not retain pre-NMS candidates")
    quads = detection.pre_nms_quads.float()
    return quads, quads, detection.pre_nms_scores.float()


def _pareto_keys(results: dict[str, dict[str, float | int]]) -> list[str]:
    keys = []
    for key, metrics in results.items():
        values = (
            float(metrics["ar/100"]),
            float(metrics["duplicates/100@0.50_fraction"]),
            float(metrics["proposals/100_per_image"]),
        )
        dominated = False
        for other_key, other in results.items():
            if other_key == key:
                continue
            other_values = (
                float(other["ar/100"]),
                float(other["duplicates/100@0.50_fraction"]),
                float(other["proposals/100_per_image"]),
            )
            no_worse = (
                other_values[0] >= values[0]
                and other_values[1] <= values[1]
                and other_values[2] <= values[2]
            )
            if no_worse and other_values != values:
                dominated = True
                break
        if not dominated:
            keys.append(key)
    return keys


@torch.inference_mode()
def _evaluate_kind(
    *,
    kind: str,
    checkpoint: Path,
    state: str,
    loader: DataLoader,
    config: Any,
    device: torch.device,
    nms_thresholds: tuple[float, ...],
    score_thresholds: tuple[float, ...],
    ego_by_image: dict[int, torch.Tensor],
    log_interval: int,
) -> dict[str, Any]:
    model = _load_model(kind, checkpoint, state, device)
    if kind == "hbb":
        decoder: Any = InferenceDecoder(
            strides=config.assignment.strides,
            top_k=config.inference.pre_nms_top_k,
            nms_iou_threshold=1.0,
            max_detections=config.inference.pre_nms_top_k,
            score_mode=config.inference.score_mode,
        )
    else:
        decoder = QuadInferenceDecoder(
            strides=config.assignment.strides,
            pre_nms_top_k=config.inference.pre_nms_top_k,
            nms_iou_threshold=1.0,
            max_proposals=config.inference.pre_nms_top_k,
        )
    settings: dict[
        str, dict[str, tuple[ProposalUtilityAccumulator, SuppressionSweepAccumulator]]
    ] = {}
    reliability: dict[str, dict[str, ScoreReliabilityAccumulator]] = {}

    def setting_group(
        key: str, name: str
    ) -> tuple[ProposalUtilityAccumulator, SuppressionSweepAccumulator]:
        groups = settings.setdefault(key, {})
        if name not in groups:
            groups[name] = (ProposalUtilityAccumulator(), SuppressionSweepAccumulator())
        return groups[name]

    def reliability_group(nms: float, name: str) -> ScoreReliabilityAccumulator:
        groups = reliability.setdefault(f"nms={nms:.2f}", {})
        if name not in groups:
            groups[name] = ScoreReliabilityAccumulator()
        return groups[name]
    forward_seconds = decode_seconds = overlap_seconds = 0.0
    candidates = invalid = seen = 0
    started_all = time.monotonic()
    for images, samples in loader:
        images = images.to(device, non_blocking=device.type == "cuda")
        _sync(device)
        started = time.perf_counter()
        output = model(images)
        _sync(device)
        forward_seconds += time.perf_counter() - started
        if kind == "quad":
            output = QuadDetectorOutput(
                tuple(item.float() for item in output.quality),
                tuple(item.float() for item in output.corner_offsets),
            )
            output_levels = output.quality
        else:
            output_levels = output.objectness
        masks = tuple(
            zip(
                *(
                    _valid_levels(
                        sample.valid_mask,
                        output_levels,
                        config.assignment.strides,
                        device,
                    )
                    for sample in samples
                )
            )
        )
        valid_masks = tuple(torch.cat(level, dim=0) for level in masks)
        _sync(device)
        started = time.perf_counter()
        detections = decoder(output, (images.shape[-2], images.shape[-1]), valid_masks)
        _sync(device)
        decode_seconds += time.perf_counter() - started
        for sample, detection in zip(samples, detections, strict=True):
            geometry, quads, scores = _candidate_tensors(kind, detection)
            candidates += detection.candidate_count
            invalid += detection.invalid_candidate_count
            gt = sample.quads.to(device)
            trusted = (
                sample.trusted_background_quads.to(device)
                if sample.trusted_background_quads is not None
                else gt.new_empty((0, 4, 2))
            )
            ignored = sample.ignore_quads.to(device)
            ego = _transform_ego(
                ego_by_image.get(sample.image_id),
                sample.transform,
                config.data.input_size,
                device,
            )
            _sync(device)
            started = time.perf_counter()
            proposal_overlaps = (
                box_iou(geometry, geometry)
                if kind == "hbb"
                else pairwise_quad_iou(quads, quads)
            )
            gt_overlaps = pairwise_quad_iou(gt, quads)
            trusted_fraction = proposal_region_fraction(
                pairwise_quad_iou(trusted, quads),
                trusted,
                quads,
                disjoint_regions=True,
            )
            ignore_fraction = proposal_region_fraction(
                pairwise_quad_iou(ignored, quads),
                ignored,
                quads,
                disjoint_regions=False,
            )
            ego_fraction = proposal_region_fraction(
                pairwise_quad_iou(ego, quads),
                ego,
                quads,
                disjoint_regions=False,
            )
            _sync(device)
            overlap_seconds += time.perf_counter() - started
            proposal_overlaps = proposal_overlaps.cpu()
            gt_overlaps = gt_overlaps.cpu()
            gt = gt.cpu()
            quads = quads.cpu()
            scores = scores.cpu()
            trusted_fraction = trusted_fraction.cpu()
            ignore_fraction = ignore_fraction.cpu()
            ego_fraction = ego_fraction.cpu()
            slices = {
                "size": sample.size_bins,
                "shape": tuple(
                    "regular" if label == "regular" else "thin"
                    for label in sample.aspect_bins
                ),
                "source_short_side": _source_size_bands(gt, sample.transform[0]),
            }
            group_names = (
                "aggregate",
                f"domain/{sample.domain}",
                f"source/{sample.source_dataset}",
            )
            for nms in nms_thresholds:
                full_keep = greedy_nms_from_overlaps(
                    proposal_overlaps,
                    scores,
                    nms,
                    max_output=100,
                )
                full_best_iou = (
                    gt_overlaps[:, full_keep].amax(dim=0)
                    if gt.shape[0]
                    else scores.new_zeros((full_keep.numel(),))
                )
                for name in group_names:
                    reliability_group(nms, name).update(
                        scores=scores[full_keep],
                        best_object_iou=full_best_iou,
                        trusted_fraction=trusted_fraction[full_keep],
                        ignore_fraction=ignore_fraction[full_keep],
                    )
                for score in score_thresholds:
                    key = f"nms={nms:.2f},score={score:.2f}"
                    eligible = torch.nonzero(scores >= score).flatten()
                    eligible_overlaps = proposal_overlaps[eligible][:, eligible]
                    replay_started = time.perf_counter()
                    keep_local = greedy_nms_from_overlaps(
                        eligible_overlaps,
                        scores[eligible],
                        nms,
                        max_output=100,
                    )
                    replay_seconds = time.perf_counter() - replay_started
                    keep = eligible[keep_local]
                    utility_args = {
                        "gt_overlaps": gt_overlaps[:, keep],
                        "gt_quads": gt,
                        "proposal_quads": quads[keep],
                        "trusted_fraction": trusted_fraction[keep],
                        "ignore_fraction": ignore_fraction[keep],
                        "ego_fraction": ego_fraction[keep],
                        "ego_present": bool(ego.numel()),
                        "slice_labels": slices,
                    }
                    for name in group_names:
                        utility, suppression = setting_group(key, name)
                        utility.update(**utility_args)
                        suppression.replay_seconds += replay_seconds
                        suppression.update(gt_overlaps, proposal_overlaps, keep)
            seen += 1
            if log_interval and seen % log_interval == 0:
                elapsed = time.monotonic() - started_all
                print(
                    f"{kind}: {seen}/{len(loader.dataset)} images "
                    f"({seen / elapsed:.2f} images/s)",
                    flush=True,
                )
    metrics_by_setting = {
        key: {
            name: _merge_metrics(*accumulators)
            for name, accumulators in sorted(groups.items())
        }
        for key, groups in settings.items()
    }
    metrics = {
        key: groups["aggregate"] for key, groups in metrics_by_setting.items()
    }
    del model
    if device.type == "cuda":
        torch.cuda.empty_cache()
    return {
        "checkpoint": str(checkpoint.resolve()),
        "checkpoint_sha256": _sha256(checkpoint),
        "images": seen,
        "candidate_count": candidates,
        "invalid_candidate_count": invalid,
        "invalid_candidate_fraction": invalid / candidates if candidates else 0.0,
        "forward_ms_per_image": 1000 * forward_seconds / max(seen, 1),
        "candidate_decode_ms_per_image": 1000 * decode_seconds / max(seen, 1),
        "overlap_matrix_ms_per_image": 1000 * overlap_seconds / max(seen, 1),
        "pareto_settings": _pareto_keys(metrics),
        "settings": metrics,
        "groups": {
            name: {
                key: groups[name]
                for key, groups in metrics_by_setting.items()
                if name in groups
            }
            for name in sorted(
                {name for groups in metrics_by_setting.values() for name in groups}
            )
        },
        "score_reliability_by_nms": {
            nms: {
                name: accumulator.compute()
                for name, accumulator in sorted(groups.items())
            }
            for nms, groups in sorted(reliability.items())
        },
    }


def main() -> None:
    args = _args()
    if args.validation_images < 1 or args.workers < 0 or args.log_interval < 0:
        raise ValueError("validation images must be positive; workers/log interval nonnegative")
    nms_thresholds = _thresholds(args.nms_thresholds, "NMS")
    score_thresholds = _thresholds(args.score_thresholds, "score")
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA sweep requested but no CUDA device is visible")
    config = load_phase3_config(args.config)
    annotations = config.data.quad_val_annotations or config.data.val_annotations
    dataset = QuadProposalDataset(
        annotations,
        config.data.image_root,
        config.data.index_dir / "quad_val.sqlite",
        config.data,
        config.augmentation,
        training=False,
        force_index=False,
    )
    selected = select_source_mixture_indices(
        dataset.records, config.data.source_weights, args.validation_images
    )
    ego_by_image = _ego_source_quads(
        config.data.index_dir / "quad_val.sqlite",
        [dataset.records[index].image_id for index in selected],
    )
    subset = Subset(dataset, selected)
    loader = DataLoader(
        subset,
        batch_size=config.data.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=device.type == "cuda",
        persistent_workers=args.workers > 0,
        collate_fn=collate_quad_proposal_samples,
    )
    if device.type == "cuda":
        warmup_compiled_quad_iou(device)
    hbb = _evaluate_kind(
        kind="hbb",
        checkpoint=args.hbb_checkpoint,
        state=args.checkpoint_state,
        loader=loader,
        config=config,
        device=device,
        nms_thresholds=nms_thresholds,
        score_thresholds=score_thresholds,
        ego_by_image=ego_by_image,
        log_interval=args.log_interval,
    )
    quad = _evaluate_kind(
        kind="quad",
        checkpoint=args.quad_checkpoint,
        state=args.checkpoint_state,
        loader=loader,
        config=config,
        device=device,
        nms_thresholds=nms_thresholds,
        score_thresholds=score_thresholds,
        ego_by_image=ego_by_image,
        log_interval=args.log_interval,
    )
    selected_identity = json.dumps(
        [
            (dataset.records[index].source_dataset, dataset.records[index].image_id)
            for index in selected
        ],
        separators=(",", ":"),
    ).encode()
    report = {
        "schema_version": "visual-inference-suppression-sweep.v1",
        "status": "measurement_complete_thresholds_not_owner_approved",
        "checkpoint_state": args.checkpoint_state,
        "device": str(device),
        "config": str(args.config.resolve()),
        "config_sha256": _sha256(args.config),
        "validation_manifest_sha256": _sha256(annotations),
        "validation_subset_sha256": hashlib.sha256(selected_identity).hexdigest(),
        "source_counts": dict(
            sorted(Counter(dataset.records[index].source_dataset for index in selected).items())
        ),
        "proposal_budget": 100,
        "pre_nms_top_k": config.inference.pre_nms_top_k,
        "nms_thresholds": nms_thresholds,
        "score_thresholds": score_thresholds,
        "hbb": hbb,
        "quad": quad,
    }
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Wrote matched suppression sweep to {output}")


if __name__ == "__main__":
    main()
