"""Evaluate fixed-budget HBB and quad utility at approved suppression settings."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time
from typing import Any

import torch
from torch.utils.data import DataLoader, Subset
from student_detector.config import load_phase3_config
from student_detector.data import select_source_mixture_indices
from student_detector.decoder import Detection, InferenceDecoder, class_agnostic_nms
from student_detector.head import QuadDetectorOutput
from student_detector.proposal_evaluation import (
    boxes_to_quads,
    load_ego_source_quads,
    load_proposal_model,
    merge_utility_and_suppression_metrics,
    source_size_bands,
    synchronize_device,
    transform_ego_quads,
    valid_output_levels,
)
from student_detector.proposal_utility import (
    ProposalUtilityAccumulator,
    REGION_FRACTION_THRESHOLD,
    UNMATCHED_IOU_THRESHOLD,
    proposal_region_fraction,
)
from student_detector.quad_data import (
    QuadProposalDataset,
    collate_quad_proposal_samples,
)
from student_detector.quad_decoder import QuadDetection, QuadInferenceDecoder
from student_detector.quad_geometry import (
    pairwise_quad_iou,
    polygon_nms,
    warmup_compiled_quad_iou,
)
from student_detector.suppression import SuppressionSweepAccumulator


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--hbb-checkpoint", type=Path, required=True)
    parser.add_argument("--quad-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--validation-images", type=int, default=800)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--device", default="cuda")
    parser.add_argument(
        "--checkpoint-state", choices=("model", "ema_model"), default="ema_model"
    )
    parser.add_argument("--log-interval", type=int, default=100)
    return parser.parse_args()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_policy(path: Path) -> dict[str, Any]:
    import yaml

    policy = yaml.safe_load(path.read_text(encoding="utf-8"))
    if policy.get("schema_version") != "visual-inference-proposal-utility.v1":
        raise ValueError("unsupported proposal utility policy")
    return policy


def _candidate_tensors(
    kind: str, detection: Detection | QuadDetection
) -> tuple[torch.Tensor, torch.Tensor]:
    if kind == "hbb":
        if not isinstance(detection, Detection) or detection.pre_nms_boxes is None:
            raise RuntimeError("HBB decoder did not retain pre-NMS candidates")
        boxes = detection.pre_nms_boxes.float()
        if detection.pre_nms_scores is None:
            raise RuntimeError("HBB decoder did not retain pre-NMS scores")
        return boxes_to_quads(boxes), detection.pre_nms_scores.float()
    if not isinstance(detection, QuadDetection) or detection.pre_nms_quads is None:
        raise RuntimeError("quad decoder did not retain pre-NMS candidates")
    if detection.pre_nms_scores is None:
        raise RuntimeError("quad decoder did not retain pre-NMS scores")
    return detection.pre_nms_quads.float(), detection.pre_nms_scores.float()


@torch.inference_mode()
def _evaluate_kind(
    *,
    kind: str,
    checkpoint: Path,
    state: str,
    loader: DataLoader,
    config: Any,
    device: torch.device,
    nms_threshold: float,
    score_threshold: float,
    ego_by_image: dict[int, torch.Tensor],
    log_interval: int,
) -> dict[str, Any]:
    model = load_proposal_model(kind, checkpoint, state, device)
    decoder: Any = (
        InferenceDecoder(
            strides=config.assignment.strides,
            top_k=config.inference.pre_nms_top_k,
            nms_iou_threshold=1.0,
            max_detections=config.inference.pre_nms_top_k,
            score_mode=config.inference.score_mode,
        )
        if kind == "hbb"
        else QuadInferenceDecoder(
            strides=config.assignment.strides,
            pre_nms_top_k=config.inference.pre_nms_top_k,
            nms_iou_threshold=1.0,
            max_proposals=config.inference.pre_nms_top_k,
        )
    )
    groups: dict[
        str, tuple[ProposalUtilityAccumulator, SuppressionSweepAccumulator]
    ] = {}

    def group(
        name: str,
    ) -> tuple[ProposalUtilityAccumulator, SuppressionSweepAccumulator]:
        if name not in groups:
            groups[name] = (ProposalUtilityAccumulator(), SuppressionSweepAccumulator())
        return groups[name]

    forward_seconds = decode_seconds = nms_seconds = region_seconds = 0.0
    seen = 0
    started_all = time.monotonic()
    for images, samples in loader:
        images = images.to(device, non_blocking=device.type == "cuda")
        synchronize_device(device)
        started = time.perf_counter()
        output = model(images)
        synchronize_device(device)
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
                    valid_output_levels(
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
        synchronize_device(device)
        started = time.perf_counter()
        detections = decoder(output, (images.shape[-2], images.shape[-1]), valid_masks)
        synchronize_device(device)
        decode_seconds += time.perf_counter() - started
        for sample, detection in zip(samples, detections, strict=True):
            candidate_quads, scores = _candidate_tensors(kind, detection)
            eligible = torch.nonzero(scores >= score_threshold).flatten()
            started = time.perf_counter()
            if kind == "hbb":
                if not isinstance(detection, Detection):
                    raise TypeError("HBB evaluation received a non-HBB detection")
                keep_local = class_agnostic_nms(
                    detection.pre_nms_boxes.float()[eligible],
                    scores[eligible],
                    nms_threshold,
                )[:100]
            else:
                keep_local = polygon_nms(
                    candidate_quads[eligible], scores[eligible], nms_threshold
                )[:100]
            synchronize_device(device)
            nms_seconds += time.perf_counter() - started
            keep = eligible[keep_local]
            proposals = candidate_quads[keep]
            gt = sample.quads.to(device)
            trusted = sample.trusted_background_quads.to(device)
            ignored = sample.ignore_quads.to(device)
            ego = transform_ego_quads(
                ego_by_image.get(sample.image_id),
                sample.transform,
                config.data.input_size,
                device,
            )
            synchronize_device(device)
            started = time.perf_counter()
            gt_overlaps = pairwise_quad_iou(gt, proposals)
            trusted_overlaps = pairwise_quad_iou(trusted, proposals)
            ignore_overlaps = pairwise_quad_iou(ignored, proposals)
            ego_overlaps = pairwise_quad_iou(ego, proposals)
            trusted_fraction = proposal_region_fraction(
                trusted_overlaps, trusted, proposals, disjoint_regions=True
            )
            ignore_fraction = proposal_region_fraction(
                ignore_overlaps, ignored, proposals, disjoint_regions=False
            )
            ego_fraction = proposal_region_fraction(
                ego_overlaps, ego, proposals, disjoint_regions=False
            )
            synchronize_device(device)
            region_seconds += time.perf_counter() - started
            slices = {
                "size": sample.size_bins,
                "shape": tuple(
                    "regular" if label == "regular" else "thin"
                    for label in sample.aspect_bins
                ),
                "source_short_side": source_size_bands(gt, sample.transform[0]),
            }
            utility_args = {
                "gt_overlaps": gt_overlaps,
                "gt_quads": gt,
                "proposal_quads": proposals,
                "trusted_fraction": trusted_fraction,
                "ignore_fraction": ignore_fraction,
                "ego_fraction": ego_fraction,
                "ego_present": bool(ego.numel()),
                "slice_labels": slices,
            }
            for name in (
                "aggregate",
                f"domain/{sample.domain}",
                f"source/{sample.source_dataset}",
            ):
                utility, suppression = group(name)
                utility.update(**utility_args)
                suppression.update(
                    gt_overlaps,
                    pairwise_quad_iou(proposals, proposals),
                    torch.arange(proposals.shape[0], device=proposals.device),
                )
            seen += 1
            if log_interval and seen % log_interval == 0:
                elapsed = time.monotonic() - started_all
                print(
                    f"{kind}: {seen}/{len(loader.dataset)} images "
                    f"({seen / elapsed:.2f} images/s)",
                    flush=True,
                )
    del model
    if device.type == "cuda":
        torch.cuda.empty_cache()
    return {
        "checkpoint": str(checkpoint.resolve()),
        "checkpoint_sha256": _sha256(checkpoint),
        "images": seen,
        "thresholds": {"nms_iou": nms_threshold, "score": score_threshold},
        "timing": {
            "neural_inference_ms_per_image": 1000 * forward_seconds / max(seen, 1),
            "candidate_decode_ms_per_image": 1000 * decode_seconds / max(seen, 1),
            "nms_ms_per_image": 1000 * nms_seconds / max(seen, 1),
            "utility_overlap_ms_per_image": 1000 * region_seconds / max(seen, 1),
            "crop_generation_ms_per_image": None,
            "timing_scope": "CPU measurement; utility overlap is evaluation-only",
        },
        "groups": {
            name: merge_utility_and_suppression_metrics(*accumulators)
            for name, accumulators in sorted(groups.items())
        },
    }


def main() -> None:
    args = _args()
    if args.validation_images < 1 or args.workers < 0 or args.log_interval < 0:
        raise ValueError(
            "validation images must be positive; workers/log interval nonnegative"
        )
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError(
            "CUDA utility evaluation requested but no CUDA device is visible"
        )
    config = load_phase3_config(args.config)
    policy = _load_policy(args.policy)
    if args.config.name != policy["base_config"]:
        raise ValueError("utility policy does not name the supplied base config")
    if args.checkpoint_state != policy["checkpoint_state"]:
        raise ValueError("checkpoint state does not match the utility policy")
    if args.validation_images != int(policy["validation_images"]):
        raise ValueError("validation image count does not match the utility policy")
    if config.inference.pre_nms_top_k != int(policy["pre_nms_top_k"]):
        raise ValueError("pre-NMS budget does not match the utility policy")
    if int(policy["proposal_budget"]) != 100:
        raise ValueError("this evaluator implements the approved K=100 contract")
    false_policy = policy["false_proposal"]
    if float(false_policy["maximum_object_iou"]) != UNMATCHED_IOU_THRESHOLD:
        raise ValueError("unmatched threshold does not match the implemented contract")
    if float(false_policy["minimum_region_fraction"]) != REGION_FRACTION_THRESHOLD:
        raise ValueError("region threshold does not match the implemented contract")
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
    ego_by_image = load_ego_source_quads(
        dataset.index_path, [dataset.records[index].image_id for index in selected]
    )
    if device.type == "cuda":
        warmup_compiled_quad_iou(device)
    results = {}
    for kind, checkpoint in (
        ("hbb", args.hbb_checkpoint),
        ("quad", args.quad_checkpoint),
    ):
        geometry_policy = policy["geometry"][kind]
        results[kind] = _evaluate_kind(
            kind=kind,
            checkpoint=checkpoint,
            state=args.checkpoint_state,
            loader=loader,
            config=config,
            device=device,
            nms_threshold=float(geometry_policy["nms_iou_threshold"]),
            score_threshold=float(geometry_policy["score_threshold"]),
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
        "schema_version": "visual-inference-proposal-utility-result.v1",
        "status": "measurement_complete_owner_review_required",
        "checkpoint_state": args.checkpoint_state,
        "device": str(device),
        "config": str(args.config.resolve()),
        "config_sha256": _sha256(args.config),
        "policy": str(args.policy.resolve()),
        "policy_sha256": _sha256(args.policy),
        "validation_manifest_sha256": _sha256(annotations),
        "validation_subset_sha256": hashlib.sha256(selected_identity).hexdigest(),
        "source_counts": dict(
            sorted(
                Counter(
                    dataset.records[index].source_dataset for index in selected
                ).items()
            )
        ),
        "metric_contract": {
            "unmatched": "maximum object IoU < 0.10",
            "ignore_exempt": "at least 50% of proposal inside one ignore polygon",
            "trusted_background_false": "unmatched, not ignore-exempt, and at least 50% inside trusted-background union",
            "object_fraction": "fraction of proposal area inside its best-overlapping labeled object",
            "object_coverage": "fraction of each labeled object's area covered by its best proposal",
        },
        "category_background_rates": {
            "ego_body": "measured from retained ego_platform_bodywork ignore polygons",
            "sky": "unavailable: trusted-background category provenance was collapsed",
            "road": "not a false class: retained road categories are positive; trusted provenance was collapsed",
            "vegetation": "not a false class: retained vegetation annotations are positive",
            "building_wall": "unavailable: no matching validation annotation and trusted provenance was collapsed",
            "license_plate": "unavailable: no matching validation annotation",
        },
        "deferred": [
            "SigLIP proxy",
            "actual crop generation and crop-generation latency",
        ],
        **results,
    }
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"Wrote proposal utility report to {output}")


if __name__ == "__main__":
    main()
