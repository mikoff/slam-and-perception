"""Compare proposal overlap, spatial, scale, and unmatched-set diversity."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from typing import Any

import torch
from torch.utils.data import DataLoader, Subset

from student_detector.config import load_phase3_config
from student_detector.data import select_source_mixture_indices
from student_detector.decoder import Detection, InferenceDecoder
from student_detector.head import QuadDetectorOutput
from student_detector.proposal_diversity import ProposalDiversityAccumulator
from student_detector.proposal_evaluation import (
    boxes_to_quads,
    load_proposal_model,
    synchronize_device,
    valid_output_levels,
)
from student_detector.quad_data import (
    QuadProposalDataset,
    collate_quad_proposal_samples,
)
from student_detector.quad_decoder import QuadDetection, QuadInferenceDecoder
from student_detector.quad_geometry import pairwise_quad_iou, warmup_compiled_quad_iou
from student_detector.suppression import greedy_nms_from_overlaps


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--hbb-checkpoint", type=Path, required=True)
    parser.add_argument("--old-quad-checkpoint", type=Path, required=True)
    parser.add_argument("--quad-checkpoint", type=Path, required=True)
    parser.add_argument("--quad-label", default="quad_candidate")
    parser.add_argument("--quad-production-nms", type=float, default=0.80)
    parser.add_argument("--quad-production-score", type=float, default=0.20)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--validation-images", type=int, default=800)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--device", default="cuda")
    parser.add_argument(
        "--checkpoint-state", choices=("model", "ema_model"), default="ema_model"
    )
    parser.add_argument("--common-nms", type=float, default=0.80)
    parser.add_argument("--log-interval", type=int, default=100)
    return parser.parse_args()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _candidate_tensors(
    kind: str, detection: Detection | QuadDetection
) -> tuple[torch.Tensor, torch.Tensor]:
    if kind == "hbb":
        if not isinstance(detection, Detection):
            raise TypeError("HBB audit received a non-HBB detection")
        if detection.pre_nms_boxes is None or detection.pre_nms_scores is None:
            raise RuntimeError("HBB decoder did not retain pre-NMS candidates")
        return boxes_to_quads(detection.pre_nms_boxes.float()), (
            detection.pre_nms_scores.float()
        )
    if not isinstance(detection, QuadDetection):
        raise TypeError("quad audit received a non-quad detection")
    if detection.pre_nms_quads is None or detection.pre_nms_scores is None:
        raise RuntimeError("quad decoder did not retain pre-NMS candidates")
    return detection.pre_nms_quads.float(), detection.pre_nms_scores.float()


@torch.inference_mode()
def _evaluate_model(
    *,
    label: str,
    kind: str,
    checkpoint: Path,
    checkpoint_state: str,
    loader: DataLoader[Any],
    config: Any,
    device: torch.device,
    production_nms: float,
    production_score: float,
    common_nms: float,
    log_interval: int,
) -> dict[str, Any]:
    model = load_proposal_model(kind, checkpoint, checkpoint_state, device)
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
    modes = {
        "production": {
            "nms_iou_threshold": production_nms,
            "score_threshold": production_score,
            "metrics": ProposalDiversityAccumulator(),
        },
        "common_nms": {
            "nms_iou_threshold": common_nms,
            "score_threshold": 0.0,
            "metrics": ProposalDiversityAccumulator(),
        },
    }
    seen = 0
    for images, samples in loader:
        images = images.to(device, non_blocking=device.type == "cuda")
        output = model(images)
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
        detections = decoder(output, (images.shape[-2], images.shape[-1]), valid_masks)
        for sample, detection in zip(samples, detections, strict=True):
            candidate_quads, scores = _candidate_tensors(kind, detection)
            pair_overlaps = pairwise_quad_iou(candidate_quads, candidate_quads)
            gt_overlaps = pairwise_quad_iou(sample.quads.to(device), candidate_quads)
            pair_overlaps_cpu = pair_overlaps.cpu()
            scores_cpu = scores.cpu()
            for mode in modes.values():
                score_threshold = float(mode["score_threshold"])
                eligible = torch.nonzero(scores_cpu >= score_threshold).flatten()
                eligible_overlaps = pair_overlaps_cpu[eligible][:, eligible]
                keep_local = greedy_nms_from_overlaps(
                    eligible_overlaps,
                    scores_cpu[eligible],
                    float(mode["nms_iou_threshold"]),
                    max_output=100,
                )
                keep = eligible[keep_local]
                accumulator = mode["metrics"]
                if not isinstance(accumulator, ProposalDiversityAccumulator):
                    raise TypeError("diversity mode has an invalid accumulator")
                accumulator.update(
                    candidate_quads[keep],
                    gt_overlaps[:, keep],
                    pair_overlaps_cpu[keep][:, keep],
                    image_size=config.data.input_size,
                )
            seen += 1
            if log_interval and seen % log_interval == 0:
                print(f"{label}: {seen}/{len(loader.dataset)} images", flush=True)
        synchronize_device(device)
    del model
    if device.type == "cuda":
        torch.cuda.empty_cache()
    return {
        "kind": kind,
        "checkpoint": str(checkpoint.resolve()),
        "checkpoint_sha256": _sha256(checkpoint),
        "checkpoint_state": checkpoint_state,
        "images": seen,
        "modes": {
            name: {
                "nms_iou_threshold": mode["nms_iou_threshold"],
                "score_threshold": mode["score_threshold"],
                "metrics": mode["metrics"].compute(),
            }
            for name, mode in modes.items()
        },
    }


def main() -> None:
    args = _args()
    if args.validation_images < 1 or args.workers < 0 or args.log_interval < 0:
        raise ValueError(
            "validation images must be positive; workers/log interval nonnegative"
        )
    if not 0 <= args.common_nms <= 1:
        raise ValueError("common NMS threshold must be in [0, 1]")
    if not 0 <= args.quad_production_nms <= 1:
        raise ValueError("quad production NMS threshold must be in [0, 1]")
    if not 0 <= args.quad_production_score <= 1:
        raise ValueError("quad production score threshold must be in [0, 1]")
    if args.quad_label in {"old_quad", "hbb_2000"}:
        raise ValueError("quad label conflicts with a fixed comparison label")
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA diversity audit requested but no CUDA is visible")
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
    loader = DataLoader(
        Subset(dataset, selected),
        batch_size=config.data.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=device.type == "cuda",
        persistent_workers=args.workers > 0,
        collate_fn=collate_quad_proposal_samples,
    )
    if device.type == "cuda":
        warmup_compiled_quad_iou(device)
    models = {}
    for label, kind, checkpoint, production_nms, production_score in (
        ("old_quad", "quad", args.old_quad_checkpoint, 0.80, 0.20),
        (
            args.quad_label,
            "quad",
            args.quad_checkpoint,
            args.quad_production_nms,
            args.quad_production_score,
        ),
        ("hbb_2000", "hbb", args.hbb_checkpoint, 0.70, 0.30),
    ):
        models[label] = _evaluate_model(
            label=label,
            kind=kind,
            checkpoint=checkpoint,
            checkpoint_state=args.checkpoint_state,
            loader=loader,
            config=config,
            device=device,
            production_nms=production_nms,
            production_score=production_score,
            common_nms=args.common_nms,
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
        "schema_version": "visual-inference-proposal-diversity.v1",
        "status": "measurement_complete_owner_review_required",
        "device": str(device),
        "config": str(args.config.resolve()),
        "config_sha256": _sha256(args.config),
        "validation_manifest_sha256": _sha256(Path(annotations)),
        "validation_subset_sha256": hashlib.sha256(selected_identity).hexdigest(),
        "source_counts": dict(
            sorted(
                Counter(
                    dataset.records[index].source_dataset for index in selected
                ).items()
            )
        ),
        "proposal_budget": 100,
        "common_nms_iou_threshold": args.common_nms,
        "cluster_iou_thresholds": [0.50, 0.75, 0.90],
        "unmatched_object_iou_threshold": 0.10,
        "models": models,
    }
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"Wrote proposal diversity audit to {output}")


if __name__ == "__main__":
    main()
