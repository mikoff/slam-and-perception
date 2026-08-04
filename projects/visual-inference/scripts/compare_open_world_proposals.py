"""Compare HBB and quad checkpoints with one polygon-recall implementation."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import sys
import time
import types
from pathlib import Path
from typing import Any

# Compatibility shim: Python 3.13 refactored pathlib to pathlib._local.
# Provide alias when loading 3.13 checkpoints under Python 3.12 environments.
if not hasattr(pathlib, "_local"):
    _local_mod = types.ModuleType("pathlib._local")
    _local_mod.PosixPath = pathlib.PosixPath
    _local_mod.WindowsPath = pathlib.WindowsPath
    _local_mod.Path = pathlib.Path
    sys.modules["pathlib._local"] = _local_mod

import torch

from torch.utils.data import DataLoader

from student_detector.config import load_phase3_config
from student_detector.decoder import InferenceDecoder
from student_detector.model import QuadProposalDetector, StudentDetector
from student_detector.quad_data import QuadProposalDataset, collate_quad_proposal_samples
from student_detector.quad_decoder import QuadDetection, QuadInferenceDecoder
from student_detector.quad_evaluation import QuadEvaluationImage, evaluate_quad_proposals
from student_detector.quad_geometry import (
    polygon_nms,
    quad_validity,
    warmup_compiled_quad_iou,
)
from student_detector.quad_targets import point_validity_from_pixel_mask


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _checkpoint_model(
    model: torch.nn.Module, path: Path, state_key: str
) -> dict[str, Any]:
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    model.load_state_dict(checkpoint[state_key], strict=False)
    return checkpoint



def _box_quads(boxes: torch.Tensor) -> torch.Tensor:
    if boxes.numel() == 0:
        return boxes.new_empty((0, 4, 2))
    x1, y1, x2, y2 = boxes.unbind(dim=1)
    return torch.stack((
        torch.stack((x1, y1), dim=1),
        torch.stack((x2, y1), dim=1),
        torch.stack((x2, y2), dim=1),
        torch.stack((x1, y2), dim=1),
    ), dim=1)


def _sync(device: torch.device) -> None:
    if device.type == "cuda":
        torch.cuda.synchronize(device)


@torch.no_grad()
def _evaluate(
    kind: str,
    checkpoint_path: Path,
    loader: DataLoader,
    config: Any,
    device: torch.device,
    state_key: str,
) -> tuple[dict[str, float], dict[str, float], dict[str, Any]]:
    model: torch.nn.Module
    neck_type = getattr(config, "neck_type", "lite")
    if kind == "hbb":
        model = StudentDetector(pretrained_backbone=False, neck_type=neck_type)
        decoder: Any = InferenceDecoder(
            strides=config.assignment.strides,
            top_k=300,
            nms_iou_threshold=1.0,
            max_detections=300,
            score_mode=config.inference.score_mode,
        )
    else:
        model = QuadProposalDetector(pretrained_backbone=False, neck_type=neck_type)
        decoder = QuadInferenceDecoder(
            strides=config.assignment.strides,
            pre_nms_top_k=300,
            nms_iou_threshold=config.inference.nms_iou_threshold,
            max_proposals=300,
        )
    checkpoint = _checkpoint_model(model, checkpoint_path, state_key)
    model.to(device).eval()
    evaluated: list[QuadEvaluationImage] = []
    forward_seconds = decode_seconds = nms_seconds = 0.0
    image_count = 0
    for images, samples in loader:
        images = images.to(device, non_blocking=device.type == "cuda")
        _sync(device)
        started = time.perf_counter()
        output = model(images)
        _sync(device)
        forward_seconds += time.perf_counter() - started
        feature_shapes = tuple(
            (tensor.shape[-2], tensor.shape[-1])
            for tensor in (output.objectness if kind == "hbb" else output.quality)
        )
        validity = tuple(zip(*(
            point_validity_from_pixel_mask(
                sample.valid_mask.to(device), feature_shapes, config.assignment.strides
            )[1]
            for sample in samples
        )))
        valid_masks = tuple(torch.stack(level) for level in validity)
        _sync(device)
        started = time.perf_counter()
        detections = decoder(output, (images.shape[-2], images.shape[-1]), valid_masks)
        _sync(device)
        decode_seconds += time.perf_counter() - started
        for sample, detection in zip(samples, detections, strict=True):
            if kind == "hbb":
                pre_quads = _box_quads(detection.boxes)
                valid = quad_validity(pre_quads)
                pre_quads, pre_scores = pre_quads[valid], detection.scores[valid]
                _sync(device)
                started = time.perf_counter()
                keep = polygon_nms(
                    pre_quads, pre_scores, config.inference.nms_iou_threshold,
                    max_output=300,
                )
                _sync(device)
                nms_seconds += time.perf_counter() - started
                final = QuadDetection(pre_quads[keep], pre_scores[keep])
                pre = QuadDetection(pre_quads, pre_scores)
            else:
                final = QuadDetection(detection.quads, detection.scores)
                pre = QuadDetection(
                    detection.pre_nms_quads,
                    detection.pre_nms_scores,
                )
            evaluated.append(QuadEvaluationImage(
                image_id=sample.image_id,
                domain=sample.domain,
                camera_type=sample.camera_type,
                image_size=(images.shape[-2], images.shape[-1]),
                ground_truth=sample.quads.cpu(),
                ignore_quads=sample.ignore_quads.cpu(),
                detection=QuadDetection(final.quads.cpu(), final.scores.cpu()),
                pre_nms_detection=QuadDetection(pre.quads.cpu(), pre.scores.cpu()),
                geometry_tiers=sample.geometry_tiers,
                object_conditions=sample.object_conditions,
                seen_statuses=sample.seen_statuses,
                size_bins=sample.size_bins,
                aspect_bins=sample.aspect_bins,
                radial_bins=sample.radial_bins,
            ))
            image_count += 1
    timings = {
        "images": float(image_count),
        "forward_ms_per_image": 1000 * forward_seconds / max(image_count, 1),
        "decode_including_nms_ms_per_image": 1000 * decode_seconds / max(image_count, 1),
        "external_polygon_nms_ms_per_image": 1000 * nms_seconds / max(image_count, 1),
    }
    return evaluate_quad_proposals(evaluated), timings, checkpoint


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--hbb-checkpoint", type=Path, required=True)
    parser.add_argument("--quad-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument(
        "--checkpoint-state", choices=("model", "ema_model"), default="model"
    )
    args = parser.parse_args()
    config = load_phase3_config(args.config)
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA comparison requested but no CUDA device is visible")
    warmup_compiled_quad_iou(device)
    manifest = config.data.quad_val_annotations or config.data.val_annotations
    dataset = QuadProposalDataset(
        manifest,
        config.data.image_root,
        config.data.index_dir / "open_world_comparison.sqlite",
        config.data,
        config.augmentation,
        training=False,
        force_index=True,
    )
    loader = DataLoader(
        dataset,
        batch_size=config.data.batch_size,
        shuffle=False,
        num_workers=config.data.workers,
        collate_fn=collate_quad_proposal_samples,
        pin_memory=device.type == "cuda",
    )
    hbb_metrics, hbb_timings, hbb_checkpoint = _evaluate(
        "hbb", args.hbb_checkpoint.resolve(), loader, config, device,
        args.checkpoint_state,
    )
    quad_metrics, quad_timings, quad_checkpoint = _evaluate(
        "quad", args.quad_checkpoint.resolve(), loader, config, device,
        args.checkpoint_state,
    )
    keys = sorted(set(hbb_metrics) | set(quad_metrics))
    report = {
        "schema_version": "quad-proposal-open-world-comparison.v1",
        "status": "measurement_complete_selection_thresholds_not_yet_frozen",
        "matched_contract": {
            "validation_manifest": str(Path(manifest).resolve()),
            "validation_manifest_sha256": _sha256(Path(manifest)),
            "images": len(dataset),
            "proposal_budgets": [50, 100, 300],
            "checkpoint_state": args.checkpoint_state,
            "iou": "exact convex polygon IoU",
            "nms_iou_threshold": config.inference.nms_iou_threshold,
        },
        "hbb": {"metrics": hbb_metrics, "timings": hbb_timings},
        "quad": {"metrics": quad_metrics, "timings": quad_timings},
        "delta_quad_minus_hbb": {
            key: quad_metrics.get(key, 0.0) - hbb_metrics.get(key, 0.0)
            for key in keys
        },
        "artifacts": {
            "config": str(args.config.resolve()),
            "config_sha256": _sha256(args.config.resolve()),
            "hbb_checkpoint": str(args.hbb_checkpoint.resolve()),
            "hbb_checkpoint_sha256": _sha256(args.hbb_checkpoint.resolve()),
            "hbb_global_step": hbb_checkpoint.get("global_step"),
            "quad_checkpoint": str(args.quad_checkpoint.resolve()),
            "quad_checkpoint_sha256": _sha256(args.quad_checkpoint.resolve()),
            "quad_global_step": quad_checkpoint.get("global_step"),
            "torch": torch.__version__,
            "cuda": torch.version.cuda,
            "device": torch.cuda.get_device_name(device) if device.type == "cuda" else str(device),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
