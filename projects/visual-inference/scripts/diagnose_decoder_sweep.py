"""Sweep fixed proposal-decoder limits on one cached Stage-0 forward pass."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
from torch.utils.data import DataLoader, Subset

from student_detector.config import load_phase3_config
from student_detector.data import (
    IndexedCocoProposalDataset,
    ProposalSample,
    collate_proposal_samples,
    select_source_mixture_indices,
)
from student_detector.decoder import Detection, InferenceDecoder
from student_detector.evaluation import EvaluationImage, evaluate_proposals
from student_detector.head import DetectorOutput
from student_detector.model import StudentDetector
from student_detector.targets import point_validity_from_pixel_mask


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=Path, default=Path("configs/phase3.yaml")
    )
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--images", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=10)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def _cpu_output(output: DetectorOutput) -> DetectorOutput:
    return DetectorOutput(
        tuple(level.cpu() for level in output.objectness),
        tuple(level.cpu() for level in output.box_distances),
        tuple(level.cpu() for level in output.centerness),
    )


def _valid_masks(
    samples: list[ProposalSample],
    feature_shapes: tuple[tuple[int, int], ...],
    strides: tuple[int, int, int],
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    per_sample = [
        point_validity_from_pixel_mask(
            sample.valid_mask, feature_shapes, strides
        )[1]
        for sample in samples
    ]
    return tuple(
        torch.stack([levels[level] for levels in per_sample])
        for level in range(3)
    )  # type: ignore[return-value]


def main() -> None:
    args = parse_args()
    config = load_phase3_config(args.config)
    device = torch.device(args.device)
    dataset = IndexedCocoProposalDataset(
        config.data.train_annotations,
        config.data.image_root,
        config.data.index_dir / "train.sqlite",
        config.data,
        config.augmentation,
        training=False,
    )
    indices = select_source_mixture_indices(
        dataset.records,
        config.data.source_weights,
        args.images,
        positive_only=True,
    )
    loader = DataLoader(
        Subset(dataset, indices),
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=0,
        collate_fn=collate_proposal_samples,
        pin_memory=device.type == "cuda",
    )
    checkpoint = torch.load(
        args.checkpoint, map_location=device, weights_only=False
    )
    model = StudentDetector(pretrained_backbone=False).to(device).eval()
    model.load_state_dict(checkpoint["model"])

    cached: list[tuple[DetectorOutput, list[ProposalSample], tuple[
        torch.Tensor, torch.Tensor, torch.Tensor
    ]]] = []
    with torch.inference_mode():
        for inputs, samples in loader:
            output = _cpu_output(model(inputs.to(device)))
            shapes = tuple(
                (level.shape[-2], level.shape[-1])
                for level in output.objectness
            )
            cached.append((
                output,
                samples,
                _valid_masks(samples, shapes, config.assignment.strides),
            ))
    del model

    result: dict[str, object] = {
        "checkpoint_epoch": checkpoint["epoch"],
        "checkpoint_global_step": checkpoint["global_step"],
        "maximum_proposals": 100,
        "sweep": {},
    }
    sweep = result["sweep"]
    assert isinstance(sweep, dict)
    for top_k in (300, 600, 1000, 3024):
        for threshold in (0.5, 0.6, 0.7, 0.8, 0.9):
            decoder = InferenceDecoder(
                strides=config.assignment.strides,
                top_k=top_k,
                nms_iou_threshold=threshold,
                max_detections=100,
                score_mode=config.inference.score_mode,
            )
            evaluated: list[EvaluationImage] = []
            for output, samples, valid_masks in cached:
                detections = decoder(
                    output,
                    (config.data.input_size, config.data.input_size),
                    valid_masks,
                )
                for sample, detection in zip(
                    samples, detections, strict=True
                ):
                    evaluated.append(EvaluationImage(
                        sample.image_id,
                        sample.domain,
                        sample.camera_type,
                        (config.data.input_size, config.data.input_size),
                        sample.boxes,
                        sample.ignore_boxes,
                        Detection(detection.boxes, detection.scores),
                    ))
            metrics = evaluate_proposals(evaluated)
            sweep[f"top_k={top_k},nms={threshold:.1f}"] = {
                key: metrics[key]
                for key in (
                    "recall/100@50",
                    "recall/100@75",
                    "recall/size_edge_small/100@50",
                    "recall/fisheye/100@50",
                    "selection_score",
                )
            }

    output = (
        args.output
        or args.checkpoint.parent / "stage0_decoder_sweep.json"
    )
    output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
