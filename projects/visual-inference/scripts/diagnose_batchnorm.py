"""Measure whether mixed pyramid BatchNorm statistics limit Stage-0 recall."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
from torch import nn
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


def _use_batch_statistics(module: nn.Module) -> None:
    for child in module.modules():
        if isinstance(child, nn.modules.batchnorm._BatchNorm):
            child.train()


def _load_model(
    checkpoint: dict[str, object],
    device: torch.device,
    variant: str,
) -> StudentDetector:
    model = StudentDetector(pretrained_backbone=False).to(device)
    model.load_state_dict(checkpoint["model"])  # type: ignore[arg-type]
    model.eval()
    if variant == "head_batch_stats":
        _use_batch_statistics(model.head)
    elif variant == "fpn_head_batch_stats":
        _use_batch_statistics(model.fpn)
        _use_batch_statistics(model.head)
    elif variant != "running_stats":
        raise ValueError(f"Unknown variant: {variant}")
    return model


def _valid_masks(
    samples: list[ProposalSample],
    feature_shapes: tuple[tuple[int, int], ...],
    strides: tuple[int, int, int],
    device: torch.device,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    per_sample = [
        point_validity_from_pixel_mask(
            sample.valid_mask.to(device), feature_shapes, strides
        )[1]
        for sample in samples
    ]
    return tuple(
        torch.stack([levels[level] for levels in per_sample])
        for level in range(3)
    )  # type: ignore[return-value]


def _evaluate_variant(
    model: StudentDetector,
    loader: DataLoader,
    decoder: InferenceDecoder,
    strides: tuple[int, int, int],
    device: torch.device,
) -> dict[str, float]:
    images: list[EvaluationImage] = []
    with torch.inference_mode():
        for inputs, samples in loader:
            output = model(inputs.to(device, non_blocking=True))
            feature_shapes = tuple(
                (level.shape[-2], level.shape[-1])
                for level in output.objectness
            )
            detections = decoder(
                output,
                (inputs.shape[-2], inputs.shape[-1]),
                _valid_masks(samples, feature_shapes, strides, device),
            )
            for sample, detection in zip(
                samples, detections, strict=True
            ):
                images.append(EvaluationImage(
                    sample.image_id,
                    sample.domain,
                    sample.camera_type,
                    (sample.image.shape[-2], sample.image.shape[-1]),
                    sample.boxes.cpu(),
                    sample.ignore_boxes.cpu(),
                    Detection(
                        detection.boxes.cpu(), detection.scores.cpu()
                    ),
                ))
    return evaluate_proposals(images)


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
    decoder = InferenceDecoder(
        strides=config.assignment.strides,
        top_k=config.inference.pre_nms_top_k,
        nms_iou_threshold=config.inference.nms_iou_threshold,
        max_detections=config.inference.max_proposals,
        score_mode=config.inference.score_mode,
    )
    result: dict[str, object] = {
        "checkpoint_epoch": checkpoint["epoch"],
        "checkpoint_global_step": checkpoint["global_step"],
        "batch_size": args.batch_size,
        "variants": {},
    }
    variants = result["variants"]
    assert isinstance(variants, dict)
    for variant in (
        "running_stats",
        "head_batch_stats",
        "fpn_head_batch_stats",
    ):
        model = _load_model(checkpoint, device, variant)
        variants[variant] = _evaluate_variant(
            model, loader, decoder, config.assignment.strides, device
        )
        del model

    output = (
        args.output
        or args.checkpoint.parent / "stage0_batchnorm_diagnostic.json"
    )
    output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
