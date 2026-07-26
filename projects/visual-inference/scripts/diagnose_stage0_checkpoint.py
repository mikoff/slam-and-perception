"""Separate Stage-0 proposal ranking, localization, and assignment ceilings."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

from student_detector.assigner import ATSSAssigner
from student_detector.config import load_phase3_config
from student_detector.data import (
    IndexedCocoProposalDataset,
    select_source_mixture_indices,
)
from student_detector.decoder import (
    Detection,
    InferenceDecoder,
    class_agnostic_nms,
)
from student_detector.evaluation import EvaluationImage, evaluate_proposals
from student_detector.geometry import box_iou, decode_ltrb, make_grid_points
from student_detector.losses import flatten_detector_output
from student_detector.model import StudentDetector
from student_detector.targets import (
    TargetBuilder,
    point_validity_from_pixel_mask,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=Path, default=Path("configs/phase3.yaml")
    )
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--images", type=int, default=50)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def _evaluation_image(sample, detection: Detection) -> EvaluationImage:
    return EvaluationImage(
        sample.image_id,
        sample.domain,
        sample.camera_type,
        (sample.image.shape[-2], sample.image.shape[-1]),
        sample.boxes.cpu(),
        sample.ignore_boxes.cpu(),
        Detection(detection.boxes.cpu(), detection.scores.cpu()),
    )


def _select(
    boxes: torch.Tensor,
    scores: torch.Tensor,
    *,
    top_k: int,
    nms_threshold: float,
    maximum: int,
) -> Detection:
    count = min(top_k, scores.numel())
    scores, indices = torch.topk(scores, count, sorted=True)
    boxes = boxes[indices]
    keep = class_agnostic_nms(boxes, scores, nms_threshold)[:maximum]
    return Detection(boxes[keep], scores[keep])


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
    selected = select_source_mixture_indices(
        dataset.records,
        config.data.source_weights,
        args.images,
        positive_only=True,
    )
    checkpoint = torch.load(
        args.checkpoint, map_location=device, weights_only=False
    )
    model = StudentDetector(pretrained_backbone=False).to(device).eval()
    model.load_state_dict(checkpoint["model"])
    assigner = ATSSAssigner(
        strides=config.assignment.strides,
        prior_sizes=config.assignment.prior_sizes,
        top_k=config.assignment.top_k,
        center_radius=config.assignment.center_radius,
    )
    target_builder = TargetBuilder(assigner)
    decoder = InferenceDecoder(
        strides=config.assignment.strides,
        top_k=config.inference.pre_nms_top_k,
        nms_iou_threshold=config.inference.nms_iou_threshold,
        max_detections=config.inference.max_proposals,
        score_mode=config.inference.score_mode,
    )
    normal: list[EvaluationImage] = []
    oracle_ranking: list[EvaluationImage] = []
    exact_targets: list[EvaluationImage] = []

    for index in selected:
        sample = dataset[index]
        image = sample.image.unsqueeze(0).to(device)
        with torch.inference_mode():
            output = model(image)
        shapes = tuple(
            (level.shape[-2], level.shape[-1])
            for level in output.objectness
        )
        points, _ = make_grid_points(
            shapes,
            config.assignment.strides,
            device=device,
            dtype=torch.float32,
        )
        valid_flat, valid_levels = point_validity_from_pixel_mask(
            sample.valid_mask.to(device),
            shapes,
            config.assignment.strides,
        )
        detected = decoder(
            output,
            (config.data.input_size, config.data.input_size),
            tuple(level.unsqueeze(0) for level in valid_levels),
        )[0]
        normal.append(_evaluation_image(sample, detected))

        _, distances, _ = flatten_detector_output(output)
        predicted_boxes = decode_ltrb(points, distances[0])
        predicted_boxes[:, 0::2].clamp_(
            0, config.data.input_size
        )
        predicted_boxes[:, 1::2].clamp_(
            0, config.data.input_size
        )
        ground_truth = sample.boxes.to(device)
        oracle_scores = (
            box_iou(predicted_boxes, ground_truth).max(dim=1).values
            if ground_truth.numel()
            else predicted_boxes.new_zeros(predicted_boxes.shape[0])
        )
        oracle_scores.masked_fill_(~valid_flat, -torch.inf)
        oracle_ranking.append(_evaluation_image(
            sample,
            _select(
                predicted_boxes,
                oracle_scores,
                top_k=config.inference.pre_nms_top_k,
                nms_threshold=config.inference.nms_iou_threshold,
                maximum=config.inference.max_proposals,
            ),
        ))

        targets = target_builder([sample], shapes, device=device)
        positive = targets.positive_mask[0]
        exact_boxes = decode_ltrb(
            points[positive], targets.box_distances[0, positive]
        )
        exact_scores = targets.centerness[0, positive]
        exact_targets.append(_evaluation_image(
            sample,
            _select(
                exact_boxes,
                exact_scores,
                top_k=config.inference.pre_nms_top_k,
                nms_threshold=config.inference.nms_iou_threshold,
                maximum=config.inference.max_proposals,
            ),
        ))

    result = {
        "checkpoint_epoch": checkpoint["epoch"],
        "checkpoint_global_step": checkpoint["global_step"],
        "network_scores_and_boxes": evaluate_proposals(normal),
        "oracle_scores_predicted_boxes": evaluate_proposals(oracle_ranking),
        "oracle_exact_assignment_targets": evaluate_proposals(exact_targets),
    }
    output = args.output or args.checkpoint.parent / "stage0_diagnostic.json"
    output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
