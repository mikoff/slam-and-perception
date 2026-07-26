"""Overlay Phase-3 ground truths, ignore regions, and HBB proposals."""

from __future__ import annotations

import argparse
from pathlib import Path

import torch
from PIL import Image, ImageDraw

from student_detector.config import load_phase3_config
from student_detector.data import (
    IMAGENET_MEAN,
    IMAGENET_STD,
    IndexedCocoProposalDataset,
)
from student_detector.decoder import InferenceDecoder
from student_detector.model import StudentDetector
from student_detector.targets import point_validity_from_pixel_mask


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=Path, default=Path("configs/phase3.yaml")
    )
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--count", type=int, default=12)
    parser.add_argument("--score-threshold", type=float, default=0.05)
    return parser.parse_args()


def _to_pil(tensor: torch.Tensor) -> Image.Image:
    mean = tensor.new_tensor(IMAGENET_MEAN).view(3, 1, 1)
    std = tensor.new_tensor(IMAGENET_STD).view(3, 1, 1)
    image = ((tensor * std + mean).clamp(0, 1) * 255).byte()
    array = image.permute(1, 2, 0).contiguous().numpy()
    return Image.fromarray(array)


def main() -> None:
    args = parse_args()
    config = load_phase3_config(args.config)
    output_dir = args.output_dir or config.output_dir / "visualizations"
    output_dir.mkdir(parents=True, exist_ok=True)
    dataset = IndexedCocoProposalDataset(
        config.data.val_annotations,
        config.data.image_root,
        config.data.index_dir / "val.sqlite",
        config.data,
        config.augmentation,
        training=False,
    )
    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    model = StudentDetector(pretrained_backbone=False).eval()
    model.load_state_dict(checkpoint.get("ema_model", checkpoint["model"]))
    decoder = InferenceDecoder(
        strides=config.assignment.strides,
        top_k=config.inference.pre_nms_top_k,
        nms_iou_threshold=config.inference.nms_iou_threshold,
        max_detections=config.inference.max_proposals,
        score_mode=config.inference.score_mode,
    )
    by_domain: dict[str, list[int]] = {}
    for index, record in enumerate(dataset.records):
        domain = config.data.source_domains.get(record.source_dataset, "unknown")
        values = by_domain.setdefault(domain, [])
        per_domain = max(1, (args.count + 2) // 3)
        if len(values) < per_domain:
            values.append(index)
        if (
            sum(len(values) for values in by_domain.values()) >= args.count
            and all(
                domain in by_domain
                for domain in ("general", "automotive", "fisheye")
            )
        ):
            break
    selected: list[int] = []
    for domain in ("general", "automotive", "fisheye"):
        selected.extend(by_domain.get(domain, []))
    if len(selected) < args.count:
        already = set(selected)
        selected.extend(
            index for index in range(len(dataset))
            if index not in already
        )
    for index in selected[: args.count]:
        sample = dataset[index]
        with torch.inference_mode():
            output = model(sample.image.unsqueeze(0))
        shapes = tuple(
            (tensor.shape[-2], tensor.shape[-1])
            for tensor in output.objectness
        )
        valid_levels = point_validity_from_pixel_mask(
            sample.valid_mask, shapes, config.assignment.strides
        )[1]
        detection = decoder(
            output,
            (config.data.input_size, config.data.input_size),
            tuple(level.unsqueeze(0) for level in valid_levels),
        )[0]
        canvas = _to_pil(sample.image)
        draw = ImageDraw.Draw(canvas)
        for box in sample.ignore_boxes:
            draw.rectangle(tuple(map(float, box)), outline="yellow", width=1)
        for box in sample.boxes:
            draw.rectangle(tuple(map(float, box)), outline="lime", width=1)
        for box, score in zip(detection.boxes, detection.scores, strict=True):
            if score < args.score_threshold:
                continue
            draw.rectangle(tuple(map(float, box)), outline="red", width=2)
            draw.text(
                (float(box[0]), float(box[1])),
                f"{float(score):.2f}",
                fill="red",
            )
        canvas.save(
            output_dir
            / f"{index:06d}_{sample.domain}_{sample.source_dataset}.jpg"
        )


if __name__ == "__main__":
    main()
