"""Categorize and visualize nuImages ground truths requiring ATSS fallback."""

from __future__ import annotations

import argparse
import json
import sqlite3
from collections import Counter
from pathlib import Path

import torch
from PIL import Image, ImageDraw

from student_detector.assigner import ATSSAssigner
from student_detector.config import load_phase3_config
from student_detector.data import (
    IMAGENET_MEAN,
    IMAGENET_STD,
    IndexedCocoProposalDataset,
)
from student_detector.targets import point_validity_from_pixel_mask


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=Path, default=Path("configs/phase3.yaml")
    )
    parser.add_argument("--source", default="nuimages")
    parser.add_argument("--samples", type=int, default=1000)
    parser.add_argument("--max-visualizations", type=int, default=64)
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args()


def _to_pil(tensor: torch.Tensor) -> Image.Image:
    mean = tensor.new_tensor(IMAGENET_MEAN).view(3, 1, 1)
    std = tensor.new_tensor(IMAGENET_STD).view(3, 1, 1)
    image = ((tensor * std + mean).clamp(0, 1) * 255).byte()
    return Image.fromarray(image.permute(1, 2, 0).contiguous().numpy())


def _make_assigner(config, *, prior_sizes, center_radius) -> ATSSAssigner:
    return ATSSAssigner(
        strides=config.assignment.strides,
        prior_sizes=prior_sizes,
        top_k=config.assignment.top_k,
        center_radius=center_radius,
    )


def _category_names(
    dataset: IndexedCocoProposalDataset,
    sample,
) -> list[str]:
    """Recover categories in the exact post-transform positive-box order."""
    with sqlite3.connect(dataset.index_path) as connection:
        rows = connection.execute(
            """
            SELECT x1, y1, x2, y2, ignore_region, category_name
            FROM annotations WHERE image_id=?
            """,
            (sample.image_id,),
        ).fetchall()
    positive_rows = [row for row in rows if not row[4]]
    component_indices = dataset._contained_component_indices(positive_rows)
    kept = [
        row for index, row in enumerate(positive_rows)
        if index not in component_indices
    ]
    if not sample.boxes.numel():
        return []
    scale, offset_x, offset_y = sample.transform
    transformed = torch.tensor(
        [row[:4] for row in kept], dtype=torch.float32
    )
    transformed *= scale
    transformed[:, 0::2] += offset_x
    transformed[:, 1::2] += offset_y
    transformed.clamp_(0, dataset.data_config.input_size)
    names: list[str] = []
    available = torch.ones(len(kept), dtype=torch.bool)
    for box in sample.boxes:
        distance = (transformed - box).abs().sum(dim=1)
        distance.masked_fill_(~available, float("inf"))
        match = int(distance.argmin())
        if not torch.isfinite(distance[match]) or distance[match] > 1e-2:
            names.append("unmatched")
        else:
            names.append(str(kept[match][5]))
            available[match] = False
    return names


def _variant_label(
    gt_index: int,
    baseline,
    pure_atss,
    compact_priors,
) -> str:
    pure_fallback = set(map(int, pure_atss.fallback_gt_indices.tolist()))
    compact_fallback = set(
        map(int, compact_priors.fallback_gt_indices.tolist())
    )
    if gt_index not in compact_fallback:
        return "resolved_by_4x_stride_priors"
    if gt_index not in pure_fallback:
        return "resolved_by_removing_center_gate"
    return "persistent_fallback"


def main() -> None:
    args = parse_args()
    config = load_phase3_config(args.config)
    output_dir = args.output_dir or (
        config.output_dir.parent.parent
        / "analysis"
        / f"{args.source}_fallbacks"
    )
    examples_dir = output_dir / "examples"
    examples_dir.mkdir(parents=True, exist_ok=True)
    dataset = IndexedCocoProposalDataset(
        config.data.train_annotations,
        config.data.image_root,
        config.data.index_dir / "train.sqlite",
        config.data,
        config.augmentation,
        training=False,
    )
    selected = [
        index for index, record in enumerate(dataset.records)
        if record.source_dataset == args.source and record.positive_count > 0
    ][: args.samples]
    shapes = tuple(
        (config.data.input_size // stride, config.data.input_size // stride)
        for stride in config.assignment.strides
    )
    baseline_assigner = _make_assigner(
        config,
        prior_sizes=config.assignment.prior_sizes,
        center_radius=config.assignment.center_radius,
    )
    pure_assigner = _make_assigner(
        config,
        prior_sizes=config.assignment.prior_sizes,
        center_radius=None,
    )
    compact_assigner = _make_assigner(
        config,
        prior_sizes=tuple(4 * stride for stride in config.assignment.strides),
        center_radius=config.assignment.center_radius,
    )

    category_counts: Counter[str] = Counter()
    reason_counts: Counter[str] = Counter()
    size_counts: Counter[str] = Counter()
    fallback_count = 0
    unrepresentable_count = 0
    gt_count = 0
    examples: list[dict[str, object]] = []
    visualized = 0

    for dataset_index in selected:
        sample = dataset[dataset_index]
        valid_flat, _ = point_validity_from_pixel_mask(
            sample.valid_mask, shapes, config.assignment.strides
        )
        assignments = [
            assigner.assign(
                sample.boxes,
                shapes,
                (config.data.input_size, config.data.input_size),
                valid_flat,
            )
            for assigner in (
                baseline_assigner, pure_assigner, compact_assigner
            )
        ]
        baseline, pure_atss, compact_priors = assignments
        gt_count += int(baseline.valid_gt_mask.sum())
        fallback_indices = list(map(
            int, baseline.fallback_gt_indices.tolist()
        ))
        unrepresentable_count += baseline.unrepresentable_gt_indices.numel()
        fallback_count += len(fallback_indices)
        if not fallback_indices:
            continue
        categories = _category_names(dataset, sample)
        image_examples: list[dict[str, object]] = []
        for gt_index in fallback_indices:
            box = sample.boxes[gt_index]
            width = float(box[2] - box[0])
            height = float(box[3] - box[1])
            area = width * height
            size = (
                "tiny" if area < 32**2
                else "small" if area < 64**2
                else "medium" if area < 128**2
                else "large"
            )
            category = (
                categories[gt_index]
                if gt_index < len(categories)
                else "unmatched"
            )
            reason = _variant_label(
                gt_index, baseline, pure_atss, compact_priors
            )
            category_counts[category] += 1
            reason_counts[reason] += 1
            size_counts[size] += 1
            detail = {
                "dataset_index": dataset_index,
                "image_id": sample.image_id,
                "category": category,
                "reason": reason,
                "size": size,
                "box_xyxy": [round(float(value), 3) for value in box],
                "width": round(width, 3),
                "height": round(height, 3),
                "area": round(area, 3),
                "aspect_ratio": round(
                    max(width, height) / max(min(width, height), 1e-7), 3
                ),
            }
            examples.append(detail)
            image_examples.append(detail)

        if visualized >= args.max_visualizations:
            continue
        canvas = _to_pil(sample.image)
        draw = ImageDraw.Draw(canvas)
        fallback_set = set(fallback_indices)
        for gt_index, box in enumerate(sample.boxes):
            color = "red" if gt_index in fallback_set else "lime"
            draw.rectangle(tuple(map(float, box)), outline=color, width=2)
        for detail in image_examples:
            box = detail["box_xyxy"]
            draw.text(
                (float(box[0]), max(0.0, float(box[1]) - 11)),
                f"{detail['category']} | {detail['reason']}",
                fill="red",
            )
        file_name = (
            f"{visualized:03d}_{dataset_index:06d}_{sample.image_id}.jpg"
        )
        canvas.save(examples_dir / file_name)
        visualized += 1

    result = {
        "source": args.source,
        "sampled_images": len(selected),
        "valid_gt": gt_count,
        "fallback_gt": fallback_count,
        "fallback_rate_per_gt": fallback_count / max(gt_count, 1),
        "unrepresentable_gt": unrepresentable_count,
        "unrepresentable_rate_per_gt": (
            unrepresentable_count / max(gt_count, 1)
        ),
        "fallback_by_category": dict(category_counts.most_common()),
        "fallback_by_controlled_variant": dict(reason_counts.most_common()),
        "fallback_by_size": dict(size_counts),
        "visualized_images": visualized,
        "examples": examples,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(
        {key: value for key, value in result.items() if key != "examples"},
        indent=2,
        sort_keys=True,
    ))


if __name__ == "__main__":
    main()
