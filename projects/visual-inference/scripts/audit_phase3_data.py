"""Audit transformed real annotations and ATSS assignment before training."""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import torch

from student_detector.assigner import ATSSAssigner
from student_detector.config import load_phase3_config
from student_detector.data import (
    IndexedCocoProposalDataset,
    select_source_mixture_indices,
)
from student_detector.targets import TargetBuilder


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=Path, default=Path("configs/phase3.yaml")
    )
    parser.add_argument("--samples", type=int, default=300)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = load_phase3_config(args.config)
    dataset = IndexedCocoProposalDataset(
        config.data.train_annotations,
        config.data.image_root,
        config.data.index_dir / "train.sqlite",
        config.data,
        config.augmentation,
        training=False,
    )
    selected = select_source_mixture_indices(
        dataset.records, config.data.source_weights, args.samples
    )
    domain_counts: dict[str, int] = defaultdict(int)
    for index in selected:
        source = dataset.records[index].source_dataset
        domain_counts[config.data.source_domains.get(source, "unknown")] += 1
    assigner = ATSSAssigner(
        strides=config.assignment.strides,
        prior_sizes=config.assignment.prior_sizes,
        top_k=config.assignment.top_k,
        center_radius=config.assignment.center_radius,
    )
    builder = TargetBuilder(assigner)
    shapes = tuple(
        (config.data.input_size // stride, config.data.input_size // stride)
        for stride in config.assignment.strides
    )
    result: dict[str, object] = {
        "sample_count": len(selected),
        "requested_sample_count": args.samples,
        "domain_sample_count": dict(domain_counts),
    }
    totals: dict[str, float] = defaultdict(float)
    observed_sources: dict[str, int] = defaultdict(int)
    source_totals: dict[str, dict[str, float]] = defaultdict(
        lambda: defaultdict(float)
    )
    for index in selected:
        sample = dataset[index]
        targets = builder([sample], shapes, device=torch.device("cpu"))
        observed_sources[sample.source_dataset] += 1
        totals["positive_instances"] += sample.boxes.shape[0]
        totals["ignore_instances"] += sample.ignore_boxes.shape[0]
        totals["empty_images"] += int(sample.boxes.shape[0] == 0)
        totals["atss_positive_locations"] += int(
            targets.positive_mask.sum()
        )
        totals["fallback_gt"] += float(targets.fallback_count)
        totals["unrepresentable_gt"] += float(
            targets.unrepresentable_count
        )
        current_source = source_totals[sample.source_dataset]
        current_source["positive_instances"] += sample.boxes.shape[0]
        current_source["fallback_gt"] += float(targets.fallback_count)
        current_source["unrepresentable_gt"] += float(
            targets.unrepresentable_count
        )
        totals["valid_point_fraction"] += float(torch.cat([
            level.flatten() for level in targets.valid_point_masks
        ]).float().mean())
        for level, count in enumerate(
            targets.positive_counts_per_level, start=3
        ):
            totals[f"positive_locations_P{level}"] += int(count)
    totals["valid_point_fraction"] /= max(len(selected), 1)
    positive_instances = max(totals["positive_instances"], 1)
    totals["fallback_rate_per_gt"] = (
        totals["fallback_gt"] / positive_instances
    )
    totals["unrepresentable_rate_per_gt"] = (
        totals["unrepresentable_gt"] / positive_instances
    )
    for values in source_totals.values():
        denominator = max(values["positive_instances"], 1)
        values["fallback_rate_per_gt"] = values["fallback_gt"] / denominator
        values["unrepresentable_rate_per_gt"] = (
            values["unrepresentable_gt"] / denominator
        )
    result["source_sample_count"] = dict(observed_sources)
    result["source_totals"] = {
        source: dict(values) for source, values in source_totals.items()
    }
    result["warnings"] = []
    if totals["fallback_rate_per_gt"] > 0.02:
        result["warnings"].append(
            "ATSS fallback rate exceeds the 2% investigation threshold"
        )
    result["totals"] = dict(totals)
    output = args.output or config.output_dir / "data_audit.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
