"""Audit deterministic sampler exposure to hard-negative focus images."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

from student_detector.config import load_phase3_config
from student_detector.data import (
    DomainMixtureBatchSampler,
    HardNegativeFocusBatchSampler,
    IndexedCocoProposalDataset,
)


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--max-steps", type=int, required=True)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> None:
    args = _args()
    if args.max_steps < 1:
        raise ValueError("max steps must be positive")
    config = load_phase3_config(args.config)
    dataset = IndexedCocoProposalDataset(
        config.data.train_annotations,
        config.data.image_root,
        config.data.index_dir / "quad_train.sqlite",
        config.data,
        config.augmentation,
        training=True,
        seed=config.schedule.seed,
    )
    sampler_type = (
        HardNegativeFocusBatchSampler
        if config.data.hard_negative_focus_per_optimizer_window
        else DomainMixtureBatchSampler
    )
    sampler_extra = (
        {
            "accumulation_steps": config.schedule.accumulation_steps,
            "focus_source_weights": config.data.hard_negative_focus_source_weights,
        }
        if sampler_type is HardNegativeFocusBatchSampler
        else {}
    )
    sampler = sampler_type(
        dataset,
        config.data.batch_size,
        domain_weights=config.data.domain_weights,
        source_weights=config.data.source_weights,
        empty_fraction=config.data.empty_fraction,
        seed=config.schedule.seed,
        batches_per_epoch=config.data.batches_per_epoch,
        **sampler_extra,
    )
    focus_indices = {
        index
        for index, record in enumerate(dataset.records)
        if (record.source_dataset, record.image_id) in dataset.hard_negative_focus
    }
    required_batches = args.max_steps * config.schedule.accumulation_steps
    draws = Counter()
    windows_with_focus: set[int] = set()
    exposed_indices: set[int] = set()
    batch_number = 0
    epoch = 0
    while batch_number < required_batches:
        sampler.set_epoch(epoch)
        for batch in sampler:
            for index in batch:
                if index not in focus_indices:
                    continue
                record = dataset.records[index]
                draws[record.source_dataset] += 1
                exposed_indices.add(index)
                windows_with_focus.add(
                    batch_number // config.schedule.accumulation_steps
                )
            batch_number += 1
            if batch_number == required_batches:
                break
        epoch += 1
    result = {
        "schema_version": "hard-negative-exposure-audit.v1",
        "max_steps": args.max_steps,
        "batches": required_batches,
        "sample_draws": required_batches * config.data.batch_size,
        "configured_focus_images": len(focus_indices),
        "exposed_focus_images": len(exposed_indices),
        "focus_draws": sum(draws.values()),
        "focus_draws_by_source": dict(sorted(draws.items())),
        "optimizer_windows_with_focus": len(windows_with_focus),
        "optimizer_windows_without_focus": args.max_steps - len(windows_with_focus),
    }
    encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        output = args.output.resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")


if __name__ == "__main__":
    main()
