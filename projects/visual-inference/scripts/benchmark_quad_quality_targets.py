"""Benchmark exact-IoU and cyclic-corner quality targets on one fixed GPU batch."""

from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path
from typing import Any

import torch
from torch.utils.data import DataLoader

from student_detector.config import load_phase3_config
from student_detector.model import QuadProposalDetector
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_data import QuadProposalDataset, collate_quad_proposal_samples
from student_detector.quad_losses import QuadProposalLoss
from student_detector.quad_targets import QuadTargetBuilder


def _shapes(output: Any) -> tuple[tuple[int, int], ...]:
    return tuple((tensor.shape[-2], tensor.shape[-1]) for tensor in output.quality)


def _run_branch(
    branch: str,
    base_state: dict[str, torch.Tensor],
    images: torch.Tensor,
    samples: list[object],
    builder: QuadTargetBuilder,
    config: Any,
    device: torch.device,
    warmup: int,
    iterations: int,
) -> dict[str, float]:
    model = QuadProposalDetector(pretrained_backbone=False).to(device)
    model.load_state_dict(base_state)
    model.train()
    criterion = QuadProposalLoss(
        strides=config.assignment.strides,
        quality_weight=config.quad.quality_weight,
        corner_weight=config.quad.corner_weight,
        validity_weight=config.quad.validity_weight,
        quality_focal_beta=config.quad.quality_focal_beta,
        quality_target_mode="iou",
        quality_blend=1.0,
        geometry_quality_target=branch,
    )
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-5)
    use_amp = config.schedule.amp and device.type == "cuda"
    scaler = torch.amp.GradScaler("cuda", enabled=use_amp)
    durations: list[float] = []
    if device.type == "cuda":
        torch.cuda.reset_peak_memory_stats(device)
    for iteration in range(warmup + iterations):
        optimizer.zero_grad(set_to_none=True)
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        started = time.perf_counter()
        with torch.autocast(device_type=device.type, dtype=torch.float16, enabled=use_amp):
            output = model(images)
        targets = builder(samples, _shapes(output), device=device)
        with torch.autocast(device_type=device.type, dtype=torch.float16, enabled=use_amp):
            loss = criterion(output, targets).total
        scaler.scale(loss).backward()
        scaler.step(optimizer)
        scaler.update()
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        elapsed = time.perf_counter() - started
        if iteration >= warmup:
            durations.append(elapsed)
    peak = (
        torch.cuda.max_memory_allocated(device) if device.type == "cuda" else 0
    )
    return {
        "median_step_ms": statistics.median(durations) * 1000.0,
        "peak_memory_mib": peak / 1024**2,
        "iterations": float(iterations),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--iterations", type=int, default=20)
    args = parser.parse_args()
    device = torch.device(args.device)
    if device.type != "cuda" or not torch.cuda.is_available():
        raise RuntimeError("G5 must be benchmarked on an available CUDA device")
    config = load_phase3_config(args.config)
    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    dataset = QuadProposalDataset(
        config.data.quad_train_annotations or config.data.train_annotations,
        config.data.image_root,
        config.data.index_dir / "g5_benchmark.sqlite",
        config.data,
        config.augmentation,
        training=False,
        force_index=True,
    )
    loader = DataLoader(
        dataset,
        batch_size=config.data.batch_size,
        shuffle=False,
        num_workers=0,
        collate_fn=collate_quad_proposal_samples,
    )
    images, samples = next(iter(loader))
    images = images.to(device)
    builder = QuadTargetBuilder(
        QuadAssigner(
            strides=config.assignment.strides,
            top_k=config.quad.top_k,
            gamma=config.quad.gamma,
            scale_sigma=config.quad.scale_sigma,
            eligible_levels=config.quad.eligible_levels,
        ),
        weak_negative_weight=config.quad.weak_negative_weight,
    )
    state = checkpoint["model"]
    proxy = _run_branch(
        "corner_proxy", state, images, samples, builder, config, device,
        args.warmup, args.iterations,
    )
    exact = _run_branch(
        "exact_iou", state, images, samples, builder, config, device,
        args.warmup, args.iterations,
    )
    time_overhead = exact["median_step_ms"] / proxy["median_step_ms"] - 1.0
    memory_overhead = (
        exact["peak_memory_mib"] / proxy["peak_memory_mib"] - 1.0
        if proxy["peak_memory_mib"]
        else 0.0
    )
    report = {
        "gate": "G5",
        "selected": "exact_iou" if time_overhead <= 0.10 and memory_overhead <= 0.10 else "corner_proxy",
        "exact_iou": exact,
        "corner_proxy": proxy,
        "exact_time_overhead_fraction": time_overhead,
        "exact_memory_overhead_fraction": memory_overhead,
        "threshold_fraction": 0.10,
        "device": torch.cuda.get_device_name(device),
        "checkpoint": str(args.checkpoint.resolve()),
    }
    output = args.output or args.checkpoint.resolve().parent / "g5_quality_target_benchmark.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
