"""Standalone batch size benchmark for quadrilateral proposal detector training.

Benchmarks physical batch sizes (e.g. 16, 32, 64) on available hardware to determine
peak VRAM usage, throughput (examples/sec), and optimal physical batch size / accumulation steps
for a target effective batch size (e.g. 64).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import torch
from torch.utils.data import DataLoader, Dataset

from student_detector.config import load_phase3_config
from student_detector.model import QuadProposalDetector
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_data import QuadProposalSample
from student_detector.quad_losses import QuadProposalLoss
from student_detector.quad_targets import QuadTargetBuilder


class DummyQuadDataset(Dataset[QuadProposalSample]):
    """Synthetic dataset generating fixed-size tensors for benchmarking forward/backward throughput."""

    def __init__(self, count: int = 256, input_size: int = 384) -> None:
        self.count = count
        self.input_size = input_size

    def __len__(self) -> int:
        return self.count

    def __getitem__(self, idx: int) -> QuadProposalSample:
        image = torch.randn(3, self.input_size, self.input_size, dtype=torch.float32)
        quads = torch.tensor([
            [[50.0, 50.0], [150.0, 50.0], [150.0, 150.0], [50.0, 150.0]],
            [[200.0, 200.0], [300.0, 200.0], [300.0, 300.0], [200.0, 300.0]],
        ], dtype=torch.float32)
        return QuadProposalSample(
            image=image,
            quads=quads,
            ignore_quads=torch.zeros((0, 4, 2), dtype=torch.float32),
            valid_mask=torch.ones((self.input_size, self.input_size), dtype=torch.bool),
            image_id=idx,
            source_dataset="coco_2017",
            domain="general",
            camera_type="pinhole",
            background_supervision=True,
            original_size=(self.input_size, self.input_size),
            transform=(1.0, 0.0, 0.0),
        )


from student_detector.quad_data import QuadProposalSample, collate_quad_proposal_samples


def benchmark_batch_size(
    batch_size: int,
    config_path: Path,
    device: torch.device,
    warmup_steps: int = 10,
    timed_steps: int = 30,
) -> dict[str, Any]:
    """Run warm-up and timed steps for a single physical batch size."""
    config = load_phase3_config(config_path)
    input_size = config.data.input_size

    dataset = DummyQuadDataset(count=(warmup_steps + timed_steps) * batch_size, input_size=input_size)
    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=False,
        collate_fn=collate_quad_proposal_samples,
        drop_last=True,
    )

    model = QuadProposalDetector(
        pretrained_backbone=False,
        neck_type=config.neck_type,
    ).to(device)

    assigner = QuadAssigner(
        scale_sigma=config.quad.scale_sigma,
        eligible_levels=config.quad.eligible_levels,
    )
    target_builder = QuadTargetBuilder(assigner)
    criterion = QuadProposalLoss(
        quality_focal_beta=config.quad.quality_focal_beta,
        corner_smooth_l1_beta=config.quad.corner_smooth_l1_beta,
        corner_weight=config.quad.corner_weight,
        validity_weight=config.quad.validity_weight,
        quality_weight=config.quad.quality_weight,
        geometry_quality_target=config.quad.geometry_quality_target,
    )

    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-3)
    scaler = torch.amp.GradScaler("cuda" if device.type == "cuda" else "cpu")

    if device.type == "cuda":
        torch.cuda.reset_peak_memory_stats(device)
        torch.cuda.empty_cache()

    model.train()
    loader_iter = iter(loader)

    # Warm-up phase
    for _ in range(warmup_steps):
        images, samples = next(loader_iter)
        images = images.to(device, non_blocking=device.type == "cuda")
        optimizer.zero_grad(set_to_none=True)
        with torch.amp.autocast("cuda" if device.type == "cuda" else "cpu"):
            output = model(images)
            shapes = tuple((t.shape[-2], t.shape[-1]) for t in output.quality)
            targets = target_builder(samples, shapes, device=device)
            loss_output = criterion(output, targets)
            total_loss = loss_output.total
        scaler.scale(total_loss).backward()
        scaler.step(optimizer)
        scaler.update()

    if device.type == "cuda":
        torch.cuda.synchronize(device)

    start_time = time.perf_counter()
    steps_completed = 0

    # Timed benchmark phase
    for _ in range(timed_steps):
        try:
            images, samples = next(loader_iter)
        except StopIteration:
            break

        images = images.to(device, non_blocking=device.type == "cuda")
        optimizer.zero_grad(set_to_none=True)
        with torch.amp.autocast("cuda" if device.type == "cuda" else "cpu"):
            output = model(images)
            shapes = tuple((t.shape[-2], t.shape[-1]) for t in output.quality)
            targets = target_builder(samples, shapes, device=device)
            loss_output = criterion(output, targets)
            total_loss = loss_output.total
        scaler.scale(total_loss).backward()
        scaler.step(optimizer)
        scaler.update()
        steps_completed += 1

    if device.type == "cuda":
        torch.cuda.synchronize(device)

    elapsed_time = time.perf_counter() - start_time
    total_images = steps_completed * batch_size
    fps = total_images / elapsed_time if elapsed_time > 0 else 0.0

    peak_vram_mb = 0.0
    allocated_vram_mb = 0.0
    total_vram_mb = 0.0
    if device.type == "cuda":
        peak_vram_mb = torch.cuda.max_memory_allocated(device) / (1024 * 1024)
        allocated_vram_mb = torch.cuda.memory_allocated(device) / (1024 * 1024)
        total_vram_mb = torch.cuda.get_device_properties(device).total_memory / (1024 * 1024)

    return {
        "physical_batch_size": batch_size,
        "steps_completed": steps_completed,
        "elapsed_seconds": round(elapsed_time, 4),
        "examples_per_second": round(fps, 2),
        "peak_vram_mb": round(peak_vram_mb, 2),
        "allocated_vram_mb": round(allocated_vram_mb, 2),
        "total_vram_mb": round(total_vram_mb, 2),
        "vram_headroom_fraction": round(1.0 - (peak_vram_mb / total_vram_mb), 4) if total_vram_mb > 0 else 1.0,
        "status": "success",
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark physical batch size for quad detector training.")
    parser.add_argument("--config", type=Path, default=Path("configs/phase3_attnres.yaml"))
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--batch-sizes", default="16,32,64", help="Comma-separated physical batch sizes to test")
    parser.add_argument("--effective-batch-size", type=int, default=64)
    parser.add_argument("--warmup-steps", type=int, default=10)
    parser.add_argument("--timed-steps", type=int, default=30)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    device = torch.device(args.device)
    candidate_batches = [int(b.strip()) for b in args.batch_sizes.split(",") if b.strip()]

    results: list[dict[str, Any]] = []
    best_candidate: dict[str, Any] | None = None
    best_fps = -1.0

    gpu_name = torch.cuda.get_device_name(device) if device.type == "cuda" else "CPU"

    for batch_size in candidate_batches:
        try:
            res = benchmark_batch_size(
                batch_size=batch_size,
                config_path=args.config,
                device=device,
                warmup_steps=args.warmup_steps,
                timed_steps=args.timed_steps,
            )
            acc_steps = math.ceil(args.effective_batch_size / batch_size)
            res["accumulation_steps"] = acc_steps
            res["effective_batch_size"] = batch_size * acc_steps
            results.append(res)

            # Pick fastest batch size that maintains at least 10% VRAM headroom
            if res["vram_headroom_fraction"] >= 0.10 and res["examples_per_second"] > best_fps:
                best_fps = res["examples_per_second"]
                best_candidate = res

        except torch.cuda.OutOfMemoryError:
            if device.type == "cuda":
                torch.cuda.empty_cache()
            results.append({
                "physical_batch_size": batch_size,
                "status": "OOM",
                "accumulation_steps": math.ceil(args.effective_batch_size / batch_size),
                "effective_batch_size": args.effective_batch_size,
            })
        except Exception as err:
            results.append({
                "physical_batch_size": batch_size,
                "status": f"Error: {err}",
            })

    output_payload = {
        "device": str(device),
        "gpu_name": gpu_name,
        "config": str(args.config),
        "target_effective_batch_size": args.effective_batch_size,
        "results": results,
        "recommended_candidate": best_candidate,
    }

    print(json.dumps(output_payload, indent=2))
    if best_candidate is not None:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
