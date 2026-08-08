"""Automated batch size tuning and VRAM benchmarking for quad detector training."""

from __future__ import annotations

import math
import time
from typing import Any

import torch
from torch.utils.data import DataLoader, Dataset

from .config import Phase3Config
from .model import QuadProposalDetector
from .quad_assigner import QuadAssigner
from .quad_data import QuadProposalSample, collate_quad_proposal_samples
from .quad_losses import QuadProposalLoss
from .quad_targets import QuadTargetBuilder


class DummyQuadDataset(Dataset[QuadProposalSample]):
    """Synthetic dataset generating fixed-size tensors for benchmarking forward/backward throughput."""

    def __init__(self, count: int = 256, input_size: int = 384) -> None:
        self.count = count
        self.input_size = input_size
        self.image = torch.randn(3, self.input_size, self.input_size, dtype=torch.float32)
        self.quads = torch.tensor([
            [[50.0, 50.0], [150.0, 50.0], [150.0, 150.0], [50.0, 150.0]],
            [[200.0, 200.0], [300.0, 200.0], [300.0, 300.0], [200.0, 300.0]],
        ], dtype=torch.float32)
        self.ignore_quads = torch.zeros((0, 4, 2), dtype=torch.float32)
        self.valid_mask = torch.ones((self.input_size, self.input_size), dtype=torch.bool)

    def __len__(self) -> int:
        return self.count

    def __getitem__(self, idx: int) -> QuadProposalSample:
        return QuadProposalSample(
            image=self.image,
            quads=self.quads,
            ignore_quads=self.ignore_quads,
            valid_mask=self.valid_mask,
            image_id=idx,
            source_dataset="coco_2017",
            domain="general",
            camera_type="pinhole",
            background_supervision=True,
            original_size=(self.input_size, self.input_size),
            transform=(1.0, 0.0, 0.0),
        )


def benchmark_batch_size(
    batch_size: int,
    config: Phase3Config,
    device: torch.device,
    warmup_steps: int = 5,
    timed_steps: int = 10,
) -> dict[str, Any]:
    """Run fast warm-up and timed steps to measure peak VRAM and throughput for a physical batch size."""
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
    total_vram_mb = 0.0
    if device.type == "cuda":
        peak_vram_mb = torch.cuda.max_memory_allocated(device) / (1024 * 1024)
        total_vram_mb = torch.cuda.get_device_properties(device).total_memory / (1024 * 1024)

    del model, optimizer, scaler, loader
    if device.type == "cuda":
        torch.cuda.empty_cache()

    return {
        "physical_batch_size": batch_size,
        "examples_per_second": round(fps, 2),
        "peak_vram_mb": round(peak_vram_mb, 2),
        "total_vram_mb": round(total_vram_mb, 2),
        "vram_headroom_fraction": round(1.0 - (peak_vram_mb / total_vram_mb), 4) if total_vram_mb > 0 else 1.0,
        "status": "success",
    }


import sys


def autotune_optimal_batch_size(
    config: Phase3Config,
    device: torch.device,
    candidate_batches: list[int] | None = None,
) -> tuple[int, int]:
    """Dynamically test candidate batch sizes on hardware and return (optimal_batch_size, accumulation_steps)."""
    if candidate_batches is None:
        candidate_batches = [16, 24, 32, 48, 64, 96, 128, 192, 256, 384, 512, 768, 1024]

    if device.type != "cuda":
        print(f"--> [Autotune] Non-CUDA device '{device}'. Using config batch size {config.data.batch_size}.", file=sys.stderr)
        return config.data.batch_size, config.schedule.accumulation_steps

    gpu_name = torch.cuda.get_device_name(device)
    total_vram_gb = torch.cuda.get_device_properties(device).total_memory / (1024**3)
    print(f"--> [Autotune] Probing GPU: {gpu_name} ({total_vram_gb:.1f} GB VRAM)...", file=sys.stderr)

    best_batch = config.data.batch_size

    left = 0
    right = len(candidate_batches) - 1

    while left <= right:
        mid = (left + right) // 2
        b = candidate_batches[mid]
        
        try:
            # We don't need accurate FPS anymore, just VRAM. So 2 warmup, 2 timed is enough!
            res = benchmark_batch_size(b, config, device, warmup_steps=2, timed_steps=2)
            headroom = res["vram_headroom_fraction"]
            fps = res["examples_per_second"]
            peak_mb = res["peak_vram_mb"]
            print(f"    Batch {b:3d} -> {fps:6.1f} img/s | Peak VRAM: {peak_mb:7.1f} MB (Headroom: {headroom*100:.1f}%)", file=sys.stderr)

            if headroom >= 0.10:
                # Valid candidate! Record it and try to find an even larger one
                best_batch = b
                left = mid + 1
            else:
                # Not enough safety headroom! Try a smaller batch
                print(f"    Batch {b:3d} -> Rejected (Headroom < 10%)", file=sys.stderr)
                right = mid - 1

        except torch.cuda.OutOfMemoryError:
            print(f"    Batch {b:3d} -> OUT OF MEMORY", file=sys.stderr)
            if device.type == "cuda":
                torch.cuda.empty_cache()
            right = mid - 1
        except Exception as err:
            print(f"    Batch {b:3d} -> Error: {err}", file=sys.stderr)
            right = mid - 1

    target_effective = config.schedule.reference_effective_batch or 64
    acc_steps = max(1, math.ceil(target_effective / best_batch))
    print(
        f"--> [Autotune] Selected optimal physical batch size: {best_batch} "
        f"(accumulation_steps={acc_steps}, effective_batch={best_batch * acc_steps})",
        file=sys.stderr,
    )

    return best_batch, acc_steps
