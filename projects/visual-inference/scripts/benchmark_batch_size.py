"""Measure safe quad-training batch throughput on the current GPU."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
from dataclasses import replace
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import torch
from accelerate import Accelerator
from torch.utils.data import DataLoader

from student_detector.config import Phase3Config, load_phase3_config
from student_detector.data import DomainMixtureBatchSampler
from student_detector.model import QuadProposalDetector
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_data import (
    QuadProposalDataset,
    collate_quad_proposal_samples,
)
from student_detector.quad_losses import QuadProposalLoss
from student_detector.quad_targets import QuadTargetBuilder
from student_detector.training_optimization import (
    ExponentialMovingAverage,
    build_detector_optimizer,
    freeze_backbone_batch_norm,
    set_reproducibility_seed,
)

REPORT_SCHEMA = "visual-inference-batch-preflight.v1"


def parse_candidates(value: str) -> list[int]:
    """Parse a sorted, unique list of positive candidate batch sizes."""
    try:
        candidates = sorted(
            {int(item.strip()) for item in value.split(",") if item.strip()}
        )
    except ValueError as error:
        raise ValueError("batch candidates must be comma-separated integers") from error
    if not candidates or candidates[0] < 1:
        raise ValueError("at least one positive batch candidate is required")
    return candidates


def recommend_batch(
    results: list[dict[str, Any]],
    *,
    minimum_headroom: float,
    plateau_fraction: float,
) -> int | None:
    """Choose the smallest safe batch within the throughput plateau."""
    if any(result.get("status") in {"error", "timeout"} for result in results):
        return None
    eligible = [
        result
        for result in results
        if result.get("status") == "pass"
        and float(result["memory_headroom_fraction"]) >= minimum_headroom
    ]
    if not eligible:
        return None
    peak_throughput = max(float(result["images_per_second"]) for result in eligible)
    plateau = [
        result
        for result in eligible
        if float(result["images_per_second"]) >= peak_throughput * plateau_fraction
    ]
    return min(int(result["batch_size"]) for result in plateau)


def _percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    index = min(math.ceil(probability * len(ordered)) - 1, len(ordered) - 1)
    return ordered[max(index, 0)]


def _criterion(config: Phase3Config) -> QuadProposalLoss:
    return QuadProposalLoss(
        strides=config.assignment.strides,
        quality_weight=config.quad.quality_weight,
        corner_weight=config.quad.corner_weight,
        corner_smooth_l1_beta=config.quad.corner_smooth_l1_beta,
        gwd_weight=config.quad.gwd_weight,
        validity_weight=config.quad.validity_weight,
        quality_focal_beta=config.quad.quality_focal_beta,
        quality_target_mode=config.quad.quality_target_mode,
        quality_blend=config.quad.quality_blend,
        geometry_quality_target=config.quad.geometry_quality_target,
    )


def _target_builder(config: Phase3Config) -> QuadTargetBuilder:
    return QuadTargetBuilder(
        QuadAssigner(
            strides=config.assignment.strides,
            top_k=config.quad.top_k,
            gamma=config.quad.gamma,
            scale_sigma=config.quad.scale_sigma,
            eligible_levels=config.quad.eligible_levels,
            scale_measure=config.quad.scale_measure,
        ),
        weak_negative_weight=config.quad.weak_negative_weight,
    )


def _feature_shapes(output: Any) -> tuple[tuple[int, int], ...]:
    return tuple((tensor.shape[-2], tensor.shape[-1]) for tensor in output.quality)


def _build_targets(
    builder: QuadTargetBuilder,
    samples: list[Any],
    prediction: Any,
    device: torch.device,
) -> Any:
    """Match the production trainer's keyword-only target-builder contract."""
    return builder(samples, _feature_shapes(prediction), device=device)


def _write_result(path: Path, result: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _run_candidate(command: list[str], timeout: int) -> int:
    """Run one candidate and kill its DataLoader process group on timeout."""
    process = subprocess.Popen(command, start_new_session=True)
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        raise


def _print_summary(results: list[dict[str, Any]], recommendation: int | None) -> None:
    print(
        "\nBatch | Status | Images/s | P50 step | P90 step | Peak reserved | Headroom"
    )
    print("------+--------+----------+----------+----------+---------------+---------")
    for result in results:
        if result["status"] == "pass":
            print(
                f"{int(result['batch_size']):5d} | pass   | "
                f"{float(result['images_per_second']):8.1f} | "
                f"{float(result['median_step_seconds']):7.3f}s | "
                f"{float(result['p90_step_seconds']):7.3f}s | "
                f"{int(result['peak_reserved_bytes']) / 2**30:10.1f} GiB | "
                f"{float(result['memory_headroom_fraction']):7.1%}"
            )
        else:
            print(
                f"{int(result['batch_size']):5d} | {str(result['status']):6s} |"
                "        - |        - |        - |             - |       -"
            )
    print(
        f"\nRecommended batch: {recommendation if recommendation else 'none'}",
        flush=True,
    )


def run_worker(args: argparse.Namespace) -> int:
    """Run one candidate in an isolated process so OOM state is discarded."""
    output = args.worker_output.resolve()
    try:
        if not torch.cuda.is_available():
            raise RuntimeError("batch preflight requires a CUDA GPU")
        shared_memory = os.statvfs("/dev/shm")
        shared_memory_total = shared_memory.f_frsize * shared_memory.f_blocks
        shared_memory_free = shared_memory.f_frsize * shared_memory.f_bavail
        print(
            f"[Batch {args.worker_batch}] /dev/shm "
            f"total={shared_memory_total / 2**30:.1f}GiB "
            f"free={shared_memory_free / 2**30:.1f}GiB",
            flush=True,
        )
        if args.workers > 0 and shared_memory_total < 16 * 2**30:
            raise RuntimeError(
                "parallel DataLoader requires at least 16 GiB /dev/shm; "
                f"container exposes {shared_memory_total / 2**20:.0f} MiB"
            )
        config = load_phase3_config(args.config)
        config = replace(
            config,
            data=replace(
                config.data, batch_size=args.worker_batch, workers=args.workers
            ),
            schedule=replace(config.schedule, accumulation_steps=1),
        )
        accelerator = Accelerator(
            mixed_precision="fp16" if config.schedule.amp else "no",
            gradient_accumulation_steps=1,
            split_batches=True,
        )
        set_reproducibility_seed(config.schedule.seed)
        dataset = QuadProposalDataset(
            config.data.quad_train_annotations or config.data.train_annotations,
            config.data.image_root,
            config.data.index_dir / "quad_train.sqlite",
            config.data,
            config.augmentation,
            training=True,
            seed=config.schedule.seed,
        )
        measured_batches = args.warmup_steps + args.measure_steps
        sampler = DomainMixtureBatchSampler(
            dataset,
            config.data.batch_size,
            domain_weights=config.data.domain_weights,
            source_weights=config.data.source_weights,
            empty_fraction=config.data.empty_fraction,
            seed=config.schedule.seed,
            batches_per_epoch=measured_batches,
        )
        loader = DataLoader(
            dataset,
            batch_sampler=sampler,
            num_workers=config.data.workers,
            collate_fn=collate_quad_proposal_samples,
            pin_memory=True,
            persistent_workers=config.data.workers > 0,
        )
        model = QuadProposalDetector(
            pretrained_backbone=False, neck_type=config.neck_type
        )
        optimizer = build_detector_optimizer(model, config)
        model, optimizer, loader = accelerator.prepare(model, optimizer, loader)
        base_model = accelerator.unwrap_model(model)
        ema = ExponentialMovingAverage(
            base_model, config.schedule.ema_decay, config.schedule.ema_ramp_steps
        )
        freeze_backbone_batch_norm(base_model.backbone)
        model.train()
        criterion = _criterion(config).to(accelerator.device)
        target_builder = _target_builder(config)
        optimizer.zero_grad(set_to_none=True)
        iterator = iter(loader)
        step_times: list[float] = []
        data_times: list[float] = []
        last_loss = 0.0

        for step in range(measured_batches):
            data_started = time.perf_counter()
            images, samples = next(iterator)
            data_seconds = time.perf_counter() - data_started
            step_started = time.perf_counter()
            images = images.to(accelerator.device, non_blocking=True)
            with accelerator.autocast():
                prediction = model(images)
            targets = _build_targets(
                target_builder, samples, prediction, accelerator.device
            )
            with accelerator.autocast():
                losses = criterion(prediction, targets)
            accelerator.backward(losses.total)
            accelerator.clip_grad_norm_(
                model.parameters(), config.schedule.gradient_clip_norm
            )
            optimizer.step()
            optimizer.zero_grad(set_to_none=True)
            ema.update(base_model)
            torch.cuda.synchronize()
            elapsed = time.perf_counter() - step_started
            last_loss = float(losses.total.detach())
            if step + 1 == args.warmup_steps:
                torch.cuda.reset_peak_memory_stats()
            elif step >= args.warmup_steps:
                data_times.append(data_seconds)
                step_times.append(elapsed)
            if step == 0 or (step + 1) % 10 == 0:
                print(
                    f"[Batch {args.worker_batch}] step {step + 1}/{measured_batches} "
                    f"data={data_seconds:.3f}s compute={elapsed:.3f}s",
                    flush=True,
                )

        total_memory = torch.cuda.get_device_properties(0).total_memory
        peak_allocated = torch.cuda.max_memory_allocated()
        peak_reserved = torch.cuda.max_memory_reserved()
        total_seconds = sum(step_times) + sum(data_times)
        result = {
            "status": "pass",
            "batch_size": args.worker_batch,
            "dataset_images": len(dataset),
            "measure_steps": args.measure_steps,
            "median_step_seconds": _percentile(step_times, 0.50),
            "p90_step_seconds": _percentile(step_times, 0.90),
            "median_data_seconds": _percentile(data_times, 0.50),
            "images_per_second": args.worker_batch * args.measure_steps / total_seconds,
            "peak_allocated_bytes": peak_allocated,
            "peak_reserved_bytes": peak_reserved,
            "total_memory_bytes": total_memory,
            "memory_headroom_fraction": 1.0 - peak_reserved / total_memory,
            "last_loss": last_loss,
            "shared_memory_total_bytes": shared_memory_total,
            "shared_memory_free_bytes_at_start": shared_memory_free,
        }
    except torch.OutOfMemoryError as error:
        result = {
            "status": "oom",
            "batch_size": args.worker_batch,
            "error": str(error),
        }
    except Exception as error:
        result = {
            "status": "error",
            "batch_size": args.worker_batch,
            "error": f"{type(error).__name__}: {error}",
        }
    _write_result(output, result)
    print(json.dumps(result, sort_keys=True), flush=True)
    return 0 if result["status"] in {"pass", "oom"} else 1


def run_preflight(args: argparse.Namespace) -> int:
    """Orchestrate isolated candidates and write the final recommendation."""
    if not torch.cuda.is_available():
        raise RuntimeError("batch preflight requires a CUDA GPU")
    candidates = parse_candidates(args.candidates)
    results: list[dict[str, Any]] = []
    with tempfile.TemporaryDirectory(prefix="batch-preflight-") as temporary:
        temporary_root = Path(temporary)
        for batch_size in candidates:
            result_path = temporary_root / f"batch-{batch_size}.json"
            command = [
                sys.executable,
                str(Path(__file__).resolve()),
                "--config",
                str(args.config),
                "--workers",
                str(args.workers),
                "--warmup-steps",
                str(args.warmup_steps),
                "--measure-steps",
                str(args.measure_steps),
                "--worker-batch",
                str(batch_size),
                "--worker-output",
                str(result_path),
            ]
            print(f"[Batch preflight] testing batch {batch_size}", flush=True)
            try:
                return_code = _run_candidate(command, args.candidate_timeout)
            except subprocess.TimeoutExpired:
                results.append(
                    {
                        "status": "timeout",
                        "batch_size": batch_size,
                        "error": f"candidate exceeded {args.candidate_timeout} seconds",
                    }
                )
                print(
                    f"[Batch preflight] batch {batch_size} timed out; "
                    "skipping larger candidates",
                    flush=True,
                )
                break
            if result_path.is_file():
                result = json.loads(result_path.read_text(encoding="utf-8"))
            else:
                result = {
                    "status": "error",
                    "batch_size": batch_size,
                    "error": f"worker exited {return_code} without a result",
                }
            results.append(result)
            if result["status"] in {"error", "oom"}:
                print(
                    f"[Batch preflight] batch {batch_size} returned "
                    f"{result['status']}; skipping larger candidates",
                    flush=True,
                )
                break

    recommendation = recommend_batch(
        results,
        minimum_headroom=args.minimum_headroom,
        plateau_fraction=args.plateau_fraction,
    )
    config_bytes = args.config.resolve().read_bytes()
    report = {
        "schema_version": REPORT_SCHEMA,
        "created_at": datetime.now(UTC).isoformat(),
        "config": str(args.config),
        "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "gpu": torch.cuda.get_device_name(0),
        "torch": torch.__version__,
        "cuda": torch.version.cuda,
        "candidates": candidates,
        "minimum_headroom": args.minimum_headroom,
        "plateau_fraction": args.plateau_fraction,
        "results": results,
        "recommended_batch_size": recommendation,
        "quality_gate_required": True,
    }
    _write_result(args.output.resolve(), report)
    _print_summary(results, recommendation)
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    if recommendation is None:
        print("No candidate met the required memory headroom", file=sys.stderr)
        return 1
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--candidates", default="16,32,64,96,128")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--warmup-steps", type=int, default=10)
    parser.add_argument("--measure-steps", type=int, default=50)
    parser.add_argument("--minimum-headroom", type=float, default=0.15)
    parser.add_argument("--plateau-fraction", type=float, default=0.95)
    parser.add_argument("--candidate-timeout", type=int, default=900)
    parser.add_argument("--output", type=Path, default=Path("batch-preflight.json"))
    parser.add_argument("--worker-batch", type=int, help=argparse.SUPPRESS)
    parser.add_argument("--worker-output", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.workers < 0 or args.warmup_steps < 1 or args.measure_steps < 1:
        parser.error("workers must be non-negative and step counts must be positive")
    if not 0 < args.minimum_headroom < 1:
        parser.error("minimum headroom must be in (0, 1)")
    if not 0 < args.plateau_fraction <= 1:
        parser.error("plateau fraction must be in (0, 1]")
    if bool(args.worker_batch) != bool(args.worker_output):
        parser.error("worker batch and output must be supplied together")
    return args


def main() -> None:
    args = parse_args()
    raise SystemExit(run_worker(args) if args.worker_batch else run_preflight(args))


if __name__ == "__main__":
    main()
