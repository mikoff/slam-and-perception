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

from student_detector.autotune import benchmark_batch_size
from student_detector.config import load_phase3_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark physical batch size for quad detector training.")
    parser.add_argument("--config", type=Path, default=Path("configs/phase3_attnres.yaml"))
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--batch-sizes", default="16,32,64,128,256", help="Comma-separated physical batch sizes to test")
    parser.add_argument("--effective-batch-size", type=int, default=64)
    parser.add_argument("--warmup-steps", type=int, default=5)
    parser.add_argument("--timed-steps", type=int, default=10)
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
            config = load_phase3_config(args.config)
            res = benchmark_batch_size(
                batch_size=batch_size,
                config=config,
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
