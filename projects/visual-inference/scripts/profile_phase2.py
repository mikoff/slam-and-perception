"""Simple CPU latency harness intended to be copied and run on the target Pi."""

from __future__ import annotations

import argparse
import statistics
import time

import torch

from student_detector import StudentDetector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-size", type=int, default=384)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--iterations", type=int, default=20)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    torch.set_num_threads(args.threads)
    torch.manual_seed(0)
    model = StudentDetector(pretrained_backbone=False).eval()
    example = torch.randn(1, 3, args.image_size, args.image_size)

    with torch.inference_mode():
        for _ in range(args.warmup):
            model(example)
        samples_ms = []
        for _ in range(args.iterations):
            start = time.perf_counter()
            model(example)
            samples_ms.append((time.perf_counter() - start) * 1000.0)

    parameters = sum(parameter.numel() for parameter in model.parameters())
    median_ms = statistics.median(samples_ms)
    mean_ms = statistics.mean(samples_ms)
    print(f"input: 1x3x{args.image_size}x{args.image_size}")
    print(f"threads: {args.threads}")
    print(f"parameters: {parameters:,}")
    print(f"mean latency: {mean_ms:.2f} ms")
    print(f"median latency: {median_ms:.2f} ms")
    print(f"median throughput: {1000.0 / median_ms:.2f} images/s")
    print("scope: raw detector only; preprocessing, decoding, and NMS are excluded")


if __name__ == "__main__":
    main()
