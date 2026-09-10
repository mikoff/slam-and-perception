"""Export and profile one fixed-shape proposal-detector candidate."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import resource
import statistics
import time
from typing import Any

import torch

from student_detector.checkpoints import checkpoint_neck_type, load_model_state_strict
from student_detector.deployment import (
    DetectorExportView,
    count_macs,
    flatten_detector_output,
    parity_metrics,
)
from student_detector.model import QuadProposalDetector, StudentDetector


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kind", choices=("hbb", "quad"), required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--checkpoint-state", default="ema_model")
    parser.add_argument("--artifact", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--input-size", type=int, default=384)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--repeats", type=int, default=50)
    return parser.parse_args()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _rss_mib(*, peak: bool) -> float:
    if peak:
        # Linux ru_maxrss is KiB. This script's host contract is Linux.
        return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    for line in Path("/proc/self/status").read_text().splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1]) / 1024
    raise RuntimeError("/proc/self/status did not expose VmRSS")


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    rank = fraction * (len(ordered) - 1)
    lower = math.floor(rank)
    upper = math.ceil(rank)
    return ordered[lower] + (rank - lower) * (ordered[upper] - ordered[lower])


def _cpu_model() -> str:
    for line in Path("/proc/cpuinfo").read_text().splitlines():
        if line.startswith("model name"):
            return line.split(":", 1)[1].strip()
    return platform.processor() or "unknown"


def _load_model(kind: str, checkpoint: dict[str, Any], state: str) -> torch.nn.Module:
    neck_type = checkpoint_neck_type(checkpoint)
    model: torch.nn.Module = (
        StudentDetector(pretrained_backbone=False, neck_type=neck_type)
        if kind == "hbb"
        else QuadProposalDetector(pretrained_backbone=False, neck_type=neck_type)
    )
    load_model_state_strict(
        model,
        checkpoint,
        kind=kind,  # type: ignore[arg-type]
        neck_type=neck_type,
        state_key=state,
    )
    return model.eval()


def _output_contract(
    kind: str, tensors: tuple[torch.Tensor, ...]
) -> list[dict[str, Any]]:
    names = (
        [f"objectness_p{level}" for level in (3, 4, 5)]
        + [f"box_distances_p{level}" for level in (3, 4, 5)]
        + [f"centerness_p{level}" for level in (3, 4, 5)]
        if kind == "hbb"
        else [f"quality_p{level}" for level in (3, 4, 5)]
        + [f"corner_offsets_p{level}" for level in (3, 4, 5)]
    )
    return [
        {"name": name, "shape": list(tensor.shape), "dtype": str(tensor.dtype)}
        for name, tensor in zip(names, tensors, strict=True)
    ]


def main() -> None:
    args = _args()
    if min(args.input_size, args.threads, args.warmup, args.repeats) < 1:
        raise ValueError("input size, threads, warmup, and repeats must be positive")
    if args.input_size % 32:
        raise ValueError("input size must be divisible by 32")
    torch.set_num_threads(args.threads)
    torch.manual_seed(20260906)
    startup_rss = _rss_mib(peak=False)
    checkpoint_path = args.checkpoint.resolve()
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    model = _load_model(args.kind, checkpoint, args.checkpoint_state)
    loaded_rss = _rss_mib(peak=False)
    example = torch.randn(1, 3, args.input_size, args.input_size)
    macs = count_macs(model, example)
    view = DetectorExportView(model).eval()
    with torch.inference_mode():
        reference = flatten_detector_output(model(example))
        for _ in range(args.warmup):
            view(example)
        durations = []
        for _ in range(args.repeats):
            started = time.perf_counter()
            view(example)
            durations.append(time.perf_counter() - started)
    inference_peak_rss = _rss_mib(peak=True)

    exported = torch.export.export(view, (example,), strict=True)
    exported_result = exported.module()(example)
    artifact = args.artifact.resolve()
    artifact.parent.mkdir(parents=True, exist_ok=True)
    torch.export.save(exported, artifact)
    loaded = torch.export.load(artifact)
    loaded_result = loaded.module()(example)
    export_parity = parity_metrics(reference, exported_result)
    reload_parity = parity_metrics(reference, loaded_result)
    passed = bool(
        export_parity["allclose_atol_1e-5_rtol_1e-5"]
        and reload_parity["allclose_atol_1e-5_rtol_1e-5"]
    )
    parameter_count = sum(parameter.numel() for parameter in model.parameters())
    parameter_bytes = sum(
        parameter.numel() * parameter.element_size() for parameter in model.parameters()
    )
    report = {
        "schema_version": "visual-inference-deployment-candidate.v1",
        "status": "pass" if passed else "parity_failure",
        "kind": args.kind,
        "checkpoint": str(checkpoint_path),
        "checkpoint_sha256": _sha256(checkpoint_path),
        "checkpoint_state": args.checkpoint_state,
        "architecture": checkpoint.get("architecture"),
        "input": {
            "shape": [1, 3, args.input_size, args.input_size],
            "dtype": "torch.float32",
        },
        "raw_output_contract": _output_contract(args.kind, reference),
        "export": {
            "format": "torch.export PT2 archive",
            "artifact": str(artifact),
            "artifact_bytes": artifact.stat().st_size,
            "artifact_sha256": _sha256(artifact),
            "graph_nodes": len(tuple(exported.graph.nodes)),
            "eager_to_exported": export_parity,
            "eager_to_reloaded": reload_parity,
            "decode_and_nms_in_graph": False,
            "quantized": False,
        },
        "cost": {
            "parameters": parameter_count,
            "fp32_parameter_bytes": parameter_bytes,
            "conv_linear_macs": macs,
            "mac_scope": "Conv2d and Linear MACs at batch 1; resize/add/activations excluded",
        },
        "cpu_inference": {
            "warmup_iterations": args.warmup,
            "measured_iterations": args.repeats,
            "threads": args.threads,
            "median_ms": statistics.median(durations) * 1000,
            "p90_ms": _percentile(durations, 0.90) * 1000,
            "images_per_second_from_median": 1 / statistics.median(durations),
            "startup_rss_mib": startup_rss,
            "model_loaded_rss_mib": loaded_rss,
            "inference_peak_process_rss_mib": inference_peak_rss,
            "inference_peak_delta_from_startup_mib": inference_peak_rss - startup_rss,
        },
        "host": {
            "architecture": platform.machine(),
            "cpu": _cpu_model(),
            "logical_cpus": os.cpu_count(),
            "torch": torch.__version__,
            "cuda_available": torch.cuda.is_available(),
            "raspberry_pi": Path("/dev/gpiomem").exists(),
        },
        "unavailable": {
            "peak_training_memory": "not recorded by the matched short runs",
            "current_desktop_gpu_profile": "NVIDIA driver unavailable",
            "raspberry_pi_latency_memory_thermal_power": "no target device attached",
            "int8_executorch": "not implemented in the current deployment path",
        },
    }
    report_path = args.report.resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, indent=2, sort_keys=True))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
