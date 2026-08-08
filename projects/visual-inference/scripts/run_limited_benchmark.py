"""Verify and run the frozen bounded LiteFPN/AttnRes benchmark matrix."""

from __future__ import annotations

import argparse
import json
import os
import statistics
import subprocess
import sys
from pathlib import Path
from typing import Any

from student_detector.provenance import sha256_file

PROJECT_ROOT = Path(__file__).resolve().parents[1]
LOCK_PATH = PROJECT_ROOT / "configs/benchmarks/phase3_bounded_v1.lock.json"


def _resolve_locked_path(value: str) -> Path:
    return (PROJECT_ROOT / value).resolve()


def verify_benchmark() -> dict[str, Any]:
    """Fail if a frozen config or manifest has changed or disappeared."""
    lock = json.loads(LOCK_PATH.read_text(encoding="utf-8"))
    entries = [
        lock["config"],
        lock["train_manifest"],
        lock["validation_manifest"],
        lock["hbb_baseline"]["checkpoint"],
        lock["hbb_baseline"]["report"],
    ]
    for entry in entries:
        path = _resolve_locked_path(entry["path"])
        if not path.is_file():
            raise FileNotFoundError(f"frozen benchmark input is missing: {path}")
        if "size" in entry and path.stat().st_size != entry["size"]:
            raise ValueError(
                f"frozen benchmark size mismatch for {path}: "
                f"expected {entry['size']}, found {path.stat().st_size}"
            )
        actual = sha256_file(path)
        if actual != entry["sha256"]:
            raise ValueError(
                f"frozen benchmark hash mismatch for {path}: "
                f"expected {entry['sha256']}, found {actual}"
            )
    return lock


def _load_hbb_baseline(lock: dict[str, Any]) -> dict[str, Any]:
    baseline = lock["hbb_baseline"]
    report_path = _resolve_locked_path(baseline["report"]["path"])
    report = json.loads(report_path.read_text(encoding="utf-8"))
    contract = report["matched_contract"]
    expected_manifest = lock["validation_manifest"]["sha256"]
    if contract.get("validation_manifest_sha256") != expected_manifest:
        raise ValueError("HBB report does not use the frozen validation manifest")
    if contract.get("checkpoint_state") != baseline["checkpoint_state"]:
        raise ValueError("HBB report checkpoint state does not match the lock")
    if report["artifacts"].get("hbb_checkpoint_sha256") != baseline["checkpoint"]["sha256"]:
        raise ValueError("HBB report checkpoint hash does not match the lock")
    return {
        "neck_type": baseline["name"],
        "state": baseline["checkpoint_state"],
        "seed": "fixed",
        "global_step": baseline["training_optimizer_steps"],
        **{
            key: report["hbb"]["metrics"].get(key)
            for key in (
                "ar/100",
                "recall/100@0.50",
                "recall/100@0.75",
                "matched_iou/median",
            )
        },
        "checkpoint": str(_resolve_locked_path(baseline["checkpoint"]["path"])),
        "evidence": "historical_fixed_checkpoint_same_validation_manifest",
    }


def _parse_csv(value: str, cast: type) -> list[Any]:
    return [cast(item.strip()) for item in value.split(",") if item.strip()]


def prepare_indexes(config_path: Path) -> None:
    """Build shared immutable indexes before parallel neck jobs can race."""
    from student_detector.config import load_phase3_config
    from student_detector.data import build_coco_parquet_index

    config = load_phase3_config(config_path)
    build_coco_parquet_index(
        config.data.quad_train_annotations or config.data.train_annotations,
        config.data.index_dir / "quad_train.sqlite",
    )
    build_coco_parquet_index(
        config.data.quad_val_annotations or config.data.val_annotations,
        config.data.index_dir / "quad_val.sqlite",
    )


def _training_command(
    *,
    config: Path,
    neck: str,
    seed: int,
    output_dir: Path,
    num_processes: int,
    no_amp: bool,
) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "accelerate.commands.launch",
        "--num_processes",
        str(num_processes),
        "--num_machines",
        "1",
        "--dynamo_backend",
        "no",
        "--mixed_precision",
        "no" if no_amp else "fp16",
        str(PROJECT_ROOT / "scripts/train_quad_proposals.py"),
        "--config",
        str(config),
        "--neck-type",
        neck,
        "--seed",
        str(seed),
        "--output-dir",
        str(output_dir),
    ]
    if no_amp:
        command.append("--no-amp")
    return command


def summarize(
    output_root: Path,
    variants: list[str],
    seeds: list[int],
    lock: dict[str, Any],
) -> Path:
    """Collect final raw/EMA metrics from every completed checkpoint."""
    import torch

    rows: list[dict[str, Any]] = []
    for neck in variants:
        for seed in seeds:
            checkpoint_path = output_root / neck / f"seed_{seed}" / "last.pt"
            if not checkpoint_path.exists():
                continue
            checkpoint = torch.load(
                checkpoint_path, map_location="cpu", weights_only=False
            )
            metrics = checkpoint.get("metrics", {})
            for state in ("raw", "ema"):
                state_metrics = metrics.get(state, {})
                rows.append({
                    "neck_type": neck,
                    "seed": seed,
                    "state": state,
                    "global_step": checkpoint.get("global_step"),
                    "ar/100": state_metrics.get("ar/100"),
                    "recall/100@0.50": state_metrics.get("recall/100@0.50"),
                    "recall/100@0.75": state_metrics.get("recall/100@0.75"),
                    "matched_iou/median": state_metrics.get("matched_iou/median"),
                    "checkpoint": str(checkpoint_path),
                })
    hbb_baseline = _load_hbb_baseline(lock)
    rows.insert(0, hbb_baseline)
    metric_keys = (
        "ar/100",
        "recall/100@0.50",
        "recall/100@0.75",
        "matched_iou/median",
    )
    aggregates: list[dict[str, Any]] = []
    for neck in variants:
        for state in ("raw", "ema"):
            matching = [
                row for row in rows
                if row["neck_type"] == neck and row["state"] == state
            ]
            aggregate: dict[str, Any] = {
                "neck_type": neck,
                "state": state,
                "runs": len(matching),
            }
            for key in metric_keys:
                values = [row[key] for row in matching if row[key] is not None]
                aggregate[f"{key}/mean"] = (
                    statistics.fmean(values) if values else None
                )
                aggregate[f"{key}/stdev"] = (
                    statistics.stdev(values) if len(values) > 1 else None
                )
            aggregates.append(aggregate)
    aggregates.insert(0, {
        "neck_type": hbb_baseline["neck_type"],
        "state": hbb_baseline["state"],
        "runs": 1,
        **{
            f"{key}/mean": hbb_baseline[key]
            for key in metric_keys
        },
        **{f"{key}/stdev": None for key in metric_keys},
    })
    paired_deltas: list[dict[str, Any]] = []
    by_identity = {
        (row["neck_type"], row["seed"], row["state"]): row for row in rows
    }
    for seed in seeds:
        for state in ("raw", "ema"):
            lite = by_identity.get(("lite", seed, state))
            attention = by_identity.get(("attn_res", seed, state))
            if lite is None or attention is None:
                continue
            paired_deltas.append({
                "seed": seed,
                "state": state,
                **{
                    f"delta/{key}": attention[key] - lite[key]
                    for key in metric_keys
                    if attention[key] is not None and lite[key] is not None
                },
            })
    quad_minus_hbb = [
        {
            "neck_type": row["neck_type"],
            "seed": row["seed"],
            "state": row["state"],
            **{
                f"delta/{key}": row[key] - hbb_baseline[key]
                for key in metric_keys
                if row[key] is not None and hbb_baseline[key] is not None
            },
        }
        for row in rows
        if row["neck_type"] in variants and row["state"] == "raw"
    ]
    summary = {
        "schema_version": "phase3-bounded-benchmark-summary.v1",
        "runs": rows,
        "aggregates": aggregates,
        "paired_attn_res_minus_lite": paired_deltas,
        "quad_minus_hbb_control_v1": quad_minus_hbb,
        "hbb_baseline_note": (
            "Fixed 1240-step historical checkpoint evaluated on the same frozen "
            "36-image validation manifest; not a matched 760-step training run."
        ),
    }
    target = output_root / "summary.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return target


def _format_metric(value: Any) -> str:
    return "-" if value is None else f"{float(value):.4f}"


def _print_table(headers: list[str], rows: list[list[str]]) -> None:
    widths = [len(header) for header in headers]
    for row in rows:
        widths = [
            max(width, len(value)) for width, value in zip(widths, row, strict=True)
        ]
    print(" | ".join(value.ljust(width) for value, width in zip(headers, widths)))
    print("-+-".join("-" * width for width in widths))
    for row in rows:
        print(" | ".join(value.ljust(width) for value, width in zip(row, widths)))


def print_summary(summary_path: Path) -> None:
    """Render the benchmark JSON as compact terminal comparison tables."""
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    print("\nPer-run validation results")
    _print_table(
        ["Neck", "State", "Seed", "AR@100", "R@50", "R@75", "Median IoU"],
        [
            [
                row["neck_type"],
                row["state"],
                str(row["seed"]),
                _format_metric(row.get("ar/100")),
                _format_metric(row.get("recall/100@0.50")),
                _format_metric(row.get("recall/100@0.75")),
                _format_metric(row.get("matched_iou/median")),
            ]
            for row in summary["runs"]
        ],
    )

    print("\nAggregate results (mean +/- sample standard deviation)")
    metric_keys = (
        ("ar/100", "AR@100"),
        ("recall/100@0.50", "R@50"),
        ("recall/100@0.75", "R@75"),
        ("matched_iou/median", "Median IoU"),
    )
    aggregate_rows: list[list[str]] = []
    for row in summary["aggregates"]:
        values = []
        for key, _ in metric_keys:
            mean = _format_metric(row.get(f"{key}/mean"))
            stdev = _format_metric(row.get(f"{key}/stdev"))
            values.append(f"{mean} +/- {stdev}" if stdev != "-" else mean)
        aggregate_rows.append([
            row["neck_type"], row["state"], str(row["runs"]), *values
        ])
    _print_table(
        ["Neck", "State", "Runs", *(label for _, label in metric_keys)],
        aggregate_rows,
    )

    paired = summary["paired_attn_res_minus_lite"]
    if paired:
        print("\nPaired deltas (AttnRes - LiteFPN; positive favors AttnRes)")
        _print_table(
            ["State", "Seed", "Delta AR", "Delta R@50", "Delta R@75", "Delta IoU"],
            [
                [
                    row["state"],
                    str(row["seed"]),
                    _format_metric(row.get("delta/ar/100")),
                    _format_metric(row.get("delta/recall/100@0.50")),
                    _format_metric(row.get("delta/recall/100@0.75")),
                    _format_metric(row.get("delta/matched_iou/median")),
                ]
                for row in paired
            ],
        )

    versus_hbb = summary.get("quad_minus_hbb_control_v1", [])
    if versus_hbb:
        print("\nRaw quad deltas versus fixed HBB control (Quad - HBB)")
        _print_table(
            ["Neck", "Seed", "Delta AR", "Delta R@50", "Delta R@75", "Delta IoU"],
            [
                [
                    row["neck_type"],
                    str(row["seed"]),
                    _format_metric(row.get("delta/ar/100")),
                    _format_metric(row.get("delta/recall/100@0.50")),
                    _format_metric(row.get("delta/recall/100@0.75")),
                    _format_metric(row.get("delta/matched_iou/median")),
                ]
                for row in versus_hbb
            ],
        )
        print(f"\nNote: {summary['hbb_baseline_note']}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", default="42,43,44")
    parser.add_argument("--necks", default="lite,attn_res")
    parser.add_argument("--num-processes", type=int, default=1)
    parser.add_argument(
        "--parallel-neck-devices",
        help=(
            "Comma-separated GPU IDs, one per --necks entry; runs the necks "
            "concurrently for each seed (for example: 0,1)"
        ),
    )
    parser.add_argument("--no-amp", action="store_true")
    parser.add_argument("--verify-only", action="store_true")
    parser.add_argument("--summarize-only", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--skip-existing", action="store_true")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=PROJECT_ROOT / "artifacts/phase3/benchmarks/bounded_v1",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    lock = verify_benchmark()
    print(f"Verified frozen benchmark {lock['benchmark']}")
    if args.verify_only:
        return
    seeds = _parse_csv(args.seeds, int)
    variants = _parse_csv(args.necks, str)
    invalid = sorted(set(variants) - set(lock["variants"]))
    if invalid:
        raise ValueError(f"unsupported benchmark necks: {invalid}")
    if args.num_processes < 1:
        raise ValueError("--num-processes must be positive")
    parallel_devices = (
        _parse_csv(args.parallel_neck_devices, str)
        if args.parallel_neck_devices
        else []
    )
    if parallel_devices and args.num_processes != 1:
        raise ValueError(
            "--parallel-neck-devices cannot be combined with --num-processes > 1"
        )
    if parallel_devices and len(parallel_devices) != len(variants):
        raise ValueError(
            "--parallel-neck-devices must provide one GPU ID per neck variant"
        )
    batch_size = int(lock["optimization"]["batch_size"])
    if batch_size % args.num_processes != 0:
        raise ValueError(
            f"frozen batch size {batch_size} is not divisible by "
            f"--num-processes={args.num_processes}"
        )
    output_root = args.output_root.resolve()
    if not args.summarize_only:
        config = _resolve_locked_path(lock["config"]["path"])
        if not args.dry_run:
            prepare_indexes(config)
        for seed in seeds:
            pending: list[tuple[str, list[str], Path, str | None]] = []
            for index, neck in enumerate(variants):
                output_dir = output_root / neck / f"seed_{seed}"
                if args.skip_existing and (output_dir / "last.pt").exists():
                    print(f"Skipping completed run {neck}/seed_{seed}")
                    continue
                command = _training_command(
                    config=config,
                    neck=neck,
                    seed=seed,
                    output_dir=output_dir,
                    num_processes=args.num_processes,
                    no_amp=args.no_amp,
                )
                device = parallel_devices[index] if parallel_devices else None
                prefix = f"CUDA_VISIBLE_DEVICES={device} " if device else ""
                print(prefix + " ".join(command), flush=True)
                pending.append((neck, command, output_dir, device))
            if args.dry_run:
                continue
            if parallel_devices:
                processes: list[tuple[str, subprocess.Popen]] = []
                for neck, command, _, assigned_device in pending:
                    environment = os.environ.copy()
                    environment["CUDA_VISIBLE_DEVICES"] = str(assigned_device)
                    processes.append((
                        neck,
                        subprocess.Popen(
                            command, cwd=PROJECT_ROOT, env=environment
                        ),
                    ))
                failures: list[tuple[str, int]] = []
                for neck, process in processes:
                    return_code = process.wait()
                    if return_code != 0:
                        failures.append((neck, return_code))
                if failures:
                    raise RuntimeError(f"parallel benchmark runs failed: {failures}")
            else:
                for _, command, _, _ in pending:
                    subprocess.run(command, cwd=PROJECT_ROOT, check=True)
    if not args.dry_run:
        target = summarize(output_root, variants, seeds, lock)
        print(f"Wrote {target}")
        print_summary(target)


if __name__ == "__main__":
    main()
