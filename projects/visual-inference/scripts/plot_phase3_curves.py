"""Export Phase-3 JSONL logs as CSV tables and compact learning-curve PNGs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args()


def _read_jsonl(path: Path) -> list[dict[str, object]]:
    if not path.exists():
        return []
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    fields = sorted({key for row in rows for key in row})
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _series(
    rows: list[dict[str, object]], key: str
) -> tuple[list[float], list[float]]:
    points = [
        (float(row["global_step"]), float(row[key]))
        for row in rows
        if "global_step" in row and key in row
    ]
    return [point[0] for point in points], [point[1] for point in points]


def _plot(
    path: Path,
    rows: list[dict[str, object]],
    panels: list[tuple[str, tuple[str, ...]]],
) -> None:
    if not rows:
        return
    figure, axes = plt.subplots(
        len(panels), 1, figsize=(10, 3.2 * len(panels)), squeeze=False
    )
    for axis, (title, keys) in zip(axes[:, 0], panels, strict=True):
        for key in keys:
            x, y = _series(rows, key)
            if x:
                axis.plot(x, y, label=key)
        axis.set_title(title)
        axis.set_xlabel("optimizer step")
        axis.grid(alpha=0.25)
        axis.legend(loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    run_dir = args.run_dir.resolve()
    output_dir = (
        args.output_dir.resolve()
        if args.output_dir
        else run_dir / "curves"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    metrics = _read_jsonl(run_dir / "metrics.jsonl")
    progress = _read_jsonl(run_dir / "progress.jsonl")
    _write_csv(output_dir / "epoch_metrics.csv", metrics)
    _write_csv(output_dir / "step_progress.csv", progress)
    _plot(
        output_dir / "training.png",
        progress,
        [
            (
                "Training losses",
                (
                    "loss/total",
                    "loss/objectness",
                    "loss/box_ciou",
                    "loss/box_ltrb",
                    "loss/centerness",
                ),
            ),
            ("Learning rates", ("lr/fpn_head", "lr/backbone")),
            (
                "Throughput and allocated memory",
                ("images_per_second", "cuda/peak_memory_gib"),
            ),
        ],
    )
    _plot(
        output_dir / "validation.png",
        metrics,
        [
            (
                "Proposal recall",
                (
                    "val/recall/100@50",
                    "val/recall/100@75",
                    "val/recall/50@50",
                    "val/recall/10@50",
                ),
            ),
            ("Average precision", ("val/ap/50", "val/ap/75")),
            (
                "Domain recall",
                (
                    "val/recall/general/100@50",
                    "val/recall/automotive/100@50",
                    "val/recall/fisheye/100@50",
                ),
            ),
        ],
    )
    print(output_dir)


if __name__ == "__main__":
    main()
