#!/usr/bin/env python3
"""Generate a bounded real-data audit of the shared augmentation policy."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
from html import escape
import json
from pathlib import Path
import sqlite3
from typing import Any

import torch
from PIL import Image, ImageDraw, ImageOps, ImageStat

from student_detector.augmentation import (
    IMAGENET_MEAN,
    IMAGENET_STD,
    effective_augmentation_policy,
    sample_augmentation_parameters,
)
from student_detector.config import load_phase3_config
from student_detector.data import ProposalTransform


DATASETS = (
    "bdd100k_images_100k",
    "coco_2017",
    "nuimages",
    "woodscape_rgb_fisheye",
)
STATE_NAMES = {0: "positive", 1: "ignore", 2: "trusted_background"}
STATE_COLORS = {0: "#22c55e", 1: "#f59e0b", 2: "#3b82f6"}


def _row(
    connection: sqlite3.Connection, dataset: str, condition: str
) -> tuple[Any, ...] | None:
    return connection.execute(
        f"""
        SELECT i.image_id, i.file_name, i.width, i.height, i.source_dataset,
               i.camera_type, a.x1, a.y1, a.x2, a.y2, a.ignore_region,
               a.category_name
        FROM images i JOIN annotations a ON a.image_id=i.image_id
        WHERE i.source_dataset=? AND {condition}
        ORDER BY i.image_id, a.rowid LIMIT 1
        """,
        (dataset,),
    ).fetchone()


def _luminance_rows(
    connection: sqlite3.Connection,
    dataset: str,
    image_root: Path,
) -> tuple[tuple[Any, ...] | None, tuple[Any, ...] | None]:
    candidates = connection.execute(
        """
        SELECT image_id, file_name FROM images
        WHERE source_dataset=? AND positive_count>0
        ORDER BY image_id LIMIT 16
        """,
        (dataset,),
    ).fetchall()
    scored: list[tuple[float, int]] = []
    for image_id, file_name in candidates:
        path = image_root / str(file_name)
        if not path.is_file():
            continue
        with Image.open(path) as loaded:
            thumbnail = loaded.convert("L")
            thumbnail.thumbnail((64, 64))
            scored.append((float(ImageStat.Stat(thumbnail).mean[0]), int(image_id)))
    if not scored:
        return None, None
    result = []
    for _, image_id in (min(scored), max(scored)):
        result.append(
            connection.execute(
                """
                SELECT i.image_id, i.file_name, i.width, i.height, i.source_dataset,
                       i.camera_type, a.x1, a.y1, a.x2, a.y2, a.ignore_region,
                       a.category_name
                FROM images i JOIN annotations a ON a.image_id=i.image_id
                WHERE i.image_id=? AND a.ignore_region=0
                ORDER BY a.rowid LIMIT 1
                """,
                (image_id,),
            ).fetchone()
        )
    return result[0], result[1]


def select_cases(
    connection: sqlite3.Connection, dataset: str, image_root: Path
) -> list[tuple[str, tuple[Any, ...] | None]]:
    """Select deterministic semantic and geometry stress cases."""
    darkest, brightest = _luminance_rows(connection, dataset, image_root)
    cases = [
        ("night-proxy", darkest),
        ("day-proxy", brightest),
        (
            "tiny-object",
            _row(
                connection,
                dataset,
                "a.ignore_region=0 AND MIN(a.x2-a.x1,a.y2-a.y1)<=16",
            ),
        ),
        (
            "boundary-object",
            _row(
                connection,
                dataset,
                "a.ignore_region=0 AND (a.x1<=1 OR a.y1<=1 "
                "OR a.x2>=i.width-1 OR a.y2>=i.height-1)",
            ),
        ),
        ("trusted-background", _row(connection, dataset, "a.ignore_region=2")),
    ]
    if dataset == "woodscape_rgb_fisheye":
        cases.append(("fisheye", _row(connection, dataset, "a.ignore_region=0")))
    return cases


def _tensor_image(tensor: torch.Tensor) -> Image.Image:
    mean = tensor.new_tensor(IMAGENET_MEAN).view(3, 1, 1)
    std = tensor.new_tensor(IMAGENET_STD).view(3, 1, 1)
    pixels = ((tensor * std + mean).clamp(0, 1) * 255).byte().permute(1, 2, 0)
    return Image.fromarray(pixels.cpu().numpy(), mode="RGB")


def _source_panel(
    image: Image.Image, box: tuple[float, ...], color: str
) -> Image.Image:
    annotated = image.copy()
    ImageDraw.Draw(annotated).rectangle(
        box, outline=color, width=max(image.width // 300, 2)
    )
    return ImageOps.pad(annotated, (384, 384), color=(124, 116, 104))


def _render_case(
    row: tuple[Any, ...],
    case: str,
    image_root: Path,
    transform: ProposalTransform,
    seed: int,
    target: Path,
) -> dict[str, Any]:
    (
        image_id,
        file_name,
        _width,
        _height,
        dataset,
        camera_type,
        x1,
        y1,
        x2,
        y2,
        state,
        category,
    ) = row
    box = torch.tensor([[x1, y1, x2, y2]], dtype=torch.float32)
    empty = box.new_empty((0, 4))
    inputs = {0: (box, empty, empty), 1: (empty, box, empty), 2: (empty, empty, box)}
    with Image.open(image_root / str(file_name)) as loaded:
        source = loaded.convert("RGB")
    result = transform(source, *inputs[int(state)], seed=seed)
    retained = result[int(state) + 1]
    color = STATE_COLORS[int(state)]
    before = _source_panel(source, (x1, y1, x2, y2), color)
    after = _tensor_image(result[0])
    if retained.numel():
        ImageDraw.Draw(after).rectangle(retained[0].tolist(), outline=color, width=3)
    combined = Image.new("RGB", (768, 384))
    combined.paste(before, (0, 0))
    combined.paste(after, (384, 0))
    combined.save(target, quality=92)
    parameters = sample_augmentation_parameters(
        transform.augmentation, transform.input_size, training=True, seed=seed
    )
    return {
        "case": case,
        "dataset": dataset,
        "camera_type": camera_type,
        "image_id": image_id,
        "category": category,
        "state": STATE_NAMES[int(state)],
        "retained": bool(retained.numel()),
        "operations": parameters.selected_operations,
        "parameters": asdict(parameters),
        "asset": target.name,
    }


def build_audit(
    config_path: Path,
    index_path: Path,
    output: Path,
    *,
    counter_samples: int,
) -> dict[str, Any]:
    """Generate audit assets, operation counters, and summary JSON."""
    config = load_phase3_config(config_path)
    transform = ProposalTransform(
        config.data.input_size,
        config.augmentation,
        training=True,
        tiny_area=config.data.tiny_area,
        tiny_min_side=config.data.tiny_min_side,
    )
    output.mkdir(parents=True, exist_ok=True)
    assets = output / "assets"
    assets.mkdir(exist_ok=True)
    operation_counts: Counter[str] = Counter()
    for seed in range(counter_samples):
        operation_counts.update(
            sample_augmentation_parameters(
                config.augmentation,
                config.data.input_size,
                training=True,
                seed=seed,
            ).selected_operations
        )
    uri = f"file:{index_path.resolve()}?mode=ro&immutable=1"
    cards: list[dict[str, Any]] = []
    unavailable: list[dict[str, str]] = []
    with sqlite3.connect(uri, uri=True) as connection:
        for dataset in DATASETS:
            for case, row in select_cases(connection, dataset, config.data.image_root):
                if row is None:
                    unavailable.append({"dataset": dataset, "case": case})
                    continue
                target = assets / f"{len(cards):03d}-{dataset}-{case}.jpg"
                cards.append(
                    _render_case(
                        row,
                        case,
                        config.data.image_root,
                        transform,
                        config.schedule.seed + len(cards),
                        target,
                    )
                )
    report = {
        "schema_version": "augmentation-audit.v1",
        "config": str(config_path.resolve()),
        "index": str(index_path.resolve()),
        "effective_policy": effective_augmentation_policy(config.augmentation),
        "counter_samples": counter_samples,
        "operation_counts": dict(sorted(operation_counts.items())),
        "cards": cards,
        "unavailable": unavailable,
    }
    (output / "summary.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    card_html = "".join(
        f"<article><img src='assets/{escape(card['asset'])}'>"
        f"<h3>{escape(card['dataset'])} · {escape(card['case'])}</h3>"
        f"<p>{escape(card['state'])} · {escape(card['category'])} · "
        f"retained={card['retained']}</p>"
        f"<code>{escape(', '.join(card['operations']))}</code></article>"
        for card in cards
    )
    unavailable_html = (
        "".join(
            f"<li>{escape(item['dataset'])}: {escape(item['case'])}</li>"
            for item in unavailable
        )
        or "<li>none</li>"
    )
    counters = "".join(
        f"<li>{escape(name)}: {count}/{counter_samples} ({count / counter_samples:.1%})</li>"
        for name, count in sorted(operation_counts.items())
    )
    (output / "index.html").write_text(
        "<!doctype html><meta charset='utf-8'><title>Augmentation audit</title>"
        "<style>body{font:14px system-ui;margin:24px;background:#111827;color:#e5e7eb}"
        "main{display:grid;grid-template-columns:repeat(auto-fit,minmax(520px,1fr));gap:18px}"
        "article{background:#1f2937;padding:12px;border-radius:8px}img{width:100%}"
        "code{white-space:normal;color:#bfdbfe}</style>"
        "<h1>Phase 1.7 shared augmentation audit</h1>"
        "<p>Left: source and reviewed target. Right: shared augmented pixels and retained target.</p>"
        f"<h2>Operation counters</h2><ul>{counters}</ul>"
        f"<h2>Unavailable contract cases</h2><ul>{unavailable_html}</ul>"
        f"<main>{card_html}</main>",
        encoding="utf-8",
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--index", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--counter-samples", type=int, default=10_000)
    args = parser.parse_args()
    report = build_audit(
        args.config, args.index, args.output, counter_samples=args.counter_samples
    )
    print(
        json.dumps(
            {"cards": len(report["cards"]), "unavailable": report["unavailable"]}
        )
    )


if __name__ == "__main__":
    main()
