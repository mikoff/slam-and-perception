#!/usr/bin/env python3
"""Classify Phase 1.8 overlap failures without rehashing image bytes."""

from __future__ import annotations

import argparse
import hashlib
import html
import json
import math
import time
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterator

import ijson
from PIL import Image, ImageDraw
from shapely import Polygon


STATE_PAIRS = (
    ("positive", "ignore"),
    ("positive", "trusted_background"),
    ("ignore", "trusted_background"),
)
AREA_EDGES = (0.0, 1e-6, 1e-3, 0.1, 1.0, 16.0, 256.0, math.inf)
GRID_SIZE = 64.0


def sha256_file(path: Path) -> str:
    """Return a streaming SHA-256 digest."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _category(record: dict[str, Any]) -> str:
    return str(
        record.get("canonical_category") or record.get("source_category") or "unknown"
    )


def _bounds(record: dict[str, Any]) -> tuple[float, float, float, float]:
    points = record["quad"]
    xs = [float(point[0]) for point in points]
    ys = [float(point[1]) for point in points]
    return min(xs), min(ys), max(xs), max(ys)


def _cells(bounds: tuple[float, float, float, float]) -> Iterator[tuple[int, int]]:
    x1, y1, x2, y2 = bounds
    last_x = math.floor(math.nextafter(x2, -math.inf) / GRID_SIZE)
    last_y = math.floor(math.nextafter(y2, -math.inf) / GRID_SIZE)
    for grid_y in range(math.floor(y1 / GRID_SIZE), last_y + 1):
        for grid_x in range(math.floor(x1 / GRID_SIZE), last_x + 1):
            yield grid_x, grid_y


def _prepare(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        {"record": record, "bounds": _bounds(record), "category": _category(record)}
        for record in records
    ]


def _grid(records: list[dict[str, Any]]) -> dict[tuple[int, int], list[int]]:
    result: dict[tuple[int, int], list[int]] = defaultdict(list)
    for index, item in enumerate(records):
        for cell in _cells(item["bounds"]):
            result[cell].append(index)
    return result


def _bbox_intersects(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> bool:
    return not (
        left[2] <= right[0]
        or right[2] <= left[0]
        or left[3] <= right[1]
        or right[3] <= left[1]
    )


def _overlap_records(
    left: list[dict[str, Any]],
    right: list[dict[str, Any]],
    right_grid: dict[tuple[int, int], list[int]],
) -> Iterator[tuple[dict[str, Any], dict[str, Any], float]]:
    """Yield exact positive-area intersections after a cheap spatial prefilter."""
    right_polygons: dict[int, Polygon] = {}
    for left_item in left:
        candidates = {
            index
            for cell in _cells(left_item["bounds"])
            for index in right_grid.get(cell, [])
        }
        left_polygon: Polygon | None = None
        for index in candidates:
            right_item = right[index]
            if not _bbox_intersects(left_item["bounds"], right_item["bounds"]):
                continue
            if left_polygon is None:
                left_polygon = Polygon(left_item["record"]["quad"])
            right_polygon = right_polygons.setdefault(
                index, Polygon(right_item["record"]["quad"])
            )
            area = left_polygon.intersection(right_polygon).area
            if area > 1e-7:
                yield left_item, right_item, area


def _area_bin(area: float) -> str:
    for lower, upper in zip(AREA_EDGES, AREA_EDGES[1:]):
        if lower < area <= upper:
            return f"({lower:g},{upper:g}]"
    return "invalid"


def _new_aggregate() -> dict[str, Any]:
    return {
        "images": 0,
        "polygon_pairs": 0,
        "pairwise_intersection_area_px2": 0.0,
        "area_bins": Counter(),
        "source_pairs": defaultdict(Counter),
        "category_pairs": defaultdict(Counter),
        "representative_examples": {},
    }


def _record_pair(
    aggregate: dict[str, Any],
    *,
    split: str,
    image: dict[str, Any],
    left_state: str,
    right_state: str,
    left: dict[str, Any],
    right: dict[str, Any],
    area: float,
) -> tuple[str, str]:
    dataset = str(image.get("source_dataset", "unknown"))
    category_key = f"{dataset}/{left['category']}/{right['category']}"
    aggregate["polygon_pairs"] += 1
    aggregate["pairwise_intersection_area_px2"] += area
    aggregate["area_bins"][_area_bin(area)] += 1
    aggregate["source_pairs"][dataset]["polygon_pairs"] += 1
    aggregate["source_pairs"][dataset]["pairwise_intersection_area_px2"] += area
    aggregate["category_pairs"][category_key]["polygon_pairs"] += 1
    aggregate["category_pairs"][category_key]["pairwise_intersection_area_px2"] += area
    example = {
        "split": split,
        "dataset": dataset,
        "source_image_id": str(image.get("source_image_id", "")),
        "file_name": str(image["file_name"]),
        "left_state": left_state,
        "left_category": left["category"],
        "left_source_annotation_id": str(
            left["record"].get("source_annotation_id", "")
        ),
        "left_quad": left["record"]["quad"],
        "right_state": right_state,
        "right_category": right["category"],
        "right_source_annotation_id": str(
            right["record"].get("source_annotation_id", "")
        ),
        "right_quad": right["record"]["quad"],
        "intersection_area_px2": area,
    }
    previous = aggregate["representative_examples"].get(category_key)
    if previous is None or area > previous["intersection_area_px2"]:
        aggregate["representative_examples"][category_key] = example
    return dataset, category_key


def scan_manifest(path: Path, split: str) -> dict[str, Any]:
    """Aggregate exact cross-state intersections from one manifest."""
    aggregates = {f"{left}/{right}": _new_aggregate() for left, right in STATE_PAIRS}
    scanned = 0
    started = time.monotonic()
    with path.open("rb") as stream:
        for image in ijson.items(stream, "images.item", use_float=True):
            scanned += 1
            states = {
                state: _prepare(image.get(state, []))
                for state in ("positive", "ignore", "trusted_background")
            }
            grids = {
                state: _grid(states[state])
                for state in ("ignore", "trusted_background")
                if states[state]
            }
            for left_state, right_state in STATE_PAIRS:
                left, right = states[left_state], states[right_state]
                if not left or not right:
                    continue
                aggregate = aggregates[f"{left_state}/{right_state}"]
                image_sources: set[str] = set()
                image_categories: set[str] = set()
                for left_item, right_item, area in _overlap_records(
                    left, right, grids[right_state]
                ):
                    source, category = _record_pair(
                        aggregate,
                        split=split,
                        image=image,
                        left_state=left_state,
                        right_state=right_state,
                        left=left_item,
                        right=right_item,
                        area=area,
                    )
                    image_sources.add(source)
                    image_categories.add(category)
                if image_categories:
                    aggregate["images"] += 1
                    for source in image_sources:
                        aggregate["source_pairs"][source]["images"] += 1
                    for category in image_categories:
                        aggregate["category_pairs"][category]["images"] += 1
            if scanned % 5_000 == 0:
                elapsed = time.monotonic() - started
                print(
                    f"DIAGNOSE {split}: {scanned:,} images in {elapsed:.1f}s",
                    flush=True,
                )
    return {
        "images_scanned": scanned,
        "state_pairs": {
            key: _jsonable_aggregate(value) for key, value in aggregates.items()
        },
    }


def _jsonable_aggregate(aggregate: dict[str, Any]) -> dict[str, Any]:
    examples = sorted(
        aggregate["representative_examples"].values(),
        key=lambda item: item["intersection_area_px2"],
        reverse=True,
    )
    return {
        "images": aggregate["images"],
        "polygon_pairs": aggregate["polygon_pairs"],
        "pairwise_intersection_area_px2": aggregate["pairwise_intersection_area_px2"],
        "area_bins": dict(sorted(aggregate["area_bins"].items())),
        "by_source": {
            key: dict(value) for key, value in sorted(aggregate["source_pairs"].items())
        },
        "by_category_pair": {
            key: dict(value)
            for key, value in sorted(
                aggregate["category_pairs"].items(),
                key=lambda item: item[1]["pairwise_intersection_area_px2"],
                reverse=True,
            )
        },
        "representative_examples": examples,
    }


def classify_duplicates(audit: dict[str, Any]) -> dict[str, Any]:
    """Group already hash-proven duplicate records without reading images again."""
    result: dict[str, Any] = {}
    for split, scan in audit["full_scan"].items():
        records = scan["duplicate_images_within_split"]
        groups: dict[str, set[str]] = defaultdict(set)
        sources: Counter[str] = Counter()
        for record in records:
            groups[record["sha256"]].update((record["first"], record["second"]))
        for paths in groups.values():
            source = Path(sorted(paths)[0]).name.split("__", 1)[0]
            sources[source] += 1
        result[split] = {
            "reported_duplicate_links": len(records),
            "unique_content_hash_groups": len(groups),
            "unique_redundant_records": sum(
                len(paths) - 1 for paths in groups.values()
            ),
            "groups_by_source": dict(sorted(sources.items())),
            "groups": [
                {"sha256": digest, "paths": sorted(paths)}
                for digest, paths in sorted(groups.items())
            ],
        }
    return result


def _render_example(image_root: Path, output: Path, example: dict[str, Any]) -> None:
    image_path = image_root / example["file_name"]
    with Image.open(image_path).convert("RGB") as source:
        original_width, original_height = source.size
        source.thumbnail((900, 900))
        scale_x, scale_y = (
            source.width / original_width,
            source.height / original_height,
        )
        draw = ImageDraw.Draw(source, "RGBA")
        left = [
            (float(x) * scale_x, float(y) * scale_y) for x, y in example["left_quad"]
        ]
        right = [
            (float(x) * scale_x, float(y) * scale_y) for x, y in example["right_quad"]
        ]
        draw.polygon(left, fill=(255, 0, 0, 55), outline=(255, 0, 0, 255), width=3)
        draw.polygon(right, fill=(0, 80, 255, 55), outline=(0, 80, 255, 255), width=3)
        source.save(output, quality=90)


def render_bundle(output: Path, image_root: Path, report: dict[str, Any]) -> None:
    """Write a compact HTML bundle and distinct-category overlap examples."""
    assets = output / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    rows = []
    figures = []
    example_number = 0
    for split, scan in report["overlaps"].items():
        for state_pair, aggregate in scan["state_pairs"].items():
            rows.append(
                f"<tr><td>{html.escape(split)}</td><td>{html.escape(state_pair)}</td>"
                f"<td>{aggregate['images']:,}</td><td>{aggregate['polygon_pairs']:,}</td>"
                f"<td>{aggregate['pairwise_intersection_area_px2']:,.6f}</td></tr>"
            )
            for example in aggregate["representative_examples"][:60]:
                name = f"overlap_{example_number:04d}.jpg"
                example_number += 1
                try:
                    _render_example(image_root, assets / name, example)
                except Exception as error:
                    example["asset_error"] = str(error)
                    continue
                example["asset"] = f"assets/{name}"
                label = (
                    f"{split} · {example['dataset']} · {example['left_category']} / "
                    f"{example['right_category']} · {example['intersection_area_px2']:.6f} px²"
                )
                figures.append(
                    f"<figure><img loading='lazy' src='assets/{name}'>"
                    f"<figcaption>{html.escape(label)}</figcaption></figure>"
                )
    (output / "diagnostic_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (output / "index.html").write_text(
        "<!doctype html><meta charset='utf-8'><title>Phase 1.8 diagnostics</title>"
        "<style>body{font:15px system-ui;max-width:1300px;margin:auto;padding:24px}"
        "table{border-collapse:collapse}td,th{padding:7px;border:1px solid #bbb}"
        ".gallery{display:grid;grid-template-columns:repeat(auto-fill,minmax(280px,1fr));gap:16px}"
        "figure{margin:0}img{max-width:100%}figcaption{overflow-wrap:anywhere}</style>"
        "<h1>Phase 1.8 failure diagnostics</h1>"
        "<p>Red is the left state; blue is the right state. Pairwise areas may double-count "
        "where multiple polygons overlap the same pixels.</p>"
        "<table><tr><th>Split</th><th>State pair</th><th>Images</th>"
        f"<th>Polygon pairs</th><th>Pairwise area px²</th></tr>{''.join(rows)}</table>"
        f"<h2>Representative category pairs</h2><div class='gallery'>{''.join(figures)}</div>",
        encoding="utf-8",
    )


def run(workspace: Path, audit_path: Path, output: Path) -> dict[str, Any]:
    audit = json.loads(audit_path.read_text())
    overlaps = {
        split: scan_manifest(
            workspace / f"output/annotations/proposals_{split}.json", split
        )
        for split in ("train", "val")
    }
    report = {
        "schema_version": "production-audit-diagnostics.v1",
        "candidate": workspace.name,
        "source_audit_report": {
            "path": str(audit_path.resolve()),
            "sha256": sha256_file(audit_path),
            "status": audit["status"],
        },
        "overlaps": overlaps,
        "within_split_duplicates": classify_duplicates(audit),
        "notes": [
            "Pairwise category areas can exceed the audit's state-union overlap area.",
            "No image bytes are rehashed; duplicate classification uses audit-proven SHA-256 groups.",
            "Loader precedence is assessed separately from raw polygon intersection.",
        ],
    }
    render_bundle(output, workspace / "output", report)
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--audit-report", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    audit_path = (
        args.audit_report.resolve()
        if args.audit_report
        else workspace / "reports/phase_1_8_production_audit/audit_report.json"
    )
    report = run(workspace, audit_path, args.output.resolve())
    print(
        json.dumps(
            {
                "candidate": report["candidate"],
                "output": str(args.output.resolve()),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
