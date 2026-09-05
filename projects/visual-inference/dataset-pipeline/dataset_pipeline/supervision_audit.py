"""Deterministic visual audit for generated WoodScape supervision states."""

from __future__ import annotations

import hashlib
import html
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw

from .reports import read_json, write_json
from .supervisely_filter import iter_project_images


AUDIT_CATEGORIES = frozenset(
    {
        "construction",
        "ego_vehicle",
        "grouped_animals",
        "grouped_pedestrian_and_animals",
        "grouped_vehicles",
    }
)
STATE_COLORS = {
    "positive": "#22c55e",
    "ignore": "#f59e0b",
    "trusted_background": "#3b82f6",
}


def _normalized(value: object) -> str:
    return str(value).strip().casefold().replace(" ", "_")


def _points(obj: dict[str, Any]) -> list[tuple[float, float]]:
    values = obj.get("points", {}).get("exterior", [])
    if str(obj.get("geometryType", "")).casefold() == "rectangle" and len(values) >= 2:
        (x1, y1), (x2, y2) = values[:2]
        values = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
    return [(float(point[0]), float(point[1])) for point in values]


def _rank(*parts: object) -> str:
    return hashlib.sha256("\0".join(map(str, parts)).encode()).hexdigest()


def _render(
    image_path: Path,
    objects: list[dict[str, Any]],
    target_index: int,
    destination: Path,
) -> None:
    with Image.open(image_path) as loaded:
        image = loaded.convert("RGB")
    scale = min(1.0, 640.0 / image.width)
    canvas = image.resize(
        (round(image.width * scale), round(image.height * scale)),
        Image.Resampling.LANCZOS,
    )
    draw = ImageDraw.Draw(canvas)
    for index, obj in enumerate(objects):
        points = [(x * scale, y * scale) for x, y in _points(obj)]
        if len(points) < 3:
            continue
        state = str(obj.get("supervisionState", "positive"))
        color = "#f43f5e" if index == target_index else STATE_COLORS.get(state, "white")
        draw.line(
            points + [points[0]], fill=color, width=5 if index == target_index else 2
        )
    destination.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(destination, quality=88)


def generate_woodscape_supervision_audit(
    project: Path,
    output: Path,
    *,
    count_per_category_state: int = 24,
) -> dict[str, Any]:
    """Render selected generated states with all neighboring supervision."""
    candidates: dict[str, list[dict[str, Any]]] = defaultdict(list)
    totals: Counter[str] = Counter()
    for split, image_path, annotation_path in iter_project_images(project):
        annotation = read_json(annotation_path)
        objects = annotation.get("objects", [])
        for index, obj in enumerate(objects):
            category = _normalized(obj.get("sourceCategory", obj.get("classTitle")))
            if category not in AUDIT_CATEGORIES:
                continue
            state = str(obj.get("supervisionState", "positive"))
            key = f"{category}:{state}"
            totals[key] += 1
            candidates[key].append(
                {
                    "rank": _rank(
                        split, image_path.name, obj.get("sourceAnnotationId", index)
                    ),
                    "split": split,
                    "image_path": image_path,
                    "annotation_path": annotation_path,
                    "object_index": index,
                    "category": category,
                    "state": state,
                    "exclusion_reason": str(obj.get("exclusionReason", "")),
                    "source_annotation_id": str(obj.get("sourceAnnotationId", index)),
                }
            )

    selected = []
    for key in sorted(candidates):
        selected.extend(
            sorted(candidates[key], key=lambda row: row["rank"])[
                :count_per_category_state
            ]
        )
    assets = output / "assets"
    cards = []
    for card_index, row in enumerate(selected):
        annotation = read_json(row["annotation_path"])
        asset_name = f"{card_index:04d}-{row['category']}-{row['state']}.jpg"
        _render(
            row["image_path"],
            annotation.get("objects", []),
            int(row["object_index"]),
            assets / asset_name,
        )
        cards.append(
            "<article><img src='assets/{}'><p><b>{}</b> · {} · {}</p>"
            "<p>{} · {}</p></article>".format(
                html.escape(asset_name),
                html.escape(str(row["category"])),
                html.escape(str(row["state"])),
                html.escape(str(row["split"])),
                html.escape(str(row["source_annotation_id"])),
                html.escape(str(row["exclusion_reason"]) or "no exclusion"),
            )
        )
    output.mkdir(parents=True, exist_ok=True)
    (output / "index.html").write_text(
        "<!doctype html><meta charset='utf-8'><title>WoodScape supervision audit</title>"
        "<style>body{font:14px sans-serif;background:#111;color:#eee}main{display:grid;"
        "grid-template-columns:repeat(auto-fill,minmax(360px,1fr));gap:16px}article{"
        "background:#222;padding:10px}img{width:100%;height:auto}p{margin:.4em 0}</style>"
        "<h1>WoodScape generated supervision audit</h1><p>Green: positive; amber: ignore; "
        "blue: trusted background; red: reviewed target.</p><main>"
        + "".join(cards)
        + "</main>",
        encoding="utf-8",
    )
    report = {
        "schema_version": "woodscape-supervision-audit.v1",
        "source_project": str(project),
        "count_per_category_state": count_per_category_state,
        "available_counts": dict(sorted(totals.items())),
        "rendered_cards": len(cards),
        "index": str(output / "index.html"),
    }
    write_json(output / "summary.json", report)
    return report
