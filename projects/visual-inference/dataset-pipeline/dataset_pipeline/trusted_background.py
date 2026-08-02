"""Derive conservative compact trusted-background geometry from source stuff."""

from __future__ import annotations

from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw

from .config import Config
from .reports import read_json
from .taxonomy import Taxonomy


def _flatten_points(value: Any) -> list[tuple[float, float]]:
    if not isinstance(value, (list, tuple)):
        return []
    if len(value) == 2 and all(isinstance(item, (int, float)) for item in value):
        return [(float(value[0]), float(value[1]))]
    points: list[tuple[float, float]] = []
    for item in value:
        points.extend(_flatten_points(item))
    return points


def _source_annotation(config: Config, image: dict[str, Any]) -> Path | None:
    source_id = Path(str(image["source_image_id"]))
    parts = list(source_id.parts)
    if "img" not in parts:
        return None
    parts[parts.index("img")] = "ann"
    return (
        config.workspace_root
        / "raw"
        / str(image["source_dataset"])
        / Path(*parts)
    ).with_name(f"{source_id.name}.json")


def _draw_quad(draw: ImageDraw.ImageDraw, record: dict[str, Any], value: int) -> None:
    points = [(float(x), float(y)) for x, y in record["quad"]]
    draw.polygon(points, fill=value)


def derive_trusted_background(
    config: Config,
    taxonomy: Taxonomy,
    manifests: dict[str, dict[str, Any]],
    *,
    tile_size: int = 32,
    minimum_occupancy: float = 1.0,
) -> dict[str, Any]:
    """Populate fully-stuff tiles after subtracting positives and ignores.

    A tile is trusted only when every covered source pixel is an allowlisted
    stuff class. Partial edge tiles are allowed, but mixed tiles remain weak.
    """
    configured = {
        dataset: {taxonomy.normalize(name) for name in names}
        for dataset, names in taxonomy.data.get(
            "trusted_background_categories", {}
        ).items()
    }
    counts: Counter[str] = Counter()
    source_categories: Counter[str] = Counter()
    for manifest in manifests.values():
        for image in manifest["images"]:
            allowed = configured.get(str(image["source_dataset"]), set())
            image["trusted_background"] = []
            image["background_supervision"] = False
            if not allowed:
                continue
            path = _source_annotation(config, image)
            if path is None or not path.exists():
                counts["missing_source_annotations"] += 1
                continue
            source = read_json(path)
            width, height = int(image["width"]), int(image["height"])
            mask = Image.new("L", (width, height), color=0)
            draw = ImageDraw.Draw(mask)
            matched = 0
            for obj in source.get("objects", []):
                category = taxonomy.normalize(str(obj.get("classTitle", "")))
                if category not in allowed:
                    continue
                points = _flatten_points(obj.get("points", {}).get("exterior", []))
                if len(points) < 3:
                    counts["unsupported_stuff_geometry"] += 1
                    continue
                draw.polygon(points, fill=255)
                matched += 1
                source_categories[f"{image['source_dataset']}:{category}"] += 1
            if not matched:
                continue
            # Positive and ignore supervision always overrides background.
            for state in ("positive", "ignore"):
                for record in image.get(state, []):
                    _draw_quad(draw, record, 0)
            array = np.asarray(mask, dtype=np.uint8)
            tiles: list[dict[str, Any]] = []
            for y1 in range(0, height, tile_size):
                y2 = min(y1 + tile_size, height)
                for x1 in range(0, width, tile_size):
                    x2 = min(x1 + tile_size, width)
                    occupancy = float((array[y1:y2, x1:x2] > 0).mean())
                    if occupancy + 1e-12 < minimum_occupancy:
                        continue
                    quad = [
                        [float(x1), float(y1)],
                        [float(x2), float(y1)],
                        [float(x2), float(y2)],
                        [float(x1), float(y2)],
                    ]
                    tiles.append({
                        "bbox": [float(x1), float(y1), float(x2 - x1), float(y2 - y1)],
                        "quad": quad,
                        "geometry_tier": "trusted_stuff_tile",
                        "fit_coverage": occupancy,
                        "fit_tightness": occupancy,
                        "state": "trusted_background",
                        "valid": True,
                        "source_annotation_id": "",
                        "source_category": "explicit_stuff",
                        "aliases": [],
                    })
            image["trusted_background"] = tiles
            image["background_supervision"] = bool(tiles)
            counts["images_with_source_stuff"] += 1
            counts["images_with_trusted_tiles"] += bool(tiles)
            counts["trusted_tiles"] += len(tiles)
            counts["trusted_pixels"] += sum(
                int(tile["bbox"][2] * tile["bbox"][3]) for tile in tiles
            )
    return {
        "policy": "fully occupied 32px tiles from explicit stuff after positive/ignore subtraction",
        "tile_size": tile_size,
        "minimum_occupancy": minimum_occupancy,
        "configured_categories": {
            key: sorted(value) for key, value in sorted(configured.items())
        },
        "source_category_instances": dict(sorted(source_categories.items())),
        "counts": dict(sorted(counts.items())),
    }
