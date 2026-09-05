"""Generate precedence-safe trusted-background polygons from manifest candidates."""

from __future__ import annotations

import hashlib
from collections import Counter
from typing import Any

import numpy as np
from PIL import Image, ImageDraw

from .config import Config
from .taxonomy import Taxonomy


def _draw_quad(draw: ImageDraw.ImageDraw, record: dict[str, Any], value: int) -> None:
    points = [(float(x), float(y)) for x, y in record["quad"]]
    draw.polygon(points, fill=value)


def _record_area(record: dict[str, Any]) -> int:
    points = [(float(x), float(y)) for x, y in record["quad"]]
    return round(
        abs(
            sum(
                points[index][0] * points[(index + 1) % len(points)][1]
                - points[index][1] * points[(index + 1) % len(points)][0]
                for index in range(len(points))
            )
            / 2.0
        )
    )


def derive_trusted_background(
    config: Config,
    taxonomy: Taxonomy,
    manifests: dict[str, dict[str, Any]],
    *,
    tile_size: int = 32,
    minimum_occupancy: float = 1.0,
) -> dict[str, Any]:
    """Tile approved candidates after positive and ignore subtraction."""
    counts: Counter[str] = Counter()
    source_categories: Counter[str] = Counter()
    source_category_area: Counter[str] = Counter()
    configured_categories: dict[str, set[str]] = {}
    contract = taxonomy.data.get("proposal_object_contract", {})
    overrides = contract.get("category_overrides", {})
    approved = contract.get("approved_category_states", {})
    for dataset in config.datasets:
        for category, proposed in overrides.get(dataset, {}).items():
            state = (
                approved.get(dataset, {})
                .get(category, {})
                .get("state", proposed.get("proposed_state"))
            )
            if state != "trusted_negative":
                continue
            normalized = taxonomy.normalize(category)
            configured_categories.setdefault(dataset, set()).add(normalized)
            key = f"{dataset}:{normalized}"
            source_categories[key] += 0
            source_category_area[key] += 0
    for manifest in manifests.values():
        for image in manifest["images"]:
            candidates = list(image.get("trusted_background", []))
            image["trusted_background"] = []
            image["background_supervision"] = False
            if not candidates:
                continue
            dataset = str(image["source_dataset"])
            width, height = int(image["width"]), int(image["height"])
            candidate_mask = Image.new("L", (width, height), color=0)
            candidate_draw = ImageDraw.Draw(candidate_mask)
            for record in candidates:
                _draw_quad(candidate_draw, record, 255)
                category = taxonomy.normalize(
                    str(record.get("source_category", "unknown"))
                )
                key = f"{dataset}:{category}"
                source_categories[key] += 1
                source_category_area[key] += _record_area(record)
                configured_categories.setdefault(dataset, set()).add(category)

            candidate_array = np.asarray(candidate_mask, dtype=np.uint8) > 0
            positive_mask = Image.new("1", (width, height), color=0)
            positive_draw = ImageDraw.Draw(positive_mask)
            for record in image.get("positive", []):
                _draw_quad(positive_draw, record, 1)
            ignore_mask = Image.new("1", (width, height), color=0)
            ignore_draw = ImageDraw.Draw(ignore_mask)
            for record in image.get("ignore", []):
                _draw_quad(ignore_draw, record, 1)
            positive_array = np.asarray(positive_mask, dtype=bool)
            ignore_array = np.asarray(ignore_mask, dtype=bool)
            counts["candidate_positive_overlap_pixels"] += int(
                np.count_nonzero(candidate_array & positive_array)
            )
            counts["candidate_ignore_overlap_pixels"] += int(
                np.count_nonzero(candidate_array & ignore_array)
            )
            trusted = candidate_array & ~positive_array & ~ignore_array

            tiles: list[dict[str, Any]] = []
            for y1 in range(0, height, tile_size):
                y2 = min(y1 + tile_size, height)
                for x1 in range(0, width, tile_size):
                    x2 = min(x1 + tile_size, width)
                    occupancy = float(trusted[y1:y2, x1:x2].mean())
                    if occupancy + 1e-12 < minimum_occupancy:
                        continue
                    quad = [
                        [float(x1), float(y1)],
                        [float(x2), float(y1)],
                        [float(x2), float(y2)],
                        [float(x1), float(y2)],
                    ]
                    identity = hashlib.sha256(
                        f"{dataset}:{image['source_image_id']}:{x1}:{y1}:{x2}:{y2}".encode()
                    ).hexdigest()[:20]
                    tiles.append(
                        {
                            "bbox": [
                                float(x1),
                                float(y1),
                                float(x2 - x1),
                                float(y2 - y1),
                            ],
                            "quad": quad,
                            "geometry_tier": "trusted_candidate_tile",
                            "fit_coverage": occupancy,
                            "fit_tightness": occupancy,
                            "state": "trusted_background",
                            "valid": True,
                            "source_annotation_id": identity,
                            "source_annotation_identity_kind": "generated_trusted_tile",
                            "source_dataset": dataset,
                            "source_split": str(image["source_split"]),
                            "source_image_id": str(image["source_image_id"]),
                            "source_category": "approved_trusted_union",
                            "original_category": "approved_trusted_union",
                            "canonical_category": "approved_trusted_union",
                            "original_iscrowd": False,
                            "original_group": False,
                            "geometry_conversion_method": "precedence_safe_32px_tile",
                            "supervision_state": "trusted_background",
                            "exclusion_reason": "",
                            "aliases": [],
                        }
                    )
            image["trusted_background"] = tiles
            image["background_supervision"] = bool(tiles)
            counts["images_with_trusted_candidates"] += 1
            counts["images_with_trusted_tiles"] += bool(tiles)
            counts["trusted_candidate_regions"] += len(candidates)
            counts["trusted_tiles"] += len(tiles)
            counts["trusted_pixels"] += sum(
                int(tile["bbox"][2] * tile["bbox"][3]) for tile in tiles
            )
    counts["positive_trusted_overlap_pixels_after_precedence"] = 0
    counts["ignore_trusted_overlap_pixels_after_precedence"] = 0
    return {
        "policy": "positive > ignore > trusted background > weak; fully occupied 32px tiles",
        "tile_size": tile_size,
        "minimum_occupancy": minimum_occupancy,
        "configured_categories": {
            key: sorted(value) for key, value in sorted(configured_categories.items())
        },
        "source_category_instances": dict(sorted(source_categories.items())),
        "source_category_area_pixels": dict(sorted(source_category_area.items())),
        "counts": dict(sorted(counts.items())),
    }
