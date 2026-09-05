"""Compact, class-agnostic proposal-manifest schema and validation."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from .reports import read_json, write_json
from .taxonomy import Taxonomy


SCHEMA_VERSION = "proposal-manifest.v2"


def _signed_area(points: list[list[float]]) -> float:
    return 0.5 * sum(
        points[index][0] * points[(index + 1) % 4][1]
        - points[index][1] * points[(index + 1) % 4][0]
        for index in range(4)
    )


def _valid_clockwise_quad(points: list[list[float]]) -> bool:
    turns = []
    for index in range(4):
        first, second, third = (
            points[index],
            points[(index + 1) % 4],
            points[(index + 2) % 4],
        )
        turns.append(
            (second[0] - first[0]) * (third[1] - second[1])
            - (second[1] - first[1]) * (third[0] - second[0])
        )
    return _signed_area(points) > 1e-6 and all(turn > 1e-6 for turn in turns)


def _quad(annotation: dict[str, Any]) -> list[list[float]]:
    value = annotation.get("quad")
    if value is None:
        x, y, width, height = map(float, annotation["bbox"])
        value = [[x, y], [x + width, y], [x + width, y + height], [x, y + height]]
    points = [[float(point[0]), float(point[1])] for point in value]
    if len(points) != 4 or any(len(point) != 2 for point in points):
        raise ValueError("manifest annotation quad must contain four points")
    if not all(math.isfinite(coord) for point in points for coord in point):
        raise ValueError("manifest annotation quad contains non-finite coordinates")
    return points


def _record(annotation: dict[str, Any], state: str) -> dict[str, Any]:
    quad = _quad(annotation)
    xs = [point[0] for point in quad]
    ys = [point[1] for point in quad]
    return {
        "bbox": [min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys)],
        "quad": quad,
        "geometry_tier": str(annotation.get("geometry_tier", "source_hbb")),
        "fit_coverage": float(annotation.get("fit_coverage", 1.0)),
        "fit_tightness": float(annotation.get("fit_tightness", 0.0)),
        "state": state,
        "valid": True,
        "source_annotation_id": str(annotation.get("source_annotation_id", "")),
        "source_annotation_identity_kind": str(
            annotation.get("source_annotation_identity_kind", "source_object_id")
        ),
        "source_dataset": str(annotation.get("source_dataset", "unknown")),
        "source_split": str(annotation.get("source_split", "unknown")),
        "source_image_id": str(annotation.get("source_image_id", "")),
        "source_category": str(annotation.get("source_category", "unknown")),
        "original_category": str(
            annotation.get(
                "original_category", annotation.get("source_category", "unknown")
            )
        ),
        "canonical_category": str(annotation.get("canonical_category", "unknown")),
        "original_iscrowd": bool(annotation.get("original_iscrowd", False)),
        "original_group": bool(annotation.get("original_group", False)),
        "geometry_conversion_method": str(
            annotation.get("geometry_conversion_method", "source_hbb")
        ),
        "supervision_state": state,
        "exclusion_reason": str(annotation.get("exclusion_reason", "")),
        "aliases": list(annotation.get("aliases", [])),
        "attributes": dict(annotation.get("attributes") or {}),
    }


def _contained_component_indices(
    records: list[dict[str, Any]], taxonomy: Taxonomy
) -> set[int]:
    policy = (
        taxonomy.data.get("proposal_object_contract", {})
        .get("global_policies", {})
        .get("nested_components", {})
    )
    components = set(policy.get("component_categories", []))
    parents = set(policy.get("parent_categories", []))
    threshold = float(policy.get("containment_threshold", 0.8))
    parent_boxes = [
        record["bbox"]
        for record in records
        if record.get("canonical_category") in parents
        or record.get("source_category") in parents
    ]
    ignored = set()
    for index, record in enumerate(records):
        if (
            record.get("canonical_category") not in components
            and record.get("source_category") not in components
        ):
            continue
        x, y, width, height = map(float, record["bbox"])
        area = max(width * height, 1e-7)
        for px, py, pwidth, pheight in parent_boxes:
            intersection_width = max(0.0, min(x + width, px + pwidth) - max(x, px))
            intersection_height = max(0.0, min(y + height, py + pheight) - max(y, py))
            if intersection_width * intersection_height / area > threshold:
                ignored.add(index)
                break
    return ignored


def build_manifest(
    data: dict[str, Any], split: str, taxonomy: Taxonomy | None = None
) -> dict[str, Any]:
    """Project merged COCO data into the compact loader-facing schema."""
    annotations_by_image: dict[int, list[dict[str, Any]]] = {}
    for annotation in data.get("annotations", []):
        annotations_by_image.setdefault(int(annotation["image_id"]), []).append(
            annotation
        )
    images = []
    for image in data.get("images", []):
        positive: list[dict[str, Any]] = []
        ignore: list[dict[str, Any]] = []
        trusted_background: list[dict[str, Any]] = []
        for annotation in annotations_by_image.get(int(image["id"]), []):
            state = str(
                annotation.get(
                    "supervision_state",
                    "ignore"
                    if bool(
                        annotation.get("ignore_region") or annotation.get("iscrowd")
                    )
                    else "positive",
                )
            )
            if state == "trusted_negative":
                state = "trusted_background"
            if state not in {"positive", "ignore", "trusted_background"}:
                raise ValueError(f"unsupported generated supervision state: {state}")
            if state == "positive" and bool(
                annotation.get("ignore_region") or annotation.get("iscrowd")
            ):
                raise ValueError("positive annotation contradicts ignore/crowd flags")
            destination = {
                "positive": positive,
                "ignore": ignore,
                "trusted_background": trusted_background,
            }[state]
            destination.append(_record(annotation, state))
        if taxonomy is not None:
            component_indices = _contained_component_indices(positive, taxonomy)
            if component_indices:
                retained = []
                for index, record in enumerate(positive):
                    if index not in component_indices:
                        retained.append(record)
                        continue
                    record["state"] = "ignore"
                    record["supervision_state"] = "ignore"
                    record["exclusion_reason"] = "contained_component_of_parent"
                    ignore.append(record)
                positive = retained
        images.append(
            {
                "image_id": int(image["id"]),
                "file_name": str(image["file_name"]),
                "width": int(image["width"]),
                "height": int(image["height"]),
                "source_dataset": str(image.get("source_dataset", "unknown")),
                "source_split": str(image.get("source_split", split)),
                "source_image_id": str(image.get("source_image_id", "")),
                "source_sequence_id": str(image.get("source_sequence_id", "")),
                "camera_type": str(image.get("camera_type", "perspective")),
                "background_supervision": False,
                "positive": positive,
                "ignore": ignore,
                "trusted_background": trusted_background,
            }
        )
    return {
        "schema_version": SCHEMA_VERSION,
        "split": split,
        "object_contract": "bounded_promptable_physical_instance",
        "images": images,
    }


def write_manifests(
    output_dir: Path,
    data_by_split: dict[str, dict[str, Any]],
    taxonomy: Taxonomy | None = None,
) -> list[Path]:
    paths = []
    for split, data in data_by_split.items():
        path = output_dir / "annotations" / f"proposals_{split}.json"
        write_json(path, build_manifest(data, split, taxonomy), compact=True)
        paths.append(path)
    return paths


def validate_manifest(path: Path, image_root: Path) -> dict[str, Any]:
    data = read_json(path)
    errors: list[str] = []
    if data.get("schema_version") != SCHEMA_VERSION:
        errors.append("unexpected schema_version")
    if data.get("split") not in {"train", "val"}:
        errors.append("split must be train or val")
    images = data.get("images")
    if not isinstance(images, list):
        errors.append("images must be a list")
        images = []
    image_ids: set[int] = set()
    annotation_count = 0
    accepted_count = 0
    coverage_failures = 0
    for image in images:
        try:
            image_id = int(image["image_id"])
            image_ids.add(image_id)
            if int(image["width"]) <= 0 or int(image["height"]) <= 0:
                errors.append(f"image {image_id}: invalid dimensions")
            if not (image_root / str(image["file_name"])).exists():
                errors.append(f"image {image_id}: missing image link")
            for state in ("positive", "ignore", "trusted_background"):
                if not isinstance(image.get(state), list):
                    errors.append(f"image {image_id}: {state} must be a list")
            for state, records in (
                ("positive", image.get("positive", [])),
                ("ignore", image.get("ignore", [])),
                ("trusted_background", image.get("trusted_background", [])),
            ):
                for annotation in records:
                    if (
                        annotation.get("state") != state
                        or annotation.get("valid") is not True
                    ):
                        errors.append(f"image {image_id}: invalid {state} record state")
                    annotation_count += 1
                    quad = _quad(annotation)
                    if not _valid_clockwise_quad(quad):
                        errors.append(
                            f"image {image_id}: {state} quad is not valid clockwise convex geometry"
                        )
                    coverage = float(annotation.get("fit_coverage", 0.0))
                    tightness = float(annotation.get("fit_tightness", math.nan))
                    if coverage < 0.98:
                        coverage_failures += 1
                        errors.append(
                            f"image {image_id}: {state} fit coverage is below 0.98"
                        )
                    if not math.isfinite(tightness) or not 0.0 <= tightness <= 1.0:
                        errors.append(
                            f"image {image_id}: {state} fit tightness must be in [0, 1]"
                        )
                    if state == "trusted_background" or annotation.get(
                        "geometry_tier"
                    ) in {
                        "source_quad",
                        "fitted_quad",
                        "source_hbb",
                        "rotated_rect",
                        "hbb_fallback",
                    }:
                        accepted_count += 1
                    xs = [point[0] for point in quad]
                    ys = [point[1] for point in quad]
                    if (
                        min(xs) < 0
                        or min(ys) < 0
                        or max(xs) > int(image["width"])
                        or max(ys) > int(image["height"])
                    ):
                        errors.append(f"image {image_id}: quad outside image")
        except (KeyError, TypeError, ValueError) as exc:
            errors.append(f"invalid image record: {exc}")
    if len(image_ids) != len(images):
        errors.append("duplicate image_id")
    if errors:
        raise ValueError(f"manifest validation failed for {path}: {len(errors)} errors")
    return {
        "schema_version": SCHEMA_VERSION,
        "split": data["split"],
        "images": len(images),
        "annotations": annotation_count,
        "accepted_geometry": accepted_count,
        "fit_coverage_failures": coverage_failures,
    }
