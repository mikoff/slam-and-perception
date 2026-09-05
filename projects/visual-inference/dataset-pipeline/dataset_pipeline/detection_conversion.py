from __future__ import annotations

import math
import shutil
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from shapely.geometry import Polygon
from shapely.validation import explain_validity

from .config import Config
from .links import link_image
from .parallel import batches, process_map
from .progress import Progress
from .reports import read_json, write_csv, write_json
from .supervisely_filter import iter_project_images


class GeometryError(ValueError):
    pass


def _flatten_points(value: Any) -> list[tuple[float, float]]:
    result: list[tuple[float, float]] = []
    if isinstance(value, (list, tuple)):
        if len(value) == 2 and all(isinstance(item, (int, float)) for item in value):
            result.append((float(value[0]), float(value[1])))
        else:
            for item in value:
                result.extend(_flatten_points(item))
    return result


def object_bbox(obj: dict[str, Any]) -> tuple[float, float, float, float]:
    geometry = str(obj.get("geometryType", "")).lower()
    if geometry in {"rectangle", "polygon", "polyline", "multipolygon"}:
        value = (
            obj.get("segments", [])
            if geometry == "multipolygon"
            else obj.get("points", {}).get("exterior", [])
        )
        points = _flatten_points(value)
        if not points:
            raise GeometryError(f"{geometry} has no points")
        xs, ys = zip(*points)
        return min(xs), min(ys), max(xs), max(ys)
    if geometry in {"bitmap", "mask"}:
        try:
            import supervisely as sly

            bitmap = sly.Bitmap.from_json(obj)
            rectangle = bitmap.to_bbox()
            points = _flatten_points(
                rectangle.to_json().get("points", {}).get("exterior", [])
            )
            if not points:
                raise GeometryError("empty mask")
            xs, ys = zip(*points)
            return min(xs), min(ys), max(xs), max(ys)
        except GeometryError:
            raise
        except Exception as exc:
            raise GeometryError(f"invalid or empty bitmap: {exc}") from exc
    raise GeometryError(f"unsupported geometry: {geometry or 'missing'}")


def _source_points(obj: dict[str, Any]) -> list[tuple[float, float]]:
    geometry = str(obj.get("geometryType", "")).lower()
    if geometry in {"rectangle", "polygon", "polyline", "multipolygon"}:
        value = (
            obj.get("segments", [])
            if geometry == "multipolygon"
            else obj.get("points", {}).get("exterior", [])
        )
        return _flatten_points(value)
    if geometry in {"bitmap", "mask"}:
        try:
            import numpy as np
            import supervisely as sly

            bitmap = sly.Bitmap.from_json(obj)
            ys, xs = np.nonzero(bitmap.data)
            if not len(xs):
                return []
            return [(float(x), float(y)) for x, y in zip(xs, ys, strict=True)]
        except Exception as exc:
            raise GeometryError(f"invalid or empty bitmap: {exc}") from exc
    raise GeometryError(f"unsupported geometry: {geometry or 'missing'}")


def _source_geometry_issues(obj: dict[str, Any], width: int, height: int) -> list[str]:
    geometry = str(obj.get("geometryType", "")).lower()
    if geometry not in {"polygon", "polyline", "multipolygon"}:
        return []
    raw_rings = (
        obj.get("segments", [])
        if geometry == "multipolygon"
        else [obj.get("points", {}).get("exterior", [])]
    )
    issues = set()
    for raw in raw_rings:
        points = _flatten_points(raw)
        if not points:
            issues.add("empty_geometry")
            continue
        if len(set(points)) < len(points):
            issues.add("duplicate_vertices")
        has_nonfinite_or_out_of_frame = any(
            not math.isfinite(x)
            or not math.isfinite(y)
            or x < 0
            or y < 0
            or x > width - 1
            or y > height - 1
            for x, y in points
        )
        if has_nonfinite_or_out_of_frame:
            issues.add("out_of_frame_coordinates")
        if any(not math.isfinite(value) for point in points for value in point):
            continue
        if geometry == "polyline":
            continue
        if len(points) < 3:
            issues.add("zero_area")
            continue
        try:
            polygon = Polygon(points)
            if not polygon.is_valid:
                reason = explain_validity(polygon).casefold()
                issues.add(
                    "self_intersection"
                    if "self-intersection" in reason
                    else "invalid_polygon"
                )
        except Exception:
            issues.add("invalid_polygon")
        if abs(_signed_area(points)) <= 1e-6:
            issues.add("zero_area")
    return sorted(issues)


def _signed_area(points: list[tuple[float, float]]) -> float:
    return 0.5 * sum(
        points[index][0] * points[(index + 1) % len(points)][1]
        - points[index][1] * points[(index + 1) % len(points)][0]
        for index in range(len(points))
    )


def _valid_source_quad(points: list[tuple[float, float]]) -> bool:
    if len(points) != 4 or abs(_signed_area(points)) < 1e-6:
        return False
    turns = []
    for index in range(4):
        first = points[index]
        second = points[(index + 1) % 4]
        third = points[(index + 2) % 4]
        turns.append(
            (second[0] - first[0]) * (third[1] - second[1])
            - (second[1] - first[1]) * (third[0] - second[0])
        )
    return all(turn > 1e-6 for turn in turns) or all(turn < -1e-6 for turn in turns)


def _canonical_quad(
    points: list[tuple[float, float]] | list[list[float]],
) -> list[list[float]]:
    if len(points) != 4:
        raise GeometryError("quad must contain four points")
    center_x = sum(float(point[0]) for point in points) / 4.0
    center_y = sum(float(point[1]) for point in points) / 4.0
    ordered = sorted(
        ((float(point[0]), float(point[1])) for point in points),
        key=lambda point: math.atan2(point[1] - center_y, point[0] - center_x),
    )
    if _signed_area(ordered) < 0:
        ordered.reverse()
    start = min(range(4), key=lambda index: (ordered[index][1], ordered[index][0]))
    ordered = ordered[start:] + ordered[:start]
    if not _valid_source_quad(ordered):
        raise GeometryError("quad is concave or degenerate")
    return [[x, y] for x, y in ordered]


def _point_coverage(
    points: list[tuple[float, float]], quad: list[list[float]]
) -> float:
    inside = 0
    for x, y in points:
        signed_distances = []
        for index in range(4):
            x1, y1 = quad[index]
            x2, y2 = quad[(index + 1) % 4]
            edge_length = math.hypot(x2 - x1, y2 - y1)
            cross = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)
            signed_distances.append(cross / max(edge_length, 1e-7))
        # cv2's float32 min-area rectangle can place a source boundary point a
        # small fraction of a pixel outside. This is raster-equivalent and must
        # not trigger an HBB fallback.
        inside += all(distance >= -0.25 for distance in signed_distances) or all(
            distance <= 0.25 for distance in signed_distances
        )
    return inside / max(len(points), 1)


def _source_area(
    source_geometry: str,
    points: list[tuple[float, float]],
    declared_area: float | None = None,
) -> float:
    if (
        declared_area is not None
        and math.isfinite(declared_area)
        and declared_area >= 0
    ):
        return declared_area
    if source_geometry in {"bitmap", "mask"}:
        return float(len(points))
    if source_geometry == "polygon" and len(points) >= 3:
        return abs(_signed_area(points))
    x1, y1, x2, y2 = _quad_bbox([[x, y] for x, y in points])
    return abs((x2 - x1) * (y2 - y1))


def _fit_quad(points: list[tuple[float, float]]) -> list[list[float]]:
    if len(points) < 3:
        raise GeometryError("at least three source points are required")
    try:
        import cv2
        import numpy as np

        cloud = np.asarray(points, dtype=np.float32)
        rectangle = cv2.minAreaRect(cloud)
        corners = cv2.boxPoints(rectangle)
        return [[float(x), float(y)] for x, y in corners]
    except Exception as exc:
        raise GeometryError(f"could not fit a containing quad: {exc}") from exc


def _quad_bbox(quad: list[list[float]]) -> tuple[float, float, float, float]:
    xs = [point[0] for point in quad]
    ys = [point[1] for point in quad]
    return min(xs), min(ys), max(xs), max(ys)


def quad_object(
    obj: dict[str, Any],
    width: int,
    height: int,
    clip: bool,
) -> tuple[dict[str, Any], bool]:
    """Preserve a valid source quad or fit a containing rotated rectangle."""
    source_geometry = str(obj.get("geometryType", "")).lower()
    repair_reasons = _source_geometry_issues(obj, width, height)
    points = _source_points(obj)
    if not points:
        raise GeometryError("source geometry has no points")
    changed = False
    clipped_points = []
    for x, y in points:
        if x < 0 or y < 0 or x > width - 1 or y > height - 1:
            changed = True
        clipped_points.append(
            (
                min(max(0.0, x), float(width - 1)),
                min(max(0.0, y), float(height - 1)),
            )
        )
    if changed and not clip:
        raise GeometryError("geometry lies partly outside image")
    points = clipped_points
    if source_geometry == "rectangle" or len(points) < 3:
        x1, y1, x2, y2 = _quad_bbox([[x, y] for x, y in points])
        quad = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
        tier = "source_hbb" if source_geometry == "rectangle" else "hbb_fallback"
    elif len(points) == 4:
        # The source tier describes the source traversal, not whether its four
        # vertices can be repaired by angular sorting. A bow-tie, concave, or
        # degenerate four-point traversal must therefore be fitted first.
        if _valid_source_quad(points):
            quad = _canonical_quad(points)
            tier = "source_quad"
        else:
            quad = _fit_quad(points)
            tier = "rotated_rect"
            repair_reasons.append("invalid_source_quad_fitted")
    else:
        quad = _fit_quad(points)
        tier = "fitted_quad"
    try:
        quad = _canonical_quad(quad)
    except GeometryError:
        x1, y1, x2, y2 = _quad_bbox([[x, y] for x, y in points])
        quad = _canonical_quad([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
        tier = "hbb_fallback"
        repair_reasons.append("invalid_fitted_quad_hbb_fallback")
    # A minimum-area rectangle can extend beyond the image even when every
    # source point is in-bounds (the fitted corners lie outside a clipped
    # object near an image edge).  Preserve the containment contract by using
    # an axis-aligned fallback in that case; this is preferable to silently
    # emitting an invalid quad or coordinate-wise clamping its topology.
    if any(x < 0.0 or y < 0.0 or x > width - 1 or y > height - 1 for x, y in quad):
        x1, y1, x2, y2 = _quad_bbox([[x, y] for x, y in points])
        quad = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
        quad = _canonical_quad(quad)
        tier = "hbb_fallback"
        repair_reasons.append("fitted_quad_out_of_frame_hbb_fallback")
    fit_coverage = _point_coverage(points, quad)
    if fit_coverage < 0.98:
        x1, y1, x2, y2 = _quad_bbox([[x, y] for x, y in points])
        quad = _canonical_quad([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
        fit_coverage = _point_coverage(points, quad)
        tier = "hbb_fallback"
        repair_reasons.append("fit_coverage_hbb_fallback")
    quad_area = abs(_signed_area([(point[0], point[1]) for point in quad]))
    output = dict(obj)
    output["geometryType"] = "rectangle" if tier == "source_hbb" else "polygon"
    output["points"] = {"exterior": quad, "interior": []}
    output["quad"] = quad
    output["geometryTier"] = tier
    output["fitCoverage"] = fit_coverage
    # Occupancy ratio: 1.0 is tight; lower values include more background.
    declared_area = obj.get("sourceArea")
    output["fitTightness"] = min(
        _source_area(
            source_geometry,
            points,
            float(declared_area) if declared_area is not None else None,
        )
        / max(quad_area, 1e-7),
        1.0,
    )
    output["sourceGeometryType"] = source_geometry
    output["geometryConversionMethod"] = {
        "source_quad": "source_quad",
        "source_hbb": "source_hbb",
        "fitted_quad": "fitted_rectangle",
        "rotated_rect": "fitted_rectangle",
        "hbb_fallback": "fallback_hbb",
    }[tier]
    output["geometryRepairReasons"] = sorted(set(repair_reasons))
    state = str(
        output.get(
            "supervisionState",
            "ignore" if bool(output.get("ignoreRegion")) else "positive",
        )
    )
    if state == "trusted_negative":
        state = "trusted_background"
    if state not in {"positive", "ignore", "trusted_background"}:
        raise GeometryError(f"unsupported supervision state: {state}")
    output["supervisionState"] = state
    output.pop("bitmap", None)
    output.pop("segments", None)
    return output, changed


def clip_bbox(
    bbox: tuple[float, float, float, float],
    width: int,
    height: int,
    clip: bool = True,
) -> tuple[tuple[float, float, float, float], bool]:
    x1, y1, x2, y2 = bbox
    if not all(math.isfinite(v) for v in bbox):
        raise GeometryError("non-finite coordinates")
    if width <= 0 or height <= 0:
        raise GeometryError("invalid image dimensions")
    if x2 < 0 or y2 < 0 or x1 > width - 1 or y1 > height - 1:
        raise GeometryError("box is entirely outside image")
    clipped = (
        max(0.0, x1),
        max(0.0, y1),
        min(float(width - 1), x2),
        min(float(height - 1), y2),
    )
    changed = clipped != bbox
    if changed and not clip:
        raise GeometryError("box lies partly outside image")
    x1, y1, x2, y2 = clipped if clip else bbox
    if x2 < x1 or y2 < y1:
        raise GeometryError("box has non-positive dimensions")
    return (x1, y1, x2, y2), changed


def rectangle_object(
    obj: dict[str, Any], width: int, height: int, clip: bool
) -> tuple[dict[str, Any], bool]:
    bbox, changed = clip_bbox(object_bbox(obj), width, height, clip)
    x1, y1, x2, y2 = bbox
    output = dict(obj)
    output["geometryType"] = "rectangle"
    output["points"] = {"exterior": [[x1, y1], [x2, y2]], "interior": []}
    output.pop("bitmap", None)
    return output, changed


def _degenerate_ignore_object(
    obj: dict[str, Any], width: int, height: int
) -> dict[str, Any]:
    """Localize degenerate source geometry as ignore, never as a positive."""
    x1, y1, x2, y2 = object_bbox(obj)
    x1, y1 = max(0.0, x1), max(0.0, y1)
    x2, y2 = min(float(width - 1), x2), min(float(height - 1), y2)
    if x2 - x1 < 1.0:
        center = (x1 + x2) * 0.5
        x1, x2 = max(0.0, center - 0.5), min(float(width - 1), center + 0.5)
    if y2 - y1 < 1.0:
        center = (y1 + y2) * 0.5
        y1, y2 = max(0.0, center - 0.5), min(float(height - 1), center + 0.5)
    quad = _canonical_quad([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
    output = dict(obj)
    output.update(
        {
            "geometryType": "polygon",
            "points": {"exterior": quad, "interior": []},
            "quad": quad,
            "geometryTier": "hbb_fallback",
            "fitCoverage": 1.0,
            "fitTightness": 0.0,
            "sourceGeometryType": str(obj.get("geometryType", "")).lower(),
            "ignoreRegion": True,
            "geometryConversionMethod": "invalid_geometry_localized_ignore_hbb",
            "supervisionState": "ignore",
            "exclusionReason": "invalid_geometry_localized_as_ignore",
        }
    )
    output.pop("bitmap", None)
    output.pop("trustedBackgroundRegion", None)
    return output


def deduplicate_geometry_representations(
    objects: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Resolve repeated representations only when source identity agrees.

    Coincident geometry with different source identities is intentionally
    retained. Official COCO multi-polygons arrive pre-merged as one object.
    """
    groups: dict[tuple[str, str], list[int]] = {}
    for index, obj in enumerate(objects):
        source_id = str(
            obj.get("sourceAnnotationId") or obj.get("id") or obj.get("key") or ""
        )
        if not source_id:
            continue
        key = (str(obj.get("classTitle", "")), source_id)
        groups.setdefault(key, []).append(index)

    discarded: set[int] = set()
    removals: list[dict[str, Any]] = []
    rank = {"multipolygon": 0, "polygon": 1, "bitmap": 2, "mask": 2, "rectangle": 3}
    for (category, source_id), indices in sorted(groups.items()):
        if len(indices) < 2:
            continue
        ordered = sorted(
            indices,
            key=lambda index: (
                rank.get(str(objects[index].get("geometryType", "")).lower(), 4),
                index,
            ),
        )
        retained = ordered[0]
        for index in ordered[1:]:
            discarded.add(index)
            same = objects[index] == objects[retained]
            removals.append(
                {
                    "source_annotation_id": source_id,
                    "source_category": category,
                    "removed_object_id": str(objects[index].get("id", index)),
                    "retained_object_id": str(objects[retained].get("id", retained)),
                    "reason": (
                        "exact_duplicate_same_source_identity"
                        if same
                        else "superseded_representation_same_source_identity"
                    ),
                }
            )
    return (
        [obj for index, obj in enumerate(objects) if index not in discarded],
        removals,
    )


@dataclass(frozen=True)
class _ConversionBatch:
    dataset: str
    images: list[tuple[str, Path, Path]]
    destination: Path
    clip: bool


def _convert_batch(
    batch: _ConversionBatch,
) -> tuple[
    int,
    int,
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[dict[str, Any]],
    dict[str, dict[str, int]],
]:
    converted_count = 0
    removals: list[dict[str, Any]] = []
    invalid = []
    clipped_rows = []
    repairs = []
    category_tiers: dict[str, Counter[str]] = defaultdict(Counter)
    for split, image, ann_path in batch.images:
        link_image(
            image.resolve(),
            batch.destination / split / "img" / image.name,
            "symlink",
            True,
        )
        annotation = read_json(ann_path)
        size = annotation.get("size", {})
        width, height = int(size.get("width", 0)), int(size.get("height", 0))
        if not width or not height:
            from PIL import Image

            with Image.open(image) as loaded:
                width, height = loaded.size
            annotation["size"] = {"width": width, "height": height}
        objects = []
        source_objects, object_removals = deduplicate_geometry_representations(
            annotation.get("objects", [])
        )
        removals.extend(
            {
                "dataset": batch.dataset,
                "split": split,
                "image": image.name,
                **removal,
            }
            for removal in object_removals
        )
        for index, obj in enumerate(source_objects):
            source_category = str(
                obj.get("sourceCategory", obj.get("classTitle", "unknown"))
            )
            row = {
                "dataset": batch.dataset,
                "split": split,
                "image": image.name,
                "object_index": index,
                "source_annotation_id": str(
                    obj.get("sourceAnnotationId") or obj.get("id") or index
                ),
                "source_category": source_category,
            }
            try:
                converted, was_clipped = quad_object(obj, width, height, batch.clip)
                objects.append(converted)
                converted_count += 1
                category_tiers[source_category][str(converted["geometryTier"])] += 1
                repairs.extend(
                    {**row, "reason": reason}
                    for reason in converted.get("geometryRepairReasons", [])
                )
                if was_clipped:
                    clipped_rows.append(row)
            except GeometryError as exc:
                try:
                    objects.append(_degenerate_ignore_object(obj, width, height))
                    converted_count += 1
                    invalid.append(
                        {
                            **row,
                            "error": str(exc),
                            "resolution": "localized_ignore_hbb",
                        }
                    )
                    category_tiers[source_category]["invalid_ignored"] += 1
                except GeometryError:
                    invalid.append({**row, "error": str(exc), "resolution": "fatal"})
        annotation["objects"] = objects
        write_json(
            batch.destination / split / "ann" / f"{image.name}.json",
            annotation,
            compact=True,
        )
    return (
        len(batch.images),
        converted_count,
        removals,
        invalid,
        clipped_rows,
        repairs,
        {category: dict(values) for category, values in category_tiers.items()},
    )


def _provenance_report(
    source_provenance: list[dict[str, Any]],
    conversion_provenance: list[dict[str, Any]],
    removals: list[dict[str, Any]],
) -> dict[str, Any]:
    """Join source and geometry accounting into the Phase 1.2 contract."""
    conversion_by_dataset = {str(row["dataset"]): row for row in conversion_provenance}
    removals_by_dataset_category: dict[tuple[str, str], Counter[str]] = defaultdict(
        Counter
    )
    for row in removals:
        removals_by_dataset_category[
            (str(row["dataset"]), str(row.get("source_category", "unknown")))
        ][str(row["reason"])] += 1

    datasets = []
    for source in source_provenance:
        dataset = str(source["dataset"])
        conversion = conversion_by_dataset.get(dataset, {})
        geometry_by_category = conversion.get("geometry_tiers_by_category", {})
        categories = []
        source_categories = source.get("per_category", {})
        for category in sorted(set(source_categories) | set(geometry_by_category)):
            source_row = source_categories.get(category, {})
            declared_drop_reasons = {
                key.removeprefix("dropped_reason:"): int(value)
                for key, value in source_row.items()
                if key.startswith("dropped_reason:")
            }
            conversion_drop_reasons = dict(
                sorted(removals_by_dataset_category[(dataset, category)].items())
            )
            all_drop_reasons = Counter(declared_drop_reasons)
            all_drop_reasons.update(conversion_drop_reasons)
            categories.append(
                {
                    "category": category,
                    "source_annotations_read": int(
                        source_row.get("source_annotations_read", 0)
                    ),
                    "unique_source_annotation_identities_retained": int(
                        source_row.get(
                            "unique_source_annotation_identities_retained", 0
                        )
                    ),
                    "multi_part_instances_merged": int(
                        source_row.get("multi_part_instances_merged", 0)
                    ),
                    "exact_geometric_duplicates_removed": sum(
                        conversion_drop_reasons.values()
                    ),
                    "crowd_group_regions_mapped_to_ignore": int(
                        source_row.get("crowd_regions_mapped_to_ignore", 0)
                        + source_row.get("group_regions_mapped_to_ignore", 0)
                    ),
                    "annotations_dropped": sum(all_drop_reasons.values()),
                    "drop_reasons": dict(sorted(all_drop_reasons.items())),
                    "geometry_conversion_tiers": dict(
                        sorted(geometry_by_category.get(category, {}).items())
                    ),
                }
            )
        dataset_drop_reasons: Counter[str] = Counter()
        for row in categories:
            dataset_drop_reasons.update(row["drop_reasons"])
        datasets.append(
            {
                "dataset": dataset,
                "identity_authority": source.get("identity_authority", "unknown"),
                "source_hashes": source.get("source_hashes", {}),
                "source_annotations_read": int(
                    source.get("source_annotations_read", 0)
                ),
                "unique_source_annotation_identities_retained": int(
                    source.get("unique_source_annotation_identities_retained", 0)
                ),
                "multi_part_instances_merged": int(
                    source.get("multi_part_instances_merged", 0)
                ),
                "exact_geometric_duplicates_removed": int(
                    conversion.get("removals", 0)
                ),
                "crowd_group_regions_mapped_to_ignore": int(
                    source.get("crowd_regions_mapped_to_ignore", 0)
                    + source.get("group_regions_mapped_to_ignore", 0)
                ),
                "annotations_dropped": sum(
                    row["annotations_dropped"] for row in categories
                ),
                "drop_reasons": dict(sorted(dataset_drop_reasons.items())),
                "high_multiplicity_exact_geometry_groups": source.get(
                    "high_multiplicity_exact_geometry_groups", []
                ),
                "per_category": categories,
            }
        )
    return {"schema_version": "annotation-provenance.v1", "datasets": datasets}


def convert_all(
    config: Config,
    dataset_name: str | None = None,
    force: bool = False,
    workers: int = 1,
) -> list[dict[str, Any]]:
    reports, invalid, clipped_rows, all_removals, all_repairs = [], [], [], [], []
    conversion_provenance = []
    for dataset in config.selected(dataset_name):
        source = config.workspace_root / "intermediate" / "filtered" / dataset.name
        destination = (
            config.workspace_root / "intermediate" / "detection" / dataset.name
        )
        if destination.exists():
            if (destination / ".detection_complete").exists() and not force:
                reports.append({"dataset": dataset.name, "status": "reused"})
                continue
            if not force:
                raise RuntimeError(
                    f"Incomplete detection project exists: {destination}; use --force"
                )
            shutil.rmtree(destination)
        destination.mkdir(parents=True)
        try:
            meta = read_json(source / "meta.json")
            write_json(destination / "meta.json", meta, compact=True)
            converted_count = 0
            removal_count = 0
            category_tiers: dict[str, Counter[str]] = defaultdict(Counter)
            jobs = (
                _ConversionBatch(
                    dataset.name,
                    batch,
                    destination,
                    bool(config.validation["clip_boxes_to_image"]),
                )
                for batch in batches(iter_project_images(source))
            )
            progress = Progress(f"Converting {dataset.name}", "images")
            for (
                batch_size,
                count,
                batch_removals,
                batch_invalid,
                batch_clipped,
                batch_repairs,
                batch_category_tiers,
            ) in process_map(_convert_batch, jobs, workers):
                converted_count += count
                removal_count += len(batch_removals)
                all_removals.extend(batch_removals)
                invalid.extend(batch_invalid)
                clipped_rows.extend(batch_clipped)
                all_repairs.extend(batch_repairs)
                for category, tiers in batch_category_tiers.items():
                    category_tiers[category].update(tiers)
                progress.add(batch_size)
            progress.finish()
        except Exception:
            shutil.rmtree(destination, ignore_errors=True)
            raise
        if any(
            row["dataset"] == dataset.name and row.get("resolution") == "fatal"
            for row in invalid
        ):
            shutil.rmtree(destination)
            break
        (destination / ".detection_complete").write_text("complete\n", encoding="utf-8")
        reports.append(
            {
                "dataset": dataset.name,
                "converted_annotations": converted_count,
                "deduplicated_geometry_representations": removal_count,
                "clipped": sum(r["dataset"] == dataset.name for r in clipped_rows),
            }
        )
        conversion_provenance.append(
            {
                "dataset": dataset.name,
                "converted_annotations": converted_count,
                "removals": removal_count,
                "removal_reasons": dict(
                    sorted(
                        Counter(
                            row["reason"]
                            for row in all_removals
                            if row["dataset"] == dataset.name
                        ).items()
                    )
                ),
                "geometry_tiers_by_category": {
                    category: dict(sorted(tiers.items()))
                    for category, tiers in sorted(category_tiers.items())
                },
                "geometry_repairs_by_category": {
                    category: dict(
                        sorted(
                            Counter(
                                row["reason"]
                                for row in all_repairs
                                if row["dataset"] == dataset.name
                                and row["source_category"] == category
                            ).items()
                        )
                    )
                    for category in sorted(category_tiers)
                },
            }
        )
    write_json(config.reports / "detection_conversion.json", reports)
    invalid_fields = [
        "dataset",
        "split",
        "image",
        "object_index",
        "source_annotation_id",
        "source_category",
        "error",
        "resolution",
    ]
    write_csv(config.reports / "invalid_geometries.csv", invalid_fields, invalid)
    write_csv(
        config.reports / "clipped_boxes.csv",
        ["dataset", "split", "image", "object_index"],
        clipped_rows,
    )
    write_csv(
        config.reports / "spatial_repairs.csv",
        [
            "dataset",
            "split",
            "image",
            "object_index",
            "source_annotation_id",
            "source_category",
            "reason",
        ],
        all_repairs,
    )
    write_json(config.reports / "conversion_provenance.json", conversion_provenance)
    source_provenance_path = config.reports / "source_provenance.json"
    source_provenance = (
        read_json(source_provenance_path) if source_provenance_path.exists() else []
    )
    selected_names = {dataset.name for dataset in config.selected(dataset_name)}
    write_json(
        config.reports / "annotation_provenance.json",
        _provenance_report(
            [
                row
                for row in source_provenance
                if str(row.get("dataset")) in selected_names
            ],
            conversion_provenance,
            all_removals,
        ),
    )
    write_csv(
        config.reports / "annotation_removals.csv",
        [
            "dataset",
            "split",
            "image",
            "source_annotation_id",
            "source_category",
            "removed_object_id",
            "retained_object_id",
            "reason",
        ],
        all_removals,
    )
    fatal_count = sum(row.get("resolution") == "fatal" for row in invalid)
    if fatal_count:
        raise GeometryError(
            f"Detection conversion found {fatal_count} unrecoverable invalid geometries"
        )
    return reports
