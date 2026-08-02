from __future__ import annotations

import math
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

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
    if geometry in {"rectangle", "polygon", "polyline"}:
        points = _flatten_points(obj.get("points", {}).get("exterior", []))
        if not points:
            raise GeometryError(f"{geometry} has no points")
        xs, ys = zip(*points)
        return min(xs), min(ys), max(xs), max(ys)
    if geometry in {"bitmap", "mask"}:
        try:
            import supervisely as sly
            bitmap = sly.Bitmap.from_json(obj)
            rectangle = bitmap.to_bbox()
            points = _flatten_points(rectangle.to_json().get("points", {}).get("exterior", []))
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
    if geometry in {"rectangle", "polygon", "polyline"}:
        return _flatten_points(obj.get("points", {}).get("exterior", []))
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


def _canonical_quad(points: list[tuple[float, float]] | list[list[float]]) -> list[list[float]]:
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


def _point_coverage(points: list[tuple[float, float]], quad: list[list[float]]) -> float:
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
        inside += (
            all(distance >= -0.25 for distance in signed_distances)
            or all(distance <= 0.25 for distance in signed_distances)
        )
    return inside / max(len(points), 1)


def _source_area(source_geometry: str, points: list[tuple[float, float]]) -> float:
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
    obj: dict[str, Any], width: int, height: int, clip: bool,
) -> tuple[dict[str, Any], bool]:
    """Preserve a valid source quad or fit a containing rotated rectangle."""
    source_geometry = str(obj.get("geometryType", "")).lower()
    points = _source_points(obj)
    if not points:
        raise GeometryError("source geometry has no points")
    changed = False
    clipped_points = []
    for x, y in points:
        if x < 0 or y < 0 or x > width - 1 or y > height - 1:
            changed = True
        clipped_points.append((
            min(max(0.0, x), float(width - 1)),
            min(max(0.0, y), float(height - 1)),
        ))
    if changed and not clip:
        raise GeometryError("geometry lies partly outside image")
    points = clipped_points
    if source_geometry == "rectangle" or len(points) < 3:
        x1, y1, x2, y2 = _quad_bbox([[x, y] for x, y in points])
        quad = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
        tier = "source_hbb" if source_geometry == "rectangle" else "hbb_fallback"
    elif len(points) == 4:
        try:
            quad = _canonical_quad(points)
            tier = "source_quad"
        except GeometryError:
            quad = _fit_quad(points)
            tier = "rotated_rect"
    else:
        quad = _fit_quad(points)
        tier = "fitted_quad"
    try:
        quad = _canonical_quad(quad)
    except GeometryError:
        x1, y1, x2, y2 = _quad_bbox([[x, y] for x, y in points])
        quad = _canonical_quad([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
        tier = "hbb_fallback"
    # A minimum-area rectangle can extend beyond the image even when every
    # source point is in-bounds (the fitted corners lie outside a clipped
    # object near an image edge).  Preserve the containment contract by using
    # an axis-aligned fallback in that case; this is preferable to silently
    # emitting an invalid quad or coordinate-wise clamping its topology.
    if any(
        x < 0.0 or y < 0.0 or x > width - 1 or y > height - 1
        for x, y in quad
    ):
        x1, y1, x2, y2 = _quad_bbox([[x, y] for x, y in points])
        quad = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
        quad = _canonical_quad(quad)
        tier = "hbb_fallback"
    bbox = _quad_bbox(quad)
    fit_coverage = _point_coverage(points, quad)
    if fit_coverage < 0.98:
        x1, y1, x2, y2 = _quad_bbox([[x, y] for x, y in points])
        quad = _canonical_quad([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
        bbox = _quad_bbox(quad)
        fit_coverage = _point_coverage(points, quad)
        tier = "hbb_fallback"
    quad_area = abs(_signed_area([(point[0], point[1]) for point in quad]))
    output = dict(obj)
    output["geometryType"] = "rectangle" if tier == "source_hbb" else "polygon"
    output["points"] = {"exterior": quad, "interior": []}
    output["quad"] = quad
    output["geometryTier"] = tier
    output["fitCoverage"] = fit_coverage
    # Occupancy ratio: 1.0 is tight; lower values include more background.
    output["fitTightness"] = min(_source_area(source_geometry, points) / max(quad_area, 1e-7), 1.0)
    output["sourceGeometryType"] = source_geometry
    output.pop("bitmap", None)
    return output, changed


def clip_bbox(
    bbox: tuple[float, float, float, float], width: int, height: int, clip: bool = True,
) -> tuple[tuple[float, float, float, float], bool]:
    x1, y1, x2, y2 = bbox
    if not all(math.isfinite(v) for v in bbox):
        raise GeometryError("non-finite coordinates")
    if width <= 0 or height <= 0:
        raise GeometryError("invalid image dimensions")
    if x2 < 0 or y2 < 0 or x1 > width - 1 or y1 > height - 1:
        raise GeometryError("box is entirely outside image")
    clipped = (max(0.0, x1), max(0.0, y1), min(float(width - 1), x2), min(float(height - 1), y2))
    changed = clipped != bbox
    if changed and not clip:
        raise GeometryError("box lies partly outside image")
    x1, y1, x2, y2 = clipped if clip else bbox
    if x2 < x1 or y2 < y1:
        raise GeometryError("box has non-positive dimensions")
    return (x1, y1, x2, y2), changed


def rectangle_object(obj: dict[str, Any], width: int, height: int, clip: bool) -> tuple[dict[str, Any], bool]:
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
    output.update({
        "geometryType": "polygon",
        "points": {"exterior": quad, "interior": []},
        "quad": quad,
        "geometryTier": "hbb_fallback",
        "fitCoverage": 1.0,
        "fitTightness": 0.0,
        "sourceGeometryType": str(obj.get("geometryType", "")).lower(),
        "ignoreRegion": True,
    })
    output.pop("bitmap", None)
    return output


def deduplicate_geometry_representations(
    objects: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], int]:
    """Collapse polygon/rectangle duplicates emitted for one source instance.

    Dataset Ninja COCO exports may contain both a polygon and its enclosing
    rectangle as separate objects. Only a mixed-geometry pair with the same
    class and exact enclosing box is collapsed; genuine same-geometry instances
    are retained.
    """
    groups: dict[tuple[str, tuple[float, float, float, float]], list[int]] = {}
    for index, obj in enumerate(objects):
        try:
            bbox = tuple(round(value, 6) for value in object_bbox(obj))
        except GeometryError:
            continue
        key = (str(obj.get("classTitle", "")), bbox)
        groups.setdefault(key, []).append(index)

    discarded: set[int] = set()
    for indices in groups.values():
        rectangles = [
            index for index in indices
            if str(objects[index].get("geometryType", "")).lower() == "rectangle"
        ]
        non_rectangles = [index for index in indices if index not in rectangles]
        pair_count = min(len(rectangles), len(non_rectangles))
        for index in rectangles[:pair_count]:
            discarded.add(index)
    return (
        [obj for index, obj in enumerate(objects) if index not in discarded],
        len(discarded),
    )


@dataclass(frozen=True)
class _ConversionBatch:
    dataset: str
    images: list[tuple[str, Path, Path]]
    destination: Path
    clip: bool


def _convert_batch(
    batch: _ConversionBatch,
) -> tuple[int, int, int, list[dict[str, Any]], list[dict[str, Any]]]:
    converted_count = 0
    duplicate_count = 0
    invalid = []
    clipped_rows = []
    for split, image, ann_path in batch.images:
        link_image(image.resolve(), batch.destination / split / "img" / image.name, "symlink", True)
        annotation = read_json(ann_path)
        size = annotation.get("size", {})
        width, height = int(size.get("width", 0)), int(size.get("height", 0))
        if not width or not height:
            from PIL import Image
            with Image.open(image) as loaded:
                width, height = loaded.size
            annotation["size"] = {"width": width, "height": height}
        objects = []
        source_objects, removed = deduplicate_geometry_representations(
            annotation.get("objects", [])
        )
        duplicate_count += removed
        for index, obj in enumerate(source_objects):
            row = {"dataset": batch.dataset, "split": split, "image": image.name, "object_index": index}
            try:
                converted, was_clipped = quad_object(obj, width, height, batch.clip)
                objects.append(converted)
                converted_count += 1
                if was_clipped:
                    clipped_rows.append(row)
            except GeometryError as exc:
                try:
                    objects.append(_degenerate_ignore_object(obj, width, height))
                    converted_count += 1
                    invalid.append({
                        **row,
                        "error": str(exc),
                        "resolution": "localized_ignore_hbb",
                    })
                except GeometryError:
                    invalid.append({**row, "error": str(exc), "resolution": "fatal"})
        annotation["objects"] = objects
        write_json(
            batch.destination / split / "ann" / f"{image.name}.json",
            annotation,
            compact=True,
        )
    return len(batch.images), converted_count, duplicate_count, invalid, clipped_rows


def convert_all(
    config: Config, dataset_name: str | None = None, force: bool = False, workers: int = 1,
) -> list[dict[str, Any]]:
    reports, invalid, clipped_rows = [], [], []
    for dataset in config.selected(dataset_name):
        source = config.workspace_root / "intermediate" / "filtered" / dataset.name
        destination = config.workspace_root / "intermediate" / "detection" / dataset.name
        if destination.exists():
            if (destination / ".detection_complete").exists() and not force:
                reports.append({"dataset": dataset.name, "status": "reused"})
                continue
            if not force:
                raise RuntimeError(f"Incomplete detection project exists: {destination}; use --force")
            shutil.rmtree(destination)
        destination.mkdir(parents=True)
        try:
            meta = read_json(source / "meta.json")
            write_json(destination / "meta.json", meta, compact=True)
            converted_count = 0
            duplicate_count = 0
            jobs = (
                _ConversionBatch(
                    dataset.name, batch, destination,
                    bool(config.validation["clip_boxes_to_image"]),
                )
                for batch in batches(iter_project_images(source))
            )
            progress = Progress(f"Converting {dataset.name}", "images")
            for (
                batch_size,
                count,
                batch_duplicates,
                batch_invalid,
                batch_clipped,
            ) in process_map(_convert_batch, jobs, workers):
                converted_count += count
                duplicate_count += batch_duplicates
                invalid.extend(batch_invalid)
                clipped_rows.extend(batch_clipped)
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
        reports.append({
            "dataset": dataset.name,
            "converted_annotations": converted_count,
            "deduplicated_geometry_representations": duplicate_count,
            "clipped": sum(r["dataset"] == dataset.name for r in clipped_rows),
        })
    write_json(config.reports / "detection_conversion.json", reports)
    fields = ["dataset", "split", "image", "object_index", "error", "resolution"]
    write_csv(config.reports / "invalid_geometries.csv", fields, invalid)
    write_csv(config.reports / "clipped_boxes.csv", fields, clipped_rows)
    fatal_count = sum(row.get("resolution") == "fatal" for row in invalid)
    if fatal_count:
        raise GeometryError(f"Detection conversion found {fatal_count} unrecoverable invalid geometries")
    return reports
