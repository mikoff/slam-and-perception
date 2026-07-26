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
        for index in non_rectangles[:pair_count]:
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
                converted, was_clipped = rectangle_object(obj, width, height, batch.clip)
                objects.append(converted)
                converted_count += 1
                if was_clipped:
                    clipped_rows.append(row)
            except GeometryError as exc:
                invalid.append({**row, "error": str(exc)})
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
            for cls in meta.get("classes", []):
                cls["shape"] = cls["geometryType"] = "rectangle"
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
        if invalid and any(row["dataset"] == dataset.name for row in invalid):
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
    fields = ["dataset", "split", "image", "object_index", "error"]
    write_csv(config.reports / "invalid_geometries.csv", fields, invalid)
    write_csv(config.reports / "clipped_boxes.csv", fields, clipped_rows)
    if invalid:
        raise GeometryError(f"Detection conversion found {len(invalid)} invalid geometries")
    return reports
