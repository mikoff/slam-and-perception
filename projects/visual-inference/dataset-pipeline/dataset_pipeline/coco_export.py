from __future__ import annotations

import shutil
import tempfile
from pathlib import Path
from typing import Any

from .config import Config
from .progress import Progress
from .reports import read_json, write_json
from .supervisely_filter import iter_project_images
from .taxonomy import Taxonomy


def _bbox(obj: dict[str, Any]) -> list[float]:
    exterior = obj.get("quad") or obj["points"]["exterior"]
    points = [[float(point[0]), float(point[1])] for point in exterior]
    if len(points) < 2:
        raise ValueError("detection object must contain at least two bbox points")
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    # Supervisely coordinates are inclusive; COCO stores extents.  Computing
    # min/max over all corners also handles rotated quads whose first two
    # points are not an axis-aligned diagonal.
    return [min(xs), min(ys), max(xs) - min(xs) + 1.0, max(ys) - min(ys) + 1.0]


def _quad(obj: dict[str, Any]) -> list[list[float]]:
    exterior = obj.get("quad") or obj.get("points", {}).get("exterior", [])
    points = [[float(point[0]), float(point[1])] for point in exterior]
    if len(points) == 2:
        x1, y1 = points[0]
        x2, y2 = points[1]
        points = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
    if len(points) != 4:
        raise ValueError("detection object must contain four quad points")
    return points


def _replace_directory(staged: Path, destination: Path) -> None:
    backup = None
    if destination.exists():
        if not destination.is_dir():
            raise RuntimeError(f"Export destination is not a directory: {destination}")
        backup = Path(tempfile.mkdtemp(prefix=f".{destination.name}.previous-", dir=destination.parent))
        backup.rmdir()
        destination.rename(backup)
    try:
        staged.rename(destination)
    except Exception:
        if backup is not None:
            backup.rename(destination)
        raise
    if backup is not None:
        shutil.rmtree(backup, ignore_errors=True)


def export_project(project: Path, output: Path, dataset: str, taxonomy: Taxonomy) -> list[Path]:
    grouped: dict[str, dict[str, list[dict[str, Any]]]] = {}
    progress = Progress(f"Exporting {dataset}", "images")
    for split, image_path, annotation_path in iter_project_images(project):
        data = grouped.setdefault(split, {"images": [], "annotations": []})
        annotation = read_json(annotation_path)
        image_id = len(data["images"]) + 1
        size = annotation["size"]
        source_id = image_path.relative_to(project).as_posix()
        data["images"].append({
            "id": image_id, "file_name": image_path.name,
            "width": int(size["width"]), "height": int(size["height"]),
            "source_dataset": dataset, "source_split": split,
            "source_image_id": source_id, "source_file_name": image_path.name,
            "raw_image_path": str(image_path.resolve(strict=True)),
        })
        for obj_index, obj in enumerate(annotation.get("objects", [])):
            category = obj["classTitle"]
            ignore_region = bool(obj.get("ignoreRegion"))
            if not ignore_region and category not in taxonomy.category_ids:
                raise ValueError(f"Detection project contains noncanonical class {category!r}")
            bbox = _bbox(obj)
            quad = _quad(obj)
            data["annotations"].append({
                "id": len(data["annotations"]) + 1, "image_id": image_id,
                "category_id": (
                    taxonomy.ignore_region_category_id
                    if ignore_region
                    else taxonomy.category_ids[category]
                ),
                "bbox": bbox,
                "area": bbox[2] * bbox[3],
                "iscrowd": int(ignore_region),
                "ignore_region": ignore_region,
                "segmentation": [],
                "quad": quad,
                "geometry_tier": obj.get("geometryTier", "source_hbb"),
                "fit_coverage": float(obj.get("fitCoverage", 1.0)),
                "fit_tightness": float(obj.get("fitTightness", 0.0)),
                "source_dataset": dataset,
                "source_annotation_id": str(obj.get("sourceAnnotationId") or obj.get("id") or obj_index),
                "source_category": obj.get("sourceCategory", category),
                "attributes": obj.get("attributes", {}),
            })
        progress.add()
    progress.finish()
    output.parent.mkdir(parents=True, exist_ok=True)
    staged = Path(tempfile.mkdtemp(prefix=f".{output.name}.export-", dir=output.parent))
    exports = [(split, f"instances_{split}.json") for split in sorted(grouped)]
    try:
        for split, name in exports:
            write_json(
                staged / name,
                {**grouped[split], "categories": taxonomy.categories},
                compact=True,
            )
        paths = [output / name for _, name in exports]
        write_json(staged / "export_manifest.json", {
            "dataset": dataset,
            "annotation_files": [str(path) for path in paths],
        })
        _replace_directory(staged, output)
        return paths
    except Exception:
        shutil.rmtree(staged, ignore_errors=True)
        raise


def export_all(config: Config, taxonomy: Taxonomy, dataset_name: str | None = None) -> list[str]:
    paths = []
    for dataset in config.selected(dataset_name):
        project = config.workspace_root / "intermediate" / "detection" / dataset.name
        output = config.workspace_root / "intermediate" / "coco" / dataset.name
        paths.extend(map(str, export_project(project, output, dataset.name, taxonomy)))
    return paths


def discover_exports(root: Path) -> list[Path]:
    return sorted(root.rglob("instances_*.json"))
