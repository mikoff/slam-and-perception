from __future__ import annotations

import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

from .config import Config, DatasetConfig
from .discovery import project_root
from .links import link_image
from .parallel import batches, process_map
from .progress import Progress
from .reports import read_json, write_json
from .taxonomy import MappingResult, Taxonomy


def _tag_attributes(tags: Any) -> dict[str, Any]:
    """Normalize Supervisely object tags into stable JSON attributes."""
    attributes: dict[str, Any] = {}
    if not isinstance(tags, list):
        return attributes
    for index, tag in enumerate(tags):
        if not isinstance(tag, dict):
            continue
        name = str(tag.get("name") or tag.get("tagName") or f"tag_{index}")
        value = tag.get("value")
        attributes[name] = True if value is None or value == "" else value
    return attributes


def iter_project_images(project: Path) -> Iterator[tuple[str, Path, Path]]:
    for split in sorted(project.iterdir()):
        if not split.is_dir() or not (split / "img").is_dir() or not (split / "ann").is_dir():
            continue
        for image in sorted((split / "img").iterdir()):
            if image.is_file():
                yield split.name, image, split / "ann" / f"{image.name}.json"


def selected_project_images(
    project: Path,
    limit_images: int | None,
    mappings: dict[str, MappingResult],
) -> list[tuple[str, Path, Path]] | Iterator[tuple[str, Path, Path]]:
    """Choose a deterministic, positive-aware bounded sample.

    A bounded run is used for data validation and smoke training, so selecting
    the first filesystem-sorted images can accidentally produce an unlabeled
    split (for example, a test export).  Prefer images with at least one
    retained object, then fill the remainder with empty images.  Full runs
    retain streaming behaviour.
    """
    images = iter_project_images(project)
    if limit_images is None:
        return images
    if limit_images <= 0:
        return []
    positive: list[tuple[str, Path, Path]] = []
    empty: list[tuple[str, Path, Path]] = []
    for item in images:
        _, _, ann_path = item
        has_retained = False
        if ann_path.exists():
            annotation = read_json(ann_path)
            for obj in annotation.get("objects", []):
                mapping = mappings.get(obj.get("classTitle"))
                if mapping is None:
                    raise ValueError(
                        f"annotation references class missing from meta.json: {obj.get('classTitle')!r}"
                    )
                if not mapping.ignored:
                    has_retained = True
                    break
        (positive if has_retained else empty).append(item)
    return (positive + empty)[:limit_images]


def filtering_plan(dataset: DatasetConfig, source: Path, taxonomy: Taxonomy, limit_images: int | None = None) -> dict[str, Any]:
    meta = read_json(source / "meta.json")
    classes = {item["title"]: item for item in meta.get("classes", [])}
    mappings = {
        title: taxonomy.map(dataset.name, title, item.get("description"))
        for title, item in classes.items()
    }
    source_annotations = retained = 0
    selected = selected_project_images(source, limit_images, mappings)
    selected_list = list(selected)
    progress = Progress(f"Planning filter for {dataset.name}", "images")
    for _, _, ann_path in selected_list:
        if not ann_path.exists():
            progress.add()
            continue
        annotation = read_json(ann_path)
        objects = annotation.get("objects", [])
        source_annotations += len(objects)
        for obj in objects:
            title = obj.get("classTitle")
            if title not in mappings:
                raise ValueError(f"{dataset.name}: annotation references class missing from meta.json: {title!r}")
            retained += not mappings[title].ignored
        progress.add()
    progress.finish()
    return {
        "dataset": dataset.name,
        "limit_images": limit_images,
        "source_classes": sorted(classes),
        "retained_classes": sorted({
            m.canonical for m in mappings.values()
            if not m.ignored and not m.ignore_region and m.canonical is not None
        }),
        "has_ignore_regions": any(m.ignore_region for m in mappings.values()),
        "ignored_classes": sorted(m.source_category for m in mappings.values() if m.ignored),
        "source_annotation_count": source_annotations,
        "retained_annotation_count": retained,
        "removed_annotation_count": source_annotations - retained,
        "selection_policy": "retained-positive-first" if limit_images is not None else "all",
        "selected_image_count": len(selected_list),
        "selected_positive_image_count": sum(
            bool(read_json(item[2]).get("objects", [])) if item[2].exists() else False
            for item in selected_list
        ),
    }


@dataclass(frozen=True)
class _FilterBatch:
    images: list[tuple[str, Path, Path]]
    destination: Path
    mappings: dict[str, MappingResult]
    mode: str
    relative: bool
    force: bool


def _filter_batch(batch: _FilterBatch) -> tuple[int, dict[str, int]]:
    actions: dict[str, int] = {}
    for split, image, ann_path in batch.images:
        out_img = batch.destination / split / "img" / image.name
        action = link_image(image, out_img, batch.mode, batch.relative, batch.force)
        actions[action] = actions.get(action, 0) + 1
        out_ann = batch.destination / split / "ann" / f"{image.name}.json"
        if not ann_path.exists():
            write_json(
                out_ann,
                {"description": "", "size": {"height": 0, "width": 0}, "objects": [], "tags": []},
                compact=True,
            )
            continue
        annotation = read_json(ann_path)
        output_objects = []
        for obj in annotation.get("objects", []):
            mapping = batch.mappings[obj["classTitle"]]
            if mapping.ignored:
                continue
            converted = dict(obj)
            converted["classTitle"] = mapping.canonical
            converted["sourceCategory"] = mapping.source_category
            converted["sourceAnnotationId"] = str(obj.get("id", obj.get("key", "")))
            converted["attributes"] = {
                **_tag_attributes(obj.get("tags")),
                **dict(obj.get("attributes") or {}),
            }
            if mapping.ignore_region:
                converted["ignoreRegion"] = True
            output_objects.append(converted)
        annotation["objects"] = output_objects
        write_json(out_ann, annotation, compact=True)
    return len(batch.images), actions


def _filter_complete(destination: Path, limit_images: int | None) -> bool:
    marker = destination / ".filter_complete"
    if not marker.exists():
        return False
    try:
        value = read_json(marker)
        return value.get("limit_images") == limit_images
    except (ValueError, AttributeError):
        # Markers written by older versions represented full runs.
        return limit_images is None


def filter_project(
    dataset: DatasetConfig, source: Path, destination: Path, taxonomy: Taxonomy,
    mode: str = "symlink", relative: bool = True, limit_images: int | None = None,
    force: bool = False, dry_run: bool = False,
    workers: int = 1, plan: dict[str, Any] | None = None,
) -> tuple[dict[str, Any], dict[str, int]]:
    plan = plan or filtering_plan(dataset, source, taxonomy, limit_images)
    if dry_run:
        return plan, {}
    if destination.exists():
        if not force:
            if _filter_complete(destination, limit_images):
                return plan, {"reused_project": 1}
            raise RuntimeError(f"Incomplete or incompatible filtered project exists: {destination}; use --force")
        shutil.rmtree(destination)
    meta = read_json(source / "meta.json")
    source_classes = {item["title"]: item for item in meta.get("classes", [])}
    mappings = {
        title: taxonomy.map(dataset.name, title, item.get("description"))
        for title, item in source_classes.items()
    }
    canonical_classes = []
    for canonical in taxonomy.data["canonical_id_order"]:
        if canonical in plan["retained_classes"]:
            canonical_classes.append({
                "title": canonical, "shape": "any", "geometryType": "any",
                "color": "#%06x" % ((taxonomy.category_ids[canonical] * 2654435761) & 0xFFFFFF),
            })
    if plan["has_ignore_regions"]:
        canonical_classes.append({
            "title": taxonomy.ignore_region_token,
            "shape": "any",
            "geometryType": "any",
            "color": "#808080",
        })
    destination.mkdir(parents=True)
    try:
        new_meta = dict(meta)
        new_meta["classes"] = canonical_classes
        write_json(destination / "meta.json", new_meta, compact=True)
        images = selected_project_images(source, limit_images, mappings)
        jobs = (
            _FilterBatch(batch, destination, mappings, mode, relative, force)
            for batch in batches(images)
        )
        actions: dict[str, int] = {}
        progress = Progress(f"Filtering {dataset.name}", "images")
        for batch_size, batch_actions in process_map(_filter_batch, jobs, workers):
            for key, value in batch_actions.items():
                actions[key] = actions.get(key, 0) + value
            progress.add(batch_size)
        progress.finish()
        write_json(destination / ".filter_complete", {"limit_images": limit_images})
        return plan, actions
    except Exception:
        shutil.rmtree(destination, ignore_errors=True)
        raise


def filter_all(
    config: Config, taxonomy: Taxonomy, dataset_name: str | None = None,
    limit_images: int | None = None, force: bool = False, dry_run: bool = False,
    workers: int = 1,
) -> list[dict[str, Any]]:
    selected = config.selected(dataset_name)
    sources = {dataset.name: project_root(dataset) for dataset in selected}
    previous_path = config.reports / "filtering_plan.json"
    previous = read_json(previous_path) if previous_path.exists() else []
    previous_by_name = {item["dataset"]: item for item in previous}
    plans = []
    for dataset in selected:
        destination = config.workspace_root / "intermediate" / "filtered" / dataset.name
        reusable = not force and _filter_complete(destination, limit_images)
        if reusable and dataset.name in previous_by_name:
            plans.append(previous_by_name[dataset.name])
        else:
            plans.append(filtering_plan(dataset, sources[dataset.name], taxonomy, limit_images))
    # This report is intentionally committed before any derived project is written.
    write_json(config.reports / "filtering_plan.json", plans)
    if dry_run:
        return plans
    link_totals = {}
    for dataset, plan in zip(selected, plans):
        destination = config.workspace_root / "intermediate" / "filtered" / dataset.name
        _, actions = filter_project(
            dataset, sources[dataset.name], destination, taxonomy,
            config.storage["image_link_mode"], bool(config.storage["relative_symlinks"]),
            limit_images, force, False, workers, plan,
        )
        for key, value in actions.items():
            link_totals[key] = link_totals.get(key, 0) + value
    saved = sum(
        path.resolve().stat().st_size
        for path in (config.workspace_root / "intermediate" / "filtered").rglob("*")
        if path.is_symlink() and path.exists()
    )
    write_json(config.reports / "link_report.json", {
        "new_symlinks": link_totals.get("new_symlink", 0),
        "reused_symlinks": link_totals.get("reused_symlink", 0),
        "hard_links": link_totals.get("hardlink", 0),
        "copied_files": link_totals.get("copied", 0),
        "broken_links": 0, "incorrect_links": 0,
        "intermediate_storage_saved": saved,
        "estimated_storage_saved": saved,
    })
    return plans
