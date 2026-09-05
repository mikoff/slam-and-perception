from __future__ import annotations

import shutil
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

from .coco_official import filter_official_coco
from .config import Config, DatasetConfig
from .discovery import project_root
from .links import link_image
from .parallel import batches, process_map
from .progress import Progress
from .reports import read_json, write_json
from .taxonomy import MappingResult, Taxonomy
from .woodscape_split import SplitAssignment, build_woodscape_split


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


def _polygon_area(obj: dict[str, Any]) -> float:
    points = obj.get("points", {}).get("exterior", [])
    if not isinstance(points, list) or len(points) < 3:
        return 0.0
    try:
        values = [(float(point[0]), float(point[1])) for point in points]
    except (TypeError, ValueError, IndexError):
        return 0.0
    return abs(
        sum(
            values[index][0] * values[(index + 1) % len(values)][1]
            - values[index][1] * values[(index + 1) % len(values)][0]
            for index in range(len(values))
        )
        / 2.0
    )


def _ego_integrity_reason(obj: dict[str, Any], width: int, height: int) -> str | None:
    """Quarantine implausible WoodScape ego masks as ignore, never negative."""
    points = obj.get("points", {}).get("exterior", [])
    if not isinstance(points, list) or len(points) < 3 or width <= 0 or height <= 0:
        return "invalid_source_geometry"
    try:
        values = [(float(point[0]), float(point[1])) for point in points]
    except (TypeError, ValueError, IndexError):
        return "invalid_source_geometry"
    area = _polygon_area(obj)
    if area <= 1.0:
        return "zero_area"
    if area / float(width * height) > 0.60:
        return "excessive_image_coverage"
    touches_boundary = any(
        x <= 3.0 or y <= 3.0 or x >= width - 4.0 or y >= height - 4.0 for x, y in values
    )
    if not touches_boundary:
        return "detached_from_image_boundary"
    return None


def iter_project_images(project: Path) -> Iterator[tuple[str, Path, Path]]:
    for split in sorted(project.iterdir()):
        if (
            not split.is_dir()
            or not (split / "img").is_dir()
            or not (split / "ann").is_dir()
        ):
            continue
        for image in sorted((split / "img").iterdir()):
            if image.is_file():
                yield split.name, image, split / "ann" / f"{image.name}.json"


def selected_project_images(
    project: Path,
    limit_images: int | None,
    policies: dict[str, dict[str, str]],
    split_assignments: dict[tuple[str, str], SplitAssignment] | None = None,
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
    positive: dict[str, list[tuple[str, Path, Path]]] = defaultdict(list)
    empty: dict[str, list[tuple[str, Path, Path]]] = defaultdict(list)
    for item in images:
        split, image, ann_path = item
        has_retained = False
        if ann_path.exists():
            annotation = read_json(ann_path)
            for obj in annotation.get("objects", []):
                policy = policies.get(obj.get("classTitle"))
                if policy is None:
                    raise ValueError(
                        f"annotation references class missing from meta.json: {obj.get('classTitle')!r}"
                    )
                if policy["supervision_state"] == "positive":
                    has_retained = True
                    break
        assignment = (split_assignments or {}).get((split, image.name))
        generated_split = assignment.generated_split if assignment else split
        (positive if has_retained else empty)[generated_split].append(item)

    def interleave(
        buckets: dict[str, list[tuple[str, Path, Path]]],
    ) -> list[tuple[str, Path, Path]]:
        ordered = []
        keys = [key for key in ("train", "val") if buckets.get(key)]
        keys.extend(sorted(set(buckets) - set(keys) - {"test"}))
        while any(buckets.get(key) for key in keys):
            for key in keys:
                if buckets.get(key):
                    ordered.append(buckets[key].pop(0))
        return ordered + buckets.get("test", [])

    return (interleave(positive) + interleave(empty))[:limit_images]


def filtering_plan(
    dataset: DatasetConfig,
    source: Path,
    taxonomy: Taxonomy,
    limit_images: int | None = None,
    split_assignments: dict[tuple[str, str], SplitAssignment] | None = None,
) -> dict[str, Any]:
    meta = read_json(source / "meta.json")
    classes = {item["title"]: item for item in meta.get("classes", [])}
    mappings = {
        title: taxonomy.map(dataset.name, title, item.get("description"))
        for title, item in classes.items()
    }
    policies = {
        title: taxonomy.contract_policy(dataset.name, title) for title in classes
    }
    source_annotations = retained = 0
    category_accounting: dict[str, Counter[str]] = defaultdict(Counter)
    retained_identities: dict[str, set[str]] = defaultdict(set)
    selected = selected_project_images(
        source, limit_images, policies, split_assignments
    )
    selected_list = list(selected)
    progress = Progress(f"Planning filter for {dataset.name}", "images")
    for _, _, ann_path in selected_list:
        if not ann_path.exists():
            progress.add()
            continue
        annotation = read_json(ann_path)
        objects = annotation.get("objects", [])
        source_annotations += len(objects)
        for object_index, obj in enumerate(objects):
            title = obj.get("classTitle")
            if title not in mappings:
                raise ValueError(
                    f"{dataset.name}: annotation references class missing from meta.json: {title!r}"
                )
            policy = policies[title]
            counter = category_accounting[str(title)]
            counter["source_annotations_read"] += 1
            source_id = str(
                obj.get("id") or obj.get("key") or f"{ann_path.name}:{object_index}"
            )
            normalized = taxonomy.normalize(str(title))
            isgroup = "group" in normalized
            state = policy["supervision_state"]
            if state == "drop":
                counter["dropped_reason:approved_drop"] += 1
            else:
                retained += 1
                retained_identities[str(title)].add(source_id)
                counter["retained_annotations"] += 1
                counter[f"retained_state:{state}"] += 1
                counter["group_regions_mapped_to_ignore"] += bool(
                    state == "ignore" and isgroup
                )
        progress.add()
    progress.finish()
    return {
        "dataset": dataset.name,
        "limit_images": limit_images,
        "source_classes": sorted(classes),
        "retained_classes": sorted(
            {
                policy["normalized_category"]
                for policy in policies.values()
                if policy["supervision_state"] == "positive"
            }
        ),
        "has_ignore_regions": any(
            policy["supervision_state"] == "ignore" for policy in policies.values()
        ),
        "has_trusted_background": any(
            policy["supervision_state"] == "trusted_negative"
            for policy in policies.values()
        ),
        "ignored_classes": sorted(
            title
            for title, policy in policies.items()
            if policy["supervision_state"] == "drop"
        ),
        "source_annotation_count": source_annotations,
        "retained_annotation_count": retained,
        "removed_annotation_count": source_annotations - retained,
        "selection_policy": "retained-positive-first"
        if limit_images is not None
        else "all",
        "selected_image_count": len(selected_list),
        "selected_positive_image_count": sum(
            any(
                policies[obj["classTitle"]]["supervision_state"] == "positive"
                for obj in read_json(item[2]).get("objects", [])
            )
            if item[2].exists()
            else False
            for item in selected_list
        ),
        "category_accounting": {
            category: {
                **dict(sorted(counts.items())),
                "unique_source_annotation_identities_retained": len(
                    retained_identities[category]
                ),
            }
            for category, counts in sorted(category_accounting.items())
        },
    }


@dataclass(frozen=True)
class _FilterBatch:
    dataset: str
    images: list[tuple[str, Path, Path]]
    destination: Path
    mappings: dict[str, MappingResult]
    policies: dict[str, dict[str, str]]
    split_assignments: dict[tuple[str, str], SplitAssignment]
    mode: str
    relative: bool
    force: bool


def _filter_batch(
    batch: _FilterBatch,
) -> tuple[int, dict[str, int], dict[str, int]]:
    actions: dict[str, int] = {}
    supervision: Counter[str] = Counter()
    for split, image, ann_path in batch.images:
        assignment = batch.split_assignments.get(
            (split, image.name), SplitAssignment(split, f"{split}:{image.name}")
        )
        output_split = assignment.generated_split
        out_img = batch.destination / output_split / "img" / image.name
        action = link_image(image, out_img, batch.mode, batch.relative, batch.force)
        actions[action] = actions.get(action, 0) + 1
        out_ann = batch.destination / output_split / "ann" / f"{image.name}.json"
        if not ann_path.exists():
            write_json(
                out_ann,
                {
                    "description": "",
                    "size": {"height": 0, "width": 0},
                    "objects": [],
                    "tags": [],
                },
                compact=True,
            )
            continue
        annotation = read_json(ann_path)
        size = annotation.get("size", {})
        width, height = int(size.get("width", 0)), int(size.get("height", 0))
        output_objects = []
        source_image_identity = f"{split}/img/{image.name}"
        for object_index, obj in enumerate(annotation.get("objects", [])):
            mapping = batch.mappings[obj["classTitle"]]
            policy = batch.policies[obj["classTitle"]]
            state = policy["supervision_state"]
            if state == "drop":
                continue
            converted = dict(obj)
            attributes = {
                **_tag_attributes(obj.get("tags")),
                **dict(obj.get("attributes") or {}),
            }
            source_annotation_id = str(
                obj.get("id") or obj.get("key") or f"{image.name}:{object_index}"
            )
            iscrowd = bool(
                attributes.get("iscrowd")
                or attributes.get("is_crowd")
                or attributes.get("crowd")
            )
            normalized_source = obj["classTitle"].casefold().replace(" ", "_")
            isgroup = iscrowd or "group" in normalized_source
            converted["classTitle"] = policy["normalized_category"]
            converted["sourceCategory"] = mapping.source_category
            converted["sourceDataset"] = batch.dataset
            converted["sourceSplit"] = split
            converted["sourceImageIdentity"] = source_image_identity
            converted["sourceAnnotationId"] = source_annotation_id
            converted["sourceAnnotationIdentityKind"] = (
                "supervisely_object_id"
                if obj.get("id") is not None or obj.get("key") is not None
                else "derived_stable_object_index"
            )
            converted["canonicalCategory"] = policy["normalized_category"]
            converted["originalIsCrowd"] = iscrowd
            converted["originalGroup"] = isgroup
            converted["geometryConversionMethod"] = (
                f"source_{str(obj.get('geometryType', 'unknown')).lower()}"
            )
            final_state = (
                "ignore"
                if iscrowd
                else "trusted_background"
                if state == "trusted_negative"
                else state
            )
            integrity_reason = None
            if (
                batch.dataset == "woodscape_rgb_fisheye"
                and str(obj["classTitle"]).strip().casefold().replace(" ", "_")
                == "ego_vehicle"
                and final_state == "trusted_background"
            ):
                integrity_reason = _ego_integrity_reason(obj, width, height)
                if integrity_reason:
                    final_state = "ignore"
            converted["supervisionState"] = final_state
            converted["exclusionReason"] = ""
            converted["attributes"] = attributes
            if final_state == "ignore":
                converted["ignoreRegion"] = True
            if final_state == "trusted_background":
                converted["trustedBackgroundRegion"] = True
            if integrity_reason:
                converted["exclusionReason"] = f"ego_integrity:{integrity_reason}"
                supervision[f"ego_quarantined:{integrity_reason}"] += 1
            supervision[f"state:{final_state}"] += 1
            supervision[f"category_state:{obj['classTitle']}:{final_state}"] += 1
            supervision[f"category_area:{obj['classTitle']}:{final_state}"] += round(
                _polygon_area(obj)
            )
            output_objects.append(converted)
        annotation["objects"] = output_objects
        annotation["sourceDataset"] = batch.dataset
        annotation["sourceSplit"] = split
        annotation["generatedSplit"] = output_split
        annotation["sourceImageIdentity"] = source_image_identity
        annotation["sourceSequenceIdentity"] = assignment.sequence_identity
        write_json(out_ann, annotation, compact=True)
    return len(batch.images), actions, dict(supervision)


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
    dataset: DatasetConfig,
    source: Path,
    destination: Path,
    taxonomy: Taxonomy,
    mode: str = "symlink",
    relative: bool = True,
    limit_images: int | None = None,
    force: bool = False,
    dry_run: bool = False,
    workers: int = 1,
    plan: dict[str, Any] | None = None,
    split_assignments: dict[tuple[str, str], SplitAssignment] | None = None,
) -> tuple[dict[str, Any], dict[str, int]]:
    plan = plan or filtering_plan(
        dataset, source, taxonomy, limit_images, split_assignments
    )
    if dry_run:
        return plan, {}
    if destination.exists():
        if not force:
            if _filter_complete(destination, limit_images):
                return plan, {"reused_project": 1}
            raise RuntimeError(
                f"Incomplete or incompatible filtered project exists: {destination}; use --force"
            )
        shutil.rmtree(destination)
    meta = read_json(source / "meta.json")
    source_classes = {item["title"]: item for item in meta.get("classes", [])}
    mappings = {
        title: taxonomy.map(dataset.name, title, item.get("description"))
        for title, item in source_classes.items()
    }
    policies = {
        title: taxonomy.contract_policy(dataset.name, title) for title in source_classes
    }
    canonical_classes = []
    retained_titles = sorted(
        {
            policy["normalized_category"]
            for policy in policies.values()
            if policy["supervision_state"] != "drop"
        }
    )
    for canonical in retained_titles:
        if canonical:
            canonical_classes.append(
                {
                    "title": canonical,
                    "shape": "any",
                    "geometryType": "any",
                    "color": "#%06x"
                    % (
                        (
                            taxonomy.category_ids.get(canonical, len(canonical))
                            * 2654435761
                        )
                        & 0xFFFFFF
                    ),
                }
            )
    if plan["has_ignore_regions"]:
        canonical_classes.append(
            {
                "title": taxonomy.ignore_region_token,
                "shape": "any",
                "geometryType": "any",
                "color": "#808080",
            }
        )
    destination.mkdir(parents=True)
    try:
        new_meta = dict(meta)
        new_meta["classes"] = canonical_classes
        write_json(destination / "meta.json", new_meta, compact=True)
        images = selected_project_images(
            source, limit_images, policies, split_assignments
        )
        jobs = (
            _FilterBatch(
                dataset.name,
                batch,
                destination,
                mappings,
                policies,
                {
                    (split, image.name): split_assignments[(split, image.name)]
                    for split, image, _ in batch
                    if split_assignments and (split, image.name) in split_assignments
                },
                mode,
                relative,
                force,
            )
            for batch in batches(images)
        )
        actions: dict[str, int] = {}
        supervision: Counter[str] = Counter()
        progress = Progress(f"Filtering {dataset.name}", "images")
        for batch_size, batch_actions, batch_supervision in process_map(
            _filter_batch, jobs, workers
        ):
            for key, value in batch_actions.items():
                actions[key] = actions.get(key, 0) + value
            supervision.update(batch_supervision)
            progress.add(batch_size)
        progress.finish()
        write_json(
            destination / ".supervision_generation.json",
            {
                "dataset": dataset.name,
                "counts": dict(sorted(supervision.items())),
            },
        )
        write_json(destination / ".filter_complete", {"limit_images": limit_images})
        return plan, actions
    except Exception:
        shutil.rmtree(destination, ignore_errors=True)
        raise


def filter_all(
    config: Config,
    taxonomy: Taxonomy,
    dataset_name: str | None = None,
    limit_images: int | None = None,
    force: bool = False,
    dry_run: bool = False,
    workers: int = 1,
) -> list[dict[str, Any]]:
    selected = config.selected(dataset_name)
    sources = {
        dataset.name: project_root(dataset)
        for dataset in selected
        if not dataset.official_annotations
    }
    previous_path = config.reports / "filtering_plan.json"
    previous = read_json(previous_path) if previous_path.exists() else []
    previous_by_name = {item["dataset"]: item for item in previous}
    plans = []
    split_assignments_by_dataset: dict[str, dict[tuple[str, str], SplitAssignment]] = {}
    for dataset in selected:
        if dataset.name != "woodscape_rgb_fisheye":
            continue
        assignments, split_report = build_woodscape_split(
            sources[dataset.name],
            float(config.validation["validation_fraction"]),
            int(config.validation["random_seed"]),
        )
        split_assignments_by_dataset[dataset.name] = assignments
        write_json(config.reports / "woodscape_sequence_split.json", split_report)
    for dataset in selected:
        destination = config.workspace_root / "intermediate" / "filtered" / dataset.name
        reusable = not force and _filter_complete(destination, limit_images)
        if reusable and dataset.name in previous_by_name:
            plans.append(previous_by_name[dataset.name])
        elif dataset.official_annotations:
            plan, _, _ = filter_official_coco(
                dataset,
                destination,
                taxonomy,
                config.storage["image_link_mode"],
                bool(config.storage["relative_symlinks"]),
                limit_images,
                force,
                True,
            )
            plans.append(plan)
        else:
            plans.append(
                filtering_plan(
                    dataset,
                    sources[dataset.name],
                    taxonomy,
                    limit_images,
                    split_assignments_by_dataset.get(dataset.name),
                )
            )
    # This report is intentionally committed before any derived project is written.
    write_json(config.reports / "filtering_plan.json", plans)
    if dry_run:
        return plans
    link_totals = {}
    provenance = []
    for dataset, plan in zip(selected, plans):
        destination = config.workspace_root / "intermediate" / "filtered" / dataset.name
        if dataset.official_annotations:
            _, actions, dataset_provenance = filter_official_coco(
                dataset,
                destination,
                taxonomy,
                config.storage["image_link_mode"],
                bool(config.storage["relative_symlinks"]),
                limit_images,
                force,
                False,
            )
            provenance.append(dataset_provenance)
        else:
            _, actions = filter_project(
                dataset,
                sources[dataset.name],
                destination,
                taxonomy,
                config.storage["image_link_mode"],
                bool(config.storage["relative_symlinks"]),
                limit_images,
                force,
                False,
                workers,
                plan,
                split_assignments_by_dataset.get(dataset.name),
            )
            category_accounting = plan.get("category_accounting", {})
            generation_path = destination / ".supervision_generation.json"
            generation = read_json(generation_path) if generation_path.exists() else {}
            provenance.append(
                {
                    "dataset": dataset.name,
                    "identity_authority": "supervisely_object_id",
                    "source_annotations_read": sum(
                        row.get("source_annotations_read", 0)
                        for row in category_accounting.values()
                    ),
                    "unique_source_annotation_identities_retained": sum(
                        row.get("unique_source_annotation_identities_retained", 0)
                        for row in category_accounting.values()
                    ),
                    "multi_part_instances_merged": 0,
                    "crowd_regions_mapped_to_ignore": 0,
                    "group_regions_mapped_to_ignore": sum(
                        row.get("group_regions_mapped_to_ignore", 0)
                        for row in category_accounting.values()
                    ),
                    "exact_geometric_duplicates_removed": 0,
                    "removal_reasons": dict(
                        sorted(
                            Counter(
                                {
                                    key.removeprefix("dropped_reason:"): sum(
                                        row.get(key, 0)
                                        for row in category_accounting.values()
                                    )
                                    for key in {
                                        field
                                        for row in category_accounting.values()
                                        for field in row
                                        if field.startswith("dropped_reason:")
                                    }
                                }
                            ).items()
                        )
                    ),
                    "generated_supervision_counts": generation.get("counts", {}),
                    "per_category": category_accounting,
                }
            )
        for key, value in actions.items():
            link_totals[key] = link_totals.get(key, 0) + value
    saved = sum(
        path.resolve().stat().st_size
        for path in (config.workspace_root / "intermediate" / "filtered").rglob("*")
        if path.is_symlink() and path.exists()
    )
    write_json(
        config.reports / "link_report.json",
        {
            "new_symlinks": link_totals.get("new_symlink", 0),
            "reused_symlinks": link_totals.get("reused_symlink", 0),
            "hard_links": link_totals.get("hardlink", 0),
            "copied_files": link_totals.get("copied", 0),
            "broken_links": 0,
            "incorrect_links": 0,
            "intermediate_storage_saved": saved,
            "estimated_storage_saved": saved,
        },
    )
    write_json(config.reports / "source_provenance.json", provenance)
    return plans
