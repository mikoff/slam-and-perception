"""Normalize official COCO instances with original identity and crowd semantics."""

from __future__ import annotations

import hashlib
import json
import os
import sqlite3
import tempfile
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterator

import ijson

from .config import DatasetConfig, OfficialAnnotationConfig
from .discovery import project_root
from .links import link_image
from .progress import Progress
from .reports import write_json
from .taxonomy import Taxonomy


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _items(path: Path, prefix: str) -> Iterator[dict[str, Any]]:
    with path.open("rb") as stream:
        yield from ijson.items(stream, prefix, use_float=True)


def verify_official_annotations(
    sources: dict[str, OfficialAnnotationConfig],
) -> dict[str, str]:
    hashes = {}
    for split, source in sorted(sources.items()):
        if not source.path.is_file():
            raise FileNotFoundError(
                f"official COCO annotations are missing for {split}: {source.path}"
            )
        actual = _sha256(source.path)
        if actual != source.sha256:
            raise ValueError(
                f"official COCO annotation hash mismatch for {split}: "
                f"expected {source.sha256}, got {actual}"
            )
        hashes[split] = actual
    return hashes


def _canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def _geometry_hash(annotation: dict[str, Any]) -> str:
    value = {
        "category_id": annotation["category_id"],
        "bbox": annotation.get("bbox"),
        "segmentation": annotation.get("segmentation"),
        "iscrowd": annotation.get("iscrowd", 0),
    }
    return hashlib.sha256(_canonical_json(value).encode("utf-8")).hexdigest()


def _create_index(
    source: OfficialAnnotationConfig,
    directory: Path,
) -> tuple[sqlite3.Connection, Path, dict[str, Any]]:
    handle, name = tempfile.mkstemp(
        prefix=f".official-coco-{source.path.stem}-", suffix=".sqlite", dir=directory
    )
    os.close(handle)
    Path(name).unlink(missing_ok=True)
    database = Path(name)
    connection = sqlite3.connect(database)
    connection.executescript(
        """
        PRAGMA journal_mode=OFF;
        PRAGMA synchronous=OFF;
        CREATE TABLE annotations (
            image_id INTEGER NOT NULL,
            annotation_id INTEGER PRIMARY KEY,
            category_id INTEGER NOT NULL,
            geometry_hash TEXT NOT NULL,
            payload TEXT NOT NULL
        );
        CREATE INDEX annotations_image ON annotations(image_id, annotation_id);
        CREATE INDEX annotations_geometry
            ON annotations(image_id, category_id, geometry_hash);
        """
    )
    counts: Counter[str] = Counter()
    per_category: dict[int, Counter[str]] = defaultdict(Counter)
    batch = []
    progress = Progress(f"Indexing {source.path.name}", "annotations")
    try:
        for annotation in _items(source.path, "annotations.item"):
            annotation_id = int(annotation["id"])
            category_id = int(annotation["category_id"])
            segmentation = annotation.get("segmentation")
            segment_count = len(segmentation) if isinstance(segmentation, list) else 0
            iscrowd = bool(annotation.get("iscrowd", 0))
            counts["source_annotations_read"] += 1
            counts["crowd_annotations"] += iscrowd
            counts["multi_part_instances"] += segment_count > 1
            per_category[category_id]["source_annotations_read"] += 1
            per_category[category_id]["crowd_annotations"] += iscrowd
            per_category[category_id]["multi_part_instances"] += segment_count > 1
            batch.append(
                (
                    int(annotation["image_id"]),
                    annotation_id,
                    category_id,
                    _geometry_hash(annotation),
                    _canonical_json(annotation),
                )
            )
            if len(batch) >= 10_000:
                connection.executemany(
                    "INSERT INTO annotations VALUES (?, ?, ?, ?, ?)", batch
                )
                connection.commit()
                progress.add(len(batch))
                batch.clear()
        if batch:
            connection.executemany(
                "INSERT INTO annotations VALUES (?, ?, ?, ?, ?)", batch
            )
            connection.commit()
            progress.add(len(batch))
        progress.finish()
    except sqlite3.IntegrityError as exc:
        connection.close()
        database.unlink(missing_ok=True)
        raise ValueError(
            f"official COCO source contains a duplicate annotation identity: {exc}"
        ) from exc
    high_multiplicity = [
        {
            "image_id": int(image_id),
            "category_id": int(category_id),
            "geometry_hash": geometry_hash,
            "multiplicity": int(multiplicity),
            "annotation_ids": [
                int(row[0])
                for row in connection.execute(
                    """
                    SELECT annotation_id FROM annotations
                    WHERE image_id=? AND category_id=? AND geometry_hash=?
                    ORDER BY annotation_id
                    """,
                    (image_id, category_id, geometry_hash),
                )
            ],
        }
        for image_id, category_id, geometry_hash, multiplicity in connection.execute(
            """
            SELECT image_id, category_id, geometry_hash, COUNT(*)
            FROM annotations
            GROUP BY image_id, category_id, geometry_hash
            HAVING COUNT(*) > 1
            ORDER BY COUNT(*) DESC, image_id, category_id, geometry_hash
            """
        )
    ]
    return (
        connection,
        database,
        {
            "counts": dict(sorted(counts.items())),
            "per_category_id": {
                str(category): dict(sorted(values.items()))
                for category, values in sorted(per_category.items())
            },
            "high_multiplicity_exact_geometry_groups": high_multiplicity,
        },
    )


def _categories(path: Path) -> dict[int, str]:
    return {
        int(category["id"]): str(category["name"])
        for category in _items(path, "categories.item")
    }


def _segments(annotation: dict[str, Any]) -> list[list[list[float]]]:
    segmentation = annotation.get("segmentation")
    if not isinstance(segmentation, list):
        return []
    result = []
    for segment in segmentation:
        if not isinstance(segment, list) or len(segment) < 6 or len(segment) % 2:
            continue
        result.append(
            [
                [float(segment[index]), float(segment[index + 1])]
                for index in range(0, len(segment), 2)
            ]
        )
    return result


def _object(
    annotation: dict[str, Any],
    category: str,
    split: str,
    image_id: int,
    taxonomy: Taxonomy,
) -> dict[str, Any]:
    mapping = taxonomy.map("coco_2017", category)
    if mapping.canonical is None:
        raise ValueError(f"official COCO category is not canonical: {category!r}")
    segments = _segments(annotation)
    iscrowd = bool(annotation.get("iscrowd", 0))
    if segments and not iscrowd:
        geometry_type = "multipolygon" if len(segments) > 1 else "polygon"
        points = segments[0]
        method = "source_multipolygon" if len(segments) > 1 else "source_polygon"
    else:
        x, y, width, height = map(float, annotation["bbox"])
        geometry_type = "rectangle"
        points = [[x, y], [x + width, y + height]]
        method = "official_bbox_fallback"
    state = "ignore" if iscrowd else "positive"
    result = {
        "id": int(annotation["id"]),
        "classTitle": mapping.canonical,
        "geometryType": geometry_type,
        "points": {"exterior": points, "interior": []},
        "segments": segments,
        "sourceArea": float(annotation.get("area", 0.0)),
        "sourceDataset": "coco_2017",
        "sourceSplit": split,
        "sourceImageIdentity": str(image_id),
        "sourceAnnotationId": str(annotation["id"]),
        "sourceAnnotationIdentityKind": "official_coco_annotation_id",
        "sourceCategory": category,
        "canonicalCategory": mapping.canonical,
        "originalIsCrowd": iscrowd,
        "originalGroup": iscrowd,
        "geometryConversionMethod": method,
        "supervisionState": state,
        "exclusionReason": "",
        "sourceSegmentCount": len(segments),
        "ignoreRegion": iscrowd,
        "attributes": {},
    }
    return result


def _source_image_path(directory: Path, official_file_name: str) -> tuple[Path, bool]:
    """Resolve Dataset Ninja's occasional appended image-format extension."""
    exact = directory / official_file_name
    if exact.is_file():
        return exact, False
    alternatives = sorted(
        path for path in directory.glob(f"{official_file_name}.*") if path.is_file()
    )
    if len(alternatives) == 1:
        return alternatives[0], True
    if not alternatives:
        raise FileNotFoundError(
            f"official COCO image is missing from extracted project: {exact}"
        )
    raise ValueError(
        f"official COCO image has ambiguous local filename alternatives: {exact}"
    )


def filter_official_coco(
    dataset: DatasetConfig,
    destination: Path,
    taxonomy: Taxonomy,
    mode: str,
    relative: bool,
    limit_images: int | None,
    force: bool,
    dry_run: bool,
) -> tuple[dict[str, Any], dict[str, int], dict[str, Any]]:
    """Build the normalized COCO project from official instance identities."""
    sources = dataset.official_annotations or {}
    if not sources:
        raise ValueError("official COCO filtering requires official_annotations")
    hashes = verify_official_annotations(sources)
    marker = destination / ".filter_complete"
    if destination.exists() and marker.exists() and not force:
        cached = json.loads(marker.read_text(encoding="utf-8"))
        if (
            cached.get("limit_images") == limit_images
            and cached.get("source_hashes") == hashes
        ):
            return cached["plan"], {"reused_project": 1}, cached["provenance"]
    if dry_run:
        source_count = sum(
            1
            for source in sources.values()
            for _ in _items(source.path, "annotations.item")
        )
        plan = {
            "dataset": dataset.name,
            "limit_images": limit_images,
            "source_annotation_count": source_count,
            "identity_authority": "official_coco_annotation_id",
            "source_hashes": hashes,
            "selection_policy": "official_image_order",
        }
        return plan, {}, {"dataset": dataset.name, "source_hashes": hashes}
    if destination.exists():
        if not force:
            raise RuntimeError(
                f"Incomplete or incompatible filtered project exists: {destination}; use --force"
            )
        import shutil

        shutil.rmtree(destination)
    destination.mkdir(parents=True)
    source_project = project_root(dataset, sdk_check=False)
    actions: Counter[str] = Counter()
    total_images = 0
    selected_images = 0
    source_annotations = 0
    retained_annotations = 0
    crowd_annotations = 0
    multipart_instances = 0
    per_category: dict[str, Counter[str]] = defaultdict(Counter)
    high_multiplicity: list[dict[str, Any]] = []
    filename_fallbacks: list[dict[str, Any]] = []
    try:
        write_json(
            destination / "meta.json",
            {
                "projectType": "images",
                "classes": [
                    {"title": name, "shape": "any", "geometryType": "any"}
                    for name in taxonomy.data["canonical_id_order"]
                ],
                "tags": [],
            },
            compact=True,
        )
        for split, source in sorted(sources.items()):
            categories = _categories(source.path)
            connection, database, index_report = _create_index(
                source, destination.parent
            )
            source_annotations += index_report["counts"].get(
                "source_annotations_read", 0
            )
            crowd_annotations += index_report["counts"].get("crowd_annotations", 0)
            multipart_instances += index_report["counts"].get("multi_part_instances", 0)
            for row in index_report["high_multiplicity_exact_geometry_groups"]:
                high_multiplicity.append({"split": split, **row})
            try:
                progress = Progress(f"Normalizing official COCO {split}", "images")
                for image in _items(source.path, "images.item"):
                    total_images += 1
                    if limit_images is not None and selected_images >= limit_images:
                        continue
                    selected_images += 1
                    image_id = int(image["id"])
                    file_name = str(image["file_name"])
                    source_image, used_fallback = _source_image_path(
                        source_project / split / "img", file_name
                    )
                    output_file_name = source_image.name
                    if used_fallback:
                        filename_fallbacks.append(
                            {
                                "split": split,
                                "image_id": image_id,
                                "official_file_name": file_name,
                                "local_file_name": output_file_name,
                                "reason": "dataset_ninja_appended_image_format_extension",
                            }
                        )
                    action = link_image(
                        source_image,
                        destination / split / "img" / output_file_name,
                        mode,
                        relative,
                        force,
                    )
                    actions[action] += 1
                    objects = []
                    for (payload,) in connection.execute(
                        "SELECT payload FROM annotations WHERE image_id=? ORDER BY annotation_id",
                        (image_id,),
                    ):
                        annotation = json.loads(payload)
                        category = categories[int(annotation["category_id"])]
                        obj = _object(annotation, category, split, image_id, taxonomy)
                        objects.append(obj)
                        retained_annotations += 1
                        counter = per_category[category]
                        counter["source_annotations_read"] += 1
                        counter["unique_source_annotation_identities_retained"] += 1
                        counter["crowd_regions_mapped_to_ignore"] += bool(
                            obj["originalIsCrowd"]
                        )
                        counter["multi_part_instances_merged"] += (
                            obj["sourceSegmentCount"] > 1
                        )
                    write_json(
                        destination / split / "ann" / f"{output_file_name}.json",
                        {
                            "description": "",
                            "size": {
                                "height": int(image["height"]),
                                "width": int(image["width"]),
                            },
                            "objects": objects,
                            "tags": [],
                            "sourceDataset": dataset.name,
                            "sourceSplit": split,
                            "sourceImageIdentity": str(image_id),
                        },
                        compact=True,
                    )
                    progress.add()
                progress.finish()
            finally:
                connection.close()
                database.unlink(missing_ok=True)
        plan = {
            "dataset": dataset.name,
            "limit_images": limit_images,
            "source_annotation_count": source_annotations,
            "retained_annotation_count": retained_annotations,
            "removed_annotation_count": 0,
            "selected_image_count": selected_images,
            "source_image_count": total_images,
            "identity_authority": "official_coco_annotation_id",
            "source_hashes": hashes,
            "selection_policy": "official_image_order",
        }
        provenance = {
            "dataset": dataset.name,
            "identity_authority": "official_coco_annotation_id",
            "source_hashes": hashes,
            "source_annotations_read": source_annotations,
            "unique_source_annotation_identities_retained": retained_annotations,
            "multi_part_instances_merged": multipart_instances,
            "crowd_regions_mapped_to_ignore": crowd_annotations,
            "exact_geometric_duplicates_removed": 0,
            "removal_reasons": {},
            "high_multiplicity_exact_geometry_groups": high_multiplicity,
            "image_filename_fallbacks": filename_fallbacks,
            "per_category": {
                category: dict(sorted(values.items()))
                for category, values in sorted(per_category.items())
            },
        }
        write_json(
            marker,
            {
                "limit_images": limit_images,
                "source_hashes": hashes,
                "plan": plan,
                "provenance": provenance,
            },
        )
        return plan, dict(actions), provenance
    except Exception:
        import shutil

        shutil.rmtree(destination, ignore_errors=True)
        raise
