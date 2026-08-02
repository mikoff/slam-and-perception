from __future__ import annotations

import math
from collections import Counter
from pathlib import Path
from typing import Any

from PIL import Image

from .coco_export import discover_exports
from .config import Config
from .progress import Progress
from .proposal_manifest import validate_manifest
from .reports import read_json, write_csv, write_json
from .taxonomy import Taxonomy


def validate_coco(
    path: Path, image_root: Path, taxonomy: Taxonomy, final: bool = False,
    raw_roots: list[Path] | None = None,
) -> tuple[list[dict[str, Any]], dict[str, Counter], list[tuple[str, str]]]:
    errors: list[dict[str, Any]] = []
    data = read_json(path)
    try:
        from pycocotools.coco import COCO
        coco = COCO()
        coco.dataset = data
        coco.createIndex()
    except Exception as exc:
        errors.append({"file": str(path), "error": f"pycocotools: {exc}"})
    images = data.get("images", [])
    annotations = data.get("annotations", [])
    image_by_id = {item.get("id"): item for item in images}
    if len(image_by_id) != len(images):
        errors.append({"file": str(path), "error": "duplicate image IDs"})
    if len({item.get("id") for item in annotations}) != len(annotations):
        errors.append({"file": str(path), "error": "duplicate annotation IDs"})
    configured_ids = taxonomy.configured_category_ids
    present_categories = {item.get("id") for item in data.get("categories", [])}
    if not present_categories.issubset(configured_ids):
        errors.append({"file": str(path), "error": "unconfigured categories in categories list"})
    counts = {"category": Counter(), "source_category": Counter(), "source_dataset": Counter()}
    identities: list[tuple[str, str]] = []
    image_progress = Progress(f"Validating images in {path.name}", "images")
    for image in images:
        counts["source_dataset"][image.get("source_dataset", "unknown")] += 1
        if not final:
            image_progress.add()
            continue
        identities.append((str(image.get("source_dataset")), str(image.get("source_image_id"))))
        candidate = image_root / image["file_name"]
        try:
            resolved = candidate.resolve(strict=True)
            if not resolved.is_file():
                raise FileNotFoundError
            if not candidate.is_symlink():
                errors.append({"file": str(path), "image_id": image["id"], "error": "final image is not a symlink"})
            if raw_roots and not any(resolved.is_relative_to(root) for root in raw_roots):
                errors.append({"file": str(path), "image_id": image["id"], "error": "final link does not point directly into raw extraction"})
            if "intermediate" in image["file_name"].split("/"):
                errors.append({"file": str(path), "image_id": image["id"], "error": "intermediate path in final annotation"})
            with Image.open(resolved) as loaded:
                if loaded.size != (int(image["width"]), int(image["height"])):
                    errors.append({"file": str(path), "image_id": image["id"], "error": "image dimension mismatch"})
        except (OSError, KeyError) as exc:
            errors.append({"file": str(path), "image_id": image.get("id"), "error": f"missing/broken image: {candidate}: {exc}"})
        image_progress.add()
    image_progress.finish()
    annotation_progress = Progress(f"Validating annotations in {path.name}", "annotations")
    for annotation in annotations:
        error_prefix = {"file": str(path), "annotation_id": annotation.get("id")}
        image = image_by_id.get(annotation.get("image_id"))
        if image is None:
            errors.append({**error_prefix, "error": "annotation references missing image"})
            annotation_progress.add()
            continue
        if annotation.get("category_id") not in configured_ids:
            errors.append({**error_prefix, "error": "annotation references unconfigured category"})
        try:
            x, y, width, height = map(float, annotation["bbox"])
            if not all(math.isfinite(value) for value in (x, y, width, height)):
                raise ValueError("non-finite box")
            if width <= 0 or height <= 0:
                raise ValueError("non-positive box")
            if x < 0 or y < 0 or x + width > image["width"] + 1e-6 or y + height > image["height"] + 1e-6:
                raise ValueError("box outside image")
            if abs(float(annotation.get("area", -1)) - width * height) > max(1e-6, width * height * 1e-9):
                raise ValueError("area mismatch")
        except (KeyError, TypeError, ValueError) as exc:
            errors.append({**error_prefix, "error": str(exc)})
        if final and (not annotation.get("source_dataset") or not annotation.get("source_category")):
            errors.append({**error_prefix, "error": "missing source metadata"})
        counts["category"][str(annotation.get("category_id"))] += 1
        counts["source_category"][str(annotation.get("source_category", "unknown"))] += 1
        annotation_progress.add()
    annotation_progress.finish()
    return errors, counts, identities


def validate_all(config: Config, taxonomy: Taxonomy) -> dict[str, Any]:
    files = [(path, config.workspace_root, False) for path in discover_exports(config.workspace_root / "intermediate" / "coco")]
    files.extend(
        (config.workspace_root / "output" / "annotations" / f"instances_{split}.json", config.workspace_root / "output", True)
        for split in ("train", "val")
    )
    manifest_paths = [
        config.workspace_root / "output" / "annotations" / f"proposals_{split}.json"
        for split in ("train", "val")
    ]
    errors, aggregate = [], {key: Counter() for key in ("category", "source_category", "source_dataset")}
    seen_by_split: dict[str, set[tuple[str, str]]] = {}
    raw_roots = [item.extracted_dir.resolve() for item in config.datasets.values()]
    for path, root, final in files:
        if not path.exists():
            errors.append({"file": str(path), "error": "annotation file missing"})
            continue
        current, counts, identities = validate_coco(path, root, taxonomy, final, raw_roots)
        errors.extend(current)
        if final:
            for key in aggregate:
                aggregate[key].update(counts[key])
            split = path.stem.removeprefix("instances_")
            duplicate = [item for item, count in Counter(identities).items() if count > 1]
            for item in duplicate:
                errors.append({"file": str(path), "error": f"duplicate source image: {item}"})
            seen_by_split[split] = set(identities)
    overlap = seen_by_split.get("train", set()) & seen_by_split.get("val", set())
    for item in sorted(overlap):
        errors.append({"file": "merged", "error": f"train/val source overlap: {item}"})
    summary = {"valid": not errors, "files_checked": len(files), "error_count": len(errors)}
    write_json(config.reports / "validation_summary.json", summary)
    write_csv(config.reports / "invalid_annotations.csv", ["file", "image_id", "annotation_id", "error"], errors)
    write_csv(config.reports / "broken_links.csv", ["file", "image_id", "error"], [e for e in errors if "image" in e.get("error", "") or "link" in e.get("error", "")])
    write_csv(config.reports / "duplicate_images.csv", ["file", "error"], [e for e in errors if "duplicate" in e.get("error", "") or "overlap" in e.get("error", "")])
    write_csv(config.reports / "category_counts.csv", ["category_id", "count"], ({"category_id": k, "count": v} for k, v in sorted(aggregate["category"].items())))
    write_csv(config.reports / "source_category_counts.csv", ["source_category", "count"], ({"source_category": k, "count": v} for k, v in sorted(aggregate["source_category"].items())))
    write_csv(config.reports / "source_dataset_counts.csv", ["source_dataset", "count"], ({"source_dataset": k, "count": v} for k, v in sorted(aggregate["source_dataset"].items())))
    clipped_source = config.reports / "clipped_boxes.csv"
    (config.reports / "clipping_report.csv").write_text(
        clipped_source.read_text(encoding="utf-8") if clipped_source.exists() else "dataset,split,image,object_index,error\n",
        encoding="utf-8",
    )
    (config.reports / "validation_report.txt").write_text(
        ("VALID\n" if not errors else "INVALID\n") + "\n".join(f"{e.get('file')}: {e.get('error')}" for e in errors) + "\n",
        encoding="utf-8",
    )
    if errors:
        raise ValueError(f"Validation failed with {len(errors)} errors; see {config.reports}")
    manifest_reports = []
    for path in manifest_paths:
        if not path.exists():
            errors.append({"file": str(path), "error": "proposal manifest missing"})
            continue
        try:
            manifest_reports.append(validate_manifest(path, config.workspace_root / "output"))
        except ValueError as exc:
            errors.append({"file": str(path), "error": str(exc)})
    if errors:
        write_json(config.reports / "proposal_manifest_validation.json", {
            "valid": False,
            "errors": errors,
        })
        raise ValueError(f"Validation failed with {len(errors)} errors; see {config.reports}")
    write_json(config.reports / "proposal_manifest_validation.json", manifest_reports)
    return summary


def verify_final_links(config: Config) -> dict[str, Any]:
    broken = []
    total = 0
    raw_roots = [item.extracted_dir.resolve() for item in config.datasets.values()]
    for path in sorted((config.workspace_root / "output" / "images").rglob("*")):
        if path.is_dir():
            continue
        total += 1
        try:
            target = path.resolve(strict=True)
            if not path.is_symlink() or not target.is_file() or not any(target.is_relative_to(root) for root in raw_roots):
                broken.append({"path": str(path), "target": str(target), "error": "not a direct raw image symlink"})
        except OSError as exc:
            broken.append({"path": str(path), "error": str(exc)})
    write_csv(config.reports / "broken_links.csv", ["path", "target", "error"], broken)
    if broken:
        raise ValueError(f"{len(broken)} invalid image links")
    return {"links": total, "valid": total, "broken": 0}
