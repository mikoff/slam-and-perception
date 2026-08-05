from __future__ import annotations

import hashlib
import re
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

from .coco_export import discover_exports
from .config import Config
from .links import link_image
from .progress import Progress
from .proposal_manifest import build_manifest
from .reports import read_json, write_json
from .taxonomy import Taxonomy
from .trusted_background import derive_trusted_background


def normalize_split(name: str, aliases: dict[str, list[str]]) -> str | None:
    value = name.casefold()
    for canonical, key in (("train", "train_aliases"), ("val", "val_aliases"), ("test", "test_aliases")):
        if value in {alias.casefold() for alias in aliases[key]}:
            return canonical
    return None


def _stable_token(value: Any) -> str:
    text = str(value)
    clean = re.sub(r"[^a-zA-Z0-9._-]+", "_", text).strip("_")
    return clean[:80] or hashlib.sha256(text.encode()).hexdigest()[:16]


def _fallback_split(dataset: str, source_id: str, fraction: float, seed: int) -> str:
    value = int(hashlib.sha256(f"{seed}:{dataset}:{source_id}".encode()).hexdigest()[:16], 16) / 2**64
    return "val" if value < fraction else "train"


def prune_unreferenced_image_links(
    image_root: Path, referenced: set[str]
) -> int:
    """Remove stale derived symlinks while never touching real image files."""
    removed = 0
    if not image_root.exists():
        return removed
    for path in image_root.rglob("*"):
        if (
            path.is_symlink()
            and path.relative_to(image_root.parent).as_posix() not in referenced
        ):
            path.unlink()
            removed += 1
    return removed


def merge_exports(config: Config, taxonomy: Taxonomy, force: bool = False) -> dict[str, Any]:
    records = []
    datasets_with_val: set[str] = set()
    for path in discover_exports(config.workspace_root / "intermediate" / "coco"):
        data = read_json(path)
        annotations = defaultdict(list)
        for annotation in data["annotations"]:
            annotations[annotation["image_id"]].append(annotation)
        for image in data["images"]:
            source_split = str(image.get("source_split", path.stem.removeprefix("instances_")))
            split = normalize_split(source_split, config.splits)
            if (
                split == "test"
                and image["source_dataset"]
                in set(config.validation["source_test_as_validation"])
            ):
                split = "val"
            if split == "val":
                datasets_with_val.add(image["source_dataset"])
            records.append({"image": image, "annotations": annotations[image["id"]], "split": split, "source_split": source_split})
    split_report = defaultdict(lambda: defaultdict(int))
    retained = 0
    for record in records:
        image = record["image"]
        split = record["split"]
        if split == "test":
            split_report[image["source_dataset"]]["excluded_test"] += 1
            continue
        if split is None:
            split = _fallback_split(
                image["source_dataset"], str(image["source_image_id"]),
                float(config.validation["validation_fraction"]), int(config.validation["random_seed"]),
            )
        elif split == "train" and image["source_dataset"] not in datasets_with_val:
            split = _fallback_split(
                image["source_dataset"], str(image["source_image_id"]),
                float(config.validation["validation_fraction"]), int(config.validation["random_seed"]),
            )
        record["split"] = split
        split_report[image["source_dataset"]][split] += 1
        records[retained] = record
        retained += 1
    del records[retained:]
    records.sort(key=lambda r: (
        r["split"], r["image"]["source_dataset"], r["source_split"],
        str(r["image"]["source_image_id"]), r["image"]["source_file_name"],
    ))

    # Group records by split and free the main records list to save memory
    records_by_split = defaultdict(list)
    for record in records:
        records_by_split[record["split"]].append(record)
    
    records = None
    import gc
    gc.collect()

    link_counts = defaultdict(int)
    results = {}
    background_report = None

    for split in ("train", "val"):
        split_records = records_by_split[split]
        output_data = {"images": [], "annotations": [], "categories": taxonomy.categories}
        image_id = 0
        progress = Progress(f"Merging {split} exports", "images")
        
        for record in split_records:
            image_id += 1
            source = Path(record["image"]["raw_image_path"]).resolve(strict=True)
            source_dataset = record["image"]["source_dataset"]
            source_split = record["source_split"]
            filename = "__".join((
                _stable_token(source_dataset), _stable_token(source_split),
                _stable_token(record["image"]["source_image_id"]), _stable_token(record["image"]["source_file_name"]),
            ))
            destination = config.workspace_root / "output" / "images" / split / filename
            action = link_image(source, destination, "symlink", True, force)
            link_counts[action] += 1
            camera = "fisheye" if source_dataset == "woodscape_rgb_fisheye" else "perspective"
            channel = Path(record["image"]["source_file_name"]).stem if camera == "fisheye" else "unknown"
            
            output_data["images"].append({
                "id": image_id, "file_name": f"images/{split}/{filename}",
                "width": record["image"]["width"], "height": record["image"]["height"],
                "source_dataset": source_dataset, "source_split": source_split,
                "source_image_id": str(record["image"]["source_image_id"]),
                "source_file_name": record["image"]["source_file_name"],
                "camera_type": camera, "camera_channel": channel,
                "background_supervision": source_dataset == "coco_2017",
            })
            
            prepared = record["annotations"]
            for annotation in prepared:
                annotation["image_id"] = image_id
                annotation.pop("id", None)
            prepared.sort(key=lambda a: (
                a["category_id"], *map(float, a["bbox"]), str(a.get("source_annotation_id", "")),
            ))
            output_data["annotations"].extend(prepared)
            
            record.clear()
            progress.add()
            
        progress.finish()
        
        # Free records memory for this split
        records_by_split[split] = None
        gc.collect()

        for annotation_id, annotation in enumerate(output_data["annotations"], 1):
            annotation["id"] = annotation_id

        write_json(
            config.workspace_root / "output" / "annotations" / f"instances_{split}.json",
            output_data,
            compact=True,
        )

        results[split] = {"images": len(output_data["images"]), "annotations": len(output_data["annotations"])}

        # Build proposals manifest
        manifest = build_manifest(output_data, split)
        output_data = None
        gc.collect()

        # Derive trusted background for this split
        split_bg_report = derive_trusted_background(config, taxonomy, {split: manifest})

        write_json(
            config.workspace_root / "output" / "annotations" / f"proposals_{split}.json",
            manifest,
            compact=True,
        )

        manifest = None
        gc.collect()

        if background_report is None:
            background_report = split_bg_report
        else:
            bg_counts = Counter(background_report.get("counts", {}))
            bg_counts.update(split_bg_report.get("counts", {}))
            bg_sources = Counter(background_report.get("source_category_instances", {}))
            bg_sources.update(split_bg_report.get("source_category_instances", {}))
            background_report["counts"] = dict(sorted(bg_counts.items()))
            background_report["source_category_instances"] = dict(sorted(bg_sources.items()))

    write_json(config.reports / "trusted_background.json", background_report)
    
    stale_links_removed = 0
    if force:
        # Memory-efficiently scan references using ijson
        referenced = set()
        for split in ("train", "val"):
            path = config.workspace_root / "output" / "annotations" / f"instances_{split}.json"
            if path.exists():
                import ijson
                with open(path, "rb") as f:
                    for filename in ijson.items(f, "images.item.file_name"):
                        referenced.add(filename)
        stale_links_removed = prune_unreferenced_image_links(
            config.workspace_root / "output" / "images", referenced
        )

    write_json(config.reports / "split_report.json", {key: dict(value) for key, value in split_report.items()})
    write_json(config.reports / "category_mapping.json", {
        name: category_id for name, category_id in taxonomy.category_ids.items()
    })
    existing_report = config.reports / "link_report.json"
    previous = read_json(existing_report) if existing_report.exists() else {}
    final_saved = sum(
        path.resolve().stat().st_size
        for path in (config.workspace_root / "output" / "images").rglob("*")
        if path.is_symlink() and path.exists()
    )
    write_json(existing_report, {
        **previous,
        "final_new_symlinks": link_counts.get("new_symlink", 0),
        "final_reused_symlinks": link_counts.get("reused_symlink", 0),
        "final_storage_saved": final_saved,
        "stale_final_links_removed": stale_links_removed,
        "estimated_storage_saved": previous.get("intermediate_storage_saved", 0) + final_saved,
    })
    return results

