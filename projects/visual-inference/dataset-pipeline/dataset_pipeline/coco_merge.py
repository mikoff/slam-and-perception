from __future__ import annotations

import hashlib
import heapq
import json
import os
import re
import tempfile
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterator, TextIO

import ijson

from .coco_export import discover_exports
from .config import Config
from .links import link_image
from .progress import Progress
from .proposal_manifest import SCHEMA_VERSION, build_manifest
from .reports import read_json, write_json
from .taxonomy import Taxonomy
from .trusted_background import derive_trusted_background


def normalize_split(name: str, aliases: dict[str, list[str]]) -> str | None:
    value = name.casefold()
    for canonical, key in (
        ("train", "train_aliases"),
        ("val", "val_aliases"),
        ("test", "test_aliases"),
    ):
        if value in {alias.casefold() for alias in aliases[key]}:
            return canonical
    return None


def _stable_token(value: Any) -> str:
    text = str(value)
    clean = re.sub(r"[^a-zA-Z0-9._-]+", "_", text).strip("_")
    return clean[:80] or hashlib.sha256(text.encode()).hexdigest()[:16]


def _fallback_split(dataset: str, source_id: str, fraction: float, seed: int) -> str:
    value = (
        int(
            hashlib.sha256(f"{seed}:{dataset}:{source_id}".encode()).hexdigest()[:16],
            16,
        )
        / 2**64
    )
    return "val" if value < fraction else "train"


def prune_unreferenced_image_links(image_root: Path, referenced: set[str]) -> int:
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


def _record_sort_key(record: dict[str, Any]) -> tuple[str, str, str, str]:
    image = record["image"]
    return (
        str(image["source_dataset"]),
        str(record["source_split"]),
        str(image["source_image_id"]),
        str(image["source_file_name"]),
    )


def _write_fragment(stream: TextIO, value: Any, first: bool) -> bool:
    if not first:
        stream.write(",")
    json.dump(
        value,
        stream,
        separators=(",", ":"),
        sort_keys=True,
        ensure_ascii=False,
    )
    return False


def _iter_json_lines(path: Path) -> Iterator[dict[str, Any]]:
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            yield json.loads(line)


def _assemble_json(
    path: Path,
    fields: list[tuple[str, Any, bool]],
) -> None:
    """Atomically assemble an object from JSON values and array fragments."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        with temporary.open("w", encoding="utf-8") as output:
            output.write("{")
            for index, (name, value, is_fragment) in enumerate(fields):
                if index:
                    output.write(",")
                output.write(json.dumps(name))
                output.write(":")
                if is_fragment:
                    output.write("[")
                    with Path(value).open(encoding="utf-8") as fragment:
                        while chunk := fragment.read(1024 * 1024):
                            output.write(chunk)
                    output.write("]")
                else:
                    json.dump(
                        value,
                        output,
                        separators=(",", ":"),
                        sort_keys=True,
                        ensure_ascii=False,
                    )
            output.write("}\n")
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _combine_background_reports(
    combined: dict[str, Any], current: dict[str, Any]
) -> None:
    for field in (
        "counts",
        "source_category_instances",
        "source_category_area_pixels",
    ):
        values = Counter(combined.get(field, {}))
        values.update(current.get(field, {}))
        combined[field] = dict(sorted(values.items()))
    configured = {
        dataset: set(categories)
        for dataset, categories in combined.get("configured_categories", {}).items()
    }
    for dataset, categories in current.get("configured_categories", {}).items():
        configured.setdefault(dataset, set()).update(categories)
    combined["configured_categories"] = {
        dataset: sorted(categories)
        for dataset, categories in sorted(configured.items())
    }


def _datasets_with_validation(
    paths: list[Path], config: Config
) -> set[str]:
    result: set[str] = set()
    source_test_as_validation = set(
        config.validation["source_test_as_validation"]
    )
    for path in paths:
        with path.open("rb") as stream:
            for image in ijson.items(stream, "images.item"):
                source_split = str(
                    image.get(
                        "source_split", path.stem.removeprefix("instances_")
                    )
                )
                generated_split = str(image.get("generated_split", source_split))
                split = normalize_split(generated_split, config.splits)
                if split == "test" and image["source_dataset"] in source_test_as_validation:
                    split = "val"
                if split == "val":
                    result.add(str(image["source_dataset"]))
    return result


def _stage_sorted_shards(
    paths: list[Path],
    directory: Path,
    config: Config,
    datasets_with_val: set[str],
) -> tuple[dict[str, list[Path]], dict[str, dict[str, int]]]:
    """Bound peak memory to one export and spill globally mergeable shards."""
    shards: dict[str, list[Path]] = {"train": [], "val": []}
    split_report: defaultdict[str, defaultdict[str, int]] = defaultdict(
        lambda: defaultdict(int)
    )
    source_test_as_validation = set(
        config.validation["source_test_as_validation"]
    )
    for path_index, path in enumerate(paths):
        data = read_json(path)
        annotations: defaultdict[Any, list[dict[str, Any]]] = defaultdict(list)
        for annotation in data["annotations"]:
            annotations[annotation["image_id"]].append(annotation)
        records_by_split: dict[str, list[dict[str, Any]]] = {
            "train": [],
            "val": [],
        }
        for image in data["images"]:
            source_split = str(
                image.get("source_split", path.stem.removeprefix("instances_"))
            )
            generated_split = str(image.get("generated_split", source_split))
            split = normalize_split(generated_split, config.splits)
            if split == "test" and image["source_dataset"] in source_test_as_validation:
                split = "val"
            if split == "test":
                split_report[image["source_dataset"]]["excluded_test"] += 1
                continue
            if split is None or (
                split == "train" and image["source_dataset"] not in datasets_with_val
            ):
                split = _fallback_split(
                    image["source_dataset"],
                    str(image["source_image_id"]),
                    float(config.validation["validation_fraction"]),
                    int(config.validation["random_seed"]),
                )
            split_report[image["source_dataset"]][split] += 1
            records_by_split[split].append(
                {
                    "image": image,
                    "annotations": annotations[image["id"]],
                    "source_split": source_split,
                }
            )
        for split, records in records_by_split.items():
            if not records:
                continue
            records.sort(key=_record_sort_key)
            shard = directory / f"{split}-{path_index:04d}.jsonl"
            with shard.open("w", encoding="utf-8") as stream:
                for record in records:
                    json.dump(
                        record,
                        stream,
                        separators=(",", ":"),
                        sort_keys=True,
                        ensure_ascii=False,
                    )
                    stream.write("\n")
            shards[split].append(shard)
        del data, annotations, records_by_split
    return shards, {
        dataset: dict(counts) for dataset, counts in split_report.items()
    }


def _merge_split(
    split: str,
    shards: list[Path],
    temporary: Path,
    config: Config,
    taxonomy: Taxonomy,
    force: bool,
    link_counts: Counter[str],
) -> tuple[dict[str, int], dict[str, Any]]:
    images_fragment = temporary / f"instances-{split}-images.json"
    annotations_fragment = temporary / f"instances-{split}-annotations.json"
    manifest_fragment = temporary / f"proposals-{split}-images.json"
    iterators = [_iter_json_lines(path) for path in shards]
    records = heapq.merge(*iterators, key=_record_sort_key)
    image_count = 0
    annotation_count = 0
    first_image = True
    first_annotation = True
    first_manifest = True
    background_report = derive_trusted_background(
        config, taxonomy, {split: {"images": []}}
    )
    progress = Progress(f"Merging {split} exports", "images")
    with (
        images_fragment.open("w", encoding="utf-8") as image_stream,
        annotations_fragment.open("w", encoding="utf-8") as annotation_stream,
        manifest_fragment.open("w", encoding="utf-8") as manifest_stream,
    ):
        for record in records:
            image_count += 1
            image = record["image"]
            source = Path(image["raw_image_path"]).resolve(strict=True)
            source_dataset = str(image["source_dataset"])
            source_split = str(record["source_split"])
            filename = "__".join(
                (
                    _stable_token(source_dataset),
                    _stable_token(source_split),
                    _stable_token(image["source_image_id"]),
                    _stable_token(image["source_file_name"]),
                )
            )
            destination = config.workspace_root / "output" / "images" / split / filename
            link_counts[link_image(source, destination, "symlink", True, force)] += 1
            camera = (
                "fisheye"
                if source_dataset == "woodscape_rgb_fisheye"
                else "perspective"
            )
            merged_image = {
                "id": image_count,
                "file_name": f"images/{split}/{filename}",
                "width": image["width"],
                "height": image["height"],
                "source_dataset": source_dataset,
                "source_split": source_split,
                "source_image_id": str(image["source_image_id"]),
                "source_sequence_id": str(
                    image.get("source_sequence_id", image["source_image_id"])
                ),
                "source_file_name": image["source_file_name"],
                "camera_type": camera,
                "camera_channel": (
                    Path(image["source_file_name"]).stem
                    if camera == "fisheye"
                    else "unknown"
                ),
                "background_supervision": False,
            }
            first_image = _write_fragment(image_stream, merged_image, first_image)
            prepared = record["annotations"]
            for annotation in prepared:
                annotation["image_id"] = image_count
                annotation.pop("id", None)
            prepared.sort(
                key=lambda annotation: (
                    annotation["category_id"],
                    *map(float, annotation["bbox"]),
                    str(annotation.get("source_annotation_id", "")),
                )
            )
            for annotation in prepared:
                annotation_count += 1
                annotation["id"] = annotation_count
                first_annotation = _write_fragment(
                    annotation_stream, annotation, first_annotation
                )
            one_image = build_manifest(
                {"images": [merged_image], "annotations": prepared},
                split,
                taxonomy,
            )
            manifest_image = one_image["images"][0]
            if manifest_image["trusted_background"]:
                current_report = derive_trusted_background(
                    config, taxonomy, {split: one_image}
                )
                _combine_background_reports(background_report, current_report)
            first_manifest = _write_fragment(
                manifest_stream, manifest_image, first_manifest
            )
            progress.add()
    progress.finish()
    annotations_dir = config.workspace_root / "output" / "annotations"
    _assemble_json(
        annotations_dir / f"instances_{split}.json",
        [
            ("annotations", annotations_fragment, True),
            ("categories", taxonomy.categories, False),
            ("images", images_fragment, True),
        ],
    )
    _assemble_json(
        annotations_dir / f"proposals_{split}.json",
        [
            ("images", manifest_fragment, True),
            ("object_contract", "bounded_promptable_physical_instance", False),
            ("schema_version", SCHEMA_VERSION, False),
            ("split", split, False),
        ],
    )
    return {
        "images": image_count,
        "annotations": annotation_count,
    }, background_report


def merge_exports(
    config: Config, taxonomy: Taxonomy, force: bool = False
) -> dict[str, Any]:
    paths = discover_exports(config.workspace_root / "intermediate" / "coco")
    datasets_with_val = _datasets_with_validation(paths, config)
    link_counts: Counter[str] = Counter()
    results: dict[str, dict[str, int]] = {}
    background_report: dict[str, Any] | None = None
    temporary_parent = config.workspace_root / "intermediate"
    temporary_parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=".merge-", dir=temporary_parent
    ) as temporary_name:
        temporary = Path(temporary_name)
        shards, split_report = _stage_sorted_shards(
            paths, temporary, config, datasets_with_val
        )
        for split in ("train", "val"):
            results[split], split_background = _merge_split(
                split,
                shards[split],
                temporary,
                config,
                taxonomy,
                force,
                link_counts,
            )
            if background_report is None:
                background_report = split_background
            else:
                _combine_background_reports(background_report, split_background)

    write_json(config.reports / "trusted_background.json", background_report)

    stale_links_removed = 0
    if force:
        referenced = set()
        for split in ("train", "val"):
            path = (
                config.workspace_root
                / "output"
                / "annotations"
                / f"instances_{split}.json"
            )
            if path.exists():
                with open(path, "rb") as f:
                    for filename in ijson.items(f, "images.item.file_name"):
                        referenced.add(filename)
        stale_links_removed = prune_unreferenced_image_links(
            config.workspace_root / "output" / "images", referenced
        )

    write_json(
        config.reports / "split_report.json",
        {key: dict(value) for key, value in split_report.items()},
    )
    write_json(
        config.reports / "category_mapping.json",
        {name: category_id for name, category_id in taxonomy.category_ids.items()},
    )
    existing_report = config.reports / "link_report.json"
    previous = read_json(existing_report) if existing_report.exists() else {}
    final_saved = sum(
        path.resolve().stat().st_size
        for path in (config.workspace_root / "output" / "images").rglob("*")
        if path.is_symlink() and path.exists()
    )
    write_json(
        existing_report,
        {
            **previous,
            "final_new_symlinks": link_counts.get("new_symlink", 0),
            "final_reused_symlinks": link_counts.get("reused_symlink", 0),
            "final_storage_saved": final_saved,
            "stale_final_links_removed": stale_links_removed,
            "estimated_storage_saved": previous.get("intermediate_storage_saved", 0)
            + final_saved,
        },
    )
    return results
