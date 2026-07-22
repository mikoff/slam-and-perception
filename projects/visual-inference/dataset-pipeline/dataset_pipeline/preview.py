from __future__ import annotations

import html
import random
from collections import defaultdict
from pathlib import Path

from PIL import Image, ImageDraw

from .config import Config
from .reports import read_json


def generate_previews(config: Config) -> dict[str, int]:
    candidates = []
    category_names = {}
    category_frequency = defaultdict(int)
    for split in ("train", "val"):
        path = config.workspace_root / "output" / "annotations" / f"instances_{split}.json"
        data = read_json(path)
        category_names.update({c["id"]: c["name"] for c in data["categories"]})
        by_image = defaultdict(list)
        for annotation in data["annotations"]:
            by_image[annotation["image_id"]].append(annotation)
            category_frequency[annotation["category_id"]] += 1
        for image in data["images"]:
            annotations = by_image[image["id"]]
            if annotations:
                areas = [a["area"] / (image["width"] * image["height"]) for a in annotations]
                size = "small" if min(areas) < 0.01 else ("large" if max(areas) > 0.25 else "medium")
                candidates.append((image["source_dataset"], split, image["camera_type"], size, image, annotations))
    rng = random.Random(int(config.validation["random_seed"]))
    rng.shuffle(candidates)
    limit = min(int(config.validation["preview_count"]), len(candidates))
    if limit < min(100, len(candidates)):
        limit = min(100, len(candidates))
    frequencies = sorted(category_frequency.values())
    rare_cutoff = frequencies[max(0, len(frequencies) // 3 - 1)] if frequencies else 0
    buckets = defaultdict(list)
    for candidate in candidates:
        dataset, split, camera, size_group, _, annotations = candidate
        rare_or_frequent = "rare" if min(category_frequency[a["category_id"]] for a in annotations) <= rare_cutoff else "frequent"
        buckets[(dataset, split, camera, size_group, rare_or_frequent)].append(candidate)
    chosen = []
    keys = sorted(buckets)
    while len(chosen) < limit and keys:
        next_keys = []
        for key in keys:
            if buckets[key] and len(chosen) < limit:
                chosen.append(buckets[key].pop())
            if buckets[key]:
                next_keys.append(key)
        keys = next_keys
    entries = []
    for dataset, split, camera, size_group, info, annotations in chosen:
        source = config.workspace_root / "output" / info["file_name"]
        resolved = source.resolve(strict=True)
        if not resolved.is_file():
            raise ValueError(f"Preview source is invalid: {source}")
        with Image.open(resolved) as loaded:
            if loaded.size != (info["width"], info["height"]):
                raise ValueError(f"Preview dimension mismatch: {source}")
            image = loaded.convert("RGB")
        draw = ImageDraw.Draw(image)
        labels = []
        for annotation in annotations:
            x, y, width, height = annotation["bbox"]
            canonical = category_names[annotation["category_id"]]
            label = f"{canonical} <- {annotation['source_category']} [{dataset}] #{annotation['id']}"
            labels.append(label)
            draw.rectangle((x, y, x + width, y + height), outline=(255, 40, 40), width=max(2, image.width // 600))
            draw.text((x + 2, max(0, y - 12)), label, fill=(255, 40, 40), stroke_width=2, stroke_fill=(255, 255, 255))
        filename = f"{dataset}__{info['id']:08d}.jpg"
        output = config.reports / "previews" / split / filename
        output.parent.mkdir(parents=True, exist_ok=True)
        image.save(output, quality=90)
        entries.append((dataset, split, output, labels))
    groups = defaultdict(list)
    for dataset, split, path, labels in entries:
        groups[(dataset, split)].append((path, labels))
    parts = ["<!doctype html><meta charset='utf-8'><title>Dataset previews</title><h1>Dataset previews</h1>"]
    for (dataset, split), values in sorted(groups.items()):
        parts.append(f"<h2>{html.escape(dataset)} / {html.escape(split)}</h2><div>")
        for path, labels in values:
            relative = path.relative_to(config.reports / "previews")
            parts.append(
                f"<a href='{relative.as_posix()}'><img loading='lazy' width='320' src='{relative.as_posix()}' "
                f"title='{html.escape('; '.join(labels), quote=True)}'></a>"
            )
        parts.append("</div>")
    (config.reports / "previews" / "index.html").write_text("\n".join(parts) + "\n", encoding="utf-8")
    return {"previews": len(entries)}
