#!/usr/bin/env python3
"""Build a small evaluation-only manifest without modifying training databases."""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import io
import json
from pathlib import Path
import random
import sqlite3
import urllib.request
import zipfile
from typing import Any

import cv2
import numpy as np
from PIL import Image

from ingest_ade20k import ADE_TARGET_CLASSES, DEFAULT_ARCHIVE
from ingest_open_images import DEFAULT_ARCHIVE_DIR, TARGET_MIDS
from ingest_roboflow_ev_parking import DEFAULT_SAMPLES_JSON, HF_BASE_URL
from student_detector.dashboard_holdout import (
    audit_records,
    file_sha256,
    training_inventory,
    validation_inventory,
)
from student_detector.human_gt_data import DEFAULT_ENRICHED_GT_INDEX_PATH
from student_detector.sam_student_data import DEFAULT_IMAGE_ROOT


def rectangle(box: list[float]) -> list[list[float]]:
    """Convert an xyxy box to its four corners."""
    x1, y1, x2, y2 = box
    return [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]


def download(url: str, path: Path) -> None:
    """Cache a single image in the evaluation directory."""
    if not path.exists():
        request = urllib.request.Request(
            url, headers={"User-Agent": "DatasetPipeline/1.0"}
        )
        with urllib.request.urlopen(request, timeout=30) as response:
            content = response.read()
        with Image.open(io.BytesIO(content)) as image:
            image.verify()
        path.write_bytes(content)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gt-index", type=Path, default=DEFAULT_ENRICHED_GT_INDEX_PATH)
    parser.add_argument("--image-root", type=Path, default=DEFAULT_IMAGE_ROOT)
    parser.add_argument("--cohort-size", type=int, default=14000)
    parser.add_argument(
        "--validation-pseudo-labels",
        type=Path,
        default=Path("artifacts/phase1/pseudo_labels/sam_pilot_800.json"),
    )
    parser.add_argument("--samples-per-cohort", type=int, default=35)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--include-new-sources",
        action="store_true",
        help="Extract/download small ADE20K, Open Images and PKLot evaluation sets.",
    )
    args = parser.parse_args()
    if args.samples_per_cohort < 1:
        parser.error("--samples-per-cohort must be positive")
    print(
        "Hashing the complete Stage-1 cohort (also excludes Stage-2 train/val)...",
        flush=True,
    )
    training = training_inventory(args.gt_index, args.image_root, args.cohort_size)
    validation = validation_inventory(args.validation_pseudo_labels, args.image_root)
    banned_ids = {r["image_id"] for r in training}
    banned_hashes = {r["sha256"] for r in training}
    used_hashes: set[str] = set()
    used_names = {Path(r["file_name"]).name for r in training}
    rng = random.Random(42)
    records: list[dict[str, Any]] = []
    counts: dict[str, int] = defaultdict(int)
    storage = DEFAULT_ARCHIVE_DIR.parent.parent / "evaluation/dashboard_holdout"
    storage.mkdir(parents=True, exist_ok=True)

    def add(
        path: Path,
        source: str,
        quads: list[list[list[float]]],
        iid: int | None = None,
        camera: str = "pinhole",
    ) -> bool:
        if not quads or counts[source] >= args.samples_per_cohort:
            return False
        digest = file_sha256(path)
        if digest in banned_hashes or digest in used_hashes:
            return False
        with Image.open(path) as image:
            width, height = image.size
        boxes = [
            [
                min(p[0] for p in q),
                min(p[1] for p in q),
                max(p[0] for p in q),
                max(p[1] for p in q),
            ]
            for q in quads
        ]
        records.append(
            dict(
                image_id=iid if iid is not None else -len(records) - 1,
                file_name=str(path.resolve()),
                source_dataset=source,
                camera_type=camera,
                width=width,
                height=height,
                gt_quads=quads,
                gt_boxes=boxes,
                sha256=digest,
            )
        )
        used_hashes.add(digest)
        counts[source] += 1
        return True

    with sqlite3.connect(f"file:{args.gt_index}?mode=ro", uri=True) as conn:
        by_source: dict[str, list[tuple[int, str, str]]] = defaultdict(list)
        for iid, name, source, camera in conn.execute(
            "SELECT image_id,file_name,source_dataset,camera_type FROM images ORDER BY image_id"
        ):
            if iid not in banned_ids:
                by_source[source].append((iid, name, camera))
        for source, candidates in sorted(by_source.items()):
            rng.shuffle(candidates)
            for iid, name, camera in candidates:
                path = args.image_root / name
                if not path.is_file():
                    continue
                quads = [
                    json.loads(r[0])
                    for r in conn.execute(
                        "SELECT quad_json FROM annotations WHERE image_id=? AND ignore_region=0",
                        (iid,),
                    )
                ]
                add(path, source, quads, iid, camera)
                if counts[source] >= args.samples_per_cohort:
                    break
            print(source, counts[source], flush=True)

    if args.include_new_sources:
        # ADE20K's official validation partition was never ingested for training.
        with zipfile.ZipFile(DEFAULT_ARCHIVE) as archive:
            names = sorted(
                n
                for n in archive.namelist()
                if "/annotations/validation/" in n and n.endswith(".png")
            )
            rng.shuffle(names)
            for name in names:
                mask = np.array(Image.open(io.BytesIO(archive.read(name))))
                quads = []
                for cls in sorted(set(np.unique(mask)) & ADE_TARGET_CLASSES.keys()):
                    contours, _ = cv2.findContours(
                        (mask == cls).astype(np.uint8),
                        cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE,
                    )
                    for contour in contours:
                        if cv2.contourArea(contour) >= 64:
                            x, y, w, h = cv2.boundingRect(contour)
                            quads.append(rectangle([x, y, x + w, y + h]))
                if not quads:
                    continue
                image_name = name.replace("/annotations/", "/images/").replace(
                    ".png", ".jpg"
                )
                path = storage / Path(image_name).name
                path.write_bytes(archive.read(image_name))
                add(path, "ade20k", quads)
                if counts["ade20k"] >= args.samples_per_cohort:
                    break
        print("ade20k", counts["ade20k"], flush=True)

        # Include both close-up cars and a random mix of the curated target classes.
        oi: dict[str, list[dict[str, str]]] = defaultdict(list)
        with (DEFAULT_ARCHIVE_DIR / "validation-annotations-bbox.csv").open() as stream:
            for row in csv.DictReader(stream):
                if row["LabelName"] in TARGET_MIDS and row["IsGroupOf"] != "1":
                    if f"open_images__{row['ImageID']}.jpg" not in used_names:
                        oi[row["ImageID"]].append(row)
        large_cars = [
            iid
            for iid, rows in oi.items()
            if any(
                r["LabelName"] == "/m/0k4j"
                and (float(r["XMax"]) - float(r["XMin"]))
                * (float(r["YMax"]) - float(r["YMin"]))
                >= 0.3
                for r in rows
            )
        ]
        rng.shuffle(large_cars)
        candidates = sorted(oi)
        rng.shuffle(candidates)
        candidates = list(
            dict.fromkeys(large_cars[: min(10, args.samples_per_cohort)] + candidates)
        )
        failures = 0
        for iid in candidates:
            path = storage / f"open_images__{iid}.jpg"
            try:
                download(
                    f"https://open-images-dataset.s3.amazonaws.com/validation/{iid}.jpg",
                    path,
                )
            except (OSError, ValueError) as exc:
                print(f"Download failed: {iid}: {exc}", flush=True)
                failures += 1
                if failures >= 10:
                    raise RuntimeError(
                        "Too many Open Images download failures"
                    ) from exc
                continue
            with Image.open(path) as image:
                w, h = image.size
            quads = [
                rectangle(
                    [
                        float(r["XMin"]) * w,
                        float(r["YMin"]) * h,
                        float(r["XMax"]) * w,
                        float(r["YMax"]) * h,
                    ]
                )
                for r in oi[iid]
            ]
            add(path, "open_images_subset", quads)
            if counts["open_images_subset"] >= args.samples_per_cohort:
                break
        print("open_images_subset", counts["open_images_subset"], flush=True)

        samples = json.loads(DEFAULT_SAMPLES_JSON.read_text())["samples"]
        candidates = [
            r
            for r in samples
            if r.get("source") in ("ufpr04", "ufpr05")
            and r.get("parking_spaces", {}).get("polylines")
            and f"parking__{Path(r['filepath']).name}" not in used_names
        ]
        rng.shuffle(candidates)
        failures = 0
        for row in candidates:
            path = storage / f"parking__{row['source']}__{Path(row['filepath']).name}"
            try:
                download(f"{HF_BASE_URL}/{row['filepath']}", path)
            except (OSError, ValueError) as exc:
                print(f"Download failed: {row['filepath']}: {exc}", flush=True)
                failures += 1
                if failures >= 10:
                    raise RuntimeError("Too many PKLot download failures") from exc
                continue
            with Image.open(path) as image:
                w, h = image.size
            quads = [
                [[p[0] * w, p[1] * h] for p in poly["points"][0]]
                for poly in row["parking_spaces"]["polylines"]
                if poly.get("points") and len(poly["points"][0]) == 4
            ]
            add(path, "roboflow_ev_parking", quads)
            if counts["roboflow_ev_parking"] >= args.samples_per_cohort:
                break
        print("roboflow_ev_parking", counts["roboflow_ev_parking"], flush=True)

    audit_records(records, training + validation, args.image_root)
    manifest = dict(
        gt_index=str(args.gt_index.resolve()),
        cohort_size=args.cohort_size,
        excluded_training_images=training,
        validation_pseudo_labels=str(args.validation_pseudo_labels.resolve()),
        excluded_validation_images=validation,
        images=records,
        counts=dict(counts),
        seed=42,
        notes="Evaluation only; original training databases unchanged. Exact duplicates of detector training and validation inputs are excluded; near-duplicate scenes are not audited. ADE20K boxes are derived from semantic components; Open Images target annotations are partial.",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2))
    print(
        f"Saved {len(records)} audited evaluation images to {args.output}", flush=True
    )


if __name__ == "__main__":
    main()
