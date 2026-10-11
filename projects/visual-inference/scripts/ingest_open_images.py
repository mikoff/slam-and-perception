#!/usr/bin/env python3
"""Ingest targeted Open Images V7 infrastructure instances into quad_train_enriched.sqlite."""

from __future__ import annotations

import argparse
import csv
import io
import json
import logging
import sqlite3
import urllib.request
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any

from PIL import Image

logging.basicConfig(
    level=logging.INFO, format="[%(asctime)s] %(levelname)s: %(message)s"
)
logger = logging.getLogger("ingest_open_images")

TARGET_MIDS: dict[str, str] = {
    # Core Mobility Agents
    "/m/01g317": "person",
    "/m/0k4j": "car",
    "/m/0199g": "bicycle",
    "/m/01bjv": "bus",
    "/m/07r04": "truck",
    "/m/04_sv": "motorcycle",
    "/m/0h2r6": "van",
    # Roadside & Urban Infrastructure
    "/m/015qbp": "parking_meter",
    "/m/01pns0": "fire_hydrant",
    "/m/033rq4": "street_light",
    "/m/015qff": "traffic_light",
    "/m/01mqdt": "traffic_sign",
    "/m/02pv19": "stop_sign",
    "/m/0bjyj5": "waste_container",
    "/m/01knjb": "billboard",
    "/m/018p4k": "cart",
    "/m/0323sq": "golf_cart",
    "/m/0qmmr": "wheelchair",
    "/m/01s55n": "suitcase",
    "/m/0hf58v5": "luggage_and_bags",
    "/m/0cvnqh": "bench",
    "/m/01jfm_": "license_plate",
}

DEFAULT_ARCHIVE_DIR = Path(
    "/media/sashamikoff/fb.com-mikoff/datasets/public/incoming_enrichment/archives/open_images"
)
DEFAULT_WORKSPACE_ROOT = Path(
    "/home/sashamikoff/git/skillup/data/visual-inference-datasets"
)
AWS_S3_BASE_URL = "https://open-images-dataset.s3.amazonaws.com"


def parse_args() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Ingest Open Images target instances into SQLite index."
    )
    parser.add_argument(
        "--archive-dir",
        type=Path,
        default=DEFAULT_ARCHIVE_DIR,
        help="Path to Open Images CSV directory.",
    )
    parser.add_argument(
        "--workspace-root",
        type=Path,
        default=DEFAULT_WORKSPACE_ROOT,
        help="Path to dataset root.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=2000,
        help="Maximum number of positive images to ingest.",
    )
    parser.add_argument(
        "--base-id",
        type=int,
        default=600000,
        help="Starting primary key ID for Open Images namespace.",
    )
    parser.add_argument(
        "--workers", type=int, default=16, help="Concurrent image download workers."
    )
    parser.add_argument(
        "--max-instances-per-class",
        type=int,
        default=50,
        help="Cap repetitive instances per class per image (default: 50).",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing annotations for this dataset namespace in the database.",
    )
    return parser


def download_single_image(
    image_id: str, split: str, dest_path: Path
) -> tuple[str, int, int] | None:
    """Download an image from S3, save to disk, and return (image_id, width, height)."""
    if dest_path.is_file():
        try:
            with Image.open(dest_path) as im:
                w, h = im.size
                return image_id, w, h
        except Exception:
            dest_path.unlink(missing_ok=True)

    url = f"{AWS_S3_BASE_URL}/{split}/{image_id}.jpg"
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "DatasetPipeline/1.0"})
        with urllib.request.urlopen(req, timeout=15) as resp:
            data = resp.read()
        with Image.open(io.BytesIO(data)) as im:
            w, h = im.size
        dest_path.write_bytes(data)
        return image_id, w, h
    except Exception as exc:
        logger.debug("Failed to download image %s (%s): %s", image_id, url, exc)
        return None


def main() -> None:
    args = parse_args().parse_args()

    db_path = args.workspace_root / "output/indexes/quad_train_enriched.sqlite"
    if not db_path.is_file():
        raise FileNotFoundError(f"Enriched database not found: {db_path}")

    raw_img_dir = args.workspace_root / "raw/open_images_subset/img"
    raw_img_dir.mkdir(parents=True, exist_ok=True)
    symlink_dir = args.workspace_root / "output/images/train"
    symlink_dir.mkdir(parents=True, exist_ok=True)

    # Read CSVs and group annotations by image
    csv_files = [
        ("validation", args.archive_dir / "validation-annotations-bbox.csv"),
        ("test", args.archive_dir / "test-annotations-bbox.csv"),
    ]

    annotations_by_image: dict[str, list[dict[str, Any]]] = {}
    image_splits: dict[str, str] = {}

    for split_name, csv_path in csv_files:
        if not csv_path.is_file():
            logger.warning("CSV file missing: %s, skipping split", csv_path)
            continue
        logger.info("Reading annotations from %s...", csv_path.name)
        with open(csv_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                label = row.get("LabelName", "")
                if label not in TARGET_MIDS:
                    continue
                if row.get("IsGroupOf") == "1":
                    continue
                img_id = row["ImageID"]
                if img_id not in annotations_by_image:
                    annotations_by_image[img_id] = []
                    image_splits[img_id] = split_name
                annotations_by_image[img_id].append(row)

    logger.info(
        "Found %d unique images with target annotations.", len(annotations_by_image)
    )

    # Connect to SQLite
    conn = sqlite3.connect(db_path)
    max_row = conn.execute("SELECT COALESCE(MAX(row_index), 0) FROM images").fetchone()[
        0
    ]
    next_row_index = max_row + 1

    existing_images = dict(
        conn.execute(
            "SELECT image_id, file_name FROM images WHERE image_id >= ? AND image_id < ?",
            (args.base_id, args.base_id + 100000),
        ).fetchall()
    )

    if args.overwrite and existing_images:
        logger.info(
            "Overwrite requested: deleting existing annotations for %d images in range [%d, %d)...",
            len(existing_images),
            args.base_id,
            args.base_id + 100000,
        )
        with conn:
            conn.execute(
                "DELETE FROM annotations WHERE image_id >= ? AND image_id < ?",
                (args.base_id, args.base_id + 100000),
            )
        target_existing = {
            Path(r[1]).stem.replace("open_images__", ""): (r[0], r[2], r[3])
            for r in conn.execute(
                "SELECT image_id, file_name, width, height FROM images WHERE image_id >= ? AND image_id < ?",
                (args.base_id, args.base_id + 100000),
            ).fetchall()
        }
    else:
        target_existing = None

    image_batch = []
    image_updates = []
    ann_batch = []
    ingested_count = 0
    total_annotations = 0
    category_counts: dict[str, int] = {v: 0 for v in set(TARGET_MIDS.values())}

    if target_existing is not None:
        logger.info("Re-ingesting %d existing Open Images...", len(target_existing))
        for img_id, (current_image_id, img_w, img_h) in target_existing.items():
            raw_anns = annotations_by_image.get(img_id, [])
            class_instance_counts: dict[str, int] = {}
            extracted_anns = []

            for row in raw_anns:
                mid = row["LabelName"]
                cls_name = TARGET_MIDS[mid]

                class_instance_counts[cls_name] = class_instance_counts.get(cls_name, 0) + 1
                if class_instance_counts[cls_name] > args.max_instances_per_class:
                    continue

                xmin = float(row["XMin"])
                ymin = float(row["YMin"])
                xmax = float(row["XMax"])
                ymax = float(row["YMax"])

                x1 = round(xmin * img_w, 2)
                y1 = round(ymin * img_h, 2)
                x2 = round(xmax * img_w, 2)
                y2 = round(ymax * img_h, 2)

                quad = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
                quad_json = json.dumps(quad)

                extracted_anns.append(
                    (
                        current_image_id,
                        x1,
                        y1,
                        x2,
                        y2,
                        0,  # ignore_region = 0 (foreground)
                        cls_name,
                        "nominal",
                        quad_json,
                        "source_hbb",
                        1.0,
                        1.0,
                        "positive",
                    )
                )
                category_counts[cls_name] += 1

            image_updates.append((len(extracted_anns), current_image_id))
            ann_batch.extend(extracted_anns)
            ingested_count += 1
            total_annotations += len(extracted_anns)

            if ingested_count % 250 == 0:
                logger.info(
                    "Processed %d / %d images (%d annotations)...",
                    ingested_count,
                    len(target_existing),
                    total_annotations,
                )
    else:
        # Prioritize images containing high-value rare infrastructure classes
        class_priority = {
            "parking_meter": 1,
            "fire_hydrant": 2,
            "waste_container": 3,
            "golf_cart": 4,
            "cart": 5,
            "wheelchair": 6,
            "stop_sign": 7,
            "traffic_light": 8,
            "street_light": 9,
            "traffic_sign": 10,
            "billboard": 11,
            "suitcase": 12,
            "luggage_and_bags": 13,
            "bench": 14,
            "license_plate": 20,
        }

        def image_priority_score(img_id: str) -> tuple[int, int]:
            anns = annotations_by_image[img_id]
            min_prio = min(
                class_priority.get(TARGET_MIDS.get(r.get("LabelName", ""), ""), 99)
                for r in anns
            )
            unique_classes = len(
                set(TARGET_MIDS.get(r.get("LabelName", ""), "") for r in anns)
            )
            return (min_prio, -unique_classes)

        sorted_image_ids = sorted(annotations_by_image.keys(), key=image_priority_score)
        selected_image_ids = sorted_image_ids[: args.limit]
        logger.info(
            "Selected %d candidate images prioritizing rare infrastructure (target limit: %d).",
            len(selected_image_ids),
            args.limit,
        )

        # Download images in parallel
        logger.info("Downloading images via %d parallel threads...", args.workers)
        image_dimensions: dict[str, tuple[int, int]] = {}
        with ThreadPoolExecutor(max_workers=args.workers) as executor:
            future_to_id = {
                executor.submit(
                    download_single_image,
                    img_id,
                    image_splits[img_id],
                    raw_img_dir / f"{img_id}.jpg",
                ): img_id
                for img_id in selected_image_ids
            }
            for future in as_completed(future_to_id):
                res = future.result()
                if res is not None:
                    img_id, w, h = res
                    image_dimensions[img_id] = (w, h)
                    if len(image_dimensions) % 250 == 0:
                        logger.info(
                            "Downloaded %d / %d images...",
                            len(image_dimensions),
                            len(selected_image_ids),
                        )

        logger.info("Successfully fetched %d images.", len(image_dimensions))

        existing_ids = set(existing_images.keys())
        for img_id, (img_w, img_h) in image_dimensions.items():
            if ingested_count >= args.limit:
                break

            current_image_id = args.base_id + ingested_count
            if current_image_id in existing_ids:
                ingested_count += 1
                continue

            raw_anns = annotations_by_image[img_id]
            class_instance_counts = {}
            extracted_anns = []

            for row in raw_anns:
                mid = row["LabelName"]
                cls_name = TARGET_MIDS[mid]

                class_instance_counts[cls_name] = class_instance_counts.get(cls_name, 0) + 1
                if class_instance_counts[cls_name] > args.max_instances_per_class:
                    continue

                xmin = float(row["XMin"])
                ymin = float(row["YMin"])
                xmax = float(row["XMax"])
                ymax = float(row["YMax"])

                x1 = round(xmin * img_w, 2)
                y1 = round(ymin * img_h, 2)
                x2 = round(xmax * img_w, 2)
                y2 = round(ymax * img_h, 2)

                quad = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
                quad_json = json.dumps(quad)

                extracted_anns.append(
                    (
                        current_image_id,
                        x1,
                        y1,
                        x2,
                        y2,
                        0,  # ignore_region = 0 (foreground)
                        cls_name,
                        "nominal",
                        quad_json,
                        "source_hbb",
                        1.0,
                        1.0,
                        "positive",
                    )
                )
                category_counts[cls_name] += 1

            if not extracted_anns:
                continue

            symlink_name = f"open_images__{img_id}.jpg"
            symlink_path = symlink_dir / symlink_name
            if not symlink_path.exists():
                rel_target = Path("../../../raw/open_images_subset/img") / f"{img_id}.jpg"
                symlink_path.symlink_to(rel_target)

            image_batch.append(
                (
                    next_row_index,
                    current_image_id,
                    f"images/train/{symlink_name}",
                    img_w,
                    img_h,
                    "open_images_subset",
                    "perspective",
                    1,
                    len(extracted_anns),
                )
            )
            ann_batch.extend(extracted_anns)

            next_row_index += 1
            ingested_count += 1
            total_annotations += len(extracted_anns)

    # Insert batch into SQLite
    logger.info(
        "Writing %d new images, %d updated images, and %d annotations to %s...",
        len(image_batch),
        len(image_updates),
        len(ann_batch),
        db_path.name,
    )
    with conn:
        if image_batch:
            conn.executemany(
                """
                INSERT INTO images (
                    row_index, image_id, file_name, width, height,
                    source_dataset, camera_type, background_supervision, positive_count
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                image_batch,
            )
        if image_updates:
            conn.executemany(
                "UPDATE images SET positive_count = ? WHERE image_id = ?",
                image_updates,
            )
        if ann_batch:
            conn.executemany(
                """
                INSERT INTO annotations (
                    image_id, x1, y1, x2, y2, ignore_region, category_name,
                    object_condition, quad_json, geometry_tier, fit_coverage,
                    fit_tightness, seen_status
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                ann_batch,
            )

    conn.close()

    logger.info("=== Open Images V7 Ingestion Complete ===")
    logger.info("Total images ingested: %d", ingested_count)
    logger.info("Total annotations: %d", total_annotations)
    for cat, cnt in sorted(category_counts.items(), key=lambda x: -x[1]):
        logger.info("  %s: %d instances", cat, cnt)


if __name__ == "__main__":
    main()
