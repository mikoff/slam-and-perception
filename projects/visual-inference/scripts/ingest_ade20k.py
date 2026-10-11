#!/usr/bin/env python3
"""Ingest ADE20K SceneParse150 infrastructure instances into quad_train_enriched.sqlite."""

from __future__ import annotations

import argparse
import io
import json
import logging
import sqlite3
import zipfile
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

logging.basicConfig(
    level=logging.INFO, format="[%(asctime)s] %(levelname)s: %(message)s"
)
logger = logging.getLogger("ingest_ade20k")

# Target ADE20K classes mapped to canonical categories (foreground "things" and mobility agents)
ADE_TARGET_CLASSES: dict[int, str] = {
    # Mobility Agents & Vehicles
    13: "person",
    21: "car",
    81: "bus",
    84: "truck",
    103: "van",
    117: "motorcycle",
    127: "animal",
    128: "bicycle",
    77: "boat",
    91: "airplane",
    104: "boat",
    # Infrastructure & Street Elements
    33: "fence",
    39: "railing",
    42: "box",
    43: "pillar",
    44: "signboard",
    70: "bench",
    87: "canopy",
    88: "street_light",
    89: "kiosk",
    94: "pole",
    107: "canopy",
    112: "barrel",
    115: "tent",
    116: "bag",
    123: "tank",
    124: "signboard",
    133: "sculpture",
    137: "traffic_light",
    139: "trash_bin",
    150: "flag",
    # Common Discrete Physical Objects & Furniture
    8: "bed",
    11: "cabinet",
    15: "door",
    16: "table",
    20: "chair",
    23: "painting",
    24: "sofa",
    25: "shelf",
    28: "mirror",
    31: "chair",
    32: "seat",
    34: "desk",
    36: "wardrobe",
    37: "lamp",
    38: "bathtub",
    40: "cushion",
    41: "stand",
    45: "cabinet",
    46: "counter",
    48: "sink",
    50: "fireplace",
    51: "refrigerator",
    56: "showcase",
    57: "table",
    58: "pillow",
    59: "door",
    63: "shelf",
    65: "table",
    66: "toilet",
    68: "book",
    72: "stove",
    74: "kitchen_island",
    75: "computer",
    76: "chair",
    78: "bar",
    79: "arcade_machine",
    80: "shack",
    82: "towel",
    83: "lamp",
    86: "lamp",
    90: "tv",
    93: "apparel",
    96: "railing",
    97: "escalator",
    98: "seat",
    99: "bottle",
    100: "counter",
    101: "poster",
    105: "fountain",
    108: "washer",
    109: "toy",
    111: "stool",
    113: "basket",
    118: "cradle",
    119: "oven",
    120: "ball",
    125: "microwave",
    126: "pot",
    130: "dishwasher",
    134: "hood",
    135: "lamp",
    136: "vase",
    138: "tray",
    140: "fan",
    143: "plate",
    144: "monitor",
    145: "bulletin_board",
    147: "radiator",
    148: "glass",
    149: "clock",
}

DEFAULT_ARCHIVE = Path(
    "/media/sashamikoff/fb.com-mikoff/datasets/public/incoming_enrichment/archives/ade20k/ADEChallengeData2016.zip"
)
DEFAULT_WORKSPACE_ROOT = Path(
    "/home/sashamikoff/git/skillup/data/visual-inference-datasets"
)


def _order_points_clockwise(points: np.ndarray) -> list[list[float]]:
    """Order 4 polygon points clockwise starting from the top-left-most point."""
    pts = points.astype(np.float32)
    # Center of mass
    center = np.mean(pts, axis=0)
    # Angles relative to center
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    sorted_indices = np.argsort(angles)
    sorted_pts = pts[sorted_indices]

    # Find the top-left point (smallest x + y)
    s = sorted_pts.sum(axis=1)
    tl_idx = int(np.argmin(s))
    # Roll so top-left is first
    ordered = np.roll(sorted_pts, -tl_idx, axis=0)

    # Check signed area (shoelace) to verify clockwise vs counter-clockwise
    area = 0.5 * sum(
        ordered[i, 0] * ordered[(i + 1) % 4, 1]
        - ordered[i, 1] * ordered[(i + 1) % 4, 0]
        for i in range(4)
    )
    if area < 0:  # If negative, invert ordering of remaining 3 points
        ordered = np.array([ordered[0], ordered[3], ordered[2], ordered[1]])

    return [[round(float(p[0]), 2), round(float(p[1]), 2)] for p in ordered]


def parse_args() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Ingest ADE20K target instances into SQLite index."
    )
    parser.add_argument(
        "--archive",
        type=Path,
        default=DEFAULT_ARCHIVE,
        help="Path to ADE20K zip archive.",
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
        "--min-area",
        type=float,
        default=64.0,
        help="Minimum pixel area for target instances.",
    )
    parser.add_argument(
        "--base-id",
        type=int,
        default=500000,
        help="Starting primary key ID for ADE20K namespace.",
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


def main() -> None:
    args = parse_args().parse_args()

    if not args.archive.is_file():
        raise FileNotFoundError(f"ADE20K archive not found: {args.archive}")

    db_path = args.workspace_root / "output/indexes/quad_train_enriched.sqlite"
    if not db_path.is_file():
        raise FileNotFoundError(f"Enriched database not found: {db_path}")

    raw_img_dir = args.workspace_root / "raw/ade20k/img"
    raw_img_dir.mkdir(parents=True, exist_ok=True)
    symlink_dir = args.workspace_root / "output/images/train"
    symlink_dir.mkdir(parents=True, exist_ok=True)

    logger.info("Opening ADE20K archive: %s", args.archive)
    zf = zipfile.ZipFile(args.archive, "r")

    # Find training annotation files
    ann_names = sorted(
        [
            n
            for n in zf.namelist()
            if n.startswith("ADEChallengeData2016/annotations/training/")
            and n.endswith(".png")
        ]
    )
    logger.info("Found %d annotation files in ADE20K training split.", len(ann_names))

    # Connect to SQLite
    conn = sqlite3.connect(db_path)
    # Check existing max row_index
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
        target_work = [
            (
                img_id,
                f"ADEChallengeData2016/annotations/training/{Path(fn).stem.replace('ade20k__', '')}.png",
            )
            for img_id, fn in sorted(existing_images.items())
        ]
        existing_ids = set()
    else:
        existing_ids = set(existing_images.keys())
        target_work = None

    ingested_count = 0
    total_annotations = 0
    category_counts: dict[str, int] = {v: 0 for v in set(ADE_TARGET_CLASSES.values())}

    image_batch = []
    image_updates = []
    ann_batch = []

    if target_work is not None:
        logger.info("Re-ingesting %d existing ADE20K images...", len(target_work))
        for current_image_id, ann_path in target_work:
            if ann_path not in zf.namelist():
                continue

            ann_bytes = zf.read(ann_path)
            mask = np.array(Image.open(io.BytesIO(ann_bytes)))
            unique_classes = set(np.unique(mask))

            matching_classes = unique_classes.intersection(ADE_TARGET_CLASSES.keys())
            extracted_anns = []

            for cls_id in matching_classes:
                cls_name = ADE_TARGET_CLASSES[cls_id]
                binary_mask = (mask == cls_id).astype(np.uint8)

                contours, _ = cv2.findContours(
                    binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                class_instances = 0

                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area < args.min_area:
                        continue

                    x, y, w, h = cv2.boundingRect(cnt)
                    x1, y1, x2, y2 = float(x), float(y), float(x + w), float(y + h)

                    rect = cv2.minAreaRect(cnt)
                    box_pts = cv2.boxPoints(rect)
                    quad = _order_points_clockwise(box_pts)
                    quad_json = json.dumps(quad)

                    extracted_anns.append(
                        (
                            current_image_id,
                            x1,
                            y1,
                            x2,
                            y2,
                            0,  # ignore_region = 0 (positive foreground)
                            cls_name,
                            "nominal",
                            quad_json,
                            "quad" if cls_name in ("pillar", "canopy") else "source_hbb",
                            1.0,
                            1.0,
                            "positive",
                        )
                    )
                    category_counts[cls_name] += 1
                    class_instances += 1
                    if class_instances >= args.max_instances_per_class:
                        break

            image_updates.append((len(extracted_anns), current_image_id))
            ann_batch.extend(extracted_anns)
            ingested_count += 1
            total_annotations += len(extracted_anns)

            if ingested_count % 250 == 0:
                logger.info(
                    "Processed %d / %d images (%d annotations)...",
                    ingested_count,
                    len(target_work),
                    total_annotations,
                )
    else:
        for ann_path in ann_names:
            if ingested_count >= args.limit:
                break

            base_stem = Path(ann_path).stem  # e.g. ADE_train_00015082
            image_subpath = f"ADEChallengeData2016/images/training/{base_stem}.jpg"
            if image_subpath not in zf.namelist():
                continue

            # Load annotation mask
            ann_bytes = zf.read(ann_path)
            mask = np.array(Image.open(io.BytesIO(ann_bytes)))
            unique_classes = set(np.unique(mask))

            matching_classes = unique_classes.intersection(ADE_TARGET_CLASSES.keys())
            if not matching_classes:
                continue

            current_image_id = args.base_id + ingested_count
            if current_image_id in existing_ids:
                ingested_count += 1
                continue

            img_h, img_w = mask.shape[:2]
            extracted_anns = []

            for cls_id in matching_classes:
                cls_name = ADE_TARGET_CLASSES[cls_id]
                binary_mask = (mask == cls_id).astype(np.uint8)

                contours, _ = cv2.findContours(
                    binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                class_instances = 0

                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area < args.min_area:
                        continue

                    x, y, w, h = cv2.boundingRect(cnt)
                    x1, y1, x2, y2 = float(x), float(y), float(x + w), float(y + h)

                    # Fit 4-point oriented rectangle
                    rect = cv2.minAreaRect(cnt)
                    box_pts = cv2.boxPoints(rect)
                    quad = _order_points_clockwise(box_pts)
                    quad_json = json.dumps(quad)

                    extracted_anns.append(
                        (
                            current_image_id,
                            x1,
                            y1,
                            x2,
                            y2,
                            0,  # ignore_region = 0 (positive foreground)
                            cls_name,
                            "nominal",
                            quad_json,
                            "quad" if cls_name in ("pillar", "canopy") else "source_hbb",
                            1.0,
                            1.0,
                            "positive",
                        )
                    )
                    category_counts[cls_name] += 1
                    class_instances += 1
                    if class_instances >= args.max_instances_per_class:
                        break

            if not extracted_anns:
                continue

            # Extract image to local raw directory
            raw_img_path = raw_img_dir / f"{base_stem}.jpg"
            if not raw_img_path.is_file():
                raw_img_path.write_bytes(zf.read(image_subpath))

            # Create symlink in output/images/train/
            symlink_name = f"ade20k__{base_stem}.jpg"
            symlink_path = symlink_dir / symlink_name
            if not symlink_path.exists():
                rel_target = Path("../../../raw/ade20k/img") / f"{base_stem}.jpg"
                symlink_path.symlink_to(rel_target)

            # Image record: (row_index, image_id, file_name, width, height, source_dataset, camera_type, background_supervision, positive_count)
            image_batch.append(
                (
                    next_row_index,
                    current_image_id,
                    f"images/train/{symlink_name}",
                    img_w,
                    img_h,
                    "ade20k",
                    "perspective",
                    1,  # background_supervision = 1
                    len(extracted_anns),
                )
            )
            ann_batch.extend(extracted_anns)

            next_row_index += 1
            ingested_count += 1
            total_annotations += len(extracted_anns)

            if ingested_count % 250 == 0:
                logger.info(
                    "Processed %d / %d images (%d annotations)...",
                    ingested_count,
                    args.limit,
                    total_annotations,
                )

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
    zf.close()

    logger.info("=== ADE20K Ingestion Complete ===")
    logger.info("Total images ingested: %d", ingested_count)
    logger.info("Total annotations: %d", total_annotations)
    for cat, cnt in sorted(category_counts.items(), key=lambda x: -x[1]):
        logger.info("  %s: %d instances", cat, cnt)


if __name__ == "__main__":
    main()
