#!/usr/bin/env python3
"""Ingest parking facility and marked slot instances into quad_train_enriched.sqlite."""

from __future__ import annotations

import argparse
import io
import json
import logging
import sqlite3
import urllib.request
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import numpy as np
from PIL import Image

logging.basicConfig(
    level=logging.INFO, format="[%(asctime)s] %(levelname)s: %(message)s"
)
logger = logging.getLogger("ingest_roboflow_ev_parking")

DEFAULT_SAMPLES_JSON = Path(
    "/media/sashamikoff/fb.com-mikoff/datasets/public/incoming_enrichment/archives/roboflow/samples.json"
)
DEFAULT_WORKSPACE_ROOT = Path(
    "/home/sashamikoff/git/skillup/data/visual-inference-datasets"
)
HF_BASE_URL = "https://huggingface.co/datasets/Voxel51/PKLot/resolve/main"


def _order_points_clockwise(points: list[list[float]]) -> list[list[float]]:
    """Order 4 polygon points clockwise starting from top-left."""
    pts = np.array(points, dtype=np.float32)
    center = np.mean(pts, axis=0)
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    sorted_indices = np.argsort(angles)
    sorted_pts = pts[sorted_indices]

    s = sorted_pts.sum(axis=1)
    tl_idx = int(np.argmin(s))
    ordered = np.roll(sorted_pts, -tl_idx, axis=0)

    area = 0.5 * sum(
        ordered[i, 0] * ordered[(i + 1) % 4, 1]
        - ordered[i, 1] * ordered[(i + 1) % 4, 0]
        for i in range(4)
    )
    if area < 0:
        ordered = np.array([ordered[0], ordered[3], ordered[2], ordered[1]])

    return [[round(float(p[0]), 2), round(float(p[1]), 2)] for p in ordered]


def parse_args() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Ingest parking slot dataset into SQLite index."
    )
    parser.add_argument(
        "--samples-json",
        type=Path,
        default=DEFAULT_SAMPLES_JSON,
        help="Path to samples.json.",
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
        default=2500,
        help="Maximum number of positive images to ingest.",
    )
    parser.add_argument(
        "--base-id",
        type=int,
        default=700000,
        help="Starting primary key ID for parking namespace.",
    )
    parser.add_argument(
        "--workers", type=int, default=16, help="Concurrent image download workers."
    )
    parser.add_argument(
        "--max-slots-per-image",
        type=int,
        default=None,
        help="Cap parking slots per image (default: None for all slots).",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing annotations for this dataset namespace in the database.",
    )
    return parser


def download_single_image(
    filepath: str, dest_path: Path
) -> tuple[str, int, int] | None:
    """Download an image from HF, save to disk, and return (filepath, width, height)."""
    if dest_path.is_file():
        try:
            with Image.open(dest_path) as im:
                w, h = im.size
                return filepath, w, h
        except Exception:
            dest_path.unlink(missing_ok=True)

    url = f"{HF_BASE_URL}/{filepath}"
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "DatasetPipeline/1.0"})
        with urllib.request.urlopen(req, timeout=20) as resp:
            data = resp.read()
        with Image.open(io.BytesIO(data)) as im:
            w, h = im.size
        dest_path.write_bytes(data)
        return filepath, w, h
    except Exception as exc:
        logger.debug("Failed to download image %s (%s): %s", filepath, url, exc)
        return None


def main() -> None:
    args = parse_args().parse_args()

    if not args.samples_json.is_file():
        raise FileNotFoundError(f"samples.json not found: {args.samples_json}")

    db_path = args.workspace_root / "output/indexes/quad_train_enriched.sqlite"
    if not db_path.is_file():
        raise FileNotFoundError(f"Enriched database not found: {db_path}")

    raw_img_dir = args.workspace_root / "raw/roboflow_ev_parking/img"
    raw_img_dir.mkdir(parents=True, exist_ok=True)
    symlink_dir = args.workspace_root / "output/images/train"
    symlink_dir.mkdir(parents=True, exist_ok=True)

    logger.info("Loading samples from %s...", args.samples_json.name)
    with open(args.samples_json, "r", encoding="utf-8") as f:
        data = json.load(f)

    all_samples = data.get("samples", [])
    logger.info("Loaded %d total samples from dataset.", len(all_samples))

    # Select samples with valid parking spaces, taking diverse weather and lot captures
    valid_samples = [
        s
        for s in all_samples
        if s.get("parking_spaces", {}).get("polylines") and s.get("filepath")
    ]
    # Sample every Nth to get wide coverage across days and weather conditions
    step = max(1, len(valid_samples) // args.limit)
    selected_samples = valid_samples[::step][: args.limit]
    logger.info(
        "Selected %d diverse candidate images (stride %d, target limit: %d).",
        len(selected_samples),
        step,
        args.limit,
    )

    # Download images in parallel
    logger.info("Downloading images via %d parallel threads...", args.workers)
    image_dimensions: dict[str, tuple[int, int]] = {}
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        future_to_path = {}
        for s in selected_samples:
            fp = s["filepath"]
            fn = Path(fp).name
            future_to_path[
                executor.submit(download_single_image, fp, raw_img_dir / fn)
            ] = fp

        for future in as_completed(future_to_path):
            res = future.result()
            if res is not None:
                fp, w, h = res
                image_dimensions[fp] = (w, h)
                if len(image_dimensions) % 250 == 0:
                    logger.info(
                        "Downloaded %d / %d images...",
                        len(image_dimensions),
                        len(selected_samples),
                    )

    logger.info("Successfully fetched %d images.", len(image_dimensions))

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
        existing_ids = set()
    else:
        existing_ids = set(existing_images.keys())

    image_batch = []
    image_updates = []
    ann_batch = []
    ingested_count = 0
    total_annotations = 0

    for s in selected_samples:
        if ingested_count >= args.limit:
            break

        fp = s["filepath"]
        if fp not in image_dimensions:
            continue

        img_w, img_h = image_dimensions[fp]
        base_name = Path(fp).name

        current_image_id = args.base_id + ingested_count
        if current_image_id in existing_ids:
            ingested_count += 1
            continue

        polylines = s.get("parking_spaces", {}).get("polylines", [])
        extracted_anns = []

        for pl in polylines:
            points_list = pl.get("points", [])
            if not points_list or len(points_list[0]) < 4:
                continue

            # First polygon ring
            raw_pts = points_list[0][:4]
            # Convert normalized coordinates to pixel space
            pixel_pts = [
                [round(p[0] * img_w, 2), round(p[1] * img_h, 2)] for p in raw_pts
            ]

            xs = [p[0] for p in pixel_pts]
            ys = [p[1] for p in pixel_pts]
            x1, y1 = max(0.0, min(xs)), max(0.0, min(ys))
            x2, y2 = min(float(img_w), max(xs)), min(float(img_h), max(ys))

            if (x2 - x1) < 8 or (y2 - y1) < 8:
                continue

            quad = _order_points_clockwise(pixel_pts)
            quad_json = json.dumps(quad)

            extracted_anns.append(
                (
                    current_image_id,
                    x1,
                    y1,
                    x2,
                    y2,
                    0,  # ignore_region = 0
                    "parking_slot",
                    "nominal",
                    quad_json,
                    "quad",  # geometry_tier = quad
                    1.0,
                    1.0,
                    "positive",
                )
            )

            if (
                args.max_slots_per_image is not None
                and len(extracted_anns) >= args.max_slots_per_image
            ):
                break

        if not extracted_anns:
            continue

        symlink_name = f"parking__{base_name}"
        symlink_path = symlink_dir / symlink_name
        if not symlink_path.exists():
            rel_target = Path("../../../raw/roboflow_ev_parking/img") / base_name
            symlink_path.symlink_to(rel_target)

        if current_image_id in existing_images:
            image_updates.append((len(extracted_anns), current_image_id))
        else:
            image_batch.append(
                (
                    next_row_index,
                    current_image_id,
                    f"images/train/{symlink_name}",
                    img_w,
                    img_h,
                    "roboflow_ev_parking",
                    "perspective",
                    1,
                    len(extracted_anns),
                )
            )
            next_row_index += 1

        ann_batch.extend(extracted_anns)
        ingested_count += 1
        total_annotations += len(extracted_anns)

    # Insert / update batch in SQLite
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

    logger.info("=== Parking Slot Ingestion Complete ===")
    logger.info("Total images ingested: %d", ingested_count)
    logger.info("Total annotations: %d (geometry_tier: quad)", total_annotations)


if __name__ == "__main__":
    main()
