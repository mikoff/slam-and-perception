#!/usr/bin/env python3
"""Synchronize updated human ground truth annotations from quad_train_enriched into sam_discovery_enriched."""

from __future__ import annotations

import argparse
import logging
import sqlite3
from pathlib import Path

logging.basicConfig(
    level=logging.INFO, format="[%(asctime)s] %(levelname)s: %(message)s"
)
logger = logging.getLogger("sync_gt_to_sam_discovery")

DEFAULT_GT_DB = Path(
    "/home/sashamikoff/git/skillup/data/visual-inference-datasets/output/indexes/quad_train_enriched.sqlite"
)
DEFAULT_SAM_DB = Path(
    "/home/sashamikoff/git/skillup/projects/visual-inference/artifacts/pseudo_labels/sam_discovery_enriched.sqlite"
)


def parse_args() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Sync human GT annotations to SAM discovery database."
    )
    parser.add_argument(
        "--gt-db",
        type=Path,
        default=DEFAULT_GT_DB,
        help="Path to authoritative human ground truth database.",
    )
    parser.add_argument(
        "--sam-db",
        type=Path,
        default=DEFAULT_SAM_DB,
        help="Path to SAM discovery database.",
    )
    parser.add_argument(
        "--min-image-id",
        type=int,
        default=500000,
        help="Minimum image_id to sync (default 500000 for enriched cohorts).",
    )
    return parser


def main() -> None:
    args = parse_args().parse_args()

    if not args.gt_db.is_file():
        raise FileNotFoundError(f"GT database not found: {args.gt_db}")
    if not args.sam_db.is_file():
        raise FileNotFoundError(f"SAM discovery database not found: {args.sam_db}")

    logger.info("Opening GT DB: %s", args.gt_db)
    logger.info("Opening SAM DB: %s", args.sam_db)

    gt_conn = sqlite3.connect(f"file:{args.gt_db}?mode=ro&immutable=1", uri=True)
    sam_conn = sqlite3.connect(args.sam_db)

    # 1. Inspect existing human_gt rows in SAM DB
    old_count = sam_conn.execute(
        "SELECT COUNT(*) FROM annotations WHERE source = 'human_gt' AND image_id >= ?",
        (args.min_image_id,),
    ).fetchone()[0]
    logger.info(
        "Found %d existing human_gt annotations in SAM DB for image_id >= %d",
        old_count,
        args.min_image_id,
    )

    # 2. Delete existing human_gt rows for enriched cohorts
    logger.info("Deleting stale human_gt records from SAM DB...")
    with sam_conn:
        sam_conn.execute(
            "DELETE FROM annotations WHERE source = 'human_gt' AND image_id >= ?",
            (args.min_image_id,),
        )

    # 2b. Synchronize image records to maintain referential integrity
    logger.info("Syncing image metadata records from GT DB...")
    gt_images = gt_conn.execute(
        "SELECT image_id, file_name, source_dataset, width, height FROM images WHERE image_id >= ?",
        (args.min_image_id,),
    ).fetchall()
    with sam_conn:
        sam_conn.executemany(
            "INSERT OR REPLACE INTO images (image_id, file_name, source_dataset, width, height, split) VALUES (?, ?, ?, ?, ?, 'train')",
            gt_images,
        )
    logger.info("Synchronized %d image records in SAM DB.", len(gt_images))

    # 3. Stream updated annotations from GT DB
    logger.info("Reading updated annotations from GT DB...")
    gt_cur = gt_conn.cursor()
    gt_cur.execute(
        """
        SELECT image_id, x1, y1, x2, y2, quad_json, category_name, ignore_region
        FROM annotations
        WHERE image_id >= ?
        ORDER BY image_id
        """,
        (args.min_image_id,),
    )

    insert_sql = """
    INSERT INTO annotations (
        image_id, bbox_x1, bbox_y1, bbox_x2, bbox_y2, quad_json, polygon_json,
        source, category, source_category, is_positive, is_discovered,
        is_suppressed, suppression_reason, stability_score, predicted_iou,
        matched_gt_id, matched_gt_category, matched_gt_iou
    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    """

    batch_size = 10000
    batch = []
    total_synced = 0

    while True:
        rows = gt_cur.fetchmany(batch_size)
        if not rows:
            break

        for r in rows:
            image_id, x1, y1, x2, y2, quad_json, cat_name, ign = r
            is_pos = 1 if ign == 0 else 0
            is_supp = 1 if ign != 0 else 0
            supp_reason = None if ign == 0 else "ignore_region"

            batch.append(
                (
                    int(image_id),
                    float(x1),
                    float(y1),
                    float(x2),
                    float(y2),
                    quad_json,
                    None,  # polygon_json
                    "human_gt",
                    str(cat_name),
                    str(cat_name),
                    is_pos,
                    0,  # is_discovered
                    is_supp,
                    supp_reason,
                    None,  # stability_score
                    None,  # predicted_iou
                    None,  # matched_gt_id
                    None,  # matched_gt_category
                    None,  # matched_gt_iou
                )
            )

        with sam_conn:
            sam_conn.executemany(insert_sql, batch)
        total_synced += len(batch)
        logger.info("Synced %d annotations...", total_synced)
        batch = []

    # 4. Verification queries
    logger.info("=== Sync Complete. Verifying SAM DB Stats ===")
    for dataset, min_id, max_id in [
        ("ade20k", 500000, 599999),
        ("open_images_subset", 600000, 699999),
    ]:
        row = sam_conn.execute(
            """
            SELECT COUNT(*), SUM(is_positive)
            FROM annotations
            WHERE source = 'human_gt' AND image_id >= ? AND image_id <= ?
            """,
            (min_id, max_id),
        ).fetchone()
        logger.info("  %s human_gt count: %d (positives: %d)", dataset, row[0], row[1])

    gt_conn.close()
    sam_conn.close()
    logger.info("Total human_gt annotations synced: %d", total_synced)


if __name__ == "__main__":
    main()
