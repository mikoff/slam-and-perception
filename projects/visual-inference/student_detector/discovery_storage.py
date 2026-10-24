"""SQLite transactional streaming storage for SAM discovery pseudo-annotations.

Provides WAL-mode database initialization, atomic batch insertion,
resumable queries, and JSON-to-SQLite migration.
"""

from __future__ import annotations

import json
from pathlib import Path
import sqlite3
from typing import Any

from .sam_student_data import split_cohort_indices


def init_discovery_db(db_path: Path | str) -> sqlite3.Connection:
    """Initialize SQLite database for streaming pseudo-label storage.

    /// @param db_path File path to SQLite database.
    /// @return Active sqlite3 connection with WAL mode and schema configured.
    """
    path = Path(db_path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)

    conn = sqlite3.connect(str(path))
    conn.execute("PRAGMA journal_mode = WAL;")
    conn.execute("PRAGMA synchronous = NORMAL;")
    conn.execute("PRAGMA foreign_keys = ON;")

    with conn:
        conn.execute(
            """
            CREATE TABLE IF NOT EXISTS metadata (
                key TEXT PRIMARY KEY,
                value TEXT
            );
            """
        )
        conn.execute(
            """
            CREATE TABLE IF NOT EXISTS images (
                image_id INTEGER PRIMARY KEY,
                file_name TEXT NOT NULL,
                source_dataset TEXT NOT NULL,
                width INTEGER NOT NULL,
                height INTEGER NOT NULL,
                split TEXT NOT NULL DEFAULT 'train',
                processed_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
            """
        )
        conn.execute(
            """
            CREATE TABLE IF NOT EXISTS annotations (
                annotation_id INTEGER PRIMARY KEY AUTOINCREMENT,
                image_id INTEGER NOT NULL,
                bbox_x1 REAL NOT NULL,
                bbox_y1 REAL NOT NULL,
                bbox_x2 REAL NOT NULL,
                bbox_y2 REAL NOT NULL,
                quad_json TEXT,
                polygon_json TEXT,
                source TEXT NOT NULL,
                category TEXT NOT NULL,
                source_category TEXT,
                is_positive INTEGER NOT NULL,
                is_discovered INTEGER NOT NULL DEFAULT 0,
                is_suppressed INTEGER NOT NULL DEFAULT 0,
                suppression_reason TEXT,
                stability_score REAL,
                predicted_iou REAL,
                matched_gt_id INTEGER,
                matched_gt_category TEXT,
                matched_gt_iou REAL,
                FOREIGN KEY(image_id) REFERENCES images(image_id)
            );
            """
        )
        conn.execute(
            "CREATE INDEX IF NOT EXISTS idx_ann_image ON annotations(image_id);"
        )
        conn.execute(
            "CREATE INDEX IF NOT EXISTS idx_ann_pos ON annotations(image_id, is_positive);"
        )
        conn.execute(
            "CREATE INDEX IF NOT EXISTS idx_images_split ON images(split);"
        )
        conn.execute(
            "CREATE INDEX IF NOT EXISTS idx_images_source ON images(source_dataset);"
        )

    return conn


def get_processed_image_ids(
    db_path: Path | str,
    require_sam_discovered: bool = False,
) -> set[int]:
    """Retrieve the set of image IDs already committed to the database.

    /// @param db_path File path to SQLite database.
    /// @param require_sam_discovered If True, only count images that have at least
    ///     one ``source = 'sam_discovered'`` annotation row (excluding images
    ///     pre-populated with human GT only).
    /// @return Set of processed image IDs.
    """
    path = Path(db_path).resolve()
    if not path.exists():
        return set()

    with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as conn:
        if require_sam_discovered:
            rows = conn.execute(
                "SELECT DISTINCT image_id FROM annotations WHERE source = 'sam_discovered'"
            ).fetchall()
        else:
            rows = conn.execute("SELECT image_id FROM images").fetchall()
        return {int(r[0]) for r in rows}


def save_image_record(
    conn: sqlite3.Connection,
    image_record: dict[str, Any],
    split: str = "train",
) -> None:
    """Save a single image discovery record with its annotations atomically.

    /// @param conn Active sqlite3 connection.
    /// @param image_record Dictionary representing image and discovered annotations.
    /// @param split Split label ('train' or 'val') if not present in record.
    """
    save_image_batch(conn, [image_record], default_split=split)


def save_image_batch(
    conn: sqlite3.Connection,
    image_records: list[dict[str, Any]],
    default_split: str = "train",
) -> None:
    """Save a batch of image discovery records within a single atomic transaction.

    /// @param conn Active sqlite3 connection.
    /// @param image_records List of image discovery records.
    /// @param default_split Fallback split label.
    """
    if not image_records:
        return

    img_rows: list[tuple[Any, ...]] = []
    ann_rows: list[tuple[Any, ...]] = []

    for rec in image_records:
        img_id = int(rec["image_id"])
        split = str(rec.get("split", default_split))
        img_rows.append(
            (
                img_id,
                str(rec["file_name"]),
                str(rec["source_dataset"]),
                int(rec["width"]),
                int(rec["height"]),
                split,
            )
        )

        for ann in rec.get("annotations", []):
            bbox = ann.get("bbox", [0.0, 0.0, 0.0, 0.0])
            quad = ann.get("quad")
            poly = ann.get("polygon")
            quad_str = json.dumps(quad) if quad else None
            poly_str = json.dumps(poly) if poly else None

            ann_rows.append(
                (
                    img_id,
                    float(bbox[0]),
                    float(bbox[1]),
                    float(bbox[2]),
                    float(bbox[3]),
                    quad_str,
                    poly_str,
                    str(ann.get("source", "unknown")),
                    str(ann.get("category", "unknown")),
                    ann.get("source_category"),
                    1 if ann.get("is_positive") else 0,
                    1 if ann.get("is_discovered") else 0,
                    1 if ann.get("is_suppressed") else 0,
                    ann.get("suppression_reason"),
                    ann.get("sam_stability_score"),
                    ann.get("sam_predicted_iou"),
                    ann.get("matched_gt_id"),
                    ann.get("matched_gt_category"),
                    ann.get("matched_gt_iou"),
                )
            )

    with conn:
        conn.executemany(
            """
            INSERT OR REPLACE INTO images (image_id, file_name, source_dataset, width, height, split)
            VALUES (?, ?, ?, ?, ?, ?)
            """,
            img_rows,
        )
        conn.executemany(
            "DELETE FROM annotations WHERE image_id = ?",
            [(r[0],) for r in img_rows],
        )
        conn.executemany(
            """
            INSERT INTO annotations (
                image_id, bbox_x1, bbox_y1, bbox_x2, bbox_y2,
                quad_json, polygon_json, source, category, source_category,
                is_positive, is_discovered, is_suppressed, suppression_reason,
                stability_score, predicted_iou, matched_gt_id, matched_gt_category, matched_gt_iou
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            ann_rows,
        )


def migrate_json_to_sqlite(
    json_path: Path | str,
    db_path: Path | str,
    val_ratio: float = 0.10,
    split_seed: int = 42,
) -> tuple[int, int]:
    """Migrate an existing JSON pseudo-labels file into the SQLite database.

    /// @param json_path Path to JSON file (e.g. sam_pilot_800.json).
    /// @param db_path Path to SQLite output database.
    /// @param val_ratio Ratio of validation images.
    /// @param split_seed Deterministic random seed for cohort splitting.
    /// @return Tuple of (num_train_images, num_val_images).
    """
    json_p = Path(json_path).resolve()
    with open(json_p, encoding="utf-8") as f:
        data = json.load(f)

    images = data["images"]
    train_indices, val_indices = split_cohort_indices(
        images, val_ratio=val_ratio, seed=split_seed
    )
    val_set = set(val_indices)

    conn = init_discovery_db(db_path)

    # Store metadata
    with conn:
        conn.execute(
            "INSERT OR REPLACE INTO metadata (key, value) VALUES (?, ?)",
            ("cohort_sha256", str(data.get("cohort_sha256", ""))),
        )
        conn.execute(
            "INSERT OR REPLACE INTO metadata (key, value) VALUES (?, ?)",
            ("schema_version", str(data.get("schema_version", "v1"))),
        )
        conn.execute(
            "INSERT OR REPLACE INTO metadata (key, value) VALUES (?, ?)",
            ("points_per_side", str(data.get("points_per_side", 24))),
        )

    # Prepare batches
    tagged_records: list[dict[str, Any]] = []
    for idx, img in enumerate(images):
        rec = dict(img)
        rec["split"] = "val" if idx in val_set else "train"
        tagged_records.append(rec)

    # Insert in 100-image chunks
    chunk_size = 100
    for i in range(0, len(tagged_records), chunk_size):
        chunk = tagged_records[i : i + chunk_size]
        save_image_batch(conn, chunk)

    conn.close()
    return len(train_indices), len(val_indices)


def get_dataset_summary(db_path: Path | str) -> dict[str, Any]:
    """Query summary statistics from discovery SQLite database.

    /// @param db_path File path to SQLite database.
    /// @return Dictionary of aggregated statistics.
    """
    path = Path(db_path).resolve()
    with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as conn:
        total_images = conn.execute("SELECT COUNT(*) FROM images").fetchone()[0]
        split_counts = dict(
            conn.execute(
                "SELECT split, COUNT(*) FROM images GROUP BY split"
            ).fetchall()
        )
        source_counts = dict(
            conn.execute(
                "SELECT source_dataset, COUNT(*) FROM images GROUP BY source_dataset"
            ).fetchall()
        )
        total_annotations = conn.execute(
            "SELECT COUNT(*) FROM annotations"
        ).fetchone()[0]
        positive_annotations = conn.execute(
            "SELECT COUNT(*) FROM annotations WHERE is_positive = 1"
        ).fetchone()[0]
        novel_annotations = conn.execute(
            "SELECT COUNT(*) FROM annotations WHERE is_discovered = 1"
        ).fetchone()[0]
        human_gt_annotations = conn.execute(
            "SELECT COUNT(*) FROM annotations WHERE source = 'human_gt'"
        ).fetchone()[0]

    return {
        "total_images": total_images,
        "split_counts": split_counts,
        "source_counts": source_counts,
        "total_annotations": total_annotations,
        "positive_annotations": positive_annotations,
        "novel_annotations": novel_annotations,
        "human_gt_annotations": human_gt_annotations,
    }
