#!/usr/bin/env python3
"""Patch missing bollards (-> pole) and architectural columns (-> pillar) into an OWLv2 cache.

Clones `--source-db` via `sqlite3.backup` into a new WAL/DELETE-safe database (`--output-db`)
using an atomic temp file, then runs OWLv2-Large on all non-ADE20K images with targeted
bollard & column prompts (`OWLV2_PATCH_BOLLARD_PILLAR_PROMPTS`) and appends surviving boxes
and FP16 768-D embeddings without altering GT annotations or existing OWLv2 boxes.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from collections.abc import Sequence
from datetime import datetime
import json
import os
from pathlib import Path
import sqlite3
import time
from typing import Any

import numpy as np
from PIL import Image
import torch
from torch.utils.data import DataLoader, Dataset
from transformers import Owlv2ImageProcessor

if hasattr(torch.multiprocessing, "set_sharing_strategy"):
    torch.multiprocessing.set_sharing_strategy("file_system")

from student_detector.annotation_hygiene import RawAnnotationRow, sanitize_image_annotations
from student_detector.human_gt_data import (
    DEFAULT_ENRICHED_GT_INDEX_PATH,
    HumanGTHBBDataset,
)
from student_detector.owlv2_teacher import (
    OWLV2_MODEL_ID,
    OWLV2_MODEL_REVISION,
    OWLV2_PATCH_BOLLARD_PILLAR_PROMPTS,
    OwlDetection,
    OwlTeacherCacheReader,
    OwlTeacherCacheWriter,
    Owlv2TeacherExtractor,
    canonicalize_gt_labels,
    merge_patch_owl_detections,
)
from student_detector.sam_student_data import DEFAULT_IMAGE_ROOT

PATCH_NAME: str = "bollard_pillar_v1"
DATASET_ORDER: tuple[str, ...] = (
    "nuimages",
    "bdd100k_images_100k",
    "woodscape_rgb_fisheye",
    "ade20k",
    "open_images_subset",
    "coco_2017",
)


class _PatchImageDataset(Dataset[dict[str, Any]]):
    """Worker-friendly RGB image loader + Owlv2ImageProcessor preprocessor."""

    def __init__(
        self,
        records: list[dict[str, Any]],
        image_root: Path,
        model_id: str = OWLV2_MODEL_ID,
        revision: str | None = OWLV2_MODEL_REVISION,
    ) -> None:
        self.records = records
        self.image_root = image_root
        self.image_processor = Owlv2ImageProcessor.from_pretrained(
            model_id, revision=revision
        )

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, index: int) -> dict[str, Any]:
        rec = self.records[index]
        img_path = self.image_root / str(rec["file_name"])
        with Image.open(img_path) as im:
            rgb = im.convert("RGB")
            orig_w, orig_h = rgb.size
            proc_out = self.image_processor(images=rgb, return_tensors="pt")
        return {
            "image_id": int(rec["image_id"]),
            "source_dataset": str(rec["source_dataset"]),
            "width": int(orig_w),
            "height": int(orig_h),
            "pixel_values": proc_out["pixel_values"].squeeze(0),
        }


def _collate_patch_batch(batch: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "image_id": [b["image_id"] for b in batch],
        "source_dataset": [b["source_dataset"] for b in batch],
        "width": [b["width"] for b in batch],
        "height": [b["height"] for b in batch],
        "pixel_values": torch.stack([b["pixel_values"] for b in batch], dim=0),
    }


def _load_clean_gt_for_image(
    conn: sqlite3.Connection,
    image_id: int,
    source_dataset: str,
) -> tuple[list[list[float]], list[str | None]]:
    """Fetch cleaned Human GT boxes and canonical labels for one image."""
    ann_rows = conn.execute(
        """
        SELECT x1, y1, x2, y2, ignore_region, category_name, geometry_tier
        FROM annotations
        WHERE image_id = ?
        ORDER BY rowid
        """,
        (int(image_id),),
    ).fetchall()
    hygiene_rows = [
        RawAnnotationRow(
            x1=float(r[0]),
            y1=float(r[1]),
            x2=float(r[2]),
            y2=float(r[3]),
            ignore_region=int(r[4]),
            category_name=str(r[5]),
            geometry_tier=str(r[6]),
        )
        for r in ann_rows
    ]
    dec = sanitize_image_annotations(hygiene_rows, source_dataset=source_dataset)
    boxes = [
        [hygiene_rows[i].x1, hygiene_rows[i].y1, hygiene_rows[i].x2, hygiene_rows[i].y2]
        for i in dec.kept_positive_indices
    ]
    raw_names = [hygiene_rows[i].category_name for i in dec.kept_positive_indices]
    return boxes, canonicalize_gt_labels(raw_names)


def _seed_output_db_atomic(source_db: Path, output_db: Path) -> None:
    """Clone `source_db` into `output_db` via an atomic temporary file using `sqlite3.backup`."""
    if not source_db.exists():
        raise FileNotFoundError(f"Source cache DB not found: {source_db}")
    output_db.parent.mkdir(parents=True, exist_ok=True)
    tmp_db = output_db.with_suffix(".sqlite.seed_tmp")
    for leftover in (tmp_db, Path(f"{tmp_db}-wal"), Path(f"{tmp_db}-shm")):
        if leftover.exists():
            leftover.unlink()
    print(f"[Patch] Seeding {output_db} from {source_db} via sqlite3.backup...", flush=True)
    with sqlite3.connect(f"file:{source_db}?mode=ro", uri=True) as src_conn:
        with sqlite3.connect(tmp_db) as dst_conn:
            src_conn.backup(dst_conn)
    os.replace(tmp_db, output_db)


def _collect_excluded_val_holdout_ids(
    gt_index: Path,
    image_root: Path,
    holdout_manifest: Path,
) -> set[int]:
    """Gather all val & holdout image IDs for zero-leakage auditing."""
    excluded: set[int] = set()
    if holdout_manifest.exists():
        raw = json.loads(holdout_manifest.read_text())
        for key in ("images", "excluded_validation_images"):
            for rec in raw.get(key, []):
                if rec.get("image_id") is not None:
                    excluded.add(int(rec["image_id"]))
    val_ds = HumanGTHBBDataset(
        cohort_db_path=gt_index,
        gt_index_path=gt_index,
        image_root=image_root,
        split="val",
        cohort_size=8000,
        sanitize_annotations=False,
    )
    excluded.update(int(r["image_id"]) for r in val_ds.images)
    return excluded


def _select_remaining_targets(
    cache_conn: sqlite3.Connection,
    gt_conn: sqlite3.Connection,
    already_patched: set[int],
    max_images: int | None,
    target_datasets: Sequence[str] | None = None,
) -> list[dict[str, Any]]:
    """Select unpatched non-ADE20K image records, round-robin sampled if `max_images` is set."""
    target_set = set(target_datasets) if target_datasets else None
    cached_rows = cache_conn.execute(
        """
        SELECT image_id, source_dataset, width, height
        FROM processed_images
        WHERE source_dataset != 'ade20k'
        """
    ).fetchall()
    gt_meta = {
        int(r[0]): (str(r[1]), int(r[2]))
        for r in gt_conn.execute("SELECT image_id, file_name, row_index FROM images").fetchall()
    }

    candidates: list[dict[str, Any]] = []
    for img_id, src, w, h in cached_rows:
        iid = int(img_id)
        src_str = str(src)
        if target_set is not None and src_str not in target_set:
            continue
        if iid in already_patched:
            continue
        meta = gt_meta.get(iid)
        if meta is None:
            raise KeyError(f"Cached image_id={iid} missing from GT index images table.")
        fn, row_idx = meta
        candidates.append(
            {
                "image_id": iid,
                "source_dataset": src_str,
                "width": int(w),
                "height": int(h),
                "file_name": fn,
                "row_index": row_idx,
            }
        )

    candidates.sort(key=lambda r: int(r["row_index"]))
    if max_images is None or max_images >= len(candidates):
        return candidates

    by_ds: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for rec in candidates:
        by_ds[str(rec["source_dataset"])].append(rec)

    ds_names = sorted(by_ds.keys())
    selected: list[dict[str, Any]] = []
    cursor = 0
    while len(selected) < max_images:
        added_any = False
        for ds in ds_names:
            bucket = by_ds[ds]
            if cursor < len(bucket):
                selected.append(bucket[cursor])
                added_any = True
                if len(selected) >= max_images:
                    break
        if not added_any:
            break
        cursor += 1
    return selected


def _audit_patched_cache(
    *,
    source_db: Path,
    output_db: Path,
    gt_index: Path,
    image_root: Path,
    holdout_manifest: Path,
    patched_in_run: int,
    filter_stats: dict[str, int],
    target_datasets: Sequence[str] | None,
) -> dict[str, Any]:
    """Run final SQLite integrity, finiteness, overlap, and per-dataset delta audit."""
    with sqlite3.connect(f"file:{source_db}?mode=ro", uri=True) as s_conn:
        src_count = int(s_conn.execute("SELECT COUNT(*) FROM processed_images").fetchone()[0])

    with sqlite3.connect(f"file:{output_db}?mode=ro", uri=True) as o_conn:
        qc_row = o_conn.execute("PRAGMA quick_check").fetchone()
        quick_check = str(qc_row[0]) if qc_row else "unknown"
        if quick_check != "ok":
            raise RuntimeError(f"SQLite PRAGMA quick_check failed on {output_db}: {quick_check}")
        patch_rows = o_conn.execute(
            """
            SELECT p.image_id, pi.source_dataset, p.added_pole, p.added_pillar
            FROM patched_images p
            JOIN processed_images pi ON p.image_id = pi.image_id
            WHERE p.patch_name = ?
            """,
            (PATCH_NAME,),
        ).fetchall()

    patch_ds_counts: dict[str, dict[str, int]] = defaultdict(
        lambda: {"patched_images": 0, "added_pole": 0, "added_pillar": 0}
    )
    for _iid, src, add_pole, add_pil in patch_rows:
        d = patch_ds_counts[str(src)]
        d["patched_images"] += 1
        d["added_pole"] += int(add_pole)
        d["added_pillar"] += int(add_pil)

    excluded_ids = _collect_excluded_val_holdout_ids(gt_index, image_root, holdout_manifest)
    reader = OwlTeacherCacheReader(output_db)
    total_images = 0
    overlap_ids: list[int] = []
    non_finite_records = 0
    ds_totals: dict[str, dict[str, int]] = defaultdict(
        lambda: {"images": 0, "final_owl_pole": 0, "final_owl_pillar": 0, "final_owl_total": 0}
    )

    try:
        for rec in reader.iter_records():
            total_images += 1
            if rec.image_id in excluded_ids:
                overlap_ids.append(rec.image_id)
            if (
                not np.isfinite(rec.owl_embeddings).all()
                or not np.isfinite(rec.owl_boxes).all()
                or not np.isfinite(rec.gt_embeddings).all()
            ):
                non_finite_records += 1
            dt = ds_totals[rec.source_dataset]
            dt["images"] += 1
            dt["final_owl_total"] += len(rec.owl_labels)
            for lbl in rec.owl_labels:
                if lbl == "pole":
                    dt["final_owl_pole"] += 1
                elif lbl == "pillar":
                    dt["final_owl_pillar"] += 1
    finally:
        reader.close()

    if total_images != src_count:
        raise RuntimeError(
            f"Image count mismatch: source={src_count} vs patched={total_images}"
        )
    if non_finite_records > 0:
        raise RuntimeError(f"Detected {non_finite_records} non-finite embedding/box records!")
    if overlap_ids:
        raise RuntimeError(
            f"Leakage detected! {len(overlap_ids)} cached images overlap val/holdout: {overlap_ids[:10]}"
        )

    per_dataset: dict[str, dict[str, int]] = {}
    tot_add_pole = 0
    tot_add_pillar = 0
    tot_final_pole = 0
    tot_final_pillar = 0
    tot_final_owl = 0

    for ds in DATASET_ORDER:
        if ds not in ds_totals:
            continue
        dt = ds_totals[ds]
        pd = patch_ds_counts.get(ds, {"patched_images": 0, "added_pole": 0, "added_pillar": 0})
        add_p = pd["added_pole"]
        add_col = pd["added_pillar"]
        tot_add_pole += add_p
        tot_add_pillar += add_col
        tot_final_pole += dt["final_owl_pole"]
        tot_final_pillar += dt["final_owl_pillar"]
        tot_final_owl += dt["final_owl_total"]
        per_dataset[ds] = {
            "images": dt["images"],
            "patched_images": pd["patched_images"],
            "source_owl_pole": dt["final_owl_pole"] - add_p,
            "added_pole": add_p,
            "final_owl_pole": dt["final_owl_pole"],
            "source_owl_pillar": dt["final_owl_pillar"] - add_col,
            "added_pillar": add_col,
            "final_owl_pillar": dt["final_owl_pillar"],
            "source_owl_total": dt["final_owl_total"] - (add_p + add_col),
            "final_owl_total": dt["final_owl_total"],
        }

    return {
        "patch_name": PATCH_NAME,
        "target_datasets": list(target_datasets) if target_datasets else "all_non_ade20k",
        "source_db": str(source_db),
        "output_db": str(output_db),
        "quick_check": quick_check,
        "total_images": total_images,
        "source_images": src_count,
        "patched_images_total": len(patch_rows),
        "patched_in_this_run": patched_in_run,
        "non_finite_records": non_finite_records,
        "val_holdout_overlap_count": len(overlap_ids),
        "overall_delta": {
            "added_pole": tot_add_pole,
            "added_pillar": tot_add_pillar,
            "source_owl_pole": tot_final_pole - tot_add_pole,
            "final_owl_pole": tot_final_pole,
            "source_owl_pillar": tot_final_pillar - tot_add_pillar,
            "final_owl_pillar": tot_final_pillar,
            "source_owl_total": tot_final_owl - (tot_add_pole + tot_add_pillar),
            "final_owl_total": tot_final_owl,
        },
        "run_filter_stats": filter_stats,
        "per_dataset": per_dataset,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-db",
        type=Path,
        default=Path("artifacts/cache/owlv2_teacher_30k.sqlite"),
    )
    parser.add_argument(
        "--output-db",
        type=Path,
        default=Path("artifacts/cache/owlv2_teacher_30k_patched.sqlite"),
    )
    parser.add_argument("--gt-index", type=Path, default=DEFAULT_ENRICHED_GT_INDEX_PATH)
    parser.add_argument("--image-root", type=Path, default=DEFAULT_IMAGE_ROOT)
    parser.add_argument(
        "--holdout-manifest",
        type=Path,
        default=Path("artifacts/dashboard_holdout_8k_8k_full.json"),
    )
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument(
        "--max-images",
        type=int,
        default=None,
        help="Optional cap (round-robin across non-ADE20K datasets) for fast smoke tests.",
    )
    parser.add_argument(
        "--datasets",
        nargs="+",
        type=str,
        default=None,
        help="Optional list of source datasets to patch; defaults to all non-ADE20K datasets.",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.datasets is not None and "ade20k" in args.datasets:
        raise ValueError(
            "ade20k already has Human GT pole & pillar annotations and must not be patched."
        )
    if args.batch_size > 4 and str(args.device).startswith("cuda"):
        print(
            f"[Warn] batch_size={args.batch_size} > 4 may risk CUDA OOM on 12.5GB RTX 3060 at 1008x1008.",
            flush=True,
        )

    if not args.output_db.exists():
        _seed_output_db_atomic(args.source_db, args.output_db)

    writer = OwlTeacherCacheWriter(args.output_db)
    writer.conn.execute(
        """
        CREATE TABLE IF NOT EXISTS patched_images (
            image_id INTEGER PRIMARY KEY,
            patch_name TEXT NOT NULL,
            added_pole INTEGER NOT NULL,
            added_pillar INTEGER NOT NULL
        )
        """
    )
    writer.commit()

    already_patched = {
        int(r[0])
        for r in writer.conn.execute(
            "SELECT image_id FROM patched_images WHERE patch_name = ?",
            (PATCH_NAME,),
        ).fetchall()
    }

    gt_conn = sqlite3.connect(f"file:{args.gt_index}?mode=ro&immutable=1", uri=True)
    remaining = _select_remaining_targets(
        writer.conn,
        gt_conn,
        already_patched,
        args.max_images,
        target_datasets=args.datasets,
    )
    ds_desc = ",".join(args.datasets) if args.datasets else "all_non_ade20k"
    print(
        f"[Patch:{PATCH_NAME}] Scope={ds_desc} | Already patched={len(already_patched)} | Remaining={len(remaining)}",
        flush=True,
    )

    filter_stats: dict[str, int] = defaultdict(int)
    processed_now = 0
    running_pole = 0
    running_pillar = 0

    if remaining:
        device = torch.device(args.device)
        extractor = Owlv2TeacherExtractor(
            device=device,
            prompts=OWLV2_PATCH_BOLLARD_PILLAR_PROMPTS,
        )
        ds = _PatchImageDataset(remaining, args.image_root, model_id=OWLV2_MODEL_ID)
        loader = DataLoader(
            ds,
            batch_size=args.batch_size,
            shuffle=False,
            num_workers=args.workers,
            prefetch_factor=2 if args.workers > 0 else None,
            collate_fn=_collate_patch_batch,
        )
        t_start = time.perf_counter()
        log_interval = 1 if len(remaining) <= 100 else 1000

        for batch_idx, batch in enumerate(loader):
            b_ids: list[int] = batch["image_id"]
            b_srcs: list[str] = batch["source_dataset"]
            b_ws: list[int] = batch["width"]
            b_hs: list[int] = batch["height"]
            b_sizes = list(zip(b_hs, b_ws, strict=True))

            ext_list = extractor.extract_batch(
                batch["pixel_values"],
                b_sizes,
                [[] for _ in b_ids],
                min_score=0.26,
                per_class_iou=0.45,
                cross_class_iou=0.55,
            )

            for i, iid in enumerate(b_ids):
                src = b_srcs[i]
                img_w, img_h = int(b_ws[i]), int(b_hs[i])
                rec = writer.get_image(iid)
                if rec is None:
                    raise KeyError(f"Missing cached record for image_id={iid}")

                gt_boxes, gt_canon = _load_clean_gt_for_image(gt_conn, iid, src)
                sup_set = set(rec.superseded_gt_indices)
                rem_gt_boxes = [
                    gt_boxes[g] for g in range(len(gt_boxes)) if g not in sup_set
                ]
                rem_gt_canon = [
                    gt_canon[g] for g in range(len(gt_canon)) if g not in sup_set
                ]

                owl_dets: list[OwlDetection] = ext_list[i]["owl_detections"]
                accepted, p_stats = merge_patch_owl_detections(
                    rem_gt_boxes,
                    rem_gt_canon,
                    rec.owl_boxes,
                    rec.owl_labels,
                    owl_dets,
                    source_dataset=src,
                    img_w=img_w,
                    img_h=img_h,
                )

                if accepted:
                    patch_to_row = {d.patch_index: r_i for r_i, d in enumerate(owl_dets)}
                    row_idxs = [patch_to_row[d.patch_index] for d in accepted]
                    added_embs = np.ascontiguousarray(
                        ext_list[i]["owl_embeddings"][row_idxs], dtype=np.float16
                    )
                    writer.append_owl_detections(
                        image_id=iid,
                        added_detections=accepted,
                        added_embeddings=added_embs,
                        extra_stats=p_stats,
                    )

                add_pole = int(p_stats["patch_added_pole"])
                add_pillar = int(p_stats["patch_added_pillar"])
                writer.conn.execute(
                    """
                    INSERT OR REPLACE INTO patched_images (
                        image_id, patch_name, added_pole, added_pillar
                    ) VALUES (?, ?, ?, ?)
                    """,
                    (int(iid), PATCH_NAME, add_pole, add_pillar),
                )

                running_pole += add_pole
                running_pillar += add_pillar
                for k, v in p_stats.items():
                    filter_stats[k] += int(v)
                processed_now += 1

            if (batch_idx + 1) % 50 == 0:
                writer.commit()

            should_log = (
                log_interval == 1
                or (processed_now % log_interval < args.batch_size and processed_now >= log_interval)
                or processed_now == len(remaining)
            )
            if should_log:
                elapsed = max(time.perf_counter() - t_start, 1e-3)
                rate = processed_now / elapsed
                eta_min = (len(remaining) - processed_now) / max(rate, 1e-6) / 60.0
                print(
                    f"  [{processed_now}/{len(remaining)}] {rate:.2f} img/s | "
                    f"ETA={eta_min:.1f}m | +pole={running_pole} | +pillar={running_pillar}",
                    flush=True,
                )

        del extractor
        if device.type == "cuda":
            torch.cuda.empty_cache()

    gt_conn.close()
    writer.set_metadata(
        f"patch_{PATCH_NAME}",
        json.dumps(
            {
                "updated_at": datetime.now().isoformat(timespec="seconds"),
                "target_datasets": list(args.datasets) if args.datasets else "all_non_ade20k",
                "prompts": [list(p) for p in OWLV2_PATCH_BOLLARD_PILLAR_PROMPTS],
                "patched_in_run": processed_now,
                "added_pole_in_run": running_pole,
                "added_pillar_in_run": running_pillar,
            }
        ),
    )
    writer.close(checkpoint_wal=True)

    summary = _audit_patched_cache(
        source_db=args.source_db,
        output_db=args.output_db,
        gt_index=args.gt_index,
        image_root=args.image_root,
        holdout_manifest=args.holdout_manifest,
        patched_in_run=processed_now,
        filter_stats=dict(filter_stats),
        target_datasets=args.datasets,
    )
    summary_path = args.output_db.with_name(f"{args.output_db.stem}_summary.json")
    summary_path.write_text(json.dumps(summary, indent=2))
    ov = summary["overall_delta"]
    print(
        f"\n[Patch Done] quick_check={summary['quick_check']} | imgs={summary['total_images']} | "
        f"patched={summary['patched_images_total']} | non_finite={summary['non_finite_records']} | "
        f"val_ho_overlap={summary['val_holdout_overlap_count']} | "
        f"+pole={ov['added_pole']} (+pillar={ov['added_pillar']}) -> "
        f"final pole={ov['final_owl_pole']} pillar={ov['final_owl_pillar']}\n"
        f"Summary -> {summary_path}",
        flush=True,
    )


if __name__ == "__main__":
    main()
