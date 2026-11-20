#!/usr/bin/env python3
"""End-to-end SOTA open-vocabulary detector training (Detector + Option A + Option B).

Trains MobileNetV4 + 128-ch P2LitePAN + Consistent Dual-Head (1-to-1 + 3DMF primary,
1-to-many TAL auxiliary) jointly with BOTH zero-runtime-text-encoder open-vocabulary heads:
  - Option A: DenseAnchorSemanticHead (YOLO-World style single-stage Conv2d head)
  - Option B: RoIPooledSemanticHead (Detic/ViLD style focal+context RoIAlign head, no coord shortcut)
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import replace
from datetime import datetime
import json
import math
from pathlib import Path
import sqlite3
import time
from typing import Any

from PIL import Image
import torch
from torch import Tensor, nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler
from torchvision.ops import box_iou

if hasattr(torch.multiprocessing, "set_sharing_strategy"):
    torch.multiprocessing.set_sharing_strategy("file_system")

from student_detector.data_isolation import audit_training_isolation
from student_detector.decoder import InferenceDecoder
from student_detector.human_gt_data import (
    DEFAULT_ENRICHED_GT_INDEX_PATH,
    HumanGTHBBDataset,
    HumanGTQuadDataset,
)
from student_detector.losses import ProposalLoss, flatten_detector_output
from student_detector.model import QuadProposalDetector, StudentDetector
from student_detector.ontology import (
    CANONICAL_MAPPINGS,
    OPEN_VOCAB_CATEGORIES,
    ZERO_SHOT_EVAL_CATEGORIES,
    build_source_annotated_mask,
    map_to_canonical,
)
from student_detector.open_vocab_heads import (
    DenseAnchorSemanticHead,
    OpenVocabularySigmoidLoss,
    RoIPooledSemanticHead,
)
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_geometry import pairwise_quad_iou, quad_from_bbox
from student_detector.quad_losses import QuadProposalLoss, flatten_quad_output
from student_detector.quad_targets import QuadTargetBuilder
from student_detector.region_head import RegionEmbeddingHead
from student_detector.sam_student_data import (
    DEFAULT_IMAGE_ROOT,
    collate_proposal_samples,
    collate_quad_proposal_samples,
)
from student_detector.siglip import SiglipTeacher
from student_detector.targets import TargetBuilder
from student_detector.task_aligned_assigner import (
    QuadTaskAlignedAssigner,
    TaskAlignedAssigner,
)
from student_detector.training_optimization import ExponentialMovingAverage
from student_detector.vocabulary import DEFAULT_CATEGORY_SYNONYMS, VocabularyBank

DEFAULT_SAM_ENRICHED_DB = Path("artifacts/pseudo_labels/sam_discovery_enriched.sqlite")
DEFAULT_HOLDOUT_MANIFEST = Path("artifacts/dashboard_holdout_8k_8k_full.json")
DEFAULT_CACHE_PATH = Path("artifacts/cache/siglip_gt_crops_8k.pt")
DEFAULT_SAM_DISCOVERY_CACHE = Path("artifacts/cache/siglip_sam_discovery_8k.pt")
DEFAULT_OPEN_WORLD_PSEUDO_GT_CACHE = Path("artifacts/cache/open_world_pseudo_gt_8k.pt")
DEFAULT_VOCAB_DIR = Path("artifacts/sota_open_vocab")
DEFAULT_VISUAL_CENTROID_PATH = Path("artifacts/sota_open_vocab/visual_centroid.pt")


class JointOpenVocabSystem(nn.Module):
    """Container module bundling detector, Option A head, and Option B head for EMA."""

    def __init__(
        self,
        detector: StudentDetector | QuadProposalDetector,
        head_a: DenseAnchorSemanticHead,
        head_b: RoIPooledSemanticHead,
    ) -> None:
        super().__init__()
        self.detector = detector
        self.head_a = head_a
        self.head_b = head_b


class _NativeGTCropDataset(Dataset[tuple[int, Tensor]]):
    """Image-level dataset reading each PIL image once and cropping all positive GT boxes."""

    def __init__(
        self,
        records: list[tuple[int, str, list[tuple[float, float, float, float]]]],
        image_root: Path,
        crop_size: int = 224,
        margin: float = 0.15,
    ) -> None:
        self.records = records
        self.image_root = image_root
        self.crop_size = crop_size
        self.margin = margin

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, idx: int) -> tuple[int, Tensor]:
        image_id, file_name, boxes = self.records[idx]
        if not boxes:
            return image_id, torch.empty((0, 3, self.crop_size, self.crop_size), dtype=torch.uint8)
        img_path = self.image_root / file_name
        with Image.open(img_path) as raw:
            pil = raw.convert("RGB")
            w_img, h_img = pil.size
            crops: list[Tensor] = []
            for x1, y1, x2, y2 in boxes:
                bw = max(x2 - x1, 1.0)
                bh = max(y2 - y1, 1.0)
                cx1 = max(0.0, min(float(w_img - 1), x1 - bw * self.margin))
                cy1 = max(0.0, min(float(h_img - 1), y1 - bh * self.margin))
                cx2 = max(cx1 + 1.0, min(float(w_img), x2 + bw * self.margin))
                cy2 = max(cy1 + 1.0, min(float(h_img), y2 + bh * self.margin))
                patch = pil.crop((cx1, cy1, cx2, cy2)).resize(
                    (self.crop_size, self.crop_size),
                    resample=Image.Resampling.BICUBIC,
                )
                arr = (
                    torch.frombuffer(bytearray(patch.tobytes()), dtype=torch.uint8)
                    .reshape(self.crop_size, self.crop_size, 3)
                    .permute(2, 0, 1)
                )
                crops.append(arr)
        return image_id, torch.stack(crops, dim=0)


def ensure_vocab_banks_and_teacher_cache(
    *,
    train_images: list[dict[str, Any]],
    gt_index_path: Path,
    image_root: Path,
    cache_path: Path,
    vocab_dir: Path,
    device: torch.device,
    workers: int = 6,
) -> tuple[VocabularyBank, dict[int, tuple[int, int]], Tensor, Tensor]:
    """Build or load frozen 37-class + 41-class VocabularyBanks and flip-safe FP16 teacher cache."""
    vocab_dir.mkdir(parents=True, exist_ok=True)
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    vocab_37_path = vocab_dir / "vocabulary_bank_37classes.pt"
    vocab_41_path = vocab_dir / "vocabulary_bank_41classes_zeroshot.pt"

    need_vocab = not (vocab_37_path.exists() and vocab_41_path.exists())
    existing_offsets: dict[int, tuple[int, int]] = {}
    existing_unflipped = torch.empty((0, 768), dtype=torch.float16)
    existing_flipped = torch.empty((0, 768), dtype=torch.float16)
    if cache_path.exists():
        loaded_cache = torch.load(cache_path, map_location="cpu", weights_only=False)
        existing_offsets = dict(loaded_cache["offsets"])
        existing_unflipped = loaded_cache["unflipped"]
        existing_flipped = loaded_cache["flipped"]

    missing_images = [
        rec for rec in train_images if int(rec["image_id"]) not in existing_offsets
    ]
    need_cache = len(missing_images) > 0

    if need_vocab or need_cache:
        print(
            f"[{datetime.now().strftime('%H:%M:%S')}] Loading SiglipTeacher on {device} "
            f"(need_vocab={need_vocab}, missing_cache_images={len(missing_images)})..."
        )
        teacher = SiglipTeacher(device=device)

        if need_vocab:
            bank_37 = VocabularyBank.from_teacher(
                teacher,
                OPEN_VOCAB_CATEGORIES,
                synonyms=DEFAULT_CATEGORY_SYNONYMS,
            )
            bank_37.save(vocab_37_path)
            bank_41 = VocabularyBank.from_teacher(
                teacher,
                OPEN_VOCAB_CATEGORIES + ZERO_SHOT_EVAL_CATEGORIES,
                synonyms=DEFAULT_CATEGORY_SYNONYMS,
            )
            bank_41.save(vocab_41_path)
            print(
                f"Saved frozen SigLIP VocabularyBanks: {len(OPEN_VOCAB_CATEGORIES)} base classes "
                f"and {len(OPEN_VOCAB_CATEGORIES) + len(ZERO_SHOT_EVAL_CATEGORIES)} zero-shot eval classes."
            )

        if need_cache:
            t0 = time.time()
            train_id_to_file = {
                int(rec["image_id"]): str(rec["file_name"]) for rec in missing_images
            }
            sorted_ids = sorted(train_id_to_file.keys())
            boxes_by_id: dict[int, list[tuple[float, float, float, float]]] = {
                i: [] for i in sorted_ids
            }
            with sqlite3.connect(f"file:{gt_index_path}?mode=ro&immutable=1", uri=True) as conn:
                conn.execute("CREATE TEMP TABLE cache_ids (image_id INTEGER PRIMARY KEY)")
                conn.executemany("INSERT INTO cache_ids VALUES (?)", [(i,) for i in sorted_ids])
                rows = conn.execute(
                    """
                    SELECT a.image_id, a.x1, a.y1, a.x2, a.y2
                    FROM annotations a
                    JOIN cache_ids c ON a.image_id = c.image_id
                    WHERE a.ignore_region = 0
                    ORDER BY a.image_id, a.rowid
                    """
                ).fetchall()
            for img_id, x1, y1, x2, y2 in rows:
                boxes_by_id[int(img_id)].append((float(x1), float(y1), float(x2), float(y2)))

            records = [(i, train_id_to_file[i], boxes_by_id[i]) for i in sorted_ids]
            total_boxes = sum(len(r[2]) for r in records)
            print(
                f"Encoding {total_boxes} positive GT crops x 2 (unflipped + flipped) "
                f"across {len(records)} missing train images..."
            )
            crop_ds = _NativeGTCropDataset(records, image_root=image_root)
            crop_loader = DataLoader(
                crop_ds,
                batch_size=32,
                shuffle=False,
                num_workers=workers,
                collate_fn=lambda batch: batch,
            )

            img_proc = teacher.processor.image_processor
            mean = torch.tensor(img_proc.image_mean, device=device, dtype=torch.float32).view(
                1, 3, 1, 1
            )
            std = torch.tensor(img_proc.image_std, device=device, dtype=torch.float32).view(
                1, 3, 1, 1
            )

            base_cursor = int(existing_unflipped.shape[0])
            new_unflipped = torch.empty((total_boxes, 768), dtype=torch.float16)
            new_flipped = torch.empty((total_boxes, 768), dtype=torch.float16)
            cursor = 0

            with torch.no_grad():
                for batch_items in crop_loader:
                    non_empty: list[Tensor] = []
                    spans: list[tuple[int, int, int]] = []
                    local_cur = 0
                    for img_id, crop_u8 in batch_items:
                        cnt = int(crop_u8.shape[0])
                        existing_offsets[img_id] = (base_cursor + cursor, cnt)
                        if cnt > 0:
                            non_empty.append(crop_u8)
                            spans.append((cursor, local_cur, cnt))
                            local_cur += cnt
                        cursor += cnt
                    if not non_empty:
                        continue
                    cat_u8 = torch.cat(non_empty, dim=0)
                    for start in range(0, cat_u8.shape[0], 256):
                        end = min(start + 256, cat_u8.shape[0])
                        chunk = cat_u8[start:end].to(device, non_blocking=True).float() / 255.0
                        normed = (chunk - mean) / std
                        normed_flip = torch.flip(normed, dims=[-1])
                        with torch.amp.autocast(
                            device.type, dtype=torch.float16, enabled=(device.type == "cuda")
                        ):
                            out_u = teacher.model.get_image_features(pixel_values=normed)
                            feat_u = (
                                out_u.pooler_output
                                if hasattr(out_u, "pooler_output")
                                and out_u.pooler_output is not None
                                else out_u[0]
                            )
                            emb_u = F.normalize(feat_u.float(), p=2.0, dim=-1).half().cpu()

                            out_f = teacher.model.get_image_features(pixel_values=normed_flip)
                            feat_f = (
                                out_f.pooler_output
                                if hasattr(out_f, "pooler_output")
                                and out_f.pooler_output is not None
                                else out_f[0]
                            )
                            emb_f = F.normalize(feat_f.float(), p=2.0, dim=-1).half().cpu()

                        for g_off, l_off, cnt in spans:
                            ov_s = max(start, l_off)
                            ov_e = min(end, l_off + cnt)
                            if ov_s < ov_e:
                                dst_s = g_off + (ov_s - l_off)
                                dst_e = g_off + (ov_e - l_off)
                                new_unflipped[dst_s:dst_e] = emb_u[ov_s - start : ov_e - start]
                                new_flipped[dst_s:dst_e] = emb_f[ov_s - start : ov_e - start]

            merged_unflipped = (
                torch.cat([existing_unflipped, new_unflipped], dim=0)
                if existing_unflipped.numel()
                else new_unflipped
            )
            merged_flipped = (
                torch.cat([existing_flipped, new_flipped], dim=0)
                if existing_flipped.numel()
                else new_flipped
            )
            torch.save(
                {
                    "offsets": existing_offsets,
                    "unflipped": merged_unflipped,
                    "flipped": merged_flipped,
                },
                cache_path,
            )
            print(
                f"Cached +{total_boxes} (total {merged_unflipped.shape[0]}) flip-safe FP16 SigLIP crop pairs "
                f"to {cache_path} in {time.time() - t0:.1f}s."
            )

        del teacher
        if device.type == "cuda":
            torch.cuda.empty_cache()

    bank_37 = VocabularyBank.load(vocab_37_path, device=device)
    cache = torch.load(cache_path, map_location="cpu", weights_only=False)
    return bank_37, cache["offsets"], cache["unflipped"], cache["flipped"]


def augment_train_with_rare_classes_and_rfs(
    train_ds: Any,
    val_sets: dict[str, list[dict[str, Any]]],
    gt_index_path: Path,
    cat_to_idx: dict[str, int],
    rfs_t: float = 0.035,
) -> tuple[Tensor, WeightedRandomSampler]:
    """Supplement train split with rare-class images from SQLite and build LVIS Repeat-Factor Sampler."""
    forbidden_ids: set[int] = {int(r["image_id"]) for r in train_ds.images}
    forbidden_files: set[str] = {str(r["file_name"]) for r in train_ds.images}
    for v_imgs in val_sets.values():
        for r in v_imgs:
            forbidden_ids.add(int(r["image_id"]))
            forbidden_files.add(str(r["file_name"]))

    rare_quotas = {
        "trash_bin": 450,
        "pillar": 300,
        "signboard": 250,
        "traffic_sign": 250,
        "traffic_light": 150,
        "street_light": 150,
        "canopy": 150,
        "cart": 150,
        "golf_cart": 50,
        "wheelchair": 150,
        "stroller": 150,
        "debris": 150,
        "parking_meter": 150,
        "fire_hydrant": 150,
        "dog": 60,
        "bench": 150,
        "license_plate": 150,
        "pole": 200,
    }
    raw_to_rare: dict[str, str] = {}
    for raw_k, can_v in CANONICAL_MAPPINGS.items():
        if can_v in rare_quotas:
            raw_to_rare[raw_k] = can_v
            raw_to_rare[raw_k.replace(" ", "_")] = can_v
    for rt in rare_quotas:
        raw_to_rare[rt] = rt
        raw_to_rare[rt.replace("_", " ")] = rt

    conn = train_ds._get_connection()
    cat_col = train_ds._get_category_column(conn) or "category_name"
    placeholders = ",".join("?" for _ in raw_to_rare)
    rows = conn.execute(
        f"""
        SELECT i.image_id, i.file_name, i.source_dataset, i.width, i.height, a.{cat_col}
        FROM annotations a
        JOIN images i ON a.image_id = i.image_id
        WHERE a.ignore_region = 0
          AND LOWER(a.{cat_col}) IN ({placeholders})
        ORDER BY CASE i.source_dataset
            WHEN 'open_images_subset' THEN 0
            WHEN 'nuimages' THEN 1
            WHEN 'bdd100k_images_100k' THEN 2
            WHEN 'bdd100k' THEN 2
            WHEN 'woodscape_rgb_fisheye' THEN 3
            ELSE 4
        END, i.image_id
        """,
        tuple(raw_to_rare.keys()),
    ).fetchall()

    added_by_cls: dict[str, int] = defaultdict(int)
    added_records: dict[int, dict[str, Any]] = {}
    for img_id_raw, fname_raw, src_raw, w_raw, h_raw, raw_c in rows:
        img_id = int(img_id_raw)
        fname = str(fname_raw)
        if img_id in forbidden_ids or fname in forbidden_files:
            continue
        can = map_to_canonical(str(raw_c))
        if can in rare_quotas and added_by_cls[can] < rare_quotas[can]:
            if img_id not in added_records:
                added_records[img_id] = {
                    "image_id": img_id,
                    "file_name": fname,
                    "source_dataset": str(src_raw),
                    "width": int(w_raw),
                    "height": int(h_raw),
                }
            added_by_cls[can] += 1

    if added_records:
        train_ds.images.extend(added_records.values())
        print(
            f"Augmented training cohort with +{len(added_records)} rare-class images "
            f"(new train total={len(train_ds.images)}): {dict(added_by_cls)}"
        )

    # Compute exact class instance counts and LVIS Repeat-Factor Sampling weights
    num_classes = len(cat_to_idx)
    class_counts = torch.ones((num_classes,), dtype=torch.float32)
    img_cls_sets: list[set[int]] = [set() for _ in range(len(train_ds.images))]
    id_to_pos = {int(rec["image_id"]): idx for idx, rec in enumerate(train_ds.images)}

    conn.execute("DROP TABLE IF EXISTS rfs_train_ids")
    conn.execute("CREATE TEMP TABLE rfs_train_ids (image_id INTEGER PRIMARY KEY)")
    conn.executemany(
        "INSERT INTO rfs_train_ids VALUES (?)", [(i,) for i in id_to_pos]
    )
    ann_rows = conn.execute(
        f"""
        SELECT a.image_id, a.{cat_col}
        FROM annotations a
        JOIN rfs_train_ids r ON a.image_id = r.image_id
        WHERE a.ignore_region = 0
        """
    ).fetchall()
    for img_id_raw, raw_c in ann_rows:
        if not raw_c:
            continue
        can = map_to_canonical(str(raw_c))
        if can is not None and can in cat_to_idx:
            c_idx = cat_to_idx[can]
            class_counts[c_idx] += 1.0
            pos_i = id_to_pos.get(int(img_id_raw))
            if pos_i is not None:
                img_cls_sets[pos_i].add(c_idx)

    if DEFAULT_OPEN_WORLD_PSEUDO_GT_CACHE.exists():
        ow_cache = torch.load(
            DEFAULT_OPEN_WORLD_PSEUDO_GT_CACHE, map_location="cpu", weights_only=False
        )
        for img_id_ow, ow_rec in ow_cache.items():
            pos_i = id_to_pos.get(int(img_id_ow))
            if pos_i is None:
                continue
            for cat_ow in ow_rec.get("categories", ()):
                if cat_ow in cat_to_idx:
                    c_idx = cat_to_idx[cat_ow]
                    class_counts[c_idx] += 1.0
                    img_cls_sets[pos_i].add(c_idx)

    n_imgs = max(len(train_ds.images), 1)
    img_freq = torch.zeros((num_classes,), dtype=torch.float32)
    for s_c in img_cls_sets:
        for c_i in s_c:
            img_freq[c_i] += 1.0
    frac = (img_freq / float(n_imgs)).clamp(min=1e-4)
    cls_repeat = torch.clamp(torch.sqrt(rfs_t / frac), min=1.0, max=4.5)

    added_img_ids = set(added_records.keys())
    priority_rare_idxs = {
        cat_to_idx[c]
        for c in (
            "trash_bin",
            "pillar",
            "signboard",
            "pole",
            "traffic_light",
            "traffic_sign",
            "tree",
            "fire_hydrant",
            "traffic_cone",
        )
        if c in cat_to_idx
    }
    oi_boost_idxs = {
        cat_to_idx[c] for c in ("trash_bin", "traffic_sign") if c in cat_to_idx
    }

    sample_weights: list[float] = []
    for rec, s_c in zip(train_ds.images, img_cls_sets, strict=True):
        w = 1.0 if not s_c else float(max(cls_repeat[c_i].item() for c_i in s_c))
        iid = int(rec["image_id"])
        if iid in added_img_ids or bool(s_c & priority_rare_idxs):
            w = max(w, 2.5)
        if str(rec.get("source_dataset", "")) == "open_images_subset" and bool(
            s_c & oi_boost_idxs
        ):
            w = max(w, 4.0)
        sample_weights.append(w)

    sampler = WeightedRandomSampler(
        weights=sample_weights,
        num_samples=len(train_ds.images),
        replacement=True,
    )
    return class_counts, sampler


def load_sam_federated_ignore_boxes(
    sam_db_path: Path,
    train_images: list[dict[str, Any]],
    sparse_sources: set[str],
    min_stability: float = 0.90,
) -> dict[int, Tensor]:
    """Preload SAM-discovered unannotated object boxes for sparse sources to mask false negatives."""
    if not sam_db_path.exists():
        return {}
    sparse_ids = [
        int(img["image_id"])
        for img in train_images
        if str(img["source_dataset"]) in sparse_sources
    ]
    if not sparse_ids:
        return {}
    by_id: dict[int, list[list[float]]] = {}
    with sqlite3.connect(f"file:{sam_db_path}?mode=ro&immutable=1", uri=True) as conn:
        conn.execute("CREATE TEMP TABLE sids (image_id INTEGER PRIMARY KEY)")
        conn.executemany("INSERT INTO sids VALUES (?)", [(i,) for i in sparse_ids])
        rows = conn.execute(
            """
            SELECT a.image_id, a.bbox_x1, a.bbox_y1, a.bbox_x2, a.bbox_y2
            FROM annotations a
            JOIN sids s ON a.image_id = s.image_id
            WHERE COALESCE(a.stability_score, 1.0) >= ?
              AND (
                (a.is_discovered = 1 AND a.is_suppressed = 0)
                OR (
                  a.category = 'contained_subpart'
                  AND (a.bbox_x2 - a.bbox_x1) * (a.bbox_y2 - a.bbox_y1) BETWEEN 400.0 AND 65000.0
                )
              )
            """,
            (min_stability,),
        ).fetchall()
    for img_id, x1, y1, x2, y2 in rows:
        if x2 > x1 + 2.0 and y2 > y1 + 2.0:
            by_id.setdefault(int(img_id), []).append([float(x1), float(y1), float(x2), float(y2)])
    return {k: torch.tensor(v, dtype=torch.float32) for k, v in by_id.items()}


def inject_federated_ignore_into_samples(
    samples: list[Any],
    sam_boxes_by_id: dict[int, Tensor],
    input_size: int,
    geometry: str,
    pseudo_gt_by_id: dict[int, dict[str, Any]] | None = None,
) -> list[Any]:
    """Inject teacher-verified open-world pseudo-GT positives and SAM federated ignore boxes."""
    if not sam_boxes_by_id and not pseudo_gt_by_id:
        return samples
    updated: list[Any] = []
    for s in samples:
        cur_s = s
        orig_h, orig_w = cur_s.original_size
        is_flip = bool(getattr(cur_s, "horizontal_flip", False))
        scale, off_x, off_y = cur_s.transform

        # 1. Inject teacher-verified open-world pseudo-GT positives (trash_bin, pillar, traffic_sign, etc.)
        if pseudo_gt_by_id is not None:
            pg = pseudo_gt_by_id.get(int(cur_s.image_id))
            if pg is not None and len(pg.get("categories", ())) > 0:
                p_cats = list(pg["categories"])
                if geometry == "hbb":
                    p_boxes = torch.tensor(pg["boxes"], dtype=torch.float32)
                    if is_flip:
                        ox1 = p_boxes[:, 0].clone()
                        ox2 = p_boxes[:, 2].clone()
                        p_boxes[:, 0] = float(orig_w) - ox2
                        p_boxes[:, 2] = float(orig_w) - ox1
                    p_boxes = p_boxes * float(scale)
                    p_boxes[:, 0::2] += float(off_x)
                    p_boxes[:, 1::2] += float(off_y)
                    p_boxes.clamp_(0.0, float(input_size))
                    val_p = (p_boxes[:, 2] - p_boxes[:, 0] >= 6.0) & (
                        p_boxes[:, 3] - p_boxes[:, 1] >= 6.0
                    )
                    if val_p.any():
                        keep_idx = val_p.nonzero(as_tuple=False).squeeze(-1).tolist()
                        pb_kept = p_boxes[val_p]
                        merged_b = (
                            torch.cat([cur_s.boxes, pb_kept], dim=0)
                            if cur_s.boxes.numel()
                            else pb_kept
                        )
                        new_cats = tuple(cur_s.category_names) + tuple(
                            p_cats[k] for k in keep_idx
                        )
                        new_gti = tuple(cur_s.gt_indices) + tuple(
                            -(k + 1) for k in keep_idx
                        )
                        cur_s = replace(
                            cur_s,
                            boxes=merged_b,
                            category_names=new_cats,
                            gt_indices=new_gti,
                        )
                else:
                    if "quads" in pg and len(pg["quads"]) == len(p_cats):
                        p_quads = torch.tensor(pg["quads"], dtype=torch.float32)
                    else:
                        p_boxes_q = torch.tensor(pg["boxes"], dtype=torch.float32)
                        p_quads = torch.stack(
                            [quad_from_bbox(b) for b in p_boxes_q], dim=0
                        )
                    if is_flip:
                        p_quads[..., 0] = float(orig_w) - p_quads[..., 0]
                        p_quads = p_quads[:, [1, 0, 3, 2], :]
                    p_quads = p_quads * float(scale)
                    p_quads[..., 0] += float(off_x)
                    p_quads[..., 1] += float(off_y)
                    p_quads.clamp_(0.0, float(input_size))
                    qw = p_quads[..., 0].amax(dim=-1) - p_quads[..., 0].amin(dim=-1)
                    qh = p_quads[..., 1].amax(dim=-1) - p_quads[..., 1].amin(dim=-1)
                    val_q = (qw >= 6.0) & (qh >= 6.0)
                    if val_q.any():
                        keep_idx = val_q.nonzero(as_tuple=False).squeeze(-1).tolist()
                        pq_kept = p_quads[val_q]
                        merged_pq = (
                            torch.cat([cur_s.quads, pq_kept], dim=0)
                            if cur_s.quads.numel()
                            else pq_kept
                        )
                        new_cats = tuple(cur_s.category_names) + tuple(
                            p_cats[k] for k in keep_idx
                        )
                        new_gti = tuple(cur_s.gt_indices) + tuple(
                            -(k + 1) for k in keep_idx
                        )
                        cur_s = replace(
                            cur_s,
                            quads=merged_pq,
                            category_names=new_cats,
                            gt_indices=new_gti,
                        )

        # 2. Inject SAM federated ignore boxes across all datasets
        native_sam = sam_boxes_by_id.get(int(cur_s.image_id))
        if native_sam is None or native_sam.numel() == 0:
            updated.append(cur_s)
            continue
        boxes = native_sam.clone()
        if is_flip:
            old_x1 = boxes[:, 0].clone()
            old_x2 = boxes[:, 2].clone()
            boxes[:, 0] = float(orig_w) - old_x2
            boxes[:, 2] = float(orig_w) - old_x1
        boxes = boxes * float(scale)
        boxes[:, 0::2] += float(off_x)
        boxes[:, 1::2] += float(off_y)
        boxes.clamp_(0.0, float(input_size))
        valid = (boxes[:, 2] - boxes[:, 0] >= 4.0) & (boxes[:, 3] - boxes[:, 1] >= 4.0)
        boxes = boxes[valid]
        if boxes.numel() == 0:
            updated.append(cur_s)
            continue
        if geometry == "hbb":
            merged = (
                torch.cat([cur_s.ignore_boxes, boxes], dim=0)
                if cur_s.ignore_boxes.numel()
                else boxes
            )
            updated.append(replace(cur_s, ignore_boxes=merged))
        else:
            sam_quads = torch.stack([quad_from_bbox(b) for b in boxes], dim=0)
            merged_q = (
                torch.cat([cur_s.ignore_quads, sam_quads], dim=0)
                if cur_s.ignore_quads.numel()
                else sam_quads
            )
            updated.append(replace(cur_s, ignore_quads=merged_q))
    return updated


def map_sample_gt_labels_and_teachers(
    sample: Any,
    cat_to_idx: dict[str, int],
    cache_offsets: dict[int, tuple[int, int]],
    cache_unflipped: Tensor,
    cache_flipped: Tensor,
    pseudo_gt_by_id: dict[int, dict[str, Any]] | None = None,
) -> tuple[Tensor, Tensor, Tensor]:
    """Return (gt_labels [M], gt_teacher_emb [M, 768], gt_has_teacher [M]) for one sample."""
    num_gt = len(sample.category_names)
    if num_gt == 0:
        return (
            torch.empty((0,), dtype=torch.long),
            torch.empty((0, 768), dtype=torch.float32),
            torch.empty((0,), dtype=torch.bool),
        )
    labels: list[int] = []
    for raw_cat in sample.category_names:
        cleaned = raw_cat.strip().lower()
        if cleaned in ZERO_SHOT_EVAL_CATEGORIES:
            labels.append(-2)
            continue
        can = map_to_canonical(raw_cat)
        if can in ZERO_SHOT_EVAL_CATEGORIES:
            labels.append(-2)
        elif can is not None and can in cat_to_idx:
            labels.append(cat_to_idx[can])
        else:
            labels.append(-2)

    teacher_emb = torch.zeros((num_gt, 768), dtype=torch.float32)
    has_teacher = torch.zeros((num_gt,), dtype=torch.bool)
    is_flip = bool(getattr(sample, "horizontal_flip", False))
    span = cache_offsets.get(int(sample.image_id))
    pg = (
        pseudo_gt_by_id.get(int(sample.image_id))
        if pseudo_gt_by_id is not None
        else None
    )
    bank = cache_flipped if is_flip else cache_unflipped
    for local_i, orig_gt_idx in enumerate(sample.gt_indices):
        if orig_gt_idx >= 0 and span is not None:
            base_off, count = span
            if orig_gt_idx < count:
                teacher_emb[local_i] = bank[base_off + orig_gt_idx].float()
                has_teacher[local_i] = True
        elif orig_gt_idx < 0 and pg is not None:
            pg_idx = -(orig_gt_idx + 1)
            pg_bank = pg["flipped"] if is_flip else pg["unflipped"]
            if 0 <= pg_idx < int(pg_bank.shape[0]):
                teacher_emb[local_i] = pg_bank[pg_idx].float()
                has_teacher[local_i] = True

    # Ignore sub-6px canvas specks (<20px native) for semantic classification so 5,000+
    # distant highway specks in BDD100K do not dominate traffic_sign/traffic_light centroids.
    gt_b = (
        sample.boxes
        if hasattr(sample, "boxes")
        else RegionEmbeddingHead.boxes_from_quads(sample.quads)
    )
    if gt_b.shape[0] == num_gt:
        min_side_384 = torch.minimum(gt_b[:, 2] - gt_b[:, 0], gt_b[:, 3] - gt_b[:, 1])
        src_ds = str(getattr(sample, "source_dataset", ""))
        for i in range(num_gt):
            ms = float(min_side_384[i].item())
            raw_c = (
                map_to_canonical(sample.category_names[i])
                if i < len(sample.category_names)
                else None
            )
            if labels[i] >= 0 and (
                ms < 6.0
                or (
                    src_ds == "bdd100k_images_100k"
                    and ms < 8.0
                    and raw_c in ("traffic_sign", "traffic_light", "pole")
                )
            ):
                labels[i] = -2
                has_teacher[i] = False

    return torch.tensor(labels, dtype=torch.long), teacher_emb, has_teacher


@torch.no_grad()
def evaluate_joint_epoch(
    system: JointOpenVocabSystem,
    val_loader: DataLoader[Any],
    decoder: InferenceDecoder | QuadInferenceDecoder,
    vocab_bank: VocabularyBank,
    cat_to_idx: dict[str, int],
    geometry: str,
    input_size: int,
    device: torch.device,
) -> dict[str, float]:
    """Fast comprehensive validation measuring overlaps, recall, Option A, Option B, and A+B."""
    system.eval()
    text_emb = vocab_bank.embeddings.to(device)

    total_imgs = 0
    total_gt = 0
    recalled_10 = 0
    recalled_100 = 0
    boxes_at_030 = 0
    overlap_pairs_025 = 0
    dup_gt_hits = 0
    matched_gt_total = 0

    gt_cls_total = 0
    opt_a_gt_top1 = 0
    opt_b_gt_top1 = 0
    opt_ab_gt_top1 = 0

    clutter_props_total = 0
    opt_a_clutter_rejected = 0
    opt_b_clutter_rejected = 0

    for images, samples in val_loader:
        images = images.to(device, non_blocking=True)
        features = system.detector.forward_features(images)
        raw_out = system.detector.forward_eval_head(features)
        detections = decoder(raw_out, (input_size, input_size))
        tower_a = system.head_a.forward_tower(features)

        for b_idx, (det, sample) in enumerate(zip(detections, samples, strict=True)):
            total_imgs += 1
            scores = det.scores
            keep_030 = scores >= 0.30
            boxes_b = det.boxes
            n_030 = int(keep_030.sum().item())
            boxes_at_030 += n_030

            if geometry == "hbb":
                gt_boxes = sample.boxes.to(device)
                if n_030 >= 2:
                    b030 = boxes_b[keep_030]
                    p_iou = box_iou(b030, b030).triu(diagonal=1)
                    overlap_pairs_025 += int((p_iou > 0.25).sum().item())
                iou_mat = (
                    box_iou(boxes_b, gt_boxes)
                    if (boxes_b.numel() and gt_boxes.numel())
                    else boxes_b.new_zeros((boxes_b.shape[0], gt_boxes.shape[0]))
                )
            else:
                gt_quads = sample.quads.to(device)
                gt_boxes = RegionEmbeddingHead.boxes_from_quads(gt_quads)
                q_b = det.quads
                if n_030 >= 2:
                    q030 = q_b[keep_030]
                    p_iou = pairwise_quad_iou(q030, q030).triu(diagonal=1)
                    overlap_pairs_025 += int((p_iou > 0.25).sum().item())
                iou_mat = (
                    pairwise_quad_iou(q_b, gt_quads)
                    if (q_b.numel() and gt_quads.numel())
                    else q_b.new_zeros((q_b.shape[0], gt_quads.shape[0]))
                )

            n_gt = int(gt_boxes.shape[0])
            total_gt += n_gt
            if n_gt > 0 and iou_mat.shape[0] > 0:
                recalled_10 += int((iou_mat[:10].max(dim=0).values >= 0.50).sum().item())
                recalled_100 += int((iou_mat[:100].max(dim=0).values >= 0.50).sum().item())
                if n_030 > 0:
                    hits_per_gt = (iou_mat[keep_030] >= 0.50).sum(dim=0)
                    matched_Mask = hits_per_gt >= 1
                    matched_gt_total += int(matched_Mask.sum().item())
                    dup_gt_hits += int((hits_per_gt >= 2).sum().item())

            # Evaluate Option A, Option B, and A+B on matched proposals & clutter proposals
            if det.flat_indices is not None and det.flat_indices.numel() > 0:
                b_tensor = torch.full_like(det.flat_indices, b_idx)
                z_a = system.head_a.forward_indexed(
                    features, b_tensor, det.flat_indices, tower_features=tower_a
                )
                _, probs_a = system.head_a.classify_embeddings(z_a, text_emb)

                feat_single = [f[b_idx : b_idx + 1] for f in features]
                z_b = system.head_b(
                    feat_single, [boxes_b], image_shapes=[(input_size, input_size)]
                )
                _, probs_b = system.head_b.classify_embeddings(z_b, text_emb)
                probs_ab = 0.5 * (probs_a + probs_b)

                # 1. Classification accuracy on best-matched proposal per in-vocab GT
                if n_gt > 0:
                    best_ious, best_prop = iou_mat.max(dim=0)
                    for g_i, raw_cat in enumerate(sample.category_names):
                        if float(best_ious[g_i].item()) < 0.50:
                            continue
                        can = map_to_canonical(raw_cat)
                        if can is None or can not in cat_to_idx:
                            continue
                        target_cls = cat_to_idx[can]
                        p_idx = int(best_prop[g_i].item())
                        gt_cls_total += 1
                        if int(probs_a[p_idx].argmax().item()) == target_cls:
                            opt_a_gt_top1 += 1
                        if int(probs_b[p_idx].argmax().item()) == target_cls:
                            opt_b_gt_top1 += 1
                        if int(probs_ab[p_idx].argmax().item()) == target_cls:
                            opt_ab_gt_top1 += 1

                # 2. Clutter rejection on @0.30 proposals with IoU < 0.15 to all GTs
                if n_030 > 0:
                    max_gt_iou = (
                        iou_mat[keep_030].max(dim=1).values
                        if n_gt > 0
                        else torch.zeros(n_030, device=device)
                    )
                    clutter_mask = max_gt_iou < 0.15
                    n_clut = int(clutter_mask.sum().item())
                    if n_clut > 0:
                        clutter_props_total += n_clut
                        pa_c = probs_a[keep_030][clutter_mask].max(dim=-1).values
                        pb_c = probs_b[keep_030][clutter_mask].max(dim=-1).values
                        opt_a_clutter_rejected += int((pa_c < 0.35).sum().item())
                        opt_b_clutter_rejected += int((pb_c < 0.35).sum().item())

    denom_img = max(total_imgs, 1)
    denom_gt = max(total_gt, 1)
    denom_cls = max(gt_cls_total, 1)
    denom_clut = max(clutter_props_total, 1)
    return {
        "boxes_per_img_030": boxes_at_030 / denom_img,
        "overlap_pairs_025": overlap_pairs_025 / denom_img,
        "dup_gt_rate": dup_gt_hits / max(matched_gt_total, 1),
        "recall_10_50": recalled_10 / denom_gt,
        "recall_100_50": recalled_100 / denom_gt,
        "opt_a_top1": opt_a_gt_top1 / denom_cls,
        "opt_b_top1": opt_b_gt_top1 / denom_cls,
        "opt_ab_top1": opt_ab_gt_top1 / denom_cls,
        "opt_a_clutter_rej": opt_a_clutter_rejected / denom_clut,
        "opt_b_clutter_rej": opt_b_clutter_rejected / denom_clut,
        "evaluated_gt_cls": float(gt_cls_total),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="End-to-end SOTA Open-Vocabulary Detector Training (Option A + Option B)."
    )
    parser.add_argument("--geometry", choices=["hbb", "quad"], default="hbb")
    parser.add_argument("--gt-index", type=Path, default=DEFAULT_ENRICHED_GT_INDEX_PATH)
    parser.add_argument("--sam-db", type=Path, default=DEFAULT_SAM_ENRICHED_DB)
    parser.add_argument("--image-root", type=Path, default=DEFAULT_IMAGE_ROOT)
    parser.add_argument("--cache-path", type=Path, default=DEFAULT_CACHE_PATH)
    parser.add_argument("--vocab-dir", type=Path, default=DEFAULT_VOCAB_DIR)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--cohort-size", type=int, default=8000)
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--lr", type=float, default=4e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--input-size", type=int, default=384)
    parser.add_argument("--fpn-channels", type=int, default=128)
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--val-interval", type=int, default=2)
    parser.add_argument(
        "--resume-sota",
        action="store_true",
        help="Resume from existing sota_joint_best.pt instead of clean Stage-2 detector warm-start.",
    )
    parser.add_argument(
        "--semantic-refinement",
        action="store_true",
        help="Refine semantic heads toward SigLIP with 0.05x detector LR and calibrated priors.",
    )
    parser.add_argument(
        "--device", type=str, default="cuda" if torch.cuda.is_available() else "cpu"
    )
    return parser.parse_args()


def warmstart_detector_from_stage2(
    detector: nn.Module, ft_sd: dict[str, Tensor]
) -> None:
    """Function-preserving network morphism from 96-ch Stage-2 P2LiteFPN to 128-ch P2LitePAN + Dual-Head."""
    det_sd = detector.state_dict()
    with torch.no_grad():
        for k, p in det_sd.items():
            if k.startswith("backbone."):
                if k in ft_sd and ft_sd[k].shape == p.shape:
                    p.copy_(ft_sd[k])
            elif k.startswith(("fpn.", "head.", "aux_head.")):
                if k.endswith("running_var"):
                    p.fill_(1.0)
                elif k.endswith("running_mean") or k.endswith(".bias"):
                    p.zero_()
                elif k.endswith(".weight") and p.ndim == 1:
                    p.fill_(1.0)
                elif k.endswith(".weight") and p.ndim >= 2:
                    nn.init.normal_(p, std=1e-4)

        for idx, lvl in enumerate((2, 3, 4, 5)):
            for suf in ("weight", "bias"):
                s = ft_sd[f"fpn.lateral{lvl}.{suf}"]
                det_sd[f"fpn.laterals.{idx}.{suf}"][: s.shape[0]].copy_(s)
            for sub in (
                "0.weight",
                "1.weight",
                "2.weight",
                "2.bias",
                "2.running_mean",
                "2.running_var",
            ):
                s = ft_sd[f"fpn.output{lvl}.{sub}"]
                d = det_sd[f"fpn.td_outputs.{idx}.{sub}"]
                if s.ndim == 1 or s.shape[1] == 1:
                    d[: s.shape[0]].copy_(s)
                else:
                    d[: s.shape[0], : s.shape[1]].copy_(s)

        for idx in range(3):
            dw = det_sd[f"fpn.bu_outputs.{idx}.0.weight"]
            dw.zero_()
            for c in range(dw.shape[0]):
                dw[c, 0, 1, 1] = 1.0
            pw = det_sd[f"fpn.bu_outputs.{idx}.1.weight"]
            nn.init.normal_(pw, std=1e-4)
            for c in range(96):
                pw[c, :, 0, 0].zero_()
                pw[c, c, 0, 0] = 1.0

        for k, s in ft_sd.items():
            if not k.startswith("head."):
                continue
            for prefix in ("head.", "aux_head."):
                dst_k = prefix + k[len("head.") :]
                if dst_k not in det_sd:
                    continue
                d = det_sd[dst_k]
                if s.shape == d.shape:
                    d.copy_(s)
                elif s.ndim == 1 or s.shape[1] == 1:
                    d[: s.shape[0]].copy_(s)
                else:
                    d[: s.shape[0], : s.shape[1]].copy_(s)

        if "head.filter_3dmf.out_proj.weight" in det_sd:
            det_sd["head.filter_3dmf.out_proj.weight"].zero_()
        if "head.filter_3dmf.out_proj.bias" in det_sd:
            det_sd["head.filter_3dmf.out_proj.bias"].zero_()


def main() -> None:
    args = parse_args()
    device = torch.device(args.device)
    if args.output_dir is None:
        args.output_dir = Path(f"artifacts/sota_8k_{args.geometry}")
    ckpt_dir = args.output_dir / "checkpoints"
    ckpt_dir.mkdir(parents=True, exist_ok=True)

    strides = (4, 8, 16, 32)
    cat_to_idx = {cat: idx for idx, cat in enumerate(OPEN_VOCAB_CATEGORIES)}

    print(
        f"[{datetime.now().strftime('%H:%M:%S')}] Starting SOTA End-to-End Open-Vocab Training "
        f"({args.geometry.upper()}) on {device}"
    )

    # 1. Datasets + Strict Isolation Audit (cohort val + 210 holdout manifest)
    if args.geometry == "hbb":
        train_ds: Any = HumanGTHBBDataset(
            cohort_db_path=args.gt_index,
            gt_index_path=args.gt_index,
            image_root=args.image_root,
            split="train",
            input_size=args.input_size,
            cohort_size=args.cohort_size,
        )
        val_ds: Any = HumanGTHBBDataset(
            cohort_db_path=args.gt_index,
            gt_index_path=args.gt_index,
            image_root=args.image_root,
            split="val",
            input_size=args.input_size,
            cohort_size=args.cohort_size,
        )
        collate_fn = collate_proposal_samples
    else:
        train_ds = HumanGTQuadDataset(
            cohort_db_path=args.gt_index,
            gt_index_path=args.gt_index,
            image_root=args.image_root,
            split="train",
            input_size=args.input_size,
            cohort_size=args.cohort_size,
        )
        val_ds = HumanGTQuadDataset(
            cohort_db_path=args.gt_index,
            gt_index_path=args.gt_index,
            image_root=args.image_root,
            split="val",
            input_size=args.input_size,
            cohort_size=args.cohort_size,
        )
        collate_fn = collate_quad_proposal_samples

    val_sets: dict[str, list[dict[str, Any]]] = {"cohort": val_ds.images}
    if DEFAULT_HOLDOUT_MANIFEST.exists():
        raw_ho = json.loads(DEFAULT_HOLDOUT_MANIFEST.read_text())
        ho_imgs = raw_ho["images"] if isinstance(raw_ho, dict) and "images" in raw_ho else raw_ho
        val_sets["dashboard_holdout"] = ho_imgs

    class_counts, train_sampler = augment_train_with_rare_classes_and_rfs(
        train_ds=train_ds,
        val_sets=val_sets,
        gt_index_path=args.gt_index,
        cat_to_idx=cat_to_idx,
    )

    audit_training_isolation(
        train_ds.images,
        val_sets,
        image_root=args.image_root,
        identity_validation_sets=set(val_sets.keys()),
    )
    print(
        f"Isolation audit PASSED: train={len(train_ds)} | val={len(val_ds)} | "
        f"holdout={len(val_sets.get('dashboard_holdout', []))}"
    )

    # 2. Frozen SigLIP VocabularyBank + Flip-Safe FP16 Teacher Crop Cache
    vocab_bank, cache_offsets, cache_unflipped, cache_flipped = (
        ensure_vocab_banks_and_teacher_cache(
            train_images=train_ds.images,
            gt_index_path=args.gt_index,
            image_root=args.image_root,
            cache_path=args.cache_path,
            vocab_dir=args.vocab_dir,
            device=device,
            workers=args.workers,
        )
    )
    text_bank_gpu = vocab_bank.embeddings.to(device)

    # 3. Federated SAM pseudo-boxes across all 6 sources + teacher-verified open-world pseudo-GT
    all_sources = {
        "ade20k",
        "open_images_subset",
        "nuimages",
        "bdd100k_images_100k",
        "woodscape_rgb_fisheye",
        "coco_2017",
    }
    sam_ignore_map = load_sam_federated_ignore_boxes(
        args.sam_db,
        train_ds.images,
        sparse_sources=all_sources,
    )
    pseudo_gt_map: dict[int, dict[str, Any]] = {}
    if DEFAULT_OPEN_WORLD_PSEUDO_GT_CACHE.exists():
        pseudo_gt_map = torch.load(
            DEFAULT_OPEN_WORLD_PSEUDO_GT_CACHE, map_location="cpu", weights_only=False
        )
    sam_discovery_cache: dict[int, dict[str, Any]] = {}
    if DEFAULT_SAM_DISCOVERY_CACHE.exists():
        sam_discovery_cache = torch.load(
            DEFAULT_SAM_DISCOVERY_CACHE, map_location="cpu", weights_only=False
        )
    distractor_vocab_path = args.vocab_dir / "vocabulary_bank_distractors.pt"
    distractor_embeddings: Tensor | None = None
    if distractor_vocab_path.exists():
        distractor_embeddings = VocabularyBank.load(
            distractor_vocab_path, device=device
        ).embeddings
    visual_centroid_path = args.vocab_dir / "visual_centroid.pt"
    if not visual_centroid_path.exists():
        visual_centroid_path = DEFAULT_VISUAL_CENTROID_PATH
    visual_centroid_raw = (
        torch.load(visual_centroid_path, map_location=device, weights_only=False)
        if visual_centroid_path.exists()
        else None
    )
    visual_centroid: Tensor | None = (
        visual_centroid_raw["visual_centroid"].to(device)
        if isinstance(visual_centroid_raw, dict)
        and "visual_centroid" in visual_centroid_raw
        else visual_centroid_raw
    )
    print(
        f"Loaded federated SAM ignore pseudo-boxes for {len(sam_ignore_map)} images, "
        f"{len(sam_discovery_cache)} SAM SigLIP discovery images, "
        f"{len(distractor_embeddings) if distractor_embeddings is not None else 0} distractors, "
        f"visual_centroid={'yes' if visual_centroid is not None else 'no'}, "
        f"and {sum(len(v.get('categories', ())) for v in pseudo_gt_map.values())} "
        f"teacher-verified open-world pseudo-GT objects across {len(pseudo_gt_map)} driving images."
    )

    train_loader = DataLoader(
        train_ds,
        batch_size=args.batch_size,
        sampler=train_sampler,
        num_workers=args.workers,
        pin_memory=(device.type == "cuda"),
        collate_fn=collate_fn,
        drop_last=True,
        persistent_workers=(args.workers > 0),
    )
    val_loader = DataLoader(
        val_ds,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=(device.type == "cuda"),
        collate_fn=collate_fn,
        persistent_workers=(args.workers > 0),
    )

    # 4. Initialize SOTA Detector + Option A + Option B + Losses
    stage2_det_ckpt = Path(
        f"artifacts/finetune_8k_{args.geometry}_8k_full/checkpoints/mobilenetv4_p2_{args.geometry}_finetune_best.pt"
    )
    stage1_backbone_ckpt = Path(
        f"artifacts/train_8k_{args.geometry}_8k_full/checkpoints/mobilenetv4_p2_{args.geometry}_best.pt"
    )
    use_stage1_backbone = stage2_det_ckpt.exists() or stage1_backbone_ckpt.exists()

    if args.geometry == "hbb":
        detector: StudentDetector | QuadProposalDetector = StudentDetector(
            pretrained_backbone=not use_stage1_backbone,
            fpn_channels=args.fpn_channels,
            neck_type="pan",
            strides=strides,
            dual_head=True,
            use_3dmf=True,
        ).to(device)
        bg_weights = {
            "coco_2017": 0.35,
            "bdd100k_images_100k": 0.15,
            "nuimages": 0.15,
            "woodscape_rgb_fisheye": 0.15,
            "ade20k": 0.18,
            "open_images_subset": 0.18,
        }
        primary_builder: Any = TargetBuilder(
            TaskAlignedAssigner(
                strides=strides,
                alpha=1.0,
                beta=4.0,
                center_radius=2.5,
                assignment_mode="one_to_one",
            ),
            background_loss_weights=bg_weights,
            default_background_weight=0.18,
        )
        aux_builder: Any = TargetBuilder(
            TaskAlignedAssigner(
                strides=strides,
                top_k=10,
                alpha=1.0,
                beta=4.0,
                center_radius=2.5,
                assignment_mode="one_to_many",
            ),
            background_loss_weights=bg_weights,
            default_background_weight=0.18,
        )
        det_criterion: Any = ProposalLoss(
            strides=strides,
            objectness_loss="quality_focal",
            box_loss="ciou",
            box_weight=2.0,
            ltrb_weight=0.5,
            centerness_weight=0.0,
            objectness_normalization="num_positives",
            use_background_floor=False,
        )
        train_proposal_decoder: Any = InferenceDecoder(
            strides=strides,
            top_k=120,
            nms_iou_threshold=0.55,
            max_detections=24,
            score_mode="objectness",
            nms_mode="hard",
            score_threshold=0.05,
        )
        eval_decoder: Any = InferenceDecoder(
            strides=strides,
            top_k=500,
            nms_iou_threshold=0.50,
            max_detections=100,
            score_mode="objectness",
            nms_mode="containment_aware",
            score_threshold=0.01,
        )
    else:
        detector = QuadProposalDetector(
            pretrained_backbone=not use_stage1_backbone,
            fpn_channels=args.fpn_channels,
            neck_type="pan",
            strides=strides,
            dual_head=True,
            use_3dmf=True,
        ).to(device)
        primary_builder = QuadTargetBuilder(
            QuadTaskAlignedAssigner(
                strides=strides,
                alpha=1.0,
                beta=4.0,
                assignment_mode="one_to_one",
            ),
            weak_negative_weight=0.18,
        )
        aux_builder = QuadTargetBuilder(
            QuadTaskAlignedAssigner(
                strides=strides,
                top_k=8,
                alpha=1.0,
                beta=4.0,
                assignment_mode="one_to_many",
            ),
            weak_negative_weight=0.18,
        )
        det_criterion = QuadProposalLoss(
            strides=strides,
            corner_weight=2.0,
            corner_smooth_l1_beta=0.25,
            gwd_weight=1.5,
            validity_weight=0.10,
            quality_target_mode="iou",
            geometry_quality_target="exact_iou",
            quality_normalization="num_positives",
            use_proxy_quality_floor=False,
        )
        train_proposal_decoder = QuadInferenceDecoder(
            strides=strides,
            pre_nms_top_k=120,
            nms_iou_threshold=0.55,
            max_proposals=24,
            centerness_alpha=0.0,
            nms_mode="hard",
            score_threshold=0.05,
        )
        eval_decoder = QuadInferenceDecoder(
            strides=strides,
            pre_nms_top_k=500,
            nms_iou_threshold=0.50,
            max_proposals=100,
            centerness_alpha=0.0,
            nms_mode="containment_aware",
            score_threshold=0.01,
        )

    if stage2_det_ckpt.exists():
        s2_ckpt = torch.load(stage2_det_ckpt, map_location=device, weights_only=False)
        warmstart_detector_from_stage2(detector, s2_ckpt["model_state_dict"])
        print(
            f"Warm-started full Detector (Backbone + P2LitePAN + Dual-Head) from Stage-2 {stage2_det_ckpt}"
        )
    elif stage1_backbone_ckpt.exists():
        s1_ckpt = torch.load(stage1_backbone_ckpt, map_location=device, weights_only=False)
        bb_state = {
            k[len("backbone.") :]: v
            for k, v in s1_ckpt.get("model_state_dict", {}).items()
            if k.startswith("backbone.")
        }
        detector.backbone.load_state_dict(bb_state, strict=True)
        print(
            f"Warm-started MobileNetV4Backbone ({len(bb_state)} tensors) from Stage-1 {stage1_backbone_ckpt}"
        )

    head_a = DenseAnchorSemanticHead(
        in_channels=args.fpn_channels,
        hidden_channels=args.fpn_channels,
        embedding_dim=768,
    ).to(device)
    head_b = RoIPooledSemanticHead(
        in_channels=args.fpn_channels,
        roi_channels=96,
        embedding_dim=768,
        output_size=5,
        strides=strides,
    ).to(device)
    ov_criterion = OpenVocabularySigmoidLoss(
        class_counts=class_counts.to(device)
    ).to(device)

    head_a.set_distractor_embeddings(distractor_embeddings)
    head_b.set_distractor_embeddings(distractor_embeddings)
    if visual_centroid is not None:
        head_a.set_visual_centroid(visual_centroid)
        head_b.set_visual_centroid(visual_centroid)

    prev_sota_ckpt = ckpt_dir / "sota_joint_best.pt"
    if args.resume_sota and prev_sota_ckpt.exists():
        prev_payload = torch.load(
            prev_sota_ckpt, map_location=device, weights_only=False
        )
        if "model_state_dict" in prev_payload:
            detector.load_state_dict(prev_payload["model_state_dict"], strict=True)
        if "head_a_state_dict" in prev_payload:
            head_a.load_state_dict(prev_payload["head_a_state_dict"], strict=False)
        if "head_b_state_dict" in prev_payload:
            head_b.load_state_dict(prev_payload["head_b_state_dict"], strict=False)
        if distractor_embeddings is not None:
            head_a.set_distractor_embeddings(distractor_embeddings)
            head_b.set_distractor_embeddings(distractor_embeddings)
        if visual_centroid is not None:
            head_a.set_visual_centroid(visual_centroid)
            head_b.set_visual_centroid(visual_centroid)
        if not args.semantic_refinement:
            target_bias = math.log(0.008 / (1.0 - 0.008))
            for sem_head in (head_a, head_b):
                if float(sem_head.log_scale.detach().exp().item()) < 60.0:
                    with torch.no_grad():
                        sem_head.log_scale.fill_(math.log(80.0))
                        sem_head.logit_bias.fill_(target_bias)
        print(
            f"Resumed Detector + Option A + Option B from {prev_sota_ckpt}"
            + (
                " (semantic refinement: FROZEN detector + calibrated scale=90/bias=-4.40, SigLIP-primary heads only)"
                if args.semantic_refinement
                else ""
            )
        )

    if args.semantic_refinement:
        for param in detector.parameters():
            param.requires_grad_(False)
        detector.eval()
        for sem_head in (head_a, head_b):
            with torch.no_grad():
                sem_head.log_scale.fill_(math.log(90.0))
                sem_head.logit_bias.fill_(-4.40)
            sem_head.log_scale.requires_grad_(False)
            sem_head.logit_bias.requires_grad_(False)

    system = JointOpenVocabSystem(detector, head_a, head_b).to(device)
    ema = ExponentialMovingAverage(
        system, decay=0.998, ramp_steps=50 if (args.resume_sota and prev_sota_ckpt.exists()) else 150
    )

    # Parameter groups: under --semantic-refinement only head_a + head_b are trainable;
    # otherwise 0.20x LR on Stage-2 backbone/FPN/aux_head and 1.0x LR on primary/OptionA/OptionB.
    backbone_decay, backbone_nodecay = [], []
    other_decay, other_nodecay = [], []
    det_lr_mult = 0.20
    det_prefixes = ("detector.backbone.", "detector.fpn.", "detector.aux_head.")
    for name, param in system.named_parameters():
        if not param.requires_grad:
            continue
        is_nodecay = param.ndim <= 1 or name.endswith(".bias") or "log_scale" in name
        if name.startswith(det_prefixes):
            (backbone_nodecay if is_nodecay else backbone_decay).append(param)
        else:
            (other_nodecay if is_nodecay else other_decay).append(param)

    param_groups = [
        {"params": other_decay, "lr": args.lr, "weight_decay": args.weight_decay},
        {"params": other_nodecay, "lr": args.lr, "weight_decay": 0.0},
    ]
    if backbone_decay:
        param_groups.append(
            {"params": backbone_decay, "lr": args.lr * det_lr_mult, "weight_decay": args.weight_decay}
        )
    if backbone_nodecay:
        param_groups.append(
            {"params": backbone_nodecay, "lr": args.lr * det_lr_mult, "weight_decay": 0.0}
        )
    optimizer = torch.optim.AdamW(param_groups)
    total_steps = args.epochs * len(train_loader)
    warmup_steps = len(train_loader)
    warmup_sched = torch.optim.lr_scheduler.LinearLR(
        optimizer, start_factor=0.1, end_factor=1.0, total_iters=max(warmup_steps, 1)
    )
    cosine_sched = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=max(total_steps - warmup_steps, 1), eta_min=1e-5
    )
    scheduler = torch.optim.lr_scheduler.SequentialLR(
        optimizer, schedulers=[warmup_sched, cosine_sched], milestones=[warmup_steps]
    )
    scaler = torch.amp.GradScaler(device.type, enabled=(device.type == "cuda"))

    best_score = -1.0
    history: list[dict[str, Any]] = []
    train_start = time.time()

    for epoch in range(1, args.epochs + 1):
        ep_t0 = time.time()
        if args.semantic_refinement:
            system.detector.eval()
            system.head_a.train()
            system.head_b.train()
        else:
            system.train()
        train_ds.set_epoch(epoch)

        sum_det_loss = 0.0
        sum_ova_loss = 0.0
        sum_ovb_loss = 0.0
        steps = 0

        for images, raw_samples in train_loader:
            images = images.to(device, non_blocking=True)
            samples = inject_federated_ignore_into_samples(
                raw_samples,
                sam_ignore_map,
                args.input_size,
                args.geometry,
                pseudo_gt_by_id=pseudo_gt_map,
            )
            optimizer.zero_grad(set_to_none=True)

            with torch.amp.autocast(
                device.type, dtype=torch.float16, enabled=(device.type == "cuda")
            ):
                if args.semantic_refinement:
                    with torch.no_grad():
                        features = system.detector.forward_features(images)
                        detached_features = tuple(f.detach() for f in features)
                        primary_out = system.detector.head(detached_features)
                        assert system.detector.aux_head is not None
                        aux_out = system.detector.aux_head(features)
                else:
                    features = system.detector.forward_features(images)
                    # YOLOv10 Consistent Dual Assignments stop-gradient: rich 1-to-many
                    # aux_head + semantic heads (A & B) train backbone + P2LitePAN without
                    # 1-to-1 interior background penalties cancelling object features.
                    detached_features = tuple(f.detach() for f in features)
                    primary_out = system.detector.head(detached_features)
                    assert system.detector.aux_head is not None
                    aux_out = system.detector.aux_head(features)

                if args.geometry == "hbb":
                    f_shapes = tuple(
                        (t.shape[-2], t.shape[-1]) for t in primary_out.objectness  # type: ignore[union-attr]
                    )
                    flat_obj_scores, _, _ = flatten_detector_output(primary_out)  # type: ignore[arg-type]
                else:
                    f_shapes = tuple(
                        (t.shape[-2], t.shape[-1]) for t in primary_out.quality  # type: ignore[union-attr]
                    )
                    flat_obj_scores, _ = flatten_quad_output(primary_out)  # type: ignore[arg-type]

                with (
                    torch.no_grad(),
                    torch.amp.autocast(device.type, enabled=False),
                ):
                    live_dets = train_proposal_decoder(
                        primary_out, (args.input_size, args.input_size)
                    )
                    batch_src_mask = build_source_annotated_mask(
                        [s.source_dataset for s in samples],
                        OPEN_VOCAB_CATEGORIES,
                        device=device,
                    )
                    # Open-world teacher shield (only active when detector weights are updating):
                    # proposals on dataset `d` that score >= 0.65 on an UNANNOTATED class are
                    # routed to ignore_boxes/quads (capped at <= 6/img to prevent runaway filtering).
                    live_boxes_list = [d.boxes.detach() for d in live_dets]
                    total_live = sum(int(b.shape[0]) for b in live_boxes_list)
                    if False and total_live > 0 and not args.semantic_refinement:
                        z_b_live = system.head_b(
                            [f.float() for f in detached_features],
                            live_boxes_list,
                            image_shapes=[(args.input_size, args.input_size)]
                            * len(samples),
                        )
                        _, probs_b_live = system.head_b.classify_embeddings(
                            z_b_live, text_bank_gpu
                        )
                        cur_l = 0
                        shielded_samples: list[Any] = []
                        for b_i, s_i in enumerate(samples):
                            n_l = int(live_boxes_list[b_i].shape[0])
                            if n_l == 0:
                                shielded_samples.append(s_i)
                                continue
                            pb_i = probs_b_live[cur_l : cur_l + n_l]
                            cur_l += n_l
                            unann_c = ~batch_src_mask[b_i]
                            if not unann_c.any():
                                shielded_samples.append(s_i)
                                continue
                            max_unann_p = pb_i[:, unann_c].max(dim=-1).values
                            cand_shield = (live_dets[b_i].scores >= 0.12) & (
                                max_unann_p >= 0.65
                            )
                            gt_b_i = (
                                s_i.boxes.to(device)
                                if args.geometry == "hbb"
                                else RegionEmbeddingHead.boxes_from_quads(
                                    s_i.quads.to(device)
                                )
                            )
                            if gt_b_i.numel() > 0 and cand_shield.any():
                                max_gt_ov = (
                                    box_iou(live_boxes_list[b_i], gt_b_i)
                                    .max(dim=1)
                                    .values
                                )
                                cand_shield = cand_shield & (max_gt_ov < 0.20)
                            if cand_shield.any():
                                cand_idx = torch.where(cand_shield)[0]
                                if cand_idx.numel() > 6:
                                    top_k_rel = torch.topk(max_unann_p[cand_idx], k=6).indices
                                    capped_mask = torch.zeros_like(cand_shield)
                                    capped_mask[cand_idx[top_k_rel]] = True
                                    cand_shield = capped_mask
                                if args.geometry == "hbb":
                                    sh_b = live_boxes_list[b_i][cand_shield].cpu()
                                    merged_ig = (
                                        torch.cat([s_i.ignore_boxes, sh_b], dim=0)
                                        if s_i.ignore_boxes.numel()
                                        else sh_b
                                    )
                                    shielded_samples.append(
                                        replace(s_i, ignore_boxes=merged_ig)
                                    )
                                else:
                                    sh_q = (
                                        live_dets[b_i]
                                        .quads.detach()[cand_shield]
                                        .cpu()
                                    )
                                    merged_ig_q = (
                                        torch.cat([s_i.ignore_quads, sh_q], dim=0)
                                        if s_i.ignore_quads.numel()
                                        else sh_q
                                    )
                                    shielded_samples.append(
                                        replace(s_i, ignore_quads=merged_ig_q)
                                    )
                            else:
                                shielded_samples.append(s_i)
                        samples = shielded_samples

                    t_primary = primary_builder(
                        samples, f_shapes, device=device, pred_output=primary_out
                    )
                    t_aux = aux_builder(
                        samples, f_shapes, device=device, pred_output=aux_out
                    )

                if args.semantic_refinement:
                    loss_det = images.new_zeros((), dtype=torch.float32)
                else:
                    loss_p = det_criterion(primary_out, t_primary).total
                    loss_aux = det_criterion(aux_out, t_aux).total
                    loss_det = loss_p + loss_aux

                # Build batch labels & teacher targets for Option A (anchor-based) & Option B (RoI-based)
                a_batch_idx_list: list[Tensor] = []
                a_anchor_idx_list: list[Tensor] = []
                a_labels_list: list[Tensor] = []
                a_teacher_list: list[Tensor] = []
                a_has_t_list: list[Tensor] = []
                a_ann_mask_list: list[Tensor] = []

                b_boxes_per_img: list[Tensor] = []
                b_labels_list: list[Tensor] = []
                b_teacher_list: list[Tensor] = []
                b_has_t_list: list[Tensor] = []
                b_ann_mask_list: list[Tensor] = []

                assert t_primary.matched_gt_indices is not None
                assert t_aux.matched_gt_indices is not None

                level_offsets: list[int] = []
                cur_anchor_off = 0
                for h_l, w_l in f_shapes:
                    level_offsets.append(cur_anchor_off)
                    cur_anchor_off += int(h_l) * int(w_l)

                for b_i, sample in enumerate(samples):
                    gt_lbls, gt_t_emb, gt_has_t = map_sample_gt_labels_and_teachers(
                        sample,
                        cat_to_idx,
                        cache_offsets,
                        cache_unflipped,
                        cache_flipped,
                        pseudo_gt_by_id=pseudo_gt_map,
                    )
                    gt_lbls_gpu = gt_lbls.to(device)
                    gt_t_gpu = gt_t_emb.to(device)
                    gt_has_t_gpu = gt_has_t.to(device)
                    num_gt_b = int(gt_lbls_gpu.numel())
                    src_mask_row = batch_src_mask[b_i : b_i + 1]

                    # --- Option A anchor selection (1-to-1 U 1-to-many positives + hard & random bg) ---
                    pos_union = t_primary.positive_mask[b_i] | t_aux.positive_mask[b_i]
                    matched_union = torch.where(
                        t_primary.positive_mask[b_i],
                        t_primary.matched_gt_indices[b_i],
                        t_aux.matched_gt_indices[b_i],
                    )
                    pos_anchors = torch.where(pos_union & (matched_union >= 0))[0]
                    if pos_anchors.numel() > 0 and num_gt_b > 0:
                        m_gt = matched_union[pos_anchors].clamp(0, num_gt_b - 1)
                        n_pa = int(pos_anchors.numel())
                        a_batch_idx_list.append(torch.full_like(pos_anchors, b_i))
                        a_anchor_idx_list.append(pos_anchors)
                        a_labels_list.append(gt_lbls_gpu[m_gt])
                        a_teacher_list.append(gt_t_gpu[m_gt])
                        a_has_t_list.append(gt_has_t_gpu[m_gt])
                        a_ann_mask_list.append(src_mask_row.expand(n_pa, -1))

                    bg_cand_mask = t_aux.trusted_background_mask[b_i] | t_aux.weak_background_mask[b_i]
                    bg_cands = torch.where(bg_cand_mask & ~pos_union)[0]
                    if bg_cands.numel() > 0:
                        bg_scores = flat_obj_scores[b_i, bg_cands].detach()
                        max_hard = 16 if args.semantic_refinement else 48
                        max_rand = 24 if args.semantic_refinement else 96
                        k_hard = min(max_hard, int(bg_cands.numel()))
                        hard_top = torch.topk(bg_scores, k=k_hard).indices
                        hard_anchors = bg_cands[hard_top]
                        k_rand = min(max_rand, int(bg_cands.numel()))
                        rand_perm = torch.randperm(bg_cands.numel(), device=device)[:k_rand]
                        bg_sel = torch.unique(torch.cat([hard_anchors, bg_cands[rand_perm]]))
                        n_bg_sel = int(bg_sel.numel())
                        a_batch_idx_list.append(torch.full((n_bg_sel,), b_i, dtype=torch.long, device=device))
                        a_anchor_idx_list.append(bg_sel)
                        a_labels_list.append(torch.full((n_bg_sel,), -1, dtype=torch.long, device=device))
                        a_teacher_list.append(torch.zeros((n_bg_sel, 768), device=device))
                        a_has_t_list.append(torch.zeros((n_bg_sel,), dtype=torch.bool, device=device))
                        a_ann_mask_list.append(src_mask_row.expand(n_bg_sel, -1))

                    # --- Option B RoI selection (GT boxes + live 1-to-1 positive & clutter proposals) ---
                    gt_boxes_b = (
                        sample.boxes.to(device)
                        if args.geometry == "hbb"
                        else RegionEmbeddingHead.boxes_from_quads(sample.quads.to(device))
                    )
                    ignore_boxes_b = (
                        sample.ignore_boxes.to(device)
                        if args.geometry == "hbb"
                        else RegionEmbeddingHead.boxes_from_quads(sample.ignore_quads.to(device))
                    )
                    roi_boxes: list[Tensor] = []
                    roi_lbls: list[Tensor] = []
                    roi_t: list[Tensor] = []
                    roi_has_t: list[Tensor] = []

                    if num_gt_b > 0:
                        roi_boxes.append(gt_boxes_b)
                        roi_lbls.append(gt_lbls_gpu)
                        roi_t.append(gt_t_gpu)
                        roi_has_t.append(gt_has_t_gpu)

                    prop_boxes = live_dets[b_i].boxes.detach()
                    if prop_boxes.numel() > 0:
                        if num_gt_b > 0:
                            ious_pg = box_iou(prop_boxes, gt_boxes_b)
                            max_iou, arg_gt = ious_pg.max(dim=1)
                        else:
                            max_iou = torch.zeros((prop_boxes.shape[0],), device=device)
                            arg_gt = torch.zeros((prop_boxes.shape[0],), dtype=torch.long, device=device)

                        pos_p = max_iou >= 0.50
                        if pos_p.any() and num_gt_b > 0:
                            mg = arg_gt[pos_p]
                            roi_boxes.append(prop_boxes[pos_p])
                            roi_lbls.append(gt_lbls_gpu[mg])
                            roi_t.append(gt_t_gpu[mg])
                            roi_has_t.append(gt_has_t_gpu[mg])

                        neg_p = max_iou < 0.20
                        if neg_p.any() and ignore_boxes_b.numel() > 0:
                            ign_iou = box_iou(prop_boxes, ignore_boxes_b).max(dim=1).values
                            neg_p = neg_p & (ign_iou < 0.25)
                        if neg_p.any():
                            nb = prop_boxes[neg_p]
                            if args.semantic_refinement and nb.shape[0] > 6:
                                nb = nb[:6]
                            n_nb = int(nb.shape[0])
                            roi_boxes.append(nb)
                            roi_lbls.append(torch.full((n_nb,), -1, dtype=torch.long, device=device))
                            roi_t.append(torch.zeros((n_nb, 768), device=device))
                            roi_has_t.append(torch.zeros((n_nb,), dtype=torch.bool, device=device))

                    # --- Pure SigLIP SAM discovery distillation (label = -2, zero human GT penalty) ---
                    sam_rec = sam_discovery_cache.get(int(sample.image_id))
                    if sam_rec is not None and len(sam_rec.get("boxes", ())) > 0:
                        s_boxes = torch.tensor(sam_rec["boxes"], dtype=torch.float32)
                        orig_h, orig_w = sample.original_size
                        is_flip_s = bool(getattr(sample, "horizontal_flip", False))
                        scale_s_tf, off_x_s, off_y_s = sample.transform
                        if is_flip_s:
                            ox1_s = s_boxes[:, 0].clone()
                            ox2_s = s_boxes[:, 2].clone()
                            s_boxes[:, 0] = float(orig_w) - ox2_s
                            s_boxes[:, 2] = float(orig_w) - ox1_s
                            s_emb_raw = sam_rec["flipped"]
                        else:
                            s_emb_raw = sam_rec["unflipped"]
                        s_boxes = s_boxes * float(scale_s_tf)
                        s_boxes[:, 0::2] += float(off_x_s)
                        s_boxes[:, 1::2] += float(off_y_s)
                        s_boxes.clamp_(0.0, float(args.input_size))
                        val_s = (s_boxes[:, 2] - s_boxes[:, 0] >= 8.0) & (
                            s_boxes[:, 3] - s_boxes[:, 1] >= 8.0
                        )
                        if val_s.any():
                            sb_gpu = s_boxes[val_s].to(device)
                            se_gpu = s_emb_raw[val_s].to(device=device, dtype=torch.float32)
                            if gt_boxes_b.numel() > 0:
                                non_gt = box_iou(sb_gpu, gt_boxes_b).max(dim=1).values < 0.25
                                sb_gpu = sb_gpu[non_gt]
                                se_gpu = se_gpu[non_gt]
                            n_sam = int(sb_gpu.shape[0])
                            if n_sam > 0:
                                if n_sam > 12:
                                    sel_s = torch.randperm(n_sam, device=device)[:12]
                                    sb_gpu = sb_gpu[sel_s]
                                    se_gpu = se_gpu[sel_s]
                                    n_sam = 12
                                roi_boxes.append(sb_gpu)
                                roi_lbls.append(torch.full((n_sam,), -2, dtype=torch.long, device=device))
                                roi_t.append(se_gpu)
                                roi_has_t.append(torch.ones((n_sam,), dtype=torch.bool, device=device))

                                # Map SAM region centers + scales to FPN grid-center anchors for Option A
                                sb_cpu_list = sb_gpu.detach().cpu().tolist()
                                cand_a_idx: list[int] = []
                                cand_a_rows: list[int] = []
                                seen_a: set[int] = set()
                                for si, (bx1, by1, bx2, by2) in enumerate(sb_cpu_list):
                                    sc_box = math.sqrt(max(1.0, (bx2 - bx1) * (by2 - by1)))
                                    lvl = 0 if sc_box < 32.0 else (1 if sc_box < 64.0 else (2 if sc_box < 128.0 else 3))
                                    lvl = min(lvl, len(f_shapes) - 1)
                                    st_l = strides[lvl]
                                    h_l, w_l = int(f_shapes[lvl][0]), int(f_shapes[lvl][1])
                                    gx = min(max(int((0.5 * (bx1 + bx2)) // st_l), 0), w_l - 1)
                                    gy = min(max(int((0.5 * (by1 + by2)) // st_l), 0), h_l - 1)
                                    f_idx = level_offsets[lvl] + gy * w_l + gx
                                    if f_idx not in seen_a:
                                        seen_a.add(f_idx)
                                        cand_a_idx.append(f_idx)
                                        cand_a_rows.append(si)
                                if cand_a_idx:
                                    cand_a_t = torch.tensor(cand_a_idx, dtype=torch.long, device=device)
                                    cand_r_t = torch.tensor(cand_a_rows, dtype=torch.long, device=device)
                                    not_pos = ~pos_union[cand_a_t]
                                    if not_pos.any():
                                        kept_a = cand_a_t[not_pos]
                                        kept_r = cand_r_t[not_pos]
                                        n_ka = int(kept_a.numel())
                                        a_batch_idx_list.append(torch.full((n_ka,), b_i, dtype=torch.long, device=device))
                                        a_anchor_idx_list.append(kept_a)
                                        a_labels_list.append(torch.full((n_ka,), -2, dtype=torch.long, device=device))
                                        a_teacher_list.append(se_gpu[kept_r])
                                        a_has_t_list.append(torch.ones((n_ka,), dtype=torch.bool, device=device))
                                        a_ann_mask_list.append(src_mask_row.expand(n_ka, -1))

                    if roi_boxes:
                        cat_rb = torch.cat(roi_boxes, dim=0)
                        b_boxes_per_img.append(cat_rb)
                        b_labels_list.append(torch.cat(roi_lbls, dim=0))
                        b_teacher_list.append(torch.cat(roi_t, dim=0))
                        b_has_t_list.append(torch.cat(roi_has_t, dim=0))
                        b_ann_mask_list.append(
                            src_mask_row.expand(int(cat_rb.shape[0]), -1)
                        )
                    else:
                        b_boxes_per_img.append(torch.empty((0, 4), device=device))

                # Forward Option A & Option B losses
                if a_batch_idx_list:
                    z_a_all = system.head_a.forward_indexed(
                        features,
                        torch.cat(a_batch_idx_list),
                        torch.cat(a_anchor_idx_list),
                    )
                    loss_ov_a = ov_criterion(
                        system.head_a,
                        z_a_all,
                        text_bank_gpu,
                        torch.cat(a_labels_list),
                        teacher_embeddings=torch.cat(a_teacher_list),
                        has_teacher=torch.cat(a_has_t_list),
                        annotated_mask=torch.cat(a_ann_mask_list, dim=0),
                    ).total
                else:
                    loss_ov_a = loss_det * 0.0

                if b_labels_list:
                    z_b_all = system.head_b(
                        features,
                        b_boxes_per_img,
                        image_shapes=[(args.input_size, args.input_size)] * len(samples),
                    )
                    loss_ov_b = ov_criterion(
                        system.head_b,
                        z_b_all,
                        text_bank_gpu,
                        torch.cat(b_labels_list),
                        teacher_embeddings=torch.cat(b_teacher_list),
                        has_teacher=torch.cat(b_has_t_list),
                        annotated_mask=torch.cat(b_ann_mask_list, dim=0),
                    ).total
                else:
                    loss_ov_b = loss_det * 0.0

                total_loss = loss_det + 0.5 * loss_ov_a + 0.5 * loss_ov_b

            scaler.scale(total_loss).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(system.parameters(), max_norm=10.0)
            scaler.step(optimizer)
            scaler.update()
            scheduler.step()
            ema.update(system)

            sum_det_loss += float(loss_det.detach().item())
            sum_ova_loss += float(loss_ov_a.detach().item())
            sum_ovb_loss += float(loss_ov_b.detach().item())
            steps += 1

        ep_sec = time.time() - ep_t0
        avg_det = sum_det_loss / max(steps, 1)
        avg_ova = sum_ova_loss / max(steps, 1)
        avg_ovb = sum_ovb_loss / max(steps, 1)

        should_eval = (epoch % args.val_interval == 0) or (epoch == args.epochs) or (epoch == 1)
        if should_eval:
            val_m = evaluate_joint_epoch(
                ema.module,
                val_loader,
                eval_decoder,
                vocab_bank,
                cat_to_idx,
                args.geometry,
                args.input_size,
                device,
            )
            # Composite quality score balancing localization recall, semantic top-1, and low overlaps
            composite = (
                0.40 * val_m["recall_100_50"]
                + 0.30 * val_m["opt_a_top1"]
                + 0.30 * val_m["opt_b_top1"]
                - 0.05 * val_m["dup_gt_rate"]
            )
            is_best = composite > best_score
            if is_best:
                best_score = composite
            record = {
                "epoch": epoch,
                "epoch_sec": round(ep_sec, 1),
                "loss_det": round(avg_det, 4),
                "loss_ov_a": round(avg_ova, 4),
                "loss_ov_b": round(avg_ovb, 4),
                "composite_score": round(composite, 4),
                **{k: round(v, 4) for k, v in val_m.items()},
            }
            history.append(record)
            print(
                f"Epoch {epoch:02d}/{args.epochs} ({ep_sec:.1f}s) | "
                f"L_det={avg_det:.3f} L_A={avg_ova:.3f} L_B={avg_ovb:.3f} | "
                f"R@100={val_m['recall_100_50']*100:.1f}% Boxes@0.3={val_m['boxes_per_img_030']:.1f} "
                f"OvlpPairs={val_m['overlap_pairs_025']:.1f} DupGT={val_m['dup_gt_rate']*100:.1f}% | "
                f"Top1[A={val_m['opt_a_top1']*100:.1f}% B={val_m['opt_b_top1']*100:.1f}% "
                f"AB={val_m['opt_ab_top1']*100:.1f}%] "
                f"ClutRej[A={val_m['opt_a_clutter_rej']*100:.1f}% B={val_m['opt_b_clutter_rej']*100:.1f}%]"
                + (" *BEST*" if is_best else ""),
                flush=True,
            )

            ema_sys: JointOpenVocabSystem = ema.module
            payload = {
                "schema_version": "siglip_primary_v2",
                "epoch": epoch,
                "geometry": args.geometry,
                "detector_config": {
                    "fpn_channels": args.fpn_channels,
                    "neck_type": "pan",
                    "strides": strides,
                    "dual_head": True,
                    "use_3dmf": True,
                },
                "model_state_dict": ema_sys.detector.state_dict(),
                "head_a_config": ema_sys.head_a.get_config(),
                "head_a_state_dict": ema_sys.head_a.state_dict(),
                "head_b_config": ema_sys.head_b.get_config(),
                "head_b_state_dict": ema_sys.head_b.state_dict(),
                "vocab_categories": list(OPEN_VOCAB_CATEGORIES),
                "metrics": val_m,
            }
            torch.save(payload, ckpt_dir / "sota_joint_last.pt")
            if is_best:
                torch.save(payload, ckpt_dir / "sota_joint_best.pt")
        else:
            print(
                f"Epoch {epoch:02d}/{args.epochs} ({ep_sec:.1f}s) | "
                f"L_det={avg_det:.3f} L_A={avg_ova:.3f} L_B={avg_ovb:.3f}",
                flush=True,
            )

    summary = {
        "geometry": args.geometry,
        "epochs": args.epochs,
        "total_minutes": round((time.time() - train_start) / 60.0, 2),
        "best_composite_score": round(best_score, 4),
        "history": history,
    }
    (args.output_dir / "training_summary.json").write_text(json.dumps(summary, indent=2))
    print(
        f"[{datetime.now().strftime('%H:%M:%S')}] Finished {args.geometry.upper()} SOTA training "
        f"in {summary['total_minutes']:.1f} min. Best checkpoint: {ckpt_dir / 'sota_joint_best.pt'}"
    )


if __name__ == "__main__":
    main()
