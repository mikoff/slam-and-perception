#!/usr/bin/env python3
"""Phase 2: Unfrozen 384x384 Oriented-Quad Open-Vocabulary Training on OWLv2-30k Cache.

Trains MobileNetV4 + 128-ch P2LitePAN + Dual-Head QuadProposalDetector (1-to-1 +
3DMF primary, 1-to-many TAL auxiliary) jointly with Option A
(DenseAnchorSemanticHead) and Option B (RoIPooledSemanticHead) in native OWLv2
768-D space:
  - Uses Clean Human GT quads (annotation_hygiene.py) + OWLv2 crowd-pedestrian
    splits + OWLv2 missing-class & bollard/pillar pseudo-labels converted via
    quad_from_bbox.
  - Warm-starts unfrozen QuadProposalDetector from `sota_8k_quad` (`model_state_dict`)
    with fresh OWLv2 Option A / Option B semantic heads and zero core prototype residuals
    (mirroring the HBB OWLv2 Phase 2 warm-start recipe), or resumes from an OWLv2 Quad checkpoint.
  - Evaluates sanitized `HumanGTQuadDataset(split='val')` with exact convex
    polygon IoU and saves best/last checkpoints + adapted Core-27/31/37/41 banks.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import copy
from datetime import datetime
import json
from pathlib import Path
import sqlite3
import time
from typing import Any

from PIL import Image
import torch
from torch import Tensor, nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler

if hasattr(torch.multiprocessing, "set_sharing_strategy"):
    torch.multiprocessing.set_sharing_strategy("file_system")

from student_detector.annotation_hygiene import RawAnnotationRow, sanitize_image_annotations
from student_detector.config import AugmentationConfig
from student_detector.human_gt_data import (
    DEFAULT_ENRICHED_GT_INDEX_PATH,
    HumanGTQuadDataset,
    collate_quad_proposal_samples,
)
from student_detector.model import QuadProposalDetector
from student_detector.ontology import (
    CLOSED_MERGE_MAP,
    CORE_27_CATEGORIES,
    OPEN_VOCAB_CATEGORIES,
    PRUNED_SCARCE_CATEGORIES,
    SOURCE_ANNOTATED_CATEGORIES,
    ZERO_SHOT_EVAL_CATEGORIES,
    map_to_canonical,
    normalize_category_name,
)
from student_detector.open_vocab_heads import (
    DenseAnchorSemanticHead,
    OpenVocabularySigmoidLoss,
    RoIPooledSemanticHead,
    filter_semantic_detections,
)
from student_detector.owlv2_teacher import OwlTeacherCacheReader
from student_detector.quad_data import (
    QuadProposalSample,
    QuadProposalTransform,
    _quad_slice_labels,
    _quad_tensor,
)
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_geometry import pairwise_quad_iou, quad_from_bbox
from student_detector.quad_losses import QuadProposalLoss, flatten_quad_output
from student_detector.quad_targets import QuadTargetBuilder, point_validity_from_pixel_mask
from student_detector.region_head import RegionEmbeddingHead
from student_detector.sam_student_data import CAMERA_TYPES, DEFAULT_IMAGE_ROOT, SOURCE_DOMAINS
from student_detector.task_aligned_assigner import QuadTaskAlignedAssigner
from student_detector.training_optimization import ExponentialMovingAverage, freeze_backbone_batch_norm
from student_detector.vocabulary import VocabularyBank

assert len(CORE_27_CATEGORIES) == 27

OWL_STREET_LIGHT_MIN_SCORE: float = 0.36
ULTRA_RARE_SAMPLER_CLASSES: frozenset[str] = frozenset(
    {"trash_bin", "stroller", "fire_hydrant", "bench", "signboard", "dog"}
)
MED_RARE_SAMPLER_CLASSES: frozenset[str] = frozenset(
    {
        "bus",
        "truck",
        "license_plate",
        "construction_barrier",
        "traffic_cone",
        "luggage_and_bags",
        "fence",
    }
)


class JointOpenVocabQuadSystem(nn.Module):
    """Bundle QuadProposalDetector + Option A + Option B + 27-core prototype residuals."""

    def __init__(
        self,
        detector: QuadProposalDetector,
        head_a: DenseAnchorSemanticHead,
        head_b: RoIPooledSemanticHead,
        frozen_core_27: Tensor,
    ) -> None:
        """/// Initialize joint oriented-quad open-vocabulary training wrapper.

        /// @param detector Dual-head QuadProposalDetector module.
        /// @param head_a Dense anchor-indexed 768-D semantic head (Option A).
        /// @param head_b Dual-scale RoI-pooled 768-D semantic head (Option B).
        /// @param frozen_core_27 Frozen OWLv2 Core-27 text prototypes [27, 768].
        """
        super().__init__()
        self.detector = detector
        self.head_a = head_a
        self.head_b = head_b
        self.register_buffer("frozen_core_27", frozen_core_27.float().clone(), persistent=False)
        self.core_proto_delta = nn.Parameter(
            torch.zeros((len(CORE_27_CATEGORIES), frozen_core_27.shape[1]), dtype=torch.float32)
        )

    def adapted_core_prototypes(self) -> Tensor:
        """/// Return unit-normalized 27-core adapted OWLv2 prototypes.

        /// @return L2-normalized [27, 768] prototype tensor.
        """
        return F.normalize(self.frozen_core_27 + self.core_proto_delta, p=2.0, dim=-1)


def build_adapted_bank(
    base_vocab: VocabularyBank,
    adapted_core_27: Tensor,
) -> VocabularyBank:
    """Slot adapted 27-core rows back into 37-seen or 41-zeroshot VocabularyBank."""
    core_map = {cat: idx for idx, cat in enumerate(CORE_27_CATEGORIES)}
    new_emb = base_vocab.embeddings.detach().float().cpu().clone()
    adapted_cpu = adapted_core_27.detach().float().cpu()
    for out_idx, cat in enumerate(base_vocab.categories):
        if cat in core_map:
            new_emb[out_idx] = adapted_cpu[core_map[cat]]
    return VocabularyBank(
        categories=base_vocab.categories,
        embeddings=new_emb,
        logit_scale=base_vocab.logit_scale,
        logit_bias=base_vocab.logit_bias,
        class_priors=base_vocab.class_priors,
        centroid=None,
    )


def build_closed_27_annotated_mask(
    source_datasets: list[str],
    device: torch.device,
) -> Tensor:
    """Build [B, 27] mask marking which CORE_27 classes each source dataset labels."""
    mask = torch.ones((len(source_datasets), len(CORE_27_CATEGORIES)), dtype=torch.bool)
    for row_idx, src in enumerate(source_datasets):
        ann = SOURCE_ANNOTATED_CATEGORIES.get(src)
        if ann is None:
            continue
        for col_idx, cat in enumerate(CORE_27_CATEGORIES):
            if cat == "pole":
                is_ann = ("pole" in ann) or ("street_light" in ann)
            else:
                is_ann = cat in ann
            mask[row_idx, col_idx] = is_ann
    return mask.to(device=device)


def _parse_quad_row(quad_json: str | None, hr: RawAnnotationRow) -> list[list[float]]:
    if quad_json:
        parsed = json.loads(quad_json)
        if isinstance(parsed, list) and len(parsed) == 4:
            return [[float(pt[0]), float(pt[1])] for pt in parsed]
    return quad_from_bbox([hr.x1, hr.y1, hr.x2, hr.y2]).tolist()


def load_sanitized_image_quads(
    conn: sqlite3.Connection,
    image_id: int,
    source_dataset: str,
) -> tuple[
    list[list[list[float]]],
    list[str | None],
    list[list[list[float]]],
    list[list[list[float]]],
]:
    """Fetch cleaned Human GT quads, ignore quads (incl. WoodScape demotions), and trusted bg."""
    rows = conn.execute(
        """
        SELECT quad_json, x1, y1, x2, y2, ignore_region, category_name, geometry_tier
        FROM annotations
        WHERE image_id = ?
        ORDER BY rowid
        """,
        (int(image_id),),
    ).fetchall()
    hygiene_rows = [
        RawAnnotationRow(
            x1=float(r[1]),
            y1=float(r[2]),
            x2=float(r[3]),
            y2=float(r[4]),
            ignore_region=int(r[5]),
            category_name=str(r[6]),
            geometry_tier=str(r[7]),
        )
        for r in rows
    ]
    dec = sanitize_image_annotations(hygiene_rows, source_dataset=source_dataset)
    kept_set = set(dec.kept_positive_indices)
    demoted_set = set(dec.demoted_to_ignore_indices)
    dropped_set = set(dec.dropped_indices)

    pos_quads: list[list[list[float]]] = []
    pos_canon: list[str | None] = []
    ignore_quads: list[list[list[float]]] = []
    trusted_bg: list[list[list[float]]] = []

    for idx, (row, hr) in enumerate(zip(rows, hygiene_rows, strict=True)):
        if idx in dropped_set:
            continue
        quad = _parse_quad_row(row[0], hr)
        if hr.ignore_region == 0:
            if idx in demoted_set:
                ignore_quads.append(quad)
            elif idx in kept_set:
                pos_quads.append(quad)
                pos_canon.append(map_to_canonical(hr.category_name))
        elif hr.ignore_region == 2:
            trusted_bg.append(quad)
        else:
            ignore_quads.append(quad)
    return pos_quads, pos_canon, ignore_quads, trusted_bg


class OwlTeacher30kQuadDataset(Dataset[tuple[QuadProposalSample, dict[str, Tensor]]]):
    """Dataset streaming Clean Human GT quads + OWLv2 pseudo-labels + 768-D embeddings."""

    def __init__(
        self,
        *,
        cache_db_path: Path,
        gt_index_path: Path,
        image_root: Path,
        holdout_manifest: Path,
        input_size: int = 384,
        seed: int = 42,
    ) -> None:
        """/// Initialize leak-verified 30k OWLv2 quad training dataset.

        /// @param cache_db_path Path to patched OWLv2 teacher cache SQLite.
        /// @param gt_index_path Path to `quad_train_enriched.sqlite`.
        /// @param image_root Monorepo image dataset root directory.
        /// @param holdout_manifest Holdout manifest checked for zero data leakage.
        /// @param input_size Square canvas size (default 384).
        /// @param seed Base deterministic augmentation seed.
        """
        super().__init__()
        self.cache_db_path = Path(cache_db_path)
        self.gt_index_path = Path(gt_index_path)
        self.image_root = Path(image_root)
        self.input_size = input_size
        self.seed = seed
        self._epoch = torch.zeros(1, dtype=torch.long)

        self._gt_conn: sqlite3.Connection | None = None
        self._cache_reader: OwlTeacherCacheReader | None = None

        self.cat37_to_idx = {c: i for i, c in enumerate(OPEN_VOCAB_CATEGORIES)}
        self.core27_to_idx = {c: i for i, c in enumerate(CORE_27_CATEGORIES)}

        # Assert strict zero overlap against val / holdout manifest before training.
        excluded_ids: set[int] = set()
        if holdout_manifest.exists():
            raw = json.loads(holdout_manifest.read_text())
            for key in ("images", "excluded_validation_images"):
                for rec in raw.get(key, []):
                    if rec.get("image_id") is not None:
                        excluded_ids.add(int(rec["image_id"]))
        val_ds = HumanGTQuadDataset(
            cohort_db_path=gt_index_path,
            gt_index_path=gt_index_path,
            image_root=image_root,
            split="val",
            cohort_size=8000,
            sanitize_annotations=False,
        )
        excluded_ids.update(int(r["image_id"]) for r in val_ds.images)

        self.transform = QuadProposalTransform(
            input_size=input_size,
            augmentation=AugmentationConfig(
                horizontal_flip_probability=0.5,
                color_jitter_probability=0.5,
                blur_probability=0.1,
                noise_probability=0.1,
                jpeg_probability=0.1,
            ),
            training=True,
            regular_min_side=2.0,
        )

        reader = OwlTeacherCacheReader(self.cache_db_path)
        gt_conn = sqlite3.connect(f"file:{self.gt_index_path}?mode=ro&immutable=1", uri=True)
        img_file_map = {
            int(r[0]): (str(r[1]), int(r[2]), int(r[3]))
            for r in gt_conn.execute("SELECT image_id, file_name, width, height FROM images").fetchall()
        }

        self.records: list[dict[str, Any]] = []
        self.sample_weights: list[float] = []
        core_counts: dict[str, int] = defaultdict(int)

        for rec in reader.iter_records():
            if rec.image_id in excluded_ids:
                raise RuntimeError(f"Data leak! Cached image_id={rec.image_id} overlaps val/holdout.")
            meta = img_file_map.get(rec.image_id)
            if meta is None:
                continue
            fn, w, h = meta
            _pos_q, pos_canon, _ig, _bg = load_sanitized_image_quads(
                gt_conn, rec.image_id, rec.source_dataset
            )
            sup_set = set(rec.superseded_gt_indices)
            rare_tier = 0

            for idx, cname in enumerate(pos_canon):
                if idx in sup_set or cname is None:
                    continue
                if cname in ULTRA_RARE_SAMPLER_CLASSES:
                    rare_tier = 2
                elif cname in MED_RARE_SAMPLER_CLASSES and rare_tier < 1:
                    rare_tier = 1
                mapped = CLOSED_MERGE_MAP.get(cname, cname)
                if mapped in self.core27_to_idx:
                    core_counts[mapped] += 1

            for o_idx, o_lbl in enumerate(rec.owl_labels):
                if o_lbl == "street_light" and float(rec.owl_scores[o_idx]) < OWL_STREET_LIGHT_MIN_SCORE:
                    continue
                if o_lbl in ULTRA_RARE_SAMPLER_CLASSES:
                    rare_tier = 2
                elif o_lbl in MED_RARE_SAMPLER_CLASSES and rare_tier < 1:
                    rare_tier = 1
                mapped = CLOSED_MERGE_MAP.get(o_lbl, o_lbl)
                if mapped in self.core27_to_idx:
                    core_counts[mapped] += 1

            self.records.append(
                {
                    "image_id": rec.image_id,
                    "file_name": fn,
                    "source_dataset": rec.source_dataset,
                    "width": w,
                    "height": h,
                }
            )
            self.sample_weights.append(2.0 if rare_tier == 2 else (1.4 if rare_tier == 1 else 1.0))

        reader.close()
        gt_conn.close()

        self.core_class_counts = torch.tensor(
            [float(max(core_counts[c], 1)) for c in CORE_27_CATEGORIES],
            dtype=torch.float32,
        )

    def set_epoch(self, epoch: int) -> None:
        self._epoch.fill_(epoch)

    def __getstate__(self) -> dict[str, Any]:
        st = dict(self.__dict__)
        st["_gt_conn"] = None
        st["_cache_reader"] = None
        return st

    def _connections(self) -> tuple[sqlite3.Connection, OwlTeacherCacheReader]:
        if self._gt_conn is None:
            self._gt_conn = sqlite3.connect(
                f"file:{self.gt_index_path}?mode=ro&immutable=1",
                uri=True,
                check_same_thread=False,
            )
        if self._cache_reader is None:
            self._cache_reader = OwlTeacherCacheReader(self.cache_db_path)
        return self._gt_conn, self._cache_reader

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, index: int) -> tuple[QuadProposalSample, dict[str, Tensor]]:
        rec_meta = self.records[index]
        img_id = int(rec_meta["image_id"])
        src_ds = str(rec_meta["source_dataset"])
        gt_conn, reader = self._connections()

        pos_quads, pos_canon, ignore_quads, trusted_bg = load_sanitized_image_quads(
            gt_conn, img_id, src_ds
        )
        cached = reader.get_image(img_id)
        if cached is None:
            raise KeyError(f"Missing cached record for image_id={img_id}")

        sup_set = set(cached.superseded_gt_indices)
        merged_quads: list[list[list[float]]] = []
        merged_labels: list[str | None] = []
        merged_embs: list[Tensor] = []
        merged_valid_t: list[bool] = []

        for g_idx, (qd, cname) in enumerate(zip(pos_quads, pos_canon, strict=True)):
            if g_idx in sup_set:
                continue
            merged_quads.append(qd)
            merged_labels.append(cname)
            if g_idx < cached.gt_embeddings.shape[0]:
                merged_embs.append(torch.from_numpy(cached.gt_embeddings[g_idx].astype("float32")))
                merged_valid_t.append(float(cached.gt_match_ious[g_idx]) >= 0.40)
            else:
                merged_embs.append(torch.zeros((768,), dtype=torch.float32))
                merged_valid_t.append(False)

        for o_idx, o_lbl in enumerate(cached.owl_labels):
            if o_lbl == "street_light" and float(cached.owl_scores[o_idx]) < OWL_STREET_LIGHT_MIN_SCORE:
                continue
            box_q = quad_from_bbox(
                torch.tensor([float(v) for v in cached.owl_boxes[o_idx].tolist()], dtype=torch.float32)
            ).tolist()
            merged_quads.append(box_q)
            merged_labels.append(o_lbl)
            merged_embs.append(torch.from_numpy(cached.owl_embeddings[o_idx].astype("float32")))
            merged_valid_t.append(True)

        img_path = self.image_root / rec_meta["file_name"]
        with Image.open(img_path) as raw_im:
            image = raw_im.convert("RGB")

        seed = self.seed + int(self._epoch.item()) * max(len(self), 1) + index
        transformed = self.transform(
            image,
            _quad_tensor(merged_quads),
            _quad_tensor(ignore_quads),
            _quad_tensor(trusted_bg),
            seed=seed,
            return_metadata=True,
        )
        # QuadProposalTransform returns an 8-tuple with retained at [6], horizontal_flip at [7].
        quads_t: Tensor = transformed[1]
        retained: tuple[int, ...] = transformed[6]
        horizontal_flip = bool(transformed[7])

        k_ret = len(retained)
        lbl37_list: list[int] = []
        closed_list: list[int] = []
        t_emb_list: list[Tensor] = []
        has_t_list: list[bool] = []

        for loc_i, orig_i in enumerate(retained):
            cname = merged_labels[orig_i]
            l37 = self.cat37_to_idx[cname] if (cname in self.cat37_to_idx) else -2
            mapped = CLOSED_MERGE_MAP.get(cname, cname) if cname is not None else None
            if mapped in PRUNED_SCARCE_CATEGORIES or mapped is None:
                l27 = -2
            elif mapped in self.core27_to_idx:
                l27 = self.core27_to_idx[mapped]
            else:
                l27 = -2

            bw = float(quads_t[loc_i, :, 0].max() - quads_t[loc_i, :, 0].min())
            bh = float(quads_t[loc_i, :, 1].max() - quads_t[loc_i, :, 1].min())
            is_large_enough = min(bw, bh) >= 10.0
            lbl37_list.append(l37)
            closed_list.append(l27)
            t_emb_list.append(merged_embs[orig_i])
            has_t_list.append(bool(merged_valid_t[orig_i] and is_large_enough))

        size_bins, aspect_bins, radial_bins = _quad_slice_labels(quads_t, self.input_size)
        sample = QuadProposalSample(
            image=transformed[0],
            quads=quads_t,
            ignore_quads=transformed[2],
            valid_mask=transformed[4],
            image_id=img_id,
            source_dataset=src_ds,
            domain=SOURCE_DOMAINS.get(src_ds, "general"),
            camera_type=CAMERA_TYPES.get(src_ds, "pinhole"),
            background_supervision=bool(transformed[3].numel()),
            original_size=(int(rec_meta["height"]), int(rec_meta["width"])),
            transform=transformed[5],
            trusted_background_quads=transformed[3],
            size_bins=size_bins,
            aspect_bins=aspect_bins,
            radial_bins=radial_bins,
            file_name=str(rec_meta["file_name"]),
            category_names=tuple((merged_labels[i] or "unknown") for i in retained),
            horizontal_flip=horizontal_flip,
        )
        targets = {
            "labels_37": torch.tensor(lbl37_list, dtype=torch.long),
            "closed_labels": torch.tensor(closed_list, dtype=torch.long),
            "teacher_embs": (
                torch.stack(t_emb_list, dim=0)
                if k_ret > 0
                else torch.empty((0, 768), dtype=torch.float32)
            ),
            "has_teacher": torch.tensor(has_t_list, dtype=torch.bool),
        }
        return sample, targets


def collate_owl30k_quad_batch(
    items: list[tuple[QuadProposalSample, dict[str, Tensor]]],
) -> tuple[Tensor, list[QuadProposalSample], list[dict[str, Tensor]]]:
    samples = [it[0] for it in items]
    metas = [it[1] for it in items]
    images, collated_samples = collate_quad_proposal_samples(samples)
    return images, collated_samples, metas


def apply_warmstart_or_resume(
    *,
    detector: QuadProposalDetector,
    head_a: DenseAnchorSemanticHead,
    head_b: RoIPooledSemanticHead,
    warmstart_ckpt: Path,
    resume_owlv2_quad_ckpt: Path | None,
    device: torch.device,
) -> tuple[bool, Tensor | None]:
    """Load resumed OWLv2 quad checkpoint or warm-start unfrozen Quad detector."""
    resumed_quad = resume_owlv2_quad_ckpt is not None and resume_owlv2_quad_ckpt.exists()
    if resumed_quad:
        assert resume_owlv2_quad_ckpt is not None
        res = torch.load(resume_owlv2_quad_ckpt, map_location=device, weights_only=False)
        detector.load_state_dict(res["model_state_dict"], strict=True)
        head_a.load_state_dict(res["head_a_state_dict"], strict=False)
        head_b.load_state_dict(res["head_b_state_dict"], strict=False)
        init_delta = res["core_proto_delta"].to(device) if "core_proto_delta" in res else None
        print(
            f"  Resumed full OWLv2 Quad system (detector + Option A/B + proto_delta) from {resume_owlv2_quad_ckpt}",
            flush=True,
        )
        return True, init_delta

    if not warmstart_ckpt.exists():
        raise FileNotFoundError(f"Warmstart Quad checkpoint not found: {warmstart_ckpt}")
    ws = torch.load(warmstart_ckpt, map_location=device, weights_only=False)
    detector.load_state_dict(ws["model_state_dict"], strict=True)
    print(f"  Warm-started unfrozen Quad detector from {warmstart_ckpt}", flush=True)
    return False, None


@torch.no_grad()
def evaluate_owlv2_quad_val(
    system: JointOpenVocabQuadSystem,
    val_loader: DataLoader[QuadProposalSample],
    decoder: QuadInferenceDecoder,
    vocab_frozen_37: VocabularyBank,
    vocab_frozen_41: VocabularyBank,
    vocab_adapted_37: VocabularyBank,
    input_size: int,
    device: torch.device,
) -> dict[str, float]:
    """Evaluate oriented-quad localization + Frozen/Adapted 37-class Opt A/B/AB + ZeroShot-4."""
    system.eval()
    strides = (4, 8, 16, 32)
    cat37_to_idx = {c: i for i, c in enumerate(OPEN_VOCAB_CATEGORIES)}
    zs_to_41 = {
        c: len(OPEN_VOCAB_CATEGORIES) + i
        for i, c in enumerate(ZERO_SHOT_EVAL_CATEGORIES)
    }
    zs_start = len(OPEN_VOCAB_CATEGORIES)

    t37_frozen = vocab_frozen_37.embeddings.to(device)
    t37_adapted = vocab_adapted_37.embeddings.to(device)
    t41_frozen = vocab_frozen_41.embeddings.to(device)

    total_gt = 0
    rec50 = 0
    rec75 = 0
    matched_030 = 0
    dup_030 = 0
    boxes_030 = 0
    agn_records: list[tuple[float, int]] = []
    ds_agn_records: dict[str, list[tuple[float, int]]] = defaultdict(list)
    ds_total_gt: dict[str, int] = defaultdict(int)

    prop_total = 0
    opt_b_frozen_top1 = 0
    opt_b_adapted_top1 = 0
    opt_ab_adapted_top1 = 0
    zs_total = 0
    zs_b_4way_top1 = 0
    post_sem_ovlp_025 = 0
    num_imgs = 0

    for images, samples in val_loader:
        images = images.to(device, non_blocking=True)
        features = system.detector.forward_features(images)
        raw_out = system.detector.forward_eval_head(features)
        shapes = tuple((t.shape[-2], t.shape[-1]) for t in raw_out.quality)
        lvl_masks = [
            point_validity_from_pixel_mask(s.valid_mask.to(device), shapes, strides)[1]
            for s in samples
        ]
        batched_masks = tuple(
            torch.stack([m[lvl] for m in lvl_masks], dim=0) for lvl in range(len(strides))
        )
        dets = decoder(raw_out, (input_size, input_size), batched_masks)
        tower_a = system.head_a.forward_tower(features)
        z_b_all = system.head_b(
            features,
            [d.boxes for d in dets],
            image_shapes=[(input_size, input_size)] * len(samples),
        )

        cursor = 0
        for b_i, (sample, det) in enumerate(zip(samples, dets, strict=True)):
            num_imgs += 1
            ds_name = str(sample.source_dataset)
            n_p = int(det.scores.numel())
            z_b = z_b_all[cursor : cursor + n_p]
            cursor += n_p

            gt_q = sample.quads.to(device)
            ign_q = sample.ignore_quads.to(device)
            n_gt = int(gt_q.shape[0])
            total_gt += n_gt
            ds_total_gt[ds_name] += n_gt
            if n_p > 0:
                boxes_030 += int((det.scores >= 0.30).sum().item())

            with torch.amp.autocast(device.type, enabled=False):
                iou_mat = (
                    pairwise_quad_iou(det.quads, gt_q)
                    if (n_p > 0 and n_gt > 0)
                    else torch.zeros((n_p, n_gt), device=device)
                )
                ign_mat = (
                    pairwise_quad_iou(det.quads, ign_q)
                    if (n_p > 0 and ign_q.shape[0] > 0)
                    else torch.zeros((n_p, 0), device=device)
                )

            if n_gt > 0 and n_p > 0:
                rec50 += int((iou_mat.max(dim=0).values >= 0.50).sum().item())
                rec75 += int((iou_mat.max(dim=0).values >= 0.75).sum().item())
                keep_030 = det.scores >= 0.30
                if bool(keep_030.any().item()):
                    hits = (iou_mat[keep_030] >= 0.50).sum(dim=0)
                    matched_030 += int((hits >= 1).sum().item())
                    dup_030 += int((hits >= 2).sum().item())

            gt_used = torch.zeros(n_gt, dtype=torch.bool, device=device)
            for p_i in range(n_p):
                sc = float(det.scores[p_i].item())
                if n_gt > 0:
                    row = iou_mat[p_i].clone()
                    row[gt_used] = -1.0
                    b_iou, b_idx = row.max(dim=0)
                    if float(b_iou.item()) >= 0.50:
                        gt_used[b_idx] = True
                        agn_records.append((sc, 1))
                        ds_agn_records[ds_name].append((sc, 1))
                        continue
                if ign_mat.shape[1] > 0 and float(ign_mat[p_i].max().item()) >= 0.50:
                    continue
                agn_records.append((sc, 0))
                ds_agn_records[ds_name].append((sc, 0))

            if n_p == 0 or det.flat_indices is None:
                continue

            b_idx_t = torch.full((n_p,), b_i, dtype=torch.long, device=device)
            z_a = system.head_a.forward_indexed(
                features, b_idx_t, det.flat_indices, tower_features=tower_a
            )
            _, pb_f37 = system.head_b.classify_embeddings(z_b, t37_frozen)
            _, pa_a37 = system.head_a.classify_embeddings(z_a, t37_adapted)
            _, pb_a37 = system.head_b.classify_embeddings(z_b, t37_adapted)
            pab_a37 = 0.30 * pa_a37 + 0.70 * pb_a37
            _, pb_41 = system.head_b.classify_embeddings(z_b, t41_frozen)

            if n_gt > 0:
                best_ious, best_p = iou_mat.max(dim=0)
                for g_i, raw_c in enumerate(sample.category_names):
                    if float(best_ious[g_i].item()) < 0.50:
                        continue
                    p_idx = int(best_p[g_i].item())
                    canon = map_to_canonical(raw_c)
                    if canon is not None and canon in cat37_to_idx:
                        c_idx = cat37_to_idx[canon]
                        prop_total += 1
                        if int(pb_f37[p_idx].argmax().item()) == c_idx:
                            opt_b_frozen_top1 += 1
                        if int(pb_a37[p_idx].argmax().item()) == c_idx:
                            opt_b_adapted_top1 += 1
                        if int(pab_a37[p_idx].argmax().item()) == c_idx:
                            opt_ab_adapted_top1 += 1
                    else:
                        zs_idx = zs_to_41.get(normalize_category_name(raw_c), -1)
                        if zs_idx >= 0:
                            zs_total += 1
                            sub = int(pb_41[p_idx, zs_start:].argmax().item())
                            if zs_start + sub == zs_idx:
                                zs_b_4way_top1 += 1

            keep_ov = filter_semantic_detections(
                det.boxes, det.scores, pab_a37, OPEN_VOCAB_CATEGORIES, quads=det.quads
            )
            if keep_ov.numel() > 1:
                top_p = pab_a37[keep_ov].max(dim=-1).values
                vis_m = (det.scores[keep_ov] >= 0.25) & (top_p >= 0.12)
                if int(vis_m.sum().item()) > 1:
                    kq = det.quads[keep_ov][vis_m]
                    with torch.amp.autocast(device.type, enabled=False):
                        pw = pairwise_quad_iou(kq, kq)
                    tri = torch.triu(torch.ones_like(pw, dtype=torch.bool), diagonal=1)
                    post_sem_ovlp_025 += int((pw[tri] > 0.25).sum().item())

    def _calc_ap50(records: list[tuple[float, int]], gt_cnt: int) -> float:
        if gt_cnt <= 0 or not records:
            return 0.0
        sorted_recs = sorted(records, key=lambda x: -x[0])
        tp, fp = 0, 0
        precs, recs = [], []
        for _, is_tp in sorted_recs:
            tp += is_tp
            fp += 1 - is_tp
            precs.append(tp / max(tp + fp, 1))
            recs.append(tp / gt_cnt)
        total_ap = 0.0
        for s_i in range(101):
            r_t = s_i / 100.0
            total_ap += max((p for p, r in zip(precs, recs, strict=True) if r >= r_t), default=0.0)
        return total_ap / 101.0

    ap50 = _calc_ap50(agn_records, total_gt)
    ds_aps = [
        _calc_ap50(ds_agn_records[ds], ds_gt)
        for ds, ds_gt in ds_total_gt.items()
        if ds_gt > 0
    ]
    mean_ds_ap50 = sum(ds_aps) / max(len(ds_aps), 1)

    d_gt = max(total_gt, 1)
    d_prop = max(prop_total, 1)
    return {
        "recall_100_50": rec50 / d_gt,
        "recall_100_75": rec75 / d_gt,
        "agn_ap50": ap50,
        "mean_ds_ap50": mean_ds_ap50,
        "boxes_030": boxes_030 / max(num_imgs, 1),
        "dup_gt_rate": dup_030 / max(matched_030, 1),
        "opt_b_frozen_top1": opt_b_frozen_top1 / d_prop,
        "opt_b_adapted_top1": opt_b_adapted_top1 / d_prop,
        "opt_ab_adapted_top1": opt_ab_adapted_top1 / d_prop,
        "zeroshot4_4way_top1": zs_b_4way_top1 / max(zs_total, 1),
        "post_sem_ovlp_025": post_sem_ovlp_025 / max(num_imgs, 1),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cache-db",
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
    parser.add_argument(
        "--warmstart-ckpt",
        type=Path,
        default=Path("artifacts/sota_8k_quad/checkpoints/sota_joint_best.pt"),
    )
    parser.add_argument(
        "--resume-owlv2-quad-ckpt",
        type=Path,
        default=None,
        help="Optional OWLv2 Quad checkpoint to resume detector + Option A/B + proto_delta.",
    )
    parser.add_argument(
        "--lr-scale",
        type=float,
        default=1.0,
        help="Multiplier applied to all tiered learning rates.",
    )
    parser.add_argument(
        "--vocab-37",
        type=Path,
        default=Path("artifacts/sota_open_vocab/owlv2_vocabulary_bank_37classes.pt"),
    )
    parser.add_argument(
        "--vocab-41",
        type=Path,
        default=Path("artifacts/sota_open_vocab/owlv2_vocabulary_bank_41classes_zeroshot.pt"),
    )
    parser.add_argument(
        "--vocab-distractors",
        type=Path,
        default=Path("artifacts/sota_open_vocab/owlv2_vocabulary_bank_distractors.pt"),
    )
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/owlv2_30k_quad"))
    parser.add_argument("--epochs", type=int, default=6)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--max-steps", type=int, default=None, help="Optional cap for fast smoke runs.")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--device", type=str, default="cuda" if torch.cuda.is_available() else "cpu")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    torch.manual_seed(args.seed)
    device = torch.device(args.device)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    ckpt_dir = args.output_dir / "checkpoints"
    ckpt_dir.mkdir(parents=True, exist_ok=True)

    vocab_frozen_37 = VocabularyBank.load(args.vocab_37, device="cpu")
    vocab_frozen_41 = VocabularyBank.load(args.vocab_41, device="cpu")
    vocab_dist = VocabularyBank.load(args.vocab_distractors, device="cpu")
    cat37_idx = {c: i for i, c in enumerate(vocab_frozen_37.categories)}
    frozen_core_27 = torch.stack(
        [vocab_frozen_37.embeddings[cat37_idx[c]] for c in CORE_27_CATEGORIES], dim=0
    ).to(device)
    text_bank_37_gpu = vocab_frozen_37.embeddings.to(device)
    dist_emb_gpu = vocab_dist.embeddings.to(device)

    print("[Phase 2 Quad] Initializing OwlTeacher30kQuadDataset & sanitized cohort_val_800...", flush=True)
    train_ds = OwlTeacher30kQuadDataset(
        cache_db_path=args.cache_db,
        gt_index_path=args.gt_index,
        image_root=args.image_root,
        holdout_manifest=args.holdout_manifest,
        input_size=384,
        seed=args.seed,
    )
    val_ds = HumanGTQuadDataset(
        cohort_db_path=args.gt_index,
        gt_index_path=args.gt_index,
        image_root=args.image_root,
        split="val",
        input_size=384,
        cohort_size=8000,
        sanitize_annotations=True,
    )
    print(
        f"  Train images: {len(train_ds)} | Val images: {len(val_ds)} | Core classes: {len(CORE_27_CATEGORIES)}",
        flush=True,
    )

    sampler = WeightedRandomSampler(
        weights=torch.DoubleTensor(train_ds.sample_weights),
        num_samples=len(train_ds),
        replacement=True,
    )
    train_loader = DataLoader(
        train_ds,
        batch_size=args.batch_size,
        sampler=sampler,
        num_workers=args.workers,
        prefetch_factor=2 if args.workers > 0 else None,
        pin_memory=(device.type == "cuda"),
        collate_fn=collate_owl30k_quad_batch,
    )
    val_loader = DataLoader(
        val_ds,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        collate_fn=collate_quad_proposal_samples,
    )

    strides = (4, 8, 16, 32)
    detector = QuadProposalDetector(
        pretrained_backbone=False,
        fpn_channels=128,
        neck_type="pan",
        strides=strides,
        dual_head=True,
        use_3dmf=True,
    ).to(device)
    head_a = DenseAnchorSemanticHead(
        in_channels=128,
        hidden_channels=128,
        embedding_dim=768,
        teacher_space="owlv2",
    ).to(device)
    head_b = RoIPooledSemanticHead(
        in_channels=128,
        roi_channels=96,
        embedding_dim=768,
        output_size=5,
        strides=strides,
        teacher_space="owlv2",
    ).to(device)

    resumed_quad, init_delta = apply_warmstart_or_resume(
        detector=detector,
        head_a=head_a,
        head_b=head_b,
        warmstart_ckpt=args.warmstart_ckpt,
        resume_owlv2_quad_ckpt=args.resume_owlv2_quad_ckpt,
        device=device,
    )
    head_a.set_distractor_embeddings(dist_emb_gpu)
    head_b.set_distractor_embeddings(dist_emb_gpu)

    system = JointOpenVocabQuadSystem(detector, head_a, head_b, frozen_core_27).to(device)
    if init_delta is not None:
        with torch.no_grad():
            system.core_proto_delta.copy_(init_delta)
    ema = ExponentialMovingAverage(
        system, decay=0.998, ramp_steps=50 if resumed_quad else 150
    )

    ov_criterion = OpenVocabularySigmoidLoss(
        class_counts=train_ds.core_class_counts.to(device),
        teacher_space="owlv2",
    ).to(device)

    primary_builder = QuadTargetBuilder(
        QuadTaskAlignedAssigner(
            strides=strides,
            alpha=1.0,
            beta=4.0,
            assignment_mode="one_to_one",
        ),
        weak_negative_weight=0.18,
        max_one_to_one_interior_area_frac=0.20,
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

    lr_scale = float(args.lr_scale)
    pg: dict[tuple[float, float], list[nn.Parameter]] = defaultdict(list)
    for name, param in system.named_parameters():
        if not param.requires_grad:
            continue
        wd = (
            0.0
            if (param.ndim <= 1 or name.endswith(".bias") or "log_scale" in name or "core_proto_delta" in name)
            else 1e-4
        )
        if name.startswith("detector.backbone."):
            lr = 1.0e-4 * lr_scale
        elif name.startswith(("detector.fpn.", "detector.aux_head.")):
            lr = 2.5e-4 * lr_scale
        elif name.startswith("detector."):
            lr = 4.0e-4 * lr_scale
        else:
            lr = 8.0e-4 * lr_scale
        pg[(lr, wd)].append(param)

    optimizer = torch.optim.AdamW(
        [{"params": ps, "lr": lr, "weight_decay": wd} for (lr, wd), ps in pg.items()]
    )
    steps_per_ep = max(len(train_loader), 1)
    total_steps = args.epochs * steps_per_ep
    warmup_steps = min(steps_per_ep, 300) if resumed_quad else min(steps_per_ep, 500)
    warmup_sched = torch.optim.lr_scheduler.LinearLR(
        optimizer, start_factor=0.1, end_factor=1.0, total_iters=warmup_steps
    )
    cosine_sched = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer,
        T_max=max(total_steps - warmup_steps, 1),
        eta_min=1e-5 * min(lr_scale, 1.0),
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
        steady_t0: float | None = None
        system.train()
        freeze_backbone_batch_norm(system.detector)
        train_ds.set_epoch(epoch)
        sum_det, sum_ova, sum_ovb, steps = 0.0, 0.0, 0.0, 0

        for images, samples, metas in train_loader:
            if args.max_steps is not None and steps >= args.max_steps:
                break
            images = images.to(device, non_blocking=True)
            optimizer.zero_grad(set_to_none=True)

            with torch.amp.autocast(device.type, dtype=torch.float16, enabled=(device.type == "cuda")):
                features = system.detector.forward_features(images)
                detached_features = tuple(f.detach() for f in features)
                primary_out = system.detector.head(detached_features)
                assert system.detector.aux_head is not None
                aux_out = system.detector.aux_head(features)
                f_shapes = tuple((t.shape[-2], t.shape[-1]) for t in primary_out.quality)
                flat_qual_scores, _ = flatten_quad_output(primary_out)

                with torch.no_grad(), torch.amp.autocast(device.type, enabled=False):
                    batch_closed_mask = build_closed_27_annotated_mask(
                        [s.source_dataset for s in samples], device
                    )
                    t_primary = primary_builder(samples, f_shapes, device=device, pred_output=primary_out)
                    t_aux = aux_builder(samples, f_shapes, device=device, pred_output=aux_out)
                    live_dets = train_proposal_decoder(
                        primary_out, (384, 384), t_primary.valid_point_masks
                    )

                with torch.amp.autocast(device.type, enabled=False):
                    loss_p = det_criterion(primary_out, t_primary).total
                    loss_aux = det_criterion(aux_out, t_aux).total
                    loss_det = loss_p + loss_aux

                adapted_core = system.adapted_core_prototypes()
                assert t_primary.matched_gt_indices is not None
                assert t_aux.matched_gt_indices is not None

                a_bidx, a_aidx, a_l37, a_l27, a_temb, a_ht, a_cmask = [], [], [], [], [], [], []
                b_boxes_list, b_l37, b_l27, b_temb, b_ht, b_cmask = [], [], [], [], [], []

                with torch.amp.autocast(device.type, enabled=False):
                    for b_i, (sample, meta) in enumerate(zip(samples, metas, strict=True)):
                        l37_gpu = meta["labels_37"].to(device)
                        l27_gpu = meta["closed_labels"].to(device)
                        temb_gpu = meta["teacher_embs"].to(device)
                        ht_gpu = meta["has_teacher"].to(device)
                        n_gt_b = int(l37_gpu.numel())
                        c_mask_row = batch_closed_mask[b_i : b_i + 1]

                        # Option A anchor positives + hard/random negatives
                        pos_union = t_primary.positive_mask[b_i] | t_aux.positive_mask[b_i]
                        matched_u = torch.where(
                            t_primary.positive_mask[b_i],
                            t_primary.matched_gt_indices[b_i],
                            t_aux.matched_gt_indices[b_i],
                        )
                        pos_anc = torch.where(pos_union & (matched_u >= 0))[0]
                        if pos_anc.numel() > 0 and n_gt_b > 0:
                            mg = matched_u[pos_anc].clamp(0, n_gt_b - 1)
                            n_pa = int(pos_anc.numel())
                            a_bidx.append(torch.full_like(pos_anc, b_i))
                            a_aidx.append(pos_anc)
                            a_l37.append(l37_gpu[mg])
                            a_l27.append(l27_gpu[mg])
                            a_temb.append(temb_gpu[mg])
                            a_ht.append(ht_gpu[mg])
                            a_cmask.append(c_mask_row.expand(n_pa, -1))

                        bg_cand_m = t_aux.trusted_background_mask[b_i] | t_aux.weak_background_mask[b_i]
                        bg_cands = torch.where(bg_cand_m & ~pos_union)[0]
                        if bg_cands.numel() > 0:
                            bg_scores = flat_qual_scores[b_i, bg_cands].detach()
                            k_h = min(48, int(bg_cands.numel()))
                            k_r = min(96, int(bg_cands.numel()))
                            hard_anc = bg_cands[torch.topk(bg_scores, k=k_h).indices]
                            rand_anc = bg_cands[torch.randperm(bg_cands.numel(), device=device)[:k_r]]
                            bg_sel = torch.unique(torch.cat([hard_anc, rand_anc]))
                            n_bg = int(bg_sel.numel())
                            a_bidx.append(torch.full((n_bg,), b_i, dtype=torch.long, device=device))
                            a_aidx.append(bg_sel)
                            a_l37.append(torch.full((n_bg,), -1, dtype=torch.long, device=device))
                            a_l27.append(torch.full((n_bg,), -1, dtype=torch.long, device=device))
                            a_temb.append(torch.zeros((n_bg, 768), device=device))
                            a_ht.append(torch.zeros((n_bg,), dtype=torch.bool, device=device))
                            a_cmask.append(c_mask_row.expand(n_bg, -1))

                        # Option B RoIs (GT quad envelopes + live quad IoU>=0.50 props + hard clutter <0.20)
                        gt_q = sample.quads.to(device)
                        ign_q = sample.ignore_quads.to(device)
                        r_boxes, r_l37, r_l27, r_temb, r_ht = [], [], [], [], []
                        if n_gt_b > 0:
                            r_boxes.append(RegionEmbeddingHead.boxes_from_quads(gt_q))
                            r_l37.append(l37_gpu)
                            r_l27.append(l27_gpu)
                            r_temb.append(temb_gpu)
                            r_ht.append(ht_gpu)

                        prop_q = live_dets[b_i].quads.detach()
                        prop_b = live_dets[b_i].boxes.detach()
                        if prop_q.numel() > 0:
                            max_iou, arg_gt = (
                                pairwise_quad_iou(prop_q, gt_q).max(dim=1)
                                if n_gt_b > 0
                                else (
                                    torch.zeros((prop_q.shape[0],), device=device),
                                    torch.zeros((prop_q.shape[0],), dtype=torch.long, device=device),
                                )
                            )
                            pos_p = max_iou >= 0.50
                            if pos_p.any() and n_gt_b > 0:
                                mg_p = arg_gt[pos_p]
                                r_boxes.append(prop_b[pos_p])
                                r_l37.append(l37_gpu[mg_p])
                                r_l27.append(l27_gpu[mg_p])
                                r_temb.append(temb_gpu[mg_p])
                                r_ht.append(ht_gpu[mg_p])

                            neg_p = max_iou < 0.20
                            if neg_p.any() and ign_q.numel() > 0:
                                ign_ov = pairwise_quad_iou(prop_q, ign_q).max(dim=1).values
                                neg_p = neg_p & (ign_ov < 0.25)
                            if neg_p.any():
                                nb = prop_b[neg_p][:12]
                                n_nb = int(nb.shape[0])
                                r_boxes.append(nb)
                                r_l37.append(torch.full((n_nb,), -1, dtype=torch.long, device=device))
                                r_l27.append(torch.full((n_nb,), -1, dtype=torch.long, device=device))
                                r_temb.append(torch.zeros((n_nb, 768), device=device))
                                r_ht.append(torch.zeros((n_nb,), dtype=torch.bool, device=device))

                        if r_boxes:
                            cat_rb = torch.cat(r_boxes, dim=0)
                            b_boxes_list.append(cat_rb)
                            b_l37.append(torch.cat(r_l37, dim=0))
                            b_l27.append(torch.cat(r_l27, dim=0))
                            b_temb.append(torch.cat(r_temb, dim=0))
                            b_ht.append(torch.cat(r_ht, dim=0))
                            b_cmask.append(c_mask_row.expand(int(cat_rb.shape[0]), -1))
                        else:
                            b_boxes_list.append(torch.empty((0, 4), device=device))

                with torch.amp.autocast(device.type, enabled=False):
                    detach_sem = (epoch == 1) and (not resumed_quad)
                    sem_feats = (
                        [f.detach().float() for f in features]
                        if detach_sem
                        else [f.float() for f in features]
                    )
                    if a_bidx:
                        z_a_all = system.head_a.forward_indexed(
                            sem_feats,
                            torch.cat(a_bidx),
                            torch.cat(a_aidx),
                        )
                        loss_ov_a = ov_criterion(
                            system.head_a,
                            z_a_all,
                            text_bank_37_gpu,
                            torch.cat(a_l37),
                            teacher_embeddings=torch.cat(a_temb),
                            has_teacher=torch.cat(a_ht),
                            closed_text_embeddings=adapted_core,
                            closed_labels=torch.cat(a_l27),
                            closed_annotated_mask=torch.cat(a_cmask, dim=0),
                        ).total
                    else:
                        loss_ov_a = loss_det * 0.0

                    if b_l37:
                        z_b_all = system.head_b(
                            sem_feats,
                            b_boxes_list,
                            image_shapes=[(384, 384)] * len(samples),
                        )
                        loss_ov_b = ov_criterion(
                            system.head_b,
                            z_b_all,
                            text_bank_37_gpu,
                            torch.cat(b_l37),
                            teacher_embeddings=torch.cat(b_temb),
                            has_teacher=torch.cat(b_ht),
                            closed_text_embeddings=adapted_core,
                            closed_labels=torch.cat(b_l27),
                            closed_annotated_mask=torch.cat(b_cmask, dim=0),
                        ).total
                    else:
                        loss_ov_b = loss_det * 0.0

                    proto_reg = 0.05 * system.core_proto_delta.pow(2).sum()
                    sem_w = 0.50 if detach_sem else 0.35
                    total_loss = loss_det + sem_w * loss_ov_a + sem_w * loss_ov_b + proto_reg

            if not torch.isfinite(total_loss):
                raise RuntimeError(
                    f"Non-finite loss at epoch {epoch} step {steps}: "
                    f"det={loss_det.item()} a={loss_ov_a.item()} b={loss_ov_b.item()}"
                )

            scaler.scale(total_loss).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(system.parameters(), max_norm=10.0)
            scaler.step(optimizer)
            scaler.update()
            with torch.no_grad():
                norms = system.core_proto_delta.norm(dim=-1, keepdim=True)
                scale_clamp = torch.where(
                    norms > 0.12, 0.12 / norms.clamp(min=1e-6), torch.ones_like(norms)
                )
                system.core_proto_delta.mul_(scale_clamp)
            scheduler.step()
            ema.update(system)

            sum_det += float(loss_det.detach().item())
            sum_ova += float(loss_ov_a.detach().item())
            sum_ovb += float(loss_ov_b.detach().item())
            steps += 1
            if steps == 2:
                steady_t0 = time.time()

        ep_sec = time.time() - ep_t0
        steady_s_per_step = (
            (time.time() - steady_t0) / max(steps - 2, 1)
            if steady_t0 is not None and steps > 2
            else ep_sec / max(steps, 1)
        )
        ema_sys: JointOpenVocabQuadSystem = ema.module
        vocab_adapted_37 = build_adapted_bank(
            vocab_frozen_37, ema_sys.adapted_core_prototypes()
        )
        vocab_adapted_41 = build_adapted_bank(
            vocab_frozen_41, ema_sys.adapted_core_prototypes()
        )
        val_m = evaluate_owlv2_quad_val(
            ema_sys,
            val_loader,
            eval_decoder,
            vocab_frozen_37,
            vocab_frozen_41,
            vocab_adapted_37,
            384,
            device,
        )
        comp = (
            0.35 * val_m["recall_100_50"]
            + 0.25 * val_m["opt_b_frozen_top1"]
            + 0.15 * val_m["opt_b_adapted_top1"]
            + 0.25 * val_m["mean_ds_ap50"]
        )
        is_best = comp > best_score
        if is_best:
            best_score = comp

        rec_ep = {
            "epoch": epoch,
            "epoch_sec": round(ep_sec, 1),
            "steady_s_per_step": round(steady_s_per_step, 3),
            "steps": steps,
            "loss_det": round(sum_det / max(steps, 1), 4),
            "loss_ov_a": round(sum_ova / max(steps, 1), 4),
            "loss_ov_b": round(sum_ovb / max(steps, 1), 4),
            "composite_score": round(comp, 4),
            **{k: round(v, 4) for k, v in val_m.items()},
        }
        history.append(rec_ep)
        print(
            f"Epoch {epoch:02d}/{args.epochs} ({ep_sec:.1f}s, {steady_s_per_step:.2f}s/step) | "
            f"L[det={rec_ep['loss_det']:.3f} A={rec_ep['loss_ov_a']:.3f} B={rec_ep['loss_ov_b']:.3f}] | "
            f"R@100={val_m['recall_100_50']*100:.1f}% R@75={val_m['recall_100_75']*100:.1f}% "
            f"dsAP={val_m['mean_ds_ap50']*100:.1f}% AP50={val_m['agn_ap50']*100:.1f}% "
            f"B03={val_m['boxes_030']:.1f} Ovlp={val_m['post_sem_ovlp_025']:.2f} | "
            f"Top1[B_frz={val_m['opt_b_frozen_top1']*100:.1f}% B_adp={val_m['opt_b_adapted_top1']*100:.1f}% AB_adp={val_m['opt_ab_adapted_top1']*100:.1f}%] "
            f"ZS4={val_m['zeroshot4_4way_top1']*100:.1f}%"
            + (" *BEST*" if is_best else ""),
            flush=True,
        )

        payload = {
            "schema_version": "owlv2_hybrid_v1",
            "epoch": epoch,
            "geometry": "quad",
            "detector_config": {
                "fpn_channels": 128,
                "neck_type": "pan",
                "strides": strides,
                "dual_head": True,
                "use_3dmf": True,
            },
            "model_state_dict": copy.deepcopy(ema_sys.detector.state_dict()),
            "head_a_config": ema_sys.head_a.get_config(),
            "head_a_state_dict": copy.deepcopy(ema_sys.head_a.state_dict()),
            "head_b_config": ema_sys.head_b.get_config(),
            "head_b_state_dict": copy.deepcopy(ema_sys.head_b.state_dict()),
            "core_proto_delta": ema_sys.core_proto_delta.detach().cpu(),
            "vocab_categories": list(OPEN_VOCAB_CATEGORIES),
            "core_27_categories": list(CORE_27_CATEGORIES),
            "metrics": val_m,
        }
        torch.save(payload, ckpt_dir / "owlv2_quad_last.pt")
        if is_best:
            torch.save(payload, ckpt_dir / "owlv2_quad_best.pt")
            vocab_adapted_37.save(args.output_dir / "owlv2_adapted_vocab_37classes.pt")
            vocab_adapted_41.save(args.output_dir / "owlv2_adapted_vocab_41classes_zeroshot.pt")
            vocab_adapted_37.subset(CORE_27_CATEGORIES).save(
                args.output_dir / "owlv2_adapted_vocab_core27.pt"
            )
            vocab_adapted_41.subset(
                CORE_27_CATEGORIES + ZERO_SHOT_EVAL_CATEGORIES
            ).save(args.output_dir / "owlv2_adapted_vocab_31classes_core27_zeroshot.pt")

    (args.output_dir / "training_history.json").write_text(
        json.dumps(
            {
                "epochs": args.epochs,
                "total_minutes": round((time.time() - train_start) / 60.0, 2),
                "best_composite_score": round(best_score, 4),
                "history": history,
            },
            indent=2,
        )
    )
    print(f"[{datetime.now().strftime('%H:%M:%S')}] Phase 2 Quad training done -> {ckpt_dir / 'owlv2_quad_best.pt'}", flush=True)


if __name__ == "__main__":
    main()
