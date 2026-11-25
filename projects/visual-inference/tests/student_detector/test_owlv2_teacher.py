"""Unit tests for OWLv2 teacher crowd-splitting, missing-class merge, and cache I/O."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import torch
import torch.nn.functional as F

from student_detector.data import ProposalSample
from student_detector.ontology import (
    CORE_27_CATEGORIES,
    OPEN_VOCAB_CATEGORIES,
    PRUNED_SCARCE_CATEGORIES,
)
from student_detector.open_vocab_heads import (
    BACKDROP_NO_SUPPRESS_CONTAINERS,
    DenseAnchorSemanticHead,
    OpenVocabularySigmoidLoss,
    RoIPooledSemanticHead,
    filter_semantic_detections,
)
from student_detector.owlv2_teacher import (
    MergedImageTargets,
    OwlDetection,
    OwlTeacherCacheReader,
    OwlTeacherCacheWriter,
    merge_clean_gt_and_owlv2,
    merge_patch_owl_detections,
)
from student_detector.targets import TargetBuilder
from student_detector.task_aligned_assigner import TaskAlignedAssigner
from student_detector.vocabulary import VocabularyBank


def test_crowd_pedestrian_split_replaces_wide_group_gt() -> None:
    gt_boxes = [
        [100.0, 100.0, 300.0, 220.0],
        [400.0, 120.0, 520.0, 220.0],
    ]
    gt_labels = ["pedestrian", "car"]
    owl_dets = [
        OwlDetection((105.0, 105.0, 165.0, 215.0), "pedestrian", 0.62, 10),
        OwlDetection((170.0, 108.0, 225.0, 218.0), "pedestrian", 0.58, 11),
        OwlDetection((232.0, 102.0, 295.0, 216.0), "pedestrian", 0.51, 12),
    ]
    merged = merge_clean_gt_and_owlv2(
        gt_boxes,
        gt_labels,
        owl_dets,
        source_dataset="nuimages",
        img_w=800,
        img_h=600,
    )
    assert merged.superseded_gt_indices == (0,)
    assert len(merged.added_owl_detections) == 3
    assert merged.crowd_split_owl_mask == (True, True, True)
    assert [d.patch_index for d in merged.added_owl_detections] == [10, 11, 12]
    assert merged.stats["crowd_gt_split"] == 1
    assert merged.stats["crowd_owl_peds_added"] == 3


def test_single_pedestrian_not_split_vertically_into_torso_and_legs() -> None:
    gt_boxes = [[100.0, 100.0, 160.0, 260.0]]
    gt_labels = ["pedestrian"]
    owl_dets = [
        OwlDetection((102.0, 102.0, 158.0, 170.0), "pedestrian", 0.65, 20),
        OwlDetection((104.0, 175.0, 156.0, 258.0), "pedestrian", 0.54, 21),
    ]
    merged = merge_clean_gt_and_owlv2(
        gt_boxes,
        gt_labels,
        owl_dets,
        source_dataset="nuimages",
        img_w=800,
        img_h=600,
    )
    assert merged.superseded_gt_indices == ()
    assert merged.added_owl_detections == ()
    assert merged.crowd_split_owl_mask == ()
    assert merged.stats["crowd_gt_split"] == 0


def test_missing_class_filters_stuff_hood_annotated_and_gt_overlap() -> None:
    gt_boxes = [
        [200.0, 200.0, 300.0, 300.0],
        [450.0, 200.0, 520.0, 320.0],
    ]
    gt_labels = ["car", "pedestrian"]
    owl_dets = [
        # Annotated class on woodscape -> skipped unless crowd split
        OwlDetection((50.0, 200.0, 130.0, 260.0), "car", 0.80, 1),
        # Giant stuff (> 50% area of 1000x800) -> filtered
        OwlDetection((0.0, 0.0, 900.0, 600.0), "building", 0.45, 2),
        # WoodScape bottom-band hood false positive (>= 12% area, cy >= 58% h) -> filtered
        OwlDetection((150.0, 480.0, 850.0, 790.0), "construction_barrier", 0.40, 3),
        # Overlaps existing GT car heavily -> filtered
        OwlDetection((205.0, 205.0, 295.0, 295.0), "trash_bin", 0.70, 4),
        # Valid missing class (stroller) -> kept!
        OwlDetection((600.0, 120.0, 700.0, 240.0), "stroller", 0.68, 5),
        # Valid capped stuff (building = 20% area) -> kept!
        OwlDetection((10.0, 10.0, 410.0, 410.0), "building", 0.42, 6),
    ]
    merged = merge_clean_gt_and_owlv2(
        gt_boxes,
        gt_labels,
        owl_dets,
        source_dataset="woodscape_rgb_fisheye",
        img_w=1000,
        img_h=800,
    )
    assert merged.superseded_gt_indices == ()
    assert [d.label for d in merged.added_owl_detections] == ["stroller", "building"]
    assert merged.crowd_split_owl_mask == (False, False)
    assert merged.stats["filtered_stuff_area"] == 1
    assert merged.stats["filtered_woodscape_hood"] == 1
    assert merged.stats["filtered_gt_overlap"] == 1
    assert merged.stats["missing_class_added"] == 2


def test_owl_teacher_cache_roundtrip_and_resume(tmp_path: Path) -> None:
    db_path = tmp_path / "owl_cache.sqlite"
    writer = OwlTeacherCacheWriter(db_path)
    writer.set_metadata("version", "1")

    emb_gt_1 = np.ones((2, 768), dtype=np.float16) * 0.25
    iou_gt_1 = np.asarray([0.81, 0.62], dtype=np.float16)
    dets_1 = [
        OwlDetection((10.0, 20.0, 50.0, 80.0), "pedestrian", 0.64, 5),
        OwlDetection((300.0, 200.0, 350.0, 290.0), "trash_bin", 0.71, 99),
    ]
    emb_owl_1 = np.stack(
        [np.full((768,), 0.5, dtype=np.float16), np.full((768,), -0.5, dtype=np.float16)],
        axis=0,
    )
    merged_1 = merge_clean_gt_and_owlv2(
        [[5.0, 15.0, 120.0, 85.0], [400.0, 400.0, 500.0, 500.0]],
        ["car", "pedestrian"],
        dets_1,
        source_dataset="nuimages",
        img_w=800,
        img_h=600,
    )
    # Manually construct a clean merged target that keeps both detections with crowd_mask=[True, False]
    from student_detector.owlv2_teacher import MergedImageTargets

    custom_merged = MergedImageTargets(
        superseded_gt_indices=(1,),
        added_owl_detections=tuple(dets_1),
        crowd_split_owl_mask=(True, False),
        stats=merged_1.stats,
    )
    writer.write_image(
        image_id=101,
        source_dataset="nuimages",
        width=800,
        height=600,
        merged=custom_merged,
        gt_match_ious=iou_gt_1,
        gt_embeddings=emb_gt_1,
        all_owl_detections=dets_1,
        all_owl_embeddings=emb_owl_1,
    )
    writer.write_image(
        image_id=202,
        source_dataset="bdd100k_images_100k",
        width=1280,
        height=720,
        merged=MergedImageTargets((), (), (), {"missing_class_added": 0}),
        gt_match_ious=np.zeros((0,), dtype=np.float16),
        gt_embeddings=np.zeros((0, 768), dtype=np.float16),
        all_owl_detections=[],
        all_owl_embeddings=np.zeros((0, 768), dtype=np.float16),
    )
    assert writer.get_completed_ids() == {101, 202}
    writer.close()

    reader = OwlTeacherCacheReader(db_path)
    assert reader.count() == 2
    assert reader.get_image_ids() == [101, 202]

    rec = reader.get_image(101)
    assert rec is not None
    assert rec.superseded_gt_indices == (1,)
    assert np.allclose(rec.gt_match_ious, iou_gt_1)
    assert np.allclose(rec.gt_embeddings, emb_gt_1)
    assert rec.owl_labels == ("pedestrian", "trash_bin")
    assert rec.owl_crowd_mask.tolist() == [True, False]
    assert np.allclose(rec.owl_embeddings, emb_owl_1)
    assert np.allclose(rec.owl_boxes[1], np.asarray([300.0, 200.0, 350.0, 290.0], dtype=np.float32))

    rec2 = reader.get_image(202)
    assert rec2 is not None
    assert rec2.owl_boxes.shape == (0, 4)
    assert rec2.gt_embeddings.shape == (0, 768)
    reader.close()


def test_owlv2_head_calibration_serialization_and_loss() -> None:
    head_a = DenseAnchorSemanticHead(
        in_channels=32, hidden_channels=32, embedding_dim=16, teacher_space="owlv2"
    )
    head_b = RoIPooledSemanticHead(
        in_channels=32,
        roi_channels=16,
        embedding_dim=16,
        hidden_dim=32,
        teacher_space="owlv2",
    )
    reloaded_a = DenseAnchorSemanticHead.from_config(head_a.get_config())
    reloaded_b = RoIPooledSemanticHead.from_config(head_b.get_config())
    assert reloaded_a.teacher_space == "owlv2"
    assert reloaded_b.teacher_space == "owlv2"
    assert round(float(reloaded_b.effective_scale().item()), 2) == 20.0
    assert round(float(reloaded_b.logit_bias.item()), 2) == -6.50

    # Non-zero visual_centroid must NOT shift OWLv2 text prompts
    reloaded_b.set_visual_centroid(torch.ones(16))
    txt = F.normalize(torch.eye(4, 16), p=2.0, dim=-1)
    assert torch.equal(reloaded_b.compute_prompt_bias(txt), torch.zeros((1, 4)))

    # Construct a visual vector with cosine 0.45 to class 0 and 0.20 to class 1
    e0 = txt[0]
    e1 = txt[1]
    e_orth = txt[2]
    # vis = 0.45*e0 + 0.20*e1 + c*e2 -> unit norm
    rem = (1.0 - 0.45**2 - 0.20**2) ** 0.5
    vis = (0.45 * e0 + 0.20 * e1 + rem * e_orth).unsqueeze(0)
    _, probs_uncal = reloaded_b.classify_embeddings(vis, txt, apply_calibration=False)
    assert abs(float(probs_uncal[0, 0].item()) - 0.924) < 0.01
    assert abs(float(probs_uncal[0, 1].item()) - 0.076) < 0.01

    # Full loss + gradient flow across head & residual closed-set prototypes
    loss_fn = OpenVocabularySigmoidLoss(
        class_counts=torch.tensor([1000.0, 200.0, 500.0]),
        teacher_space="owlv2",
    )
    reloaded_b.set_distractor_embeddings(txt[3:4])
    vis_batch = F.normalize(torch.randn(5, 16), p=2.0, dim=-1).requires_grad_(True)
    frozen_37 = F.normalize(torch.randn(4, 16), p=2.0, dim=-1)
    frozen_core = frozen_37[:3]
    proto_delta = torch.zeros_like(frozen_core, requires_grad=True)
    adapted_core = F.normalize(frozen_core + proto_delta, p=2.0, dim=-1)

    teacher_embs = F.normalize(torch.randn(5, 16), p=2.0, dim=-1)
    labels_37 = torch.tensor([0, 1, -2, -1, 2])
    closed_labels = torch.tensor([0, 1, -2, -1, 2])
    has_teacher = torch.tensor([True, True, True, False, False])
    out = loss_fn(
        reloaded_b,
        vis_batch,
        frozen_37,
        labels_37,
        teacher_embeddings=teacher_embs,
        has_teacher=has_teacher,
        closed_text_embeddings=adapted_core,
        closed_labels=closed_labels,
    )
    assert torch.isfinite(out.total)
    out.total.backward()
    assert vis_batch.grad is not None and torch.isfinite(vis_batch.grad).all()
    assert proto_delta.grad is not None and float(proto_delta.grad.abs().sum().item()) > 0.0
    assert reloaded_b.log_scale.grad is not None and reloaded_b.logit_bias.grad is not None


def test_filter_semantic_detections_keeps_foreground_in_front_of_building_and_tree() -> None:
    assert {"building", "tree", "fence", "canopy"} <= BACKDROP_NO_SUPPRESS_CONTAINERS
    cats = ("building", "tree", "pedestrian", "car", "pole", "trash_bin", "license_plate")
    # 0: large building [0,0,400,300], 1: pedestrian inside building, 2: car inside building
    # 3: large tree [420,0,620,300], 4: pole inside tree
    # 5: trash_bin [650,100,750,220], 6: spurious tiny license_plate sticker inside trash_bin -> suppressed!
    boxes = torch.tensor(
        [
            [0.0, 0.0, 400.0, 300.0],
            [80.0, 120.0, 130.0, 260.0],
            [180.0, 160.0, 320.0, 270.0],
            [420.0, 0.0, 620.0, 300.0],
            [500.0, 60.0, 520.0, 280.0],
            [650.0, 100.0, 750.0, 220.0],
            [680.0, 130.0, 710.0, 150.0],
        ]
    )
    obj = torch.tensor([0.92, 0.78, 0.84, 0.88, 0.74, 0.90, 0.65])
    probs = torch.full((7, 7), 0.01)
    probs[0, 0] = 0.90  # building
    probs[1, 2] = 0.85  # pedestrian
    probs[2, 3] = 0.88  # car
    probs[3, 1] = 0.86  # tree
    probs[4, 4] = 0.80  # pole
    probs[5, 5] = 0.89  # trash_bin
    probs[6, 6] = 0.60  # license_plate inside trash_bin

    kept = filter_semantic_detections(boxes, obj, probs, cats)
    assert kept.tolist() == [0, 1, 2, 3, 4, 5]


def test_target_builder_caps_one_to_one_interior_suppression_for_backdrop_boxes() -> None:
    strides = (4, 8, 16, 32)
    shapes = ((32, 32), (16, 16), (8, 8), (4, 4))
    builder = TargetBuilder(
        TaskAlignedAssigner(strides=strides, assignment_mode="one_to_one"),
        default_background_weight=0.18,
        max_one_to_one_interior_area_frac=0.20,
    )
    large_backdrop = ProposalSample(
        image=torch.zeros(3, 128, 128),
        boxes=torch.tensor([[10.0, 10.0, 90.0, 92.0]]),  # ~40% of 128x128
        ignore_boxes=torch.empty((0, 4)),
        valid_mask=torch.ones(128, 128, dtype=torch.bool),
        image_id=1,
        source_dataset="nuimages",
        domain="automotive",
        camera_type="pinhole",
        background_supervision=False,
        original_size=(128, 128),
        transform=(1.0, 0.0, 0.0),
    )
    small_car = ProposalSample(
        image=torch.zeros(3, 128, 128),
        boxes=torch.tensor([[16.0, 16.0, 48.0, 48.0]]),  # 6.25% of 128x128
        ignore_boxes=torch.empty((0, 4)),
        valid_mask=torch.ones(128, 128, dtype=torch.bool),
        image_id=2,
        source_dataset="nuimages",
        domain="automotive",
        camera_type="pinhole",
        background_supervision=False,
        original_size=(128, 128),
        transform=(1.0, 0.0, 0.0),
    )
    t_backdrop = builder([large_backdrop], shapes, device=torch.device("cpu"))
    t_car = builder([small_car], shapes, device=torch.device("cpu"))
    assert int(t_backdrop.positive_mask.sum().item()) == 1
    assert int(t_backdrop.trusted_background_mask.sum().item()) == 0
    assert int(t_car.positive_mask.sum().item()) == 1
    assert int(t_car.trusted_background_mask.sum().item()) > 10


def test_core27_ontology_and_vocabulary_subset() -> None:
    assert len(CORE_27_CATEGORIES) == 27
    assert "street_light" not in CORE_27_CATEGORIES
    assert "pole" in CORE_27_CATEGORIES
    assert not (set(CORE_27_CATEGORIES) & PRUNED_SCARCE_CATEGORIES)

    emb = torch.arange(len(OPEN_VOCAB_CATEGORIES) * 8, dtype=torch.float32).reshape(
        len(OPEN_VOCAB_CATEGORIES), 8
    )
    emb = F.normalize(emb + 1.0, p=2.0, dim=-1)
    bank = VocabularyBank(
        categories=OPEN_VOCAB_CATEGORIES,
        embeddings=emb,
        logit_scale=20.0,
        logit_bias=-6.5,
    )
    sub = bank.subset(CORE_27_CATEGORIES)
    assert sub.categories == CORE_27_CATEGORIES
    assert sub.embeddings.shape == (27, 8)
    pole_idx_full = OPEN_VOCAB_CATEGORIES.index("pole")
    pole_idx_sub = sub.categories.index("pole")
    assert torch.allclose(sub.embeddings[pole_idx_sub], emb[pole_idx_full])


def test_merge_patch_owl_detections_filters_and_deduplicates() -> None:
    gt_boxes = [[100.0, 100.0, 180.0, 300.0]]
    gt_labels = ["pillar"]
    existing_owl_boxes = [[300.0, 150.0, 340.0, 320.0]]
    existing_owl_labels = ["pole"]

    patch_dets = [
        # Valid metal bollard -> pole (>= 0.28, <= 15% area)
        OwlDetection((400.0, 250.0, 430.0, 360.0), "pole", 0.34, 10),
        # Valid column -> pillar (>= 0.26, <= 25% area)
        OwlDetection((600.0, 80.0, 680.0, 380.0), "pillar", 0.27, 11),
        # Cross-class duplicate of the valid pole above -> filtered by owl overlap
        OwlDetection((401.0, 252.0, 429.0, 358.0), "pillar", 0.29, 12),
        # Low-score pole (0.27 < 0.28) -> filtered
        OwlDetection((500.0, 250.0, 525.0, 350.0), "pole", 0.27, 13),
        # Oversized pole (>15% area of 1000x800 = 120,000 px^2) -> filtered (500*300 = 150,000)
        OwlDetection((50.0, 400.0, 550.0, 700.0), "pole", 0.50, 14),
        # Overlaps GT pillar heavily -> filtered
        OwlDetection((102.0, 104.0, 178.0, 296.0), "pillar", 0.60, 15),
        # Overlaps existing OWL pole heavily -> filtered
        OwlDetection((302.0, 152.0, 338.0, 318.0), "pole", 0.55, 16),
    ]

    acc_nu, st_nu = merge_patch_owl_detections(
        gt_boxes,
        gt_labels,
        existing_owl_boxes,
        existing_owl_labels,
        patch_dets,
        source_dataset="nuimages",
        img_w=1000,
        img_h=800,
    )
    assert [(d.label, d.patch_index) for d in acc_nu] == [("pole", 10), ("pillar", 11)]
    assert st_nu["patch_added_pole"] == 1
    assert st_nu["patch_added_pillar"] == 1
    assert st_nu["patch_filtered_area"] == 1
    assert st_nu["patch_filtered_gt_overlap"] == 1
    assert st_nu["patch_filtered_owl_overlap"] == 2

    # WoodScape annotates pole -> skip pole, keep valid pillar, block ego-hood pillar
    ws_dets = [
        OwlDetection((400.0, 250.0, 430.0, 360.0), "pole", 0.65, 20),
        OwlDetection((600.0, 80.0, 680.0, 380.0), "pillar", 0.30, 21),
        # Bottom-band ego-hood candidate: cy=635/800=0.79 >= 0.58, area=600*250/(1000*800)=0.1875 >= 0.12
        OwlDetection((200.0, 510.0, 800.0, 760.0), "pillar", 0.45, 22),
    ]
    acc_ws, st_ws = merge_patch_owl_detections(
        [],
        [],
        [],
        [],
        ws_dets,
        source_dataset="woodscape_rgb_fisheye",
        img_w=1000,
        img_h=800,
    )
    assert [(d.label, d.patch_index) for d in acc_ws] == [("pillar", 21)]
    assert st_ws["patch_added_pole"] == 0
    assert st_ws["patch_added_pillar"] == 1
    assert st_ws["patch_filtered_woodscape_hood"] == 1

    # ADE20K annotates both pole & pillar -> skips both
    acc_ade, st_ade = merge_patch_owl_detections(
        [],
        [],
        [],
        [],
        patch_dets[:2],
        source_dataset="ade20k",
        img_w=1000,
        img_h=800,
    )
    assert acc_ade == ()
    assert st_ade["patch_added_pole"] == 0
    assert st_ade["patch_added_pillar"] == 0


def test_owl_teacher_cache_append_owl_detections(tmp_path: Path) -> None:
    db_path = tmp_path / "owl_patch_append.sqlite"
    writer = OwlTeacherCacheWriter(db_path)

    gt_emb = np.full((1, 768), 0.125, dtype=np.float16)
    gt_ious = np.asarray([0.85], dtype=np.float16)
    orig_det = OwlDetection((50.0, 60.0, 90.0, 140.0), "trash_bin", 0.72, 7)
    orig_emb = np.full((1, 768), 0.25, dtype=np.float16)

    merged = MergedImageTargets(
        superseded_gt_indices=(),
        added_owl_detections=(orig_det,),
        crowd_split_owl_mask=(False,),
        stats={"missing_class_added": 1, "crowd_gt_split": 0},
    )
    writer.write_image(
        image_id=555,
        source_dataset="nuimages",
        width=1600,
        height=900,
        merged=merged,
        gt_match_ious=gt_ious,
        gt_embeddings=gt_emb,
        all_owl_detections=(orig_det,),
        all_owl_embeddings=orig_emb,
    )

    patch_dets = (
        OwlDetection((400.0, 300.0, 435.0, 420.0), "pole", 0.44, 15),
        OwlDetection((610.0, 100.0, 690.0, 440.0), "pillar", 0.31, 22),
    )
    bad_embs = np.zeros((2, 768), dtype=np.float32)
    bad_embs[0, 3] = float("nan")
    with pytest.raises(ValueError, match="Non-finite"):
        writer.append_owl_detections(
            image_id=555,
            added_detections=patch_dets,
            added_embeddings=bad_embs,
        )

    patch_embs = np.stack(
        [np.full((768,), -0.25, dtype=np.float16), np.full((768,), 0.5, dtype=np.float16)],
        axis=0,
    )
    n_app = writer.append_owl_detections(
        image_id=555,
        added_detections=patch_dets,
        added_embeddings=patch_embs,
        extra_stats={"patch_added_pole": 1, "patch_added_pillar": 1},
    )
    assert n_app == 2
    writer.close(checkpoint_wal=True)

    assert not (tmp_path / "owl_patch_append.sqlite-wal").exists()
    reader = OwlTeacherCacheReader(db_path)
    rec = reader.get_image(555)
    assert rec is not None
    assert rec.superseded_gt_indices == ()
    assert np.allclose(rec.gt_match_ious, gt_ious)
    assert np.allclose(rec.gt_embeddings, gt_emb)
    assert rec.owl_labels == ("trash_bin", "pole", "pillar")
    assert rec.owl_crowd_mask.tolist() == [False, False, False]
    assert np.allclose(rec.owl_scores, np.asarray([0.72, 0.44, 0.31], dtype=np.float32))
    assert rec.owl_embeddings.shape == (3, 768)
    assert np.allclose(rec.owl_embeddings[0], orig_emb[0])
    assert np.allclose(rec.owl_embeddings[1:], patch_embs)
    assert rec.stats["missing_class_added"] == 3
    assert rec.stats["patch_added_pole"] == 1
    assert rec.stats["patch_added_pillar"] == 1
    reader.close()

