"""Unit tests for the SOTA open-vocabulary architecture (Option A & Option B) and root-cause fixes."""

from __future__ import annotations

import torch
import torch.nn.functional as F

from student_detector.augmentation import canvas_box_to_native
from student_detector.backbone import StubP2Backbone
from student_detector.config import AugmentationConfig
from student_detector.data import ProposalSample, ProposalTransform
from student_detector.decoder import InferenceDecoder, class_agnostic_nms
from student_detector.head import QuadDetectorOutput
from student_detector.losses import ProposalLoss
from student_detector.model import QuadProposalDetector, StudentDetector
from student_detector.ontology import (
    OPEN_VOCAB_CATEGORIES,
    SOURCE_ANNOTATED_CATEGORIES,
    ZERO_SHOT_EVAL_CATEGORIES,
    build_source_annotated_mask,
    map_to_canonical,
)
from student_detector.open_vocab_heads import (
    DenseAnchorSemanticHead,
    OpenVocabularySigmoidLoss,
    RoIPooledSemanticHead,
    filter_semantic_detections,
)
from student_detector.quad_data import QuadProposalTransform
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_geometry import polygon_nms, quad_from_bbox
from student_detector.quad_losses import QuadProposalLoss
from student_detector.quad_targets import QuadTargetBuilder
from student_detector.targets import TargetBuilder
from student_detector.task_aligned_assigner import (
    QuadTaskAlignedAssigner,
    TaskAlignedAssigner,
)


def test_canvas_box_to_native_inverts_flipped_and_unflipped_letterbox() -> None:
    from PIL import Image

    original_size = (200, 400)  # (H, W)
    image = Image.new("RGB", (400, 200), color=(100, 120, 140))
    native_box = torch.tensor([[40.0, 50.0, 120.0, 150.0]], dtype=torch.float32)

    for flip_prob, expected_flip in ((0.0, False), (1.0, True)):
        cfg = AugmentationConfig(
            horizontal_flip_probability=flip_prob,
            scale_min=1.0,
            scale_max=1.0,
            translation_fraction=0.0,
            color_jitter_probability=0.0,
            blur_probability=0.0,
            jpeg_probability=0.0,
            noise_probability=0.0,
        )
        tf = ProposalTransform(
            input_size=384,
            augmentation=cfg,
            training=True,
            tiny_area=4.0,
            tiny_min_side=2.0,
        )
        out = tf(
            image,
            native_box.clone(),
            torch.empty((0, 4)),
            seed=123,
            return_metadata=True,
        )
        canvas_boxes = out[1]
        transform_tuple = out[5]
        is_flipped = out[8]
        assert is_flipped is expected_flip
        recovered = torch.tensor(
            canvas_box_to_native(
                canvas_boxes[0],
                transform_tuple,
                original_size,
                horizontal_flip=is_flipped,
                margin=0.0,
            ),
            dtype=torch.float32,
        )
        torch.testing.assert_close(recovered, native_box[0], atol=1e-4, rtol=1e-4)


def test_proposal_and_quad_transforms_return_retained_indices_and_flip_flag() -> None:
    from PIL import Image

    image = Image.new("RGB", (200, 100), color=(128, 128, 128))
    cfg = AugmentationConfig(
        horizontal_flip_probability=1.0,
        scale_min=1.0,
        scale_max=1.0,
        translation_fraction=0.0,
        color_jitter_probability=0.0,
        blur_probability=0.0,
        jpeg_probability=0.0,
        noise_probability=0.0,
    )
    boxes = torch.tensor(
        [
            [0.0, 0.0, 0.1, 0.1],  # Degenerate -> dropped
            [20.0, 10.0, 120.0, 80.0],  # Valid -> retained index 1
        ],
        dtype=torch.float32,
    )
    hbb_tf = ProposalTransform(
        input_size=128,
        augmentation=cfg,
        training=True,
        tiny_area=4.0,
        tiny_min_side=2.0,
    )
    hbb_out = hbb_tf(
        image,
        boxes.clone(),
        torch.empty((0, 4)),
        seed=7,
        return_metadata=True,
    )
    assert hbb_out[1].shape == (1, 4)
    assert hbb_out[7] == (1,)
    assert hbb_out[8] is True

    quads = torch.stack([quad_from_bbox(b) for b in boxes])
    quad_tf = QuadProposalTransform(
        input_size=128,
        augmentation=cfg,
        training=True,
        regular_min_side=4.0,
    )
    quad_out = quad_tf(
        image,
        quads.clone(),
        torch.empty((0, 4, 2)),
        seed=7,
        return_metadata=True,
    )
    assert quad_out[1].shape == (1, 4, 2)
    assert quad_out[6] == (1,)
    assert quad_out[7] is True


def test_pan_and_3dmf_detectors_export_and_backprop() -> None:
    hbb = StudentDetector(
        backbone=StubP2Backbone(),
        fpn_channels=128,
        neck_type="pan",
        strides=(4, 8, 16, 32),
        dual_head=True,
        use_3dmf=True,
    )
    quad = QuadProposalDetector(
        backbone=StubP2Backbone(),
        fpn_channels=128,
        neck_type="pan",
        strides=(4, 8, 16, 32),
        dual_head=True,
        use_3dmf=True,
    )
    x = torch.randn(2, 3, 64, 64)
    primary_hbb, aux_hbb = hbb.train()(x)
    assert len(primary_hbb.objectness) == 4
    assert len(aux_hbb.objectness) == 4
    loss_hbb = sum(t.mean() for t in primary_hbb.objectness) + sum(
        t.mean() for t in aux_hbb.objectness
    )
    loss_hbb.backward()
    assert hbb.head.filter_3dmf is not None
    assert hbb.aux_head is not None and hbb.aux_head.filter_3dmf is None
    assert hbb.head.filter_3dmf.in_proj.weight.grad is not None

    primary_q, aux_q = quad.train()(x)
    loss_q = sum(t.mean() for t in primary_q.quality) + sum(
        t.mean() for t in aux_q.quality
    )
    loss_q.backward()
    assert quad.head.filter_3dmf is not None
    assert quad.head.filter_3dmf.in_proj.weight.grad is not None

    # Static torch.export compatibility check in eval mode
    exported = torch.export.export(hbb.eval(), (x[:1],))
    out_exp = exported.module()(x[:1])
    assert out_exp.objectness[0].shape == (1, 1, 16, 16)


def test_one_to_one_targets_supervise_unmatched_gt_interior_as_trusted_background() -> None:
    strides = (4, 8, 16, 32)
    shapes = ((16, 16), (8, 8), (4, 4), (2, 2))
    sample = ProposalSample(
        image=torch.zeros(3, 64, 64),
        boxes=torch.tensor([[8.0, 8.0, 48.0, 48.0]]),
        ignore_boxes=torch.empty((0, 4)),
        valid_mask=torch.ones(64, 64, dtype=torch.bool),
        image_id=1,
        source_dataset="nuimages",  # Sparse dataset without trusted_background_boxes
        domain="automotive",
        camera_type="perspective",
        background_supervision=False,
        original_size=(64, 64),
        transform=(1.0, 0.0, 0.0),
    )
    hbb_targets = TargetBuilder(
        TaskAlignedAssigner(strides=strides, assignment_mode="one_to_one"),
        default_background_weight=0.05,
    )([sample], shapes, device=torch.device("cpu"))
    assert int(hbb_targets.positive_mask.sum().item()) == 1
    assert int(hbb_targets.ignore_mask.sum().item()) == 0
    assert int(hbb_targets.trusted_background_mask.sum().item()) > 10
    assert hbb_targets.matched_gt_indices is not None

    quad_targets = QuadTargetBuilder(
        QuadTaskAlignedAssigner(strides=strides, assignment_mode="one_to_one"),
        weak_negative_weight=0.05,
    )([sample], shapes, device=torch.device("cpu"))
    assert int(quad_targets.positive_mask.sum().item()) == 1
    assert int(quad_targets.trusted_background_mask.sum().item()) > 10


def test_true_gfl_num_positives_normalization_and_strict_quad_iou_target() -> None:
    strides = (8, 16, 32)
    shapes = ((8, 8), (4, 4), (2, 2))
    sample = ProposalSample(
        image=torch.zeros(3, 64, 64),
        boxes=torch.tensor([[8.0, 8.0, 40.0, 40.0]]),
        ignore_boxes=torch.empty((0, 4)),
        valid_mask=torch.ones(64, 64, dtype=torch.bool),
        image_id=1,
        source_dataset="coco_2017",
        domain="general",
        camera_type="perspective",
        background_supervision=True,
        original_size=(64, 64),
        transform=(1.0, 0.0, 0.0),
        trusted_background_boxes=torch.tensor([[0.0, 0.0, 64.0, 64.0]]),
    )
    hbb_targets = TargetBuilder(
        TaskAlignedAssigner(strides=strides, assignment_mode="one_to_one")
    )([sample], shapes, device=torch.device("cpu"))
    det = StudentDetector(backbone=StubP2Backbone(), strides=(4, 8, 16, 32))
    del det
    from student_detector.head import DetectorOutput

    hbb_out = DetectorOutput(
        objectness=tuple(torch.zeros(1, 1, h, w) for h, w in shapes),
        box_distances=tuple(torch.ones(1, 4, h, w) * 8.0 for h, w in shapes),
        centerness=tuple(torch.zeros(1, 1, h, w) for h, w in shapes),
    )
    strict_loss = ProposalLoss(
        strides=strides,
        objectness_normalization="num_positives",
        use_background_floor=False,
    )(hbb_out, hbb_targets)
    floored_loss = ProposalLoss(
        strides=strides,
        objectness_normalization="num_positives",
        use_background_floor=True,
    )(hbb_out, hbb_targets)
    assert strict_loss.objectness > floored_loss.objectness

    # Quad strict exact_iou without 0.5 * proxy_quality floor (tested on both
    # collapsed zero-area quads and valid non-overlapping shifted quads)
    quad_targets = QuadTargetBuilder(
        QuadTaskAlignedAssigner(strides=strides, assignment_mode="one_to_one")
    )([sample], shapes, device=torch.device("cpu"))
    collapsed_offsets = tuple(torch.full((1, 8, h, w), 20.0) for h, w in shapes)
    q_collapsed = QuadProposalLoss(
        strides=strides,
        quality_target_mode="iou",
        quality_normalization="num_positives",
        use_proxy_quality_floor=False,
    )(
        QuadDetectorOutput(
            quality=tuple(torch.zeros(1, 1, h, w) for h, w in shapes),
            corner_offsets=collapsed_offsets,
        ),
        quad_targets,
    )
    assert float(q_collapsed.quality_target_mean.item()) == 0.0
    assert torch.isfinite(q_collapsed.total)

    valid_shifted_vec = torch.tensor(
        [19.0, 19.0, 21.0, 19.0, 21.0, 21.0, 19.0, 21.0]
    ).view(1, 8, 1, 1)
    shifted_offsets = tuple(valid_shifted_vec.expand(1, 8, h, w) for h, w in shapes)
    q_strict = QuadProposalLoss(
        strides=strides,
        quality_target_mode="iou",
        quality_normalization="num_positives",
        use_proxy_quality_floor=False,
    )(
        QuadDetectorOutput(
            quality=tuple(torch.zeros(1, 1, h, w) for h, w in shapes),
            corner_offsets=shifted_offsets,
        ),
        quad_targets,
    )
    assert float(q_strict.quality_target_mean.item()) == 0.0


def test_quad_decoder_rejects_bowtie_and_diverged_oob_quads() -> None:
    quality = (
        torch.tensor([[[[5.0, 4.0]]]]),
        torch.full((1, 1, 1, 1), -10.0),
        torch.full((1, 1, 1, 1), -10.0),
    )
    # Anchor 0: self-intersecting bow-tie; Anchor 1: 99% out-of-bounds diverged quad
    bowtie = torch.tensor([-1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0])
    diverged = torch.tensor([-50.0, -50.0, 50.0, -50.0, 50.0, 50.0, -50.0, 50.0])
    offsets_l0 = torch.stack([bowtie, diverged], dim=-1).view(1, 8, 1, 2)
    output = QuadDetectorOutput(
        quality=quality,
        corner_offsets=(offsets_l0, torch.zeros(1, 8, 1, 1), torch.zeros(1, 8, 1, 1)),
    )
    det = QuadInferenceDecoder(pre_nms_top_k=5, max_proposals=5)(output, (32, 32))[0]
    assert det.quads.shape[0] == 0
    assert det.invalid_candidate_count >= 2


def test_containment_aware_nms_suppresses_same_class_fragments_preserves_parts() -> None:
    # Box 0: building [0, 0, 100, 100] (class 0)
    # Box 1: contained building fragment [10, 10, 40, 40] (class 0, IoMin = 1.0 > 0.80)
    # Box 2: window inside building [50, 20, 70, 50] (class 1, IoMin = 1.0, different class!)
    boxes = torch.tensor(
        [
            [0.0, 0.0, 100.0, 100.0],
            [10.0, 10.0, 40.0, 40.0],
            [50.0, 20.0, 70.0, 50.0],
        ]
    )
    scores = torch.tensor([0.92, 0.85, 0.88])
    classes = torch.tensor([0, 0, 1])

    keep_hbb = class_agnostic_nms(
        boxes,
        scores,
        iou_threshold=0.50,
        nms_mode="containment_aware",
        iomin_threshold=0.80,
        class_ids=classes,
    )
    assert keep_hbb.tolist() == [0, 2]

    quads = torch.stack([quad_from_bbox(b) for b in boxes])
    keep_quad = polygon_nms(
        quads,
        scores,
        iou_threshold=0.50,
        nms_mode="containment_aware",
        iomin_threshold=0.80,
        class_ids=classes,
    )
    assert keep_quad.tolist() == [0, 2]


def test_option_a_dense_vs_indexed_exact_parity_and_decoder_flat_indices() -> None:
    torch.manual_seed(0)
    head_a = DenseAnchorSemanticHead(
        in_channels=128, hidden_channels=128, embedding_dim=768
    ).eval()
    features = (
        torch.randn(2, 128, 16, 16),
        torch.randn(2, 128, 8, 8),
        torch.randn(2, 128, 4, 4),
        torch.randn(2, 128, 2, 2),
    )
    dense_maps = head_a.forward_dense(features)
    flat_dense = torch.cat(
        [m.permute(0, 2, 3, 1).reshape(2, -1, 768) for m in dense_maps],
        dim=1,
    )

    batch_idx = torch.tensor([0, 0, 1, 1], dtype=torch.long)
    anchor_idx = torch.tensor([0, 255, 256 + 13, 256 + 64 + 16 + 3], dtype=torch.long)
    indexed = head_a.forward_indexed(features, batch_idx, anchor_idx)
    expected = flat_dense[batch_idx, anchor_idx]
    torch.testing.assert_close(indexed, expected, atol=1e-6, rtol=1e-5)

    # Verify InferenceDecoder exposes flat_indices matching Option A anchor space
    det_model = StudentDetector(
        backbone=StubP2Backbone(),
        fpn_channels=128,
        neck_type="pan",
        strides=(4, 8, 16, 32),
    ).eval()
    imgs = torch.randn(1, 3, 64, 64)
    feats = det_model.forward_features(imgs)
    raw_out = det_model.head(feats)
    decoded = InferenceDecoder(strides=(4, 8, 16, 32), top_k=20, max_detections=5)(
        raw_out, (64, 64)
    )[0]
    assert decoded.flat_indices is not None
    assert decoded.flat_indices.shape == decoded.scores.shape


def test_option_b_vertical_shift_invariance_and_open_vocab_sigmoid_loss() -> None:
    torch.manual_seed(42)
    head_b = RoIPooledSemanticHead(
        in_channels=128,
        roi_channels=96,
        embedding_dim=768,
        strides=(4, 8, 16, 32),
    ).eval()
    # Spatially uniform feature maps: shifting box from y=20 (top) to y=300 (bottom)
    # MUST produce bit-identical embeddings because geom_encoder shortcut is gone!
    uniform_feats = tuple(
        torch.ones(1, 128, 384 // s, 384 // s) * 0.5 for s in (4, 8, 16, 32)
    )
    top_box = torch.tensor([[100.0, 20.0, 180.0, 60.0]])
    bottom_box = torch.tensor([[100.0, 300.0, 180.0, 340.0]])
    z_top = head_b(uniform_feats, [top_box], image_shapes=[(384, 384)])
    z_bot = head_b(uniform_feats, [bottom_box], image_shapes=[(384, 384)])
    torch.testing.assert_close(z_top, z_bot, atol=1e-6, rtol=1e-6)

    # Test OpenVocabularySigmoidLoss across positive (>=0), background (-1), and OOV (-2)
    head_b.train()
    text_bank = F.normalize(torch.randn(38, 768), p=2.0, dim=-1)
    boxes = [
        torch.tensor(
            [
                [20.0, 20.0, 80.0, 80.0],  # class 3 (in-vocab)
                [100.0, 100.0, 160.0, 160.0],  # -1 (hard-negative clutter)
                [200.0, 200.0, 260.0, 260.0],  # -2 (OOV zero-shot GT object)
            ]
        )
    ]
    feats = tuple(
        torch.randn(1, 128, 384 // s, 384 // s, requires_grad=True)
        for s in (4, 8, 16, 32)
    )
    vis_emb = head_b(feats, boxes, image_shapes=[(384, 384)])
    labels = torch.tensor([3, -1, -2], dtype=torch.long)
    teacher_emb = F.normalize(torch.randn(3, 768), p=2.0, dim=-1)
    has_teacher = torch.tensor([True, False, True], dtype=torch.bool)

    criterion = OpenVocabularySigmoidLoss()
    loss_out = criterion(
        head_b,
        vis_emb,
        text_bank,
        labels,
        teacher_embeddings=teacher_emb,
        has_teacher=has_teacher,
    )
    assert loss_out.num_foreground == 1
    assert loss_out.num_background == 1
    assert loss_out.num_oov == 1
    assert torch.isfinite(loss_out.total)
    loss_out.total.backward()
    assert all(f.grad is not None and torch.isfinite(f.grad).all() for f in feats)


def test_open_vocab_ontology_disjoint_from_zero_shot_eval_categories() -> None:
    assert len(OPEN_VOCAB_CATEGORIES) == 37
    assert len(set(OPEN_VOCAB_CATEGORIES)) == 37
    assert set(OPEN_VOCAB_CATEGORIES).isdisjoint(set(ZERO_SHOT_EVAL_CATEGORIES))
    assert map_to_canonical("Traffic sign") == "traffic_sign"
    assert map_to_canonical("signboard") == "signboard"
    assert map_to_canonical("windowpane") == "window"
    assert map_to_canonical("door") == "door"
    assert map_to_canonical("lamp") == "indoor_lamp_or_light_fixture"
    assert map_to_canonical("bed") == "bed_and_bedding"


def test_quad_fp16_amp_stability_large_canvas() -> None:
    """Ensure FP16 quads up to 384x384 do not overflow 65504 or mismatch dtypes in NMS/IoU."""
    from student_detector.quad_geometry import pairwise_quad_iou

    q_a = quad_from_bbox(torch.tensor([10.0, 10.0, 360.0, 360.0])).unsqueeze(0).half()
    q_b = quad_from_bbox(torch.tensor([20.0, 20.0, 350.0, 350.0])).unsqueeze(0).half()
    iou = pairwise_quad_iou(q_a, q_b)
    assert iou.dtype == torch.float32
    assert torch.isfinite(iou).all()
    assert 0.80 < float(iou[0, 0].item()) < 1.0

    keep, kept_scores = polygon_nms(
        torch.cat([q_a, q_b], dim=0),
        torch.tensor([0.9, 0.8], dtype=torch.float16),
        0.50,
        nms_mode="hard",
        return_scores=True,
    )
    assert keep.tolist() == [0]
    assert kept_scores.shape == (1,)


def test_slender_scale_gating_sub_stride_tube_and_small_object_calibration() -> None:
    """Verify tall 3px-wide poles between stride=4 grid columns match P2 and small objects avoid interior crush."""
    strides = (4, 8, 16, 32)
    shapes = ((96, 96), (48, 48), (24, 24), (12, 12))
    # 3px wide pole [14.2, 20.0, 17.2, 160.0] lies strictly between P2 grid x=14.0 and x=18.0
    # and has sqrt(area) = sqrt(420) = 20.5, plus a 5x10px traffic light [40.0, 40.0, 45.0, 50.0].
    thin_pole_and_light = torch.tensor(
        [
            [14.2, 20.0, 17.2, 160.0],
            [40.0, 40.0, 45.0, 50.0],
        ],
        dtype=torch.float32,
    )
    hbb_assigner = TaskAlignedAssigner(
        strides=strides, alpha=1.0, beta=4.0, assignment_mode="one_to_one"
    )
    hbb_assign = hbb_assigner.assign(thin_pole_and_light, shapes, (384, 384))
    assert int(hbb_assign.positive_mask.sum().item()) == 2
    # Both slender pole and 5x10 traffic light must route to P2 (level 0, stride=4)
    assert int(hbb_assign.positive_counts_per_level[0].item()) == 2
    assert (hbb_assign.box_targets[hbb_assign.positive_mask] >= 0.0).all()

    quads = torch.stack([quad_from_bbox(b) for b in thin_pole_and_light])
    quad_assigner = QuadTaskAlignedAssigner(
        strides=strides, alpha=1.0, beta=4.0, assignment_mode="one_to_one"
    )
    quad_assign = quad_assigner.assign(quads, shapes, (384, 384))
    assert int(quad_assign.positive_mask.sum().item()) == 2
    assert int(quad_assign.positive_counts_per_level[0].item()) == 2

    # Small/slender objects (min_side < 12) must NOT mark their 1-cell neighbors as trusted_background
    small_sample = ProposalSample(
        image=torch.zeros(3, 64, 64),
        boxes=torch.tensor([[20.0, 20.0, 25.0, 30.0]]),
        ignore_boxes=torch.empty((0, 4)),
        valid_mask=torch.ones(64, 64, dtype=torch.bool),
        image_id=2,
        source_dataset="bdd100k_images_100k",
        domain="automotive",
        camera_type="pinhole",
        background_supervision=False,
        original_size=(64, 64),
        transform=(1.0, 0.0, 0.0),
    )
    small_targets = TargetBuilder(
        TaskAlignedAssigner(
            strides=(4, 8, 16, 32), alpha=1.0, beta=4.0, assignment_mode="one_to_one"
        ),
        default_background_weight=0.25,
    )([small_sample], ((16, 16), (8, 8), (4, 4), (2, 2)), device=torch.device("cpu"))
    assert int(small_targets.positive_mask.sum().item()) == 1
    assert int(small_targets.trusted_background_mask.sum().item()) == 0


def test_federated_and_seesaw_open_vocab_sigmoid_loss() -> None:
    """Verify dataset-aware negative masking protects unannotated poles/lights/bins and Seesaw balances tail classes."""
    mask = build_source_annotated_mask(["bdd100k_images_100k", "nuimages", "ade20k"])
    cat_to_idx = {c: i for i, c in enumerate(OPEN_VOCAB_CATEGORIES)}
    # BDD100K does not annotate pole, pillar, or trash_bin
    assert not bool(mask[0, cat_to_idx["pole"]].item())
    assert not bool(mask[0, cat_to_idx["pillar"]].item())
    assert not bool(mask[0, cat_to_idx["trash_bin"]].item())
    assert bool(mask[0, cat_to_idx["traffic_light"]].item())
    # nuImages does not annotate traffic_light, traffic_sign, pole, pillar, or trash_bin
    assert not bool(mask[1, cat_to_idx["traffic_light"]].item())
    assert not bool(mask[1, cat_to_idx["traffic_sign"]].item())
    assert not bool(mask[1, cat_to_idx["pole"]].item())
    # ADE20K annotates pillar, trash_bin, pole, signboard
    assert bool(mask[2, cat_to_idx["pillar"]].item())
    assert bool(mask[2, cat_to_idx["trash_bin"]].item())
    assert "woodscape_rgb_fisheye" in SOURCE_ANNOTATED_CATEGORIES

    num_classes = len(OPEN_VOCAB_CATEGORIES)
    head = DenseAnchorSemanticHead(
        in_channels=32, hidden_channels=32, embedding_dim=768
    )
    txt = F.normalize(torch.randn(num_classes, 768), p=2.0, dim=-1)
    vis = torch.randn(2, 768, requires_grad=True)
    # Row 0: background (-1) from bdd100k; Row 1: positive trash_bin
    trash_idx = cat_to_idx["trash_bin"]
    labels = torch.tensor([-1, trash_idx], dtype=torch.long)
    ann_mask = build_source_annotated_mask(["bdd100k_images_100k", "ade20k"])
    counts = torch.full((num_classes,), 5000.0)
    counts[trash_idx] = 45.0

    criterion = OpenVocabularySigmoidLoss(class_counts=counts)
    with torch.autocast("cpu", dtype=torch.bfloat16):
        out = criterion(head, vis, txt, labels, annotated_mask=ann_mask)
    assert torch.isfinite(out.total)
    out.total.backward()
    assert vis.grad is not None and torch.isfinite(vis.grad).all()


def test_forward_eval_head_blends_primary_and_aux() -> None:
    """Verify forward_eval_head blends primary 3DMF and 1-to-many aux logits on HBB and Quad."""
    strides = (4, 8, 16, 32)
    x = torch.randn(1, 3, 64, 64)
    det_hbb = StudentDetector(
        backbone=StubP2Backbone(),
        fpn_channels=32,
        neck_type="pan",
        strides=strides,
        dual_head=True,
        use_3dmf=True,
    )
    det_hbb.eval()
    fh = det_hbb.forward_features(x)
    out_h = det_hbb.forward_eval_head(fh, aux_weight=0.40)
    assert len(out_h.objectness) == 4
    assert len(out_h.box_distances) == 4

    det_quad = QuadProposalDetector(
        backbone=StubP2Backbone(),
        fpn_channels=32,
        neck_type="pan",
        strides=strides,
        dual_head=True,
        use_3dmf=True,
    )
    det_quad.eval()
    fq = det_quad.forward_features(x)
    out_q = det_quad.forward_eval_head(fq, aux_weight=0.40)
    assert len(out_q.quality) == 4
    assert len(out_q.corner_offsets) == 4


def test_distractor_veto_and_filter_semantic_detections() -> None:
    """Verify distractor veto suppresses background probs and semantic NMS deduplicates stacks/sub-parts."""
    head = DenseAnchorSemanticHead(in_channels=32, hidden_channels=32, embedding_dim=16)
    txt = F.normalize(torch.eye(4, 16), p=2.0, dim=-1)
    dist = F.normalize(torch.tensor([[0.0] * 10 + [1.0] + [0.0] * 5]), p=2.0, dim=-1)
    head.set_distractor_embeddings(dist)

    vis_fg = F.normalize(txt[0:1], p=2.0, dim=-1)
    vis_bg = F.normalize(dist + 0.1 * txt[0:1], p=2.0, dim=-1)
    _, p_fg = head.classify_embeddings(vis_fg, txt)
    _, p_bg = head.classify_embeddings(vis_bg, txt)
    assert float(p_fg[0, 0].item()) > 0.80
    assert float(p_bg[0].max().item()) < 0.12

    # Check strict load_state_dict round-trip with distractor buffer
    reloaded = DenseAnchorSemanticHead(in_channels=32, hidden_channels=32, embedding_dim=16)
    reloaded.load_state_dict(head.state_dict(), strict=True)
    assert reloaded.distractor_embeddings is not None

    # Semantic containment filter:
    # 0: trash_bin [0, 0, 100, 100] (joint high)
    # 1: overlapping duplicate trash_bin [5, 10, 95, 95] (same class IoMin > 0.60 -> suppress)
    # 2: spurious license_plate sticker inside trash_bin [20, 20, 40, 35] -> suppress
    # 3: building [200, 0, 320, 150]
    # 4: valid window inside building [220, 20, 250, 60] -> keep!
    cats = ("trash_bin", "license_plate", "building", "window")
    boxes = torch.tensor(
        [
            [0.0, 0.0, 100.0, 100.0],
            [5.0, 10.0, 95.0, 95.0],
            [20.0, 20.0, 40.0, 35.0],
            [200.0, 0.0, 320.0, 150.0],
            [220.0, 20.0, 250.0, 60.0],
        ]
    )
    obj = torch.tensor([0.9, 0.8, 0.7, 0.85, 0.75])
    probs = torch.tensor(
        [
            [0.90, 0.01, 0.01, 0.01],
            [0.70, 0.01, 0.01, 0.01],
            [0.01, 0.60, 0.01, 0.01],
            [0.01, 0.01, 0.85, 0.01],
            [0.01, 0.01, 0.01, 0.80],
        ]
    )
    kept = filter_semantic_detections(boxes, obj, probs, cats)
    assert kept.tolist() == [0, 3, 4]


def test_visual_centroid_prompt_debiasing_and_loss() -> None:
    """Verify set_visual_centroid removes prompt baseline bias across classification & distillation loss."""
    head = DenseAnchorSemanticHead(in_channels=32, hidden_channels=32, embedding_dim=16)
    # Prompt 0 shares a +0.5 component with shared background vector; prompt 1 is orthogonal to it.
    bg_dir = F.normalize(torch.tensor([[0.0] * 15 + [1.0]]), p=2.0, dim=-1)
    p0 = F.normalize(
        torch.tensor([[1.0] + [0.0] * 14 + [0.5]]), p=2.0, dim=-1
    )
    p1 = F.normalize(
        torch.tensor([[0.0, 1.0] + [0.0] * 13 + [0.0]]), p=2.0, dim=-1
    )
    txt = torch.cat([p0, p1], dim=0)
    # Generic visual crop sitting along bg_dir with slightly stronger p1 content than p0 content
    vis = F.normalize(bg_dir + 0.10 * p1 + 0.05 * p0, p=2.0, dim=-1)

    # Without centroid de-biasing, p0 wins purely from bg_dir overlap
    logits_raw, _ = head.classify_embeddings(vis, txt, apply_calibration=False)
    assert int(logits_raw[0].argmax().item()) == 0

    # With centroid de-biasing set near bg_dir, p0's baseline offset is subtracted and p1 wins
    head.set_visual_centroid(bg_dir)
    bias = head.compute_prompt_bias(txt)
    assert float(bias[0, 0].item()) > float(bias[0, 1].item())
    assert abs(float(bias[0, 1].item())) < 1e-6
    logits_deb, _ = head.classify_embeddings(vis, txt, apply_calibration=False)
    assert int(logits_deb[0].argmax().item()) == 1

    # Round-trip load_state_dict preserves visual_centroid and OpenVocabularySigmoidLoss is finite
    reloaded = DenseAnchorSemanticHead(in_channels=32, hidden_channels=32, embedding_dim=16)
    reloaded.load_state_dict(head.state_dict(), strict=True)
    assert reloaded.visual_centroid is not None
    assert torch.allclose(reloaded.visual_centroid, head.visual_centroid)

    vis_train = vis.clone().detach().requires_grad_(True)
    teacher_emb = vis.clone().detach()
    criterion = OpenVocabularySigmoidLoss()
    loss_out = criterion(
        head,
        vis_train,
        txt,
        torch.tensor([-2], dtype=torch.long),
        teacher_embeddings=teacher_emb,
        has_teacher=torch.tensor([True]),
    )
    assert torch.isfinite(loss_out.total)
    loss_out.total.backward()
    assert vis_train.grad is not None and torch.isfinite(vis_train.grad).all()



