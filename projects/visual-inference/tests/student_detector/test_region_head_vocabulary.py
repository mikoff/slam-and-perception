"""Tests for RegionEmbeddingHead context expansion, geometric conditioning, and VocabularyBank calibration."""

from __future__ import annotations

import torch
import torch.nn.functional as F

from student_detector.region_head import RegionEmbeddingHead
from student_detector.vocabulary import (
    VocabularyBank,
    expected_calibration_error,
    fit_temperature,
)


def _features() -> tuple[torch.Tensor, ...]:
    return tuple(torch.randn(1, 96, 384 // s, 384 // s) for s in (4, 8, 16, 32))


def test_expand_boxes_matches_teacher_margin_and_clamps() -> None:
    boxes = torch.tensor([[100.0, 100.0, 200.0, 150.0], [0.0, 0.0, 384.0, 384.0]])
    out = RegionEmbeddingHead.expand_boxes(boxes, 0.15, (384, 384))
    torch.testing.assert_close(out[0], torch.tensor([85.0, 92.5, 215.0, 157.5]))
    torch.testing.assert_close(out[1], torch.tensor([0.0, 0.0, 384.0, 384.0]))


def test_compute_box_geometry() -> None:
    boxes = torch.tensor([[0.0, 0.0, 384.0, 192.0]])  # Top half banner
    geom = RegionEmbeddingHead.compute_box_geometry(boxes, (384, 384))
    assert geom.shape == (1, 6)
    xc, yc, w, h, aspect_log, area_norm = geom[0].tolist()
    assert abs(xc - 0.5) < 1e-4
    assert abs(yc - 0.25) < 1e-4
    assert abs(w - 1.0) < 1e-4
    assert abs(h - 0.5) < 1e-4
    assert aspect_log > 0.0  # Wide banner
    assert area_norm > 0.0

    # Empty tensor check
    empty_geom = RegionEmbeddingHead.compute_box_geometry(torch.empty((0, 4)), (384, 384))
    assert empty_geom.shape == (0, 6)


def test_head_outputs_unit_embeddings_and_handles_empty() -> None:
    for use_ctx in (True, False):
        for use_geom in (True, False):
            head = RegionEmbeddingHead(use_context_roi=use_ctx, use_box_geometry=use_geom).eval()
            feats = _features()
            emb = head(feats, [torch.tensor([[10.0, 20.0, 120.0, 200.0]])])
            assert emb.shape == (1, 768)
            torch.testing.assert_close(emb.norm(dim=-1), torch.ones(1))
            assert head(feats, [torch.empty((0, 4))]).shape == (0, 768)


def test_head_config_serialization() -> None:
    head = RegionEmbeddingHead(
        context_margin=0.40,
        focal_margin=0.10,
        use_context_roi=True,
        use_box_geometry=True,
        geom_dim=32,
    )
    cfg = head.get_config()
    head2 = RegionEmbeddingHead.from_config(cfg)
    assert head2.context_margin == 0.40
    assert head2.focal_margin == 0.10
    assert head2.use_context_roi is True
    assert head2.use_box_geometry is True
    assert head2.geom_dim == 32


def test_temperature_changes_confidence_not_label() -> None:
    bank = VocabularyBank(
        categories=("a", "b", "c"),
        embeddings=F.normalize(torch.randn(3, 8), dim=-1),
        logit_scale=4.0,
        logit_bias=-10.0,
    )
    x = F.normalize(torch.randn(5, 8), dim=-1)
    _, p1 = bank.classify(x)
    _, p4 = bank.classify(x, temperature=4.0)
    assert torch.equal(p1.argmax(-1), p4.argmax(-1))
    assert (p4.max(-1).values <= p1.max(-1).values + 1e-6).all()


def test_fit_temperature_reduces_overconfidence() -> None:
    torch.manual_seed(0)
    n, c = 4000, 10
    targets = torch.randint(0, c, (n,))
    # Overconfident logits that are right only ~50% of the time.
    preds = torch.where(torch.rand(n) < 0.5, targets, torch.randint(0, c, (n,)))
    logits = torch.randn(n, c) + 12.0 * F.one_hot(preds, c)
    t = fit_temperature(logits, targets)
    assert t > 1.0
    ece_raw = expected_calibration_error(logits.softmax(-1), targets)
    ece_cal = expected_calibration_error((logits / t).softmax(-1), targets)
    assert ece_cal < ece_raw
    assert ece_cal < 0.05
