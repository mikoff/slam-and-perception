from __future__ import annotations

import sys

import pytest

from scripts.visualize_quad_validation import (
    _args,
    _checkpoint_state_key,
    _html_document,
    _select_records_per_source,
)
from student_detector.data import ImageRecord


def test_visualization_loads_ema_only_checkpoint() -> None:
    checkpoint = {"selected_state": "ema_model", "ema_model": {}}

    assert _checkpoint_state_key(checkpoint, raw_model=False) == "ema_model"


def test_visualization_rejects_raw_request_for_ema_only_checkpoint() -> None:
    checkpoint = {"selected_state": "ema_model", "ema_model": {}}

    with pytest.raises(ValueError, match="raw"):
        _checkpoint_state_key(checkpoint, raw_model=True)


def test_visualization_html_has_independent_overlay_controls() -> None:
    document = _html_document(
        [
            {
                "image": "0000_1.jpg",
                "imageId": 1,
                "sourceDataset": "bdd100k_images_100k",
                "domain": "automotive",
                "groundTruth": [],
                "ignored": [],
                "proposals": [],
            }
        ],
        initial_threshold=0.17,
    )

    assert 'class="threshold" type="range"' in document
    assert 'class="show-ground-truth"' in document
    assert 'class="show-ignore"' in document
    assert 'class="show-proposals"' in document
    assert "const initialThreshold = 0.17" in document
    assert '"image":"0000_1.jpg"' in document
    assert "source=${record.sourceDataset}" in document


def test_visualization_accepts_a_local_nms_override(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "visualize_quad_validation.py",
            "--checkpoint",
            "best_raw.pt",
            "--nms-iou-threshold",
            "0.7",
        ],
    )

    assert _args().nms_iou_threshold == pytest.approx(0.7)


def test_visualization_uses_readable_audit_defaults(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        sys,
        "argv",
        ["visualize_quad_validation.py", "--checkpoint", "best_raw.pt"],
    )

    args = _args()

    assert args.nms_iou_threshold == pytest.approx(0.7)
    assert args.score_threshold == pytest.approx(0.4)


def test_visualization_samples_each_source_evenly_and_interleaves() -> None:
    records = [
        ImageRecord(
            index, index, f"{index}.jpg", 32, 32, source, "perspective", False, 1
        )
        for source in ("source_b", "source_a")
        for index in range(10)
    ]

    selected, counts = _select_records_per_source(records, 3)

    assert counts == {"source_a": 3, "source_b": 3}
    assert [record.source_dataset for record in selected] == [
        "source_a",
        "source_b",
        "source_a",
        "source_b",
        "source_a",
        "source_b",
    ]
    assert [record.image_id for record in selected[::2]] == [0, 4, 9]
