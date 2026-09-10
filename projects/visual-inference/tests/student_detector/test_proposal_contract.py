from __future__ import annotations

import torch

from student_detector.decoder import Detection
from student_detector.proposal_contract import (
    SourceTransform,
    records_from_hbb,
    records_from_quads,
)
from student_detector.quad_decoder import QuadDetection


def _transform() -> SourceTransform:
    return SourceTransform(
        scale=2.0,
        offset_x=0.0,
        offset_y=20.0,
        source_size=(100, 200),
        model_size=(240, 400),
    )


def test_hbb_contract_inverts_letterbox_and_uses_rectangular_corners() -> None:
    detection = Detection(
        boxes=torch.tensor([[20.0, 40.0, 100.0, 120.0]]),
        scores=torch.tensor([0.75]),
        levels=torch.tensor([3]),
        location_indices=torch.tensor([17]),
    )

    record = records_from_hbb(
        detection,
        image_id=9,
        source_dataset="fixture",
        source_transform=_transform(),
    )[0]

    assert record.proposal_id == "fixture:9:hbb:P3:17"
    assert record.model_geometry == (
        (20.0, 40.0),
        (100.0, 40.0),
        (100.0, 120.0),
        (20.0, 120.0),
    )
    assert record.source_geometry == (
        (10.0, 10.0),
        (50.0, 10.0),
        (50.0, 50.0),
        (10.0, 50.0),
    )


def test_contract_clips_to_source_boundaries() -> None:
    detection = QuadDetection(
        quads=torch.tensor(
            [[[-10.0, 0.0], [500.0, 0.0], [500.0, 250.0], [-10.0, 250.0]]]
        ),
        scores=torch.tensor([0.8]),
        levels=torch.tensor([4]),
        location_indices=torch.tensor([2]),
    )
    record = records_from_quads(
        detection,
        image_id=1,
        source_dataset="fixture",
        source_transform=_transform(),
    )[0]
    assert record.source_geometry == (
        (0.0, 0.0),
        (200.0, 0.0),
        (200.0, 100.0),
        (0.0, 100.0),
    )


def test_rectangular_hbb_and_quad_share_the_same_geometry_representation() -> None:
    box = torch.tensor([[20.0, 40.0, 100.0, 120.0]])
    quad = torch.tensor(
        [[[20.0, 40.0], [100.0, 40.0], [100.0, 120.0], [20.0, 120.0]]]
    )
    metadata = {
        "scores": torch.tensor([0.75]),
        "levels": torch.tensor([3]),
        "location_indices": torch.tensor([17]),
    }
    hbb = records_from_hbb(
        Detection(boxes=box, **metadata),
        image_id=9,
        source_dataset="fixture",
        source_transform=_transform(),
    )[0]
    quad_record = records_from_quads(
        QuadDetection(quads=quad, **metadata),
        image_id=9,
        source_dataset="fixture",
        source_transform=_transform(),
    )[0]

    assert hbb.model_geometry == quad_record.model_geometry
    assert hbb.source_geometry == quad_record.source_geometry


def test_equal_scores_are_ordered_by_stable_dense_identity() -> None:
    detection = QuadDetection(
        quads=torch.tensor(
            [
                [[0.0, 20.0], [2.0, 20.0], [2.0, 22.0], [0.0, 22.0]],
                [[4.0, 20.0], [6.0, 20.0], [6.0, 22.0], [4.0, 22.0]],
                [[8.0, 20.0], [10.0, 20.0], [10.0, 22.0], [8.0, 22.0]],
            ]
        ),
        scores=torch.tensor([0.5, 0.5, 0.6]),
        levels=torch.tensor([4, 3, 5]),
        location_indices=torch.tensor([2, 8, 1]),
    )

    first = records_from_quads(
        detection,
        image_id=4,
        source_dataset="fixture",
        source_transform=_transform(),
        max_proposals=2,
    )
    second = records_from_quads(
        detection,
        image_id=4,
        source_dataset="fixture",
        source_transform=_transform(),
        max_proposals=2,
    )

    assert [record.proposal_id for record in first] == [
        "fixture:4:quad:P5:1",
        "fixture:4:quad:P3:8",
    ]
    assert first == second


def test_score_threshold_filters_before_assigning_contiguous_ranks() -> None:
    detection = Detection(
        boxes=torch.tensor(
            [[0.0, 20.0, 2.0, 22.0], [4.0, 20.0, 6.0, 22.0]]
        ),
        scores=torch.tensor([0.4, 0.8]),
        levels=torch.tensor([3, 3]),
        location_indices=torch.tensor([1, 2]),
    )
    records = records_from_hbb(
        detection,
        image_id=1,
        source_dataset="fixture",
        source_transform=_transform(),
        score_threshold=0.5,
    )
    assert len(records) == 1
    assert records[0].rank == 0
    assert records[0].location_index == 2


def test_contract_requires_decoder_identity_metadata() -> None:
    detection = Detection(torch.empty((0, 4)), torch.empty((0,)))
    try:
        records_from_hbb(
            detection,
            image_id=1,
            source_dataset="fixture",
            source_transform=_transform(),
        )
    except ValueError as error:
        assert "retain level and location" in str(error)
    else:
        raise AssertionError("missing identity metadata must be rejected")
