from __future__ import annotations

import json
from pathlib import Path
import sqlite3

import torch

from student_detector.proposal_evaluation import (
    boxes_to_quads,
    load_ego_source_quads,
    source_size_bands,
    transform_ego_quads,
    valid_output_levels,
)


def test_boxes_to_quads_preserves_box_corners() -> None:
    boxes = torch.tensor([[1.0, 2.0, 5.0, 7.0]])

    quads = boxes_to_quads(boxes)

    assert torch.equal(
        quads,
        torch.tensor([[[1.0, 2.0], [5.0, 2.0], [5.0, 7.0], [1.0, 7.0]]]),
    )
    assert boxes_to_quads(torch.empty((0, 4))).shape == (0, 4, 2)


def test_source_size_bands_uses_unscaled_short_side() -> None:
    quads = boxes_to_quads(
        torch.tensor(
            [
                [0.0, 0.0, 6.0, 10.0],
                [0.0, 0.0, 16.0, 40.0],
                [0.0, 0.0, 520.0, 700.0],
            ]
        )
    )

    assert source_size_bands(quads, scale=2.0) == (
        "[0,4)",
        "[8,16)",
        "[256,inf)",
    )


def test_transform_ego_quads_applies_scale_offset_and_clamp() -> None:
    source = boxes_to_quads(torch.tensor([[-2.0, 1.0, 8.0, 12.0]]))

    transformed = transform_ego_quads(
        source,
        transform=(2.0, 3.0, -1.0),
        input_size=20,
        device=torch.device("cpu"),
    )

    assert transformed.shape == (1, 4, 2)
    assert torch.equal(transformed.amin(dim=1), torch.tensor([[0.0, 1.0]]))
    assert torch.equal(transformed.amax(dim=1), torch.tensor([[19.0, 20.0]]))
    assert transform_ego_quads(
        None, (1.0, 0.0, 0.0), 20, torch.device("cpu")
    ).shape == (0, 4, 2)


def test_load_ego_source_quads_filters_category_and_image_ids(
    tmp_path: Path,
) -> None:
    index_path = tmp_path / "index.sqlite"
    with sqlite3.connect(index_path) as connection:
        connection.execute(
            "CREATE TABLE annotations "
            "(image_id INTEGER, category_name TEXT, quad_json TEXT)"
        )
        rows = (
            (1, "ego_platform_bodywork", json.dumps([[0, 0], [2, 0], [2, 1], [0, 1]])),
            (1, "object", json.dumps([[4, 4], [5, 4], [5, 5], [4, 5]])),
            (2, "ego_platform_bodywork", json.dumps([[1, 1], [3, 1], [3, 2], [1, 2]])),
        )
        connection.executemany("INSERT INTO annotations VALUES (?, ?, ?)", rows)

    result = load_ego_source_quads(index_path, [1])

    assert set(result) == {1}
    assert result[1].shape == (1, 4, 2)
    assert torch.equal(result[1][0, 0], torch.tensor([0.0, 0.0]))


def test_valid_output_levels_matches_output_shapes() -> None:
    valid_mask = torch.ones((16, 16), dtype=torch.bool)
    output_levels = (torch.empty((1, 1, 4, 4)), torch.empty((1, 1, 2, 2)))

    levels = valid_output_levels(valid_mask, output_levels, (4, 8), torch.device("cpu"))

    assert tuple(level.shape for level in levels) == ((1, 4, 4), (1, 2, 2))
    assert all(level.all() for level in levels)
