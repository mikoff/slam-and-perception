from __future__ import annotations

from html.parser import HTMLParser

import torch

from scripts.visualize_quad_comparison import (
    _embed_record_images,
    _html_document,
    _proposal_rows,
)


def test_proposal_rows_preserve_rank_score_and_match_quality() -> None:
    rows = _proposal_rows(
        torch.tensor(
            [
                [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]],
                [[4.0, 4.0], [5.0, 4.0], [5.0, 5.0], [4.0, 5.0]],
            ]
        ),
        torch.tensor([0.9, 0.4]),
        torch.tensor([0.75, 0.0]),
    )

    assert rows[0]["rank"] == 1
    assert rows[0]["score"] == 0.9
    assert rows[0]["bestIou"] == 0.75
    assert rows[1]["rank"] == 2


def test_comparison_html_has_three_panels_and_audit_controls() -> None:
    document = _html_document(
        {
            "sourceCounts": {"source": 1},
            "subsetSha": "subset",
            "models": {
                "historical": {"nms": 0.9},
                "current": {"nms": 0.75},
            },
            "records": [],
            "unsafe": "</script>",
        }
    )
    HTMLParser().feed(document)

    assert "<\\/script>" in document
    assert "Ground truth" in document
    assert "Historical" in document
    assert "Current" in document
    assert 'id="topk"' in document
    assert 'id="match-color"' in document
    assert "createImageBitmap(new Blob" in document
    assert "__AUDIT_DATA__" not in document


def test_gallery_can_embed_images_for_single_file_viewing(tmp_path) -> None:
    images = tmp_path / "images"
    images.mkdir()
    (images / "sample.jpg").write_bytes(b"jpeg-bytes")
    data = {"records": [{"image": "images/sample.jpg"}]}

    _embed_record_images(data, tmp_path)

    assert data["records"][0]["image"] == "data:image/jpeg;base64,anBlZy1ieXRlcw=="
