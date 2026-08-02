from __future__ import annotations

from pathlib import Path

from dataset_pipeline.config import Config
from dataset_pipeline.g1_adjudication import adjudicate_row
from dataset_pipeline.reports import write_json


def _config(tmp_path: Path) -> Config:
    return Config(
        path=tmp_path / "config.yaml",
        workspace_root=tmp_path,
        datasets={},
        storage={},
        splits={},
        validation={"random_seed": 42},
        taxonomy_path=tmp_path / "taxonomy.json",
    )


def _row(quad: list[list[float]]) -> dict:
    return {
        "audit_id": "aaaaaaaaaaaaaaaa",
        "split": "train",
        "image_id": 1,
        "file_name": "images/train/one.jpg",
        "image_width": 12,
        "image_height": 12,
        "source_dataset": "source",
        "source_split": "train",
        "source_image_id": "train/img/one.jpg",
        "source_annotation_id": "object-1",
        "source_category": "object",
        "state": "positive",
        "geometry_tier": "fitted_quad",
        "fit_coverage": 1.0,
        "fit_tightness": 1.0,
        "quad": quad,
    }


def test_automatic_adjudication_passes_independently_consistent_quad(
    tmp_path: Path,
) -> None:
    annotation = tmp_path / "intermediate/filtered/source/train/ann/one.jpg.json"
    write_json(
        annotation,
        {
            "objects": [
                {
                    "sourceAnnotationId": "object-1",
                    "geometryType": "polygon",
                    "points": {"exterior": [[1, 1], [9, 1], [9, 9], [1, 9], [1, 1]]},
                }
            ]
        },
    )
    result = adjudicate_row(_config(tmp_path), _row([[1, 1], [9, 1], [9, 9], [1, 9]]))
    assert result["automatic_result"] == "PASS"
    assert result["independent_fit_coverage"] == 1.0
    assert all(result["automatic_checks"].values())


def test_automatic_adjudication_fails_quad_that_misses_source_geometry(
    tmp_path: Path,
) -> None:
    annotation = tmp_path / "intermediate/filtered/source/train/ann/one.jpg.json"
    write_json(
        annotation,
        {
            "objects": [
                {
                    "sourceAnnotationId": "object-1",
                    "geometryType": "polygon",
                    "points": {"exterior": [[1, 1], [9, 1], [9, 9], [1, 9], [1, 1]]},
                }
            ]
        },
    )
    result = adjudicate_row(_config(tmp_path), _row([[1, 1], [5, 1], [5, 9], [1, 9]]))
    assert result["automatic_result"] == "FAIL"
    assert (
        result["automatic_checks"]["source_geometry_containment_at_least_0_98"] is False
    )
