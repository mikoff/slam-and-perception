"""Unit and integration tests for non-destructive annotation hygiene rules."""

from __future__ import annotations

import json
from pathlib import Path
import sqlite3

from PIL import Image
import torch

from student_detector.annotation_hygiene import (
    RawAnnotationRow,
    sanitize_image_annotations,
)
from student_detector.human_gt_data import HumanGTHBBDataset, HumanGTQuadDataset
from student_detector.semantic_evaluation_data import load_semantic_image_records


def _quad_from_box(x1: float, y1: float, x2: float, y2: float) -> str:
    return json.dumps([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])


def test_nuimages_1to1_duplicate_drops_source_hbb_keeps_fitted_quad() -> None:
    rows = [
        RawAnnotationRow(596.9, 435.4, 648.0, 475.2, 0, "bicycle", "fitted_quad"),
        RawAnnotationRow(599.0, 437.0, 642.0, 472.0, 0, "bicycle", "source_hbb"),
    ]
    dec = sanitize_image_annotations(rows, source_dataset="nuimages")
    assert dec.kept_positive_indices == (0,)
    assert dec.dropped_indices == (1,)
    assert dec.demoted_to_ignore_indices == ()
    assert dec.stats["dropped_dup_hbb"] == 1
    assert dec.stats["dropped_occluded_quad_fragments"] == 0


def test_nuimages_occluded_multi_fragment_keeps_source_hbb_drops_pieces() -> None:
    # Mimics nuimages #188347 occluded pedestrian: 1 source_hbb + 4 fitted_quad sub-pieces.
    rows = [
        RawAnnotationRow(452.0, 357.7, 477.6, 391.2, 0, "adult", "fitted_quad"),
        RawAnnotationRow(455.0, 335.0, 501.0, 410.0, 0, "adult", "source_hbb"),
        RawAnnotationRow(461.0, 399.0, 492.0, 408.0, 0, "adult", "fitted_quad"),
        RawAnnotationRow(461.9, 329.7, 511.5, 375.1, 0, "adult", "fitted_quad"),
        RawAnnotationRow(480.0, 370.0, 502.0, 391.0, 0, "adult", "fitted_quad"),
    ]
    dec = sanitize_image_annotations(rows, source_dataset="nuimages")
    assert dec.kept_positive_indices == (1,)
    assert dec.dropped_indices == (0, 2, 3, 4)
    assert dec.stats["dropped_occluded_quad_fragments"] == 4


def test_nuimages_single_partial_fragment_keeps_source_hbb_drops_fragment() -> None:
    rows = [
        RawAnnotationRow(100.0, 100.0, 200.0, 200.0, 0, "car", "source_hbb"),
        RawAnnotationRow(110.0, 110.0, 145.0, 145.0, 0, "car", "fitted_quad"),
    ]
    dec = sanitize_image_annotations(rows, source_dataset="nuimages")
    assert dec.kept_positive_indices == (0,)
    assert dec.dropped_indices == (1,)
    assert dec.stats["dropped_occluded_quad_fragments"] == 1


def test_nuimages_unmatched_and_distinct_adjacent_objects_kept() -> None:
    rows = [
        RawAnnotationRow(10.0, 10.0, 60.0, 60.0, 0, "car", "source_hbb"),
        RawAnnotationRow(90.0, 10.0, 140.0, 60.0, 0, "car", "fitted_quad"),
        RawAnnotationRow(200.0, 100.0, 250.0, 150.0, 0, "car", "fitted_quad"),
    ]
    dec = sanitize_image_annotations(rows, source_dataset="nuimages")
    assert dec.kept_positive_indices == (0, 1, 2)
    assert dec.dropped_indices == ()


def test_cross_tier_applies_to_coco_not_bdd_or_ade20k() -> None:
    rows = [
        RawAnnotationRow(50.0, 50.0, 150.0, 150.0, 0, "car", "fitted_quad"),
        RawAnnotationRow(51.0, 51.0, 149.0, 149.0, 0, "car", "source_hbb"),
    ]
    dec_coco = sanitize_image_annotations(rows, source_dataset="coco_2017")
    assert dec_coco.dropped_indices == (1,)

    for ds in ("bdd100k_images_100k", "ade20k"):
        dec_other = sanitize_image_annotations(rows, source_dataset=ds)
        assert dec_other.kept_positive_indices == (0, 1)
        assert dec_other.dropped_indices == ()


def test_woodscape_demotes_noisy_classes() -> None:
    rows = [
        RawAnnotationRow(10.0, 10.0, 90.0, 90.0, 0, "trafficsign_indistingushable", "fitted_quad"),
        RawAnnotationRow(100.0, 10.0, 180.0, 90.0, 0, "other_wheeled_transport", "fitted_quad"),
        RawAnnotationRow(200.0, 10.0, 280.0, 90.0, 0, "movable_object", "fitted_quad"),
        RawAnnotationRow(300.0, 10.0, 380.0, 90.0, 0, "car", "fitted_quad"),
    ]
    dec = sanitize_image_annotations(rows, source_dataset="woodscape_rgb_fisheye")
    assert dec.kept_positive_indices == (3,)
    assert dec.demoted_to_ignore_indices == (0, 1, 2)
    assert dec.stats["woodscape_demoted_noisy_class"] == 3


def test_woodscape_tiny_fragments_and_valid_pole_rules() -> None:
    rows = [
        # <16px minor side on non-pole -> demote
        RawAnnotationRow(10.0, 10.0, 22.0, 50.0, 0, "person", "fitted_quad"),
        # 20x12 (<256 area) -> demote
        RawAnnotationRow(40.0, 10.0, 60.0, 22.0, 0, "car", "fitted_quad"),
        # Short pole stub (10x25: major=25 < 32) -> demote
        RawAnnotationRow(80.0, 10.0, 90.0, 35.0, 0, "pole", "fitted_quad"),
        # Valid tall thin pole (10x50: major=50 >= 32, area=500 >= 320) -> KEEP
        RawAnnotationRow(120.0, 10.0, 130.0, 60.0, 0, "pole", "fitted_quad"),
    ]
    dec = sanitize_image_annotations(rows, source_dataset="woodscape_rgb_fisheye")
    assert dec.kept_positive_indices == (3,)
    assert dec.demoted_to_ignore_indices == (0, 1, 2)
    assert dec.stats["woodscape_demoted_tiny_fragment"] == 3


def test_woodscape_nested_fragment_demoted_side_by_side_kept() -> None:
    rows = [
        RawAnnotationRow(700.0, 30.0, 780.0, 115.0, 0, "person", "fitted_quad"),
        RawAnnotationRow(733.0, 70.0, 759.0, 95.5, 0, "person", "fitted_quad"),
        RawAnnotationRow(937.0, 49.0, 1001.0, 120.0, 0, "person", "fitted_quad"),
    ]
    dec = sanitize_image_annotations(rows, source_dataset="woodscape_rgb_fisheye")
    assert dec.kept_positive_indices == (0, 2)
    assert dec.demoted_to_ignore_indices == (1,)
    assert dec.stats["woodscape_demoted_nested_fragment"] == 1


def _build_synthetic_hygiene_db(tmp_path: Path) -> tuple[Path, Path]:
    img_root = tmp_path / "imgs"
    img_root.mkdir()
    Image.new("RGB", (400, 400), "gray").save(img_root / "nu.jpg")
    Image.new("RGB", (400, 400), "gray").save(img_root / "ws.jpg")

    db_path = tmp_path / "test_enriched.sqlite"
    with sqlite3.connect(db_path) as conn:
        conn.execute(
            """
            CREATE TABLE images (
                row_index INTEGER PRIMARY KEY,
                image_id INTEGER UNIQUE NOT NULL,
                file_name TEXT NOT NULL,
                width INTEGER NOT NULL,
                height INTEGER NOT NULL,
                source_dataset TEXT NOT NULL,
                camera_type TEXT NOT NULL,
                background_supervision INTEGER NOT NULL,
                positive_count INTEGER NOT NULL DEFAULT 0
            )
            """
        )
        conn.execute(
            """
            CREATE TABLE annotations (
                image_id INTEGER NOT NULL,
                x1 REAL NOT NULL, y1 REAL NOT NULL,
                x2 REAL NOT NULL, y2 REAL NOT NULL,
                ignore_region INTEGER NOT NULL,
                category_name TEXT NOT NULL,
                object_condition TEXT NOT NULL,
                quad_json TEXT NOT NULL,
                geometry_tier TEXT NOT NULL,
                fit_coverage REAL NOT NULL,
                fit_tightness REAL NOT NULL,
                seen_status TEXT NOT NULL
            )
            """
        )
        conn.execute(
            "INSERT INTO images VALUES (0, 1001, 'nu.jpg', 400, 400, 'nuimages', 'perspective', 0, 3)"
        )
        conn.execute(
            "INSERT INTO images VALUES (1, 1002, 'ws.jpg', 400, 400, 'woodscape_rgb_fisheye', 'fisheye', 0, 3)"
        )

        # nuimages raw positives: idx 0 = dup source_hbb (dropped), idx 1 = fitted_quad (kept), idx 2 = car fitted_quad (kept)
        nu_rows = [
            (1001, 50.0, 50.0, 150.0, 150.0, 0, "bicycle", "clean", _quad_from_box(50, 50, 150, 150), "source_hbb", 1.0, 1.0, "seen"),
            (1001, 51.0, 51.0, 149.0, 149.0, 0, "bicycle", "clean", _quad_from_box(51, 51, 149, 149), "fitted_quad", 1.0, 1.0, "seen"),
            (1001, 200.0, 200.0, 300.0, 300.0, 0, "car", "clean", _quad_from_box(200, 200, 300, 300), "fitted_quad", 1.0, 1.0, "seen"),
        ]
        # woodscape raw positives: idx 0 = noisy class (demoted to ignore), idx 1 = tiny <16px (demoted to ignore), idx 2 = valid car (kept)
        ws_rows = [
            (1002, 20.0, 20.0, 100.0, 100.0, 0, "other_wheeled_transport", "clean", _quad_from_box(20, 20, 100, 100), "fitted_quad", 1.0, 1.0, "seen"),
            (1002, 120.0, 20.0, 132.0, 50.0, 0, "person", "clean", _quad_from_box(120, 20, 132, 50), "fitted_quad", 1.0, 1.0, "seen"),
            (1002, 200.0, 200.0, 320.0, 320.0, 0, "car", "clean", _quad_from_box(200, 200, 320, 320), "fitted_quad", 1.0, 1.0, "seen"),
        ]
        conn.executemany(
            "INSERT INTO annotations VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)",
            nu_rows + ws_rows,
        )
    return db_path, img_root


def test_human_gt_datasets_and_semantic_eval_preserve_gt_indices(tmp_path: Path) -> None:
    db_path, img_root = _build_synthetic_hygiene_db(tmp_path)

    hbb_clean = HumanGTHBBDataset(
        cohort_db_path=db_path,
        gt_index_path=db_path,
        image_root=img_root,
        split="all",
        cohort_size=None,
        input_size=400,
        sanitize_annotations=True,
    )
    nu_sample = hbb_clean[0]
    assert nu_sample.image_id == 1001
    assert nu_sample.boxes.shape == (2, 4)
    assert nu_sample.category_names == ("bicycle", "car")
    assert nu_sample.gt_indices == (1, 2)  # Preserves raw idx 1 and 2 (raw idx 0 dropped)!

    ws_sample = hbb_clean[1]
    assert ws_sample.image_id == 1002
    assert ws_sample.boxes.shape == (1, 4)
    assert ws_sample.ignore_boxes.shape == (2, 4)
    assert ws_sample.category_names == ("car",)
    assert ws_sample.gt_indices == (2,)  # Preserves raw idx 2 (0 and 1 demoted)!

    quad_clean = HumanGTQuadDataset(
        cohort_db_path=db_path,
        gt_index_path=db_path,
        image_root=img_root,
        split="all",
        cohort_size=None,
        input_size=400,
        sanitize_annotations=True,
    )
    nu_q = quad_clean[0]
    assert nu_q.quads.shape == (2, 4, 2)
    assert nu_q.gt_indices == (1, 2)

    ws_q = quad_clean[1]
    assert ws_q.quads.shape == (1, 4, 2)
    assert ws_q.ignore_quads.shape == (2, 4, 2)
    assert ws_q.gt_indices == (2,)
    assert torch.all(torch.isfinite(ws_q.quads))

    # Raw baseline still works when sanitize_annotations=False
    hbb_raw = HumanGTHBBDataset(
        cohort_db_path=db_path,
        gt_index_path=db_path,
        image_root=img_root,
        split="all",
        cohort_size=None,
        input_size=400,
        sanitize_annotations=False,
    )
    assert hbb_raw[0].boxes.shape == (3, 4)
    assert hbb_raw[1].boxes.shape == (3, 4)

    # Semantic eval record loader also respects sanitize_annotations
    eval_clean = load_semantic_image_records(img_root, index_path=db_path, sanitize_annotations=True)
    assert len(eval_clean[0].positive_ground_truths) == 2
    assert len(eval_clean[1].positive_ground_truths) == 1
    assert sum(1 for g in eval_clean[1].ground_truths if g.is_ignore) == 2
