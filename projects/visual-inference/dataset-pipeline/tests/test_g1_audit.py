from __future__ import annotations

from pathlib import Path

from PIL import Image

from dataset_pipeline.g1_audit import (
    VOC20_SEEN,
    _draw_overlay,
    _open_world_views,
    _sample,
    hash_tree,
    recover_review_text,
)


def _image(source_category: str) -> dict:
    record = {
        "quad": [[1, 1], [9, 1], [9, 9], [1, 9]],
        "bbox": [1, 1, 8, 8],
        "geometry_tier": "source_quad",
        "fit_coverage": 1.0,
        "fit_tightness": 1.0,
        "state": "positive",
        "valid": True,
        "source_annotation_id": source_category,
        "source_category": source_category,
        "aliases": [],
    }
    return {
        "image_id": 1,
        "file_name": "images/train/one.jpg",
        "width": 10,
        "height": 10,
        "source_dataset": "coco_2017",
        "source_split": "train2017",
        "source_image_id": "one",
        "camera_type": "perspective",
        "background_supervision": True,
        "positive": [record],
        "ignore": [],
        "trusted_background": [],
    }


def test_open_world_train_moves_unseen_to_ignore_and_val_keeps_it(
    tmp_path: Path,
) -> None:
    base = {
        "schema_version": "quad-proposal-manifest.v1",
        "object_contract": "bounded_promptable_physical_instance",
    }
    manifests = {
        "train": {
            **base,
            "split": "train",
            "images": [_image("toaster"), _image("person")],
        },
        "val": {
            **base,
            "split": "val",
            "images": [_image("toaster"), _image("person")],
        },
    }
    report = _open_world_views(manifests, tmp_path)
    train = __import__("json").loads(
        (tmp_path / "open_world_coco_train.json").read_text()
    )
    val = __import__("json").loads((tmp_path / "open_world_coco_val.json").read_text())
    assert "person" in VOC20_SEEN and "toaster" not in VOC20_SEEN
    assert train["images"][0]["positive"] == []
    assert (
        train["images"][0]["ignore"][0]["ignore_reason"]
        == "withheld_open_world_category"
    )
    assert val["images"][0]["positive"][0]["seen_status"] == "unseen"
    assert report["pass"] is True


def test_stratified_sample_is_deterministic_and_covers_populated_groups() -> None:
    rows = []
    for source in ("a", "b"):
        for tier in ("source_quad", "hbb_fallback"):
            for index in range(3):
                rows.append(
                    {
                        "audit_id": f"{source}-{tier}-{index}",
                        "source_dataset": source,
                        "split": "train",
                        "state": "positive",
                        "geometry_tier": tier,
                        "size_bin": "small",
                        "aspect_bin": "regular",
                        "tightness_bin": "high",
                        "radial_bin": "center",
                    }
                )
    first = _sample(rows, 4, 42)
    second = _sample(rows, 4, 42)
    assert [row["audit_id"] for row in first] == [row["audit_id"] for row in second]
    assert {(row["source_dataset"], row["geometry_tier"]) for row in first} == {
        (source, tier)
        for source in ("a", "b")
        for tier in ("source_quad", "hbb_fallback")
    }


def test_hash_tree_tracks_relative_path_and_content(tmp_path: Path) -> None:
    (tmp_path / "a").mkdir()
    path = tmp_path / "a" / "record.json"
    path.write_text("{}\n", encoding="utf-8")
    first = hash_tree(tmp_path, {".json"})
    path.write_text('{"changed":true}\n', encoding="utf-8")
    second = hash_tree(tmp_path, {".json"})
    assert first["files"] == second["files"] == 1
    assert first["sha256"] != second["sha256"]


def test_recover_review_text_preserves_decisions_and_reports_unresolved(
    tmp_path: Path,
) -> None:
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    (bundle / "audit_instances.jsonl").write_text(
        "\n".join(
            [
                '{"audit_id":"aaaaaaaaaaaaaaaa","source_dataset":"one","geometry_tier":"source_quad"}',
                '{"audit_id":"bbbbbbbbbbbbbbbb","source_dataset":"two","geometry_tier":"fitted_quad"}',
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    copied = tmp_path / "review.txt"
    copied.write_text(
        """Reviewer
tester
aaaaaaaaaaaaaaaa
Decision
PASS
Issue
Notes
bbbbbbbbbbbbbbbb
Decision
UNCERTAIN
Issue
coverage
Notes
""",
        encoding="utf-8",
    )
    summary = recover_review_text(bundle, copied, tmp_path / "recovered")
    assert summary["decision_counts"] == {"PASS": 1, "UNCERTAIN": 1}
    assert summary["review_complete"] is False
    assert summary["recovered"] == 2


def test_overlay_edges_and_corner_markers_are_translucent(tmp_path: Path) -> None:
    source = tmp_path / "source.png"
    destination = tmp_path / "overlay.jpg"
    Image.new("RGB", (100, 100), "white").save(source)
    row = {
        "state": "positive",
        "quad": [[20, 20], [80, 20], [80, 80], [20, 80]],
    }
    _draw_overlay(source, destination, row, [row])
    with Image.open(destination) as rendered:
        edge = rendered.convert("RGB").getpixel((50, 20))
        corner = rendered.convert("RGB").getpixel((20, 20))
    assert edge[0] > 0 and edge[1] < 255
    assert corner[0] > 0 and corner[1] < 255
