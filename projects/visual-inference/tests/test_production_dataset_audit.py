from __future__ import annotations

import importlib.util
import json
from pathlib import Path

from PIL import Image


SCRIPT = Path(__file__).parents[1] / "scripts/audit_production_dataset.py"
SPEC = importlib.util.spec_from_file_location("production_audit", SCRIPT)
assert SPEC and SPEC.loader
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def test_manifest_scalar_reads_field_after_images(tmp_path: Path) -> None:
    manifest = tmp_path / "manifest.json"
    manifest.write_text(
        '{"images":[{"image_id":1}],"schema_version":"proposal-manifest.v2"}'
    )
    assert AUDIT.manifest_scalar(manifest, "schema_version") == "proposal-manifest.v2"


def test_full_manifest_scan_finds_overlap_and_builds_overlay(tmp_path: Path) -> None:
    root = tmp_path / "output"
    image_path = root / "images/train/example.jpg"
    image_path.parent.mkdir(parents=True)
    Image.new("RGB", (20, 20), "white").save(image_path)
    positive = {
        "quad": [[2, 2], [10, 2], [10, 10], [2, 10]],
        "canonical_category": "car",
        "source_annotation_id": "positive-a",
        "supervision_state": "positive",
    }
    trusted = {
        **positive,
        "source_annotation_id": "trusted-a",
        "supervision_state": "trusted_background",
    }
    manifest = tmp_path / "manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "images": [
                    {
                        "image_id": 1,
                        "file_name": "images/train/example.jpg",
                        "width": 20,
                        "height": 20,
                        "source_dataset": "demo",
                        "source_image_id": "image-1",
                        "source_sequence_id": "sequence-1",
                        "positive": [positive],
                        "ignore": [],
                        "trusted_background": [trusted],
                    }
                ],
                "schema_version": "proposal-manifest.v2",
                "split": "train",
            }
        )
    )

    result = AUDIT._scan_manifest(manifest, root, "train", tmp_path / "assets")

    assert result["images"] == 1
    assert result["regions_by_state"] == {"positive": 1, "trusted_background": 1}
    assert result["raw_state_intersections"]["positive/trusted_background_images"] == 1
    assert result["effective_supervision_conflicts"] == []
    assert len(result["overlay_samples"]) == 2
    assert len(list((tmp_path / "assets").glob("*.jpg"))) == 2


def test_render_html_reports_successful_binding(tmp_path: Path) -> None:
    path = tmp_path / "index.html"
    AUDIT._render_html(
        path,
        {
            "binding": {"pass": True, "hard_failures": []},
            "hard_failures": [],
            "warnings": [],
            "status": "binding_pass",
            "inventory": {},
        },
    )

    rendered = path.read_text()
    assert "Binding gate: PASSED" in rendered
    assert "binding-only preflight" in rendered
    assert "Binding gate: FAILED" not in rendered


def test_scan_findings_names_raw_overlap_and_duplicate_warnings() -> None:
    scans = {
        "train": {
            "unreadable_images": [],
            "invalid_polygons": [],
            "effective_supervision_conflicts": [],
            "source_identity_collisions": [],
            "duplicate_images_within_split": [{"sha256": "same"}],
            "raw_state_intersections": {
                "positive/ignore_images": 3,
                "positive/ignore_px2": 12.5,
            },
        }
    }

    failures, warnings = AUDIT._scan_findings(
        {"hard_failures": []},
        scans,
        {
            "source_identity_collisions": [],
            "sequence_leakage": [],
            "duplicate_image_sha256": [],
        },
    )

    assert failures == []
    assert warnings == [
        "train: 3 images have raw positive/ignore intersection "
        "(12.500000 px^2 union area)",
        "train: 1 repeated image-content records within split",
    ]


def test_full_manifest_scan_rejects_cross_state_source_identity(tmp_path: Path) -> None:
    root = tmp_path / "output"
    image_path = root / "images/train/example.jpg"
    image_path.parent.mkdir(parents=True)
    Image.new("RGB", (20, 20), "white").save(image_path)
    record = {
        "quad": [[2, 2], [10, 2], [10, 10], [2, 10]],
        "canonical_category": "car",
        "source_annotation_id": "same-source-object",
        "source_annotation_identity_kind": "official",
    }
    manifest = tmp_path / "manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "images": [
                    {
                        "file_name": "images/train/example.jpg",
                        "width": 20,
                        "height": 20,
                        "source_dataset": "demo",
                        "source_image_id": "image-1",
                        "positive": [{**record, "supervision_state": "positive"}],
                        "ignore": [{**record, "supervision_state": "ignore"}],
                        "trusted_background": [],
                    }
                ]
            }
        )
    )

    result = AUDIT._scan_manifest(manifest, root, "train", tmp_path / "assets")

    assert len(result["effective_supervision_conflicts"]) == 1
    assert (
        "source annotation also appears as positive"
        in result["effective_supervision_conflicts"][0]
    )
    failures, _ = AUDIT._scan_findings(
        {"hard_failures": []},
        {"train": result},
        {
            "source_identity_collisions": [],
            "sequence_leakage": [],
            "duplicate_image_sha256": [],
        },
    )
    assert failures == ["train: 1 effective supervision conflicts"]
