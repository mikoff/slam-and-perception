from __future__ import annotations

import csv
import json
from pathlib import Path

from PIL import Image

from dataset_pipeline.contract_audit import (
    _write_audit_html,
    build_contract_inventory,
    generate_contract_audit_bundle,
    recover_contract_review_text,
)


def _bdd_project(root: Path) -> None:
    root.mkdir(parents=True)
    (root / "meta.json").write_text(
        json.dumps(
            {
                "classes": [
                    {"title": "car"},
                    {"title": "drivable area"},
                    {"title": "lane"},
                ]
            }
        ),
        encoding="utf-8",
    )
    image_dir = root / "train" / "img"
    annotation_dir = root / "train" / "ann"
    image_dir.mkdir(parents=True)
    annotation_dir.mkdir()
    Image.new("RGB", (40, 30), "white").save(image_dir / "one.jpg")
    objects = []
    for object_id, category, y, geometry in (
        (1, "car", 2, "rectangle"),
        (2, "drivable area", 12, "rectangle"),
        (3, "lane", 22, "line"),
    ):
        objects.append(
            {
                "id": object_id,
                "classTitle": category,
                "geometryType": geometry,
                "points": {"exterior": [[2, y], [35, y + 5]], "interior": []},
            }
        )
    (annotation_dir / "one.jpg.json").write_text(
        json.dumps(
            {
                "size": {"width": 40, "height": 30},
                "objects": objects,
            }
        ),
        encoding="utf-8",
    )


def test_contract_inventory_resolves_defaults_and_explicit_states(
    tmp_path: Path, config_factory, taxonomy
) -> None:
    config = config_factory(
        {"bdd100k_images_100k": {"archive": tmp_path / "unused.tar"}}
    )
    _bdd_project(config.datasets["bdd100k_images_100k"].extracted_dir)

    inventory = build_contract_inventory(config, taxonomy)
    by_category = {row["source_category"]: row for row in inventory}

    assert by_category["car"]["proposed_state"] == "positive"
    assert by_category["car"]["audit_required"] is False
    assert by_category["drivable area"]["current_state"] == "trusted_negative"
    assert by_category["drivable area"]["proposed_state"] == "trusted_negative"
    assert by_category["lane"]["proposed_state"] == "trusted_negative"
    assert by_category["lane"]["approved_state"] == "drop"


def test_approved_contract_exactly_covers_audited_overrides(taxonomy) -> None:
    contract = taxonomy.data["proposal_object_contract"]
    proposed = {
        (dataset, category)
        for dataset, categories in contract["category_overrides"].items()
        for category, policy in categories.items()
        if policy["audit_required"]
    }
    approved = {
        (dataset, category)
        for dataset, categories in contract["approved_category_states"].items()
        for category in categories
    }

    assert contract["status"] == "approved"
    assert proposed == approved
    assert len(approved) == 30
    review_path = (
        Path(__file__).parents[1] / "configs/proposal_object_contract_review.csv"
    )
    with review_path.open(newline="", encoding="utf-8") as stream:
        reviewed = {
            (row["source_dataset"], row["source_category"]): row["approved_state"]
            for row in csv.DictReader(stream)
        }
    configured = {
        (dataset, category): policy["state"]
        for dataset, categories in contract["approved_category_states"].items()
        for category, policy in categories.items()
    }
    assert reviewed == configured


def test_contract_bundle_reports_source_scarcity_and_writes_overlays(
    tmp_path: Path, config_factory, taxonomy
) -> None:
    config = config_factory(
        {"bdd100k_images_100k": {"archive": tmp_path / "unused.tar"}}
    )
    _bdd_project(config.datasets["bdd100k_images_100k"].extracted_dir)
    destination = tmp_path / "bundle"

    report = generate_contract_audit_bundle(
        config, taxonomy, count=2, destination=destination
    )

    assert report["all_source_categories_resolved"] is True
    assert report["audited_category_pairs"] == 2
    assert report["rendered_examples"] == 2
    assert report["strict_sample_count_met"] is False
    assert len(list((destination / "assets").glob("*.jpg"))) == 2
    assert (destination / "index.html").is_file()
    assert (destination / "bundle_manifest.json").is_file()


def test_contract_html_keeps_zero_instance_policy_visible(tmp_path: Path) -> None:
    policy = {
        "source_dataset": "woodscape_rgb_fisheye",
        "source_category": "structure",
        "normalized_source_category": "structure",
        "normalized_category": "building_or_wall",
        "current_state": "ignore",
        "proposed_state": "trusted_negative",
        "rationale": "Scene structure.",
        "audit_required": True,
    }
    destination = tmp_path / "index.html"

    _write_audit_html(
        destination,
        {"definition": "Physical instances only."},
        [policy],
        [],
        {},
        100,
    )

    document = destination.read_text(encoding="utf-8")
    assert "woodscape_rgb_fisheye · structure" in document
    assert "only 0 examples" in document


def test_review_text_recovery_infers_accept_state_and_reports_missing(
    tmp_path: Path,
) -> None:
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    inventory = [
        {
            "source_dataset": "woodscape_rgb_fisheye",
            "source_category": "transparent",
            "normalized_source_category": "transparent",
            "proposed_state": "ignore",
            "audit_required": True,
        },
        {
            "source_dataset": "woodscape_rgb_fisheye",
            "source_category": "structure",
            "normalized_source_category": "structure",
            "proposed_state": "trusted_negative",
            "audit_required": True,
        },
    ]
    (bundle / "category_inventory.json").write_text(
        json.dumps(inventory), encoding="utf-8"
    )
    review = tmp_path / "review.txt"
    review.write_text(
        """Candidate proposal-object contract · Reviewer
mikoff
woodscape_rgb_fisheye · transparent
Decision
ACCEPT
Approved state
Exception rule / notes
Show 100 source overlays
""",
        encoding="utf-8",
    )

    summary = recover_contract_review_text(bundle, review, tmp_path / "recovered")

    assert summary["recovered"] == 1
    assert summary["missing"] == ["woodscape_rgb_fisheye:structure"]
    assert summary["approved_state_counts"] == {"ignore": 1}
    assert summary["errors"] == []
    assert summary["review_complete"] is False
