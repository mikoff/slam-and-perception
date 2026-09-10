from __future__ import annotations

from copy import deepcopy

import pytest

from student_detector.hard_negative_promotion import build_default_supplement


def _policy() -> dict[str, object]:
    return {
        "schema_version": "visual-inference-hard-negative-promotion.v1",
        "strategy": "certified_trusted_background_only",
        "promote_classifications": ["certified_trusted_background"],
        "requirements": {
            "maximum_object_iou": 0.1,
            "maximum_ignore_fraction": 0.5,
            "minimum_trusted_background_fraction": 0.5,
        },
        "rejected_disposition": "no_training_change",
        "training_action": "focus_existing_trusted_background",
        "effective_supervision": "existing trusted-background intersection",
    }


def _manifest() -> dict[str, object]:
    common = {
        "source_dataset": "source",
        "domain": "general",
        "image_id": 7,
        "file_name": "image.jpg",
        "score": 0.8,
        "best_object_iou": 0.0,
        "ignore_fraction": 0.0,
        "trusted_background_fraction": 0.8,
        "semantic_state": "trusted_background",
        "source_box": [1.0, 2.0, 5.0, 7.0],
    }
    return {
        "schema_version": "visual-inference-hard-negative-candidates.v1",
        "status": "candidate_review_required",
        "inputs": {"train_manifest_sha256": "train-hash"},
        "pool": {
            "identity_sha256": "pool-hash",
            "images": 2,
            "validation_excluded": True,
        },
        "candidates": [
            common
            | {
                "candidate_id": "approved",
                "classification": "certified_trusted_background",
            },
            common
            | {
                "candidate_id": "ignored",
                "classification": "unverified_background",
            },
        ],
    }


def test_default_policy_promotes_only_certified_background() -> None:
    supplement, decisions = build_default_supplement(_manifest(), _policy())

    assert supplement["counts"] == {
        "candidate_decisions": 2,
        "promoted_regions": 1,
        "rejected_regions": 1,
        "images_with_promotions": 1,
        "promoted_by_domain": {"general": 1},
    }
    focus = supplement["images"][0]["focus_regions"][0]
    assert focus["candidate_id"] == "approved"
    assert focus["training_action"] == "focus_existing_trusted_background"
    assert supplement["source_contract"]["train_manifest_sha256"] == "train-hash"
    assert [decision["decision"] for decision in decisions] == [
        "approve_negative",
        "ignore",
    ]


def test_tampered_certified_candidate_is_rejected() -> None:
    manifest = deepcopy(_manifest())
    manifest["candidates"][0]["trusted_background_fraction"] = 0.49  # type: ignore[index]

    with pytest.raises(ValueError, match="lacks trusted-background coverage"):
        build_default_supplement(manifest, _policy())
