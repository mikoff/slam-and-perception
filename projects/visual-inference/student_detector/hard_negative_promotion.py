"""Build a conservative supplemental manifest from reviewed mining candidates."""

from __future__ import annotations

from collections import Counter, defaultdict
from typing import Any


def _source_quad(box: list[float]) -> list[list[float]]:
    if len(box) != 4:
        raise ValueError("source box must contain four coordinates")
    x1, y1, x2, y2 = (float(value) for value in box)
    if not (x2 > x1 and y2 > y1):
        raise ValueError("source box must have positive area")
    return [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]


def build_default_supplement(
    candidate_manifest: dict[str, Any],
    policy: dict[str, Any],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Promote only candidates backed by certified trusted-background geometry."""
    if candidate_manifest.get("schema_version") != (
        "visual-inference-hard-negative-candidates.v1"
    ):
        raise ValueError("unsupported candidate manifest")
    if candidate_manifest.get("status") != "candidate_review_required":
        raise ValueError("candidate manifest is not a completed review candidate set")
    if policy.get("schema_version") != ("visual-inference-hard-negative-promotion.v1"):
        raise ValueError("unsupported promotion policy")
    approved_classes = set(policy["promote_classifications"])
    requirements = policy["requirements"]
    grouped: dict[tuple[str, int], list[dict[str, Any]]] = defaultdict(list)
    decisions = []
    seen_ids: set[str] = set()
    for candidate in candidate_manifest["candidates"]:
        candidate_id = str(candidate["candidate_id"])
        if candidate_id in seen_ids:
            raise ValueError(f"duplicate candidate ID {candidate_id}")
        seen_ids.add(candidate_id)
        promoted = candidate["classification"] in approved_classes
        if promoted:
            if float(candidate["best_object_iou"]) >= float(
                requirements["maximum_object_iou"]
            ):
                raise ValueError(
                    f"promoted candidate {candidate_id} overlaps an object"
                )
            if float(candidate["ignore_fraction"]) >= float(
                requirements["maximum_ignore_fraction"]
            ):
                raise ValueError(f"promoted candidate {candidate_id} overlaps ignore")
            if float(candidate["trusted_background_fraction"]) < float(
                requirements["minimum_trusted_background_fraction"]
            ):
                raise ValueError(
                    f"promoted candidate {candidate_id} lacks trusted-background coverage"
                )
            focus_region = {
                "candidate_id": candidate_id,
                "training_action": policy["training_action"],
                "effective_supervision": policy["effective_supervision"],
                "selector_geometry_type": "hbb",
                "selector_bbox": [float(value) for value in candidate["source_box"]],
                "selector_quad": _source_quad(candidate["source_box"]),
                "score": float(candidate["score"]),
                "trusted_background_fraction": float(
                    candidate["trusted_background_fraction"]
                ),
                "source": "phase3_hard_negative_mining_v1",
            }
            key = (str(candidate["source_dataset"]), int(candidate["image_id"]))
            grouped[key].append(focus_region)
        decisions.append(
            {
                "candidate_id": candidate_id,
                "decision": "approve_negative" if promoted else "ignore",
                "classification": str(candidate["classification"]),
                "source_dataset": str(candidate["source_dataset"]),
                "domain": str(candidate["domain"]),
                "image_id": int(candidate["image_id"]),
                "score": float(candidate["score"]),
            }
        )
    images = []
    for (source, image_id), regions in sorted(grouped.items()):
        source_candidate = next(
            candidate
            for candidate in candidate_manifest["candidates"]
            if candidate["source_dataset"] == source
            and int(candidate["image_id"]) == image_id
        )
        images.append(
            {
                "source_dataset": source,
                "domain": source_candidate["domain"],
                "image_id": image_id,
                "file_name": source_candidate["file_name"],
                "focus_regions": sorted(
                    regions, key=lambda region: region["candidate_id"]
                ),
            }
        )
    promoted = sum(len(image["focus_regions"]) for image in images)
    supplement = {
        "schema_version": "visual-inference-hard-negative-supplement.v1",
        "status": "approved_supplement_not_applied",
        "strategy": policy["strategy"],
        "rejected_disposition": policy["rejected_disposition"],
        "training_action": policy["training_action"],
        "effective_supervision": policy["effective_supervision"],
        "counts": {
            "candidate_decisions": len(decisions),
            "promoted_regions": promoted,
            "rejected_regions": len(decisions) - promoted,
            "images_with_promotions": len(images),
            "promoted_by_domain": dict(
                sorted(
                    Counter(
                        decision["domain"]
                        for decision in decisions
                        if decision["decision"] == "approve_negative"
                    ).items()
                )
            ),
        },
        "requirements": requirements,
        "source_contract": {
            "train_manifest_sha256": candidate_manifest["inputs"][
                "train_manifest_sha256"
            ],
            "mining_pool_identity_sha256": candidate_manifest["pool"][
                "identity_sha256"
            ],
            "mining_pool_images": int(candidate_manifest["pool"]["images"]),
            "validation_excluded": bool(
                candidate_manifest["pool"]["validation_excluded"]
            ),
        },
        "images": images,
    }
    return supplement, decisions


__all__ = ["build_default_supplement"]
