from __future__ import annotations

import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts/diagnose_production_audit.py"
SPEC = importlib.util.spec_from_file_location("production_audit_diagnostics", SCRIPT)
assert SPEC and SPEC.loader
DIAGNOSTICS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DIAGNOSTICS)


def _record(quad: list[list[float]], category: str) -> dict[str, object]:
    return {"quad": quad, "canonical_category": category}


def test_overlap_records_rejects_touching_edges_and_measures_area() -> None:
    left = DIAGNOSTICS._prepare([_record([[0, 0], [4, 0], [4, 4], [0, 4]], "car")])
    right = DIAGNOSTICS._prepare(
        [
            _record([[4, 0], [8, 0], [8, 4], [4, 4]], "road"),
            _record([[2, 2], [6, 2], [6, 6], [2, 6]], "group"),
        ]
    )

    overlaps = list(DIAGNOSTICS._overlap_records(left, right, DIAGNOSTICS._grid(right)))

    assert len(overlaps) == 1
    assert overlaps[0][1]["category"] == "group"
    assert overlaps[0][2] == 4.0


def test_classify_duplicates_groups_repeated_first_record() -> None:
    audit = {
        "full_scan": {
            "train": {
                "duplicate_images_within_split": [
                    {"sha256": "x", "first": "/d/coco__a", "second": "/d/coco__b"},
                    {"sha256": "x", "first": "/d/coco__a", "second": "/d/coco__c"},
                ]
            }
        }
    }

    result = DIAGNOSTICS.classify_duplicates(audit)["train"]

    assert result["reported_duplicate_links"] == 2
    assert result["unique_content_hash_groups"] == 1
    assert result["unique_redundant_records"] == 2
    assert result["groups_by_source"] == {"coco": 1}
