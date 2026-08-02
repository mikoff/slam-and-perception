"""Write an evidence-based G0/G1 readiness report without waiving missing gates."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any

from student_detector.config import load_phase3_config


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _identities(path: Path) -> set[tuple[str, str]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    return {
        (str(image.get("source_dataset", "")), str(image.get("source_image_id", image.get("image_id", image.get("id")))))
        for image in data.get("images", [])
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/phase3.yaml"))
    parser.add_argument(
        "--reports-dir",
        type=Path,
        default=Path("../../data/visual-inference-datasets/reports"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/phase3/gate_readiness.json"),
    )
    parser.add_argument("--hbb-tests-passed", action="store_true")
    args = parser.parse_args()
    config = load_phase3_config(args.config)
    reports = args.reports_dir.resolve()
    proposal_validation = json.loads(
        (reports / "proposal_manifest_validation.json").read_text(encoding="utf-8")
    )
    validation_summary = json.loads(
        (reports / "validation_summary.json").read_text(encoding="utf-8")
    )
    link_report = json.loads((reports / "link_report.json").read_text(encoding="utf-8"))
    with (reports / "invalid_geometries.csv").open(newline="", encoding="utf-8") as stream:
        invalid_geometries = list(csv.DictReader(stream))

    train_manifest = config.data.quad_train_annotations or config.data.train_annotations
    val_manifest = config.data.quad_val_annotations or config.data.val_annotations
    overlap = sorted(_identities(train_manifest) & _identities(val_manifest))
    all_manifest_rows_valid = all(
        row["accepted_geometry"] == row["annotations"]
        and row["fit_coverage_failures"] == 0
        and row["schema_version"] == "quad-proposal-manifest.v1"
        for row in proposal_validation
    )
    localized_invalids = all(
        row.get("resolution") == "localized_ignore_hbb" for row in invalid_geometries
    )
    g0_checks = {
        "hbb_tests_pass": args.hbb_tests_passed,
        "hbb_full_production_report_exists": False,
        "hbb_category_disjoint_report_exists": False,
        "target_quality_ceiling_report_exists": False,
        "annotation_ceiling_report_exists": False,
        "numerical_selection_thresholds_frozen": False,
        "train_val_identity_overlap_zero": not overlap,
        "open_world_category_leakage_zero": False,
        "contract_and_protocol_fingerprinted": True,
    }
    g1_checks: dict[str, bool | None] = {
        "pipeline_validation_passes": validation_summary.get("valid") is True,
        "manifest_schema_and_geometry_pass": all_manifest_rows_valid,
        "image_links_valid": link_report.get("broken_links") == 0
        and link_report.get("incorrect_links") == 0,
        "fit_coverage_failures_zero": all(
            row["fit_coverage_failures"] == 0 for row in proposal_validation
        ),
        "localized_invalids_become_ignore": localized_invalids,
        "eligible_instance_representability_at_least_98_percent": None,
        "state_policy_audit_complete": None,
        "byte_deterministic_real_conversion_proven": None,
        "stratified_300_instance_visual_audit_passes": False,
    }
    revision = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=Path(__file__).resolve().parents[1],
        text=True,
        capture_output=True,
        check=True,
    ).stdout.strip()
    report: dict[str, Any] = {
        "G0": {
            "pass": all(g0_checks.values()),
            "checks": g0_checks,
            "missing": [key for key, value in g0_checks.items() if not value],
        },
        "G1": {
            "pass": all(value is True for value in g1_checks.values()),
            "checks": g1_checks,
            "missing": [key for key, value in g1_checks.items() if value is not True],
        },
        "evidence": {
            "proposal_manifest_validation": proposal_validation,
            "invalid_geometry_rows": len(invalid_geometries),
            "train_val_identity_overlap": overlap,
            "train_manifest": str(train_manifest),
            "train_manifest_sha256": _sha256(train_manifest),
            "val_manifest": str(val_manifest),
            "val_manifest_sha256": _sha256(val_manifest),
            "config": str(args.config.resolve()),
            "config_sha256": _sha256(args.config.resolve()),
            "software_revision": revision,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
