#!/usr/bin/env python3
"""Audit the concrete two-stage detector datasets before a training run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from student_detector.data_isolation import (
    assert_same_image_records,
    audit_training_isolation,
)
from student_detector.human_gt_data import (
    HumanGTHBBDataset,
    HumanGTQuadDataset,
    load_cohort_image_records,
)
from student_detector.sam_student_data import (
    DEFAULT_IMAGE_ROOT,
    SAMStudentDataset,
    SAMStudentHBBDataset,
    split_cohort_indices,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify pipeline-wide detector train/validation isolation."
    )
    parser.add_argument("--cohort-db", type=Path, required=True)
    parser.add_argument("--gt-index", type=Path, required=True)
    parser.add_argument("--cohort-size", type=int, required=True)
    parser.add_argument("--val-pseudo-labels", type=Path, required=True)
    parser.add_argument("--image-root", type=Path, default=DEFAULT_IMAGE_ROOT)
    parser.add_argument("--pipeline-val-ratio", type=float, default=0.10)
    parser.add_argument("--split-seed", type=int, default=42)
    parser.add_argument("--dashboard-manifest", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--skip-hashes",
        action="store_true",
        help="Skip exact-content checks; intended only for fast unit tests.",
    )
    return parser.parse_args()


def _records(dataset: Any) -> list[dict[str, Any]]:
    return list(dataset.images)


def main() -> None:
    args = parse_args()
    if not 0 < args.pipeline_val_ratio < 1:
        raise ValueError("--pipeline-val-ratio must be between 0 and 1")
    cohort_size = args.cohort_size if args.cohort_size > 0 else None
    common = {
        "image_root": args.image_root,
        "cohort_size": cohort_size,
        "gt_index_path": args.gt_index,
        "input_size": 32,
    }
    cohort = load_cohort_image_records(
        args.cohort_db, args.gt_index, cohort_size=cohort_size
    )
    train_indices, val_indices = split_cohort_indices(
        cohort,
        val_ratio=args.pipeline_val_ratio,
        seed=args.split_seed,
    )
    expected_train = [cohort[index] for index in train_indices]
    expected_val = [cohort[index] for index in val_indices]

    sam_hbb_train = SAMStudentHBBDataset(
        pseudo_labels_path=args.cohort_db,
        split="train",
        val_ratio=args.pipeline_val_ratio,
        split_seed=args.split_seed,
        **common,
    )
    sam_quad_train = SAMStudentDataset(
        pseudo_labels_path=args.cohort_db,
        split="train",
        val_ratio=args.pipeline_val_ratio,
        split_seed=args.split_seed,
        **common,
    )
    sam_hbb_val = SAMStudentHBBDataset(
        pseudo_labels_path=args.cohort_db,
        split="val",
        val_ratio=args.pipeline_val_ratio,
        split_seed=args.split_seed,
        **common,
    )
    sam_quad_val = SAMStudentDataset(
        pseudo_labels_path=args.cohort_db,
        split="val",
        val_ratio=args.pipeline_val_ratio,
        split_seed=args.split_seed,
        **common,
    )
    human_hbb_train = HumanGTHBBDataset(
        cohort_db_path=args.cohort_db,
        split="train",
        val_ratio=args.pipeline_val_ratio,
        split_seed=args.split_seed,
        **common,
    )
    human_quad_train = HumanGTQuadDataset(
        cohort_db_path=args.cohort_db,
        split="train",
        val_ratio=args.pipeline_val_ratio,
        split_seed=args.split_seed,
        **common,
    )
    human_hbb_val = HumanGTHBBDataset(
        cohort_db_path=args.cohort_db,
        split="val",
        val_ratio=args.pipeline_val_ratio,
        split_seed=args.split_seed,
        **common,
    )
    human_quad_val = HumanGTQuadDataset(
        cohort_db_path=args.cohort_db,
        split="val",
        val_ratio=args.pipeline_val_ratio,
        split_seed=args.split_seed,
        **common,
    )

    for name, records in {
        "Stage-1 HBB train": _records(sam_hbb_train),
        "Stage-1 quad train": _records(sam_quad_train),
        "Stage-2 HBB train": _records(human_hbb_train),
        "Stage-2 quad train": _records(human_quad_train),
    }.items():
        assert_same_image_records(
            expected_train,
            records,
            image_root=args.image_root,
            expected_name="canonical train split",
            actual_name=name,
        )
    for name, records in {
        "Stage-1 HBB validation": _records(sam_hbb_val),
        "Stage-1 quad validation": _records(sam_quad_val),
        "Stage-2 HBB validation": _records(human_hbb_val),
        "Stage-2 quad validation": _records(human_quad_val),
    }.items():
        assert_same_image_records(
            expected_val,
            records,
            image_root=args.image_root,
            expected_name="canonical validation split",
            actual_name=name,
        )

    external_hbb = SAMStudentHBBDataset(
        pseudo_labels_path=args.val_pseudo_labels,
        image_root=args.image_root,
        split="all",
        input_size=32,
    )
    external_quad = SAMStudentDataset(
        pseudo_labels_path=args.val_pseudo_labels,
        image_root=args.image_root,
        split="all",
        input_size=32,
    )
    assert_same_image_records(
        _records(external_hbb),
        _records(external_quad),
        image_root=args.image_root,
        expected_name="external HBB validation",
        actual_name="external quad validation",
    )

    validation_sets: dict[str, list[dict[str, Any]]] = {
        "reserved_cohort": expected_val,
        "external": _records(external_hbb),
    }
    if args.dashboard_manifest is not None and args.dashboard_manifest.exists():
        dashboard = json.loads(args.dashboard_manifest.read_text())
        validation_sets["dashboard"] = dashboard["images"]

    report = audit_training_isolation(
        expected_train,
        validation_sets,
        image_root=args.image_root,
        verify_hashes=not args.skip_hashes,
        identity_validation_sets={"reserved_cohort"},
    )
    report.update(
        {
            "cohort_images": len(cohort),
            "requested_cohort_size": args.cohort_size,
            "pipeline_val_ratio": args.pipeline_val_ratio,
            "split_seed": args.split_seed,
            "stage1_hbb_train": len(sam_hbb_train),
            "stage1_quad_train": len(sam_quad_train),
            "stage2_hbb_train": len(human_hbb_train),
            "stage2_quad_train": len(human_quad_train),
            "stage1_hbb_reserved_validation": len(sam_hbb_val),
            "stage1_quad_reserved_validation": len(sam_quad_val),
            "reserved_validation": len(expected_val),
            "external_validation": len(external_hbb),
        }
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
