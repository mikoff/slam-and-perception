"""Measure HBB loss-family gradients on fixed stratified calibration cohorts."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from dataclasses import replace
import json
from pathlib import Path
import sqlite3
import time

import torch

from student_detector.assigner import ATSSAssigner
from student_detector.config import Phase3Config, load_phase3_config
from student_detector.data import (
    ImageRecord,
    IndexedCocoProposalDataset,
    ProposalSample,
    select_source_mixture_indices,
)
from student_detector.gradient_diagnostics import (
    measure_gradient_influence,
    summarize_gradient_measurements,
)
from student_detector.losses import ProposalLoss
from student_detector.model import StudentDetector
from student_detector.provenance import sha256_file
from student_detector.targets import TargetBuilder


SIZE_BANDS = {
    "edge_small": (100.0, 256.0),
    "small": (256.0, 1_024.0),
    "medium": (1_024.0, 9_216.0),
    "large": (9_216.0, float("inf")),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument(
        "--checkpoint",
        action="append",
        required=True,
        metavar="LABEL=PATH",
        help="Repeat for warm-up and later checkpoints",
    )
    parser.add_argument("--state", choices=("auto", "model", "ema_model"), default="auto")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--batches-per-slice", type=int, default=4)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def _checkpoint_arguments(values: Sequence[str]) -> list[tuple[str, Path]]:
    result = []
    labels: set[str] = set()
    for value in values:
        label, separator, path = value.partition("=")
        if not separator or not label or not path:
            raise ValueError("checkpoints must use LABEL=PATH")
        if label in labels:
            raise ValueError(f"duplicate checkpoint label: {label}")
        labels.add(label)
        result.append((label, Path(path).expanduser().resolve()))
    return result


def _normalized_weights(
    config: Phase3Config, records: Sequence[ImageRecord]
) -> dict[str, float]:
    available = {record.source_dataset for record in records}
    weights = {
        source: weight
        for source, weight in config.data.source_weights.items()
        if source in available
    }
    total = sum(weights.values())
    if not weights or total <= 0:
        raise RuntimeError("calibration cohort has no configured source")
    return {source: weight / total for source, weight in weights.items()}


def _select_from_eligible(
    dataset: IndexedCocoProposalDataset,
    eligible: set[int],
    config: Phase3Config,
    count: int,
) -> list[int]:
    indexed_records = [
        (index, record)
        for index, record in enumerate(dataset.records)
        if record.row_index in eligible
    ]
    records = [record for _, record in indexed_records]
    selected = select_source_mixture_indices(
        records, _normalized_weights(config, records), count
    )
    return [indexed_records[index][0] for index in selected]


def _size_band_rows(index_path: Path, input_size: int) -> dict[str, set[int]]:
    """Return images containing a positive GT in each validation area band."""
    finite_bands = tuple(SIZE_BANDS.items())
    flag_sql = ", ".join(
        (
            f"MAX(CASE WHEN area >= {minimum} AND area < {maximum} "
            f"THEN 1 ELSE 0 END) AS {name}"
            if maximum != float("inf")
            else f"MAX(CASE WHEN area >= {minimum} THEN 1 ELSE 0 END) AS {name}"
        )
        for name, (minimum, maximum) in finite_bands
    )
    query = f"""
        WITH positive_areas AS (
            SELECT i.row_index,
                   (a.x2 - a.x1) * (a.y2 - a.y1) *
                   MIN({float(input_size)} / i.width,
                       {float(input_size)} / i.height) *
                   MIN({float(input_size)} / i.width,
                       {float(input_size)} / i.height) AS area
            FROM annotations AS a
            JOIN images AS i USING(image_id)
            WHERE a.ignore_region = 0
        )
        SELECT row_index, {flag_sql}
        FROM positive_areas
        GROUP BY row_index
        ORDER BY row_index
    """
    result = {name: set() for name in SIZE_BANDS}
    with sqlite3.connect(
        f"file:{index_path.resolve()}?mode=ro&immutable=1", uri=True
    ) as connection:
        for row in connection.execute(query):
            for offset, name in enumerate(SIZE_BANDS, start=1):
                if row[offset]:
                    result[name].add(int(row[0]))
    return result


def _cohort_indices(
    dataset: IndexedCocoProposalDataset,
    config: Phase3Config,
    count: int,
) -> dict[str, list[int]]:
    cohorts = {
        "overall": select_source_mixture_indices(
            dataset.records, config.data.source_weights, count
        )
    }
    for domain in config.data.domain_weights:
        eligible = {
            record.row_index
            for record in dataset.records
            if config.data.source_domains.get(record.source_dataset) == domain
        }
        cohorts[f"domain/{domain}"] = _select_from_eligible(
            dataset, eligible, config, count
        )
    for band, eligible in _size_band_rows(
        dataset.index_path, config.data.input_size
    ).items():
        cohorts[f"size/{band}"] = _select_from_eligible(
            dataset, eligible, config, count
        )
    return cohorts


def _criterion(config: Phase3Config) -> ProposalLoss:
    return ProposalLoss(
        strides=config.assignment.strides,
        objectness_weight=config.loss.objectness_weight,
        box_weight=config.loss.box_weight,
        ltrb_weight=config.loss.ltrb_weight,
        centerness_weight=config.loss.centerness_weight,
        objectness_loss=config.loss.objectness_loss,
        quality_focal_beta=config.loss.quality_focal_beta,
        box_loss=config.loss.box_loss,
        box_weighting=config.loss.box_weighting,
        focal_alpha=config.loss.focal_alpha,
        focal_gamma=config.loss.focal_gamma,
    )


def _target_builder(config: Phase3Config) -> TargetBuilder:
    return TargetBuilder(
        ATSSAssigner(
            strides=config.assignment.strides,
            prior_sizes=config.assignment.prior_sizes,
            top_k=config.assignment.top_k,
            center_radius=config.assignment.center_radius,
        ),
        background_loss_weights=config.data.background_loss_weights,
    )


def _load_state(
    model: StudentDetector, checkpoint: dict[str, object], requested: str
) -> str:
    selected = requested
    if selected == "auto":
        selected = "ema_model" if "ema_model" in checkpoint else "model"
    state = checkpoint.get(selected)
    if not isinstance(state, dict):
        raise KeyError(f"checkpoint does not contain state {selected!r}")
    model.load_state_dict(state, strict=True)
    return selected


def _samples(
    dataset: IndexedCocoProposalDataset, indices: Sequence[int]
) -> list[ProposalSample]:
    return [dataset[index] for index in indices]


def _size_filtered_samples(
    samples: Sequence[ProposalSample], band: str
) -> list[ProposalSample]:
    minimum, maximum = SIZE_BANDS[band]
    result = []
    for sample in samples:
        sizes = sample.boxes[:, 2:] - sample.boxes[:, :2]
        areas = sizes[:, 0] * sizes[:, 1]
        selected = (areas >= minimum) & (areas < maximum)
        excluded = sample.boxes[~selected]
        ignore_boxes = torch.cat((sample.ignore_boxes, excluded), dim=0)
        result.append(
            replace(
                sample,
                boxes=sample.boxes[selected],
                ignore_boxes=ignore_boxes,
                trusted_background_boxes=sample.boxes.new_empty((0, 4)),
            )
        )
    return result


def _measure_batch(
    model: StudentDetector,
    samples: Sequence[ProposalSample],
    target_builder: TargetBuilder,
    criterion: ProposalLoss,
    device: torch.device,
) -> dict[str, float]:
    images = torch.stack([sample.image for sample in samples]).to(device)
    backbone_features = tuple(model.backbone(images))  # type: ignore[arg-type]
    pyramid_features = tuple(model.fpn(backbone_features))  # type: ignore[arg-type]
    output = model.head(pyramid_features)
    shapes = tuple(
        (tensor.shape[-2], tensor.shape[-1]) for tensor in output.objectness
    )
    targets = target_builder(samples, shapes, device=device)
    losses = criterion(output, targets)
    return measure_gradient_influence(
        losses.family_terms,
        {
            "fpn_shared": pyramid_features,
            "backbone_c5": (backbone_features[-1],),
        },
    )


def main() -> None:
    args = parse_args()
    if args.batch_size < 1 or args.batches_per_slice < 1:
        raise ValueError("batch size and batches per slice must be positive")
    checkpoints = _checkpoint_arguments(args.checkpoint)
    config = load_phase3_config(args.config)
    device = torch.device(
        "cuda"
        if args.device == "auto" and torch.cuda.is_available()
        else "cpu"
        if args.device == "auto"
        else args.device
    )
    dataset = IndexedCocoProposalDataset(
        config.data.train_annotations,
        config.data.image_root,
        config.data.index_dir / "quad_train.sqlite",
        config.data,
        config.augmentation,
        training=False,
        seed=config.schedule.seed,
    )
    cohort_size = args.batch_size * args.batches_per_slice
    cohorts = _cohort_indices(dataset, config, cohort_size)
    report: dict[str, object] = {
        "schema_version": "proposal-gradient-influence.v1",
        "config": str(args.config.resolve()),
        "config_sha256": sha256_file(args.config.resolve()),
        "device": str(device),
        "precision": "float32",
        "model_mode": "eval",
        "size_slice_policy": (
            "retain only positives in the named transformed-area band; move "
            "other positives to ignore and disable background quality terms"
        ),
        "batch_size": args.batch_size,
        "batches_per_slice": args.batches_per_slice,
        "cohorts": {
            name: [dataset.records[index].image_id for index in indices]
            for name, indices in cohorts.items()
        },
        "checkpoints": {},
    }
    checkpoint_reports = report["checkpoints"]
    assert isinstance(checkpoint_reports, dict)
    for label, path in checkpoints:
        started = time.perf_counter()
        checkpoint = torch.load(path, map_location="cpu", weights_only=False)
        model = StudentDetector(
            pretrained_backbone=False,
            neck_type=config.neck_type,
            strides=config.assignment.strides,
            head_seed=config.schedule.seed,
        ).to(device)
        selected_state = _load_state(model, checkpoint, args.state)
        model.eval()
        criterion = _criterion(config)
        target_builder = _target_builder(config)
        slices = {}
        for cohort, indices in cohorts.items():
            measurements = []
            for start in range(0, len(indices), args.batch_size):
                batch_samples = _samples(
                    dataset, indices[start : start + args.batch_size]
                )
                if cohort.startswith("size/"):
                    batch_samples = _size_filtered_samples(
                        batch_samples, cohort.removeprefix("size/")
                    )
                measurements.append(
                    _measure_batch(
                        model,
                        batch_samples,
                        target_builder,
                        criterion,
                        device,
                    )
                )
            slices[cohort] = summarize_gradient_measurements(measurements)
        checkpoint_reports[label] = {
            "path": str(path),
            "sha256": sha256_file(path),
            "global_step": int(checkpoint.get("global_step", -1)),
            "state": selected_state,
            "duration_seconds": time.perf_counter() - started,
            "slices": slices,
        }
        del model
        if device.type == "cuda":
            torch.cuda.empty_cache()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "output": str(args.output.resolve()),
                "checkpoints": list(checkpoint_reports),
                "cohorts": list(cohorts),
                "batches_per_slice": args.batches_per_slice,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
