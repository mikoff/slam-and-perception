"""Write a source-coordinate HBB/quad proposal-contract visual audit."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import html
import json
from pathlib import Path
from typing import Any

import torch
from PIL import Image, ImageDraw

from student_detector.checkpoints import checkpoint_neck_type, load_model_state_strict
from student_detector.config import load_phase3_config
from student_detector.data import select_source_mixture_indices
from student_detector.decoder import InferenceDecoder
from student_detector.model import QuadProposalDetector, StudentDetector
from student_detector.proposal_contract import (
    PROPOSAL_CONTRACT_SCHEMA,
    ProposalRecord,
    SourceTransform,
    records_from_hbb,
    records_from_quads,
)
from student_detector.quad_data import QuadProposalDataset
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_targets import point_validity_from_pixel_mask


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--hbb-checkpoint", type=Path, required=True)
    parser.add_argument("--quad-checkpoint", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--images", type=int, default=8)
    parser.add_argument("--display-proposals", type=int, default=10)
    parser.add_argument("--hbb-nms-iou-threshold", type=float)
    parser.add_argument("--quad-nms-iou-threshold", type=float)
    parser.add_argument("--hbb-score-threshold", type=float, default=0.0)
    parser.add_argument("--quad-score-threshold", type=float, default=0.0)
    parser.add_argument("--device", default="cuda")
    parser.add_argument(
        "--checkpoint-state", choices=("model", "ema_model"), default="ema_model"
    )
    return parser.parse_args()


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_model(
    kind: str, checkpoint_path: Path, state: str, device: torch.device
) -> torch.nn.Module:
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    neck_type = checkpoint_neck_type(checkpoint)
    model: torch.nn.Module
    if kind == "hbb":
        model = StudentDetector(pretrained_backbone=False, neck_type=neck_type)
    else:
        model = QuadProposalDetector(pretrained_backbone=False, neck_type=neck_type)
    load_model_state_strict(
        model,
        checkpoint,
        kind=kind,
        neck_type=neck_type,
        state_key=state,
    )
    return model.to(device).eval()


def _valid_levels(
    valid_mask: torch.Tensor,
    output_levels: tuple[torch.Tensor, ...],
    strides: tuple[int, ...],
    device: torch.device,
) -> tuple[torch.Tensor, ...]:
    shapes = tuple((item.shape[-2], item.shape[-1]) for item in output_levels)
    levels = point_validity_from_pixel_mask(valid_mask.to(device), shapes, strides)[1]
    return tuple(level.unsqueeze(0) for level in levels)


def _draw_panel(
    source: Image.Image,
    label: str,
    records: tuple[ProposalRecord, ...],
    ground_truth: torch.Tensor,
    display_proposals: int,
) -> Image.Image:
    panel = source.copy().convert("RGB")
    draw = ImageDraw.Draw(panel)
    for quad in ground_truth:
        points = [(float(x), float(y)) for x, y in quad]
        draw.line(points + [points[0]], fill=(40, 210, 80), width=3)
    for record in records[:display_proposals]:
        points = list(record.source_geometry)
        draw.line(points + [points[0]], fill=(245, 170, 35), width=2)
        draw.text(points[0], f"{record.rank}:{record.score:.2f}", fill=(245, 170, 35))
    draw.rectangle((0, 0, min(panel.width, 230), 26), fill=(0, 0, 0))
    draw.text((6, 6), label, fill=(255, 255, 255))
    return panel


def _write_html(output_dir: Path, rows: list[dict[str, Any]]) -> None:
    cards = "\n".join(
        "<article><h2>"
        f"{html.escape(str(row['source_dataset']))} / {row['image_id']}"
        "</h2><img src='"
        f"{html.escape(str(row['visual']))}' loading='lazy'></article>"
        for row in rows
    )
    document = f"""<!doctype html>
<meta charset="utf-8">
<title>Phase 2.2 proposal-contract audit</title>
<style>
body {{ background:#171717; color:#eee; font-family:sans-serif; margin:24px; }}
article {{ margin-bottom:32px; }} img {{ max-width:100%; height:auto; }}
.key {{ margin-bottom:20px; }}
</style>
<h1>Phase 2.2 source-coordinate proposal audit</h1>
<p class="key">Green: ground truth. Amber: top decoded proposals. Left: HBB P3–P5.
Right: quad P3–P5. Both panels use identical colors to avoid implying a winner.</p>
{cards}
"""
    (output_dir / "index.html").write_text(document, encoding="utf-8")


@torch.no_grad()
def main() -> None:
    args = _args()
    if args.images < 1 or args.display_proposals < 1:
        raise ValueError("image and display-proposal counts must be positive")
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA audit requested but no CUDA device is visible")
    config = load_phase3_config(args.config)
    dataset = QuadProposalDataset(
        config.data.quad_val_annotations or config.data.val_annotations,
        config.data.image_root,
        config.data.index_dir / "quad_val.sqlite",
        config.data,
        config.augmentation,
        training=False,
        force_index=False,
    )
    selected = select_source_mixture_indices(
        dataset.records, config.data.source_weights, args.images
    )
    hbb_model = _load_model(
        "hbb", args.hbb_checkpoint, args.checkpoint_state, device
    )
    quad_model = _load_model(
        "quad", args.quad_checkpoint, args.checkpoint_state, device
    )
    hbb_decoder = InferenceDecoder(
        strides=config.assignment.strides,
        top_k=config.inference.pre_nms_top_k,
        nms_iou_threshold=(
            args.hbb_nms_iou_threshold
            if args.hbb_nms_iou_threshold is not None
            else config.inference.nms_iou_threshold
        ),
        max_detections=100,
        score_mode=config.inference.score_mode,
    )
    quad_decoder = QuadInferenceDecoder(
        strides=config.assignment.strides,
        pre_nms_top_k=config.inference.pre_nms_top_k,
        nms_iou_threshold=(
            args.quad_nms_iou_threshold
            if args.quad_nms_iou_threshold is not None
            else config.inference.nms_iou_threshold
        ),
        max_proposals=100,
    )
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    audit_rows: list[dict[str, Any]] = []
    for audit_index, dataset_index in enumerate(selected):
        database_record = dataset.records[dataset_index]
        sample = dataset[dataset_index]
        images = sample.image.unsqueeze(0).to(device)
        hbb_output = hbb_model(images)
        quad_output = quad_model(images)
        image_size = (int(images.shape[-2]), int(images.shape[-1]))
        hbb_detection = hbb_decoder(
            hbb_output,
            image_size,
            _valid_levels(
                sample.valid_mask,
                hbb_output.objectness,
                config.assignment.strides,
                device,
            ),
        )[0]
        quad_detection = quad_decoder(
            quad_output,
            image_size,
            _valid_levels(
                sample.valid_mask,
                quad_output.quality,
                config.assignment.strides,
                device,
            ),
        )[0]
        transform = SourceTransform(
            *sample.transform,
            source_size=sample.original_size,
            model_size=image_size,
        )
        hbb_records = records_from_hbb(
            hbb_detection,
            image_id=sample.image_id,
            source_dataset=sample.source_dataset,
            source_transform=transform,
            score_threshold=args.hbb_score_threshold,
        )
        quad_records = records_from_quads(
            quad_detection,
            image_id=sample.image_id,
            source_dataset=sample.source_dataset,
            source_transform=transform,
            score_threshold=args.quad_score_threshold,
        )
        ground_truth = transform.model_to_source(sample.quads)
        with Image.open(config.data.image_root / database_record.file_name) as loaded:
            source = loaded.convert("RGB")
        hbb_panel = _draw_panel(
            source, "HBB P3-P5", hbb_records, ground_truth, args.display_proposals
        )
        quad_panel = _draw_panel(
            source, "Quad P3-P5", quad_records, ground_truth, args.display_proposals
        )
        visual = f"{audit_index:02d}_{sample.source_dataset}_{sample.image_id}.jpg"
        canvas = Image.new("RGB", (source.width * 2, source.height))
        canvas.paste(hbb_panel, (0, 0))
        canvas.paste(quad_panel, (source.width, 0))
        canvas.save(output_dir / visual, quality=92)
        audit_rows.append(
            {
                "image_id": sample.image_id,
                "source_dataset": sample.source_dataset,
                "visual": visual,
                "hbb": [asdict(record) for record in hbb_records],
                "quad": [asdict(record) for record in quad_records],
            }
        )
    report = {
        "schema_version": "visual-inference-proposal-contract-audit.v1",
        "proposal_schema_version": PROPOSAL_CONTRACT_SCHEMA,
        "status": "awaiting_owner_visual_review",
        "production_proposal_budget": 100,
        "checkpoint_state": args.checkpoint_state,
        "hbb_nms_iou_threshold": (
            args.hbb_nms_iou_threshold
            if args.hbb_nms_iou_threshold is not None
            else config.inference.nms_iou_threshold
        ),
        "quad_nms_iou_threshold": (
            args.quad_nms_iou_threshold
            if args.quad_nms_iou_threshold is not None
            else config.inference.nms_iou_threshold
        ),
        "hbb_score_threshold": args.hbb_score_threshold,
        "quad_score_threshold": args.quad_score_threshold,
        "config_sha256": _sha256(args.config),
        "validation_manifest_sha256": _sha256(
            config.data.quad_val_annotations or config.data.val_annotations
        ),
        "hbb_checkpoint_sha256": _sha256(args.hbb_checkpoint),
        "quad_checkpoint_sha256": _sha256(args.quad_checkpoint),
        "images": audit_rows,
    }
    (output_dir / "audit.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    _write_html(output_dir, audit_rows)
    print(f"Wrote {len(audit_rows)} proposal-contract audit images to {output_dir}")


if __name__ == "__main__":
    main()
