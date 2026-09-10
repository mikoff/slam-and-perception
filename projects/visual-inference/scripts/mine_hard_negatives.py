"""Mine a bounded, review-only hard-negative candidate bundle."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict, deque
import csv
import hashlib
import html
import json
from pathlib import Path
import sqlite3
import time
from typing import Any

from PIL import Image, ImageDraw, ImageOps
import torch
from torch.utils.data import DataLoader, Subset
import yaml

from student_detector.checkpoints import checkpoint_neck_type, load_model_state_strict
from student_detector.config import load_phase3_config
from student_detector.decoder import InferenceDecoder, class_agnostic_nms
from student_detector.hard_negative_mining import (
    deduplicate_and_cap,
    review_classification,
    select_hashed_pool_indices,
)
from student_detector.model import StudentDetector
from student_detector.proposal_utility import (
    intersection_from_iou,
    proposal_region_fraction,
)
from student_detector.provenance import sha256_file
from student_detector.quad_data import (
    QuadProposalDataset,
    collate_quad_proposal_samples,
)
from student_detector.quad_geometry import (
    canonicalize_quads,
    pairwise_quad_iou,
    quad_area,
    quad_validity,
    warmup_compiled_quad_iou,
)
from student_detector.quad_targets import point_validity_from_pixel_mask


STATE_NAMES = {0: "positive", 1: "ignore", 2: "trusted_background"}


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--pool-images", type=int)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--log-interval", type=int, default=50)
    return parser.parse_args()


def _load_policy(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise TypeError("mining policy must be a mapping")
    if value.get("schema_version") != "visual-inference-hard-negative-mining.v1":
        raise ValueError("unsupported mining policy")
    return value


def _valid_levels(
    valid_mask: torch.Tensor,
    output_levels: tuple[torch.Tensor, ...],
    strides: tuple[int, ...],
    device: torch.device,
) -> tuple[torch.Tensor, ...]:
    shapes = tuple((item.shape[-2], item.shape[-1]) for item in output_levels)
    levels = point_validity_from_pixel_mask(valid_mask.to(device), shapes, strides)[1]
    return tuple(level.unsqueeze(0) for level in levels)


def _box_quads(boxes: torch.Tensor) -> torch.Tensor:
    if not boxes.numel():
        return boxes.new_empty((0, 4, 2))
    x1, y1, x2, y2 = boxes.unbind(dim=1)
    return torch.stack(
        (
            torch.stack((x1, y1), dim=1),
            torch.stack((x2, y1), dim=1),
            torch.stack((x2, y2), dim=1),
            torch.stack((x1, y2), dim=1),
        ),
        dim=1,
    )


def _transform_quads(
    source_quads: torch.Tensor,
    transform: tuple[float, float, float],
    input_size: int,
    device: torch.device,
) -> torch.Tensor:
    if not source_quads.numel():
        return torch.empty((0, 4, 2), dtype=torch.float32, device=device)
    scale, offset_x, offset_y = transform
    quads = source_quads.to(device) * scale
    quads[..., 0] += offset_x
    quads[..., 1] += offset_y
    quads.clamp_(0, input_size)
    quads = canonicalize_quads(quads)
    return quads[quad_validity(quads)]


def _source_box(
    model_box: list[float],
    transform: tuple[float, float, float],
    original_size: tuple[int, int],
) -> list[float]:
    scale, offset_x, offset_y = transform
    height, width = original_size
    return [
        max(0.0, min(float(width), (model_box[0] - offset_x) / scale)),
        max(0.0, min(float(height), (model_box[1] - offset_y) / scale)),
        max(0.0, min(float(width), (model_box[2] - offset_x) / scale)),
        max(0.0, min(float(height), (model_box[3] - offset_y) / scale)),
    ]


def _dhash(image: Image.Image) -> str:
    pixels = list(ImageOps.grayscale(image).resize((9, 8)).get_flattened_data())
    bits = 0
    for row in range(8):
        for column in range(8):
            bits = (bits << 1) | int(
                pixels[row * 9 + column] > pixels[row * 9 + column + 1]
            )
    return f"{bits:016x}"


def _crop(image: Image.Image, box: list[float], padding: float = 0.08) -> Image.Image:
    x1, y1, x2, y2 = box
    pad_x = (x2 - x1) * padding
    pad_y = (y2 - y1) * padding
    bounds = (
        max(0, int(x1 - pad_x)),
        max(0, int(y1 - pad_y)),
        min(image.width, max(int(x2 + pad_x + 0.999), int(x1) + 1)),
        min(image.height, max(int(y2 + pad_y + 0.999), int(y1) + 1)),
    )
    return image.crop(bounds)


class SemanticRegions:
    """Read per-image source regions without loading the full manifest."""

    def __init__(self, index_path: Path) -> None:
        self.connection = sqlite3.connect(
            f"file:{index_path}?mode=ro&immutable=1", uri=True
        )

    def rows(self, image_id: int) -> list[tuple[list[list[float]], str, str]]:
        rows = self.connection.execute(
            """
            SELECT quad_json, ignore_region, category_name
            FROM annotations WHERE image_id=?
            """,
            (image_id,),
        ).fetchall()
        return [
            (json.loads(encoded), STATE_NAMES[int(state)], str(category))
            for encoded, state, category in rows
        ]

    def close(self) -> None:
        self.connection.close()


def _index_metadata(index_path: Path) -> dict[str, str]:
    with sqlite3.connect(
        f"file:{index_path}?mode=ro&immutable=1", uri=True
    ) as connection:
        return dict(connection.execute("SELECT key, value FROM metadata").fetchall())


def _semantic_matches(
    rows: list[tuple[list[list[float]], str, str]],
    proposals: torch.Tensor,
    transform: tuple[float, float, float],
    input_size: int,
    device: torch.device,
) -> tuple[list[str | None], list[str | None], torch.Tensor]:
    if not rows or not proposals.numel():
        return (
            [None] * len(proposals),
            [None] * len(proposals),
            proposals.new_zeros((len(proposals),)),
        )
    source = torch.tensor([row[0] for row in rows], dtype=torch.float32)
    regions = _transform_quads(source, transform, input_size, device)
    if regions.shape[0] != len(rows):
        raise RuntimeError("source semantic region became invalid during letterbox")
    overlaps = pairwise_quad_iou(regions, proposals)
    intersections = intersection_from_iou(
        overlaps, quad_area(regions), quad_area(proposals)
    )
    fractions = intersections / quad_area(proposals)[None, :].clamp_min(1e-7)
    best_fraction, best_index = fractions.max(dim=0)
    states = [rows[index][1] for index in best_index.tolist()]
    categories = [rows[index][2] for index in best_index.tolist()]
    return states, categories, best_fraction.clamp(0, 1)


def _candidate_id(
    source: str, image_id: int, level: int, location: int, box: list[float]
) -> str:
    identity = json.dumps(
        [source, image_id, level, location, [round(value, 3) for value in box]],
        separators=(",", ":"),
    ).encode()
    return hashlib.sha256(identity).hexdigest()[:20]


def _stratified_audit(
    candidates: list[dict[str, Any]], count: int
) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str], deque[dict[str, Any]]] = defaultdict(deque)
    for candidate in sorted(
        candidates, key=lambda item: (-float(item["score"]), item["candidate_id"])
    ):
        groups[(candidate["classification"], candidate["domain"])].append(candidate)
    selected: list[dict[str, Any]] = []
    keys = sorted(groups)
    while keys and len(selected) < count:
        remaining = []
        for key in keys:
            if groups[key] and len(selected) < count:
                selected.append(groups[key].popleft())
            if groups[key]:
                remaining.append(key)
        keys = remaining
    return selected


def _draw_quad(draw: ImageDraw.ImageDraw, quad: list[list[float]], color: str) -> None:
    points = [(float(x), float(y)) for x, y in quad]
    draw.line(points + [points[0]], fill=color, width=3)


def _write_audit(
    output_dir: Path,
    candidates: list[dict[str, Any]],
    dataset: QuadProposalDataset,
    records_by_identity: dict[tuple[str, int], int],
    semantic: SemanticRegions,
) -> None:
    images_dir = output_dir / "audit_images"
    images_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    cards = []
    colors = {"positive": "lime", "ignore": "yellow", "trusted_background": "blue"}
    for candidate in candidates:
        key = (candidate["source_dataset"], candidate["image_id"])
        record = dataset.records[records_by_identity[key]]
        source_path = dataset.image_root / record.file_name
        with Image.open(source_path) as loaded:
            image = loaded.convert("RGB")
        draw = ImageDraw.Draw(image)
        for quad, state, _ in semantic.rows(record.image_id):
            _draw_quad(draw, quad, colors[state])
        box = candidate["source_box"]
        draw.rectangle(tuple(box), outline="red", width=5)
        image.thumbnail((720, 480), Image.Resampling.LANCZOS)
        relative = Path("audit_images") / f"{candidate['candidate_id']}.jpg"
        image.save(output_dir / relative, quality=88)
        rows.append(
            {
                "candidate_id": candidate["candidate_id"],
                "decision": "review",
                "note": "",
                "classification": candidate["classification"],
                "score": f"{candidate['score']:.6f}",
                "domain": candidate["domain"],
                "source_dataset": candidate["source_dataset"],
                "image_id": candidate["image_id"],
                "semantic_category": candidate["semantic_category"] or "",
                "trusted_background_fraction": (
                    f"{candidate['trusted_background_fraction']:.6f}"
                ),
            }
        )
        cards.append(
            "<article><img src='{}'><h3>{}</h3><p>{} · {} · score {:.3f}</p>"
            "<p>semantic={} · trusted={:.3f}</p></article>".format(
                html.escape(str(relative)),
                html.escape(candidate["candidate_id"]),
                html.escape(candidate["classification"]),
                html.escape(candidate["domain"]),
                candidate["score"],
                html.escape(candidate["semantic_category"] or "none"),
                candidate["trusted_background_fraction"],
            )
        )
    fields = list(rows[0]) if rows else ["candidate_id", "decision", "note"]
    with (output_dir / "review.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    document = """<!doctype html><meta charset='utf-8'><title>Hard-negative audit</title>
<style>body{font:14px sans-serif;background:#111;color:#eee}main{display:grid;grid-template-columns:repeat(auto-fill,minmax(360px,1fr));gap:12px}article{background:#222;padding:8px}img{width:100%;height:auto}p{margin:.3em 0}</style>
<h1>Hard-negative mining audit</h1>
<p>Red: candidate proposal; lime: positive; yellow: ignore; blue: trusted background.</p>
<p>Edit <code>review.csv</code>: set decision to <code>approve_negative</code>, <code>ignore</code>, or <code>uncertain</code>. Only clearly verified background may be approved; potential objects remain ignore.</p><main>"""
    document += "\n".join(cards) + "</main>\n"
    (output_dir / "index.html").write_text(document, encoding="utf-8")


@torch.inference_mode()
def main() -> None:
    args = _args()
    if args.workers < 0 or args.log_interval < 0:
        raise ValueError("workers and log interval must be nonnegative")
    policy = _load_policy(args.policy)
    config = load_phase3_config(args.config)
    if args.config.name != policy["base_config"]:
        raise ValueError("mining policy does not name the supplied base config")
    checkpoint_sha256 = sha256_file(args.checkpoint)
    if checkpoint_sha256 != policy["checkpoint_sha256"]:
        raise ValueError("checkpoint hash does not match the frozen mining policy")
    pool_images = args.pool_images or int(policy["pool"]["images"])
    if pool_images < 1 or pool_images > int(policy["pool"]["images"]):
        raise ValueError("pool images must be within the frozen policy pool size")
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA mining requested but no CUDA device is visible")
    annotations = config.data.quad_train_annotations or config.data.train_annotations
    dataset = QuadProposalDataset(
        annotations,
        config.data.image_root,
        config.data.index_dir / "quad_train.sqlite",
        config.data,
        config.augmentation,
        training=False,
        force_index=False,
        verify_index_source=False,
    )
    index_metadata = _index_metadata(dataset.index_path)
    manifest_sha256 = index_metadata.get("source_signature", "").removeprefix("sha256:")
    if manifest_sha256 != policy["train_manifest_sha256"]:
        raise ValueError("prebuilt index source hash does not match the mining policy")
    selected = select_hashed_pool_indices(
        dataset.records,
        config.data.source_weights,
        pool_images,
        seed=int(policy["pool"]["seed"]),
    )
    loader = DataLoader(
        Subset(dataset, selected),
        batch_size=config.data.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=device.type == "cuda",
        persistent_workers=args.workers > 0,
        collate_fn=collate_quad_proposal_samples,
    )
    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    neck_type = checkpoint_neck_type(checkpoint)
    model = StudentDetector(pretrained_backbone=False, neck_type=neck_type)
    load_model_state_strict(
        model,
        checkpoint,
        kind="hbb",
        neck_type=neck_type,
        state_key=policy["checkpoint_state"],
    )
    model = model.to(device).eval()
    proposal_policy = policy["proposal"]
    decoder = InferenceDecoder(
        strides=config.assignment.strides,
        top_k=int(proposal_policy["pre_nms_top_k"]),
        nms_iou_threshold=1.0,
        max_detections=int(proposal_policy["pre_nms_top_k"]),
        score_mode=config.inference.score_mode,
    )
    if device.type == "cuda":
        warmup_compiled_quad_iou(device)
    semantic = SemanticRegions(dataset.index_path)
    raw_candidates: list[dict[str, Any]] = []
    started = time.monotonic()
    seen = 0
    for images, samples in loader:
        images = images.to(device, non_blocking=device.type == "cuda")
        output = model(images)
        masks = tuple(
            zip(
                *(
                    _valid_levels(
                        sample.valid_mask,
                        output.objectness,
                        config.assignment.strides,
                        device,
                    )
                    for sample in samples
                )
            )
        )
        detections = decoder(
            output,
            (images.shape[-2], images.shape[-1]),
            tuple(torch.cat(level, dim=0) for level in masks),
        )
        for sample, detection in zip(samples, detections, strict=True):
            eligible = torch.nonzero(
                detection.scores >= float(proposal_policy["score_threshold"])
            ).flatten()
            keep_local = class_agnostic_nms(
                detection.boxes.float()[eligible],
                detection.scores.float()[eligible],
                float(proposal_policy["nms_iou_threshold"]),
            )[: int(proposal_policy["proposal_budget"])]
            keep = eligible[keep_local]
            boxes = detection.boxes.float()[keep]
            scores = detection.scores.float()[keep]
            levels = detection.levels[keep]
            locations = detection.location_indices[keep]
            proposals = _box_quads(boxes)
            gt = sample.quads.to(device)
            trusted = sample.trusted_background_quads.to(device)
            ignored = sample.ignore_quads.to(device)
            best_object_iou = (
                pairwise_quad_iou(gt, proposals).amax(dim=0)
                if gt.numel()
                else scores.new_zeros((len(proposals),))
            )
            trusted_fraction = proposal_region_fraction(
                pairwise_quad_iou(trusted, proposals),
                trusted,
                proposals,
                disjoint_regions=True,
            )
            ignore_fraction = proposal_region_fraction(
                pairwise_quad_iou(ignored, proposals),
                ignored,
                proposals,
                disjoint_regions=False,
            )
            source_rows = semantic.rows(sample.image_id)
            semantic_states, semantic_categories, semantic_fractions = (
                _semantic_matches(
                    source_rows,
                    proposals,
                    sample.transform,
                    config.data.input_size,
                    device,
                )
            )
            record = dataset.records[selected[seen]]
            with Image.open(dataset.image_root / record.file_name) as loaded:
                source_image = loaded.convert("RGB")
            scanned = 0
            for index in range(len(proposals)):
                classification = review_classification(
                    best_object_iou=float(best_object_iou[index]),
                    ignore_fraction=float(ignore_fraction[index]),
                    trusted_background_fraction=float(trusted_fraction[index]),
                    semantic_state=semantic_states[index],
                    semantic_fraction=float(semantic_fractions[index]),
                    unmatched_iou=float(proposal_policy["unmatched_iou"]),
                    region_fraction=float(proposal_policy["region_fraction"]),
                )
                if classification is None:
                    continue
                if scanned >= int(proposal_policy["scan_per_image"]):
                    break
                model_box = [float(value) for value in boxes[index].tolist()]
                source_box = _source_box(
                    model_box, sample.transform, sample.original_size
                )
                crop = _crop(source_image, source_box)
                candidate_id = _candidate_id(
                    sample.source_dataset,
                    sample.image_id,
                    int(levels[index]),
                    int(locations[index]),
                    source_box,
                )
                raw_candidates.append(
                    {
                        "candidate_id": candidate_id,
                        "source_dataset": sample.source_dataset,
                        "domain": sample.domain,
                        "image_id": sample.image_id,
                        "file_name": record.file_name,
                        "score": float(scores[index]),
                        "model_box": model_box,
                        "source_box": source_box,
                        "feature_level": int(levels[index]),
                        "location_index": int(locations[index]),
                        "best_object_iou": float(best_object_iou[index]),
                        "ignore_fraction": float(ignore_fraction[index]),
                        "trusted_background_fraction": float(trusted_fraction[index]),
                        "semantic_state": semantic_states[index],
                        "semantic_category": semantic_categories[index],
                        "semantic_fraction": float(semantic_fractions[index]),
                        "classification": classification,
                        "crop_dhash": _dhash(crop),
                    }
                )
                scanned += 1
            seen += 1
            if args.log_interval and seen % args.log_interval == 0:
                elapsed = time.monotonic() - started
                print(
                    f"mining: {seen}/{pool_images} images "
                    f"({seen / elapsed:.2f} images/s)",
                    flush=True,
                )
    kept, rejected = deduplicate_and_cap(
        raw_candidates,
        per_image=int(policy["selection"]["per_image"]),
        per_domain={
            str(key): int(value)
            for key, value in policy["selection"]["per_domain"].items()
        },
        perceptual_hamming=int(policy["selection"]["perceptual_hamming"]),
    )
    selected_identity = json.dumps(
        [
            (dataset.records[index].source_dataset, dataset.records[index].image_id)
            for index in selected
        ],
        separators=(",", ":"),
    ).encode()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    audit = _stratified_audit(kept, min(int(policy["audit"]["images"]), len(kept)))
    records_by_identity = {
        (record.source_dataset, record.image_id): index
        for index, record in enumerate(dataset.records)
    }
    _write_audit(output_dir, audit, dataset, records_by_identity, semantic)
    semantic.close()
    manifest = {
        "schema_version": "visual-inference-hard-negative-candidates.v1",
        "status": (
            "smoke_complete_not_for_review"
            if pool_images != int(policy["pool"]["images"])
            else "candidate_review_required"
        ),
        "promotion_policy": (
            "No candidate is training supervision until an owner-approved rule "
            "accepts certified background; possible objects and uncertain space "
            "remain ignore."
        ),
        "inputs": {
            "config": str(args.config.resolve()),
            "config_sha256": sha256_file(args.config),
            "policy": str(args.policy.resolve()),
            "policy_sha256": sha256_file(args.policy),
            "checkpoint": str(args.checkpoint.resolve()),
            "checkpoint_sha256": checkpoint_sha256,
            "checkpoint_state": policy["checkpoint_state"],
            "train_manifest": str(annotations.resolve()),
            "train_manifest_sha256": manifest_sha256,
            "train_index": str(dataset.index_path),
        },
        "pool": {
            "images": pool_images,
            "seed": int(policy["pool"]["seed"]),
            "identity_sha256": hashlib.sha256(selected_identity).hexdigest(),
            "source_counts": dict(
                sorted(
                    Counter(
                        dataset.records[index].source_dataset for index in selected
                    ).items()
                )
            ),
            "validation_excluded": True,
        },
        "counts": {
            "raw_candidates": len(raw_candidates),
            "retained_candidates": len(kept),
            "audit_candidates": len(audit),
            "retained_by_classification": dict(
                sorted(Counter(item["classification"] for item in kept).items())
            ),
            "retained_by_domain": dict(
                sorted(Counter(item["domain"] for item in kept).items())
            ),
            "rejected": rejected,
        },
        "candidates": kept,
    }
    manifest_path = output_dir / "candidates.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "manifest": str(manifest_path),
                "status": manifest["status"],
                "pool_images": pool_images,
                **manifest["counts"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
