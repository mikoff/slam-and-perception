"""Deterministic mixed-geometry microfixture for the G6 learnability gate."""

from __future__ import annotations

import hashlib
import json
import random
from pathlib import Path
from typing import Any

import yaml
from PIL import Image, ImageDraw


FIXTURE_SCHEMA = "quad-g6-microfixture.v1"


def _background(seed: int, color: tuple[int, int, int]) -> Image.Image:
    generator = random.Random(seed)
    image = Image.new("RGB", (384, 384), color)
    draw = ImageDraw.Draw(image)
    for _ in range(80):
        x = generator.randrange(384)
        y = generator.randrange(384)
        shade = tuple(max(0, min(255, channel + generator.randrange(-12, 13))) for channel in color)
        draw.rectangle((x, y, min(x + 3, 383), min(y + 3, 383)), fill=shade)
    for offset in range(0, 384, 32):
        draw.line((0, offset, 383, offset), fill=tuple(channel + 4 for channel in color))
    return image


def _draw_object(
    image: Image.Image,
    quad: list[list[float]],
    fill: tuple[int, int, int],
) -> None:
    points = [(round(point[0]), round(point[1])) for point in quad]
    draw = ImageDraw.Draw(image)
    draw.polygon(points, fill=fill, outline=(245, 245, 245), width=3)
    center_x = round(sum(point[0] for point in quad) / 4)
    center_y = round(sum(point[1] for point in quad) / 4)
    draw.ellipse((center_x - 6, center_y - 6, center_x + 6, center_y + 6), fill=(20, 20, 20))


def _record(
    quad: list[list[float]],
    *,
    tier: str,
    category: str,
    condition: str,
) -> dict[str, Any]:
    xs = [point[0] for point in quad]
    ys = [point[1] for point in quad]
    return {
        "aliases": [],
        "bbox": [min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys)],
        "fit_coverage": 1.0,
        "fit_tightness": 1.0,
        "geometry_tier": tier,
        "object_condition": condition,
        "quad": quad,
        "source_annotation_id": f"g6-{category}",
        "source_category": category,
        "state": "positive",
        "valid": True,
    }


def _image_record(
    image_id: int,
    file_name: str,
    positives: list[dict[str, Any]],
    ignores: list[dict[str, Any]],
    *,
    trusted: bool,
) -> dict[str, Any]:
    return {
        "background_supervision": trusted,
        "camera_type": "perspective",
        "file_name": file_name,
        "height": 384,
        "ignore": ignores,
        "image_id": image_id,
        "positive": positives,
        "source_dataset": "g6_dense" if trusted else "g6_weak",
        "source_image_id": str(image_id),
        "source_split": "train",
        "trusted_background": [],
        "width": 384,
    }


def create_g6_microfixture(output_dir: str | Path) -> dict[str, str]:
    """Create fixture images, compact manifest, config, and fingerprint."""
    root = Path(output_dir).resolve()
    images_dir = root / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    rectangle = [[34.0, 42.0], [154.0, 42.0], [154.0, 158.0], [34.0, 158.0]]
    rotated = [[192.0, 34.0], [294.0, 92.0], [248.0, 174.0], [146.0, 116.0]]
    trapezoid = [[44.0, 210.0], [164.0, 190.0], [184.0, 326.0], [64.0, 306.0]]
    thin = [[28.0, 168.0], [348.0, 194.0], [346.0, 210.0], [26.0, 184.0]]
    parent = [[46.0, 70.0], [326.0, 70.0], [326.0, 332.0], [46.0, 332.0]]
    nested = [[126.0, 226.0], [222.0, 226.0], [222.0, 286.0], [126.0, 286.0]]
    ignored = [[304.0, 18.0], [378.0, 18.0], [378.0, 118.0], [304.0, 118.0]]

    definitions = [
        (1, [(rectangle, (210, 55, 55))], [], True),
        (2, [(rotated, (50, 105, 220)), (trapezoid, (230, 175, 40))], [], False),
        (3, [(thin, (45, 205, 190))], [], False),
        (4, [(parent, (65, 170, 80)), (nested, (205, 55, 190))], [(ignored, (230, 115, 35))], True),
        (5, [], [], True),
        (6, [], [], False),
    ]
    records_by_id = {
        1: [_record(rectangle, tier="source_hbb", category="crate", condition="whole_object")],
        2: [
            _record(rotated, tier="rotated_rect", category="rotated_panel", condition="whole_object"),
            _record(trapezoid, tier="source_quad", category="perspective_sign", condition="perspective_object"),
        ],
        3: [_record(thin, tier="fitted_quad", category="thin_beam", condition="thin_object")],
        4: [
            _record(parent, tier="source_hbb", category="vehicle", condition="whole_object"),
            _record(nested, tier="source_hbb", category="display", condition="nested_part"),
        ],
        5: [],
        6: [],
    }
    ignore_record = _record(
        ignored,
        tier="source_hbb",
        category="ambiguous_region",
        condition="ignore",
    )
    ignore_record["state"] = "ignore"

    manifest_images: list[dict[str, Any]] = []
    for image_id, objects, ignored_objects, trusted in definitions:
        image = _background(image_id, (92 + image_id * 4, 98 + image_id * 3, 106 + image_id * 2))
        for quad, color in objects:
            _draw_object(image, quad, color)
        for quad, color in ignored_objects:
            _draw_object(image, quad, color)
        file_name = f"images/g6_{image_id}.png"
        image.save(root / file_name)
        manifest_images.append(
            _image_record(
                image_id,
                file_name,
                records_by_id[image_id],
                [ignore_record] if image_id == 4 else [],
                trusted=trusted,
            )
        )

    manifest = {
        "fixture_schema": FIXTURE_SCHEMA,
        "images": manifest_images,
        "object_contract": "bounded_promptable_physical_instance",
        "schema_version": "quad-proposal-manifest.v1",
        "split": "train",
    }
    manifest_path = root / "proposals_g6.json"
    manifest_bytes = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode()
    manifest_path.write_bytes(manifest_bytes)
    fingerprint = hashlib.sha256(manifest_bytes).hexdigest()

    config = {
        "data": {
            "train_annotations": str(manifest_path),
            "val_annotations": str(manifest_path),
            "quad_train_annotations": str(manifest_path),
            "quad_val_annotations": str(manifest_path),
            "image_root": str(root),
            "index_dir": str(root / "index"),
            "input_size": 384,
            "batch_size": 6,
            "workers": 0,
            "empty_fraction": 0.3333333333333333,
            "domain_weights": {"general": 1.0},
            "source_domains": {"g6_dense": "general", "g6_weak": "general"},
            "source_weights": {"g6_dense": 0.5, "g6_weak": 0.5},
            "dense_background_sources": ["g6_dense"],
            "background_loss_weights": {},
            "quad_regular_min_side": 16,
            "thin_major_axis_min": 8,
            "thin_aspect_ratio_min": 3,
            "thin_area": 16,
        },
        "augmentation": {
            "horizontal_flip_probability": 0.0,
            "color_jitter_probability": 0.0,
            "scale_min": 1.0,
            "scale_max": 1.0,
            "translation_fraction": 0.0,
            "positive_visible_fraction": 0.6,
            "ignore_visible_fraction": 0.2,
        },
        "assignment": {"strides": [8, 16, 32], "prior_sizes": [64, 128, 256], "top_k": 9},
        "inference": {"pre_nms_top_k": 300, "nms_iou_threshold": 0.9, "max_proposals": 100},
        "quad": {
            "top_k": 9,
            "gamma": 2.0,
            "scale_sigma": 0.75,
            "eligible_levels": 2,
            "weak_negative_weight": 0.05,
            "quality_weight": 1.0,
            "corner_weight": 2.0,
            "validity_weight": 0.05,
            "quality_focal_beta": 2.0,
            "quality_target_mode": "centerness",
            "quality_blend": 0.0,
            "geometry_quality_target": "exact_iou",
        },
        "schedule": {
            "epochs": 100,
            "freeze_backbone_epochs": 0,
            "head_learning_rate": 0.0003,
            "backbone_lr_multiplier": 0.1,
            "reference_effective_batch": 64,
            "weight_decay": 0.0001,
            "warmup_steps": 25,
            "min_lr_ratio": 0.01,
            "gradient_clip_norm": 10.0,
            "ema_decay": 0.9998,
            "accumulation_steps": 1,
            "amp": True,
            "seed": 42,
            "checkpoint_every_epochs": 1,
            "checkpoint_every_steps": 100,
        },
        "pretrained_backbone": True,
        "output_dir": str(root / "run"),
    }
    config_path = root / "phase3_g6.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    metadata = {
        "config": str(config_path),
        "fixture_schema": FIXTURE_SCHEMA,
        "image_ids": "1,2,3,4,5,6",
        "manifest": str(manifest_path),
        "manifest_sha256": fingerprint,
        "root": str(root),
    }
    (root / "fixture.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return metadata
