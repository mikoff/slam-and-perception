"""Typed Phase-3 training configuration loaded from YAML."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class DataConfig:
    train_annotations: Path
    val_annotations: Path
    image_root: Path
    index_dir: Path
    quad_train_annotations: Path | None = None
    quad_val_annotations: Path | None = None
    input_size: int = 384
    batch_size: int = 10
    batches_per_epoch: int | None = None
    workers: int = 4
    empty_fraction: float = 0.075
    domain_weights: dict[str, float] = field(default_factory=lambda: {
        "general": 0.50,
        "automotive": 0.30,
        "fisheye": 0.20,
    })
    source_domains: dict[str, str] = field(default_factory=lambda: {
        "coco_2017": "general",
        "nuimages": "automotive",
        "bdd100k_images_100k": "automotive",
        "woodscape_rgb_fisheye": "fisheye",
    })
    source_weights: dict[str, float] = field(default_factory=lambda: {
        "coco_2017": 0.50,
        "nuimages": 0.18,
        "bdd100k_images_100k": 0.12,
        "woodscape_rgb_fisheye": 0.20,
    })
    dense_background_sources: tuple[str, ...] = ("coco_2017",)
    background_loss_weights: dict[str, float] = field(default_factory=lambda: {
        "woodscape_rgb_fisheye": 0.05,
    })
    tiny_area: float = 100.0
    tiny_min_side: float = 4.0
    quad_regular_min_side: float = 16.0
    thin_major_axis_min: float = 8.0
    thin_aspect_ratio_min: float = 3.0
    thin_area: float = 16.0
    component_categories: tuple[str, ...] = (
        "face", "head", "hand", "arm", "leg", "foot", "shoe",
        "wheel", "tire", "license_plate", "mirror", "door",
        "handle", "screen", "logo",
    )
    parent_categories: tuple[str, ...] = (
        "person", "pedestrian_adult", "pedestrian_child",
        "pedestrian_other", "pedestrian_construction_worker",
        "pedestrian_police_officer", "car", "truck", "bus", "van",
        "motorcycle", "bicycle", "construction_vehicle",
    )
    component_containment_threshold: float = 0.8


@dataclass(frozen=True)
class AugmentationConfig:
    horizontal_flip_probability: float = 0.5
    color_jitter_probability: float = 0.8
    brightness: float = 0.20
    contrast: float = 0.20
    saturation: float = 0.20
    blur_probability: float = 0.10
    noise_probability: float = 0.10
    jpeg_probability: float = 0.10
    scale_min: float = 0.8
    scale_max: float = 1.2
    translation_fraction: float = 0.10
    positive_visible_fraction: float = 0.60
    ignore_visible_fraction: float = 0.20


@dataclass(frozen=True)
class AssignmentConfig:
    strides: tuple[int, int, int] = (8, 16, 32)
    prior_sizes: tuple[int, int, int] = (64, 128, 256)
    top_k: int = 9
    center_radius: float | None = 1.5


@dataclass(frozen=True)
class LossConfig:
    objectness_weight: float = 1.0
    box_weight: float = 2.0
    ltrb_weight: float = 0.5
    centerness_weight: float = 0.0
    objectness_loss: str = "quality_focal"
    quality_focal_beta: float = 2.0
    box_loss: str = "ciou"
    box_weighting: str = "uniform"
    focal_alpha: float = 0.25
    focal_gamma: float = 2.0


@dataclass(frozen=True)
class InferenceConfig:
    pre_nms_top_k: int = 300
    # Proposal recall is the primary objective; leave duplicate suppression
    # conservative and let the fixed 100-proposal cap control output size.
    nms_iou_threshold: float = 0.9
    max_proposals: int = 100
    score_mode: str = "objectness"


@dataclass(frozen=True)
class QuadConfig:
    top_k: int = 9
    gamma: float = 2.0
    scale_sigma: float = 0.75
    eligible_levels: int = 2
    scale_measure: str = "area"
    weak_negative_weight: float = 0.0
    quality_weight: float = 1.0
    corner_weight: float = 2.0
    corner_smooth_l1_beta: float = 1.0
    gwd_weight: float = 0.0
    validity_weight: float = 0.05
    quality_focal_beta: float = 2.0
    quality_target_mode: str = "centerness"
    quality_blend: float = 0.0
    geometry_quality_target: str = "corner_proxy"


@dataclass(frozen=True)
class ScheduleConfig:
    epochs: int = 40
    freeze_backbone_epochs: int = 2
    head_learning_rate: float = 3e-4
    backbone_lr_multiplier: float = 0.1
    reference_effective_batch: int = 64
    weight_decay: float = 1e-4
    warmup_steps: int = 1000
    min_lr_ratio: float = 0.01
    gradient_clip_norm: float = 10.0
    ema_decay: float = 0.9998
    ema_ramp_steps: int = 0
    accumulation_steps: int = 1
    amp: bool = True
    seed: int = 42
    checkpoint_every_epochs: int = 1
    checkpoint_every_steps: int = 500


@dataclass(frozen=True)
class Phase3Config:
    data: DataConfig
    augmentation: AugmentationConfig = field(default_factory=AugmentationConfig)
    assignment: AssignmentConfig = field(default_factory=AssignmentConfig)
    loss: LossConfig = field(default_factory=LossConfig)
    inference: InferenceConfig = field(default_factory=InferenceConfig)
    quad: QuadConfig = field(default_factory=QuadConfig)
    schedule: ScheduleConfig = field(default_factory=ScheduleConfig)
    output_dir: Path = Path("runs/phase3")
    pretrained_backbone: bool = True
    neck_type: str = "lite"


def _construct(cls: type, values: dict[str, Any] | None):
    return cls(**(values or {}))


def _resolve(base: Path, value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (base / path).resolve()


def load_phase3_config(path: str | Path) -> Phase3Config:
    """Load YAML and resolve all paths relative to the configuration file."""
    config_path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    data_raw = dict(raw.get("data") or {})
    required = {"train_annotations", "val_annotations", "image_root", "index_dir"}
    missing = required - set(data_raw)
    if missing:
        raise ValueError(f"Phase 3 data config is missing: {sorted(missing)}")
    for key in required:
        data_raw[key] = _resolve(config_path.parent, data_raw[key])
    for key in ("quad_train_annotations", "quad_val_annotations"):
        if data_raw.get(key) is not None:
            data_raw[key] = _resolve(config_path.parent, data_raw[key])
    for key in (
        "dense_background_sources",
        "component_categories",
        "parent_categories",
    ):
        if key in data_raw:
            data_raw[key] = tuple(data_raw[key])
    assignment_raw = dict(raw.get("assignment") or {})
    for key in ("strides", "prior_sizes"):
        if key in assignment_raw:
            assignment_raw[key] = tuple(assignment_raw[key])
    config = Phase3Config(
        data=DataConfig(**data_raw),
        augmentation=_construct(AugmentationConfig, raw.get("augmentation")),
        assignment=AssignmentConfig(**assignment_raw),
        loss=_construct(LossConfig, raw.get("loss")),
        inference=_construct(InferenceConfig, raw.get("inference")),
        quad=_construct(QuadConfig, raw.get("quad")),
        schedule=_construct(ScheduleConfig, raw.get("schedule")),
        output_dir=_resolve(config_path.parent, raw.get("output_dir", "runs/phase3")),
        pretrained_backbone=bool(raw.get("pretrained_backbone", True)),
        neck_type=str(raw.get("neck_type", "lite")),
    )
    if config.data.input_size % max(config.assignment.strides) != 0:
        raise ValueError("input_size must be divisible by the largest stride")
    if config.neck_type not in {"lite", "attn_res"}:
        raise ValueError("neck_type must be 'lite' or 'attn_res'")
    if abs(sum(config.data.domain_weights.values()) - 1.0) > 1e-6:
        raise ValueError("domain_weights must sum to 1")
    if abs(sum(config.data.source_weights.values()) - 1.0) > 1e-6:
        raise ValueError("source_weights must sum to 1")
    if any(
        weight < 0 or weight > 1
        for weight in config.data.background_loss_weights.values()
    ):
        raise ValueError("background_loss_weights must be in [0, 1]")
    if config.loss.objectness_loss not in {"focal", "quality_focal"}:
        raise ValueError("objectness_loss must be 'focal' or 'quality_focal'")
    if config.loss.box_loss not in {"ciou", "giou"}:
        raise ValueError("box_loss must be 'ciou' or 'giou'")
    if config.loss.box_weighting not in {"uniform", "centerness"}:
        raise ValueError("box_weighting must be 'uniform' or 'centerness'")
    if config.loss.ltrb_weight < 0:
        raise ValueError("ltrb_weight must be non-negative")
    if config.schedule.checkpoint_every_steps < 0:
        raise ValueError("checkpoint_every_steps must be non-negative")
    if config.data.batches_per_epoch is not None and config.data.batches_per_epoch < 1:
        raise ValueError("batches_per_epoch must be positive when configured")
    if not 0 <= config.schedule.ema_decay < 1:
        raise ValueError("ema_decay must be in [0, 1)")
    if config.schedule.ema_ramp_steps < 0:
        raise ValueError("ema_ramp_steps must be non-negative")
    if config.inference.score_mode not in {
        "objectness", "objectness_x_centerness"
    }:
        raise ValueError(
            "score_mode must be 'objectness' or "
            "'objectness_x_centerness'"
        )
    if config.quad.top_k < 1 or config.quad.eligible_levels < 1:
        raise ValueError("quad top_k and eligible_levels must be positive")
    if config.quad.scale_measure not in {"area", "maximum_extent"}:
        raise ValueError("quad scale_measure must be area or maximum_extent")
    if (
        config.data.tiny_area <= 0
        or config.data.tiny_min_side <= 0
        or config.data.quad_regular_min_side <= 0
        or config.data.thin_major_axis_min <= 0
        or config.data.thin_aspect_ratio_min < 1
        or config.data.thin_area <= 0
    ):
        raise ValueError("quad size thresholds must be positive and aspect ratio must be at least 1")
    if not 0 <= config.quad.weak_negative_weight <= 1:
        raise ValueError("quad weak_negative_weight must be in [0, 1]")
    if config.quad.gwd_weight < 0:
        raise ValueError("quad gwd_weight must be non-negative")
    if config.quad.corner_smooth_l1_beta <= 0:
        raise ValueError("quad corner_smooth_l1_beta must be positive")
    if config.quad.quality_target_mode not in {"centerness", "blend", "iou"}:
        raise ValueError("quad quality_target_mode must be centerness, blend, or iou")
    if not 0 <= config.quad.quality_blend <= 1:
        raise ValueError("quad quality_blend must be in [0, 1]")
    if config.quad.geometry_quality_target not in {"exact_iou", "corner_proxy"}:
        raise ValueError(
            "quad geometry_quality_target must be exact_iou or corner_proxy"
        )
    return config
