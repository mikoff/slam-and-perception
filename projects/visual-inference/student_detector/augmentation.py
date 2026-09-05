"""Shared deterministic image augmentation for HBB and quadrilateral targets."""

from __future__ import annotations

import io
import random
from dataclasses import asdict, dataclass
from typing import Any

import torch
from PIL import Image, ImageEnhance, ImageFilter
from torch import Tensor

from .config import AugmentationConfig


IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)
PADDING_RGB = tuple(round(channel * 255) for channel in IMAGENET_MEAN)
BLUR_RADIUS_RANGE = (0.1, 1.0)
JPEG_QUALITY_RANGE = (45, 95)
NOISE_STANDARD_DEVIATION = 0.02
EXCLUDED_OPERATIONS = (
    "rotation",
    "perspective",
    "mosaic",
    "mixup",
    "cutmix",
    "synthetic_object_insertion",
)


@dataclass(frozen=True)
class AugmentationParameters:
    """All random choices consumed by image and geometry adapters."""

    horizontal_flip: bool
    scale_multiplier: float
    shift_x: float
    shift_y: float
    brightness_factor: float
    contrast_factor: float
    saturation_factor: float
    blur_radius: float | None
    jpeg_quality: int | None
    noise_standard_deviation: float
    noise_seed: int

    @property
    def selected_operations(self) -> tuple[str, ...]:
        operations = ["letterbox"]
        if self.horizontal_flip:
            operations.append("horizontal_flip")
        if self.scale_multiplier != 1.0:
            operations.append("scale")
        if self.shift_x != 0.0 or self.shift_y != 0.0:
            operations.append("translation")
        if (
            self.brightness_factor != 1.0
            or self.contrast_factor != 1.0
            or self.saturation_factor != 1.0
        ):
            operations.append("color_jitter")
        if self.blur_radius is not None:
            operations.append("blur")
        if self.jpeg_quality is not None:
            operations.append("jpeg")
        if self.noise_standard_deviation > 0:
            operations.append("noise")
        return tuple(operations)


@dataclass(frozen=True)
class AugmentedImage:
    """Pixel result and shared affine metadata."""

    tensor: Tensor
    valid_mask: Tensor
    transform: tuple[float, float, float]
    parameters: AugmentationParameters


def sample_augmentation_parameters(
    config: AugmentationConfig,
    input_size: int,
    *,
    training: bool,
    seed: int,
) -> AugmentationParameters:
    """Sample every stochastic operation once from an image/epoch seed."""
    if not training:
        return AugmentationParameters(
            False, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, None, None, 0.0, seed
        )
    generator = random.Random(seed)
    horizontal_flip = generator.random() < config.horizontal_flip_probability
    scale_multiplier = generator.uniform(config.scale_min, config.scale_max)
    translation = config.translation_fraction * input_size
    shift_x = generator.uniform(-translation, translation)
    shift_y = generator.uniform(-translation, translation)
    if generator.random() < config.color_jitter_probability:
        brightness_factor = generator.uniform(
            1 - config.brightness, 1 + config.brightness
        )
        contrast_factor = generator.uniform(1 - config.contrast, 1 + config.contrast)
        saturation_factor = generator.uniform(
            1 - config.saturation, 1 + config.saturation
        )
    else:
        brightness_factor = contrast_factor = saturation_factor = 1.0
    blur_radius = (
        generator.uniform(*BLUR_RADIUS_RANGE)
        if generator.random() < config.blur_probability
        else None
    )
    jpeg_quality = (
        generator.randint(*JPEG_QUALITY_RANGE)
        if generator.random() < config.jpeg_probability
        else None
    )
    noise_standard_deviation = (
        NOISE_STANDARD_DEVIATION
        if generator.random() < config.noise_probability
        else 0.0
    )
    return AugmentationParameters(
        horizontal_flip,
        scale_multiplier,
        shift_x,
        shift_y,
        brightness_factor,
        contrast_factor,
        saturation_factor,
        blur_radius,
        jpeg_quality,
        noise_standard_deviation,
        seed,
    )


def apply_image_augmentation(
    image: Image.Image,
    input_size: int,
    parameters: AugmentationParameters,
) -> AugmentedImage:
    """Apply sampled image operations and return normalized FP32 CHW pixels."""
    image = image.convert("RGB")
    width, height = image.size
    if parameters.horizontal_flip:
        image = image.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
    scale = min(input_size / width, input_size / height) * parameters.scale_multiplier
    offset_x = (input_size - width * scale) * 0.5 + parameters.shift_x
    offset_y = (input_size - height * scale) * 0.5 + parameters.shift_y
    inverse = (
        1.0 / scale,
        0.0,
        -offset_x / scale,
        0.0,
        1.0 / scale,
        -offset_y / scale,
    )
    image = image.transform(
        (input_size, input_size),
        Image.Transform.AFFINE,
        inverse,
        resample=Image.Resampling.BILINEAR,
        fillcolor=PADDING_RGB,
    )
    valid = Image.new("L", (width, height), color=255).transform(
        (input_size, input_size),
        Image.Transform.AFFINE,
        inverse,
        resample=Image.Resampling.NEAREST,
        fillcolor=0,
    )
    if parameters.brightness_factor != 1.0:
        image = ImageEnhance.Brightness(image).enhance(parameters.brightness_factor)
    if parameters.contrast_factor != 1.0:
        image = ImageEnhance.Contrast(image).enhance(parameters.contrast_factor)
    if parameters.saturation_factor != 1.0:
        image = ImageEnhance.Color(image).enhance(parameters.saturation_factor)
    if parameters.blur_radius is not None:
        image = image.filter(ImageFilter.GaussianBlur(radius=parameters.blur_radius))
    if parameters.jpeg_quality is not None:
        encoded = io.BytesIO()
        image.save(encoded, format="JPEG", quality=parameters.jpeg_quality)
        encoded.seek(0)
        with Image.open(encoded) as decoded:
            image = decoded.convert("RGB")
    tensor = pil_to_normalized_tensor(image)
    if parameters.noise_standard_deviation > 0:
        noise_generator = torch.Generator().manual_seed(parameters.noise_seed)
        tensor = (
            tensor
            + torch.randn(tensor.shape, generator=noise_generator)
            * parameters.noise_standard_deviation
        )
    return AugmentedImage(
        tensor,
        mask_to_tensor(valid),
        (scale, offset_x, offset_y),
        parameters,
    )


def pil_to_normalized_tensor(image: Image.Image) -> Tensor:
    """Convert RGB PIL pixels into normalized FP32 CHW form."""
    width, height = image.size
    tensor = (
        torch.frombuffer(bytearray(image.tobytes()), dtype=torch.uint8)
        .reshape(height, width, 3)
        .permute(2, 0, 1)
        .float()
        / 255.0
    )
    mean = tensor.new_tensor(IMAGENET_MEAN).view(3, 1, 1)
    std = tensor.new_tensor(IMAGENET_STD).view(3, 1, 1)
    return (tensor - mean) / std


def mask_to_tensor(mask: Image.Image) -> Tensor:
    """Convert an L-mode valid-pixel mask into a boolean tensor."""
    width, height = mask.size
    return (
        torch.frombuffer(bytearray(mask.tobytes()), dtype=torch.uint8)
        .reshape(height, width)
        .bool()
    )


def effective_augmentation_policy(config: AugmentationConfig) -> dict[str, Any]:
    """Return requested probabilities plus fixed effective operation bounds."""
    return {
        "schema_version": "shared-augmentation.v1",
        "configured": asdict(config),
        "effective_bounds": {
            "blur_radius": BLUR_RADIUS_RANGE,
            "jpeg_quality": JPEG_QUALITY_RANGE,
            "noise_standard_deviation": NOISE_STANDARD_DEVIATION,
        },
        "geometry": "aspect-preserving affine letterbox; horizontal flip only",
        "padding_rgb": PADDING_RGB,
        "normalization": {"mean": IMAGENET_MEAN, "std": IMAGENET_STD},
        "validation": "deterministic letterbox only",
        "excluded_operations": EXCLUDED_OPERATIONS,
    }
