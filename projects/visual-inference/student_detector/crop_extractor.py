"""Object crop extraction and preprocessing for SigLIP teacher baselines."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from PIL import Image
import torch
from torch import Tensor
import torchvision.transforms.functional as TF

from .rectification import extract_rectified_patches

CropMode = Literal["axis_aligned", "rectified"]


@dataclass(frozen=True)
class CropPolicy:
    """Immutable policy definition for object crop extraction."""

    mode: CropMode = "axis_aligned"
    context_margin: float = 0.15
    target_size: tuple[int, int] = (224, 224)
    min_pixel_size: float = 2.0
    use_original_resolution: bool = True


@dataclass(frozen=True)
class ExtractedCrop:
    """Extracted crop result and its validity metadata."""

    image: Image.Image
    is_valid: bool
    bounding_box: tuple[float, float, float, float]
    original_size: tuple[int, int]
    context_margin: float
    crop_mode: CropMode


def _quad_to_bbox(quad: Tensor | list[list[float]] | tuple) -> tuple[float, float, float, float]:
    """Extract bounding box (x1, y1, x2, y2) enclosing quad points."""
    if isinstance(quad, Tensor):
        pts = quad.detach().cpu().reshape(-1, 2)
        x1 = float(pts[:, 0].min().item())
        y1 = float(pts[:, 1].min().item())
        x2 = float(pts[:, 0].max().item())
        y2 = float(pts[:, 1].max().item())
    else:
        pts = list(quad)
        x1 = min(float(p[0]) for p in pts)
        y1 = min(float(p[1]) for p in pts)
        x2 = max(float(p[0]) for p in pts)
        y2 = max(float(p[1]) for p in pts)
    return x1, y1, x2, y2


def extract_object_crop(
    image: Image.Image,
    geometry: Tensor | list[list[float]] | tuple[float, float, float, float],
    policy: CropPolicy,
) -> ExtractedCrop:
    """Extract one object crop from an image according to CropPolicy.

    /// @brief Extracts a crop from a PIL image given a quad or bbox geometry.
    /// @param image Source PIL Image.
    /// @param geometry Quad [[x1,y1],...] or bbox (x1, y1, x2, y2).
    /// @param policy CropPolicy specifying mode, margin, and resolution.
    /// @return ExtractedCrop containing PIL Image (resized to target_size) and validity.
    """
    img_w, img_h = image.size
    target_w, target_h = policy.target_size

    # Check if geometry is quad or bbox
    is_bbox = False
    if isinstance(geometry, (tuple, list)) and len(geometry) == 4 and not isinstance(geometry[0], (list, tuple)):
        is_bbox = True
        x1, y1, x2, y2 = (float(geometry[0]), float(geometry[1]), float(geometry[2]), float(geometry[3]))
    else:
        x1, y1, x2, y2 = _quad_to_bbox(geometry)

    box_w = max(x2 - x1, 0.0)
    box_h = max(y2 - y1, 0.0)

    if box_w < policy.min_pixel_size or box_h < policy.min_pixel_size:
        # Degenerate crop: return a blank target_size image marked invalid
        blank = Image.new("RGB", (target_w, target_h), color=(0, 0, 0))
        return ExtractedCrop(
            image=blank,
            is_valid=False,
            bounding_box=(x1, y1, x2, y2),
            original_size=(img_w, img_h),
            context_margin=policy.context_margin,
            crop_mode=policy.mode,
        )

    if policy.mode == "rectified" and not is_bbox:
        # Projective rectification
        quad_t = torch.tensor(geometry, dtype=torch.float32).reshape(1, 4, 2)
        img_t = TF.to_tensor(image.convert("RGB"))  # [3, H, W]
        try:
            patch = extract_rectified_patches(
                img_t,
                quad_t,
                output_size=policy.target_size,
            )
            # patch is [1, 3, target_h, target_w]
            patch_img = TF.to_pil_image(patch.squeeze(0).clamp(0, 1))
            return ExtractedCrop(
                image=patch_img,
                is_valid=True,
                bounding_box=(x1, y1, x2, y2),
                original_size=(img_w, img_h),
                context_margin=0.0,
                crop_mode="rectified",
            )
        except Exception:
            # Fall back to axis_aligned if homography solve fails
            pass

    # Axis aligned crop with context margin
    pad_w = box_w * policy.context_margin
    pad_h = box_h * policy.context_margin

    cx1 = max(0.0, x1 - pad_w)
    cy1 = max(0.0, y1 - pad_h)
    cx2 = min(float(img_w), x2 + pad_w)
    cy2 = min(float(img_h), y2 + pad_h)

    if (cx2 - cx1) < policy.min_pixel_size or (cy2 - cy1) < policy.min_pixel_size:
        blank = Image.new("RGB", (target_w, target_h), color=(0, 0, 0))
        return ExtractedCrop(
            image=blank,
            is_valid=False,
            bounding_box=(cx1, cy1, cx2, cy2),
            original_size=(img_w, img_h),
            context_margin=policy.context_margin,
            crop_mode="axis_aligned",
        )

    crop = image.crop((int(cx1), int(cy1), int(cx2), int(cy2)))
    resized_crop = crop.resize((target_w, target_h), Image.Resampling.BILINEAR)

    return ExtractedCrop(
        image=resized_crop,
        is_valid=True,
        bounding_box=(cx1, cy1, cx2, cy2),
        original_size=(img_w, img_h),
        context_margin=policy.context_margin,
        crop_mode="axis_aligned",
    )


__all__ = [
    "CropMode",
    "CropPolicy",
    "ExtractedCrop",
    "extract_object_crop",
]
