#!/usr/bin/env python3
"""Export visual detection comparison panels (Student Predictions vs Ground Truth) for Run A."""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
import torch

from student_detector.decoder import InferenceDecoder
from student_detector.model import StudentDetector
from student_detector.sam_student_data import (
    DEFAULT_IMAGE_ROOT,
    DEFAULT_PSEUDO_LABELS_PATH,
    SAMStudentHBBDataset,
)


def draw_labeled_box(
    draw: ImageDraw.ImageDraw,
    box: list[float],
    color: tuple[int, int, int],
    label: str | None = None,
    width: int = 3,
    font: ImageFont.ImageFont | None = None,
) -> None:
    x1, y1, x2, y2 = box
    draw.rectangle([x1, y1, x2, y2], outline=color, width=width)
    if label:
        text_bbox = draw.textbbox((x1, y1), label, font=font)
        tw = text_bbox[2] - text_bbox[0]
        th = text_bbox[3] - text_bbox[1]
        ty = max(0.0, y1 - th - 4)
        draw.rectangle([x1, ty, x1 + tw + 6, ty + th + 4], fill=color)
        draw.text((x1 + 3, ty + 2), label, fill=(0, 0, 0), font=font)


def draw_labeled_item(
    draw: ImageDraw.ImageDraw,
    item: list,
    color: tuple[int, int, int],
    label: str | None = None,
    width: int = 2,
    font: ImageFont.ImageFont | None = None,
) -> None:
    if len(item) == 4 and isinstance(item[0], (int, float)):
        draw_labeled_box(draw, item, color, label=label, width=width, font=font)
    elif len(item) == 4 and isinstance(item[0], (list, tuple)):
        pts = [(float(pt[0]), float(pt[1])) for pt in item]
        draw.polygon(pts, outline=color, width=width)
        if label:
            x1, y1 = pts[0]
            text_bbox = draw.textbbox((x1, y1), label, font=font)
            tw = text_bbox[2] - text_bbox[0]
            th = text_bbox[3] - text_bbox[1]
            ty = max(0.0, y1 - th - 4)
            draw.rectangle([x1, ty, x1 + tw + 6, ty + th + 4], fill=color)
            draw.text((x1 + 3, ty + 2), label, fill=(0, 0, 0), font=font)


def create_panel_image(
    orig_img: Image.Image,
    predictions: list[dict],
    gt_boxes: list[list[float]],
    sam_novel_boxes: list[list[float]],
    metadata: dict,
    font: ImageFont.ImageFont | None,
    title_font: ImageFont.ImageFont | None,
) -> Image.Image:
    w, h = orig_img.size
    # Resize if huge to keep layout crisp (max width 960 per panel)
    target_w = min(w, 800)
    scale = target_w / w
    target_h = int(h * scale)
    base_img = orig_img.resize((target_w, target_h), Image.Resampling.LANCZOS)

    banner_h = 48
    panel_w = target_w
    panel_h = target_h + banner_h

    # Panel 1: Predictions
    p1 = Image.new("RGB", (panel_w, panel_h), (15, 23, 42))
    p1.paste(base_img, (0, banner_h))
    d1 = ImageDraw.Draw(p1)
    mode_title = metadata.get("mode_title", "PREDICTIONS")
    d1.text(
        (12, 12),
        f"{mode_title} — {len(predictions)} detections (conf ≥ 0.20)",
        fill=(56, 189, 248),
        font=title_font,
    )
    for p in predictions:
        b = p["box"]
        if len(b) == 4 and isinstance(b[0], (int, float)):
            item = [b[0] * scale, b[1] * scale + banner_h, b[2] * scale, b[3] * scale + banner_h]
        else:
            item = [[pt[0] * scale, pt[1] * scale + banner_h] for pt in b]
        score = p["score"]
        draw_labeled_item(
            d1,
            item,
            color=(56, 189, 248),
            label=f"{score:.2f}",
            width=2,
            font=font,
        )

    # Panel 2: Ground Truth & SAM
    p2 = Image.new("RGB", (panel_w, panel_h), (15, 23, 42))
    p2.paste(base_img, (0, banner_h))
    d2 = ImageDraw.Draw(p2)
    d2.text(
        (12, 12),
        f"SUPERVISION — {len(gt_boxes)} Human GT (Amber) | {len(sam_novel_boxes)} SAM Novel (Green)",
        fill=(251, 191, 36),
        font=title_font,
    )
    # Draw SAM novel first
    for b in sam_novel_boxes:
        if len(b) == 4 and isinstance(b[0], (int, float)):
            item = [b[0] * scale, b[1] * scale + banner_h, b[2] * scale, b[3] * scale + banner_h]
        else:
            item = [[pt[0] * scale, pt[1] * scale + banner_h] for pt in b]
        draw_labeled_item(d2, item, color=(52, 211, 153), width=2, font=font)
    # Draw Human GT on top
    for b in gt_boxes:
        if len(b) == 4 and isinstance(b[0], (int, float)):
            item = [b[0] * scale, b[1] * scale + banner_h, b[2] * scale, b[3] * scale + banner_h]
        else:
            item = [[pt[0] * scale, pt[1] * scale + banner_h] for pt in b]
        draw_labeled_item(d2, item, color=(251, 191, 36), width=3, font=font)

    # Composite side-by-side
    margin = 8
    total_w = panel_w * 2 + margin
    total_h = panel_h + 36
    comp = Image.new("RGB", (total_w, total_h), (11, 17, 32))

    # Top title bar
    comp_draw = ImageDraw.Draw(comp)
    header_text = (
        f"Dataset: {metadata['dataset']} | Image #{metadata['image_id']} "
        f"| Camera: {metadata['camera']} | Res: {w}x{h}"
    )
    comp_draw.text((16, 8), header_text, fill=(248, 250, 252), font=title_font)

    comp.paste(p1, (0, 36))
    comp.paste(p2, (panel_w + margin, 36))
    return comp


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=Path("artifacts/stage2/checkpoints/mobilenetv4_p2_hbb_best.pt"),
    )
    parser.add_argument(
        "--pseudo-labels",
        type=Path,
        default=Path("artifacts/phase1/pseudo_labels/sam_pilot_800.json"),
    )
    parser.add_argument(
        "--geometry",
        type=str,
        choices=["hbb", "quad"],
        default="hbb",
        help="Geometry: 'hbb' or 'quad' (default: 'hbb').",
    )
    parser.add_argument("--image-root", type=Path, default=DEFAULT_IMAGE_ROOT)
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--nms-mode",
        type=str,
        default="part_aware",
        choices=["part_aware", "nms_free", "hard"],
        help="NMS mode ('part_aware', 'nms_free', or 'hard').",
    )
    parser.add_argument("--samples-per-cohort", type=int, default=2)
    parser.add_argument("--conf-threshold", type=float, default=0.20)
    parser.add_argument(
        "--wrap-to-bbox",
        action="store_true",
        help="Wrap predicted quads into axis-aligned horizontal bounding boxes (HBB).",
    )
    parser.add_argument("--device", type=str, default="cpu")
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    device = torch.device(args.device)

    print(f"Loading checkpoint: {args.checkpoint}")
    ckpt = torch.load(args.checkpoint, map_location=device)
    strides = ckpt.get("strides", (4, 8, 16, 32))
    is_dual = ckpt.get("recipe") == "recipe2" or any(
        k.startswith("aux_head") for k in ckpt["model_state_dict"].keys()
    )

    if args.geometry == "hbb":
        model = StudentDetector(
            pretrained_backbone=False,
            neck_type="lite",
            strides=strides,
            fpn_channels=96,
            dual_head=is_dual,
        ).to(device)
        decoder = InferenceDecoder(
            strides=strides,
            top_k=1000,
            nms_iou_threshold=0.50,
            max_detections=100,
            score_mode="objectness_x_centerness",
            centerness_alpha=1.0,
            nms_mode=args.nms_mode,
            part_scale_ratio=0.4,
        )
        val_dataset = SAMStudentHBBDataset(
            pseudo_labels_path=args.pseudo_labels,
            image_root=args.image_root,
            split="val",
            input_size=384,
        )
    else:
        from student_detector.model import QuadProposalDetector
        from student_detector.quad_decoder import QuadInferenceDecoder
        from student_detector.sam_student_data import SAMStudentDataset

        model = QuadProposalDetector(
            pretrained_backbone=False,
            neck_type="lite",
            strides=strides,
            fpn_channels=96,
            dual_head=is_dual,
        ).to(device)
        decoder = QuadInferenceDecoder(
            strides=strides,
            pre_nms_top_k=1000,
            nms_iou_threshold=0.50,
            max_proposals=100,
            centerness_alpha=0.0,
            nms_mode=args.nms_mode,
            part_scale_ratio=0.4,
        )
        val_dataset = SAMStudentDataset(
            pseudo_labels_path=args.pseudo_labels,
            image_root=args.image_root,
            split="val",
            input_size=384,
        )

    model.load_state_dict(ckpt["model_state_dict"])
    model.eval()

    by_cohort: dict[str, list[int]] = {}
    for idx, img in enumerate(val_dataset.images):
        by_cohort.setdefault(img["source_dataset"], []).append(idx)

    selected_indices: list[int] = []
    for cohort_name, indices in sorted(by_cohort.items()):
        picked = indices[: args.samples_per_cohort]
        selected_indices.extend(picked)
        print(f"Cohort {cohort_name}: picked {len(picked)} images")

    font = None
    title_font = None
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 11)
        title_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 13)
    except Exception:
        font = ImageFont.load_default()
        title_font = ImageFont.load_default()

    with torch.inference_mode():
        for count, idx in enumerate(selected_indices, 1):
            sample = val_dataset[idx]
            rec = val_dataset.images[idx]
            img_path = args.image_root / rec["file_name"]
            orig_img = Image.open(img_path).convert("RGB")
            orig_h, orig_w = sample.original_size
            scale, off_x, off_y = sample.transform

            inp = sample.image.unsqueeze(0).to(device)
            output = model(inp)
            det = decoder(output, (384, 384))[0]

            predictions = []
            if args.geometry == "hbb":
                boxes = det.boxes.clone()
                scores = det.scores.clone()
                boxes[:, 0] = ((boxes[:, 0] - off_x) / scale).clamp(0, orig_w)
                boxes[:, 1] = ((boxes[:, 1] - off_y) / scale).clamp(0, orig_h)
                boxes[:, 2] = ((boxes[:, 2] - off_x) / scale).clamp(0, orig_w)
                boxes[:, 3] = ((boxes[:, 3] - off_y) / scale).clamp(0, orig_h)
                for b, s in zip(boxes, scores, strict=True):
                    score_val = float(s)
                    if score_val >= args.conf_threshold:
                        predictions.append({
                            "box": [float(b[0]), float(b[1]), float(b[2]), float(b[3])],
                            "score": score_val,
                        })
            else:
                quads = det.quads.clone()
                scores = det.scores.clone()
                for q, s in zip(quads, scores, strict=True):
                    score_val = float(s)
                    if score_val >= args.conf_threshold:
                        q_unscaled = q.clone()
                        q_unscaled[:, 0] = ((q_unscaled[:, 0] - off_x) / scale).clamp(0, orig_w)
                        q_unscaled[:, 1] = ((q_unscaled[:, 1] - off_y) / scale).clamp(0, orig_h)
                        if args.wrap_to_bbox:
                            x1 = float(q_unscaled[:, 0].min())
                            y1 = float(q_unscaled[:, 1].min())
                            x2 = float(q_unscaled[:, 0].max())
                            y2 = float(q_unscaled[:, 1].max())
                            predictions.append({
                                "box": [x1, y1, x2, y2],
                                "score": score_val,
                            })
                        else:
                            predictions.append({
                                "box": q_unscaled.tolist(),
                                "score": score_val,
                            })

            predictions.sort(key=lambda x: x["score"], reverse=True)
            predictions = predictions[:30]

            gt_boxes = []
            sam_novel = []
            geom_key = "bbox" if (args.geometry == "hbb" or args.wrap_to_bbox) else "quad"
            for ann in rec.get("annotations", []):
                item = ann.get(geom_key)
                if not item or ann.get("is_suppressed") or not ann.get("is_positive"):
                    continue
                if ann.get("source") == "human_gt":
                    gt_boxes.append(item)
                elif ann.get("source") == "sam_discovered":
                    sam_novel.append(item)

            cohort_clean = rec["source_dataset"].replace("_images_100k", "").replace("_rgb_fisheye", "")
            meta = {
                "dataset": cohort_clean.upper(),
                "image_id": rec["image_id"],
                "camera": sample.camera_type,
                "mode_title": f"{args.geometry.upper()} ({args.nms_mode.replace('_', ' ').upper()})",
            }

            panel = create_panel_image(
                orig_img,
                predictions,
                gt_boxes,
                sam_novel,
                meta,
                font,
                title_font,
            )
            out_file = args.output_dir / f"detection_sample_{count:02d}_{cohort_clean}_{rec['image_id']}.png"
            panel.save(out_file, "PNG", optimize=True)
            print(f"[{count}/{len(selected_indices)}] Saved {out_file.name} ({len(predictions)} preds, {len(gt_boxes)} GT, {len(sam_novel)} SAM)")

    print(f"\nAll detection visual comparison panels saved to {args.output_dir}")


if __name__ == "__main__":
    main()
