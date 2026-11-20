#!/usr/bin/env python3
"""Run frozen SigLIP teacher open-vocabulary classification on HBB and Quad proposals.

Extracts native-resolution axis-aligned crops (15% context margin) from audited
holdout proposals and scores them against the expanded open-world vocabulary bank.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import re
import time
from typing import Any

from PIL import Image
import torch

from student_detector.crop_extractor import CropPolicy, _quad_to_bbox, extract_object_crop
from student_detector.ontology import (
    BASE_CATEGORIES,
    DISTRACTOR_CATEGORIES,
    EXTENDED_CATEGORIES,
    NOVEL_CATEGORIES,
)
from student_detector.sam_student_data import DEFAULT_IMAGE_ROOT
from student_detector.siglip import SiglipTeacher

ADDITIONAL_CATEGORIES: tuple[str, ...] = (
    "tree",
    "window",
    "door",
    "building",
    "motorcycle",
    "bicycle",
    "stairs",
)

ALL_CATEGORIES: tuple[str, ...] = (
    BASE_CATEGORIES
    + NOVEL_CATEGORIES
    + EXTENDED_CATEGORIES
    + ADDITIONAL_CATEGORIES
    + DISTRACTOR_CATEGORIES
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dashboard-html",
        type=Path,
        default=Path("artifacts/dashboard_master_comparison_8k_8k_full.html"),
        help="Input dashboard HTML containing evaluated proposals.",
    )
    parser.add_argument(
        "--image-root",
        type=Path,
        default=DEFAULT_IMAGE_ROOT,
        help="Root path containing raw dataset images.",
    )
    parser.add_argument(
        "--output-summary",
        type=Path,
        default=Path("artifacts/open_vocabulary_evaluation_summary.json"),
        help="Output JSON summary report.",
    )
    parser.add_argument(
        "--output-enriched-data",
        type=Path,
        default=Path("artifacts/open_vocabulary_images_data.json"),
        help="Output enriched images_data with SigLIP semantic labels.",
    )
    parser.add_argument(
        "--embeddings-cache",
        type=Path,
        default=Path("artifacts/open_vocabulary_crop_embeddings.pt"),
        help="Path to save or load precomputed crop embeddings.",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="Execution device for SigLIP teacher.",
    )
    parser.add_argument(
        "--score-threshold",
        type=float,
        default=0.15,
        help="Minimum proposal score to extract and classify.",
    )
    parser.add_argument(
        "--max-proposals-per-model",
        type=int,
        default=50,
        help="Maximum proposals per model per image to classify.",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=64,
        help="Batch size for SigLIP vision encoder.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    print(f"Loading dashboard HTML from {args.dashboard_html}...")
    html_content = args.dashboard_html.read_text()
    m_img = re.search(r"const IMAGES = (\[.*?\]);\s*\n", html_content, re.S)
    if not m_img:
        raise ValueError("Could not find const IMAGES in dashboard HTML")
    images_data: list[dict[str, Any]] = json.loads(m_img.group(1))
    print(f"Loaded {len(images_data)} evaluation images from dashboard.")

    print(f"Initializing SiglipTeacher on {args.device}...")
    teacher = SiglipTeacher(device=args.device)
    categories_list = list(ALL_CATEGORIES)
    distractor_set = set(DISTRACTOR_CATEGORIES)

    print(f"Building text representation bank for {len(categories_list)} categories...")
    text_bank, text_meta = teacher.build_text_bank(categories_list)
    text_bank = text_bank.to(args.device)

    crop_policy = CropPolicy(
        mode="axis_aligned",
        context_margin=0.15,
        target_size=(224, 224),
        min_pixel_size=2.0,
        use_original_resolution=True,
    )

    # Collect crops to classify
    # Tuple of: (img_idx, model_key, prop_idx, crop_pil)
    crop_tasks: list[tuple[int, str, int, Image.Image]] = []

    t0 = time.time()
    total_proposals = 0
    print("Extracting native-resolution axis-aligned crops (15% margin)...")
    for img_idx, item in enumerate(images_data):
        file_name = item["file_name"]
        img_path = args.image_root / file_name
        if not img_path.exists():
            continue

        try:
            pil_img = Image.open(img_path).convert("RGB")
        except Exception as e:
            print(f"Warning: could not open image {img_path}: {e}")
            continue

        for model_key in ("hbb_after", "quad_after"):
            preds = item.get(model_key, [])
            valid_preds = [
                (idx, p)
                for idx, p in enumerate(preds)
                if p.get("score", 0.0) >= args.score_threshold
            ][: args.max_proposals_per_model]

            for prop_idx, p in valid_preds:
                total_proposals += 1
                if "box" in p:
                    geom = p["box"]
                elif "quad" in p:
                    geom = _quad_to_bbox(p["quad"])
                else:
                    continue

                crop_res = extract_object_crop(pil_img, geom, crop_policy)
                if crop_res.is_valid:
                    crop_tasks.append((img_idx, model_key, prop_idx, crop_res.image))

    dt_extract = time.time() - t0
    print(
        f"Extracted {len(crop_tasks)} valid crops across {len(images_data)} images in {dt_extract:.2f}s."
    )

    # Batch encode image crops with SigLIP or load from cache
    if args.embeddings_cache.exists():
        print(f"Loading precomputed crop embeddings from {args.embeddings_cache}...")
        image_embeddings = torch.load(args.embeddings_cache, map_location=args.device)
        dt_encode = 0.0
    else:
        print(f"Encoding crops with SigLIP teacher (batch size {args.batch_size})...")
        all_crop_images = [task[3] for task in crop_tasks]
        all_image_embeddings: list[torch.Tensor] = []

        t1 = time.time()
        for b_start in range(0, len(all_crop_images), args.batch_size):
            b_imgs = all_crop_images[b_start : b_start + args.batch_size]
            emb = teacher.encode_images(b_imgs, batch_size=args.batch_size)
            all_image_embeddings.append(emb)
            if (b_start // args.batch_size) % 10 == 0:
                print(
                    f"  Encoded {min(b_start + args.batch_size, len(all_crop_images))}/{len(all_crop_images)} crops...",
                    flush=True,
                )

        image_embeddings = torch.cat(all_image_embeddings, dim=0).to(args.device)
        dt_encode = time.time() - t1
        print(
            f"Encoded {len(all_crop_images)} crops in {dt_encode:.2f}s ({len(all_crop_images)/max(dt_encode, 0.001):.1f} crops/sec)."
        )
        torch.save(image_embeddings.cpu(), args.embeddings_cache)
        print(f"Saved precomputed crop embeddings to {args.embeddings_cache}")

    print("Computing calibrated open-vocabulary logits, categorical softmax, and sigmoids...")
    logits, sigmoid_probs = teacher.compute_similarity(image_embeddings, text_bank)
    sigmoid_cpu = sigmoid_probs.cpu()
    softmax_cpu = torch.softmax(logits, dim=-1).cpu()

    # Distribute predictions back to images_data
    category_counts: Counter[str] = Counter()
    distractor_counts: Counter[str] = Counter()
    domain_category_counts: dict[str, Counter[str]] = defaultdict(Counter)
    model_stats = {
        "hbb_after": {"total": 0, "distractor": 0, "foreground": 0},
        "quad_after": {"total": 0, "distractor": 0, "foreground": 0},
    }

    for task_idx, (img_idx, model_key, prop_idx, _) in enumerate(crop_tasks):
        prop_softmax = softmax_cpu[task_idx]
        prop_sigmoid = sigmoid_cpu[task_idx]
        top3_indices = torch.topk(prop_softmax, k=min(3, len(categories_list))).indices.tolist()

        top_cat = categories_list[top3_indices[0]]
        # sem_prob uses categorical Softmax over the vocabulary bank
        top_prob = round(float(prop_softmax[top3_indices[0]].item()), 4)
        raw_sig = round(float(prop_sigmoid[top3_indices[0]].item()), 4)
        top3_list = [
            [categories_list[i], round(float(prop_softmax[i].item()), 4)]
            for i in top3_indices
        ]
        is_distractor = top_cat in distractor_set

        # Store in proposal dictionary
        target_pred = images_data[img_idx][model_key][prop_idx]
        target_pred["label"] = top_cat
        target_pred["sem_prob"] = top_prob
        target_pred["raw_sigmoid"] = raw_sig
        target_pred["top3"] = top3_list
        target_pred["is_distractor"] = is_distractor

        # Accumulate statistics
        src_domain = images_data[img_idx].get("source_dataset", "unknown")
        model_stats[model_key]["total"] += 1
        if is_distractor:
            model_stats[model_key]["distractor"] += 1
            distractor_counts[top_cat] += 1
        else:
            model_stats[model_key]["foreground"] += 1
            category_counts[top_cat] += 1
            domain_category_counts[src_domain][top_cat] += 1

    summary = {
        "evaluated_images": len(images_data),
        "total_crops_classified": len(crop_tasks),
        "extraction_duration_seconds": round(dt_extract, 2),
        "encoding_duration_seconds": round(dt_encode, 2),
        "throughput_fps": round(len(all_crop_images) / max(dt_encode, 0.001), 1),
        "model_statistics": model_stats,
        "foreground_category_distribution": dict(category_counts.most_common()),
        "distractor_distribution": dict(distractor_counts.most_common()),
        "domain_category_distribution": {
            d: dict(c.most_common(10)) for d, c in domain_category_counts.items()
        },
        "all_categories_audited": categories_list,
        "calibration_note": (
            "Semantic confidence scores (sem_prob and top3) use Categorical Softmax across the "
            "vocabulary bank logits. Raw SigLIP binary sigmoids (scale=117.33, bias=-12.932) require "
            "cos >= 0.1102 to reach 50% probability, rendering small/distant proposal crops near-zero "
            "on absolute binary scales while their relative categorical ranking remains accurate."
        ),
    }

    print("\n" + "=" * 60)
    print("OPEN-VOCABULARY PROPOSAL EVALUATION SUMMARY")
    print("=" * 60)
    print(f"Total evaluated images: {len(images_data)}")
    print(f"Total classified proposal crops: {len(crop_tasks)}")
    print(f"Throughput: {summary['throughput_fps']} crops/second on {args.device}")
    print(f"\nHBB Proposals Classified: {model_stats['hbb_after']['total']}")
    print(
        f"  Foreground: {model_stats['hbb_after']['foreground']} ({model_stats['hbb_after']['foreground']/max(model_stats['hbb_after']['total'], 1)*100:.1f}%)"
    )
    print(
        f"  Distractor: {model_stats['hbb_after']['distractor']} ({model_stats['hbb_after']['distractor']/max(model_stats['hbb_after']['total'], 1)*100:.1f}%)"
    )
    print(f"\nQuad Proposals Classified: {model_stats['quad_after']['total']}")
    print(
        f"  Foreground: {model_stats['quad_after']['foreground']} ({model_stats['quad_after']['foreground']/max(model_stats['quad_after']['total'], 1)*100:.1f}%)"
    )
    print(
        f"  Distractor: {model_stats['quad_after']['distractor']} ({model_stats['quad_after']['distractor']/max(model_stats['quad_after']['total'], 1)*100:.1f}%)"
    )
    print("\nTop 15 Discovered Foreground Classes:")
    for cat, count in category_counts.most_common(15):
        print(f"  {cat:20s}: {count:5d}")
    print("\nBackground / Distractor Breakdown:")
    for cat, count in distractor_counts.most_common():
        print(f"  {cat:20s}: {count:5d}")
    print("=" * 60)

    args.output_summary.parent.mkdir(parents=True, exist_ok=True)
    args.output_summary.write_text(json.dumps(summary, indent=2))
    print(f"Wrote evaluation summary to {args.output_summary}")

    args.output_enriched_data.write_text(json.dumps(images_data))
    print(f"Wrote enriched images data to {args.output_enriched_data}")


if __name__ == "__main__":
    main()
