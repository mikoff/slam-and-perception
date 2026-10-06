# Phase 5.0: Open-World SAM Object Discovery & Edge Proposal Architecture

**Status:** Approved Architectural Strategy  
**Date:** 2026-10-06  
**Target Hardware:** Raspberry Pi 5 (Cortex-A76 CPU / Edge NPU)  
**Downstream Stage:** SigLIP zero-shot open-vocabulary classifier (`google/siglip-base-patch16-224`)  
**Pilot Dataset:** 800-Image Multi-Domain Benchmark Cohort (COCO, nuImages, BDD100k, WoodScape)

---

## 1. Executive Summary & Context of the Pivot

Following the comprehensive audit documented in [`00_preliminary_studies_system_audit.md`](file:///home/sashamikoff/git/skillup/projects/visual-inference/development_stages/00_preliminary_studies_system_audit.md), the legacy closed-world proposal models (both HBB and Quad) exhibited two terminal pathologies:
1. **Severe Proposal Duplication / Stacking:** 5–10 proposals clustered on top of each other for every prominent object, caused by static 18-anchor ATSS assignment, decoupled geometric centerness targets, and artificially inflated NMS thresholds (0.78 / 0.90) tuned for benchmark AR@100.
2. **Systematic Blindness to Natural & Structural Objects:** Near-zero detection of trees, pillars, building columns, and utility poles, caused by an annotation gap in the underlying datasets (where these classes were unannotated or explicitly marked as negative background) combined with a $0.0$ weak-negative background loss weight across 50% of the training data.
3. **Small-Object Dropout:** 0.0% recall on tiny objects (<16px) and 1.6% on small objects (16–32px) due to 384x384 downsampling and the absence of a stride-4 (P2) feature pyramid level.

**The Solution:** Pivot from supervised closed-world proposal training to an **unsupervised foundation-model object discovery pipeline (SAM AMG)**, generating dual-geometry pseudo-labels (HBB + Quads) on the 800-image cohort, followed by training an edge-optimized MobileNetV4 student equipped with Task-Aligned Assignment and a P2 feature pyramid.

---

## 2. Key Architectural Decisions

### Decision 1: Teacher Selection — SAM Automatic Mask Generator (AMG)
- **Choice:** Segment Anything Model (SAM) in Automatic Mask Generator mode.
- **Rationale over Grounding DINO:** Grounding DINO requires explicit text prompt queries (e.g. `"car, tree, pillar"`). If an unexpected physical object appears (e.g., road debris, strange machinery, fallen traffic barrel), Grounding DINO will omit it. Because Stage 2 (SigLIP) handles arbitrary open-vocabulary text matching, Stage 1 must be **strictly class-agnostic**, proposing any visually distinct physical entity without semantic vocabulary bias.
- **Rationale over CutLER:** While CutLER is conceptually aligned, it depends on complex Detectron2 environments. SAM AMG is natively supported in PyTorch, has mature ViT-B/L/H checkpoints, and provides high-fidelity boundary masks.

### Decision 2: Pilot Scope — 800-Image Benchmark Cohort First
- **Choice:** Run the discovery and extraction pipeline on the frozen 800-image multi-domain benchmark cohort before scaling to full datasets (COCO 2017, nuImages, BDD100k, WoodScape).
- **Rationale:** Enables fast iteration, qualitative verification of discovered background objects (verifying that trees and pillars are cleanly segmented without grass/asphalt texture bleed), and empirical tuning of stability/area filters.

### Decision 3: Dual-Geometry Extraction (HBB + Quads)
- **Choice:** Simultaneously extract both Axis-Aligned Bounding Boxes (HBB) and Oriented Quadrilaterals (Quads) from every discovered mask and save them side-by-side in the annotation schema.
- **Rationale:** 
  - Deriving both representations from a connected binary mask $M$ is computationally negligible:
    $$\text{HBB} = [x_{\min}, y_{\min}, x_{\max}, y_{\max}]$$
    $$\text{Quad} = \text{cv2.minAreaRect}(\text{contour}) \quad \text{or} \quad \text{fit\_quad\_from\_points}(\text{contour})$$
  - Postpones the hard commitment between HBB and Quad student heads until downstream SigLIP crop trade-offs are validated.
  - Prevents needing to re-run expensive SAM inference if the geometric representation is changed later.

### Decision 4: Hybrid Supervision Contract
- **Choice:** Do not discard verified human ground truth (nuImages, BDD100k, WoodScape, COCO). Instead, run SAM discovery to find **unannotated background objects** (trees, pillars, street furniture, poles) and merge them with existing human annotations.
- **Deduplication Rule:** If a SAM-discovered mask has an IoU $> 0.50$ with an existing human ground-truth box (e.g. a car), the human annotation takes precedence to preserve clean instance boundaries. Discovered masks with IoU $\le 0.15$ with human GT are ingested as novel physical instances.

### Decision 5: Part-vs-Whole & Artifact Filtering
To prevent SAM from over-segmenting (e.g., segmenting a car into 6 separate masks: wheel, door, mirror, window, headlight, body):
- **Stability Score Threshold:** Keep masks with `stability_score >= 0.88`.
- **Predicted IoU Threshold:** Keep masks with `predicted_iou >= 0.85`.
- **Scale Floor:** Discard masks with area $< 16 \times 16\text{ px}$ (too small for reliable SigLIP feature embedding).
- **Containment Suppression:** If a smaller mask is $>85\%$ contained inside a larger mask, suppress the child mask unless it matches explicitly configured promptable parts (wheels, mirrors).

---

## 3. Edge Deployment Strategy (Raspberry Pi 5 + SigLIP)

### 3.1 The Computational Bottleneck Reality
Deploying a two-stage open-vocabulary perception pipeline on a Raspberry Pi 5 (Quad-core Cortex-A76 @ 2.4 GHz) imposes strict latency constraints:

| Stage | Model | Size / Precision | RPi 5 Execution Latency | Feasible Budget per Frame |
|---|---|---|---|---|
| **Stage 1 (Proposals)** | MobileNetV4-Conv + LiteFPN | INT8 / FP16 | $\mathbf{\approx 15 - 25\text{ ms}}$ | 1 full-frame pass ($\approx 40$ FPS) |
| **Stage 2 (Embedder)** | SigLIP (`siglip-base-patch16-224`) | INT8 / FP16 | $\mathbf{\approx 100 - 180\text{ ms}}$ **per crop** | **$K \le 5 - 10$ crops max** ($\approx 1 - 2$ FPS) |

### 3.2 Implications for Stage 1:
1. **Proposal Stacking is Fatal:** Emitting 5 redundant proposals on the same car wastes 50% of the RPi's entire per-frame compute budget on duplicate SigLIP crops.
2. **Tight Proposal Budget:** Instead of $K=100$, the production proposal budget for RPi must be **$K \le 10$ to $15$ distinct regions**.
3. **Strict Deduplication:** NMS must run at standard IoU **$0.45$–$0.50$** so that every single crop forwarded to SigLIP represents a distinct visual entity.

---

## 4. Student Detector Architectural Upgrades

To ensure the student network can learn from the new SAM-enriched dataset without repeating past failures, the student architecture incorporates the following upgrades:

```
[Input: 384x384 or 512x512 RGB]
             │
             ▼
[MobileNetV4 Backbone] ──► C2 (stride 4), C3 (stride 8), C4 (stride 16), C5 (stride 32)
             │
             ▼
[P2-LiteFPN Neck] ───────► P2 (stride 4), P3 (stride 8), P4 (stride 16)
             │
             ▼
[Task-Aligned Head] ─────► Quality logits (trained with QFL to predict box IoU)
                     ─────► Geometry offsets (LTRB for HBB / 8 signed offsets for Quad)
```

1. **Re-activation of P2 (Stride 4):**
   - Incorporating P2 ($96 \times 96$ feature map at $384 \times 384$) allows distant pedestrians and thin poles to enclose multiple grid cells, completely eliminating the assigner's `unrepresentable` dropout.
2. **Task-Aligned Assignment (TAL) & Quality Focal Loss (QFL):**
   - Dynamic anchor assignment selecting only top 3–4 anchors based on joint alignment:
     $$\text{cost} = \text{score}^\alpha \times \text{IoU}^\beta$$
   - Quality targets are set directly to the **predicted IoU** ($t = \text{IoU}$):
     $$\mathcal{L}_{\text{QFL}} = |\text{IoU} - \sigma(\text{logit})|^\beta \cdot \text{BCE}(\sigma(\text{logit}), \text{IoU})$$
   - Anchors with poor corner regressions are pushed to score $0.0$, creating a sharp single peak per object and eliminating visual stacking.
3. **Calibrated Background Loss:**
   - Reinstate non-zero negative supervision (`weak_negative_weight: 0.20`) to prevent background hallucinations.

---

## 5. Technical Data Schema for Dual-Geometry Pseudo-Labels

Discovered instances from the SAM pilot on the 800-image cohort are stored in `artifacts/phase5/pseudo_labels/sam_pilot_800.json`:

```json
{
  "image_id": "coco_2017_000000123456",
  "file_name": "000000123456.jpg",
  "width": 640,
  "height": 480,
  "annotations": [
    {
      "id": 1001,
      "source": "human_gt",
      "category": "car",
      "bbox": [120.5, 200.0, 310.0, 350.5],
      "quad": [[120.5, 200.0], [310.0, 200.0], [310.0, 350.5], [120.5, 350.5]],
      "area": 28519.75,
      "is_discovered": false
    },
    {
      "id": 2001,
      "source": "sam_discovered",
      "category": "unlabeled_object",
      "bbox": [450.0, 80.0, 520.0, 380.0],
      "quad": [[465.0, 80.0], [520.0, 85.0], [505.0, 380.0], [450.0, 375.0]],
      "area": 21000.0,
      "sam_stability_score": 0.94,
      "sam_predicted_iou": 0.91,
      "is_discovered": true
    }
  ]
}
```

---

## 6. Phased Implementation Roadmap

- [ ] **Phase 5.1: SAM Pilot Extraction Script (`scripts/run_sam_pilot_discovery.py`)**
  - Load SAM (`sam_vit_b` or `sam_vit_h`) on NVIDIA RTX 3060.
  - Process the frozen 800-image validation cohort.
  - Implement hierarchical containment and stability filtering.
  - Extract dual geometries (HBB + Quad) per mask.
- [ ] **Phase 5.2: Discovery Inspection & Visual Verification**
  - Generate an interactive HTML inspection tool showing:
    - Blue: Verified Human GT (cars, pedestrians).
    - Green: SAM Discovered Objects (trees, pillars, street furniture).
    - Red: Suppressed texture noise / part-fragments.
  - Verify that trees and pillars are cleanly captured without clutter.
- [ ] **Phase 5.3: Student Head & Loss Refactor**
  - Integrate P2 feature map into `StudentDetector` / `QuadProposalDetector`.
  - Wire Quality Focal Loss with dynamic IoU targets (`quality_target_mode: iou`).
  - Set NMS threshold to $0.50$ and evaluate candidate clustering metrics.
- [ ] **Phase 5.4: End-to-End Edge Export & SigLIP Validation**
  - Export student to ONNX / TFLite INT8 for Raspberry Pi 5.
  - Measure end-to-end latency: MobileNetV4 inference + Top-8 crop extraction + SigLIP classification.
