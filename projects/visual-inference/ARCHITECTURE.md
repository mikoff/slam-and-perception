# Module Purpose & Boundaries

This project owns class-agnostic proposal-object contracts, SAM-based open-world object discovery, MobileNetV4 edge student detector training (HBB and Quad geometries), and Raspberry Pi 5 edge export.

# Technical Contracts & Interfaces

- Input tensors: FP32 `[B, 3, H, W]`, default resolution 384x384 (divisible by 32).
- Backbone: `MobileNetV4Backbone` (`mobilenetv4_conv_medium.e500_r256_in1k`) exposing C2 (48ch), C3 (80ch), C4 (160ch), and C5 (256ch).
- Neck: `P2LiteFPN` outputs 4 pyramid levels `[B, 96, H / s, W / s]` for strides $s \in (4, 8, 16, 32)$.
- Head Architectures:
  - **HBB Student Detector (`StudentDetector`)**: `SharedDetectionHead` predicting objectness `[B, 1, H_l, W_l]`, FCOS-style distances $(l, t, r, b)$ `[B, 4, H_l, W_l]`, and centerness `[B, 1, H_l, W_l]`. Supports `dual_head=True` (primary 1-to-1 head + auxiliary 1-to-many TAL head).
  - **Quad Detector (`QuadProposalDetector`)**: `SharedQuadProposalHead` predicting quality logit `[B, 1, H_l, W_l]` and 8 corner vertex offsets `[B, 8, H_l, W_l]`. Supports `dual_head=True` and learnable level scales.
- Target Builders & Assigners:
  - **Baseline (ATSS)**: `ATSSAssigner` (with aspect-adapted priors, interior-first candidate selection, $\mu + 0.5\sigma$ multi-level routing, and stride-smoothed centerness) and `QuadAssigner` ($k=4$, `scale_measure="area"`, `eligible_levels=2`) to preserve P3 medium-object recall while routing tall/slender pillars and columns across P4–P5.
  - **Recipe 2 (TAL)**: `TaskAlignedAssigner` and `QuadTaskAlignedAssigner` with 1-to-1 greedy bipartite matching and auxiliary 1-to-many alignment ($t = s^\alpha \times \text{IoU}^\beta$). Features **Scale-Gated FPN Specialization** enforcing canonical receptive field scale intervals (P2: $[0, 64]$ px, P3: $[32, 128]$ px, P4: $[64, 256]$ px, P5: $[128, \infty)$ px) to prevent macro-scene competition from overriding fine instance heads.
  - **Topological Container Suppression**: `suppress_container_boxes` and `suppress_container_quads` dynamically detect composite container proposals (enclosing $\ge 2$ children or 1 child with area $>15\%$ and $\ge 8\times$ child area) and downgrade them to ignore regions (`w=0`) rather than foreground targets.
  - **TargetBuilder Interior Guard**: Restricts the non-matched ignore mask to ground-truth objects $< 95\%$ image area (`TargetBuilder` for 1-to-1 TAL), ensuring close-up foreground objects do not zero out background supervision across the entire image.
- Loss Formulations:
  - **HBB (`ProposalLoss`)**: Quality Focal Loss on objectness (`num_positives` with `counts / 16` background floor to prevent sparse-GT window/pillar suppression) + CIoU box loss + per-axis span-weighted Smooth-$\mathcal{L}_1$ LTRB loss + BCE centerness loss.
  - **Quad (`QuadProposalLoss`)**: Quality Focal Loss (with `0.5 * corner_quality_proxy` floor on `exact_iou`) + per-axis span-weighted Corner L1 loss + per-axis variance-normalized Gaussian Wasserstein Distance (GWD) loss + corner validity loss.
- Training Pipelines & Automation:
  - **Stage 1 (Self-Supervised Discovery Pre-Training)**: Trained on 10,000 balanced multi-domain images using SAM pseudo-labels (`SAMStudentHBBDataset`). High recall baseline across broad object candidates. Run script: `scripts/train_student_pilot.py`.
  - **Stage 2 (Canonical Ground-Truth Fine-Tuning)**: Fine-tunes Stage 1 checkpoints on verified human GT from `quad_train_enriched.sqlite` (`HumanGTHBBDataset` for HBB, `HumanGTQuadDataset` for Quads). HBB uses CIoU (`box_weight=2.0`) + Smooth-L1 (`ltrb_weight=0.5`) with `num_positives` normalization and `centerness_alpha=0.25`. Quad uses GWD (`gwd_weight=2.0`) + Min-over-8 Corner Smooth-$\mathcal{L}_1$ (`corner_weight=2.0`). Both employ `weak_negative_weight=0.15` and continuous QFL IoU calibration ($\alpha=0.0$ for quads). Unified script: `scripts/finetune_student_gt.py --geometry {hbb, quad}`.
  - **Unified Pipeline Automation**: `scripts/run_two_stage_pipeline.sh` orchestrates Stage 1, Stage 2, evaluation, and dashboard generation across geometries in a single entry point.
- Decoders & Suppression:
  - `InferenceDecoder` (HBB) and `QuadInferenceDecoder` (Quads, with boundary coordinate clamping prior to `quad_validity`) support `"hard"`, `"part_aware"` (scale ratio $\le 0.4$, max duplicate IoU threshold `part_max_iou=0.65`), `"part_aware_soft"`, and `"nms_free"`.
  - Centerness scoring parameter `centerness_alpha` ($\in [0.0, 1.0]$) blends scores as $\text{score} = \sigma(\text{obj}) \times \sigma(\text{center})^\alpha$. For Stage-2 HBB, $\alpha=0.25$ balances centerness with QFL IoU calibration; for Quads, centerness is decoupled ($\alpha = 0.0$) to avoid aspect-ratio distortion penalties.
  - Stage 2 Quad Fine-Tuning on 14k Enriched curriculum achieved **72.47% GT Recall@100@50**, **42.53% GT Rec@10@50** (+14.32% jump), **38.45% AP@50**, and **36.93% Rec@100@75** (up from 19.98% at 10k). Checkpoint: `artifacts/finetune_14k_quad/checkpoints/mobilenetv4_p2_quad_finetune_best.pt`.
  - Stage 2 HBB Fine-Tuning on 14k Enriched reached **57.13% GT Recall@100@50** (+2.75% over 10k v2), **35.96% Rec@10@50** (+6.84%), and **36.69% AP@50** (+14.00% gain). Checkpoint: `artifacts/finetune_14k_hbb/checkpoints/mobilenetv4_p2_hbb_finetune_best.pt`.
  - Qualitative visualization defaults to **Part-Aware NMS (IoU threshold 0.45, duplicate limit 0.65)** to suppress adjacent anchor cluster duplicates from 1-to-1 ignore mask halos.
- Pseudo-Label Storage & DataLoader:
  - Streaming transactional SQLite (`sam_discovery.sqlite`, WAL mode) via `discovery_storage.py`. Lazy per-worker SQLite reader in `SAMStudentHBBDataset` and `SAMStudentDataset`. Full-scale dataset discovery (267.8k images) orchestrated with start/pause/resume daemon via `scripts/manage_sam_discovery.sh`.
  - Authoritative human GT datasets: `HumanGTHBBDataset` and `HumanGTQuadDataset` in `student_detector/human_gt_data.py`. Enriched with full ADE20K (16.8k img: 152.1k things/agents/structure) and balanced Open Images V7 (15.0k img: 41.3k vehicles & infrastructure assets), with rooftop PKLot purged (299.6k total images, 4.10M human GT annotations) in `quad_train_enriched.sqlite` and synchronized with `sam_discovery_enriched.sqlite`.

# Active Design Patterns & Decisions

- **Two-Stage Training Paradigm (OWLv2 Analogy)**: Stage 1 trains broad visual representation on diverse SAM pseudo-labels; Stage 2 sharpens bounding envelopes and calibrates confidence scores exclusively on human ground truth.
- **Envelope Tightening Loss**: Joint Smooth-$\mathcal{L}_1$ (`ltrb_weight=0.5`) + CIoU (`box_weight=2.0`) directly penalizes loose boundaries, halving coordinate loss ($0.3557 \to 0.1757$).
- P2-stride feature pyramid ($s=4$) enables direct representation and localization of small objects down to 4–16 px across all pyramid levels.
- Hybrid supervision combines confirmed human GT (6,452 thing instances) with SAM novel discoveries (25,728 instances with stability >= 0.88).
- Suppressed internal subparts (28,354 instances) and non-object stuff (`road_surface`) are assigned to ignore regions rather than hard negatives.
- Post-processing decoders (`InferenceDecoder` for HBB, `QuadInferenceDecoder` for Quads) apply NMS outside the model graph.
- Longest-edge orthonormal derivation for quad axes and scales eliminates `torch.linalg.eigh` eigensolver instability on square or near-square quads with repeated eigenvalues.
- Mixed precision: FP16 autocast with `GradScaler` on CUDA, consuming ~3.48 GB VRAM at batch size 32.
- Vectorized Target Building: `ATSSAssigner`, `TargetBuilder`, `TaskAlignedAssigner`, and `QuadTaskAlignedAssigner` are fully tensorized, avoiding CPU-GPU sync stalls ($9.5\times$ TargetBuilder acceleration).
- Multi-worker multiprocessing: Uses `torch.multiprocessing.set_sharing_strategy("file_system")` to prevent Linux IPC file descriptor exhaustion with SQLite multi-worker DataLoaders.

# Local Constraints & Gotchas

- **Letterbox Invariants & Level Masks**: Images are transformed into a square $384 \times 384$ canvas with aspect-preserving letterboxing (`sample.transform = (scale, off_x, off_y)`). When running inference, `level_masks` sampled from `sample.valid_mask` via `point_validity_from_pixel_mask` MUST be passed to `InferenceDecoder` so padding cells are masked to $-\infty$. When projecting boxes back to original image space, coordinates must be un-letterboxed: `boxes[:, 0::2] = ((boxes[:, 0::2] - off_x) / scale).clamp(0, orig_w)` and `boxes[:, 1::2] = ((boxes[:, 1::2] - off_y) / scale).clamp(0, orig_h)`. Never multiply letterboxed coordinates by crude aspect ratios.
- Stride-8 (P3) models drop small objects (<16px); student models must maintain P2 (stride 4).
- Non-finite loss values trigger immediate training abort via fail-fast `torch.isfinite` check before backward pass.
- Suppressed SAM subparts must remain in ignore masks; treating them as background penalties penalizes detector predictions of whole objects.
- `sample.original_size` is `(height, width)`. When calculating scale and unletterboxing, unpack as `orig_h, orig_w`.
- Image files are read from dataset storage at `/home/sashamikoff/git/skillup/data/visual-inference-datasets/output/`.
- Training pipelines: `scripts/train_student_pilot.py` (Stage 1), `scripts/finetune_student_gt.py` (Stage 2 HBB & Quad), and comparative visualizers `scripts/visualize_student_comparison.py` (Master 4-model dashboard), `scripts/visualize_v1_vs_v2_comparison.py` (v1 vs v2 parity dashboard), & `scripts/visualize_student_validation.py`.
