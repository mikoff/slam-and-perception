# Module Purpose & Boundaries

This project owns class-agnostic proposal-object contracts, SAM-based open-world object discovery, dual-geometry (HBB and Quad) student detector training, Raspberry Pi 5 edge export, and SigLIP semantic distillation. GitHub Actions dispatches dstack cloud work through the Packet bridge.

# Technical Contracts & Interfaces

- Input tensors are FP32 `[B, 3, H, W]`, with spatial dimensions divisible by 32 (default 384x384 or 512x512).
- Feature strides are P2 (stride 4), P3 (stride 8), and P4 (stride 16) to resolve small-object representability.
- Discovered pseudo-labels provide dual geometries per instance: `bbox` `[x1, y1, x2, y2]` and `quad` `[4, 2]` minimum-area oriented rectangles.
- SigLIP teacher is pinned to `google/siglip-base-patch16-224` (revision `7fd15f0689c79d79e38b1c2e2e2370a7bf2761ed`), producing unit-normalized `[N, 768]` embeddings.
- Crop extraction uses axis-aligned source crops with 15% context margin (`contextual_source`) resized to `[224, 224]`.
- Proposal ranking combines IoU-aligned quality and calibrated teacher similarity: `S_{ic} = q_i * sigmoid(exp(logit_scale) * (z_i @ w_c) + logit_bias)`.
- Student detector heads use Task-Aligned Assignment (TAL) and Quality Focal Loss (QFL) predicting actual box IoU.
- Accelerated deployment artifacts are exported INT8 PT2/ONNX graphs; post-processing decoders run outside the graph.

# Active Design Patterns & Decisions

- Unsupervised discovery uses SAM Automatic Mask Generator (AMG) on the frozen 800-image cohort to capture unannotated natural/structural objects (trees, pillars, furniture).
- Dual-geometry extraction derives both HBB and Quad representations simultaneously, postponing student architecture commitment without re-running SAM.
- Hybrid supervision preserves verified human GT (nuImages, BDD100k, WoodScape) and merges non-overlapping SAM background instances (IoU <= 0.15 with human GT).
- Part-vs-whole suppression enforces stability score >= 0.88, predicted IoU >= 0.85, and 85% containment suppression to prevent mask fracturing.
- Production proposal budget for Raspberry Pi 5 is capped at K <= 10–15 regions via standard NMS at IoU 0.45–0.50 to eliminate redundant crops.
- Category compatibility masks allow synonymous labels (e.g. `person` satisfies `pedestrian`) without forcing false negatives.

# Local Constraints & Gotchas

- SigLIP ViT-B execution on RPi 5 CPU takes 100–180ms per crop; proposal stacking or budgets >15 crops severely choke per-frame latency.
- Stride-8 (P3) models silently drop <16px objects as `unrepresentable`; student models must retain P2 (stride 4).
- Without containment suppression, SAM segments vehicle sub-components (doors, wheels, lights) into duplicate competing instances.
- Background quality loss must retain non-zero weak negative weight (0.15–0.20); zero negative weight causes score uncalibration.
- Production images are local symlinks and exceed archive scratch space; streaming pipelines must be used.
- Offline crop embeddings must bind image identity, geometry hash, and teacher revision in cache keys.
