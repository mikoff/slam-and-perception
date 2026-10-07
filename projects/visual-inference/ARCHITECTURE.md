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
  - **Baseline (ATSS)**: `ATSSAssigner` and `QuadAssigner` with $k=4$, center prior radius 1.0.
  - **Recipe 2 (TAL)**: `TaskAlignedAssigner` and `QuadTaskAlignedAssigner` with 1-to-1 greedy bipartite matching and auxiliary 1-to-many alignment ($t = s^\alpha \times \text{IoU}^\beta$).
- Loss Formulations:
  - **HBB (`ProposalLoss`)**: Quality Focal Loss on objectness + CIoU box loss + BCE centerness loss.
  - **Quad (`QuadProposalLoss`)**: Quality Focal Loss + Corner L1 loss + Gaussian Wasserstein Distance (GWD) loss + corner validity loss.
- Decoders & Suppression:
  - `InferenceDecoder` and `QuadInferenceDecoder` support `"hard"`, `"part_aware"` (scale ratio $\le 0.4$), `"part_aware_soft"`, and `"nms_free"`. Quad centerness is decoupled (`centerness_alpha = 0.0`).
- Pseudo-Label Storage & DataLoader:
  - Streaming transactional SQLite (`sam_discovery.sqlite`, WAL mode) via `discovery_storage.py`. Lazy per-worker SQLite reader in `SAMStudentHBBDataset` and `SAMStudentDataset`.

# Active Design Patterns & Decisions

- P2-stride feature pyramid ($s=4$) enables direct representation and localization of small objects down to 4–16 px across all pyramid levels.
- Hybrid supervision combines confirmed human GT (6,452 thing instances) with SAM novel discoveries (25,728 instances with stability >= 0.88).
- Suppressed internal subparts (28,354 instances) and non-object stuff (`road_surface`) are assigned to ignore regions rather than hard negatives.
- Post-processing decoders (`InferenceDecoder` for HBB, `QuadInferenceDecoder` for Quads) apply NMS outside the model graph.
- Anisotropic diagonal jitter (`1e-4 * I`) and LAPACK CPU fallback in eigensolvers prevent cuSOLVER `LinAlgError` on degenerate or isotropic quad boxes.
- Mixed precision: FP16 autocast with `GradScaler` on CUDA, consuming ~1.7 GB VRAM at batch size 16.

# Local Constraints & Gotchas

- Stride-8 (P3) models drop small objects (<16px); student models must maintain P2 (stride 4).
- PyTorch CUDA eigensolvers (`linalg.eigh`) fail to converge when symmetric 2x2 covariance matrices have repeated eigenvalues without jitter.
- Suppressed SAM subparts must remain in ignore masks; treating them as background penalties penalizes detector predictions of whole objects.
- `sample.original_size` is `(height, width)`. When calculating scale and unletterboxing, unpack as `orig_h, orig_w`.
- Image files are read from dataset storage at `/home/sashamikoff/git/skillup/data/visual-inference-datasets/output/`.
- Training pipeline: `scripts/train_student_pilot.py` and visualizer `scripts/visualize_student_validation.py`.
