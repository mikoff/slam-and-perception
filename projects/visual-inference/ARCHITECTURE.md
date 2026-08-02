# Module Purpose & Boundaries

This project trains proposal detectors that answer where objects are. The HBB
path remains a comparison control; the quad path is class-agnostic and has no
SigLIP, prompt, or semantic classification dependency.

The exported neural network ends at dense P3-P5 outputs. Quad decoding,
canonicalization, top-K, exact polygon IoU/NMS, and metrics run outside it.

# Technical Contracts & Interfaces

- Input images are normalized FP32 tensors `[B, 3, H, W]`; `H` and `W` are
  divisible by 32 and production configuration currently uses 384 x 384.
- MobileNetV4 and LiteFPN emit P3/P4/P5 `[B, 96, H/s, W/s]` at strides 8/16/32.
- `QuadDetectorOutput.quality` has one logit per location; `corner_offsets` has
  eight unmasked signed, stride-normalized values per location.
- Training targets are `[B, S]` quality/masks/GT-owner indices and `[B, S, 8]`
  corner offsets, where `S` is the flattened sum of P3-P5 locations.
- Dataset samples carry positive `[N, 4, 2]` and ignore `[M, 4, 2]` quads plus
  a pixel-validity mask, geometry tiers, and object-condition labels. Compact
  manifests are indexed through SQLite schema version 4.
- The decoder returns canonical `[K, 4, 2]` quads and `[K]` quality scores,
  with at most 100 deployed proposals after reference polygon NMS.

# Active Design Patterns & Decisions

- Static assignment uses full-quad inclusion, area centroid, PCA local axes,
  normalized elliptical center distance, and smooth FPN scale compatibility.
- Scale compatibility selects locations only. Rotated centerness supervises
  proposal quality before transition to detached geometry quality.
- Direct-corner Smooth-L1 minimizes over four cyclic starts and both windings;
  predicted corners are never sorted inside the training graph.
- Validity loss is computed in stride-normalized coordinates and penalizes
  collapsed edges, low area, bow-ties, and inconsistent convex turns.
- Quality Focal Loss is normalized independently for positive, trusted
  background, and weak/unlabeled groups; ignore locations contribute zero.
- CUDA AMP is supported. Pretrained backbone BatchNorm statistics are frozen;
  randomly initialized backbones keep BatchNorm trainable to avoid FP16 overflow.
- Micro-overfit mode disables augmentation and EMA validation, uses fixed image
  IDs, repeats every fixture image per optimizer batch, and stores curriculum
  state in checkpoints.
- Quad checkpoints contain model/EMA, optimizer, scheduler, AMP scaler, complete
  config, curriculum, epoch/batch position, and support deterministic resume.
- Validation caches each GT/proposal IoU matrix. It logs assigned and final
  recall, corner error, matched score, NMS delta, slice recall, and score groups;
  the expensive all-dense oracle runs only in final gate reports.

# Local Constraints & Gotchas

- G5 selected the aspect-aware cyclic/reversed corner-quality proxy for training
  score targets: exact IoU cost 502.8% extra median step time and 0% extra memory
  on the RTX 3060. Exact polygon IoU remains mandatory for validation.
- Reference polygon NMS copies candidates to CPU and is correctness-first; time
  neural inference and NMS separately. It stops once the requested ranked output
  budget is full because later lower-score candidates cannot enter that prefix.
- The primary regular target floor is a 16-pixel shortest side after resizing;
  thin objects use the explicit major-axis/aspect/area exception.
- `uv` CUDA works only with host GPU device access in the managed sandbox. The
  verified runtime is PyTorch 2.6.0+cu124 on an RTX 3060 with a CUDA 13 driver.
- The selected proxy and deterministic six-positive G6 fixture pass R@100 at
  IoU .50/.75 from saved step-150 `best.pt`. Later checkpoints can drift on the
  thin .75 boundary, so AR-based selection is required; G0/G1 still block.
