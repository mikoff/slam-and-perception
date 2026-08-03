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
- Dataset samples carry positive, ignore, and explicit trusted-background
  quads plus a pixel-validity mask and evaluation slice labels. Compact
  manifests are indexed through SQLite schema version 6.
- The decoder returns canonical `[K, 4, 2]` quads and `[K]` quality scores,
  with at most 100 deployed proposals after reference polygon NMS.

# Active Design Patterns & Decisions

- Static assignment uses full-quad inclusion, area centroid, PCA local axes,
  normalized elliptical center distance, and smooth FPN scale compatibility.
- Scale compatibility selects locations only. Rotated centerness supervises
  proposal quality before transition to detached geometry quality.
- Direct-corner Smooth-L1 minimizes over four cyclic starts and both windings;
  predicted corners are never sorted inside the training graph. Its selected
  beta is 1.0; GWD remains an optional zero-cost auxiliary with selected weight 0.
- Validity loss is computed in stride-normalized coordinates and penalizes
  collapsed edges, low area, bow-ties, and inconsistent convex turns.
- Quality Focal Loss is normalized independently for positive, trusted
  background, and weak/unlabeled groups; ignore locations contribute zero.
- Trusted-background point inclusion broadcasts over every tile and grid point;
  per-tile GPU kernel launches are prohibited by the throughput contract.
- CUDA AMP is supported. Pretrained backbone BatchNorm statistics are frozen;
  randomly initialized backbones keep BatchNorm trainable to avoid FP16 overflow.
- Reproducibility is seeded before dataset, sampler, and model construction;
  seeding only inside the training loop is too late for random head weights.
- Production quad loaders keep workers persistent. Their epoch counter is a
  shared tensor so deterministic augmentation still changes between epochs.
- Fully in-frame transformed quads are canonicalized as one batch. Only
  boundary-crossing quads enter the slower clipping/fitting loop.
- Micro-overfit mode disables augmentation and EMA validation, uses fixed image
  IDs, repeats every fixture image per optimizer batch, and stores curriculum
  state in checkpoints.
- Quad checkpoints contain model/EMA, EMA ramp state, optimizer, scheduler, AMP
  scaler, complete config, curriculum, epoch/batch position, and deterministic
  resume state.
- Validation caches each GT/proposal IoU matrix. It logs assigned and final
  recall, corner error, matched score, NMS delta, slice recall, and score groups;
  the expensive all-dense oracle runs only in final gate reports.
- Open-world comparison uses one quad validation loader and one exact polygon
  metric implementation for HBB envelopes and direct quad proposals.

# Local Constraints & Gotchas

- Exact aligned quad IoU and pairwise NMS use parity-tested vectorized candidate
  intersections; fixed-size CUDA overlap batches are compiled after warm-up.
- G5 cost now passes for exact-IoU score targets (315.74 vs 313.27 ms/step,
  +0.79%, equal 1052.6 MiB), but the matched learning trial regressed. The
  selected score target therefore remains the aspect-aware corner proxy.
- Exact quad decode plus polygon NMS is about 13.8 ms/image on the RTX 3060,
  down from seconds/image; neural forward is about 2.5 ms/image.
- The primary regular target floor is a 16-pixel shortest side after resizing;
  thin objects use the explicit major-axis/aspect/area exception.
- `uv` CUDA works only with host GPU device access in the managed sandbox. The
  verified runtime is PyTorch 2.6.0+cu124 on an RTX 3060 with a CUDA 13 driver.
- EMA uses a 0.9962930409 long-run cap with a 200-update exponential ramp. This
  removes the former 0.9998 initialization domination and is checkpointed.
- The seeded six-positive G6 baseline passes R@100 at IoU .50 and .75. GWD
  weights 0.25/0.05 and Smooth-L1 beta 0.1 fail the thin fitted-quad case and
  are rejected.
- G1 and G6 pass. Deterministic G7 still fails: raw quad AR/R50/R75 are
  0.0970/0.2537/0.0597 versus HBB 0.1164/0.3022/0.0709. HBB remains active.
