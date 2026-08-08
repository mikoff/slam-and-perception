# Module Purpose & Boundaries

This project trains proposal detectors that answer where objects are. The HBB
path remains a comparison control; the quad path is class-agnostic and has no
SigLIP, prompt, or semantic classification dependency.

The exported neural network ends at dense P3-P5 outputs. Quad decoding,
canonicalization, top-K, exact polygon IoU/NMS, and metrics run outside it.

# Technical Contracts & Interfaces

- Input images are normalized FP32 tensors `[B, 3, H, W]`; `H` and `W` are
  divisible by 32 and production configuration currently uses 384 x 384.
- MobileNetV4 with `LiteFPN` or `AttnResLiteFPN` emits P3/P4/P5 `[B, 96, H/s, W/s]` at strides 8/16/32.
- `AttnResLiteFPN` (`neck_type: "attn_res"`) calculates dynamic per-pixel Softmax depth weights ($\alpha_{l,3}, \alpha_{l,4}, \alpha_{l,5}$) via $1\times1$ Conv projection across resized lateral maps.
- `SharedQuadProposalHead` initializes `corner_offsets.bias` to explicit HBB
  grid prior `[-1.0, -1.0, +1.0, -1.0, +1.0, +1.0, -1.0, +1.0]` (in stride units)
  with zero weight initialization to eliminate Step 1 zero-area box collapse.
- Stride-normalized targets ($t_i = \frac{P_i - G}{s_l}$) handle level scaling
  statically; `self.scales` parameters are removed to prevent AdamW distortion.
- `decode_dense_quad_output` applies 0-cost Centerness Score Modulation:
  $S_{\text{final}} = S_{\text{quality}}^{0.7} \times S_{\text{grid\_centerness}}^{0.3}$.
- The decoder returns canonical `[K, 4, 2]` quads and `[K]` quality scores,
  with at most 100 deployed proposals after reference polygon NMS.

# Active Design Patterns & Decisions

- Static assignment uses full-quad inclusion, area centroid, PCA local axes,
  normalized elliptical center distance, and scale compatibility (`scale_sigma: 0.75`, `eligible_levels: 2`).
- Direct-corner Smooth-L1 ($\beta=1.0$) minimizes over four cyclic starts;
  GT diagonal ($d_{\text{GT}}$) loss normalization balances multi-scale gradients.
- AttnRes dynamic depth weighting prevents feature dilution between deep semantic maps ($P_5$) and high-resolution spatial boundary maps ($P_3, P_4$).
- Quality Focal Loss uses normalized target groups; centerness modulation
  filters peripheral boundary noise during inference decoding.
- TensorBoard `SummaryWriter` logs step losses, learning rate decay, validation
  recalls, and a dedicated `Dataset/Summary` audit text tab.
- Quad checkpoints contain model/EMA, EMA ramp state, optimizer, scheduler, AMP
  scaler, complete config, curriculum, epoch/batch position, and deterministic
  resume state.
- Lean cloud training pipeline (RunPod / Packet.ai + GHA) stages datasets from S3 to NVMe via `aws s3 sync`, loads annotation index into memory to avoid SQLite worker locks, logs to W&B, executes via robust Python process daemon (`scripts/cloud/run_cloud_daemon.py`) with signal traps and periodic S3 sync, and guarantees node self-termination upon completion.
- Validation caches GT/proposal IoU matrices under `@torch.no_grad()`, streams real-time evaluation math progress logs to prevent watchdog timeouts, and writes atomic checkpoints (`last.pt.tmp` -> `last.pt`).

# Local Constraints & Gotchas

- **Oracle vs Ranked Audit**: Oracle AR@100 reaches **0.1638** (vs HBB **0.1164**),
  proving quad geometry regression already outperforms HBB; remaining gap is score calibration.
- **Empirical Breakthroughs (v12 AttnRes)**:
  - **Large Objects R@50**: **0.6333** (+46.2% over baseline quad v11 **0.4333** / +111% over HBB **0.3000**).
  - **Large Objects R@75**: **0.2333** (+133.3% over baseline quad v11 **0.1000** / +250% over HBB **0.0667**).
  - **Unseen Categories R@50**: **0.2000** (+133% over HBB control **0.0857**).
  - **Overall Recall@50**: **0.2948** (beating baseline quad v11 **0.2910**).
  - **Recall@75**: **0.0709** (tied with HBB / baseline quad v11).
  - **Median Matched IoU**: **0.2729** (+17.0% precision over HBB **0.2332**).
- Exact quad decode plus polygon NMS runs in ~11.2 ms/image; forward pass is ~3.09 ms/image on GPU.
- `uv` environment includes pinned PyTorch 2.6.0+cu124 and TensorBoard 2.21.0.
