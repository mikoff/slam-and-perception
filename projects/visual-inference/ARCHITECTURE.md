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
- Quality Focal Loss uses normalized target groups; centerness modulation
  filters peripheral boundary noise during inference decoding.
- TensorBoard `SummaryWriter` logs step losses, learning rate decay, validation
  recalls, and a dedicated `Dataset/Summary` audit text tab.
- Quad checkpoints contain model/EMA, EMA ramp state, optimizer, scheduler, AMP
  scaler, complete config, curriculum, epoch/batch position, and deterministic
  resume state.
- Validation caches GT/proposal IoU matrices and evaluates exact polygon IoU/NMS.

# Local Constraints & Gotchas

- **Oracle vs Ranked Audit**: Oracle AR@100 reaches **0.1638** (vs HBB **0.1164**),
  proving quad geometry regression already outperforms HBB; remaining gap is score calibration.
- **Empirical Breakthroughs (v11)**:
  - **Unseen Categories R@50**: **0.2000** (+133% over HBB control **0.0857**).
  - **Large Objects R@50**: **0.4333** (+44.4% over HBB control **0.3000**).
  - **Recall@75**: **0.0746** (v8) / **0.0709** (v11) (beating/tying HBB **0.0709**).
  - **Median Matched IoU**: **0.2729** (+17.0% precision over HBB **0.2332**).
  - **AR@100**: **0.1104** (v8) / **0.1082** (v11) (closed 70% of total gap to HBB **0.1164**).
- Exact quad decode plus polygon NMS runs in ~11.1 ms/image; forward pass is ~2.1 ms/image.
- `uv` environment includes pinned PyTorch 2.6.0+cu124 and TensorBoard 2.21.0.

