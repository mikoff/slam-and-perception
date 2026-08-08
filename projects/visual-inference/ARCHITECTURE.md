# Module Purpose & Boundaries

This project trains proposal detectors that answer where objects are. The HBB
path remains a comparison control; the quad path is class-agnostic and has no
SigLIP, prompt, or semantic classification dependency.

The exported neural network ends at dense P3-P5 outputs. Quad decoding,
canonicalization, top-K, exact polygon IoU/NMS, and metrics run outside it.

# Technical Contracts & Interfaces

- Input images are normalized FP32 tensors `[B, 3, H, W]`; `H` and `W` are
  divisible by 32 and production configuration currently uses 384 x 384.
- MobileNetV4 with exactly one of `LiteFPN` (`neck_type: "lite"`) or
  `AttnResLiteFPN` (`"attn_res"`) emits P3/P4/P5 `[B, 96, H/s, W/s]` at
  strides 8/16/32; checkpoints carry a neck-specific architecture ID.
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
- AttnRes performs learned per-pixel depth selection; LiteFPN remains the
  required control until paired multi-seed evidence establishes a winner.
- Quality Focal Loss uses normalized target groups; centerness modulation
  filters peripheral boundary noise during inference decoding.
- Accelerate owns device placement, FP16 scaling, gradient accumulation, and
  distributed synchronization. Global batches are split across GPUs so GPU
  count does not change benchmark optimizer steps or effective batch size.
- `training_runtime` owns the lifecycle, `training_optimization` owns optimizer/
  EMA state, and `training_reporting` isolates external side effects; small HBB
  and quad adapters retain target, loss, decode, validation, and selection policy.
- Validation records raw and EMA metrics independently; `best_raw.pt`,
  `best_ema.pt`, and the backward-compatible raw `best.pt` are distinct.
- Every checkpoint declares `selected_state` (`model` or `ema_model`), so
  evaluators never infer which stored weights a filename is meant to deploy.
- Quad checkpoints contain model/EMA, EMA ramp state, optimizer, scheduler, AMP
  scaler, complete config, curriculum, epoch/batch position, and deterministic
  resume state.
- Checkpoint loads infer legacy necks from state keys, validate phase/neck, and
  use strict tensor loading; silent partial architecture loads are forbidden.
- Every run writes `run_contract.json` with resolved config, manifest/lock
  hashes, Git state, runtime versions, model signature, and optimizer-step count.
- `configs/benchmarks/phase3_bounded_v1.*` freezes the 364-train/36-validation
  benchmark at batch 20, 19 batches/epoch, 40 epochs, and 760 updates; its lock
  also pins the fixed `hbb_control_v1` checkpoint and same-manifest raw report.
- Lean cloud training pipeline (RunPod / Packet.ai + GHA) stages datasets from S3 to NVMe via `aws s3 sync`, loads annotation index into memory to avoid SQLite worker locks, logs to W&B, executes via robust Python process daemon (`scripts/cloud/run_cloud_daemon.py`) with signal traps and periodic S3 sync, and guarantees node self-termination upon completion.
- Validation caches GT/proposal IoU matrices under `@torch.no_grad()`, streams real-time evaluation math progress logs to prevent watchdog timeouts, and writes atomic checkpoints (`last.pt.tmp` -> `last.pt`).

# Local Constraints & Gotchas

- Historical v11 LiteFPN and v12 AttnRes reports used different manifest hashes;
  their one-seed numbers are hypotheses, not a production selection gate.
- HBB control v1 is a valid same-validation deployment baseline but trained for
  1,240 updates; it is not a schedule-matched ablation against the 760-step runs.
- Bounded v1 quad training clamps its configured 1,000-step warmup to all 760
  updates; preserve v1 and correct this only in a separately locked v2 recipe.
- Historical v12 raw R@50 was 0.2948, while its EMA validation was near zero;
  never compare an EMA training log with a raw checkpoint report.
- The old v12 HBB delta is invalid because the evaluator partially loaded a
  LiteFPN HBB checkpoint into an AttnRes model; strict loading now prevents it.
- AttnRes attention can become near-hard depth gating, especially at P5; inspect
  paired-seed variance before attributing slice gains to the neck.
- Exact quad decode plus polygon NMS runs in ~11.2 ms/image; forward pass is ~3.09 ms/image on GPU.
- The current lock targets PyTorch 2.13.0+cu130; historical 2.6.0+cu124 results
  are not bitwise-reproducible without the old lock and manifest bytes.
