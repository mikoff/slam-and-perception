# Phase 4 quad continuation to step 20,000

Status: prepared and verified; launch requires owner approval after commit.

## Contract

Continue the exact step-15k optimizer trajectory for the final 5,000 successful
updates in its original 20,000-step learning-rate schedule. Architecture, data,
loss, seed, effective batch, and NMS-independent training behavior remain fixed.

| Item | Value |
|---|---|
| Source | `artifacts/phase4/local_quad_p3_seed42_step15000_v1/last.pt` |
| Source SHA-256 | `d6e06957ec79d811c26fa72ef35875abf3005463932fde3a64ec3489bfa48b40` |
| Source state | step 15,000; epoch 15, batch 32; four cumulative AMP skips |
| Target | 20,000 successful optimizer updates |
| Output | `artifacts/phase4/local_quad_p3_seed42_step20000_v1` |
| Effective batch | 16 × 8 = 128 |
| AMP boundary | four cumulative skips maximum; any new skip aborts |
| Exit operating point | NMS `0.75`, score `0.20`, K=100 |

The source is mid-epoch. Exact resume restores model, EMA, optimizer, scheduler,
scaler, sampler position, RNG, best scores, and skip count. Do not train from
`best_ema.pt`; it is a weights-only evaluation artifact.

The 15k frozen audit passed all seven gates. Its duplicate fraction is 0.08888
against the 0.090 ceiling, so the 20k result is not presumed better and receives
the same frozen utility, suppression, and diversity audit before promotion.

## Prepared launch command

Run only from a clean commit after separate owner approval:

```bash
PHASE2_DATASET_ROOT=/media/sashamikoff/fb.com-mikoff/datasets/generated/phase3-production-bg-policy-v2-2026-08-29/output \
REQUIRE_PREBUILT_INDEX=1 CUDA_VISIBLE_DEVICES=0 \
uv run python scripts/train_quad_proposals.py \
  --config configs/correction_phase4_quad_p3_local_rtx3060_step20000_v1.yaml \
  --resume artifacts/phase4/local_quad_p3_seed42_step15000_v1/last.pt \
  --max-steps 20000 --device cuda
```

The output directory must be absent initially. After a durable checkpoint, only
its own `artifacts/phase4/local_quad_p3_seed42_step20000_v1/last.pt` may resume
the interrupted stage.

## Exit decision

Audit the step-20k EMA on the frozen 800 images at NMS `0.75`, score `0.20`.
Promotion requires every existing gate to pass and a separate owner decision.
