# Phase 4 quad continuation to step 15,000

Status: completed and audited.

## Contract

Continue the exact step-10k optimizer trajectory for 5,000 additional successful
updates. Architecture, data, loss, learning-rate schedule, seed, effective batch,
and the 20,000-step scheduling horizon remain unchanged. The output is isolated
from the immutable 10k source.

| Item | Value |
|---|---|
| Source | `artifacts/phase4/local_quad_p3_seed42_step10000_v1/last.pt` |
| Source SHA-256 | `bb438343f4898be4109365769f2ff0ea6f4052d5c24a2eb983268f4f7720812c` |
| Source state | step 10,000; epoch 10, batch 16; two cumulative AMP skips |
| Target | 15,000 successful optimizer updates |
| Output | `artifacts/phase4/local_quad_p3_seed42_step15000_v1` |
| Effective batch | 16 × 8 = 128 |
| AMP boundary | four cumulative skips maximum; fifth aborts |
| Exit operating point | NMS `0.75`, score `0.20`, K=100 |

The source is mid-epoch. Exact resume restores sampler position, RNG, model, EMA,
optimizer, scheduler, scaler, best scores, and skip count. `best_ema.pt` must not
be used as a training source.

The training recipe retains its embedded NMS `0.70` for exact continuity. The
frozen entry and exit decisions explicitly use candidate NMS `0.75`; suppression
does not affect optimizer updates.

## Executed launch contract

```bash
PHASE2_DATASET_ROOT=/media/sashamikoff/fb.com-mikoff/datasets/generated/phase3-production-bg-policy-v2-2026-08-29/output \
REQUIRE_PREBUILT_INDEX=1 CUDA_VISIBLE_DEVICES=0 \
uv run python scripts/train_quad_proposals.py \
  --config configs/correction_phase4_quad_p3_local_rtx3060_step15000_v1.yaml \
  --resume artifacts/phase4/local_quad_p3_seed42_step10000_v1/last.pt \
  --max-steps 15000 --device cuda
```

The run completed at step 15,000 with four cumulative AMP skips. Its EMA was
evaluated on the same frozen 800-image cohort at NMS `0.75`, score `0.20`.

## Exit result

| Metric | Step 15k | Gate | Result |
|---|---:|---:|---|
| AR@100 | 0.26646 | >= 0.240 | pass |
| Recall@100 IoU 0.75 | 0.23972 | >= 0.205 | pass |
| Duplicate fraction | 0.08888 | <= 0.090 | pass |
| All-pair IoU 0.50 fraction | 0.02830 | <= 0.040 | pass |
| Unmatched-pair IoU 0.75 fraction | 0.00000 | <= 0.0075 | pass |
| Unmatched IoU 0.75 cluster p95 | 1 | <= 8 | pass |

All continuation gates passed. The duplicate gate has only 0.00112 absolute
headroom, so the 15k→20k stage keeps the cumulative four-skip ceiling and must
be audited before any production promotion.
