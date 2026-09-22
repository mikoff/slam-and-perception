# Phase 4 step-20k recovery v2

Status: prepared and verified; launch requires owner approval after commit.

## Incident and recovery boundary

Recovery v1 accepted the fifth cumulative AMP skip and continued normally to
step 19,735. A sixth skip then reached its configured ceiling. The durable
`last.pt` is step 19,500 with five skips and scaler `32768`.

Recovery v2 resumes that exact full state. The only recipe change is
`max_optimizer_step_skips: 5 → 6`. It permits one dynamic-scaler backoff and
still aborts on a seventh skip.

| Item | Value |
|---|---|
| Recovery source | `artifacts/phase4/local_quad_p3_seed42_step20000_v1/last.pt` |
| Source step | 19,500 |
| Source SHA-256 | `9475e33b870661cac88d3cb6e17852c35b2a2987c712e73f2c22d5a68f6f26f2` |
| Target | 20,000 successful optimizer updates |
| Existing/new skip allowance | 5 / 1 |
| Output | Existing isolated step-20k directory |

## Prepared recovery command

Run only from a clean commit after separate owner approval:

```bash
PHASE2_DATASET_ROOT=/media/sashamikoff/fb.com-mikoff/datasets/generated/phase3-production-bg-policy-v2-2026-08-29/output \
REQUIRE_PREBUILT_INDEX=1 CUDA_VISIBLE_DEVICES=0 \
uv run python scripts/train_quad_proposals.py \
  --config configs/correction_phase4_quad_p3_local_rtx3060_step20000_recovery_v2.yaml \
  --resume artifacts/phase4/local_quad_p3_seed42_step20000_v1/last.pt \
  --max-steps 20000 --device cuda
```

Completion still requires the frozen 800-image utility, suppression, and
diversity audit before any promotion decision.
