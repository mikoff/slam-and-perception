# Phase 4 step-20k recovery

Status: prepared and verified; launch requires owner approval after commit.

## Incident and recovery boundary

The first 20k attempt intentionally aborted at step 17,319 when dynamic AMP
loss scaling skipped its fifth cumulative optimizer update. The durable
`last.pt` is step 17,000 with four skips and scaler `32768`.

Recovery replays from that exact full-state checkpoint. The only recipe change
is `max_optimizer_step_skips: 4 → 5`. The expected overflow can therefore lower
the scaler to `16384` and training can continue; a sixth skip still aborts.

| Item | Value |
|---|---|
| Recovery source | `artifacts/phase4/local_quad_p3_seed42_step20000_v1/last.pt` |
| Source step | 17,000 |
| Source SHA-256 | `e314d53bf673e5220d8c1b1e159f6a207dfc31677dce6a50fd41ca37e7e7a526` |
| Target | 20,000 successful optimizer updates |
| Existing/new skip allowance | 4 / 1 |
| Output | Existing isolated step-20k directory |

## Prepared recovery command

Run only from a clean commit after separate owner approval:

```bash
PHASE2_DATASET_ROOT=/media/sashamikoff/fb.com-mikoff/datasets/generated/phase3-production-bg-policy-v2-2026-08-29/output \
REQUIRE_PREBUILT_INDEX=1 CUDA_VISIBLE_DEVICES=0 \
uv run python scripts/train_quad_proposals.py \
  --config configs/correction_phase4_quad_p3_local_rtx3060_step20000_recovery_v1.yaml \
  --resume artifacts/phase4/local_quad_p3_seed42_step20000_v1/last.pt \
  --max-steps 20000 --device cuda
```

The first recovery preflight binds the exact step-17k hash and state. Later
durable checkpoints in the same output may resume without weakening the skip
limit. Completion still requires the frozen 800-image audit.
