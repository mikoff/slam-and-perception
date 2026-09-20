# Phase 4 staged quad continuation

Status: bounded recovery reached step 9,360 and stopped on its third FP16
overflow. Final recovery from the durable step-9,000 checkpoint is approved but
not relaunched.

## Purpose

Continue the existing quad optimizer trajectory from step 5,000 to step 10,000
without changing its architecture, data, loss, schedule, seed, or batch
partition. This is an experiment, not a production promotion. The Phase 2 quad
EMA remains the production incumbent.

The frozen diversity audit weakens the original interpretation of unmatched
proposals as false positives: the 5k model has lower overall overlap, broader
spatial coverage, and broader scale coverage than the incumbent. Its unmatched
high-IoU tail is slightly larger, so explicit overlap-cluster gates replace
unmatched rate as the continuation stop condition.

## Immutable inputs

| Item | Value |
|---|---|
| Resume checkpoint | `artifacts/phase4/local_quad_p3_seed42_pilot_v1/last.pt` |
| Resume SHA-256 | `3f1459627bea4dd13c8ddfd51c9edb3c8ac551769c3457f5dc312aa92c6c9926` |
| Resume state | full training state at global step 5,000; epoch 4 complete |
| Current recovery checkpoint | `artifacts/phase4/local_quad_p3_seed42_step10000_v1/last.pt` |
| Current recovery SHA-256 | `df2357c7d291b60d9ff51fc9703bed3534a5d8883887366a8e74eec1cbc999f6` |
| Current recovery state | full training state at global step 9,000; two persisted AMP skips |
| Target | global step 10,000 |
| New output | `artifacts/phase4/local_quad_p3_seed42_step10000_v1` |
| Frozen cohort | 800 images, subset SHA-256 `e705233a…e15806` |

`best_ema.pt` is not a resumable checkpoint. The new output directory prevents
the source `last.pt` and the 5k evidence from being overwritten.

## Launch boundary

After the contract tests and local identity preflight pass, the final supervised
recovery command is:

```bash
PHASE2_DATASET_ROOT=/media/sashamikoff/fb.com-mikoff/datasets/generated/phase3-production-bg-policy-v2-2026-08-29/output \
REQUIRE_PREBUILT_INDEX=1 CUDA_VISIBLE_DEVICES=0 \
uv run python scripts/train_quad_proposals.py \
  --config configs/correction_phase4_quad_p3_local_rtx3060_step10000_v1.yaml \
  --resume artifacts/phase4/local_quad_p3_seed42_step10000_v1/last.pt \
  --max-steps 10000 --device cuda
```

Do not pass `--initialize-from`; doing so would reset the optimizer trajectory.
The existing output directory is required for this interrupted-stage recovery.

If the stage is interrupted after writing a checkpoint, rerun the command with
`--resume artifacts/phase4/local_quad_p3_seed42_step10000_v1/last.pt`. The
preflight accepts that recovery path only when its embedded configuration
matches and its global step is strictly between 5,000 and 10,000.

The final recovery permits at most four cumulative AMP-skipped optimizer
windows. Skipped windows are logged and do not advance global step, scheduler,
or EMA. The checkpoint already records two skips; PyTorch reduces the scaler
automatically when the third event is replayed. A fifth cumulative skip aborts
the run. This preserves the 10,000-successful-update target while preventing
silent unbounded drift.

## Step-10k decision

Evaluate the step-10k EMA on the same frozen utility and diversity cohort. AR,
high-IoU recall, existing duplicates, all-proposal overlap, and unmatched-only
overlap clusters are continuation gates. Object fraction and unmatched rate are
reported but cannot reject the run by themselves. Step 15k requires every gate
to pass and separate owner approval; step 20k follows the same rule.
