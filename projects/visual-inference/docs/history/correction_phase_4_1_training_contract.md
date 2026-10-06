# Correction Phase 4.1 training contract

Status: approved and implemented; Phase 4.2 and 4.2a passed.

## Recommended contract

| Decision | Proposed value |
|---|---|
| Geometry | Train HBB LiteFPN P3–P5 as challenger; retain quad as incumbent |
| Initialization | Selected step-200 EMA weights; reset every training/runtime state |
| Training budget | 20,000 successful optimizer steps |
| Sampling budget | 2,560,000 images at effective batch 128 |
| Schedule | 1,000-step warm-up, then cosine to 1% of base LR |
| FP16 scaler | initial scale 8192; fail closed on a skipped optimizer step |
| Learning rates | head/FPN `3e-4`; backbone `3e-5` |
| Backbone freeze | none; all warm-started weights train from step 1 |
| Loss | QFL 1.0 + CIoU 2.0 + LTRB 0.5; centerness disabled |
| EMA | reset from initialized model; decay 0.9998 with 1,000-step ramp |
| Checkpoints | every 500 steps; latest, previous, best-primary, best-constraint |
| Validation | raw and EMA every 2,000 steps and at completion |
| Selection | both HBB-baseline and quad-incumbent policies must pass on the frozen 800-image cohort |
| Primary seed | 42 |
| Second seed | 43 only after an inconclusive result and separate approval |
| Cloud | existing GitHub → Packet bridge → attempt-specific dstack SSH-fleet path; one RTX 4090 |
| Placement | Packet bridge resolves an available exact-RTX4090 pool and compatible region |
| Ceilings | 12 hours and `$12.00` projected compute cost |
| W&B | project `visual-inference`; immutable GitHub-derived run ID |
| S3 | immutable `runs/${RUN_ID}` prefix |

The dollar ceiling is not a claim about Packet's current price. Packet's launch
options used by the bridge do not expose a price, so the owner enters the exact
current account rate as `PACKET_HOURLY_RATE_USD`. Preflight rejects the dispatch
when rate times 12 hours exceeds `$12.00`, and GitHub records the supplied rate
in its summary. The bounded pilot then records measured throughput, projected
runtime, and projected cost before the full-run approval decision.
Each attempt uses a fresh, uniquely named Packet host and fleet; it may not
adopt an existing instance. The bridge may make at most three replacement
attempts using its existing cleanup and reconciliation path.

The recipe defines 1,000 sampler batches as one scheduling epoch so validation
and checkpoint points are exact. It does not claim that this is an exhaustive
dataset pass. The primary exposure measure is successful optimizer steps and
sampled images.

Metric-based early stopping is disabled because the frozen AR100 selection
metric is produced by the separate proposal-utility evaluator, not the lighter
epoch validator. Operational failures and either approved ceiling stop the run.
A failed batch-128 preflight requires a new contract version and approval; the
runtime may not silently change batch, accumulation, or learning rate.

Machine-readable inputs:

- recipe: `configs/correction_phase4_hbb_p3_v1.yaml`, SHA-256
  `5224ab61782666ed0f1487d92f644eac344cc76e796482698dd346b873e3bd02`;
- contract: `configs/benchmarks/correction_phase4_training_contract_v1.yaml`,
  SHA-256 `2056ab74806c6201e47916c834be3f21a96a257e1f974a4cf58d098261eadc44`;
- promotion policy: `configs/benchmarks/correction_phase4_promotion_v1.yaml`,
  SHA-256 `2f5357907fa115807a27fa473cb38443d80ea971386f23417839202e51377ac5`.

## Implementation audit

The existing Packet provisioning pipeline is retained: the GitHub workflow,
fresh-host Packet bridge, host bootstrap, attempt-specific dstack SSH fleet,
and reconciliation path remain intact. The Phase 4 entrypoint now selects the
HBB trainer, downloads and verifies the exact initialization artifact, rejects
incompatible resume, and narrows Packet execution to 12 hours. HBB now has the
quad path's strict initialization, S3 auto-resume, immutable run identity, W&B
setup, and raw-plus-EMA validation.

The [Phase 4.2 local preflight](correction_phase_4_2_local_preflight.md) proves
the implementation gates, including real CUDA FP16 updates and a mid-epoch
resume. The Packet renderer still removes dstack's marketplace `max_price`,
because that field cannot protect a Packet-launched SSH fleet; the exact
owner-supplied hourly rate and automated projection check remain mandatory.
Passing this preflight does not authorize a cloud pilot or full run.

Step 4.2a subsequently made quad the explicit production incumbent. A final HBB
checkpoint must pass both the original HBB-baseline policy and the new
quad-incumbent promotion policy; neither training nor a successful pilot changes
the selected production geometry by itself.

## Owner decision

Approval freezes the architecture, initialization, 20,000-step budget,
validation/checkpoint cadence, seed policy, and cloud ceilings above. Changes
afterward require a v2 recipe and contract. Cloud pilot and full retraining each
remain separate approval points.
