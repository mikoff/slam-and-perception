# Phase 4 staged quad continuation

Status: step 10,000 complete. The frozen 800-image audit passes all continuation
gates at the owner-approved Phase 4 candidate operating point of NMS `0.75` and
score `0.20`. Step-15k preparation is approved; launch remains a separate decision.

## Purpose and boundary

The exact optimizer trajectory continued from step 5,000 to 10,000 without an
architecture, data, loss, schedule, seed, or batch-partition change. This remains
an experiment: the Phase 2 quad EMA is the production incumbent, and the selected
suppression point applies only to Phase 4 candidate evaluation.

The run completed after bounded recovery from step 9,000. Its final checkpoint
records 10,000 successful updates and two cumulative AMP-skipped windows, below
the fail-closed maximum of four.

## Immutable outputs

| Item | Value |
|---|---|
| Full-state checkpoint | `artifacts/phase4/local_quad_p3_seed42_step10000_v1/last.pt` |
| Full-state SHA-256 | `bb438343f4898be4109365769f2ff0ea6f4052d5c24a2eb983268f4f7720812c` |
| EMA checkpoint | `artifacts/phase4/local_quad_p3_seed42_step10000_v1/best_ema.pt` |
| EMA SHA-256 | `c17214972b7de15f9296f91dd496e74323374ee985cc4d7ad8c7d053dc8b0906` |
| Final state | global step 10,000; epoch 10, batch 16; epoch incomplete |
| Frozen cohort | 800 images; subset SHA-256 `e705233a…e15806` |
| Suppression sweep SHA-256 | `d05794cf…f6a0f8` |
| NMS-0.75 diversity SHA-256 | `ad3c89b9…97c9531` |

`best_ema.pt` is evaluation-only. Any further training must resume from the exact
full-state `last.pt` into a new output directory.

## Step-10k decision

| Gate | Limit | Observed | Result |
|---|---:|---:|---|
| AR@100 | ≥0.240 | 0.26135 | pass |
| Recall@100, IoU 0.75 | ≥0.205 | 0.23092 | pass |
| Duplicate fraction | ≤0.090 | 0.08142 | pass |
| All-pair IoU≥0.50 fraction | ≤0.040 | 0.02726 | pass |
| Unmatched-pair IoU≥0.75 fraction | ≤0.0075 | 0.00000 | pass |
| Unmatched IoU-0.75 cluster p95 | ≤8 | 1 | pass |
| Images with unmatched cluster ≥20 | 0 | 0 | pass |

NMS `0.75` loses only 0.16 AR percentage points relative to `0.80`, reduces
duplicates from 11.06% to 8.14%, and improves low-IoU, small-object, and
thin-object recall. Total unmatched rate remains report-only; the retained
unmatched proposals remain spatially and scale diverse.

## Next stage

The isolated step-15k recipe and evidence-bound contract are documented in
`correction_phase_4_quad_step15000_v1.md`. Preparing those files does not approve
the launch. Step 20k still requires a new frozen exit audit and separate approval.
