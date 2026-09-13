# Correction Phase 4.2a HBB-versus-quad promotion contract

Status: complete. Quad remains the production incumbent; HBB is the Phase 4
challenger. No cloud work was launched by this step.

## Decision rule

A Phase 4 HBB checkpoint is promoted only when both independent decisions pass:

1. the existing `correction_phase3_selection_v1.yaml` policy requires at least
   +0.25 percentage points AR100 over the retained HBB baseline and applies its
   protected quality gates;
2. `correction_phase4_promotion_v1.yaml` requires at least +0.25 percentage
   points AR100 over the frozen quad incumbent and applies the same approved
   non-inferiority margins relative to quad.

Any failed objective or protected gate rejects promotion. A warning-band result
is inconclusive and requires seed 43 or future paired-per-image evidence, with
separate owner approval. HBB latency is report-only: it may support a passing
decision but cannot override a quality failure. Training, pilot completion, or
export success alone never changes the incumbent.

## Current control result

The policy was applied to the frozen Phase 3 utility report, using its HBB and
quad groups on the identical 800-image cohort. It correctly returns `fail`.

| Gate | HBB minus quad | Status |
|---|---:|---|
| Primary AR100 | -1.775 pp | Fail |
| R100@0.50 | -0.325 pp | Inconclusive |
| R100@0.75 | -3.520 pp | Fail |
| Proposal object fraction | -2.937 pp | Fail |
| Automotive R100@0.50 | -1.624 pp | Fail |
| Small R100@0.50 | -1.174 pp | Fail |
| Thin R100@0.50 | -1.291 pp | Fail |
| Trusted-background false | +0.277 pp | Fail |
| Unmatched proposals | +1.270 pp | Fail |

HBB still passes the coverage and duplicate gates, but those advantages do not
override the failures. This establishes the intended starting state: Phase 4
tests whether full HBB training can earn promotion; it does not presume it.

## Machine-readable policy

- Path: `configs/benchmarks/correction_phase4_promotion_v1.yaml`
- SHA-256: `2f5357907fa115807a27fa473cb38443d80ea971386f23417839202e51377ac5`
- Comparison: baseline geometry `quad`, candidate geometry `hbb`
- Required checkpoint state: EMA
- Required final proposal budget: at most 100 per image
