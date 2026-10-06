# Correction Phase 3.5 selection contract

Status: complete. Policy and checker verified; the owner approved the numerical
margins on 2026-09-09. No training or model configuration changed in this step.

## Frozen comparison contract

Candidate and baseline utility reports must use EMA weights, the identical
800-image ordered validation subset, validation manifest, source counts, metric
definitions, HBB thresholds, and retained quad checkpoint/results. HBB uses the
approved NMS/score policy `0.70/0.30` at K=100. A mismatch yields
`invalid_contract`, not a model-quality decision.

The primary objective is AR100 because it averages recall across IoU 0.50-0.95.
A candidate must improve AR100 by at least 0.25 percentage points and clear
every guardrail to pass. A clear guardrail violation fails. Values between the
warning and failure boundaries are inconclusive.

## Frozen numerical gates

For higher-is-better metrics, more-negative deltas are worse. For lower-is-
better metrics, more-positive deltas are worse. All boundaries are percentage
points relative to the matched baseline.

| Metric | Direction | Warning boundary | Failure boundary |
|---|---|---:|---:|
| Primary AR100 | Higher | improvement below +0.25 | below -0.25 |
| Overall R100/0.50 | Higher | below -0.10 | below -0.50 |
| Overall R100/0.75 | Higher | below -0.10 | below -0.30 |
| Object coverage | Higher | below -0.10 | below -0.50 |
| Proposal object fraction | Higher | below -0.25 | below -1.00 |
| General/automotive/fisheye R100/0.50 | Higher | below -0.25 | below -1.00 |
| Small or thin R100/0.50 | Higher | below -0.25 | below -0.75 |
| Source 16-32 px R100/0.50 | Higher | below -0.25 | below -1.00 |
| Trusted-background false | Lower | above +0.10 | above +0.25 |
| Unmatched | Lower | above +0.20 | above +0.50 |
| Duplicate/all | Lower | above +0.20 | above +0.50 |
| Ego-body overlap diagnostic | Lower | above +1.00 | above +2.00 |
| Proposals/image | Lower | none | absolute maximum 100 |

These are practical non-inferiority margins, not confidence intervals. The
current aggregate reports do not retain paired per-image sufficient statistics,
so a valid bootstrap cannot be reconstructed. An otherwise acceptable result
inside a warning band is `inconclusive` and requires another seed or a future
report that retains paired per-image statistics.

Model-tiny recall remains report-only until the baseline is nonzero. Medium and
large slices and K=10/K=50 metrics are also reported but not primary gates.
Latency is deferred here because loss-only candidates have identical
architecture. Sky/structure/vegetation trusted-background rates are unavailable
because provenance was collapsed; crowded-scene recall is unavailable because
the frozen report has no crowd slice. SigLIP remains outside this milestone.

## Verified decision behavior

`scripts/evaluate_selection_candidate.py` applies the versioned YAML policy and
writes a provenance-bound JSON decision with input hashes. Unit tests cover pass, fail,
inconclusive, lower-is-better warnings, and invalid comparison contracts.

Applied to the objectness-1.5 experiment, the checker returns `fail`:

| Item | Delta | Status |
|---|---:|---|
| Primary AR100 | -0.226 pp | Inconclusive |
| Overall R100/0.50 | -0.344 pp | Inconclusive |
| Overall R100/0.75 | -0.230 pp | Inconclusive |
| General R100/0.50 | -1.358 pp | **Fail** |
| Small R100/0.50 | -0.782 pp | **Fail** |
| Source 16-32 px R100/0.50 | -1.495 pp | **Fail** |
| Fisheye R100/0.50 | +0.854 pp | Pass |
| Trusted-background false | +0.089 pp | Pass |
| Duplicate/all | -0.107 pp | Pass |

Policy: `configs/benchmarks/correction_phase3_selection_v1.yaml`, SHA-256
`23186efcbcf9cf1387ab4877adc76846d828a1f39a0dd573d209535eb6e2beda`.
Decision artifact:
`artifacts/phase3/correction_quality_1p5_v1/selection_quality1p5_vs_baseline.json`,
SHA-256 `4aae9f9a9fd230aab3f4470a8ed9a0b40f44c6cbf478f96beb65a538ccc5b8b5`.

This is the fixed Phase 3 selection contract. Changing a margin requires a new
policy version rather than overwriting this file.
