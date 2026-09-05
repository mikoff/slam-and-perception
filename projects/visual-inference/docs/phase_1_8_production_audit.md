# Phase 1.8 production audit

Status: complete and owner-approved. The candidate was subsequently published;
Phase 2 remains blocked only on the clean-worker portion of the Phase 1.9 gate.

## Bound candidate

The audit passed its raw-source, conversion-policy, approved-split, manifest,
index, and dataset-contract binding checks for
`phase3-production-bg-policy-v2-2026-08-29`.

| Split | Images | Positive | Ignore | Trusted background |
|---|---:|---:|---:|---:|
| Train | 266,977 | 3,765,518 | 45,846 | 21,473,325 |
| Validation | 32,268 | 569,716 | 1,906 | 3,041,953 |

Contract SHA-256:
`0c41b8af31825feb6929067a4cb7128c36569af9c19cb6d14f5cf77dc3ec797a`.
The manifests are `proposal-manifest.v2`, both SQLite indexes match the
contract and manifest hashes, and no image was unreadable.

## Checks that passed

- Zero invalid polygons or source-identity collisions within either split.
- Zero source-identity, sequence, or image-content leakage across splits.
- Every one of the 299,245 images and every indexed region was visited.
- HBB and quad target tests prove `positive > ignore > trusted > weak`: a
  positive coincident with full-frame ignore and trusted regions remains the
  only supervised target at its assigned points.
- The stratified audit bundle contains 431 category/state/source entries.

## Raw polygon intersections

The original auditor labeled every raw cross-state polygon intersection as a
hard failure. The exact follow-up diagnostic reproduced all gate image counts
and classified the intersecting polygon pairs. Audit schema v2 reports these as
warnings while hard-failing effective supervision conflicts.

| Pair | Train images | Validation images | Diagnostic conclusion |
|---|---:|---:|---|
| Positive / ignore | 14,721 | 947 | Mostly expected precedence containment |
| Positive / trusted | 2,426 | 227 | Raster-tile/continuous-quad boundary effects |
| Ignore / trusted | 138 | 8 | Raster-tile/continuous-quad boundary effects |

Positive/ignore intersections are dominated by COCO crowd/policy-ignore regions
and WoodScape ego-mask integrity quarantine. An implausible WoodScape ego mask
covering more than 60% of an image is deliberately changed from trusted negative
to ignore; representative quarantined quads span the frame, while valid positive
objects win during target construction. Treating those raw intersections as
simultaneous labels is therefore incorrect.

Trusted-background tiles are selected from raster masks after positive and
ignore subtraction. Continuous polygon intersection detects small boundary
slivers that raster selection does not: positive/trusted totals 1,501.79 px²
over 2,426 train images and 150.95 px² over 227 validation images; the largest
reviewed example is 23.49 px² against a 32×32 tile. Target precedence removes
these slivers before background assignment.

## Duplicate-content warnings

The audit found 27 exact-content groups in train (12 COCO, 14 nuImages, one BDD)
and 11 in validation (nine nuImages, two BDD), representing 29 and 11 redundant
records respectively. All remain inside their original source and split; no
hash crosses train/validation. At 40 of 299,245 records, this is source-level
redundancy rather than leakage and is non-blocking unless the owner requests
deduplication.

## Approved decision

The owner approved keeping the candidate unchanged and distinguishing reported
raw intersections from effective supervision conflicts. The former and exact
within-split duplicates are warnings; contradictory state declarations,
cross-state source identities, and cross-split leakage remain hard failures.
The final full audit under this definition passed with zero hard failures and
zero effective supervision conflicts. Its SHA-256 is
`869785b36ad8c8b936508447b3162e88b8dc2b8e2e68902b624462b646be32a6`.
The owner-review sidecar SHA-256 is
`31eda08eccb5d3d610dad8684de4d7aff8faa0f7ddca23f823d1352cf0429083`.

The immutable archive and manifest were published under the approved dataset ID.
Clean-machine staging and HBB/quad loader smoke remain Phase 1.9 work. Do not
begin Phase 2 until that final gate is complete.
