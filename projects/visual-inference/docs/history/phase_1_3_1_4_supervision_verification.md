# Phase 1.3–1.4 supervision verification

Date: 2026-08-28

## Implemented contract

- `proposal-manifest.v2` is the single positive/ignore/trusted-background state
  authority for HBB and quad readers.
- Category state and nested-component containment are resolved during common
  manifest generation, not in either loader.
- Trusted candidates are rasterized and tiled only after applying
  `positive > ignore > trusted background > weak` precedence.
- Spatial checks report self-intersection, duplicate vertices, zero area, and
  out-of-frame coordinates. Repairs are logged; localized unrepairable geometry
  becomes ignore.
- WoodScape ego bodywork is trusted background only when its mask has nonzero
  area, covers at most 60% of the image, and touches an image boundary.
- Owner-approved category exceptions are preserved: grouped vehicles and
  grouped animals are ignore; `grouped_pedestrian_and_animals` is positive;
  construction is trusted background on annotated pixels.
- WoodScape validation is a whole-sequence holdout from labeled RGB training
  frames. Raw test frames remain qualitative only.

## Full WoodScape filter evidence

The isolated full filter read 15,000 images and retained 616,960 approved
annotations. It generated these states before geometry-specific encoding:

| State/family | Count |
|---|---:|
| Positive | 332,707 |
| Ignore | 41,736 |
| Trusted background candidates | 242,517 |
| Construction trusted background | 106,079 |
| Ego trusted background | 16,251 |
| Ego quarantined to ignore | 1,830 |
| Grouped vehicles ignore | 6,888 |
| Grouped people/animals positive | 3,219 |
| Grouped animals ignore | 2 |

Ego quarantine reasons were 1,553 excessive-coverage masks, 271 detached masks,
and 6 zero-area masks.

## Sequence split evidence

- 8,234 labeled RGB images form 194 timestamp-derived sequences.
- 191 sequences / 7,411 RGB images remain in training.
- 3 sequences / 823 RGB images form validation (9.995%).
- Train/validation sequence overlap is zero.
- All 2,766 raw test images remain qualitative.

## End-to-end real-data sample

A deterministic 128-image sample interleaved 64 train and 64 validation frames.
Conversion processed 7,843 source annotations. Both proposal manifests passed
geometry and link validation with zero fit-coverage failures.

| Split | Positive | Ignore | Trusted background |
|---|---:|---:|---:|
| Train | 2,483 | 78 | 50,671 |
| Validation | 2,381 | 165 | 50,720 |

HBB and quad readers independently reported exactly the validation counts above.
The trusted-background report recorded zero positive/trusted overlap and zero
ignore/trusted overlap after precedence. It also records candidate count and
area for every configured source category, including zero-count categories in a
bounded sample.

The full geometry conversion was deliberately not published or substituted for
the existing dataset during this implementation gate. It must run as part of the
later immutable full-regeneration step after this supervision review is accepted.

## Owner visual gate

Review the generated
[`woodscape_supervision_audit_phase_1_4/index.html`](../../../data/visual-inference-datasets/reports/woodscape_supervision_audit_phase_1_4/index.html).
It contains 122 deterministic cards: green is positive, amber is ignore, blue is
trusted background, and red is the reviewed target. Approve or identify any
incorrect ego, construction, or grouped-region states before proceeding.
