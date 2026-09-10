# Correction Phase 3.6 hard-negative mining

Status: comparison complete; reject the mined candidate. No source annotation
was changed, and the retained normalized HBB baseline remains selected.

## Frozen mining round

`scripts/mine_hard_negatives.py` creates the candidate set,
`scripts/promote_hard_negatives.py` applies reviewed decisions, and
`scripts/audit_hard_negative_exposure.py` verifies optimizer-window exposure.

The retained normalized HBB baseline (`objectness_weight=1.0`) ran with EMA
weights over a deterministic 800-image mining pool drawn only from the train
manifest. The validation split is excluded. The pool uses seed 3601 and the
approved source mixture: 400 COCO, 144 nuImages, 96 BDD100K, and 160 WoodScape.

Inference uses the approved HBB policy: score 0.30, NMS IoU 0.70, K=100, and
pre-NMS K=300. A proposal is unmatched only when its maximum object IoU is below
0.10. At least 50% proposal coverage establishes overlap with an ignore or
trusted-background region. The immutable prebuilt train index is accepted only
when its schema and stored source-manifest SHA-256 match the frozen policy; this
avoids rereading the very large JSON manifest during startup.

Candidates are ranked by score, limited to three per source image, capped at
600 general, 360 automotive, and 240 fisheye examples, and checked for
near-identical crops with a 64-bit perceptual hash. These caps prevent a single
image or domain from dominating the supplemental set.

## Result

| Item | Count |
|---|---:|
| Mining-pool images | 800 |
| Raw high-score unmatched candidates | 7,632 |
| Retained after caps/deduplication | 1,200 |
| Certified trusted background | 40 |
| Existing ignore region | 171 |
| Possible labeled object | 355 |
| Unverified background | 634 |
| Stratified audit cards | 160 |

Domain caps retained 600 general, 360 automotive, and 240 fisheye candidates.
No near-duplicate crop was found before the image/domain caps were filled.
Automated checks found no invalid source box, score below 0.30, or retained
candidate with object IoU at or above 0.10.

## Review and promotion decision

Open `artifacts/phase3/correction_mining_v1/full/index.html`. Red is the mined
proposal, lime is a positive region, yellow is an existing ignore region, and
blue is certified trusted background. The 160 cards are stratified across
classification and domain rather than being only the highest scores.

Edit the `decision` column in `review.csv` using one of:

- `approve_negative`: the red proposal is clearly background;
- `ignore`: it contains or may contain a useful proposal object;
- `uncertain`: evidence is insufficient.

On 2026-09-09 the owner delegated the conservative default strategy. The
promotion tool approved only the 40 `certified_trusted_background` candidates;
the other 1,160 decisions have no training effect. The 40 selectors span 24
images: 10 automotive and 30 fisheye. COCO contributes none because it has no
certified trusted-background regions.

The red proposal box is only a focus/resampling selector. It does not become a
new background polygon. Effective supervision is restricted to the intersection
with trusted-background regions already present in the immutable source
manifest, while positive and ignore precedence remains unchanged. This matters
for proposals that partly cover a labeled object despite having at least 50%
trusted-background coverage.

## Provenance

- Mining policy SHA-256:
  `e133090690e48b528f3255908f037a3c22ef793ca91728d1f6fd4ea2011e35a7`
- Baseline checkpoint SHA-256:
  `3bb70fcbc3e81ee5cd2198d2626f570f0bb6a4b3e4e10cb82fd61497958f8063`
- Pool identity SHA-256:
  `5f4aa91611f1893e4cba15997b5214a17877e45d3d5298d1bfa390da9a52648c`
- Candidate manifest SHA-256:
  `8f9be8e0d82242c70e608fbe31479fced9a6890e9f8818920e3a95be9694e259`
- Review CSV SHA-256 before decisions:
  `0b35ac158eb131536fc1501414c354a1a38f0e93dec3ca0c1031462a15c1d40b`
- Promotion policy SHA-256:
  `a92895d6bd2028dbdbf46d0b482b938c59b9f52461aa6d81c99d3585aca8924b`
- Approved supplemental manifest SHA-256:
  `65115c293506834de5c25f2668a630d7c57fa224743f745f58771f93cc26cffc`
- Finalized default decisions SHA-256:
  `5a0b10bf3cd24cbc9854a7a14a0a564d0ddd86c758aa3023b8f9b93fba1f9a5b`

## Controlled comparison

The owner approved one same-source, non-empty focus replacement per 32-image
optimizer window, with a 1:3 BDD-to-WoodScape focus ratio. The replacement
preserves the original source/domain counts. Only existing trusted-background
points inside the focus selector receive 2x objectness weight; positives and
ignore regions keep precedence. The exposure audit verified exactly 200 focus
draws in 200 windows: 50 BDD and 150 WoodScape.

The 200-step candidate started from the same model state as the retained
baseline (`7af6a0f2...a244`). All 147 detector tests passed before training. The
fixed 800-image utility report satisfied every comparison-contract identity
check, but the frozen Phase 3.5 selection policy rejected the candidate.

| Selection metric | Baseline | Candidate | Delta (pp) | Gate |
|---|---:|---:|---:|---|
| Overall recall 100@0.50 | 40.693% | 40.750% | +0.057 | pass |
| Fisheye recall 100@0.50 | 23.683% | 24.205% | +0.522 | pass |
| Small recall 100@0.50 | 32.238% | 30.595% | -1.643 | fail |
| Thin recall 100@0.50 | 6.653% | 5.660% | -0.993 | fail |
| Trusted-background false fraction | 2.229% | 2.595% | +0.366 | fail |
| Unmatched false fraction | 47.833% | 50.638% | +2.805 | fail |
| AR100 objective | 14.783% | 14.609% | -0.174 | inconclusive |

The change therefore failed its intended false-proposal objective and damaged
protected small/thin slices. Do not promote it or calibrate thresholds for it.
The retained normalized baseline remains the input to the next owner-approved
step.

Comparison provenance:

- Candidate config SHA-256:
  `6d68fe14676dfbbb439349f214d582b2f18f6522b118819994de10efb507f8cb`
- Exposure report SHA-256:
  `364b1ebca3a3bddc708131973e6e4bfb23c93c8653b2ce2c070e243a9ae91c26`
- Candidate utility report SHA-256:
  `a3fcd2daa64e0398651e120a8485d3e3099b6ddbe98bfa3536c4f9ca914b84f8`
- Selection result SHA-256:
  `497ba875fe9ba6cd34a49a8deae8372f5a10fa2215c687855ef9f01e5e9a5cd7`
