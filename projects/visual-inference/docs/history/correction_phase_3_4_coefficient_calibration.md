# Correction Phase 3.4 coefficient calibration

Status: complete. The matched candidate was rejected; retain objectness weight
1.0 and do not run the upper 2.0 control.

## Product-weighted interpretation

Phase 2 measured only 1.29% certified trusted-background false proposals for
the HBB primary, while the approved suppression policy reduced duplicate/all to
7.33%. The larger unresolved failures are geometric and coverage related:
R100/0.75 is 10.04%, model-tiny recall is zero, thin-object R100/0.50 is 7.65%,
and WoodScape R100/0.50 is 28.14%. Ego-body overlap remains a risk, but its
ignore-only annotation cannot support treating it as a negative class.

Localization should therefore remain the dominant family. Quality still needs
enough influence to rank useful objects within K=100 and should not continue
shrinking unchecked relative to localization.

## Target definition

Use the geometric mean of the overall FPN and C5 median quality/localization
norm ratios as the scalar calibration target. The geometric mean treats the two
shared sites symmetrically in multiplicative ratio space while the report still
retains both site values and every domain/size slice.

With the current coefficients, this aggregate is 0.252 at step 100 and 0.104 at
step 200. Scaling only the objectness coefficient changes the fixed-checkpoint
quality gradient linearly; actual retraining remains nonlinear.

| Candidate | Objectness | CIoU | LTRB | Predicted aggregate @100 | Predicted aggregate @200 | Role |
|---|---:|---:|---:|---:|---:|---|
| Baseline | 1.0 | 2.0 | 0.5 | 0.252 | 0.104 | Unchanged control |
| Recommended | 1.5 | 2.0 | 0.5 | 0.378 | 0.157 | Moderate quality reinforcement |
| Upper control | 2.0 | 2.0 | 0.5 | 0.504 | 0.209 | Bound sensitivity without approaching parity |

The recommended target is **0.38 after warm-up**, implemented by changing only
`objectness_weight` from 1.0 to 1.5. At the measured step-100 checkpoint this
predicts FPN Q:L 0.363 and C5 Q:L 0.394. Localization remains roughly 2.5-2.8
times stronger at both sites. Keeping CIoU and LTRB unchanged preserves their
approved internal geometry balance.

Do not change centerness, weak-negative, trusted-negative, focal exponent, GWD,
or suppression in this calibration. Those are separate experiment variables;
combining them would prevent attribution.

## Offline calibration result

The owner approved the 0.38 target and an offline measurement-only cycle. The
versioned candidate config changes only `objectness_weight` 1.0 to 1.5 and its
isolated output directory. The raw step-100 baseline checkpoint was remeasured
on the identical ordered cohort.

| Cohort | FPN Q:L baseline | FPN Q:L candidate | C5 Q:L baseline | C5 Q:L candidate |
|---|---:|---:|---:|---:|
| Overall | 0.242 | 0.363 | 0.263 | 0.394 |
| General | 0.217 | 0.326 | 0.221 | 0.332 |
| Automotive | 0.228 | 0.342 | 0.268 | 0.401 |
| Fisheye | 0.225 | 0.338 | 0.272 | 0.409 |
| Edge-small | 0.259 | 0.388 | 0.271 | 0.407 |
| Small | 0.211 | 0.316 | 0.206 | 0.309 |
| Medium | 0.195 | 0.292 | 0.183 | 0.275 |
| Large | 0.304 | 0.456 | 0.272 | 0.408 |

The achieved overall geometric-mean ratio is 0.378. Localization remains the
larger median gradient in every slice. The checkpoint and ordered image cohorts
are identical to the baseline measurement. Unweighted and localization terms
are unchanged; weighted quality summaries scale by 1.5 within 0.02% FP32
tolerance, their variances scale by 2.25 within 0.1%, and cosine changes by less
than `5e-5`.

Artifacts:

- config: `configs/benchmarks/correction_phase3_quality_1p5_v1.yaml`, SHA-256
  `67ebc11e61b0134c7b09a081a4309dd64afb37d86d61691f197e474b9b2d75d8`;
- report: `artifacts/phase3/gradient_calibration_v1/gradient_step100_quality1p5_8x2.json`,
  SHA-256 `af3950c2c2f98b018d3cd3052470752c4d354e956e89b51a987bea3900f5788b`.

## Matched candidate result

A fresh seed-42 candidate used the same initial model-state hash
`7af6a0f21eb179b673d96290fcd41f6259f64efb512338d45a521208b8afa244`
as the baseline. It trained to step 100, was measured, resumed to step 200, and
was measured again. All four baseline/candidate reports use the same ordered
cohort IDs and contain finite results.

The trained candidate adapts away from the fixed-checkpoint prediction. Its
overall aggregate Q:L is 0.319 at step 100 and 0.177 at step 200, versus baseline
0.252 and 0.104. Localization remains dominant at every measured shared site;
the largest candidate step-200 ratio is edge-small FPN at 0.680.

| Cohort | Baseline @100 | Candidate @100 | Baseline @200 | Candidate @200 |
|---|---:|---:|---:|---:|
| Overall | 0.252 | 0.319 | 0.104 | 0.177 |
| General | 0.219 | 0.315 | 0.106 | 0.155 |
| Automotive | 0.247 | 0.327 | 0.139 | 0.312 |
| Fisheye | 0.248 | 0.340 | 0.177 | 0.291 |
| Edge-small | 0.265 | 0.397 | 0.295 | 0.559 |
| Small | 0.208 | 0.289 | 0.149 | 0.228 |
| Medium | 0.189 | 0.216 | 0.111 | 0.187 |
| Large | 0.287 | 0.325 | 0.116 | 0.211 |

The training-time EMA metrics at step 200 are nearly tied and slightly favor
the candidate: selection score +0.00010, R100/0.50 +0.00164, R100/0.75
+0.00178, and median matched IoU +0.00092. Those metrics use the training
decoder, so the decision uses a second frozen evaluation under the approved
HBB NMS/score policy `0.70/0.30`.

| Production-policy metric | Baseline | Candidate | Delta (percentage points) |
|---|---:|---:|---:|
| AR100 | 14.783% | 14.557% | -0.226 |
| R100/0.50 | 40.693% | 40.348% | -0.344 |
| R100/0.75 | 7.442% | 7.213% | -0.230 |
| Object coverage | 83.071% | 83.460% | +0.388 |
| Proposal object fraction | 50.115% | 49.361% | -0.754 |
| Trusted-background false | 2.229% | 2.318% | +0.089 |
| Unmatched | 47.833% | 48.011% | +0.178 |
| Duplicate/all | 6.395% | 6.288% | -0.107 |
| Proposals/image | 97.985 | 98.379 | +0.394 |

Candidate R100/0.50 changes by -1.36 points for general, -0.71 for automotive,
and +0.85 for fisheye. Small and thin recall fall by 0.78 and 0.70 points;
medium is unchanged at IoU 0.50, while large R100/0.75 falls by 4.13 points.
The repeated quad-control metrics are identical between reports.

## Decision and artifacts

The 1.5 candidate demonstrates no production-policy benefit and weakens priority
small/thin slices. The deltas are not treated as proof of harm from a single
seed and short run, but they do not justify changing the baseline. Retain
`objectness_weight: 1.0`; skip the upper 2.0 control because moving further in
the same direction lacks supporting evidence. No adaptive coefficients are
introduced.

Candidate SHA-256 values:

- step-100 checkpoint: `952c0884da924df839dd498348c22c6dee270d73742ea3abdda26a7a584fac66`;
- step-200 checkpoint: `219e23439366ad0963c6c0547474fa57f16eed7fe3c4e3476ffc05998ce2f828`;
- run contract: `20d389762bd17476f319d2d2046b3fe52f93f2bbe3521dbf035e1d102b6d770a`;
- step-100 gradient report: `446a6e5dd3a1cf77323ec41004adcef645b488a873dbf58ecd344ee03878a205`;
- step-200 gradient report: `f9150736779241ae42c0f42f90ad559a1c7a51590c0b72e061218ef9c44c194d`;
- baseline utility report: `eafb5a74ed2984d2b54d966b993715c7104d9c778c9576e6fcf968a622c56c9e`;
- candidate utility report: `8b64b600c30f8c292f07e50dff251635314ed74e828ba0a11a50add6717611bd`.
