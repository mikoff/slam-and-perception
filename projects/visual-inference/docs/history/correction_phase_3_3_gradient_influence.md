# Correction Phase 3.3 gradient influence

Status: complete. Instrumentation, final-checkpoint smoke calibration, and the
normalized-loss post-warm-up versus later-checkpoint comparison are verified.
No coefficient change was made in this step.

## Measurement contract

`scripts/measure_gradient_influence.py` accepts repeated `LABEL=PATH`
checkpoints and evaluates the same deterministic cohorts for each one. It runs
in FP32/eval mode and uses `autograd.grad`, leaving parameter gradients,
optimizer state, checkpoints, and model weights untouched.

The loss families are:

- quality: objectness plus centerness, before and after their coefficients;
- localization: CIoU plus LTRB, before and after their coefficients.

Gradient sites are all shared P3–P5 FPN outputs together and the C5 backbone
feature feeding the pyramid. Each site reports family norms, family-to-total
norm ratios, and weighted quality/localization cosine. Ratios need not sum to
one because the family gradients are vectors and may align or conflict.

Cohorts cover the configured source mixture, each domain, and the existing
transformed-area bands. Size slices retain only positives in the selected band,
move other positives to ignore, and remove background-quality supervision so
their gradients are object-size specific. Summaries include mean, median,
population variance, minimum, and maximum across batches.

## Final-checkpoint calibration

The schema was exercised on the Phase 3.1 step-400 EMA checkpoint with four
two-image batches per cohort. This checkpoint was trained with the pre-3.2
reduction and evaluated with the current normalized objective, so the result is
a tool/scale audit rather than a temporal training comparison.

| Cohort | Quality loss | Localization loss | FPN Q/total | FPN L/total | FPN cosine | C5 Q/total | C5 L/total | C5 cosine |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Overall | 0.0323 | 0.8756 | 0.173 | 0.973 | 0.096 | 0.144 | 0.967 | 0.267 |
| General | 0.0301 | 0.9012 | 0.174 | 0.972 | 0.086 | 0.126 | 0.966 | 0.197 |
| Automotive | 0.0175 | 0.9454 | 0.139 | 0.983 | 0.045 | 0.104 | 0.980 | 0.087 |
| Fisheye | 0.0389 | 1.4823 | 0.186 | 0.971 | 0.062 | 0.132 | 0.974 | 0.144 |
| Edge-small | 0.0495 | 1.8338 | 0.254 | 0.942 | 0.104 | 0.229 | 0.939 | 0.250 |
| Small | 0.0225 | 1.1502 | 0.111 | 0.991 | 0.023 | 0.088 | 0.996 | 0.022 |
| Medium | 0.0475 | 1.1486 | 0.182 | 0.954 | 0.125 | 0.117 | 0.950 | 0.264 |
| Large | 0.0393 | 0.8540 | 0.209 | 0.968 | 0.096 | 0.140 | 0.971 | 0.196 |

Localization dominates weighted gradient magnitude in every measured slice.
Median cosines are weakly positive, so the families are not broadly opposed at
this checkpoint. Individual automotive, edge-small, medium, and small batches
include small negative cosines at one site, so localized conflict exists.

The four-batch overall coefficient of variation is 0.60/0.34 for FPN
quality/localization norms and 0.68/0.40 at C5. Phase 3.4 must therefore use a
larger fixed cohort and robust medians rather than calibrating from this smoke
scale alone.

Artifact: `artifacts/phase3/gradient_influence_v1/final_4x2.json`, SHA-256
`9011ae97905c76657579f4d513ebc34825236e0b165bb4a1f4b688bdfc6f9ff1`.

## Normalized-loss temporal calibration

A contract-bound HBB P3-P5 seed-42 run used the unchanged approved coefficients
and the Phase 3.2 per-image reduction. The run stopped at step 100, measured the
raw model, then resumed to step 200 and measured the raw model again. Each slice
contains the same 16 ordered image IDs at both checkpoints, evaluated as eight
two-image batches. The external drive disconnected during the first resume;
replay from the preserved step-100 checkpoint reproduced every logged loss
exactly through the interruption point before completing.

The table reports the ratio of median weighted quality norm to median weighted
localization norm. A value below one means localization has the larger median
gradient. Cosine is the median quality/localization gradient cosine.

| Cohort | FPN Q:L @100 | FPN Q:L @200 | FPN cosine @100 | FPN cosine @200 | C5 Q:L @100 | C5 Q:L @200 | C5 cosine @100 | C5 cosine @200 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Overall | 0.242 | 0.134 | 0.030 | 0.043 | 0.263 | 0.082 | 0.092 | 0.111 |
| General | 0.217 | 0.126 | 0.027 | 0.043 | 0.221 | 0.089 | 0.090 | 0.069 |
| Automotive | 0.228 | 0.159 | 0.022 | 0.037 | 0.268 | 0.122 | 0.097 | 0.151 |
| Fisheye | 0.225 | 0.209 | 0.036 | 0.053 | 0.272 | 0.149 | 0.166 | 0.205 |
| Edge-small | 0.259 | 0.369 | 0.038 | 0.029 | 0.271 | 0.236 | 0.253 | 0.186 |
| Small | 0.211 | 0.168 | 0.032 | 0.042 | 0.206 | 0.134 | 0.119 | 0.144 |
| Medium | 0.195 | 0.153 | -0.013 | 0.042 | 0.183 | 0.081 | -0.070 | 0.054 |
| Large | 0.304 | 0.153 | 0.058 | 0.020 | 0.272 | 0.088 | 0.104 | 0.051 |

Localization remains dominant at both sites in every slice. Quality influence
generally falls relative to localization by step 200, except for edge-small FPN
features. Median cosines remain near zero to mildly positive; the medium-object
negative medians at step 100 become positive at step 200. Overall norm
coefficients of variation remain substantial (0.51-0.55 at step 100 and
0.58-0.72 at step 200), so Phase 3.4 should base any one-time coefficient
proposal on robust medians and verify the achieved ratio before comparison.

Artifacts and SHA-256 values:

- `hbb_p3_seed42/step_0100.pt`: `afcb91a74daa5af24881619f28f33b9a79b55be5c7a1e9d4d19de1f3c1c67966`;
- `hbb_p3_seed42/step_0200.pt`: `3bb70fcbc3e81ee5cd2198d2626f570f0bb6a4b3e4e10cb82fd61497958f8063`;
- `gradient_step100_8x2.json`: `22c5dde3d3b7146dc06de6329744928c5a950ea43358c65644644db18b406a39`;
- `gradient_step200_8x2.json`: `11dd0cc8854aa4758cb533eea4f1b5c980708ec11e14c02998c31410c70dd7f6`;
- `hbb_p3_seed42/run_contract.json`: `ab4d36c037eeab7f1a77ebde24e1a2d1bb8ae9c3359531fe747d71e453fefbef`.

The owner subsequently approved Phase 3.4. Its matched coefficient candidate
was measured and rejected, leaving this unchanged-weight baseline selected.
