# Correction Phase 3.2 loss normalization

Status: complete. Implementation and verification passed; the owner approved
the subsequent normalized calibration run and Phase 3.3 diagnostics.

## Scope

This step changes only the selected HBB P3–P5 training objective's reduction.
QFL, CIoU, stride-normalized SmoothL1 LTRB, and their coefficients are unchanged.
The retained quad control is not being retrained, so its objective is unchanged.
Polygon-sample normalization is therefore not applicable to this HBB step.

## Reduction contract

For image `i`, every localization component is reduced as:

```text
L_i = sum_j(weight_ij * loss_ij) / max(sum_j(weight_ij), 1)
L_batch = mean_i(L_i)
```

Uniform CIoU and LTRB use one assignment weight per positive point. The optional
centerness-weighted CIoU path uses the exact sum of centerness weights. Images
without positive assignments contribute a graph-connected zero.

Quality is reduced independently for positive, trusted-background, and weak
states within each image, then the three state terms are added and averaged over
images. Weak point weights remain coefficients: they are not cancelled by the
denominator. Ignore points have zero effective weight and are excluded from the
objective. Counterfactual raw weak/ignore values remain diagnostics only.

Diagnostics report, for every component and state:

- raw and coefficient-weighted loss contributions;
- point count and total effective weight;
- present, active, and empty image counts;
- source/domain contributions that sum back to the batch component.

This sample-first reduction makes equal-sized gradient-accumulation partitions
equivalent to the corresponding full batch and prevents images with more
annotations or supervised feature points from dominating solely by density.

## Verification

| Gate | Result |
|---|---|
| Duplicate an identical sample | loss components unchanged |
| Full batch versus two equal microbatches | loss and one-step parameters match within `2e-6` |
| Empty positive/trusted states | finite graph-connected zeros and explicit empty counts |
| Weak weight zero | 12,017 production points present, zero active/effective contribution |
| Ignore exclusion | 39 production points present, zero active/effective contribution |
| Component reconstruction | exact on the production audit batch |
| State reconstruction | exact on the production audit batch |
| Source/domain reconstruction | maximum error `3.73e-9` |
| Detector regression suite | 126 passed |

The production audit used eight deterministic source-balanced images and the
Phase 3.1 final EMA checkpoint on the RTX 3060. The normalized batch values were
total `0.908193`, objectness `0.030270`, CIoU `0.361911`, and LTRB `0.308199`.

These values are a semantic scale check, not a model-quality comparison. The
normalization intentionally changes loss scale, so Phase 3.1 training metrics
must not be compared numerically with a future post-3.2 run as though only model
quality changed.
