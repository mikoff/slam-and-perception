# Inspecting the quad proposal detector

The maintained model is a quadrilateral proposal detector. The old Phase-2
inspection CLI was removed, so this guide focuses on the current model stages
and diagnostics.

## Model path

| Stage | Symbol | Diagnostic meaning |
| --- | --- | --- |
| Backbone | C3/C4/C5 | Fine-to-coarse features at strides 8, 16, and 32. |
| Neck | P3/P4/P5 | Fused feature maps consumed by the quad head. |
| Head | quality logits | Proposal ranking before sigmoid and policy thresholding. |
| Head | corner offsets | Eight signed values describing four corners relative to each grid point. |
| Decoder | dense quads | Every decoded grid-location quad before validity filtering and NMS. |
| Decoder | final proposals | Valid, canonicalized, polygon-NMS-filtered proposals. |
| Training | assignments/losses | Representability, matched locations, and quality/localization errors. |

The implementation is split across student_detector/backbone.py,
student_detector/neck.py, student_detector/head.py,
student_detector/quad_decoder.py, student_detector/quad_assigner.py, and
student_detector/quad_losses.py. The complete checkpoint and stage map is in
[code_audit.md](code_audit.md).

## Checkpoint diagnostics

Run the maintained detector tests first:

    uv run --group dev pytest -q tests/student_detector

For a raw-versus-EMA geometry report on a configured validation set:

    uv run python scripts/diagnose_quad_checkpoint.py \
      --config configs/phase3.yaml \
      --checkpoint artifacts/phase3/checkpoints/best_ema.pt \
      --output artifacts/diagnostics/quad_checkpoint.json \
      --device cpu

For prediction overlays and validation metrics:

    uv run python scripts/visualize_quad_validation.py \
      --config configs/phase3.yaml \
      --checkpoint artifacts/phase3/checkpoints/best_ema.pt \
      --output-dir artifacts/phase3/validation_visuals \
      --device cpu

Use the Phase 4 best_ema.pt as the A/B control described in
code_audit.md. Do not use last.pt for ordinary inference; it is a
training-resume state.

## Inspecting tensors in Python

The package exports the canonical quad API:

    import torch
    from student_detector import (
        QuadInferenceDecoder,
        QuadProposalDetector,
        decode_dense_quad_output,
    )

    model = QuadProposalDetector(pretrained_backbone=False).eval()
    images = torch.randn(1, 3, 384, 384)

    with torch.inference_mode():
        features = model.forward_features(images)
        output = model.head(features)
        dense_quads, dense_scores = decode_dense_quad_output(output)
        detections = QuadInferenceDecoder(max_proposals=100)(
            output, image_size=(384, 384)
        )

features contains P3/P4/P5 tensors with spatial sizes 48x48, 24x24, and
12x12 for a 384x384 input. output.quality and output.corner_offsets preserve
the same pyramid-level structure.

Inspect the stages in this order:

1. P3/P4/P5: determine whether localization detail and context are present.
2. Quality logits: determine whether the model ranks likely object locations.
3. Dense decoded quads: determine whether offsets produce sensible geometry.
4. Validity filtering and canonicalization: determine whether geometry is finite,
   convex, clockwise, and inside the image.
5. Polygon NMS: determine whether duplicate proposals are suppressed too
   aggressively or too weakly.

This ordering separates feature, ranking, localization, geometry, and
suppression failures.

## Optional rectification

student_detector/rectification.py contains a tested homography utility for a
future crop/rectification stage. It is intentionally outside detector forward,
training, and deployment. Keep it only if downstream quad crops are part of the
planned system; otherwise it can be archived together with its test.

## Common interpretation mistakes

- A high quality logit is a ranking signal, not a calibrated probability.
- Dense decoded quads include invalid and duplicate candidates by design.
- The selected score threshold and polygon-NMS IoU are external operating
  policy, not necessarily embedded in the model checkpoint.
- Random-weight features only validate tensor plumbing; they do not demonstrate
  learned localization.
