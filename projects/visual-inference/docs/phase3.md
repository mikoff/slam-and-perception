# Phase 3: Proposal Training

Phase 3 is the current class-agnostic proposal baseline. It trains a small
MobileNetV4 detector to rank foreground regions and regress quadrilateral boxes.
This baseline is retained as a measurable localization scaffold while the
future open-vocabulary architecture is being prepared.

## Current Model

```text
RGB image
  -> MobileNetV4-Conv-Medium
  -> three-level 96-channel LiteFPN / AttnResLiteFPN (Dynamic Softmax Depth Selection)
  -> shared class-agnostic quad head
       -> quality score
       -> unmasked eight-coordinate corner offsets
```

Assignment, loss construction, decoding, clipping, and NMS remain outside the
exported model. The raw model output is fixed for P3, P4, and P5 and contains
quality and corner offset tensors.

The current code does not implement SigLIP, region embeddings, text
embeddings, distillation, or INT8 quantization.

## Data Scope

Only datasets already present in the workspace are supported:

- COCO 2017
- nuImages
- BDD100K Images 100K
- WoodScape RGB Fisheye

The dataset pipeline lives in
[`dataset-pipeline/`](../dataset-pipeline/). It converts the four local
Supervisely exports into a unified COCO view, preserves source metadata, keeps
raw polygon annotations under the data workspace, and validates final image
links.

The canonical mixture is 50% COCO, 18% nuImages, 12% BDD100K, and 20%
WoodScape. Labels are used for source-aware supervision and ignore handling;
the detector does not train closed-set class logits.

## Commands

Run from this directory:

```bash
uv sync --frozen
uv run pytest -q tests
uv run python scripts/verify_phase2.py
uv run python scripts/audit_phase3_data.py --samples 300
uv run python scripts/train_quad_proposals.py --config configs/phase3_attnres.yaml
```

Compare open-world quad proposal checkpoints:

```bash
uv run python scripts/compare_open_world_proposals.py \
  --config configs/phase3_attnres.yaml \
  --hbb-checkpoint artifacts/phase3/open_world/hbb_control_v1/last.pt \
  --quad-checkpoint artifacts/phase3/open_world/quad_candidate_v12_attnres/last.pt \
  --output artifacts/phase3/open_world/comparison_v12_attnres/report.json
```

## Cleanup Boundary

The active detector surface consists of `student_detector/`,
`scripts/train_quad_proposals.py`, `scripts/audit_phase3_data.py`,
`scripts/compare_open_world_proposals.py`, the Phase-2 inspection utilities, and
`configs/phase3_attnres.yaml`.
