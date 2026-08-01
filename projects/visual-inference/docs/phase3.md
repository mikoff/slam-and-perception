# Phase 3: Proposal Training

Phase 3 is the current class-agnostic proposal baseline. It trains a small
MobileNetV4 detector to rank foreground regions and regress horizontal boxes.
This baseline is retained as a measurable localization scaffold while the
future open-vocabulary architecture is being prepared.

## Current Model

```text
RGB image
  -> MobileNetV4-Conv-Medium
  -> three-level 96-channel Lite FPN
  -> shared class-agnostic head
       -> objectness
       -> LTRB box distances
       -> optional centerness loss
```

Assignment, loss construction, decoding, clipping, and NMS remain outside the
exported model. The raw model output is fixed for P3, P4, and P5 and contains
objectness, box distances, and centerness tensors.

The current code does not implement DQCO, SigLIP, region embeddings, text
embeddings, distillation, polygon NMS, or quantization. The discarded
HBB-relative quad experiment is not part of this project.

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
links. No Waymo, Argoverse, Objects365, DOTA, FAIR1M, Cityscapes, Mapillary, or
SynWoodScape source is configured.

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
uv run python scripts/train_phase3.py --config configs/phase3.yaml
```

Visualize HBB proposals with a checkpoint:

```bash
uv run python scripts/visualize_phase3.py \
  --checkpoint path/to/checkpoint.pt \
  --count 12
```

## Cleanup Boundary

The active detector surface consists of `student_detector/`,
`scripts/train_phase3.py`, `scripts/audit_phase3_data.py`,
`scripts/visualize_phase3.py`, the Phase-2 inspection utilities, and
`configs/phase3.yaml`. Colab/XLA launchers, one-off accelerator diagnostics,
quad fitting experiments, and generated artifacts are intentionally excluded.
