# Visual Inference

This project develops an embedded open-vocabulary detector in phases.

## Phase 2 student detector

The `student_detector` package contains the MobileNetV4-Conv-Medium backbone,
96-channel Lite FPN, shared class-agnostic detector head, ATSS assigner, and
inference decoder. The implementation notes and commands are in
[`docs/phase2.md`](docs/phase2.md).

To explore layer shapes, execution graphs, activation maps, and training
distributions, see [`docs/model_inspection.md`](docs/model_inspection.md).

Run its completion checks from this directory:

```bash
uv run --group dev pytest -q tests/student_detector
uv run python scripts/verify_phase2.py
```

## Phase 3 proposal training

Phase 3 trains the Phase-2 HBB detector as a class-agnostic proposal generator.
Its data policy, ATSS/loss rationale, commands, measured audits, and remaining
gates are documented in [`docs/phase3.md`](docs/phase3.md).

Useful entry points:

```bash
uv run pytest -q tests
uv run python scripts/audit_phase3_data.py --samples 300
uv run python scripts/train_phase3.py --overfit-images 50 --epochs 20
uv run python scripts/train_phase3.py --config configs/phase3.yaml
```

## Phase 1 dataset preparation

The nested `dataset-pipeline` project prepares four Dataset Ninja datasets:
nuImages, WoodScape RGB Fisheye, BDD100K Images 100K, and COCO 2017.

Its environment and commands are documented in
[`dataset-pipeline/README.md`](dataset-pipeline/README.md). Run its tests with:

```bash
cd dataset-pipeline
uv run pytest -q
```
