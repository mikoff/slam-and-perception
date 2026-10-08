# Visual Inference

This project develops an embedded open-vocabulary detector in phases.

The current milestone stops at a retrained, evaluated proposal detector; SigLIP
work is deferred. The [roadmap](docs/proposal_detector_roadmap.md) maps the original
project phases to the correction/retraining work packages and defines the gates.
The current code audit and checkpoint map are in
[`docs/code_audit.md`](docs/code_audit.md).

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
```

## Phase 3 proposal training

Phase 3 trains HBB and quad class-agnostic proposal detectors.
Its data policy, ATSS/loss rationale, commands, measured audits, and remaining
gates are documented in [`docs/phase3.md`](docs/phase3.md).

Useful entry points:

```bash
uv run pytest -q tests
uv run python scripts/audit_phase3_data.py --samples 300
uv run python scripts/train_phase3.py --overfit-images 50 --epochs 20
uv run python scripts/train_phase3.py --config configs/phase3.yaml
```

## Two-Stage Proposal Training Pipeline (OWLv2 Paradigm)

To produce tight object proposals and calibrated objectness for downstream open-vocabulary vision-language models (SigLIP), the student detector uses a two-stage training paradigm inspired by OWLv2:

1. **Stage 1: Self-Supervised Discovery Pre-Training (15 Epochs)**
   - Pre-trains on 10,000 multi-domain images using SAM discovery pseudo-labels (`SAMStudentHBBDataset`).
   - Maximizes candidate recall across broad scenes (NuImages, BDD100k, WoodScape Fisheye, COCO 2017).
2. **Stage 2: Canonical Ground-Truth Fine-Tuning (5 Epochs)**
   - Fine-tunes pre-trained weights strictly on verified human ground-truth boxes (`HumanGTHBBDataset`).
   - Uses joint **Scale-Invariant CIoU** (`box_weight=2.0`) + **Smooth-$\mathcal{L}_1$ Distance Loss** (`ltrb_weight=0.5`) to tighten baggy proposal envelopes.
   - Retains `weak_negative_weight=0.15` to protect open-world recall for unseen categories.

### One-Command Automated Execution

```bash
# Run full pipeline: Stage 1 pre-training, Stage 2 fine-tuning, and comparison dashboard
./scripts/run_two_stage_pipeline.sh all

# Or run individual stages:
./scripts/run_two_stage_pipeline.sh stage1     # 15-ep SAM pre-training
./scripts/run_two_stage_pipeline.sh stage2     # 5-ep Human GT fine-tuning
./scripts/run_two_stage_pipeline.sh dashboard  # Interactive comparison audit
```

### Measured Benchmark Impact

| Metric | Stage 1 (SAM Baseline) | Stage 2 (Human GT FT) | Delta / Gain | Significance |
|---|---|---|---|---|
| **Pure Human GT Recall@100@50** | 15.97% | **19.38%** | **+3.41% (+21.3%)** | Primary object discovery rate |
| **Pure Human GT Recall@100@75** | 1.03% | **1.27%** | **+23.3% relative** | Envelope tightness on true objects |
| **Pure Human GT AP@50** | 1.19% | **1.87%** | **+57.1% relative** | Confidence calibration & ranking |
| **Smooth-$\mathcal{L}_1$ Loss** | 0.3557 | **0.1757** | **-50.6% (halved)** | Boundary coordinate error penalty |

### Interactive Validation Dashboards

- [Stage-2 Before & After Comparison Dashboard](artifacts/finetune_10k_hbb/dashboard_finetune_comparison.html): Side-by-side inspection of 60 multi-domain validation images comparing ground-truth boxes (green), baseline proposals (cyan), and fine-tuned proposals (purple).
- [Master 10k Visualization Portal](artifacts/visualization_dashboard.html): Unified dashboard covering HBB, QUAD, and Stage-2 fine-tuning results.

## Cloud Training Pipeline

For single-GPU automated training on Packet.ai via GitHub Actions, see:
- [`docs/cloud_training_runbook.md`](docs/cloud_training_runbook.md) (Complete Step-by-Step Instructions)

## Phase 1 dataset preparation

The nested `dataset-pipeline` project prepares four Dataset Ninja datasets:
nuImages, WoodScape RGB Fisheye, BDD100K Images 100K, and COCO 2017.

Its environment and commands are documented in
[`dataset-pipeline/README.md`](dataset-pipeline/README.md). Run its tests with:

```bash
cd dataset-pipeline
uv run pytest -q
```
