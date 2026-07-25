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

## Phase 1 dataset preparation

This project prepares four Dataset Ninja datasets for object-detection experiments:
nuImages, WoodScape RGB Fisheye, BDD100K Images 100K, and COCO 2017.

Run the complete workflow from this directory:

```bash
uv run scripts/prepare_datasets.py
```

It downloads the datasets into `datasets/supervisely/`, discovers projects by
`meta.json`, converts them to detection projects, exports each project to COCO,
merges the exports into `datasets/coco_merged/`, validates the result with
`pycocotools`, and writes previews under `datasets/coco_merged/previews/`.

To reuse already-downloaded datasets:

```bash
uv run scripts/prepare_datasets.py --skip-download
```

The downloader now checks that each response is a real tar archive. If Dataset
Ninja's Dropbox link is temporarily disabled, download the Supervisely archive
from the corresponding Dataset Ninja page, place/extract it under
`datasets/supervisely/`, and rerun with `--skip-download`.
