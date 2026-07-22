# Visual Inference Dataset Preparation

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
