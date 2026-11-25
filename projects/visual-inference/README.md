# Visual Inference

Edge-targeted **MobileNetV4 + P2LitePAN** open-vocabulary object detectors (Horizontal Bounding Box & Oriented Quadrilateral) distilled from **OWLv2-Large** (`google/owlv2-large-patch14-ensemble`) into a `384x384`, `7.4M–10.2M`-parameter runtime with zero runtime text encoder.

For architectural contracts and empirical history, see [`ARCHITECTURE.md`](ARCHITECTURE.md).

---

## Verified Benchmark Summary (`holdout_210` & `cohort_val_800`)

Evaluated on sanitized annotations across 7 domains (`bdd100k_images_100k`, `nuimages`, `woodscape_rgb_fisheye`, `coco_2017`, `ade20k`, `open_images_subset`, plus unseen holdout sources) using `p=1.25, top_k=1000, max_detections=100`:

| Model | Input | Opt-B Params | `R@100@50` (Val / Holdout) | Agnostic `AP50` | Opt-B Seen Top-1 (Val / Holdout) | Opt-B `mAP@50` | Zero-Shot-4 (4w / Top-5) | Speed (RTX 3060 / 4-thr CPU) |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| **OWLv2-Large Teacher** *(Core-27 Frozen)* | `1008×1008` | 437.61M | `—` / **`70.0%`** | `—` / **`20.5%`** | `—` / **`86.9%`** | `—` / **`31.5%`** | `—` / `96.2%` | `282.55 ms` / `—` |
| **OWLv2-30k HBB `v3`** *(Core-27 Adapted)* | `384×384` | **10.18M** | `53.9%` / `51.6%` | `13.8%` / `11.7%` | `84.0%` / **`86.5%`** | `15.1%` / **`16.6%`** | `95.4%` / `94.7%` (`97.4%` AB) | `22.23 ms` / `247.3 ms` *(eager)* |
| **OWLv2-30k Quad** *(Core-27 Adapted)* | `384×384` | **10.18M** | `51.9%` / `50.6%` | **`17.5%`** / **`16.9%`** | `83.8%` / `83.6%` | `14.6%` / `12.1%` (`15.5%` AB) | `92.7%` / **`97.3%`** (**`100%`**) | `22.10 ms` / `155.8 ms` *(eager)* |
| **Legacy SigLIP-8k HBB Opt-B** | `384×384` | 10.18M | `43.2%` / `43.1%` | `—` | `70.4%` / `71.3%` | `7.7%` / `12.9%` | `60.2%` / `61.5%` | `—` / `288.7 ms` *(eager)* |

---

## Environment & Reproducibility Pinning

1. **Python & Packages**: Pinned via [`pyproject.toml`](pyproject.toml) (including `torch>=2.7.0`, `torchvision>=0.22.0`, `transformers>=4.40.0`, `timm>=1.0.27,<2`, `shapely>=2.1,<3`) and [`uv.lock`](uv.lock):
   ```bash
   cd projects/visual-inference
   uv sync --frozen --group dev
   ```
2. **Teacher Revision**: `OWLV2_MODEL_ID = "google/owlv2-large-patch14-ensemble"` is pinned to commit SHA `95e26936e865f87db1742128404b3c035d47d89d` (`OWLV2_MODEL_REVISION` in [`student_detector/owlv2_teacher.py`](student_detector/owlv2_teacher.py)).
3. **Holdout Isolation Lockfile**: Exact 210 holdout image IDs, SHA256 hashes, and excluded 800-image validation sets are tracked in [`configs/holdout_210_ids.json`](configs/holdout_210_ids.json).

---

## End-to-End Reproduction Pipeline

All commands below assume working directory `projects/visual-inference`.

### 1. Prepare Datasets, Enriched GT Index & Holdout Manifest
Prepare the base datasets using [`dataset-pipeline/README.md`](dataset-pipeline/README.md), ingest auxiliary sets (`ingest_ade20k.py`, `ingest_open_images.py`, `ingest_roboflow_ev_parking.py`), and generate the leak-audited holdout manifest (`artifacts/dashboard_holdout_8k_8k_full.json`):
```bash
uv run python scripts/prepare_dashboard_holdout.py \
  --cohort-size 8000 \
  --output artifacts/dashboard_holdout_8k_8k_full.json \
  --include-new-sources
```

### 2. Stage-1/Stage-2 & SigLIP Warm-Start Baselines (`sota_8k_hbb`, `sota_8k_quad`)
Run Stage-1/2 proposal pre-training (`COHORT_SIZE=8000 ./scripts/run_two_stage_pipeline.sh all`), then warm-start the 128-ch `P2LitePAN` + dual-head checkpoints:
```bash
uv run python scripts/train_open_vocab_sota.py --geometry hbb --output-dir artifacts/sota_8k_hbb
uv run python scripts/train_open_vocab_sota.py --geometry quad --output-dir artifacts/sota_8k_quad
```

### 3. Build OWLv2-30k Unified Cache & Bollard/Pillar Patch
Build the leak-free 12k cache + native OWLv2 vocabulary banks, scale to 30k balanced/scarce-class images, and patch bollards (`pole`) & columns (`pillar`):
```bash
# 3a. Initial 12k cache + OWLv2 vocabulary banks -> artifacts/sota_open_vocab/owlv2_vocabulary_bank_*.pt
uv run python scripts/build_owlv2_teacher_cache.py \
  --num-images 12000 \
  --output-db artifacts/cache/owlv2_teacher_12k.sqlite \
  --patch-wide-pedestrians

# 3b. Scale 12k -> 30k (preserving 12k via sqlite3.backup)
uv run python scripts/build_owlv2_teacher_cache.py \
  --num-images 30000 \
  --seed-from-db artifacts/cache/owlv2_teacher_12k.sqlite \
  --output-db artifacts/cache/owlv2_teacher_30k.sqlite

# 3c. Targeted bollard/column patch across non-ADE20K driving scenes -> 30k patched DB
uv run python scripts/patch_owlv2_cache_bollards_pillars.py \
  --source-db artifacts/cache/owlv2_teacher_30k.sqlite \
  --output-db artifacts/cache/owlv2_teacher_30k_patched.sqlite
```

### 4. Train OWLv2-30k HBB `v3` & Oriented Quad Models
```bash
# HBB Stage A: 12 epochs on 12k cache (from sota_8k_hbb)
uv run python scripts/train_owlv2_hbb.py \
  --cache-db artifacts/cache/owlv2_teacher_12k.sqlite \
  --warmstart-ckpt artifacts/sota_8k_hbb/checkpoints/sota_joint_best.pt \
  --output-dir artifacts/owlv2_12k_hbb \
  --epochs 12 --batch-size 16 --workers 4

# HBB Stage B: 6-epoch 30k scaling
uv run python scripts/train_owlv2_hbb.py \
  --cache-db artifacts/cache/owlv2_teacher_30k.sqlite \
  --resume-owlv2-ckpt artifacts/owlv2_12k_hbb/checkpoints/owlv2_hbb_best.pt \
  --output-dir artifacts/owlv2_30k_hbb \
  --epochs 6 --lr-scale 0.6 --batch-size 16 --workers 4

# HBB Stage C (v3): 3-epoch refinement on patched 30k DB (raw IoU, 1.0x pillar sampling)
uv run python scripts/train_owlv2_hbb.py \
  --cache-db artifacts/cache/owlv2_teacher_30k_patched.sqlite \
  --resume-owlv2-ckpt artifacts/owlv2_30k_hbb/checkpoints/owlv2_hbb_best.pt \
  --output-dir artifacts/owlv2_30k_hbb_v3 \
  --epochs 3 --lr-scale 0.45 --batch-size 16 --workers 4

# Oriented Quad: 6-epoch training directly on patched 30k DB (from sota_8k_quad)
uv run python scripts/train_owlv2_quad.py \
  --cache-db artifacts/cache/owlv2_teacher_30k_patched.sqlite \
  --warmstart-ckpt artifacts/sota_8k_quad/checkpoints/sota_joint_best.pt \
  --output-dir artifacts/owlv2_30k_quad \
  --epochs 6 --batch-size 16 --workers 4
```

### 5. Benchmark & Generate Joint Unseen Dashboard
```bash
# Core-27 HBB v3 benchmark (p=1.25, top_k=1000, max_detections=100)
uv run python scripts/benchmark_open_vocab_options.py \
  --only-hbb --skip-legacy \
  --hbb-sota-ckpt artifacts/owlv2_30k_hbb_v3/checkpoints/owlv2_hbb_best.pt \
  --vocab-37 artifacts/owlv2_30k_hbb_v3/owlv2_adapted_vocab_core27.pt \
  --vocab-41 artifacts/owlv2_30k_hbb_v3/owlv2_adapted_vocab_31classes_core27_zeroshot.pt \
  --vocab-distractors artifacts/sota_open_vocab/owlv2_vocabulary_bank_distractors.pt \
  --decoder-top-k 1000 --decoder-max-detections 100 \
  --small-box-min-side 16.0 --small-box-score-power 1.25 \
  --output-json artifacts/owlv2_30k_hbb_v3/benchmark_30k_v3_core27_adapted_p125.json

# Core-27 Oriented Quad benchmark (p=1.25, top_k=1000, max_detections=100)
uv run python scripts/benchmark_open_vocab_options.py \
  --only-quad --skip-legacy \
  --quad-sota-ckpt artifacts/owlv2_30k_quad/checkpoints/owlv2_quad_best.pt \
  --vocab-37 artifacts/owlv2_30k_quad/owlv2_adapted_vocab_core27.pt \
  --vocab-41 artifacts/owlv2_30k_quad/owlv2_adapted_vocab_31classes_core27_zeroshot.pt \
  --vocab-distractors artifacts/sota_open_vocab/owlv2_vocabulary_bank_distractors.pt \
  --decoder-top-k 1000 --decoder-max-detections 100 \
  --small-box-min-side 16.0 --small-box-score-power 1.25 \
  --output-json artifacts/owlv2_30k_quad/benchmark_30k_quad_core27_adapted_p125.json

# Head-to-head OWLv2-Large Teacher vs Student v3 on holdout_210
uv run python scripts/benchmark_owlv2_teacher_vs_student.py \
  --output-json artifacts/owlv2_30k_hbb_v3/benchmark_teacher_vs_student_holdout210.json

# Joint Unseen HBB v3 + Oriented Quad Interactive Dashboard (0/236 train overlap check)
uv run python scripts/visualize_open_vocab_ab_dashboard.py \
  --hbb-sota-ckpt artifacts/owlv2_30k_hbb_v3/checkpoints/owlv2_hbb_best.pt \
  --quad-sota-ckpt artifacts/owlv2_30k_quad/checkpoints/owlv2_quad_best.pt \
  --vocab-37 artifacts/owlv2_30k_hbb_v3/owlv2_adapted_vocab_37classes.pt \
  --vocab-41 artifacts/owlv2_30k_hbb_v3/owlv2_adapted_vocab_41classes_zeroshot.pt \
  --vocab-core27 artifacts/owlv2_30k_hbb_v3/owlv2_adapted_vocab_core27.pt \
  --vocab-core31 artifacts/owlv2_30k_hbb_v3/owlv2_adapted_vocab_31classes_core27_zeroshot.pt \
  --quad-vocab-37 artifacts/owlv2_30k_quad/owlv2_adapted_vocab_37classes.pt \
  --quad-vocab-41 artifacts/owlv2_30k_quad/owlv2_adapted_vocab_41classes_zeroshot.pt \
  --quad-vocab-core27 artifacts/owlv2_30k_quad/owlv2_adapted_vocab_core27.pt \
  --quad-vocab-core31 artifacts/owlv2_30k_quad/owlv2_adapted_vocab_31classes_core27_zeroshot.pt \
  --vocab-distractors artifacts/sota_open_vocab/owlv2_vocabulary_bank_distractors.pt \
  --benchmark-json artifacts/owlv2_30k_hbb_v3/benchmark_30k_v3_core27_adapted_p125.json \
  --quad-benchmark-json artifacts/owlv2_30k_quad/benchmark_30k_quad_core27_adapted_p125.json \
  --assert-no-overlap-cache artifacts/cache/owlv2_teacher_30k_patched.sqlite \
  --decoder-top-k 1000 --decoder-max-detections 100 \
  --small-box-min-side 16.0 --small-box-score-power 1.25 \
  --quad-decoder-top-k 1000 --quad-decoder-max-detections 100 \
  --quad-small-box-min-side 16.0 --quad-small-box-score-power 1.25 \
  --output-html artifacts/owlv2_30k_quad/dashboard_open_vocab_a_b_ab.html
```

---

## Verification & Legacy Docs

```bash
uv lock --check
uv run ruff check student_detector/ scripts/ tests/
uv run pytest
```

Architecture & historical docs: [`docs/code_audit.md`](docs/code_audit.md), [`docs/open_vocabulary_embedding_guide.md`](docs/open_vocabulary_embedding_guide.md), [`docs/visual_inference_system_audit.md`](docs/visual_inference_system_audit.md), [`docs/history/phase2.md`](docs/history/phase2.md), [`docs/history/phase3.md`](docs/history/phase3.md), [`docs/history/proposal_detector_roadmap.md`](docs/history/proposal_detector_roadmap.md), and [`docs/history/model_inspection.md`](docs/history/model_inspection.md).
